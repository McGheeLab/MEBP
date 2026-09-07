"""XYTopSpeed.py — measure the XY stage's TRUE top speed, with no camera and no GUI.

WHY THIS EXISTS
───────────────
``XYStage.set_speed_mm_s`` converts a requested mm/s into a Prior ``SMS`` percentage
by dividing by the stage's top speed::

    SMS% = requested_um_s / top_speed_um_s * 100

so that denominator decides whether a commanded 1 mm/s really is 1 mm/s. Until this
module existed the measurement lived only inside
``gui/pages/workflows/timing_calibration_workflow.py::_run_top_speed``, tangled with the
motion-detector widgets, the plot and the Qt bridge — so the one-click calibration could
not run it. It could only *read* a previously stored value, and when that value was
absent (which is the normal state of a fresh machine) it aborted telling the operator to
go and press a different button. That loop is the defect this module closes.

MEASUREMENT
───────────
At FULL speed (``SMS,100``), sweep single-move DISTANCES and time each move to
stillness::

    t(d) = d / v_top + overhead

The constant accel/decel ramp and the still-detection window fall into the *intercept*,
so they cancel: the **slope** of time-vs-distance is ``1 / v_top``. Five distances × two
repeats is enough — two independent runs on ME3B_01 fitted 6.036 and 6.006 mm/s, a 1.6 %
agreement, so the repeats mostly buy confidence rather than precision.

Motion is detected from the stage's OWN reported position (``EncoderMotion``), not a
camera: at rest the Prior reports a constant position (≈0 µm/poll) and while moving it
changes by tens–hundreds of µm/poll. That is a far cleaner still/moving signal than
frame differencing and is immune to focus, texture and lighting.

DIFFERENCES FROM THE GUI VERSION THIS REPLACES
──────────────────────────────────────────────
* **Absolute stage µm**, not zero-ref mm, so it shares one coordinate frame with
  ``XYAutoCalibration.check_fits`` / ``center_stage``. The GUI version centred about the
  envelope midpoint but drove the sweep about the operator's start location.
* **The envelope is checked.** The GUI version issued a 10 mm sweep with no
  ``check_fits`` call. A clamped move does not fail — it fits a *shorter* distance into
  the same time and reports a **low** top speed, which then makes every commanded speed
  run proportionally FAST. Here the distance is shrunk to fit, or the run refuses.
* **Directions alternate**, so the return leg *is* the next measurement rather than dead
  time between measurements, and axis asymmetry shows as spread rather than bias.

SAFETY
──────
XY only — this module never touches Z, and the caller must already have the needle
retracted (pass ``safe_z_mm`` and centring will retract before travelling). Every exit
path — success, no-motion, exception, cooperative stop — stops the stage, returns it to
where it started and restores the stage speed, and the position poller is suspended for
the duration so it cannot contend for the serial port.
"""

from __future__ import annotations

import json
import logging
import math
import time
from datetime import datetime
from pathlib import Path

from SupportClasses.XYDeadTime import fit_line

logger = logging.getLogger(__name__)

#: Sweep shape. Five distances is what the fit needs; more buys little because the
#: residual is dominated by the still-detection window, which is common to every point.
DEFAULT_N_POINTS = 5
DEFAULT_REPEATS = 2
DEFAULT_MAX_DIST_MM = 10.0
DEFAULT_STILL_WINDOW_S = 0.35

#: Per-move wall-clock bound. A 10 mm move at a pathologically slow 0.5 mm/s is 20 s.
DEFAULT_MOVE_TIMEOUT_S = 25.0
#: How long to wait for the stage to go quiet before a timed move starts.
DEFAULT_QUIET_TIMEOUT_S = 15.0


class EncoderMotion:
    """Stillness detector driven by the STAGE'S OWN reported position.

    ``metric()`` = |Δposition| in µm since the previous call. At rest the Prior's
    reported position is constant (≈0 µm/poll); while moving it changes by tens to
    hundreds of µm/poll. Each call is one direct ``get_xy_position(cached=False)``
    query, so the position poller must be suspended for the duration or these queries
    contend for the serial port.

    Moved verbatim (bar the name) from the timing page's ``_EncoderMotion``.
    """

    #: Physical still/moving thresholds for the position-delta metric (µm per poll).
    #: The metric is an ABSOLUTE distance, so these are fixed — no scene calibration is
    #: needed, unlike a camera's arbitrary units. (That scene calibration is exactly
    #: what made the old settle sweep fail: on a short segment its "peak" caught one
    #: poll spanning the whole move and set the threshold above every subsequent
    #: per-poll motion, so every measurement timed out.)
    STILL_UM = 3.0
    MOTION_UM = 60.0     # nominal "clearly moving" peak (display/log only)

    def __init__(self, controller):
        self._ctrl = controller
        self._prev = None

    def fixed_threshold(self):
        """``(floor, peak, still_thresh)`` in µm/poll — used INSTEAD of a
        scene-relative calibration for this physical detector."""
        return (0.0, self.MOTION_UM, self.STILL_UM)

    def available(self) -> bool:
        c = self._ctrl
        return (c is not None and getattr(c, "is_xy_connected", False)
                and hasattr(c, "get_xy_position"))

    def reset(self) -> None:
        self._prev = None

    def _read(self):
        try:
            p = self._ctrl.get_xy_position(cached=False)
        except Exception:
            return None
        if not p or p[0] is None or p[1] is None:
            return None
        return (float(p[0]), float(p[1]))

    def metric(self):
        p = self._read()
        if p is None:
            return None
        if self._prev is None:          # first sample after reset — no delta yet
            self._prev = p
            return None
        d = math.hypot(p[0] - self._prev[0], p[1] - self._prev[1])
        self._prev = p
        return d

    def frame_count(self):
        return None


# ── still/moving primitives (were private methods on the GUI page) ────────

def _stopped(stop_evt) -> bool:
    return bool(stop_evt is not None and stop_evt.is_set())


def dwell(tracker, dur: float, *, stop_evt=None, on_sample=None) -> list:
    """Sample the motion metric for ``dur`` seconds; return the values seen."""
    out = []
    end = time.monotonic() + dur
    while time.monotonic() < end and not _stopped(stop_evt):
        m = tracker.metric()
        if m is not None:
            out.append(m)
            if on_sample:
                try:
                    on_sample(time.monotonic(), m)
                except Exception:
                    pass
        time.sleep(0.025)
    return out


def wait_quiet(tracker, thresh: float, window: float, timeout: float, *,
               stop_evt=None, on_sample=None) -> bool:
    """Block until the metric stays below ``thresh`` for ``window`` seconds — i.e.
    the stage has settled. Returns False on timeout."""
    t0 = time.monotonic()
    below_since = None
    while not _stopped(stop_evt) and (time.monotonic() - t0) < timeout:
        now = time.monotonic()
        m = tracker.metric()
        if m is not None:
            if on_sample:
                try:
                    on_sample(now, m)
                except Exception:
                    pass
            if m < thresh:
                if below_since is None:
                    below_since = now
                elif now - below_since >= window:
                    return True
            else:
                below_since = None
        time.sleep(0.025)
    return False


def watch_until_still(tracker, thresh: float, window: float, timeout: float, *,
                      stop_evt=None, on_sample=None):
    """Watch until MOTION is seen (metric ≥ ``thresh``) and THEN the metric stays
    below ``thresh`` for ``window``. Returns the time motion first ceased (so the
    reported duration excludes the confirmation window), or None on timeout.

    The motion-first requirement is what prevents a false 0 ms reading when the
    motion is not being detected at all — that surfaces as a timeout instead, which
    is diagnosable.
    """
    t0 = time.monotonic()
    below_since = None
    seen_motion = False
    while not _stopped(stop_evt) and (time.monotonic() - t0) < timeout:
        now = time.monotonic()
        m = tracker.metric()
        if m is not None:
            if on_sample:
                try:
                    on_sample(now, m)
                except Exception:
                    pass
            if m >= thresh:
                seen_motion = True
                below_since = None
            elif seen_motion:
                if below_since is None:
                    below_since = now
                elif now - below_since >= window:
                    return below_since
        time.sleep(0.02)
    return None


def rest_threshold(controller, tracker, origin_um, *, stop_evt=None,
                   on_sample=None) -> tuple:
    """Still/moving threshold for the stage-position detector: measure ONLY the
    at-rest jitter (no move) and place the threshold safely above it.

    Deliberately no move-peak: the peak is exactly what made the camera threshold
    fragile on short segments. The metric here is absolute µm, and real motion
    (tens–hundreds of µm/poll) clears a 3 µm floor by a wide margin.

    Returns ``(floor, peak, still_thresh)`` in µm/poll.
    """
    _goto(controller, origin_um)
    tracker.reset()
    dwell(tracker, 1.5, stop_evt=stop_evt, on_sample=on_sample)   # drain open loop
    rest = dwell(tracker, 1.0, stop_evt=stop_evt, on_sample=on_sample)
    if rest:
        srt = sorted(rest)
        floor = srt[min(len(srt) - 1, int(len(srt) * 0.9))]       # 90th percentile
    else:
        floor = 0.0
    still_um = getattr(tracker, "STILL_UM", 3.0)
    motion_um = getattr(tracker, "MOTION_UM", 60.0)
    return (floor, motion_um, max(still_um, floor * 3.0, floor + 3.0))


# ── motion helpers (absolute stage µm throughout) ─────────────────────────

def _read_um(controller):
    try:
        p = controller.get_xy_position(cached=False)
    except Exception:
        return None
    if not p or p[0] is None or p[1] is None:
        return None
    return (float(p[0]), float(p[1]))


def _goto(controller, target_um) -> None:
    """Absolute-µm move. No arrival wait — callers that need one use the tracker's
    stillness detection, which is a stronger condition than proximity."""
    try:
        controller.move_xy_absolute_um(float(target_um[0]), float(target_um[1]))
    except Exception as e:                                    # pragma: no cover
        logger.debug("XYTopSpeed goto failed: %s", e)


def _r2(xs, ys, slope, intercept) -> float:
    """Coefficient of determination for the fitted line — the run's own quality
    signal, used downstream to gate an automatic commit."""
    n = len(ys)
    if n < 3:
        return 0.0
    my = sum(ys) / n
    ss_tot = sum((y - my) ** 2 for y in ys)
    if ss_tot <= 1e-15:
        return 0.0
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    return max(0.0, 1.0 - ss_res / ss_tot)


def measure_top_speed(
    controller,
    *,
    max_dist_mm: float = DEFAULT_MAX_DIST_MM,
    n_points: int = DEFAULT_N_POINTS,
    repeats: int = DEFAULT_REPEATS,
    still_window_s: float = DEFAULT_STILL_WINDOW_S,
    direction=(1.0, 0.0),
    center_first: bool = True,
    origin_um=None,
    safe_z_mm=None,
    move_timeout_s: float = DEFAULT_MOVE_TIMEOUT_S,
    quiet_timeout_s: float = DEFAULT_QUIET_TIMEOUT_S,
    stop_evt=None,
    on_progress=None,
    on_sample=None,
    on_marker=None,
) -> dict:
    """Measure the stage's true top speed (µm/s at ``SMS,100``).

    Args:
        controller: a ``StageController``.
        max_dist_mm: longest single move in the sweep; shrunk to fit the envelope.
        n_points: number of distances (evenly spaced up to ``max_dist_mm``).
        repeats: timed moves per distance; directions alternate.
        still_window_s: how long the metric must stay below threshold to count as
            stopped.
        direction: unit vector for the sweep axis (default +X).
        center_first: travel to the middle of the reachable envelope first. A probe
            starting near a travel extent CLAMPS, and a clamped probe silently
            measures the clamp.
        origin_um: explicit sweep anchor in absolute stage µm (overrides centring).
        safe_z_mm: retract height used by the centring travel.
        stop_evt: ``threading.Event`` for a cooperative abort.
        on_progress: ``callable(str)`` status lines.
        on_sample: ``callable(t_monotonic, delta_um)`` for a live motion strip.
        on_marker: ``callable(t_monotonic, "cmd"|"still")`` for strip markers.

    Returns:
        ``{"top_speed_um_s", "slope_s_per_mm", "intercept_s", "r2", "n",
        "distances_mm", "points", "rows", "still_thresh_um", "jitter_um",
        "origin_um"}`` or ``{"error": ..., "rows": [...]}``.
    """
    def _say(msg):
        logger.info("[top-speed] %s", msg)
        if on_progress:
            try:
                on_progress(msg)
            except Exception:
                pass

    xy = getattr(controller, "xy_stage", None)
    if xy is None or not getattr(controller, "is_xy_connected", False):
        return {"error": "XY stage not connected"}

    tracker = EncoderMotion(controller)
    if not tracker.available():
        return {"error": "stage-position detector unavailable"}

    max_dist_mm = max(0.5, float(max_dist_mm))
    n_points = max(2, int(n_points))
    repeats = max(1, int(repeats))

    # ── envelope: shrink to fit, or refuse ──
    # This check did not exist in the GUI version. Without it a clamped sweep fits a
    # truncated distance into the same time and reports a LOW top speed — which then
    # makes every commanded mm/s run proportionally FAST.
    try:
        from SupportClasses import XYAutoCalibration as AC
        fit = AC.check_fits(controller, max_dist_mm * 1000.0)
        if not fit["ok"] and fit["usable_um"] > 0:
            max_dist_mm = max(0.5, fit["usable_um"] / 1000.0)
            _say(f"sweep shrunk to {max_dist_mm:.2f} mm to stay clear of the "
                 f"travel limits")
        elif not fit["ok"]:
            return {"error": fit["reason"]}
        if origin_um is None and center_first:
            res = AC.center_stage(controller, safe_z_mm=safe_z_mm)
            if not res["ok"]:
                return {"error": f"could not centre the stage: {res['reason']}"}
            origin_um = res["center_um"]
            _say(f"centred at ({origin_um[0]:.0f}, {origin_um[1]:.0f}) µm")
    except Exception as e:                                    # pragma: no cover
        return {"error": f"could not prepare the test area: {e}"}

    if origin_um is None:
        origin_um = _read_um(controller)
    if origin_um is None:
        return {"error": "could not read stage position"}
    origin_um = (float(origin_um[0]), float(origin_um[1]))

    ux, uy = float(direction[0]), float(direction[1])
    norm = math.hypot(ux, uy) or 1.0
    ux, uy = ux / norm, uy / norm

    # Full speed — the whole point is to find what the stage does flat out.
    restore_speed = None
    try:
        if hasattr(xy, "set_acceleration"):
            xy.set_acceleration(80)
        if hasattr(xy, "set_velocity"):
            restore_speed = 20
            xy.set_velocity(100)                              # SMS,100
    except Exception:
        pass

    suspend = getattr(controller, "suspend_position_poller", None)
    resume = getattr(controller, "resume_position_poller", None)
    if callable(suspend):
        try:
            suspend()
        except Exception:
            pass

    rows: list = []
    dists = [round(max_dist_mm * k / n_points, 3) for k in range(1, n_points + 1)]
    try:
        floor, _peak, still_thresh = rest_threshold(
            controller, tracker, origin_um, stop_evt=stop_evt, on_sample=on_sample)
        _say(f"rest jitter {floor:.1f} µm/poll → still threshold "
             f"{still_thresh:.1f} µm/poll")
        if _stopped(stop_evt):
            return {"error": "stopped", "rows": rows}

        _say("sweeping " + ", ".join(f"{d:g}" for d in dists) + " mm at full speed")
        for d in dists:
            if _stopped(stop_evt):
                break
            times = []
            for rep in range(repeats):
                if _stopped(stop_evt):
                    break
                # Alternate: even reps go out from the origin, odd reps come back.
                # The return leg IS the next measurement, so nothing is spent
                # repositioning and axis asymmetry shows as spread, not bias.
                out = (rep % 2 == 0)
                anchor = origin_um if out else (origin_um[0] + ux * d * 1000.0,
                                                origin_um[1] + uy * d * 1000.0)
                target = ((origin_um[0] + ux * d * 1000.0,
                           origin_um[1] + uy * d * 1000.0) if out else origin_um)
                t = _time_one_move(
                    controller, tracker, anchor, target, still_thresh,
                    still_window_s, move_timeout_s, quiet_timeout_s,
                    stop_evt=stop_evt, on_sample=on_sample, on_marker=on_marker)
                if t is not None:
                    times.append(t)
            if not times:
                _say(f"  d={d:g} mm: no stop detected (timeout)")
                continue
            avg = sum(times) / len(times)
            rows.append({"dist_mm": d, "time_s": avg, "reps": len(times),
                         "times_s": [round(x, 4) for x in times]})
            _say(f"  d={d:g} mm: {avg * 1000:.0f} ms (n={len(times)})")

        pts = [(r["dist_mm"], r["time_s"]) for r in rows]
        if len(pts) < 2:
            return {"error": "not enough usable points to fit a top speed",
                    "rows": rows}
        slope, intercept = fit_line([p[0] for p in pts], [p[1] for p in pts])
        if slope <= 1e-6:
            return {"error": "degenerate time-vs-distance fit (slope ≤ 0)",
                    "rows": rows}
        top_um_s = 1000.0 / slope           # mm/s → µm/s
        r2 = _r2([p[0] for p in pts], [p[1] for p in pts], slope, intercept)
        _say(f"top speed {top_um_s / 1000.0:.3f} mm/s "
             f"(slope {slope:.5f} s/mm, intercept {intercept:.3f} s, R²={r2:.4f})")
        return {
            "top_speed_um_s": top_um_s,
            "slope_s_per_mm": slope,
            "intercept_s": intercept,
            "r2": r2,
            "n": len(pts),
            "distances_mm": dists,
            "points": pts,
            "rows": rows,
            "still_thresh_um": still_thresh,
            "jitter_um": floor,
            "origin_um": origin_um,
        }
    except Exception as e:                                    # pragma: no cover
        logger.exception("top-speed measurement error")
        return {"error": str(e), "rows": rows}
    finally:
        # Always: stop, go home, restore the stage speed, resume the poller.
        try:
            if hasattr(controller, "send_velocity_xy"):
                controller.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        try:
            _goto(controller, origin_um)
        except Exception:
            pass
        try:
            if restore_speed is not None and hasattr(xy, "set_velocity"):
                xy.set_velocity(restore_speed)
        except Exception:
            pass
        if callable(resume):
            try:
                resume()
            except Exception:
                pass


def _time_one_move(controller, tracker, anchor_um, target_um, still_thresh,
                   still_window, move_timeout, quiet_timeout, *,
                   stop_evt=None, on_sample=None, on_marker=None):
    """Time ONE move from ``anchor_um`` to ``target_um``: settle at the anchor,
    command the move, and measure how long until motion ceases. Returns seconds,
    or None if the stage never went quiet or never stopped."""
    _goto(controller, anchor_um)
    tracker.reset()
    if not wait_quiet(tracker, still_thresh, still_window, quiet_timeout,
                      stop_evt=stop_evt, on_sample=on_sample):
        return None
    if _stopped(stop_evt):
        return None
    _goto(controller, target_um)
    t_cmd = time.monotonic()
    if on_marker:
        try:
            on_marker(t_cmd, "cmd")
        except Exception:
            pass
    t_still = watch_until_still(tracker, still_thresh, still_window, move_timeout,
                                stop_evt=stop_evt, on_sample=on_sample)
    if t_still is None:
        return None
    if on_marker:
        try:
            on_marker(t_still, "still")
        except Exception:
            pass
    return max(0.0, t_still - t_cmd)


def write_jsonl(result: dict, log_dir) -> str:
    """Append-free record of one sweep, mirroring the format the GUI page wrote so
    existing ``logs/timing/topspeed_*.jsonl`` files stay comparable."""
    try:
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = log_dir / f"topspeed_{stamp}.jsonl"
        slope = float(result.get("slope_s_per_mm", 0.0) or 0.0)
        top = result.get("top_speed_um_s")
        with open(path, "w", encoding="utf-8") as f:
            f.write(json.dumps({
                "event": "config", "kind": "top_speed_sweep",
                "distances_mm": result.get("distances_mm", []),
                "repeats": max((r.get("reps", 0) for r in result.get("rows", [])),
                               default=0),
                "still_window_s": result.get("still_window_s",
                                             DEFAULT_STILL_WINDOW_S)}) + "\n")
            for r in result.get("rows", []):
                f.write(json.dumps({"event": "point", **r}) + "\n")
            f.write(json.dumps({
                "event": "fit", "slope_s_per_mm": round(slope, 6),
                "intercept_s": round(float(result.get("intercept_s", 0.0)), 5),
                "r2": round(float(result.get("r2", 0.0)), 5),
                "top_speed_mm_s": (None if not top else round(top / 1000.0, 4)),
            }) + "\n")
        return str(path)
    except Exception as e:                                    # pragma: no cover
        logger.debug("top-speed JSONL write failed: %s", e)
        return ""
