"""XYDeadTime.py — measure the XY stage's command→motion dead time by step response.

WHY THIS EXISTS
───────────────
The velocity follower's stability limit is

    v_max ≈ lookahead / ((loop_period + dead_time) · safety)

so ``dead_time`` directly divides the achievable print speed. Until now the only
available number was ``PrintTimingCalibrationStore.get_phase_lag_s()``, the mean
of the ``by_phase.intercept_s`` column — but that column is an *optically*
measured **settle** time ("how long until the camera frames stop changing"), not
a transport delay. On ME3B V1 its five entries are 0.1215, −0.1200, 0.5766,
−0.2360 and 0.1634 s: two are physically impossible, and the dominant 0.5766 s is
a single-point fit of a single run. Their mean, 0.2872 s, is 90 % of the
dead-time budget and caps prints at **0.31 mm/s on a stage measured at 5.9 mm/s**.

``slope_s_per_seg`` is not a substitute — it is seconds *per segment*, a
discrete-streaming quantity with different units.

This module measures the right thing directly, with no camera: from rest, send a
velocity command and time how long until the stage's own encoder reports real
motion. The same trace also yields the first-order rise time and the achieved
cruise speed — an independent cross-check of ``xy_max_speed_um_s``.

MEASUREMENT
───────────
Per trial:
  1. sit still and poll → at-rest jitter σ (also gives the real poll interval);
  2. ``t_cmd``: send the velocity command;
  3. poll until displacement exceeds ``max(4σ, MIN_MOTION_UM)``;
     ``dead_time = t_first_motion − t_cmd − poll_interval/2``
     (the sample could have landed anywhere inside its interval);
  4. keep polling through the ramp → fit the cruise leg; its **x-intercept** is
     the classic FOPDT "apparent lag" ``L`` (dead time + half the rise), which is
     what a pure-pursuit loop actually experiences, and its **slope** is the
     achieved speed;
  5. stop, return to start, reverse direction, repeat.

Directions alternate (and ±X / ±Y / ±diag are all available) so backlash and
axis asymmetry show up as spread rather than biasing a single number.

SAFETY
──────
XY only — this module never touches Z, and the caller must already have the
needle retracted to a safe Z (the challenge dialog's ``_prep()`` does).
Every exit path — success, no-motion, exception, cooperative stop — sends
``VS 0,0`` and returns the stage to its starting position, and the position
poller is suspended for the duration so it cannot contend for the serial port.
The probe speed is clamped to ``safety_limits.max_xy_speed`` and each trial is
bounded both by a displacement limit and by a wall-clock timeout.
"""

from __future__ import annotations

import logging
import math
import time

logger = logging.getLogger(__name__)

# A displacement must clear this (and the measured jitter) to count as motion.
MIN_MOTION_UM = 3.0
# Rest-jitter sampling.
REST_SAMPLES = 12
# Per-trial caps.
DEFAULT_MAX_TRAVEL_UM = 1500.0
DEFAULT_TIMEOUT_S = 3.0
# Directions offered, as unit vectors.
AXES = {
    "x": (1.0, 0.0),
    "y": (0.0, 1.0),
    "diag": (1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)),
}


def _median(vals):
    v = sorted(vals)
    n = len(v)
    if not n:
        return 0.0
    mid = n // 2
    return v[mid] if n % 2 else 0.5 * (v[mid - 1] + v[mid])


def _mad_spread(vals):
    """Robust spread: 1.4826 × median-absolute-deviation (≈ σ for a normal)."""
    if len(vals) < 2:
        return 0.0
    med = _median(vals)
    return 1.4826 * _median([abs(v - med) for v in vals])


def fit_line(xs, ys):
    """Least-squares ``(slope, intercept)``; ``(0, 0)`` if degenerate.

    v7.21.2: promoted from ``_fit_line`` to a public name so ``XYTopSpeed`` fits its
    time-vs-distance sweep with the SAME arithmetic, rather than carrying a third
    copy of least squares (the GUI page had its own).
    """
    n = len(xs)
    if n < 2:
        return (0.0, 0.0)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 1e-12:
        return (0.0, 0.0)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    return (slope, my - slope * mx)


#: Back-compat alias — this module's own callers still use the private name.
_fit_line = fit_line


def measure_velocity_dead_time(
    controller,
    *,
    speed_um_s: float = 3000.0,
    axis: str = "diag",
    repeats: int = 5,
    max_travel_um: float = DEFAULT_MAX_TRAVEL_UM,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    center_first: bool = True,
    safe_z_mm=None,
    stop_evt=None,
    on_progress=None,
) -> dict:
    """Measure command→motion dead time (and the achieved cruise speed).

    Args:
        controller: a ``StageController``.
        speed_um_s: step magnitude; clamped to ``safety_limits.max_xy_speed``.
        axis: ``"x"``, ``"y"`` or ``"diag"`` (the follower commands both axes
            every tick, so ``"diag"`` is the most representative default).
        repeats: trials; directions alternate so backlash shows as spread.
        max_travel_um / timeout_s: per-trial bounds.
        stop_evt: optional ``threading.Event`` for a cooperative abort.
        on_progress: optional ``callable(str)`` for status lines.

    Returns:
        ``{"dead_time_s", "dead_time_spread_s", "apparent_lag_s", "tau_s",
        "cruise_um_s", "n", "trials": [...], "axis", "speed_um_s"}`` or
        ``{"error": "..."}``. ``dead_time_s`` is the median first-motion delay;
        ``apparent_lag_s`` is the (larger, more conservative) FOPDT intercept —
        prefer it for the stability limit.
    """
    def _say(msg):
        logger.info("[dead-time] %s", msg)
        if on_progress:
            try:
                on_progress(msg)
            except Exception:
                pass

    xy = getattr(controller, "xy_stage", None)
    if xy is None or not getattr(controller, "is_xy_connected", False):
        return {"error": "XY stage not connected"}
    if not hasattr(controller, "send_velocity_xy"):
        return {"error": "controller has no continuous-velocity command"}

    ux, uy = AXES.get(str(axis).lower(), AXES["diag"])

    # ── Centre the stage first, and shrink the excursion to fit ──
    # A probe that starts near a travel extent CLAMPS, and a clamped probe does
    # not fail loudly — it reports "no motion detected" or fits a truncated
    # distance. Both would be silently wrong, so centre first and bound the run.
    if center_first:
        try:
            from SupportClasses import XYAutoCalibration as AC
            fit = AC.check_fits(controller, max_travel_um)
            if not fit["ok"] and fit["usable_um"] > 0:
                max_travel_um = max(100.0, fit["usable_um"])
                _say(f"excursion shrunk to {max_travel_um:.0f} µm to stay clear "
                     f"of the travel limits")
            elif not fit["ok"]:
                return {"error": fit["reason"]}
            res = AC.center_stage(controller, safe_z_mm=safe_z_mm)
            if not res["ok"]:
                return {"error": f"could not centre the stage: {res['reason']}"}
            _say(f"centred at ({res['center_um'][0]:.0f}, "
                 f"{res['center_um'][1]:.0f}) µm — "
                 f"±{max_travel_um:.0f} µm excursion fits with margin")
        except Exception as e:                            # pragma: no cover
            return {"error": f"could not centre the stage: {e}"}

    def _read():
        try:
            p = controller.get_xy_position(cached=False)
        except Exception:
            return None
        if not p or p[0] is None or p[1] is None:
            return None
        return (float(p[0]), float(p[1]))

    p0 = _read()
    if p0 is None:
        return {"error": "could not read stage position"}
    start_x, start_y = p0

    # Clamp the step magnitude to the envelope.
    v = abs(float(speed_um_s))
    try:
        cap = float(getattr(controller.safety_limits, "max_xy_speed", 0) or 0)
        if cap > 0:
            v = min(v, cap)
    except Exception:
        pass
    v = max(v, 200.0)

    # SMS must not cap the commanded VS below the probe speed, or we would be
    # timing the SMS limiter instead of the transport delay.
    try:
        if hasattr(xy, "set_acceleration"):
            xy.set_acceleration(80)
        if hasattr(xy, "set_speed_mm_s"):
            xy.set_speed_mm_s(max(v / 1000.0 * 1.5, 3.0))
    except Exception:
        pass

    suspend = getattr(controller, "suspend_position_poller", None)
    resume = getattr(controller, "resume_position_poller", None)
    if callable(suspend):
        try:
            suspend()
        except Exception:
            pass

    trials: list = []
    try:
        # ── at-rest jitter + the real poll interval ──
        rest = []
        t_prev = time.monotonic()
        intervals = []
        for _ in range(REST_SAMPLES):
            q = _read()
            now = time.monotonic()
            intervals.append(now - t_prev)
            t_prev = now
            if q is not None:
                rest.append(math.hypot(q[0] - start_x, q[1] - start_y))
        jitter = _mad_spread(rest) if len(rest) > 2 else 0.0
        poll_dt = _median(intervals) if intervals else 0.02
        thresh = max(MIN_MOTION_UM, 4.0 * jitter)
        _say(f"rest jitter {jitter:.2f} µm · poll {poll_dt * 1000:.0f} ms · "
             f"motion threshold {thresh:.1f} µm")

        for i in range(max(1, int(repeats))):
            if stop_evt is not None and stop_evt.is_set():
                break
            sign = 1.0 if (i % 2 == 0) else -1.0
            base = _read()
            if base is None:
                continue
            bx, by = base

            samples = []          # (t_rel, displacement_um)
            t_cmd = time.monotonic()
            controller.send_velocity_xy(v * ux * sign, v * uy * sign)
            first_motion = None
            while True:
                if stop_evt is not None and stop_evt.is_set():
                    break
                q = _read()
                now = time.monotonic()
                t_rel = now - t_cmd
                if q is not None:
                    d = math.hypot(q[0] - bx, q[1] - by)
                    samples.append((t_rel, d))
                    if first_motion is None and d > thresh:
                        # Correct for the sampling interval: the crossing could
                        # have happened anywhere inside this poll.
                        first_motion = max(0.0, t_rel - 0.5 * poll_dt)
                    if d >= max_travel_um:
                        break
                if t_rel >= timeout_s:
                    break
            controller.send_velocity_xy(0.0, 0.0)

            if first_motion is None:
                _say(f"trial {i + 1}: no motion detected — check VS direction "
                     f"and that SMS is not zero")
                trials.append({"dir": sign, "moved": False})
                continue

            # Cruise leg = the last ~60 % of the samples that are past the
            # motion threshold; its slope is the achieved speed and its
            # x-intercept is the FOPDT apparent lag.
            moving = [(t, d) for (t, d) in samples if d > thresh]
            cruise = moving[max(0, int(len(moving) * 0.4)):]
            slope, intercept = _fit_line([t for t, _ in cruise],
                                         [d for _, d in cruise])
            cruise_um_s = slope if slope > 0 else 0.0
            apparent_lag = (-intercept / slope) if slope > 1e-9 else first_motion
            apparent_lag = max(0.0, min(apparent_lag, timeout_s))
            tau = max(0.0, apparent_lag - first_motion)

            trials.append({
                "dir": sign, "moved": True,
                "dead_time_s": first_motion,
                "apparent_lag_s": apparent_lag,
                "tau_s": tau,
                "cruise_um_s": cruise_um_s,
                "n_samples": len(samples),
            })
            _say(f"trial {i + 1}: dead time {first_motion * 1000:.0f} ms · "
                 f"apparent lag {apparent_lag * 1000:.0f} ms · "
                 f"cruise {cruise_um_s:.0f} µm/s")

            # Return to the trial's start so trials don't walk the stage.
            _return_to(controller, bx, by)

        ok = [t for t in trials if t.get("moved")]
        if not ok:
            return {"error": "no motion detected — verify the VS direction on a "
                             "low safe path, and that SMS is non-zero",
                    "trials": trials}

        dts = [t["dead_time_s"] for t in ok]
        lags = [t["apparent_lag_s"] for t in ok]
        taus = [t["tau_s"] for t in ok]
        cruises = [t["cruise_um_s"] for t in ok if t["cruise_um_s"] > 0]
        out = {
            "dead_time_s": _median(dts),
            "dead_time_spread_s": _mad_spread(dts),
            "apparent_lag_s": _median(lags),
            "apparent_lag_spread_s": _mad_spread(lags),
            "tau_s": _median(taus),
            "cruise_um_s": _median(cruises) if cruises else 0.0,
            "n": len(ok),
            "trials": trials,
            "axis": str(axis),
            "speed_um_s": v,
            "poll_dt_s": poll_dt,
            "jitter_um": jitter,
        }
        _say(f"MEDIAN dead time {out['dead_time_s'] * 1000:.0f} ms "
             f"(±{out['dead_time_spread_s'] * 1000:.0f}) · apparent lag "
             f"{out['apparent_lag_s'] * 1000:.0f} ms · cruise "
             f"{out['cruise_um_s']:.0f} µm/s over {out['n']} trials")
        return out

    except Exception as e:                                # pragma: no cover
        logger.exception("dead-time measurement failed")
        return {"error": f"{e}", "trials": trials}
    finally:
        # Belt and braces: stop, go home, resume the poller — on EVERY path.
        try:
            controller.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        _return_to(controller, start_x, start_y)
        if callable(resume):
            try:
                resume()
            except Exception:
                pass


def _return_to(controller, x_um, y_um):
    """Best-effort XY-only return to an absolute µm position. Never raises, and
    never touches Z."""
    try:
        controller.send_velocity_xy(0.0, 0.0)
    except Exception:
        pass
    try:
        mover = getattr(controller, "move_xy_absolute_um", None)
        if callable(mover):
            mover(x_um, y_um)
            waiter = getattr(controller, "wait_for_xy_arrival", None)
            if callable(waiter):
                waiter(x_um / 1000.0, y_um / 1000.0,
                       tolerance_mm=0.05, timeout_s=10.0)
    except Exception:
        logger.debug("dead-time probe: return-to-start failed", exc_info=True)


def stable_speed_mm_s(dead_time_s, control_loop_ms, lookahead_mm,
                      *, safety=2.0, lead_time_frac=0.0):
    """The pure-pursuit stable speed for a given dead time — the arithmetic the
    calibration readout shows the operator, in one place so the UI and the
    follower cannot disagree.
    """
    loop_s = max(0.0, float(control_loop_ms or 0.0)) / 1000.0
    lag = max(0.0, float(dead_time_s or 0.0))
    residual = lag * (1.0 - min(1.0, max(0.0, float(lead_time_frac or 0.0))))
    dead = loop_s + residual
    if dead <= 1e-6:
        return None
    return max(0.05, float(lookahead_mm) / (dead * max(1.0, float(safety))))
