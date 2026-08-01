"""
PrintTimingCalibrationStore.py — measured XY↔ZP print-timing sync model.

v7.5.x: The discrete print path streams XY moves to the Prior controller
OPEN-LOOP (each ``G x,y`` is acknowledged on receipt and executes
asynchronously), paced only by a ``time.sleep`` ESTIMATE of the segment
transit time. Short segments physically take longer than that estimate
(accel/decel), so the Prior falls progressively behind the commanded path —
while the pump (Marlin, bounded by an M400 barrier every few segments) keeps
up with the commands. The result is that the pump finishes the path's volume
BEFORE the needle physically finishes tracing it → the tail of the print is
deposited dry.

The XY↔ZP Timing Calibration workflow MEASURES this lag buildup on the real
machine over long runs (1–5 min) and records, per print speed, an empirical
model of the sync error so the print pacing can be corrected:

  • ``effective_speed_mm_s`` — the speed the stage ACTUALLY sustains (path
    distance physically traced ÷ elapsed), which is < the commanded speed.
  • ``correction_factor``    — commanded_speed ÷ effective_speed (> 1). The
    per-segment loop sleep should be multiplied by this so the loop issues
    commands no faster than the stage can physically execute them, keeping
    the pump locked to the needle.
  • ``lag_rate_mm_per_s``    — how fast the XY backlog (commanded − actual
    path distance) grows per second of printing. Multiply by the print
    duration to predict the final pump-ahead distance.

Estimates are REFINED across runs (run-count-weighted running mean), so each
calibration makes the model "better and better".

Data file: config/hardware/print_timing_calibration.json

Structure::

    {
      "version": "1.0",
      "by_speed": {
        "5.00": {
          "speed_mm_s": 5.0,
          "effective_speed_mm_s": 3.61,
          "correction_factor": 1.385,
          "lag_rate_mm_per_s": 0.276,
          "seg_len_mm": 0.262,
          "runs": 3,
          "samples": 4120,
          "last_duration_s": 300.0,
          "last_updated": "2026-06-18T15:40:00"
        }
      }
    }
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_PATH = Path("config/hardware/print_timing_calibration.json")


def _speed_key(speed_mm_s: float) -> str:
    """Bucket speeds to 2 dp so repeated runs at the same speed merge."""
    return f"{float(speed_mm_s):.2f}"


def _turn_key(shape: str, speed_mm_s: float, distance_mm: float) -> str:
    """Bucket a turn-lag point by (shape, speed, leg distance)."""
    return f"{str(shape).lower()}|{float(speed_mm_s):.2f}|{float(distance_mm):.2f}"


class PrintTimingCalibrationStore:
    """Load/save/refine the per-speed XY↔ZP print-timing sync model."""

    # v7.5.x: per-print-mode path-following parameters, tuned by the XY Printing
    # Challenge and READ by the real print path (PrintManager). Defaults mirror
    # PrintManager's constants (kept in sync here to avoid importing it). The
    # challenge writes these; Quick Print stamps them onto PrintSettings.
    PATH_TUNING_DEFAULTS = {
        # open-loop streamed path: pace multiplier (>1 slows the per-segment
        # sleep to match a stage that runs slower than commanded, keeping the
        # pump locked to the needle). 1.0 = legacy behaviour.
        "open_loop": {"pace_correction": 1.0},
        # confirmed per-segment: arrival tolerance (µm) + corner angle. The
        # executor waits/drains ONLY at vertices whose turn exceeds
        # corner_angle_deg (straight edges stream), so a star stops at its
        # corners, not every sampled node.
        "confirm": {"settle_tol_um": 40.0, "corner_angle_deg": 30.0},
        # velocity closed-loop pure-pursuit follower + corner-aware speed
        # scheduling + cross-track PID (Kp/Kd; 0 = pure pursuit until tuned).
        # corner_angle_deg = turn angle that counts as a corner;
        # corner_speed_factor = fraction of print speed allowed AT a sharp
        # corner (0..1; lower = slower/tighter through corners).
        # ── Stage-1/3 additions: EVERY new key defaults to 0 (= off), so an
        # existing calibration file that holds only the seven legacy keys loads
        # and fills the rest with zeros → byte-identical legacy behaviour, no
        # migration. See VelocityControl.resolve_control / pursuit_step.
        #
        # Speed decoupling (the fix for "the tuner just slows everything down"):
        #   lead_time_frac     — share of the dead time compensated by prediction
        #   min_lookahead_frac — size the lookahead FROM speed × dead time, so
        #                        lookahead stops BEING the speed knob
        #   max_speed_frac     — clamp to a fraction of the measured top speed
        #   hold_speed         — 1 = pin straight-line speed; corners are then
        #                        the only modulation ("slow only at corners")
        # Control-law robustness:
        #   normal_bound_frac / min_forward_frac — bound the cross-track command
        #                        so it can never overwhelm forward motion
        #   d_filter_hz, pid_ki, i_limit_mm_s
        #   reacquire_*        — recover instead of deadlocking when off-path
        #   stall_* / dither_* — detect "commanding speed but not progressing"
        #   lateral_accel_mm_s2 / decel_accel_mm_s2 / profile_sample_mm
        #   max_cross_track_mm / runaway_ticks — operator-tightenable guards
        #   jerk_pct           — Prior SCS S-curve limit (0 = don't touch)
        "velocity": {"lookahead_mm": 0.6, "control_hz": 25.0, "decel_mm": 1.5,
                     "corner_angle_deg": 30.0, "corner_speed_factor": 0.4,
                     "pid_kp": 0.0, "pid_kd": 0.0,
                     # speed decoupling
                     "lead_time_frac": 0.0, "min_lookahead_frac": 0.0,
                     "max_speed_frac": 0.0, "hold_speed": 0.0,
                     "deadtime_safety": 0.0,
                     # cross-track command shaping
                     "normal_bound_frac": 0.0, "min_forward_frac": 0.0,
                     "d_filter_hz": 0.0, "pid_ki": 0.0, "i_limit_mm_s": 0.0,
                     # re-acquire
                     "reacquire_cross_mm": 0.0, "reacquire_ticks": 0.0,
                     "reacquire_window_mm": 0.0, "reacquire_speed_frac": 0.0,
                     "reacquire_max_s": 0.0, "reacquire_back_step_mm": 0.0,
                     # stall / dither
                     "stall_ds_frac": 0.0, "stall_ticks": 0.0,
                     "dither_ratio_max": 0.0, "dither_min_mm": 0.0,
                     # prediction
                     "lead_max_mm": 0.0, "vel_filter_hz": 0.0,
                     # speed profile
                     "lateral_accel_mm_s2": 0.0, "decel_accel_mm_s2": 0.0,
                     "profile_sample_mm": 0.0,
                     # guards + hardware
                     "max_cross_track_mm": 0.0, "runaway_ticks": 0.0,
                     "jerk_pct": 0.0},
    }

    # Default print resolution element (µm) — the smallest feature the system can
    # resolve (≈ smallest needle/bead), which defines the acceptable path-
    # deviation tolerance. Overridable per machine.
    RESOLUTION_ELEMENT_UM_DEFAULT = 30.0

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._data: dict = {"version": "1.0", "by_speed": {}, "by_turn": {},
                            "by_phase": {}, "path_tuning": {}, "enabled": True}
        self._load()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
            if not isinstance(self._data.get("by_speed"), dict):
                self._data["by_speed"] = {}
            if not isinstance(self._data.get("by_turn"), dict):
                self._data["by_turn"] = {}
            if not isinstance(self._data.get("by_phase"), dict):
                self._data["by_phase"] = {}
            if not isinstance(self._data.get("path_tuning"), dict):
                self._data["path_tuning"] = {}
            if "enabled" not in self._data:
                self._data["enabled"] = True
        except Exception as exc:
            logger.warning(
                f"PrintTimingCalibrationStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "by_speed": {}}

    def save(self) -> None:
        """Atomic write (tmp + os.replace) so a crash can't truncate the file."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(
                dir=str(self._path.parent), suffix=".tmp")
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(self._data, f, indent=2)
                os.replace(tmp, self._path)
            finally:
                if os.path.exists(tmp):
                    try:
                        os.remove(tmp)
                    except OSError:
                        pass
            logger.debug(f"PrintTimingCalibrationStore: saved to {self._path}")
        except Exception as exc:
            logger.error(f"PrintTimingCalibrationStore: failed to save: {exc}")

    # ── Read ──────────────────────────────────────────────────────

    def get_xy_max_speed_um_s(self) -> Optional[float]:
        """The measured XY top speed (µm/s at 100% SMS), or None. Feeds the
        mm/s↔SMS-% conversion so commanded speeds are correct."""
        v = self._data.get("xy_max_speed_um_s")
        return float(v) if v else None

    def set_xy_max_speed_um_s(self, value_um_s: float) -> None:
        self._data["xy_max_speed_um_s"] = float(value_um_s)
        self.save()
        logger.info("XY top speed measured/stored: %.0f µm/s (%.2f mm/s).",
                    value_um_s, value_um_s / 1000.0)

    def get_control_loop_ms(self) -> Optional[float]:
        """The measured CLOSED-LOOP control period (ms per send-velocity +
        read-position cycle, WHILE moving), or None. This is the loop_period the
        velocity-following print path can actually run at — it caps the stable
        print speed. Measured by the 'Check comms rate' calibration."""
        v = self._data.get("control_loop_ms")
        return float(v) if v else None

    def set_control_loop_ms(self, value_ms: float) -> None:
        self._data["control_loop_ms"] = float(value_ms)
        self.save()
        logger.info("Control-loop period measured/stored: %.1f ms (%.1f Hz).",
                    value_ms, (1000.0 / value_ms) if value_ms else 0.0)

    def get_phase_lag_s(self) -> Optional[float]:
        """LEGACY phase-lag estimate (s) = mean ``intercept_s`` over the measured
        ``by_phase`` settle-sweep entries, or None if none measured.

        ⚠ This is a **settle time**, not a command→motion **dead time**, and it
        should not be what steers the velocity follower's speed cap. It is an
        optically-measured "how long until the frames stop changing" intercept:
        on ME3B V1 the five entries are 0.1215, −0.1200, 0.5766, −0.2360, 0.1634
        s — two are physically impossible negatives, and the dominant 0.5766 s is
        a **single-point fit of one run**. Their mean (0.2872 s) is 90 % of the
        dead-time budget and caps prints at 0.31 mm/s on a 5.9 mm/s stage.

        ``slope_s_per_seg`` is NOT a substitute: it is seconds *per segment* (a
        discrete-streaming lag), a different unit entirely.

        Prefer :meth:`get_velocity_dead_time_s`, which is measured by stepping the
        velocity command and timing the position response — the quantity the
        pure-pursuit stability limit actually depends on. This method is kept as
        the fallback for machines that have not run that probe yet.
        """
        entries = self._data.get("by_phase", {})
        vals = []
        for e in entries.values() if isinstance(entries, dict) else []:
            try:
                v = float(e.get("intercept_s"))
                if v >= 0:
                    vals.append(v)
            except (TypeError, ValueError, AttributeError):
                continue
        return (sum(vals) / len(vals)) if vals else None

    # ── purpose-measured velocity dead time (preferred over phase lag) ──

    def get_velocity_dead_time_s(self) -> Optional[float]:
        """Measured command→motion transport delay (s), or None if unmeasured.

        Produced by ``XYDeadTime.measure_velocity_dead_time``: from rest, send a
        velocity command and time the first real displacement. Unlike
        :meth:`get_phase_lag_s` this is the quantity the follower's stability
        limit depends on, and it is measured without a camera.
        """
        v = self._data.get("velocity_dead_time_s")
        try:
            return float(v) if v else None
        except (TypeError, ValueError):
            return None

    def set_velocity_dead_time_s(self, value_s: float, *, n: int = 0,
                                 spread_s: float = 0.0,
                                 tau_s: Optional[float] = None) -> None:
        """Store the measured dead time (plus its provenance, so a later reader
        can judge how much to trust it — the lesson of the by_phase table)."""
        self._data["velocity_dead_time_s"] = float(value_s)
        meta = {"n": int(n), "spread_s": float(spread_s),
                "last_updated": datetime.now().isoformat(timespec="seconds")}
        if tau_s is not None:
            meta["tau_s"] = float(tau_s)
        self._data["velocity_dead_time_meta"] = meta
        self.save()
        logger.info("Velocity dead time measured/stored: %.1f ms "
                    "(n=%d, spread %.1f ms).",
                    value_s * 1000.0, n, spread_s * 1000.0)

    def get_velocity_dead_time_meta(self) -> dict:
        m = self._data.get("velocity_dead_time_meta")
        return dict(m) if isinstance(m, dict) else {}

    def effective_dead_time_s(self) -> tuple:
        """``(dead_time_s, source)`` — the value the follower should use and where
        it came from: ``"measured"`` (the step-response probe), ``"phase_lag"``
        (the legacy by_phase mean), or ``"unmeasured"``.

        Exists so the UI can say WHICH number is capping the speed rather than
        leaving the operator to guess.
        """
        dt = self.get_velocity_dead_time_s()
        if dt and dt > 0:
            return (dt, "measured")
        lag = self.get_phase_lag_s()
        if lag and lag > 0:
            return (lag, "phase_lag")
        return (0.0, "unmeasured")

    def get_resolution_element_um(self) -> float:
        """The print resolution element (µm) = smallest resolvable feature ≈ the
        acceptable path-deviation tolerance. Drives the robustness panel PASS/FAIL
        and the follower's arrival tolerance."""
        v = self._data.get("resolution_element_um")
        try:
            return float(v) if v else self.RESOLUTION_ELEMENT_UM_DEFAULT
        except (TypeError, ValueError):
            return self.RESOLUTION_ELEMENT_UM_DEFAULT

    def set_resolution_element_um(self, value_um: float) -> None:
        self._data["resolution_element_um"] = float(value_um)
        self.save()

    def get_robustness_matrix(self) -> dict:
        """Persisted robustness-panel matrix (shape/size/speed lists). Falls back
        to a small default (3 shapes × 2 sizes × 3 speeds)."""
        m = self._data.get("robustness_matrix")
        if isinstance(m, dict) and m.get("shapes") and m.get("sizes_mm") \
                and m.get("speeds_mm_s"):
            return {"shapes": list(m["shapes"]),
                    "sizes_mm": [float(x) for x in m["sizes_mm"]],
                    "speeds_mm_s": [float(x) for x in m["speeds_mm_s"]]}
        return {"shapes": ["Square", "Star", "Circle"],
                "sizes_mm": [6.0, 12.0],
                "speeds_mm_s": [2.0, 5.0, 10.0]}

    def set_robustness_matrix(self, shapes, sizes_mm, speeds_mm_s) -> None:
        self._data["robustness_matrix"] = {
            "shapes": list(shapes),
            "sizes_mm": [float(x) for x in sizes_mm],
            "speeds_mm_s": [float(x) for x in speeds_mm_s]}
        self.save()

    def stable_velocity_speed_mm_s(
        self, lookahead_mm: Optional[float] = None, safety: float = 2.0,
    ) -> Optional[float]:
        """The max print speed the velocity follower can sustain without the
        pure-pursuit controller oscillating, given the measured control-loop
        period: ``v ≈ lookahead / (loop_period · safety)`` (the stage must not
        travel more than a fraction of the lookahead between corrections).
        None if the loop period hasn't been measured yet."""
        ms = self.get_control_loop_ms()
        if not ms or ms <= 0:
            return None
        if lookahead_mm is None:
            lookahead_mm = self.get_mode_params("velocity").get(
                "lookahead_mm", 0.6)
        loop_s = ms / 1000.0
        return max(0.05, float(lookahead_mm) / (loop_s * max(1.0, safety)))

    def is_enabled(self) -> bool:
        """Whether the print path should APPLY the measured timing correction
        (the single 'Use timing calibration' toggle; default on)."""
        return bool(self._data.get("enabled", True))

    def set_enabled(self, on: bool) -> None:
        self._data["enabled"] = bool(on)
        self.save()

    # ── Per-mode path-following params (tuned by the XY Printing Challenge) ──

    def get_mode_params(self, mode: str) -> dict:
        """Tuned path params for a print mode ('open_loop' | 'confirm' |
        'velocity'), merged over the defaults so a caller always gets every
        key."""
        base = dict(self.PATH_TUNING_DEFAULTS.get(mode, {}))
        stored = self._data.get("path_tuning", {}).get(mode)
        if isinstance(stored, dict):
            for k, v in stored.items():
                if isinstance(v, (int, float)):
                    base[k] = float(v)
        return base

    def set_mode_params(self, mode: str, params: dict) -> None:
        """Merge + persist tuned params for a mode (unknown keys ignored)."""
        if mode not in self.PATH_TUNING_DEFAULTS:
            return
        bucket = self._data.setdefault("path_tuning", {})
        cur = bucket.get(mode)
        cur = dict(cur) if isinstance(cur, dict) else {}
        allowed = set(self.PATH_TUNING_DEFAULTS[mode])
        for k, v in (params or {}).items():
            if k in allowed and isinstance(v, (int, float)):
                cur[k] = float(v)
        bucket[mode] = cur
        self.save()

    def all_mode_params(self) -> dict:
        """Every mode's merged params (defaults + stored)."""
        return {m: self.get_mode_params(m) for m in self.PATH_TUNING_DEFAULTS}

    def correction_for(self, speed_mm_s: float) -> Optional[float]:
        """The pacing multiplier (>1) for a print speed — slow the command
        issue rate by this so it matches the stage's MEASURED effective speed,
        keeping the pump locked to the needle. None if uncalibrated for this
        speed (caller should then pace unchanged)."""
        e = self.get(speed_mm_s)
        if e and e.get("correction_factor"):
            try:
                return float(e["correction_factor"])
            except (TypeError, ValueError):
                return None
        return None

    def get(self, speed_mm_s: float) -> Optional[dict]:
        """Return the merged estimate for a speed bucket, or None."""
        return self._data.get("by_speed", {}).get(_speed_key(speed_mm_s))

    def all(self) -> dict:
        return dict(self._data.get("by_speed", {}))

    def get_turn(self, shape: str, speed_mm_s: float,
                 distance_mm: float) -> Optional[dict]:
        return self._data.get("by_turn", {}).get(
            _turn_key(shape, speed_mm_s, distance_mm))

    def all_turns(self) -> dict:
        return dict(self._data.get("by_turn", {}))

    # ── Write / refine ────────────────────────────────────────────

    def update(
        self,
        speed_mm_s: float,
        effective_speed_mm_s: float,
        lag_rate_mm_per_s: float,
        seg_len_mm: float,
        sample_count: int,
        duration_s: float,
        buffer_drain_s: Optional[float] = None,
    ) -> dict:
        """Merge one run's measurement into the per-speed estimate.

        Refinement is a run-count-weighted running mean: each successive
        calibration nudges the stored estimate toward the new measurement,
        so the model improves with every run instead of being overwritten.

        ``buffer_drain_s`` (optional) is the OPTICALLY-measured delay between
        the controller reporting the move sequence done and the stage physically
        coming to rest (the Prior's command-buffer drain) — the trustworthy,
        controller-independent ground truth. Merged the same way when provided.

        Returns the merged entry.
        """
        speed_mm_s = float(speed_mm_s)
        eff = max(float(effective_speed_mm_s), 1e-6)
        lag_rate = float(lag_rate_mm_per_s)
        key = _speed_key(speed_mm_s)
        bucket = self._data.setdefault("by_speed", {})
        prev = bucket.get(key)
        drain = None if buffer_drain_s is None else float(buffer_drain_s)

        if prev and isinstance(prev, dict) and prev.get("runs", 0) >= 1:
            n = int(prev.get("runs", 1))
            # weighted running mean over runs (older runs keep their weight)
            eff = (prev.get("effective_speed_mm_s", eff) * n + eff) / (n + 1)
            lag_rate = (prev.get("lag_rate_mm_per_s", lag_rate) * n
                        + lag_rate) / (n + 1)
            if drain is not None and prev.get("buffer_drain_s") is not None:
                drain = (prev.get("buffer_drain_s") * n + drain) / (n + 1)
            elif drain is None:
                drain = prev.get("buffer_drain_s")
            runs = n + 1
            samples = int(prev.get("samples", 0)) + int(sample_count)
        else:
            runs = 1
            samples = int(sample_count)

        correction = speed_mm_s / eff if eff > 1e-6 else 1.0
        entry = {
            "speed_mm_s": round(speed_mm_s, 4),
            "effective_speed_mm_s": round(eff, 4),
            "correction_factor": round(correction, 4),
            "lag_rate_mm_per_s": round(lag_rate, 5),
            "seg_len_mm": round(float(seg_len_mm), 5),
            "runs": runs,
            "samples": samples,
            "last_duration_s": round(float(duration_s), 1),
            "last_updated": datetime.now().isoformat(timespec="seconds"),
        }
        if drain is not None:
            entry["buffer_drain_s"] = round(drain, 4)
        bucket[key] = entry
        self.save()
        logger.info(
            "Print-timing calibration @ %.2f mm/s updated (run %d): "
            "effective %.2f mm/s, correction ×%.3f, lag %.3f mm/s",
            speed_mm_s, runs, eff, correction, lag_rate)
        return entry


    def update_turn(
        self,
        shape: str,
        speed_mm_s: float,
        distance_mm: float,
        lag_mean_s: float,
        lag_std_s: float,
        n: int,
        divergence_rate_s_per_s: Optional[float] = None,
        fps: Optional[float] = None,
    ) -> dict:
        """Merge one turn-lag sweep point (shape × speed × leg distance) into the
        ``by_turn`` model, run-count-weighted like :meth:`update`. The turn lag
        is the optically-measured delay between a commanded direction change and
        the stage physically changing direction. Returns the merged entry."""
        key = _turn_key(shape, speed_mm_s, distance_mm)
        bucket = self._data.setdefault("by_turn", {})
        prev = bucket.get(key)
        mean = float(lag_mean_s)
        div = None if divergence_rate_s_per_s is None \
            else float(divergence_rate_s_per_s)

        if prev and isinstance(prev, dict) and prev.get("runs", 0) >= 1:
            m = int(prev.get("runs", 1))
            mean = (prev.get("lag_mean_s", mean) * m + mean) / (m + 1)
            if div is not None and prev.get("divergence_rate_s_per_s") is not None:
                div = (prev.get("divergence_rate_s_per_s") * m + div) / (m + 1)
            elif div is None:
                div = prev.get("divergence_rate_s_per_s")
            runs = m + 1
            turns = int(prev.get("turns", 0)) + int(n)
        else:
            runs = 1
            turns = int(n)

        entry = {
            "shape": str(shape).lower(),
            "speed_mm_s": round(float(speed_mm_s), 4),
            "distance_mm": round(float(distance_mm), 4),
            "lag_mean_s": round(mean, 4),
            "lag_std_s": round(float(lag_std_s), 4),
            "turns": turns,
            "runs": runs,
            "last_updated": datetime.now().isoformat(timespec="seconds"),
        }
        if div is not None:
            entry["divergence_rate_s_per_s"] = round(div, 6)
        if fps is not None:
            entry["fps"] = round(float(fps), 2)
        bucket[key] = entry
        self.save()
        logger.info(
            "Turn-lag calibration %s @ %.2f mm/s, leg %.2f mm (run %d): "
            "lag %.3f±%.3f s over %d turns",
            shape, speed_mm_s, distance_mm, runs, mean, lag_std_s, n)
        return entry


    def get_phase(self, speed_mm_s: float,
                  seg_len_mm: float) -> Optional[dict]:
        return self._data.get("by_phase", {}).get(
            f"{float(speed_mm_s):.2f}|{float(seg_len_mm):.2f}")

    def all_phase(self) -> dict:
        return dict(self._data.get("by_phase", {}))

    def update_phase(
        self,
        speed_mm_s: float,
        seg_len_mm: float,
        intercept_s: float,
        slope_s_per_seg: float,
        n_points: int,
    ) -> dict:
        """Merge one settle-delay sweep's fit into the per-(speed, segment)
        phase model: ``delay(N) ≈ intercept_s + slope_s_per_seg · N``. The slope
        is the controller's per-segment phase lag (how much further the stage
        falls behind for each extra streamed segment) — the timing that must be
        allocated to keep moves synced with the ZP stage. Run-count-weighted
        like the other estimates."""
        key = f"{float(speed_mm_s):.2f}|{float(seg_len_mm):.2f}"
        bucket = self._data.setdefault("by_phase", {})
        prev = bucket.get(key)
        inter = float(intercept_s)
        slope = float(slope_s_per_seg)
        if prev and isinstance(prev, dict) and prev.get("runs", 0) >= 1:
            m = int(prev.get("runs", 1))
            inter = (prev.get("intercept_s", inter) * m + inter) / (m + 1)
            slope = (prev.get("slope_s_per_seg", slope) * m + slope) / (m + 1)
            runs = m + 1
        else:
            runs = 1
        entry = {
            "speed_mm_s": round(float(speed_mm_s), 4),
            "seg_len_mm": round(float(seg_len_mm), 4),
            "intercept_s": round(inter, 5),
            "slope_s_per_seg": round(slope, 6),
            "n_points": int(n_points),
            "runs": runs,
            "last_updated": datetime.now().isoformat(timespec="seconds"),
        }
        bucket[key] = entry
        self.save()
        logger.info(
            "Phase-lag model @ %.2f mm/s, seg %.2f mm (run %d): "
            "delay ≈ %.0f + %.1f·N ms",
            speed_mm_s, seg_len_mm, runs, inter * 1000, slope * 1000)
        return entry


_store: Optional[PrintTimingCalibrationStore] = None


def get_store() -> PrintTimingCalibrationStore:
    """Process-wide singleton (mirrors the other calibration stores)."""
    global _store
    if _store is None:
        _store = PrintTimingCalibrationStore()
    return _store
