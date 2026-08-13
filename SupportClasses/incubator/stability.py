"""
stability.py — the numbers that tell you whether the heater actually works.

On a rig where a small film heater fights a large water thermal mass, the
interesting questions are not "what is the temperature" but:

  * How fast is it rising, and when will it get there?   -> rate, ETA
  * Did it overshoot, and how long did it take to settle? -> overshoot, settle
  * How tightly does it hold?                            -> ripple ±
  * **Is the heater even big enough?**                    -> steady-state duty
  * How sluggish is the system?                           -> time constant

The steady-state duty is the single most diagnostic figure here. At a stable
hold, duty is exactly the fraction of heater power the losses consume. Sitting
at 95-100 % means the heater is at its limit and cannot hold that setpoint
against ambient — insulate the block or fit a bigger heater. Sitting at 20 %
means plenty of headroom.

Pure Python, no numpy, no Qt.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class StabilityReport:
    """Snapshot of derived metrics for one zone."""

    temp_c: float | None = None
    target_c: float | None = None
    error_c: float | None = None
    duty_pct: float | None = None

    rate_c_per_min: float | None = None
    eta_s: float | None = None

    in_band: bool = False
    settled: bool = False
    settle_time_s: float | None = None
    overshoot_c: float | None = None

    ripple_c: float | None = None          # peak-to-peak over the settled window
    ripple_half_c: float | None = None     # ± figure, i.e. ripple/2
    mean_c: float | None = None

    steady_duty_pct: float | None = None
    time_constant_s: float | None = None

    #: Operator-facing one-liner about heater adequacy.
    headroom_note: str = ""

    samples: int = 0


class StabilityTracker:
    """
    Rolling statistics for one zone.

    Feed it :meth:`add` on every reading. All timing is ``time.monotonic``-based
    and supplied by the caller, so a simulated/accelerated clock works too.
    """

    #: Treated as "at setpoint" inside this band.
    BAND_C = 0.30
    #: Must stay in band this long (seconds) before we call it settled.
    SETTLE_DWELL_S = 60.0
    #: Rolling window kept for rate/ripple maths.
    WINDOW_S = 900.0
    #: Minimum samples before reporting a rate.
    MIN_RATE_SAMPLES = 6
    #: Trailing window used for the steady-state duty figure.
    STEADY_DUTY_WINDOW_S = 120.0
    #: Trailing window used for the ripple figure (longer, to catch slow drift).
    RIPPLE_WINDOW_S = 300.0
    #: While holding, temperature may wander this multiple of the band without
    #: invalidating the hold statistics. Requiring an unbroken in-band streak
    #: made the duty and ripple readouts flicker to "—" on every small
    #: excursion, which is exactly when an operator wants to see them.
    HOLD_BAND_MULTIPLE = 3.0
    #: Fraction of the window that must be near target to count as "holding".
    HOLD_FRACTION = 0.6

    def __init__(self, zone_id: str, *, band_c: float | None = None,
                 settle_dwell_s: float | None = None):
        self.zone_id = zone_id
        self.band_c = self.BAND_C if band_c is None else band_c
        self.settle_dwell_s = (
            self.SETTLE_DWELL_S if settle_dwell_s is None else settle_dwell_s
        )
        self._pts: deque[tuple[float, float, float | None]] = deque()  # (t, temp, duty)
        self._target: float | None = None
        self._target_set_at: float | None = None
        self._start_temp: float | None = None
        #: Strict unbroken in-band streak — resets on any excursion. Used ONLY to
        #: decide settle time, where an unbroken dwell is the point.
        self._in_band_since: float | None = None
        #: First moment the band was reached after the last target change. NOT
        #: reset by later excursions, so hold statistics keep a stable start
        #: point and do not collapse back into the ramp.
        self._first_in_band_at: float | None = None
        self._settle_time_s: float | None = None
        self._peak_after_target: float | None = None

    # ── input ───────────────────────────────────────────────────────

    def set_target(self, target_c: float | None, *, now: float | None = None) -> None:
        """
        Declare a new setpoint. Resets the transient metrics (settle time,
        overshoot) because they are only meaningful relative to one step.
        """
        t = time.monotonic() if now is None else now
        if target_c is not None and self._target is not None:
            if abs(float(target_c) - float(self._target)) < 1e-6:
                return  # same target, keep the existing transient history
        self._target = None if target_c is None else float(target_c)
        self._target_set_at = t
        self._start_temp = self._pts[-1][1] if self._pts else None
        self._in_band_since = None
        self._first_in_band_at = None
        self._settle_time_s = None
        self._peak_after_target = None

    def add(self, temp_c: float, duty_pct: float | None = None,
            *, now: float | None = None) -> None:
        t = time.monotonic() if now is None else now
        self._pts.append((t, float(temp_c), duty_pct))
        self._trim(t)

        if self._target is None or self._target <= 0:
            self._in_band_since = None
            return

        # Track the extreme reached since the step, for overshoot.
        if self._peak_after_target is None:
            self._peak_after_target = float(temp_c)
        else:
            rising = self._start_temp is None or self._target >= self._start_temp
            if rising:
                self._peak_after_target = max(self._peak_after_target, float(temp_c))
            else:
                self._peak_after_target = min(self._peak_after_target, float(temp_c))

        if abs(float(temp_c) - self._target) <= self.band_c:
            if self._first_in_band_at is None:
                self._first_in_band_at = t
            if self._in_band_since is None:
                self._in_band_since = t
            elif (
                self._settle_time_s is None
                and (t - self._in_band_since) >= self.settle_dwell_s
                and self._target_set_at is not None
            ):
                self._settle_time_s = self._in_band_since - self._target_set_at
        else:
            self._in_band_since = None

    def reset(self) -> None:
        self._pts.clear()
        self._target = None
        self._target_set_at = None
        self._start_temp = None
        self._in_band_since = None
        self._first_in_band_at = None
        self._settle_time_s = None
        self._peak_after_target = None

    def _trim(self, now: float) -> None:
        cutoff = now - self.WINDOW_S
        while self._pts and self._pts[0][0] < cutoff:
            self._pts.popleft()

    # ── derived metrics ─────────────────────────────────────────────

    def _rate_c_per_min(self) -> float | None:
        """Least-squares slope over the recent window, in °C/min."""
        if len(self._pts) < self.MIN_RATE_SAMPLES:
            return None
        # Use the most recent ~120 s so the figure is responsive.
        t_end = self._pts[-1][0]
        pts = [(t, v) for t, v, _ in self._pts if t >= t_end - 120.0]
        if len(pts) < self.MIN_RATE_SAMPLES:
            pts = [(t, v) for t, v, _ in self._pts]
        n = len(pts)
        t0 = pts[0][0]
        xs = [p[0] - t0 for p in pts]
        ys = [p[1] for p in pts]
        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx <= 1e-9:
            return None
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        return (sxy / sxx) * 60.0

    def _holding_tail(self, window_s: float) -> list[tuple[float, float, float | None]] | None:
        """
        The trailing ``window_s`` of samples, but only if the zone is actually
        HOLDING rather than still ramping.

        "Holding" means most of the window sits within a few times the settle
        band of target. That tolerates the small excursions a real PID loop makes
        while still refusing to report hold statistics during a ramp, where they
        would be meaningless.
        """
        if self._target is None or self._target <= 0 or not self._pts:
            return None
        if self._first_in_band_at is None:
            return None  # never reached target yet — still ramping
        t_end = self._pts[-1][0]
        # Never look back past the moment we first arrived, so the ramp cannot
        # contaminate hold statistics.
        start = max(t_end - window_s, self._first_in_band_at)
        tail = [p for p in self._pts if p[0] >= start]
        if len(tail) < 4:
            return None
        tol = self.band_c * self.HOLD_BAND_MULTIPLE
        near = sum(1 for _t, v, _d in tail if abs(v - self._target) <= tol)
        if near < self.HOLD_FRACTION * len(tail):
            return None
        return tail

    def _ripple(self) -> tuple[float | None, float | None]:
        """Peak-to-peak and mean temperature while holding."""
        tail = self._holding_tail(self.RIPPLE_WINDOW_S)
        if tail is None:
            return None, None
        vals = [v for _t, v, _d in tail]
        return max(vals) - min(vals), sum(vals) / len(vals)

    def _steady_duty(self) -> float | None:
        """
        Mean heater duty while holding — the heater-adequacy figure.

        At a stable hold, duty equals the fraction of heater power the losses
        consume, so this answers "is the heater big enough?" directly.
        """
        tail = self._holding_tail(self.STEADY_DUTY_WINDOW_S)
        if tail is None:
            return None
        duties = [d for _t, _v, d in tail if d is not None]
        if len(duties) < 4:
            return None
        return sum(duties) / len(duties)

    def _time_constant(self) -> float | None:
        """
        Crude first-order time constant from the approach curve.

        For ``T(t) = T_target - (T_target - T_0) e^{-t/tau}``, the time to cover
        63.2 % of the step IS tau. We look for the first sample past that
        fraction and report the elapsed time. Rough by design — it is a
        sanity-check magnitude ("minutes or hours?"), not a fitted parameter.
        """
        if (
            self._target is None
            or self._start_temp is None
            or self._target_set_at is None
        ):
            return None
        span = self._target - self._start_temp
        if abs(span) < 2.0:
            return None
        threshold = self._start_temp + 0.632 * span
        for t, v, _ in self._pts:
            if t < self._target_set_at:
                continue
            reached = v >= threshold if span > 0 else v <= threshold
            if reached:
                dt = t - self._target_set_at
                return dt if dt > 0 else None
        return None

    def _eta_s(self, temp: float, rate_c_per_min: float | None) -> float | None:
        if self._target is None or rate_c_per_min is None:
            return None
        remaining = self._target - temp
        if abs(remaining) <= self.band_c:
            return 0.0
        # Only meaningful if we are moving the right way, and not trivially slow.
        if remaining * rate_c_per_min <= 0 or abs(rate_c_per_min) < 0.01:
            return None
        return abs(remaining / rate_c_per_min) * 60.0

    @staticmethod
    def _headroom_note(steady_duty: float | None) -> str:
        if steady_duty is None:
            return ""
        if steady_duty >= 95.0:
            return (
                "Heater is saturated at this setpoint — it has no reserve. "
                "Insulate the block or fit a higher-power heater."
            )
        if steady_duty >= 80.0:
            return "Little reserve (>80% duty) — expect poor disturbance rejection."
        if steady_duty >= 40.0:
            return "Comfortable duty with usable reserve."
        return "Plenty of headroom — heater is comfortably oversized for this hold."

    # ── report ──────────────────────────────────────────────────────

    def report(self) -> StabilityReport:
        if not self._pts:
            return StabilityReport(target_c=self._target)

        _, temp, duty = self._pts[-1]
        rate = self._rate_c_per_min()
        ripple, mean = self._ripple()
        steady = self._steady_duty()
        in_band = (
            self._target is not None
            and self._target > 0
            and abs(temp - self._target) <= self.band_c
        )

        overshoot = None
        if (
            self._target is not None
            and self._peak_after_target is not None
            and self._start_temp is not None
        ):
            rising = self._target >= self._start_temp
            raw = (
                self._peak_after_target - self._target
                if rising
                else self._target - self._peak_after_target
            )
            overshoot = max(0.0, raw)

        return StabilityReport(
            temp_c=temp,
            target_c=self._target,
            error_c=None if self._target is None else temp - self._target,
            duty_pct=duty,
            rate_c_per_min=rate,
            eta_s=self._eta_s(temp, rate),
            in_band=in_band,
            settled=self._settle_time_s is not None,
            settle_time_s=self._settle_time_s,
            overshoot_c=overshoot,
            ripple_c=ripple,
            ripple_half_c=None if ripple is None else ripple / 2.0,
            mean_c=mean,
            steady_duty_pct=steady,
            time_constant_s=self._time_constant(),
            headroom_note=self._headroom_note(steady),
            samples=len(self._pts),
        )


def format_duration(seconds: float | None) -> str:
    """Human-friendly duration for the UI ("4 min 12 s", "1 h 06 m")."""
    if seconds is None or not math.isfinite(seconds) or seconds < 0:
        return "—"
    s = int(round(seconds))
    if s < 60:
        return f"{s} s"
    if s < 3600:
        return f"{s // 60} min {s % 60:02d} s"
    return f"{s // 3600} h {(s % 3600) // 60:02d} m"
