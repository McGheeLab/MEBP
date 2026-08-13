"""
safety.py — host-side guardrails.

Scope discipline matters here. **The firmware owns the safety loop**, not this
tool: Marlin's ``THERMAL_PROTECTION_*`` watchdogs, MINTEMP/MAXTEMP limits and
``kill()`` are what actually protect the hardware, and they keep working when
the host is closed, crashed, or unplugged. Everything in this file is a
convenience layer that reduces the chance of an operator mistake and makes a
firmware fault legible — it is explicitly NOT a substitute for the firmware
protections, and the UI says so.

What lives here:
  * a hard setpoint ceiling (a 37 C water bath has no business at 200 C),
  * a fault latch, so a firmware fault stays visible and blocks further
    commands until acknowledged rather than scrolling past,
  * a generic channel-vs-channel divergence check, dormant in phase 1 (each
    zone has a single sensor) and ready for the phase-2 sensor box.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from .marlin_gcode import Fault

# ═══════════════════════════════════════════════════════════════════
# Setpoint limits
# ═══════════════════════════════════════════════════════════════════

#: Hard ceiling. The application is a 37 C cell-culture bath; anything much
#: above this is either a mistake or a boiling/steam hazard given the medium is
#: water. Raise deliberately, in source, with the physical rig in mind.
MAX_SETPOINT_C = 50.0

#: Above this, ask the operator to confirm.
CAUTION_SETPOINT_C = 42.0

#: A jump larger than this warrants confirmation.
CONFIRM_JUMP_C = 10.0

#: Below this the heater is simply off; we never command "cooling" (there is
#: none — the rig can only heat and then lose heat to ambient).
MIN_SETPOINT_C = 0.0


@dataclass(frozen=True)
class SetpointCheck:
    """Result of validating a requested setpoint."""

    requested_c: float
    allowed_c: float
    clamped: bool
    needs_confirm: bool
    reason: str = ""

    @property
    def blocked(self) -> bool:
        return False  # clamping never blocks outright; it lowers the value


def check_setpoint(
    requested_c: float,
    *,
    current_target_c: float | None = None,
    max_c: float = MAX_SETPOINT_C,
) -> SetpointCheck:
    """Clamp to the ceiling and decide whether confirmation is warranted."""
    req = float(requested_c)
    allowed = max(MIN_SETPOINT_C, min(max_c, req))
    clamped = abs(allowed - req) > 1e-9

    reasons = []
    if clamped:
        reasons.append(f"clamped to the {max_c:.0f} °C software ceiling")

    needs_confirm = False
    if allowed > CAUTION_SETPOINT_C:
        needs_confirm = True
        reasons.append(
            f"above the {CAUTION_SETPOINT_C:.0f} °C caution threshold for a "
            f"water-filled vessel"
        )
    if current_target_c is not None and allowed - float(current_target_c) > CONFIRM_JUMP_C:
        needs_confirm = True
        reasons.append(
            f"a jump of more than {CONFIRM_JUMP_C:.0f} °C from the current target"
        )

    return SetpointCheck(
        requested_c=req,
        allowed_c=allowed,
        clamped=clamped,
        needs_confirm=needs_confirm,
        reason="; ".join(reasons),
    )


# ═══════════════════════════════════════════════════════════════════
# Fault latch
# ═══════════════════════════════════════════════════════════════════

@dataclass
class LatchedFault:
    fault: Fault
    zone_id: str | None
    at: float = field(default_factory=time.time)

    @property
    def summary(self) -> str:
        who = f" on {self.zone_id}" if self.zone_id else ""
        return f"{self.fault.kind.replace('_', ' ')}{who}"


class FaultLatch:
    """
    Holds the first fault seen until explicitly acknowledged.

    A latch (rather than a transient banner) is the right shape because Marlin
    stops acknowledging G-code after ``kill()`` — every subsequent command will
    silently time out, and an operator who missed a scrolling message would have
    no idea why. Latched state also lets us refuse to send further heater
    commands, which would otherwise look like the tool is broken.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._latched: LatchedFault | None = None

    def record(self, fault: Fault, zone_id: str | None) -> bool:
        """Latch a fault. Returns True if this was the first (newly latched)."""
        with self._lock:
            if self._latched is not None:
                return False
            self._latched = LatchedFault(fault=fault, zone_id=zone_id)
            return True

    def acknowledge(self) -> None:
        with self._lock:
            self._latched = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._latched is not None

    @property
    def current(self) -> LatchedFault | None:
        with self._lock:
            return self._latched

    def blocking_reason(self) -> str:
        with self._lock:
            if self._latched is None:
                return ""
            f = self._latched.fault
            return (
                f"Heater fault latched ({self._latched.summary}). The board has "
                f"halted and will ignore commands until it is POWER-CYCLED. "
                f"{f.hint}"
            )


# ═══════════════════════════════════════════════════════════════════
# Sensor divergence (phase-2 seam)
# ═══════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class DivergenceEvent:
    channel_a: str
    channel_b: str
    value_a: float
    value_b: float
    delta_c: float
    threshold_c: float

    @property
    def message(self) -> str:
        return (
            f"Sensors disagree by {self.delta_c:.2f} °C "
            f"({self.channel_a} {self.value_a:.2f} vs "
            f"{self.channel_b} {self.value_b:.2f}), "
            f"threshold {self.threshold_c:.2f} °C"
        )


class SensorDivergenceMonitor:
    """
    Compares two named channels and reports when they disagree too much.

    Unconfigured in phase 1: with one sensor per zone there is nothing to
    cross-check. Once the sensor box supplies an independent reading of the same
    physical thing, configure a pair here and a drifting or detached thermistor
    becomes detectable — which is the realistic failure mode for a film heater
    bonded to a block.

    ``require_consecutive`` avoids alarming on a single noisy sample; ``action``
    is advisory and interpreted by the controller (``"alarm"`` or
    ``"alarm_and_off"``).
    """

    def __init__(
        self,
        *,
        channel_a: str = "",
        channel_b: str = "",
        threshold_c: float = 2.0,
        require_consecutive: int = 3,
        action: str = "alarm",
        enabled: bool = False,
    ):
        self.channel_a = channel_a
        self.channel_b = channel_b
        self.threshold_c = float(threshold_c)
        self.require_consecutive = max(1, int(require_consecutive))
        self.action = action
        self.enabled = enabled
        self._streak = 0
        self._fired = False

    @property
    def configured(self) -> bool:
        return bool(self.enabled and self.channel_a and self.channel_b)

    def configure(
        self,
        channel_a: str,
        channel_b: str,
        *,
        threshold_c: float | None = None,
        action: str | None = None,
    ) -> None:
        self.channel_a = channel_a
        self.channel_b = channel_b
        if threshold_c is not None:
            self.threshold_c = float(threshold_c)
        if action is not None:
            self.action = action
        self.enabled = True
        self.reset()

    def disable(self) -> None:
        self.enabled = False
        self.reset()

    def reset(self) -> None:
        self._streak = 0
        self._fired = False

    def check(self, values: dict[str, float]) -> DivergenceEvent | None:
        """
        Evaluate one sample. ``values`` maps channel uid -> corrected °C.

        Returns an event only on the sample that crosses the consecutive
        threshold, so the caller alarms once rather than every tick.
        """
        if not self.configured:
            return None
        a = values.get(self.channel_a)
        b = values.get(self.channel_b)
        if a is None or b is None:
            self._streak = 0
            return None

        delta = abs(float(a) - float(b))
        if delta <= self.threshold_c:
            self._streak = 0
            self._fired = False
            return None

        self._streak += 1
        if self._streak < self.require_consecutive or self._fired:
            return None

        self._fired = True
        return DivergenceEvent(
            channel_a=self.channel_a,
            channel_b=self.channel_b,
            value_a=float(a),
            value_b=float(b),
            delta_c=delta,
            threshold_c=self.threshold_c,
        )
