"""
power_staircase.py — how much bed power can this rig actually carry?

v7.18 round 6. The symptom that produced this module is not "no heat" but
"the board vanishes the moment the heater starts": ``app.log`` on
2026-08-17 shows ``WriteFile ERROR_BAD_COMMAND`` **0.45 s** after
``M140 S37``, four attempts out of four, and never once while the heater
was off. The bed itself heats fine (22.3 -> 33.3 °C at duty 100/127), so
the useful measurement is the highest power level the USB link survives.

**Marlin has no set-bed-PWM G-code.** ``M140`` sets a TARGET and the
firmware picks the duty, so a partial power level has to be arranged:

``mode="pid"``
    Reduce the bed PID to pure proportional (``M304 P<k> I0 D0``). Then

        pid_output = P * error      (clamped to MAX_BED_POWER, 255)
        B@         = pid_output>>1  (which is why B@ reports 0..127)

    so choosing ``P`` against a known error picks the duty. ``I`` is zeroed
    because integral windup would drift the duty off the level under test;
    ``D`` because it chases sensor noise.

``mode="pwm"``
    Host-side slow PWM: full-power bursts gated on and off, at a period of
    seconds rather than Marlin's ~0.13 s.

⚠ **WHAT A DUTY LEVEL DOES AND DOES NOT LIMIT** (corrected 2026-08-17 by
the bench data below). Marlin's bed heating is TIME-PROPORTIONED soft PWM:
the MOSFET is either fully on or fully off, so the PEAK current at each
switch-on is the SAME at 10% as at 100%. A duty level limits average power
and the WIDTH of each ON pulse (~13 ms at 10% of Marlin's period vs ~400 ms
at 10% of a 4 s host period) — it does not limit the current amplitude the
supply must source. So the two modes separate *pulse width* sensitivity,
not peak from average:

  * dies at a LOW rung in either mode  -> a single full-current pulse is
    already too much. No software level can fix that; the current is set
    entirely by the heater resistance (I = V/R). Measure R.
  * survives short pulses, dies on long ones -> the supply rides through on
    its bulk capacitance but cannot sustain the load. Capacity problem.

**ME3B V1, 2026-08-17: it died at the 10% rung, 258 ms after ``M140 S45``
was accepted, with the bed never reporting any duty at all** — the first
case above.

This module is pure: no serial, no Qt, no I/O. It is the single home for
the arithmetic AND for the interpretation, shared by
``tools_incubator_heater_diagnostic.py`` (bench, app closed) and the
Incubator page's Power-staircase tab (in-app, over the shared ZP link).
Two copies of a verdict is how two surfaces come to disagree about the
same measurement.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .marlin_gcode import HEATER_PWM_FULL_SCALE

#: ``B@`` is reported on a 0..127 scale (Marlin's ``soft_pwm_amount``).
#: Aliased, never re-declared: ``marlin_gcode`` already owns this number for
#: the duty bar, the trend plot and the heater-adequacy metric, and a second
#: copy is how two surfaces come to disagree about what "100%" means.
DUTY_FULL = HEATER_PWM_FULL_SCALE

#: A rung only tests anything if the firmware actually DELIVERS the duty
#: asked for. The error shrinks as the plate warms, and at the ceiling the
#: duty collapses to zero — calling that "survived 100%" would be a false
#: all-clear, the worst answer a diagnostic can give. Below this fraction of
#: the request a rung is reported NOT DELIVERED, never as survived.
DELIVERED_FRAC = 0.6

_RE_M304 = re.compile(
    r"M304\s+P\s*(-?[\d.]+)\s+I\s*(-?[\d.]+)\s+D\s*(-?[\d.]+)", re.I)


def parse_bed_pid(lines) -> tuple[float, float, float] | None:
    """Pull (P, I, D) out of an M503 dump, or None if the build has no M304."""
    for text in lines or ():
        m = _RE_M304.search(str(text))
        if m:
            return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None


def p_gain_for_duty(duty_pct: float, error_c: float) -> float:
    """P that should produce ``duty_pct`` of full power at ``error_c`` error.

    Inverts the relation in the module docstring. Only ever a STARTING
    guess — the measured ``B@`` is what gets reported, and the caller trims
    P once against it, because MAX_BED_POWER and the ``>>1`` are build-time
    choices we should not assume.
    """
    want = max(0.0, min(100.0, float(duty_pct))) / 100.0 * DUTY_FULL
    return max(0.01, min(500.0, 2.0 * want / max(0.5, float(error_c))))


def plan_staircase(spec) -> list[int]:
    """``"10,25,50,100"`` (or a sequence) -> ``[10, 25, 50, 100]``.

    Sorted, de-duped, 1..100 only. Ascending is not cosmetic: the run stops
    at the first rung that kills the link, so "the highest surviving level"
    is only meaningful if every lower one was tried first.
    """
    parts = (spec if isinstance(spec, (list, tuple))
             else str(spec).replace(" ", "").split(","))
    out: set[int] = set()
    for part in parts:
        if part == "" or part is None:
            continue
        try:
            v = int(round(float(part)))
        except (TypeError, ValueError):
            raise ValueError(f"not a duty percentage: {part!r}")
        if not 1 <= v <= 100:
            raise ValueError(f"duty out of range 1-100: {v}")
        out.add(v)
    if not out:
        raise ValueError("no rungs given")
    return sorted(out)


#: How far a rung's measured duty may exceed its request before the level is
#: reported as NOT limited. A firmware that ignores the gains (bang-bang bed,
#: or a build where P has no authority) runs every rung at full power, and an
#: operator who believes they asked for 10% would have applied 100%.
OVERSHOOT_BAND = 0.25


def rung_delivered(pct: int, max_duty: int,
                   frac: float = DELIVERED_FRAC) -> bool:
    """Did the firmware actually drive the power this rung asked for?"""
    return max_duty >= frac * (pct / 100.0 * DUTY_FULL)


#: A rung at or above this duty, held at least OPEN_MIN_SECONDS, must move
#: the temperature. ME3B V1 measured +11 C in 18 s at full duty, so a flat
#: reading under a sustained high duty is not a slow heater -- it is an open
#: circuit.
OPEN_MIN_DUTY_PCT = 50.0
OPEN_MIN_SECONDS = 8.0
OPEN_MIN_RISE_C = 0.5


def rung_open_circuit(duty_pct: float, seconds: float, rise_c: float) -> bool:
    """Commanded real power, for long enough, and nothing got warm?

    ``@:``/``B@:`` is the duty Marlin COMMANDS -- it is not a measurement of
    current, and Marlin reports it just as happily into an open circuit.
    Bench 2026-08-17: HE0 reported 127/127 for 20 s with the heater
    UNPLUGGED, no temperature change and no supply sag. Reporting that as
    "survived 100%" would tell the operator their supply carries full power
    when nothing was connected at all -- the same false all-clear this
    module already guards against on the other side.

    Deliberately conservative: only a HIGH duty held for a real interval
    counts, because a genuine heater at 3% for two seconds legitimately
    shows nothing.
    """
    return (duty_pct >= OPEN_MIN_DUTY_PCT
            and seconds >= OPEN_MIN_SECONDS
            and rise_c < OPEN_MIN_RISE_C)


def rung_overshot(pct: int, max_duty: int,
                  band: float = OVERSHOOT_BAND) -> bool:
    """Did the firmware run this rung far ABOVE the level asked for?

    Not a failure of the link — a failure of the LIMITING, which matters
    because the operator is told what power they are applying.
    """
    return max_duty > (pct / 100.0 * DUTY_FULL) + band * DUTY_FULL


@dataclass
class StaircaseOutcome:
    """Everything the verdict needs. Filled in by whichever surface ran it."""

    mode: str = "pid"
    supply_v: float = 12.0
    #: (requested %, measured peak B@) for rungs that loaded the link.
    survived: list[tuple[int, int]] = field(default_factory=list)
    #: (requested %, measured peak B@) for rungs the firmware never drove.
    undelivered: list[tuple[int, int]] = field(default_factory=list)
    #: (requested %, duty reached, kind, detail) — kind in
    #: dropped/fault/silent/refused/garbled.
    died_at: tuple[int, int, str, str] | None = None
    #: Stopped on the temperature limit rather than on a link failure.
    aborted_hot: bool = False
    #: Operator pressed stop.
    cancelled: bool = False
    #: Rungs that commanded real power for a real interval and produced NO
    #: temperature change: (requested %, measured peak B@, rise C).
    no_heat: list[tuple[int, int, float]] = field(default_factory=list)

    @property
    def top_surviving_pct(self) -> int | None:
        return self.survived[-1][0] if self.survived else None


def verdict_lines(out: StaircaseOutcome) -> list[str]:
    """The interpretation, as plain ASCII lines. ONE home, two surfaces."""
    v = out.supply_v
    lines: list[str] = []
    if out.survived:
        pct, duty = out.survived[-1]
        lines.append(
            f"Highest level SURVIVED: {pct}% requested, measured duty "
            f"{duty}/{DUTY_FULL:.0f} ({100.0 * duty / DUTY_FULL:.0f}% power).")
    elif out.died_at is not None:
        lines.append(
            "No rung survived -- the link failed at the lowest level tried.")
    elif not out.cancelled:
        lines.append("NOTHING WAS PROVEN: no rung delivered the power it "
                     "asked for, so the link was never actually loaded.")

    if out.undelivered:
        lines.append("")
        lines.append("Not delivered (peak duty far below the request, so "
                     "these rungs are inconclusive rather than passes):")
        for pct, duty in out.undelivered:
            lines.append(f"   {pct:3d}% requested -> only {duty}/{DUTY_FULL:.0f} "
                         f"measured")
        lines.append("Usually the bed reached the ceiling: raise the ceiling, "
                     "or let it cool between runs. Duty stuck at 0 with plenty "
                     "of headroom is an UPSTREAM fault instead -- bed disabled "
                     "in the build, or a latched thermal fault.")

    if out.no_heat:
        lines.append("")
        lines.append("!! NOTHING WAS DRAWING. These rungs commanded real "
                     "power and the temperature did not move:")
        for pct, duty, rise in out.no_heat:
            lines.append(f"   {pct:3d}% requested, duty {duty}/{DUTY_FULL:.0f} "
                         f"delivered, temperature {rise:+.2f} C")
        lines.append("The duty field is what the firmware COMMANDS, not proof "
                     "that current flowed -- it reads the same into an open "
                     "circuit. So the link was never loaded and any survival "
                     "above means nothing about the supply. Check that the "
                     "heater is really landed on this zone's output "
                     "(clamped on bare copper, not a fan header), and that "
                     "the probe is on the input that regulates it.")

    over = [(p, d) for p, d in out.survived if rung_overshot(p, d)]
    if over and out.mode == "pid":
        lines.append("")
        lines.append("!! THE LEVEL WAS NOT LIMITED. These rungs ran far above "
                     "the power they asked for:")
        for pct, duty in over:
            lines.append(f"   {pct:3d}% requested -> {duty}/{DUTY_FULL:.0f} "
                         f"({100.0 * duty / DUTY_FULL:.0f}%) measured")
        lines.append("The firmware is not honouring the proportional gain, so "
                     "every rung was effectively full power and the staircase "
                     "did not step. Treat the levels above as 100%, and read "
                     "the survival result as full power only.")

    if out.cancelled:
        lines.append("")
        lines.append("Stopped by the operator. Anything above is still valid "
                     "as a lower bound.")
        return lines

    if out.aborted_hot:
        lines.append("")
        lines.append("Stopped on the TEMPERATURE limit, not on a link "
                     "failure -- the heater is delivering heat. Whatever was "
                     "reached above is a lower bound on what this rig carries.")
        return lines

    if out.died_at is None:
        lines.append("")
        if out.survived:
            lines.append("The link survived every rung it actually loaded.")
        if out.top_surviving_pct is not None and out.top_surviving_pct >= 100:
            lines.append(
                "So the supply CAN carry full bed power. If the failure still "
                "happens in normal use, the remaining variable is the traffic "
                "sharing this port -- the ZP position poller's M114 at ~3 Hz.")
        elif out.survived:
            lines.append("Re-run with a higher top rung to find the ceiling.")
        return lines

    pct, duty, kind, detail = out.died_at
    lines.append("")
    lines.append(f"FAILED at the {pct}% rung (duty reached {duty}/{DUTY_FULL:.0f} "
                 f"before it went).")
    lines.append(f"Failure mode: {kind} -- {detail}")
    lines.append("")
    if kind == "dropped":
        lines.append("'dropped' means the PORT died, not the firmware: the "
                     "board left USB. Switching the heater pulled the "
                     f"{v:g} V rail down far enough to reset the MCU.")
        lines.append("")
        lines.append("With everything powered OFF, measure the heater "
                     "resistance.")
        lines.append(f"At {v:g} V:  P = {v * v:g}/R  and  I = {v:g}/R.")
        for watts in (30, 60, 120):
            ohms = (v * v) / watts
            lines.append(f"   {ohms:5.2f} ohm -> {watts:3d} W, "
                         f"{v / ohms:5.2f} A")
        lines.append("Compare that current against the PSU rating AND the "
                     "board's bed-output rating. A reading far below the "
                     "film's nameplate means a partial short.")
        if pct <= 25:
            lines.append("")
            lines.append(f"It died at only {pct}%. Marlin's bed PWM is "
                         f"time-proportioned -- the MOSFET is fully ON during "
                         f"each pulse -- so a low duty does NOT reduce the "
                         f"current the supply must source, only how long it "
                         f"must source it. Dying this low means ONE full "
                         f"pulse is already too much, and no software level "
                         f"will fix it. The current is set entirely by the "
                         f"heater resistance: measure R FIRST.")
        elif out.mode == "pid":
            lines.append("")
            lines.append(f"It carried every rung below {pct}%, so the supply "
                         f"rides out short pulses but cannot sustain the "
                         f"load. That is a capacity problem rather than a "
                         f"short.")
        elif out.mode == "pid":
            lines.append("")
            lines.append("It carried every lower rung, so the supply is close "
                         "but cannot deliver the last of it. Capping duty at "
                         "the highest surviving level is a real fix; a bigger "
                         "PSU is the better one.")
        if out.mode == "pwm":
            lines.append("")
            lines.append("This mode uses long full-current bursts. Re-run in "
                         "pid mode at the same rung, which uses much shorter "
                         "ones: if that survives, the supply is riding "
                         "through on its bulk capacitance and the fault is "
                         "capacity, not a short.")
    elif kind == "refused":
        lines.append("The board ANSWERED and said no. That is a control-path "
                     "verdict, not a power fault -- the zone named in the "
                     "message is not the one with the heater on it.")
    elif kind in ("fault", "silent"):
        lines.append("This is the FIRMWARE stopping, not the supply: the port "
                     "was still open. Marlin has almost certainly called "
                     "kill() and needs a POWER CYCLE. The stock "
                     "thermal-protection windows are sized for a 3D printer "
                     "bed, not this load.")
    return lines
