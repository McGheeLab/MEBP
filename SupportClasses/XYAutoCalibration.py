"""XYAutoCalibration.py — one-shot, systematic XY motion calibration.

THE CENTRAL IDEA
────────────────
The old auto-tune SEARCHED for parameters by driving shapes and minimising a
score. That was slow (a velocity tune was 26 hardware runs), non-repeatable
(each candidate evaluated once against ~73 µm of run-to-run noise), and — worst —
it searched over parameters that are not free variables at all.

Almost every follower parameter is a DERIVED CONSEQUENCE of three measurable
machine properties:

    loop_period   — how fast we can close the loop      (measure_control_loop_rate)
    dead_time     — command → motion transport delay    (XYDeadTime)
    top_speed     — what the stage can actually deliver (XYSpeedProbe / dead-time cruise)

From those, closed-form physics fixes the rest:

    L          = dead_time + loop_period              the effective delay
    Ku         = π/(2L)     Tu = 4L                   the cross-track plant is a
                                                      pure integrator + delay
    kp         = 0.33·Ku                              Ziegler–Nichols over Ku
    speed_cap  = lookahead/(L·safety)                 pure-pursuit stability
    lookahead  = target_speed·L·safety                ...solved for the lookahead
                                                      that ALLOWS the target speed
    decel      ≥ target_speed·L                       the stage coasts for one dead
                                                      time after the ramp begins

So the honest calibration is **measure → derive → verify**, not *search*. Search
is reserved for the one or two properties that genuinely are not derivable
(the lateral-acceleration limit), and even then as a bounded confirmation rather
than an open-ended sweep. That is what makes it a one-click, systematic,
repeatable process: the same machine measured twice yields the same settings.

This module is PURE — no Qt, no serial, no numpy — so the whole derivation and
its validation are unit-testable without hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

from SupportClasses import VelocityControl as VC


def _ceil_to(value: float, places: int) -> float:
    """Round UP to ``places`` decimals.

    Used wherever rounding down would make a derived setting fall just short of
    what it was solved for (e.g. a lookahead that no longer permits the target
    speed). Prefer being a hair generous over silently missing the target.
    """
    f = 10.0 ** places
    return math.ceil(value * f - 1e-9) / f


# ── Inputs ────────────────────────────────────────────────────────────

@dataclass
class MeasuredMachine:
    """What the probes measured. Zeros mean "not measured yet"."""
    control_loop_ms: float = 0.0
    dead_time_s: float = 0.0
    dead_time_spread_s: float = 0.0
    tau_s: float = 0.0                  # first-order rise time from the step
    top_speed_um_s: float = 0.0
    cruise_um_s: float = 0.0            # independent cross-check of top_speed
    backlash_um: float = 0.0
    lateral_accel_mm_s2: float = 0.0    # 0 = not measured (searched separately)
    declared_max_speed_um_s: float = 0.0  # from the controller JSON, for contrast

    def effective_delay_s(self) -> float:
        """``L`` — the delay the pure-pursuit stability limit depends on."""
        return max(0.0, self.dead_time_s) + max(0.0, self.control_loop_ms) / 1000.0

    def is_complete(self) -> bool:
        return (self.control_loop_ms > 0 and self.dead_time_s > 0
                and self.top_speed_um_s > 0)

    def missing(self) -> list:
        out = []
        if self.control_loop_ms <= 0:
            out.append("comms rate")
        if self.dead_time_s <= 0:
            out.append("dead time")
        if self.top_speed_um_s <= 0:
            out.append("top speed")
        return out


@dataclass
class CalibrationPolicy:
    """The choices a human makes; everything else is derived.

    ``target_speed_mm_s`` is the headline: "how fast do you want to print?".
    The derivation then sizes the lookahead, the decel and the gains to make that
    speed achievable and stable, instead of silently throttling to whatever the
    lookahead happened to permit.
    """
    target_speed_mm_s: float = 5.0
    resolution_um: float = 30.0
    #: Pure-pursuit stability factor used at RUN time. 2.0 = travel at most half a
    #: lookahead per effective delay (the follower's long-standing default).
    safety: float = 2.0
    #: Extra headroom applied when SIZING the lookahead, so the operating point
    #: sits comfortably inside the stability limit instead of exactly on it.
    #:
    #: Solving `lookahead = target·L·safety` puts the achievable cap EQUAL to the
    #: target — i.e. right on the boundary — and a stage has dynamics beyond the
    #: measured delay (the first-order rise, backlash, a slightly optimistic
    #: apparent-lag estimate). A bench simulation at the boundary dithered badly;
    #: with headroom it has somewhere to give.
    lookahead_margin: float = 1.5
    #: Fraction of the dead time the follower compensates by predicting ahead.
    #: 0 until the lead predictor is verified on hardware (Stage 3.5).
    lead_time_frac: float = 0.0
    #: Never command more than this fraction of the MEASURED top speed.
    max_speed_frac: float = 0.9
    #: Ziegler–Nichols aggressiveness + an extra divisor on the gains.
    zn_method: str = "some_overshoot"
    gain_margin: float = 1.0
    #: Leave the derivative at zero until the derivative low-pass filter is
    #: consumed by the control law — differentiating a µm-quantised encoder at
    #: 25 Hz injects more noise than the D term removes.
    use_kd: bool = False
    #: Pin the straight-line speed so corners are the ONLY modulation.
    hold_speed: bool = True
    #: Turn angle counted as a corner.
    corner_angle_deg: float = 30.0
    #: Enable the Stage-3 command-shaping / recovery guards in the derived set.
    include_guards: bool = True


@dataclass
class Note:
    """One derived parameter and the reason for its value — so the tool can show
    its work instead of emitting a magic number."""
    param: str
    value: float
    reason: str


@dataclass
class Warning_:
    level: str        # "error" | "warn" | "info"
    message: str


@dataclass
class Derived:
    values: dict = field(default_factory=dict)     # → store "velocity" bucket
    notes: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    summary: dict = field(default_factory=dict)    # headline derived quantities

    def note_text(self) -> list:
        return [f"{n.param} = {n.value:g}  ← {n.reason}" for n in self.notes]


# ── The derivation ────────────────────────────────────────────────────

def derive_settings(m: MeasuredMachine, p: CalibrationPolicy) -> Derived:
    """Compute every derivable follower parameter from the measurements.

    Deterministic: same measurements + same policy → same settings, always.
    """
    d = Derived()
    add = d.notes.append
    warn = d.warnings.append

    if not m.is_complete():
        warn(Warning_("error", "Not calibrated — still need: "
                               + ", ".join(m.missing())))
        return d

    L = m.effective_delay_s()
    loop_s = m.control_loop_ms / 1000.0
    top_mm_s = m.top_speed_um_s / 1000.0
    safety = max(1.0, p.safety)
    res_mm = max(1e-4, p.resolution_um / 1000.0)

    # ── 1. Loop rate ──
    control_hz = max(5.0, min(60.0, 1000.0 / m.control_loop_ms))
    d.values["control_hz"] = round(control_hz, 1)
    add(Note("control_hz", d.values["control_hz"],
             f"1000/{m.control_loop_ms:.2f} ms measured closed-loop period"))

    # ── 2. Achievable target speed ──
    speed_ceiling = top_mm_s * p.max_speed_frac
    target = min(p.target_speed_mm_s, speed_ceiling)
    if target < p.target_speed_mm_s - 1e-9:
        warn(Warning_("warn",
                      f"Target {p.target_speed_mm_s:.2f} mm/s exceeds "
                      f"{p.max_speed_frac:.0%} of the measured top speed "
                      f"({top_mm_s:.2f} mm/s) — capped to {target:.2f} mm/s."))
    d.values["max_speed_frac"] = p.max_speed_frac
    add(Note("max_speed_frac", p.max_speed_frac,
             f"never command above {p.max_speed_frac:.0%} of the measured "
             f"{top_mm_s:.2f} mm/s top speed (the legacy code never clamped by "
             f"it, so an over-command became a silent gain error)"))

    # ── 3. Dead-time compensation ──
    lead = min(1.0, max(0.0, p.lead_time_frac))
    residual_L = loop_s + m.dead_time_s * (1.0 - lead)
    d.values["lead_time_frac"] = lead
    d.values["deadtime_safety"] = safety
    add(Note("lead_time_frac", lead,
             "share of the dead time removed by prediction" if lead
             else "0 = no prediction credit until the lead predictor is "
                  "hardware-verified (conservative)"))

    # ── 4. Lookahead — SOLVED for the target speed, not guessed ──
    # This is the crux. The legacy code let the lookahead SET the speed
    # (cap = la/(L·safety)), so tuning the lookahead was really tuning speed and
    # any error-minimising search drove it to the smallest value in its grid.
    # Inverting the same relation sizes the lookahead FROM the speed we want.
    margin = max(1.0, float(p.lookahead_margin or 1.0))
    la_needed = target * residual_L * safety * margin
    la_floor = 2.0 * res_mm            # below the noise floor it is meaningless
    # Round UP, never to-nearest: rounding down would leave the achievable cap a
    # hair below the target, so the run would silently print slightly slow than
    # asked and `cap_reason` would read "dead_time" instead of "print_speed".
    lookahead = _ceil_to(max(la_needed, la_floor), 3)
    d.values["lookahead_mm"] = lookahead
    add(Note("lookahead_mm", lookahead,
             f"target {target:.2f} mm/s × residual delay {residual_L * 1000:.0f} ms "
             f"× safety {safety:g} × {margin:g} headroom (floored at 2× the "
             f"{p.resolution_um:.0f} µm resolution element, rounded up). The "
             f"headroom keeps the operating point inside the stability limit "
             f"rather than exactly on it."))
    # Tell resolve_control to size it the same way at run time. Derived from the
    # ROUNDED lookahead, and rounded up too, so the runtime re-derivation can
    # never come out below what we just computed.
    min_la_frac = (lookahead / (target * L)) if (target * L) > 1e-9 else 0.0
    d.values["min_lookahead_frac"] = _ceil_to(min_la_frac, 3)
    add(Note("min_lookahead_frac", d.values["min_lookahead_frac"],
             "so the runtime re-derives the same lookahead from the dynamics "
             "instead of the lookahead dictating the speed"))

    # ── 5. hold_speed — corners become the only modulation ──
    achievable_cap = lookahead / (residual_L * safety) if residual_L > 1e-9 else target
    can_hold = achievable_cap + 1e-9 >= target
    d.values["hold_speed"] = 1.0 if (p.hold_speed and can_hold) else 0.0
    if p.hold_speed and can_hold:
        add(Note("hold_speed", 1.0,
                 f"the derived lookahead supports {achievable_cap:.2f} mm/s ≥ the "
                 f"{target:.2f} mm/s target, so straights can run at full speed "
                 f"and corners are the only slowdown"))
    elif p.hold_speed:
        warn(Warning_("warn",
                      f"hold_speed left OFF: the stability limit only allows "
                      f"{achievable_cap:.2f} mm/s at this lookahead."))

    # ── 6. Cross-track gains — analytic, not searched ──
    g = VC.pid_gains_from_dead_time(m.dead_time_s, m.control_loop_ms,
                                    method=p.zn_method,
                                    gain_margin=p.gain_margin,
                                    use_kd=p.use_kd)
    # ⚠ Only APPLY a cross-track gain if the control law can bound it. While
    # pursuit_step still adds the PD correction to a full-magnitude pursuit
    # vector with no re-clamp, a correct gain produces a mostly-perpendicular
    # command (76 % sideways at the logged 0.64 mm error) and the follower runs
    # away — so an automated calibration must NOT hand the machine that gain.
    # Pure pursuit at the right SPEED is already the large win; the gain switches
    # on with the bounded composition.
    gains_safe = bool(getattr(VC, "SUPPORTS_BOUNDED_CROSS_TRACK", False)) \
        and bool(p.include_guards)
    d.values["pid_kp"] = round(g["kp"], 3) if gains_safe else 0.0
    d.values["pid_kd"] = round(g["kd"], 4) if gains_safe else 0.0
    d.values["pid_ki"] = 0.0
    if gains_safe:
        add(Note("pid_kp", d.values["pid_kp"],
                 f"Ziegler–Nichols {p.zn_method} over the ANALYTIC Ku = π/(2L) = "
                 f"{g['ku']:.2f} (L = {L * 1000:.0f} ms); stability limit is "
                 f"kp < {g['kp_stability_limit']:.2f}"))
    else:
        add(Note("pid_kp", 0.0,
                 f"0 = PURE PURSUIT. The analytic gain would be "
                 f"{g['kp']:.2f} (Ku = π/(2L) = {g['ku']:.2f}), but the control "
                 f"law does not yet bound the cross-track correction relative to "
                 f"the commanded speed, so applying it would steer mostly "
                 f"sideways. Speed and geometry are calibrated regardless."))
    add(Note("pid_kd", d.values["pid_kd"],
             "0 — unfiltered D on a µm-quantised encoder at "
             f"{control_hz:.0f} Hz injects more noise than it removes"
             if not p.use_kd else f"0.11·Ku·Tu with Tu = 4L = {g['tu']:.3f} s"))
    add(Note("pid_ki", 0.0,
             "the plant is already an integrator, so P alone has zero "
             "steady-state cross-track error and a second integrator is a "
             "classic limit-cycle source"))

    # ── 7. Derivative filter corner ──
    d.values["d_filter_hz"] = round(max(1.0, control_hz / 8.0), 1)
    add(Note("d_filter_hz", d.values["d_filter_hz"],
             f"control rate {control_hz:.0f} Hz ÷ 8 — above the closed-loop "
             f"bandwidth, well below Nyquist"))

    # ── 8. End-of-path decel — must cover the COAST distance ──
    # After the ramp starts the stage keeps moving for one dead time. A decel
    # window shorter than that guarantees endpoint overshoot.
    coast_mm = target * L
    accel_eff = (target / m.tau_s) if m.tau_s > 1e-6 else 0.0
    accel_mm = (target * target / (2.0 * accel_eff)) if accel_eff > 1e-9 else 0.0
    decel = max(coast_mm + accel_mm, 3.0 * res_mm)
    d.values["decel_mm"] = round(decel, 3)
    add(Note("decel_mm", d.values["decel_mm"],
             f"coast {coast_mm * 1000:.0f} µm (target × L) + ramp "
             f"{accel_mm * 1000:.0f} µm (τ = {m.tau_s * 1000:.0f} ms) — a shorter "
             f"window guarantees endpoint overshoot"))

    # ── 9. Corner scheduling ──
    d.values["corner_angle_deg"] = p.corner_angle_deg
    add(Note("corner_angle_deg", p.corner_angle_deg,
             "turn angle counted as a corner"))
    if m.lateral_accel_mm_s2 > 0:
        # v ≤ sqrt(a_lat·R); the tightest radius the follower can hold is set by
        # the lookahead, so that fixes the corner speed fraction.
        v_corner = math.sqrt(m.lateral_accel_mm_s2 * lookahead)
        csf = max(0.05, min(1.0, v_corner / target)) if target > 1e-9 else 0.4
        d.values["corner_speed_factor"] = round(csf, 3)
        add(Note("corner_speed_factor", d.values["corner_speed_factor"],
                 f"√(a_lat {m.lateral_accel_mm_s2:.0f} mm/s² × lookahead "
                 f"{lookahead:.2f} mm) = {v_corner:.2f} mm/s at a sharp corner"))
    else:
        d.values["corner_speed_factor"] = 0.4
        add(Note("corner_speed_factor", 0.4,
                 "default — the lateral-acceleration limit has not been measured, "
                 "so this is the one parameter still worth a bounded search"))

    # ── 10. Confirm-mode tolerance ──
    d.values["settle_tol_um"] = round(p.resolution_um, 1)
    add(Note("settle_tol_um", d.values["settle_tol_um"],
             "arrival tolerance = the resolution element (tighter is chasing "
             "noise, looser prints a visibly wrong corner)"))

    # ── 11. Stage-3 command shaping + recovery guards ──
    if p.include_guards:
        d.values["normal_bound_frac"] = 0.5
        d.values["min_forward_frac"] = 0.35
        add(Note("normal_bound_frac", 0.5,
                 "cap the cross-track correction at half the commanded speed so "
                 "it can never dominate forward motion (the deadlock was a 76 % "
                 "perpendicular command)"))
        add(Note("min_forward_frac", 0.35,
                 "guarantee forward progress so the follower cannot stall in "
                 "place"))
        d.values["reacquire_cross_mm"] = round(max(5.0 * res_mm,
                                                   0.25 * lookahead), 3)
        d.values["reacquire_ticks"] = 5.0
        d.values["reacquire_window_mm"] = round(4.0 * lookahead, 3)
        d.values["reacquire_speed_frac"] = 0.35
        d.values["reacquire_max_s"] = 3.0
        d.values["reacquire_back_step_mm"] = round(0.5 * lookahead, 3)
        add(Note("reacquire_cross_mm", d.values["reacquire_cross_mm"],
                 "enter recovery past this cross-track error instead of "
                 "deadlocking off-path"))
        d.values["stall_ds_frac"] = 0.15
        d.values["stall_ticks"] = round(max(5.0, control_hz), 0)
        d.values["dither_ratio_max"] = 3.0
        d.values["dither_min_mm"] = 1.0
        add(Note("stall_ticks", d.values["stall_ticks"],
                 f"≈1 s at {control_hz:.0f} Hz — the logged deadlock ran 37 s "
                 f"before a human noticed"))
        d.values["max_cross_track_mm"] = round(max(0.5, 10.0 * res_mm), 3)
        d.values["runaway_ticks"] = 8.0
        add(Note("max_cross_track_mm", d.values["max_cross_track_mm"],
                 "abort threshold, tightened from the 3.0 mm default now that "
                 "tracking is real (0.64 mm never tripped it)"))
        d.values["lead_max_mm"] = round(max(2.0 * res_mm, 0.5 * lookahead), 3)
        d.values["vel_filter_hz"] = round(max(1.0, control_hz / 4.0), 1)

    # ── 12. Open-loop pacing (for the legacy streamed path) ──
    # Deliberately NOT derived from the dead-time probe's cruise speed: that probe
    # raises SMS to ~1.5× its own step magnitude, so its cruise reads back the
    # PROBE speed, not the stage maximum. Dividing one by the other produces a
    # meaningless ratio (it read 1.98 in a bench simulation purely because the
    # probe ran at 3 mm/s against a 5.9 mm/s stage). Open-loop pacing needs the
    # dedicated top-speed sweep, so it is left alone here.
    d.values.pop("pace_correction", None)

    # ── headline summary ──
    d.summary = {
        "L_ms": L * 1000.0,
        "residual_L_ms": residual_L * 1000.0,
        "target_speed_mm_s": target,
        "achievable_cap_mm_s": achievable_cap,
        "top_speed_mm_s": top_mm_s,
        "ku": g["ku"],
        "tu_s": g["tu"],
        "kp_stability_limit": g["kp_stability_limit"],
        "coast_mm": coast_mm,
        # The analytically-correct gain, reported even when it is withheld
        # because the control law cannot yet bound it.
        "pid_kp_pending": round(g["kp"], 3),
        "gains_applied": gains_safe,
    }
    d.warnings.extend(validate(m, p, d.values, summary=d.summary))
    return d


def validate(m: MeasuredMachine, p: CalibrationPolicy, values: dict,
             summary: dict | None = None) -> list:
    """Cross-check a derived (or hand-edited) settings set against the physics.

    Catches the specific ways this system has actually gone wrong.
    """
    out = []
    if not m.is_complete():
        return [Warning_("error", "Not calibrated — still need: "
                                  + ", ".join(m.missing()))]
    L = m.effective_delay_s()
    ku, _tu = VC.plant_ultimate_gain(m.dead_time_s, m.control_loop_ms)
    kp = float(values.get("pid_kp", 0.0) or 0.0)
    la = float(values.get("lookahead_mm", 0.0) or 0.0)
    decel = float(values.get("decel_mm", 0.0) or 0.0)
    res_mm = max(1e-4, p.resolution_um / 1000.0)
    target = (summary or {}).get("target_speed_mm_s", p.target_speed_mm_s)

    if kp >= ku:
        out.append(Warning_("error",
                            f"kp {kp:.2f} is at/above the stability limit "
                            f"π/(2L) = {ku:.2f} — the cross-track loop will "
                            f"oscillate."))
    elif kp > 0.6 * ku:
        out.append(Warning_("warn",
                            f"kp {kp:.2f} is above 60 % of the {ku:.2f} "
                            f"stability limit — little margin left."))

    coast = target * L
    if decel > 0 and decel < coast:
        out.append(Warning_("warn",
                            f"decel window {decel:.2f} mm is shorter than the "
                            f"{coast:.2f} mm the stage coasts in one dead time — "
                            f"expect endpoint overshoot."))

    if la > 0 and la < 2.0 * res_mm:
        out.append(Warning_("warn",
                            f"lookahead {la * 1000:.0f} µm is below 2× the "
                            f"{p.resolution_um:.0f} µm resolution element — the "
                            f"carrot is inside the noise floor."))

    # The failure the operator hit: the corner limit sitting above the speed cap.
    csf = float(values.get("corner_speed_factor", 0.0) or 0.0)
    cap = (summary or {}).get("achievable_cap_mm_s", 0.0)
    if csf > 0 and cap > 0 and target * csf > cap:
        out.append(Warning_("warn",
                            f"corner tuning is INERT: the sharpest corner limit "
                            f"({target * csf:.2f} mm/s) is above the {cap:.2f} mm/s "
                            f"cap, so min(cap, corner) is always the cap."))

    if m.dead_time_spread_s > 0 and m.dead_time_s > 0:
        rel = m.dead_time_spread_s / m.dead_time_s
        if rel > 0.25:
            out.append(Warning_("warn",
                                f"dead-time spread is {rel:.0%} of its value — "
                                f"re-measure before trusting the gains."))
        else:
            out.append(Warning_("info",
                                f"dead time {m.dead_time_s * 1000:.0f} ms ± "
                                f"{m.dead_time_spread_s * 1000:.1f} ms "
                                f"({rel:.1%}) — a tight measurement."))

    if m.declared_max_speed_um_s > 0 and m.top_speed_um_s > 0:
        ratio = m.declared_max_speed_um_s / m.top_speed_um_s
        if ratio > 3.0:
            out.append(Warning_("warn",
                                f"the controller declares "
                                f"{m.declared_max_speed_um_s:.0f} µm/s but the "
                                f"stage measured {m.top_speed_um_s:.0f} ({ratio:.1f}×) "
                                f"— re-run the top-speed sweep over ≥20 mm so "
                                f"accel stops dominating the fit."))

    if m.cruise_um_s > 0 and m.top_speed_um_s > 0:
        r = m.cruise_um_s / m.top_speed_um_s
        if not (0.5 <= r <= 2.0):
            out.append(Warning_("warn",
                                f"the dead-time probe's cruise speed "
                                f"({m.cruise_um_s:.0f} µm/s) disagrees with the "
                                f"stored top speed ({m.top_speed_um_s:.0f}) by "
                                f"{r:.1f}× — one of them is wrong."))
    return out


# ── The one-click plan (what runs, in what order, and why) ────────────

@dataclass
class Step:
    key: str
    title: str
    detail: str
    needs: tuple = ()          # keys that must run first
    moves_stage: bool = True


#: The ordered calibration sequence. The ORDER IS THE POINT: every later step
#: consumes an earlier measurement, which is why running these piecemeal (as the
#: separate buttons required) so easily produced an inconsistent set.
CALIBRATION_STEPS = (
    Step("comms", "Comms rate",
         "Interleave a velocity command and a fresh position read WHILE moving → "
         "the real closed-loop period. Bounds every speed below.",
         moves_stage=True),
    Step("deadtime", "Dead time",
         "Step the velocity command and time the position response → the "
         "command→motion transport delay, plus the rise time and an independent "
         "cruise-speed cross-check. Divides the achievable print speed.",
         needs=("comms",)),
    Step("topspeed", "Top speed",
         "Sweep distances at full SMS and fit time-vs-distance → the speed the "
         "stage really delivers. Clamps the command so it stays achievable.",
         needs=()),
    Step("derive", "Derive settings",
         "Compute the lookahead, gains, decel, guards and corner schedule in "
         "closed form from the three measurements. No search, no hardware.",
         needs=("comms", "deadtime", "topspeed"), moves_stage=False),
    Step("verify", "Verify",
         "Drive Square + Star + Circle at the target speed and score them with "
         "the completion/dither-aware metric. This is the accept/reject gate.",
         needs=("derive",)),
    Step("refine", "Refine (only if verify fails)",
         "Bounded search over the one or two parameters that are NOT derivable "
         "(chiefly the corner speed factor), evaluated against the noise floor.",
         needs=("verify",)),
    Step("accept", "Apply to all prints",
         "Persist the settings so every print — Quick Print and Full Print — "
         "inherits them.",
         needs=("verify",), moves_stage=False),
)


def plan_summary(policy: CalibrationPolicy) -> list:
    """Human-readable plan, for the confirm dialog before anything moves."""
    return [f"{i + 1}. {s.title} — {s.detail}"
            for i, s in enumerate(CALIBRATION_STEPS)]


# ── Test area: always centre the stage before probing ─────────────────
#
# Every probe here moves the stage relative to wherever it starts. If it starts
# near a travel extent the move CLAMPS — and a clamped probe does not fail
# loudly, it silently measures the clamp: the dead-time probe sees "no motion",
# the loop-rate probe sees a stalled oscillation, the top-speed sweep fits a
# truncated distance. So the correct behaviour is to travel to the middle of the
# reachable envelope first, and to refuse (or shrink) any test whose excursion
# would not fit with margin.

#: Keep this much clear of the envelope wall, beyond the test's own excursion.
ENVELOPE_MARGIN_UM = 2000.0


def envelope_center_um(controller) -> tuple | None:
    """Absolute stage µm at the middle of the reachable XY envelope, or None.

    Reuses ``StageController.default_plate_center_um`` /
    ``SafetyLimits.xy_center`` — the bounds are absolute stage µm (v7.5.x), so the
    midpoint needs no ``zero_position`` offset.
    """
    try:
        c = controller.default_plate_center_um()
        if c and c[0] is not None and c[1] is not None:
            return (float(c[0]), float(c[1]))
    except Exception:
        pass
    try:
        c = controller.safety_limits.xy_center()
        return (float(c[0]), float(c[1]))
    except Exception:
        return None


def envelope_half_extent_um(controller) -> tuple | None:
    """``(half_x, half_y)`` of the reachable envelope, or None."""
    try:
        sl = controller.safety_limits
        return (0.5 * (float(sl.xy_max_x) - float(sl.xy_min_x)),
                0.5 * (float(sl.xy_max_y) - float(sl.xy_min_y)))
    except Exception:
        return None


def usable_test_radius_um(controller, *, margin_um=ENVELOPE_MARGIN_UM) -> float:
    """Largest excursion from the envelope centre that stays clear of the walls.

    0.0 when the envelope is unknown — callers should then refuse to probe rather
    than guess.
    """
    half = envelope_half_extent_um(controller)
    if not half:
        return 0.0
    return max(0.0, min(half[0], half[1]) - max(0.0, margin_um))


def check_fits(controller, half_extent_um, *, margin_um=ENVELOPE_MARGIN_UM) -> dict:
    """Would a test needing ``±half_extent_um`` about the centre fit?

    Returns ``{ok, usable_um, requested_um, center_um, reason}``.
    """
    center = envelope_center_um(controller)
    usable = usable_test_radius_um(controller, margin_um=margin_um)
    req = abs(float(half_extent_um or 0.0))
    if center is None or usable <= 0.0:
        return {"ok": False, "usable_um": usable, "requested_um": req,
                "center_um": center,
                "reason": "XY travel envelope unknown — record the Min/Max "
                          "limits on Hardware Setup → Device first."}
    if req > usable:
        return {"ok": False, "usable_um": usable, "requested_um": req,
                "center_um": center,
                "reason": (f"test needs ±{req:.0f} µm about the centre but only "
                           f"±{usable:.0f} µm is clear of the travel limits "
                           f"(keeping a {margin_um:.0f} µm margin)")}
    return {"ok": True, "usable_um": usable, "requested_um": req,
            "center_um": center, "reason": ""}


def fit_shape_size_mm(controller, requested_mm, *, margin_um=ENVELOPE_MARGIN_UM
                      ) -> float:
    """Shrink a challenge-shape size so its bounding box fits about the centre.

    A shape of size ``S`` centred on the origin spans ±S/2, so the usable size is
    twice the usable radius. Returns 0.0 if nothing fits.
    """
    usable = usable_test_radius_um(controller, margin_um=margin_um)
    if usable <= 0.0:
        return 0.0
    return max(0.0, min(float(requested_mm or 0.0), 2.0 * usable / 1000.0))


def center_stage(controller, *, safe_z_mm=None, tolerance_mm=0.05,
                 timeout_s=30.0) -> dict:
    """Travel to the middle of the reachable envelope before probing.

    SAFETY: uses ``safe_travel_to`` when a safe Z is known, so the needle is
    retracted BEFORE the cross-position XY move and never descends
    (``target_z_mm=None``) — the CLAUDE.md retract-before-travel invariant. Falls
    back to an XY-only absolute move when no safe Z is available (an XY-only rig).

    Returns ``{ok, center_um, reason}``.
    """
    center = envelope_center_um(controller)
    if center is None:
        return {"ok": False, "center_um": None,
                "reason": "XY travel envelope unknown — cannot find the centre."}
    cx, cy = center
    try:
        if safe_z_mm is not None and hasattr(controller, "safe_travel_to"):
            ok = controller.safe_travel_to(
                cx, cy, safe_z_mm=safe_z_mm, target_z_mm=None)
            if ok is False:
                return {"ok": False, "center_um": center,
                        "reason": "retract-before-travel was not confirmed — "
                                  "refusing to move XY."}
        else:
            controller.move_xy_absolute_um(cx, cy)
            waiter = getattr(controller, "wait_for_xy_arrival", None)
            if callable(waiter):
                waiter(cx / 1000.0, cy / 1000.0, tolerance_mm=tolerance_mm,
                       timeout_s=timeout_s)
    except Exception as e:
        return {"ok": False, "center_um": center, "reason": f"{e}"}

    # VERIFY, don't assume. Observed on ME3B V1: this reported ok=True while the
    # stage was still ~0.7 mm away, because the arrival wait's return value was
    # ignored and (separately) position reads can be corrupted by an undrained
    # controller ack. Every probe is then measured about the WRONG origin, and a
    # probe that silently starts off-centre is exactly what the centring exists to
    # prevent.
    reader = getattr(controller, "get_xy_position", None)
    if callable(reader):
        try:
            p = reader(cached=False)
            if not p or p[0] is None or p[1] is None:
                return {"ok": False, "center_um": center,
                        "reason": "could not read the position back to confirm "
                                  "centring (the controller returned no value)"}
            err = math.hypot(float(p[0]) - cx, float(p[1]) - cy)
            tol_um = max(50.0, tolerance_mm * 1000.0)
            if err > tol_um:
                return {"ok": False, "center_um": center,
                        "reason": (f"centring did not land: {err:.0f} µm from the "
                                   f"centre (tolerance {tol_um:.0f} µm)")}
        except Exception as e:
            return {"ok": False, "center_um": center,
                    "reason": f"could not confirm centring: {e}"}
    return {"ok": True, "center_um": center, "reason": ""}


# ── Reading / writing the store ───────────────────────────────────────

def measured_from_store(store, *, declared_max_speed_um_s: float = 0.0
                        ) -> MeasuredMachine:
    """Gather whatever has already been measured."""
    meta = {}
    try:
        meta = store.get_velocity_dead_time_meta() or {}
    except Exception:
        pass
    dead, _src = (0.0, "")
    try:
        dead, _src = store.effective_dead_time_s()
    except Exception:
        pass
    v = {}
    try:
        v = store.get_mode_params("velocity")
    except Exception:
        pass
    return MeasuredMachine(
        control_loop_ms=float(store.get_control_loop_ms() or 0.0),
        dead_time_s=float(dead or 0.0),
        dead_time_spread_s=float(meta.get("spread_s", 0.0) or 0.0),
        tau_s=float(meta.get("tau_s", 0.0) or 0.0),
        top_speed_um_s=float(store.get_xy_max_speed_um_s() or 0.0),
        cruise_um_s=float(meta.get("cruise_um_s", 0.0) or 0.0),
        lateral_accel_mm_s2=float(v.get("lateral_accel_mm_s2", 0.0) or 0.0),
        declared_max_speed_um_s=float(declared_max_speed_um_s or 0.0),
    )


def apply_to_store(store, derived: Derived) -> None:
    """Persist the derived settings into the shared calibration store."""
    if not derived.values:
        return
    vel = {k: v for k, v in derived.values.items()
           if k not in ("pace_correction", "settle_tol_um")}
    store.set_mode_params("velocity", vel)
    if "pace_correction" in derived.values:
        store.set_mode_params("open_loop",
                              {"pace_correction": derived.values["pace_correction"]})
    if "settle_tol_um" in derived.values:
        store.set_mode_params("confirm", {
            "settle_tol_um": derived.values["settle_tol_um"],
            "corner_angle_deg": derived.values.get("corner_angle_deg", 30.0)})


def stamp_print_settings(settings, store, *, prefer_measured_dead_time=True):
    """Stamp the calibrated tuning onto a ``PrintSettings`` — the ONE place that
    does it, so every print path inherits the same numbers.

    Previously only Quick Print stamped any of this, which meant a bench tuning
    session had **no effect at all** on a Full Print. Both paths now call here.

    Best-effort: any failure leaves ``settings`` at its legacy defaults.
    """
    try:
        ol = store.get_mode_params("open_loop")
        cf = store.get_mode_params("confirm")
        v = store.get_mode_params("velocity")
    except Exception:
        return settings
    try:
        settings.pace_correction = ol.get("pace_correction", 1.0)
        settings.segment_settle_tol_um = cf.get("settle_tol_um", 0.0)
        settings.confirm_corner_angle_deg = cf.get("corner_angle_deg", 0.0)
        settings.vel_lookahead_mm = v.get("lookahead_mm", 0.0)
        settings.vel_control_hz = v.get("control_hz", 0.0)
        settings.vel_decel_mm = v.get("decel_mm", 0.0)
        settings.vel_corner_angle_deg = v.get("corner_angle_deg", 0.0)
        settings.vel_corner_speed_factor = v.get("corner_speed_factor", 0.0)
        settings.vel_pid_kp = v.get("pid_kp", 0.0)
        settings.vel_pid_kd = v.get("pid_kd", 0.0)
        settings.xy_max_speed_um_s = store.get_xy_max_speed_um_s() or 0.0
        settings.control_loop_ms = store.get_control_loop_ms() or 0.0
        settings.phase_lag_s = store.get_phase_lag_s() or 0.0
        settings.vel_tuning = dict(v)
        settings.xy_jerk_pct = v.get("jerk_pct", 0.0)
        if prefer_measured_dead_time:
            dt = store.get_velocity_dead_time_s()
            if dt:
                settings.vel_tuning["dead_time_s"] = float(dt)
        # v7.6: stamp the first-order rise time too, so the JOB carries
        # everything a StageCharacteristics needs (dead time + τ + top speed +
        # loop period). The feed-plan executor then builds its plan from the
        # stamped settings alone — reproducible from the job, no execute-time
        # global-store read, and testable with a plain PrintSettings.
        try:
            meta = store.get_velocity_dead_time_meta() or {}
            tau = float(meta.get("tau_s", 0.0) or 0.0)
            if tau > 0:
                settings.vel_tuning["tau_s"] = tau
        except Exception:
            pass
    except Exception:
        pass
    return settings
