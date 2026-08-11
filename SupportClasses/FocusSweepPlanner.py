"""
Focus sweep planning — where to sample, and how far it is safe to go.

A sweep is a coarse-to-fine bracket refinement: each rung samples a range, the
measured peak re-centres the next rung, and the range shrinks because the
previous rung already localised the peak. That is the robust form of "increments
that gradient-descend towards focus".

THE COLLISION GUARD LIVES HERE, IN PURE CODE
--------------------------------------------
The microscope focus drive moves the objective **toward the specimen**. The bound
on how far it may travel is the objective's working distance — a 20x with ~1 mm
WD physically cannot sweep ±1 mm. Putting that bound in the planner rather than
in the worker means it is unit-testable with no hardware, and that every rung,
including ones generated later by :func:`next_rung`, inherits it.

Every value in a returned plan satisfies ALL of:

    |v - centre|            <= min(requested_half_range, wd_budget)
    soft_lo + lead_in       <= v <= soft_hi
    v - lead_in             >= soft_lo          (the backlash approach fits too)

A plan that would be EMPTY raises rather than returning ``()``. A zero-sample
sweep would otherwise be reported downstream as "no peak found", which reads as
an optics problem when it is really "your limits forbid this sweep".

BACKLASH IS NOT OPTIONAL
------------------------
Every rung boundary reverses direction, so each focus position must be approached
from the SAME side with a lead-in overshoot. Without it the rungs carry backlash
with opposite signs, which shows up as a scale error — and it is the same
property that makes the focus↔needle-Z fit meaningful at all.

Pure: no Qt, no hardware, no I/O, no numpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ── tunables ──────────────────────────────────────────────────────────────

#: Next rung's half-range as a multiple of this rung's step. After a rung the
#: discrete argmax is within half a step of the true peak for a well-sampled
#: unimodal curve; noise and asymmetry can add another half. 2x is the safety
#: factor on that bound. Larger merely re-covers ground already searched.
BRACKET_SHRINK = 2.0

#: Coarse step ceiling as a multiple of depth of field. Above roughly 8 DOF the
#: metric's decay between samples is shallow enough that a real peak can hide
#: between them. Self-validating: FocusCurve measures the FWHM, and a rung-0 step
#: that exceeds the measured FWHM was under-sampled and got lucky.
COARSE_STEP_DOF_MULT = 8.0

#: Step floor as a multiple of the drive's own resolution. Below this you are
#: sweeping the encoder, not the optics.
MIN_STEP_DRIVE_MULT = 4.0

#: Overshoot used to approach every focus position from the same direction.
BACKLASH_LEADIN_UM = 10.0

#: How much parfocality error to allow for when handing off to another
#: objective, before its offset has been measured on this rig. Once measured, the
#: caller passes the real number and this budget stops mattering.
PARFOCAL_BUDGET_UM = 50.0

#: Per-sample wall time: settle + frame grab. 0.30 s is realistic at full
#: resolution on a 5 MP sCMOS including discarded frames; binned sweeps are much
#: faster.
DEFAULT_SETTLE_S = 0.30

#: Turret rotation + settle, for the wall-time estimate.
DEFAULT_TURRET_S = 2.0

#: Search bounds for the samples-per-side that minimises total frames.
K_MIN, K_MAX = 4, 16


@dataclass(frozen=True, kw_only=True)
class SweepRung:
    index: int
    center_um: float
    half_range_um: float
    step_um: float
    z_targets_um: tuple[float, ...]
    lead_in_um: float
    #: Step as a multiple of depth of field — the under-sampling diagnostic.
    step_over_dof: float | None = None

    @property
    def num_frames(self) -> int:
        return len(self.z_targets_um)

    def approach_um(self, z_um: float) -> float:
        """Where to go BEFORE ``z_um`` so it is always approached upward."""
        return float(z_um) - self.lead_in_um


@dataclass(frozen=True, kw_only=True)
class SweepPlan:
    rungs: tuple[SweepRung, ...]
    objective_label: str = ""
    dof_um: float | None = None
    target_sigma_um: float | None = None
    clamped: bool = False
    clamp_reasons: tuple[str, ...] = ()
    estimated_seconds: float = 0.0

    @property
    def total_frames(self) -> int:
        return sum(r.num_frames for r in self.rungs)

    def describe(self) -> str:
        bits = [f"{len(self.rungs)} rungs", f"{self.total_frames} frames",
                f"~{self.estimated_seconds:.0f} s"]
        if self.clamped:
            bits.append("CLAMPED: " + "; ".join(self.clamp_reasons))
        return " · ".join(bits)


def choose_ladder(range_ratio: float, k_lo: int = K_MIN,
                  k_hi: int = K_MAX) -> tuple[int, int]:
    """``(K, m)`` — samples per side and rung count — minimising total frames.

    With ``K`` samples per side the step is ``R/K`` and the next half-range is
    ``BRACKET_SHRINK * R/K``, so each rung divides the range by ``K/2``. After
    ``m`` rungs the final step is ``R0/K * (2/K)^(m-1)``; requiring that to reach
    the target step gives ``range_ratio <= K * (K/2)^(m-1)``.
    """
    ratio = max(1.0, float(range_ratio))
    best = None
    for K in range(max(2, int(k_lo)), int(k_hi) + 1):
        shrink = K / BRACKET_SHRINK
        if shrink <= 1.0:
            continue
        m = 1
        reach = float(K)
        while reach < ratio and m < 12:
            reach *= shrink
            m += 1
        if reach < ratio:
            continue
        frames = m * (2 * K + 1)
        if best is None or frames < best[0]:
            best = (frames, K, m)
    if best is None:
        return (K_MAX, 6)
    return (best[1], best[2])


def sweep_targets_um(*, center_um: float, half_range_um: float, step_um: float,
                     soft_lo: float | None = None,
                     soft_hi: float | None = None,
                     lead_in_um: float = BACKLASH_LEADIN_UM,
                     ) -> tuple[float, ...]:
    """The sample positions for one rung, ascending, all inside every bound.

    Raises ``ValueError`` when the bounds leave nothing to sample. Returning an
    empty tuple instead would surface downstream as "no peak found" — an optics
    diagnosis for what is really a limits problem.
    """
    if step_um <= 0:
        raise ValueError("focus sweep step must be positive")
    if half_range_um < 0:
        raise ValueError("focus sweep half-range cannot be negative")
    lo = center_um - half_range_um
    hi = center_um + half_range_um
    # The backlash approach position must also be legal, so the usable floor is
    # the soft limit PLUS the lead-in.
    if soft_lo is not None:
        lo = max(lo, float(soft_lo) + max(0.0, lead_in_um))
    if soft_hi is not None:
        hi = min(hi, float(soft_hi))
    if hi < lo:
        raise ValueError(
            f"the focus limits leave no room to sweep around {center_um:.1f} µm "
            f"(usable {lo:.1f}..{hi:.1f} µm). Widen the focus soft limits, or "
            f"move the focus closer to the middle of its travel.")
    n = int(math.floor((hi - lo) / step_um + 1e-9)) + 1
    if n < 1:
        raise ValueError(
            f"a {step_um:.2f} µm step does not fit in the usable range "
            f"{lo:.1f}..{hi:.1f} µm around {center_um:.1f} µm")
    return tuple(round(lo + k * step_um, 6) for k in range(n))


def plan_sweep(*, optics, center_um: float,
               requested_half_range_um: float = 1000.0,
               focus_limits_um: tuple | None = None,
               soft_limits_um: tuple | None = None,
               prior_sigma_um: float | None = None,
               drive_resolution_um: float = 0.025,
               settle_s: float = DEFAULT_SETTLE_S,
               ) -> tuple[SweepPlan | None, str]:
    """Build a coarse-to-fine plan for one objective. ``(plan, reason)``.

    ``prior_sigma_um`` is how well the centre is already known — from a stored
    plate-to-plate prior, or from the live fit over sites already measured. It
    only ever NARROWS the search; it never contributes to the result.
    """
    from SupportClasses.ObjectiveOptics import (
        depth_of_field_um, wd_bounded_half_range_um, achievable_focus_sigma_um,
        refuse_if_incomplete)

    why = refuse_if_incomplete(optics)
    if why:
        return (None, why)
    dof = depth_of_field_um(optics)
    target_sigma = achievable_focus_sigma_um(optics)
    if not dof or dof <= 0:
        return (None, "depth of field is unknown for this objective")

    clamp_reasons: list[str] = []

    # -- the collision guard, first and unconditionally.
    wd_half, wd_known = wd_bounded_half_range_um(optics)
    half = float(requested_half_range_um)
    if half > wd_half:
        if wd_known:
            clamp_reasons.append(
                f"range limited to ±{wd_half:.0f} µm by the objective's "
                f"{optics.working_distance_mm:g} mm working distance "
                f"(asked ±{half:.0f} µm)")
        else:
            clamp_reasons.append(
                f"this objective reports no working distance, so the sweep is "
                f"limited to ±{wd_half:.0f} µm. Widen it only if you know the "
                f"objective's working distance.")
        half = wd_half

    # -- a known prior narrows the search. 3 sigma plus one DOF of margin.
    if prior_sigma_um is not None and prior_sigma_um >= 0:
        narrowed = 3.0 * float(prior_sigma_um) + dof
        if narrowed < half:
            half = narrowed

    lo_hi = _merge_limits(focus_limits_um, soft_limits_um)

    # -- rung geometry.
    step_floor = MIN_STEP_DRIVE_MULT * max(1e-9, float(drive_resolution_um))
    step_target = max(dof / 3.0, step_floor)
    coarse_cap = COARSE_STEP_DOF_MULT * dof
    K, m = choose_ladder(half / step_target if step_target > 0 else 1.0)

    rungs: list[SweepRung] = []
    r_half = half
    for idx in range(m):
        step = r_half / K
        if idx == 0 and step > coarse_cap:
            # Refuse to under-sample rung 0: add samples rather than widen steps.
            step = coarse_cap
            clamp_reasons.append(
                f"coarse step capped at {coarse_cap:.1f} µm "
                f"({COARSE_STEP_DOF_MULT:g}× depth of field) so a real peak "
                f"cannot hide between samples")
        step = max(step, step_floor)
        try:
            targets = sweep_targets_um(
                center_um=center_um, half_range_um=r_half, step_um=step,
                soft_lo=lo_hi[0], soft_hi=lo_hi[1])
        except ValueError as e:
            return (None, str(e))
        rungs.append(SweepRung(
            index=idx, center_um=float(center_um), half_range_um=float(r_half),
            step_um=float(step), z_targets_um=targets,
            lead_in_um=BACKLASH_LEADIN_UM,
            step_over_dof=(step / dof) if dof else None))
        r_half = BRACKET_SHRINK * step
        if r_half <= step_floor:
            break

    plan = SweepPlan(
        rungs=tuple(rungs), objective_label=optics.label, dof_um=dof,
        target_sigma_um=target_sigma,
        clamped=bool(clamp_reasons), clamp_reasons=tuple(clamp_reasons),
        estimated_seconds=estimate_wall_time_s(tuple(rungs), settle_s=settle_s))
    return (plan, "")


def plan_handoff(*, to_optics, prior_peak_um: float,
                 prior_sigma_um: float | None = None,
                 parfocal_delta_um: float = 0.0,
                 parfocal_budget_um: float = PARFOCAL_BUDGET_UM,
                 **kw) -> tuple[SweepPlan | None, str]:
    """Plan the next rung of an objective ladder, centred by parfocality.

    ``parfocal_delta_um`` is the MEASURED focus offset from the previous
    objective to this one. Passing it is what turns the escalation from a search
    into a direct move; ``parfocal_budget_um`` is the fallback uncertainty used
    while that offset is still unknown.
    """
    centre = float(prior_peak_um) + float(parfocal_delta_um or 0.0)
    unknown = parfocal_budget_um if not parfocal_delta_um else 0.0
    sigma = math.hypot(float(prior_sigma_um or 0.0), float(unknown))
    kw.setdefault("requested_half_range_um", max(4.0 * sigma, 20.0))
    return plan_sweep(optics=to_optics, center_um=centre,
                      prior_sigma_um=sigma, **kw)


def next_rung(*, plan: SweepPlan, rung_index: int,
              measured_peak_um: float,
              focus_limits_um: tuple | None = None,
              soft_limits_um: tuple | None = None) -> SweepRung | None:
    """The following rung, RE-CENTRED on the measured peak.

    Re-centring on the measurement rather than on the nominal centre is the whole
    point of a coarse-to-fine ladder: it is what lets the range shrink safely.
    """
    if rung_index + 1 >= len(plan.rungs):
        return None
    nxt = plan.rungs[rung_index + 1]
    lo_hi = _merge_limits(focus_limits_um, soft_limits_um)
    try:
        targets = sweep_targets_um(
            center_um=float(measured_peak_um),
            half_range_um=nxt.half_range_um, step_um=nxt.step_um,
            soft_lo=lo_hi[0], soft_hi=lo_hi[1], lead_in_um=nxt.lead_in_um)
    except ValueError:
        return None
    from dataclasses import replace
    return replace(nxt, center_um=float(measured_peak_um),
                   z_targets_um=targets)


def estimate_wall_time_s(rungs, *, settle_s: float = DEFAULT_SETTLE_S,
                         turret_s: float = 0.0) -> float:
    frames = sum(len(r.z_targets_um) for r in rungs)
    return float(frames) * float(settle_s) + float(turret_s)


def _merge_limits(hardware, soft) -> tuple[float | None, float | None]:
    """Intersect the SDK's declared travel with the operator's soft limits."""
    lo = hi = None
    for pair in (hardware, soft):
        if not pair:
            continue
        try:
            a, b = pair[0], pair[1]
        except (TypeError, IndexError):
            continue
        if a is not None:
            lo = float(a) if lo is None else max(lo, float(a))
        if b is not None:
            hi = float(b) if hi is None else min(hi, float(b))
    return (lo, hi)
