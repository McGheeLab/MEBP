"""XYFeedPlan.py — a DETERMINISTIC, feature-aware motion plan for a known path.

The print path is known a priori, so instead of asking one fixed tuning to
survive every geometry (it provably cannot — the 2026-07-28 geometry panel
measured ~220 µm corner cuts at any completing tuning, plus hard stalls at
180° reversals and sub-lookahead features), the plan is built FROM the path's
own features, section by section:

  • **Sharp corners and reversals are section splits.** Pure pursuit cannot
    turn a corner without cutting it by ~0.4·lookahead, and cannot turn 180°
    at all — so the plan decelerates and STOPS exactly on the vertex (tight
    arrive tolerance), then continues as a fresh section in the new direction.
    Zero corner cut, zero reversal stall, by construction.
  • **Curved sections get a curvature-sized lookahead.** The pursuit standoff
    on an arc of radius R is ≈ lookahead²/(2R); inverting for a deviation
    budget δ gives  lookahead = √(2·R·δ)  (with margin), and the dead-time
    stability rule then sets that section's speed:  v = lookahead/(L·safety).
  • **Straights run the full lookahead at the full target speed.**

The deviation budget δ is the resolution element divided by a validated
prediction headroom (the deterministic stage model measured ~1.56× optimistic
against hardware), so a plan that PREDICTS δ tracks ≤ the element when run.

Everything here is pure Python and rides the existing shared pieces:
``XYStageModel`` (the machine's saved dynamics), ``XYPathSimulator.follow_path``
(the ONE follower loop, simulated or real — sections are just consecutive
follow_path runs), and ``XYChallenge.path_report`` (scoring vs the FULL
original path, so section splits cannot hide anything).

``plan_rows`` exports the plan as a time-parameterised X/Y/pump trajectory
(the deterministic all-axis artefact): pump volume is arc-length-proportional,
so the µL column is exact whatever the stage actually does — the runtime
velocity follower also deposits ∝ measured Δs, which keeps the bead correct
even when the stage runs behind the plan's timing.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

from SupportClasses import VelocityControl as VC
from SupportClasses import XYChallenge as XC
from SupportClasses import XYPathSimulator as PS
from SupportClasses.XYStageModel import StageCharacteristics, XYStageModel


#: Deviation budget = element / headroom. 1.6 ≈ the measured median
#: actual/predicted p95 ratio on ME3B V1 (geometry panel, 2026-07-28).
PREDICTION_HEADROOM = 1.6

#: A vertex turning more than this is a SPLIT (stop on the vertex). Must stay
#: above the per-vertex turn angle of a coarsely sampled circle (a 0.5 mm step
#: on a 1 mm radius turns ~29°/vertex) or arcs would shatter into stops.
CORNER_SPLIT_DEG = 55.0

#: Lookahead floor/ceiling (mm). The floor keeps the carrot meaningfully ahead
#: of the ~1 µm encoder quantum + per-tick travel; the ceiling is overridden by
#: the machine-derived lookahead when smaller.
LA_MIN_MM = 0.08
LA_MAX_MM = 1.5

#: Section-end arrival tolerance (mm) — a stop must land ON the vertex to keep
#: the corner inside the deviation budget (the follower's default 0.05 mm
#: arrive would already eat the whole 30 µm element). The post-arrival coast
#: adds ~floor-speed·lag on top, so the two together must stay ≤ the budget.
ARRIVE_MM = 0.01

#: Absolute end-taper floor speed (mm/s): the crossing/coast speed at a split.
#: Coast ≈ floor · (dead time + τ) ≈ 8 µm on ME3B V1.
END_FLOOR_MM_S = 0.08

#: Margin on the lag used by the end taper (the model's τ decay makes the
#: actual velocity trail the commanded taper).
END_LAG_MARGIN = 1.35

#: Wall-clock cost of ONE corner stop (s) — MEASURED on hardware, not assumed.
#:
#: The 2026-07-29 instrumented star run (9 stops, 28.64 mm at 3 mm/s) took
#: 21.4 s against 9.5 s of pure motion → **1.31 s per stop**, of which only
#: ~0.45 s is the XY standing still; the rest is the decel into the 10 µm
#: arrive tolerance, the inter-section settle dwell, and re-acquiring speed on
#: the next section. The original 0.35 s guess understated a 9-stop path's time
#: by 1.7×, which matters because this estimate is exactly how the operator
#: prices "finer resolution costs time" before committing to a print.
STOP_COST_S = 1.3

#: Lookahead may use at most this fraction of its section's length (a carrot
#: past the section end just pursues the endpoint, wasting the taper room).
LA_SECTION_FRAC = 0.4

#: Extra margin on the curvature-sized lookahead (standoff model is ideal).
LA_CURVE_MARGIN = 0.8


# ── Geometry helpers ──────────────────────────────────────────────────

def local_radius_mm(pts, i) -> float:
    """Circumradius through vertices (i-1, i, i+1); ``inf`` when collinear or
    at the ends."""
    n = len(pts)
    if i <= 0 or i >= n - 1:
        return float("inf")
    ax, ay = pts[i - 1]
    bx, by = pts[i]
    cx, cy = pts[i + 1]
    a = math.hypot(bx - ax, by - ay)
    b = math.hypot(cx - bx, cy - by)
    c = math.hypot(cx - ax, cy - ay)
    area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    if area2 < 1e-12:
        return float("inf")
    return (a * b * c) / (2.0 * area2)


def min_attainable_resolution_um(char: StageCharacteristics, *,
                                 margin: float = 1.0,
                                 encoder_quantum_um: float = 1.0) -> float:
    """The finest resolution element (µm) this machine can actually hold.

    A corner STOP is where the plan's accuracy is decided, and three PHYSICAL
    terms set its floor:

      • the arrival tolerance the stop lands within (``ARRIVE_MM`` → 10 µm),
      • the post-arrival coast — even the taper's floor speed keeps moving for
        one lag: ``END_FLOOR_MM_S · (dead + τ + loop/2) · END_LAG_MARGIN``
        (≈ 12 µm on ME3B V1),
      • the encoder quantum (you cannot resolve below one count).

    On ME3B V1 that sums to ≈ 23 µm — and the hardware panel measured a
    worst-cell p95 of 27 µm at a 30 µm element with everything else at 1–2 µm,
    so the sum is the right order and correctly stays BELOW the 30 µm element
    the machine was proven to hold.

    ⚠ Deliberately NOT scaled by ``PREDICTION_HEADROOM``: that factor corrects
    the *simulator's* optimism about predicted deviations, whereas every term
    here is computed from measured dynamics directly. Multiplying by it double-
    counts and produced a 36.8 µm floor — which would have warned that 30 µm is
    unattainable on the very machine that demonstrably held it. ``margin`` is
    left as a caller knob for a deliberately conservative floor.

    Returns 0.0 when the machine has not been characterised (nothing to say).
    """
    if not char.is_complete():
        return 0.0
    coast_mm = (END_FLOOR_MM_S
                * (char.dead_time_s + char.tau_s
                   + 0.5 * char.control_loop_ms / 1000.0)
                * END_LAG_MARGIN)
    return max(0.0, float(margin)) * (ARRIVE_MM * 1000.0
                                      + coast_mm * 1000.0
                                      + max(0.0, float(encoder_quantum_um)))


def split_indices(pts, corner_split_deg=CORNER_SPLIT_DEG) -> list:
    """Vertex indices where the plan stops: every turn sharper than the split
    angle (covers both corners and 180° reversals)."""
    out = []
    for i in range(1, len(pts) - 1):
        if VC.corner_turn_angle_deg(pts, i) > corner_split_deg:
            out.append(i)
    return out


# ── The plan ──────────────────────────────────────────────────────────

@dataclass
class PlanSection:
    pts: list                    # section polyline (mm), shares split vertices
    lookahead_mm: float
    speed_mm_s: float
    arrive_mm: float = ARRIVE_MM
    min_radius_mm: float = float("inf")
    length_mm: float = 0.0
    est_time_s: float = 0.0
    reason: str = ""             # why this lookahead/speed (shows its work)
    #: v7.7 (continuous plans only): arc-length profiles replacing the scalar
    #: lookahead / speed, so a corner is slowed through instead of stopped on.
    lookahead_at: object = None
    speed_at: object = None


@dataclass
class FeedPlan:
    ideal: list                  # the FULL original path (scoring reference)
    sections: list = field(default_factory=list)
    target_speed_mm_s: float = 0.0
    element_um: float = 30.0
    deviation_budget_um: float = 0.0
    est_time_s: float = 0.0
    n_stops: int = 0
    notes: list = field(default_factory=list)
    #: v7.7: True for a never-stopping plan (one section + arc-length profiles).
    continuous: bool = False

    def summary(self) -> str:
        if self.continuous:
            return (f"continuous (no stops), est {self.est_time_s:.1f} s; "
                    f"budget {self.deviation_budget_um:.0f} µm of the "
                    f"{self.element_um:.0f} µm element")
        return (f"{len(self.sections)} section(s), {self.n_stops} stop(s), "
                f"est {self.est_time_s:.1f} s; budget "
                f"{self.deviation_budget_um:.0f} µm of the "
                f"{self.element_um:.0f} µm element")


def build_plan(pts, char: StageCharacteristics, *, target_speed_mm_s,
               element_um=30.0, headroom=PREDICTION_HEADROOM,
               corner_split_deg=CORNER_SPLIT_DEG, safety=2.0,
               la_max_mm=None) -> FeedPlan:
    """Build the deterministic feature-aware plan for ``pts``.

    ``la_max_mm`` defaults to the machine-derived straight-line lookahead
    (target·L·safety·1.5 — the same sizing rule the one-click calibration
    uses), so straights run the target speed with margin.
    """
    ps = max(0.05, float(target_speed_mm_s))
    delta_mm = (float(element_um) / 1000.0) / max(1.0, float(headroom))
    L = char.dead_time_s + char.control_loop_ms / 1000.0
    if la_max_mm is None:
        la_max_mm = min(LA_MAX_MM, max(LA_MIN_MM, ps * L * safety * 1.5))

    plan = FeedPlan(ideal=list(pts), target_speed_mm_s=ps,
                    element_um=float(element_um),
                    deviation_budget_um=delta_mm * 1000.0)
    if len(pts) < 2:
        return plan

    splits = split_indices(pts, corner_split_deg)
    bounds = [0] + splits + [len(pts) - 1]
    for k in range(len(bounds) - 1):
        seg = pts[bounds[k]:bounds[k + 1] + 1]
        if len(seg) < 2:
            continue
        cum = VC.polyline_arclength(seg)
        length = cum[-1]
        if length < 1e-6:
            continue
        # curvature-sized lookahead: standoff la²/(2R) ≤ δ
        r_min = min((local_radius_mm(seg, i) for i in range(1, len(seg) - 1)),
                    default=float("inf"))
        if math.isfinite(r_min):
            la_curve = LA_CURVE_MARGIN * math.sqrt(2.0 * r_min * delta_mm)
            reason = (f"min radius {r_min:.2f} mm → lookahead "
                      f"√(2·R·δ)·{LA_CURVE_MARGIN:g}")
        else:
            la_curve = la_max_mm
            reason = "straight — full lookahead"
        la = max(LA_MIN_MM, min(la_max_mm, la_curve,
                                length * LA_SECTION_FRAC))
        if length * LA_SECTION_FRAC < la_curve:
            reason += f"; capped to {LA_SECTION_FRAC:g}·section"
        # dead-time stability sets the speed the lookahead can sustain
        v = min(ps, la / (L * safety)) if L > 1e-6 else ps
        est = length / max(0.05, v) + (STOP_COST_S if k < len(bounds) - 2
                                       else 0.0)
        plan.sections.append(PlanSection(
            pts=list(seg), lookahead_mm=la, speed_mm_s=v,
            min_radius_mm=r_min, length_mm=length, est_time_s=est,
            reason=reason))
    plan.n_stops = max(0, len(plan.sections) - 1)
    plan.est_time_s = sum(s.est_time_s for s in plan.sections)
    if plan.n_stops:
        plan.notes.append(
            f"{plan.n_stops} corner/reversal stop(s) — pure pursuit cannot "
            f"turn > {corner_split_deg:g}° inside the budget, so the plan "
            f"stops ON those vertices")
    return plan


# ── Continuous plan: slow through corners, never stop ─────────────────
#
# The sectioned planner above stops ON every sharp vertex, which is what got
# 18/18 hardware shapes under 30 µm. The operator's requirement is the other end
# of that trade: *"We should never pause on corners, just slow down. We always
# want the pump moving unless there is a break in the section that is planned."*
#
# Slowing alone would NOT help: the pure-pursuit standoff is la²/(2R), so it is
# the LOOKAHEAD that sets how far inside a corner the stage cuts. What makes a
# continuous corner accurate is shrinking the carrot there — and dead-time
# stability (v ≤ la/(L·safety)) then *forces* the speed down with it. So this
# builds two arc-length profiles from the same curvature physics the sectioned
# planner uses per section, and hands them to one uninterrupted ``follow_path``.
#
# ⚠ A 180° cusp has R → 0, so its budget lookahead and therefore its speed go to
# the floor: the stage crawls through a reversal rather than stopping. That is a
# slowdown, not a dwell — the pump keeps advancing because deposition tracks
# arc length — but it is the geometry where "never pause" is closest to
# physically impossible, and where a stall is most likely.

#: Retained for callers/tests; the continuous planner no longer gates knots on
#: turn angle (that ignored smooth curvature — a 5 mm circle sampled at 0.5 mm
#: turns only ~11°/vertex, under any sane threshold, yet its radius absolutely
#: constrains the lookahead). A knot is now created wherever EITHER the arc or
#: the corner model asks for less than the straight-line lookahead.
CORNER_SLOW_DEG = 12.0

#: A turn sharper than this is a REVERSAL and still gets a stop, because a cusp
#: cannot be traversed by pure pursuit at any speed: a carrot placed forward in
#: arc length sits *behind* the stage in space, so the follower has nothing
#: coherent to chase. This is geometry, not policy — simulation confirms a
#: never-stopping reversal runs to the wall cap without progressing.
REVERSAL_STOP_DEG = 150.0

#: Ramp margin: how many lag-lengths of travel to allow for reaching a knot's
#: reduced speed. The profile must be *reachable* or the stage arrives at the
#: corner still fast, which is exactly the overshoot the end taper fixed.
RAMP_LAG_MARGIN = 2.5

#: Arc-length resolution of the precomputed profile, and its hard sample cap.
PROFILE_STEP_MM = 0.05
PROFILE_MAX_SAMPLES = 40000

#: Lookahead floor for a CONTINUOUS plan (mm). Lower than ``LA_MIN_MM`` because
#: a corner taken without stopping needs a much shorter carrot, and the speed
#: there is correspondingly low — at ~0.3 mm/s and 31 Hz the per-tick travel is
#: ~10 µm, so a 30 µm carrot is still 3 ticks ahead and well clear of the ~1 µm
#: encoder quantum.
LA_MIN_CONTINUOUS_MM = 0.03

#: Lead margin on the lag distance: the slowdown must be COMMANDED this far
#: before the corner, because the stage's response trails by dead time + τ. Only
#: applied on the approach side — accelerating away early would push the stage
#: wide on the way out.
LEAD_LAG_MARGIN = 1.25


def corner_budget_lookahead_mm(turn_deg: float, delta_mm: float) -> float:
    """Lookahead that keeps a SHARP vertex's cut inside ``delta_mm``.

    ⚠ The arc formula ``la = √(2·R·δ)`` describes a *smoothly curved* path and
    badly under-constrains a genuine corner: at a polyline vertex the two legs
    meet at an angle and pure pursuit chords across, cutting

        cut ≈ (la / 2) · sin(θ / 2)

    which is LINEAR in the lookahead, not quadratic. Using the arc formula at a
    90° vertex asked for 0.092 mm and simulated 40 µm against an 18.75 µm budget;
    inverting the corner formula asks for 0.053 mm instead. Returns ``inf`` for a
    turn too gentle to constrain anything.
    """
    th = max(0.0, min(180.0, float(turn_deg)))
    half = math.sin(math.radians(th) / 2.0)
    if half < 1e-6:
        return float("inf")
    return 2.0 * float(delta_mm) / half


class ArcProfile:
    """A precomputed, monotone-grid ``f(s)`` over arc length (mm).

    Evaluated once per control tick, so it must be O(log n) rather than a scan
    over every corner: a 20 k-point path at 31 Hz would otherwise burn ~600 k
    comparisons a second inside the control loop.
    """

    def __init__(self, grid: list, values: list, fallback: float):
        self._g = grid
        self._v = values
        self._fallback = float(fallback)

    def __call__(self, s: float) -> float:
        g = self._g
        if not g:
            return self._fallback
        i = bisect.bisect_right(g, float(s)) - 1
        if i < 0:
            i = 0
        elif i >= len(self._v):
            i = len(self._v) - 1
        return self._v[i]

    @property
    def values(self) -> list:
        return list(self._v)

    def min_value(self) -> float:
        return min(self._v) if self._v else self._fallback


def build_continuous_plan(pts, char: StageCharacteristics, *,
                          target_speed_mm_s, element_um=30.0,
                          headroom=PREDICTION_HEADROOM, safety=2.0,
                          la_max_mm=None, corner_slow_deg=CORNER_SLOW_DEG
                          ) -> FeedPlan:
    """Build a plan that SLOWS through corners instead of stopping on them.

    Lookahead and speed vary along the path: at every vertex the carrot is sized
    by whichever of the two deviation models binds (arc curvature, or the sharp
    corner formula), the speed follows from dead-time stability, and the profile
    is ramped — and lead-compensated — so it is actually reachable.

    Yields ONE section for any path without a cusp. A near-180° reversal is the
    one exception and still splits (see :data:`REVERSAL_STOP_DEG`), because pure
    pursuit cannot traverse a cusp at any speed.
    """
    ps = max(0.05, float(target_speed_mm_s))
    delta_mm = (float(element_um) / 1000.0) / max(1.0, float(headroom))
    L = char.dead_time_s + char.control_loop_ms / 1000.0
    if la_max_mm is None:
        la_max_mm = min(LA_MAX_MM, max(LA_MIN_MM, ps * L * safety * 1.5))

    plan = FeedPlan(ideal=list(pts), target_speed_mm_s=ps,
                    element_um=float(element_um),
                    deviation_budget_um=delta_mm * 1000.0)
    plan.continuous = True
    if len(pts) < 2:
        return plan

    v_straight = min(ps, la_max_mm / (L * safety)) if L > 1e-6 else ps

    # Reversals still split — see REVERSAL_STOP_DEG. Everything else is slowed
    # through, so a path with no cusp yields exactly one section.
    rev = [i for i in range(1, len(pts) - 1)
           if VC.corner_turn_angle_deg(pts, i) > REVERSAL_STOP_DEG]
    bounds = [0] + rev + [len(pts) - 1]
    for k in range(len(bounds) - 1):
        seg = pts[bounds[k]:bounds[k + 1] + 1]
        if len(seg) >= 2:
            sec = _continuous_section(seg, delta_mm, L, safety, ps,
                                      la_max_mm, v_straight, char)
            if sec is not None:
                plan.sections.append(sec)
    plan.n_stops = max(0, len(plan.sections) - 1)
    plan.est_time_s = sum(s.est_time_s for s in plan.sections) \
        + plan.n_stops * STOP_COST_S
    plan.notes.append(
        "continuous — corners are slowed through, never stopped on; the pump "
        "keeps advancing for the whole of each section")
    slowest = min((s.speed_at.min_value() for s in plan.sections
                   if s.speed_at is not None), default=v_straight)
    if slowest < v_straight - 1e-9:
        plan.notes.append(f"slowest point {slowest:.2f} mm/s (straights "
                          f"{v_straight:.2f} mm/s)")
    if plan.n_stops:
        plan.notes.append(
            f"⚠ {plan.n_stops} unavoidable stop(s) at near-180° reversal(s): a "
            f"cusp cannot be traversed by pure pursuit at any speed, so the "
            f"plan stops there and the pump pauses with it")
    return plan


def _continuous_section(pts, delta_mm, L, safety, ps, la_max_mm, v_straight,
                        char: StageCharacteristics):
    """One never-stopping section: arc-length lookahead + speed profiles."""
    cum = VC.polyline_arclength(pts)
    total = cum[-1]
    if total < 1e-9:
        return None

    # ── one knot per vertex whose geometry needs a smaller carrot ──
    knots = []                       # (s, lookahead_mm, speed_mm_s, angle, R)
    sharpest = 0.0
    for i in range(1, len(pts) - 1):
        ang = VC.corner_turn_angle_deg(pts, i)
        sharpest = max(sharpest, ang)
        r = local_radius_mm(pts, i)
        la_arc = (LA_CURVE_MARGIN * math.sqrt(2.0 * r * delta_mm)
                  if math.isfinite(r) else la_max_mm)
        # Both models apply; the binding one is whichever asks for less. A
        # sampled arc is constrained by curvature, a real vertex by its angle.
        la_i = min(la_arc, corner_budget_lookahead_mm(ang, delta_mm))
        la_i = max(LA_MIN_CONTINUOUS_MM, min(la_max_mm, la_i))
        v_i = min(ps, la_i / (L * safety)) if L > 1e-6 else ps
        if la_i < la_max_mm - 1e-12 or v_i < v_straight - 1e-12:
            knots.append((cum[i], la_i, v_i, ang, r))

    ramp = max(0.15, ps * (L + char.tau_s) * RAMP_LAG_MARGIN)
    # Command the slowdown this far EARLY, so the stage is actually slow by the
    # time it reaches the corner rather than one lag-length later.
    lead = ps * (char.dead_time_s + char.tau_s
                 + 0.5 * char.control_loop_ms / 1000.0) * LEAD_LAG_MARGIN

    if not knots:
        la_prof = ArcProfile([0.0], [la_max_mm], la_max_mm)
        v_prof = ArcProfile([0.0], [v_straight], v_straight)
        reason = "straight — full lookahead, no slowdowns"
    else:
        step = max(1e-3, min(PROFILE_STEP_MM, ramp / 8.0))
        n = int(total / step) + 1
        if n > PROFILE_MAX_SAMPLES:
            step = total / (PROFILE_MAX_SAMPLES - 1)
            n = PROFILE_MAX_SAMPLES
        grid = [min(total, i * step) for i in range(n)]
        # the knot positions themselves must be sample points, or a narrow
        # minimum can fall between grid lines and never be commanded
        grid.extend(k[0] for k in knots)
        grid = sorted(set(grid))
        la_vals, v_vals = [], []
        for sg in grid:
            la, v = la_max_mm, v_straight
            for (sc, la_c, v_c, _a, _r) in knots:
                # Lead only on the APPROACH: pretend we are already `lead` mm
                # further along, so braking is commanded early enough to have
                # taken effect. Past the corner, distance is measured normally so
                # the stage does not accelerate away before it has turned.
                d = (sc - (sg + lead)) if (sg + lead) < sc else (sg - sc)
                f = min(1.0, max(0.0, d) / ramp)
                la = min(la, la_c + (la_max_mm - la_c) * f)
                v = min(v, v_c + (v_straight - v_c) * f)
            la_vals.append(la)
            v_vals.append(v)
        la_prof = ArcProfile(grid, la_vals, la_max_mm)
        v_prof = ArcProfile(grid, v_vals, v_straight)
        reason = (f"{len(knots)} slowdown(s), sharpest {sharpest:.0f}° → "
                  f"lookahead {la_prof.min_value():.3f}–{la_max_mm:.3f} mm, "
                  f"speed {v_prof.min_value():.2f}–{v_straight:.2f} mm/s over "
                  f"{ramp:.2f} mm ramps")

    # est time = ∫ ds/v(s) over the profile grid (no stop cost — no stops here)
    est = 0.0
    g = v_prof._g
    if len(g) >= 2:
        for a, b in zip(g, g[1:]):
            est += (b - a) / max(0.05, v_prof(a))
    else:
        est = total / max(0.05, v_straight)

    return PlanSection(
        pts=list(pts), lookahead_mm=la_max_mm, speed_mm_s=v_straight,
        min_radius_mm=min((k[4] for k in knots), default=float("inf")),
        length_mm=total, est_time_s=est, reason=reason,
        lookahead_at=la_prof, speed_at=v_prof)


# ── Running a plan (simulated or real — same code) ────────────────────

def tuning_for(section: PlanSection) -> dict:
    """The follower tuning one section runs with. Corner handling is OFF —
    sections contain no sharp corners by construction (they were split), and
    hold_speed pins the section's computed speed.

    Public so the real print executor (``PrintManager._run_plan_section``) runs
    the SAME per-section numbers the simulation does — the shared-law contract
    extended to the plan.
    """
    return dict(PS.TUNING_DEFAULTS,
                lookahead=section.lookahead_mm,
                decel=max(0.15, min(0.5, section.length_mm * 0.3)),
                corner_angle=89.0, corner_factor=1.0,
                hold_speed=1.0, max_speed_frac=0.9)


#: Back-compat alias (the private name predates the print integration).
_tuning_for = tuning_for


def section_end_taper(char: StageCharacteristics,
                      section: PlanSection) -> tuple:
    """``(end_lag_s, end_floor_frac)`` for one section's lag-aware end taper.

    The naive ``remaining/decel`` taper commands the speed appropriate to where
    the stage WAS one dead time ago, so it crosses the section end fast and
    overshoots by ≈ v·lag (measured 153 µm at 3 mm/s on the ME3B model). Sizing
    the taper by the measured lag lands it ON the vertex instead — which is what
    makes a corner STOP cost ~8 µm rather than ~150 µm, and therefore what makes
    the ≤ 30 µm element reachable at a corner.
    """
    lag = (char.dead_time_s + char.tau_s
           + 0.5 * char.control_loop_ms / 1000.0) * END_LAG_MARGIN
    floor = min(0.1, END_FLOOR_MM_S / max(0.05, section.speed_mm_s))
    return lag, floor


def run_plan(plan: FeedPlan, io, char: StageCharacteristics, *, stop=None,
             on_section=None):
    """Execute every section through the shared ``follow_path`` over ``io``
    (bind to the stage model → simulation; to the controller → hardware).
    Returns ``(samples, info)`` — samples concatenated across sections with
    monotone timestamps, ``info`` mirrors follow_path's plus per-section data.
    """
    all_samples = []
    section_infos = []
    t_base = 0.0
    completed = True
    reason = "arrived"
    for k, sec in enumerate(plan.sections):
        if stop is not None and stop.is_set():
            completed, reason = False, "stopped"
            break
        tuning = tuning_for(sec)
        resolved = PS.resolve_for(char, print_speed_mm_s=sec.speed_mm_s,
                                  tuning=tuning)
        # lag-aware end taper: land ON the split vertex instead of flying
        # v·lag past it (the follower's naive taper measured 153 µm over)
        lag, floor = section_end_taper(char, sec)
        # v7.7: a CONTINUOUS section carries arc-length profiles — the corners
        # inside it are slowed through, so the wall cap must be sized on the
        # slowest point rather than the straight-line speed.
        v_worst = sec.speed_mm_s
        if sec.speed_at is not None:
            try:
                v_worst = max(0.05, sec.speed_at.min_value())
            except Exception:
                v_worst = sec.speed_mm_s
        samples, info = PS.follow_path(
            sec.pts, io, print_speed_mm_s=sec.speed_mm_s, tuning=tuning,
            resolved=resolved, stop=stop, arrive_mm=sec.arrive_mm,
            end_lag_s=lag, end_floor_frac=floor,
            lookahead_at=sec.lookahead_at, speed_limit_at=sec.speed_at,
            max_wall_s=sec.length_mm / max(0.05, v_worst) * 8.0 + 10.0)
        if k < len(plan.sections) - 1:
            # settle dwell at the split: the residual coast decays before the
            # next section starts, so every section begins from rest ON its
            # start vertex (follow_path already sent VS 0,0)
            io.sleep(0.3)
        for sm in samples:
            t = sm.t if sm.t is not None else 0.0
            all_samples.append(XC.Sample(sm[0], sm[1], t_base + t))
        t_base += info["wall_s"]
        section_infos.append({"index": k, "info": info,
                              "lookahead_mm": sec.lookahead_mm,
                              "speed_mm_s": sec.speed_mm_s})
        if on_section is not None:
            on_section(k, len(plan.sections), info)
        if not info["completed"]:
            completed, reason = False, info["stopped_reason"]
            break
    return all_samples, {"completed": completed,
                         "stopped_reason": reason if not completed
                         else "arrived",
                         "wall_s": t_base, "sections": section_infos}


def simulate_plan(plan: FeedPlan, char: StageCharacteristics,
                  *, resolution_um=None) -> PS.SimResult:
    """Simulate the whole plan on the saved stage model and score it against
    the FULL original path."""
    res_um = float(resolution_um or plan.element_um)
    if not plan.sections:
        return PS.SimResult(verdict="fail", stopped_reason="empty")
    start = plan.sections[0].pts[0]
    model = XYStageModel(char, x_um=start[0] * 1000.0, y_um=start[1] * 1000.0)
    io = PS.model_io(model)

    # between sections the stage has stopped ON the split vertex — no teleport
    samples, info = run_plan(plan, io, char)
    report = XC.path_report(samples, plan.ideal,
                            commanded_speed_mm_s=plan.target_speed_mm_s,
                            resolution_um=res_um,
                            status="ok" if info["completed"]
                            else info["stopped_reason"],
                            wall_s=info["wall_s"])
    score = XC.composite_score(report, resolution_um=res_um)
    cross = []
    for s in info["sections"]:
        cross.extend(s["info"]["cross_profile"])
    return PS.SimResult(
        samples=samples, report=report, score=score,
        verdict=XC.verdict_for(report, res_um),
        resolved={"sections": len(plan.sections)},
        cross_profile=cross,
        fail_regions=[],
        completed=info["completed"], stopped_reason=info["stopped_reason"],
        wall_s=info["wall_s"])


# ── The deterministic all-axis trajectory (time-matched rows) ─────────

def plan_rows(plan: FeedPlan, *, vol_per_mm_uL=0.0, z_mm=0.0,
              pump_index=0) -> list:
    """Export the plan as time-parameterised rows ``[t_s, x, y, z, p1, p2,
    p3]`` (mm/µL): X/Y from each section's vertices at its planned speed with
    a stop dwell at splits, Z constant (in-path printing never moves Z), pump
    volume ∝ arc length — so pump and Z are TIME-MATCHED to the planned XY by
    construction, and the µL column stays exact even if the stage runs behind
    plan (the runtime follower also deposits per measured Δs)."""
    rows = []
    t = 0.0
    vol = 0.0
    pumps = [0.0, 0.0, 0.0]

    def emit(x, y):
        pumps[pump_index] = vol
        rows.append([t, x, y, z_mm] + list(pumps))

    for k, sec in enumerate(plan.sections):
        v = max(0.05, sec.speed_mm_s)
        pts = sec.pts
        if not rows:
            emit(pts[0][0], pts[0][1])
        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0],
                           pts[i][1] - pts[i - 1][1])
            t += d / v
            vol += d * vol_per_mm_uL
            emit(pts[i][0], pts[i][1])
        if k < len(plan.sections) - 1:
            t += 0.35                     # stop/settle dwell at the split
            emit(pts[-1][0], pts[-1][1])
    return rows
