"""XYCalibrationRun.py — ONE XY calibration: measure → commit → derive → tune → verify.

WHY THIS EXISTS
───────────────
The XY tuning used to be a row of single-setting buttons (settle sweep, measure top
speed, apply, check comms, auto-tune, ZN, PID from dead time, robustness, measure dead
time) whose ORDER was load-bearing and undocumented — every later one consumes an
earlier measurement. Running them piecemeal produced an inconsistent set, and the
"one-click" path that was supposed to fix that **could not complete**:

* it never measured the top speed, it only READ it from
  ``PrintTimingCalibrationStore``; and
* nothing writes that store any more (v7.21.1 correctly stopped the timing worker from
  silently overwriting the operator's declared value, but never repointed the store's
  seven readers).

So on any real machine ``measured_from_store()`` reported ``top_speed_um_s = 0``,
``is_complete()`` was False, and the run aborted telling the operator to press a button
that now writes ``settings.json`` instead of the store — an unbreakable loop.

This module runs the whole sequence itself, in the one order that works, and writes the
measured top speed to **every** home so no consumer can be left reading a stale zero.

WHAT IT DOES
────────────
    centre → comms rate → dead time → top speed → COMMIT the top speed
           → derive (closed form) → grid tune (SIMULATION) → verify (hardware) → persist

**The grid is scored in simulation, not on the stage.** ``XYPathSimulator`` is
stdlib-only and runs on a virtual clock, so one candidate costs ~20–55 ms instead of the
20–60 s a hardware run costs; a 15×15 grid is ~12 s where the coordinate descent it
replaces was 8–22 MINUTES. It drives the SAME ``follow_path`` loop the hardware verify
uses, so simulation and stage differ only by the stage itself. Only the winner is driven
for real, as the accept/reject gate.

TWO THINGS THAT ARE EASY TO GET WRONG HERE
──────────────────────────────────────────
1. **The simulator's tuning keys are NOT the store's keys.**
   ``XYPathSimulator`` uses ``lookahead`` / ``corner_factor`` / ``pid_kp`` → ``kp``;
   the store uses ``lookahead_mm`` / ``corner_speed_factor`` / ``pid_kp``.
   ``resolve_for`` does ``p.update(tuning)`` then reads ``p["lookahead"]``, so passing a
   store-named key is **silently ignored** and every grid cell scores identically. All
   conversion goes through :func:`tuning_from_store_keys`, and a test pins that varying
   each axis actually moves the score.

2. **The requested speed is not the speed that runs.** ``resolve_control`` caps the
   command at ``lookahead / (L · safety)``. On ME3B_01 a requested 3.0 mm/s actually
   runs at 0.43 mm/s with ``cap_reason="dead_time"``. Everything reported to the
   operator quotes the RESOLVED cap, never the request.

SAFETY
──────
XY only — nothing here moves Z except through ``safe_travel_to``/``ensure_retracted_to``
(retract → confirm → travel, never descends). Every probe already stops the stage and
returns home in its own ``finally``; the outer ``finally`` here stops the stage and
re-retracts regardless of how the run ended. Every excursion is bounded by
``check_fits`` so no probe can reach a travel extent — a clamped probe does not fail
loudly, it silently measures the clamp.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

from SupportClasses import XYAutoCalibration as AC
from SupportClasses import XYChallenge as XC
from SupportClasses import XYDeadTime as DT
from SupportClasses import XYPathSimulator as PS
from SupportClasses import XYTopSpeed as TS
from SupportClasses.XYFeedPlan import corner_budget_lookahead_mm
from SupportClasses.XYStageModel import StageCharacteristics

logger = logging.getLogger(__name__)

#: Operator-facing grid coarseness → points per axis. The grid is 2-D
#: (lookahead × corner speed factor), so candidate count is the square.
GRID_LEVELS = {
    "very_coarse": 3, "coarse": 5, "medium": 7, "fine": 11, "very_fine": 15,
}
GRID_LEVEL_ORDER = ("very_coarse", "coarse", "medium", "fine", "very_fine")
GRID_LEVEL_LABELS = {
    "very_coarse": "Very coarse — 3×3 (9)",
    "coarse": "Coarse — 5×5 (25)",
    "medium": "Medium — 7×7 (49)",
    "fine": "Fine — 11×11 (121)",
    "very_fine": "Very fine — 15×15 (225)",
}

STEP_ORDER = ("centre", "comms", "deadtime", "topspeed", "commit",
              "derive", "grid", "verify", "persist")
STEP_TITLES = {
    "centre": "Centring", "comms": "Comms rate", "deadtime": "Dead time",
    "topspeed": "Top speed", "commit": "Applying top speed",
    "derive": "Deriving settings", "grid": "Tuning (simulated grid)",
    "verify": "Verifying on the stage", "persist": "Saving",
}
#: Nominal seconds per step, for the progress clock's remaining-time estimate.
#: Measured on ME3B_01; only used for display and for the degrade decision.
STEP_NOMINAL_S = {"centre": 6.0, "comms": 4.0, "deadtime": 8.0, "topspeed": 24.0,
                  "commit": 0.5, "derive": 0.5, "grid": 4.0, "verify": 9.0,
                  "persist": 4.0}

#: Ties within this fraction of the best score are broken on throughput, not noise.
TIE_REL = 0.02
TIE_ABS = 0.05
#: ``resolve_control`` floors the lookahead here, so the grid cannot go below it.
MIN_LOOKAHEAD_MM = 0.05

#: A measurement this far from an existing declaration, on a poor fit, is refused
#: rather than committed — the residual protection now that Apply is automatic.
COMMIT_DISAGREE_RATIO = 3.0
COMMIT_MIN_R2 = 0.98


# ── request / result types ────────────────────────────────────────────────

@dataclass
class CalibrationRequest:
    target_speed_mm_s: float = 5.0
    resolution_um: float = 30.0
    grid_level: str = "medium"
    shape: str = "Star"
    feature_mm: float = 2.0
    step_mm: float = 0.1
    safe_z_mm: float | None = None
    top_speed_max_dist_mm: float = 10.0
    top_speed_repeats: int = 2
    top_speed_points: int = 5
    deadtime_repeats: int = 5
    comms_iterations: int = 40
    verify_on_hardware: bool = True
    time_budget_s: float = 180.0

    def grid_points(self) -> int:
        return GRID_LEVELS.get(str(self.grid_level), GRID_LEVELS["medium"])


@dataclass
class StepProgress:
    step: str
    title: str
    state: str = "pending"        # pending|running|done|failed|skipped|aborted
    frac: float = 0.0
    message: str = ""
    elapsed_s: float = 0.0
    remaining_s: float = 0.0
    detail: dict = field(default_factory=dict)


@dataclass
class GridCandidate:
    lookahead_mm: float
    corner_speed_factor: float
    score: float
    passed: bool
    p95_um: float
    max_um: float
    rms_um: float
    completion: float
    dither: float
    fail_reasons: list = field(default_factory=list)


@dataclass
class GridResult:
    candidates: list = field(default_factory=list)
    best: GridCandidate | None = None
    lookahead_axis: list = field(default_factory=list)
    corner_axis: list = field(default_factory=list)
    speed_mm_s: float = 0.0
    resolved_cap_mm_s: float = 0.0
    cap_reason: str = ""
    n: int = 0
    n_pass: int = 0
    corner_axis_inert: bool = False
    sim_s: float = 0.0

    @property
    def all_failed(self) -> bool:
        return self.n > 0 and self.n_pass == 0


@dataclass
class CalibrationResult:
    ok: bool = False
    summary: str = ""
    aborted: bool = False
    error: str = ""
    measured: object = None            # AC.MeasuredMachine
    derived: object = None             # AC.Derived
    top_speed_um_s: float = 0.0
    top_speed_fit: dict = field(default_factory=dict)
    grid: GridResult | None = None
    verify: dict = field(default_factory=dict)
    feed_plan: dict = field(default_factory=dict)
    applied: dict = field(default_factory=dict)
    steps: list = field(default_factory=list)
    wall_s: float = 0.0


# ── tuning-dict plumbing (trap 1) ─────────────────────────────────────────

#: store key → simulator key, the inverse of ``XYPathSimulator._STORE_KEY_MAP``.
_STORE_TO_SIM = {v: k for k, v in PS._STORE_KEY_MAP.items()}


def tuning_from_store_keys(values: dict) -> dict:
    """Build a SIMULATOR tuning dict from STORE-named values.

    The two vocabularies differ (``lookahead_mm`` vs ``lookahead``) and
    ``resolve_for`` silently ignores an unknown key, so a hand-built dict using
    store names produces a grid where every cell scores identically. This is the one
    conversion point.
    """
    out = dict(PS.TUNING_DEFAULTS)
    for store_key, val in (values or {}).items():
        sim_key = _STORE_TO_SIM.get(store_key)
        if sim_key is None:
            continue
        try:
            out[sim_key] = float(val)
        except (TypeError, ValueError):
            continue
    return out


def sharpest_turn_deg(pts) -> float:
    """Largest turn angle on the polyline (0 = straight, 180 = reversal).

    This is what sizes the grid's lookahead floor: pure pursuit chords across a
    vertex, cutting ``≈ (lookahead/2)·sin(θ/2)``, so the sharpest vertex sets the
    tightest lookahead the shape can tolerate.
    """
    worst = 0.0
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-9 or nb < 1e-9:
            continue
        c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
        worst = max(worst, math.degrees(math.acos(c)))
    return worst


def _geomspace(lo: float, hi: float, n: int) -> list:
    lo = max(1e-6, float(lo))
    hi = max(lo * 1.0001, float(hi))
    if n <= 1:
        return [lo]
    return [lo * (hi / lo) ** (i / (n - 1)) for i in range(n)]


def _linspace(lo: float, hi: float, n: int) -> list:
    if n <= 1:
        return [lo]
    return [lo + (hi - lo) * i / (n - 1) for i in range(n)]


def build_grid(derived, request: CalibrationRequest, ideal_mm) -> tuple:
    """``(lookahead_axis, corner_axis)`` for the tune.

    🔴 **The lookahead floor is the CORNER BUDGET, not a fraction of the derived
    value.** ``derive_settings`` solves ``la0 = target · L · safety · margin`` — it
    sizes the lookahead so the target SPEED is permitted, which is a different
    question from what the shape's corners tolerate. On ME3B_01 with a 2 mm star,
    ``la0`` at 2 mm/s is 0.838 mm while the 142° tips tolerate 0.040 mm — 21× apart.
    Measured head to head on this rig:

    ===========  ==========================  ==============================
    target       axis ``[la0/4 … 2·la0]``    axis ``[corner_budget … 2·la0]``
    ===========  ==========================  ==============================
    0.6 mm/s     p95 14.9 µm  PASS           p95 12.6 µm  PASS
    1.0 mm/s     p95 19.6 µm  PASS           p95 13.1 µm  PASS
    2.0 mm/s     p95 40.2 µm  **FAIL**       p95 15.3 µm  **PASS**
    ===========  ==========================  ==============================

    A ``la0/4`` floor sits above the optimum at every useful speed and misses it
    outright at 2 mm/s — which reads as "no tuning can print this shape" when in
    fact the machine holds it comfortably.

    Geometric spacing because the speed cap goes as ``1/la`` and the corner cut as
    ``la`` — the structure is multiplicative, so a linear axis wastes half its points
    in the flat region above ``la0``.
    """
    n = request.grid_points()
    la0 = float((derived.values or {}).get("lookahead_mm", 0.0) or 0.0)
    budget = corner_budget_lookahead_mm(
        sharpest_turn_deg(ideal_mm), max(1e-4, request.resolution_um / 1.6 / 1000.0))
    if not math.isfinite(budget):
        budget = MIN_LOOKAHEAD_MM
    lo = max(MIN_LOOKAHEAD_MM, budget)
    hi = min(max(2.0 * la0, lo * 4.0), max(0.5 * request.feature_mm, lo * 2.0))
    if hi <= lo * 1.05:
        hi = lo * 4.0
    return (_geomspace(lo, hi, n), _linspace(0.05, 1.0, n))


# ── the grid (pure: no hardware, no Qt) ───────────────────────────────────

def path_length_mm(pts) -> float:
    return sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts)))


def _sim_wall_cap_s(ideal_mm, effective_speed_mm_s: float) -> float:
    """Virtual-time cap for one candidate.

    Simulation cost is proportional to the number of ticks, i.e. to the path's
    DURATION, so a slow candidate is also an expensive one. A flat 40 s cap made a
    0.2 mm/s candidate cost ~560 ms against ~25 ms at 2 mm/s — enough to push a
    15x15 grid past two minutes. Three times the nominal traverse is generous
    enough that a genuinely-following run always completes, while a stalled one is
    cut off early (and a truncated run is already scored as incomplete).
    """
    nominal = path_length_mm(ideal_mm) / max(0.05, float(effective_speed_mm_s))
    return max(6.0, min(60.0, 2.5 * nominal))




def candidate_values(base_values: dict, *, lookahead_mm: float,
                     corner_factor: float) -> dict:
    """Store-named values for ONE grid candidate.

    🔴 Two derived levers MUST be neutralised or the grid is silently inert:

    * ``min_lookahead_frac`` — ``resolve_control`` does
      ``la = max(la, mlf · speed · L)``. ``derive_settings`` sets ``mlf`` precisely so
      the runtime re-derives ``la0``, so inheriting it raises EVERY candidate's
      lookahead back to ``la0``: the axis varies, the simulation does not, and the
      grid returns an arbitrary point that looks like a result.
    * ``hold_speed`` — pins ``speed_cap`` to the requested speed, defeating the
      dead-time cap that a small lookahead is supposed to impose. With it on, a
      candidate that could not physically follow the path is scored as if it could.

    Both are re-derived from the WINNER in :func:`finalise_values`, so what is
    persisted still reproduces exactly what was simulated.
    """
    vals = dict(base_values)
    vals["lookahead_mm"] = float(lookahead_mm)
    vals["corner_speed_factor"] = float(corner_factor)
    vals["min_lookahead_frac"] = 0.0
    vals["hold_speed"] = 0.0
    return vals


def finalise_values(derived, best, *, speed_mm_s: float, char) -> dict:
    """Re-derive the two neutralised levers from the winning lookahead so the
    PRINT path reproduces the tuning that was simulated.

    ``min_lookahead_frac`` is recomputed so ``resolve_control``'s floor lands on the
    chosen lookahead rather than the closed-form one, and ``hold_speed`` is set only
    when the chosen lookahead genuinely sustains the chosen speed.
    """
    vals = dict(derived.values or {})
    if best is None:
        return vals
    vals["lookahead_mm"] = float(best.lookahead_mm)
    vals["corner_speed_factor"] = float(best.corner_speed_factor)
    safety = float(vals.get("deadtime_safety", 2.0) or 2.0)
    dead_total = char.dead_time_s + char.control_loop_ms / 1000.0
    sp = max(0.05, float(speed_mm_s))
    if dead_total > 1e-6:
        vals["min_lookahead_frac"] = round(best.lookahead_mm / (sp * dead_total), 3)
        cap = best.lookahead_mm / (dead_total * max(1.0, safety))
        vals["hold_speed"] = 1.0 if cap >= sp else 0.0
    return vals


def _sim_one(ideal_mm, char, *, base_values, lookahead_mm, corner_factor,
             speed_mm_s, resolution_um):
    tuning = tuning_from_store_keys(candidate_values(
        base_values, lookahead_mm=lookahead_mm, corner_factor=corner_factor))
    resolved = PS.resolve_for(char, print_speed_mm_s=speed_mm_s, tuning=tuning)
    cap = float(resolved["speed_cap_mm_s"])
    # Score against the speed the follower was CAPPED to, not the speed requested.
    # composite_score's time term compares wall-clock to the nominal traverse at the
    # commanded speed; using the request would charge every candidate for the cap
    # (a factor of 10 on a small lookahead) — a penalty that says nothing about
    # tuning quality and would simply bias the grid toward the largest lookahead.
    res = PS.simulate_follow(
        ideal_mm, char=char, print_speed_mm_s=speed_mm_s, tuning=tuning,
        resolution_um=resolution_um, max_wall_s=_sim_wall_cap_s(ideal_mm, cap))
    res.report.update(XC.path_report(
        res.samples, ideal_mm, commanded_speed_mm_s=cap,
        resolution_um=resolution_um,
        corner_angle_deg=float(tuning["corner_angle"]),
        status="ok" if res.completed else res.stopped_reason,
        wall_s=res.wall_s))
    res.score.update(XC.composite_score(res.report, resolution_um=resolution_um))
    res.resolved = resolved
    return res, tuning


def score_grid(ideal_mm, char, *, derived, request: CalibrationRequest,
               stop_evt=None, on_candidate=None) -> GridResult:
    """Exhaustive 2-D grid over lookahead × corner speed factor, scored entirely in
    simulation, returning the FASTEST candidate that holds the resolution element.

    **Speed is an output, not an input.** ``resolve_control`` caps the commanded speed
    at ``lookahead / (L · safety)``, so on a small feature the requested speed is
    almost never what runs: the lookahead sets both the accuracy AND the achievable
    speed. The grid therefore commands the operator's target and lets the cap bind,
    and the answer to "how fast can this machine print this shape?" is the winner's
    RESOLVED cap.

    **The sweep runs from the LARGEST lookahead down.** The cap is monotonic in the
    lookahead, so the first lookahead row that yields a pass already contains the
    fastest passing candidate — no smaller lookahead can be faster. Stopping there is
    not a heuristic, it is the optimum under this objective, and it is also much
    cheaper: simulation cost is proportional to path DURATION, so the fast candidates
    examined first are the cheap ones (≈20 ms) and the slow ones (≈160 ms) are only
    ever reached when nothing passes.

    The objective is ``composite_score``, never ``path_error`` — the latter is
    one-sided and untimed, so a crawling, retraced or early-stopped run scores BEST
    under it, which is exactly why the coordinate descent this replaces drove the
    stored tuning to the minimum of both its grids.
    """
    t0 = time.monotonic()
    base_values = dict(derived.values or {})
    la_axis, cf_axis = build_grid(derived, request, ideal_mm)
    speed = max(0.05, float(request.target_speed_mm_s))

    out = GridResult(lookahead_axis=la_axis, corner_axis=cf_axis, speed_mm_s=speed)
    caps: dict = {}
    scores_by_la: dict = {}
    for la in sorted(la_axis, reverse=True):
        row_passed = False
        for cf in cf_axis:
            if stop_evt is not None and stop_evt.is_set():
                break
            r, tun = _sim_one(ideal_mm, char, base_values=base_values,
                              lookahead_mm=la, corner_factor=cf,
                              speed_mm_s=speed,
                              resolution_um=request.resolution_um)
            cand = GridCandidate(
                lookahead_mm=la, corner_speed_factor=cf,
                score=float(r.score.get("score", 1e9)),
                passed=bool(r.score.get("pass")),
                p95_um=float(r.report.get("p95_um", 0.0)),
                max_um=float(r.report.get("max_um", 0.0)),
                rms_um=float(r.report.get("rms_um", 0.0)),
                completion=float(r.report.get("completion_frac", 0.0)),
                dither=float(r.report.get("dither_ratio", 0.0)),
                fail_reasons=list(r.score.get("fail_reasons", [])))
            caps[(round(la, 6), round(cf, 6))] = float(
                r.resolved.get("speed_cap_mm_s", 0.0))
            out.candidates.append(cand)
            scores_by_la.setdefault(round(la, 6), set()).add(round(cand.score, 6))
            row_passed = row_passed or cand.passed
            if on_candidate:
                try:
                    on_candidate(cand)
                except Exception:
                    pass
        if stop_evt is not None and stop_evt.is_set():
            break
        if row_passed:
            # Monotonic cap ⇒ nothing below this lookahead can be faster.
            break

    out.n = len(out.candidates)
    out.n_pass = sum(1 for c in out.candidates if c.passed)

    # The corner axis is INERT when the dead-time cap sits below every corner limit:
    # ``min(cap, corner_limit)`` is then always the cap and the factor changes
    # nothing. Say so, or the operator reads N identical scores as a broken tool.
    out.corner_axis_inert = bool(
        len(cf_axis) > 1 and scores_by_la
        and all(len(v) == 1 for v in scores_by_la.values()))

    out.best = _pick_best(out.candidates, caps=caps, la0=float(
        (derived.values or {}).get("lookahead_mm", 0.0) or 0.0))
    if out.best is not None:
        key = (round(out.best.lookahead_mm, 6),
               round(out.best.corner_speed_factor, 6))
        out.resolved_cap_mm_s = caps.get(key, 0.0)
        res = PS.resolve_for(char, print_speed_mm_s=speed,
                             tuning=tuning_from_store_keys(candidate_values(
                                 base_values,
                                 lookahead_mm=out.best.lookahead_mm,
                                 corner_factor=out.best.corner_speed_factor)))
        out.cap_reason = str(res.get("cap_reason", ""))
    out.sim_s = time.monotonic() - t0
    return out


def _pick_best(candidates, *, caps: dict, la0: float):
    """The FASTEST candidate that holds the element; failing that, the least bad.

    Among passing candidates the ranking is the achievable speed (the resolved cap),
    because "speed is an output" is the whole point — picking the lowest deviation
    instead would always choose the smallest lookahead and reproduce the
    tuning-brings-the-velocity-down failure this replaces.

    Ties (within ``max(TIE_ABS, TIE_REL × best)`` on the tie-breaking quantity) then
    prefer the LARGER corner speed factor — throughput again, and an explicit guard
    against parking at the corner floor — then the lookahead nearest the closed-form
    value we can explain, then index order so the same machine measured twice gives
    the same answer.
    """
    if not candidates:
        return None

    def _cap(c):
        return caps.get((round(c.lookahead_mm, 6),
                         round(c.corner_speed_factor, 6)), 0.0)

    passing = [c for c in candidates if c.passed]
    if passing:
        best_cap = max(_cap(c) for c in passing)
        band = max(TIE_ABS * 0.01, TIE_REL * abs(best_cap))
        tied = [c for c in passing if _cap(c) >= best_cap - band]
    else:
        best_score = min(c.score for c in candidates)
        band = max(TIE_ABS, TIE_REL * abs(best_score))
        tied = [c for c in candidates if c.score <= best_score + band]

    def _key(c):
        near = (abs(math.log(max(c.lookahead_mm, 1e-6) / la0)) if la0 > 0 else 0.0)
        return (-c.corner_speed_factor, c.score, near)

    return sorted(tied, key=_key)[0]


# ── the top-speed write: one function, five homes ─────────────────────────

def commit_top_speed(controller, settings, store, um_s: float, *,
                     fit: dict | None = None) -> dict:
    """Make a measured XY top speed authoritative EVERYWHERE, in one call.

    Order matters. The store is written FIRST because it is the home nobody writes
    today and the one that unblocks everything else: ``measured_from_store`` (so the
    derivation can run at all), ``StageCharacteristics.from_store`` (so the simulator
    models a CLAMPED stage — ``XYStageModel.command_velocity`` only clamps
    ``if top > 0``, so an empty store silently models an infinitely fast stage and
    every grid candidate looks better than it will be), and ``stamp_print_settings``
    (which otherwise stamps ``xy_max_speed_um_s = 0.0`` onto every print job).

    Then the live controller — one existing setter that moves the declaration, the
    ``safety_limits`` mirror, the ``XYStage`` mm/s↔SMS denominator and the jog anchors
    together — then both ``settings.json`` keys.

    ``notify_speed_limits_changed()`` is deliberately NOT called here: it runs
    ``refresh_jog_speed_limits`` inline and is not thread-safe, so the caller does it
    on the GUI thread. It is reported as ``applied["notify_pending"]``.
    """
    applied = {"store": False, "declaration": False, "settings_safety": False,
               "settings_device_profile": False, "notify_pending": False,
               "ok": False, "reason": ""}
    try:
        v = float(um_s)
    except (TypeError, ValueError):
        applied["reason"] = "measurement is not a number"
        return applied
    if v <= 0:
        applied["reason"] = "no usable measurement"
        return applied

    # Residual protection now that the write is automatic: a wildly different value
    # from a poor fit is refused rather than silently rescaling every mm/s command.
    r2 = float((fit or {}).get("r2", 1.0) or 0.0)
    declared = None
    try:
        if hasattr(controller, "declared_xy_top_speed_um_s"):
            declared = controller.declared_xy_top_speed_um_s()
    except Exception:
        declared = None
    if declared and r2 < COMMIT_MIN_R2:
        ratio = v / float(declared)
        if ratio > COMMIT_DISAGREE_RATIO or ratio < 1.0 / COMMIT_DISAGREE_RATIO:
            applied["reason"] = (
                f"measured {v / 1000.0:.2f} mm/s disagrees with the configured "
                f"{float(declared) / 1000.0:.2f} mm/s by {ratio:.1f}× on a poor fit "
                f"(R²={r2:.3f}) — not applied; re-run or set it by hand")
            return applied

    if store is not None:
        try:
            store.set_xy_max_speed_um_s(v)
            applied["store"] = True
        except Exception as e:
            logger.debug("commit_top_speed: store write failed: %s", e)
    if controller is not None and hasattr(controller, "set_xy_top_speed_um_s"):
        try:
            controller.set_xy_top_speed_um_s(v)
            applied["declaration"] = True
            applied["notify_pending"] = True
        except Exception as e:
            logger.debug("commit_top_speed: declaration failed: %s", e)
    if settings is not None:
        try:
            settings.set("safety_limits.max_xy_speed", v)
            applied["settings_safety"] = True
            settings.set("device_profile.xy_max_speed_um_s", v)
            applied["settings_device_profile"] = True
            settings.set("device_profile.xy_max_speed_source", "measured")
            settings.set("device_profile.xy_max_speed_measured_at",
                         time.strftime("%Y-%m-%dT%H:%M:%S"))
            settings.save()
        except Exception as e:
            logger.debug("commit_top_speed: settings write failed: %s", e)
    applied["ok"] = applied["store"] or applied["declaration"]
    return applied


# ── hardware verify ───────────────────────────────────────────────────────

def verify_on_hardware(controller, ideal_mm, *, tuning, speed_mm_s, resolution_um,
                       char, safe_z_mm=None, stop_evt=None) -> dict:
    """Drive the winning tuning on the REAL stage, once, and score it.

    Uses ``XYPathSimulator.follow_path`` with ``controller_io`` — the same loop the
    grid simulated — so a sim/hardware disagreement is a statement about the stage,
    never about two different follower implementations.
    """
    out = {"ran": False, "pass": False, "reason": ""}
    if not getattr(controller, "is_xy_connected", False):
        out["reason"] = "XY not connected"
        return out
    start = ideal_mm[0]
    try:
        if safe_z_mm is not None and hasattr(controller, "safe_travel_to"):
            z = getattr(controller, "zero_position", {}) or {}
            controller.safe_travel_to(
                start[0] * 1000.0 + z.get("x", 0.0),
                start[1] * 1000.0 + z.get("y", 0.0),
                safe_z_mm=float(safe_z_mm), target_z_mm=None)
        elif hasattr(controller, "move_xy_absolute"):
            controller.move_xy_absolute(start[0], start[1], from_zero_ref=True)
    except Exception as e:
        out["reason"] = f"could not reach the start point: {e}"
        return out
    if stop_evt is not None and stop_evt.is_set():
        out["reason"] = "stopped"
        return out

    resolved = PS.resolve_for(char, print_speed_mm_s=speed_mm_s, tuning=tuning)
    suspend = getattr(controller, "suspend_position_poller", None)
    resume = getattr(controller, "resume_position_poller", None)
    if callable(suspend):
        try:
            suspend()
        except Exception:
            pass
    try:
        samples, info = PS.follow_path(
            ideal_mm, PS.controller_io(controller), print_speed_mm_s=speed_mm_s,
            tuning=tuning, resolved=resolved, stop=stop_evt,
            max_wall_s=max(8.0, 3.0 * _nominal_wall_s(ideal_mm, resolved)))
        report = XC.path_report(
            samples, ideal_mm, commanded_speed_mm_s=speed_mm_s,
            resolution_um=resolution_um,
            corner_angle_deg=float(tuning.get("corner_angle", 30.0)),
            status="ok" if info["completed"] else info["stopped_reason"],
            wall_s=info["wall_s"])
        score = XC.composite_score(report, resolution_um=resolution_um)
        out.update({
            "ran": True, "pass": bool(score.get("pass")),
            "score": float(score.get("score", 0.0)),
            "fail_reasons": list(score.get("fail_reasons", [])),
            "p95_um": float(report.get("p95_um", 0.0)),
            "max_um": float(report.get("max_um", 0.0)),
            "rms_um": float(report.get("rms_um", 0.0)),
            "completion": float(report.get("completion_frac", 0.0)),
            "dither": float(report.get("dither_ratio", 0.0)),
            "wall_s": float(info.get("wall_s", 0.0)),
            "verdict": XC.verdict_for(report, resolution_um),
            "samples": samples, "ideal": ideal_mm,
        })
    except Exception as e:                                    # pragma: no cover
        logger.exception("hardware verify failed")
        out["reason"] = str(e)
    finally:
        try:
            controller.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        if callable(resume):
            try:
                resume()
            except Exception:
                pass
    return out


def _nominal_wall_s(ideal_mm, resolved) -> float:
    total = 0.0
    for i in range(1, len(ideal_mm)):
        total += math.hypot(ideal_mm[i][0] - ideal_mm[i - 1][0],
                            ideal_mm[i][1] - ideal_mm[i - 1][1])
    v = max(0.05, float(resolved.get("speed_cap_mm_s", 1.0) or 1.0))
    return total / v


# ── persistence ───────────────────────────────────────────────────────────

def persist(store, derived, best: GridCandidate | None,
            request: CalibrationRequest, *, speed_mm_s: float = 0.0,
            char=None) -> None:
    """Write the derived + tuned settings so EVERY print inherits them.

    ``AC.apply_to_store`` is the existing single writer for the tuning buckets, and
    ``AC.stamp_print_settings`` (called by both Quick Print and Full Print at build
    time) reads them straight back — so a print started after this returns picks the
    new tuning up with no restart.
    """
    if derived is None or not getattr(derived, "values", None):
        return
    if best is not None and char is not None and speed_mm_s > 0:
        derived.values.update(finalise_values(
            derived, best, speed_mm_s=speed_mm_s, char=char))
    elif best is not None:
        derived.values["lookahead_mm"] = float(best.lookahead_mm)
        derived.values["corner_speed_factor"] = float(best.corner_speed_factor)
    AC.apply_to_store(store, derived)
    try:
        store.set_resolution_element_um(float(request.resolution_um))
    except Exception:
        pass


# ── the run ───────────────────────────────────────────────────────────────

def _mk_steps() -> list:
    return [StepProgress(step=k, title=STEP_TITLES[k]) for k in STEP_ORDER]


def run_calibration(controller, *, settings, store, request: CalibrationRequest,
                    stop_evt=None, on_progress=None) -> CalibrationResult:
    """The whole calibration. Blocking; call from a worker thread."""
    res = CalibrationResult(steps=_mk_steps())
    t_run = time.monotonic()
    by_key = {s.step: s for s in res.steps}

    def _emit(step_key, state, msg="", frac=1.0, **detail):
        st = by_key[step_key]
        st.state = state
        st.frac = frac
        if msg:
            st.message = msg
        st.elapsed_s = time.monotonic() - t_run
        st.detail.update(detail)
        done = STEP_ORDER.index(step_key) + (1 if state in ("done", "skipped") else 0)
        st.remaining_s = max(0.0, sum(STEP_NOMINAL_S[k]
                                      for k in STEP_ORDER[done:]))
        if on_progress:
            try:
                on_progress(st)
            except Exception:
                pass

    def _aborted() -> bool:
        return bool(stop_evt is not None and stop_evt.is_set())

    def _finish_aborted(step_key):
        by_key[step_key].state = "aborted"
        for k in STEP_ORDER[STEP_ORDER.index(step_key) + 1:]:
            by_key[k].state = "skipped"
        res.aborted = True
        res.summary = "Stopped — nothing was saved."
        res.wall_s = time.monotonic() - t_run
        return res

    # A run that cannot retract has no business moving the stage at all.
    if request.safe_z_mm is None and getattr(controller, "is_zp_connected", False):
        res.error = ("No Safe Z is set, so the needle cannot be retracted before "
                     "travelling. Set Fast Move / Safe Z on the Calibration page "
                     "first.")
        res.summary = res.error
        return res

    ideal = _build_shape(controller, request)
    if ideal is None or len(ideal) < 3:
        res.error = "Could not build the test shape inside the travel envelope."
        res.summary = res.error
        return res

    try:
        if request.safe_z_mm is not None and hasattr(controller, "ensure_retracted_to"):
            controller.ensure_retracted_to(float(request.safe_z_mm))

        # ── 1. centre ──
        _emit("centre", "running", "travelling to the middle of the envelope")
        if _aborted():
            return _finish_aborted("centre")
        centred = AC.center_stage(controller, safe_z_mm=request.safe_z_mm)
        if not centred["ok"]:
            _emit("centre", "failed", centred["reason"])
            res.error = f"Could not centre the stage: {centred['reason']}"
            res.summary = res.error
            return res
        origin_um = centred["center_um"]
        _emit("centre", "done",
              f"centred at ({origin_um[0]:.0f}, {origin_um[1]:.0f}) µm")

        # ── 2. comms rate ──
        if _aborted():
            return _finish_aborted("comms")
        _emit("comms", "running", "measuring the closed-loop cadence while moving")
        r = controller.measure_control_loop_rate(
            iterations=int(request.comms_iterations), stop_evt=stop_evt)
        if "error" in r:
            _emit("comms", "failed", r["error"])
            res.error = f"Comms rate: {r['error']}"
            res.summary = res.error
            return res
        store.set_control_loop_ms(r["avg_period_ms"])
        _emit("comms", "done",
              f"{r['avg_period_ms']:.1f} ms ({r['control_hz']:.1f} Hz)")

        # ── 3. dead time ──
        if _aborted():
            return _finish_aborted("deadtime")
        _emit("deadtime", "running", "stepping the velocity command")
        dt = DT.measure_velocity_dead_time(
            controller, axis="diag", repeats=int(request.deadtime_repeats),
            center_first=False, safe_z_mm=request.safe_z_mm, stop_evt=stop_evt,
            on_progress=lambda m: _emit("deadtime", "running", m, frac=0.5))
        if "error" in dt:
            _emit("deadtime", "failed", dt["error"])
            res.error = f"Dead time: {dt['error']}"
            res.summary = res.error
            return res
        lag = max(dt["apparent_lag_s"], dt["dead_time_s"])
        store.set_velocity_dead_time_s(
            lag, n=dt["n"], spread_s=dt.get("apparent_lag_spread_s", 0.0),
            tau_s=dt.get("tau_s"))
        _emit("deadtime", "done",
              f"{lag * 1000:.0f} ms (τ {dt.get('tau_s', 0.0) * 1000:.0f} ms)")

        # ── 4. top speed ──
        if _aborted():
            return _finish_aborted("topspeed")
        _emit("topspeed", "running", "sweeping distances at full speed")
        ts = TS.measure_top_speed(
            controller, max_dist_mm=request.top_speed_max_dist_mm,
            n_points=int(request.top_speed_points),
            repeats=int(request.top_speed_repeats),
            center_first=False, origin_um=origin_um,
            safe_z_mm=request.safe_z_mm, stop_evt=stop_evt,
            on_progress=lambda m: _emit("topspeed", "running", m, frac=0.5))
        if "error" in ts:
            _emit("topspeed", "failed", ts["error"])
            res.error = f"Top speed: {ts['error']}"
            res.summary = res.error
            return res
        res.top_speed_um_s = float(ts["top_speed_um_s"])
        res.top_speed_fit = {k: ts[k] for k in
                             ("slope_s_per_mm", "intercept_s", "r2", "n")}
        _emit("topspeed", "done",
              f"{res.top_speed_um_s / 1000.0:.2f} mm/s (R²={ts['r2']:.3f})")

        # ── 5. commit (this is the line that fixes the operator's loop) ──
        _emit("commit", "running", "making the measurement authoritative")
        res.applied = commit_top_speed(controller, settings, store,
                                       res.top_speed_um_s, fit=res.top_speed_fit)
        if not res.applied.get("ok"):
            _emit("commit", "failed", res.applied.get("reason", ""))
            res.error = res.applied.get("reason", "could not apply the top speed")
            res.summary = res.error
            return res
        _emit("commit", "done", "applied to the hardware config and the store")

        # ── 6. derive ──
        _emit("derive", "running", "closed-form derivation")
        measured = AC.measured_from_store(store)
        if measured.cruise_um_s <= 0 and dt.get("cruise_um_s"):
            measured.cruise_um_s = dt["cruise_um_s"]
        if not measured.is_complete():
            _emit("derive", "failed", "incomplete measurements")
            res.error = "Still missing: " + ", ".join(measured.missing())
            res.summary = res.error
            return res
        policy = AC.CalibrationPolicy(
            target_speed_mm_s=max(0.05, float(request.target_speed_mm_s)),
            resolution_um=float(request.resolution_um))
        derived = AC.derive_settings(measured, policy)
        if not derived.values:
            msg = "; ".join(w.message for w in derived.warnings)
            _emit("derive", "failed", msg)
            res.error = msg or "derivation failed"
            res.summary = res.error
            return res
        res.measured, res.derived = measured, derived
        _emit("derive", "done",
              f"lookahead {derived.values['lookahead_mm']:.3f} mm · "
              f"{derived.values['control_hz']:.0f} Hz")

        # ── 7. grid (simulation only) ──
        if _aborted():
            return _finish_aborted("grid")
        n = request.grid_points()
        _emit("grid", "running", f"scoring {n}×{n} candidates in simulation")
        char = StageCharacteristics.from_store(store)
        grid = score_grid(ideal, char, derived=derived, request=request,
                          stop_evt=stop_evt)
        res.grid = grid
        if grid.best is None:
            _emit("grid", "failed", "no candidates scored")
            res.error = "The grid produced no candidates."
            res.summary = res.error
            return res
        _emit("grid", "done",
              f"best p95 {grid.best.p95_um:.0f} µm at lookahead "
              f"{grid.best.lookahead_mm:.3f} mm · corner {grid.best.corner_speed_factor:.2f} "
              f"({grid.n_pass}/{grid.n} passed, {grid.sim_s:.1f} s)")
        res.feed_plan = _feed_plan_compare(ideal, char, grid.speed_mm_s,
                                           request.resolution_um)

        # ── 8. verify on the stage ──
        spent = time.monotonic() - t_run
        budget_left = float(request.time_budget_s) - spent
        if not request.verify_on_hardware:
            _emit("verify", "skipped", "verification turned off")
        elif budget_left < STEP_NOMINAL_S["verify"] + STEP_NOMINAL_S["persist"]:
            _emit("verify", "skipped",
                  f"skipped to stay inside the {request.time_budget_s:.0f} s budget")
        elif _aborted():
            return _finish_aborted("verify")
        else:
            _emit("verify", "running", "driving the winning tuning on the stage")
            # The stage must be driven with exactly the tuning that was
            # simulated, so the candidate's neutralised levers are used here too.
            vals = candidate_values(dict(derived.values),
                                    lookahead_mm=grid.best.lookahead_mm,
                                    corner_factor=grid.best.corner_speed_factor)
            res.verify = verify_on_hardware(
                controller, ideal, tuning=tuning_from_store_keys(vals),
                speed_mm_s=grid.speed_mm_s,
                resolution_um=request.resolution_um, char=char,
                safe_z_mm=request.safe_z_mm, stop_evt=stop_evt)
            if res.verify.get("ran"):
                _emit("verify", "done",
                      f"p95 {res.verify['p95_um']:.0f} µm · "
                      f"{res.verify['verdict']}")
            else:
                _emit("verify", "failed", res.verify.get("reason", ""))

        # ── 9. persist ──
        _emit("persist", "running", "saving the tuning")
        persist(store, derived, grid.best, request,
                speed_mm_s=grid.speed_mm_s, char=char)
        _emit("persist", "done", "every print inherits these settings")

        res.ok = True
        res.summary = _summarise(res, request)
        return res
    except Exception as e:                                    # pragma: no cover
        logger.exception("XY calibration error")
        res.error = str(e)
        res.summary = f"Error: {e}"
        return res
    finally:
        res.wall_s = time.monotonic() - t_run
        try:
            controller.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        try:
            if request.safe_z_mm is not None and hasattr(
                    controller, "ensure_retracted_to"):
                controller.ensure_retracted_to(float(request.safe_z_mm))
        except Exception:
            pass


def _build_shape(controller, request: CalibrationRequest):
    """The scoring shape, in ZERO-REF mm about the envelope centre (the frame
    ``PS.controller_io`` reads), shrunk to fit the travel envelope."""
    size = float(request.feature_mm)
    try:
        size = AC.fit_shape_size_mm(controller, size)
    except Exception:
        pass
    if size <= 0:
        return None
    pts = XC.make_shape(request.shape, size, max(0.02, float(request.step_mm)))
    try:
        cx_um, cy_um = controller.default_plate_center_um()
        z = getattr(controller, "zero_position", {}) or {}
        cx = (cx_um - z.get("x", 0.0)) / 1000.0
        cy = (cy_um - z.get("y", 0.0)) / 1000.0
    except Exception:
        cx = cy = 0.0
    return XC.offset_path(pts, cx, cy)


def _feed_plan_compare(ideal, char, speed_mm_s, resolution_um) -> dict:
    """The honest counterpart to the grid winner.

    The single-tuning follower must cross every corner with ONE lookahead; the feed
    plan stops on each sharp vertex and continues as a fresh section, so on a small
    feature it scores far better. Reporting only the grid winner would read as "the
    calibration failed" when the machine can in fact print the shape — via the path
    this calibration's top-speed write is what enables.
    """
    try:
        from SupportClasses import XYFeedPlan as FP
        plan = FP.build_plan(ideal, char, target_speed_mm_s=speed_mm_s,
                             element_um=resolution_um)
        sim = FP.simulate_plan(plan, char, resolution_um=resolution_um)
        rep = getattr(sim, "report", {}) or {}
        return {"ok": True, "sections": len(getattr(plan, "sections", []) or []),
                "p95_um": float(rep.get("p95_um", 0.0)),
                "max_um": float(rep.get("max_um", 0.0)),
                "verdict": getattr(sim, "verdict", "")}
    except Exception as e:
        logger.debug("feed-plan comparison failed: %s", e)
        return {"ok": False, "reason": str(e)}


def _summarise(res: CalibrationResult, request: CalibrationRequest) -> str:
    g = res.grid
    bits = [f"Top speed {res.top_speed_um_s / 1000.0:.2f} mm/s (applied)."]
    if g and g.best:
        bits.append(
            f"Tuned: lookahead {g.best.lookahead_mm:.3f} mm, corner "
            f"{g.best.corner_speed_factor:.2f} — simulated p95 "
            f"{g.best.p95_um:.0f} µm on a {request.feature_mm:g} mm "
            f"{request.shape.lower()}.")
        if g.resolved_cap_mm_s:
            bits.append(
                f"Holds {request.resolution_um:.0f} µm at "
                f"{g.resolved_cap_mm_s:.2f} mm/s ({g.cap_reason}-limited).")
        if g.all_failed:
            bits.append(
                f"⚠ No candidate held {request.resolution_um:.0f} µm — the best is "
                f"reported. Use the feed-plan path for features this small.")
        if g.corner_axis_inert:
            bits.append(
                "⚠ Corner speed had no effect (the dead-time cap binds below every "
                "corner limit), so its value is not meaningful here.")
    if res.verify.get("ran"):
        bits.append(f"Stage check: p95 {res.verify['p95_um']:.0f} µm, "
                    f"{res.verify['verdict']}.")
    elif res.verify.get("reason"):
        bits.append(f"Stage check did not run: {res.verify['reason']}.")
    fp = res.feed_plan
    if fp.get("ok"):
        bits.append(f"Feed-plan path (stops at corners): p95 "
                    f"{fp['p95_um']:.0f} µm, {fp['verdict']}.")
    bits.append(f"Done in {res.wall_s:.0f} s.")
    return " ".join(bits)
