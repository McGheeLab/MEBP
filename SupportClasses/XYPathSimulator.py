"""XYPathSimulator.py — predict how the TUNED stage will actually follow a path.

Built on two pieces:

  • :class:`~SupportClasses.XYStageModel.XYStageModel` — the machine's SAVED
    dynamics (dead time, rise time τ, top speed, loop period, readout quantum),
    persisted by the calibration flow so simulations need no hardware.
  • **One shared follower loop**, :func:`follow_path` — an exact transcription
    of the XY-Challenge bench's velocity driver (itself the mirror of
    ``PrintManager._execute_print_path_velocity``), over injected I/O bindings.
    The simulation binds it to the model with a virtual clock; a hardware run
    binds it to the real controller with the wall clock. Same code, same
    ``resolve_control``/``plan_speed_limits``/``pursuit_step`` calls, same
    ``max_ds``/decel/arrive/runaway semantics — so "the simulator says this
    print works" and "the stage does this" cannot drift apart in the loop
    logic, only in the fidelity of the stage model (which is validated against
    real traces on ME3B V1).

What it is for (operator-facing):

  • **Geometry panel** — simulate (or run) a matrix of shapes × sizes and see
    ideal vs actual side by side: where geometry itself (sharp corners, fine
    combs, small loops) defeats the current tuning.
  • **Sketch printability** — split a compiled print trajectory into its
    printing sub-paths and simulate each with the saved characteristics +
    current tuning, flagging the regions whose predicted deviation exceeds the
    resolution element ("this corner will overshoot ~180 µm").
  • **Tuning what-ifs** — apply a different tuning dict to the same model and
    compare predicted reports before committing anything to the stage.

Everything here is pure Python (math + the pure XYChallenge/VelocityControl
modules); no Qt, no serial, no numpy.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from SupportClasses import VelocityControl as VC
from SupportClasses import XYChallenge as XC
from SupportClasses.XYStageModel import StageCharacteristics, XYStageModel


# Mirrors of the bench driver's class constants (xy_challenge_dialog).
VEL_ARRIVE_MM = 0.05
VEL_RUNAWAY_MM = 3.0
VEL_RUNAWAY_TICKS = 8


# ── Tuning: one dict, sourced from the shared store ───────────────────

#: Bench defaults, used when a store key is absent/0 — same numbers as the
#: dialog's PARAM_SPECS so a simulation with no explicit tuning behaves like
#: the bench with untouched spins.
TUNING_DEFAULTS = {
    "lookahead": 0.2, "decel": 1.5, "kp": 0.0, "kd": 0.0,
    "corner_angle": 30.0, "corner_factor": 0.4, "control_hz": 25.0,
    "hold_speed": 0.0, "max_speed_frac": 0.0, "min_lookahead_frac": 0.0,
    "lead_time_frac": 0.0, "deadtime_safety": 2.0,
}

_STORE_KEY_MAP = {
    "lookahead": "lookahead_mm", "decel": "decel_mm", "kp": "pid_kp",
    "kd": "pid_kd", "corner_angle": "corner_angle_deg",
    "corner_factor": "corner_speed_factor", "control_hz": "control_hz",
    "hold_speed": "hold_speed", "max_speed_frac": "max_speed_frac",
    "min_lookahead_frac": "min_lookahead_frac",
    "lead_time_frac": "lead_time_frac", "deadtime_safety": "deadtime_safety",
}


def tuning_from_store(store) -> dict:
    """The machine's CURRENT velocity tuning as a simulator tuning dict —
    store zeros fall back to the bench defaults, exactly as the dialog's
    spins would show them."""
    try:
        v = store.get_mode_params("velocity") or {}
    except Exception:
        v = {}
    out = {}
    for key, dflt in TUNING_DEFAULTS.items():
        try:
            raw = float(v.get(_STORE_KEY_MAP[key], 0.0) or 0.0)
        except (TypeError, ValueError):
            raw = 0.0
        # hold_speed / the frac levers are meaningfully zero; the geometry
        # params fall back to the bench defaults when unset.
        if key in ("hold_speed", "max_speed_frac", "min_lookahead_frac",
                   "lead_time_frac"):
            out[key] = raw
        else:
            out[key] = raw if raw > 0 else dflt
    return out


def resolve_for(char: StageCharacteristics, *, print_speed_mm_s, tuning,
                fallback_max_um_s=50000.0) -> dict:
    """``resolve_control`` grounded in the SAVED characteristics — the same
    call, same arguments, as the bench's ``_resolve`` / the print path."""
    p = dict(TUNING_DEFAULTS)
    p.update(tuning or {})
    return VC.resolve_control(
        print_speed_mm_s=print_speed_mm_s,
        lookahead_mm=max(0.05, float(p["lookahead"])),
        xy_max_speed_um_s=char.top_speed_um_s,
        control_loop_ms=char.control_loop_ms,
        phase_lag_s=0.0,
        default_control_hz=max(5.0, float(p["control_hz"])),
        fallback_max_um_s=fallback_max_um_s,
        dead_time_s=char.dead_time_s,
        lead_time_frac=float(p["lead_time_frac"]),
        min_lookahead_frac=float(p["min_lookahead_frac"]),
        max_speed_frac=float(p["max_speed_frac"]),
        hold_speed=bool(p["hold_speed"]),
        safety=float(p["deadtime_safety"]) or 2.0)


# ── The ONE follower loop, over injected I/O ──────────────────────────

@dataclass
class FollowIO:
    """The four things the follower loop touches. Bind them to the stage model
    (virtual clock) or to the real controller (wall clock)."""
    read_mm: object          # () -> (x, y) mm | None
    send_um_s: object        # (vx_um_s, vy_um_s) -> None
    clock: object            # () -> seconds (monotonic)
    sleep: object            # (dt_s) -> None


def model_io(model: XYStageModel) -> FollowIO:
    """Bindings for a simulated run: the 'clock' is the model's own time and
    'sleep' integrates the model, so a simulation is deterministic and runs at
    machine speed."""
    return FollowIO(
        read_mm=lambda: tuple(c / 1000.0 for c in model.read_position()),
        send_um_s=model.command_velocity,
        clock=lambda: model.t,
        sleep=model.advance)


def controller_io(controller) -> FollowIO:
    """Bindings for a REAL hardware run (zero-ref mm, like the bench)."""
    def _read():
        try:
            p = controller.get_xy_position(cached=False)
        except Exception:
            return None
        if not p or p[0] is None or p[1] is None:
            return None
        z = getattr(controller, "zero_position", {})
        return ((p[0] - z.get("x", 0)) / 1000.0,
                (p[1] - z.get("y", 0)) / 1000.0)
    return FollowIO(read_mm=_read, send_um_s=controller.send_velocity_xy,
                    clock=time.monotonic, sleep=time.sleep)


def follow_path(ideal_mm, io: FollowIO, *, print_speed_mm_s, tuning,
                resolved, stop=None, max_wall_s=None, on_tick=None,
                arrive_mm=VEL_ARRIVE_MM, end_lag_s=0.0, end_floor_frac=0.1,
                lookahead_at=None, speed_limit_at=None):
    """Drive ``io`` along ``ideal_mm`` with the shared velocity-following law.

    An exact transcription of the bench's ``_drive_velocity`` tick (which
    mirrors the print path): per-tick ``max_ds`` window, end-of-path decel
    taper from the capped speed, cross-track PID via ``pursuit_step``, VS
    magnitude clamp, 8-tick runaway guard, arc-length + endpoint arrival test,
    wall-time cap. ALWAYS sends ``(0, 0)`` on every exit.

    Returns ``(samples, info)`` — timestamped :class:`XYChallenge.Sample` list
    plus ``{"stopped_reason", "completed", "wall_s", "cross_profile"}`` where
    ``cross_profile`` is ``[(s_mm, signed_cross_um), …]`` per tick (the basis
    of the printability fail-region map).
    """
    p = dict(TUNING_DEFAULTS)
    p.update(tuning or {})
    speed = max(0.05, float(print_speed_mm_s))
    decel = max(0.1, float(p["decel"]))
    kp = max(0.0, float(p["kp"]))
    kd = max(0.0, float(p["kd"]))
    cum = VC.polyline_arclength(ideal_mm)
    total = cum[-1]
    speed_cap = resolved["speed_cap_mm_s"]
    lookahead = resolved.get("lookahead_mm", p["lookahead"])
    max_um = resolved["max_um_s"]
    dt = 1.0 / resolved["control_hz"]
    # v7.7: a caller may supply arc-length PROFILES instead of the fixed
    # lookahead + the corner-angle speed rule — that is how a corner gets taken
    # by slowing (with a correspondingly smaller carrot) rather than stopping.
    # Both default to None, in which case this is byte-identical to before.
    if speed_limit_at is None:
        speed_limit_at, _corners = VC.plan_speed_limits(
            ideal_mm, cum, speed, corner_angle_deg=float(p["corner_angle"]),
            corner_speed_factor=float(p["corner_factor"]), decel_mm=decel)

    state = VC.PursuitState()
    samples = []
    cross_profile = []
    reason = "wall_cap"
    runaway = 0
    prev_pos = None
    v_meas_mm_s = 0.0     # MEASURED speed (successive reads) for the lag taper
                          # — during braking the actual speed exceeds the
                          # commanded one (τ + pipeline), so the commanded
                          # value under-predicts the flight distance
    if max_wall_s is None:
        max_wall_s = total / speed * 6.0 + 15.0
    t0 = io.clock()
    last_t = t0
    try:
        while True:
            if stop is not None and stop.is_set():
                reason = "stopped"
                break
            tick = io.clock()
            if tick - t0 > max_wall_s:
                reason = "wall_cap"
                break
            pos = io.read_mm()
            if pos is None:
                io.sleep(dt)
                continue
            samples.append(XC.Sample(pos[0], pos[1], tick - t0))
            dt_real = max(1e-3, tick - last_t)
            last_t = tick
            if prev_pos is not None:
                v_meas_mm_s = math.hypot(pos[0] - prev_pos[0],
                                         pos[1] - prev_pos[1]) / dt_real
            prev_pos = pos
            max_ds = min(1.5, max(0.15, speed * dt_real * 6.0))
            remaining = total - state.s
            cap = speed_cap
            if remaining < decel:
                # Lag-aware taper (opt-in, end_lag_s > 0): taper on where the
                # stage WILL be when this command takes effect. The naive
                # `remaining/decel` taper commands ~full taper speed at the
                # position the stage occupied one dead-time ago, so it crosses
                # the endpoint fast and overshoots by ~v·lag (measured 153 µm
                # at 3 mm/s on the ME3B model) before creeping back.
                rem_eff = remaining
                if end_lag_s > 0.0 and v_meas_mm_s > 0.0:
                    rem_eff = max(0.0, remaining - v_meas_mm_s * end_lag_s)
                cap = min(cap, max(end_floor_frac, rem_eff / decel)
                          * min(speed, speed_cap))
            vx, vy, sN, cross = VC.pursuit_step(
                pos, ideal_mm, cum, state, lookahead=lookahead,
                speed_cap_mm_s=cap, speed_limit_at=speed_limit_at,
                dt=dt_real, max_ds=max_ds, kp=kp, kd=kd,
                lookahead_at=lookahead_at)
            # state.prev_cross is the SIGNED cross-track at s, just computed.
            cross_profile.append((sN, state.prev_cross * 1000.0))
            if on_tick is not None:
                on_tick(sN, total, pos)
            vmag = math.hypot(vx, vy)
            if vmag > max_um and vmag > 0:
                vx *= max_um / vmag
                vy *= max_um / vmag
            if cross > VEL_RUNAWAY_MM:
                runaway += 1
                if runaway >= VEL_RUNAWAY_TICKS:
                    reason = "runaway"
                    break
            else:
                runaway = 0
            dist_end = math.hypot(pos[0] - ideal_mm[-1][0],
                                  pos[1] - ideal_mm[-1][1])
            if sN >= total - 1e-6 and dist_end <= arrive_mm:
                reason = "arrived"
                break
            try:
                io.send_um_s(vx, vy)
            except Exception:
                reason = "send_error"
                break
            el = io.clock() - tick
            if el < dt:
                io.sleep(dt - el)
    finally:
        try:
            io.send_um_s(0.0, 0.0)
        except Exception:
            pass
    return samples, {"stopped_reason": reason,
                     "completed": reason == "arrived",
                     "wall_s": io.clock() - t0,
                     "cross_profile": cross_profile}


# ── Simulation entry points ───────────────────────────────────────────

@dataclass
class SimResult:
    """One simulated (or driven) run, scored."""
    samples: list = field(default_factory=list)
    report: dict = field(default_factory=dict)
    score: dict = field(default_factory=dict)
    verdict: str = "fail"
    resolved: dict = field(default_factory=dict)
    cross_profile: list = field(default_factory=list)
    fail_regions: list = field(default_factory=list)
    completed: bool = False
    stopped_reason: str = ""
    wall_s: float = 0.0


def fail_regions_from_profile(ideal_mm, cross_profile, *, resolution_um=30.0):
    """Contiguous stretches of the path whose |cross-track| exceeded the
    resolution element, located in world coordinates.

    Severity mirrors :func:`XYChallenge.verdict_for`'s max-deviation bounds:
    ``"fail"`` above 2× the element (would fail the run), ``"warn"`` above 1×.
    Returns ``[{s0_mm, s1_mm, peak_um, x_mm, y_mm, severity}, …]``.
    """
    res = max(1e-6, float(resolution_um))
    cum = VC.polyline_arclength(ideal_mm)
    regions = []
    cur = None
    for s, c_um in cross_profile:
        mag = abs(c_um)
        if mag > res:
            if cur is None:
                cur = {"s0_mm": s, "s1_mm": s, "peak_um": mag, "peak_s": s}
            else:
                cur["s1_mm"] = s
                if mag > cur["peak_um"]:
                    cur["peak_um"] = mag
                    cur["peak_s"] = s
        elif cur is not None:
            regions.append(cur)
            cur = None
    if cur is not None:
        regions.append(cur)
    out = []
    for r in regions:
        x, y = VC.point_at_arclength(ideal_mm, cum, r["peak_s"])
        out.append({"s0_mm": r["s0_mm"], "s1_mm": r["s1_mm"],
                    "peak_um": r["peak_um"], "x_mm": x, "y_mm": y,
                    "severity": "fail" if r["peak_um"] > 2.0 * res
                    else "warn"})
    return out


def simulate_follow(ideal_mm, *, char: StageCharacteristics,
                    print_speed_mm_s, tuning=None, resolution_um=30.0,
                    start_offset_mm=(0.0, 0.0), max_wall_s=None) -> SimResult:
    """Simulate the tuned follower over ``ideal_mm`` with the machine's saved
    characteristics, and score it with the same honest metrics the bench uses.

    ``start_offset_mm`` displaces the starting position from ``ideal[0]`` to
    model an imperfect approach (e.g. the measured point-to-point positioning
    error)."""
    tuning = dict(TUNING_DEFAULTS, **(tuning or {}))
    resolved = resolve_for(char, print_speed_mm_s=print_speed_mm_s,
                           tuning=tuning)
    model = XYStageModel(
        char,
        x_um=(ideal_mm[0][0] + start_offset_mm[0]) * 1000.0,
        y_um=(ideal_mm[0][1] + start_offset_mm[1]) * 1000.0)
    samples, info = follow_path(
        ideal_mm, model_io(model), print_speed_mm_s=print_speed_mm_s,
        tuning=tuning, resolved=resolved, max_wall_s=max_wall_s)
    report = XC.path_report(samples, ideal_mm,
                            commanded_speed_mm_s=print_speed_mm_s,
                            resolution_um=resolution_um,
                            corner_angle_deg=float(tuning["corner_angle"]),
                            status="ok" if info["completed"]
                            else info["stopped_reason"],
                            wall_s=info["wall_s"])
    score = XC.composite_score(report, resolution_um=resolution_um)
    return SimResult(
        samples=samples, report=report, score=score,
        verdict=XC.verdict_for(report, resolution_um),
        resolved=resolved, cross_profile=info["cross_profile"],
        fail_regions=fail_regions_from_profile(
            ideal_mm, info["cross_profile"], resolution_um=resolution_um),
        completed=info["completed"], stopped_reason=info["stopped_reason"],
        wall_s=info["wall_s"])


# ── Geometry panel ────────────────────────────────────────────────────

DEFAULT_PANEL_SHAPES = ("Square", "Circle", "Star", "Zigzag",
                        "Line-Reversal", "Comb")
DEFAULT_PANEL_SIZES = (2.0, 5.0, 10.0)


def simulate_panel(*, char: StageCharacteristics, print_speed_mm_s,
                   tuning=None, shapes=None, sizes=None, step_mm=0.5,
                   resolution_um=30.0, on_cell=None) -> list:
    """Simulate a shape × size matrix. Returns one dict per cell:
    ``{shape, size_mm, ideal, result: SimResult}`` (row-major, shapes outer)."""
    cells = []
    for name in (shapes or DEFAULT_PANEL_SHAPES):
        for size in (sizes or DEFAULT_PANEL_SIZES):
            ideal = XC.make_shape(name, float(size), step_mm)
            r = simulate_follow(ideal, char=char,
                                print_speed_mm_s=print_speed_mm_s,
                                tuning=tuning, resolution_um=resolution_um)
            cell = {"shape": name, "size_mm": float(size), "ideal": ideal,
                    "result": r}
            cells.append(cell)
            if on_cell is not None:
                on_cell(cell)
    return cells


# ── Sketch / trajectory printability ──────────────────────────────────

def printing_subpaths_from_trajectory(traj, *, min_points=2,
                                      min_length_mm=1e-3) -> list:
    """Split an Nx7 trajectory ``[x, y, z, p1, p2, p3, t]`` into its PRINTING
    XY sub-paths — the pieces the velocity follower would actually drive.

    A segment is a *travel* (breaks the sub-path) when the pumps do not advance
    while XY moves — the same rule as Quick Print's ``_travel_mask``. Segments
    with no XY motion (pure Z lifts/lowers, pump-only moves) neither extend nor
    break a sub-path.
    """
    rows = [tuple(float(v) for v in r[:6]) for r in (traj or [])]
    subs = []
    cur = []

    def _flush():
        nonlocal cur
        if len(cur) >= min_points:
            length = sum(math.hypot(cur[i][0] - cur[i - 1][0],
                                    cur[i][1] - cur[i - 1][1])
                         for i in range(1, len(cur)))
            if length >= min_length_mm:
                subs.append(cur)
        cur = []

    for i in range(1, len(rows)):
        x0, y0 = rows[i - 1][0], rows[i - 1][1]
        x1, y1 = rows[i][0], rows[i][1]
        seg = math.hypot(x1 - x0, y1 - y0)
        dp = sum(abs(rows[i][j] - rows[i - 1][j]) for j in (3, 4, 5))
        if seg <= 1e-6:
            continue                      # Z-only / pump-only: no XY motion
        if dp <= 1e-9:
            _flush()                      # travel: pen up between sub-paths
            continue
        if not cur:
            cur = [(x0, y0)]
        cur.append((x1, y1))
    _flush()
    return subs


def check_toolpath(subpaths_mm, *, char: StageCharacteristics,
                   print_speed_mm_s, tuning=None, resolution_um=30.0,
                   on_path=None) -> dict:
    """Simulate every printing sub-path and aggregate a printability verdict.

    Returns ``{"paths": [{index, length_mm, n_pts, result}, …], "verdict",
    "worst_p95_um", "worst_max_um", "fail_regions", "all_completed"}`` where
    ``fail_regions`` carry world coordinates so a canvas can mark them.
    """
    order = {"pass": 0, "marginal": 1, "fail": 2}
    paths = []
    regions = []
    worst = "pass"
    worst_p95 = 0.0
    worst_max = 0.0
    all_completed = True
    for idx, pts in enumerate(subpaths_mm or []):
        cum = VC.polyline_arclength(pts)
        r = simulate_follow(pts, char=char,
                            print_speed_mm_s=print_speed_mm_s,
                            tuning=tuning, resolution_um=resolution_um)
        entry = {"index": idx, "length_mm": cum[-1], "n_pts": len(pts),
                 "ideal": list(pts), "result": r}
        paths.append(entry)
        for reg in r.fail_regions:
            regions.append(dict(reg, path_index=idx))
        if order.get(r.verdict, 2) > order.get(worst, 0):
            worst = r.verdict
        worst_p95 = max(worst_p95, float(r.report.get("p95_um") or 0.0))
        worst_max = max(worst_max, float(r.report.get("max_um") or 0.0))
        all_completed = all_completed and r.completed
        if on_path is not None:
            on_path(entry)
    if not paths:
        worst = "fail"
    if not all_completed:
        worst = "fail"
    return {"paths": paths, "verdict": worst, "worst_p95_um": worst_p95,
            "worst_max_um": worst_max, "fail_regions": regions,
            "all_completed": all_completed}


def check_trajectory(traj, *, char: StageCharacteristics, print_speed_mm_s,
                     tuning=None, resolution_um=30.0, on_path=None) -> dict:
    """Convenience: split an Nx7 trajectory and :func:`check_toolpath` it."""
    return check_toolpath(
        printing_subpaths_from_trajectory(traj), char=char,
        print_speed_mm_s=print_speed_mm_s, tuning=tuning,
        resolution_um=resolution_um, on_path=on_path)
