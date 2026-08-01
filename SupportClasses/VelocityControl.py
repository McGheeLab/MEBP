"""VelocityControl.py — shared, PURE control law for velocity-following printing.

The XY-Challenge bench and the real print (PrintManager) both drive the stage with
the SAME arc-length pure-pursuit law so that whatever you tune on the bench is
exactly what prints. Everything here is pure Python (``math`` only) — no Qt, no
serial, no numpy — so it is cheap in the real-time loop and trivially testable.

Pieces:
  • Arc-length polyline helpers (canonical home; PrintManager re-exports them).
  • ``corner_flags`` / ``plan_speed_limits`` — corner detection + a corner-aware
    speed-limit profile (slow into sharp turns, ramp back out) as a function of
    arc length.
  • ``PursuitState`` + ``pursuit_step`` — one control tick: project the measured
    position onto the path, place a carrot a lookahead ahead of real progress,
    steer toward it, and ADD a cross-track PID correction (perpendicular pull
    back onto the path). Magnitude clamped to the corner-aware speed limit.
  • ``resolve_control`` — grounds the loop rate + speed cap in the machine's
    MEASURED calibration (control-loop period, true max speed, phase lag), which
    is what removes the dead-time overshoot limit-cycle and lets it run as fast
    as comms allow.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


#: Does ``pursuit_step`` bound the cross-track correction relative to the
#: commanded speed (tangential/normal decomposition + forward floor)?
#:
#: FALSE today: the PD correction is still ADDED to a full-magnitude pursuit
#: vector with no re-clamp, so a large ``kp`` produces a mostly-PERPENDICULAR
#: command — at the logged 0.64 mm cross-track error with kp≈4.9 the command was
#: 76 % sideways, which is what deadlocked the follower.
#:
#: Consumers use this to decide whether a non-zero cross-track gain is safe to
#: apply: ``XYAutoCalibration.derive_settings`` deliberately derives ``pid_kp = 0``
#: (pure pursuit) while this is False, so an automated calibration cannot hand the
#: machine a gain the control law will mishandle. Flip it to True in the same
#: change that lands the bounded composition.
SUPPORTS_BOUNDED_CROSS_TRACK = False


# ── Arc-length polyline helpers (canonical home) ──────────────────────

def polyline_arclength(pts):
    """Cumulative arc length (same units as pts) at each vertex; cum[0]=0."""
    cum = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    return cum


def point_at_arclength(pts, cum, s):
    """(x, y) at arc length ``s`` along the polyline (clamped to the ends)."""
    total = cum[-1]
    if s <= 0.0:
        return (pts[0][0], pts[0][1])
    if s >= total:
        return (pts[-1][0], pts[-1][1])
    lo, hi = 0, len(cum) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if cum[mid] < s:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    seg = cum[i] - cum[i - 1]
    t = (s - cum[i - 1]) / seg if seg > 1e-12 else 0.0
    return (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
            pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))


def tangent_at_arclength(pts, cum, s):
    """Unit tangent (dx, dy) of the polyline at arc length ``s`` — the local
    direction of travel. Clamped to the end segments."""
    total = cum[-1]
    n = len(pts)
    if n < 2:
        return (1.0, 0.0)
    if s <= 0.0:
        i = 1
    elif s >= total:
        i = n - 1
    else:
        lo, hi = 0, n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
    dx = pts[i][0] - pts[i - 1][0]
    dy = pts[i][1] - pts[i - 1][1]
    d = math.hypot(dx, dy)
    if d <= 1e-12:
        return (1.0, 0.0)
    return (dx / d, dy / d)


#: The projection MUST always be able to examine at least this many segments
#: forward of the cursor, whatever ``max_ahead`` says.
#:
#: Without this the follower deadlocks whenever the toolpath's segments are longer
#: than the window. The window is ``max(0.15, speed·dt·6)``, so at 1 mm/s and
#: 25 Hz it is 0.239 mm — smaller than the 0.486 mm mean segment of a real sketch.
#: The scan then breaks after examining only the CURRENT segment, so ``s`` pins at
#: that segment's end, the carrot stops advancing, and the stage sits there while
#: the cross-track term thrashes it about. That is precisely the logged failure in
#: ``logs/prints/print_20260727_195339_*.jsonl``: arc-length progress froze at
#: 0.509 mm — one 0.4859 mm segment — while 12.4 mm of travel produced 1.0 mm of
#: net motion.
#:
#: Two segments is far less than one revolution of a spiral, so the loop-snap
#: protection the window exists for is unaffected; and the per-tick committed
#: advance is still bounded by ``max_ds`` in ``pursuit_step``, so pump deposition
#: and snap bounds are unchanged.
MIN_SEGMENTS_EXAMINED = 2


def project_on_polyline(pos, pts, cum, seg_start, max_ahead):
    """Project ``pos`` onto the polyline, searching segments forward from
    ``seg_start`` while their far end stays within ``max_ahead`` arc length of
    ``cum[seg_start]`` (bounds the search + prevents snapping to a later loop of
    a self-intersecting path). Returns ``(s, seg_idx, cross_dist)``.

    Always examines at least :data:`MIN_SEGMENTS_EXAMINED` segments so a window
    narrower than the local segment length cannot stall progress — see that
    constant for the failure it prevents.
    """
    n = len(pts)
    best_s = cum[min(seg_start, n - 1)]
    best_i = min(seg_start, n - 2) if n >= 2 else 0
    best_d = float("inf")
    limit = cum[min(seg_start, n - 1)] + max_ahead
    i = max(0, min(seg_start, n - 2))
    examined = 0
    while i < n - 1:
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 > 1e-12:
            t = ((pos[0] - ax) * dx + (pos[1] - ay) * dy) / seg2
            t = max(0.0, min(1.0, t))
        else:
            t = 0.0
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(pos[0] - px, pos[1] - py)
        if d < best_d:
            best_d = d
            best_i = i
            best_s = cum[i] + t * math.sqrt(seg2)
        examined += 1
        if examined >= MIN_SEGMENTS_EXAMINED and cum[i + 1] > limit:
            break
        i += 1
    return best_s, best_i, best_d


# ── Corner detection + corner-aware speed-limit profile ───────────────

def corner_flags(pts, angle_deg):
    """Boolean list (length N): True at a vertex whose turn angle between the
    incoming and outgoing segment exceeds ``angle_deg`` (0° = straight through).
    Pure mirror of TrajectoryPlanner.detect_corners, without the numpy dep."""
    n = len(pts)
    flags = [False] * n
    if n < 3:
        return flags
    for i in range(1, n - 1):
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        cx, cy = pts[i + 1]
        v1x, v1y = bx - ax, by - ay
        v2x, v2y = cx - bx, cy - by
        l1 = math.hypot(v1x, v1y)
        l2 = math.hypot(v2x, v2y)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        cos_a = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
        if math.degrees(math.acos(cos_a)) > angle_deg:
            flags[i] = True
    return flags


def corner_turn_angle_deg(pts, i):
    """Turn angle (deg) at vertex ``i`` (0 = straight, 180 = full reversal)."""
    n = len(pts)
    if i <= 0 or i >= n - 1:
        return 0.0
    ax, ay = pts[i - 1]
    bx, by = pts[i]
    cx, cy = pts[i + 1]
    v1x, v1y = bx - ax, by - ay
    v2x, v2y = cx - bx, cy - by
    l1 = math.hypot(v1x, v1y)
    l2 = math.hypot(v2x, v2y)
    if l1 < 1e-6 or l2 < 1e-6:
        return 0.0
    cos_a = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
    return math.degrees(math.acos(cos_a))


def plan_speed_limits(pts, cum, print_speed, *, corner_angle_deg=30.0,
                      corner_speed_factor=0.4, decel_mm=1.5):
    """Build a corner-aware speed-limit profile ``speed_limit_at(s)`` (mm/s) over
    the path. Each corner sharper than ``corner_angle_deg`` gets a speed limit
    that scales from ``print_speed`` (a just-past-threshold bend) down toward
    ``print_speed·corner_speed_factor`` (a full 180° reversal); the limit ramps
    linearly (in arc length) from the full speed back down to the corner limit
    over ``decel_mm`` on each side, so the profile is reachable. Straight paths
    (no corners) return a constant ``print_speed`` limit.

    Returns ``(speed_limit_at, corners)`` where ``corners`` is a list of
    ``(s_corner, angle_deg, corner_speed)`` for logging/tests.
    """
    ps = max(0.05, float(print_speed))
    csf = max(0.0, min(1.0, float(corner_speed_factor)))
    dz = max(1e-3, float(decel_mm))
    corners = []
    n = len(pts)
    for i in range(1, n - 1):
        ang = corner_turn_angle_deg(pts, i)
        if ang > corner_angle_deg:
            # sharper turn → slower: lerp full→factor by angle/180
            frac = csf + (1.0 - csf) * max(0.0, 1.0 - ang / 180.0)
            corners.append((cum[i], ang, ps * frac))

    if not corners:
        def _flat(_s, _ps=ps):
            return _ps
        return _flat, corners

    def _limit(s):
        v = ps
        for sc, _ang, vc in corners:
            ramp = vc + (ps - vc) * min(1.0, abs(s - sc) / dz)
            if ramp < v:
                v = ramp
        return max(0.05, v)

    return _limit, corners


# ── Pure-pursuit + cross-track PID control step ───────────────────────

@dataclass
class PursuitState:
    """Mutable per-run state for :func:`pursuit_step`."""
    s: float = 0.0            # arc-length progress committed so far
    seg_i: int = 0            # projection segment cursor (forward-only)
    prev_cross: float = 0.0   # previous SIGNED cross-track error (mm) for kd
    max_cross: float = 0.0    # running |cross| max (mm), diagnostic


def pursuit_step(pos, pts, cum, state, *, lookahead, speed_cap_mm_s,
                 speed_limit_at, dt, max_ds, kp=0.0, kd=0.0,
                 lookahead_at=None):
    """One control tick. ``pos`` = measured (x, y) in the path's units (mm).

    Projects ``pos`` onto the path (bounded forward window → no loop-snapping),
    advances ``state.s`` (forward-only, ≤ ``max_ds``), places a carrot
    ``lookahead`` ahead, steers toward it at ``min(speed_cap, corner-limit)``,
    and adds a cross-track PID correction perpendicular to the path so the stage
    is pulled back onto the line (kp/kd = 0 ⇒ pure pursuit).

    v7.7: ``lookahead_at(s) -> mm`` optionally makes the carrot distance vary
    along the path (``None`` = the fixed ``lookahead``, byte-identical). This is
    what lets a corner be taken by SLOWING DOWN instead of stopping: the
    pure-pursuit standoff is ``lookahead²/(2R)``, so it is the *lookahead* — not
    the speed — that sets how far inside a corner the stage cuts. Shrinking the
    carrot at a corner is the only thing that reduces the cut; the speed must
    then come down with it because dead-time stability requires
    ``v ≤ lookahead/(L·safety)``.

    Returns ``(vx_um_s, vy_um_s, s, cross_unsigned_mm)``.
    """
    total = cum[-1]
    s_raw, seg_i, cross = project_on_polyline(pos, pts, cum, state.seg_i, max_ds)
    s = min(max(s_raw, state.s), state.s + max_ds)
    state.seg_i = seg_i

    la = float(lookahead if lookahead_at is None else lookahead_at(s))
    # carrot ahead of REAL progress
    s_tgt = min(s + la, total)
    cx, cy = point_at_arclength(pts, cum, s_tgt)
    ex, ey = cx - pos[0], cy - pos[1]
    dist = math.hypot(ex, ey)

    v = min(float(speed_cap_mm_s), speed_limit_at(s))
    if dist > 1e-9:
        vx = v * ex / dist
        vy = v * ey / dist
    else:
        vx = vy = 0.0

    # cross-track PID: signed distance from the path (left of travel = +),
    # correction pushes back toward the line.
    p = point_at_arclength(pts, cum, s)
    tx, ty = tangent_at_arclength(pts, cum, s)
    d_signed = tx * (pos[1] - p[1]) - ty * (pos[0] - p[0])
    ddot = (d_signed - state.prev_cross) / dt if dt > 1e-9 else 0.0
    state.prev_cross = d_signed
    if kp or kd:
        corr = -(kp * d_signed + kd * ddot)   # mm/s along the left normal
        vx += corr * (-ty)
        vy += corr * (tx)

    state.s = s
    if cross > state.max_cross:
        state.max_cross = cross
    return vx * 1000.0, vy * 1000.0, s, cross


# ── Ground the control params in MEASURED calibration ─────────────────

def resolve_control(*, print_speed_mm_s, lookahead_mm,
                    xy_max_speed_um_s=0.0, control_loop_ms=0.0, phase_lag_s=0.0,
                    default_control_hz=25.0, fallback_max_um_s=50000.0,
                    safety=2.0,
                    dead_time_s=0.0, lead_time_frac=0.0,
                    min_lookahead_frac=0.0, max_speed_frac=0.0,
                    hold_speed=False):
    """Resolve the loop rate, SMS/clamp ceiling, and the dead-time speed cap from
    the machine's MEASURED calibration. All measured inputs default to 0 (=
    unmeasured) so an uncalibrated machine falls back to the class constants /
    safety envelope exactly as before.

    Returns ``{control_hz, max_um_s, speed_cap_mm_s, lookahead_mm, dead_time_s,
    lead_s, cap_reason}``. The first three keys are numerically UNCHANGED from
    the legacy behaviour whenever the new arguments are left at their defaults
    (regression-locked by ``TestLegacyIdentityResolveControl``).

      • control_hz  — 1000/control_loop_ms (clamped 5..60) else default.
      • max_um_s    — measured true top speed (for SMS + the VS clamp) else the
        safety-envelope fallback.
      • speed_cap_mm_s — the commanded straight-line speed after the dead-time
        stability limit and the top-speed clamp.
      • lookahead_mm — the RESOLVED lookahead (see ``min_lookahead_frac``).
      • cap_reason  — which term is binding: ``"print_speed"``, ``"dead_time"``,
        ``"top_speed"`` or ``"hold_speed"``. Surfaced in the UI so the operator
        can see WHY a print is slow instead of guessing.

    ── Why the new arguments exist ───────────────────────────────────────
    The legacy cap is ``lookahead / ((loop + phase_lag) · safety)``, which makes
    **lookahead the speed knob**: sweeping it 0.2 → 1.3 mm sweeps the commanded
    speed 0.31 → 2.04 mm/s. An auto-tune that minimises path error therefore
    minimises *speed*, and parks at the smallest lookahead in its grid — exactly
    what was observed on ME3B V1. Worse, the resulting cap (0.31 mm/s) sits below
    every corner speed limit, so ``min(cap, corner_limit)`` is always the cap and
    the corner-slowdown feature cannot do anything at all.

    Three levers separate the concerns, all default-off:

      • ``dead_time_s`` — a purpose-measured command→motion dead time (see
        ``XYDeadTime.measure_velocity_dead_time``). Used INSTEAD of
        ``phase_lag_s`` when non-zero. ``phase_lag_s`` is derived from
        ``by_phase.intercept_s``, which is an optically-measured *settle* time,
        not a transport delay — on this machine it is polluted by a single-point
        0.577 s fit and two physically impossible negatives.
      • ``lead_time_frac`` — the fraction of the dead time that the follower
        compensates by predicting ahead (Stage 3's lead predictor). Only the
        UNCOMPENSATED remainder constrains the speed.
      • ``min_lookahead_frac`` — size the lookahead FROM the dynamics
        (``frac · speed · dead_time``) instead of letting it set the speed. This
        is the lever that actually removes the coupling.
      • ``max_speed_frac`` — clamp to a fraction of the *measured* top speed.
        The legacy code never clamped by it at all, so a command above what the
        stage can deliver became a silent open-loop gain error.
      • ``hold_speed`` — pin the straight-line speed to ``print_speed_mm_s`` so
        the corner limit is the ONLY modulation ("keep the velocity, slow only at
        corners"). Only safe once the dead time is real; the caller is expected
        to gate it.
    """
    ps = max(0.05, float(print_speed_mm_s))
    la = max(1e-3, float(lookahead_mm))
    loop_ms = float(control_loop_ms or 0.0)

    if loop_ms > 0:
        control_hz = max(5.0, min(60.0, 1000.0 / loop_ms))
    else:
        control_hz = float(default_control_hz)

    mx = float(xy_max_speed_um_s or 0.0)
    max_um_s = mx if mx > 0 else float(fallback_max_um_s)

    # Transport delay: prefer the purpose-measured value over the legacy
    # by_phase-derived settle time.
    measured_dt = max(0.0, float(dead_time_s or 0.0))
    lag = measured_dt if measured_dt > 0 else max(0.0, float(phase_lag_s or 0.0))

    # Size the lookahead from the dynamics, so it stops being the speed knob.
    dead_total = (loop_ms / 1000.0 + lag) if loop_ms > 0 else lag
    mlf = max(0.0, float(min_lookahead_frac or 0.0))
    if mlf > 0 and dead_total > 1e-6:
        la = max(la, mlf * ps * dead_total)

    # Only the uncompensated share of the dead time constrains the speed.
    ltf = min(1.0, max(0.0, float(lead_time_frac or 0.0)))
    lead_s = lag * ltf
    residual_lag = lag - lead_s

    speed_cap = ps
    cap_reason = "print_speed"
    if loop_ms > 0:
        dead_s = loop_ms / 1000.0 + residual_lag
        if dead_s > 1e-6:
            cap = la / (dead_s * max(1.0, safety))
            if cap < speed_cap:
                speed_cap = cap
                cap_reason = "dead_time"

    if hold_speed:
        # Corners are the only modulation; the follower's corner limit still
        # applies on top of this via min(speed_cap, corner_limit).
        speed_cap = ps
        cap_reason = "hold_speed"

    msf = max(0.0, float(max_speed_frac or 0.0))
    if msf > 0:
        top = (max_um_s / 1000.0) * msf
        if top < speed_cap:
            speed_cap = top
            cap_reason = "top_speed"

    speed_cap = max(0.05, speed_cap)

    return {"control_hz": control_hz, "max_um_s": max_um_s,
            "speed_cap_mm_s": speed_cap, "lookahead_mm": la,
            "dead_time_s": dead_total, "lead_s": lead_s,
            "cap_reason": cap_reason}


def plant_ultimate_gain(dead_time_s, control_loop_ms=0.0):
    """``(Ku, Tu)`` derived ANALYTICALLY from the measured dead time.

    No relay experiment needed — and this is both exactly repeatable and free.

    The cross-track plant is known in closed form: a perpendicular velocity
    command integrates directly into cross-track position, so from ``v_n`` to
    ``d`` the transfer function is a **pure integrator with transport delay**::

        d(s) / v_n(s) = e^{-Ls} / s          L = dead_time + loop_period

    A proportional controller ``v_n = kp·d`` is on the stability boundary when the
    loop phase reaches −180°. The integrator contributes −90°, so the delay must
    contribute the other −90°::

        ω·L = π/2   →   ω_u = π/(2L)
        |kp/ω_u| = 1 at the boundary  →   Ku = ω_u = π/(2L)
        Tu = 2π/ω_u = 4L

    Returns ``(Ku, Tu)``, or ``(0.0, 0.0)`` if ``L`` is not known.

    This is also the **physical ceiling on any measured Ku**: a relay experiment
    that reports a larger value has measured something other than the sustained
    limit cycle (see :func:`relay_sanity`). On ME3B V1 the relay reported
    ``Ku = 18.8`` against a theoretical maximum of 15.9 — while its ``Tu`` of
    0.391 s matched the predicted 0.396 s to about 1 %, which is strong
    confirmation that the model is right and only the amplitude estimate was off.
    """
    L = max(0.0, float(dead_time_s or 0.0)) + \
        max(0.0, float(control_loop_ms or 0.0)) / 1000.0
    if L <= 1e-6:
        return (0.0, 0.0)
    return (math.pi / (2.0 * L), 4.0 * L)


def pid_gains_from_dead_time(dead_time_s, control_loop_ms=0.0, *,
                             method="some_overshoot", gain_margin=1.0,
                             use_kd=True):
    """Cross-track PID gains from the MEASURED dead time — repeatable by design.

    Preferred over the relay experiment: same underlying Ziegler–Nichols tuning
    rule, but ``Ku``/``Tu`` come from :func:`plant_ultimate_gain` instead of from
    an oscillation whose amplitude is easy to mis-measure. Running it twice gives
    the same answer, which is the property the relay tuner lacked.

    ``gain_margin`` > 1 divides the gains down for extra margin (2.0 = half
    gain). ``use_kd=False`` returns ``kd = 0``, which is the right choice while
    the derivative is unfiltered — differentiating a µm-quantised encoder at
    25 Hz injects more noise than the D term removes.

    Returns ``{kp, ki, kd, ku, tu, kp_stability_limit, L_s}``. ``ki`` is reported
    for completeness but should stay 0: the plant is already an integrator, so a
    P-only cross-track controller has zero steady-state error and there is no
    constant lateral disturbance for an I term to reject — a second integrator is
    a classic source of exactly the limit cycle this system suffered from.
    """
    ku, tu = plant_ultimate_gain(dead_time_s, control_loop_ms)
    if ku <= 0.0:
        return {"kp": 0.0, "ki": 0.0, "kd": 0.0, "ku": 0.0, "tu": 0.0,
                "kp_stability_limit": 0.0, "L_s": 0.0}
    factors = {"no_overshoot": (0.20, 0.40, 0.066),
               "some_overshoot": (0.33, 0.66, 0.11),
               "classic": (0.60, 1.20, 0.075)}
    fp, fi, fd = factors.get(method, factors["some_overshoot"])
    gm = max(1.0, float(gain_margin or 1.0))
    kp = fp * ku / gm
    kd = (fd * ku * tu / gm) if use_kd else 0.0
    return {"kp": kp, "ki": 0.0, "kd": kd, "ku": ku, "tu": tu,
            "kp_stability_limit": ku, "L_s": tu / 4.0,
            "ki_theoretical": fi * ku / tu / gm}


def relay_sanity(ku_measured, tu_measured, dead_time_s, control_loop_ms=0.0,
                 *, tol=1.15):
    """Judge a relay-measured ``(Ku, Tu)`` against what the plant can physically do.

    Returns ``{ok, ku_limit, tu_expected, ku_ratio, tu_ratio, reason}``.

    A measured ``Ku`` above ``π/(2L)`` is impossible, so it means the amplitude
    was under-estimated — typically because the switching signal chattered on
    encoder noise near the line and dragged the mean half-cycle amplitude down.
    ``Tu`` is the more trustworthy half of a relay experiment (it is a timing, not
    an amplitude), so a matching ``Tu`` with an inflated ``Ku`` is the signature
    of exactly that failure.
    """
    ku_limit, tu_expected = plant_ultimate_gain(dead_time_s, control_loop_ms)
    out = {"ok": False, "ku_limit": ku_limit, "tu_expected": tu_expected,
           "ku_ratio": 0.0, "tu_ratio": 0.0, "reason": ""}
    if ku_limit <= 0.0:
        out["reason"] = "dead time unmeasured — cannot sanity-check"
        return out
    ku = max(0.0, float(ku_measured or 0.0))
    tu = max(0.0, float(tu_measured or 0.0))
    out["ku_ratio"] = ku / ku_limit if ku_limit else 0.0
    out["tu_ratio"] = tu / tu_expected if tu_expected else 0.0
    if ku <= 0.0 or tu <= 0.0:
        out["reason"] = "degenerate measurement"
        return out
    if out["ku_ratio"] > tol:
        out["reason"] = (
            f"measured Ku {ku:.1f} exceeds the physical maximum "
            f"{ku_limit:.1f} by {out['ku_ratio']:.1f}× — the oscillation "
            f"amplitude was under-measured (noise chatter near the line)")
        return out
    if not (0.5 <= out["tu_ratio"] <= 2.0):
        out["reason"] = (
            f"measured Tu {tu:.2f}s is far from the predicted {tu_expected:.2f}s "
            f"— the relay did not drive the expected limit cycle")
        return out
    out["ok"] = True
    return out


def relay_ultimate_gain(relay_amplitude_mm_s, oscillation_amplitude_mm,
                        period_s):
    """Åström relay-feedback estimate of the ultimate gain/period for auto-
    tuning the cross-track PID. A relay of perpendicular velocity ``±d`` (mm/s)
    that flips on the sign of the cross-track error drives a limit cycle of
    peak amplitude ``a`` (mm) and period ``Tu`` (s). Then::

        Ku = 4·d / (π·a)   (units (mm/s)/mm = 1/s, matching the kp term)
        Tu = period_s

    Returns ``(Ku, Tu)`` — feed to ``MotionController.compute_zn_pid_gains``.
    Returns ``(0.0, 0.0)`` on a degenerate measurement.
    """
    d = abs(float(relay_amplitude_mm_s))
    a = abs(float(oscillation_amplitude_mm))
    tu = abs(float(period_s))
    if a < 1e-6 or tu < 1e-6:
        return (0.0, 0.0)
    ku = 4.0 * d / (math.pi * a)
    return (ku, tu)
