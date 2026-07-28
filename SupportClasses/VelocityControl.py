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


def project_on_polyline(pos, pts, cum, seg_start, max_ahead):
    """Project ``pos`` onto the polyline, searching segments forward from
    ``seg_start`` while their far end stays within ``max_ahead`` arc length of
    ``cum[seg_start]`` (bounds the search + prevents snapping to a later loop of
    a self-intersecting path). Returns ``(s, seg_idx, cross_dist)``."""
    n = len(pts)
    best_s = cum[min(seg_start, n - 1)]
    best_i = min(seg_start, n - 2) if n >= 2 else 0
    best_d = float("inf")
    limit = cum[min(seg_start, n - 1)] + max_ahead
    i = max(0, min(seg_start, n - 2))
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
        if cum[i + 1] > limit:
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
                 speed_limit_at, dt, max_ds, kp=0.0, kd=0.0):
    """One control tick. ``pos`` = measured (x, y) in the path's units (mm).

    Projects ``pos`` onto the path (bounded forward window → no loop-snapping),
    advances ``state.s`` (forward-only, ≤ ``max_ds``), places a carrot
    ``lookahead`` ahead, steers toward it at ``min(speed_cap, corner-limit)``,
    and adds a cross-track PID correction perpendicular to the path so the stage
    is pulled back onto the line (kp/kd = 0 ⇒ pure pursuit).

    Returns ``(vx_um_s, vy_um_s, s, cross_unsigned_mm)``.
    """
    total = cum[-1]
    s_raw, seg_i, cross = project_on_polyline(pos, pts, cum, state.seg_i, max_ds)
    s = min(max(s_raw, state.s), state.s + max_ds)
    state.seg_i = seg_i

    # carrot ahead of REAL progress
    s_tgt = min(s + lookahead, total)
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
                    safety=2.0):
    """Resolve the loop rate, SMS/clamp ceiling, and a dead-time speed cap from
    the machine's MEASURED calibration. All measured inputs default to 0 (=
    unmeasured) so an uncalibrated machine falls back to the class constants /
    safety envelope exactly as before.

    Returns ``{control_hz, max_um_s, speed_cap_mm_s}``:
      • control_hz  — 1000/control_loop_ms (clamped 5..60) else default.
      • max_um_s    — measured true top speed (for SMS + the VS clamp) else the
        safety-envelope fallback.
      • speed_cap_mm_s — print speed, capped so the stage can't travel more than
        a safe fraction of the lookahead within one control period (loop period
        + phase lag) → kills the pure-pursuit dead-time overshoot limit-cycle.
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

    speed_cap = ps
    if loop_ms > 0:
        dead_s = loop_ms / 1000.0 + max(0.0, float(phase_lag_s or 0.0))
        if dead_s > 1e-6:
            cap = la / (dead_s * max(1.0, safety))
            speed_cap = min(speed_cap, cap)
    speed_cap = max(0.05, speed_cap)

    return {"control_hz": control_hz, "max_um_s": max_um_s,
            "speed_cap_mm_s": speed_cap}


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
