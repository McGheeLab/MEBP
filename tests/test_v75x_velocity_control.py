"""test_v75x_velocity_control.py — the shared PURE control law (VelocityControl).

Covers the corner detection + corner-aware speed-limit profile, the pursuit +
cross-track-PID step (carrot steering + a PID pull that reduces cross-track error),
resolve_control grounding params in measured calibration (the dead-time speed cap
that kills the overshoot limit-cycle), and the ZN relay-gain estimate.
No GUI / hardware.
"""

import math
import unittest

from SupportClasses import VelocityControl as VC


class TestCornerDetection(unittest.TestCase):
    def test_straight_has_no_corners(self):
        pts = [(0, 0), (1, 0), (2, 0), (3, 0)]
        self.assertEqual(VC.corner_flags(pts, 30.0), [False] * 4)

    def test_right_angle_flagged(self):
        pts = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]   # 90° at index 2
        flags = VC.corner_flags(pts, 30.0)
        self.assertTrue(flags[2])
        self.assertFalse(flags[1])
        self.assertFalse(flags[3])

    def test_turn_angle(self):
        pts = [(0, 0), (1, 0), (2, 1)]                    # 45° turn at index 1
        self.assertAlmostEqual(VC.corner_turn_angle_deg(pts, 1), 45.0, places=3)


class TestSpeedLimits(unittest.TestCase):
    def test_flat_when_no_corners(self):
        pts = [(0, 0), (1, 0), (2, 0), (3, 0)]
        cum = VC.polyline_arclength(pts)
        limit, corners = VC.plan_speed_limits(pts, cum, 5.0, corner_angle_deg=30)
        self.assertEqual(corners, [])
        self.assertAlmostEqual(limit(0.0), 5.0)
        self.assertAlmostEqual(limit(2.5), 5.0)

    def test_slows_at_corner_and_recovers(self):
        # long straight, sharp 90° corner in the middle, long straight
        pts = [(0, 0), (2, 0), (4, 0), (4, 2), (4, 4)]   # corner @ (4,0), s=4
        cum = VC.polyline_arclength(pts)
        limit, corners = VC.plan_speed_limits(
            pts, cum, 6.0, corner_angle_deg=30, corner_speed_factor=0.3,
            decel_mm=1.5)
        self.assertEqual(len(corners), 1)
        s_corner = corners[0][0]
        self.assertLess(limit(s_corner), 6.0)            # slowed at the corner
        self.assertLess(limit(s_corner), limit(s_corner - 3.0))  # ramp in
        self.assertAlmostEqual(limit(s_corner - 3.0), 6.0, places=3)  # full away
        self.assertGreaterEqual(limit(s_corner), 0.05)


class TestPursuitStep(unittest.TestCase):
    def _line(self):
        pts = [(0.0, 0.0), (10.0, 0.0)]
        return pts, VC.polyline_arclength(pts)

    def test_steers_forward_along_line(self):
        pts, cum = self._line()
        st = VC.PursuitState()
        flat = lambda _s: 5.0
        vx, vy, s, cross = VC.pursuit_step(
            (0.0, 0.0), pts, cum, st, lookahead=0.6, speed_cap_mm_s=5.0,
            speed_limit_at=flat, dt=0.04, max_ds=1.0)
        self.assertGreater(vx, 0.0)                      # +x
        self.assertAlmostEqual(vy, 0.0, places=6)
        self.assertLess(cross, 1e-6)

    def test_speed_cap_and_corner_limit_respected(self):
        pts, cum = self._line()
        st = VC.PursuitState()
        vx, vy, _s, _c = VC.pursuit_step(
            (0.0, 0.0), pts, cum, st, lookahead=0.6, speed_cap_mm_s=2.0,
            speed_limit_at=lambda _s: 1.0, dt=0.04, max_ds=1.0)   # limit < cap
        self.assertAlmostEqual(math.hypot(vx, vy), 1000.0, delta=1.0)  # 1 mm/s

    def test_cross_track_pid_pulls_back(self):
        # a point off the line (+y) → the PID term adds a -y velocity component
        pts, cum = self._line()
        base = VC.pursuit_step(
            (3.0, 0.5), pts, cum, VC.PursuitState(), lookahead=0.6,
            speed_cap_mm_s=5.0, speed_limit_at=lambda _s: 5.0, dt=0.04,
            max_ds=1.0, kp=0.0, kd=0.0)
        withpid = VC.pursuit_step(
            (3.0, 0.5), pts, cum, VC.PursuitState(), lookahead=0.6,
            speed_cap_mm_s=5.0, speed_limit_at=lambda _s: 5.0, dt=0.04,
            max_ds=1.0, kp=3.0, kd=0.0)
        # PID makes the y-velocity more negative (stronger pull back to y=0)
        self.assertLess(withpid[1], base[1])

    def test_forward_only_progress(self):
        pts, cum = self._line()
        st = VC.PursuitState()
        st.s = 5.0
        _vx, _vy, s, _c = VC.pursuit_step(
            (2.0, 0.0), pts, cum, st, lookahead=0.6, speed_cap_mm_s=5.0,
            speed_limit_at=lambda _s: 5.0, dt=0.04, max_ds=1.0)
        self.assertGreaterEqual(s, 5.0)                  # never goes backward


class TestResolveControl(unittest.TestCase):
    def test_unmeasured_falls_back(self):
        r = VC.resolve_control(print_speed_mm_s=5.0, lookahead_mm=0.6,
                               default_control_hz=25.0, fallback_max_um_s=22000.0)
        self.assertEqual(r["control_hz"], 25.0)
        self.assertEqual(r["max_um_s"], 22000.0)
        self.assertAlmostEqual(r["speed_cap_mm_s"], 5.0)   # no cap w/o loop time

    def test_measured_caps_speed_and_sets_rate(self):
        # 150 ms loop, lookahead 0.6 mm, safety 2 → cap = 0.6/(0.15·2) = 2.0 mm/s
        r = VC.resolve_control(
            print_speed_mm_s=5.0, lookahead_mm=0.6, control_loop_ms=150.0,
            xy_max_speed_um_s=22000.0, default_control_hz=25.0, safety=2.0)
        self.assertAlmostEqual(r["control_hz"], 1000.0 / 150.0, places=3)
        self.assertEqual(r["max_um_s"], 22000.0)
        self.assertAlmostEqual(r["speed_cap_mm_s"], 2.0, places=3)

    def test_phase_lag_widens_deadtime(self):
        no_lag = VC.resolve_control(
            print_speed_mm_s=10.0, lookahead_mm=0.6, control_loop_ms=100.0,
            phase_lag_s=0.0, safety=2.0)["speed_cap_mm_s"]
        with_lag = VC.resolve_control(
            print_speed_mm_s=10.0, lookahead_mm=0.6, control_loop_ms=100.0,
            phase_lag_s=0.1, safety=2.0)["speed_cap_mm_s"]
        self.assertLess(with_lag, no_lag)                # more dead time → slower


class TestRelayGain(unittest.TestCase):
    def test_astrom_formula(self):
        # Ku = 4d/(π a); Tu = period
        ku, tu = VC.relay_ultimate_gain(2.0, 0.5, 0.8)
        self.assertAlmostEqual(ku, 4.0 * 2.0 / (math.pi * 0.5), places=6)
        self.assertAlmostEqual(tu, 0.8)

    def test_degenerate_returns_zero(self):
        self.assertEqual(VC.relay_ultimate_gain(2.0, 0.0, 0.8), (0.0, 0.0))
        self.assertEqual(VC.relay_ultimate_gain(2.0, 0.5, 0.0), (0.0, 0.0))


# ── Golden legacy-identity contract ──────────────────────────────────
#
# The XY-Challenge upgrade adds bounded cross-track authority, a forward floor,
# a filtered derivative, re-acquire, stall/dither detection, a lead predictor
# and a curvature-aware speed profile. EVERY one of those is opt-in: with the
# new tuning absent (or all-zero) the law must behave EXACTLY as it does today,
# because the uncalibrated machine and the existing print path both rely on it.
#
# These tests recompute today's formulas INLINE (deliberately duplicating them
# rather than calling the module) and assert bit-for-bit agreement. If a later
# refactor changes the legacy arithmetic even slightly, these fail.

class TestLegacyIdentityPursuitStep(unittest.TestCase):
    """pursuit_step with no tuning == the documented legacy formula."""

    @staticmethod
    def _legacy(pos, pts, cum, state, *, lookahead, speed_cap_mm_s,
                speed_limit_at, dt, max_ds, kp, kd):
        """Verbatim transcription of the v7.5.3 pursuit_step body."""
        total = cum[-1]
        s_raw, seg_i, cross = VC.project_on_polyline(
            pos, pts, cum, state.seg_i, max_ds)
        s = min(max(s_raw, state.s), state.s + max_ds)
        state.seg_i = seg_i

        s_tgt = min(s + lookahead, total)
        cx, cy = VC.point_at_arclength(pts, cum, s_tgt)
        ex, ey = cx - pos[0], cy - pos[1]
        dist = math.hypot(ex, ey)

        v = min(float(speed_cap_mm_s), speed_limit_at(s))
        if dist > 1e-9:
            vx = v * ex / dist
            vy = v * ey / dist
        else:
            vx = vy = 0.0

        p = VC.point_at_arclength(pts, cum, s)
        tx, ty = VC.tangent_at_arclength(pts, cum, s)
        d_signed = tx * (pos[1] - p[1]) - ty * (pos[0] - p[0])
        ddot = (d_signed - state.prev_cross) / dt if dt > 1e-9 else 0.0
        state.prev_cross = d_signed
        if kp or kd:
            corr = -(kp * d_signed + kd * ddot)
            vx += corr * (-ty)
            vy += corr * (tx)

        state.s = s
        if cross > state.max_cross:
            state.max_cross = cross
        return vx * 1000.0, vy * 1000.0, s, cross

    def _paths(self):
        square = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        line = [(0.0, 0.0), (5.0, 0.0)]
        zig = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.4), (0.0, 0.4), (0.0, 0.8)]
        return {"square": square, "line": line, "zigzag": zig}

    def test_matches_legacy_formula_across_grid(self):
        for name, pts in self._paths().items():
            cum = VC.polyline_arclength(pts)
            for kp, kd in [(0.0, 0.0), (2.0, 0.0), (0.0, 0.05), (4.938, 0.7622)]:
                for off in [0.0, 0.05, -0.2, 0.64]:
                    # Two independent states walked in lockstep.
                    st_a = VC.PursuitState()
                    st_b = VC.PursuitState()
                    limit = (lambda _s: 1.0)
                    for step in range(12):
                        base = VC.point_at_arclength(pts, cum, 0.3 * step)
                        tx, ty = VC.tangent_at_arclength(pts, cum, 0.3 * step)
                        pos = (base[0] - ty * off, base[1] + tx * off)
                        got = VC.pursuit_step(
                            pos, pts, cum, st_a, lookahead=0.6,
                            speed_cap_mm_s=1.0, speed_limit_at=limit,
                            dt=0.04, max_ds=0.24, kp=kp, kd=kd)
                        want = self._legacy(
                            pos, pts, cum, st_b, lookahead=0.6,
                            speed_cap_mm_s=1.0, speed_limit_at=limit,
                            dt=0.04, max_ds=0.24, kp=kp, kd=kd)
                        for g, w in zip(got, want):
                            self.assertAlmostEqual(
                                g, w, delta=1e-12,
                                msg=f"{name} kp={kp} kd={kd} off={off} "
                                    f"step={step}: {got} != {want}")
                        self.assertAlmostEqual(st_a.s, st_b.s, delta=1e-12)
                        self.assertEqual(st_a.seg_i, st_b.seg_i)
                        self.assertAlmostEqual(st_a.prev_cross, st_b.prev_cross,
                                               delta=1e-12)

    def test_zero_gains_is_pure_pursuit_toward_carrot(self):
        """kp=kd=0 → the command points exactly at the carrot at speed v."""
        pts = [(0.0, 0.0), (5.0, 0.0)]
        cum = VC.polyline_arclength(pts)
        st = VC.PursuitState()
        pos = (0.5, 0.3)                      # 0.3 mm off the line
        vx, vy, s, cross = VC.pursuit_step(
            pos, pts, cum, st, lookahead=0.6, speed_cap_mm_s=2.0,
            speed_limit_at=lambda _s: 2.0, dt=0.04, max_ds=0.24,
            kp=0.0, kd=0.0)
        cx, cy = VC.point_at_arclength(pts, cum, min(s + 0.6, cum[-1]))
        ex, ey = cx - pos[0], cy - pos[1]
        n = math.hypot(ex, ey)
        self.assertAlmostEqual(math.hypot(vx, vy), 2000.0, places=6)
        self.assertAlmostEqual(vx / 1000.0, 2.0 * ex / n, places=9)
        self.assertAlmostEqual(vy / 1000.0, 2.0 * ey / n, places=9)


class TestLegacyIdentityResolveControl(unittest.TestCase):
    """resolve_control's three original keys, recomputed inline."""

    @staticmethod
    def _legacy(*, print_speed_mm_s, lookahead_mm, xy_max_speed_um_s=0.0,
                control_loop_ms=0.0, phase_lag_s=0.0,
                default_control_hz=25.0, fallback_max_um_s=50000.0,
                safety=2.0):
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

    def test_matches_legacy_across_grid(self):
        cases = []
        for ps in [0.05, 1.0, 5.0, 20.0]:
            for la in [0.001, 0.2, 0.6, 2.5]:
                for loop in [0.0, 5.0, 31.75, 200.0, 500.0]:
                    for lag in [0.0, 0.0287, 0.2872, 1.0]:
                        for mx in [0.0, 5945.611449038091, 50000.0]:
                            cases.append((ps, la, loop, lag, mx))
        for ps, la, loop, lag, mx in cases:
            got = VC.resolve_control(
                print_speed_mm_s=ps, lookahead_mm=la, xy_max_speed_um_s=mx,
                control_loop_ms=loop, phase_lag_s=lag)
            want = self._legacy(
                print_speed_mm_s=ps, lookahead_mm=la, xy_max_speed_um_s=mx,
                control_loop_ms=loop, phase_lag_s=lag)
            # The three legacy keys must be present and numerically unchanged.
            # Extra keys (resolved lookahead, dead time, lead, cap reason) are
            # additive and do not affect existing callers.
            self.assertLessEqual({"control_hz", "max_um_s", "speed_cap_mm_s"},
                                 set(got))
            for k in want:
                self.assertAlmostEqual(got[k], want[k], delta=1e-12,
                                       msg=f"{k} for {(ps, la, loop, lag, mx)}")
            # ...and the resolved lookahead defaults to the requested one.
            self.assertAlmostEqual(got["lookahead_mm"], max(1e-3, la),
                                   delta=1e-12)
            self.assertEqual(got["lead_s"], 0.0)

    def test_pins_this_machines_measured_cap(self):
        """The exact number the calibration panel must explain.

        Measured on ME3B V1 (2026-07-28): xy_max_speed 5945.6 µm/s,
        control_loop 31.75 ms, phase lag 0.2872 s (mean of the non-negative
        by_phase intercepts), stored lookahead 0.2 mm → 0.3136 mm/s, which is
        1/19th of the stage's measured top speed.
        """
        r = VC.resolve_control(
            print_speed_mm_s=5.0, lookahead_mm=0.2,
            xy_max_speed_um_s=5945.611449038091,
            control_loop_ms=31.75, phase_lag_s=0.2871766666666667)
        self.assertAlmostEqual(r["speed_cap_mm_s"], 0.3135517046761011, places=9)
        self.assertAlmostEqual(r["control_hz"], 31.496062992125985, places=9)

    def test_corner_limits_are_all_above_that_cap(self):
        """Why corner_speed_factor is provably inert on this machine.

        Commanded speed is min(cap, corner_limit). With cap = 0.314 mm/s and
        print_speed = 5 mm/s the corner limits span 1.00–4.25 mm/s, so the min()
        is ALWAYS the cap and the corner knob cannot change anything — which is
        exactly what the operator's sweep showed (RMS 8 µm flat across the whole
        corner_speed_factor grid).
        """
        cap = 0.3135517046761011
        ps = 5.0
        for csf in [0.2, 0.3, 0.4, 0.55, 0.7]:
            for ang in (90.0, 180.0):
                frac = csf + (1.0 - csf) * max(0.0, 1.0 - ang / 180.0)
                limit = ps * frac
                self.assertGreater(limit, cap)
                self.assertAlmostEqual(min(cap, limit), cap, places=12)


class TestProjectionWindowNeverStallsProgress(unittest.TestCase):
    """ROOT CAUSE of the logged deadlock.

    ``pursuit_step`` bounds the projection search to ``max_ds = max(0.15,
    speed·dt·6)`` arc length ahead of the cursor. The scan broke as soon as the
    next segment's far end exceeded that window — so when the window was SMALLER
    than the local segment, only the CURRENT segment was ever examined, ``s``
    pinned at that segment's end, the carrot stopped advancing, and the stage sat
    there while the cross-track term thrashed it about.

    Real evidence (``logs/prints/print_20260727_195339_*.jsonl``): print speed
    0.996 mm/s at 25 Hz → window 0.239 mm, against a toolpath whose mean segment
    was 0.4859 mm. Arc-length progress froze at **0.509 mm** — one segment — while
    12.4 mm of travel produced 1.0 mm of net motion.
    """

    PTS = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]

    def _walk(self, max_ds, n=40):
        cum = VC.polyline_arclength(self.PTS)
        st = VC.PursuitState()
        for k in range(n):
            pos = (min(2.0, 0.05 * k), 0.0)
            VC.pursuit_step(pos, self.PTS, cum, st, lookahead=0.4,
                            speed_cap_mm_s=1.0, speed_limit_at=lambda _s: 1.0,
                            dt=0.04, max_ds=max_ds, kp=0.0, kd=0.0)
        return st.s

    def test_progress_continues_with_a_sub_segment_window(self):
        """The exact failing configuration: window 0.239 mm, segments 0.5 mm."""
        self.assertGreater(self._walk(0.239), 1.5)

    def test_progress_continues_across_a_range_of_windows(self):
        for max_ds in (0.15, 0.239, 0.30, 0.49, 0.51, 0.90, 1.5):
            self.assertGreater(self._walk(max_ds), 1.5,
                               msg=f"stalled with max_ds={max_ds}")

    def test_the_operators_exact_numbers_no_longer_stall(self):
        speed, dt = 0.996, 1.0 / 25.0
        max_ds = min(1.5, max(0.15, speed * dt * 6.0))
        self.assertLess(max_ds, 0.4859)          # window IS smaller than a segment
        pts = [(0.4859 * i, 0.0) for i in range(30)]
        cum = VC.polyline_arclength(pts)
        st = VC.PursuitState()
        for k in range(300):
            pos = (min(cum[-1], speed * dt * k), 0.0)
            VC.pursuit_step(pos, pts, cum, st, lookahead=0.2,
                            speed_cap_mm_s=speed,
                            speed_limit_at=lambda _s: speed, dt=dt,
                            max_ds=max_ds, kp=0.0, kd=0.0)
        self.assertGreater(st.s, 5.0,
                           msg="froze near one segment length, as in the log")

    def test_per_tick_advance_is_still_bounded(self):
        """The window also bounds pump deposition and guards loop-snap, so the
        per-tick committed advance must remain capped even though the SEARCH now
        looks further."""
        cum = VC.polyline_arclength(self.PTS)
        st = VC.PursuitState()
        prev, worst = 0.0, 0.0
        for k in range(40):
            pos = (min(2.0, 0.2 * k), 0.0)       # deliberately jump far each tick
            VC.pursuit_step(pos, self.PTS, cum, st, lookahead=0.4,
                            speed_cap_mm_s=1.0, speed_limit_at=lambda _s: 1.0,
                            dt=0.04, max_ds=0.239, kp=0.0, kd=0.0)
            worst = max(worst, st.s - prev)
            prev = st.s
        self.assertLessEqual(worst, 0.239 + 1e-9)

    def test_examines_at_least_two_segments(self):
        self.assertGreaterEqual(VC.MIN_SEGMENTS_EXAMINED, 2)
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        cum = VC.polyline_arclength(pts)
        # A tiny window must still be able to reach into the second segment.
        s, seg_i, _d = VC.project_on_polyline((1.4, 0.0), pts, cum, 0, 0.01)
        self.assertGreater(s, 1.0)
        self.assertEqual(seg_i, 1)

    def test_still_cannot_reach_a_distant_loop(self):
        """Loop-snap protection is unaffected: two segments is nothing like one
        revolution of a spiral."""
        pts = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)] \
            + [(1.0 + 0.5 * i, 5.0) for i in range(6)] \
            + [(0.0, 0.02)]        # returns spatially NEAR the start, far in arc
        cum = VC.polyline_arclength(pts)
        s, _i, _d = VC.project_on_polyline((0.02, 0.0), pts, cum, 0, 0.239)
        self.assertLess(s, 1.0)    # did not snap to the far, near-in-space vertex


class TestSpeedDecoupledFromLookahead(unittest.TestCase):
    """The headline Stage-1 behaviour: lookahead must stop being the speed knob.

    Grounded in the real ME3B V1 calibration so the numbers are the operator's.
    """

    CAL = dict(xy_max_speed_um_s=5945.611449038091,
               control_loop_ms=31.75, phase_lag_s=0.2871766666666667)

    def _cap(self, la, **kw):
        return VC.resolve_control(print_speed_mm_s=5.0, lookahead_mm=la,
                                  **self.CAL, **kw)

    def test_legacy_lookahead_sweep_is_really_a_speed_sweep(self):
        """Documents the defect: the tuner's lookahead grid IS a speed grid."""
        caps = [self._cap(la)["speed_cap_mm_s"]
                for la in (0.2, 0.35, 0.5, 0.65, 0.8, 1.0, 1.3)]
        self.assertAlmostEqual(caps[0], 0.3135517046761011, places=9)
        self.assertAlmostEqual(caps[-1], 2.038086080394657, places=9)
        self.assertEqual(caps, sorted(caps))            # strictly increasing
        self.assertGreater(caps[-1] / caps[0], 6.0)     # 6.5× speed range

    def test_min_lookahead_frac_breaks_the_coupling(self):
        """With the lookahead sized FROM the dynamics, the smallest grid value
        no longer produces the slowest run."""
        small = self._cap(0.2, min_lookahead_frac=3.0)
        large = self._cap(1.3, min_lookahead_frac=3.0)
        self.assertAlmostEqual(small["speed_cap_mm_s"],
                               large["speed_cap_mm_s"], places=9)
        # ...and the resolved lookahead came from speed × dead time.
        self.assertGreater(small["lookahead_mm"], 0.2)

    def test_lead_time_frac_raises_the_cap(self):
        base = self._cap(0.2)["speed_cap_mm_s"]
        comp = self._cap(0.2, lead_time_frac=0.7)["speed_cap_mm_s"]
        self.assertGreater(comp, base * 2.5)            # ~3.3× on these numbers
        full = self._cap(0.2, lead_time_frac=1.0)["speed_cap_mm_s"]
        self.assertGreater(full, comp)

    def test_measured_dead_time_overrides_the_by_phase_settle_time(self):
        """A real 40 ms transport delay must not be shouted down by the 287 ms
        settle-time average that currently throttles the machine."""
        slow = self._cap(0.6)["speed_cap_mm_s"]
        fast = self._cap(0.6, dead_time_s=0.040)["speed_cap_mm_s"]
        self.assertGreater(fast, slow * 4.0)
        self.assertEqual(self._cap(0.6, dead_time_s=0.040)["dead_time_s"],
                         0.03175 + 0.040)

    def test_max_speed_frac_clamps_to_the_measured_top_speed(self):
        # Ask for 20 mm/s from a stage measured at 5.95 — without this clamp the
        # legacy code commanded it anyway (a silent open-loop gain error).
        r = VC.resolve_control(print_speed_mm_s=20.0, lookahead_mm=3.0,
                               **self.CAL, dead_time_s=0.001,
                               max_speed_frac=0.9)
        self.assertAlmostEqual(r["speed_cap_mm_s"], 5.945611449038091 * 0.9,
                               places=9)
        self.assertEqual(r["cap_reason"], "top_speed")
        # Requesting less than the stage can do is NOT clamped up.
        r2 = self._cap(3.0, dead_time_s=0.001, max_speed_frac=0.9)
        self.assertAlmostEqual(r2["speed_cap_mm_s"], 5.0, places=9)
        self.assertEqual(r2["cap_reason"], "print_speed")

    def test_hold_speed_pins_the_straight_line_speed(self):
        """The operator's requirement: full speed on straights, corners only."""
        r = self._cap(0.2, hold_speed=True)
        self.assertAlmostEqual(r["speed_cap_mm_s"], 5.0, places=9)
        self.assertEqual(r["cap_reason"], "hold_speed")

    def test_hold_speed_lets_the_corner_limit_bind_again(self):
        """Why hold_speed is the prerequisite for corner tuning.

        With the legacy cap (0.314) every corner limit is above it, so
        min(cap, corner_limit) is always the cap and corner_speed_factor is
        inert. Pinning the speed makes the corner limit the binding term.
        """
        ps = 5.0
        legacy_cap = self._cap(0.2)["speed_cap_mm_s"]
        held_cap = self._cap(0.2, hold_speed=True)["speed_cap_mm_s"]
        for csf in (0.2, 0.4, 0.7):
            corner = ps * csf                    # a 180° reversal
            self.assertEqual(min(legacy_cap, corner), legacy_cap)  # inert
            self.assertAlmostEqual(min(held_cap, corner), corner)  # binds
        # ...and different factors now give DIFFERENT commanded speeds.
        speeds = {min(held_cap, ps * csf) for csf in (0.2, 0.4, 0.7)}
        self.assertEqual(len(speeds), 3)

    def test_cap_reason_reports_the_binding_term(self):
        self.assertEqual(self._cap(0.2)["cap_reason"], "dead_time")
        self.assertEqual(
            self._cap(5.0, dead_time_s=0.001)["cap_reason"], "print_speed")


class TestLegacyIdentitySpeedLimits(unittest.TestCase):
    """plan_speed_limits must stay byte-identical (the new curvature-aware
    profile lands alongside it as plan_speed_profile, not in place of it)."""

    @staticmethod
    def _legacy(pts, cum, print_speed, *, corner_angle_deg, corner_speed_factor,
                decel_mm):
        ps = max(0.05, float(print_speed))
        csf = max(0.0, min(1.0, float(corner_speed_factor)))
        dz = max(1e-3, float(decel_mm))
        corners = []
        for i in range(1, len(pts) - 1):
            ang = VC.corner_turn_angle_deg(pts, i)
            if ang > corner_angle_deg:
                frac = csf + (1.0 - csf) * max(0.0, 1.0 - ang / 180.0)
                corners.append((cum[i], ang, ps * frac))
        if not corners:
            return (lambda _s, _ps=ps: _ps), corners

        def _limit(s):
            v = ps
            for sc, _ang, vc in corners:
                ramp = vc + (ps - vc) * min(1.0, abs(s - sc) / dz)
                if ramp < v:
                    v = ramp
            return max(0.05, v)
        return _limit, corners

    def test_matches_legacy_profile(self):
        paths = {
            "square": [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)],
            "line": [(0.0, 0.0), (5.0, 0.0)],
            "reversal": [(0.0, 0.0), (2.0, 0.0), (0.0, 0.0)],
        }
        for name, pts in paths.items():
            cum = VC.polyline_arclength(pts)
            for ps in [0.05, 1.0, 5.0]:
                for csf in [0.0, 0.2, 0.4, 1.0]:
                    for dz in [1e-4, 0.5, 1.5]:
                        got_fn, got_c = VC.plan_speed_limits(
                            pts, cum, ps, corner_angle_deg=30.0,
                            corner_speed_factor=csf, decel_mm=dz)
                        want_fn, want_c = self._legacy(
                            pts, cum, ps, corner_angle_deg=30.0,
                            corner_speed_factor=csf, decel_mm=dz)
                        self.assertEqual(len(got_c), len(want_c), msg=name)
                        for g, w in zip(got_c, want_c):
                            for gi, wi in zip(g, w):
                                self.assertAlmostEqual(gi, wi, delta=1e-12)
                        total = cum[-1]
                        for k in range(41):
                            s = total * k / 40.0
                            self.assertAlmostEqual(
                                got_fn(s), want_fn(s), delta=1e-12,
                                msg=f"{name} ps={ps} csf={csf} dz={dz} s={s}")


class TestLegacyIdentityProjection(unittest.TestCase):
    """project_on_polyline is forward-only today; the re-acquire work adds an
    opt-in backward window. Pin the forward-only behaviour."""

    def test_forward_only_cannot_look_behind(self):
        pts = [(0.0, 0.0), (5.0, 0.0)]
        cum = VC.polyline_arclength(pts)
        # Position is well BEHIND seg_start's arc length; a forward-only search
        # clamps the projection to the window start, it does not walk back.
        s, seg_i, cross = VC.project_on_polyline(
            (0.2, 0.0), pts, cum, 0, 0.24)
        self.assertAlmostEqual(s, 0.2, places=9)
        # ...but with the cursor already advanced, the same query cannot recover
        # a smaller s than the window allows.
        pts2 = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        cum2 = VC.polyline_arclength(pts2)
        s2, _i2, _c2 = VC.project_on_polyline((0.1, 0.0), pts2, cum2, 2, 0.24)
        self.assertGreaterEqual(s2, cum2[2] - 1e-12)


if __name__ == "__main__":
    unittest.main()
