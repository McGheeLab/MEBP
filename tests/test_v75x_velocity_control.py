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


if __name__ == "__main__":
    unittest.main()
