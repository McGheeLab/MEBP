"""test_v75x_quick_print_velocity_follow.py — closed-loop velocity-following print.

The open-loop PRINT_PATH streamed each XY move fire-and-forget, paced by
time.sleep — so a stage ~2.5× slower than commanded fell PROGRESSIVELY behind
the stream (lag grew to ~6-10 mm across a ~10 mm print) and the deposited pattern
smeared. Fix: PrintSettings.velocity_follow → a real-time control loop
(_execute_print_path_velocity) that polls the ACTUAL position and continuously
re-commands a velocity vector (Prior VS) toward a carrot placed a fixed lookahead
ahead of the stage's REAL arc-length progress (pure pursuit). The carrot can
never run ahead of a slow stage, and the pump deposits volume per real distance
travelled — so the bead is correct at any speed. Quick Print enables it.

Covers: arc-length helpers, dispatch, the runaway guard (VS sign inversion), the
pump-per-real-distance deposition, end-stop, and Quick Print wiring.
"""

import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType,
    polyline_arclength, point_at_arclength, project_on_polyline,
)


# ── 1. Pure arc-length helpers ───────────────────────────────────────

class TestArcLengthHelpers(unittest.TestCase):
    def test_cumulative_arclength(self):
        pts = [(0, 0), (3, 0), (3, 4)]     # 3 then 4 → 7
        self.assertEqual(polyline_arclength(pts), [0.0, 3.0, 7.0])

    def test_point_at_arclength_mid_and_clamped(self):
        pts = [(0, 0), (10, 0)]
        cum = polyline_arclength(pts)
        self.assertEqual(point_at_arclength(pts, cum, 2.5), (2.5, 0.0))
        self.assertEqual(point_at_arclength(pts, cum, -1), (0.0, 0.0))   # clamp lo
        self.assertEqual(point_at_arclength(pts, cum, 999), (10.0, 0.0))  # clamp hi

    def test_project_on_and_off_path(self):
        pts = [(0, 0), (10, 0)]
        cum = polyline_arclength(pts)
        s, i, d = project_on_polyline((4.0, 0.0), pts, cum, 0, 20)
        self.assertAlmostEqual(s, 4.0, places=6)
        self.assertAlmostEqual(d, 0.0, places=6)
        # a point 5 mm off the path → cross distance 5, projects onto x=4
        s2, _i2, d2 = project_on_polyline((4.0, 5.0), pts, cum, 0, 20)
        self.assertAlmostEqual(s2, 4.0, places=6)
        self.assertAlmostEqual(d2, 5.0, places=6)

    def test_bounded_window_prevents_loop_snap(self):
        # Boustrophedon (rows 0.4 mm apart) reproduces the spiral runaway: a query
        # point between two rows is EUCLIDEAN-closest to the *later* row but a
        # full pass away in ARC LENGTH. A large search window snaps to it
        # (teleporting the carrot → runaway); a bounded window keeps it on the
        # current row.
        pts = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0),   # row0  s 0..5
               (5, 0.4),                                          # step  s 5.4
               (4, 0.4), (3, 0.4), (2, 0.4), (1, 0.4), (0, 0.4)]  # row1  s 6.4..10.4
        cum = polyline_arclength(pts)
        query = (2.5, 0.3)          # closer to row1 (0.1) than row0 (0.3)
        seg_on_row0 = 2             # we are tracking row0, near s≈2.4
        # large window → SNAP to row1 (~s 7.9)
        s_big, _i, _d = project_on_polyline(query, pts, cum, seg_on_row0, 10.0)
        self.assertGreater(s_big, 6.0)
        # bounded window → stays on row0 (~s 2.5)
        s_small, _i2, _d2 = project_on_polyline(query, pts, cum, seg_on_row0, 1.0)
        self.assertLess(s_small, 3.5)


# ── 2. Dispatch: velocity_follow routes _execute_print_path → velocity ─

class TestDispatch(unittest.TestCase):
    def _pm(self, velocity_follow, has_vel=True, xy_conn=True):
        ctrl = MagicMock()
        ctrl.is_xy_connected = xy_conn
        if not has_vel:
            del ctrl.send_velocity_xy
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            velocity_follow=velocity_follow, print_speed_mm_s=5.0))
        pm.exec_logger = None
        pm._execute_print_path_velocity = MagicMock()
        pm._set_xy_speed_for_print = lambda *a, **k: None
        pm._wait_for_xy_settle = lambda *a, **k: None
        pm._print_pump_suckback = lambda *a, **k: None
        return pm

    def _run(self, pm):
        with patch("time.sleep"):
            pm._execute_print_path(PrintCommand(
                type=CommandType.PRINT_PATH,
                params={"points": [(0.0, 0.0), (1.0, 0.0)], "pump": "P1",
                        "flow_rate_uL_s": 0.0, "flow_rate": 0.0}))

    def test_velocity_follow_uses_velocity_path(self):
        pm = self._pm(velocity_follow=True)
        self._run(pm)
        pm._execute_print_path_velocity.assert_called_once()

    def test_default_uses_open_loop(self):
        pm = self._pm(velocity_follow=False)
        self._run(pm)
        pm._execute_print_path_velocity.assert_not_called()

    def test_falls_back_when_no_velocity_command(self):
        pm = self._pm(velocity_follow=True, has_vel=False)
        self._run(pm)
        pm._execute_print_path_velocity.assert_not_called()


# ── 3. Integrating fake stage that actually follows VS ───────────────

class _VelStage:
    """XY stage that integrates the commanded VS velocity into position (like
    XYStageSimulator) so the control loop physically progresses."""
    def __init__(self):
        self.vx = self.vy = 0.0

    def set_acceleration(self, a):
        pass

    def set_speed_mm_s(self, s):
        pass


class _VelCtrl:
    def __init__(self, follow=True, mode="follow"):
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.safety_limits = SimpleNamespace(max_xy_speed=50000.0)
        self.xy_stage = _VelStage()
        self.zp_stage = SimpleNamespace(axis_map={"P1": "Y"})
        self._x = 0.0    # µm (raw)
        self._y = 0.0
        self._vx = 0.0   # µm/s
        self._vy = 0.0
        self._last = time.monotonic()
        self.pump_uL = 0.0
        self.stop_calls = 0
        self.vel_calls = 0
        self._mode = mode   # "follow" integrates VS; "perp" drifts +y (runaway)

    def _integrate(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        if self._mode == "perp":
            self._y += 4000.0 * dt          # 4 mm/s off-path, ignores command
        else:
            self._x += self._vx * dt
            self._y += self._vy * dt

    def send_velocity_xy(self, vx, vy):
        self.vel_calls += 1
        if vx == 0.0 and vy == 0.0:
            self.stop_calls += 1
        self._integrate()
        self._vx, self._vy = vx, vy

    def get_xy_position(self, cached=False):
        self._integrate()
        return (self._x, self._y, 0.0)

    def move_xy_absolute(self, x_mm, y_mm, from_zero_ref=True):
        self._integrate()
        self._x = x_mm * 1000.0
        self._y = y_mm * 1000.0
        self._vx = self._vy = 0.0

    def move_pump_uL(self, pump, uL, rate=None):
        self.pump_uL += uL


class _Harness:
    def run(self, ctrl, pts, flow=0.4, speed=8.0):
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            velocity_follow=True, print_speed_mm_s=speed))
        pm.exec_logger = None
        pm._print_pump_suckback = lambda *a, **k: None
        pm._zp_connected_at_start = True
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
            "points": pts, "pump": "P1",
            "flow_rate_uL_s": flow, "flow_rate": 0.01})
        pm._execute_print_path_velocity(cmd)
        return pm


class TestVelocityFollowRun(unittest.TestCase):
    def test_follows_to_end_and_stops(self):
        ctrl = _VelCtrl()
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]  # 3 mm line
        _Harness().run(ctrl, pts)
        # ended near the final point
        self.assertLess(abs(ctrl._x / 1000.0 - 3.0), 0.1)
        self.assertLess(abs(ctrl._y / 1000.0 - 0.0), 0.1)
        # and the stage was explicitly stopped (VS 0,0) at least once
        self.assertGreaterEqual(ctrl.stop_calls, 1)

    def test_deposits_volume_per_real_distance(self):
        ctrl = _VelCtrl()
        pts = [(0.0, 0.0), (2.0, 0.0)]          # 2 mm
        _Harness().run(ctrl, pts, flow=0.4, speed=8.0)
        # vol_per_mm = flow/speed = 0.05 µL/mm → 2 mm ⇒ ~0.10 µL
        self.assertAlmostEqual(ctrl.pump_uL, 0.10, delta=0.03)

    def test_completes_multiloop_spiral_without_snap(self):
        # A shrinking spiral (adjacent loops close in space but a full turn apart
        # in arc length) — the exact geometry that snapped the projection and
        # tripped the runaway on hardware. With the bounded per-tick advance it
        # must run to the end with no RuntimeError.
        pts = []
        turns, per_turn = 3.0, 24
        n = int(turns * per_turn)
        for k in range(n + 1):
            ang = 2 * math.pi * turns * k / n
            r = 2.0 - 1.6 * k / n          # 2.0 mm → 0.4 mm
            pts.append((r * math.cos(ang), r * math.sin(ang)))
        ctrl = _VelCtrl()
        # seed the stage at the path start so the first move is a no-op
        ctrl._x, ctrl._y = pts[0][0] * 1000.0, pts[0][1] * 1000.0
        _Harness().run(ctrl, pts, flow=0.4, speed=8.0)   # no exception = pass
        # reached the spiral centre (final point)
        self.assertLess(math.hypot(ctrl._x / 1000.0 - pts[-1][0],
                                   ctrl._y / 1000.0 - pts[-1][1]), 0.2)

    def test_runaway_guard_aborts_on_off_path_motion(self):
        # A stage that drifts perpendicular to the path (≈ a VS sign inversion)
        # must trip the cross-track runaway guard and raise (→ outer loop aborts
        # + retracts), not drive on blindly.
        ctrl = _VelCtrl(mode="perp")
        pts = [(0.0, 0.0), (5.0, 0.0)]
        with self.assertRaises(RuntimeError):
            _Harness().run(ctrl, pts)
        # and it stopped the stage on the way out (finally: VS 0,0)
        self.assertGreaterEqual(ctrl.stop_calls, 1)


# ── 4. Quick Print wiring ────────────────────────────────────────────

class TestQuickPrintEnablesVelocity(unittest.TestCase):
    """The Motion-mode selector maps to the PrintSettings flags. Default
    (open_loop) = the original streamed path; 'velocity'/'confirm' opt in."""

    def _page(self, mode):
        from gui.pages.workflows import quick_print_workflow as qpw
        page = qpw.QuickPrintWorkflowPage.__new__(qpw.QuickPrintWorkflowPage)

        class _C:
            def print_z_dir(self):
                return 1.0

            def plate_axis_sign(self):
                return (1.0, 1.0)

            def default_travel_z(self, z, margin_mm=10.0):
                return z + margin_mm

        page._controller = _C()
        page._safe_z = 40.0
        page._resolved_print_kinematics = lambda: (2.5, 0.4, 0.1)
        page._resolve_print_z = lambda: 21.0
        page._pump = lambda: "P1"
        page._travel_speed = None
        page._line_retract_spin = None
        page._line_z_speed_spin = None
        page._line_xy_speed_spin = None
        if mode is not None:
            page._motion_mode = lambda: mode
        return page

    def test_velocity_mode(self):
        s = self._page("velocity")._build_settings()
        self.assertTrue(s.velocity_follow)
        self.assertFalse(s.confirm_each_segment)

    def test_confirm_mode(self):
        s = self._page("confirm")._build_settings()
        self.assertTrue(s.confirm_each_segment)
        self.assertFalse(s.velocity_follow)

    def test_open_loop_is_default(self):
        # no selector present → default open_loop → neither flag set
        s = self._page(None)._build_settings()
        self.assertFalse(s.velocity_follow)
        self.assertFalse(s.confirm_each_segment)

    def test_defaults_are_false(self):
        s = PrintSettings()
        self.assertFalse(s.velocity_follow)
        self.assertFalse(s.confirm_each_segment)


if __name__ == "__main__":
    unittest.main()
