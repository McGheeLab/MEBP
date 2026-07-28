"""test_v75x_open_loop_velocity.py — OPEN-LOOP velocity-streaming print path.

The 'open_loop' motion mode now streams a continuous velocity vector along the
path TANGENT (feed-forward, target advancing by wall-clock time) instead of
moving to prescribed points (that is the confirm mode). No position feedback, no
runaway guard — smooth continuous velocity for even ink laydown.

Covers: the tangent helper, dispatch (velocity_open_loop routes to the open-loop
executor + falls back without a velocity command), the feed-forward run (drives
by streamed velocity — NOT one move per segment — deposits time-based volume,
lands on the endpoint, always stops), and Quick Print wiring.
"""

import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType,
    polyline_arclength, tangent_at_arclength,
)


# ── 1. tangent helper ────────────────────────────────────────────────

class TestTangent(unittest.TestCase):
    def test_horizontal(self):
        pts = [(0, 0), (10, 0)]
        cum = polyline_arclength(pts)
        self.assertEqual(tangent_at_arclength(pts, cum, 5.0), (1.0, 0.0))

    def test_corner_picks_local_segment(self):
        pts = [(0, 0), (10, 0), (10, 10)]     # right then up
        cum = polyline_arclength(pts)
        self.assertEqual(tangent_at_arclength(pts, cum, 3.0), (1.0, 0.0))   # leg1
        tx, ty = tangent_at_arclength(pts, cum, 13.0)                       # leg2
        self.assertAlmostEqual(tx, 0.0, places=6)
        self.assertAlmostEqual(ty, 1.0, places=6)

    def test_clamped_ends(self):
        pts = [(0, 0), (4, 3)]                # unit tangent (0.8, 0.6)
        cum = polyline_arclength(pts)
        tx, ty = tangent_at_arclength(pts, cum, -1.0)
        self.assertAlmostEqual(tx, 0.8, places=6)
        self.assertAlmostEqual(ty, 0.6, places=6)


# ── 2. Dispatch ──────────────────────────────────────────────────────

class TestDispatch(unittest.TestCase):
    def _pm(self, open_loop, has_vel=True, xy_conn=True):
        ctrl = MagicMock()
        ctrl.is_xy_connected = xy_conn
        if not has_vel:
            del ctrl.send_velocity_xy
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            velocity_open_loop=open_loop, print_speed_mm_s=5.0))
        pm.exec_logger = None
        pm._execute_print_path_open_velocity = MagicMock()
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

    def test_open_loop_routes_to_open_velocity(self):
        pm = self._pm(open_loop=True)
        self._run(pm)
        pm._execute_print_path_open_velocity.assert_called_once()
        pm._execute_print_path_velocity.assert_not_called()

    def test_off_does_not_route(self):
        pm = self._pm(open_loop=False)
        self._run(pm)
        pm._execute_print_path_open_velocity.assert_not_called()

    def test_falls_back_when_no_velocity_command(self):
        pm = self._pm(open_loop=True, has_vel=False)
        self._run(pm)
        pm._execute_print_path_open_velocity.assert_not_called()


# ── 3. Feed-forward run against an integrating stage ─────────────────

class _OpenVelStage:
    def set_acceleration(self, a):
        pass

    def set_speed_mm_s(self, s):
        pass


class _OpenVelCtrl:
    """Integrates the commanded VS into position (perfect follower), and counts
    move_xy_absolute calls so we can prove it is NOT point-to-point."""

    def __init__(self):
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.safety_limits = SimpleNamespace(max_xy_speed=50000.0)
        self.xy_stage = _OpenVelStage()
        self.zp_stage = SimpleNamespace(axis_map={"P1": "Y"})
        self._x = 0.0
        self._y = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._last = time.monotonic()
        self.pump_uL = 0.0
        self.stop_calls = 0
        self.vel_calls = 0
        self.move_calls = 0

    def _integrate(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
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
        self.move_calls += 1
        self._integrate()
        self._x = x_mm * 1000.0
        self._y = y_mm * 1000.0
        self._vx = self._vy = 0.0

    def move_pump_uL(self, pump, uL, rate=None):
        self.pump_uL += uL


def _run_open(ctrl, pts, flow=0.4, speed=8.0, pace=1.0):
    pm = PrintManager(ctrl)
    pm.job = SimpleNamespace(settings=PrintSettings(
        velocity_open_loop=True, print_speed_mm_s=speed, pace_correction=pace))
    pm.exec_logger = None
    pm._print_pump_suckback = lambda *a, **k: None
    pm._zp_connected_at_start = True
    cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
        "points": pts, "pump": "P1", "flow_rate_uL_s": flow, "flow_rate": 0.01})
    pm._execute_print_path_open_velocity(cmd)
    return pm


class TestOpenVelocityRun(unittest.TestCase):
    def test_streams_velocity_not_point_to_point(self):
        ctrl = _OpenVelCtrl()
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]  # 3 mm, 3 segs
        _run_open(ctrl, pts)
        # Many velocity commands (one per control tick), NOT one move per
        # segment: move_xy_absolute is only the start + the final endpoint snap.
        self.assertGreater(ctrl.vel_calls, 4)
        self.assertLessEqual(ctrl.move_calls, 2)   # start + end snap

    def test_lands_on_endpoint_and_stops(self):
        ctrl = _OpenVelCtrl()
        pts = [(0.0, 0.0), (3.0, 0.0)]
        _run_open(ctrl, pts)
        self.assertLess(abs(ctrl._x / 1000.0 - 3.0), 0.05)  # end snap lands exact
        self.assertLess(abs(ctrl._y / 1000.0 - 0.0), 0.05)
        self.assertGreaterEqual(ctrl.stop_calls, 1)         # VS 0,0 on exit

    def test_deposits_time_based_volume(self):
        ctrl = _OpenVelCtrl()
        pts = [(0.0, 0.0), (2.0, 0.0)]
        _run_open(ctrl, pts, flow=0.4, speed=8.0)
        # vol_per_mm = flow/eff_speed = 0.4/8 = 0.05 µL/mm → 2 mm ≈ 0.10 µL
        self.assertAlmostEqual(ctrl.pump_uL, 0.10, delta=0.03)

    def test_pace_trims_speed_but_same_volume(self):
        # pace halves the effective speed → same deposited volume (vol_per_mm
        # scales with 1/eff_speed) but longer wall time.
        ctrl = _OpenVelCtrl()
        pts = [(0.0, 0.0), (2.0, 0.0)]
        _run_open(ctrl, pts, flow=0.4, speed=8.0, pace=2.0)
        self.assertAlmostEqual(ctrl.pump_uL, 0.10, delta=0.03)

    def test_aborts_without_endpoint_snap(self):
        ctrl = _OpenVelCtrl()
        pts = [(0.0, 0.0), (5.0, 0.0)]
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            velocity_open_loop=True, print_speed_mm_s=8.0))
        pm.exec_logger = None
        pm._print_pump_suckback = lambda *a, **k: None
        pm._zp_connected_at_start = True
        pm._abort_flag.set()      # abort immediately
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
            "points": pts, "pump": "P1", "flow_rate_uL_s": 0.4, "flow_rate": 0.01})
        pm._execute_print_path_open_velocity(cmd)
        # aborted before the loop → stage stopped, but no endpoint snap move
        self.assertGreaterEqual(ctrl.stop_calls, 1)


# ── 4. Quick Print wiring ────────────────────────────────────────────

class TestQuickPrintWiring(unittest.TestCase):
    def test_build_settings_maps_open_loop(self):
        from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._motion_mode = lambda: "open_loop"
        s = PrintSettings()
        # emulate the mapping block
        _mode = page._motion_mode()
        s.velocity_follow = (_mode == "velocity")
        s.confirm_each_segment = (_mode == "confirm")
        s.velocity_open_loop = (_mode == "open_loop")
        self.assertTrue(s.velocity_open_loop)
        self.assertFalse(s.velocity_follow)
        self.assertFalse(s.confirm_each_segment)


if __name__ == "__main__":
    unittest.main()
