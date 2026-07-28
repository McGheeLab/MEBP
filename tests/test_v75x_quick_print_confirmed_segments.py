"""test_v75x_quick_print_confirmed_segments.py — confirmed per-segment printing.

Root cause (from a real Quick Print run's execution log): PRINT_PATH streams each
XY segment OPEN-LOOP, paced only by ``time.sleep(seg_len/print_speed)``. On a stage
whose real throughput is slower than the commanded speed (Prior SMS% mis-calibrated
/ short accel-dominated segments), the physical stage falls PROGRESSIVELY behind the
commanded stream (observed lag grew to ~6-10 mm across a ~10 mm print) while the pump
keeps extruding on the fast software schedule → the deposited pattern smears.

Fix: ``PrintSettings.confirm_each_segment`` — when set, ``_execute_print_path`` WAITS
for the stage to physically arrive (± ``_SEGMENT_SETTLE_TOL_UM``) and drains the pump
board (M400), so the stream can never outrun the stage. Quick Print sets it True.

v7.5.x (corner-only): confirm now STOPS (wait-for-arrival + pump drain) only at real
CORNERS (turn angle > ``confirm_corner_angle_deg``) and the final point — straight
edges STREAM (a short paced sleep, like the open path). So a star stops at its tips
and prints each straight edge as one continuous move, instead of crawling at every
sampled node. This test locks that behaviour; the default (open-loop) path is
unchanged.
"""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType,
)


class _ZP:
    def __init__(self, outer):
        self._outer = outer
        self.axis_map = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}

    def flush_moves(self, timeout_s=10.0):
        self._outer.flush_calls += 1
        return True

    def move_relative(self, axes, feedrate=None):
        pass


class _Ctrl:
    def __init__(self):
        self.is_zp_connected = True
        self.flush_calls = 0
        self.xy_calls = 0
        self.zp_stage = _ZP(self)
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.safety_limits = SimpleNamespace(enabled=False)

    def move_pump_uL(self, pump, vol, rate):
        pass

    def move_xy_absolute(self, x, y, from_zero_ref=True):
        self.xy_calls += 1


def _straight(n):
    return [(float(i), 0.0) for i in range(n)]     # 1 mm segments, no corners


def _l_shape():
    # Straight run along +X, a 90° corner at index 3, then straight up +Y.
    return [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0),
            (3.0, 1.0), (3.0, 2.0), (3.0, 3.0)]     # corner @3, last @6


class _Harness:
    """Runs a PRINT_PATH and records every _wait_for_xy_settle call."""

    def run(self, pts, confirm, flow=0.25, angle=30.0):
        ctrl = _Ctrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            print_speed_mm_s=5.0, print_feedrate=200.0,
            confirm_each_segment=confirm, confirm_corner_angle_deg=angle))
        pm.exec_logger = None
        pm._set_xy_speed_for_print = lambda *a, **k: None
        self.settle_calls = []
        pm._wait_for_xy_settle = (
            lambda x, y, timeout=3.0, tolerance=50:
            self.settle_calls.append((x, y, tolerance)))
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
            "points": list(pts), "pump": "P2",
            "flow_rate_uL_s": flow, "flow_rate": 0.01})
        with patch("time.sleep") as slept:
            pm._execute_print_path(cmd)
        self.slept = slept
        self.ctrl = ctrl
        return pm


class TestConfirmedSegments(unittest.TestCase):
    def test_confirmed_stops_only_at_corners(self):
        h = _Harness()
        h.run(_l_shape(), confirm=True)           # corner @ (3,0), last @ (3,3)
        # settle = move-to-start (1) + corner stop + last-point stop + end drain (1)
        self.assertEqual(len(h.settle_calls), 4)
        in_loop_stops = [c[:2] for c in h.settle_calls[1:-1]]
        self.assertEqual(in_loop_stops, [(3.0, 0.0), (3.0, 3.0)])

    def test_confirmed_streams_straight_edges(self):
        h = _Harness()
        h.run(_straight(5), confirm=True)         # no corners
        # only start + the final-point stop + end-of-path drain = 3
        self.assertEqual(len(h.settle_calls), 3)
        # straight edges are streamed → time.sleep IS used
        self.assertTrue(h.slept.called)

    def test_confirmed_uses_segment_tolerance_at_corner(self):
        h = _Harness()
        h.run(_l_shape(), confirm=True)
        stop_waits = h.settle_calls[1:-1]         # exclude start + end drain
        self.assertTrue(stop_waits)
        for _x, _y, tol in stop_waits:
            self.assertEqual(tol, PrintManager._SEGMENT_SETTLE_TOL_UM)

    def test_confirmed_drains_pump_at_corners_only(self):
        h = _Harness()
        h.run(_l_shape(), confirm=True)           # corner @3 + last @6
        # drains at the 2 stop nodes only (short edges → no periodic barrier)
        self.assertEqual(h.ctrl.flush_calls, 2)

    def test_open_loop_default_unchanged(self):
        h = _Harness()
        h.run(_straight(25), confirm=False)       # 24 segments
        # default: settle only at start + end-of-path drain
        self.assertEqual(len(h.settle_calls), 2)
        # periodic barrier every 8 → i = 8,16,24 → 3 flushes
        self.assertEqual(h.ctrl.flush_calls, 3)
        # and it paces via time.sleep
        self.assertTrue(h.slept.called)

    def test_default_flag_is_false(self):
        self.assertFalse(PrintSettings().confirm_each_segment)


class TestQuickPrintEnablesConfirm(unittest.TestCase):
    """Quick Print's Motion-mode selector = 'confirm' must enable the stop-and-go
    confirmed-segment mode (and NOT velocity). (Default is open_loop; velocity is
    covered in test_v75x_quick_print_velocity_follow.)"""

    def test_confirm_mode_enables_confirmed_segments(self):
        from gui.pages.workflows import quick_print_workflow as qpw
        # Build a bare page instance without running __init__ (GUI-free), then
        # drive _build_settings with the minimal collaborators it reads.
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
        page._motion_mode = lambda: "confirm"

        settings = page._build_settings()
        self.assertTrue(settings.confirm_each_segment)
        self.assertFalse(settings.velocity_follow)


if __name__ == "__main__":
    unittest.main()
