"""test_v75x_print_path_planner_barrier.py — print-path planner-buffer hardening.

A multi-agent diff (print path vs the passing raw-motion stress components) found
the print-only ZP/USB failure is planner-buffer saturation: PRINT_PATH issues one
pump G0 + one XY G0 per waypoint with NO M400 barrier, paced only by the XY
transit time — so a long, finely-sampled path admits pump moves faster than the
board executes them, the ~16-block planner buffer fills, the board stalls under
flow control, and the CH340 WriteFile faults (ERROR_BAD_COMMAND). The raw-motion
components never fail because each confirms/flushes (drains the buffer) or is
net-zero/short.

Incremental hardening (this change):
  1. M400 barrier every PrintManager._PATH_BARRIER_EVERY segments — bounds buffer
     depth so it can never saturate.
  2. Pace by the rate-limiting axis (max(xy_move_time, pump_move_time, 0.05)).
  3. Suspend the port-health watchdog (not just the poller) for PRINT_PATH so
     nothing but the print thread touches the ZP COM handle during the burst
     (its in_waiting/ClearCommError racing a paused write is a fault surface).
"""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from SupportClasses.SerialUtils import ConnectionWatchdog
from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType,
)


# ── 1. ConnectionWatchdog pause / resume ─────────────────────────────

class TestWatchdogPauseResume(unittest.TestCase):
    def _wd(self):
        wd = ConnectionWatchdog(check_interval=0.01)
        self.port_getter_calls = 0
        def getter():
            self.port_getter_calls += 1
            return SimpleNamespace(in_waiting=0)  # "healthy" port
        wd.watch("ZP", getter, lambda: None, fail_threshold=3)
        return wd

    def test_paused_watch_is_not_health_checked(self):
        wd = self._wd()
        wd._run_one_cycle()                      # not paused → checked
        self.assertEqual(self.port_getter_calls, 1)
        wd.pause("ZP")
        wd._run_one_cycle()                      # paused → skipped
        wd._run_one_cycle()
        self.assertEqual(self.port_getter_calls, 1)
        wd.resume("ZP")
        wd._run_one_cycle()                      # resumed → checked again
        self.assertEqual(self.port_getter_calls, 2)

    def test_resume_resets_fail_window(self):
        wd = self._wd()
        wd._watches["ZP"]["fail_count"] = 2
        wd.pause("ZP")
        wd.resume("ZP")
        self.assertEqual(wd._watches["ZP"]["fail_count"], 0)

    def test_pause_unknown_name_is_safe(self):
        wd = ConnectionWatchdog()
        wd.pause("NOPE")    # must not raise
        wd.resume("NOPE")


# ── 2. PRINT_PATH suspends/resumes BOTH poller and watchdog ──────────

class TestPrintPathIsolatesZpHandle(unittest.TestCase):
    def _pm(self):
        ctrl = MagicMock()
        pm = PrintManager(ctrl)
        pm.job = MagicMock()
        pm._execute_print_path = MagicMock()
        return pm, ctrl

    def test_print_path_suspends_and_resumes_watchdog(self):
        pm, ctrl = self._pm()
        pm._execute_command(PrintCommand(
            type=CommandType.PRINT_PATH, params={"points": [(0, 0), (1, 1)]}))
        ctrl.suspend_zp_watchdog.assert_called_once()
        ctrl.resume_zp_watchdog.assert_called_once()
        ctrl.suspend_position_poller.assert_called_once()
        ctrl.resume_position_poller.assert_called_once()

    def test_watchdog_resumed_even_on_exception(self):
        pm, ctrl = self._pm()
        pm._execute_print_path = MagicMock(side_effect=RuntimeError("mid-path"))
        with self.assertRaises(RuntimeError):
            pm._execute_command(PrintCommand(
                type=CommandType.PRINT_PATH, params={"points": []}))
        ctrl.resume_zp_watchdog.assert_called_once()
        ctrl.resume_position_poller.assert_called_once()


# ── 3. M400 barrier every N segments in _execute_print_path ──────────

class _BarrierZP:
    def __init__(self, outer):
        self._outer = outer
        self.axis_map = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}

    def flush_moves(self, timeout_s=10.0):
        self._outer.flush_calls += 1
        return True

    def move_relative(self, axes, feedrate=None):
        pass


class _BarrierCtrl:
    def __init__(self):
        self.is_zp_connected = True
        self.flush_calls = 0
        self.pump_calls = 0
        self.xy_calls = 0
        self.zp_stage = _BarrierZP(self)
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.safety_limits = SimpleNamespace(enabled=False)

    def move_pump_uL(self, pump, vol, rate):
        self.pump_calls += 1

    def move_xy_absolute(self, x, y, from_zero_ref=True):
        self.xy_calls += 1


class TestPrintPathBarrier(unittest.TestCase):
    def _run_path(self, ctrl, n_points, flow=0.25):
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            print_speed_mm_s=5.0, print_feedrate=200.0))
        pm.exec_logger = None
        pm._set_xy_speed_for_print = lambda *a, **k: None
        pts = [(float(i), 0.0) for i in range(n_points)]  # 1mm segments
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
            "points": pts, "pump": "P2",
            "flow_rate_uL_s": flow, "flow_rate": 0.01})
        with patch("time.sleep"):   # don't actually pace in the test
            pm._execute_print_path(cmd)
        return pm

    def test_barrier_fires_every_n_segments(self):
        ctrl = _BarrierCtrl()
        # 25 points = 24 segments; barrier (every 8) at i = 8, 16, 24 → 3.
        self._run_path(ctrl, 25)
        self.assertEqual(ctrl.flush_calls, 3)

    def test_barrier_skipped_when_zp_disconnected(self):
        ctrl = _BarrierCtrl()
        ctrl.is_zp_connected = False
        self._run_path(ctrl, 25)
        self.assertEqual(ctrl.flush_calls, 0)

    def test_short_path_drains_at_least_once(self):
        # A path shorter than the barrier interval still completes; the buffer
        # is bounded by the path length itself (no barrier needed mid-path).
        ctrl = _BarrierCtrl()
        self._run_path(ctrl, 5)   # 4 segments < 8 → no barrier
        self.assertEqual(ctrl.flush_calls, 0)
        self.assertGreater(ctrl.xy_calls, 0)   # but it did print


# ── 4. End-of-path XY drain (XY/ZP de-sync fix) ──────────────────────

class TestPrintPathDrainsXYAtEnd(unittest.TestCase):
    """The per-segment XY moves are open-loop streamed, so the Prior is still
    draining queued moves when the segment loop ends. _execute_print_path must
    wait for the stage to reach the FINAL path point before returning — else
    the print 'completes' and Z retracts while XY is still moving."""

    def _run(self, pts):
        ctrl = _BarrierCtrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            print_speed_mm_s=5.0, print_feedrate=200.0))
        pm.exec_logger = None
        pm._set_xy_speed_for_print = lambda *a, **k: None
        calls = []
        pm._wait_for_xy_settle = (
            lambda x, y, timeout=3.0, tolerance=50: calls.append((x, y)))
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
            "points": pts, "pump": "P2",
            "flow_rate_uL_s": 0.0, "flow_rate": 0.0})
        with patch("time.sleep"):
            pm._execute_print_path(cmd)
        return calls

    def test_settles_at_final_point_after_loop(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        calls = self._run(pts)
        # move-to-start settle (pts[0]) + the new end-of-path drain (pts[-1])
        self.assertGreaterEqual(len(calls), 2)
        self.assertEqual(calls[0], (0.0, 0.0))     # move to start
        self.assertEqual(calls[-1], (3.0, 0.0))    # end-of-path drain = final pt

    def test_drain_target_is_last_point_not_start(self):
        pts = [(5.0, 5.0), (6.0, 7.0)]
        calls = self._run(pts)
        self.assertEqual(calls[-1], (6.0, 7.0))


if __name__ == "__main__":
    unittest.main()
