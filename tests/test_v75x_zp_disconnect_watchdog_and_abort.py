"""test_v75x_zp_disconnect_watchdog_and_abort.py

Second-round ZP disconnect-during-print fixes (after the poller-suspend /
read-hardening round). Log evidence (logs/prints/*.jsonl) showed ZP dropping
mid-/end-print even with the poller suspended during PRINT_PATH — pointing at
the INDEPENDENT port-health watchdog, which fired on the FIRST transient
`check_port_health` failure (`serial.in_waiting` can raise intermittently under
a print's heavy write load on Windows). And the print then silently dry-ran to a
bogus "completed" against the dead board.

Covered here:
- A) `ConnectionWatchdog` debounce — a single failed health check does NOT
     disconnect; it takes `fail_threshold` CONSECUTIVE failures; a healthy check
     resets the window.
- B) `PrintManager._execute_loop` ABORTS + logs `zp_disconnected` when ZP (present
     at start) drops mid-print, instead of dry-running; and does NOT abort when ZP
     was never present at start.
- C) `_execute_print_path` stops the path immediately on a mid-path ZP drop.
"""

import unittest
from unittest.mock import MagicMock, patch

from SupportClasses.SerialUtils import ConnectionWatchdog
from SupportClasses.PrintManager import (
    PrintManager, PrintCommand, CommandType, PrintSettings,
    PrintState, build_well_plate_job,
)


# ── A) Watchdog debounce ─────────────────────────────────────────────

class _FakePort:
    """check_port_health reads is_open + in_waiting; in_waiting raises when
    unhealthy (mimics a transient Windows ClearCommError under write load)."""

    def __init__(self):
        self.is_open = True
        self.healthy = True

    @property
    def in_waiting(self):
        if not self.healthy:
            raise OSError("transient in_waiting failure")
        return 0


class TestWatchdogDebounce(unittest.TestCase):
    def _wd(self, threshold):
        wd = ConnectionWatchdog(check_interval=999)  # never auto-runs
        port = _FakePort()
        fired = []
        wd.watch("ZP", lambda: port, lambda: fired.append(True),
                 fail_threshold=threshold)
        return wd, port, fired

    def test_single_transient_failure_does_not_disconnect(self):
        wd, port, fired = self._wd(threshold=3)
        wd._run_one_cycle()              # healthy → was_connected
        port.healthy = False
        wd._run_one_cycle()              # 1 fail
        self.assertEqual(fired, [])

    def test_disconnect_after_threshold_consecutive_failures(self):
        wd, port, fired = self._wd(threshold=3)
        wd._run_one_cycle()              # healthy
        port.healthy = False
        wd._run_one_cycle()              # 1
        wd._run_one_cycle()              # 2
        self.assertEqual(fired, [])
        wd._run_one_cycle()              # 3 → disconnect
        self.assertEqual(fired, [True])
        # Does not fire again on further failures (one edge per disconnect).
        wd._run_one_cycle()
        self.assertEqual(fired, [True])

    def test_healthy_check_resets_the_window(self):
        wd, port, fired = self._wd(threshold=3)
        wd._run_one_cycle()              # healthy
        port.healthy = False
        wd._run_one_cycle()              # 1
        wd._run_one_cycle()              # 2
        port.healthy = True
        wd._run_one_cycle()              # reset
        port.healthy = False
        wd._run_one_cycle()              # 1 (fresh)
        wd._run_one_cycle()              # 2
        self.assertEqual(fired, [])      # never reached 3 consecutive


# ── B) PrintManager aborts (not dry-runs) on mid-print ZP disconnect ──

class TestAbortOnZpDisconnect(unittest.TestCase):
    def _job(self):
        return build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0)],
            settings=PrintSettings(num_layers=1, travel_z_height=7.0),
            return_home=False,
        )

    def _pm(self, ctrl):
        pm = PrintManager(ctrl)
        pm._stop_recorder = MagicMock()
        pm._record_history = MagicMock()
        pm.exec_logger = MagicMock()
        pm.load_job(self._job())
        return pm

    def _run(self, pm):
        with patch("SupportClasses.PrintManager.save_print_progress"), \
                patch("SupportClasses.PrintManager.clear_print_progress"):
            pm._execute_loop()

    def test_aborts_and_logs_when_zp_drops_mid_print(self):
        ctrl = MagicMock()
        ctrl.is_zp_connected = True
        pm = self._pm(ctrl)
        # Drop ZP the moment the first command executes.
        def drop(_cmd):
            ctrl.is_zp_connected = False
        pm._execute_command = MagicMock(side_effect=drop)
        self._run(pm)
        self.assertEqual(pm.state, PrintState.ERROR)
        kinds = [c.args[0] for c in pm.exec_logger.log.call_args_list if c.args]
        self.assertIn("zp_disconnected", kinds)

    def test_no_abort_when_zp_absent_at_start(self):
        # A job intentionally run without ZP (e.g. XY-only sim) must NOT abort.
        ctrl = MagicMock()
        ctrl.is_zp_connected = False
        pm = self._pm(ctrl)
        pm._execute_command = MagicMock()
        self._run(pm)
        self.assertEqual(pm.state, PrintState.COMPLETED)


# ── C) PRINT_PATH stops immediately on a mid-path ZP drop ────────────

class TestPrintPathStopsOnDisconnect(unittest.TestCase):
    def test_print_path_halts_when_zp_disconnected(self):
        ctrl = MagicMock()
        ctrl.is_zp_connected = False  # dropped before/at the path
        pm = PrintManager(ctrl)
        pm._zp_connected_at_start = True   # was present at job start
        pm.job = MagicMock()
        pm.job.settings = PrintSettings(num_layers=1)
        pm._active_pump = "P1"
        pm.exec_logger = None
        pm._wait_for_xy_settle = MagicMock()
        cmd = PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
                    "pump": "P1", "flow_rate_uL_s": 0.25})
        pm._execute_print_path(cmd)
        # The segment loop bailed at i=1 (before any extrusion) — no pump moves.
        ctrl.move_pump_uL.assert_not_called()

    def test_print_path_runs_when_connected(self):
        ctrl = MagicMock()
        ctrl.is_zp_connected = True
        pm = PrintManager(ctrl)
        pm._zp_connected_at_start = True
        pm.job = MagicMock()
        pm.job.settings = PrintSettings(num_layers=1)
        pm._active_pump = "P1"
        pm.exec_logger = None
        pm._wait_for_xy_settle = MagicMock()
        with patch("SupportClasses.PrintManager.time.sleep"):
            cmd = PrintCommand(
                type=CommandType.PRINT_PATH,
                params={"points": [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                        "pump": "P1", "flow_rate_uL_s": 0.25})
            pm._execute_print_path(cmd)
        # Connected → the path actually extrudes along its segments.
        self.assertGreater(ctrl.move_pump_uL.call_count, 0)


if __name__ == "__main__":
    unittest.main()
