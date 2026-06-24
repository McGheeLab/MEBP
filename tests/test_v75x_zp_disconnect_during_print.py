"""test_v75x_zp_disconnect_during_print.py — ZP disconnect-during-print fixes.

Root cause (proven from logs/prints/*.jsonl: cached ZP z went None ~4-8 s into
the PRINT_PATH loop while the job still logged "completed"): the ~2.5 s
PositionPoller liveness watchdog FALSE-POSITIVES a "ZP disconnected" mid-print
because the poller's M114 reads contend with the dense per-segment ZP write
stream and (a) the poller is left active during PRINT_PATH, (b) resume() never
reset the fail window, and (c) get_current_position() did a single non-blocking
read with no RX drain so a busy board looked dead.

Fixes covered here:
- #2  PositionPoller.resume() resets _zp_fail_count (fresh liveness window).
- #1  StageController.suspend/resume_position_poller() helpers (guarded), and
      the discrete PRINT_PATH wraps execution in suspend → ... → resume (finally).
- #3  ZPStage.get_current_position() issues M114 as a synchronous transaction
      (write → read lines until 'ok', parsing the position line), so a busy
      board != a dead board. v7.5.x flow-control Phase 1 superseded the original
      "drain stale acks before M114" approach: every command now consumes its
      own 'ok' (see ZPStage.send_data), so the RX buffer is clean by the time
      M114 runs — no pre-drain needed, and a real silent board still reports a
      failed read for the poller-liveness watchdog.
"""

import threading
import unittest
from unittest.mock import MagicMock

from SupportClasses.StageController import StageController, PositionPoller
from SupportClasses.ZPStage import ZPStageManager
from SupportClasses.PrintManager import PrintManager, PrintCommand, CommandType


# ── #2: resume() resets the liveness fail window ─────────────────────

class TestPollerResumeResetsFailCount(unittest.TestCase):
    def test_resume_resets_fail_count_and_clears_suspend(self):
        p = PositionPoller(poll_interval=0.3)
        p._zp_fail_count = 7
        p._suspended = True
        p.resume()
        self.assertEqual(p._zp_fail_count, 0)
        self.assertFalse(p._suspended)

    def test_threshold_is_about_2p5s_of_polls(self):
        # ~2.5 s / 0.3 s ≈ 8 consecutive failures before declaring a disconnect.
        p = PositionPoller(poll_interval=0.3)
        self.assertEqual(p._zp_fail_threshold, max(5, int(2.5 / 0.3)))

    def test_fresh_window_after_resume_needs_full_threshold_again(self):
        # Simulate the poll loop's failure accounting directly: at threshold-1,
        # a resume() (e.g. after a PRINT_PATH / safe_travel_to) must wipe the
        # count so a single later failure cannot immediately fire on_zp_lost.
        p = PositionPoller(poll_interval=0.3)
        fired = []
        p.on_zp_lost = lambda: fired.append(True)
        p._zp_fail_count = p._zp_fail_threshold - 1
        p.resume()
        # One failed read after resume → count is 1, far below threshold.
        p._zp_fail_count += 1
        self.assertLess(p._zp_fail_count, p._zp_fail_threshold)
        self.assertEqual(fired, [])


# ── #1: poller suspend/resume helpers + PRINT_PATH wrapping ──────────

class TestSuspendResumeHelpers(unittest.TestCase):
    def test_helpers_guarded_when_no_poller(self):
        sc = StageController.__new__(StageController)  # bypass heavy __init__
        # No _pos_poller attribute at all — must not raise.
        sc.suspend_position_poller()
        sc.resume_position_poller()

    def test_helpers_delegate_to_poller(self):
        sc = StageController.__new__(StageController)
        sc._pos_poller = MagicMock()
        sc.suspend_position_poller()
        sc.resume_position_poller()
        sc._pos_poller.suspend.assert_called_once()
        sc._pos_poller.resume.assert_called_once()


class TestPrintPathSuspendsPoller(unittest.TestCase):
    def _pm(self):
        ctrl = MagicMock()
        pm = PrintManager(ctrl)
        pm.job = MagicMock()  # _execute_command reads self.job.settings
        return pm, ctrl

    def test_print_path_suspends_then_resumes(self):
        pm, ctrl = self._pm()
        pm._execute_print_path = MagicMock()
        cmd = PrintCommand(type=CommandType.PRINT_PATH,
                           params={"points": [(0.0, 0.0), (1.0, 1.0)]})
        pm._execute_command(cmd)
        ctrl.suspend_position_poller.assert_called_once()
        ctrl.resume_position_poller.assert_called_once()
        pm._execute_print_path.assert_called_once_with(cmd)

    def test_print_path_resumes_poller_even_on_exception(self):
        pm, ctrl = self._pm()
        pm._execute_print_path = MagicMock(side_effect=RuntimeError("mid-path"))
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params={"points": []})
        with self.assertRaises(RuntimeError):
            pm._execute_command(cmd)
        # finally must still resume — otherwise the live position display freezes.
        ctrl.resume_position_poller.assert_called_once()

    def test_non_print_path_does_not_touch_poller(self):
        pm, ctrl = self._pm()
        # A COMMENT command must not suspend/resume the poller.
        pm._execute_command(PrintCommand(type=CommandType.COMMENT, label="x"))
        ctrl.suspend_position_poller.assert_not_called()
        ctrl.resume_position_poller.assert_not_called()


# ── #3: synchronous M114 transaction (write → read until 'ok') ───────

class _FakeSerial:
    """Line-oriented fake: readline() pops successive scripted lines (the
    synchronous flow-control path reads line-by-line until 'ok'). An empty
    queue returns b"" (a real serial readline timing out on a silent board)."""

    def __init__(self, lines):
        self.is_open = True
        self._lines = list(lines)
        self.reset_calls = 0
        self.writes = []

    def reset_input_buffer(self):
        self.reset_calls += 1

    def write(self, b):
        self.writes.append(b)

    def flush(self):
        pass

    def close(self):
        pass

    def readline(self):
        return self._lines.pop(0) if self._lines else b""


def _zp_with_serial(fake):
    zp = ZPStageManager.__new__(ZPStageManager)  # bypass __init__ (no real port)
    zp.serial = fake
    zp.simulate = False
    zp._serial_lock = threading.RLock()
    zp.x_pos = zp.y_pos = zp.z_pos = zp.e_pos = 0.0
    zp._last_position_read_ok = True
    zp._board_reset_detected = False
    return zp


_M114_LINES = [b"X:1.0 Y:2.0 Z:3.0 E:4.0 Count X:1 Y:2 Z:3\n", b"ok\n"]


class TestGetCurrentPositionSynchronous(unittest.TestCase):
    def test_m114_sent_and_position_parsed(self):
        fake = _FakeSerial(list(_M114_LINES))
        zp = _zp_with_serial(fake)
        pos = zp.get_current_position()
        self.assertTrue(any(b"M114" in w for w in fake.writes))
        self.assertTrue(zp._last_position_read_ok)
        self.assertEqual(pos, (1.0, 2.0, 3.0, 4.0))

    def test_no_predrain_buffer_is_clean_under_flow_control(self):
        # With synchronous flow control every prior command consumed its own
        # 'ok', so the RX buffer is clean — M114 needs NO pre-drain.
        fake = _FakeSerial(list(_M114_LINES))
        zp = _zp_with_serial(fake)
        zp.get_current_position()
        self.assertEqual(fake.reset_calls, 0)

    def test_busy_keepalive_then_reply_is_not_a_dead_board(self):
        # A long move emits 'echo:busy: processing' before the M114 reply —
        # the board is alive, not dead.
        fake = _FakeSerial([b"echo:busy: processing\n"] + list(_M114_LINES))
        zp = _zp_with_serial(fake)
        pos = zp.get_current_position()
        self.assertTrue(zp._last_position_read_ok)
        self.assertEqual(pos, (1.0, 2.0, 3.0, 4.0))

    def test_dead_board_reports_failed_read(self):
        fake = _FakeSerial([])  # nothing ever comes back
        zp = _zp_with_serial(fake)
        zp.get_current_position()
        # The poller-liveness logic still sees the failure (escalates a true
        # power-off), unchanged by the busy-board tolerance.
        self.assertFalse(zp._last_position_read_ok)


if __name__ == "__main__":
    unittest.main()
