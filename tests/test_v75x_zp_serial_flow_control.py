"""test_v75x_zp_serial_flow_control.py — Marlin `ok` flow control (Phase 1).

The ZP board is a Marlin controller; Marlin runs prints for hours without
dropping ONLY because reliable hosts wait for its `ok` acknowledgement before
sending the next command. The old `ZPStage.send_data` fired commands blind,
outrunning Marlin's small serial/planner buffers under load → overflow → the
board desyncs/goes silent → our recurring "ZP disconnected".

Phase 1 makes `send_data` synchronous on real hardware: write → read lines until
`ok`, handling `busy` keep-alives, `Error:` replies, mid-session board-reset
banners, and silence. Simulator keeps fire-and-forget (it answers everything and
the suite relies on non-blocking writes).
"""

import threading
import time
import unittest

from SupportClasses.ZPStage import ZPStageManager


class _ScriptedSerial:
    """Real-port stand-in. `readline()` pops scripted response lines (bytes
    ending in \\n); an empty queue returns b"" like a serial readline timing
    out. Records writes so tests can assert what was sent."""

    def __init__(self, lines=None, *, loop_ok=False):
        self.is_open = True
        self._lines = list(lines or [])
        self.loop_ok = loop_ok          # if True, always ack with 'ok'
        self.writes: list[str] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data.decode("utf-8", errors="replace").strip())

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        if self._lines:
            return self._lines.pop(0)
        if self.loop_ok:
            return b"ok\n"
        return b""

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


def _bare_zp(serial):
    zp = ZPStageManager.__new__(ZPStageManager)  # bypass __init__ (no real port)
    zp.serial = serial
    zp.simulate = False
    zp._serial_lock = threading.RLock()
    zp.feedrate = 1000.0
    zp.x_pos = zp.y_pos = zp.z_pos = zp.e_pos = 0.0
    zp._last_position_read_ok = True
    zp._board_reset_detected = False
    return zp


class TestSendDataWaitsForOk(unittest.TestCase):
    def test_returns_true_and_consumes_ok(self):
        ser = _ScriptedSerial([b"ok\n"])
        zp = _bare_zp(ser)
        self.assertTrue(zp.send_data("G0 X1"))
        self.assertEqual(ser.writes, ["G0 X1"])

    def test_returns_false_and_flags_on_silence(self):
        # Board never answers 'ok' → command unconfirmed. The command is still
        # written, but send_data reports failure and clears the liveness flag so
        # the poller escalates to a disconnect.
        ser = _ScriptedSerial([])  # silent
        zp = _bare_zp(ser)
        zp.DEFAULT_OK_TIMEOUT_S = 0.3  # keep the test fast
        t0 = time.monotonic()
        ok = zp.send_data("G0 X1")
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertFalse(zp._last_position_read_ok)
        self.assertIn("G0 X1", ser.writes)        # the write still happened
        self.assertLess(elapsed, 2.0)             # bounded by the ok timeout

    def test_busy_keepalive_then_ok_succeeds(self):
        # A long move emits 'echo:busy: processing' before 'ok' — alive, not dead.
        ser = _ScriptedSerial([b"echo:busy: processing\n", b"ok\n"])
        zp = _bare_zp(ser)
        self.assertTrue(zp.send_data("G0 X50"))

    def test_marlin_error_reply_returns_false(self):
        ser = _ScriptedSerial([b"Error:Unknown command\n"])
        zp = _bare_zp(ser)
        self.assertFalse(zp.send_data("G0 X1"))

    def test_board_reset_banner_detected(self):
        # A mid-session reset (brownout/auto-reset) prints the boot banner while
        # we await 'ok' — detect it, flag it, fail the command.
        ser = _ScriptedSerial([b"start\n"])
        zp = _bare_zp(ser)
        ok = zp.send_data("G0 X1")
        self.assertFalse(ok)
        self.assertTrue(zp._board_reset_detected)
        self.assertFalse(zp._last_position_read_ok)

    def test_wait_ok_false_is_fire_and_forget(self):
        ser = _ScriptedSerial([])  # would time out if it waited
        zp = _bare_zp(ser)
        t0 = time.monotonic()
        self.assertTrue(zp.send_data("M114", wait_ok=False))
        self.assertLess(time.monotonic() - t0, 0.2)  # did NOT wait for ok
        self.assertIn("M114", ser.writes)


class TestAdaptiveTimeoutFailFast(unittest.TestCase):
    """Once the board is known silent, commands must fail FAST (short budget)
    instead of paying the full DEFAULT_OK_TIMEOUT_S each — otherwise a marginal
    board cascades into multi-second hangs across a whole print/travel."""

    def test_silent_board_uses_short_timeout(self):
        ser = _ScriptedSerial(loop_ok=False)  # never acks
        zp = _bare_zp(ser)
        zp.DEFAULT_OK_TIMEOUT_S = 10.0   # would hang 10 s if used
        zp.SILENT_OK_TIMEOUT_S = 0.2
        zp._last_position_read_ok = False  # board already known silent
        t0 = time.monotonic()
        ok = zp.send_data("G0 X1")
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertLess(elapsed, 2.0)  # short budget, not the 10 s default

    def test_ack_restores_full_timeout_flag(self):
        ser = _ScriptedSerial(loop_ok=True)
        zp = _bare_zp(ser)
        zp._last_position_read_ok = False
        self.assertTrue(zp.send_data("G0 X1"))
        # A successful ack proves the board is alive → flag restored.
        self.assertTrue(zp._last_position_read_ok)

    def test_flush_moves_capped_when_silent(self):
        ser = _ScriptedSerial(loop_ok=False)  # M400 never acks
        zp = _bare_zp(ser)
        zp._last_position_read_ok = False
        t0 = time.monotonic()
        ok = zp.flush_moves(timeout_s=15.0)
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertLess(elapsed, 4.0)  # capped to ~2 s, not 15 s

    def test_flush_moves_holds_lock_atomically(self):
        # The M400 round trip must hold _serial_lock for its whole duration so a
        # concurrent reader (straggler poller M114 / jog handler) can't slip in
        # and STEAL the M400 'ok' between reads — the root cause of bogus
        # "M400 timed out" on a live board.
        ser = _ScriptedSerial(loop_ok=False)  # never acks → loops the full wait
        zp = _bare_zp(ser)
        zp._last_position_read_ok = False  # caps the wait
        t = threading.Thread(target=lambda: zp.flush_moves(timeout_s=1.0),
                             daemon=True)
        t.start()
        time.sleep(0.2)  # flush_moves is now mid read-loop
        # A different thread must NOT be able to grab the serial lock mid-flush.
        got = zp._serial_lock.acquire(timeout=0.1)
        if got:
            zp._serial_lock.release()
        self.assertFalse(got, "flush_moves must hold _serial_lock atomically")
        t.join(timeout=3.0)
        # …and the lock is released once it returns.
        got2 = zp._serial_lock.acquire(timeout=0.5)
        self.assertTrue(got2)
        zp._serial_lock.release()


class TestMoveCommandsAreSynchronous(unittest.TestCase):
    def test_move_relative_waits_for_ok(self):
        ser = _ScriptedSerial(loop_ok=True)
        zp = _bare_zp(ser)
        zp.move_relative({"X": 1.5}, feedrate=600)
        g0 = [w for w in ser.writes if w.startswith("G0")]
        self.assertEqual(len(g0), 1)
        self.assertIn("X1.5000", g0[0])

    def test_move_absolute_brackets_with_g90_g91_each_acked(self):
        ser = _ScriptedSerial(loop_ok=True)
        zp = _bare_zp(ser)
        zp.move_absolute({"X": 10.0}, feedrate_mm_min=300)
        self.assertEqual(ser.writes[0], "G90")
        self.assertTrue(ser.writes[1].startswith("G0 X10.0000"))
        self.assertEqual(ser.writes[2], "G91")


class TestGetCurrentPositionSynchronous(unittest.TestCase):
    _M114 = [b"X:1.0 Y:2.0 Z:3.0 E:4.0 Count X:1 Y:2 Z:3\n", b"ok\n"]

    def test_parses_position_and_consumes_ok(self):
        ser = _ScriptedSerial(list(self._M114))
        zp = _bare_zp(ser)
        pos = zp.get_current_position()
        self.assertEqual(pos, (1.0, 2.0, 3.0, 4.0))
        self.assertTrue(zp._last_position_read_ok)
        self.assertIn("M114", ser.writes)

    def test_silent_board_marks_failed_read(self):
        ser = _ScriptedSerial([])
        zp = _bare_zp(ser)
        zp.get_current_position()
        self.assertFalse(zp._last_position_read_ok)


class TestSimulatorStaysFireAndForget(unittest.TestCase):
    def test_simulate_does_not_block_for_ok(self):
        # In simulate mode send_data must NOT try to read 'ok' (the sim answers
        # via its own response queue) — a serial without readline must be fine.
        class _NoReadline:
            is_open = True
            def __init__(self): self.writes = []
            def write(self, d): self.writes.append(d)
            def flush(self): pass
            def stop(self): pass  # __del__ → stop() in teardown (simulate path)
        ser = _NoReadline()
        zp = _bare_zp(ser)
        zp.simulate = True
        self.assertTrue(zp.send_data("G0 X1"))  # no AttributeError, no block
        self.assertTrue(ser.writes)


if __name__ == "__main__":
    unittest.main()
