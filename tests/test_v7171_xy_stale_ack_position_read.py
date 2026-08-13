"""v7.17.1 — the Prior stale-``R`` ack that corrupts every position read.

From the operator's log (2026-08-13 15:51:43), after two moves each preceded
by a speed setter::

    set_velocity: 10000 µm/s → SMS 20%
    XY relative move: (5000, 0)
    set_velocity: 10000 µm/s → SMS 20%
    XY relative move: (5000, 0)
    Failed to parse XY position: Expected 3 values, got 1: R

The Prior acks EVERY command with a bare ``R``. ``get_current_position``
already flushes the RX buffer before querying, but a flush can only discard
bytes that have already ARRIVED — an ack still in flight lands after it, and
becomes the first line read back after ``P``.

A failed parse returns ``(None, None, None)``, which clears the poller's
``_last_position_read_ok``, makes ``wait_for_xy_arrival`` poll garbage until it
times out, and leaves every cached-position reader stale. That is what "the
software got very slow" is made of.

⚠ These drive the REAL ``XYStageManager`` methods against a fake serial port.
The fix is deliberately read-side only: ``_send_protocol_command`` carries a
v7.5.x note that draining the ack after each WRITE was tried on hardware and
made things worse, so a test that let the write path change would be checking
the wrong thing.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.XYStage import XYStageManager      # noqa: E402


class _FakeSerial:
    """Minimal pyserial surface: a scripted RX stream, CR-terminated."""

    def __init__(self, lines):
        # Each entry is one CR-terminated reply the port will hand back.
        self._buf = bytearray()
        for ln in lines:
            self._buf += ln.encode() + b"\r"
        self.timeout = 0.1
        self.written = []
        self.flushes = 0
        self.reads = 0

    # -- reads -------------------------------------------------------
    def read(self, n=1):
        self.reads += 1
        if not self._buf:
            return b""
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def reset_input_buffer(self):
        # Models the REAL bug: bytes already arrived are dropped, but anything
        # still in flight is not. The scripted stream is what arrives AFTER.
        self.flushes += 1

    # -- writes ------------------------------------------------------
    def write(self, data):
        self.written.append(bytes(data))
        return len(data)

    def flush(self):
        pass

    def close(self):
        pass

    @property
    def is_open(self):
        return True


def _stage(lines):
    """A real XYStageManager wired to a fake port, no hardware, no detection."""
    m = XYStageManager.__new__(XYStageManager)
    import threading
    m.simulate = False
    m._serial_lock = threading.RLock()
    m.spo = _FakeSerial(lines)
    m._protocol = None          # protocol-less == Prior CSV defaults, ack "R"
    return m


class TestTheReportedFailure(unittest.TestCase):

    def test_a_stale_R_no_longer_becomes_a_failed_parse(self):
        """The exact operator symptom, reproduced end to end."""
        st = _stage(["R", "1000,2000,0"])
        self.assertEqual(st.get_current_position(), (1000.0, 2000.0, 0.0))

    def test_the_naive_reader_really_would_have_failed(self):
        """Guard the guard: proves the fixture reproduces the defect, so the
        test above cannot pass merely because nothing was ever wrong."""
        st = _stage(["R", "1000,2000,0"])
        from SupportClasses.XYStage import _read_response_cr
        first = _read_response_cr(st.spo, timeout=0.1)
        self.assertEqual(first.strip(), "R")
        self.assertEqual(st._parse_position_response(first), (None, None, None))

    def test_several_queued_acks_are_all_skipped(self):
        """Two moves each preceded by a setter = four acks in the log."""
        st = _stage(["R", "R", "R", "R", "1000,2000,0"])
        self.assertEqual(st.get_current_position(), (1000.0, 2000.0, 0.0))


class TestItCannotSwallowRealData(unittest.TestCase):

    def test_a_position_reply_is_never_mistaken_for_an_ack(self):
        st = _stage(["1000,2000,0"])
        self.assertEqual(st.get_current_position(), (1000.0, 2000.0, 0.0))

    def test_a_trailing_R_on_a_position_line_still_parses(self):
        """Prior appends R to the position line itself on some firmwares —
        that is payload, not a bare ack, and the parser strips it."""
        st = _stage(["1000,2000,0R"])
        self.assertEqual(st.get_current_position(), (1000.0, 2000.0, 0.0))

    def test_a_ludl_position_reply_is_not_an_ack(self):
        """`:A` alone is an ack; `:A 399 321` is a POSITION. Exact-match, not
        prefix-match, is what keeps those apart."""
        st = _stage([])
        class _P:
            ack_success_token = ":A"
        st._protocol = _P()
        self.assertTrue(st._is_bare_ack(":A"))
        self.assertFalse(st._is_bare_ack(":A 399 321"))


class TestItCannotHangThePoller(unittest.TestCase):

    def test_an_endless_ack_stream_gives_up(self):
        """A controller answering only acks must not spin the poll thread."""
        st = _stage(["R"] * 200)
        self.assertEqual(st.get_current_position(), (None, None, None))

    def test_it_stops_within_the_ack_cap(self):
        st = _stage(["R"] * 200)
        st.get_current_position()
        consumed = 200 - (len(st.spo._buf) // 2)
        self.assertLessEqual(
            consumed, st._MAX_STALE_ACKS + 1,
            "the reader kept draining past its cap")

    def test_silence_is_reported_unchanged(self):
        """No reply must still read as 'no answer', so the existing
        liveness/disconnect handling above is untouched."""
        st = _stage([])
        self.assertEqual(st.get_current_position(), (None, None, None))

    def test_silence_returns_at_once_instead_of_burning_the_budget(self):
        """A dead port must cost ONE read, not a full skip loop.

        Asserting only on the (None,None,None) result cannot see this: an
        empty reply misclassified as an ack produces the same answer, just
        after spinning the whole timeout — on every poll, on a board that has
        gone quiet, which is precisely when the app must stay responsive.
        """
        st = _stage([])
        st.get_current_position()
        self.assertEqual(
            st.spo.reads, 1,
            "a silent port was polled repeatedly inside one position query")

    def test_the_query_is_still_actually_sent(self):
        st = _stage(["R", "1000,2000,0"])
        st.get_current_position()
        self.assertTrue(
            any(b"P" in w for w in st.spo.written),
            "the position query never went out")

    def test_the_pre_read_flush_is_still_performed(self):
        """Belt and braces: the flush handles already-arrived backlog, this
        fix handles the in-flight ack. Both are needed."""
        st = _stage(["1000,2000,0"])
        st.get_current_position()
        self.assertEqual(st.spo.flushes, 1)


if __name__ == "__main__":
    unittest.main()
