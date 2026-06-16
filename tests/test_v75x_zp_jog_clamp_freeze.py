"""
v7.5.x tests — ZP jog clamp freeze fix.

SAFETY/RELIABILITY BUG: while continuously jogging a ZP axis against a
soft limit, the board froze and the poller dropped the connection. The
soft-limit clamp (logged as "Z clamped: … → …") leaves a tiny residual
delta (boundary − current ≈ a few µm of float noise). The old
`ZPStage.move_relative` gate (`abs(d) > 1e-6`) let that residual through,
so the 8 Hz jog loop emitted one micro-move EVERY segment, forever, while
the stick was held at the limit. Worse, raw `f"{d}"` formatted those tiny
values in scientific notation ("Z9e-05"), which not all G-code parsers
accept. The stream of un-acked / malformed commands froze the board.

Fix (`ZPStage.move_relative` / `move_absolute`):
  * drop sub-resolution moves (`abs(d) >= 1e-4` mm = 0.1 µm), so a
    fully-clamped jog sends NOTHING (loop idles instead of flooding);
  * fixed-decimal formatting (never scientific notation);
  * floor feedrate to ≥ 1 mm/min (an "F0" stalls Marlin's planner — a
    sibling freeze path).

Covered here:
  1. A tiny clamp residual produces NO command.
  2. A real move IS sent, fixed-decimal, no scientific notation.
  3. No emitted G-code ever contains an exponent ('e'/'E' from a number).
  4. Feedrate is never emitted as F0 (floored), incl. None/0/negative.
  5. move_absolute uses fixed-decimal formatting too.
"""

import importlib
import re
import threading
import unittest

from SupportClasses.ZPStage import ZPStageManager

zp_mod = importlib.import_module(ZPStageManager.__module__)

# Match a number written in scientific notation, e.g. 9e-05, 1.5E+3.
_SCI_RE = re.compile(r"\d[eE][+-]?\d")


class _RecordingSerial:
    def __init__(self):
        self.is_open = True
        self.writes: list[str] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data.decode("utf-8", errors="replace"))

    def flush(self) -> None:
        pass

    def close(self) -> None:  # used by __del__ → stop() in teardown
        self.is_open = False


def _bare_zp():
    zp = ZPStageManager.__new__(ZPStageManager)
    zp.serial = _RecordingSerial()
    zp._serial_lock = threading.RLock()
    zp.feedrate = 1000.0
    zp.simulate = False  # so __del__ → stop() doesn't AttributeError in teardown
    return zp


def _g0_writes(zp):
    """Just the G0 motion commands (strip the trailing newline)."""
    return [w.strip() for w in zp.serial.writes if w.strip().startswith("G0")]


class TestMoveRelativeClampResidual(unittest.TestCase):
    def test_subresolution_residual_sends_nothing(self):
        zp = _bare_zp()
        for tiny in (1.5e-05, 9e-06, -2.7e-05, 5e-05):
            zp.move_relative({"Z": tiny}, 600)
        self.assertEqual(zp.serial.writes, [],
                         "sub-resolution clamp residuals must not be sent")

    def test_real_move_is_sent(self):
        zp = _bare_zp()
        zp.move_relative({"Z": 0.012}, 600)
        sent = _g0_writes(zp)
        self.assertEqual(len(sent), 1)
        self.assertIn("Z0.0120", sent[0])

    def test_threshold_boundary(self):
        zp = _bare_zp()
        zp.move_relative({"Z": 1e-4}, 600)   # exactly at threshold → sent
        zp.move_relative({"Z": 9e-5}, 600)   # just below → dropped
        sent = _g0_writes(zp)
        self.assertEqual(len(sent), 1, "only the >= 1e-4 move should be sent")
        self.assertIn("Z0.0001", sent[0])


class TestNoScientificNotation(unittest.TestCase):
    def test_no_exponent_in_any_command(self):
        zp = _bare_zp()
        # Values that the old raw f"{d}" rendered as sci-notation but are
        # large enough to be sent after rounding.
        for d in (0.00012, 0.000345, 1.2345, -0.0009):
            zp.move_relative({"Z": d}, 600)
        for w in zp.serial.writes:
            self.assertIsNone(_SCI_RE.search(w),
                              f"scientific notation leaked into G-code: {w!r}")

    def test_move_absolute_fixed_decimal(self):
        zp = _bare_zp()
        zp.move_absolute({"Z": 0.000123, "E": 12.5}, feedrate_mm_min=300)
        joined = "".join(zp.serial.writes)
        self.assertIsNone(_SCI_RE.search(joined),
                          f"scientific notation in move_absolute: {joined!r}")
        self.assertIn("Z0.0001", joined)
        self.assertIn("E12.5000", joined)


class TestFeedrateFloor(unittest.TestCase):
    def _feedrate_of(self, cmd):
        m = re.search(r"F(\d+(?:\.\d+)?)", cmd)
        return float(m.group(1)) if m else None

    def test_zero_feedrate_floored(self):
        zp = _bare_zp()
        zp.move_relative({"Z": 0.5}, 0)          # F0 would stall Marlin
        fr = self._feedrate_of(_g0_writes(zp)[0])
        self.assertGreaterEqual(fr, 1.0, "feedrate must never be emitted as F0")

    def test_negative_feedrate_floored(self):
        zp = _bare_zp()
        zp.move_relative({"Z": 0.5}, -50)
        fr = self._feedrate_of(_g0_writes(zp)[0])
        self.assertGreaterEqual(fr, 1.0)

    def test_none_feedrate_uses_default(self):
        zp = _bare_zp()
        zp.move_relative({"Z": 0.5}, None)
        fr = self._feedrate_of(_g0_writes(zp)[0])
        self.assertEqual(fr, 1000.0)  # zp.feedrate


if __name__ == "__main__":
    unittest.main()
