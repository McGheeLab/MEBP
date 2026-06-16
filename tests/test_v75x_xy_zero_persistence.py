"""
v7.5.x tests — XY (and ZP) zero-reference persistence across restart.

Reported bug: the Prior ProScan controller stays powered between software
restarts and keeps its absolute position, but the XY readout comes up wrong.
Root cause: the displayed position is ``raw - zero_position`` and
``zero_position`` was only persisted by the explicit "Set Zero" buttons.
Non-button paths that mutate it (the Xbox "zero_needle_pos" -> _calibrate_zero,
auto-calibration) changed it only in RAM, and the clean-shutdown save_settings()
did not persist it — so on restart the reference reverted (often to 0) and the
retained-but-correct raw position was interpreted against the wrong zero.

The fix persists ``zero_position`` on clean shutdown. These tests validate the
save -> restart -> correlate contract that fix relies on:
  1. The zero reference round-trips through Settings (the exact calls
     save_settings now makes, and the main.py restore on startup).
  2. With the reference restored, displayed = raw - zero_position lands back on
     the same zero-referenced value the user saw before closing.
"""

import tempfile
import unittest
from pathlib import Path

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings


def _stop(ctrl):
    try:
        ctrl._pos_poller.stop()
    except Exception:
        pass


class TestZeroPersistenceRoundTrip(unittest.TestCase):
    def setUp(self):
        self._dir = tempfile.mkdtemp()
        self._path = str(Path(self._dir) / "settings.json")

    def tearDown(self):
        import shutil
        shutil.rmtree(self._dir, ignore_errors=True)

    def _persist_like_save_settings(self, ctrl):
        """Mirror the line save_settings() now runs on clean shutdown."""
        s = Settings(self._path)
        s.load()
        s.set_section("zero_position", dict(ctrl.zero_position))
        s.save()

    def _restore_like_main(self, ctrl):
        """Mirror main.py's startup restore (saved_zero -> controller)."""
        s = Settings(self._path)
        s.load()
        saved = s.get_section("zero_position")
        if saved:
            ctrl.zero_position.update(saved)

    def test_xy_zero_survives_restart(self):
        # Session 1: user zeroes XY at absolute (50000, 30000) µm (e.g. via the
        # Xbox button — a path that does NOT itself persist).
        ctrl1 = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl1)
        ctrl1.zero_position["x"] = 50000.0
        ctrl1.zero_position["y"] = 30000.0
        self._persist_like_save_settings(ctrl1)  # clean shutdown

        # Session 2: fresh controller starts at default zero, then restores.
        ctrl2 = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl2)
        self.assertEqual(ctrl2.zero_position["x"], 0.0)  # default before restore
        self._restore_like_main(ctrl2)

        self.assertEqual(ctrl2.zero_position["x"], 50000.0)
        self.assertEqual(ctrl2.zero_position["y"], 30000.0)

    def test_restored_reference_yields_correct_zero_ref_position(self):
        # Before close: zero ref at (50000, 30000); stage jogged to absolute
        # (60000, 35000) -> the user sees zero-ref (10000, 5000) µm = (10, 5) mm.
        ctrl1 = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl1)
        ctrl1.zero_position["x"] = 50000.0
        ctrl1.zero_position["y"] = 30000.0
        ctrl1.get_xy_position = lambda cached=True: (60000.0, 35000.0, 0.0)
        before = ctrl1.get_xy_position_mm()
        self.assertAlmostEqual(before[0], 10.0, places=6)
        self.assertAlmostEqual(before[1], 5.0, places=6)
        self._persist_like_save_settings(ctrl1)

        # After restart: ProScan retained its absolute position (controller
        # stayed powered) -> same raw read. With the reference restored, the
        # zero-ref readout must match what it was before closing.
        ctrl2 = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl2)
        ctrl2.get_xy_position = lambda cached=True: (60000.0, 35000.0, 0.0)
        self._restore_like_main(ctrl2)
        after = ctrl2.get_xy_position_mm()

        self.assertAlmostEqual(after[0], before[0], places=6)
        self.assertAlmostEqual(after[1], before[1], places=6)

    def test_without_restore_position_would_be_wrong(self):
        # Guard: demonstrates the bug's mechanism — if the reference is NOT
        # restored (the pre-fix behavior), the same raw read decodes wrong.
        ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, ctrl)
        ctrl.get_xy_position = lambda cached=True: (60000.0, 35000.0, 0.0)
        # zero_position left at default (0,0) — i.e. reference lost on restart.
        wrong = ctrl.get_xy_position_mm()
        self.assertAlmostEqual(wrong[0], 60.0, places=6)  # absolute, not 10.0
        self.assertNotAlmostEqual(wrong[0], 10.0, places=6)


if __name__ == "__main__":
    unittest.main()
