"""
test_v75x_gentle_retract_slow_lift.py — every print-execution retract lifts the
needle's first ~1 mm SLOWLY (so a deposited bead can't peel off with the needle
at high speed), then finishes at the fast retract feedrate.

Covers the shared StageController primitive (_retract_z_slow_then_fast) and its
use by ensure_retracted_to / safe_travel_to, plus the PrintManager TRAVEL_UP and
hybrid DirectCommandExecutor raise sites that funnel through them.

No GUI / hardware required.
"""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.StageController import StageController


def _ctrl(slow_dist=1.0, slow_fr=60.0, z_up_sign=-1.0, retract_fr=250.0,
          insert_fr=100.0):
    """Minimal real StageController (no __init__) with the slow-lift attrs."""
    c = StageController.__new__(StageController)
    c.xy_stage = MagicMock()
    c.zp_stage = MagicMock()
    c.zp_stage.flush_moves = MagicMock(return_value=True)
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0, "P2": 0, "P3": 0}
    c._pos_poller = MagicMock()
    c._zp_retract_feedrate = retract_fr
    c._zp_insert_feedrate = insert_fr
    c._min_travel_z_mm = None
    c._z_up_sign = z_up_sign
    c._retract_slow_dist_mm = slow_dist
    c._retract_slow_feedrate = slow_fr
    c.move_z_absolute = MagicMock()
    c.move_xy_absolute = MagicMock()
    c.wait_for_z_arrival = MagicMock(return_value=True)
    c.wait_for_xy_arrival = MagicMock(return_value=True)
    return c


class TestSlowThenFastPrimitive(unittest.TestCase):
    """_retract_z_slow_then_fast: slow first mm on a lift, single move otherwise."""

    def test_lift_does_slow_then_fast(self):
        c = _ctrl()
        # ZDIR=-1: needle DOWN at print raw +25 (height -25); target raw 0
        # (height 0) is a +25 mm lift.
        ok = c._retract_z_slow_then_fast(25.0, 0.0, 250.0, 15.0)
        self.assertTrue(ok)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 2)
        # 1st: slow intermediate = 1 mm above current in the HEIGHT frame.
        # height -24 → zero-ref raw = -24 * z_up_sign(-1) = 24.
        self.assertAlmostEqual(calls[0].args[0], 24.0, places=6)
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 60.0)
        # 2nd: fast to the final target at the retract feedrate.
        self.assertAlmostEqual(calls[1].args[0], 0.0, places=6)
        self.assertEqual(calls[1].kwargs["feedrate_mm_min"], 250.0)

    def test_descent_is_single_move(self):
        c = _ctrl()
        # Target raw +25 (height -25) is BELOW current raw 0 (height 0): a
        # descent — no slow lead-in, just one move.
        c._retract_z_slow_then_fast(0.0, 25.0, 100.0, 15.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)
        self.assertAlmostEqual(c.move_z_absolute.call_args.args[0], 25.0)

    def test_short_lift_is_entirely_slow(self):
        c = _ctrl(slow_dist=5.0)
        # Lift of only 2 mm (< slow_dist) → the whole lift is the slow segment;
        # the final move is the (redundant, same-Z) confirm.
        c._retract_z_slow_then_fast(2.0, 0.0, 250.0, 15.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 60.0)   # slow
        # Intermediate reaches the target (height 0 → raw 0).
        self.assertAlmostEqual(calls[0].args[0], 0.0, places=6)

    def test_slow_dist_zero_is_single_move(self):
        c = _ctrl(slow_dist=0.0)
        c._retract_z_slow_then_fast(25.0, 0.0, 250.0, 15.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)

    def test_no_current_z_is_single_move(self):
        c = _ctrl()
        c._retract_z_slow_then_fast(None, 0.0, 250.0, 15.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)

    def test_returns_false_on_arrival_timeout(self):
        c = _ctrl()
        c.wait_for_z_arrival = MagicMock(return_value=False)
        self.assertFalse(c._retract_z_slow_then_fast(25.0, 0.0, 250.0, 15.0))

    def test_plus_one_polarity_lift(self):
        # ZDIR=+1 machine: higher raw = higher needle. Lift from raw 2 → 12.
        c = _ctrl(z_up_sign=1.0)
        c._retract_z_slow_then_fast(2.0, 12.0, 250.0, 15.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 2)
        self.assertAlmostEqual(calls[0].args[0], 3.0, places=6)   # +1 mm
        self.assertAlmostEqual(calls[1].args[0], 12.0, places=6)


class TestEnsureRetractedGentle(unittest.TestCase):
    def test_below_target_lifts_slow_then_fast(self):
        c = _ctrl()
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        self.assertTrue(c.ensure_retracted_to(0.0))
        self.assertEqual(c.move_z_absolute.call_count, 2)   # slow + fast

    def test_already_above_no_motion(self):
        c = _ctrl()
        c.get_zp_position = MagicMock(return_value=(-10.0, 0.0, -10.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=-10.0)
        self.assertTrue(c.ensure_retracted_to(0.0))
        c.move_z_absolute.assert_not_called()


class TestSafeTravelGentle(unittest.TestCase):
    def test_raise_is_slow_then_fast_then_descent(self):
        c = _ctrl()
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        ok = c.safe_travel_to(5000.0, 10000.0, safe_z_mm=0.0, target_z_mm=25.0)
        self.assertTrue(ok)
        calls = c.move_z_absolute.call_args_list
        # slow lift + fast lift (to safe) + descent to target = 3 Z moves.
        self.assertEqual(len(calls), 3)
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 60.0)     # slow
        self.assertEqual(calls[1].kwargs["feedrate_mm_min"], 250.0)    # fast
        self.assertEqual(calls[2].kwargs["feedrate_mm_min"], 100.0)    # insert
        c.move_xy_absolute.assert_called_once()


class TestSetterAndStubsLegacy(unittest.TestCase):
    def test_setter(self):
        c = _ctrl()
        c.set_retract_slow_lift(2.5, 120.0)
        self.assertEqual(c._retract_slow_dist_mm, 2.5)
        self.assertEqual(c._retract_slow_feedrate, 120.0)
        c.set_retract_slow_lift(0.0)            # disable, keep feedrate
        self.assertEqual(c._retract_slow_dist_mm, 0.0)
        self.assertEqual(c._retract_slow_feedrate, 120.0)

    def test_new_stub_without_attr_is_legacy_single_move(self):
        # A __new__ controller lacking _retract_slow_dist_mm (the existing test
        # fakes) must keep the legacy single-move retract (getattr default 0).
        c = StageController.__new__(StageController)
        c.zp_stage = MagicMock()
        c.zp_stage.flush_moves = MagicMock(return_value=True)
        c._z_up_sign = -1.0
        c.move_z_absolute = MagicMock()
        c.wait_for_z_arrival = MagicMock(return_value=True)
        c.zero_position = {"Z": 0.0}
        c._retract_z_slow_then_fast(25.0, 0.0, 250.0, 15.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)


class TestPrintManagerTravelUpGentle(unittest.TestCase):
    def _pm(self):
        from SupportClasses.PrintManager import PrintManager
        ctrl = MagicMock()
        ctrl._zp_retract_feedrate = 500.0
        ctrl.is_zp_connected = True
        pm = PrintManager(ctrl)
        pm.job = MagicMock()
        pm.job.settings = SimpleNamespace(travel_z_height=39.59,
                                          print_z_height=18.44)
        pm.exec_logger = None
        return pm, ctrl

    def test_travel_up_uses_ensure_retracted(self):
        from SupportClasses.PrintManager import PrintCommand, CommandType
        pm, ctrl = self._pm()
        pm._execute_command(PrintCommand(type=CommandType.TRAVEL_UP))
        ctrl.ensure_retracted_to.assert_called_once_with(39.59)


class TestHybridRaiseZGentle(unittest.TestCase):
    def test_raise_z_delegates_to_ensure_retracted(self):
        from SupportClasses.PrintManager import DirectCommandExecutor
        ctrl = MagicMock()
        ctrl.is_zp_connected = True
        ctrl.ensure_retracted_to = MagicMock(return_value=True)
        direct = DirectCommandExecutor(ctrl)
        self.assertTrue(direct.raise_z(12.0, feedrate_mm_min=300.0))
        ctrl.ensure_retracted_to.assert_called_once()
        self.assertEqual(ctrl.ensure_retracted_to.call_args.args[0], 12.0)

    def test_raise_from_well_uses_raise_z(self):
        from SupportClasses.PrintManager import DirectCommandExecutor
        ctrl = MagicMock()
        ctrl.is_zp_connected = True
        ctrl.ensure_retracted_to = MagicMock(return_value=True)
        direct = DirectCommandExecutor(ctrl)
        direct.raise_from_well(SimpleNamespace(travel_z_height=30.0,
                                               fast_z_feedrate_mm_min=400.0))
        ctrl.ensure_retracted_to.assert_called_once()
        self.assertEqual(ctrl.ensure_retracted_to.call_args.args[0], 30.0)

    def test_raise_z_fallback_to_move_z(self):
        from SupportClasses.PrintManager import DirectCommandExecutor
        ctrl = MagicMock()
        ctrl.is_zp_connected = True
        del ctrl.ensure_retracted_to        # older controller
        ctrl.wait_for_z_arrival = MagicMock(return_value=True)
        direct = DirectCommandExecutor(ctrl)
        self.assertTrue(direct.raise_z(12.0, feedrate_mm_min=300.0))
        ctrl.move_z_absolute.assert_called_once()


if __name__ == "__main__":
    unittest.main()
