"""Tests for v7.5.x — Guarantee Z retract before cross-position XY travel.

CRITICAL SAFETY behavior: any XY move to a DIFFERENT location must first retract
the needle to the safe / "move" Z (and confirm Z arrival) before the XY move —
EXCEPT within-well print-pattern moves. These tests cover:

  * Polarity-safe helpers (z_height_of / needle_at_or_above / default_travel_z),
    correct on both ZDIR=+1 and the ME3B V1 ZDIR=-1 convention.
  * ensure_retracted_to never DESCENDS (so a misconfigured/low target degrades
    to a no-op, never a crash) and raises+waits when the needle is below safe.
  * PrintManager MOVE_XY / HOME_XY self-retract (via _retract_for_travel) before
    the XY move — independent of a separate TRAVEL_UP plan step.
  * The jog / spheroid click-to-travel handlers ALWAYS route through
    safe_travel_to when a Safe Z is known (the old polarity-wrong "skip retract
    when current_z >= safe_z" gate is gone).
  * Quick Print's travel-Z fallback is a polarity-safe lift, not the raw 5.0.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock, Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.StageController import StageController, ZDIR

_app = None


def setUpModule():
    # A QApplication is needed for the SafeTravelWorker (QObject) used by the
    # click-to-travel tests below (v7.5.x: travel now runs off the GUI thread).
    global _app
    from PySide6.QtWidgets import QApplication
    _app = QApplication.instance() or QApplication(sys.argv)


def _make_controller(zp_connected=True):
    """Minimal real StageController for helper tests (no __init__)."""
    ctrl = StageController.__new__(StageController)
    ctrl.xy_stage = MagicMock()
    ctrl.zp_stage = MagicMock() if zp_connected else None
    if ctrl.zp_stage is not None:
        ctrl.zp_stage.flush_moves = MagicMock(return_value=True)
    ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0, "P2": 0, "P3": 0}
    ctrl._pos_poller = MagicMock()
    ctrl._zp_retract_feedrate = 250.0
    ctrl._zp_insert_feedrate = 100.0
    ctrl._min_travel_z_mm = None
    ctrl.move_z_absolute = MagicMock()
    ctrl.wait_for_z_arrival = MagicMock(return_value=True)
    return ctrl


# ── Polarity helpers ────────────────────────────────────────────────


class TestPolarityHelpers(unittest.TestCase):
    def test_zdir_is_minus_one_on_this_machine(self):
        self.assertEqual(ZDIR, -1.0)

    def test_z_height_of_inverts_on_zdir_minus_one(self):
        c = _make_controller()
        # ZDIR=-1: a numerically lower raw Z is physically HIGHER.
        self.assertEqual(c.z_height_of(-10.0), 10.0)
        self.assertEqual(c.z_height_of(0.0), 0.0)
        self.assertEqual(c.z_height_of(25.0), -25.0)

    def test_needle_at_or_above_polarity_safe_zdir_minus_one(self):
        c = _make_controller()
        # Needle retracted at raw -10 (height +10) IS above safe raw 0.
        self.assertTrue(c.needle_at_or_above(-10.0, 0.0))
        # Needle down at print raw +25 (height -25) is NOT above safe raw 0.
        self.assertFalse(c.needle_at_or_above(25.0, 0.0))
        # Equal within tolerance.
        self.assertTrue(c.needle_at_or_above(0.0, 0.0))

    def test_needle_at_or_above_polarity_safe_zdir_plus_one(self):
        c = _make_controller()
        with patch("SupportClasses.StageController.ZDIR", 1.0):
            # Conventional machine: higher raw = higher needle.
            self.assertTrue(c.needle_at_or_above(15.0, 10.0))
            self.assertFalse(c.needle_at_or_above(5.0, 10.0))

    def test_default_travel_z_is_a_lift_in_height_frame(self):
        c = _make_controller()
        # ZDIR=-1: lift 10 mm above print raw -25.88 → MORE-negative raw.
        z = c.default_travel_z(-25.88, margin_mm=10.0)
        self.assertAlmostEqual(z, -35.88)
        self.assertGreater(c.z_height_of(z), c.z_height_of(-25.88))
        # Never returns the dangerous raw literal 5.0.
        self.assertNotEqual(z, 5.0)

    def test_default_travel_z_zdir_plus_one(self):
        c = _make_controller()
        with patch("SupportClasses.StageController.ZDIR", 1.0):
            self.assertAlmostEqual(c.default_travel_z(2.0, margin_mm=10.0), 12.0)


# ── ensure_retracted_to ─────────────────────────────────────────────


class TestEnsureRetractedTo(unittest.TestCase):
    def test_no_zp_returns_true_without_motion(self):
        c = _make_controller(zp_connected=False)
        self.assertTrue(c.ensure_retracted_to(0.0))

    def test_no_descend_when_already_above(self):
        """Needle already retracted (height-above target) → no Z move."""
        c = _make_controller()
        # Needle at raw -10 (height +10), target raw 0 (height 0): already above.
        c.get_zp_position = MagicMock(return_value=(-10.0, 0.0, -10.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=-10.0)
        ok = c.ensure_retracted_to(0.0)
        self.assertTrue(ok)
        c.move_z_absolute.assert_not_called()

    def test_no_descend_with_bad_low_target(self):
        """A target that is BELOW the needle in the HEIGHT frame must never be
        descended to — degrade to a no-op rather than a crash-down."""
        c = _make_controller()
        # Needle retracted at raw -30 (height +30); bad target raw 5.0 is
        # height -5, i.e. far BELOW the needle. Must not lower toward it.
        c.get_zp_position = MagicMock(return_value=(-30.0, 0.0, -30.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=-30.0)
        ok = c.ensure_retracted_to(5.0)
        self.assertTrue(ok)
        c.move_z_absolute.assert_not_called()  # never descends

    def test_raises_and_waits_when_below(self):
        """Needle below safe (height) → raise to safe + wait for arrival."""
        c = _make_controller()
        # Needle down at print raw +25 (height -25); target raw 0 (height 0).
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        ok = c.ensure_retracted_to(0.0)
        self.assertTrue(ok)
        c.move_z_absolute.assert_called_once()
        args, kwargs = c.move_z_absolute.call_args
        self.assertEqual(args[0], 0.0)
        self.assertTrue(kwargs.get("from_zero_ref"))
        c.wait_for_z_arrival.assert_called_once()

    def test_returns_false_on_retract_timeout(self):
        c = _make_controller()
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        c.wait_for_z_arrival = MagicMock(return_value=False)
        self.assertFalse(c.ensure_retracted_to(0.0))

    def test_floor_raises_target_in_height_frame(self):
        """_min_travel_z_mm (insert clearance) raises the target when it is a
        higher needle position than the requested safe_z."""
        c = _make_controller()
        # On ZDIR=-1 a *higher* needle = lower raw. Floor raw -20 (height +20)
        # is higher than safe raw 0 (height 0) → target should become -20.
        c._min_travel_z_mm = -20.0
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        c.ensure_retracted_to(0.0)
        args, _ = c.move_z_absolute.call_args
        self.assertEqual(args[0], -20.0)


# ── PrintManager retract-before-travel ──────────────────────────────


class TestPrintManagerRetractForTravel(unittest.TestCase):
    def _make_pm(self, travel_z=-35.0, zp_connected=True, has_helper=True):
        from SupportClasses.PrintManager import PrintManager
        pm = PrintManager.__new__(PrintManager)
        if has_helper:
            ctrl = MagicMock()
            ctrl.ensure_retracted_to = MagicMock(return_value=True)
        else:
            ctrl = Mock(spec=["is_zp_connected", "move_xy_absolute", "xy_stage"])
        ctrl.is_zp_connected = zp_connected
        pm.controller = ctrl
        pm.job = MagicMock()
        pm.job.settings = MagicMock()
        pm.job.settings.travel_z_height = travel_z
        pm.job.settings.travel_speed_mm_s = 10.0
        pm.exec_logger = None
        return pm, ctrl

    def test_retract_for_travel_calls_ensure(self):
        pm, ctrl = self._make_pm(travel_z=-35.0)
        pm._retract_for_travel("home_xy")
        ctrl.ensure_retracted_to.assert_called_once_with(-35.0)

    def test_retract_skipped_when_zp_disconnected(self):
        pm, ctrl = self._make_pm(zp_connected=False)
        pm._retract_for_travel("move_xy")
        ctrl.ensure_retracted_to.assert_not_called()

    def test_retract_skipped_when_no_travel_z(self):
        pm, ctrl = self._make_pm(travel_z=None)
        pm._retract_for_travel("move_xy")
        ctrl.ensure_retracted_to.assert_not_called()

    def test_retract_noop_when_controller_lacks_helper(self):
        # Older controller without ensure_retracted_to → must not crash.
        pm, ctrl = self._make_pm(has_helper=False)
        pm._retract_for_travel("move_xy")  # should be a clean no-op

    def test_move_xy_retracts_before_xy(self):
        """The discrete MOVE_XY handler retracts (ensure_retracted_to) BEFORE
        issuing the XY move."""
        from SupportClasses.PrintManager import PrintManager, PrintCommand, CommandType
        order = []
        pm, ctrl = self._make_pm(travel_z=-35.0)
        ctrl.ensure_retracted_to.side_effect = lambda *a, **k: order.append("retract") or True
        ctrl.move_xy_absolute.side_effect = lambda *a, **k: order.append("movexy")
        ctrl.xy_stage.set_speed_mm_s = MagicMock()
        pm._wait_for_xy_settle = MagicMock()
        cmd = PrintCommand(type=CommandType.MOVE_XY, params={"x": 1.0, "y": 2.0})
        pm._execute_command(cmd)
        self.assertEqual(order, ["retract", "movexy"])

    def test_home_xy_retracts_before_xy(self):
        from SupportClasses.PrintManager import PrintManager, PrintCommand, CommandType
        order = []
        pm, ctrl = self._make_pm(travel_z=-35.0)
        ctrl.ensure_retracted_to.side_effect = lambda *a, **k: order.append("retract") or True
        ctrl.move_xy_absolute.side_effect = lambda *a, **k: order.append("movexy")
        ctrl.xy_stage.set_speed_mm_s = MagicMock()
        pm._wait_for_xy_settle = MagicMock()
        cmd = PrintCommand(type=CommandType.HOME_XY)
        pm._execute_command(cmd)
        self.assertEqual(order, ["retract", "movexy"])


# ── GUI click-to-travel always retracts ─────────────────────────────


class TestClickToTravelAlwaysRetracts(unittest.TestCase):
    def _ctrl(self, current_z_raw=25.0):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.get_zp_position.return_value = (current_z_raw, 0.0, current_z_raw, 0.0)
        ctrl.zp_logical_value.return_value = current_z_raw
        ctrl.safe_travel_to = MagicMock()
        ctrl.move_xy_absolute = MagicMock()
        return ctrl

    @staticmethod
    def _join_travel(worker):
        # v7.5.x: click-to-travel dispatches safe_travel_to to a worker thread;
        # join it so the (still-guaranteed) call has happened before asserting.
        if getattr(worker, "_thread", None) is not None:
            worker._thread.join(2.0)

    def test_jog_click_always_safe_travel_when_safe_z_set(self):
        from gui.pages.jog_control import JogControlPage
        from gui.widgets.safe_travel_worker import SafeTravelWorker
        page = JogControlPage.__new__(JogControlPage)
        # Needle DOWN at print (raw 25) — the old gate would skip the retract.
        page.controller = self._ctrl(current_z_raw=25.0)
        page._safe_z = 0.0
        page._travel_worker = SafeTravelWorker()
        page._set_travelling = lambda busy: None  # __new__ page has no widgets
        page._on_workspace_position_clicked(9000.0, 0.0)
        self._join_travel(page._travel_worker)
        page.controller.safe_travel_to.assert_called_once()
        page.controller.move_xy_absolute.assert_not_called()

    def test_spheroid_click_always_safe_travel_when_safe_z_set(self):
        from gui.pages.workflows.spheroid_pickup_workflow import (
            SpheroidPickupWorkflowPage,
        )
        from gui.widgets.safe_travel_worker import SafeTravelWorker
        page = SpheroidPickupWorkflowPage.__new__(SpheroidPickupWorkflowPage)
        page._controller = self._ctrl(current_z_raw=25.0)
        page._safe_z = 0.0
        page._travel_worker = SafeTravelWorker()
        page._on_workspace_position_clicked(9000.0, 0.0)
        self._join_travel(page._travel_worker)
        page._controller.safe_travel_to.assert_called_once()
        page._controller.move_xy_absolute.assert_not_called()


# ── Quick Print travel-Z fallback ───────────────────────────────────


class TestQuickPrintTravelZFallback(unittest.TestCase):
    def _make_page(self, safe_z):
        from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._safe_z = safe_z
        page._flow_spin = Mock()
        page._flow_spin.value.return_value = 0.25
        page._pump_combo = Mock()
        page._pump_combo.currentText.return_value = "P1"
        page._printz_spin = Mock()
        page._printz_spin.value.return_value = 0.2
        ctrl = MagicMock()
        ctrl.print_height_to_zref.return_value = -25.88
        ctrl.default_travel_z.return_value = -35.88
        page._controller = ctrl
        return page, ctrl

    def test_fallback_uses_polarity_safe_default_not_literal_5(self):
        page, ctrl = self._make_page(safe_z=None)
        settings = page._build_settings()
        ctrl.default_travel_z.assert_called_once_with(-25.88, margin_mm=10.0)
        self.assertAlmostEqual(settings.travel_z_height, -35.88)
        self.assertNotEqual(settings.travel_z_height, 5.0)

    def test_uses_safe_z_when_configured(self):
        page, ctrl = self._make_page(safe_z=-30.0)
        settings = page._build_settings()
        self.assertAlmostEqual(settings.travel_z_height, -30.0)
        ctrl.default_travel_z.assert_not_called()


if __name__ == "__main__":
    unittest.main()
