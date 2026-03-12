"""
Tests for v7.3.1 Phase 6 — Well Plate Navigator & Fast Travel.

Tests cover:
- WellPlateNavigator widget state management (set_plate, set_well_positions,
  set_current_well, update_current_from_position)
- Signal emission on well click
- Safety gate logic (safe_z and position checks)
- StageController.safe_travel_to() call sequence
- CalibrationPage.get_calibration_data() accessor
- CalibrationPage.calibration_data_changed signal emission
"""

import math
import sys
import unittest
from unittest.mock import MagicMock, patch, call

from SupportClasses.WellPlate import WellPlate


# ── WellPlateNavigator Tests (headless — no QApplication needed for state) ──


class TestWellPlateNavigatorState(unittest.TestCase):
    """Test WellPlateNavigator internal state without rendering."""

    @classmethod
    def setUpClass(cls):
        """Ensure QApplication exists for widget tests."""
        from PySide6.QtWidgets import QApplication
        if QApplication.instance() is None:
            cls._app = QApplication(sys.argv)
        else:
            cls._app = QApplication.instance()

    def _make_nav(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        return WellPlateNavigator()

    def test_initial_state(self):
        nav = self._make_nav()
        self.assertIsNone(nav._plate)
        self.assertEqual(nav._calibrated_wells, set())
        self.assertIsNone(nav._current_well)
        self.assertIsNone(nav._hover_well)
        self.assertIsNone(nav._well_positions)

    def test_set_plate(self):
        nav = self._make_nav()
        plate = WellPlate.from_format(96)
        nav.set_plate(plate)
        self.assertIs(nav._plate, plate)
        self.assertIsNone(nav._hover_well)
        self.assertIsNone(nav._current_well)

    def test_set_well_positions_updates_calibrated_wells(self):
        nav = self._make_nav()
        positions = {"A1": (1000.0, 2000.0), "A2": (10000.0, 2000.0)}
        nav.set_well_positions(positions)
        self.assertEqual(nav._calibrated_wells, {"A1", "A2"})
        self.assertIs(nav._well_positions, positions)

    def test_set_well_positions_none(self):
        nav = self._make_nav()
        nav.set_well_positions({"A1": (0, 0)})
        nav.set_well_positions(None)
        self.assertIsNone(nav._well_positions)
        # Calibrated wells should remain from previous call (not cleared)
        self.assertEqual(nav._calibrated_wells, {"A1"})

    def test_set_current_well(self):
        nav = self._make_nav()
        nav.set_current_well("B3")
        self.assertEqual(nav._current_well, "B3")

    def test_set_current_well_no_change_no_redundant_update(self):
        nav = self._make_nav()
        nav.set_current_well("B3")
        # Second call with same value should not trigger update
        with patch.object(nav, 'update') as mock_update:
            nav.set_current_well("B3")
            mock_update.assert_not_called()

    def test_set_calibrated_wells(self):
        nav = self._make_nav()
        wells = {"A1", "A2", "B1"}
        nav.set_calibrated_wells(wells)
        self.assertEqual(nav._calibrated_wells, wells)


class TestWellPlateNavigatorCurrentTracking(unittest.TestCase):
    """Test update_current_from_position() nearest-well logic."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        if QApplication.instance() is None:
            cls._app = QApplication(sys.argv)
        else:
            cls._app = QApplication.instance()

    def _make_nav_with_plate(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        nav = WellPlateNavigator()
        plate = WellPlate.from_format(96)
        nav.set_plate(plate)
        # Generate positions with 9mm spacing (typical 96-well)
        positions = plate.get_all_positions_from_a1(0.0, 0.0)
        nav.set_well_positions(positions)
        return nav, plate, positions

    def test_exact_position_highlights_well(self):
        nav, plate, positions = self._make_nav_with_plate()
        a1_pos = positions["A1"]
        nav.update_current_from_position(a1_pos[0], a1_pos[1])
        self.assertEqual(nav._current_well, "A1")

    def test_nearby_position_highlights_well(self):
        nav, plate, positions = self._make_nav_with_plate()
        a1_pos = positions["A1"]
        # Offset by 1mm — still within 0.6 * 9mm threshold
        nav.update_current_from_position(a1_pos[0] + 1000, a1_pos[1])
        self.assertEqual(nav._current_well, "A1")

    def test_far_position_clears_highlight(self):
        nav, plate, positions = self._make_nav_with_plate()
        # Position very far from any well
        nav.update_current_from_position(999999.0, 999999.0)
        self.assertIsNone(nav._current_well)

    def test_no_positions_is_noop(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        nav = WellPlateNavigator()
        nav.update_current_from_position(0, 0)  # Should not raise
        self.assertIsNone(nav._current_well)

    def test_empty_positions_is_noop(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        nav = WellPlateNavigator()
        nav.set_well_positions({})
        nav.update_current_from_position(0, 0)
        self.assertIsNone(nav._current_well)


class TestWellPlateNavigatorSignal(unittest.TestCase):
    """Test well_clicked signal emission."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        if QApplication.instance() is None:
            cls._app = QApplication(sys.argv)
        else:
            cls._app = QApplication.instance()

    def test_signal_exists(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        nav = WellPlateNavigator()
        received = []
        nav.well_clicked.connect(lambda name: received.append(name))
        # Manually emit to verify signal works
        nav.well_clicked.emit("C5")
        self.assertEqual(received, ["C5"])


# ── StageController.safe_travel_to() Tests ──────────────────────────


class TestSafeTravelTo(unittest.TestCase):
    """Test the safe_travel_to() method on StageController."""

    def _make_controller(self, xy_connected=True, zp_connected=True):
        """Create a mock StageController with controllable connection state.

        Uses a real StageController instance with xy_stage/zp_stage set to
        control the is_xy_connected/is_zp_connected property values.
        """
        from SupportClasses.StageController import StageController
        ctrl = StageController.__new__(StageController)
        # Properties check xy_stage/zp_stage for truthiness
        ctrl.xy_stage = MagicMock() if xy_connected else None
        ctrl.zp_stage = MagicMock() if zp_connected else None
        ctrl.move_z_absolute = MagicMock()
        ctrl.move_xy_absolute = MagicMock()
        return ctrl

    def test_full_sequence(self):
        """Verify Z-up → XY → Z-down ordering."""
        ctrl = self._make_controller()
        ctrl.safe_travel_to(5000.0, 10000.0, safe_z_mm=15.0, target_z_mm=5.0)

        calls = ctrl.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 2)
        # First call: raise to safe Z
        self.assertEqual(calls[0], call(15.0, from_zero_ref=True))
        # Second call: lower to target Z
        self.assertEqual(calls[1], call(5.0, from_zero_ref=True))
        # XY move
        ctrl.move_xy_absolute.assert_called_once_with(5000.0, 10000.0, from_zero_ref=False)

    def test_no_target_z_stays_at_safe(self):
        """If target_z_mm is None, only one Z move (up to safe)."""
        ctrl = self._make_controller()
        ctrl.safe_travel_to(5000.0, 10000.0, safe_z_mm=15.0)

        calls = ctrl.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0], call(15.0, from_zero_ref=True))

    def test_xy_only(self):
        """With no ZP connected, only XY move happens."""
        ctrl = self._make_controller(zp_connected=False)
        ctrl.safe_travel_to(5000.0, 10000.0, safe_z_mm=15.0, target_z_mm=5.0)

        ctrl.move_z_absolute.assert_not_called()
        ctrl.move_xy_absolute.assert_called_once()

    def test_zp_only(self):
        """With no XY connected, only Z moves happen."""
        ctrl = self._make_controller(xy_connected=False)
        ctrl.safe_travel_to(5000.0, 10000.0, safe_z_mm=15.0, target_z_mm=5.0)

        ctrl.move_xy_absolute.assert_not_called()
        self.assertEqual(ctrl.move_z_absolute.call_count, 2)

    def test_nothing_connected(self):
        """With no hardware connected, no moves happen."""
        ctrl = self._make_controller(xy_connected=False, zp_connected=False)
        ctrl.safe_travel_to(5000.0, 10000.0, safe_z_mm=15.0)

        ctrl.move_z_absolute.assert_not_called()
        ctrl.move_xy_absolute.assert_not_called()


# ── CalibrationPage Data Accessor & Signal Tests ─────────────────────


class TestCalibrationPageDataBridge(unittest.TestCase):
    """Test CalibrationPage.get_calibration_data() and signal emission."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        if QApplication.instance() is None:
            cls._app = QApplication(sys.argv)
        else:
            cls._app = QApplication.instance()

    def _make_cal_page(self):
        """Create a CalibrationPage with mocked controller."""
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.get_zp_position.return_value = (None, None, None)
        page = CalibrationPage(ctrl, settings=None)
        return page

    def test_get_calibration_data_empty(self):
        page = self._make_cal_page()
        plate, positions, safe_z = page.get_calibration_data()
        self.assertIsNone(plate)
        self.assertIsNone(positions)
        self.assertIsNone(safe_z)

    def test_get_calibration_data_with_predicted(self):
        page = self._make_cal_page()
        plate = WellPlate.from_format(96)
        page._plate = plate
        page._taught_a1 = (5000.0, 10000.0)
        page._compute_predicted_positions()
        plate_out, positions_out, safe_z_out = page.get_calibration_data()
        self.assertIs(plate_out, plate)
        self.assertIsNotNone(positions_out)
        self.assertIn("A1", positions_out)
        self.assertIsNone(safe_z_out)

    def test_get_calibration_data_prefers_calibrated(self):
        page = self._make_cal_page()
        page._plate = WellPlate.from_format(6)
        page._predicted_positions = {"A1": (0, 0), "A2": (9000, 0)}
        page._calibrated_positions = {"A1": (100, 50), "A2": (9100, 50)}
        page._safe_z = 12.5
        plate_out, positions_out, safe_z_out = page.get_calibration_data()
        # Should return calibrated, not predicted
        self.assertEqual(positions_out["A1"], (100, 50))
        self.assertAlmostEqual(safe_z_out, 12.5)

    def test_signal_emitted_on_compute_predicted(self):
        page = self._make_cal_page()
        received = []
        page.calibration_data_changed.connect(lambda: received.append(True))
        page._plate = WellPlate.from_format(6)
        page._taught_a1 = (0.0, 0.0)
        page._compute_predicted_positions()
        self.assertTrue(len(received) > 0, "calibration_data_changed not emitted")

    def test_signal_emitted_on_set_safe_z(self):
        page = self._make_cal_page()
        received = []
        page.calibration_data_changed.connect(lambda: received.append(True))
        # Mock controller to return a Z position
        page.controller.get_zp_position.return_value = (20.0, None, None)
        page._set_safe_z()
        self.assertTrue(len(received) > 0, "calibration_data_changed not emitted on safe_z")
        self.assertIsNotNone(page._safe_z)


# ── Jog Page set_calibration_data() Tests ────────────────────────────


class TestJogPageCalibrationData(unittest.TestCase):
    """Test JogControlPage.set_calibration_data() integration."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        if QApplication.instance() is None:
            cls._app = QApplication(sys.argv)
        else:
            cls._app = QApplication.instance()

    def _make_jog_page(self):
        from gui.pages.jog_control import JogControlPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        page = JogControlPage(ctrl)
        return page

    def test_set_calibration_data_stores_state(self):
        page = self._make_jog_page()
        plate = WellPlate.from_format(96)
        positions = {"A1": (0, 0), "B1": (0, 9000)}
        page.set_calibration_data(plate, positions, 15.0)
        self.assertIs(page._plate, plate)
        self.assertIs(page._well_positions, positions)
        self.assertEqual(page._safe_z, 15.0)

    def test_set_calibration_data_updates_navigator(self):
        page = self._make_jog_page()
        plate = WellPlate.from_format(6)
        positions = plate.get_all_positions_from_a1(0, 0)
        page.set_calibration_data(plate, positions, 10.0)
        if page._well_nav is not None:
            self.assertIs(page._well_nav._plate, plate)
            self.assertEqual(page._well_nav._calibrated_wells, set(positions.keys()))

    def test_set_calibration_data_none_values(self):
        page = self._make_jog_page()
        page.set_calibration_data(None, None, None)
        self.assertIsNone(page._plate)
        self.assertIsNone(page._well_positions)
        self.assertIsNone(page._safe_z)


if __name__ == "__main__":
    unittest.main()
