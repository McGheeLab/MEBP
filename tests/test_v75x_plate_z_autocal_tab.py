"""
v7.5.x — Plate Z auto-calibration moved to its own tab (after Plate Location).

The focus-based per-well Z-bottom auto-cal (Step 1 first-spot + Step 2 auto-cal +
the live microscope feed) split out of the "Needle Offset Calibration" tab into a
new "Plate Z Auto-Cal" tab, positioned after "Plate Location" (it drives to each
calibration well, so it needs the finished XY map). The needle's vertical
*reference heights* stay on the Needle Offset tab (taught before Plate Location).

These tests build a real ``CalibrationPage`` offscreen and verify the new tab
order, that each widget group lives where expected, and that the live-camera
start now fires on the new tab.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from gui.pages.calibration import CalibrationPage  # noqa: E402


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_page(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (1000.0, 2000.0)
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        return CalibrationPage(ctrl, settings=None)


class TestTabOrder(_Base):
    def test_five_tabs_in_expected_order(self):
        page = self._make_page()
        tabs = page._workflow_tabs
        titles = [tabs.tabText(i) for i in range(tabs.count())]
        self.assertEqual(titles, [
            "Needle Location",
            "Needle Offset Calibration",
            "Plate Location",
            "Plate Z Auto-Cal",
            "Custom",
        ])

    def test_autocal_tab_is_after_plate_location(self):
        page = self._make_page()
        tabs = page._workflow_tabs
        titles = [tabs.tabText(i) for i in range(tabs.count())]
        self.assertGreater(titles.index("Plate Z Auto-Cal"),
                           titles.index("Plate Location"))

    def test_tab_indices_recorded(self):
        page = self._make_page()
        self.assertEqual(page._zoff_tab_index, 1)
        self.assertEqual(page._zauto_tab_index, 3)
        self.assertEqual(
            page._workflow_tabs.tabText(page._zauto_tab_index),
            "Plate Z Auto-Cal")


class TestWidgetPlacement(_Base):
    def test_reference_height_widgets_exist(self):
        page = self._make_page()
        for attr in ("_zoff_lbl_replace_z", "_zoff_lbl_max_z",
                     "_zoff_lbl_safe_z", "_zoff_lbl_top_z",
                     "_zoff_lbl_plate_bottom_z", "_zoff_btn_estimate"):
            self.assertTrue(hasattr(page, attr), attr)

    def test_autocal_widgets_and_live_feed_exist(self):
        page = self._make_page()
        for attr in ("_zauto_btn_confirm", "_zauto_btn_record",
                     "_zauto_btn_accept", "_zauto_btn_redo",
                     "_zoff_btn_run_z", "_zoff_btn_cancel_z",
                     "_zoff_lbl_auto_z", "_zoff_live_view"):
            self.assertTrue(hasattr(page, attr), attr)

    def test_autocal_widgets_under_autocal_tab(self):
        # The auto-cal run button must be a descendant of the Plate Z Auto-Cal
        # tab page, not the Needle Offset tab page.
        page = self._make_page()
        tabs = page._workflow_tabs
        autocal_page = tabs.widget(page._zauto_tab_index)
        zoff_page = tabs.widget(page._zoff_tab_index)
        self.assertIn(autocal_page,
                      _ancestors(page._zoff_btn_run_z))
        self.assertNotIn(zoff_page,
                         _ancestors(page._zoff_btn_run_z))
        # Reference-height button stays under the Needle Offset tab.
        self.assertIn(zoff_page, _ancestors(page._zoff_btn_estimate))


class TestLiveCameraTrigger(_Base):
    def test_camera_starts_on_autocal_tab(self):
        page = self._make_page()
        page._zoff_ensure_live_camera = MagicMock()
        page._on_workflow_tab_changed(page._zauto_tab_index)
        page._zoff_ensure_live_camera.assert_called_once()

    def test_camera_not_started_on_needle_offset_tab(self):
        page = self._make_page()
        page._zoff_ensure_live_camera = MagicMock()
        page._on_workflow_tab_changed(page._zoff_tab_index)
        page._zoff_ensure_live_camera.assert_not_called()


def _ancestors(widget):
    out = []
    w = widget.parent()
    while w is not None:
        out.append(w)
        w = w.parent()
    return out


if __name__ == "__main__":
    unittest.main()
