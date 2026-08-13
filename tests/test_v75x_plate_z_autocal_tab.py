"""
v7.5.x — Plate Z auto-calibration moved to its own tab (after Plate Location).

The focus-based per-well Z-bottom auto-cal (Step 1 first-spot + Step 2 auto-cal +
the live microscope feed) lives on the "Plate Bed Level" tab, positioned after
"Plate Location" (it drives to each calibration well, so it needs the finished XY
map).

v7.5.x (later): the standalone "Needle Offset Calibration" tab was merged into
"Needle Location" (its reference-Z widgets now build under that tab), and a new
"Pump Compliance" tab was added right after Needle Location. So the order is now
Needle Location / Pump Compliance / Plate Location / Plate Z Auto-Cal / Custom.

These tests build a real ``CalibrationPage`` offscreen and verify the tab order,
that each widget group lives where expected, and that the live-camera start fires
on the right tabs.
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
            "Pump Compliance",
            "Plate Location",
            "Rosettes",
            "Plate Bed Level",
            "Custom",
        ])

    def test_autocal_tab_is_after_plate_location(self):
        page = self._make_page()
        tabs = page._workflow_tabs
        titles = [tabs.tabText(i) for i in range(tabs.count())]
        self.assertGreater(titles.index("Plate Bed Level"),
                           titles.index("Plate Location"))

    def test_the_bed_level_tab_leads_with_the_optical_leveller(self):
        """v7.11 renamed this tab and made it about the plate BED. The optical
        bed leveller is the primary flow; the legacy needle-descent auto-cal is
        kept as a second sub-tab because the Custom tab's button still opens it
        and it owns the microscope feed the descent watches."""
        page = self._make_page()
        sub = page._bed_level_tabs
        self.assertEqual([sub.tabText(i) for i in range(sub.count())],
                         ["Bed level (optical)", "Needle Z auto-cal"])
        self.assertEqual(sub.currentIndex(), 0)

    def test_the_legacy_autocal_controls_are_still_built(self):
        """Renaming the tab must not strip the flow the Custom-tab button and
        the per-well focus tests both still drive."""
        page = self._make_page()
        for attr in ("_zoff_btn_run_z", "_zauto_btn_confirm",
                     "_zauto_btn_record", "_zauto_btn_accept",
                     "_zoff_live_view"):
            self.assertTrue(hasattr(page, attr), attr)

    def test_tab_indices_recorded(self):
        page = self._make_page()
        # The former "Needle Offset Calibration" tab (and its _zoff_tab_index)
        # was merged into Needle Location; Pump Compliance now sits at index 1.
        self.assertFalse(hasattr(page, "_zoff_tab_index"))
        self.assertEqual(page._compcal_tab_index, 1)
        self.assertEqual(page._zauto_tab_index, 4)
        self.assertEqual(
            page._workflow_tabs.tabText(page._compcal_tab_index),
            "Pump Compliance")
        self.assertEqual(
            page._workflow_tabs.tabText(page._zauto_tab_index),
            "Plate Bed Level")


class TestWidgetPlacement(_Base):
    def test_reference_height_widgets_exist(self):
        page = self._make_page()
        for attr in ("_zoff_lbl_replace_z",
                     "_zoff_lbl_safe_z", "_zoff_lbl_top_z",
                     "_zoff_lbl_plate_bottom_z", "_zoff_btn_estimate"):
            self.assertTrue(hasattr(page, attr), attr)
        # v7.17: Max Z retired (no consumer anywhere) — its label is gone.
        self.assertFalse(hasattr(page, "_zoff_lbl_max_z"))

    def test_autocal_widgets_and_live_feed_exist(self):
        page = self._make_page()
        for attr in ("_zauto_btn_confirm", "_zauto_btn_record",
                     "_zauto_btn_accept", "_zauto_btn_redo",
                     "_zoff_btn_run_z", "_zoff_btn_cancel_z",
                     "_zoff_lbl_auto_z", "_zoff_live_view"):
            self.assertTrue(hasattr(page, attr), attr)

    def test_autocal_widgets_under_autocal_tab(self):
        # The auto-cal run button must be a descendant of the Plate Z Auto-Cal
        # tab page, not the (merged) Needle Location tab page.
        page = self._make_page()
        tabs = page._workflow_tabs
        autocal_page = tabs.widget(page._zauto_tab_index)
        needle_loc_page = tabs.widget(0)          # merged Needle Location tab
        self.assertIn(autocal_page,
                      _ancestors(page._zoff_btn_run_z))
        self.assertNotIn(needle_loc_page,
                         _ancestors(page._zoff_btn_run_z))
        # Reference-height button now lives under the merged Needle Location tab.
        self.assertIn(needle_loc_page, _ancestors(page._zoff_btn_estimate))


class TestLiveCameraTrigger(_Base):
    def test_camera_starts_on_autocal_tab(self):
        page = self._make_page()
        page._zoff_ensure_live_camera = MagicMock()
        page._on_workflow_tab_changed(page._zauto_tab_index)
        page._zoff_ensure_live_camera.assert_called_once()

    def test_microscope_camera_not_started_on_needle_location_tab(self):
        page = self._make_page()
        page._zoff_ensure_live_camera = MagicMock()
        page._on_workflow_tab_changed(0)          # Needle Location tab
        page._zoff_ensure_live_camera.assert_not_called()

    def test_needle_cameras_start_on_compliance_tab(self):
        page = self._make_page()
        page._compcal_ensure_live_cameras = MagicMock()
        page._zoff_ensure_live_camera = MagicMock()
        page._on_workflow_tab_changed(page._compcal_tab_index)
        page._compcal_ensure_live_cameras.assert_called_once()
        # The microscope feed helper is NOT the one used on this tab.
        page._zoff_ensure_live_camera.assert_not_called()


class TestNeedleLocationBigPane(_Base):
    """v7.13 — the Needle Location tab gives the VIDEO the real estate.

    Supersedes v7.11's TestNeedleLocationScrolls, which pinned the old layout
    (a vertical cameras/controls splitter whose controls column scrolled). The
    redesign replaces that with one big camera pane — a QStackedWidget the
    wizard drives via step_changed (side cams for steps 1-2, the wizard's
    microscope pane for steps 3-4) — beside a compact wizard column, with no
    scroll area burying the wizard.
    """

    def _main_split(self, page):
        from PySide6.QtCore import Qt
        from PySide6.QtWidgets import QSplitter
        stack = page._needle_loc_camera_stack
        w = stack.parentWidget()
        while w is not None and not isinstance(w, QSplitter):
            w = w.parentWidget()
        self.assertIsNotNone(w, "the camera stack must sit in a splitter")
        self.assertEqual(w.orientation(), Qt.Horizontal)
        return w

    def test_the_camera_stack_is_the_splitters_first_pane(self):
        page = self._make_page()
        sp = self._main_split(page)
        self.assertIs(sp.widget(0), page._needle_loc_camera_stack)

    def test_the_video_pane_gets_the_lions_share(self):
        page = self._make_page()
        page.show()
        self._app.processEvents()
        page.setFixedSize(1400, 700)
        self._app.processEvents()
        page.layout().activate()
        self._app.processEvents()
        sp = self._main_split(page)
        sizes = sp.sizes()
        self.assertGreater(
            sizes[0], sizes[1],
            "the camera pane must be wider than the wizard column — the "
            "whole point of the v7.13 restructure")
        page.hide()

    def test_the_wizard_is_not_buried_in_a_scroll_area(self):
        """The old layout scrolled the wizard to the bottom of a narrow
        column; the redesign forbids any QScrollArea between the wizard and
        the tab (the Advanced expander keeps its own INTERNAL scroll)."""
        from PySide6.QtWidgets import QScrollArea
        page = self._make_page()
        w = page._needle_bore_wizard.parentWidget()
        while w is not None:
            self.assertNotIsInstance(
                w, QScrollArea,
                "the wizard must not live inside a scroll area")
            w = w.parentWidget()

    def test_the_side_cam_row_keeps_its_floor(self):
        page = self._make_page()
        stack = page._needle_loc_camera_stack
        self.assertGreaterEqual(stack.widget(0).minimumHeight(), 200)

    def test_the_microscope_feed_is_worth_clicking_on(self):
        """Bore tips are picked on it and a bore is a few hundred µm across.
        The height now comes from the BIG PANE, not a min-height floor: on
        step 3 the stack shows the microscope pane at pane height."""
        page = self._make_page()
        wiz = page._needle_bore_wizard
        feed = getattr(wiz, "_mic_feed", None)
        if feed is None:
            self.skipTest("no CameraFeedView in this build")
        page.show()
        self._app.processEvents()
        page.setFixedSize(1400, 700)
        self._app.processEvents()
        wiz.go_to_step("bores")
        self._app.processEvents()
        stack = page._needle_loc_camera_stack
        self.assertEqual(stack.currentIndex(), 1)
        self.assertIs(stack.widget(1), wiz.microscope_pane())
        self.assertGreaterEqual(
            stack.height(), 400,
            "the shared pane must give the microscope real height")
        page.hide()


def _ancestors(widget):
    out = []
    w = widget.parent()
    while w is not None:
        out.append(w)
        w = w.parent()
    return out


if __name__ == "__main__":
    unittest.main()
