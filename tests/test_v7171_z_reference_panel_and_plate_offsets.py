"""v7.17.1 — Advanced Z references: open by default, resizable, and a
plate-type round trip driven from the picker itself.

Operator: *"I need to be able to expand up advanced plate z references to be
much taller. in fact this should not be hidden by default"* and *"on the
references picker I need another button for assign these to the plate type for
auto application next time. and a button for apply plate offsets based on plate
top position from that assignment"*.

The panel used to start collapsed AND, once opened, share the wizard column
50/50 (both were added at stretch=1) — so the "Assign to plate type" button had
been sitting inside a panel most operators never opened.

The new direction is ``plate_z_refs_from_top``: the stored offsets are mm below
the needle-cam fiducial, so anchoring on a TAUGHT plate top cancels the
fiducial and keeps only the plate's own feature spacing.
"""

import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.StageController import StageController      # noqa: E402


def _ctrl(offsets, z_up_sign=1.0, zero_z=0.0):
    """A StageController with just the Z frame + offsets wired."""
    c = StageController.__new__(StageController)
    c._plate_z_offsets = dict(offsets)
    c._needle_cam_z_user = None
    c._z_up_sign = z_up_sign
    c.zero_position = {"Z": zero_z}
    return c


class TestDerivingFromTheTaughtTop(unittest.TestCase):

    OFF = {"top": 10.0, "bottom": 12.0, "safe": 4.0, "max": 1.0}

    def test_spacing_is_preserved_relative_to_the_taught_top(self):
        """bottom sits 2 mm below top, safe 6 mm above it — whatever the top
        actually turns out to be."""
        c = _ctrl(self.OFF)
        top_zref = c.user_z_to_zref(30.0)
        refs = c.plate_z_refs_from_top(top_zref)
        self.assertAlmostEqual(c.zref_to_user_z(refs["plate_bottom_z"]), 28.0, 6)
        self.assertAlmostEqual(c.zref_to_user_z(refs["safe_z"]), 36.0, 6)
        self.assertAlmostEqual(c.zref_to_user_z(refs["plate_max_z"]), 39.0, 6)

    def test_the_fiducial_cancels_out(self):
        """THE point of anchoring on the top: the answer must not depend on
        the needle-cam fiducial at all."""
        c = _ctrl(self.OFF)
        top_zref = c.user_z_to_zref(30.0)
        a = c.plate_z_refs_from_top(top_zref)
        c._needle_cam_z_user = 999.0          # move the fiducial
        b = c.plate_z_refs_from_top(top_zref)
        self.assertEqual(a, b)

    def test_a_thicker_plate_moves_everything_with_the_top(self):
        """Re-teaching the top 1 mm higher must raise the derived floor by
        exactly 1 mm — that is what absorbs plate-to-plate variation."""
        c = _ctrl(self.OFF)
        lo = c.plate_z_refs_from_top(c.user_z_to_zref(30.0))
        hi = c.plate_z_refs_from_top(c.user_z_to_zref(31.0))
        self.assertAlmostEqual(
            c.zref_to_user_z(hi["plate_bottom_z"])
            - c.zref_to_user_z(lo["plate_bottom_z"]), 1.0, 6)

    def test_it_holds_on_the_inverted_Z_machine(self):
        """ME3B V1 runs z_up_sign = -1; a sign slip here would put the derived
        floor on the wrong side of the glass."""
        c = _ctrl(self.OFF, z_up_sign=-1.0, zero_z=-76.35)
        top_zref = c.user_z_to_zref(30.0)
        refs = c.plate_z_refs_from_top(top_zref)
        self.assertAlmostEqual(c.zref_to_user_z(refs["plate_bottom_z"]), 28.0, 6)
        self.assertAlmostEqual(c.zref_to_user_z(refs["safe_z"]), 36.0, 6)

    def test_the_bottom_really_is_BELOW_the_top(self):
        """Direction, not just distance — the thing a sign error breaks."""
        for sign, zero in ((1.0, 0.0), (-1.0, -76.35)):
            with self.subTest(z_up_sign=sign):
                c = _ctrl(self.OFF, z_up_sign=sign, zero_z=zero)
                top = 30.0
                refs = c.plate_z_refs_from_top(c.user_z_to_zref(top))
                self.assertLess(
                    c.zref_to_user_z(refs["plate_bottom_z"]), top,
                    "the derived well floor came out ABOVE the plate top")
                self.assertGreater(
                    c.zref_to_user_z(refs["safe_z"]), top,
                    "the derived travel height came out BELOW the plate top")

    def test_the_top_itself_is_never_returned(self):
        """It is the input. Returning it would invite a caller to overwrite a
        measurement with a value derived from itself."""
        c = _ctrl(self.OFF)
        self.assertNotIn("plate_top_z",
                         c.plate_z_refs_from_top(c.user_z_to_zref(30.0)))

    def test_no_stored_offsets_refuses_rather_than_returning_zeros(self):
        """With nothing stored every delta is zero, which would report the
        well floor as exactly the plate's top surface — confident and wrong."""
        self.assertIsNone(_ctrl({}).plate_z_refs_from_top(0.0))

    def test_a_partial_assignment_still_derives_what_it_knows(self):
        c = _ctrl({"top": 10.0, "bottom": 12.0})
        refs = c.plate_z_refs_from_top(c.user_z_to_zref(30.0))
        self.assertAlmostEqual(c.zref_to_user_z(refs["plate_bottom_z"]), 28.0, 6)


class TestTheHandlerIsSafe(unittest.TestCase):
    """The derived floor must never arm the print-floor clamp."""

    def _page(self, offsets, top=30.0):
        from types import SimpleNamespace
        from gui.pages.calibration import CalibrationPage
        c = _ctrl(offsets)
        page = SimpleNamespace(
            controller=c,
            _top_z=(c.user_z_to_zref(top) if top is not None else None),
            _plate_bottom_z=None, _safe_z=None,
            _zoff_lbl_apply_from_top=None,
            _zoff_lbl_plate_bottom_z=None, _zoff_lbl_safe_z=None,
            pushes=[],
        )
        page._zoff_user_z = lambda z: c.zref_to_user_z(z)
        page._zoff_push_plate_bottom_to_controller = (
            lambda z, src: page.pushes.append((z, src)))
        page._emit_calibration_data_changed = lambda: None
        page._run = lambda: CalibrationPage._zoff_apply_offsets_from_top(page)
        return page

    def test_the_derived_bottom_is_tagged_estimated(self):
        p = self._page({"top": 10.0, "bottom": 12.0, "safe": 4.0})
        p._run()
        self.assertEqual([src for _z, src in p.pushes], ["estimated"],
                         "a derived floor was published as a measurement — it "
                         "would arm the print-floor clamp on a guess")

    def test_it_refuses_without_a_taught_top(self):
        p = self._page({"top": 10.0, "bottom": 12.0}, top=None)
        p._run()
        self.assertEqual(p.pushes, [])
        self.assertIsNone(p._plate_bottom_z)

    def test_it_refuses_when_the_plate_has_no_offsets(self):
        p = self._page({})
        p._run()
        self.assertEqual(p.pushes, [])
        self.assertIsNone(p._plate_bottom_z)

    def test_it_does_not_touch_the_taught_top(self):
        p = self._page({"top": 10.0, "bottom": 12.0, "safe": 4.0})
        before = p._top_z
        p._run()
        self.assertEqual(p._top_z, before)


def _page_ctrl():
    """The controller stand-in the real CalibrationPage needs to build."""
    from unittest.mock import MagicMock
    ctrl = MagicMock()
    ctrl.is_xy_connected = True
    ctrl.is_zp_connected = False
    ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
    ctrl.get_xy_position.return_value = (1000.0, 2000.0)
    ctrl.z_up_sign.return_value = 1.0
    ctrl.zref_to_user_z.side_effect = lambda z: float(z)
    return ctrl


class TestThePanelDefaults(unittest.TestCase):

    def test_the_advanced_panel_is_open_and_on_a_splitter(self):
        """Both halves of the operator's ask, checked on a REAL page."""
        from PySide6.QtWidgets import QApplication, QSplitter
        from PySide6.QtCore import Qt
        app = QApplication.instance() or QApplication([])
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage(_page_ctrl(), settings=None)
        try:
            self.assertTrue(
                page._needle_loc_adv_btn.isChecked(),
                "Advanced Z references still starts collapsed")
            split = page._needle_loc_adv_split
            self.assertIsInstance(split, QSplitter)
            self.assertEqual(split.orientation(), Qt.Vertical)
            self.assertEqual(
                split.count(), 2,
                "the wizard and the advanced panel must be the two halves, "
                "so the divider can be dragged between them")
            self.assertFalse(
                split.childrenCollapsible(),
                "a draggable pane that can vanish entirely is worse than a "
                "fixed one")
        finally:
            page.deleteLater()

    def test_both_plate_type_buttons_are_on_the_picker(self):
        from PySide6.QtWidgets import QApplication
        app = QApplication.instance() or QApplication([])
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage(_page_ctrl(), settings=None)
        try:
            self.assertTrue(hasattr(page, "_zoff_btn_assign_to_type"))
            self.assertTrue(hasattr(page, "_zoff_btn_apply_from_top"))
        finally:
            page.deleteLater()


if __name__ == "__main__":
    unittest.main()
