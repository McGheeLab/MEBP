"""
v7.13 — bore-wizard step 2: plate bottom derived from plate top + typed offset.

The plate bottom is contestable to teach directly (the needle can't see it);
the plate TOP is verifiable by eye. Step 2 therefore captures the top and
derives the bottom from a typed "bottom is X mm below top" offset, auto-filled
from the selected plate type's stored z_offsets (bottom − top — both are mm
below the same fiducial, so the difference is the physical distance and is
fiducial-independent).

The polarity check is the mutation test that matters: a flipped sign puts the
derived bottom on the WRONG SIDE of the top — above it on a z_up_sign=+1
machine — and every consumer downstream (park height, floor, print Z) would
then aim past the glass.
"""

import os
import sys
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import gui.widgets.needle_bore_wizard as wizmod
from SupportClasses.PhysicalModels import NeedleBore, NeedleSpec
from gui.widgets.needle_bore_wizard import (
    NeedleBoreWizard,
    STEP_Z_REFS,
    plate_bottom_zref_from_top,
)


def _backpack():
    return NeedleSpec(needle_form="backpack", bores=[
        NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
        NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P2")])


# ── the pure helper ─────────────────────────────────────────────────

class TestPlateBottomFromTop(unittest.TestCase):

    def test_positive_up_sign_puts_the_bottom_numerically_below_the_top(self):
        self.assertAlmostEqual(
            plate_bottom_zref_from_top(20.0, 1.5, +1.0), 18.5, places=9)

    def test_negative_up_sign_puts_the_bottom_numerically_above_the_top(self):
        """ME3B V1: the needle descends as zero-ref Z increases (z_up_sign=-1),
        so 'below the top' is a LARGER zero-ref number. A flipped sign here is
        the classic polarity bug — the derived bottom would sit a plate
        thickness past the glass."""
        self.assertAlmostEqual(
            plate_bottom_zref_from_top(20.0, 1.5, -1.0), 21.5, places=9)

    def test_zero_offset_is_the_top_itself(self):
        self.assertEqual(plate_bottom_zref_from_top(12.0, 0.0, 1.0), 12.0)

    def test_the_derivation_is_self_consistent_with_derive_z_up_sign(self):
        """After applying, print_z_dir() must come out equal to z_up_sign —
        the whole reason the helper takes z_up_sign, not print_z_dir()."""
        from SupportClasses.StageController import derive_z_up_sign
        for sign in (+1.0, -1.0):
            top = 20.0
            bottom = plate_bottom_zref_from_top(top, 1.5, sign)
            self.assertEqual(derive_z_up_sign(top, bottom), sign)


# ── the wizard's step-2 surface ─────────────────────────────────────

class _Ctrl:
    def __init__(self, z_up=1.0):
        self.is_xy_connected = True
        self._z_up = z_up

    def get_xy_position(self, cached=False):
        return (0.0, 0.0)

    def z_up_sign(self):
        return self._z_up

    def get_plate_bottom_z_source(self):
        return ""


class _Base(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _wizard(self, *, top=None, offsets=None, z_up=1.0):
        self.ctrl = _Ctrl(z_up=z_up)
        self.top_captures = []
        self.safe_captures = []
        self.applied = []
        self.emitted = []
        offsets = {} if offsets is None else dict(offsets)

        host = SimpleNamespace(
            controller=self.ctrl,
            _hardware_config=SimpleNamespace(
                needle=_backpack(),
                plate_z_offsets=lambda: offsets),
            _camera_manager=None,
            _plate_bottom_z=None,
            _safe_z=None,
            _top_z=top,
            _max_z=None,
            _needle_origin_um=(1.0, 2.0),
            _needle_loc_xy_um=None,
            _ploc_microscope_cam_idx=lambda: None,
            _zoff_set_top_z=lambda: self.top_captures.append(True),
            _zoff_set_safe_z=lambda: self.safe_captures.append(True),
            _zoff_apply_plate_bottom_z=(
                lambda z, source="estimated":
                self.applied.append((float(z), source))),
            _apply_plate_type_z_estimates=lambda force=False: None,
        )
        self.host = host
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        w.calibration_changed.connect(lambda: self.emitted.append(True))
        return w


class TestAutofill(_Base):

    def test_offset_fills_from_bottom_minus_top(self):
        w = self._wizard(offsets={"top": 36.0, "bottom": 37.2, "safe": 30.0})
        w._autofill_bottom_offset()
        self.assertAlmostEqual(w._s2_bottom_offset.value(), 1.2, places=6)
        self.assertIn("Auto-filled", w._s2_offset_hint.text())

    def test_missing_offsets_leave_a_datasheet_hint(self):
        w = self._wizard(offsets={"top": 36.0})     # no bottom
        before = w._s2_bottom_offset.value()
        w._autofill_bottom_offset()
        self.assertEqual(w._s2_bottom_offset.value(), before)
        self.assertIn("datasheet", w._s2_offset_hint.text())

    def test_degenerate_offsets_are_rejected(self):
        """bottom above top in the offset frame is impossible geometry."""
        w = self._wizard(offsets={"top": 37.0, "bottom": 36.0})
        w._autofill_bottom_offset()
        self.assertIn("datasheet", w._s2_offset_hint.text())

    def test_a_typed_value_is_never_silently_overwritten(self):
        w = self._wizard(offsets={"top": 36.0, "bottom": 37.2})
        w._s2_bottom_offset.setValue(2.5)            # operator types
        self.assertTrue(w._s2_offset_user_edited)
        w._autofill_bottom_offset()                  # e.g. plate re-push
        self.assertAlmostEqual(w._s2_bottom_offset.value(), 2.5, places=6)

    def test_the_refresh_button_replaces_a_typed_value(self):
        w = self._wizard(offsets={"top": 36.0, "bottom": 37.2})
        w._s2_bottom_offset.setValue(2.5)
        w._autofill_bottom_offset(force=True)
        self.assertAlmostEqual(w._s2_bottom_offset.value(), 1.2, places=6)
        self.assertFalse(w._s2_offset_user_edited)

    def test_set_hardware_config_reruns_the_autofill(self):
        w = self._wizard(offsets={"top": 36.0, "bottom": 37.2})
        w._s2_bottom_offset.setValue(9.9)
        w._s2_offset_user_edited = False             # untouched by the user
        w.set_hardware_config(self.host._hardware_config)
        self.assertAlmostEqual(w._s2_bottom_offset.value(), 1.2, places=6)


class TestCaptureAndApply(_Base):

    def test_set_plate_top_captures_and_emits(self):
        """The emit is load-bearing: _set_top_z does not emit on its own, and
        the controller only learns the new top through the changed signal."""
        w = self._wizard()

        def capture():
            self.top_captures.append(True)
            self.host._top_z = 20.0
        self.host._zoff_set_top_z = capture
        w._on_set_plate_top()
        self.assertEqual(self.top_captures, [True])
        self.assertEqual(self.emitted, [True])

    def test_a_failed_top_capture_refuses_inline(self):
        w = self._wizard()                            # capture leaves None
        w._on_set_plate_top()
        self.assertTrue(w._refusal_text)
        self.assertEqual(self.emitted, [])

    def test_apply_derives_through_z_up_sign_and_tags_estimated(self):
        w = self._wizard(top=20.0, z_up=-1.0)
        w._s2_bottom_offset.setValue(1.5)
        w._on_apply_bottom()
        self.assertEqual(len(self.applied), 1)
        z, source = self.applied[0]
        self.assertAlmostEqual(z, 21.5, places=9)     # -1: bottom is LARGER
        self.assertEqual(source, "estimated")

    def test_apply_without_a_top_refuses_inline(self):
        w = self._wizard(top=None)
        w._on_apply_bottom()
        self.assertEqual(self.applied, [])
        self.assertTrue(w._refusal_text)

    def test_apply_button_is_disabled_until_the_top_is_captured(self):
        w = self._wizard(top=None)
        w.refresh()
        self.assertFalse(w._s2_apply_bottom.isEnabled())
        self.host._top_z = 20.0
        w.refresh()
        self.assertTrue(w._s2_apply_bottom.isEnabled())

    def test_set_safe_captures(self):
        w = self._wizard()

        def capture():
            self.safe_captures.append(True)
            self.host._safe_z = 44.0
        self.host._zoff_set_safe_z = capture
        w._on_set_safe()
        self.assertEqual(self.safe_captures, [True])

    def test_the_preview_shows_the_derived_bottom(self):
        w = self._wizard(top=20.0, z_up=1.0)
        w._s2_bottom_offset.setValue(1.5)
        w.refresh()
        self.assertIn("18.500", w._s2_bottom_preview.text())


# ── the host writer on the REAL page ────────────────────────────────

class TestHostWriterOnTheRealPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from unittest.mock import MagicMock
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (1000.0, 2000.0)
        ctrl.z_up_sign.return_value = 1.0
        ctrl.zref_to_user_z.side_effect = lambda z: float(z)
        page = CalibrationPage(ctrl, settings=None)
        self.addCleanup(page.deleteLater)
        return page, ctrl

    def test_apply_sets_the_reference_and_pushes_tagged(self):
        page, ctrl = self._page()
        page._zoff_apply_plate_bottom_z(18.5, source="estimated")
        self.assertEqual(page._plate_bottom_z, 18.5)
        # v7.9.1: the push now also carries the ANCHOR XY — where this height
        # was measured. Without it the plate-bed-level preflight refuses
        # ("the taught Plate Bottom Z has no recorded XY"), which is exactly
        # what blocked the operator's contact touch-off. Assert both, so
        # dropping either the provenance or the anchor fails here.
        ctrl.set_plate_bottom_z.assert_called_with(
            18.5, at_xy_um=(1000.0, 2000.0), source="estimated")
        self.assertEqual(page._plate_bottom_anchor_xy_um, (1000.0, 2000.0))
        self.assertIn("estimated", page._zoff_lbl_plate_bottom_z.text())

    def test_the_wizard_accessor_contract_holds(self):
        """The wizard reaches _zoff_apply_plate_bottom_z by getattr name; the
        AST contract suite checks existence, this checks callability."""
        page, _ = self._page()
        self.assertTrue(callable(page._zoff_apply_plate_bottom_z))


# ── the source tag survives an untagged re-push ─────────────────────

class TestSourceTagSurvivesUntaggedPush(unittest.TestCase):

    def test_untagged_push_keeps_the_estimated_tag(self):
        """gui/app.py re-pushes the plate bottom UNTAGGED on every
        calibration_data_changed. If that cleared the provenance, the
        'plate bottom is only estimated' warnings downstream could never
        fire — set_plate_bottom_z must only overwrite source when given one."""
        from SupportClasses.StageController import StageController
        sc = StageController.__new__(StageController)
        StageController.set_plate_bottom_z(sc, 18.5, source="estimated")
        self.assertEqual(StageController.get_plate_bottom_z_source(sc),
                         "estimated")
        StageController.set_plate_bottom_z(sc, 18.5)          # untagged
        self.assertEqual(StageController.get_plate_bottom_z_source(sc),
                         "estimated")
        self.assertEqual(StageController.get_plate_bottom_z(sc), 18.5)


if __name__ == "__main__":
    unittest.main()
