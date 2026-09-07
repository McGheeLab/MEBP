"""
v7.13 — the restructured Needle Location tab + the focal-plane ↔ needle-tip
constant.

Layout: one BIG camera pane (a QStackedWidget the wizard drives via
step_changed — side cams for steps 1-2, the wizard's microscope pane for
steps 3-4) beside a compact wizard column. The legacy side-camera bore group
is retired from view but must stay parented (its own setVisible(True) inside
_bore_cal_refresh would otherwise float a top-level window).

The f–z constant: every optical rung pairs a fitted tip focus with the needle
Z read back at that moment, so each is an independent estimate of the offset in
``needle_z = focal_sign × f/1000 + offset``. Constancy across rungs IS the
verification. The datum's writer and reader used to build keys that could never
match — that pin lives here too.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtGui import QShowEvent
from PySide6.QtWidgets import QApplication

from SupportClasses.PhysicalModels import NeedleBore, NeedleSpec
from SupportClasses.PlateBottomOptical import (
    RungMeasurement,
    focus_needle_offset_mm,
)
from gui.widgets.needle_bore_wizard import (
    STEP_BORES,
    STEP_NEEDLE_ZERO,
    NeedleBoreWizard,
)


def _backpack():
    return NeedleSpec(needle_form="backpack", bores=[
        NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
        NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P2")])


# ── the pure f–z constant ───────────────────────────────────────────

class TestFocusNeedleOffset(unittest.TestCase):

    def _rung(self, z, f, refusal=""):
        return RungMeasurement(margin_um=500.0, needle_z_zref_mm=z,
                               focus_tip_um=f, refusal=refusal)

    def test_the_offset_is_constant_across_a_clean_ladder(self):
        """f=5000 µm paired with z=6.0+5.0 → offset 6.0, at every rung."""
        rungs = [self._rung(11.0, 5000.0), self._rung(10.5, 4500.0),
                 self._rung(10.2, 4200.0)]
        off, spread, n = focus_needle_offset_mm(rungs, +1.0)
        self.assertAlmostEqual(off, 6.0, places=9)
        self.assertAlmostEqual(spread, 0.0, places=9)
        self.assertEqual(n, 3)

    def test_a_flipped_sign_changes_the_answer(self):
        """The mutation check: z − f/1000 vs z + f/1000 differ by 2f/1000."""
        rungs = [self._rung(11.0, 5000.0)]
        plus, _, _ = focus_needle_offset_mm(rungs, +1.0)
        minus, _, _ = focus_needle_offset_mm(rungs, -1.0)
        self.assertAlmostEqual(minus - plus, 10.0, places=9)

    def test_the_median_resists_one_outlier(self):
        rungs = [self._rung(11.0, 5000.0), self._rung(10.5, 4500.0),
                 self._rung(10.9, 4200.0)]     # outlier: offset 6.7
        off, spread, n = focus_needle_offset_mm(rungs, +1.0)
        self.assertAlmostEqual(off, 6.0, places=9)
        self.assertAlmostEqual(spread, 0.7, places=9)

    def test_refused_rungs_are_excluded(self):
        rungs = [self._rung(11.0, 5000.0),
                 self._rung(99.0, 4500.0, refusal="NO_PEAK")]
        off, _, n = focus_needle_offset_mm(rungs, +1.0)
        self.assertAlmostEqual(off, 6.0, places=9)
        self.assertEqual(n, 1)

    def test_no_usable_rungs_returns_none(self):
        off, spread, n = focus_needle_offset_mm([], +1.0)
        self.assertIsNone(off)
        self.assertEqual((spread, n), (0.0, 0))


# ── the datum key: writer == reader ─────────────────────────────────

class TestDatumKeyAgreement(unittest.TestCase):

    def _shared_host(self):
        mgr = SimpleNamespace(
            camera_identity=lambda idx: ("camX_5&abc", "Andor Zyla"),
            cameras=[None] * 4,
            is_um_per_px_calibrated=lambda idx: True,
            effective_um_per_px=lambda idx, w: 0.5,
            start=lambda idx: None)
        return SimpleNamespace(
            controller=None,
            _camera_manager=mgr,
            _hardware_config=SimpleNamespace(
                needle=_backpack(),
                camera_config=SimpleNamespace(
                    current_objective_name="10x Plan Fluor")),
            _plate_bottom_z=10.0, _safe_z=44.0, _top_z=None, _max_z=None,
            _needle_origin_um=(1.0, 2.0), _needle_loc_xy_um=None,
            _ploc_microscope_cam_idx=lambda: 2,
            _ploc_plate_key=lambda: "plate-24_Rosette",
        )

    def test_writer_and_reader_build_the_same_key(self):
        """The v7.13 fix: the wizard wrote 'cam|obj' as the CAMERA half with
        an empty objective, the plate-level wizard read the bare camera with
        an empty objective — keys that could never be equal, so the stored
        datum was unreadable. Both now resolve (bare identity, objective)."""
        from SupportClasses.PlateFocusDatumStore import datum_key
        import gui.widgets.plate_level_wizard as plw

        host = self._shared_host()
        app = QApplication.instance() or QApplication(sys.argv)  # noqa: F841
        wiz = NeedleBoreWizard(host)
        self.addCleanup(wiz.deleteLater)
        writer_key = datum_key(wiz._datum_camera_identity(),
                               wiz._datum_objective_name(),
                               wiz._plate_key())

        reader = plw.PlateLevelWizard.__new__(plw.PlateLevelWizard)
        reader._host = host
        reader_key = datum_key(
            plw.PlateLevelWizard._camera_name(reader),
            plw.PlateLevelWizard._objective_name(reader),
            plw.PlateLevelWizard._plate_key(reader))
        self.assertEqual(writer_key, reader_key)
        self.assertIn("camX_5_abc", writer_key)      # bare identity, mangled
        self.assertIn("10x", writer_key)             # real objective name


class TestWriteFocusDatum(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_fzdatum_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_PLATE_FOCUS_DATUM_DIR")
        os.environ["MEBP_PLATE_FOCUS_DATUM_DIR"] = self._tmp.name
        import SupportClasses.PlateFocusDatumStore as mod
        mod.reset_store()

        def _restore():
            if self._prev is None:
                os.environ.pop("MEBP_PLATE_FOCUS_DATUM_DIR", None)
            else:
                os.environ["MEBP_PLATE_FOCUS_DATUM_DIR"] = self._prev
            mod.reset_store()
        self.addCleanup(_restore)

    def test_the_datum_carries_the_rung_median_offset_and_the_epoch(self):
        ctrl = SimpleNamespace(
            is_xy_connected=True,
            zero_position={"x": 0.0, "y": 0.0, "Z": 3.25},
            get_xy_position=lambda cached=False: (100_000.0, 50_000.0),
            print_z_dir=lambda: 1.0,
            z_up_sign=lambda: 1.0)
        host = SimpleNamespace(
            controller=ctrl,
            _camera_manager=SimpleNamespace(
                camera_identity=lambda idx: ("camX", "name"),
                cameras=[None] * 4,
                is_um_per_px_calibrated=lambda idx: True,
                effective_um_per_px=lambda idx, w: 0.5,
                start=lambda idx: None),
            _hardware_config=SimpleNamespace(
                needle=_backpack(),
                camera_config=SimpleNamespace(
                    current_objective_name="10x")),
            _plate_bottom_z=10.0, _safe_z=44.0, _top_z=None, _max_z=None,
            _needle_origin_um=(1.0, 2.0), _needle_loc_xy_um=None,
            _ploc_microscope_cam_idx=lambda: 2,
            _ploc_plate_key=lambda: "p24",
        )
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        w._scope = lambda: None          # no live microscope in the test
        w._opt_f0 = 5000.0
        w._opt_rungs = [
            RungMeasurement(margin_um=1000.0, needle_z_zref_mm=11.0,
                            focus_tip_um=5000.0),
            RungMeasurement(margin_um=500.0, needle_z_zref_mm=10.5,
                            focus_tip_um=4500.0),
        ]
        rec = SimpleNamespace(plate_bottom_zref_mm=10.0, scale=1.0,
                              n_used=2, span_um=500.0)
        w._write_focus_datum(rec)

        from SupportClasses.PlateFocusDatumStore import get_store
        stored = get_store().get("camX", "10x", "p24")
        self.assertIsNotNone(stored, "the datum must be written under the "
                                     "(bare camera, objective, plate) key")
        # Tip-pair median: 11.0 − 5.0 = 10.5 − 4.5 = 6.0 (sign +1)
        self.assertAlmostEqual(stored["offset_mm"], 6.0, places=9)
        self.assertEqual(stored["focal_sign"], 1)
        # The zero-Z epoch makes is_stale() live — it used to be omitted.
        self.assertAlmostEqual(stored["zero_z_mm"], 3.25, places=9)
        self.assertTrue(get_store().is_stale("camX", "10x", "p24",
                                             current_zero_z_mm=3.40))
        self.assertFalse(get_store().is_stale("camX", "10x", "p24",
                                              current_zero_z_mm=3.25))


# ── the restructured tab on the REAL page ───────────────────────────

class TestNeedleLocTabLayout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, needle=None):
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
        if needle is not None:
            page._hardware_config = SimpleNamespace(needle=needle)
        return page

    def test_the_camera_stack_exists_with_two_pages(self):
        page = self._page()
        self.assertEqual(page._needle_loc_camera_stack.count(), 2)

    def test_the_wizard_step_drives_the_stack(self):
        page = self._page()
        wiz = page._needle_bore_wizard
        wiz.go_to_step("bores")
        self.assertEqual(page._needle_loc_camera_stack.currentIndex(), 1)
        wiz.go_to_step("touchoff")
        self.assertEqual(page._needle_loc_camera_stack.currentIndex(), 1)
        wiz.go_to_step("needle_zero")
        self.assertEqual(page._needle_loc_camera_stack.currentIndex(), 0)

    def test_the_microscope_pane_is_page_one_of_the_stack(self):
        page = self._page()
        wiz = page._needle_bore_wizard
        self.assertIs(page._needle_loc_camera_stack.widget(1),
                      wiz.microscope_pane())

    def test_the_step1_controls_live_inside_the_wizard_page(self):
        """set_step_panel mounted the centring controls on step 1 — the
        attribute names are unchanged, only where they render moved."""
        page = self._page()
        wiz = page._needle_bore_wizard
        step1 = wiz._pages[STEP_NEEDLE_ZERO]
        btn = page._needle_loc_btn_center
        parent = btn.parentWidget()
        while parent is not None and parent is not step1:
            parent = parent.parentWidget()
        self.assertIs(parent, step1)

    def test_the_legacy_bore_group_is_parented_and_hidden(self):
        """Its own setVisible(True) fires for a multi-bore needle; without a
        hidden parent it would float as a top-level window."""
        page = self._page(needle=_backpack())
        page._needle_loc_camera_info = lambda role: (1.67, 640, 45.0)
        page._bore_cal_refresh()
        grp = page._bore_cal_group
        self.assertFalse(grp.isWindow())
        self.assertIsNotNone(grp.parentWidget())
        self.assertFalse(grp.isVisibleTo(page))

    def test_the_wizard_column_has_no_burying_scroll(self):
        """The wizard itself must not sit inside a QScrollArea (the old layout
        buried it); only the Advanced expander's content keeps its scroll."""
        from PySide6.QtWidgets import QScrollArea
        page = self._page()
        w = page._needle_bore_wizard.parentWidget()
        while w is not None:
            self.assertNotIsInstance(w.parentWidget(), QScrollArea)
            w = w.parentWidget()

    def test_the_advanced_z_panel_is_OPEN_by_default(self):
        """v7.17.1 — contract deliberately inverted (operator request).

        It used to start collapsed, on the reasoning that step 2 carried the
        primary flow and this was the escape hatch. In practice the panel holds
        the reference-height picker, the live XZ side view used to drive Z, and
        the plate-type learn/apply round trip — so hiding it hid the controls
        operators reach for most, and the "assign to plate type" button in
        particular was effectively undiscoverable.

        It is now open, and lives in a vertical splitter so it can be dragged
        much taller than the old fixed half-column. The toggle still exists for
        reclaiming the space.
        """
        page = self._page()
        self.assertTrue(page._needle_loc_adv_btn.isChecked())
        # The reference-height labels still exist (setters write them).
        self.assertTrue(hasattr(page, "_zoff_lbl_plate_bottom_z"))


class TestWizardOpensAndArms(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_wiz13_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
            Path(self._tmp.name) / "bore.json")
        import SupportClasses.NeedleBoreCalibrationStore as mod
        self._prev_singleton = mod._store
        mod._store = None

        def _restore():
            mod._store = self._prev_singleton
            if self._prev is None:
                os.environ.pop("MEBP_NEEDLE_BORE_CAL_PATH", None)
            else:
                os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = self._prev
        self.addCleanup(_restore)

    def _wizard(self, origin=(1.0, 2.0), bottom=10.0):
        floor_calls = []
        ctrl = SimpleNamespace(
            is_xy_connected=True,
            get_xy_position=lambda cached=False: (100_000.0, 50_000.0),
            z_up_sign=lambda: 1.0,
            print_z_dir=lambda: 1.0,
            print_height_to_zref=lambda h: 10.0 + float(h),
            get_needle_camera_offset_um=lambda: None,
            set_print_floor_active=lambda on: floor_calls.append(bool(on)))
        host = SimpleNamespace(
            controller=ctrl,
            _hardware_config=SimpleNamespace(needle=_backpack()),
            _camera_manager=SimpleNamespace(
                cameras=[None] * 4,
                is_um_per_px_calibrated=lambda idx: True,
                effective_um_per_px=lambda idx, w: 0.5,
                start=lambda idx: None),
            _plate_bottom_z=bottom, _safe_z=44.0, _top_z=None, _max_z=None,
            _needle_origin_um=origin, _needle_loc_xy_um=None,
            _ploc_microscope_cam_idx=lambda: 2,
        )
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        w.floor_calls = floor_calls
        return w

    def test_opens_on_the_first_incomplete_step(self):
        self.assertEqual(self._wizard(origin=None).current_step,
                         STEP_NEEDLE_ZERO)
        self.assertEqual(self._wizard().current_step, STEP_BORES)

    def test_construction_commands_nothing(self):
        w = self._wizard()
        self.assertEqual(w.floor_calls, [])

    def test_show_rearms_the_floor_on_an_armed_step(self):
        """hideEvent disarms; without a showEvent re-arm the floor stayed OFF
        after a page revisit until a strip button happened to be clicked."""
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        self.assertEqual(w.floor_calls, [True])
        from PySide6.QtGui import QHideEvent
        w.hideEvent(QHideEvent())
        self.assertEqual(w.floor_calls, [True, False])
        w.showEvent(QShowEvent())
        self.assertEqual(w.floor_calls, [True, False, True])

    def test_show_emits_step_changed_so_the_host_pane_syncs(self):
        w = self._wizard()
        got = []
        w.step_changed.connect(got.append)
        w.showEvent(QShowEvent())
        self.assertEqual(got, [w.current_step])

    def test_back_next_walk_the_step_order(self):
        w = self._wizard(origin=None)      # opens on step 1
        w._on_next()
        self.assertEqual(w.current_step, "z_refs")
        w._on_back()
        self.assertEqual(w.current_step, "needle_zero")
        w._on_back()                        # clamped at the first step
        self.assertEqual(w.current_step, "needle_zero")


if __name__ == "__main__":
    unittest.main()
