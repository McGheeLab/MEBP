"""
v7.5.x — Plate template + 3-well quick re-registration.

Scan-once workflow: a completed mosaic's well map is saved as a reusable per-(plate,
camera, objective) TEMPLATE; later the operator re-measures just 3 wells and the whole
plate re-registers by fitting template→measured. Covers:

  * PlateTemplateStore — save/get/has/clear round-trip + corner ref-well pick.
  * PlateWarpCalibrator.register_from_template — recovers a known plate transform
    from 3 reference wells and applies it to ALL template wells.
  * CalibrationPage wiring — save-template helper, re-register button enable gate,
    and the quick-re-register guard when no template exists.
"""

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from SupportClasses.PlateTemplateStore import (  # noqa: E402
    PlateTemplateStore, corner_ref_wells, make_key)
from SupportClasses.PlateWarpCalibrator import register_from_template  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402


def _app():
    return QApplication.instance() or QApplication(sys.argv)


def _grid_template(rows=4, cols=6, pitch=500.0, ox=1000.0, oy=2000.0):
    t = {}
    for i in range(rows):
        for j in range(cols):
            t[f"{chr(65 + i)}{j + 1}"] = (ox + j * pitch, oy + i * pitch)
    return t


# ── PlateTemplateStore ─────────────────────────────────────────────

class TestPlateTemplateStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.store = PlateTemplateStore(Path(self._tmp.name) / "pt.json")

    def tearDown(self):
        self._tmp.cleanup()

    def test_save_and_get_roundtrip(self):
        wells = _grid_template()
        key = self.store.save_template("24", "camA", "4x", wells,
                                       um_per_px=3.1, mosaic_scale=0.026)
        self.assertEqual(key, make_key("24", "camA", "4x"))
        self.assertTrue(self.store.has("24", "camA", "4x"))
        got = self.store.get_wells("24", "camA", "4x")
        self.assertEqual(len(got), 24)
        self.assertEqual(got["A1"], (1000.0, 2000.0))
        refs = self.store.get_ref_wells("24", "camA", "4x")
        self.assertEqual(len(refs), 3)
        self.assertEqual(len(set(refs)), 3)

    def test_persists_across_instances(self):
        self.store.save_template("24", "camA", "4x", _grid_template())
        s2 = PlateTemplateStore(self.store._path)
        self.assertTrue(s2.has("24", "camA", "4x"))
        self.assertEqual(len(s2.get_wells("24", "camA", "4x")), 24)

    def test_camera_objective_keyed(self):
        self.store.save_template("24", "camA", "4x", _grid_template())
        self.assertTrue(self.store.has("24", "camA", "4x"))
        self.assertFalse(self.store.has("24", "camA", "10x"))   # diff objective
        self.assertFalse(self.store.has("24", "camB", "4x"))    # diff camera
        self.assertFalse(self.store.has("96", "camA", "4x"))    # diff plate

    def test_too_few_wells_rejected(self):
        self.assertIsNone(
            self.store.save_template("6", "camA", "4x", {"A1": (0, 0)}))

    def test_clear(self):
        self.store.save_template("24", "camA", "4x", _grid_template())
        self.store.clear("24", "camA", "4x")
        self.assertFalse(self.store.has("24", "camA", "4x"))

    def test_corner_ref_wells_distinct(self):
        refs = corner_ref_wells(_grid_template())
        self.assertEqual(len(refs), 3)
        self.assertEqual(len(set(refs)), 3)
        self.assertIn("A1", refs)        # origin corner


# ── register_from_template ─────────────────────────────────────────

class TestRegisterFromTemplate(unittest.TestCase):
    def test_recovers_known_transform_from_3_wells(self):
        tmpl = _grid_template()
        ang, s, tx, ty = math.radians(6.0), 1.03, 420.0, -260.0
        R = np.array([[math.cos(ang), -math.sin(ang)],
                      [math.sin(ang), math.cos(ang)]])

        def xf(p):
            q = s * (R @ np.array(p))
            return (q[0] + tx, q[1] + ty)
        truth = {n: xf(p) for n, p in tmpl.items()}
        measured = {n: truth[n] for n in ("A1", "A6", "D1")}   # only 3 wells
        reg, warp = register_from_template(tmpl, measured)
        self.assertIsNotNone(warp)
        self.assertEqual(len(reg), 24)
        err = max(math.hypot(reg[n][0] - truth[n][0], reg[n][1] - truth[n][1])
                  for n in tmpl)
        self.assertLess(err, 1.0)        # whole plate recovered from 3 wells

    def test_too_few_shared_returns_empty(self):
        tmpl = _grid_template()
        reg, warp = register_from_template(tmpl, {"A1": (0.0, 0.0)})
        self.assertEqual(reg, {})
        self.assertIsNone(warp)


# ── CalibrationPage wiring ─────────────────────────────────────────

class TestPageTemplateWiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.default_plate_center_um.return_value = (50000.0, 40000.0)
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(24)
        page._camera_manager = None          # objective-only key
        return page

    def test_save_template_and_button_gate(self):
        import SupportClasses.PlateTemplateStore as ptmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        ptmod._store_singleton = PlateTemplateStore(Path(tmp.name) / "pt.json")
        self.addCleanup(lambda: setattr(ptmod, "_store_singleton", None))
        page = self._page()
        # No template yet → button disabled.
        page._ploc_refresh_reregister_button()
        self.assertFalse(page._ploc_btn_reregister.isEnabled())
        # Save the as-built map → template stored, button enables.
        ok = page._ploc_save_plate_template(_grid_template(), um_per_px=3.1)
        self.assertTrue(ok)
        page._ploc_refresh_reregister_button()
        self.assertTrue(page._ploc_btn_reregister.isEnabled())

    def test_quick_reregister_guarded_without_template(self):
        import gui.pages.calibration as calmod
        import SupportClasses.PlateTemplateStore as ptmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        ptmod._store_singleton = PlateTemplateStore(Path(tmp.name) / "pt.json")
        self.addCleanup(lambda: setattr(ptmod, "_store_singleton", None))
        page = self._page()
        shown = []

        class _MB:
            @staticmethod
            def information(*a, **k):
                shown.append(a[2] if len(a) > 2 else "")
            @staticmethod
            def warning(*a, **k):
                shown.append(a[2] if len(a) > 2 else "")
        orig = calmod.QMessageBox
        calmod.QMessageBox = _MB
        try:
            page._ploc_quick_reregister()      # no template → informs, no crash
        finally:
            calmod.QMessageBox = orig
        self.assertTrue(shown)


# ── Per-well rosette mosaic (sub-well mapping) ─────────────────────

def _fake_rosette_plate():
    from SupportClasses.WellPlate import WellInfo

    wells = [
        WellInfo("B1", 1, 0, 0.0, 9.0, 6.0),          # a normal well
        WellInfo("A1.a", 0, 0, 0.0, 0.0, 2.0, is_subwell=True, parent_well="A1"),
        WellInfo("A1.b", 0, 0, 2.0, 0.0, 2.0, is_subwell=True, parent_well="A1"),
        WellInfo("A1.c", 0, 0, 0.0, 2.0, 2.0, is_subwell=True, parent_well="A1"),
        WellInfo("A1.d", 0, 0, 2.0, 2.0, 2.0, is_subwell=True, parent_well="A1"),
    ]

    class _P:
        format = 24
        rows = 4
        cols = 6

        def get_all_wells(self):
            return list(wells)

    return _P()


class TestRosetteSubwell(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.default_plate_center_um.return_value = (50000.0, 40000.0)
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        return CalibrationPage(ctrl, settings=None)

    def test_subplate_adapter(self):
        from gui.pages.calibration import _SubPlate
        sp = _SubPlate(_fake_rosette_plate(), "A1")
        self.assertEqual(sorted(sp.well_names), ["A1.a", "A1.b", "A1.c", "A1.d"])
        self.assertEqual(sp.get_well_position("A1.b"), (2.0, 0.0))
        self.assertEqual(sp.get_well_info("A1.c").diameter, 2.0)
        self.assertEqual(sp.rows, 0)          # → dialog uses 3-corner, not grid
        self.assertEqual(sp.cols, 0)

    def test_rosette_parent_wells(self):
        page = self._page()
        page._plate = _fake_rosette_plate()
        self.assertEqual(page._ploc_rosette_parent_wells(), ["A1"])

    def test_no_rosette_returns_empty(self):
        page = self._page()
        page._plate = WellPlate.from_format(24)   # no sub-wells
        self.assertEqual(page._ploc_rosette_parent_wells(), [])

    def test_subwell_scan_bounds(self):
        page = self._page()
        page._plate = _fake_rosette_plate()
        page._calibrated_positions = None
        page._predicted_positions = {
            "A1.a": (1000.0, 2000.0), "A1.b": (1500.0, 2000.0),
            "A1.c": (1000.0, 2500.0), "A1.d": (1500.0, 2500.0)}
        b = page._ploc_subwell_scan_bounds("A1")
        self.assertIsNotNone(b)
        # bbox x[1000,1500] y[2000,2500]; margin = r(1000) + 1500 = 2500.
        self.assertAlmostEqual(b[0], -1500.0, places=1)
        self.assertAlmostEqual(b[1], -500.0, places=1)
        self.assertAlmostEqual(b[2], 4000.0, places=1)
        self.assertAlmostEqual(b[3], 5000.0, places=1)

    def test_scan_well_guarded_without_rosette(self):
        import gui.pages.calibration as calmod
        page = self._page()
        page._plate = WellPlate.from_format(24)
        shown = []

        class _MB:
            @staticmethod
            def information(*a, **k):
                shown.append(a[2] if len(a) > 2 else "")
            @staticmethod
            def warning(*a, **k):
                shown.append(a[2] if len(a) > 2 else "")
        orig = calmod.QMessageBox
        calmod.QMessageBox = _MB
        try:
            page._ploc_scan_rosette_well()        # no rosette → informs, no crash
        finally:
            calmod.QMessageBox = orig
        self.assertTrue(shown)


if __name__ == "__main__":
    unittest.main()
