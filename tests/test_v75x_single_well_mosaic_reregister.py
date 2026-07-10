"""
test_v75x_single_well_mosaic_reregister.py

v7.5.x — single-well mosaics as a first-class calibration tool + the guided,
orientation-aware well-mapping dialog:

  * Single-well mosaics persist SEPARATELY (``"{plate_key}#{well}"`` keys in
    the same MosaicStore) and never clobber the plate mosaic.
  * The reworked ``MosaicWellMappingDialog``: 180° VIEW rotation (display-only,
    coordinates byte-identical), guided per-well confirm, edge-point circle
    fit, rosette centre → sub-well auto-placement.
  * "Re-register from scanned wells": each picked single-well centre records
    an anchor; 1 anchor = pure translation (mosaic + wells together),
    2 = similarity, 3+ = affine. Anchors are session-only, reset on apply and
    on plate switch.
"""

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402

from PySide6.QtCore import QPointF  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

_APP = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.MosaicStore import MosaicStore  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402


def _app():
    return QApplication.instance() or QApplication(sys.argv)


# ── MosaicStore single-well keys ─────────────────────────────────────

class TestMosaicStoreWellKeys(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.store = MosaicStore(Path(self._tmp.name) / "pm.json")
        self.img = np.full((30, 40, 3), 100, dtype=np.uint8)

    def test_well_key_persists_separately_from_plate(self):
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0))
        self.store.save("96#A1", self.img, (10.0, 20.0, 110.0, 120.0))
        self.assertTrue(self.store.has("96"))
        self.assertTrue(self.store.has_well("96", "A1"))
        self.assertEqual(self.store.list_well_keys("96"), ["A1"])
        # Clearing the well entry leaves the plate mosaic intact.
        self.store.clear("96#A1")
        self.assertFalse(self.store.has_well("96", "A1"))
        self.assertTrue(self.store.has("96"))
        # Plate extent unaffected by the well entry lifecycle.
        self.assertEqual(self.store.get_extent_um("96"),
                         (0.0, 0.0, 400.0, 300.0))

    def test_well_keys_do_not_leak_into_plate_lookup(self):
        self.store.save("96#B2", self.img, (0.0, 0.0, 100.0, 100.0))
        self.assertFalse(self.store.has("96"))
        self.assertEqual(self.store.list_well_keys("96"), ["B2"])
        # Sub-well names ("A1.a") are fine in keys too.
        self.store.save("96#A1.a", self.img, (0.0, 0.0, 50.0, 50.0))
        self.assertIn("A1.a", self.store.list_well_keys("96"))


# ── Reworked mapping dialog ──────────────────────────────────────────

def _swap_training_store(case):
    import SupportClasses.WellTrainingStore as wts
    tmp = tempfile.TemporaryDirectory()
    case.addCleanup(tmp.cleanup)
    wts._store_singleton = wts.WellTrainingStore(Path(tmp.name))
    case.addCleanup(lambda: setattr(wts, "_store_singleton", None))


class TestDialogFlipRotation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _run_corner_flow(self, flip: bool):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog, corner_well_names)
        _swap_training_store(self)
        plate = WellPlate.from_format(6)
        img = np.full((300, 400, 3), 100, dtype=np.uint8)   # featureless
        dlg = MosaicWellMappingDialog(
            plate, img, (0.0, 0.0, 4000.0, 3000.0), 0.1,
            um_per_px=10.0, plate_key="6", plate_flip_180=flip)
        for w in corner_well_names(plate):
            nx, ny = plate.get_well_position(w)
            dlg._on_view_clicked(QPointF(nx * 5 + 60, ny * 5 + 60))
            dlg._on_confirm_well()
        dlg._on_confirm()
        return dlg

    def test_flip_rotates_view_but_results_identical(self):
        d0 = self._run_corner_flow(flip=False)
        d1 = self._run_corner_flow(flip=True)
        # Same click sequence → identical back-projected results.
        r0, r1 = d0.results(), d1.results()
        self.assertEqual(set(r0), set(r1))
        for n in r0:
            self.assertAlmostEqual(r0[n][0], r1[n][0], places=6)
            self.assertAlmostEqual(r0[n][1], r1[n][1], places=6)
        # The flipped view carries a 180° rotation (m11 ≈ m22 ≈ −zoom).
        t0 = d0._view.transform()
        t1 = d1._view.transform()
        self.assertGreater(t0.m11(), 0)
        self.assertLess(t1.m11(), 0)
        self.assertLess(t1.m22(), 0)

    def test_fit_view_preserves_rotation(self):
        d1 = self._run_corner_flow(flip=True)
        d1._fit()
        self.assertLess(d1._view.transform().m11(), 0)


class TestDialogGuidedQueue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dlg(self):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        _swap_training_store(self)
        plate = WellPlate.from_format(6)
        img = np.full((300, 400, 3), 100, dtype=np.uint8)
        return MosaicWellMappingDialog(
            plate, img, (0.0, 0.0, 4000.0, 3000.0), 0.1,
            um_per_px=10.0, plate_key="6"), plate

    def test_click_without_confirm_does_not_advance(self):
        dlg, plate = self._dlg()
        first = dlg._current_name()
        dlg._on_view_clicked(QPointF(100, 100))
        self.assertEqual(dlg._current_name(), first)   # still corner 1
        dlg._on_view_clicked(QPointF(140, 140))        # re-places, no advance
        self.assertEqual(dlg._current_name(), first)
        self.assertEqual(len(dlg._well_items), 1)
        # Marker followed the second click.
        m = dlg._well_items[first]
        c = dlg._marker_center(m)
        self.assertAlmostEqual(c.x(), 140.0, places=3)
        dlg._on_confirm_well()
        self.assertNotEqual(dlg._current_name(), first)

    def test_redo_removes_marker_and_stays(self):
        dlg, plate = self._dlg()
        first = dlg._current_name()
        dlg._on_view_clicked(QPointF(100, 100))
        dlg._on_redo_well()
        self.assertEqual(dlg._current_name(), first)
        self.assertNotIn(first, dlg._well_items)
        self.assertFalse(dlg._btn_confirm_well.isEnabled())

    def test_confirmed_marker_locks(self):
        from PySide6.QtWidgets import QGraphicsEllipseItem
        dlg, plate = self._dlg()
        first = dlg._current_name()
        dlg._on_view_clicked(QPointF(100, 100))
        dlg._on_confirm_well()
        m = dlg._well_items[first]
        self.assertEqual(m.state, "confirmed")
        self.assertFalse(bool(
            m.circle.flags()
            & QGraphicsEllipseItem.GraphicsItemFlag.ItemIsMovable))


class TestDialogEdgeMode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dlg(self):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        _swap_training_store(self)
        plate = WellPlate.from_format(6)
        img = np.full((300, 400, 3), 100, dtype=np.uint8)
        dlg = MosaicWellMappingDialog(
            plate, img, (0.0, 0.0, 4000.0, 3000.0), 0.1,
            um_per_px=10.0, plate_key="6")
        dlg._btn_mode_edge.setChecked(True)     # toggles EDGE mode
        return dlg, plate

    def test_edge_fit_accepts_good_circle(self):
        dlg, plate = self._dlg()
        first = dlg._current_name()
        # 6-well plate: well diameter ~34.8 mm → nominal r_px = d/2·1000·0.1.
        r_px = dlg._nominal_radius_px(first)
        self.assertGreater(r_px, 0)
        cx, cy = 150.0, 120.0
        for ang in (0, 90, 200, 300):
            a = math.radians(ang)
            dlg._on_view_clicked(
                QPointF(cx + r_px * math.cos(a), cy + r_px * math.sin(a)))
        self.assertIsNotNone(dlg._edge_fit)
        self.assertTrue(dlg._btn_confirm_well.isEnabled())
        fcx, fcy, fr = dlg._edge_fit
        self.assertAlmostEqual(fcx, cx, delta=1.0)
        self.assertAlmostEqual(fcy, cy, delta=1.0)
        self.assertAlmostEqual(fr, r_px, delta=1.0)
        dlg._on_confirm_well()
        m = dlg._well_items[first]
        c = dlg._marker_center(m)
        self.assertAlmostEqual(c.x(), cx, delta=1.0)
        self.assertAlmostEqual(c.y(), cy, delta=1.0)

    def test_edge_fit_rejects_wrong_radius(self):
        dlg, plate = self._dlg()
        first = dlg._current_name()
        r_bad = dlg._nominal_radius_px(first) * 3.0    # 3× nominal → reject
        cx, cy = 150.0, 120.0
        for ang in (0, 90, 200, 300):
            a = math.radians(ang)
            dlg._on_view_clicked(
                QPointF(cx + r_bad * math.cos(a), cy + r_bad * math.sin(a)))
        self.assertIsNone(dlg._edge_fit)
        self.assertFalse(dlg._btn_confirm_well.isEnabled())
        # Clear points recovers.
        dlg._clear_edge_state()
        self.assertEqual(dlg._edge_pts, [])

    def test_two_points_not_enough(self):
        dlg, plate = self._dlg()
        dlg._on_view_clicked(QPointF(100, 100))
        dlg._on_view_clicked(QPointF(120, 100))
        self.assertIsNone(dlg._edge_fit)
        self.assertFalse(dlg._btn_confirm_well.isEnabled())


class _FakeSubPlate:
    """3 sub-wells of parent B2, mirroring calibration._SubPlate's surface."""

    def __init__(self):
        self.format = "24#B2"
        self.rows = 0
        self.cols = 0
        self.well_diameter = 0.0
        self._names = ["B2.a", "B2.b", "B2.c"]

    @property
    def well_names(self):
        return list(self._names)

    def get_all_wells(self):
        return []

    def get_well_position(self, name):
        # plate-local mm (unused when predicted_um provided)
        return {"B2.a": (0.0, 0.0), "B2.b": (2.0, 0.0),
                "B2.c": (0.0, 2.0)}[name]

    def get_well_info(self, name):
        raise KeyError(name)


class TestDialogRosetteFlow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dlg(self):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        _swap_training_store(self)
        sub = _FakeSubPlate()
        img = np.full((300, 400, 3), 100, dtype=np.uint8)
        # scale 0.1 px/µm; predicted stage µm: pattern around (20000, 10000)
        predicted = {"B2.a": (19000.0, 10000.0),
                     "B2.b": (21000.0, 10000.0),
                     "B2.c": (20000.0, 12000.0)}
        dlg = MosaicWellMappingDialog(
            sub, img, (0.0, 0.0, 4000.0, 3000.0), 0.1,
            um_per_px=10.0, plate_key="24#B2",
            rosette=True, predicted_um=predicted, center_label="B2")
        return dlg, predicted

    def test_center_confirm_places_subwells_at_offsets(self):
        dlg, predicted = self._dlg()
        self.assertEqual(dlg._current_name(), "⊕centre")
        # Place + confirm the pattern centre at scene (200, 150).
        dlg._on_view_clicked(QPointF(200.0, 150.0))
        dlg._on_confirm_well()
        self.assertEqual(len(dlg._well_items), 3)
        # Expected: centre + (predicted − centroid) × scale.
        cx = sum(p[0] for p in predicted.values()) / 3.0
        cy = sum(p[1] for p in predicted.values()) / 3.0
        for n, (px_um, py_um) in predicted.items():
            m = dlg._well_items[n]
            c = dlg._marker_center(m)
            self.assertAlmostEqual(
                c.x(), 200.0 + (px_um - cx) * 0.1, places=3, msg=n)
            self.assertAlmostEqual(
                c.y(), 150.0 + (py_um - cy) * 0.1, places=3, msg=n)

    def test_confirm_exports_subwells_not_center(self):
        dlg, predicted = self._dlg()
        dlg._on_view_clicked(QPointF(200.0, 150.0))
        dlg._on_confirm_well()
        # Drag one sub-well by (+10, −5) scene px.
        m = dlg._well_items["B2.b"]
        m.circle.setPos(10.0, -5.0)
        dlg._on_confirm()
        res = dlg.results()
        self.assertEqual(set(res), {"B2.a", "B2.b", "B2.c"})
        self.assertNotIn("⊕centre", res)
        # The dragged sub-well's µm shifted by d/scale.
        cx = sum(p[0] for p in predicted.values()) / 3.0
        base_px = 200.0 + (predicted["B2.b"][0] - cx) * 0.1
        self.assertAlmostEqual(res["B2.b"][0], (base_px + 10.0) / 0.1,
                               places=3)

    def test_confirm_requires_center_first(self):
        dlg, _ = self._dlg()
        dlg._on_confirm()                        # nothing placed
        self.assertEqual(dlg.results(), {})

    def test_skip_all_exports_autoplaced(self):
        dlg, predicted = self._dlg()
        dlg._on_view_clicked(QPointF(200.0, 150.0))
        dlg._on_confirm_well()
        for _ in range(3):
            dlg._on_skip_well()
        dlg._on_confirm()
        self.assertEqual(len(dlg.results()), 3)


# ── CalibrationPage: single-well flow + re-register ──────────────────

class _CalBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _make_page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.get_zp_position.return_value = (None, None, None)
        ctrl.default_plate_center_um.return_value = (50000.0, 40000.0)
        ctrl.plate_axis_sign.return_value = (1.0, 1.0)
        ctrl.plate_flip_180.return_value = False
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(6)
        page._ploc_plate_key = lambda: "test-plate"
        return page

    @staticmethod
    def _grid(page):
        return page._plate.get_all_positions_from_plate_center(
            50000.0, 40000.0)

    def _wire_map(self, page):
        grid = self._grid(page)
        page._predicted_positions = dict(grid)
        page._calibrated_positions = dict(grid)
        page._reference_markers = {}
        page._xy_teach_points = {}
        page._refresh_ploc_view = lambda: None
        page._emit_calibration_data_changed = lambda: None
        page._save_calibration = lambda **k: None
        page._ploc_shift_mosaic_by = MagicMock(return_value=False)
        return grid

    def _swap_mosaic_store(self):
        import SupportClasses.MosaicStore as msmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        msmod._store_singleton = msmod.MosaicStore(Path(tmp.name) / "pm.json")
        self.addCleanup(lambda: setattr(msmod, "_store_singleton", None))
        return msmod._store_singleton


class TestSingleWellFinishPath(_CalBase):
    def test_finish_persists_well_mosaic_and_not_plate(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        opened = []
        page._ploc_open_single_well_mapping = (
            lambda *a, **k: opened.append(a))
        page._ploc_mosaic_running = True
        page._ploc_scan_subwell_parent = "B2"
        page._ploc_scan_well_is_rosette = False
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        page._ploc_on_mosaic_finished(
            img, (100.0, 200.0, 500.0, 500.0), 0.1, 4, [])
        # Persisted under the WELL key; the plate mosaic untouched.
        self.assertTrue(store.has_well("test-plate", "B2"))
        self.assertFalse(store.has("test-plate"))
        self.assertEqual(store.get_extent_um("test-plate#B2"),
                         (100.0, 200.0, 500.0, 500.0))
        # Routed to the single-well opener with the rosette flag.
        self.assertEqual(len(opened), 1)
        self.assertEqual(opened[0][0], "B2")
        self.assertFalse(opened[0][4])          # is_rosette=False
        # Flags reset.
        self.assertIsNone(page._ploc_scan_subwell_parent)
        self.assertFalse(page._ploc_scan_well_is_rosette)


class _FakeMappingDialog:
    """Recording stand-in for MosaicWellMappingDialog."""
    last_kwargs = None
    next_results = {}

    def __init__(self, *args, **kwargs):
        type(self).last_kwargs = dict(kwargs)
        type(self).last_args = args

    def exec(self):
        return True

    def results(self):
        return dict(type(self).next_results)

    def saved_sample_path(self):
        return None


class TestSingleWellPickAndAnchor(_CalBase):
    def test_plain_well_pick_merges_and_records_anchor(self):
        page = self._make_page()
        grid = self._wire_map(page)
        map_b2 = grid["B2"]
        measured = (map_b2[0] + 250.0, map_b2[1] - 100.0)
        _FakeMappingDialog.next_results = {"B2": measured}
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        with patch("gui.dialogs.mosaic_well_mapping_dialog"
                   ".MosaicWellMappingDialog", _FakeMappingDialog):
            page._ploc_open_single_well_mapping(
                "B2", img, (0.0, 0.0, 400.0, 300.0), 0.1, False)
        # Merged into the calibration + reference markers.
        self.assertEqual(page._calibrated_positions["B2"], measured)
        self.assertEqual(page._reference_markers["B2"], measured)
        # Anchor recorded with the PRE-merge map position.
        a = page._ploc_well_anchors["B2"]
        self.assertEqual(a["measured"], measured)
        self.assertEqual(a["map"], tuple(map_b2))
        self.assertIn("(1)", page._ploc_btn_scan_reregister.text())
        self.assertTrue(page._ploc_btn_scan_reregister.isEnabled())
        # Dialog got the single-well kwargs.
        kw = _FakeMappingDialog.last_kwargs
        self.assertFalse(kw["rosette"])
        self.assertEqual(kw["center_label"], "B2")
        self.assertEqual(kw["plate_key"], "test-plate#B2")
        self.assertIn("plate_flip_180", kw)

    def test_rosette_pick_adds_parent_centroid_fallback(self):
        page = self._make_page()
        self._wire_map(page)
        # Dialog returns only sub-well names → parent = centroid fallback.
        # (format-6 plate has no real sub-wells → stand in for _SubPlate too.)
        _FakeMappingDialog.next_results = {
            "B2.a": (100.0, 200.0), "B2.b": (300.0, 200.0),
            "B2.c": (200.0, 400.0)}
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        with patch("gui.dialogs.mosaic_well_mapping_dialog"
                   ".MosaicWellMappingDialog", _FakeMappingDialog), \
             patch("gui.pages.calibration._SubPlate",
                   lambda plate, parent: _FakeSubPlate()):
            page._ploc_open_single_well_mapping(
                "B2", img, (0.0, 0.0, 400.0, 300.0), 0.1, True)
        self.assertEqual(page._calibrated_positions["B2"], (200.0, 800.0 / 3))
        self.assertIn("B2.a", page._calibrated_positions)
        self.assertIn("B2", page._ploc_well_anchors)
        kw = _FakeMappingDialog.last_kwargs
        self.assertTrue(kw["rosette"])


class TestReregisterFromScanned(_CalBase):
    def test_one_anchor_translation(self):
        page = self._make_page()
        grid = self._wire_map(page)
        map_b2 = grid["B2"]
        page._ploc_well_anchors["B2"] = {
            "measured": (map_b2[0] + 500.0, map_b2[1] - 200.0),
            "map": tuple(map_b2)}
        page._ploc_reregister_from_scanned()
        # Every well shifted by exactly (500, −200).
        for n, (gx, gy) in grid.items():
            self.assertAlmostEqual(
                page._calibrated_positions[n][0], gx + 500.0, places=4, msg=n)
            self.assertAlmostEqual(
                page._calibrated_positions[n][1], gy - 200.0, places=4, msg=n)
        # Mosaic followed (translation path shifts via the shared helper).
        page._ploc_shift_mosaic_by.assert_called_once()
        args = page._ploc_shift_mosaic_by.call_args[0]
        self.assertAlmostEqual(args[0], 500.0, places=4)
        self.assertAlmostEqual(args[1], -200.0, places=4)
        # Anchors consumed.
        self.assertEqual(page._ploc_well_anchors, {})
        self.assertFalse(page._ploc_btn_scan_reregister.isEnabled())

    def test_one_anchor_no_double_correction_of_merged_centres(self):
        """The pick flow merges the MEASURED centre into the map, so the plain
        translation would move the anchor well AGAIN (to measured+E). The
        merged pins must keep every measured centre exactly where it was
        measured while the stale wells shift by E."""
        page = self._make_page()
        grid = self._wire_map(page)
        map_b2 = grid["B2"]
        measured = (map_b2[0] + 500.0, map_b2[1] - 200.0)
        sub_meas = {"B2.a": (measured[0] - 900.0, measured[1]),
                    "B2.b": (measured[0] + 900.0, measured[1])}
        # Simulate the pick: measured centres already merged into the map.
        page._calibrated_positions["B2"] = measured
        page._calibrated_positions.update(sub_meas)
        page._reference_markers.update({"B2": measured, **sub_meas})
        page._ploc_well_anchors["B2"] = {
            "measured": measured, "map": tuple(map_b2),
            "merged": {"B2": measured, **sub_meas}}
        page._ploc_reregister_from_scanned()
        # Anchor well + its sub-wells stay EXACTLY at their measured values.
        self.assertEqual(page._calibrated_positions["B2"], measured)
        for n, p in sub_meas.items():
            self.assertEqual(page._calibrated_positions[n], p)
            self.assertEqual(page._reference_markers[n], p)
        # Every OTHER well shifted by E = (500, −200).
        for n, (gx, gy) in grid.items():
            if n == "B2":
                continue
            self.assertAlmostEqual(
                page._calibrated_positions[n][0], gx + 500.0, places=4, msg=n)
            self.assertAlmostEqual(
                page._calibrated_positions[n][1], gy - 200.0, places=4, msg=n)

    def test_two_anchor_pins_merged_subwells(self):
        """2+ fit: merged sub-well centres are measured ground truth — they
        must not be re-warped by the plate fit."""
        page = self._make_page()
        grid = self._wire_map(page)
        t = (300.0, -150.0)

        def _xf(p):
            return (p[0] + t[0], p[1] + t[1])

        subs = {"A1.a": (12345.0, 23456.0)}
        page._calibrated_positions["A1"] = _xf(grid["A1"])
        page._calibrated_positions["B3"] = _xf(grid["B3"])
        page._calibrated_positions.update(subs)
        page._ploc_well_anchors["A1"] = {
            "measured": _xf(grid["A1"]), "map": tuple(grid["A1"]),
            "merged": {"A1": _xf(grid["A1"]), **subs}}
        page._ploc_well_anchors["B3"] = {
            "measured": _xf(grid["B3"]), "map": tuple(grid["B3"]),
            "merged": {"B3": _xf(grid["B3"])}}
        page._ploc_reregister_from_scanned()
        # The merged sub-well is pinned verbatim, not warped.
        self.assertEqual(page._calibrated_positions["A1.a"], subs["A1.a"])
        # Anchor parents land exactly at measured.
        for n in ("A1", "B3"):
            self.assertAlmostEqual(page._calibrated_positions[n][0],
                                   _xf(grid[n])[0], places=4)
            self.assertAlmostEqual(page._calibrated_positions[n][1],
                                   _xf(grid[n])[1], places=4)

    def test_two_anchor_similarity(self):
        page = self._make_page()
        grid = self._wire_map(page)
        # Known similarity: rotate 2° about origin + translate (300, −150).
        th = math.radians(2.0)
        c, s = math.cos(th), math.sin(th)

        def _xf(p):
            return (c * p[0] - s * p[1] + 300.0,
                    s * p[0] + c * p[1] - 150.0)

        for n in ("A1", "B3"):
            page._ploc_well_anchors[n] = {
                "measured": _xf(grid[n]), "map": tuple(grid[n])}
        page._ploc_reregister_from_scanned()
        for n, p in grid.items():
            ex = _xf(p)
            self.assertAlmostEqual(page._calibrated_positions[n][0], ex[0],
                                   delta=1.0, msg=n)
            self.assertAlmostEqual(page._calibrated_positions[n][1], ex[1],
                                   delta=1.0, msg=n)
        # Explicit positions supersede the warp state.
        self.assertIsNone(page._plate_warp)
        self.assertIsNone(page._three_well_calibration)
        self.assertEqual(page._ploc_well_anchors, {})

    def test_three_anchor_affine(self):
        page = self._make_page()
        grid = self._wire_map(page)

        def _xf(p):
            return (1.001 * p[0] + 100.0, 0.999 * p[1] - 80.0)

        for n in ("A1", "A3", "B1"):
            page._ploc_well_anchors[n] = {
                "measured": _xf(grid[n]), "map": tuple(grid[n])}
        page._ploc_reregister_from_scanned()
        for n, p in grid.items():
            ex = _xf(p)
            self.assertAlmostEqual(page._calibrated_positions[n][0], ex[0],
                                   delta=1.0, msg=n)
            self.assertAlmostEqual(page._calibrated_positions[n][1], ex[1],
                                   delta=1.0, msg=n)

    def test_mosaic_shift_translation_component(self):
        page = self._make_page()
        grid = self._wire_map(page)
        # Pure-translation 2-anchor fit → mosaic shift = that translation.
        page._ploc_overlay_ext = (0.0, 0.0, 1000.0, 1000.0)
        for n in ("A1", "B3"):
            page._ploc_well_anchors[n] = {
                "measured": (grid[n][0] + 120.0, grid[n][1] + 60.0),
                "map": tuple(grid[n])}
        page._ploc_reregister_from_scanned()
        page._ploc_shift_mosaic_by.assert_called_once()
        args = page._ploc_shift_mosaic_by.call_args[0]
        self.assertAlmostEqual(args[0], 120.0, delta=0.5)
        self.assertAlmostEqual(args[1], 60.0, delta=0.5)

    def test_implausible_fit_prompt_no_keeps_map(self):
        import gui.pages.calibration as calmod
        page = self._make_page()
        grid = self._wire_map(page)
        # Degenerate: two anchors measured at nearly the SAME point → huge
        # scale collapse → implausible.
        page._ploc_well_anchors["A1"] = {
            "measured": (50000.0, 40000.0), "map": tuple(grid["A1"])}
        page._ploc_well_anchors["B3"] = {
            "measured": (50000.5, 40000.5), "map": tuple(grid["B3"])}

        class _MB:
            StandardButton = calmod.QMessageBox.StandardButton
            @staticmethod
            def question(*a, **k):
                return calmod.QMessageBox.StandardButton.No
            @staticmethod
            def warning(*a, **k):
                return None
            @staticmethod
            def information(*a, **k):
                return None
        orig = calmod.QMessageBox
        calmod.QMessageBox = _MB
        try:
            page._ploc_reregister_from_scanned()
        finally:
            calmod.QMessageBox = orig
        # Map unchanged; anchors kept (operator can re-pick / retry).
        for n, p in grid.items():
            self.assertEqual(page._calibrated_positions[n], p)
        self.assertEqual(len(page._ploc_well_anchors), 2)

    def test_plate_switch_clears_anchors(self):
        page = self._make_page()
        self._wire_map(page)
        page._ploc_well_anchors["A1"] = {
            "measured": (1.0, 2.0), "map": (0.0, 0.0)}
        page._ploc_refresh_scanned_reregister_button()
        self.assertTrue(page._ploc_btn_scan_reregister.isEnabled())
        page._reset_calibration_state()
        self.assertEqual(page._ploc_well_anchors, {})
        self.assertFalse(page._ploc_btn_scan_reregister.isEnabled())

    def test_button_label_tracks_count(self):
        page = self._make_page()
        page._ploc_refresh_scanned_reregister_button()
        self.assertIn("(0)", page._ploc_btn_scan_reregister.text())
        self.assertFalse(page._ploc_btn_scan_reregister.isEnabled())
        page._ploc_well_anchors = {"A1": {}, "B2": {}}
        page._ploc_refresh_scanned_reregister_button()
        self.assertIn("(2)", page._ploc_btn_scan_reregister.text())
        self.assertTrue(page._ploc_btn_scan_reregister.isEnabled())


if __name__ == "__main__":
    unittest.main()
