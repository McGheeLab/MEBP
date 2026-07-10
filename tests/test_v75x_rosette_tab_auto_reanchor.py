"""
test_v75x_rosette_tab_auto_reanchor.py

v7.5.x — operator-feedback batch on plate calibration:

  * BUG FIX: the mapping dialog back-projects in the TRUSTED stage frame —
    the extent handed to it has the global registration shift REMOVED
    (``MosaicStore`` records ``shift_um``; both the full-plate and single-well
    paths subtract it). Zero shift ⇒ byte-identical.
  * Plate-level mapping = MAIN wells only (flattened rosette sub-wells are
    excluded; dropped rosette parents re-added at their pattern centroid).
  * Four-button Mosaic group + Advanced… menu; Map-wells auto-opens after a
    full-plate scan.
  * Auto re-anchor: feature patch saved from the manual re-anchor's live
    click (``ReanchorFeatureStore``), re-found hands-free by template match
    (``VisionDetector.find_template``) → global translation.
  * Rosettes tab (index 3; Plate Z Auto-Cal bumped to 4).
  * Single-well mosaics composite onto the plate mosaic
    (``MosaicStore.composite_with_wells``), toggleable.
  * "Mosaic + well" view = outline-only circles (incl. sub-wells).
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402

from PySide6.QtWidgets import QApplication  # noqa: E402

_APP = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.MosaicStore import (  # noqa: E402
    MosaicStore, composite_with_wells)
from SupportClasses.ReanchorFeatureStore import ReanchorFeatureStore  # noqa: E402
from SupportClasses.VisionDetector import find_template  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402


def _app():
    return QApplication.instance() or QApplication(sys.argv)


# ── MosaicStore: shift metadata + extent update + composite ──────────

class TestMosaicStoreShift(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.store = MosaicStore(Path(self._tmp.name) / "pm.json")
        self.img = np.full((30, 40, 3), 100, dtype=np.uint8)

    def test_shift_um_round_trip(self):
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0),
                        shift_um=(12.5, -3.25))
        self.assertEqual(self.store.get_shift_um("96"), (12.5, -3.25))
        # Persists across instances.
        s2 = MosaicStore(self.store._path)
        self.assertEqual(s2.get_shift_um("96"), (12.5, -3.25))

    def test_legacy_entry_shift_is_zero(self):
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0))
        meta = self.store.get_meta("96")
        meta.pop("shift_um", None)          # simulate a legacy entry
        self.store._save_meta()
        self.assertEqual(self.store.get_shift_um("96"), (0.0, 0.0))
        self.assertEqual(self.store.get_shift_um("nope"), (0.0, 0.0))

    def test_update_extent_metadata_only(self):
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0),
                        shift_um=(5.0, 6.0))
        ok = self.store.update_extent("96", (10.0, 20.0, 410.0, 320.0))
        self.assertTrue(ok)
        self.assertEqual(self.store.get_extent_um("96"),
                         (10.0, 20.0, 410.0, 320.0))
        # shift_um + image survive the metadata rewrite.
        self.assertEqual(self.store.get_shift_um("96"), (5.0, 6.0))
        self.assertIsNotNone(self.store.load_image("96"))
        self.assertFalse(self.store.update_extent("nope", (0, 0, 1, 1)))

    def test_composite_with_wells_pastes_at_extent(self):
        # Plate: 100×100 px over (0..1000, 0..1000) µm → 0.1 px/µm.
        plate = np.zeros((100, 100, 3), dtype=np.uint8)
        self.store.save("96", plate, (0.0, 0.0, 1000.0, 1000.0))
        # Well mosaic: solid white, extent (200..400, 300..500) µm.
        well = np.full((20, 20, 3), 255, dtype=np.uint8)
        self.store.save("96#B2", well, (200.0, 300.0, 400.0, 500.0))
        comp, ext = composite_with_wells(
            self.store, "96", plate, (0.0, 0.0, 1000.0, 1000.0))
        self.assertEqual(ext, (0.0, 0.0, 1000.0, 1000.0))
        # Pasted block: x 20..40, y 30..50 px (µm × 0.1).
        self.assertEqual(int(comp[40, 30, 0]), 255)      # inside
        self.assertEqual(int(comp[10, 10, 0]), 0)        # outside untouched
        self.assertEqual(int(comp[70, 70, 0]), 0)
        # Input not mutated.
        self.assertEqual(int(plate[40, 30, 0]), 0)

    def test_composite_no_wells_passthrough(self):
        plate = np.zeros((50, 50, 3), dtype=np.uint8)
        comp, ext = composite_with_wells(
            self.store, "96", plate, (0.0, 0.0, 500.0, 500.0))
        self.assertIs(comp, plate)

    def test_wells_meta_round_trip(self):
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0))
        ok = self.store.set_wells("96", {"A1": (100.0, 200.0),
                                         "B2": (300.5, -40.0)})
        self.assertTrue(ok)
        self.assertEqual(self.store.get_wells("96"),
                         {"A1": (100.0, 200.0), "B2": (300.5, -40.0)})
        # Persists across instances.
        s2 = MosaicStore(self.store._path)
        self.assertEqual(s2.get_wells("96")["A1"], (100.0, 200.0))
        # No entry / no mapping → {}.
        self.assertEqual(self.store.get_wells("nope"), {})
        self.assertFalse(self.store.set_wells("nope", {"A1": (0, 0)}))
        self.assertFalse(self.store.set_wells("96", {}))

    def test_fresh_save_drops_stored_wells(self):
        # A re-SCAN rebuilds the entry — the stale mapping must not survive.
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0))
        self.store.set_wells("96", {"A1": (1.0, 2.0)})
        self.store.save("96", self.img, (0.0, 0.0, 400.0, 300.0))
        self.assertEqual(self.store.get_wells("96"), {})

    def test_composite_clips_out_of_extent_well(self):
        plate = np.zeros((100, 100, 3), dtype=np.uint8)
        self.store.save("96", plate, (0.0, 0.0, 1000.0, 1000.0))
        # Well partly outside the plate extent (negative µm) — clipped paste.
        well = np.full((20, 20, 3), 255, dtype=np.uint8)
        self.store.save("96#A1", well, (-100.0, -100.0, 100.0, 100.0))
        comp, _ = composite_with_wells(
            self.store, "96", plate, (0.0, 0.0, 1000.0, 1000.0))
        self.assertEqual(int(comp[5, 5, 0]), 255)        # visible part pasted
        self.assertEqual(int(comp[50, 50, 0]), 0)


# ── ReanchorFeatureStore ──────────────────────────────────────────────

class TestReanchorFeatureStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.store = ReanchorFeatureStore(Path(self._tmp.name) / "rf.json")
        self.patch = np.full((40, 40, 3), 128, dtype=np.uint8)

    def test_save_get_load_round_trip(self):
        ok = self.store.save("96", self.patch, (12345.0, 6789.0),
                             um_per_px=3.1, camobj="cam|4x")
        self.assertTrue(ok)
        self.assertTrue(self.store.has("96"))
        rec = self.store.get("96")
        self.assertEqual(rec["stage_um"], [12345.0, 6789.0])
        self.assertAlmostEqual(rec["um_per_px"], 3.1)
        self.assertEqual(rec["camobj"], "cam|4x")
        p = self.store.load_patch("96")
        self.assertEqual(p.shape, self.patch.shape)
        # Persists across instances.
        s2 = ReanchorFeatureStore(self.store._path)
        self.assertTrue(s2.has("96"))

    def test_set_stage_um_tracks_map_moves(self):
        self.store.save("96", self.patch, (100.0, 200.0), um_per_px=1.0)
        self.assertTrue(self.store.set_stage_um("96", 150.0, 250.0))
        self.assertEqual(self.store.get("96")["stage_um"], [150.0, 250.0])
        self.assertFalse(self.store.set_stage_um("nope", 0.0, 0.0))

    def test_clear_removes_image(self):
        self.store.save("96", self.patch, (0.0, 0.0), um_per_px=1.0)
        self.store.clear("96")
        self.assertFalse(self.store.has("96"))
        self.assertIsNone(self.store.load_patch("96"))


# ── VisionDetector.find_template ─────────────────────────────────────

class TestFindTemplate(unittest.TestCase):
    def _frame_with_feature(self, cx, cy):
        """Noise-free frame with a distinctive blob at (cx, cy)."""
        import cv2
        frame = np.full((200, 300), 40, dtype=np.uint8)
        cv2.circle(frame, (cx, cy), 12, 220, -1)
        cv2.rectangle(frame, (cx - 4, cy - 18), (cx + 4, cy - 10), 180, -1)
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    def test_finds_feature_at_offset(self):
        frame = self._frame_with_feature(210, 140)
        # The template = a crop around the same feature drawn at a different
        # frame position (identical appearance).
        src = self._frame_with_feature(60, 60)
        patch = src[30:90, 30:90]
        res = find_template(frame, patch)
        self.assertIsNotNone(res)
        cx, cy, conf = res
        self.assertGreater(conf, 0.8)
        self.assertAlmostEqual(cx, 210.0, delta=2.0)
        self.assertAlmostEqual(cy, 140.0, delta=2.0)

    def test_rejects_bad_inputs(self):
        frame = self._frame_with_feature(100, 100)
        self.assertIsNone(find_template(None, frame))
        self.assertIsNone(find_template(frame, None))
        big = np.zeros((500, 500, 3), dtype=np.uint8)
        self.assertIsNone(find_template(frame, big))    # patch > frame
        tiny = np.zeros((4, 4, 3), dtype=np.uint8)
        self.assertIsNone(find_template(frame, tiny))   # patch < 8 px


# ── Dialog: main wells only ──────────────────────────────────────────

class _FakeWell:
    def __init__(self, name, x, y, is_subwell=False, parent_well=None,
                 diameter=6.0, row=0, col=0):
        self.name = name
        self.x = x
        self.y = y
        self.is_subwell = is_subwell
        self.parent_well = parent_well
        self.diameter = diameter
        self.row = row
        self.col = col


class _FakeRosettePlate:
    """6-well plate where A1 is a rosette (parent dropped, 3 sub-wells)."""

    format = 6
    rows = 2
    cols = 3
    well_diameter = 34.8

    def __init__(self):
        self._wells = []
        for r in range(2):
            for c in range(3):
                nm = f"{chr(ord('A') + r)}{c + 1}"
                if nm == "A1":
                    for i, sub in enumerate("abc"):
                        self._wells.append(_FakeWell(
                            f"A1.{sub}", c * 39.0 + i, r * 39.0,
                            is_subwell=True, parent_well="A1",
                            diameter=5.0, row=r, col=c))
                else:
                    self._wells.append(_FakeWell(
                        nm, c * 39.0, r * 39.0, row=r, col=c))

    @property
    def well_names(self):
        return [w.name for w in self._wells]

    def get_all_wells(self):
        return list(self._wells)

    def get_well_position(self, name):
        for w in self._wells:
            if w.name == name:
                return (w.x, w.y)
        raise KeyError(name)

    def get_well_info(self, name):
        for w in self._wells:
            if w.name == name:
                return w
        raise KeyError(name)


class TestMainWellNames(unittest.TestCase):
    def test_excludes_subwells_adds_parents(self):
        from gui.dialogs.mosaic_well_mapping_dialog import main_well_names
        names = main_well_names(_FakeRosettePlate())
        self.assertNotIn("A1.a", names)
        self.assertNotIn("A1.b", names)
        self.assertIn("A1", names)            # dropped parent re-added
        self.assertIn("B3", names)
        self.assertEqual(len(names), 6)

    def test_plain_plate_unchanged(self):
        from gui.dialogs.mosaic_well_mapping_dialog import main_well_names
        plate = WellPlate.from_format(6)
        self.assertEqual(set(main_well_names(plate)), set(plate.well_names))

    def test_plate_mapping_queue_has_no_subwells(self):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        import SupportClasses.WellTrainingStore as wts
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        wts._store_singleton = wts.WellTrainingStore(Path(tmp.name))
        self.addCleanup(lambda: setattr(wts, "_store_singleton", None))
        img = np.full((300, 400, 3), 100, dtype=np.uint8)
        dlg = MosaicWellMappingDialog(
            _FakeRosettePlate(), img, (0.0, 0.0, 4000.0, 3000.0), 0.1,
            plate_key="6")
        self.assertNotIn("A1.a", dlg._map_names)
        self.assertIn("A1", dlg._map_names)
        # Corner queue draws from main names only.
        for c in dlg._corners:
            self.assertNotIn(".", c)
        # The rosette parent's nominal position = its sub-well centroid.
        nx, ny = dlg._nominal_position("A1")
        self.assertAlmostEqual(nx, 1.0, places=6)    # (0+1+2)/3
        self.assertAlmostEqual(ny, 0.0, places=6)


class _FakeCustomRosettePlate(_FakeRosettePlate):
    """from_wells-style plate: the uniform ``well_diameter`` attribute is
    0.0 ("varies — per-well"), main wells carry the real plate Ø, and the
    rosette sub-wells a small one — the real geometry a rosette design
    compiles to."""
    well_diameter = 0.0

    def __init__(self):
        super().__init__()
        for w in self._wells:
            if not w.is_subwell:
                w.diameter = 34.8


class TestParentFitDiameter(unittest.TestCase):
    """Operator rule: a rosette PARENT is always fitted with the plate's
    NORMAL well diameter — never rosette/sub-well geometry."""

    def _dlg(self, plate):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        import SupportClasses.WellTrainingStore as wts
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        wts._store_singleton = wts.WellTrainingStore(Path(tmp.name))
        self.addCleanup(lambda: setattr(wts, "_store_singleton", None))
        img = np.full((300, 400, 3), 100, dtype=np.uint8)
        return MosaicWellMappingDialog(
            plate, img, (0.0, 0.0, 4000.0, 3000.0), 0.1, plate_key="6")

    def test_parent_uses_plate_normal_diameter(self):
        dlg = self._dlg(_FakeCustomRosettePlate())
        # Parent "A1" has no WellInfo (dropped at compile) → NORMAL well Ø,
        # not the 5 mm sub-well Ø and not the 3 mm unknown fallback.
        self.assertAlmostEqual(dlg._fit_diameter_mm("A1"), 34.8)
        self.assertAlmostEqual(dlg._well_radius_px("A1"),
                               34.8 * 1000.0 / 2.0 * 0.1)
        # The edge-fit sanity check is now ACTIVE for the parent.
        self.assertAlmostEqual(dlg._nominal_radius_px("A1"),
                               34.8 * 1000.0 / 2.0 * 0.1)

    def test_subwell_keeps_its_own_diameter(self):
        dlg = self._dlg(_FakeCustomRosettePlate())
        self.assertAlmostEqual(dlg._fit_diameter_mm("A1.a"), 5.0)

    def test_plain_plate_unchanged(self):
        dlg = self._dlg(WellPlate.from_format(6))
        self.assertAlmostEqual(dlg._fit_diameter_mm("B2"), 34.8, places=3)


# ── CalibrationPage integration ──────────────────────────────────────

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

    def _swap_mosaic_store(self):
        import SupportClasses.MosaicStore as msmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        msmod._store_singleton = msmod.MosaicStore(Path(tmp.name) / "pm.json")
        self.addCleanup(lambda: setattr(msmod, "_store_singleton", None))
        return msmod._store_singleton

    def _swap_feature_store(self):
        import SupportClasses.ReanchorFeatureStore as rfmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        rfmod._store_singleton = rfmod.ReanchorFeatureStore(
            Path(tmp.name) / "rf.json")
        self.addCleanup(lambda: setattr(rfmod, "_store_singleton", None))
        return rfmod._store_singleton


class _RecordingDialog:
    last_args = None
    last_kwargs = None

    def __init__(self, *args, **kwargs):
        type(self).last_args = args
        type(self).last_kwargs = dict(kwargs)

    def exec(self):
        return False        # cancel — we only inspect the handoff


class TestShiftFreeDialogHandoff(_CalBase):
    def test_full_plate_mapping_gets_unshifted_extent(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("test-plate", img, (100.0, 200.0, 500.0, 500.0),
                   mosaic_scale=0.1, shift_um=(25.0, -10.0))
        with patch("gui.dialogs.mosaic_well_mapping_dialog"
                   ".MosaicWellMappingDialog", _RecordingDialog):
            page._ploc_open_well_mapping()
        ext = _RecordingDialog.last_args[2]
        self.assertAlmostEqual(ext[0], 75.0, places=6)    # 100 − 25
        self.assertAlmostEqual(ext[1], 210.0, places=6)   # 200 − (−10)
        self.assertAlmostEqual(ext[2], 475.0, places=6)
        self.assertAlmostEqual(ext[3], 510.0, places=6)

    def test_zero_shift_extent_unchanged(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("test-plate", img, (100.0, 200.0, 500.0, 500.0),
                   mosaic_scale=0.1)
        with patch("gui.dialogs.mosaic_well_mapping_dialog"
                   ".MosaicWellMappingDialog", _RecordingDialog):
            page._ploc_open_well_mapping()
        ext = _RecordingDialog.last_args[2]
        self.assertEqual(tuple(ext), (100.0, 200.0, 500.0, 500.0))

    def test_single_well_mapping_subtracts_shift(self):
        page = self._make_page()
        page._predicted_positions = dict(
            page._plate.get_all_positions_from_plate_center(50000.0, 40000.0))
        page._calibrated_positions = None
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        with patch("gui.dialogs.mosaic_well_mapping_dialog"
                   ".MosaicWellMappingDialog", _RecordingDialog):
            page._ploc_open_single_well_mapping(
                "B2", img, (100.0, 200.0, 500.0, 500.0), 0.1, False,
                shift_um=(25.0, -10.0))
        ext = _RecordingDialog.last_args[2]
        self.assertAlmostEqual(ext[0], 75.0, places=6)
        self.assertAlmostEqual(ext[1], 210.0, places=6)

    def test_single_well_finish_persists_shift(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        page._ploc_open_single_well_mapping = lambda *a, **k: None
        page._ploc_load_persisted_mosaic = lambda: None
        page._ploc_mosaic_running = True
        page._ploc_scan_subwell_parent = "B2"
        page._ploc_scan_well_is_rosette = False
        page._ploc_mosaic_world_shift = lambda: (7.0, 8.0)
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        page._ploc_on_mosaic_finished(
            img, (100.0, 200.0, 500.0, 500.0), 0.1, 4, [])
        self.assertEqual(store.get_shift_um("test-plate#B2"), (7.0, 8.0))


class TestFourButtonGroup(_CalBase):
    def test_buttons_and_menu(self):
        page = self._make_page()
        self.assertEqual(page._ploc_btn_mosaic_cal.text(),
                         "Calibrate mosaic scan")
        self.assertEqual(page._ploc_btn_map_wells.text(), "Map wells…")
        self.assertEqual(page._ploc_btn_reanchor.text(), "Re-anchor mosaic")
        self.assertEqual(page._ploc_btn_auto_reanchor.text(),
                         "Auto re-anchor mosaic")
        adv_buttons = [b for a, b in page._ploc_adv_actions]
        # v7.5.x: "Map wells…" is promoted onto the main button list — not
        # hidden, no longer mirrored in the Advanced menu.
        self.assertFalse(page._ploc_btn_map_wells.isHidden())
        self.assertNotIn(page._ploc_btn_map_wells, adv_buttons)
        # Advanced tools stay hidden but alive; menu mirrors their state.
        self.assertTrue(page._ploc_btn_load_mosaic.isHidden())
        self.assertIn(page._ploc_btn_load_mosaic, adv_buttons)
        self.assertFalse(page._ploc_align_group.isVisible())
        page._ploc_well_anchors = {"A1": {}}
        page._ploc_refresh_scanned_reregister_button()
        page._ploc_sync_advanced_menu()
        texts = [a.text() for a, b in page._ploc_adv_actions]
        self.assertTrue(any("(1)" in t for t in texts), texts)

    def test_main_buttons_vertical_order(self):
        from PySide6.QtWidgets import QVBoxLayout
        page = self._make_page()
        lay = page._ploc_mosaic_box.layout()
        self.assertIsInstance(lay, QVBoxLayout)
        idx = [lay.indexOf(b) for b in (
            page._ploc_btn_mosaic_cal, page._ploc_btn_mosaic,
            page._ploc_btn_map_wells, page._ploc_btn_reanchor,
            page._ploc_btn_auto_reanchor)]
        self.assertTrue(all(i >= 0 for i in idx), idx)
        self.assertEqual(idx, sorted(idx))      # top-to-bottom order kept

    def test_full_scan_auto_opens_mapping(self):
        page = self._make_page()
        self._swap_mosaic_store()
        opened = []
        page._ploc_open_well_mapping = lambda: opened.append(True)
        page._ploc_update_mosaic_preview = lambda *a, **k: None
        page._ploc_apply_mosaic = lambda *a, **k: None
        page._ploc_fit_from_mosaic_detections = lambda *a, **k: 0
        page._ploc_mosaic_running = True
        page._ploc_scan_subwell_parent = None
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        page._ploc_on_mosaic_finished(
            img, (0.0, 0.0, 400.0, 300.0), 0.1, 4, [])
        self.assertEqual(opened, [True])


class TestLoadMosaicFromOtherPlate(_CalBase):
    def test_copy_mosaic_key_copies_plate_and_wells(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("src", img, (0.0, 0.0, 400.0, 300.0),
                   um_per_px=3.3, mosaic_scale=0.1, frames=7,
                   shift_um=(5.0, -2.0))
        store.save("src#B2", img, (10.0, 10.0, 110.0, 110.0),
                   mosaic_scale=0.4)
        n = page._ploc_copy_mosaic_key(store, "src", "test-plate")
        self.assertEqual(n, 2)
        self.assertTrue(store.has("test-plate"))
        self.assertEqual(store.get_extent_um("test-plate"),
                         (0.0, 0.0, 400.0, 300.0))
        self.assertEqual(store.get_shift_um("test-plate"), (5.0, -2.0))
        meta = store.get_meta("test-plate")
        self.assertAlmostEqual(meta["mosaic_scale"], 0.1)
        self.assertAlmostEqual(meta["um_per_px"], 3.3)
        self.assertTrue(store.has_well("test-plate", "B2"))
        # Source untouched.
        self.assertTrue(store.has("src"))
        self.assertTrue(store.has_well("src", "B2"))

    def test_copy_missing_source_fails(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        self.assertEqual(
            page._ploc_copy_mosaic_key(store, "nope", "test-plate"), -1)
        self.assertFalse(store.has("test-plate"))

    def test_list_plate_keys_excludes_well_entries(self):
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("src", img, (0.0, 0.0, 400.0, 300.0))
        store.save("src#B2", img, (10.0, 10.0, 110.0, 110.0))
        self.assertEqual(store.list_plate_keys(), ["src"])

    def test_copy_carries_stored_well_mapping(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("src", img, (0.0, 0.0, 400.0, 300.0))
        store.set_wells("src", {"A1": (10.0, 20.0)})
        page._ploc_copy_mosaic_key(store, "src", "test-plate")
        self.assertEqual(store.get_wells("test-plate"),
                         {"A1": (10.0, 20.0)})

    def test_handler_flow_with_dialogs_applies_mapping(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("src", img, (0.0, 0.0, 400.0, 300.0), mosaic_scale=0.1)
        # Stored mapping: two wells valid on the 6-well plate + one bogus.
        store.set_wells("src", {"A1": (11000.0, 12000.0),
                                "B2": (31000.0, 42000.0),
                                "Z9": (1.0, 1.0)})
        page._ploc_load_persisted_mosaic = MagicMock()
        page._ploc_show_mosaic_mode = MagicMock()
        with patch("PySide6.QtWidgets.QInputDialog.getItem",
                   side_effect=lambda *a, **k: (a[3][0], True)):
            page._ploc_load_mosaic_from_other_plate()
        self.assertTrue(store.has("test-plate"))
        page._ploc_load_persisted_mosaic.assert_called_once()
        # The last good mapping loaded with the mosaic (bogus name dropped).
        self.assertEqual(page._calibrated_positions,
                         {"A1": (11000.0, 12000.0),
                          "B2": (31000.0, 42000.0)})
        self.assertEqual(page._taught_a1, (11000.0, 12000.0))
        self.assertIsNone(page._plate_warp)

    def test_apply_stored_wells_no_layout_match(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("test-plate", img, (0.0, 0.0, 400.0, 300.0))
        store.set_wells("test-plate", {"Z9": (1.0, 1.0)})
        page._calibrated_positions = {"A1": (5.0, 5.0)}
        n = page._ploc_apply_stored_mosaic_wells(store, "test-plate")
        self.assertEqual(n, 0)
        # Nothing was clobbered.
        self.assertEqual(page._calibrated_positions, {"A1": (5.0, 5.0)})

    def test_shift_mosaic_moves_stored_mapping(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("test-plate", img, (0.0, 0.0, 400.0, 300.0),
                   shift_um=(3.0, 4.0))
        store.set_wells("test-plate", {"A1": (100.0, 200.0)})
        page._ploc_overlay_img = img
        page._ploc_overlay_ext = (0.0, 0.0, 400.0, 300.0)
        page._ploc_set_overlay_image = MagicMock()
        self.assertTrue(page._ploc_shift_mosaic_by(10.0, -5.0))
        self.assertEqual(store.get_wells("test-plate"),
                         {"A1": (110.0, 195.0)})
        # Registration shift + extent both preserved/translated as before.
        self.assertEqual(store.get_shift_um("test-plate"), (3.0, 4.0))
        self.assertEqual(store.get_extent_um("test-plate"),
                         (10.0, -5.0, 410.0, 295.0))

    def test_handler_no_sources_informs(self):
        page = self._make_page()
        self._swap_mosaic_store()
        with patch("gui.pages.calibration.QMessageBox") as mb:
            page._ploc_load_mosaic_from_other_plate()
        mb.information.assert_called()


class _AcceptingDialog:
    """Mapping dialog stand-in that accepts with a fixed result set."""

    results_value = {"A1": (11000.0, 12000.0), "B2": (31000.0, 42000.0)}

    def __init__(self, *args, **kwargs):
        pass

    def exec(self):
        return True

    def results(self):
        return dict(type(self).results_value)

    def saved_sample_path(self):
        return None


class TestMappingStoredWithMosaic(_CalBase):
    def test_map_wells_confirm_stores_mapping_in_mosaic(self):
        page = self._make_page()
        store = self._swap_mosaic_store()
        img = np.full((30, 40, 3), 90, dtype=np.uint8)
        store.save("test-plate", img, (0.0, 0.0, 400.0, 300.0),
                   mosaic_scale=0.1)
        page._ploc_feed_affine = MagicMock()
        page._ploc_save_plate_template = lambda *a, **k: False
        page._ploc_refresh_reregister_button = lambda: None
        with patch("gui.dialogs.mosaic_well_mapping_dialog"
                   ".MosaicWellMappingDialog", _AcceptingDialog):
            page._ploc_open_well_mapping()
        self.assertEqual(store.get_wells("test-plate"),
                         _AcceptingDialog.results_value)


class TestAutoReanchor(_CalBase):
    def test_feature_position_tracks_translation(self):
        page = self._make_page()
        fstore = self._swap_feature_store()
        fstore.save("test-plate", np.full((40, 40, 3), 90, dtype=np.uint8),
                    (1000.0, 2000.0), um_per_px=1.0)
        grid = page._plate.get_all_positions_from_plate_center(
            50000.0, 40000.0)
        page._predicted_positions = dict(grid)
        page._calibrated_positions = dict(grid)
        page._reference_markers = {}
        page._xy_teach_points = {}
        page._refresh_ploc_view = lambda: None
        page._emit_calibration_data_changed = lambda: None
        page._save_calibration = lambda **k: None
        page._ploc_shift_mosaic_by = lambda ex, ey: False
        page._ploc_apply_global_translation(50.0, -25.0)
        self.assertEqual(fstore.get("test-plate")["stage_um"],
                         [1050.0, 1975.0])

    def test_worker_match_math(self):
        from gui.pages.calibration import _AutoReanchorWorker
        import cv2
        # Frame with a feature at (210, 140); patch of the same feature.
        frame = np.full((200, 300), 40, dtype=np.uint8)
        cv2.circle(frame, (210, 140), 12, 220, -1)
        cv2.rectangle(frame, (206, 122), (214, 130), 180, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        src = np.full((200, 300), 40, dtype=np.uint8)
        cv2.circle(src, (60, 60), 12, 220, -1)
        cv2.rectangle(src, (56, 42), (64, 50), 180, -1)
        src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
        patch = src[30:90, 30:90]

        ctrl = MagicMock()
        ctrl.get_xy_position.return_value = (10000.0, 20000.0)
        mgr = MagicMock()
        mgr.effective_um_per_px.return_value = 2.0
        # identity-style pixel→stage: offset from centre × 2 µm/px
        mgr.pixel_to_stage_offset.side_effect = (
            lambda idx, x, y, w, h: ((x - w / 2) * 2.0, (y - h / 2) * 2.0))
        worker = _AutoReanchorWorker(
            ctrl, cam=None, cam_mgr=mgr, cam_idx=0,
            target_um=(0.0, 0.0), safe_z=None, patch=patch,
            stored_um_per_px=2.0)
        res = worker._match(frame)
        self.assertIsNotNone(res)
        ax, ay, conf = res
        self.assertGreater(conf, 0.8)
        # actual = stage + (match − centre) × 2 = 10000 + (210−150)*2 …
        self.assertAlmostEqual(ax, 10000.0 + (210 - 150) * 2.0, delta=5.0)
        self.assertAlmostEqual(ay, 20000.0 + (140 - 100) * 2.0, delta=5.0)

    def test_done_applies_translation(self):
        page = self._make_page()
        fstore = self._swap_feature_store()
        fstore.save("test-plate", np.full((40, 40, 3), 90, dtype=np.uint8),
                    (1000.0, 2000.0), um_per_px=1.0)
        applied = {}
        page._ploc_apply_global_translation = (
            lambda ex, ey, pin=None: applied.setdefault("E", (ex, ey)) or 6)
        page._ploc_on_auto_reanchor_done(1150.0, 1950.0, 0.9)
        self.assertAlmostEqual(applied["E"][0], 150.0, places=6)
        self.assertAlmostEqual(applied["E"][1], -50.0, places=6)

    def test_button_gate_requires_feature(self):
        page = self._make_page()
        self._swap_feature_store()               # empty store
        page._ploc_refresh_auto_reanchor_button()
        self.assertFalse(page._ploc_btn_auto_reanchor.isEnabled())


class TestRosetteTab(_CalBase):
    def test_tab_order_and_indices(self):
        page = self._make_page()
        tabs = page._workflow_tabs
        names = [tabs.tabText(i) for i in range(tabs.count())]
        self.assertEqual(names[3], "Rosettes")
        self.assertEqual(page._rosette_tab_index, 3)
        self.assertEqual(names[page._zauto_tab_index], "Plate Z Auto-Cal")

    def test_combo_empty_without_rosettes(self):
        page = self._make_page()
        page._rosette_refresh_wells()
        self.assertEqual(page._rosette_well_combo.count(), 0)
        page._rosette_refresh_status()
        self.assertFalse(page._rosette_btn_scan.isEnabled())

    def test_scan_selected_delegates(self):
        page = self._make_page()
        calls = []
        page._ploc_scan_single_well = lambda n, r: calls.append((n, r))
        page._rosette_well_combo.addItem("B2 (rosette)", "B2")
        page._rosette_scan_selected()
        self.assertEqual(calls, [("B2", True)])


class TestOverlayCirclesOnly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_overlay_mode_renders(self):
        from PySide6.QtGui import QImage
        from gui.widgets.jog_workspace_view import JogWorkspaceView

        class _SL:
            xy_min_x, xy_min_y, xy_max_x, xy_max_y = 0.0, 0.0, 100000.0, 80000.0
            def check_xy_near_limit(self, *a, **k):
                return {}
        view = JogWorkspaceView()
        view.set_safety_limits(_SL())
        view.resize(400, 300)
        view.set_well_positions(
            {"A1": (10000.0, 10000.0), "A1.a": (11000.0, 10500.0)},
            "calibrated")
        view.set_plate_display_mode("overlay")
        self.assertEqual(view._plate_display_mode, "overlay")
        target = QImage(400, 300, QImage.Format.Format_ARGB32)
        view.render(target)          # outline-only path: must not crash
        view.set_plate_display_mode("well")
        view.render(target)          # legacy path intact


if __name__ == "__main__":
    unittest.main()
