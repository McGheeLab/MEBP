"""Tests for the v7.5.x Fluorescence Mosaic workflow + store + overlay.

Covers:
  - FluorescenceMosaicStore: save/list/extent/colour, per-well + per-plate blend,
    persistence round-trip, delete.
  - The shared _fluorescence_overlay helper drives a real WorkspaceTargetView.
  - JogWorkspaceView gains an independent fluorescence overlay layer.
  - The workflow tile is registered + routes to the page; the page builds
    offscreen and selects channels.

GUI tests are skipped if PySide6 / an offscreen platform isn't available.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from SupportClasses.FluorescenceMosaicStore import (
    FluorescenceMosaicStore, CHANNELS, default_color, well_key,
)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtWidgets import QApplication
    _QT = True
except Exception:   # pragma: no cover
    _QT = False

_app = None


def _ensure_app():
    global _app
    if _app is None:
        _app = QApplication.instance() or QApplication([])
    return _app


def _img(h=40, w=60):
    rng = np.random.RandomState(0)
    return (rng.rand(h, w, 3) * 255).astype("uint8")


class TestFluorescenceMosaicStore(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.store = FluorescenceMosaicStore(Path(self.dir) / "f.json")
        self.ext = (1000.0, 2000.0, 1600.0, 2400.0)

    def test_channels_and_default_colors(self):
        self.assertEqual(CHANNELS, ("DAPI", "FITC", "mCherry", "Cy5"))
        for ch in CHANNELS:
            c = default_color(ch)
            self.assertEqual(len(c), 3)

    def test_well_key(self):
        self.assertEqual(well_key("24", "A1"), "24|A1")

    def test_save_and_read(self):
        ok = self.store.save_channel(
            "24", "A1", "DAPI", _img(), self.ext, color_rgb=(0, 0, 255),
            objective="10x", um_per_px=0.9, mosaic_scale=0.3, frames=12)
        self.assertTrue(ok)
        self.assertEqual(self.store.list_channels("24", "A1"), ["DAPI"])
        self.assertEqual(self.store.get_extent_um("24", "A1"), self.ext)
        self.assertEqual(self.store.channel_color("24", "A1", "DAPI"), (0, 0, 255))
        self.assertTrue(self.store.has("24", "A1"))
        self.assertEqual(self.store.list_wells("24"), ["A1"])

    def test_default_color_when_unspecified(self):
        self.store.save_channel("24", "A1", "FITC", _img(), self.ext)
        self.assertEqual(
            self.store.channel_color("24", "A1", "FITC"), default_color("FITC"))

    def test_set_channel_color(self):
        self.store.save_channel("24", "A1", "Cy5", _img(), self.ext)
        self.assertTrue(self.store.set_channel_color("24", "A1", "Cy5", (1, 2, 3)))
        self.assertEqual(self.store.channel_color("24", "A1", "Cy5"), (1, 2, 3))
        self.assertFalse(self.store.set_channel_color("24", "Z9", "Cy5", (1, 2, 3)))

    def test_composite_overlay_blends(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext, color_rgb=(0, 0, 255))
        self.store.save_channel("24", "A1", "FITC", _img(), self.ext, color_rgb=(0, 255, 0))
        img, ext = self.store.composite_overlay("24", "A1")
        self.assertIsNotNone(img)
        self.assertEqual(img.shape[:2], (40, 60))
        self.assertEqual(ext, self.ext)

    def test_composite_overlay_subset(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext)
        self.store.save_channel("24", "A1", "FITC", _img(), self.ext)
        img, _ = self.store.composite_overlay("24", "A1", channels=["DAPI"])
        self.assertIsNotNone(img)

    def test_composite_overlay_empty(self):
        img, ext = self.store.composite_overlay("24", "ZZ")
        self.assertIsNone(img)
        self.assertIsNone(ext)

    def test_composite_plate_overlay(self):
        self.store.save_channel("24", "A1", "DAPI", _img(),
                                (0.0, 0.0, 600.0, 400.0))
        self.store.save_channel("24", "B2", "FITC", _img(),
                                (1000.0, 1000.0, 1600.0, 1400.0))
        img, ext = self.store.composite_plate_overlay("24")
        self.assertIsNotNone(img)
        # Union extent spans both wells.
        self.assertEqual(ext, (0.0, 0.0, 1600.0, 1400.0))

    def test_persistence_round_trip(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext)
        self.store.save_channel("24", "A1", "FITC", _img(), self.ext)
        store2 = FluorescenceMosaicStore(Path(self.dir) / "f.json")
        self.assertEqual(set(store2.list_channels("24", "A1")), {"DAPI", "FITC"})
        self.assertIsNotNone(store2.load_channel_image("24", "A1", "DAPI"))

    def test_clear(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext)
        self.store.save_channel("24", "A1", "FITC", _img(), self.ext)
        self.store.clear_channel("24", "A1", "DAPI")
        self.assertEqual(self.store.list_channels("24", "A1"), ["FITC"])
        self.store.clear_well("24", "A1")
        self.assertFalse(self.store.has("24", "A1"))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestOverlayHelper(unittest.TestCase):
    def setUp(self):
        _ensure_app()
        import SupportClasses.FluorescenceMosaicStore as fms
        self.fms = fms
        self.dir = tempfile.mkdtemp()
        fms._store_singleton = FluorescenceMosaicStore(Path(self.dir) / "f.json")
        fms._store_singleton.save_channel(
            "24", "A1", "DAPI", _img(50, 80), (1000.0, 2000.0, 1800.0, 2500.0),
            color_rgb=(0, 0, 255))

    def tearDown(self):
        import SupportClasses.FluorescenceMosaicStore as fms
        fms._store_singleton = None

    def _view(self):
        class SL:
            xy_min_x = 0.0; xy_min_y = 0.0
            xy_max_x = 120000.0; xy_max_y = 80000.0
        from gui.widgets.workspace_target_view import WorkspaceTargetView
        v = WorkspaceTargetView()
        v.set_safety_limits(SL())
        return v

    def test_has_any_fluor(self):
        from gui.pages.workflows._fluorescence_overlay import has_any_fluor
        self.assertTrue(has_any_fluor("24"))
        self.assertFalse(has_any_fluor("96"))

    def test_plate_key_of(self):
        from gui.pages.workflows._fluorescence_overlay import plate_key_of

        class HW:
            active_plate_key = "24"
        self.assertEqual(plate_key_of(HW()), "24")
        self.assertIsNone(plate_key_of(None))

    def test_load_plate_overlay(self):
        from gui.pages.workflows._fluorescence_overlay import load_plate_fluor_overlay
        v = self._view()
        ok = load_plate_fluor_overlay(v, "24", visible=True)
        self.assertTrue(ok)
        self.assertTrue(v.has_fluor())
        v.resize(300, 200)
        v.repaint()

    def test_load_plate_overlay_none(self):
        from gui.pages.workflows._fluorescence_overlay import load_plate_fluor_overlay
        v = self._view()
        self.assertFalse(load_plate_fluor_overlay(v, "96", visible=True))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestJogWorkspaceFluorLayer(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_fluor_layer_api(self):
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        v = JogWorkspaceView()
        self.assertFalse(v.has_fluor())
        v.set_fluor_overlay(None, None)
        self.assertFalse(v.has_fluor())
        v.set_fluor_visible(True)   # no overlay → still no-op safe
        v.set_fluor_opacity(0.5)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestWorkflowRegistrationAndPage(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_tile_registered(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        ids = [t.workflow_id for t in WORKFLOWS]
        self.assertIn("fluorescence_mosaic", ids)
        tile = next(t for t in WORKFLOWS if t.workflow_id == "fluorescence_mosaic")
        self.assertTrue(tile.enabled)

    def test_page_builds_and_selects_channels(self):
        os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())

        class SL:
            xy_min_x = 0.0; xy_min_y = 0.0
            xy_max_x = 120000.0; xy_max_y = 80000.0

        class Ctrl:
            safety_limits = SL()
            is_zp_connected = False
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        class HW:
            active_plate_key = "24"

        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        pg = FluorescenceMosaicWorkflowPage(
            controller=Ctrl(), settings=object(), camera_manager=None)
        pg.set_hardware_config(HW())
        self.assertEqual(
            pg._selected_channels(), ["DAPI", "FITC", "mCherry", "Cy5"])
        # Unchecking a channel removes it from the capture set.
        pg._channel_checks["FITC"].setChecked(False)
        self.assertNotIn("FITC", pg._selected_channels())


class TestMosaicBuilderCanvasInit(unittest.TestCase):
    """Bug 2 regression: the worker's builder must be canvas-initialized
    (via generate_raster_positions) or it stitches nothing → empty viewer."""

    def _builder(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        return MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=2.0,
                             overlap=0.25, target_mosaic_px=400, register=False)

    def test_uninitialized_builder_stitches_nothing(self):
        b = self._builder()
        b.add_raster_frame(_img(60, 80), 200.0, 150.0, index=0)
        # The OLD (buggy) path: no generate_raster_positions on this builder.
        self.assertIsNone(b.stitch_incremental())
        self.assertIsNone(b.composite)
        self.assertIsNone(b.canvas_extent_um)

    def test_initialized_builder_produces_composite(self):
        b = self._builder()
        # The FIX: allocate the canvas on the SAME builder before stitching.
        b.generate_raster_positions((0.0, 0.0, 400.0, 300.0), overlap=0.25)
        b.add_raster_frame(_img(60, 80), 200.0, 150.0, index=0)
        b.stitch_incremental()
        self.assertIsNotNone(b.composite)
        self.assertIsNotNone(b.canvas_extent_um)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestRasterPlanAndGridPreview(unittest.TestCase):
    """Bug 1 (FOV/coverage) + the grid-preview feature."""

    def setUp(self):
        _ensure_app()

    # ── Fakes ──
    class _FakeCam:
        def get_current_frame(self):
            return _img(686, 916)

        def capture_fresh_frame(self, **_k):
            return _img(686, 916)

    class _FakeCM:
        def __init__(self, eff):
            self._eff = eff
            self.cameras = [TestRasterPlanAndGridPreview._FakeCam()]

        def effective_um_per_px(self, _idx, _w):
            return self._eff

        def get_um_per_px(self, _idx):
            return self._eff

        def is_um_per_px_calibrated(self, _idx):
            return True

        def set_um_per_px(self, *a, **k):
            pass

        def set_rotation_deg(self, *a, **k):
            pass

        def is_running(self, _idx):
            return True

        def start(self, _idx):
            pass

        def stop(self, _idx):
            pass

    class _FakeWell:
        def __init__(self, name, row, col, diameter):
            self.name = name
            self.row = row
            self.col = col
            self.diameter = diameter

    class _FakePlate:
        well_diameter = 6.35  # mm (96-well)
        rows = 1
        cols = 1
        well_names = ["A1"]

        def get_all_wells(self):
            return [TestRasterPlanAndGridPreview._FakeWell("A1", 0, 0, 6.35)]

        def get_well_position(self, _name):
            return (0.0, 0.0)

    def _page(self, eff=0.385):
        os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())

        class SL:
            xy_min_x = 0.0; xy_min_y = 0.0
            xy_max_x = 120000.0; xy_max_y = 80000.0

        class Ctrl:
            safety_limits = SL()
            is_zp_connected = False
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        pg = FluorescenceMosaicWorkflowPage(
            controller=Ctrl(), settings=object(), camera_manager=None)
        # Inject a fake camera_manager AFTER construction (no real CameraFeedView).
        pg._camera_manager = self._FakeCM(eff)
        pg._plate = self._FakePlate()
        pg._well_positions = {"A1": (50000.0, 40000.0)}
        pg._scan_well = "A1"
        return pg

    def test_compute_raster_plan_covers_well_many_tiles(self):
        pg = self._page(eff=0.385)   # FOV ≈ 916*0.385 ≈ 353 µm
        plan = pg._compute_raster_plan("A1")
        self.assertIsNotNone(plan)
        # A 6.35 mm well at ~0.35 mm FOV must be many tiles in BOTH axes.
        self.assertGreater(plan["cols"], 5)
        self.assertGreater(plan["rows"], 5)
        self.assertGreater(len(plan["grid"]), 25)
        # FOV must be the live-frame FOV, not a multi-mm inflated value.
        self.assertLess(plan["fov_um"][0], 1000.0)

    def test_microscope_um_per_px_rescales_by_live_width(self):
        import SupportClasses.ObjectiveCalibration as oc

        class _FakeObjStore:
            def get_calibration(self, _cam, _obj):
                return {"measured_um_per_px": 1.54, "resolution": [916, 686]}

        orig = oc.get_store
        oc.get_store = lambda *a, **k: _FakeObjStore()
        try:
            pg = self._page()

            class _Cfg:
                class camera_config:
                    current_objective_name = "2x"
                    camera_spec = type("S", (), {"name": "BUC3D"})()
            pg._hw_config = _Cfg()
            # 916 px → base measured value; 3664 px → rescaled by 916/3664.
            self.assertAlmostEqual(pg._microscope_um_per_px(916, 99.0), 1.54, places=3)
            self.assertAlmostEqual(
                pg._microscope_um_per_px(3664, 99.0), 1.54 * 916 / 3664, places=4)
        finally:
            oc.get_store = orig

    def test_grid_preview_populates_viewer_and_navigator(self):
        pg = self._page(eff=0.385)
        pg._refresh_grid_preview()
        self.assertIsNotNone(pg._mosaic_view._grid_group)
        self.assertGreater(pg._navigator._raster_cols, 1)
        self.assertGreater(pg._navigator._raster_rows, 1)

    def test_inherits_shared_mosaic_frame_orient(self):
        """The single-well scan must follow the EXACT same pattern as the
        full-plate mosaic: it reads frame_orient / overlap / fov_um from the
        shared ``mosaic_scan`` settings section (not its own defaults). This is
        the misalignment fix — ME3B V1's camera is mounted rot180."""
        class _Settings:
            def get_section(self, name):
                if name == "mosaic_scan":
                    return {"frame_orient": "rot180", "overlap_pct": 5,
                            "fov_um": 2822}
                return None

        pg = self._page(eff=0.385)
        pg._settings = _Settings()
        scan = pg._scan_settings()
        self.assertEqual(scan["frame_orient"], "rot180")
        self.assertEqual(scan["overlap_pct"], 5)
        self.assertEqual(scan["fov_um"], 2822)

    def test_fov_override_is_fallback_when_no_objective_cal(self):
        """When the selected objective has NO stored calibration, the shared
        ``fov_um`` override sizes the tiles (a graceful fallback). Here the page
        has no matching objective calibration, so fov_um wins."""
        class _Settings:
            def get_section(self, name):
                return {"fov_um": 916.0} if name == "mosaic_scan" else None

        pg = self._page(eff=0.385)
        pg._settings = _Settings()
        plan = pg._compute_raster_plan("A1")
        self.assertIsNotNone(plan)
        # fov_um=916 over a 916 px frame → eff = 1.0 µm/px → FOV = 916 µm.
        self.assertAlmostEqual(plan["eff_um_per_px"], 1.0, places=3)
        self.assertAlmostEqual(plan["fov_um"][0], 916.0, places=1)

    def test_objective_um_per_px_overrides_shared_fov(self):
        """REGRESSION: the fluorescence workflow is objective-SELECTABLE, so the
        per-objective calibrated µm/px must WIN over the shared full-plate
        ``fov_um``. Otherwise a 4x scan inherits the 2x FOV and the raster is
        spaced too far apart (gaps between tiles). Here a 4x objective is
        calibrated at 1.547 µm/px @ 916 px while a stale 2822 µm (2x) FOV
        override is present — the plan must use the 4x value, not the override."""
        import SupportClasses.ObjectiveCalibration as oc

        class _FakeObjStore:
            def get_calibration(self, cam, obj):
                if obj == "4x":
                    return {"measured_um_per_px": 1.547113,
                            "resolution": [916, 686]}
                return None

        class _Settings:
            def get_section(self, name):
                # Stale 2x-sized FOV override (≈2854 µm) — must be IGNORED
                # because the selected objective (4x) has a real calibration.
                return {"fov_um": 2822} if name == "mosaic_scan" else None

        class _Cfg:
            class camera_config:
                current_objective_name = "4x"
                camera_spec = type("S", (), {"name": "BUC3D"})()

        orig = oc.get_store
        oc.get_store = lambda *a, **k: _FakeObjStore()
        try:
            pg = self._page(eff=0.385)
            pg._settings = _Settings()
            pg._hw_config = _Cfg()
            plan = pg._compute_raster_plan("A1")
            self.assertIsNotNone(plan)
            # 4x @ 916 px → 1.547 µm/px → FOV ≈ 1417 µm (NOT the 2822 override).
            self.assertAlmostEqual(plan["eff_um_per_px"], 1.547113, places=4)
            self.assertLess(plan["fov_um"][0], 1600.0)
            # And the objective helper agrees, rescaling to the live width.
            self.assertAlmostEqual(pg._objective_um_per_px(916), 1.547113, places=4)
            self.assertAlmostEqual(
                pg._objective_um_per_px(1832), 1.547113 * 916 / 1832, places=5)
        finally:
            oc.get_store = orig

    def test_learned_calibration_overrides_objective_and_rescales(self):
        """A mosaic FOV/spacing calibration (learned, per camera+objective) wins
        over the objective µm/px AND is resolution-safe: a value measured at
        916 px rescales for a wider live frame."""
        import SupportClasses.ObjectiveCalibration as oc
        import SupportClasses.MosaicAlignmentStore as mas

        class _ObjStore:
            def get_calibration(self, cam, obj):
                return {"measured_um_per_px": 1.54, "resolution": [916, 686]}

        class _AlignStore:      # learned spacing correction @ 916 px
            def get_um_per_px(self, key):
                return 1.20
            def get_resolution(self, key):
                return (916, 686)

        o_orig, a_orig = oc.get_store, mas.get_store
        oc.get_store = lambda *a, **k: _ObjStore()
        mas.get_store = lambda *a, **k: _AlignStore()
        try:
            pg = self._page(eff=0.385)

            class _Cfg:
                class camera_config:
                    current_objective_name = "4x"
                    camera_spec = type("S", (), {"name": "BUC3D"})()
            pg._hw_config = _Cfg()
            # Live frame is 916 px (the fake cam) → learned used directly, and it
            # beats the objective's 1.54.
            self.assertAlmostEqual(pg._learned_um_per_px(916), 1.20, places=3)
            plan = pg._compute_raster_plan("A1")
            self.assertAlmostEqual(plan["eff_um_per_px"], 1.20, places=3)
            # At a wider live frame the learned value rescales (µm/px ∝ 1/width).
            self.assertAlmostEqual(
                pg._learned_um_per_px(1832), 1.20 * 916 / 1832, places=4)
        finally:
            oc.get_store, mas.get_store = o_orig, a_orig

    def test_legacy_learned_without_resolution_is_ignored(self):
        """REGRESSION: a learned value with NO recorded resolution (a stale
        pre-resolution-stamp entry, e.g. the 2x value captured at 916 px) must be
        ignored so it can't reintroduce the wrong-spacing bug at a different live
        resolution — the resolution-safe objective µm/px is used instead."""
        import SupportClasses.ObjectiveCalibration as oc
        import SupportClasses.MosaicAlignmentStore as mas

        class _ObjStore:
            def get_calibration(self, cam, obj):
                return {"measured_um_per_px": 0.778843, "resolution": [3664, 2748]}

        class _AlignStore:      # stale 2x value, NO resolution stamp
            def get_um_per_px(self, key):
                return 3.094
            def get_resolution(self, key):
                return None

        o_orig, a_orig = oc.get_store, mas.get_store
        oc.get_store = lambda *a, **k: _ObjStore()
        mas.get_store = lambda *a, **k: _AlignStore()
        try:
            pg = self._page(eff=0.385)

            class _Cfg:
                class camera_config:
                    current_objective_name = "2x"
                    camera_spec = type("S", (), {"name": "BUC3D"})()
            pg._hw_config = _Cfg()
            # Un-stamped learned → ignored at any width.
            self.assertIsNone(pg._learned_um_per_px(916))
            self.assertIsNone(pg._learned_um_per_px(3664))
            # Plan falls to the resolution-safe objective value (rescaled to the
            # 916 px live frame), NOT the stale learned 3.094 or the fov_um.
            plan = pg._compute_raster_plan("A1")
            self.assertAlmostEqual(
                plan["eff_um_per_px"], 0.778843 * 3664 / 916, places=3)
        finally:
            oc.get_store, mas.get_store = o_orig, a_orig

    def test_settings_dialog_has_calibrate_button(self):
        """The settings popout exposes the mosaic-calibration entry point."""
        pg = self._page(eff=0.385)
        self.assertTrue(hasattr(pg, "_cal_button"))
        self.assertTrue(hasattr(pg, "_cal_status_lbl"))
        self.assertEqual(pg._cal_button.text(), "Calibrate…")
        # Refreshing the status must not raise (no calibration → objective note).
        pg._refresh_calibration_status()
        self.assertIn("not calibrated", pg._cal_status_lbl.text())

    def test_open_mosaic_calibration_wires_dialog(self):
        """Calibrate… opens the shared MosaicCalibrationDialog wired to this
        workflow's camera / objective key / selected well, and forces fov_um=0
        so the calibration mosaic uses the OBJECTIVE scale, not the shared 2x
        FOV override."""
        import gui.dialogs.mosaic_calibration_dialog as mcd

        captured = {}

        class _FakeDlg:
            def __init__(self, controller, mgr, cam_idx, **kw):
                captured["cam_idx"] = cam_idx
                captured.update(kw)

            def exec(self):
                return 0

        class _Settings:
            def get_section(self, name):
                return {"fov_um": 2822} if name == "mosaic_scan" else None

        orig = mcd.MosaicCalibrationDialog
        mcd.MosaicCalibrationDialog = _FakeDlg
        try:
            pg = self._page(eff=0.385)   # ctrl not ZP-connected; µm/px calibrated
            pg._settings = _Settings()
            pg._scan_well = "A1"
            pg._open_mosaic_calibration()
            self.assertIn("align_key", captured)
            self.assertGreater(captured["um_per_px_camera"], 0.0)
            # The shared 2822 FOV must be overridden to 0 (objective scale).
            self.assertEqual(captured["settings"]["fov_um"], 0)
            self.assertEqual(captured["frame_size"], (916, 686))
            # Centre = the selected well (A1 at (50000, 40000) in _page()).
            self.assertEqual(captured["center_um"], (50000.0, 40000.0))
        finally:
            mcd.MosaicCalibrationDialog = orig


@unittest.skipUnless(_QT, "PySide6 not available")
class TestMosaicAlignmentStoreResolution(unittest.TestCase):
    """The alignment store records + returns the capture resolution (so a
    consumer at a different width can rescale), backward-compatibly."""

    def test_resolution_round_trip_and_legacy(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        st = MosaicAlignmentStore(Path(tempfile.mkdtemp()) / "ma.json")
        st.set_um_per_px("cam|4x", 1.23, source="quick_fov",
                         resolution=(3664, 2748))
        self.assertAlmostEqual(st.get_um_per_px("cam|4x"), 1.23, places=3)
        self.assertEqual(st.get_resolution("cam|4x"), (3664.0, 2748.0))
        # Legacy write (no resolution) → get_resolution is None.
        st.set_um_per_px("cam|2x", 0.7)
        self.assertAlmostEqual(st.get_um_per_px("cam|2x"), 0.7, places=3)
        self.assertIsNone(st.get_resolution("cam|2x"))
        # Survives a reload.
        st2 = MosaicAlignmentStore(st._path)
        self.assertEqual(st2.get_resolution("cam|4x"), (3664.0, 2748.0))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestWorkerProducesComposite(unittest.TestCase):
    """Bug 2 end-to-end: with the canvas-initialized builder, the worker emits a
    NON-None composite + extent (so the viewer shows it and the channel saves)."""

    def setUp(self):
        _ensure_app()

    class _Cam:
        def __init__(self):
            self._n = 0

        def frame_count_value(self):
            self._n += 1
            return self._n

        def get_current_frame(self):
            return _img(60, 80)

    class _Ctrl:
        zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        def suspend_position_poller(self): pass
        def resume_position_poller(self): pass
        def safe_travel_to(self, *a, **k): pass
        def move_xy_absolute_um(self, *a, **k): pass
        def wait_for_xy_arrival(self, *a, **k): pass

        def get_xy_position(self, cached=False):
            return (200.0, 150.0)

    def test_worker_emits_composite(self):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            _SingleWellMosaicWorker)
        from SupportClasses.MosaicBuilder import MosaicBuilder
        bounds = (0.0, 0.0, 300.0, 200.0)
        builder = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=2.0,
                                overlap=0.25, target_mosaic_px=400, register=False)
        positions = builder.generate_raster_positions(bounds, overlap=0.25)
        self.assertTrue(positions)
        worker = _SingleWellMosaicWorker(
            self._Ctrl(), self._Cam(), builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.5, settle_ms=0)
        got = {}

        def on_done(comp, ext, scale, frames):
            got["comp"] = comp
            got["ext"] = ext

        worker.finished_ok.connect(on_done)
        worker.run()   # synchronous (same thread) — direct-connected slot fires
        self.assertIsNotNone(got.get("comp"))
        self.assertIsNotNone(got.get("ext"))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestNavigatorRasterGrid(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_set_raster_grid_paints(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        from PySide6.QtGui import QPixmap

        class _W:
            def __init__(self, name, row, col):
                self.name = name; self.row = row; self.col = col

        class _Plate:
            rows = 2; cols = 3
            well_spacing_x = 9.0

            def get_all_wells(self):
                return [_W("A1", 0, 0), _W("A2", 0, 1), _W("B1", 1, 0)]

        nav = WellPlateNavigator()
        nav.set_plate(_Plate())
        nav.set_current_well("A1")
        nav.set_raster_grid(7, 7)
        self.assertEqual((nav._raster_cols, nav._raster_rows), (7, 7))
        nav.resize(220, 140)
        pm = QPixmap(nav.size())
        nav.render(pm)   # exercises paintEvent with the grid overlay
        nav.set_raster_grid(0, 0)
        self.assertEqual((nav._raster_cols, nav._raster_rows), (0, 0))


if __name__ == "__main__":
    unittest.main()
