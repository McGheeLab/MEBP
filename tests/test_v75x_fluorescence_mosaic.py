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
