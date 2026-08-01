"""
v7.5.x — Full-plate mosaic scan + detect-all-wells + toggleable overlay.

Covers the new pieces wired for the operator request "raster the whole plate,
stitch the snapshots, circle-detect from the stitched field, and overlay it on
pages like Jog":

  * MosaicStore  — PNG + absolute-µm extent persistence round-trip.
  * WellDetector.detect_wells — multi-circle detection on a stitched field.
  * JogWorkspaceView.set_mosaic_overlay / set_mosaic_visible + pixmap_from_bgr.
  * CalibrationPage._ploc_detect_and_fit_from_mosaic — mosaic px → absolute
    stage µm inversion + predicted-well matching feeds the warp fit.
  * CalibrationPage Plate Location UI exposes the Mosaic scan button + toggle.
  * JogPage._load_mosaic_overlay enables the toggle when a mosaic exists.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402

from PySide6.QtWidgets import QApplication  # noqa: E402
from PySide6.QtGui import QPixmap  # noqa: E402

from SupportClasses.MosaicStore import MosaicStore  # noqa: E402
from SupportClasses.VisionDetector import WellDetector, DetectionResult  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402
from gui.widgets.jog_workspace_view import JogWorkspaceView, pixmap_from_bgr  # noqa: E402


def _app():
    return QApplication.instance() or QApplication(sys.argv)


# ── MosaicStore ────────────────────────────────────────────────────

class TestMosaicStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.store = MosaicStore(Path(self._tmp.name) / "plate_mosaics.json")
        self.img = np.full((40, 60, 3), 128, dtype=np.uint8)
        self.extent = (1000.0, 2000.0, 7000.0, 6000.0)

    def tearDown(self):
        self._tmp.cleanup()

    def test_save_then_load_round_trip(self):
        ok = self.store.save("96", self.img, self.extent,
                             um_per_px=3.34, mosaic_scale=0.01, frames=12)
        self.assertTrue(ok)
        self.assertTrue(self.store.has("96"))
        self.assertEqual(self.store.get_extent_um("96"), self.extent)
        loaded = self.store.load_image("96")
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.shape, self.img.shape)

    def test_persists_across_instances(self):
        self.store.save("6", self.img, self.extent)
        store2 = MosaicStore(self.store._path)
        self.assertTrue(store2.has("6"))
        self.assertEqual(store2.get_extent_um("6"), self.extent)

    def test_missing_key_is_empty(self):
        self.assertFalse(self.store.has("nope"))
        self.assertIsNone(self.store.get_extent_um("nope"))
        self.assertIsNone(self.store.load_image("nope"))

    def test_clear_removes_image_and_meta(self):
        self.store.save("12", self.img, self.extent)
        p = self.store.image_path("12")
        self.assertTrue(Path(p).exists())
        self.store.clear("12")
        self.assertFalse(self.store.has("12"))
        self.assertFalse(Path(p).exists())


# ── WellDetector.detect_wells ──────────────────────────────────────

class TestDetectWells(unittest.TestCase):
    def test_detects_multiple_circles(self):
        # White field with 4 dark-ringed circles at known centres.
        import cv2
        img = np.full((400, 400, 3), 255, dtype=np.uint8)
        centres = [(100, 100), (300, 100), (100, 300), (300, 300)]
        for (cx, cy) in centres:
            cv2.circle(img, (cx, cy), 40, (0, 0, 0), 3)
        dets = WellDetector.detect_wells(
            img, expected_diameter_px=80, min_dist_px=120)
        self.assertGreaterEqual(len(dets), 3)
        # Every detection should sit near one of the drawn centres.
        for d in dets:
            near = min(np.hypot(d.center_px[0] - cx, d.center_px[1] - cy)
                       for cx, cy in centres)
            self.assertLess(near, 25)

    def test_empty_on_blank(self):
        img = np.full((200, 200, 3), 255, dtype=np.uint8)
        self.assertEqual(
            WellDetector.detect_wells(img, expected_diameter_px=40), [])

    def test_guards_bad_input(self):
        self.assertEqual(WellDetector.detect_wells(None, 40), [])
        img = np.zeros((10, 10, 3), dtype=np.uint8)
        self.assertEqual(WellDetector.detect_wells(img, 0), [])


# ── JogWorkspaceView overlay ───────────────────────────────────────

class TestWorkspaceOverlay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_pixmap_from_bgr(self):
        arr = np.full((20, 30, 3), 64, dtype=np.uint8)
        pm = pixmap_from_bgr(arr)
        self.assertIsInstance(pm, QPixmap)
        self.assertEqual((pm.width(), pm.height()), (30, 20))
        self.assertIsNone(pixmap_from_bgr(None))

    def test_set_and_clear_overlay(self):
        view = JogWorkspaceView()
        self.assertFalse(view.has_mosaic())
        pm = pixmap_from_bgr(np.full((10, 10, 3), 128, dtype=np.uint8))
        view.set_mosaic_overlay(pm, (0.0, 0.0, 1000.0, 1000.0))
        self.assertTrue(view.has_mosaic())
        view.set_mosaic_visible(True)
        self.assertTrue(view._mosaic_visible)
        view.set_mosaic_overlay(None, None)
        self.assertFalse(view.has_mosaic())

    def test_paint_with_overlay_does_not_raise(self):
        from PySide6.QtGui import QImage

        class _SL:  # minimal safety-limits stand-in
            xy_min_x, xy_min_y, xy_max_x, xy_max_y = 0.0, 0.0, 100000.0, 80000.0
            def check_xy_near_limit(self, *a, **k):
                return {}
        view = JogWorkspaceView()
        view.set_safety_limits(_SL())
        view.resize(400, 300)
        view.set_mosaic_overlay(
            pixmap_from_bgr(np.full((50, 60, 3), 90, dtype=np.uint8)),
            (10000.0, 10000.0, 70000.0, 60000.0))
        view.set_mosaic_visible(True)
        target = QImage(400, 300, QImage.Format.Format_ARGB32)
        # Renders the widget (→ paintEvent → _paint_mosaic_overlay) into target.
        view.render(target)


# ── CalibrationPage mosaic detect/fit + UI ─────────────────────────

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
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=None)
        return page


class TestMosaicDetectFit(_CalBase):
    def test_pixels_map_to_stage_and_match_predicted(self):
        page = self._make_page()
        page._plate = WellPlate.from_format(6)
        page._predicted_positions = (
            page._plate.get_all_positions_from_plate_center(50000.0, 40000.0))
        page._calibrated_positions = None

        # Synthetic mosaic transform: extent origin + scale (px per µm).
        scale = 0.02
        xs = [p[0] for p in page._predicted_positions.values()]
        ys = [p[1] for p in page._predicted_positions.values()]
        ox, oy = min(xs) - 4000.0, min(ys) - 4000.0
        w = int((max(xs) - ox + 4000.0) * scale)
        h = int((max(ys) - oy + 4000.0) * scale)
        extent = (ox, oy, ox + w / scale, oy + h / scale)
        mosaic = np.full((h, w, 3), 200, dtype=np.uint8)

        # Place a detection exactly at each predicted well centre (in px).
        def fake_detect(img, expected_d_px, **k):
            out = []
            for (wx, wy) in page._predicted_positions.values():
                out.append(DetectionResult(
                    center_px=((wx - ox) * scale, (wy - oy) * scale),
                    radius_px=expected_d_px / 2.0, confidence=1.0,
                    method="hough_multi"))
            return out

        orig = WellDetector.detect_wells
        WellDetector.detect_wells = staticmethod(fake_detect)
        try:
            n = page._ploc_detect_and_fit_from_mosaic(mosaic, extent, scale)
        finally:
            WellDetector.detect_wells = orig

        self.assertEqual(n, len(page._predicted_positions))
        # Each matched result lands back on its predicted well centre (≤2 µm).
        for name, (wx, wy) in page._predicted_positions.items():
            self.assertIn(name, page._ploc_well_results)
            rx, ry = page._ploc_well_results[name]
            self.assertAlmostEqual(rx, wx, delta=2.0)
            self.assertAlmostEqual(ry, wy, delta=2.0)
        # Warp fit ran → calibrated positions exist.
        self.assertIsNotNone(page._calibrated_positions)

    def test_no_scale_is_noop(self):
        page = self._make_page()
        page._plate = WellPlate.from_format(6)
        page._predicted_positions = {"A1": (1.0, 2.0)}
        self.assertEqual(
            page._ploc_detect_and_fit_from_mosaic(
                np.zeros((4, 4, 3), np.uint8), (0, 0, 4, 4), 0.0), 0)

    def test_world_shift_helper(self):
        page = self._make_page()
        # No live builder → no shift to remove.
        self.assertEqual(page._ploc_mosaic_world_shift(), (0.0, 0.0))

        class _B:
            _global_shift_um = (-3170.0, -5135.0)
        page._ploc_mosaic_builder = _B()
        self.assertEqual(page._ploc_mosaic_world_shift(), (-3170.0, -5135.0))

    def test_fit_removes_display_shift_for_navigation(self):
        # The display/registration shift folded into canvas_extent_um must NOT
        # translate the stage coords we drive to. The composite pixels are
        # placed at the raw stage frame (MosaicBuilder._canvas_origin_um), so a
        # detected well must back-project to its TRUE stage position regardless
        # of any overlay shift — else every well's target is off by the shift
        # (the real-world (-3170, -5135) µm "off by >1 FOV" calibration bug).
        page = self._make_page()
        page._plate = WellPlate.from_format(6)
        page._predicted_positions = (
            page._plate.get_all_positions_from_plate_center(50000.0, 40000.0))
        page._calibrated_positions = None

        scale = 0.02
        xs = [p[0] for p in page._predicted_positions.values()]
        ys = [p[1] for p in page._predicted_positions.values()]
        raw_ox, raw_oy = min(xs) - 4000.0, min(ys) - 4000.0   # trusted origin

        # Detections are pixels relative to the RAW (placement) origin.
        det_pixels = [((wx - raw_ox) * scale, (wy - raw_oy) * scale,
                       300.0 * scale)
                      for (wx, wy) in page._predicted_positions.values()]

        # Live builder carrying a large display shift; canvas_extent_um returns
        # raw_origin + shift, which is what the worker hands to the fit.
        shift = (-3170.0, -5135.0)

        class _B:
            _global_shift_um = shift
        page._ploc_mosaic_builder = _B()
        shifted_extent = (raw_ox + shift[0], raw_oy + shift[1],
                          raw_ox + shift[0] + 1.0, raw_oy + shift[1] + 1.0)

        n = page._ploc_fit_from_mosaic_detections(
            det_pixels, shifted_extent, scale)
        self.assertEqual(n, len(page._predicted_positions))
        # Each well lands on its TRUE predicted centre (shift removed), not on
        # the shifted position.
        for name, (wx, wy) in page._predicted_positions.items():
            rx, ry = page._ploc_well_results[name]
            self.assertAlmostEqual(rx, wx, delta=2.0)
            self.assertAlmostEqual(ry, wy, delta=2.0)


class TestManualReanchor(_CalBase):
    def test_translation_shifts_all_wells_and_persists_via_warp(self):
        page = self._make_page()
        page._plate = WellPlate.from_format(6)
        page._predicted_positions = (
            page._plate.get_all_positions_from_plate_center(50000.0, 40000.0))
        # Start from an already-calibrated map (= predicted here) + markers.
        page._calibrated_positions = dict(page._predicted_positions)
        page._reference_markers = {
            "A1": tuple(page._predicted_positions["A1"])}

        ex, ey = 3170.0, 5135.0
        n = page._ploc_apply_global_translation(ex, ey)
        self.assertEqual(n, len(page._predicted_positions))

        # Every well shifted by exactly E.
        for name, (px, py) in page._predicted_positions.items():
            cx, cy = page._calibrated_positions[name]
            self.assertAlmostEqual(cx, px + ex, delta=1e-6)
            self.assertAlmostEqual(cy, py + ey, delta=1e-6)
        # Reference markers shifted too.
        self.assertAlmostEqual(
            page._reference_markers["A1"][0],
            page._predicted_positions["A1"][0] + ex, delta=1e-6)

        # The correction persists through the warp: re-applying it to the
        # predicted grid (what _load_calibration does on restart) reproduces
        # the shifted positions — NOT dependent on in-memory state.
        self.assertIsNotNone(page._plate_warp)
        reloaded = page._plate_warp.correct_positions(page._predicted_positions)
        for name, (px, py) in page._predicted_positions.items():
            self.assertAlmostEqual(reloaded[name][0], px + ex, delta=1.0)
            self.assertAlmostEqual(reloaded[name][1], py + ey, delta=1.0)

    def test_reanchor_state_and_button(self):
        page = self._make_page()
        self.assertTrue(hasattr(page, "_ploc_btn_reanchor"))
        self.assertIsNone(page._ploc_reanchor_stage)


class TestMosaicUI(_CalBase):
    def test_button_and_view_combo_exist(self):
        page = self._make_page()
        self.assertTrue(hasattr(page, "_ploc_btn_mosaic"))
        self.assertTrue(hasattr(page, "_ploc_view_combo"))
        # Three plate-view modes: ideal well / mosaic / overlay.
        self.assertEqual(page._ploc_view_combo.count(), 3)
        self.assertEqual(
            [page._ploc_view_combo.itemData(i) for i in range(3)],
            ["well", "mosaic", "overlay"])
        self.assertFalse(page._ploc_mosaic_running)

    def test_plate_key_prefers_live_plate(self):
        page = self._make_page()
        page._plate = WellPlate.from_format(24)
        self.assertEqual(page._ploc_plate_key(), "24")

    def test_view_mode_sets_visibility(self):
        page = self._make_page()
        view = page._ploc_plate_view

        # Mosaic-only: mosaic visible, ideal well grid hidden.
        page._ploc_view_combo.setCurrentIndex(
            page._ploc_view_combo.findData("mosaic"))
        self.assertTrue(page._ploc_mosaic_show)
        self.assertTrue(view._mosaic_visible)
        self.assertFalse(view._wells_visible)

        # Overlay: both visible.
        page._ploc_view_combo.setCurrentIndex(
            page._ploc_view_combo.findData("overlay"))
        self.assertTrue(view._mosaic_visible)
        self.assertTrue(view._wells_visible)

        # Ideal well: mosaic hidden, well grid visible.
        page._ploc_view_combo.setCurrentIndex(
            page._ploc_view_combo.findData("well"))
        self.assertFalse(page._ploc_mosaic_show)
        self.assertFalse(view._mosaic_visible)
        self.assertTrue(view._wells_visible)


# ── Jog page overlay load ──────────────────────────────────────────

class TestJogOverlayLoad(_CalBase):
    def test_load_enables_toggle_when_mosaic_present(self):
        import SupportClasses.MosaicStore as ms
        from gui.pages.jog_control import JogControlPage

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicStore(Path(tmp.name) / "plate_mosaics.json")
        store.save("6", np.full((20, 30, 3), 100, dtype=np.uint8),
                   (0.0, 0.0, 3000.0, 2000.0))
        # Point the module singleton at our temp store.
        orig = ms._store_singleton
        ms._store_singleton = store
        try:
            ctrl = MagicMock()
            ctrl.is_xy_connected = False
            ctrl.is_zp_connected = False
            ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
            ctrl.safety_limits = MagicMock()
            ctrl.z_up_sign.return_value = -1.0
            page = JogControlPage(ctrl)
            page._plate = WellPlate.from_format(6)
            page._load_mosaic_overlay()
            # Mosaic-dependent view modes become enabled when a mosaic loads.
            model = page._plate_view_combo.model()
            self.assertTrue(model.item(1).isEnabled())
            self.assertTrue(model.item(2).isEnabled())
            self.assertTrue(page._workspace_view.has_mosaic())
        finally:
            ms._store_singleton = orig


# ── Camera fresh-frame capture (v7.5.x defect fixes) ───────────────

class _FakeCapture:
    """Minimal cv2.VideoCapture stand-in whose read() advances a counter."""
    def __init__(self):
        self.reads = 0

    def isOpened(self):
        return True

    def read(self):
        self.reads += 1
        return True, np.full((4, 4, 3), self.reads, dtype=np.uint8)


class TestFreshFrameCapture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _widget(self):
        from gui.widgets.camera_widget import CameraWidget
        return CameraWidget(camera_label="T", compact=True)

    def test_frame_counter_starts_zero_and_is_threadsafe_getter(self):
        w = self._widget()
        self.assertEqual(w.frame_count_value(), 0)

    def test_capture_fresh_frame_drains_buffer(self):
        w = self._widget()
        w._backend_type = "opencv"
        cap = _FakeCapture()
        w._capture = cap
        # discard 3 buffered frames, then return the 4th (current) frame.
        frame = w.capture_fresh_frame(discard_n_frames=3, settle_ms=0)
        self.assertEqual(cap.reads, 4)
        self.assertIsNotNone(frame)
        self.assertEqual(int(frame[0, 0, 0]), 4)

    def test_capture_fresh_frame_no_backend_returns_none(self):
        w = self._widget()
        w._backend_type = "opencv"
        w._capture = None
        self.assertIsNone(w.capture_fresh_frame(discard_n_frames=2))


# ── Mosaic scan worker thread (runs off the GUI/grabber thread) ────

class _FakeCtrl:
    def __init__(self):
        self.moves = []          # (x, y, safe_z, target_z, kind)
        self._last = (0.0, 0.0)
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self.poller_suspended = False

    def safe_travel_to(self, x, y, safe_z_mm=None, target_z_mm=None,
                       apply_insert_floor=True):
        self.moves.append((x, y, safe_z_mm, target_z_mm, "safe"))
        self._last = (x, y)

    def move_xy_absolute_um(self, x, y, fast=False):
        # Pure XY (no ZP) — Z stays at safe; recorded with no safe_z/target_z.
        self.moves.append((x, y, None, None, "xy"))
        self._last = (x, y)

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def get_xy_position(self, cached=False):
        return (self._last[0], self._last[1])

    def suspend_position_poller(self):
        self.poller_suspended = True

    def resume_position_poller(self):
        self.poller_suspended = False


class _FakeCam:
    """Camera stand-in: frame_count_value advances fast (no real wait)."""
    def __init__(self, frame):
        self._frame = frame
        self._c = 0

    def frame_count_value(self):
        self._c += 10
        return self._c

    def get_current_frame(self):
        return self._frame.copy()


class TestMosaicWorker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_worker_moves_captures_and_finishes(self):
        from gui.pages.calibration import _MosaicScanWorker
        from SupportClasses.MosaicBuilder import MosaicBuilder

        frame = np.full((60, 80, 3), 128, dtype=np.uint8)
        builder = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                                overlap=0.25, target_mosaic_px=400)
        grid = builder.generate_raster_positions((0, 0, 2000, 1500),
                                                 overlap=0.25)
        self.assertGreater(len(grid), 1)

        ctrl = _FakeCtrl()
        cam = _FakeCam(frame)
        worker = _MosaicScanWorker(ctrl, cam, builder, grid, safe_z=5.0,
                                   expected_d_px=40.0, min_dist_px=20.0)
        tiles = []
        finished = {}
        worker.tile.connect(lambda c, e: tiles.append(c))
        worker.finished_ok.connect(
            lambda c, e, s, f, d: finished.update(
                comp=c, extent=e, scale=s, frames=f, dets=d))
        worker.run()  # synchronous (same-thread) for the test

        # One move per grid point; every move stays retracted (target_z None).
        self.assertEqual(len(ctrl.moves), len(grid))
        self.assertTrue(all(m[3] is None for m in ctrl.moves))
        # Only the FIRST tile does a full safe-travel (Z confirm on the ZP);
        # every later tile is a pure XY move (no per-tile ZP traffic).
        self.assertEqual(ctrl.moves[0][4], "safe")
        self.assertEqual(ctrl.moves[0][2], 5.0)
        self.assertTrue(all(m[4] == "xy" for m in ctrl.moves[1:]))
        # The position poller was suspended during the raster (resumed after).
        self.assertFalse(ctrl.poller_suspended)
        # One live-preview tile per point + a finished payload.
        self.assertEqual(len(tiles), len(grid))
        self.assertEqual(finished.get("frames"), len(grid))
        self.assertIsInstance(finished.get("dets"), list)

    def test_worker_stop_halts_early(self):
        from gui.pages.calibration import _MosaicScanWorker
        from SupportClasses.MosaicBuilder import MosaicBuilder
        frame = np.full((60, 80, 3), 128, dtype=np.uint8)
        builder = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                                overlap=0.25, target_mosaic_px=400)
        grid = builder.generate_raster_positions((0, 0, 4000, 3000),
                                                 overlap=0.25)
        ctrl = _FakeCtrl()
        worker = _MosaicScanWorker(ctrl, _FakeCam(frame), builder, grid,
                                   safe_z=0.0, expected_d_px=40.0,
                                   min_dist_px=20.0)
        worker.stop()      # pre-stopped → loop exits immediately
        worker.run()
        self.assertEqual(len(ctrl.moves), 0)

    def test_worker_does_not_double_orient(self):
        # v7.5.x regression: the coarse per-tile "frame_orient" (rot180/flip) is
        # RETIRED — tile orientation is applied ONCE by the builder's calibrated
        # _orient_tile (rotation + flip X + flip Y from the ground-truth store).
        # A stale mosaic_scan.frame_orient="rot180" double-oriented the WHOLE-plate
        # scan (builder _orient_tile + coarse rot180) while the calibration dialog
        # (built with settings={} → "none") came out right — the exact
        # "calibration correct, full plate wrong" report.
        from gui.pages.calibration import _MosaicScanWorker
        from SupportClasses.MosaicBuilder import MosaicBuilder
        frame = np.zeros((60, 80, 3), dtype=np.uint8)
        frame[5, 5] = (255, 255, 255)          # off-centre marker
        builder = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0)
        worker = _MosaicScanWorker(
            _FakeCtrl(), _FakeCam(frame), builder, [(0, 0)], safe_z=0.0,
            expected_d_px=0.0, min_dist_px=1.0, frame_orient="rot180")
        # The stale coarse setting is IGNORED (not stored, not applied).
        self.assertEqual(worker._frame_orient, "none")
        out = worker._orient_frame(frame)       # retired → identity no-op
        self.assertTrue(np.array_equal(out, frame))

    def test_worker_aborts_on_sustained_no_frames(self):
        # A camera that never delivers frames → the worker aborts (failed)
        # after _MAX_CONSEC_NONE tiles instead of silently building nothing.
        from gui.pages.calibration import _MosaicScanWorker
        from SupportClasses.MosaicBuilder import MosaicBuilder

        class _NoneCam:
            def __init__(self):
                self._c = 0

            def frame_count_value(self):
                self._c += 10
                return self._c

            def get_current_frame(self):
                return None

        builder = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                                overlap=0.25, target_mosaic_px=400)
        grid = builder.generate_raster_positions((0, 0, 6000, 5000),
                                                 overlap=0.25)
        self.assertGreater(len(grid), _MOSAIC_MAX_CONSEC())
        ctrl = _FakeCtrl()
        worker = _MosaicScanWorker(ctrl, _NoneCam(), builder, grid, safe_z=0.0,
                                   expected_d_px=40.0, min_dist_px=20.0)
        failed = []
        finished = []
        worker.failed.connect(lambda m: failed.append(m))
        worker.finished_ok.connect(lambda *a: finished.append(a))
        worker.run()
        self.assertEqual(len(failed), 1)
        self.assertEqual(len(finished), 0)
        self.assertEqual(len(ctrl.moves), _MOSAIC_MAX_CONSEC())


def _MOSAIC_MAX_CONSEC():
    from gui.pages.calibration import _MosaicScanWorker
    return _MosaicScanWorker._MAX_CONSEC_NONE


class TestReentrancyAndShutdown(_CalBase):
    def test_run_queue_blocked_during_mosaic_scan(self):
        page = self._make_page()
        page._ploc_running = False
        page._ploc_mosaic_running = True
        page._ploc_run_queue()       # must early-return, not start a queue run
        self.assertFalse(page._ploc_running)

    def test_shutdown_mosaic_worker_no_worker_is_safe(self):
        page = self._make_page()
        page._ploc_mosaic_running = True
        page._ploc_mosaic_worker = None
        page._shutdown_mosaic_worker()   # no crash
        self.assertFalse(page._ploc_mosaic_running)

    def test_stale_tile_after_cancel_ignored(self):
        page = self._make_page()
        page._ploc_mosaic_running = False   # as after a cancel
        img = np.full((10, 10, 3), 50, dtype=np.uint8)
        # Should no-op (guard on _ploc_mosaic_running) — no exception, no paint.
        page._ploc_on_mosaic_tile(img, (0.0, 0.0, 1000.0, 1000.0))
        page._ploc_on_mosaic_progress(3, 9)


# ── Bounds clipping + overlap + preview ────────────────────────────

class TestBoundsAndOverlap(_CalBase):
    def test_overlap_has_exactly_one_home(self):
        """v7.5.x: the dead ``_PLOC_MOSAIC_OVERLAP`` class constant is gone.

        It was a fourth value for the tile overlap that nothing read, alongside
        the configured setting, the calibration dialog's own default and a
        hardcoded 0.50 in the legacy per-well scan. Overlap now lives only in
        ``mosaic_scan.overlap_pct`` and is resolved by MosaicCalibration.
        """
        from gui.pages.calibration import CalibrationPage
        from SupportClasses.MosaicCalibration import (
            SCAN_DEFAULTS, RECOMMENDED_OVERLAP_FRAC)
        self.assertFalse(hasattr(CalibrationPage, "_PLOC_MOSAIC_OVERLAP"))
        self.assertEqual(SCAN_DEFAULTS["overlap_pct"], 25)
        self.assertAlmostEqual(RECOMMENDED_OVERLAP_FRAC, 0.25)

    def test_clip_bounds_to_envelope(self):
        page = self._make_page()
        # Wells span well beyond a small envelope in +Y.
        xs = [10000.0, 90000.0]
        ys = [10000.0, 200000.0]
        env = (0.0, 0.0, 114332.0, 76645.0)
        b = page._ploc_clip_scan_bounds(xs, ys, margin=1000.0, env=env)
        self.assertIsNotNone(b)
        # Clipped to the envelope on the overshooting max edges.
        self.assertLessEqual(b[2], 114332.0)
        self.assertLessEqual(b[3], 76645.0)
        self.assertGreaterEqual(b[0], 0.0)
        self.assertGreaterEqual(b[1], 0.0)

    def test_clip_bounds_empty_when_outside_envelope(self):
        page = self._make_page()
        # All wells beyond the envelope's max X → empty intersection.
        xs = [200000.0, 210000.0]
        ys = [10000.0, 20000.0]
        env = (0.0, 0.0, 114332.0, 76645.0)
        self.assertIsNone(
            page._ploc_clip_scan_bounds(xs, ys, margin=500.0, env=env))

    def test_live_mosaic_preview_widget_exists(self):
        page = self._make_page()
        self.assertTrue(hasattr(page, "_ploc_mosaic_preview"))

    def test_preview_helpers_do_not_raise(self):
        # Regression: _ploc_reset_mosaic_preview used QPixmap() which was not
        # imported in calibration.py → NameError → mosaic-scan click was a
        # silent no-op. Both preview helpers must run cleanly.
        page = self._make_page()
        page._ploc_reset_mosaic_preview()
        img = np.full((30, 40, 3), 70, dtype=np.uint8)
        page._ploc_update_mosaic_preview(img)
        self.assertFalse(page._ploc_mosaic_preview.pixmap().isNull())

    def test_fit_from_precomputed_detections(self):
        page = self._make_page()
        page._plate = WellPlate.from_format(6)
        page._predicted_positions = (
            page._plate.get_all_positions_from_plate_center(50000.0, 40000.0))
        scale = 0.02
        xs = [p[0] for p in page._predicted_positions.values()]
        ys = [p[1] for p in page._predicted_positions.values()]
        ox, oy = min(xs) - 4000.0, min(ys) - 4000.0
        extent = (ox, oy, max(xs) + 4000.0, max(ys) + 4000.0)
        # One detection at each predicted well centre, in mosaic px.
        det_pixels = [((wx - ox) * scale, (wy - oy) * scale, 50.0)
                      for (wx, wy) in page._predicted_positions.values()]
        n = page._ploc_fit_from_mosaic_detections(det_pixels, extent, scale)
        self.assertEqual(n, len(page._predicted_positions))
        for name, (wx, wy) in page._predicted_positions.items():
            rx, ry = page._ploc_well_results[name]
            self.assertAlmostEqual(rx, wx, delta=2.0)
            self.assertAlmostEqual(ry, wy, delta=2.0)


# ── Mosaic scan settings dialog + wiring ───────────────────────────

class _FakeSettings:
    def __init__(self, sections=None):
        self._s = dict(sections or {})
    def get_section(self, k):
        return self._s.get(k)
    def set_section(self, k, v):
        self._s[k] = v
    def get(self, k, default=None):
        return default
    def set(self, k, v):
        pass
    def save(self):
        pass


class TestMosaicSettings(_CalBase):
    def test_merged_settings_overlays_defaults(self):
        from gui.dialogs.mosaic_settings_dialog import (
            merged_settings, MOSAIC_SCAN_DEFAULTS)
        self.assertEqual(merged_settings(None), MOSAIC_SCAN_DEFAULTS)
        m = merged_settings({"settle_ms": 999, "bogus": 1})
        self.assertEqual(m["settle_ms"], 999)
        self.assertEqual(m["overlap_pct"],
                         MOSAIC_SCAN_DEFAULTS["overlap_pct"])
        self.assertNotIn("bogus", m)

    def test_dialog_round_trip_and_defaults(self):
        from gui.dialogs.mosaic_settings_dialog import (
            MosaicScanSettingsDialog, MOSAIC_SCAN_DEFAULTS)
        # v7.5.x: frame_orient / fov_um / spacing_um are RETIRED (orientation and
        # scale are measured in one place — see MOSAIC_SCAN_DEFAULTS' note), so
        # they are no longer part of the round-trip.
        custom = {"overlap_pct": 40, "settle_ms": 500, "fresh_frames": 6,
                  "fresh_timeout_s": 4.0, "target_px": 4000,
                  "detect_param2": 25, "detect_tol_pct": 50,
                  "register": False, "max_shift_um": 120,
                  "reg_method": "phase",
                  "cal_cols": 5, "cal_rows": 5}
        dlg = MosaicScanSettingsDialog(custom)
        self.assertEqual(dlg.values(), custom)
        dlg.set_values(MOSAIC_SCAN_DEFAULTS)
        self.assertEqual(dlg.values(), MOSAIC_SCAN_DEFAULTS)

    def test_retired_keys_are_not_offered_or_resurrected(self):
        """v7.5.x REGRESSION: the retired override keys must not reappear.

        ``frame_orient`` double-oriented the mosaic (and only the fluorescence
        scan honoured it, so the same camera produced two orientations);
        ``fov_um`` / ``spacing_um`` overrode every measured value from a hidden
        submenu. A stale settings.json may still carry them — they must be
        ignored, not round-tripped back out.
        """
        from gui.dialogs.mosaic_settings_dialog import (
            MosaicScanSettingsDialog, MOSAIC_SCAN_DEFAULTS, merged_settings)
        for key in ("frame_orient", "fov_um", "spacing_um"):
            self.assertNotIn(key, MOSAIC_SCAN_DEFAULTS)
        stale = {"frame_orient": "rot180", "fov_um": 2200, "spacing_um": 1500,
                 "overlap_pct": 5}
        self.assertNotIn("frame_orient", merged_settings(stale))
        self.assertNotIn("fov_um", merged_settings(stale))
        dlg = MosaicScanSettingsDialog(stale)
        out = dlg.values()
        self.assertNotIn("frame_orient", out)
        self.assertNotIn("fov_um", out)
        self.assertNotIn("spacing_um", out)

    def test_settings_dialog_preserves_undisplayed_keys(self):
        # Opening Settings + OK must not drop keys it has no control for
        # (e.g. the calibration grid size).
        from gui.dialogs.mosaic_settings_dialog import MosaicScanSettingsDialog
        dlg = MosaicScanSettingsDialog({"cal_cols": 7, "cal_rows": 9})
        out = dlg.values()
        self.assertEqual(out["cal_cols"], 7)
        self.assertEqual(out["cal_rows"], 9)

    # v7.5.x: ``test_frame_orient_transforms_tile`` DELETED. It asserted the
    # retired coarse per-tile transform still applied
    # (``w._frame_orient == "rot180"``, ``_orient_frame`` rotating the frame) and
    # so directly contradicted its own sibling
    # ``TestMosaicWorker.test_worker_does_not_double_orient``, which pins the
    # correct behaviour: the worker forces "none" and orientation is applied ONCE
    # by the builder's calibrated ``_orient_tile``. It had been failing silently
    # because ``TestManualAlignPage`` hangs alphabetically before this class.

    def test_worker_stores_timing_and_detect_params(self):
        from gui.pages.calibration import _MosaicScanWorker
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0)
        w = _MosaicScanWorker(
            _FakeCtrl(), _FakeCam(np.zeros((60, 80, 3), np.uint8)), b,
            [(0, 0)], 0.0, 40.0, 20.0, fresh_frames=7, fresh_timeout_s=5.0,
            settle_ms=400, detect_param2=22.0, detect_tolerance=0.5)
        self.assertEqual(w._settle_ms, 400)
        self.assertEqual(w._fresh_frames, 7)
        self.assertAlmostEqual(w._fresh_timeout_s, 5.0)
        self.assertEqual(w._detect_param2, 22.0)
        self.assertAlmostEqual(w._detect_tolerance, 0.5)

    def test_page_loads_settings_from_store(self):
        from gui.pages.calibration import CalibrationPage
        from unittest.mock import MagicMock
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.get_zp_position.return_value = (None, None, None)
        ctrl.z_up_sign.return_value = -1.0
        stngs = _FakeSettings({"mosaic_scan": {"settle_ms": 750}})
        page = CalibrationPage(ctrl, settings=stngs)
        self.assertEqual(page._mosaic_settings["settle_ms"], 750)

    def test_open_settings_applies_and_persists(self):
        import gui.dialogs.mosaic_settings_dialog as msd

        new_vals = {"overlap_pct": 33, "settle_ms": 600, "fresh_frames": 5,
                    "fresh_timeout_s": 3.0, "target_px": 3500,
                    "detect_param2": 28, "detect_tol_pct": 45}

        class _FakeDlg:
            def __init__(self, settings=None, parent=None):
                pass
            def exec(self):
                return True
            def values(self):
                return dict(new_vals)

        page = self._make_page()
        page.settings = _FakeSettings()
        orig = msd.MosaicScanSettingsDialog
        msd.MosaicScanSettingsDialog = _FakeDlg
        try:
            page._ploc_open_mosaic_settings()
        finally:
            msd.MosaicScanSettingsDialog = orig
        self.assertEqual(page._mosaic_settings, new_vals)
        self.assertEqual(page.settings.get_section("mosaic_scan"), new_vals)


class TestMosaicSpacingAndRegistration(unittest.TestCase):
    def test_generate_raster_step_override(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(100, 100), micron_per_pixel=10.0)
        pos = b.generate_raster_positions(
            (0, 0, 5000, 5000), overlap=0.25, step_x_um=300, step_y_um=300)
        xs = sorted({round(x, 1) for x, _ in pos})
        diffs = [round(xs[i + 1] - xs[i]) for i in range(len(xs) - 1)]
        # Every interior step is the override (300); the final column may be a
        # clamped remainder (≤ 300) covering the boundary.
        self.assertTrue(all(d == 300 for d in diffs[:-1]), diffs)
        self.assertLessEqual(diffs[-1], 300)
        self.assertGreater(diffs.count(300), 5)   # override took effect

    def test_max_shift_px_manual_and_auto(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                          target_mosaic_px=300, register=True, max_shift_um=80)
        b.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        self.assertAlmostEqual(b._max_shift_px(), 80 * b._mosaic_scale, places=3)
        b2 = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                           target_mosaic_px=300)  # max_shift_um=0 → auto
        b2.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        fov_w_px = 60 * 10.0 * b2._mosaic_scale
        self.assertAlmostEqual(b2._max_shift_px(), 0.2 * fov_w_px, places=3)

    def test_registration_stitch_runs_and_finalizes(self):
        import cv2
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                          overlap=0.5, target_mosaic_px=300, register=True,
                          max_shift_um=80)
        b.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        f = np.full((60, 60, 3), 40, dtype=np.uint8)
        cv2.circle(f, (30, 30), 12, (220, 220, 220), -1)
        b.add_raster_frame(f, 300.0, 300.0, 0)
        b.stitch_incremental()
        b.add_raster_frame(f, 600.0, 300.0, 1)
        b.stitch_incremental()
        self.assertIsNotNone(b.composite)
        # A single GLOBAL shift, bounded by max_shift_um, applied to the extent.
        shift = b.finalize_global_shift()
        self.assertEqual(len(shift), 2)
        self.assertLessEqual(abs(shift[0]), 80 + 1e-6)
        self.assertLessEqual(abs(shift[1]), 80 + 1e-6)

    def test_featureless_tiles_contribute_no_shift(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                          overlap=0.5, target_mosaic_px=300, register=True)
        b.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        flat = np.full((60, 60, 3), 100, dtype=np.uint8)   # uniform → no texture
        b.add_raster_frame(flat, 300.0, 300.0, 0)
        b.stitch_incremental()
        b.add_raster_frame(flat, 600.0, 300.0, 1)
        b.stitch_incremental()
        # Featureless overlap → no measurement collected, global shift = 0.
        self.assertEqual(b._measured_shifts, [])
        self.assertEqual(b.finalize_global_shift(), (0.0, 0.0))

    def test_global_shift_is_median_applied_to_extent(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                          target_mosaic_px=300, register=True,
                          max_shift_um=100000)   # large bound → no clamp
        b.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        raw = tuple(b._canvas_extent_um)
        b._measured_shifts = [(4.0, 2.0), (6.0, 2.0), (5.0, 3.0)]  # px
        shift = b.finalize_global_shift()                          # median (5,2)
        scale = b._mosaic_scale
        self.assertAlmostEqual(shift[0], 5.0 / scale, places=3)
        self.assertAlmostEqual(shift[1], 2.0 / scale, places=3)
        ext = b.canvas_extent_um
        self.assertAlmostEqual(ext[0], raw[0] + 5.0 / scale, places=2)
        self.assertAlmostEqual(ext[2], raw[2] + 5.0 / scale, places=2)
        self.assertAlmostEqual(ext[1], raw[1] + 2.0 / scale, places=2)

    def test_per_tile_mode_retained(self):
        # The per-tile registration mode is kept for future apps (not used by
        # the plate mosaic scan, which uses "global").
        import cv2
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                          overlap=0.5, target_mosaic_px=300, register=True,
                          max_shift_um=80, register_mode="per_tile")
        self.assertEqual(b._register_mode, "per_tile")
        b.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        f = np.full((60, 60, 3), 40, dtype=np.uint8)
        cv2.circle(f, (30, 30), 12, (220, 220, 220), -1)
        b.add_raster_frame(f, 300.0, 300.0, 0)
        b.stitch_incremental()
        b.add_raster_frame(f, 600.0, 300.0, 1)
        b.stitch_incremental()
        # Per-tile mode applies shifts inline → nothing queued for a global one.
        self.assertEqual(b._measured_shifts, [])
        self.assertIsNotNone(b.composite)

    def test_default_mode_is_global(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(register=True)
        self.assertEqual(b._register_mode, "global")

    def test_initial_shift_preseed_and_kept(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(60, 60), micron_per_pixel=10.0,
                          target_mosaic_px=300, register=True,
                          initial_shift_um=(7.0, -3.0))
        b.generate_raster_positions((0, 0, 1200, 1200), overlap=0.5)
        raw = tuple(b._canvas_extent_um)
        ext = b.canvas_extent_um            # pre-seeded shift applied
        self.assertAlmostEqual(ext[0], raw[0] + 7.0, places=3)
        self.assertAlmostEqual(ext[1], raw[1] - 3.0, places=3)
        # No fresh measurements → the learned pre-seed is KEPT (not zeroed).
        shift = b.finalize_global_shift()
        self.assertAlmostEqual(shift[0], 7.0, places=3)
        self.assertAlmostEqual(shift[1], -3.0, places=3)


class TestMosaicAlignmentStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        self.store = MosaicAlignmentStore(
            Path(self._tmp.name) / "mosaic_alignment.json")

    def tearDown(self):
        self._tmp.cleanup()

    def test_um_per_px_and_shift_round_trip(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        self.store.set_um_per_px("camA|10x", 3.21, source="quick_fov")
        self.store.set_shift_um("camA|10x", 12.0, -3.0, frames=40)
        self.assertAlmostEqual(self.store.get_um_per_px("camA|10x"), 3.21,
                               places=4)
        self.assertEqual(self.store.get_shift_um("camA|10x"), (12.0, -3.0))
        # Persists across instances.
        s2 = MosaicAlignmentStore(self.store._path)
        self.assertAlmostEqual(s2.get_um_per_px("camA|10x"), 3.21, places=4)
        self.assertEqual(s2.get_shift_um("camA|10x"), (12.0, -3.0))

    def test_missing_and_clear(self):
        self.assertIsNone(self.store.get_um_per_px("nope"))
        self.assertIsNone(self.store.get_shift_um("nope"))
        self.store.set_um_per_px("k", 2.0)
        self.store.clear("k")
        self.assertIsNone(self.store.get_um_per_px("k"))

    def test_rejects_nonpositive_um(self):
        self.store.set_um_per_px("k", 0.0)
        self.assertIsNone(self.store.get_um_per_px("k"))

    def test_is_manual_flags_manual_align_source(self):
        self.store.set_shift_um("k", 1.0, 2.0, source="mosaic_scan")
        self.assertFalse(self.store.is_manual("k"))
        self.store.set_shift_um("k", 3.0, 4.0, source="manual_align")
        self.assertTrue(self.store.is_manual("k"))
        self.assertFalse(self.store.is_manual("absent"))


class TestCameraObjectiveKey(_CalBase):
    def test_key_falls_back_to_objective(self):
        from types import SimpleNamespace
        page = self._make_page()
        page._camera_manager = None   # no identity → objective-only key
        page._hardware_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="10x"),
            camera_for_role=lambda role: 0)
        self.assertEqual(page._ploc_camera_objective_key(), "10x")

    def test_key_default_when_no_objective(self):
        page = self._make_page()
        page._camera_manager = None
        page._hardware_config = None
        self.assertEqual(page._ploc_camera_objective_key(), "default")


class TestMosaicCalibrationDialog(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dlg(self, store=None):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        from unittest.mock import MagicMock
        return MosaicCalibrationDialog(
            MagicMock(), MagicMock(), 0, safe_z=5.0, align_key="camX|10x",
            store=store,
            settings={"cal_cols": 5, "cal_rows": 5, "overlap_pct": 25,
                      "settle_ms": 300, "fresh_frames": 3, "fov_um": 0,
                      "max_shift_um": 0, "frame_orient": "none"},
            center_um=(50000.0, 40000.0), frame_size=(80, 60),
            um_per_px_camera=10.0)

    def test_grid_defaults_5x5(self):
        self.assertEqual(self._dlg().grid_values(), (5, 5))

    def test_centered_scan_bounds_yields_grid(self):
        from gui.dialogs.mosaic_calibration_dialog import centered_scan_bounds
        from SupportClasses.MosaicBuilder import MosaicBuilder
        fov_w, fov_h = 800.0, 600.0
        step_x, step_y = fov_w * 0.75, fov_h * 0.75    # 25% overlap
        b = centered_scan_bounds(50000, 40000, 5, 5, step_x, step_y,
                                 fov_w, fov_h)
        mb = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0)
        pos = mb.generate_raster_positions(b, overlap=0.25, step_x_um=step_x,
                                           step_y_um=step_y)
        xs = sorted({round(x) for x, _ in pos})
        ys = sorted({round(y) for _, y in pos})
        self.assertEqual(len(xs), 5)
        self.assertEqual(len(ys), 5)
        self.assertEqual(len(pos), 25)
        self.assertAlmostEqual(sum(xs) / len(xs), 50000, delta=1.0)  # centred

    def test_finished_stores_global_delta(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicBuilder import MosaicBuilder
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        dlg = self._dlg(store=store)
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                          register=True, initial_shift_um=(8.0, -2.0))
        b.generate_raster_positions((0, 0, 2000, 1500), overlap=0.25)
        b._measured_shifts = [(1.0, 1.0)]
        dlg._builder = b
        dlg._worker = object()    # truthy → _on_finished proceeds
        comp = np.full((50, 60, 3), 120, dtype=np.uint8)
        dlg._on_finished(comp, b.canvas_extent_um, b._mosaic_scale, 25, [])
        self.assertEqual(store.get_shift_um("camX|10x"), (8.0, -2.0))

    def test_finished_no_overlaps_preserves_prior(self):
        # A calibration that registers ZERO textured overlaps must NOT overwrite
        # a previously-learned shift with (0,0).
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicBuilder import MosaicBuilder
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        store.set_shift_um("camX|10x", 5.0, 5.0, frames=10)   # prior good value
        dlg = self._dlg(store=store)
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                          register=True, initial_shift_um=(5.0, 5.0))
        b.generate_raster_positions((0, 0, 2000, 1500), overlap=0.25)
        b._measured_shifts = []     # nothing registered
        dlg._builder = b
        dlg._worker = object()
        comp = np.full((50, 60, 3), 120, dtype=np.uint8)
        dlg._on_finished(comp, b.canvas_extent_um, b._mosaic_scale, 9, [])
        self.assertEqual(store.get_shift_um("camX|10x"), (5.0, 5.0))   # kept


class TestCalibrateSafeZGate(_CalBase):
    def test_calibrate_blocks_without_safe_z_when_zp_connected(self):
        import gui.pages.calibration as calmod
        from unittest.mock import MagicMock

        class _MB:
            warned = []
            @staticmethod
            def warning(*a, **k):
                _MB.warned.append(a[2] if len(a) > 2 else "")
            @staticmethod
            def information(*a, **k):
                pass

        page = self._make_page()
        page.controller.is_zp_connected = True
        page._safe_z = None
        page._camera_manager = MagicMock()
        orig = calmod.QMessageBox
        calmod.QMessageBox = _MB
        try:
            page._ploc_quick_mosaic_calibrate()
        finally:
            calmod.QMessageBox = orig
        # Refused with a Safe-Z warning before any stage move / dialog.
        self.assertTrue(any("Safe" in w for w in _MB.warned), _MB.warned)


# ── Manual global registration (slide mosaic onto wells by eye) ────

class TestWorkspaceManualShift(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_shift_and_opacity_setters(self):
        view = JogWorkspaceView()
        self.assertEqual(view.mosaic_shift(), (0.0, 0.0))
        view.set_mosaic_shift(12.0, -7.0)
        self.assertEqual(view.mosaic_shift(), (12.0, -7.0))
        view.set_mosaic_opacity(0.3)
        self.assertAlmostEqual(view._mosaic_opacity, 0.3, places=3)
        # Opacity is clamped to a visible range.
        view.set_mosaic_opacity(5.0)
        self.assertLessEqual(view._mosaic_opacity, 1.0)
        view.set_mosaic_opacity(-1.0)
        self.assertGreaterEqual(view._mosaic_opacity, 0.05)

    def test_new_overlay_resets_user_shift(self):
        view = JogWorkspaceView()
        view.set_mosaic_shift(40.0, 40.0)
        pm = pixmap_from_bgr(np.full((10, 10, 3), 128, dtype=np.uint8))
        view.set_mosaic_overlay(pm, (0.0, 0.0, 1000.0, 1000.0))
        self.assertEqual(view.mosaic_shift(), (0.0, 0.0))

    def test_paint_with_user_shift_does_not_raise(self):
        from PySide6.QtGui import QImage

        class _SL:
            xy_min_x, xy_min_y, xy_max_x, xy_max_y = 0.0, 0.0, 100000.0, 80000.0
            def check_xy_near_limit(self, *a, **k):
                return {}
        view = JogWorkspaceView()
        view.set_safety_limits(_SL())
        view.resize(400, 300)
        view.set_mosaic_overlay(
            pixmap_from_bgr(np.full((50, 60, 3), 90, dtype=np.uint8)),
            (10000.0, 10000.0, 70000.0, 60000.0))
        view.set_mosaic_visible(True)
        view.set_mosaic_shift(500.0, -800.0)
        target = QImage(400, 300, QImage.Format.Format_ARGB32)
        view.render(target)


class TestMicroscopeUmPerPxResolution(_CalBase):
    """`_ploc_microscope_um_per_px` rescales the objective µm/px to the current
    capture width (root cause of needing 50% overlap to keep tiles touching)."""

    def _patch_objectives(self, cam, obj, meas, res):
        import SupportClasses.ObjectiveCalibration as objmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = objmod.ObjectiveCalibrationStore(Path(tmp.name) / "obj.json")
        store.set_calibration(cam, obj, meas, res)
        orig = objmod._store
        objmod._store = store
        self.addCleanup(lambda: setattr(objmod, "_store", orig))

    def _page_with_cam(self, cam="C3", obj="10x"):
        from types import SimpleNamespace
        page = self._make_page()
        page._hardware_config = SimpleNamespace(
            camera_config=SimpleNamespace(
                current_objective_name=obj,
                camera_spec=SimpleNamespace(name=cam)))
        return page

    def test_rescales_for_larger_capture_width(self):
        self._patch_objectives("C3", "10x", 3.34, (912, 686))
        page = self._page_with_cam()
        # Captured at 1832 px wide → µm/px halves (≈3.34·912/1832).
        eff = page._ploc_microscope_um_per_px(1832, fallback=99.0)
        self.assertAlmostEqual(eff, 3.34 * (912.0 / 1832.0), places=4)

    def test_same_width_returns_measured(self):
        self._patch_objectives("C3", "10x", 3.34, (912, 686))
        page = self._page_with_cam()
        self.assertAlmostEqual(
            page._ploc_microscope_um_per_px(912, fallback=99.0), 3.34, places=4)

    def test_no_calibration_falls_back(self):
        page = self._page_with_cam()        # objectives store untouched/empty
        self.assertEqual(
            page._ploc_microscope_um_per_px(1832, fallback=7.5), 7.5)


class TestManualAlignPage(_CalBase):
    """Plate Location manual-align sliders + Store handler."""

    def _page_with_mosaic(self, align_store, mosaic_store):
        from types import SimpleNamespace
        import SupportClasses.MosaicAlignmentStore as amod
        import SupportClasses.MosaicStore as msmod
        amod._store_singleton = align_store
        msmod._store_singleton = mosaic_store
        self.addCleanup(lambda: setattr(amod, "_store_singleton", None))
        self.addCleanup(lambda: setattr(msmod, "_store_singleton", None))
        page = self._make_page()
        page._plate = WellPlate.from_format(6)              # plate key "6"
        page._camera_manager = None                         # objective-only key
        page._hardware_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="10x"))
        # Seed a persisted overlay so the bake path runs.
        img = np.full((20, 30, 3), 100, dtype=np.uint8)
        ext = (1000.0, 2000.0, 7000.0, 6000.0)
        mosaic_store.save("6", img, ext, um_per_px=3.0, mosaic_scale=0.01,
                          frames=12)
        page._ploc_set_overlay_image(img, ext)
        return page, ext

    def test_sliders_exist_and_nudge_view(self):
        page = self._make_page()
        self.assertTrue(hasattr(page, "_ploc_align_dx"))
        self.assertTrue(hasattr(page, "_ploc_align_op"))
        page._ploc_align_dx.setValue(25)
        page._ploc_align_dy.setValue(-15)
        self.assertEqual(page._ploc_plate_view.mosaic_shift(), (25.0, -15.0))

    def test_store_adds_prior_and_bakes_extent(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicStore import MosaicStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        astore = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        astore.set_shift_um("10x", 4.0, 1.0, frames=5)      # prior
        mstore = MosaicStore(Path(tmp.name) / "pm.json")
        page, ext = self._page_with_mosaic(astore, mstore)

        page._ploc_align_dx.setValue(10)
        page._ploc_align_dy.setValue(-6)
        page._ploc_store_manual_align()

        # Stored shift = prior + nudge.
        self.assertEqual(astore.get_shift_um("10x"), (14.0, -5.0))
        # Persisted overlay extent baked by the nudge (not the prior).
        new_ext = mstore.get_extent_um("6")
        self.assertAlmostEqual(new_ext[0], ext[0] + 10.0, places=3)
        self.assertAlmostEqual(new_ext[1], ext[1] - 6.0, places=3)
        self.assertAlmostEqual(new_ext[2], ext[2] + 10.0, places=3)
        # Live nudge zeroed after store (now lives in the extent).
        self.assertEqual(page._ploc_plate_view.mosaic_shift(), (0.0, 0.0))
        self.assertEqual(page._ploc_align_dx.value(), 0)

    def test_store_is_idempotent_no_double_apply(self):
        # Clicking Store twice without re-nudging must NOT apply the shift twice
        # (the live nudge resets to 0 after the first store).
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicStore import MosaicStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        astore = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        mstore = MosaicStore(Path(tmp.name) / "pm.json")
        page, ext = self._page_with_mosaic(astore, mstore)
        page._ploc_align_dx.setValue(12)
        page._ploc_align_dy.setValue(8)
        page._ploc_store_manual_align()
        self.assertEqual(astore.get_shift_um("10x"), (12.0, 8.0))
        first_ext = mstore.get_extent_um("6")
        page._ploc_store_manual_align()        # second click, no new nudge
        self.assertEqual(astore.get_shift_um("10x"), (12.0, 8.0))   # unchanged
        self.assertEqual(mstore.get_extent_um("6"), first_ext)      # unchanged

    def test_store_without_mosaic_is_guarded(self):
        import gui.pages.calibration as calmod

        class _MB:
            shown = []
            @staticmethod
            def information(*a, **k):
                _MB.shown.append(a[2] if len(a) > 2 else "")
        page = self._make_page()
        page._ploc_plate_view.set_mosaic_overlay(None, None)
        orig = calmod.QMessageBox
        calmod.QMessageBox = _MB
        try:
            page._ploc_store_manual_align()
        finally:
            calmod.QMessageBox = orig
        self.assertTrue(_MB.shown)

    def test_reset_zeroes_sliders(self):
        page = self._make_page()
        page._ploc_align_dx.setValue(20)
        page._ploc_align_dy.setValue(20)
        page._ploc_reset_manual_align()
        self.assertEqual(page._ploc_align_dx.value(), 0)
        self.assertEqual(page._ploc_plate_view.mosaic_shift(), (0.0, 0.0))

    def test_clear_stored_removes_shift(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicStore import MosaicStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        astore = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        astore.set_shift_um("10x", 7.0, 7.0, source="manual_align")
        mstore = MosaicStore(Path(tmp.name) / "pm.json")
        page, _ = self._page_with_mosaic(astore, mstore)
        page._ploc_clear_stored_align()
        self.assertIsNone(astore.get_shift_um("10x"))

    def test_auto_finish_keeps_manual_align(self):
        # A full scan that registers must NOT silently clobber a stored
        # manual_align shift.
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicStore import MosaicStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        astore = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        astore.set_shift_um("10x", 11.0, -9.0, source="manual_align")
        mstore = MosaicStore(Path(tmp.name) / "pm.json")
        page, _ = self._page_with_mosaic(astore, mstore)
        page._ploc_mosaic_running = True

        class _FakeBuilder:
            _measured_shifts = [(1.0, 1.0), (1.2, 0.9)]   # auto registered
            _global_shift_um = (50.0, 50.0)               # measured median
        page._ploc_mosaic_builder = _FakeBuilder()
        comp = np.full((20, 30, 3), 100, dtype=np.uint8)
        ext = (1000.0, 2000.0, 7000.0, 6000.0)
        page._ploc_on_mosaic_finished(comp, ext, 0.01, 12, [])
        # Manual value preserved (not overwritten with the measured median).
        self.assertEqual(astore.get_shift_um("10x"), (11.0, -9.0))
        self.assertTrue(astore.is_manual("10x"))


class TestCalibrationDialogHandoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dlg(self):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        return MosaicCalibrationDialog(
            MagicMock(), MagicMock(), 0, safe_z=5.0, align_key="camX|10x",
            store=None,
            settings={"cal_cols": 5, "cal_rows": 5, "overlap_pct": 25,
                      "settle_ms": 300, "fresh_frames": 3, "fov_um": 0,
                      "max_shift_um": 0, "frame_orient": "none"},
            center_um=(50000.0, 40000.0), frame_size=(80, 60),
            um_per_px_camera=10.0)

    def test_apply_settings_captures_tuned_values(self):
        dlg = self._dlg()
        self.assertIsNone(dlg.applied_settings())
        dlg._spin_overlap.setValue(40)
        dlg._spin_settle.setValue(450)
        dlg._on_apply_settings()
        applied = dlg.applied_settings()
        self.assertEqual(applied["overlap_pct"], 40)
        self.assertEqual(applied["settle_ms"], 450)

    def test_result_overlay_captured_on_finish_even_with_no_overlaps(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        dlg = self._dlg()
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                          register=True, initial_shift_um=(0.0, 0.0))
        b.generate_raster_positions((0, 0, 2000, 1500), overlap=0.25)
        b._measured_shifts = []          # nothing registered
        dlg._builder = b
        dlg._worker = object()
        comp = np.full((50, 60, 3), 120, dtype=np.uint8)
        dlg._on_finished(comp, b.canvas_extent_um, b._mosaic_scale, 9, [])
        # Composite is still handed back so the page can show it for manual align.
        rc, re = dlg.result_overlay()
        self.assertIsNotNone(rc)
        self.assertIsNotNone(re)


class TestWorkspaceZoomPan(unittest.TestCase):
    """Opt-in zoom/pan on JogWorkspaceView (for manual mosaic registration).
    Default-off so Jog / Plate Location are unchanged; when on, a manual Δ
    produces a VISIBLE pixel translation (the root of 'images didn't move')."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _view(self):
        class SL:
            xy_min_x, xy_min_y, xy_max_x, xy_max_y = 0.0, 0.0, 100000.0, 80000.0
            def check_xy_near_limit(self, *a, **k):
                return {}
        v = JogWorkspaceView()
        v.set_safety_limits(SL())
        v.resize(400, 320)
        return v

    def test_zoom_on_by_default_and_reset_on_disable(self):
        v = self._view()
        self.assertTrue(v._zoom_enabled)   # v7.5.x: on by default on every page
        v.set_zoom(5.0)
        self.assertEqual(v.zoom(), 5.0)
        v.set_zoom_enabled(False)        # disabling resets to fit
        self.assertEqual(v.zoom(), 1.0)
        self.assertEqual(v._pan, [0.0, 0.0])

    def test_disabled_view_ignores_zoom_and_pan(self):
        v = self._view()
        v.set_zoom_enabled(False)        # explicitly lock to fit-to-envelope
        a = v._um_to_px(0.0, 0.0)
        v._zoom = 9.0                    # force; should be ignored while disabled
        v._pan = [100.0, 0.0]
        b = v._um_to_px(0.0, 0.0)
        self.assertAlmostEqual(a.x(), b.x(), places=3)
        self.assertAlmostEqual(a.y(), b.y(), places=3)

    def test_zoom_magnifies_pixel_delta(self):
        import math
        v = self._view()
        v.set_zoom_enabled(True)

        def dpx(z):
            v.set_zoom(z)
            a = v._um_to_px(0.0, 0.0)
            b = v._um_to_px(1000.0, 0.0)
            return math.hypot(b.x() - a.x(), b.y() - a.y())
        d1 = dpx(1.0)
        d10 = dpx(10.0)
        self.assertGreater(d10, d1 * 5)     # ~10× → a manual nudge becomes visible

    def test_zoom_clamped(self):
        v = self._view()
        v.set_zoom_enabled(True)
        v.set_zoom(1000.0)
        self.assertLessEqual(v.zoom(), 60.0)
        v.set_zoom(0.01)
        self.assertGreaterEqual(v.zoom(), 1.0)

    def test_pan_offsets_mapping_when_enabled(self):
        v = self._view()
        v.set_zoom_enabled(True)
        a = v._um_to_px(0.0, 0.0)
        v._pan = [50.0, -30.0]
        b = v._um_to_px(0.0, 0.0)
        self.assertAlmostEqual(b.x() - a.x(), 50.0, places=3)
        self.assertAlmostEqual(b.y() - a.y(), -30.0, places=3)

    def test_manual_shift_translates_visibly_when_zoomed(self):
        # The drawn mosaic corner = _um_to_px(min + Δ); at zoom the same Δ moves
        # it many px (so sliding the deltas visibly translates the image).
        v = self._view()
        v.set_zoom_enabled(True)
        v.set_zoom(12.0)
        base = v._um_to_px(10000.0, 10000.0)
        shifted = v._um_to_px(10000.0 + 500.0, 10000.0)
        moved = abs(shifted.x() - base.x()) + abs(shifted.y() - base.y())
        self.assertGreater(moved, 3.0)

    def test_wheel_during_pan_reanchors(self):
        # A wheel tick while a left-drag pan is in progress must re-anchor the
        # pan to the just-updated _pan (else the next mouseMove snaps it back).
        # Duck-typed event (version-proof vs the QWheelEvent ctor): wheelEvent
        # only uses angleDelta().y(), position().x()/.y(), accept().
        from PySide6.QtCore import QPoint, QPointF

        class _FakeWheel:
            def __init__(self, x, y, dy):
                self._pos = QPointF(x, y)
                self._delta = QPoint(0, dy)
            def angleDelta(self):
                return self._delta
            def position(self):
                return self._pos
            def accept(self):
                pass
        v = self._view()
        v.set_zoom_enabled(True)
        v._panning = True
        v._pan_anchor_px = (10.0, 10.0)
        v._pan_anchor_val = (0.0, 0.0)
        v._pan = [0.0, 0.0]
        v.wheelEvent(_FakeWheel(200.0, 160.0, 120))
        self.assertGreater(v.zoom(), 1.0)                 # zoomed in
        self.assertEqual(v._pan_anchor_px, (200.0, 160.0))
        self.assertEqual(tuple(v._pan_anchor_val), tuple(v._pan))


class TestWorkspaceZoomControls(unittest.TestCase):
    """v7.5.x +/-/hand overlay controls + Shift-drag / hand-tool pan, on every
    page that hosts the workspace view."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _view(self):
        class SL:
            xy_min_x, xy_min_y, xy_max_x, xy_max_y = 0.0, 0.0, 100000.0, 80000.0
            def check_xy_near_limit(self, *a, **k):
                return {}
        v = JogWorkspaceView()
        v.set_safety_limits(SL())
        v.resize(400, 320)
        return v

    def test_controls_exist_and_visible_by_default(self):
        v = self._view()
        for name in ("_btn_zoom_in", "_btn_zoom_out", "_btn_pan"):
            self.assertTrue(hasattr(v, name))
            # isHidden() is the explicit flag (isVisible() needs a shown parent).
            self.assertFalse(getattr(v, name).isHidden())
        self.assertTrue(v._btn_pan.isCheckable())

    def test_controls_hidden_when_zoom_disabled(self):
        v = self._view()
        v.set_zoom_enabled(False)
        self.assertTrue(v._btn_zoom_in.isHidden())
        self.assertTrue(v._btn_pan.isHidden())

    def test_plus_minus_buttons_zoom_about_centre(self):
        v = self._view()
        z0 = v.zoom()
        v._btn_zoom_in.click()
        self.assertGreater(v.zoom(), z0)
        z1 = v.zoom()
        v._btn_zoom_out.click()
        self.assertLess(v.zoom(), z1)

    def test_zoom_out_at_min_resets_pan(self):
        v = self._view()
        v.set_zoom(1.2)                   # slightly above min
        v._pan = [40.0, 20.0]
        v._zoom_about_center(1.0 / 1.6)   # drops to min → snap back to fit
        self.assertEqual(v.zoom(), 1.0)
        self.assertEqual(v._pan, [0.0, 0.0])

    def test_hand_tool_toggles_pan_mode(self):
        v = self._view()
        self.assertFalse(v._pan_tool_active)
        v._btn_pan.setChecked(True)
        self.assertTrue(v._pan_tool_active)
        v._btn_pan.setChecked(False)
        self.assertFalse(v._pan_tool_active)

    def _press(self, v, x, y, shift=False):
        from PySide6.QtCore import QPointF, Qt

        class _Evt:
            def __init__(self, x, y, shift):
                self._p = QPointF(x, y)
                self._btn = Qt.MouseButton.LeftButton
                self._mods = (Qt.KeyboardModifier.ShiftModifier if shift
                              else Qt.KeyboardModifier.NoModifier)
            def button(self):
                return self._btn
            def position(self):
                return self._p
            def modifiers(self):
                return self._mods
        return _Evt(x, y, shift)

    def test_shift_drag_pans_plain_click_travels(self):
        clicks = []
        v = self._view()
        v.position_clicked.connect(lambda x, y: clicks.append((x, y)))
        v.set_target_mode("free")

        # Shift + press → starts a pan, no click emitted.
        v.mousePressEvent(self._press(v, 200, 160, shift=True))
        self.assertTrue(v._panning)
        self.assertEqual(clicks, [])
        v.mouseReleaseEvent(self._press(v, 210, 160, shift=True))
        self.assertFalse(v._panning)

        # Plain press (no Shift, hand tool off) → click-to-travel.
        v.mousePressEvent(self._press(v, 200, 160, shift=False))
        self.assertFalse(v._panning)
        self.assertEqual(len(clicks), 1)

    def test_hand_tool_drag_pans_without_shift(self):
        clicks = []
        v = self._view()
        v.position_clicked.connect(lambda x, y: clicks.append((x, y)))
        v._btn_pan.setChecked(True)       # hand tool on
        v.mousePressEvent(self._press(v, 200, 160, shift=False))
        self.assertTrue(v._panning)
        self.assertEqual(clicks, [])      # pan, not a travel


class TestMosaicBuilderTileRects(unittest.TestCase):
    def test_tile_rects_match_records(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0)
        b.generate_raster_positions((0, 0, 4000, 3000), overlap=0.25)
        self.assertEqual(b.tile_rects_px(), [])         # no frames yet
        f = np.full((60, 80, 3), 100, dtype=np.uint8)
        b.add_raster_frame(f, 1000.0, 800.0, 0)
        b.add_raster_frame(f, 2000.0, 800.0, 1)
        rects = b.tile_rects_px()
        self.assertEqual(len(rects), 2)
        for (x, y, w, h) in rects:
            self.assertGreater(w, 0)
            self.assertGreater(h, 0)
        # Larger stage_x → tile placed further right (px x increases).
        self.assertGreater(rects[1][0], rects[0][0])

    def test_tile_images_carry_frame_and_footprint(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0)
        b.generate_raster_positions((0, 0, 4000, 3000), overlap=0.25)
        self.assertEqual(b.tile_images_px(), [])
        f0 = np.full((60, 80, 3), 30, dtype=np.uint8)
        f1 = np.full((60, 80, 3), 200, dtype=np.uint8)
        b.add_raster_frame(f0, 1000.0, 800.0, 0)
        b.add_raster_frame(f1, 2000.0, 800.0, 1)
        imgs = b.tile_images_px()
        self.assertEqual(len(imgs), 2)
        frame, left, top, w, h = imgs[0]
        self.assertIs(frame, f0)             # carries the actual frame
        self.assertGreater(w, 0)
        self.assertGreater(h, 0)
        self.assertGreater(imgs[1][1], imgs[0][1])    # tile 1 further right


class TestMosaicRegistrationView(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _tiles(self):
        f = np.full((60, 80, 3), 90, dtype=np.uint8)
        # Two tiles side by side: centres at (40,30) and (140,30).
        return [(f, 0.0, 0.0, 80.0, 60.0), (f, 100.0, 0.0, 80.0, 60.0)]

    def test_set_tiles_and_outlines(self):
        from gui.widgets.mosaic_registration_view import MosaicRegistrationView
        v = MosaicRegistrationView()
        v.resize(400, 300)
        self.assertFalse(v.has_content())
        v.set_tiles(self._tiles(), 0.02)
        self.assertTrue(v.has_content())
        self.assertEqual(v.tile_count(), 2)
        self.assertEqual(len(v._outline_items), 2)        # one outline per tile
        v.set_image_opacity(0.4)
        v.reset_view()                                    # no crash

    def test_spacing_factor_moves_tiles_apart(self):
        from gui.widgets.mosaic_registration_view import MosaicRegistrationView
        v = MosaicRegistrationView()
        v.resize(400, 300)
        v.set_tiles(self._tiles(), 0.02)
        # At k=1 the tiles sit at their nominal centres (40 and 140 → gap 100).
        x0a = v._pix_items[0].pos().x()
        x1a = v._pix_items[1].pos().x()
        gap1 = x1a - x0a
        # Spread apart (kx=1.5): the inter-tile gap grows.
        v.set_spacing_factor(1.5, 1.0)
        gap2 = v._pix_items[1].pos().x() - v._pix_items[0].pos().x()
        self.assertGreater(gap2, gap1 + 1.0)
        self.assertEqual(v.spacing_factor(), (1.5, 1.0))
        # Pull together (kx=0.5): the gap shrinks below nominal.
        v.set_spacing_factor(0.5, 1.0)
        gap3 = v._pix_items[1].pos().x() - v._pix_items[0].pos().x()
        self.assertLess(gap3, gap1 - 1.0)

    def test_render_does_not_raise(self):
        from gui.widgets.mosaic_registration_view import MosaicRegistrationView
        v = MosaicRegistrationView()
        v.resize(400, 300)
        v.set_tiles(self._tiles(), 0.01)
        v.set_spacing_factor(1.2, 1.2)
        pm = v.grab()                       # paints the view → pixmap
        self.assertFalse(pm.isNull())


class TestCalibrationDialogManualAlign(unittest.TestCase):
    """Spacing-calibration sliders + Store INSIDE the Calibrate… pop-out — the
    operator moves tiles toward/away to align overlaps → corrected FOV (µm/px)."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dlg(self, store=None):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        return MosaicCalibrationDialog(
            MagicMock(), MagicMock(), 0, safe_z=5.0, align_key="camX|10x",
            store=store,
            settings={"cal_cols": 5, "cal_rows": 5, "overlap_pct": 25,
                      "settle_ms": 300, "fresh_frames": 3, "fov_um": 0,
                      "max_shift_um": 0, "frame_orient": "none"},
            center_um=(50000.0, 40000.0), frame_size=(80, 60),
            um_per_px_camera=10.0)

    def test_sliders_exist_and_drive_spacing(self):
        dlg = self._dlg()
        self.assertTrue(hasattr(dlg, "_align_dx"))
        self.assertIsNotNone(dlg._view)
        self.assertFalse(dlg._align_dx.isEnabled())       # until a mosaic exists
        dlg._set_align_enabled(True)
        dlg._align_dx.setValue(30)        # +30% X gap → kx=1.3
        dlg._align_dy.setValue(-20)       # -20% Y gap → ky=0.8
        kx, ky = dlg._view.spacing_factor()
        self.assertAlmostEqual(kx, 1.3, places=6)
        self.assertAlmostEqual(ky, 0.8, places=6)

    def test_store_corrected_um_per_px_and_fov(self):
        # Spreading the tiles (k>1) to align overlaps → assumed FOV was too big →
        # corrected µm/px = assumed / k, stored as um_per_px.
        #
        # v7.5.x: the corrected value is NO LONGER also written into a ``fov_um``
        # spin. That field's only purpose was to carry the number into
        # ``mosaic_scan`` as a standing override of the camera calibration; the
        # correction now propagates via _propagate_um_per_px (live manager +
        # objective store), which is what the mosaics and click mapping read.
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        dlg = self._dlg(store=store)
        dlg._result_comp = np.full((40, 50, 3), 120, dtype=np.uint8)
        dlg._set_align_enabled(True)
        dlg._align_dx.setValue(25)        # +25% → kx=1.25
        dlg._align_dy.setValue(25)        # +25% → ky=1.25 → k=1.25
        dlg._store_manual_align()
        # corrected = 10.0 / 1.25 = 8.0 µm/px.
        self.assertAlmostEqual(store.get_um_per_px("camX|10x"), 8.0, places=3)
        self.assertFalse(hasattr(dlg, "_spin_fov"))
        # Idempotent.
        dlg._store_manual_align()
        self.assertAlmostEqual(store.get_um_per_px("camX|10x"), 8.0, places=3)

    def test_seed_sliders_from_stored_um_per_px(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        store.set_um_per_px("camX|10x", 8.0, source="quick_fov")
        dlg = self._dlg(store=store)
        dlg._seed_sliders_from_store()
        # assumed 10 / stored 8 = k 1.25 → +25 %.
        self.assertEqual(dlg._align_dx.value(), 25)
        self.assertEqual(dlg._align_dy.value(), 25)
        kx, ky = dlg._view.spacing_factor()
        self.assertAlmostEqual(kx, 1.25, places=6)

    def test_store_without_mosaic_is_guarded(self):
        dlg = self._dlg()
        dlg._result_comp = None
        dlg._set_align_enabled(True)
        dlg._store_manual_align()         # must not raise
        self.assertIn("first", dlg._status.text().lower())

    def test_height_capped_to_finite_value(self):
        # The dialog caps its height so the pinned Build/Close row can't overflow.
        dlg = self._dlg()
        self.assertLess(dlg.maximumHeight(), 16_777_215)   # below Qt's default max
        self.assertGreater(dlg.maximumHeight(), 0)
        dlg._apply_height_cap(initial=False)               # showEvent path
        self.assertLess(dlg.maximumHeight(), 16_777_215)

    def test_clear_stored_removes_shift(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        store.set_shift_um("camX|10x", 5.0, 5.0, source="manual_align")
        dlg = self._dlg(store=store)
        dlg._clear_stored_align()
        self.assertIsNone(store.get_shift_um("camX|10x"))

    def test_auto_finish_keeps_manual_align(self):
        from SupportClasses.MosaicAlignmentStore import MosaicAlignmentStore
        from SupportClasses.MosaicBuilder import MosaicBuilder
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = MosaicAlignmentStore(Path(tmp.name) / "ma.json")
        store.set_shift_um("camX|10x", 6.0, -6.0, source="manual_align")
        dlg = self._dlg(store=store)
        b = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                          register=True, initial_shift_um=(6.0, -6.0))
        b.generate_raster_positions((0, 0, 2000, 1500), overlap=0.25)
        b._measured_shifts = [(1.0, 1.0)]      # auto registered
        b._global_shift_um = (40.0, 40.0)
        dlg._builder = b
        dlg._worker = object()
        comp = np.full((50, 60, 3), 120, dtype=np.uint8)
        dlg._on_finished(comp, b.canvas_extent_um, b._mosaic_scale, 25, [])
        # Manual value preserved despite a successful auto registration.
        self.assertEqual(store.get_shift_um("camX|10x"), (6.0, -6.0))
        self.assertTrue(store.is_manual("camX|10x"))


# ── Manual well mapping on the mosaic + training-data export ───────

class TestWellTrainingStore(unittest.TestCase):
    def test_save_and_list_roundtrip(self):
        import json
        from SupportClasses.WellTrainingStore import WellTrainingStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        st = WellTrainingStore(Path(tmp.name))
        img = np.full((40, 60, 3), 120, dtype=np.uint8)
        wells = {"A1": {"px": (10, 10), "um": (100, 100), "r_px": 5.0},
                 "A2": {"px": (30, 10), "um": (300, 100), "r_px": 5.0}}
        path = st.save_sample("6", img, wells, extent_um=(0, 0, 600, 400),
                              mosaic_scale=0.1, um_per_px=10.0,
                              timestamp="20260617_000000")
        self.assertIsNotNone(path)
        p = Path(path)
        self.assertTrue((p / "mosaic.png").exists())
        meta = json.loads((p / "labels.json").read_text(encoding="utf-8"))
        self.assertEqual(meta["plate_key"], "6")
        self.assertEqual(len(meta["wells"]), 2)
        self.assertEqual(meta["wells"]["A1"]["px"], [10.0, 10.0])
        self.assertEqual(meta["wells"]["A1"]["um"], [100.0, 100.0])
        self.assertEqual(meta["image_size_px"], [60, 40])
        self.assertEqual(st.list_samples(), [str(p)])

    def test_no_wells_returns_none(self):
        from SupportClasses.WellTrainingStore import WellTrainingStore
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        st = WellTrainingStore(Path(tmp.name))
        self.assertIsNone(
            st.save_sample("6", np.zeros((10, 10, 3), dtype=np.uint8), {}))


class TestWellMappingMath(unittest.TestCase):
    def test_affine_recovers_linear_map(self):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            solve_affine_3, apply_affine)
        nom = [(0, 0), (10, 0), (0, 8)]
        tgt = [(50, 50), (100, 50), (50, 90)]      # px = nom*5 + (50,50)
        aff = solve_affine_3(nom, tgt)
        self.assertIsNotNone(aff)
        x, y = apply_affine(aff, 4, 4)
        self.assertAlmostEqual(x, 70.0, places=3)
        self.assertAlmostEqual(y, 70.0, places=3)

    def test_collinear_returns_none(self):
        from gui.dialogs.mosaic_well_mapping_dialog import solve_affine_3
        self.assertIsNone(
            solve_affine_3([(0, 0), (1, 1), (2, 2)],
                           [(0, 0), (1, 1), (2, 2)]))

    def test_corner_wells_distinct(self):
        from gui.dialogs.mosaic_well_mapping_dialog import corner_well_names
        c = corner_well_names(WellPlate.from_format(96))
        self.assertEqual(len(c), 3)
        self.assertEqual(len(set(c)), 3)


class TestWellMappingDialog(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_corners_autofill_and_results(self):
        from PySide6.QtCore import QPointF
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog, corner_well_names)
        import SupportClasses.WellTrainingStore as wts
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        wts._store_singleton = wts.WellTrainingStore(Path(tmp.name))
        self.addCleanup(lambda: setattr(wts, "_store_singleton", None))

        plate = WellPlate.from_format(6)
        img = np.full((300, 400, 3), 100, dtype=np.uint8)
        # extent origin 0, scale 0.1 px/µm → px → µm = px / 0.1.
        dlg = MosaicWellMappingDialog(
            plate, img, (0.0, 0.0, 4000.0, 3000.0), 0.1,
            um_per_px=10.0, plate_key="6")
        corners = corner_well_names(plate)
        for w in corners:                          # click each corner: px=nom*5+60
            nx, ny = plate.get_well_position(w)
            dlg._on_view_clicked(QPointF(nx * 5 + 60, ny * 5 + 60))
            dlg._on_confirm_well()                 # v7.5.x guided per-well confirm
        # All wells auto-placed from the 3 confirmed corners.
        self.assertEqual(len(dlg._well_items), len(plate.well_names))
        dlg._on_confirm()
        res = dlg.results()
        self.assertEqual(len(res), len(plate.well_names))
        # A corner well maps back to clicked-px / scale (µm).
        w0 = corners[0]
        nx, ny = plate.get_well_position(w0)
        self.assertAlmostEqual(res[w0][0], (nx * 5 + 60) / 0.1, delta=30)
        self.assertAlmostEqual(res[w0][1], (ny * 5 + 60) / 0.1, delta=30)
        # Training sample written.
        self.assertIsNotNone(dlg.saved_sample_path())
        self.assertTrue(
            (Path(dlg.saved_sample_path()) / "labels.json").exists())


class TestWellMappingPageGuard(_CalBase):
    def test_open_guarded_without_mosaic(self):
        import gui.pages.calibration as calmod
        import SupportClasses.MosaicStore as msmod
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        msmod._store_singleton = msmod.MosaicStore(
            Path(tmp.name) / "pm.json")            # empty store
        self.addCleanup(lambda: setattr(msmod, "_store_singleton", None))
        page = self._make_page()
        page._plate = WellPlate.from_format(6)
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
            page._ploc_open_well_mapping()          # no mosaic → informs, no crash
        finally:
            calmod.QMessageBox = orig
        self.assertTrue(shown)


# ── Robust filled-disc well detector (the "best well finder") ──────

def _synthetic_plate(rows, cols, r=40, pitch=120, margin=80,
                     color=(255, 0, 0), clip_left=False):
    """Filled-circle plate image (BGR). color default = blue (like 24.png)."""
    import cv2
    h = margin * 2 + (rows - 1) * pitch
    w = margin * 2 + (cols - 1) * pitch
    img = np.zeros((h, w, 3), dtype=np.uint8)
    off = -r // 2 if clip_left else 0          # push col 0 partly off-frame
    for i in range(rows):
        for j in range(cols):
            cx = margin + j * pitch + (off if j == 0 else 0)
            cy = margin + i * pitch
            cv2.circle(img, (cx, cy), r, color, -1)
    return img


class TestFilledWellDetector(unittest.TestCase):
    def test_detects_all_filled_circles(self):
        from SupportClasses.VisionDetector import WellDetector
        img = _synthetic_plate(4, 6)
        dets = WellDetector.detect_filled_wells(img)
        self.assertEqual(len(dets), 24)
        for d in dets:
            self.assertAlmostEqual(d.radius_px, 40, delta=6)

    def test_color_agnostic_green_wells(self):
        from SupportClasses.VisionDetector import WellDetector
        img = _synthetic_plate(3, 4, color=(0, 200, 0))   # green wells
        self.assertEqual(len(WellDetector.detect_filled_wells(img)), 12)

    def test_clipped_well_radius_recovered_not_undersized(self):
        # A well cut by a straight chord (clipped at a mosaic margin) must keep
        # its TRUE radius — the fitted circle hugs the visible arc and may run
        # off-image. (Naive whole-contour fit undersizes; the robust fit doesn't.)
        import cv2
        from SupportClasses.VisionDetector import WellDetector
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        cv2.circle(img, (120, 200), 120, (255, 0, 0), -1)   # true r=120
        img[:, :40] = 0          # straight chord at x=40 → left ~17% missing
        dets = WellDetector.detect_filled_wells(img)
        self.assertEqual(len(dets), 1)
        d = dets[0]
        self.assertAlmostEqual(d.radius_px, 120, delta=10)     # not undersized
        self.assertAlmostEqual(d.center_px[0], 120, delta=15)  # centre recovered

    def test_grid_fit_assigns_all_nodes(self):
        from SupportClasses.VisionDetector import WellDetector
        img = _synthetic_plate(4, 6)
        c = [(d.center_px[0], d.center_px[1])
             for d in WellDetector.detect_filled_wells(img)]
        aff, assign = WellDetector.fit_well_grid(c, 4, 6)
        self.assertIsNotNone(aff)
        self.assertEqual(len(assign), 24)

    def test_grid_fills_missing_and_rejects_extra(self):
        from SupportClasses.VisionDetector import WellDetector
        img = _synthetic_plate(4, 6)
        dets = WellDetector.detect_filled_wells(img)
        c = [(d.center_px[0], d.center_px[1]) for d in dets]
        # Drop one (missing) + add a far-off spurious blob (extra).
        c2 = c[:10] + c[11:] + [(5.0, 5.0)]
        aff, assign = WellDetector.fit_well_grid(c2, 4, 6)
        self.assertIsNotNone(aff)
        # 23 real wells map to distinct nodes; the spurious point doesn't
        # displace them all — at most one node is wrong.
        self.assertGreaterEqual(len(assign), 22)

    def test_real_24_well_mosaic(self):
        import os
        import cv2
        p = "config/hardware/mosaics/24.png"
        if not os.path.exists(p):
            self.skipTest("real 24.png mosaic not present")
        from SupportClasses.VisionDetector import WellDetector
        img = cv2.imread(p)
        dets = WellDetector.detect_filled_wells(img)
        self.assertGreaterEqual(len(dets), 24)
        c = [(d.center_px[0], d.center_px[1]) for d in dets]
        aff, assign = WellDetector.fit_well_grid(c, 4, 6)
        self.assertEqual(len(assign), 24)        # all 24 wells resolved


class TestWellMappingAutoDetect(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_auto_detect_on_open_places_all_wells(self):
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        import SupportClasses.WellTrainingStore as wts
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        wts._store_singleton = wts.WellTrainingStore(Path(tmp.name))
        self.addCleanup(lambda: setattr(wts, "_store_singleton", None))
        plate = WellPlate.from_format(24)           # 4×6, Ø15.6 mm @ 19.3 mm
        # v7.5.x: the auto-detector is MODEL-DRIVEN — it looks for wells of
        # the size and spacing the plate definition states, at the mosaic's
        # own scale. So the synthetic image has to be geometrically consistent
        # with the plate (it previously was not: 40 px wells 120 px apart under
        # a scale that made the plate 390 px wells 965 px apart).
        scale = 0.006                                # px per µm
        pitch_px = int(round(19300.0 * scale))       # 116
        r_px = int(round(15600.0 / 2.0 * scale))     # 47
        img = _synthetic_plate(4, 6, r=r_px, pitch=pitch_px, margin=90)
        h, w = img.shape[:2]
        dlg = MosaicWellMappingDialog(
            plate, img, (0.0, 0.0, w / scale, h / scale), scale,
            um_per_px=1.0 / scale, plate_key="24")
        # Auto-detect ran on open → all 24 wells already placed.
        self.assertEqual(len(dlg._well_items), 24)
        dlg._on_confirm()
        self.assertEqual(len(dlg.results()), 24)


if __name__ == "__main__":
    unittest.main()
