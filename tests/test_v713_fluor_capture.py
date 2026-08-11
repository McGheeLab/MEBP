"""
test_v713_fluor_capture.py — fluorescence mosaic capture upgrades (v7.13).

Drives the REAL _SingleWellMosaicWorker synchronously (same-thread run(), the
existing suite's pattern) to pin:
- the probe visit (well centre) gets the retract-gated safe_travel_to and the
  raster's tile 0 then uses a plain move;
- averaged raw tiles convert through ONE frozen per-channel level pair, and
  the meta records levels + avg count;
- a camera without raw support falls back to today's single display frames
  (meta says avg 1, levels None);
- the microscope LEASE is acquired/released in pairs on success, abort and
  refusal, and the entry focus is restored (release-not-in-finally is the
  mutation this exists to catch);
- end-to-end autofocus over a synthetic tilted sample surface: checkerboard
  sweeps recover the plane, later channels replay it;
- per-channel exposure apply/record/restore page helpers;
- store round-trip of the new channel metadata + processed-image preference.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.pages.workflows.fluorescence_mosaic_workflow import (
    _SingleWellMosaicWorker, FluorescenceMosaicWorkflowPage)
from SupportClasses.MosaicBuilder import MosaicBuilder
from SupportClasses.TileAutofocus import TileAutofocus
from SupportClasses.MosaicFocusTracker import MosaicFocusTracker, PlanePredictor

from tests.test_v713_tile_autofocus import FakeScope, FakeFocusCam


def _img(h, w, val=40):
    return np.full((h, w, 3), val, dtype=np.uint8)


class _Cam:
    """Display-frame-only camera (no raw support)."""

    def __init__(self):
        self._n = 0

    def frame_count_value(self):
        self._n += 1
        return self._n

    def get_current_frame(self):
        return _img(64, 64)


class _AvgCam(_Cam):
    """Raw-capable camera: capture_raw_average returns a uint16 ramp."""

    def __init__(self):
        super().__init__()
        self.avg_calls = []

    def capture_raw_average(self, n, timeout_s=10.0):
        self.avg_calls.append((int(n), float(timeout_s)))
        ramp = np.linspace(200, 3000, 64 * 64).reshape(64, 64)
        return ramp.astype(np.uint16)


class _Ctrl:
    zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    def __init__(self):
        self.safe_calls = []
        self.moves = []

    def suspend_position_poller(self):
        pass

    def resume_position_poller(self):
        pass

    def safe_travel_to(self, x, y, **k):
        self.safe_calls.append((x, y))

    def move_xy_absolute_um(self, x, y, **k):
        self.moves.append((x, y))

    def wait_for_xy_arrival(self, *a, **k):
        pass

    def get_xy_position(self, cached=False):
        return (self.moves[-1] if self.moves
                else (self.safe_calls[-1] if self.safe_calls else (0.0, 0.0)))


class _LeaseScope(FakeScope):
    def __init__(self, allow=True, **kw):
        super().__init__(**kw)
        self.allow = allow
        self.acquired = 0
        self.released = 0

    def try_acquire(self, owner, timeout=0.0):
        if self.allow:
            self.acquired += 1
        return self.allow

    def release(self, owner):
        self.released += 1

    def lease_owner(self):
        return "someone else"


def _builder(frame=(64, 64), bounds=(0.0, 0.0, 300.0, 300.0)):
    b = MosaicBuilder(frame_size_px=frame, micron_per_pixel=2.0,
                      overlap=0.25, target_mosaic_px=400, register=False)
    positions = b.generate_raster_positions(bounds, overlap=0.25)
    return b, positions


def _run(worker):
    got = {}

    def on_done(comp, ext, scale, frames, shift, meta):
        got.update(comp=comp, ext=ext, shift=shift, meta=meta)

    fails = []
    worker.finished_ok.connect(on_done)
    worker.failed.connect(fails.append)
    worker.run()
    return got, fails


class TestAveragedCapture(unittest.TestCase):
    def test_probe_gets_safe_travel_tiles_use_plain_moves(self):
        builder, positions = _builder()
        ctrl = _Ctrl()
        worker = _SingleWellMosaicWorker(
            ctrl, _AvgCam(), builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            avg_frames=4, probe_xy=(150.0, 150.0))
        got, fails = _run(worker)
        self.assertFalse(fails)
        # ONE retract-gated travel — the probe; every tile a plain move.
        self.assertEqual(ctrl.safe_calls, [(150.0, 150.0)])
        self.assertEqual(len(ctrl.moves), len(positions))

    def test_meta_records_frozen_levels_and_avg(self):
        builder, positions = _builder()
        cam = _AvgCam()
        worker = _SingleWellMosaicWorker(
            _Ctrl(), cam, builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            avg_frames=4, probe_xy=(150.0, 150.0), exposure_us=25000.0)
        got, fails = _run(worker)
        self.assertFalse(fails)
        meta = got["meta"]
        self.assertEqual(meta["avg_frames"], 4)
        self.assertEqual(meta["exposure_us"], 25000.0)
        lo, hi = meta["display_levels"]
        self.assertLess(lo, hi)
        self.assertGreaterEqual(lo, 0.0)
        # probe + one per tile
        self.assertEqual(len(cam.avg_calls), 1 + len(positions))
        self.assertTrue(all(n == 4 for n, _t in cam.avg_calls))
        self.assertIsNotNone(got["comp"])

    def test_levels_frozen_once_for_all_tiles(self):
        # The probe freezes (lo, hi); no tile may re-freeze them (that is the
        # per-tile autoscale bug this feature removes).
        builder, positions = _builder()
        worker = _SingleWellMosaicWorker(
            _Ctrl(), _AvgCam(), builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            avg_frames=2, probe_xy=(150.0, 150.0))
        frozen = {}
        orig = worker._freeze_levels

        def counting(raw):
            out = orig(raw)
            frozen.setdefault("n", 0)
            frozen["n"] += 1
            return out

        worker._freeze_levels = counting
        _run(worker)
        self.assertEqual(frozen["n"], 1)

    def test_no_raw_support_falls_back_single_frame(self):
        builder, positions = _builder()
        worker = _SingleWellMosaicWorker(
            _Ctrl(), _Cam(), builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            avg_frames=8, probe_xy=(150.0, 150.0))
        got, fails = _run(worker)
        self.assertFalse(fails)
        meta = got["meta"]
        self.assertEqual(meta["avg_frames"], 1)
        self.assertIsNone(meta["display_levels"])
        self.assertIsNotNone(got["comp"])

    def test_avg_one_never_calls_raw(self):
        builder, positions = _builder()
        cam = _AvgCam()
        worker = _SingleWellMosaicWorker(
            _Ctrl(), cam, builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            avg_frames=1)
        got, fails = _run(worker)
        self.assertFalse(fails)
        self.assertEqual(cam.avg_calls, [])
        self.assertEqual(got["meta"]["avg_frames"], 1)


class TestLeaseAndRestore(unittest.TestCase):
    def _af_worker(self, scope, stop=False, allow=True):
        builder, positions = _builder()
        cam = FakeFocusCam(scope, true_z=1000.0)
        af = TileAutofocus(scope=scope, cam=cam, dof_um=10.0, settle_s=0.0)
        worker = _SingleWellMosaicWorker(
            _Ctrl(), cam, builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            scope=scope, autofocus=af)
        if stop:
            worker._stop = True
        return worker

    def test_lease_paired_on_success(self):
        scope = _LeaseScope(focus=1000.0)
        worker = self._af_worker(scope)
        _got, fails = _run(worker)
        self.assertFalse(fails)
        self.assertEqual(scope.acquired, 1)
        self.assertEqual(scope.released, 1)     # mutation: release not in finally
        self.assertAlmostEqual(scope.focus, 1000.0)   # entry focus restored

    def test_lease_released_on_abort(self):
        scope = _LeaseScope(focus=1000.0)
        worker = self._af_worker(scope, stop=True)
        _run(worker)
        self.assertEqual(scope.acquired, 1)
        self.assertEqual(scope.released, 1)

    def test_refused_lease_fails_loudly_without_release(self):
        scope = _LeaseScope(allow=False)
        worker = self._af_worker(scope, allow=False)
        _got, fails = _run(worker)
        self.assertTrue(fails)
        self.assertIn("driving the microscope", fails[0])
        self.assertEqual(scope.released, 0)


class TestAutofocusEndToEnd(unittest.TestCase):
    A, B, C = 0.05, 0.02, 1000.0    # synthetic sample surface (µm focus)

    def _true_z(self, x, y):
        return self.C + self.A * x + self.B * y

    def _wire(self, scope, cam):
        ctrl = _Ctrl()
        outer_move = ctrl.move_xy_absolute_um
        outer_safe = ctrl.safe_travel_to

        def move(x, y, **k):
            cam.true_z = self._true_z(x, y)
            outer_move(x, y, **k)

        def safe(x, y, **k):
            cam.true_z = self._true_z(x, y)
            outer_safe(x, y, **k)

        ctrl.move_xy_absolute_um = move
        ctrl.safe_travel_to = safe
        return ctrl

    def _optics(self):
        from SupportClasses.ObjectiveOptics import ObjectiveOptics
        return ObjectiveOptics(label="10x", numerical_aperture=0.30,
                               working_distance_mm=16.0,
                               um_per_px_sample=1.0, frame_wh=(64, 64))

    def test_first_channel_measures_the_plane(self):
        scope = _LeaseScope(focus=1005.0)
        cam = FakeFocusCam(scope, true_z=self._true_z(150, 150), sigma_um=12.0)
        ctrl = self._wire(scope, cam)
        builder, positions = _builder()
        af = TileAutofocus(scope=scope, cam=cam, dof_um=10.0, settle_s=0.0)
        tracker = MosaicFocusTracker(
            well_center_um=(150.0, 150.0), well_radius_um=5000.0,
            lattice_spacing=1, dof_um=10.0, positions=positions)
        worker = _SingleWellMosaicWorker(
            ctrl, cam, builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            scope=scope, autofocus=af, tracker=tracker,
            optics=self._optics(), probe_xy=(150.0, 150.0))
        got, fails = _run(worker)
        self.assertFalse(fails)
        meta = got["meta"]
        self.assertIn("focus_map", meta)
        self.assertGreaterEqual(len(meta["focus_map"]), len(positions) // 2)
        plane = meta["af"]["plane"]
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane[0], self.A, delta=0.02)
        self.assertAlmostEqual(plane[1], self.B, delta=0.02)
        # Entry focus restored + lease paired.
        self.assertAlmostEqual(scope.focus, 1005.0)
        self.assertEqual(scope.acquired, scope.released)
        self._map = meta["focus_map"]

    def test_second_channel_replays_without_sweeps(self):
        # Replay: one focus goto per tile driven by the predictor, no sweeps.
        samples = [{"x_um": x, "y_um": y,
                    "focus_um": self._true_z(x, y)}
                   for (x, y) in [(0, 0), (300, 0), (0, 300), (300, 300)]]
        scope = _LeaseScope(focus=1005.0)
        cam = FakeFocusCam(scope, true_z=1000.0)
        ctrl = self._wire(scope, cam)
        builder, positions = _builder()
        af = TileAutofocus(scope=scope, cam=cam, dof_um=10.0, settle_s=0.0)
        worker = _SingleWellMosaicWorker(
            ctrl, cam, builder, positions, safe_z=5.0,
            fresh_frames=1, fresh_timeout_s=0.2, settle_ms=0,
            scope=scope, autofocus=af,
            focus_predictor=PlanePredictor(samples))
        got, fails = _run(worker)
        self.assertFalse(fails)
        # Replay mode records no map of its own.
        self.assertNotIn("focus_map", got["meta"])
        # Each tile: lead-in + target = 2 moves; plus the entry restore.
        self.assertEqual(len(scope.moves), 2 * len(positions) + 1)
        # The last per-tile target matches the plane at the last tile.
        lx, ly = positions[-1]
        self.assertAlmostEqual(scope.moves[-2], self._true_z(lx, ly),
                               places=3)


class TestPageExposureHelpers(unittest.TestCase):
    def _fake_page(self):
        calls = []
        mgr = SimpleNamespace(
            set_hw_exposure_us=lambda idx, us: calls.append((idx, us)))
        page = SimpleNamespace(
            _camera_manager=mgr,
            _resolve_microscope_cam_idx=lambda: 0,
            _entry_exposure_us=None,
            # v7.14 — the exposure restore now also restores the capture
            # RESOLUTION: both are run-scoped camera state snapshotted at
            # start, so pairing them means every exit path that restores one
            # restores the other. Modelled here so this stub matches the real
            # page (its own coverage lives in test_v714_full_res_mosaic).
            _full_res_prev=None, _full_res_canvas=None,
            _restore_full_res=lambda: None)
        return page, calls

    def test_apply_channel_exposure_ms_to_us(self):
        page, calls = self._fake_page()
        FluorescenceMosaicWorkflowPage._apply_channel_exposure(page, 12.5)
        self.assertEqual(calls, [(0, 12500)])   # mutation: ×1000 vs ÷1000

    def test_zero_means_do_not_touch(self):
        page, calls = self._fake_page()
        FluorescenceMosaicWorkflowPage._apply_channel_exposure(page, 0.0)
        self.assertEqual(calls, [])

    def test_restore_entry_exposure_once(self):
        page, calls = self._fake_page()
        page._entry_exposure_us = 30000.0
        FluorescenceMosaicWorkflowPage._restore_entry_exposure(page)
        self.assertEqual(calls, [(0, 30000)])
        self.assertIsNone(page._entry_exposure_us)
        FluorescenceMosaicWorkflowPage._restore_entry_exposure(page)
        self.assertEqual(len(calls), 1)         # second call is a no-op


class TestStoreMetadataAndProcessed(unittest.TestCase):
    def setUp(self):
        import SupportClasses.FluorescenceMosaicStore as fms
        self._fms = fms
        self._orig = fms._store_singleton
        tmp = Path(tempfile.mkdtemp()) / "fluor.json"
        fms._store_singleton = fms.FluorescenceMosaicStore(tmp)

    def tearDown(self):
        self._fms._store_singleton = self._orig

    def _save(self, **kw):
        st = self._fms.get_store()
        img = np.full((40, 40, 3), 50, dtype=np.uint8)
        ok = st.save_channel("p", "A1", "DAPI", img,
                             (0.0, 0.0, 100.0, 100.0),
                             um_per_px=2.0, mosaic_scale=0.5, **kw)
        self.assertTrue(ok)
        return st

    def test_new_metadata_round_trip(self):
        st = self._save(exposure_us=42000.0, display_levels=(120.0, 8000.0),
                        avg_frames=8)
        self.assertEqual(st.get_display_levels("p", "A1", "DAPI"),
                         (120.0, 8000.0))
        self.assertEqual(st.get_avg_frames("p", "A1", "DAPI"), 8)
        self.assertEqual(st.get_exposure_us("p", "A1", "DAPI"), 42000.0)

    def test_legacy_entry_defaults(self):
        st = self._save()
        self.assertIsNone(st.get_display_levels("p", "A1", "DAPI"))
        self.assertEqual(st.get_avg_frames("p", "A1", "DAPI"), 1)
        self.assertIsNone(st.get_processing("p", "A1", "DAPI"))

    def test_processed_preferred_by_overlay_raw_by_default(self):
        st = self._save()
        proc = np.full((40, 40, 3), 200, dtype=np.uint8)
        self.assertTrue(st.attach_processed("p", "A1", "DAPI", proc,
                                            {"denoise": "median"}))
        raw = st.load_channel_image("p", "A1", "DAPI")
        self.assertEqual(int(raw[0, 0, 0]), 50)          # detection input
        pref = st.load_channel_image("p", "A1", "DAPI", prefer_processed=True)
        self.assertEqual(int(pref[0, 0, 0]), 200)        # overlay input
        overlay, _ext = st.composite_overlay("p", "A1")
        self.assertGreater(int(overlay.max()), 150)
        self.assertEqual(st.get_processing("p", "A1", "DAPI"),
                         {"denoise": "median"})

    def test_fresh_save_drops_stale_processed(self):
        st = self._save()
        st.attach_processed("p", "A1", "DAPI",
                            np.full((40, 40, 3), 200, dtype=np.uint8),
                            {"denoise": "median"})
        self._save()                                     # re-scan the channel
        self.assertIsNone(st.get_processing("p", "A1", "DAPI"))
        pref = st.load_channel_image("p", "A1", "DAPI", prefer_processed=True)
        self.assertEqual(int(pref[0, 0, 0]), 50)         # raw again


if __name__ == "__main__":
    unittest.main()
