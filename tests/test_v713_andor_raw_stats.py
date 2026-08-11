"""
test_v713_andor_raw_stats.py — raw 16-bit frame statistics (v7.13).

The display auto-scale hides clipping, so exposure decisions need statistics
taken from RAW counts before the mono→BGR8 conversion. Covers:
- compute_raw_frame_stats (exact clipped fractions at 12-bit and 16-bit clip
  levels — mutation guard: `>=` vs `>` at the clip level; histogram integrity;
  decimation policy; degenerate inputs),
- the REAL AndorBackend reader loop retaining stats AND still producing the
  display frame (stub camera, no SDK),
- widget/manager passthrough, the histogram widget's offscreen painting, and
  the live-feed SATURATED badge threshold behaviour.
"""

import os
import sys
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.mono_display import compute_raw_frame_stats, mono_to_bgr8
from gui.widgets.andor_backend import AndorBackend
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager
from gui.widgets.camera_feed_view import (
    CameraFeedView, SATURATION_WARN_FRAC)
from gui.widgets.raw_histogram_widget import RawHistogramWidget

from tests.test_v75x_andor_zyla_camera import _andor_widget
from tests.test_v713_andor_sensor_features import _StubCam, _full_features


# ── compute_raw_frame_stats (pure) ────────────────────────────────────

class TestComputeRawFrameStats(unittest.TestCase):
    def test_exact_clipped_fraction_16bit(self):
        # 64x64 → stride 1 → every pixel sampled: the fraction is EXACT.
        f = np.full((64, 64), 1000, dtype=np.uint16)
        f[0, :8] = 65535
        st = compute_raw_frame_stats(f, 65535)
        self.assertEqual(st["clipped_frac"], 8 / 4096)
        self.assertEqual(st["clip_level"], 65535)

    def test_exact_clipped_fraction_12bit(self):
        # Values AT the clip level count as clipped (mutation guard: >= → >
        # makes this zero and fails).
        f = np.full((64, 64), 200, dtype=np.uint16)
        f[0, :4] = 4095      # exactly at clip
        f[1, :4] = 4200      # above clip
        st = compute_raw_frame_stats(f, 4095)
        self.assertEqual(st["clipped_frac"], 8 / 4096)
        self.assertEqual(st["clip_level"], 4095)

    def test_histogram_integrity(self):
        f = np.random.randint(0, 4096, size=(64, 64)).astype(np.uint16)
        st = compute_raw_frame_stats(f, 4095, bins=256)
        self.assertEqual(int(np.sum(st["hist"])), st["sample_count"])
        self.assertEqual(st["hist_range"], (0, 4096))
        self.assertEqual(len(st["hist"]), 256)

    def test_decimation_matches_auto_levels_policy(self):
        # 1024 px long axis → stride 2 → 512x512 samples.
        f = np.zeros((1024, 1024), dtype=np.uint16)
        st = compute_raw_frame_stats(f, 65535)
        self.assertEqual(st["sample_count"], 512 * 512)

    def test_min_max_mean(self):
        f = np.full((32, 32), 100, dtype=np.uint16)
        f[0, 0] = 5000
        st = compute_raw_frame_stats(f, 65535)
        self.assertEqual(st["min"], 100.0)
        self.assertEqual(st["max"], 5000.0)
        self.assertGreater(st["mean"], 100.0)

    def test_degenerate_inputs(self):
        self.assertIsNone(compute_raw_frame_stats(None, 65535))
        self.assertIsNone(compute_raw_frame_stats(np.zeros(4), 65535))
        self.assertIsNone(
            compute_raw_frame_stats(np.zeros((0, 4), dtype=np.uint16), 65535))

    def test_3d_frame_reduced_to_first_plane(self):
        f = np.zeros((16, 16, 3), dtype=np.uint16)
        f[..., 0] = 4095
        st = compute_raw_frame_stats(f, 4095)
        self.assertEqual(st["clipped_frac"], 1.0)


# ── Reader loop retention (real backend, stub cam) ────────────────────

class TestReaderLoopStats(unittest.TestCase):
    def _run_loop_with_frames(self, be, stub, frames, settle_s=1.5):
        for f in frames:
            stub.stage_frame(f)
        be._running = True
        t = threading.Thread(target=be._reader_loop, daemon=True)
        t.start()
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            if be.get_raw_frame_stats() is not None and be._frame is not None:
                break
            time.sleep(0.01)
        be._running = False
        t.join(timeout=2.0)

    def test_stats_and_display_frame_both_produced(self):
        feats = _full_features()
        feats["SensorTemperature"] = {"values": None, "value": -0.2}
        feats["TemperatureStatus"] = {"values": ["Cooling", "Stabilised"],
                                      "value": "Stabilised"}
        stub = _StubCam(features=feats)
        be = AndorBackend()
        be._cam = stub
        be._probe_sensor_features()
        be._clip_level = 65535
        frame = np.full((64, 64), 500, dtype=np.uint16)
        frame[0, :8] = 65535
        self._run_loop_with_frames(be, stub, [frame])
        st = be.get_raw_frame_stats()
        self.assertIsNotNone(st)      # mutation guard: stats computation gone
        self.assertAlmostEqual(st["clipped_frac"], 8 / 4096)
        self.assertEqual(st["temperature_c"], -0.2)
        self.assertEqual(st["temperature_status"], "Stabilised")
        with be._lock:
            self.assertIsNotNone(be._frame)   # display path still alive
            self.assertEqual(be._frame.shape, (64, 64, 3))

    def test_stats_respect_current_clip_level(self):
        stub = _StubCam(features=_full_features())
        be = AndorBackend()
        be._cam = stub
        be._probe_sensor_features()
        be._clip_level = 4095
        frame = np.full((64, 64), 100, dtype=np.uint16)
        frame[0, :4] = 4095
        self._run_loop_with_frames(be, stub, [frame])
        st = be.get_raw_frame_stats()
        self.assertEqual(st["clip_level"], 4095)
        self.assertAlmostEqual(st["clipped_frac"], 4 / 4096)

    def test_release_clears_stats(self):
        stub = _StubCam(features=_full_features())
        be = AndorBackend()
        be._cam = stub
        be._probe_sensor_features()
        frame = np.full((32, 32), 100, dtype=np.uint16)
        self._run_loop_with_frames(be, stub, [frame])
        self.assertIsNotNone(be.get_raw_frame_stats())
        be.release()
        self.assertIsNone(be.get_raw_frame_stats())

    def test_snapshot_hist_is_a_copy(self):
        stub = _StubCam(features=_full_features())
        be = AndorBackend()
        be._cam = stub
        be._probe_sensor_features()
        frame = np.full((32, 32), 100, dtype=np.uint16)
        self._run_loop_with_frames(be, stub, [frame])
        a = be.get_raw_frame_stats()
        b = be.get_raw_frame_stats()
        a["hist"][:] = -1
        self.assertNotEqual(int(a["hist"][0]), int(b["hist"][0]))


# ── Widget / manager passthrough ──────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestPassthrough(unittest.TestCase):
    def test_widget_and_manager_return_backend_stats(self):
        mgr, cam = _andor_widget()
        st = cam.get_raw_frame_stats()
        self.assertIsNotNone(st)
        self.assertEqual(st["clip_level"], 65535)
        st2 = mgr.get_raw_frame_stats(0)
        self.assertIsNotNone(st2)

    def test_none_for_slot_without_backend(self):
        mgr = CameraManager(max_cameras=1)
        self.assertIsNone(mgr.get_raw_frame_stats(0))


# ── Histogram widget ──────────────────────────────────────────────────

class TestRawHistogramWidget(unittest.TestCase):
    def _paint(self, w):
        w.resize(200, 100)
        pix = w.grab()          # forces a paintEvent offscreen
        self.assertFalse(pix.isNull())

    def test_paints_with_stats(self):
        w = RawHistogramWidget()
        hist = np.random.randint(0, 1000, size=256)
        w.set_stats({"hist": hist, "clipped_frac": 0.0, "clip_level": 65535})
        self._paint(w)

    def test_paints_with_clipping(self):
        w = RawHistogramWidget()
        hist = np.zeros(256, dtype=np.int64)
        hist[-1] = 500
        w.set_stats({"hist": hist, "clipped_frac": 0.02, "clip_level": 4095})
        self._paint(w)

    def test_paints_with_none_and_all_zero(self):
        w = RawHistogramWidget()
        w.set_stats(None)
        self._paint(w)
        w.set_stats({"hist": np.zeros(256, dtype=np.int64),
                     "clipped_frac": 0.0, "clip_level": 65535})
        self._paint(w)          # log-y must not divide by zero


# ── Live-feed SATURATED badge ─────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestFeedSaturationBadge(unittest.TestCase):
    def _view(self, clipped_frac):
        mgr = CameraManager(max_cameras=1)
        view = CameraFeedView(camera_manager=mgr, cam_idx=0,
                              show_crosshair=False, label="t")
        mgr.get_raw_frame_stats = (
            lambda i, cf=clipped_frac:
            None if cf is None else {"clipped_frac": cf, "clip_level": 65535})
        return view

    def test_flag_set_above_threshold(self):
        view = self._view(SATURATION_WARN_FRAC * 2)
        view._update_saturation_flag()
        self.assertTrue(view._saturated)

    def test_flag_clear_below_threshold(self):
        view = self._view(SATURATION_WARN_FRAC / 10)
        view._update_saturation_flag()
        self.assertFalse(view._saturated)

    def test_flag_clear_when_no_stats(self):
        view = self._view(None)
        view._saturated = True
        view._update_saturation_flag()
        self.assertFalse(view._saturated)

    def test_render_with_badge_does_not_crash(self):
        view = self._view(1.0)
        view._update_saturation_flag()
        view.resize(320, 240)
        img = QImage(64, 64, QImage.Format_RGB888)
        img.fill(0xFF8800)
        view._render_frame(img)      # badge branch executes cleanly

    def test_dialog_signal_section_renders_fake_stats(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        mgr, _cam = _andor_widget()
        dlg = CameraSettingsDialog(mgr, 0)
        self.assertTrue(dlg._has_raw_stats)
        dlg._refresh_raw_stats()
        self.assertIn("clipped", dlg._signal_label.text())


if __name__ == "__main__":
    unittest.main()
