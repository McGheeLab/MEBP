"""
test_v713_tucam_raw_parity.py — the Tucsen gets the Zyla's v7.13 treatment.

Raw 16-bit statistics (histogram / clipped fraction against the frame's OWN
declared bit depth), the shared RawAverageRequest averaged-capture contract,
capability advertising (Signal section + SATURATED badge light up through the
same 'andor_raw_stats' gate), the software-correction readout line, and the
retroactive post-processing helper.

No SDK: the real TUCamBackend's raw-plane servicing is pure Python once the
plane exists, and the widget path runs with the backend injected.
"""

import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.mono_display import RawAverageRequest
from gui.widgets.tucam_backend import TUCamBackend
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager


class TestServiceRawPlane(unittest.TestCase):
    def _plane(self, val=100, clip_at=None, n_clip=8):
        p = np.full((64, 64), val, dtype=np.uint16)
        if clip_at is not None:
            p.flat[:n_clip] = clip_at
        return p

    def test_clip_level_follows_frame_depth(self):
        be = TUCamBackend()
        be._service_raw_plane(self._plane(clip_at=4095), depth=12)
        st = be.get_raw_frame_stats()
        self.assertEqual(st["clip_level"], 4095)
        self.assertEqual(be.get_raw_clip_level(), 4095)
        self.assertAlmostEqual(st["clipped_frac"], 8 / 4096)
        # 16-bit frame → 65535 clip; the 4095 pixels no longer count.
        be._service_raw_plane(self._plane(clip_at=4095), depth=16)
        st = be.get_raw_frame_stats()
        self.assertEqual(st["clip_level"], 65535)
        self.assertEqual(st["clipped_frac"], 0.0)

    def test_bogus_depth_falls_back_to_full_scale(self):
        be = TUCamBackend()
        be._service_raw_plane(self._plane(), depth=0)
        self.assertEqual(be.get_raw_frame_stats()["clip_level"], 65535)

    def test_stats_snapshot_is_a_copy(self):
        be = TUCamBackend()
        be._service_raw_plane(self._plane(), depth=16)
        a = be.get_raw_frame_stats()
        b = be.get_raw_frame_stats()
        a["hist"][:] = -1
        self.assertNotEqual(int(a["hist"][0]), int(b["hist"][0]))

    def test_avg_request_serviced_by_plane_path(self):
        be = TUCamBackend()
        req = RawAverageRequest(2)
        be._avg_request = req
        be._service_raw_plane(np.full((16, 16), 59000, dtype=np.uint16), 16)
        be._service_raw_plane(np.full((16, 16), 61000, dtype=np.uint16), 16)
        self.assertTrue(req.done.is_set())
        out = req.result()
        self.assertTrue(np.all(out == 60000))   # float64 acc, no overflow
        self.assertIsNone(be._avg_request)      # slot cleared for next caller

    def test_fail_pending_unblocks_caller(self):
        be = TUCamBackend()
        req = RawAverageRequest(100)
        be._avg_request = req
        waiter = threading.Thread(target=req.done.wait, daemon=True)
        waiter.start()
        be._fail_pending_average("test")
        waiter.join(timeout=2.0)
        self.assertFalse(waiter.is_alive())
        self.assertIsNone(req.result())

    def test_stop_stream_fails_pending(self):
        be = TUCamBackend()
        req = RawAverageRequest(100)
        be._avg_request = req
        be._stop_stream()          # closed backend: guarded no-op otherwise
        self.assertTrue(req.done.is_set())
        self.assertIsNotNone(req.error)

    def test_capture_refused_when_closed(self):
        be = TUCamBackend()
        self.assertIsNone(be.capture_raw_average(4, timeout_s=0.2))

    def test_settings_carry_clip_level(self):
        be = TUCamBackend()
        be._service_raw_plane(self._plane(), depth=12)
        self.assertEqual(be.get_settings()["raw_clip_level"], 4095)


@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestWidgetParity(unittest.TestCase):
    def _tucam_widget(self):
        mgr = CameraManager(max_cameras=1)
        cam = mgr.cameras[0]
        be = TUCamBackend()
        be._service_raw_plane(np.full((32, 32), 100, dtype=np.uint16), 12)
        cam._tucam = be
        cam._backend_type = "tucam"
        cam._running = True
        cam._camera_index = 0
        return mgr, cam, be

    def test_caps_advertise_raw_stats(self):
        _mgr, cam, _be = self._tucam_widget()
        ctrls = cam.hardware_capabilities()["controls"]
        self.assertIn("andor_raw_stats", ctrls)   # the shared gate key

    def test_stats_and_average_route_through_manager(self):
        mgr, _cam, be = self._tucam_widget()
        st = mgr.get_raw_frame_stats(0)
        self.assertIsNotNone(st)
        self.assertEqual(st["clip_level"], 4095)
        # capture_raw_average routes to the backend (closed → clean None).
        self.assertIsNone(mgr.capture_raw_average(0, 4, timeout_s=0.2))


@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestSoftwareCorrectionReadout(unittest.TestCase):
    def test_readout_shows_display_only_correction(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        from tests.test_v75x_andor_zyla_camera import _andor_widget
        mgr, cam = _andor_widget()
        cam.set_brightness(12)
        cam.set_contrast(1.10)
        dlg = CameraSettingsDialog(mgr, 0)
        text = dlg._readout.toPlainText()
        self.assertIn("software corr", text)
        self.assertIn("+12", text)
        self.assertIn("1.10", text)
        self.assertIn("display-only", text)


class TestRetroactivePostProcess(unittest.TestCase):
    def setUp(self):
        import SupportClasses.FluorescenceMosaicStore as fms
        self._fms = fms
        self._orig = fms._store_singleton
        tmp = Path(tempfile.mkdtemp()) / "fluor.json"
        fms._store_singleton = fms.FluorescenceMosaicStore(tmp)

    def tearDown(self):
        self._fms._store_singleton = self._orig

    def _page(self, denoise="median"):
        msgs = []
        return SimpleNamespace(
            _plate_key=lambda: "p",
            _scan_well="A1",
            _post_settings=lambda: {"denoise": denoise,
                                    "denoise_strength": 1,
                                    "bg_subtract": False,
                                    "bg_radius_um": 100.0},
            _refresh_preview=lambda: None,
            _status=SimpleNamespace(setText=msgs.append),
        ), msgs

    def _apply(self, page):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        FluorescenceMosaicWorkflowPage._apply_post_to_captured(page)

    def test_applies_to_stored_channels(self):
        st = self._fms.get_store()
        img = np.full((40, 40, 3), 50, dtype=np.uint8)
        img[10, 10] = 255                       # speckle the median removes
        st.save_channel("p", "A1", "DAPI", img, (0, 0, 100, 100),
                        mosaic_scale=0.5)
        page, msgs = self._page()
        self._apply(page)
        self.assertIn("1/1", msgs[-1])
        self.assertEqual(st.get_processing("p", "A1", "DAPI")["denoise"],
                         "median")
        proc = st.load_channel_image("p", "A1", "DAPI",
                                     prefer_processed=True)
        self.assertEqual(int(proc[10, 10, 0]), 50)   # speckle gone
        raw = st.load_channel_image("p", "A1", "DAPI")
        self.assertEqual(int(raw[10, 10, 0]), 255)   # raw untouched

    def test_refuses_when_nothing_enabled(self):
        st = self._fms.get_store()
        st.save_channel("p", "A1", "DAPI",
                        np.full((40, 40, 3), 50, dtype=np.uint8),
                        (0, 0, 100, 100))
        page, msgs = self._page(denoise="off")
        self._apply(page)
        self.assertIn("Enable", msgs[-1])
        self.assertIsNone(st.get_processing("p", "A1", "DAPI"))

    def test_no_channels_reports(self):
        page, msgs = self._page()
        self._apply(page)
        self.assertIn("No captured channels", msgs[-1])


if __name__ == "__main__":
    unittest.main()
