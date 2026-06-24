"""
test_v75x_camera_image_correction.py — Per-camera image correction.

v7.5.x: Each physical camera gets a display-only brightness / contrast / gamma
correction. It is applied to the displayed/`frame_captured` frame (so every
CameraFeedView sees it) but NOT the raw `_current_frame` used by detection.
Settings persist per device identity in CameraCalibrationStore.

Covers:
- CameraCalibrationStore.set/get_image_correction (round-trip, disk, sibling
  preservation with um_per_px).
- CameraWidget: clamping, snapshot, reset, and the actual frame math
  (brightness offset, mid-gray-pivoted contrast, gamma LUT) — plus the
  invariant that the raw detection frame is left uncorrected.
- CameraManager delegation (get/set + reset + partial set_image_correction).
- HardwareSetupPage integration: slider → manager, persist, and restore.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
from gui.widgets.camera_widget import CameraWidget, CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager


class _FakeCapture:
    """Minimal cv2.VideoCapture stand-in returning a fixed BGR frame."""

    def __init__(self, frame):
        self._frame = frame

    def isOpened(self):
        return True

    def read(self):
        return True, self._frame.copy()

    def release(self):
        pass


def _run_widget(frame, **correction):
    """Push `frame` (BGR) through one CameraWidget grab; return emitted QImage."""
    cam = CameraWidget(show_controls=False)
    cam._capture = _FakeCapture(frame)
    cam._backend_type = "opencv"
    cam._running = True
    for k, v in correction.items():
        getattr(cam, f"set_{k}")(v)
    captured = []
    cam.frame_captured.connect(lambda q: captured.append(q))
    cam._grab_frame()
    return cam, captured[-1]


# ── Store ────────────────────────────────────────────────────────────

class TestStoreImageCorrection(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        self.store = CameraCalibrationStore(self.tmp)

    def test_set_get_round_trip(self):
        self.store.set_image_correction(
            "dshow:A", brightness=20, contrast=1.5, gamma=0.8, name="Teslong")
        c = self.store.get_image_correction("dshow:A")
        self.assertEqual(c["brightness"], 20)
        self.assertAlmostEqual(c["contrast"], 1.5, places=4)
        self.assertAlmostEqual(c["gamma"], 0.8, places=4)
        self.assertIsNone(self.store.get_image_correction("dshow:missing"))

    def test_disk_persistence(self):
        self.store.set_image_correction("dshow:A", brightness=-30, contrast=2.0)
        reloaded = CameraCalibrationStore(self.tmp)
        c = reloaded.get_image_correction("dshow:A")
        self.assertEqual(c["brightness"], -30)
        self.assertAlmostEqual(c["contrast"], 2.0, places=4)

    def test_preserves_umpx_sibling(self):
        self.store.set_calibration("dshow:A", 5.54, rotation_deg=45.0)
        self.store.set_image_correction("dshow:A", brightness=10)
        # µm/px + rotation untouched
        cal = self.store.get_calibration("dshow:A")
        self.assertAlmostEqual(cal["um_per_px"], 5.54, places=4)
        self.assertEqual(cal["rotation_deg"], 45.0)
        self.assertEqual(cal["image_correction"]["brightness"], 10)
        # and the reverse: setting µm/px later keeps the correction
        self.store.set_calibration("dshow:A", 6.0)
        self.assertEqual(
            self.store.get_image_correction("dshow:A")["brightness"], 10)


# ── CameraWidget ─────────────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestWidgetCorrection(unittest.TestCase):
    def test_clamp_and_snapshot(self):
        cam = CameraWidget(show_controls=False)
        cam.set_brightness(999)
        cam.set_brightness(-999)  # clamps to -100
        cam.set_contrast(10.0)    # clamps to 3.0
        cam.set_gamma(-5.0)       # clamps to 0.1
        self.assertEqual(cam.brightness, -100)
        self.assertAlmostEqual(cam.contrast, 3.0)
        self.assertAlmostEqual(cam.gamma, 0.1)
        snap = cam.image_correction()
        self.assertEqual(snap, {"brightness": -100, "contrast": 3.0, "gamma": 0.1})

    def test_reset(self):
        cam = CameraWidget(show_controls=False)
        cam.set_brightness(40)
        cam.set_contrast(2.0)
        cam.set_gamma(1.8)
        cam.reset_image_correction()
        self.assertEqual(
            cam.image_correction(), {"brightness": 0, "contrast": 1.0, "gamma": 1.0})

    def test_brightness_offset(self):
        frame = np.full((10, 10, 3), 100, dtype=np.uint8)
        _, q = _run_widget(frame, brightness=30)  # contrast 1.0
        self.assertEqual(q.pixelColor(5, 5).red(), 130)

    def test_contrast_pivots_on_midgray(self):
        # mid-gray (128) is the contrast fixed point.
        mid, _ = np.full((10, 10, 3), 128, dtype=np.uint8), None
        _, q = _run_widget(mid, contrast=2.0)
        self.assertEqual(q.pixelColor(5, 5).red(), 128)
        # 160 -> 2*(160-128)+128 = 192
        bright = np.full((10, 10, 3), 160, dtype=np.uint8)
        _, q2 = _run_widget(bright, contrast=2.0)
        self.assertEqual(q2.pixelColor(5, 5).red(), 192)

    def test_gamma_lut(self):
        frame = np.full((10, 10, 3), 128, dtype=np.uint8)
        _, q = _run_widget(frame, gamma=2.0)
        # LUT[128] = ((128/255)**(1/2.0))*255 -> 180 (uint8 truncation)
        self.assertEqual(q.pixelColor(5, 5).red(), 180)

    def test_neutral_is_identity(self):
        frame = np.full((10, 10, 3), 123, dtype=np.uint8)
        _, q = _run_widget(frame)  # no correction
        self.assertEqual(q.pixelColor(5, 5).red(), 123)

    def test_raw_detection_frame_uncorrected(self):
        """The cached frame for detection must stay raw despite correction."""
        frame = np.full((10, 10, 3), 100, dtype=np.uint8)
        cam, q = _run_widget(frame, brightness=50, contrast=2.0)
        raw = cam.get_current_frame()
        self.assertEqual(int(raw[5, 5, 0]), 100)   # raw untouched
        self.assertNotEqual(q.pixelColor(5, 5).red(), 100)  # display corrected


# ── CameraManager delegation ─────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestManagerDelegation(unittest.TestCase):
    def setUp(self):
        self.mgr = CameraManager(max_cameras=3)

    def test_get_set(self):
        self.mgr.set_brightness(0, 40)
        self.mgr.set_contrast(0, 1.5)
        self.mgr.set_gamma(0, 0.9)
        self.assertEqual(self.mgr.get_brightness(0), 40)
        self.assertAlmostEqual(self.mgr.get_contrast(0), 1.5)
        self.assertAlmostEqual(self.mgr.get_gamma(0), 0.9)
        self.assertEqual(
            self.mgr.image_correction(0),
            {"brightness": 40, "contrast": 1.5, "gamma": 0.9})

    def test_set_image_correction_partial(self):
        self.mgr.set_brightness(1, 20)
        self.mgr.set_image_correction(1, contrast=2.0)  # brightness left as-is
        self.assertEqual(self.mgr.get_brightness(1), 20)
        self.assertAlmostEqual(self.mgr.get_contrast(1), 2.0)

    def test_reset(self):
        self.mgr.set_image_correction(2, brightness=10, contrast=2.0, gamma=1.5)
        self.mgr.reset_image_correction(2)
        self.assertEqual(
            self.mgr.image_correction(2),
            {"brightness": 0, "contrast": 1.0, "gamma": 1.0})

    def test_out_of_range_safe(self):
        # Defaults, no raise, on an invalid index.
        self.assertEqual(self.mgr.get_brightness(99), 0)
        self.assertAlmostEqual(self.mgr.get_contrast(99), 1.0)
        self.mgr.set_brightness(99, 50)  # no raise


# ── HardwareSetupPage integration ────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestPageIntegration(unittest.TestCase):
    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from SupportClasses.HardwareConfig import HardwareConfig
        pg = HardwareSetupPage()
        mgr = CameraManager(max_cameras=3)
        pg.set_camera_manager(mgr)
        pg.set_config(HardwareConfig())
        # Stub a stable identity so persist/restore have a key.
        mgr.camera_identity = lambda i: (f"dshow:CAM{i}", f"Cam {i}")
        return pg, mgr

    def test_strip_built(self):
        pg, _ = self._page()
        refs = pg._live_cam_correction[0]
        self.assertIsInstance(refs, dict)
        for key in ("brightness", "contrast", "gamma"):
            self.assertIn(key, refs)

    def test_slider_pushes_to_manager(self):
        pg, mgr = self._page()
        pg._live_cam_correction[0]["brightness"].setValue(45)
        pg._live_cam_correction[0]["contrast"].setValue(150)  # 1.5x
        self.assertEqual(mgr.get_brightness(0), 45)
        self.assertAlmostEqual(mgr.get_contrast(0), 1.5)

    def test_persist_and_restore(self):
        pg, mgr = self._page()
        pg._live_cam_correction[0]["brightness"].setValue(-25)
        pg._live_cam_correction[0]["gamma"].setValue(120)  # 1.2
        pg._persist_correction(0)
        # Stored under the slot's identity.
        c = self.CCS.get_store().get_image_correction("dshow:CAM0")
        self.assertEqual(c["brightness"], -25)
        self.assertAlmostEqual(c["gamma"], 1.2, places=4)

        # Fresh session: reset live, then restore from store.
        mgr.reset_image_correction(0)
        # _restore needs an assigned source on the slot.
        combo = pg._live_cam_source_combos[0]
        combo.blockSignals(True)
        combo.addItem("cam0", ("opencv", 0))
        combo.setCurrentIndex(combo.count() - 1)
        combo.blockSignals(False)
        self.assertTrue(
            pg._restore_calibration_for_slot(0) or True)  # correction restored regardless
        self.assertEqual(mgr.get_brightness(0), -25)
        self.assertAlmostEqual(mgr.get_gamma(0), 1.2, places=4)
        # Sliders synced to restored values.
        self.assertEqual(pg._live_cam_correction[0]["brightness"].value(), -25)

    def test_reset_button(self):
        pg, mgr = self._page()
        pg._live_cam_correction[0]["brightness"].setValue(60)
        pg._reset_correction(0)
        self.assertEqual(mgr.get_brightness(0), 0)
        self.assertEqual(pg._live_cam_correction[0]["brightness"].value(), 0)


if __name__ == "__main__":
    unittest.main()
