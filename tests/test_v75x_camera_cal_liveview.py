"""
test_v75x_camera_cal_liveview.py — CameraManager µm/px calibration-flag semantics.

v7.5.x: The needle-zero (Needle Location) and plate-location workflows must
refuse to run on a camera whose µm/px is still the 1.67 seed default, but
proceed once it has been explicitly calibrated. They distinguish the two via
``CameraManager.is_um_per_px_calibrated`` rather than ``get_um_per_px > 0``
(the seed default is > 0 and would mask the uncalibrated case).

Regression guard for the bug where Needle Location read ``cam._um_per_px``
(absent on CameraWidget → always 0.0 → "cannot compute µm/px") instead of the
shared CameraManager value.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.camera_manager import CameraManager
from SupportClasses.HardwareConfig import HardwareConfig, MAX_LIVE_CAMERAS


class TestUmPerPxCalibrationFlag(unittest.TestCase):
    def setUp(self):
        self.mgr = CameraManager(max_cameras=3)

    def test_default_is_uncalibrated(self):
        """Fresh slots report not-calibrated even though get_um_per_px > 0."""
        for i in range(3):
            self.assertFalse(self.mgr.is_um_per_px_calibrated(i))
            # The seed default is a positive number — exactly the trap the
            # flag exists to guard against.
            self.assertGreater(self.mgr.get_um_per_px(i), 0.0)

    def test_set_marks_calibrated(self):
        self.mgr.set_um_per_px(1, 0.823)
        self.assertTrue(self.mgr.is_um_per_px_calibrated(1))
        self.assertAlmostEqual(self.mgr.get_um_per_px(1), 0.823, places=6)

    def test_per_slot_independence(self):
        """Calibrating one slot does not flip the others."""
        self.mgr.set_um_per_px(0, 1.23)
        self.assertTrue(self.mgr.is_um_per_px_calibrated(0))
        self.assertFalse(self.mgr.is_um_per_px_calibrated(1))
        self.assertFalse(self.mgr.is_um_per_px_calibrated(2))

    def test_out_of_range_is_safe(self):
        self.assertFalse(self.mgr.is_um_per_px_calibrated(-1))
        self.assertFalse(self.mgr.is_um_per_px_calibrated(99))
        # set_um_per_px on an OOB index must not raise or mark anything.
        self.mgr.set_um_per_px(99, 5.0)
        for i in range(3):
            self.assertFalse(self.mgr.is_um_per_px_calibrated(i))


class TestLegacyCameraCalibrationsIgnored(unittest.TestCase):
    """v7.5.x: per-camera µm/px moved out of HardwareConfig into the
    per-machine CameraCalibrationStore (see test_v75x_camera_calibration_store).
    A legacy "camera_calibrations" key in an old saved config is ignored and
    no longer serialized back out."""

    def test_field_removed_from_serialization(self):
        cfg = HardwareConfig()
        self.assertNotIn("camera_calibrations", cfg.to_dict())
        self.assertFalse(hasattr(cfg, "camera_calibrations"))

    def test_legacy_key_ignored_on_load(self):
        back = HardwareConfig.from_dict(
            {"camera_calibrations": {"dshow:A": {"um_per_px": 5.5}}})
        self.assertNotIn("camera_calibrations", back.to_dict())


class TestCameraIdentity(unittest.TestCase):
    """v7.5.x: device-identity resolution (name + USB port) used to key the
    per-camera calibration store."""

    def setUp(self):
        from gui.widgets import camera_identity as ci
        self.ci = ci
        # Two physically-identical cameras (same name/VID/PID, distinct ports).
        self.ds = [
            {"index": 0, "name": "Teslong Camera",
             "device_path": "\\\\?\\usb#vid_f007&pid_a999&mi_00#6&29d1719c&2&0000#{guid}\\global"},
            {"index": 1, "name": "Teslong Camera",
             "device_path": "\\\\?\\usb#vid_f007&pid_a999&mi_00#7&b643b4c&0&0000#{guid}\\global"},
        ]

    def test_short_port_tag(self):
        self.assertEqual(
            self.ci.short_port_tag(self.ds[0]["device_path"]), "6&29d1719c&2")
        self.assertEqual(self.ci.short_port_tag(""), "")

    def test_label_includes_type_and_port(self):
        lbl = self.ci.label_for(self.ds[0]["name"], self.ds[0]["device_path"])
        self.assertIn("Teslong Camera", lbl)
        self.assertIn("6&29d1719c&2", lbl)

    def test_identical_cameras_get_distinct_identities(self):
        id0 = self.ci.identity_for_source(("opencv", 0), self.ds)
        id1 = self.ci.identity_for_source(("opencv", 1), self.ds)
        self.assertIsNotNone(id0)
        self.assertIsNotNone(id1)
        self.assertEqual(id0[1], "Teslong Camera")        # same type
        self.assertNotEqual(id0[0], id1[0])               # distinct identity

    def test_fallbacks(self):
        # No DirectShow info → index-based identity.
        self.assertEqual(
            self.ci.identity_for_source(("opencv", 5), self.ds),
            ("opencv:5", "Camera 5"))
        self.assertEqual(
            self.ci.identity_for_source(("toupcam", "ABC"), self.ds),
            ("toupcam:ABC", "ToupCam"))
        self.assertEqual(
            self.ci.identity_for_source(("simulated", "microscope"), self.ds),
            ("simulated:microscope", "Simulated Camera"))
        self.assertIsNone(self.ci.identity_for_source(None, self.ds))


if __name__ == "__main__":
    unittest.main()
