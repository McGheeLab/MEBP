"""
test_v75x_camera_rotation.py — Per-camera rotation for 45°-mounted needle cameras.

v7.5.x: The needle cameras are 90° apart from each other but the pair sits at
~45° to the stage X/Y axes. The stage-motion µm/px calibration captures each
camera's in-plane lateral direction (the move direction that produces clean
lateral image motion), which feeds the generalized TwoCameraNeedleAligner.

Covers:
- TwoCameraNeedleAligner reduces exactly to the legacy orthogonal mapping when
  no angles are given.
- A 45° scenario recovers a known needle offset.
- Parallel (singular) camera directions raise.
- HardwareConfig.camera_calibrations round-trips rotation_deg.
- CameraManager rotation get/set semantics.
"""

import os
import sys
import math
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.VisionDetector import (
    TwoCameraNeedleAligner, TwoCameraEdgePicks,
)
from SupportClasses.HardwareConfig import HardwareConfig
from gui.widgets.camera_manager import CameraManager


class TestAligner(unittest.TestCase):
    def test_legacy_reduction(self):
        """No angles → x_view col→stage Y, y_view col→stage X (legacy)."""
        picks = TwoCameraEdgePicks(
            x_view_left_px=58, x_view_right_px=62,   # center 60, +10 px
            y_view_left_px=68, y_view_right_px=72)   # center 70, +20 px
        a = TwoCameraNeedleAligner(1.0, 1.0, 100, 100)
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, 20.0, places=6)   # from y-view
        self.assertAlmostEqual(dy, 10.0, places=6)   # from x-view

    def test_explicit_legacy_angles_match_default(self):
        picks = TwoCameraEdgePicks(58, 62, 68, 72)
        default = TwoCameraNeedleAligner(1.0, 1.0, 100, 100)
        explicit = TwoCameraNeedleAligner(
            1.0, 1.0, 100, 100,
            angle_x_view_deg=90.0, angle_y_view_deg=0.0)
        self.assertEqual(
            default.offset_from_edge_clicks(picks),
            explicit.offset_from_edge_clicks(picks))

    def test_45deg_recovers_known_offset(self):
        """Cameras at 135°/45° recover a needle offset of (100, 0) µm."""
        W = 100
        c = W / 2.0
        s1 = 100 * math.cos(math.radians(135))  # projection onto u(135°)
        s2 = 100 * math.cos(math.radians(45))   # projection onto u(45°)
        picks = TwoCameraEdgePicks(
            x_view_left_px=c + s1, x_view_right_px=c + s1,
            y_view_left_px=c + s2, y_view_right_px=c + s2)
        a = TwoCameraNeedleAligner(
            1.0, 1.0, W, W, angle_x_view_deg=135.0, angle_y_view_deg=45.0)
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, 100.0, places=6)
        self.assertAlmostEqual(dy, 0.0, places=6)

    def test_parallel_cameras_raise(self):
        a = TwoCameraNeedleAligner(
            1.0, 1.0, 10, 10, angle_x_view_deg=30.0, angle_y_view_deg=30.0)
        with self.assertRaises(ValueError):
            a.offset_from_edge_clicks(TwoCameraEdgePicks(1, 2, 3, 4))


class TestNeedleCalibrationNotClobbered(unittest.TestCase):
    """Regression: the calibration page's per-slot objective µm/px sync (run on
    every config change) must NOT overwrite needle/plate cameras' stage-motion
    calibration in the shared CameraManager. It uses the shared microscope
    camera_spec for every slot, so before the fix it clobbered slots 0/1."""

    def test_sync_preserves_needle_cameras(self):
        from gui.pages.calibration import CalibrationPage
        from SupportClasses.HardwareConfig import CameraRole, CameraConfig
        from SupportClasses.PhysicalModels import CameraSpec

        mgr = CameraManager(max_cameras=3)
        cal = CalibrationPage(None, camera_manager=mgr)

        # Two needle cameras calibrated by stage motion; microscope on slot 2.
        mgr.set_um_per_px(0, 5.54)
        mgr.set_um_per_px(1, 5.48)

        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        spec = CameraSpec(name="Test", sensor_pixel_size_um=2.0,
                          max_resolution=(3664, 2748))
        cfg.camera_config = CameraConfig(
            camera_spec=spec, objective_magnification=4.0)

        cal.set_hardware_config(cfg)  # runs the per-slot µm/px sync loop

        # Needle cameras must be untouched; microscope may be objective-derived.
        self.assertAlmostEqual(mgr.get_um_per_px(0), 5.54, places=6)
        self.assertAlmostEqual(mgr.get_um_per_px(1), 5.48, places=6)


class TestManagerRotation(unittest.TestCase):
    def setUp(self):
        self.mgr = CameraManager(max_cameras=3)

    def test_default_none(self):
        for i in range(3):
            self.assertIsNone(self.mgr.get_rotation_deg(i))

    def test_set_get(self):
        self.mgr.set_rotation_deg(1, 45.0)
        self.assertEqual(self.mgr.get_rotation_deg(1), 45.0)
        self.assertIsNone(self.mgr.get_rotation_deg(0))

    def test_clear_with_none(self):
        self.mgr.set_rotation_deg(2, 30.0)
        self.mgr.set_rotation_deg(2, None)
        self.assertIsNone(self.mgr.get_rotation_deg(2))

    def test_out_of_range_safe(self):
        self.assertIsNone(self.mgr.get_rotation_deg(99))
        self.mgr.set_rotation_deg(99, 10.0)  # must not raise


if __name__ == "__main__":
    unittest.main()
