"""
test_v75x_needle_center_direction_z.py
======================================

v7.5.x — Needle auto-center direction fix + Z centering on the crosshair.

See `coding plans/Update plans/MEBP_v75x_NEEDLE_CENTER_DIRECTION_AND_Z.md`.

Covers:
- ``plus_column_direction_deg`` resolves the +column stage direction from the
  MEASURED displacement sign (the wrong-direction auto-center fix), not the
  arbitrary commanded preset.
- The corrected angle, fed back through ``TwoCameraNeedleAligner``, recenters a
  needle offset that the raw-commanded (wrong-sign) angle would push away.
- ``CalibrationPage._needle_loc_compute_z_offset_um`` maps the tip-bottom rows
  to a height-frame Z move (up = +), averages the two views, and honors Invert.
"""

import os
import sys
import math
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.dialogs.pixel_calibration_dialog import plus_column_direction_deg
from SupportClasses.VisionDetector import (
    TwoCameraNeedleAligner, TwoCameraEdgePicks,
)
from SupportClasses.HardwareConfig import HardwareConfig, CameraRole
from gui.widgets.camera_manager import CameraManager


def _norm(a: float) -> float:
    return ((a + 180.0) % 360.0) - 180.0


class TestPlusColumnDirection(unittest.TestCase):
    def test_negative_dx_keeps_commanded_angle(self):
        # dx < 0 → +θ̂ already increases column → angle unchanged.
        self.assertAlmostEqual(plus_column_direction_deg(45.0, -7.0), 45.0)
        self.assertAlmostEqual(plus_column_direction_deg(-45.0, -3.0), -45.0)

    def test_positive_dx_flips_180(self):
        # dx > 0 → "+column" direction is the opposite stage direction.
        self.assertAlmostEqual(plus_column_direction_deg(45.0, 7.0), -135.0)
        self.assertAlmostEqual(plus_column_direction_deg(-45.0, 3.0), 135.0)

    def test_result_is_normalized(self):
        for cmd in (-180.0, -90.0, 0.0, 30.0, 90.0, 179.0):
            for dx in (-2.0, 2.0):
                a = plus_column_direction_deg(cmd, dx)
                self.assertTrue(-180.0 <= a < 180.0, f"{cmd},{dx} -> {a}")

    def test_on_axis_reduces_to_sign_rule(self):
        # A purely lateral move (dy=0) reduces to the simple ±180° sign rule:
        # dx<0 → θ, dx>0 → θ+180. (The dialog gates magnitude>=3, so the
        # degenerate dx=dy=0 case never reaches the helper.)
        self.assertAlmostEqual(plus_column_direction_deg(90.0, -4.0, 0.0), 90.0)
        self.assertAlmostEqual(plus_column_direction_deg(90.0, 4.0, 0.0), -90.0)

    def test_off_axis_recovers_true_lateral_axis(self):
        # The camera's true lateral axis is β=30°; stage→image rotation α=−β, so
        # a commanded move θ produces content displacement at φ=θ−β+180. The
        # helper must recover β regardless of the (quantized) commanded preset —
        # this is the off-axis correction that fixes the "lands left" residual.
        beta = 30.0
        for theta in (0.0, 45.0, 90.0, -45.0, 135.0):
            phi = math.radians(theta - beta + 180.0)
            dx, dy = math.cos(phi), math.sin(phi)
            got = plus_column_direction_deg(theta, dx, dy)
            self.assertAlmostEqual(_norm(got - beta), 0.0, places=4,
                                   msg=f"theta={theta} -> {got}, want {beta}")

    def test_independent_of_commanded_preset(self):
        # Two operators picking different presets for the SAME physical camera
        # must store the same lateral axis (given consistent measured motion).
        beta = -20.0
        results = []
        for theta in (45.0, -45.0):
            phi = math.radians(theta - beta + 180.0)
            results.append(plus_column_direction_deg(
                theta, math.cos(phi), math.sin(phi)))
        self.assertAlmostEqual(_norm(results[0] - results[1]), 0.0, places=4)


class TestAlignerWithCorrectedSign(unittest.TestCase):
    """The corrected angle recenters; the wrong (un-flipped) sign diverges."""

    def test_corrected_sign_recovers_offset(self):
        W = 100
        c = W / 2.0
        # True needle stage offset.
        n = (80.0, 0.0)
        # Two cameras whose +column directions are 135° and 45° (a valid 90°
        # pair). The operator commanded -45° and +45°; both measured dx>0, so
        # the helper flips each by 180° → 135° and 225°→-135°... use 45°/135°
        # directly as the *corrected* angles and confirm recovery.
        a_x = plus_column_direction_deg(-45.0, +5.0)   # -> 135°
        a_y = plus_column_direction_deg(45.0, -5.0)    # ->  45°
        self.assertAlmostEqual(a_x, 135.0)
        self.assertAlmostEqual(a_y, 45.0)
        s1 = n[0] * math.cos(math.radians(a_x)) + n[1] * math.sin(math.radians(a_x))
        s2 = n[0] * math.cos(math.radians(a_y)) + n[1] * math.sin(math.radians(a_y))
        picks = TwoCameraEdgePicks(
            x_view_left_px=c + s1, x_view_right_px=c + s1,
            y_view_left_px=c + s2, y_view_right_px=c + s2)
        aligner = TwoCameraNeedleAligner(
            1.0, 1.0, W, W, angle_x_view_deg=a_x, angle_y_view_deg=a_y)
        dx, dy = aligner.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, n[0], places=6)
        self.assertAlmostEqual(dy, n[1], places=6)


class _FakeCam:
    def __init__(self, h: int, w: int):
        self._frame = np.zeros((h, w, 3), dtype=np.uint8)

    def get_current_frame(self):
        return self._frame


class TestZOffset(unittest.TestCase):
    def _page(self, h=480, w=640, upp=5.0):
        from gui.pages.calibration import CalibrationPage
        mgr = CameraManager(max_cameras=3)
        # Replace the real widgets with fakes that report a known frame size.
        mgr._cameras[0] = _FakeCam(h, w)
        mgr._cameras[1] = _FakeCam(h, w)
        mgr.set_um_per_px(0, upp)
        mgr.set_um_per_px(1, upp)
        cal = CalibrationPage(None, camera_manager=mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        cal._hardware_config = cfg
        return cal

    def test_tip_below_center_moves_up(self):
        cal = self._page(h=480, upp=5.0)
        # Tip 60 px below center in both views → +60 px → +300 µm up.
        cal._needle_loc_rows = {
            "x_left": 300, "x_right": 300, "y_left": 300, "y_right": 300}
        dz = cal._needle_loc_compute_z_offset_um()
        self.assertIsNotNone(dz)
        self.assertAlmostEqual(dz, 60.0 * 5.0, places=6)   # +300 µm (up)

    def test_tip_above_center_moves_down(self):
        cal = self._page(h=480, upp=5.0)
        cal._needle_loc_rows = {
            "x_left": 180, "x_right": 180, "y_left": 180, "y_right": 180}
        dz = cal._needle_loc_compute_z_offset_um()
        self.assertAlmostEqual(dz, -60.0 * 5.0, places=6)  # -300 µm (down)

    def test_invert_negates(self):
        cal = self._page(h=480, upp=5.0)
        cal._needle_loc_rows = {
            "x_left": 300, "x_right": 300, "y_left": 300, "y_right": 300}
        cal._needle_loc_z_invert_chk.setChecked(True)
        dz = cal._needle_loc_compute_z_offset_um()
        self.assertAlmostEqual(dz, -300.0, places=6)

    def test_two_views_averaged(self):
        cal = self._page(h=480, upp=5.0)
        # X-view tip at +60 px → +300 µm; Y-view tip at +20 px → +100 µm.
        cal._needle_loc_rows = {
            "x_left": 300, "x_right": 300, "y_left": 260, "y_right": 260}
        dz = cal._needle_loc_compute_z_offset_um()
        self.assertAlmostEqual(dz, (300.0 + 100.0) / 2.0, places=6)

    def test_no_rows_returns_none(self):
        cal = self._page()
        cal._needle_loc_rows = {}
        self.assertIsNone(cal._needle_loc_compute_z_offset_um())

    def test_uncalibrated_camera_skipped(self):
        from gui.pages.calibration import CalibrationPage
        mgr = CameraManager(max_cameras=3)
        mgr._cameras[0] = _FakeCam(480, 640)
        mgr._cameras[1] = _FakeCam(480, 640)
        # Only camera 0 calibrated; camera 1 left at the uncalibrated seed.
        mgr.set_um_per_px(0, 5.0)
        cal = CalibrationPage(None, camera_manager=mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        cal._hardware_config = cfg
        cal._needle_loc_rows = {
            "x_left": 300, "x_right": 300, "y_left": 300, "y_right": 300}
        # Y-view skipped (uncalibrated) → only X-view contributes.
        dz = cal._needle_loc_compute_z_offset_um()
        self.assertAlmostEqual(dz, 300.0, places=6)


class TestDeadband(unittest.TestCase):
    """`_needle_loc_already_centered`: crosshair within the needle span in BOTH
    views → no XY move (stops near-center click-noise from nudging the needle)."""

    def _page(self, h=480, w=640, upp=5.0):
        from gui.pages.calibration import CalibrationPage
        mgr = CameraManager(max_cameras=3)
        mgr._cameras[0] = _FakeCam(h, w)
        mgr._cameras[1] = _FakeCam(h, w)
        mgr.set_um_per_px(0, upp)
        mgr.set_um_per_px(1, upp)
        cal = CalibrationPage(None, camera_manager=mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        cal._hardware_config = cfg
        return cal

    def test_centered_when_crosshair_within_span_both_views(self):
        # 640px frame → center 320; both spans bracket 320.
        cal = self._page()
        cal._needle_loc_picks = TwoCameraEdgePicks(310, 330, 315, 325)
        self.assertTrue(cal._needle_loc_already_centered())

    def test_off_center_when_x_view_span_excludes_center(self):
        cal = self._page()
        cal._needle_loc_picks = TwoCameraEdgePicks(100, 140, 315, 325)
        self.assertFalse(cal._needle_loc_already_centered())

    def test_off_center_when_y_view_span_excludes_center(self):
        cal = self._page()
        cal._needle_loc_picks = TwoCameraEdgePicks(310, 330, 500, 560)
        self.assertFalse(cal._needle_loc_already_centered())

    def test_click_order_independent(self):
        # right-then-left clicks (L>R) must still bracket center via min/max.
        cal = self._page()
        cal._needle_loc_picks = TwoCameraEdgePicks(330, 310, 325, 315)
        self.assertTrue(cal._needle_loc_already_centered())

    def test_uncalibrated_falls_back_to_moving(self):
        # No µm/px on Y-view → cannot confirm centered → False (so it moves).
        from gui.pages.calibration import CalibrationPage
        mgr = CameraManager(max_cameras=3)
        mgr._cameras[0] = _FakeCam(480, 640)
        mgr._cameras[1] = _FakeCam(480, 640)
        mgr.set_um_per_px(0, 5.0)  # only X-view calibrated
        cal = CalibrationPage(None, camera_manager=mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        cal._hardware_config = cfg
        cal._needle_loc_picks = TwoCameraEdgePicks(310, 330, 315, 325)
        self.assertFalse(cal._needle_loc_already_centered())


if __name__ == "__main__":
    unittest.main()
