"""
test_v730_simulated_camera.py — Tests for the SimulatedCamera frame generator.

Verifies:
- Frame generation at correct resolution and format
- Well rendering: red circles at correct positions, black outside
- Needle rendering: dark circle at frame center
- Focus blur: sharp at focal Z, blurry away from it
- Position changes translate wells in the frame
- cv2.VideoCapture-compatible API (read, isOpened, release)
- Integration with VisionDetector (well detection, needle detection, focus score)
"""

import unittest
import math

import cv2
import numpy as np

from SupportClasses.SimulatedCamera import SimulatedCamera
from SupportClasses.WellPlate import WellPlate
from SupportClasses.VisionDetector import (
    WellDetector, NeedleDetector, DetectionResult, FocusResult,
)


class TestFrameGeneration(unittest.TestCase):
    """Test basic frame generation."""

    def test_default_frame_shape(self):
        sim = SimulatedCamera(resolution=(916, 686))
        frame = sim.generate_frame()
        self.assertEqual(frame.shape, (686, 916, 3))
        self.assertEqual(frame.dtype, np.uint8)

    def test_custom_resolution(self):
        sim = SimulatedCamera(resolution=(640, 480))
        frame = sim.generate_frame()
        self.assertEqual(frame.shape, (480, 640, 3))

    def test_read_api(self):
        sim = SimulatedCamera()
        ret, frame = sim.read()
        self.assertTrue(ret)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape[2], 3)

    def test_is_opened(self):
        sim = SimulatedCamera()
        self.assertTrue(sim.isOpened())

    def test_release(self):
        sim = SimulatedCamera()
        sim.release()  # Should not raise

    def test_background_is_dark(self):
        """Frame with no plate or needle should be dark."""
        sim = SimulatedCamera(plate=None)
        sim.show_needle = False
        frame = sim.generate_frame()
        # Background should be near-black
        mean_brightness = frame.mean()
        self.assertLess(mean_brightness, 15)


class TestWellRendering(unittest.TestCase):
    """Test that wells appear as red circles."""

    def _make_sim_at_a1(self, um_per_px=20.0):
        """SimulatedCamera positioned directly over A1.

        Uses 20 µm/px (low magnification) so the 6.35mm well fits in the
        640×480 frame (FOV ≈ 12.8mm × 9.6mm, well ≈ 317px diameter).
        """
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=um_per_px,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.show_needle = False
        sim.set_stage_position(origin[0], origin[1], z_mm=0.0)
        return sim

    def test_well_visible_at_a1(self):
        """When positioned over A1, center of frame should be red."""
        sim = self._make_sim_at_a1()
        frame = sim.generate_frame()
        cx, cy = 320, 240
        # Red channel should be dominant at center
        b, g, r = frame[cy, cx]
        self.assertGreater(r, 100, f"Expected red at center, got BGR=({b},{g},{r})")
        self.assertGreater(r, b)
        self.assertGreater(r, g)

    def test_background_is_black_far_from_wells(self):
        """Area far from any well should be dark."""
        # At 20 µm/px, A1 well radius ≈ 159px. The corners at (0,0) are
        # ~400px from center — well beyond the well radius.
        sim = self._make_sim_at_a1()
        frame = sim.generate_frame()
        # Check top-left corner (far from A1 center)
        b, g, r = frame[0, 0]
        self.assertLess(r, 30, f"Corner should be dark, got BGR=({b},{g},{r})")

    def test_wells_translate_with_stage_movement(self):
        """Moving stage right should shift wells right out of frame center."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        # 20 µm/px: well ≈ 317px diameter, 1mm shift = 50px
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=20.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.show_needle = False

        # Centered on A1
        sim.set_stage_position(origin[0], origin[1])
        frame1 = sim.generate_frame()
        center_red_1 = int(frame1[240, 320, 2])  # Red channel at center

        # Move stage 5mm to the right (well shifts 250px left)
        sim.set_stage_position(origin[0] + 5000.0, origin[1])
        frame2 = sim.generate_frame()
        center_red_2 = int(frame2[240, 320, 2])

        # A1 should no longer be at center after 5mm shift
        self.assertGreater(center_red_1, center_red_2,
                          "Well should have moved away from center")

    def test_multiple_wells_visible(self):
        """With low magnification, multiple wells should be visible."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        # 20 µm/px → FOV ≈ 12.8mm × 9.6mm, 96-well spacing = 9mm
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=20.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.show_needle = False
        # Position between A1 and B2 to see multiple wells
        sim.set_stage_position(origin[0] + 4500.0, origin[1] + 4500.0)
        frame = sim.generate_frame()

        # Count red pixels (wells)
        red_mask = (frame[:, :, 2] > 100) & (frame[:, :, 1] < 50)
        red_pixel_count = red_mask.sum()
        self.assertGreater(red_pixel_count, 1000,
                          "Should see significant red area from multiple wells")


class TestNeedleRendering(unittest.TestCase):
    """Test needle appears as dark circle at frame center."""

    def test_needle_at_center_when_in_focus(self):
        """Needle should create a dark region at frame center over a well."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        # 3.34 µm/px: needle 910µm = 272px diameter, well fills entire frame
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=3.34,
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_stage_position(origin[0], origin[1], z_mm=0.0)
        sim.set_focal_z(0.0)
        frame = sim.generate_frame()

        cx, cy = 320, 240
        # Center should be dark (needle blocks red well below)
        brightness = frame[cy, cx].mean()
        self.assertLess(brightness, 30, "Needle center should be dark")

    def test_needle_hidden_when_disabled(self):
        """No needle should appear when show_needle is False."""
        sim = SimulatedCamera(
            resolution=(640, 480),
            plate=None,
        )
        sim.show_needle = False
        frame = sim.generate_frame()
        # Entire frame should be uniform background
        std = frame.std()
        self.assertLess(std, 5, "Frame should be uniform without needle")


class TestFocusSimulation(unittest.TestCase):
    """Test that needle blur varies with Z distance from focal plane."""

    def _make_needle_sim(self, z_mm: float, focal_z: float = 0.0):
        """Simulate needle over a well at microscope magnification.

        Uses 3.34 µm/px (2× objective) so needle 910µm = 272px.
        The well (6.35mm) fills the entire 640×480 frame — all bright red
        background, needle appears as a dark circle.
        """
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=3.34,
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_focal_z(focal_z)
        sim.set_stage_position(origin[0], origin[1], z_mm=z_mm)
        return sim.generate_frame()

    def test_in_focus_sharp_edge(self):
        """At focal plane, needle edge should be sharp (high gradient)."""
        frame = self._make_needle_sim(z_mm=0.0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F).var()
        self.assertGreater(laplacian, 1.0, "In-focus frame should have sharp edges")

    def test_out_of_focus_blurry(self):
        """Far from focal plane, needle should be blurry (low gradient)."""
        sharp_frame = self._make_needle_sim(z_mm=0.0)
        blurry_frame = self._make_needle_sim(z_mm=2.0)

        sharp_lap = cv2.Laplacian(
            cv2.cvtColor(sharp_frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        blurry_lap = cv2.Laplacian(
            cv2.cvtColor(blurry_frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()

        self.assertGreater(sharp_lap, blurry_lap,
                          f"In-focus ({sharp_lap:.1f}) should be sharper than "
                          f"out-of-focus ({blurry_lap:.1f})")

    def test_focus_decreases_with_large_defocus(self):
        """Focus score should clearly decrease with significant defocus."""
        scores = []
        # Use larger Z steps for clear differentiation
        for z in [0.0, 1.0, 3.0]:
            frame = self._make_needle_sim(z_mm=z)
            result = NeedleDetector.compute_focus_score(frame)
            scores.append(result.score)

        self.assertGreater(scores[0], scores[1],
                          f"In-focus ({scores[0]:.1f}) should beat 1mm defocus ({scores[1]:.1f})")
        self.assertGreater(scores[0], scores[2],
                          f"In-focus ({scores[0]:.1f}) should beat 3mm defocus ({scores[2]:.1f})")


class TestVisionDetectorIntegration(unittest.TestCase):
    """Test that VisionDetector algorithms work on simulated frames."""

    def test_well_detection_on_simulated_frame(self):
        """WellDetector should find the well in a simulated frame."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        # 20 µm/px: well 6.35mm = 317px diameter, fits in 640×480
        um_per_px = 20.0
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=um_per_px,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.show_needle = False
        sim.set_stage_position(origin[0], origin[1])

        frame = sim.generate_frame()
        expected_diameter_px = plate.well_diameter * 1000.0 / um_per_px

        result = WellDetector.detect_well_with_fallback(
            frame, expected_diameter_px=expected_diameter_px)

        self.assertIsNotNone(result, "Should detect the well in simulated frame")
        # Center should be near frame center
        cx, cy = result.center_px
        self.assertAlmostEqual(cx, 320, delta=50)
        self.assertAlmostEqual(cy, 240, delta=50)

    def test_needle_detection_on_simulated_frame(self):
        """NeedleDetector should find needle at microscope magnification.

        At 3.34 µm/px, needle 910µm = 272px and well fills entire frame.
        This simulates the real use case: bright field + dark needle.
        """
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        needle_od_um = 910.0
        um_per_px = 3.34
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=um_per_px,
            needle_od_um=needle_od_um,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_stage_position(origin[0], origin[1], z_mm=0.0)
        sim.set_focal_z(0.0)

        frame = sim.generate_frame()
        expected_od_px = needle_od_um / um_per_px

        result = NeedleDetector.detect_needle(frame, expected_od_px=expected_od_px)

        self.assertIsNotNone(result, "Should detect needle in simulated frame")
        cx, cy = result.center_px
        self.assertAlmostEqual(cx, 320, delta=40)
        self.assertAlmostEqual(cy, 240, delta=40)

    def test_focus_score_varies_with_z(self):
        """Focus score should track simulated defocus at microscope magnification."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=3.34,
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_stage_position(origin[0], origin[1])
        sim.set_focal_z(0.0)

        scores = []
        for z in [0.0, 1.0, 3.0]:
            sim.set_stage_position(origin[0], origin[1], z_mm=z)
            frame = sim.generate_frame()
            result = NeedleDetector.compute_focus_score(frame)
            scores.append(result.score)

        # In-focus should have highest score
        self.assertGreater(scores[0], scores[1],
                          f"In-focus score ({scores[0]:.1f}) should beat 1mm defocus ({scores[1]:.1f})")
        self.assertGreater(scores[0], scores[2],
                          f"In-focus score ({scores[0]:.1f}) should beat 3mm defocus ({scores[2]:.1f})")


class TestGaussianDOFModel(unittest.TestCase):
    """Test the Gaussian depth-of-field envelope per objective magnification."""

    def test_needle_invisible_at_1mm_defocus_4x(self):
        """At 4× (1.67 µm/px), 1mm defocus should make needle invisible."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=1.67,  # 4× objective
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_focal_z(0.0)

        # DOF half-width at 4× should be ~0.10mm
        self.assertAlmostEqual(sim.dof_halfwidth_mm, 0.1002, places=3)

        # At 1mm defocus: exp(-0.5 * (1.0/0.1)^2) ≈ 0.0 — below visibility cutoff
        sim.set_stage_position(origin[0], origin[1], z_mm=1.0)
        frame_defocused = sim.generate_frame()

        # Without needle for comparison
        sim.show_needle = False
        sim.set_stage_position(origin[0], origin[1], z_mm=1.0)
        frame_no_needle = sim.generate_frame()

        # Frames should be nearly identical (needle invisible)
        diff = np.abs(frame_defocused.astype(float) - frame_no_needle.astype(float))
        max_diff = diff.max()
        self.assertLess(max_diff, 2.0,
                       f"Needle should be invisible at 1mm defocus on 4×, max diff={max_diff}")

    def test_needle_visible_at_small_defocus_4x(self):
        """At 4× (1.67 µm/px), 0.05mm defocus should keep needle visible."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=1.67,
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_focal_z(0.0)
        sim.set_stage_position(origin[0], origin[1], z_mm=0.05)
        frame = sim.generate_frame()

        cx, cy = 320, 240
        brightness = frame[cy, cx].mean()
        # Needle should still darken the center noticeably
        self.assertLess(brightness, 100,
                       f"Needle should be visible at 0.05mm defocus, brightness={brightness}")

    def test_dof_varies_by_magnification(self):
        """Different µm/px values should produce different DOF half-widths."""
        sim_2x = SimulatedCamera(micron_per_pixel=3.34)   # 2×
        sim_4x = SimulatedCamera(micron_per_pixel=1.67)   # 4×
        sim_10x = SimulatedCamera(micron_per_pixel=0.668)  # 10×
        sim_20x = SimulatedCamera(micron_per_pixel=0.334)  # 20×

        # Higher magnification = narrower DOF
        self.assertGreater(sim_2x.dof_halfwidth_mm, sim_4x.dof_halfwidth_mm)
        self.assertGreater(sim_4x.dof_halfwidth_mm, sim_10x.dof_halfwidth_mm)
        self.assertGreater(sim_10x.dof_halfwidth_mm, sim_20x.dof_halfwidth_mm)

        # Check approximate values
        self.assertAlmostEqual(sim_2x.dof_halfwidth_mm, 0.200, places=2)
        self.assertAlmostEqual(sim_4x.dof_halfwidth_mm, 0.100, places=2)
        self.assertAlmostEqual(sim_10x.dof_halfwidth_mm, 0.040, places=2)
        self.assertAlmostEqual(sim_20x.dof_halfwidth_mm, 0.020, places=2)

    def test_needle_invisible_at_smaller_defocus_for_20x(self):
        """20× has very narrow DOF — needle invisible at 0.2mm defocus."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=0.334,  # 20× objective
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_focal_z(0.0)

        # DOF half-width at 20× is ~0.02mm
        # At 0.2mm defocus: exp(-0.5 * (0.2/0.02)^2) = exp(-50) ≈ 0.0
        sim.set_stage_position(origin[0], origin[1], z_mm=0.2)
        frame_defocused = sim.generate_frame()

        sim.show_needle = False
        frame_no_needle = sim.generate_frame()

        diff = np.abs(frame_defocused.astype(float) - frame_no_needle.astype(float))
        max_diff = diff.max()
        self.assertLess(max_diff, 2.0,
                       f"Needle should be invisible at 0.2mm defocus on 20×, max diff={max_diff}")

    def test_2x_needle_still_visible_at_moderate_defocus(self):
        """2× has wide DOF — needle should still be visible at 0.2mm defocus."""
        plate = WellPlate.from_format(96)
        origin = (50000.0, 50000.0)
        sim = SimulatedCamera(
            resolution=(640, 480),
            micron_per_pixel=3.34,  # 2× objective, DOF ~0.20mm
            needle_od_um=910.0,
            plate=plate,
            plate_origin_um=origin,
        )
        sim.set_focal_z(0.0)
        sim.set_stage_position(origin[0], origin[1], z_mm=0.2)
        frame = sim.generate_frame()

        # At 0.2mm defocus with 0.20mm DOF half-width:
        # opacity = exp(-0.5 * (0.2/0.2)^2) = exp(-0.5) ≈ 0.61
        # Needle should still be clearly visible
        cx, cy = 320, 240
        brightness = frame[cy, cx].mean()
        self.assertLess(brightness, 120,
                       f"Needle should darken center at 0.2mm defocus on 2×, brightness={brightness}")


if __name__ == "__main__":
    unittest.main()
