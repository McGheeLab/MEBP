"""
Tests for MosaicBuilder — v7.3.1 Phase 2.

Tests affine fitting accuracy, position correction, mosaic stitching,
and edge cases (few points, identity transform, large rotation).
"""

import math
import unittest

import numpy as np

from SupportClasses.MosaicBuilder import (
    MosaicBuilder,
    AffineCalibration,
    FrameRecord,
    MosaicResult,
)
from SupportClasses.WellPlate import WellPlate


class _FakeDetection:
    """Minimal detection result stub."""
    def __init__(self, center_px, confidence=0.8):
        self.center_px = center_px
        self.confidence = confidence


class TestAffineCalibration(unittest.TestCase):
    """Test the AffineCalibration dataclass independently."""

    def test_identity(self):
        cal = AffineCalibration()
        self.assertTrue(cal.is_identity)
        x, y = cal.correct_position(1000.0, 2000.0)
        self.assertAlmostEqual(x, 1000.0, places=1)
        self.assertAlmostEqual(y, 2000.0, places=1)

    def test_pure_translation(self):
        cal = AffineCalibration(
            translation_um=(100.0, -50.0),
            center_um=(0.0, 0.0),
        )
        x, y = cal.correct_position(1000.0, 2000.0)
        self.assertAlmostEqual(x, 1100.0, places=1)
        self.assertAlmostEqual(y, 1950.0, places=1)

    def test_pure_rotation_90(self):
        cal = AffineCalibration(
            rotation_deg=90.0,
            scale=1.0,
            center_um=(0.0, 0.0),
        )
        x, y = cal.correct_position(1000.0, 0.0)
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertAlmostEqual(y, 1000.0, places=1)

    def test_scale_2x(self):
        cal = AffineCalibration(
            scale=2.0,
            center_um=(0.0, 0.0),
        )
        x, y = cal.correct_position(100.0, 200.0)
        self.assertAlmostEqual(x, 200.0, places=1)
        self.assertAlmostEqual(y, 400.0, places=1)

    def test_correct_positions_dict(self):
        cal = AffineCalibration(
            translation_um=(10.0, 20.0),
            center_um=(0.0, 0.0),
        )
        predicted = {"A1": (100.0, 200.0), "A2": (300.0, 400.0)}
        corrected = cal.correct_positions(predicted)
        self.assertAlmostEqual(corrected["A1"][0], 110.0, places=1)
        self.assertAlmostEqual(corrected["A1"][1], 220.0, places=1)
        self.assertAlmostEqual(corrected["A2"][0], 310.0, places=1)
        self.assertAlmostEqual(corrected["A2"][1], 420.0, places=1)

    def test_is_identity_false(self):
        cal = AffineCalibration(rotation_deg=1.5)
        self.assertFalse(cal.is_identity)


class TestMosaicBuilderFrameCollection(unittest.TestCase):
    """Test frame addition and metadata tracking."""

    def setUp(self):
        self.builder = MosaicBuilder(
            frame_size_px=(100, 80),
            micron_per_pixel=5.0,
        )

    def test_add_frame_no_detection(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        rec = self.builder.add_frame("A1", frame, 50000.0, 40000.0)
        self.assertEqual(rec.well_name, "A1")
        self.assertIsNone(rec.detected_offset_um)
        self.assertEqual(self.builder.frame_count, 1)

    def test_add_frame_with_detection(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        det = _FakeDetection(center_px=(55.0, 42.0), confidence=0.9)
        rec = self.builder.add_frame("A1", frame, 50000.0, 40000.0, det)
        # Center is at (50, 40), detected at (55, 42) → offset (5, 2) px → (25, 10) µm
        self.assertIsNotNone(rec.detected_offset_um)
        self.assertAlmostEqual(rec.detected_offset_um[0], 25.0, places=1)
        self.assertAlmostEqual(rec.detected_offset_um[1], 10.0, places=1)
        self.assertAlmostEqual(rec.confidence, 0.9)

    def test_reset(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        self.builder.add_frame("A1", frame, 0.0, 0.0)
        self.assertEqual(self.builder.frame_count, 1)
        self.builder.reset()
        self.assertEqual(self.builder.frame_count, 0)


class TestAffineFitting(unittest.TestCase):
    """Test affine fitting with synthetic data."""

    def _make_builder_with_known_transform(
        self, rotation_deg=0.0, scale=1.0, translation_um=(0.0, 0.0), noise_um=0.0
    ):
        """Create a builder with synthetic frames that simulate a known transform.

        We create predicted positions from a 96-well plate, apply a known
        transform to get 'true' positions, then simulate detection offsets
        such that the detected absolute positions equal the true positions.
        """
        plate = WellPlate.from_format(96)
        a1_x, a1_y = 50000.0, 40000.0
        predicted = plate.get_all_positions_from_a1(a1_x, a1_y)

        # Apply known transform to predicted → true positions
        rad = math.radians(rotation_deg)
        cos_r = math.cos(rad) * scale
        sin_r = math.sin(rad) * scale
        tx, ty = translation_um

        # Rotation center = centroid of predicted
        cx = np.mean([p[0] for p in predicted.values()])
        cy = np.mean([p[1] for p in predicted.values()])

        rng = np.random.RandomState(42)
        builder = MosaicBuilder(frame_size_px=(100, 80), micron_per_pixel=5.0)

        for name, (px, py) in predicted.items():
            # True position after transform
            dx = px - cx
            dy = py - cy
            true_x = cos_r * dx - sin_r * dy + cx + tx
            true_y = sin_r * dx + cos_r * dy + cy + ty

            if noise_um > 0:
                true_x += rng.normal(0, noise_um)
                true_y += rng.normal(0, noise_um)

            # Simulate: stage moves to predicted position, detection finds offset
            # detected_absolute = stage_pos + offset
            # We want detected_absolute = true_pos
            # So offset = true_pos - stage_pos = true_pos - predicted_pos
            offset_x_um = true_x - px
            offset_y_um = true_y - py

            # Convert offset µm → pixel offset from frame center
            offset_x_px = offset_x_um / 5.0
            offset_y_px = offset_y_um / 5.0

            # Detection center = frame center + offset
            det_cx = 50.0 + offset_x_px
            det_cy = 40.0 + offset_y_px

            frame = np.zeros((80, 100, 3), dtype=np.uint8)
            det = _FakeDetection(center_px=(det_cx, det_cy), confidence=0.9)
            builder.add_frame(name, frame, px, py, det)

        return builder, predicted

    def test_identity_transform(self):
        """No rotation/scale/translation should recover identity."""
        builder, predicted = self._make_builder_with_known_transform()
        cal = builder.fit_affine(predicted)
        self.assertAlmostEqual(cal.rotation_deg, 0.0, places=2)
        self.assertAlmostEqual(cal.scale, 1.0, places=4)
        self.assertAlmostEqual(cal.translation_um[0], 0.0, places=1)
        self.assertAlmostEqual(cal.translation_um[1], 0.0, places=1)
        self.assertLess(cal.residual_um, 1.0)

    def test_rotation_2deg(self):
        """Detect 2-degree rotation."""
        builder, predicted = self._make_builder_with_known_transform(rotation_deg=2.0)
        cal = builder.fit_affine(predicted)
        self.assertAlmostEqual(cal.rotation_deg, 2.0, places=1)
        self.assertAlmostEqual(cal.scale, 1.0, places=3)
        self.assertLess(cal.residual_um, 1.0)

    def test_rotation_negative(self):
        """Detect negative rotation."""
        builder, predicted = self._make_builder_with_known_transform(rotation_deg=-1.5)
        cal = builder.fit_affine(predicted)
        self.assertAlmostEqual(cal.rotation_deg, -1.5, places=1)

    def test_scale_102(self):
        """Detect 2% scale change."""
        builder, predicted = self._make_builder_with_known_transform(scale=1.02)
        cal = builder.fit_affine(predicted)
        self.assertAlmostEqual(cal.scale, 1.02, places=3)

    def test_translation_only(self):
        """Pure translation."""
        builder, predicted = self._make_builder_with_known_transform(
            translation_um=(150.0, -80.0)
        )
        cal = builder.fit_affine(predicted)
        self.assertAlmostEqual(cal.translation_um[0], 150.0, places=0)
        self.assertAlmostEqual(cal.translation_um[1], -80.0, places=0)
        self.assertAlmostEqual(cal.rotation_deg, 0.0, places=2)

    def test_combined_transform(self):
        """Rotation + scale + translation together."""
        builder, predicted = self._make_builder_with_known_transform(
            rotation_deg=1.0, scale=1.01, translation_um=(50.0, -30.0)
        )
        cal = builder.fit_affine(predicted)
        self.assertAlmostEqual(cal.rotation_deg, 1.0, places=1)
        self.assertAlmostEqual(cal.scale, 1.01, places=2)
        self.assertLess(cal.residual_um, 5.0)

    def test_noisy_detection(self):
        """Fit still reasonable with noise."""
        builder, predicted = self._make_builder_with_known_transform(
            rotation_deg=1.5, noise_um=20.0
        )
        cal = builder.fit_affine(predicted)
        # Should be within ~0.5 deg of true
        self.assertAlmostEqual(cal.rotation_deg, 1.5, delta=0.5)
        self.assertEqual(cal.num_points, 96)

    def test_correction_roundtrip(self):
        """Apply correction to predicted positions, verify they match 'true' positions."""
        builder, predicted = self._make_builder_with_known_transform(
            rotation_deg=1.5, scale=1.005
        )
        result = builder.build(predicted)
        corrected = result.correct_positions(predicted)

        # Re-compute true positions for comparison
        rad = math.radians(1.5)
        cos_r = math.cos(rad) * 1.005
        sin_r = math.sin(rad) * 1.005
        cx = np.mean([p[0] for p in predicted.values()])
        cy = np.mean([p[1] for p in predicted.values()])

        max_err = 0.0
        for name, (px, py) in predicted.items():
            dx = px - cx
            dy = py - cy
            true_x = cos_r * dx - sin_r * dy + cx
            true_y = sin_r * dx + cos_r * dy + cy
            err = math.sqrt((corrected[name][0] - true_x)**2 +
                            (corrected[name][1] - true_y)**2)
            max_err = max(max_err, err)

        # Corrected positions should be within 5 µm of truth
        self.assertLess(max_err, 5.0,
                        f"Max correction error {max_err:.1f} µm exceeds 5 µm threshold")

    def test_too_few_points(self):
        """With < 2 detected points, returns identity."""
        builder = MosaicBuilder(frame_size_px=(100, 80), micron_per_pixel=5.0)
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        builder.add_frame("A1", frame, 50000.0, 40000.0)  # No detection
        predicted = {"A1": (50000.0, 40000.0)}
        cal = builder.fit_affine(predicted)
        self.assertTrue(cal.is_identity)
        self.assertEqual(cal.num_points, 0)

    def test_confidence_filtering(self):
        """Low-confidence detections are excluded from fit."""
        builder = MosaicBuilder(frame_size_px=(100, 80), micron_per_pixel=5.0)
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        # Add 2 low-confidence detections
        det_low = _FakeDetection(center_px=(50.0, 40.0), confidence=0.1)
        builder.add_frame("A1", frame, 50000.0, 40000.0, det_low)
        builder.add_frame("A2", frame, 59000.0, 40000.0, det_low)
        predicted = {"A1": (50000.0, 40000.0), "A2": (59000.0, 40000.0)}
        cal = builder.fit_affine(predicted, min_confidence=0.3)
        self.assertEqual(cal.num_points, 0)


class TestMosaicBuild(unittest.TestCase):
    """Test the high-level build() method."""

    def test_build_returns_result(self):
        plate = WellPlate.from_format(6)
        predicted = plate.get_all_positions_from_a1(50000.0, 40000.0)
        builder = MosaicBuilder(frame_size_px=(100, 80), micron_per_pixel=5.0)

        for name in plate.meander_order():
            frame = np.zeros((80, 100, 3), dtype=np.uint8)
            px, py = predicted[name]
            det = _FakeDetection(center_px=(50.0, 40.0), confidence=0.9)
            builder.add_frame(name, frame, px, py, det)

        result = builder.build(predicted)
        self.assertIsInstance(result, MosaicResult)
        self.assertEqual(result.frames_captured, 6)
        self.assertEqual(result.frames_detected, 6)
        self.assertIsNotNone(result.mosaic_image)
        self.assertIsNotNone(result.calibration)

    def test_build_no_predictions(self):
        """Build without predictions should still produce mosaic."""
        builder = MosaicBuilder(frame_size_px=(100, 80), micron_per_pixel=5.0)
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        builder.add_frame("A1", frame, 50000.0, 40000.0)
        result = builder.build()
        self.assertIsNotNone(result.mosaic_image)
        self.assertTrue(result.calibration.is_identity)


class TestMosaicStitching(unittest.TestCase):
    """Test mosaic image stitching specifically."""

    def test_mosaic_not_all_black(self):
        """Mosaic with non-black frames should have non-zero pixels."""
        builder = MosaicBuilder(frame_size_px=(100, 80), micron_per_pixel=5.0)
        # White frame
        frame = np.ones((80, 100, 3), dtype=np.uint8) * 255
        builder.add_frame("A1", frame, 0.0, 0.0)
        builder.add_frame("A2", frame, 500.0, 0.0)
        mosaic = builder.build_mosaic()
        self.assertIsNotNone(mosaic)
        self.assertGreater(mosaic.sum(), 0)

    def test_empty_builder_returns_none(self):
        builder = MosaicBuilder()
        self.assertIsNone(builder.build_mosaic())


if __name__ == "__main__":
    unittest.main()
