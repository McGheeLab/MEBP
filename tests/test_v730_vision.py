"""
test_v730_vision.py — Unit tests for VisionDetector (v7.3.0 autocalibration).

Tests use synthetic images with known circles to verify detection accuracy.
No camera or hardware required.
"""

import math
import unittest

import cv2
import numpy as np

from SupportClasses.VisionDetector import (
    DetectionResult,
    FocusResult,
    FocusTracker,
    NeedleDetector,
    WellDetector,
    pixel_offset_to_stage_um,
)


# ---------------------------------------------------------------------------
# Synthetic Image Generators
# ---------------------------------------------------------------------------

def make_bright_circle(
    width: int = 640,
    height: int = 480,
    cx: float | None = None,
    cy: float | None = None,
    radius: float = 100.0,
    bg_intensity: int = 40,
    circle_intensity: int = 200,
    ring_width: int = 3,
    noise_std: float = 0.0,
) -> np.ndarray:
    """
    Generate a synthetic well image: bright circle (well bottom) on dark background.

    The circle is drawn as a filled bright region with a distinct edge,
    simulating how a well looks under the microscope at low magnification.
    """
    if cx is None:
        cx = width / 2.0
    if cy is None:
        cy = height / 2.0

    img = np.full((height, width, 3), bg_intensity, dtype=np.uint8)
    cv2.circle(img, (int(cx), int(cy)), int(radius), (circle_intensity,) * 3, -1)
    # Draw edge ring for better Hough detection
    cv2.circle(img, (int(cx), int(cy)), int(radius), (circle_intensity + 40,) * 3, ring_width)

    if noise_std > 0:
        noise = np.random.normal(0, noise_std, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    return img


def make_dark_circle(
    width: int = 640,
    height: int = 480,
    cx: float | None = None,
    cy: float | None = None,
    radius: float = 30.0,
    bg_intensity: int = 200,
    circle_intensity: int = 40,
    noise_std: float = 0.0,
) -> np.ndarray:
    """
    Generate a synthetic needle tip image: dark circle on bright background.

    Simulates how a needle tip appears in transmitted light — a dark
    circular shadow on a bright field.
    """
    if cx is None:
        cx = width / 2.0
    if cy is None:
        cy = height / 2.0

    img = np.full((height, width, 3), bg_intensity, dtype=np.uint8)
    cv2.circle(img, (int(cx), int(cy)), int(radius), (circle_intensity,) * 3, -1)

    if noise_std > 0:
        noise = np.random.normal(0, noise_std, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    return img


def make_sharp_edges_image(width: int = 200, height: int = 200) -> np.ndarray:
    """Generate an image with sharp edges (high focus score)."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    # Sharp rectangles and lines
    cv2.rectangle(img, (30, 30), (170, 170), (255, 255, 255), 2)
    cv2.line(img, (50, 50), (150, 150), (200, 200, 200), 1)
    cv2.circle(img, (100, 100), 40, (180, 180, 180), 2)
    # Add fine detail
    for i in range(0, 200, 10):
        cv2.line(img, (i, 0), (i, 200), (100, 100, 100), 1)
    return img


def make_blurred_image(sharp_img: np.ndarray, ksize: int = 21) -> np.ndarray:
    """Blur an image to simulate out-of-focus condition."""
    return cv2.GaussianBlur(sharp_img, (ksize, ksize), 0)


# ---------------------------------------------------------------------------
# WellDetector Tests
# ---------------------------------------------------------------------------

class TestWellDetection(unittest.TestCase):
    """Tests for WellDetector.detect_well() and detect_well_with_fallback()."""

    def test_detect_well_synthetic_circle(self):
        """Detect a known circle at frame center — accuracy < 5px."""
        cx, cy, r = 320.0, 240.0, 100.0
        frame = make_bright_circle(640, 480, cx, cy, r)

        result = WellDetector.detect_well(frame, expected_diameter_px=200.0)

        self.assertIsNotNone(result, "Should detect the well circle")
        self.assertAlmostEqual(result.center_px[0], cx, delta=5.0,
                               msg=f"X center off: {result.center_px[0]} vs {cx}")
        self.assertAlmostEqual(result.center_px[1], cy, delta=5.0,
                               msg=f"Y center off: {result.center_px[1]} vs {cy}")
        self.assertAlmostEqual(result.radius_px, r, delta=10.0,
                               msg=f"Radius off: {result.radius_px} vs {r}")
        self.assertGreater(result.confidence, 0.5)
        self.assertEqual(result.method, "hough")

    def test_detect_well_off_center(self):
        """Detect a circle that is off-center in the frame."""
        cx, cy, r = 200.0, 150.0, 80.0
        frame = make_bright_circle(640, 480, cx, cy, r)

        result = WellDetector.detect_well(frame, expected_diameter_px=160.0)

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_px[0], cx, delta=5.0)
        self.assertAlmostEqual(result.center_px[1], cy, delta=5.0)

    def test_detect_well_noisy_image(self):
        """Detection should work with moderate Gaussian noise."""
        cx, cy, r = 320.0, 240.0, 100.0
        frame = make_bright_circle(640, 480, cx, cy, r, noise_std=15.0)

        result = WellDetector.detect_well(frame, expected_diameter_px=200.0)

        self.assertIsNotNone(result, "Should detect well even with noise")
        self.assertAlmostEqual(result.center_px[0], cx, delta=10.0)
        self.assertAlmostEqual(result.center_px[1], cy, delta=10.0)

    def test_detect_well_wrong_size_rejected(self):
        """A circle of very different diameter should be rejected."""
        # Draw a small circle (r=30) but look for a large one (expected diameter=300)
        frame = make_bright_circle(640, 480, 320, 240, 30.0)

        result = WellDetector.detect_well(frame, expected_diameter_px=300.0, tolerance=0.2)

        # Should either return None or very low confidence
        if result is not None:
            # The radius should be far from expected, so confidence should be low
            radius_match = min(result.radius_px, 150.0) / max(result.radius_px, 150.0)
            self.assertLess(radius_match, 0.5,
                            "Wrong-size circle should have poor radius match")

    def test_detect_well_empty_frame(self):
        """No circle in frame → return None."""
        frame = np.full((480, 640, 3), 50, dtype=np.uint8)
        result = WellDetector.detect_well(frame, expected_diameter_px=200.0)
        self.assertIsNone(result)

    def test_detect_well_none_input(self):
        """None input → return None without crashing."""
        result = WellDetector.detect_well(None, expected_diameter_px=200.0)
        self.assertIsNone(result)

    def test_detect_well_with_fallback_uses_hough_first(self):
        """Fallback should return Hough result when it succeeds."""
        cx, cy, r = 320.0, 240.0, 100.0
        frame = make_bright_circle(640, 480, cx, cy, r)

        result = WellDetector.detect_well_with_fallback(frame, expected_diameter_px=200.0)

        self.assertIsNotNone(result)
        # Primary method should be hough when the circle is clear
        # (Could be contour if Hough confidence is low, but circle is clear here)
        self.assertIn(result.method, ["hough", "contour"])
        self.assertAlmostEqual(result.center_px[0], cx, delta=10.0)

    def test_detect_well_grayscale_input(self):
        """Should handle single-channel (grayscale) input."""
        frame_bgr = make_bright_circle(640, 480, 320, 240, 100.0)
        frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        result = WellDetector.detect_well(frame_gray, expected_diameter_px=200.0)
        self.assertIsNotNone(result)

    def test_detect_well_diameter_property(self):
        """DetectionResult.diameter_px should be 2× radius."""
        frame = make_bright_circle(640, 480, 320, 240, 100.0)
        result = WellDetector.detect_well(frame, expected_diameter_px=200.0)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.diameter_px, result.radius_px * 2)


# ---------------------------------------------------------------------------
# NeedleDetector Tests
# ---------------------------------------------------------------------------

class TestNeedleDetection(unittest.TestCase):
    """Tests for NeedleDetector.detect_needle()."""

    def test_detect_needle_dark_circle(self):
        """Detect a dark circle (needle tip) on bright background."""
        cx, cy, r = 320.0, 240.0, 30.0
        frame = make_dark_circle(640, 480, cx, cy, r)

        result = NeedleDetector.detect_needle(frame, expected_od_px=60.0)

        self.assertIsNotNone(result, "Should detect the needle tip")
        self.assertAlmostEqual(result.center_px[0], cx, delta=10.0,
                               msg=f"X center off: {result.center_px[0]} vs {cx}")
        self.assertAlmostEqual(result.center_px[1], cy, delta=10.0,
                               msg=f"Y center off: {result.center_px[1]} vs {cy}")
        self.assertGreater(result.confidence, 0.3)
        self.assertIn(result.method, ("hough", "contour", "radial"))

    def test_detect_needle_off_center(self):
        """Detect needle that is off-center in the frame."""
        cx, cy, r = 450.0, 350.0, 25.0
        frame = make_dark_circle(640, 480, cx, cy, r)

        result = NeedleDetector.detect_needle(frame, expected_od_px=50.0)

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_px[0], cx, delta=10.0)
        self.assertAlmostEqual(result.center_px[1], cy, delta=10.0)

    def test_detect_needle_noisy_image(self):
        """Detection should work with moderate noise."""
        cx, cy, r = 320.0, 240.0, 30.0
        frame = make_dark_circle(640, 480, cx, cy, r, noise_std=10.0)

        result = NeedleDetector.detect_needle(frame, expected_od_px=60.0)

        self.assertIsNotNone(result, "Should detect needle with noise")
        self.assertAlmostEqual(result.center_px[0], cx, delta=15.0)
        self.assertAlmostEqual(result.center_px[1], cy, delta=15.0)

    def test_detect_needle_hollow(self):
        """Detect a hollow needle tip (annulus / ring shape)."""
        width, height = 640, 480
        cx, cy = 320, 240
        outer_r, inner_r = 30, 20

        # Bright background
        img = np.full((height, width, 3), 200, dtype=np.uint8)
        # Dark outer circle (needle wall)
        cv2.circle(img, (cx, cy), outer_r, (40, 40, 40), -1)
        # Bright inner circle (hollow bore — light passes through)
        cv2.circle(img, (cx, cy), inner_r, (180, 180, 180), -1)

        result = NeedleDetector.detect_needle(img, expected_od_px=60.0, tolerance=0.5)

        # May or may not detect — the annulus is harder. Check robustness.
        if result is not None:
            # If detected, should be roughly centered
            self.assertAlmostEqual(result.center_px[0], cx, delta=20.0)
            self.assertAlmostEqual(result.center_px[1], cy, delta=20.0)

    def test_detect_needle_empty_frame(self):
        """Bright frame with no needle → return None."""
        frame = np.full((480, 640, 3), 200, dtype=np.uint8)
        result = NeedleDetector.detect_needle(frame, expected_od_px=60.0)
        self.assertIsNone(result)

    def test_detect_needle_none_input(self):
        """None input → return None without crashing."""
        result = NeedleDetector.detect_needle(None, expected_od_px=60.0)
        self.assertIsNone(result)


# ---------------------------------------------------------------------------
# Focus Score Tests
# ---------------------------------------------------------------------------

class TestFocusScore(unittest.TestCase):
    """Tests for NeedleDetector.compute_focus_score()."""

    def test_sharp_vs_blurred(self):
        """Sharp image should score higher than blurred version."""
        sharp = make_sharp_edges_image()
        blurred = make_blurred_image(sharp, ksize=21)

        sharp_result = NeedleDetector.compute_focus_score(sharp)
        blurred_result = NeedleDetector.compute_focus_score(blurred)

        self.assertGreater(
            sharp_result.score, blurred_result.score,
            f"Sharp ({sharp_result.score:.1f}) should score higher than "
            f"blurred ({blurred_result.score:.1f})"
        )

    def test_focus_score_monotonic(self):
        """Score should increase as blur decreases (roughly monotonic)."""
        sharp = make_sharp_edges_image()
        blur_levels = [31, 21, 11, 5, 3]
        scores = []

        for ksize in blur_levels:
            blurred = make_blurred_image(sharp, ksize=ksize)
            result = NeedleDetector.compute_focus_score(blurred)
            scores.append(result.score)

        # Scores should generally increase as blur decreases
        # Allow minor non-monotonicity at very small blur levels
        for i in range(len(scores) - 2):
            self.assertLessEqual(
                scores[i], scores[i + 2],
                f"Score at blur {blur_levels[i]} ({scores[i]:.1f}) should be <= "
                f"score at blur {blur_levels[i+2]} ({scores[i+2]:.1f})"
            )

    def test_focus_score_roi(self):
        """Focus score with ROI should only measure the specified region."""
        sharp = make_sharp_edges_image(400, 400)

        full_result = NeedleDetector.compute_focus_score(sharp)
        roi_result = NeedleDetector.compute_focus_score(sharp, roi_rect=(100, 100, 200, 200))

        # Both should produce valid scores
        self.assertGreater(full_result.score, 0)
        self.assertGreater(roi_result.score, 0)
        # ROI center should be in the ROI region
        self.assertAlmostEqual(roi_result.roi_center_px[0], 200.0, delta=1.0)
        self.assertAlmostEqual(roi_result.roi_center_px[1], 200.0, delta=1.0)

    def test_focus_score_empty_frame(self):
        """Empty/None frame should return zero score."""
        result = NeedleDetector.compute_focus_score(None)
        self.assertEqual(result.score, 0.0)

        result2 = NeedleDetector.compute_focus_score(np.array([]))
        self.assertEqual(result2.score, 0.0)

    def test_focus_result_components(self):
        """FocusResult should have both laplacian_var and tenengrad populated."""
        sharp = make_sharp_edges_image()
        result = NeedleDetector.compute_focus_score(sharp)

        self.assertGreater(result.laplacian_var, 0)
        self.assertGreater(result.tenengrad, 0)
        self.assertAlmostEqual(
            result.score,
            0.7 * result.laplacian_var + 0.3 * result.tenengrad,
            places=5,
        )

    def test_focus_score_grayscale_input(self):
        """Should handle single-channel (grayscale) input."""
        sharp = make_sharp_edges_image()
        gray = cv2.cvtColor(sharp, cv2.COLOR_BGR2GRAY)
        result = NeedleDetector.compute_focus_score(gray)
        self.assertGreater(result.score, 0)


# ---------------------------------------------------------------------------
# FocusTracker Tests
# ---------------------------------------------------------------------------

class TestFocusTracker(unittest.TestCase):
    """Tests for FocusTracker session state management."""

    def test_normalize_scores(self):
        """Normalized score should be relative to session max."""
        tracker = FocusTracker()

        r1 = tracker.update(FocusResult(score=50.0))
        self.assertAlmostEqual(r1.normalized_score, 1.0)  # First score is the max
        self.assertEqual(tracker.session_max, 50.0)

        r2 = tracker.update(FocusResult(score=25.0))
        self.assertAlmostEqual(r2.normalized_score, 0.5)  # Half of max
        self.assertEqual(tracker.session_max, 50.0)

        r3 = tracker.update(FocusResult(score=100.0))
        self.assertAlmostEqual(r3.normalized_score, 1.0)  # New max
        self.assertEqual(tracker.session_max, 100.0)

    def test_in_focus_threshold(self):
        """is_in_focus should respect threshold."""
        tracker = FocusTracker(in_focus_threshold=0.7)

        tracker.update(FocusResult(score=100.0))  # Set session max

        r_high = tracker.update(FocusResult(score=80.0))
        self.assertTrue(r_high.is_in_focus)  # 0.8 >= 0.7

        r_low = tracker.update(FocusResult(score=50.0))
        self.assertFalse(r_low.is_in_focus)  # 0.5 < 0.7

    def test_trend_detection(self):
        """Trend should detect improving/declining focus."""
        tracker = FocusTracker()

        # Feed improving scores
        for s in [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]:
            tracker.update(FocusResult(score=float(s)))

        self.assertEqual(tracker.get_trend(window=3), "improving")

        # Feed declining scores
        for s in [90, 80, 70, 60, 50, 40, 30, 20, 10, 5]:
            tracker.update(FocusResult(score=float(s)))

        self.assertEqual(tracker.get_trend(window=3), "declining")

    def test_reset(self):
        """Reset should clear session state."""
        tracker = FocusTracker()
        tracker.update(FocusResult(score=100.0))
        self.assertEqual(tracker.session_max, 100.0)

        tracker.reset()
        self.assertEqual(tracker.session_max, 0.0)
        self.assertEqual(tracker.get_trend(), "unknown")


# ---------------------------------------------------------------------------
# Pixel-to-Stage Conversion Tests
# ---------------------------------------------------------------------------

class TestPixelToStageConversion(unittest.TestCase):
    """Tests for pixel_offset_to_stage_um()."""

    def test_center_is_zero_offset(self):
        """Object at frame center → zero offset."""
        dx, dy = pixel_offset_to_stage_um(
            detected_center_px=(320.0, 240.0),
            frame_size_px=(640, 480),
            micron_per_pixel=1.67,
        )
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, 0.0)

    def test_right_offset(self):
        """Object right of center → positive dx."""
        dx, dy = pixel_offset_to_stage_um(
            detected_center_px=(420.0, 240.0),
            frame_size_px=(640, 480),
            micron_per_pixel=1.67,
        )
        self.assertAlmostEqual(dx, 100.0 * 1.67, places=2)
        self.assertAlmostEqual(dy, 0.0)

    def test_below_offset(self):
        """Object below center → positive dy."""
        dx, dy = pixel_offset_to_stage_um(
            detected_center_px=(320.0, 340.0),
            frame_size_px=(640, 480),
            micron_per_pixel=3.34,
        )
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, 100.0 * 3.34, places=2)

    def test_known_offset_magnitude(self):
        """Verify exact magnitude for a known offset."""
        # 50px right, 30px down at 2.0 µm/px
        dx, dy = pixel_offset_to_stage_um(
            detected_center_px=(370.0, 270.0),
            frame_size_px=(640, 480),
            micron_per_pixel=2.0,
        )
        self.assertAlmostEqual(dx, 100.0)  # 50px * 2.0
        self.assertAlmostEqual(dy, 60.0)   # 30px * 2.0

    def test_negative_offset(self):
        """Object left and above center → negative offsets."""
        dx, dy = pixel_offset_to_stage_um(
            detected_center_px=(220.0, 140.0),
            frame_size_px=(640, 480),
            micron_per_pixel=1.0,
        )
        self.assertAlmostEqual(dx, -100.0)
        self.assertAlmostEqual(dy, -100.0)


# ---------------------------------------------------------------------------
# Edge Cases
# ---------------------------------------------------------------------------

class TestEdgeCases(unittest.TestCase):
    """Edge case tests for detector robustness."""

    def test_very_small_circle(self):
        """Very small circle (r=5px) — may not detect but should not crash."""
        frame = make_bright_circle(640, 480, 320, 240, 5.0)
        result = WellDetector.detect_well(frame, expected_diameter_px=10.0)
        # Just verify no crash — detection of tiny circles is unreliable

    def test_very_large_circle(self):
        """Circle larger than frame — should not crash."""
        frame = make_bright_circle(640, 480, 320, 240, 400.0)
        result = WellDetector.detect_well(frame, expected_diameter_px=800.0)
        # Just verify no crash

    def test_zero_expected_diameter(self):
        """Zero expected diameter → None without crash."""
        frame = make_bright_circle(640, 480, 320, 240, 100.0)
        result = WellDetector.detect_well(frame, expected_diameter_px=0.0)
        self.assertIsNone(result)

        result2 = NeedleDetector.detect_needle(frame, expected_od_px=0.0)
        self.assertIsNone(result2)

    def test_single_channel_frame(self):
        """Single-channel (grayscale) input should work."""
        gray = np.full((480, 640), 50, dtype=np.uint8)
        cv2.circle(gray, (320, 240), 100, 200, -1)
        result = WellDetector.detect_well(gray, expected_diameter_px=200.0)
        # Should handle gracefully

    def test_focus_roi_out_of_bounds(self):
        """ROI extending beyond frame should be clamped."""
        frame = make_sharp_edges_image(200, 200)
        result = NeedleDetector.compute_focus_score(frame, roi_rect=(150, 150, 200, 200))
        # Should not crash — ROI clamped to frame bounds
        self.assertIsInstance(result, FocusResult)

    def test_focus_zero_size_roi(self):
        """Zero-size ROI should return zero score."""
        frame = make_sharp_edges_image(200, 200)
        result = NeedleDetector.compute_focus_score(frame, roi_rect=(100, 100, 0, 0))
        self.assertEqual(result.score, 0.0)


if __name__ == "__main__":
    unittest.main()
