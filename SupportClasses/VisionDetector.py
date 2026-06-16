"""
VisionDetector.py — Vision-based detection algorithms for MEBP v7.3.0.

Backend detection algorithms with ZERO GUI dependencies.

Classes:
    DetectionResult: Detected circle (well or needle) with position and confidence
    FocusResult: Focus quality metric for needle focus assist
    WellDetector: Detect circular well edges via HoughCircles + fallback
    NeedleDetector: Detect needle tip as dark circle, compute focus quality

All methods accept BGR numpy arrays (OpenCV convention) and return
dataclass results. Thread-safe — no mutable shared state.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Result Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class DetectionResult:
    """
    Result of a circle detection (well or needle).

    Pixel coordinates are relative to the input frame origin (top-left).
    Micron values are only populated if a CameraConfig is provided to
    the conversion helper.
    """
    center_px: tuple[float, float]      # (cx, cy) in pixels
    radius_px: float                     # Detected radius in pixels
    confidence: float                    # 0.0–1.0
    center_um: tuple[float, float] = (0.0, 0.0)  # Converted to µm (if calibrated)
    radius_um: float = 0.0              # Converted to µm
    method: str = ""                    # "hough", "contour", "adaptive_thresh"

    @property
    def diameter_px(self) -> float:
        return self.radius_px * 2

    @property
    def diameter_um(self) -> float:
        return self.radius_um * 2


@dataclass
class FocusResult:
    """
    Focus quality measurement for needle focus assist.

    The score increases as the image becomes sharper (needle more in focus).
    normalized_score is relative to the session maximum observed so far.
    """
    score: float                        # Raw focus metric (higher = sharper)
    normalized_score: float = 0.0       # 0.0–1.0 relative to session best
    is_in_focus: bool = False           # Above threshold
    roi_center_px: tuple[float, float] = (0.0, 0.0)  # Region analyzed
    laplacian_var: float = 0.0          # Laplacian variance component
    tenengrad: float = 0.0              # Tenengrad gradient component


# ---------------------------------------------------------------------------
# Coordinate Conversion Helpers
# ---------------------------------------------------------------------------

def pixel_offset_to_stage_um(
    detected_center_px: tuple[float, float],
    frame_size_px: tuple[int, int],
    micron_per_pixel: float,
) -> tuple[float, float]:
    """
    Convert detected center offset from frame center to stage movement in µm.

    Positive dx = detected center is right of frame center.
    Positive dy = detected center is below frame center.

    Note: Camera-to-stage axis mapping (sign flips, rotation) must be
    applied by the caller based on the mounting configuration.

    Args:
        detected_center_px: (cx, cy) detected position in pixels
        frame_size_px: (width, height) of the frame
        micron_per_pixel: µm/px scale factor

    Returns:
        (dx_um, dy_um) offset from frame center in micrometers
    """
    frame_cx = frame_size_px[0] / 2.0
    frame_cy = frame_size_px[1] / 2.0

    dx_px = detected_center_px[0] - frame_cx
    dy_px = detected_center_px[1] - frame_cy

    return (dx_px * micron_per_pixel, dy_px * micron_per_pixel)


# ---------------------------------------------------------------------------
# Internal Scoring Helpers
# ---------------------------------------------------------------------------

def _score_circles(
    circles: np.ndarray,
    frame_center: tuple[float, float],
    expected_radius: float,
) -> np.ndarray:
    """
    Score detected circles by proximity to frame center and radius match.

    Returns the single best circle as [cx, cy, radius].
    """
    if len(circles) == 1:
        return circles[0]

    best_score = -1.0
    best = circles[0]

    for c in circles:
        cx, cy, r = c[0], c[1], c[2]

        # Distance from frame center (normalized by frame diagonal)
        dist = math.sqrt((cx - frame_center[0]) ** 2 + (cy - frame_center[1]) ** 2)
        max_dist = math.sqrt(frame_center[0] ** 2 + frame_center[1] ** 2)
        dist_score = 1.0 - min(dist / max_dist, 1.0) if max_dist > 0 else 0.5

        # Radius match (1.0 = perfect, falls off with mismatch)
        if expected_radius > 0:
            radius_ratio = min(r, expected_radius) / max(r, expected_radius)
        else:
            radius_ratio = 0.5

        score = 0.5 * dist_score + 0.5 * radius_ratio
        if score > best_score:
            best_score = score
            best = c

    return best


def _compute_confidence(
    circle: np.ndarray,
    frame_center: tuple[float, float],
    expected_radius: float,
) -> float:
    """
    Compute confidence score (0.0–1.0) for a detected circle.

    Based on:
    - How close the radius matches expected
    - How close the center is to the frame center
    """
    cx, cy, r = circle[0], circle[1], circle[2]

    # Radius match (1.0 = perfect)
    if expected_radius > 0:
        radius_match = min(r, expected_radius) / max(r, expected_radius)
    else:
        radius_match = 0.5

    # Center proximity (1.0 = at frame center)
    dist = math.sqrt((cx - frame_center[0]) ** 2 + (cy - frame_center[1]) ** 2)
    max_dist = math.sqrt(frame_center[0] ** 2 + frame_center[1] ** 2)
    center_match = 1.0 - min(dist / max_dist, 1.0) if max_dist > 0 else 0.5

    # Weighted combination
    return 0.6 * radius_match + 0.4 * center_match


def _area_match(actual_area: float, expected_area: float) -> float:
    """Compute area similarity score (0.0–1.0). 1.0 = perfect match."""
    if expected_area <= 0:
        return 0.5
    return min(actual_area, expected_area) / max(actual_area, expected_area)


# ---------------------------------------------------------------------------
# WellDetector
# ---------------------------------------------------------------------------

class WellDetector:
    """
    Detect circular well edges in microscope camera frames.

    The well appears as a bright circle (well bottom) surrounded by a
    darker ring (well wall shadow). At lower magnifications the entire
    well may be visible; at higher magnifications only a partial arc
    may be in the FOV.

    All methods are stateless and thread-safe.
    """

    @staticmethod
    def detect_well(
        frame: np.ndarray,
        expected_diameter_px: float,
        tolerance: float = 0.3,
        dp: float = 1.5,
        param1: float = 100.0,
        param2: float = 30.0,
        blur_ksize: int = 9,
    ) -> DetectionResult | None:
        """
        Detect a circular well edge using HoughCircles.

        Args:
            frame: BGR image from camera
            expected_diameter_px: Expected well diameter in pixels
            tolerance: Fraction of diameter for radius range (±30% default)
            dp: HoughCircles accumulator resolution ratio
            param1: Canny edge threshold (higher = fewer edges)
            param2: Accumulator threshold (lower = more detections)
            blur_ksize: Gaussian blur kernel size (must be odd)

        Returns:
            DetectionResult if a well is found, None otherwise
        """
        if frame is None or frame.size == 0:
            return None

        if expected_diameter_px <= 0:
            logger.warning("detect_well: expected_diameter_px must be positive")
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame.copy()
        blurred = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 2)

        expected_radius = expected_diameter_px / 2.0
        min_radius = max(1, int(expected_radius * (1.0 - tolerance)))
        max_radius = int(expected_radius * (1.0 + tolerance))

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=expected_diameter_px * 0.8,
            param1=param1,
            param2=param2,
            minRadius=min_radius,
            maxRadius=max_radius,
        )

        if circles is None:
            return None

        # Score and pick best circle
        frame_center = (frame.shape[1] / 2.0, frame.shape[0] / 2.0)
        best = _score_circles(circles[0], frame_center, expected_radius)
        confidence = _compute_confidence(best, frame_center, expected_radius)

        return DetectionResult(
            center_px=(float(best[0]), float(best[1])),
            radius_px=float(best[2]),
            confidence=confidence,
            method="hough",
        )

    @staticmethod
    def _detect_well_contour(
        frame: np.ndarray,
        expected_diameter_px: float,
        tolerance: float = 0.3,
    ) -> DetectionResult | None:
        """
        Fallback well detection using adaptive threshold + contour + ellipse fitting.

        Useful when HoughCircles fails (e.g., partial circle, low contrast).
        """
        if frame is None or frame.size == 0:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame.copy()

        # Adaptive threshold to handle uneven lighting
        thresh = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 51, 5,
        )

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        expected_radius = expected_diameter_px / 2.0
        expected_area = math.pi * expected_radius ** 2
        frame_center = (frame.shape[1] / 2.0, frame.shape[0] / 2.0)

        best_result: DetectionResult | None = None
        best_score = -1.0

        for cnt in contours:
            # Need at least 5 points for ellipse fitting
            if len(cnt) < 5:
                continue

            area = cv2.contourArea(cnt)
            # Filter by area: must be within tolerance of expected
            if area < expected_area * (1 - tolerance) ** 2:
                continue
            if area > expected_area * (1 + tolerance) ** 2:
                continue

            # Fit ellipse
            ellipse = cv2.fitEllipse(cnt)
            (cx, cy), (w, h), angle = ellipse

            # Check circularity via axis ratio
            if w <= 0 or h <= 0:
                continue
            axis_ratio = min(w, h) / max(w, h)
            if axis_ratio < 0.5:  # Too elongated to be a well
                continue

            avg_radius = (w + h) / 4.0  # Average radius from ellipse axes
            radius_match = min(avg_radius, expected_radius) / max(avg_radius, expected_radius)

            # Distance from frame center
            dist = math.sqrt((cx - frame_center[0]) ** 2 + (cy - frame_center[1]) ** 2)
            max_dist = math.sqrt(frame_center[0] ** 2 + frame_center[1] ** 2)
            center_score = 1.0 - min(dist / max_dist, 1.0) if max_dist > 0 else 0.5

            score = 0.4 * radius_match + 0.3 * axis_ratio + 0.3 * center_score
            if score > best_score:
                best_score = score
                confidence = 0.4 * radius_match + 0.3 * axis_ratio + 0.3 * center_score
                best_result = DetectionResult(
                    center_px=(float(cx), float(cy)),
                    radius_px=float(avg_radius),
                    confidence=confidence,
                    method="contour",
                )

        return best_result

    @classmethod
    def detect_well_with_fallback(
        cls,
        frame: np.ndarray,
        expected_diameter_px: float,
        tolerance: float = 0.3,
        hough_confidence_threshold: float = 0.4,
    ) -> DetectionResult | None:
        """
        Detect well using HoughCircles first, falling back to contour fitting.

        Uses HoughCircles as primary method. If it fails or returns a
        low-confidence result, tries adaptive threshold + ellipse fitting.

        Args:
            frame: BGR image from camera
            expected_diameter_px: Expected well diameter in pixels
            tolerance: Radius tolerance fraction
            hough_confidence_threshold: Minimum Hough confidence before fallback

        Returns:
            Best DetectionResult or None
        """
        # Try HoughCircles first
        result = cls.detect_well(frame, expected_diameter_px, tolerance)
        if result is not None and result.confidence >= hough_confidence_threshold:
            return result

        # Fallback to contour fitting
        contour_result = cls._detect_well_contour(frame, expected_diameter_px, tolerance)

        # Return whichever has higher confidence
        if result is None:
            return contour_result
        if contour_result is None:
            return result
        return result if result.confidence >= contour_result.confidence else contour_result


# ---------------------------------------------------------------------------
# NeedleDetector
# ---------------------------------------------------------------------------

class NeedleDetector:
    """
    Detect needle tip and measure focus quality in microscope frames.

    The needle tip appears as a dark circle (or annulus if hollow) against
    a bright background under transmitted light illumination. As the needle
    descends toward the focal plane, the circle becomes sharper.

    All methods are stateless and thread-safe. Focus score tracking
    (session max for normalization) is managed by the caller.
    """

    @staticmethod
    def detect_needle(
        frame: np.ndarray,
        expected_od_px: float,
        tolerance: float = 0.4,
        min_circularity: float = 0.5,
        **kwargs,
    ) -> DetectionResult | None:
        """
        Detect needle tip as a dark circular region.

        Multi-strategy approach:
          1. HoughCircles on inverted grayscale (handles moderate blur)
          2. Contour-based with adaptive threshold scaled to needle size
          3. Radial intensity profile from frame center (last resort)

        The best result across strategies is returned.

        Args:
            frame: BGR image from camera
            expected_od_px: Expected needle outer diameter in pixels
            tolerance: Fraction of OD for size matching (±40% default)
            min_circularity: Minimum circularity for contour strategy

        Returns:
            DetectionResult if needle is found, None otherwise
        """
        if frame is None or frame.size == 0:
            return None

        if expected_od_px <= 0:
            logger.warning("detect_needle: expected_od_px must be positive")
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame.copy()
        frame_center = (gray.shape[1] / 2.0, gray.shape[0] / 2.0)
        expected_r = expected_od_px / 2.0

        # Strategy 1: HoughCircles on inverted image
        result_hough = NeedleDetector._detect_hough(
            gray, expected_r, tolerance, frame_center)

        # Strategy 2: Contour-based with adaptive threshold
        result_contour = NeedleDetector._detect_contour(
            gray, expected_r, tolerance, min_circularity, frame_center)

        # Strategy 3: Radial intensity profile from frame center
        result_radial = NeedleDetector._detect_radial(
            gray, expected_r, tolerance, frame_center)

        # Pick the best result by confidence
        best = None
        for r in (result_hough, result_contour, result_radial):
            if r is not None and (best is None or r.confidence > best.confidence):
                best = r
        return best

    @staticmethod
    def _detect_hough(
        gray: np.ndarray,
        expected_r: float,
        tolerance: float,
        frame_center: tuple[float, float],
    ) -> DetectionResult | None:
        """HoughCircles on inverted grayscale — robust to moderate blur."""
        # Invert: dark needle → bright circle for Hough detection
        inv = cv2.bitwise_not(gray)
        # Median blur to reduce noise while preserving circle edges
        ksize = max(3, int(expected_r * 0.04) | 1)
        ksize = min(ksize, 31)
        blurred = cv2.medianBlur(inv, ksize)

        min_r = max(5, int(expected_r * (1.0 - tolerance)))
        max_r = int(expected_r * (1.0 + tolerance))
        min_dist = max(20, int(expected_r))

        # Try multiple param2 thresholds (accumulator sensitivity)
        for param2 in (30, 20, 40, 15):
            circles = cv2.HoughCircles(
                blurred, cv2.HOUGH_GRADIENT, dp=1.5,
                minDist=min_dist,
                param1=80, param2=param2,
                minRadius=min_r, maxRadius=max_r,
            )
            if circles is not None and len(circles[0]) > 0:
                break
        else:
            return None

        # Score each circle by size match + center proximity
        best_score = -1.0
        best_circle = None
        for c in circles[0]:
            cx, cy, r = float(c[0]), float(c[1]), float(c[2])
            size_score = min(r, expected_r) / max(r, expected_r)
            dist = math.sqrt((cx - frame_center[0]) ** 2 +
                             (cy - frame_center[1]) ** 2)
            max_dist = math.sqrt(frame_center[0] ** 2 + frame_center[1] ** 2)
            center_score = (1.0 - min(dist / max_dist, 1.0)
                            if max_dist > 0 else 0.5)
            score = 0.5 * size_score + 0.5 * center_score
            if score > best_score:
                best_score = score
                best_circle = (cx, cy, r, size_score)

        if best_circle is None:
            return None

        cx, cy, r, size_sc = best_circle
        confidence = min(0.5 * size_sc + 0.5 * best_score, 1.0)
        return DetectionResult(
            center_px=(cx, cy),
            radius_px=r,
            confidence=confidence,
            method="hough",
        )

    @staticmethod
    def _detect_contour(
        gray: np.ndarray,
        expected_r: float,
        tolerance: float,
        min_circularity: float,
        frame_center: tuple[float, float],
    ) -> DetectionResult | None:
        """Adaptive threshold + contour analysis with scaled parameters."""
        expected_od = expected_r * 2.0
        expected_area = math.pi * expected_r ** 2

        # Scale block_size to ~40% of expected diameter, ensure odd & >= 11
        block_size = max(11, int(expected_od * 0.4) | 1)
        # Clamp to reasonable range
        block_size = min(block_size, 501)

        # Try multiple c_offset values for robustness
        for c_offset in (10, 5, 15, 20):
            thresh = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, block_size, c_offset,
            )

            # Scale morphology kernel to needle size
            kern_size = max(3, int(expected_r * 0.03) | 1)
            kern_size = min(kern_size, 21)
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kern_size, kern_size))
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(
                thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            candidates = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < expected_area * (1.0 - tolerance):
                    continue
                if area > expected_area * (1.0 + tolerance):
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter <= 0:
                    continue

                circularity = 4.0 * math.pi * area / (perimeter ** 2)
                if circularity < min_circularity:
                    continue

                # Use moments for center (more accurate than minEnclosingCircle)
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                else:
                    (cx, cy), _ = cv2.minEnclosingCircle(cnt)

                # Radius from area (assumes circle) — avoids minEnclosingCircle
                # overestimate from non-circular contours
                radius = math.sqrt(area / math.pi)

                area_score = _area_match(area, expected_area)
                candidates.append(
                    (cx, cy, radius, circularity, area, area_score))

            if candidates:
                break
        else:
            return None

        # Score: circularity + area match + center proximity
        def candidate_score(c):
            cx, cy, r, circ, area, a_score = c
            dist = math.sqrt(
                (cx - frame_center[0]) ** 2 + (cy - frame_center[1]) ** 2)
            max_dist = math.sqrt(
                frame_center[0] ** 2 + frame_center[1] ** 2)
            center_sc = (1.0 - min(dist / max_dist, 1.0)
                         if max_dist > 0 else 0.5)
            return 0.4 * circ + 0.3 * a_score + 0.3 * center_sc

        best = max(candidates, key=candidate_score)
        cx, cy, radius, circularity, area, area_sc = best
        confidence = min(0.5 * circularity + 0.5 * area_sc, 1.0)

        return DetectionResult(
            center_px=(float(cx), float(cy)),
            radius_px=float(radius),
            confidence=confidence,
            method="contour",
        )

    @staticmethod
    def _detect_radial(
        gray: np.ndarray,
        expected_r: float,
        tolerance: float,
        frame_center: tuple[float, float],
    ) -> DetectionResult | None:
        """
        Radial intensity profile from frame center — last-resort strategy.

        Samples intensity along radial lines from center outward. The
        needle edge appears as a dark-to-bright transition. This works
        even with significant blur where contour/Hough methods fail.
        """
        h, w = gray.shape[:2]
        cx, cy = frame_center
        max_r = int(expected_r * (1.0 + tolerance))
        max_r = min(max_r, int(min(cx, cy, w - cx, h - cy) - 1))
        if max_r < 10:
            return None

        # Sample along 36 radial directions (every 10 degrees)
        n_angles = 36
        transition_radii = []
        for i in range(n_angles):
            angle = 2.0 * math.pi * i / n_angles
            cos_a, sin_a = math.cos(angle), math.sin(angle)

            # Build radial profile
            profile = []
            for r in range(max_r):
                px = int(cx + r * cos_a)
                py = int(cy + r * sin_a)
                if 0 <= px < w and 0 <= py < h:
                    profile.append(float(gray[py, px]))
                else:
                    break

            if len(profile) < 20:
                continue

            profile = np.array(profile)
            # Smooth the profile to handle noise
            if len(profile) > 7:
                kernel_1d = np.ones(7) / 7.0
                profile = np.convolve(profile, kernel_1d, mode='same')

            # Find the maximum gradient (dark → bright transition)
            gradient = np.diff(profile)
            if len(gradient) < 5:
                continue

            # Look for transition in the expected radius range
            min_search = max(0, int(expected_r * (1.0 - tolerance)))
            max_search = min(len(gradient), int(expected_r * (1.0 + tolerance)))
            if min_search >= max_search:
                continue

            search_grad = gradient[min_search:max_search]
            if len(search_grad) == 0:
                continue

            peak_idx = int(np.argmax(search_grad))
            peak_val = search_grad[peak_idx]

            # Require a meaningful positive gradient (dark → bright)
            if peak_val > 2.0:
                transition_radii.append(min_search + peak_idx)

        if len(transition_radii) < 6:  # Need at least 6/36 consistent angles
            return None

        # Robust radius estimate: median of transitions
        radius = float(np.median(transition_radii))

        # Confidence from consistency of radius estimates and size match
        std = float(np.std(transition_radii))
        consistency = max(0.0, 1.0 - std / (radius * 0.3)) if radius > 0 else 0.0
        size_match = (min(radius, expected_r) / max(radius, expected_r)
                      if expected_r > 0 else 0.5)
        coverage = min(len(transition_radii) / n_angles, 1.0)
        confidence = min(0.3 * consistency + 0.4 * size_match + 0.3 * coverage, 1.0)

        # Confidence penalty — radial assumes center = frame center
        confidence *= 0.85

        return DetectionResult(
            center_px=(float(cx), float(cy)),
            radius_px=radius,
            confidence=confidence,
            method="radial",
        )

    # ── Relaxed detection + edge refinement (v7.3.3) ────────────

    @staticmethod
    def detect_needle_relaxed(
        frame: np.ndarray,
        expected_od_px: float = 0.0,
        min_circularity: float = 0.4,
        **kwargs,
    ) -> DetectionResult | None:
        """
        Detect needle tip with very relaxed size constraints.

        Used when the µm/px calibration may be inaccurate, so the expected
        diameter could be far off. If expected_od_px is 0, searches for any
        prominent dark circle in the frame.

        Args:
            frame: BGR image from camera
            expected_od_px: Expected OD in pixels (0 = unconstrained)
            min_circularity: Minimum circularity for contour strategy

        Returns:
            DetectionResult if a circle is found, None otherwise
        """
        if frame is None or frame.size == 0:
            return None

        gray = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if len(frame.shape) == 3 else frame.copy())
        frame_center = (gray.shape[1] / 2.0, gray.shape[0] / 2.0)
        h, w = gray.shape[:2]

        if expected_od_px > 0:
            # Wide tolerance: ±80%
            expected_r = expected_od_px / 2.0
            tolerance = 0.8
        else:
            # Unconstrained: search from 10px to 1/3 of min dimension
            expected_r = min(h, w) / 6.0  # midpoint of search range
            tolerance = 0.9  # covers 10% to 190% of midpoint

        result_hough = NeedleDetector._detect_hough(
            gray, expected_r, tolerance, frame_center)
        result_contour = NeedleDetector._detect_contour(
            gray, expected_r, tolerance, min_circularity, frame_center)
        result_radial = NeedleDetector._detect_radial(
            gray, expected_r, tolerance, frame_center)

        best = None
        for r in (result_hough, result_contour, result_radial):
            if r is not None and (best is None or r.confidence > best.confidence):
                best = r
        return best

    @staticmethod
    def refine_to_edge(
        frame: np.ndarray,
        initial_center: tuple[float, float],
        initial_radius: float,
        n_angles: int = 72,
        search_range: float = 0.5,
    ) -> DetectionResult | None:
        """
        Refine a detected circle to snap to the actual needle edge.

        Samples radial intensity profiles from the initial center and finds
        the strongest dark→bright gradient near the initial radius. Fits a
        circle through those edge points using the Kasa algebraic method.

        Args:
            frame: BGR image from camera
            initial_center: (cx, cy) initial circle center in pixels
            initial_radius: Initial radius in pixels
            n_angles: Number of radial directions to sample
            search_range: Fraction of radius to search (±50% default)

        Returns:
            Refined DetectionResult, or None if refinement fails
        """
        if frame is None or frame.size == 0:
            return None
        if initial_radius < 3:
            return None

        gray = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if len(frame.shape) == 3 else frame.copy())
        h, w = gray.shape[:2]
        cx, cy = initial_center

        r_min = max(3, int(initial_radius * (1.0 - search_range)))
        r_max = min(int(initial_radius * (1.0 + search_range)),
                    int(min(cx, cy, w - cx, h - cy) - 1))
        if r_max <= r_min:
            return None

        edge_points = []
        gradient_strengths = []

        for i in range(n_angles):
            angle = 2.0 * math.pi * i / n_angles
            cos_a, sin_a = math.cos(angle), math.sin(angle)

            # Build radial intensity profile from r_min to r_max
            profile = []
            for r in range(r_min, r_max + 1):
                px = int(cx + r * cos_a)
                py = int(cy + r * sin_a)
                if 0 <= px < w and 0 <= py < h:
                    profile.append(float(gray[py, px]))
                else:
                    break

            if len(profile) < 5:
                continue

            profile_arr = np.array(profile)
            # Smooth to suppress noise
            if len(profile_arr) > 5:
                kernel = np.ones(5) / 5.0
                profile_arr = np.convolve(profile_arr, kernel, mode='same')

            # Find strongest positive gradient (dark → bright = needle edge)
            gradient = np.diff(profile_arr)
            if len(gradient) < 2:
                continue

            peak_idx = int(np.argmax(gradient))
            peak_val = gradient[peak_idx]

            if peak_val > 1.5:  # Meaningful transition
                edge_r = r_min + peak_idx
                edge_x = cx + edge_r * cos_a
                edge_y = cy + edge_r * sin_a
                edge_points.append((edge_x, edge_y))
                gradient_strengths.append(peak_val)

        if len(edge_points) < 12:
            return None

        points = np.array(edge_points)

        # Filter outliers: remove points > 2σ from median radius
        radii = np.sqrt((points[:, 0] - cx) ** 2 + (points[:, 1] - cy) ** 2)
        median_r = np.median(radii)
        std_r = np.std(radii)
        if std_r > 0:
            mask = np.abs(radii - median_r) < 2.0 * std_r
            points = points[mask]

        if len(points) < 12:
            return None

        # Fit circle through edge points
        fit = NeedleDetector._fit_circle_kasa(points)
        if fit is None:
            return None

        fit_cx, fit_cy, fit_r = fit

        # Validate: refined center shouldn't drift too far from initial
        center_drift = math.sqrt((fit_cx - cx) ** 2 + (fit_cy - cy) ** 2)
        if center_drift > initial_radius * 0.5:
            # Fallback: use initial center with median radius
            fit_cx, fit_cy = cx, cy
            fit_r = float(np.median(
                np.sqrt((points[:, 0] - cx) ** 2 + (points[:, 1] - cy) ** 2)))

        # Confidence from edge consistency and angular coverage
        refined_radii = np.sqrt(
            (points[:, 0] - fit_cx) ** 2 + (points[:, 1] - fit_cy) ** 2)
        consistency = max(0.0, 1.0 - float(np.std(refined_radii)) /
                         (fit_r * 0.15)) if fit_r > 0 else 0.0
        coverage = min(len(points) / n_angles, 1.0)
        confidence = min(0.5 * consistency + 0.5 * coverage, 1.0)

        return DetectionResult(
            center_px=(float(fit_cx), float(fit_cy)),
            radius_px=float(fit_r),
            confidence=confidence,
            method="edge_refine",
        )

    @staticmethod
    def _fit_circle_kasa(points: np.ndarray) -> tuple[float, float, float] | None:
        """
        Fit a circle to 2D points using the Kasa algebraic method.

        Solves the over-determined system: x² + y² = A·x + B·y + C
        Then: cx = A/2, cy = B/2, r = sqrt(C + cx² + cy²)

        Args:
            points: Nx2 array of (x, y) coordinates

        Returns:
            (cx, cy, radius) or None if fit fails
        """
        if points is None or len(points) < 3:
            return None

        x = points[:, 0]
        y = points[:, 1]
        rhs = x ** 2 + y ** 2

        # Build matrix [x, y, 1]
        A = np.column_stack([x, y, np.ones(len(x))])

        # Solve least-squares: A @ [a, b, c] = rhs
        try:
            result, residuals, rank, sv = np.linalg.lstsq(A, rhs, rcond=None)
        except np.linalg.LinAlgError:
            return None

        a, b, c = result
        cx = a / 2.0
        cy = b / 2.0
        r_sq = c + cx ** 2 + cy ** 2

        if r_sq <= 0:
            return None

        return (float(cx), float(cy), float(math.sqrt(r_sq)))

    @staticmethod
    def compute_focus_score(
        frame: np.ndarray,
        roi_rect: tuple[int, int, int, int] | None = None,
    ) -> FocusResult:
        """
        Compute focus quality metric using Laplacian variance + Tenengrad.

        Higher score = sharper edges = better focus. As the needle descends
        toward the focal plane, the score increases to a peak (best focus)
        then decreases.

        Args:
            frame: BGR image from camera
            roi_rect: Optional (x, y, w, h) region of interest. If None, uses full frame.

        Returns:
            FocusResult with raw score (caller normalizes against session max)
        """
        if frame is None or frame.size == 0:
            return FocusResult(score=0.0, roi_center_px=(0.0, 0.0))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame.copy()

        roi_center = (gray.shape[1] / 2.0, gray.shape[0] / 2.0)

        if roi_rect is not None:
            x, y, w, h = roi_rect
            # Clamp to frame bounds
            x = max(0, x)
            y = max(0, y)
            w = min(w, gray.shape[1] - x)
            h = min(h, gray.shape[0] - y)
            if w <= 0 or h <= 0:
                return FocusResult(score=0.0, roi_center_px=roi_center)
            gray = gray[y:y + h, x:x + w]
            roi_center = (x + w / 2.0, y + h / 2.0)

        # Primary: Laplacian variance (fast, reliable focus metric)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        laplacian_var = float(laplacian.var())

        # Secondary: Tenengrad (Sobel gradient magnitude mean)
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        tenengrad = float((gx ** 2 + gy ** 2).mean())

        # Combined score (weighted)
        score = 0.7 * laplacian_var + 0.3 * tenengrad

        return FocusResult(
            score=score,
            normalized_score=0.0,   # Caller normalizes against session max
            is_in_focus=False,      # Caller determines threshold
            roi_center_px=roi_center,
            laplacian_var=laplacian_var,
            tenengrad=tenengrad,
        )


# ---------------------------------------------------------------------------
# Focus Score Tracker (session state helper)
# ---------------------------------------------------------------------------

class FocusTracker:
    """
    Tracks focus scores across a session for normalization.

    Maintains the session-maximum score to normalize FocusResult values
    to 0.0–1.0. Not thread-safe — designed to be owned by a single
    DetectionWorker or GUI controller.
    """

    def __init__(self, in_focus_threshold: float = 0.7):
        """
        Args:
            in_focus_threshold: Normalized score above which is_in_focus = True
        """
        self._session_max: float = 0.0
        self._in_focus_threshold = in_focus_threshold
        self._score_history: list[float] = []
        self._history_maxlen: int = 100

    @property
    def session_max(self) -> float:
        return self._session_max

    @property
    def in_focus_threshold(self) -> float:
        return self._in_focus_threshold

    @in_focus_threshold.setter
    def in_focus_threshold(self, value: float) -> None:
        self._in_focus_threshold = max(0.0, min(1.0, value))

    def update(self, result: FocusResult) -> FocusResult:
        """
        Normalize a FocusResult against the session maximum.

        Updates session_max if this score is higher, then returns a
        new FocusResult with normalized_score and is_in_focus populated.

        Args:
            result: Raw FocusResult from NeedleDetector.compute_focus_score()

        Returns:
            Updated FocusResult with normalized_score and is_in_focus set
        """
        if result.score > self._session_max:
            self._session_max = result.score

        # Track history
        self._score_history.append(result.score)
        if len(self._score_history) > self._history_maxlen:
            self._score_history.pop(0)

        # Normalize
        if self._session_max > 0:
            normalized = result.score / self._session_max
        else:
            normalized = 0.0

        return FocusResult(
            score=result.score,
            normalized_score=normalized,
            is_in_focus=normalized >= self._in_focus_threshold,
            roi_center_px=result.roi_center_px,
            laplacian_var=result.laplacian_var,
            tenengrad=result.tenengrad,
        )

    def get_trend(self, window: int = 5) -> str:
        """
        Get recent focus trend direction.

        Returns:
            "improving" | "declining" | "stable" | "unknown"
        """
        if len(self._score_history) < window + 1:
            return "unknown"

        recent = self._score_history[-window:]
        older = self._score_history[-(window + window) : -window]
        if not older:
            return "unknown"

        recent_avg = sum(recent) / len(recent)
        older_avg = sum(older) / len(older)

        if self._session_max > 0:
            change = (recent_avg - older_avg) / self._session_max
        else:
            change = 0.0

        if change > 0.02:
            return "improving"
        elif change < -0.02:
            return "declining"
        else:
            return "stable"

    def reset(self) -> None:
        """Reset session state for a new focus assist session."""
        self._session_max = 0.0
        self._score_history.clear()


# ---------------------------------------------------------------------------
# Pixel Displacement Measurement (v7.3.3 — camera µm/px calibration)
# ---------------------------------------------------------------------------

def measure_pixel_displacement(
    frame_before: np.ndarray,
    frame_after: np.ndarray,
) -> tuple[float, float, float]:
    """
    Measure sub-pixel displacement between two frames using phase correlation.

    Used for camera µm/px calibration: capture before/after a known stage move,
    then compute µm/px = move_distance / pixel_displacement.

    Args:
        frame_before: BGR image captured before stage move
        frame_after: BGR image captured after stage move

    Returns:
        (dx_pixels, dy_pixels, confidence) where confidence is 0.0–1.0
        from the phase correlation response. dx/dy follow image convention
        (positive dx = rightward, positive dy = downward).

    Raises:
        ValueError: If frames have different shapes or are empty.
    """
    if frame_before is None or frame_after is None:
        raise ValueError("Both frames must be non-None")
    if frame_before.shape[:2] != frame_after.shape[:2]:
        raise ValueError(
            f"Frame shapes differ: {frame_before.shape[:2]} vs {frame_after.shape[:2]}")
    if frame_before.size == 0:
        raise ValueError("Frames are empty")

    # Convert to grayscale float64 (required by cv2.phaseCorrelate)
    if len(frame_before.shape) == 3:
        gray_before = cv2.cvtColor(frame_before, cv2.COLOR_BGR2GRAY)
    else:
        gray_before = frame_before.copy()
    if len(frame_after.shape) == 3:
        gray_after = cv2.cvtColor(frame_after, cv2.COLOR_BGR2GRAY)
    else:
        gray_after = frame_after.copy()

    a = gray_before.astype(np.float64)
    b = gray_after.astype(np.float64)

    # Hanning window suppresses edge artifacts in FFT-based correlation
    h, w = a.shape
    hann = cv2.createHanningWindow((w, h), cv2.CV_64F)

    (dx, dy), response = cv2.phaseCorrelate(a, b, hann)

    # response is the peak value of the normalized cross-power spectrum (0–1)
    confidence = float(max(0.0, min(response, 1.0)))

    return (float(dx), float(dy), confidence)


# ---------------------------------------------------------------------------
# v7.4.4: Two-Camera Needle Aligner (user-click driven)
# ---------------------------------------------------------------------------

@dataclass
class TwoCameraEdgePicks:
    """The four edge clicks gathered during the Needle Location workflow."""
    x_view_left_px: float | None = None
    x_view_right_px: float | None = None
    y_view_left_px: float | None = None
    y_view_right_px: float | None = None

    def x_view_complete(self) -> bool:
        return (self.x_view_left_px is not None
                and self.x_view_right_px is not None)

    def y_view_complete(self) -> bool:
        return (self.y_view_left_px is not None
                and self.y_view_right_px is not None)

    def complete(self) -> bool:
        return self.x_view_complete() and self.y_view_complete()


class TwoCameraNeedleAligner:
    """Compute the stage offset that recenters the needle in two cameras.

    Mounting assumption: two side cameras mounted orthogonally to the
    workspace.

      * `NEEDLE_X` looks down the **X** axis. Its image columns map to
        stage Y; rows map to stage Z.
      * `NEEDLE_Y` looks down the **Y** axis. Its image columns map to
        stage X; rows map to stage Z.

    The user clicks the left and right visible edges of the needle in
    each camera. The midpoint of those two clicks is the needle's
    pixel-center along the column axis. The pixel offset from the
    frame's column center, scaled by that camera's µm/pixel, gives the
    stage offset needed to bring the needle to the optical center.

    Sign conventions follow the existing project convention: positive
    pixel offset (center is right of frame center) → positive stage
    offset along the mapped axis. Callers can pass per-axis sign flips
    via `x_sign` / `y_sign` when the physical mounting reverses one
    axis.

    v7.5.x — rotated mountings: when the cameras are *not* mounted
    orthogonally to the stage axes (e.g. at 45°), pass each camera's
    column→stage **direction angle** via ``angle_x_view_deg`` /
    ``angle_y_view_deg`` — the stage-plane angle (degrees CCW from stage
    +X) that a positive column offset corresponds to. A positive column
    offset of ``off`` px means the needle is displaced from the optical
    axis by ``off × µm/px`` along that direction; the recentering move is
    the solution of the 2×2 system of both cameras' projections. With the
    angles left as ``None`` the legacy orthogonal mapping is used
    (x_view column → stage +Y = 90°, y_view column → stage +X = 0°), so
    existing behavior is unchanged.
    """

    def __init__(
        self,
        um_per_px_x_view: float,
        um_per_px_y_view: float,
        frame_width_x_view: int,
        frame_width_y_view: int,
        x_sign: float = 1.0,
        y_sign: float = 1.0,
        angle_x_view_deg: float | None = None,
        angle_y_view_deg: float | None = None,
    ) -> None:
        if um_per_px_x_view <= 0 or um_per_px_y_view <= 0:
            raise ValueError("um_per_px must be positive for both cameras")
        if frame_width_x_view <= 0 or frame_width_y_view <= 0:
            raise ValueError("frame widths must be positive")
        self.um_per_px_x_view = float(um_per_px_x_view)
        self.um_per_px_y_view = float(um_per_px_y_view)
        self.frame_width_x_view = int(frame_width_x_view)
        self.frame_width_y_view = int(frame_width_y_view)
        self.x_sign = float(x_sign)
        self.y_sign = float(y_sign)
        # Legacy orthogonal mapping when unset: x_view→+Y (90°), y_view→+X (0°).
        self.angle_x_view_deg = (
            90.0 if angle_x_view_deg is None else float(angle_x_view_deg))
        self.angle_y_view_deg = (
            0.0 if angle_y_view_deg is None else float(angle_y_view_deg))

    @staticmethod
    def _midpoint(a: float, b: float) -> float:
        return (a + b) / 2.0

    def offset_from_edge_clicks(
        self, picks: TwoCameraEdgePicks
    ) -> tuple[float, float]:
        """Return `(dx_um, dy_um)` — the stage move that recenters the
        needle in both views.

        Raises ``ValueError`` if any pick is missing, or if the two camera
        direction angles are parallel (degenerate, non-invertible).
        """
        if not picks.complete():
            raise ValueError(
                "All four edge picks must be supplied "
                "(x_view left+right and y_view left+right)"
            )

        # Signed column offset (px) of the needle center from each frame's
        # center, → physical displacement (µm) along that camera's direction.
        x_view_offset_px = self._midpoint(
            picks.x_view_left_px, picks.x_view_right_px
        ) - (self.frame_width_x_view / 2.0)
        y_view_offset_px = self._midpoint(
            picks.y_view_left_px, picks.y_view_right_px
        ) - (self.frame_width_y_view / 2.0)
        s1 = x_view_offset_px * self.um_per_px_x_view
        s2 = y_view_offset_px * self.um_per_px_y_view

        a1 = math.radians(self.angle_x_view_deg)
        a2 = math.radians(self.angle_y_view_deg)
        # U · n = s, where rows of U are the cameras' stage-direction unit
        # vectors and n is the needle's stage-XY offset from the optical axes.
        det = math.cos(a1) * math.sin(a2) - math.cos(a2) * math.sin(a1)
        if abs(det) < 1e-9:
            raise ValueError(
                "Camera direction angles are parallel (singular) — "
                f"x_view={self.angle_x_view_deg:.1f}°, "
                f"y_view={self.angle_y_view_deg:.1f}°"
            )
        # n = U⁻¹ · s   (2×2 closed form)
        nx = (math.sin(a2) * s1 - math.sin(a1) * s2) / det
        ny = (-math.cos(a2) * s1 + math.cos(a1) * s2) / det
        return (float(self.x_sign * nx), float(self.y_sign * ny))


# ---------------------------------------------------------------------------
# v7.4.4: Edge-Fit Well Locator (known-radius partial-arc fit)
# ---------------------------------------------------------------------------

class EdgeFitWellLocator:
    """Locate a well center by fitting a circle of *known* radius to its
    visible edge — works whether the well fills the frame or only a
    partial arc is visible (objective dependent: 2× / 4× / 10×).

    The Plate Location workflow drives the stage to the user-clicked
    target well and uses this fitter to refine the well's actual centre
    against the predicted centre. The radius is locked to the plate's
    known well diameter, which is far more robust than free-radius
    Hough fits when only a fraction of the rim is visible.
    """

    def __init__(self, canny_low: int = 50, canny_high: int = 150) -> None:
        self.canny_low = int(canny_low)
        self.canny_high = int(canny_high)

    @staticmethod
    def _to_gray(frame: np.ndarray) -> np.ndarray:
        if frame.ndim == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def fit_partial_arc(
        self,
        frame: np.ndarray,
        expected_radius_px: float,
        um_per_px: float,
        tolerance_pct: float = 0.10,
        min_edge_pixels: int = 30,
    ) -> Optional[DetectionResult]:
        """Fit a circle of *known* radius to the well's visible edge.

        Parameters
        ----------
        frame : np.ndarray
            Single camera frame (BGR or grayscale).
        expected_radius_px : float
            The well's known radius in pixels (= half the plate-spec
            well diameter, converted via ``um_per_px``).
        um_per_px : float
            For optional µm conversion of the result; not used in the
            fit itself.
        tolerance_pct : float
            Allowed deviation of the fitted radius from
            ``expected_radius_px`` (0.10 = ±10 %). A fit outside this
            band is rejected.
        min_edge_pixels : int
            Sanity threshold; fewer detected edge pixels → return None.
        """
        if expected_radius_px <= 0 or um_per_px <= 0:
            return None
        gray = self._to_gray(frame)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        ys, xs = np.nonzero(edges)
        if xs.size < min_edge_pixels:
            return None

        # Algebraic least-squares circle fit (Kåsa). Even with the
        # radius free, this is numerically robust for arcs >~45°. We
        # then validate the fitted radius against the known value.
        x = xs.astype(np.float64)
        y = ys.astype(np.float64)
        A = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
        b = x * x + y * y
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return None
        cx_fit, cy_fit, c = float(sol[0]), float(sol[1]), float(sol[2])
        r_squared = c + cx_fit * cx_fit + cy_fit * cy_fit
        if r_squared <= 0:
            return None
        r_fit = math.sqrt(r_squared)

        # Radius-band guard rejects bogus fits to background clutter.
        lo = expected_radius_px * (1.0 - tolerance_pct)
        hi = expected_radius_px * (1.0 + tolerance_pct)
        if not (lo <= r_fit <= hi):
            return None

        # Confidence = inverse residual normalized by radius. Closer
        # to 1.0 means the edge pixels lie tightly on the fitted
        # circle.
        residuals = np.abs(np.hypot(x - cx_fit, y - cy_fit) - r_fit)
        rms = float(np.sqrt(np.mean(residuals * residuals)))
        confidence = float(np.clip(1.0 - rms / max(r_fit, 1.0), 0.0, 1.0))

        # Snap the radius to the known value for downstream use; the
        # fitted radius only existed to validate the candidate.
        return DetectionResult(
            center_px=(cx_fit, cy_fit),
            radius_px=float(expected_radius_px),
            confidence=confidence,
            center_um=(cx_fit * um_per_px, cy_fit * um_per_px),
            radius_um=expected_radius_px * um_per_px,
            method="edge_fit_partial_arc",
        )


# ---------------------------------------------------------------------------
# v7.4.6: Adaptive well-fit strategy + rim-point sampling
# ---------------------------------------------------------------------------

# Thresholds for picking a circle-fit strategy from the FOV ratio, defined
# as well_diameter_um / min(fov_w_um, fov_h_um) — i.e. how many fields of
# view the well spans. Tunable.
WELL_FIT_FULL_CIRCLE_MAX_RATIO = 0.85   # whole well comfortably fits in view
WELL_FIT_PARTIAL_ARC_MAX_RATIO = 3.0    # arc still curves enough for a
                                        # known-radius single-frame fit


def select_well_fit_strategy(
    well_diameter_um: float,
    fov_w_um: float,
    fov_h_um: float,
) -> str:
    """Pick the circle-fit strategy from well size vs. camera field of view.

    Returns one of:

    * ``"full_circle"`` — the whole well fits in one frame; detect the
      full circle directly (Hough / contour).
    * ``"partial_arc"`` — only an arc is visible, but it curves enough to
      pin the center via a known-radius fit on a single frame.
    * ``"multi_edge"`` — the well is so much larger than the FOV that the
      visible rim is nearly flat; the caller must sample several rim
      points and fit a circle through them.
    """
    fov_min = min(fov_w_um, fov_h_um)
    if fov_min <= 0 or well_diameter_um <= 0:
        return "partial_arc"
    ratio = well_diameter_um / fov_min
    if ratio <= WELL_FIT_FULL_CIRCLE_MAX_RATIO:
        return "full_circle"
    if ratio <= WELL_FIT_PARTIAL_ARC_MAX_RATIO:
        return "partial_arc"
    return "multi_edge"


def fit_circle_to_points(points) -> "tuple[float, float, float] | None":
    """Kåsa algebraic circle fit through ≥3 points.

    ``points`` is an Nx2 array-like of (x, y). With exactly three
    non-collinear points this is the exact circumcircle. Returns
    ``(cx, cy, radius)`` or None (too few points / collinear).
    """
    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] < 3 or arr.shape[1] != 2:
        return None
    # Reject (near-)collinear point sets: Kåsa still returns a finite
    # bogus circle for points on a line, so guard explicitly via the
    # smaller eigenvalue of the centered covariance.
    centered = arr - arr.mean(axis=0)
    evals = np.linalg.eigvalsh(centered.T @ centered)
    if evals[0] <= 1e-6 * max(float(evals[1]), 1e-9):
        return None
    return NeedleDetector._fit_circle_kasa(arr)


def detect_rim_point_near_center(
    frame: np.ndarray,
    canny_low: int = 50,
    canny_high: int = 150,
    search_frac: float = 0.30,
    min_edge_pixels: int = 5,
) -> "tuple[float, float, float] | None":
    """Find the well-rim crossing nearest the frame center.

    Used by the multi-edge strategy: the stage is aimed at a predicted
    rim point, so the true rim crosses near the frame center. Canny-detect
    edges, restrict to a central window (reject far clutter), take the
    edge pixel nearest the center, and average its immediate neighbors for
    sub-pixel stability.

    Returns ``(px, py, confidence)`` in pixel coordinates, or None.
    """
    if frame is None:
        return None
    gray = frame
    if gray.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, int(canny_low), int(canny_high))
    ys, xs = np.nonzero(edges)
    if xs.size < min_edge_pixels:
        return None
    h, w = gray.shape[:2]
    fcx, fcy = w / 2.0, h / 2.0
    xs = xs.astype(np.float64)
    ys = ys.astype(np.float64)
    win_x = w * search_frac
    win_y = h * search_frac
    mask = (np.abs(xs - fcx) < win_x) & (np.abs(ys - fcy) < win_y)
    if int(mask.sum()) >= min_edge_pixels:
        sx, sy = xs[mask], ys[mask]
    else:
        sx, sy = xs, ys
    d2 = (sx - fcx) ** 2 + (sy - fcy) ** 2
    k = int(np.argmin(d2))
    px0, py0 = sx[k], sy[k]
    cluster_r = 0.05 * min(w, h)
    near = ((sx - px0) ** 2 + (sy - py0) ** 2) < (cluster_r * cluster_r)
    px = float(np.mean(sx[near]))
    py = float(np.mean(sy[near]))
    confidence = float(np.clip(int(near.sum()) / 20.0, 0.0, 1.0))
    return (px, py, confidence)
