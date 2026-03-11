"""
test_v730_detection_worker.py — Tests for DetectionWorker and CameraWidget.get_current_frame().

Tests Phase 3: parallel processing infrastructure.
- Thread-safe frame access from CameraWidget
- DetectionWorker mode switching and signal emission
- Detection throughput (target <50ms per cycle)
"""

import time
import threading
import unittest

import cv2
import numpy as np

from SupportClasses.VisionDetector import (
    DetectionResult,
    FocusResult,
    WellDetector,
    NeedleDetector,
)
from gui.widgets.detection_worker import DetectionMode, DetectionWorker


# ---------------------------------------------------------------------------
# Mock CameraWidget (no Qt dependency needed)
# ---------------------------------------------------------------------------

class MockCameraWidget:
    """
    Simulates CameraWidget.get_current_frame() for testing.

    Thread-safe frame buffer that can be updated from a background thread
    to simulate live camera capture.
    """

    def __init__(self, frame: np.ndarray | None = None):
        self._frame = frame
        self._lock = threading.Lock()

    def set_frame(self, frame: np.ndarray | None) -> None:
        with self._lock:
            self._frame = frame

    def get_current_frame(self) -> np.ndarray | None:
        with self._lock:
            if self._frame is not None:
                return self._frame.copy()
            return None


def make_well_frame(
    width=640, height=480, cx=320.0, cy=240.0, radius=100.0,
) -> np.ndarray:
    """Synthetic well image: bright circle on dark background."""
    img = np.full((height, width, 3), 40, dtype=np.uint8)
    cv2.circle(img, (int(cx), int(cy)), int(radius), (200, 200, 200), -1)
    cv2.circle(img, (int(cx), int(cy)), int(radius), (240, 240, 240), 3)
    return img


def make_needle_frame(
    width=640, height=480, cx=320.0, cy=240.0, radius=30.0,
) -> np.ndarray:
    """Synthetic needle image: dark circle on bright background."""
    img = np.full((height, width, 3), 200, dtype=np.uint8)
    cv2.circle(img, (int(cx), int(cy)), int(radius), (40, 40, 40), -1)
    return img


# ---------------------------------------------------------------------------
# CameraWidget.get_current_frame() Tests
# ---------------------------------------------------------------------------

class TestGetCurrentFrame(unittest.TestCase):
    """Test thread-safe frame access via MockCameraWidget (same interface as CameraWidget)."""

    def test_returns_none_when_no_frame(self):
        mock = MockCameraWidget()
        self.assertIsNone(mock.get_current_frame())

    def test_returns_copy_not_reference(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mock = MockCameraWidget(frame)

        f1 = mock.get_current_frame()
        f2 = mock.get_current_frame()

        # Should be equal but not the same object
        np.testing.assert_array_equal(f1, f2)
        self.assertFalse(f1 is f2, "Should return copies, not references")

    def test_frame_update_visible(self):
        mock = MockCameraWidget()
        mock.set_frame(np.full((100, 100, 3), 50, dtype=np.uint8))

        f1 = mock.get_current_frame()
        self.assertEqual(f1[0, 0, 0], 50)

        mock.set_frame(np.full((100, 100, 3), 200, dtype=np.uint8))
        f2 = mock.get_current_frame()
        self.assertEqual(f2[0, 0, 0], 200)

    def test_concurrent_access(self):
        """Multiple threads can read frames concurrently without crashes."""
        frame = make_well_frame()
        mock = MockCameraWidget(frame)
        errors = []

        def reader():
            try:
                for _ in range(100):
                    f = mock.get_current_frame()
                    if f is not None:
                        _ = f.shape
            except Exception as e:
                errors.append(e)

        def writer():
            try:
                for _ in range(100):
                    mock.set_frame(make_well_frame())
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=reader) for _ in range(4)
        ] + [threading.Thread(target=writer)]

        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        self.assertEqual(errors, [], f"Concurrent access errors: {errors}")


# ---------------------------------------------------------------------------
# DetectionWorker Tests (without Qt event loop)
# ---------------------------------------------------------------------------

class TestDetectionWorkerConfig(unittest.TestCase):
    """Test DetectionWorker configuration (no threading needed)."""

    def test_initial_mode_is_idle(self):
        worker = DetectionWorker()
        self.assertEqual(worker.mode, DetectionMode.IDLE)

    def test_set_mode(self):
        worker = DetectionWorker()
        worker.set_mode(DetectionMode.WELL_DETECT, expected_diameter_px=200.0)
        self.assertEqual(worker.mode, DetectionMode.WELL_DETECT)

    def test_set_camera_widget(self):
        mock = MockCameraWidget()
        worker = DetectionWorker()
        worker.set_camera_widget(mock)
        self.assertIs(worker._camera_widget, mock)

    def test_detection_mode_enum(self):
        """All expected modes exist."""
        modes = [DetectionMode.IDLE, DetectionMode.WELL_DETECT,
                 DetectionMode.NEEDLE_DETECT, DetectionMode.FOCUS_ASSIST]
        self.assertEqual(len(modes), 4)

    def test_well_params(self):
        worker = DetectionWorker()
        worker.set_well_params(tolerance=0.5, param1=80.0, param2=40.0)
        self.assertEqual(worker._well_tolerance, 0.5)
        self.assertEqual(worker._well_param1, 80.0)
        self.assertEqual(worker._well_param2, 40.0)

    def test_needle_params(self):
        worker = DetectionWorker()
        worker.set_needle_params(tolerance=0.6, min_circularity=0.4)
        self.assertEqual(worker._needle_tolerance, 0.6)
        self.assertEqual(worker._needle_min_circularity, 0.4)


# ---------------------------------------------------------------------------
# Detection Throughput Tests
# ---------------------------------------------------------------------------

class TestDetectionThroughput(unittest.TestCase):
    """Verify detection algorithms meet the <50ms per frame target."""

    def _time_detection(self, func, iterations=20):
        """Run a detection function multiple times and return avg ms."""
        times = []
        for _ in range(iterations):
            start = time.perf_counter()
            func()
            elapsed = (time.perf_counter() - start) * 1000.0
            times.append(elapsed)
        return sum(times) / len(times)

    def test_well_detection_throughput(self):
        """Well detection should average <50ms per frame on 640×480."""
        frame = make_well_frame()
        avg_ms = self._time_detection(
            lambda: WellDetector.detect_well(frame, expected_diameter_px=200.0),
        )
        print(f"\n  Well detection avg: {avg_ms:.1f}ms")
        self.assertLess(avg_ms, 50.0,
                        f"Well detection too slow: {avg_ms:.1f}ms (target <50ms)")

    def test_well_detection_with_fallback_throughput(self):
        """Well detection with fallback should average <80ms per frame."""
        frame = make_well_frame()
        avg_ms = self._time_detection(
            lambda: WellDetector.detect_well_with_fallback(
                frame, expected_diameter_px=200.0
            ),
        )
        print(f"\n  Well + fallback avg: {avg_ms:.1f}ms")
        self.assertLess(avg_ms, 80.0,
                        f"Well+fallback too slow: {avg_ms:.1f}ms (target <80ms)")

    def test_needle_detection_throughput(self):
        """Needle detection should average <50ms per frame on 640×480."""
        frame = make_needle_frame()
        avg_ms = self._time_detection(
            lambda: NeedleDetector.detect_needle(frame, expected_od_px=60.0),
        )
        print(f"\n  Needle detection avg: {avg_ms:.1f}ms")
        self.assertLess(avg_ms, 50.0,
                        f"Needle detection too slow: {avg_ms:.1f}ms (target <50ms)")

    def test_focus_score_throughput(self):
        """Focus scoring should average <20ms per frame on 640×480."""
        frame = make_well_frame()
        avg_ms = self._time_detection(
            lambda: NeedleDetector.compute_focus_score(frame),
        )
        print(f"\n  Focus score avg: {avg_ms:.1f}ms")
        self.assertLess(avg_ms, 20.0,
                        f"Focus scoring too slow: {avg_ms:.1f}ms (target <20ms)")

    def test_full_resolution_throughput(self):
        """Detection at 916×686 (preview resolution) should be <100ms."""
        frame = np.full((686, 916, 3), 40, dtype=np.uint8)
        cv2.circle(frame, (458, 343), 150, (200, 200, 200), -1)

        avg_ms = self._time_detection(
            lambda: WellDetector.detect_well(frame, expected_diameter_px=300.0),
        )
        print(f"\n  Well detection at 916×686 avg: {avg_ms:.1f}ms")
        self.assertLess(avg_ms, 100.0,
                        f"Full-res detection too slow: {avg_ms:.1f}ms (target <100ms)")


if __name__ == "__main__":
    unittest.main()
