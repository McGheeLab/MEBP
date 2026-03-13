"""
DetectionWorker — QThread-based parallel vision processing for MEBP v7.3.0.

Pull-based architecture: the worker pulls the latest frame from
CameraWidget.get_current_frame() at its own pace, runs the appropriate
detector, and emits results via Qt signals. This naturally throttles
detection to whatever rate the algorithms can sustain without building
up a frame queue backlog.

Usage::

    worker = DetectionWorker(camera_widget)
    worker.well_detected.connect(on_well_found)
    worker.needle_detected.connect(on_needle_found)
    worker.focus_updated.connect(on_focus_update)

    worker.set_mode(DetectionMode.WELL_DETECT, expected_diameter_px=200.0)
    worker.start()

    # Later:
    worker.stop_detection()
    worker.wait()
"""

from __future__ import annotations

import logging
import time
from enum import Enum, auto

from PySide6.QtCore import QThread, Signal

logger = logging.getLogger(__name__)

try:
    import numpy as np
    from SupportClasses.VisionDetector import (
        DetectionResult,
        FocusResult,
        FocusTracker,
        WellDetector,
        NeedleDetector,
    )
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    logger.warning("VisionDetector not available — detection worker disabled")


class DetectionMode(Enum):
    """Operating mode for the detection worker."""
    IDLE = auto()            # Paused — no processing
    WELL_DETECT = auto()     # Detect circular well edges
    NEEDLE_DETECT = auto()   # Detect needle tip (dark circle)
    NEEDLE_DETECT_RELAXED = auto()  # v7.3.3: Relaxed needle detection (wide tolerance)
    FOCUS_ASSIST = auto()    # Compute focus quality score


class DetectionWorker(QThread):
    """
    Background thread that pulls frames from a CameraWidget and runs
    vision detection algorithms.

    Signals:
        well_detected(DetectionResult): Well circle found
        needle_detected(DetectionResult): Needle tip found
        focus_updated(FocusResult): New focus score computed
        detection_cleared(): Detection lost (no result this cycle)

    The worker does NOT receive every frame. It pulls the latest frame
    when ready, runs detection, emits the result, then immediately
    pulls the next frame. If detection takes 30ms and frames arrive
    at 15 FPS (~67ms), the worker naturally processes every other frame.
    """

    # Signals (use object type for dataclass payloads)
    well_detected = Signal(object)       # DetectionResult
    needle_detected = Signal(object)     # DetectionResult
    focus_updated = Signal(object)       # FocusResult
    detection_cleared = Signal()         # No detection this cycle

    def __init__(self, camera_widget=None, parent=None):
        """
        Args:
            camera_widget: Object with get_current_frame() -> np.ndarray | None.
                          Typically a CameraWidget instance.
            parent: QObject parent.
        """
        super().__init__(parent)
        self._camera_widget = camera_widget
        self._mode = DetectionMode.IDLE
        self._stop_flag = False

        # Detection parameters (set via set_mode or properties)
        self._expected_diameter_px: float = 0.0
        self._expected_od_px: float = 0.0
        self._focus_roi: tuple[int, int, int, int] | None = None

        # Hough tuning parameters
        self._well_tolerance: float = 0.3
        self._well_param1: float = 100.0
        self._well_param2: float = 30.0

        # Needle tuning parameters
        self._needle_tolerance: float = 0.4
        self._needle_min_circularity: float = 0.5

        # Focus tracking
        self._focus_tracker = FocusTracker() if VISION_AVAILABLE else None

        # Timing
        self._min_cycle_ms: float = 30.0  # Minimum cycle time to avoid CPU spin
        self._last_cycle_ms: float = 0.0  # Last cycle duration for diagnostics

    # ── Configuration ──────────────────────────────────────────

    @property
    def mode(self) -> DetectionMode:
        return self._mode

    @property
    def last_cycle_ms(self) -> float:
        """Duration of the last detection cycle in milliseconds."""
        return self._last_cycle_ms

    @property
    def focus_tracker(self) -> FocusTracker | None:
        return self._focus_tracker

    def set_camera_widget(self, camera_widget) -> None:
        """Set or change the camera widget (frame source)."""
        self._camera_widget = camera_widget

    def set_mode(
        self,
        mode: DetectionMode,
        expected_diameter_px: float = 0.0,
        expected_od_px: float = 0.0,
        focus_roi: tuple[int, int, int, int] | None = None,
    ) -> None:
        """
        Change detection mode and parameters.

        Can be called while the worker is running — takes effect on the
        next cycle.

        Args:
            mode: New detection mode
            expected_diameter_px: For WELL_DETECT — expected well diameter in px
            expected_od_px: For NEEDLE_DETECT — expected needle OD in px
            focus_roi: For FOCUS_ASSIST — (x, y, w, h) ROI or None for full frame
        """
        self._mode = mode
        if expected_diameter_px > 0:
            self._expected_diameter_px = expected_diameter_px
        if expected_od_px > 0:
            self._expected_od_px = expected_od_px
        self._focus_roi = focus_roi

        # Reset focus tracker on mode change to FOCUS_ASSIST
        if mode == DetectionMode.FOCUS_ASSIST and self._focus_tracker:
            self._focus_tracker.reset()

    def set_well_params(
        self,
        tolerance: float = 0.3,
        param1: float = 100.0,
        param2: float = 30.0,
    ) -> None:
        """Set HoughCircles tuning parameters for well detection."""
        self._well_tolerance = tolerance
        self._well_param1 = param1
        self._well_param2 = param2

    def set_needle_params(
        self,
        tolerance: float = 0.4,
        min_circularity: float = 0.5,
    ) -> None:
        """Set needle detection tuning parameters."""
        self._needle_tolerance = tolerance
        self._needle_min_circularity = min_circularity

    # ── Thread Lifecycle ───────────────────────────────────────

    def stop_detection(self) -> None:
        """Signal the worker to stop. Call wait() after to block until done."""
        self._stop_flag = True
        self._mode = DetectionMode.IDLE

    def run(self) -> None:
        """Main worker loop — pull frame, detect, emit, repeat."""
        if not VISION_AVAILABLE:
            logger.error("DetectionWorker: VisionDetector not available")
            return

        logger.info("DetectionWorker started")
        self._stop_flag = False

        while not self._stop_flag:
            cycle_start = time.perf_counter()

            if self._mode == DetectionMode.IDLE:
                # Idle — sleep and check again
                time.sleep(0.05)
                continue

            # Pull latest frame
            frame = self._get_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            # Run detection based on mode
            try:
                if self._mode == DetectionMode.WELL_DETECT:
                    self._detect_well(frame)
                elif self._mode == DetectionMode.NEEDLE_DETECT:
                    self._detect_needle(frame)
                elif self._mode == DetectionMode.NEEDLE_DETECT_RELAXED:
                    self._detect_needle_relaxed(frame)
                elif self._mode == DetectionMode.FOCUS_ASSIST:
                    self._compute_focus(frame)
            except Exception as e:
                logger.warning(f"DetectionWorker error: {e}")

            # Track cycle time
            elapsed_ms = (time.perf_counter() - cycle_start) * 1000.0
            self._last_cycle_ms = elapsed_ms

            # Throttle to avoid CPU spin on fast detections
            if elapsed_ms < self._min_cycle_ms:
                time.sleep((self._min_cycle_ms - elapsed_ms) / 1000.0)

        logger.info("DetectionWorker stopped")

    def _get_frame(self):
        """Pull the latest frame from the camera widget."""
        if self._camera_widget is None:
            return None
        try:
            return self._camera_widget.get_current_frame()
        except Exception:
            return None

    # ── Detection Methods ──────────────────────────────────────

    def _detect_well(self, frame) -> None:
        """Run well detection and emit result."""
        if self._expected_diameter_px <= 0:
            return

        result = WellDetector.detect_well_with_fallback(
            frame,
            expected_diameter_px=self._expected_diameter_px,
            tolerance=self._well_tolerance,
        )

        if result is not None:
            self.well_detected.emit(result)
        else:
            self.detection_cleared.emit()

    def _detect_needle(self, frame) -> None:
        """Run needle detection and emit result."""
        if self._expected_od_px <= 0:
            return

        result = NeedleDetector.detect_needle(
            frame,
            expected_od_px=self._expected_od_px,
            tolerance=self._needle_tolerance,
            min_circularity=self._needle_min_circularity,
        )

        if result is not None:
            self.needle_detected.emit(result)
        else:
            self.detection_cleared.emit()

    def _detect_needle_relaxed(self, frame) -> None:
        """Run relaxed needle detection (wide tolerance) and emit result."""
        result = NeedleDetector.detect_needle_relaxed(
            frame,
            expected_od_px=self._expected_od_px,  # may be 0 for unconstrained
            min_circularity=0.4,
        )

        if result is not None:
            self.needle_detected.emit(result)
        else:
            self.detection_cleared.emit()

    def _compute_focus(self, frame) -> None:
        """Compute focus score and emit normalized result."""
        raw = NeedleDetector.compute_focus_score(frame, roi_rect=self._focus_roi)

        if self._focus_tracker:
            result = self._focus_tracker.update(raw)
        else:
            result = raw

        self.focus_updated.emit(result)
