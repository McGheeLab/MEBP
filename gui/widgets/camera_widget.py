"""
Camera Widget — Live microscope camera feed for calibration verification.

Provides a live camera feed widget that can be embedded in any page.
Uses OpenCV (cv2) for camera capture and converts frames to QImage
for display in a QLabel.

Features:
    - Auto-detect available cameras
    - Live feed with configurable FPS
    - Software brightness / gamma adjustment (hardware-independent)
    - Crosshair overlay for needle alignment
    - Snapshot capture (save to file)
    - Compact mode for multi-camera layouts

Falls back gracefully if OpenCV is not installed.

Usage::

    from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE
    cam = CameraWidget(camera_label="Cam 1", compact=True, parent=self)
    cam.start_with_index(0)
    cam.set_brightness(20)
    cam.set_gamma(1.5)
    cam.stop()
"""

from __future__ import annotations

import logging
from pathlib import Path
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QCheckBox, QGroupBox, QFileDialog, QSlider,
    QFrame, QSizePolicy,
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor

logger = logging.getLogger(__name__)

# Try to import OpenCV
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logger.info("OpenCV (cv2) not installed — camera widget disabled")

# v7.3-camera: Try to import ToupCam backend
try:
    from gui.widgets.toupcam_backend import ToupCamBackend, TOUPCAM_AVAILABLE
except ImportError:
    TOUPCAM_AVAILABLE = False
    ToupCamBackend = None
    logger.info("ToupCam backend not available")

# v7.3-camera: Unified availability flag
CAMERA_AVAILABLE = CV2_AVAILABLE or bool(TOUPCAM_AVAILABLE)



def detect_cameras(max_index: int = 4) -> list[int]:
    """Probe camera indices and return those that are available.

    v7.2.5 S7: Reduced max_index 8->4, suppresses OpenCV errors,
    stops after 2 consecutive failures for speed.
    """
    if not CV2_AVAILABLE:
        return []

    # Suppress OpenCV error spam during probing
    old_log_level = None
    try:
        old_log_level = cv2.getLogLevel()
        cv2.setLogLevel(0)  # SILENT
    except (AttributeError, cv2.error):
        pass

    available = []
    consecutive_fails = 0
    try:
        for idx in range(max_index):
            try:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    available.append(idx)
                    cap.release()
                    consecutive_fails = 0
                else:
                    consecutive_fails += 1
                    if consecutive_fails >= 2:
                        break
            except Exception:
                consecutive_fails += 1
                if consecutive_fails >= 2:
                    break
    finally:
        if old_log_level is not None:
            try:
                cv2.setLogLevel(old_log_level)
            except (AttributeError, cv2.error):
                pass

    logger.info(f"Camera detection: found {len(available)} camera(s) "
               f"at indices {available}")
    return available


def detect_toupcam_cameras() -> list[dict]:
    """v7.3-camera: Detect ToupTek/Bestscope cameras.
    
    Returns list of dicts with 'id' and 'displayname' keys.
    """
    if not TOUPCAM_AVAILABLE or ToupCamBackend is None:
        return []
    try:
        return ToupCamBackend.enumerate()
    except Exception as e:
        logger.warning(f"ToupCam detection error: {e}")
        return []




def detect_cameras_async(callback, max_index: int = 4):
    """
    v7.2.6: Detect cameras in a background thread.

    Args:
        callback: Called with list[int] of found camera indices.
                  Called from background thread — use QTimer.singleShot(0, fn)
                  to marshal back to main thread.
        max_index: Max camera index to probe.
    """
    import threading

    def _worker():
        indices = detect_cameras(max_index)
        callback(indices)

    t = threading.Thread(target=_worker, daemon=True, name="CameraDetect")
    t.start()
    return t


class CameraWidget(QWidget):
    """
    Live camera feed widget with crosshair overlay and software image
    adjustments (brightness, gamma).

    Args:
        camera_label: Display name shown in the header (e.g. "Camera 1").
        compact:      If True, uses a smaller minimum size suitable for
                      multi-camera grid layouts.
        show_controls: If True, show the built-in control row.  When False,
                       the camera is controlled externally (e.g. from a
                       context-panel).

    Emits:
        frame_captured(QImage): Every time a new frame is captured.
    """

    frame_captured = Signal(object)  # QImage

    def __init__(
        self,
        camera_label: str = "Camera",
        compact: bool = False,
        show_controls: bool = True,
        parent=None,
    ):
        super().__init__(parent)
        self._capture = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._grab_frame)
        self._camera_index = 0
        self._show_crosshair = True
        self._fps = 15
        self._running = False
        self._camera_label = camera_label
        self._compact = compact

        # Software image adjustments
        self._brightness: int = 0       # -100 … +100
        self._gamma: float = 1.0        # 0.1 … 3.0
        self._gamma_lut = None          # Precomputed LUT for speed

        # v7.3-camera: ToupCam backend state
        self._toupcam = None
        self._backend_type = "opencv"  # "opencv" or "toupcam"

        self._setup_ui(show_controls)

    # ── UI Construction ──────────────────────────────────────────

    def _setup_ui(self, show_controls: bool):
        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 0, 0, 0)

        if not CAMERA_AVAILABLE:
            layout.addWidget(QLabel(
                "Camera unavailable — install OpenCV or ToupTek SDK:\n"
                "  pip install opencv-python"
            ))
            return

        if show_controls:
            # Header / controls row
            header = QHBoxLayout()
            header.setSpacing(4)

            header.addWidget(QLabel(f"<b>{self._camera_label}</b>"))

            header.addWidget(QLabel("Src:"))
            self.camera_combo = QComboBox()
            self.camera_combo.setMaximumWidth(100)
            self._populate_cameras()
            header.addWidget(self.camera_combo)

            self.btn_start = QPushButton("▶")
            self.btn_start.setFixedWidth(32)
            self.btn_start.setToolTip("Start / Stop camera")
            self.btn_start.clicked.connect(self.toggle)
            header.addWidget(self.btn_start)

            self.chk_crosshair = QCheckBox("✛")
            self.chk_crosshair.setChecked(True)
            self.chk_crosshair.setToolTip("Toggle crosshair overlay")
            self.chk_crosshair.toggled.connect(self._on_crosshair_toggle)
            header.addWidget(self.chk_crosshair)

            btn_snap = QPushButton("📷")
            btn_snap.setFixedWidth(32)
            btn_snap.setToolTip("Save snapshot")
            btn_snap.clicked.connect(self.take_snapshot)
            header.addWidget(btn_snap)

            header.addStretch()
            layout.addLayout(header)
        else:
            # Even without controls, provide a combo for internal use
            self.camera_combo = QComboBox()
            self.camera_combo.setVisible(False)
            self._populate_cameras()
            layout.addWidget(self.camera_combo)

        # Video display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if self._compact:
            self.video_label.setMinimumSize(200, 150)
        else:
            self.video_label.setMinimumSize(320, 240)
        self.video_label.setStyleSheet(
            "background-color: #181825; border: 1px solid #45475a;"
        )
        self.video_label.setText(f"{self._camera_label} — stopped")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.video_label, stretch=1)

    def _populate_cameras(self):
        """Show placeholder in camera combo -- no hardware probe.

        v7.2.5 S7: Camera detection is now lazy. This method just
        sets a placeholder. Call refresh_cameras() or start() to
        actually probe hardware.
        """
        self.camera_combo.clear()
        if not CV2_AVAILABLE:
            return
        self._cameras_detected = False
        self.camera_combo.addItem("Click Detect or Start", -1)

    def refresh_cameras(self):
        """Detect cameras and populate the combo.

        v7.3-camera: Detects both OpenCV and ToupCam cameras.
        Combo item data is a tuple: ("opencv", index) or ("toupcam", device_id).
        """
        self.camera_combo.clear()
        if not CAMERA_AVAILABLE:
            return

        # OpenCV cameras
        if CV2_AVAILABLE:
            for idx in detect_cameras():
                self.camera_combo.addItem(f"CV2: Camera {idx}", ("opencv", idx))

        # ToupCam cameras  (v7.3-camera)
        for tc_dev in detect_toupcam_cameras():
            name = tc_dev.get('displayname', 'ToupCam')
            dev_id = tc_dev.get('id', '')
            self.camera_combo.addItem(f"TC: {name}", ("toupcam", dev_id))

        if self.camera_combo.count() == 0:
            self.camera_combo.addItem("No cameras found", -1)
        self._cameras_detected = True

    def start(self):
        """Start the camera feed using the currently selected combo index.

        v7.2.5 S7: Triggers lazy camera detection on first start.
        """
        # v7.3-camera: Support either backend
        if not CAMERA_AVAILABLE or self._running:
            return

        # Lazy detection: probe hardware on first start
        if not getattr(self, "_cameras_detected", False):
            self.refresh_cameras()

        cam_data = self.camera_combo.currentData()
        if cam_data is None or cam_data == -1:
            self.video_label.setText("No camera available")
            return

        # v7.3-camera: Route by backend type
        if isinstance(cam_data, tuple) and len(cam_data) == 2:
            backend_type, identifier = cam_data
            if backend_type == "toupcam":
                self._start_toupcam(identifier)
                return
            else:
                self.start_with_index(identifier)
                return

        # Legacy: plain integer index (backward compat)
        if isinstance(cam_data, int) and cam_data >= 0:
            self.start_with_index(cam_data)

    def start_with_index(self, camera_index: int):
        """Start the camera feed with a specific device index."""
        if not CV2_AVAILABLE or self._running:
            return

        self._capture = cv2.VideoCapture(camera_index)
        if not self._capture.isOpened():
            self.video_label.setText(f"Failed to open camera {camera_index}")
            self._capture = None
            return

        self._camera_index = camera_index
        self._running = True
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("⏹")
        logger.info(f"{self._camera_label}: camera {camera_index} started at {self._fps} FPS")

    def stop(self):
        """Stop the camera feed."""
        self._timer.stop()
        self._running = False
        if self._capture:
            self._capture.release()
            self._capture = None
        self.video_label.setText(f"{self._camera_label} — stopped")
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("▶")
        logger.info(f"{self._camera_label}: camera stopped")

    def _start_toupcam(self, device_id: str):
        """v7.3-camera: Start a ToupCam camera feed."""
        if not TOUPCAM_AVAILABLE or ToupCamBackend is None or self._running:
            return

        self._toupcam = ToupCamBackend()
        if not self._toupcam.open(device_id):
            self.video_label.setText(f"Failed to open ToupCam")
            self._toupcam = None
            return

        w, h = self._toupcam.get_resolution()
        self._running = True
        self._backend_type = "toupcam"
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("\u23f9")
        logger.info(f"{self._camera_label}: ToupCam started ({w}x{h}) at {self._fps} FPS")


    def toggle(self):
        """Toggle camera on/off."""
        if self._running:
            self.stop()
        else:
            self.start()

    # ── Image Adjustment Properties ───────────────────────────────

    def set_brightness(self, value: int):
        """Set software brightness offset (-100 to +100)."""
        self._brightness = max(-100, min(100, value))

    def set_gamma(self, value: float):
        """Set software gamma (0.1 to 3.0).  1.0 = no change."""
        self._gamma = max(0.1, min(3.0, value))
        # Precompute a lookup table for speed
        inv_gamma = 1.0 / self._gamma
        self._gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in range(256)]
        ).astype("uint8")

    @property
    def brightness(self) -> int:
        return self._brightness

    @property
    def gamma(self) -> float:
        return self._gamma

    # ── Frame Capture ─────────────────────────────────────────────

    def _grab_frame(self):
        """Capture, adjust, and display one frame.

        v7.3-camera: Reads from OpenCV or ToupCam backend.
        """
        # v7.3-camera: Dual backend read
        backend = getattr(self, '_backend_type', 'opencv')
        if backend == 'toupcam':
            tc = getattr(self, '_toupcam', None)
            if tc is None or not tc.isOpened():
                self.stop()
                return
            ret, frame = tc.read()
        else:
            if not self._capture or not self._capture.isOpened():
                self.stop()
                return
            ret, frame = self._capture.read()

        if not ret or frame is None:
            return

        # Import numpy/cv2 for processing
        try:
            import cv2
            import numpy as np
        except ImportError:
            return

        # Convert BGR -> RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Apply software brightness
        if self._brightness != 0:
            rgb = cv2.convertScaleAbs(rgb, alpha=1.0, beta=self._brightness)

        # Apply software gamma via LUT
        if self._gamma_lut is not None and abs(self._gamma - 1.0) > 0.01:
            rgb = cv2.LUT(rgb, self._gamma_lut)

        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        # Make a copy so the numpy buffer stays valid for QImage
        q_img = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        # Draw crosshair overlay
        pixmap = QPixmap.fromImage(q_img)
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # Scale to fit label
        scaled = pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)

        self.frame_captured.emit(q_img)


    def _draw_crosshair(self, pixmap: QPixmap):
        """Draw a crosshair overlay on the pixmap."""
        painter = QPainter(pixmap)
        pen = QPen(QColor("#f38ba8"), 1, Qt.PenStyle.DashLine)
        painter.setPen(pen)

        cx = pixmap.width() // 2
        cy = pixmap.height() // 2

        # Horizontal line
        painter.drawLine(0, cy, pixmap.width(), cy)
        # Vertical line
        painter.drawLine(cx, 0, cx, pixmap.height())

        # Center circle
        pen.setStyle(Qt.PenStyle.SolidLine)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawEllipse(cx - 15, cy - 15, 30, 30)

        painter.end()

    def _on_crosshair_toggle(self, checked: bool):
        self._show_crosshair = checked

    # ── Snapshot ──────────────────────────────────────────────────

    def take_snapshot(self):
        """Save current frame to file."""
        if not self._capture or not self._capture.isOpened():
            return

        ret, frame = self._capture.read()
        if not ret:
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"snapshot_{self._camera_label.replace(' ', '_')}_{ts}.png"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Snapshot", default_name,
            "Images (*.png *.jpg *.bmp)"
        )
        if filepath:
            cv2.imwrite(filepath, frame)
            logger.info(f"Snapshot saved to {filepath}")

    # ── Cleanup ───────────────────────────────────────────────────

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)

    @property
    def is_running(self) -> bool:
        return self._running
