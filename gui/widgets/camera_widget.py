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


def detect_cameras(max_index: int = 8) -> list[int]:
    """Probe camera indices and return those that are available."""
    if not CV2_AVAILABLE:
        return []
    available = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            available.append(idx)
            cap.release()
    return available


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

        self._setup_ui(show_controls)

    # ── UI Construction ──────────────────────────────────────────

    def _setup_ui(self, show_controls: bool):
        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 0, 0, 0)

        if not CV2_AVAILABLE:
            layout.addWidget(QLabel(
                "Camera unavailable — install OpenCV:\n"
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
        """Detect available cameras and populate the source combo."""
        self.camera_combo.clear()
        if not CV2_AVAILABLE:
            return
        for idx in detect_cameras():
            self.camera_combo.addItem(f"Camera {idx}", idx)
        if self.camera_combo.count() == 0:
            self.camera_combo.addItem("No cameras found", -1)

    def refresh_cameras(self):
        """Re-scan for cameras (can be called externally)."""
        self._populate_cameras()

    # ── Start / Stop ──────────────────────────────────────────────

    def start(self):
        """Start the camera feed using the currently selected combo index."""
        if not CV2_AVAILABLE or self._running:
            return

        idx = self.camera_combo.currentData()
        if idx is None or idx < 0:
            self.video_label.setText("No camera available")
            return
        self.start_with_index(idx)

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
        """Capture, adjust, and display one frame."""
        if not self._capture or not self._capture.isOpened():
            self.stop()
            return

        ret, frame = self._capture.read()
        if not ret:
            return

        # Convert BGR → RGB
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
