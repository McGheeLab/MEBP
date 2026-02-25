"""
Camera Widget — Live microscope camera feed for calibration verification.

Enhancement 2 from ARCHITECTURE.md task list.

Provides a live camera feed widget that can be embedded in the calibration
page or used standalone.  Uses OpenCV (cv2) for camera capture and converts
frames to QImage for display in a QLabel.

Features:
    - Auto-detect available cameras
    - Live feed with configurable FPS
    - Crosshair overlay for needle alignment
    - Snapshot capture (save to file)
    - Resolution selection

Falls back gracefully if OpenCV is not installed.

Usage::

    from gui.widgets.camera_widget import CameraWidget
    cam = CameraWidget(parent=self)
    cam.start()
    cam.stop()
"""

from __future__ import annotations

import logging
from pathlib import Path
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QCheckBox, QGroupBox, QFileDialog,
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


class CameraWidget(QWidget):
    """
    Live camera feed widget with crosshair overlay.

    Emits:
        frame_captured(QImage): Every time a new frame is captured.
    """

    frame_captured = Signal(object)  # QImage

    def __init__(self, parent=None):
        super().__init__(parent)
        self._capture = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._grab_frame)
        self._camera_index = 0
        self._show_crosshair = True
        self._fps = 15
        self._running = False

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        if not CV2_AVAILABLE:
            layout.addWidget(QLabel(
                "Camera unavailable — install OpenCV:\n"
                "  pip install opencv-python"
            ))
            return

        # Controls row
        controls = QHBoxLayout()

        controls.addWidget(QLabel("Camera:"))
        self.camera_combo = QComboBox()
        self._populate_cameras()
        controls.addWidget(self.camera_combo)

        self.btn_start = QPushButton("▶ Start")
        self.btn_start.clicked.connect(self.toggle)
        controls.addWidget(self.btn_start)

        self.chk_crosshair = QCheckBox("Crosshair")
        self.chk_crosshair.setChecked(True)
        self.chk_crosshair.toggled.connect(self._on_crosshair_toggle)
        controls.addWidget(self.chk_crosshair)

        btn_snapshot = QPushButton("📷 Snapshot")
        btn_snapshot.clicked.connect(self.take_snapshot)
        controls.addWidget(btn_snapshot)

        controls.addStretch()
        layout.addLayout(controls)

        # Video display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(320, 240)
        self.video_label.setStyleSheet(
            "background-color: #181825; border: 1px solid #45475a;"
        )
        self.video_label.setText("Camera stopped")
        layout.addWidget(self.video_label, stretch=1)

    def _populate_cameras(self):
        """Detect available cameras."""
        self.camera_combo.clear()
        if not CV2_AVAILABLE:
            return
        # Test first 5 camera indices
        for idx in range(5):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.camera_combo.addItem(f"Camera {idx}", idx)
                cap.release()
        if self.camera_combo.count() == 0:
            self.camera_combo.addItem("No cameras found", -1)

    # ── Start / Stop ──────────────────────────────────────────────

    def start(self):
        """Start the camera feed."""
        if not CV2_AVAILABLE or self._running:
            return

        idx = self.camera_combo.currentData()
        if idx is None or idx < 0:
            self.video_label.setText("No camera available")
            return

        self._capture = cv2.VideoCapture(idx)
        if not self._capture.isOpened():
            self.video_label.setText(f"Failed to open camera {idx}")
            self._capture = None
            return

        self._running = True
        self._timer.start(int(1000 / self._fps))
        self.btn_start.setText("⏹ Stop")
        logger.info(f"Camera {idx} started at {self._fps} FPS")

    def stop(self):
        """Stop the camera feed."""
        self._timer.stop()
        self._running = False
        if self._capture:
            self._capture.release()
            self._capture = None
        self.video_label.setText("Camera stopped")
        self.btn_start.setText("▶ Start")
        logger.info("Camera stopped")

    def toggle(self):
        """Toggle camera on/off."""
        if self._running:
            self.stop()
        else:
            self.start()

    # ── Frame Capture ─────────────────────────────────────────────

    def _grab_frame(self):
        """Capture and display one frame."""
        if not self._capture or not self._capture.isOpened():
            self.stop()
            return

        ret, frame = self._capture.read()
        if not ret:
            return

        # Convert BGR → RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        q_img = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        # Draw crosshair overlay
        if self._show_crosshair:
            pixmap = QPixmap.fromImage(q_img)
            self._draw_crosshair(pixmap)
        else:
            pixmap = QPixmap.fromImage(q_img)

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
        default_name = f"snapshot_{ts}.png"

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
