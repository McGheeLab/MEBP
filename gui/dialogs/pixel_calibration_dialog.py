"""
pixel_calibration_dialog.py — Modal dialog for empirical µm/px calibration.

v7.3.3: Measures the actual µm/px ratio by moving the stage a known distance
and correlating the resulting pixel displacement between two captured frames.

Workflow:
    1. Capture frame at current position
    2. Move stage a known distance (user-configurable)
    3. Wait for settlement, capture second frame
    4. Phase-correlate to measure pixel displacement
    5. µm/px = distance_moved / pixel_displacement

Requires a running camera and connected stage controller.
"""

from __future__ import annotations

import logging
import math
from enum import Enum, auto

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QComboBox, QPushButton,
    QDialogButtonBox, QGroupBox, QMessageBox,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)

try:
    from SupportClasses.VisionDetector import measure_pixel_displacement
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    logger.warning("VisionDetector not available — pixel calibration disabled")


class _CalState(Enum):
    """Calibration state machine."""
    READY = auto()
    CAPTURING_BEFORE = auto()
    MOVING = auto()
    SETTLING = auto()
    CAPTURING_AFTER = auto()
    RESULT = auto()
    ERROR = auto()


class PixelCalibrationDialog(QDialog):
    """Modal dialog for empirical camera µm/px calibration via stage movement."""

    def __init__(self, camera_manager, controller, cam_idx: int = 0,
                 parent=None):
        super().__init__(parent)
        self._camera_manager = camera_manager
        self._controller = controller
        self._cam_idx = cam_idx
        self._state = _CalState.READY

        self._frame_before = None
        self._frame_after = None
        self.result_um_per_px: float | None = None

        self.setWindowTitle("Calibrate µm/px")
        self.setMinimumWidth(420)
        self.setStyleSheet(f"background-color: {COLORS['base']}; "
                           f"color: {COLORS['text']};")

        self._build_ui()

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # Instructions
        instr = QLabel(
            "Calibrate the actual µm/px by moving the stage a known distance "
            "and measuring pixel displacement via phase correlation.\n\n"
            "Ensure a textured sample is in view (not a blank field).")
        instr.setWordWrap(True)
        instr.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(instr)

        # Settings group
        settings_grp = QGroupBox("Settings")
        settings_grp.setStyleSheet(
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: 8px; padding-top: 14px; "
            f"color: {COLORS['text']}; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; padding: 2px 6px; }}")
        form = QFormLayout(settings_grp)

        self._spin_distance = QDoubleSpinBox()
        self._spin_distance.setRange(50.0, 2000.0)
        self._spin_distance.setValue(200.0)
        self._spin_distance.setSuffix(" µm")
        self._spin_distance.setDecimals(0)
        form.addRow("Move distance:", self._spin_distance)

        self._combo_axis = QComboBox()
        self._combo_axis.addItems(["X axis", "Y axis"])
        form.addRow("Move axis:", self._combo_axis)

        self._spin_settle = QDoubleSpinBox()
        self._spin_settle.setRange(200, 3000)
        self._spin_settle.setValue(500)
        self._spin_settle.setSuffix(" ms")
        self._spin_settle.setDecimals(0)
        form.addRow("Settlement time:", self._spin_settle)

        layout.addWidget(settings_grp)

        # Status label
        self._lbl_status = QLabel("Ready — click Start to begin calibration.")
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 9pt; padding: 4px;")
        layout.addWidget(self._lbl_status)

        # Result display (hidden until result)
        self._result_group = QGroupBox("Result")
        self._result_group.setStyleSheet(
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: 8px; padding-top: 14px; "
            f"color: {COLORS['text']}; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; padding: 2px 6px; }}")
        result_form = QFormLayout(self._result_group)

        self._lbl_displacement = QLabel("—")
        result_form.addRow("Pixel displacement:", self._lbl_displacement)
        self._lbl_confidence = QLabel("—")
        result_form.addRow("Confidence:", self._lbl_confidence)
        self._lbl_umpx = QLabel("—")
        self._lbl_umpx.setStyleSheet(
            f"color: {COLORS['green']}; font-weight: bold; font-size: 11pt;")
        result_form.addRow("Computed µm/px:", self._lbl_umpx)

        self._result_group.setVisible(False)
        layout.addWidget(self._result_group)

        layout.addStretch()

        # Buttons
        btn_layout = QHBoxLayout()

        self._btn_start = QPushButton("Start")
        self._btn_start.setStyleSheet(
            f"QPushButton {{ background-color: {COLORS['blue']}; "
            f"color: {COLORS['base']}; padding: 6px 16px; "
            f"border-radius: 4px; font-weight: bold; }}")
        self._btn_start.clicked.connect(self._start_calibration)
        btn_layout.addWidget(self._btn_start)

        self._btn_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setText("Accept")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        self._btn_box.accepted.connect(self._accept_result)
        self._btn_box.rejected.connect(self.reject)
        btn_layout.addWidget(self._btn_box)

        layout.addLayout(btn_layout)

    # ── State machine ─────────────────────────────────────────────

    def _set_status(self, text: str, color: str = "yellow"):
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(
            f"color: {COLORS[color]}; font-size: 9pt; padding: 4px;")

    def _start_calibration(self):
        """Begin the calibration sequence."""
        if not VISION_AVAILABLE:
            QMessageBox.warning(self, "Unavailable",
                                "Vision module not available.")
            return

        mgr = self._camera_manager
        if mgr is None or not mgr.is_running(self._cam_idx):
            QMessageBox.warning(self, "Camera Required",
                                "Start the camera before calibrating.")
            return

        if self._controller is None:
            QMessageBox.warning(self, "Stage Required",
                                "Stage controller is not connected.")
            return

        self._state = _CalState.CAPTURING_BEFORE
        self._btn_start.setEnabled(False)
        self._result_group.setVisible(False)
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        self._set_status("Capturing frame 1...")

        # Capture frame 1 — use a short delay to let the UI update
        QTimer.singleShot(50, self._capture_before)

    def _capture_before(self):
        """Capture the first frame, then initiate stage move."""
        cam_widget = self._camera_manager.cameras[self._cam_idx]
        self._frame_before = cam_widget.capture_fresh_frame()

        if self._frame_before is None:
            self._set_status("Failed to capture frame 1. Is the camera running?",
                             "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        # Move stage
        distance = self._spin_distance.value()
        axis = self._combo_axis.currentIndex()  # 0=X, 1=Y

        self._state = _CalState.MOVING
        self._set_status(
            f"Moving stage {distance:.0f} µm along "
            f"{'X' if axis == 0 else 'Y'}...")

        dx = distance if axis == 0 else 0.0
        dy = distance if axis == 1 else 0.0

        try:
            self._controller.move_xy_relative_um(dx, dy)
        except Exception as e:
            self._set_status(f"Stage move failed: {e}", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        # Wait for settlement
        self._state = _CalState.SETTLING
        settle_ms = int(self._spin_settle.value())
        self._set_status(f"Waiting {settle_ms} ms for stage to settle...")
        QTimer.singleShot(settle_ms, self._capture_after)

    def _capture_after(self):
        """Capture the second frame and compute displacement."""
        cam_widget = self._camera_manager.cameras[self._cam_idx]
        self._frame_after = cam_widget.capture_fresh_frame()

        if self._frame_after is None:
            self._set_status("Failed to capture frame 2.", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            self._move_back()
            return

        self._state = _CalState.CAPTURING_AFTER
        self._set_status("Computing displacement...")

        # Move stage back to original position
        self._move_back()

        # Compute displacement
        try:
            dx, dy, confidence = measure_pixel_displacement(
                self._frame_before, self._frame_after)
        except Exception as e:
            self._set_status(f"Phase correlation failed: {e}", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        # Compute result
        magnitude = math.sqrt(dx * dx + dy * dy)
        distance_um = self._spin_distance.value()

        if magnitude < 3.0:
            self._set_status(
                f"Pixel displacement too small ({magnitude:.1f} px). "
                f"Try increasing the move distance or ensure textured "
                f"content is in view.", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        if confidence < 0.15:
            self._set_status(
                f"Low confidence ({confidence:.2f}). The field may be too "
                f"featureless. Ensure a sample or textured surface is in view.",
                "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        um_per_px = distance_um / magnitude

        # Display result
        self._state = _CalState.RESULT
        self._lbl_displacement.setText(
            f"dx={dx:.2f} px, dy={dy:.2f} px  (magnitude: {magnitude:.2f} px)")
        self._lbl_confidence.setText(f"{confidence:.3f}")

        conf_color = "green" if confidence >= 0.3 else "yellow"
        self._lbl_confidence.setStyleSheet(
            f"color: {COLORS[conf_color]}; font-weight: bold;")

        self._lbl_umpx.setText(f"{um_per_px:.4f} µm/px")
        self._result_group.setVisible(True)

        self.result_um_per_px = um_per_px

        self._set_status("Calibration complete. Accept to use this value.",
                         "green")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(True)
        self._btn_start.setEnabled(True)
        self._btn_start.setText("Retry")

    def _move_back(self):
        """Move stage back to the original position."""
        distance = self._spin_distance.value()
        axis = self._combo_axis.currentIndex()
        dx = -distance if axis == 0 else 0.0
        dy = -distance if axis == 1 else 0.0
        try:
            self._controller.move_xy_relative_um(dx, dy)
        except Exception as e:
            logger.warning(f"Failed to move stage back: {e}")

    def _accept_result(self):
        """Accept the calibration result and close."""
        if self.result_um_per_px is not None:
            logger.info(f"Pixel calibration accepted: {self.result_um_per_px:.4f} µm/px")
            self.accept()
        else:
            self.reject()
