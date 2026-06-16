"""
pixel_calibration_dialog.py — Modal dialog for empirical µm/px calibration.

v7.3.3: Measures the actual µm/px ratio by moving the stage a known distance
and correlating the resulting pixel displacement between two captured frames.

v7.5.x: Live view + selectable move direction. The needle cameras are 90°
apart from each other but the pair is mounted at ~45° to the stage X/Y axes,
so a stage move in the wrong direction drives the needle *along the camera's
optical axis* — it just goes in/out of focus and shows almost no lateral
motion. The dialog now shows the live feed and overlays the detected
phase-correlation displacement as an arrow, and lets the operator pick the
move direction (presets + free angle) until they get strong lateral motion.
The accepted direction is reported as the camera's in-plane rotation
(``result_rotation_deg``) for the needle-centering aligner.

Workflow:
    1. Capture frame at current position
    2. Move stage a known distance along the chosen direction
    3. Wait for settlement, capture second frame
    4. Phase-correlate to measure pixel displacement (shown as an arrow)
    5. µm/px = distance_moved / pixel_displacement_magnitude

Requires a running camera and connected stage controller.
"""

from __future__ import annotations

import logging
import math
from enum import Enum, auto

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QPushButton,
    QDialogButtonBox, QGroupBox, QMessageBox, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size

logger = logging.getLogger(__name__)

try:
    from SupportClasses.VisionDetector import measure_pixel_displacement
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    logger.warning("VisionDetector not available — pixel calibration disabled")

try:
    from gui.widgets.camera_feed_view import CameraFeedView
    FEED_AVAILABLE = True
except ImportError:
    CameraFeedView = None
    FEED_AVAILABLE = False


class _CalState(Enum):
    """Calibration state machine."""
    READY = auto()
    CAPTURING_BEFORE = auto()
    MOVING = auto()
    SETTLING = auto()
    CAPTURING_AFTER = auto()
    RESULT = auto()
    ERROR = auto()


# Move-direction presets (stage-frame angle, degrees CCW from +X).
_DIRECTION_PRESETS = [
    ("X →", 0.0),
    ("Y ↑", 90.0),
    ("Diag ↗", 45.0),
    ("Diag ↘", -45.0),
]


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
        # v7.5.x: the accepted move direction = the camera's in-plane lateral
        # stage direction (deg from +X), fed to the needle-centering aligner.
        self.result_rotation_deg: float | None = None

        self.setWindowTitle("Calibrate µm/px")
        self.setMinimumWidth(s(900))
        self.setMinimumHeight(s(520))
        self.setStyleSheet(f"background-color: {COLORS['base']}; "
                           f"color: {COLORS['text']};")

        self._build_ui()
        self._start_feed()

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self):
        outer = QHBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(12))

        # ── Left: live feed ───────────────────────────────────────
        if FEED_AVAILABLE and self._camera_manager is not None:
            self._feed = CameraFeedView(
                camera_manager=self._camera_manager,
                cam_idx=self._cam_idx,
                show_crosshair=True,
                label=f"Camera {self._cam_idx + 1} — live",
            )
            self._feed.setMinimumSize(s(480), s(380))
            outer.addWidget(self._feed, stretch=1)
        else:
            self._feed = None
            placeholder = QLabel("Live view unavailable")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setMinimumSize(s(480), s(380))
            placeholder.setStyleSheet(
                f"background-color: #181825; color: {COLORS['subtext0']};")
            outer.addWidget(placeholder, stretch=1)

        # ── Right: controls ───────────────────────────────────────
        side = QVBoxLayout()
        side.setSpacing(s(10))
        outer.addLayout(side, stretch=0)

        instr = QLabel(
            "Move the stage a known distance and measure the pixel shift "
            "(phase correlation). The detected motion is drawn as a green "
            "arrow on the feed.\n\n"
            "These cameras sit at ~45° to the X/Y axes — if a move just "
            "changes focus with little arrow, the needle is moving along "
            "the camera's optical axis. Pick the direction that gives the "
            "longest arrow.")
        instr.setWordWrap(True)
        instr.setMaximumWidth(s(340))
        instr.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        side.addWidget(instr)

        # Settings group
        settings_grp = QGroupBox("Settings")
        settings_grp.setStyleSheet(self._group_style())
        form = QFormLayout(settings_grp)

        self._spin_distance = QDoubleSpinBox()
        self._spin_distance.setRange(50.0, 2000.0)
        self._spin_distance.setValue(200.0)
        self._spin_distance.setSuffix(" µm")
        self._spin_distance.setDecimals(0)
        form.addRow("Move distance:", self._spin_distance)

        self._spin_direction = QDoubleSpinBox()
        self._spin_direction.setRange(-180.0, 180.0)
        self._spin_direction.setValue(45.0)
        self._spin_direction.setSuffix("°")
        self._spin_direction.setDecimals(1)
        self._spin_direction.setToolTip(
            "Stage move direction in the XY plane (0° = +X, 90° = +Y).")
        form.addRow("Move direction:", self._spin_direction)

        # Direction preset buttons.
        preset_row = QHBoxLayout()
        preset_row.setSpacing(s(4))
        for text, ang in _DIRECTION_PRESETS:
            b = QPushButton(text)
            b.setMaximumWidth(s(70))
            b.clicked.connect(
                lambda _c=False, a=ang: self._spin_direction.setValue(a))
            preset_row.addWidget(b)
        preset_holder = QWidget()
        preset_holder.setLayout(preset_row)
        form.addRow("Presets:", preset_holder)

        self._spin_settle = QDoubleSpinBox()
        self._spin_settle.setRange(200, 3000)
        self._spin_settle.setValue(500)
        self._spin_settle.setSuffix(" ms")
        self._spin_settle.setDecimals(0)
        form.addRow("Settlement time:", self._spin_settle)

        side.addWidget(settings_grp)

        # Status label
        self._lbl_status = QLabel("Ready — click Measure to move & detect.")
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setMaximumWidth(s(340))
        self._lbl_status.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: {scaled_font_size(9)}pt; "
            f"padding: 4px;")
        side.addWidget(self._lbl_status)

        # Result display
        self._result_group = QGroupBox("Result")
        self._result_group.setStyleSheet(self._group_style())
        result_form = QFormLayout(self._result_group)

        self._lbl_displacement = QLabel("—")
        result_form.addRow("Pixel displacement:", self._lbl_displacement)
        self._lbl_angle = QLabel("—")
        result_form.addRow("Detected angle:", self._lbl_angle)
        self._lbl_confidence = QLabel("—")
        result_form.addRow("Confidence:", self._lbl_confidence)
        self._lbl_umpx = QLabel("—")
        self._lbl_umpx.setStyleSheet(
            f"color: {COLORS['green']}; font-weight: bold; "
            f"font-size: {scaled_font_size(11)}pt;")
        result_form.addRow("Computed µm/px:", self._lbl_umpx)

        self._result_group.setVisible(False)
        side.addWidget(self._result_group)

        side.addStretch()

        # Buttons
        btn_layout = QHBoxLayout()
        self._btn_start = QPushButton("Measure")
        self._btn_start.setObjectName("accentBtn")
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

        side.addLayout(btn_layout)

    def _group_style(self) -> str:
        return (
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: 8px; padding-top: 14px; "
            f"color: {COLORS['text']}; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; "
            f"padding: 2px 6px; }}")

    # ── Camera feed lifecycle ─────────────────────────────────────

    def _start_feed(self):
        """Ensure the camera is running so the live feed shows."""
        mgr = self._camera_manager
        if mgr is None:
            return
        try:
            if not mgr.is_running(self._cam_idx):
                mgr.start(self._cam_idx)
        except Exception as e:
            logger.debug(f"PixelCalibrationDialog: feed start skipped — {e}")

    # ── State machine ─────────────────────────────────────────────

    def _set_status(self, text: str, color: str = "yellow"):
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(
            f"color: {COLORS[color]}; font-size: {scaled_font_size(9)}pt; "
            f"padding: 4px;")

    def _move_vector(self) -> tuple[float, float]:
        """Stage (dx, dy) µm for the current distance + direction."""
        distance = self._spin_distance.value()
        theta = math.radians(self._spin_direction.value())
        return (distance * math.cos(theta), distance * math.sin(theta))

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
        if self._feed is not None:
            self._feed.set_overlay_vector(None, None)
        self._set_status("Capturing frame 1...")

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

        dx, dy = self._move_vector()
        self._state = _CalState.MOVING
        self._set_status(
            f"Moving stage {self._spin_distance.value():.0f} µm at "
            f"{self._spin_direction.value():.0f}° "
            f"(dx={dx:.0f}, dy={dy:.0f})...")

        try:
            self._controller.move_xy_relative_um(dx, dy)
        except Exception as e:
            self._set_status(f"Stage move failed: {e}", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

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
        self._move_back()

        try:
            dx, dy, confidence = measure_pixel_displacement(
                self._frame_before, self._frame_after)
        except Exception as e:
            self._set_status(f"Phase correlation failed: {e}", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        magnitude = math.sqrt(dx * dx + dy * dy)
        distance_um = self._spin_distance.value()

        # Always show the detected vector so the operator can see the motion.
        if self._feed is not None:
            self._feed.set_overlay_vector(dx, dy, f"{magnitude:.0f}px")
        img_angle = math.degrees(math.atan2(dy, dx))
        self._lbl_displacement.setText(
            f"dx={dx:.2f}, dy={dy:.2f} px  (|d|={magnitude:.2f} px)")
        self._lbl_angle.setText(f"{img_angle:.1f}° (image)")
        self._lbl_confidence.setText(f"{confidence:.3f}")
        self._result_group.setVisible(True)

        if magnitude < 3.0:
            self._set_status(
                f"Very little lateral motion ({magnitude:.1f} px). The needle "
                f"is likely moving along this camera's optical axis (in/out of "
                f"focus). Try a different move direction (e.g. ±45°).", "red")
            self._lbl_umpx.setText("—")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            self._btn_start.setText("Measure")
            return

        if confidence < 0.15:
            self._set_status(
                f"Low confidence ({confidence:.2f}). The field may be too "
                f"featureless, or the motion is mostly out-of-focus. Try a "
                f"different direction or a textured target.", "red")
            self._lbl_umpx.setText("—")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            self._btn_start.setText("Measure")
            return

        um_per_px = distance_um / magnitude
        self._state = _CalState.RESULT

        conf_color = "green" if confidence >= 0.3 else "yellow"
        self._lbl_confidence.setStyleSheet(
            f"color: {COLORS[conf_color]}; font-weight: bold;")
        self._lbl_umpx.setText(f"{um_per_px:.4f} µm/px")

        self.result_um_per_px = um_per_px
        self.result_rotation_deg = float(self._spin_direction.value())

        self._set_status(
            "Good lateral motion — Accept to use this µm/px and direction, "
            "or try other directions to compare.", "green")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(True)
        self._btn_start.setEnabled(True)
        self._btn_start.setText("Re-measure")

    def _move_back(self):
        """Move stage back to the original position."""
        dx, dy = self._move_vector()
        try:
            self._controller.move_xy_relative_um(-dx, -dy)
        except Exception as e:
            logger.warning(f"Failed to move stage back: {e}")

    def _accept_result(self):
        """Accept the calibration result and close."""
        if self.result_um_per_px is not None:
            logger.info(
                f"Pixel calibration accepted: {self.result_um_per_px:.4f} "
                f"µm/px @ {self.result_rotation_deg:.1f}°")
            self.accept()
        else:
            self.reject()
