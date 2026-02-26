"""
Calibration Page — Multi-camera layout with calibration steps in context panel.

Main content: Position readout + up to 3 simultaneous camera feeds
Context panel (left box): Camera settings, plate config, 3-step calibration
    wizard (zero needle, teach plate, validate), calibration save/load

Camera feeds support software brightness and gamma adjustment which works
regardless of webcam hardware capabilities.
"""

from __future__ import annotations

import math
import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QFrame, QSizePolicy, QMessageBox, QCheckBox, QSlider,
    QScrollArea,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from SupportClasses.WellPlate import WellPlate, PLATE_DEFINITIONS
from gui.styles import COLORS

logger = logging.getLogger(__name__)

# Optional camera support
try:
    from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE, detect_cameras
except ImportError:
    CameraWidget = None
    CV2_AVAILABLE = False

    def detect_cameras(max_index=8):
        return []

# Maximum simultaneous cameras
MAX_CAMERAS = 3


class CalibrationPage(QWidget):
    """Multi-camera calibration page with steps in the context panel."""

    _page_title_text = "Calibration"

    def __init__(self, controller: StageController, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # Calibration state
        self._plate: WellPlate | None = None
        self._taught_a1: tuple[float, float] | None = None
        self._taught_corner: tuple[float, float] | None = None
        self._corner_well: str = "H12"
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._rotation = 0.0
        self._scale = 1.0

        # Camera widgets (up to MAX_CAMERAS)
        self._cameras: list[CameraWidget] = []
        self._context_widget = None

        self._setup_ui()

    def get_page_title(self) -> str:
        return "Calibration"

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL  (Steps 1-2-3 + Camera Settings + Plate Config)
    # ════════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        """Build the context panel with calibration steps and settings."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(5)

        # ── Camera Controls (global) ─────────────────────────────
        cam_label = QLabel("Camera Controls")
        cam_label.setObjectName("contextSectionLabel")
        layout.addWidget(cam_label)

        if CV2_AVAILABLE:
            # Brightness slider
            layout.addWidget(self._make_slider_row(
                "Brightness:", -100, 100, 0,
                "cam_brightness", self._on_brightness_changed,
            ))

            # Gamma slider
            layout.addWidget(self._make_slider_row(
                "Gamma:", 10, 300, 100,
                "cam_gamma", self._on_gamma_changed,
            ))

            # FPS
            fps_row = QHBoxLayout()
            fps_row.addWidget(QLabel("FPS:"))
            self._cam_fps_spin = QSpinBox()
            self._cam_fps_spin.setRange(1, 60)
            self._cam_fps_spin.setValue(15)
            self._cam_fps_spin.valueChanged.connect(self._on_fps_changed)
            fps_row.addWidget(self._cam_fps_spin)
            layout.addLayout(fps_row)

            # Crosshair toggle
            self._chk_crosshair = QCheckBox("Show Crosshair (all cameras)")
            self._chk_crosshair.setChecked(True)
            self._chk_crosshair.toggled.connect(self._on_crosshair_toggled)
            layout.addWidget(self._chk_crosshair)

            # Detect cameras button
            btn_detect = QPushButton("🔄 Detect Cameras")
            btn_detect.setObjectName("flatBtn")
            btn_detect.clicked.connect(self._refresh_all_cameras)
            layout.addWidget(btn_detect)
        else:
            layout.addWidget(QLabel(
                "Camera unavailable.\nInstall: pip install opencv-python"
            ))

        # ── Plate Configuration ──────────────────────────────────
        plate_label = QLabel("Plate Configuration")
        plate_label.setObjectName("contextSectionLabel")
        layout.addWidget(plate_label)

        plate_row = QHBoxLayout()
        plate_row.addWidget(QLabel("Format:"))
        self.ctx_plate_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            self.ctx_plate_combo.addItem(f"{fmt}-well", fmt)
        self.ctx_plate_combo.setCurrentIndex(4)  # 96-well default
        self.ctx_plate_combo.currentIndexChanged.connect(self._on_plate_changed)
        plate_row.addWidget(self.ctx_plate_combo, stretch=1)
        layout.addLayout(plate_row)

        # ── Step 1: Zero Needle ──────────────────────────────────
        s1_label = QLabel("Step 1 — Zero Needle")
        s1_label.setObjectName("contextSectionLabel")
        layout.addWidget(s1_label)

        layout.addWidget(QLabel("Jog to contact, then Set Zero."))

        s1_row = QHBoxLayout()
        btn_set_zero = QPushButton("Set Zero")
        btn_set_zero.setObjectName("successBtn")
        btn_set_zero.setMaximumHeight(26)
        btn_set_zero.clicked.connect(self._set_zero)
        s1_row.addWidget(btn_set_zero)

        btn_goto_zero = QPushButton("Go to Zero")
        btn_goto_zero.setMaximumHeight(26)
        btn_goto_zero.clicked.connect(self._goto_zero)
        s1_row.addWidget(btn_goto_zero)
        layout.addLayout(s1_row)

        self.lbl_zero_status = QLabel("Not set")
        self.lbl_zero_status.setStyleSheet(f"color: {COLORS['yellow']};")
        layout.addWidget(self.lbl_zero_status)

        # ── Step 2: Teach Plate Position ─────────────────────────
        s2_label = QLabel("Step 2 — Teach Plate")
        s2_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2_label)

        layout.addWidget(QLabel("Jog to A1 → Record, corner → Record."))

        # A1 row
        a1_row = QHBoxLayout()
        a1_row.addWidget(QLabel("A1:"))
        self.lbl_a1 = QLabel("—")
        self.lbl_a1.setStyleSheet(f"color: {COLORS['overlay0']};")
        a1_row.addWidget(self.lbl_a1, stretch=1)
        btn_rec_a1 = QPushButton("Rec")
        btn_rec_a1.setMaximumHeight(24)
        btn_rec_a1.setMaximumWidth(40)
        btn_rec_a1.clicked.connect(self._record_a1)
        a1_row.addWidget(btn_rec_a1)
        btn_go_a1 = QPushButton("Go")
        btn_go_a1.setMaximumHeight(24)
        btn_go_a1.setMaximumWidth(30)
        btn_go_a1.clicked.connect(self._goto_a1)
        a1_row.addWidget(btn_go_a1)
        layout.addLayout(a1_row)

        # Corner row
        cr_row = QHBoxLayout()
        cr_row.addWidget(QLabel("Corner:"))
        self.lbl_corner = QLabel("—")
        self.lbl_corner.setStyleSheet(f"color: {COLORS['overlay0']};")
        cr_row.addWidget(self.lbl_corner, stretch=1)
        btn_rec_corner = QPushButton("Rec")
        btn_rec_corner.setMaximumHeight(24)
        btn_rec_corner.setMaximumWidth(40)
        btn_rec_corner.clicked.connect(self._record_corner)
        cr_row.addWidget(btn_rec_corner)
        btn_go_corner = QPushButton("Go")
        btn_go_corner.setMaximumHeight(24)
        btn_go_corner.setMaximumWidth(30)
        btn_go_corner.clicked.connect(self._goto_corner)
        cr_row.addWidget(btn_go_corner)
        layout.addLayout(cr_row)

        btn_calc = QPushButton("Calculate Alignment")
        btn_calc.setObjectName("accentBtn")
        btn_calc.setMaximumHeight(28)
        btn_calc.clicked.connect(self._calculate_alignment)
        layout.addWidget(btn_calc)

        self.lbl_alignment = QLabel("")
        self.lbl_alignment.setWordWrap(True)
        self.lbl_alignment.setStyleSheet(f"color: {COLORS['subtext0']};")
        layout.addWidget(self.lbl_alignment)

        # ── Step 3: Validate ─────────────────────────────────────
        s3_label = QLabel("Step 3 — Validate")
        s3_label.setObjectName("contextSectionLabel")
        layout.addWidget(s3_label)

        val_row = QHBoxLayout()
        val_row.addWidget(QLabel("Well:"))
        self.val_well_combo = QComboBox()
        val_row.addWidget(self.val_well_combo, stretch=1)
        btn_goto_well = QPushButton("Go")
        btn_goto_well.setMaximumHeight(24)
        btn_goto_well.clicked.connect(self._goto_well)
        val_row.addWidget(btn_goto_well)
        layout.addLayout(val_row)

        self.lbl_val_result = QLabel("")
        self.lbl_val_result.setWordWrap(True)
        self.lbl_val_result.setStyleSheet(f"color: {COLORS['subtext0']};")
        layout.addWidget(self.lbl_val_result)

        # ── Calibration Persistence ──────────────────────────────
        persist_label = QLabel("Calibration Data")
        persist_label.setObjectName("contextSectionLabel")
        layout.addWidget(persist_label)

        self.ctx_lbl_cal_status = QLabel("Not calibrated")
        self.ctx_lbl_cal_status.setObjectName("contextLabel")
        self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['yellow']};")
        layout.addWidget(self.ctx_lbl_cal_status)

        cal_btn_row = QHBoxLayout()
        btn_save_cal = QPushButton("💾 Save")
        btn_save_cal.setMaximumHeight(26)
        btn_save_cal.clicked.connect(self._save_calibration)
        cal_btn_row.addWidget(btn_save_cal)

        btn_load_cal = QPushButton("📂 Load")
        btn_load_cal.setMaximumHeight(26)
        btn_load_cal.clicked.connect(self._load_calibration)
        cal_btn_row.addWidget(btn_load_cal)
        layout.addLayout(cal_btn_row)

        layout.addStretch()

        self._context_widget = ctx

        # Now that all widgets exist, load any saved calibration
        self._load_calibration()

        return ctx

    # ── Context panel helpers ─────────────────────────────────────

    def _make_slider_row(self, label: str, min_val: int, max_val: int,
                         default: int, attr_name: str, slot) -> QWidget:
        """Create a compact labeled slider row."""
        widget = QWidget()
        lay = QVBoxLayout(widget)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(2)

        top = QHBoxLayout()
        top.addWidget(QLabel(label))
        value_lbl = QLabel(str(default))
        value_lbl.setObjectName("dimLabel")
        value_lbl.setMinimumWidth(30)
        top.addStretch()
        top.addWidget(value_lbl)
        lay.addLayout(top)

        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default)
        slider.valueChanged.connect(lambda v: value_lbl.setText(str(v)))
        slider.valueChanged.connect(slot)
        setattr(self, f"_slider_{attr_name}", slider)
        lay.addWidget(slider)

        return widget

    # ── Camera settings callbacks (apply to ALL cameras) ──────────

    def _on_brightness_changed(self, value: int):
        for cam in self._cameras:
            cam.set_brightness(value)

    def _on_gamma_changed(self, value: int):
        for cam in self._cameras:
            cam.set_gamma(value / 100.0)

    def _on_fps_changed(self, value: int):
        for cam in self._cameras:
            cam._fps = value
            if cam._running:
                cam._timer.setInterval(int(1000 / value))

    def _on_crosshair_toggled(self, checked: bool):
        for cam in self._cameras:
            cam._show_crosshair = checked
            if hasattr(cam, 'chk_crosshair'):
                cam.chk_crosshair.blockSignals(True)
                cam.chk_crosshair.setChecked(checked)
                cam.chk_crosshair.blockSignals(False)

    def _refresh_all_cameras(self):
        for cam in self._cameras:
            cam.refresh_cameras()

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT UI  (position readout + multi-camera grid)
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        outer.addWidget(scroll)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 8, 12, 8)
        scroll.setWidget(container)

        mono = QFont("Consolas", 12)

        # ── Position Readout (compact) ────────────────────────────
        pos_card = QFrame()
        pos_card.setObjectName("cardFrame")
        pos_layout = QHBoxLayout(pos_card)
        pos_layout.setSpacing(16)
        for name, attr in [("X:", "lbl_x"), ("Y:", "lbl_y"), ("Z:", "lbl_z")]:
            pos_layout.addWidget(QLabel(name))
            lbl = QLabel("—")
            lbl.setFont(mono)
            lbl.setObjectName("valueLabel")
            pos_layout.addWidget(lbl)
            setattr(self, attr, lbl)
        pos_layout.addStretch()
        layout.addWidget(pos_card)

        # ── Camera Feeds Grid ─────────────────────────────────────
        cam_card = QFrame()
        cam_card.setObjectName("cardFrame")
        cam_layout = QVBoxLayout(cam_card)
        cam_layout.setSpacing(4)

        cam_title = QLabel("Camera Feeds")
        cam_title.setObjectName("sectionLabel")
        cam_layout.addWidget(cam_title)

        if CV2_AVAILABLE and CameraWidget is not None:
            self._cam_grid = QHBoxLayout()
            self._cam_grid.setSpacing(6)

            for i in range(MAX_CAMERAS):
                cam = CameraWidget(
                    camera_label=f"Camera {i + 1}",
                    compact=(MAX_CAMERAS > 1),
                    show_controls=True,
                    parent=self,
                )
                self._cameras.append(cam)
                self._cam_grid.addWidget(cam, stretch=1)

            cam_layout.addLayout(self._cam_grid, stretch=1)
        else:
            no_cam = QLabel(
                "Camera unavailable\n"
                "Install: pip install opencv-python"
            )
            no_cam.setAlignment(Qt.AlignCenter)
            no_cam.setStyleSheet(f"color: {COLORS['overlay0']};")
            cam_layout.addWidget(no_cam, stretch=1)

        layout.addWidget(cam_card, stretch=1)

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATE
    # ════════════════════════════════════════════════════════════════

    def on_status_update(self):
        """Called periodically by the main window."""
        ctrl = self.controller

        # Update position readout
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            zx = xy[0] - ctrl.zero_position["x"]
            zy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{zx:.0f}")
            self.lbl_y.setText(f"{zy:.0f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")

        zp = ctrl.get_zp_position(cached=True)
        if isinstance(zp, (list, tuple)) and len(zp) >= 1 and zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position.get('Z', 0):.2f}")
        else:
            self.lbl_z.setText("—")

    # ════════════════════════════════════════════════════════════════
    #  CALIBRATION STEPS
    # ════════════════════════════════════════════════════════════════

    # ── Step 1: Zero Needle ──────────────────────────────────────

    def _set_zero(self):
        self.controller._calibrate_zero()
        z = self.controller.zero_position
        self.lbl_zero_status.setText(
            f"Set: X={z['x']:.0f} Y={z['y']:.0f} Z={z['Z']:.2f}")
        self.lbl_zero_status.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Zero set: {z}")

    def _goto_zero(self):
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
        self.controller.move_z_absolute(0, from_zero_ref=True)

    # ── Step 2: Teach Well Plate ─────────────────────────────────

    def _on_plate_changed(self, idx):
        """Handle plate format change."""
        sender = self.sender()
        if sender is None:
            return

        fmt = sender.currentData()
        if fmt is None:
            return

        self._plate = WellPlate.from_format(fmt)
        defn = PLATE_DEFINITIONS[fmt]
        rows, cols = defn["rows"], defn["cols"]

        row_letter = chr(ord('A') + rows - 1)
        self._corner_well = f"{row_letter}{cols}"

        wells = self._plate.well_names
        self.val_well_combo.clear()
        self.val_well_combo.addItems(wells)

        # Reset taught positions
        self._taught_a1 = None
        self._taught_corner = None
        self.lbl_a1.setText("—")
        self.lbl_corner.setText("—")
        self.lbl_alignment.setText("")

    def _record_a1(self):
        xy = self.controller.get_xy_position(cached=False)
        if xy[0] is None:
            return
        self._taught_a1 = (xy[0], xy[1])
        self.lbl_a1.setText(f"({xy[0]:.0f}, {xy[1]:.0f})")
        self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

    def _goto_a1(self):
        if self._taught_a1:
            self.controller.move_xy_absolute(
                self._taught_a1[0] - self.controller.zero_position["x"],
                self._taught_a1[1] - self.controller.zero_position["y"],
                from_zero_ref=True,
            )

    def _record_corner(self):
        xy = self.controller.get_xy_position(cached=False)
        if xy[0] is None:
            return
        self._taught_corner = (xy[0], xy[1])
        self.lbl_corner.setText(f"({xy[0]:.0f}, {xy[1]:.0f})")
        self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")

    def _goto_corner(self):
        if self._taught_corner:
            self.controller.move_xy_absolute(
                self._taught_corner[0] - self.controller.zero_position["x"],
                self._taught_corner[1] - self.controller.zero_position["y"],
                from_zero_ref=True,
            )

    def _calculate_alignment(self):
        if not self._taught_a1 or not self._taught_corner or not self._plate:
            self.lbl_alignment.setText(
                "⚠ Record A1 and corner first, and select plate format."
            )
            return

        a1_expected = self._plate.get_well_position("A1")
        corner_expected = self._plate.get_well_position(self._corner_well)

        if a1_expected is None or corner_expected is None:
            self.lbl_alignment.setText(
                "⚠ Cannot compute alignment for this plate format."
            )
            return

        ex = corner_expected[0] - a1_expected[0]
        ey = corner_expected[1] - a1_expected[1]
        mx = self._taught_corner[0] - self._taught_a1[0]
        my = self._taught_corner[1] - self._taught_a1[1]

        expected_dist = math.sqrt(ex**2 + ey**2)
        measured_dist = math.sqrt(mx**2 + my**2)

        if expected_dist < 0.001:
            return

        self._scale = measured_dist / expected_dist
        expected_angle = math.atan2(ey, ex)
        measured_angle = math.atan2(my, mx)
        self._rotation = math.degrees(measured_angle - expected_angle)

        self._offset_x = self._taught_a1[0] - self.controller.zero_position["x"]
        self._offset_y = self._taught_a1[1] - self.controller.zero_position["y"]

        self.lbl_alignment.setText(
            f"✅ Scale: {self._scale:.4f} | "
            f"Rot: {self._rotation:.2f}° | "
            f"Off: ({self._offset_x:.0f}, {self._offset_y:.0f})"
        )
        self.lbl_alignment.setStyleSheet(f"color: {COLORS['green']};")

        if hasattr(self, 'ctx_lbl_cal_status'):
            self.ctx_lbl_cal_status.setText("✅ Calibrated")
            self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")

    # ── Step 3: Validate ─────────────────────────────────────────

    def _goto_well(self):
        well = self.val_well_combo.currentText()
        if not well or not self._plate:
            return

        pos = self._plate.get_well_position(well)
        if pos is None:
            return

        a1_expected = self._plate.get_well_position("A1")
        if a1_expected is None:
            return

        dx = pos[0] - a1_expected[0]
        dy = pos[1] - a1_expected[1]

        rad = math.radians(self._rotation)
        rx = dx * math.cos(rad) - dy * math.sin(rad)
        ry = dx * math.sin(rad) + dy * math.cos(rad)

        target_x = self._offset_x + rx * self._scale
        target_y = self._offset_y + ry * self._scale

        self.controller.move_xy_absolute(target_x, target_y, from_zero_ref=True)
        self.lbl_val_result.setText(
            f"Moving to {well} → ({target_x:.0f}, {target_y:.0f})"
        )

    # ════════════════════════════════════════════════════════════════
    #  CALIBRATION PERSISTENCE
    # ════════════════════════════════════════════════════════════════

    def _save_calibration(self):
        if self.settings is None:
            return
        cal_data = {
            "plate_format": self._plate.format_key if self._plate else None,
            "taught_a1": list(self._taught_a1) if self._taught_a1 else None,
            "taught_corner": list(self._taught_corner) if self._taught_corner else None,
            "offset_x": self._offset_x,
            "offset_y": self._offset_y,
            "rotation": self._rotation,
            "scale": self._scale,
        }
        self.settings.set_section("calibration", cal_data)
        self.settings.save()
        logger.info("Calibration saved to settings")

    def _load_calibration(self):
        if self.settings is None:
            return
        cal = self.settings.get_section("calibration")
        if not cal:
            return

        if cal.get("plate_format"):
            self._plate = WellPlate.from_format(cal["plate_format"])
            wells = self._plate.well_names
            self.val_well_combo.clear()
            self.val_well_combo.addItems(wells)

        if cal.get("taught_a1"):
            self._taught_a1 = tuple(cal["taught_a1"])
            self.lbl_a1.setText(
                f"({self._taught_a1[0]:.0f}, {self._taught_a1[1]:.0f})"
            )
            self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

        if cal.get("taught_corner"):
            self._taught_corner = tuple(cal["taught_corner"])
            self.lbl_corner.setText(
                f"({self._taught_corner[0]:.0f}, {self._taught_corner[1]:.0f})"
            )
            self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")

        self._offset_x = cal.get("offset_x", 0)
        self._offset_y = cal.get("offset_y", 0)
        self._rotation = cal.get("rotation", 0)
        self._scale = cal.get("scale", 1.0)

        if self._scale != 1.0 or self._rotation != 0:
            self.lbl_alignment.setText(
                f"Loaded | Scale: {self._scale:.4f} | "
                f"Rotation: {self._rotation:.2f}°"
            )
            if hasattr(self, 'ctx_lbl_cal_status'):
                self.ctx_lbl_cal_status.setText("✅ Loaded from settings")
                self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")