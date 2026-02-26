"""
Calibration Page — Guided workflow with camera settings in context panel.

Main content: 3-step calibration wizard (zero needle, teach plate, validate)
Context panel: camera source/settings, plate format, calibration save/load

The context panel provides camera gamma, brightness, FPS, crosshair toggle,
and webcam source selection — accessible whenever the Calibration workflow
is active.
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
    from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE
except ImportError:
    CameraWidget = None
    CV2_AVAILABLE = False


class CalibrationPage(QWidget):
    """Three-step guided calibration wizard with camera context panel."""

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

        # Camera widget (created in context panel)
        self._camera_widget: CameraWidget | None = None
        self._context_widget = None

        self._setup_ui()
        self._load_calibration()

    def get_page_title(self) -> str:
        return "Calibration"

    def get_context_widget(self) -> QWidget:
        """Build camera settings + calibration config context panel."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        # ── Camera Settings ──────────────────────────────────────
        cam_label = QLabel("Camera Settings")
        cam_label.setObjectName("contextSectionLabel")
        layout.addWidget(cam_label)

        if CV2_AVAILABLE:
            # Camera source selection
            src_row = QHBoxLayout()
            src_row.addWidget(QLabel("Source:"))
            self._cam_source_combo = QComboBox()
            self._cam_source_combo.setToolTip("Select webcam device")
            self._populate_cameras()
            self._cam_source_combo.currentIndexChanged.connect(self._on_camera_source_changed)
            src_row.addWidget(self._cam_source_combo, stretch=1)
            layout.addLayout(src_row)

            # Refresh cameras button
            btn_refresh_cam = QPushButton("🔄 Detect Cameras")
            btn_refresh_cam.setObjectName("flatBtn")
            btn_refresh_cam.clicked.connect(self._populate_cameras)
            layout.addWidget(btn_refresh_cam)

            # Brightness
            layout.addWidget(self._make_slider_row(
                "Brightness:", -100, 100, 0,
                "cam_brightness", self._on_brightness_changed
            ))

            # Contrast / Gamma
            layout.addWidget(self._make_slider_row(
                "Gamma:", 10, 300, 100,
                "cam_gamma", self._on_gamma_changed
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
            self._chk_crosshair = QCheckBox("Show Crosshair")
            self._chk_crosshair.setChecked(True)
            self._chk_crosshair.toggled.connect(self._on_crosshair_toggled)
            layout.addWidget(self._chk_crosshair)

            # Camera controls
            cam_btn_row = QHBoxLayout()
            self._btn_cam_start = QPushButton("▶ Start")
            self._btn_cam_start.setObjectName("successBtn")
            self._btn_cam_start.clicked.connect(self._start_camera)
            cam_btn_row.addWidget(self._btn_cam_start)

            self._btn_cam_stop = QPushButton("⏹ Stop")
            self._btn_cam_stop.setObjectName("dangerBtn")
            self._btn_cam_stop.setEnabled(False)
            self._btn_cam_stop.clicked.connect(self._stop_camera)
            cam_btn_row.addWidget(self._btn_cam_stop)
            layout.addLayout(cam_btn_row)

            btn_snapshot = QPushButton("📷 Snapshot")
            btn_snapshot.clicked.connect(self._take_snapshot)
            layout.addWidget(btn_snapshot)
        else:
            layout.addWidget(QLabel(
                "Camera unavailable.\nInstall OpenCV:\npip install opencv-python"
            ))

        # ── Plate Settings ───────────────────────────────────────
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

        # ── Calibration Persistence ──────────────────────────────
        persist_label = QLabel("Calibration Data")
        persist_label.setObjectName("contextSectionLabel")
        layout.addWidget(persist_label)

        self.ctx_lbl_cal_status = QLabel("Not calibrated")
        self.ctx_lbl_cal_status.setObjectName("contextLabel")
        self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['yellow']};")
        layout.addWidget(self.ctx_lbl_cal_status)

        btn_save_cal = QPushButton("💾 Save Calibration")
        btn_save_cal.clicked.connect(self._save_calibration)
        layout.addWidget(btn_save_cal)

        btn_load_cal = QPushButton("📂 Load Calibration")
        btn_load_cal.clicked.connect(self._load_calibration)
        layout.addWidget(btn_load_cal)

        layout.addStretch()

        self._context_widget = ctx
        return ctx

    # ── Camera Context Helpers ───────────────────────────────────

    def _make_slider_row(self, label: str, min_val: int, max_val: int,
                         default: int, attr_name: str, slot) -> QWidget:
        """Create a labeled slider row for the context panel."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(2)

        top = QHBoxLayout()
        top.addWidget(QLabel(label))
        value_lbl = QLabel(str(default))
        value_lbl.setObjectName("dimLabel")
        value_lbl.setMinimumWidth(30)
        top.addStretch()
        top.addWidget(value_lbl)
        layout.addLayout(top)

        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default)
        slider.valueChanged.connect(lambda v: value_lbl.setText(str(v)))
        slider.valueChanged.connect(slot)
        setattr(self, f"_slider_{attr_name}", slider)
        layout.addWidget(slider)

        return widget

    def _populate_cameras(self):
        """Detect available cameras and populate the source combo."""
        if not CV2_AVAILABLE:
            return
        self._cam_source_combo.clear()
        # Probe up to 8 camera indices
        import cv2
        for i in range(8):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                self._cam_source_combo.addItem(f"Camera {i}", i)
                cap.release()

        if self._cam_source_combo.count() == 0:
            self._cam_source_combo.addItem("No cameras found", -1)

    def _on_camera_source_changed(self, index: int):
        if self._camera_widget and self._camera_widget._running:
            self._stop_camera()
            cam_idx = self._cam_source_combo.currentData()
            if cam_idx is not None and cam_idx >= 0:
                self._camera_widget._camera_index = cam_idx
                self._start_camera()

    def _on_brightness_changed(self, value: int):
        if self._camera_widget and self._camera_widget._capture:
            import cv2
            self._camera_widget._capture.set(cv2.CAP_PROP_BRIGHTNESS, value)

    def _on_gamma_changed(self, value: int):
        if self._camera_widget and self._camera_widget._capture:
            import cv2
            self._camera_widget._capture.set(cv2.CAP_PROP_GAMMA, value / 100.0)

    def _on_fps_changed(self, value: int):
        if self._camera_widget:
            self._camera_widget._fps = value
            if self._camera_widget._running:
                self._camera_widget._timer.setInterval(int(1000 / value))

    def _on_crosshair_toggled(self, checked: bool):
        if self._camera_widget:
            self._camera_widget._show_crosshair = checked

    def _start_camera(self):
        if self._camera_widget:
            cam_idx = self._cam_source_combo.currentData() if hasattr(self, '_cam_source_combo') else 0
            if cam_idx is not None and cam_idx >= 0:
                self._camera_widget._camera_index = cam_idx
            self._camera_widget.start()
            self._btn_cam_start.setEnabled(False)
            self._btn_cam_stop.setEnabled(True)

    def _stop_camera(self):
        if self._camera_widget:
            self._camera_widget.stop()
            self._btn_cam_start.setEnabled(True)
            self._btn_cam_stop.setEnabled(False)

    def _take_snapshot(self):
        if self._camera_widget and hasattr(self._camera_widget, 'take_snapshot'):
            self._camera_widget.take_snapshot()

    # ── Main Content UI ──────────────────────────────────────────

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
        layout.setContentsMargins(16, 12, 16, 12)
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

        # ── Main content: Camera + Steps side by side ─────────────
        content_row = QHBoxLayout()
        content_row.setSpacing(8)

        # Steps column
        steps_widget = QWidget()
        steps_layout = QVBoxLayout(steps_widget)
        steps_layout.setSpacing(8)
        steps_layout.setContentsMargins(0, 0, 0, 0)

        # Step 1: Zero Needle
        step1 = QFrame()
        step1.setObjectName("cardFrame")
        s1 = QVBoxLayout(step1)
        s1.setSpacing(4)
        s1_title = QLabel("Step 1 — Zero Needle")
        s1_title.setObjectName("sectionLabel")
        s1.addWidget(s1_title)

        s1.addWidget(QLabel(
            "Jog needle to contact point, then press 'Set Zero'."
        ))

        s1_row = QHBoxLayout()
        btn_set_zero = QPushButton("Set Zero Here")
        btn_set_zero.setObjectName("successBtn")
        btn_set_zero.clicked.connect(self._set_zero)
        s1_row.addWidget(btn_set_zero)

        btn_goto_zero = QPushButton("Go to Zero")
        btn_goto_zero.clicked.connect(self._goto_zero)
        s1_row.addWidget(btn_goto_zero)

        self.lbl_zero_status = QLabel("Not set")
        self.lbl_zero_status.setStyleSheet(f"color: {COLORS['yellow']};")
        s1_row.addWidget(self.lbl_zero_status)
        s1_row.addStretch()
        s1.addLayout(s1_row)
        steps_layout.addWidget(step1)

        # Step 2: Teach Well Plate
        step2 = QFrame()
        step2.setObjectName("cardFrame")
        s2 = QVBoxLayout(step2)
        s2.setSpacing(4)
        s2_title = QLabel("Step 2 — Teach Plate Position")
        s2_title.setObjectName("sectionLabel")
        s2.addWidget(s2_title)

        s2.addWidget(QLabel(
            "Jog to A1 center → Record.  Jog to corner well → Record."
        ))

        teach_grid = QGridLayout()
        teach_grid.setSpacing(4)

        teach_grid.addWidget(QLabel("A1:"), 0, 0)
        self.lbl_a1 = QLabel("Not recorded")
        self.lbl_a1.setStyleSheet(f"color: {COLORS['overlay0']};")
        teach_grid.addWidget(self.lbl_a1, 0, 1)
        btn_rec_a1 = QPushButton("Record")
        btn_rec_a1.setMaximumHeight(26)
        btn_rec_a1.clicked.connect(self._record_a1)
        teach_grid.addWidget(btn_rec_a1, 0, 2)
        btn_go_a1 = QPushButton("Go to")
        btn_go_a1.setMaximumHeight(26)
        btn_go_a1.clicked.connect(self._goto_a1)
        teach_grid.addWidget(btn_go_a1, 0, 3)

        teach_grid.addWidget(QLabel("Corner:"), 1, 0)
        self.lbl_corner = QLabel("Not recorded")
        self.lbl_corner.setStyleSheet(f"color: {COLORS['overlay0']};")
        teach_grid.addWidget(self.lbl_corner, 1, 1)
        btn_rec_corner = QPushButton("Record")
        btn_rec_corner.setMaximumHeight(26)
        btn_rec_corner.clicked.connect(self._record_corner)
        teach_grid.addWidget(btn_rec_corner, 1, 2)
        btn_go_corner = QPushButton("Go to")
        btn_go_corner.setMaximumHeight(26)
        btn_go_corner.clicked.connect(self._goto_corner)
        teach_grid.addWidget(btn_go_corner, 1, 3)

        s2.addLayout(teach_grid)

        calc_row = QHBoxLayout()
        btn_calc = QPushButton("Calculate Alignment")
        btn_calc.setObjectName("accentBtn")
        btn_calc.clicked.connect(self._calculate_alignment)
        calc_row.addWidget(btn_calc)
        self.lbl_alignment = QLabel("")
        self.lbl_alignment.setWordWrap(True)
        self.lbl_alignment.setStyleSheet(f"color: {COLORS['subtext0']};")
        calc_row.addWidget(self.lbl_alignment, stretch=1)
        s2.addLayout(calc_row)
        steps_layout.addWidget(step2)

        # Step 3: Validate
        step3 = QFrame()
        step3.setObjectName("cardFrame")
        s3 = QVBoxLayout(step3)
        s3.setSpacing(4)
        s3_title = QLabel("Step 3 — Validate Alignment")
        s3_title.setObjectName("sectionLabel")
        s3.addWidget(s3_title)

        val_row = QHBoxLayout()
        val_row.addWidget(QLabel("Well:"))
        self.val_well_combo = QComboBox()
        val_row.addWidget(self.val_well_combo)
        btn_goto_well = QPushButton("Go to Well")
        btn_goto_well.clicked.connect(self._goto_well)
        val_row.addWidget(btn_goto_well)
        val_row.addStretch()
        s3.addLayout(val_row)

        self.lbl_val_result = QLabel("")
        self.lbl_val_result.setWordWrap(True)
        self.lbl_val_result.setStyleSheet(f"color: {COLORS['subtext0']};")
        s3.addWidget(self.lbl_val_result)
        steps_layout.addWidget(step3)

        steps_layout.addStretch()
        content_row.addWidget(steps_widget, stretch=2)

        # Camera feed column
        cam_widget = QFrame()
        cam_widget.setObjectName("cardFrame")
        cam_layout = QVBoxLayout(cam_widget)
        cam_layout.setSpacing(4)

        cam_title = QLabel("Camera Feed")
        cam_title.setObjectName("sectionLabel")
        cam_layout.addWidget(cam_title)

        if CV2_AVAILABLE and CameraWidget is not None:
            self._camera_widget = CameraWidget()
            cam_layout.addWidget(self._camera_widget, stretch=1)
        else:
            no_cam = QLabel("Camera unavailable\nInstall: pip install opencv-python")
            no_cam.setAlignment(Qt.AlignCenter)
            no_cam.setStyleSheet(f"color: {COLORS['overlay0']};")
            cam_layout.addWidget(no_cam, stretch=1)

        content_row.addWidget(cam_widget, stretch=1)

        layout.addLayout(content_row)

    # ── Status Update ────────────────────────────────────────────

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
        if zp.get("Z") is not None:
            self.lbl_z.setText(f"{zp['Z'] - ctrl.zero_position.get('Z', 0):.2f}")
        else:
            self.lbl_z.setText("—")

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
        """Handle plate format change (from either main or context combo)."""
        sender = self.sender()
        if sender is None:
            return

        fmt = sender.currentData()
        if fmt is None:
            return

        self._plate = WellPlate.from_format(fmt)
        defn = PLATE_DEFINITIONS[fmt]
        rows, cols = defn["rows"], defn["cols"]

        # Determine corner well
        row_letter = chr(ord('A') + rows - 1)
        self._corner_well = f"{row_letter}{cols}"

        # Update validation well combo
        wells = self._plate.well_names
        self.val_well_combo.clear()
        self.val_well_combo.addItems(wells)

        # Sync the other combo if present
        if hasattr(self, 'ctx_plate_combo') and sender != self.ctx_plate_combo:
            self.ctx_plate_combo.blockSignals(True)
            self.ctx_plate_combo.setCurrentIndex(idx)
            self.ctx_plate_combo.blockSignals(False)

        # Reset taught positions
        self._taught_a1 = None
        self._taught_corner = None
        self.lbl_a1.setText("Not recorded")
        self.lbl_corner.setText("Not recorded")
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
            self.lbl_alignment.setText("⚠ Record A1 and corner first, and select plate format.")
            return

        # Get expected positions from plate geometry
        a1_expected = self._plate.get_well_position("A1")
        corner_expected = self._plate.get_well_position(self._corner_well)

        if a1_expected is None or corner_expected is None:
            self.lbl_alignment.setText("⚠ Cannot compute alignment for this plate format.")
            return

        # Vectors
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
            f"✅ Aligned | Scale: {self._scale:.4f} | "
            f"Rotation: {self._rotation:.2f}° | "
            f"Offset: ({self._offset_x:.0f}, {self._offset_y:.0f})"
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
        self.lbl_val_result.setText(f"Moving to {well} → ({target_x:.0f}, {target_y:.0f})")

    # ── Calibration Persistence ──────────────────────────────────

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
            self.lbl_a1.setText(f"({self._taught_a1[0]:.0f}, {self._taught_a1[1]:.0f})")
            self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

        if cal.get("taught_corner"):
            self._taught_corner = tuple(cal["taught_corner"])
            self.lbl_corner.setText(f"({self._taught_corner[0]:.0f}, {self._taught_corner[1]:.0f})")
            self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")

        self._offset_x = cal.get("offset_x", 0)
        self._offset_y = cal.get("offset_y", 0)
        self._rotation = cal.get("rotation", 0)
        self._scale = cal.get("scale", 1.0)

        if self._scale != 1.0 or self._rotation != 0:
            self.lbl_alignment.setText(
                f"Loaded | Scale: {self._scale:.4f} | Rotation: {self._rotation:.2f}°")
            if hasattr(self, 'ctx_lbl_cal_status'):
                self.ctx_lbl_cal_status.setText("✅ Loaded from settings")
                self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")
