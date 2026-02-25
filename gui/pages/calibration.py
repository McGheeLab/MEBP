"""
Calibration Page — Guided workflow for needle zeroing and well plate alignment.

Steps:
    1. Zero Needle:   Jog to needle-tip contact position → Set Zero.
    2. Teach Plate:   Jog to well A1 and to the diagonally opposite corner
                      well → Calculate plate alignment (offset + rotation).
    3. Validate:      Move to several calculated well centres and visually
                      confirm alignment.

Enhancements:
    - Enhancement 2: Camera integration for visual verification
    - Enhancement 5: Calibration persistence to settings.json
"""

from __future__ import annotations

import math
import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QFrame,
    QStackedWidget, QSizePolicy, QMessageBox,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from SupportClasses.WellPlate import WellPlate, PLATE_DEFINITIONS

logger = logging.getLogger(__name__)

# Optional camera support
try:
    from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE
except ImportError:
    CameraWidget = None
    CV2_AVAILABLE = False


class CalibrationPage(QWidget):
    """Three-step guided calibration wizard."""

    def __init__(self, controller: StageController, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings  # Enhancement 5: for calibration persistence

        # Calibration state
        self._plate: WellPlate | None = None
        self._taught_a1: tuple[float, float] | None = None    # machine coords of A1
        self._taught_corner: tuple[float, float] | None = None # machine coords of far corner
        self._corner_well: str = "H12"
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._rotation = 0.0  # degrees
        self._scale = 1.0

        self._setup_ui()

        # Enhancement 5: Load saved calibration
        self._load_calibration()

    # ── UI ────────────────────────────────────────────────────────

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        mono = QFont("Consolas", 12)

        # Position readout (always visible)
        pos_group = QGroupBox("Current Position")
        pos_grid = QGridLayout(pos_group)
        for i, (name, attr) in enumerate([
            ("X:", "lbl_x"), ("Y:", "lbl_y"), ("Z:", "lbl_z"),
        ]):
            pos_grid.addWidget(QLabel(name), 0, i * 2)
            lbl = QLabel("—")
            lbl.setFont(mono)
            lbl.setObjectName("valueLabel")
            pos_grid.addWidget(lbl, 0, i * 2 + 1)
            setattr(self, attr, lbl)
        layout.addWidget(pos_group)

        # ── Step 1: Zero Needle ──────────────────────────────────
        step1 = QGroupBox("Step 1 — Zero Needle")
        s1 = QVBoxLayout(step1)
        s1.addWidget(QLabel(
            "Use the Jog Control tab (or Xbox controller) to position the needle tip\n"
            "at the printing contact point.  Then press 'Set Zero' to store this\n"
            "as the origin for all subsequent movements."
        ))

        s1_row = QHBoxLayout()
        btn_set_zero = QPushButton("Set Zero Here")
        btn_set_zero.setObjectName("connectBtn")
        btn_set_zero.clicked.connect(self._set_zero)
        s1_row.addWidget(btn_set_zero)

        btn_goto_zero = QPushButton("Go to Zero")
        btn_goto_zero.clicked.connect(self._goto_zero)
        s1_row.addWidget(btn_goto_zero)

        self.lbl_zero_status = QLabel("Not set")
        self.lbl_zero_status.setStyleSheet("color: #f9e2af;")
        s1_row.addWidget(self.lbl_zero_status)
        s1_row.addStretch()
        s1.addLayout(s1_row)
        layout.addWidget(step1)

        # ── Step 2: Teach Well Plate ─────────────────────────────
        step2 = QGroupBox("Step 2 — Teach Well Plate Position")
        s2 = QVBoxLayout(step2)
        s2.addWidget(QLabel(
            "Select a plate format, then jog to the centre of well A1 and record it.\n"
            "Next jog to the diagonally opposite corner well and record that.\n"
            "The system will compute offset and rotation to align the plate."
        ))

        plate_row = QHBoxLayout()
        plate_row.addWidget(QLabel("Plate format:"))
        self.plate_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            self.plate_combo.addItem(f"{fmt}-well", fmt)
        self.plate_combo.setCurrentIndex(4)  # 96
        self.plate_combo.currentIndexChanged.connect(self._on_plate_changed)
        plate_row.addWidget(self.plate_combo)
        plate_row.addStretch()
        s2.addLayout(plate_row)

        teach_grid = QGridLayout()

        # A1
        teach_grid.addWidget(QLabel("Well A1:"), 0, 0)
        self.lbl_a1 = QLabel("Not recorded")
        self.lbl_a1.setStyleSheet("color: #6c7086;")
        teach_grid.addWidget(self.lbl_a1, 0, 1)
        btn_record_a1 = QPushButton("Record A1")
        btn_record_a1.clicked.connect(self._record_a1)
        teach_grid.addWidget(btn_record_a1, 0, 2)
        btn_goto_a1 = QPushButton("Go to A1")
        btn_goto_a1.clicked.connect(self._goto_a1)
        teach_grid.addWidget(btn_goto_a1, 0, 3)

        # Corner well
        teach_grid.addWidget(QLabel("Corner well:"), 1, 0)
        self.lbl_corner = QLabel("Not recorded")
        self.lbl_corner.setStyleSheet("color: #6c7086;")
        teach_grid.addWidget(self.lbl_corner, 1, 1)
        btn_record_corner = QPushButton("Record Corner")
        btn_record_corner.clicked.connect(self._record_corner)
        teach_grid.addWidget(btn_record_corner, 1, 2)
        btn_goto_corner = QPushButton("Go to Corner")
        btn_goto_corner.clicked.connect(self._goto_corner)
        teach_grid.addWidget(btn_goto_corner, 1, 3)

        s2.addLayout(teach_grid)

        calc_row = QHBoxLayout()
        btn_calc = QPushButton("Calculate Alignment")
        btn_calc.setObjectName("connectBtn")
        btn_calc.clicked.connect(self._calculate_alignment)
        calc_row.addWidget(btn_calc)

        # Enhancement 5: Save/Load calibration buttons
        btn_save_cal = QPushButton("💾 Save Calibration")
        btn_save_cal.clicked.connect(self._save_calibration)
        btn_save_cal.setToolTip("Save plate alignment to settings for reuse")
        calc_row.addWidget(btn_save_cal)

        btn_load_cal = QPushButton("📂 Load Calibration")
        btn_load_cal.clicked.connect(self._load_calibration)
        btn_load_cal.setToolTip("Restore previously saved plate alignment")
        calc_row.addWidget(btn_load_cal)

        self.lbl_alignment = QLabel("")
        self.lbl_alignment.setWordWrap(True)
        self.lbl_alignment.setStyleSheet("color: #a6adc8;")
        calc_row.addWidget(self.lbl_alignment, stretch=1)
        s2.addLayout(calc_row)
        layout.addWidget(step2)

        # ── Step 3: Validate ─────────────────────────────────────
        step3 = QGroupBox("Step 3 — Validate Alignment")
        s3 = QVBoxLayout(step3)
        s3.addWidget(QLabel(
            "Select a well and press 'Go to Well' to move the needle to its\n"
            "calculated centre.  Visually confirm alignment."
        ))

        val_row = QHBoxLayout()
        val_row.addWidget(QLabel("Well:"))
        self.val_well_combo = QComboBox()
        val_row.addWidget(self.val_well_combo)

        btn_goto_well = QPushButton("Go to Well")
        btn_goto_well.clicked.connect(self._goto_well)
        val_row.addWidget(btn_goto_well)

        btn_raise_z = QPushButton("Raise Z (+5 mm)")
        btn_raise_z.clicked.connect(lambda: self.controller.move_z_relative(-5.0))
        val_row.addWidget(btn_raise_z)

        val_row.addStretch()
        s3.addLayout(val_row)

        self.lbl_val_result = QLabel("")
        self.lbl_val_result.setStyleSheet("color: #a6adc8;")
        s3.addWidget(self.lbl_val_result)
        layout.addWidget(step3)

        # Enhancement 2: Camera feed for visual calibration verification
        if CameraWidget is not None and CV2_AVAILABLE:
            cam_group = QGroupBox("Microscope Camera (Enhancement 2)")
            cam_layout = QVBoxLayout(cam_group)
            self.camera_widget = CameraWidget()
            cam_layout.addWidget(self.camera_widget)
            layout.addWidget(cam_group)
        else:
            self.camera_widget = None

        layout.addStretch()

        # Initialise plate
        self._on_plate_changed()

    # ── Step 1 actions ────────────────────────────────────────────

    def _set_zero(self):
        self.controller._calibrate_zero()
        zp = self.controller.zero_position
        self.lbl_zero_status.setText(
            f"Zero: X={zp['x']:.0f}  Y={zp['y']:.0f}  Z={zp['Z']:.2f}")
        self.lbl_zero_status.setStyleSheet("color: #a6e3a1;")
        logger.info("Zero reference set")

    def _goto_zero(self):
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
        self.controller.move_z_absolute(0, from_zero_ref=True)

    # ── Step 2 actions ────────────────────────────────────────────

    def _on_plate_changed(self):
        fmt = self.plate_combo.currentData()
        if fmt is None:
            return
        self._plate = WellPlate.from_format(fmt)
        # Determine corner well
        last_row = chr(ord("A") + self._plate.rows - 1)
        self._corner_well = f"{last_row}{self._plate.cols}"
        self._taught_a1 = None
        self._taught_corner = None
        self.lbl_a1.setText("Not recorded")
        self.lbl_a1.setStyleSheet("color: #6c7086;")
        self.lbl_corner.setText(f"Not recorded ({self._corner_well})")
        self.lbl_corner.setStyleSheet("color: #6c7086;")
        self.lbl_alignment.setText("")

        # Fill validation combo
        self.val_well_combo.clear()
        if self._plate:
            # Add a selection of wells for validation
            wells = self._plate.well_names
            check_wells = [wells[0]]  # first
            if len(wells) > 1:
                check_wells.append(wells[len(wells) // 4])
                check_wells.append(wells[len(wells) // 2])
                check_wells.append(wells[3 * len(wells) // 4])
                check_wells.append(wells[-1])
            # Remove duplicates preserving order
            seen = set()
            for w in check_wells:
                if w not in seen:
                    self.val_well_combo.addItem(w)
                    seen.add(w)
            # Also add all wells
            self.val_well_combo.insertSeparator(len(seen))
            for w in wells:
                if w not in seen:
                    self.val_well_combo.addItem(w)

    def _record_a1(self):
        pos = self.controller.get_xy_position(cached=False)
        if pos[0] is None:
            self.lbl_a1.setText("Error: cannot read position")
            self.lbl_a1.setStyleSheet("color: #f38ba8;")
            return
        self._taught_a1 = (pos[0], pos[1])
        zx = pos[0] - self.controller.zero_position["x"]
        zy = pos[1] - self.controller.zero_position["y"]
        self.lbl_a1.setText(f"({zx:.0f}, {zy:.0f})")
        self.lbl_a1.setStyleSheet("color: #a6e3a1;")
        logger.info("A1 recorded at machine (%.0f, %.0f)", pos[0], pos[1])

    def _record_corner(self):
        pos = self.controller.get_xy_position(cached=False)
        if pos[0] is None:
            self.lbl_corner.setText("Error: cannot read position")
            self.lbl_corner.setStyleSheet("color: #f38ba8;")
            return
        self._taught_corner = (pos[0], pos[1])
        zx = pos[0] - self.controller.zero_position["x"]
        zy = pos[1] - self.controller.zero_position["y"]
        self.lbl_corner.setText(f"{self._corner_well}: ({zx:.0f}, {zy:.0f})")
        self.lbl_corner.setStyleSheet("color: #a6e3a1;")
        logger.info("%s recorded at machine (%.0f, %.0f)", self._corner_well, pos[0], pos[1])

    def _goto_a1(self):
        if self._taught_a1:
            zx = self.controller.zero_position["x"]
            zy = self.controller.zero_position["y"]
            self.controller.move_xy_absolute(
                self._taught_a1[0] - zx, self._taught_a1[1] - zy, from_zero_ref=True)

    def _goto_corner(self):
        if self._taught_corner:
            zx = self.controller.zero_position["x"]
            zy = self.controller.zero_position["y"]
            self.controller.move_xy_absolute(
                self._taught_corner[0] - zx, self._taught_corner[1] - zy, from_zero_ref=True)

    def _calculate_alignment(self):
        if not self._plate:
            self.lbl_alignment.setText("Select a plate format first.")
            return
        if not self._taught_a1 or not self._taught_corner:
            self.lbl_alignment.setText("Record both A1 and the corner well first.")
            return

        # Expected vector from A1 to corner in plate coordinates (mm → stage units)
        try:
            a1x, a1y = self._plate.get_well_position("A1")
            cx, cy = self._plate.get_well_position(self._corner_well)
        except KeyError as exc:
            self.lbl_alignment.setText(f"Well not found: {exc}")
            return

        expected_dx = cx - a1x  # mm
        expected_dy = cy - a1y

        # Actual vector (machine coords)
        actual_dx = self._taught_corner[0] - self._taught_a1[0]
        actual_dy = self._taught_corner[1] - self._taught_a1[1]

        # Calculate scale (steps per mm) and rotation
        exp_len = math.sqrt(expected_dx**2 + expected_dy**2)
        act_len = math.sqrt(actual_dx**2 + actual_dy**2)

        if exp_len < 0.001:
            self.lbl_alignment.setText("Error: expected distance is zero.")
            return

        scale = act_len / exp_len  # steps per mm

        exp_angle = math.atan2(expected_dy, expected_dx)
        act_angle = math.atan2(actual_dy, actual_dx)
        rotation_rad = act_angle - exp_angle
        self._rotation = math.degrees(rotation_rad)

        # A1 offset in zero-relative coordinates
        self._offset_x = self._taught_a1[0] - self.controller.zero_position["x"] - a1x * scale
        self._offset_y = self._taught_a1[1] - self.controller.zero_position["y"] - a1y * scale

        # Store the scale for well position calculations
        self._scale = scale

        self.lbl_alignment.setText(
            f"Scale: {scale:.1f} steps/mm | Rotation: {self._rotation:.2f}° | "
            f"Offset: ({self._offset_x:.0f}, {self._offset_y:.0f}) steps"
        )
        self.lbl_alignment.setStyleSheet("color: #a6e3a1;")
        logger.info("Alignment: scale=%.1f rot=%.2f° offset=(%.0f, %.0f)",
                     scale, self._rotation, self._offset_x, self._offset_y)

    # ── Step 3 actions ────────────────────────────────────────────

    def _goto_well(self):
        well_name = self.val_well_combo.currentText()
        if not well_name or not self._plate:
            return
        if not hasattr(self, "_scale"):
            self.lbl_val_result.setText("Run 'Calculate Alignment' first.")
            self.lbl_val_result.setStyleSheet("color: #f38ba8;")
            return

        try:
            wx, wy = self._plate.get_well_position(well_name)
        except KeyError:
            self.lbl_val_result.setText(f"Well {well_name} not found.")
            return

        # Transform plate coords → zero-relative stage coords
        rad = math.radians(self._rotation)
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        sx = (wx * cos_r - wy * sin_r) * self._scale + self._offset_x
        sy = (wx * sin_r + wy * cos_r) * self._scale + self._offset_y

        self.controller.move_xy_absolute(sx, sy, from_zero_ref=True)
        self.lbl_val_result.setText(f"Moving to {well_name}: ({sx:.0f}, {sy:.0f}) steps from zero")
        self.lbl_val_result.setStyleSheet("color: #89b4fa;")
        logger.info("Validate: moving to %s at (%.0f, %.0f)", well_name, sx, sy)

    # ── Timer update ──────────────────────────────────────────────

    def update_data(self):
        ctrl = self.controller
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            self.lbl_x.setText(f"{xy[0] - ctrl.zero_position['x']:.0f}")
            self.lbl_y.setText(f"{xy[1] - ctrl.zero_position['y']:.0f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.3f}")
        else:
            self.lbl_z.setText("—")

    # ── Enhancement 5: Calibration Persistence ────────────────────

    def _save_calibration(self):
        """Save current plate alignment to settings.json."""
        if self.settings is None:
            logger.warning("No settings object — cannot save calibration")
            return
        if not hasattr(self, "_scale"):
            QMessageBox.warning(self, "Save Error",
                                "Run 'Calculate Alignment' before saving.")
            return

        cal_data = {
            "plate_format": self.plate_combo.currentData(),
            "taught_a1": list(self._taught_a1) if self._taught_a1 else None,
            "taught_corner": list(self._taught_corner) if self._taught_corner else None,
            "corner_well": self._corner_well,
            "offset_x": self._offset_x,
            "offset_y": self._offset_y,
            "rotation": self._rotation,
            "scale": self._scale,
            "alignment_valid": True,
        }
        self.settings.set_section("calibration", cal_data)
        self.settings.save()
        logger.info("Calibration saved to settings.json")
        QMessageBox.information(self, "Saved",
                                "Plate alignment saved to settings.json")

    def _load_calibration(self):
        """Load plate alignment from settings.json."""
        if self.settings is None:
            return

        cal = self.settings.get_section("calibration")
        if not cal or not cal.get("alignment_valid"):
            logger.debug("No valid saved calibration found")
            return

        # Restore plate format
        fmt = cal.get("plate_format", 96)
        for i in range(self.plate_combo.count()):
            if self.plate_combo.itemData(i) == fmt:
                self.plate_combo.setCurrentIndex(i)
                break

        # Restore taught points
        a1 = cal.get("taught_a1")
        if a1 and len(a1) == 2:
            self._taught_a1 = tuple(a1)
            zx = a1[0] - self.controller.zero_position["x"]
            zy = a1[1] - self.controller.zero_position["y"]
            self.lbl_a1.setText(f"({zx:.0f}, {zy:.0f}) [saved]")
            self.lbl_a1.setStyleSheet("color: #89b4fa;")

        corner = cal.get("taught_corner")
        if corner and len(corner) == 2:
            self._taught_corner = tuple(corner)
            self._corner_well = cal.get("corner_well", "H12")
            zx = corner[0] - self.controller.zero_position["x"]
            zy = corner[1] - self.controller.zero_position["y"]
            self.lbl_corner.setText(
                f"{self._corner_well}: ({zx:.0f}, {zy:.0f}) [saved]")
            self.lbl_corner.setStyleSheet("color: #89b4fa;")

        # Restore computed alignment
        self._offset_x = cal.get("offset_x", 0.0)
        self._offset_y = cal.get("offset_y", 0.0)
        self._rotation = cal.get("rotation", 0.0)
        self._scale = cal.get("scale", 1.0)

        self.lbl_alignment.setText(
            f"[Loaded] Scale: {self._scale:.1f} steps/mm | "
            f"Rotation: {self._rotation:.2f}° | "
            f"Offset: ({self._offset_x:.0f}, {self._offset_y:.0f}) steps"
        )
        self.lbl_alignment.setStyleSheet("color: #89b4fa;")
        logger.info("Calibration loaded from settings.json")
