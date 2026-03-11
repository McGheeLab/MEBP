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
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from PySide6.QtGui import QPainter, QPen, QBrush, QColor

from SupportClasses.StageController import StageController
from SupportClasses.WellPlate import WellPlate, PLATE_DEFINITIONS
from gui.styles import COLORS, SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE
from gui.unit_helpers import steps_to_um, format_um, DEFAULT_MICROSTEPS_PER_MICRON

try:
    from SupportClasses.HardwareConfig import HardwareConfig, CameraConfig
except ImportError:
    HardwareConfig = None
    CameraConfig = None

logger = logging.getLogger(__name__)

# Optional camera support
try:
    from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE, detect_cameras
    try:
        from gui.widgets.camera_widget import CAMERA_AVAILABLE
    except ImportError:
        CAMERA_AVAILABLE = CV2_AVAILABLE
    try:
        from gui.widgets.camera_widget import CAMERA_AVAILABLE
    except ImportError:
        CAMERA_AVAILABLE = CV2_AVAILABLE
except ImportError:
    CameraWidget = None
    CV2_AVAILABLE = False

    def detect_cameras(max_index=8):
        return []

# v7.3.0: Optional vision detection support
try:
    from gui.widgets.detection_worker import DetectionWorker, DetectionMode
    from SupportClasses.VisionDetector import (
        DetectionResult, FocusResult, pixel_offset_to_stage_um,
    )
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    DetectionWorker = None
    DetectionMode = None

# Maximum simultaneous cameras
MAX_CAMERAS = 3


# ═══════════════════════════════════════════════════════════════════
# Calibration helper widgets  (v7.3.1-calviews)
# ═══════════════════════════════════════════════════════════════════

class _CalibrationPlateView(QGraphicsView):
    """Top-down plate view for calibration page — shows needle XY position."""

    # v7.2.7: Click signal for well navigation
    try:
        from PySide6.QtCore import Signal as _Sig
        well_clicked = _Sig(str)
    except Exception:
        pass

    SCALE  = 4.0
    WELL_R = 3.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._plate        = None
        self._taught_a1    = None
        self._taught_corner = None
        # v7.2.7: calibration workflow state
        self._safe_z = None          # mm (zero-ref) — safe travel height
        self._top_z = None           # mm (zero-ref) — plate top surface
        self._taught_a1_z = None     # mm (zero-ref) — Z at A1 glass bottom
        self._taught_corner_z = None # mm (zero-ref) — Z at corner
        self._taught_third = None    # (x_um, y_um) absolute — third teach point
        self._taught_third_z = None  # mm (zero-ref)
        self._third_well = None      # well name for third point
        self._z_plane_result = None  # PlaneResult from WellBottomDetector
        self._z_buffer_mm = 0.5      # approach buffer above Top Z

        self._corner_well  = "H12"
        self._needle_xy    = None
        self._needle_item  = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumHeight(130)

    def set_plate(self, plate):
        self._plate = plate
        self._rebuild()

    def set_taught_a1(self, pt):
        self._taught_a1 = pt

    def set_taught_corner(self, pt, corner_well):
        self._taught_corner = pt
        self._corner_well   = corner_well

    def set_needle_xy(self, x_mm, y_mm):
        self._needle_xy = (x_mm, y_mm)
        self._update_needle()

    def _rebuild(self):
        self._scene.clear()
        self._needle_item = None
        if self._plate is None:
            return
        S = self.SCALE
        wells = list(self._plate.get_all_wells())
        for well in wells:
            cx, cy, r = well.x * S, well.y * S, self.WELL_R
            self._scene.addEllipse(
                cx - r, cy - r, 2 * r, 2 * r,
                QPen(QColor("#585b70"), 0.5),
                QBrush(QColor("#313244")),
            ).setToolTip(well.name)

        # Highlight A1 and corner well
        wmap = {w.name: w for w in wells}
        for wname, color in [("A1", "#89b4fa"), (self._corner_well, "#74c7ec")]:
            w = wmap.get(wname)
            if w:
                cx, cy = w.x * S, w.y * S
                r2 = self.WELL_R + 1.5
                self._scene.addEllipse(
                    cx - r2, cy - r2, 2 * r2, 2 * r2,
                    QPen(QColor(color), 1.5),
                    QBrush(QColor("#00000000")),
                )

        # Needle placeholder
        self._needle_item = self._scene.addEllipse(
            -6, -6, 12, 12,
            QPen(QColor("#f38ba8"), 1.5),
            QBrush(QColor("#f38ba880")),
        )
        self._needle_item.setVisible(False)
        self._needle_item.setZValue(10)

        rect = self._scene.itemsBoundingRect().adjusted(-8, -8, 8, 8)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    def _update_needle(self):
        if self._needle_item is None:
            return
        if self._needle_xy is None:
            self._needle_item.setVisible(False)
            return
        x_mm, y_mm = self._needle_xy
        if self._taught_a1 is not None:
            x_mm = x_mm - self._taught_a1[0]
            y_mm = y_mm - self._taught_a1[1]
        S = self.SCALE
        self._needle_item.setRect(x_mm * S - 6, y_mm * S - 6, 12, 12)
        self._needle_item.setVisible(True)

    def mousePressEvent(self, event):
        """v7.2.7: Click a well to navigate to it."""
        if self._plate is None:
            super().mousePressEvent(event)
            return
        scene_pos = self.mapToScene(event.pos())
        S = self.SCALE
        # Hit-test wells
        best_well = None
        best_dist = float('inf')
        for well in self._plate.get_all_wells():
            cx, cy = well.x * S, well.y * S
            dx = scene_pos.x() - cx
            dy = scene_pos.y() - cy
            dist = (dx*dx + dy*dy) ** 0.5
            if dist < self.WELL_R + 2 and dist < best_dist:
                best_dist = dist
                best_well = well.name
        if best_well:
            try:
                self.well_clicked.emit(best_well)
            except AttributeError:
                pass
            logger.debug(f"Plate view clicked: {best_well}")
        else:
            super().mousePressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._plate:
            rect = self._scene.itemsBoundingRect().adjusted(-8, -8, 8, 8)
            if not rect.isEmpty():
                self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)


class _CalibrationYZView(QGraphicsView):
    """YZ side view for calibration — shows live needle Z and well-bottom offsets."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene    = QGraphicsScene(self)
        self.setScene(self._scene)
        self._z_min    = -2.0
        self._z_max    =  8.0
        self._needle_z = None
        self._offsets: dict = {}
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setFixedWidth(90)
        self.setMinimumHeight(130)

    def set_well_z_offsets(self, offsets: dict):
        self._offsets = offsets
        self._rebuild()

    def set_needle_z(self, z_mm):
        self._needle_z = z_mm
        self._rebuild()

    def _z_to_y(self, z, h=200):
        frac = (z - self._z_min) / max(self._z_max - self._z_min, 0.001)
        return h * (1.0 - frac)

    def _rebuild(self):
        self._scene.clear()
        h, w = 200, 70
        self._scene.addRect(0, 0, w, h,
                            QPen(QColor("#45475a"), 0),
                            QBrush(QColor("#11111b")))
        # Axis label
        from PySide6.QtWidgets import QGraphicsTextItem
        t = self._scene.addText("Z (mm)")
        t.setDefaultTextColor(QColor("#a6adc8"))
        t.setPos(2, 2)

        # Well-bottom tick lines
        for z in self._offsets.values():
            y = self._z_to_y(z, h)
            self._scene.addLine(10, y, w - 4, y, QPen(QColor("#585b70"), 0.8))

        # Z = 0 baseline
        y0 = self._z_to_y(0.0, h)
        self._scene.addLine(10, y0, w - 4, y0, QPen(QColor("#6c7086"), 1.0))

        # Live needle Z
        if self._needle_z is not None:
            yn = self._z_to_y(self._needle_z, h)
            self._scene.addLine(0, yn, w, yn, QPen(QColor("#f38ba8"), 2.0))
            zt = self._scene.addText(f"{self._needle_z:.2f}")
            zt.setDefaultTextColor(QColor("#f38ba8"))
            zt.setPos(2, max(0, yn - 14))

        self.setSceneRect(0, 0, w, h)
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.IgnoreAspectRatio)


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

        # v7.1.1: Microsteps-to-microns conversion factor
        self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON
        self._hardware_config = None  # v7.2: HardwareConfig

        # v7.3.0: Auto-detection state
        self._detection_worker: DetectionWorker | None = None
        self._last_well_result = None       # Latest DetectionResult from worker
        self._autodetect_target: str = ""   # "a1", "corner", "third"

        # v7.3.0 Phase 6: Needle detection + focus assist state
        self._last_needle_result = None     # Latest needle DetectionResult
        self._needle_detecting: bool = False
        self._focus_assisting: bool = False
        self._best_focus_z: float | None = None       # Best-focus Z (mm, zero-ref)
        self._best_focus_score: float = 0.0            # Score at best Z

        # v7.2.7-hotfix: ensure all calibration attrs

        for _a in ['_safe_z','_top_z','_taught_a1_z','_taught_corner_z',

                  '_taught_third','_taught_third_z','_third_well',

                  '_z_plane_result']:

            if not hasattr(self, _a): setattr(self, _a, None)

        if not hasattr(self, '_z_buffer_mm'): self._z_buffer_mm = 0.5

        if not hasattr(self, '_corner_well'): self._corner_well = 'H12'


        self._setup_ui()

    def get_page_title(self) -> str:
        return "Calibration"

    def set_microsteps_per_micron(self, value: float):
        """Update the microsteps-per-micron conversion factor."""
        self._microsteps_per_micron = value

    def set_hardware_config(self, config):
        """v7.2.4: Set hardware config — sync plate format, needle info, and plate model.

        Syncs BOTH the main plate_combo (if it exists) and the context panel
        ctx_plate_combo. Also rebuilds the WellPlate model and validation
        well combo so calibration uses the correct plate geometry.
        """
        self._hardware_config = config
        if config is None:
            return

        # Auto-select plate format in context panel combo
        if hasattr(config, 'plate_format') and config.plate_format:
            fmt = config.plate_format
            # Sync main plate_combo (if page has one)
            if hasattr(self, 'plate_combo'):
                for i in range(self.plate_combo.count()):
                    if self.plate_combo.itemData(i) == fmt:
                        self.plate_combo.setCurrentIndex(i)
                        break
            # Sync context panel plate combo
            if hasattr(self, 'ctx_plate_combo'):
                for i in range(self.ctx_plate_combo.count()):
                    if self.ctx_plate_combo.itemData(i) == fmt:
                        if self.ctx_plate_combo.currentIndex() != i:
                            self.ctx_plate_combo.setCurrentIndex(i)
                        break
            # Rebuild the plate model directly so calibration always matches HW config
            self._plate = WellPlate.from_format(fmt)
            defn = PLATE_DEFINITIONS[fmt]
            rows, cols = defn["rows"], defn["cols"]
            self._corner_well = f"{chr(ord('A') + rows - 1)}{cols}"
            if hasattr(self, 'val_well_combo'):
                wells = self._plate.well_names
                self.val_well_combo.clear()
                self.val_well_combo.addItems(wells)
            logger.info(f"Calibration: plate format synced to {fmt}-well from HardwareConfig")

        # Show needle info in calibration context panel
        if hasattr(config, 'needle') and config.needle:
            n = config.needle
            needle_text = (f"Needle: {n.gauge}G | ID: {n.id_um:.0f} \u00b5m | "
                           f"Length: {n.length_inches:.1f}\"")
            if hasattr(self, 'ctx_lbl_needle_info'):
                self.ctx_lbl_needle_info.setText(needle_text)



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

        # v7.3.0 Phase 6: Needle detection + focus assist
        if VISION_AVAILABLE:
            needle_label = QLabel("Needle Alignment")
            needle_label.setObjectName("contextSectionLabel")
            layout.addWidget(needle_label)

            # Detect Needle button row
            nd_row = QHBoxLayout()
            self._btn_detect_needle = QPushButton("Detect Needle")
            self._btn_detect_needle.setMaximumHeight(24)
            self._btn_detect_needle.setCheckable(True)
            self._btn_detect_needle.setToolTip(
                "Detect needle tip in camera FOV using vision")
            self._btn_detect_needle.toggled.connect(self._toggle_needle_detect)
            nd_row.addWidget(self._btn_detect_needle)

            self._btn_focus_assist = QPushButton("Focus Assist")
            self._btn_focus_assist.setMaximumHeight(24)
            self._btn_focus_assist.setCheckable(True)
            self._btn_focus_assist.setToolTip(
                "Real-time focus quality bar — adjust Z for sharpest image")
            self._btn_focus_assist.setEnabled(False)
            self._btn_focus_assist.toggled.connect(self._toggle_focus_assist)
            nd_row.addWidget(self._btn_focus_assist)
            layout.addLayout(nd_row)

            # Needle detection status
            self._lbl_needle_status = QLabel("")
            self._lbl_needle_status.setWordWrap(True)
            self._lbl_needle_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_needle_status)

            # Focus quality display
            self._lbl_focus_status = QLabel("")
            self._lbl_focus_status.setWordWrap(True)
            self._lbl_focus_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_focus_status)

            # Best focus Z
            self._lbl_best_focus = QLabel("")
            self._lbl_best_focus.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_best_focus)

        # ── Step 2: Teach Plate Position ─────────────────────────
        # ── Step 2A — Safe Z ─────────────────────────────
        s2a_label = QLabel("Step 2A — Safe Z")
        s2a_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2a_label)
        layout.addWidget(QLabel("Raise needle to safe travel height, then set."))
        s2a_row = QHBoxLayout()
        btn_safe_z = QPushButton("Set Safe Z")
        btn_safe_z.setObjectName("successBtn")
        btn_safe_z.setMaximumHeight(26)
        btn_safe_z.clicked.connect(self._set_safe_z)
        s2a_row.addWidget(btn_safe_z)
        self.lbl_safe_z = QLabel("Not set")
        self.lbl_safe_z.setStyleSheet(f"color: {COLORS['yellow']};")
        s2a_row.addWidget(self.lbl_safe_z, stretch=1)
        layout.addLayout(s2a_row)

        # ── Step 2B — Top Z ──────────────────────────────
        s2b_label = QLabel("Step 2B — Top Z (Plate Surface)")
        s2b_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2b_label)
        layout.addWidget(QLabel("Lower needle to plate top surface, then set."))
        s2b_row = QHBoxLayout()
        btn_top_z = QPushButton("Set Top Z")
        btn_top_z.setMaximumHeight(26)
        btn_top_z.clicked.connect(self._set_top_z)
        s2b_row.addWidget(btn_top_z)
        self.lbl_top_z = QLabel("Not set")
        self.lbl_top_z.setStyleSheet(f"color: {COLORS['yellow']};")
        s2b_row.addWidget(self.lbl_top_z, stretch=1)
        layout.addLayout(s2b_row)

        # ── Step 2C — Teach A1 XYZ ──────────────────────
        s2c_label = QLabel("Step 2C — Teach A1 (Center Bottom)")
        s2c_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2c_label)
        layout.addWidget(QLabel("Jog to center bottom of well A1."))
        a1_row = QHBoxLayout()
        a1_row.addWidget(QLabel("A1:"))
        self.lbl_a1 = QLabel("—")
        self.lbl_a1.setStyleSheet(f"color: {COLORS['overlay0']};")
        a1_row.addWidget(self.lbl_a1, stretch=1)
        btn_rec_a1 = QPushButton("Rec XYZ")
        btn_rec_a1.setMaximumHeight(24)
        btn_rec_a1.setMaximumWidth(60)
        btn_rec_a1.clicked.connect(self._record_a1_xyz)
        a1_row.addWidget(btn_rec_a1)
        btn_go_a1 = QPushButton("Go")
        btn_go_a1.setMaximumHeight(24)
        btn_go_a1.setMaximumWidth(30)
        btn_go_a1.clicked.connect(self._goto_a1)
        a1_row.addWidget(btn_go_a1)
        layout.addLayout(a1_row)

        # v7.3.0: Auto-detect controls for A1
        if VISION_AVAILABLE:
            a1_detect_row = QHBoxLayout()
            self._btn_detect_a1 = QPushButton("Auto-Detect Well")
            self._btn_detect_a1.setMaximumHeight(24)
            self._btn_detect_a1.setCheckable(True)
            self._btn_detect_a1.setToolTip("Use camera to detect well center automatically")
            self._btn_detect_a1.toggled.connect(
                lambda on: self._toggle_well_detect("a1", on))
            a1_detect_row.addWidget(self._btn_detect_a1)
            self._btn_center_a1 = QPushButton("Center")
            self._btn_center_a1.setMaximumHeight(24)
            self._btn_center_a1.setMaximumWidth(50)
            self._btn_center_a1.setToolTip("Move stage to center well in camera FOV")
            self._btn_center_a1.setEnabled(False)
            self._btn_center_a1.clicked.connect(
                lambda: self._center_on_detection("a1"))
            a1_detect_row.addWidget(self._btn_center_a1)
            self._btn_accept_a1 = QPushButton("Accept")
            self._btn_accept_a1.setMaximumHeight(24)
            self._btn_accept_a1.setMaximumWidth(50)
            self._btn_accept_a1.setToolTip("Accept detected position as A1")
            self._btn_accept_a1.setEnabled(False)
            self._btn_accept_a1.clicked.connect(
                lambda: self._accept_detection("a1"))
            a1_detect_row.addWidget(self._btn_accept_a1)
            layout.addLayout(a1_detect_row)

            # Detection status label for A1
            self._lbl_detect_a1 = QLabel("")
            self._lbl_detect_a1.setWordWrap(True)
            self._lbl_detect_a1.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_detect_a1)

        # ── Step 2D — Teach Corner (auto-navigate) ──────
        s2d_label = QLabel("Step 2D — Teach Corner (Auto-Navigate)")
        s2d_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2d_label)
        cr_row = QHBoxLayout()
        cr_row.addWidget(QLabel("Corner:"))
        self.lbl_corner = QLabel("—")
        self.lbl_corner.setStyleSheet(f"color: {COLORS['overlay0']};")
        cr_row.addWidget(self.lbl_corner, stretch=1)
        btn_go_corner_auto = QPushButton("Go ▶")
        btn_go_corner_auto.setMaximumHeight(24)
        btn_go_corner_auto.setMaximumWidth(40)
        btn_go_corner_auto.setToolTip("Safe-travel to estimated corner position")
        btn_go_corner_auto.clicked.connect(self._goto_corner_auto)
        cr_row.addWidget(btn_go_corner_auto)
        btn_rec_corner = QPushButton("Rec XYZ")
        btn_rec_corner.setMaximumHeight(24)
        btn_rec_corner.setMaximumWidth(60)
        btn_rec_corner.clicked.connect(self._record_corner_xyz)
        cr_row.addWidget(btn_rec_corner)
        layout.addLayout(cr_row)

        # v7.3.0: Auto-detect controls for Corner
        if VISION_AVAILABLE:
            cr_detect_row = QHBoxLayout()
            self._btn_detect_corner = QPushButton("Auto-Detect Well")
            self._btn_detect_corner.setMaximumHeight(24)
            self._btn_detect_corner.setCheckable(True)
            self._btn_detect_corner.setToolTip(
                "Use camera to detect corner well center")
            self._btn_detect_corner.toggled.connect(
                lambda on: self._toggle_well_detect("corner", on))
            cr_detect_row.addWidget(self._btn_detect_corner)
            self._btn_center_corner = QPushButton("Center")
            self._btn_center_corner.setMaximumHeight(24)
            self._btn_center_corner.setMaximumWidth(50)
            self._btn_center_corner.setEnabled(False)
            self._btn_center_corner.clicked.connect(
                lambda: self._center_on_detection("corner"))
            cr_detect_row.addWidget(self._btn_center_corner)
            self._btn_accept_corner = QPushButton("Accept")
            self._btn_accept_corner.setMaximumHeight(24)
            self._btn_accept_corner.setMaximumWidth(50)
            self._btn_accept_corner.setEnabled(False)
            self._btn_accept_corner.clicked.connect(
                lambda: self._accept_detection("corner"))
            cr_detect_row.addWidget(self._btn_accept_corner)
            layout.addLayout(cr_detect_row)

            self._lbl_detect_corner = QLabel("")
            self._lbl_detect_corner.setWordWrap(True)
            self._lbl_detect_corner.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_detect_corner)

        # ── Step 2E — Teach Third Point ──────────────────
        s2e_label = QLabel("Step 2E — Third Point (Z Plane)")
        s2e_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2e_label)
        th_row = QHBoxLayout()
        self.lbl_third = QLabel("—")
        self.lbl_third.setStyleSheet(f"color: {COLORS['overlay0']};")
        th_row.addWidget(self.lbl_third, stretch=1)
        btn_go_third = QPushButton("Go ▶")
        btn_go_third.setMaximumHeight(24)
        btn_go_third.setMaximumWidth(40)
        btn_go_third.setToolTip("Safe-travel to estimated third point")
        btn_go_third.clicked.connect(self._goto_third_auto)
        th_row.addWidget(btn_go_third)
        btn_rec_third = QPushButton("Rec XYZ")
        btn_rec_third.setMaximumHeight(24)
        btn_rec_third.setMaximumWidth(60)
        btn_rec_third.clicked.connect(self._record_third_xyz)
        th_row.addWidget(btn_rec_third)
        layout.addLayout(th_row)

        # ── Z Plane Status ────────────────────────────────
        self.lbl_zplane = QLabel("0/3 teach points")
        self.lbl_zplane.setWordWrap(True)
        self.lbl_zplane.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(self.lbl_zplane)

        self._gen_status = QLabel("")
        self._gen_status.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(self._gen_status)

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
        for name, attr in [("X (µm):", "lbl_x"), ("Y (µm):", "lbl_y"), ("Z (mm):", "lbl_z")]:
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

        if CAMERA_AVAILABLE and CameraWidget is not None:
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

        # v7.3.1-calviews: plate + YZ views
        self._build_cal_views(layout)

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATE
    # ════════════════════════════════════════════════════════════════

    def _on_detect_cameras(self):
        """v7.2.6: Manually trigger camera detection in background."""
        try:
            from gui.widgets.camera_widget import detect_cameras_async, CV2_AVAILABLE
        except ImportError:
            logger.warning("camera_widget not available")
            return

        if not CAMERA_AVAILABLE:
            logger.warning("No camera backend available")
            return

        if hasattr(self, '_btn_detect_cameras'):
            self._btn_detect_cameras.setEnabled(False)
            self._btn_detect_cameras.setText("🔄 Detecting...")

        def _on_found(indices):
            from PySide6.QtCore import QTimer
            def _apply():
                self._available_cameras = indices
                if hasattr(self, '_btn_detect_cameras'):
                    self._btn_detect_cameras.setText(f"🔍 {len(indices)} camera(s)")
                    self._btn_detect_cameras.setEnabled(True)
                logger.info(f"Camera detection: {indices}")
            QTimer.singleShot(0, _apply)

        detect_cameras_async(_on_found, max_index=4)



    # ── Plate + YZ views (v7.3.1-calviews) ──────────────────────

    def _build_cal_views(self, parent_layout) -> None:
        """Add plate-view + YZ-view row below camera card."""
        from PySide6.QtWidgets import QGroupBox
        grp = QGroupBox("Plate Position View")
        grp.setStyleSheet(
            "QGroupBox { color: #cdd6f4; font-weight: bold; "
            "border: 1px solid #45475a; border-radius: 4px; "
            "margin-top: 6px; padding-top: 12px; }")
        grp_layout = QHBoxLayout(grp)
        grp_layout.setSpacing(4)
        grp_layout.setContentsMargins(4, 4, 4, 4)

        self._cal_plate_view = _CalibrationPlateView()
        # v7.2.7: Connect click-to-navigate
        if hasattr(self._cal_plate_view, 'well_clicked'):
            self._cal_plate_view.well_clicked.connect(self._navigate_to_well)

        grp_layout.addWidget(self._cal_plate_view, stretch=1)

        self._cal_yz_view = _CalibrationYZView()
        grp_layout.addWidget(self._cal_yz_view)

        parent_layout.addWidget(grp)

    def _start_cal_position_poll(self) -> None:
        """Start 250 ms timer to poll needle position."""
        if not hasattr(self, "_cal_pos_timer"):
            self._cal_pos_timer = QTimer(self)
            self._cal_pos_timer.setInterval(250)
            self._cal_pos_timer.timeout.connect(self._poll_cal_position)
        self._cal_pos_timer.start()

    def _poll_cal_position(self) -> None:
        """Poll XY+Z and push to plate + YZ views."""
        ctrl = self.controller
        if ctrl is None:
            return
        try:
            xy = ctrl.get_xy_position()
            zp = ctrl.get_zp_position()
        except Exception:
            return
        mpm = self._microsteps_per_micron or 1.0
        try:
            if isinstance(xy, dict):
                x_s, y_s = xy.get("x", 0) or 0, xy.get("y", 0) or 0
            elif xy and len(xy) >= 2:
                x_s, y_s = xy[0], xy[1]
            else:
                return
            x_mm = float(x_s) / (mpm * 1000.0)
            y_mm = float(y_s) / (mpm * 1000.0)
        except Exception:
            return
        try:
            z_mm = float(zp.get("Z", 0) or 0) if isinstance(zp, dict)                    else (float(zp[0]) if zp and len(zp) >= 1 else 0.0)
        except Exception:
            z_mm = 0.0

        if hasattr(self, "_cal_plate_view"):
            self._cal_plate_view.set_needle_xy(x_mm, y_mm)
        if hasattr(self, "_cal_yz_view"):
            self._cal_yz_view.set_needle_z(z_mm)

    def _update_cal_view_plate(self) -> None:
        """Push current plate model to cal views."""
        if not hasattr(self, "_cal_plate_view"):
            return
        if self._plate is not None:
            self._cal_plate_view.set_plate(self._plate)
            self._cal_plate_view.set_taught_a1(self._taught_a1)
            self._cal_plate_view.set_taught_corner(
                self._taught_corner, self._corner_well)
        if hasattr(self, "_cal_yz_view") and self._plate is not None:
            offsets = {}
            if hasattr(self._plate, "get_all_z_offsets"):
                offsets = self._plate.get_all_z_offsets()
            self._cal_yz_view.set_well_z_offsets(offsets)


    def _configure_simulated_cameras(self):
        """v7.3.0: Push world state to any simulated camera backends."""
        for cam in self._cameras:
            sim = cam.simulated_camera
            if sim is None:
                continue

            # Wire controller for auto position pull
            if sim._controller is None and self.controller is not None:
                sim.set_controller(self.controller)

            # Push plate geometry
            if self._plate is not None:
                origin = self._taught_a1 if self._taught_a1 else (50000.0, 50000.0)
                sim.set_plate(self._plate, plate_origin_um=origin)

            # Push camera config
            cam_cfg = self._get_camera_config()
            if cam_cfg is not None and cam_cfg.micron_per_pixel:
                sim._um_per_px = cam_cfg.micron_per_pixel
                sim._fov_w_um = sim._width * sim._um_per_px
                sim._fov_h_um = sim._height * sim._um_per_px

            # Push needle spec
            hw = self._hardware_config
            if hw is not None and getattr(hw, 'needle', None) is not None:
                sim.set_needle(hw.needle.od_um, hw.needle.id_um)

    def on_status_update(self):
        """Called periodically by the main window."""
        # v7.3.0: Keep simulated cameras synced
        self._configure_simulated_cameras()

        ctrl = self.controller

        # Update position readout
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            # v7.2.7: controller reports µm directly — no conversion needed
            ux = xy[0] - ctrl.zero_position["x"]
            uy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{ux:,.1f}")
            self.lbl_y.setText(f"{uy:,.1f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")

        zp = ctrl.get_zp_position(cached=True)
        if isinstance(zp, (list, tuple)) and len(zp) >= 1 and zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position.get('Z', 0):.2f}")

            # v7.2.7: feed needle to plate view
            if hasattr(self, '_cal_plate_view') and self._taught_a1 is not None:
                zero_x = ctrl.zero_position.get('x', 0)
                zero_y = ctrl.zero_position.get('y', 0)
                nx_mm = (xy[0] - self._taught_a1[0]) / 1000.0
                ny_mm = (xy[1] - self._taught_a1[1]) / 1000.0
                self._cal_plate_view.set_needle_xy(nx_mm, ny_mm)
        else:
            self.lbl_z.setText("—")

    # ════════════════════════════════════════════════════════════════
    #  CALIBRATION STEPS
    # ════════════════════════════════════════════════════════════════

    # ── Step 1: Zero Needle ──────────────────────────────────────

    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7-simple: blocking absolute moves. Simulator now blocks like real hw."""
        ctrl = self.controller
        safe_z = getattr(self, '_safe_z', None) or 0.0

        # Step 1: Raise Z
        if ctrl.is_zp_connected:
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)

        # Step 2: Fast XY travel (blocks until arrival on both real hw and simulator)
        if ctrl.is_xy_connected:
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):
                    ctrl.xy_stage.set_speed_mm_s(50.0)
                else:
                    ctrl.xy_stage.set_velocity(100)
            ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)

        # Step 3: Lower Z
        if ctrl.is_zp_connected:
            if target_z_mm is not None:
                ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
            elif getattr(self, '_top_z', None) is not None:
                approach = self._top_z + getattr(self, '_z_buffer_mm', 0.5)
                ctrl.move_z_absolute(approach, from_zero_ref=True)

        logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")


    def _estimate_well_position_um(self, well_name):
        """v7.2.7-scalefix: estimate well position — ignore scale/rotation until calibrated.

        After A1 is taught but BEFORE corner is taught, scale/rotation
        should be 1.0/0.0. Only apply non-identity transform if both
        A1 and corner have been taught AND scale is reasonable.
        """
        if self._taught_a1 is None or self._plate is None:
            return None
        try:
            wx, wy = self._plate.get_well_position(well_name)
            a1x, a1y = self._plate.get_well_position("A1")
            dx_mm = wx - a1x
            dy_mm = wy - a1y

            # Only apply scale/rotation if both points taught AND scale is sane
            scale = getattr(self, '_scale', 1.0)
            rot = getattr(self, '_rotation', 0.0)
            has_corner = getattr(self, '_taught_corner', None) is not None

            if has_corner and 0.8 < scale < 1.2 and abs(rot) < 10:
                # Apply calibrated transform
                if abs(rot) > 0.001:
                    import math as _m
                    rad = _m.radians(rot)
                    dx_mm, dy_mm = (dx_mm*_m.cos(rad) - dy_mm*_m.sin(rad),
                                    dx_mm*_m.sin(rad) + dy_mm*_m.cos(rad))
                dx_mm *= scale
                dy_mm *= scale
            elif has_corner and (scale < 0.8 or scale > 1.2):
                logger.warning(f"Ignoring stale scale={scale:.2f} (out of range 0.8-1.2)")

            tx = self._taught_a1[0] + dx_mm * 1000.0
            ty = self._taught_a1[1] + dy_mm * 1000.0
            logger.info(f"Est {well_name}: offset ({dx_mm:.2f},{dy_mm:.2f})mm, "
                        f"scale={scale:.4f}, rot={rot:.2f}° "
                        f"-> ({tx:.0f},{ty:.0f}) µm")
            return (tx, ty)
        except Exception as e:
            logger.warning(f"Cannot estimate {well_name}: {e}")
            return None


    def _pick_third_well(self):
        """v7.2.7: Choose a third teach point that maximizes triangle area.

        Given A1 and the diagonal corner, pick a well on the other diagonal.
        For 96-well H12 corner: third = A12 or H1.
        """
        if not self._plate:
            return None
        rows = self._plate.rows
        cols = self._plate.cols
        corner_letter = chr(ord('A') + rows - 1)
        # If corner is bottom-right (e.g., H12), pick top-right (A12)
        third = f"A{cols}"
        # If that's A1, pick bottom-left instead
        if third == "A1":
            third = f"{corner_letter}1"
        return third

    def _set_zero(self):
        self.controller._calibrate_zero()
        z = self.controller.zero_position
        # v7.2.7: zero positions already in µm
        zx_um = z['x']
        zy_um = z['y']
        self.lbl_zero_status.setText(
            f"Set: X={zx_um:,.1f} µm  Y={zy_um:,.1f} µm  Z={z['Z']:.2f} mm")
        self.lbl_zero_status.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Zero set: {z}")

    def _goto_zero(self):
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
        self.controller.move_z_absolute(0, from_zero_ref=True)

    # ── Step 2: Teach Well Plate ─────────────────────────────────

    def _set_safe_z(self):
        """v7.2.7: Record current Z as safe travel height."""
        zp = self.controller.get_zp_position(cached=False)
        if zp is None or zp[0] is None:
            return
        self._safe_z = zp[0] - self.controller.zero_position.get("Z", 0)
        if hasattr(self, 'lbl_safe_z'):
            self.lbl_safe_z.setText(f"Safe Z: {self._safe_z:.2f} mm")
            self.lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Safe Z set: {self._safe_z:.2f} mm")

    def _set_top_z(self):
        """v7.2.7: Record current Z as plate top surface."""
        zp = self.controller.get_zp_position(cached=False)
        if zp is None or zp[0] is None:
            return
        self._top_z = zp[0] - self.controller.zero_position.get("Z", 0)
        if hasattr(self, 'lbl_top_z'):
            self.lbl_top_z.setText(f"Top Z: {self._top_z:.2f} mm")
            self.lbl_top_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Top Z set: {self._top_z:.2f} mm")

    def _record_a1_xyz(self):
        """v7.2.7: Record A1 with full XYZ."""
        xy = self.controller.get_xy_position(cached=False)
        zp = self.controller.get_zp_position(cached=False)
        if xy[0] is None:
            return
        self._taught_a1 = (xy[0], xy[1])
        ax = xy[0] - self.controller.zero_position["x"]
        ay = xy[1] - self.controller.zero_position["y"]
        if zp is not None and zp[0] is not None:
            self._taught_a1_z = zp[0] - self.controller.zero_position.get("Z", 0)
        z_str = f"  Z: {self._taught_a1_z:.2f} mm" if self._taught_a1_z is not None else ""
        if hasattr(self, 'lbl_a1'):
            self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) µm{z_str}")
            self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")
        # Update plate view
        if hasattr(self, '_cal_plate_view'):
            self._cal_plate_view.set_taught_a1((ax / 1000.0, ay / 1000.0))
            self._cal_plate_view._rebuild()
        # v7.2.7-scalefix: reset scale/rotation — only valid after corner is taught
        self._scale = 1.0
        self._rotation = 0.0
        self._offset_x = 0
        self._offset_y = 0
        logger.info(f"Taught A1 XYZ: {self._taught_a1}, Z={self._taught_a1_z}")

    def _goto_corner_auto(self):
        """v7.2.7: Auto-navigate to estimated corner position with safe travel."""
        if self._taught_a1 is None or self._plate is None:
            QMessageBox.warning(self, "Cannot Navigate",
                                "Record A1 and select plate format first.")
            return
        est = self._estimate_well_position_um(self._corner_well)
        if est is None:
            return
        self._safe_navigate_to(est[0], est[1])
        if hasattr(self, '_gen_status'):
            self._gen_status.setText(f"Navigated to {self._corner_well} estimate")

    def _record_corner_xyz(self):
        """v7.2.7: Record corner with full XYZ."""
        xy = self.controller.get_xy_position(cached=False)
        zp = self.controller.get_zp_position(cached=False)
        if xy[0] is None:
            return
        self._taught_corner = (xy[0], xy[1])
        cx = xy[0] - self.controller.zero_position["x"]
        cy = xy[1] - self.controller.zero_position["y"]
        if zp is not None and zp[0] is not None:
            self._taught_corner_z = zp[0] - self.controller.zero_position.get("Z", 0)
        z_str = f"  Z: {self._taught_corner_z:.2f} mm" if self._taught_corner_z is not None else ""
        if hasattr(self, 'lbl_corner'):
            self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) µm{z_str}")
            self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")
        if hasattr(self, '_cal_plate_view'):
            self._cal_plate_view.set_taught_corner(
                (cx / 1000.0, cy / 1000.0), self._corner_well)
            self._cal_plate_view._rebuild()
        logger.info(f"Taught corner XYZ: {self._taught_corner}, Z={self._taught_corner_z}")
        # Auto-try Z plane fit
        self._try_fit_z_plane()

    def _goto_third_auto(self):
        """v7.2.7: Auto-navigate to estimated third teach point."""
        if self._taught_a1 is None or self._plate is None:
            QMessageBox.warning(self, "Cannot Navigate",
                                "Record A1 and select plate format first.")
            return
        self._third_well = self._pick_third_well()
        if self._third_well is None:
            return
        est = self._estimate_well_position_um(self._third_well)
        if est is None:
            return
        self._safe_navigate_to(est[0], est[1])
        if hasattr(self, 'lbl_third'):
            self.lbl_third.setText(f"At {self._third_well} estimate — fine-tune & record")
            self.lbl_third.setStyleSheet(f"color: {COLORS['yellow']};")

    def _record_third_xyz(self):
        """v7.2.7: Record third teach point with XYZ."""
        xy = self.controller.get_xy_position(cached=False)
        zp = self.controller.get_zp_position(cached=False)
        if xy[0] is None:
            return
        self._taught_third = (xy[0], xy[1])
        tx = xy[0] - self.controller.zero_position["x"]
        ty = xy[1] - self.controller.zero_position["y"]
        if zp is not None and zp[0] is not None:
            self._taught_third_z = zp[0] - self.controller.zero_position.get("Z", 0)
        well_name = self._third_well or "3rd"
        z_str = f"  Z: {self._taught_third_z:.2f} mm" if self._taught_third_z is not None else ""
        if hasattr(self, 'lbl_third'):
            self.lbl_third.setText(f"{well_name}: ({tx:,.1f}, {ty:,.1f}) µm{z_str}")
            self.lbl_third.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Taught third XYZ: {self._taught_third}, Z={self._taught_third_z}")
        self._try_fit_z_plane()

    def _try_fit_z_plane(self):
        """v7.2.7-hotfix: try_fit — Z-plane fit with 3 points."""
        points = []
        if getattr(self, '_taught_a1', None) and getattr(self, '_taught_a1_z', None) is not None:
            points.append(("A1", self._taught_a1_z))
        if getattr(self, '_taught_corner', None) and getattr(self, '_taught_corner_z', None) is not None:
            points.append((getattr(self, '_corner_well', 'corner'), self._taught_corner_z))
        if getattr(self, '_taught_third', None) and getattr(self, '_taught_third_z', None) is not None:
            points.append((getattr(self, '_third_well', '3rd'), self._taught_third_z))
        if len(points) < 3:
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"{len(points)}/3 points — need {3-len(points)} more")
            return
        try:
            from SupportClasses.WellSetup import WellBottomDetector
            det = WellBottomDetector(self._plate)
            for name, z in points:
                det.add_point(name, z)
            result = det.fit_plane()
            if result:
                self._z_plane_result = result
                if hasattr(self, 'lbl_zplane'):
                    self.lbl_zplane.setText(f"Z plane: {result.describe()}")
                    self.lbl_zplane.setStyleSheet(f"color: {COLORS['green']};")
                logger.info(f"Z plane: {result.describe()}")
        except Exception as e:
            logger.error(f"Z plane fit failed: {e}")
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"Fit failed: {e}")


    def _navigate_to_well(self, well_name):
        """v7.2.7: Navigate to any well via plate view click."""
        if self._safe_z is None:
            QMessageBox.warning(self, "No Safe Z",
                                "Set Safe Z height before navigating.")
            return
        if self._taught_a1 is None:
            QMessageBox.warning(self, "No A1",
                                "Teach A1 position before navigating to wells.")
            return
        est = self._estimate_well_position_um(well_name)
        if est is None:
            return
        # Estimate Z from plane if available
        target_z = None
        if self._z_plane_result:
            try:
                rel_x, rel_y = self._plate.get_well_position(well_name)
                target_z = self._z_plane_result.z_at(rel_x, rel_y) + self._z_buffer_mm
            except Exception:
                pass
        self._safe_navigate_to(est[0], est[1], target_z)
        logger.info(f"Navigated to well {well_name}")

    def _on_plate_changed(self, idx):
        """v7.2.7-hotfix: plate_changed — guards all widget refs."""
        sender = self.sender()
        if sender is None:
            return
        fmt = sender.currentData()
        if fmt is None:
            return
        self._plate = WellPlate.from_format(fmt)
        defn = PLATE_DEFINITIONS[fmt]
        rows, cols = defn["rows"], defn["cols"]
        self._corner_well = f"{chr(ord('A') + rows - 1)}{cols}"
        if hasattr(self, 'val_well_combo'):
            self.val_well_combo.clear()
            self.val_well_combo.addItems(self._plate.well_names)
        self._taught_a1 = None
        self._scale = 1.0
        self._rotation = 0.0

        self._taught_corner = None
        if hasattr(self, '_taught_third'): self._taught_third = None
        for attr in ['lbl_a1','lbl_corner','lbl_third']:
            if hasattr(self, attr): getattr(self, attr).setText("—")
        if hasattr(self, 'lbl_alignment'): self.lbl_alignment.setText("")
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view:
            self._cal_plate_view.set_plate(self._plate)


    def _record_a1(self):

        """v7.2.7: Record A1 position — positions are µm directly."""

        xy = self.controller.get_xy_position(cached=False)

        zp = self.controller.get_zp_position(cached=False)

        if xy[0] is None:

            return

        self._taught_a1 = (xy[0], xy[1])

        # Display zero-referenced position in µm

        ax = xy[0] - self.controller.zero_position["x"]

        ay = xy[1] - self.controller.zero_position["y"]

        self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) µm")

        self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

        # Store Z if available

        if zp is not None and zp[0] is not None:

            self._taught_a1_z = zp[0] - self.controller.zero_position.get("Z", 0)

        logger.info(f"Taught A1: {self._taught_a1}")


    def _goto_a1(self):


        """v7.2.7: Navigate to taught A1 — raw µm, no zero-ref."""


        if self._taught_a1:


            # _taught_a1 stores absolute stage position in µm


            # Use from_zero_ref=False since it's already absolute


            self.controller.move_xy_absolute(


                self._taught_a1[0], self._taught_a1[1],


                from_zero_ref=False,


            )


    def _record_corner(self):

        """v7.2.7: Record corner position — positions are µm directly."""

        xy = self.controller.get_xy_position(cached=False)

        zp = self.controller.get_zp_position(cached=False)

        if xy[0] is None:

            return

        self._taught_corner = (xy[0], xy[1])

        # Display zero-referenced position in µm

        cx = xy[0] - self.controller.zero_position["x"]

        cy = xy[1] - self.controller.zero_position["y"]

        self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) µm")

        self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")

        # Store Z if available

        if zp is not None and zp[0] is not None:

            self._taught_corner_z = zp[0] - self.controller.zero_position.get("Z", 0)

        logger.info(f"Taught corner: {self._taught_corner}")


    def _goto_corner(self):


        """v7.2.7: Navigate to taught corner — raw µm, no zero-ref."""


        if self._taught_corner:


            self.controller.move_xy_absolute(


                self._taught_corner[0], self._taught_corner[1],


                from_zero_ref=False,


            )


    def _calculate_alignment(self):
        """v7.2.7: alignment with µm — compute scale + rotation from taught points.

        taught_a1/corner are in absolute µm. Plate geometry is in mm.
        Convert taught positions to mm relative to A1 before comparing.
        """
        if not self._taught_a1 or not self._taught_corner or not self._plate:
            self.lbl_alignment.setText(
                "\u26a0 Record A1 and corner first, and select plate format.")
            return

        a1_expected = self._plate.get_well_position("A1")
        corner_expected = self._plate.get_well_position(self._corner_well)

        if a1_expected is None or corner_expected is None:
            self.lbl_alignment.setText(
                "\u26a0 Cannot compute alignment for this plate format.")
            return

        # Expected vector in mm (plate geometry)
        ex = corner_expected[0] - a1_expected[0]
        ey = corner_expected[1] - a1_expected[1]

        # Measured vector: taught positions are absolute µm, convert to mm
        mx_um = self._taught_corner[0] - self._taught_a1[0]
        my_um = self._taught_corner[1] - self._taught_a1[1]
        mx_mm = mx_um / 1000.0
        my_mm = my_um / 1000.0

        expected_dist = math.sqrt(ex**2 + ey**2)
        measured_dist = math.sqrt(mx_mm**2 + my_mm**2)

        if expected_dist < 0.001:
            return

        self._scale = measured_dist / expected_dist
        expected_angle = math.atan2(ey, ex)
        measured_angle = math.atan2(my_mm, mx_mm)
        self._rotation = math.degrees(measured_angle - expected_angle)

        # Offset: A1 position relative to zero reference, in µm
        self._offset_x = self._taught_a1[0] - self.controller.zero_position["x"]
        self._offset_y = self._taught_a1[1] - self.controller.zero_position["y"]

        self.lbl_alignment.setText(
            f"\u2705 Scale: {self._scale:.4f} | "
            f"Rot: {self._rotation:.2f}\u00b0 | "
            f"A1 off: ({self._offset_x:,.1f}, {self._offset_y:,.1f}) \u00b5m")
        self.lbl_alignment.setStyleSheet(f"color: {COLORS['green']};")

        if hasattr(self, 'ctx_lbl_cal_status'):
            self.ctx_lbl_cal_status.setText("\u2705 Calibrated")
            self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")


    def _goto_well(self):
        """v7.2.7: goto_well µm — navigate to computed well position.

        Uses plate geometry (mm) + taught A1 (µm) + scale/rotation
        to compute the absolute stage position for any well.
        """
        well = self.val_well_combo.currentText()
        if not well or not self._plate or not self._taught_a1:
            return

        try:
            pos = self._plate.get_well_position(well)
        except KeyError:
            return
        if pos is None:
            return

        a1_expected = self._plate.get_well_position("A1")
        if a1_expected is None:
            return

        # Vector from A1 to target well in plate mm
        dx_mm = pos[0] - a1_expected[0]
        dy_mm = pos[1] - a1_expected[1]

        # Apply rotation
        rad = math.radians(self._rotation)
        rx = dx_mm * math.cos(rad) - dy_mm * math.sin(rad)
        ry = dx_mm * math.sin(rad) + dy_mm * math.cos(rad)

        # Apply scale, convert to µm, add A1 absolute position
        target_x_um = self._taught_a1[0] + rx * self._scale * 1000.0
        target_y_um = self._taught_a1[1] + ry * self._scale * 1000.0

        # Safe travel if safe_z is available
        if hasattr(self, '_safe_z') and self._safe_z is not None:
            if hasattr(self, '_safe_navigate_to'):
                self._safe_navigate_to(target_x_um, target_y_um)
            else:
                self.controller.move_xy_absolute(
                    target_x_um, target_y_um, from_zero_ref=False)
        else:
            self.controller.move_xy_absolute(
                target_x_um, target_y_um, from_zero_ref=False)

        # Display
        rel_x = target_x_um - self.controller.zero_position["x"]
        rel_y = target_y_um - self.controller.zero_position["y"]
        if hasattr(self, 'lbl_val_result'):
            self.lbl_val_result.setText(
                f"Moving to {well} \u2192 ({rel_x:,.1f}, {rel_y:,.1f}) \u00b5m")


    def _save_calibration(self):
        """v7.2.7: save new fields — safe_z, top_z, Z plane, third point."""
        if self.settings is None:
            return
        cal_data = {
            "plate_format": self._plate.format if self._plate else None,
            "taught_a1": list(self._taught_a1) if self._taught_a1 else None,
            "taught_corner": list(self._taught_corner) if self._taught_corner else None,
            "offset_x": self._offset_x,
            "offset_y": self._offset_y,
            "rotation": self._rotation,
            "scale": self._scale,
            # v7.2.7 additions
            "safe_z": self._safe_z if hasattr(self, '_safe_z') else None,
            "top_z": self._top_z if hasattr(self, '_top_z') else None,
            "taught_a1_z": getattr(self, '_taught_a1_z', None),
            "taught_corner_z": getattr(self, '_taught_corner_z', None),
            "taught_third": list(self._taught_third) if getattr(self, '_taught_third', None) else None,
            "taught_third_z": getattr(self, '_taught_third_z', None),
            "third_well": getattr(self, '_third_well', None),
            "corner_well": getattr(self, '_corner_well', "H12"),
        }
        # Save Z plane coefficients if fitted
        zp = getattr(self, '_z_plane_result', None)
        if zp is not None:
            cal_data["z_plane"] = {
                "a": zp.a, "b": zp.b, "c": zp.c,
                "r_squared": zp.r_squared,
            }
        self.settings.set_section("calibration", cal_data)
        self.settings.save()
        logger.info("Calibration saved to settings (v7.2.7)")
        if hasattr(self, 'ctx_lbl_cal_status'):
            self.ctx_lbl_cal_status.setText("\u2705 Saved")
            self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")


    def _load_calibration(self):
        """v7.2.7: load new fields — safe_z, top_z, Z plane, third point."""
        if self.settings is None:
            return
        cal = self.settings.get_section("calibration")
        if not cal:
            return

        if cal.get("plate_format"):
            try:
                self._plate = WellPlate.from_format(cal["plate_format"])
                wells = self._plate.well_names
                if hasattr(self, 'val_well_combo'):
                    self.val_well_combo.clear()
                    self.val_well_combo.addItems(wells)
            except (ValueError, KeyError):
                pass


                # v7.2.7-hotfix: plate view on load
                if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:
                    self._cal_plate_view.set_plate(self._plate)
        if cal.get("taught_a1"):
            self._taught_a1 = tuple(cal["taught_a1"])
            # Display zero-ref position directly (positions are µm)
            zero_x = self.controller.zero_position.get('x', 0)
            zero_y = self.controller.zero_position.get('y', 0)
            ax = self._taught_a1[0] - zero_x
            ay = self._taught_a1[1] - zero_y
            if hasattr(self, 'lbl_a1'):
                z_str = ""
                if cal.get("taught_a1_z") is not None:
                    z_str = f"  Z: {cal['taught_a1_z']:.2f} mm"
                self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) \u00b5m{z_str}")
                self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

        if cal.get("taught_corner"):
            self._taught_corner = tuple(cal["taught_corner"])
            zero_x = self.controller.zero_position.get('x', 0)
            zero_y = self.controller.zero_position.get('y', 0)
            cx = self._taught_corner[0] - zero_x
            cy = self._taught_corner[1] - zero_y
            if hasattr(self, 'lbl_corner'):
                z_str = ""
                if cal.get("taught_corner_z") is not None:
                    z_str = f"  Z: {cal['taught_corner_z']:.2f} mm"
                self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) \u00b5m{z_str}")
                self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")

        self._offset_x = cal.get("offset_x", 0)
        self._offset_y = cal.get("offset_y", 0)
        self._rotation = cal.get("rotation", 0)
        self._scale = cal.get("scale", 1.0)

        # v7.2.7 fields
        if hasattr(self, '_safe_z'):
            self._safe_z = cal.get("safe_z")
            if self._safe_z is not None and hasattr(self, 'lbl_safe_z'):
                self.lbl_safe_z.setText(f"Safe Z: {self._safe_z:.2f} mm")
                self.lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")

        if hasattr(self, '_top_z'):
            self._top_z = cal.get("top_z")
            if self._top_z is not None and hasattr(self, 'lbl_top_z'):
                self.lbl_top_z.setText(f"Top Z: {self._top_z:.2f} mm")
                self.lbl_top_z.setStyleSheet(f"color: {COLORS['green']};")

        self._taught_a1_z = cal.get("taught_a1_z")
        self._taught_corner_z = cal.get("taught_corner_z")

        if cal.get("taught_third"):
            self._taught_third = tuple(cal["taught_third"])
            self._taught_third_z = cal.get("taught_third_z")
            self._third_well = cal.get("third_well")

        self._corner_well = cal.get("corner_well", getattr(self, '_corner_well', 'H12'))

        # Restore Z plane
        zp_data = cal.get("z_plane")
        if zp_data:
            try:
                from SupportClasses.WellSetup import PlaneResult
                self._z_plane_result = PlaneResult(
                    a=zp_data["a"], b=zp_data["b"], c=zp_data["c"],
                    r_squared=zp_data.get("r_squared", 0),
                    num_points=3,
                )
                if hasattr(self, 'lbl_zplane'):
                    self.lbl_zplane.setText(f"Z plane: {self._z_plane_result.describe()}")
                    self.lbl_zplane.setStyleSheet(f"color: {COLORS['green']};")
            except (ImportError, KeyError) as e:
                logger.debug(f"Could not restore Z plane: {e}")

        if self._scale != 1.0 or self._rotation != 0:
            if hasattr(self, 'lbl_alignment'):
                self.lbl_alignment.setText(
                    f"Loaded | Scale: {self._scale:.4f} | "
                    f"Rotation: {self._rotation:.2f}\u00b0")
            if hasattr(self, 'ctx_lbl_cal_status'):
                self.ctx_lbl_cal_status.setText("\u2705 Loaded from settings")
                self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")

        logger.info("Calibration loaded from settings (v7.2.7)")

    # ════════════════════════════════════════════════════════════════
    #  v7.3.0: WELL AUTO-DETECTION
    # ════════════════════════════════════════════════════════════════

    def _get_camera_config(self):
        """Get CameraConfig from hardware config, or None."""
        if self._hardware_config is None:
            return None
        return getattr(self._hardware_config, 'camera_config', None)

    def _get_expected_diameter_px(self) -> float | None:
        """Compute expected well diameter in pixels from plate geometry + camera config."""
        if self._plate is None:
            return None
        cam_cfg = self._get_camera_config()
        if cam_cfg is None or cam_cfg.micron_per_pixel is None:
            return None
        # well_diameter is in mm → convert to µm → convert to pixels
        diameter_um = self._plate.well_diameter * 1000.0
        return cam_cfg.um_to_pixel(diameter_um)

    def _get_primary_camera(self):
        """Get the first running camera widget, or first camera if none running."""
        for cam in self._cameras:
            if cam.is_running:
                return cam
        return self._cameras[0] if self._cameras else None

    def _ensure_detection_worker(self) -> bool:
        """Create or reconfigure the DetectionWorker. Returns True if ready."""
        if not VISION_AVAILABLE or DetectionWorker is None:
            return False

        cam = self._get_primary_camera()
        if cam is None:
            return False

        if self._detection_worker is None:
            self._detection_worker = DetectionWorker(
                camera_widget=cam, parent=self)
            self._detection_worker.well_detected.connect(
                self._on_well_detected)
            self._detection_worker.needle_detected.connect(
                self._on_needle_detected)
            self._detection_worker.focus_updated.connect(
                self._on_focus_updated)
            self._detection_worker.detection_cleared.connect(
                self._on_detection_cleared)
        else:
            self._detection_worker.set_camera_widget(cam)

        return True

    def _toggle_well_detect(self, target: str, enabled: bool) -> None:
        """Toggle well auto-detection for a target ('a1', 'corner')."""
        if not enabled:
            self._stop_detection()
            return

        # Uncheck the other detect button
        if target == "a1" and hasattr(self, '_btn_detect_corner'):
            self._btn_detect_corner.setChecked(False)
        elif target == "corner" and hasattr(self, '_btn_detect_a1'):
            self._btn_detect_a1.setChecked(False)

        # Check prerequisites
        diameter_px = self._get_expected_diameter_px()
        if diameter_px is None:
            QMessageBox.warning(
                self, "Cannot Auto-Detect",
                "Configure plate format and camera settings first.\n"
                "Camera config is set in Hardware Setup (Page 0).")
            # Uncheck the button
            btn = (self._btn_detect_a1 if target == "a1"
                   else self._btn_detect_corner)
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
            return

        cam = self._get_primary_camera()
        if cam is None or not cam.is_running:
            QMessageBox.warning(
                self, "Cannot Auto-Detect",
                "Start a camera feed first.")
            btn = (self._btn_detect_a1 if target == "a1"
                   else self._btn_detect_corner)
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
            return

        if not self._ensure_detection_worker():
            return

        self._autodetect_target = target
        self._last_well_result = None

        # Configure and start worker
        self._detection_worker.set_mode(
            DetectionMode.WELL_DETECT,
            expected_diameter_px=diameter_px,
        )

        # Set frame size on overlay
        overlay = cam.detection_overlay
        if overlay is not None:
            frame = cam.get_current_frame()
            if frame is not None:
                overlay.set_frame_size(frame.shape[1], frame.shape[0])
            overlay.set_info_text(
                f"Detecting well ({target.upper()})...")

        if not self._detection_worker.isRunning():
            self._detection_worker.start()

        self._update_detect_buttons(target, detecting=True)
        logger.info(f"Well auto-detect started for {target}, "
                    f"expected diameter: {diameter_px:.0f}px")

    def _shutdown_detection_worker(self) -> None:
        """Fully stop and join the detection thread (for cleanup/close)."""
        if self._detection_worker is not None:
            self._detection_worker.stop_detection()
            if self._detection_worker.isRunning():
                self._detection_worker.wait(2000)  # 2s timeout
            self._detection_worker = None

    def closeEvent(self, event):
        """Ensure DetectionWorker thread is stopped before destruction."""
        self._shutdown_detection_worker()
        super().closeEvent(event)

    def _stop_detection(self) -> None:
        """Stop the detection worker and clear overlays."""
        if self._detection_worker is not None:
            self._detection_worker.set_mode(DetectionMode.IDLE)

        self._autodetect_target = ""
        self._last_well_result = None
        self._last_needle_result = None
        self._needle_detecting = False
        self._focus_assisting = False

        # Clear overlays on all cameras
        for cam in self._cameras:
            overlay = cam.detection_overlay
            if overlay is not None:
                overlay.clear()

        # Reset well detect button states
        for attr in ('_btn_detect_a1', '_btn_detect_corner'):
            btn = getattr(self, attr, None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)

        # Reset needle/focus button states
        for attr in ('_btn_detect_needle', '_btn_focus_assist'):
            btn = getattr(self, attr, None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)

        # Disable focus assist button when needle detection stops
        btn_fa = getattr(self, '_btn_focus_assist', None)
        if btn_fa is not None:
            btn_fa.setEnabled(False)

        self._update_detect_buttons("", detecting=False)

    def _update_detect_buttons(self, target: str, detecting: bool) -> None:
        """Enable/disable Center and Accept buttons based on detection state."""
        for tgt, btn_center, btn_accept in [
            ("a1",
             getattr(self, '_btn_center_a1', None),
             getattr(self, '_btn_accept_a1', None)),
            ("corner",
             getattr(self, '_btn_center_corner', None),
             getattr(self, '_btn_accept_corner', None)),
        ]:
            if btn_center is not None:
                btn_center.setEnabled(detecting and tgt == target)
            if btn_accept is not None:
                btn_accept.setEnabled(detecting and tgt == target)

    def _on_well_detected(self, result) -> None:
        """Slot: DetectionWorker found a well. Update overlay and status."""
        self._last_well_result = result
        target = self._autodetect_target

        cam = self._get_primary_camera()
        if cam is None:
            return

        # Update overlay
        overlay = cam.detection_overlay
        if overlay is not None:
            overlay.set_well_detection(result)

            # Compute and display offset from frame center
            cam_cfg = self._get_camera_config()
            frame = cam.get_current_frame()
            if frame is not None and cam_cfg is not None and cam_cfg.micron_per_pixel:
                dx_um, dy_um = pixel_offset_to_stage_um(
                    result.center_px,
                    (frame.shape[1], frame.shape[0]),
                    cam_cfg.micron_per_pixel,
                )
                overlay.set_offset_text(
                    f"\u0394x: {dx_um:+.0f} \u00b5m, "
                    f"\u0394y: {dy_um:+.0f} \u00b5m")

        # Update status label
        lbl = getattr(self, f'_lbl_detect_{target}', None)
        if lbl is not None:
            conf_pct = result.confidence * 100
            lbl.setText(
                f"Well detected ({result.method}) — "
                f"confidence: {conf_pct:.0f}%  "
                f"r: {result.radius_px:.0f}px")
            color = COLORS['green'] if result.confidence >= 0.6 else COLORS['yellow']
            lbl.setStyleSheet(f"color: {color}; font-size: 9pt;")

        # Enable Center/Accept if confidence is reasonable
        has_result = result.confidence >= 0.3
        btn_center = getattr(self, f'_btn_center_{target}', None)
        btn_accept = getattr(self, f'_btn_accept_{target}', None)
        if btn_center is not None:
            btn_center.setEnabled(has_result)
        if btn_accept is not None:
            btn_accept.setEnabled(has_result)

    def _on_detection_cleared(self) -> None:
        """Slot: DetectionWorker found nothing this cycle."""
        target = self._autodetect_target

        cam = self._get_primary_camera()
        overlay = cam.detection_overlay if cam is not None else None

        # Handle well detection clearing
        if target in ("a1", "corner"):
            self._last_well_result = None
            if overlay is not None:
                overlay.set_well_detection(None)
                overlay.set_offset_text("")

            lbl = getattr(self, f'_lbl_detect_{target}', None)
            if lbl is not None:
                lbl.setText("Searching for well...")
                lbl.setStyleSheet(
                    f"color: {COLORS['overlay0']}; font-size: 9pt;")

            # Disable Center/Accept when no detection
            btn_center = getattr(self, f'_btn_center_{target}', None)
            btn_accept = getattr(self, f'_btn_accept_{target}', None)
            if btn_center is not None:
                btn_center.setEnabled(False)
            if btn_accept is not None:
                btn_accept.setEnabled(False)

        # Handle needle detection clearing
        if self._needle_detecting:
            self._last_needle_result = None
            if overlay is not None:
                overlay.set_needle_detection(None)

            lbl = getattr(self, '_lbl_needle_status', None)
            if lbl is not None:
                lbl.setText("Searching for needle tip...")
                lbl.setStyleSheet(
                    f"color: {COLORS['overlay0']}; font-size: 9pt;")

    def _center_on_detection(self, target: str) -> None:
        """Move stage so the detected well center aligns with camera center."""
        result = self._last_well_result
        if result is None:
            return

        cam = self._get_primary_camera()
        cam_cfg = self._get_camera_config()
        if cam is None or cam_cfg is None or cam_cfg.micron_per_pixel is None:
            return

        frame = cam.get_current_frame()
        if frame is None:
            return

        dx_um, dy_um = pixel_offset_to_stage_um(
            result.center_px,
            (frame.shape[1], frame.shape[0]),
            cam_cfg.micron_per_pixel,
        )

        offset_mag = math.sqrt(dx_um ** 2 + dy_um ** 2)
        logger.info(f"Centering on well: dx={dx_um:.1f}µm, dy={dy_um:.1f}µm "
                    f"(magnitude: {offset_mag:.1f}µm)")

        if offset_mag < 5.0:
            lbl = getattr(self, f'_lbl_detect_{target}', None)
            if lbl is not None:
                lbl.setText(f"Already centered (offset: {offset_mag:.1f} \u00b5m)")
            return

        # Get current position and move relative
        xy = self.controller.get_xy_position(cached=False)
        if xy[0] is None:
            return

        # Move by the detected offset (µm → absolute position)
        # Note: sign convention may need calibration per microscope mounting
        new_x = xy[0] + dx_um
        new_y = xy[1] + dy_um
        self.controller.move_xy_absolute(new_x, new_y, from_zero_ref=False)

        lbl = getattr(self, f'_lbl_detect_{target}', None)
        if lbl is not None:
            lbl.setText(f"Moved {offset_mag:.0f} \u00b5m to center well")

    def _accept_detection(self, target: str) -> None:
        """Accept the current detection as the taught position."""
        # Stop detection
        self._stop_detection()

        # Record position using the existing record methods
        if target == "a1":
            self._record_a1_xyz()
            logger.info("Auto-detection accepted for A1")
        elif target == "corner":
            self._record_corner_xyz()
            logger.info("Auto-detection accepted for corner")

    # ════════════════════════════════════════════════════════════════
    #  v7.3.0 Phase 6: NEEDLE AUTO-DETECTION & FOCUS ASSIST
    # ════════════════════════════════════════════════════════════════

    def _get_expected_od_px(self) -> float | None:
        """Compute expected needle OD in pixels from HardwareConfig + CameraConfig."""
        if self._hardware_config is None:
            return None
        needle = getattr(self._hardware_config, 'needle', None)
        if needle is None:
            return None
        cam_cfg = self._get_camera_config()
        if cam_cfg is None or cam_cfg.micron_per_pixel is None:
            return None
        return cam_cfg.um_to_pixel(needle.od_um)

    def _toggle_needle_detect(self, enabled: bool) -> None:
        """Toggle needle tip auto-detection."""
        if not enabled:
            self._stop_detection()
            return

        # Stop any active well detection
        for attr in ('_btn_detect_a1', '_btn_detect_corner'):
            btn = getattr(self, attr, None)
            if btn is not None and btn.isChecked():
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)

        # Check prerequisites: needle spec + camera config
        od_px = self._get_expected_od_px()
        if od_px is None:
            QMessageBox.warning(
                self, "Cannot Detect Needle",
                "Configure needle gauge and camera settings first.\n"
                "Needle is set in Hardware Setup (Page 0).")
            btn = getattr(self, '_btn_detect_needle', None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)
            return

        cam = self._get_primary_camera()
        if cam is None or not cam.is_running:
            QMessageBox.warning(
                self, "Cannot Detect Needle",
                "Start a camera feed first.")
            btn = getattr(self, '_btn_detect_needle', None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)
            return

        if not self._ensure_detection_worker():
            return

        self._needle_detecting = True
        self._last_needle_result = None

        # Configure and start worker in NEEDLE_DETECT mode
        self._detection_worker.set_mode(
            DetectionMode.NEEDLE_DETECT,
            expected_od_px=od_px,
        )

        # Set frame size on overlay
        overlay = cam.detection_overlay
        if overlay is not None:
            frame = cam.get_current_frame()
            if frame is not None:
                overlay.set_frame_size(frame.shape[1], frame.shape[0])
            overlay.set_info_text("Detecting needle...")

        if not self._detection_worker.isRunning():
            self._detection_worker.start()

        # Enable focus assist button
        btn_fa = getattr(self, '_btn_focus_assist', None)
        if btn_fa is not None:
            btn_fa.setEnabled(True)

        lbl = getattr(self, '_lbl_needle_status', None)
        if lbl is not None:
            lbl.setText("Searching for needle tip...")
            lbl.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")

        logger.info(f"Needle detection started, expected OD: {od_px:.0f}px")

    def _toggle_focus_assist(self, enabled: bool) -> None:
        """Toggle focus assist mode (real-time focus quality bar)."""
        if not enabled:
            # If needle detection is still active, switch back to it
            if self._needle_detecting:
                od_px = self._get_expected_od_px()
                if od_px is not None and self._detection_worker is not None:
                    self._detection_worker.set_mode(
                        DetectionMode.NEEDLE_DETECT,
                        expected_od_px=od_px,
                    )
            else:
                self._stop_detection()
            self._focus_assisting = False
            lbl = getattr(self, '_lbl_focus_status', None)
            if lbl is not None:
                lbl.setText("")
            # Clear focus overlay
            cam = self._get_primary_camera()
            if cam is not None:
                overlay = cam.detection_overlay
                if overlay is not None:
                    overlay.set_focus_result(None)
            return

        if not self._ensure_detection_worker():
            return

        self._focus_assisting = True
        self._best_focus_z = None
        self._best_focus_score = 0.0

        cam = self._get_primary_camera()
        if cam is not None:
            overlay = cam.detection_overlay
            if overlay is not None:
                frame = cam.get_current_frame()
                if frame is not None:
                    overlay.set_frame_size(frame.shape[1], frame.shape[0])
                overlay.set_info_text("Focus Assist — adjust Z")

        self._detection_worker.set_mode(DetectionMode.FOCUS_ASSIST)

        if not self._detection_worker.isRunning():
            self._detection_worker.start()

        lbl = getattr(self, '_lbl_focus_status', None)
        if lbl is not None:
            lbl.setText("Adjust Z axis — watching focus quality...")
            lbl.setStyleSheet(f"color: {COLORS['blue']}; font-size: 9pt;")

        logger.info("Focus assist started")

    def _on_needle_detected(self, result) -> None:
        """Slot: DetectionWorker found a needle tip."""
        if not self._needle_detecting:
            return

        self._last_needle_result = result

        cam = self._get_primary_camera()
        if cam is None:
            return

        # Update overlay
        overlay = cam.detection_overlay
        if overlay is not None:
            overlay.set_needle_detection(result)

        # Update status label
        lbl = getattr(self, '_lbl_needle_status', None)
        if lbl is not None:
            conf_pct = result.confidence * 100
            od_px = result.diameter_px

            # Show expected vs detected OD
            expected_od_px = self._get_expected_od_px()
            if expected_od_px is not None:
                cam_cfg = self._get_camera_config()
                if cam_cfg is not None and cam_cfg.micron_per_pixel:
                    detected_od_um = cam_cfg.pixel_to_um(od_px)
                    expected_od_um = cam_cfg.pixel_to_um(expected_od_px)
                    lbl.setText(
                        f"Needle detected — confidence: {conf_pct:.0f}%  "
                        f"OD: {detected_od_um:.0f} \u00b5m "
                        f"(expected: {expected_od_um:.0f} \u00b5m)")
                else:
                    lbl.setText(
                        f"Needle detected — confidence: {conf_pct:.0f}%  "
                        f"OD: {od_px:.0f}px")
            else:
                lbl.setText(
                    f"Needle detected — confidence: {conf_pct:.0f}%  "
                    f"OD: {od_px:.0f}px")

            color = COLORS['green'] if result.confidence >= 0.6 else COLORS['yellow']
            lbl.setStyleSheet(f"color: {color}; font-size: 9pt;")

    def _on_focus_updated(self, result) -> None:
        """Slot: DetectionWorker computed a new focus score."""
        if not self._focus_assisting:
            return

        cam = self._get_primary_camera()
        if cam is None:
            return

        # Update overlay focus bar
        overlay = cam.detection_overlay
        if overlay is not None:
            overlay.set_focus_result(result)

        # Track best focus Z
        zp = self.controller.get_zp_position(cached=True)
        current_z = None
        if isinstance(zp, (list, tuple)) and len(zp) >= 1 and zp[0] is not None:
            current_z = zp[0] - self.controller.zero_position.get("Z", 0)

        if result.normalized_score > self._best_focus_score:
            self._best_focus_score = result.normalized_score
            if current_z is not None:
                self._best_focus_z = current_z

        # Get focus trend from worker's tracker
        trend = "unknown"
        if (self._detection_worker is not None
                and self._detection_worker.focus_tracker is not None):
            trend = self._detection_worker.focus_tracker.get_trend()

        # Update focus status label with directional hint
        lbl = getattr(self, '_lbl_focus_status', None)
        if lbl is not None:
            score_pct = result.normalized_score * 100

            if trend == "improving":
                hint = "Getting sharper — keep going"
                color = COLORS['green']
            elif trend == "declining":
                hint = "Getting blurrier — reverse Z direction"
                color = COLORS['red']
            elif trend == "stable" and result.is_in_focus:
                hint = "In focus!"
                color = COLORS['green']
            elif trend == "stable":
                hint = "Stable — try moving Z"
                color = COLORS['yellow']
            else:
                hint = "Adjust Z axis..."
                color = COLORS['blue']

            lbl.setText(f"Focus: {score_pct:.0f}%  — {hint}")
            lbl.setStyleSheet(f"color: {color}; font-size: 9pt;")

        # Update best focus label
        lbl_best = getattr(self, '_lbl_best_focus', None)
        if lbl_best is not None and self._best_focus_z is not None:
            lbl_best.setText(
                f"Best focus at Z = {self._best_focus_z:.3f} mm "
                f"({self._best_focus_score * 100:.0f}%)")
            lbl_best.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")

