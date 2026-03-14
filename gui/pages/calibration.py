"""
Calibration Page — Multi-camera layout with calibration wizard.

v7.3.2: Layout restructured.
    Context panel (left box): Calibration wizard + plate position view + plate config.
    Main content: Position readout, camera feeds (configurable count).
    Per-camera settings (brightness, gamma, FPS) via CameraWidget's built-in settings panel.
    Step 1 "Zero Needle" removed — use jog page Set Zero instead.
    Steps renumbered: Safe Z (1A), Top Z (1B), Auto-Calibrate (1C), Z-Cal (2).
    Camera feed count selectable (1/2/3).

Camera feeds support software brightness and gamma adjustment which works
regardless of webcam hardware capabilities.
"""

from __future__ import annotations

import math
import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox,
    QFrame, QSizePolicy, QMessageBox, QCheckBox,
    QScrollArea,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from PySide6.QtGui import QPainter, QPen, QBrush, QColor

from SupportClasses.StageController import StageController
from SupportClasses.WellPlate import WellPlate, PLATE_DEFINITIONS
from gui.styles import COLORS, SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE
from gui.unit_helpers import stage_to_um, format_um, DEFAULT_XY_POSITION_SCALE
from gui.scaling import s, scaled_font_size

try:
    from SupportClasses.HardwareConfig import HardwareConfig, CameraConfig
except ImportError:
    HardwareConfig = None
    CameraConfig = None

# v7.3.4: Objective calibration persistence
try:
    from SupportClasses.ObjectiveCalibration import get_store as _get_obj_store
    OBJECTIVE_CAL_AVAILABLE = True
except ImportError:
    OBJECTIVE_CAL_AVAILABLE = False
    def _get_obj_store():
        return None

# v7.3.4: Empirical µm/px calibration dialog
try:
    from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
    PIXEL_CAL_DIALOG_AVAILABLE = True
except ImportError:
    PIXEL_CAL_DIALOG_AVAILABLE = False
    PixelCalibrationDialog = None

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
    # v7.3.1: Double-click signal for per-well scanning
    try:
        from PySide6.QtCore import Signal as _Sig
        well_clicked = _Sig(str)
        well_double_clicked = _Sig(str)
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
        self._predicted_positions: dict[str, tuple[float, float]] | None = None  # v7.3.1
        self._calibrated_positions: dict[str, tuple[float, float]] | None = None  # v7.3.1
        self._needle_xy    = None
        self._needle_item  = None
        self._well_composites: dict[str, tuple] = {}  # well_name → (bgr, diameter_mm)
        self._well_composite_items: dict = {}  # well_name → QGraphicsPixmapItem
        self._calibration_wells: set[str] = set()  # v7.3.1: wells used for 3-point calibration
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumHeight(s(130))

    def set_plate(self, plate):
        self._plate = plate
        self._rebuild()

    def set_taught_a1(self, pt):
        self._taught_a1 = pt

    def set_taught_corner(self, pt, corner_well):
        self._taught_corner = pt
        self._corner_well   = corner_well

    def set_predicted_positions(self, positions: dict[str, tuple[float, float]] | None):
        """v7.3.1: Store geometry-predicted positions and refresh view."""
        self._predicted_positions = positions
        self._rebuild()

    def set_calibrated_positions(self, positions: dict[str, tuple[float, float]] | None):
        """v7.3.1: Store mosaic-calibrated positions and refresh view."""
        self._calibrated_positions = positions
        self._rebuild()

    def set_calibration_wells(self, well_names: set):
        """v7.3.1: Mark wells used for 3-point auto-calibration (highlighted on plate view)."""
        self._calibration_wells = well_names
        self._rebuild()

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
                QBrush(QColor("#45475a")),
            ).setToolTip(well.name)

        # v7.3.1: Show predicted-position dots (yellow) for geometry-predicted wells
        if self._predicted_positions and self._taught_a1 is not None:
            a1_x_mm, a1_y_mm = self._taught_a1  # plate view coords are mm relative to A1
            pred_pen = QPen(QColor("#f9e2af"), 0.5)
            pred_brush = QBrush(QColor("#f9e2af80"))
            pr = 1.8  # small dot radius
            for wname, (px_um, py_um) in self._predicted_positions.items():
                # Convert absolute µm to mm-relative-to-A1 for plate view
                rx_mm = (px_um / 1000.0) - a1_x_mm
                ry_mm = (py_um / 1000.0) - a1_y_mm
                dot = self._scene.addEllipse(
                    rx_mm * S - pr, ry_mm * S - pr, 2 * pr, 2 * pr,
                    pred_pen, pred_brush,
                )
                dot.setToolTip(f"{wname} (predicted)")
                dot.setZValue(5)

        # v7.3.1: Show calibrated-position dots (green) — override predicted dots
        if self._calibrated_positions and self._taught_a1 is not None:
            a1_x_mm, a1_y_mm = self._taught_a1
            cal_pen = QPen(QColor("#a6e3a1"), 0.8)
            cal_brush = QBrush(QColor("#a6e3a1a0"))
            cr = 2.0
            for wname, (cx_um, cy_um) in self._calibrated_positions.items():
                rx_mm = (cx_um / 1000.0) - a1_x_mm
                ry_mm = (cy_um / 1000.0) - a1_y_mm
                dot = self._scene.addEllipse(
                    rx_mm * S - cr, ry_mm * S - cr, 2 * cr, 2 * cr,
                    cal_pen, cal_brush,
                )
                dot.setToolTip(f"{wname} (calibrated)")
                dot.setZValue(6)

        # Highlight A1, corner well, and calibration reference wells
        wmap = {w.name: w for w in wells}
        highlighted = set()
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
                highlighted.add(wname)
        # v7.3.1: Highlight 3-well calibration reference wells
        for wname in self._calibration_wells:
            if wname in highlighted or wname not in wmap:
                continue
            w = wmap[wname]
            cx, cy = w.x * S, w.y * S
            r2 = self.WELL_R + 1.5
            self._scene.addEllipse(
                cx - r2, cy - r2, 2 * r2, 2 * r2,
                QPen(QColor("#89b4fa"), 1.5),
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

        # v7.3.1: Re-apply per-well composite overlays after rebuild
        self._well_composite_items.clear()
        self._reapply_well_composites()

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

    def mouseDoubleClickEvent(self, event):
        """v7.3.1: Double-click a well to trigger per-well scan."""
        if self._plate is None:
            super().mouseDoubleClickEvent(event)
            return
        scene_pos = self.mapToScene(event.pos())
        S = self.SCALE
        best_well = None
        best_dist = float('inf')
        for well in self._plate.get_all_wells():
            cx, cy = well.x * S, well.y * S
            dx = scene_pos.x() - cx
            dy = scene_pos.y() - cy
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < self.WELL_R + 2 and dist < best_dist:
                best_dist = dist
                best_well = well.name
        if best_well:
            try:
                self.well_double_clicked.emit(best_well)
            except AttributeError:
                pass
            logger.debug(f"Plate view double-clicked: {best_well}")
        else:
            super().mouseDoubleClickEvent(event)

    # ── v7.3.1: Per-well composite display ────────────────────────

    def set_well_composite(self, well_name: str, composite_bgr,
                           well_diameter_mm: float):
        """Display a per-well mosaic composite overlaying the well circle.

        Args:
            well_name: Well to overlay (e.g. "B3").
            composite_bgr: BGR uint8 numpy array from per-well scan.
            well_diameter_mm: Well diameter in mm (used for sizing).
        """
        import numpy as np
        self._well_composites[well_name] = (composite_bgr, well_diameter_mm)
        self._apply_well_composite(well_name)

    def _apply_well_composite(self, well_name: str):
        """Create/update a QGraphicsPixmapItem for a per-well composite."""
        try:
            import cv2
            from PySide6.QtGui import QImage, QPixmap
            from PySide6.QtWidgets import QGraphicsPixmapItem
        except ImportError:
            return

        if well_name not in self._well_composites:
            return
        if self._plate is None:
            return

        composite_bgr, diameter_mm = self._well_composites[well_name]
        if composite_bgr is None:
            return

        S = self.SCALE
        wmap = {w.name: w for w in self._plate.get_all_wells()}
        well = wmap.get(well_name)
        if well is None:
            return

        # Scene size for this well
        well_size = int(diameter_mm * S)
        if well_size < 2:
            return

        # Resize composite to fit the well circle on the plate view
        resized = cv2.resize(composite_bgr, (well_size, well_size),
                             interpolation=cv2.INTER_AREA)
        resized_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        qimg = QImage(resized_rgb.data, well_size, well_size,
                      well_size * 3, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg.copy())

        # Position at well center in scene coordinates
        cx = well.x * S - well_size / 2.0
        cy = well.y * S - well_size / 2.0

        # Reuse existing item or create new one
        if well_name in self._well_composite_items:
            item = self._well_composite_items[well_name]
            item.setPixmap(pixmap)
            item.setPos(cx, cy)
        else:
            item = self._scene.addPixmap(pixmap)
            item.setPos(cx, cy)
            item.setZValue(2)  # above well circles (0), below predicted dots (5)
            item.setOpacity(0.90)
            self._well_composite_items[well_name] = item

    def _reapply_well_composites(self):
        """Re-create all per-well composite pixmaps after scene rebuild."""
        self._well_composite_items.clear()
        for well_name in self._well_composites:
            self._apply_well_composite(well_name)

    # ── v7.3.1: Progressive mosaic display ────────────────────────

    def update_scan_composite(self, composite_bgr, canvas_extent_um,
                              a1_x_um: float, a1_y_um: float):
        """Replace the mosaic overlay with the latest composite image.

        Called during raster scan to progressively display the growing stitched
        mosaic. A single pixmap item is reused and repositioned each update.

        Args:
            composite_bgr: Full composite image (BGR uint8 from MosaicBuilder).
            canvas_extent_um: (min_x, min_y, max_x, max_y) world extent of the
                             composite canvas in µm (includes FOV padding).
            a1_x_um: A1 X position in absolute µm (coordinate reference).
            a1_y_um: A1 Y position in absolute µm (coordinate reference).
        """
        try:
            import cv2
            from PySide6.QtGui import QImage, QPixmap

            if composite_bgr is None:
                return

            S = self.SCALE
            min_x, min_y, max_x, max_y = canvas_extent_um

            # Canvas world extent in mm (relative to A1)
            canvas_w_mm = (max_x - min_x) / 1000.0
            canvas_h_mm = (max_y - min_y) / 1000.0

            # Top-left of canvas relative to A1 in mm
            rel_min_x_mm = (min_x - a1_x_um) / 1000.0
            rel_min_y_mm = (min_y - a1_y_um) / 1000.0

            # Scene coordinates matching the canvas world extent
            sx = rel_min_x_mm * S
            sy = rel_min_y_mm * S
            sw = max(1, int(canvas_w_mm * S))
            sh = max(1, int(canvas_h_mm * S))

            # Resize composite to scene dimensions
            ch, cw = composite_bgr.shape[:2]
            if cw < 1 or ch < 1:
                return

            resized = cv2.resize(composite_bgr, (sw, sh), interpolation=cv2.INTER_AREA)
            resized_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

            qimg = QImage(resized_rgb.data, sw, sh,
                          sw * 3, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg.copy())

            # Reuse existing pixmap item or create new one
            if not hasattr(self, '_scan_composite_item') or self._scan_composite_item is None:
                self._scan_composite_item = self._scene.addPixmap(pixmap)
                self._scan_composite_item.setZValue(1)
                self._scan_composite_item.setOpacity(0.85)
            else:
                self._scan_composite_item.setPixmap(pixmap)

            self._scan_composite_item.setPos(sx, sy)
        except Exception as e:
            logger.debug(f"Failed to update scan composite: {e}")

    def clear_scan_frames(self):
        """Remove all scan frame overlays from the plate view."""
        self._scan_composite_item = None
        # Rebuild removes everything and re-adds wells/dots
        self._rebuild()

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
        self.setFixedWidth(s(90))
        self.setMinimumHeight(s(130))

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

    # v7.3.1: Emitted when calibration data changes (safe_z, positions, plate)
    calibration_data_changed = Signal()
    # v7.3.3: Emitted when µm/px is calibrated via needle detection (cam_idx, value)
    um_per_px_calibrated = Signal(int, float)

    def __init__(self, controller: StageController, settings=None,
                 camera_manager=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        self._camera_manager = camera_manager  # v7.3.3: shared CameraManager

        # Calibration state
        self._plate: WellPlate | None = None
        self._taught_a1: tuple[float, float] | None = None
        self._taught_corner: tuple[float, float] | None = None
        self._corner_well: str = "H12"
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._rotation = 0.0
        self._scale = 1.0

        # Camera widgets — from CameraManager if available, else empty
        if camera_manager is not None:
            self._cameras = camera_manager.cameras
            camera_manager.cameras_detected.connect(self._refresh_source_combos)
        else:
            self._cameras: list = []
        self._context_widget = None

        # v7.1.1: Microsteps-to-microns conversion factor
        self._xy_position_scale = DEFAULT_XY_POSITION_SCALE
        self._hardware_config = None  # v7.2: HardwareConfig

        # v7.3.1: Geometry-predicted positions (well_name → (x_um, y_um) absolute)
        self._predicted_positions: dict[str, tuple[float, float]] | None = None
        # v7.3.1: Calibrated positions after mosaic scan (affine-corrected)
        self._calibrated_positions: dict[str, tuple[float, float]] | None = None
        # v7.3.1: Mosaic scan state
        self._mosaic_builder = None   # MosaicBuilder instance during scan
        self._scan_positions: list[tuple[float, float]] = []  # Raster scan positions (µm)
        self._scan_index: int = 0         # Current well index during scan
        self._scan_timer = None           # QTimer for scan ticks
        self._scanning: bool = False
        # v7.3.1: Z-offset teaching state
        self._z_teach_points: dict[str, float] = {}  # well_name → Z offset (mm, zero-ref)
        # v7.3.2: Manual XY teaching state
        self._xy_teach_points: dict[str, tuple[float, float]] = {}  # well_name → (x_um, y_um) absolute
        self._edge_left: tuple[float, float] | None = None   # left edge (x_um, y_um)
        self._edge_right: tuple[float, float] | None = None  # right edge (x_um, y_um)

        # v7.3.0: Auto-detection state
        self._detection_worker: DetectionWorker | None = None
        self._last_well_result = None       # Latest DetectionResult from worker
        self._autodetect_target: str = ""   # "a1", "corner", "third"

        # v7.3.0 Phase 6: Needle detection + focus assist state
        self._last_needle_result = None     # Latest needle DetectionResult
        self._needle_detecting: bool = False
        self._needle_relaxed_mode: bool = False   # v7.3.3: relaxed detect active
        self._needle_interactive: bool = False     # v7.3.3: in accept/reject mode
        self._focus_assisting: bool = False
        self._best_focus_z: float | None = None       # Best-focus Z (mm, zero-ref)
        self._best_focus_score: float = 0.0            # Score at best Z

        # v7.3.1: Auto Z-calibration state
        self._auto_z_scanning: bool = False
        self._auto_z_phase: str = "idle"
        self._auto_z_wells: list[str] = []
        self._auto_z_well_idx: int = 0
        self._auto_z_results: dict[str, float] = {}  # well_name → Z bottom (mm, zero-ref)
        self._auto_z_current_z: float = 0.0
        self._auto_z_best_z: float | None = None
        self._auto_z_best_score: float = 0.0
        self._auto_z_baseline_score: float = 0.0
        self._auto_z_decline_count: int = 0
        self._auto_z_timer = None
        self._auto_z_coarse_step: float = 0.15  # mm
        self._auto_z_fine_step: float = 0.03    # mm
        self._auto_z_sweep_end: float = 0.0

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

    def get_calibration_data(self):
        """v7.3.1: Return (plate, well_positions, safe_z) for jog page fast-travel.

        Returns calibrated positions if available, otherwise predicted positions.
        """
        positions = self._calibrated_positions or self._predicted_positions
        return self._plate, positions, getattr(self, '_safe_z', None)

    def _emit_calibration_data_changed(self):
        """v7.3.1: Notify listeners that calibration data has changed."""
        self.calibration_data_changed.emit()

    def set_xy_position_scale(self, value: float):
        """Update the XY position scale factor."""
        self._xy_position_scale = value

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

        # v7.3.4: Sync objective selector and µm/px display for all camera slots
        # Use the objective stored in camera_config if available
        cam_cfg = getattr(config, 'camera_config', None)
        obj_combos = getattr(self, '_ctx_cam_obj_combos', [])
        if cam_cfg is not None and obj_combos:
            # Map nominal magnification back to objective name
            nom_mag = getattr(cam_cfg, 'objective_magnification', None)
            if nom_mag is not None and OBJECTIVE_CAL_AVAILABLE:
                store = _get_obj_store()
                matched_name = None
                if store:
                    for obj in store.objectives:
                        if abs(obj["nominal_magnification"] - nom_mag) < 0.01:
                            matched_name = obj["name"]
                            break
                if matched_name:
                    for combo in obj_combos:
                        idx_in_combo = combo.findText(matched_name)
                        if idx_in_combo >= 0:
                            combo.blockSignals(True)
                            combo.setCurrentIndex(idx_in_combo)
                            combo.blockSignals(False)
        for cam_idx in range(len(obj_combos)):
            self._ctx_update_um_px_display(cam_idx)



    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL  (Plate Config + Plate View + Wizard Steps)
    # ════════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        """Build the context panel: camera controls + plate config + wizard."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(5)

        # ── Camera Controls (collapsible) ────────────────────────
        cam_header_row = QHBoxLayout()
        self._ctx_cam_toggle = QPushButton("▾ Camera Controls")
        self._ctx_cam_toggle.setObjectName("flatBtn")
        self._ctx_cam_toggle.setStyleSheet(
            f"text-align: left; font-weight: 600; color: {COLORS['blue']}; "
            f"padding: 6px 0px 2px 0px; border: none; background: transparent;"
            f"border-bottom: 1px solid {COLORS['surface1']}; margin-bottom: 4px;")
        self._ctx_cam_toggle.setCursor(Qt.PointingHandCursor)
        self._ctx_cam_toggle.clicked.connect(self._toggle_cam_section)
        cam_header_row.addWidget(self._ctx_cam_toggle, stretch=1)
        layout.addLayout(cam_header_row)

        self._ctx_cam_container = QWidget()
        cam_lay = QVBoxLayout(self._ctx_cam_container)
        cam_lay.setContentsMargins(0, 0, 0, 4)
        cam_lay.setSpacing(4)

        # Row 1: Refresh sources (detection is done in Hardware Setup)
        refresh_row = QHBoxLayout()
        self._btn_refresh_sources = QPushButton("Refresh Sources")
        self._btn_refresh_sources.setMinimumWidth(s(110))
        self._btn_refresh_sources.setToolTip(
            "Refresh camera sources (detect in Hardware Setup)")
        self._btn_refresh_sources.clicked.connect(self._refresh_source_combos)
        refresh_row.addWidget(self._btn_refresh_sources)
        self._lbl_cam_count = QLabel("0 sources")
        self._lbl_cam_count.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        refresh_row.addWidget(self._lbl_cam_count)
        refresh_row.addStretch()
        cam_lay.addLayout(refresh_row)

        # Row 2: Tile arrangement + click-to-move toggle
        opts_row = QHBoxLayout()
        opts_row.addWidget(QLabel("Layout:"))
        self._cam_tile_combo = QComboBox()
        self._cam_tile_combo.addItem("Single", "single")
        self._cam_tile_combo.addItem("Horizontal", "horizontal")
        self._cam_tile_combo.addItem("Vertical", "vertical")
        self._cam_tile_combo.addItem("Square (2x2)", "square")
        self._cam_tile_combo.setMaximumWidth(s(110))
        self._cam_tile_combo.currentIndexChanged.connect(self._on_tile_changed)
        opts_row.addWidget(self._cam_tile_combo)
        opts_row.addStretch()
        self._btn_click_to_move = QPushButton("Click→Move: OFF")
        self._btn_click_to_move.setCheckable(True)
        self._btn_click_to_move.setChecked(False)
        self._btn_click_to_move.setMaximumWidth(s(130))
        self._btn_click_to_move.setToolTip(
            "Click on the camera feed to move the XY stage so that\n"
            "the clicked point is centred under the camera.")
        self._btn_click_to_move.toggled.connect(self._on_click_to_move_toggled)
        opts_row.addWidget(self._btn_click_to_move)
        cam_lay.addLayout(opts_row)

        # Per-camera control rows
        self._ctx_cam_rows: list[QWidget] = []
        self._ctx_cam_src_combos: list[QComboBox] = []
        self._ctx_cam_start_btns: list[QPushButton] = []
        self._ctx_cam_settings_panels: list[QFrame] = []
        self._ctx_cam_checkboxes: list[QCheckBox] = []
        # v7.3.4: Per-camera µm/px calibration widgets
        self._ctx_cam_obj_combos: list[QComboBox] = []
        self._ctx_cam_theo_labels: list[QLabel] = []
        self._ctx_cam_cal_labels: list[QLabel] = []
        self._ctx_cam_fov_labels: list[QLabel] = []
        for i in range(MAX_CAMERAS):
            row_frame = QFrame()
            row_frame.setStyleSheet(
                f"QFrame {{ border: 1px solid {COLORS['surface1']}; "
                f"border-radius: 4px; padding: 2px; }}")
            row_lay = QVBoxLayout(row_frame)
            row_lay.setContentsMargins(4, 3, 4, 3)
            row_lay.setSpacing(2)

            # Top: checkbox + label + source combo
            top_row = QHBoxLayout()
            chk = QCheckBox()
            chk.setChecked(i == 0)  # First camera checked by default
            chk.setToolTip(f"Show Camera {i+1} in display")
            chk.toggled.connect(lambda checked, idx=i: self._on_cam_checkbox_toggled(idx, checked))
            top_row.addWidget(chk)
            self._ctx_cam_checkboxes.append(chk)
            top_row.addWidget(QLabel(f"<b>Cam {i+1}</b>"))
            src_combo = QComboBox()
            src_combo.setMinimumWidth(s(80))
            src_combo.setToolTip(f"Camera {i+1} source")
            top_row.addWidget(src_combo, stretch=1)
            self._ctx_cam_src_combos.append(src_combo)
            row_lay.addLayout(top_row)

            # Bottom: buttons
            btn_row = QHBoxLayout()
            btn_row.setSpacing(3)
            btn_start = QPushButton("Start")
            btn_start.setMinimumWidth(s(50))
            btn_start.clicked.connect(lambda checked=False, idx=i: self._ctx_toggle_cam(idx))
            btn_row.addWidget(btn_start)
            self._ctx_cam_start_btns.append(btn_start)

            btn_snap = QPushButton("Snap")
            btn_snap.setMinimumWidth(s(42))
            btn_snap.clicked.connect(lambda checked=False, idx=i: self._ctx_snap_cam(idx))
            btn_row.addWidget(btn_snap)

            btn_xhair = QCheckBox("Crosshair")
            btn_xhair.setChecked(True)
            btn_xhair.toggled.connect(lambda checked, idx=i: self._ctx_crosshair_cam(idx, checked))
            btn_row.addWidget(btn_xhair)

            btn_settings = QPushButton("Settings")
            btn_settings.setCheckable(True)
            btn_settings.setMinimumWidth(s(56))
            btn_settings.toggled.connect(lambda checked, idx=i: self._ctx_toggle_settings(idx, checked))
            btn_row.addWidget(btn_settings)
            row_lay.addLayout(btn_row)

            # Collapsible settings panel (brightness, gamma, FPS)
            settings_panel = QFrame()
            settings_panel.setVisible(False)
            sp_lay = QVBoxLayout(settings_panel)
            sp_lay.setContentsMargins(4, 2, 4, 2)
            sp_lay.setSpacing(2)

            from PySide6.QtWidgets import QSlider, QSpinBox
            bri_row = QHBoxLayout()
            bri_row.addWidget(QLabel("Bri:"))
            sld_bri = QSlider(Qt.Horizontal)
            sld_bri.setRange(-100, 100)
            sld_bri.setValue(0)
            sld_bri.valueChanged.connect(lambda v, idx=i: self._ctx_set_brightness(idx, v))
            bri_row.addWidget(sld_bri)
            sp_lay.addLayout(bri_row)

            gam_row = QHBoxLayout()
            gam_row.addWidget(QLabel("Gam:"))
            sld_gam = QSlider(Qt.Horizontal)
            sld_gam.setRange(10, 300)
            sld_gam.setValue(100)
            sld_gam.valueChanged.connect(lambda v, idx=i: self._ctx_set_gamma(idx, v))
            gam_row.addWidget(sld_gam)
            sp_lay.addLayout(gam_row)

            fps_row = QHBoxLayout()
            fps_row.addWidget(QLabel("FPS:"))
            spn_fps = QSpinBox()
            spn_fps.setRange(1, 60)
            spn_fps.setValue(15)
            spn_fps.valueChanged.connect(lambda v, idx=i: self._ctx_set_fps(idx, v))
            fps_row.addWidget(spn_fps)
            fps_row.addStretch()
            sp_lay.addLayout(fps_row)

            # v7.3.4: Objective & µm/px calibration section
            sep = QFrame()
            sep.setFrameShape(QFrame.HLine)
            sep.setStyleSheet(f"color: {COLORS['surface1']}; margin: 2px 0;")
            sp_lay.addWidget(sep)

            obj_hdr = QLabel("Objective & Scale")
            obj_hdr.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-weight: bold; font-size: 8pt;")
            sp_lay.addWidget(obj_hdr)

            obj_row = QHBoxLayout()
            obj_row.addWidget(QLabel("Obj:"))
            obj_combo = QComboBox()
            obj_combo.setMinimumWidth(s(70))
            obj_combo.setToolTip("Select the installed objective lens")
            # Populate with standard objectives (or whatever the store has)
            if OBJECTIVE_CAL_AVAILABLE:
                store = _get_obj_store()
                for name in store.objective_names():
                    obj_combo.addItem(name)
            else:
                for name in ["1x", "2x", "4x", "10x", "20x", "40x", "100x"]:
                    obj_combo.addItem(name)
            obj_combo.currentIndexChanged.connect(
                lambda _idx, cam=i: self._ctx_on_objective_changed(cam))
            obj_row.addWidget(obj_combo, stretch=1)
            sp_lay.addLayout(obj_row)
            self._ctx_cam_obj_combos.append(obj_combo)

            theo_lbl = QLabel("Theoretical: —")
            theo_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 8pt;")
            theo_lbl.setWordWrap(True)
            sp_lay.addWidget(theo_lbl)
            self._ctx_cam_theo_labels.append(theo_lbl)

            cal_lbl = QLabel("Calibrated: —")
            cal_lbl.setStyleSheet(
                f"color: {COLORS['overlay0']}; font-size: 8pt;")
            cal_lbl.setWordWrap(True)
            sp_lay.addWidget(cal_lbl)
            self._ctx_cam_cal_labels.append(cal_lbl)

            fov_lbl = QLabel("FOV: —")
            fov_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 8pt;")
            fov_lbl.setWordWrap(True)
            sp_lay.addWidget(fov_lbl)
            self._ctx_cam_fov_labels.append(fov_lbl)

            btn_cal_umpx = QPushButton("Calibrate µm/px")
            btn_cal_umpx.setToolTip(
                "Empirically measure µm/px via stage motion + phase correlation")
            btn_cal_umpx.setStyleSheet(
                f"QPushButton {{ background-color: {COLORS['surface1']}; "
                f"color: {COLORS['text']}; padding: 3px 8px; "
                f"border-radius: 3px; font-size: 8pt; }}"
                f"QPushButton:hover {{ background-color: {COLORS['blue']}; "
                f"color: {COLORS['base']}; }}")
            btn_cal_umpx.clicked.connect(
                lambda checked=False, cam=i: self._ctx_calibrate_um_per_px(cam))
            sp_lay.addWidget(btn_cal_umpx)

            row_lay.addWidget(settings_panel)
            self._ctx_cam_settings_panels.append(settings_panel)

            cam_lay.addWidget(row_frame)
            self._ctx_cam_rows.append(row_frame)

        layout.addWidget(self._ctx_cam_container)

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

        # v7.3.2: Plate position view in context panel
        self._build_cal_views(layout)

        # v7.3.2: Calibration wizard steps in context panel
        self._build_wizard_steps(layout)

        layout.addStretch()

        self._context_widget = ctx

        # Now that all widgets exist, load any saved calibration
        self._load_calibration()

        return ctx

    def _get_max_slots(self) -> int:
        """Return max display slots for the current tile mode."""
        mode = getattr(self, '_cam_tile_mode', 'single')
        if mode == "square":
            return 4
        elif mode in ("horizontal", "vertical"):
            return MAX_CAMERAS
        else:  # single
            return 1

    def _get_checked_camera_indices(self) -> list[int]:
        """Return indices of cameras whose checkboxes are checked."""
        checkboxes = getattr(self, '_ctx_cam_checkboxes', [])
        return [i for i, chk in enumerate(checkboxes) if chk.isChecked()]

    def _on_cam_checkbox_toggled(self, idx: int, checked: bool):
        """v7.3.3: Toggle camera visibility via checkbox, enforce slot limit."""
        checkboxes = getattr(self, '_ctx_cam_checkboxes', [])
        if not checkboxes:
            return
        max_slots = self._get_max_slots()
        checked_indices = self._get_checked_camera_indices()

        if checked and len(checked_indices) > max_slots:
            # Too many checked — uncheck oldest checked cameras (not the one just toggled)
            others = [i for i in checked_indices if i != idx]
            while len(others) + 1 > max_slots and others:
                old = others.pop(0)
                checkboxes[old].blockSignals(True)
                checkboxes[old].setChecked(False)
                checkboxes[old].blockSignals(False)

        self._relayout_cameras()

    def _on_tile_changed(self, index: int):
        """v7.3.3: Change camera tile arrangement."""
        tile = getattr(self, '_cam_tile_combo', None)
        if tile is None:
            return
        self._cam_tile_mode = tile.currentData() or "single"
        max_slots = self._get_max_slots()
        # If too many cameras checked for new mode, uncheck extras
        checkboxes = getattr(self, '_ctx_cam_checkboxes', [])
        checked_indices = self._get_checked_camera_indices()
        while len(checked_indices) > max_slots:
            # Uncheck the last checked camera
            last = checked_indices.pop()
            checkboxes[last].blockSignals(True)
            checkboxes[last].setChecked(False)
            checkboxes[last].blockSignals(False)
        self._relayout_cameras()

    def _relayout_cameras(self):
        """v7.3.3: Re-arrange camera widgets in the grid layout using checkboxes."""
        grid = getattr(self, '_cam_grid_layout', None)
        if grid is None:
            return
        mode = getattr(self, '_cam_tile_mode', 'single')
        checked = self._get_checked_camera_indices()

        # Remove all widgets from grid without setParent(None)
        # (setParent(None) kills the display of running cameras)
        while grid.count():
            item = grid.takeAt(0)
            w = item.widget()
            if w:
                w.hide()
                grid.removeWidget(w)

        # Determine grid positions based on tile mode
        if mode == "square":
            slots = 4
            positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        elif mode == "vertical":
            slots = max(len(checked), 1)
            positions = [(i, 0) for i in range(slots)]
        elif mode == "horizontal":
            slots = max(len(checked), 1)
            positions = [(0, i) for i in range(slots)]
        else:  # single
            slots = 1
            positions = [(0, 0)]

        # Place checked camera feed views into slots; fill remaining with placeholders
        feed_views = getattr(self, '_cam_feed_views', [])
        for slot_idx, (r, c) in enumerate(positions):
            if slot_idx < len(checked):
                cam_idx = checked[slot_idx]
                if cam_idx < len(feed_views):
                    fv = feed_views[cam_idx]
                    fv.setVisible(True)
                    grid.addWidget(fv, r, c)
                    continue
            # Empty slot — show placeholder
            placeholders = getattr(self, '_cam_placeholders', [])
            if slot_idx < len(placeholders):
                ph = placeholders[slot_idx]
                ph.setVisible(True)
                grid.addWidget(ph, r, c)

        # Hide all unchecked feed views
        for i in range(len(feed_views)):
            if i not in checked:
                feed_views[i].setVisible(False)

        # Clear ALL old stretch factors first (max possible is 2 rows x 2 cols)
        for r in range(max(grid.rowCount(), 2)):
            grid.setRowStretch(r, 0)
        for c in range(max(grid.columnCount(), 2)):
            grid.setColumnStretch(c, 0)

        # Set equal stretch only for rows/cols actually in use
        max_r = max(r for r, c in positions) + 1 if positions else 1
        max_c = max(c for r, c in positions) + 1 if positions else 1
        for r in range(max_r):
            grid.setRowStretch(r, 1)
        for c in range(max_c):
            grid.setColumnStretch(c, 1)

    def _toggle_cam_section(self):
        """v7.3.3: Collapse/expand camera controls section."""
        container = getattr(self, '_ctx_cam_container', None)
        toggle = getattr(self, '_ctx_cam_toggle', None)
        if container is None or toggle is None:
            return
        visible = container.isVisible()
        container.setVisible(not visible)
        prefix = "▸" if visible else "▾"
        toggle.setText(f"{prefix} Camera Controls")

    def _on_click_to_move_toggled(self, checked: bool):
        """Toggle click-to-move mode on/off."""
        self._click_to_move_enabled = checked
        btn = getattr(self, '_btn_click_to_move', None)
        if btn:
            btn.setText("Click→Move: ON" if checked else "Click→Move: OFF")
            btn.setStyleSheet(
                f"background-color: {COLORS.get('green', '#a6e3a1')}; color: #1e1e2e;"
                if checked else ""
            )

    def _on_feed_clicked(self, cam_idx: int, px_x: float, px_y: float):
        """Move XY stage so the clicked image point centres under the camera."""
        if not getattr(self, '_click_to_move_enabled', False):
            return
        if not self.controller or not self.controller.xy_stage:
            return
        mgr = getattr(self, '_camera_manager', None)
        if mgr is None:
            return
        feed_views = getattr(self, '_cam_feed_views', [])
        if cam_idx >= len(feed_views):
            return
        img_w, img_h = feed_views[cam_idx].image_size
        if img_w == 0 or img_h == 0:
            return
        dx_um, dy_um = mgr.pixel_to_stage_offset(cam_idx, px_x, px_y, img_w, img_h)
        logger.debug(
            f"[CalibPage] click-to-move cam={cam_idx} px=({px_x:.1f},{px_y:.1f}) "
            f"→ dx={dx_um:.1f} dy={dy_um:.1f} µm"
        )
        self.controller.move_xy_relative_um(dx_um, dy_um)

    def _refresh_source_combos(self):
        """v7.3.3: Refresh source combos from CameraManager (no detection)."""
        mgr = self._camera_manager
        if mgr is None:
            return

        src_items = mgr.available_sources
        num = len(src_items)

        # Sync context panel source combos
        for ctx_combo in getattr(self, '_ctx_cam_src_combos', []):
            prev_data = ctx_combo.currentData()
            ctx_combo.blockSignals(True)
            ctx_combo.clear()
            ctx_combo.addItem("— None —", None)
            for text, data in src_items:
                ctx_combo.addItem(text, data)
            if prev_data:
                idx = ctx_combo.findData(prev_data)
                if idx >= 0:
                    ctx_combo.setCurrentIndex(idx)
            ctx_combo.blockSignals(False)

        lbl = getattr(self, '_lbl_cam_count', None)
        if lbl:
            lbl.setText(f"{num} sources")

    # ── Context panel camera button callbacks ─────────────────

    def _ctx_toggle_cam(self, idx: int):
        """Start/stop camera from context panel."""
        if idx >= len(self._cameras):
            return
        mgr = self._camera_manager

        # Set camera source from context combo before starting
        ctx_combos = getattr(self, '_ctx_cam_src_combos', [])
        if idx < len(ctx_combos) and mgr:
            src_data = ctx_combos[idx].currentData()
            if src_data:
                mgr.set_source(idx, src_data)

        if mgr:
            mgr.toggle(idx)
        else:
            self._cameras[idx].toggle()
        running = getattr(self._cameras[idx], '_running', False)
        btn = self._ctx_cam_start_btns[idx]
        btn.setText("Stop" if running else "Start")

        # Auto-check the checkbox and relayout so camera appears in the live view
        checkboxes = getattr(self, '_ctx_cam_checkboxes', [])
        if running and idx < len(checkboxes) and not checkboxes[idx].isChecked():
            checkboxes[idx].setChecked(True)  # triggers _on_cam_checkbox_toggled → _relayout
        else:
            # Checkbox was already checked — force relayout so the widget refreshes
            self._relayout_cameras()

    def _ctx_snap_cam(self, idx: int):
        """Take snapshot from context panel."""
        if idx < len(self._cameras):
            self._cameras[idx].take_snapshot()

    def _ctx_crosshair_cam(self, idx: int, checked: bool):
        """Toggle crosshair from context panel."""
        if idx < len(self._cameras):
            self._cameras[idx]._show_crosshair = checked

    def _ctx_toggle_settings(self, idx: int, checked: bool):
        """Show/hide per-camera settings in context panel."""
        panels = getattr(self, '_ctx_cam_settings_panels', [])
        if idx < len(panels):
            panels[idx].setVisible(checked)

    def _ctx_set_brightness(self, idx: int, value: int):
        """Set camera brightness from context panel slider."""
        if idx < len(self._cameras):
            self._cameras[idx].set_brightness(value)

    def _ctx_set_gamma(self, idx: int, value: int):
        """Set camera gamma from context panel slider (10-300 → 0.1-3.0)."""
        if idx < len(self._cameras):
            self._cameras[idx].set_gamma(value / 100.0)

    def _ctx_set_fps(self, idx: int, value: int):
        """Set camera FPS from context panel spinner."""
        if idx < len(self._cameras):
            cam = self._cameras[idx]
            cam._fps = value
            if cam._running:
                cam._timer.setInterval(int(1000 / max(1, value)))

    # ════════════════════════════════════════════════════════════════
    #  v7.3.4: OBJECTIVE & µm/px CALIBRATION HELPERS
    # ════════════════════════════════════════════════════════════════

    def _get_camera_spec_for_idx(self, idx: int):
        """Return CameraSpec from hardware config for a camera slot, or None.

        Currently all camera slots share the same hardware config camera_spec
        (multi-camera configs are a future enhancement).
        """
        hw = getattr(self, '_hardware_config', None)
        if hw is None:
            return None
        cam_cfg = getattr(hw, 'camera_config', None)
        if cam_cfg is None:
            return None
        return getattr(cam_cfg, 'camera_spec', None)

    def _get_camera_model_for_idx(self, idx: int) -> str:
        """Return a camera model name string suitable as an objectives.json key.

        Tries to extract the model ID from the camera spec name
        (e.g. "Bestscope BUC3D-1000C (ToupTek...)" → "BUC3D-1000C").
        Falls back to "unknown" when no spec is available.
        """
        spec = self._get_camera_spec_for_idx(idx)
        if spec is None:
            return "unknown"
        # Prefer the first parenthetical token or the first word-run with digits
        name: str = spec.name
        # e.g. "Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)"
        # Extract the second word if the first looks like a brand
        parts = name.split()
        for part in parts:
            if any(ch.isdigit() for ch in part):
                return part.rstrip("()")
        return parts[0] if parts else "unknown"

    def _ctx_on_objective_changed(self, idx: int) -> None:
        """Called when the objective combo selection changes."""
        self._ctx_update_um_px_display(idx)

    def _ctx_update_um_px_display(self, idx: int) -> None:
        """Recompute and refresh the µm/px labels for camera slot idx.

        Reads:
          - The selected objective name from the combo
          - The camera spec from the hardware config (sensor_pixel_size_um,
            max_resolution, active_resolution)
          - Any stored calibration from ObjectiveCalibrationStore

        Writes:
          - Theoretical label (from sensor spec + nominal magnification)
          - Calibrated label (from objectives.json, green if available)
          - FOV label
          - Propagates the best available value to CameraManager
        """
        obj_combos = getattr(self, '_ctx_cam_obj_combos', [])
        theo_labels = getattr(self, '_ctx_cam_theo_labels', [])
        cal_labels = getattr(self, '_ctx_cam_cal_labels', [])
        fov_labels = getattr(self, '_ctx_cam_fov_labels', [])

        if idx >= len(obj_combos):
            return

        obj_name = obj_combos[idx].currentText() if obj_combos[idx].count() else ""
        spec = self._get_camera_spec_for_idx(idx)
        hw = getattr(self, '_hardware_config', None)
        cam_cfg = getattr(hw, 'camera_config', None) if hw else None

        # Theoretical µm/px
        theoretical_um_per_px: float | None = None
        if spec is not None and OBJECTIVE_CAL_AVAILABLE:
            store = _get_obj_store()
            nom_mag = store.nominal_magnification(obj_name) if store else None
            if nom_mag and nom_mag > 0:
                active_res = (
                    cam_cfg.active_resolution if cam_cfg else (916, 686)
                )
                effective_px = spec.effective_pixel_size_um(active_res)
                theoretical_um_per_px = effective_px / nom_mag
        elif spec is not None:
            # Fallback: try to get magnification from hardware config
            nom_mag = (
                cam_cfg.objective_magnification if cam_cfg else 1.0
            )
            if nom_mag and nom_mag > 0:
                active_res = (
                    cam_cfg.active_resolution if cam_cfg else (916, 686)
                )
                effective_px = spec.effective_pixel_size_um(active_res)
                theoretical_um_per_px = effective_px / nom_mag

        if idx < len(theo_labels):
            if theoretical_um_per_px is not None:
                theo_labels[idx].setText(
                    f"Theoretical: {theoretical_um_per_px:.3f} µm/px")
            else:
                theo_labels[idx].setText("Theoretical: — (no camera spec)")

        # Calibrated µm/px from objectives.json
        calibrated_um_per_px: float | None = None
        if OBJECTIVE_CAL_AVAILABLE and obj_name:
            store = _get_obj_store()
            camera_model = self._get_camera_model_for_idx(idx)
            cal = store.get_calibration(camera_model, obj_name) if store else None
            if cal:
                calibrated_um_per_px = cal.get("measured_um_per_px")
                cal_date = cal.get("date", "")
                cal_res = cal.get("resolution", [])
                if idx < len(cal_labels):
                    cal_labels[idx].setText(
                        f"Calibrated: {calibrated_um_per_px:.3f} µm/px"
                        f"{' (' + str(cal_res[0]) + 'x' + str(cal_res[1]) + ')' if cal_res else ''}"
                        f"{' [' + cal_date + ']' if cal_date else ''}")
                    cal_labels[idx].setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 8pt;")
            else:
                if idx < len(cal_labels):
                    cal_labels[idx].setText("Calibrated: — (not yet calibrated)")
                    cal_labels[idx].setStyleSheet(
                        f"color: {COLORS['overlay0']}; font-size: 8pt;")

        # Active µm/px: prefer calibrated over theoretical
        active_um_per_px = calibrated_um_per_px or theoretical_um_per_px

        # FOV label
        if idx < len(fov_labels):
            if active_um_per_px and cam_cfg:
                w, h = cam_cfg.active_resolution
                fov_w = w * active_um_per_px / 1000.0  # mm
                fov_h = h * active_um_per_px / 1000.0
                fov_labels[idx].setText(
                    f"FOV: {fov_w:.2f} × {fov_h:.2f} mm")
            else:
                fov_labels[idx].setText("FOV: —")

        # Push to CameraManager
        mgr = getattr(self, '_camera_manager', None)
        if mgr is not None and active_um_per_px is not None:
            mgr.set_um_per_px(idx, active_um_per_px)
            logger.debug(
                f"Camera {idx}: applied {active_um_per_px:.4f} µm/px "
                f"(obj={obj_name}, calibrated={calibrated_um_per_px is not None})")

        # Also update hardware config objective magnification if we have a
        # nominal mag for the selected objective
        if cam_cfg is not None and OBJECTIVE_CAL_AVAILABLE and obj_name:
            store = _get_obj_store()
            nom_mag = store.nominal_magnification(obj_name) if store else None
            if nom_mag is not None:
                cam_cfg.objective_magnification = nom_mag

    def _ctx_calibrate_um_per_px(self, idx: int) -> None:
        """Open the empirical µm/px calibration dialog for camera slot idx.

        On accept:
          - Saves the measured value to objectives.json keyed by camera model
            and currently selected objective name.
          - Writes the value into hardware_config.camera_config for immediate
            downstream use (clears override when switching objectives; see
            _ctx_update_um_px_display).
          - Refreshes the µm/px display labels.
          - Emits um_per_px_calibrated to update the hardware setup page.
        """
        if not PIXEL_CAL_DIALOG_AVAILABLE or PixelCalibrationDialog is None:
            QMessageBox.warning(self, "Unavailable",
                                "Pixel calibration dialog is not available.")
            return

        mgr = getattr(self, '_camera_manager', None)
        if mgr is None or not mgr.is_running(idx):
            QMessageBox.warning(self, "Camera Required",
                                "Start Camera %d before calibrating." % (idx + 1))
            return

        if self.controller is None:
            QMessageBox.warning(self, "Stage Required",
                                "Stage controller must be connected for calibration.")
            return

        dlg = PixelCalibrationDialog(mgr, self.controller, cam_idx=idx,
                                     parent=self)
        if not dlg.exec():
            return

        measured = dlg.result_um_per_px
        if measured is None:
            return

        # Persist to objectives.json
        if OBJECTIVE_CAL_AVAILABLE:
            store = _get_obj_store()
            obj_combos = getattr(self, '_ctx_cam_obj_combos', [])
            obj_name = (obj_combos[idx].currentText()
                        if idx < len(obj_combos) and obj_combos[idx].count()
                        else "unknown")
            camera_model = self._get_camera_model_for_idx(idx)
            hw = getattr(self, '_hardware_config', None)
            cam_cfg = getattr(hw, 'camera_config', None) if hw else None
            active_res = (
                cam_cfg.active_resolution if cam_cfg else (916, 686)
            )
            if store is not None:
                store.set_calibration(camera_model, obj_name,
                                      measured, active_res)

        # Write into hardware config for immediate use by downstream code
        hw = getattr(self, '_hardware_config', None)
        if hw is not None:
            cam_cfg = getattr(hw, 'camera_config', None)
            if cam_cfg is not None:
                cam_cfg.micron_per_pixel_override = measured

        # Refresh labels and push to CameraManager
        self._ctx_update_um_px_display(idx)

        # Emit so hardware setup page can also update its readout
        self.um_per_px_calibrated.emit(idx, measured)
        logger.info(
            f"Camera {idx}: calibration accepted — {measured:.4f} µm/px")

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT UI  (position readout + camera feeds)
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        _bg = COLORS['base']
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(f"QScrollArea {{ background-color: {_bg}; border: none; }}")
        outer.addWidget(scroll)

        container = QWidget()
        container.setStyleSheet(f"background-color: {_bg};")
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 8, 12, 8)
        scroll.setWidget(container)

        mono = QFont("Consolas", scaled_font_size(12))

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

        # ── Camera Feeds (grid layout, controls in context panel) ───
        cam_card = QFrame()
        cam_card.setObjectName("cardFrame")
        cam_card_layout = QVBoxLayout(cam_card)
        cam_card_layout.setSpacing(0)
        cam_card_layout.setContentsMargins(4, 4, 4, 4)

        self._cam_grid_widget = QWidget()
        self._cam_grid_layout = QGridLayout(self._cam_grid_widget)
        self._cam_grid_layout.setSpacing(4)
        self._cam_grid_layout.setContentsMargins(0, 0, 0, 0)

        if self._camera_manager and self._camera_manager.is_available:
            from gui.widgets.camera_feed_view import CameraFeedView

            # Create feed views — lightweight displays subscribed to CameraWidgets
            self._cam_feed_views: list[CameraFeedView] = []
            self._click_to_move_enabled = False
            for i in range(MAX_CAMERAS):
                fv = CameraFeedView(
                    camera_manager=self._camera_manager,
                    cam_idx=i,
                    show_crosshair=True,
                    label=f"Camera {i + 1}",
                    parent=self,
                )
                # Connect click signal for click-to-move
                fv.clicked.connect(
                    lambda px, py, idx=i: self._on_feed_clicked(idx, px, py)
                )
                self._cam_feed_views.append(fv)

            # Placeholder labels for empty slots (square mode)
            self._cam_placeholders: list[QLabel] = []
            for i in range(MAX_CAMERAS):
                ph = QLabel(f"Camera {i + 1}")
                ph.setAlignment(Qt.AlignCenter)
                ph.setStyleSheet(
                    f"background-color: #181825; border: 1px solid {COLORS['surface1']}; "
                    f"color: {COLORS['text']}; font-size: 12pt; font-weight: 600;")
                ph.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                ph.setMinimumSize(s(200), s(150))
                self._cam_placeholders.append(ph)

            # Initial layout: single camera, first checked
            self._cam_tile_mode = "single"
            self._relayout_cameras()
        else:
            self._cam_feed_views = []
            self._cam_placeholders = []
            self._click_to_move_enabled = False
            no_cam = QLabel(
                "Camera unavailable\n"
                "Install: pip install opencv-python"
            )
            no_cam.setAlignment(Qt.AlignCenter)
            no_cam.setStyleSheet(f"color: {COLORS['overlay0']};")
            self._cam_grid_layout.addWidget(no_cam, 0, 0)

        cam_card_layout.addWidget(self._cam_grid_widget, stretch=1)
        layout.addWidget(cam_card, stretch=1)

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
            self._btn_detect_cameras.setText("Detecting...")

        def _on_found(indices):
            from PySide6.QtCore import QTimer
            def _apply():
                self._available_cameras = indices
                if hasattr(self, '_btn_detect_cameras'):
                    self._btn_detect_cameras.setText("Detect Cameras")
                    self._btn_detect_cameras.setEnabled(True)
                lbl = getattr(self, '_lbl_cam_count', None)
                if lbl:
                    lbl.setText(f"{len(indices)} found")
                # Checkboxes manage visibility now; no combo to refresh
                logger.info(f"Camera detection: {indices}")
            QTimer.singleShot(0, _apply)

        detect_cameras_async(_on_found, max_index=4)



    # ── Plate + YZ views (v7.3.1-calviews) ──────────────────────

    def _build_cal_views(self, parent_layout) -> None:
        """Add plate-view + YZ-view to context panel."""
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
        # v7.3.1: Connect double-click for per-well scanning
        if hasattr(self._cal_plate_view, 'well_double_clicked'):
            self._cal_plate_view.well_double_clicked.connect(self._start_well_scan)

        grp_layout.addWidget(self._cal_plate_view, stretch=1)

        self._cal_yz_view = _CalibrationYZView()
        grp_layout.addWidget(self._cal_yz_view)

        parent_layout.addWidget(grp)

    def _build_wizard_steps(self, parent_layout) -> None:
        """v7.3.2: Build calibration wizard steps in context panel.

        Step 1 (Zero Needle) removed — use jog page Set Zero instead.
        """
        wiz_frame = QFrame()
        wiz_frame.setObjectName("cardFrame")
        layout = QVBoxLayout(wiz_frame)
        layout.setSpacing(5)
        layout.setContentsMargins(8, 6, 8, 6)

        wiz_title = QLabel("Calibration Wizard")
        wiz_title.setStyleSheet(SECTION_TITLE_STYLE)
        layout.addWidget(wiz_title)

        # ── Step 1A — Safe Z ─────────────────────────────
        s1a_label = QLabel("Step 1A \u2014 Safe Z")
        s1a_label.setObjectName("contextSectionLabel")
        layout.addWidget(s1a_label)
        layout.addWidget(QLabel("Raise needle to safe travel height, then set."))
        s1a_row = QHBoxLayout()
        btn_safe_z = QPushButton("Set Safe Z")
        btn_safe_z.setObjectName("successBtn")
        btn_safe_z.setMaximumHeight(s(26))
        btn_safe_z.clicked.connect(self._set_safe_z)
        s1a_row.addWidget(btn_safe_z)
        self.lbl_safe_z = QLabel("Not set")
        self.lbl_safe_z.setStyleSheet(f"color: {COLORS['yellow']};")
        s1a_row.addWidget(self.lbl_safe_z, stretch=1)
        layout.addLayout(s1a_row)

        # ── Step 1B — Top Z ──────────────────────────────
        s1b_label = QLabel("Step 1B \u2014 Top Z (Plate Surface)")
        s1b_label.setObjectName("contextSectionLabel")
        layout.addWidget(s1b_label)
        layout.addWidget(QLabel("Lower needle to plate top surface, then set."))
        s1b_row = QHBoxLayout()
        btn_top_z = QPushButton("Set Top Z")
        btn_top_z.setMaximumHeight(s(26))
        btn_top_z.clicked.connect(self._set_top_z)
        s1b_row.addWidget(btn_top_z)
        self.lbl_top_z = QLabel("Not set")
        self.lbl_top_z.setStyleSheet(f"color: {COLORS['yellow']};")
        s1b_row.addWidget(self.lbl_top_z, stretch=1)
        layout.addLayout(s1b_row)

        # ── Step 1C — Auto-Calibrate (3-Well) ────────────
        s1c_label = QLabel("Step 1C \u2014 Auto-Calibrate (3-Well)")
        s1c_label.setObjectName("contextSectionLabel")
        layout.addWidget(s1c_label)
        layout.addWidget(QLabel("Auto-detect 3 wells to calibrate plate position and orientation."))

        scan_btn_row = QHBoxLayout()
        self._btn_start_scan = QPushButton("Auto-Calibrate")
        self._btn_start_scan.setObjectName("successBtn")
        self._btn_start_scan.setMaximumHeight(s(26))
        self._btn_start_scan.setToolTip(
            "Move to 3 reference wells, auto-detect each, fit affine correction")
        self._btn_start_scan.clicked.connect(self._start_plate_scan)
        scan_btn_row.addWidget(self._btn_start_scan)

        self._btn_cancel_scan = QPushButton("Cancel")
        self._btn_cancel_scan.setMaximumHeight(s(26))
        self._btn_cancel_scan.setMaximumWidth(s(60))
        self._btn_cancel_scan.setEnabled(False)
        self._btn_cancel_scan.clicked.connect(self._cancel_plate_scan)
        scan_btn_row.addWidget(self._btn_cancel_scan)

        self._btn_accept_scan = QPushButton("Accept")
        self._btn_accept_scan.setMaximumHeight(s(26))
        self._btn_accept_scan.setMaximumWidth(s(60))
        self._btn_accept_scan.setEnabled(False)
        self._btn_accept_scan.clicked.connect(self._accept_plate_scan)
        scan_btn_row.addWidget(self._btn_accept_scan)
        layout.addLayout(scan_btn_row)

        self._lbl_scan_progress = QLabel("")
        self._lbl_scan_progress.setWordWrap(True)
        self._lbl_scan_progress.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(self._lbl_scan_progress)

        self._lbl_scan_result = QLabel("")
        self._lbl_scan_result.setWordWrap(True)
        self._lbl_scan_result.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(self._lbl_scan_result)

        # Fast-travel buttons for calibration wells
        self._cal_travel_frame = QWidget()
        cal_travel_layout = QHBoxLayout(self._cal_travel_frame)
        cal_travel_layout.setContentsMargins(0, 0, 0, 0)
        cal_travel_layout.addWidget(QLabel("Go to:"))
        self._cal_travel_btns: list[QPushButton] = []
        for i in range(3):
            btn = QPushButton("\u2014")
            btn.setMaximumHeight(s(24))
            btn.setMaximumWidth(s(60))
            btn.setEnabled(False)
            btn.clicked.connect(lambda checked, idx=i: self._goto_calibration_well(idx))
            cal_travel_layout.addWidget(btn)
            self._cal_travel_btns.append(btn)
        cal_travel_layout.addStretch()
        self._cal_travel_frame.setVisible(False)
        layout.addWidget(self._cal_travel_frame)

        # ── Step 2 — Z-Bottom Calibration ─────────────────
        s2_label = QLabel("Step 2 \u2014 Z-Bottom Calibration")
        s2_label.setObjectName("contextSectionLabel")
        layout.addWidget(s2_label)
        layout.addWidget(QLabel(
            "Find well bottom Z by lowering needle and tracking focus peak."))

        depth_row = QHBoxLayout()
        depth_row.addWidget(QLabel("Well depth:"))
        self._well_depth_spin = QDoubleSpinBox()
        self._well_depth_spin.setRange(1.0, 50.0)
        self._well_depth_spin.setDecimals(2)
        self._well_depth_spin.setSuffix(" mm")
        self._well_depth_spin.setValue(17.4)
        self._well_depth_spin.setMaximumWidth(s(100))
        self._well_depth_spin.setToolTip(
            "Distance from plate top surface to well bottom glass")
        depth_row.addWidget(self._well_depth_spin)
        depth_row.addStretch()
        layout.addLayout(depth_row)

        auto_z_row = QHBoxLayout()
        self._btn_auto_z_cal = QPushButton("Auto Z-Cal")
        self._btn_auto_z_cal.setObjectName("successBtn")
        self._btn_auto_z_cal.setMaximumHeight(s(26))
        self._btn_auto_z_cal.setToolTip(
            "Automatically find Z bottom at 3 calibration wells using focus search")
        self._btn_auto_z_cal.clicked.connect(self._start_auto_z_cal)
        auto_z_row.addWidget(self._btn_auto_z_cal)
        self._btn_cancel_auto_z = QPushButton("Cancel")
        self._btn_cancel_auto_z.setMaximumHeight(s(26))
        self._btn_cancel_auto_z.setMaximumWidth(s(60))
        self._btn_cancel_auto_z.setEnabled(False)
        self._btn_cancel_auto_z.clicked.connect(self._cancel_auto_z)
        auto_z_row.addWidget(self._btn_cancel_auto_z)
        layout.addLayout(auto_z_row)

        self._lbl_auto_z_progress = QLabel("")
        self._lbl_auto_z_progress.setWordWrap(True)
        self._lbl_auto_z_progress.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(self._lbl_auto_z_progress)

        # ── Manual Teaching ─────────────────────────────────
        manual_label = QLabel("Manual Teaching")
        manual_label.setObjectName("contextSectionLabel")
        layout.addWidget(manual_label)

        # Well selector + navigation
        teach_nav_row = QHBoxLayout()
        teach_nav_row.addWidget(QLabel("Well:"))
        self._zteach_well_combo = QComboBox()
        self._zteach_well_combo.setMaximumWidth(s(70))
        teach_nav_row.addWidget(self._zteach_well_combo)
        btn_zteach_go = QPushButton("Go \u25b6")
        btn_zteach_go.setMaximumHeight(s(24))
        btn_zteach_go.setMaximumWidth(s(40))
        btn_zteach_go.setToolTip("Safe-travel to selected well")
        btn_zteach_go.clicked.connect(self._zteach_goto_well)
        teach_nav_row.addWidget(btn_zteach_go)
        btn_zteach_next = QPushButton("Next")
        btn_zteach_next.setMaximumHeight(s(24))
        btn_zteach_next.setMaximumWidth(s(40))
        btn_zteach_next.setToolTip("Advance to next well and navigate")
        btn_zteach_next.clicked.connect(self._zteach_next_well)
        teach_nav_row.addWidget(btn_zteach_next)
        layout.addLayout(teach_nav_row)

        # Embedded jog controls
        try:
            from gui.widgets.jog_button_array import JogButtonArray
            self._zteach_jog = JogButtonArray(compact=True, parent=self)
            self._zteach_jog.jog_xy_requested.connect(self._zteach_jog_xy)
            self._zteach_jog.jog_z_requested.connect(self._zteach_jog_z)
            self._zteach_jog.home_requested.connect(self._jog_xy_home)
            layout.addWidget(self._zteach_jog)
        except ImportError:
            pass

        # Vision tools (detect needle + focus assist)
        if VISION_AVAILABLE:
            nd_row = QHBoxLayout()
            self._btn_detect_needle = QPushButton("Detect Needle")
            self._btn_detect_needle.setMaximumHeight(s(24))
            self._btn_detect_needle.setCheckable(True)
            self._btn_detect_needle.setToolTip(
                "Detect needle tip in camera FOV using vision")
            self._btn_detect_needle.toggled.connect(self._toggle_needle_detect)
            nd_row.addWidget(self._btn_detect_needle)
            self._btn_focus_assist = QPushButton("Focus Assist")
            self._btn_focus_assist.setMaximumHeight(s(24))
            self._btn_focus_assist.setCheckable(True)
            self._btn_focus_assist.setToolTip(
                "Real-time focus quality bar \u2014 adjust Z for sharpest image")
            self._btn_focus_assist.setEnabled(False)
            self._btn_focus_assist.toggled.connect(self._toggle_focus_assist)
            nd_row.addWidget(self._btn_focus_assist)
            layout.addLayout(nd_row)

            self._lbl_needle_status = QLabel("")
            self._lbl_needle_status.setWordWrap(True)
            self._lbl_needle_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_needle_status)

            # v7.3.3: Relaxed detect + accept/reject/adjust row
            relax_row = QHBoxLayout()
            self._btn_detect_needle_relaxed = QPushButton("Relaxed Detect")
            self._btn_detect_needle_relaxed.setMaximumHeight(s(24))
            self._btn_detect_needle_relaxed.setCheckable(True)
            self._btn_detect_needle_relaxed.setToolTip(
                "Detect needle with wide tolerance (when µm/px may be wrong)")
            self._btn_detect_needle_relaxed.toggled.connect(
                self._toggle_needle_detect_relaxed)
            relax_row.addWidget(self._btn_detect_needle_relaxed)
            layout.addLayout(relax_row)

            # Accept/Reject/Adjust row (hidden until detection)
            self._needle_ar_frame = QFrame()
            ar_layout = QHBoxLayout(self._needle_ar_frame)
            ar_layout.setContentsMargins(0, 0, 0, 0)
            ar_layout.setSpacing(4)
            self._btn_needle_minus = QPushButton("\u2212")  # minus sign
            self._btn_needle_minus.setFixedWidth(s(28))
            self._btn_needle_minus.setMaximumHeight(s(24))
            self._btn_needle_minus.setToolTip("Shrink circle")
            self._btn_needle_minus.clicked.connect(
                lambda: self._adjust_needle_radius(-2))
            ar_layout.addWidget(self._btn_needle_minus)
            self._btn_needle_plus = QPushButton("+")
            self._btn_needle_plus.setFixedWidth(s(28))
            self._btn_needle_plus.setMaximumHeight(s(24))
            self._btn_needle_plus.setToolTip("Expand circle")
            self._btn_needle_plus.clicked.connect(
                lambda: self._adjust_needle_radius(2))
            ar_layout.addWidget(self._btn_needle_plus)
            self._btn_needle_accept = QPushButton("Accept")
            self._btn_needle_accept.setMaximumHeight(s(24))
            self._btn_needle_accept.setStyleSheet(
                f"color: {COLORS['green']}; font-weight: bold;")
            self._btn_needle_accept.clicked.connect(self._accept_needle_detection)
            ar_layout.addWidget(self._btn_needle_accept)
            self._btn_needle_reject = QPushButton("Reject")
            self._btn_needle_reject.setMaximumHeight(s(24))
            self._btn_needle_reject.setStyleSheet(
                f"color: {COLORS['red']};")
            self._btn_needle_reject.clicked.connect(self._reject_needle_detection)
            ar_layout.addWidget(self._btn_needle_reject)
            self._needle_ar_frame.setVisible(False)
            layout.addWidget(self._needle_ar_frame)

            self._lbl_focus_status = QLabel("")
            self._lbl_focus_status.setWordWrap(True)
            self._lbl_focus_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_focus_status)
            self._lbl_best_focus = QLabel("")
            self._lbl_best_focus.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            layout.addWidget(self._lbl_best_focus)

        # XY well center: edge-finding (mark left + right edges, auto-center)
        edge_label = QLabel("Find Well Center")
        edge_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt; font-weight: bold;")
        layout.addWidget(edge_label)

        edge_row = QHBoxLayout()
        self._btn_mark_left = QPushButton("\u25c0 Mark Left")
        self._btn_mark_left.setMaximumHeight(s(26))
        self._btn_mark_left.setToolTip(
            "Jog crosshair to the left edge of the well, then click")
        self._btn_mark_left.clicked.connect(self._manual_mark_left)
        edge_row.addWidget(self._btn_mark_left)
        self._btn_mark_right = QPushButton("Mark Right \u25b6")
        self._btn_mark_right.setMaximumHeight(s(26))
        self._btn_mark_right.setToolTip(
            "Jog crosshair to the right edge of the well, then click")
        self._btn_mark_right.clicked.connect(self._manual_mark_right)
        edge_row.addWidget(self._btn_mark_right)
        layout.addLayout(edge_row)

        self._lbl_edge_status = QLabel("")
        self._lbl_edge_status.setWordWrap(True)
        self._lbl_edge_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(self._lbl_edge_status)

        # Direct record buttons: Record XY (center), Record Z, Record XYZ
        rec_row1 = QHBoxLayout()
        btn_rec_xy = QPushButton("Record XY")
        btn_rec_xy.setObjectName("successBtn")
        btn_rec_xy.setMaximumHeight(s(26))
        btn_rec_xy.setToolTip(
            "Record current crosshair position as this well's center")
        btn_rec_xy.clicked.connect(self._manual_record_xy)
        rec_row1.addWidget(btn_rec_xy)
        btn_rec_z = QPushButton("Record Z")
        btn_rec_z.setObjectName("successBtn")
        btn_rec_z.setMaximumHeight(s(26))
        btn_rec_z.setToolTip("Record current Z as this well's bottom")
        btn_rec_z.clicked.connect(self._zteach_record_z)
        rec_row1.addWidget(btn_rec_z)
        btn_rec_both = QPushButton("Record XYZ")
        btn_rec_both.setMaximumHeight(s(26))
        btn_rec_both.setToolTip("Record current position as well center + Z bottom")
        btn_rec_both.clicked.connect(self._manual_record_xyz)
        rec_row1.addWidget(btn_rec_both)
        layout.addLayout(rec_row1)

        # Fit buttons
        fit_row = QHBoxLayout()
        btn_fit_xy = QPushButton("Fit XY (\u22652)")
        btn_fit_xy.setMaximumHeight(s(26))
        btn_fit_xy.setToolTip(
            "Compute calibrated positions from taught XY points (need \u22652)")
        btn_fit_xy.clicked.connect(self._manual_fit_xy)
        fit_row.addWidget(btn_fit_xy)
        btn_fit_z = QPushButton("Fit Z-Plane (\u22653)")
        btn_fit_z.setMaximumHeight(s(26))
        btn_fit_z.setToolTip("Fit Z plane from recorded Z points (need \u22653)")
        btn_fit_z.clicked.connect(self._try_fit_z_plane)
        fit_row.addWidget(btn_fit_z)
        layout.addLayout(fit_row)

        # Status labels
        self._lbl_manual_xy_status = QLabel("XY: 0 wells taught")
        self._lbl_manual_xy_status.setWordWrap(True)
        self._lbl_manual_xy_status.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(self._lbl_manual_xy_status)

        self.lbl_zplane = QLabel("Z: 0 teach points")
        self.lbl_zplane.setWordWrap(True)
        self.lbl_zplane.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(self.lbl_zplane)

        # Legacy third point support
        self.lbl_third = QLabel("")
        self.lbl_third.setVisible(False)
        layout.addWidget(self.lbl_third)

        self._gen_status = QLabel("")
        self._gen_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        layout.addWidget(self._gen_status)

        # ── Validate ──────────────────────────────────────
        val_label = QLabel("Validate")
        val_label.setObjectName("contextSectionLabel")
        layout.addWidget(val_label)

        val_row = QHBoxLayout()
        val_row.addWidget(QLabel("Well:"))
        self.val_well_combo = QComboBox()
        val_row.addWidget(self.val_well_combo, stretch=1)
        btn_goto_well = QPushButton("Go")
        btn_goto_well.setMaximumHeight(s(24))
        btn_goto_well.clicked.connect(self._goto_well)
        val_row.addWidget(btn_goto_well)
        layout.addLayout(val_row)

        self.lbl_val_result = QLabel("")
        self.lbl_val_result.setWordWrap(True)
        self.lbl_val_result.setStyleSheet(f"color: {COLORS['subtext0']};")
        layout.addWidget(self.lbl_val_result)

        # ── Calibration Persistence ──────────────────────
        persist_label = QLabel("Calibration Data")
        persist_label.setObjectName("contextSectionLabel")
        layout.addWidget(persist_label)

        self.ctx_lbl_cal_status = QLabel("Not calibrated")
        self.ctx_lbl_cal_status.setObjectName("contextLabel")
        self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['yellow']};")
        layout.addWidget(self.ctx_lbl_cal_status)

        cal_btn_row = QHBoxLayout()
        btn_save_cal = QPushButton("Save")
        btn_save_cal.setMaximumHeight(s(26))
        btn_save_cal.clicked.connect(self._save_calibration)
        cal_btn_row.addWidget(btn_save_cal)

        btn_load_cal = QPushButton("Load")
        btn_load_cal.setMaximumHeight(s(26))
        btn_load_cal.clicked.connect(self._load_calibration)
        cal_btn_row.addWidget(btn_load_cal)
        layout.addLayout(cal_btn_row)

        parent_layout.addWidget(wiz_frame)

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
        scale = self._xy_position_scale or 1.0
        try:
            if isinstance(xy, dict):
                x_s, y_s = xy.get("x", 0) or 0, xy.get("y", 0) or 0
            elif xy and len(xy) >= 2:
                x_s, y_s = xy[0], xy[1]
            else:
                return
            x_mm = float(x_s) / (scale * 1000.0)
            y_mm = float(y_s) / (scale * 1000.0)
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
                if self._taught_a1:
                    sim.set_plate(self._plate, plate_origin_um=self._taught_a1)
                else:
                    # v7.3.1: Auto-compute A1 from plate center assumption
                    # Stage center = (65000, 42500) µm for 130×85mm travel
                    sim.set_plate(self._plate, plate_center_um=(65000.0, 42500.0))

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

            # v7.3.1: Set focal plane at well bottom (top_z - well_depth)
            if self._top_z is not None and self._plate is not None:
                well_depth = self._plate.well_depth_mm
                if hasattr(self, '_well_depth_spin'):
                    well_depth = self._well_depth_spin.value()
                sim.set_focal_z(self._top_z - well_depth)

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
            if hasattr(self, '_cal_plate_view') and self._taught_a1 is not None and xy[0] is not None:
                nx_mm = (xy[0] - self._taught_a1[0]) / 1000.0
                ny_mm = (xy[1] - self._taught_a1[1]) / 1000.0
                self._cal_plate_view.set_needle_xy(nx_mm, ny_mm)
        else:
            self.lbl_z.setText("—")

    # ════════════════════════════════════════════════════════════════
    #  CALIBRATION STEPS
    # ════════════════════════════════════════════════════════════════

    # ── Step 1: Zero Needle ──────────────────────────────────────

    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None,
                          lower_z=True):
        """Safe navigation using StageController.safe_travel_to().

        v7.3.2: Delegates to the standard safe_travel_to() which waits for
        Z arrival before XY move and XY arrival before Z descent.

        Args:
            target_x_um: Absolute X in µm (raw stage coords).
            target_y_um: Absolute Y in µm (raw stage coords).
            target_z_mm: Optional Z to lower to (mm, zero-ref). Ignored if lower_z=False.
            lower_z: If False, stay at safe Z after XY move (used during plate calibration).
        """
        safe_z = getattr(self, '_safe_z', None) or 0.0

        # Compute final Z target
        final_z = None
        if lower_z:
            if target_z_mm is not None:
                final_z = target_z_mm
            elif getattr(self, '_top_z', None) is not None:
                final_z = self._top_z + getattr(self, '_z_buffer_mm', 0.5)

        self.controller.safe_travel_to(
            target_x_um, target_y_um,
            safe_z_mm=safe_z,
            target_z_mm=final_z,
        )
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

    def _compute_predicted_positions(self):
        """v7.3.1: Compute geometry-predicted positions for all wells from A1 + plate spacing.

        Called after A1 is taught (manual or auto-detect) and after loading calibration.
        Updates both internal state and the plate view overlay.
        """
        if self._taught_a1 is None or self._plate is None:
            self._predicted_positions = None
            return
        self._predicted_positions = self._plate.get_all_positions_from_a1(
            self._taught_a1[0], self._taught_a1[1]
        )
        logger.info(f"Predicted {len(self._predicted_positions)} well positions from A1 + geometry")
        # Update plate view
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:
            self._cal_plate_view.set_predicted_positions(self._predicted_positions)
        # v7.3.1: Populate Z-teach well combo
        self._zteach_populate_wells()
        self._emit_calibration_data_changed()

    # ── v7.3.1: 3-Well Auto-Calibration ────────────────────────────

    def _get_calibration_wells(self) -> list[str]:
        """Return 3 well names forming a right triangle for plate calibration.

        Uses A1, A(last_col), (last_row)(last_col) — e.g. for 96-well: A1, A12, H12.
        """
        from SupportClasses.WellPlate import ROW_LABELS
        if self._plate is None:
            return []
        rows = self._plate.rows
        cols = self._plate.cols
        return [
            "A1",
            f"A{cols}",
            f"{ROW_LABELS[rows - 1]}{cols}",
        ]

    def _compute_approach_position(self, well_x_um: float, well_y_um: float,
                                   well_index: int) -> tuple[float, float]:
        """Compute the stage position for approaching a calibration well.

        If the well diameter fits within 80% of the camera FOV, go to the
        well center. Otherwise, offset to an edge so the circle boundary
        is visible for detection.

        Args:
            well_x_um: Predicted well center X (absolute µm).
            well_y_um: Predicted well center Y (absolute µm).
            well_index: 0, 1, or 2 — determines offset direction for large wells.

        Returns:
            (approach_x_um, approach_y_um)
        """
        if self._plate is None:
            return (well_x_um, well_y_um)

        well_diam_um = self._plate.well_diameter * 1000.0

        # Determine FOV size
        cam = self._get_primary_camera()
        cam_cfg = self._get_camera_config()
        um_per_px = getattr(self, '_scan_um_per_px', 3.34)
        if cam_cfg and cam_cfg.micron_per_pixel:
            um_per_px = cam_cfg.micron_per_pixel
        frame = cam.get_current_frame() if cam else None
        if frame is not None:
            fh, fw = frame.shape[:2]
        else:
            fw, fh = 3664, 2748  # default camera resolution
        fov_w_um = fw * um_per_px
        fov_h_um = fh * um_per_px
        fov_min = min(fov_w_um, fov_h_um)

        # Decision: center vs edge approach
        if well_diam_um < fov_min * 0.8:
            # Well fits comfortably in FOV — go to center
            return (well_x_um, well_y_um)

        # Large well — offset from center so we see the edge
        well_radius_um = well_diam_um / 2.0
        offset = well_radius_um - fov_min / 4.0

        # Vary offset direction per well for better geometry
        if well_index == 0:
            return (well_x_um - offset, well_y_um)  # approach from left
        elif well_index == 1:
            return (well_x_um, well_y_um + offset)   # approach from bottom
        else:
            return (well_x_um, well_y_um - offset)   # approach from top

    def _detect_well_at_position(self, approach_x_um: float,
                                 approach_y_um: float,
                                 well_index: int = 0) -> tuple[float, float] | None:
        """Navigate to a position, capture frame, detect well, return absolute coords.

        For small wells (fit in FOV): uses circle detection.
        For large wells (larger than FOV): uses intensity-profile edge detection.

        Args:
            approach_x_um: Target stage X position.
            approach_y_um: Target stage Y position.
            well_index: 0, 1, or 2 — determines edge detection axis for large wells.

        Returns:
            (detected_x_um, detected_y_um) in absolute stage coords, or None.
        """
        import numpy as np

        self._safe_navigate_to(approach_x_um, approach_y_um, lower_z=False)

        cam = self._get_primary_camera()
        if cam is None:
            return None
        frame = cam.capture_fresh_frame() if hasattr(cam, 'capture_fresh_frame') else cam.get_current_frame()
        if frame is None:
            return None

        # Get actual stage position
        xy = self.controller.get_xy_position(cached=False)
        stage_x = xy[0] if xy and xy[0] is not None else approach_x_um
        stage_y = xy[1] if xy and xy[1] is not None else approach_y_um

        if self._plate is None:
            return None

        fh, fw = frame.shape[:2]
        cam_cfg = self._get_camera_config()
        um_per_px = getattr(self, '_scan_um_per_px', 3.34)
        if cam_cfg and cam_cfg.micron_per_pixel:
            um_per_px = cam_cfg.micron_per_pixel

        diameter_um = self._plate.well_diameter * 1000.0
        fov_min = min(fw, fh) * um_per_px

        # ── Small well: circle detection ──
        if diameter_um < fov_min * 0.8 and VISION_AVAILABLE:
            expected_diam_px = diameter_um / um_per_px
            if expected_diam_px < 10:
                return None
            try:
                from SupportClasses.VisionDetector import WellDetector
                detection = WellDetector.detect_well_with_fallback(
                    frame, expected_diam_px, tolerance=0.5)
                if detection is None:
                    return None
                dx_um, dy_um = pixel_offset_to_stage_um(
                    detection.center_px, (fw, fh), um_per_px)
                return (stage_x + dx_um, stage_y + dy_um)
            except Exception as e:
                logger.debug(f"Circle detection failed: {e}")
                return None

        # ── Large well: intensity-profile edge detection ──
        # The well edge appears as a strong bright↔dark transition.
        # We project the image along the axis parallel to the edge to get
        # a 1D intensity profile, then find the steepest gradient.
        well_radius_um = diameter_um / 2.0

        # Convert to grayscale
        if frame.ndim == 3:
            gray = np.mean(frame, axis=2).astype(np.float64)
        else:
            gray = frame.astype(np.float64)

        # Smooth kernel for noise rejection
        kernel_size = max(11, int(min(fw, fh) * 0.03) | 1)  # odd, ~3% of frame
        kernel = np.ones(kernel_size) / kernel_size
        # Margin: ignore frame edges (artifacts from camera/rendering)
        margin = max(20, int(min(fw, fh) * 0.05))

        if well_index == 0:
            # Approach from -X: edge runs vertically, profile along X
            profile = gray.mean(axis=0)  # average over Y → shape (fw,)
            smooth = np.convolve(profile, kernel, mode='same')
            grad = np.diff(smooth)
            # Search only interior of frame (skip margin on both sides)
            search = np.abs(grad[margin:fw - margin])
            edge_idx = int(np.argmax(search)) + margin
            edge_x_um = stage_x + (edge_idx - fw / 2.0) * um_per_px
            # Positive gradient (dark→bright, L→R) = left edge → center to the right
            # Negative gradient (bright→dark, L→R) = right edge → center to the left
            if grad[edge_idx] > 0:
                center_x = edge_x_um + well_radius_um
            else:
                center_x = edge_x_um - well_radius_um
            logger.debug(f"Edge detection [X]: edge at px {edge_idx}, "
                         f"grad={grad[edge_idx]:.1f}, center_x={center_x:.0f}")
            return (center_x, stage_y)

        else:
            # Approach from +Y (index 1) or -Y (index 2): edge runs horizontally
            profile = gray.mean(axis=1)  # average over X → shape (fh,)
            smooth = np.convolve(profile, kernel, mode='same')
            grad = np.diff(smooth)
            search = np.abs(grad[margin:fh - margin])
            edge_idx = int(np.argmax(search)) + margin
            edge_y_um = stage_y + (edge_idx - fh / 2.0) * um_per_px
            # Positive gradient (dark→bright, top→bottom) = top edge → center below
            # Negative gradient (bright→dark, top→bottom) = bottom edge → center above
            if grad[edge_idx] > 0:
                center_y = edge_y_um + well_radius_um
            else:
                center_y = edge_y_um - well_radius_um
            logger.debug(f"Edge detection [Y]: edge at px {edge_idx}, "
                         f"grad={grad[edge_idx]:.1f}, center_y={center_y:.0f}")
            return (stage_x, center_y)

    def _start_plate_scan(self):
        """Begin 3-well auto-calibration.

        v7.3.1: Moves to 3 strategically chosen wells (right triangle),
        auto-detects each via circle fitting, then computes a similarity
        transform to correct predicted well positions for plate orientation.
        """
        if self._plate is None:
            QMessageBox.warning(self, "Cannot Scan",
                                "Select a plate format first.")
            return
        safe_z = getattr(self, '_safe_z', None)
        if safe_z is None:
            QMessageBox.warning(self, "Cannot Scan",
                                "Set Safe Z first (Step 2A).")
            return

        # Get camera and config
        cam = self._get_primary_camera()
        cam_cfg = self._get_camera_config()
        if cam is None or not getattr(cam, 'is_running', False):
            QMessageBox.warning(self, "No Camera",
                                "Start a camera before scanning.")
            return

        frame = cam.get_current_frame()
        if frame is None:
            QMessageBox.warning(self, "No Frame",
                                "Camera is not producing frames.")
            return
        um_per_px = 3.34
        if cam_cfg is not None:
            scale = cam_cfg.micron_per_pixel or getattr(cam_cfg, 'computed_micron_per_pixel', None)
            if scale:
                um_per_px = scale
        self._scan_um_per_px = um_per_px

        # Compute predicted positions from A1 or plate-center assumption
        stage_center = (65000.0, 42500.0)  # 130×85mm travel center
        if self._taught_a1 is not None:
            self._predicted_positions = self._plate.get_all_positions_from_a1(
                *self._taught_a1)
        else:
            self._predicted_positions = self._plate.get_all_positions_from_plate_center(
                *stage_center)
            # Set plate view reference A1 from computed position
            if "A1" in self._predicted_positions:
                a1_um = self._predicted_positions["A1"]
                zero_x = self.controller.zero_position.get("x", 0)
                zero_y = self.controller.zero_position.get("y", 0)
                a1_mm = ((a1_um[0] - zero_x) / 1000.0, (a1_um[1] - zero_y) / 1000.0)
                if hasattr(self, '_cal_plate_view'):
                    self._cal_plate_view.set_taught_a1(a1_mm)

        # Update plate view with predicted positions
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:
            if self._predicted_positions:
                self._cal_plate_view.set_predicted_positions(self._predicted_positions)

        # Identify the 3 calibration wells
        self._cal_well_names = self._get_calibration_wells()
        self._cal_well_detected: dict[str, tuple[float, float]] = {}
        self._cal_well_index = 0
        self._scanning = True
        self._scan_phase = "three_well"

        # Show calibration wells on plate view
        if hasattr(self, '_cal_plate_view'):
            self._cal_plate_view.set_calibration_wells(set(self._cal_well_names))

        # UI state
        self._btn_start_scan.setEnabled(False)
        self._btn_cancel_scan.setEnabled(True)
        self._btn_accept_scan.setEnabled(False)
        self._lbl_scan_result.setText("")
        self._lbl_scan_progress.setText(
            f"Auto-calibrating: 0/{len(self._cal_well_names)} wells...")

        # Start scan timer
        from PySide6.QtCore import QTimer as _QT
        self._scan_timer = _QT(self)
        self._scan_timer.setSingleShot(True)
        self._scan_timer.timeout.connect(self._scan_tick)
        self._scan_timer.start(0)
        logger.info(f"3-well auto-calibration started: {self._cal_well_names}")

    def _scan_tick(self):
        """Process one calibration well per tick in 3-well auto-calibration."""
        if not self._scanning:
            return

        cal_names = getattr(self, '_cal_well_names', [])
        idx = getattr(self, '_cal_well_index', 0)

        if idx >= len(cal_names):
            self._finish_plate_scan()
            return

        well_name = cal_names[idx]

        # Get predicted position
        if self._predicted_positions is None or well_name not in self._predicted_positions:
            logger.warning(f"No predicted position for {well_name}, skipping")
            self._cal_well_index += 1
            self._scan_timer.start(0)
            return

        pred_x, pred_y = self._predicted_positions[well_name]

        # Compute approach position (center vs edge)
        approach_x, approach_y = self._compute_approach_position(
            pred_x, pred_y, idx)

        # Navigate and detect
        detected = self._detect_well_at_position(approach_x, approach_y, well_index=idx)
        if detected is not None:
            self._cal_well_detected[well_name] = detected
            logger.info(f"Calibration well {well_name} detected at "
                        f"({detected[0]:.0f}, {detected[1]:.0f}) µm")
        else:
            logger.warning(f"Calibration well {well_name} not detected")

        # Update progress
        self._cal_well_index += 1
        n_detected = len(self._cal_well_detected)
        self._lbl_scan_progress.setText(
            f"Auto-calibrating: {self._cal_well_index}/{len(cal_names)} wells "
            f"({n_detected} detected)...")

        # Schedule next tick
        if self._scanning:
            self._scan_timer.start(0)

    def _detect_well_in_fov(self, frame, stage_x_um, stage_y_um):
        """Attempt well detection only when a predicted well is within the camera FOV.

        Returns:
            (well_name, DetectionResult) or (None, None)
        """
        import math as _m
        fh, fw = frame.shape[:2]
        cam_cfg = self._get_camera_config()
        um_per_px = 3.34
        if cam_cfg and cam_cfg.micron_per_pixel:
            um_per_px = cam_cfg.micron_per_pixel

        fov_w_um = fw * um_per_px
        fov_h_um = fh * um_per_px
        half_fov = min(fov_w_um, fov_h_um) / 2.0

        # Find the nearest predicted well within the FOV
        nearest_well = None
        nearest_dist = float('inf')
        for name, (wx, wy) in self._predicted_positions.items():
            d = _m.sqrt((stage_x_um - wx) ** 2 + (stage_y_um - wy) ** 2)
            if d < nearest_dist:
                nearest_dist = d
                nearest_well = name

        # Only attempt detection if the well center is within the FOV
        if nearest_well is None or nearest_dist > half_fov:
            return None, None

        # Compute expected diameter in pixels, capped to 80% of smaller frame dim
        if self._plate is None:
            return None, None
        diameter_um = self._plate.well_diameter * 1000.0
        expected_diam_px = diameter_um / um_per_px
        max_diam_px = min(fw, fh) * 0.8
        expected_diam_px = min(expected_diam_px, max_diam_px)

        if expected_diam_px < 10:
            return None, None

        try:
            from SupportClasses.VisionDetector import WellDetector
            detection = WellDetector.detect_well_with_fallback(
                frame, expected_diam_px, tolerance=0.5)
            if detection is not None:
                return nearest_well, detection
        except Exception as e:
            logger.debug(f"Detection failed near {nearest_well}: {e}")

        return None, None

    def _display_composite(self, composite):
        """Update the plate view with the current stitched composite image."""
        if not hasattr(self, '_cal_plate_view') or self._cal_plate_view is None:
            return
        if composite is None:
            return
        if self._mosaic_builder is None:
            return

        # Use the actual canvas extent (includes half-FOV padding), not scan_bounds
        canvas_extent = self._mosaic_builder.canvas_extent_um
        if canvas_extent is None:
            return

        # Determine A1 position (absolute µm) for coordinate conversion
        if self._taught_a1 is not None:
            a1_x_um = self._taught_a1[0]
            a1_y_um = self._taught_a1[1]
        elif self._predicted_positions and "A1" in self._predicted_positions:
            a1_x_um, a1_y_um = self._predicted_positions["A1"]
        else:
            return

        self._cal_plate_view.update_scan_composite(
            composite, canvas_extent, a1_x_um, a1_y_um)

    def _identify_nearest_well(self, x_um: float, y_um: float) -> str | None:
        """Find the nearest predicted well to a stage position."""
        if self._predicted_positions is None:
            return None
        import math as _m
        best_name = None
        best_dist = float('inf')
        for name, (wx, wy) in self._predicted_positions.items():
            d = _m.sqrt((x_um - wx) ** 2 + (y_um - wy) ** 2)
            if d < best_dist:
                best_dist = d
                best_name = name
        # Only match if within half the well spacing
        if self._plate and best_dist < self._plate.well_spacing_x * 1000.0 * 0.5:
            return best_name
        return None

    def _finish_plate_scan(self):
        """Complete 3-well auto-calibration — fit similarity transform from detections."""
        import numpy as np

        self._scanning = False
        if self._scan_timer:
            self._scan_timer.stop()

        cal_names = getattr(self, '_cal_well_names', [])
        detected = getattr(self, '_cal_well_detected', {})
        n_detected = len(detected)

        self._btn_start_scan.setEnabled(True)
        self._btn_cancel_scan.setEnabled(False)
        self._scan_phase = "idle"

        if n_detected < 2 or self._predicted_positions is None:
            self._btn_accept_scan.setEnabled(False)
            self._lbl_scan_result.setText(
                f"Only {n_detected} well(s) detected — need \u22652 for calibration. "
                f"Try adjusting camera or plate position.")
            self._lbl_scan_result.setStyleSheet(f"color: {COLORS['yellow']}; font-size: 9pt;")
            self._lbl_scan_progress.setText(
                f"Auto-calibration complete: {n_detected}/{len(cal_names)} wells detected")
            logger.warning(f"3-well calibration: only {n_detected} detected")
            return

        # Build matched point arrays: predicted → detected
        pred_pts = []
        det_pts = []
        for wname in cal_names:
            if wname in detected and wname in self._predicted_positions:
                pred_pts.append(self._predicted_positions[wname])
                det_pts.append(detected[wname])

        pred_arr = np.array(pred_pts, dtype=np.float64)
        det_arr = np.array(det_pts, dtype=np.float64)

        # Procrustes: find rotation, scale, translation mapping predicted → detected
        # Centroids
        pred_centroid = pred_arr.mean(axis=0)
        det_centroid = det_arr.mean(axis=0)
        pred_c = pred_arr - pred_centroid
        det_c = det_arr - det_centroid

        # SVD for optimal rotation
        H = pred_c.T @ det_c
        U, S, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0 if d >= 0 else -1.0])
        R = Vt.T @ D @ U.T

        # Scale: ratio of spreads
        pred_spread = np.sqrt((pred_c ** 2).sum())
        det_spread = np.sqrt((det_c ** 2).sum())
        scale = det_spread / pred_spread if pred_spread > 1e-6 else 1.0

        # Translation
        t = det_centroid - scale * (R @ pred_centroid)

        # Compute residuals
        transformed = scale * (pred_arr @ R.T) + t
        residuals = np.sqrt(((transformed - det_arr) ** 2).sum(axis=1))
        residual_um = float(residuals.mean())

        # Extract rotation angle
        rotation_deg = float(math.degrees(math.atan2(R[1, 0], R[0, 0])))

        # Store calibration as AffineCalibration-compatible object
        from SupportClasses.MosaicBuilder import AffineCalibration
        self._three_well_calibration = AffineCalibration(
            rotation_deg=rotation_deg,
            scale=scale,
            translation_um=(float(t[0]), float(t[1])),
            num_points=n_detected,
            residual_um=residual_um,
        )

        # Show detected positions on plate view
        if hasattr(self, '_cal_plate_view'):
            cal_positions = {wname: detected[wname] for wname in detected}
            self._cal_plate_view.set_calibrated_positions(cal_positions)

        self._btn_accept_scan.setEnabled(True)
        self._lbl_scan_result.setText(
            f"Rotation: {rotation_deg:.3f}\u00b0 | "
            f"Scale: {scale:.5f} | "
            f"Residual: {residual_um:.1f} \u00b5m")
        self._lbl_scan_result.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")
        self._lbl_scan_progress.setText(
            f"{n_detected}/{len(cal_names)} wells detected. "
            f"Double-click any well to scan.")

        logger.info(f"3-well calibration done: {n_detected} detected, "
                    f"rotation={rotation_deg:.3f}\u00b0, scale={scale:.5f}, "
                    f"residual={residual_um:.1f} µm")

    def _cancel_plate_scan(self):
        """Cancel an in-progress plate or well scan."""
        self._scanning = False
        self._scan_phase = "idle"
        if self._scan_timer:
            self._scan_timer.stop()
        # Also cancel per-well scan if active
        if getattr(self, '_well_scan_timer', None):
            self._well_scan_timer.stop()
        self._well_scan_active = None
        self._btn_start_scan.setEnabled(True)
        self._btn_cancel_scan.setEnabled(False)
        self._btn_accept_scan.setEnabled(False)
        self._lbl_scan_progress.setText("Scan cancelled")
        self._lbl_scan_result.setText("")
        logger.info("Plate scan cancelled")

    # ── v7.3.1: Per-well on-demand scanning ────────────────────────

    def _start_well_scan(self, well_name: str):
        """Start a local raster scan of a single well area.

        Triggered by double-clicking a well in the plate view. Creates a
        fresh MosaicBuilder scoped to the well's bounding box and runs a
        small serpentine raster scan to produce a high-detail composite.
        """
        scan_phase = getattr(self, '_scan_phase', 'idle')
        if scan_phase != 'idle':
            QMessageBox.warning(self, "Scan Active",
                                "Wait for current scan to complete.")
            return

        if self._plate is None:
            return

        # Get well position (prefer calibrated, fall back to predicted)
        positions = getattr(self, '_calibrated_positions', None) or \
                    getattr(self, '_predicted_positions', None)
        if positions is None or well_name not in positions:
            logger.warning(f"Cannot scan well {well_name}: no position known")
            return

        well_x_um, well_y_um = positions[well_name]

        # Compute local scan bounds
        well_bounds = self._plate.get_single_well_scan_bounds_um(
            well_x_um, well_y_um, margin_factor=1.25)

        # Get camera and config
        cam = self._get_primary_camera()
        cam_cfg = self._get_camera_config()
        if cam is None or not getattr(cam, 'is_running', False):
            QMessageBox.warning(self, "No Camera",
                                "Start a camera before scanning.")
            return

        frame = cam.get_current_frame()
        if frame is None:
            return
        fh, fw = frame.shape[:2]
        um_per_px = getattr(self, '_scan_um_per_px', 3.34)

        # Import MosaicBuilder
        try:
            from SupportClasses.MosaicBuilder import MosaicBuilder
        except ImportError:
            return

        # Create per-well MosaicBuilder with 50% overlap for good stitching
        self._well_scan_builder = MosaicBuilder(
            frame_size_px=(fw, fh),
            micron_per_pixel=um_per_px,
            overlap=0.50,
        )
        self._well_scan_positions = self._well_scan_builder.generate_raster_positions(
            well_bounds, overlap=0.50)
        self._well_scan_index = 0
        self._well_scan_active = well_name
        self._scan_phase = "well_scan"

        # UI state
        self._btn_start_scan.setEnabled(False)
        self._btn_cancel_scan.setEnabled(True)
        total = len(self._well_scan_positions)
        self._lbl_scan_progress.setText(
            f"Scanning well {well_name}: 0/{total} positions...")

        # Start timer
        from PySide6.QtCore import QTimer as _QT
        self._well_scan_timer = _QT(self)
        self._well_scan_timer.setSingleShot(True)
        self._well_scan_timer.timeout.connect(self._well_scan_tick)
        self._well_scan_timer.start(0)
        logger.info(f"Per-well scan started: {well_name}, {total} positions")

    def _well_scan_tick(self):
        """Process one position in a per-well scan."""
        builder = getattr(self, '_well_scan_builder', None)
        active = getattr(self, '_well_scan_active', None)
        if builder is None or active is None:
            return

        positions = self._well_scan_positions
        total = len(positions)
        idx = self._well_scan_index

        if idx >= total:
            self._finish_well_scan()
            return

        tx, ty = positions[idx]
        self._safe_navigate_to(tx, ty)

        cam = self._get_primary_camera()
        frame = cam.capture_fresh_frame() if cam else None
        if frame is None:
            self._well_scan_index += 1
            self._well_scan_timer.start(0)
            return

        xy = self.controller.get_xy_position(cached=False)
        sx = xy[0] if xy and xy[0] is not None else tx
        sy = xy[1] if xy and xy[1] is not None else ty

        builder.add_raster_frame(frame, sx, sy, index=idx)
        builder.stitch_incremental()

        self._well_scan_index += 1
        self._lbl_scan_progress.setText(
            f"Scanning well {active}: {self._well_scan_index}/{total} positions...")

        if self._well_scan_active is not None:
            self._well_scan_timer.start(0)

    def _finish_well_scan(self):
        """Complete a per-well scan — store composite and display on plate view."""
        well_name = getattr(self, '_well_scan_active', None)
        self._well_scan_active = None
        self._scan_phase = "idle"

        if getattr(self, '_well_scan_timer', None):
            self._well_scan_timer.stop()

        builder = getattr(self, '_well_scan_builder', None)
        if builder is None or well_name is None:
            return

        # Build final mosaic for this well
        composite = builder.build_mosaic()
        if composite is not None:
            if not hasattr(self, '_per_well_composites'):
                self._per_well_composites = {}
            self._per_well_composites[well_name] = composite

            # Display on plate view
            if hasattr(self, '_cal_plate_view') and self._plate is not None:
                self._cal_plate_view.set_well_composite(
                    well_name, composite, self._plate.well_diameter)

        self._well_scan_builder = None

        # UI state
        self._btn_start_scan.setEnabled(True)
        self._btn_cancel_scan.setEnabled(False)
        self._lbl_scan_progress.setText(
            f"Well {well_name} scanned. Double-click another well to scan.")
        logger.info(f"Per-well scan finished: {well_name}")

    def _accept_plate_scan(self):
        """Accept the 3-well calibration — apply affine correction to all wells."""
        import numpy as np

        cal = getattr(self, '_three_well_calibration', None)
        if cal is None or self._predicted_positions is None:
            return

        # Apply the similarity transform to all predicted positions
        self._calibrated_positions = cal.correct_positions(self._predicted_positions)

        # Always set _taught_a1 from calibrated A1 position
        # (use direct detection if available, otherwise calibrated prediction)
        zero_x = self.controller.zero_position.get("x", 0)
        zero_y = self.controller.zero_position.get("y", 0)
        detected = getattr(self, '_cal_well_detected', {})
        if "A1" in detected:
            a1_pos = detected["A1"]
        elif "A1" in self._calibrated_positions:
            a1_pos = self._calibrated_positions["A1"]
        else:
            a1_pos = None
        if a1_pos is not None:
            self._taught_a1 = (a1_pos[0], a1_pos[1])
            a1_mm = ((a1_pos[0] - zero_x) / 1000.0, (a1_pos[1] - zero_y) / 1000.0)
            if hasattr(self, '_cal_plate_view'):
                self._cal_plate_view.set_taught_a1(a1_mm)

        # Update scale/rotation for legacy compatibility
        self._scale = cal.scale
        self._rotation = cal.rotation_deg

        # Update plate view
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:
            self._cal_plate_view.set_calibrated_positions(self._calibrated_positions)

        # Show fast-travel buttons for calibration wells
        cal_names = getattr(self, '_cal_well_names', [])
        if hasattr(self, '_cal_travel_frame') and cal_names:
            for i, btn in enumerate(self._cal_travel_btns):
                if i < len(cal_names):
                    btn.setText(cal_names[i])
                    btn.setEnabled(True)
                else:
                    btn.setText("—")
                    btn.setEnabled(False)
            self._cal_travel_frame.setVisible(True)

        # Populate Z-teach combo now that positions are known
        self._zteach_populate_wells()

        self._btn_accept_scan.setEnabled(False)
        self._lbl_scan_progress.setText(
            f"\u2705 Calibrated {len(self._calibrated_positions)} wells from 3-well detection")
        self._lbl_scan_progress.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")

        logger.info(f"3-well calibration accepted: {len(self._calibrated_positions)} wells, "
                    f"rotation={cal.rotation_deg:.3f}\u00b0, scale={cal.scale:.5f}")
        self._emit_calibration_data_changed()

    # ── v7.3.1: Auto Z-Bottom Calibration ─────────────────────────

    def _start_auto_z_cal(self):
        """Start automated Z-bottom calibration for the 3 calibration wells.

        For each well:
        1. Navigate XY to calibration well (at safe Z)
        2. Fast descend to estimated bottom + margin
        3. Coarse sweep downward watching focus score
        4. Fine sweep around the focus peak
        5. Record best-focus Z as well bottom
        6. Retract to safe Z, move to next well
        7. Fit Z plane from 3 results
        """
        # Validation
        if self._plate is None:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Select a plate format first.")
            return
        if getattr(self, '_safe_z', None) is None:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Set Safe Z first (Step 2A).")
            return
        if getattr(self, '_top_z', None) is None:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Set Top Z first (Step 2B).")
            return
        # Need position data to navigate to wells
        if not self._calibrated_positions and not self._predicted_positions:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Run Auto-Calibrate (Step 2C) first.")
            return

        cam = self._get_primary_camera()
        if cam is None or not getattr(cam, 'is_running', False):
            QMessageBox.warning(self, "No Camera",
                                "Start a camera before auto Z calibration.")
            return

        # Get well depth from spinbox
        well_depth_mm = self._well_depth_spin.value()
        self._auto_z_well_depth = well_depth_mm

        # Wells to calibrate (same 3 as plate calibration)
        self._auto_z_wells = self._get_calibration_wells()
        self._auto_z_well_idx = 0
        self._auto_z_results = {}
        self._auto_z_scanning = True
        self._auto_z_phase = "navigate"

        # UI state
        self._btn_auto_z_cal.setEnabled(False)
        self._btn_cancel_auto_z.setEnabled(True)
        self._lbl_auto_z_progress.setText(
            f"Auto Z-Cal: 0/{len(self._auto_z_wells)} wells...")
        self._lbl_auto_z_progress.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: 9pt;")

        # Turn on needle detection + focus assist for visual feedback
        if VISION_AVAILABLE and self._ensure_detection_worker():
            # Start needle detection (also enables focus assist button)
            self._needle_detecting = True
            self._focus_assisting = True
            self._best_focus_z = None
            self._best_focus_score = 0.0
            od_px = self._get_expected_od_px()
            if od_px is not None:
                from gui.widgets.detection_worker import DetectionMode
                self._detection_worker.set_mode(
                    DetectionMode.FOCUS_ASSIST)
                if not self._detection_worker.isRunning():
                    self._detection_worker.start()
            # Activate the toggle buttons so the user sees them on
            btn = getattr(self, '_btn_detect_needle', None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(True)
                btn.blockSignals(False)
            btn_fa = getattr(self, '_btn_focus_assist', None)
            if btn_fa is not None:
                btn_fa.setEnabled(True)
                btn_fa.blockSignals(True)
                btn_fa.setChecked(True)
                btn_fa.blockSignals(False)

        # Start timer
        from PySide6.QtCore import QTimer as _QT
        self._auto_z_timer = _QT(self)
        self._auto_z_timer.setSingleShot(True)
        self._auto_z_timer.timeout.connect(self._auto_z_tick)
        self._auto_z_timer.start(0)

        logger.info(f"Auto Z-Cal started: {self._auto_z_wells}, "
                    f"well_depth={well_depth_mm:.2f}mm")

    def _cancel_auto_z(self):
        """Cancel the auto Z-calibration in progress."""
        self._auto_z_scanning = False
        self._auto_z_phase = "idle"
        if self._auto_z_timer is not None:
            self._auto_z_timer.stop()
        # Retract to safe Z
        if self.controller.is_zp_connected and self._safe_z is not None:
            self.controller.move_z_absolute(self._safe_z, from_zero_ref=True)
        # Stop detection worker
        self._stop_detection()
        self._auto_z_uncheck_vision_buttons()
        # UI
        self._btn_auto_z_cal.setEnabled(True)
        self._btn_cancel_auto_z.setEnabled(False)
        self._lbl_auto_z_progress.setText("Auto Z-Cal cancelled.")
        self._lbl_auto_z_progress.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 9pt;")
        logger.info("Auto Z-Cal cancelled")

    def _auto_z_tick(self):
        """State machine for auto Z-bottom calibration."""
        if not self._auto_z_scanning:
            return

        idx = self._auto_z_well_idx
        wells = self._auto_z_wells

        if idx >= len(wells):
            self._auto_z_finish()
            return

        well_name = wells[idx]

        if self._auto_z_phase == "navigate":
            # Navigate to well XY at safe Z (no Z lowering)
            est = None
            if self._calibrated_positions and well_name in self._calibrated_positions:
                est = self._calibrated_positions[well_name]
            elif self._predicted_positions and well_name in self._predicted_positions:
                est = self._predicted_positions[well_name]
            if est is None:
                logger.warning(f"Auto Z-Cal: no position for {well_name}, skipping")
                self._auto_z_well_idx += 1
                self._auto_z_phase = "navigate"
                self._auto_z_timer.start(0)
                return

            self._safe_navigate_to(est[0], est[1], lower_z=False)
            self._lbl_auto_z_progress.setText(
                f"Auto Z-Cal: {well_name} — descending to search region...")

            # Compute sweep range — SAFETY: never go below estimated bottom
            well_depth = self._auto_z_well_depth
            estimated_bottom = self._top_z - well_depth
            approach_margin = 2.0   # mm above estimated bottom to start search

            self._auto_z_current_z = estimated_bottom + approach_margin
            # Hard safety floor: never descend below estimated bottom
            self._auto_z_sweep_end = estimated_bottom
            self._auto_z_best_z = None
            self._auto_z_best_score = 0.0
            self._auto_z_baseline_score = 0.0
            self._auto_z_decline_count = 0

            # Fast descend to approach position
            self.controller.move_z_absolute(
                self._auto_z_current_z, from_zero_ref=True)

            # Capture baseline focus score (no needle in focus here)
            self._auto_z_capture_baseline()

            self._auto_z_phase = "coarse"
            self._auto_z_timer.start(300)

        elif self._auto_z_phase == "coarse":
            # Coarse sweep: 0.15mm steps, descending from above.
            # Looking for needle to first appear (blurry focus rising).
            # SAFETY: never descend below estimated bottom.
            score = self._auto_z_get_focus_score()
            z = self._auto_z_current_z

            self._lbl_auto_z_progress.setText(
                f"Auto Z-Cal: {well_name} \u2014 coarse Z={z:.3f}mm  "
                f"focus={score:.1f}")

            # Track best seen so far
            if score > self._auto_z_best_score:
                self._auto_z_best_score = score
                self._auto_z_best_z = z
                self._auto_z_decline_count = 0
            elif self._auto_z_best_z is not None:
                self._auto_z_decline_count += 1

            # Detect needle appearing: score rising above 2x baseline
            needle_detected = (
                self._auto_z_baseline_score > 0
                and score > 2.0 * self._auto_z_baseline_score
            ) or (
                self._auto_z_best_score > 0
                and self._auto_z_best_score > 2.0 * max(self._auto_z_baseline_score, 1.0)
            )

            if needle_detected:
                # Needle is appearing — switch to fine sweep.
                # Back up above best Z to re-approach slowly.
                fine_start = self._auto_z_best_z + 0.3
                self._auto_z_current_z = fine_start
                self.controller.move_z_absolute(fine_start, from_zero_ref=True)
                self._auto_z_best_z = None
                self._auto_z_best_score = 0.0
                self._auto_z_decline_count = 0
                self._auto_z_phase = "fine"
                self._auto_z_timer.start(300)
                return

            # SAFETY: hit the floor — do not descend further
            if z <= self._auto_z_sweep_end:
                if self._auto_z_best_z is not None:
                    # Had some focus signal, refine it
                    fine_start = self._auto_z_best_z + 0.3
                    self._auto_z_current_z = fine_start
                    self.controller.move_z_absolute(fine_start, from_zero_ref=True)
                    self._auto_z_best_z = None
                    self._auto_z_best_score = 0.0
                    self._auto_z_decline_count = 0
                    self._auto_z_phase = "fine"
                    self._auto_z_timer.start(300)
                else:
                    logger.warning(f"Auto Z-Cal: no focus found for {well_name} "
                                   f"(stopped at safety floor Z={z:.3f}mm)")
                    self._lbl_auto_z_progress.setText(
                        f"Auto Z-Cal: {well_name} \u2014 no focus detected "
                        f"(stopped at safety floor)")
                    self._auto_z_phase = "retract"
                    self._auto_z_timer.start(0)
                return

            # Step down (coarse)
            next_z = self._auto_z_current_z - self._auto_z_coarse_step
            # Clamp to safety floor
            if next_z < self._auto_z_sweep_end:
                next_z = self._auto_z_sweep_end
            self._auto_z_current_z = next_z
            self.controller.move_z_absolute(next_z, from_zero_ref=True)
            self._auto_z_timer.start(300)

        elif self._auto_z_phase == "fine":
            # Fine sweep: 0.03mm steps, tracking peak focus.
            # SAFETY: approach from above only. Stop as soon as focus
            # declines — the peak IS the well bottom, do not go past it.
            score = self._auto_z_get_focus_score()
            z = self._auto_z_current_z

            self._lbl_auto_z_progress.setText(
                f"Auto Z-Cal: {well_name} \u2014 fine Z={z:.3f}mm  "
                f"focus={score:.1f}  "
                f"best={self._auto_z_best_score:.1f}")

            if score > self._auto_z_best_score:
                self._auto_z_best_score = score
                self._auto_z_best_z = z
                self._auto_z_decline_count = 0
            elif self._auto_z_best_z is not None:
                self._auto_z_decline_count += 1

            # Peak found: 3 consecutive declining steps → stop immediately.
            # The needle is approaching the glass — do NOT continue down.
            if self._auto_z_decline_count >= 3 and self._auto_z_best_z is not None:
                self._auto_z_results[well_name] = self._auto_z_best_z
                self._z_teach_points[well_name] = self._auto_z_best_z
                logger.info(f"Auto Z-Cal: {well_name} bottom at "
                            f"Z={self._auto_z_best_z:.3f}mm "
                            f"(score={self._auto_z_best_score:.1f})")
                self._auto_z_phase = "retract"
                self._auto_z_timer.start(0)
                return

            # SAFETY: hard floor — never descend below estimated bottom
            if z <= self._auto_z_sweep_end:
                if self._auto_z_best_z is not None:
                    self._auto_z_results[well_name] = self._auto_z_best_z
                    self._z_teach_points[well_name] = self._auto_z_best_z
                    logger.info(f"Auto Z-Cal: {well_name} bottom at "
                                f"Z={self._auto_z_best_z:.3f}mm "
                                f"(stopped at safety floor)")
                else:
                    logger.warning(f"Auto Z-Cal: {well_name} no clear peak "
                                   f"(stopped at safety floor)")
                self._auto_z_phase = "retract"
                self._auto_z_timer.start(0)
                return

            # Step down (fine) — clamp to safety floor
            next_z = self._auto_z_current_z - self._auto_z_fine_step
            if next_z < self._auto_z_sweep_end:
                next_z = self._auto_z_sweep_end
            self._auto_z_current_z = next_z
            self.controller.move_z_absolute(next_z, from_zero_ref=True)
            self._auto_z_timer.start(300)

        elif self._auto_z_phase == "retract":
            # Retract to safe Z, then move to next well
            self.controller.move_z_absolute(self._safe_z, from_zero_ref=True)

            # Update progress
            n_done = len(self._auto_z_results)
            n_total = len(self._auto_z_wells)
            self._lbl_auto_z_progress.setText(
                f"Auto Z-Cal: {n_done}/{n_total} wells done")

            # Update Z teach display
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(
                    f"{len(self._z_teach_points)}/3 teach points")
                if len(self._z_teach_points) >= 3:
                    self.lbl_zplane.setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 9pt;")

            # Next well
            self._auto_z_well_idx += 1
            self._auto_z_phase = "navigate"
            self._auto_z_timer.start(200)

    def _auto_z_uncheck_vision_buttons(self):
        """Uncheck the detect needle / focus assist toggle buttons without triggering handlers."""
        for attr in ('_btn_detect_needle', '_btn_focus_assist'):
            btn = getattr(self, attr, None)
            if btn is not None and btn.isChecked():
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)

    def _auto_z_capture_baseline(self):
        """Capture baseline focus score at approach position (no needle in focus)."""
        cam = self._get_primary_camera()
        if cam is None:
            self._auto_z_baseline_score = 0.0
            return
        import time
        time.sleep(0.2)  # brief settle for camera
        frame = cam.get_current_frame()
        if frame is None:
            self._auto_z_baseline_score = 0.0
            return
        try:
            from SupportClasses.VisionDetector import NeedleDetector
            result = NeedleDetector.compute_focus_score(frame)
            self._auto_z_baseline_score = result.score
            logger.debug(f"Auto Z baseline focus: {result.score:.1f}")
        except Exception:
            self._auto_z_baseline_score = 0.0

    def _auto_z_get_focus_score(self) -> float:
        """Grab current frame and compute focus score synchronously."""
        cam = self._get_primary_camera()
        if cam is None:
            return 0.0
        frame = cam.get_current_frame()
        if frame is None:
            return 0.0
        try:
            from SupportClasses.VisionDetector import NeedleDetector
            result = NeedleDetector.compute_focus_score(frame)
            return result.score
        except Exception:
            return 0.0

    def _auto_z_finish(self):
        """Complete auto Z-cal: fit Z plane from recorded points."""
        self._auto_z_scanning = False
        self._auto_z_phase = "idle"
        # Stop detection worker
        self._stop_detection()
        self._auto_z_uncheck_vision_buttons()

        n_found = len(self._auto_z_results)
        if n_found == 0:
            self._lbl_auto_z_progress.setText(
                "Auto Z-Cal failed: no well bottoms detected.")
            self._lbl_auto_z_progress.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 9pt;")
        else:
            # Build result summary
            parts = [f"{w}: Z={z:.3f}mm" for w, z in self._auto_z_results.items()]
            self._lbl_auto_z_progress.setText(
                f"\u2705 Auto Z-Cal: {n_found} wells \u2014 {', '.join(parts)}")
            self._lbl_auto_z_progress.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 9pt;")

            # Auto-fit Z plane if we have 3+ points
            if n_found >= 3:
                self._try_fit_z_plane()

        # UI state
        self._btn_auto_z_cal.setEnabled(True)
        self._btn_cancel_auto_z.setEnabled(False)

        logger.info(f"Auto Z-Cal finished: {n_found} wells, "
                    f"results={self._auto_z_results}")

    # ── v7.3.1: Manual Z-Offset Teaching ──────────────────────────

    def _zteach_populate_wells(self):
        """Populate the Z-teach well combo from the current plate."""
        if not hasattr(self, '_zteach_well_combo'):
            return
        self._zteach_well_combo.clear()
        if self._plate is None:
            return
        # Pre-select strategic wells for plane fitting
        wells = self._plate.well_names
        self._zteach_well_combo.addItems(wells)
        # Default to A1
        self._zteach_well_combo.setCurrentIndex(0)

    def _zteach_goto_well(self):
        """Navigate to the selected Z-teach well."""
        well_name = self._zteach_well_combo.currentText()
        if not well_name:
            return
        self._navigate_to_well(well_name)

    def _zteach_next_well(self):
        """Advance to the next well in the combo and navigate there."""
        if not hasattr(self, '_zteach_well_combo'):
            return
        idx = self._zteach_well_combo.currentIndex()
        if idx < self._zteach_well_combo.count() - 1:
            self._zteach_well_combo.setCurrentIndex(idx + 1)
        else:
            self._zteach_well_combo.setCurrentIndex(0)
        self._zteach_goto_well()

    def _zteach_record_z(self):
        """Record current Z position for the selected well."""
        well_name = self._zteach_well_combo.currentText()
        if not well_name:
            return
        zp = self.controller.get_zp_position(cached=False)
        if zp is None or zp[0] is None:
            return
        z_offset = zp[0] - self.controller.zero_position.get("Z", 0)
        self._z_teach_points[well_name] = z_offset

        # Also store as taught_a1_z / taught_corner_z / taught_third_z for legacy compat
        if well_name == "A1":
            self._taught_a1_z = z_offset
        elif well_name == self._corner_well:
            self._taught_corner_z = z_offset
        elif self._taught_third is None or well_name == getattr(self, '_third_well', None):
            self._taught_third_z = z_offset
            self._third_well = well_name
            xy = self.controller.get_xy_position(cached=False)
            if xy and xy[0] is not None:
                self._taught_third = (xy[0], xy[1])

        n = len(self._z_teach_points)
        if hasattr(self, 'lbl_zplane'):
            wells_str = ", ".join(sorted(self._z_teach_points.keys()))
            self.lbl_zplane.setText(f"{n}/3 teach points: {wells_str}")
            color = COLORS['green'] if n >= 3 else COLORS['yellow']
            self.lbl_zplane.setStyleSheet(f"color: {color}; font-size: 9pt;")

        logger.info(f"Z-teach: {well_name} = {z_offset:.3f} mm ({n} points total)")

    # ── v7.3.2: Manual XY + XYZ Teaching ──────────────────────────

    def _manual_mark_left(self):
        """Mark the left edge of the well at the current crosshair position."""
        xy = self.controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            return
        self._edge_left = (xy[0], xy[1])
        self._btn_mark_left.setStyleSheet(
            f"background-color: {COLORS['green']}; color: {COLORS['crust']};")
        if hasattr(self, '_lbl_edge_status'):
            if self._edge_right is not None:
                self._lbl_edge_status.setText("Both edges marked. Computing center...")
                self._compute_center_from_edges()
            else:
                self._lbl_edge_status.setText(
                    f"Left: ({xy[0]:.0f}, {xy[1]:.0f}) \u2014 now mark right edge")
        logger.debug(f"Edge left: ({xy[0]:.1f}, {xy[1]:.1f})")

    def _manual_mark_right(self):
        """Mark the right edge of the well at the current crosshair position."""
        xy = self.controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            return
        self._edge_right = (xy[0], xy[1])
        self._btn_mark_right.setStyleSheet(
            f"background-color: {COLORS['green']}; color: {COLORS['crust']};")
        if hasattr(self, '_lbl_edge_status'):
            if self._edge_left is not None:
                self._lbl_edge_status.setText("Both edges marked. Computing center...")
                self._compute_center_from_edges()
            else:
                self._lbl_edge_status.setText(
                    f"Right: ({xy[0]:.0f}, {xy[1]:.0f}) \u2014 now mark left edge")
        logger.debug(f"Edge right: ({xy[0]:.1f}, {xy[1]:.1f})")

    def _compute_center_from_edges(self):
        """Compute well center as midpoint of left and right edges, record it."""
        if self._edge_left is None or self._edge_right is None:
            return
        cx = (self._edge_left[0] + self._edge_right[0]) / 2.0
        cy = (self._edge_left[1] + self._edge_right[1]) / 2.0

        well_name = self._zteach_well_combo.currentText()
        if not well_name:
            return

        self._xy_teach_points[well_name] = (cx, cy)

        # Wire into legacy refs
        if well_name == "A1":
            self._taught_a1 = (cx, cy)
            self._compute_predicted_positions()
        elif well_name == self._corner_well:
            self._taught_corner = (cx, cy)

        # Compute diameter for info
        import math
        dx = self._edge_right[0] - self._edge_left[0]
        dy = self._edge_right[1] - self._edge_left[1]
        diameter_um = math.sqrt(dx * dx + dy * dy)

        self._update_manual_xy_status()
        if hasattr(self, '_lbl_edge_status'):
            self._lbl_edge_status.setText(
                f"\u2705 {well_name} center: ({cx:.0f}, {cy:.0f}) \u00b5m "
                f"| \u00d8 {diameter_um:.0f} \u00b5m")
            self._lbl_edge_status.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 9pt;")
        logger.info(f"Edge-center: {well_name} = ({cx:.1f}, {cy:.1f}) "
                    f"\u00b5m, diameter={diameter_um:.0f} \u00b5m")

        # Reset edge state + button styles for next well
        self._edge_left = None
        self._edge_right = None
        self._btn_mark_left.setStyleSheet("")
        self._btn_mark_right.setStyleSheet("")

    def _manual_record_xy(self):
        """Record current XY position as the selected well's center."""
        well_name = self._zteach_well_combo.currentText()
        if not well_name:
            return
        xy = self.controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            return
        self._xy_teach_points[well_name] = (xy[0], xy[1])

        # Also wire into legacy A1/corner refs
        if well_name == "A1":
            self._taught_a1 = (xy[0], xy[1])
            self._compute_predicted_positions()
        elif well_name == self._corner_well:
            self._taught_corner = (xy[0], xy[1])

        self._update_manual_xy_status()
        logger.info(f"XY-teach: {well_name} = ({xy[0]:.1f}, {xy[1]:.1f}) \u00b5m "
                    f"({len(self._xy_teach_points)} points)")

    def _manual_record_xyz(self):
        """Record current XY + Z for the selected well."""
        self._manual_record_xy()
        self._zteach_record_z()

    def _update_manual_xy_status(self):
        """Update the manual XY teaching status label."""
        if not hasattr(self, '_lbl_manual_xy_status'):
            return
        n = len(self._xy_teach_points)
        if n == 0:
            self._lbl_manual_xy_status.setText("XY: 0 wells taught")
            self._lbl_manual_xy_status.setStyleSheet(
                f"color: {COLORS['overlay0']}; font-size: 9pt;")
        else:
            names = ", ".join(sorted(self._xy_teach_points.keys()))
            color = COLORS['green'] if n >= 2 else COLORS['yellow']
            self._lbl_manual_xy_status.setText(f"XY: {n} wells taught: {names}")
            self._lbl_manual_xy_status.setStyleSheet(
                f"color: {color}; font-size: 9pt;")

    def _manual_fit_xy(self):
        """Compute calibrated well positions from manually taught XY points.

        With 1 point: uses geometry prediction from A1.
        With 2 points: computes scale + rotation (legacy alignment).
        With 3+ points: uses Procrustes SVD if available, else scale+rotation from first 2.
        """
        n = len(self._xy_teach_points)
        if n == 0:
            if hasattr(self, '_gen_status'):
                self._gen_status.setText("Need at least 1 taught XY point.")
                self._gen_status.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
            return

        # With just A1, compute geometry-predicted positions
        if n == 1 and "A1" in self._xy_teach_points:
            self._taught_a1 = self._xy_teach_points["A1"]
            self._compute_predicted_positions()
            if hasattr(self, '_gen_status'):
                self._gen_status.setText(
                    "1 point (A1): geometry-predicted positions applied.")
                self._gen_status.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")
            self._emit_calibration_data_changed()
            return

        # With 2+ points, try Procrustes/affine fit
        if self._plate is None:
            if hasattr(self, '_gen_status'):
                self._gen_status.setText("Select a plate format first.")
                self._gen_status.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
            return

        # Build predicted vs measured point pairs
        if self._predicted_positions is None:
            # Need A1 to compute predictions
            if "A1" in self._xy_teach_points:
                self._taught_a1 = self._xy_teach_points["A1"]
                self._compute_predicted_positions()
            else:
                # Use stage center as rough estimate
                center_x, center_y = 65000.0, 42500.0
                self._predicted_positions = \
                    self._plate.get_all_positions_from_plate_center(center_x, center_y)

        if self._predicted_positions is None:
            return

        # Collect matched point pairs (predicted, measured)
        pred_pts = []
        meas_pts = []
        for name, (mx, my) in self._xy_teach_points.items():
            if name in self._predicted_positions:
                px, py = self._predicted_positions[name]
                pred_pts.append((px, py))
                meas_pts.append((mx, my))

        if len(pred_pts) < 2:
            if hasattr(self, '_gen_status'):
                self._gen_status.setText(
                    f"Need \u22652 matched wells (have {len(pred_pts)}). "
                    "Ensure taught wells exist in plate format.")
                self._gen_status.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
            return

        # Try 3-well Procrustes SVD calibration
        try:
            from SupportClasses.MosaicCalibrator import MosaicCalibrator
            mcal = MosaicCalibrator()
            for (px, py), (mx, my) in zip(pred_pts, meas_pts):
                mcal.add_point(px, py, mx, my)
            mcal.solve()
            self._calibrated_positions = mcal.correct_positions(
                self._predicted_positions)
            # v7.3.4: Convert to AffineCalibration so _save_calibration persists it
            try:
                import numpy as _np
                from SupportClasses.MosaicBuilder import AffineCalibration
                _R = mcal._R
                self._three_well_calibration = AffineCalibration(
                    rotation_deg=float(_np.degrees(_np.arctan2(_R[1, 0], _R[0, 0]))),
                    scale=mcal._scale,
                    translation_um=(mcal._tx, mcal._ty),
                    center_um=(0.0, 0.0),
                    num_points=mcal.n_points,
                    residual_um=mcal.rms_error_um,
                )
            except Exception:
                pass
            if hasattr(self, '_cal_plate_view'):
                self._cal_plate_view.set_calibrated_positions(
                    self._calibrated_positions)
            n_cal = len(self._calibrated_positions)
            err = mcal.rms_error_um if hasattr(mcal, 'rms_error_um') else 0
            if hasattr(self, '_gen_status'):
                self._gen_status.setText(
                    f"\u2705 Calibrated {n_cal} wells from {n} taught points"
                    + (f" (RMS: {err:.1f} \u00b5m)" if err else ""))
                self._gen_status.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: 9pt;")
            logger.info(f"Manual XY fit: {n_cal} wells from {n} taught points")
            self._save_calibration()  # v7.3.4: auto-persist calibrated positions
        except (ImportError, Exception) as e:
            # Fallback: 2-point scale+rotation via legacy method
            if "A1" in self._xy_teach_points:
                self._taught_a1 = self._xy_teach_points["A1"]
            corner_key = self._corner_well
            if corner_key in self._xy_teach_points:
                self._taught_corner = self._xy_teach_points[corner_key]
            if self._taught_a1 and self._taught_corner:
                self._calculate_alignment()
                if hasattr(self, '_gen_status'):
                    self._gen_status.setText(
                        f"\u2705 2-point alignment (A1 + {corner_key})")
                    self._gen_status.setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 9pt;")
            else:
                if hasattr(self, '_gen_status'):
                    self._gen_status.setText(f"Fit failed: {e}")
                    self._gen_status.setStyleSheet(
                        f"color: {COLORS['red']}; font-size: 9pt;")
                logger.warning(f"Manual XY fit failed: {e}")
                return

        self._emit_calibration_data_changed()

    def _zteach_jog_xy(self, dx_um: float, dy_um: float):
        """Handle XY jog from embedded jog array."""
        if self.controller.is_xy_connected:
            self.controller.move_xy_relative_um(dx_um, dy_um)

    def _zteach_jog_z(self, dz_mm: float):
        """Handle Z jog from embedded jog array."""
        if self.controller.is_zp_connected:
            self.controller.move_z_relative(dz_mm)

    def _jog_xy_home(self):
        """Move to zero reference."""
        if self.controller.is_xy_connected:
            self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
        if self.controller.is_zp_connected:
            self.controller.move_z_absolute(0, from_zero_ref=True)

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

    # v7.3.2: Step 1 "Zero Needle" removed — use jog page Set Zero instead.

    # ── Teach Well Plate ──────────────────────────────────────────

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
        self._emit_calibration_data_changed()

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
        # v7.3.1: Compute geometry-predicted positions for all wells
        self._compute_predicted_positions()

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
        """v7.3.1: Z-plane fit using all available teach points.
        Combines legacy A1/corner/third points with v7.3.1 z_teach_points.
        """
        # Collect all unique points (z_teach_points override legacy)
        points_dict: dict[str, float] = {}
        if getattr(self, '_taught_a1', None) and getattr(self, '_taught_a1_z', None) is not None:
            points_dict["A1"] = self._taught_a1_z
        if getattr(self, '_taught_corner', None) and getattr(self, '_taught_corner_z', None) is not None:
            points_dict[getattr(self, '_corner_well', 'corner')] = self._taught_corner_z
        if getattr(self, '_taught_third', None) and getattr(self, '_taught_third_z', None) is not None:
            points_dict[getattr(self, '_third_well', '3rd')] = self._taught_third_z
        # v7.3.1: Z-teach points override legacy
        for name, z in self._z_teach_points.items():
            points_dict[name] = z
        points = list(points_dict.items())
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
        """v7.2.7: Navigate to any well via plate view click.
        v7.3.1: Prefers calibrated > predicted > estimated positions.
        """
        if self._safe_z is None:
            QMessageBox.warning(self, "No Safe Z",
                                "Set Safe Z height before navigating.")
            return
        # v7.3.1: Use calibrated position if available, then predicted, then estimated
        est = None
        if self._calibrated_positions and well_name in self._calibrated_positions:
            est = self._calibrated_positions[well_name]
        elif self._predicted_positions and well_name in self._predicted_positions:
            est = self._predicted_positions[well_name]
        elif self._taught_a1 is not None:
            est = self._estimate_well_position_um(well_name)
        if est is None:
            QMessageBox.warning(self, "No Position Data",
                                "Run Auto-Calibrate first, or select a plate format.")
            return
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

    def _goto_calibration_well(self, idx: int):
        """Fast-travel to one of the 3 calibration wells."""
        cal_names = getattr(self, '_cal_well_names', [])
        if idx >= len(cal_names):
            return
        self._navigate_to_well(cal_names[idx])

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
        self._predicted_positions = None  # v7.3.1: clear predictions on plate change
        self._calibrated_positions = None
        self._xy_teach_points.clear()  # v7.3.2: clear manual XY teach points
        self._z_teach_points.clear()   # v7.3.2: clear manual Z teach points
        self._three_well_calibration = None
        if hasattr(self, '_taught_third'): self._taught_third = None
        for attr in ['lbl_a1','lbl_corner','lbl_third']:
            if hasattr(self, attr): getattr(self, attr).setText("—")
        if hasattr(self, 'lbl_alignment'): self.lbl_alignment.setText("")
        # Reset fast-travel buttons
        if hasattr(self, '_cal_travel_frame'):
            self._cal_travel_frame.setVisible(False)
            for btn in self._cal_travel_btns:
                btn.setText("—")
                btn.setEnabled(False)
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view:
            self._cal_plate_view._predicted_positions = None
            self._cal_plate_view.set_plate(self._plate)

        # Update well depth spinbox from plate definition
        if hasattr(self, '_well_depth_spin') and self._plate is not None:
            self._well_depth_spin.setValue(self._plate.well_depth_mm)


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
        # v7.3.1: Compute geometry-predicted positions for all wells
        self._compute_predicted_positions()

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
        v7.3.1: Prefers calibrated positions, lowers needle to focus Z.

        Navigates XY to the well, then lowers the needle to the
        Z-plane-predicted bottom (focus position) so the user can
        verify the needle is at the correct well center and depth.
        """
        well = self.val_well_combo.currentText()
        if not well or not self._plate:
            return

        # v7.3.1: Use calibrated > predicted > estimated positions
        target_x_um = target_y_um = None
        if self._calibrated_positions and well in self._calibrated_positions:
            target_x_um, target_y_um = self._calibrated_positions[well]
        elif self._predicted_positions and well in self._predicted_positions:
            target_x_um, target_y_um = self._predicted_positions[well]
        elif self._taught_a1 is not None:
            try:
                pos = self._plate.get_well_position(well)
            except KeyError:
                return
            if pos is None:
                return
            a1_expected = self._plate.get_well_position("A1")
            if a1_expected is None:
                return
            dx_mm = pos[0] - a1_expected[0]
            dy_mm = pos[1] - a1_expected[1]
            rad = math.radians(self._rotation)
            rx = dx_mm * math.cos(rad) - dy_mm * math.sin(rad)
            ry = dx_mm * math.sin(rad) + dy_mm * math.cos(rad)
            target_x_um = self._taught_a1[0] + rx * self._scale * 1000.0
            target_y_um = self._taught_a1[1] + ry * self._scale * 1000.0
        if target_x_um is None:
            return

        # Compute target Z: Z-plane predicted bottom, or recorded teach point
        target_z = None
        z_source = ""
        if self._z_plane_result:
            try:
                rel_x, rel_y = self._plate.get_well_position(well)
                target_z = self._z_plane_result.z_at(rel_x, rel_y)
                z_source = "Z-plane"
            except Exception:
                pass
        if target_z is None and well in self._z_teach_points:
            target_z = self._z_teach_points[well]
            z_source = "recorded"

        # Navigate XY at safe Z, then lower to target Z (focus position)
        if hasattr(self, '_safe_z') and self._safe_z is not None:
            self._safe_navigate_to(target_x_um, target_y_um, target_z)
        else:
            self.controller.move_xy_absolute(
                target_x_um, target_y_um, from_zero_ref=False)

        # Display
        rel_x = target_x_um - self.controller.zero_position["x"]
        rel_y = target_y_um - self.controller.zero_position["y"]
        z_str = ""
        if target_z is not None:
            z_str = f"  Z={target_z:.3f}mm ({z_source})"
        if hasattr(self, 'lbl_val_result'):
            self.lbl_val_result.setText(
                f"Moving to {well} \u2192 ({rel_x:,.1f}, {rel_y:,.1f}) \u00b5m{z_str}")


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
        # v7.3.1: Save 3-well affine calibration
        three_well_cal = getattr(self, '_three_well_calibration', None)
        if three_well_cal is not None and not three_well_cal.is_identity:
            cal_data["mosaic_affine"] = {
                "rotation_deg": three_well_cal.rotation_deg,
                "scale": three_well_cal.scale,
                "translation_um": list(three_well_cal.translation_um),
                "center_um": list(three_well_cal.center_um),
                "num_points": three_well_cal.num_points,
                "residual_um": three_well_cal.residual_um,
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
                # Update well depth spinbox from plate definition
                if hasattr(self, '_well_depth_spin'):
                    self._well_depth_spin.setValue(self._plate.well_depth_mm)
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

        # v7.3.1: Recompute predicted positions from loaded A1 + plate
        self._compute_predicted_positions()

        # v7.3.1: Restore mosaic affine calibration and recompute calibrated positions
        affine_data = cal.get("mosaic_affine")
        if affine_data and self._predicted_positions:
            try:
                from SupportClasses.MosaicBuilder import AffineCalibration
                mcal = AffineCalibration(
                    rotation_deg=affine_data["rotation_deg"],
                    scale=affine_data["scale"],
                    translation_um=tuple(affine_data["translation_um"]),
                    center_um=tuple(affine_data["center_um"]),
                    num_points=affine_data.get("num_points", 0),
                    residual_um=affine_data.get("residual_um", 0.0),
                )
                self._three_well_calibration = mcal
                self._calibrated_positions = mcal.correct_positions(self._predicted_positions)
                if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:
                    self._cal_plate_view.set_calibrated_positions(self._calibrated_positions)
                if hasattr(self, '_lbl_scan_progress'):
                    self._lbl_scan_progress.setText(
                        f"\u2705 Loaded: {len(self._calibrated_positions)} calibrated wells")
                    self._lbl_scan_progress.setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 9pt;")
                logger.info(f"3-well affine restored: {mcal.rotation_deg:.3f}\u00b0, "
                            f"scale={mcal.scale:.5f}")
                # Restore fast-travel buttons
                cal_names = self._get_calibration_wells()
                if hasattr(self, '_cal_travel_frame') and cal_names:
                    self._cal_well_names = cal_names
                    for i, btn in enumerate(self._cal_travel_btns):
                        if i < len(cal_names):
                            btn.setText(cal_names[i])
                            btn.setEnabled(True)
                        else:
                            btn.setText("—")
                            btn.setEnabled(False)
                    self._cal_travel_frame.setVisible(True)
            except (ImportError, KeyError) as e:
                logger.debug(f"Could not restore mosaic affine: {e}")

        logger.info("Calibration loaded from settings (v7.2.7)")
        self._emit_calibration_data_changed()

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

    # ── v7.3.3: Relaxed needle detection + interactive refinement ──

    def _toggle_needle_detect_relaxed(self, enabled: bool) -> None:
        """Toggle relaxed needle detection (wide tolerance)."""
        if not enabled:
            self._stop_detection()
            self._needle_relaxed_mode = False
            self._needle_interactive = False
            ar = getattr(self, '_needle_ar_frame', None)
            if ar is not None:
                ar.setVisible(False)
            return

        # Uncheck strict detect if on
        btn_strict = getattr(self, '_btn_detect_needle', None)
        if btn_strict is not None and btn_strict.isChecked():
            btn_strict.blockSignals(True)
            btn_strict.setChecked(False)
            btn_strict.blockSignals(False)

        cam = self._get_primary_camera()
        if cam is None or not cam.is_running:
            QMessageBox.warning(self, "Cannot Detect",
                                "Start a camera feed first.")
            btn = getattr(self, '_btn_detect_needle_relaxed', None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)
            return

        if not self._ensure_detection_worker():
            return

        self._needle_detecting = True
        self._needle_relaxed_mode = True
        self._needle_interactive = False
        self._last_needle_result = None

        # Use expected OD if available, else 0 for unconstrained
        od_px = self._get_expected_od_px() or 0.0

        self._detection_worker.set_mode(
            DetectionMode.NEEDLE_DETECT_RELAXED,
            expected_od_px=od_px,
        )

        overlay = cam.detection_overlay
        if overlay is not None:
            frame = cam.get_current_frame()
            if frame is not None:
                overlay.set_frame_size(frame.shape[1], frame.shape[0])
            overlay.set_info_text("Relaxed needle detection...")

        if not self._detection_worker.isRunning():
            self._detection_worker.start()

        lbl = getattr(self, '_lbl_needle_status', None)
        if lbl is not None:
            lbl.setText("Searching (relaxed tolerance)...")
            lbl.setStyleSheet(f"color: {COLORS['blue']}; font-size: 9pt;")

        logger.info(f"Relaxed needle detection started (expected OD: {od_px:.0f}px)")

    def _enter_interactive_needle_mode(self, result) -> None:
        """Switch from continuous detection to interactive accept/reject."""
        self._needle_interactive = True

        # Stop the detection worker (single-shot result)
        if self._detection_worker is not None:
            self._detection_worker.set_mode(DetectionMode.IDLE)

        cam = self._get_primary_camera()
        if cam is not None:
            overlay = cam.detection_overlay
            if overlay is not None:
                # Clear auto-detected circle, show adjustable one
                overlay.set_needle_detection(None)
                overlay.set_adjustable_needle(
                    result.center_px, result.radius_px)
                overlay.set_info_text("Verify needle — Accept or Reject")

        # Try to refine to edge
        if VISION_AVAILABLE and cam is not None:
            frame = cam.get_current_frame()
            if frame is not None:
                try:
                    from SupportClasses.VisionDetector import NeedleDetector
                    refined = NeedleDetector.refine_to_edge(
                        frame, result.center_px, result.radius_px)
                    if refined is not None:
                        result = refined
                        if cam is not None:
                            overlay = cam.detection_overlay
                            if overlay is not None:
                                overlay.set_adjustable_needle(
                                    refined.center_px, refined.radius_px)
                        logger.info(f"Needle refined to edge: r={refined.radius_px:.1f}px")
                except Exception as e:
                    logger.debug(f"Edge refinement failed: {e}")

        self._last_needle_result = result

        # Show accept/reject buttons
        ar = getattr(self, '_needle_ar_frame', None)
        if ar is not None:
            ar.setVisible(True)

        # Update status
        lbl = getattr(self, '_lbl_needle_status', None)
        if lbl is not None:
            od_px = result.radius_px * 2
            lbl.setText(
                f"Detected OD: {od_px:.0f} px  |  "
                f"Adjust with +/\u2212, then Accept or Reject")
            lbl.setStyleSheet(f"color: {COLORS['blue']}; font-size: 9pt;")

    def _adjust_needle_radius(self, delta_px: float) -> None:
        """Adjust the interactive needle circle radius."""
        cam = self._get_primary_camera()
        if cam is None:
            return
        overlay = cam.detection_overlay
        if overlay is not None:
            overlay.adjust_radius(delta_px)
            circle = overlay.get_adjustable_circle()
            if circle is not None:
                center, r = circle
                lbl = getattr(self, '_lbl_needle_status', None)
                if lbl is not None:
                    lbl.setText(
                        f"OD: {r * 2:.0f} px  |  "
                        f"Adjust with +/\u2212, then Accept or Reject")

    def _accept_needle_detection(self) -> None:
        """Accept the current needle detection result."""
        cam = self._get_primary_camera()
        overlay = cam.detection_overlay if cam else None

        # Get final circle from overlay (may have been adjusted)
        if overlay is not None:
            circle = overlay.get_adjustable_circle()
            if circle is not None:
                center, radius = circle
                # Update the stored result with adjusted values
                if self._last_needle_result is not None:
                    from SupportClasses.VisionDetector import DetectionResult
                    self._last_needle_result = DetectionResult(
                        center_px=center,
                        radius_px=radius,
                        confidence=self._last_needle_result.confidence,
                        method=self._last_needle_result.method + "_accepted",
                    )
            overlay.clear_adjustable()
            overlay.set_info_text("")

        # Hide accept/reject UI
        ar = getattr(self, '_needle_ar_frame', None)
        if ar is not None:
            ar.setVisible(False)

        self._needle_interactive = False
        self._needle_detecting = False
        self._needle_relaxed_mode = False

        # Uncheck the relaxed detect button
        for attr in ('_btn_detect_needle_relaxed', '_btn_detect_needle'):
            btn = getattr(self, attr, None)
            if btn is not None and btn.isChecked():
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)

        # If needle gauge is known, compute implied µm/px
        result = self._last_needle_result
        if result is not None:
            od_px = result.radius_px * 2
            hw_cfg = getattr(self, '_hardware_config', None)
            needle = hw_cfg.needle if hw_cfg else None
            if needle is not None and needle.od_um > 0:
                implied_umpx = needle.od_um / od_px
                lbl = getattr(self, '_lbl_needle_status', None)
                if lbl is not None:
                    lbl.setText(
                        f"Accepted — OD: {od_px:.0f} px  |  "
                        f"Known OD: {needle.od_um:.0f} µm ({needle.gauge}G)  |  "
                        f"Implied µm/px: {implied_umpx:.3f}")
                    lbl.setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 9pt;")

                # Emit signal to update hardware page
                cam_idx = getattr(self, '_primary_cam_idx', 0)
                self.um_per_px_calibrated.emit(cam_idx, implied_umpx)
                logger.info(
                    f"Needle-based µm/px calibration: {implied_umpx:.4f} "
                    f"(needle {needle.gauge}G OD={needle.od_um:.0f}µm, "
                    f"detected {od_px:.0f}px)")
            else:
                lbl = getattr(self, '_lbl_needle_status', None)
                if lbl is not None:
                    lbl.setText(f"Accepted — OD: {od_px:.0f} px")
                    lbl.setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 9pt;")

    def _reject_needle_detection(self) -> None:
        """Reject the current detection and re-enable detection."""
        cam = self._get_primary_camera()
        if cam is not None:
            overlay = cam.detection_overlay
            if overlay is not None:
                overlay.clear_adjustable()
                overlay.clear()

        ar = getattr(self, '_needle_ar_frame', None)
        if ar is not None:
            ar.setVisible(False)

        self._needle_interactive = False
        self._last_needle_result = None

        lbl = getattr(self, '_lbl_needle_status', None)
        if lbl is not None:
            lbl.setText("Rejected — click detect to retry")
            lbl.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")

        # Stop detection, uncheck buttons
        self._stop_detection()
        for attr in ('_btn_detect_needle_relaxed', '_btn_detect_needle'):
            btn = getattr(self, attr, None)
            if btn is not None and btn.isChecked():
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)

    def _on_needle_detected(self, result) -> None:
        """Slot: DetectionWorker found a needle tip."""
        if not self._needle_detecting:
            return

        self._last_needle_result = result

        # v7.3.3: In relaxed mode, enter interactive accept/reject
        if self._needle_relaxed_mode and not self._needle_interactive:
            self._enter_interactive_needle_mode(result)
            return

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

