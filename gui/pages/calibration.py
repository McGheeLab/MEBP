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
import time
import threading

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox,
    QFrame, QSizePolicy, QMessageBox, QCheckBox, QSlider,
    QScrollArea, QTabWidget, QSplitter, QListWidget, QListWidgetItem,
)
from PySide6.QtCore import Qt, Signal, QTimer, QThread, QObject
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QPixmap

from SupportClasses.StageController import StageController, plate_relative_to_zref
from SupportClasses.WellPlate import WellPlate, PLATE_DEFINITIONS
from gui.styles import COLORS, SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE
from gui.unit_helpers import stage_to_um, format_um, DEFAULT_XY_POSITION_SCALE
from gui.scaling import s, sf, sp, scaled_font_size

try:
    from SupportClasses.HardwareConfig import HardwareConfig, CameraConfig, CameraRole
except ImportError:
    HardwareConfig = None
    CameraConfig = None
    CameraRole = None

# v7.4.4: Two-camera needle aligner + edge-fit well locator
try:
    from SupportClasses.VisionDetector import (
        TwoCameraNeedleAligner, TwoCameraEdgePicks, EdgeFitWellLocator,
        select_well_fit_strategy, detect_rim_point_near_center,
        fit_circle_to_points,
    )
    NEEDLE_LOCATION_AVAILABLE = True
except ImportError:
    NEEDLE_LOCATION_AVAILABLE = False
    TwoCameraNeedleAligner = None
    TwoCameraEdgePicks = None
    EdgeFitWellLocator = None
    select_well_fit_strategy = None
    detect_rim_point_near_center = None
    fit_circle_to_points = None

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
        wmap = {w.name: w for w in wells}
        for well in wells:
            cx, cy, r = well.x * S, well.y * S, self.WELL_R
            self._scene.addEllipse(
                cx - r, cy - r, 2 * r, 2 * r,
                QPen(QColor("#585b70"), 0.5),
                QBrush(QColor("#45475a")),
            ).setToolTip(well.name)

        # v7.5.x: predicted/calibrated dots are drawn ON each well's PLATE-LOCAL
        # grid cell BY NAME (A1 top-left, +col right, +row down — same frame as
        # the grey grid + the A1/corner highlights below). Previously they were
        # positioned in absolute µm relative to ``_taught_a1``, which (a) had a
        # latent µm/mm unit mix and (b) anchored on a value that can desync from
        # the calibrated A1 (e.g. after a mosaic re-label) — placing the WRONG
        # well's dot at the origin/top-left. By-name guarantees the dot label
        # overlays its own cell regardless of taught_a1 / plate orientation.
        if self._predicted_positions:
            pred_pen = QPen(QColor("#f9e2af"), 0.5)
            pred_brush = QBrush(QColor("#f9e2af80"))
            pr = 1.8  # small dot radius
            for wname in self._predicted_positions:
                w = wmap.get(wname)
                if w is None:
                    continue
                dot = self._scene.addEllipse(
                    w.x * S - pr, w.y * S - pr, 2 * pr, 2 * pr,
                    pred_pen, pred_brush,
                )
                dot.setToolTip(f"{wname} (predicted)")
                dot.setZValue(5)

        if self._calibrated_positions:
            cal_pen = QPen(QColor("#a6e3a1"), 0.8)
            cal_brush = QBrush(QColor("#a6e3a1a0"))
            cr = 2.0
            for wname in self._calibrated_positions:
                w = wmap.get(wname)
                if w is None:
                    continue
                dot = self._scene.addEllipse(
                    w.x * S - cr, w.y * S - cr, 2 * cr, 2 * cr,
                    cal_pen, cal_brush,
                )
                dot.setToolTip(f"{wname} (calibrated)")
                dot.setZValue(6)

        # Highlight A1, corner well, and calibration reference wells
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


class _MosaicScanWorker(QThread):
    """v7.5.x: runs the full-plate mosaic raster on a BACKGROUND thread.

    The camera's display grab timer keeps running on the GUI thread (the
    "image grabber"); this worker only *commands* stage moves (the
    ``StageController`` motion API is thread-safe — the print executors already
    drive it from worker threads) and *samples* frames via the camera's
    thread-safe ``frame_count_value()`` / ``get_current_frame()`` — it never
    touches the camera backend directly, so there is no race with the grabber.
    All frame *processing* (stitch + detect) happens here, off the GUI thread,
    so the UI no longer freezes during a scan.

    Signals (all delivered queued → GUI-thread slots):
        progress(done, total)
        tile(composite_bgr_copy, extent_tuple)         — live preview, per tile
        finished_ok(composite, extent, scale, frames, detections)
            detections = list of (px, py, radius_px) on the final mosaic
        failed(message)
    """

    progress = Signal(int, int)
    tile = Signal(object, object)
    finished_ok = Signal(object, object, float, int, object)
    failed = Signal(str)

    def __init__(self, controller, cam, builder, positions, safe_z,
                 expected_d_px, min_dist_px, fresh_frames=3,
                 fresh_timeout_s=2.5, settle_ms=0,
                 detect_param2=30.0, detect_tolerance=0.35,
                 frame_orient="none", parent=None):
        super().__init__(parent)
        self._controller = controller
        self._cam = cam
        self._builder = builder
        self._positions = list(positions)
        self._safe_z = safe_z
        self._expected_d_px = float(expected_d_px)
        self._min_dist_px = float(min_dist_px)
        self._fresh_frames = int(fresh_frames)
        self._fresh_timeout_s = float(fresh_timeout_s)
        # Extra settle (ms) after each move before counting fresh frames — gives
        # the camera time to finish exposing a sharp, post-move frame.
        self._settle_ms = max(0, int(settle_ms))
        self._detect_param2 = float(detect_param2)
        self._detect_tolerance = float(detect_tolerance)
        # Per-tile transform to align the camera frame with the stage axes when
        # the camera is mounted rotated/mirrored (else neighbouring tiles don't
        # line up). "none" | "rot180" | "fliph" | "flipv".
        self._frame_orient = str(frame_orient or "none")
        self._stop = False

    def stop(self):
        self._stop = True

    def _orient_frame(self, frame):
        """Apply the configured camera-mount transform to a captured frame."""
        if frame is None or self._frame_orient == "none":
            return frame
        try:
            import cv2
            if self._frame_orient == "rot180":
                return cv2.rotate(frame, cv2.ROTATE_180)
            if self._frame_orient == "fliph":
                return cv2.flip(frame, 1)
            if self._frame_orient == "flipv":
                return cv2.flip(frame, 0)
        except Exception:
            pass
        return frame

    def _grab_post_move_frame(self):
        """Give the camera time to settle, then wait for the GUI grab timer to
        deliver N fresh frames (draining any buffered backlog so the frame is
        post-move) and return the latest."""
        cam = self._cam
        # Settle first: lets the exposure window / callback advance past the
        # move so the frames we then count are genuinely post-move + sharp.
        if self._settle_ms > 0 and not self._stop:
            time.sleep(self._settle_ms / 1000.0)
        try:
            c0 = cam.frame_count_value()
        except Exception:
            c0 = None
        if c0 is not None:
            t_end = time.time() + self._fresh_timeout_s
            while time.time() < t_end and not self._stop:
                try:
                    if cam.frame_count_value() - c0 >= self._fresh_frames:
                        break
                except Exception:
                    break
                time.sleep(0.02)
        try:
            return cam.get_current_frame()
        except Exception:
            return None

    # Abort the scan if this many consecutive tiles yield no fresh frame
    # (camera stopped / crashed) rather than silently building a partial mosaic.
    _MAX_CONSEC_NONE = 8

    def _suspend_poller(self):
        try:
            self._controller.suspend_position_poller()
        except Exception:
            pass

    def _resume_poller(self):
        try:
            self._controller.resume_position_poller()
        except Exception:
            pass

    def run(self):
        # Suspend the background position poller for the whole raster: its
        # periodic ZP M114 reads otherwise contend with stage I/O on the shared
        # serial buses over a long scan and trip the ZP write-timeout / liveness
        # watchdogs. Paired with _resume_poller() in finally.
        self._suspend_poller()
        try:
            total = len(self._positions)
            consecutive_none = 0
            for idx, (tx, ty) in enumerate(self._positions):
                if self._stop:
                    break
                try:
                    if idx == 0:
                        # First tile: full safe travel — retract Z to safe AND
                        # confirm, then XY. Establishes the constant safe Z (the
                        # page also pre-retracts before the worker starts).
                        self._controller.safe_travel_to(
                            tx, ty, safe_z_mm=self._safe_z, target_z_mm=None)
                    else:
                        # Z is already at safe and never changes during the
                        # raster (the microscope images from overhead), so do a
                        # PURE XY move — NO per-tile ZP/Marlin traffic. Calling
                        # safe_travel_to every tile re-confirms Z on the ZP board
                        # ~once per tile; over a long raster that contends with
                        # the poller and causes the ZP "Write timeout" that
                        # stopped the build. A plain XY move touches only the
                        # ProScan (XY) controller.
                        self._controller.move_xy_absolute_um(tx, ty)
                        try:
                            zero = self._controller.zero_position
                            self._controller.wait_for_xy_arrival(
                                (tx - float(zero.get("x", 0.0))) / 1000.0,
                                (ty - float(zero.get("y", 0.0))) / 1000.0)
                        except Exception:
                            pass
                except Exception as e:
                    logger.warning(
                        f"Mosaic worker: move to ({tx:.0f},{ty:.0f}) failed: {e}")
                if self._stop:
                    break
                frame = self._grab_post_move_frame()
                if frame is None:
                    # No fresh frame (camera stalled / timed out). Don't silently
                    # build a partial mosaic — warn, and abort on a sustained run.
                    consecutive_none += 1
                    logger.warning(
                        f"Mosaic worker: no fresh frame at tile {idx} "
                        f"({consecutive_none} consecutive)")
                    if consecutive_none >= self._MAX_CONSEC_NONE:
                        self.failed.emit(
                            f"camera stopped delivering frames "
                            f"({consecutive_none} tiles in a row) — scan aborted")
                        return
                    self.progress.emit(idx + 1, total)
                    continue
                consecutive_none = 0
                try:
                    xy = self._controller.get_xy_position(cached=False)
                    sx = xy[0] if xy and xy[0] is not None else tx
                    sy = xy[1] if xy and xy[1] is not None else ty
                except Exception:
                    sx, sy = tx, ty
                frame = self._orient_frame(frame)
                self._builder.add_raster_frame(frame, sx, sy, index=idx)
                self._builder.stitch_incremental()
                comp = self._builder.composite
                self.tile.emit(
                    comp.copy() if comp is not None else None,
                    self._builder.canvas_extent_um)
                self.progress.emit(idx + 1, total)

            if self._stop:
                # Cancelled — the GUI already cleaned up; emit nothing.
                return

            # Aggregate the per-overlap measurements into ONE global alignment
            # shift (median, bounded) applied uniformly via canvas_extent_um —
            # featureless tiles contributed nothing and are unaffected.
            try:
                self._builder.finalize_global_shift()
            except Exception as e:
                logger.debug(f"Mosaic global shift finalize skipped: {e}")

            # Detect on the INCREMENTAL composite (placed via _mosaic_scale /
            # _canvas_origin_um) — it is coordinate-faithful to canvas_extent_um.
            # NOTE: build_mosaic()'s phase-correlation branch recomputes a LOCAL
            # origin/scale it never writes back, which would desync detection
            # pixels from canvas_extent_um (shift ≈ half-FOV + wrong scale).
            # Position-based placement is reliable here given accurate encoders.
            composite = self._builder.composite
            extent = self._builder.canvas_extent_um
            scale = float(getattr(self._builder, "_mosaic_scale", 0.0) or 0.0)
            frames = self._builder.frame_count
            detections = []
            if composite is not None:
                try:
                    from SupportClasses.VisionDetector import WellDetector
                    # Robust filled-disc detection first; Hough as fallback.
                    dets = WellDetector.detect_filled_wells(composite)
                    if len(dets) < 3 and self._expected_d_px > 0:
                        dets = WellDetector.detect_wells(
                            composite, self._expected_d_px,
                            tolerance=self._detect_tolerance,
                            min_dist_px=self._min_dist_px,
                            param2=self._detect_param2)
                    detections = [
                        (float(d.center_px[0]), float(d.center_px[1]),
                         float(d.radius_px)) for d in dets]
                except Exception as e:
                    logger.warning(f"Mosaic worker: detection failed: {e}")
            # v7.5.x: detection + the final composite copy are done — release the
            # float64 accumulators (~190 MB on a full-plate canvas), keeping only
            # the uint8 display cache. Frees the bulk of the scan's RAM before the
            # overlay/save step (which fixes the post-build lag from memory
            # pressure). Best-effort — never block the result.
            try:
                self._builder.free_accumulators()
            except Exception:
                pass
            self.finished_ok.emit(
                composite.copy() if composite is not None else None,
                extent, scale, frames, detections)
        except Exception as e:
            logger.exception("Mosaic worker crashed")
            self.failed.emit(str(e))
        finally:
            self._resume_poller()


class _SubPlate:
    """Minimal WellPlate-like view over the flattened rosette SUB-WELLS of one
    parent well, so the mosaic well-mapping dialog can map a single rosette well
    (per-well mosaic). Exposes only what the dialog uses. rows/cols = 0 so the
    dialog's grid auto-detect bails to the 3-corner affine method (rosette
    sub-well patterns aren't a regular grid)."""

    def __init__(self, plate, parent_well):
        self._subs = [
            w for w in plate.get_all_wells()
            if getattr(w, "is_subwell", False)
            and getattr(w, "parent_well", None) == parent_well]
        self.format = f"{getattr(plate, 'format', 'plate')}#{parent_well}"
        self.rows = 0
        self.cols = 0
        self.well_diameter = 0.0

    @property
    def well_names(self):
        return [w.name for w in self._subs]

    def get_all_wells(self):
        return list(self._subs)

    def get_well_position(self, name):
        for w in self._subs:
            if w.name == name:
                return (w.x, w.y)
        raise KeyError(name)

    def get_well_info(self, name):
        for w in self._subs:
            if w.name == name:
                return w
        raise KeyError(name)


class _SingleWell:
    """Minimal WellPlate-like view over ONE plain (non-rosette) well, so the
    mosaic well-mapping dialog can pick a single well's centre on its per-well
    mosaic. Mirrors ``_SubPlate``'s surface; rows/cols = 0 keeps the dialog off
    the grid paths."""

    def __init__(self, plate, well_name):
        self._w = [w for w in plate.get_all_wells() if w.name == well_name]
        self.format = f"{getattr(plate, 'format', 'plate')}#{well_name}"
        self.rows = 0
        self.cols = 0
        # mm, matching WellPlate.well_diameter semantics.
        self.well_diameter = (
            float(getattr(self._w[0], "diameter", 0.0) or 0.0)
            if self._w else 0.0)

    @property
    def well_names(self):
        return [w.name for w in self._w]

    def get_all_wells(self):
        return list(self._w)

    def get_well_position(self, name):
        for w in self._w:
            if w.name == name:
                return (w.x, w.y)
        raise KeyError(name)

    def get_well_info(self, name):
        for w in self._w:
            if w.name == name:
                return w
        raise KeyError(name)


class _AutoReanchorWorker(QThread):
    """v7.5.x: hands-free "Auto re-anchor mosaic" — travels to the saved
    feature's last known (map-frame) position, grabs a fresh live frame,
    template-matches the saved feature patch in it, and reports the feature's
    ACTUAL absolute stage position. The GUI thread then applies
    ``E = actual − stored`` as a global map translation.

    Runs off the GUI thread (safe-travel blocks for the Z-retract confirm).
    Only thread-safe surfaces are touched: ``StageController`` motion API,
    ``CameraWidget.capture_fresh_frame``, and the camera manager's pure-math
    µm/px + pixel→stage converters.

    Signals (queued → GUI thread):
        finished_ok(actual_x_um, actual_y_um, confidence)
        failed(message)
    """

    finished_ok = Signal(float, float, float)
    failed = Signal(str)

    def __init__(self, controller, cam, cam_mgr, cam_idx,
                 target_um, safe_z, patch, stored_um_per_px,
                 min_confidence=0.5, settle_ms=400, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._cam = cam
        self._cam_mgr = cam_mgr
        self._cam_idx = int(cam_idx)
        self._target_um = (float(target_um[0]), float(target_um[1]))
        self._safe_z = safe_z
        self._patch = patch
        self._stored_upp = float(stored_um_per_px or 0.0)
        self._min_conf = float(min_confidence)
        self._settle_ms = max(0, int(settle_ms))

    def _suspend_poller(self):
        try:
            self._controller.suspend_position_poller()
        except Exception:
            pass

    def _resume_poller(self):
        try:
            self._controller.resume_position_poller()
        except Exception:
            pass

    def run(self):   # pragma: no cover - thread body; logic tested via _match
        self._suspend_poller()
        try:
            ok = True
            try:
                ok = self._controller.safe_travel_to(
                    self._target_um[0], self._target_um[1],
                    safe_z_mm=self._safe_z, target_z_mm=None)
            except Exception as e:
                self.failed.emit(f"travel failed: {e}")
                return
            if not ok:
                self.failed.emit(
                    "could not confirm the retract/travel to the feature's "
                    "position — check the stage and retry.")
                return
            frame = None
            try:
                frame = self._cam.capture_fresh_frame(
                    discard_n_frames=3, settle_ms=self._settle_ms)
            except Exception:
                frame = None
            if frame is None:
                self.failed.emit("no live frame from the microscope camera.")
                return
            res = self._match(frame)
            if res is None:
                self.failed.emit(
                    "feature not found in view — jog it into the live view "
                    "and re-run the manual re-anchor to re-save it.")
                return
            ax, ay, conf = res
            self.finished_ok.emit(ax, ay, conf)
        except Exception as e:
            logger.exception("Auto re-anchor worker crashed")
            self.failed.emit(str(e))
        finally:
            self._resume_poller()

    def _match(self, frame):
        """Template-match the stored patch in ``frame`` → the feature's
        ACTUAL absolute stage µm + confidence, or None below the confidence
        gate. Pure math — unit-testable without a thread."""
        import cv2
        from SupportClasses.VisionDetector import find_template
        fh, fw = frame.shape[:2]
        patch = self._patch
        # Rescale the patch when the live µm/px differs from capture (>2%).
        try:
            live_upp = float(self._cam_mgr.effective_um_per_px(
                self._cam_idx, fw) or 0.0)
        except Exception:
            live_upp = 0.0
        if self._stored_upp > 0 and live_upp > 0:
            ratio = self._stored_upp / live_upp
            if abs(ratio - 1.0) > 0.02:
                nw = max(8, int(round(patch.shape[1] * ratio)))
                nh = max(8, int(round(patch.shape[0] * ratio)))
                try:
                    patch = cv2.resize(patch, (nw, nh),
                                       interpolation=cv2.INTER_AREA)
                except Exception:
                    pass
        res = find_template(frame, patch)
        if res is None or res[2] < self._min_conf:
            if res is not None:
                logger.info(
                    f"Auto re-anchor: match confidence {res[2]:.2f} below "
                    f"{self._min_conf:.2f} — rejected.")
            return None
        cx, cy, conf = res
        # Rotation-aware pixel → stage offset (same mapping as a live click).
        dx_um, dy_um = self._cam_mgr.pixel_to_stage_offset(
            self._cam_idx, cx, cy, fw, fh)
        xy = self._controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            return None
        return (float(xy[0]) + dx_um, float(xy[1]) + dy_um, float(conf))


class _CompCalBridge(QObject):
    """Thread → GUI bridge for the pump compliance calibration. The pump step
    runs on a daemon thread so the live needle cameras don't freeze; the result
    is marshalled back to the GUI thread via this queued signal."""
    done = Signal(bool, str)


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
        # v7.5.x: taught reference well centres (absolute stage µm) kept as
        # permanent registration markers on the plate + microscope views.
        self._reference_markers: dict[str, tuple[float, float]] = {}
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
        # v7.4.2: right-side context panel holds the Camera / Needle /
        # Plate tabs that used to live in the left context. Built
        # lazily by ``get_right_context_widget``.
        self._right_context_widget = None
        # The original ``get_context_widget`` body now becomes the
        # widget mounted into the right tabbed panel; this flag lets
        # us reroute the addWidget calls inside the existing helpers
        # so we don't have to refactor every callsite.
        self._building_right_context = False

        # v7.1.1: Microsteps-to-microns conversion factor
        self._xy_position_scale = DEFAULT_XY_POSITION_SCALE
        self._hardware_config = None  # v7.2: HardwareConfig
        # v7.5.x: the plate key the in-memory calibration currently belongs to
        # (set by _activate_calibration_for_key). Taught calibration is archived
        # per active_plate_key (settings.calibration_by_plate) so each plate
        # TYPE keeps its own map; a plate switch flushes the outgoing key and
        # restores the incoming one. ``None`` until the first plate is active.
        self._loaded_cal_key: str | None = None

        # v7.3.1: Geometry-predicted positions (well_name → (x_um, y_um) absolute)
        self._predicted_positions: dict[str, tuple[float, float]] | None = None
        # v7.3.1: Calibrated positions after mosaic scan (affine-corrected)
        self._calibrated_positions: dict[str, tuple[float, float]] | None = None
        # v7.5.x: interpolating warp (affine base + TPS residual) fitted from
        # the freeform / manual-teach control points; source of truth for
        # _calibrated_positions when set. None ⇒ no freeform warp active.
        self._plate_warp = None  # SupportClasses.PlateWarpCalibrator | None
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

        # v7.5.x: polarity-general sweep state. The coarse/fine sweep now
        # walks a *height above a reference bottom* ``h`` (mm, + = away from
        # the plate / safe side), converting each ``h`` to a zero-ref Z via
        # ``plate_relative_to_zref``. "Toward the plate" = decreasing ``h``,
        # so this is correct on both a conventional machine (ZDIR=+1) and
        # ME3B V1 (ZDIR=-1, needle descends as zero-ref Z increases).
        self._auto_z_ref_z: float = 0.0       # reference bottom (zero-ref mm)
        self._auto_z_h: float = 0.0           # current height above ref (mm)
        self._auto_z_floor_h: float = 0.0     # lowest allowed height (mm)
        self._auto_z_best_h: float | None = None

        # v7.5.x: guided per-well Z-bottom calibration. Each calibration well
        # is taught the same way: travel there, the operator refocuses the
        # microscope on the glass + confirms, then the needle lowers and the
        # best-focus Z is recorded (auto-detect peak, with manual override).
        # The descent window is seeded from the previous accepted Z (else
        # Plate Bottom Z, else ``top_z − well_depth``) so it tracks plate tilt.
        self._auto_z_pending_z: float | None = None   # provisional Z awaiting Accept
        self._auto_z_last_z: float | None = None      # last accepted Z (window seed)
        self._zauto_approach_margin: float = 1.0      # mm above ref to start descent
        self._zauto_tilt_margin: float = 0.5          # mm below ref allowed (floor)

        # v7.2.7-hotfix: ensure all calibration attrs

        for _a in ['_safe_z','_top_z','_taught_a1_z','_taught_corner_z',

                  '_taught_third','_taught_third_z','_third_well',

                  '_z_plane_result',

                  # v7.4.4: new Z reference heights for the Needle
                  # Offset Calibration tab.
                  '_replace_z','_max_z','_plate_bottom_z']:

            if not hasattr(self, _a): setattr(self, _a, None)

        if not hasattr(self, '_z_buffer_mm'): self._z_buffer_mm = 0.5

        if not hasattr(self, '_corner_well'): self._corner_well = 'H12'

        # v7.4.0-a: Debounced auto-save. _emit_calibration_data_changed fires
        # from many places (safe Z teach, top Z teach, 3-point teach, mosaic
        # tick). Writing settings.json on each emit would thrash disk during
        # mosaic scans, so coalesce into a single write 500 ms after the last
        # change.
        self._autosave_timer = QTimer(self)
        self._autosave_timer.setSingleShot(True)
        self._autosave_timer.setInterval(500)
        self._autosave_timer.timeout.connect(self._save_calibration)

        self._setup_ui()

    def get_page_title(self) -> str:
        return "Calibration"

    def get_calibration_data(self):
        """v7.3.1: Return (plate, well_positions, safe_z) for jog page fast-travel.

        Returns calibrated positions if available, otherwise predicted positions.
        """
        positions = self._calibrated_positions or self._predicted_positions
        return self._plate, positions, getattr(self, '_safe_z', None)

    def _has_plate_calibration(self) -> bool:
        """v7.5.x: True when the page holds a *real* plate calibration (taught
        A1, a freeform warp, or a non-identity 3-well affine). This is the
        keystone of a usable calibration — used to (a) gate the durable
        last-known snapshot so an empty/cleared page never overwrites a good
        one, and (b) decide at startup whether to offer a restore (no prompt
        when calibration survived)."""
        if getattr(self, '_taught_a1', None) is not None:
            return True
        if getattr(self, '_plate_warp', None) is not None:
            return True
        # v7.5.x: explicit ground-truth positions (re-derived from the mosaic,
        # stored directly with no warp) are a real calibration too.
        if getattr(self, '_calibrated_positions', None):
            return True
        twc = getattr(self, '_three_well_calibration', None)
        return twc is not None and not getattr(twc, 'is_identity', True)

    def _plate_axis_sign(self) -> tuple[float, float]:
        """v7.5.x: per-machine plate-local→stage axis sign (single source of
        truth on the controller). Used for every GEOMETRIC well prediction so
        non-taught wells land on the physically-correct side of A1. Defaults to
        ``(1, 1)`` when the controller is absent / older."""
        ctrl = self.controller
        if ctrl is not None and hasattr(ctrl, "plate_axis_sign"):
            try:
                sign = ctrl.plate_axis_sign()
                return (float(sign[0]), float(sign[1]))
            except Exception:
                pass
        return (1.0, 1.0)

    def _orientation_stamp(self) -> bool | None:
        """v7.5.x: the current plate orientation as a plain bool for stamping a
        saved calibration (JSON-serialisable; None when unknown). The load-side
        guard discards stale taught XY calibration when this changes."""
        ctrl = self.controller
        if ctrl is not None and hasattr(ctrl, "plate_flip_180"):
            try:
                return bool(ctrl.plate_flip_180())
            except Exception:
                pass
        return None

    # ── v7.5.x: per-plate-key calibration archive ──────────────────
    #
    # The taught calibration (XY wells / warp / Z references / Z-plane /
    # reference markers) belongs to a SPECIFIC plate identity. It is archived
    # per ``active_plate_key`` so each plate TYPE (Corning glass-bottom vs NEST
    # plastic vs generic) keeps its own map, and selecting a plate brings its
    # calibration back. The live ``settings.calibration`` section ALWAYS mirrors
    # the *active* plate's flat block (so every existing flat reader keeps
    # working unchanged); ``settings.calibration_by_plate`` is the keyed archive.

    def _calibration_key(self) -> str | None:
        """Identity used to key the taught calibration (mirrors
        ``_ploc_plate_key``): ``active_plate_key`` → ``plate.format`` → None."""
        hw = getattr(self, "_hardware_config", None)
        key = getattr(hw, "active_plate_key", None) if hw is not None else None
        if key is None:
            key = (getattr(self._plate, "format", None)
                   if getattr(self, "_plate", None) is not None else None)
        return str(key) if key is not None else None

    def _calibration_archive(self) -> dict:
        """Return the per-plate calibration archive ``{key: cal_data}``.

        One-time migration: if the archive is empty/absent but a LEGACY flat
        ``settings.calibration`` block exists, seed it under its
        ``plate_format`` (so the existing single calibration is preserved and
        attached to the format it was taught for). Returns a dict the caller
        mutates then writes back via ``set_section('calibration_by_plate', …)``.
        """
        if self.settings is None:
            return {}
        archive = self.settings.get_section("calibration_by_plate")
        if isinstance(archive, dict) and archive:
            return dict(archive)
        archive = {}
        flat = self.settings.get_section("calibration")
        if isinstance(flat, dict) and flat and (
                flat.get("plate_format") is not None
                or any(k in flat for k in (
                    "taught_a1", "safe_z", "top_z", "plate_warp",
                    "calibrated_positions", "mosaic_affine"))):
            legacy_key = flat.get("plate_format")
            legacy_key = str(legacy_key) if legacy_key is not None else "legacy"
            archive[legacy_key] = dict(flat)
            self.settings.set_section("calibration_by_plate", archive)
            logger.info(
                "Calibration: migrated legacy flat block into "
                "calibration_by_plate['%s']", legacy_key)
        return archive

    def _reset_calibration_state(self) -> None:
        """Clear the in-memory taught calibration + its labels to a blank,
        un-calibrated state. Called on a plate switch BEFORE restoring the
        incoming plate's archived block (so a plate with no stored calibration
        shows blank instead of bleeding the previous plate's data)."""
        self._taught_a1 = None
        self._taught_corner = None
        self._taught_a1_z = None
        self._taught_corner_z = None
        if hasattr(self, "_taught_third"):
            self._taught_third = None
        self._taught_third_z = None
        self._third_well = None
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._rotation = 0.0
        self._scale = 1.0
        self._safe_z = None
        self._top_z = None
        self._replace_z = None
        self._max_z = None
        self._plate_bottom_z = None
        self._three_well_calibration = None
        self._plate_warp = None
        self._calibrated_positions = None
        self._reference_markers = {}
        self._z_plane_result = None
        if hasattr(self, "_xy_teach_points"):
            self._xy_teach_points.clear()
        if hasattr(self, "_z_teach_points"):
            self._z_teach_points.clear()
        # v7.5.x: single-well re-registration anchors belong to the outgoing
        # plate's map frame — never let them survive a plate switch.
        try:
            self._ploc_clear_well_anchors()
        except Exception:
            pass
        # Reset the visible labels so a stale value never lingers.
        for attr in ("lbl_a1", "lbl_corner", "lbl_third"):
            if hasattr(self, attr):
                getattr(self, attr).setText("—")
        if hasattr(self, "lbl_alignment"):
            self.lbl_alignment.setText("")
        for lbl_attr, prefix in (
                ("lbl_safe_z", "Safe Z"), ("lbl_top_z", "Top Z"),
                ("_zoff_lbl_safe_z", "Fast Move Z"),
                ("_zoff_lbl_top_z", "Plate Top Z"),
                ("_zoff_lbl_replace_z", "Replace Z"),
                ("_zoff_lbl_max_z", "Max Z"),
                ("_zoff_lbl_plate_bottom_z", "Plate Bottom Z")):
            lbl = getattr(self, lbl_attr, None)
            if lbl is not None:
                lbl.setText(f"{prefix}: —" if lbl_attr.startswith("_zoff")
                            else "Not set")
                lbl.setStyleSheet(f"color: {COLORS['subtext0']};")

    def _activate_calibration_for_key(self, new_key) -> None:
        """Make ``new_key`` the active plate's calibration: load its archived
        block into the live ``settings.calibration`` flat section + memory, or
        reset to blank when none is stored. Updates ``_loaded_cal_key``."""
        if self.settings is None:
            self._loaded_cal_key = str(new_key) if new_key is not None else None
            return
        archive = self._calibration_archive()
        block = archive.get(str(new_key)) if new_key is not None else None
        # The flat section always mirrors the ACTIVE plate (keeps every flat
        # reader correct). Empty when the plate has no stored calibration.
        self.settings.set_section("calibration", dict(block) if block else {})
        self._reset_calibration_state()
        self._loaded_cal_key = str(new_key) if new_key is not None else None
        if block:
            self._load_calibration()

    def _wells_in_zero_ref(self) -> dict:
        """v7.4.4: stage-µm well positions converted to zero-ref µm for
        ``JogWorkspaceView``. Falls back to plate-geometry-only
        predictions when no calibration has happened yet."""
        positions = self._calibrated_positions or self._predicted_positions
        if not positions:
            # Geometry-only: centre the plate on the XY safety envelope.
            if self._plate is None or self.controller is None:
                return {}
            zero = self.controller.zero_position
            zx = float(zero.get("x", 0.0))
            zy = float(zero.get("y", 0.0))
            try:
                # v7.5.x: seed at the envelope centre (absolute stage µm) …
                cx, cy = self.controller.default_plate_center_um()
                approx = self._plate.get_all_positions_from_plate_center(
                    cx, cy, self._plate_axis_sign())
            except Exception:
                return {}
            # … then convert to zero-ref µm by subtracting zero_position (NOT
            # the centre), so the plate lands at the envelope centre on the
            # canvas rather than pinned to (0, 0).
            return {
                name: (wx - zx, wy - zy)
                for name, (wx, wy) in approx.items()
            }
        if self.controller is None:
            return {}
        zero = self.controller.zero_position
        zx = float(zero.get("x", 0.0))
        zy = float(zero.get("y", 0.0))
        return {
            name: (wx - zx, wy - zy)
            for name, (wx, wy) in positions.items()
        }

    def recenter_default_plate(self) -> None:
        """v7.5.x: re-seed the uncalibrated plate prediction on the current
        XY safety-envelope centre and refresh views.

        Called when the user saves new safety limits so the plate re-centres
        live (without a restart). No-op once a real calibration exists — the
        gate protects taught/affine-fitted positions from being clobbered.

        The gate also covers a solved-but-not-yet-accepted 3-well/manual fit
        (``_three_well_calibration`` set before ``_calibrated_positions`` /
        ``_taught_a1``): re-seeding ``_predicted_positions`` would move the grid
        the pending affine was fitted against, silently miscalibrating on Accept.
        """
        if (self._calibrated_positions is not None
                or self._taught_a1 is not None
                or getattr(self, "_three_well_calibration", None) is not None):
            return
        if self._plate is None or self.controller is None:
            return
        try:
            cx, cy = self.controller.default_plate_center_um()
            self._predicted_positions = (
                self._plate.get_all_positions_from_plate_center(
                    cx, cy, self._plate_axis_sign()))
        except Exception as e:
            logger.debug(f"recenter_default_plate skipped: {e}")
            return
        # Refreshes the Plate-Location view and pushes to the Jog page /
        # Workflows mode via the calibration_data_changed signal.
        self._emit_calibration_data_changed()

    def get_z_references(self) -> dict:
        """v7.4.4: Return the five captured Z reference heights for
        consumers outside the calibration page.

        Keys (all values are mm, zero-referenced; missing = ``None``):

        * ``replace_z``       — needle-swap clearance (highest)
        * ``max_z``           — soft-limit ceiling
        * ``fast_move_z``     — fast XY travel height (legacy ``safe_z``)
        * ``plate_top_z``     — plate's top surface (legacy ``top_z``)
        * ``plate_bottom_z``  — well floor (lowest)
        """
        return {
            "replace_z":      getattr(self, "_replace_z", None),
            "max_z":          getattr(self, "_max_z", None),
            "fast_move_z":    getattr(self, "_safe_z", None),
            "plate_top_z":    getattr(self, "_top_z", None),
            "plate_bottom_z": getattr(self, "_plate_bottom_z", None),
        }

    def _reference_in_zero_ref(self) -> dict:
        """Taught reference markers (absolute stage µm) → zero-ref µm for the
        plate view (same frame as ``_wells_in_zero_ref``)."""
        if not self._reference_markers or self.controller is None:
            return {}
        zero = self.controller.zero_position
        zx = float(zero.get("x", 0.0))
        zy = float(zero.get("y", 0.0))
        return {n: (x - zx, y - zy)
                for n, (x, y) in self._reference_markers.items()}

    def _refresh_ploc_view(self) -> None:
        """v7.4.4: push the latest plate + well positions into the
        shared XY workspace view on the Plate Location tab. No-op
        when the view hasn't been built yet."""
        view = getattr(self, "_ploc_plate_view", None)
        if view is None:
            return
        if self._plate is not None and hasattr(view, "set_plate"):
            view.set_plate(self._plate)
        if hasattr(view, "set_well_positions"):
            kind = "calibrated" if self._calibrated_positions else "approximate"
            view.set_well_positions(self._wells_in_zero_ref(), kind)
        if hasattr(view, "set_safety_limits"):
            sl = getattr(self.controller, "safety_limits", None) if self.controller else None
            if sl is not None:
                view.set_safety_limits(sl)
        hw = getattr(self, "_hardware_config", None)
        needle = getattr(hw, "needle", None) if hw is not None else None
        od = getattr(needle, "od_um", None) if needle is not None else None
        if od and hasattr(view, "set_needle"):
            view.set_needle(float(od))
        # v7.5.x: keep the live microscope view bound to the Microscope slot
        # so its feed is shown whenever that camera is running (guarded so we
        # only rebind on an actual change).
        live = getattr(self, "_ploc_live_view", None)
        if (live is not None and hw is not None
                and self._camera_manager is not None and CameraRole is not None):
            try:
                mi = hw.camera_for_role(CameraRole.MICROSCOPE)
            except Exception:
                mi = None
            if mi is not None and mi != self._ploc_live_cam_idx:
                self._ploc_live_cam_idx = mi
                try:
                    live.set_camera(mi)
                except Exception as e:
                    logger.debug(f"PlateLocation: live view rebind failed: {e}")
                # v7.5.x: keep the Plate Z Auto-Cal tab's live view on the
                # same microscope slot.
                zoff_live = getattr(self, "_zoff_live_view", None)
                if zoff_live is not None:
                    try:
                        zoff_live.set_camera(mi)
                    except Exception as e:
                        logger.debug(
                            f"PlateZAutoCal: live view rebind failed: {e}")

        # v7.5.x: push the permanent reference markers (taught well centres)
        # to the plate view (zero-ref µm) and the live microscope overlay
        # (absolute µm + current µm/px + stage centre).
        if hasattr(view, "set_reference_markers"):
            view.set_reference_markers(self._reference_in_zero_ref())
        if live is not None and hasattr(live, "set_reference_markers"):
            upp = None
            if self._camera_manager is not None:
                try:
                    upp = float(self._camera_manager.get_um_per_px(
                        self._ploc_live_cam_idx) or 0.0)
                except Exception:
                    upp = None
            stage_xy = None
            if self.controller is not None:
                try:
                    xy = self.controller.get_xy_position(cached=True)
                    if xy and xy[0] is not None:
                        stage_xy = (float(xy[0]), float(xy[1]))
                except Exception:
                    stage_xy = None
            live.set_reference_markers(
                [(n, x, y) for n, (x, y) in self._reference_markers.items()],
                stage_x_um=stage_xy[0] if stage_xy else None,
                stage_y_um=stage_xy[1] if stage_xy else None,
                um_per_px=upp)

        # v7.5.x: keep the Z side view (next to the well layout) in sync —
        # captured Z references (Safe badge), needle, envelope, display sign.
        xz = getattr(self, "_ploc_xz_view", None)
        if xz is not None:
            sl_xz = (getattr(self.controller, "safety_limits", None)
                     if self.controller else None)
            if sl_xz is not None:
                xz.set_safety_limits(sl_xz)
            if od:
                xz.set_needle(float(od))
            if (self.controller is not None
                    and hasattr(self.controller, "z_up_sign")):
                try:
                    xz.set_z_display_sign(self.controller.z_up_sign())
                except Exception:
                    pass
            xz.set_z_references(self.get_z_references())

    def _on_ploc_go_to_z(self, z_mm: float) -> None:
        """v7.5.x: Plate Location Z side-view badge clicked → move Z there.

        ``z_mm`` is zero-referenced (the raw move frame the side view emits;
        the green "Safe" badge carries the Fast Move Z). This is a manual,
        operator-initiated Z move used to retract the needle up to safe travel
        before/while teaching wells. Uses ``move_z_absolute`` so the Z soft
        limits are still respected.
        """
        ctrl = self.controller
        if ctrl is None or not getattr(ctrl, "is_zp_connected", False):
            logger.info("PlateLocation go-to-Z: ZP stage not connected")
            return
        logger.info("PlateLocation go-to-Z: %.2f mm (zero-ref)", z_mm)
        try:
            ctrl.move_z_absolute(z_mm, from_zero_ref=True)
        except Exception as exc:
            logger.warning("PlateLocation go-to-Z failed: %s", exc)

    def _emit_calibration_data_changed(self):
        """v7.3.1: Notify listeners that calibration data has changed.

        v7.4.0-a: Also schedule a debounced auto-save to settings.json so
        users never lose taught Safe Z / Top Z / 3-point positions just
        because they forgot to click the manual Save button.
        """
        self._refresh_ploc_view()
        self.calibration_data_changed.emit()
        if self.settings is not None and hasattr(self, '_autosave_timer'):
            self._autosave_timer.start()
        # v7.4.3: keep the reusable jog context panel's Hardware Info /
        # Safe Z line in sync whenever calibration changes.
        panel = getattr(self, "_jog_left_panel", None)
        if panel is not None:
            try:
                panel.set_calibration_data(*self.get_calibration_data())
            except Exception:
                pass
            # v7.4.4: also push the full Z reference set so the
            # Hardware Info card reflects Replace / Max / Plate Top /
            # Plate Bottom Z, not just Fast Move Z.
            if hasattr(panel, "set_z_references"):
                try:
                    panel.set_z_references(self.get_z_references())
                except Exception:
                    pass

    def set_xy_position_scale(self, value: float):
        """Update the XY position scale factor."""
        self._xy_position_scale = value

    def set_hardware_config(self, config):
        """v7.2.4: Set hardware config — sync plate format, needle info, and plate model.

        Syncs BOTH the main plate_combo (if it exists) and the context panel
        ctx_plate_combo. Also rebuilds the WellPlate model and validation
        well combo so calibration uses the correct plate geometry.
        """
        # v7.5.x: per-plate-key calibration. Capture the key the in-memory
        # calibration currently belongs to BEFORE adopting the new config.
        _old_cal_key = self._loaded_cal_key
        self._hardware_config = config
        if config is None:
            return

        # v7.4.5: use active_plate_key (str for custom, int for standard).
        key = getattr(config, 'active_plate_key', None)
        if key is None:
            # Older configs without the new field — fall back to plate_format.
            key = getattr(config, 'plate_format', None)
        _new_cal_key = str(key) if key is not None else None
        # v7.5.x: GEOMETRY may differ from the identity key — a plate TYPE
        # carries identity/Z-offsets but resolves to a plain base format, so a
        # layered custom design (e.g. a rosette saved under plate_name) supplies
        # the geometry. `geometry_plate_key` prefers the custom design when both
        # are set; identity/segregation stays on `key` (active_plate_key).
        geom_key = getattr(config, 'geometry_plate_key', None) or key
        # If the active plate changed, flush the OUTGOING plate's calibration to
        # its archive NOW — self._plate + the in-memory taught state are still
        # the old plate's here (the geometry rebuild happens below). The
        # INCOMING plate's calibration is restored after that rebuild.
        if (_new_cal_key != _old_cal_key and _old_cal_key is not None
                and self.settings is not None):
            try:
                self._save_calibration(key=_old_cal_key)
            except Exception as e:
                logger.debug(
                    f"flush outgoing calibration ({_old_cal_key}) skipped: {e}")
        if key:
            # Sync main plate_combo (only resolves for integer standards)
            if hasattr(self, 'plate_combo') and isinstance(key, int):
                for i in range(self.plate_combo.count()):
                    if self.plate_combo.itemData(i) == key:
                        self.plate_combo.setCurrentIndex(i)
                        break
            # Sync context panel plate combo (same, only for standards)
            if hasattr(self, 'ctx_plate_combo') and isinstance(key, int):
                for i in range(self.ctx_plate_combo.count()):
                    if self.ctx_plate_combo.itemData(i) == key:
                        if self.ctx_plate_combo.currentIndex() != i:
                            self.ctx_plate_combo.setCurrentIndex(i)
                        break
            # Rebuild the plate model via the new polymorphic loader (geometry
            # key so a layered custom/rosette design is honored under a type).
            try:
                self._plate = WellPlate.load(geom_key)
            except Exception as e:
                logger.warning(
                    f"Failed to load plate '{geom_key}': {e}; "
                    f"falling back to 24-well")
                self._plate = WellPlate.from_format(24)
            rows, cols = self._plate.rows, self._plate.cols
            self._corner_well = f"{chr(ord('A') + rows - 1)}{cols}"
            if hasattr(self, 'val_well_combo'):
                wells = self._plate.well_names
                self.val_well_combo.clear()
                self.val_well_combo.addItems(wells)
            # v7.4.4: build a geometry-only predicted-positions map
            # centered on the controller zero so the workspace view
            # (and the Jog page, via set_calibration_data) immediately
            # shows the correct number of wells in roughly the right
            # spot even before any real calibration has happened. The
            # calibration flow will replace these with the
            # affine-fitted positions later.
            #
            # If saved calibrated positions exist but belong to a
            # different plate format (user switched 96 → 24 in
            # Hardware Setup, for example), they are stale and must
            # be cleared so they don't bleed into the new workspace.
            new_well_names = set(self._plate.well_names)
            if self._calibrated_positions:
                cal_names = set(self._calibrated_positions.keys())
                if not cal_names.issubset(new_well_names) or not new_well_names.issubset(cal_names):
                    logger.info(
                        "Clearing calibrated positions — they belong to a "
                        "different plate format than the current hardware "
                        f"config ({len(cal_names)} wells vs {len(new_well_names)})."
                    )
                    self._calibrated_positions = None
            if self.controller is not None:
                try:
                    # v7.5.x: centre the default plate on the XY safety
                    # envelope (not on zero) so it sits in the middle of the
                    # configured travel even for an asymmetric envelope.
                    cx, cy = self.controller.default_plate_center_um()
                    self._predicted_positions = (
                        self._plate.get_all_positions_from_plate_center(
                            cx, cy, self._plate_axis_sign()))
                except Exception as e:
                    logger.debug(f"geometry-only well prediction skipped: {e}")
            logger.info(f"Calibration: plate synced to {key} from HardwareConfig")

            # v7.5.x: restore the INCOMING plate's archived calibration (taught
            # wells / warp / Z references) — or reset to blank when it has none
            # — so the calibration follows the plate. Only on an actual key
            # change; an unrelated config change (needle/pump) keeps the live
            # calibration intact (the same-well-set guard above still applies).
            if _new_cal_key != _old_cal_key:
                try:
                    self._activate_calibration_for_key(_new_cal_key)
                except Exception as e:
                    logger.debug(
                        f"activate calibration ({_new_cal_key}) skipped: {e}")
            else:
                self._loaded_cal_key = _new_cal_key

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

        # v7.4.4: refresh the Needle Location tab — the new role
        # assignments may have changed which cameras to mount.
        if hasattr(self, '_needle_loc_cam_slots'):
            try:
                self._needle_loc_refresh_cameras()
            except Exception as e:
                logger.debug(f"_needle_loc_refresh_cameras skipped: {e}")

        # v7.4.4: keep the shared XY workspace view in sync with the
        # latest plate / needle / safety envelope.
        try:
            self._refresh_ploc_view()
        except Exception:
            pass

        # v7.5.x: show this plate's stored stitched mosaic (if any) as the
        # plate-view overlay.
        try:
            self._ploc_load_persisted_mosaic()
        except Exception as e:
            logger.debug(f"PlateLocation: mosaic reload skipped: {e}")

        # v7.4.3: forward into the reusable standard jog context panel
        panel = getattr(self, "_jog_left_panel", None)
        if panel is not None:
            try:
                panel.set_hardware_config(config)
            except Exception:
                pass

        # v7.4.4: a fresh plate (or just-rebuilt predicted positions)
        # must be broadcast via ``calibration_data_changed`` so the Jog
        # page swaps out the stale ``load_startup_plate`` wells. Without
        # this emit, switching plate formats in Hardware Setup left the
        # workspace canvas showing the old (or settings-default) plate.
        try:
            self._emit_calibration_data_changed()
        except Exception as e:
            logger.debug(f"calibration_data_changed emit skipped: {e}")

        # v7.5.x: a selected plate TYPE supplies the plate-Z guesses. After the
        # controller adopted the type's offsets (StageController.set_hardware_
        # config) auto-fill any UN-taught Z references from them. Runs after
        # _load_calibration() (in __init__) has restored taught values, so the
        # taught-wins gate cannot clobber a real calibration.
        try:
            self._apply_plate_type_z_estimates(force=False)
        except Exception as e:
            logger.debug(f"plate-type Z auto-fill skipped: {e}")


    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL  (Plate Config + Plate View + Wizard Steps)
    # ════════════════════════════════════════════════════════════════

    # v7.4.2: ``get_context_widget`` now returns a soft-limit jog panel
    # (no Connect Hardware) so the user can drive the stages while
    # calibrating. The original camera/plate/wizard content moved to
    # the right context panel, exposed via ``get_right_context_widget``.

    def get_context_widget(self) -> QWidget:
        """Left context panel — the project's standard reusable jog panel.

        v7.4.3: switched from a bare ``HardwareControlPanel`` to
        :class:`StandardJogContextPanel`, which keeps the same jog UI
        (safety limits engaged) and adds the Quick Actions, Absolute
        Go To, and Hardware Info sections. The same panel is reused on
        the Jog page so users learn one jog UI.
        """
        if not hasattr(self, "_jog_left_panel") or self._jog_left_panel is None:
            from gui.widgets.standard_jog_context import StandardJogContextPanel
            ctrl = getattr(self, "controller", None)
            settings = getattr(self, "settings", None)
            panel = StandardJogContextPanel(
                controller=ctrl,
                settings=settings,
                show_connect=False,
                bypass_safety=False,
            )
            # Push any hardware config or calibration data that already
            # arrived before the panel was instantiated.
            hw = getattr(self, "_hardware_config", None)
            if hw is not None:
                panel.set_hardware_config(hw)
            try:
                panel.set_calibration_data(*self.get_calibration_data())
            except Exception:
                pass
            # v7.4.4: push the full Z reference set on first build.
            if hasattr(panel, "set_z_references"):
                try:
                    panel.set_z_references(self.get_z_references())
                except Exception:
                    pass
            self._jog_left_panel = panel
        return self._jog_left_panel

    def get_right_context_title(self) -> str:
        return ""

    def get_right_context_widget(self) -> QWidget | None:
        """v7.4.4: right context retired. The four workflow tabs
        (Needle Location / Z-Offset / Plate Location / Custom) live in
        the main page area now, so returning None tells app.py not to
        mount a right column for this page."""
        return None

    # ════════════════════════════════════════════════════════════════
    #  v7.4.4: Custom (freeform) tab — wraps the v7.4.2 sub-tabs so
    #  nothing the user could do before disappears.
    # ════════════════════════════════════════════════════════════════

    def _build_custom_tab(self) -> QWidget:
        """Build the Custom tab: the live multi-camera grid plus the
        v7.4.2 Camera Setup / Full Wizard / Plate Calibration sections
        nested as sub-tabs so power users keep full access while the
        new workflow tabs handle the common jobs."""
        outer = QWidget()
        outer_lay = QVBoxLayout(outer)
        outer_lay.setContentsMargins(s(6), s(6), s(6), s(6))
        outer_lay.setSpacing(s(6))

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(s(4))

        # Left side: the legacy multi-camera grid (relocated from the
        # main page area). The grid widget was built in _setup_ui but
        # left parentless — adopting it here gives the Custom tab the
        # live view today's users expect.
        cam_card = QFrame()
        cam_card.setObjectName("cardFrame")
        cam_lay = QVBoxLayout(cam_card)
        cam_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        cam_lay.setSpacing(s(2))
        if hasattr(self, '_cam_grid_widget'):
            cam_lay.addWidget(self._cam_grid_widget, stretch=1)
        splitter.addWidget(cam_card)

        # Right side: control sub-tabs (Camera Setup / Full Wizard /
        # Plate Calibration). These are the v7.4.2 builders, which
        # are stateful and run once.
        sub_tabs = QTabWidget()
        sub_tabs.setObjectName("calibrationCustomSubTabs")

        cam_tab = QWidget()
        cam_tab_lay = QVBoxLayout(cam_tab)
        cam_tab_lay.setContentsMargins(s(10), s(10), s(10), s(10))
        cam_tab_lay.setSpacing(s(6))
        self._build_camera_tab_content(cam_tab_lay)
        cam_tab_lay.addStretch(1)
        sub_tabs.addTab(cam_tab, "Camera Setup")

        needle_tab = QWidget()
        needle_lay = QVBoxLayout(needle_tab)
        needle_lay.setContentsMargins(s(10), s(10), s(10), s(10))
        needle_lay.setSpacing(s(6))
        self._build_needle_tab_content(needle_lay)
        needle_lay.addStretch(1)
        sub_tabs.addTab(needle_tab, "Full Wizard")

        plate_tab = QWidget()
        plate_lay = QVBoxLayout(plate_tab)
        plate_lay.setContentsMargins(s(10), s(10), s(10), s(10))
        plate_lay.setSpacing(s(6))
        self._build_plate_tab_content(plate_lay)
        plate_lay.addStretch(1)
        sub_tabs.addTab(plate_tab, "Plate Calibration")

        splitter.addWidget(sub_tabs)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        outer_lay.addWidget(splitter, stretch=1)
        return outer

    # ════════════════════════════════════════════════════════════════
    #  v7.4.4: Needle Location tab — dual-camera edge-click workflow
    # ════════════════════════════════════════════════════════════════

    def _build_needle_location_tab(self) -> QWidget:
        """Workflow tab: pick needle edges in two side cameras, then
        recenter so the needle sits at the optical center of both.

        Saves the resulting stage position as ``needle_origin_um`` —
        the workspace reference that travels across plate loadouts.
        """
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(6))

        # Persistent edge-pick state for this tab.
        if NEEDLE_LOCATION_AVAILABLE:
            self._needle_loc_picks = TwoCameraEdgePicks()
        else:
            self._needle_loc_picks = None
        # 0 = waiting for x_left, 1 = x_right, 2 = y_left, 3 = y_right, 4 = ready
        self._needle_loc_step = 0
        self._needle_loc_views: list = []  # [x_view, y_view]
        # v7.5.x: per-click image ROW (py) for the four edge picks. Rows map to
        # stage Z in each side camera, so the midpoint of a view's two clicks is
        # the tip-bottom row → drives the Z move that lands the tip on the
        # crosshair. Keyed "x_left"/"x_right"/"y_left"/"y_right".
        self._needle_loc_rows: dict[str, float] = {}

        # v7.5.x: saved approximate needle location (absolute Prior stage µm) —
        # the centered needle position, a stable per-machine datum (the side
        # cameras are fixed to the frame). Drives the "Go to needle location"
        # quick-move. Persisted in the device profile (settings) so it survives
        # restarts; the ProScan keeps its absolute frame across power cycles.
        self._needle_loc_xy_um: tuple[float, float] | None = None
        if self.settings is not None:
            try:
                _saved = self.settings.get("device_profile.needle_loc_xy_um")
                if _saved and len(_saved) >= 2:
                    self._needle_loc_xy_um = (float(_saved[0]), float(_saved[1]))
            except Exception:
                self._needle_loc_xy_um = None

        # ── Status banner ────────────────────────────────────────
        self._needle_loc_banner = QLabel()
        self._needle_loc_banner.setWordWrap(True)
        self._needle_loc_banner.setStyleSheet(
            f"background-color: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; padding: {sp(8)}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        outer.addWidget(self._needle_loc_banner)

        # ── Splitter: cameras (top) / wizard steps (bottom) ──────
        splitter = QSplitter(Qt.Vertical)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(s(4))

        # Camera tiles row (a horizontal splitter inside the vertical one).
        cam_row_wrap = QWidget()
        cam_row = QHBoxLayout(cam_row_wrap)
        cam_row.setContentsMargins(0, 0, 0, 0)
        cam_row.setSpacing(s(6))
        cam_splitter = QSplitter(Qt.Horizontal)
        cam_splitter.setChildrenCollapsible(False)
        cam_splitter.setHandleWidth(s(4))
        # Two slots — they get filled by _needle_loc_refresh_cameras() once
        # the user assigns roles in Hardware Setup → Cameras.
        self._needle_loc_cam_slots = [QWidget(), QWidget()]
        for i, slot in enumerate(self._needle_loc_cam_slots):
            lay = QVBoxLayout(slot)
            lay.setContentsMargins(0, 0, 0, 0)
            label = QLabel(
                f"<b>{'X-view' if i == 0 else 'Y-view'}</b>"
            )
            label.setStyleSheet(f"color: {COLORS['blue']};")
            lay.addWidget(label)
            placeholder = QLabel(
                "No camera assigned.\n"
                "Set role in Hardware Setup → Cameras."
            )
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet(
                f"background-color: #181825; "
                f"color: {COLORS['subtext0']}; "
                f"border: 1px solid {COLORS['surface1']}; "
                f"min-height: {sp(180)}; padding: {sp(12)};")
            lay.addWidget(placeholder, stretch=1)
            slot.setProperty("_placeholder", placeholder)
            slot.setProperty("_feed_view", None)
            cam_splitter.addWidget(slot)
        cam_row.addWidget(cam_splitter, stretch=1)
        splitter.addWidget(cam_row_wrap)

        # Wizard panel.
        wiz_panel = QWidget()
        wiz_lay = QVBoxLayout(wiz_panel)
        wiz_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        wiz_lay.setSpacing(s(6))

        self._needle_loc_step_label = QLabel()
        self._needle_loc_step_label.setStyleSheet(
            f"font-weight: 600; color: {COLORS['text']};")
        wiz_lay.addWidget(self._needle_loc_step_label)

        # ── v7.5.x: Quick-move to the saved approximate needle location ──
        # The side cameras are bolted to the frame, so the centered needle
        # position is a stable per-machine datum. Driving here (retract Z →
        # XY → lower to the needle-cam Z) re-enters the needle into both views
        # so the operator can immediately re-center, instead of hand-jogging
        # it back into frame.
        goto_row = QHBoxLayout()
        self._needle_loc_btn_goto = QPushButton("⤵ Go to needle location")
        self._needle_loc_btn_goto.setToolTip(
            "Quick-move to the saved approximate needle location: retract Z to "
            "the Fast-Move height, travel XY, then lower to the needle-cam Z so "
            "the needle re-enters both side views, ready to re-center.")
        self._needle_loc_btn_goto.clicked.connect(self._needle_loc_goto)
        goto_row.addWidget(self._needle_loc_btn_goto)
        self._needle_loc_btn_set = QPushButton("Set current as location")
        self._needle_loc_btn_set.setToolTip(
            "Capture the current stage XY as the approximate needle location. "
            "Use this once on a fresh machine to seed the quick-move before the "
            "first Center & Save (which then updates it automatically).")
        self._needle_loc_btn_set.clicked.connect(self._needle_loc_set_current)
        goto_row.addWidget(self._needle_loc_btn_set)
        # v7.5.x: accept the last-known needle calibration WITHOUT re-centering
        # with the side cameras. For operators who are sure nothing about the
        # needle / its mounting / the cameras has changed since last session.
        self._needle_loc_btn_last_known = QPushButton(
            "✓ Use last known location")
        self._needle_loc_btn_last_known.setToolTip(
            "Restore the last-known needle calibration (needle zero + "
            "needle-cam Z + saved needle XY) WITHOUT re-centering with the "
            "side cameras. Only use this if nothing about the needle, its "
            "mounting, or the cameras has changed since it was saved.")
        self._needle_loc_btn_last_known.clicked.connect(
            self._needle_loc_use_last_known)
        goto_row.addWidget(self._needle_loc_btn_last_known)
        goto_row.addStretch()
        wiz_lay.addLayout(goto_row)

        self._needle_loc_goto_label = QLabel()
        self._needle_loc_goto_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        self._needle_loc_goto_label.setWordWrap(True)
        wiz_lay.addWidget(self._needle_loc_goto_label)

        # Pick status table — four rows.
        self._needle_loc_pick_labels: dict[str, QLabel] = {}
        pick_grid = QGridLayout()
        pick_grid.setHorizontalSpacing(s(8))
        pick_grid.setVerticalSpacing(s(4))
        for row, (key, label) in enumerate([
            ("x_left", "X-view left edge:"),
            ("x_right", "X-view right edge:"),
            ("y_left", "Y-view left edge:"),
            ("y_right", "Y-view right edge:"),
        ]):
            pick_grid.addWidget(QLabel(label), row, 0)
            v = QLabel("—")
            v.setStyleSheet(f"color: {COLORS['subtext0']};")
            self._needle_loc_pick_labels[key] = v
            pick_grid.addWidget(v, row, 1)
        wiz_lay.addLayout(pick_grid)

        # Computed offset preview.
        self._needle_loc_offset_label = QLabel("Offset preview: —")
        self._needle_loc_offset_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-style: italic;")
        wiz_lay.addWidget(self._needle_loc_offset_label)

        # v7.5.x: Z centering — drive Z so the tip's center-bottom lands on the
        # crosshair (rows of each side camera map to stage Z). Opt-in; the
        # vertical sign is not captured by the µm/px calibration, so an Invert
        # toggle handles flipped camera mounts. The Z move is soft-limit +
        # print-floor clamped (move_z_user_relative).
        self._needle_loc_z_center_chk = QCheckBox(
            "Also center Z (tip → crosshair)")
        self._needle_loc_z_center_chk.setChecked(True)
        self._needle_loc_z_center_chk.setToolTip(
            "After recentering XY, move Z so the needle tip's center-bottom "
            "sits on the camera crosshair. The vertical move is clamped by the "
            "Z soft-limit. Click the tip's bottom-left / bottom-right corners.")
        self._needle_loc_z_center_chk.toggled.connect(
            lambda _=False: self._needle_loc_update_ui())
        wiz_lay.addWidget(self._needle_loc_z_center_chk)

        self._needle_loc_z_invert_chk = QCheckBox(
            "Invert Z direction (flipped camera mount)")
        self._needle_loc_z_invert_chk.setToolTip(
            "Toggle if the Z move drives the tip away from the crosshair "
            "instead of toward it.")
        self._needle_loc_z_invert_chk.toggled.connect(
            lambda _=False: self._needle_loc_update_ui())
        wiz_lay.addWidget(self._needle_loc_z_invert_chk)

        # Action buttons.
        action_row = QHBoxLayout()
        self._needle_loc_btn_reset = QPushButton("Reset picks")
        self._needle_loc_btn_reset.clicked.connect(self._needle_loc_reset)
        action_row.addWidget(self._needle_loc_btn_reset)
        action_row.addStretch()
        self._needle_loc_btn_center = QPushButton("Center & Save needle origin")
        self._needle_loc_btn_center.setObjectName("accentBtn")
        self._needle_loc_btn_center.setEnabled(False)
        self._needle_loc_btn_center.clicked.connect(
            self._needle_loc_center_and_save)
        action_row.addWidget(self._needle_loc_btn_center)
        wiz_lay.addLayout(action_row)

        # Last saved origin.
        self._needle_loc_origin_label = QLabel("needle_origin_um: not set")
        self._needle_loc_origin_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; padding-top: {sp(6)};")
        wiz_lay.addWidget(self._needle_loc_origin_label)

        splitter.addWidget(wiz_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # ── v7.5.x: merge the former "Needle Offset Calibration" tab in as the
        # right pane. Center the needle in the side cameras (left) while setting
        # its reference-Z heights on the XZ elevation view (right) — one tab,
        # since the operator treats them as the same needle-reference cal.
        main_split = QSplitter(Qt.Horizontal)
        main_split.setChildrenCollapsible(False)
        main_split.setHandleWidth(s(4))
        main_split.addWidget(splitter)
        main_split.addWidget(self._build_z_offset_content())
        main_split.setStretchFactor(0, 3)
        main_split.setStretchFactor(1, 2)
        outer.addWidget(main_split, stretch=1)

        # Initial UI state.
        self._needle_loc_refresh_cameras()
        self._needle_loc_update_ui()
        self._needle_loc_update_goto_ui()
        return page

    # ════════════════════════════════════════════════════════════════
    #  v7.5.x: Pump compliance / pressure-relief calibration
    # ════════════════════════════════════════════════════════════════

    def _build_compliance_cal_group(self) -> QGroupBox:
        """Guided per-pump compliance calibration controls (hosted on the Pump
        Compliance tab, above a live needle side-camera view).

        Procedure (watch the needle tip in the side cameras): ① dispense in
        small steps until a droplet appears, ② click "Start calibration" to zero
        the counters (the flex is now fully toward dispense), ③ aspirate in
        steps until the droplet is pulled back into the needle, ④ click "Finish"
        to compute the relief c = A/2 from the aspirate-back total, ⑤ Save. The
        relief / compliance value is HALF that aspirate-back volume — the µL of
        plunger travel needed to remove the residual drivetrain flex in one
        direction. Saved per-pump and used by backlash compensation. "Reset"
        returns to the free-dispense phase to retry.
        """
        grp = QGroupBox("Pump compliance / pressure-relief calibration")
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(6))

        # Guidance note — text is driven per-phase by _compcal_update_phase_ui.
        self._compcal_note = QLabel()
        self._compcal_note.setWordWrap(True)
        self._compcal_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(self._compcal_note)

        # Calibration state. ``_compcal_phase`` drives the guided 5-step flow:
        #   "prep"    — free-dispense to form a droplet (NOT counted)
        #   "measure" — after "Start calibration" zeroes the counters; aspirate
        #               accumulates the net aspirate-back A
        #   "done"    — c = A/2 computed by "Finish"; ready to Save
        self._compcal_dispensed_uL = 0.0
        self._compcal_aspirated_uL = 0.0
        self._compcal_phase = "prep"
        self._compcal_busy = False
        self._compcal_pending_dir = 0.0
        self._compcal_pending_uL = 0.0
        self._compcal_bridge = _CompCalBridge()
        self._compcal_bridge.done.connect(self._compcal_on_step_done)

        # Pump + step/rate row.
        top = QHBoxLayout()
        top.addWidget(QLabel("Pump:"))
        self._compcal_pump = QComboBox()
        for pid in ("P1", "P2", "P3"):
            self._compcal_pump.addItem(pid)
        self._compcal_pump.currentIndexChanged.connect(
            lambda _=0: self._compcal_refresh())
        top.addWidget(self._compcal_pump)
        top.addWidget(QLabel("Step:"))
        self._compcal_step = QDoubleSpinBox()
        self._compcal_step.setRange(0.001, 5.0)
        self._compcal_step.setDecimals(3)
        self._compcal_step.setSingleStep(0.1)
        self._compcal_step.setValue(0.1)
        self._compcal_step.setSuffix(" µL")
        top.addWidget(self._compcal_step)
        top.addWidget(QLabel("Rate:"))
        self._compcal_rate = QDoubleSpinBox()
        self._compcal_rate.setRange(0.01, 50.0)
        self._compcal_rate.setDecimals(2)
        self._compcal_rate.setSingleStep(0.1)
        self._compcal_rate.setValue(0.5)
        self._compcal_rate.setSuffix(" µL/s")
        top.addWidget(self._compcal_rate)
        top.addStretch()
        lay.addLayout(top)

        # Live fill readout.
        self._compcal_fill_lbl = QLabel("Fill: —")
        self._compcal_fill_lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
        lay.addWidget(self._compcal_fill_lbl)

        # Step buttons.
        step_row = QHBoxLayout()
        self._compcal_btn_dispense = QPushButton("Dispense +step")
        self._compcal_btn_dispense.setToolTip(
            "① Push a step of fluid OUT. Repeat until a droplet forms at the "
            "tip. (After Start, a dispense subtracts from A.)")
        self._compcal_btn_dispense.clicked.connect(
            lambda: self._compcal_step_move(+1.0))
        step_row.addWidget(self._compcal_btn_dispense)
        self._compcal_btn_aspirate = QPushButton("Aspirate −step")
        self._compcal_btn_aspirate.setToolTip(
            "③ Draw a step of fluid IN. After Start, repeat until the droplet "
            "is pulled back into the needle; the accumulated total is A.")
        self._compcal_btn_aspirate.clicked.connect(
            lambda: self._compcal_step_move(-1.0))
        step_row.addWidget(self._compcal_btn_aspirate)
        lay.addLayout(step_row)

        # Totals.
        self._compcal_totals_lbl = QLabel(
            "Dispensed: 0.000 µL   Aspirated (A): 0.000 µL   Relief c=A/2: — µL")
        self._compcal_totals_lbl.setStyleSheet(f"color: {COLORS['text']};")
        lay.addWidget(self._compcal_totals_lbl)

        # Phase control: ② Start calibration (zeroes) → ④ Finish (compute c).
        phase_row = QHBoxLayout()
        self._compcal_btn_start = QPushButton("② Start calibration (zero)")
        self._compcal_btn_start.setToolTip(
            "The droplet looks right (flex is fully toward dispense) — zero the "
            "counters and begin measuring the aspirate-back A. Aspirate from "
            "here counts toward the calibration.")
        self._compcal_btn_start.clicked.connect(self._compcal_start)
        phase_row.addWidget(self._compcal_btn_start)
        self._compcal_btn_compute = QPushButton("④ Finish (compute c)")
        self._compcal_btn_compute.setToolTip(
            "The droplet is fully back in the needle — compute the relief value "
            "c = A/2 from the aspirate-back total measured since Start.")
        self._compcal_btn_compute.clicked.connect(self._compcal_compute)
        phase_row.addWidget(self._compcal_btn_compute)
        phase_row.addStretch()
        lay.addLayout(phase_row)

        # ⑤ Save (with manual override of the computed value).
        save_row = QHBoxLayout()
        save_row.addWidget(QLabel("Relief c:"))
        self._compcal_manual = QDoubleSpinBox()
        self._compcal_manual.setRange(0.0, 50.0)
        self._compcal_manual.setDecimals(3)
        self._compcal_manual.setSingleStep(0.01)
        self._compcal_manual.setSuffix(" µL")
        self._compcal_manual.setToolTip(
            "The relief value that will be saved (auto-filled by 'Finish'; "
            "editable to fine-tune or enter manually).")
        # A manual entry enables Save even outside the guided flow (the manual-
        # override path); a bare 0 keeps it disabled.
        self._compcal_manual.valueChanged.connect(
            lambda _=0.0: self._compcal_update_phase_ui())
        save_row.addWidget(self._compcal_manual)
        self._compcal_btn_save = QPushButton("⑤ Save relief")
        self._compcal_btn_save.setObjectName("accentBtn")
        self._compcal_btn_save.clicked.connect(self._compcal_save)
        save_row.addWidget(self._compcal_btn_save)
        save_row.addStretch()
        lay.addLayout(save_row)

        # Reset + status.
        bottom = QHBoxLayout()
        self._compcal_btn_reset = QPushButton("Reset")
        self._compcal_btn_reset.clicked.connect(self._compcal_reset)
        bottom.addWidget(self._compcal_btn_reset)
        self._compcal_status = QLabel("")
        self._compcal_status.setWordWrap(True)
        self._compcal_status.setStyleSheet(f"color: {COLORS['subtext0']};")
        bottom.addWidget(self._compcal_status, stretch=1)
        lay.addLayout(bottom)

        self._compcal_update_totals()
        self._compcal_update_phase_ui()
        self._compcal_refresh()
        return grp

    def _build_pump_compliance_tab(self) -> QWidget:
        """Workflow tab (next to Needle Location): guided per-pump compliance /
        pressure-relief calibration. A live needle side-camera view on top so
        the operator can watch the droplet at the tip; the guided controls
        below. The needle is parked at the needle location (no XY/Z travel here)
        so both side cameras frame the tip."""
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(6))

        splitter = QSplitter(Qt.Vertical)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(s(4))

        # ── Live needle side-camera views (watch the droplet at the tip) ──
        cam_row_wrap = QWidget()
        cam_row = QHBoxLayout(cam_row_wrap)
        cam_row.setContentsMargins(0, 0, 0, 0)
        cam_row.setSpacing(s(6))
        cam_splitter = QSplitter(Qt.Horizontal)
        cam_splitter.setChildrenCollapsible(False)
        cam_splitter.setHandleWidth(s(4))
        self._compcal_cam_slots = [QWidget(), QWidget()]
        for i, slot in enumerate(self._compcal_cam_slots):
            slay = QVBoxLayout(slot)
            slay.setContentsMargins(0, 0, 0, 0)
            label = QLabel(f"<b>{'X-view' if i == 0 else 'Y-view'}</b>")
            label.setStyleSheet(f"color: {COLORS['blue']};")
            slay.addWidget(label)
            placeholder = QLabel(
                "No camera assigned.\n"
                "Set role in Hardware Setup → Cameras.")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet(
                f"background-color: #181825; "
                f"color: {COLORS['subtext0']}; "
                f"border: 1px solid {COLORS['surface1']}; "
                f"min-height: {sp(140)}; padding: {sp(12)};")
            slay.addWidget(placeholder, stretch=1)
            slot.setProperty("_placeholder", placeholder)
            slot.setProperty("_feed_view", None)
            cam_splitter.addWidget(slot)
        cam_row.addWidget(cam_splitter, stretch=1)
        splitter.addWidget(cam_row_wrap)

        # ── Guided controls (scrollable) ──
        controls = QScrollArea()
        controls.setWidgetResizable(True)
        controls.setWidget(self._build_compliance_cal_group())
        controls.setFrameShape(QFrame.NoFrame)
        splitter.addWidget(controls)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        outer.addWidget(splitter, stretch=1)

        self._compcal_refresh_cameras()
        return page

    def _compcal_refresh_cameras(self) -> None:
        """Mount the NEEDLE_X / NEEDLE_Y cameras into the Pump Compliance tab
        slots (view only — no edge-pick clicks here), or show a placeholder."""
        if CameraRole is None or not hasattr(self, "_compcal_cam_slots"):
            return
        hw = getattr(self, "_hardware_config", None)
        for slot_idx, role in enumerate(
                [CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y]):
            slot = self._compcal_cam_slots[slot_idx]
            placeholder = slot.property("_placeholder")
            old_feed = slot.property("_feed_view")
            cam_idx = hw.camera_for_role(role) if hw is not None else None
            if cam_idx is None or self._camera_manager is None:
                if old_feed is not None:
                    old_feed.setVisible(False)
                if placeholder is not None:
                    placeholder.setVisible(True)
                continue
            if old_feed is not None:
                try:
                    old_feed.set_camera(cam_idx)
                    old_feed.setVisible(True)
                    if placeholder is not None:
                        placeholder.setVisible(False)
                    continue
                except Exception:
                    pass
            try:
                from gui.widgets.camera_feed_view import CameraFeedView
                fv = CameraFeedView(
                    camera_manager=self._camera_manager,
                    cam_idx=cam_idx,
                    show_crosshair=True,
                    label=f"Camera {cam_idx + 1}",
                    parent=slot,
                )
                slot.layout().addWidget(fv, stretch=1)
                slot.setProperty("_feed_view", fv)
                if placeholder is not None:
                    placeholder.setVisible(False)
            except Exception as e:
                logger.warning(
                    f"PumpCompliance: failed to mount cam {cam_idx}: {e}")

    def _compcal_ensure_live_cameras(self) -> None:
        """Bind + start the needle side cameras on the Pump Compliance tab so
        the droplet is visible. Mirrors ``_zoff_ensure_live_camera``; ``start``
        is a no-op when a camera is already running."""
        self._compcal_refresh_cameras()
        if CameraRole is None or self._camera_manager is None:
            return
        hw = getattr(self, "_hardware_config", None)
        if hw is None:
            return
        for role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y):
            try:
                cam_idx = hw.camera_for_role(role)
                if cam_idx is not None:
                    self._camera_manager.start(cam_idx)
            except Exception as e:
                logger.debug(f"PumpCompliance: camera start failed: {e}")

    def _compcal_pump_id(self) -> str:
        return (self._compcal_pump.currentText()
                if hasattr(self, "_compcal_pump") else "P1") or "P1"

    def _compcal_refresh(self) -> None:
        """Refresh the fill readout + the saved-relief hint for the current pump."""
        pump = self._compcal_pump_id()
        ctrl = self.controller
        # Live fill (if the plunger is calibrated) + the currently-saved relief.
        fill_txt = "Fill: —"
        try:
            if getattr(ctrl, "is_pump_plunger_calibrated", None) and \
                    ctrl.is_pump_plunger_calibrated(pump):
                fill = ctrl.pump_fill_uL(pump)
                cap = ctrl.pump_capacity_uL(pump) if hasattr(
                    ctrl, "pump_capacity_uL") else None
                if fill is not None:
                    fill_txt = (f"Fill: {fill:.2f} µL"
                                + (f" / {cap:.0f} µL" if cap else ""))
        except Exception:
            pass
        saved = 0.0
        try:
            saved = float(ctrl.pump_relief_uL(pump))
        except Exception:
            saved = 0.0
        self._compcal_fill_lbl.setText(
            f"{fill_txt}    ·    saved relief ({pump}): {saved:.3f} µL")

    def _compcal_set_busy(self, busy: bool) -> None:
        self._compcal_busy = busy
        for name in ("_compcal_btn_dispense", "_compcal_btn_aspirate",
                     "_compcal_btn_start", "_compcal_btn_compute",
                     "_compcal_btn_save", "_compcal_btn_reset",
                     "_compcal_pump"):
            w = getattr(self, name, None)
            if w is not None:
                w.setEnabled(not busy)

    def _compcal_update_phase_ui(self) -> None:
        """Set the guidance note + button enable-states for the current phase.
        Never touches enable state while a pump move is in flight (busy) — the
        step-done handler re-applies it after the move completes."""
        if getattr(self, "_compcal_busy", False):
            return
        phase = getattr(self, "_compcal_phase", "prep")
        note = {
            "prep": "① Dispense in steps until a droplet forms at the tip, "
                    "then ② click “Start calibration”.",
            "measure": "③ Aspirate in steps until the droplet is pulled back "
                       "into the needle, then ④ click “Finish”.",
            "done": "Review the relief value and ⑤ click “Save relief”. "
                    "Click “Reset” to retry.",
        }.get(phase, "")
        if getattr(self, "_compcal_note", None) is not None:
            self._compcal_note.setText(note)
        manual_val = 0.0
        m = getattr(self, "_compcal_manual", None)
        if m is not None:
            try:
                manual_val = float(m.value())
            except Exception:
                manual_val = 0.0
        states = {
            # Dispense forms the droplet (prep) and net-corrects an overshoot
            # (measure); off once finished.
            "_compcal_btn_dispense": phase in ("prep", "measure"),
            # Aspirate only counts toward A after "Start" (measure phase).
            "_compcal_btn_aspirate": phase == "measure",
            "_compcal_btn_start": phase == "prep",
            "_compcal_btn_compute": phase == "measure",
            # Save after "Finish", or whenever a value has been entered manually
            # (the manual-override path) — never a bare 0.
            "_compcal_btn_save": phase == "done" or manual_val > 0.0,
            "_compcal_btn_reset": True,
            # Lock the pump selector once measuring — one measurement per pump.
            "_compcal_pump": phase == "prep",
        }
        for name, on in states.items():
            w = getattr(self, name, None)
            if w is not None:
                w.setEnabled(on)

    def _compcal_start(self) -> None:
        """Step ②: the droplet looks right (flex fully toward dispense) — zero
        the counters and begin measuring the aspirate-back A. No pump move."""
        self._compcal_dispensed_uL = 0.0
        self._compcal_aspirated_uL = 0.0
        self._compcal_phase = "measure"
        self._compcal_update_totals()
        self._compcal_update_phase_ui()
        self._compcal_status.setText(
            "Calibration started (counters zeroed). Aspirate until the droplet "
            "is pulled back into the needle, then click “Finish”.")

    def _compcal_step_move(self, direction: float) -> None:
        """Dispense (+) or aspirate (−) one step, off the GUI thread."""
        if self._compcal_busy:
            return
        ctrl = self.controller
        if ctrl is None or not hasattr(ctrl, "move_pump_uL"):
            self._compcal_status.setText("No controller.")
            return
        pump = self._compcal_pump_id()
        step = float(self._compcal_step.value())
        rate = float(self._compcal_rate.value())
        vol = direction * step
        self._compcal_pending_dir = direction
        self._compcal_pending_uL = step
        self._compcal_set_busy(True)
        self._compcal_status.setText(
            f"{'Dispensing' if direction > 0 else 'Aspirating'} "
            f"{step:.3f} µL on {pump}…")

        def _worker():
            ok = True
            try:
                # compensate=False: we are MEASURING the compliance, so the
                # calibration move must not itself apply backlash comp.
                ctrl.move_pump_uL(pump, vol, rate_uL_s=rate,
                                  settle=True, compensate=False)
            except TypeError:
                try:
                    ctrl.move_pump_uL(pump, vol, rate_uL_s=rate)
                except Exception:
                    ok = False
            except Exception:
                ok = False
            self._compcal_bridge.done.emit(ok, pump)

        threading.Thread(target=_worker, daemon=True).start()

    def _compcal_on_step_done(self, ok: bool, pump: str) -> None:
        """Queued back on the GUI thread after a step move finishes. Only moves
        AFTER "Start calibration" (measure phase) feed the aspirate total A; a
        dispense during measure self-corrects (net). Prep-phase moves only
        update the informational free-dispense tally."""
        if ok:
            step = self._compcal_pending_uL
            d = self._compcal_pending_dir
            phase = getattr(self, "_compcal_phase", "prep")
            if phase == "measure":
                # Net aspirate since Start: aspirate adds, dispense subtracts.
                if d < 0:
                    self._compcal_aspirated_uL += step
                elif d > 0:
                    self._compcal_aspirated_uL -= step
                if self._compcal_aspirated_uL < 0.0:
                    self._compcal_aspirated_uL = 0.0
            else:
                # Prep: informational free-dispense tally (not used in the calc).
                if d > 0:
                    self._compcal_dispensed_uL += step
                elif d < 0:
                    self._compcal_dispensed_uL -= step
                    if self._compcal_dispensed_uL < 0.0:
                        self._compcal_dispensed_uL = 0.0
            self._compcal_status.setText("Step complete.")
        else:
            self._compcal_status.setText(
                "Step FAILED — check the pump is configured / connected.")
        self._compcal_update_totals()
        self._compcal_set_busy(False)
        self._compcal_update_phase_ui()
        self._compcal_refresh()

    def _compcal_update_totals(self) -> None:
        phase = getattr(self, "_compcal_phase", "prep")
        if phase == "prep":
            self._compcal_totals_lbl.setText(
                f"Free dispense (forming droplet): "
                f"{self._compcal_dispensed_uL:.3f} µL   —   click “Start "
                f"calibration” when the droplet is ready.")
        else:
            a = self._compcal_aspirated_uL
            c_txt = f"{a / 2.0:.3f} µL" if a > 0 else "— µL"
            self._compcal_totals_lbl.setText(
                f"Aspirated since start (A): {a:.3f} µL   "
                f"Relief c = A/2: {c_txt}")

    def _compcal_compute(self) -> None:
        """Step ④ ("Finish"): compute the relief c = A/2 from the aspirate-back
        total measured since Start; auto-fill the manual spin (still editable)."""
        if getattr(self, "_compcal_phase", "prep") != "measure":
            self._compcal_status.setText(
                "Click “Start calibration” first, then aspirate the droplet "
                "back in.")
            return
        a = self._compcal_aspirated_uL
        if a <= 0:
            self._compcal_status.setText(
                "Aspirate the droplet back in first (A must be > 0).")
            return
        c = a / 2.0
        self._compcal_manual.setValue(c)
        self._compcal_phase = "done"
        self._compcal_update_totals()
        self._compcal_update_phase_ui()
        self._compcal_status.setText(
            f"Relief c = A/2 = {c:.3f} µL. Review and click “Save relief”.")

    def _compcal_save(self) -> None:
        pump = self._compcal_pump_id()
        c = float(self._compcal_manual.value())
        ctrl = self.controller
        if ctrl is None or not hasattr(ctrl, "set_pump_relief_uL"):
            self._compcal_status.setText("No controller — cannot save.")
            return
        try:
            ctrl.set_pump_relief_uL(pump, c)
        except Exception as exc:
            self._compcal_status.setText(f"Save failed: {exc}")
            return
        # Persist to the device profile (mirrors how the plunger setup persists).
        if self.settings is not None and hasattr(ctrl, "get_pump_relief_all"):
            try:
                self.settings.set("device_profile.pump_compliance_uL",
                                  ctrl.get_pump_relief_all())
                self.settings.save()
            except Exception as exc:
                logger.debug("persist pump compliance failed: %s", exc)
        try:
            self._emit_calibration_data_changed()
        except Exception:
            pass
        self._compcal_status.setText(
            f"Saved relief {c:.3f} µL for {pump}. Enable backlash compensation "
            f"on the Jog page to use it.")
        self._compcal_refresh()

    def _compcal_reset(self) -> None:
        """Reset the measurement to retry — back to the free-dispense phase,
        counters and the computed value cleared."""
        self._compcal_dispensed_uL = 0.0
        self._compcal_aspirated_uL = 0.0
        self._compcal_phase = "prep"
        if hasattr(self, "_compcal_manual"):
            self._compcal_manual.setValue(0.0)
        self._compcal_update_totals()
        self._compcal_update_phase_ui()
        self._compcal_status.setText(
            "Reset. Dispense to form a droplet, then Start calibration.")

    def _needle_loc_refresh_cameras(self) -> None:
        """Mount the cameras for NEEDLE_X / NEEDLE_Y roles into the
        Needle Location tab slots, or show the placeholder if a role
        is missing."""
        if CameraRole is None or not hasattr(self, '_needle_loc_cam_slots'):
            return
        hw = getattr(self, '_hardware_config', None)
        roles_ok = True
        for slot_idx, role in enumerate(
            [CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y]
        ):
            slot = self._needle_loc_cam_slots[slot_idx]
            placeholder = slot.property("_placeholder")
            old_feed = slot.property("_feed_view")
            cam_idx = hw.camera_for_role(role) if hw is not None else None
            if cam_idx is None or self._camera_manager is None:
                roles_ok = False
                if old_feed is not None:
                    old_feed.setVisible(False)
                if placeholder is not None:
                    placeholder.setVisible(True)
                continue
            # Build (or rebind) a fresh CameraFeedView.
            if old_feed is not None:
                try:
                    old_feed.set_camera(cam_idx)
                    old_feed.setVisible(True)
                    if placeholder is not None:
                        placeholder.setVisible(False)
                    continue
                except Exception:
                    pass
            try:
                from gui.widgets.camera_feed_view import CameraFeedView
                fv = CameraFeedView(
                    camera_manager=self._camera_manager,
                    cam_idx=cam_idx,
                    show_crosshair=True,
                    label=f"Camera {cam_idx + 1}",
                    parent=slot,
                )
                fv.clicked.connect(
                    lambda px, py, role_=role: self._needle_loc_on_click(role_, px, py)
                )
                slot.layout().addWidget(fv, stretch=1)
                slot.setProperty("_feed_view", fv)
                if placeholder is not None:
                    placeholder.setVisible(False)
            except Exception as e:
                logger.warning(f"NeedleLocation: failed to mount cam {cam_idx}: {e}")
                roles_ok = False
        if not roles_ok:
            self._needle_loc_banner.setText(
                "Assign Needle X-view and Needle Y-view roles to two "
                "live cameras in Hardware Setup → Cameras to enable "
                "this workflow."
            )
            self._needle_loc_banner.setStyleSheet(
                f"background-color: {COLORS['surface0']}; "
                f"color: {COLORS['yellow']}; padding: {sp(8)}; "
                f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        else:
            self._needle_loc_banner.setText(
                "Step 1 of 4: bring the needle into both camera views using "
                "the jog controls on the left. Then click the needle's left "
                "and right edges in the X-view, then the Y-view."
            )
            self._needle_loc_banner.setStyleSheet(
                f"background-color: {COLORS['surface0']}; "
                f"color: {COLORS['text']}; padding: {sp(8)}; "
                f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")

    def _needle_loc_on_click(
        self, role: "CameraRole", px: float, py: float
    ) -> None:
        """Route a camera-feed click into the current edge-pick step."""
        if self._needle_loc_picks is None:
            return
        step = self._needle_loc_step
        if role == CameraRole.NEEDLE_X:
            if step == 0:
                self._needle_loc_picks.x_view_left_px = px
                self._needle_loc_rows["x_left"] = py
                self._needle_loc_step = 1
            elif step == 1:
                self._needle_loc_picks.x_view_right_px = px
                self._needle_loc_rows["x_right"] = py
                self._needle_loc_step = 2
        elif role == CameraRole.NEEDLE_Y:
            if step == 2:
                self._needle_loc_picks.y_view_left_px = px
                self._needle_loc_rows["y_left"] = py
                self._needle_loc_step = 3
            elif step == 3:
                self._needle_loc_picks.y_view_right_px = px
                self._needle_loc_rows["y_right"] = py
                self._needle_loc_step = 4
        self._needle_loc_update_ui()

    def _needle_loc_reset(self) -> None:
        if self._needle_loc_picks is None:
            return
        self._needle_loc_picks = TwoCameraEdgePicks()
        self._needle_loc_rows = {}
        self._needle_loc_step = 0
        self._needle_loc_update_ui()

    def _needle_loc_update_ui(self) -> None:
        if self._needle_loc_picks is None:
            return
        picks = self._needle_loc_picks
        labels = self._needle_loc_pick_labels

        def _fmt(v):
            return f"{v:.1f} px" if v is not None else "—"

        labels["x_left"].setText(_fmt(picks.x_view_left_px))
        labels["x_right"].setText(_fmt(picks.x_view_right_px))
        labels["y_left"].setText(_fmt(picks.y_view_left_px))
        labels["y_right"].setText(_fmt(picks.y_view_right_px))

        step_texts = [
            "Click the tip's BOTTOM-LEFT corner in X-view",
            "Click the tip's BOTTOM-RIGHT corner in X-view",
            "Click the tip's BOTTOM-LEFT corner in Y-view",
            "Click the tip's BOTTOM-RIGHT corner in Y-view",
            "All corners picked — review then click Center & Save",
        ]
        self._needle_loc_step_label.setText(
            f"Step {min(self._needle_loc_step + 1, 4)} / 4: "
            f"{step_texts[self._needle_loc_step]}"
        )

        # Preview offset if everything is picked and we have µm/px.
        ready = picks.complete()
        self._needle_loc_btn_center.setEnabled(ready)
        if ready:
            try:
                dx, dy = self._needle_loc_compute_offset_um()
                text = f"Offset preview: ΔX = {dx:+.1f} µm, ΔY = {dy:+.1f} µm"
                if self._needle_loc_z_center_chk.isChecked():
                    dz = self._needle_loc_compute_z_offset_um()
                    if dz is not None:
                        text += f", ΔZ = {dz / 1000.0:+.3f} mm (up = +)"
                    else:
                        text += ", ΔZ = — (no µm/px)"
                # Per-camera column offset (px from frame center) + angle — the
                # most telling numbers for diagnosing a mis-center. A needle
                # visually on the crosshair should read col≈0 in BOTH views.
                diag = self._needle_loc_offset_breakdown()
                if diag:
                    text += "\n" + diag
                if self._needle_loc_already_centered():
                    text += ("\nOn crosshair (within needle width) — "
                             "Save records origin without moving.")
                self._needle_loc_offset_label.setText(text)
            except Exception as e:
                self._needle_loc_offset_label.setText(
                    f"Offset preview: cannot compute ({e})"
                )
        else:
            self._needle_loc_offset_label.setText("Offset preview: —")

    def _needle_loc_camera_info(
        self, role: "CameraRole"
    ) -> tuple[float, int, float | None] | None:
        """Return (um_per_px, frame_width_px, rotation_deg) for the camera
        with `role`, or None if unresolvable. ``rotation_deg`` is None when
        the camera's in-plane rotation has not been measured."""
        if CameraRole is None or self._camera_manager is None:
            return None
        hw = getattr(self, '_hardware_config', None)
        if hw is None:
            return None
        cam_idx = hw.camera_for_role(role)
        if cam_idx is None:
            return None
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            return None
        # v7.5.x: µm/px lives on the shared CameraManager (written by the
        # needle-camera calibration in Hardware Setup → Cameras), not on the
        # CameraWidget. Reading `getattr(cam, "_um_per_px")` always missed and
        # forced the "cannot compute µm/px" path even after calibration. Gate
        # on the explicit calibration flag so an uncalibrated camera still
        # fails cleanly (the manager seeds a 1.67 default we must not trust).
        if self._camera_manager.is_um_per_px_calibrated(cam_idx):
            um_per_px = float(self._camera_manager.get_um_per_px(cam_idx) or 0.0)
        else:
            um_per_px = 0.0
        # Best-effort frame width: prefer last captured frame size.
        frame = None
        try:
            frame = cam.get_current_frame()
        except Exception:
            frame = None
        if frame is not None and frame.ndim >= 2:
            frame_w = int(frame.shape[1])
        else:
            frame_w = 0
        if um_per_px <= 0 or frame_w <= 0:
            return None
        # v7.5.x: in-plane rotation (deg) measured during µm/px calibration —
        # the stage direction that maps to this camera's lateral image axis.
        # None when unmeasured (aligner falls back to the nominal mounting).
        rotation_deg = self._camera_manager.get_rotation_deg(cam_idx)
        return um_per_px, frame_w, rotation_deg

    def _needle_loc_compute_offset_um(self) -> tuple[float, float]:
        """Run the two-camera aligner; raises ValueError if not ready."""
        if not NEEDLE_LOCATION_AVAILABLE or self._needle_loc_picks is None:
            raise ValueError("NeedleLocation: aligner unavailable")
        x_info = self._needle_loc_camera_info(CameraRole.NEEDLE_X)
        y_info = self._needle_loc_camera_info(CameraRole.NEEDLE_Y)
        if x_info is None or y_info is None:
            raise ValueError("camera µm/px or frame size not available")
        # v7.5.x: per-camera rotation → absolute column→stage direction angle
        # for the aligner. The measured value IS that lateral direction (the
        # move direction that produced clean lateral motion); pass it directly,
        # or None to let the aligner use its nominal orthogonal mounting.
        aligner = TwoCameraNeedleAligner(
            um_per_px_x_view=x_info[0],
            um_per_px_y_view=y_info[0],
            frame_width_x_view=x_info[1],
            frame_width_y_view=y_info[1],
            angle_x_view_deg=x_info[2],
            angle_y_view_deg=y_info[2],
        )
        return aligner.offset_from_edge_clicks(self._needle_loc_picks)

    def _needle_loc_compute_z_offset_um(self) -> float | None:
        """Height-frame Z move (µm, up = +) that lands the needle tip's
        center-bottom on the crosshair, averaged over both side cameras.

        Each side camera's image ROWS map to stage Z. The midpoint of a view's
        two corner clicks is the tip-bottom row; its offset from the frame's
        vertical center, scaled by µm/px, is the needle's apparent vertical
        offset. Default sign: tip below the crosshair (row past center →
        +vertical) ⇒ retract (move up, +height); the *Invert Z* toggle flips it
        for vertically-mirrored mounts. Returns None if neither view has a
        calibrated µm/px / live frame.
        """
        if (CameraRole is None or self._camera_manager is None
                or not self._needle_loc_rows):
            return None
        hw = getattr(self, '_hardware_config', None)
        if hw is None:
            return None
        invert = (hasattr(self, "_needle_loc_z_invert_chk")
                  and self._needle_loc_z_invert_chk.isChecked())
        estimates: list[float] = []
        for role, left_key, right_key in (
            (CameraRole.NEEDLE_X, "x_left", "x_right"),
            (CameraRole.NEEDLE_Y, "y_left", "y_right"),
        ):
            if left_key not in self._needle_loc_rows or \
                    right_key not in self._needle_loc_rows:
                continue
            cam_idx = hw.camera_for_role(role)
            if cam_idx is None:
                continue
            if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
                continue
            um_per_px = float(
                self._camera_manager.get_um_per_px(cam_idx) or 0.0)
            if um_per_px <= 0:
                continue
            try:
                frame = self._camera_manager.cameras[cam_idx].get_current_frame()
            except (AttributeError, IndexError):
                frame = None
            if frame is None or frame.ndim < 2 or frame.shape[0] <= 0:
                continue
            frame_h = int(frame.shape[0])
            tip_row = (self._needle_loc_rows[left_key]
                       + self._needle_loc_rows[right_key]) / 2.0
            row_offset_px = tip_row - (frame_h / 2.0)  # + = below center
            estimates.append(row_offset_px * um_per_px)
        if not estimates:
            return None
        height_um = sum(estimates) / len(estimates)
        return -height_um if invert else height_um

    def _needle_loc_offset_breakdown(self) -> str:
        """Compact per-camera column-offset string for the preview label.

        ``X: col=+3.1px @135°  Y: col=−1.8px @45°`` — col is the needle center's
        signed pixel offset from the frame center. On the crosshair both ≈ 0.
        Returns '' if a view's clicks/µm-px aren't available.
        """
        picks = self._needle_loc_picks
        if picks is None:
            return ""
        parts = []
        for role, tag, lpx, rpx in (
            (CameraRole.NEEDLE_X, "X",
             getattr(picks, "x_view_left_px", None),
             getattr(picks, "x_view_right_px", None)),
            (CameraRole.NEEDLE_Y, "Y",
             getattr(picks, "y_view_left_px", None),
             getattr(picks, "y_view_right_px", None)),
        ):
            info = self._needle_loc_camera_info(role)
            if info is None or lpx is None or rpx is None:
                continue
            _upp, frame_w, rot = info
            off_px = (lpx + rpx) / 2.0 - frame_w / 2.0
            ang = "—" if rot is None else f"{rot:.0f}°"
            parts.append(f"{tag}: col={off_px:+.1f}px @{ang}")
        return "  ".join(parts)

    def _needle_loc_already_centered(self) -> bool:
        """True if the crosshair (frame center) lies BETWEEN the two edge
        clicks in BOTH views — i.e. the needle already covers the optical
        center to within its own width, so no recenter move is needed.

        Returns False if either view's clicks / frame size are unavailable (so
        the caller falls back to moving), or if the crosshair is outside the
        needle span in either view.
        """
        picks = self._needle_loc_picks
        if picks is None:
            return False
        checked = 0
        for role, lpx, rpx in (
            (CameraRole.NEEDLE_X,
             getattr(picks, "x_view_left_px", None),
             getattr(picks, "x_view_right_px", None)),
            (CameraRole.NEEDLE_Y,
             getattr(picks, "y_view_left_px", None),
             getattr(picks, "y_view_right_px", None)),
        ):
            info = self._needle_loc_camera_info(role)
            if info is None or lpx is None or rpx is None:
                return False
            _upp, frame_w, _rot = info
            center = frame_w / 2.0
            if not (min(lpx, rpx) <= center <= max(lpx, rpx)):
                return False
            checked += 1
        return checked == 2

    def _needle_loc_log_diagnostics(self, dx_um: float, dy_um: float) -> None:
        """Log the full needle-centering computation for HW debugging.

        Emits, per side camera: the two edge clicks, their column midpoint, the
        frame width + its center, the column offset (px and µm), the µm/px, and
        the rotation angle fed to the aligner — then the resulting XY move and
        the current stage position. Lets a mis-center be pinned to a specific
        input rather than guessed.
        """
        try:
            picks = self._needle_loc_picks
            lines = ["[needle-loc] ── Center & Save diagnostics ──"]
            for role, name, lpx, rpx in (
                (CameraRole.NEEDLE_X, "X-view",
                 getattr(picks, "x_view_left_px", None),
                 getattr(picks, "x_view_right_px", None)),
                (CameraRole.NEEDLE_Y, "Y-view",
                 getattr(picks, "y_view_left_px", None),
                 getattr(picks, "y_view_right_px", None)),
            ):
                info = self._needle_loc_camera_info(role)
                if info is None or lpx is None or rpx is None:
                    lines.append(f"  {name}: info/clicks unavailable "
                                 f"(L={lpx}, R={rpx}, info={info})")
                    continue
                upp, frame_w, rot = info
                mid = (lpx + rpx) / 2.0
                off_px = mid - frame_w / 2.0
                lines.append(
                    f"  {name}: L={lpx:.1f} R={rpx:.1f} mid={mid:.1f} px | "
                    f"width={frame_w} center={frame_w / 2.0:.1f} | "
                    f"col_offset={off_px:+.1f}px ({off_px * upp:+.1f}µm) | "
                    f"um/px={upp:.4f} | angle={rot}")
            try:
                xy = self.controller.get_xy_position(cached=False)
            except Exception:
                xy = None
            lines.append(f"  -> move dx={dx_um:+.1f}µm dy={dy_um:+.1f}µm "
                         f"| XY before (raw)={xy}")
            logger.info("\n".join(lines))
        except Exception as e:
            logger.debug(f"needle-loc diagnostics failed: {e}")

    def _needle_loc_center_and_save(self) -> None:
        """Drive the stage to recenter the needle, then save
        ``needle_origin_um`` as the current stage position."""
        if not NEEDLE_LOCATION_AVAILABLE:
            return
        try:
            dx_um, dy_um = self._needle_loc_compute_offset_um()
        except Exception as e:
            QMessageBox.warning(
                self, "Needle Location",
                f"Cannot compute offset: {e}")
            return

        if self.controller is None:
            QMessageBox.warning(
                self, "Needle Location",
                "Stage controller not connected.")
            return

        # v7.5.x diagnostics: log the full per-camera breakdown so a mis-center
        # can be traced to clicks / frame width / µm/px / angle vs. the move.
        self._needle_loc_log_diagnostics(dx_um, dy_um)

        # v7.5.x deadband: if the crosshair already falls WITHIN the needle (the
        # frame center lies between the two edge clicks) in BOTH views, the
        # needle is already on the crosshair to within its own width — record the
        # origin WITHOUT an XY move. This stops near-center click-noise (which the
        # 2-camera solve amplifies) from nudging a hand-centered needle off the
        # crosshair. Large offsets (crosshair outside the needle span) still move.
        if self._needle_loc_already_centered():
            logger.info(
                "[needle-loc] crosshair already within the needle span in both "
                "views — recording origin without an XY move "
                f"(would-be move dx={dx_um:+.1f} dy={dy_um:+.1f} µm).")
        else:
            try:
                self.controller.move_xy_relative_um(dx_um, dy_um)
            except Exception as e:
                QMessageBox.critical(
                    self, "Needle Location",
                    f"Move failed: {e}")
                return
            try:
                xy_after = self.controller.get_xy_position(cached=False)
                logger.info(f"[needle-loc] XY after move (raw): {xy_after}")
            except Exception:
                pass

        # v7.5.x: optionally center Z so the tip's center-bottom lands on the
        # crosshair. Height-frame move (up = +), soft-limit + print-floor
        # clamped by move_z_user_relative. Best-effort: a failure here does not
        # block saving the XY origin.
        if (hasattr(self, "_needle_loc_z_center_chk")
                and self._needle_loc_z_center_chk.isChecked()):
            try:
                dz_um = self._needle_loc_compute_z_offset_um()
                if dz_um is not None and abs(dz_um) >= 1.0:
                    self.controller.move_z_user_relative(dz_um / 1000.0)
            except Exception as e:
                logger.warning(f"NeedleLocation Z centering failed: {e}")

        # Capture the post-move stage XY as the needle origin.
        try:
            xy = self.controller.get_xy_position(cached=False)
        except Exception:
            xy = (None, None)
        if xy and xy[0] is not None:
            zero = self.controller.zero_position
            origin_x_um = stage_to_um(xy[0] - zero.get("x", 0),
                                      self._xy_position_scale)
            origin_y_um = stage_to_um(xy[1] - zero.get("y", 0),
                                      self._xy_position_scale)
            self._needle_origin_um = (origin_x_um, origin_y_um)
            self._needle_loc_origin_label.setText(
                f"needle_origin_um: ({origin_x_um:.1f}, {origin_y_um:.1f}) µm"
            )
            self._needle_loc_origin_label.setStyleSheet(
                f"color: {COLORS['green']}; padding-top: {sp(6)};")
            try:
                self._emit_calibration_data_changed()
            except Exception:
                pass
            # v7.5.x: also persist the ABSOLUTE stage XY as the quick-move
            # needle location so the operator can drive straight back here next
            # session (the side cameras are fixed to the frame).
            self._needle_loc_store_xy(xy[0], xy[1])

        # v7.5.x: capture the needle-tip-camera Z fiducial. The edge clicks
        # select the tip's bottom corners, so once centered (incl. the optional
        # Z move above) the needle sits at a repeatable Z. Record it (user
        # frame) so the plate Z references can be PRE-FILLED from the standard
        # offsets. The Z move is a small centering delta still in flight, so this
        # is an approximate fiducial; refine via "Estimate plate Z" if needed.
        try:
            raw_z = self.controller.capture_current_z_raw()
            if raw_z is not None:
                cam_user = self.controller.set_needle_cam_z_from_raw(raw_z)
                if self.settings is not None:
                    self.settings.set("device_profile.needle_cam_z", cam_user)
                    self.settings.save()
                if hasattr(self, "_zoff_lbl_needle_cam"):
                    self._zoff_lbl_needle_cam.setText(
                        f"Needle-cam Z: {cam_user:.2f} mm — click "
                        f"'Estimate plate Z' to pre-fill guesses.")
                    self._zoff_lbl_needle_cam.setStyleSheet(
                        f"color: {COLORS['green']}; font-size: 9pt;")
        except Exception as e:
            logger.debug(f"needle-cam Z capture skipped: {e}")

        # Reset for the next iteration.
        self._needle_loc_reset()

    # ── v7.5.x: quick-move to the saved approximate needle location ──

    def _needle_loc_store_xy(self, x_um: float, y_um: float) -> None:
        """Persist (x, y) absolute Prior stage µm as the approximate needle
        location and refresh the quick-move UI."""
        self._needle_loc_xy_um = (float(x_um), float(y_um))
        if self.settings is not None:
            try:
                self.settings.set("device_profile.needle_loc_xy_um",
                                  [float(x_um), float(y_um)])
                self.settings.save()
            except Exception as e:
                logger.debug(f"needle-loc XY persist skipped: {e}")
        self._needle_loc_update_goto_ui()

    def _needle_loc_update_goto_ui(self) -> None:
        """Enable the go-to button + show the saved location (zero-ref µm)."""
        if not hasattr(self, "_needle_loc_btn_goto"):
            return
        loc = getattr(self, "_needle_loc_xy_um", None)
        self._needle_loc_btn_goto.setEnabled(bool(loc))
        if not loc:
            self._needle_loc_goto_label.setText(
                "Saved needle location: not set — jog the needle into view + "
                "'Set current as location', or run Center & Save once.")
            return
        # Display zero-referenced µm to match the rest of the UI.
        zx = zy = 0.0
        try:
            zero = self.controller.zero_position
            zx, zy = zero.get("x", 0.0), zero.get("y", 0.0)
        except Exception:
            pass
        self._needle_loc_goto_label.setText(
            f"Saved needle location: ({loc[0] - zx:,.0f}, "
            f"{loc[1] - zy:,.0f}) µm")

    def _needle_loc_set_current(self) -> None:
        """Capture the current stage XY as the approximate needle location
        (seeds the quick-move on a fresh machine before the first calibration)."""
        if self.controller is None:
            QMessageBox.warning(self, "Needle Location",
                                "Stage controller not connected.")
            return
        try:
            xy = self.controller.get_xy_position(cached=False)
        except Exception:
            xy = (None, None)
        if not xy or xy[0] is None:
            QMessageBox.warning(self, "Needle Location",
                                "Could not read the stage position.")
            return
        self._needle_loc_store_xy(xy[0], xy[1])
        logger.info("[needle-loc] saved needle location (abs µm): "
                    f"({xy[0]:.1f}, {xy[1]:.1f})")

    def _needle_loc_use_last_known(self) -> None:
        """v7.5.x: accept the last-known needle calibration without re-centering.

        When the operator is certain that nothing about the needle, its
        mounting, or the side cameras has changed since the last session, this
        restores the saved needle reference — needle zero (the workspace
        reference frame), the needle-cam Z fiducial, and the saved needle XY —
        and marks the needle location as established. It is a software-only
        restore (no stage motion), so the operator can skip the camera-based
        Center & Save workflow entirely.

        All three values are read from the persisted ``settings`` (where the
        needle calibration writes them), so this is immune to whatever the live
        page state happens to be.
        """
        if self.settings is None:
            QMessageBox.warning(
                self, "Needle Location",
                "Settings unavailable — cannot load the last known needle "
                "location.")
            return

        # ── Gather the persisted needle reference ────────────────
        saved_xy = None
        try:
            _xy = self.settings.get("device_profile.needle_loc_xy_um")
            if _xy and len(_xy) >= 2:
                saved_xy = (float(_xy[0]), float(_xy[1]))
        except Exception:
            saved_xy = None

        saved_cam_z = None
        try:
            _cz = self.settings.get("device_profile.needle_cam_z")
            if _cz is not None:
                saved_cam_z = float(_cz)
        except Exception:
            saved_cam_z = None

        saved_zero = None
        try:
            _z = self.settings.get_section("zero_position")
            if isinstance(_z, dict) and _z:
                saved_zero = _z
        except Exception:
            saved_zero = None

        if saved_xy is None and saved_cam_z is None:
            QMessageBox.information(
                self, "Needle Location",
                "No last-known needle location is saved yet.\n\n"
                "Run 'Center & Save needle origin' once (or 'Set current as "
                "location') so it can be reused next session.")
            return

        # ── Confirm — the operator is asserting nothing changed ──
        zx = zy = 0.0
        try:
            zero = self.controller.zero_position if self.controller else {}
            zx, zy = zero.get("x", 0.0), zero.get("y", 0.0)
        except Exception:
            pass
        lines = []
        if saved_xy is not None:
            lines.append(
                f"• Needle XY: ({saved_xy[0] - zx:,.0f}, "
                f"{saved_xy[1] - zy:,.0f}) µm")
        if saved_cam_z is not None:
            lines.append(f"• Needle-cam Z: {saved_cam_z:.2f} mm")
        if saved_zero is not None:
            lines.append("• Needle zero reference")

        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Question)
        box.setWindowTitle("Use last known needle location?")
        box.setText(
            "Restore the last-known needle calibration and skip the "
            "camera-based re-centering?")
        box.setInformativeText(
            "Only do this if NOTHING about the needle, its mounting, or the "
            "side cameras has changed since it was last saved — otherwise the "
            "needle reference will be wrong.\n\nRestoring:\n"
            + "\n".join(lines))
        use_btn = box.addButton(
            "Use last known", QMessageBox.ButtonRole.AcceptRole)
        box.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(use_btn)
        box.exec()
        if box.clickedButton() is not use_btn:
            return

        # ── Apply the saved needle reference (software only; no motion) ──
        # Needle zero — re-establish the workspace reference frame first so the
        # derived origin below uses the restored zero.
        if saved_zero is not None and self.controller is not None:
            try:
                self.controller.zero_position.update(
                    {k: v for k, v in saved_zero.items()
                     if isinstance(v, (int, float))})
            except Exception as e:
                logger.warning(
                    f"[needle-loc] restore zero_position failed: {e}")

        # Needle-cam Z fiducial.
        if saved_cam_z is not None and self.controller is not None:
            try:
                self.controller.set_needle_cam_z_user(saved_cam_z)
            except Exception as e:
                logger.debug(f"[needle-loc] restore needle-cam Z failed: {e}")
            if hasattr(self, "_zoff_lbl_needle_cam"):
                self._zoff_lbl_needle_cam.setText(
                    f"Needle-cam Z: {saved_cam_z:.2f} mm (last known) — click "
                    f"'Estimate plate Z' to pre-fill guesses.")
                self._zoff_lbl_needle_cam.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: 9pt;")

        # Saved needle XY + derived origin (zero-ref µm).
        if saved_xy is not None:
            self._needle_loc_xy_um = saved_xy
            try:
                zero = self.controller.zero_position if self.controller else {}
                origin_x = stage_to_um(saved_xy[0] - zero.get("x", 0),
                                       self._xy_position_scale)
                origin_y = stage_to_um(saved_xy[1] - zero.get("y", 0),
                                       self._xy_position_scale)
                self._needle_origin_um = (origin_x, origin_y)
                self._needle_loc_origin_label.setText(
                    f"needle_origin_um: ({origin_x:.1f}, {origin_y:.1f}) "
                    f"µm (last known)")
                self._needle_loc_origin_label.setStyleSheet(
                    f"color: {COLORS['green']}; padding-top: {sp(6)};")
            except Exception as e:
                logger.debug(f"[needle-loc] derive origin failed: {e}")

        try:
            self._emit_calibration_data_changed()
        except Exception:
            pass
        self._needle_loc_update_goto_ui()
        self._needle_loc_banner.setText(
            "✓ Loaded the last-known needle location. Verify the needle "
            "is in view ('⤵ Go to needle location'); re-center only if it "
            "looks off.")
        self._needle_loc_banner.setStyleSheet(
            f"background-color: {COLORS['surface0']}; "
            f"color: {COLORS['green']}; padding: {sp(8)}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        logger.info(
            "[needle-loc] restored last-known needle reference "
            f"(xy={saved_xy}, cam_z={saved_cam_z})")

    def _needle_loc_goto(self) -> None:
        """Quick-move to the saved approximate needle location so the needle
        re-enters both side views. Safe-travels: retract Z to the Fast-Move
        (Safe) height → XY → lower to the needle-cam Z (where the needle sat
        when last centered). Honors the retract-before-XY safety rule."""
        loc = getattr(self, "_needle_loc_xy_um", None)
        if not loc:
            QMessageBox.information(self, "Needle Location",
                "No saved needle location yet. Jog the needle into the side "
                "cameras and click 'Set current as location', or run "
                "'Center & Save' once.")
            return
        if self.controller is None:
            QMessageBox.warning(self, "Needle Location",
                                "Stage controller not connected.")
            return
        # Safety: a retract needs a known Safe Z (Fast-Move Z). With ZP
        # connected and no Safe Z, the retract falls back to zero-ref 0 (the
        # bottom datum on ME3B V1) — a crash. Gate like the plate run.
        if (self.controller.is_zp_connected
                and getattr(self, "_safe_z", None) is None):
            QMessageBox.warning(self, "Needle Location",
                "Set the Fast Move (Safe) Z on the Needle Offset tab first so "
                "the needle can retract before traveling.")
            return
        x_um, y_um = float(loc[0]), float(loc[1])
        # Z target = the needle-cam Z (user frame) → zero-ref, if captured.
        target_z = None
        try:
            cam_z = self.controller.get_needle_cam_z_user()
            if cam_z is not None:
                target_z = self.controller.user_z_to_zref(float(cam_z))
        except Exception:
            target_z = None
        # Blocking GUI-thread move — disable the buttons + repaint so the
        # operator sees it working (mirrors the other calibration gotos).
        self._needle_loc_btn_goto.setEnabled(False)
        self._needle_loc_btn_set.setEnabled(False)
        try:
            self._needle_loc_btn_goto.repaint()
        except Exception:
            pass
        try:
            self._safe_navigate_to(
                x_um, y_um,
                target_z_mm=target_z,
                lower_z=(target_z is not None))
        except Exception as e:
            QMessageBox.critical(self, "Needle Location",
                                 f"Move failed: {e}")
        finally:
            self._needle_loc_btn_set.setEnabled(True)
            self._needle_loc_update_goto_ui()

    # ════════════════════════════════════════════════════════════════
    #  v7.4.4: Reference-Z heights — Z handlers (v7.5.x: merged into the
    #  Needle Location tab as its right pane; no longer a standalone tab)
    # ════════════════════════════════════════════════════════════════

    def _build_z_offset_content(self) -> QWidget:
        """Right pane of the Needle Location tab (formerly the standalone
        "Needle Offset Calibration" tab): capture the needle's vertical
        reference heights (Replace / Max / Fast-Move / Plate-Top / Plate-Bottom
        Z) and pre-fill plate-Z guesses from the needle-cam fiducial. These come
        BEFORE Plate Location because the plate calibration retracts to the
        Fast-Move / Safe Z between wells. The focus-based per-well Z-bottom
        auto-cal is on the separate "Plate Z Auto-Cal" tab (after Plate
        Location), since it needs the finished XY map."""
        page = QWidget()
        page_lay = QVBoxLayout(page)
        page_lay.setContentsMargins(0, 0, 0, 0)
        page_lay.setSpacing(0)

        controls = QWidget()
        outer = QVBoxLayout(controls)
        outer.setContentsMargins(s(10), s(10), s(10), s(10))
        outer.setSpacing(s(8))

        intro = QLabel(
            "Needle offset calibration anchors the needle's vertical travel. "
            "Capture Fast Move Z (travel height) and Plate Top Z (plate "
            "surface) here — Plate Location uses them to retract safely "
            "between wells. Per-well Z-bottom auto-cal is on the Plate Z "
            "Auto-Cal tab (after Plate Location)."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['subtext0']};")
        outer.addWidget(intro)

        # ── Reference Z heights (top → bottom) ───────────────────
        # v7.4.4: five Z reference heights captured by "set current Z"
        # buttons, ordered from highest (Replace) to lowest (Plate
        # Bottom). Each button captures the current Z relative to the
        # stage zero; the label updates with the saved value.
        z_group = QGroupBox("Reference Z heights (top → bottom)")
        # v7.5.x: lay the capture-button grid (left) beside a live XZ side view
        # (right) in a resizable splitter, so the operator can move Z around
        # quickly while setting the reference heights (mirrors Plate Location).
        z_group_lay = QVBoxLayout(z_group)
        z_group_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        z_group_lay.setSpacing(s(4))
        z_split = QSplitter(Qt.Horizontal)
        z_split.setChildrenCollapsible(False)
        z_split.setHandleWidth(s(4))
        z_grid_widget = QWidget()
        z_grid = QGridLayout(z_grid_widget)
        z_grid.setContentsMargins(0, 0, 0, 0)
        z_grid.setHorizontalSpacing(s(8))
        z_grid.setVerticalSpacing(s(6))

        z_rows = [
            # (button_label, tooltip, handler, label_attr, label_prefix)
            ("Set Replace Z",
             "Needle-swap clearance — highest safe Z when changing "
             "the needle. Set with stage at the position you use "
             "for replacing the needle.",
             self._zoff_set_replace_z, "_zoff_lbl_replace_z", "Replace Z"),
            ("Set Max Z",
             "Soft-limit ceiling — the highest Z the stage may "
             "travel to during normal motion.",
             self._zoff_set_max_z, "_zoff_lbl_max_z", "Max Z"),
            ("Set Fast Move Z",
             "Travel height for fast XY moves between wells "
             "(retract before XY, return below afterward).",
             self._zoff_set_safe_z, "_zoff_lbl_safe_z", "Fast Move Z"),
            ("Set Plate Top Z",
             "Plate's top surface — first contact with the plate "
             "from above.",
             self._zoff_set_top_z, "_zoff_lbl_top_z", "Plate Top Z"),
            ("Set Plate Bottom Z",
             "Well floor — needle just touching the bottom of a "
             "calibrated well.",
             self._zoff_set_plate_bottom_z, "_zoff_lbl_plate_bottom_z",
             "Plate Bottom Z"),
        ]
        for row, (btn_label, tooltip, handler, lbl_attr, prefix) in enumerate(z_rows):
            btn = QPushButton(btn_label)
            btn.setToolTip(tooltip)
            btn.clicked.connect(handler)
            z_grid.addWidget(btn, row, 0)
            lbl = QLabel(f"{prefix}: —")
            lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
            z_grid.addWidget(lbl, row, 1)
            setattr(self, lbl_attr, lbl)
        z_grid.setRowStretch(len(z_rows), 1)
        z_split.addWidget(z_grid_widget)

        # v7.5.x: live XZ side view beside the capture grid. Reuses the same
        # widget + go-to-Z handler as the Plate Location tab; the green "Safe"
        # badge retracts to Fast Move Z, the others drive to each reference.
        from gui.widgets.xz_side_view import XZSideView
        self._zoff_xz_view = XZSideView()
        try:
            if self.controller is not None and hasattr(self.controller, "z_up_sign"):
                self._zoff_xz_view.set_z_display_sign(self.controller.z_up_sign())
        except Exception:
            pass
        _sl = getattr(self.controller, "safety_limits", None) if self.controller else None
        if _sl is not None:
            self._zoff_xz_view.set_safety_limits(_sl)
        try:
            _od = (float(getattr(self._hardware_config.needle, "od_um", 0.0))
                   if self._hardware_config
                   and getattr(self._hardware_config, "needle", None) else 0.0)
            if _od:
                self._zoff_xz_view.set_needle(float(_od))
        except Exception:
            pass
        self._zoff_xz_view.set_z_references(self.get_z_references())
        self._zoff_xz_view.go_to_z_requested.connect(self._on_ploc_go_to_z)

        xz_wrap = QWidget()
        xz_wrap_lay = QVBoxLayout(xz_wrap)
        xz_wrap_lay.setContentsMargins(0, 0, 0, 0)
        xz_wrap_lay.setSpacing(s(4))
        xz_wrap_lay.addWidget(self._zoff_xz_view, stretch=1)
        _xz_hint = QLabel(
            "Click the green “Safe” badge to retract to Fast Move Z; "
            "the other badges drive to each reference height.")
        _xz_hint.setWordWrap(True)
        _xz_hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        xz_wrap_lay.addWidget(_xz_hint)
        z_split.addWidget(xz_wrap)
        z_split.setStretchFactor(0, 2)
        z_split.setStretchFactor(1, 1)
        z_group_lay.addWidget(z_split)

        outer.addWidget(z_group)

        # ── v7.5.x: pre-fill plate Z guesses from the needle-cam fiducial ──
        ncam_group = QGroupBox("Estimate plate Z from needle-cam")
        ncam_lay = QVBoxLayout(ncam_group)
        ncam_info = QLabel(
            "After XY needle calibration captures the needle-tip-camera Z, "
            "click below to PRE-FILL Plate Top / Plate Bottom / Fast-Move Z "
            "with guesses (needle-cam Z minus the standard offsets set on "
            "Hardware Setup → Device). Refine them with the steps below. "
            "Max / Replace Z stay manual."
        )
        ncam_info.setWordWrap(True)
        ncam_info.setStyleSheet(f"color: {COLORS['subtext0']};")
        ncam_lay.addWidget(ncam_info)
        ncam_row = QHBoxLayout()
        self._zoff_btn_estimate = QPushButton("Estimate plate Z (pre-fill guesses)")
        self._zoff_btn_estimate.clicked.connect(
            self._zoff_estimate_from_needle_cam)
        ncam_row.addWidget(self._zoff_btn_estimate)
        ncam_row.addStretch()
        ncam_lay.addLayout(ncam_row)
        self._zoff_lbl_needle_cam = QLabel(
            "Needle-cam Z: not captured (run XY needle calibration).")
        self._zoff_lbl_needle_cam.setWordWrap(True)
        self._zoff_lbl_needle_cam.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        ncam_lay.addWidget(self._zoff_lbl_needle_cam)
        outer.addWidget(ncam_group)

        # ── v7.5.x: learn loop — save the taught Z back to the plate type ──
        learn_group = QGroupBox("Save Z offsets to plate type")
        learn_lay = QVBoxLayout(learn_group)
        learn_info = QLabel(
            "Once you've taught the real heights above (Plate Top / Bottom / "
            "Fast-Move / Max), save them back onto the SELECTED plate type so "
            "the next time you pick that plate the guesses auto-fill "
            "accurately. Built-in products are kept pristine — this writes a "
            "user override. Requires a captured needle-cam Z and a selected "
            "plate type (not 'Generic')."
        )
        learn_info.setWordWrap(True)
        learn_info.setStyleSheet(f"color: {COLORS['subtext0']};")
        learn_lay.addWidget(learn_info)
        learn_row = QHBoxLayout()
        self._zoff_btn_save_to_type = QPushButton(
            "Save measured Z offsets to this plate type")
        self._zoff_btn_save_to_type.clicked.connect(
            self._zoff_save_offsets_to_plate_type)
        learn_row.addWidget(self._zoff_btn_save_to_type)
        learn_row.addStretch()
        learn_lay.addLayout(learn_row)
        self._zoff_lbl_save_to_type = QLabel("")
        self._zoff_lbl_save_to_type.setWordWrap(True)
        self._zoff_lbl_save_to_type.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        learn_lay.addWidget(self._zoff_lbl_save_to_type)
        outer.addWidget(learn_group)

        # ── Quick actions powered by the captured heights ────────
        actions_group = QGroupBox("Quick actions")
        actions_lay = QHBoxLayout(actions_group)
        actions_lay.setSpacing(s(8))
        btn_goto_replace = QPushButton("Go to Replace Z")
        btn_goto_replace.setToolTip(
            "Retract Z to the captured Replace Z (needle-swap height). "
            "XY stays put.")
        btn_goto_replace.clicked.connect(self._zoff_goto_replace_z)
        actions_lay.addWidget(btn_goto_replace)
        actions_lay.addStretch()
        outer.addWidget(actions_group)

        # Pointer to the Plate Z Auto-Cal tab (runs after Plate Location).
        autocal_note = QLabel(
            "Next: finish <b>Plate Location</b> (the XY map), then open "
            "<b>Plate Z Auto-Cal</b> to find each well's bottom by focus."
        )
        autocal_note.setWordWrap(True)
        autocal_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; padding: {sp(6)}; "
            f"background-color: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        outer.addWidget(autocal_note)

        outer.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setFrameShape(QFrame.NoFrame)
        page_lay.addWidget(scroll)
        return page

    def _build_rosette_tab(self) -> QWidget:
        """Workflow tab (after Plate Location): calibrate ROSETTE SUB-WELL
        locations. Pick a rosette-assigned well → mosaic-scan just that well →
        place the sub-well pattern centre on its mosaic (sub-wells auto-place
        from the rosette design geometry) → drag any sub-well to refine →
        confirm. Saved single-well mosaics can be re-mapped without
        re-scanning."""
        page = QWidget()
        page_lay = QVBoxLayout(page)
        page_lay.setContentsMargins(0, 0, 0, 0)
        page_lay.setSpacing(0)

        split = QSplitter(Qt.Vertical)
        split.setChildrenCollapsible(False)
        split.setHandleWidth(s(4))

        controls = QWidget()
        outer = QVBoxLayout(controls)
        outer.setContentsMargins(s(10), s(10), s(10), s(10))
        outer.setSpacing(s(8))

        intro = QLabel(
            "Calibrate rosette SUB-WELL locations from a single-well mosaic. "
            "Plate-level mapping (Plate Location tab) places only the MAIN "
            "wells; here you scan one rosette-assigned well at high "
            "resolution, click the pattern centre, and refine each sub-well. "
            "Requires the plate map (run Plate Location first).")
        intro.setWordWrap(True)
        intro.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        outer.addWidget(intro)

        box = QGroupBox("Rosette sub-well calibration")
        grid = QGridLayout(box)
        grid.setContentsMargins(s(8), s(4), s(8), s(4))
        grid.setHorizontalSpacing(s(6))
        grid.setVerticalSpacing(s(4))
        grid.setColumnStretch(1, 1)
        grid.addWidget(QLabel("Rosette well:"), 0, 0)
        self._rosette_well_combo = QComboBox()
        self._rosette_well_combo.setToolTip(
            "Wells with a rosette insert assigned in the plate designer. "
            "“(scanned)” = a single-well mosaic is already saved for it.")
        self._rosette_well_combo.currentIndexChanged.connect(
            lambda _i: self._rosette_refresh_status())
        grid.addWidget(self._rosette_well_combo, 0, 1)
        self._rosette_btn_scan = QPushButton("Scan rosette well")
        self._rosette_btn_scan.setToolTip(
            "Mosaic-scan just this well (needle retracted to Safe Z), then "
            "open the sub-well mapping: click the pattern centre, refine each "
            "sub-well, confirm. The mosaic is saved per well.")
        self._rosette_btn_scan.clicked.connect(self._rosette_scan_selected)
        grid.addWidget(self._rosette_btn_scan, 1, 0)
        self._rosette_btn_map = QPushButton("Map sub-wells from saved scan…")
        self._rosette_btn_map.setToolTip(
            "Re-open the sub-well mapping on this well's SAVED mosaic — no "
            "re-scan, no stage motion.")
        self._rosette_btn_map.clicked.connect(self._rosette_map_from_saved)
        grid.addWidget(self._rosette_btn_map, 1, 1)
        self._rosette_status = QLabel("")
        self._rosette_status.setWordWrap(True)
        self._rosette_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        grid.addWidget(self._rosette_status, 2, 0, 1, 2)
        outer.addWidget(box)
        outer.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setFrameShape(QFrame.NoFrame)
        split.addWidget(scroll)

        live_group = QGroupBox("Live microscope")
        live_lay = QVBoxLayout(live_group)
        live_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        live_lay.setSpacing(s(4))
        from gui.widgets.camera_feed_view import CameraFeedView
        self._rosette_live_view = CameraFeedView(
            camera_manager=self._camera_manager,
            cam_idx=getattr(self, "_ploc_live_cam_idx", 0),
            show_crosshair=True,
            label="Microscope feed — starts when this tab is shown "
                  "(or start it on the Cameras tab)",
        )
        live_lay.addWidget(self._rosette_live_view, stretch=1)
        split.addWidget(live_group)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        page_lay.addWidget(split)
        self._rosette_refresh_wells()
        return page

    def _rosette_refresh_wells(self) -> None:
        """(Re)populate the rosette-well picker; annotate saved scans."""
        combo = getattr(self, "_rosette_well_combo", None)
        if combo is None:
            return
        current = combo.currentData()
        parents = self._ploc_rosette_parent_wells()
        scanned = set()
        try:
            store = self._ploc_mosaic_store()
            if store is not None and hasattr(store, "list_well_keys"):
                scanned = set(store.list_well_keys(self._ploc_plate_key()))
        except Exception:
            scanned = set()
        combo.blockSignals(True)
        combo.clear()
        for p in parents:
            combo.addItem(f"{p} (scanned)" if p in scanned else p, p)
        if current:
            idx = combo.findData(current)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        combo.blockSignals(False)
        self._rosette_refresh_status()

    def _rosette_refresh_status(self) -> None:
        """Scanned / calibrated state of the selected rosette well."""
        lbl = getattr(self, "_rosette_status", None)
        combo = getattr(self, "_rosette_well_combo", None)
        if lbl is None or combo is None:
            return
        name = combo.currentData()
        if not name:
            lbl.setText(
                "No rosette wells in the active plate — add a rosette to a "
                "well in the plate designer (Hardware Setup → Plate) first.")
            for b in (getattr(self, "_rosette_btn_scan", None),
                      getattr(self, "_rosette_btn_map", None)):
                if b is not None:
                    b.setEnabled(False)
            return
        store = self._ploc_mosaic_store()
        has_scan = bool(store is not None and hasattr(store, "has_well")
                        and store.has_well(self._ploc_plate_key(), name))
        subs = []
        try:
            subs = [w.name for w in self._plate.get_all_wells()
                    if getattr(w, "is_subwell", False)
                    and getattr(w, "parent_well", None) == name]
        except Exception:
            subs = []
        cal = self._calibrated_positions or {}
        n_cal = sum(1 for sname in subs if sname in cal)
        parts = [f"{len(subs)} sub-wells"]
        parts.append("scan saved" if has_scan else "not scanned yet")
        parts.append(f"{n_cal}/{len(subs)} calibrated" if subs else "")
        lbl.setText(f"{name}: " + " · ".join(p for p in parts if p))
        btn_scan = getattr(self, "_rosette_btn_scan", None)
        if btn_scan is not None:
            btn_scan.setEnabled(True)
        btn_map = getattr(self, "_rosette_btn_map", None)
        if btn_map is not None:
            btn_map.setEnabled(has_scan)

    def _rosette_scan_selected(self) -> None:
        combo = getattr(self, "_rosette_well_combo", None)
        name = combo.currentData() if combo is not None else None
        if not name:
            QMessageBox.information(
                self, "Rosettes",
                "No rosette wells in the active plate. Add a rosette to a "
                "well in the plate designer (Hardware Setup → Plate) first.")
            return
        self._ploc_scan_single_well(name, True)

    def _rosette_map_from_saved(self) -> None:
        """Re-open the sub-well mapping on the SAVED single-well mosaic — no
        re-scan, no stage motion."""
        combo = getattr(self, "_rosette_well_combo", None)
        name = combo.currentData() if combo is not None else None
        if not name:
            return
        store = self._ploc_mosaic_store()
        wkey = f"{self._ploc_plate_key()}#{name}"
        if store is None or not store.has(wkey):
            QMessageBox.information(
                self, "Rosettes",
                f"No saved mosaic for {name} yet — run “Scan rosette well” "
                f"first.")
            return
        img = store.load_image(wkey)
        extent = store.get_extent_um(wkey)
        meta = store.get_meta(wkey) or {}
        scale = float(meta.get("mosaic_scale", 0.0) or 0.0)
        if img is None or extent is None or not scale:
            QMessageBox.warning(
                self, "Rosettes",
                f"The saved mosaic for {name} is missing its extent/scale — "
                f"re-scan the well.")
            return
        shift = (store.get_shift_um(wkey)
                 if hasattr(store, "get_shift_um") else (0.0, 0.0))
        self._ploc_open_single_well_mapping(
            name, img, extent, scale, True, shift_um=shift,
            um_per_px=float(meta.get("um_per_px", 0.0) or 0.0))
        self._rosette_refresh_status()

    def _rosette_ensure_live_camera(self) -> None:
        """Point the Rosettes live view at the microscope slot and start it
        (mirrors _zoff_ensure_live_camera)."""
        view = getattr(self, "_rosette_live_view", None)
        if view is None or self._camera_manager is None:
            return
        cam_idx = getattr(self, "_ploc_live_cam_idx", 0)
        try:
            if view.cam_idx != cam_idx:
                view.set_camera(cam_idx)
            self._camera_manager.start(cam_idx)
        except Exception as e:
            logger.debug(f"Rosettes: live camera start failed: {e}")

    def _build_plate_z_autocal_tab(self) -> QWidget:
        """Workflow tab (after Plate Location): focus-based per-well Z-bottom
        auto-cal. The operator teaches the first well's bottom by hand while
        watching the live microscope feed, then the auto-cal walks Z to each
        remaining well's bottom by maximizing focus, seeded from that taught Z.
        Requires the finished Plate Location XY map + Fast Move Z. Wires the
        existing v7.4.2 Z handlers so the math is unchanged."""
        page = QWidget()
        # Controls on top (scrollable), live microscope feed below, split
        # vertically so the operator can watch the needle approach focus while
        # teaching the first spot.
        page_lay = QVBoxLayout(page)
        page_lay.setContentsMargins(0, 0, 0, 0)
        page_lay.setSpacing(0)
        split = QSplitter(Qt.Vertical)
        split.setChildrenCollapsible(False)
        split.setHandleWidth(s(4))

        controls = QWidget()
        outer = QVBoxLayout(controls)
        outer.setContentsMargins(s(10), s(10), s(10), s(10))
        outer.setSpacing(s(8))

        intro = QLabel(
            "Find each calibration well's Z-bottom by focus. Teach the first "
            "well bottom by hand below (jog Z while watching the live feed), "
            "then auto-cal the remaining wells. Requires a finished Plate "
            "Location (the XY map) and Fast Move Z."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['subtext0']};")
        outer.addWidget(intro)

        # ── Guided per-well Z-bottom calibration ─────────────────
        # v7.5.x: each calibration well is taught the same way — retract +
        # travel there, the operator refocuses the microscope on the glass
        # (manual knob) and confirms, then the needle lowers and the
        # best-focus Z (sharpest needle circle = tip at the glass plane) is
        # recorded. The operator can override (Record now) and Accept/Redo.
        guide_group = QGroupBox("Per-well Z-bottom calibration")
        guide_lay = QVBoxLayout(guide_group)
        guide_info = QLabel(
            "For each calibration well the stage retracts and travels there. "
            "Focus the microscope on the glass, then click <b>Confirm focus &amp; "
            "lower needle</b>. The needle lowers and the best-focus Z is "
            "recorded — Accept to keep it, or Redo. Requires a finished Plate "
            "Location (the XY map) and Fast Move Z."
        )
        guide_info.setWordWrap(True)
        guide_info.setStyleSheet(f"color: {COLORS['subtext0']};")
        guide_lay.addWidget(guide_info)

        # Start / Cancel the per-well run.
        run_row = QHBoxLayout()
        self._zoff_btn_run_z = QPushButton("Start Z Auto-Cal")
        self._zoff_btn_run_z.setObjectName("accentBtn")
        self._zoff_btn_run_z.clicked.connect(self._start_auto_z_cal)
        run_row.addWidget(self._zoff_btn_run_z)
        self._zoff_btn_cancel_z = QPushButton("Cancel")
        self._zoff_btn_cancel_z.setMaximumWidth(s(70))
        self._zoff_btn_cancel_z.setEnabled(False)
        self._zoff_btn_cancel_z.clicked.connect(self._cancel_auto_z)
        run_row.addWidget(self._zoff_btn_cancel_z)
        run_row.addStretch()
        guide_lay.addLayout(run_row)

        # Per-well action buttons (enabled per phase by _auto_z_phase_buttons).
        act_row = QHBoxLayout()
        self._zauto_btn_confirm = QPushButton("Confirm focus & lower needle")
        self._zauto_btn_confirm.setObjectName("successBtn")
        self._zauto_btn_confirm.setEnabled(False)
        self._zauto_btn_confirm.setToolTip(
            "Confirm the microscope is focused on the glass at this well. "
            "The needle then lowers to find best focus.")
        self._zauto_btn_confirm.clicked.connect(self._zauto_confirm_focus)
        act_row.addWidget(self._zauto_btn_confirm)
        self._zauto_btn_record = QPushButton("Record now")
        self._zauto_btn_record.setEnabled(False)
        self._zauto_btn_record.setToolTip(
            "Manual override: record the current Z as this well's bottom "
            "(use if auto-detect picks the wrong frame).")
        self._zauto_btn_record.clicked.connect(self._zauto_record_now)
        act_row.addWidget(self._zauto_btn_record)
        act_row.addStretch()
        guide_lay.addLayout(act_row)

        acc_row = QHBoxLayout()
        self._zauto_btn_accept = QPushButton("Accept & next well")
        self._zauto_btn_accept.setObjectName("successBtn")
        self._zauto_btn_accept.setEnabled(False)
        self._zauto_btn_accept.clicked.connect(self._zauto_accept_well)
        acc_row.addWidget(self._zauto_btn_accept)
        self._zauto_btn_redo = QPushButton("Redo well")
        self._zauto_btn_redo.setEnabled(False)
        self._zauto_btn_redo.clicked.connect(self._zauto_redo_well)
        acc_row.addWidget(self._zauto_btn_redo)
        acc_row.addStretch()
        guide_lay.addLayout(acc_row)

        self._zoff_lbl_auto_z = QLabel("Idle — press Start to begin.")
        self._zoff_lbl_auto_z.setWordWrap(True)
        self._zoff_lbl_auto_z.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        guide_lay.addWidget(self._zoff_lbl_auto_z)
        outer.addWidget(guide_group)

        # ── Pointer to power-user features ───────────────────────
        manual_note = QLabel(
            "For per-well manual Z teach, edge-finding, and Z-plane "
            "tilt fit, open <b>Custom → Full Wizard</b>."
        )
        manual_note.setWordWrap(True)
        manual_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; padding: {sp(6)}; "
            f"background-color: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        outer.addWidget(manual_note)

        outer.addStretch(1)

        # Scrollable controls so the live feed always has room below.
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setFrameShape(QFrame.NoFrame)
        split.addWidget(scroll)

        # ── Live microscope feed (passive) ───────────────────────
        live_group = QGroupBox("Live microscope")
        live_lay = QVBoxLayout(live_group)
        live_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        live_lay.setSpacing(s(4))
        from gui.widgets.camera_feed_view import CameraFeedView
        self._zoff_live_view = CameraFeedView(
            camera_manager=self._camera_manager,
            cam_idx=getattr(self, "_ploc_live_cam_idx", 0),
            show_crosshair=True,
            label="Microscope feed — starts when this tab is shown "
                  "(or start it on the Cameras tab)",
        )
        live_lay.addWidget(self._zoff_live_view, stretch=1)
        split.addWidget(live_group)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        page_lay.addWidget(split)
        return page

    # ── v7.4.4: Z reference setters ──────────────────────────────
    #
    # The existing wizard uses ``_safe_z`` (Fast Move Z) and ``_top_z``
    # (Plate Top Z) — those internals stay so the rest of the pipeline
    # (safe-travel logic, soft limits, etc.) keeps working. The three
    # new heights (Replace Z, Max Z, Plate Bottom Z) get fresh
    # attributes plus a shared capture helper.

    def _zoff_capture_current_z(self) -> float | None:
        """Read the current stage Z (mm, zero-referenced) or None.

        Stays zero-ref — that's the storage frame for the Z references and the
        print pipeline. Use :meth:`_zoff_user_z` only when *displaying* it.
        """
        ctrl = getattr(self, "controller", None)
        if ctrl is None:
            return None
        zp = ctrl.get_zp_position(cached=False)
        z_val = ctrl.zp_logical_value(zp, "Z") if zp else None
        if z_val is None:
            return None
        return z_val - ctrl.zero_position.get("Z", 0)

    def _zoff_user_z(self, zref_mm: float) -> float:
        """Zero-ref Z (storage) → unified user frame (0 at bottom datum, up = +)
        for DISPLAY only. Falls back to identity without a controller."""
        ctrl = getattr(self, "controller", None)
        if ctrl is not None and hasattr(ctrl, "zref_to_user_z"):
            return ctrl.zref_to_user_z(zref_mm)
        return zref_mm

    def _zoff_estimate_from_needle_cam(self) -> None:
        """v7.5.x: pre-fill Plate Top / Plate Bottom / Fast-Move Z with guesses
        derived from the needle-cam Z fiducial + the standard offsets. Editable
        — the operator refines them with the manual/auto Z steps. Max/Replace
        stay manual and are not touched."""
        ctrl = getattr(self, "controller", None)
        refs = ctrl.estimate_plate_z_refs() if ctrl is not None else None
        if not refs:
            QMessageBox.information(
                self, "Estimate plate Z",
                "No needle-camera Z has been captured yet. Run the XY needle "
                "calibration on the Needle Location tab first (centering the "
                "tip records its camera Z), then try again.")
            return
        # Pre-fill the references (zero-ref storage) + push to controller.
        # v7.5.x: Max Z is now inherited too (kept as a print reference only —
        # NOT written to safety_limits.z_max; see _zoff_set_max_z).
        self._top_z = refs["plate_top_z"]
        self._plate_bottom_z = refs["plate_bottom_z"]
        self._safe_z = refs["safe_z"]
        if refs.get("plate_max_z") is not None:
            self._max_z = refs["plate_max_z"]
        for lbl_attr, prefix, val in (
            ("_zoff_lbl_top_z", "Plate Top Z", self._top_z),
            ("_zoff_lbl_plate_bottom_z", "Plate Bottom Z", self._plate_bottom_z),
            ("_zoff_lbl_safe_z", "Fast Move Z", self._safe_z),
            ("_zoff_lbl_max_z", "Max Z", getattr(self, "_max_z", None)),
        ):
            lbl = getattr(self, lbl_attr, None)
            if lbl is not None and val is not None:
                lbl.setText(f"{prefix}: {self._zoff_user_z(val):.2f} mm (guess)")
                lbl.setStyleSheet(f"color: {COLORS['yellow']};")
        # Keep the controller's plate datum in sync for printing.
        try:
            ctrl.set_plate_top_z(self._top_z)
            ctrl.set_plate_bottom_z(self._plate_bottom_z)
        except Exception:
            pass
        self._zoff_lbl_needle_cam.setText(
            f"Pre-filled guesses from needle-cam Z "
            f"{ctrl.get_needle_cam_z_user():.2f} mm. Refine below, then save.")
        self._zoff_lbl_needle_cam.setStyleSheet(
            f"color: {COLORS['green']}; font-size: 9pt;")
        self._emit_calibration_data_changed()

    def _apply_plate_type_z_estimates(self, force: bool = False) -> None:
        """v7.5.x: auto-fill plate-Z references from the SELECTED plate type's
        offsets when they aren't already taught.

        Called at the end of ``set_hardware_config`` so selecting a plate type
        makes its Z guesses appear without an extra click. The taught-wins gate
        (fill only when the reference is currently ``None``) means restored /
        manually-taught values are never clobbered; ``force=True`` overrides
        (re-derive from the new type's offsets). Max Z stays a print reference
        only (never touches ``safety_limits.z_max``).
        """
        ctrl = getattr(self, "controller", None)
        if ctrl is None or not hasattr(ctrl, "estimate_plate_z_refs"):
            return
        try:
            refs = ctrl.estimate_plate_z_refs()
        except Exception:
            refs = None
        if not refs:
            return   # no needle-cam fiducial yet → nothing to guess from
        targets = (
            ("_top_z", "plate_top_z", "_zoff_lbl_top_z", "Plate Top Z"),
            ("_plate_bottom_z", "plate_bottom_z",
             "_zoff_lbl_plate_bottom_z", "Plate Bottom Z"),
            ("_safe_z", "safe_z", "_zoff_lbl_safe_z", "Fast Move Z"),
            ("_max_z", "plate_max_z", "_zoff_lbl_max_z", "Max Z"),
        )
        changed = False
        for attr, ref_key, lbl_attr, prefix in targets:
            val = refs.get(ref_key)
            if val is None:
                continue
            if not force and getattr(self, attr, None) is not None:
                continue   # taught / restored value wins
            setattr(self, attr, val)
            changed = True
            lbl = getattr(self, lbl_attr, None)
            if lbl is not None:
                lbl.setText(f"{prefix}: {self._zoff_user_z(val):.2f} mm (guess)")
                lbl.setStyleSheet(f"color: {COLORS['yellow']};")
        if changed:
            try:
                self._emit_calibration_data_changed()
            except Exception:
                pass

    def _zoff_save_offsets_to_plate_type(self) -> None:
        """v7.5.x learn loop: persist the currently-taught Z references back
        onto the SELECTED plate type as mm-below-fiducial offsets, so this
        plate auto-fills accurately next time. Writes a USER override (built-in
        products stay pristine) and refreshes the live guess source."""
        ctrl = getattr(self, "controller", None)
        cam = ctrl.get_needle_cam_z_user() if ctrl is not None else None
        if cam is None:
            QMessageBox.information(
                self, "Save Z offsets",
                "No needle-camera Z fiducial has been captured yet. Run the XY "
                "needle calibration on the Needle Location tab first, then "
                "teach the plate heights and try again.")
            return
        hw = getattr(self, "_hardware_config", None)
        type_id = getattr(hw, "plate_type_id", "") if hw is not None else ""
        if not type_id:
            QMessageBox.information(
                self, "Save Z offsets",
                "Select a specific plate TYPE on Hardware Setup → Plate first "
                "(the generic format has nowhere to store offsets).")
            return
        # offset = fiducial − user_z(reference)  (mm BELOW the fiducial)
        ref_map = (
            ("top", getattr(self, "_top_z", None)),
            ("bottom", getattr(self, "_plate_bottom_z", None)),
            ("safe", getattr(self, "_safe_z", None)),
            ("max", getattr(self, "_max_z", None)),
        )
        new_off = {key: float(cam) - float(ctrl.zref_to_user_z(zref))
                   for key, zref in ref_map if zref is not None}
        if not new_off:
            QMessageBox.information(
                self, "Save Z offsets",
                "No Z references are set yet — teach Plate Top / Bottom / "
                "Fast-Move / Max above first.")
            return
        try:
            from SupportClasses.PlateTypeStore import get_store, PlateType
            store = get_store()
            pt = store.get(type_id)
            if pt is None:
                self._zoff_lbl_save_to_type.setText(
                    f"Plate type '{type_id}' not found in the library.")
                self._zoff_lbl_save_to_type.setStyleSheet(
                    f"color: {COLORS['red']};")
                return
            merged = dict(pt.z_offsets)
            merged.update(new_off)
            updated = PlateType(
                id=pt.id, base_format=pt.base_format,
                display_name=pt.display_name, manufacturer=pt.manufacturer,
                model=pt.model, bottom_material=pt.bottom_material,
                well_depth_mm=pt.well_depth_mm, z_offsets=merged, builtin=False)
            store.save_user(updated)
            # Refresh the live guess source so the next estimate matches.
            ctrl.set_plate_z_offsets(
                top=merged.get("top"), bottom=merged.get("bottom"),
                safe=merged.get("safe"), max=merged.get("max"))
        except Exception as e:
            logger.warning(f"Save Z offsets to plate type failed: {e}")
            self._zoff_lbl_save_to_type.setText(f"Save failed: {e}")
            self._zoff_lbl_save_to_type.setStyleSheet(
                f"color: {COLORS['red']};")
            return
        saved_keys = ", ".join(sorted(new_off))
        self._zoff_lbl_save_to_type.setText(
            f"Saved offsets ({saved_keys}) to plate type '{type_id}'. Future "
            f"selections of this plate will auto-fill these guesses.")
        self._zoff_lbl_save_to_type.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(
            f"Learn loop: saved Z offsets {new_off} to plate type '{type_id}'")

    def _zoff_set_safe_z(self) -> None:
        """Fast Move Z setter — delegates to the legacy ``_set_safe_z``
        so soft-limit / safe-travel callers keep using ``self._safe_z``.
        """
        self._set_safe_z()
        if getattr(self, '_safe_z', None) is not None:
            self._zoff_lbl_safe_z.setText(
                f"Fast Move Z: {self._zoff_user_z(self._safe_z):.2f} mm")
            self._zoff_lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")

    def _zoff_set_top_z(self) -> None:
        """Plate Top Z setter — delegates to legacy ``_set_top_z``."""
        self._set_top_z()
        if getattr(self, '_top_z', None) is not None:
            self._zoff_lbl_top_z.setText(
                f"Plate Top Z: {self._zoff_user_z(self._top_z):.2f} mm")
            self._zoff_lbl_top_z.setStyleSheet(f"color: {COLORS['green']};")

    def _zoff_set_replace_z(self) -> None:
        """Replace Z setter — needle-swap clearance height."""
        z = self._zoff_capture_current_z()
        if z is None:
            return
        self._replace_z = z
        self._zoff_lbl_replace_z.setText(
            f"Replace Z: {self._zoff_user_z(z):.2f} mm")
        self._zoff_lbl_replace_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Replace Z set: {z:.2f} mm (zero-ref)")
        self._emit_calibration_data_changed()

    def _zoff_set_max_z(self) -> None:
        """Max Z setter — captures the upper Z reference height for print
        planning.

        v7.5.x: this no longer writes into the safety envelope. The Z
        soft limits are owned by Hardware Setup → Device
        (``safety_limits.z_min/z_max``). The old behaviour pushed the
        calibrated Max Z into ``z_max``, which clobbered the user's
        device-setup range and — on machines where the needle descends as
        Z *increases* (so Max Z is numerically *below* Plate Bottom Z) —
        inverted the envelope, collapsing ``clamp_z`` to a single point so
        every jog was clamped. The value is still kept as a print/Z
        reference via ``self._max_z``."""
        z = self._zoff_capture_current_z()
        if z is None:
            return
        self._max_z = z
        self._zoff_lbl_max_z.setText(f"Max Z: {self._zoff_user_z(z):.2f} mm")
        self._zoff_lbl_max_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Max Z set: {z:.2f} mm (zero-ref)")
        self._emit_calibration_data_changed()

    def _zoff_set_plate_bottom_z(self) -> None:
        """Plate Bottom Z setter — captures the lower Z reference height
        (well floor) for print planning.

        v7.5.x: this no longer writes into the safety envelope; the Z soft
        limits are owned by Hardware Setup → Device (see
        ``_zoff_set_max_z`` for the inverted-envelope bug this caused). The
        value is still kept as a print/Z reference via
        ``self._plate_bottom_z``."""
        z = self._zoff_capture_current_z()
        if z is None:
            return
        self._plate_bottom_z = z
        self._zoff_lbl_plate_bottom_z.setText(
            f"Plate Bottom Z: {self._zoff_user_z(z):.2f} mm")
        self._zoff_lbl_plate_bottom_z.setStyleSheet(
            f"color: {COLORS['green']};")
        logger.info(f"Plate Bottom Z set: {z:.2f} mm")
        self._emit_calibration_data_changed()

    def _zoff_goto_replace_z(self) -> None:
        """Drive Z to the captured Replace Z for needle swapping.

        Disabled when Replace Z hasn't been set yet. Uses an absolute
        Z move; XY stays put.
        """
        if self.controller is None:
            return
        z = getattr(self, "_replace_z", None)
        if z is None:
            QMessageBox.information(
                self, "Replace Z",
                "Replace Z has not been captured yet. Jog Z to the "
                "needle-swap height and press 'Set Replace Z' first.")
            return
        try:
            self.controller.move_z_absolute(z, from_zero_ref=True)
            logger.info(f"Moved to Replace Z: {z:.2f} mm")
        except Exception as e:
            QMessageBox.critical(
                self, "Replace Z",
                f"Move to Replace Z failed: {e}")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.4: Plate Location tab — click-snap N-well XY map workflow
    # ════════════════════════════════════════════════════════════════

    def _build_plate_location_tab(self) -> QWidget:
        """Workflow tab: user clicks N>3 wells on the plate view, the
        system visits each and fits the known-radius well edge to
        refine the XY affine map."""
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(6))

        # ── Workflow version toggle (v7.5.x) ─────────────────────────
        # The Mosaic tools are the primary plate-calibration workflow; the
        # older per-well well-fit + target-queue flow is kept fully working
        # but DEPRECATED behind this toggle until it is retired. Only widget
        # visibility switches — nothing is torn down. Persisted in
        # settings.json ("plate_location_prefs" → "workflow").
        wf_row_w = QWidget()
        wf_row = QHBoxLayout(wf_row_w)
        wf_row.setContentsMargins(0, 0, 0, 0)
        wf_row.setSpacing(s(6))
        wf_row.addWidget(QLabel("Plate calibration:"))
        self._ploc_workflow_combo = QComboBox()
        self._ploc_workflow_combo.addItem(
            "Mosaic workflow (recommended)", "mosaic")
        self._ploc_workflow_combo.addItem(
            "Legacy well-fit / target queue (deprecated)", "legacy")
        self._ploc_workflow_combo.setToolTip(
            "Which version of plate calibration this tab shows. Mosaic = "
            "scan the plate once + map wells on the stitched image "
            "(recommended). Legacy = drive to each well and fit it by hand "
            "(deprecated; kept until it is removed).")
        self._ploc_workflow_combo.currentIndexChanged.connect(
            self._ploc_on_workflow_changed)
        wf_row.addWidget(self._ploc_workflow_combo, stretch=1)
        wf_row.addStretch()
        outer.addWidget(wf_row_w)

        # ── Status banner ────────────────────────────────────────
        # Text swapped per workflow version by _ploc_set_workflow_mode.
        self._ploc_banner = QLabel(self._PLOC_BANNER_MOSAIC)
        self._ploc_banner.setWordWrap(True)
        self._ploc_banner.setStyleSheet(
            f"background-color: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; padding: {sp(8)}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        outer.addWidget(self._ploc_banner)

        # ── Well-fit mode toggle (v7.5.x) ────────────────────────────
        # Manual (default, per user): click the well edge in the live view
        # at each rim point. Auto: camera edge-detection (legacy path).
        # LEGACY-workflow-only (read solely by the queue run) — hidden in
        # the mosaic workflow.
        self._ploc_mode_row_w = QWidget()
        mode_row = QHBoxLayout(self._ploc_mode_row_w)
        mode_row.setContentsMargins(0, 0, 0, 0)
        mode_row.setSpacing(s(6))
        mode_row.addWidget(QLabel("Well fit:"))
        self._ploc_mode_combo = QComboBox()
        self._ploc_mode_combo.addItem(
            "Manual — click well edge in live view", "manual")
        self._ploc_mode_combo.addItem("Auto — camera edge-detect", "auto")
        self._ploc_mode_combo.setCurrentIndex(0)  # manual default
        self._ploc_mode_combo.currentIndexChanged.connect(
            self._ploc_on_mode_changed)
        mode_row.addWidget(self._ploc_mode_combo, stretch=1)
        mode_row.addStretch()
        outer.addWidget(self._ploc_mode_row_w)

        # ── Horizontal splitter: plate view on left, queue on right
        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(s(4))

        # v7.4.4: use the same top-down XY canvas the Jog page uses
        # (``JogWorkspaceView``) instead of the bespoke legacy
        # ``_CalibrationPlateView``. The two pages now show identical
        # workspace visualizations — safety envelope, plate outline,
        # wells colored by calibrated/approximate status, needle at
        # scale, breadcrumbs, click-to-snap.
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        self._ploc_plate_view = JogWorkspaceView(parent=page)
        sl = getattr(self.controller, "safety_limits", None) if self.controller else None
        if sl is not None:
            self._ploc_plate_view.set_safety_limits(sl)
        # Needle outer diameter (drawn to scale) from the hardware config.
        hw = getattr(self, "_hardware_config", None)
        needle = getattr(hw, "needle", None) if hw is not None else None
        od = getattr(needle, "od_um", None) if needle is not None else None
        if od:
            self._ploc_plate_view.set_needle(float(od))
        # v7.4.6: the Plate Location workflow now accepts BOTH well
        # snapping (Snap mode) and arbitrary freeform points (Free
        # mode), toggled via the built-in Free/Snap control on the
        # canvas. Start in Snap so existing muscle memory is unchanged.
        self._ploc_plate_view.set_target_mode("snap")
        if self._plate is not None:
            self._ploc_plate_view.set_plate(self._plate)
            self._ploc_plate_view.set_well_positions(
                self._wells_in_zero_ref(), "approximate")
        self._ploc_plate_view.well_clicked.connect(self._ploc_on_well_clicked)
        # Free-mode clicks emit position_clicked with zero-ref µm. (Snap
        # mode emits it too, alongside well_clicked — the handler guards
        # on the active mode so wells aren't double-enqueued.)
        self._ploc_plate_view.position_clicked.connect(
            self._ploc_on_position_clicked)

        # v7.5.x: live view area under the plate map — the microscope feed on
        # the left and a LIVE stitched-mosaic preview on the right (the mosaic
        # scan builds it tile-by-tile on a worker thread; this preview updates
        # per tile). The manual click-rim well fit uses the microscope feed.
        live_group = QGroupBox("Live view")
        live_lay = QVBoxLayout(live_group)
        live_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        live_lay.setSpacing(s(4))

        live_split = QSplitter(Qt.Horizontal)
        live_split.setChildrenCollapsible(False)
        live_split.setHandleWidth(s(4))

        from gui.widgets.camera_feed_view import CameraFeedView
        feed_col = QWidget()
        feed_lay = QVBoxLayout(feed_col)
        feed_lay.setContentsMargins(0, 0, 0, 0)
        feed_lay.setSpacing(s(2))
        feed_lay.addWidget(QLabel("Microscope"))
        self._ploc_live_view = CameraFeedView(
            camera_manager=self._camera_manager,
            cam_idx=0,
            show_crosshair=True,
            label="Microscope feed — starts on Run/Mosaic scan (or start it "
                  "on the Cameras tab)",
        )
        self._ploc_live_view.clicked.connect(self._ploc_on_live_view_click)
        feed_lay.addWidget(self._ploc_live_view, stretch=1)
        live_split.addWidget(feed_col)

        # Live mosaic preview.
        prev_col = QWidget()
        prev_lay = QVBoxLayout(prev_col)
        prev_lay.setContentsMargins(0, 0, 0, 0)
        prev_lay.setSpacing(s(2))
        prev_lay.addWidget(QLabel("Mosaic (live)"))
        self._ploc_mosaic_preview = QLabel("No mosaic yet — run a Mosaic scan")
        self._ploc_mosaic_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._ploc_mosaic_preview.setMinimumSize(s(160), s(120))
        self._ploc_mosaic_preview.setStyleSheet(
            f"background-color: {COLORS['base']}; color: {COLORS['subtext0']}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        prev_lay.addWidget(self._ploc_mosaic_preview, stretch=1)
        # Whole preview column (label + image) — hidden in the legacy
        # workflow so the live microscope feed gets the full width.
        self._ploc_mosaic_prev_col = prev_col
        live_split.addWidget(prev_col)
        live_split.setStretchFactor(0, 1)
        live_split.setStretchFactor(1, 1)
        live_lay.addWidget(live_split, stretch=1)

        self._ploc_live_hint = QLabel("")
        self._ploc_live_hint.setWordWrap(True)
        self._ploc_live_hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        live_lay.addWidget(self._ploc_live_hint)

        # v7.5.x: Z side view next to the well layout. Lets the operator
        # retract the needle UP to safe travel manually — clicking the green
        # "Safe" badge drives Z to the Fast Move height (a pure up-Z move;
        # soft limits respected) before/while teaching wells. Mirrors the Jog
        # page's XZ side view.
        from gui.widgets.xz_side_view import XZSideView
        self._ploc_xz_view = XZSideView()
        if self.controller is not None and hasattr(self.controller, "z_up_sign"):
            try:
                self._ploc_xz_view.set_z_display_sign(
                    self.controller.z_up_sign())
            except Exception:
                pass
        if sl is not None:
            self._ploc_xz_view.set_safety_limits(sl)
        if od:
            self._ploc_xz_view.set_needle(float(od))
        self._ploc_xz_view.set_z_references(self.get_z_references())
        self._ploc_xz_view.go_to_z_requested.connect(self._on_ploc_go_to_z)

        xz_group = QGroupBox("Z side view")
        xz_lay = QVBoxLayout(xz_group)
        xz_lay.setContentsMargins(s(4), s(4), s(4), s(4))
        xz_lay.setSpacing(s(4))
        xz_lay.addWidget(self._ploc_xz_view, stretch=1)
        xz_hint = QLabel(
            "Click the green “Safe” badge to retract the needle up "
            "to the Fast Move Z.")
        xz_hint.setWordWrap(True)
        xz_hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        xz_lay.addWidget(xz_hint)

        # Top of the left column: well layout (plate map) + Z side view,
        # side by side.
        plate_row = QSplitter(Qt.Horizontal)
        plate_row.setChildrenCollapsible(False)
        plate_row.setHandleWidth(s(4))
        plate_row.addWidget(self._ploc_plate_view)
        plate_row.addWidget(xz_group)
        plate_row.setStretchFactor(0, 3)
        plate_row.setStretchFactor(1, 1)

        # Left column: plate map + Z side view (top) + live microscope (bottom).
        left_split = QSplitter(Qt.Vertical)
        left_split.setChildrenCollapsible(False)
        left_split.setHandleWidth(s(4))
        left_split.addWidget(plate_row)
        left_split.addWidget(live_group)
        left_split.setStretchFactor(0, 3)
        left_split.setStretchFactor(1, 2)
        splitter.addWidget(left_split)

        # Queue + controls panel.
        ctrl_panel = QWidget()
        ctrl_lay = QVBoxLayout(ctrl_panel)
        ctrl_lay.setContentsMargins(s(6), s(6), s(6), s(6))
        ctrl_lay.setSpacing(s(6))

        # v7.5.x: the well-fit target queue is the LEGACY workflow — kept
        # fully functional but grouped so the "Plate calibration" toggle at
        # the top of the tab can hide it as one unit (mosaic = default).
        self._ploc_queue_group = QGroupBox("Well-fit queue (legacy)")
        qg_lay = QVBoxLayout(self._ploc_queue_group)
        qg_lay.setContentsMargins(s(6), s(4), s(6), s(4))
        qg_lay.setSpacing(s(6))
        self._ploc_legacy_note = QLabel(
            "Deprecated — prefer the Mosaic workflow. This per-well queue "
            "flow will be removed in a future version.")
        self._ploc_legacy_note.setWordWrap(True)
        self._ploc_legacy_note.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 9pt;")
        qg_lay.addWidget(self._ploc_legacy_note)
        qg_lay.addWidget(QLabel("Target queue:"))
        self._ploc_queue_list = QListWidget()
        self._ploc_queue_list.setSelectionMode(
            QListWidget.SelectionMode.SingleSelection)
        qg_lay.addWidget(self._ploc_queue_list, stretch=1)

        btn_row = QHBoxLayout()
        self._ploc_btn_clear = QPushButton("Clear queue")
        self._ploc_btn_clear.clicked.connect(self._ploc_clear_queue)
        btn_row.addWidget(self._ploc_btn_clear)
        btn_row.addStretch()
        self._ploc_btn_run = QPushButton("Run")
        self._ploc_btn_run.setObjectName("accentBtn")
        self._ploc_btn_run.setEnabled(False)
        self._ploc_btn_run.clicked.connect(self._ploc_run_queue)
        btn_row.addWidget(self._ploc_btn_run)
        qg_lay.addLayout(btn_row)
        ctrl_lay.addWidget(self._ploc_queue_group, stretch=1)

        # v7.5.x: full-plate mosaic scan — raster the whole plate, stitch a
        # composite, detect ALL wells on it at once (robust where single-frame
        # per-well edge detection fails), plus a toggle to overlay the stitched
        # field on the plate map.
        # The actions live in a compact 2-column grid inside a "Mosaic" group:
        # a single horizontal row of all six buttons forced the control panel
        # very wide and squeezed the plate + live views.
        mosaic_box = QGroupBox("Mosaic")
        # Kept as an attribute so the workflow toggle can hide the whole
        # group in the legacy well-fit version of this tab.
        self._ploc_mosaic_box = mosaic_box
        mosaic_lay = QVBoxLayout(mosaic_box)
        mosaic_lay.setContentsMargins(s(8), s(4), s(8), s(4))
        mosaic_lay.setSpacing(s(4))

        # v7.5.x main-button list, VERTICAL (operator request): Calibrate
        # mosaic scan · Mosaic scan · Map wells… · Re-anchor mosaic · Auto
        # re-anchor mosaic. Everything else lives in the "Advanced…" menu.
        # Legacy button ATTRS are kept alive as hidden buttons so every
        # handler wire, enable/label refresh, and test keeps working — the
        # menu actions simply .click() them (a disabled button stays a no-op).
        self._ploc_btn_mosaic_cal = QPushButton("Calibrate mosaic scan")
        self._ploc_btn_mosaic_cal.setToolTip(
            "Calibrate the mosaic scan for this camera + objective: build a "
            "small (e.g. 5×5) mosaic, run the global registration to estimate "
            "the alignment delta, and store it so full mosaics start "
            "pre-registered.")
        self._ploc_btn_mosaic_cal.clicked.connect(
            self._ploc_quick_mosaic_calibrate)
        self._ploc_btn_mosaic = QPushButton("Mosaic scan")
        self._ploc_btn_mosaic.setToolTip(
            "Raster the whole plate, stitch every snapshot into one image, "
            "detect all wells, then open the guided well-mapping dialog to "
            "confirm/refine the MAIN wells (rosette sub-wells calibrate on "
            "the Rosettes tab).")
        self._ploc_btn_mosaic.clicked.connect(self._ploc_start_mosaic_scan)
        # v7.5.x: "Map wells…" promoted from the Advanced menu onto the main
        # button list (operator request) — it still auto-opens after every
        # Mosaic scan; this re-opens it on the stored mosaic without a
        # re-scan.
        self._ploc_btn_map_wells = QPushButton("Map wells…")
        self._ploc_btn_map_wells.setToolTip(
            "Re-open the guided well mapping on the stored mosaic without "
            "re-scanning (it auto-opens after every Mosaic scan).")
        self._ploc_btn_map_wells.clicked.connect(self._ploc_open_well_mapping)
        # Manual single-point re-anchor (NO-TRAVEL): jog a feature into the
        # live view + click it, then click the SAME feature on the plate
        # overview; the offset shifts the WHOLE map (wells + mosaic) by a pure
        # translation. The live click also SAVES the feature patch for the
        # hands-free auto re-anchor.
        self._ploc_btn_reanchor = QPushButton("Re-anchor mosaic")
        self._ploc_btn_reanchor.setToolTip(
            "Manual re-anchor for a mis-registered map (e.g. the plate was "
            "remounted): jog a recognizable feature into the LIVE view and "
            "click it, then click that same feature on the plate overview. "
            "The offset shifts the whole map — wells AND mosaic, together. "
            "No stage motion. The live click also saves the feature for "
            "“Auto re-anchor mosaic”.")
        self._ploc_btn_reanchor.clicked.connect(self._ploc_toggle_reanchor)
        self._ploc_btn_auto_reanchor = QPushButton("Auto re-anchor mosaic")
        self._ploc_btn_auto_reanchor.setEnabled(False)
        self._ploc_btn_auto_reanchor.clicked.connect(self._ploc_auto_reanchor)
        for _b in (self._ploc_btn_mosaic_cal, self._ploc_btn_mosaic,
                   self._ploc_btn_map_wells, self._ploc_btn_reanchor,
                   self._ploc_btn_auto_reanchor):
            _b.setSizePolicy(QSizePolicy.Policy.Expanding,
                             QSizePolicy.Policy.Fixed)
            mosaic_lay.addWidget(_b)

        # ── Advanced tools (hidden buttons + one menu) ──────────────
        self._ploc_btn_mosaic_cfg = QPushButton("Settings…")
        self._ploc_btn_mosaic_cfg.setToolTip(
            "Tune the mosaic scan — camera settle/timing, FOV overlap, "
            "resolution, and well-detection sensitivity.")
        self._ploc_btn_mosaic_cfg.clicked.connect(
            self._ploc_open_mosaic_settings)
        # v7.5.x: adopt a mosaic already scanned under ANOTHER plate key.
        self._ploc_btn_load_mosaic = QPushButton(
            "Load mosaic from another plate…")
        self._ploc_btn_load_mosaic.setToolTip(
            "Copy a mosaic already scanned for ANOTHER plate selection to "
            "this plate (the source stays). Its single-well scans come "
            "along. After loading, use Map wells… / Re-anchor to line the "
            "map up with the current mount.")
        self._ploc_btn_load_mosaic.clicked.connect(
            self._ploc_load_mosaic_from_other_plate)
        self._ploc_btn_reregister = QPushButton("Quick re-register")
        self._ploc_btn_reregister.setToolTip(
            "Re-register this plate from a saved mosaic template: the camera "
            "drives to 3 reference wells, auto-detects each, and fits the whole "
            "plate — no full re-scan. Needs the SAME camera + objective.")
        self._ploc_btn_reregister.clicked.connect(self._ploc_quick_reregister)
        self._ploc_btn_scan_well = QPushButton("Scan well…")
        self._ploc_btn_scan_well.setToolTip(
            "Build a high-resolution mosaic of ONE well (any well). A rosette "
            "well maps its sub-wells; a plain well picks its centre. Each "
            "picked centre becomes an anchor for “Re-register from scanned "
            "wells”. Saved per well (never overwrites the plate mosaic).")
        self._ploc_btn_scan_well.clicked.connect(self._ploc_scan_well)
        self._ploc_btn_rederive = QPushButton(
            "Re-derive wells from saved mosaic (ground truth)")
        self._ploc_btn_rederive.setToolTip(
            "Re-label every well from the saved mosaic's detected centres for "
            "the current plate orientation, and store them directly as the "
            "calibration. Use after the plate orientation changed. Discards the "
            "previous taught/warp calibration. No stage motion.")
        self._ploc_btn_rederive.clicked.connect(self._ploc_rederive_wrapper)
        self._ploc_btn_scan_reregister = QPushButton(
            "Re-register from scanned wells (0)")
        self._ploc_btn_scan_reregister.setToolTip(
            "Use the well centres you picked from single-well scans to "
            "re-register the whole plate: 1 well = translation shift; "
            "2 = translation + rotation; 3+ = affine. Anchors reset after "
            "applying. No stage motion.")
        self._ploc_btn_scan_reregister.clicked.connect(
            self._ploc_reregister_from_scanned)
        self._ploc_btn_scan_reregister.setEnabled(False)
        for _b in (self._ploc_btn_mosaic_cfg, self._ploc_btn_load_mosaic,
                   self._ploc_btn_reregister, self._ploc_btn_scan_well,
                   self._ploc_btn_rederive, self._ploc_btn_scan_reregister):
            _b.setParent(mosaic_box)
            _b.setVisible(False)

        view_row = QWidget()
        vrl = QHBoxLayout(view_row)
        vrl.setContentsMargins(0, 0, 0, 0)
        vrl.setSpacing(s(4))
        vrl.addWidget(QLabel("Plate view:"))
        self._ploc_view_combo = QComboBox()
        self._ploc_view_combo.setToolTip(
            "Idealized well grid, the stitched plate mosaic, or both overlaid "
            "(mosaic background + well-location circles).")
        self._ploc_view_combo.addItem("Ideal well", "well")
        self._ploc_view_combo.addItem("Mosaic", "mosaic")
        self._ploc_view_combo.addItem("Mosaic + well", "overlay")
        self._ploc_view_combo.currentIndexChanged.connect(
            self._ploc_on_view_mode_changed)
        vrl.addWidget(self._ploc_view_combo, stretch=1)
        # v7.5.x: overlay the saved single-well mosaics ("Scan well…" /
        # Rosettes tab) onto the plate mosaic — toggleable, default ON.
        self._ploc_wellscan_check = QCheckBox("Well scans")
        self._ploc_wellscan_check.setChecked(True)
        self._ploc_wellscan_check.setToolTip(
            "Overlay the saved single-well mosaics (high-res “Scan well” "
            "images) onto the plate mosaic in every mosaic view.")
        self._ploc_wellscan_check.toggled.connect(
            self._ploc_on_wellscan_toggled)
        vrl.addWidget(self._ploc_wellscan_check)
        mosaic_lay.addWidget(view_row)

        self._ploc_btn_advanced = QPushButton("Advanced… ▾")
        self._ploc_btn_advanced.setToolTip(
            "Less-common tools: scan settings, load a mosaic scanned for "
            "another plate, template quick re-register, re-derive from saved "
            "mosaic, single-well scans + re-register, manual align sliders.")
        from PySide6.QtWidgets import QMenu
        adv_menu = QMenu(self._ploc_btn_advanced)
        self._ploc_advanced_menu = adv_menu
        self._ploc_adv_actions = []
        for _b in (self._ploc_btn_mosaic_cfg, self._ploc_btn_load_mosaic,
                   self._ploc_btn_reregister, self._ploc_btn_scan_well,
                   self._ploc_btn_scan_reregister, self._ploc_btn_rederive):
            _act = adv_menu.addAction(_b.text())
            _act.setToolTip(_b.toolTip())
            _act.triggered.connect(_b.click)      # disabled button = no-op
            self._ploc_adv_actions.append((_act, _b))
        adv_menu.addSeparator()
        self._ploc_act_manual_align = adv_menu.addAction(
            "Manual align (slide mosaic by eye)")
        self._ploc_act_manual_align.setCheckable(True)
        adv_menu.aboutToShow.connect(self._ploc_sync_advanced_menu)
        self._ploc_btn_advanced.setMenu(adv_menu)
        mosaic_lay.addWidget(self._ploc_btn_advanced)
        ctrl_lay.addWidget(mosaic_box)

        # v7.5.x manual global registration — when auto registration can't lock
        # on (featureless tiles between/within wells), slide the whole stitched
        # mosaic onto the well grid BY EYE with these sliders (one global shift,
        # never per-tile — the stage is accurate). "Store alignment" persists the
        # delta for this camera + objective so future mosaics start aligned, and
        # bakes it into the saved overlay. Works for both the full mosaic and the
        # Calibrate… pop-out (which pushes its composite to this same overlay).
        self._ploc_align_group = QGroupBox("Manual align (slide mosaic by eye)")
        self._ploc_align_group.setCheckable(True)
        self._ploc_align_group.setChecked(False)
        self._ploc_align_group.toggled.connect(
            self._ploc_on_align_mode_toggled)
        ag = QGridLayout(self._ploc_align_group)
        ag.setContentsMargins(s(8), s(4), s(8), s(4))
        ag.addWidget(QLabel("Δx µm"), 0, 0)
        self._ploc_align_dx = QSlider(Qt.Horizontal)
        self._ploc_align_dx.setRange(-10000, 10000)   # ±10 mm headroom
        self._ploc_align_dx.setSingleStep(5)
        self._ploc_align_dx.setPageStep(200)
        self._ploc_align_dx.setValue(0)
        self._ploc_align_dx.valueChanged.connect(self._ploc_on_align_changed)
        ag.addWidget(self._ploc_align_dx, 0, 1)
        self._ploc_align_dx_lbl = QLabel("0")
        self._ploc_align_dx_lbl.setMinimumWidth(s(44))
        ag.addWidget(self._ploc_align_dx_lbl, 0, 2)
        ag.addWidget(QLabel("Δy µm"), 1, 0)
        self._ploc_align_dy = QSlider(Qt.Horizontal)
        self._ploc_align_dy.setRange(-10000, 10000)
        self._ploc_align_dy.setSingleStep(5)
        self._ploc_align_dy.setPageStep(200)
        self._ploc_align_dy.setValue(0)
        self._ploc_align_dy.valueChanged.connect(self._ploc_on_align_changed)
        ag.addWidget(self._ploc_align_dy, 1, 1)
        self._ploc_align_dy_lbl = QLabel("0")
        ag.addWidget(self._ploc_align_dy_lbl, 1, 2)
        ag.addWidget(QLabel("Opacity"), 2, 0)
        self._ploc_align_op = QSlider(Qt.Horizontal)
        self._ploc_align_op.setRange(5, 100)
        self._ploc_align_op.setValue(55)
        self._ploc_align_op.valueChanged.connect(self._ploc_on_align_changed)
        ag.addWidget(self._ploc_align_op, 2, 1)
        self._ploc_align_op_lbl = QLabel("55%")
        ag.addWidget(self._ploc_align_op_lbl, 2, 2)
        align_btns = QHBoxLayout()
        self._ploc_align_store_btn = QPushButton("Store alignment")
        self._ploc_align_store_btn.setToolTip(
            "Save this shift as the camera + objective's alignment delta (added "
            "to any prior stored shift) and bake it into the saved overlay.")
        self._ploc_align_store_btn.clicked.connect(
            self._ploc_store_manual_align)
        align_btns.addWidget(self._ploc_align_store_btn)
        self._ploc_align_reset_btn = QPushButton("Reset")
        self._ploc_align_reset_btn.clicked.connect(
            self._ploc_reset_manual_align)
        align_btns.addWidget(self._ploc_align_reset_btn)
        self._ploc_align_clear_btn = QPushButton("Clear stored")
        self._ploc_align_clear_btn.setToolTip(
            "Forget this camera + objective's stored alignment so auto "
            "registration sets it on the next scan (releases a manual lock).")
        self._ploc_align_clear_btn.clicked.connect(
            self._ploc_clear_stored_align)
        align_btns.addWidget(self._ploc_align_clear_btn)
        ag.addLayout(align_btns, 3, 0, 1, 3)
        # v7.5.x: hidden by default — surfaced via Advanced… → Manual align.
        self._ploc_align_group.setVisible(False)
        self._ploc_act_manual_align.toggled.connect(
            self._ploc_align_group.setVisible)
        ctrl_lay.addWidget(self._ploc_align_group)

        # v7.4.6: confirm-mode controls — shown only while Run is paused
        # at a freeform point waiting for the user to manually center.
        self._ploc_confirm_label = QLabel("")
        self._ploc_confirm_label.setWordWrap(True)
        self._ploc_confirm_label.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 9pt;")
        self._ploc_confirm_label.setVisible(False)
        ctrl_lay.addWidget(self._ploc_confirm_label)

        confirm_row = QHBoxLayout()
        self._ploc_btn_confirm = QPushButton("Confirm position")
        self._ploc_btn_confirm.setObjectName("successBtn")
        self._ploc_btn_confirm.clicked.connect(self._ploc_confirm)
        confirm_row.addWidget(self._ploc_btn_confirm)
        self._ploc_btn_skip = QPushButton("Skip")
        self._ploc_btn_skip.clicked.connect(self._ploc_skip)
        confirm_row.addWidget(self._ploc_btn_skip)
        self._ploc_btn_cancel = QPushButton("Cancel run")
        self._ploc_btn_cancel.clicked.connect(self._ploc_cancel_run)
        confirm_row.addWidget(self._ploc_btn_cancel)
        self._ploc_confirm_row = confirm_row
        for _b in (self._ploc_btn_confirm, self._ploc_btn_skip,
                   self._ploc_btn_cancel):
            _b.setVisible(False)
        ctrl_lay.addLayout(confirm_row)

        self._ploc_status = QLabel("Queue: 0 wells, 0 freeform")
        self._ploc_status.setStyleSheet(f"color: {COLORS['subtext0']};")
        ctrl_lay.addWidget(self._ploc_status)

        # Keep the control panel from hogging width — the views are the point.
        ctrl_panel.setMaximumWidth(s(360))
        splitter.addWidget(ctrl_panel)
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 1)
        outer.addWidget(splitter, stretch=1)

        # Track queue + fits.
        #
        # v7.4.6: the queue is now heterogeneous. Each entry is either:
        #   • a ``str`` well name (Snap mode)                       — or —
        #   • a dict ``{"kind": "free", "x", "y", "status"}`` for a
        #     freeform point. ``x``/``y`` are zero-ref µm (the same
        #     frame the canvas emits). ``status`` ∈ queued/taught/skipped.
        self._ploc_queue: list = []
        self._ploc_fits: dict[str, tuple[float, float]] = {}
        # v7.5.x: which version of the tab is showing — "mosaic" (default)
        # or the deprecated "legacy" well-fit/queue flow. Set for real by
        # _ploc_set_workflow_mode at the end of this builder.
        self._ploc_workflow_mode: str = "mosaic"
        # Freeform manual-teach pairs accumulated during a Run, as
        # ((src_x_stage, src_y_stage), (meas_x_stage, meas_y_stage)) in
        # absolute stage µm. Consumed by ``_manual_fit_xy`` then cleared.
        self._free_teach_pairs: list = []
        # Run state machine.
        self._ploc_running: bool = False
        self._ploc_steps: list = []
        self._ploc_run_idx: int = 0
        self._ploc_well_results: dict[str, tuple[float, float]] = {}
        # Pause state — when a step needs user interaction. ``kind`` is
        # "freeform" (one Confirm) or "well_3pt" (collect 3 rim points).
        self._ploc_pause_kind: str | None = None
        self._ploc_manual_well: str | None = None
        self._ploc_manual_points: list = []
        # Cached per-run well-fit resources (set up in _ploc_run_queue).
        self._ploc_plate_cam = None
        self._ploc_um_per_px: float = 0.0
        self._ploc_expected_radius_px: float = 0.0
        self._ploc_locator = None
        # v7.5.x manual click-rim well-fit state.
        #   _ploc_fit_mode    : "manual" (click edges) | "auto" (edge-detect)
        #   _ploc_live_cam_idx: microscope slot the live view shows
        #   _ploc_rim_targets : predicted rim stage-µm positions to visit
        #   _ploc_rim_idx     : index of the rim point currently being clicked
        #   _ploc_click_points: clicked rim points in absolute stage µm
        self._ploc_fit_mode: str = "manual"
        self._ploc_live_cam_idx: int = 0
        self._ploc_rim_targets: list = []
        self._ploc_rim_idx: int = 0
        self._ploc_click_points: list = []
        # v7.5.x manual single-point re-anchor state (NO-TRAVEL flow):
        #   _ploc_reanchor_stage    : None | "await_live" | "await_overview"
        #   _ploc_reanchor_live_abs : the feature's ACTUAL absolute-µm position,
        #                             captured from the live-view click (step 1).
        # The operator jogs a recognizable feature into the live view, clicks it
        # there (→ live_abs), then clicks that same feature on the plate overview
        # (→ map_abs). The correction E = live_abs − map_abs shifts the whole map
        # (well centres + mosaic overlay) by one translation. The stage never
        # moves during re-anchor.
        self._ploc_reanchor_stage: str | None = None
        self._ploc_reanchor_live_abs: tuple[float, float] | None = None
        # v7.5.x full-plate mosaic scan state. The scan runs on a worker
        # QThread (_MosaicScanWorker) so the camera grab timer + UI stay live.
        self._ploc_mosaic_running: bool = False
        # v7.5.x: single-well scan — when set, the mosaic scan uses these
        # bounds (one well) and routes its finish to the single-well mapping
        # (rosette wells map their sub-wells; plain wells pick the centre).
        self._ploc_scan_bounds_override = None
        self._ploc_scan_subwell_parent = None
        self._ploc_scan_well_is_rosette = False
        # v7.5.x: measured single-well anchors for "Re-register from scanned
        # wells": well → {"measured": (x,y) abs µm, "map": (x,y) abs µm at pick
        # time}. SESSION-ONLY (valid only against the current map frame);
        # cleared after every re-register and on any plate/type switch.
        self._ploc_well_anchors: dict = {}
        # v7.5.x: hands-free auto re-anchor worker (travel + template match).
        self._ploc_auto_reanchor_worker = None
        self._ploc_mosaic_builder = None
        self._ploc_mosaic_positions: list = []
        self._ploc_mosaic_index: int = 0
        self._ploc_mosaic_total: int = 0
        self._ploc_mosaic_worker = None
        # Tunable mosaic-scan parameters (camera timing / raster / detection),
        # editable via the "Settings…" dialog and persisted in settings.json.
        try:
            from gui.dialogs.mosaic_settings_dialog import merged_settings
            stored = (self.settings.get_section("mosaic_scan")
                      if self.settings is not None else None)
            self._mosaic_settings = merged_settings(stored)
        except Exception as e:
            logger.debug(f"Mosaic settings load skipped: {e}")
            from gui.dialogs.mosaic_settings_dialog import MOSAIC_SCAN_DEFAULTS
            self._mosaic_settings = dict(MOSAIC_SCAN_DEFAULTS)
        self._ploc_mosaic_cam = None
        self._ploc_mosaic_um_per_px: float = 0.0
        self._ploc_mosaic_show: bool = False
        # Cached source overlay (numpy BGR + absolute-µm extent) for manual-align
        # re-bake; set by _ploc_set_overlay_image, cleared when the overlay clears.
        self._ploc_overlay_img = None
        self._ploc_overlay_ext = None
        # Show any previously-stitched mosaic for this plate (best-effort).
        try:
            self._ploc_load_persisted_mosaic()
        except Exception as e:
            logger.debug(f"PlateLocation: mosaic preload skipped: {e}")
        self._ploc_refresh_reregister_button()
        self._ploc_refresh_auto_reanchor_button()
        # v7.5.x: restore the persisted workflow version (default = mosaic).
        wf_mode = "mosaic"
        try:
            prefs = (self.settings.get_section("plate_location_prefs")
                     if self.settings is not None else None)
            if isinstance(prefs, dict) and prefs.get("workflow") == "legacy":
                wf_mode = "legacy"
        except Exception as e:
            logger.debug(f"PlateLocation: workflow pref load skipped: {e}")
        try:
            self._ploc_workflow_combo.blockSignals(True)
            _idx = self._ploc_workflow_combo.findData(wf_mode)
            if _idx >= 0:
                self._ploc_workflow_combo.setCurrentIndex(_idx)
        finally:
            self._ploc_workflow_combo.blockSignals(False)
        self._ploc_set_workflow_mode(wf_mode, persist=False)
        return page

    # ── Adaptive well auto-cal tuning (v7.4.6) ───────────────────────
    # Minimum detection confidence to accept a single-frame fit.
    _PLOC_MIN_CONF = 0.20
    # Rim points sampled per well in the multi-edge strategy.
    _PLOC_RIM_SAMPLES = 8
    # Stage settle time (s) between rim-sample moves.
    _PLOC_SETTLE_S = 0.15
    # Reject a multi-edge / 3-point fit whose radius deviates this much
    # from the plate's known well radius (fractional).
    _PLOC_RADIUS_TOL = 0.50
    # v7.5.x: full-plate mosaic raster overlap (fraction of the camera FOV).
    # Grid spacing = FOV · (1 − overlap). 0.25 = 25% overlap.
    _PLOC_MOSAIC_OVERLAP = 0.25

    # ── Plate-calibration workflow versions (v7.5.x) ─────────────────
    # The Mosaic tools are the default workflow; the per-well well-fit /
    # target-queue flow is kept as a deprecated alternate version until it
    # is removed. Banner text per version:
    _PLOC_BANNER_MOSAIC = (
        "Mosaic workflow: Calibrate mosaic scan (once per camera + "
        "objective) → Mosaic scan (Map wells opens automatically when the "
        "scan finishes) → Re-anchor / Auto re-anchor after a plate remount. "
        "Rosette sub-wells calibrate on the Rosettes tab; extra tools live "
        "under Advanced…. This builds the XY map only — Z calibration "
        "lives in the Needle Offset tab."
    )
    _PLOC_BANNER_LEGACY = (
        "[Deprecated] The per-well well-fit queue is superseded by the "
        "Mosaic workflow — switch back above unless you specifically need "
        "this flow.\n"
        "Use the Free/Snap toggle (top-left of the plate view) to "
        "choose how clicks enqueue targets. Snap = click wells, Free = "
        "click anywhere to drop a freeform point. Mix both, then Run.\n"
        "Well-fit mode (below): Manual = the stage moves to each well, "
        "then you JOG to bring each rim edge into the live microscope "
        "view and click the well EDGE (each click uses the current "
        "stage position; ≥3 clicks around the rim → circle fit → "
        "center). Auto = camera edge-detection. This builds the XY map "
        "only — Z calibration lives in the Needle Offset tab."
    )

    def _ploc_on_workflow_changed(self) -> None:
        """The 'Plate calibration' version combo changed — apply it, unless
        a queue run or mosaic scan is active (never yank the UI out from
        under a run; revert the combo instead)."""
        combo = getattr(self, "_ploc_workflow_combo", None)
        mode = (combo.currentData() if combo is not None else None) or "mosaic"
        if (getattr(self, "_ploc_running", False)
                or getattr(self, "_ploc_mosaic_running", False)):
            QMessageBox.information(
                self, "Plate calibration",
                "Finish or cancel the current run/scan before switching "
                "the plate-calibration workflow.")
            cur = getattr(self, "_ploc_workflow_mode", "mosaic")
            if combo is not None:
                try:
                    combo.blockSignals(True)
                    _idx = combo.findData(cur)
                    if _idx >= 0:
                        combo.setCurrentIndex(_idx)
                finally:
                    combo.blockSignals(False)
            return
        self._ploc_set_workflow_mode(mode)

    def _ploc_set_workflow_mode(self, mode: str, persist: bool = True) -> None:
        """Show ONE version of the Plate Location tab: "mosaic" (default)
        or the deprecated "legacy" well-fit/target-queue flow.

        Widgets are only hidden, never torn down — every attribute stays
        alive and every handler stays wired, so the legacy flow keeps
        working (and its tests keep passing) until it is actually removed.
        ``persist=True`` writes the choice to settings.json
        ("plate_location_prefs" → "workflow").
        """
        mode = "legacy" if mode == "legacy" else "mosaic"
        self._ploc_workflow_mode = mode
        legacy = mode == "legacy"
        for name, visible in (
                ("_ploc_mosaic_box", not legacy),
                ("_ploc_mosaic_prev_col", not legacy),
                ("_ploc_mode_row_w", legacy),
                ("_ploc_queue_group", legacy),
                ("_ploc_status", legacy),
        ):
            w = getattr(self, name, None)
            if w is not None:
                try:
                    w.setVisible(visible)
                except Exception:
                    pass
        if legacy:
            # The Advanced menu (the align group's only entry point) hides
            # with the mosaic box — cleanly exit align mode + hide the
            # sliders so they can't linger orphaned.
            grp = getattr(self, "_ploc_align_group", None)
            if grp is not None:
                try:
                    grp.setChecked(False)   # clean exit via its handler
                    grp.setVisible(False)
                except Exception:
                    pass
            act = getattr(self, "_ploc_act_manual_align", None)
            if act is not None:
                try:
                    act.blockSignals(True)
                    act.setChecked(False)
                finally:
                    act.blockSignals(False)
        banner = getattr(self, "_ploc_banner", None)
        if banner is not None:
            try:
                banner.setText(self._PLOC_BANNER_LEGACY if legacy
                               else self._PLOC_BANNER_MOSAIC)
            except Exception:
                pass
        if persist and getattr(self, "settings", None) is not None:
            try:
                prefs = self.settings.get_section("plate_location_prefs")
                prefs = dict(prefs) if isinstance(prefs, dict) else {}
                prefs["workflow"] = mode
                self.settings.set_section("plate_location_prefs", prefs)
                self.settings.save()
            except Exception as e:
                logger.warning(
                    f"Plate-calibration workflow pref save failed: {e}")

    def _ploc_on_well_clicked(self, well_name: str) -> None:
        if not well_name:
            return
        # Manual re-anchor: in step 2 (await_overview) a well click picks the
        # well CENTRE as the map point (more precise than a freeform click). Snap
        # mode also emits position_clicked; whichever fires first applies + ends
        # re-anchor, the other is then a no-op. While re-anchor is active in any
        # OTHER stage (step 1 = await_live), swallow overview clicks so they
        # don't pollute the run queue.
        if getattr(self, "_ploc_reanchor_stage", None) is not None:
            if self._ploc_reanchor_stage == "await_overview":
                try:
                    gx, gy = self._predict_well_xy(well_name)
                except Exception:
                    return
                self._ploc_reanchor_overview(gx, gy, well_name)
            return
        if self._ploc_running:
            return
        # Mosaic workflow: the target queue is hidden — don't mutate
        # invisible state. (Re-anchor overview clicks are handled above.)
        if getattr(self, "_ploc_workflow_mode", "mosaic") != "legacy":
            return
        # Toggle: clicking an already-queued well removes it.
        if well_name in self._ploc_queue:
            self._ploc_queue.remove(well_name)
        else:
            self._ploc_queue.append(well_name)
        self._ploc_refresh_queue_view()

    # ── Freeform (arbitrary XY) targets — v7.4.6 ─────────────────────

    # Click within this radius (zero-ref µm) of an existing freeform
    # point removes it instead of adding a new one (mirrors well toggle).
    _PLOC_FREE_REMOVE_RADIUS_UM = 500.0

    def _ploc_on_position_clicked(self, x_zr: float, y_zr: float) -> None:
        """Handle a freeform click on the plate view.

        Snap mode emits ``position_clicked`` too (next to
        ``well_clicked``); we only act in Free mode so wells are not
        double-enqueued.
        """
        # Manual re-anchor: in step 2 (await_overview) accept a Free-selection
        # overview click as the map point (zero-ref → absolute µm). While
        # re-anchor is active in any other stage (step 1 = await_live), swallow
        # overview clicks so they don't pollute the run queue.
        if getattr(self, "_ploc_reanchor_stage", None) is not None:
            if self._ploc_reanchor_stage == "await_overview":
                zero = self.controller.zero_position if self.controller else {}
                gx = float(x_zr) + float(zero.get("x", 0.0))
                gy = float(y_zr) + float(zero.get("y", 0.0))
                self._ploc_reanchor_overview(gx, gy, "clicked point")
            return
        if self._ploc_running:
            return
        # Mosaic workflow: the target queue is hidden — don't mutate
        # invisible state. (Re-anchor overview clicks are handled above.)
        if getattr(self, "_ploc_workflow_mode", "mosaic") != "legacy":
            return
        try:
            mode = self._ploc_plate_view.target_mode()
        except Exception:
            mode = "snap"
        if mode != "free":
            return
        # Toggle-remove if the click lands near an existing freeform pt.
        near = self._ploc_find_free_near(x_zr, y_zr)
        if near is not None:
            self._ploc_queue.remove(near)
        else:
            self._ploc_queue.append({
                "kind": "free",
                "x": float(x_zr),
                "y": float(y_zr),
                "status": "queued",
            })
        self._ploc_refresh_queue_view()

    def _ploc_find_free_near(self, x_zr: float, y_zr: float):
        """Return the nearest queued freeform entry within the remove
        radius of (x_zr, y_zr) zero-ref µm, or None."""
        best = None
        best_d2 = self._PLOC_FREE_REMOVE_RADIUS_UM ** 2
        for entry in self._ploc_queue:
            if not isinstance(entry, dict) or entry.get("kind") != "free":
                continue
            dx = entry["x"] - x_zr
            dy = entry["y"] - y_zr
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best_d2 = d2
                best = entry
        return best

    def _ploc_clear_queue(self) -> None:
        if self._ploc_running:
            return
        self._ploc_queue = []
        self._ploc_fits = {}
        self._free_teach_pairs = []
        self._ploc_refresh_queue_view()

    def _ploc_refresh_queue_view(self) -> None:
        self._ploc_queue_list.clear()
        n_wells = 0
        n_free = 0
        for entry in self._ploc_queue:
            if isinstance(entry, str):
                n_wells += 1
                status = "fitted" if entry in self._ploc_fits else "queued"
                label = f"{entry}  —  {status}"
            else:
                n_free += 1
                status = entry.get("status", "queued")
                label = (f"⌖ ({entry['x']:+.0f}, {entry['y']:+.0f}) µm"
                         f"  —  {status}")
                # Show the measured delta (how far off it was) once taught.
                if status == "taught" and "dx" in entry:
                    label += (f"   Δ({entry['dx']:+.0f}, {entry['dy']:+.0f}) µm"
                              f"  |{(entry['dx'] ** 2 + entry['dy'] ** 2) ** 0.5:.0f}|")
            self._ploc_queue_list.addItem(QListWidgetItem(label))
        self._ploc_status.setText(
            f"Queue: {n_wells} wells, {n_free} freeform")
        # Enable Run with any target queued; the fit step reports if
        # there are too few points to solve the affine.
        self._ploc_btn_run.setEnabled(
            (n_wells + n_free) >= 1 and not self._ploc_running)

    def _ploc_run_queue(self) -> None:
        """Start the queue run.

        Wells are auto edge-fit (needs the Plate camera). Freeform
        points pause the run so the user can manually center on the
        target before confirming (v7.4.6). Camera resources are only
        required when the queue contains at least one well.
        """
        if self._ploc_running:
            return
        # Don't start a queue run while a mosaic scan owns the stage (the
        # objective-confirm dialog is modal, so a Run click could otherwise
        # slip in before the mosaic worker launches → conflicting XY moves).
        if getattr(self, "_ploc_mosaic_running", False):
            return
        if self.controller is None:
            QMessageBox.warning(
                self, "Plate Location", "Stage controller required.")
            return
        if self._plate is None:
            QMessageBox.warning(
                self, "Plate Location", "No plate format selected.")
            return
        # v7.5.x: Z heights must be set up FIRST — the run hops between wells
        # and must retract the needle to the Safe/Move Z before each XY move
        # (and the operator jogs Z down to focus on each well's edge). Without a
        # known Safe Z there is nothing safe to retract to. Enforce ordering
        # whenever a needle/ZP is connected (nothing to retract otherwise).
        zp_connected = bool(getattr(self.controller, 'is_zp_connected', False))
        if zp_connected and getattr(self, '_safe_z', None) is None:
            QMessageBox.warning(
                self, "Plate Location",
                "Set the Z heights first.\n\nOn the Needle Offset Calibration "
                "tab, capture at least the Safe / Move Z height. The plate "
                "calibration retracts the needle to that height before moving "
                "between wells, so it must be set before you run this step.")
            return

        n_wells = sum(1 for e in self._ploc_queue if isinstance(e, str))
        # Only validate / cache the camera path when wells are queued.
        if n_wells > 0:
            if not NEEDLE_LOCATION_AVAILABLE:
                QMessageBox.warning(
                    self, "Plate Location",
                    "Vision helpers not available — install opencv-python.")
                return
            if self._camera_manager is None:
                QMessageBox.warning(
                    self, "Plate Location",
                    "Camera manager required to edge-fit wells.")
                return
            hw = getattr(self, '_hardware_config', None)
            plate_cam_idx = (
                hw.camera_for_role(CameraRole.MICROSCOPE) if hw is not None else None
            )
            if plate_cam_idx is None:
                QMessageBox.warning(
                    self, "Plate Location",
                    "Assign a camera the Microscope role in "
                    "Hardware Setup → Cameras.")
                return
            try:
                plate_cam = self._camera_manager.cameras[plate_cam_idx]
            except (AttributeError, IndexError):
                QMessageBox.warning(
                    self, "Plate Location",
                    f"Camera index {plate_cam_idx} not available.")
                return
            # v7.5.x: read the calibrated µm/px from the shared CameraManager
            # (not the CameraWidget, which never carries `_um_per_px`), gated
            # on the explicit calibration flag so the uncalibrated case still
            # routes to the "calibrate first" warning below.
            if self._camera_manager.is_um_per_px_calibrated(plate_cam_idx):
                um_per_px = float(
                    self._camera_manager.get_um_per_px(plate_cam_idx) or 0.0)
            else:
                um_per_px = 0.0
            if um_per_px <= 0:
                QMessageBox.warning(
                    self, "Plate Location",
                    "Microscope camera µm/pixel not calibrated. "
                    "Calibrate in Hardware Setup → Cameras.")
                return
            self._ploc_plate_cam = plate_cam
            self._ploc_um_per_px = um_per_px
            well_radius_um = (self._plate.well_diameter * 1000.0) / 2.0
            self._ploc_expected_radius_px = well_radius_um / um_per_px
            self._ploc_locator = EdgeFitWellLocator()
            # v7.5.x: the live view shows this microscope slot; require the
            # operator to confirm the in-use objective before driving.
            self._ploc_live_cam_idx = plate_cam_idx
            if not self._ploc_confirm_objective(plate_cam_idx, um_per_px):
                return
            if self._ploc_fit_mode == "manual":
                self._ploc_ensure_live_camera()

        # Initialise the run state machine and kick it off.
        self._ploc_running = True
        self._ploc_steps = list(self._ploc_queue)
        self._ploc_run_idx = 0
        self._ploc_well_results = {}
        self._free_teach_pairs = []
        self._ploc_pause_kind = None
        self._ploc_manual_well = None
        self._ploc_manual_points = []
        for e in self._ploc_queue:
            if isinstance(e, dict):
                e["status"] = "queued"
        self._ploc_btn_run.setEnabled(False)
        self._ploc_btn_clear.setEnabled(False)

        # v7.5.x CRITICAL SAFETY: retract the needle to a safe height ONCE
        # before the plate-location scan so the cross-plate hops to each well
        # center / rim point (driven below via move_xy_absolute_um) never drag a
        # lowered needle across the plate. ensure_retracted_to never descends,
        # so this is harmless if the needle is already parked up.
        try:
            if hasattr(self.controller, "ensure_retracted_to"):
                safe_z = (self._safe_z
                          if getattr(self, "_safe_z", None) is not None else 0.0)
                self.controller.ensure_retracted_to(safe_z)
        except Exception as e:
            logger.warning(f"PlateLocation: pre-scan retract failed: {e}")

        self._ploc_advance()

    # ── Well auto-calibration (v7.4.6 adaptive circle fitting) ───────

    def _ploc_capture(self):
        """Grab a fresh frame from the cached plate camera, or None."""
        cam = self._ploc_plate_cam
        if cam is None:
            return None
        try:
            return cam.capture_fresh_frame()
        except Exception:
            try:
                return cam.get_current_frame()
            except Exception:
                return None

    def _ploc_safe_goto(self, x_um: float, y_um: float) -> None:
        """Inter-well hop: retract the needle to the Safe/Move Z and wait,
        then travel XY (staying retracted — no descent). Use for every move to
        a NEW well so a needle the operator jogged down to focus is lifted
        clear of the plate first. Intra-well rim-sample moves stay bare.

        The run gate guarantees ``_safe_z`` is set; the bare-move fallback only
        guards a missing reference."""
        if getattr(self, '_safe_z', None) is not None:
            try:
                self._safe_navigate_to(x_um, y_um, lower_z=False)
                return
            except Exception as e:
                logger.warning(
                    f"PlateLocation: safe goto failed ({e}); bare XY move")
        self.controller.move_xy_absolute_um(x_um, y_um)

    def _ploc_auto_fit_well(self, well_name: str):
        """Auto-calibrate one well's center, picking the circle-fit
        strategy from well size vs. camera FOV.

        Returns the measured center in absolute stage µm, or None when
        the arc can't be found automatically (→ manual 3-point fallback).
        """
        wells_by_name = {w.name: w for w in self._plate.get_all_wells()}
        if well_name not in wells_by_name:
            return None
        try:
            cx_pred, cy_pred = self._predict_well_xy(well_name)
        except Exception as e:
            logger.warning(f"PlateLocation: predict for {well_name} failed: {e}")
            return None
        # Centered observation frame + FOV. Retract Z first (inter-well hop).
        try:
            self._ploc_safe_goto(cx_pred, cy_pred)
        except Exception as e:
            logger.warning(f"PlateLocation: move to {well_name} failed: {e}")
            return None
        frame = self._ploc_capture()
        if frame is None:
            return None
        um = self._ploc_um_per_px
        fh, fw = frame.shape[0], frame.shape[1]
        d_um = float(self._plate.well_diameter) * 1000.0
        strategy = select_well_fit_strategy(d_um, fw * um, fh * um)
        logger.info(
            f"PlateLocation: {well_name} strategy={strategy} "
            f"(well Ø{d_um:.0f}µm vs FOV {fw*um:.0f}×{fh*um:.0f}µm)")
        if strategy in ("full_circle", "partial_arc"):
            return self._ploc_single_frame_fit(
                frame, strategy, cx_pred, cy_pred)
        # multi_edge: sample several rim points and fit a circle.
        return self._ploc_multi_edge_fit(cx_pred, cy_pred, d_um / 2.0)

    def _ploc_single_frame_fit(self, frame, strategy, cx_pred, cy_pred):
        """Detect the well center in a single centered frame.

        ``full_circle`` → Hough/contour, falling back to a known-radius
        arc fit; ``partial_arc`` → known-radius arc fit, falling back to
        Hough/contour. Returns stage-µm center or None.
        """
        um = self._ploc_um_per_px
        r_px = self._ploc_expected_radius_px

        def _hough():
            try:
                from SupportClasses.VisionDetector import WellDetector
                return WellDetector.detect_well_with_fallback(
                    frame, expected_diameter_px=2.0 * r_px, tolerance=0.3)
            except Exception as e:
                logger.warning(f"PlateLocation: full-circle detect failed: {e}")
                return None

        def _arc():
            return self._ploc_locator.fit_partial_arc(
                frame, expected_radius_px=r_px, um_per_px=um)

        result = _hough() if strategy == "full_circle" else _arc()
        if result is None or result.confidence < self._PLOC_MIN_CONF:
            alt = _arc() if strategy == "full_circle" else _hough()
            if alt is not None and (
                result is None or alt.confidence > result.confidence
            ):
                result = alt
        if result is None or result.confidence < self._PLOC_MIN_CONF:
            return None
        fw = frame.shape[1]
        fh = frame.shape[0]
        off_x = (result.center_px[0] - fw / 2.0) * um
        off_y = (result.center_px[1] - fh / 2.0) * um
        return (cx_pred + off_x, cy_pred + off_y)

    def _ploc_multi_edge_fit(self, cx_pred, cy_pred, r_um):
        """Drive to ``_PLOC_RIM_SAMPLES`` predicted rim points, detect the
        local edge at each, and fit a circle through the collected rim
        points. Returns stage-µm center or None (→ manual fallback)."""
        import math
        import time
        um = self._ploc_um_per_px
        n = max(3, int(self._PLOC_RIM_SAMPLES))
        pts: list[tuple[float, float]] = []
        for i in range(n):
            th = 2.0 * math.pi * i / n
            ax = cx_pred + r_um * math.cos(th)
            ay = cy_pred + r_um * math.sin(th)
            try:
                self.controller.move_xy_absolute_um(ax, ay)
            except Exception as e:
                logger.warning(f"PlateLocation: rim move failed: {e}")
                continue
            time.sleep(self._PLOC_SETTLE_S)
            frame = self._ploc_capture()
            if frame is None:
                continue
            rp = detect_rim_point_near_center(frame)
            if rp is None:
                continue
            px, py, _conf = rp
            fw = frame.shape[1]
            fh = frame.shape[0]
            off_x = (px - fw / 2.0) * um
            off_y = (py - fh / 2.0) * um
            pts.append((ax + off_x, ay + off_y))
        if len(pts) < 3:
            logger.info(
                f"PlateLocation: multi-edge found only {len(pts)} rim "
                "points — falling back to manual.")
            return None
        fit = fit_circle_to_points(pts)
        if fit is None:
            return None
        cx, cy, r = fit
        if r_um > 0 and abs(r - r_um) / r_um > self._PLOC_RADIUS_TOL:
            logger.warning(
                f"PlateLocation: multi-edge radius {r:.0f}µm vs expected "
                f"{r_um:.0f}µm — rejecting fit.")
            return None
        return (cx, cy)

    # ── v7.5.x: manual click-rim well fit + objective confirmation ───

    def _ploc_on_mode_changed(self, _idx: int = 0) -> None:
        """Well-fit mode combo changed (ignored mid-run)."""
        if self._ploc_running:
            return
        self._ploc_fit_mode = self._ploc_mode_combo.currentData() or "manual"

    def _ploc_ensure_live_camera(self) -> None:
        """Point the live view at the microscope slot and start it.

        ``set_camera`` is only re-issued when the bound slot actually
        changes (it disconnects/reconnects the frame signal), so calling
        this once per well doesn't flicker the feed. ``start`` is a no-op
        when the camera is already running."""
        if self._camera_manager is None:
            return
        cam_idx = self._ploc_live_cam_idx
        try:
            if self._ploc_live_view.cam_idx != cam_idx:
                self._ploc_live_view.set_camera(cam_idx)
            self._camera_manager.start(cam_idx)
        except Exception as e:
            logger.debug(f"PlateLocation: live camera start failed: {e}")

    def _ploc_confirm_objective(self, cam_idx: int, um_per_px: float) -> bool:
        """Make the operator confirm the objective in use matches the one
        selected in the config (whose µm/px maps clicks → stage µm).

        Returns True to proceed, False to abort the run. Blocks when no
        objective is selected (we can't know what's on the scope).
        """
        hw = getattr(self, '_hardware_config', None)
        cam_cfg = getattr(hw, 'camera_config', None) if hw else None
        obj_name = (getattr(cam_cfg, 'current_objective_name', None)
                    if cam_cfg else None)
        if not obj_name:
            QMessageBox.warning(
                self, "Confirm objective",
                "No microscope objective is selected. In Hardware Setup → "
                "Cameras, set the objective currently installed on the "
                "scope (its µm/px calibration is what maps your clicks to "
                "stage positions), then run again.")
            return False
        # Best-effort note on whether a per-objective calibration is stored.
        cal_note = ""
        try:
            from SupportClasses.ObjectiveCalibration import get_store
            model = self._get_camera_model_for_idx(cam_idx)
            cal = get_store().get_calibration(model, obj_name)
            if cal:
                cal_note = (
                    f"\nStored calibration: "
                    f"{float(cal['measured_um_per_px']):.4f} µm/px"
                    f" ({cal.get('date', '')}).")
            else:
                cal_note = ("\n⚠ No stored per-objective calibration — using "
                            "the camera's current µm/px.")
        except Exception:
            pass
        resp = QMessageBox.question(
            self, "Confirm objective",
            f"Calibrating with objective: {obj_name}\n"
            f"µm/pixel in use: {um_per_px:.4f}{cal_note}\n\n"
            f"Is the microscope physically set to the {obj_name} objective "
            f"right now?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return resp == QMessageBox.Yes

    def _ploc_begin_well_click_rim(self, well_name: str) -> None:
        """Manual well fit (jog-driven): drive ONCE to the predicted well
        centre as a starting anchor, then let the operator jog to bring each
        rim edge into view and click it. Each click records the rim point at
        the stage's CURRENT position, so manual jog adjustments are honoured
        — the stage is not driven again until the next well. Collected clicks
        → circle fit → centre (on Confirm)."""
        self._ploc_pause_kind = "well_click_rim"
        self._ploc_manual_well = well_name
        self._ploc_click_points = []
        self._ploc_rim_idx = 0
        self._ploc_rim_targets = []
        try:
            cx, cy = self._predict_well_xy(well_name)
        except Exception as e:
            logger.warning(
                f"PlateLocation: click-rim setup for {well_name} failed: {e}")
            self._ploc_run_idx += 1
            self._ploc_pause_kind = None
            self._ploc_advance()
            return
        # Anchor near the well; the operator jogs from here to each rim edge.
        # This is the only automatic move — clicks never drive the stage.
        # Inter-well hop → retract the needle to Safe Z first (the operator may
        # have jogged it down to focus on the previous well).
        try:
            self._ploc_safe_goto(cx, cy)
        except Exception as e:
            logger.warning(f"PlateLocation: anchor move failed: {e}")
        self._ploc_ensure_live_camera()
        try:
            self._ploc_live_view.set_overlay_vector(None, None)
        except Exception:
            pass
        self._ploc_update_click_rim_prompt()
        self._ploc_confirm_label.setVisible(True)
        for b in (self._ploc_btn_confirm, self._ploc_btn_skip,
                  self._ploc_btn_cancel):
            b.setVisible(True)

    def _ploc_update_click_rim_prompt(self) -> None:
        """Refresh the click-rim prompt with the running click count."""
        got = len(self._ploc_click_points)
        suggested = max(3, int(self._PLOC_RIM_SAMPLES))
        self._ploc_confirm_label.setText(
            f"{self._ploc_manual_well}: jog to bring each rim edge into view "
            f"and click the well EDGE. Collected {got} (aim for ~{suggested} "
            f"spread around the rim). Confirm to fit (needs ≥3), Skip to "
            f"skip this well."
        )
        self._ploc_live_hint.setText(
            "Each click uses the stage's CURRENT position — jog freely, then "
            "click the rim. Spread clicks around the whole circle.")

    def _ploc_on_live_view_click(self, px_x: float, px_y: float) -> None:
        """Live-view click during a manual click-rim well fit → record the
        clicked rim point in absolute stage µm and advance to the next rim
        position. No-op outside a click-rim pause."""
        # Manual re-anchor: the live click marks where the overview target
        # ACTUALLY is → compute the correction vector and shift the whole map.
        if getattr(self, "_ploc_reanchor_stage", None) == "await_live":
            self._ploc_reanchor_live_click(px_x, px_y)
            return
        if (not self._ploc_running
                or self._ploc_pause_kind != "well_click_rim"):
            return
        if self._camera_manager is None:
            return
        img_w, img_h = self._ploc_live_view.image_size
        if not img_w or not img_h:
            return
        cam_idx = self._ploc_live_cam_idx
        dx_um, dy_um = self._camera_manager.pixel_to_stage_offset(
            cam_idx, px_x, px_y, img_w, img_h)
        xy = self.controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            QMessageBox.warning(
                self, "Plate Location",
                "Could not read current stage position.")
            return
        # get_xy_position is absolute stage µm (xy_position_scale=1), the
        # same frame as _predict_well_xy / move_xy_absolute_um.
        sx = float(xy[0]) + dx_um
        sy = float(xy[1]) + dy_um
        self._ploc_click_points.append((sx, sy))
        # Visual confirmation: arrow from view center to the click. The stage
        # is NOT moved — the operator jogs to the next edge themselves.
        try:
            self._ploc_live_view.set_overlay_vector(
                px_x - img_w / 2.0, px_y - img_h / 2.0, "✓")
        except Exception:
            pass
        self._ploc_update_click_rim_prompt()

    def _ploc_finalize_click_rim(self) -> None:
        """Fit a circle to the clicked rim points and record the well
        center (or warn + leave uncalibrated), then advance the run."""
        well = self._ploc_manual_well
        pts = list(self._ploc_click_points)
        try:
            self._ploc_live_view.set_overlay_vector(None, None)
        except Exception:
            pass
        fit = (fit_circle_to_points(pts)
               if (len(pts) >= 3 and fit_circle_to_points is not None)
               else None)
        r_exp = float(self._plate.well_diameter) * 1000.0 / 2.0
        if fit is not None and r_exp > 0:
            if abs(fit[2] - r_exp) / r_exp > self._PLOC_RADIUS_TOL:
                logger.warning(
                    f"PlateLocation: click-rim radius {fit[2]:.0f}µm vs "
                    f"expected {r_exp:.0f}µm — rejecting {well}.")
                fit = None
        if fit is not None:
            cx, cy, r = fit
            self._ploc_well_results[well] = (cx, cy)
            # v7.5.x: keep a permanent reference marker at the taught centre
            # so it shows on the plate + microscope views immediately and after.
            self._reference_markers[well] = (cx, cy)
            logger.info(
                f"PlateLocation: click-rim fit {well} → "
                f"({cx:.1f}, {cy:.1f}) µm, r={r:.1f} µm "
                f"from {len(pts)} clicks")
        else:
            QMessageBox.warning(
                self, "Plate Location",
                f"Manual fit for {well} needs ≥3 valid rim clicks (got "
                f"{len(pts)}) forming a plausible circle. Well left "
                "uncalibrated.")
        self._ploc_manual_well = None
        self._ploc_click_points = []
        self._ploc_rim_targets = []
        self._ploc_rim_idx = 0
        self._ploc_pause_kind = None
        self._ploc_run_idx += 1
        self._ploc_exit_confirm_mode()
        self._ploc_live_hint.setText("")
        self._ploc_refresh_queue_view()
        # v7.5.x: show this well's reference marker on both views immediately
        # (don't wait for the run to finish).
        try:
            self._refresh_ploc_view()
        except Exception:
            pass
        self._ploc_advance()

    def _ploc_advance(self) -> None:
        """Process queue steps until one needs user interaction (freeform
        centering, or a manual 3-point well fit), or the queue is done."""
        steps = self._ploc_steps
        total = len(steps)
        while self._ploc_run_idx < total:
            step = steps[self._ploc_run_idx]
            if isinstance(step, str):
                # v7.5.x: Manual mode pauses to let the operator click the
                # well edge in the live view at each rim point.
                if self._ploc_fit_mode == "manual":
                    self._ploc_begin_well_click_rim(step)
                    return
                center = self._ploc_auto_fit_well(step)
                if center is not None:
                    self._ploc_well_results[step] = center
                    self._ploc_run_idx += 1
                    continue
                # Arc not found → pause for a manual 3-point fit.
                self._ploc_begin_well_manual(step)
                return
            # Freeform point: drive there, then pause for centering.
            x_zr = step["x"]
            y_zr = step["y"]
            zero = self.controller.zero_position
            sx = x_zr + zero.get("x", 0)
            sy = y_zr + zero.get("y", 0)
            try:
                self._ploc_safe_goto(sx, sy)  # retract Z first (inter-point hop)
            except Exception as e:
                logger.warning(f"PlateLocation: move to freeform failed: {e}")
            self._ploc_pause_kind = "freeform"
            self._ploc_enter_confirm_mode(self._ploc_run_idx, total, x_zr, y_zr)
            return
        self._ploc_finish_run()

    def _ploc_enter_confirm_mode(
        self, idx: int, total: int, x_zr: float, y_zr: float
    ) -> None:
        self._ploc_confirm_label.setText(
            f"Step {idx + 1}/{total}: moved to freeform "
            f"({x_zr:+.0f}, {y_zr:+.0f}) µm. Jog to center the target "
            "in the plate camera, then Confirm (or Skip)."
        )
        self._ploc_confirm_label.setVisible(True)
        for b in (self._ploc_btn_confirm, self._ploc_btn_skip,
                  self._ploc_btn_cancel):
            b.setVisible(True)

    def _ploc_exit_confirm_mode(self) -> None:
        self._ploc_confirm_label.setVisible(False)
        for b in (self._ploc_btn_confirm, self._ploc_btn_skip,
                  self._ploc_btn_cancel):
            b.setVisible(False)

    def _ploc_begin_well_manual(self, well_name: str) -> None:
        """Enter manual 3-point mode for `well_name` after auto-fit
        failed. Nudges the stage to a predicted rim point so the rim is
        in view, then waits for three Confirms."""
        self._ploc_pause_kind = "well_3pt"
        self._ploc_manual_well = well_name
        self._ploc_manual_points = []
        try:
            cx, cy = self._predict_well_xy(well_name)
            r_um = float(self._plate.well_diameter) * 1000.0 / 2.0
            self._ploc_safe_goto(cx + r_um, cy)  # retract Z first (inter-well)
        except Exception as e:
            logger.warning(f"PlateLocation: manual-start move failed: {e}")
        self._ploc_update_manual_prompt(0)
        self._ploc_confirm_label.setVisible(True)
        for b in (self._ploc_btn_confirm, self._ploc_btn_skip,
                  self._ploc_btn_cancel):
            b.setVisible(True)

    def _ploc_update_manual_prompt(self, got: int) -> None:
        self._ploc_confirm_label.setText(
            f"Auto-fit failed for {self._ploc_manual_well}. Jog to a "
            f"point on the well rim and Confirm — collected {got}/3. "
            "Pick 3 well-separated points around the circle (Skip to "
            "leave this well uncalibrated)."
        )

    def _ploc_confirm(self) -> None:
        """Confirm the current manual position. Dispatches on pause kind:
        freeform records one teach pair; well_3pt collects rim points and
        fits a circle once three are gathered; well_click_rim fits the
        edge-clicks collected so far (v7.5.x)."""
        if not self._ploc_running:
            return
        if self._ploc_pause_kind == "well_click_rim":
            # Confirm = fit now with the rim edges already clicked.
            self._ploc_finalize_click_rim()
            return
        xy = self.controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            QMessageBox.warning(
                self, "Plate Location",
                "Could not read current stage position.")
            return
        pos = (float(xy[0]), float(xy[1]))
        kind = self._ploc_pause_kind

        if kind == "freeform":
            step = self._ploc_steps[self._ploc_run_idx]
            zero = self.controller.zero_position
            # Pair the clicked (predicted) location with the confirmed
            # (measured) one, both in absolute stage µm.
            src = (step["x"] + zero.get("x", 0), step["y"] + zero.get("y", 0))
            self._free_teach_pairs.append((src, pos))
            # Per-point delta = how far off the stage was from where software
            # said the point should be (absolute µm → frame-independent).
            step["dx"] = float(pos[0] - src[0])
            step["dy"] = float(pos[1] - src[1])
            step["status"] = "taught"
            self._ploc_run_idx += 1
            self._ploc_pause_kind = None
            self._ploc_exit_confirm_mode()
            self._ploc_refresh_queue_view()
            self._ploc_advance()
            return

        if kind == "well_3pt":
            self._ploc_manual_points.append(pos)
            got = len(self._ploc_manual_points)
            if got < 3:
                self._ploc_update_manual_prompt(got)
                return
            well = self._ploc_manual_well
            fit = fit_circle_to_points(self._ploc_manual_points)
            r_exp = float(self._plate.well_diameter) * 1000.0 / 2.0
            ok = fit is not None
            if ok and r_exp > 0:
                ok = abs(fit[2] - r_exp) / r_exp <= self._PLOC_RADIUS_TOL
            if ok:
                cx, cy, r = fit
                self._ploc_well_results[well] = (cx, cy)
                self._reference_markers[well] = (cx, cy)
                logger.info(
                    f"PlateLocation: 3-point fit {well} → "
                    f"({cx:.1f}, {cy:.1f}) µm, r={r:.1f} µm")
            else:
                QMessageBox.warning(
                    self, "Plate Location",
                    f"3-point fit for {well} was rejected (points "
                    "collinear or radius implausible). Well left "
                    "uncalibrated.")
                logger.warning(
                    f"PlateLocation: 3-point fit rejected for {well} "
                    f"(fit={fit}, expected r={r_exp:.0f}µm).")
            self._ploc_manual_points = []
            self._ploc_manual_well = None
            self._ploc_pause_kind = None
            self._ploc_run_idx += 1
            self._ploc_exit_confirm_mode()
            self._ploc_refresh_queue_view()
            self._ploc_advance()

    def _ploc_skip(self) -> None:
        """Skip the current paused step (freeform point, or a well's
        manual fit) and advance.

        v7.5.x: in well_click_rim mode, Skip leaves this well uncalibrated
        and advances to the next queued target."""
        if not self._ploc_running:
            return
        if self._ploc_pause_kind == "well_click_rim":
            try:
                self._ploc_live_view.set_overlay_vector(None, None)
            except Exception:
                pass
            self._ploc_manual_well = None
            self._ploc_click_points = []
            self._ploc_pause_kind = None
            self._ploc_run_idx += 1
            self._ploc_exit_confirm_mode()
            self._ploc_live_hint.setText("")
            self._ploc_refresh_queue_view()
            self._ploc_advance()
            return
        step = self._ploc_steps[self._ploc_run_idx]
        if self._ploc_pause_kind == "freeform" and isinstance(step, dict):
            step["status"] = "skipped"
        self._ploc_manual_points = []
        self._ploc_manual_well = None
        self._ploc_pause_kind = None
        self._ploc_run_idx += 1
        self._ploc_exit_confirm_mode()
        self._ploc_refresh_queue_view()
        self._ploc_advance()

    def _ploc_cancel_run(self) -> None:
        """Abort an in-progress run, discarding its results."""
        # v7.5.x: the Cancel button also stops an in-progress mosaic scan.
        if getattr(self, "_ploc_mosaic_running", False):
            self._ploc_cancel_mosaic_scan()
            return
        if not self._ploc_running:
            return
        self._ploc_running = False
        self._ploc_steps = []
        self._ploc_run_idx = 0
        self._ploc_well_results = {}
        self._free_teach_pairs = []
        self._ploc_pause_kind = None
        self._ploc_manual_well = None
        self._ploc_manual_points = []
        self._ploc_rim_targets = []
        self._ploc_rim_idx = 0
        self._ploc_click_points = []
        try:
            self._ploc_live_view.set_overlay_vector(None, None)
        except Exception:
            pass
        self._ploc_live_hint.setText("")
        self._ploc_exit_confirm_mode()
        self._ploc_btn_clear.setEnabled(True)
        self._ploc_refresh_queue_view()
        self._ploc_status.setText("Run cancelled.")

    def _ploc_finish_run(self) -> None:
        """Feed all collected points (well fits + freeform teach pairs)
        into the affine and refresh the UI."""
        self._ploc_running = False
        try:
            self._ploc_live_view.set_overlay_vector(None, None)
        except Exception:
            pass
        self._ploc_live_hint.setText("")
        self._ploc_fits = dict(self._ploc_well_results)
        # v7.5.x: also capture auto-fit well centres as reference markers
        # (manual paths already added theirs live).
        self._reference_markers.update(self._ploc_well_results)
        n_taught = sum(
            1 for e in self._ploc_queue
            if isinstance(e, dict) and e.get("status") == "taught")
        n_skipped = sum(
            1 for e in self._ploc_queue
            if isinstance(e, dict) and e.get("status") == "skipped")
        self._ploc_exit_confirm_mode()
        self._ploc_btn_clear.setEnabled(True)
        self._ploc_refresh_queue_view()
        # _ploc_feed_affine invokes _manual_fit_xy, which also consumes
        # self._free_teach_pairs (the freeform manual-teach pairs).
        try:
            self._ploc_feed_affine(self._ploc_well_results)
        except Exception as e:
            logger.warning(f"PlateLocation: affine fit failed: {e}")
        # Freeform pairs are now folded into the saved warp; clear so a
        # later manual-tab refit doesn't double-count stale points.
        self._free_teach_pairs = []
        # RMS of the per-point deltas (how far off the points were overall).
        deltas = [(e["dx"], e["dy"]) for e in self._ploc_queue
                  if isinstance(e, dict) and e.get("status") == "taught"
                  and "dx" in e]
        rms_txt = ""
        if deltas:
            rms = (sum(dx * dx + dy * dy for dx, dy in deltas)
                   / len(deltas)) ** 0.5
            rms_txt = f" — freeform RMS Δ {rms:.0f} µm"
        self._ploc_status.setText(
            f"Done — {len(self._ploc_well_results)} wells fitted, "
            f"{n_taught} freeform taught, {n_skipped} skipped." + rms_txt
        )

    def _predict_well_xy(self, well_name: str) -> tuple[float, float]:
        """Best-effort predicted stage XY (µm) for `well_name`.

        Returns absolute stage µm so the caller can pass it straight to
        ``move_xy_absolute_um``.

        v7.5.x: reads the SAME position dict the Plate Location canvas
        draws (``_calibrated_positions`` preferred, else
        ``_predicted_positions``) so the stage drives to exactly the
        well the operator sees and snapped. These dicts are stored in
        absolute stage µm (the canvas subtracts ``zero_position`` only
        for drawing), so they are returned directly — NOT offset by
        ``zero_position``. The prior code keyed off ``_predicted_wells``,
        an attribute never assigned anywhere, and added ``zero_position``
        to it; that branch never fired, so every snapped move fell
        through to the zero-anchored grid fallback below and landed on a
        grid offset from stage-zero (between wells) instead of on the
        drawn well.
        """
        zero = self.controller.zero_position
        # Primary: the displayed / working positions, already absolute
        # stage µm. Calibrated takes precedence over predicted (matches
        # ``_wells_in_zero_ref`` / the canvas).
        positions = self._calibrated_positions or self._predicted_positions
        if positions and well_name in positions:
            return positions[well_name]
        # Fallback: A1-relative geometry (no position dict yet).
        wells_by_name = {w.name: w for w in self._plate.get_all_wells()}
        a1 = wells_by_name["A1"]
        target = wells_by_name[well_name]
        sx, sy = self._plate_axis_sign()
        dx_um = sx * (target.x - a1.x) * 1000.0
        dy_um = sy * (target.y - a1.y) * 1000.0
        # If A1 has been taught, anchor at that stage position (same
        # formula as WellPlate.get_all_positions_from_a1).
        if getattr(self, '_taught_a1', None):
            ax, ay = self._taught_a1
            return (ax + dx_um, ay + dy_um)
        # No prediction and no taught A1: mirror the canvas geometry-only
        # seed (plate centred on the XY safety-envelope midpoint) so the
        # move still matches the drawn map.
        try:
            cx, cy = self.controller.default_plate_center_um()
            seeded = self._plate.get_all_positions_from_plate_center(
                cx, cy, self._plate_axis_sign())
            if well_name in seeded:
                return seeded[well_name]
        except Exception:
            pass
        # Last resort: anchor at stage zero.
        return (zero.get("x", 0) + dx_um, zero.get("y", 0) + dy_um)

    def _ploc_feed_affine(
        self, results: dict[str, tuple[float, float]]
    ) -> None:
        """Push the fitted well centers into the affine fitter.

        ``results`` maps well name → measured center in absolute stage
        µm. These are written into ``_xy_teach_points`` (the store
        ``_manual_fit_xy`` actually pairs against the geometry-predicted
        positions) plus the legacy A1/corner/third trackers, then the
        fit is recomputed. ``_manual_fit_xy`` additionally folds in any
        freeform pairs accumulated in ``_free_teach_pairs``.
        """
        for name, (sx, sy) in results.items():
            # _manual_fit_xy and the manual-teach tab both key on
            # absolute stage µm — store the measured centers directly.
            self._xy_teach_points[name] = (sx, sy)
            if name == "A1":
                self._taught_a1 = (sx, sy)
            elif getattr(self, '_corner_well', None) == name:
                self._taught_corner = (sx, sy)
            elif self._taught_third is None:
                self._third_well = name
                self._taught_third = (sx, sy)
        try:
            self._manual_fit_xy()
            self._emit_calibration_data_changed()
        except Exception as e:
            logger.warning(f"PlateLocation: _manual_fit_xy failed: {e}")

    # ════════════════════════════════════════════════════════════════
    #  v7.5.x: Full-plate MOSAIC scan — raster the whole plate, stitch
    #  every camera snapshot into one composite, detect ALL wells on that
    #  field at once (single-frame per-well edge detection is unreliable),
    #  and use the stitched image as a toggleable background overlay.
    # ════════════════════════════════════════════════════════════════

    def _ploc_plate_key(self) -> str:
        """Stable key for the active plate (used by the mosaic store).

        v7.5.x: prefer the hardware config's ``active_plate_key`` — the
        authoritative identity. A plate TYPE (Corning/NEST/…) shares its base
        format's geometry, so ``self._plate.format`` is the base int and would
        COLLIDE across types of one format; ``active_plate_key`` carries the
        distinct type id so each type's mosaic/template segregates. Fall back
        to the live ``WellPlate.format`` when no hardware config is present.
        """
        hw = getattr(self, "_hardware_config", None)
        key = getattr(hw, "active_plate_key", None) if hw is not None else None
        if key is None:
            key = (getattr(self._plate, "format", None)
                   if self._plate is not None else None)
        return str(key) if key is not None else "plate"

    def _ploc_microscope_cam_idx(self):
        """Index of the camera assigned the Microscope role (or None)."""
        hw = getattr(self, "_hardware_config", None)
        if hw is not None and CameraRole is not None:
            try:
                return hw.camera_for_role(CameraRole.MICROSCOPE)
            except Exception:
                return None
        return None

    def _ploc_microscope_um_per_px(self, frame_w, fallback):
        """µm/px for the microscope at the CURRENT capture width.

        The objective calibration's ``measured_um_per_px`` was taken at a
        specific resolution; if the camera now captures at a different width
        (e.g. switched 912→1832 px), µm/px scales inversely. We rescale by
        ``cal_width / current_width`` so the mosaic FOV is correct — otherwise
        the FOV is wrong (too large if calibrated at a smaller width) and the
        raster needs huge overlap just to keep tiles touching. Falls back to the
        live CameraManager value (which is NOT resolution-scaled) when no
        objective calibration with a recorded resolution is available.
        """
        try:
            from SupportClasses.ObjectiveCalibration import get_store as _objs
            hw = getattr(self, "_hardware_config", None)
            cam_cfg = getattr(hw, "camera_config", None) if hw else None
            obj = (getattr(cam_cfg, "current_objective_name", None)
                   if cam_cfg else None)
            spec = getattr(cam_cfg, "camera_spec", None) if cam_cfg else None
            cam_name = getattr(spec, "name", None) if spec else None
            if cam_name and obj:
                cal = _objs().get_calibration(str(cam_name), str(obj))
                if cal:
                    meas = float(cal.get("measured_um_per_px") or 0.0)
                    res = cal.get("resolution")
                    cal_w = float(res[0]) if (res and len(res) >= 1) else 0.0
                    if meas > 0 and cal_w > 0 and frame_w > 0:
                        eff = meas * (cal_w / float(frame_w))
                        if abs(cal_w - frame_w) > 1:
                            logger.info(
                                f"Mosaic µm/px rescaled for resolution: "
                                f"{meas:.4f}@{cal_w:.0f}px → {eff:.4f}@"
                                f"{frame_w:.0f}px")
                        return eff
        except Exception as e:
            logger.debug(f"microscope µm/px scale skipped: {e}")
        return fallback

    def _ploc_camera_objective_key(self) -> str:
        """Key for the mosaic-alignment store: microscope camera identity +
        current objective (the learned correction is a property of both).
        Falls back to objective-only, then 'default'."""
        obj = None
        hw = getattr(self, "_hardware_config", None)
        cam_cfg = getattr(hw, "camera_config", None) if hw is not None else None
        if cam_cfg is not None:
            obj = getattr(cam_cfg, "current_objective_name", None)
        obj = obj or "default"
        ident = None
        mgr = self._camera_manager
        cam_idx = self._ploc_microscope_cam_idx()
        if mgr is not None and cam_idx is not None:
            try:
                res = mgr.camera_identity(cam_idx)   # (key, name) | None
                if res:
                    ident = res[0]
            except Exception:
                ident = None
        return f"{ident}|{obj}" if ident else str(obj)

    def _ploc_mosaic_align_store(self):
        try:
            from SupportClasses.MosaicAlignmentStore import get_store
            return get_store()
        except Exception as e:
            logger.debug(f"MosaicAlignmentStore unavailable: {e}")
            return None

    def _ploc_mosaic_store(self):
        try:
            from SupportClasses.MosaicStore import get_store
            return get_store()
        except Exception as e:
            logger.debug(f"MosaicStore unavailable: {e}")
            return None

    def _ploc_start_mosaic_scan(self) -> None:
        """Raster the whole plate, stitch a mosaic, detect every well, fit.

        Thin wrapper: guards re-entry and SURFACES any failure — so a click is
        never a silent no-op and a mid-setup exception can't leave the busy flag
        stuck (which would make every later click silently return).
        """
        logger.info("Mosaic scan requested")
        if self._ploc_running or getattr(self, "_ploc_mosaic_running", False):
            logger.info(
                "Mosaic scan ignored — already busy "
                f"(queue_run={self._ploc_running}, "
                f"mosaic={getattr(self, '_ploc_mosaic_running', False)})")
            QMessageBox.information(
                self, "Mosaic scan",
                "Another operation is already running on this tab (a target "
                "queue run or a mosaic scan). Finish or Cancel it first, then "
                "try the Mosaic scan again.")
            return
        try:
            self._ploc_start_mosaic_scan_impl()
        except Exception as e:
            logger.exception("Mosaic scan start failed")
            self._ploc_mosaic_running = False
            try:
                self._ploc_mosaic_cleanup_ui()
            except Exception:
                pass
            QMessageBox.warning(
                self, "Mosaic scan",
                f"Could not start the mosaic scan:\n{e}")

    def _ploc_start_mosaic_scan_impl(self) -> None:
        """Mosaic-scan setup body (see the _ploc_start_mosaic_scan wrapper)."""
        # v7.5.x: snapshot + CLEAR the single-well override up-front so any
        # early-return gate below (Safe-Z / camera / µm/px / frames) can't
        # leave stale flags that would mis-route a LATER full-plate scan into
        # the single-well path. Re-armed right before the worker launches.
        _sw_override = getattr(self, "_ploc_scan_bounds_override", None)
        _sw_parent = getattr(self, "_ploc_scan_subwell_parent", None)
        _sw_rosette = bool(getattr(self, "_ploc_scan_well_is_rosette", False))
        self._ploc_scan_bounds_override = None
        self._ploc_scan_subwell_parent = None
        self._ploc_scan_well_is_rosette = False
        if self.controller is None or self._plate is None:
            QMessageBox.warning(
                self, "Mosaic scan", "Stage controller and plate required.")
            return
        if not NEEDLE_LOCATION_AVAILABLE:
            QMessageBox.warning(
                self, "Mosaic scan",
                "Vision helpers not available — install opencv-python.")
            return
        # Z must be set up first (the raster retracts before each XY hop).
        zp_connected = bool(getattr(self.controller, 'is_zp_connected', False))
        if zp_connected and getattr(self, '_safe_z', None) is None:
            QMessageBox.warning(
                self, "Mosaic scan",
                "Set the Safe / Move Z first (Needle Offset Calibration "
                "tab). The scan retracts the needle to that height before "
                "every XY move.")
            return
        if self._camera_manager is None:
            QMessageBox.warning(
                self, "Mosaic scan", "Camera manager required.")
            return
        hw = getattr(self, '_hardware_config', None)
        cam_idx = (hw.camera_for_role(CameraRole.MICROSCOPE)
                   if (hw is not None and CameraRole is not None) else None)
        if cam_idx is None:
            QMessageBox.warning(
                self, "Mosaic scan",
                "Assign a camera the Microscope role in "
                "Hardware Setup → Cameras.")
            return
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            QMessageBox.warning(
                self, "Mosaic scan", f"Camera index {cam_idx} not available.")
            return
        if not getattr(cam, 'is_running', False):
            try:
                self._camera_manager.start(cam_idx)
            except Exception:
                pass
        frame = None
        try:
            frame = cam.get_current_frame()
            if frame is None and hasattr(cam, "capture_fresh_frame"):
                # The camera may have only just been started — give it a moment
                # to deliver the first frame before giving up.
                frame = cam.capture_fresh_frame(
                    discard_n_frames=2, settle_ms=300)
        except Exception as e:
            logger.warning(f"Mosaic scan: frame grab failed: {e}")
            frame = None
        if frame is None:
            QMessageBox.warning(
                self, "Mosaic scan",
                "Microscope camera is not producing frames yet. Start it on "
                "the Cameras tab (or wait for the live view to appear), then "
                "retry.")
            return
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            QMessageBox.warning(
                self, "Mosaic scan",
                "Microscope camera µm/pixel not calibrated. Calibrate in "
                "Hardware Setup → Cameras.")
            return
        um_per_px = float(self._camera_manager.get_um_per_px(cam_idx) or 0.0)
        if um_per_px <= 0:
            QMessageBox.warning(
                self, "Mosaic scan",
                "Microscope camera µm/pixel not calibrated.")
            return
        self._ploc_live_cam_idx = cam_idx

        # Ensure predicted well positions exist (seed from plate centre).
        if not (self._calibrated_positions or self._predicted_positions):
            try:
                cx, cy = self.controller.default_plate_center_um()
                self._predicted_positions = (
                    self._plate.get_all_positions_from_plate_center(
                        cx, cy, self._plate_axis_sign()))
            except Exception as e:
                QMessageBox.warning(
                    self, "Mosaic scan",
                    f"Could not compute predicted well positions: {e}")
                return
        positions = self._calibrated_positions or self._predicted_positions
        if not positions:
            QMessageBox.warning(
                self, "Mosaic scan", "No well positions to scan.")
            return

        # v7.5.x: DEFAULT whole-plate scan region = the ENTIRE reachable XY
        # travel envelope, so the full XY space is mapped (see
        # _ploc_whole_plate_scan_bounds). A single-well (rosette) override below
        # narrows the scan to one well.
        sl = getattr(self.controller, "safety_limits", None)
        env = None
        if sl is not None:
            try:
                env = (float(sl.xy_min_x), float(sl.xy_min_y),
                       float(sl.xy_max_x), float(sl.xy_max_y))
            except Exception:
                env = None
        bounds = self._ploc_whole_plate_scan_bounds(positions, env)
        # Single-well scan: override the whole-plate bounds with the chosen
        # well's bounds (clipped to the envelope). Uses the snapshot taken at
        # the top (instance flags already cleared; re-armed at launch below).
        ov = _sw_override
        if ov is not None:
            bounds = self._ploc_clip_scan_bounds(
                [ov[0], ov[2]], [ov[1], ov[3]], 0.0, env)
        if bounds is None:
            QMessageBox.warning(
                self, "Mosaic scan",
                "The scan region falls outside the reachable XY travel "
                "envelope — nothing to scan. Check the plate calibration and "
                "the device XY travel limits.")
            return

        fh, fw = frame.shape[:2]
        try:
            from SupportClasses.MosaicBuilder import MosaicBuilder
        except ImportError:
            QMessageBox.warning(
                self, "Mosaic scan", "MosaicBuilder unavailable.")
            return
        cfg = getattr(self, "_mosaic_settings", None) or {}
        overlap_frac = float(cfg.get("overlap_pct", 25)) / 100.0
        target_px = int(cfg.get("target_px", 3000))
        align_store = self._ploc_mosaic_align_store()
        align_key = self._ploc_camera_objective_key()
        # Effective µm/px: explicit FOV override > LEARNED (Quick FOV calibration
        # or a prior mosaic for this camera+objective) > camera µm/px. The
        # learned value sizes tiles correctly so the live registration has very
        # little to correct.
        fov_um = float(cfg.get("fov_um", 0) or 0)
        if fov_um > 0 and fw > 0:
            eff_um_per_px = fov_um / fw
        else:
            learned = (align_store.get_um_per_px(align_key)
                       if align_store is not None else None)
            # Resolution-scaled objective µm/px (fixes the FOV when the camera's
            # capture resolution differs from the objective-calibration one).
            cam_um = self._ploc_microscope_um_per_px(fw, um_per_px)
            eff_um_per_px = learned if (learned and learned > 0) else cam_um
        # Pre-seed the learned global-registration shift so the mosaic is
        # registered from the first tile; finalize refines it.
        init_shift = (0.0, 0.0)
        if align_store is not None:
            s = align_store.get_shift_um(align_key)
            if s:
                # The registration/manual shift corrects sub-FOV stitch residual
                # for the OVERLAY only; a value of several FOV is corrupt (e.g. a
                # stale manual_align nudge) and would fling the overlay off-plate.
                # Bound it so a bad stored value can't poison a fresh scan.
                fov_w_um = max(1.0, fw * eff_um_per_px)
                fov_h_um = max(1.0, fh * eff_um_per_px)
                lim_x, lim_y = 2.0 * fov_w_um, 2.0 * fov_h_um
                if abs(s[0]) > lim_x or abs(s[1]) > lim_y:
                    logger.warning(
                        f"Mosaic: ignoring stored alignment shift "
                        f"({s[0]:.0f}, {s[1]:.0f}) µm — exceeds ±2 FOV "
                        f"(±{lim_x:.0f}, ±{lim_y:.0f}); treating as corrupt.")
                else:
                    init_shift = s
        # Manual grid spacing override (µm); 0 = auto (FOV × (1 − overlap)).
        spacing_um = float(cfg.get("spacing_um", 0) or 0)
        register = bool(cfg.get("register", True))
        max_shift_um = float(cfg.get("max_shift_um", 0) or 0)
        builder = MosaicBuilder(
            frame_size_px=(fw, fh), micron_per_pixel=eff_um_per_px,
            overlap=overlap_frac, target_mosaic_px=target_px,
            register=register, max_shift_um=max_shift_um,
            initial_shift_um=init_shift,
            # v7.5.x: full-plate scan can be hundreds of tiles — free each raw
            # frame after it's blended so RAM stays bounded (this path consumes
            # only the live ``.composite`` display cache, never re-blends).
            retain_frames=False)
        # Raster spaced by the camera FOV at the configured overlap, or by the
        # explicit grid spacing when set.
        step = spacing_um if spacing_um > 0 else None
        grid = builder.generate_raster_positions(
            bounds, overlap=overlap_frac, step_x_um=step, step_y_um=step)
        # Belt-and-suspenders: drop any grid point outside the envelope so no
        # move ever clamps (the half-FOV inset should already keep centres in,
        # but a collapsed-to-centre tiny region is guarded here too).
        if env is not None:
            grid = [(x, y) for (x, y) in grid
                    if env[0] <= x <= env[2] and env[1] <= y <= env[3]]
        if not grid:
            QMessageBox.warning(
                self, "Mosaic scan",
                "No reachable raster points were generated for this plate.")
            return

        # All validation passed + computed. CLAIM the busy state BEFORE the
        # (modal) objective-confirm dialog: that dialog spins a nested event
        # loop, and without this a Run click during it would start a queue run
        # concurrently with the mosaic (conflicting XY moves). _ploc_run_queue
        # also guards on _ploc_mosaic_running.
        self._ploc_mosaic_running = True
        self._ploc_btn_run.setEnabled(False)
        self._ploc_btn_clear.setEnabled(False)
        if getattr(self, "_ploc_btn_mosaic", None) is not None:
            self._ploc_btn_mosaic.setEnabled(False)

        # Objective gate (same as the queue run). Decline → release busy state.
        if not self._ploc_confirm_objective(cam_idx, um_per_px):
            self._ploc_mosaic_running = False
            self._ploc_mosaic_cleanup_ui()
            return

        # Tile-count confirmation. A full-plate mosaic at the microscope FOV can
        # be thousands of tiles (each = move + settle + stitch), i.e. a long
        # unattended run — make the scale explicit and cancelable so it isn't
        # mistaken for a hang.
        n_tiles = len(grid)
        est_min = n_tiles * 1.5 / 60.0   # ~1.5 s/tile rough estimate
        proceed = QMessageBox.question(
            self, "Mosaic scan",
            f"This will raster {n_tiles} tiles at the current objective FOV "
            f"(~{est_min:.0f} min). The needle stays retracted at Safe Z and "
            f"the UI stays responsive — you can Cancel any time.\n\nStart the "
            f"mosaic scan?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if proceed != QMessageBox.StandardButton.Yes:
            self._ploc_mosaic_running = False
            self._ploc_mosaic_cleanup_ui()
            return

        self._ploc_mosaic_cam = cam
        self._ploc_mosaic_um_per_px = um_per_px
        self._ploc_mosaic_builder = builder
        self._ploc_mosaic_positions = grid
        self._ploc_mosaic_index = 0
        self._ploc_mosaic_total = len(grid)
        # Show the composite live as it stitches (operator watches it build).
        self._ploc_show_mosaic_mode()
        self._ploc_reset_mosaic_preview()
        self._ploc_ensure_live_camera()

        # One-time pre-scan retract so the raster never drags a lowered needle.
        try:
            if hasattr(self.controller, "ensure_retracted_to"):
                self.controller.ensure_retracted_to(
                    self._safe_z if getattr(self, "_safe_z", None) is not None
                    else 0.0)
        except Exception as e:
            logger.warning(f"Mosaic scan: pre-scan retract failed: {e}")

        total = len(grid)
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(f"Mosaic scan: 0/{total} tiles…")
        # Show only Cancel from the confirm row.
        self._ploc_btn_cancel.setVisible(True)
        self._ploc_btn_cancel.setEnabled(True)

        # Detection parameters (applied to the final stitched mosaic, in px).
        well_d_um, _pitch = self._ploc_mosaic_metrics()
        scale_est = float(getattr(builder, "_mosaic_scale", 0.0) or 0.0)
        expected_d_px = well_d_um * scale_est
        min_dist_px = max(1.0, _pitch * scale_est * 0.7)

        # Camera-timing + detection knobs from the Settings dialog.
        settle_ms = int(cfg.get("settle_ms", 300))
        fresh_frames = int(cfg.get("fresh_frames", 3))
        fresh_timeout_s = float(cfg.get("fresh_timeout_s", 2.5))
        detect_param2 = float(cfg.get("detect_param2", 30))
        detect_tol = float(cfg.get("detect_tol_pct", 35)) / 100.0
        frame_orient = str(cfg.get("frame_orient", "none"))

        # Re-arm the single-well flags now that every gate has passed — the
        # finish handler routes on them (see the snapshot+clear at the top).
        self._ploc_scan_bounds_override = _sw_override
        self._ploc_scan_subwell_parent = _sw_parent
        self._ploc_scan_well_is_rosette = _sw_rosette

        # Run the raster + stitch + detect on a BACKGROUND thread so the camera
        # grab timer + UI stay responsive (no more freezing). The worker only
        # commands stage moves and samples frames thread-safely.
        safe_z = (self._safe_z if getattr(self, "_safe_z", None) is not None
                  else 0.0)
        self._ploc_mosaic_worker = _MosaicScanWorker(
            self.controller, cam, builder, grid, safe_z,
            expected_d_px, min_dist_px,
            fresh_frames=fresh_frames, fresh_timeout_s=fresh_timeout_s,
            settle_ms=settle_ms, detect_param2=detect_param2,
            detect_tolerance=detect_tol, frame_orient=frame_orient)
        self._ploc_mosaic_worker.progress.connect(
            self._ploc_on_mosaic_progress)
        self._ploc_mosaic_worker.tile.connect(self._ploc_on_mosaic_tile)
        self._ploc_mosaic_worker.finished_ok.connect(
            self._ploc_on_mosaic_finished)
        self._ploc_mosaic_worker.failed.connect(self._ploc_on_mosaic_failed)
        self._ploc_mosaic_worker.start()
        logger.info(
            f"PlateLocation mosaic scan: {total} tiles over {bounds} µm "
            f"(overlap {overlap_frac:.0%}, settle {settle_ms}ms, "
            f"{fresh_frames} fresh frames, worker thread)")

    # ── Mosaic scan worker signal handlers (run on the GUI thread) ──

    def _ploc_on_mosaic_progress(self, done: int, total: int) -> None:
        # Ignore stale signals delivered after a cancel (running is False then).
        if not self._ploc_mosaic_running:
            return
        self._ploc_confirm_label.setText(f"Mosaic scan: {done}/{total} tiles…")

    def _ploc_on_mosaic_tile(self, composite, extent) -> None:
        """A tile was stitched — update the live preview + plate overlay."""
        if not self._ploc_mosaic_running:
            return
        if composite is None or extent is None:
            return
        self._ploc_update_mosaic_preview(composite)
        try:
            self._ploc_set_overlay_image(composite, extent)
        except Exception:
            pass

    def _ploc_on_mosaic_finished(self, composite, extent, scale, frames,
                                 detections) -> None:
        """Worker finished: persist + overlay + fit the warp from detections."""
        # Ignore a stale finished delivered after a cancel.
        if not self._ploc_mosaic_running:
            return
        self._ploc_mosaic_running = False
        self._ploc_mosaic_cleanup_ui()
        self._ploc_mosaic_worker = None
        # Single-well scan → persist the well's mosaic separately, then map it
        # (rosette: sub-wells; plain well: pick the centre) — NOT the plate.
        parent = getattr(self, "_ploc_scan_subwell_parent", None)
        if parent:
            is_rosette = bool(getattr(self, "_ploc_scan_well_is_rosette",
                                      True))
            # Capture the builder's global-registration shift BEFORE the
            # builder is released — the extent is display-registered (shifted)
            # while the pixels sit in the trusted raw stage frame; the mapping
            # back-projection must use extent − shift.
            shift = self._ploc_mosaic_world_shift()
            self._ploc_scan_subwell_parent = None
            self._ploc_scan_bounds_override = None
            self._ploc_scan_well_is_rosette = False
            self._ploc_mosaic_builder = None
            self._ploc_apply_single_well_mosaic(
                parent, composite, extent, scale, frames, shift_um=shift)
            # Refresh the plate overlay so the new well scan composites in,
            # and let the Jog page reload its copy from the store.
            try:
                self._ploc_load_persisted_mosaic()
            except Exception:
                pass
            try:
                self.calibration_data_changed.emit()
            except Exception:
                pass
            try:
                self._rosette_refresh_wells()
            except Exception:
                pass
            self._ploc_open_single_well_mapping(
                parent, composite, extent, scale, is_rosette,
                shift_um=shift)
            return
        if composite is None:
            self._ploc_confirm_label.setText(
                "Mosaic scan: stitching produced no image.")
            self._ploc_mosaic_builder = None
            return
        self._ploc_update_mosaic_preview(composite)
        # Learn the global registration shift for this camera + objective so the
        # next mosaic starts pre-registered. Only store when SOMETHING actually
        # registered — a featureless plate measures nothing, and we must not
        # overwrite a previously-learned (or manually-stored) shift with a null.
        n_meas = len(
            getattr(self._ploc_mosaic_builder, "_measured_shifts", None) or [])
        kept_manual = False
        try:
            shift = getattr(self._ploc_mosaic_builder, "_global_shift_um", None)
            store = self._ploc_mosaic_align_store()
            if shift is not None and store is not None and n_meas > 0:
                if store.is_manual(self._ploc_camera_objective_key()):
                    # A deliberate by-eye manual_align is authoritative — don't
                    # silently clobber it with an auto median. Clear it (Manual
                    # align → Clear stored) to let auto take over again.
                    kept_manual = True
                    logger.info(
                        "Mosaic scan: keeping stored manual_align shift "
                        "(auto registration not overwriting).")
                else:
                    store.set_shift_um(
                        self._ploc_camera_objective_key(),
                        shift[0], shift[1], frames=frames, source="mosaic_scan")
        except Exception as e:
            logger.debug(f"Mosaic alignment store (shift) skipped: {e}")
        self._ploc_apply_mosaic(composite, extent, scale, frames)
        n_fit = self._ploc_fit_from_mosaic_detections(
            detections, extent, scale)
        dropped = max(0, getattr(self, "_ploc_mosaic_total", frames) - frames)
        drop_txt = f", {dropped} tiles dropped (camera)" if dropped else ""
        if kept_manual:
            hint = ("  Kept your stored manual alignment (auto not overwriting) "
                    "— use Manual align → Clear stored to let auto take over.")
        elif n_meas > 0:
            hint = ""
        else:
            hint = ("  Auto registration found no overlaps — use “Manual align” "
                    "to slide it onto the wells, then Store alignment.")
        # If auto-detect mapped few/no wells, point the operator at manual mapping.
        map_hint = ("  Few wells auto-detected — confirm the corners in the "
                    "mapping dialog." if n_fit < 3 else "")
        self._ploc_confirm_label.setText(
            f"Mosaic done: {frames} tiles stitched, "
            f"{len(detections or [])} circles, {n_fit} wells fitted{drop_txt}."
            f"{hint}{map_hint}")
        self._ploc_mosaic_builder = None
        logger.info(
            f"PlateLocation mosaic finished: {frames} tiles, {n_fit} wells "
            f"fit, {dropped} dropped, {n_meas} overlaps registered")
        # v7.5.x: mapping is part of the scan flow (operator decision) — auto-
        # open the guided Map-wells dialog on the freshly stored mosaic
        # (auto-detect pre-places every MAIN well; the operator confirms /
        # refines / cancels). The store path recovers the trusted
        # back-projection frame via the recorded shift_um.
        try:
            self._ploc_open_well_mapping()
        except Exception as e:
            logger.warning(f"Auto-open well mapping after scan failed: {e}")

    def _ploc_on_mosaic_failed(self, msg: str) -> None:
        if not self._ploc_mosaic_running:
            return
        self._ploc_mosaic_running = False
        self._ploc_scan_bounds_override = None
        self._ploc_scan_subwell_parent = None
        self._ploc_scan_well_is_rosette = False
        self._ploc_mosaic_cleanup_ui()
        self._ploc_mosaic_builder = None
        self._ploc_mosaic_worker = None
        self._ploc_confirm_label.setText(f"Mosaic scan failed: {msg}")
        logger.warning(f"PlateLocation mosaic scan failed: {msg}")

    def _ploc_mosaic_cleanup_ui(self) -> None:
        self._ploc_btn_run.setEnabled(True)
        self._ploc_btn_clear.setEnabled(True)
        if getattr(self, "_ploc_btn_mosaic", None) is not None:
            self._ploc_btn_mosaic.setEnabled(True)
        self._ploc_btn_cancel.setVisible(False)

    def _ploc_whole_plate_scan_bounds(self, positions, env):
        """Default whole-plate mosaic scan region (absolute stage µm).

        v7.5.x: the ENTIRE reachable XY travel envelope, so the full XY space is
        mapped (not just the well-centre extent). ``generate_raster_positions``
        insets by half-FOV so the camera CENTRES stay inside, and grid points
        outside the envelope are dropped, so no move clamps to the boundary.
        Falls back to the well-centre extent + margin (well radius + 1 mm) when
        no usable XY envelope (``env`` = (min_x, min_y, max_x, max_y)) is
        configured. Returns a (min_x, min_y, max_x, max_y) tuple or None.
        """
        if env is not None and env[2] > env[0] and env[3] > env[1]:
            return (float(env[0]), float(env[1]), float(env[2]), float(env[3]))
        xs = [p[0] for p in positions.values()]
        ys = [p[1] for p in positions.values()]
        if not xs or not ys:
            return None
        well_d_um, _pitch = self._ploc_mosaic_metrics()
        margin = well_d_um / 2.0 + 1000.0
        return self._ploc_clip_scan_bounds(xs, ys, margin, env)

    def _ploc_clip_scan_bounds(self, xs, ys, margin, env):
        """Well-extent ± margin, clipped to the reachable XY envelope.

        ``env`` = (xy_min_x, xy_min_y, xy_max_x, xy_max_y) absolute stage µm,
        or None to skip clipping. Returns a (min_x, min_y, max_x, max_y) tuple,
        or None when the intersection is empty (wells entirely outside reach).
        """
        b = [min(xs) - margin, min(ys) - margin,
             max(xs) + margin, max(ys) + margin]
        if env is not None:
            b = [max(b[0], env[0]), max(b[1], env[1]),
                 min(b[2], env[2]), min(b[3], env[3])]
        if b[2] <= b[0] or b[3] <= b[1]:
            return None
        return (b[0], b[1], b[2], b[3])

    def _ploc_mosaic_metrics(self) -> tuple[float, float]:
        """(well_diameter_µm, well_pitch_µm) for the active plate (fallbacks)."""
        try:
            well_d_um = float(self._plate.well_diameter) * 1000.0
        except Exception:
            well_d_um = 0.0
        if well_d_um <= 0:
            well_d_um = 6000.0
        try:
            pitch_um = min(float(self._plate.well_spacing_x),
                           float(self._plate.well_spacing_y)) * 1000.0
        except Exception:
            pitch_um = well_d_um
        if pitch_um <= 0:
            pitch_um = well_d_um
        return well_d_um, pitch_um

    def _ploc_open_mosaic_settings(self) -> None:
        """Pop out the mosaic-scan settings dialog; apply + persist on OK."""
        if getattr(self, "_ploc_mosaic_running", False):
            QMessageBox.information(
                self, "Mosaic scan",
                "Finish or Cancel the running mosaic scan before changing "
                "its settings.")
            return
        try:
            from gui.dialogs.mosaic_settings_dialog import (
                MosaicScanSettingsDialog)
        except Exception as e:
            logger.warning(f"Mosaic settings dialog unavailable: {e}")
            return
        dlg = MosaicScanSettingsDialog(
            getattr(self, "_mosaic_settings", None), parent=self)
        if dlg.exec():
            self._mosaic_settings = dlg.values()
            if self.settings is not None:
                try:
                    self.settings.set_section(
                        "mosaic_scan", self._mosaic_settings)
                    self.settings.save()
                except Exception as e:
                    logger.warning(f"Mosaic settings save failed: {e}")
            logger.info(f"Mosaic settings updated: {self._mosaic_settings}")

    # ── Small-mosaic alignment calibration (per camera + objective) ──

    def _ploc_calibration_center(self):
        """Absolute-µm point to centre the calibration mosaic on — the well
        nearest the plate centre (texture), else the envelope centre."""
        positions = self._calibrated_positions or self._predicted_positions
        try:
            cx, cy = self.controller.default_plate_center_um()
        except Exception:
            cx, cy = 0.0, 0.0
        if positions:
            best = min(positions.values(),
                       key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            return (float(best[0]), float(best[1]))
        return (float(cx), float(cy))

    def _ploc_quick_mosaic_calibrate(self) -> None:
        """Open the small-mosaic alignment calibration pop-out: the user defines
        a small (default 5×5) grid + settings, it builds that mosaic and runs the
        global registration to estimate the alignment delta, stored per
        camera + objective and pre-applied to future full mosaics."""
        if getattr(self, "_ploc_mosaic_running", False):
            QMessageBox.information(
                self, "Calibration",
                "Wait for the mosaic scan to finish before calibrating.")
            return
        if self.controller is None or self._camera_manager is None:
            QMessageBox.warning(
                self, "Calibration",
                "Stage controller and camera manager required.")
            return
        # CRITICAL SAFETY: same gate as the full scan. Without a Safe Z (and a
        # connected ZP), the calibration mosaic's retract would fall back to
        # zero-ref 0 — the plate-bottom datum on ME3B V1 (ZDIR=-1) — driving the
        # needle DOWN into the plate before the XY raster.
        zp_connected = bool(getattr(self.controller, 'is_zp_connected', False))
        if zp_connected and getattr(self, '_safe_z', None) is None:
            QMessageBox.warning(
                self, "Calibration",
                "Set the Safe / Move Z first (Needle Offset Calibration tab). "
                "The calibration mosaic retracts the needle to that height "
                "before moving between tiles.")
            return
        cam_idx = self._ploc_microscope_cam_idx()
        if cam_idx is None:
            QMessageBox.warning(
                self, "Calibration",
                "Assign a camera the Microscope role in "
                "Hardware Setup → Cameras.")
            return
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            QMessageBox.warning(
                self, "Calibration", f"Camera {cam_idx} not available.")
            return
        if not getattr(cam, "is_running", False):
            try:
                self._camera_manager.start(cam_idx)
            except Exception:
                pass
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            QMessageBox.warning(
                self, "Calibration",
                "Calibrate the microscope µm/pixel first (Hardware Setup → "
                "Cameras).")
            return
        um0 = float(self._camera_manager.get_um_per_px(cam_idx) or 0.0)
        if um0 <= 0:
            QMessageBox.warning(
                self, "Calibration", "Microscope µm/pixel not calibrated.")
            return
        self._ploc_live_cam_idx = cam_idx
        self._ploc_ensure_live_camera()
        frame = None
        try:
            frame = cam.get_current_frame()
            if frame is None:
                frame = cam.capture_fresh_frame(discard_n_frames=2,
                                                settle_ms=300)
        except Exception:
            frame = None
        if frame is None:
            QMessageBox.warning(
                self, "Calibration",
                "Microscope camera is not producing frames yet. Start it on "
                "the Cameras tab, then retry.")
            return
        fh, fw = frame.shape[:2]

        try:
            from gui.dialogs.mosaic_calibration_dialog import (
                MosaicCalibrationDialog)
        except Exception as e:
            logger.warning(f"Mosaic calibration dialog unavailable: {e}")
            return
        dlg = MosaicCalibrationDialog(
            self.controller, self._camera_manager, cam_idx,
            safe_z=getattr(self, "_safe_z", None),
            align_key=self._ploc_camera_objective_key(),
            store=self._ploc_mosaic_align_store(),
            settings=getattr(self, "_mosaic_settings", None) or {},
            center_um=self._ploc_calibration_center(),
            frame_size=(fw, fh),
            um_per_px_camera=self._ploc_microscope_um_per_px(fw, um0),
            parent=self)
        dlg.exec()
        # Persist the grid size the operator settled on, and let the calibration
        # mosaic push its tuned settings into the full-mosaic builder.
        try:
            cols, rows = dlg.grid_values()
            self._mosaic_settings["cal_cols"] = cols
            self._mosaic_settings["cal_rows"] = rows
            applied = dlg.applied_settings()
            if applied:
                self._mosaic_settings.update(applied)
            if self.settings is not None:
                self.settings.set_section("mosaic_scan", self._mosaic_settings)
                self.settings.save()
        except Exception as e:
            logger.debug(f"Cal grid persist skipped: {e}")
        # Show the calibration composite on the plate map so the operator can
        # manual-align it there (works the same for full + calibration mosaics).
        try:
            comp, ext = dlg.result_overlay()
            if comp is not None and ext is not None:
                self._ploc_show_mosaic_mode()
                self._ploc_set_overlay_image(comp, ext)
        except Exception as e:
            logger.debug(f"Cal overlay handoff skipped: {e}")

    def _ploc_cancel_mosaic_scan(self) -> None:
        """Abort the mosaic scan; keep whatever was stitched so far."""
        self._ploc_mosaic_running = False
        self._ploc_scan_bounds_override = None
        self._ploc_scan_subwell_parent = None
        self._ploc_scan_well_is_rosette = False
        worker = getattr(self, "_ploc_mosaic_worker", None)
        if worker is not None:
            # Drop any in-flight (queued) signals so a late tile/finished from
            # the worker can't re-apply results after we've torn the scan down.
            for sig in (worker.tile, worker.progress,
                        worker.finished_ok, worker.failed):
                try:
                    sig.disconnect()
                except Exception:
                    pass
            try:
                worker.stop()
                worker.wait(4000)
            except Exception:
                pass
        self._ploc_mosaic_worker = None
        self._ploc_mosaic_builder = None
        self._ploc_mosaic_cleanup_ui()
        self._ploc_confirm_label.setText("Mosaic scan cancelled.")
        logger.info("PlateLocation mosaic scan cancelled")

    def _ploc_detect_and_fit_from_mosaic(self, mosaic, extent, scale) -> int:
        """Detect all wells on the stitched mosaic and fit the warp.

        Used directly when detection runs on the GUI thread (and by tests);
        the worker path detects off-thread and calls
        :meth:`_ploc_fit_from_mosaic_detections` with the pixel circles.
        Returns the number of matched wells.
        """
        if mosaic is None or extent is None or not scale:
            return 0
        well_d_um, pitch_um = self._ploc_mosaic_metrics()
        expected_d_px = well_d_um * scale
        min_dist_px = max(1.0, pitch_um * scale * 0.7)
        try:
            from SupportClasses.VisionDetector import WellDetector
        except Exception as e:
            logger.warning(f"Mosaic well detect failed: {e}")
            return 0
        # Primary: robust filled-disc blob detection (color/illumination/clip
        # agnostic) — far more reliable than HoughCircles on a stitched mosaic.
        # When the plate grid is known, keep only grid-consistent blobs (rejects
        # spurious detections). Fall back to Hough if it yields too few.
        dets = []
        try:
            filled = WellDetector.detect_filled_wells(mosaic)
            rows = int(getattr(self._plate, "rows", 0) or 0)
            cols = int(getattr(self._plate, "cols", 0) or 0)
            if filled and rows and cols and len(filled) >= 4:
                centers = [(d.center_px[0], d.center_px[1]) for d in filled]
                _aff, assign = WellDetector.fit_well_grid(centers, rows, cols)
                if assign:
                    filled = [filled[i] for i in sorted(set(assign.values()))]
            dets = filled
        except Exception as e:
            logger.debug(f"Filled-well detect skipped: {e}")
        if len(dets) < 3:
            try:
                dets = WellDetector.detect_wells(
                    mosaic, expected_d_px, min_dist_px=min_dist_px)
            except Exception as e:
                logger.warning(f"Mosaic well detect failed: {e}")
                return 0
        det_pixels = [(float(d.center_px[0]), float(d.center_px[1]),
                       float(d.radius_px)) for d in dets]
        return self._ploc_fit_from_mosaic_detections(det_pixels, extent, scale)

    def _ploc_mosaic_world_shift(self) -> tuple[float, float]:
        """The global-registration / manual-align shift (µm) that the live
        mosaic builder folds into ``canvas_extent_um``.

        That shift is a DISPLAY-OVERLAY correction only — the composite *pixels*
        are placed at the raw stage-encoder positions
        (``MosaicBuilder._canvas_origin_um``), NOT at the shifted extent. So when
        we back-project a detected well to the stage µm we will command the stage
        to, we MUST remove this shift, or every well's target XY is translated by
        it (a stale ``manual_align`` nudge of (-3170, -5135) µm once shifted the
        whole plate calibration by several fields of view). Returns (0, 0) when
        no builder is live (GUI/test direct calls pass an unshifted extent)."""
        b = getattr(self, "_ploc_mosaic_builder", None)
        s = getattr(b, "_global_shift_um", None) if b is not None else None
        try:
            return (float(s[0]), float(s[1]))
        except (TypeError, ValueError, IndexError):
            return (0.0, 0.0)

    def _ploc_fit_from_mosaic_detections(self, det_pixels, extent,
                                         scale) -> int:
        """Map mosaic-pixel circle detections → absolute stage µm, match each
        to the nearest predicted well within tolerance, and route the matched
        pairs through ``_ploc_feed_affine`` (same warp path as the manual fit).

        ``det_pixels`` = list of (px, py, radius_px) on the final mosaic.
        Returns the number of matched wells.
        """
        if not det_pixels or extent is None or not scale:
            return 0
        positions = self._calibrated_positions or self._predicted_positions
        if not positions:
            return 0
        well_d_um, pitch_um = self._ploc_mosaic_metrics()
        # Back-project into the TRUSTED-STAGE frame the tiles were placed in:
        # the display/registration shift folded into ``extent`` must be removed
        # so the wells we drive to match where the stage actually was (see
        # ``_ploc_mosaic_world_shift``). Exact inverse of the placement in
        # ``MosaicBuilder._blend_tile_to_composite`` (which uses the unshifted
        # ``_canvas_origin_um``).
        gx, gy = self._ploc_mosaic_world_shift()
        ox = float(extent[0]) - gx
        oy = float(extent[1]) - gy
        det_pts = [(ox + px / scale, oy + py / scale)
                   for (px, py, _r) in det_pixels]
        tol = max(well_d_um * 0.6, pitch_um * 0.4)
        results: dict[str, tuple[float, float]] = {}
        used: set[int] = set()
        for name, (px_um, py_um) in positions.items():
            best, best_d = None, tol
            for i, (dx, dy) in enumerate(det_pts):
                if i in used:
                    continue
                dist = math.hypot(dx - px_um, dy - py_um)
                if dist < best_d:
                    best_d = dist
                    best = i
            if best is not None:
                used.add(best)
                results[name] = det_pts[best]

        if not results:
            logger.warning(
                f"Mosaic detect: {len(det_pixels)} circles but none matched a "
                f"predicted well (tol={tol:.0f} µm)")
            return 0
        if len(results) < 3:
            logger.warning(
                f"Mosaic detect: only {len(results)} wells matched — fit may "
                f"be weak (need ≥3 for affine).")
        self._ploc_well_results = dict(results)
        self._reference_markers.update(results)
        try:
            self._ploc_feed_affine(results)
        except Exception as e:
            logger.warning(f"Mosaic warp fit failed: {e}")
        return len(results)

    # ── v7.5.x: re-derive wells from the saved mosaic (ground truth) ──

    def _ploc_rederive_wrapper(self) -> None:
        """Button slot: re-derive wells from the saved mosaic, surfacing any
        error (a bare Qt slot swallows exceptions → a silent no-op)."""
        try:
            self._ploc_rederive_from_saved_mosaic()
        except Exception as e:
            logger.exception("Re-derive from saved mosaic failed")
            try:
                QMessageBox.warning(
                    self, "Re-derive wells",
                    f"Could not re-derive wells from the saved mosaic:\n{e}")
            except Exception:
                pass

    def _ploc_rederive_from_saved_mosaic(self) -> None:
        """Re-derive EVERY well's position from the saved mosaic's ground-truth
        detected centres, RELABELLED for the current plate orientation
        (``controller.plate_axis_sign()``), and store them DIRECTLY as the
        calibration — discarding any stale taught/warp/affine state.

        The mosaic was captured at trusted absolute stage positions, so the
        detected centres are ground truth; only the NAME→position binding
        changes with the orientation. Software-only — commands NO stage motion.
        """
        from SupportClasses import MosaicWellRemap

        if self._plate is None:
            self._ploc_set_status("No plate selected.", "yellow")
            return
        if self.controller is None or not hasattr(
                self.controller, "plate_axis_sign"):
            self._ploc_set_status(
                "No controller — cannot read the plate orientation.", "yellow")
            return
        sign = self._plate_axis_sign()
        rows = int(getattr(self._plate, "rows", 0) or 0)
        cols = int(getattr(self._plate, "cols", 0) or 0)
        want = rows * cols if (rows and cols) else 0

        # ── Gather ground-truth detected centres (orientation-INDEPENDENT) ──
        # Source A: the already-detected centres held in the live calibration
        # (the complete, validated set from the last mosaic map). Captured NOW,
        # before we clear anything.
        candidates: list[list] = []
        for src in (self._reference_markers,
                    getattr(self, "_ploc_well_results", None)):
            if src:
                candidates.append([(float(x), float(y))
                                   for (x, y) in src.values()])
        # Source B: re-detect the saved mosaic PNG (most "from the mosaic").
        try:
            store = self._ploc_mosaic_store()
            key = self._ploc_plate_key()
            if store is not None and key is not None and store.has(key):
                img = store.load_image(key)
                extent = store.get_extent_um(key)
                meta = store.get_meta(key) or {}
                scale = float(meta.get("mosaic_scale", 0) or 0)
                if img is not None and extent is not None and scale:
                    det = MosaicWellRemap.detect_raw_positions(
                        img, extent, scale, self._plate)
                    if det:
                        candidates.append(det)
        except Exception as e:
            logger.debug(f"Re-derive: saved-mosaic re-detect skipped: {e}")

        if not candidates:
            self._ploc_set_status(
                "No saved mosaic and no detected wells to re-derive from. "
                "Run a Mosaic scan or Map wells first.", "yellow")
            return
        # Prefer the most complete source (full plate beats a partial set).
        positions = max(candidates, key=len)

        results = MosaicWellRemap.label_positions(positions, self._plate, sign)
        if not results:
            self._ploc_set_status(
                "Re-derive failed: no wells matched the plate grid.", "yellow")
            return

        # ── Clear the stale plate calibration (mirror _on_plate_changed) ──
        # Kills the previous taught_a1/corner/third + the bogus warp/affine so
        # nothing re-introduces the old orientation.
        self._taught_a1 = None
        self._taught_corner = None
        if hasattr(self, "_taught_third"):
            self._taught_third = None
        self._third_well = None
        self._scale = 1.0
        self._rotation = 0.0
        self._predicted_positions = None
        self._three_well_calibration = None
        self._plate_warp = None
        self._xy_teach_points.clear()

        # ── Store the re-derived positions DIRECTLY (no warp) ──
        self._calibrated_positions = dict(results)
        self._ploc_well_results = dict(results)
        self._reference_markers = dict(results)
        # The whole map was rebuilt — single-well anchors' map frame is stale.
        try:
            self._ploc_clear_well_anchors()
        except Exception:
            pass
        # A1 is a real measured centre — set it so _has_plate_calibration() and
        # the geometric prediction anchor agree with the explicit dict.
        if "A1" in results:
            self._taught_a1 = results["A1"]
        if hasattr(self, "_cal_plate_view") and self._cal_plate_view is not None:
            self._cal_plate_view.set_calibrated_positions(self._calibrated_positions)
        # Refresh A1 label if present.
        if hasattr(self, "lbl_a1") and "A1" in results:
            ax = results["A1"][0] - self.controller.zero_position.get("x", 0)
            ay = results["A1"][1] - self.controller.zero_position.get("y", 0)
            self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) µm")
            self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

        # v7.5.x: keep the mapping stored WITH the mosaic in sync.
        try:
            _ws = self._ploc_mosaic_store()
            _wk = self._ploc_plate_key()
            if (_ws is not None and hasattr(_ws, "set_wells")
                    and _ws.has(_wk)):
                _ws.set_wells(_wk, results)
        except Exception as e:
            logger.debug(f"Re-derive: store-with-mosaic skipped: {e}")
        self._save_calibration()
        self._emit_calibration_data_changed()
        a1 = results.get("A1")
        a1s = f"A1=({a1[0]:,.0f}, {a1[1]:,.0f}) µm" if a1 else ""
        self._ploc_set_status(
            f"✅ Re-derived {len(results)}"
            + (f"/{want}" if want else "")
            + f" wells from the saved mosaic (ground truth). {a1s} "
            "Previous calibration discarded.", "green")
        logger.info(
            "Re-derived %d wells from saved mosaic (sign=%s); %s",
            len(results), sign, a1s)

    def _ploc_set_status(self, text: str, color: str = "subtext0") -> None:
        """Best-effort status line for Plate-Location actions (label varies by
        build; never raises)."""
        for attr in ("_ploc_status", "_lbl_scan_progress",
                     "ctx_lbl_cal_status"):
            lbl = getattr(self, attr, None)
            if lbl is not None and hasattr(lbl, "setText"):
                try:
                    lbl.setText(text)
                    lbl.setStyleSheet(
                        f"color: {COLORS.get(color, color)}; font-size: 9pt;")
                    return
                except Exception:
                    continue
        logger.info("[Plate Location] %s", text)

    # ── Per-well rosette mosaic ─────────────────────────────────────

    def _ploc_rosette_parent_wells(self) -> list:
        """Parent wells that contain flattened rosette sub-wells."""
        if self._plate is None:
            return []
        parents: list = []
        try:
            for w in self._plate.get_all_wells():
                if getattr(w, "is_subwell", False) and getattr(
                        w, "parent_well", None):
                    if w.parent_well not in parents:
                        parents.append(w.parent_well)
        except Exception:
            return []
        return parents

    def _ploc_subwell_scan_bounds(self, parent):
        """Absolute-µm scan bounds covering one rosette well's sub-wells
        (bbox of their calibrated/predicted centres + radius + margin)."""
        pos = self._calibrated_positions or self._predicted_positions or {}
        subs = [w for w in self._plate.get_all_wells()
                if getattr(w, "is_subwell", False)
                and getattr(w, "parent_well", None) == parent]
        pts = [pos[w.name] for w in subs if w.name in pos]
        if not pts:
            return None
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        rad_um = max((float(w.diameter) for w in subs), default=2.0) * 1000.0 / 2.0
        margin = rad_um + 1500.0
        return (min(xs) - margin, min(ys) - margin,
                max(xs) + margin, max(ys) + margin)

    def _ploc_well_scan_bounds(self, well_name):
        """Absolute-µm scan bounds covering ONE plain well (its calibrated /
        predicted centre ± radius + margin). The positions are already
        orientation-correct stage µm (plate_axis_sign baked in at prediction),
        same as the rosette bounds path."""
        pos = self._calibrated_positions or self._predicted_positions or {}
        if well_name not in pos:
            return None
        cx, cy = pos[well_name]
        w = None
        try:
            w = next((w for w in self._plate.get_all_wells()
                      if w.name == well_name), None)
        except Exception:
            w = None
        rad_um = (float(getattr(w, "diameter", 0.0) or 0.0) * 1000.0 / 2.0
                  if w is not None else 0.0)
        if rad_um <= 0:
            rad_um = 2000.0
        margin = rad_um + 1500.0
        return (cx - margin, cy - margin, cx + margin, cy + margin)

    def _ploc_scan_well(self) -> None:
        """Mosaic-scan a single well at high resolution. A ROSETTE well then
        maps its sub-wells (place the pattern centre → refine); a PLAIN well
        picks its centre. Reuses the full mosaic-scan machinery (validation,
        Safe-Z gate, retract-safe raster) with the well's bounds. Each picked
        centre also records an anchor for "Re-register from scanned wells"."""
        if self._ploc_running or getattr(self, "_ploc_mosaic_running", False):
            QMessageBox.information(
                self, "Scan well",
                "Another operation is already running on this tab.")
            return
        if self._plate is None:
            QMessageBox.information(self, "Scan well", "No plate loaded.")
            return
        # Well positions must exist to compute the scan bounds.
        if not (self._calibrated_positions or self._predicted_positions):
            try:
                self._compute_predicted_positions()
            except Exception:
                pass
        rosette_parents = set(self._ploc_rosette_parent_wells())
        # Offer PARENT wells (not the flattened sub-wells) + plain wells.
        names = []
        try:
            for w in self._plate.get_all_wells():
                if getattr(w, "is_subwell", False):
                    continue
                names.append(w.name)
        except Exception:
            pass
        for p in rosette_parents:       # parents are dropped at flatten time
            if p not in names:
                names.append(p)
        if not names:
            QMessageBox.information(self, "Scan well", "No wells to scan.")
            return
        # Rosette parents first (they get the richer sub-well flow), annotated.
        scanned = set()
        try:
            store = self._ploc_mosaic_store()
            if store is not None and hasattr(store, "list_well_keys"):
                scanned = set(store.list_well_keys(self._ploc_plate_key()))
        except Exception:
            scanned = set()

        def _label(n):
            tags = []
            if n in rosette_parents:
                tags.append("rosette")
            if n in scanned:
                tags.append("scanned")
            return f"{n} ({', '.join(tags)})" if tags else n

        ordered = ([n for n in names if n in rosette_parents]
                   + [n for n in names if n not in rosette_parents])
        items = [_label(n) for n in ordered]
        from PySide6.QtWidgets import QInputDialog
        picked, ok = QInputDialog.getItem(
            self, "Scan well", "Well to scan (single-well mosaic):",
            items, 0, False)
        if not ok or not picked:
            return
        name = ordered[items.index(picked)]
        self._ploc_scan_single_well(name, name in rosette_parents)

    def _ploc_scan_single_well(self, name: str, is_rosette: bool) -> None:
        """Kick off a single-well mosaic scan for ``name`` (bounds override +
        finish-routing flags). Shared by the Advanced "Scan well…" picker and
        the Rosettes tab."""
        if self._ploc_running or getattr(self, "_ploc_mosaic_running", False):
            QMessageBox.information(
                self, "Scan well",
                "Another operation is already running on this tab.")
            return
        if not (self._calibrated_positions or self._predicted_positions):
            try:
                self._compute_predicted_positions()
            except Exception:
                pass
        bounds = (self._ploc_subwell_scan_bounds(name) if is_rosette
                  else self._ploc_well_scan_bounds(name))
        if bounds is None:
            QMessageBox.warning(
                self, "Scan well",
                f"No position known for {name} — calibrate the plate "
                f"(or run a full mosaic + Map wells) first.")
            return
        self._ploc_scan_bounds_override = bounds
        self._ploc_scan_subwell_parent = name
        self._ploc_scan_well_is_rosette = bool(is_rosette)
        # Reuse the standard scan (validation, worker, retract-safe raster).
        self._ploc_start_mosaic_scan()

    def _ploc_plate_flip_180(self) -> bool:
        """Plate-orientation flag for presenting mosaics in the plate frame
        (A1 top-left). Prefers the live controller setting; falls back to the
        display constant (same fallback as the mosaic preview)."""
        try:
            fn = getattr(self.controller, "plate_flip_180", None)
            if callable(fn):
                return bool(fn())
        except Exception:
            pass
        try:
            from gui.widgets.jog_workspace_view import JogWorkspaceView
            return bool(getattr(JogWorkspaceView, "_FLIP_DISPLAY_180", False))
        except Exception:
            return False

    def _ploc_apply_single_well_mosaic(self, well, composite, extent, scale,
                                       frames, shift_um=(0.0, 0.0)) -> None:
        """Persist a single-well mosaic under its own key
        (``"{plate_key}#{well}"``) — never touches the plate mosaic/overlay.
        ``shift_um`` records the registration shift baked into the extent so a
        later mapping re-open can recover the trusted back-projection frame."""
        store = self._ploc_mosaic_store()
        if store is None or composite is None or extent is None:
            return
        try:
            store.save(
                f"{self._ploc_plate_key()}#{well}", composite, extent,
                um_per_px=getattr(self, "_ploc_mosaic_um_per_px", 0.0),
                mosaic_scale=scale, frames=frames, shift_um=shift_um)
        except Exception as e:
            logger.warning(f"Single-well mosaic persist failed: {e}")

    def _ploc_open_single_well_mapping(self, parent, composite, extent, scale,
                                       is_rosette=True,
                                       shift_um=(0.0, 0.0),
                                       um_per_px=None) -> None:
        """Map one well's per-well mosaic: a ROSETTE well places the sub-well
        pattern centre then refines each sub-well; a PLAIN well picks its
        centre. Merges the measured centres (sub-wells AND the parent) into the
        calibration and records a re-registration anchor for the parent.

        ``shift_um`` = the registration shift baked into ``extent``; the
        mapping dialog back-projects pixels in the TRUSTED stage frame, so it
        receives ``extent − shift`` (the same removal the detection path does
        via ``_ploc_mosaic_world_shift`` — a shifted extent would offset every
        picked centre by the stored alignment shift)."""
        self._ploc_confirm_label.setVisible(True)
        if composite is None or extent is None or not scale:
            self._ploc_confirm_label.setText(
                f"Well {parent}: scan produced no usable image.")
            return
        try:
            gx, gy = float(shift_um[0]), float(shift_um[1])
        except (TypeError, ValueError, IndexError):
            gx = gy = 0.0
        extent = (float(extent[0]) - gx, float(extent[1]) - gy,
                  float(extent[2]) - gx, float(extent[3]) - gy)
        if is_rosette:
            subplate = _SubPlate(self._plate, parent)
            if not subplate.well_names:
                self._ploc_confirm_label.setText(
                    f"{parent} has no sub-wells to map.")
                return
        else:
            subplate = _SingleWell(self._plate, parent)
            if not subplate.well_names:
                self._ploc_confirm_label.setText(
                    f"{parent} not found in the plate.")
                return
        try:
            from gui.dialogs.mosaic_well_mapping_dialog import (
                MosaicWellMappingDialog)
        except Exception as e:
            logger.warning(f"Single-well mapping dialog unavailable: {e}")
            return
        # Predicted stage-µm for the dialog's rosette geometry (sub-well
        # offsets around the picked centre) — rotation + orientation baked in.
        pos = self._calibrated_positions or self._predicted_positions or {}
        predicted = {n: pos[n] for n in subplate.well_names if n in pos}
        # The map's belief of the PARENT centre — captured BEFORE the merge
        # below so the re-registration anchor measures the true offset.
        map_at_pick = pos.get(parent)
        if map_at_pick is None and predicted:
            # Parent centre ≈ centroid of the sub-well map positions.
            vals = list(predicted.values())
            map_at_pick = (sum(v[0] for v in vals) / len(vals),
                           sum(v[1] for v in vals) / len(vals))
        if um_per_px is None:
            um_per_px = float(
                getattr(self, "_ploc_mosaic_um_per_px", 0.0) or 0.0)
        dlg = MosaicWellMappingDialog(
            subplate, composite, extent, scale,
            um_per_px=float(um_per_px or 0.0),
            # v7.5.x: key by the active plate identity (type id), not the base
            # int format, so per-well training segregates per plate type.
            plate_key=f"{self._ploc_plate_key()}#{parent}",
            plate_flip_180=self._ploc_plate_flip_180(),
            rosette=is_rosette,
            predicted_um=predicted,
            center_label=parent,
            parent=self)
        if not dlg.exec():
            self._ploc_confirm_label.setText(
                f"Well {parent}: mapping cancelled.")
            return
        results = dlg.results()
        if not results:
            return
        # Parent centre: from the dialog if present, else the centroid of the
        # measured sub-well centres (the pattern centre).
        parent_meas = results.get(parent)
        if parent_meas is None:
            pts = list(results.values())
            parent_meas = (sum(p[0] for p in pts) / len(pts),
                           sum(p[1] for p in pts) / len(pts))
            results = {**results, parent: parent_meas}
        # Merge everything (sub-wells AND parent) into the calibration.
        self._ploc_well_results.update(results)
        self._reference_markers.update(results)
        if self._calibrated_positions is None:
            self._calibrated_positions = {}
        self._calibrated_positions.update(results)
        # Record the re-registration anchor (measured vs map-at-pick). The full
        # merged results are kept as ground-truth PINS: a later re-register
        # shifts/fits the STALE wells but must NOT move these already-measured
        # centres again (double-correction).
        if parent_meas is not None and map_at_pick is not None:
            self._ploc_well_anchors[parent] = {
                "measured": (float(parent_meas[0]), float(parent_meas[1])),
                "map": (float(map_at_pick[0]), float(map_at_pick[1])),
                "merged": {n: (float(p[0]), float(p[1]))
                           for n, p in results.items()},
            }
        self._ploc_refresh_scanned_reregister_button()
        try:
            self._emit_calibration_data_changed()
        except Exception:
            pass
        n_sub = len([n for n in results if n != parent])
        self._ploc_confirm_label.setText(
            f"Mapped {parent}"
            + (f" + {n_sub} sub-wells" if is_rosette and n_sub else "")
            + " from its mosaic — anchor recorded for re-registration.")
        logger.info(
            f"Single-well mapping: {parent} "
            f"({len(results)} centres, rosette={is_rosette})")

    def _ploc_open_well_mapping(self) -> None:
        """Manual well mapping on the stored mosaic: click the 3 corner wells →
        auto-fill the grid from the plate layout → drag to refine → confirm.
        Confirm feeds the same warp fit as the manual teach AND saves a labeled
        training sample (image + well centres) for detection R&D.
        """
        store = self._ploc_mosaic_store()
        key = self._ploc_plate_key()
        if store is None or not store.has(key):
            QMessageBox.information(
                self, "Map wells",
                "Run a Mosaic scan first — there's no stitched mosaic for this "
                "plate to map wells on.")
            return
        img = store.load_image(key)
        extent = store.get_extent_um(key)
        meta = store.get_meta(key) or {}
        scale = float(meta.get("mosaic_scale", 0.0) or 0.0)
        if img is None or extent is None or not scale:
            QMessageBox.warning(
                self, "Map wells",
                "The stored mosaic is missing its extent/scale — re-run the "
                "Mosaic scan.")
            return
        if self._plate is None:
            QMessageBox.warning(self, "Map wells", "No plate loaded.")
            return
        # Back-project in the TRUSTED stage frame: the stored extent is
        # display-registered (includes the global registration shift) while
        # the mosaic PIXELS sit at raw stage positions — remove the recorded
        # shift before handing the extent to the mapping dialog (mirrors
        # _ploc_fit_from_mosaic_detections). Legacy entries → (0, 0).
        gx, gy = (store.get_shift_um(key)
                  if hasattr(store, "get_shift_um") else (0.0, 0.0))
        extent = (float(extent[0]) - gx, float(extent[1]) - gy,
                  float(extent[2]) - gx, float(extent[3]) - gy)
        try:
            from gui.dialogs.mosaic_well_mapping_dialog import (
                MosaicWellMappingDialog)
        except Exception as e:
            logger.warning(f"Well mapping dialog unavailable: {e}")
            return
        dlg = MosaicWellMappingDialog(
            self._plate, img, extent, scale,
            um_per_px=float(meta.get("um_per_px", 0.0) or 0.0),
            plate_key=key,
            plate_flip_180=self._ploc_plate_flip_180(),
            parent=self)
        if not dlg.exec():
            return
        results = dlg.results()
        if not results:
            return
        # Same path as auto-detect / manual teach: feed the warp + mark refs.
        self._ploc_well_results = dict(results)
        self._reference_markers.update(results)
        try:
            self._ploc_feed_affine(results)
        except Exception as e:
            logger.warning(f"Well mapping warp fit failed: {e}")
        # v7.5.x: the confirmed mapping travels WITH the mosaic — store it in
        # the mosaic entry so loading this mosaic anywhere (e.g. "Load mosaic
        # from another plate…") restores the mapping without re-mapping.
        try:
            if hasattr(store, "set_wells"):
                store.set_wells(key, results)
        except Exception as e:
            logger.debug(f"Well mapping: store-with-mosaic skipped: {e}")
        # Save the as-built well map as a reusable TEMPLATE so the operator can
        # later re-register this plate from just 3 wells (no full re-scan).
        tpl_saved = self._ploc_save_plate_template(
            results, um_per_px=float(meta.get("um_per_px", 0.0) or 0.0),
            mosaic_scale=float(meta.get("mosaic_scale", 0.0) or 0.0))
        saved = dlg.saved_sample_path()
        msg = f"Mapped {len(results)} wells from the mosaic."
        if saved:
            msg += " Saved a labeled training sample for detection R&D."
        if tpl_saved:
            msg += " Saved a plate template (3-well quick re-register enabled)."
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(msg)
        self._ploc_refresh_reregister_button()
        logger.info(
            f"Well mapping: {len(results)} wells fitted; sample={saved}; "
            f"template={tpl_saved}")

    def _ploc_template_store(self):
        try:
            from SupportClasses.PlateTemplateStore import get_store
            return get_store()
        except Exception as e:
            logger.debug(f"PlateTemplateStore unavailable: {e}")
            return None

    def _ploc_save_plate_template(self, results, um_per_px=0.0,
                                  mosaic_scale=0.0) -> bool:
        """Persist the as-built well map for this plate + camera + objective."""
        store = self._ploc_template_store()
        if store is None or not results:
            return False
        try:
            key = store.save_template(
                self._ploc_plate_key(), self._ploc_camera_objective_key(), None,
                results, um_per_px=um_per_px, mosaic_scale=mosaic_scale)
            return key is not None
        except Exception as e:
            logger.warning(f"Plate template save failed: {e}")
            return False

    def _ploc_refresh_reregister_button(self) -> None:
        """Enable Quick re-register only when a template exists for the current
        plate + camera + objective."""
        btn = getattr(self, "_ploc_btn_reregister", None)
        if btn is None:
            return
        store = self._ploc_template_store()
        ok = False
        try:
            ok = bool(store and store.has(
                self._ploc_plate_key(), self._ploc_camera_objective_key(), None))
        except Exception:
            ok = False
        btn.setEnabled(ok)
        btn.setToolTip(
            "Re-register this plate from a saved mosaic template: the camera "
            "drives to 3 reference wells, auto-detects each, and fits the whole "
            "plate — no full re-scan. Needs the SAME camera + objective."
            if ok else
            "No saved plate template for this plate + camera + objective yet. "
            "Run a Mosaic scan + Map wells once to enable quick re-register.")

    # ── Manual single-point re-anchor (Quick re-register fallback) ──────
    def _ploc_toggle_reanchor(self) -> None:
        """Toggle the 2-click manual re-anchor capture on/off."""
        if getattr(self, "_ploc_reanchor_stage", None) is not None:
            self._ploc_cancel_reanchor("Re-anchor cancelled.")
        else:
            self._ploc_start_reanchor()

    def _ploc_start_reanchor(self) -> None:
        """Begin a manual single-point re-anchor (NO-TRAVEL): the operator jogs a
        recognizable plate feature into the live microscope view and clicks it
        there, then clicks the SAME feature on the plate overview. The measured
        offset E = live_actual − overview_map shifts the whole map (well centres
        AND the mosaic overlay) by one translation (persisted). The stage does
        NOT move during re-anchor — so it corrects an arbitrarily large remount
        offset that a travel-first flow could not."""
        if self._ploc_running or self._ploc_mosaic_running:
            QMessageBox.information(
                self, "Re-anchor",
                "Finish the current run / mosaic scan first.")
            return
        if (self.controller is None
                or not getattr(self.controller, "is_xy_connected", False)):
            QMessageBox.warning(self, "Re-anchor", "Connect the XY stage first.")
            return
        # No Safe-Z gate: the no-travel flow issues no motion, so there is no
        # retract/descent to guard (the operator jogs manually — the jog
        # exemption owns Z).
        cam_idx = self._ploc_microscope_cam_idx()
        if cam_idx is None or self._camera_manager is None:
            QMessageBox.warning(
                self, "Re-anchor",
                "Assign + start the Microscope camera (Hardware Setup → Cameras).")
            return
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            QMessageBox.warning(
                self, "Re-anchor",
                "Calibrate the microscope µm/pixel first.")
            return
        if not (self._calibrated_positions or self._predicted_positions):
            QMessageBox.warning(
                self, "Re-anchor",
                "No well map to re-anchor yet — calibrate or map wells first.")
            return
        self._ploc_live_cam_idx = cam_idx
        self._ploc_ensure_live_camera()
        self._ploc_reanchor_stage = "await_live"
        self._ploc_reanchor_live_abs = None
        self._ploc_btn_reanchor.setText("Cancel re-anchor")
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            "Re-anchor 1/2: jog a recognizable feature (a well, corner or mark) "
            "into the LIVE view, then click its centre in the live view.")

    def _ploc_cancel_reanchor(self, msg: str = "") -> None:
        self._ploc_reanchor_stage = None
        self._ploc_reanchor_live_abs = None
        if getattr(self, "_ploc_btn_reanchor", None) is not None:
            self._ploc_btn_reanchor.setText("Re-anchor map (1 point)…")
        try:
            self._ploc_live_view.set_overlay_vector(None, None)
        except Exception:
            pass
        if msg:
            self._ploc_confirm_label.setText(msg)

    def _ploc_reanchor_live_click(self, px_x: float, px_y: float) -> None:
        """Re-anchor step 1 (NO-TRAVEL): the operator has jogged a feature into
        the live view and clicked it. Record where that feature ACTUALLY is in
        absolute stage µm (current stage XY + the rotation-corrected pixel
        offset), then wait for the overview click. No stage motion."""
        if self._camera_manager is None:
            self._ploc_cancel_reanchor("Re-anchor aborted (no camera).")
            return
        img_w, img_h = self._ploc_live_view.image_size
        if not img_w or not img_h:
            QMessageBox.warning(self, "Re-anchor", "No live frame to click.")
            return
        dx_um, dy_um = self._camera_manager.pixel_to_stage_offset(
            self._ploc_live_cam_idx, px_x, px_y, img_w, img_h)
        xy = self.controller.get_xy_position(cached=False)
        if xy is None or xy[0] is None:
            QMessageBox.warning(self, "Re-anchor", "Stage position unavailable.")
            return
        # feature_abs = current stage position + pixel offset (both absolute µm;
        # pixel_to_stage_offset now applies the calibrated camera rotation).
        self._ploc_reanchor_live_abs = (
            float(xy[0]) + dx_um, float(xy[1]) + dy_um)
        # Save the clicked feature as the "Auto re-anchor" template: an image
        # patch around the click + its stage position + capture µm/px. The
        # auto flow later travels here and template-matches this patch.
        # Best-effort — a failure never blocks the manual re-anchor.
        try:
            self._ploc_save_reanchor_feature(px_x, px_y, img_w, img_h)
        except Exception as e:
            logger.debug(f"Re-anchor feature save skipped: {e}")
        try:
            self._ploc_live_view.set_overlay_vector(
                px_x - img_w / 2.0, px_y - img_h / 2.0, "✓")
        except Exception:
            pass
        self._ploc_reanchor_stage = "await_overview"
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            "Re-anchor 2/2: click that SAME feature on the plate overview "
            "(snap to its well, or Free-click the exact point).")

    def _ploc_save_reanchor_feature(self, px_x: float, px_y: float,
                                    img_w: int, img_h: int) -> None:
        """Capture the live-frame patch around a re-anchor click and persist it
        as this plate's auto-re-anchor feature (``ReanchorFeatureStore``). The
        stored ``stage_um`` is the PATCH-CENTRE's absolute position (what the
        auto flow's template match returns), computed with the same
        rotation-aware pixel→stage mapping as the click itself."""
        cam = None
        try:
            cam = self._camera_manager.cameras[self._ploc_live_cam_idx]
        except Exception:
            cam = None
        frame = cam.get_current_frame() if cam is not None else None
        if frame is None:
            return
        fh, fw = frame.shape[:2]
        # Click coords are in the live view's IMAGE frame; rescale in case the
        # raw frame size differs.
        sx = fw / float(img_w) if img_w else 1.0
        sy = fh / float(img_h) if img_h else 1.0
        cx = int(round(px_x * sx))
        cy = int(round(px_y * sy))
        half = max(48, int(fw * 0.125))          # patch ≈ 1/4 frame width
        x0, x1 = max(0, cx - half), min(fw, cx + half)
        y0, y1 = max(0, cy - half), min(fh, cy + half)
        if x1 - x0 < 32 or y1 - y0 < 32:
            return
        patch = frame[y0:y1, x0:x1].copy()
        # The clipped patch's centre (frame px → view px → stage offset).
        pcx_view = (x0 + x1) / 2.0 / sx
        pcy_view = (y0 + y1) / 2.0 / sy
        dxc, dyc = self._camera_manager.pixel_to_stage_offset(
            self._ploc_live_cam_idx, pcx_view, pcy_view, img_w, img_h)
        xy = self.controller.get_xy_position(cached=True)
        if xy is None or xy[0] is None:
            return
        patch_abs = (float(xy[0]) + dxc, float(xy[1]) + dyc)
        # µm/px resolved for the live frame width (resolution-aware).
        try:
            um_per_px = self._camera_manager.effective_um_per_px(
                self._ploc_live_cam_idx, fw)
        except Exception:
            um_per_px = self._camera_manager.get_um_per_px(
                self._ploc_live_cam_idx)
        from SupportClasses.ReanchorFeatureStore import get_store as _feat_store
        ok = _feat_store().save(
            self._ploc_plate_key(), patch, patch_abs,
            um_per_px=float(um_per_px or 0.0),
            camobj=self._ploc_camera_objective_key())
        if ok:
            logger.info(
                "Re-anchor feature saved for auto re-anchor "
                f"({patch.shape[1]}×{patch.shape[0]} px at "
                f"({patch_abs[0]:.0f}, {patch_abs[1]:.0f}) µm)")
            self._ploc_refresh_auto_reanchor_button()

    # ── Auto re-anchor (hands-free, from the saved feature) ────────────

    def _ploc_feature_store(self):
        try:
            from SupportClasses.ReanchorFeatureStore import get_store
            return get_store()
        except Exception as e:
            logger.debug(f"ReanchorFeatureStore unavailable: {e}")
            return None

    def _ploc_sync_advanced_menu(self) -> None:
        """Mirror the hidden legacy buttons' live text + enabled state onto
        their Advanced-menu actions (e.g. the "(N)" anchor count)."""
        for act, btn in getattr(self, "_ploc_adv_actions", []):
            try:
                act.setText(btn.text())
                act.setEnabled(btn.isEnabled())
            except Exception:
                pass

    def _ploc_refresh_auto_reanchor_button(self) -> None:
        """Enable "Auto re-anchor mosaic" only when this plate has a saved
        feature (captured by a manual re-anchor's live-view click)."""
        btn = getattr(self, "_ploc_btn_auto_reanchor", None)
        if btn is None:
            return
        store = self._ploc_feature_store()
        has = bool(store is not None and store.has(self._ploc_plate_key()))
        busy = getattr(self, "_ploc_auto_reanchor_worker", None) is not None
        btn.setEnabled(has and not busy)
        btn.setToolTip(
            "Hands-free re-anchor: travels to the saved feature's last "
            "position, finds it in the live view (template match), and shifts "
            "the whole map — mosaic + wells — by the measured offset."
            if has else
            "No saved feature for this plate yet — run “Re-anchor mosaic” "
            "once; its live-view click saves the feature automatically.")

    def _ploc_auto_reanchor(self) -> None:
        """Hands-free re-anchor from the saved feature patch (worker thread)."""
        if self._ploc_running or getattr(self, "_ploc_mosaic_running", False):
            QMessageBox.information(
                self, "Auto re-anchor",
                "Another operation is already running on this tab.")
            return
        if getattr(self, "_ploc_auto_reanchor_worker", None) is not None:
            return
        if (self.controller is None
                or not getattr(self.controller, "is_xy_connected", False)):
            QMessageBox.warning(self, "Auto re-anchor",
                                "Connect the XY stage first.")
            return
        # Safe-Z gate — the travel retracts first (ZDIR=-1 crash guard).
        if (bool(getattr(self.controller, "is_zp_connected", False))
                and getattr(self, "_safe_z", None) is None):
            QMessageBox.warning(
                self, "Auto re-anchor",
                "Set the Safe / Move Z first (Needle Offset Calibration tab).")
            return
        cam_idx = self._ploc_microscope_cam_idx()
        if cam_idx is None or self._camera_manager is None:
            QMessageBox.warning(
                self, "Auto re-anchor",
                "Assign + start the Microscope camera (Hardware Setup → "
                "Cameras).")
            return
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            QMessageBox.warning(self, "Auto re-anchor",
                                "Calibrate the microscope µm/pixel first.")
            return
        if not (self._calibrated_positions or self._predicted_positions):
            QMessageBox.warning(
                self, "Auto re-anchor",
                "No well map to re-anchor yet — calibrate or map wells first.")
            return
        store = self._ploc_feature_store()
        rec = store.get(self._ploc_plate_key()) if store is not None else None
        patch = (store.load_patch(self._ploc_plate_key())
                 if store is not None else None)
        if not rec or patch is None or not rec.get("stage_um"):
            QMessageBox.information(
                self, "Auto re-anchor",
                "No saved feature for this plate — run “Re-anchor mosaic” "
                "once; its live-view click saves the feature automatically.")
            return
        camobj = self._ploc_camera_objective_key()
        if rec.get("camobj") and camobj and rec["camobj"] != camobj:
            logger.warning(
                f"Auto re-anchor: feature saved with '{rec['camobj']}', "
                f"current is '{camobj}' — template rescaled by µm/px; the "
                f"match-confidence gate decides.")
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except Exception:
            QMessageBox.warning(self, "Auto re-anchor",
                                "Microscope camera not available.")
            return
        self._ploc_live_cam_idx = cam_idx
        self._ploc_ensure_live_camera()
        worker = _AutoReanchorWorker(
            self.controller, cam, self._camera_manager, cam_idx,
            rec["stage_um"], getattr(self, "_safe_z", None), patch,
            rec.get("um_per_px", 0.0))
        worker.finished_ok.connect(self._ploc_on_auto_reanchor_done)
        worker.failed.connect(self._ploc_on_auto_reanchor_failed)
        self._ploc_auto_reanchor_worker = worker
        self._ploc_refresh_auto_reanchor_button()
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            "Auto re-anchor: travelling to the saved feature…")
        worker.start()

    def _ploc_on_auto_reanchor_done(self, ax: float, ay: float,
                                    conf: float) -> None:
        """Worker found the feature → apply E = actual − stored (GUI thread)."""
        self._ploc_auto_reanchor_worker = None
        store = self._ploc_feature_store()
        rec = store.get(self._ploc_plate_key()) if store is not None else None
        if not rec or not rec.get("stage_um"):
            self._ploc_refresh_auto_reanchor_button()
            return
        ex = float(ax) - float(rec["stage_um"][0])
        ey = float(ay) - float(rec["stage_um"][1])
        if math.hypot(ex, ey) > 30000.0:
            if QMessageBox.question(
                    self, "Auto re-anchor",
                    f"The measured correction is large: "
                    f"({ex:.0f}, {ey:.0f}) µm. Apply it anyway?",
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
            ) != QMessageBox.StandardButton.Yes:
                self._ploc_refresh_auto_reanchor_button()
                self._ploc_confirm_label.setText("Auto re-anchor cancelled.")
                return
        n = self._ploc_apply_global_translation(ex, ey)
        self._ploc_refresh_auto_reanchor_button()
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            f"Auto re-anchored {n} wells + mosaic by ({ex:.0f}, {ey:.0f}) µm "
            f"(match {conf:.0%}).")
        logger.info(
            f"Auto re-anchor: shifted {n} wells by ({ex:.1f}, {ey:.1f}) µm, "
            f"confidence {conf:.2f}")

    def _ploc_on_auto_reanchor_failed(self, msg: str) -> None:
        self._ploc_auto_reanchor_worker = None
        self._ploc_refresh_auto_reanchor_button()
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(f"Auto re-anchor failed: {msg}")
        logger.warning(f"Auto re-anchor failed: {msg}")

    def _ploc_reanchor_overview(self, gx: float, gy: float, label: str) -> None:
        """Re-anchor step 2 (NO-TRAVEL): the operator clicked the same feature on
        the overview → its MAP position (absolute µm). Correction E = live_actual
        − map. Shift the whole map (well centres + mosaic overlay) by E. No stage
        motion."""
        live = getattr(self, "_ploc_reanchor_live_abs", None)
        if live is None:
            self._ploc_cancel_reanchor("Re-anchor aborted (click the live view "
                                       "first).")
            return
        map_abs = (float(gx), float(gy))
        ex = live[0] - map_abs[0]
        ey = live[1] - map_abs[1]
        # Sanity: a single-point translation should be modest; a wild value is a
        # mis-click. Confirm before committing a very large shift.
        if math.hypot(ex, ey) > 30000.0:
            if QMessageBox.question(
                    self, "Re-anchor",
                    f"The measured correction is large: "
                    f"({ex:.0f}, {ey:.0f}) µm. Apply it anyway?",
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
            ) != QMessageBox.StandardButton.Yes:
                self._ploc_cancel_reanchor("Re-anchor cancelled.")
                return
        n = self._ploc_apply_global_translation(ex, ey)
        self._ploc_cancel_reanchor()
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            f"Re-anchored {n} wells + mosaic by ({ex:.0f}, {ey:.0f}) µm — saved.")
        logger.info(
            f"Manual re-anchor ({label}): shifted {n} wells + mosaic by "
            f"({ex:.1f}, {ey:.1f}) µm")

    def _ploc_apply_global_translation(self, ex: float, ey: float,
                                       pin: dict | None = None) -> int:
        """Shift the ENTIRE well map by (ex, ey) µm and persist it.

        The correction is folded into the warp as predicted→(current + E) pairs
        so it survives a restart (``_load_calibration`` re-applies the warp to
        the predicted grid). ``_taught_a1`` / the predicted grid are left
        untouched — the warp carries the whole correction. Returns the well
        count shifted.

        ``pin`` (v7.5.x): well → (x, y) absolute µm entries that are MEASURED
        ground truth (e.g. centres picked on a single-well mosaic, already
        merged into the map at pick time). They are exempt from the shift —
        pinned verbatim AFTER the translation, and included in the warp rebuild
        so they survive a reload. Prevents double-correcting an anchor well
        whose map position already equals its measured value."""
        predicted = self._predicted_positions
        if not predicted:
            try:
                self._compute_predicted_positions()
            except Exception:
                pass
            predicted = self._predicted_positions
        if not predicted:
            return 0
        base = self._calibrated_positions or predicted
        corrected = {
            name: (base.get(name, predicted[name])[0] + ex,
                   base.get(name, predicted[name])[1] + ey)
            for name in predicted
        }
        if pin:
            corrected.update({
                n: (float(p[0]), float(p[1])) for n, p in pin.items()})
        # Rebuild the warp predicted→corrected (re-applied to the predicted grid
        # on reload — same contract as _manual_fit_xy / the load path).
        try:
            from SupportClasses.PlateWarpCalibrator import PlateWarpCalibrator
            warp = PlateWarpCalibrator()
            for name, (px, py) in predicted.items():
                cx, cy = corrected[name]
                warp.add_point(px, py, cx, cy)
            if warp.n_points >= 2:
                warp.solve()
                self._plate_warp = warp
                self._three_well_calibration = None  # warp supersedes
        except Exception as e:
            logger.warning(f"Re-anchor: warp rebuild failed: {e}")
        self._calibrated_positions = corrected
        # Shift the persisted reference markers + in-memory teach points so they
        # stay on the corrected wells (markers are restored verbatim on load).
        if self._reference_markers:
            self._reference_markers = {
                n: (x + ex, y + ey)
                for n, (x, y) in self._reference_markers.items()}
            if pin:
                self._reference_markers.update({
                    n: (float(p[0]), float(p[1]))
                    for n, p in pin.items()
                    if n in self._reference_markers})
        if getattr(self, "_xy_teach_points", None):
            self._xy_teach_points = {
                n: (x + ex, y + ey)
                for n, (x, y) in self._xy_teach_points.items()}
        # Shift the mosaic overlay by the SAME E so the image and the well
        # centres move as one unit (persisted BEFORE the emit below → the Jog
        # page re-reads the shifted extent from MosaicStore). No mosaic loaded ⇒
        # graceful no-op; the wells still shift.
        try:
            self._ploc_shift_mosaic_by(ex, ey)
        except Exception as e:
            logger.warning(f"Re-anchor: mosaic shift failed: {e}")
        try:
            self._refresh_ploc_view()
        except Exception:
            pass
        try:
            self._emit_calibration_data_changed()
            self._save_calibration()
        except Exception as e:
            logger.warning(f"Re-anchor: save failed: {e}")
        # Any map translation invalidates the single-well anchors' recorded
        # map frame (except the pins, which are absolute ground truth) — drop
        # them so a later "Re-register from scanned wells" can't over-correct.
        try:
            self._ploc_clear_well_anchors()
        except Exception:
            pass
        # The auto-re-anchor feature position lives in the MAP frame — move it
        # with the map so "Auto re-anchor mosaic" always travels to where the
        # map currently believes the feature to be.
        try:
            from SupportClasses.ReanchorFeatureStore import (
                get_store as _feat_store)
            rec = _feat_store().get(self._ploc_plate_key())
            if rec and rec.get("stage_um"):
                fx, fy = rec["stage_um"][0], rec["stage_um"][1]
                _feat_store().set_stage_um(
                    self._ploc_plate_key(), fx + ex, fy + ey)
        except Exception as e:
            logger.debug(f"Re-anchor feature position shift skipped: {e}")
        return len(corrected)

    def _ploc_quick_reregister(self) -> None:
        """Scan-once re-registration: drive to the template's 3 reference wells,
        auto-detect each well centre, fit the template→measured transform, and
        apply it to ALL template wells. Same camera + objective required."""
        store = self._ploc_template_store()
        plate_key = self._ploc_plate_key()
        camobj = self._ploc_camera_objective_key()
        if store is None or not store.has(plate_key, camobj, None):
            QMessageBox.information(
                self, "Quick re-register",
                "No saved plate template for this plate + camera + objective "
                "yet — run a Mosaic scan, then “Map wells…” once to create one "
                "(automatic re-register needs it).\n\nTo fix a mis-registered "
                "map right now without a template, use “Re-anchor map "
                "(1 point)…”.")
            return
        if (self.controller is None
                or not getattr(self.controller, "is_xy_connected", False)):
            QMessageBox.warning(
                self, "Quick re-register", "Connect the XY stage first.")
            return
        # Safe-Z gate — driving between wells must retract (ZDIR=-1 crash guard).
        if (bool(getattr(self.controller, "is_zp_connected", False))
                and getattr(self, "_safe_z", None) is None):
            QMessageBox.warning(
                self, "Quick re-register",
                "Set the Safe / Move Z first (Needle Offset Calibration tab).")
            return
        cam_idx = self._ploc_microscope_cam_idx()
        if cam_idx is None or self._camera_manager is None:
            QMessageBox.warning(
                self, "Quick re-register",
                "Assign + start the Microscope camera (Hardware Setup → Cameras).")
            return
        cam = self._camera_manager.cameras[cam_idx]
        if not getattr(cam, "is_running", False):
            try:
                self._camera_manager.start(cam_idx)
            except Exception:
                pass
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            QMessageBox.warning(
                self, "Quick re-register",
                "Calibrate the microscope µm/pixel first.")
            return
        template_wells = store.get_wells(plate_key, camobj, None)
        ref_wells = [w for w in store.get_ref_wells(plate_key, camobj, None)
                     if w in template_wells][:3]
        if len(ref_wells) < 3:
            QMessageBox.warning(
                self, "Quick re-register",
                "Template has fewer than 3 reference wells — re-run Map wells.")
            return
        try:
            from SupportClasses.VisionDetector import WellDetector
            from SupportClasses.PlateWarpCalibrator import register_from_template
        except Exception as e:
            logger.warning(f"Quick re-register unavailable: {e}")
            return

        self._ploc_confirm_label.setVisible(True)
        measured: dict = {}
        for idx, name in enumerate(ref_wells):
            tx, ty = template_wells[name]
            self._ploc_confirm_label.setText(
                f"Quick re-register: driving to {name} ({idx + 1}/3)…")
            self._ploc_confirm_label.repaint()
            try:
                self._ploc_safe_goto(tx, ty)
            except Exception as e:
                logger.warning(f"re-register goto {name} failed: {e}")
            frame = None
            try:
                frame = cam.capture_fresh_frame(discard_n_frames=3, settle_ms=300)
            except Exception:
                try:
                    frame = cam.get_current_frame()
                except Exception:
                    frame = None
            if frame is None:
                QMessageBox.warning(
                    self, "Quick re-register", f"No camera frame at {name}.")
                return
            dets = WellDetector.detect_filled_wells(frame)
            if not dets:
                QMessageBox.warning(
                    self, "Quick re-register",
                    f"Couldn't auto-detect well {name} in view (the saved "
                    f"template anchor may be too far off for the well to be in "
                    f"frame).\n\nUse “Re-anchor map (1 point)…” instead: click "
                    f"that well on the overview, then click it in the live view "
                    f"— one measurement re-anchors the whole map.")
                return
            h, w = frame.shape[:2]
            cxp, cyp = w / 2.0, h / 2.0
            best = min(dets, key=lambda d: (d.center_px[0] - cxp) ** 2
                       + (d.center_px[1] - cyp) ** 2)
            dx_um, dy_um = self._camera_manager.pixel_to_stage_offset(
                cam_idx, best.center_px[0], best.center_px[1], w, h)
            cur = self.controller.get_xy_position(cached=False)
            if cur is None or cur[0] is None:
                QMessageBox.warning(
                    self, "Quick re-register", "Stage position unavailable.")
                return
            measured[name] = (float(cur[0]) + dx_um, float(cur[1]) + dy_um)

        reg, warp = register_from_template(template_wells, measured)
        if not reg:
            QMessageBox.warning(
                self, "Quick re-register",
                "Could not fit the 3 measured wells to the template.")
            return
        # Reject an implausible fit (bad detection): same plate+camera ⇒ the
        # transform should be ~rigid (scale ≈ 1, small rotation).
        # (_warp_is_plausible returns (ok, reason) — must be unpacked; the old
        # `not self._warp_is_plausible(warp)` was always False on the tuple.)
        if warp is not None and hasattr(self, "_warp_is_plausible"):
            ok_fit, why = self._warp_is_plausible(warp)
            if not ok_fit:
                QMessageBox.warning(
                    self, "Quick re-register",
                    f"The 3-well fit looks implausible ({why}) — likely a "
                    "mis-detection. Retry, or use “Re-anchor map (1 point)…” "
                    "for a manual fix.")
                return
        self._calibrated_positions = dict(reg)
        self._ploc_well_results = dict(reg)
        self._reference_markers.update(reg)
        # The template re-registration moved the whole map — the single-well
        # anchors' recorded map frame is now stale; drop them.
        try:
            self._ploc_clear_well_anchors()
        except Exception:
            pass
        try:
            self._emit_calibration_data_changed()
        except Exception:
            pass
        rms = float(getattr(warp, "rms_error_um", 0.0) or 0.0)
        self._ploc_confirm_label.setText(
            f"Re-registered {len(reg)} wells from {len(measured)} measured "
            f"(fit RMS {rms:.1f} µm).")
        logger.info(
            f"Quick re-register: {len(reg)} wells from {len(measured)} refs, "
            f"rms={rms:.1f} µm")

    # ── Re-register from scanned single wells (v7.5.x) ─────────────

    def _ploc_refresh_scanned_reregister_button(self) -> None:
        """Track the anchor count on the button label / enabled state."""
        btn = getattr(self, "_ploc_btn_scan_reregister", None)
        if btn is None:
            return
        n = len(getattr(self, "_ploc_well_anchors", {}) or {})
        btn.setText(f"Re-register from scanned wells ({n})")
        btn.setEnabled(n >= 1)

    def _ploc_clear_well_anchors(self) -> None:
        """Drop all single-well re-registration anchors (map frame changed /
        plate switched / anchors consumed)."""
        if getattr(self, "_ploc_well_anchors", None):
            self._ploc_well_anchors = {}
        self._ploc_refresh_scanned_reregister_button()
        # Plate switches route through here — re-resolve the auto-re-anchor
        # feature availability for the (possibly new) plate key.
        try:
            self._ploc_refresh_auto_reanchor_button()
        except Exception:
            pass

    def _ploc_reregister_from_scanned(self) -> None:
        """Re-register the WHOLE plate from the well centres picked on
        single-well mosaics. 1 anchor = pure translation (mosaic + wells shift
        together); 2 = translation + rotation (similarity); 3+ = affine.
        No stage motion. Anchors reset after applying (the map moved under
        them)."""
        anchors = dict(getattr(self, "_ploc_well_anchors", {}) or {})
        if not anchors:
            QMessageBox.information(
                self, "Re-register",
                "Scan and pick at least one well first (Scan well…).")
            return
        # Ground-truth pins: every centre merged from the single-well picks is
        # already MEASURED — the re-register corrects the stale wells but must
        # not move these again (double-correction).
        pins: dict = {}
        for a in anchors.values():
            merged = a.get("merged")
            if isinstance(merged, dict):
                pins.update(merged)
        for name, a in anchors.items():        # anchor parents always pinned
            pins.setdefault(name, tuple(a["measured"]))
        if len(anchors) == 1:
            name, a = next(iter(anchors.items()))
            ex = a["measured"][0] - a["map"][0]
            ey = a["measured"][1] - a["map"][1]
            n = self._ploc_apply_global_translation(ex, ey, pin=pins)
            self._ploc_clear_well_anchors()
            self._ploc_confirm_label.setVisible(True)
            self._ploc_confirm_label.setText(
                f"Re-registered {n} wells + mosaic by ({ex:.0f}, {ey:.0f}) µm "
                f"from scanned well {name} (its measured centres pinned).")
            logger.info(
                f"Scanned-well re-register (1 anchor {name}): "
                f"shift ({ex:.1f}, {ey:.1f}) µm across {n} wells, "
                f"{len(pins)} pinned")
            return
        # 2+ anchors → similarity (2) / affine (3+) fit of the whole map.
        try:
            from SupportClasses.PlateWarpCalibrator import register_from_template
        except Exception as e:
            QMessageBox.warning(self, "Re-register",
                                f"Warp calibrator unavailable: {e}")
            return
        template = dict(self._calibrated_positions
                        or self._predicted_positions or {})
        if not template:
            QMessageBox.warning(
                self, "Re-register", "No well map to re-register.")
            return
        # Fit against the map AS IT WAS at pick time (the anchors' "map"
        # values) — with session-only anchors + reset-on-apply these equal the
        # current map, but using the recorded values is the honest pairing.
        fit_template = dict(template)
        for name, a in anchors.items():
            fit_template[name] = tuple(a["map"])
        measured = {n: tuple(a["measured"]) for n, a in anchors.items()}
        reg, warp = register_from_template(fit_template, measured)
        if not reg:
            QMessageBox.warning(
                self, "Re-register",
                "Could not fit the scanned wells — re-pick them.")
            return
        if warp is not None and hasattr(self, "_warp_is_plausible"):
            ok_fit, why = self._warp_is_plausible(warp)
            if not ok_fit:
                if QMessageBox.question(
                        self, "Re-register",
                        f"The fit looks implausible ({why}). Apply anyway?",
                        QMessageBox.StandardButton.Yes
                        | QMessageBox.StandardButton.No,
                        QMessageBox.StandardButton.No
                ) != QMessageBox.StandardButton.Yes:
                    return
        # Commit: explicit ground-truth positions supersede any stale warp —
        # same contract as the orientation-remap flow (calibrated_positions
        # persists directly, warp/affine reconstruction suppressed). The pinned
        # measured centres override the fit for the scanned wells (the warp
        # lands them ≈there already; the pin makes them exact and protects any
        # merged sub-wells from being re-warped).
        reg = dict(reg)
        reg.update({n: (float(p[0]), float(p[1])) for n, p in pins.items()})
        self._calibrated_positions = dict(reg)
        self._ploc_well_results = dict(reg)
        self._reference_markers.update(reg)
        self._plate_warp = None
        self._three_well_calibration = None
        try:
            self._emit_calibration_data_changed()
            self._save_calibration()
        except Exception as e:
            logger.warning(f"Scanned-well re-register save failed: {e}")
        # Mosaic overlay: follow the fit's TRANSLATION component only (the
        # bitmap is not rotated/scaled — a visible approximation for 2+ fits).
        shifted = False
        try:
            shifted = self._ploc_shift_mosaic_for_warp(warp)
        except Exception as e:
            logger.warning(f"Scanned-well re-register mosaic shift failed: {e}")
        # The auto-re-anchor feature position is a map-frame point — move it
        # through the same fit.
        try:
            from SupportClasses.ReanchorFeatureStore import (
                get_store as _feat_store)
            rec = _feat_store().get(self._ploc_plate_key())
            if rec and rec.get("stage_um") and warp is not None:
                nx, ny = warp.transform(rec["stage_um"][0], rec["stage_um"][1])
                _feat_store().set_stage_um(self._ploc_plate_key(), nx, ny)
        except Exception as e:
            logger.debug(f"Re-anchor feature warp move skipped: {e}")
        n_anchor = len(anchors)
        self._ploc_clear_well_anchors()
        try:
            self._refresh_ploc_view()
        except Exception:
            pass
        rms = float(getattr(warp, "rms_error_um", 0.0) or 0.0)
        mode = str(getattr(warp, "mode", "") or "fit")
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            f"Re-registered {len(reg)} wells from {n_anchor} scanned wells "
            f"({mode}, RMS {rms:.1f} µm)."
            + (" Mosaic shifted by the fit's translation only (not "
               "rotated/scaled)." if shifted else ""))
        logger.info(
            f"Scanned-well re-register ({n_anchor} anchors, {mode}): "
            f"{len(reg)} wells, rms={rms:.1f} µm")

    def _ploc_shift_mosaic_for_warp(self, warp) -> bool:
        """Follow a 2+-anchor re-registration with the mosaic overlay:
        translate it by the warp's displacement at the mosaic's centre (the
        translation component — the bitmap is NOT rotated/scaled). Returns True
        if a mosaic was shifted."""
        if warp is None:
            return False
        ext = getattr(self, "_ploc_overlay_ext", None)
        if ext is None:
            store = self._ploc_mosaic_store()
            ext = (store.get_extent_um(self._ploc_plate_key())
                   if store is not None else None)
        if not (isinstance(ext, (tuple, list)) and len(ext) >= 4):
            return False
        cx = (float(ext[0]) + float(ext[2])) / 2.0
        cy = (float(ext[1]) + float(ext[3])) / 2.0
        try:
            nx, ny = warp.transform(cx, cy)
        except Exception as e:
            logger.debug(f"warp.transform failed for mosaic shift: {e}")
            return False
        return self._ploc_shift_mosaic_by(nx - cx, ny - cy)

    # ── Live mosaic preview (updated per stitched tile) ────────────

    def _ploc_reset_mosaic_preview(self) -> None:
        lbl = getattr(self, "_ploc_mosaic_preview", None)
        if lbl is not None:
            lbl.setText("Building mosaic…")
            lbl.setPixmap(QPixmap())

    def _ploc_update_mosaic_preview(self, composite) -> None:
        lbl = getattr(self, "_ploc_mosaic_preview", None)
        if lbl is None or composite is None:
            return
        try:
            from gui.widgets.jog_workspace_view import (
                pixmap_from_bgr, JogWorkspaceView)
            pm = pixmap_from_bgr(composite)
            if pm is None or pm.isNull():
                return
            # The composite is built in stage-coordinate space (min stage at
            # top-left). On this machine the plate view flips 180° (the Prior II
            # stage has +X/+Y toward the operator's top-left), so flip the
            # preview to match — it then builds in the SAME direction the stage
            # rasters and lines up with the plate-map overlay.
            if getattr(JogWorkspaceView, "_FLIP_DISPLAY_180", False):
                from PySide6.QtGui import QTransform
                pm = pm.transformed(QTransform().rotate(180))
            target = lbl.size()
            if target.width() > 2 and target.height() > 2:
                pm = pm.scaled(
                    target, Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation)
            lbl.setPixmap(pm)
        except Exception as e:
            logger.debug(f"Mosaic preview update skipped: {e}")

    def _ploc_reset_align_sliders_silent(self) -> None:
        """Zero the manual-align sliders without re-pushing to the view (the
        view's own user-shift is reset when a fresh overlay is set)."""
        for name in ("_ploc_align_dx", "_ploc_align_dy"):
            sld = getattr(self, name, None)
            if sld is not None:
                sld.blockSignals(True)
                sld.setValue(0)
                sld.blockSignals(False)
        if getattr(self, "_ploc_align_dx_lbl", None) is not None:
            self._ploc_align_dx_lbl.setText("0")
        if getattr(self, "_ploc_align_dy_lbl", None) is not None:
            self._ploc_align_dy_lbl.setText("0")

    def _ploc_apply_mosaic(self, mosaic, extent, scale, frames) -> None:
        """Persist the mosaic + show it as the plate-view overlay."""
        store = self._ploc_mosaic_store()
        if store is not None and mosaic is not None and extent is not None:
            try:
                store.save(
                    self._ploc_plate_key(), mosaic, extent,
                    um_per_px=getattr(self, "_ploc_mosaic_um_per_px", 0.0),
                    mosaic_scale=scale, frames=frames,
                    # Record the registration shift baked into the extent so
                    # store-loaded mapping recovers the trusted frame.
                    shift_um=self._ploc_mosaic_world_shift())
            except Exception as e:
                logger.warning(f"Mosaic persist failed: {e}")
        # Default to showing the freshly captured mosaic.
        self._ploc_show_mosaic_mode()
        self._ploc_set_overlay_image(mosaic, extent)
        # Notify other pages (Jog) so they reload the overlay from the store.
        try:
            self.calibration_data_changed.emit()
        except Exception:
            pass

    def _ploc_include_well_scans_on(self) -> bool:
        """State of the "Include well scans" overlay toggle (default ON)."""
        cb = getattr(self, "_ploc_wellscan_check", None)
        return bool(cb.isChecked()) if cb is not None else True

    def _ploc_on_wellscan_toggled(self, _checked: bool = False) -> None:
        """Re-push the overlay with/without the single-well composites."""
        img = getattr(self, "_ploc_overlay_img", None)
        ext = getattr(self, "_ploc_overlay_ext", None)
        if img is not None and ext is not None:
            self._ploc_set_overlay_image(img, ext)

    def _ploc_set_overlay_image(self, mosaic, extent) -> None:
        """Push a numpy BGR mosaic + absolute-µm extent to the plate view."""
        view = getattr(self, "_ploc_plate_view", None)
        if view is None or not hasattr(view, "set_mosaic_overlay"):
            return
        # A fresh overlay zeros the view's manual-align nudge — keep the page
        # sliders in lock-step so they can't desync from what's drawn.
        self._ploc_reset_align_sliders_silent()
        if mosaic is None or extent is None:
            view.set_mosaic_overlay(None, None)
            self._ploc_overlay_img = None
            self._ploc_overlay_ext = None
            self._ploc_set_mosaic_modes_enabled(False)
            return
        # Cache the source array + extent so manual-align can re-bake the overlay
        # at a shifted extent even when there's no persisted MosaicStore entry
        # (e.g. a calibration-dialog composite handed off but not yet scanned).
        # PURITY GUARD: the cache always holds the PLAIN plate mosaic — the
        # single-well composite below is built only for the VIEW push, so
        # _ploc_shift_mosaic_by never re-persists well images baked into the
        # plate mosaic file.
        self._ploc_overlay_img = mosaic
        self._ploc_overlay_ext = tuple(float(v) for v in extent)
        display = mosaic
        if self._ploc_include_well_scans_on():
            try:
                from SupportClasses.MosaicStore import composite_with_wells
                display, _ = composite_with_wells(
                    self._ploc_mosaic_store(), self._ploc_plate_key(),
                    mosaic, self._ploc_overlay_ext)
            except Exception as e:
                logger.debug(f"Well-scan composite skipped: {e}")
                display = mosaic
        try:
            from gui.widgets.jog_workspace_view import pixmap_from_bgr
            pm = pixmap_from_bgr(display)
        except Exception:
            pm = None
        view.set_mosaic_overlay(pm, tuple(float(v) for v in extent))
        ok = pm is not None
        self._ploc_set_mosaic_modes_enabled(ok)
        # Re-apply the current plate-view mode (set_mosaic_overlay only sets the
        # image, not the visibility flags).
        if getattr(self, "_ploc_view_combo", None) is not None:
            self._ploc_on_view_mode_changed()
        elif hasattr(view, "set_mosaic_visible"):
            view.set_mosaic_visible(getattr(self, "_ploc_mosaic_show", True))

    def _ploc_load_persisted_mosaic(self) -> None:
        """Load the stored mosaic for the active plate onto the plate view."""
        store = self._ploc_mosaic_store()
        if store is None:
            return
        key = self._ploc_plate_key()
        if not store.has(key):
            self._ploc_set_overlay_image(None, None)
            return
        img = store.load_image(key)
        extent = store.get_extent_um(key)
        self._ploc_set_overlay_image(img, extent)

    # ── Load a mosaic scanned under another plate key (v7.5.x) ────────

    @staticmethod
    def _ploc_copy_mosaic_key(store, src: str, dst: str) -> int:
        """Copy the plate mosaic stored under key ``src`` to key ``dst``
        (image + extent + scale + shift + its single-well scans; the source
        entries are untouched). Returns the number of entries copied
        (plate + wells), or −1 when the plate copy failed."""
        def _copy_one(a, b) -> bool:
            img = store.load_image(a)
            ext = store.get_extent_um(a)
            meta = store.get_meta(a) or {}
            if img is None or ext is None:
                return False
            ok = bool(store.save(
                b, img, ext,
                um_per_px=float(meta.get("um_per_px", 0.0) or 0.0),
                mosaic_scale=float(meta.get("mosaic_scale", 0.0) or 0.0),
                frames=int(meta.get("frames", 0) or 0),
                shift_um=store.get_shift_um(a)))
            # The last good well mapping travels with the mosaic (save()
            # rebuilds the meta, so carry it across explicitly).
            if ok and hasattr(store, "get_wells"):
                try:
                    wells = store.get_wells(a)
                    if wells:
                        store.set_wells(b, wells)
                except Exception as exc:
                    logger.debug(f"Load mosaic: wells copy skipped: {exc}")
            return ok
        if not _copy_one(src, dst):
            return -1
        n = 1
        for well in store.list_well_keys(src):
            try:
                if _copy_one(f"{src}#{well}", f"{dst}#{well}"):
                    n += 1
            except Exception as exc:
                logger.debug(f"Load mosaic: well '{well}' copy skipped: {exc}")
        return n

    def _ploc_apply_stored_mosaic_wells(self, store, key) -> int:
        """Apply the well mapping stored WITH a mosaic (written on every
        Map-wells confirm) as this plate's calibration — positions stored
        DIRECTLY, no warp (the re-derive commit pattern). Stored names that
        don't exist on the current plate's mapping list are ignored; returns
        the number applied (0 = nothing usable)."""
        wells = {}
        try:
            if hasattr(store, "get_wells"):
                wells = store.get_wells(key) or {}
        except Exception:
            wells = {}
        if not wells or self._plate is None:
            return 0
        try:
            from gui.dialogs.mosaic_well_mapping_dialog import main_well_names
            valid = set(main_well_names(self._plate))
        except Exception:
            valid = None
        use = {str(n): (float(p[0]), float(p[1]))
               for n, p in wells.items()
               if valid is None or str(n) in valid}
        if not use:
            logger.info(
                f"Stored mosaic mapping ({len(wells)} wells) does not match "
                "the current plate layout — not applied.")
            return 0
        # Mirror the re-derive commit: clear the stale taught/warp state so
        # nothing re-introduces the old map, then store the mapping directly.
        self._taught_a1 = None
        self._taught_corner = None
        if hasattr(self, "_taught_third"):
            self._taught_third = None
        self._third_well = None
        self._scale = 1.0
        self._rotation = 0.0
        self._predicted_positions = None
        self._three_well_calibration = None
        self._plate_warp = None
        self._xy_teach_points.clear()
        self._calibrated_positions = dict(use)
        self._ploc_well_results = dict(use)
        self._reference_markers = dict(use)
        try:
            self._ploc_clear_well_anchors()
        except Exception:
            pass
        if "A1" in use:
            self._taught_a1 = use["A1"]
        view = getattr(self, "_cal_plate_view", None)
        if view is not None:
            try:
                view.set_calibrated_positions(self._calibrated_positions)
            except Exception:
                pass
        self._save_calibration()
        self._emit_calibration_data_changed()
        try:
            self._refresh_ploc_view()
        except Exception:
            pass
        return len(use)

    def _ploc_load_mosaic_from_other_plate(self) -> None:
        """Adopt a mosaic already scanned under ANOTHER plate key as this
        plate's mosaic (a COPY — the source stays). The last good well
        mapping stored with the mosaic is applied automatically when it
        matches the current plate layout. Extents are absolute stage µm, so
        after loading the operator lines the map up with the current mount
        via Re-anchor (or re-maps via Map wells…)."""
        store = self._ploc_mosaic_store()
        if store is None:
            QMessageBox.warning(self, "Load mosaic",
                                "Mosaic store unavailable.")
            return
        cur = str(self._ploc_plate_key())
        keys = sorted(k for k in store.list_plate_keys()
                      if str(k) != cur and store.has(k))
        if not keys:
            QMessageBox.information(
                self, "Load mosaic",
                "No other plate has a saved mosaic on this machine.")
            return
        items = []
        for k in keys:
            meta = store.get_meta(k) or {}
            wells = store.list_well_keys(k)
            extra = ", ".join(x for x in (
                str(meta.get("date") or ""),
                f"{meta.get('frames')} tiles" if meta.get("frames") else "",
                f"+{len(wells)} well scan(s)" if wells else "") if x)
            items.append(f"{k}  ({extra})" if extra else str(k))
        from PySide6.QtWidgets import QInputDialog
        choice, ok = QInputDialog.getItem(
            self, "Load mosaic from another plate",
            f"Copy which plate's saved mosaic to '{cur}'?\n"
            "The source stays; this plate's current mosaic is replaced.",
            items, 0, False)
        if not ok or not choice:
            return
        src = keys[items.index(choice)]
        if store.has(cur):
            resp = QMessageBox.question(
                self, "Load mosaic",
                f"'{cur}' already has a saved mosaic — replace it with the "
                f"one from '{src}'?")
            if resp != QMessageBox.StandardButton.Yes:
                return
        n = self._ploc_copy_mosaic_key(store, src, cur)
        if n < 0:
            QMessageBox.warning(
                self, "Load mosaic",
                f"Could not read the saved mosaic for '{src}'.")
            return
        # The last good well mapping stored with the mosaic loads with it.
        applied = 0
        try:
            applied = self._ploc_apply_stored_mosaic_wells(store, cur)
        except Exception as e:
            logger.warning(f"Load mosaic: stored mapping apply failed: {e}")
        try:
            self._ploc_load_persisted_mosaic()
            self._ploc_show_mosaic_mode()
        except Exception as e:
            logger.debug(f"Load mosaic: overlay refresh skipped: {e}")
        # Jog page + rosette picker pick up the new mosaic / well scans.
        try:
            self._emit_calibration_data_changed()
        except Exception:
            pass
        try:
            self._rosette_refresh_wells()
        except Exception:
            pass
        if applied:
            tail = (f"Applied its saved mapping ({applied} wells) — use "
                    "Re-anchor / Auto re-anchor to line the map up with the "
                    "current mount.")
        else:
            tail = ("It has no saved well mapping for this plate layout — "
                    "run Map wells… to map it once (the mapping is then "
                    "saved with the mosaic).")
        try:
            self._ploc_live_hint.setText(
                f"Loaded the '{src}' mosaic for '{cur}' ({n} entr"
                f"{'y' if n == 1 else 'ies'}). {tail}")
        except Exception:
            pass
        logger.info(
            f"Load mosaic: copied '{src}' → '{cur}' ({n} entries, "
            f"{applied} mapped wells applied)")

    def _ploc_on_view_mode_changed(self, _idx: int = 0) -> None:
        """Apply the chosen plate-view mode (well / mosaic / overlay) to the
        plate map."""
        combo = getattr(self, "_ploc_view_combo", None)
        mode = combo.currentData() if combo is not None else "well"
        mode = mode or "well"
        self._ploc_mosaic_show = mode in ("mosaic", "overlay")
        view = getattr(self, "_ploc_plate_view", None)
        if view is not None and hasattr(view, "set_plate_display_mode"):
            view.set_plate_display_mode(mode)

    def _ploc_set_mosaic_modes_enabled(self, ok: bool) -> None:
        """Enable/disable the mosaic-dependent view modes; revert to the
        ideal-well view when no mosaic is available."""
        combo = getattr(self, "_ploc_view_combo", None)
        if combo is None:
            return
        model = combo.model()
        for idx in (1, 2):  # "Mosaic", "Mosaic + well"
            item = model.item(idx)
            if item is not None:
                item.setEnabled(ok)
        if not ok and combo.currentIndex() != 0:
            combo.setCurrentIndex(0)   # fires _ploc_on_view_mode_changed

    def _ploc_show_mosaic_mode(self) -> None:
        """Switch the plate view to a mosaic-showing mode (used after a fresh
        mosaic is built). Keeps an already-active mosaic mode; otherwise picks
        'overlay' so the operator can compare the mosaic against the ideal grid."""
        combo = getattr(self, "_ploc_view_combo", None)
        if combo is None:
            self._ploc_mosaic_show = True
            return
        if combo.currentData() not in ("mosaic", "overlay"):
            idx = combo.findData("overlay")
            if idx >= 0:
                combo.setCurrentIndex(idx)   # fires _ploc_on_view_mode_changed
        else:
            self._ploc_on_view_mode_changed()

    # ── Manual global registration (slide mosaic onto wells by eye) ──

    def _ploc_on_align_mode_toggled(self, on: bool) -> None:
        """Entering manual-align makes sure the mosaic is visible (and at the
        chosen opacity) so the operator can see what they're sliding."""
        if not on:
            return
        # The mosaic must be visible to slide it onto the wells by eye.
        self._ploc_show_mosaic_mode()
        self._ploc_on_align_changed()

    def _ploc_on_align_changed(self, _v: int = 0) -> None:
        """Live-apply the slider Δx/Δy (µm) + opacity to the plate overlay."""
        dx = float(self._ploc_align_dx.value())
        dy = float(self._ploc_align_dy.value())
        op = int(self._ploc_align_op.value())
        self._ploc_align_dx_lbl.setText(f"{dx:.0f}")
        self._ploc_align_dy_lbl.setText(f"{dy:.0f}")
        self._ploc_align_op_lbl.setText(f"{op}%")
        view = getattr(self, "_ploc_plate_view", None)
        if view is None:
            return
        if hasattr(view, "set_mosaic_shift"):
            view.set_mosaic_shift(dx, dy)
        if hasattr(view, "set_mosaic_opacity"):
            view.set_mosaic_opacity(op / 100.0)

    def _ploc_reset_manual_align(self) -> None:
        """Zero the live nudge (does not touch a previously stored alignment)."""
        for sld in (self._ploc_align_dx, self._ploc_align_dy):
            sld.blockSignals(True)
            sld.setValue(0)
            sld.blockSignals(False)
        self._ploc_align_dx_lbl.setText("0")
        self._ploc_align_dy_lbl.setText("0")
        view = getattr(self, "_ploc_plate_view", None)
        if view is not None and hasattr(view, "set_mosaic_shift"):
            view.set_mosaic_shift(0.0, 0.0)

    def _ploc_clear_stored_align(self) -> None:
        """Forget this camera + objective's stored alignment so auto
        registration sets it on the next scan (releases a manual lock)."""
        key = self._ploc_camera_objective_key()
        store = self._ploc_mosaic_align_store()
        if store is not None and key:
            try:
                store.clear(key)
            except Exception as e:
                logger.warning(f"Clear stored alignment failed: {e}")
        self._ploc_reset_manual_align()
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            f"Cleared stored alignment for '{key}' — auto registration will "
            f"set it on the next scan.")
        logger.info(f"Manual mosaic align cleared: key='{key}'")

    def _ploc_shift_mosaic_by(self, ex: float, ey: float) -> bool:
        """Translate the mosaic overlay by (ex, ey) µm so the mosaic IMAGE tracks
        a well-map correction (re-anchor) or a by-eye nudge (manual align) — the
        image and the well centres move as one unit.

        Shifts the cached overlay extent, persists it to the per-plate
        ``MosaicStore`` (only when that plate has a stored mosaic — a calibration
        composite handed off in-memory is shifted in the view but not persisted),
        and re-pushes the overlay at the shifted extent via
        ``_ploc_set_overlay_image`` (which resets the view's user-shift + the page
        align sliders, so this is idempotent and cannot double-apply).

        Does NOT touch ``MosaicAlignmentStore`` — that "seed the next scan"
        per-camera + objective offset is specific to the by-eye manual-align
        intent and is handled by that caller. Returns True if a mosaic overlay
        was shifted (False = no mosaic loaded / zero shift)."""
        img = getattr(self, "_ploc_overlay_img", None)
        ext = getattr(self, "_ploc_overlay_ext", None)
        if img is None or ext is None:
            return False
        # Defensive: the extent must be the 4-element (min_x, min_y, max_x,
        # max_y) — a malformed cache must degrade to a no-op, not an IndexError.
        if not (isinstance(ext, (tuple, list)) and len(ext) >= 4):
            logger.warning(f"Mosaic shift: malformed overlay extent {ext!r}")
            return False
        if not ex and not ey:
            return False
        new_ext = (ext[0] + ex, ext[1] + ey, ext[2] + ex, ext[3] + ey)
        mstore = self._ploc_mosaic_store()
        pkey = self._ploc_plate_key()
        if mstore is not None and mstore.has(pkey):
            meta = mstore.get_meta(pkey) or {}
            # A pure map translation moves WHERE the image sits, not the
            # pixel↔extent registration — preserve the stored shift_um.
            sh = meta.get("shift_um")
            if not (isinstance(sh, (list, tuple)) and len(sh) >= 2):
                sh = (0.0, 0.0)
            # save() rebuilds the meta (dropping the stored last-good well
            # mapping) — snapshot it first and re-store it SHIFTED by the
            # same E so the mapping travels with the mosaic.
            wells_snap = {}
            try:
                if hasattr(mstore, "get_wells"):
                    wells_snap = mstore.get_wells(pkey)
            except Exception:
                wells_snap = {}
            try:
                mstore.save(
                    pkey, img, new_ext,
                    um_per_px=float(meta.get("um_per_px", 0.0)),
                    mosaic_scale=float(meta.get("mosaic_scale", 0.0)),
                    frames=int(meta.get("frames", 0)),
                    shift_um=(float(sh[0]), float(sh[1])))
            except Exception as e:
                logger.warning(f"Mosaic shift persist failed: {e}")
            if wells_snap and hasattr(mstore, "set_wells"):
                try:
                    mstore.set_wells(pkey, {
                        n: (p[0] + ex, p[1] + ey)
                        for n, p in wells_snap.items()})
                except Exception as e:
                    logger.debug(f"Mosaic shift: wells move skipped: {e}")
        # Move every stored SINGLE-WELL mosaic by the same E (metadata-only
        # extent rewrite) so the well scans travel with the plate as one unit.
        if mstore is not None and hasattr(mstore, "list_well_keys"):
            try:
                for well in mstore.list_well_keys(pkey):
                    wkey = f"{pkey}#{well}"
                    wext = mstore.get_extent_um(wkey)
                    if wext is None:
                        continue
                    mstore.update_extent(
                        wkey, (wext[0] + ex, wext[1] + ey,
                               wext[2] + ex, wext[3] + ey))
            except Exception as e:
                logger.debug(f"Mosaic shift: well-extent move skipped: {e}")
        self._ploc_set_overlay_image(img, new_ext)
        return True

    def _ploc_store_manual_align(self) -> None:
        """Persist the manual nudge as this camera + objective's alignment delta
        (prior stored shift + this nudge) so future mosaics start pre-aligned,
        and bake the nudge into the overlay (cached + persisted) so it stays
        aligned. Then zero the live nudge.

        Re-baking the cached overlay at the shifted extent — rather than only
        the on-disk copy — keeps the in-memory and persisted overlays in lock
        step and means a single store is idempotent (no double-apply if clicked
        twice), and it works for a calibration-dialog composite that was handed
        off but never persisted to MosaicStore.
        """
        view = getattr(self, "_ploc_plate_view", None)
        img = getattr(self, "_ploc_overlay_img", None)
        ext = getattr(self, "_ploc_overlay_ext", None)
        if (view is None or not (hasattr(view, "has_mosaic") and view.has_mosaic())
                or img is None or ext is None):
            QMessageBox.information(
                self, "Manual align",
                "Run a Mosaic scan (or a Calibrate… mosaic) first — there's no "
                "mosaic overlay to align.")
            return
        dx = float(self._ploc_align_dx.value())
        dy = float(self._ploc_align_dy.value())
        # 1) Persist the corrected global shift for this camera + objective so
        #    the NEXT mosaic seeds it (prior stored shift + this nudge — the
        #    operator nudged from whatever was already displayed).
        key = self._ploc_camera_objective_key()
        store = self._ploc_mosaic_align_store()
        total = (dx, dy)
        if store is not None and key:
            try:
                prior = store.get_shift_um(key) or (0.0, 0.0)
                total = (prior[0] + dx, prior[1] + dy)
                store.set_shift_um(
                    key, total[0], total[1], source="manual_align")
            except Exception as e:
                logger.warning(f"Manual align store failed: {e}")
        # 2+3) Persist the shifted overlay to the per-plate MosaicStore + re-push
        #      it at the shifted extent (resets the view user-shift + page sliders
        #      so a second Store can't double-apply). Shared with re-anchor.
        self._ploc_shift_mosaic_by(dx, dy)
        try:
            self.calibration_data_changed.emit()
        except Exception:
            pass
        self._ploc_confirm_label.setVisible(True)
        self._ploc_confirm_label.setText(
            f"Manual align stored for '{key}': total shift "
            f"({total[0]:.0f}, {total[1]:.0f}) µm.")
        logger.info(
            f"Manual mosaic align stored: key='{key}' nudge=({dx:.0f},{dy:.0f}) "
            f"total=({total[0]:.0f},{total[1]:.0f}) µm")

    # ── Right-context tab builders (v7.4.2) ──────────────────────
    #
    # Each builder writes widgets into the passed-in layout. They
    # exist as wrappers around the original inline build code so we
    # don't have to refactor every reference to the camera /
    # wizard / plate-view widgets that the page already uses.

    def _build_camera_tab_content(self, layout: "QVBoxLayout") -> None:
        """Camera controls — sources, layout, per-camera rows, objective
        & µm/px settings. Sourced from the original get_context_widget."""
        self._build_camera_container_into(layout)

    def _build_needle_tab_content(self, layout: "QVBoxLayout") -> None:
        """Needle Z setup + Z-bottom Auto-Cal + Manual Teaching.

        v7.4.2 first cut: the entire calibration wizard lives here
        because Safe Z / Top Z / Z-Bottom are needle-Z positioning.
        Plate-detection steps (auto-cal 3-well) bleed into here too —
        a future pass can split them out.
        """
        self._build_wizard_steps(layout)

    def _build_plate_tab_content(self, layout: "QVBoxLayout") -> None:
        """Plate format combo + plate position view."""
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
        self._build_cal_views(layout)

    def _build_camera_container_into(self, layout: "QVBoxLayout") -> None:
        """Full camera-controls block. Body extracted from the original
        get_context_widget body — all per-camera state lives on self
        so this can only safely run once."""
        # The original implementation in the v7.4.1 file is below;
        # we call it to populate the camera widgets.
        self._build_legacy_camera_section(layout)

    # Legacy build retained for back-compat — invoked by the camera
    # tab builder above.

    def _build_legacy_camera_section(self, layout) -> None:
        """Original camera-controls build, now scoped to a passed-in layout."""

        # ── Camera Controls (collapsible) ────────────────────────
        cam_header_row = QHBoxLayout()
        self._ctx_cam_toggle = QPushButton("▾ Camera Controls")
        self._ctx_cam_toggle.setObjectName("flatBtn")
        self._ctx_cam_toggle.setStyleSheet(
            f"text-align: left; font-weight: 600; color: {COLORS['blue']}; "
            f"padding: {sp(6)} 0px {sp(2)} 0px; border: none; background: transparent;"
            f"border-bottom: 1px solid {COLORS['surface1']}; margin-bottom: {sp(4)};")
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
                f"border-radius: {sp(4)}; padding: {sp(2)}; }}")
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
                f"color: {COLORS['text']}; padding: {sp(3)} {sp(8)}; "
                f"border-radius: {sp(3)}; font-size: {sf(8)}pt; }}"
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

        # v7.4.2: Plate config + plate views + wizard moved to other
        # tabs — see _build_needle_tab_content and _build_plate_tab_content.
        # No return — this helper writes into the passed-in layout.

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

        # Push to CameraManager — ONLY for the microscope camera.
        # v7.5.x: the objective-derived µm/px is a microscope concept, but
        # `_get_camera_spec_for_idx` returns the shared `camera_config.camera_spec`
        # for *every* slot, so pushing it to all slots clobbered the needle/plate
        # cameras' stage-motion calibration on every config change (this method
        # runs for all slots from `set_hardware_config`). Gate the push to the
        # slot actually tagged MICROSCOPE; other slots keep their measured value.
        mgr = getattr(self, '_camera_manager', None)
        hw = getattr(self, '_hardware_config', None)
        micro_idx = (
            hw.camera_for_role(CameraRole.MICROSCOPE)
            if (hw is not None and CameraRole is not None) else None
        )
        if (mgr is not None and active_um_per_px is not None
                and micro_idx is not None and idx == micro_idx):
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
        """v7.4.4: Calibration page IS the workflow tab widget.

        The right context column was removed — every control needed
        for a workflow lives inside that workflow's tab. The page
        layout is just a compact stage-position strip across the top
        plus the four workflow tabs filling the remaining space:

          1. **Needle Location** — dual-camera edge-click recenter
          2. **Z-Offset Calibration** — Safe Z / Top Z / Z-Bottom auto
          3. **Plate Location** — click-snap N-well XY-map workflow
          4. **Custom** — legacy camera grid + full wizard + plate cal
        """
        _bg = COLORS['base']
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(6), s(8), s(6))
        outer.setSpacing(s(6))

        mono = QFont("Consolas", scaled_font_size(12))

        # ── Compact position readout strip ────────────────────────
        pos_card = QFrame()
        pos_card.setObjectName("cardFrame")
        pos_layout = QHBoxLayout(pos_card)
        pos_layout.setContentsMargins(s(10), s(4), s(10), s(4))
        pos_layout.setSpacing(s(12))
        for name, attr in [("X (µm):", "lbl_x"),
                           ("Y (µm):", "lbl_y"),
                           ("Z (mm):", "lbl_z")]:
            pos_layout.addWidget(QLabel(name))
            lbl = QLabel("—")
            lbl.setFont(mono)
            lbl.setObjectName("valueLabel")
            pos_layout.addWidget(lbl)
            setattr(self, attr, lbl)
        pos_layout.addStretch()
        outer.addWidget(pos_card)

        # ── Build the legacy camera grid widget (parent=None for now)
        # so the Custom tab can adopt it once it's constructed. The
        # grid + relayout machinery is unchanged from v7.4.2; only
        # its parent owner moves.
        self._cam_grid_widget = QWidget()
        self._cam_grid_layout = QGridLayout(self._cam_grid_widget)
        self._cam_grid_layout.setSpacing(4)
        self._cam_grid_layout.setContentsMargins(0, 0, 0, 0)

        if self._camera_manager and self._camera_manager.is_available:
            from gui.widgets.camera_feed_view import CameraFeedView

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
                fv.clicked.connect(
                    lambda px, py, idx=i: self._on_feed_clicked(idx, px, py)
                )
                self._cam_feed_views.append(fv)

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

        # ── Workflow tabs ────────────────────────────────────────
        self._workflow_tabs = QTabWidget()
        self._workflow_tabs.setObjectName("calibrationWorkflowTabs")
        # v7.5.x: Needle Location now ALSO hosts the reference-Z heights (the
        # former "Needle Offset Calibration" tab, merged in — the operator
        # treats them as one needle-reference calibration). The Z heights still
        # come before Plate Location (which retracts to the Safe/Move Z between
        # wells, so they must be taught first).
        self._workflow_tabs.addTab(
            self._build_needle_location_tab(), "Needle Location")
        # v7.5.x: pump compliance / pressure-relief calibration on its own tab,
        # right next to Needle Location (the needle is parked at the needle
        # location, in view of the side cameras, while it runs).
        self._workflow_tabs.addTab(
            self._build_pump_compliance_tab(), "Pump Compliance")
        self._workflow_tabs.addTab(
            self._build_plate_location_tab(), "Plate Location")
        # v7.5.x: rosette sub-well calibration on its own tab, right after
        # Plate Location (it needs the plate map for the well's scan bounds):
        # mosaic-scan one rosette-assigned well, place the pattern centre,
        # refine each sub-well.
        self._workflow_tabs.addTab(
            self._build_rosette_tab(), "Rosettes")
        # v7.5.x: the focus-based per-well Z-bottom auto-cal runs AFTER Plate
        # Location (it drives to each calibration well, so it needs the
        # finished XY map). Its live microscope feed lives on this tab.
        self._workflow_tabs.addTab(
            self._build_plate_z_autocal_tab(), "Plate Z Auto-Cal")
        self._workflow_tabs.addTab(
            self._build_custom_tab(), "Custom")
        # Index of the Pump Compliance tab — start the live needle side-camera
        # feeds when it's shown so the operator can watch the droplet.
        self._compcal_tab_index = 1
        # Index of the Rosettes tab — start the microscope feed when shown.
        self._rosette_tab_index = 3
        # Index of the Plate Z Auto-Cal tab — start the microscope feed when
        # it's shown so the operator can teach the first spot by eye.
        self._zauto_tab_index = 4
        self._workflow_tabs.currentChanged.connect(
            self._on_workflow_tab_changed)
        outer.addWidget(self._workflow_tabs, stretch=1)

        # The right-context entry point still exists on this class
        # for back-compat with app.py's hasattr check, but it now
        # returns None so no right column is shown.
        self._right_context_widget = None

        # Trigger the initial calibration restore.
        try:
            self._load_calibration()
        except Exception as e:
            logger.debug(f"Initial calibration load skipped: {e}")

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
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: {sp(4)}; "
            f"margin-top: {sp(6)}; padding-top: {sp(12)}; }}")
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
            "Guided per-well Z-bottom calibration (opens the Plate Z Auto-Cal "
            "tab: refocus the glass, confirm, the needle finds best focus)")
        self._btn_auto_z_cal.clicked.connect(self._start_auto_z_cal_on_tab)
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
            if isinstance(zp, dict):
                z_mm = float(zp.get("Z", 0) or 0)
            else:
                # v7.4.2 hotfix: tuple form — route through axis_map.
                z_val = ctrl.zp_logical_value(zp, "Z") if zp else None
                z_mm = float(z_val) if z_val is not None else 0.0
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
            # v7.5.x: _cal_plate_view positions its predicted/calibrated dots
            # as ``well_abs_mm − a1_mm`` — i.e. it expects A1 in ABSOLUTE mm.
            # The page's _taught_a1 is in absolute µm, so passing it raw was a
            # 1000× frame error that flung the dots far off, ballooned the
            # scene's bounding rect, and made fitInView shrink the plate to a
            # speck (the "plate view changes size" report). Convert to mm.
            self._cal_plate_view.set_taught_a1(
                (self._taught_a1[0] / 1000.0, self._taught_a1[1] / 1000.0)
                if self._taught_a1 is not None else None)
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
                    # v7.5.x: seed the simulated world's plate at the SAME
                    # per-machine XY safety-envelope centre used for the
                    # scan/manual-fit predictions (was hardcoded (65000,42500),
                    # the legacy 130×85mm travel midpoint). Keeping all seeds
                    # coupled is essential — otherwise the simulated stage
                    # drives to predicted wells in one frame while the camera
                    # renders the plate in another, breaking simulated
                    # auto-calibration.
                    try:
                        center = self.controller.default_plate_center_um()
                    except Exception:
                        center = (65000.0, 42500.0)  # 130×85mm travel center (fallback)
                    sim.set_plate(self._plate, plate_center_um=center)

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
        # v7.4.3: forward the tick to the reusable jog context panel
        panel = getattr(self, "_jog_left_panel", None)
        if panel is not None:
            try:
                panel.on_status_update()
            except Exception:
                pass

        ctrl = self.controller

        # Update position readout
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            # v7.2.7: controller reports µm directly — no conversion needed
            ux = xy[0] - ctrl.zero_position["x"]
            uy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{ux:,.1f}")
            self.lbl_y.setText(f"{uy:,.1f}")
            # v7.4.4: feed the shared XY workspace view so the needle
            # tracks live (same canvas the Jog page uses).
            ploc = getattr(self, "_ploc_plate_view", None)
            if ploc is not None and hasattr(ploc, "set_position"):
                try:
                    # v7.5.x: absolute envelope → push zero so it draws zero-ref.
                    if hasattr(ploc, "set_zero_offset"):
                        ploc.set_zero_offset(
                            ctrl.zero_position["x"], ctrl.zero_position["y"])
                    ploc.set_position(ux, uy)
                except Exception:
                    pass
            # v7.5.x: track the live stage centre on the microscope overlay so
            # the reference markers stay registered as the stage moves.
            live = getattr(self, "_ploc_live_view", None)
            if live is not None and hasattr(live, "set_reference_stage_position"):
                try:
                    live.set_reference_stage_position(
                        float(xy[0]), float(xy[1]))
                except Exception:
                    pass
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")
            ploc = getattr(self, "_ploc_plate_view", None)
            if ploc is not None and hasattr(ploc, "set_position"):
                try:
                    ploc.set_position(None, None)
                except Exception:
                    pass

        zp = ctrl.get_zp_position(cached=True)
        # v7.4.2 hotfix: read Z via logical axis (honours axis_map).
        z_val = ctrl.zp_logical_value(zp, "Z")
        # v7.5.x: feed the Plate Location Z side view — live needle Z plus the
        # zero offsets so it renders in the zero-ref frame (mirrors Jog page).
        xz = getattr(self, "_ploc_xz_view", None)
        if xz is not None:
            try:
                zero = ctrl.zero_position
                xz.set_zero_offset_x(zero.get("x", 0.0))
                xz.set_zero_offset_z(zero.get("Z", 0.0))
                zx_um = (xy[0] - zero["x"]) if xy[0] is not None else None
                z_zr = ((z_val - zero.get("Z", 0.0))
                        if z_val is not None else None)
                xz.set_position(zx_um, z_zr)
            except Exception:
                pass
        # v7.5.x: feed the Needle Offset tab Z side view the same way (live
        # needle Z + zero offsets + current reference heights).
        xzo = getattr(self, "_zoff_xz_view", None)
        if xzo is not None:
            try:
                zero = ctrl.zero_position
                xzo.set_zero_offset_x(zero.get("x", 0.0))
                xzo.set_zero_offset_z(zero.get("Z", 0.0))
                zx_um = (xy[0] - zero["x"]) if xy[0] is not None else None
                z_zr = ((z_val - zero.get("Z", 0.0))
                        if z_val is not None else None)
                xzo.set_position(zx_um, z_zr)
                xzo.set_z_references(self.get_z_references())
            except Exception:
                pass
        if z_val is not None:
            # v7.5.x: show Z in the unified user frame (0 at bottom datum,
            # up = +) so it agrees with the rest of the app.
            _z_disp = (ctrl.raw_to_user_z(z_val)
                       if hasattr(ctrl, "raw_to_user_z")
                       else z_val - ctrl.zero_position.get('Z', 0))
            self.lbl_z.setText(f"{_z_disp:.2f}")

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
            # Map the plate-local offset onto stage axes (per-machine sign).
            sx, sy = self._plate_axis_sign()
            dx_mm = sx * (wx - a1x)
            dy_mm = sy * (wy - a1y)

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
            self._taught_a1[0], self._taught_a1[1], self._plate_axis_sign()
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

        # Compute predicted positions from A1 or plate-center assumption.
        # v7.5.x: seed the plate-centre at the XY safety-envelope centre
        # (per-machine) instead of the hardcoded 130×85mm travel midpoint,
        # so it agrees with the default-seed used elsewhere.
        try:
            stage_center = self.controller.default_plate_center_um()
        except Exception:
            stage_center = (65000.0, 42500.0)  # 130×85mm travel center (fallback)
        if self._taught_a1 is not None:
            self._predicted_positions = self._plate.get_all_positions_from_a1(
                *self._taught_a1, plate_axis_sign=self._plate_axis_sign())
        else:
            self._predicted_positions = self._plate.get_all_positions_from_plate_center(
                *stage_center, plate_axis_sign=self._plate_axis_sign())
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
        # v7.5.x: drain the camera's buffered backlog + settle so the captured
        # frame is post-move (not a stale frame exposed during the move).
        frame = (cam.capture_fresh_frame(discard_n_frames=5, settle_ms=120)
                 if cam else None)
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
        # This path uses the 3-well similarity affine, not the freeform warp —
        # drop any stale warp so it isn't persisted/restored over this fit.
        self._plate_warp = None

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

    # ── v7.5.x: Needle Offset tab — live feed + manual first spot ──

    def _on_workflow_tab_changed(self, index: int) -> None:
        """Start the relevant live feed for the tab being shown: the microscope
        feed on Plate Z Auto-Cal (teach the first spot by eye), and the needle
        side cameras on Pump Compliance (watch the droplet at the tip)."""
        if index == getattr(self, "_zauto_tab_index", -1):
            self._zoff_ensure_live_camera()
        if index == getattr(self, "_compcal_tab_index", -1):
            self._compcal_ensure_live_cameras()
        if index == getattr(self, "_rosette_tab_index", -1):
            self._rosette_ensure_live_camera()
            self._rosette_refresh_wells()

    def _zoff_ensure_live_camera(self) -> None:
        """Point the Plate Z Auto-Cal live view at the microscope slot and
        start it. Mirrors ``_ploc_ensure_live_camera``; ``set_camera`` is
        only re-issued on an actual slot change and ``start`` is a no-op
        when the camera is already running."""
        view = getattr(self, "_zoff_live_view", None)
        if view is None or self._camera_manager is None:
            return
        cam_idx = getattr(self, "_ploc_live_cam_idx", 0)
        try:
            if view.cam_idx != cam_idx:
                view.set_camera(cam_idx)
            self._camera_manager.start(cam_idx)
        except Exception as e:
            logger.debug(f"NeedleOffset: live camera start failed: {e}")

    # ── dual-widget progress helpers (Custom tab + Needle Offset tab) ──

    def _auto_z_set_progress(self, text: str, color: str | None = None) -> None:
        """Write the auto-cal progress line to whichever progress labels
        exist (the Custom-tab wizard and/or the Needle Offset tab)."""
        for attr in ("_lbl_auto_z_progress", "_zoff_lbl_auto_z"):
            lbl = getattr(self, attr, None)
            if lbl is not None:
                lbl.setText(text)
                if color is not None:
                    lbl.setStyleSheet(f"color: {color}; font-size: 9pt;")

    def _auto_z_set_running(self, running: bool) -> None:
        """Toggle the Run/Cancel buttons on both auto-cal hosts."""
        for run_attr in ("_btn_auto_z_cal", "_zoff_btn_run_z"):
            btn = getattr(self, run_attr, None)
            if btn is not None:
                btn.setEnabled(not running)
        for cancel_attr in ("_btn_cancel_auto_z", "_zoff_btn_cancel_z"):
            btn = getattr(self, cancel_attr, None)
            if btn is not None:
                btn.setEnabled(running)
        if not running:
            # Run finished/cancelled — clear the per-well action buttons too.
            self._auto_z_phase_buttons()

    def _auto_z_phase_buttons(self) -> None:
        """Enable/disable the per-well action buttons for the current phase.

        Confirm is live only while awaiting the operator's glass-focus
        confirmation; Record-now only during the descent; Accept/Redo only
        once a Z has been recorded. ``getattr``-guarded so the Custom-tab
        host (which lacks these buttons) is unaffected."""
        phase = getattr(self, "_auto_z_phase", "idle")
        scanning = getattr(self, "_auto_z_scanning", False)
        states = {
            "_zauto_btn_confirm": scanning and phase == "await_focus",
            "_zauto_btn_record": scanning and phase in ("coarse", "fine"),
            "_zauto_btn_accept": scanning and phase == "recorded",
            "_zauto_btn_redo": scanning and phase == "recorded",
        }
        for attr, on in states.items():
            btn = getattr(self, attr, None)
            if btn is not None:
                btn.setEnabled(on)

    # ── v7.3.1: Auto Z-Bottom Calibration ─────────────────────────

    def _start_auto_z_cal_on_tab(self) -> None:
        """Switch to the Plate Z Auto-Cal tab (where the per-well Confirm /
        Record / Accept buttons live) and start the guided flow. Used by the
        Custom-tab "Auto Z-Cal" button so the operator isn't left on a tab
        without the interactive controls."""
        tabs = getattr(self, "_workflow_tabs", None)
        idx = getattr(self, "_zauto_tab_index", None)
        if tabs is not None and idx is not None:
            tabs.setCurrentIndex(idx)
        self._start_auto_z_cal()

    def _start_auto_z_cal(self):
        """Start the guided per-well Z-bottom calibration.

        For EACH calibration well:
        1. Retract Z + travel XY to the well, then pause (``await_focus``).
        2. The operator refocuses the microscope on the glass and clicks
           Confirm — the needle then lowers (``coarse``/``fine``).
        3. The best-focus Z is auto-recorded (with a Record-now override).
        4. The operator Accepts (advance) or Redoes the well.
        5. After the last well, the Z plane is fit from the recorded points.
        """
        # Validation
        if self._plate is None:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Select a plate format first.")
            return
        if getattr(self, '_safe_z', None) is None:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Set Fast Move Z first.")
            return
        if getattr(self, '_top_z', None) is None:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Set Plate Top Z first.")
            return
        # Need position data to navigate to wells
        if not self._calibrated_positions and not self._predicted_positions:
            QMessageBox.warning(self, "Cannot Auto Z-Cal",
                                "Finish Plate Location (the XY map) first.")
            return

        # Make sure the microscope feed is live before we check it.
        self._zoff_ensure_live_camera()

        cam = self._get_primary_camera()
        if cam is None or not getattr(cam, 'is_running', False):
            QMessageBox.warning(self, "No Camera",
                                "Start a camera before auto Z calibration.")
            return

        # Get well depth from spinbox (estimate fallback for the window seed)
        if hasattr(self, '_well_depth_spin'):
            well_depth_mm = self._well_depth_spin.value()
        else:
            well_depth_mm = getattr(self._plate, 'well_depth_mm', 17.4)
        self._auto_z_well_depth = well_depth_mm

        # Wells to calibrate (same 3 as plate calibration). Every well uses
        # the same guided refocus → confirm → lower → record flow.
        self._auto_z_wells = self._get_calibration_wells()
        self._auto_z_results = {}
        self._auto_z_last_z = None
        self._auto_z_pending_z = None
        self._auto_z_well_idx = 0
        self._auto_z_scanning = True
        self._auto_z_phase = "navigate"

        # UI state
        self._auto_z_set_running(True)
        self._auto_z_set_progress(
            f"Auto Z-Cal: 0/{len(self._auto_z_wells)} wells...",
            COLORS['blue'])

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
        self._auto_z_set_running(False)
        self._auto_z_set_progress("Auto Z-Cal cancelled.", COLORS['yellow'])
        logger.info("Auto Z-Cal cancelled")

    def _auto_z_move_to_h(self, h: float) -> float:
        """Move Z to a *height above the reference bottom* ``h`` (mm, + =
        away from the plate / safe side), clamping to the floor.

        ``h`` → zero-ref Z via ``plate_relative_to_zref`` so "toward the
        plate" (decreasing ``h``) is physically correct on both a
        conventional machine (ZDIR=+1) and ME3B V1 (ZDIR=-1, where the
        needle descends as zero-ref Z *increases*). Returns the clamped
        ``h``."""
        h = max(h, self._auto_z_floor_h)
        self._auto_z_h = h
        z = plate_relative_to_zref(self._auto_z_ref_z, h)
        self._auto_z_current_z = z
        if self.controller is not None:
            self.controller.move_z_absolute(z, from_zero_ref=True)
        return h

    def _auto_z_tick(self):
        """Timer-driven part of the per-well Z-bottom state machine.

        Handles ``navigate`` (travel to the well, then pause for the
        operator to refocus the glass) and the ``coarse``/``fine`` needle
        descent. ``await_focus`` and ``recorded`` are operator-gated (driven
        by the Confirm / Record / Accept / Redo buttons), not the timer.

        The descent is parametrized by ``h`` = height above the reference
        bottom (mm, + = safe/away from plate) and is polarity-general (see
        ``_auto_z_move_to_h``)."""
        if not self._auto_z_scanning:
            return

        idx = self._auto_z_well_idx
        wells = self._auto_z_wells

        if idx >= len(wells):
            self._auto_z_finish()
            return

        well_name = wells[idx]

        if self._auto_z_phase == "navigate":
            # Retract + travel to the well XY at safe Z (no Z lowering), then
            # pause for the operator to refocus the microscope on the glass.
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
            self._auto_z_enter_await_focus()
            # Operator-gated from here — wait for Confirm (no timer restart).

        elif self._auto_z_phase == "coarse":
            # Coarse sweep: 0.15mm steps, descending from above.
            # Looking for needle to first appear (blurry focus rising).
            # SAFETY: never descend below estimated bottom.
            score = self._auto_z_get_focus_score()
            h = self._auto_z_h
            z = self._auto_z_current_z

            self._auto_z_set_progress(
                f"Auto Z-Cal: {well_name} \u2014 coarse Z={z:.3f}mm  "
                f"focus={score:.1f}")

            # Track best seen so far
            if score > self._auto_z_best_score:
                self._auto_z_best_score = score
                self._auto_z_best_z = z
                self._auto_z_best_h = h
                self._auto_z_decline_count = 0
            elif self._auto_z_best_h is not None:
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
                # Needle is appearing — back up away from the plate
                # (increase h) above the best and re-approach in fine.
                self._auto_z_move_to_h(self._auto_z_best_h + 0.3)
                self._auto_z_best_z = None
                self._auto_z_best_h = None
                self._auto_z_best_score = 0.0
                self._auto_z_decline_count = 0
                self._auto_z_phase = "fine"
                self._auto_z_timer.start(300)
                return

            # SAFETY: hit the floor — do not descend further
            if h <= self._auto_z_floor_h + 1e-9:
                if self._auto_z_best_h is not None:
                    # Had some focus signal, refine it
                    self._auto_z_move_to_h(self._auto_z_best_h + 0.3)
                    self._auto_z_best_z = None
                    self._auto_z_best_h = None
                    self._auto_z_best_score = 0.0
                    self._auto_z_decline_count = 0
                    self._auto_z_phase = "fine"
                    self._auto_z_timer.start(300)
                else:
                    logger.warning(f"Auto Z-Cal: no focus found for {well_name} "
                                   f"(stopped at safety floor Z={z:.3f}mm)")
                    self._auto_z_enter_recorded(None)
                return

            # Step toward the plate (coarse)
            self._auto_z_move_to_h(h - self._auto_z_coarse_step)
            self._auto_z_timer.start(300)

        elif self._auto_z_phase == "fine":
            # Fine sweep: 0.03mm steps, tracking peak focus.
            # SAFETY: approach from above only. Stop as soon as focus
            # declines — the peak IS the well bottom, do not go past it.
            score = self._auto_z_get_focus_score()
            h = self._auto_z_h
            z = self._auto_z_current_z

            self._auto_z_set_progress(
                f"Auto Z-Cal: {well_name} \u2014 fine Z={z:.3f}mm  "
                f"focus={score:.1f}  "
                f"best={self._auto_z_best_score:.1f}")

            if score > self._auto_z_best_score:
                self._auto_z_best_score = score
                self._auto_z_best_z = z
                self._auto_z_best_h = h
                self._auto_z_decline_count = 0
            elif self._auto_z_best_h is not None:
                self._auto_z_decline_count += 1

            # Peak found: 3 consecutive declining steps → record immediately.
            # The needle is approaching the glass — do NOT continue down.
            if self._auto_z_decline_count >= 3 and self._auto_z_best_z is not None:
                logger.info(f"Auto Z-Cal: {well_name} best focus at "
                            f"Z={self._auto_z_best_z:.3f}mm "
                            f"(score={self._auto_z_best_score:.1f})")
                self._auto_z_enter_recorded(self._auto_z_best_z)
                return

            # SAFETY: hard floor — never descend past the search window
            if h <= self._auto_z_floor_h + 1e-9:
                if self._auto_z_best_z is not None:
                    logger.info(f"Auto Z-Cal: {well_name} best focus at "
                                f"Z={self._auto_z_best_z:.3f}mm "
                                f"(stopped at safety floor)")
                else:
                    logger.warning(f"Auto Z-Cal: {well_name} no clear peak "
                                   f"(stopped at safety floor)")
                self._auto_z_enter_recorded(self._auto_z_best_z)
                return

            # Step toward the plate (fine)
            self._auto_z_move_to_h(h - self._auto_z_fine_step)
            self._auto_z_timer.start(300)

    # ── per-well operator-gated steps (Confirm / Record / Accept / Redo) ──

    def _auto_z_enter_await_focus(self) -> None:
        """Pause at the current well for the operator to refocus the
        microscope on the glass (manual knob) and click Confirm."""
        idx = self._auto_z_well_idx
        well = self._auto_z_wells[idx]
        self._auto_z_phase = "await_focus"
        self._auto_z_pending_z = None
        self._auto_z_set_progress(
            f"At {well} ({idx + 1}/{len(self._auto_z_wells)}). Focus the "
            f"microscope on the glass, then click “Confirm focus & lower "
            f"needle”.", COLORS['blue'])
        self._auto_z_phase_buttons()

    def _zauto_confirm_focus(self) -> None:
        """Operator confirmed the glass is in focus — set up the safe descent
        window and start lowering the needle to find best focus."""
        if not self._auto_z_scanning or self._auto_z_phase != "await_focus":
            return
        well = self._auto_z_wells[self._auto_z_well_idx]

        # Reference bottom for the bounded descent window: the previous
        # accepted Z (tracks plate tilt) → Plate Bottom Z → top_z − well_depth.
        if self._auto_z_last_z is not None:
            self._auto_z_ref_z = self._auto_z_last_z
        elif getattr(self, "_plate_bottom_z", None) is not None:
            self._auto_z_ref_z = self._plate_bottom_z
        elif getattr(self, "_top_z", None) is not None:
            self._auto_z_ref_z = self._top_z - getattr(
                self, "_auto_z_well_depth", 17.4)
        else:
            self._auto_z_ref_z = 0.0
        self._auto_z_floor_h = -self._zauto_tilt_margin

        self._auto_z_best_z = None
        self._auto_z_best_h = None
        self._auto_z_best_score = 0.0
        self._auto_z_baseline_score = 0.0
        self._auto_z_decline_count = 0

        # Fast move to the approach height (above the glass = safe), then
        # capture the glass-only baseline focus (needle not yet in focus).
        self._auto_z_move_to_h(self._zauto_approach_margin)
        self._auto_z_capture_baseline()

        self._auto_z_phase = "coarse"
        self._auto_z_phase_buttons()
        self._auto_z_set_progress(
            f"{well} — lowering needle to find best focus…",
            COLORS['blue'])
        if self._auto_z_timer is not None:
            self._auto_z_timer.start(300)

    def _zauto_record_now(self) -> None:
        """Manual override during the descent: record the current best-focus
        Z (or the live Z if none seen yet) as this well's bottom."""
        if not self._auto_z_scanning or self._auto_z_phase not in ("coarse", "fine"):
            return
        z = (self._auto_z_best_z if self._auto_z_best_z is not None
             else self._auto_z_current_z)
        self._auto_z_enter_recorded(z)

    def _auto_z_enter_recorded(self, z: float | None) -> None:
        """Hold the provisional best-focus Z ``z`` for the current well and
        retract to safe Z, waiting for the operator to Accept or Redo."""
        well = self._auto_z_wells[self._auto_z_well_idx]
        # Retract so the operator can inspect / the next travel stays safe.
        if (self.controller is not None
                and getattr(self.controller, "is_zp_connected", False)
                and self._safe_z is not None):
            self.controller.move_z_absolute(self._safe_z, from_zero_ref=True)
        self._auto_z_pending_z = z
        self._auto_z_phase = "recorded"
        self._auto_z_phase_buttons()
        if z is None:
            self._auto_z_set_progress(
                f"{well} — no clear focus peak. Redo, or refocus and use "
                f"“Record now” during the next descent.",
                COLORS['yellow'])
        else:
            self._auto_z_set_progress(
                f"{well} — best-focus Z = {z:.3f} mm. Accept to keep it, "
                f"or Redo.", COLORS['green'])

    def _zauto_accept_well(self) -> None:
        """Keep the provisional Z for the current well and advance (fitting
        the Z plane once the last well is accepted)."""
        if not self._auto_z_scanning or self._auto_z_phase != "recorded":
            return
        well = self._auto_z_wells[self._auto_z_well_idx]
        z = self._auto_z_pending_z
        if z is not None:
            self._auto_z_results[well] = z
            self._z_teach_points[well] = z
            self._auto_z_last_z = z
            logger.info(f"Auto Z-Cal: {well} bottom accepted at Z={z:.3f}mm")
        # Update the Z-plane teach display.
        if hasattr(self, 'lbl_zplane'):
            self.lbl_zplane.setText(
                f"{len(self._z_teach_points)}/3 teach points")
            if len(self._z_teach_points) >= 3:
                self.lbl_zplane.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: 9pt;")
        self._auto_z_pending_z = None
        self._auto_z_well_idx += 1
        if self._auto_z_well_idx >= len(self._auto_z_wells):
            self._auto_z_finish()
        else:
            self._auto_z_phase = "navigate"
            self._auto_z_phase_buttons()
            if self._auto_z_timer is not None:
                self._auto_z_timer.start(200)

    def _zauto_redo_well(self) -> None:
        """Discard the provisional Z and re-teach the current well (the stage
        is already at this well, retracted — just re-arm the focus gate)."""
        if not self._auto_z_scanning or self._auto_z_phase != "recorded":
            return
        self._auto_z_pending_z = None
        self._auto_z_enter_await_focus()

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
            self._auto_z_set_progress(
                "Auto Z-Cal failed: no well bottoms detected.", COLORS['red'])
        else:
            # Build result summary
            parts = [f"{w}: Z={z:.3f}mm" for w, z in self._auto_z_results.items()]
            self._auto_z_set_progress(
                f"\u2705 Auto Z-Cal: {n_found} wells \u2014 {', '.join(parts)}",
                COLORS['green'])

            # Auto-fit Z plane if we have 3+ points
            if n_found >= 3:
                self._try_fit_z_plane()

        # UI state
        self._auto_z_set_running(False)

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
        # v7.4.2 hotfix: read Z via logical axis.
        z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        if z_val is None:
            return
        z_offset = z_val - self.controller.zero_position.get("Z", 0)
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

    # v7.5.x: bounds for accepting a manual plate-warp fit. A real plate
    # alignment is ~1.0 scale on BOTH axes and a few degrees of rotation;
    # anything outside these is a degenerate fit from close/noisy/near-collinear
    # control points, which extrapolates distant wells off the plate. Bounds are
    # on the LINEAR part only (translation is inherently limited by the reachable
    # stage range, and the plate may legitimately be larger than the travel —
    # see the ME3B V1 envelope-vs-footprint note — so absolute position is not a
    # valid test).
    _FIT_MIN_SCALE = 0.8
    _FIT_MAX_SCALE = 1.25
    _FIT_MAX_ROTATION_DEG = 30.0

    def _warp_is_plausible(self, warp) -> tuple[bool, str]:
        """Guard against a degenerate manual fit. Returns ``(ok, reason)``.

        Rejects when either axis of the affine linear part is scaled outside
        ``[_FIT_MIN_SCALE, _FIT_MAX_SCALE]`` or the rotation exceeds
        ``_FIT_MAX_ROTATION_DEG`` — the signatures of a fit that would send
        navigation to the wrong place and drive the stage to an envelope corner.
        """
        try:
            rot, _scl, _ = warp.similarity_approx()
            smin, smax = warp.linear_scale_range()
            if smin < self._FIT_MIN_SCALE or smax > self._FIT_MAX_SCALE:
                return False, (f"implausible scale (axes {smin:.2f}–{smax:.2f}×, "
                               f"expected ~1.0)")
            if abs(rot) > self._FIT_MAX_ROTATION_DEG:
                return False, f"implausible rotation {rot:.0f}°"
        except Exception as e:
            # Can't analyze (e.g. degraded translation-only mode) — don't block.
            logger.debug(f"_warp_is_plausible: linear check skipped: {e}")
        return True, ""

    def _manual_fit_xy(self):
        """Compute calibrated well positions from manually taught XY points.

        With 1 point: uses geometry prediction from A1.
        With 2 points: computes scale + rotation (legacy alignment).
        With 3+ points: uses Procrustes SVD if available, else scale+rotation from first 2.
        """
        # v7.4.6: freeform manual-teach pairs (Plate Location tab) count
        # as taught points too — each is an (predicted, measured) stage-µm
        # pair injected straight into the Procrustes fit below.
        free_pairs = list(getattr(self, '_free_teach_pairs', []))
        n = len(self._xy_teach_points) + len(free_pairs)
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
                # Use the XY safety-envelope centre as a rough estimate.
                # v7.5.x: per-machine envelope centre rather than the
                # hardcoded 130×85mm travel midpoint.
                try:
                    center_x, center_y = self.controller.default_plate_center_um()
                except Exception:
                    center_x, center_y = 65000.0, 42500.0
                self._predicted_positions = \
                    self._plate.get_all_positions_from_plate_center(
                        center_x, center_y, self._plate_axis_sign())

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
        # v7.4.6: freeform pairs carry their own (predicted, measured)
        # stage-\u00b5m coordinates \u2014 no well-name lookup needed.
        for (sx, sy), (tx, ty) in free_pairs:
            pred_pts.append((sx, sy))
            meas_pts.append((tx, ty))

        if len(pred_pts) < 2:
            if hasattr(self, '_gen_status'):
                self._gen_status.setText(
                    f"Need \u22652 matched points (have {len(pred_pts)}). "
                    "Add more wells or freeform points.")
                self._gen_status.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
            return

        # v7.5.x: interpolating warp (affine base + TPS residual). The
        # corrected plate passes exactly through every control point and
        # interpolates the leftover deltas smoothly between them, replacing
        # the single global similarity fit. Degrades automatically:
        # 2 pts \u2192 similarity, 3 pts \u2192 affine, \u22654 pts \u2192 affine + TPS warp.
        try:
            from SupportClasses.PlateWarpCalibrator import PlateWarpCalibrator
            warp = PlateWarpCalibrator()
            for (px, py), (mx, my) in zip(pred_pts, meas_pts):
                warp.add_point(px, py, mx, my)
            warp.solve()
            # v7.5.x: reject a degenerate fit before committing it. A manual
            # fit from 3 close-together or noisy clicks can extrapolate distant
            # wells far off the plate; navigating to such a well then clamps to
            # an envelope corner and drives the stage to a wrong extreme. Keep
            # the previous (predicted) positions and ask the user to re-teach.
            ok_fit, why = self._warp_is_plausible(warp)
            if not ok_fit:
                logger.warning(f"Manual XY warp rejected: {why}")
                if hasattr(self, '_gen_status'):
                    self._gen_status.setText(
                        f"⚠ Calibration rejected: {why}. Re-teach using "
                        f"3+ well-separated wells, clicking on each well's rim.")
                    self._gen_status.setStyleSheet(
                        f"color: {COLORS['red']}; font-size: 9pt;")
                return
            self._plate_warp = warp
            self._calibrated_positions = warp.correct_positions(
                self._predicted_positions)
            # Mirror the warp's linear part into an AffineCalibration so the
            # legacy display labels + mosaic_affine persistence keep working
            # (approximate \u2014 drops shear / the TPS term; the authoritative
            # positions come from the warp, restored from plate_warp on load).
            try:
                from SupportClasses.MosaicBuilder import AffineCalibration
                rot, scl, (tx, ty) = warp.similarity_approx()
                self._three_well_calibration = AffineCalibration(
                    rotation_deg=rot,
                    scale=scl,
                    translation_um=(tx, ty),
                    center_um=(0.0, 0.0),
                    num_points=warp.n_points,
                    residual_um=warp.rms_error_um,
                )
            except Exception:
                pass
            if hasattr(self, '_cal_plate_view'):
                self._cal_plate_view.set_calibrated_positions(
                    self._calibrated_positions)
            n_cal = len(self._calibrated_positions)
            mode_txt = {
                "translation": "shift",
                "similarity": "similarity",
                "affine": "affine",
                "affine_tps": "interpolated warp",
            }.get(warp.mode, warp.mode)
            if hasattr(self, '_gen_status'):
                self._gen_status.setText(
                    f"\u2705 Calibrated {n_cal} wells from {n} points "
                    f"({mode_txt}) \u2014 avg \u0394 "
                    f"{warp.mean_correction_um:.0f} \u00b5m, "
                    f"max \u0394 {warp.max_correction_um:.0f} \u00b5m")
                self._gen_status.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: 9pt;")
            logger.info(
                f"Manual XY warp: {n_cal} wells from {n} points "
                f"(mode={warp.mode}, mean\u0394={warp.mean_correction_um:.1f}\u00b5m, "
                f"max\u0394={warp.max_correction_um:.1f}\u00b5m)")
            self._save_calibration()  # auto-persist calibrated positions + warp
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
        """Handle Z jog from embedded jog array.

        v7.5.x: dz_mm is a HEIGHT-frame delta (+ = up); route through
        move_z_user_relative so jogging down to teach a well bottom follows the
        taught z_up_sign (the Z▼ button really lowers the needle).
        """
        if self.controller.is_zp_connected:
            self.controller.move_z_user_relative(dz_mm)

    def _jog_xy_home(self):
        """Move to the zero reference (XY origin, Z=0).

        v7.5.x CRITICAL SAFETY: retract Z to the safe height and WAIT before the
        XY traverse so a lowered needle is never dragged to the origin (the old
        code moved XY first, then Z). Routes through ``_safe_navigate_to``, whose
        internal ``_safe_z or 0.0`` fallback keeps the uncalibrated case safe
        (raw Z=0 is the retracted reference on this machine).
        """
        if self.controller.is_xy_connected:
            zero = self.controller.zero_position
            self._safe_navigate_to(zero.get("x", 0), zero.get("y", 0),
                                   target_z_mm=0.0)
        elif self.controller.is_zp_connected:
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
        # v7.4.2 hotfix: read Z via logical axis.
        z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        if z_val is None:
            return
        self._safe_z = z_val - self.controller.zero_position.get("Z", 0)
        if hasattr(self, 'lbl_safe_z'):
            self.lbl_safe_z.setText(f"Safe Z: {self._safe_z:.2f} mm")
            self.lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Safe Z set: {self._safe_z:.2f} mm")
        self._emit_calibration_data_changed()

    def _set_top_z(self):
        """v7.2.7: Record current Z as plate top surface."""
        zp = self.controller.get_zp_position(cached=False)
        # v7.4.2 hotfix: read Z via logical axis.
        z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        if z_val is None:
            return
        self._top_z = z_val - self.controller.zero_position.get("Z", 0)
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
        # v7.4.2 hotfix: route Z read through axis_map.
        a1_z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        if a1_z_val is not None:
            self._taught_a1_z = a1_z_val - self.controller.zero_position.get("Z", 0)
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
        # v7.4.2 hotfix: route Z read through axis_map.
        corner_z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        if corner_z_val is not None:
            self._taught_corner_z = corner_z_val - self.controller.zero_position.get("Z", 0)
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
        # v7.4.2 hotfix: route Z read through axis_map.
        third_z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        if third_z_val is not None:
            self._taught_third_z = third_z_val - self.controller.zero_position.get("Z", 0)
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
        # v7.5.x: re-selecting the *current* plate is a no-op. Without this
        # guard a programmatic/duplicate combo change (e.g. reflecting the
        # loaded plate format back onto the combo at startup) would clear a
        # just-loaded calibration, and the debounced auto-save would then
        # persist the nulls over good data. A genuine format change still
        # clears below (different plate ⇒ recalibrate).
        if self._plate is not None and getattr(self._plate, 'format', None) == fmt:
            return
        # v7.5.x: flush the OUTGOING plate's calibration to its archive while
        # self._plate is still the old plate, then (after the rebuild + clears
        # below) restore the incoming plate's archived calibration.
        _old_cal_key = self._loaded_cal_key
        if (_old_cal_key is not None and str(fmt) != str(_old_cal_key)
                and self.settings is not None):
            try:
                self._save_calibration(key=_old_cal_key)
            except Exception as e:
                logger.debug(
                    f"flush outgoing calibration ({_old_cal_key}) skipped: {e}")
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
        self._plate_warp = None        # v7.5.x: clear freeform interpolating warp
        self._reference_markers = {}   # v7.5.x: stale on a format change
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

        # v7.5.x: a different plate ⇒ a different stored mosaic. Load the new
        # plate's overlay (or clear it when none is stored).
        try:
            self._ploc_load_persisted_mosaic()
        except Exception as e:
            logger.debug(f"PlateLocation: mosaic reload on plate change: {e}")
        self._ploc_refresh_reregister_button()

        # v7.5.x: restore this plate's archived calibration (taught wells / warp
        # / Z references) — or stay blank when it has none — so calibration
        # follows the plate instead of being cleared on every plate change.
        try:
            self._activate_calibration_for_key(str(fmt))
        except Exception as e:
            logger.debug(f"activate calibration ({fmt}) skipped: {e}")


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
        # v7.4.2 hotfix: route Z read through axis_map.
        a1_z_alt = self.controller.zp_logical_value(zp, "Z") if zp else None
        if a1_z_alt is not None:
            self._taught_a1_z = a1_z_alt - self.controller.zero_position.get("Z", 0)

        logger.info(f"Taught A1: {self._taught_a1}")
        # v7.3.1: Compute geometry-predicted positions for all wells
        self._compute_predicted_positions()

    def _goto_a1(self):
        """v7.2.7: Navigate to taught A1 (absolute µm).

        v7.5.x CRITICAL SAFETY: route through ``_safe_navigate_to`` so Z
        retracts to the safe height and is confirmed there before the XY
        traverse (the old bare ``move_xy_absolute`` dragged a lowered needle
        across the plate). ``lower_z=False`` keeps the needle at safe Z.
        """
        if self._taught_a1:
            self._safe_navigate_to(
                self._taught_a1[0], self._taught_a1[1], lower_z=False)


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

        # v7.4.2 hotfix: route Z read through axis_map.
        corner_z_alt = self.controller.zp_logical_value(zp, "Z") if zp else None
        if corner_z_alt is not None:
            self._taught_corner_z = corner_z_alt - self.controller.zero_position.get("Z", 0)

        logger.info(f"Taught corner: {self._taught_corner}")


    def _goto_corner(self):
        """v7.2.7: Navigate to taught corner (absolute µm).

        v7.5.x CRITICAL SAFETY: route through ``_safe_navigate_to`` (retract Z
        + wait before XY) — the corner is the longest cross-plate traverse, so
        a lowered needle dragged here is the worst case. ``lower_z=False``
        keeps the needle at safe Z.
        """
        if self._taught_corner:
            self._safe_navigate_to(
                self._taught_corner[0], self._taught_corner[1], lower_z=False)


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
            # Map the plate-local offset onto stage axes (per-machine sign).
            _sx, _sy = self._plate_axis_sign()
            dx_mm = _sx * (pos[0] - a1_expected[0])
            dy_mm = _sy * (pos[1] - a1_expected[1])
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

        # v7.5.x CRITICAL SAFETY: navigate XY at safe Z, then lower to target Z.
        # ALWAYS route through _safe_navigate_to (retract to safe Z + wait before
        # XY); its internal `_safe_z or 0.0` fallback keeps the uncalibrated case
        # safe (raw Z=0 is the retracted reference) instead of the old bare move
        # that dragged a lowered needle across the plate.
        self._safe_navigate_to(target_x_um, target_y_um, target_z)

        # Display
        rel_x = target_x_um - self.controller.zero_position["x"]
        rel_y = target_y_um - self.controller.zero_position["y"]
        z_str = ""
        if target_z is not None:
            z_str = f"  Z={target_z:.3f}mm ({z_source})"
        if hasattr(self, 'lbl_val_result'):
            self.lbl_val_result.setText(
                f"Moving to {well} \u2192 ({rel_x:,.1f}, {rel_y:,.1f}) \u00b5m{z_str}")


    def _save_calibration(self, key=None):
        """v7.2.7: save new fields — safe_z, top_z, Z plane, third point.

        v7.5.x: the flat ``calibration`` section mirrors the ACTIVE plate's
        block (kept for every legacy flat reader), and the block is ALSO
        archived under its plate identity in ``calibration_by_plate`` so the
        calibration follows the plate. ``key`` overrides the archive key — used
        to flush the OUTGOING plate before a switch (its in-memory state is
        still current, but the active key has already moved on)."""
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
            # v7.4.4 additions — Needle Offset Calibration heights.
            "replace_z": getattr(self, '_replace_z', None),
            "max_z": getattr(self, '_max_z', None),
            "plate_bottom_z": getattr(self, '_plate_bottom_z', None),
            # v7.5.x: stamp the plate orientation the taught/warped positions
            # were captured under. A saved warp's predicted-leg uses the
            # geometric sign; if the machine orientation later changes, applying
            # the old warp would drive to garbage — so on load we invalidate it
            # and force a re-teach when this stamp ≠ the current orientation.
            "plate_flip_180": self._orientation_stamp(),
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
        # v7.5.x: save the freeform interpolating warp (affine + TPS residual)
        # as raw control-point pairs — re-solved on load so it reproduces the
        # exact-at-control-points correction (the mosaic_affine above is only
        # an approximate similarity fallback for older/downgraded builds).
        warp = getattr(self, '_plate_warp', None)
        if warp is not None and warp.n_points >= 2:
            cal_data["plate_warp"] = warp.to_dict()
        # v7.5.x: persist EXPLICIT calibrated well positions (name→absolute
        # stage µm) when they were set DIRECTLY with NO warp — i.e. by the
        # "Re-derive wells from saved mosaic (ground truth)" action. These are
        # the measured ground-truth centres; on load they ARE the calibration
        # and suppress any warp/affine reconstruction. (Only written when no
        # warp exists, so the normal taught/warp flow is byte-identical.)
        if self._calibrated_positions and warp is None:
            cal_data["calibrated_positions"] = {
                n: [float(x), float(y)]
                for n, (x, y) in self._calibrated_positions.items()
            }
        # v7.5.x: persist the taught reference well centres (absolute stage µm)
        # so the registration markers reappear on the plate + microscope views
        # after a restart.
        if self._reference_markers:
            cal_data["reference_markers"] = {
                n: [x, y] for n, (x, y) in self._reference_markers.items()
            }
        # Save Z plane coefficients if fitted
        zp = getattr(self, '_z_plane_result', None)
        if zp is not None:
            cal_data["z_plane"] = {
                "a": zp.a, "b": zp.b, "c": zp.c,
                "r_squared": zp.r_squared,
            }
        # v7.5.x: the flat section mirrors the ACTIVE plate (legacy readers);
        # when flushing the OUTGOING plate (explicit key ≠ active), don't clobber
        # the active flat block — only archive under the given key.
        archive_key = str(key) if key is not None else self._calibration_key()
        active_key = self._calibration_key()
        if key is None or str(key) == str(active_key):
            self.settings.set_section("calibration", cal_data)
        # v7.5.x: archive the block under its plate identity so the calibration
        # follows the plate (each plate TYPE keeps its own taught map).
        if archive_key is not None:
            archive = self._calibration_archive()
            archive[archive_key] = cal_data
            self.settings.set_section("calibration_by_plate", archive)
        self.settings.save()
        logger.info("Calibration saved to settings (v7.2.7) [key=%s]",
                    archive_key)
        # v7.5.x: refresh the durable last-known-good snapshot (needle zero +
        # plate + Z + hardware fingerprint) so a future restart can offer to
        # restore the whole calibration as a unit. Gated on a real plate
        # calibration so an empty/cleared page never clobbers a good snapshot.
        if self._has_plate_calibration() and self.controller is not None:
            try:
                from SupportClasses.CalibrationSnapshotStore import (
                    get_store as _get_cal_snapshot_store)
                _snap_store = _get_cal_snapshot_store()
                _snap_store.save_snapshot(
                    zero_position=dict(self.controller.zero_position),
                    calibration=cal_data,
                    fingerprint=_snap_store.build_fingerprint(self.settings),
                )
            except Exception as e:
                logger.debug(f"calibration snapshot save skipped: {e}")
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
        # Work on a copy so the orientation guard below can drop stale keys
        # without mutating the in-memory settings section.
        cal = dict(cal)

        # v7.5.x: orientation guard. The taught A1 / corner / warp / reference
        # markers were captured under a specific plate orientation; the warp's
        # predicted-leg uses the geometric axis sign. If the machine's
        # orientation changed since this calibration was saved, applying the
        # stale XY calibration would drive to a mirrored (garbage) position —
        # so drop the orientation-dependent XY keys and force a re-teach.
        # Orientation-INDEPENDENT data (Z heights, plate format, Z plane) is
        # kept. A missing stamp (older calibration) is treated as "unknown" and
        # not invalidated, to avoid nagging on upgrade.
        _stored_flip = cal.get("plate_flip_180")
        _cur_flip = (self.controller.plate_flip_180()
                     if (self.controller is not None
                         and hasattr(self.controller, "plate_flip_180"))
                     else None)
        if (_stored_flip is not None and _cur_flip is not None
                and bool(_stored_flip) != bool(_cur_flip)):
            logger.warning(
                "Calibration orientation stamp (plate_flip_180=%s) differs "
                "from the current machine (%s) — discarding the stale XY "
                "calibration (taught wells / warp); please re-teach the plate.",
                _stored_flip, _cur_flip)
            for _k in ("taught_a1", "taught_corner", "taught_third",
                       "mosaic_affine", "plate_warp", "reference_markers",
                       "calibrated_positions"):
                cal.pop(_k, None)
            if hasattr(self, 'ctx_lbl_cal_status'):
                self.ctx_lbl_cal_status.setText(
                    "⚠ Plate orientation changed — re-teach the plate")
                self.ctx_lbl_cal_status.setStyleSheet(
                    f"color: {COLORS['yellow']};")

        if cal.get("plate_format"):
            try:
                # v7.5.x: honor composed geometry — a rosette/custom design
                # layered under a plate TYPE must NOT be reset to the archived
                # base-format int (that would drop the sub-wells that
                # set_hardware_config just loaded). Prefer the live config's
                # geometry key; fall back to the archived plate_format on any
                # load failure.
                _geom_key = getattr(
                    getattr(self, "_hardware_config", None),
                    "geometry_plate_key", None)
                try:
                    self._plate = (
                        WellPlate.load(_geom_key) if _geom_key is not None
                        else WellPlate.from_format(cal["plate_format"]))
                except Exception:
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
        # v7.5.x: references are stored zero-ref; display in the unified user
        # frame (0 at bottom datum, up = +) via _zoff_user_z.
        if hasattr(self, '_safe_z'):
            self._safe_z = cal.get("safe_z")
            if self._safe_z is not None and hasattr(self, 'lbl_safe_z'):
                self.lbl_safe_z.setText(
                    f"Safe Z: {self._zoff_user_z(self._safe_z):.2f} mm")
                self.lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")
            # v7.4.4: mirror onto the Needle Offset tab label.
            if self._safe_z is not None and hasattr(self, '_zoff_lbl_safe_z'):
                self._zoff_lbl_safe_z.setText(
                    f"Fast Move Z: {self._zoff_user_z(self._safe_z):.2f} mm")
                self._zoff_lbl_safe_z.setStyleSheet(
                    f"color: {COLORS['green']};")

        if hasattr(self, '_top_z'):
            self._top_z = cal.get("top_z")
            if self._top_z is not None and hasattr(self, 'lbl_top_z'):
                self.lbl_top_z.setText(
                    f"Top Z: {self._zoff_user_z(self._top_z):.2f} mm")
                self.lbl_top_z.setStyleSheet(f"color: {COLORS['green']};")
            if self._top_z is not None and hasattr(self, '_zoff_lbl_top_z'):
                self._zoff_lbl_top_z.setText(
                    f"Plate Top Z: {self._zoff_user_z(self._top_z):.2f} mm")
                self._zoff_lbl_top_z.setStyleSheet(
                    f"color: {COLORS['green']};")

        # v7.4.4: new Z reference heights.
        for key, attr, lbl_attr, prefix in [
            ("replace_z", "_replace_z", "_zoff_lbl_replace_z", "Replace Z"),
            ("max_z", "_max_z", "_zoff_lbl_max_z", "Max Z"),
            ("plate_bottom_z", "_plate_bottom_z",
             "_zoff_lbl_plate_bottom_z", "Plate Bottom Z"),
        ]:
            val = cal.get(key)
            setattr(self, attr, val)
            lbl = getattr(self, lbl_attr, None)
            if val is not None and lbl is not None:
                lbl.setText(f"{prefix}: {self._zoff_user_z(val):.2f} mm")
                lbl.setStyleSheet(f"color: {COLORS['green']};")

        # v7.5.x: the Z soft-limit envelope is owned by Hardware Setup →
        # Device (settings ``safety_limits.z_min/z_max``, loaded onto the
        # live controller in main.py), NOT by the calibrated Max Z / Plate
        # Bottom Z. The old v7.4.4 restore pushed those into the envelope on
        # every startup, which clobbered the user's device-setup range and —
        # on machines where the needle descends as Z *increases* (Max Z
        # numerically below Plate Bottom Z) — inverted the envelope so
        # clamp_z() collapsed to a single point and every jog was clamped to
        # it. The captured heights remain available as print/Z references via
        # self._max_z / self._plate_bottom_z.

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

        # v7.5.x: restore the taught reference well centres (absolute stage µm)
        # so the registration markers reappear on the plate + microscope views.
        ref = cal.get("reference_markers")
        if isinstance(ref, dict):
            restored: dict[str, tuple[float, float]] = {}
            for n, xy in ref.items():
                try:
                    restored[str(n)] = (float(xy[0]), float(xy[1]))
                except (TypeError, ValueError, IndexError):
                    continue
            self._reference_markers = restored

        # v7.5.x: EXPLICIT calibrated well positions (name→absolute stage µm),
        # stored directly with NO warp by the "Re-derive wells from saved
        # mosaic (ground truth)" action. When present these ARE the calibration
        # — they are the measured ground-truth centres and need no warp/affine,
        # so they take precedence over and SUPPRESS the warp/affine
        # reconstruction below (which would otherwise re-introduce a stale
        # correction). Dropped by the orientation guard above on a flip.
        explicit_cp = cal.get("calibrated_positions")
        if isinstance(explicit_cp, dict) and explicit_cp:
            restored_cp: dict[str, tuple[float, float]] = {}
            for n, xy in explicit_cp.items():
                try:
                    restored_cp[str(n)] = (float(xy[0]), float(xy[1]))
                except (TypeError, ValueError, IndexError):
                    continue
            if restored_cp:
                self._calibrated_positions = restored_cp
                self._plate_warp = None
                self._three_well_calibration = None
                if (hasattr(self, '_cal_plate_view')
                        and self._cal_plate_view is not None):
                    self._cal_plate_view.set_calibrated_positions(restored_cp)
                logger.info(
                    "Loaded %d explicit calibrated well positions "
                    "(ground-truth mosaic re-derive; no warp).",
                    len(restored_cp))

        # v7.5.x: Restore the freeform interpolating warp (preferred over the
        # approximate mosaic_affine similarity below). Re-solves from the
        # stored control-point pairs so the exact-at-control-points correction
        # is reproduced rather than degrading to a similarity on restart.
        # Skipped when explicit calibrated_positions were restored above.
        warp_data = cal.get("plate_warp")
        if warp_data and self._predicted_positions and not self._calibrated_positions:
            try:
                from SupportClasses.PlateWarpCalibrator import PlateWarpCalibrator
                from SupportClasses.MosaicBuilder import AffineCalibration
                warp = PlateWarpCalibrator.from_dict(warp_data)
                if warp.n_points >= 2:
                    self._plate_warp = warp
                    self._calibrated_positions = warp.correct_positions(
                        self._predicted_positions)
                    rot, scl, (tx, ty) = warp.similarity_approx()
                    self._three_well_calibration = AffineCalibration(
                        rotation_deg=rot, scale=scl, translation_um=(tx, ty),
                        center_um=(0.0, 0.0), num_points=warp.n_points,
                        residual_um=warp.rms_error_um)
                    if (hasattr(self, '_cal_plate_view')
                            and self._cal_plate_view is not None):
                        self._cal_plate_view.set_calibrated_positions(
                            self._calibrated_positions)
                    if hasattr(self, '_lbl_scan_progress'):
                        self._lbl_scan_progress.setText(
                            f"✅ Loaded warp: {len(self._calibrated_positions)} "
                            f"wells ({warp.mode})")
                        self._lbl_scan_progress.setStyleSheet(
                            f"color: {COLORS['green']}; font-size: 9pt;")
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
                    logger.info(
                        f"Plate warp restored: mode={warp.mode}, "
                        f"{warp.n_points} points, "
                        f"meanΔ={warp.mean_correction_um:.1f}µm")
            except Exception as e:
                logger.debug(f"Could not restore plate warp: {e}")

        # v7.3.1: Restore mosaic affine calibration and recompute calibrated
        # positions — only when no freeform warp was restored above.
        affine_data = cal.get("mosaic_affine")
        if (self._plate_warp is None and affine_data
                and self._predicted_positions
                and not self._calibrated_positions):
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

    def _shutdown_mosaic_worker(self) -> None:
        """v7.5.x: stop + join the mosaic scan thread (for cleanup/close).

        Called from this page's closeEvent AND from MainWindow.closeEvent's
        per-page loop (alongside _shutdown_detection_worker) so closing the app
        mid-scan can't leave a running QThread that emits into a torn-down page
        or touches the controller during shutdown.
        """
        self._ploc_mosaic_running = False
        worker = getattr(self, "_ploc_mosaic_worker", None)
        if worker is not None:
            for sig in (worker.tile, worker.progress,
                        worker.finished_ok, worker.failed):
                try:
                    sig.disconnect()
                except Exception:
                    pass
            try:
                worker.stop()
                if worker.isRunning():
                    worker.wait(4000)
            except Exception:
                pass
            self._ploc_mosaic_worker = None

    def closeEvent(self, event):
        """Ensure background threads are stopped before destruction."""
        self._shutdown_detection_worker()
        self._shutdown_mosaic_worker()
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
        # v7.4.2 hotfix: route Z read through axis_map.
        zp = self.controller.get_zp_position(cached=True)
        z_val = self.controller.zp_logical_value(zp, "Z") if zp else None
        current_z = None
        if z_val is not None:
            current_z = z_val - self.controller.zero_position.get("Z", 0)

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

