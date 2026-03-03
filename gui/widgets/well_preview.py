"""
well_preview.py — XY-only well preview widget with zoom, pan, and bounds checking.

MEBP v7.2.4 Session 4: Replaces L-shaped ProjectionCanvas in the Print Objects
tab with a single, large, zoomable/pannable XY top-down well view.

Features:
  - Mouse-wheel zoom with configurable limits
  - Middle-click or Ctrl+click pan
  - Zoom-to-fit button and zoom percentage indicator
  - Well boundary circle drawn at correct diameter from HardwareConfig
  - Object trajectory rendering with per-object colors
  - Out-of-bounds detection: OOB segments drawn in red/dashed
  - Axis alignment: X=right, Y=up (standard Cartesian)
  - Grid lines and mm scale ruler
  - Lightweight QGraphicsView-based (no 3-view overhead)

Used by: gui/pages/print_objects.py (Tab 2)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

from PySide6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsPathItem, QGraphicsTextItem, QGraphicsLineItem,
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSizePolicy, QToolBar,
)
from PySide6.QtCore import Qt, QRectF, QPointF, Signal
from PySide6.QtGui import (
    QColor, QPen, QBrush, QPainter, QFont, QPainterPath,
    QWheelEvent, QMouseEvent, QKeyEvent,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

SCENE_SCALE = 20.0            # mm → scene pixels
MIN_ZOOM = 0.1
MAX_ZOOM = 10.0
ZOOM_STEP = 1.15

# Colors (Catppuccin Mocha)
BG_COLOR = "#181825"          # Mantle
WELL_BOUNDARY_COLOR = "#585b70"  # Surface2
GRID_COLOR = "#313244"        # Surface0
AXIS_COLOR = "#45475a"        # Surface2
RULER_COLOR = "#6c7086"       # Overlay0
OOB_COLOR = "#f38ba8"         # Red — out of bounds
OOB_DASH_PATTERN = [6, 4]
DEFAULT_PATH_COLOR = "#a6e3a1"  # Green
HIGHLIGHT_COLOR = "#ffffff"
WELL_FILL_ALPHA = 15
GRID_ALPHA = 50


# ═══════════════════════════════════════════════════════════════════
# Data containers
# ═══════════════════════════════════════════════════════════════════

@dataclass
class ObjectPath:
    """A named, colored path representing one print object's XY trajectory."""
    name: str
    color: str = DEFAULT_PATH_COLOR
    points: list[tuple[float, float, float]] = field(default_factory=list)
    is_oob: bool = False  # Set by bounds checker

    def clear(self):
        self.points.clear()


# ═══════════════════════════════════════════════════════════════════
# WellPreviewScene — Custom scene with grid drawing
# ═══════════════════════════════════════════════════════════════════

class WellPreviewScene(QGraphicsScene):
    """Scene that draws a mm grid in the background."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundBrush(QBrush(QColor(BG_COLOR)))
        self._grid_spacing_mm = 1.0  # 1mm grid
        self._extent_mm = 10.0       # Grid extent each direction

    def set_extent(self, extent_mm: float):
        self._extent_mm = extent_mm

    def drawBackground(self, painter: QPainter, rect: QRectF):
        """Draw mm grid behind all items."""
        super().drawBackground(painter, rect)
        painter.save()

        # Grid
        grid_pen = QPen(QColor(GRID_COLOR))
        grid_pen.setWidthF(0.5)
        painter.setPen(grid_pen)

        spacing = self._grid_spacing_mm * SCENE_SCALE
        extent = self._extent_mm * SCENE_SCALE

        # Vertical lines
        x = -extent
        while x <= extent:
            painter.drawLine(QPointF(x, -extent), QPointF(x, extent))
            x += spacing

        # Horizontal lines
        y = -extent
        while y <= extent:
            painter.drawLine(QPointF(-extent, y), QPointF(extent, y))
            y += spacing

        # Axes (thicker)
        axis_pen = QPen(QColor(AXIS_COLOR))
        axis_pen.setWidthF(1.5)
        painter.setPen(axis_pen)
        painter.drawLine(QPointF(-extent, 0), QPointF(extent, 0))  # X axis
        painter.drawLine(QPointF(0, -extent), QPointF(0, extent))  # Y axis

        # Axis labels
        font = QFont("Consolas", 7)
        painter.setFont(font)
        painter.setPen(QColor(RULER_COLOR))
        painter.drawText(QPointF(extent - 20, 12), "X (mm)")
        # Y label (remember Y is flipped in scene: up = negative)
        painter.drawText(QPointF(5, -extent + 12), "Y (mm)")

        # Ruler tick labels every mm
        for i in range(-int(self._extent_mm), int(self._extent_mm) + 1):
            if i == 0:
                continue
            sx = i * SCENE_SCALE
            # X axis ticks
            painter.drawText(QPointF(sx - 5, 12), str(i))
            # Y axis ticks (scene Y is inverted)
            painter.drawText(QPointF(5, -sx + 4), str(i))

        painter.restore()


# ═══════════════════════════════════════════════════════════════════
# WellPreviewView — QGraphicsView with zoom + pan
# ═══════════════════════════════════════════════════════════════════

class WellPreviewView(QGraphicsView):
    """
    Zoomable, pannable QGraphicsView for XY well preview.

    Zoom: mouse wheel
    Pan: middle-click drag OR Ctrl+left-click drag
    Coordinates: X=right, Y=up (scene Y is inverted)
    """

    zoom_changed = Signal(float)  # Emits current zoom percentage

    def __init__(self, scene: WellPreviewScene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHints(
            QPainter.RenderHint.Antialiasing
            | QPainter.RenderHint.SmoothPixmapTransform
        )
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setStyleSheet(f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumHeight(200)

        self._zoom_factor = 1.0
        self._panning = False
        self._pan_start = QPointF()

    # ── Zoom ──────────────────────────────────────────────────────

    def wheelEvent(self, event: QWheelEvent):
        if event.angleDelta().y() > 0:
            factor = ZOOM_STEP
        else:
            factor = 1.0 / ZOOM_STEP

        new_zoom = self._zoom_factor * factor
        if MIN_ZOOM <= new_zoom <= MAX_ZOOM:
            self._zoom_factor = new_zoom
            self.scale(factor, factor)
            self.zoom_changed.emit(self._zoom_factor * 100.0)

    def get_zoom_percent(self) -> float:
        return self._zoom_factor * 100.0

    def set_zoom(self, percent: float):
        target = max(MIN_ZOOM, min(MAX_ZOOM, percent / 100.0))
        factor = target / self._zoom_factor
        self._zoom_factor = target
        self.scale(factor, factor)
        self.zoom_changed.emit(self._zoom_factor * 100.0)

    def zoom_to_fit(self):
        """Fit the scene contents in the viewport."""
        self.resetTransform()
        self._zoom_factor = 1.0
        items_rect = self.scene().itemsBoundingRect()
        if items_rect.isNull():
            items_rect = QRectF(-100, -100, 200, 200)
        items_rect.adjust(-20, -20, 20, 20)
        self.fitInView(items_rect, Qt.AspectRatioMode.KeepAspectRatio)
        # Estimate new zoom factor from transform
        t = self.transform()
        self._zoom_factor = t.m11()
        self.zoom_changed.emit(self._zoom_factor * 100.0)

    # ── Pan ───────────────────────────────────────────────────────

    def mousePressEvent(self, event: QMouseEvent):
        if (event.button() == Qt.MouseButton.MiddleButton
                or (event.button() == Qt.MouseButton.LeftButton
                    and event.modifiers() & Qt.KeyboardModifier.ControlModifier)):
            self._panning = True
            self._pan_start = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._panning:
            delta = event.position() - self._pan_start
            self._pan_start = event.position()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - int(delta.x()))
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - int(delta.y()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if self._panning:
            self._panning = False
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    # ── Resize ────────────────────────────────────────────────────

    def resizeEvent(self, event):
        super().resizeEvent(event)


# ═══════════════════════════════════════════════════════════════════
# WellPreviewWidget — Complete preview with toolbar + view
# ═══════════════════════════════════════════════════════════════════

class WellPreviewWidget(QWidget):
    """
    Composite widget: toolbar (zoom controls) + WellPreviewView.

    Public API:
        set_well_diameter(mm)     — draw well boundary circle
        set_object_paths(paths)   — render trajectory paths
        clear_object_paths()      — remove all paths
        check_bounds(objects)     — returns list of OOB object indices
        set_oob_indices(indices)  — mark specific objects as OOB
        zoom_to_fit()             — fit view
        refresh()                 — redraw
    """

    # Signal emitted with list of OOB object indices after bounds check
    oob_detected = Signal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self._well_diameter_mm = 6.0  # Default 96-well plate
        self._object_paths: list[ObjectPath] = []
        self._oob_indices: set[int] = set()
        self._highlight_index: int | None = None

        # Scene items (for cleanup)
        self._well_circle_item: QGraphicsEllipseItem | None = None
        self._path_items: list[QGraphicsPathItem] = []

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # ── Toolbar ───────────────────────────────────────────────
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(4, 2, 4, 2)
        toolbar.setSpacing(4)

        btn_style = (
            f"QPushButton {{ background: {COLORS['surface0']}; color: {COLORS['text']}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 3px; "
            f"padding: 2px 8px; font-size: 11px; }}"
            f"QPushButton:hover {{ background: {COLORS['surface1']}; }}"
        )

        self._btn_fit = QPushButton("Fit")
        self._btn_fit.setStyleSheet(btn_style)
        self._btn_fit.setToolTip("Zoom to fit all objects")
        self._btn_fit.clicked.connect(self.zoom_to_fit)
        toolbar.addWidget(self._btn_fit)

        self._btn_zoom_in = QPushButton("+")
        self._btn_zoom_in.setStyleSheet(btn_style)
        self._btn_zoom_in.setFixedWidth(28)
        self._btn_zoom_in.clicked.connect(lambda: self._view.set_zoom(
            self._view.get_zoom_percent() * ZOOM_STEP))
        toolbar.addWidget(self._btn_zoom_in)

        self._btn_zoom_out = QPushButton("−")
        self._btn_zoom_out.setStyleSheet(btn_style)
        self._btn_zoom_out.setFixedWidth(28)
        self._btn_zoom_out.clicked.connect(lambda: self._view.set_zoom(
            self._view.get_zoom_percent() / ZOOM_STEP))
        toolbar.addWidget(self._btn_zoom_out)

        self._zoom_label = QLabel("100%")
        self._zoom_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px; min-width: 40px;")
        toolbar.addWidget(self._zoom_label)

        toolbar.addStretch()

        self._well_info_label = QLabel("")
        self._well_info_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10px;")
        toolbar.addWidget(self._well_info_label)

        layout.addLayout(toolbar)

        # ── Graphics View ─────────────────────────────────────────
        self._scene = WellPreviewScene()
        self._view = WellPreviewView(self._scene)
        self._view.zoom_changed.connect(self._on_zoom_changed)
        layout.addWidget(self._view, stretch=1)

    # ══════════════════════════════════════════════════════════════
    #  PUBLIC API
    # ══════════════════════════════════════════════════════════════

    def set_well_diameter(self, diameter_mm: float):
        """Set well boundary circle diameter (from plate format)."""
        self._well_diameter_mm = max(0.1, diameter_mm)
        self._scene.set_extent(diameter_mm * 0.8)
        self._well_info_label.setText(f"Well Ø {diameter_mm:.1f} mm")
        self._rebuild_scene()

    def set_object_paths(self, paths: list[ObjectPath]):
        """Set all object trajectory paths and refresh."""
        self._object_paths = list(paths)
        self._run_bounds_check()
        self._rebuild_scene()

    def clear_object_paths(self):
        """Remove all object paths."""
        self._object_paths.clear()
        self._oob_indices.clear()
        self._rebuild_scene()

    def set_highlight(self, index: int | None):
        """Highlight a specific object index (white border)."""
        self._highlight_index = index
        self._rebuild_scene()

    def set_oob_indices(self, indices: set[int]):
        """Externally set OOB indices (used by flash timer)."""
        self._oob_indices = set(indices)
        self._rebuild_scene()

    def check_bounds(self) -> list[int]:
        """Check all objects against well boundary, return OOB indices."""
        self._run_bounds_check()
        return list(self._oob_indices)

    def zoom_to_fit(self):
        self._view.zoom_to_fit()

    def refresh(self):
        self._rebuild_scene()

    # ══════════════════════════════════════════════════════════════
    #  BOUNDS CHECKING (S4.9)
    # ══════════════════════════════════════════════════════════════

    def _run_bounds_check(self):
        """Check each object path against well radius."""
        well_radius = self._well_diameter_mm / 2.0
        oob = set()
        for i, path in enumerate(self._object_paths):
            for px, py, *_ in path.points:
                dist = math.sqrt(px * px + py * py)
                if dist > well_radius:
                    oob.add(i)
                    path.is_oob = True
                    break
            else:
                path.is_oob = False
        self._oob_indices = oob
        if oob:
            self.oob_detected.emit(list(oob))

    # ══════════════════════════════════════════════════════════════
    #  SCENE REBUILD
    # ══════════════════════════════════════════════════════════════

    def _rebuild_scene(self):
        """Clear and redraw all items in the scene."""
        # Remove old items (keep background)
        for item in self._path_items:
            self._scene.removeItem(item)
        self._path_items.clear()
        if self._well_circle_item:
            self._scene.removeItem(self._well_circle_item)
            self._well_circle_item = None

        # ── Well boundary circle ──────────────────────────────────
        r_scene = (self._well_diameter_mm / 2.0) * SCENE_SCALE
        self._well_circle_item = QGraphicsEllipseItem(
            -r_scene, -r_scene, 2 * r_scene, 2 * r_scene)

        well_pen = QPen(QColor(WELL_BOUNDARY_COLOR))
        well_pen.setWidthF(2.0)
        self._well_circle_item.setPen(well_pen)

        fill = QColor(WELL_BOUNDARY_COLOR)
        fill.setAlpha(WELL_FILL_ALPHA)
        self._well_circle_item.setBrush(QBrush(fill))
        self._scene.addItem(self._well_circle_item)

        # ── Object paths ──────────────────────────────────────────
        well_radius = self._well_diameter_mm / 2.0

        for i, obj_path in enumerate(self._object_paths):
            if not obj_path.points:
                continue

            is_highlighted = (self._highlight_index is not None
                              and i == self._highlight_index)
            is_oob = i in self._oob_indices

            if is_oob:
                # Draw in-bounds segments normally, OOB segments in red/dashed
                self._draw_path_with_oob(obj_path, well_radius, is_highlighted)
            else:
                # Draw entire path in object color
                self._draw_simple_path(obj_path, is_highlighted)

        # Set scene rect with padding
        extent = (self._well_diameter_mm * 0.8) * SCENE_SCALE
        self._scene.setSceneRect(-extent, -extent, 2 * extent, 2 * extent)

    def _draw_simple_path(self, obj_path: ObjectPath, highlighted: bool):
        """Draw a single-color path (all in-bounds)."""
        color = HIGHLIGHT_COLOR if highlighted else obj_path.color
        pen = QPen(QColor(color))
        pen.setWidthF(2.0)

        pp = QPainterPath()
        first = True
        for px, py, *_ in obj_path.points:
            # Scene coords: X=right (same), Y needs inversion for up=positive
            sx = px * SCENE_SCALE
            sy = -py * SCENE_SCALE  # Invert Y for Cartesian
            if first:
                pp.moveTo(sx, sy)
                first = False
            else:
                pp.lineTo(sx, sy)

        item = QGraphicsPathItem(pp)
        item.setPen(pen)
        self._scene.addItem(item)
        self._path_items.append(item)

    def _draw_path_with_oob(
        self, obj_path: ObjectPath, well_radius: float, highlighted: bool,
    ):
        """Draw path with OOB segments in red/dashed (S4.11)."""
        normal_color = HIGHLIGHT_COLOR if highlighted else obj_path.color
        normal_pen = QPen(QColor(normal_color))
        normal_pen.setWidthF(2.0)

        oob_pen = QPen(QColor(OOB_COLOR))
        oob_pen.setWidthF(2.5)
        oob_pen.setDashPattern(OOB_DASH_PATTERN)

        # Build segments, switching pen when crossing boundary
        current_path = QPainterPath()
        current_is_oob = False
        first = True

        for px, py, *_ in obj_path.points:
            dist = math.sqrt(px * px + py * py)
            pt_oob = dist > well_radius

            sx = px * SCENE_SCALE
            sy = -py * SCENE_SCALE

            if first:
                current_path.moveTo(sx, sy)
                current_is_oob = pt_oob
                first = False
                continue

            if pt_oob != current_is_oob:
                # Flush current segment
                item = QGraphicsPathItem(current_path)
                item.setPen(oob_pen if current_is_oob else normal_pen)
                self._scene.addItem(item)
                self._path_items.append(item)

                # Start new segment from this point
                current_path = QPainterPath()
                current_path.moveTo(sx, sy)
                current_is_oob = pt_oob
            else:
                current_path.lineTo(sx, sy)

        # Flush final segment
        if not current_path.isEmpty():
            item = QGraphicsPathItem(current_path)
            item.setPen(oob_pen if current_is_oob else normal_pen)
            self._scene.addItem(item)
            self._path_items.append(item)

    # ══════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════════════════════════

    def _on_zoom_changed(self, percent: float):
        self._zoom_label.setText(f"{percent:.0f}%")
