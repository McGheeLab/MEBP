"""
projection_canvas.py — Reusable multi-projection canvas for MEBP v7.1.

Unified L-shaped triple-projection widget used across the application:
- Tab 2 (Print Objects): object preview with color-coded ink paths
- Tab 3 (Well Setup): plate-scale well bottom heights + needle overlay
- Print Monitor: real-time trajectory tracking with upcoming waypoints

Layout:
┌──────────────┬────────┐
│              │        │
│   XY (large) │ ZY     │
│              │ (tall) │
│              │        │
├──────────────┴────────┤
│   XZ (wide)           │
└───────────────────────┘

Each pane renders (as applicable):
- Well boundary (circle or line, from plate spec)
- Multi-object paths, each with its own color
- Well bottom height markers (plate-scale mode)
- Completed path (solid line) + upcoming waypoints (dashed)
- Needle crosshair with tracking error ring
- Axis label and grid (optional)

Two scale modes:
- "well"  — single well view, positions relative to well center (mm)
- "plate" — full plate view, positions in plate coordinates (mm)

Session D — Task P5.15.

Replaces / unifies:
- trajectory_view.py  → ProjectionView  (Monitor real-time tracking)
- print_well_setup.py → MiniProjectionView (Well Setup bottoms)
"""

from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass, field

from PySide6.QtWidgets import (
    QWidget, QGraphicsView, QGraphicsScene,
    QVBoxLayout, QHBoxLayout, QSplitter, QSizePolicy,
    QGraphicsPathItem, QGraphicsRectItem, QGraphicsItem,
    QMenu,
)
from PySide6.QtCore import Qt, QRectF, QPointF, Signal, QMimeData
from PySide6.QtGui import (
    QColor, QPen, QBrush, QPainter, QFont,
    QPainterPath, QDrag, QCursor,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

WELL_SCALE = 8.0              # mm → scene px (well-scale mode)
PLATE_SCALE = 4.0             # mm → scene px (plate-scale mode)
NEEDLE_DOT_RADIUS = 4         # px
CROSSHAIR_SIZE = 10           # px arm length
PATH_PEN_WIDTH = 2.0
UPCOMING_PEN_WIDTH = 1.5
UPCOMING_DASH_PATTERN = [4, 4]
DEFAULT_UPCOMING_COUNT = 20
WELL_BOTTOM_RECT_W = 4        # px width of well-bottom marker
Z_AMPLIFY = 10.0              # Z axis amplification for plate-scale views
MAX_COMPLETED_POINTS = 5000

# Catppuccin-based palette
COMPLETED_PATH_COLOR = "#a6e3a1"    # Green
UPCOMING_PATH_COLOR = "#f9e2af"     # Yellow dashed
NEEDLE_COLOR = "#f5c2e7"            # Pink
WELL_BOUNDARY_COLOR = "#45475a"     # Surface2
TRACKING_ERROR_COLOR = "#f38ba8"    # Red ring
BG_COLOR = "#181825"                # Mantle
WELL_BOTTOM_COLOR = "#45475a"       # Surface2
GRID_COLOR = "#313244"              # Surface0

# Drag-and-drop
OBJECT_MIME_TYPE = "application/x-mebp-object"
SELECTION_COLOR = "#89dceb"          # Sky  — selected border
HANDLE_SIZE = 6                      # px   — corner drag handles


# ═══════════════════════════════════════════════════════════════════
# Data containers for multi-object rendering
# ═══════════════════════════════════════════════════════════════════

@dataclass
class ObjectPath:
    """A named, colored path for one print object."""
    name: str
    color: str = COMPLETED_PATH_COLOR
    points: list[tuple[float, float, float]] = field(default_factory=list)

    def clear(self):
        self.points.clear()


@dataclass
class WellBottomMarker:
    """A well position + z-offset for plate-scale rendering."""
    name: str
    x_mm: float
    y_mm: float
    z_offset_mm: float = 0.0
    well_diameter_mm: float = 6.0


@dataclass
class PlacedObject:
    """An object placed into the interactive well preview via drag-drop."""
    name: str
    color: str = COMPLETED_PATH_COLOR
    points: list[tuple[float, float, float]] = field(default_factory=list)
    x_offset: float = 0.0   # mm — user-positioned offset from well center
    y_offset: float = 0.0
    z_offset: float = 0.0
    library_key: str = ""    # key back into the object library dict

    @property
    def offset_points(self) -> list[tuple[float, float, float]]:
        """Points translated by the user-set offset."""
        return [
            (p[0] + self.x_offset, p[1] + self.y_offset, p[2] + self.z_offset)
            for p in self.points
        ]


# ═══════════════════════════════════════════════════════════════════
# Single Projection Pane
# ═══════════════════════════════════════════════════════════════════

class ProjectionPane(QGraphicsView):
    """
    A single 2D projection pane (XY, ZY, or XZ).

    Supports two rendering modes:

    **Well-scale** (default):
        Renders within a single well — path lines, needle crosshair,
        upcoming waypoints, tracking error ring.  Coordinates are mm
        relative to well center.

    **Plate-scale**:
        Renders the full plate overview — well bottom markers as small
        rectangles, needle crosshair.  Coordinates are mm in plate space.
        Z axis is amplified for visibility.

    Axes:
        XY: h=x, v=y  (top-down)
        ZY: h=z, v=y  (side view)
        XZ: h=x, v=z  (front view)
    """

    def __init__(
        self,
        mode: str = "XY",
        scale_mode: str = "well",
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._mode = mode           # "XY", "ZY", "XZ"
        self._scale_mode = scale_mode  # "well" or "plate"
        self._scale = WELL_SCALE if scale_mode == "well" else PLATE_SCALE

        # ── Well-scale data ───────────────────────────────────────
        self._completed_points: deque[tuple[float, float]] = deque(
            maxlen=MAX_COMPLETED_POINTS)
        self._upcoming_points: list[tuple[float, float]] = []
        self._needle_pos: tuple[float, float] | None = None
        self._tracking_error_mm: float = 0.0
        self._well_diameter_mm: float = 0.0

        # Multi-object colored paths
        self._object_paths: list[ObjectPath] = []

        # ── Plate-scale data ──────────────────────────────────────
        self._well_markers: list[WellBottomMarker] = []
        self._needle_3d: tuple[float, float, float] | None = None

        # ── View settings ─────────────────────────────────────────
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet(
            f"background-color: {BG_COLOR}; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)

        # Mode-specific sizing
        if mode == "XY":
            self.setMinimumSize(200, 200)
            self.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        elif mode == "ZY":
            self.setFixedWidth(160 if scale_mode == "plate" else 140)
            self.setMinimumHeight(150)
            self.setSizePolicy(
                QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        else:  # XZ
            self.setFixedHeight(120 if scale_mode == "plate" else 100)
            self.setMinimumWidth(200)
            self.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    # ══════════════════════════════════════════════════════════════
    #  WELL-SCALE API  (paths, needle tracking, upcoming waypoints)
    # ══════════════════════════════════════════════════════════════

    def set_well_diameter(self, diameter_mm: float) -> None:
        """Set well boundary circle/line diameter."""
        self._well_diameter_mm = diameter_mm

    def add_completed_point(
        self, x: float, y: float, z: float,
    ) -> None:
        """Add a point to the default completed path."""
        h, v = self._project(x, y, z)
        self._completed_points.append((h, v))

    def set_upcoming_waypoints(
        self,
        waypoints: list[tuple[float, float, float]],
    ) -> None:
        """Set the upcoming waypoint list (next N points)."""
        self._upcoming_points = [self._project(*pt) for pt in waypoints]

    def set_needle_position(
        self,
        x: float | None,
        y: float | None,
        z: float | None,
    ) -> None:
        """Update needle position (works in both scale modes)."""
        if x is not None and y is not None and z is not None:
            self._needle_3d = (x, y, z)
            h, v = self._project(x, y, z)
            self._needle_pos = (h, v)
        else:
            self._needle_3d = None
            self._needle_pos = None

    def set_tracking_error(self, error_mm: float) -> None:
        """Set current tracking error for visual indicator ring."""
        self._tracking_error_mm = error_mm

    # ── Multi-object paths ────────────────────────────────────────

    def set_object_paths(self, paths: list[ObjectPath]) -> None:
        """Set multiple named + colored object paths for rendering."""
        self._object_paths = list(paths)

    def clear_object_paths(self) -> None:
        """Remove all object paths."""
        self._object_paths.clear()

    def clear_path(self) -> None:
        """Clear all path data (default + objects)."""
        self._completed_points.clear()
        self._upcoming_points.clear()
        self._needle_pos = None
        self._needle_3d = None
        self._tracking_error_mm = 0.0
        self._object_paths.clear()
        self._redraw()

    # ══════════════════════════════════════════════════════════════
    #  PLATE-SCALE API  (well bottom markers)
    # ══════════════════════════════════════════════════════════════

    def set_well_markers(self, markers: list[WellBottomMarker]) -> None:
        """Set well bottom height markers for plate-scale view."""
        self._well_markers = list(markers)

    def clear_well_markers(self) -> None:
        """Remove all well markers."""
        self._well_markers.clear()

    # ══════════════════════════════════════════════════════════════
    #  COMMON API
    # ══════════════════════════════════════════════════════════════

    def refresh(self) -> None:
        """Trigger a full scene redraw."""
        self._redraw()

    def set_scale_mode(self, mode: str) -> None:
        """Switch between 'well' and 'plate' scale modes."""
        self._scale_mode = mode
        self._scale = WELL_SCALE if mode == "well" else PLATE_SCALE

    # ══════════════════════════════════════════════════════════════
    #  PROJECTION
    # ══════════════════════════════════════════════════════════════

    def _project(self, x: float, y: float, z: float) -> tuple[float, float]:
        """Project 3D (x,y,z) onto this view's 2D axes."""
        if self._scale_mode == "plate":
            # Plate-scale: amplify Z for visibility
            if self._mode == "XY":
                return (x, y)
            elif self._mode == "ZY":
                return (z * Z_AMPLIFY, y)
            else:  # XZ
                return (x, z * Z_AMPLIFY)
        else:
            # Well-scale: 1:1 mm
            if self._mode == "XY":
                return (x, y)
            elif self._mode == "ZY":
                return (z, y)
            else:  # XZ
                return (x, z)

    def _scene_coord(self, h: float, v: float) -> tuple[float, float]:
        """Convert projected (h, v) in mm to scene coordinates."""
        return (h * self._scale, v * self._scale)

    # ══════════════════════════════════════════════════════════════
    #  RENDERING
    # ══════════════════════════════════════════════════════════════

    def _redraw(self) -> None:
        """Redraw the entire scene."""
        self._scene.clear()

        if self._scale_mode == "plate":
            self._draw_plate_mode()
        else:
            self._draw_well_mode()

        # Axis label (both modes)
        font = QFont("Segoe UI", 8)
        label = self._scene.addText(self._mode, font)
        label.setDefaultTextColor(QColor(COLORS["overlay0"]))
        br = self._scene.itemsBoundingRect()
        label.setPos(br.left() - 5, br.top() - 15)

        # Fit view
        rect = self._scene.itemsBoundingRect().adjusted(-15, -15, 15, 15)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    # ── Well-scale rendering ──────────────────────────────────────

    def _draw_well_mode(self) -> None:
        """Render well-scale view: well boundary + paths + needle."""
        s = self._scale

        # Well boundary
        if self._well_diameter_mm > 0:
            r = self._well_diameter_mm / 2 * s
            pen = QPen(QColor(WELL_BOUNDARY_COLOR), 1.5, Qt.PenStyle.DotLine)
            if self._mode == "XY":
                self._scene.addEllipse(-r, -r, 2 * r, 2 * r, pen)
            else:
                # Side/front: horizontal line at well bottom (z=0)
                self._scene.addLine(-r, 0, r, 0, pen)

        # Multi-object colored paths
        for obj_path in self._object_paths:
            if len(obj_path.points) < 2:
                continue
            path = QPainterPath()
            projected = [self._project(*pt) for pt in obj_path.points]
            sx0, sy0 = self._scene_coord(*projected[0])
            path.moveTo(sx0, sy0)
            for pt in projected[1:]:
                sx, sy = self._scene_coord(*pt)
                path.lineTo(sx, sy)
            pen = QPen(QColor(obj_path.color), PATH_PEN_WIDTH)
            self._scene.addPath(path, pen)

        # Default completed path (solid green polyline)
        if len(self._completed_points) >= 2:
            path = QPainterPath()
            pts = list(self._completed_points)
            path.moveTo(pts[0][0] * s, pts[0][1] * s)
            for h, v in pts[1:]:
                path.lineTo(h * s, v * s)
            pen = QPen(QColor(COMPLETED_PATH_COLOR), PATH_PEN_WIDTH)
            self._scene.addPath(path, pen)

        # Upcoming waypoints (dashed yellow polyline)
        if len(self._upcoming_points) >= 2:
            path = QPainterPath()
            path.moveTo(
                self._upcoming_points[0][0] * s,
                self._upcoming_points[0][1] * s,
            )
            for h, v in self._upcoming_points[1:]:
                path.lineTo(h * s, v * s)
            pen = QPen(QColor(UPCOMING_PATH_COLOR), UPCOMING_PEN_WIDTH)
            pen.setDashPattern(UPCOMING_DASH_PATTERN)
            self._scene.addPath(path, pen)

        # Upcoming dots
        dot_pen = QPen(QColor(UPCOMING_PATH_COLOR), 1)
        dot_brush = QBrush(QColor(UPCOMING_PATH_COLOR))
        for h, v in self._upcoming_points:
            self._scene.addEllipse(
                h * s - 2, v * s - 2, 4, 4,
                dot_pen, dot_brush,
            )

        # Needle crosshair
        self._draw_needle_crosshair(s)

    # ── Plate-scale rendering ─────────────────────────────────────

    def _draw_plate_mode(self) -> None:
        """Render plate-scale view: well-bottom markers + needle."""
        s = self._scale
        well_pen = QPen(QColor(WELL_BOTTOM_COLOR), 1)
        well_brush = QBrush(QColor(WELL_BOTTOM_COLOR))

        for marker in self._well_markers:
            h, v = self._project(marker.x_mm, marker.y_mm, marker.z_offset_mm)
            sx, sy = h * s, v * s

            if self._mode == "XY":
                # Top-down: small circle per well
                r = marker.well_diameter_mm * s * 0.15
                self._scene.addEllipse(
                    sx - r, sy - r, 2 * r, 2 * r,
                    well_pen, well_brush,
                )
            elif self._mode == "ZY":
                # Side view: small rect at (z_amp, y)
                well_h = marker.well_diameter_mm * s * 0.3
                self._scene.addRect(
                    QRectF(sx - WELL_BOTTOM_RECT_W / 2,
                           sy - well_h / 2,
                           WELL_BOTTOM_RECT_W, well_h),
                    well_pen, well_brush,
                )
            else:  # XZ
                # Front view: small rect at (x, z_amp)
                well_w = marker.well_diameter_mm * s * 0.3
                self._scene.addRect(
                    QRectF(sx - well_w / 2,
                           sy - WELL_BOTTOM_RECT_W / 2,
                           well_w, WELL_BOTTOM_RECT_W),
                    well_pen, well_brush,
                )

        # Needle crosshair (uses self._needle_pos set via set_needle_position)
        self._draw_needle_crosshair(s)

    # ── Shared needle rendering ───────────────────────────────────

    def _draw_needle_crosshair(self, s: float) -> None:
        """Draw needle crosshair + optional tracking error ring."""
        if self._needle_pos is None:
            return

        nh, nv = self._needle_pos
        sx, sy = nh * s, nv * s

        # Tracking error ring
        if self._tracking_error_mm > 0.01:
            err_r = self._tracking_error_mm * s
            err_pen = QPen(
                QColor(TRACKING_ERROR_COLOR), 1, Qt.PenStyle.DotLine)
            self._scene.addEllipse(
                sx - err_r, sy - err_r, 2 * err_r, 2 * err_r, err_pen,
            )

        # Crosshair lines
        needle_pen = QPen(QColor(NEEDLE_COLOR), 2)
        self._scene.addLine(
            sx - CROSSHAIR_SIZE, sy, sx + CROSSHAIR_SIZE, sy, needle_pen)
        self._scene.addLine(
            sx, sy - CROSSHAIR_SIZE, sx, sy + CROSSHAIR_SIZE, needle_pen)

        # Center dot
        self._scene.addEllipse(
            sx - NEEDLE_DOT_RADIUS, sy - NEEDLE_DOT_RADIUS,
            NEEDLE_DOT_RADIUS * 2, NEEDLE_DOT_RADIUS * 2,
            QPen(QColor(NEEDLE_COLOR), 1),
            QBrush(QColor(NEEDLE_COLOR)),
        )

    # ── Resize ────────────────────────────────────────────────────

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        rect = self._scene.itemsBoundingRect().adjusted(-15, -15, 15, 15)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)


# ═══════════════════════════════════════════════════════════════════
# L-Shaped Triple Projection Canvas
# ═══════════════════════════════════════════════════════════════════

class ProjectionCanvas(QWidget):
    """
    L-shaped triple-projection widget combining XY, ZY, and XZ panes.

    ┌──────────────┬────────┐
    │              │        │
    │   XY (large) │ ZY     │
    │              │ (tall) │
    │              │        │
    ├──────────────┴────────┤
    │   XZ (wide)           │
    └───────────────────────┘

    Provides a unified API that forwards to all three panes.

    Usage examples:

        # Well-scale for Print Objects preview
        canvas = ProjectionCanvas(scale_mode="well")
        canvas.set_well_diameter(6.0)
        canvas.set_object_paths([ObjectPath("ring", "#89b4fa", pts)])
        canvas.refresh()

        # Plate-scale for Well Setup projection
        canvas = ProjectionCanvas(scale_mode="plate")
        canvas.set_well_markers(markers)
        canvas.set_needle_position(10.0, 20.0, 0.5)
        canvas.refresh()
    """

    def __init__(
        self,
        scale_mode: str = "well",
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._scale_mode = scale_mode

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Top row: XY + ZY
        top_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.xy_view = ProjectionPane("XY", scale_mode)
        self.zy_view = ProjectionPane("ZY", scale_mode)
        top_splitter.addWidget(self.xy_view)
        top_splitter.addWidget(self.zy_view)
        top_splitter.setStretchFactor(0, 4)
        top_splitter.setStretchFactor(1, 1)
        layout.addWidget(top_splitter, stretch=3)

        # Bottom row: XZ
        self.xz_view = ProjectionPane("XZ", scale_mode)
        layout.addWidget(self.xz_view, stretch=1)

        self._views = [self.xy_view, self.zy_view, self.xz_view]

    # ══════════════════════════════════════════════════════════════
    #  WELL-SCALE API  (forwarded to all 3 panes)
    # ══════════════════════════════════════════════════════════════

    def set_well_diameter(self, diameter_mm: float) -> None:
        """Set well boundary on all views."""
        for v in self._views:
            v.set_well_diameter(diameter_mm)

    def add_completed_point(
        self, x: float, y: float, z: float,
    ) -> None:
        """Add a completed path point to all views."""
        for v in self._views:
            v.add_completed_point(x, y, z)

    def set_upcoming_waypoints(
        self,
        waypoints: list[tuple[float, float, float]],
    ) -> None:
        """Set upcoming waypoints on all views."""
        for v in self._views:
            v.set_upcoming_waypoints(waypoints)

    def set_object_paths(self, paths: list[ObjectPath]) -> None:
        """Set multi-object colored paths on all views."""
        for v in self._views:
            v.set_object_paths(paths)

    def clear_object_paths(self) -> None:
        """Remove all object paths from all views."""
        for v in self._views:
            v.clear_object_paths()

    # ══════════════════════════════════════════════════════════════
    #  PLATE-SCALE API  (forwarded to all 3 panes)
    # ══════════════════════════════════════════════════════════════

    def set_well_markers(self, markers: list[WellBottomMarker]) -> None:
        """Set well bottom markers on all views."""
        for v in self._views:
            v.set_well_markers(markers)

    def clear_well_markers(self) -> None:
        """Remove well markers from all views."""
        for v in self._views:
            v.clear_well_markers()

    # ══════════════════════════════════════════════════════════════
    #  COMMON API  (forwarded to all 3 panes)
    # ══════════════════════════════════════════════════════════════

    def set_needle_position(
        self,
        x: float | None,
        y: float | None,
        z: float | None,
    ) -> None:
        """Update needle position on all views."""
        for v in self._views:
            v.set_needle_position(x, y, z)

    def set_tracking_error(self, error_mm: float) -> None:
        """Set tracking error on all views."""
        for v in self._views:
            v.set_tracking_error(error_mm)

    def clear_path(self) -> None:
        """Clear all path data on all views."""
        for v in self._views:
            v.clear_path()

    def refresh(self) -> None:
        """Trigger redraw on all views."""
        for v in self._views:
            v.refresh()

    def set_scale_mode(self, mode: str) -> None:
        """Switch all panes between 'well' and 'plate' scale modes."""
        self._scale_mode = mode
        for v in self._views:
            v.set_scale_mode(mode)


# ═══════════════════════════════════════════════════════════════════
# Convenience factory functions
# ═══════════════════════════════════════════════════════════════════

def create_well_preview() -> ProjectionCanvas:
    """Create a well-scale projection canvas for object preview."""
    return ProjectionCanvas(scale_mode="well")


def create_plate_projection() -> ProjectionCanvas:
    """Create a plate-scale projection canvas for well setup."""
    return ProjectionCanvas(scale_mode="plate")


# ═══════════════════════════════════════════════════════════════════
# Backward-Compatible MiniProjectionView Adapter
# ═══════════════════════════════════════════════════════════════════

class MiniProjectionView(ProjectionPane):
    """
    Drop-in replacement for the inline MiniProjectionView that was
    previously in print_well_setup.py.

    Provides the ``set_plate_data(plate, z_offsets)`` convenience API
    so that Well Setup tab code needs only an import change.
    """

    def __init__(self, mode: str = "ZY", parent: QWidget | None = None):
        super().__init__(mode=mode, scale_mode="plate", parent=parent)

    def set_plate_data(
        self,
        plate,
        z_offsets: dict[str, float] | None = None,
    ) -> None:
        """
        Update plate geometry for projection.

        Converts WellPlate + z_offsets dict into WellBottomMarker list
        and delegates to the canonical ``set_well_markers`` API.

        Args:
            plate: WellPlate instance with ``get_all_wells()`` and
                   ``well_diameter`` attributes.
            z_offsets: Dict mapping well name → Z offset in mm.
        """
        if plate is None:
            self.clear_well_markers()
            self.refresh()
            return

        z_off = z_offsets or {}
        markers: list[WellBottomMarker] = []

        try:
            wells = plate.get_all_wells()
            diameter = getattr(plate, 'well_diameter', 6.0)
            for w in wells:
                markers.append(WellBottomMarker(
                    name=w.name,
                    x_mm=w.x,
                    y_mm=w.y,
                    z_offset_mm=z_off.get(w.name, 0.0),
                    well_diameter_mm=diameter,
                ))
        except Exception:
            logger.debug("Failed to read plate wells for projection")

        self.set_well_markers(markers)
        self.refresh()


# ═══════════════════════════════════════════════════════════════════
# Draggable Object Item (for Interactive mode)
# ═══════════════════════════════════════════════════════════════════

class DraggableObjectItem(QGraphicsPathItem):
    """
    A movable print-object path in the XY projection scene.

    Users can click-drag to reposition. The item shows a selection
    highlight when selected.  Position changes are reported back to
    the owning InteractiveProjectionPane.
    """

    def __init__(
        self,
        placed: PlacedObject,
        scale: float = WELL_SCALE,
        parent: QGraphicsItem | None = None,
    ):
        super().__init__(parent)
        self.placed = placed
        self._scale = scale

        # Flags
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setAcceptHoverEvents(True)
        self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))

        self._rebuild_path()

    def _rebuild_path(self) -> None:
        """Build the QPainterPath from placed object points (XY only)."""
        path = QPainterPath()
        pts = self.placed.points  # raw points, offset applied via item pos
        if len(pts) >= 2:
            path.moveTo(pts[0][0] * self._scale, pts[0][1] * self._scale)
            for p in pts[1:]:
                path.lineTo(p[0] * self._scale, p[1] * self._scale)
        elif len(pts) == 1:
            # Single point: draw a small dot
            r = 3
            path.addEllipse(
                pts[0][0] * self._scale - r, pts[0][1] * self._scale - r,
                r * 2, r * 2)
        self.setPath(path)
        self.setPen(QPen(QColor(self.placed.color), PATH_PEN_WIDTH))
        self.setBrush(QBrush(Qt.BrushStyle.NoBrush))

        # Set initial position from offset
        self.setPos(
            self.placed.x_offset * self._scale,
            self.placed.y_offset * self._scale)

    def paint(self, painter, option, widget=None) -> None:
        """Custom paint: path + selection highlight."""
        super().paint(painter, option, widget)
        if self.isSelected():
            br = self.boundingRect()
            painter.setPen(QPen(
                QColor(SELECTION_COLOR), 1.5, Qt.PenStyle.DashLine))
            painter.drawRect(br)

    def itemChange(self, change, value):
        """Track position changes and update PlacedObject offset."""
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            new_pos = value
            self.placed.x_offset = new_pos.x() / self._scale
            self.placed.y_offset = new_pos.y() / self._scale
            # Notify parent pane (if it has a callback)
            scene = self.scene()
            if scene:
                for view in scene.views():
                    if isinstance(view, InteractiveProjectionPane):
                        view._on_item_moved(self)
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        self.setCursor(QCursor(Qt.CursorShape.ClosedHandCursor))
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))
        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event):
        """Right-click to delete."""
        menu = QMenu()
        del_action = menu.addAction(f"Remove '{self.placed.name}'")
        action = menu.exec(event.screenPos())
        if action == del_action:
            scene = self.scene()
            if scene:
                for view in scene.views():
                    if isinstance(view, InteractiveProjectionPane):
                        view._remove_placed_item(self)


# ═══════════════════════════════════════════════════════════════════
# Interactive Projection Pane (XY view that accepts drops)
# ═══════════════════════════════════════════════════════════════════

class InteractiveProjectionPane(ProjectionPane):
    """
    XY projection pane with drag-drop support for object placement.

    Accepts drops of OBJECT_MIME_TYPE (from the library list) and
    creates DraggableObjectItems that users can reposition by dragging.
    """

    # Internal signals (connected by InteractiveProjectionCanvas)
    _object_placed_sig = Signal(str, float, float)   # name, x_mm, y_mm
    _object_moved_sig = Signal(str, float, float)    # name, x_mm, y_mm
    _object_removed_sig = Signal(str)                # name

    def __init__(self, parent=None):
        super().__init__(mode="XY", scale_mode="well", parent=parent)
        self.setAcceptDrops(True)
        self._placed_items: list[DraggableObjectItem] = []
        # The resolver callback is set by the tab so we can get point
        # data for a library object name at drop time.
        self._resolve_library: callable = None

    def set_library_resolver(self, fn) -> None:
        """Set callback: fn(library_key) → PlacedObject or None."""
        self._resolve_library = fn

    # ── Drop handling ─────────────────────────────────────────────

    def dragEnterEvent(self, event) -> None:
        if event.mimeData().hasFormat(OBJECT_MIME_TYPE):
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event) -> None:
        if event.mimeData().hasFormat(OBJECT_MIME_TYPE):
            event.acceptProposedAction()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event) -> None:
        if not event.mimeData().hasFormat(OBJECT_MIME_TYPE):
            super().dropEvent(event)
            return

        library_key = bytes(
            event.mimeData().data(OBJECT_MIME_TYPE)).decode("utf-8")

        # Convert drop position to mm coordinates
        scene_pos = self.mapToScene(event.position().toPoint())
        x_mm = scene_pos.x() / self._scale
        y_mm = scene_pos.y() / self._scale

        # Resolve the object from the library
        placed = None
        if self._resolve_library:
            placed = self._resolve_library(library_key, x_mm, y_mm)

        if placed is None:
            event.ignore()
            return

        self._add_placed_object(placed)
        event.acceptProposedAction()
        self._object_placed_sig.emit(placed.name, x_mm, y_mm)

    # ── Placed object management ──────────────────────────────────

    def _add_placed_object(self, placed: PlacedObject) -> None:
        """Add a DraggableObjectItem to the scene."""
        item = DraggableObjectItem(placed, self._scale)
        self._scene.addItem(item)
        self._placed_items.append(item)

    def _remove_placed_item(self, item: DraggableObjectItem) -> None:
        """Remove a placed item (called from context menu)."""
        name = item.placed.name
        self._scene.removeItem(item)
        if item in self._placed_items:
            self._placed_items.remove(item)
        self._object_removed_sig.emit(name)

    def _on_item_moved(self, item: DraggableObjectItem) -> None:
        """Called when a DraggableObjectItem is repositioned."""
        self._object_moved_sig.emit(
            item.placed.name,
            item.placed.x_offset,
            item.placed.y_offset,
        )
        # Update ZY/XZ mirrors on the parent canvas
        canvas = self._parent_canvas()
        if canvas:
            canvas._sync_side_views()

    def _parent_canvas(self):
        """Walk up to find the owning InteractiveProjectionCanvas."""
        w = self.parentWidget()
        while w:
            if isinstance(w, InteractiveProjectionCanvas):
                return w
            w = w.parentWidget()
        return None

    def get_placed_objects(self) -> list[PlacedObject]:
        """Return all placed objects with current positions."""
        return [item.placed for item in self._placed_items]

    def clear_placed_objects(self) -> None:
        """Remove all placed items from the scene."""
        for item in list(self._placed_items):
            self._scene.removeItem(item)
        self._placed_items.clear()

    # ── Redraw override (preserve placed items) ───────────────────

    def _redraw(self) -> None:
        """Redraw scene but preserve DraggableObjectItems."""
        # Save placed items (they are children of the scene)
        saved = list(self._placed_items)
        for item in saved:
            self._scene.removeItem(item)

        # Normal redraw (clears scene, draws boundary + paths)
        super()._redraw()

        # Re-add placed items
        for item in saved:
            self._scene.addItem(item)


# ═══════════════════════════════════════════════════════════════════
# Interactive Projection Canvas (full L-shaped with drag-drop)
# ═══════════════════════════════════════════════════════════════════

class InteractiveProjectionCanvas(ProjectionCanvas):
    """
    ProjectionCanvas with interactive drag-drop object placement.

    The XY pane accepts drops from the library list. Objects can be
    repositioned by dragging. ZY and XZ panes show placed objects
    as read-only mirrors.

    Signals:
        object_placed(name, x_mm, y_mm)  — object dropped into preview
        object_moved(name, x_mm, y_mm)   — object repositioned by drag
        object_removed(name)             — object deleted via right-click
    """

    object_placed = Signal(str, float, float)
    object_moved = Signal(str, float, float)
    object_removed = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        # Skip ProjectionCanvas.__init__ — we override the layout
        QWidget.__init__(self, parent)
        self._scale_mode = "well"

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Top row: Interactive XY + read-only ZY
        top_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.xy_view = InteractiveProjectionPane()
        self.zy_view = ProjectionPane("ZY", "well")
        top_splitter.addWidget(self.xy_view)
        top_splitter.addWidget(self.zy_view)
        top_splitter.setStretchFactor(0, 4)
        top_splitter.setStretchFactor(1, 1)
        layout.addWidget(top_splitter, stretch=3)

        # Bottom: read-only XZ
        self.xz_view = ProjectionPane("XZ", "well")
        layout.addWidget(self.xz_view, stretch=1)

        self._views = [self.xy_view, self.zy_view, self.xz_view]

        # Wire internal signals → public signals
        self.xy_view._object_placed_sig.connect(self.object_placed)
        self.xy_view._object_moved_sig.connect(self.object_moved)
        self.xy_view._object_removed_sig.connect(self._on_removed)

    def _on_removed(self, name: str) -> None:
        """Handle removal: update side views, re-emit."""
        self._sync_side_views()
        self.object_removed.emit(name)

    def set_library_resolver(self, fn) -> None:
        """Set callback for resolving library key → PlacedObject."""
        self.xy_view.set_library_resolver(fn)

    def get_placed_objects(self) -> list[PlacedObject]:
        """Return all placed objects with current user-set positions."""
        return self.xy_view.get_placed_objects()

    def clear_placed_objects(self) -> None:
        """Remove all placed objects from the preview."""
        self.xy_view.clear_placed_objects()
        self._sync_side_views()

    def _sync_side_views(self) -> None:
        """Update ZY and XZ panes with placed object paths."""
        placed = self.xy_view.get_placed_objects()
        paths = [
            ObjectPath(
                name=p.name,
                color=p.color,
                points=p.offset_points,
            )
            for p in placed
        ]
        self.zy_view.set_object_paths(paths)
        self.zy_view.refresh()
        self.xz_view.set_object_paths(paths)
        self.xz_view.refresh()


def create_interactive_well_preview() -> InteractiveProjectionCanvas:
    """Create an interactive well-scale canvas for object arrangement."""
    return InteractiveProjectionCanvas()
