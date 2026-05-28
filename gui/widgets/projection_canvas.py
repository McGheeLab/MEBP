"""
projection_canvas.py — Reusable multi-projection canvas for MEBP v7.1.

Unified L-shaped triple-projection widget used across the application:
- Tab 2 (Print Objects): interactive drag-drop object arrangement
- Tab 3 (Well Setup): plate-scale well bottom heights + needle overlay
- Print Monitor: real-time trajectory tracking with upcoming waypoints

Layout (double-click any side pane to promote it to primary):
┌──────────────┬────────┐
│              │        │
│   PRIMARY    │ SIDE   │
│   (large)    │ (tall) │
│              │        │
├──────────────┴────────┤
│   SIDE (wide)         │
└───────────────────────┘

Each pane renders (as applicable):
- Well boundary (circle or line, from plate spec)
- Multi-object paths, each with its own color
- Well bottom height markers (plate-scale mode)
- Completed path (solid line) + upcoming waypoints (dashed)
- Needle crosshair with tracking error ring
- Millimetre grid and ruler markings

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
    QMenu, QPushButton, QLabel,
)
from PySide6.QtCore import Qt, QRectF, QPointF, Signal, QMimeData
from PySide6.QtGui import (
    QColor, QPen, QBrush, QPainter, QFont,
    QPainterPath, QDrag, QCursor,
)

from gui.styles import COLORS
from gui.scaling import s as _s, scaled_font_size as _sf

logger = logging.getLogger(__name__)

# Qt max widget size (used to reset fixed-size constraints)
_QMAX = 16777215


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
DEFAULT_WELL_EXTENT = 5.0     # mm — fallback ruler range when no well set

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

# Ruler / grid
RULER_FONT_SIZE = 6
RULER_LABEL_COLOR_KEY = "overlay0"
GRID_ALPHA = 70                      # 0-255


# ═══════════════════════════════════════════════════════════════════
# Data containers
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
    sphere_radius_mm: float = 0.0  # >0 for point objects — renders as filled sphere

    @property
    def offset_points(self) -> list[tuple[float, float, float]]:
        """Points translated by the user-set offset."""
        return [
            (p[0] + self.x_offset, p[1] + self.y_offset, p[2] + self.z_offset)
            for p in self.points
        ]


# ═══════════════════════════════════════════════════════════════════
# Helper: nice grid step
# ═══════════════════════════════════════════════════════════════════

def _nice_step(range_mm: float) -> float:
    """Pick a readable grid spacing for a given axis range."""
    if range_mm <= 0:
        return 1.0
    raw = range_mm / 6.0
    mag = 10 ** math.floor(math.log10(max(raw, 1e-9)))
    norm = raw / mag
    if norm < 1.5:
        return mag
    elif norm < 3.5:
        return 2 * mag
    elif norm < 7.5:
        return 5 * mag
    return 10 * mag


# ═══════════════════════════════════════════════════════════════════
# Single Projection Pane
# ═══════════════════════════════════════════════════════════════════

class ProjectionPane(QGraphicsView):
    """
    A single 2D projection pane (XY, ZY, XZ, or YZ).

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
        ZY: h=z, v=y  (side view, Z horizontal)
        XZ: h=x, v=z  (front view)
        YZ: h=y, v=z  (side view, Y horizontal) — added in v7.5.1

    Double-click to promote this view to primary position.
    """

    promote_requested = Signal(str)  # emits mode name ("XY"/"ZY"/"XZ"/"YZ")
    # v7.5.1: emitted when wheel-zoom changes the transform; canvases
    # in row layout broadcast the factor to peer panes for locked zoom.
    zoom_changed = Signal(float)

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
        self._well_depth_mm: float = 0.0  # v7.6.0: for XZ/YZ outline

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

        # Default size policy (overridden by canvas layout)
        if mode == "XY":
            initial_position = "primary"
        elif mode == "ZY":
            initial_position = "side_right"
        elif mode == "YZ":
            # Row-layout default; same constraints as side_right
            initial_position = "side_right"
        else:  # XZ
            initial_position = "side_bottom"
        self._apply_position_sizing(initial_position)

        # v7.5.1: external-pan suppression flag so canvas-driven
        # scrollbar sync doesn't recurse back into the peer panes.
        self._suppress_pan_signal = False

    # ── Position sizing (called by canvas when swapping views) ────

    def _apply_position_sizing(self, position: str) -> None:
        """Adjust size policy for layout position."""
        if position == "primary":
            self.setMinimumSize(200, 200)
            self.setMaximumSize(_QMAX, _QMAX)
            self.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        elif position == "side_right":
            self.setMinimumSize(80, 120)
            self.setMaximumSize(180, _QMAX)
            self.setSizePolicy(
                QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        elif position == "side_bottom":
            self.setMinimumSize(200, 60)
            self.setMaximumSize(_QMAX, 160)
            self.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

    # ══════════════════════════════════════════════════════════════
    #  WELL-SCALE API
    # ══════════════════════════════════════════════════════════════

    def set_well_diameter(self, diameter_mm: float) -> None:
        self._well_diameter_mm = diameter_mm

    def set_well_depth(self, depth_mm: float) -> None:
        """v7.6.0: well depth (mm) so the Z-axis views (XZ, YZ) can
        draw the well-height outline with the floor at the bottom."""
        self._well_depth_mm = depth_mm

    def add_completed_point(self, x: float, y: float, z: float) -> None:
        h, v = self._project(x, y, z)
        self._completed_points.append((h, v))

    def set_upcoming_waypoints(
        self, waypoints: list[tuple[float, float, float]],
    ) -> None:
        self._upcoming_points = [self._project(*pt) for pt in waypoints]

    def set_needle_position(
        self, x: float | None, y: float | None, z: float | None,
    ) -> None:
        if x is not None and y is not None and z is not None:
            self._needle_3d = (x, y, z)
            h, v = self._project(x, y, z)
            self._needle_pos = (h, v)
        else:
            self._needle_3d = None
            self._needle_pos = None

    def set_tracking_error(self, error_mm: float) -> None:
        self._tracking_error_mm = error_mm

    # ── Multi-object paths ────────────────────────────────────────

    def set_object_paths(self, paths: list[ObjectPath]) -> None:
        self._object_paths = list(paths)

    def clear_object_paths(self) -> None:
        self._object_paths.clear()

    def clear_path(self) -> None:
        self._completed_points.clear()
        self._upcoming_points.clear()
        self._needle_pos = None
        self._needle_3d = None
        self._tracking_error_mm = 0.0
        self._object_paths.clear()
        self._redraw()

    # ══════════════════════════════════════════════════════════════
    #  PLATE-SCALE API
    # ══════════════════════════════════════════════════════════════

    def set_well_markers(self, markers: list[WellBottomMarker]) -> None:
        self._well_markers = list(markers)

    def clear_well_markers(self) -> None:
        self._well_markers.clear()

    # ══════════════════════════════════════════════════════════════
    #  COMMON API
    # ══════════════════════════════════════════════════════════════

    def refresh(self) -> None:
        self._redraw()

    def set_scale_mode(self, mode: str) -> None:
        self._scale_mode = mode
        self._scale = WELL_SCALE if mode == "well" else PLATE_SCALE

    # ══════════════════════════════════════════════════════════════
    #  PROJECTION
    # ══════════════════════════════════════════════════════════════

    def _project(self, x: float, y: float, z: float) -> tuple[float, float]:
        if self._scale_mode == "plate":
            if self._mode == "XY":
                return (x, y)
            elif self._mode == "ZY":
                return (z * Z_AMPLIFY, y)
            elif self._mode == "YZ":
                return (y, z * Z_AMPLIFY)
            else:  # XZ
                return (x, z * Z_AMPLIFY)
        else:
            if self._mode == "XY":
                return (x, y)
            elif self._mode == "ZY":
                return (z, y)
            elif self._mode == "YZ":
                # v7.6.0: Z runs upward — floor (z=0) at the bottom, well
                # top above. Scene-y grows downward, so negate z.
                return (y, -z)
            else:  # XZ
                return (x, -z)

    def _scene_coord(self, h: float, v: float) -> tuple[float, float]:
        return (h * self._scale, v * self._scale)

    # ══════════════════════════════════════════════════════════════
    #  AXIS INFO
    # ══════════════════════════════════════════════════════════════

    def _axis_labels(self) -> tuple[str, str]:
        if self._mode == "XY":
            return ("X", "Y")
        elif self._mode == "ZY":
            return ("Z", "Y")
        elif self._mode == "YZ":
            return ("Y", "Z")
        return ("X", "Z")

    def _axis_scales(self) -> tuple[float, float]:
        """mm → scene factors for (horizontal, vertical) axes."""
        s = self._scale
        if self._scale_mode == "plate":
            if self._mode == "ZY":
                return (s * Z_AMPLIFY, s)
            elif self._mode == "YZ":
                return (s, s * Z_AMPLIFY)
            elif self._mode == "XZ":
                return (s, s * Z_AMPLIFY)
        return (s, s)

    # ══════════════════════════════════════════════════════════════
    #  RENDERING
    # ══════════════════════════════════════════════════════════════

    def _redraw(self) -> None:
        self._scene.clear()

        # Background: grid + rulers
        self._draw_rulers()

        # Content
        if self._scale_mode == "plate":
            self._draw_plate_mode()
        else:
            self._draw_well_mode()

        # Mode badge (top-left)
        font = QFont("Segoe UI", 8, QFont.Weight.Bold)
        label = self._scene.addText(self._mode, font)
        label.setDefaultTextColor(QColor(COLORS["overlay0"]))
        br = self._scene.itemsBoundingRect()
        label.setPos(br.left(), br.top() - 16)

        # Fit view (stable region — see _fit_view)
        self._fit_view()

    def _fit_rect(self) -> QRectF | None:
        """Scene-coord rectangle the view should frame.

        For well-scale views this is the fixed well/ruler extent — NOT
        the item bounding rect — so dragging an object never rescales
        the view (which made objects accelerate away and feel jumpy).
        Plate-scale views keep the legacy item-bounds behaviour.
        """
        if self._scale_mode != "well":
            return None
        h_min, h_max, v_min, v_max = self._ruler_extent_mm()
        if (h_max - h_min) < 0.01 and (v_max - v_min) < 0.01:
            return None
        h_fac, v_fac = self._axis_scales()
        margin = 0.12 * max(h_max - h_min, v_max - v_min)
        return QRectF(
            (h_min - margin) * h_fac,
            (v_min - margin) * v_fac,
            (h_max - h_min + 2 * margin) * h_fac,
            (v_max - v_min + 2 * margin) * v_fac,
        )

    def _fit_view(self) -> None:
        fit = self._fit_rect()
        if fit is not None and not fit.isEmpty():
            # Pin the scene rect so dragged items stay bounded + the
            # zoom is constant regardless of object position.
            self._scene.setSceneRect(fit)
            self.fitInView(fit, Qt.AspectRatioMode.KeepAspectRatio)
            return
        rect = self._scene.itemsBoundingRect().adjusted(-15, -15, 15, 15)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    # ── Grid + mm rulers ──────────────────────────────────────────

    def _draw_rulers(self) -> None:
        h_fac, v_fac = self._axis_scales()
        h_label, v_label = self._axis_labels()

        h_min, h_max, v_min, v_max = self._ruler_extent_mm()
        h_range = h_max - h_min
        v_range = v_max - v_min
        if h_range < 0.01 and v_range < 0.01:
            return

        step = _nice_step(max(h_range, v_range))

        # Grid lines
        gc = QColor(GRID_COLOR)
        gc.setAlpha(GRID_ALPHA)
        grid_pen = QPen(gc, 0.5, Qt.PenStyle.DotLine)

        h = math.ceil(h_min / step) * step
        while h <= h_max + step * 0.01:
            sx = h * h_fac
            self._scene.addLine(
                sx, v_min * v_fac, sx, v_max * v_fac, grid_pen)
            h += step

        v = math.ceil(v_min / step) * step
        while v <= v_max + step * 0.01:
            sy = v * v_fac
            self._scene.addLine(
                h_min * h_fac, sy, h_max * h_fac, sy, grid_pen)
            v += step

        # Ruler labels
        font = QFont("Segoe UI", RULER_FONT_SIZE)
        lc = QColor(COLORS[RULER_LABEL_COLOR_KEY])
        use_decimal = step < 1

        # Bottom (horizontal axis)
        h = math.ceil(h_min / step) * step
        while h <= h_max + step * 0.01:
            if abs(h) > step * 0.01:
                sx = h * h_fac
                fmt = f"{h:.1f}" if use_decimal else f"{h:.0f}"
                txt = self._scene.addText(fmt, font)
                txt.setDefaultTextColor(lc)
                tb = txt.boundingRect()
                txt.setPos(sx - tb.width() / 2, v_max * v_fac + 3)
            h += step

        # Left (vertical axis)
        z_view = self._is_well_height_view()
        v = math.ceil(v_min / step) * step
        while v <= v_max + step * 0.01:
            sy = v * v_fac
            if z_view:
                # Vertical scene-y is negated z; show the Z *height*
                # (0 = floor at the bottom, increasing upward). Skip any
                # below-floor ticks.
                z_val = -v
                if z_val >= -step * 0.01:
                    fmt = f"{z_val:.1f}" if use_decimal else f"{z_val:.0f}"
                    txt = self._scene.addText(fmt, font)
                    txt.setDefaultTextColor(lc)
                    tb = txt.boundingRect()
                    txt.setPos(h_min * h_fac - tb.width() - 3,
                               sy - tb.height() / 2)
            elif abs(v) > step * 0.01:
                fmt = f"{v:.1f}" if use_decimal else f"{v:.0f}"
                txt = self._scene.addText(fmt, font)
                txt.setDefaultTextColor(lc)
                tb = txt.boundingRect()
                txt.setPos(h_min * h_fac - tb.width() - 3,
                           sy - tb.height() / 2)
            v += step

        # Axis name labels
        nf = QFont("Segoe UI", 7, QFont.Weight.Bold)
        ah = self._scene.addText(f"{h_label} (mm)", nf)
        ah.setDefaultTextColor(lc)
        ah.setPos(h_max * h_fac - 16, v_max * v_fac + 14)
        av = self._scene.addText(f"{v_label}", nf)
        av.setDefaultTextColor(lc)
        av.setPos(h_min * h_fac - 18, v_min * v_fac - 16)

    def _is_well_height_view(self) -> bool:
        """True for the side/front well-scale projections whose vertical
        axis is Z (drawn 0 = floor at the bottom, up to the well top)."""
        return self._scale_mode == "well" and self._mode in ("XZ", "YZ")

    def _ruler_extent_mm(self) -> tuple[float, float, float, float]:
        """(h_min, h_max, v_min, v_max) in mm for ruler drawing."""
        if self._scale_mode == "well":
            r = (self._well_diameter_mm / 2
                 if self._well_diameter_mm > 0
                 else DEFAULT_WELL_EXTENT)
            ext = r * 1.3
            if self._is_well_height_view():
                # Vertical = Z height. Scene-y is negated z, so the
                # floor (z=0) sits at scene-y 0 (bottom) and the well
                # top at scene-y -depth (above). Range bottom→top:
                # a touch below the floor up past the rim.
                depth = (self._well_depth_mm
                         if self._well_depth_mm > 0 else DEFAULT_WELL_EXTENT)
                return (-ext, ext, -depth * 1.12, depth * 0.08)
            return (-ext, ext, -ext, ext)

        if not self._well_markers:
            return (0, 0, 0, 0)

        if self._mode == "XY":
            hs = [m.x_mm for m in self._well_markers]
            vs = [m.y_mm for m in self._well_markers]
        elif self._mode == "ZY":
            hs = [m.z_offset_mm for m in self._well_markers]
            vs = [m.y_mm for m in self._well_markers]
        else:
            hs = [m.x_mm for m in self._well_markers]
            vs = [m.z_offset_mm for m in self._well_markers]

        mh = max((max(hs) - min(hs)) * 0.15, 3)
        mv = max((max(vs) - min(vs)) * 0.15, 3)
        return (min(hs) - mh, max(hs) + mh,
                min(vs) - mv, max(vs) + mv)

    # ── Well-scale rendering ──────────────────────────────────────

    def _draw_well_mode(self) -> None:
        s = self._scale

        # Well boundary
        if self._well_diameter_mm > 0:
            r = self._well_diameter_mm / 2 * s
            pen = QPen(QColor(WELL_BOUNDARY_COLOR), 1.5, Qt.PenStyle.DotLine)
            if self._mode == "XY":
                # Top-down: well circle.
                self._scene.addEllipse(-r, -r, 2 * r, 2 * r, pen)
            elif self._mode in ("XZ", "YZ") and self._well_depth_mm > 0:
                # v7.6.0: side/front views show the well *height*. The Z
                # axis runs upward — floor (z=0) at scene-y 0 (bottom),
                # rim (z=+depth) at scene-y -depth (top). Solid floor +
                # rim lines, dotted side walls, faint fill.
                depth = self._well_depth_mm * s
                fill = QColor(WELL_BOUNDARY_COLOR)
                fill.setAlpha(24)
                self._scene.addRect(-r, -depth, 2 * r, depth,
                                    QPen(Qt.PenStyle.NoPen), QBrush(fill))
                # Dotted side walls
                self._scene.addLine(-r, -depth, -r, 0, pen)
                self._scene.addLine(r, -depth, r, 0, pen)
                # Rim (top, z=depth) — dotted
                self._scene.addLine(-r, -depth, r, -depth, pen)
                # Floor (bottom, z=0) — solid, emphasized
                floor_pen = QPen(QColor(WELL_BOUNDARY_COLOR), 2.0,
                                 Qt.PenStyle.SolidLine)
                self._scene.addLine(-r, 0, r, 0, floor_pen)
            else:
                # ZY or no depth available: just the rim line.
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

        # Completed path (solid green)
        if len(self._completed_points) >= 2:
            path = QPainterPath()
            pts = list(self._completed_points)
            path.moveTo(pts[0][0] * s, pts[0][1] * s)
            for h, v in pts[1:]:
                path.lineTo(h * s, v * s)
            pen = QPen(QColor(COMPLETED_PATH_COLOR), PATH_PEN_WIDTH)
            self._scene.addPath(path, pen)

        # Upcoming waypoints (dashed yellow)
        if len(self._upcoming_points) >= 2:
            path = QPainterPath()
            path.moveTo(
                self._upcoming_points[0][0] * s,
                self._upcoming_points[0][1] * s)
            for h, v in self._upcoming_points[1:]:
                path.lineTo(h * s, v * s)
            pen = QPen(QColor(UPCOMING_PATH_COLOR), UPCOMING_PEN_WIDTH)
            pen.setDashPattern(UPCOMING_DASH_PATTERN)
            self._scene.addPath(path, pen)

        dot_pen = QPen(QColor(UPCOMING_PATH_COLOR), 1)
        dot_brush = QBrush(QColor(UPCOMING_PATH_COLOR))
        for h, v in self._upcoming_points:
            self._scene.addEllipse(
                h * s - 2, v * s - 2, 4, 4, dot_pen, dot_brush)

        self._draw_needle_crosshair(s)

    # ── Plate-scale rendering ─────────────────────────────────────

    def _draw_plate_mode(self) -> None:
        s = self._scale
        wp = QPen(QColor(WELL_BOTTOM_COLOR), 1)
        wb = QBrush(QColor(WELL_BOTTOM_COLOR))

        for m in self._well_markers:
            h, v = self._project(m.x_mm, m.y_mm, m.z_offset_mm)
            sx, sy = h * s, v * s

            if self._mode == "XY":
                r = m.well_diameter_mm * s * 0.15
                self._scene.addEllipse(
                    sx - r, sy - r, 2 * r, 2 * r, wp, wb)
            elif self._mode == "ZY":
                wh = m.well_diameter_mm * s * 0.3
                self._scene.addRect(QRectF(
                    sx - WELL_BOTTOM_RECT_W / 2, sy - wh / 2,
                    WELL_BOTTOM_RECT_W, wh), wp, wb)
            else:
                ww = m.well_diameter_mm * s * 0.3
                self._scene.addRect(QRectF(
                    sx - ww / 2, sy - WELL_BOTTOM_RECT_W / 2,
                    ww, WELL_BOTTOM_RECT_W), wp, wb)

        self._draw_needle_crosshair(s)

    # ── Needle crosshair ──────────────────────────────────────────

    def _draw_needle_crosshair(self, s: float) -> None:
        if self._needle_pos is None:
            return
        nh, nv = self._needle_pos
        sx, sy = nh * s, nv * s

        if self._tracking_error_mm > 0.01:
            er = self._tracking_error_mm * s
            ep = QPen(QColor(TRACKING_ERROR_COLOR), 1, Qt.PenStyle.DotLine)
            self._scene.addEllipse(sx - er, sy - er, 2 * er, 2 * er, ep)

        np_ = QPen(QColor(NEEDLE_COLOR), 2)
        self._scene.addLine(
            sx - CROSSHAIR_SIZE, sy, sx + CROSSHAIR_SIZE, sy, np_)
        self._scene.addLine(
            sx, sy - CROSSHAIR_SIZE, sx, sy + CROSSHAIR_SIZE, np_)
        self._scene.addEllipse(
            sx - NEEDLE_DOT_RADIUS, sy - NEEDLE_DOT_RADIUS,
            NEEDLE_DOT_RADIUS * 2, NEEDLE_DOT_RADIUS * 2,
            QPen(QColor(NEEDLE_COLOR), 1), QBrush(QColor(NEEDLE_COLOR)))

    # ── Events ────────────────────────────────────────────────────

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._fit_view()

    def mouseDoubleClickEvent(self, event) -> None:
        self.promote_requested.emit(self._mode)
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event) -> None:
        # v7.5.1: wheel zooms (and the row-canvas broadcasts the factor
        # so all three projection panes stay locked at the same scale).
        # Without this override Qt's default wheelEvent scrolls, which
        # collides with the canvas-level scrollbar sync.
        delta = event.angleDelta().y()
        if delta == 0:
            super().wheelEvent(event)
            return
        zoom_in = delta > 0
        factor = 1.15 if zoom_in else 1.0 / 1.15
        self.scale(factor, factor)
        self.zoom_changed.emit(factor)
        event.accept()

    def apply_external_zoom(self, factor: float) -> None:
        """Apply a zoom factor without re-broadcasting (peer panes
        receive this from the canvas during row-layout sync)."""
        self.scale(factor, factor)


# ═══════════════════════════════════════════════════════════════════
# L-Shaped Triple Projection Canvas (with view-swap support)
# ═══════════════════════════════════════════════════════════════════

_SIDE_ORDER = {
    "XY": ("ZY", "XZ"),
    "ZY": ("XZ", "XY"),
    "XZ": ("XY", "ZY"),
}


class ProjectionCanvas(QWidget):
    """
    Triple-projection widget. Supports two layout modes:

    * ``"L"`` (default) — XY primary top-left, ZY tall right, XZ wide
      bottom. Double-click a side pane to promote it.
    * ``"row"`` (v7.5.1) — XY, XZ, YZ side-by-side at equal stretch.
      Pan and zoom are **synchronized** across panes along their
      shared axes so the trio reads as a single coupled view.
    """

    primary_view_changed = Signal(str)

    def __init__(
        self,
        scale_mode: str = "well",
        panes: dict[str, ProjectionPane] | None = None,
        parent: QWidget | None = None,
        layout_mode: str = "L",
    ):
        super().__init__(parent)
        self._scale_mode = scale_mode
        self._layout_mode = layout_mode
        self._primary_mode = "XY"
        self._syncing_view = False  # recursion guard for pan/zoom sync

        if panes is None:
            if layout_mode == "row":
                # Horizontal trio: XY | XZ | YZ
                self._panes: dict[str, ProjectionPane] = {
                    "XY": ProjectionPane("XY", scale_mode),
                    "XZ": ProjectionPane("XZ", scale_mode),
                    "YZ": ProjectionPane("YZ", scale_mode),
                }
            else:
                self._panes = {
                    "XY": ProjectionPane("XY", scale_mode),
                    "ZY": ProjectionPane("ZY", scale_mode),
                    "XZ": ProjectionPane("XZ", scale_mode),
                }
        else:
            self._panes = panes

        for pane in self._panes.values():
            pane.promote_requested.connect(self.set_primary_view)

        self.xy_view = self._panes["XY"]
        # Side-pane convenience handles — keep both keys around even
        # when the layout doesn't actually use one, for code that
        # introspects ``self.zy_view`` / ``self.xz_view``.
        self.zy_view = self._panes.get("ZY")
        self.xz_view = self._panes.get("XZ")
        self.yz_view = self._panes.get("YZ")
        self._views = list(self._panes.values())

        # Row-mode view-visibility state (toggled via the top button bar).
        self._row_modes = ("XY", "XZ", "YZ")
        self._visible_modes = list(self._row_modes)
        self._view_toggle_btns: dict[str, QPushButton] = {}

        self._main_layout = QVBoxLayout(self)
        self._main_layout.setContentsMargins(0, 0, 0, 0)
        self._main_layout.setSpacing(2)

        # v7.6.0: a row of XY / XZ / YZ toggle chips above the panes lets
        # the user pick which projections are shown; the chosen views
        # share the full width evenly.
        if layout_mode == "row":
            self._main_layout.addWidget(self._build_view_toggle_bar())

        # Panes live in a dedicated host so the toggle bar persists
        # across ``_rebuild_layout`` (which wipes & recreates the host).
        self._panes_host = QWidget(self)
        self._host_layout = QVBoxLayout(self._panes_host)
        self._host_layout.setContentsMargins(0, 0, 0, 0)
        self._host_layout.setSpacing(2)
        self._main_layout.addWidget(self._panes_host, 1)

        self._rebuild_layout()
        if layout_mode == "row":
            self._wire_row_sync()

    # ── View swapping ─────────────────────────────────────────────

    def set_primary_view(self, mode: str) -> None:
        if mode not in self._panes or mode == self._primary_mode:
            return
        self._primary_mode = mode
        self._rebuild_layout()
        self.primary_view_changed.emit(mode)

    def _rebuild_layout(self) -> None:
        host_layout = getattr(self, "_host_layout", self._main_layout)

        # Detach panes
        for pane in self._panes.values():
            pane.setParent(None)

        # Remove old widgets from the panes host layout
        while host_layout.count():
            item = host_layout.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)
                w.deleteLater()

        if self._layout_mode == "row":
            # v7.5.1: equal-width horizontal row of panes. v7.6.0: only
            # the toggled-on views are shown; they share the full width.
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(2)
            for mode in self._row_modes:
                if mode not in self._visible_modes:
                    continue
                pane = self._panes.get(mode)
                if pane is None:
                    continue
                pane._apply_position_sizing("primary")
                # Equal stretch so all shown views share width evenly.
                row.addWidget(pane, 1)
            container = QWidget(self)
            container.setLayout(row)
            host_layout.addWidget(container, stretch=1)
            for mode, pane in self._panes.items():
                pane.setVisible(mode in self._visible_modes)
                if mode in self._visible_modes:
                    pane.refresh()
            return

        # ── L-shaped layout (legacy default) ──────────────────────
        primary = self._panes[self._primary_mode]
        sr_mode, sb_mode = _SIDE_ORDER[self._primary_mode]
        side_r = self._panes[sr_mode]
        side_b = self._panes[sb_mode]

        primary._apply_position_sizing("primary")
        side_r._apply_position_sizing("side_right")
        side_b._apply_position_sizing("side_bottom")

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(primary)
        top.addWidget(side_r)
        top.setStretchFactor(0, 4)
        top.setStretchFactor(1, 1)

        host_layout.addWidget(top, stretch=3)
        host_layout.addWidget(side_b, stretch=1)

        for pane in self._panes.values():
            pane.refresh()

    # ── View-visibility toggle bar (v7.6.0) ───────────────────────

    def _build_view_toggle_bar(self) -> QWidget:
        """A compact row of XY / XZ / YZ toggle chips. The chosen views
        share the full canvas width; deselecting all is disallowed."""
        bar = QWidget(self)
        bar.setObjectName("projViewToggleBar")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(_s(6), _s(4), _s(6), _s(4))
        lay.setSpacing(_s(6))

        caption = QLabel("Views:")
        caption.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9.5)}pt;")
        lay.addWidget(caption)

        for mode in self._row_modes:
            btn = QPushButton(mode)
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedHeight(_s(24))
            btn.setMinimumWidth(_s(44))
            btn.setStyleSheet(self._toggle_chip_style())
            btn.clicked.connect(
                lambda _checked, m=mode: self._on_view_toggled(m))
            lay.addWidget(btn)
            self._view_toggle_btns[mode] = btn

        lay.addStretch(1)
        return bar

    @staticmethod
    def _toggle_chip_style() -> str:
        return (
            f"QPushButton {{"
            f"  background: {COLORS['surface0']};"
            f"  color: {COLORS['subtext0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {_s(6)}px;"
            f"  padding: {_s(2)}px {_s(10)}px;"
            f"  font-weight: 700; font-size: {_sf(9.5)}pt;"
            f"}}"
            f"QPushButton:hover {{ border-color: {COLORS['mauve']}; }}"
            f"QPushButton:checked {{"
            f"  background: {COLORS['mauve']};"
            f"  color: {COLORS['base']};"
            f"  border-color: {COLORS['mauve']};"
            f"}}"
        )

    def _on_view_toggled(self, mode: str) -> None:
        visible = [m for m in self._row_modes
                   if self._view_toggle_btns[m].isChecked()]
        if not visible:
            # Never allow zero panes — re-check the one just turned off.
            self._view_toggle_btns[mode].setChecked(True)
            return
        self._visible_modes = visible
        self._rebuild_layout()

    def set_visible_views(self, modes: list[str]) -> None:
        """Programmatically set which row-mode views are shown."""
        modes = [m for m in self._row_modes if m in modes]
        if not modes:
            return
        self._visible_modes = modes
        for m, btn in self._view_toggle_btns.items():
            btn.blockSignals(True)
            btn.setChecked(m in modes)
            btn.blockSignals(False)
        self._rebuild_layout()

    # ── Row-layout view sync (v7.5.1) ─────────────────────────────

    def _wire_row_sync(self) -> None:
        """Lock pan + zoom across the XY/XZ/YZ panes along their
        shared axes.

        Shared-axis map (row layout):
            XY h (X)  ↔  XZ h (X)
            XY v (Y)  ↔  YZ h (Y)
            XZ v (Z)  ↔  YZ v (Z)
        """
        xy = self._panes.get("XY")
        xz = self._panes.get("XZ")
        yz = self._panes.get("YZ")
        if xy is None or xz is None or yz is None:
            return

        # Zoom — every pane broadcasts the factor, peers apply it.
        for src in (xy, xz, yz):
            src.zoom_changed.connect(
                lambda factor, origin=src: self._broadcast_zoom(origin, factor)
            )

        # Pan — wire scrollbars per shared axis.
        # XY h ↔ XZ h (both X)
        xy.horizontalScrollBar().valueChanged.connect(
            lambda v: self._sync_scroll(xy, xz, "h", "h", v))
        xz.horizontalScrollBar().valueChanged.connect(
            lambda v: self._sync_scroll(xz, xy, "h", "h", v))
        # XY v ↔ YZ h (XY's Y is YZ's horizontal)
        xy.verticalScrollBar().valueChanged.connect(
            lambda v: self._sync_scroll(xy, yz, "v", "h", v))
        yz.horizontalScrollBar().valueChanged.connect(
            lambda v: self._sync_scroll(yz, xy, "h", "v", v))
        # XZ v ↔ YZ v (both Z)
        xz.verticalScrollBar().valueChanged.connect(
            lambda v: self._sync_scroll(xz, yz, "v", "v", v))
        yz.verticalScrollBar().valueChanged.connect(
            lambda v: self._sync_scroll(yz, xz, "v", "v", v))

    def _broadcast_zoom(self, origin: ProjectionPane, factor: float) -> None:
        if self._syncing_view:
            return
        self._syncing_view = True
        try:
            for pane in self._panes.values():
                if pane is origin:
                    continue
                pane.apply_external_zoom(factor)
        finally:
            self._syncing_view = False

    def _sync_scroll(
        self,
        src: ProjectionPane,
        dst: ProjectionPane,
        src_axis: str,
        dst_axis: str,
        value: int,
    ) -> None:
        if self._syncing_view:
            return
        self._syncing_view = True
        try:
            # Translate src scrollbar value → equivalent dst scrollbar
            # value. The two panes typically have the same scene scale
            # at the same zoom (we broadcast zoom uniformly), so the
            # scrollbar ranges should match; if not, scale by ratio.
            src_bar = (src.horizontalScrollBar() if src_axis == "h"
                       else src.verticalScrollBar())
            dst_bar = (dst.horizontalScrollBar() if dst_axis == "h"
                       else dst.verticalScrollBar())
            src_range = max(1, src_bar.maximum() - src_bar.minimum())
            dst_range = max(1, dst_bar.maximum() - dst_bar.minimum())
            rel = (value - src_bar.minimum()) / src_range
            dst_value = dst_bar.minimum() + int(round(rel * dst_range))
            dst_bar.setValue(dst_value)
        finally:
            self._syncing_view = False

    # ══════════════════════════════════════════════════════════════
    #  WELL-SCALE API
    # ══════════════════════════════════════════════════════════════

    def set_well_diameter(self, diameter_mm: float) -> None:
        for v in self._views:
            v.set_well_diameter(diameter_mm)

    def set_well_depth(self, depth_mm: float) -> None:
        """v7.6.0: broadcast well depth to all panes (XZ/YZ use it
        to draw the well-height outline)."""
        for v in self._views:
            v.set_well_depth(depth_mm)
        for v in self._views:
            v.refresh()

    def add_completed_point(self, x: float, y: float, z: float) -> None:
        for v in self._views:
            v.add_completed_point(x, y, z)

    def set_upcoming_waypoints(
        self, waypoints: list[tuple[float, float, float]],
    ) -> None:
        for v in self._views:
            v.set_upcoming_waypoints(waypoints)

    def set_object_paths(self, paths: list[ObjectPath]) -> None:
        for v in self._views:
            v.set_object_paths(paths)

    def clear_object_paths(self) -> None:
        for v in self._views:
            v.clear_object_paths()

    # ══════════════════════════════════════════════════════════════
    #  PLATE-SCALE API
    # ══════════════════════════════════════════════════════════════

    def set_well_markers(self, markers: list[WellBottomMarker]) -> None:
        for v in self._views:
            v.set_well_markers(markers)

    def clear_well_markers(self) -> None:
        for v in self._views:
            v.clear_well_markers()

    # ══════════════════════════════════════════════════════════════
    #  COMMON API
    # ══════════════════════════════════════════════════════════════

    def set_needle_position(
        self, x: float | None, y: float | None, z: float | None,
    ) -> None:
        for v in self._views:
            v.set_needle_position(x, y, z)

    def set_tracking_error(self, error_mm: float) -> None:
        for v in self._views:
            v.set_tracking_error(error_mm)

    def clear_path(self) -> None:
        for v in self._views:
            v.clear_path()

    def refresh(self) -> None:
        for v in self._views:
            v.refresh()

    def set_scale_mode(self, mode: str) -> None:
        self._scale_mode = mode
        for v in self._views:
            v.set_scale_mode(mode)


# ═══════════════════════════════════════════════════════════════════
# Convenience factory functions
# ═══════════════════════════════════════════════════════════════════

def create_well_preview() -> ProjectionCanvas:
    return ProjectionCanvas(scale_mode="well")


def create_plate_projection() -> ProjectionCanvas:
    return ProjectionCanvas(scale_mode="plate")


# ═══════════════════════════════════════════════════════════════════
# Backward-Compatible MiniProjectionView Adapter
# ═══════════════════════════════════════════════════════════════════

class MiniProjectionView(ProjectionPane):
    """
    Drop-in replacement for the inline MiniProjectionView that was
    previously in print_well_setup.py.
    """

    def __init__(self, mode: str = "ZY", parent: QWidget | None = None):
        super().__init__(mode=mode, scale_mode="plate", parent=parent)

    def set_plate_data(
        self, plate, z_offsets: dict[str, float] | None = None,
    ) -> None:
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
                    name=w.name, x_mm=w.x, y_mm=w.y,
                    z_offset_mm=z_off.get(w.name, 0.0),
                    well_diameter_mm=diameter))
        except Exception:
            logger.debug("Failed to read plate wells for projection")

        self.set_well_markers(markers)
        self.refresh()


# ═══════════════════════════════════════════════════════════════════
# Axis mapping for mode-aware dragging
# ═══════════════════════════════════════════════════════════════════
#
# Each projection mode uses two of the three spatial axes (x, y, z)
# for its horizontal and vertical scene directions:
#
#   Mode   h-axis   v-axis
#   ─────  ──────   ──────
#   XY     x        y
#   ZY     z        y
#   XZ     x        z
#
# When the user drags an item, the scene delta is written back to
# the PlacedObject offset along the *corresponding* mm axes.
# When syncing, each pane reads the PlacedObject offsets it cares
# about and sets its item position accordingly.

def _project_point(pt: tuple[float, float, float],
                   mode: str) -> tuple[float, float]:
    """Project a 3D point onto the two axes of *mode*.

    v7.6.0: in XZ/YZ the vertical scene axis is Z drawn *upward* (floor
    z=0 at the bottom), so z is negated to match Qt's downward scene-y —
    consistent with ``ProjectionPane._project``."""
    x, y, z = pt
    if mode == "XY":
        return (x, y)
    elif mode == "ZY":
        return (z, y)
    elif mode == "YZ":
        return (y, -z)
    return (x, -z)  # XZ


def _offset_h_v(placed: PlacedObject,
                mode: str) -> tuple[float, float]:
    """Return (h_mm, v_mm) scene offset for *mode* (z negated in XZ/YZ
    so +z renders upward — see ``_project_point``)."""
    if mode == "XY":
        return (placed.x_offset, placed.y_offset)
    elif mode == "ZY":
        return (placed.z_offset, placed.y_offset)
    elif mode == "YZ":
        return (placed.y_offset, -placed.z_offset)
    return (placed.x_offset, -placed.z_offset)  # XZ


def _write_offset_h_v(placed: PlacedObject,
                      mode: str, h_mm: float, v_mm: float) -> None:
    """Write a (h_mm, v_mm) scene offset back onto the right PlacedObject
    fields (z negated in XZ/YZ to undo the upward-Z scene mapping)."""
    if mode == "XY":
        placed.x_offset = h_mm
        placed.y_offset = v_mm
    elif mode == "ZY":
        placed.z_offset = h_mm
        placed.y_offset = v_mm
    elif mode == "YZ":
        placed.y_offset = h_mm
        placed.z_offset = -v_mm
    else:  # XZ
        placed.x_offset = h_mm
        placed.z_offset = -v_mm


# ═══════════════════════════════════════════════════════════════════
# Draggable Object Item (mode-aware)
# ═══════════════════════════════════════════════════════════════════

class DraggableObjectItem(QGraphicsPathItem):
    """
    A movable print-object path rendered in a specific projection.

    The *mode* determines which 3D axes are visible and which
    PlacedObject offset components are written on drag.

    Click-drag to reposition.  Right-click to remove.
    """

    def __init__(
        self,
        placed: PlacedObject,
        mode: str = "XY",
        scale: float = WELL_SCALE,
        parent: QGraphicsItem | None = None,
    ):
        super().__init__(parent)
        self.placed = placed
        self._mode = mode
        self._scale = scale
        self._suppress_sync = False   # prevents recursive sync loops

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setAcceptHoverEvents(True)
        self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))
        self._rebuild_path()

    # ── Path + position ───────────────────────────────────────────

    def _rebuild_path(self) -> None:
        """Build QPainterPath projected to this view's axes."""
        path = QPainterPath()
        pts = self.placed.points
        s = self._scale
        color = QColor(self.placed.color)

        # Sphere rendering for point objects
        if self.placed.sphere_radius_mm > 0 and len(pts) >= 1:
            h0, v0 = _project_point(pts[0], self._mode)
            r = self.placed.sphere_radius_mm * s
            r = max(r, 2)  # minimum visible size
            path.addEllipse(h0 * s - r, v0 * s - r, r * 2, r * 2)
            self.setPath(path)
            self.setPen(QPen(color, 1.0))
            fill = QColor(color)
            fill.setAlpha(120)
            self.setBrush(QBrush(fill))
            self.sync_position_from_placed()
            return

        if len(pts) >= 2:
            h0, v0 = _project_point(pts[0], self._mode)
            path.moveTo(h0 * s, v0 * s)
            for p in pts[1:]:
                h, v = _project_point(p, self._mode)
                path.lineTo(h * s, v * s)
        elif len(pts) == 1:
            h0, v0 = _project_point(pts[0], self._mode)
            r = 3
            path.addEllipse(h0 * s - r, v0 * s - r, r * 2, r * 2)
        self.setPath(path)
        self.setPen(QPen(color, PATH_PEN_WIDTH))
        self.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        self.sync_position_from_placed()

    def sync_position_from_placed(self) -> None:
        """Set scene position from the PlacedObject offset (no recursion)."""
        self._suppress_sync = True
        h_mm, v_mm = _offset_h_v(self.placed, self._mode)
        self.setPos(h_mm * self._scale, v_mm * self._scale)
        self._suppress_sync = False

    # ── Paint ─────────────────────────────────────────────────────

    def paint(self, painter, option, widget=None) -> None:
        super().paint(painter, option, widget)
        if self.isSelected():
            br = self.boundingRect()
            painter.setPen(QPen(
                QColor(SELECTION_COLOR), 1.5, Qt.PenStyle.DashLine))
            painter.drawRect(br)

    # ── Drag feedback ─────────────────────────────────────────────

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            # Clamp the proposed scene position to the pane's scene rect
            # so the object can't be flung off into empty space (where it
            # becomes hard to retrieve). value is the candidate QPointF.
            scene = self.scene()
            if scene is not None:
                sr = scene.sceneRect()
                if sr.isValid() and not sr.isEmpty():
                    x = min(max(value.x(), sr.left()), sr.right())
                    y = min(max(value.y(), sr.top()), sr.bottom())
                    if x != value.x() or y != value.y():
                        return QPointF(x, y)
            return value
        if (change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged
                and not self._suppress_sync):
            h_mm = value.x() / self._scale
            v_mm = value.y() / self._scale
            _write_offset_h_v(self.placed, self._mode, h_mm, v_mm)
            # Notify the parent pane (which forwards to the canvas)
            scene = self.scene()
            if scene:
                for view in scene.views():
                    if isinstance(view, InteractiveProjectionPane):
                        view._on_item_moved(self)
                        break
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        self.setCursor(QCursor(Qt.CursorShape.ClosedHandCursor))
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))
        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event):
        menu = QMenu()
        del_action = menu.addAction(f"Remove '{self.placed.name}'")
        action = menu.exec(event.screenPos())
        if action == del_action:
            scene = self.scene()
            if scene:
                for view in scene.views():
                    if isinstance(view, InteractiveProjectionPane):
                        view._request_remove(self.placed)
                        break


# ═══════════════════════════════════════════════════════════════════
# Interactive Projection Pane (works for ANY mode: XY, ZY, XZ)
# ═══════════════════════════════════════════════════════════════════

class InteractiveProjectionPane(ProjectionPane):
    """
    Projection pane with drag-drop support for object placement.

    Works for any mode (XY, ZY, XZ).  Accepts drops of
    OBJECT_MIME_TYPE from the library list and creates
    DraggableObjectItems that users can reposition by dragging.

    The pane does NOT own the PlacedObject list — that belongs
    to the InteractiveProjectionCanvas.  This pane only owns
    the QGraphicsItem wrappers.
    """

    # Signals forwarded to InteractiveProjectionCanvas
    _item_moved_sig = Signal(object, str)       # (PlacedObject, source_mode)
    _drop_requested_sig = Signal(str, str, float, float)  # lib_key, mode, h, v
    _remove_requested_sig = Signal(object)      # PlacedObject

    def __init__(self, mode: str = "XY", parent=None):
        super().__init__(mode=mode, scale_mode="well", parent=parent)
        self.setAcceptDrops(True)
        self._placed_items: dict[int, DraggableObjectItem] = {}
        # key = id(PlacedObject) → DraggableObjectItem

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

        scene_pos = self.mapToScene(event.position().toPoint())
        h_mm = scene_pos.x() / self._scale
        v_mm = scene_pos.y() / self._scale

        event.acceptProposedAction()
        # Delegate to canvas (which creates PlacedObject + items in ALL panes)
        self._drop_requested_sig.emit(library_key, self._mode, h_mm, v_mm)

    # ── Item management (called by canvas) ────────────────────────

    def add_item_for(self, placed: PlacedObject) -> None:
        """Create a DraggableObjectItem for the given PlacedObject."""
        item = DraggableObjectItem(placed, self._mode, self._scale)
        self._scene.addItem(item)
        self._placed_items[id(placed)] = item

    def remove_item_for(self, placed: PlacedObject) -> None:
        """Remove the DraggableObjectItem linked to *placed*."""
        item = self._placed_items.pop(id(placed), None)
        if item and item.scene():
            self._scene.removeItem(item)

    def sync_item_positions(self, exclude_placed: PlacedObject | None = None) -> None:
        """
        Re-read offsets from each PlacedObject and update item scene
        positions (used after a drag in another pane).
        """
        for pid, item in self._placed_items.items():
            if exclude_placed is not None and pid == id(exclude_placed):
                continue  # this is the pane being dragged — skip
            item.sync_position_from_placed()

    def clear_all_items(self) -> None:
        for item in self._placed_items.values():
            if item.scene():
                self._scene.removeItem(item)
        self._placed_items.clear()

    # ── Internal callbacks from DraggableObjectItem ───────────────

    def _on_item_moved(self, item: DraggableObjectItem) -> None:
        """Called by DraggableObjectItem.itemChange during drag."""
        self._item_moved_sig.emit(item.placed, self._mode)

    def _request_remove(self, placed: PlacedObject) -> None:
        """Called by DraggableObjectItem context menu."""
        self._remove_requested_sig.emit(placed)

    # ── Redraw override (preserve draggable items) ────────────────

    def _redraw(self) -> None:
        saved = list(self._placed_items.values())
        for item in saved:
            if item.scene():
                self._scene.removeItem(item)

        super()._redraw()  # grid + boundary + static paths

        for item in saved:
            self._scene.addItem(item)


# ═══════════════════════════════════════════════════════════════════
# Interactive Projection Canvas (all 3 panes draggable)
# ═══════════════════════════════════════════════════════════════════

class InteractiveProjectionCanvas(ProjectionCanvas):
    """
    ProjectionCanvas where **all three panes** support drag-drop.

    Drag an object in the XY view and the ZY and XZ views update
    live.  Drag in ZY and the other two follow.  Drop into any pane.

    The canvas owns the canonical list of PlacedObjects.  Each pane
    holds only the QGraphicsItem wrappers pointing to the same
    PlacedObject instances, so offset writes in one pane are
    instantly visible when the others re-read.

    Signals:
        object_placed(name, x_mm, y_mm)  — object dropped (XY offsets)
        object_moved(name, x_mm, y_mm)   — repositioned   (XY offsets)
        object_removed(name)             — deleted via right-click
        primary_view_changed(mode)       — inherited
    """

    object_placed = Signal(str, float, float)
    object_moved = Signal(str, float, float)
    object_removed = Signal(str)

    def __init__(
        self,
        parent: QWidget | None = None,
        layout_mode: str = "L",
    ):
        if layout_mode == "row":
            panes = {
                "XY": InteractiveProjectionPane("XY"),
                "XZ": InteractiveProjectionPane("XZ"),
                "YZ": InteractiveProjectionPane("YZ"),
            }
        else:
            panes = {
                "XY": InteractiveProjectionPane("XY"),
                "ZY": InteractiveProjectionPane("ZY"),
                "XZ": InteractiveProjectionPane("XZ"),
            }
        super().__init__(
            scale_mode="well", panes=panes, parent=parent,
            layout_mode=layout_mode,
        )

        # Canonical placed-object list
        self._placed_objects: list[PlacedObject] = []
        self._syncing = False  # recursion guard

        # Wire internal signals from every pane
        for mode, pane in self._panes.items():
            if isinstance(pane, InteractiveProjectionPane):
                pane._item_moved_sig.connect(self._on_item_moved)
                pane._drop_requested_sig.connect(self._on_drop_requested)
                pane._remove_requested_sig.connect(self._on_remove_requested)

    # ── Drop handling ─────────────────────────────────────────────

    def set_library_resolver(self, fn) -> None:
        """fn(library_key, x_mm, y_mm) → PlacedObject | None."""
        self._resolve_library = fn

    def _on_drop_requested(
        self, library_key: str, source_mode: str,
        h_mm: float, v_mm: float,
    ) -> None:
        """A pane received a drop — create the PlacedObject centrally."""
        resolver = getattr(self, '_resolve_library', None)
        if resolver is None:
            return

        # Map drop (h, v) to (x, y) offsets for the resolver
        # (resolver always receives x_mm, y_mm regardless of pane)
        if source_mode == "XY":
            x_mm, y_mm = h_mm, v_mm
        elif source_mode == "ZY":
            x_mm, y_mm = 0.0, v_mm
        elif source_mode == "YZ":     # v7.5.1: h=Y, v=Z
            x_mm, y_mm = 0.0, h_mm
        else:  # XZ
            x_mm, y_mm = h_mm, 0.0

        placed = resolver(library_key, x_mm, y_mm)
        if placed is None:
            return

        # If dropped into a side view, the orthogonal axis becomes Z
        if source_mode == "ZY":
            placed.z_offset = h_mm
        elif source_mode == "YZ":     # v7.5.1
            placed.z_offset = v_mm
        elif source_mode == "XZ":
            placed.z_offset = v_mm

        self._add_placed_object(placed)
        self.object_placed.emit(placed.name, placed.x_offset, placed.y_offset)

    # ── Move handling (cross-pane sync) ───────────────────────────

    def _on_item_moved(self, placed: PlacedObject, source_mode: str) -> None:
        """A drag happened in *source_mode* — sync the other panes."""
        if self._syncing:
            return
        self._syncing = True
        try:
            for mode, pane in self._panes.items():
                if (mode != source_mode
                        and isinstance(pane, InteractiveProjectionPane)):
                    pane.sync_item_positions()
        finally:
            self._syncing = False
        self.object_moved.emit(
            placed.name, placed.x_offset, placed.y_offset)

    # ── Remove handling ───────────────────────────────────────────

    def _on_remove_requested(self, placed: PlacedObject) -> None:
        """Remove a placed object from all panes."""
        name = placed.name
        if placed in self._placed_objects:
            self._placed_objects.remove(placed)
        for pane in self._panes.values():
            if isinstance(pane, InteractiveProjectionPane):
                pane.remove_item_for(placed)
        self.object_removed.emit(name)

    # ── Central placed-object management ──────────────────────────

    def _add_placed_object(self, placed: PlacedObject) -> None:
        """Add to canonical list and create items in ALL panes."""
        self._placed_objects.append(placed)
        for pane in self._panes.values():
            if isinstance(pane, InteractiveProjectionPane):
                pane.add_item_for(placed)
                pane.refresh()

    def get_placed_objects(self) -> list[PlacedObject]:
        return list(self._placed_objects)

    def clear_placed_objects(self) -> None:
        self._placed_objects.clear()
        for pane in self._panes.values():
            if isinstance(pane, InteractiveProjectionPane):
                pane.clear_all_items()
                pane.refresh()


def create_interactive_well_preview() -> InteractiveProjectionCanvas:
    """Create an interactive well-scale canvas for object arrangement
    in the legacy L-shaped layout."""
    return InteractiveProjectionCanvas()


def create_horizontal_well_preview() -> InteractiveProjectionCanvas:
    """v7.5.1: Create an interactive well-scale canvas with the three
    projections in a horizontal row (XY | XZ | YZ), pan + zoom locked
    across panes."""
    return InteractiveProjectionCanvas(layout_mode="row")
