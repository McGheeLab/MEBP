"""
sketch_canvas.py — lightweight direct-manipulation canvas for the Print
Builder Sketch tool (v7.5.x).

Draws / edits a :class:`SupportClasses.SketchTrajectory.Sketch` of vector
primitives (line / rect / circle / ellipse / polygon). Deliberately *not*
built on the constraint-solving PlateDesignerCanvas — this is a plain
custom-painted ``QWidget`` with a simple pan/zoom transform, so drawing feels
direct (drag to create, drag to move/resize) and exact sizes are typed in the
properties panel.

Coordinates: world space is millimetres (y increases downward, matching the
2D preview). All shapes live in the shared ``Sketch`` model so the compiler
and properties panel see the same source of truth.
"""

from __future__ import annotations

import copy
import math
from enum import Enum, auto

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import (
    QPainter, QPen, QColor, QBrush, QPolygonF, QCursor,
)
from PySide6.QtWidgets import QWidget, QSizePolicy

from gui.styles import COLORS
from gui.scaling import s
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compute_fill_region,
)


class Tool(Enum):
    SELECT = auto()
    LINE = auto()
    RECT = auto()
    CIRCLE = auto()
    ELLIPSE = auto()
    POLYGON = auto()
    FILL = auto()      # paint-bucket: click inside an enclosed region


# Default per-pump colours for shapes.
PUMP_HEX = ["#89b4fa", "#a6e3a1", "#f9b387"]

_HANDLE_PX = 7          # resize-handle half-size (screen px, pre-scale)
_HIT_PX = 8             # hit-test tolerance (screen px, pre-scale)


class SketchCanvas(QWidget):
    """Interactive vector sketch surface bound to a ``Sketch`` model."""

    sketch_changed = Signal()
    selection_changed = Signal(int)   # selected shape index, or -1
    tool_changed = Signal(object)     # Tool
    fill_result = Signal(bool)        # paint-bucket success / not-enclosed

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(s(320), s(320))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self._sketch = Sketch()
        self._tool = Tool.SELECT
        self._selected = -1

        # View transform: screen = origin + world * scale (px per mm).
        self._scale = 8.0
        self._origin = QPointF(0, 0)       # screen px of world (0,0)
        self._origin_init = False

        self._snap_mm = 0.0                # grid snap step; 0 = off
        self._grid_mm = 5.0
        self._osnap = True                 # snap to existing object borders
        self._snap_marker = None           # world QPointF of active snap hit
        self._active_pump = 0              # pump for new shapes/fills
        self._default_line_width = 0.4     # bead width for new shapes (needle Ø)
        self._ref_well_d = 0.0             # reference standard-well Ø (mm); 0=off
        # Compiled toolpath shown as the main raster view. Each run is
        # (pump_index | -1 for travel, [(x,y) world mm, ...]).
        self._tp_runs: list[tuple[int, list[tuple[float, float]]]] = []

        # Interaction state
        self._mode = None                  # 'draw' | 'move' | 'resize' | 'pan' | 'poly'
        self._draw_start = None            # world QPointF
        self._draw_cur = None
        self._poly_pts: list[tuple[float, float]] = []
        self._move_anchor = None           # world QPointF at grab
        self._move_orig = None             # snapshot of shape at grab
        self._resize_kind = None           # which handle
        self._pan_start = None

        # Undo
        self._undo: list[dict] = []
        self._redo: list[dict] = []
        self._undo_cap = 50

    # ── Model access ──────────────────────────────────────────────

    def sketch(self) -> Sketch:
        return self._sketch

    def set_sketch(self, sketch: Sketch):
        self._sketch = sketch
        self._selected = -1
        self._undo.clear()
        self._redo.clear()
        self.selection_changed.emit(-1)
        self.update()

    def selected_index(self) -> int:
        return self._selected

    def selected_shape(self) -> SketchShape | None:
        if 0 <= self._selected < len(self._sketch.shapes):
            return self._sketch.shapes[self._selected]
        return None

    def set_selected(self, index: int):
        self._selected = index if 0 <= index < len(self._sketch.shapes) else -1
        self.selection_changed.emit(self._selected)
        self.update()

    # ── Tool / view controls ──────────────────────────────────────

    def set_tool(self, tool: Tool):
        self._tool = tool
        self._poly_pts = []
        self._mode = None
        if tool != Tool.SELECT:
            self._selected = -1
            self.selection_changed.emit(-1)
        self.tool_changed.emit(tool)
        self.update()

    def get_tool(self) -> Tool:
        return self._tool

    def set_snap(self, mm: float):
        self._snap_mm = max(0.0, mm)

    def set_object_snap(self, enabled: bool):
        self._osnap = bool(enabled)

    def set_active_pump(self, index: int):
        self._active_pump = max(0, min(2, int(index)))

    def set_default_line_width(self, mm: float):
        """Bead width applied to newly drawn shapes (typically needle Ø)."""
        if mm and mm > 0:
            self._default_line_width = float(mm)

    def set_reference_well(self, diameter_mm: float):
        """Draw a dashed standard-well outline (Ø mm) at the origin; 0 = off."""
        self._ref_well_d = max(0.0, float(diameter_mm or 0.0))
        self.update()

    def set_toolpath(self, trajectory, pump_states):
        """Set the compiled toolpath to render as the main raster view.

        ``trajectory`` is an Nx7 array (or None) in world mm; ``pump_states``
        is the parallel per-waypoint [f1,f2,f3] flow list. Segments are
        grouped into colored print runs / grey travel runs once here, so
        painting only maps world→screen.
        """
        self._tp_runs = []
        if trajectory is None or len(trajectory) < 2:
            self.update()
            return

        def seg_pump(i: int) -> int:
            st = pump_states[i] if pump_states and i < len(pump_states) else None
            if st and sum(st) > 0:
                return max(range(len(st)), key=lambda k: st[k])
            return -1

        n = len(trajectory)
        cats = [seg_pump(i) for i in range(n - 1)]
        i = 0
        while i < len(cats):
            cat = cats[i]
            j = i
            while j < len(cats) and cats[j] == cat:
                j += 1
            pts = [(float(trajectory[k][0]), float(trajectory[k][1]))
                   for k in range(i, j + 1)]
            self._tp_runs.append((cat, pts))
            i = j
        self.update()

    def fit_view(self):
        """Center the view; frame existing geometry if any."""
        b = self._content_bounds()
        w, h = max(1, self.width()), max(1, self.height())
        if b is None:
            self._scale = 8.0
            self._origin = QPointF(w / 2, h / 2)
        else:
            minx, miny, maxx, maxy = b
            span_x = max(maxx - minx, 1.0)
            span_y = max(maxy - miny, 1.0)
            margin = 0.85
            self._scale = max(1.0, min((w * margin) / span_x,
                                       (h * margin) / span_y))
            cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
            self._origin = QPointF(w / 2 - cx * self._scale,
                                   h / 2 - cy * self._scale)
        self._origin_init = True
        self.update()

    # ── Undo / redo ───────────────────────────────────────────────

    def _snapshot(self):
        self._undo.append(self._sketch.to_dict())
        if len(self._undo) > self._undo_cap:
            self._undo.pop(0)
        self._redo.clear()

    def can_undo(self) -> bool:
        return bool(self._undo)

    def can_redo(self) -> bool:
        return bool(self._redo)

    def undo(self):
        if not self._undo:
            return
        self._redo.append(self._sketch.to_dict())
        self._restore(self._undo.pop())

    def redo(self):
        if not self._redo:
            return
        self._undo.append(self._sketch.to_dict())
        self._restore(self._redo.pop())

    def _restore(self, snap: dict):
        self._sketch = Sketch.from_dict(snap)
        self._selected = min(self._selected, len(self._sketch.shapes) - 1)
        self.selection_changed.emit(self._selected)
        self.sketch_changed.emit()
        self.update()

    def delete_selected(self):
        if 0 <= self._selected < len(self._sketch.shapes):
            self._snapshot()
            self._sketch.shapes.pop(self._selected)
            self._selected = -1
            self.selection_changed.emit(-1)
            self.sketch_changed.emit()
            self.update()

    # ── Coordinate transforms ─────────────────────────────────────

    def _w2s(self, x: float, y: float) -> QPointF:
        return QPointF(self._origin.x() + x * self._scale,
                       self._origin.y() + y * self._scale)

    def _s2w(self, px: float, py: float) -> QPointF:
        return QPointF((px - self._origin.x()) / self._scale,
                       (py - self._origin.y()) / self._scale)

    def _snap(self, p: QPointF, exclude: int = -1) -> QPointF:
        # Object/border snap takes priority over grid snap. Snap targets are
        # vertices/centers (exact) and the nearest point on each border.
        self._snap_marker = None
        if self._osnap:
            tol = s(_HIT_PX + 2) / self._scale
            best = None
            best_d = tol
            # Vertices/centers first (tight), then edges.
            for vx, vy in self._snap_vertices(exclude):
                d = math.hypot(vx - p.x(), vy - p.y())
                if d < best_d:
                    best_d, best = d, QPointF(vx, vy)
            if best is None:
                for ex, ey in self._snap_edges(p, exclude):
                    d = math.hypot(ex - p.x(), ey - p.y())
                    if d < best_d:
                        best_d, best = d, QPointF(ex, ey)
            if best is not None:
                self._snap_marker = best
                return best
        if self._snap_mm > 0:
            return QPointF(round(p.x() / self._snap_mm) * self._snap_mm,
                           round(p.y() / self._snap_mm) * self._snap_mm)
        return p

    def _snap_vertices(self, exclude: int):
        """Exact snap points: corners, endpoints, polygon vertices, centers."""
        out = []
        for i, sh in enumerate(self._sketch.shapes):
            if i == exclude or sh.kind == "region":
                continue
            if sh.kind in ("circle", "ellipse", "rect"):
                out.append((sh.cx, sh.cy))
            if sh.kind == "rect":
                hw, hh = sh.width / 2, sh.height / 2
                out += [(sh.cx - hw, sh.cy - hh), (sh.cx + hw, sh.cy - hh),
                        (sh.cx + hw, sh.cy + hh), (sh.cx - hw, sh.cy + hh)]
            elif sh.kind in ("line", "polygon"):
                out += list(sh.points)
        return out

    def _snap_edges(self, p: QPointF, exclude: int):
        """Nearest point on each shape's border to ``p``."""
        x, y = p.x(), p.y()
        out = []
        for i, sh in enumerate(self._sketch.shapes):
            if i == exclude or sh.kind == "region":
                continue
            if sh.kind == "circle":
                dx, dy = x - sh.cx, y - sh.cy
                d = math.hypot(dx, dy) or 1.0
                out.append((sh.cx + sh.radius * dx / d,
                            sh.cy + sh.radius * dy / d))
            elif sh.kind == "ellipse":
                ang = math.atan2((y - sh.cy), (x - sh.cx))
                out.append((sh.cx + sh.rx * math.cos(ang),
                            sh.cy + sh.ry * math.sin(ang)))
            elif sh.kind == "rect":
                hw, hh = sh.width / 2, sh.height / 2
                cxv = min(max(x, sh.cx - hw), sh.cx + hw)
                cyv = min(max(y, sh.cy - hh), sh.cy + hh)
                # project onto nearest edge
                out.append((sh.cx - hw if abs(x - (sh.cx - hw)) <
                            abs(x - (sh.cx + hw)) else sh.cx + hw, cyv))
                out.append((cxv, sh.cy - hh if abs(y - (sh.cy - hh)) <
                            abs(y - (sh.cy + hh)) else sh.cy + hh))
            elif sh.kind in ("line", "polygon") and len(sh.points) >= 2:
                pts = sh.points + ([sh.points[0]] if sh.kind == "polygon"
                                   and len(sh.points) >= 3 else [])
                for j in range(len(pts) - 1):
                    out.append(_nearest_on_seg(x, y, *pts[j], *pts[j + 1]))
        return out

    def _content_bounds(self):
        xs, ys = [], []
        for sh in self._sketch.shapes:
            for (x, y) in self._shape_extent(sh):
                xs.append(x)
                ys.append(y)
        if self._ref_well_d > 0:                # frame the reference well too
            r = self._ref_well_d / 2.0
            xs += [-r, r]
            ys += [-r, r]
        if not xs:
            return None
        return min(xs), min(ys), max(xs), max(ys)

    @staticmethod
    def _shape_extent(sh: SketchShape):
        if sh.kind == "circle":
            return [(sh.cx - sh.radius, sh.cy - sh.radius),
                    (sh.cx + sh.radius, sh.cy + sh.radius)]
        if sh.kind == "ellipse":
            return [(sh.cx - sh.rx, sh.cy - sh.ry),
                    (sh.cx + sh.rx, sh.cy + sh.ry)]
        if sh.kind == "rect":
            return [(sh.cx - sh.width / 2, sh.cy - sh.height / 2),
                    (sh.cx + sh.width / 2, sh.cy + sh.height / 2)]
        return list(sh.points)

    # ── Painting ──────────────────────────────────────────────────

    def showEvent(self, event):
        super().showEvent(event)
        if not self._origin_init:
            self.fit_view()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(COLORS.get("crust", "#11111b")))

        self._draw_grid(p)
        self._draw_axes(p)
        self._draw_reference_well(p)
        self._draw_toolpath(p)             # the raster (what prints)

        for i, sh in enumerate(self._sketch.shapes):
            self._draw_shape(p, sh, selected=(i == self._selected))

        self._draw_in_progress(p)

        if self._snap_marker is not None:
            c = self._w2s(self._snap_marker.x(), self._snap_marker.y())
            p.setPen(QPen(QColor(COLORS.get("yellow", "#f9e2af")), s(1.5)))
            p.setBrush(Qt.NoBrush)
            r = s(5)
            p.drawLine(int(c.x() - r), int(c.y()), int(c.x() + r), int(c.y()))
            p.drawLine(int(c.x()), int(c.y() - r), int(c.x()), int(c.y() + r))
            p.drawEllipse(c, r, r)
        p.end()

    def _draw_grid(self, p: QPainter):
        b_tl = self._s2w(0, 0)
        b_br = self._s2w(self.width(), self.height())
        g = self._grid_mm
        if self._scale * g < 6:        # too dense → coarsen
            g *= 5
        pen = QPen(QColor(COLORS.get("surface0", "#313244")), 1)
        p.setPen(pen)
        x0 = math.floor(b_tl.x() / g) * g
        x = x0
        while x < b_br.x():
            sx = self._w2s(x, 0).x()
            p.drawLine(int(sx), 0, int(sx), self.height())
            x += g
        y0 = math.floor(b_tl.y() / g) * g
        y = y0
        while y < b_br.y():
            sy = self._w2s(0, y).y()
            p.drawLine(0, int(sy), self.width(), int(sy))
            y += g

    def _draw_axes(self, p: QPainter):
        pen = QPen(QColor(COLORS.get("surface2", "#585b70")), 1, Qt.DashLine)
        p.setPen(pen)
        o = self._w2s(0, 0)
        p.drawLine(0, int(o.y()), self.width(), int(o.y()))
        p.drawLine(int(o.x()), 0, int(o.x()), self.height())

    def _draw_reference_well(self, p: QPainter):
        if self._ref_well_d <= 0:
            return
        r = self._ref_well_d / 2.0 * self._scale
        c = self._w2s(0, 0)
        pen = QPen(QColor(COLORS.get("mauve", "#cba6f7")), s(1.2), Qt.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(c, r, r)
        p.setPen(QColor(COLORS.get("mauve", "#cba6f7")))
        p.drawText(int(c.x() - r), int(c.y() - r) - s(4),
                   f"standard well Ø {self._ref_well_d:.1f} mm")

    def _draw_toolpath(self, p: QPainter):
        if not self._tp_runs:
            return
        for cat, wpts in self._tp_runs:
            if len(wpts) < 2:
                continue
            poly = QPolygonF([self._w2s(x, y) for (x, y) in wpts])
            if cat < 0:                    # travel move
                pen = QPen(QColor(150, 150, 150, 70), s(0.8), Qt.DashLine)
            else:                          # print move — colour by pump
                col = QColor(PUMP_HEX[cat % len(PUMP_HEX)])
                col.setAlpha(235)
                pen = QPen(col, s(1.6))
                pen.setCapStyle(Qt.RoundCap)
                pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(poly)

    def _draw_shape(self, p: QPainter, sh: SketchShape, selected: bool):
        """Editable overlay: thin shape boundary on top of the raster. The
        toolpath (drawn beneath) shows the actual bead width / fill, so the
        overlay is just a slim outline + handles for editing."""
        col = QColor(sh.color)
        bright = QColor(COLORS.get("text", "#cdd6f4"))

        # Region = baked fill: a dashed bounding outline so it's selectable.
        if sh.kind == "region":
            if not sh.points:
                return
            xs = [x for x, _ in sh.points]
            ys = [y for _, y in sh.points]
            edge = QColor(col)
            edge.setAlpha(220 if selected else 110)
            p.setPen(QPen(bright if selected else edge, s(1.2), Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(self._w2s(min(xs), min(ys)),
                              self._w2s(max(xs), max(ys))))
            return

        edge = QColor(bright if selected else col)
        if not selected:
            edge.setAlpha(210)
        pen = QPen(edge, s(2) if selected else s(1.4))
        if not selected:
            pen.setStyle(Qt.DashLine)      # overlay reads as a guide
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)

        if sh.kind == "circle":
            c = self._w2s(sh.cx, sh.cy)
            r = sh.radius * self._scale
            p.drawEllipse(c, r, r)
        elif sh.kind == "ellipse":
            c = self._w2s(sh.cx, sh.cy)
            p.drawEllipse(c, sh.rx * self._scale, sh.ry * self._scale)
        elif sh.kind == "rect":
            tl = self._w2s(sh.cx - sh.width / 2, sh.cy - sh.height / 2)
            p.drawRect(int(tl.x()), int(tl.y()),
                       int(sh.width * self._scale),
                       int(sh.height * self._scale))
        elif sh.kind == "line":
            if len(sh.points) >= 2:
                p.drawLine(self._w2s(*sh.points[0]), self._w2s(*sh.points[1]))
        elif sh.kind == "polygon":
            if len(sh.points) >= 2:
                poly = QPolygonF([self._w2s(x, y) for (x, y) in sh.points])
                if len(sh.points) >= 3:
                    p.drawPolygon(poly)
                else:
                    p.drawPolyline(poly)

        if selected and self._tool == Tool.SELECT:
            self._draw_handles(p, sh)

    def _draw_handles(self, p: QPainter, sh: SketchShape):
        p.setBrush(QBrush(QColor(COLORS.get("blue", "#89b4fa"))))
        p.setPen(QPen(QColor(COLORS.get("crust", "#11111b")), 1))
        hh = s(_HANDLE_PX)
        for (hx, hy) in self._handle_points(sh).values():
            c = self._w2s(hx, hy)
            p.drawRect(int(c.x() - hh), int(c.y() - hh), 2 * hh, 2 * hh)

    @staticmethod
    def _handle_points(sh: SketchShape) -> dict:
        if sh.kind == "circle":
            return {"r": (sh.cx + sh.radius, sh.cy)}
        if sh.kind == "ellipse":
            return {"rxry": (sh.cx + sh.rx, sh.cy + sh.ry)}
        if sh.kind == "rect":
            return {"wh": (sh.cx + sh.width / 2, sh.cy + sh.height / 2)}
        if sh.kind == "line" and len(sh.points) >= 2:
            return {"p0": sh.points[0], "p1": sh.points[1]}
        return {}

    def _draw_in_progress(self, p: QPainter):
        if self._mode == "poly" and self._poly_pts:
            pen = QPen(QColor(COLORS.get("yellow", "#f9e2af")), s(1.5),
                       Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            pts = [self._w2s(x, y) for (x, y) in self._poly_pts]
            if self._draw_cur is not None:
                pts.append(self._w2s(self._draw_cur.x(), self._draw_cur.y()))
            p.drawPolyline(QPolygonF(pts))
            for pt in [self._w2s(x, y) for (x, y) in self._poly_pts]:
                p.drawEllipse(pt, 3, 3)
            return

        if self._mode != "draw" or self._draw_start is None \
                or self._draw_cur is None:
            return
        pen = QPen(QColor(COLORS.get("yellow", "#f9e2af")), s(1.5), Qt.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        a, b = self._draw_start, self._draw_cur
        if self._tool == Tool.LINE:
            p.drawLine(self._w2s(a.x(), a.y()), self._w2s(b.x(), b.y()))
        elif self._tool == Tool.CIRCLE:
            r = math.hypot(b.x() - a.x(), b.y() - a.y())
            c = self._w2s(a.x(), a.y())
            p.drawEllipse(c, r * self._scale, r * self._scale)
        elif self._tool == Tool.RECT:
            tl = self._w2s(min(a.x(), b.x()), min(a.y(), b.y()))
            p.drawRect(int(tl.x()), int(tl.y()),
                       int(abs(b.x() - a.x()) * self._scale),
                       int(abs(b.y() - a.y()) * self._scale))
        elif self._tool == Tool.ELLIPSE:
            c = self._w2s((a.x() + b.x()) / 2, (a.y() + b.y()) / 2)
            p.drawEllipse(c, abs(b.x() - a.x()) / 2 * self._scale,
                          abs(b.y() - a.y()) / 2 * self._scale)

    # ── Mouse / keyboard ──────────────────────────────────────────

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 1.15 if delta > 0 else 1 / 1.15
        cursor = event.position()
        before = self._s2w(cursor.x(), cursor.y())
        self._scale = max(0.5, min(200.0, self._scale * factor))
        after = self._w2s(before.x(), before.y())
        self._origin += QPointF(cursor.x() - after.x(), cursor.y() - after.y())
        self.update()

    def mousePressEvent(self, event):
        pos = event.position()
        raw = self._s2w(pos.x(), pos.y())
        w = self._snap(raw)

        if event.button() == Qt.MiddleButton:
            self._mode = "pan"
            self._pan_start = pos
            self.setCursor(QCursor(Qt.ClosedHandCursor))
            return

        if event.button() == Qt.RightButton:
            if self._tool == Tool.POLYGON and self._mode == "poly":
                self._commit_polygon()
            return

        if event.button() != Qt.LeftButton:
            return

        if self._tool == Tool.SELECT:
            self._press_select(pos, raw)
        elif self._tool == Tool.FILL:
            self._do_fill(raw)
        elif self._tool == Tool.POLYGON:
            self._poly_pts.append((w.x(), w.y()))
            self._mode = "poly"
            self.update()
        else:
            self._mode = "draw"
            self._draw_start = w
            self._draw_cur = w
            self.update()

    def mouseMoveEvent(self, event):
        pos = event.position()
        raw = self._s2w(pos.x(), pos.y())

        if self._mode == "pan" and self._pan_start is not None:
            self._origin += pos - self._pan_start
            self._pan_start = pos
            self.update()
            return

        if self._mode == "draw":
            self._draw_cur = self._snap(raw)
            self.update()
            return

        if self._mode == "poly":
            self._draw_cur = self._snap(raw)
            self.update()
            return

        if self._mode == "move" and self._move_orig is not None:
            self._apply_move(self._snap(raw, exclude=self._selected))
            self.update()
            return

        if self._mode == "resize":
            self._apply_resize(self._snap(raw, exclude=self._selected))
            self.update()
            return

        # Idle hover with a drawing tool — preview the snap target.
        if self._tool in (Tool.LINE, Tool.RECT, Tool.CIRCLE, Tool.ELLIPSE,
                          Tool.POLYGON):
            self._snap(raw)
            self.update()

    def mouseReleaseEvent(self, event):
        if self._mode == "pan":
            self._mode = None
            self.setCursor(QCursor(Qt.ArrowCursor))
            return

        if self._mode == "draw":
            self._commit_draw()
            return

        if self._mode in ("move", "resize"):
            self._mode = None
            self._move_orig = None
            self._resize_kind = None
            self.sketch_changed.emit()
            return

    def mouseDoubleClickEvent(self, event):
        if self._tool == Tool.POLYGON and self._mode == "poly":
            self._commit_polygon()

    def keyPressEvent(self, event):
        k = event.key()
        if k in (Qt.Key_Delete, Qt.Key_Backspace):
            self.delete_selected()
        elif k == Qt.Key_Escape:
            self._poly_pts = []
            self._mode = None
            self.update()
        elif k in (Qt.Key_Return, Qt.Key_Enter):
            if self._tool == Tool.POLYGON and self._mode == "poly":
                self._commit_polygon()
        else:
            super().keyPressEvent(event)

    # ── Interaction helpers ───────────────────────────────────────

    def _press_select(self, screen_pos: QPointF, w: QPointF):
        # Resize handle first (for the already-selected shape).
        sh = self.selected_shape()
        if sh is not None:
            kind = self._handle_at(sh, screen_pos)
            if kind is not None:
                self._snapshot()
                self._mode = "resize"
                self._resize_kind = kind
                return
        # Otherwise hit-test shapes (topmost first).
        idx = self._shape_at(w)
        self.set_selected(idx)
        if idx >= 0:
            self._snapshot()
            self._mode = "move"
            self._move_anchor = w
            self._move_orig = copy.deepcopy(self._sketch.shapes[idx])

    def _handle_at(self, sh: SketchShape, screen_pos: QPointF):
        tol = s(_HIT_PX) + s(_HANDLE_PX)
        for name, (hx, hy) in self._handle_points(sh).items():
            c = self._w2s(hx, hy)
            if math.hypot(c.x() - screen_pos.x(), c.y() - screen_pos.y()) <= tol:
                return name
        return None

    def _shape_at(self, w: QPointF) -> int:
        tol = s(_HIT_PX) / self._scale
        for i in range(len(self._sketch.shapes) - 1, -1, -1):
            if self._hit(self._sketch.shapes[i], w, tol):
                return i
        return -1

    @staticmethod
    def _hit(sh: SketchShape, w: QPointF, tol: float) -> bool:
        x, y = w.x(), w.y()
        if sh.kind == "region":
            if not sh.points:
                return False
            xs = [px for px, _ in sh.points]
            ys = [py for _, py in sh.points]
            return (min(xs) - tol <= x <= max(xs) + tol
                    and min(ys) - tol <= y <= max(ys) + tol)
        if sh.kind == "circle":
            d = math.hypot(x - sh.cx, y - sh.cy)
            return (d <= sh.radius + tol) if sh.filled \
                else abs(d - sh.radius) <= tol
        if sh.kind == "ellipse":
            if sh.rx <= 0 or sh.ry <= 0:
                return False
            v = ((x - sh.cx) / sh.rx) ** 2 + ((y - sh.cy) / sh.ry) ** 2
            return v <= 1.0 + tol
        if sh.kind == "rect":
            return (abs(x - sh.cx) <= sh.width / 2 + tol
                    and abs(y - sh.cy) <= sh.height / 2 + tol)
        if sh.kind == "line" and len(sh.points) >= 2:
            return _dist_to_seg(x, y, *sh.points[0], *sh.points[1]) <= tol
        if sh.kind == "polygon" and len(sh.points) >= 2:
            if len(sh.points) >= 3 and _point_in_poly(x, y, sh.points):
                return True
            pts = sh.points + [sh.points[0]] if len(sh.points) >= 3 \
                else sh.points
            for j in range(len(pts) - 1):
                if _dist_to_seg(x, y, *pts[j], *pts[j + 1]) <= tol:
                    return True
        return False

    def _apply_move(self, w: QPointF):
        if self._move_orig is None or self._move_anchor is None:
            return
        dx = w.x() - self._move_anchor.x()
        dy = w.y() - self._move_anchor.y()
        sh = self._sketch.shapes[self._selected]
        orig = self._move_orig
        if orig.kind in ("circle", "ellipse", "rect"):
            sh.cx = orig.cx + dx
            sh.cy = orig.cy + dy
        else:
            sh.points = [(px + dx, py + dy) for (px, py) in orig.points]

    def _apply_resize(self, w: QPointF):
        sh = self._sketch.shapes[self._selected]
        k = self._resize_kind
        if k == "r":
            sh.radius = max(0.1, math.hypot(w.x() - sh.cx, w.y() - sh.cy))
        elif k == "rxry":
            sh.rx = max(0.1, abs(w.x() - sh.cx))
            sh.ry = max(0.1, abs(w.y() - sh.cy))
        elif k == "wh":
            sh.width = max(0.1, 2 * abs(w.x() - sh.cx))
            sh.height = max(0.1, 2 * abs(w.y() - sh.cy))
        elif k == "p0":
            sh.points[0] = (w.x(), w.y())
        elif k == "p1":
            sh.points[1] = (w.x(), w.y())

    def _commit_draw(self):
        a, b = self._draw_start, self._draw_cur
        self._mode = None
        self._draw_start = self._draw_cur = None
        if a is None or b is None:
            self.update()
            return
        color = PUMP_HEX[self._active_pump]
        pump = self._active_pump
        lw = self._default_line_width
        sh = None
        if self._tool == Tool.LINE:
            if math.hypot(b.x() - a.x(), b.y() - a.y()) > 0.2:
                sh = SketchShape(kind="line",
                                 points=[(a.x(), a.y()), (b.x(), b.y())],
                                 pump_index=pump, color=color, line_width_mm=lw)
        elif self._tool == Tool.CIRCLE:
            r = math.hypot(b.x() - a.x(), b.y() - a.y())
            if r > 0.2:
                sh = SketchShape(kind="circle", cx=a.x(), cy=a.y(),
                                 radius=r, pump_index=pump, color=color,
                                 line_width_mm=lw)
        elif self._tool == Tool.RECT:
            w_, h_ = abs(b.x() - a.x()), abs(b.y() - a.y())
            if w_ > 0.2 and h_ > 0.2:
                sh = SketchShape(kind="rect",
                                 cx=(a.x() + b.x()) / 2, cy=(a.y() + b.y()) / 2,
                                 width=w_, height=h_, pump_index=pump,
                                 color=color, line_width_mm=lw)
        elif self._tool == Tool.ELLIPSE:
            rx, ry = abs(b.x() - a.x()) / 2, abs(b.y() - a.y()) / 2
            if rx > 0.1 and ry > 0.1:
                sh = SketchShape(kind="ellipse",
                                 cx=(a.x() + b.x()) / 2, cy=(a.y() + b.y()) / 2,
                                 rx=rx, ry=ry, pump_index=pump, color=color,
                                 line_width_mm=lw)
        if sh is not None:
            self._snapshot()
            self._sketch.shapes.append(sh)
            self._selected = len(self._sketch.shapes) - 1
            self.selection_changed.emit(self._selected)
            self.sketch_changed.emit()
        self.update()

    def _do_fill(self, world: QPointF):
        pts = compute_fill_region(
            self._sketch.shapes, (world.x(), world.y()),
            max(self._sketch.line_spacing_mm, 0.1))
        if not pts:
            self.fill_result.emit(False)
            return
        self._snapshot()
        sh = SketchShape(
            kind="region",
            points=[(float(x), float(y)) for (x, y) in pts],
            pump_index=self._active_pump,
            color=PUMP_HEX[self._active_pump])
        self._sketch.shapes.append(sh)
        self._selected = len(self._sketch.shapes) - 1
        self.selection_changed.emit(self._selected)
        self.sketch_changed.emit()
        self.fill_result.emit(True)
        self.update()

    def _commit_polygon(self):
        pts = list(self._poly_pts)
        self._poly_pts = []
        self._mode = None
        self._draw_cur = None
        if len(pts) >= 2:
            self._snapshot()
            sh = SketchShape(kind="polygon", points=pts,
                             pump_index=self._active_pump,
                             color=PUMP_HEX[self._active_pump],
                             line_width_mm=self._default_line_width)
            self._sketch.shapes.append(sh)
            self._selected = len(self._sketch.shapes) - 1
            self.selection_changed.emit(self._selected)
            self.sketch_changed.emit()
        self.update()


# ── Geometry helpers ──────────────────────────────────────────────

def _dist_to_seg(px, py, x1, y1, x2, y2) -> float:
    nx, ny = _nearest_on_seg(px, py, x1, y1, x2, y2)
    return math.hypot(px - nx, py - ny)


def _nearest_on_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return (x1, y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return (x1 + t * dx, y1 + t * dy)


def _point_in_poly(x, y, pts) -> bool:
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and \
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside
