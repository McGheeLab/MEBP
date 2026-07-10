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
    Sketch, SketchShape, compute_fill_region, backtrace_shape,
)


class Tool(Enum):
    SELECT = auto()
    LINE = auto()
    RECT = auto()
    CIRCLE = auto()
    ELLIPSE = auto()
    POLYGON = auto()
    FILL = auto()      # paint-bucket: click inside an enclosed region
    TRAVEL = auto()    # place a retract-&-move point (pen-up break)


# Default per-pump colours for shapes.
PUMP_HEX = ["#89b4fa", "#a6e3a1", "#f9b387"]

# Colour for retract-&-move (travel) point markers — distinct from every pump
# colour, the grey travel line, mauve selection, red safe-boundary and yellow
# snap/warning cues.
TRAVEL_HEX = "#f5c2e7"     # pink

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
        self._selected = -1                # primary index (single-shape props)
        self._selection: set[int] = set()  # all selected shape indices

        # View transform: screen = origin + world * scale (px per mm).
        self._scale = 8.0
        self._origin = QPointF(0, 0)       # screen px of world (0,0)
        self._origin_init = False

        self._snap_mm = 0.0                # grid snap step; 0 = off
        self._grid_mm = 5.0
        self._osnap = True                 # snap to existing object borders
        self._snap_marker = None           # world QPointF of active snap hit
        self._active_ink_id = 1            # abstract ink for new shapes/fills
        self._default_line_width = 0.4     # bead width for new shapes (needle Ø)
        self._ref_well_d = 0.0             # reference standard-well Ø (mm); 0=off
        self._safe_well_d = 0.0            # needle-safe inner boundary Ø (mm); 0=off
        # Compiled toolpath shown as the main raster view. Each run is
        # (pump_index | -1 for travel, [(x,y) world mm, ...]).
        self._tp_runs: list[tuple[int, list[tuple[float, float]]]] = []
        self._show_thickness = False       # draw print runs at the bead width
        # World-mm width of the shaded thickness band (deposited bead). Set by
        # the page to needle inner Ø × extrusion multiplier; 0 = fall back to
        # the fill pitch (``line_spacing_mm``).
        self._bead_width_mm = 0.0

        # Interaction state
        # _mode: 'draw' | 'move' | 'resize' | 'pan' | 'poly'
        #        | 'marquee' (rubber-band select) | 'gmove' | 'gresize'
        self._mode = None
        self._draw_start = None            # world QPointF
        self._draw_cur = None
        self._poly_pts: list[tuple[float, float]] = []
        self._move_anchor = None           # world QPointF at grab
        self._move_orig = None             # snapshot of shape at grab
        self._resize_kind = None           # which handle
        self._pan_start = None
        # Marquee (lasso) + group-transform state
        self._marquee_start = None         # world QPointF
        self._marquee_cur = None           # world QPointF
        self._marquee_additive = False     # Ctrl/Shift → add to selection
        self._group_orig = None            # {idx: SketchShape} snapshot at grab
        self._group_anchor = None          # (x, y) scale anchor (world mm)
        self._group_ref = None             # (x, y) handle pos at grab (world mm)

        # Undo
        self._undo: list[dict] = []
        self._redo: list[dict] = []
        self._undo_cap = 50

    # ── Model access ──────────────────────────────────────────────

    def sketch(self) -> Sketch:
        return self._sketch

    def set_sketch(self, sketch: Sketch):
        self._sketch = sketch
        # Reset the active abstract ink to a valid one so new shapes reference
        # an ink that exists in this sketch.
        self._active_ink_id = sketch.inks[0].id if getattr(
            sketch, "inks", None) else 1
        self._selected = -1
        self._selection = set()
        self._undo.clear()
        self._redo.clear()
        self.selection_changed.emit(-1)
        self.update()

    def active_ink_id(self) -> int:
        return int(self._active_ink_id)

    def selected_index(self) -> int:
        return self._selected

    def selected_shape(self) -> SketchShape | None:
        if 0 <= self._selected < len(self._sketch.shapes):
            return self._sketch.shapes[self._selected]
        return None

    def set_selected(self, index: int):
        """Select exactly one shape (or none if out of range)."""
        n = len(self._sketch.shapes)
        self._set_selection({index} if 0 <= index < n else set())

    def select_indices(self, indices):
        """Select an explicit set of shapes (used by the sequence panel to
        highlight a section's shapes on click)."""
        self._set_selection({int(i) for i in indices})

    def selected_indices(self) -> list[int]:
        return sorted(self._selection)

    def selection_count(self) -> int:
        return len(self._selection)

    def select_all(self):
        self._set_selection(set(range(len(self._sketch.shapes))))

    def clear_selection(self):
        self._set_selection(set())

    def _set_selection(self, indices: set[int]):
        """Set the full selection set; the primary index (for the single-shape
        properties panel) is the sole member when exactly one is selected."""
        n = len(self._sketch.shapes)
        self._selection = {i for i in indices if 0 <= i < n}
        self._selected = (next(iter(self._selection))
                          if len(self._selection) == 1 else -1)
        self.selection_changed.emit(self._selected)
        self.update()

    # ── Tool / view controls ──────────────────────────────────────

    def set_tool(self, tool: Tool):
        self._tool = tool
        self._poly_pts = []
        self._mode = None
        if tool != Tool.SELECT:
            self._set_selection(set())
        self.tool_changed.emit(tool)
        self.update()

    def get_tool(self) -> Tool:
        return self._tool

    def set_snap(self, mm: float):
        self._snap_mm = max(0.0, mm)

    def set_object_snap(self, enabled: bool):
        self._osnap = bool(enabled)

    def set_active_ink(self, ink_id: int):
        """Abstract ink id stamped on newly drawn shapes/fills."""
        self._active_ink_id = int(ink_id)

    def _active_ink_color(self) -> str:
        """Colour of the active abstract ink (palette fallback if unknown)."""
        ink = self._sketch.ink_by_id(self._active_ink_id)
        if ink is not None:
            return ink.color
        return PUMP_HEX[(self._active_ink_id - 1) % len(PUMP_HEX)]

    def set_default_line_width(self, mm: float):
        """Bead width applied to newly drawn shapes (typically needle Ø)."""
        if mm and mm > 0:
            self._default_line_width = float(mm)

    def set_reference_well(self, diameter_mm: float):
        """Draw a dashed standard-well outline (Ø mm) at the origin; 0 = off."""
        self._ref_well_d = max(0.0, float(diameter_mm or 0.0))
        self.update()

    def set_safe_boundary(self, diameter_mm: float):
        """Draw a dashed inner 'needle-safe' boundary (Ø mm) at the origin —
        the well edge inset by the needle radius, so the needle wall never
        contacts the well wall when printing. 0 = off."""
        self._safe_well_d = max(0.0, float(diameter_mm or 0.0))
        self.update()

    def set_show_thickness(self, on: bool):
        """When on, draw a shaded band around each print run at the deposited
        bead width (see :meth:`set_bead_width_mm`), with a crisp centerline on
        top, so the user sees how thick the printed lines will be."""
        self._show_thickness = bool(on)
        self.update()

    def set_bead_width_mm(self, mm: float):
        """World-mm width of the shaded print-thickness band — the deposited
        bead. Typically needle inner Ø × extrusion multiplier. 0 falls back to
        the fill pitch (``line_spacing_mm``)."""
        self._bead_width_mm = max(0.0, float(mm or 0.0))
        if self._show_thickness:
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
            # Clamp to the same ceiling as wheel-zoom so a tiny-span sketch
            # (e.g. only tightly-clustered travel points) can't zoom to absurdity.
            self._scale = min(200.0, max(1.0, min((w * margin) / span_x,
                                                   (h * margin) / span_y)))
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
        keep = {i for i in self._selection
                if 0 <= i < len(self._sketch.shapes)}
        self._selection = keep
        self._selected = next(iter(keep)) if len(keep) == 1 else -1
        self.selection_changed.emit(self._selected)
        self.sketch_changed.emit()
        self.update()

    def delete_selected(self):
        """Delete every selected shape (supports multi-selection)."""
        if not self._selection:
            return
        self._snapshot()
        for i in sorted(self._selection, reverse=True):
            if 0 <= i < len(self._sketch.shapes):
                self._sketch.shapes.pop(i)
        self._set_selection(set())
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
            if sh.kind == "travel":
                out.append((sh.cx, sh.cy))
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
        if sh.kind == "travel":
            return [(sh.cx, sh.cy)]
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

    # ── Group selection geometry / transforms ─────────────────────

    def _group_bbox(self):
        """Bounding box (minx, miny, maxx, maxy) of all selected shapes' extents,
        or None when nothing is selected."""
        xs, ys = [], []
        for i in self._selection:
            if 0 <= i < len(self._sketch.shapes):
                for (x, y) in self._shape_extent(self._sketch.shapes[i]):
                    xs.append(x)
                    ys.append(y)
        if not xs:
            return None
        return min(xs), min(ys), max(xs), max(ys)

    @staticmethod
    def _shape_bbox(sh: SketchShape):
        xs = [x for x, _ in SketchCanvas._shape_extent(sh)]
        ys = [y for _, y in SketchCanvas._shape_extent(sh)]
        if not xs:
            return None
        return min(xs), min(ys), max(xs), max(ys)

    @staticmethod
    def _transform_shape(dst: SketchShape, src: SketchShape,
                         translate, anchor, sc):
        """Rebuild ``dst`` from snapshot ``src`` under an optional translation
        ``(dx, dy)`` then a uniform scale ``sc`` about ``anchor`` (world mm).
        Used for group move (sc=None) and group resize (translate=None)."""
        def xf(x, y):
            if translate is not None:
                x += translate[0]
                y += translate[1]
            if sc is not None and anchor is not None:
                x = anchor[0] + (x - anchor[0]) * sc
                y = anchor[1] + (y - anchor[1]) * sc
            return x, y

        k = src.kind
        if k in ("circle", "ellipse", "rect", "travel"):
            dst.cx, dst.cy = xf(src.cx, src.cy)
            if sc is not None:                  # travel points carry no size
                if k == "circle":
                    dst.radius = max(0.05, src.radius * sc)
                elif k == "ellipse":
                    dst.rx = max(0.05, src.rx * sc)
                    dst.ry = max(0.05, src.ry * sc)
                elif k == "rect":
                    dst.width = max(0.05, src.width * sc)
                    dst.height = max(0.05, src.height * sc)
                # k == "travel": position-only, carries no size — nothing to scale
        else:                                   # line / polygon / region
            dst.points = [xf(px, py) for (px, py) in src.points]

    def scale_selection(self, factor: float):
        """Uniformly scale every selected shape about the selection's centre
        by ``factor`` (used by the properties-panel 'Apply scale' button)."""
        if not self._selection or factor <= 0:
            return
        bbox = self._group_bbox()
        if bbox is None:
            return
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        self._snapshot()
        for i in list(self._selection):
            if 0 <= i < len(self._sketch.shapes):
                sh = self._sketch.shapes[i]
                self._transform_shape(sh, copy.deepcopy(sh), None,
                                      (cx, cy), float(factor))
        self.sketch_changed.emit()
        self.update()

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
        self._draw_safe_boundary(p)
        self._draw_toolpath(p)             # the raster (what prints)

        for i, sh in enumerate(self._sketch.shapes):
            self._draw_shape(p, sh, selected=(i in self._selection))

        self._draw_selection_overlay(p)
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
                   f"well wall Ø {self._ref_well_d:.1f} mm")

    def _draw_safe_boundary(self, p: QPainter):
        """Inner 'needle-safe' ring: the well wall inset by the needle radius.
        Keeping the toolpath inside this ring guarantees the needle outer wall
        never touches the well wall."""
        if self._safe_well_d <= 0:
            return
        r = self._safe_well_d / 2.0 * self._scale
        c = self._w2s(0, 0)
        col = QColor(COLORS.get("red", "#f38ba8"))
        pen = QPen(col, s(1.2), Qt.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(c, r, r)
        p.setPen(col)
        p.drawText(int(c.x() - r), int(c.y() + r) + s(12),
                   f"needle-safe Ø {self._safe_well_d:.1f} mm")

    def _draw_toolpath(self, p: QPainter):
        if not self._tp_runs:
            return
        # Width of the shaded bead band when "show thickness" is on — the
        # deposited bead (needle inner Ø × extrusion multiplier, pushed via
        # ``set_bead_width_mm``); falls back to the fill pitch when unset.
        bead_mm = (self._bead_width_mm if self._bead_width_mm > 0
                   else self._sketch.line_spacing_mm)
        bead_px = max(1.0, bead_mm * self._scale)
        p.setBrush(Qt.NoBrush)
        for cat, wpts in self._tp_runs:
            if len(wpts) < 2:
                continue
            poly = QPolygonF([self._w2s(x, y) for (x, y) in wpts])
            if cat < 0:                    # travel / fast move (never deposited)
                col = QColor(COLORS.get("overlay1", "#7f849c"))
                col.setAlpha(200)
                pen = QPen(col, s(1.0), Qt.DashLine)
                p.setPen(pen)
                p.drawPolyline(poly)
                continue
            # Print move — colour by pump.
            base = QColor(PUMP_HEX[cat % len(PUMP_HEX)])
            if self._show_thickness:
                # Shaded deposited-bead footprint …
                band = QColor(base)
                band.setAlpha(80)
                band_pen = QPen(band, bead_px)
                band_pen.setCapStyle(Qt.RoundCap)
                band_pen.setJoinStyle(Qt.RoundJoin)
                p.setPen(band_pen)
                p.drawPolyline(poly)
                # … with a crisp centerline on top so the path stays legible.
                core = QColor(base)
                core.setAlpha(235)
                core_pen = QPen(core, s(1.3))
                core_pen.setCapStyle(Qt.RoundCap)
                core_pen.setJoinStyle(Qt.RoundJoin)
                p.setPen(core_pen)
                p.drawPolyline(poly)
            else:
                base.setAlpha(235)
                pen = QPen(base, s(1.6))
                pen.setCapStyle(Qt.RoundCap)
                pen.setJoinStyle(Qt.RoundJoin)
                p.setPen(pen)
                p.drawPolyline(poly)

    def _draw_shape(self, p: QPainter, sh: SketchShape, selected: bool):
        """Editable overlay: thin shape boundary on top of the raster. The
        toolpath (drawn beneath) shows the actual bead width / fill, so the
        overlay is just a slim outline + handles for editing."""
        col = QColor(sh.color)
        bright = QColor(COLORS.get("text", "#cdd6f4"))

        # Retract-&-move point: a colour-coded diamond with an up-chevron
        # (needle lifts here). No printed geometry.
        if sh.kind == "travel":
            c = self._w2s(sh.cx, sh.cy)
            tc = QColor(TRAVEL_HEX)
            r = s(6)
            diamond = QPolygonF([
                QPointF(c.x(), c.y() - r), QPointF(c.x() + r, c.y()),
                QPointF(c.x(), c.y() + r), QPointF(c.x() - r, c.y())])
            fill = QColor(tc)
            fill.setAlpha(170 if selected else 90)
            p.setBrush(QBrush(fill))
            p.setPen(QPen(bright if selected else tc,
                          s(2) if selected else s(1.5)))
            p.drawPolygon(diamond)
            # Up-chevron = retract cue.
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(bright if selected else tc, s(1.5)))
            p.drawLine(QPointF(c.x() - r * 0.45, c.y() + r * 0.15),
                       QPointF(c.x(), c.y() - r * 0.45))
            p.drawLine(QPointF(c.x() + r * 0.45, c.y() + r * 0.15),
                       QPointF(c.x(), c.y() - r * 0.45))
            return

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

        # Per-shape resize handles only for a lone selection; a multi-selection
        # uses the group bounding-box handle instead.
        if (selected and self._tool == Tool.SELECT
                and len(self._selection) == 1):
            self._draw_handles(p, sh)
            # Draggable print-start marker (outline shapes only) — lets the user
            # set WHERE the shape begins printing, snapping to existing lines.
            if self._shape_supports_start(sh):
                self._draw_start_marker(p, sh)

    def _draw_handles(self, p: QPainter, sh: SketchShape):
        p.setBrush(QBrush(QColor(COLORS.get("blue", "#89b4fa"))))
        p.setPen(QPen(QColor(COLORS.get("crust", "#11111b")), 1))
        hh = s(_HANDLE_PX)
        for (hx, hy) in self._handle_points(sh).values():
            c = self._w2s(hx, hy)
            p.drawRect(int(c.x() - hh), int(c.y() - hh), 2 * hh, 2 * hh)

    def _draw_start_marker(self, p: QPainter, sh: SketchShape):
        """Green print-start marker for the selected outline shape: a dot on the
        shape's effective start + a leader to a small flag offset outward (so it
        never collides with the blue resize handles). Drag it to set the start
        point (see ``_press_select`` / ``_apply_resize``)."""
        green = QColor(COLORS.get("green", "#a6e3a1"))
        sw_world = self._effective_start_world(sh)
        sw = self._w2s(sw_world.x(), sw_world.y())
        m = self._start_marker_screen(sh)
        p.setPen(QPen(green, s(1.2)))
        p.setBrush(Qt.NoBrush)
        p.drawLine(sw, m)
        p.setBrush(QBrush(green))
        p.setPen(QPen(green, s(1)))
        p.drawEllipse(sw, s(3), s(3))
        r = s(6)
        tri = QPolygonF([QPointF(m.x() - r, m.y() - r),
                         QPointF(m.x() + r, m.y() - r),
                         QPointF(m.x(), m.y() + r)])
        fill = QColor(green)
        fill.setAlpha(210)
        p.setBrush(QBrush(fill))
        p.setPen(QPen(QColor(COLORS.get("crust", "#11111b")), s(1)))
        p.drawPolygon(tri)

    def _draw_selection_overlay(self, p: QPainter):
        """Group bounding box + resize handle (2+ selected) and the marquee
        rubber-band while a lasso drag is in progress."""
        blue = QColor(COLORS.get("blue", "#89b4fa"))
        if len(self._selection) >= 2:
            bbox = self._group_bbox()
            if bbox is not None:
                tl = self._w2s(bbox[0], bbox[1])
                br = self._w2s(bbox[2], bbox[3])
                p.setPen(QPen(blue, s(1.4), Qt.DashLine))
                p.setBrush(Qt.NoBrush)
                p.drawRect(QRectF(tl, br))
                # Bottom-right corner = the group resize handle.
                hh = s(_HANDLE_PX)
                p.setBrush(QBrush(blue))
                p.setPen(QPen(QColor(COLORS.get("crust", "#11111b")), 1))
                p.drawRect(int(br.x() - hh), int(br.y() - hh), 2 * hh, 2 * hh)

        if (self._mode == "marquee" and self._marquee_start is not None
                and self._marquee_cur is not None):
            a = self._w2s(self._marquee_start.x(), self._marquee_start.y())
            b = self._w2s(self._marquee_cur.x(), self._marquee_cur.y())
            fill = QColor(blue)
            fill.setAlpha(40)
            p.setPen(QPen(blue, s(1.2), Qt.DashLine))
            p.setBrush(QBrush(fill))
            p.drawRect(QRectF(a, b))

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
            self._press_select(pos, raw, event.modifiers())
        elif self._tool == Tool.FILL:
            self._do_fill(raw)
        elif self._tool == Tool.TRAVEL:
            self._place_travel(w)
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

        if self._mode == "marquee":
            self._marquee_cur = raw
            self.update()
            return

        if self._mode == "gmove" and self._group_orig is not None:
            # No object-snap here: it would latch onto the very shapes being
            # dragged (``_snap`` can exclude only one index, not the group).
            self._apply_group_move(raw)
            self.update()
            return

        if self._mode == "gresize" and self._group_orig is not None:
            self._apply_group_resize(raw)
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
                          Tool.POLYGON, Tool.TRAVEL):
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

        if self._mode == "marquee":
            self._commit_marquee()
            return

        if self._mode in ("move", "resize"):
            # A start-point drag changes the shape's custom/default state, so
            # nudge the properties panel to rebuild (via selection_changed).
            was_start = (self._mode == "resize" and self._resize_kind == "start")
            self._mode = None
            self._move_orig = None
            self._resize_kind = None
            self.sketch_changed.emit()
            if was_start:
                self.selection_changed.emit(self._selected)
            return

        if self._mode in ("gmove", "gresize"):
            self._mode = None
            self._group_orig = None
            self._group_anchor = None
            self._group_ref = None
            self.sketch_changed.emit()
            return

    def mouseDoubleClickEvent(self, event):
        if self._tool == Tool.POLYGON and self._mode == "poly":
            self._commit_polygon()

    def keyPressEvent(self, event):
        k = event.key()
        if k == Qt.Key_A and (event.modifiers() & Qt.ControlModifier):
            self.select_all()
        elif k in (Qt.Key_Delete, Qt.Key_Backspace):
            self.delete_selected()
        elif k == Qt.Key_Escape:
            self._poly_pts = []
            self._mode = None
            self.clear_selection()
            self.update()
        elif k in (Qt.Key_Return, Qt.Key_Enter):
            if self._tool == Tool.POLYGON and self._mode == "poly":
                self._commit_polygon()
        else:
            super().keyPressEvent(event)

    # ── Interaction helpers ───────────────────────────────────────

    def _press_select(self, screen_pos: QPointF, w: QPointF, modifiers=None):
        additive = bool(modifiers is not None and (
            modifiers & (Qt.ControlModifier | Qt.ShiftModifier)))

        # 1) Group resize handle (bottom-right of the multi-selection bbox).
        if len(self._selection) >= 2 and self._group_handle_at(screen_pos):
            self._begin_group_resize()
            return

        # 2) Per-shape resize handle for a lone selection. The print-start flag
        # is checked FIRST (it sits offset outside the shape) so it stays
        # grabbable even where it would otherwise overlap an endpoint handle.
        if len(self._selection) == 1:
            sh = self.selected_shape()
            if sh is not None:
                if (self._shape_supports_start(sh)
                        and self._start_handle_at(sh, screen_pos)):
                    self._snapshot()
                    self._mode = "resize"
                    self._resize_kind = "start"
                    return
                kind = self._handle_at(sh, screen_pos)
                if kind is not None:
                    self._snapshot()
                    self._mode = "resize"
                    self._resize_kind = kind
                    return

        # 3) Hit-test shapes (topmost first).
        idx = self._shape_at(w)
        if idx >= 0:
            if additive:                       # Ctrl/Shift → toggle membership
                sel = set(self._selection)
                sel.discard(idx) if idx in sel else sel.add(idx)
                self._set_selection(sel)
                return
            if idx in self._selection and len(self._selection) >= 2:
                self._begin_group_move(w)      # drag within a group → move all
                return
            self.set_selected(idx)             # select one + start moving it
            self._snapshot()
            self._mode = "move"
            self._move_anchor = w
            self._move_orig = copy.deepcopy(self._sketch.shapes[idx])
            return

        # 4) Empty space → marquee (lasso) rubber-band select.
        self._mode = "marquee"
        self._marquee_start = w
        self._marquee_cur = w
        self._marquee_additive = additive
        self.update()

    # ── Group / marquee interaction ───────────────────────────────

    def _group_handle_at(self, screen_pos: QPointF) -> bool:
        bbox = self._group_bbox()
        if bbox is None:
            return False
        c = self._w2s(bbox[2], bbox[3])        # bottom-right corner
        tol = s(_HIT_PX) + s(_HANDLE_PX)
        return math.hypot(c.x() - screen_pos.x(),
                          c.y() - screen_pos.y()) <= tol

    def _group_snapshot(self) -> dict:
        return {i: copy.deepcopy(self._sketch.shapes[i])
                for i in self._selection if 0 <= i < len(self._sketch.shapes)}

    def _begin_group_move(self, w: QPointF):
        self._snapshot()
        self._mode = "gmove"
        self._move_anchor = w
        self._group_orig = self._group_snapshot()

    def _begin_group_resize(self):
        bbox = self._group_bbox()
        if bbox is None:
            return
        self._snapshot()
        self._mode = "gresize"
        self._group_anchor = (bbox[0], bbox[1])    # top-left = scale anchor
        self._group_ref = (bbox[2], bbox[3])       # bottom-right = grabbed handle
        self._group_orig = self._group_snapshot()

    def _apply_group_move(self, w: QPointF):
        if self._move_anchor is None:
            return
        dx = w.x() - self._move_anchor.x()
        dy = w.y() - self._move_anchor.y()
        for idx, orig in self._group_orig.items():
            if 0 <= idx < len(self._sketch.shapes):
                self._transform_shape(self._sketch.shapes[idx], orig,
                                      (dx, dy), None, None)

    def _apply_group_resize(self, w: QPointF):
        ax, ay = self._group_anchor
        rx, ry = self._group_ref
        d0 = math.hypot(rx - ax, ry - ay)
        if d0 < 1e-6:
            return
        sc = max(0.05, min(50.0, math.hypot(w.x() - ax, w.y() - ay) / d0))
        for idx, orig in self._group_orig.items():
            if 0 <= idx < len(self._sketch.shapes):
                self._transform_shape(self._sketch.shapes[idx], orig,
                                      None, (ax, ay), sc)

    def _commit_marquee(self):
        a, b = self._marquee_start, self._marquee_cur
        additive = self._marquee_additive
        self._mode = None
        self._marquee_start = self._marquee_cur = None
        self._marquee_additive = False
        if a is None or b is None:
            self.update()
            return
        # A negligible drag = a click on empty space → clear (unless additive).
        if abs(b.x() - a.x()) < 0.2 and abs(b.y() - a.y()) < 0.2:
            if not additive:
                self._set_selection(set())
            else:
                self.update()
            return
        rect = (min(a.x(), b.x()), min(a.y(), b.y()),
                max(a.x(), b.x()), max(a.y(), b.y()))
        hits = {i for i, sh in enumerate(self._sketch.shapes)
                if self._shape_in_rect(sh, rect)}
        self._set_selection((set(self._selection) | hits) if additive else hits)

    def _shape_in_rect(self, sh: SketchShape, rect) -> bool:
        """True when a shape's bounding box intersects the marquee rectangle."""
        bb = self._shape_bbox(sh)
        if bb is None:
            return False
        sminx, sminy, smaxx, smaxy = bb
        rminx, rminy, rmaxx, rmaxy = rect
        return not (smaxx < rminx or sminx > rmaxx
                    or smaxy < rminy or sminy > rmaxy)

    def _handle_at(self, sh: SketchShape, screen_pos: QPointF):
        tol = s(_HIT_PX) + s(_HANDLE_PX)
        for name, (hx, hy) in self._handle_points(sh).items():
            c = self._w2s(hx, hy)
            if math.hypot(c.x() - screen_pos.x(), c.y() - screen_pos.y()) <= tol:
                return name
        return None

    # ── Print-start point (continuity control) ────────────────────

    @staticmethod
    def _shape_supports_start(sh: SketchShape) -> bool:
        """Only unfilled outlines have a controllable print-start point; fills /
        regions (rasters) and travel markers do not."""
        return (sh.kind in ("line", "circle", "ellipse", "rect", "polygon")
                and not sh.filled)

    @staticmethod
    def _default_start_world(sh: SketchShape):
        """Geometric default start (matches the compiler's untouched path)."""
        k = sh.kind
        if k == "circle":
            return (sh.cx + sh.radius, sh.cy)
        if k == "ellipse":
            return (sh.cx + sh.rx, sh.cy)
        if k == "rect":
            return (sh.cx - sh.width / 2, sh.cy - sh.height / 2)
        if k in ("line", "polygon") and sh.points:
            return (sh.points[0][0], sh.points[0][1])
        return (sh.cx, sh.cy)

    @staticmethod
    def _resolve_start_on_shape(sh: SketchShape, sp):
        """Nearest point ON the shape's outline to ``sp`` — mirrors where the
        compiler will actually begin the path (closest ring vertex / endpoint)."""
        x, y = float(sp[0]), float(sp[1])
        k = sh.kind
        if k == "circle":
            dx, dy = x - sh.cx, y - sh.cy
            d = math.hypot(dx, dy) or 1.0
            return (sh.cx + sh.radius * dx / d, sh.cy + sh.radius * dy / d)
        if k == "ellipse":
            ang = math.atan2(y - sh.cy, x - sh.cx)
            return (sh.cx + sh.rx * math.cos(ang), sh.cy + sh.ry * math.sin(ang))
        if k == "rect":
            hw, hh = sh.width / 2, sh.height / 2
            corners = [(sh.cx - hw, sh.cy - hh), (sh.cx + hw, sh.cy - hh),
                       (sh.cx + hw, sh.cy + hh), (sh.cx - hw, sh.cy + hh),
                       (sh.cx - hw, sh.cy - hh)]
            best, bd = None, None
            for j in range(4):
                nx, ny = _nearest_on_seg(x, y, *corners[j], *corners[j + 1])
                dd = math.hypot(nx - x, ny - y)
                if bd is None or dd < bd:
                    bd, best = dd, (nx, ny)
            return best
        if k == "line" and len(sh.points) >= 2:
            p0, p1 = sh.points[0], sh.points[1]
            return p0 if (math.hypot(p0[0] - x, p0[1] - y)
                          <= math.hypot(p1[0] - x, p1[1] - y)) else p1
        if k == "polygon" and sh.points:
            if len(sh.points) >= 3:
                pts = list(sh.points) + [sh.points[0]]
                best, bd = None, None
                for j in range(len(pts) - 1):
                    nx, ny = _nearest_on_seg(x, y, *pts[j], *pts[j + 1])
                    dd = math.hypot(nx - x, ny - y)
                    if bd is None or dd < bd:
                        bd, best = dd, (nx, ny)
                return best
            p0, p1 = sh.points[0], sh.points[-1]
            return p0 if (math.hypot(p0[0] - x, p0[1] - y)
                          <= math.hypot(p1[0] - x, p1[1] - y)) else p1
        return (sh.cx, sh.cy)

    def _effective_start_world(self, sh: SketchShape) -> QPointF:
        """Where the shape actually begins printing: the resolved start_point if
        set, else the geometric default."""
        sp = getattr(sh, "start_point", None)
        if sp is None:
            return QPointF(*self._default_start_world(sh))
        return QPointF(*self._resolve_start_on_shape(sh, sp))

    def _start_marker_screen(self, sh: SketchShape) -> QPointF:
        """Screen position of the draggable start flag — the effective start
        pushed a fixed distance outward from the shape's centre so it never
        overlaps a resize handle."""
        sw = self._effective_start_world(sh)
        bb = self._shape_bbox(sh)
        base = self._w2s(sw.x(), sw.y())
        if bb is None:
            return base
        cx, cy = (bb[0] + bb[2]) / 2.0, (bb[1] + bb[3]) / 2.0
        dx, dy = sw.x() - cx, sw.y() - cy
        L = math.hypot(dx, dy)
        if L < 1e-9:
            dx, dy, L = 1.0, -1.0, math.sqrt(2.0)
        off = s(_HANDLE_PX) * 2.2
        return QPointF(base.x() + dx / L * off, base.y() + dy / L * off)

    def _start_handle_at(self, sh: SketchShape, screen_pos: QPointF) -> bool:
        c = self._start_marker_screen(sh)
        tol = s(_HIT_PX) + s(_HANDLE_PX)
        return math.hypot(c.x() - screen_pos.x(),
                          c.y() - screen_pos.y()) <= tol

    def clear_start_point(self, index: int):
        """Reset a shape's print start to its geometric default."""
        if not (0 <= index < len(self._sketch.shapes)):
            return
        sh = self._sketch.shapes[index]
        if getattr(sh, "start_point", None) is None:
            return
        self._snapshot()
        sh.start_point = None
        self.sketch_changed.emit()
        self.update()

    def optimize(self, needle=None):
        """Reorder shapes + set start points to minimize print discontinuities
        (user-placed retract points stay as fixed breaks). In single-needle mode
        (resolved from the sketch/``needle``) it also groups same-channel shapes
        to cut ink swaps. Undoable."""
        from SupportClasses.SketchTrajectory import optimize_print_order
        if len(self._sketch.shapes) < 2:
            return
        self._snapshot()
        self._sketch = optimize_print_order(self._sketch, needle=needle)
        self._set_selection(set())
        self.sketch_changed.emit()
        self.update()

    def _shape_at(self, w: QPointF) -> int:
        tol = s(_HIT_PX) / self._scale
        for i in range(len(self._sketch.shapes) - 1, -1, -1):
            if self._hit(self._sketch.shapes[i], w, tol):
                return i
        return -1

    @staticmethod
    def _hit(sh: SketchShape, w: QPointF, tol: float) -> bool:
        x, y = w.x(), w.y()
        if sh.kind == "travel":
            return math.hypot(x - sh.cx, y - sh.cy) <= tol
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
        if orig.kind in ("circle", "ellipse", "rect", "travel"):
            sh.cx = orig.cx + dx
            sh.cy = orig.cy + dy
        else:
            sh.points = [(px + dx, py + dy) for (px, py) in orig.points]

    def _apply_resize(self, w: QPointF):
        sh = self._sketch.shapes[self._selected]
        k = self._resize_kind
        if k == "start":
            # ``w`` is already snapped to existing lines/vertices (see
            # mouseMoveEvent). Store the raw anchor; the compiler + the marker
            # resolve it onto the shape's own outline.
            sh.start_point = (w.x(), w.y())
            return
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
        ink_id = self._active_ink_id
        color = self._active_ink_color()
        lw = self._default_line_width
        sh = None
        if self._tool == Tool.LINE:
            if math.hypot(b.x() - a.x(), b.y() - a.y()) > 0.2:
                sh = SketchShape(kind="line",
                                 points=[(a.x(), a.y()), (b.x(), b.y())],
                                 ink_id=ink_id, color=color, line_width_mm=lw)
        elif self._tool == Tool.CIRCLE:
            r = math.hypot(b.x() - a.x(), b.y() - a.y())
            if r > 0.2:
                sh = SketchShape(kind="circle", cx=a.x(), cy=a.y(),
                                 radius=r, ink_id=ink_id, color=color,
                                 line_width_mm=lw)
        elif self._tool == Tool.RECT:
            w_, h_ = abs(b.x() - a.x()), abs(b.y() - a.y())
            if w_ > 0.2 and h_ > 0.2:
                sh = SketchShape(kind="rect",
                                 cx=(a.x() + b.x()) / 2, cy=(a.y() + b.y()) / 2,
                                 width=w_, height=h_, ink_id=ink_id,
                                 color=color, line_width_mm=lw)
        elif self._tool == Tool.ELLIPSE:
            rx, ry = abs(b.x() - a.x()) / 2, abs(b.y() - a.y()) / 2
            if rx > 0.1 and ry > 0.1:
                sh = SketchShape(kind="ellipse",
                                 cx=(a.x() + b.x()) / 2, cy=(a.y() + b.y()) / 2,
                                 rx=rx, ry=ry, ink_id=ink_id, color=color,
                                 line_width_mm=lw)
        if sh is not None:
            self._snapshot()
            self._sketch.shapes.append(sh)
            # Select the new shape via the canonical setter so BOTH _selection
            # (highlight + handles) and _selected (props panel) stay in sync.
            self._set_selection({len(self._sketch.shapes) - 1})
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
            ink_id=self._active_ink_id,
            color=self._active_ink_color())
        self._sketch.shapes.append(sh)
        self._set_selection({len(self._sketch.shapes) - 1})
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
                             ink_id=self._active_ink_id,
                             color=self._active_ink_color(),
                             line_width_mm=self._default_line_width)
            self._sketch.shapes.append(sh)
            self._set_selection({len(self._sketch.shapes) - 1})
            self.sketch_changed.emit()
        self.update()

    # ── Retract-&-move points + back-trace ─────────────────────────

    def _place_travel(self, w: QPointF):
        """Append a retract-&-move point at ``w`` (world mm): the needle lifts
        and travels here (pen-up), splitting the print into separate runs."""
        self._snapshot()
        sh = SketchShape(kind="travel", cx=w.x(), cy=w.y(), color=TRAVEL_HEX)
        self._sketch.shapes.append(sh)
        self._set_selection({len(self._sketch.shapes) - 1})
        self.sketch_changed.emit()
        self.update()

    def _run_indices(self, seed: int) -> list[int]:
        """Indices of the maximal contiguous run of PRINTING shapes containing
        ``seed`` — i.e. the 'continuous collection of lines between retract
        points'. A ``travel`` shape bounds the run. Empty if ``seed`` is a
        travel point / out of range."""
        shapes = self._sketch.shapes
        if not (0 <= seed < len(shapes)) or shapes[seed].kind == "travel":
            return []
        lo = seed
        while lo - 1 >= 0 and shapes[lo - 1].kind != "travel":
            lo -= 1
        hi = seed
        while hi + 1 < len(shapes) and shapes[hi + 1].kind != "travel":
            hi += 1
        return list(range(lo, hi + 1))

    def backtrace_run(self, seed_index: int, z_offset: float = 0.0,
                      xy_offset: float = 0.0, print_on_return: bool = True
                      ) -> int:
        """Back-trace the continuous run containing ``seed_index``: append a
        reversed copy of every shape in the run (in reverse order), offset by
        ``z_offset`` mm in height and ``xy_offset`` mm in-plane, right after the
        run so the needle retraces it. ``print_on_return=False`` = move-only.
        Returns the number of shapes added."""
        run = self._run_indices(seed_index)
        if not run:
            return 0
        self._snapshot()
        src = [self._sketch.shapes[i] for i in run]
        new = [backtrace_shape(sh, z_offset=z_offset, xy_offset=xy_offset,
                               print_on_return=print_on_return)
               for sh in reversed(src)]
        at = run[-1] + 1
        self._sketch.shapes[at:at] = new
        self._set_selection(set(range(at, at + len(new))))
        self.sketch_changed.emit()
        self.update()
        return len(new)


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
