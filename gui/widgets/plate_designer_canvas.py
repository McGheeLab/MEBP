"""
plate_designer_canvas.py — interactive 2-D canvas for the v7.4.5
parametric well-plate designer.

A `QGraphicsView` subclass that owns a `PlateDesign` + `PlateSketchSolver`
and renders the plate outline, the wells, and per-constraint markers.
Mouse handling is driven by a `Tool` enum so the same view supports
selecting, placing single wells, dropping a grid, and authoring
constraints between picked entities.

Key signals
-----------
* `selection_changed(list[int])`       — list of currently-selected entity ids
* `design_changed()`                   — design mutated (refresh properties)
* `solve_report(object)`               — `SolveReport` after every solve
* `hover_pos_changed(float, float)`    — cursor at (mm, mm) in design space

Scene Z-order (back → front):
    PlateOutlineItem (-30) → GridItem (-20) → WellItem (10) →
    ConstraintMarkerItem (20) → RubberBand (40)
"""

from __future__ import annotations

import logging
import math
from enum import Enum, auto
from typing import Optional

from PySide6.QtCore import (
    Qt, Signal, QRectF, QPointF, QSize, QTimer, QSizeF,
)
from PySide6.QtGui import (
    QBrush, QColor, QCursor, QFont, QMouseEvent, QPainter, QPen,
    QWheelEvent, QKeyEvent, QShortcut, QKeySequence,
)
from PySide6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsLineItem, QGraphicsTextItem,
    QGraphicsItemGroup, QGraphicsItem, QSizePolicy, QWidget,
)

from gui.scaling import s, scaled_font_size
from SupportClasses.PlateDesign import (
    PlateDesign, Point, Well, Group, Line, Constraint, EntityId,
)
from SupportClasses.PlateSketchSolver import (
    PlateSketchSolver, SolveReport, DOFStatus,
)

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

# mm → scene units (pixels at 1:1 zoom). Lifted from well_plate_view.py.
SCALE_FACTOR = 6.0

PAD = 12.0    # scene padding around plate outline

# Catppuccin Mocha palette inline (avoids cross-importing styles.py).
COLOR_BG = "#1e1e2e"
COLOR_PLATE_FILL = "#181825"
COLOR_PLATE_BORDER = "#45475a"
COLOR_GRID_MAJOR = "#313244"
COLOR_GRID_MINOR = "#262635"
COLOR_WELL_DEFAULT = "#74c7ec"      # sapphire
COLOR_WELL_LOCKED = "#a6e3a1"       # green (fixed)
COLOR_WELL_GROUP = "#f9e2af"        # yellow (grouped non-fixed)
COLOR_SELECTED = "#cba6f7"          # mauve
COLOR_HOVER = "#89b4fa"             # blue
COLOR_LABEL = "#cdd6f4"             # text
COLOR_CONSTRAINT = "#94e2d5"        # teal


# ═══════════════════════════════════════════════════════════════════
# Tool state machine
# ═══════════════════════════════════════════════════════════════════

class Tool(Enum):
    SELECT = auto()
    DRAW_SINGLE_WELL = auto()
    DRAW_GRID = auto()
    DRAW_CIRCLE_PATTERN = auto()
    DRAW_LINE = auto()
    DRAW_CONSTRUCTION_LINE = auto()
    DIMENSION = auto()          # v7.4.7 — edge-distance dimensions
    ADD_CONSTRAINT = auto()


# ═══════════════════════════════════════════════════════════════════
# Scene items
# ═══════════════════════════════════════════════════════════════════

class WellItem(QGraphicsEllipseItem):
    """A single well rendered as a filled circle. Stores the entity id."""

    def __init__(self, well: Well, x_mm: float, y_mm: float):
        radius = well.diameter / 2.0 * SCALE_FACTOR
        super().__init__(
            x_mm * SCALE_FACTOR - radius,
            y_mm * SCALE_FACTOR - radius,
            radius * 2, radius * 2,
        )
        self.entity_id = well.id
        self._selected = False
        self._hover = False
        self._fixed = False  # set by canvas based on whether center is fixed
        self._grouped = well.group is not None
        self.setAcceptHoverEvents(True)
        self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))
        self.setZValue(10)
        self._apply_style()
        self.setToolTip(f"{well.name} (Ø {well.diameter:.2f} mm)")

    def set_selected(self, sel: bool) -> None:
        self._selected = sel
        self._apply_style()

    def set_fixed(self, fixed: bool) -> None:
        self._fixed = fixed
        self._apply_style()

    def _apply_style(self) -> None:
        # Fill — locked > grouped > default.
        if self._fixed:
            fill = QColor(COLOR_WELL_LOCKED)
        elif self._grouped:
            fill = QColor(COLOR_WELL_GROUP)
        else:
            fill = QColor(COLOR_WELL_DEFAULT)
        fill.setAlpha(170)
        self.setBrush(QBrush(fill))

        if self._selected:
            pen = QPen(QColor(COLOR_SELECTED), 3.0)
        elif self._hover:
            pen = QPen(QColor(COLOR_HOVER), 2.0)
        else:
            pen = QPen(QColor(fill).darker(150), 1.5)
        self.setPen(pen)

    def hoverEnterEvent(self, event):
        self._hover = True
        self._apply_style()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self._hover = False
        self._apply_style()
        super().hoverLeaveEvent(event)


class RadiusHandleItem(QGraphicsEllipseItem):
    """Draggable endpoint of a circle pattern's radius (v7.4.8).

    Dragging it sets the ring radius (distance from centre) AND rotates
    the pattern (angle from centre). Stores the owning group's id.
    """

    HANDLE_PX = 7.0

    def __init__(self, group_id: int, x_mm: float, y_mm: float):
        rpx = self.HANDLE_PX
        super().__init__(
            x_mm * SCALE_FACTOR - rpx, y_mm * SCALE_FACTOR - rpx,
            rpx * 2, rpx * 2)
        self.group_id = group_id
        col = QColor(COLOR_CONSTRAINT)
        self.setBrush(QBrush(col))
        self.setPen(QPen(QColor("#1e1e2e"), 1.0))
        self.setZValue(23)
        self.setCursor(QCursor(Qt.CursorShape.SizeAllCursor))
        self.setToolTip("Drag to resize / rotate the pattern")


class PlateOutlineItem(QGraphicsRectItem):
    """Rectangular plate footprint. A1 sits at scene (0,0); the outline
    extends from (-a1_offset_x, -a1_offset_y) to
    (width-a1_offset_x, height-a1_offset_y) in mm so wells live INSIDE
    the rectangle rather than at its corner.
    """

    def __init__(
        self,
        width_mm: float,
        height_mm: float,
        a1_offset_x_mm: float = 0.0,
        a1_offset_y_mm: float = 0.0,
    ):
        super().__init__(
            -a1_offset_x_mm * SCALE_FACTOR,
            -a1_offset_y_mm * SCALE_FACTOR,
            width_mm * SCALE_FACTOR,
            height_mm * SCALE_FACTOR,
        )
        self.setBrush(QBrush(QColor(COLOR_PLATE_FILL)))
        self.setPen(QPen(QColor(COLOR_PLATE_BORDER), 2.0))
        self.setZValue(-30)


class LineItem(QGraphicsLineItem):
    """A renderable line in the design (solid or dashed when construction)."""

    def __init__(self, line, ax_mm: float, ay_mm: float,
                 bx_mm: float, by_mm: float):
        super().__init__(
            ax_mm * SCALE_FACTOR, ay_mm * SCALE_FACTOR,
            bx_mm * SCALE_FACTOR, by_mm * SCALE_FACTOR,
        )
        self.entity_id = line.id
        self._construction = line.construction
        if self._construction:
            pen = QPen(QColor(COLOR_CONSTRAINT), 1.4, Qt.PenStyle.DashLine)
        else:
            pen = QPen(QColor(COLOR_LABEL), 1.8, Qt.PenStyle.SolidLine)
        self.setPen(pen)
        self.setZValue(2)


class WellLabelItem(QGraphicsTextItem):
    """Small label rendered on top of each well (the well's name)."""

    def __init__(self, name: str, x_mm: float, y_mm: float,
                 diameter_mm: float):
        super().__init__(name)
        self.setDefaultTextColor(QColor(COLOR_LABEL))
        font = QFont("Segoe UI", 7)
        font.setBold(True)
        self.setFont(font)
        # Center the label on the well.
        br = self.boundingRect()
        self.setPos(
            x_mm * SCALE_FACTOR - br.width() / 2,
            y_mm * SCALE_FACTOR - br.height() / 2,
        )
        # Hide when wells are too small to fit the text.
        self._diameter_px = diameter_mm * SCALE_FACTOR
        if self._diameter_px < br.width() + 4:
            self.setVisible(False)
        self.setZValue(12)
        # Don't capture mouse events — clicks should hit the well underneath.
        self.setAcceptedMouseButtons(Qt.MouseButton.NoButton)


# ═══════════════════════════════════════════════════════════════════
# Canvas widget
# ═══════════════════════════════════════════════════════════════════

class PlateDesignerCanvas(QGraphicsView):
    """Interactive parametric plate sketcher view."""

    selection_changed = Signal(list)        # list[int] of entity ids
    design_changed = Signal()
    solve_report = Signal(object)           # SolveReport
    hover_pos_changed = Signal(float, float)  # (x_mm, y_mm)
    tool_changed = Signal(object)           # Tool enum
    well_drill_requested = Signal(int)      # v7.4.8: double-click a well to
                                            # zoom in + design its rosette

    # Grid spacing for the snap grid (mm).
    GRID_MAJOR_MM = 10.0
    GRID_MINOR_MM = 1.0

    # Default placement params for new wells.
    DEFAULT_WELL_DIAMETER_MM = 6.0
    DEFAULT_GRID_ROWS = 8
    DEFAULT_GRID_COLS = 12
    DEFAULT_GRID_SPACING_MM = 9.0
    # Circle Pattern defaults.
    DEFAULT_RING_COUNT = 8
    DEFAULT_RING_RADIUS_MM = 20.0

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self._design: Optional[PlateDesign] = None
        self._solver: Optional[PlateSketchSolver] = None

        self._well_items: dict[EntityId, WellItem] = {}
        self._line_items: dict[EntityId, LineItem] = {}
        self._outline_item: Optional[PlateOutlineItem] = None

        # Line-tool intermediate state — first click captured as a Point id.
        self._line_first_point: Optional[EntityId] = None

        # Selection state — entity ids (Wells primarily).
        self._selected_ids: list[EntityId] = []

        # Constraint-add pick state (used by ADD_CONSTRAINT tool).
        self._constraint_picks: list[EntityId] = []
        self._pending_constraint_kind: str = ""

        # Drag state.
        self._dragging_id: Optional[EntityId] = None
        self._drag_start_scene: Optional[QPointF] = None
        self._drag_timer = QTimer(self)
        self._drag_timer.setSingleShot(True)
        self._drag_timer.setInterval(33)        # ~30 fps
        self._drag_pending_target: Optional[tuple[float, float]] = None
        self._drag_timer.timeout.connect(self._flush_drag_update)

        # Pan state (middle mouse button).
        self._panning = False
        self._pan_start: Optional[QPointF] = None

        # Current tool.
        self._tool: Tool = Tool.SELECT

        # Snap settings.
        self._snap_to_grid = False

        # v7.4.7: scroll-wheel zoom toggle (default on = legacy behavior).
        self._wheel_zoom_enabled = True

        # v7.4.7: edge-dimension reference mode for the Dimension tool
        # ("center" | "edge") + tracked inline editor proxies.
        self._dim_ref_mode = "center"
        self._dim_editors: list = []

        # v7.4.8: Circle Pattern tool — configurable params + drag-to-set
        # radius (press snaps the centre → drag sets radius → release
        # commits). Radius also editable post-placement via the group card.
        self._circle_params = {
            "count": 6,
            "diameter": self.DEFAULT_WELL_DIAMETER_MM,
            "center_well": False,
        }
        self._circle_center: Optional[tuple] = None   # (x, y) mm while defining
        self._circle_radius: float = 0.0
        self._circle_defining: bool = False
        # Radius-handle drag (resize + rotate a placed circle pattern).
        self._radius_handles: dict = {}                # group_id → handle item
        self._dragging_radius_group: Optional[EntityId] = None

        # Undo / redo — snapshot stack of PlateDesign.to_dict() blobs.
        # Capacity caps memory: 50 × ~30 KB = ~1.5 MB for full plates.
        self._undo_stack: list[dict] = []
        self._redo_stack: list[dict] = []
        self._undo_cap: int = 50

        # Drag-preview overlay (ghost wells shown while a pattern tool is
        # active and the cursor is over the canvas). Cleared on tool
        # change / mouse leave.
        self._preview_items: list = []

        # View setup.
        self.setRenderHints(QPainter.RenderHint.Antialiasing
                            | QPainter.RenderHint.SmoothPixmapTransform)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(
            QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setBackgroundBrush(QBrush(QColor(COLOR_BG)))
        self.setMouseTracking(True)
        self.setMinimumSize(s(520), s(380))
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)
        self.setStyleSheet(f"border: 1px solid {COLOR_PLATE_BORDER};")

        # Keyboard shortcuts.
        QShortcut(QKeySequence("Esc"), self, activated=self._cancel_tool)
        QShortcut(QKeySequence("F"), self, activated=self.fit_view)
        QShortcut(QKeySequence("Delete"), self, activated=self.delete_selection)
        QShortcut(QKeySequence("Backspace"), self,
                  activated=self.delete_selection)

    # ── Background grid (drawn via drawBackground) ────────────────

    def drawBackground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawBackground(painter, rect)
        if self._design is None or self._outline_item is None:
            return

        # The outline is positioned in scene coordinates already (no
        # transform / parent offset), so its `rect()` IS the scene rect.
        plate_rect = self._outline_item.rect()

        # Minor grid (1 mm).
        minor_spacing = self.GRID_MINOR_MM * SCALE_FACTOR
        painter.setPen(QPen(QColor(COLOR_GRID_MINOR), 0))
        x = plate_rect.left()
        while x <= plate_rect.right():
            painter.drawLine(QPointF(x, plate_rect.top()),
                             QPointF(x, plate_rect.bottom()))
            x += minor_spacing
        y = plate_rect.top()
        while y <= plate_rect.bottom():
            painter.drawLine(QPointF(plate_rect.left(), y),
                             QPointF(plate_rect.right(), y))
            y += minor_spacing

        # Major grid (10 mm).
        major_spacing = self.GRID_MAJOR_MM * SCALE_FACTOR
        painter.setPen(QPen(QColor(COLOR_GRID_MAJOR), 0))
        x = plate_rect.left()
        while x <= plate_rect.right():
            painter.drawLine(QPointF(x, plate_rect.top()),
                             QPointF(x, plate_rect.bottom()))
            x += major_spacing
        y = plate_rect.top()
        while y <= plate_rect.bottom():
            painter.drawLine(QPointF(plate_rect.left(), y),
                             QPointF(plate_rect.right(), y))
            y += major_spacing

    # ── Public API ────────────────────────────────────────────────

    def set_design(self, design: PlateDesign) -> None:
        """Mount a new design — rebuilds the scene + solver + history."""
        self._design = design
        self._solver = PlateSketchSolver(design)
        self._selected_ids.clear()
        self._constraint_picks.clear()
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._rebuild_scene()
        # Initial solve — wakes the design up to a consistent state.
        if self._solver:
            report = self._solver.solve()
            self._apply_solver_result()
            self.solve_report.emit(report)
        QTimer.singleShot(50, self.fit_view)

    # ── Undo / redo ───────────────────────────────────────────────

    def push_undo_snapshot(self) -> None:
        """Capture the current design as an undo checkpoint.

        Mutators call this BEFORE making changes so Ctrl+Z brings the
        design back to its pre-change state. Redo stack is cleared on
        every mutation (standard undo/redo semantics).
        """
        if self._design is None:
            return
        snap = self._design.to_dict()
        self._undo_stack.append(snap)
        if len(self._undo_stack) > self._undo_cap:
            self._undo_stack.pop(0)
        self._redo_stack.clear()

    def can_undo(self) -> bool:
        return bool(self._undo_stack)

    def can_redo(self) -> bool:
        return bool(self._redo_stack)

    def undo(self) -> None:
        if not self._undo_stack or self._design is None:
            return
        self._redo_stack.append(self._design.to_dict())
        snap = self._undo_stack.pop()
        self._restore_snapshot(snap)

    def redo(self) -> None:
        if not self._redo_stack or self._design is None:
            return
        self._undo_stack.append(self._design.to_dict())
        snap = self._redo_stack.pop()
        self._restore_snapshot(snap)

    def _restore_snapshot(self, snap: dict) -> None:
        """Replace the live design's contents with a serialized snapshot."""
        if self._design is None:
            return
        restored = PlateDesign.from_dict(snap)
        # Swap fields on the *existing* design instance so external
        # references (e.g. the designer widget) stay valid.
        self._design.name = restored.name
        self._design.description = restored.description
        self._design.units = restored.units
        self._design.outline = restored.outline
        self._design.entities = restored.entities
        self._design.constraints = restored.constraints
        self._design.a1_offset_x = restored.a1_offset_x
        self._design.a1_offset_y = restored.a1_offset_y
        self._design.well_depth_default_mm = restored.well_depth_default_mm
        self._design._next_entity_id = restored._next_entity_id
        self._design._next_constraint_id = restored._next_constraint_id
        # Selection ids may now refer to dead entities — drop them.
        self._selected_ids = [
            eid for eid in self._selected_ids
            if eid in self._design.entities
        ]
        self._rebuild_scene()
        if self._solver:
            report = self._solver.solve()
            self._apply_solver_result()
            self.solve_report.emit(report)
        self.selection_changed.emit(list(self._selected_ids))
        self.design_changed.emit()

    @property
    def design(self) -> Optional[PlateDesign]:
        return self._design

    @property
    def solver(self) -> Optional[PlateSketchSolver]:
        return self._solver

    def set_tool(self, tool: Tool) -> None:
        self._tool = tool
        self._constraint_picks.clear()
        self._pending_constraint_kind = ""
        # Reset any in-progress circle-pattern definition.
        self._circle_defining = False
        self._circle_center = None
        self._clear_preview()
        # Cursor hints per tool.
        if tool == Tool.SELECT:
            self.setCursor(QCursor(Qt.CursorShape.ArrowCursor))
        elif tool in (Tool.DRAW_SINGLE_WELL, Tool.DRAW_GRID,
                      Tool.DRAW_CIRCLE_PATTERN):
            self.setCursor(QCursor(Qt.CursorShape.CrossCursor))
        elif tool in (Tool.ADD_CONSTRAINT, Tool.DIMENSION):
            self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.tool_changed.emit(tool)

    def set_dim_ref_mode(self, mode: str) -> None:
        """Set reference mode ('center'|'edge') for new edge dimensions."""
        self._dim_ref_mode = mode if mode in ("center", "edge") else "center"

    def set_circle_param(self, key: str, value) -> None:
        """Set a Circle Pattern parameter (count / diameter / center_well)."""
        if key in self._circle_params:
            self._circle_params[key] = value

    def get_circle_params(self) -> dict:
        return dict(self._circle_params)

    def get_tool(self) -> Tool:
        return self._tool

    def set_snap_to_grid(self, enabled: bool) -> None:
        self._snap_to_grid = enabled

    def set_selection(self, entity_ids: list[EntityId]) -> None:
        # Clear old selection.
        for eid in self._selected_ids:
            it = self._well_items.get(eid)
            if it:
                it.set_selected(False)
        self._selected_ids = list(entity_ids)
        for eid in self._selected_ids:
            it = self._well_items.get(eid)
            if it:
                it.set_selected(True)
        self.selection_changed.emit(list(self._selected_ids))

    def clear_selection(self) -> None:
        self.set_selection([])

    def selected_ids(self) -> list[EntityId]:
        return list(self._selected_ids)

    def begin_add_constraint(self, kind: str) -> None:
        """Switch to ADD_CONSTRAINT tool with the requested kind."""
        self._pending_constraint_kind = kind
        self.set_tool(Tool.ADD_CONSTRAINT)
        self._constraint_picks = []

    def lock_selection(self) -> None:
        """Toggle the `fix` constraint on currently-selected wells.

        Adds a `fix` constraint snapping each well's center to its
        current position. Re-running on already-locked wells removes
        the `fix` constraint instead.
        """
        if self._design is None or not self._selected_ids:
            return
        self.push_undo_snapshot()
        # Build a set of point ids currently fixed via `fix` constraints
        # so we can toggle off.
        existing: dict[EntityId, int] = {}
        for c in self._design.constraints:
            if c.kind == "fix" and c.refs:
                existing[c.refs[0]] = c.id

        for ent_id in self._selected_ids:
            well = self._design.entities.get(ent_id)
            if not isinstance(well, Well):
                continue
            center_id = well.center
            if center_id in existing:
                # Unlock.
                self._design.remove_constraint(existing[center_id])
            else:
                # Lock at current position.
                center = self._design.entities.get(center_id)
                snap = ((center.x, center.y)
                        if isinstance(center, Point) else None)
                self._design.add_constraint(Constraint(
                    kind="fix", refs=[center_id], snapshot=snap))
        self._resolve_and_repaint()

    def delete_selection(self) -> None:
        if self._design is None or not self._selected_ids:
            return
        self.push_undo_snapshot()
        for ent_id in list(self._selected_ids):
            self._design.remove_entity(ent_id)
        self._selected_ids.clear()
        self._rebuild_scene()
        self._resolve_and_repaint()
        self.selection_changed.emit([])

    def fit_view(self) -> None:
        if self._scene.itemsBoundingRect().isEmpty():
            return
        rect = self._scene.itemsBoundingRect().adjusted(-PAD, -PAD, PAD, PAD)
        self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    # ── Scene construction ────────────────────────────────────────

    def _rebuild_scene(self) -> None:
        """Clear and redraw everything from the design."""
        self._scene.clear()
        self._well_items.clear()
        self._line_items.clear()
        self._outline_item = None
        self._label_items: list = []
        self._preview_items = []           # cleared by scene.clear()
        self._dim_editors = []             # proxies destroyed by scene.clear()
        self._radius_handles = {}          # destroyed by scene.clear()

        if self._design is None:
            return

        # Plate outline. For rect plates A1 sits at scene origin (0,0)
        # and the outline extends back by (a1_offset_x, a1_offset_y) so
        # A1 is inset from the top-left corner. For a circular bore
        # (v7.4.7 rosette mode) the outline is an ellipse centered at the
        # origin (= the parent well's center).
        outline = self._design.outline
        if outline.kind == "circle":
            r_px = outline.radius * SCALE_FACTOR
            self._outline_item = QGraphicsEllipseItem(-r_px, -r_px,
                                                      r_px * 2, r_px * 2)
            self._outline_item.setBrush(QBrush(QColor(COLOR_PLATE_FILL)))
            self._outline_item.setPen(QPen(QColor(COLOR_PLATE_BORDER), 2.0))
            self._outline_item.setZValue(-30)
            self._scene.addItem(self._outline_item)
        else:
            self._outline_item = PlateOutlineItem(
                outline.width, outline.height,
                self._design.a1_offset_x, self._design.a1_offset_y,
            )
            self._scene.addItem(self._outline_item)

        # Origin marker (small cross at A1 — design coords (0,0)).
        cross_size = 6.0  # scene units
        pen = QPen(QColor(COLOR_CONSTRAINT), 1.2)
        h = self._scene.addLine(-cross_size, 0, cross_size, 0, pen)
        v = self._scene.addLine(0, -cross_size, 0, cross_size, pen)
        h.setZValue(-25); v.setZValue(-25)

        # Lines (rendered behind wells via z-order on the item).
        for ent in self._design.entities.values():
            if isinstance(ent, Line):
                p1 = self._design.entities.get(ent.p1)
                p2 = self._design.entities.get(ent.p2)
                if not (isinstance(p1, Point) and isinstance(p2, Point)):
                    continue
                item = LineItem(ent, p1.x, p1.y, p2.x, p2.y)
                self._scene.addItem(item)
                self._line_items[ent.id] = item

        # Wells + per-well name labels.
        fixed_centers = self._collect_fixed_point_ids()
        for well in self._design.get_wells():
            center = self._design.entities.get(well.center)
            if not isinstance(center, Point):
                continue
            item = WellItem(well, center.x, center.y)
            item.set_fixed(well.center in fixed_centers)
            # v7.4.8: on the top-level plate, hint that double-click drills
            # into the well to design its rosette (sub-wells).
            if self._design.outline.kind != "circle":
                has_ros = getattr(well, "rosette_design", None) is not None
                verb = "edit" if has_ros else "add"
                item.setToolTip(
                    f"{well.name} (Ø {well.diameter:.2f} mm)\n"
                    f"double-click to {verb} rosette / sub-wells")
            self._scene.addItem(item)
            self._well_items[well.id] = item

            label = WellLabelItem(
                well.name, center.x, center.y, well.diameter)
            self._scene.addItem(label)
            self._label_items.append(label)

        # Row / column header labels for grid-style plates.
        self._draw_grid_headers()

        # Constraint dimension markers (distance pills, lock glyphs).
        self._draw_constraint_markers()

        # Edge-distance dimension lines + inline editable fields.
        self._draw_dimensions()

        # v7.4.8: editable radius dimension on circle-pattern groups.
        self._draw_circle_radius_dimensions()

        # v7.4.8: rosette badge on parent wells that contain a rosette.
        self._draw_rosette_badges()

        # Force background redraw so the grid picks up plate dimensions.
        self.viewport().update()

    def _draw_circle_radius_dimensions(self) -> None:
        """For each circle-pattern group, draw a radius line + a read-only
        "r X.XX" label. The radius is edited in the group's properties
        card (a normal panel spinbox) — NOT an in-scene widget, which
        crashed when typing (QGraphicsProxyWidget + QAbstractSpinBox). v7.4.8."""
        if self._design is None:
            return
        from SupportClasses.PlateDesign import Group
        for ent in list(self._design.entities.values()):
            if not isinstance(ent, Group) or ent.pattern_kind != "circle":
                continue
            p = ent.params or {}
            cx = float(p.get("center_x", 0.0))
            cy = float(p.get("center_y", 0.0))
            r = float(p.get("radius", 0.0))
            if r <= 0:
                continue
            # Radius line drawn at the pattern's current start angle so
            # the draggable handle sits on a real ring well.
            start = math.radians(float(p.get("start_angle_deg", 0.0)))
            ex = cx + r * math.cos(start)
            ey = cy + r * math.sin(start)
            pen = QPen(QColor(COLOR_CONSTRAINT), 1.2)
            line = self._scene.addLine(
                cx * SCALE_FACTOR, cy * SCALE_FACTOR,
                ex * SCALE_FACTOR, ey * SCALE_FACTOR, pen)
            line.setZValue(18)
            line.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self._add_dimension_label(
                f"r {r:.2f}", (cx + ex) / 2.0 * SCALE_FACTOR,
                (cy + ey) / 2.0 * SCALE_FACTOR)
            # Draggable endpoint (resize + rotate).
            handle = RadiusHandleItem(ent.id, ex, ey)
            self._scene.addItem(handle)
            self._radius_handles[ent.id] = handle

    def _add_dimension_label(self, text: str, mx: float, my: float) -> None:
        """A read-only, zoom-invariant dimension caption on the canvas."""
        font = QFont("Segoe UI", 8)
        font.setBold(True)
        item = self._scene.addText(text, font)
        item.setDefaultTextColor(QColor(COLOR_CONSTRAINT))
        item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        item.setZValue(21)
        item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        br = item.boundingRect()
        item.setPos(mx - br.width() / 2, my - br.height() / 2)
        self._dim_editors.append(item)

    def _draw_rosette_badges(self) -> None:
        """Mark top-level wells that hold a rosette with a small ring of
        dots, so the user can see which wells contain inserts."""
        if self._design is None or self._design.outline.kind == "circle":
            return
        pen = QPen(QColor(COLOR_CONSTRAINT), 0)
        brush = QBrush(QColor(COLOR_CONSTRAINT))
        for well in self._design.get_wells():
            if getattr(well, "rosette_design", None) is None:
                continue
            center = self._design.entities.get(well.center)
            if not isinstance(center, Point):
                continue
            cx = center.x * SCALE_FACTOR
            cy = center.y * SCALE_FACTOR
            badge_r = max(well.diameter * 0.18, 0.6) * SCALE_FACTOR
            n_sub = len(well.rosette_design.get_wells())
            n_dots = min(max(n_sub, 3), 8)
            dot = max(badge_r * 0.18, 1.0)
            for i in range(n_dots):
                ang = 2 * math.pi * i / n_dots
                dx = cx + badge_r * math.cos(ang)
                dy = cy + badge_r * math.sin(ang)
                it = self._scene.addEllipse(
                    dx - dot, dy - dot, dot * 2, dot * 2, pen, brush)
                it.setZValue(14)
                it.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def _draw_dimensions(self) -> None:
        """Render dist_left_edge / dist_top_edge constraints as witness
        lines with an inline editable distance field at each midpoint.

        v7.4.7. Skipped in circular (rosette) mode — no rectangular edges.
        """
        if self._design is None or self._design.outline.kind == "circle":
            return
        left_x = -self._design.a1_offset_x
        top_y = -self._design.a1_offset_y
        pen = QPen(QColor(COLOR_CONSTRAINT), 1.2)

        for c in self._design.constraints:
            if c.kind not in ("dist_left_edge", "dist_top_edge"):
                continue
            if not c.refs:
                continue
            well = self._design.entities.get(c.refs[0])
            if not isinstance(well, Well):
                continue
            center = self._design.entities.get(well.center)
            if not isinstance(center, Point):
                continue
            r = well.diameter / 2.0 if c.mode == "edge" else 0.0
            cx, cy = center.x, center.y

            if c.kind == "dist_left_edge":
                x0, y0 = left_x, cy
                x1, y1 = cx - r, cy
            else:  # dist_top_edge
                x0, y0 = cx, top_y
                x1, y1 = cx, cy - r

            line = self._scene.addLine(
                x0 * SCALE_FACTOR, y0 * SCALE_FACTOR,
                x1 * SCALE_FACTOR, y1 * SCALE_FACTOR, pen)
            line.setZValue(18)
            # End ticks.
            tick = 3.0
            if c.kind == "dist_left_edge":
                self._scene.addLine(
                    x0 * SCALE_FACTOR, (y0 - tick), x0 * SCALE_FACTOR,
                    (y0 + tick), pen).setZValue(18)
            else:
                self._scene.addLine(
                    (x0 * SCALE_FACTOR - tick), y0 * SCALE_FACTOR,
                    (x0 * SCALE_FACTOR + tick), y0 * SCALE_FACTOR,
                    pen).setZValue(18)

            # Read-only caption at the midpoint. Editing happens in the
            # well's Constraints card (a normal panel spinbox) — embedding
            # a spinbox in the scene crashed on key input.
            mx = (x0 + x1) / 2.0 * SCALE_FACTOR
            my = (y0 + y1) / 2.0 * SCALE_FACTOR
            val = c.value if c.value is not None else 0.0
            arrow = "↔" if c.kind == "dist_left_edge" else "↕"
            self._add_dimension_label(f"{arrow} {val:.2f}", mx, my)

    def set_constraint_value(self, constraint_id: int, value: float) -> None:
        """Set a constraint's numeric value + re-solve (called from the
        properties panel — safe, normal-widget editing). v7.4.8."""
        if self._design is None:
            return
        target = next((c for c in self._design.constraints
                       if c.id == constraint_id), None)
        if target is None or target.value == value:
            return
        self.push_undo_snapshot()
        target.value = value
        report = None
        if self._solver:
            report = self._solver.solve()
            self._apply_solver_result()
        self._rebuild_scene()
        self.design_changed.emit()
        if report is not None:
            self.solve_report.emit(report)

    def _draw_constraint_markers(self) -> None:
        """Lightweight visual annotations on top of the scene.

        v7.4.6: lock glyph on fixed wells; mid-segment "↔ <mm>" pill on
        distance constraints. Skips when zoom is too low or well-count
        too high to keep the canvas readable.
        """
        if self._design is None:
            return
        wells = self._design.get_wells()
        # Heuristic: hide markers on dense plates (96+ wells) — too much
        # visual noise.
        if len(wells) > 64:
            return

        # Lock glyphs on fixed wells.
        font = QFont("Segoe UI", 8)
        font.setBold(True)
        lock_color = QColor("#f9e2af")  # yellow
        fixed_centers = self._collect_fixed_point_ids()
        for well in wells:
            if well.center not in fixed_centers:
                continue
            center = self._design.entities.get(well.center)
            if not isinstance(center, Point):
                continue
            t = self._scene.addText("🔒", font)
            t.setDefaultTextColor(lock_color)
            br = t.boundingRect()
            radius_px = well.diameter / 2.0 * SCALE_FACTOR
            t.setPos(
                center.x * SCALE_FACTOR + radius_px * 0.4 - br.width() / 2,
                center.y * SCALE_FACTOR - radius_px - br.height(),
            )
            t.setZValue(22)
            t.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

        # Distance constraints: "↔ X.X mm" mid-segment.
        for c in self._design.constraints:
            if c.kind == "distance_pp" and c.value is not None and len(c.refs) >= 2:
                p1 = self._design.entities.get(c.refs[0])
                p2 = self._design.entities.get(c.refs[1])
                if not (isinstance(p1, Point) and isinstance(p2, Point)):
                    continue
                mid_x = (p1.x + p2.x) / 2.0 * SCALE_FACTOR
                mid_y = (p1.y + p2.y) / 2.0 * SCALE_FACTOR
                text = self._scene.addText(
                    f"↔ {c.value:.2f}", font)
                text.setDefaultTextColor(QColor(COLOR_CONSTRAINT))
                br = text.boundingRect()
                text.setPos(mid_x - br.width() / 2,
                            mid_y - br.height() / 2)
                text.setZValue(21)
                text.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def _draw_grid_headers(self) -> None:
        """Draw row letters and column numbers around a regular grid.

        Detects "grid-likeness" by checking whether wells in the same
        group share a `pattern_kind == "grid"`. Falls back gracefully
        when wells are irregular.
        """
        if self._design is None:
            return
        # No A/B/C row headers for a circular rosette bore.
        if self._design.outline.kind == "circle":
            return
        # Find the first grid group and use its dimensions.
        grid_group = next(
            (e for e in self._design.entities.values()
             if hasattr(e, "pattern_kind")
             and getattr(e, "pattern_kind", "") == "grid"),
            None,
        )
        if grid_group is None:
            return
        params = getattr(grid_group, "params", {}) or {}
        rows = int(params.get("rows", 0))
        cols = int(params.get("cols", 0))
        sx = float(params.get("spacing_x", 0.0))
        sy = float(params.get("spacing_y", 0.0))
        if rows <= 0 or cols <= 0 or sx <= 0 or sy <= 0:
            return

        # Bring in the ANSI row letters.
        from SupportClasses.WellPlate import ROW_LABELS

        font = QFont("Segoe UI", 9)
        font.setBold(True)
        label_color = QColor(COLOR_LABEL)
        label_color.setAlpha(180)

        # Row labels at x = -spacing_x/2 (just left of A column).
        max_well_d = max(
            (w.diameter for w in self._design.get_wells()),
            default=6.0,
        )
        margin = max(max_well_d * 0.8, 2.0) * SCALE_FACTOR
        for r in range(rows):
            letter = ROW_LABELS[r] if r < len(ROW_LABELS) else str(r)
            t = self._scene.addText(letter, font)
            t.setDefaultTextColor(label_color)
            br = t.boundingRect()
            t.setPos(
                -margin - br.width(),
                r * sy * SCALE_FACTOR - br.height() / 2,
            )
            t.setZValue(-15)
        for c in range(cols):
            num = str(c + 1)
            t = self._scene.addText(num, font)
            t.setDefaultTextColor(label_color)
            br = t.boundingRect()
            t.setPos(
                c * sx * SCALE_FACTOR - br.width() / 2,
                -margin - br.height(),
            )
            t.setZValue(-15)

    def _collect_fixed_point_ids(self) -> set[EntityId]:
        fixed: set[EntityId] = set()
        if self._design is None:
            return fixed
        for ent in self._design.entities.values():
            if isinstance(ent, Point) and ent.fixed:
                fixed.add(ent.id)
        for c in self._design.constraints:
            if c.kind in ("ground", "fix"):
                fixed.update(c.refs)
        return fixed

    def _apply_solver_result(self) -> None:
        """After a solve, re-position WellItems to match solved Points."""
        if self._design is None:
            return
        fixed_centers = self._collect_fixed_point_ids()
        for well in self._design.get_wells():
            item = self._well_items.get(well.id)
            if item is None:
                continue
            center = self._design.entities.get(well.center)
            if not isinstance(center, Point):
                continue
            radius = well.diameter / 2.0 * SCALE_FACTOR
            item.setRect(
                center.x * SCALE_FACTOR - radius,
                center.y * SCALE_FACTOR - radius,
                radius * 2, radius * 2,
            )
            item.set_fixed(well.center in fixed_centers)
            item.setToolTip(f"{well.name} (Ø {well.diameter:.2f} mm)")

    def _resolve_and_repaint(self) -> None:
        """Run the solver + apply + emit reports."""
        if self._solver is None:
            return
        report = self._solver.solve()
        self._apply_solver_result()
        self.design_changed.emit()
        self.solve_report.emit(report)

    # ── Geometry helpers ──────────────────────────────────────────

    def _scene_to_mm(self, scene_pt: QPointF) -> tuple[float, float]:
        return (scene_pt.x() / SCALE_FACTOR, scene_pt.y() / SCALE_FACTOR)

    def _snap_mm(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        if not self._snap_to_grid:
            return (x_mm, y_mm)
        step = self.GRID_MINOR_MM
        return (round(x_mm / step) * step, round(y_mm / step) * step)

    def _well_id_at(self, scene_pt: QPointF) -> Optional[EntityId]:
        for it in self._scene.items(scene_pt):
            if isinstance(it, WellItem):
                return it.entity_id
        return None

    def _entity_id_at(self, scene_pt: QPointF) -> Optional[EntityId]:
        """Find the topmost interactive entity (well or line) at the cursor."""
        for it in self._scene.items(scene_pt):
            if isinstance(it, (WellItem, LineItem)):
                return it.entity_id
        return None

    def _radius_handle_at(self, scene_pt: QPointF) -> Optional[EntityId]:
        """Group id whose radius handle is under the cursor, else None."""
        for it in self._scene.items(scene_pt):
            if isinstance(it, RadiusHandleItem):
                return it.group_id
        return None

    # ── Mouse handlers ────────────────────────────────────────────

    def mousePressEvent(self, event: QMouseEvent) -> None:
        scene_pt = self.mapToScene(event.pos())
        mm = self._scene_to_mm(scene_pt)

        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(QCursor(Qt.CursorShape.ClosedHandCursor))
            event.accept()
            return

        if event.button() == Qt.MouseButton.LeftButton:
            ctrl = bool(event.modifiers() & Qt.KeyboardModifier.ControlModifier)
            # Radius handle takes priority in SELECT mode (resize+rotate).
            if self._tool == Tool.SELECT:
                gid = self._radius_handle_at(scene_pt)
                if gid is not None:
                    self._dragging_radius_group = gid
                    self.push_undo_snapshot()
                    event.accept()
                    return
            if self._tool == Tool.SELECT:
                self._handle_select_press(scene_pt, ctrl)
            elif self._tool == Tool.DRAW_SINGLE_WELL:
                self._handle_draw_well(mm)
            elif self._tool == Tool.DRAW_GRID:
                self._handle_draw_grid(mm)
            elif self._tool == Tool.DRAW_CIRCLE_PATTERN:
                self._circle_press(mm)
            elif self._tool in (Tool.DRAW_LINE, Tool.DRAW_CONSTRUCTION_LINE):
                self._handle_draw_line(mm,
                                       construction=(
                                           self._tool ==
                                           Tool.DRAW_CONSTRUCTION_LINE))
            elif self._tool == Tool.DIMENSION:
                self._handle_dimension(scene_pt)
            elif self._tool == Tool.ADD_CONSTRAINT:
                self._handle_add_constraint_pick(scene_pt)
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        scene_pt = self.mapToScene(event.pos())
        mm = self._scene_to_mm(scene_pt)
        self.hover_pos_changed.emit(*mm)

        if self._panning and self._pan_start is not None:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            h = self.horizontalScrollBar()
            v = self.verticalScrollBar()
            h.setValue(h.value() - int(delta.x()))
            v.setValue(v.value() - int(delta.y()))
            event.accept()
            return

        # Radius-handle drag on a placed circle pattern: resize + rotate.
        if self._dragging_radius_group is not None and self._design is not None:
            self._drag_radius_handle(mm)
            event.accept()
            return

        # Circle Pattern: while defining the radius (button held), drag
        # from the snapped centre outward.
        if self._circle_defining and self._circle_center is not None:
            cx, cy = self._circle_center
            self._circle_radius = max(math.hypot(mm[0] - cx, mm[1] - cy), 0.1)
            self._draw_circle_drag_preview()
            event.accept()
            return

        # Pattern-tool preview overlay (hover, before pressing).
        if self._tool == Tool.DRAW_GRID:
            self._update_grid_preview(self._snap_mm(*mm))
        elif self._tool == Tool.DRAW_CIRCLE_PATTERN:
            self._update_circle_preview(self._snap_mm(*mm))
        elif self._tool == Tool.DRAW_SINGLE_WELL:
            self._update_single_well_preview(self._snap_mm(*mm))

        if self._dragging_id is not None and self._solver is not None:
            # Lazy snapshot — only capture on first real movement.
            if not self._drag_snapshot_taken:
                self.push_undo_snapshot()
                self._drag_snapshot_taken = True
            snapped = self._snap_mm(*mm)
            self._drag_pending_target = snapped
            if not self._drag_timer.isActive():
                self._drag_timer.start()
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.MiddleButton and self._panning:
            self._panning = False
            self._pan_start = None
            self.setCursor(QCursor(Qt.CursorShape.ArrowCursor))
            event.accept()
            return

        # Radius-handle drag finished → solve + notify.
        if (event.button() == Qt.MouseButton.LeftButton
                and self._dragging_radius_group is not None):
            self._dragging_radius_group = None
            self._resolve_and_repaint()
            event.accept()
            return

        # Circle Pattern: release commits the ring at the dragged radius.
        if (event.button() == Qt.MouseButton.LeftButton
                and self._circle_defining):
            self._commit_circle()
            event.accept()
            return

        if (event.button() == Qt.MouseButton.LeftButton
                and self._dragging_id is not None and self._solver):
            # Flush any pending drag move.
            if self._drag_pending_target is not None:
                self._solver.update_drag(self._drag_pending_target)
                self._drag_pending_target = None
            report = self._solver.end_drag()
            self._dragging_id = None
            self._apply_solver_result()
            self.design_changed.emit()
            self.solve_report.emit(report)
            event.accept()
            return

        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        """v7.4.8: double-click a well on the top-level plate to zoom in
        and design its rosette (sub-wells). No-op inside a rosette (the
        outline is circular) or while a draw tool is active."""
        if (event.button() == Qt.MouseButton.LeftButton
                and self._design is not None
                and self._design.outline.kind != "circle"
                and self._tool == Tool.SELECT):
            # Cancel any in-progress drag started by the preceding press.
            if self._dragging_id is not None and self._solver is not None:
                self._solver.end_drag()
                self._dragging_id = None
            eid = self._well_id_at(self.mapToScene(event.pos()))
            if eid is not None:
                self.well_drill_requested.emit(eid)
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event: QWheelEvent) -> None:
        # v7.4.7: wheel zoom is toggleable — some users don't want it.
        if not self._wheel_zoom_enabled:
            event.ignore()
            return
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def set_wheel_zoom_enabled(self, enabled: bool) -> None:
        """Enable/disable scroll-wheel zoom (Fit + buttons still work)."""
        self._wheel_zoom_enabled = bool(enabled)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        # Tool shortcuts handled by parent's shortcuts; canvas-local
        # only handles modifiers that affect drag-in-progress.
        super().keyPressEvent(event)

    # ── Tool handlers ─────────────────────────────────────────────

    def _handle_select_press(self, scene_pt: QPointF, ctrl: bool) -> None:
        if self._design is None or self._solver is None:
            return
        # Wells before lines if both are under the cursor (well sit on top
        # of lines in z-order anyway, but be explicit).
        eid = self._well_id_at(scene_pt)
        if eid is None:
            eid = self._entity_id_at(scene_pt)
        if eid is None:
            # Empty click → clear selection.
            if not ctrl:
                self.clear_selection()
            return

        # Toggle or set selection.
        if ctrl:
            if eid in self._selected_ids:
                self._selected_ids.remove(eid)
                it = self._well_items.get(eid)
                if it:
                    it.set_selected(False)
            else:
                self._selected_ids.append(eid)
                it = self._well_items.get(eid)
                if it:
                    it.set_selected(True)
            self.selection_changed.emit(list(self._selected_ids))
            return

        # Replace selection.
        self.set_selection([eid])
        ent = self._design.entities.get(eid)
        # Only wells are draggable; lines are click-to-select.
        if not isinstance(ent, Well):
            return
        center = self._design.entities.get(ent.center)
        if not isinstance(center, Point):
            return
        # Snapshot the pre-drag state so Ctrl+Z reverts the entire drag.
        # Pushed lazily on first mouseMove to avoid pushing for bare clicks.
        self._dragging_id = eid
        self._drag_start_scene = scene_pt
        self._drag_snapshot_taken = False
        # Begin drag at the well's current position (no jump on press).
        self._solver.begin_drag(ent.center, (center.x, center.y))

    def _flush_drag_update(self) -> None:
        if (self._solver is None
                or self._drag_pending_target is None
                or self._dragging_id is None):
            return
        report = self._solver.update_drag(self._drag_pending_target)
        self._drag_pending_target = None
        self._apply_solver_result()
        self.solve_report.emit(report)

    def _handle_draw_well(self, mm: tuple[float, float]) -> None:
        if self._design is None:
            return
        self.push_undo_snapshot()
        x, y = self._snap_mm(*mm)
        name = self._next_custom_name()
        well = self._design.add_well(
            x=x, y=y,
            diameter=self.DEFAULT_WELL_DIAMETER_MM,
            name=name, naming_scheme="MANUAL",
        )
        self._rebuild_scene()
        self._resolve_and_repaint()
        # Return to select and pre-select the new well so the user
        # can immediately edit its name / diameter / position.
        self.set_tool(Tool.SELECT)
        self.set_selection([well.id])

    def _handle_draw_grid(self, mm: tuple[float, float]) -> None:
        if self._design is None:
            return
        self.push_undo_snapshot()
        x, y = self._snap_mm(*mm)
        # Use defaults — the properties panel can refine later.
        self._design.add_grid(
            rows=self.DEFAULT_GRID_ROWS,
            cols=self.DEFAULT_GRID_COLS,
            spacing_x=self.DEFAULT_GRID_SPACING_MM,
            spacing_y=self.DEFAULT_GRID_SPACING_MM,
            origin_x=x, origin_y=y,
            diameter=self.DEFAULT_WELL_DIAMETER_MM,
            group_name=self._next_group_name(),
        )
        self._relabel_if_rosette()
        self._rebuild_scene()
        self._resolve_and_repaint()
        self.set_tool(Tool.SELECT)

    def _handle_dimension(self, scene_pt: QPointF) -> None:
        """Click a well → add editable left+top edge-distance dimensions.

        Distances are seeded from the well's current position so nothing
        jumps; the user then edits the inline fields.
        """
        if self._design is None:
            return
        eid = self._well_id_at(scene_pt)
        if eid is None:
            return
        well = self._design.entities.get(eid)
        if not isinstance(well, Well):
            return
        center = self._design.entities.get(well.center)
        if not isinstance(center, Point):
            return
        mode = self._dim_ref_mode
        r = well.diameter / 2.0 if mode == "edge" else 0.0
        left_x = -self._design.a1_offset_x
        top_y = -self._design.a1_offset_y
        cur_left = (center.x - r) - left_x
        cur_top = (center.y - r) - top_y

        # Skip duplicates: one left + one top dim per well.
        existing = {
            c.kind for c in self._design.constraints
            if c.refs and c.refs[0] == eid
            and c.kind in ("dist_left_edge", "dist_top_edge")
        }
        if existing >= {"dist_left_edge", "dist_top_edge"}:
            self.set_selection([eid])
            self.set_tool(Tool.SELECT)
            return

        self.push_undo_snapshot()
        if "dist_left_edge" not in existing:
            self._design.add_constraint(Constraint(
                kind="dist_left_edge", refs=[eid],
                value=round(cur_left, 3), mode=mode))
        if "dist_top_edge" not in existing:
            self._design.add_constraint(Constraint(
                kind="dist_top_edge", refs=[eid],
                value=round(cur_top, 3), mode=mode))
        self.set_selection([eid])
        self.set_tool(Tool.SELECT)
        self._rebuild_scene()
        self._resolve_and_repaint()

    def _handle_draw_line(self, mm: tuple[float, float],
                          construction: bool) -> None:
        """Two-click line creation.

        First click drops a Point at the cursor and captures it; second
        click drops the second Point + a Line connecting them, then
        returns to SELECT.

        Construction lines have their endpoints fixed by default so the
        line acts as immovable reference geometry — users can constrain
        wells onto it without the line itself drifting. Regular lines
        leave endpoints free.
        """
        if self._design is None:
            return
        snapped = self._snap_mm(*mm)
        if self._line_first_point is None:
            self.push_undo_snapshot()
            p1 = self._design.add_entity(Point(
                x=snapped[0], y=snapped[1], fixed=construction))
            self._line_first_point = p1.id
            return
        # Second click.
        p2 = self._design.add_entity(Point(
            x=snapped[0], y=snapped[1], fixed=construction))
        line = self._design.add_entity(Line(
            p1=self._line_first_point, p2=p2.id,
            construction=construction,
        ))
        self._line_first_point = None
        self._rebuild_scene()
        self._resolve_and_repaint()
        self.set_tool(Tool.SELECT)
        # Pre-select the line so the user can edit / delete it immediately.
        self.set_selection([line.id])

    def _relabel_if_rosette(self) -> None:
        """In rosette (circular) mode, keep sub-well names as letters a,b,c…
        so flattened names read A1.a, A1.b (v7.4.8)."""
        if self._design is not None and self._design.outline.kind == "circle":
            self._design.relabel_wells_as_letters()

    def _snap_circle_center(self, mm: tuple[float, float]) -> tuple[float, float]:
        """Snap the ring centre to the bore/plate centre by default.

        v7.4.8: in rosette mode the bore is centred at the origin (0,0); a
        press within ~40% of the bore radius snaps there so rings are
        concentric by default. Falls back to the grid-snapped cursor.
        """
        if self._design is not None and self._design.outline.kind == "circle":
            bore = self._design.outline.radius or 0.0
            if math.hypot(mm[0], mm[1]) <= max(bore * 0.4, 3.0):
                return (0.0, 0.0)
        return self._snap_mm(*mm)

    def _handle_draw_circle_pattern(self, mm: tuple[float, float]) -> None:
        """Click-to-drop convenience: place a ring at a default radius using
        the current params (no drag). The interactive flow is
        `_circle_press` → drag → `_commit_circle`."""
        self._circle_press(mm)
        self._commit_circle()

    def _drag_radius_handle(self, mm: tuple[float, float]) -> None:
        """Live resize + rotate while dragging a ring's radius handle.

        The handle's polar position relative to the ring centre sets both
        the radius (distance) and the start angle (direction). Repositions
        existing ring wells in place (no rebuild) for smoothness. v7.4.8.
        """
        if self._design is None:
            return
        from SupportClasses.PlateDesign import Group
        grp = self._design.entities.get(self._dragging_radius_group)
        if not isinstance(grp, Group):
            return
        p = grp.params
        cx = float(p.get("center_x", 0.0))
        cy = float(p.get("center_y", 0.0))
        dx, dy = mm[0] - cx, mm[1] - cy
        radius = max(math.hypot(dx, dy), 0.1)
        angle = math.degrees(math.atan2(dy, dx))
        self._design.update_circle_layout(
            self._dragging_radius_group, radius, angle)
        self._rebuild_scene()   # redraw wells + handle (rings are small)

    def _circle_press(self, mm: tuple[float, float]) -> None:
        """Begin defining a circle pattern: snap centre, await drag radius."""
        if self._design is None:
            return
        self._circle_center = self._snap_circle_center(mm)
        # Seed a small radius; the drag (or the params field) sets the rest.
        self._circle_radius = max(
            (self._design.outline.radius or 0.0) * 0.5, 1.0)
        self._circle_defining = True
        self._draw_circle_drag_preview()

    def _draw_circle_drag_preview(self) -> None:
        """Preview ring + radius dimension line while defining."""
        self._clear_preview()
        if self._circle_center is None:
            return
        cx, cy = self._circle_center
        r = self._circle_radius
        params = self._circle_params
        n = max(int(params.get("count", 6)), 1)
        sub_d = float(params.get("diameter", self.DEFAULT_WELL_DIAMETER_MM))
        # Ring outline.
        ring_pen = QPen(QColor(COLOR_SELECTED), 1.0, Qt.PenStyle.DashLine)
        ring = self._scene.addEllipse(
            (cx - r) * SCALE_FACTOR, (cy - r) * SCALE_FACTOR,
            r * 2 * SCALE_FACTOR, r * 2 * SCALE_FACTOR,
            ring_pen, QBrush(Qt.BrushStyle.NoBrush))
        ring.setZValue(14)
        ring.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self._preview_items.append(ring)
        # Ghost wells (center + ring).
        if params.get("center_well"):
            self._add_preview_circle(cx, cy, sub_d / 2.0)
        for i in range(n):
            ang = (2 * math.pi * i) / n
            self._add_preview_circle(
                cx + r * math.cos(ang), cy + r * math.sin(ang), sub_d / 2.0)
        # Radius dimension line + live readout.
        line_pen = QPen(QColor(COLOR_CONSTRAINT), 1.2)
        line = self._scene.addLine(
            cx * SCALE_FACTOR, cy * SCALE_FACTOR,
            (cx + r) * SCALE_FACTOR, cy * SCALE_FACTOR, line_pen)
        line.setZValue(19)
        line.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self._preview_items.append(line)
        font = QFont("Segoe UI", 9)
        font.setBold(True)
        txt = self._scene.addText(f"r = {r:.2f} mm", font)
        txt.setDefaultTextColor(QColor(COLOR_CONSTRAINT))
        txt.setPos((cx + r / 2.0) * SCALE_FACTOR,
                   cy * SCALE_FACTOR - txt.boundingRect().height())
        txt.setZValue(20)
        txt.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self._preview_items.append(txt)

    def _commit_circle(self) -> None:
        """Place the circle pattern with the current params + dragged radius."""
        if self._design is None or self._circle_center is None:
            self._circle_defining = False
            return
        self.push_undo_snapshot()
        cx, cy = self._circle_center
        params = self._circle_params
        self._design.add_circle_pattern(
            count=max(int(params.get("count", 6)), 1),
            center_x=cx, center_y=cy,
            radius=max(self._circle_radius, 0.1),
            diameter=float(params.get("diameter", self.DEFAULT_WELL_DIAMETER_MM)),
            center_well=bool(params.get("center_well", False)),
            group_name=self._next_ring_name(),
        )
        self._relabel_if_rosette()
        self._circle_defining = False
        self._circle_center = None
        self._clear_preview()
        # Clear selection so the placed ring's options show in the standard
        # (no-selection) properties view. v7.4.8.
        self._selected_ids = []
        self._rebuild_scene()
        self._resolve_and_repaint()
        self.set_tool(Tool.SELECT)
        self.selection_changed.emit([])

    def _handle_add_constraint_pick(self, scene_pt: QPointF) -> None:
        if self._design is None:
            return
        eid = self._well_id_at(scene_pt)
        if eid is None:
            return
        self._constraint_picks.append(eid)
        self.set_selection(list(self._constraint_picks))
        # Apply once enough picks have been collected for this kind.
        if self._enough_picks_for_kind(self._pending_constraint_kind):
            self._commit_pending_constraint()

    @staticmethod
    def _enough_picks_for_kind(kind: str) -> bool:
        if kind == "fix":
            return False  # single-shot; UI applies via Lock action
        if kind in ("coincident_pp", "concentric", "distance_pp",
                    "horizontal", "vertical", "equal_radius"):
            return False  # finalized via _commit_pending_constraint with =2
        return False

    def _commit_pending_constraint(self) -> None:
        if (self._design is None
                or not self._pending_constraint_kind
                or len(self._constraint_picks) < 2):
            return
        kind = self._pending_constraint_kind
        if kind in ("coincident_pp", "horizontal", "vertical",
                    "concentric", "equal_radius"):
            self._add_simple_constraint(kind, self._constraint_picks[:2])
        elif kind == "distance_pp":
            # Default to current distance, user edits in properties panel.
            w1 = self._design.entities.get(self._constraint_picks[0])
            w2 = self._design.entities.get(self._constraint_picks[1])
            d = 0.0
            if isinstance(w1, Well) and isinstance(w2, Well):
                c1 = self._design.entities[w1.center]
                c2 = self._design.entities[w2.center]
                if isinstance(c1, Point) and isinstance(c2, Point):
                    d = math.hypot(c1.x - c2.x, c1.y - c2.y)
            self._add_simple_constraint(
                kind, self._constraint_picks[:2], value=d)
        # Reset.
        self._constraint_picks = []
        self._pending_constraint_kind = ""
        self.set_tool(Tool.SELECT)
        self._resolve_and_repaint()

    def _add_simple_constraint(
        self, kind: str, well_ids: list[EntityId],
        value: Optional[float] = None,
    ) -> None:
        """Translate Well refs to their underlying Point ids and add."""
        if self._design is None:
            return
        refs: list[EntityId] = []
        for wid in well_ids:
            ent = self._design.entities.get(wid)
            if isinstance(ent, Well):
                refs.append(ent.center)
            else:
                refs.append(wid)
        if kind == "equal_radius":
            # equal_radius applies to Wells directly (not their centers).
            refs = list(well_ids)
        self._design.add_constraint(Constraint(
            kind=kind, refs=refs, value=value))

    def rebuild_group_now(
        self, group_id: EntityId, new_params: dict,
    ) -> None:
        """Public group-edit API for the properties panel."""
        if self._design is None:
            return
        self.push_undo_snapshot()
        try:
            self._design.rebuild_group(group_id, new_params)
        except Exception as e:
            logger.warning(f"rebuild_group failed: {e}")
            return
        # Keep rosette sub-well letter names after a rebuild.
        self._relabel_if_rosette()
        # Selection may now point at deleted wells — clear it; the user
        # can re-pick from the refreshed group.
        self._selected_ids.clear()
        self._rebuild_scene()
        self._resolve_and_repaint()
        self.selection_changed.emit([])

    def add_constraint_now(self, kind: str, value: Optional[float] = None) -> bool:
        """Apply a constraint to the current selection. Returns True on success.

        Used by the properties panel "Add" buttons that don't go through
        the canvas pick state machine.
        """
        if self._design is None or len(self._selected_ids) < 2:
            return False
        self.push_undo_snapshot()
        self._add_simple_constraint(kind, self._selected_ids[:2], value=value)
        self._resolve_and_repaint()
        return True

    def add_constraint_explicit(
        self,
        kind: str,
        refs: list[EntityId],
        value: Optional[float] = None,
    ) -> bool:
        """Add a constraint with explicit refs (skip well→center translation).

        Used by mixed-selection actions (e.g. point-on-line) where the
        caller already resolved the right entity ids.
        """
        if self._design is None or not refs:
            return False
        self.push_undo_snapshot()
        self._design.add_constraint(Constraint(
            kind=kind, refs=list(refs), value=value))
        self._resolve_and_repaint()
        return True

    # ── Naming helpers ────────────────────────────────────────────

    def _next_custom_name(self) -> str:
        """Generate a unique name for a new free well.

        In rosette mode (circular outline) sub-wells get letter names
        a, b, c… so flattened names read A1.a, A1.b (v7.4.8). On a normal
        plate, free wells get 'Custom-N'.
        """
        if self._design is None:
            return "Custom-1"
        existing = {w.name for w in self._design.get_wells()}
        if self._design.outline.kind == "circle":
            # Letter suffixes a, b, … (skip taken ones).
            i = 0
            while True:
                s = ""
                n = i + 1
                while n > 0:
                    n, rem = divmod(n - 1, 26)
                    s = chr(ord("a") + rem) + s
                if s not in existing:
                    return s
                i += 1
        i = 1
        while f"Custom-{i}" in existing:
            i += 1
        return f"Custom-{i}"

    def _next_group_name(self) -> str:
        if self._design is None:
            return "grid-1"
        existing = {
            e.name for e in self._design.entities.values()
            if isinstance(e, Group)
        }
        i = 1
        while f"grid-{i}" in existing:
            i += 1
        return f"grid-{i}"

    def _next_ring_name(self) -> str:
        if self._design is None:
            return "ring-1"
        existing = {
            e.name for e in self._design.entities.values()
            if isinstance(e, Group)
        }
        i = 1
        while f"ring-{i}" in existing:
            i += 1
        return f"ring-{i}"

    def _cancel_tool(self) -> None:
        self._constraint_picks = []
        self._pending_constraint_kind = ""
        self.set_tool(Tool.SELECT)
        self.clear_selection()

    # ── Drag preview overlay ──────────────────────────────────────

    def leaveEvent(self, event) -> None:
        self._clear_preview()
        super().leaveEvent(event)

    def _clear_preview(self) -> None:
        for it in self._preview_items:
            try:
                self._scene.removeItem(it)
            except Exception:
                pass
        self._preview_items = []

    def _make_preview_pen(self) -> QPen:
        col = QColor(COLOR_SELECTED)
        col.setAlpha(140)
        pen = QPen(col, 1.4, Qt.PenStyle.DashLine)
        return pen

    def _make_preview_brush(self) -> QBrush:
        col = QColor(COLOR_SELECTED)
        col.setAlpha(40)
        return QBrush(col)

    def _add_preview_circle(self, x_mm: float, y_mm: float,
                            r_mm: float) -> None:
        radius_px = r_mm * SCALE_FACTOR
        it = self._scene.addEllipse(
            x_mm * SCALE_FACTOR - radius_px,
            y_mm * SCALE_FACTOR - radius_px,
            radius_px * 2, radius_px * 2,
            self._make_preview_pen(), self._make_preview_brush(),
        )
        it.setZValue(15)
        # Don't accept events — clicks pass through to the canvas.
        it.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self._preview_items.append(it)

    def _update_single_well_preview(
        self, mm: tuple[float, float],
    ) -> None:
        self._clear_preview()
        self._add_preview_circle(
            mm[0], mm[1], self.DEFAULT_WELL_DIAMETER_MM / 2.0)

    def _update_grid_preview(
        self, mm: tuple[float, float],
    ) -> None:
        self._clear_preview()
        x0, y0 = mm
        sx = self.DEFAULT_GRID_SPACING_MM
        sy = self.DEFAULT_GRID_SPACING_MM
        r = self.DEFAULT_WELL_DIAMETER_MM / 2.0
        for row in range(self.DEFAULT_GRID_ROWS):
            for col in range(self.DEFAULT_GRID_COLS):
                self._add_preview_circle(x0 + col * sx, y0 + row * sy, r)

    def _update_circle_preview(
        self, mm: tuple[float, float],
    ) -> None:
        self._clear_preview()
        cx, cy = mm
        # Hover preview (before pressing) uses the configured params; the
        # radius is a hint (the actual radius is set by dragging).
        params = self._circle_params
        if self._design is not None and self._design.outline.kind == "circle":
            radius = max((self._design.outline.radius or 0.0) * 0.6, 1.0)
        else:
            radius = self.DEFAULT_RING_RADIUS_MM
        r = float(params.get("diameter", self.DEFAULT_WELL_DIAMETER_MM)) / 2.0
        n = max(int(params.get("count", self.DEFAULT_RING_COUNT)), 1)
        # Outline ring (where the well centers sit).
        ring_pen = QPen(QColor(COLOR_CONSTRAINT), 1.0, Qt.PenStyle.DashLine)
        ring_pen.setColor(QColor(COLOR_SELECTED))
        ring = self._scene.addEllipse(
            (cx - radius) * SCALE_FACTOR,
            (cy - radius) * SCALE_FACTOR,
            radius * 2 * SCALE_FACTOR,
            radius * 2 * SCALE_FACTOR,
            ring_pen, QBrush(Qt.BrushStyle.NoBrush),
        )
        ring.setZValue(14)
        ring.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self._preview_items.append(ring)
        for i in range(n):
            angle = (2 * math.pi * i) / n
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            self._add_preview_circle(x, y, r)
