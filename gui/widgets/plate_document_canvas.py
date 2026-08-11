"""Canvas for the v7.12 :class:`~SupportClasses.PlateDocument`.

A ``QGraphicsView`` in millimetres × :data:`SCALE`, Y-DOWN (A1 top-left), which
is the document's storage frame. What the operator READS and TYPES goes through
``doc.to_display()`` / ``doc.from_display()`` so the chosen origin is honoured
without geometry ever moving.

Behaviours worth calling out:

* **Selection has one mutator** (``_set_selection``). The v7.4.x canvas mutated
  it inline in four places, and every feature added on top needed its own
  bookkeeping.
* **Pattern members select their PATTERN**; Alt+click selects the member. In
  the old canvas clicking a member opened a card whose X/Y spin boxes wrote the
  point directly, and the next rebuild silently reverted them. Selecting the
  feature means the controls on screen are the ones that actually drive the
  geometry.
* **Marquee** uses ``scene.items(rect, IntersectsItemShape)`` rather than
  ``setDragMode(RubberBandDrag)``, which would claim the left button view-wide
  and fight the tool state machine.
* **No in-scene editors, ever.** ``QGraphicsProxyWidget`` + ``QAbstractSpinBox``
  segfaults on key input in this app; clicking a dimension pill focuses its
  spin box in the properties panel instead.
* The drag ghost lives in :class:`~SupportClasses.PlateSolver.PlateSolver`, not
  on the document, and is torn down on tool change and Escape.
"""
from __future__ import annotations

import logging
import math
from enum import Enum, auto
from typing import Optional

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import (
    QBrush, QColor, QCursor, QKeyEvent, QKeySequence, QMouseEvent, QPainter,
    QPen, QShortcut, QWheelEvent,
)
from PySide6.QtWidgets import (
    QAbstractSpinBox, QApplication, QComboBox, QGraphicsEllipseItem,
    QGraphicsItem, QGraphicsLineItem, QGraphicsRectItem, QGraphicsScene,
    QGraphicsSimpleTextItem, QGraphicsView, QLineEdit, QPlainTextEdit,
    QSizePolicy, QTextEdit, QWidget,
)

from gui.scaling import s
from gui.widgets.plate_canvas_gizmos import (
    AnchorMarkerItem, DatumMarkerItem, RadiusHandleItem, RotationHandleItem,
    SeedHandleItem,
)
from SupportClasses.PlateDocument import (
    DATUMS, EntityId, GridPattern, Line, PatternFeature, PlateDocument, Point,
    Ref, RingPattern, Well,
)
from SupportClasses.PlateSolver import PlateSolver
from SupportClasses.SolveTypes import DOFStatus, SolveReport

logger = logging.getLogger(__name__)

SCALE = 6.0                      # scene units per millimetre

C_BG = "#1e1e2e"
C_PLATE = "#181825"
C_EDGE = "#585b70"
C_GRID_MINOR = "#26263a"
C_GRID_MAJOR = "#313244"
C_WELL = "#89b4fa"
C_WELL_PATTERN = "#f9e2af"
C_SEL = "#cba6f7"
C_HOVER = "#89dceb"
C_TEXT = "#cdd6f4"
C_DIM = "#94e2d5"
C_CONFLICT = "#f38ba8"
C_ROSETTE = "#94e2d5"

GRID_MINOR_MM = 1.0
GRID_MAJOR_MM = 10.0
_GRID_MIN_DEVICE_PX = 4.0
_ZOOM_MIN, _ZOOM_MAX = 0.05, 40.0
_HIT_PX = 7
_DATUM_PX = 9          # grab band for plate edges / centre-lines, in DEVICE px


class Tool(Enum):
    SELECT = auto()
    WELL = auto()
    RING = auto()
    GRID = auto()
    LINE = auto()
    DIMENSION = auto()


def _text_focused() -> bool:
    """True when a text-entry widget holds focus.

    Tool letters live on this widget's ``keyPressEvent`` rather than on
    window-scoped ``QShortcut``s precisely so they cannot reach a sibling
    editor; this is the belt-and-braces second check.
    """
    w = QApplication.focusWidget()
    if w is None:
        return False
    if isinstance(w, (QLineEdit, QAbstractSpinBox, QTextEdit, QPlainTextEdit)):
        return True
    return isinstance(w, QComboBox) and w.isEditable()


# ═══════════════════════════════════════════════════════════════════

class _WellItem(QGraphicsEllipseItem):
    def __init__(self, ref: Ref, x, y, dia, derived: bool):
        r = dia * SCALE / 2.0
        super().__init__(-r, -r, 2 * r, 2 * r)
        self.setPos(x * SCALE, y * SCALE)
        self.ref = ref
        self.derived = derived
        self.selected = False
        self.conflicted = False
        self.setAcceptHoverEvents(True)
        self.setZValue(10)
        self._hover = False
        self._restyle()

    def hoverEnterEvent(self, ev):
        self._hover = True
        self._restyle()
        super().hoverEnterEvent(ev)

    def hoverLeaveEvent(self, ev):
        self._hover = False
        self._restyle()
        super().hoverLeaveEvent(ev)

    def _restyle(self):
        base = QColor(C_WELL_PATTERN if self.derived else C_WELL)
        fill = QColor(base)
        fill.setAlpha(70 if self.derived else 90)
        self.setBrush(QBrush(fill))
        if self.conflicted:
            pen = QPen(QColor(C_CONFLICT), 2.4)
        elif self.selected:
            pen = QPen(QColor(C_SEL), 2.4)
        elif self._hover:
            pen = QPen(QColor(C_HOVER), 1.8)
        else:
            pen = QPen(base, 1.2)
        # Dashed = driven by a feature, not directly editable.
        if self.derived and not self.selected:
            pen.setStyle(Qt.DashLine)
        pen.setCosmetic(True)
        self.setPen(pen)


# ═══════════════════════════════════════════════════════════════════

class PlateDocumentCanvas(QGraphicsView):
    selection_changed = Signal(list)      # list[Ref]
    document_changed = Signal()
    solve_reported = Signal(object)       # SolveReport
    hover_moved = Signal(float, float)    # DISPLAY mm
    tool_changed = Signal(object)
    status_message = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._doc: Optional[PlateDocument] = None
        self._solver: Optional[PlateSolver] = None
        self._tool = Tool.SELECT
        self._selection: list[Ref] = []
        self._primary: Optional[Ref] = None
        self._report: Optional[SolveReport] = None
        # v7.9.1: False = builder behaviour (a click on a pattern member selects
        # the PATTERN; Alt reaches the member). True = layout behaviour, the
        # inverse. See set_member_pick_default and _press_select.
        self._member_pick_default = False
        # v7.9.1: rosette id → document, so a seated rosette can be drawn as
        # ITSELF. None = no resolver, and placements fall back to the generic
        # mark. Cache is per-rebuild (see _rosette_subwells).
        self._rosette_loader = None
        self._rosette_cache: dict = {}

        self._undo: list[dict] = []
        self._redo: list[dict] = []
        self._undo_cap = 50

        self._snap_mm = 0.0
        self._dragging: Optional[Ref] = None
        self._drag_pending: Optional[tuple[float, float]] = None
        self._gizmo_drag: Optional[tuple[str, EntityId]] = None
        self._marquee_from: Optional[QPointF] = None
        self._marquee_item: Optional[QGraphicsRectItem] = None
        self._pending_pt: Optional[tuple[float, float]] = None
        self._dim_pick: list[object] = []
        self._dim_mode = "center"
        self._preview: list = []           # transient hover/rubber-band items
        self._wheel_zoom = True

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHints(QPainter.Antialiasing
                            | QPainter.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QBrush(QColor(C_BG)))
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumSize(s(420), s(320))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._solve_timer = QTimer(self)
        self._solve_timer.setSingleShot(True)
        self._solve_timer.setInterval(33)          # ~30 fps ghost solves
        self._solve_timer.timeout.connect(self._flush_drag)

        for seq, slot in (("Esc", self.cancel_tool),
                          ("F", self.fit_view),
                          ("Delete", self.delete_selection),
                          ("Backspace", self.delete_selection)):
            sc = QShortcut(QKeySequence(seq), self, activated=slot)
            sc.setContext(Qt.WidgetWithChildrenShortcut)

    # ── Document ──────────────────────────────────────────────────

    @property
    def document(self) -> Optional[PlateDocument]:
        return self._doc

    @property
    def solver(self) -> Optional[PlateSolver]:
        return self._solver

    def set_document(self, doc: Optional[PlateDocument]) -> None:
        self._doc = doc
        self._solver = PlateSolver(doc) if doc is not None else None
        self._selection, self._primary = [], None
        self._undo.clear()
        self._redo.clear()
        self.rebuild()
        QTimer.singleShot(0, self.fit_view)
        self.selection_changed.emit([])

    # ── Undo / redo ───────────────────────────────────────────────

    def snapshot(self) -> None:
        if self._doc is None:
            return
        self._undo.append(self._doc.to_dict())
        if len(self._undo) > self._undo_cap:
            self._undo.pop(0)
        self._redo.clear()

    def can_undo(self) -> bool:
        return bool(self._undo)

    def can_redo(self) -> bool:
        return bool(self._redo)

    def undo(self) -> None:
        if self._undo and self._doc is not None:
            self._redo.append(self._doc.to_dict())
            self._restore(self._undo.pop())

    def redo(self) -> None:
        if self._redo and self._doc is not None:
            self._undo.append(self._doc.to_dict())
            self._restore(self._redo.pop())

    def _restore(self, blob: dict) -> None:
        fresh = PlateDocument.from_dict(blob)
        # Field-swap onto the live instance so external references (the panel,
        # the workspace) stay valid.
        d = self._doc
        d.meta, d.boundary, d.defaults = fresh.meta, fresh.boundary, fresh.defaults
        d.entities, d.constraints = fresh.entities, fresh.constraints
        d.units = fresh.units
        d._next_entity_id = fresh._next_entity_id
        d._next_constraint_id = fresh._next_constraint_id
        live = {w.key for w in d.evaluate()}
        self._set_selection([r for r in self._selection if r in live],
                            emit=True)
        self.rebuild()
        self.document_changed.emit()

    # ── Tools ─────────────────────────────────────────────────────

    def tool(self) -> Tool:
        return self._tool

    def set_tool(self, tool: Tool) -> None:
        self._end_drag()
        self._tool = tool
        self._pending_pt = None
        self._dim_pick.clear()
        self._clear_preview()
        if tool == Tool.SELECT:
            self.setCursor(QCursor(Qt.ArrowCursor))
        elif tool == Tool.DIMENSION:
            self.setCursor(QCursor(Qt.PointingHandCursor))
        else:
            self.setCursor(QCursor(Qt.CrossCursor))
        self.tool_changed.emit(tool)
        self.status_message.emit(self._tool_hint())

    def _tool_hint(self) -> str:
        return {
            Tool.SELECT: "Select — drag a well or an anchor; "
                         "Alt+click a pattern member.",
            Tool.WELL: "Click to place a single well.",
            Tool.RING: "Click the ring centre, then drag out its diameter.",
            Tool.GRID: "Click the seed — the first well. "
                       "Rows, columns and spacing are in its settings.",
            Tool.LINE: "Click the start, then the end. "
                       "Esc cancels, right-click cancels.",
            Tool.DIMENSION: "Pick two of: a well · a ring centre or grid seed "
                            "(the ✛ marker) · a plate edge · a centre-line. "
                            "Esc cancels.",
        }.get(self._tool, "")

    def cancel_tool(self) -> None:
        """Escape backs out one step at a time.

        Cancelling a half-finished pick used to drop you all the way to Select,
        so mis-clicking the first of a dimension's two references cost you the
        tool as well. Now the first Escape abandons the gesture and leaves you
        armed; only a second one leaves the tool.
        """
        if self._pending_pt is not None or self._dim_pick:
            self._pending_pt = None
            self._dim_pick.clear()
            self._clear_preview()
            self.status_message.emit("Cancelled — " + self._tool_hint())
            return
        if self._tool != Tool.SELECT:
            self.set_tool(Tool.SELECT)
            return
        self._set_selection([])

    def set_snap_mm(self, mm: float) -> None:
        self._snap_mm = float(mm or 0.0)

    def set_dim_mode(self, mode: str) -> None:
        self._dim_mode = mode if mode in ("center", "edge", "far") else "center"

    def set_wheel_zoom(self, on: bool) -> None:
        self._wheel_zoom = bool(on)

    # ── Selection (single mutator) ────────────────────────────────

    def _set_selection(self, refs, emit: bool = True) -> None:
        self._selection = list(dict.fromkeys(refs))
        self._primary = self._selection[0] if len(self._selection) == 1 else None
        for it in self._scene.items():
            if isinstance(it, _WellItem):
                want = it.ref in self._selection or (
                    it.ref[1] and (it.ref[0], "") in self._selection)
                if it.selected != want:
                    it.selected = want
                    it._restyle()
        self._refresh_gizmos()
        if emit:
            self.selection_changed.emit(list(self._selection))

    def set_rosette_loader(self, loader) -> None:
        """Supply ``rosette_id -> PlateDocument`` so seated rosettes draw real.

        v7.9.1. Without it every placement rendered as the same generic six-dot
        ring, so the operator could not tell a 3-channel insert from a 6-channel
        one, nor see the rotation they had set. Same signature as the
        ``RosetteLoader`` ``compile()`` takes — usually ``store.get``.
        """
        self._rosette_loader = loader
        self._rosette_cache = {}
        if self._doc is not None:
            self.rebuild()

    def set_member_pick_default(self, on: bool) -> None:
        """Make a plain click select a pattern MEMBER instead of the pattern.

        v7.9.1, for the rosette Layout tab. The builder's rule — a member
        selects its feature, Alt reaches the member — is right when you are
        editing pattern PARAMETERS, but wrong when a click means "seat a rosette
        in this well": a standard plate is a single `GridPattern`, so every
        click yielded the pattern, which is not a well. Alt still reaches the
        pattern here, so both targets stay available in both modes.
        """
        self._member_pick_default = bool(on)

    def selection(self) -> list[Ref]:
        return list(self._selection)

    def primary(self) -> Optional[Ref]:
        return self._primary

    def select_refs(self, refs) -> None:
        self._set_selection(list(refs))

    def select_all(self) -> None:
        if self._doc:
            self._set_selection([w.key for w in self._doc.evaluate()])

    def selected_features(self) -> list[PatternFeature]:
        out = []
        for eid, _m in self._selection:
            ent = self._doc.entities.get(eid) if self._doc else None
            if isinstance(ent, PatternFeature) and ent not in out:
                out.append(ent)
        return out

    def delete_selection(self) -> None:
        if not self._doc or not self._selection:
            return
        self.snapshot()
        for eid in {r[0] for r in self._selection}:
            self._doc.remove_entity(eid)
        self._set_selection([], emit=True)
        self._after_edit()

    # ── Scene build ───────────────────────────────────────────────

    def rebuild(self) -> None:
        self._scene.clear()
        # v7.9.1: resolve each rosette at most once per rebuild, but re-resolve
        # on the NEXT one, so editing a rosette in the builder and coming back
        # shows the new geometry rather than a stale cached copy.
        self._rosette_cache = {}
        self._marquee_item = None
        self._preview.clear()      # scene.clear() already destroyed the items
        if self._doc is None:
            return
        b = self._doc.boundary
        x0, y0, x1, y1 = b.extent_a1()

        if b.is_circle():
            outline = QGraphicsEllipseItem(x0 * SCALE, y0 * SCALE,
                                           (x1 - x0) * SCALE,
                                           (y1 - y0) * SCALE)
        else:
            outline = QGraphicsRectItem(x0 * SCALE, y0 * SCALE,
                                        (x1 - x0) * SCALE, (y1 - y0) * SCALE)
        pen = QPen(QColor(C_EDGE), 1.6)
        pen.setCosmetic(True)
        outline.setPen(pen)
        outline.setBrush(QBrush(QColor(C_PLATE)))
        outline.setZValue(-30)
        self._scene.addItem(outline)
        self._outline = outline

        ox, oy = b.origin_offset()
        datum = DatumMarkerItem(self._origin_label())
        datum.setPos(ox * SCALE, oy * SCALE)
        self._scene.addItem(datum)

        for ent in self._doc.entities.values():
            if isinstance(ent, Line):
                p1 = self._doc.entities.get(ent.p1)
                p2 = self._doc.entities.get(ent.p2)
                if isinstance(p1, Point) and isinstance(p2, Point):
                    li = QGraphicsLineItem(p1.x * SCALE, p1.y * SCALE,
                                           p2.x * SCALE, p2.y * SCALE)
                    lp = QPen(QColor(C_DIM if ent.construction else C_TEXT),
                              1.2)
                    lp.setCosmetic(True)
                    if ent.construction:
                        lp.setStyle(Qt.DashLine)
                    li.setPen(lp)
                    li.setZValue(2)
                    self._scene.addItem(li)

        conflicts = set(self._report.conflicts) if self._report else set()
        conflicted_refs: set[Ref] = set()
        for c in self._doc.constraints:
            if c.id in conflicts:
                conflicted_refs.update(c.refs)

        for w in self._doc.evaluate():
            item = _WellItem(w.key, w.x, w.y, w.diameter_mm,
                             derived=bool(w.key[1]))
            item.selected = (w.key in self._selection
                             or (w.key[1] and (w.key[0], "") in self._selection))
            item.conflicted = w.key in conflicted_refs
            item._restyle()
            self._scene.addItem(item)
            if w.rosette is not None:
                self._draw_rosette_badge(w)
            self._draw_label(w)

        self._draw_anchors()
        self._draw_dimensions()
        self._refresh_gizmos()

    def _draw_anchors(self) -> None:
        """One always-visible, always-hittable marker per pattern anchor.

        Without this a ring whose ``center_well`` is off has nothing on screen
        at its reference point, so requirement 2's "dimension everything from
        that point" was unreachable — the only clickable things were the ring
        MEMBERS.
        """
        for ent in self._doc.entities.values():
            if not isinstance(ent, PatternFeature):
                continue
            anchor = self._doc.entities.get(ent.anchor)
            if not isinstance(anchor, Point):
                continue
            kind = "ring centre" if isinstance(ent, RingPattern) else "grid seed"
            m = AnchorMarkerItem((ent.id, ""), kind)
            m.setPos(anchor.x * SCALE, anchor.y * SCALE)
            self._scene.addItem(m)

    def _origin_label(self) -> str:
        return str(getattr(self._doc.boundary.origin_ref, "value",
                           self._doc.boundary.origin_ref)).replace("_", " ")

    def _draw_label(self, w) -> None:
        r_px = w.diameter_mm * SCALE / 2.0
        t = QGraphicsSimpleTextItem(w.name)
        t.setBrush(QBrush(QColor(C_TEXT)))
        t.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        t.setZValue(12)
        t.setAcceptedMouseButtons(Qt.NoButton)
        br = t.boundingRect()
        t.setPos(w.x * SCALE - br.width() / 2.0,
                 w.y * SCALE - br.height() / 2.0)
        # Hide when the label would spill outside its own well at this zoom.
        zoom = self.transform().m11()
        t.setVisible(br.width() < 2 * r_px * zoom)
        self._scene.addItem(t)

    def _draw_rosette_badge(self, w) -> None:
        """Draw a seated rosette AS ITSELF — its own sub-wells, at their real
        size, positions and rotation.

        v7.9.1. This used to paint a hard-coded ring of six dots for every
        rosette, so a 3-channel insert and a 6-channel one were indistinguishable
        on the plate and the rotation the operator had set was invisible. The
        geometry comes from `rosette_subwell_offsets`, the SAME helper
        ``PlateDocument.compile()`` uses, so the picture cannot drift from what
        gets printed.

        The generic six-dot mark is kept for the one case that has no geometry
        to show: a placement whose rosette cannot be resolved. That must stay
        visible — compile() deliberately keeps such a well rather than dropping
        it, and `validate()` reports it by name.
        """
        subs = self._rosette_subwells(getattr(w.rosette, "rosette_id", ""))
        if not subs:
            self._draw_unresolved_rosette_mark(w)
            return
        from SupportClasses.PlateDocument import rosette_subwell_offsets
        offsets = rosette_subwell_offsets(
            subs, getattr(w.rosette, "rotation_deg", 0.0))
        for sub, (dx, dy) in zip(subs, offsets):
            sr = max(float(sub.diameter_mm), 0.0) * SCALE / 2.0
            if sr <= 0:
                continue
            d = QGraphicsEllipseItem(-sr, -sr, 2 * sr, 2 * sr)
            d.setPos((w.x + dx) * SCALE, (w.y + dy) * SCALE)
            fill = QColor(C_ROSETTE)
            fill.setAlpha(110)
            d.setBrush(QBrush(fill))
            pen = QPen(QColor(C_ROSETTE), 1.4)
            pen.setCosmetic(True)
            d.setPen(pen)
            d.setZValue(14)
            d.setAcceptedMouseButtons(Qt.NoButton)
            self._scene.addItem(d)

    def _draw_unresolved_rosette_mark(self, w) -> None:
        """The legacy six-dot badge — now ONLY for an unresolvable rosette."""
        r = w.diameter_mm * SCALE / 2.0
        for i in range(6):
            a = math.radians(60 * i)
            d = QGraphicsEllipseItem(-r * 0.09, -r * 0.09,
                                     r * 0.18, r * 0.18)
            d.setPos(w.x * SCALE + r * 0.5 * math.sin(a),
                     w.y * SCALE + r * 0.5 * math.cos(a))
            d.setBrush(QBrush(QColor(C_ROSETTE)))
            d.setPen(QPen(Qt.NoPen))
            d.setZValue(14)
            d.setAcceptedMouseButtons(Qt.NoButton)
            self._scene.addItem(d)

    def _rosette_subwells(self, rosette_id: str) -> list:
        """Sub-wells for *rosette_id*, memoised for the life of one rebuild.

        `rebuild` runs on every edit and a plate can seat the same rosette in
        many wells, so resolving per well would re-read the document each time.
        """
        if not rosette_id or self._rosette_loader is None:
            return []
        if rosette_id in self._rosette_cache:
            return self._rosette_cache[rosette_id]
        from SupportClasses.PlateDocument import PlateDocument
        try:
            doc = self._rosette_loader(rosette_id)
        except Exception:                              # pragma: no cover
            doc = None
        subs = PlateDocument.rosette_subwells(doc)
        self._rosette_cache[rosette_id] = subs
        return subs

    def _draw_dimensions(self) -> None:
        if self._doc is None:
            return
        conflicts = set(self._report.conflicts) if self._report else set()
        pos = {w.key: (w.x, w.y) for w in self._doc.evaluate()}

        def _xy(ref):
            """Position of any dimensionable ref — including a pattern anchor,
            which is not a well and so is absent from ``evaluate()``."""
            hit = pos.get(ref)
            if hit is not None:
                return hit
            base = self._solver._base_point(ref) if self._solver else None
            if not base:
                return None
            pt = self._doc.entities.get(base[0])
            return (pt.x + base[1], pt.y + base[2]) \
                if isinstance(pt, Point) else None

        for c in self._doc.constraints:
            if c.kind == "distance" and len(c.refs) >= 2:
                self._draw_distance_pill(c, _xy, conflicts)
                continue
            if c.kind != "distance_to_datum" or not c.refs:
                continue
            xy = _xy(c.refs[0])
            if xy is None:
                continue
            d = self._doc.boundary.datum_value(c.datum)
            if d is None:
                continue
            x, y = xy
            vertical = c.datum in ("edge_top", "edge_bottom", "axis_h")
            if vertical:
                a, b = QPointF(x * SCALE, y * SCALE), QPointF(x * SCALE, d * SCALE)
            else:
                a, b = QPointF(x * SCALE, y * SCALE), QPointF(d * SCALE, y * SCALE)
            colour = QColor(C_CONFLICT if c.id in conflicts else C_DIM)
            line = QGraphicsLineItem(a.x(), a.y(), b.x(), b.y())
            lp = QPen(colour, 1.0)
            lp.setCosmetic(True)
            lp.setStyle(Qt.DashLine)
            line.setPen(lp)
            line.setZValue(18)
            line.setAcceptedMouseButtons(Qt.NoButton)
            self._scene.addItem(line)

            pill = QGraphicsSimpleTextItem(f"{c.value:.2f}")
            pill.setBrush(QBrush(colour))
            pill.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            pill.setZValue(21)
            pill.setData(0, c.id)          # click → focus its panel spin box
            pill.setPos((a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0)
            pill.setToolTip(f"{c.datum.replace('_', ' ')} — click to edit")
            self._scene.addItem(pill)

    def _draw_distance_pill(self, c, xy_of, conflicts: set) -> None:
        """A well-to-well dimension. These carried no drawing at all before —
        you added one and the canvas looked unchanged."""
        a, b = xy_of(c.refs[0]), xy_of(c.refs[1])
        if a is None or b is None:
            return
        colour = QColor(C_CONFLICT if c.id in conflicts else C_DIM)
        line = QGraphicsLineItem(a[0] * SCALE, a[1] * SCALE,
                                 b[0] * SCALE, b[1] * SCALE)
        lp = QPen(colour, 1.0)
        lp.setCosmetic(True)
        lp.setStyle(Qt.DashLine)
        line.setPen(lp)
        line.setZValue(18)
        line.setAcceptedMouseButtons(Qt.NoButton)
        self._scene.addItem(line)

        pill = QGraphicsSimpleTextItem(f"{c.value:.2f}")
        pill.setBrush(QBrush(colour))
        pill.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        pill.setZValue(21)
        pill.setData(0, c.id)
        pill.setPos((a[0] + b[0]) / 2.0 * SCALE, (a[1] + b[1]) / 2.0 * SCALE)
        pill.setToolTip("Distance — click to edit")
        self._scene.addItem(pill)

    # ── Gizmos ────────────────────────────────────────────────────

    def _refresh_gizmos(self) -> None:
        for it in list(self._scene.items()):
            if isinstance(it, (RadiusHandleItem, RotationHandleItem,
                               SeedHandleItem)):
                self._scene.removeItem(it)
        if self._doc is None or len(self.selected_features()) != 1:
            return
        feat = self.selected_features()[0]
        anchor = self._doc.anchor_point(feat)
        if anchor is None:
            return

        seed = SeedHandleItem(feat.id)
        seed.setPos(anchor.x * SCALE, anchor.y * SCALE)
        self._scene.addItem(seed)

        if isinstance(feat, RingPattern):
            r = feat.ring_diameter_mm / 2.0
            th = math.radians(feat.start_angle_deg)
            hx = anchor.x + r * math.sin(th)
            hy = anchor.y + r * math.cos(th)
            rad = RadiusHandleItem(feat.id)
            rad.setPos(hx * SCALE, hy * SCALE)
            self._scene.addItem(rad)
            # The rotation handle sits OUTSIDE the ring so it can never land
            # on a well — the ambiguity the single old handle had.
            rot = RotationHandleItem(feat.id, "ring")
            rot.setPos((anchor.x + (r + 3.0) * math.sin(th)) * SCALE,
                       (anchor.y + (r + 3.0) * math.cos(th)) * SCALE)
            self._scene.addItem(rot)
        elif isinstance(feat, GridPattern):
            span = max(feat.cols - 1, 1) * feat.pitch_x_mm
            th = math.radians(feat.rotation_deg)
            hx = anchor.x + (span + 4.0) * math.cos(th)
            hy = anchor.y + (span + 4.0) * math.sin(th)
            rot = RotationHandleItem(feat.id, "grid")
            rot.setPos(hx * SCALE, hy * SCALE)
            self._scene.addItem(rot)

    # ── Painting ──────────────────────────────────────────────────

    def drawBackground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawBackground(painter, rect)
        if self._doc is None:
            return
        x0, y0, x1, y1 = self._doc.boundary.extent_a1()
        zoom = self.transform().m11()

        minor = GRID_MINOR_MM * SCALE
        if minor * zoom >= _GRID_MIN_DEVICE_PX:
            painter.setPen(QPen(QColor(C_GRID_MINOR), 0))
            self._grid_lines(painter, x0, y0, x1, y1, minor)
        painter.setPen(QPen(QColor(C_GRID_MAJOR), 0))
        self._grid_lines(painter, x0, y0, x1, y1, GRID_MAJOR_MM * SCALE)

    @staticmethod
    def _grid_lines(painter, x0, y0, x1, y1, step):
        left, top = x0 * SCALE, y0 * SCALE
        right, bottom = x1 * SCALE, y1 * SCALE
        x = left
        while x <= right:
            painter.drawLine(QPointF(x, top), QPointF(x, bottom))
            x += step
        y = top
        while y <= bottom:
            painter.drawLine(QPointF(left, y), QPointF(right, y))
            y += step

    # ── View ──────────────────────────────────────────────────────

    def fit_view(self) -> None:
        if self._doc is None:
            return
        x0, y0, x1, y1 = self._doc.boundary.extent_a1()
        pad = 6.0
        self.fitInView(QRectF((x0 - pad) * SCALE, (y0 - pad) * SCALE,
                              (x1 - x0 + 2 * pad) * SCALE,
                              (y1 - y0 + 2 * pad) * SCALE),
                       Qt.KeepAspectRatio)

    def wheelEvent(self, event: QWheelEvent) -> None:
        if not self._wheel_zoom:
            event.ignore()
            return
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        cur = self.transform().m11()
        tgt = max(_ZOOM_MIN, min(_ZOOM_MAX, cur * factor))
        if abs(tgt - cur) > 1e-9:
            self.scale(tgt / cur, tgt / cur)

    # ── Coordinates ───────────────────────────────────────────────

    def _scene_to_mm(self, p: QPointF) -> tuple[float, float]:
        x, y = p.x() / SCALE, p.y() / SCALE
        if self._snap_mm > 0:
            x = round(x / self._snap_mm) * self._snap_mm
            y = round(y / self._snap_mm) * self._snap_mm
        return (x, y)

    def _hit_well(self, scene_pt: QPointF) -> Optional[_WellItem]:
        for it in self._scene.items(scene_pt):
            if isinstance(it, _WellItem):
                return it
        return None

    def _hit_gizmo(self, scene_pt: QPointF):
        for it in self._scene.items(scene_pt):
            if isinstance(it, (RadiusHandleItem, RotationHandleItem,
                               SeedHandleItem)):
                return it
        return None

    def _hit_anchor(self, scene_pt: QPointF) -> Optional[AnchorMarkerItem]:
        for it in self._scene.items(scene_pt):
            if isinstance(it, AnchorMarkerItem):
                return it
        return None

    def _hit_pill(self, scene_pt: QPointF) -> Optional[int]:
        for it in self._scene.items(scene_pt):
            if isinstance(it, QGraphicsSimpleTextItem) and it.data(0):
                return int(it.data(0))
        return None

    # ── Live preview ──────────────────────────────────────────────
    #
    # Everything below is transient decoration: it never touches the document
    # and is rebuilt from scratch on each mouse move. It exists because both
    # of the operator's complaints reduce to "I cannot see what this click is
    # about to do" — a plate edge was an invisible band, and the line and ring
    # tools committed with no indication of what was being drawn.

    def _clear_preview(self) -> None:
        for it in self._preview:
            if it.scene() is self._scene:
                self._scene.removeItem(it)
        self._preview.clear()

    def _preview_line(self, x0, y0, x1, y1, colour: str,
                      dashed: bool = True, width: float = 1.8) -> None:
        it = QGraphicsLineItem(x0 * SCALE, y0 * SCALE, x1 * SCALE, y1 * SCALE)
        pen = QPen(QColor(colour), width)
        pen.setCosmetic(True)
        if dashed:
            pen.setStyle(Qt.DashLine)
        it.setPen(pen)
        it.setZValue(30)
        it.setAcceptedMouseButtons(Qt.NoButton)
        self._scene.addItem(it)
        self._preview.append(it)

    def _preview_marker(self, x, y, colour: str, r_mm: float = 0.0) -> None:
        r = (r_mm * SCALE) if r_mm > 0 else s(9)
        it = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        it.setPos(x * SCALE, y * SCALE)
        pen = QPen(QColor(colour), 2.2)
        pen.setCosmetic(True)
        it.setPen(pen)
        it.setBrush(QBrush(Qt.NoBrush))
        if r_mm <= 0:
            it.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        it.setZValue(30)
        it.setAcceptedMouseButtons(Qt.NoButton)
        self._scene.addItem(it)
        self._preview.append(it)

    def _highlight_target(self, target, colour: str) -> None:
        """Draw what a dimension pick refers to."""
        if target is None or self._doc is None:
            return
        if target[0] == "datum":
            d = self._doc.boundary.datum_value(target[1])
            if d is None:
                return
            x0, y0, x1, y1 = self._doc.boundary.extent_a1()
            pad = max(x1 - x0, y1 - y0) * 0.04
            if target[1] in ("edge_top", "edge_bottom", "axis_h"):
                self._preview_line(x0 - pad, d, x1 + pad, d, colour,
                                   dashed=False, width=2.6)
            else:
                self._preview_line(d, y0 - pad, d, y1 + pad, colour,
                                   dashed=False, width=2.6)
            return
        xy = {w.key: (w.x, w.y, w.diameter_mm)
              for w in self._doc.evaluate()}.get(target[1])
        if xy is not None:
            self._preview_marker(xy[0], xy[1], colour,
                                 r_mm=xy[2] / 2.0 + 0.45)
            return
        base = self._solver._base_point(target[1]) if self._solver else None
        if base:                       # a pattern anchor with no well on it
            pt = self._doc.entities.get(base[0])
            if isinstance(pt, Point):
                self._preview_marker(pt.x + base[1], pt.y + base[2], colour)

    def _update_preview(self, sp: QPointF, mm: tuple[float, float]) -> None:
        self._clear_preview()
        if self._doc is None or self._dragging is not None \
                or self._gizmo_drag is not None:
            return

        if self._tool == Tool.DIMENSION:
            for picked in self._dim_pick:
                self._highlight_target(picked, C_SEL)
            self._highlight_target(self._dim_target(sp), C_HOVER)
        elif self._tool == Tool.LINE and self._pending_pt is not None:
            self._preview_line(self._pending_pt[0], self._pending_pt[1],
                               mm[0], mm[1], C_HOVER)
            self._preview_marker(*self._pending_pt, C_SEL)
        elif self._tool == Tool.RING and self._pending_pt is not None:
            cx, cy = self._pending_pt
            r = math.hypot(mm[0] - cx, mm[1] - cy)
            self._preview_marker(cx, cy, C_SEL)
            if r > 0.05:
                self._preview_marker(cx, cy, C_HOVER, r_mm=r)
                self._preview_line(cx, cy, mm[0], mm[1], C_HOVER)

    def leaveEvent(self, event) -> None:
        self._clear_preview()
        super().leaveEvent(event)

    # ── Mouse ─────────────────────────────────────────────────────

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if self._doc is None:
            return super().mousePressEvent(event)
        if event.button() == Qt.MiddleButton:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            return super().mousePressEvent(event)
        if event.button() == Qt.RightButton:
            # CAD idiom: right-click backs out of the gesture in flight.
            if self._pending_pt is not None or self._dim_pick:
                self.cancel_tool()
                event.accept()
                return
            return super().mousePressEvent(event)
        if event.button() != Qt.LeftButton:
            return super().mousePressEvent(event)

        sp = self.mapToScene(event.pos())
        mm = self._scene_to_mm(sp)

        if self._tool == Tool.SELECT:
            giz = self._hit_gizmo(sp)
            if giz is not None:
                self.snapshot()
                self._gizmo_drag = (giz.kind, giz.owner_id)
                event.accept()
                return
            cid = self._hit_pill(sp)
            if cid is not None:
                self.select_constraint(cid)
                event.accept()
                return
            self._press_select(sp, event)
            event.accept()
            return

        if self._tool == Tool.WELL:
            self.snapshot()
            self._doc.add_well(mm[0], mm[1], self._next_well_name())
            self._after_edit()
        elif self._tool == Tool.RING:
            self._pending_pt = mm
        elif self._tool == Tool.GRID:
            self.snapshot()
            feat = self._doc.add_grid(mm[0], mm[1], rows=4, cols=6,
                                      pitch_x_mm=19.3, pitch_y_mm=19.3)
            self._after_edit()
            self._set_selection([(feat.id, "")])
        elif self._tool == Tool.LINE:
            if self._pending_pt is None:
                self._pending_pt = mm
            else:
                self.snapshot()
                p1 = self._doc.add_point(*self._pending_pt, construction=True)
                p2 = self._doc.add_point(*mm, construction=True)
                self._doc.add_entity(Line(p1=p1.id, p2=p2.id,
                                          construction=True))
                self._pending_pt = None
                self._after_edit()
        elif self._tool == Tool.DIMENSION:
            self._dimension_pick(sp)
        event.accept()

    def _press_select(self, sp: QPointF, event: QMouseEvent) -> None:
        additive = bool(event.modifiers()
                        & (Qt.ControlModifier | Qt.ShiftModifier))
        member_pick = bool(event.modifiers() & Qt.AltModifier)
        anchor = self._hit_anchor(sp)
        if anchor is not None:
            # The anchor marker is the only way to grab a ring whose centre
            # carries no well.
            self._set_selection(list(self._selection) + [anchor.ref]
                                if additive else [anchor.ref])
            self.snapshot()
            self._dragging = anchor.ref
            if self._solver and self._solver._base_point(anchor.ref):
                self._solver.begin_drag(anchor.ref, self._scene_to_mm(sp))
            return
        item = self._hit_well(sp)
        if item is None:
            self._marquee_from = sp
            if not additive:
                self._set_selection([])
            return
        # A pattern member selects its FEATURE unless Alt is held — because in
        # the BUILDER what you edit is the pattern's parameters.
        # v7.9.1: the LAYOUT tab inverts this (`set_member_pick_default`). There
        # a click means "seat a rosette in THIS well", and since a 24-well plate
        # is one GridPattern, the builder rule handed every click the pattern —
        # which is not a well at all.
        if self._member_pick_default:
            ref = (item.ref[0], "") if member_pick else item.ref
        else:
            ref = item.ref if (member_pick or not item.ref[1]) \
                else (item.ref[0], "")
        if additive:
            sel = list(self._selection)
            sel.remove(ref) if ref in sel else sel.append(ref)
            self._set_selection(sel)
        else:
            self._set_selection([ref])
        # Only a standalone well or a pattern anchor is draggable; generated
        # members are not (the operator's choice — the settings and the
        # handles are the edit paths, so params can never drift from geometry).
        if not item.ref[1]:
            self.snapshot()
            self._dragging = ref
            base = self._solver._base_point(ref) if self._solver else None
            if base:
                self._solver.begin_drag(ref, self._scene_to_mm(sp))

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._doc is None:
            return super().mouseMoveEvent(event)
        sp = self.mapToScene(event.pos())
        mm = self._scene_to_mm(sp)
        dx, dy = self._doc.to_display(*mm)
        self.hover_moved.emit(dx, dy)
        self._update_preview(sp, mm)

        if self._gizmo_drag is not None:
            self._apply_gizmo(self._gizmo_drag, mm, event.modifiers())
            event.accept()
            return
        if self._dragging is not None:
            self._drag_pending = mm
            if not self._solve_timer.isActive():
                self._solve_timer.start()
            event.accept()
            return
        if self._marquee_from is not None:
            self._update_marquee(sp)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MiddleButton:
            self.setDragMode(QGraphicsView.NoDrag)
            return super().mouseReleaseEvent(event)

        if self._gizmo_drag is not None:
            self._gizmo_drag = None
            self._after_edit()
            event.accept()
            return
        if self._dragging is not None:
            self._flush_drag()
            self._end_drag()
            self._after_edit()
            event.accept()
            return
        if self._marquee_from is not None:
            self._commit_marquee(self.mapToScene(event.pos()), event)
            event.accept()
            return
        if self._tool == Tool.RING and self._pending_pt is not None:
            centre = self._pending_pt
            mm = self._scene_to_mm(self.mapToScene(event.pos()))
            dia = 2.0 * math.hypot(mm[0] - centre[0], mm[1] - centre[1])
            self._pending_pt = None
            if dia > 0.4:
                self.snapshot()
                feat = self._doc.add_ring(centre[0], centre[1], count=6,
                                          ring_diameter_mm=dia)
                self._after_edit()
                self._set_selection([(feat.id, "")])
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _end_drag(self) -> None:
        if self._solver is not None and self._solver.dragging:
            self._solver.end_drag()
        self._dragging = None
        self._drag_pending = None

    def _flush_drag(self) -> None:
        if self._dragging is None or self._drag_pending is None:
            return
        target, self._drag_pending = self._drag_pending, None
        if self._solver is not None and self._solver.dragging:
            self._report = self._solver.update_drag(target)
            self.solve_reported.emit(self._report)
        else:
            base = self._solver._base_point(self._dragging) if self._solver \
                else None
            if base:
                pt = self._doc.entities[base[0]]
                pt.x, pt.y = target[0] - base[1], target[1] - base[2]
        self.rebuild()

    # ── Gizmo drags ───────────────────────────────────────────────

    def _apply_gizmo(self, giz, mm, modifiers) -> None:
        kind, eid = giz
        feat = self._doc.entities.get(eid)
        anchor = self._doc.anchor_point(feat) if isinstance(
            feat, PatternFeature) else None
        if anchor is None:
            return
        shift = bool(Qt.KeyboardModifier(modifiers)
                     & Qt.KeyboardModifier.ShiftModifier)

        if kind == "seed":
            anchor.x, anchor.y = mm
        elif kind == "radius" and isinstance(feat, RingPattern):
            r = math.hypot(mm[0] - anchor.x, mm[1] - anchor.y)
            if shift:
                r = round(r / 0.25) * 0.25
            feat.ring_diameter_mm = max(2.0 * r, 0.1)   # ANGLE UNTOUCHED
        elif kind == "rotate":
            if isinstance(feat, RingPattern):
                ang = math.degrees(math.atan2(mm[0] - anchor.x,
                                              mm[1] - anchor.y))
                if shift:
                    ang = round(ang / 15.0) * 15.0
                feat.start_angle_deg = ang              # SIZE UNTOUCHED
            elif isinstance(feat, GridPattern):
                ang = math.degrees(math.atan2(mm[1] - anchor.y,
                                              mm[0] - anchor.x))
                if shift:
                    ang = round(ang / 15.0) * 15.0
                feat.rotation_deg = ang
        self.rebuild()
        self.document_changed.emit()

    # ── Marquee ───────────────────────────────────────────────────

    def _update_marquee(self, sp: QPointF) -> None:
        rect = QRectF(self._marquee_from, sp).normalized()
        if self._marquee_item is None:
            self._marquee_item = QGraphicsRectItem()
            pen = QPen(QColor(C_SEL), 1.0)
            pen.setCosmetic(True)
            pen.setStyle(Qt.DashLine)
            self._marquee_item.setPen(pen)
            fill = QColor(C_SEL)
            fill.setAlpha(40)
            self._marquee_item.setBrush(QBrush(fill))
            self._marquee_item.setZValue(40)
            self._scene.addItem(self._marquee_item)
        self._marquee_item.setRect(rect)

    def _commit_marquee(self, sp: QPointF, event) -> None:
        rect = QRectF(self._marquee_from, sp).normalized()
        self._marquee_from = None
        if self._marquee_item is not None:
            self._scene.removeItem(self._marquee_item)
            self._marquee_item = None
        # A tiny drag is a click, not a marquee.
        if rect.width() < SCALE * 0.2 and rect.height() < SCALE * 0.2:
            return
        additive = bool(event.modifiers()
                        & (Qt.ControlModifier | Qt.ShiftModifier))
        hits: list[Ref] = []
        for it in self._scene.items(rect, Qt.IntersectsItemShape):
            if isinstance(it, _WellItem):
                ref = (it.ref[0], "") if it.ref[1] else it.ref
                if ref not in hits:
                    hits.append(ref)
        self._set_selection((self._selection + hits) if additive else hits)

    # ── Dimensions ────────────────────────────────────────────────

    def _dimension_pick(self, sp: QPointF) -> None:
        target = self._dim_target(sp)
        if target is None:
            self.status_message.emit(
                "Nothing to dimension there — hover until something "
                "highlights, then click. Pickable: wells, the ✛ ring-centre / "
                "grid-seed markers, the four plate edges and the centre-lines.")
            return
        if target in self._dim_pick:
            self.status_message.emit(
                f"{self._describe(target)} is already the first reference — "
                "pick a different one.")
            return
        self._dim_pick.append(target)
        if len(self._dim_pick) < 2:
            self.status_message.emit(
                f"From {self._describe(target)} → now pick what to measure to.")
            self._update_preview(sp, self._scene_to_mm(sp))
            return
        a, b = self._dim_pick[0], self._dim_pick[1]
        self._dim_pick.clear()
        self._create_dimension(a, b)

    def _dim_target(self, sp: QPointF):
        # A pattern anchor wins over a coincident seed well: it is the point
        # the operator means by "the reference point of this grid", and it
        # keeps its identity when the A1-corner direction flips (the seed
        # member key does not).
        anchor = self._hit_anchor(sp)
        if anchor is not None:
            return ("ref", anchor.ref)
        item = self._hit_well(sp)
        if item is not None:
            return ("ref", item.ref)
        d = self._nearest_datum(sp)
        return ("datum", d) if d else None

    def _datum_tol_mm(self) -> float:
        """A screen-constant grab band.

        The old ``max(tol, 1.5)`` mm floor meant that zoomed out on a 128 mm
        plate the four edges were a couple of device pixels wide — you were
        clicking blind at an invisible target, which is most of why this tool
        felt unusable.
        """
        return _DATUM_PX / max(self.transform().m11() * SCALE, 1e-6)

    def _nearest_datum(self, sp: QPointF) -> str:
        x, y = sp.x() / SCALE, sp.y() / SCALE
        cands = self._datum_distances(x, y)
        if not cands:
            return ""
        best = min(cands, key=cands.get)
        return best if cands[best] <= self._datum_tol_mm() else ""

    def _datum_distances(self, x: float, y: float) -> dict:
        x0, y0, x1, y1 = self._doc.boundary.extent_a1()
        return {
            "edge_left": abs(x - x0), "edge_right": abs(x - x1),
            "edge_top": abs(y - y0), "edge_bottom": abs(y - y1),
            "axis_v": abs(x - (x0 + x1) / 2.0),
            "axis_h": abs(y - (y0 + y1) / 2.0),
        }

    def _describe(self, target) -> str:
        if target[0] != "ref":
            return target[1].replace("_", " ")
        eid, member = target[1]
        ent = self._doc.entities.get(eid) if self._doc else None
        if isinstance(ent, PatternFeature) and not member:
            return ("ring centre" if isinstance(ent, RingPattern)
                    else "grid seed")
        if member:
            return f"member {member}"
        return getattr(ent, "name", None) or "well"

    def _create_dimension(self, a, b) -> None:
        mode = "edge" if self._dim_mode in ("edge", "far") else "center"
        if a[0] == "datum" and b[0] == "ref":
            a, b = b, a
        self.snapshot()
        if a[0] == "ref" and b[0] == "datum":
            val = self._solver.seed_value("distance_to_datum", [a[1]],
                                          datum=b[1], mode=mode)
            self._doc.add_constraint("distance_to_datum", [a[1]],
                                     value=val or 0.0, datum=b[1], mode=mode)
            self.status_message.emit(
                f"Dimension: {self._describe(a)} → "
                f"{b[1].replace('_', ' ')} = {val:.2f} mm. "
                "Edit it in the Constraints card.")
        elif a[0] == "ref" and b[0] == "ref":
            val = self._solver.seed_value("distance", [a[1], b[1]], mode=mode)
            self._doc.add_constraint("distance", [a[1], b[1]],
                                     value=val or 0.0, mode=mode)
            self.status_message.emit(
                f"Dimension: {self._describe(a)} → {self._describe(b)} "
                f"= {val:.2f} mm")
        else:
            self.status_message.emit(
                "Two plate edges cannot be dimensioned to each other — one "
                "reference must be a well or a pattern anchor.")
            return
        self._after_edit()

    def select_constraint(self, cid: int) -> None:
        c = self._doc.constraint_by_id(cid) if self._doc else None
        if c:
            self._set_selection(list(c.refs))

    # ── Constraint helpers used by the panel ──────────────────────

    def can_add_constraint(self, kind: str) -> bool:
        return self._resolve_constraint(kind, dry_run=True)[0]

    def add_constraint_for_selection(self, kind: str) -> tuple[bool, str]:
        ok, msg = self._resolve_constraint(kind, dry_run=False)
        if ok:
            self._after_edit()
        return ok, msg

    def _resolve_constraint(self, kind: str, dry_run: bool
                            ) -> tuple[bool, str]:
        """Single funnel for both button enablement and creation."""
        if self._doc is None or self._solver is None:
            return (False, "No document.")
        sel = list(self._selection)
        two = len(sel) >= 2

        if kind in ("horizontal", "vertical", "coincident", "concentric"):
            if not two:
                return (False, "Select two wells.")
            if not dry_run:
                self.snapshot()
                self._doc.add_constraint(kind, sel[:2])
            return (True, f"{kind} added.")
        if kind == "distance":
            if not two:
                return (False, "Select two wells.")
            if not dry_run:
                self.snapshot()
                val = self._solver.seed_value("distance", sel[:2])
                self._doc.add_constraint("distance", sel[:2], value=val or 0.0)
            return (True, "Distance added.")
        if kind == "equal_radius":
            if not two:
                return (False, "Select two wells.")
            if not dry_run:
                self.snapshot()
                self._doc.add_constraint("equal_radius", sel[:2])
            return (True, "Equal diameter added.")
        if kind == "fix":
            if not sel:
                return (False, "Select something to lock.")
            if not dry_run:
                self.snapshot()
                self._doc.add_constraint("fix", sel)
            return (True, "Locked.")
        return (False, "Select two wells.")

    # ── Solve / edit funnel ───────────────────────────────────────

    def _after_edit(self) -> None:
        self.solve()
        self.rebuild()
        self.document_changed.emit()

    def solve(self) -> Optional[SolveReport]:
        if self._solver is None:
            return None
        self._report = self._solver.solve()
        self.solve_reported.emit(self._report)
        return self._report

    def last_report(self) -> Optional[SolveReport]:
        return self._report

    def _next_well_name(self) -> str:
        taken = {w.name for w in self._doc.evaluate()}
        i = 1
        while f"W{i}" in taken:
            i += 1
        return f"W{i}"

    # ── Keyboard ──────────────────────────────────────────────────

    _TOOL_KEYS = {
        Qt.Key_S: Tool.SELECT, Qt.Key_W: Tool.WELL, Qt.Key_C: Tool.RING,
        Qt.Key_G: Tool.GRID, Qt.Key_L: Tool.LINE, Qt.Key_D: Tool.DIMENSION,
    }

    def keyPressEvent(self, event: QKeyEvent) -> None:
        mods = event.modifiers()
        if mods & (Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier):
            if event.key() == Qt.Key_A and mods & Qt.ControlModifier:
                self.select_all()
                event.accept()
                return
            return super().keyPressEvent(event)
        if _text_focused():
            return super().keyPressEvent(event)
        tool = self._TOOL_KEYS.get(event.key())
        if tool is not None:
            self.set_tool(tool)
            event.accept()
            return
        super().keyPressEvent(event)
