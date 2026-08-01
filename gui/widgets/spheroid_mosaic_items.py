"""
spheroid_mosaic_items.py — editable spheroid circles on a mosaic QGraphicsView.

v7.8: overlays detected spheroids on the survey tab's mosaic as circles the
operator can drag to recentre and resize by either gesture:

  * grab the RADIUS HANDLE and drag — one-radius sizing, fast; or
  * click 3+ RIM POINTS and let a circle be least-squares fitted — more accurate
    on an irregular or fuzzy spheroid.

Conventions taken from the two proven precedents in this codebase, deliberately:

* Each circle's rect is built in ABSOLUTE scene coordinates with ``pos()`` left
  at (0, 0), so its centre is ``rect().center() + pos()`` — the
  ``mosaic_well_mapping_dialog._place_marker`` / ``_marker_center`` convention.
  Keeping it identical means our back-projection is the same arithmetic that
  dialog's ``_on_confirm`` has been driving the stage with.
* The radius handle is NOT ``ItemIsMovable``; the VIEW drives it, exactly like
  ``plate_designer_canvas.RadiusHandleItem`` — hit-test on press, update on move,
  commit on release.
* The label and handle are CHILDREN of the circle (``setParentItem``), so a drag
  carries them. The well-mapping dialog makes them siblings and its labels lag
  behind during a drag; that is an existing bug there, not a convention to copy.
* No in-scene widgets. ``plate_designer_canvas`` records that in-scene
  ``QGraphicsProxyWidget`` spin boxes crashed, so every numeric edit lives in a
  plain side panel or modal.

Scene coordinates are mosaic pixels 1:1 (the host adds the pixmap at the origin),
so a scene point converts to absolute stage µm with
``SpheroidDetector.back_project_px``. This module holds NO stage geometry of its
own and commands no motion — it reports pixel radii and centres, and the host
converts.
"""

from __future__ import annotations

import logging
import math
from typing import Optional

from PySide6.QtCore import QObject, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QCursor, QFont, QPen
from PySide6.QtWidgets import (
    QGraphicsEllipseItem, QGraphicsItem, QGraphicsSimpleTextItem,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)

# Handle radius in SCENE px before the ItemIgnoresTransformations flag pins it to
# a constant on-screen size. Big enough to grab at any zoom.
_HANDLE_PX = 5.0
# Grab tolerance for the handle, in scene px scaled by the view transform.
_HANDLE_GRAB_PX = 10.0
_MIN_RADIUS_PX = 1.5


class _RadiusHandleItem(QGraphicsEllipseItem):
    """Drag target that sets its parent circle's radius.

    Not movable itself — the view hit-tests it and drives the resize, so the
    handle can never drift away from the circle it belongs to.
    """

    def __init__(self, det_id: str, parent: QGraphicsItem):
        r = _HANDLE_PX
        super().__init__(-r, -r, 2 * r, 2 * r, parent)
        self.det_id = str(det_id)
        self.setBrush(QBrush(QColor(COLORS["yellow"])))
        pen = QPen(QColor("#1e1e2e"))
        pen.setCosmetic(True)
        pen.setWidthF(1.0)
        self.setPen(pen)
        self.setZValue(6)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations,
                     True)
        self.setCursor(QCursor(Qt.CursorShape.SizeHorCursor))
        self.setToolTip("Drag to set this spheroid's diameter")


class SpheroidCircleItem(QGraphicsEllipseItem):
    """One spheroid's editable circle, with its Ø caption and radius handle."""

    def __init__(self, det_id: str, cx: float, cy: float, r: float,
                 *, color: QColor, dashed: bool = False,
                 label: str = "", editable: bool = True):
        super().__init__(QRectF(cx - r, cy - r, 2 * r, 2 * r))
        self.det_id = str(det_id)
        self._r = float(r)
        self._color = QColor(color)
        self._dashed = bool(dashed)
        self.setZValue(4)
        self.setBrush(QBrush(QColor(0, 0, 0, 0)))
        self._apply_pen()
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, editable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setToolTip(label or self.det_id)

        # Children, so a drag carries them (see the module docstring).
        self._label = QGraphicsSimpleTextItem(label or self.det_id, self)
        self._label.setBrush(QBrush(self._color))
        self._label.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        self._label.setZValue(5)
        font = QFont("Consolas")
        font.setPointSize(8)
        self._label.setFont(font)

        self.handle: Optional[_RadiusHandleItem] = None
        if editable:
            self.handle = _RadiusHandleItem(self.det_id, self)
        self._reposition_children()

    # ── geometry ──────────────────────────────────────────────────

    def center(self) -> QPointF:
        """Centre in SCENE coords (``rect().center() + pos()``)."""
        c = self.rect().center()
        return QPointF(c.x() + self.pos().x(), c.y() + self.pos().y())

    def radius(self) -> float:
        return self._r

    def set_radius(self, r: float) -> None:
        """Resize about the CURRENT centre, keeping ``pos()`` untouched."""
        r = max(_MIN_RADIUS_PX, float(r))
        c = self.rect().center()
        self._r = r
        self.setRect(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r))
        self._reposition_children()

    def set_dashed(self, dashed: bool) -> None:
        self._dashed = bool(dashed)
        self._apply_pen()

    def set_highlight(self, on: bool) -> None:
        """Thicken the stroke for the row selected in the side panel."""
        self._highlight = bool(on)
        self._apply_pen()

    def set_label(self, text: str) -> None:
        self._label.setText(text)
        self.setToolTip(text)
        self._reposition_children()

    def _apply_pen(self) -> None:
        pen = QPen(self._color)
        pen.setCosmetic(True)   # constant stroke width at any zoom
        pen.setWidthF(3.0 if getattr(self, "_highlight", False) else 1.8)
        if self._dashed:
            pen.setStyle(Qt.PenStyle.DashLine)
        self.setPen(pen)

    def _reposition_children(self) -> None:
        c = self.rect().center()
        if self.handle is not None:
            self.handle.setPos(c.x() + self._r, c.y())
        self._label.setPos(c.x() + self._r + 4.0, c.y() - 14.0)


class SpheroidOverlay(QObject):
    """Owns the spheroid circles on a host ``QGraphicsView`` + the rim-fit layer.

    The host wires its view's ``scene_clicked`` / ``scene_dragged`` /
    ``scene_released`` signals to :meth:`on_scene_press` / :meth:`on_scene_drag`
    / :meth:`on_scene_release` and enables the view's interactive-items mode.

    Emits geometry in MOSAIC PIXELS; the host converts to stage µm through
    ``SpheroidDetector`` so there is exactly one back-projection in the feature.
    """

    # (det_id, radius_px)
    radius_changed = Signal(str, float)
    radius_committed = Signal(str, float)
    # (det_id, cx_px, cy_px)
    center_committed = Signal(str, float, float)
    # (det_id) selected by clicking its circle
    circle_clicked = Signal(str)
    # (cx_px, cy_px, r_px) from a completed rim fit
    rim_fitted = Signal(float, float, float)
    # An empty-space click, when neither rim mode nor a handle drag claimed it.
    empty_clicked = Signal(float, float)

    def __init__(self, view, parent: QObject | None = None):
        super().__init__(parent)
        self._view = view
        self._items: dict[str, SpheroidCircleItem] = {}
        self._rim_points: list[tuple[float, float]] = []
        self._rim_items: list[QGraphicsItem] = []
        self._rim_preview: Optional[QGraphicsEllipseItem] = None
        self._rim_fit: Optional[tuple[float, float, float]] = None
        self._rim_mode = False
        self._dragging_radius: str = ""
        self._dragging_center: str = ""

    # ── circle lifecycle ──────────────────────────────────────────

    def clear(self) -> None:
        scene = self._scene()
        for item in self._items.values():
            if scene is not None:
                scene.removeItem(item)
        self._items.clear()
        self.clear_rim_points()

    def _scene(self):
        getter = getattr(self._view, "scene_obj", None)
        if callable(getter):
            return getter()
        return self._view.scene()

    def set_circles(self, entries) -> None:
        """Rebuild every circle. ``entries`` = iterable of dicts with
        ``det_id``, ``cx``, ``cy``, ``r`` (mosaic px) and optional ``color``,
        ``dashed``, ``label``."""
        self.clear()
        scene = self._scene()
        if scene is None:
            return
        for e in entries:
            item = SpheroidCircleItem(
                e["det_id"], float(e["cx"]), float(e["cy"]), float(e["r"]),
                color=QColor(e.get("color") or COLORS["green"]),
                dashed=bool(e.get("dashed", False)),
                label=str(e.get("label") or e["det_id"]))
            scene.addItem(item)
            self._items[item.det_id] = item

    def item(self, det_id: str) -> Optional[SpheroidCircleItem]:
        return self._items.get(str(det_id))

    def set_highlight(self, det_id: str) -> None:
        for tid, item in self._items.items():
            item.set_highlight(tid == str(det_id))

    def center_on(self, det_id: str) -> None:
        item = self._items.get(str(det_id))
        if item is None:
            return
        try:
            self._view.centerOn(item.center())
        except Exception:
            pass

    # ── rim-point mode ────────────────────────────────────────────

    def set_rim_mode(self, on: bool) -> None:
        self._rim_mode = bool(on)
        if not self._rim_mode:
            self.clear_rim_points()

    def rim_mode(self) -> bool:
        return self._rim_mode

    def rim_fit(self):
        """``(cx_px, cy_px, r_px)`` of the current fit, or None."""
        return self._rim_fit

    def clear_rim_points(self) -> None:
        scene = self._scene()
        for it in self._rim_items:
            if scene is not None:
                try:
                    scene.removeItem(it)
                except Exception:
                    pass
        self._rim_items.clear()
        self._rim_points.clear()
        self._clear_rim_preview()
        self._rim_fit = None

    def _clear_rim_preview(self) -> None:
        if self._rim_preview is None:
            return
        scene = self._scene()
        if scene is not None:
            try:
                scene.removeItem(self._rim_preview)
            except Exception:
                pass
        self._rim_preview = None

    def commit_rim_fit(self) -> bool:
        """Emit the current fit and clear the points. False when there is none."""
        if self._rim_fit is None:
            return False
        cx, cy, r = self._rim_fit
        self.clear_rim_points()
        self.rim_fitted.emit(cx, cy, r)
        return True

    def _add_rim_point(self, pt: QPointF) -> None:
        scene = self._scene()
        self._rim_points.append((pt.x(), pt.y()))
        if scene is not None:
            dot = QGraphicsEllipseItem(pt.x() - 2.0, pt.y() - 2.0, 4.0, 4.0)
            peach = QColor(COLORS["peach"])
            pen = QPen(peach)
            pen.setCosmetic(True)
            dot.setPen(pen)
            dot.setBrush(QBrush(peach))
            dot.setZValue(7)
            dot.setFlag(
                QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, False)
            scene.addItem(dot)
            self._rim_items.append(dot)
        self._refit_rim()

    def _refit_rim(self) -> None:
        self._clear_rim_preview()
        self._rim_fit = None
        if len(self._rim_points) < 3:
            return
        try:
            from SupportClasses.VisionDetector import fit_circle_to_points
            fit = fit_circle_to_points(self._rim_points)
        except Exception as exc:
            logger.debug("mosaic rim fit failed: %s", exc)
            return
        if fit is None:
            return
        cx, cy, r = (float(v) for v in fit)
        if r < _MIN_RADIUS_PX:
            return
        self._rim_fit = (cx, cy, r)
        scene = self._scene()
        if scene is None:
            return
        preview = QGraphicsEllipseItem(cx - r, cy - r, 2 * r, 2 * r)
        pen = QPen(QColor(COLORS["yellow"]), 2, Qt.PenStyle.DashLine)
        pen.setCosmetic(True)
        preview.setPen(pen)
        preview.setBrush(QBrush(QColor(0, 0, 0, 0)))
        preview.setZValue(7)
        scene.addItem(preview)
        self._rim_preview = preview

    # ── mouse routing (driven by the host view) ───────────────────

    def _handle_at(self, scene_pt: QPointF) -> str:
        """det_id of the radius handle under a scene point, else ""."""
        tol = _HANDLE_GRAB_PX / max(1e-6, self._view_scale())
        best = ""
        best_d = tol
        for det_id, item in self._items.items():
            if item.handle is None:
                continue
            hp = item.handle.scenePos()
            d = math.hypot(scene_pt.x() - hp.x(), scene_pt.y() - hp.y())
            if d <= best_d:
                best_d = d
                best = det_id
        return best

    def _view_scale(self) -> float:
        try:
            return abs(self._view.transform().m11()) or 1.0
        except Exception:
            return 1.0

    def on_scene_press(self, scene_pt: QPointF) -> None:
        """A left press that the view did NOT hand to a movable item."""
        if self._rim_mode:
            self._add_rim_point(scene_pt)
            return
        # The radius handle takes priority — same as the plate designer, so
        # resizing a small circle is never hijacked into a move.
        det_id = self._handle_at(scene_pt)
        if det_id:
            self._dragging_radius = det_id
            self._apply_radius_drag(scene_pt, commit=False)
            return
        hit = self._circle_at(scene_pt)
        if hit:
            self.circle_clicked.emit(hit)
            return
        self.empty_clicked.emit(scene_pt.x(), scene_pt.y())

    def _circle_at(self, scene_pt: QPointF) -> str:
        """det_id whose circle contains (or nearly contains) a scene point."""
        best = ""
        best_d = None
        for det_id, item in self._items.items():
            c = item.center()
            d = math.hypot(scene_pt.x() - c.x(), scene_pt.y() - c.y())
            if d <= item.radius() * 1.1 and (best_d is None or d < best_d):
                best_d = d
                best = det_id
        return best

    def on_scene_drag(self, scene_pt: QPointF) -> None:
        if self._dragging_radius:
            self._apply_radius_drag(scene_pt, commit=False)

    def on_scene_release(self, scene_pt: QPointF) -> None:
        if self._dragging_radius:
            self._apply_radius_drag(scene_pt, commit=True)
            self._dragging_radius = ""
            return
        # A Qt-driven move drag may have relocated a circle; report the result so
        # the host can re-derive its stage position.
        for det_id, item in self._items.items():
            if item.pos().x() or item.pos().y():
                c = item.center()
                # Fold the drag offset back into the rect so the invariant
                # "centre == rect().center() + pos()" starts from pos()==(0,0)
                # again, matching how the circle was first built.
                item.setPos(0.0, 0.0)
                item.setRect(QRectF(c.x() - item.radius(),
                                    c.y() - item.radius(),
                                    2 * item.radius(), 2 * item.radius()))
                item._reposition_children()
                self.center_committed.emit(det_id, c.x(), c.y())

    def _apply_radius_drag(self, scene_pt: QPointF, *, commit: bool) -> None:
        item = self._items.get(self._dragging_radius)
        if item is None:
            return
        c = item.center()
        r = math.hypot(scene_pt.x() - c.x(), scene_pt.y() - c.y())
        item.set_radius(r)
        signal = self.radius_committed if commit else self.radius_changed
        signal.emit(item.det_id, item.radius())

    def is_dragging(self) -> bool:
        return bool(self._dragging_radius or self._dragging_center)
