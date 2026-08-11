"""Screen-constant handles for the v7.12 plate canvas.

All of these carry ``ItemIgnoresTransformations``, so their hit area is a fixed
number of device pixels at any zoom. That is what keeps a handle grabbable on a
zoomed-out 384-well plate and stops the rotation handle landing on top of a
well.

``RotationHandleItem`` is deliberately separate from ``RadiusHandleItem``: the
v7.4.x canvas had ONE handle whose drag set radius *and* angle together, so a
ring could not be oriented without also resizing it — which is precisely what
the operator asked to fix ("a handle to rotate one of the circles to orient
it"). Two facts that are independently settable need two handles.

The same ``RotationHandleItem`` serves the ring, the grid and a placed rosette.
"""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainterPath, QPen
from PySide6.QtWidgets import QGraphicsItem, QGraphicsObject

# Catppuccin Mocha, inlined to keep this module import-light.
COL_HANDLE = "#94e2d5"      # teal
COL_ROTATE = "#f9e2af"      # yellow
COL_SEED = "#cba6f7"        # mauve
COL_DATUM = "#a6adc8"       # subtext0

HANDLE_PX = 9.0
Z_GIZMO = 24


class _BaseHandle(QGraphicsObject):
    """A screen-constant grab target that reports drags in SCENE coordinates."""

    #: Set by the canvas; called with (kind, owner_id, scene_pos).
    def __init__(self, kind: str, owner_id: int, colour: str,
                 tooltip: str = ""):
        super().__init__()
        self.kind = kind
        self.owner_id = owner_id
        self._colour = QColor(colour)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setZValue(Z_GIZMO)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.OpenHandCursor)
        if tooltip:
            self.setToolTip(tooltip)
        self._hover = False

    def boundingRect(self) -> QRectF:
        h = HANDLE_PX
        return QRectF(-h, -h, 2 * h, 2 * h)

    def hoverEnterEvent(self, ev):
        self._hover = True
        self.update()
        super().hoverEnterEvent(ev)

    def hoverLeaveEvent(self, ev):
        self._hover = False
        self.update()
        super().hoverLeaveEvent(ev)

    def _pen(self) -> QPen:
        pen = QPen(QColor("#11111b"))
        pen.setWidthF(1.2)
        pen.setCosmetic(True)
        return pen

    def _brush(self) -> QBrush:
        c = QColor(self._colour)
        if not self._hover:
            c.setAlpha(215)
        return QBrush(c)


class RadiusHandleItem(_BaseHandle):
    """Square. Drag sets the ring DIAMETER; the start angle is locked."""

    def __init__(self, owner_id: int):
        super().__init__("radius", owner_id, COL_HANDLE,
                         "Drag to resize the ring (angle stays put).\n"
                         "Hold Shift to snap to 0.5 mm.")

    def paint(self, p, _opt, _w=None):
        h = HANDLE_PX * 0.62
        p.setPen(self._pen())
        p.setBrush(self._brush())
        p.drawRect(QRectF(-h, -h, 2 * h, 2 * h))


class RotationHandleItem(_BaseHandle):
    """Round, on a short stalk. Drag sets the ANGLE; the size is locked."""

    def __init__(self, owner_id: int, label: str = "ring"):
        super().__init__("rotate", owner_id, COL_ROTATE,
                         f"Drag to orient the {label} (size stays put).\n"
                         f"Hold Shift to snap to 15°.")

    def paint(self, p, _opt, _w=None):
        r = HANDLE_PX * 0.66
        pen = self._pen()
        p.setPen(pen)
        p.setBrush(self._brush())
        p.drawEllipse(QPointF(0.0, 0.0), r, r)
        # A tick, so the handle reads as "turn me" rather than "drag me".
        p.setPen(QPen(QColor("#11111b"), 1.4))
        p.drawLine(QPointF(0.0, -r), QPointF(0.0, -r - HANDLE_PX * 0.55))


class SeedHandleItem(_BaseHandle):
    """Diamond at a pattern's anchor. Drag moves the whole pattern."""

    def __init__(self, owner_id: int):
        super().__init__("seed", owner_id, COL_SEED,
                         "The pattern's anchor — drag to move the whole "
                         "pattern.\nDimension this point to place the pattern.")

    def paint(self, p, _opt, _w=None):
        h = HANDLE_PX * 0.7
        path = QPainterPath()
        path.moveTo(0.0, -h)
        path.lineTo(h, 0.0)
        path.lineTo(0.0, h)
        path.lineTo(-h, 0.0)
        path.closeSubpath()
        p.setPen(self._pen())
        p.setBrush(self._brush())
        p.drawPath(path)


class AnchorMarkerItem(QGraphicsItem):
    """The always-visible reference point of a pattern.

    Distinct from :class:`SeedHandleItem`, which is the bold *drag* handle that
    appears only while the pattern is selected. This is the quiet marker that
    is ALWAYS on screen and always hit-testable, because a ring centre is not
    otherwise a thing you can click: with ``center_well=False`` there is no
    well there, so before this existed the ring's own reference point could not
    be dimensioned to a plate edge at all.

    Screen-constant, so it stays grabbable on a zoomed-out plate.
    """

    def __init__(self, ref, label: str = ""):
        super().__init__()
        self.ref = ref
        self._label = label
        self.hover = False
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setAcceptHoverEvents(True)
        self.setAcceptedMouseButtons(Qt.NoButton)   # the view does hit-testing
        self.setZValue(Z_GIZMO - 3)
        self.setToolTip("Pattern reference point"
                        + (f" — {label}" if label else "")
                        + "\nDimension this to a plate edge to place the "
                          "pattern.")

    def boundingRect(self) -> QRectF:
        h = HANDLE_PX
        return QRectF(-h, -h, 2 * h, 2 * h)

    def hoverEnterEvent(self, ev):
        self.hover = True
        self.update()
        super().hoverEnterEvent(ev)

    def hoverLeaveEvent(self, ev):
        self.hover = False
        self.update()
        super().hoverLeaveEvent(ev)

    def paint(self, p, _opt, _w=None):
        arm = HANDLE_PX * 0.78
        c = QColor(COL_SEED)
        pen = QPen(c)
        pen.setWidthF(2.0 if self.hover else 1.3)
        pen.setCosmetic(True)
        p.setPen(pen)
        p.drawLine(QPointF(-arm, 0.0), QPointF(arm, 0.0))
        p.drawLine(QPointF(0.0, -arm), QPointF(0.0, arm))
        p.setBrush(QBrush(c) if self.hover else QBrush(Qt.NoBrush))
        p.drawEllipse(QPointF(0.0, 0.0), arm * 0.42, arm * 0.42)


class DatumMarkerItem(QGraphicsItem):
    """A labelled origin triad at the document's chosen datum."""

    def __init__(self, label: str = ""):
        super().__init__()
        self._label = label
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setZValue(Z_GIZMO - 2)

    def boundingRect(self) -> QRectF:
        return QRectF(-6, -34, 90, 46)

    def paint(self, p, _opt, _w=None):
        arm = 22.0
        pen = QPen(QColor(COL_DATUM))
        pen.setWidthF(1.4)
        pen.setCosmetic(True)
        p.setPen(pen)
        p.drawLine(QPointF(0, 0), QPointF(arm, 0))
        p.drawLine(QPointF(0, 0), QPointF(0, -arm))
        p.setBrush(QBrush(QColor(COL_DATUM)))
        p.drawEllipse(QPointF(0, 0), 2.2, 2.2)
        f = p.font()
        f.setPointSizeF(max(f.pointSizeF() - 1.0, 6.0))
        p.setFont(f)
        p.drawText(QPointF(arm + 3, 3), "X")
        p.drawText(QPointF(3, -arm - 3), "Y")
        if self._label:
            p.setPen(QPen(QColor(COL_DATUM)))
            p.drawText(QPointF(6, 14), self._label)
