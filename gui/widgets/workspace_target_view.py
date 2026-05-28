"""workspace_target_view.py — JogWorkspaceView with target overlays.

v7.4.x: Subclasses `JogWorkspaceView` (the top-down XY canvas used on
the Jog page) to render pick + place target markers on top of the
normal plate / wells / breadcrumb / needle layer.

The base view's `_um_to_px(x_zr_um, y_zr_um)` is reused — targets are
expected in zero-ref µm (same frame as `set_well_positions`). The
workflow page converts the picker's stage-frame µm into zero-ref µm
before pushing in.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from gui.styles import COLORS
from gui.widgets.jog_workspace_view import JogWorkspaceView


# (x_zr_um, y_zr_um, target_id)
TargetTuple = tuple[float, float, str]


class WorkspaceTargetView(JogWorkspaceView):
    """JogWorkspaceView that also paints pick + place target markers.

    Picks render as small green-outlined circles with their id labels.
    Place renders as a slightly larger mauve-outlined circle.
    """

    _PICK_RADIUS_PX = 7
    _PLACE_RADIUS_PX = 9

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._pick_targets: list[TargetTuple] = []
        self._place_targets: list[TargetTuple] = []

    # ── public API ─────────────────────────────────────────────────

    def set_pick_targets(self, targets: list[TargetTuple]) -> None:
        self._pick_targets = list(targets)
        self.update()

    def set_place_targets(self, targets: list[TargetTuple]) -> None:
        self._place_targets = list(targets)
        self.update()

    def clear_targets(self) -> None:
        self._pick_targets = []
        self._place_targets = []
        self.update()

    # ── painting ───────────────────────────────────────────────────

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self._pick_targets and not self._place_targets:
            return

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Connector lines: pick[i] ↔ place[i].
        p.setPen(QPen(QColor(COLORS["overlay0"]), 1, Qt.DashLine))
        for (px, py, _), (dx, dy, _) in zip(
                self._pick_targets, self._place_targets):
            p.drawLine(self._um_to_px(px, py), self._um_to_px(dx, dy))

        # Picks: green
        green = QColor(COLORS["green"])
        green_fill = QColor(green.red(), green.green(), green.blue(), 70)
        p.setBrush(QBrush(green_fill))
        p.setPen(QPen(green, 2))
        for x_um, y_um, tid in self._pick_targets:
            pos = self._um_to_px(x_um, y_um)
            p.drawEllipse(pos, self._PICK_RADIUS_PX, self._PICK_RADIUS_PX)
            p.drawText(pos + QPointF(self._PICK_RADIUS_PX + 3, -2), tid)

        # Places: mauve
        mauve = QColor(COLORS["mauve"])
        mauve_fill = QColor(mauve.red(), mauve.green(), mauve.blue(), 90)
        p.setBrush(QBrush(mauve_fill))
        p.setPen(QPen(mauve, 3))
        for x_um, y_um, tid in self._place_targets:
            pos = self._um_to_px(x_um, y_um)
            p.drawEllipse(pos, self._PLACE_RADIUS_PX, self._PLACE_RADIUS_PX)
            p.drawText(pos + QPointF(self._PLACE_RADIUS_PX + 3, -2), tid)

        p.end()
