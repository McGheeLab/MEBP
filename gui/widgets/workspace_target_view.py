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


# (x_zr_um, y_zr_um, target_id[, size_um])
# The 4th element is OPTIONAL — a measured spheroid diameter in µm, added in
# v7.8. Existing 3-tuple callers keep the flat fallback radius.
TargetTuple = tuple

class WorkspaceTargetView(JogWorkspaceView):
    """JogWorkspaceView that also paints pick + place target markers.

    Picks render as small green-outlined circles with their id labels.
    Place renders as a slightly larger mauve-outlined circle.

    v7.8: a target carrying a measured diameter is drawn at its TRUE size (never
    below the flat minimum, so a small spheroid stays clickable/visible), which
    makes the relative sizes in a queue readable at a glance.
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
        for a, b in zip(self._pick_targets, self._place_targets):
            p.drawLine(self._um_to_px(a[0], a[1]), self._um_to_px(b[0], b[1]))

        # Picks: green
        green = QColor(COLORS["green"])
        green_fill = QColor(green.red(), green.green(), green.blue(), 70)
        p.setBrush(QBrush(green_fill))
        p.setPen(QPen(green, 2))
        self._draw_target_set(p, self._pick_targets, self._PICK_RADIUS_PX)

        # Places: mauve
        mauve = QColor(COLORS["mauve"])
        mauve_fill = QColor(mauve.red(), mauve.green(), mauve.blue(), 90)
        p.setBrush(QBrush(mauve_fill))
        p.setPen(QPen(mauve, 3))
        self._draw_target_set(p, self._place_targets, self._PLACE_RADIUS_PX)

        p.end()

    def _draw_target_set(self, p, targets, min_radius_px: int) -> None:
        for t in targets:
            x_um, y_um, tid = t[0], t[1], t[2]
            size_um = float(t[3]) if len(t) > 3 else 0.0
            r = self._target_radius_px(size_um, min_radius_px)
            pos = self._um_to_px(x_um, y_um)
            p.drawEllipse(pos, r, r)
            p.drawText(pos + QPointF(r + 3, -2), tid)

    def _target_radius_px(self, size_um: float, min_radius_px: int) -> float:
        """A measured diameter in screen px, floored at the flat marker size."""
        if size_um <= 0:
            return float(min_radius_px)
        # Two µm points one diameter apart → the on-screen diameter.
        try:
            a = self._um_to_px(0.0, 0.0)
            b = self._um_to_px(size_um, 0.0)
        except Exception:
            return float(min_radius_px)
        return max(float(min_radius_px), abs(b.x() - a.x()) / 2.0)
