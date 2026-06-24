"""
print_trajectory_monitor.py — Quick Print live trajectory + needle view.

v7.5.x: A lightweight top-down canvas for the Quick Print page. It shows the
**planned** print toolpath for the selected well + object combo (the trajectory
waypoint path), and — as the print runs — draws the **live executed path** over
it plus a **fading mauve ghost trail** at the needle (the same breadcrumb effect
used on the Jog page overview, ``JogWorkspaceView``).

Unlike ``JogWorkspaceView`` (which frames the whole XY envelope + plate), this
view **auto-fits to the selected well's planned path** so the individual
waypoints are clearly visible. It is purely a display — it owns no motion and
emits no signals.

Coordinate contract
-------------------
Every input is **zero-referenced µm** (the same frame the Jog page feeds
``JogWorkspaceView.set_position``), so the planned path and the live needle
position line up exactly:

    planned waypoint (zero-ref µm) = (well_center_mm + path_point_mm) * 1000
    live needle      (zero-ref µm) = stage_xy_um - controller.zero_position

The render is rotated 180° (reflected about the fitted-region centre) to match
the Jog overview's orientation; because the reflection is applied uniformly to
every drawn point, the planned/executed/ghost overlays always register.

Public API
----------
    set_planned_path(segments)        # list[list[(x_um, y_um)]] | None
    set_well_boundary(center_um, radius_um)
    set_needle(outer_diameter_um)
    set_position(x_um, y_um)          # both can be None
    set_recording(on)                 # accumulate the executed polyline
    reset_live()                      # clear executed polyline + ghost trail
"""

from __future__ import annotations

from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from gui.scaling import s, scaled_font_size
from gui.styles import COLORS

# ── Tuning ─────────────────────────────────────────────────────────

_BREADCRUMB_MAX = 24            # ghost-trail length (matches JogWorkspaceView)
_EXECUTED_MAX = 6000           # cap the persistent executed polyline
_NEEDLE_MIN_PX = 4.0
_MIN_SPAN_UM = 500.0           # smallest framed extent (so a dot doesn't blow up)
_FIT_FILL = 0.86               # fraction of the content rect the path fills


def _qc(name: str, alpha: int | None = None) -> QColor:
    c = QColor(COLORS[name])
    if alpha is not None:
        c.setAlpha(alpha)
    return c


class PrintTrajectoryMonitorView(QWidget):
    """Top-down planned-path + live-needle monitor. Coords are zero-ref µm."""

    # Rotate the rendering 180° to match JogWorkspaceView's orientation (the
    # Prior II stage origin is bottom-right with +X/+Y toward top-left). The
    # reflection is its own inverse and applied uniformly, so all overlays
    # stay registered regardless. Driven by the per-machine ``plate_flip_180``
    # setting (single source of truth on StageController); the owning page
    # pushes the live value via :meth:`set_plate_flip_180`. The class constant
    # is the construction-time default.
    _FLIP_DISPLAY_180 = True

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(s(240), s(200))
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        # Per-machine 180° flip (default = class constant until pushed).
        self._flip_180: bool = self._FLIP_DISPLAY_180

        # Planned toolpath: one polyline per object/segment (zero-ref µm).
        self._segments: list[list[tuple[float, float]]] = []
        # Optional well boundary (centre zero-ref µm, radius µm).
        self._well_center_um: tuple[float, float] | None = None
        self._well_radius_um: float = 0.0

        # Live state.
        self._needle_od_um: float = 410.0
        self._needle_x_um: float | None = None
        self._needle_y_um: float | None = None
        self._recording: bool = False
        self._breadcrumbs: deque[tuple[float, float]] = deque(
            maxlen=_BREADCRUMB_MAX)
        self._executed: list[tuple[float, float]] = []

        # Cached bounding box of the framed content (zero-ref µm).
        self._bbox: tuple[float, float, float, float] | None = None

    # ── Public API ─────────────────────────────────────────────────

    def set_planned_path(
        self, segments: list[list[tuple[float, float]]] | None
    ) -> None:
        """Store the planned toolpath (zero-ref µm), one list per object.

        ``None`` or empty clears the path and shows the placeholder."""
        cleaned: list[list[tuple[float, float]]] = []
        for seg in (segments or []):
            pts = [(float(x), float(y)) for (x, y) in seg]
            if pts:
                cleaned.append(pts)
        self._segments = cleaned
        self._recompute_bbox()
        self.update()

    def set_well_boundary(
        self, center_um: tuple[float, float] | None, radius_um: float
    ) -> None:
        """Optional well-wall circle drawn behind the path (zero-ref µm)."""
        if center_um is None or radius_um <= 0:
            self._well_center_um = None
            self._well_radius_um = 0.0
        else:
            self._well_center_um = (float(center_um[0]), float(center_um[1]))
            self._well_radius_um = float(radius_um)
        self._recompute_bbox()
        self.update()

    def set_needle(self, outer_diameter_um: float | None) -> None:
        if outer_diameter_um and outer_diameter_um > 0:
            self._needle_od_um = float(outer_diameter_um)
        self.update()

    def set_plate_flip_180(self, flip: bool) -> None:
        """v7.5.x: per-machine 180° display flip (from StageController
        ``plate_flip_180``), pushed by the owning page so this view agrees with
        the JogWorkspaceView orientation convention."""
        flip = bool(flip)
        if flip != self._flip_180:
            self._flip_180 = flip
            self.update()

    def set_position(self, x_um: float | None, y_um: float | None) -> None:
        """Update the live needle position (zero-ref µm).

        Appends to the fading ghost trail when the needle has moved > 1 µm
        (matches ``JogWorkspaceView.set_position``), and — while recording —
        to the persistent executed polyline."""
        if x_um is None or y_um is None:
            self._needle_x_um = None
            self._needle_y_um = None
            self.update()
            return
        x_um = float(x_um)
        y_um = float(y_um)
        moved = (self._needle_x_um is None
                 or abs(x_um - self._needle_x_um) > 1.0
                 or abs(y_um - self._needle_y_um) > 1.0)
        if moved:
            self._breadcrumbs.append((x_um, y_um))
            if self._recording:
                self._executed.append((x_um, y_um))
                if len(self._executed) > _EXECUTED_MAX:
                    del self._executed[0:len(self._executed) - _EXECUTED_MAX]
        self._needle_x_um = x_um
        self._needle_y_um = y_um
        self.update()

    def set_recording(self, on: bool) -> None:
        """Start/stop accumulating the persistent executed polyline."""
        self._recording = bool(on)

    def reset_live(self) -> None:
        """Clear the executed polyline and the ghost trail (e.g. at print
        start, or when the well/object selection changes)."""
        self._executed.clear()
        self._breadcrumbs.clear()
        self.update()

    # ── Framing / coordinate mapping ───────────────────────────────

    def _recompute_bbox(self) -> None:
        xs: list[float] = []
        ys: list[float] = []
        for seg in self._segments:
            for x, y in seg:
                xs.append(x)
                ys.append(y)
        if self._well_center_um is not None and self._well_radius_um > 0:
            cx, cy = self._well_center_um
            r = self._well_radius_um
            xs.extend((cx - r, cx + r))
            ys.extend((cy - r, cy + r))
        if not xs:
            self._bbox = None
            return
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        # Enforce a minimum span so a single point / tiny path stays sane.
        if x_max - x_min < _MIN_SPAN_UM:
            cx = (x_min + x_max) / 2.0
            x_min, x_max = cx - _MIN_SPAN_UM / 2.0, cx + _MIN_SPAN_UM / 2.0
        if y_max - y_min < _MIN_SPAN_UM:
            cy = (y_min + y_max) / 2.0
            y_min, y_max = cy - _MIN_SPAN_UM / 2.0, cy + _MIN_SPAN_UM / 2.0
        self._bbox = (x_min, y_min, x_max, y_max)

    def _content_rect(self) -> QRectF:
        m = s(8)
        readout = s(20)
        return QRectF(
            m, m,
            max(1.0, self.width() - 2 * m),
            max(1.0, self.height() - 2 * m - readout),
        )

    def _scale(self) -> tuple[float, float, float] | None:
        """Return (k px/µm, ox, oy) fitting the bbox into the content rect."""
        if self._bbox is None:
            return None
        x_min, y_min, x_max, y_max = self._bbox
        env_w = max(1.0, x_max - x_min)
        env_h = max(1.0, y_max - y_min)
        rect = self._content_rect()
        k = min(rect.width() / env_w, rect.height() / env_h) * _FIT_FILL
        drawn_w = env_w * k
        drawn_h = env_h * k
        ox = rect.left() + (rect.width() - drawn_w) / 2.0 - x_min * k
        oy = rect.top() + (rect.height() - drawn_h) / 2.0 - y_min * k
        return (k, ox, oy)

    def _apply_display_flip(
        self, x_um: float, y_um: float
    ) -> tuple[float, float]:
        if not self._flip_180 or self._bbox is None:
            return (x_um, y_um)
        x_min, y_min, x_max, y_max = self._bbox
        return ((x_min + x_max) - x_um, (y_min + y_max) - y_um)

    def _um_to_px(self, x_um: float, y_um: float) -> QPointF:
        sc = self._scale()
        if sc is None:
            return QPointF(0, 0)
        k, ox, oy = sc
        fx, fy = self._apply_display_flip(x_um, y_um)
        return QPointF(fx * k + ox, fy * k + oy)

    # ── Painting ───────────────────────────────────────────────────

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), _qc('base'))

        if not self._segments and self._well_center_um is None:
            self._paint_placeholder(p)
            p.end()
            return

        p.save()
        p.setClipRect(self._content_rect())
        self._paint_well_boundary(p)
        self._paint_planned(p)
        self._paint_executed(p)
        self._paint_breadcrumbs(p)
        self._paint_needle(p)
        p.restore()

        self._paint_readout(p)
        p.end()

    def _paint_placeholder(self, p: QPainter) -> None:
        font = p.font()
        font.setPointSizeF(scaled_font_size(10))
        p.setFont(font)
        p.setPen(QPen(_qc('overlay0')))
        p.drawText(
            self.rect(), Qt.AlignmentFlag.AlignCenter,
            "Select a well and an object to preview the print plan",
        )

    def _paint_well_boundary(self, p: QPainter) -> None:
        if self._well_center_um is None or self._well_radius_um <= 0:
            return
        sc = self._scale()
        if sc is None:
            return
        k = sc[0]
        center = self._um_to_px(*self._well_center_um)
        r_px = self._well_radius_um * k
        p.setBrush(QBrush(_qc('crust')))
        p.setPen(QPen(_qc('surface1'), 1.0, Qt.PenStyle.DashLine))
        p.drawEllipse(center, r_px, r_px)

    def _paint_planned(self, p: QPainter) -> None:
        if not self._segments:
            return
        # Thin dashed grey travel between objects (segment end → next start).
        travel_pen = QPen(_qc('overlay0', 120), 1.0, Qt.PenStyle.DashLine)
        prev_end: tuple[float, float] | None = None
        for seg in self._segments:
            if prev_end is not None and seg:
                p.setPen(travel_pen)
                p.drawLine(self._um_to_px(*prev_end), self._um_to_px(*seg[0]))
            prev_end = seg[-1] if seg else prev_end

        # Planned print runs: connected blue polyline + small waypoint dots.
        path_pen = QPen(_qc('blue', 220), max(1.0, s(1.4)))
        path_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        path_pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        dot_r = max(1.0, s(1.6))
        for seg in self._segments:
            if len(seg) >= 2:
                poly = QPolygonF([self._um_to_px(x, y) for (x, y) in seg])
                p.setPen(path_pen)
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawPolyline(poly)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(_qc('blue', 150)))
            for x, y in seg:
                p.drawEllipse(self._um_to_px(x, y), dot_r, dot_r)

    def _paint_executed(self, p: QPainter) -> None:
        if len(self._executed) < 2:
            return
        poly = QPolygonF([self._um_to_px(x, y) for (x, y) in self._executed])
        pen = QPen(_qc('green'), max(1.4, s(2.0)))
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPolyline(poly)

    def _paint_breadcrumbs(self, p: QPainter) -> None:
        if not self._breadcrumbs:
            return
        n = len(self._breadcrumbs)
        for i, (x, y) in enumerate(self._breadcrumbs):
            t = (i + 1) / n
            alpha = int(20 + 120 * t)
            radius = max(1.2, s(2) * t)
            p.setBrush(QBrush(_qc('mauve', alpha)))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(self._um_to_px(x, y), radius, radius)

    def _paint_needle(self, p: QPainter) -> None:
        if self._needle_x_um is None:
            return
        sc = self._scale()
        if sc is None:
            return
        k = sc[0]
        center = self._um_to_px(self._needle_x_um, self._needle_y_um)
        r_px = max(_NEEDLE_MIN_PX, (self._needle_od_um / 2.0) * k)

        # Soft halo so the marker stays visible over the path.
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_qc('red', 60)))
        p.drawEllipse(center, r_px + s(3), r_px + s(3))
        # Open ring sized to the needle OD.
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(_qc('red'), 1.5))
        p.drawEllipse(center, r_px, r_px)
        # Centre dot.
        p.setBrush(QBrush(_qc('red')))
        p.setPen(Qt.PenStyle.NoPen)
        dot_r = max(1.2, r_px * 0.22)
        p.drawEllipse(center, dot_r, dot_r)

    def _paint_readout(self, p: QPainter) -> None:
        content = self._content_rect()
        band = QRectF(content.left(), content.bottom() + s(4),
                      content.width(), s(18))
        font = p.font()
        font.setPointSizeF(scaled_font_size(9))
        p.setFont(font)
        p.setPen(QPen(_qc('subtext0')))
        if self._needle_x_um is None:
            text = "X: —   Y: —"
        else:
            text = (f"X: {self._needle_x_um:+,.1f} µm    "
                    f"Y: {self._needle_y_um:+,.1f} µm")
        p.drawText(
            band,
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            text,
        )
        if self._recording:
            p.setPen(QPen(_qc('green')))
            p.drawText(
                band,
                int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter),
                "● recording",
            )
