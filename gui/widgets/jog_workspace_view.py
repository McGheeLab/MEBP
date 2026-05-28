"""
JogWorkspaceView — polished top-down XY workspace canvas.

A clean, dashboard-grade rendering of the printer's XY workspace as
seen from directly above. Designed to feel like a single coherent
display rather than a stack of overlays: subtle grid, soft plate,
crisp wells, a glowing needle, a fading breadcrumb trail, and a clean
segmented mode toggle at the top.

All input coordinates are in **zero-referenced µm**. Click events emit
zero-ref µm. Free-target mode lets the user click anywhere inside the
safety envelope; Snap-to-wells mode constrains clicks to the nearest
calibrated/approximate well.

Public API
----------
    set_plate(plate)
    set_well_positions(positions, kind="calibrated"|"approximate")
    set_safety_limits(limits)
    set_needle(outer_diameter_um)
    set_position(x_um, y_um)         # both can be None
    set_target_mode("free"|"snap")
    clear_breadcrumbs()

Signals
-------
    position_clicked(x_um, y_um)     # zero-ref
    well_clicked(name)               # snap mode
"""

from __future__ import annotations

import math
from collections import deque
from typing import Literal

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (
    QBrush, QColor, QFont, QMouseEvent, QPainter, QPen,
)
from PySide6.QtWidgets import QMenu, QSizePolicy, QWidget

from gui.scaling import s, scaled_font_size
from gui.styles import COLORS

# ── Tuning ─────────────────────────────────────────────────────────

_BREADCRUMB_MAX = 24
_NEEDLE_MIN_PX = 4.0
_NEAR_LIMIT_MARGIN_UM = 500.0
_SNAP_RADIUS_FRACTION = 0.55


def _qc(name: str, alpha: int | None = None) -> QColor:
    c = QColor(COLORS[name])
    if alpha is not None:
        c.setAlpha(alpha)
    return c


def _mix(a: QColor, b: QColor, t: float) -> QColor:
    """Linear mix between two QColors, ignoring alpha."""
    return QColor(
        int(a.red() * (1 - t) + b.red() * t),
        int(a.green() * (1 - t) + b.green() * t),
        int(a.blue() * (1 - t) + b.blue() * t),
    )


class JogWorkspaceView(QWidget):
    """Top-down XY workspace canvas. Coords are zero-ref µm."""

    position_clicked = Signal(float, float)
    well_clicked = Signal(str)
    # Fired when the user picks "Fast travel here" from the right-click
    # context menu. Coordinates are zero-ref µm in the same frame as
    # ``position_clicked``. The page is expected to run the full
    # safe-travel-then-restore-Z workflow.
    fast_travel_requested = Signal(float, float)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(s(280), s(220))
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)

        # State
        self._safety_limits = None
        self._plate = None
        self._wells_cal: dict[str, tuple[float, float]] = {}
        self._wells_approx: dict[str, tuple[float, float]] = {}
        self._current_well: str | None = None
        self._needle_od_um: float = 410.0
        self._needle_x_um: float | None = None
        self._needle_y_um: float | None = None
        self._hover_px: tuple[float, float] | None = None
        self._mode: Literal["free", "snap"] = "snap"
        self._breadcrumbs: deque[tuple[float, float]] = deque(
            maxlen=_BREADCRUMB_MAX)

        # Toggle hit boxes (computed in paint)
        self._toggle_rect_free: QRectF | None = None
        self._toggle_rect_snap: QRectF | None = None

    # ── Public API ─────────────────────────────────────────────────

    def set_safety_limits(self, limits) -> None:
        self._safety_limits = limits
        self.update()

    def set_plate(self, plate) -> None:
        self._plate = plate
        self.update()

    def set_well_positions(
        self,
        positions: dict[str, tuple[float, float]] | None,
        kind: Literal["calibrated", "approximate"] = "calibrated",
    ) -> None:
        if positions is None:
            positions = {}
        if kind == "calibrated":
            self._wells_cal = dict(positions)
            for n in self._wells_cal:
                self._wells_approx.pop(n, None)
        else:
            self._wells_approx = {
                n: p for n, p in positions.items()
                if n not in self._wells_cal
            }
        self.update()

    def set_needle(self, outer_diameter_um: float | None) -> None:
        if outer_diameter_um and outer_diameter_um > 0:
            self._needle_od_um = float(outer_diameter_um)
        self.update()

    def set_position(self, x_um: float | None, y_um: float | None) -> None:
        if x_um is None or y_um is None:
            self._needle_x_um = None
            self._needle_y_um = None
            self.update()
            return
        if (self._needle_x_um is None
                or abs(x_um - self._needle_x_um) > 1.0
                or abs(y_um - self._needle_y_um) > 1.0):
            self._breadcrumbs.append((float(x_um), float(y_um)))
        self._needle_x_um = float(x_um)
        self._needle_y_um = float(y_um)
        self._current_well = self._nearest_well_within_radius()
        self.update()

    def set_target_mode(self, mode: Literal["free", "snap"]) -> None:
        if mode != self._mode and mode in ("free", "snap"):
            self._mode = mode
            self.update()

    def target_mode(self) -> str:
        return self._mode

    def clear_breadcrumbs(self) -> None:
        self._breadcrumbs.clear()
        self.update()

    # ── Coordinate mapping ─────────────────────────────────────────

    def _envelope_bounds(self) -> tuple[float, float, float, float] | None:
        if self._safety_limits is None:
            return None
        return (
            float(self._safety_limits.xy_min_x),
            float(self._safety_limits.xy_min_y),
            float(self._safety_limits.xy_max_x),
            float(self._safety_limits.xy_max_y),
        )

    def _content_rect(self) -> QRectF:
        """The pixel-space rect the workspace draws into.

        Reserves a clean band at the top for the segmented mode toggle
        and a clean band at the bottom for the coordinate readout. The
        side margin is set to just a few pixels so the envelope fills
        nearly the entire canvas horizontally.
        """
        m = s(4)
        toggle_band = s(34)
        readout_band = s(22)
        return QRectF(
            m,
            m + toggle_band,
            max(1, self.width() - 2 * m),
            max(1, self.height() - 2 * m - toggle_band - readout_band),
        )

    def _scale(self) -> tuple[float, float, float] | None:
        env = self._envelope_bounds()
        if env is None:
            return None
        x_min, y_min, x_max, y_max = env
        env_w = max(1.0, x_max - x_min)
        env_h = max(1.0, y_max - y_min)
        rect = self._content_rect()
        sx = rect.width() / env_w
        sy = rect.height() / env_h
        k = min(sx, sy)
        drawn_w = env_w * k
        drawn_h = env_h * k
        ox = rect.left() + (rect.width() - drawn_w) / 2.0 - x_min * k
        oy = rect.top() + (rect.height() - drawn_h) / 2.0 - y_min * k
        return (k, ox, oy)

    def _um_to_px(self, x_um: float, y_um: float) -> QPointF:
        sc = self._scale()
        if sc is None:
            return QPointF(0, 0)
        k, ox, oy = sc
        return QPointF(x_um * k + ox, y_um * k + oy)

    def _px_to_um(self, px_x: float, px_y: float) -> tuple[float, float]:
        sc = self._scale()
        if sc is None:
            return (0.0, 0.0)
        k, ox, oy = sc
        return ((px_x - ox) / k, (px_y - oy) / k)

    # ── Snap logic ─────────────────────────────────────────────────

    def _well_radius_um(self, name: str | None = None) -> float:
        """Well radius in µm. v7.4.8: per-well when `name` is given (custom
        plates / flattened rosette sub-wells have mixed diameters); else a
        representative radius (uniform diameter, or the max for customs)."""
        if self._plate is None:
            return 0.0
        if name is not None:
            try:
                d = self._plate.get_well_info(name).diameter
                if d > 0:
                    return d * 1000.0 / 2.0
            except Exception:
                pass
        d = float(getattr(self._plate, "well_diameter", 0.0) or 0.0)
        if d <= 0:
            # Custom plate (no uniform diameter) — use the largest well.
            d = self._plate._max_well_radius_mm() * 2.0
        return d * 1000.0 / 2.0

    def _all_wells(self) -> dict[str, tuple[float, float]]:
        merged = dict(self._wells_approx)
        merged.update(self._wells_cal)
        return merged

    def _nearest_well_within_radius(self) -> str | None:
        if self._needle_x_um is None or not self._all_wells():
            return None
        r = self._well_radius_um()
        if r <= 0:
            return None
        best, best_d = None, r * 1.05
        for name, (wx, wy) in self._all_wells().items():
            d = math.hypot(wx - self._needle_x_um,
                           wy - self._needle_y_um)
            if d < best_d:
                best_d = d
                best = name
        return best

    def _snap_hit(self, x_um: float, y_um: float) -> tuple[str, float, float] | None:
        r = self._well_radius_um() * _SNAP_RADIUS_FRACTION
        if r <= 0:
            return None
        best, bx, by, bd = None, 0.0, 0.0, r * 1.05
        for name, (wx, wy) in self._all_wells().items():
            d = math.hypot(wx - x_um, wy - y_um)
            if d < bd:
                bd = d
                best, bx, by = name, wx, wy
        if best is None:
            return None
        return (best, bx, by)

    # ── Painting ───────────────────────────────────────────────────

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)

        # Solid dark base
        p.fillRect(self.rect(), _qc('base'))

        self._paint_toggle(p)

        if self._safety_limits is None:
            self._paint_placeholder(p)
            p.end()
            return

        self._paint_envelope(p)
        self._paint_plate_and_wells(p)
        self._paint_breadcrumbs(p)
        self._paint_needle(p)
        self._paint_hover_ghost(p)
        self._paint_readout(p)

        p.end()

    # ── Segmented mode toggle ─────────────────────────────────────

    def _paint_toggle(self, p: QPainter) -> None:
        # A single rounded pill containing two segments. The active
        # segment fills with mauve; the inactive segment is a subtle
        # surface tone. The pill sits flush against the top-left.
        x = s(12)
        y = s(10)
        h = s(26)
        seg_w = s(96)
        gap_inset = 2
        total_w = seg_w * 2

        outer = QRectF(x, y, total_w, h)
        # Pill background
        p.setPen(QPen(_qc('surface2'), 1.0))
        p.setBrush(QBrush(_qc('surface0')))
        p.drawRoundedRect(outer, h / 2, h / 2)

        free_rect = QRectF(x + gap_inset, y + gap_inset,
                           seg_w - gap_inset, h - 2 * gap_inset)
        snap_rect = QRectF(x + seg_w, y + gap_inset,
                           seg_w - gap_inset, h - 2 * gap_inset)
        self._toggle_rect_free = free_rect
        self._toggle_rect_snap = snap_rect

        # Active segment fill
        active = self._mode
        if active == "free":
            active_rect = free_rect
        else:
            active_rect = snap_rect
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_qc('mauve')))
        p.drawRoundedRect(active_rect, (h - 2 * gap_inset) / 2,
                          (h - 2 * gap_inset) / 2)

        # Labels
        font = p.font()
        font.setPointSizeF(scaled_font_size(9))
        font.setBold(True)
        p.setFont(font)

        free_color = _qc('crust') if active == "free" else _qc('subtext0')
        snap_color = _qc('crust') if active == "snap" else _qc('subtext0')
        p.setPen(QPen(free_color))
        p.drawText(free_rect, Qt.AlignmentFlag.AlignCenter, "Free")
        p.setPen(QPen(snap_color))
        p.drawText(snap_rect, Qt.AlignmentFlag.AlignCenter, "Snap to well")

    # ── Placeholder ────────────────────────────────────────────────

    def _paint_placeholder(self, p: QPainter) -> None:
        font = p.font()
        font.setPointSizeF(scaled_font_size(10))
        p.setFont(font)
        p.setPen(QPen(_qc('overlay0')))
        p.drawText(
            self.rect(),
            Qt.AlignmentFlag.AlignCenter,
            "Waiting for hardware…",
        )

    # ── Envelope ───────────────────────────────────────────────────

    def _paint_envelope(self, p: QPainter) -> None:
        env = self._envelope_bounds()
        if env is None:
            return
        x_min, y_min, x_max, y_max = env
        tl = self._um_to_px(x_min, y_min)
        br = self._um_to_px(x_max, y_max)
        rect = QRectF(tl, br).normalized()

        # Soft inset surface for the printer envelope (subtle, lifted
        # from the page background so the plate sits on top of it).
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_qc('mantle')))
        p.drawRoundedRect(rect, s(8), s(8))

        # Hairline border for the envelope edge
        pen = QPen(_qc('surface1'), 1.0)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(rect, s(8), s(8))

        # Near-limit edge highlight: subtle glow only when stage is
        # within margin. Soft, not harsh.
        near = {}
        if (self._needle_x_um is not None
                and self._safety_limits is not None):
            near = self._safety_limits.check_xy_near_limit(
                self._needle_x_um, self._needle_y_um,
                margin=_NEAR_LIMIT_MARGIN_UM)

        if any(near.get(k) for k in
               ("x_near_min", "x_near_max", "y_near_min", "y_near_max")):
            # Thin red border for warning state
            p.setPen(QPen(_qc('red'), 1.6))
            p.drawRoundedRect(rect, s(8), s(8))

    # ── Plate + wells ──────────────────────────────────────────────

    def _paint_plate_and_wells(self, p: QPainter) -> None:
        wells = self._all_wells()
        if not wells:
            return

        # Plate bounds with tasteful padding
        r_um = self._well_radius_um()
        xs = [w[0] for w in wells.values()]
        ys = [w[1] for w in wells.values()]
        x_min, x_max = min(xs) - r_um, max(xs) + r_um
        y_min, y_max = min(ys) - r_um, max(ys) + r_um
        pad = r_um * 0.45
        plate_rect = QRectF(
            self._um_to_px(x_min - pad, y_min - pad),
            self._um_to_px(x_max + pad, y_max + pad),
        ).normalized()

        # Plate body with a very faint gradient feel via two
        # overlapping rounded rects (cheap "elevation").
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_qc('crust')))
        p.drawRoundedRect(plate_rect, s(10), s(10))
        # Subtle highlight at top
        highlight = QColor(_qc('surface0'))
        highlight.setAlpha(60)
        p.setBrush(QBrush(highlight))
        p.drawRoundedRect(
            plate_rect.adjusted(0, 0, 0, -plate_rect.height() * 0.55),
            s(10), s(10))
        # Hairline border
        p.setPen(QPen(_qc('surface1'), 1.0))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(plate_rect, s(10), s(10))

        # Wells: soft circles with a clean border
        sc = self._scale()
        if sc is None:
            return
        k = sc[0]

        def _r_px(nm: str) -> float:
            return max(3.0, self._well_radius_um(nm) * k)

        # First pass — non-current wells
        for name, (wx, wy) in wells.items():
            if name == self._current_well:
                continue
            self._paint_well(p, wx, wy, _r_px(name), name, current=False)

        # Second pass — current well drawn last so its highlight sits
        # on top of everything else (no z-fighting against neighbors).
        if self._current_well is not None and self._current_well in wells:
            wx, wy = wells[self._current_well]
            self._paint_well(p, wx, wy, _r_px(self._current_well),
                             self._current_well, current=True)

    def _paint_well(self, p: QPainter, wx: float, wy: float,
                    r_px: float, name: str, *, current: bool) -> None:
        center = self._um_to_px(wx, wy)
        if name in self._wells_cal:
            fill = _qc('green', 36)
            border = _qc('green', 200)
        elif name in self._wells_approx:
            fill = _qc('yellow', 28)
            border = _qc('yellow', 170)
        else:
            fill = QColor(0, 0, 0, 0)
            border = _qc('surface2')

        if current:
            # Soft halo behind the current-well marker
            halo = _qc('blue', 70)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(halo))
            p.drawEllipse(center, r_px + s(5), r_px + s(5))
            fill = _qc('blue', 80)
            border = _qc('blue')

        p.setBrush(QBrush(fill))
        p.setPen(QPen(border, 1.2))
        p.drawEllipse(center, r_px, r_px)

    # ── Breadcrumb trail ───────────────────────────────────────────

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

    # ── Target hair (current XY position overlay) ─────────────────

    def _paint_needle(self, p: QPainter) -> None:
        """Render the current XY position as a target-hair crosshair.

        A thin red horizontal line spans the entire content area at the
        needle's Y, a thin red vertical line spans the entire content
        area at the needle's X. They meet at an open ring sized to the
        needle's outer diameter, with a small center dot inside the
        ring for precise targeting.
        """
        if self._needle_x_um is None:
            return
        center = self._um_to_px(self._needle_x_um, self._needle_y_um)
        sc = self._scale()
        if sc is None:
            return
        k = sc[0]
        r_px = max(_NEEDLE_MIN_PX, (self._needle_od_um / 2.0) * k)
        content = self._content_rect()
        cx, cy = center.x(), center.y()

        # Faint full-length target lines (broken around the ring)
        line_pen = QPen(_qc('red', 200), 1.0)
        p.setPen(line_pen)
        gap = r_px + s(3)
        # Horizontal
        p.drawLine(QPointF(content.left() + s(2), cy),
                   QPointF(cx - gap, cy))
        p.drawLine(QPointF(cx + gap, cy),
                   QPointF(content.right() - s(2), cy))
        # Vertical
        p.drawLine(QPointF(cx, content.top() + s(2)),
                   QPointF(cx, cy - gap))
        p.drawLine(QPointF(cx, cy + gap),
                   QPointF(cx, content.bottom() - s(2)))

        # Subtle red glow behind the ring so it stays visible over wells
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_qc('red', 60)))
        p.drawEllipse(center, r_px + s(3), r_px + s(3))

        # Open ring at the needle position (sized to OD)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(_qc('red'), 1.5))
        p.drawEllipse(center, r_px, r_px)

        # Center dot for precision
        p.setBrush(QBrush(_qc('red')))
        p.setPen(Qt.PenStyle.NoPen)
        dot_r = max(1.2, r_px * 0.22)
        p.drawEllipse(center, dot_r, dot_r)

    # ── Hover ghost ────────────────────────────────────────────────

    def _paint_hover_ghost(self, p: QPainter) -> None:
        if self._hover_px is None:
            return
        hx, hy = self._hover_px
        ux, uy = self._px_to_um(hx, hy)
        env = self._envelope_bounds()
        if env is None:
            return
        x_min, y_min, x_max, y_max = env
        if not (x_min <= ux <= x_max and y_min <= uy <= y_max):
            return

        snap_info = None
        if self._mode == "snap":
            snap_info = self._snap_hit(ux, uy)
            if snap_info is None:
                # Faint "no target" cursor only
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.setPen(QPen(_qc('overlay0'), 1.0, Qt.PenStyle.DotLine))
                p.drawEllipse(QPointF(hx, hy), s(7), s(7))
                return
            _, sx, sy = snap_info
            anchor = self._um_to_px(sx, sy)
            ux, uy = sx, sy
        else:
            anchor = QPointF(hx, hy)

        sc = self._scale()
        if sc is None:
            return
        k = sc[0]
        r_px = max(_NEEDLE_MIN_PX, (self._needle_od_um / 2.0) * k)

        # Ghost ring at the anchor — clean dashed circle
        p.setBrush(Qt.BrushStyle.NoBrush)
        pen = QPen(_qc('peach'), 1.3, Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.drawEllipse(anchor, max(r_px * 1.4, s(6)),
                      max(r_px * 1.4, s(6)))

        # Floating mini-label
        font = p.font()
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(True)
        p.setFont(font)

        if snap_info is not None:
            label = f"{snap_info[0]} · {ux:+,.0f}, {uy:+,.0f} µm"
        else:
            label = f"{ux:+,.0f}, {uy:+,.0f} µm"
        # Center the label below-right of the anchor, padded.
        pad = s(8)
        lw = s(160)
        lh = s(18)
        lx = anchor.x() + s(12)
        ly = anchor.y() + s(10)
        # Clamp to widget bounds
        if lx + lw > self.width() - s(4):
            lx = anchor.x() - lw - s(12)
        if ly + lh > self.height() - s(4):
            ly = anchor.y() - lh - s(10)
        bg = QRectF(lx, ly, lw, lh)
        p.setBrush(QBrush(_qc('crust', 230)))
        p.setPen(QPen(_qc('peach'), 1.0))
        p.drawRoundedRect(bg, s(4), s(4))
        p.setPen(QPen(_qc('text')))
        p.drawText(
            bg.adjusted(pad, 0, -pad, 0),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            label,
        )

    # ── Bottom readout ─────────────────────────────────────────────

    def _paint_readout(self, p: QPainter) -> None:
        content = self._content_rect()
        band_y = content.bottom() + s(4)
        band = QRectF(content.left(), band_y, content.width(), s(22))

        font = p.font()
        font.setPointSizeF(scaled_font_size(10))
        font.setBold(True)
        p.setFont(font)
        if self._needle_x_um is None:
            text = "X: —   Y: —"
        else:
            text = (f"X: {self._needle_x_um:+,.1f} µm    "
                    f"Y: {self._needle_y_um:+,.1f} µm")
        p.setPen(QPen(_qc('text')))
        p.drawText(
            band,
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            text,
        )

        # Subtle mode marker at the right end
        mode_text = "Free" if self._mode == "free" else "Snap"
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(False)
        p.setFont(font)
        p.setPen(QPen(_qc('subtext0')))
        p.drawText(
            band,
            int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter),
            f"mode · {mode_text}",
        )

    # ── Mouse handling ─────────────────────────────────────────────

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        self._hover_px = (event.position().x(), event.position().y())
        self.update()

    def leaveEvent(self, _e) -> None:
        self._hover_px = None
        self.update()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        px = event.position().x()
        py = event.position().y()

        if self._toggle_rect_free and self._toggle_rect_free.contains(px, py):
            self.set_target_mode("free")
            return
        if self._toggle_rect_snap and self._toggle_rect_snap.contains(px, py):
            self.set_target_mode("snap")
            return

        env = self._envelope_bounds()
        if env is None:
            return
        ux, uy = self._px_to_um(px, py)
        x_min, y_min, x_max, y_max = env
        if not (x_min <= ux <= x_max and y_min <= uy <= y_max):
            return

        if self._mode == "snap":
            hit = self._snap_hit(ux, uy)
            if hit is None:
                return
            name, wx, wy = hit
            self.well_clicked.emit(name)
            self.position_clicked.emit(wx, wy)
        else:
            self.position_clicked.emit(ux, uy)

    # ── Right-click context menu ───────────────────────────────────

    def contextMenuEvent(self, event):
        """Right-click → "Fast travel here" popup.

        Works in both Free and Snap modes. In Snap mode the target
        snaps to the nearest well (within the snap radius); if no well
        is in range, no menu appears. In Free mode the target is the
        exact cursor position inside the safety envelope.
        """
        if self._safety_limits is None:
            return
        env = self._envelope_bounds()
        if env is None:
            return
        pos = event.pos()
        ux, uy = self._px_to_um(pos.x(), pos.y())
        x_min, y_min, x_max, y_max = env
        if not (x_min <= ux <= x_max and y_min <= uy <= y_max):
            return

        target_x: float
        target_y: float
        label: str

        if self._mode == "snap":
            hit = self._snap_hit(ux, uy)
            if hit is None:
                return
            name, target_x, target_y = hit
            label = f"Fast travel here  ({name})"
        else:
            target_x, target_y = ux, uy
            label = "Fast travel here"

        menu = QMenu(self)
        menu.setStyleSheet(self._context_menu_stylesheet())
        action = menu.addAction(label)
        sub = menu.addAction(f"   {target_x:+,.1f}  ·  {target_y:+,.1f} µm")
        sub.setEnabled(False)
        chosen = menu.exec(event.globalPos())
        if chosen is action:
            self.fast_travel_requested.emit(target_x, target_y)

    @staticmethod
    def _context_menu_stylesheet() -> str:
        return (
            f"QMenu {{"
            f"  background-color: {COLORS['mantle']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {s(6)}px;"
            f"  padding: {s(4)}px;"
            f"}}"
            f"QMenu::item {{"
            f"  background-color: transparent;"
            f"  color: {COLORS['text']};"
            f"  padding: {s(6)}px {s(14)}px;"
            f"  border-radius: {s(4)}px;"
            f"}}"
            f"QMenu::item:selected {{"
            f"  background-color: {COLORS['mauve']};"
            f"  color: {COLORS['crust']};"
            f"}}"
            f"QMenu::item:disabled {{"
            f"  color: {COLORS['overlay0']};"
            f"  background-color: transparent;"
            f"  font-size: {int(s(11))}px;"
            f"}}"
        )
