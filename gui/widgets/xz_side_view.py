"""
XZSideView — polished side-view visualization (X horizontal, Z vertical).

A clean side-on rendering of the needle hovering over the plate. The
needle is drawn at scale — its length and outer diameter come from the
configured needle spec — so the operator can see the relationship
between current Z and the plate at a glance.

Layers (back → front)
---------------------
  1. Soft inset plot surface (mantle, rounded, hairline border)
  2. Plate block at Z = 0 (crust, with brighter top edge for the
     plate-surface line)
  3. Well cavity beneath the needle when one is configured
  4. Safe-Z indicator line + badge
  5. Needle: tapered body + triangle tip + soft halo
  6. Right-side annotation card (label / value rows, divider lines)
  7. Bottom readout band (X and Z in mm)

Public API
----------
    set_safety_limits(limits)
    set_needle(outer_diameter_um, length_mm)
    set_safe_z(safe_z_mm)
    set_position(x_um, z_mm)         # either can be None
    set_well_under_needle(diameter_mm, depth_mm)   # both None to clear

Custom Z locations (v7.5.x, opt-in — Jog page only)
---------------------------------------------------
    set_custom_z_enabled(True)       # enables the gestures below
    set_custom_z_locations(values)   # restore (raw zero-ref mm; no emit)
    custom_z_locations()             # -> list[float]
    add_custom_z(z) / remove_custom_z(z)

With the feature enabled, click-dragging on the plot shows a floating mm
readout tag and on release tags that Z as a custom location; a plain
click ON the current-Z line tags the needle's current Z. Each custom
location renders like a Z reference (dashed line + clickable ★ badge →
``go_to_z_requested``) plus an ✕ remove button (right-click on the badge
also removes). ``custom_z_changed(list)`` fires on every add/remove so
the host page can persist. All values are raw zero-ref mm (the move
frame); display goes through the ``_disp()`` sign like everything else.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF, QWheelEvent
from PySide6.QtWidgets import (
    QPushButton, QScrollBar, QSizePolicy, QWidget,
)

from gui.scaling import s, scaled_font_size, sf, sp
from gui.styles import COLORS

_NEEDLE_MIN_PX_WIDE = 4.0


def _qc(name: str, alpha: int | None = None) -> QColor:
    c = QColor(COLORS[name])
    if alpha is not None:
        c.setAlpha(alpha)
    return c


class XZSideView(QWidget):
    """Side-view canvas: X horizontal, Z vertical, needle drawn to scale."""

    # Fired when the user clicks one of the captured-Z badges.
    # The argument is the target Z in mm (zero-referenced).
    go_to_z_requested = Signal(float)

    # v7.5.x: fired whenever the operator adds/removes a custom Z
    # location. Argument = the full current list (raw zero-ref mm) so
    # the host page can persist it. NOT fired by the programmatic
    # set_custom_z_locations() restore (no save loop).
    custom_z_changed = Signal(list)

    # Two custom locations closer than this are considered the same
    # (dedup tolerance for add_custom_z / match tolerance for remove).
    CUSTOM_Z_TOL_MM = 0.005

    # Ordered list of Z-reference keys → (short label, accent color key)
    # Each labelled Z value the page sets via set_z_references gets a
    # thin dashed line at that Z + a clickable badge on the right edge.
    _Z_REF_META: list[tuple[str, str, str]] = [
        # (key, label, color key from gui.styles.COLORS)
        ("replace_z",      "Replace",     "peach"),
        ("max_z",          "Max",         "red"),
        ("fast_move_z",    "Safe",        "green"),
        ("plate_top_z",    "Plate ↑",     "blue"),
        ("plate_bottom_z", "Plate ↓",     "mauve"),
    ]

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(s(180), s(220))
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)

        self._safety_limits = None
        # v7.5.x: XY envelope is absolute stage µm; this view is fed zero-ref
        # X, so subtract the zero reference from the X bounds for display.
        self._zero_off_x: float = 0.0
        # v7.5.x: Z envelope is absolute Marlin raw mm; this view is fed
        # zero-ref Z, so subtract the zero reference from the Z bounds.
        self._zero_off_z: float = 0.0
        # v7.5.x: Z display sign. +1 = the view's native up-positive frame
        # (unchanged). -1 = number Z as height (up = +) on machines whose raw
        # Marlin Z increases downward (ME3B V1). Applied ONLY to display:
        # the safety bounds, the needle/reference pixel mapping, and the
        # readout text. The go_to_z_requested emit stays in the raw zero-ref
        # frame (hit rects store the raw value) so the move contract is
        # unchanged. Pushed in via set_z_display_sign().
        self._z_disp_sign: float = 1.0
        self._needle_od_um: float = 410.0
        self._needle_length_mm: float = 12.7
        self._safe_z_mm: float | None = None
        self._x_um: float | None = None
        self._z_mm: float | None = None
        self._well_diameter_mm: float | None = None
        self._well_depth_mm: float | None = None

        # Z-reference values (keyed by the same labels as the
        # calibration page's get_z_references()). Each non-None entry
        # renders as a dashed horizontal line + clickable badge.
        self._z_refs: dict[str, float | None] = {
            "replace_z": None, "max_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }
        # Hit rects for the badges (populated in paint).
        self._z_ref_hit_rects: dict[str, tuple[QRectF, float]] = {}
        # v7.9.1: which references the quick-move strip shows. None = all
        # (default, unchanged). See set_visible_z_references.
        self._z_ref_visible: set[str] | None = None

        # ── v7.5.x: custom Z locations (opt-in; Jog page) ──────────
        # Values are raw zero-ref mm — the same frame as _z_refs and
        # the go_to_z_requested emit, so the move contract holds on
        # both Z polarities. Display converts via _disp().
        self._custom_z_enabled = False
        self._custom_z: list[float] = []
        # (badge_rect, close_rect, z_raw) triples, populated in paint.
        self._custom_hit_rects: list[tuple[QRectF, QRectF, float]] = []
        # Drag-to-tag gesture state. A press inside the plot is only
        # "pending" until the cursor moves past the threshold — a plain
        # click is reserved for tagging the current-Z line.
        self._drag_pending = False
        self._drag_active = False
        self._drag_press_pos: QPointF | None = None
        self._drag_z_disp: float | None = None

        # Z-axis zoom — 1.0 means show the full safety range.
        # Larger values zoom in around `_z_zoom_center_mm` (or current
        # cursor Z if the wheel was used).
        self._z_zoom: float = 1.0
        self._z_zoom_center_mm: float | None = None
        # Hit rect for the reset-zoom badge (computed in paint)
        self._reset_zoom_rect: QRectF | None = None

        # ── Z-zoom controls ────────────────────────────────────
        # Buttons + vertical scrollbar live as child widgets of the
        # canvas. Positioned in resizeEvent. The mouse-wheel handler
        # still works in parallel.
        self._scrollbar_updating = False
        self._btn_zoom_in = QPushButton("+", self)
        self._btn_zoom_in.setObjectName("xzZoomBtn")
        self._btn_zoom_in.setCursor(Qt.PointingHandCursor)
        self._btn_zoom_in.setToolTip("Zoom in (Z axis)")
        self._btn_zoom_in.clicked.connect(self._on_zoom_in)

        self._btn_zoom_out = QPushButton("−", self)
        self._btn_zoom_out.setObjectName("xzZoomBtn")
        self._btn_zoom_out.setCursor(Qt.PointingHandCursor)
        self._btn_zoom_out.setToolTip("Zoom out (Z axis)")
        self._btn_zoom_out.clicked.connect(self._on_zoom_out)

        self._scrollbar = QScrollBar(Qt.Orientation.Vertical, self)
        self._scrollbar.setObjectName("xzZoomScroll")
        self._scrollbar.setToolTip(
            "Scroll the visible Z range (only while zoomed in)")
        self._scrollbar.valueChanged.connect(
            self._on_scrollbar_changed)

        self._style_zoom_widgets()
        self._sync_scrollbar()

    # ── Public API ─────────────────────────────────────────────────

    def set_safety_limits(self, limits) -> None:
        self._safety_limits = limits
        self.update()

    def set_zero_offset_x(self, zero_x_um: float) -> None:
        """v7.5.x: current X zero reference (absolute stage µm). Shifts the
        absolute X envelope into the zero-ref frame this view is fed."""
        off = float(zero_x_um)
        if off != self._zero_off_x:
            self._zero_off_x = off
            self.update()

    def set_zero_offset_z(self, zero_z_mm: float) -> None:
        """v7.5.x: current Z zero reference (absolute Marlin raw mm). Shifts the
        absolute Z envelope into the zero-ref frame this view is fed."""
        off = float(zero_z_mm)
        if off != self._zero_off_z:
            self._zero_off_z = off
            self.update()

    def set_z_display_sign(self, sign: float) -> None:
        """v7.5.x: set the Z display sign (+1 native, -1 = height/up-positive).

        Display-only: flips the picture orientation, the safety bounds, and
        the readout text. The ``go_to_z_requested`` emit stays raw zero-ref.
        """
        sign = -1.0 if float(sign) < 0 else 1.0
        if sign != self._z_disp_sign:
            self._z_disp_sign = sign
            self.update()

    def _disp(self, z_zero_ref: float) -> float:
        """Raw zero-ref Z (mm) → this view's display frame (height when -1)."""
        return self._z_disp_sign * z_zero_ref

    def set_needle(self, outer_diameter_um: float | None = None,
                   length_mm: float | None = None) -> None:
        if outer_diameter_um is not None and outer_diameter_um > 0:
            self._needle_od_um = float(outer_diameter_um)
        if length_mm is not None and length_mm > 0:
            self._needle_length_mm = float(length_mm)
        self.update()

    def set_safe_z(self, safe_z_mm: float | None) -> None:
        """Legacy hook — mirrors into the ``fast_move_z`` slot of the
        full Z-references dict so all consumers stay in sync.
        """
        value = float(safe_z_mm) if safe_z_mm is not None else None
        self._safe_z_mm = value
        self._z_refs["fast_move_z"] = value
        self.update()

    def set_z_references(self, refs: dict) -> None:
        """Receive captured Z heights from the Calibration page.

        Keys (all mm, zero-referenced; missing = ``None``):
            ``replace_z`` · ``max_z`` · ``fast_move_z`` ·
            ``plate_top_z`` · ``plate_bottom_z``

        Each non-None entry draws a thin dashed line on the side view
        plus a small clickable badge on the right edge. Clicking the
        badge emits :sig:`go_to_z_requested(z_mm)`.
        """
        if not refs:
            return
        for key in self._z_refs:
            if key in refs:
                value = refs[key]
                self._z_refs[key] = (
                    float(value) if value is not None else None)
        # Keep the legacy safe-Z mirror in sync
        if "fast_move_z" in refs and refs["fast_move_z"] is not None:
            self._safe_z_mm = float(refs["fast_move_z"])
        elif "fast_move_z" in refs and refs["fast_move_z"] is None:
            self._safe_z_mm = None
        self.update()

    def set_visible_z_references(self, keys) -> None:
        """Choose WHICH Z-reference badges the quick-move strip shows.

        v7.9.1. ``keys`` is an iterable of the keys in :attr:`_Z_REF_META`, or
        ``None`` to show every reference that has a value (the previous, and
        still default, behaviour). Unknown keys are ignored.

        This is presentation only: the hidden reference keeps its value and
        every consumer that reads it — the print floor, the Hardware Info card,
        `get_z_references` — is untouched. Hiding by writing ``None`` into the
        reference dict instead would have disarmed the print-floor clamp.
        """
        if keys is None:
            self._z_ref_visible = None
        else:
            known = {k for k, _, _ in self._Z_REF_META}
            self._z_ref_visible = {str(k) for k in keys} & known
        self.update()

    def visible_z_references(self) -> set:
        """The currently shown reference keys (all of them when unfiltered)."""
        if self._z_ref_visible is None:
            return {k for k, _, _ in self._Z_REF_META}
        return set(self._z_ref_visible)

    def set_position(self, x_um: float | None, z_mm: float | None) -> None:
        self._x_um = float(x_um) if x_um is not None else None
        self._z_mm = float(z_mm) if z_mm is not None else None
        self.update()

    def set_well_under_needle(
        self,
        well_diameter_mm: float | None,
        well_depth_mm: float | None,
    ) -> None:
        self._well_diameter_mm = (
            float(well_diameter_mm) if well_diameter_mm else None)
        self._well_depth_mm = (
            float(well_depth_mm) if well_depth_mm else None)
        self.update()

    # ── v7.5.x: custom Z locations ─────────────────────────────────

    def set_custom_z_enabled(self, enabled: bool) -> None:
        """Enable the drag-to-tag / click-current-line gestures.

        Off by default so the other XZSideView hosts (calibration +
        workflow pages) keep their exact legacy mouse behaviour.
        """
        self._custom_z_enabled = bool(enabled)
        if self._custom_z_enabled:
            self.setToolTip(
                "Drag on the Z axis to tag a custom Z location (mm readout "
                "follows the cursor; release to set it).\n"
                "Click the red current-Z line to tag the needle's current "
                "height.\n"
                "Click a ★ badge to move Z there · ✕ (or right-click the "
                "badge) removes it.")
        self.update()

    def set_custom_z_locations(self, values) -> None:
        """Restore custom Z locations (raw zero-ref mm).

        Programmatic — does NOT emit ``custom_z_changed`` so a host
        restoring from settings can't loop back into a save.
        """
        cleaned: list[float] = []
        for v in (values or []):
            try:
                z = float(v)
            except (TypeError, ValueError):
                continue
            if not any(abs(z - c) <= self.CUSTOM_Z_TOL_MM for c in cleaned):
                cleaned.append(z)
        self._custom_z = sorted(cleaned)
        self.update()

    def custom_z_locations(self) -> list[float]:
        """Current custom Z locations (raw zero-ref mm, sorted)."""
        return list(self._custom_z)

    def add_custom_z(self, z_raw_mm) -> bool:
        """Add a custom location (raw zero-ref mm). Dedups within
        ``CUSTOM_Z_TOL_MM``. Emits ``custom_z_changed`` on success."""
        try:
            z = float(z_raw_mm)
        except (TypeError, ValueError):
            return False
        if any(abs(z - c) <= self.CUSTOM_Z_TOL_MM for c in self._custom_z):
            return False
        self._custom_z.append(z)
        self._custom_z.sort()
        self.update()
        self.custom_z_changed.emit(list(self._custom_z))
        return True

    def remove_custom_z(self, z_raw_mm) -> bool:
        """Remove the custom location matching ``z_raw_mm`` (within
        tolerance). Emits ``custom_z_changed`` on success."""
        try:
            z = float(z_raw_mm)
        except (TypeError, ValueError):
            return False
        for i, c in enumerate(self._custom_z):
            if abs(z - c) <= self.CUSTOM_Z_TOL_MM:
                del self._custom_z[i]
                self.update()
                self.custom_z_changed.emit(list(self._custom_z))
                return True
        return False

    # ── Coordinate scales ──────────────────────────────────────────

    def _zoom_strip_width(self) -> int:
        return s(20)

    def _content_rect(self) -> QRectF:
        """Plot area, with chrome reserved for the zoom strip on the
        right and a two-line readout band along the bottom.
        """
        m = s(8)
        strip_w = self._zoom_strip_width()
        readout_band = s(38)
        return QRectF(
            m,
            m,
            max(1, self.width() - 2 * m - strip_w - s(4)),
            max(1, self.height() - 2 * m - readout_band),
        )

    def _x_bounds_um(self) -> tuple[float, float] | None:
        if self._safety_limits is None:
            return None
        # v7.5.x: absolute envelope → zero-ref for display.
        zx = self._zero_off_x
        return (float(self._safety_limits.xy_min_x) - zx,
                float(self._safety_limits.xy_max_x) - zx)

    def _z_safety_bounds(self) -> tuple[float, float] | None:
        """v7.5.x: the absolute Z envelope shifted into this view's zero-ref
        frame (subtract the current Z zero reference). Single chokepoint so
        the scale, scrollbar, and paint all agree."""
        if self._safety_limits is None:
            return None
        off = self._zero_off_z
        # v7.5.x: into the display frame. ZDIR=-1 reverses ordering, so
        # convert both ends and re-sort so callers always get (min, max).
        a = self._disp(float(self._safety_limits.z_min) - off)
        b = self._disp(float(self._safety_limits.z_max) - off)
        return (min(a, b), max(a, b))

    def _z_bounds_mm(self) -> tuple[float, float] | None:
        """Visible Z bounds, accounting for the user's zoom level."""
        sb = self._z_safety_bounds()
        if sb is None:
            return None
        base_min, base_max = sb
        if self._z_zoom <= 1.0:
            return (base_min, base_max)
        base_range = base_max - base_min
        view_range = base_range / self._z_zoom
        center = self._z_zoom_center_mm
        if center is None:
            # Default zoom focus: the plate surface (Z = 0).
            center = 0.0
        half = view_range / 2.0
        z_min = center - half
        z_max = center + half
        # Pin to the safety bounds — don't show Z outside the envelope.
        if z_min < base_min:
            z_max = min(base_max, z_max + (base_min - z_min))
            z_min = base_min
        if z_max > base_max:
            z_min = max(base_min, z_min - (z_max - base_max))
            z_max = base_max
        return (z_min, z_max)

    def _x_to_px(self, x_um: float) -> float:
        bounds = self._x_bounds_um()
        rect = self._content_rect()
        if bounds is None:
            return rect.center().x()
        x_min, x_max = bounds
        span = max(1.0, x_max - x_min)
        frac = (x_um - x_min) / span
        return rect.left() + frac * rect.width()

    def _z_to_px(self, z_mm: float) -> float:
        bounds = self._z_bounds_mm()
        rect = self._content_rect()
        if bounds is None:
            return rect.center().y()
        z_min, z_max = bounds
        span = max(0.1, z_max - z_min)
        frac = (z_max - z_mm) / span
        return rect.top() + frac * rect.height()

    def _x_um_per_px(self) -> float:
        bounds = self._x_bounds_um()
        rect = self._content_rect()
        if bounds is None or rect.width() <= 0:
            return 1.0
        return (bounds[1] - bounds[0]) / rect.width()

    def _px_to_z_disp(self, y_px: float) -> float | None:
        """Inverse of :meth:`_z_to_px` — pixel row → Z in the DISPLAY
        frame. Uses the zoomed visible bounds so dragging while zoomed
        maps exactly; the input row is clamped into the plot."""
        bounds = self._z_bounds_mm()
        rect = self._content_rect()
        if bounds is None or rect.height() <= 0:
            return None
        z_min, z_max = bounds
        y = min(max(float(y_px), rect.top()), rect.bottom())
        frac = (y - rect.top()) / max(1.0, rect.height())
        return z_max - frac * (z_max - z_min)

    # ── Painting ───────────────────────────────────────────────────

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)

        p.fillRect(self.rect(), _qc('base'))

        if self._safety_limits is None:
            self._paint_placeholder(p)
            p.end()
            return

        rect = self._content_rect()

        # Plot area: subtle inset surface
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_qc('mantle')))
        p.drawRoundedRect(rect, s(8), s(8))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(_qc('surface1'), 1.0))
        p.drawRoundedRect(rect, s(8), s(8))

        self._paint_plate_and_well(p, rect)
        self._paint_z_references(p, rect)
        self._paint_needle(p, rect)
        self._paint_readout(p, rect)
        self._paint_zoom_badge(p, rect)
        self._paint_drag_ghost(p, rect)

        p.end()

    def _paint_placeholder(self, p: QPainter) -> None:
        p.setPen(QPen(_qc('overlay0')))
        font = p.font()
        font.setPointSizeF(scaled_font_size(10))
        p.setFont(font)
        p.drawText(self.rect(),
                   Qt.AlignmentFlag.AlignCenter,
                   "Waiting for hardware…")

    def _paint_plate_and_well(self, p: QPainter, rect: QRectF) -> None:
        z0_y = self._z_to_px(0.0)

        # Plate block: from Z=0 down to the bottom of the plot, filled
        # with crust + a brighter top "surface" stripe.
        if z0_y < rect.bottom():
            plate_rect = QRectF(
                rect.left() + 1, z0_y,
                rect.width() - 2, rect.bottom() - z0_y - 1)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(_qc('crust')))
            p.drawRect(plate_rect)
            # Subtle horizontal lines for "material" texture
            line_pen = QPen(_qc('surface0', 80), 1.0)
            p.setPen(line_pen)
            step = s(8)
            y = plate_rect.top() + step
            while y < plate_rect.bottom():
                p.drawLine(QPointF(plate_rect.left(), y),
                           QPointF(plate_rect.right(), y))
                y += step

            # Bright surface line
            p.setPen(QPen(_qc('text'), 1.4))
            p.drawLine(QPointF(rect.left() + 1, z0_y),
                       QPointF(rect.right() - 1, z0_y))

            # Plate-surface label badge
            self._draw_pill(
                p,
                QPointF(rect.left() + s(8), z0_y - s(10)),
                "plate surface",
                _qc('subtext0'),
                _qc('crust', 200),
            )

        # Well cavity beneath the needle, when applicable
        if (self._well_diameter_mm and self._well_depth_mm
                and self._x_um is not None):
            half_w_um = (self._well_diameter_mm / 2.0) * 1000.0
            x_left = self._x_to_px(self._x_um - half_w_um)
            x_right = self._x_to_px(self._x_um + half_w_um)
            z_top = self._z_to_px(0.0)
            z_bot = self._z_to_px(-self._well_depth_mm)
            well = QRectF(
                QPointF(min(x_left, x_right), z_top),
                QPointF(max(x_left, x_right), z_bot),
            ).normalized()
            # Fill (transparent darker)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(_qc('base', 230)))
            p.drawRoundedRect(well, s(3), s(3))
            # Edge
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(_qc('blue'), 1.2))
            p.drawRoundedRect(well, s(3), s(3))
            # Dashed bottom line for "well bottom"
            pen = QPen(_qc('blue', 200), 1.2, Qt.PenStyle.DashLine)
            p.setPen(pen)
            p.drawLine(QPointF(well.left() + s(2), well.bottom()),
                       QPointF(well.right() - s(2), well.bottom()))

    def _paint_z_references(self, p: QPainter, rect: QRectF) -> None:
        """Draw a dashed line + clickable badge for each set Z reference.

        Badges are right-anchored inside the plot rect and remember
        their hit rectangle so :meth:`mousePressEvent` can route clicks
        to :sig:`go_to_z_requested`.

        v7.5.x: the operator's custom Z locations are painted in the
        same pass (after the references) so they share the vertical
        badge collision-nudge and read as one family.
        """
        self._z_ref_hit_rects.clear()
        self._custom_hit_rects = []

        # Pre-compute badge sizing once per paint (consistent across
        # rows).
        font = p.font()
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(True)
        p.setFont(font)
        metrics = p.fontMetrics()
        pad_x = s(8)
        pad_y = s(3)
        badge_h = metrics.height() + pad_y * 2

        # Track previously-drawn badge centres so adjacent labels nudge
        # to avoid stacking on top of each other.
        placed: list[float] = []

        for key, label, color_key in self._Z_REF_META:
            value = self._z_refs.get(key)
            if value is None:
                continue
            # v7.9.1: operator-chosen subset (None = show all).
            if self._z_ref_visible is not None and key not in self._z_ref_visible:
                continue
            # v7.5.x: draw in the display frame; the hit rect below keeps the
            # raw zero-ref `value` so the go-to emit stays in the move frame.
            z_y = self._z_to_px(self._disp(value))
            # Skip if the line is outside the visible (zoomed) plot
            if z_y < rect.top() - 1 or z_y > rect.bottom() + 1:
                continue

            accent = _qc(color_key)
            # Subtle accent-tinted dashed line
            line_pen = QPen(QColor(accent), 1.0, Qt.PenStyle.DashLine)
            p.setPen(line_pen)
            p.drawLine(QPointF(rect.left() + s(4), z_y),
                       QPointF(rect.right() - s(4), z_y))

            # Badge text: short label + value (display frame; up = +)
            text = f"{label} · {self._disp(value):.2f} mm"
            text_w = metrics.horizontalAdvance(text)
            badge_w = text_w + pad_x * 2
            badge_x = rect.right() - badge_w - s(2)

            # Nudge the badge vertically if it would collide with one
            # we just drew above. Keeps overlapping labels readable
            # without overpainting.
            badge_cy = z_y
            for prev_cy in placed:
                if abs(badge_cy - prev_cy) < badge_h + s(2):
                    if badge_cy >= prev_cy:
                        badge_cy = prev_cy + badge_h + s(2)
                    else:
                        badge_cy = prev_cy - badge_h - s(2)
            # Clamp inside plot
            badge_cy = max(rect.top() + badge_h / 2,
                           min(rect.bottom() - badge_h / 2, badge_cy))
            placed.append(badge_cy)

            badge_rect = QRectF(
                badge_x, badge_cy - badge_h / 2, badge_w, badge_h)
            self._z_ref_hit_rects[key] = (badge_rect, value)

            # Leader line from the badge back to its Z line (when
            # nudged away from the row)
            if abs(badge_cy - z_y) > 1:
                leader_pen = QPen(accent, 1.0, Qt.PenStyle.DotLine)
                p.setPen(leader_pen)
                p.drawLine(QPointF(badge_rect.left() - s(2), badge_cy),
                           QPointF(badge_rect.left() - s(8), z_y))

            # Pill background + border
            p.setPen(QPen(accent, 1.0))
            p.setBrush(QBrush(_qc('crust', 230)))
            p.drawRoundedRect(badge_rect, badge_h / 2, badge_h / 2)
            # Label
            p.setPen(QPen(accent))
            p.drawText(badge_rect, Qt.AlignmentFlag.AlignCenter, text)

        # ── v7.5.x: custom Z locations (same badge family, ★ + ✕) ──
        accent = _qc('yellow')
        close_d = badge_h * 0.8
        for z_raw in self._custom_z:
            z_disp = self._disp(z_raw)
            z_y = self._z_to_px(z_disp)
            if z_y < rect.top() - 1 or z_y > rect.bottom() + 1:
                continue

            line_pen = QPen(QColor(accent), 1.0, Qt.PenStyle.DashLine)
            p.setPen(line_pen)
            p.drawLine(QPointF(rect.left() + s(4), z_y),
                       QPointF(rect.right() - s(4), z_y))

            text = f"★ {z_disp:+.2f} mm"
            text_w = metrics.horizontalAdvance(text)
            badge_w = text_w + pad_x * 2
            badge_x = rect.right() - badge_w - s(2)

            # Same vertical collision-nudge as the Z-ref badges.
            badge_cy = z_y
            for prev_cy in placed:
                if abs(badge_cy - prev_cy) < badge_h + s(2):
                    if badge_cy >= prev_cy:
                        badge_cy = prev_cy + badge_h + s(2)
                    else:
                        badge_cy = prev_cy - badge_h - s(2)
            badge_cy = max(rect.top() + badge_h / 2,
                           min(rect.bottom() - badge_h / 2, badge_cy))
            placed.append(badge_cy)

            badge_rect = QRectF(
                badge_x, badge_cy - badge_h / 2, badge_w, badge_h)
            close_rect = QRectF(
                badge_rect.left() - close_d - s(4),
                badge_cy - close_d / 2, close_d, close_d)
            self._custom_hit_rects.append(
                (badge_rect, close_rect, float(z_raw)))

            if abs(badge_cy - z_y) > 1:
                leader_pen = QPen(accent, 1.0, Qt.PenStyle.DotLine)
                p.setPen(leader_pen)
                p.drawLine(QPointF(close_rect.left() - s(2), badge_cy),
                           QPointF(close_rect.left() - s(8), z_y))

            # ✕ remove button (left of the badge)
            p.setPen(QPen(_qc('red'), 1.0))
            p.setBrush(QBrush(_qc('crust', 230)))
            p.drawEllipse(close_rect)
            close_font = p.font()
            close_font.setPointSizeF(scaled_font_size(7))
            close_font.setBold(True)
            p.setFont(close_font)
            p.setPen(QPen(_qc('red')))
            p.drawText(close_rect, Qt.AlignmentFlag.AlignCenter, "✕")
            p.setFont(font)

            # ★ badge pill (click → go to this Z)
            p.setPen(QPen(accent, 1.0))
            p.setBrush(QBrush(_qc('crust', 230)))
            p.drawRoundedRect(badge_rect, badge_h / 2, badge_h / 2)
            p.setPen(QPen(accent))
            p.drawText(badge_rect, Qt.AlignmentFlag.AlignCenter, text)

    def _paint_drag_ghost(self, p: QPainter, rect: QRectF) -> None:
        """Floating readout while the operator drags on the Z axis:
        a dashed line at the cursor Z + a tag with the location in mm
        (display frame). Painted last so it rides above everything."""
        if not self._drag_active or self._drag_z_disp is None:
            return
        z_y = self._z_to_px(self._drag_z_disp)
        pen = QPen(_qc('yellow'), 1.4, Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.drawLine(QPointF(rect.left() + s(2), z_y),
                   QPointF(rect.right() - s(2), z_y))

        # Tag pill just above the line, left-ish so it doesn't fight
        # the right-anchored badges. Flips below near the top edge.
        text = f"Z {self._drag_z_disp:+.3f} mm — release to tag"
        font = p.font()
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(True)
        p.setFont(font)
        metrics = p.fontMetrics()
        pill_h = metrics.height() + s(3) * 2
        tag_y = z_y - pill_h - s(4)
        if tag_y < rect.top() + s(2):
            tag_y = z_y + s(4)
        self._draw_pill(
            p,
            QPointF(rect.left() + s(26), tag_y),
            text,
            _qc('yellow'),
            _qc('crust', 230),
        )

    def _paint_needle(self, p: QPainter, rect: QRectF) -> None:
        """Render the current Z as a red horizontal line spanning the plot.

        A small inward-pointing triangle on each side marks the line as
        the current Z; a tiny tick at the needle's X position shows
        where the tip is in X. This replaces the previous tapered
        needle drawing — much cleaner and easier to read against the
        plate.
        """
        if self._z_mm is None:
            return

        z_y = self._z_to_px(self._disp(self._z_mm))
        if z_y < rect.top() or z_y > rect.bottom():
            return

        # Red horizontal line spanning the plot area
        pen = QPen(_qc('red'), 1.8)
        p.setPen(pen)
        p.drawLine(QPointF(rect.left() + s(2), z_y),
                   QPointF(rect.right() - s(2), z_y))

        # Inward arrow caps so the line reads as "current Z"
        cap_h = s(4)
        cap_w = s(6)
        p.setBrush(QBrush(_qc('red')))
        p.setPen(Qt.PenStyle.NoPen)
        left_cap = QPolygonF([
            QPointF(rect.left() + s(2), z_y - cap_h),
            QPointF(rect.left() + s(2) + cap_w, z_y),
            QPointF(rect.left() + s(2), z_y + cap_h),
        ])
        right_cap = QPolygonF([
            QPointF(rect.right() - s(2), z_y - cap_h),
            QPointF(rect.right() - s(2) - cap_w, z_y),
            QPointF(rect.right() - s(2), z_y + cap_h),
        ])
        p.drawPolygon(left_cap)
        p.drawPolygon(right_cap)

        # Tip indicator: small square at the X position
        if self._x_um is not None:
            tip_x = self._x_to_px(self._x_um)
            if rect.left() < tip_x < rect.right():
                marker = s(5)
                p.setBrush(QBrush(_qc('red')))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawRect(QRectF(tip_x - marker / 2,
                                  z_y - marker / 2,
                                  marker, marker))

    # ── Bottom readout (two lines: primary X/Z + secondary Δ info) ──

    def _paint_readout(self, p: QPainter, rect: QRectF) -> None:
        band_top = rect.bottom() + s(4)
        # Primary line — large X / Z
        line1 = QRectF(rect.left(), band_top, rect.width(), s(18))
        font = p.font()
        font.setPointSizeF(scaled_font_size(10))
        font.setBold(True)
        p.setFont(font)
        # v7.5.x: readout in the display frame (up = + when sign is -1).
        disp_z = self._disp(self._z_mm) if self._z_mm is not None else None
        if self._x_um is None or disp_z is None:
            text = "X: —   Z: —"
        else:
            text = (f"X: {self._x_um / 1000.0:+.3f} mm    "
                    f"Z: {disp_z:+.3f} mm")
        p.setPen(QPen(_qc('text')))
        p.drawText(
            line1,
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            text,
        )

        # Secondary line — subtle Δ info + Safe Z
        line2 = QRectF(rect.left(), band_top + s(18),
                       rect.width(), s(16))
        parts: list[tuple[str, QColor]] = []
        if disp_z is not None:
            color = (_qc('green') if disp_z > 0
                     else (_qc('red') if disp_z < 0 else _qc('subtext0')))
            parts.append((f"Δ plate {disp_z:+.2f} mm", color))
        if self._safe_z_mm is not None:
            parts.append(
                (f"safe Z {self._disp(self._safe_z_mm):.2f} mm", _qc('green')))
        if (self._well_depth_mm is not None
                and disp_z is not None):
            # In the display frame the well bottom sits at -well_depth (below
            # the plate); remaining travel = disp_z - (-well_depth).
            depth_remaining = disp_z - (-self._well_depth_mm)
            parts.append(
                (f"Δ well {depth_remaining:+.2f} mm", _qc('blue')))

        if parts:
            font.setPointSizeF(scaled_font_size(8))
            font.setBold(False)
            p.setFont(font)
            metrics = p.fontMetrics()
            sep = "  ·  "
            sep_w = metrics.horizontalAdvance(sep)
            x = line2.left()
            sep_color = _qc('surface2')
            for i, (text, color) in enumerate(parts):
                tw = metrics.horizontalAdvance(text)
                if i > 0:
                    p.setPen(QPen(sep_color))
                    p.drawText(
                        QRectF(x, line2.top(), sep_w, line2.height()),
                        int(Qt.AlignmentFlag.AlignLeft
                            | Qt.AlignmentFlag.AlignVCenter),
                        sep,
                    )
                    x += sep_w
                p.setPen(QPen(color))
                p.drawText(
                    QRectF(x, line2.top(), tw + s(4), line2.height()),
                    int(Qt.AlignmentFlag.AlignLeft
                        | Qt.AlignmentFlag.AlignVCenter),
                    text,
                )
                x += tw

    # ── Zoom widget styling + sync ────────────────────────────────

    def _style_zoom_widgets(self) -> None:
        btn_style = (
            f"QPushButton#xzZoomBtn {{"
            f"  background-color: {COLORS['surface0']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(4)};"
            f"  font-weight: 700;"
            f"  font-size: {sf(11)}pt;"
            f"  min-height: 0px;"
            f"  padding: 0px;"
            f"}}"
            f"QPushButton#xzZoomBtn:hover {{"
            f"  background-color: {COLORS['surface1']};"
            f"  border-color: {COLORS['mauve']};"
            f"  color: {COLORS['mauve']};"
            f"}}"
            f"QPushButton#xzZoomBtn:pressed {{"
            f"  background-color: {COLORS['mauve']};"
            f"  color: {COLORS['crust']};"
            f"}}"
            f"QPushButton#xzZoomBtn:disabled {{"
            f"  color: {COLORS['overlay0']};"
            f"  border-color: {COLORS['surface0']};"
            f"  background-color: {COLORS['mantle']};"
            f"}}"
        )
        self._btn_zoom_in.setStyleSheet(btn_style)
        self._btn_zoom_out.setStyleSheet(btn_style)
        self._scrollbar.setStyleSheet(
            f"QScrollBar#xzZoomScroll:vertical {{"
            f"  background-color: {COLORS['mantle']};"
            f"  border: 1px solid {COLORS['surface0']};"
            f"  border-radius: {sp(3)};"
            f"  width: {s(12)}px;"
            f"  margin: 0;"
            f"}}"
            f"QScrollBar#xzZoomScroll::handle:vertical {{"
            f"  background-color: {COLORS['mauve']};"
            f"  border-radius: {sp(3)};"
            f"  min-height: {s(18)}px;"
            f"}}"
            f"QScrollBar#xzZoomScroll::handle:vertical:hover {{"
            f"  background-color: {COLORS['pink']};"
            f"}}"
            f"QScrollBar#xzZoomScroll::add-line:vertical,"
            f"QScrollBar#xzZoomScroll::sub-line:vertical {{"
            f"  height: 0; width: 0;"
            f"  background: transparent;"
            f"}}"
            f"QScrollBar#xzZoomScroll::add-page:vertical,"
            f"QScrollBar#xzZoomScroll::sub-page:vertical {{"
            f"  background: transparent;"
            f"}}"
            f"QScrollBar#xzZoomScroll:disabled {{"
            f"  background-color: {COLORS['mantle']};"
            f"}}"
            f"QScrollBar#xzZoomScroll::handle:vertical:disabled {{"
            f"  background-color: {COLORS['surface1']};"
            f"}}"
        )

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._position_zoom_widgets()

    def _position_zoom_widgets(self) -> None:
        rect = self._content_rect()
        strip_w = self._zoom_strip_width()
        btn_size = s(18)
        scroll_w = s(12)

        strip_x = int(rect.right()) + s(4)
        # Buttons centered in the strip horizontally
        btn_x = strip_x + (strip_w - btn_size) // 2
        scroll_x = strip_x + (strip_w - scroll_w) // 2

        self._btn_zoom_in.setGeometry(
            btn_x, int(rect.top()), btn_size, btn_size)
        self._btn_zoom_out.setGeometry(
            btn_x, int(rect.bottom()) - btn_size,
            btn_size, btn_size)
        self._scrollbar.setGeometry(
            scroll_x,
            int(rect.top()) + btn_size + s(3),
            scroll_w,
            max(s(20),
                int(rect.height()) - 2 * (btn_size + s(3))))

    def _on_zoom_in(self) -> None:
        if self._safety_limits is None:
            return
        if self._z_zoom_center_mm is None:
            # Default zoom focus is the plate surface
            self._z_zoom_center_mm = 0.0
        new_zoom = min(self._z_zoom * 1.5, 30.0)
        self._z_zoom = new_zoom
        self._sync_scrollbar()
        self.update()

    def _on_zoom_out(self) -> None:
        new_zoom = max(self._z_zoom / 1.5, 1.0)
        self._z_zoom = new_zoom
        if new_zoom <= 1.0:
            self._z_zoom = 1.0
            self._z_zoom_center_mm = None
        self._sync_scrollbar()
        self.update()

    def _on_scrollbar_changed(self, sb_value: int) -> None:
        if self._scrollbar_updating or self._safety_limits is None:
            return
        if self._z_zoom <= 1.0:
            return
        sb = self._z_safety_bounds()
        if sb is None:
            return
        z_min, z_max = sb  # zero-ref frame (matches _z_zoom_center_mm)
        total = z_max - z_min
        view_range = total / self._z_zoom
        half = view_range / 2.0
        center_min = z_min + half
        center_max = z_max - half
        if center_max <= center_min:
            return
        # Scrollbar units = 0.01 mm. value 0 → top (center_max).
        center = center_max - sb_value / 100.0
        self._z_zoom_center_mm = max(center_min, min(center_max, center))
        self.update()

    def _sync_scrollbar(self) -> None:
        """Push current zoom state into the scrollbar + button enabled
        flags. Set ``_scrollbar_updating`` while we mutate it so the
        ``valueChanged`` callback short-circuits."""
        self._scrollbar_updating = True
        try:
            if (self._safety_limits is None
                    or self._z_zoom <= 1.0):
                self._scrollbar.setEnabled(False)
                self._scrollbar.setRange(0, 0)
                self._scrollbar.setValue(0)
            else:
                z_min, z_max = self._z_safety_bounds()  # zero-ref frame
                total = z_max - z_min
                view_range = total / self._z_zoom
                half = view_range / 2.0
                center_min = z_min + half
                center_max = z_max - half
                if center_max <= center_min:
                    self._scrollbar.setEnabled(False)
                    self._scrollbar.setRange(0, 0)
                    self._scrollbar.setValue(0)
                else:
                    sb_max = max(1, int((center_max - center_min) * 100))
                    self._scrollbar.setEnabled(True)
                    self._scrollbar.setRange(0, sb_max)
                    self._scrollbar.setPageStep(
                        max(1, int(view_range * 100)))
                    self._scrollbar.setSingleStep(
                        max(1, int(view_range * 10)))
                    center = (self._z_zoom_center_mm
                              if self._z_zoom_center_mm is not None
                              else 0.0)
                    center = max(center_min, min(center_max, center))
                    self._scrollbar.setValue(
                        int((center_max - center) * 100))
        finally:
            self._scrollbar_updating = False
        # Button enabled states
        self._btn_zoom_in.setEnabled(self._z_zoom < 30.0)
        self._btn_zoom_out.setEnabled(self._z_zoom > 1.0)

    # ── Zoom badge ─────────────────────────────────────────────────

    def _paint_zoom_badge(self, p: QPainter, rect: QRectF) -> None:
        """A small badge in the top-left showing the current Z zoom.

        When zoomed in, clicking the badge resets to 1×. The text reads
        `1×` at default zoom and `4.2×` etc. when zoomed.
        """
        text = (f"Z · {self._z_zoom:g}×" if self._z_zoom <= 1.0
                else f"Z · {self._z_zoom:.1f}×  (click to reset)")
        font = p.font()
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(True)
        p.setFont(font)
        metrics = p.fontMetrics()
        text_w = metrics.horizontalAdvance(text)
        pad_x = s(8)
        pad_y = s(3)
        badge_w = text_w + pad_x * 2
        badge_h = metrics.height() + pad_y * 2
        badge_rect = QRectF(rect.left() + s(8), rect.top() + s(8),
                            badge_w, badge_h)
        self._reset_zoom_rect = badge_rect

        if self._z_zoom <= 1.0:
            text_color = _qc('subtext0')
            border_color = _qc('surface2')
        else:
            text_color = _qc('mauve')
            border_color = _qc('mauve')
        p.setPen(QPen(border_color, 1.0))
        p.setBrush(QBrush(_qc('crust', 200)))
        p.drawRoundedRect(badge_rect, badge_h / 2, badge_h / 2)
        p.setPen(QPen(text_color))
        p.drawText(badge_rect, Qt.AlignmentFlag.AlignCenter, text)

    # ── Mouse / wheel handling ─────────────────────────────────────

    def wheelEvent(self, event: QWheelEvent) -> None:
        """Scroll wheel zooms the Z axis around the cursor."""
        if self._safety_limits is None:
            return
        delta = event.angleDelta().y()
        if delta == 0:
            return
        rect = self._content_rect()
        # Only zoom when the cursor is inside the plot area
        pos = event.position()
        if not rect.contains(pos):
            return
        # Cursor Z (in mm) given current zoom — use it as the new center
        bounds = self._z_bounds_mm()
        if bounds is None:
            return
        z_min, z_max = bounds
        frac = (pos.y() - rect.top()) / max(1.0, rect.height())
        cursor_z = z_max - frac * (z_max - z_min)
        # Step the zoom factor
        if delta > 0:
            new_zoom = min(self._z_zoom * 1.4, 30.0)
        else:
            new_zoom = max(self._z_zoom / 1.4, 1.0)
        self._z_zoom = new_zoom
        self._z_zoom_center_mm = cursor_z
        if self._z_zoom <= 1.0:
            self._z_zoom = 1.0
            self._z_zoom_center_mm = None
        self._sync_scrollbar()
        self.update()
        event.accept()

    def _drag_threshold_px(self) -> float:
        return float(s(4))

    def _current_line_hit_px(self) -> float:
        return float(s(6))

    def mouseMoveEvent(self, event) -> None:
        pos = event.position()

        # v7.5.x: live drag-to-tag — update the floating readout.
        if self._drag_pending or self._drag_active:
            if (self._drag_pending and self._drag_press_pos is not None
                    and (pos - self._drag_press_pos).manhattanLength()
                    > self._drag_threshold_px()):
                self._drag_pending = False
                self._drag_active = True
            if self._drag_active:
                self._drag_z_disp = self._px_to_z_disp(pos.y())
                self.setCursor(Qt.SizeVerCursor)
                self.update()
            event.accept()
            return

        # Use a pointing-hand cursor over the clickable badges
        over_clickable = False
        if (self._reset_zoom_rect is not None
                and self._reset_zoom_rect.contains(pos)
                and self._z_zoom > 1.0):
            over_clickable = True
        else:
            for rect, _ in self._z_ref_hit_rects.values():
                if rect.contains(pos):
                    over_clickable = True
                    break
            if not over_clickable:
                for badge_rect, close_rect, _ in self._custom_hit_rects:
                    if badge_rect.contains(pos) or close_rect.contains(pos):
                        over_clickable = True
                        break
        if over_clickable:
            self.setCursor(Qt.PointingHandCursor)
        elif (self._custom_z_enabled
                and self._safety_limits is not None
                and self._content_rect().contains(pos)):
            # Hint that the plot is draggable / clickable for tagging
            self.setCursor(Qt.CrossCursor)
        else:
            self.setCursor(Qt.ArrowCursor)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event) -> None:
        pos = event.position()
        # Click the zoom badge to reset
        if (self._reset_zoom_rect is not None
                and self._reset_zoom_rect.contains(pos)):
            if self._z_zoom > 1.0:
                self._z_zoom = 1.0
                self._z_zoom_center_mm = None
                self._sync_scrollbar()
                self.update()
                event.accept()
                return
        # v7.5.x: custom-location badges — ✕ removes; left-click on the
        # ★ badge drives Z there; right-click on the badge also removes.
        for badge_rect, close_rect, z_raw in list(self._custom_hit_rects):
            if close_rect.contains(pos):
                self.remove_custom_z(z_raw)
                event.accept()
                return
            if badge_rect.contains(pos):
                if event.button() == Qt.MouseButton.RightButton:
                    self.remove_custom_z(z_raw)
                else:
                    self.go_to_z_requested.emit(float(z_raw))
                event.accept()
                return
        # Click one of the Z-reference badges to jump there
        for key, (rect, z_value) in self._z_ref_hit_rects.items():
            if rect.contains(pos):
                self.go_to_z_requested.emit(float(z_value))
                event.accept()
                return
        # v7.5.x: begin a drag-to-tag gesture inside the plot. Becomes
        # a live drag once the cursor moves past the threshold; a plain
        # click is resolved on release (current-Z line → tag current).
        if (self._custom_z_enabled
                and event.button() == Qt.MouseButton.LeftButton
                and self._safety_limits is not None
                and self._content_rect().contains(pos)):
            self._drag_pending = True
            self._drag_active = False
            self._drag_press_pos = QPointF(pos)
            self._drag_z_disp = self._px_to_z_disp(pos.y())
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if not (self._drag_pending or self._drag_active):
            super().mouseReleaseEvent(event)
            return
        was_drag = self._drag_active
        press_pos = self._drag_press_pos
        drag_disp = self._drag_z_disp
        self._drag_pending = False
        self._drag_active = False
        self._drag_press_pos = None
        self._drag_z_disp = None

        if was_drag and drag_disp is not None:
            # Dragged tag → commit at the dragged Z. Display → raw
            # zero-ref: raw = disp × sign (sign ∈ {±1}).
            self.add_custom_z(drag_disp * self._z_disp_sign)
        elif press_pos is not None and self._z_mm is not None:
            # Plain click — only meaningful ON the current-Z line:
            # tag the needle's current location.
            line_y = self._z_to_px(self._disp(self._z_mm))
            if abs(press_pos.y() - line_y) <= self._current_line_hit_px():
                self.add_custom_z(self._z_mm)
        self.setCursor(Qt.ArrowCursor)
        self.update()
        event.accept()

    # ── Pill helper ───────────────────────────────────────────────

    def _draw_pill(self, p: QPainter, top_left: QPointF, text: str,
                   text_color: QColor, bg_color: QColor) -> None:
        font = p.font()
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(True)
        p.setFont(font)
        metrics = p.fontMetrics()
        text_w = metrics.horizontalAdvance(text)
        pad_x = s(6)
        pad_y = s(2)
        pill_w = text_w + pad_x * 2
        pill_h = metrics.height() + pad_y * 2
        rect = QRectF(top_left.x(), top_left.y(), pill_w, pill_h)
        p.setPen(QPen(text_color, 1.0))
        p.setBrush(QBrush(bg_color))
        p.drawRoundedRect(rect, pill_h / 2, pill_h / 2)
        p.setPen(QPen(text_color))
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)
