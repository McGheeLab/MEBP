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

from PySide6.QtCore import QPointF, QRectF, QSize, QSizeF, Qt, Signal
from PySide6.QtGui import (
    QBrush, QColor, QFont, QImage, QMouseEvent, QPainter, QPen, QPixmap,
    QTransform,
)
from PySide6.QtWidgets import QMenu, QSizePolicy, QToolButton, QWidget

from gui.scaling import s, scaled_font_size
from gui.styles import COLORS
from gui.widgets.icons import icon


def pixmap_from_bgr(bgr) -> QPixmap | None:
    """Convert a numpy BGR (or grayscale) uint8 image to a QPixmap.

    Shared by the pages that push a stitched plate mosaic into
    :meth:`JogWorkspaceView.set_mosaic_overlay`. ``.copy()`` detaches the
    QImage from the numpy buffer so the pixmap survives the array being freed.
    """
    if bgr is None:
        return None
    try:
        import numpy as np
        arr = np.ascontiguousarray(bgr)
        h, w = arr.shape[:2]
        if arr.ndim == 2:
            img = QImage(arr.data, w, h, w, QImage.Format.Format_Grayscale8)
        else:
            img = QImage(arr.data, w, h, 3 * w, QImage.Format.Format_BGR888)
        return QPixmap.fromImage(img.copy())
    except Exception:
        return None

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

    # v7.5.x: rotate the top-down rendering 180°. The current Prior II stage
    # has its origin at the bottom-right with +X/+Y toward the top-left, so an
    # unflipped canvas (+X→right, +Y→down) draws the plate upside-down relative
    # to the operator's physical view. Reflecting both axes about the envelope
    # centre matches the physical orientation. Applied in BOTH the forward and
    # inverse coordinate maps, so click targeting stays correct.
    #
    # This is now driven by the per-machine ``plate_flip_180`` setting (single
    # source of truth on ``StageController`` — see DEFAULT_PLATE_FLIP_180); the
    # owning page pushes it via :meth:`set_plate_flip_180`. The class constant
    # below is only the construction-time default until the page pushes the
    # live value.
    _FLIP_DISPLAY_180 = True

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(s(280), s(220))
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)

        # State
        self._safety_limits = None
        # v7.5.x: per-machine 180° display flip (driven by plate_flip_180).
        # Defaults to the class constant until the owning page pushes the live
        # per-machine value via set_plate_flip_180().
        self._flip_180: bool = self._FLIP_DISPLAY_180
        # v7.5.x: the XY envelope is stored ABSOLUTE stage µm, but this view is
        # fed zero-referenced needle/well positions. Subtract the current zero
        # reference from the envelope bounds so both share the zero-ref frame.
        self._zero_off: tuple[float, float] = (0.0, 0.0)
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

        # v7.5.x: persistent reference markers (e.g. taught well centres),
        # zero-ref µm, drawn as a distinct ⊕ so the operator can confirm
        # registration against the plate/wells.
        self._ref_markers: dict[str, tuple[float, float]] = {}

        # v7.5.x: optional stitched full-plate mosaic drawn UNDER the wells as
        # a background overlay. Extent is ABSOLUTE stage µm (shifted by
        # _zero_off at paint time, like the envelope). Toggled via
        # set_mosaic_visible(); built by the Plate Location mosaic scan.
        self._mosaic_pixmap: QPixmap | None = None
        self._mosaic_extent_abs: tuple[float, float, float, float] | None = None
        self._mosaic_visible: bool = False
        self._mosaic_opacity: float = 0.55
        # v7.5.x: cache the scaled (+180°-rotated) overlay so paintEvent doesn't
        # re-scale the full-res mosaic (~18 MB) on every repaint (needle moves,
        # hovers). Rebuilt only when the on-screen size / flip / source change;
        # keyed by (w, h, flip, source-id). Fixes the post-mosaic-build lag.
        self._mosaic_scaled_cache: QPixmap | None = None
        self._mosaic_cache_key: tuple | None = None
        # v7.5.x: the idealized well grid can be hidden so the operator can view
        # the mosaic on its own. Combined with _mosaic_visible this gives the
        # three plate-view modes: ideal-well / mosaic / mosaic-overlaid-on-well
        # (see set_plate_display_mode()).
        self._wells_visible: bool = True
        self._plate_display_mode: str = "well"
        # v7.5.x: live manual-alignment nudge (µm) added to the overlay extent
        # so the operator can slide the mosaic onto the well grid by eye.
        self._mosaic_user_shift: tuple[float, float] = (0.0, 0.0)
        # v7.5.x: SEPARATE multi-channel fluorescence overlay (independent of the
        # brightfield plate mosaic above). Built by the Fluorescence Mosaic
        # workflow + persisted per (plate, well); any workflow can show it as a
        # registered background. Extent is ABSOLUTE stage µm (shifted by
        # _zero_off at paint, like the mosaic). Toggled via set_fluor_visible().
        self._fluor_pixmap: QPixmap | None = None
        self._fluor_extent_abs: tuple[float, float, float, float] | None = None
        self._fluor_visible: bool = False
        self._fluor_opacity: float = 0.75
        # v7.5.x: zoom + pan so the operator can magnify small features (e.g.
        # spots seen in the mosaic). Enabled by default on every page. The wheel
        # zooms about the cursor; +/- buttons zoom about centre; PAN happens on
        # Shift+left-drag OR when the hand tool is toggled on (so a plain left-
        # click still travels / snaps / teaches). _zoom multiplies the
        # fit-to-envelope scale; _pan is an extra pixel offset.
        self._zoom_enabled: bool = True
        self._zoom: float = 1.0
        self._pan: list[float] = [0.0, 0.0]
        self._panning: bool = False
        self._pan_anchor_px: tuple[float, float] | None = None
        self._pan_anchor_val: tuple[float, float] | None = None
        # Hand/pan tool: when active a plain left-drag pans (no Shift needed).
        self._pan_tool_active: bool = False
        self._build_zoom_controls()

    # ── Public API ─────────────────────────────────────────────────

    _ZOOM_MIN = 1.0
    _ZOOM_MAX = 60.0

    def set_zoom_enabled(self, enabled: bool) -> None:
        """Enable wheel-zoom + pan + the overlay +/-/hand controls. On by
        default; disable to lock a view at fit-to-envelope."""
        self._zoom_enabled = bool(enabled)
        if not self._zoom_enabled:
            self._pan_tool_active = False
            if hasattr(self, "_btn_pan"):
                self._btn_pan.setChecked(False)
            self.reset_view()
        self._update_zoom_controls_visibility()
        self._update_pan_cursor()

    # ── Overlay zoom/pan controls (+/-/hand) ───────────────────────

    def _build_zoom_controls(self) -> None:
        """Create the floating +/-/hand buttons over the canvas (top-right).
        Present on every page that hosts this view, so zoom/pan is always
        available."""
        r = s(6)
        c = _qc('surface0')
        bg = f"rgba({c.red()},{c.green()},{c.blue()},0.86)"
        sheet = (
            "QToolButton {"
            f"  background: {bg};"
            f"  border: 1px solid {COLORS['surface2']};"
            f"  border-radius: {r}px;"
            "}"
            f"QToolButton:hover {{ background: {COLORS['surface1']}; }}"
            f"QToolButton:checked {{ background: {COLORS['mauve']};"
            f"  border-color: {COLORS['mauve']}; }}"
        )

        def _mk(name: str, tip: str, checkable: bool = False) -> QToolButton:
            b = QToolButton(self)
            b.setIcon(icon(name, px=s(15)))
            b.setIconSize(QSize(s(15), s(15)))
            b.setToolTip(tip)
            b.setCheckable(checkable)
            b.setCursor(Qt.PointingHandCursor)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.setFixedSize(s(26), s(26))
            b.setStyleSheet(sheet)
            return b

        self._btn_zoom_in = _mk("plus", "Zoom in (mouse wheel up)")
        self._btn_zoom_out = _mk("minus", "Zoom out (mouse wheel down)")
        self._btn_pan = _mk(
            "hand",
            "Pan tool — drag to pan (or hold Shift + drag any time). "
            "Click again to return to click-to-move.",
            checkable=True)
        self._btn_zoom_in.clicked.connect(lambda: self._zoom_about_center(1.6))
        self._btn_zoom_out.clicked.connect(
            lambda: self._zoom_about_center(1.0 / 1.6))
        self._btn_pan.toggled.connect(self._on_pan_tool_toggled)
        self._update_zoom_controls_visibility()
        self._layout_zoom_controls()

    def _update_zoom_controls_visibility(self) -> None:
        for name in ("_btn_zoom_in", "_btn_zoom_out", "_btn_pan"):
            b = getattr(self, name, None)
            if b is not None:
                b.setVisible(self._zoom_enabled)

    def _layout_zoom_controls(self) -> None:
        if not hasattr(self, "_btn_zoom_in"):
            return
        m = s(8)
        gap = s(4)
        bw = self._btn_zoom_in.width()
        x = max(0, self.width() - m - bw)
        y = s(10)
        for b in (self._btn_zoom_in, self._btn_zoom_out, self._btn_pan):
            b.move(int(x), int(y))
            b.raise_()
            y += b.height() + gap

    def _on_pan_tool_toggled(self, checked: bool) -> None:
        self._pan_tool_active = bool(checked)
        self._update_pan_cursor()

    def _update_pan_cursor(self) -> None:
        if self._zoom_enabled and self._pan_tool_active:
            self.setCursor(Qt.OpenHandCursor)
        else:
            self.setCursor(Qt.CrossCursor)

    def _zoom_about_center(self, factor: float) -> None:
        """Zoom by ``factor`` about the centre of the drawn content (used by the
        +/- buttons; the wheel zooms about the cursor instead)."""
        if not self._zoom_enabled:
            return
        rect = self._content_rect()
        cx, cy = rect.center().x(), rect.center().y()
        ux, uy = self._px_to_um(cx, cy)
        new_zoom = max(self._ZOOM_MIN, min(self._ZOOM_MAX, self._zoom * factor))
        if new_zoom == self._zoom:
            return
        self._zoom = new_zoom
        if abs(new_zoom - self._ZOOM_MIN) < 1e-6:
            self._pan = [0.0, 0.0]            # snap back to fit at min zoom
        else:
            npx = self._um_to_px(ux, uy)      # keep centre point fixed
            self._pan[0] += cx - npx.x()
            self._pan[1] += cy - npx.y()
        self.update()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._layout_zoom_controls()

    def set_zoom(self, zoom: float) -> None:
        z = max(self._ZOOM_MIN, min(self._ZOOM_MAX, float(zoom)))
        if z != self._zoom:
            self._zoom = z
            self.update()

    def zoom(self) -> float:
        return self._zoom

    def reset_view(self) -> None:
        """Back to fit-to-envelope (zoom 1, no pan)."""
        self._zoom = 1.0
        self._pan = [0.0, 0.0]
        self.update()

    def set_safety_limits(self, limits) -> None:
        self._safety_limits = limits
        self.update()

    def set_plate_flip_180(self, flip: bool) -> None:
        """v7.5.x: per-machine 180° display flip (from StageController
        ``plate_flip_180``). Pushed by the owning page so every plate view
        agrees on the orientation convention (A1 top-left / stage 0,0
        bottom-right)."""
        flip = bool(flip)
        if flip != self._flip_180:
            self._flip_180 = flip
            self.update()

    def set_zero_offset(self, zero_x_um: float, zero_y_um: float) -> None:
        """v7.5.x: current zero reference (absolute stage µm). The absolute XY
        envelope is shifted by this offset for display so it lines up with the
        zero-referenced needle/well positions this view is fed."""
        off = (float(zero_x_um), float(zero_y_um))
        if off != self._zero_off:
            self._zero_off = off
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

    def set_reference_markers(
        self, markers: dict[str, tuple[float, float]] | None
    ) -> None:
        """Persistent reference points (zero-ref µm) drawn as a distinct ⊕ —
        e.g. the taught well centres, so the operator can verify the plate map
        registers against them."""
        self._ref_markers = dict(markers) if markers else {}
        self.update()

    def set_mosaic_overlay(
        self,
        pixmap: QPixmap | None,
        extent_abs_um: tuple[float, float, float, float] | None,
    ) -> None:
        """Set (or clear) the stitched full-plate mosaic background overlay.

        ``pixmap`` is the composite (use :func:`pixmap_from_bgr`); ``extent_abs_um``
        is its world extent ``(min_x, min_y, max_x, max_y)`` in **absolute stage
        µm** (``MosaicBuilder.canvas_extent_um``). Pass ``None`` to clear.
        Visibility is controlled separately by :meth:`set_mosaic_visible`.
        """
        self._mosaic_pixmap = pixmap
        if (extent_abs_um is not None and len(extent_abs_um) == 4
                and pixmap is not None):
            self._mosaic_extent_abs = tuple(float(v) for v in extent_abs_um)
        else:
            self._mosaic_extent_abs = None
            self._mosaic_pixmap = None
        # Invalidate the scaled-overlay cache (a new/cleared source).
        self._mosaic_scaled_cache = None
        self._mosaic_cache_key = None
        # A fresh overlay starts un-nudged; the page seeds any stored shift.
        self._mosaic_user_shift = (0.0, 0.0)
        self.update()

    def set_mosaic_visible(self, visible: bool) -> None:
        self._mosaic_visible = bool(visible)
        self.update()

    def mosaic_output_rotation(self) -> int:
        """The DISPLAY-ONLY whole-mosaic output rotation in degrees (0/90/180/270).

        v7.5.x (operator: *"we need the ability to rotate the entire mosaic to
        ensure the output of the mosaic is the correct way up and down"*).

        ⚠ Display only, by design. Mosaic tiles are placed at trusted raw stage
        positions, and every consumer back-projects
        ``stage = (extent - shift) + px/scale`` to derive well centres for
        MOTION. Rotating the stored composite or its extent would reintroduce the
        class of bug that once drove the stage millimetres off target, so this is
        applied at paint time and nowhere else.
        """
        return int(getattr(self, "_mosaic_output_rotation", 0) or 0)

    def set_mosaic_output_rotation(self, rotation_deg: float) -> None:
        """Set the display-only whole-mosaic output rotation (snapped to 90°)."""
        try:
            quad = int(round(float(rotation_deg) / 90.0)) % 4 * 90
        except (TypeError, ValueError):
            quad = 0
        if quad == self.mosaic_output_rotation():
            return
        self._mosaic_output_rotation = quad
        # The cached scaled+rotated pixmap is keyed on this — drop it.
        self._mosaic_scaled_cache = None
        self._mosaic_cache_key = None
        self.update()

    def set_wells_visible(self, visible: bool) -> None:
        """Show/hide the idealized well grid (paint-only; snapping is
        unaffected)."""
        self._wells_visible = bool(visible)
        self.update()

    def set_plate_display_mode(self, mode: str) -> None:
        """Set the plate-view mode in one call. ``mode`` is one of:

        * ``"well"``    — idealized well grid only (mosaic hidden)
        * ``"mosaic"``  — stitched plate mosaic only (well grid hidden)
        * ``"overlay"`` — mosaic drawn under the idealized well grid

        Unknown values fall back to ``"well"``.
        """
        if mode not in ("well", "mosaic", "overlay"):
            mode = "well"
        # v7.5.x: "overlay" renders the wells as OUTLINE-ONLY circles over the
        # mosaic background (no plate body / fills) — the mosaic IS the plate.
        self._plate_display_mode = mode
        self._wells_visible = mode in ("well", "overlay")
        self._mosaic_visible = mode in ("mosaic", "overlay")
        self.update()

    def set_mosaic_shift(self, dx_um: float, dy_um: float) -> None:
        """Live manual-alignment nudge (µm) applied to the overlay extent — lets
        the operator slide the mosaic onto the well grid by eye. Does not rebuild
        the pixmap."""
        sh = (float(dx_um), float(dy_um))
        if sh != self._mosaic_user_shift:
            self._mosaic_user_shift = sh
            self.update()

    def mosaic_shift(self) -> tuple[float, float]:
        return self._mosaic_user_shift

    def set_mosaic_opacity(self, opacity: float) -> None:
        a = max(0.05, min(1.0, float(opacity)))
        if a != self._mosaic_opacity:
            self._mosaic_opacity = a
            self.update()

    def has_mosaic(self) -> bool:
        return self._mosaic_pixmap is not None and self._mosaic_extent_abs is not None

    # ── Fluorescence overlay (independent of the brightfield mosaic) ──

    def set_fluor_overlay(
        self,
        pixmap: QPixmap | None,
        extent_abs_um: tuple[float, float, float, float] | None,
    ) -> None:
        """Set (or clear) the multi-channel fluorescence overlay.

        ``pixmap`` is the blended false-colour image (use :func:`pixmap_from_bgr`
        on ``FluorescenceMosaicStore.composite_overlay``); ``extent_abs_um`` is
        its world extent ``(min_x, min_y, max_x, max_y)`` in **absolute stage µm**.
        Pass ``None`` to clear. Visibility is controlled by :meth:`set_fluor_visible`.
        """
        self._fluor_pixmap = pixmap
        if (extent_abs_um is not None and len(extent_abs_um) == 4
                and pixmap is not None):
            self._fluor_extent_abs = tuple(float(v) for v in extent_abs_um)
        else:
            self._fluor_extent_abs = None
            self._fluor_pixmap = None
        self.update()

    def set_fluor_visible(self, visible: bool) -> None:
        self._fluor_visible = bool(visible)
        self.update()

    def set_fluor_opacity(self, opacity: float) -> None:
        a = max(0.05, min(1.0, float(opacity)))
        if a != self._fluor_opacity:
            self._fluor_opacity = a
            self.update()

    def has_fluor(self) -> bool:
        return self._fluor_pixmap is not None and self._fluor_extent_abs is not None

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
        # v7.5.x: envelope bounds are absolute stage µm; subtract the zero
        # reference so they are in the same zero-ref frame as set_position().
        zx, zy = self._zero_off
        return (
            float(self._safety_limits.xy_min_x) - zx,
            float(self._safety_limits.xy_min_y) - zy,
            float(self._safety_limits.xy_max_x) - zx,
            float(self._safety_limits.xy_max_y) - zy,
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
        # Fit-to-envelope base scale, then the opt-in zoom multiplier + pan.
        k = min(sx, sy) * (self._zoom if self._zoom_enabled else 1.0)
        drawn_w = env_w * k
        drawn_h = env_h * k
        ox = rect.left() + (rect.width() - drawn_w) / 2.0 - x_min * k
        oy = rect.top() + (rect.height() - drawn_h) / 2.0 - y_min * k
        if self._zoom_enabled:
            ox += self._pan[0]
            oy += self._pan[1]
        return (k, ox, oy)

    def _apply_display_flip(
        self, x_um: float, y_um: float
    ) -> tuple[float, float]:
        """Reflect a zero-ref µm point about the envelope centre when the
        display is rotated 180° (see ``set_plate_flip_180``). The reflection is
        its own inverse, so applying it in both the forward (µm→px) and inverse
        (px→µm) maps rotates the whole render while keeping clicks accurate."""
        if not self._flip_180:
            return (x_um, y_um)
        env = self._envelope_bounds()
        if env is None:
            return (x_um, y_um)
        x_min, y_min, x_max, y_max = env
        return ((x_min + x_max) - x_um, (y_min + y_max) - y_um)

    def _um_to_px(self, x_um: float, y_um: float) -> QPointF:
        sc = self._scale()
        if sc is None:
            return QPointF(0, 0)
        k, ox, oy = sc
        fx, fy = self._apply_display_flip(x_um, y_um)
        return QPointF(fx * k + ox, fy * k + oy)

    def _px_to_um(self, px_x: float, px_y: float) -> tuple[float, float]:
        sc = self._scale()
        if sc is None:
            return (0.0, 0.0)
        k, ox, oy = sc
        return self._apply_display_flip((px_x - ox) / k, (px_y - oy) / k)

    # ── Snap logic ─────────────────────────────────────────────────

    def _well_radius_um(self, name: str | None = None) -> float:
        """Well radius in µm. v7.4.8: per-well when `name` is given (custom
        plates / flattened rosette sub-wells have mixed diameters); else a
        representative radius (uniform diameter, or the max for customs)."""
        if self._plate is None:
            return 0.0
        # v7.12: the per-well → plate-level → largest-well chain lives on
        # WellPlate. It used to be written out here and in three other places.
        try:
            return float(self._plate.well_diameter_of(name)) * 1000.0 / 2.0
        except Exception:                                  # pragma: no cover
            d = float(getattr(self._plate, "well_diameter", 0.0) or 0.0)
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

        if self._safety_limits is None:
            self._paint_placeholder(p)
            self._paint_toggle(p)
            p.end()
            return

        # Workspace content is clipped to the content rect so an off-envelope
        # well (e.g. from a coarse/bad calibration) can't bleed into the toggle
        # band at the top or spill outside the workspace box.
        p.save()
        p.setClipRect(self._content_rect())
        self._paint_envelope(p)
        self._paint_mosaic_overlay(p)
        self._paint_fluor_overlay(p)
        self._paint_plate_and_wells(p)
        self._paint_breadcrumbs(p)
        self._paint_reference_markers(p)
        self._paint_needle(p)
        self._paint_hover_ghost(p)
        p.restore()

        self._paint_readout(p)
        # v7.5.x: toggle painted LAST so the Free / Snap-to-well buttons are
        # never obscured by the plate or wells (they used to paint over a
        # toggle drawn first when a well rendered near the top-left corner).
        self._paint_toggle(p)

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

    # ── Mosaic overlay ─────────────────────────────────────────────

    def _paint_mosaic_overlay(self, p: QPainter) -> None:
        """Draw the stitched plate mosaic registered to its world extent.

        Extent is absolute stage µm → shifted into the zero-ref frame (like the
        envelope). Under the 180° display flip, the world-min corner maps to the
        screen-max corner, so the image is rotated 180° about the rect centre to
        register pixel(0,0) (= world min) onto its flipped screen position.
        """
        if (not self._mosaic_visible or self._mosaic_pixmap is None
                or self._mosaic_extent_abs is None):
            return
        zx, zy = self._zero_off
        ux, uy = self._mosaic_user_shift           # live manual-align nudge
        min_x, min_y, max_x, max_y = self._mosaic_extent_abs
        tl = self._um_to_px(min_x - zx + ux, min_y - zy + uy)
        br = self._um_to_px(max_x - zx + ux, max_y - zy + uy)
        rect = QRectF(tl, br).normalized()
        if rect.width() < 1 or rect.height() < 1:
            return
        # v7.5.x: scale (+180° rotate) ONCE into a cache, keyed by the cache size
        # + flip + source identity. The manual-align nudge only moves rect's
        # top-left (size is shift-invariant), so panning/needle motion reuse the
        # cache; only zoom / a new mosaic / flip change rebuild it — and past the
        # source-resolution cap below, not even zoom does, so paintEvent then
        # blits the pre-rendered pixmap with at most a cheap clipped magnify.
        tw = max(1, int(round(rect.width())))
        th = max(1, int(round(rect.height())))
        out_rot = self.mosaic_output_rotation()
        # Compose the plate-frame 180° display flip with the operator's
        # whole-mosaic output rotation. Both are DISPLAY-ONLY: the stored
        # composite and its extent are untouched, so the back-projection
        # every consumer uses to derive well centres for MOTION is unaffected.
        total = ((180 if self._flip_180 else 0) + int(out_rot)) % 360
        # On-screen box the (possibly quarter-turned) image occupies: a 90/270
        # turn transposes it, 0/180 leave it as-is. Same rule as drawing the
        # transformed pixmap at rect.topLeft() at its own size, which is what
        # this used to do — so the destination geometry is unchanged.
        dest_w, dest_h = (th, tw) if total % 180 else (tw, th)
        # ⚠ NEVER scale the cache beyond the source's OWN resolution. `rect` is
        # the mosaic's on-screen size, so it grows with the zoom and the old
        # unbounded scaled(tw, th) asked for a pixmap whose AREA grows as zoom²:
        # on a full-plate mosaic at _ZOOM_MAX that is 59943x40299 = 9.7 GB, which
        # SEGFAULTS the process (reproduced; .transformed() below would double
        # it). Capping costs nothing — upscaling a 3000 px source to 60000 px
        # invents no detail; the painter magnifies the capped cache into the SAME
        # destination rect under the SmoothPixmapTransform hint paintEvent
        # already sets, so the picture is unchanged while peak memory stays
        # O(source) instead of O(zoom²). Below the cap (every zoom that worked
        # before) the cache is scaled exactly as it was and drawn 1:1.
        cw = max(1, min(tw, self._mosaic_pixmap.width()))
        ch = max(1, min(th, self._mosaic_pixmap.height()))
        key = (cw, ch, bool(self._flip_180), out_rot, id(self._mosaic_pixmap))
        if key != self._mosaic_cache_key or self._mosaic_scaled_cache is None:
            # Drop the previous cache FIRST: at high zoom the old and new
            # pixmaps are the two largest allocations in the process, and
            # holding both doubles the peak for no reason.
            self._mosaic_scaled_cache = None
            self._mosaic_cache_key = None
            scaled = self._mosaic_pixmap.scaled(
                cw, ch, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
            if total:
                scaled = scaled.transformed(QTransform().rotate(total))
            self._mosaic_scaled_cache = scaled
            self._mosaic_cache_key = key
        p.save()
        p.setOpacity(self._mosaic_opacity)
        p.drawPixmap(
            QRectF(rect.topLeft(), QSizeF(float(dest_w), float(dest_h))),
            self._mosaic_scaled_cache,
            QRectF(self._mosaic_scaled_cache.rect()),
        )
        p.restore()

    def _paint_fluor_overlay(self, p: QPainter) -> None:
        """Draw the blended fluorescence overlay registered to its world extent
        (same frame handling as the brightfield mosaic)."""
        if (not self._fluor_visible or self._fluor_pixmap is None
                or self._fluor_extent_abs is None):
            return
        zx, zy = self._zero_off
        min_x, min_y, max_x, max_y = self._fluor_extent_abs
        tl = self._um_to_px(min_x - zx, min_y - zy)
        br = self._um_to_px(max_x - zx, max_y - zy)
        rect = QRectF(tl, br).normalized()
        if rect.width() < 1 or rect.height() < 1:
            return
        p.save()
        p.setOpacity(self._fluor_opacity)
        if self._flip_180:
            p.translate(rect.center())
            p.rotate(180)
            p.translate(-rect.center())
        p.drawPixmap(rect, self._fluor_pixmap,
                     QRectF(self._fluor_pixmap.rect()))
        p.restore()

    # ── Plate + wells ──────────────────────────────────────────────

    def _paint_plate_and_wells(self, p: QPainter) -> None:
        if not self._wells_visible:
            return
        wells = self._all_wells()
        if not wells:
            return

        # v7.5.x: in "overlay" mode the MOSAIC is the plate — draw no plate
        # body and no fills, just outline circles at the well locations
        # (including calibrated rosette sub-wells; the positions dicts carry
        # them and _well_radius_um resolves their per-well diameters).
        overlay = (getattr(self, "_plate_display_mode", "well") == "overlay")
        if not overlay:
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
        overlay = (getattr(self, "_plate_display_mode", "well") == "overlay")
        if name in self._wells_cal:
            fill = _qc('green', 36)
            border = _qc('green', 200)
        elif name in self._wells_approx:
            fill = _qc('yellow', 28)
            border = _qc('yellow', 170)
        else:
            fill = QColor(0, 0, 0, 0)
            border = _qc('surface2')

        if overlay:
            # Mosaic background mode: OUTLINE circles only — never obscure
            # the image (current well = blue ring, slightly thicker).
            if current:
                border = _qc('blue')
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(border, 1.8 if current else 1.4))
            p.drawEllipse(center, r_px, r_px)
            return

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

    # ── Reference markers (taught well centres) ────────────────────

    def _paint_reference_markers(self, p: QPainter) -> None:
        """Draw persistent reference points as a distinct pink ⊕ + label."""
        if not self._ref_markers:
            return
        color = _qc('pink')
        font = p.font()
        font.setPointSizeF(scaled_font_size(8))
        font.setBold(True)
        p.setFont(font)
        r = s(5)
        for name, (x_um, y_um) in self._ref_markers.items():
            c = self._um_to_px(x_um, y_um)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(color, 1.6))
            p.drawEllipse(c, r, r)
            p.drawLine(QPointF(c.x() - r - s(2), c.y()),
                       QPointF(c.x() + r + s(2), c.y()))
            p.drawLine(QPointF(c.x(), c.y() - r - s(2)),
                       QPointF(c.x(), c.y() + r + s(2)))
            if name:
                p.setPen(QPen(color))
                p.drawText(QPointF(c.x() + r + s(3), c.y() - s(3)), name)

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

    def wheelEvent(self, event) -> None:
        if not self._zoom_enabled:
            super().wheelEvent(event)
            return
        delta = event.angleDelta().y()
        if delta == 0:
            return
        px = event.position().x()
        py = event.position().y()
        ux, uy = self._px_to_um(px, py)             # µm under cursor (old zoom)
        new_zoom = max(self._ZOOM_MIN,
                       min(self._ZOOM_MAX, self._zoom * (1.0015 ** delta)))
        if new_zoom == self._zoom:
            event.accept()
            return
        self._zoom = new_zoom
        if abs(new_zoom - self._ZOOM_MIN) < 1e-6:
            self._pan = [0.0, 0.0]                   # snap back to fit at min
        else:
            # Keep the µm point under the cursor fixed (zoom about cursor).
            npx = self._um_to_px(ux, uy)
            self._pan[0] += px - npx.x()
            self._pan[1] += py - npx.y()
        # If a left-drag pan is in progress (wheel + drag with one mouse),
        # re-anchor it to the just-updated pan so the next mouseMove doesn't
        # snap the overlay back to the press-time snapshot.
        if self._panning:
            self._pan_anchor_px = (px, py)
            self._pan_anchor_val = (self._pan[0], self._pan[1])
        self.update()
        event.accept()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        self._hover_px = (event.position().x(), event.position().y())
        if (self._panning and self._pan_anchor_px is not None
                and (event.buttons() & Qt.MouseButton.LeftButton)):
            ax, ay = self._pan_anchor_px
            bx, by = self._pan_anchor_val
            self._pan = [bx + (event.position().x() - ax),
                         by + (event.position().y() - ay)]
        self.update()

    def leaveEvent(self, _e) -> None:
        self._hover_px = None
        self.update()

    def _handle_click_at(self, px: float, py: float) -> None:
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

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        px = event.position().x()
        py = event.position().y()
        # Pan when the hand tool is active or Shift is held; otherwise the press
        # is a normal click (travel / snap / rim teach), handled on press as
        # before. The Free/Snap toggle always wins so it stays usable.
        shift = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        want_pan = self._zoom_enabled and (self._pan_tool_active or shift)
        if want_pan:
            if (self._toggle_rect_free
                    and self._toggle_rect_free.contains(px, py)):
                self.set_target_mode("free")
                return
            if (self._toggle_rect_snap
                    and self._toggle_rect_snap.contains(px, py)):
                self.set_target_mode("snap")
                return
            self._panning = True
            self._pan_anchor_px = (px, py)
            self._pan_anchor_val = (self._pan[0], self._pan[1])
            self.setCursor(Qt.ClosedHandCursor)
            return
        self._handle_click_at(px, py)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() != Qt.MouseButton.LeftButton or not self._panning:
            return
        self._panning = False
        self._pan_anchor_px = None
        self._pan_anchor_val = None
        self._update_pan_cursor()

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
