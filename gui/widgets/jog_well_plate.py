"""
WellPlateNavigator — Interactive well plate widget for fast-travel navigation.

Renders a clickable well plate grid. Clicking a well emits a signal that
the parent page uses to trigger safe fast-travel.

v7.3.1 — Phase 6

Colour coding:
- Grey (#313244): uncalibrated well
- Yellow (#f9e2af): approximate well (geometry-predicted, pre-calibration)
- Green (#a6e3a1): calibrated well (position known from mosaic scan)
- Blue (#89b4fa): current position (nearest well)
- Orange (#fab387): selected/hovered well

Usage::

    nav = WellPlateNavigator()
    nav.set_plate(WellPlate.from_format(96))
    nav.set_calibrated_wells({"A1", "A2", ...})
    nav.well_clicked.connect(lambda name: controller.safe_travel_to(...))
"""

from __future__ import annotations

import logging
import math

from PySide6.QtWidgets import QWidget, QToolTip
from PySide6.QtCore import Qt, Signal, QRectF, QPointF
from PySide6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QMouseEvent, QPainterPath,
)

from gui.scaling import s, scaled_font_size
from gui.widgets.plate_layout import fit_wells, wells_from_plate

logger = logging.getLogger(__name__)


class WellPlateNavigator(QWidget):
    """Clickable well plate grid for jog-page fast travel.

    Signals:
        well_clicked(str): Emitted when user clicks a well.
    """

    well_clicked = Signal(str)
    #: v7.19: the multi-selection changed. Payload is in PLATE order, so a
    #: readout or a tooltip built from it is stable across repaints. Only ever
    #: emitted while :meth:`set_multi_select_enabled` is on.
    selection_changed = Signal(list)

    # Colours (Catppuccin Mocha palette)
    _CLR_BG = QColor("#11111b")
    _CLR_UNCAL = QColor("#313244")
    _CLR_UNCAL_BORDER = QColor("#585b70")
    _CLR_CAL = QColor("#a6e3a180")
    _CLR_CAL_BORDER = QColor("#a6e3a1")
    _CLR_APPROX = QColor("#f9e2af60")       # v7.3.2: Yellow (semi-transparent): approximate
    _CLR_APPROX_BORDER = QColor("#f9e2af")  # v7.3.2: Yellow border
    _CLR_CURRENT = QColor("#89b4fa80")
    _CLR_CURRENT_BORDER = QColor("#89b4fa")
    _CLR_HOVER = QColor("#fab38780")
    _CLR_HOVER_BORDER = QColor("#fab387")
    _CLR_TEXT = QColor("#cdd6f4")
    _CLR_LABEL = QColor("#6c7086")

    # v7.19 print-queue overlay. Teal / mauve / red collide with none of the four
    # state hues above (green calibrated · yellow approximate · blue current ·
    # orange hover) and stay distinguishable at the radius floor below.
    _CLR_QUEUE_PATH = QColor("#94e2d5")        # a queued toolpath
    _CLR_QUEUE_FILL = QColor("#94e2d51f")      # ~12 % teal wash on a queued well
    _CLR_QUEUE_BORDER = QColor("#94e2d5")
    _CLR_QUEUE_BAD = QColor("#f38ba8")         # queued but NOT runnable
    _CLR_SELECTED = QColor("#cba6f7")          # in the multi-selection

    # v7.12: floor so a small well on a mixed-diameter plate (a 5.5 mm rosette
    # bore beside a 28 mm insert) stays visible and clickable in this widget,
    # which is often only ~200 px wide.
    _MIN_WELL_RADIUS_PX = 4.0

    # v7.19: below this radius a toolpath glyph becomes a filled centre dot. A
    # few-pixel scribble carries no shape (on a 384 plate the well is at the
    # floor above) and costs thousands of lineTo calls per well; a dot still says
    # "something is queued here", which is the only readable fact left.
    _GLYPH_MIN_R_PX = 6.0

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._plate = None
        # v7.12: the plate's real outline in the A1-relative mm frame, when it
        # can be resolved (see set_plate). None → frame the wells instead.
        self._footprint: tuple[float, float, float, float] | None = None
        self._calibrated_wells: set[str] = set()
        self._approximate_wells: set[str] = set()  # v7.3.2: geometry-predicted positions
        self._current_well: str | None = None
        self._hover_well: str | None = None
        self._well_positions: dict[str, tuple[float, float]] | None = None  # well → (x_um, y_um)
        # v7.5.x: optional raster-grid preview drawn inside the current well
        # (cols × rows tile grid) — used by the Fluorescence Mosaic workflow to
        # show the planned single-well raster coverage. 0,0 = off.
        self._raster_cols = 0
        self._raster_rows = 0
        # v7.19: per-well toolpath glyphs, {well: (paths, colour|None, ok)}.
        # Empty = this widget paints EXACTLY what it painted before v7.19, which
        # is what keeps the Jog page and the Fluorescence Mosaic page unchanged.
        self._well_paths: dict[str, tuple] = {}
        # v7.19: opt-in multi-selection, OFF by default (see
        # set_multi_select_enabled for why that default is load-bearing).
        self._multi_enabled = False
        self._selected_wells: set[str] = set()
        self._sel_anchor: str | None = None      # Shift-range anchor
        self._brush_active = False
        self._brush_additive = False
        self._brush_base: set[str] = set()
        self._brush_touched: set[str] = set()

        self.setMinimumSize(s(160), s(100))
        self.setMouseTracking(True)
        self.setToolTip("Click a well to fast-travel")

    # ── Public API ─────────────────────────────────────────────

    def set_plate(self, plate, footprint=None):
        """Set the well plate model. Triggers repaint.

        v7.12: *footprint* is the plate outline ``(x0, y0, x1, y1)`` in
        A1-relative mm. Callers rarely need to pass it — it is resolved from
        the plate's own identity via
        ``PlateDocumentStore.plate_footprint_extent_mm`` — but an explicit
        value wins, which is what lets tests pin the framing.
        """
        self._plate = plate
        self._footprint = footprint if footprint is not None \
            else self._resolve_footprint(plate)
        self._hover_well = None
        self._current_well = None
        # v7.19: a new plate renames or removes every well, so anything keyed by
        # well name is stale. The owning page re-pushes what still applies (see
        # QuickPrintWorkflowPage.set_calibration_data).
        self._well_paths = {}
        self._selected_wells = set()
        self._sel_anchor = None
        self.update()

    @staticmethod
    def _resolve_footprint(plate):
        """Plate outline in A1-relative mm, or None.

        Never raises: a plate we cannot identify simply frames its wells, which
        is what this widget did for its whole life before v7.12.
        """
        if plate is None:
            return None
        try:
            from SupportClasses.PlateDocumentStore import (
                plate_footprint_extent_mm,
            )
            return plate_footprint_extent_mm(getattr(plate, "format", None))
        except Exception:                                  # pragma: no cover
            logger.debug("No plate footprint for navigator", exc_info=True)
            return None

    def set_calibrated_wells(self, names: set[str]):
        """Mark wells that have calibrated positions."""
        self._calibrated_wells = names
        self.update()

    def set_well_positions(self, positions: dict[str, tuple[float, float]] | None):
        """Store well positions for tooltip display (calibrated — green)."""
        self._well_positions = positions
        if positions:
            self._calibrated_wells = set(positions.keys())
            self._approximate_wells.clear()  # Calibrated replaces approximate
        self.update()

    def set_approximate_positions(self, positions: dict[str, tuple[float, float]] | None):
        """v7.3.2: Store geometry-predicted positions (yellow, pre-calibration).

        These are replaced when set_well_positions() is called with calibrated data.
        """
        self._well_positions = positions
        if positions:
            self._approximate_wells = set(positions.keys())
            self._calibrated_wells.clear()
        else:
            self._approximate_wells.clear()
        self.update()

    def set_current_well(self, well_name: str | None):
        """Highlight the well nearest to the current stage position."""
        if well_name != self._current_well:
            self._current_well = well_name
            self.update()

    def set_raster_grid(self, cols: int, rows: int):
        """v7.5.x: overlay a ``cols`` × ``rows`` raster-tile grid inside the
        current well (clipped to its circle). Pass 0, 0 to clear. Used by the
        Fluorescence Mosaic workflow to preview single-well scan coverage."""
        cols = max(0, int(cols))
        rows = max(0, int(rows))
        if (cols, rows) != (self._raster_cols, self._raster_rows):
            self._raster_cols = cols
            self._raster_rows = rows
            self.update()

    # ── v7.19: per-well toolpath glyphs ────────────────────────────

    def set_well_paths(self, overlays) -> None:
        """Draw a small toolpath glyph inside named wells.

        *overlays* maps well name → ``(paths, colour_hex_or_None, ok)``:

          ``paths``   ``[[(x_mm, y_mm), …], …]`` in **WELL-RELATIVE mm** — one
                      polyline per contiguous print run, i.e. exactly the shape
                      ``QuickPrintWorkflowPage._path_segments_for_selection``
                      already returns, so no caller converts anything.
          ``colour``  hex string, or None for the default teal.
          ``ok``      False renders it as queued-but-NOT-runnable (dashed red).

        A 2-tuple ``(paths, colour)`` and a bare list of polylines are both
        accepted, so a caller holding nothing but geometry needs no tuple.

        ``None`` / ``{}`` clears it, and this widget then paints EXACTLY what it
        painted before this method existed — the Fluorescence Mosaic page never
        calls it and must stay pixel-identical.

        Unlike :meth:`set_raster_grid` this is keyed on the CALLER'S well names,
        never on ``_current_well``: a raster preview is a single-well thing, a
        print queue is a plate-wide one.
        """
        norm = self._normalise_overlays(overlays)
        # Change gate, as set_current_well / set_raster_grid already model: the
        # owning page pushes this on every debounced widget edit, and a repaint
        # per keystroke on a 384-well plate is a real cost.
        if norm != self._well_paths:
            self._well_paths = norm
            self.update()

    @staticmethod
    def _normalise_overlays(overlays) -> dict:
        """Coerce whatever a caller passed into ``{well: (paths, colour, ok)}``.

        Total on purpose: this feeds ``paintEvent``, which must never raise, so a
        malformed entry is dropped rather than allowed to reach the painter.
        """
        if not overlays or not isinstance(overlays, dict):
            return {}
        out: dict[str, tuple] = {}
        for name, value in overlays.items():
            if not isinstance(name, str) or not name:
                continue
            colour, ok = None, True
            if isinstance(value, tuple):
                if len(value) >= 3:
                    paths, colour, ok = value[0], value[1], bool(value[2])
                elif len(value) == 2:
                    paths, colour = value[0], value[1]
                elif len(value) == 1:
                    paths = value[0]
                else:
                    continue
            else:
                paths = value
            clean: list[list[tuple[float, float]]] = []
            if isinstance(paths, (list, tuple)):
                for poly in paths:
                    if not isinstance(poly, (list, tuple)):
                        continue
                    pts = []
                    for pt in poly:
                        try:
                            px, py = pt[0], pt[1]
                            pts.append((float(px), float(py)))
                        except (TypeError, ValueError, IndexError):
                            continue
                    if len(pts) >= 2:
                        clean.append(pts)
            if clean or not ok:
                out[name] = (clean, colour if isinstance(colour, str) else None,
                             ok)
        return out

    # ── v7.19: multi-selection ─────────────────────────────────────

    def set_multi_select_enabled(self, on: bool) -> None:
        """Turn drag/Ctrl/Shift multi-selection on for this instance.

        **Default OFF, deliberately.** The flag is not a preference — it states
        that the owning page has somewhere to put a SET of wells. The Jog page's
        contract is "click a well → fast-travel", and you cannot travel to six
        wells; the Fluorescence Mosaic page scans exactly one well. Turning this
        on for them would change what a click means on a page that has no use for
        the answer, and would make a Ctrl+click silently NOT travel.
        """
        on = bool(on)
        if on == self._multi_enabled:
            return
        self._multi_enabled = on
        if not on:
            self._selected_wells = set()
            self._sel_anchor = None
            self._brush_active = False
        self.update()

    def multi_select_enabled(self) -> bool:
        return self._multi_enabled

    def selected_wells(self) -> list[str]:
        """The multi-selection, in PLATE order (not click order)."""
        return self._ordered(self._selected_wells)

    def set_selected_wells(self, names) -> None:
        """Replace the multi-selection. Emits only when it actually changed."""
        self._set_selection({n for n in (names or []) if isinstance(n, str)})

    def clear_selection(self) -> None:
        self.set_selected_wells([])

    def _ordered(self, names) -> list[str]:
        """*names* in plate order; unknown wells sort last, alphabetically."""
        names = set(names or [])
        if not names:
            return []
        layout = self._well_layout()
        if layout is None:
            return sorted(names)
        order = [w[0] for w in layout["wells"] if w[0] in names]
        rest = sorted(n for n in names if n not in set(order))
        return order + rest

    def _set_selection(self, names: set) -> None:
        if names == self._selected_wells:
            return          # a re-crossed well must not re-emit mid-drag
        self._selected_wells = set(names)
        self.update()
        self.selection_changed.emit(self._ordered(self._selected_wells))

    def _range_wells(self, a: str, b: str) -> set:
        """The rectangular row/col span between two wells.

        Only meaningful on a REAL grid: on a parametric plate ``row``/``col`` are
        a pseudo-grid where a whole ring of wells shares one cell, so a "range"
        there would select wells that are not in that row at all. Degrades to
        just *b*, which is what a plain click would have given.
        """
        if not self._draw_headers() or self._plate is None:
            return {b}
        try:
            info = {w.name: w for w in self._plate.get_all_wells()}
            wa, wb = info[a], info[b]
        except Exception:
            return {b}
        r0, r1 = sorted((wa.row, wb.row))
        c0, c1 = sorted((wa.col, wb.col))
        return {w.name for w in info.values()
                if r0 <= w.row <= r1 and c0 <= w.col <= c1}

    def update_current_from_position(self, x_um: float, y_um: float):
        """Determine nearest well to stage position and highlight it."""
        if self._well_positions is None or not self._well_positions:
            return
        best_name = None
        best_dist = float('inf')
        for name, (wx, wy) in self._well_positions.items():
            d = math.sqrt((x_um - wx) ** 2 + (y_um - wy) ** 2)
            if d < best_dist:
                best_dist = d
                best_name = name
        # Only highlight if within ~1 well spacing. v7.12: `well_spacing_x` is
        # 0.0 on a parametric plate, so this gate used to be "within 0 µm" —
        # the current well could never light up. Measure the pitch from the
        # wells themselves (identical to the declared spacing on a regular
        # plate), and fall back to the well radius on a one-well plate.
        if self._plate is None:
            self.set_current_well(None)
            return
        pitch_mm = self._plate.nearest_neighbour_pitch_mm()
        if pitch_mm <= 0:
            pitch_mm = self._plate.representative_well_diameter
        if pitch_mm > 0 and best_dist < pitch_mm * 1000.0 * 0.6:
            self.set_current_well(best_name)
        else:
            self.set_current_well(None)

    # ── Geometry helpers ───────────────────────────────────────

    def _draw_headers(self) -> bool:
        """True when row/column headers describe this plate.

        Headers spell a regular grid ("row B, column 4"), which a parametric
        plate does not have: ``rows``/``cols`` there are a pseudo-grid where a
        whole ring of wells shares one cell, so the letters would label wells
        that are not in that row at all. Same gate ``WellPlateView`` uses.
        """
        plate = self._plate
        return bool(plate is not None
                    and getattr(plate, "well_spacing_x", 0) > 0
                    and getattr(plate, "well_spacing_y", 0) > 0)

    def _well_layout(self):
        """Compute the mm → px transform for rendering.

        v7.12: was a ``rows`` × ``cols`` cell grid indexed by ``well.row`` /
        ``well.col``. On a parametric plate several wells share a cell (an
        entire 3-well ring lands on one), so they were drawn stacked — a ring
        rendered as a single circle — and every well got the same radius. Now
        every well is placed at its real ``x``/``y`` and sized by its own
        diameter, which is also what makes the standard grid come out right:
        a regular plate's real geometry *is* a regular grid.
        """
        if self._plate is None:
            return None
        wells = wells_from_plate(self._plate)
        if not wells:
            return None

        headers = self._draw_headers()
        margins = (18.0, 14.0, 4.0, 4.0) if headers else (4.0, 4.0, 4.0, 4.0)
        transform = fit_wells(
            wells, self.width(), self.height(), margins,
            footprint=self._footprint,
            min_radius_px=self._MIN_WELL_RADIUS_PX,
        )
        if transform is None:
            return None
        return {"transform": transform, "wells": wells, "headers": headers}

    def _well_center(self, layout, well_name: str):
        """Pixel centre of *well_name*, or None if it is not on the plate."""
        for name, x, y, _d in layout["wells"]:
            if name == well_name:
                return layout["transform"].to_px(x, y)
        return None

    def _well_radius(self, layout, diameter_mm: float) -> float:
        return layout["transform"].radius_px(diameter_mm)

    def _hit_test(self, pos) -> str | None:
        """Find the well under the mouse.

        Picks the NEAREST centre rather than the first well whose circle
        contains the point: on a plate with mixed diameters a small well can
        sit inside a large neighbour's circle, and row-major "first match"
        would make it unclickable.
        """
        layout = self._well_layout()
        if layout is None:
            return None

        best_name, best_dist = None, float("inf")
        for name, x, y, diameter in layout["wells"]:
            cx, cy = layout["transform"].to_px(x, y)
            dist = math.hypot(pos.x() - cx, pos.y() - cy)
            if dist <= self._well_radius(layout, diameter) + 2 and dist < best_dist:
                best_dist, best_name = dist, name
        return best_name

    # ── Painting ───────────────────────────────────────────────

    def paintEvent(self, event):
        if self._plate is None:
            return
        layout = self._well_layout()
        if layout is None:
            return

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        p.fillRect(self.rect(), self._CLR_BG)

        transform = layout["transform"]

        # Row / column labels — only where they mean something (a real grid).
        if layout["headers"]:
            self._paint_headers(p, layout)

        # Wells — each at its own centre and its own size.
        for name, wx, wy, diameter in layout["wells"]:
            cx, cy = transform.to_px(wx, wy)
            r = self._well_radius(layout, diameter)

            # Determine colour
            if name == self._hover_well:
                fill = self._CLR_HOVER
                border = self._CLR_HOVER_BORDER
            elif name == self._current_well:
                fill = self._CLR_CURRENT
                border = self._CLR_CURRENT_BORDER
            elif name in self._calibrated_wells:
                fill = self._CLR_CAL
                border = self._CLR_CAL_BORDER
            elif name in self._approximate_wells:
                fill = self._CLR_APPROX
                border = self._CLR_APPROX_BORDER
            else:
                fill = self._CLR_UNCAL
                border = self._CLR_UNCAL_BORDER

            # v7.19: "queued" is a FIFTH fact that co-occurs with the four above
            # — a well can be calibrated AND queued AND selected AND the one
            # being edited, and the operator needs all of that at once. So it
            # gets its own channels: a faint wash over the calibration fill
            # (painted first, underneath) plus a slightly heavier border. It
            # never REPLACES the calibration colour, which would hide it.
            entry = self._well_paths.get(name)
            border_w = 1.0
            if entry is not None:
                p.setPen(QPen(border, 1.0))
                p.setBrush(QBrush(fill))
                p.drawEllipse(QPointF(cx, cy), r, r)
                fill = self._CLR_QUEUE_FILL
                if name not in (self._hover_well, self._current_well):
                    border = (self._CLR_QUEUE_BORDER if entry[2]
                              else self._CLR_QUEUE_BAD)
                border_w = 1.2 if entry[2] else 1.6
            if name == self._current_well:
                # The ACTIVE well is the thickest ring on screen: it is the one
                # the editor widgets are bound to.
                border_w = 2.2

            p.setPen(QPen(border, border_w))
            p.setBrush(QBrush(fill))
            p.drawEllipse(QPointF(cx, cy), r, r)

        # v7.19: queued-print toolpath glyphs. CLIPPED to each well circle, so a
        # print larger than its well cannot bleed onto a neighbour — the clip is
        # what makes "does it fit?" legible at a glance, and an oversized print
        # is a real design problem the operator should SEE rather than have the
        # renderer quietly scale away.
        if self._well_paths:
            by_name = {w[0]: w for w in layout["wells"]}
            for name, (paths, colour, ok) in self._well_paths.items():
                hit = by_name.get(name)
                if hit is None:
                    continue
                cx, cy = transform.to_px(hit[1], hit[2])
                r = self._well_radius(layout, hit[3])
                p.save()
                pen = QPen(self._CLR_QUEUE_BAD if not ok
                           else QColor(colour or self._CLR_QUEUE_PATH))
                pen.setWidthF(max(0.8, s(1.1)))
                pen.setCosmetic(True)      # 1 device px at any mm→px scale
                if not ok:
                    pen.setStyle(Qt.PenStyle.DashLine)
                p.setPen(pen)
                p.setBrush(QBrush(Qt.BrushStyle.NoBrush))

                if r < self._GLYPH_MIN_R_PX or not paths:
                    dot = max(1.2, r * 0.35)
                    p.setBrush(QBrush(pen.color()))
                    p.drawEllipse(QPointF(cx, cy), dot, dot)
                    p.restore()
                    continue

                clip = QPainterPath()
                clip.addEllipse(QPointF(cx, cy), r, r)
                p.setClipPath(clip)
                spilled = False
                for poly in paths:
                    path = QPainterPath()
                    for i, (mx, my) in enumerate(poly):
                        # WELL-RELATIVE mm → px. **NO Y FLIP.** This widget is a
                        # plate-local schematic: PlateTransform.to_px puts A1
                        # top-left with +Y painting DOWN, and these polylines are
                        # in the same plate-local frame the executor consumes.
                        # (PrintThumbnail flips Y because it is a standalone
                        # "Y up" chart with no plate around it. This is not that.)
                        px_ = cx + float(mx) * transform.scale
                        py_ = cy + float(my) * transform.scale
                        if math.hypot(px_ - cx, py_ - cy) > r:
                            spilled = True
                        if i == 0:
                            path.moveTo(px_, py_)
                        else:
                            path.lineTo(px_, py_)
                    p.drawPath(path)
                p.setClipping(False)
                if spilled:
                    # The clip truncated something. Say so, rather than showing a
                    # print that appears to fit when it does not. ⚠ This is a
                    # RENDERING fact only: `_well_radius` is floored at
                    # _MIN_WELL_RADIUS_PX, so a floored r can EXCEED the true
                    # well radius and a genuinely-spilling print would read as
                    # fitting. The authoritative fit check is in mm, on the page.
                    p.setPen(QPen(self._CLR_QUEUE_BAD, max(1.0, s(1.4))))
                    p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                    p.drawEllipse(QPointF(cx, cy), r, r)
                p.restore()

        # v7.5.x: raster-grid preview inside the current well (Fluorescence
        # Mosaic). Drawn last so it sits on top of the well fill, clipped to the
        # well circle so it reads as "the tile grid that will cover this well".
        if (self._raster_cols > 0 and self._raster_rows > 0
                and self._current_well):
            # v7.12: resolve THIS well's own centre and radius. The old code
            # indexed the pseudo-grid by row/col and reused `r` left over from
            # the well loop, so on a mixed-diameter plate the preview grid was
            # drawn at the last-painted well's size.
            hit = next((w for w in layout["wells"]
                        if w[0] == self._current_well), None)
            if hit is not None:
                cx, cy = transform.to_px(hit[1], hit[2])
                r = self._well_radius(layout, hit[3])
                p.save()
                clip = QPainterPath()
                clip.addEllipse(QPointF(cx, cy), r, r)
                p.setClipPath(clip)
                grid_pen = QPen(QColor("#cdd6f4"), 0.6)
                p.setPen(grid_pen)
                left, top = cx - r, cy - r
                cw = (2 * r) / self._raster_cols
                ch = (2 * r) / self._raster_rows
                for i in range(1, self._raster_cols):
                    x = left + i * cw
                    p.drawLine(QPointF(x, top), QPointF(x, top + 2 * r))
                for j in range(1, self._raster_rows):
                    y = top + j * ch
                    p.drawLine(QPointF(left, y), QPointF(left + 2 * r, y))
                # Emphasise the previewed well's outline.
                p.setClipping(False)
                p.setPen(QPen(QColor("#f9e2af"), 1.4))
                p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                p.drawEllipse(QPointF(cx, cy), r, r)
                p.restore()

        # v7.19: the multi-selection, painted LAST as an OUTSET ring. Selection is
        # orthogonal to calibrated / queued / active, so it must not join the
        # fill-precedence chain (that would hide one of the others). Outsetting
        # is what keeps it readable at the _MIN_WELL_RADIUS_PX floor: at r = 4 the
        # ring lands at 5.5 and never paints over the glyph inside.
        if self._multi_enabled and self._selected_wells:
            by_name = {w[0]: w for w in layout["wells"]}
            p.setPen(QPen(self._CLR_SELECTED, max(1.4, s(1.8))))
            p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            for name in self._selected_wells:
                hit = by_name.get(name)
                if hit is None:
                    continue
                cx, cy = transform.to_px(hit[1], hit[2])
                r = self._well_radius(layout, hit[3]) + max(1.0, s(1.5))
                p.drawEllipse(QPointF(cx, cy), r, r)

        p.end()

    def _paint_headers(self, p: QPainter, layout) -> None:
        """Row letters and column numbers for a regular grid.

        Positions come from the real well centres of column 0 / row 0, not
        from cell arithmetic, so the labels stay glued to the wells they name
        under the geometric layout.
        """
        from SupportClasses.WellPlate import ROW_LABELS

        transform = layout["transform"]
        pitch_px = max(8.0, self._plate.nearest_neighbour_pitch_mm()
                       * transform.scale)
        p.setFont(QFont("Consolas",
                        scaled_font_size(max(7, int(pitch_px * 0.35)))))
        p.setPen(self._CLR_LABEL)

        seen_rows: set[int] = set()
        seen_cols: set[int] = set()
        for well in self._plate.get_all_wells():
            cx, cy = transform.to_px(well.x, well.y)
            if well.col == 0 and well.row not in seen_rows:
                seen_rows.add(well.row)
                if 0 <= well.row < len(ROW_LABELS):
                    p.drawText(
                        QRectF(0, cy - 8, 16, 16),
                        Qt.AlignmentFlag.AlignRight
                        | Qt.AlignmentFlag.AlignVCenter,
                        ROW_LABELS[well.row])
            if well.row == 0 and well.col not in seen_cols:
                seen_cols.add(well.col)
                p.drawText(
                    QRectF(cx - 12, 0, 24, 13),
                    Qt.AlignmentFlag.AlignCenter
                    | Qt.AlignmentFlag.AlignBottom,
                    str(well.col + 1))

    # ── Mouse events ───────────────────────────────────────────

    def mouseMoveEvent(self, event: QMouseEvent):
        well = self._hit_test(event.pos())
        if well != self._hover_well:
            self._hover_well = well
            self.update()
        # v7.19: brush-drag — the pointer paints wells into the selection. The
        # button test is required because setMouseTracking(True) means this fires
        # with no button held. A tooltip storm mid-drag is unusable, so it is
        # suppressed for the duration.
        if self._brush_active:
            if well and well not in self._brush_touched:
                self._brush_touched.add(well)
                if self._sel_anchor is None:
                    self._sel_anchor = well
                self._set_selection(self._brush_base | self._brush_touched)
            QToolTip.hideText()
            return
        # Tooltip
        if well and self._well_positions and well in self._well_positions:
            x, y = self._well_positions[well]
            prefix = "~" if well in self._approximate_wells else ""
            QToolTip.showText(event.globalPosition().toPoint(),
                              f"{well}: {prefix}({x:.0f}, {y:.0f}) µm"
                              + (" (approx)" if well in self._approximate_wells else ""))
        elif well:
            QToolTip.showText(event.globalPosition().toPoint(), well)
        else:
            QToolTip.hideText()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        well = self._hit_test(event.pos())
        if not self._multi_enabled:
            # Byte-for-byte the pre-v7.19 body — the Jog page and the
            # Fluorescence Mosaic page must be unchanged.
            if well:
                self.well_clicked.emit(well)
                logger.debug(f"Well plate navigator: clicked {well}")
            return

        mods = event.modifiers()
        ctrl = bool(mods & Qt.KeyboardModifier.ControlModifier)
        shift = bool(mods & Qt.KeyboardModifier.ShiftModifier)
        self._brush_active = True
        self._brush_additive = ctrl or shift
        self._brush_base = set(self._selected_wells) if self._brush_additive \
            else set()
        self._brush_touched = set()
        if well is None:
            # A stroke that started on empty space may still cross wells; the
            # release decides whether it was really "click nothing to clear".
            self._set_selection(self._brush_base)
            return
        if shift and self._sel_anchor:
            self._brush_touched = self._range_wells(self._sel_anchor, well)
        else:
            self._brush_touched = {well}
            self._sel_anchor = well
        self._set_selection(self._brush_base | self._brush_touched)
        # A PLAIN press is still a click, so every existing consumer is unchanged
        # and the page can make this well the one being edited. Ctrl/Shift do NOT
        # emit: adding to a selection must not move what the operator is editing.
        if not self._brush_additive:
            self.well_clicked.emit(well)
            logger.debug(f"Well plate navigator: clicked {well}")

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() != Qt.MouseButton.LeftButton or not self._brush_active:
            return
        self._brush_active = False
        if not self._brush_touched and not self._brush_additive:
            self._set_selection(set())     # a plain click on empty space clears
        self._brush_base = set()
        self._brush_touched = set()

    def leaveEvent(self, event):
        # v7.19: deliberately does NOT cancel a brush-drag. The mouse is grabbed
        # for the duration of a drag, so moves keep arriving after the pointer
        # leaves the widget; cancelling here would drop wells the operator is
        # still selecting whenever they overshoot an edge.
        if self._hover_well is not None:
            self._hover_well = None
            self.update()
