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

    # v7.12: floor so a small well on a mixed-diameter plate (a 5.5 mm rosette
    # bore beside a 28 mm insert) stays visible and clickable in this widget,
    # which is often only ~200 px wide.
    _MIN_WELL_RADIUS_PX = 4.0

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

            p.setPen(QPen(border, 1.0))
            p.setBrush(QBrush(fill))
            p.drawEllipse(QPointF(cx, cy), r, r)

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
        if event.button() == Qt.MouseButton.LeftButton:
            well = self._hit_test(event.pos())
            if well:
                self.well_clicked.emit(well)
                logger.debug(f"Well plate navigator: clicked {well}")

    def leaveEvent(self, event):
        if self._hover_well is not None:
            self._hover_well = None
            self.update()
