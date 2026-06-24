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

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._plate = None
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

    def set_plate(self, plate):
        """Set the well plate model. Triggers repaint."""
        self._plate = plate
        self._hover_well = None
        self._current_well = None
        self.update()

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
        # Only highlight if within ~1 well spacing
        if self._plate and best_dist < self._plate.well_spacing_x * 1000.0 * 0.6:
            self.set_current_well(best_name)
        else:
            self.set_current_well(None)

    # ── Geometry helpers ───────────────────────────────────────

    def _well_layout(self):
        """Compute layout metrics for rendering."""
        if self._plate is None:
            return None
        rows = self._plate.rows
        cols = self._plate.cols
        w = self.width()
        h = self.height()

        # Reserve margin for row/col labels
        margin_left = 18
        margin_top = 14
        margin_right = 4
        margin_bottom = 4

        avail_w = w - margin_left - margin_right
        avail_h = h - margin_top - margin_bottom

        cell_w = avail_w / max(cols, 1)
        cell_h = avail_h / max(rows, 1)
        cell = min(cell_w, cell_h)
        radius = cell * 0.38

        # Center the grid
        grid_w = cols * cell
        grid_h = rows * cell
        ox = margin_left + (avail_w - grid_w) / 2
        oy = margin_top + (avail_h - grid_h) / 2

        return {
            "rows": rows, "cols": cols,
            "cell": cell, "radius": radius,
            "ox": ox, "oy": oy,
        }

    def _well_center(self, layout, row, col):
        """Get pixel center of a well given layout metrics."""
        x = layout["ox"] + (col + 0.5) * layout["cell"]
        y = layout["oy"] + (row + 0.5) * layout["cell"]
        return x, y

    def _hit_test(self, pos) -> str | None:
        """Find well under mouse position."""
        if self._plate is None:
            return None
        layout = self._well_layout()
        if layout is None:
            return None

        from SupportClasses.WellPlate import ROW_LABELS
        for well in self._plate.get_all_wells():
            cx, cy = self._well_center(layout, well.row, well.col)
            dx = pos.x() - cx
            dy = pos.y() - cy
            if math.sqrt(dx * dx + dy * dy) <= layout["radius"] + 2:
                return well.name
        return None

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

        from SupportClasses.WellPlate import ROW_LABELS
        r = layout["radius"]

        # Row labels
        label_font = QFont("Consolas", scaled_font_size(max(7, int(layout["cell"] * 0.35))))
        p.setFont(label_font)
        p.setPen(self._CLR_LABEL)
        for row in range(layout["rows"]):
            _, cy = self._well_center(layout, row, 0)
            p.drawText(QRectF(0, cy - 8, layout["ox"] - 2, 16),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       ROW_LABELS[row])

        # Column labels
        for col in range(layout["cols"]):
            cx, _ = self._well_center(layout, 0, col)
            p.drawText(QRectF(cx - 12, 0, 24, layout["oy"] - 1),
                       Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignBottom,
                       str(col + 1))

        # Wells
        for well in self._plate.get_all_wells():
            cx, cy = self._well_center(layout, well.row, well.col)
            name = well.name

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
            well_obj = next(
                (w for w in self._plate.get_all_wells()
                 if w.name == self._current_well), None)
            if well_obj is not None:
                cx, cy = self._well_center(layout, well_obj.row, well_obj.col)
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
