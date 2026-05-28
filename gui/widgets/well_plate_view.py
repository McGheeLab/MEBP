"""
well_plate_view.py — Interactive top-down well plate visualization for MEBP v7.1.

QGraphicsView-based widget with full multi-select support:
1. Click        — select single well (clears previous)
2. Ctrl+Click   — toggle individual well in/out of selection
3. Shift+Click  — range select (rectangle from last click to current)
4. Rubber-band  — click-drag to draw selection rectangle
5. Row/Col header click — select entire row or column
6. Ctrl+A       — select all wells

Visual features:
- Wells color-coded by role (Catppuccin palette)
- Selected wells: bright highlight border
- Hover tooltips with well info
- Rosette overlay icon
- Mini-preview of assigned geometry on print wells
- Real-time needle position crosshair (updated from controller)
- Row (A–H) and column (1–12) header labels

Signals:
- selection_changed(list[str])    — selected well names
- well_double_clicked(str)        — double-click on one well
- context_menu_requested(list[str], QPoint) — right-click on selection

Session G — Tasks P5.20, P5.21, P5.22, P5.24, P5.31.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsTextItem, QGraphicsLineItem, QGraphicsRectItem,
    QGraphicsItemGroup, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QMenu, QSizePolicy,
)
from PySide6.QtCore import (
    Qt, Signal, QRectF, QPointF, QSizeF, QTimer,
)
from PySide6.QtGui import (
    QColor, QPen, QBrush, QFont, QPainter, QKeySequence,
    QShortcut, QMouseEvent, QWheelEvent, QCursor,
)

from SupportClasses.PhysicalModels import WellRole, ROLE_COLORS
from SupportClasses.WellPlate import WellPlate, WellInfo, ROW_LABELS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

# Visual scale: mm → scene units (pixels at 1:1 zoom)
SCALE_FACTOR = 6.0

# Well appearance
WELL_BORDER_WIDTH = 1.5
WELL_SELECTED_BORDER_WIDTH = 3.0
WELL_HOVER_BORDER_WIDTH = 2.0
SELECTED_BORDER_COLOR = "#cba6f7"    # Mauve (Catppuccin)
HOVER_BORDER_COLOR = "#89b4fa"       # Blue (Catppuccin)
EMPTY_WELL_COLOR = "#585b70"

# Needle crosshair
NEEDLE_COLOR = "#f5c2e7"            # Pink
NEEDLE_SIZE = 12                     # Crosshair arm length in scene units

# Header labels
HEADER_FONT_SIZE = 10
HEADER_MARGIN = 20                   # px from plate edge

# Plate frame
PLATE_PADDING = 15                   # Scene units padding around wells


# ═══════════════════════════════════════════════════════════════════
# Well Graphics Item
# ═══════════════════════════════════════════════════════════════════

class WellGraphicsItem(QGraphicsEllipseItem):
    """
    A single well rendered as a filled circle with border.

    Stores well metadata for selection/query. Supports hover highlighting,
    selection highlighting, and role-based fill coloring.
    """

    def __init__(
        self,
        well_info: WellInfo,
        radius_scene: float,
        parent=None,
    ):
        # Create bounding rect centered on well position
        cx = well_info.x * SCALE_FACTOR
        cy = well_info.y * SCALE_FACTOR
        super().__init__(
            cx - radius_scene, cy - radius_scene,
            radius_scene * 2, radius_scene * 2,
            parent,
        )

        self.well_info = well_info
        self._selected = False
        self._role = WellRole.EMPTY
        self._color = EMPTY_WELL_COLOR
        self._label_text = ""

        # Enable hover events
        self.setAcceptHoverEvents(True)
        self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        # Default appearance
        self._apply_style()

        # Tooltip
        self._update_tooltip()

    @property
    def well_name(self) -> str:
        return self.well_info.name

    @property
    def is_selected(self) -> bool:
        return self._selected

    def set_selected(self, selected: bool) -> None:
        """Toggle visual selection state."""
        self._selected = selected
        self._apply_style()

    def set_role(self, role: WellRole, label: str = "") -> None:
        """Update role, color, and label."""
        self._role = role
        self._color = ROLE_COLORS.get(role, EMPTY_WELL_COLOR)
        self._label_text = label
        self._apply_style()
        self._update_tooltip()

    def set_color(self, color: str) -> None:
        """Direct color override."""
        self._color = color
        self._apply_style()

    def _apply_style(self) -> None:
        """Apply current visual state (fill + border)."""
        fill = QColor(self._color)
        fill.setAlpha(180)
        self.setBrush(QBrush(fill))

        if self._selected:
            pen = QPen(QColor(SELECTED_BORDER_COLOR), WELL_SELECTED_BORDER_WIDTH)
        else:
            pen = QPen(QColor(self._color).darker(130), WELL_BORDER_WIDTH)

        self.setPen(pen)

    def _update_tooltip(self) -> None:
        role_str = self._role.value.capitalize() if self._role != WellRole.EMPTY else "Empty"
        tip = f"{self.well_info.name} — {role_str}"
        if self._label_text:
            tip += f"\n{self._label_text}"
        self.setToolTip(tip)

    def hoverEnterEvent(self, event):
        if not self._selected:
            pen = QPen(QColor(HOVER_BORDER_COLOR), WELL_HOVER_BORDER_WIDTH)
            self.setPen(pen)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self._apply_style()
        super().hoverLeaveEvent(event)


# ═══════════════════════════════════════════════════════════════════
# Well Plate View Widget
# ═══════════════════════════════════════════════════════════════════

class WellPlateView(QGraphicsView):
    """
    Interactive top-down well plate view with multi-select.

    Renders the plate as colored circles on a QGraphicsScene, with
    row/column headers, rubber-band selection, and real-time needle
    position overlay.

    Signals:
        selection_changed — emits list of selected well names
        well_double_clicked — emits single well name on double-click
        context_menu_requested — emits (well_names, QPoint) on right-click
    """

    selection_changed = Signal(list)
    well_double_clicked = Signal(str)
    context_menu_requested = Signal(list, QPointF)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        # Data
        self._plate: WellPlate | None = None
        self._well_items: dict[str, WellGraphicsItem] = {}
        self._selected_wells: list[str] = []
        self._last_click_well: str | None = None

        # Needle overlay
        self._needle_x: float | None = None
        self._needle_y: float | None = None
        self._needle_items: list = []

        # Rubber-band tracking
        self._rubber_band_origin: QPointF | None = None
        self._rubber_band_rect: QGraphicsRectItem | None = None

        # View settings
        self.setRenderHints(
            QPainter.RenderHint.Antialiasing
            | QPainter.RenderHint.SmoothPixmapTransform
        )
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumHeight(200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Background color
        self.setStyleSheet("background-color: #1e1e2e; border: 1px solid #45475a;")

        # Keyboard shortcuts
        self._setup_shortcuts()

    # ── Public API ────────────────────────────────────────────────

    def set_plate(self, plate: WellPlate) -> None:
        """Load a new plate and redraw all wells."""
        self._plate = plate
        self._selected_wells.clear()
        self._last_click_well = None
        self._rebuild_scene()

    def get_selected_wells(self) -> list[str]:
        """Return list of currently selected well names."""
        return list(self._selected_wells)

    def set_selection(self, well_names: list[str]) -> None:
        """Programmatically set selection."""
        self._clear_selection(emit=False)
        for name in well_names:
            item = self._well_items.get(name)
            if item:
                item.set_selected(True)
                self._selected_wells.append(name)
        self.selection_changed.emit(list(self._selected_wells))

    def clear_selection(self) -> None:
        """Clear all selection."""
        self._clear_selection(emit=True)

    def update_well_appearance(
        self,
        well_name: str,
        role: WellRole,
        label: str = "",
    ) -> None:
        """Update a single well's role color and label."""
        item = self._well_items.get(well_name)
        if item:
            item.set_role(role, label)

    def update_all_wells(
        self,
        assignments: dict[str, tuple[WellRole, str]],
    ) -> None:
        """
        Bulk update well appearances.

        Args:
            assignments: {well_name: (WellRole, label_text)}
        """
        for name, (role, label) in assignments.items():
            item = self._well_items.get(name)
            if item:
                item.set_role(role, label)

    def set_needle_position(
        self,
        x_mm: float | None,
        y_mm: float | None,
    ) -> None:
        """
        Update the real-time needle position overlay.

        Pass None to hide the needle indicator.
        """
        self._needle_x = x_mm
        self._needle_y = y_mm
        self._draw_needle()

    def fit_view(self) -> None:
        """Zoom to fit entire plate in view."""
        if self._scene.itemsBoundingRect().isEmpty():
            return
        rect = self._scene.itemsBoundingRect().adjusted(
            -PLATE_PADDING, -PLATE_PADDING, PLATE_PADDING, PLATE_PADDING)
        self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    # ── Scene Construction ────────────────────────────────────────

    def _rebuild_scene(self) -> None:
        """Clear and redraw the entire scene from plate data."""
        self._scene.clear()
        self._well_items.clear()
        self._needle_items.clear()

        if self._plate is None:
            return

        wells = self._plate.get_all_wells()
        if not wells:
            return

        # v7.4.8: custom plates have per-well diameters (well_diameter is
        # 0.0). Use a representative radius for the border/header layout and
        # render each well at its own size below.
        if self._plate.well_diameter > 0:
            radius_mm = self._plate.well_diameter / 2.0
        else:
            radius_mm = max((w.diameter for w in wells), default=6.0) / 2.0
        radius_scene = radius_mm * SCALE_FACTOR

        # Standard grids draw A/B/C + 1/2/3 headers; custom plates (no
        # uniform spacing) skip them — the labels would overlap at origin.
        draw_headers = (self._plate.well_spacing_x > 0
                        and self._plate.well_spacing_y > 0)

        # Draw plate outline (light border rectangle)
        min_x = min(w.x for w in wells) * SCALE_FACTOR
        min_y = min(w.y for w in wells) * SCALE_FACTOR
        max_x = max(w.x for w in wells) * SCALE_FACTOR
        max_y = max(w.y for w in wells) * SCALE_FACTOR
        plate_rect = QRectF(
            min_x - radius_scene - PLATE_PADDING,
            min_y - radius_scene - PLATE_PADDING,
            (max_x - min_x) + 2 * radius_scene + 2 * PLATE_PADDING,
            (max_y - min_y) + 2 * radius_scene + 2 * PLATE_PADDING,
        )
        plate_border = self._scene.addRect(
            plate_rect,
            QPen(QColor("#45475a"), 2),
            QBrush(QColor("#181825")),
        )
        plate_border.setZValue(-10)

        # Draw row headers (A, B, C, …)
        header_font = QFont("Segoe UI", HEADER_FONT_SIZE)
        for r in range(self._plate.rows if draw_headers else 0):
            label_text = ROW_LABELS[r] if r < len(ROW_LABELS) else str(r)
            y_pos = r * self._plate.well_spacing_y * SCALE_FACTOR
            text = self._scene.addText(label_text, header_font)
            text.setDefaultTextColor(QColor("#a6adc8"))
            text.setPos(
                -radius_scene - HEADER_MARGIN - text.boundingRect().width(),
                y_pos - text.boundingRect().height() / 2,
            )
            text.setZValue(5)

            # Row header click zone (invisible rect)
            zone = self._scene.addRect(
                QRectF(
                    -radius_scene - HEADER_MARGIN - 20,
                    y_pos - radius_scene,
                    HEADER_MARGIN + 20,
                    2 * radius_scene,
                ),
                QPen(Qt.PenStyle.NoPen),
                QBrush(Qt.BrushStyle.NoBrush),
            )
            zone.setData(0, f"row:{r}")
            zone.setAcceptHoverEvents(True)
            zone.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        # Draw column headers (1, 2, 3, …)
        for c in range(self._plate.cols if draw_headers else 0):
            label_text = str(c + 1)
            x_pos = c * self._plate.well_spacing_x * SCALE_FACTOR
            text = self._scene.addText(label_text, header_font)
            text.setDefaultTextColor(QColor("#a6adc8"))
            text.setPos(
                x_pos - text.boundingRect().width() / 2,
                -radius_scene - HEADER_MARGIN - text.boundingRect().height(),
            )
            text.setZValue(5)

            # Column header click zone
            zone = self._scene.addRect(
                QRectF(
                    x_pos - radius_scene,
                    -radius_scene - HEADER_MARGIN - 20,
                    2 * radius_scene,
                    HEADER_MARGIN + 20,
                ),
                QPen(Qt.PenStyle.NoPen),
                QBrush(Qt.BrushStyle.NoBrush),
            )
            zone.setData(0, f"col:{c}")
            zone.setAcceptHoverEvents(True)
            zone.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        # Draw wells — each at its own diameter (v7.4.8: custom plates /
        # flattened rosette sub-wells have mixed sizes).
        for well in wells:
            well_r = ((well.diameter / 2.0) * SCALE_FACTOR
                      if well.diameter > 0 else radius_scene)
            item = WellGraphicsItem(well, well_r)
            item.setZValue(1)
            self._scene.addItem(item)
            self._well_items[well.name] = item

        # Initial view fit (deferred for layout to settle)
        QTimer.singleShot(50, self.fit_view)

    # ── Needle Overlay ────────────────────────────────────────────

    def _draw_needle(self) -> None:
        """Draw or update the needle crosshair on the scene."""
        # Remove old needle items
        for item in self._needle_items:
            self._scene.removeItem(item)
        self._needle_items.clear()

        if self._needle_x is None or self._needle_y is None:
            return

        sx = self._needle_x * SCALE_FACTOR
        sy = self._needle_y * SCALE_FACTOR
        pen = QPen(QColor(NEEDLE_COLOR), 2, Qt.PenStyle.SolidLine)

        # Horizontal crosshair line
        h_line = self._scene.addLine(
            sx - NEEDLE_SIZE, sy, sx + NEEDLE_SIZE, sy, pen)
        h_line.setZValue(10)
        self._needle_items.append(h_line)

        # Vertical crosshair line
        v_line = self._scene.addLine(
            sx, sy - NEEDLE_SIZE, sx, sy + NEEDLE_SIZE, pen)
        v_line.setZValue(10)
        self._needle_items.append(v_line)

        # Center dot
        dot_size = 3
        dot = self._scene.addEllipse(
            sx - dot_size, sy - dot_size, dot_size * 2, dot_size * 2,
            QPen(QColor(NEEDLE_COLOR), 1),
            QBrush(QColor(NEEDLE_COLOR)),
        )
        dot.setZValue(10)
        self._needle_items.append(dot)

    # ── Selection Logic ───────────────────────────────────────────

    def _clear_selection(self, emit: bool = True) -> None:
        """Deselect all wells."""
        for name in self._selected_wells:
            item = self._well_items.get(name)
            if item:
                item.set_selected(False)
        self._selected_wells.clear()
        if emit:
            self.selection_changed.emit([])

    def _select_well(self, name: str) -> None:
        """Add a well to selection."""
        if name not in self._selected_wells:
            self._selected_wells.append(name)
            item = self._well_items.get(name)
            if item:
                item.set_selected(True)

    def _deselect_well(self, name: str) -> None:
        """Remove a well from selection."""
        if name in self._selected_wells:
            self._selected_wells.remove(name)
            item = self._well_items.get(name)
            if item:
                item.set_selected(False)

    def _toggle_well(self, name: str) -> None:
        """Toggle a well's selection state."""
        if name in self._selected_wells:
            self._deselect_well(name)
        else:
            self._select_well(name)

    def _select_range(self, from_name: str, to_name: str) -> None:
        """
        Shift+click range select: rectangle between two wells.
        """
        if self._plate is None:
            return
        try:
            w1 = self._plate.get_well_info(from_name)
            w2 = self._plate.get_well_info(to_name)
        except KeyError:
            return

        r_min, r_max = sorted([w1.row, w2.row])
        c_min, c_max = sorted([w1.col, w2.col])

        for name, item in self._well_items.items():
            if (r_min <= item.well_info.row <= r_max
                    and c_min <= item.well_info.col <= c_max):
                self._select_well(name)

    def _select_row(self, row_index: int) -> None:
        """Select all wells in a row."""
        for name, item in self._well_items.items():
            if item.well_info.row == row_index:
                self._select_well(name)

    def _select_column(self, col_index: int) -> None:
        """Select all wells in a column."""
        for name, item in self._well_items.items():
            if item.well_info.col == col_index:
                self._select_well(name)

    def _select_all(self) -> None:
        """Select all wells."""
        for name in self._well_items:
            self._select_well(name)
        self.selection_changed.emit(list(self._selected_wells))

    def _well_at_pos(self, scene_pos: QPointF) -> str | None:
        """Find the well name at a scene position, or None."""
        items = self._scene.items(scene_pos)
        for item in items:
            if isinstance(item, WellGraphicsItem):
                return item.well_name
        return None

    def _header_at_pos(self, scene_pos: QPointF) -> tuple[str, int] | None:
        """
        Check if a scene position is over a row/col header zone.

        Returns ("row", index) or ("col", index) or None.
        """
        items = self._scene.items(scene_pos)
        for item in items:
            if isinstance(item, QGraphicsRectItem):
                data = item.data(0)
                if isinstance(data, str) and ":" in data:
                    kind, idx_str = data.split(":", 1)
                    try:
                        return (kind, int(idx_str))
                    except ValueError:
                        pass
        return None

    # ── Mouse Events ──────────────────────────────────────────────

    def mousePressEvent(self, event: QMouseEvent) -> None:
        scene_pos = self.mapToScene(event.pos())
        ctrl = bool(event.modifiers() & Qt.KeyboardModifier.ControlModifier)
        shift = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)

        if event.button() == Qt.MouseButton.LeftButton:
            # Check row/col header click
            header = self._header_at_pos(scene_pos)
            if header:
                kind, idx = header
                if not ctrl:
                    self._clear_selection(emit=False)
                if kind == "row":
                    self._select_row(idx)
                elif kind == "col":
                    self._select_column(idx)
                self._restore_well_borders()  # v7.2.5: restore status colors
                self._restore_well_borders()  # v7.2.5: restore status colors
                self.selection_changed.emit(list(self._selected_wells))
                return

            # Check well click
            well_name = self._well_at_pos(scene_pos)
            if well_name:
                if ctrl:
                    self._toggle_well(well_name)
                elif shift and self._last_click_well:
                    self._clear_selection(emit=False)
                    self._select_range(self._last_click_well, well_name)
                else:
                    self._clear_selection(emit=False)
                    self._select_well(well_name)
                self._last_click_well = well_name
                self.selection_changed.emit(list(self._selected_wells))
            else:
                # Start rubber-band on empty area
                if not ctrl and not shift:
                    self._clear_selection(emit=False)
                self._rubber_band_origin = scene_pos
                self._rubber_band_rect = self._scene.addRect(
                    QRectF(scene_pos, QSizeF(0, 0)),
                    QPen(QColor(SELECTED_BORDER_COLOR), 1, Qt.PenStyle.DashLine),
                    QBrush(QColor(SELECTED_BORDER_COLOR + "30")),
                )
                self._rubber_band_rect.setZValue(20)

        elif event.button() == Qt.MouseButton.RightButton:
            # Context menu
            if self._selected_wells:
                self.context_menu_requested.emit(
                    list(self._selected_wells), scene_pos)

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._rubber_band_origin and self._rubber_band_rect:
            scene_pos = self.mapToScene(event.pos())
            rect = QRectF(self._rubber_band_origin, scene_pos).normalized()
            self._rubber_band_rect.setRect(rect)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if (event.button() == Qt.MouseButton.LeftButton
                and self._rubber_band_rect):
            # Finalize rubber-band selection
            rect = self._rubber_band_rect.rect()
            for name, item in self._well_items.items():
                well_center = QPointF(
                    item.well_info.x * SCALE_FACTOR,
                    item.well_info.y * SCALE_FACTOR,
                )
                if rect.contains(well_center):
                    self._select_well(name)

            # Cleanup rubber-band
            self._scene.removeItem(self._rubber_band_rect)
            self._rubber_band_rect = None
            self._rubber_band_origin = None
            self.selection_changed.emit(list(self._selected_wells))

        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        scene_pos = self.mapToScene(event.pos())
        well_name = self._well_at_pos(scene_pos)
        if well_name:
            self.well_double_clicked.emit(well_name)
        super().mouseDoubleClickEvent(event)

    # ── Wheel Zoom ────────────────────────────────────────────────

    def wheelEvent(self, event: QWheelEvent) -> None:
        factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(factor, factor)
        else:
            self.scale(1 / factor, 1 / factor)

    # ── Keyboard Shortcuts ────────────────────────────────────────

    def _setup_shortcuts(self) -> None:
        shortcut = QShortcut(QKeySequence.StandardKey.SelectAll, self)
        shortcut.activated.connect(self._select_all)

    # ── Resize ────────────────────────────────────────────────────

    # ── Role-based well appearance (v7.3.1-rolecolors) ───────────

    def update_all_wells(
        self,
        appearances: "dict[str, tuple]",
    ) -> None:
        """
        Set fill colour for every well from its role.

        appearances: {well_name: (WellRole, display_label)}
        Falls back gracefully when ROLE_COLORS is unavailable.
        """
        try:
            from SupportClasses.PhysicalModels import ROLE_COLORS
        except ImportError:
            ROLE_COLORS = {}

        for name, (role, label) in appearances.items():
            item = self._well_items.get(name)
            if item is None:
                continue
            hex_color = ROLE_COLORS.get(role, "#585b70")
            from PySide6.QtGui import QBrush, QColor
            item.setBrush(QBrush(QColor(hex_color)))
            item.setToolTip(f"{name}: {label or role.value if hasattr(role, 'value') else str(role)}")

    def set_well_color(self, name: str, hex_color: str) -> None:
        """Set fill colour for a single well (hex string)."""
        from PySide6.QtGui import QBrush, QColor
        item = self._well_items.get(name)
        if item:
            item.setBrush(QBrush(QColor(hex_color)))

    def set_well_border_color(self, name: str, hex_color: str) -> None:
        """Set border (pen) colour for a single well (hex string)."""
        from PySide6.QtGui import QPen, QColor
        item = self._well_items.get(name)
        if item:
            pen = item.pen()
            pen.setColor(QColor(hex_color))
            pen.setWidthF(2.0 if hex_color not in ("#585b70", "#45475a") else 0.5)
            item.setPen(pen)

    def update_well_rosettes(
        self,
        rosette_map: "dict[str, object]",
        plate: "object | None" = None,
    ) -> None:
        """
        Draw rosette subwells inside wells that have a rosette attached.

        rosette_map: {well_name: RosetteInsert | None}
        plate: WellPlate used to look up well diameter (optional)
        """
        # Remove any existing rosette child items
        for name, items in list(getattr(self, "_rosette_items", {}).items()):
            for it in items:
                self._scene.removeItem(it)
        self._rosette_items = {}

        if not rosette_map:
            return

        try:
            from SupportClasses.PhysicalModels import ROLE_COLORS, WellRole
        except ImportError:
            return
        from PySide6.QtGui import QBrush, QColor, QPen

        # Determine scale from existing items
        if not self._well_items or not self._plate:
            return

        # Use first well to estimate pixel-to-mm ratio
        sample_well = next(iter(self._plate.get_all_wells()), None)
        if sample_well is None:
            return
        sample_item = self._well_items.get(sample_well.name)
        if sample_item is None:
            return

        # pixel radius of a well in scene coordinates
        well_r_px = sample_item.rect().width() / 2.0
        # mm radius of a well
        from SupportClasses.WellPlate import PLATE_DEFINITIONS
        defn = PLATE_DEFINITIONS.get(self._plate.format_key, {})
        well_diam_mm = defn.get("well_diameter_mm", 6.4) if defn else 6.4
        well_r_mm = well_diam_mm / 2.0
        px_per_mm = well_r_px / max(well_r_mm, 0.01)

        for well_name, rosette in rosette_map.items():
            if rosette is None:
                continue
            parent_item = self._well_items.get(well_name)
            if parent_item is None:
                continue
            cx = parent_item.rect().center().x()
            cy = parent_item.rect().center().y()

            child_items = []
            for sw in getattr(rosette, "subwells", []):
                # Sub-well offset in mm → pixels
                try:
                    dx_mm, dy_mm = rosette.get_subwell_xy(sw.index)
                except Exception:
                    import math
                    angle_r = math.radians(getattr(sw, "angle_deg", 0))
                    r_mm    = getattr(sw, "radial_offset_mm", 0)
                    dx_mm   = r_mm * math.sin(angle_r)
                    dy_mm   = -r_mm * math.cos(angle_r)

                sw_diam_mm = getattr(sw, "diameter_mm", 1.5)
                sw_r_px    = max((sw_diam_mm / 2.0) * px_per_mm, 1.5)
                scx        = cx + dx_mm * px_per_mm
                scy        = cy + dy_mm * px_per_mm

                # Pick colour from subwell role if available
                sw_role = None
                sw_color = "#a6adc8"
                it = self._scene.addEllipse(
                    scx - sw_r_px, scy - sw_r_px,
                    2 * sw_r_px, 2 * sw_r_px,
                    QPen(QColor("#1e1e2e"), 0.5),
                    QBrush(QColor(sw_color)),
                )
                it.setZValue(2)
                child_items.append(it)

            self._rosette_items[well_name] = child_items


    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.fit_view()


# ═══════════════════════════════════════════════════════════════════
# Legend Widget
# ═══════════════════════════════════════════════════════════════════

    def zoom_in(self):
        """Zoom in by 25%."""
        self.scale(1.25, 1.25)

    def zoom_out(self):
        """Zoom out by 20%."""
        self.scale(0.8, 0.8)

    def reset_zoom(self):
        """Reset to fit-in-view."""
        self.resetTransform()
        self.fit_view()


    def update_well_status(self, status_map: dict[str, str]):
        """
        v7.2.5: Update well border colors based on assignment status.

        Args:
            status_map: {well_name: "ready" | "incomplete" | "empty"}

        Border colors:
            ready      → green (#a6e3a1) — fully configured
            incomplete → red (#f38ba8) — has role but missing details
            empty      → gray (#585b70) — no assignment
        """
        STATUS_COLORS = {
            "ready":      "#a6e3a1",  # Green
            "incomplete": "#f38ba8",  # Red
            "empty":      "#585b70",  # Gray
        }

        for well_name, status in status_map.items():
            item = self._well_items.get(well_name)
            if item is None:
                continue

            color = STATUS_COLORS.get(status, STATUS_COLORS["empty"])

            # Only update border if well is not currently selected
            # (selected wells keep their selection highlight)
            if well_name not in self._selected_wells:
                pen = QPen(QColor(color), WELL_BORDER_WIDTH)
                item.setPen(pen)

            # Store status for re-application after selection changes
            if not hasattr(self, '_well_status'):
                self._well_status = {}
            self._well_status[well_name] = status

    def _restore_well_borders(self):
        """v7.2.5: Restore status-based borders after selection changes."""
        if not hasattr(self, '_well_status'):
            return

        STATUS_COLORS = {
            "ready":      "#a6e3a1",
            "incomplete": "#f38ba8",
            "empty":      "#585b70",
        }

        for well_name, item in self._well_items.items():
            if well_name in self._selected_wells:
                continue  # Keep selection highlight
            status = self._well_status.get(well_name, "empty")
            color = STATUS_COLORS.get(status, STATUS_COLORS["empty"])
            pen = QPen(QColor(color), WELL_BORDER_WIDTH)
            item.setPen(pen)


class WellRoleLegend(QWidget):
    """
    Compact horizontal legend showing well role colors.

    Each role displays a colored dot + label.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(12)

        for role in WellRole:
            dot_label = QLabel(f"● {role.value.capitalize()}")
            color = ROLE_COLORS.get(role, "#585b70")
            dot_label.setStyleSheet(
                f"color: {color}; font-size: 11px; font-weight: bold;")
            layout.addWidget(dot_label)

        layout.addStretch()
