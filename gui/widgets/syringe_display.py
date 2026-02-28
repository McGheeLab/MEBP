"""
syringe_display.py — Visual syringe fill-level widget for MEBP v7.1.

Displays a vertical syringe representation showing:
- Stacked fluid column (oil/buffer/ink) proportional to volume
- Current plunger position
- Total capacity + current ink volume in µL
- Ink name + color
- Printing mode indicator (Incremental / Continuous)
- Flow rate indicator (animated bar during active printing)
- Warning icon if ink is low or buffer is depleted

    ┌──────────────────┐
    │  ░░░░░░░░░░░░░░  │  ← mineral oil (gray)
    │  ░░░░░░░░░░░░░░  │
    │  ████████████████  │  ← buffer (light blue)
    │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │  ← ink (ink's display color)
    │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │
    └───────┤├─────────┘  ← needle tip
         P1 │ 100µL
     Ink: 42.3 µL Hydrogel A
     Buf: 5.0 µL  Mode: Incr.

Session H — Task P6.6.
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy
from PySide6.QtCore import Qt, QRectF, QTimer
from PySide6.QtGui import (
    QColor, QPen, QBrush, QPainter, QFont, QLinearGradient,
)

from gui.styles import COLORS
from SupportClasses.PhysicalModels import (
    PumpLoadout, FluidColumn, SyringeSpec, PrintingMode,
)

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

BARREL_WIDTH = 50         # px
BARREL_HEIGHT = 140       # px
NEEDLE_WIDTH = 6          # px
NEEDLE_HEIGHT = 18        # px
PLUNGER_HEIGHT = 6        # px
CORNER_RADIUS = 4

LOW_INK_THRESHOLD_UL = 5.0
LOW_BUFFER_THRESHOLD_UL = 1.0

# Fluid layer colors
OIL_COLOR = "#6c7086"
BUFFER_COLOR = "#89b4fa"
EMPTY_COLOR = "#313244"
DEFAULT_INK_COLOR = "#a6e3a1"
PLUNGER_COLOR = "#cdd6f4"
WARNING_COLOR = "#f38ba8"
BARREL_BORDER_COLOR = "#585b70"
NEEDLE_COLOR = "#a6adc8"

# Flow animation
FLOW_ANIM_INTERVAL_MS = 80
FLOW_STRIPE_COUNT = 4


# ═══════════════════════════════════════════════════════════════════
# Syringe Barrel Canvas (Custom Paint)
# ═══════════════════════════════════════════════════════════════════

class SyringeBarrelWidget(QWidget):
    """
    Custom-painted vertical syringe barrel with fluid column.

    Renders the barrel outline, stacked fluid segments (oil/buffer/ink/empty),
    plunger bar, needle tip, and optional flow animation stripes.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumSize(BARREL_WIDTH + 20, BARREL_HEIGHT + NEEDLE_HEIGHT + 30)
        self.setMaximumWidth(BARREL_WIDTH + 30)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)

        # State
        self._fractions: dict[str, float] = {
            "oil": 0.0, "buffer": 0.0, "ink": 0.0, "empty": 1.0,
        }
        self._ink_color: str = DEFAULT_INK_COLOR
        self._is_flowing: bool = False
        self._flow_offset: int = 0
        self._warning: bool = False

        # Flow animation timer
        self._flow_timer = QTimer(self)
        self._flow_timer.timeout.connect(self._advance_flow)

    def set_fractions(
        self,
        fractions: dict[str, float],
        ink_color: str = DEFAULT_INK_COLOR,
    ) -> None:
        """Update fluid column fractions and redraw."""
        self._fractions = fractions
        self._ink_color = ink_color
        self.update()

    def set_flowing(self, flowing: bool) -> None:
        """Enable/disable flow animation."""
        self._is_flowing = flowing
        if flowing and not self._flow_timer.isActive():
            self._flow_timer.start(FLOW_ANIM_INTERVAL_MS)
        elif not flowing:
            self._flow_timer.stop()
            self._flow_offset = 0
            self.update()

    def set_warning(self, warning: bool) -> None:
        """Enable/disable warning border (low ink / depleted buffer)."""
        self._warning = warning
        self.update()

    def _advance_flow(self) -> None:
        self._flow_offset = (self._flow_offset + 1) % (FLOW_STRIPE_COUNT * 2)
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # Positioning
        barrel_x = (w - BARREL_WIDTH) // 2
        barrel_y = 10
        barrel_w = BARREL_WIDTH
        barrel_h = BARREL_HEIGHT

        needle_x = w // 2 - NEEDLE_WIDTH // 2
        needle_y = barrel_y + barrel_h
        needle_h = NEEDLE_HEIGHT

        # ── Draw barrel background ───────────────────────────
        painter.setBrush(QBrush(QColor(EMPTY_COLOR)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(
            barrel_x, barrel_y, barrel_w, barrel_h,
            CORNER_RADIUS, CORNER_RADIUS,
        )

        # ── Draw fluid segments (top-down: oil → buffer → ink → empty) ──
        y_cursor = barrel_y
        segment_order = ["oil", "buffer", "ink", "empty"]
        colors = {
            "oil": QColor(OIL_COLOR),
            "buffer": QColor(BUFFER_COLOR),
            "ink": QColor(self._ink_color),
            "empty": QColor(EMPTY_COLOR),
        }

        for key in segment_order:
            frac = self._fractions.get(key, 0.0)
            seg_h = int(frac * barrel_h)
            if seg_h <= 0:
                continue

            color = colors[key]
            painter.setBrush(QBrush(color))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(barrel_x + 1, int(y_cursor), barrel_w - 2, seg_h)
            y_cursor += seg_h

        # ── Flow animation stripes (ink section only) ────────
        if self._is_flowing:
            ink_frac = self._fractions.get("ink", 0.0)
            if ink_frac > 0:
                oil_h = int(self._fractions.get("oil", 0.0) * barrel_h)
                buf_h = int(self._fractions.get("buffer", 0.0) * barrel_h)
                ink_top = barrel_y + oil_h + buf_h
                ink_h = int(ink_frac * barrel_h)

                stripe_pen = QPen(QColor(255, 255, 255, 40), 2)
                painter.setPen(stripe_pen)
                spacing = max(ink_h // FLOW_STRIPE_COUNT, 4)
                for i in range(FLOW_STRIPE_COUNT):
                    sy = ink_top + (i * spacing + self._flow_offset * 2) % max(ink_h, 1)
                    if ink_top <= sy < ink_top + ink_h:
                        painter.drawLine(
                            barrel_x + 4, int(sy),
                            barrel_x + barrel_w - 4, int(sy),
                        )

        # ── Plunger (top of oil) ─────────────────────────────
        oil_frac = self._fractions.get("oil", 0.0)
        plunger_y = barrel_y + max(0, int(oil_frac * barrel_h) - PLUNGER_HEIGHT)
        painter.setBrush(QBrush(QColor(PLUNGER_COLOR)))
        painter.setPen(QPen(QColor(BARREL_BORDER_COLOR), 1))
        painter.drawRect(barrel_x + 2, plunger_y, barrel_w - 4, PLUNGER_HEIGHT)

        # Plunger rod (extends above barrel)
        rod_x = w // 2
        painter.setPen(QPen(QColor(PLUNGER_COLOR), 3))
        painter.drawLine(rod_x, 0, rod_x, plunger_y + PLUNGER_HEIGHT // 2)

        # ── Barrel border ────────────────────────────────────
        border_color = QColor(WARNING_COLOR) if self._warning else QColor(BARREL_BORDER_COLOR)
        border_width = 2.5 if self._warning else 1.5
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(border_color, border_width))
        painter.drawRoundedRect(
            barrel_x, barrel_y, barrel_w, barrel_h,
            CORNER_RADIUS, CORNER_RADIUS,
        )

        # ── Needle tip ───────────────────────────────────────
        painter.setBrush(QBrush(QColor(NEEDLE_COLOR)))
        painter.setPen(QPen(QColor(BARREL_BORDER_COLOR), 1))
        painter.drawRect(needle_x, needle_y, NEEDLE_WIDTH, needle_h)

        # Needle taper
        taper_y = needle_y + needle_h
        painter.setPen(QPen(QColor(NEEDLE_COLOR), 2))
        painter.drawLine(w // 2, taper_y, w // 2, taper_y + 4)

        painter.end()


# ═══════════════════════════════════════════════════════════════════
# Complete Syringe Display Widget
# ═══════════════════════════════════════════════════════════════════

class SyringeDisplayWidget(QWidget):
    """
    Full syringe display: barrel visualization + text labels.

    Shows pump ID, capacity, ink volume, ink name, buffer volume,
    printing mode, and warning indicators.
    """

    def __init__(
        self,
        pump_id: str = "P1",
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._pump_id = pump_id
        self._pump_loadout: PumpLoadout | None = None
        self.setMinimumWidth(80)
        self.setMaximumWidth(100)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        layout.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        # Barrel visualization
        self._barrel = SyringeBarrelWidget()
        layout.addWidget(self._barrel, stretch=1)

        # Pump ID + capacity
        self._id_label = QLabel(pump_id)
        self._id_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._id_label.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: bold; font-size: 12px;")
        layout.addWidget(self._id_label)

        # Volume readout
        self._vol_label = QLabel("— µL")
        self._vol_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._vol_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px;")
        layout.addWidget(self._vol_label)

        # Ink info
        self._ink_label = QLabel("")
        self._ink_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._ink_label.setStyleSheet(
            f"color: {COLORS['text']}; font-size: 9px;")
        self._ink_label.setWordWrap(True)
        layout.addWidget(self._ink_label)

        # Mode + buffer
        self._mode_label = QLabel("")
        self._mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._mode_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9px;")
        layout.addWidget(self._mode_label)

        # Warning label
        self._warn_label = QLabel("")
        self._warn_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._warn_label.setStyleSheet(
            f"color: {WARNING_COLOR}; font-size: 9px; font-weight: bold;")
        self._warn_label.setVisible(False)
        layout.addWidget(self._warn_label)

    # ── Public API ────────────────────────────────────────────────

    def set_pump_loadout(self, loadout: PumpLoadout) -> None:
        """Update display from a PumpLoadout object."""
        self._pump_loadout = loadout
        self._refresh()

    def set_flowing(self, flowing: bool) -> None:
        """Enable/disable flow animation."""
        self._barrel.set_flowing(flowing)

    def update_fluid_column(self, fluid_column: FluidColumn) -> None:
        """Quick update just the fluid column (during active print)."""
        if self._pump_loadout:
            self._pump_loadout.fluid_column = fluid_column
            self._refresh()

    def _refresh(self) -> None:
        """Redraw everything from current pump loadout."""
        loadout = self._pump_loadout
        if loadout is None or loadout.syringe is None:
            self._id_label.setText(self._pump_id)
            self._vol_label.setText("No syringe")
            self._ink_label.setText("")
            self._mode_label.setText("")
            self._barrel.set_fractions({"empty": 1.0})
            self._barrel.set_warning(False)
            self._warn_label.setVisible(False)
            return

        syringe = loadout.syringe
        fc = loadout.fluid_column

        # Update barrel
        fractions = fc.volume_fractions(syringe)
        ink_color = DEFAULT_INK_COLOR
        if fc.ink_spec and fc.ink_spec.display_color:
            ink_color = fc.ink_spec.display_color
        self._barrel.set_fractions(fractions, ink_color)

        # ID + capacity
        self._id_label.setText(f"{self._pump_id} │ {syringe.volume_uL}µL")

        # Volume readout
        ink_vol = fc.ink_volume_uL
        self._vol_label.setText(f"Ink: {ink_vol:.1f} µL")

        # Ink name
        if fc.ink_spec:
            self._ink_label.setText(fc.ink_spec.name)
            self._ink_label.setStyleSheet(
                f"color: {ink_color}; font-size: 9px; font-weight: bold;")
        else:
            self._ink_label.setText("No ink")
            self._ink_label.setStyleSheet(
                f"color: {COLORS['overlay0']}; font-size: 9px;")

        # Mode + buffer
        mode_str = loadout.printing_mode.value[:4].capitalize()
        buf_str = f"Buf: {fc.buffer_volume_uL:.1f}µL"
        self._mode_label.setText(f"{buf_str}  {mode_str}.")

        # Warnings
        warnings = []
        if ink_vol <= LOW_INK_THRESHOLD_UL and ink_vol > 0:
            warnings.append("Low ink")
        elif ink_vol <= 0:
            warnings.append("Empty")
        if fc.buffer_volume_uL <= LOW_BUFFER_THRESHOLD_UL and fc.has_buffer:
            warnings.append("Low buf")

        has_warning = len(warnings) > 0
        self._barrel.set_warning(has_warning)
        self._warn_label.setVisible(has_warning)
        if warnings:
            self._warn_label.setText("⚠ " + ", ".join(warnings))


# ═══════════════════════════════════════════════════════════════════
# Syringe Status Row (3 syringes side by side)
# ═══════════════════════════════════════════════════════════════════

class SyringeStatusPanel(QWidget):
    """
    Horizontal row of 3 SyringeDisplayWidgets for P1/P2/P3.

    Used in the Print Monitor page to show all pump states at once.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        from PySide6.QtWidgets import QHBoxLayout

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        self.syringes: dict[str, SyringeDisplayWidget] = {}
        for pid in ["P1", "P2", "P3"]:
            widget = SyringeDisplayWidget(pump_id=pid)
            self.syringes[pid] = widget
            layout.addWidget(widget)

    def update_from_workspace(
        self,
        pumps: dict[str, PumpLoadout],
    ) -> None:
        """Update all syringe displays from workspace pump loadouts."""
        for pid, widget in self.syringes.items():
            loadout = pumps.get(pid)
            if loadout:
                widget.set_pump_loadout(loadout)

    def set_flowing(self, pump_id: str, flowing: bool) -> None:
        """Set flow animation for a specific pump."""
        widget = self.syringes.get(pump_id)
        if widget:
            widget.set_flowing(flowing)
