"""
pump_rack.py — three-column pump rack with sideways syringes.

Each pump occupies one column card. The barrel is drawn horizontally
(rotated 90° from the printer-monitor variant) so the section stays
short. Inside each column, top to bottom:

      ▌━━━━━━━━━━━━━━━━━━━━━┃░░░░░░ ▓▓▓▓▓ █████ ╞═╡──▶
                                                        ← needle tip
                         [P1]              ← PID chip
                       42.3 µL             ← LIVE plunger fill
                     100 µL · Ink A        ← capacity + ink subtext

Three columns are arranged side-by-side in :class:`PumpRack`, giving
the same "row with three columns" layout but with horizontal barrels
that keep the total section height low.

The primary readout reflects the LIVE plunger fill (how full the
syringe is right now, derived from the plunger position via the
controller's plunger calibration) — see :meth:`PumpColumn.set_live_fill`.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf, sp
from gui.styles import COLORS
from SupportClasses.PhysicalModels import FluidColumn, PumpLoadout, SyringeSpec

logger = logging.getLogger(__name__)

# ── Tuning ─────────────────────────────────────────────────────────

BARREL_HEIGHT = 22
PLUNGER_ROD_LENGTH = 24
NEEDLE_LENGTH = 16
NEEDLE_THICKNESS = 5
PLUNGER_DISC_WIDTH = 5
HUB_WIDTH = 8

LOW_INK_THRESHOLD_UL = 5.0
LOW_BUFFER_THRESHOLD_UL = 1.0

OIL_COLOR = "#6c7086"
BUFFER_COLOR = "#89b4fa"
EMPTY_COLOR = "#1e1e2e"
DEFAULT_INK_COLOR = "#a6e3a1"
PLUNGER_COLOR = "#cdd6f4"
WARNING_COLOR = "#f38ba8"
BARREL_BORDER_COLOR = "#45475a"
NEEDLE_COLOR = "#a6adc8"


def _qc(name: str, alpha: int | None = None) -> QColor:
    c = QColor(COLORS[name])
    if alpha is not None:
        c.setAlpha(alpha)
    return c


# ── Barrel widget ──────────────────────────────────────────────────


class HorizontalBarrel(QWidget):
    """Custom-painted horizontal syringe barrel.

    Left → right: plunger knob, plunger rod, barrel (oil → buffer →
    ink → empty), hub (Luer collar), tapered needle tip.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(s(BARREL_HEIGHT + 10))
        self.setMaximumHeight(s(BARREL_HEIGHT + 14))
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._fractions: dict[str, float] = {
            "oil": 0.0, "buffer": 0.0, "ink": 0.0, "empty": 1.0,
        }
        self._ink_color = DEFAULT_INK_COLOR
        self._warning = False
        self._enabled = True

    def set_fractions(self, fractions: dict[str, float],
                      ink_color: str = DEFAULT_INK_COLOR) -> None:
        self._fractions = fractions
        self._ink_color = ink_color
        self.update()

    def set_warning(self, warning: bool) -> None:
        self._warning = warning
        self.update()

    def set_enabled_state(self, enabled: bool) -> None:
        self._enabled = enabled
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        barrel_h = s(BARREL_HEIGHT)
        barrel_y = (h - barrel_h) / 2.0
        center_y = h / 2.0

        rod_w = s(PLUNGER_ROD_LENGTH)
        hub_w = s(HUB_WIDTH)
        needle_w = s(NEEDLE_LENGTH)

        barrel_x = rod_w
        barrel_w = max(s(40), w - rod_w - hub_w - needle_w - s(4))
        barrel_right = barrel_x + barrel_w

        radius = barrel_h / 2.5
        body_rect = QRectF(barrel_x, barrel_y, barrel_w, barrel_h)

        # ── Plunger rod ──────────────────────────────────────
        rod_color = (QColor(PLUNGER_COLOR) if self._enabled
                     else _qc('overlay0'))
        pen = QPen(rod_color, max(2, s(2)))
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(pen)
        p.drawLine(QPointF(s(4), center_y),
                   QPointF(barrel_x + s(2), center_y))
        knob_r = s(4)
        p.setBrush(QBrush(rod_color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(s(4), center_y), knob_r, knob_r)

        # ── Barrel body (empty bg) ───────────────────────────
        bg = QColor(EMPTY_COLOR) if self._enabled else _qc('mantle')
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(bg))
        p.drawRoundedRect(body_rect, radius, radius)

        # ── Fluid segments (left → right) ────────────────────
        if self._enabled:
            seg_pad = 1
            clip = body_rect.adjusted(seg_pad, seg_pad, -seg_pad, -seg_pad)
            clip_path = QPainterPath()
            inset = max(0.5, radius - seg_pad)
            clip_path.addRoundedRect(clip, inset, inset)
            p.save()
            p.setClipPath(clip_path)
            x_cursor = barrel_x
            for key, color in (
                ("oil",    QColor(OIL_COLOR)),
                ("buffer", QColor(BUFFER_COLOR)),
                ("ink",    QColor(self._ink_color)),
            ):
                frac = self._fractions.get(key, 0.0)
                seg_w = int(frac * barrel_w)
                if seg_w <= 0:
                    continue
                p.setBrush(QBrush(color))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawRect(QRectF(
                    x_cursor, barrel_y + seg_pad,
                    seg_w, barrel_h - 2 * seg_pad))
                x_cursor += seg_w
            p.restore()

            # ── Plunger disc (at left of oil) ────────────────
            oil_frac = self._fractions.get("oil", 0.0)
            disc_w = s(PLUNGER_DISC_WIDTH)
            disc_x = barrel_x + max(0, int(oil_frac * barrel_w) - disc_w)
            p.setBrush(QBrush(QColor(PLUNGER_COLOR)))
            p.setPen(QPen(QColor(BARREL_BORDER_COLOR), 1))
            p.drawRect(QRectF(disc_x, barrel_y + 1,
                              disc_w, barrel_h - 2))

        # ── Barrel border ────────────────────────────────────
        border_color = (QColor(WARNING_COLOR) if self._warning
                        else QColor(BARREL_BORDER_COLOR))
        border_w = 1.5 if self._warning else 1.0
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(border_color, border_w))
        p.drawRoundedRect(body_rect, radius, radius)

        # ── Hub (Luer collar) ────────────────────────────────
        hub_h = s(NEEDLE_THICKNESS) + s(4)
        hub_y = center_y - hub_h / 2.0
        hub_rect = QRectF(barrel_right, hub_y, hub_w, hub_h)
        p.setBrush(QBrush(_qc('surface2')))
        p.setPen(QPen(QColor(BARREL_BORDER_COLOR), 1))
        p.drawRoundedRect(hub_rect, hub_h / 2.5, hub_h / 2.5)

        # ── Needle (tapered triangle) ────────────────────────
        needle_color = (QColor(NEEDLE_COLOR) if self._enabled
                        else _qc('overlay0'))
        nt = s(NEEDLE_THICKNESS)
        needle_x_start = barrel_right + hub_w
        # Half-body (rectangle)
        body_n = QRectF(needle_x_start, center_y - nt / 2.0,
                        needle_w * 0.5, nt)
        p.setBrush(QBrush(needle_color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(body_n)
        # Taper
        tri = QPolygonF([
            QPointF(needle_x_start + needle_w * 0.5,
                    center_y - nt / 2.0),
            QPointF(needle_x_start + needle_w * 0.5,
                    center_y + nt / 2.0),
            QPointF(needle_x_start + needle_w, center_y),
        ])
        p.setBrush(QBrush(needle_color))
        p.drawPolygon(tri)

        p.end()


# ── Pump column (one card) ────────────────────────────────────────


class PumpColumn(QWidget):
    """A single pump's card — horizontal barrel on top, readouts below."""

    def __init__(self, pump_id: str = "P1", parent: QWidget | None = None):
        super().__init__(parent)
        self._pump_id = pump_id
        self._loadout: PumpLoadout | None = None
        # v7.5.x perf: set_live_fill runs on EVERY 300ms status tick AND every
        # 33ms motion tick (jog pump panel + the always-mounted Custom "Syringe
        # overview" section). setStyleSheet forces a full QSS re-parse of the
        # widget subtree — doing it 4×/column/tick made the UI uniformly
        # sluggish. Cache the last STYLE state so the (expensive) restyle only
        # runs when the visual style actually changes; text + bar fraction (the
        # only things that change every tick) update unconditionally + cheaply.
        self._style_state: tuple | None = None

        self._card = QFrame(self)
        self._card.setObjectName("pumpColCard")
        self._refresh_card_style(active=False)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._card)

        body = QVBoxLayout(self._card)
        body.setContentsMargins(s(10), s(8), s(10), s(10))
        body.setSpacing(s(4))

        # Barrel row (full-width inside the card)
        self._barrel = HorizontalBarrel()
        body.addWidget(self._barrel)

        # Chip — small centered pill
        chip_row = QHBoxLayout()
        chip_row.setContentsMargins(0, s(2), 0, 0)
        chip_row.addStretch(1)
        self._chip = QLabel(pump_id)
        self._chip.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._chip.setFixedSize(s(34), s(18))
        self._refresh_chip_style(active=False, ink_color=None)
        chip_row.addWidget(self._chip)
        chip_row.addStretch(1)
        body.addLayout(chip_row)

        # Primary readout
        self._vol_label = QLabel("—")
        self._vol_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._vol_label.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(13)}pt;"
            f"font-weight: 700;"
            f"letter-spacing: 0.3px;"
        )
        body.addWidget(self._vol_label)

        # Subtext
        self._sub_label = QLabel("")
        self._sub_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._sub_label.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(8)}pt;"
        )
        self._sub_label.setWordWrap(False)
        body.addWidget(self._sub_label)

    # ── Public API ─────────────────────────────────────────────────

    def set_pump_loadout(self, loadout: PumpLoadout) -> None:
        self._loadout = loadout
        self._style_state = None  # legacy path → force the next restyle
        self._refresh()

    def update_fluid_column(self, fc: FluidColumn) -> None:
        if self._loadout:
            self._loadout.fluid_column = fc
            self._style_state = None
            self._refresh()

    def set_unconfigured(self) -> None:
        self._loadout = None
        # Per-tick guard (see set_live_fill): only restyle on the first
        # transition into the unconfigured state — this runs every tick for a
        # pump with no syringe.
        if self._style_state == ("unconfigured",):
            return
        self._style_state = ("unconfigured",)
        self._refresh()

    def set_live_fill(
        self,
        *,
        fill_uL: float | None,
        capacity_uL: float | None,
        raw_mm: float | None,
        syringe: SyringeSpec | None,
        ink_spec=None,
        calibrated: bool,
    ) -> None:
        """v7.5.x: show the LIVE plunger fill — how much fluid is in the
        syringe right now, derived from the live plunger position via the
        controller's plunger calibration (0 = empty/fully dispensed →
        capacity = full/fully aspirated). Mirrors the control-panel P1/P2/P3
        readout and replaces the static, never-updated ``FluidColumn`` view.

        * ``calibrated`` + ``fill_uL`` known → µL fill + a proportional bar.
        * configured but un-calibrated → the raw plunger position (mm), which
          still updates live, plus a "calibrate plunger" hint.
        * no syringe configured → the "not configured" placeholder.
        """
        self._loadout = None  # live-fill path owns the display now

        if syringe is None:
            self.set_unconfigured()
            return

        # InkSpec's hex display colour is `.color` (NOT `display_color`, which
        # is not an attribute — the getattr would silently fall through).
        ink_color = getattr(ink_spec, "color", None) if ink_spec is not None \
            else None
        ink_name = getattr(ink_spec, "name", None) if ink_spec is not None \
            else None
        # A pump with no ink assigned still has a meaningful fill level — show
        # it in a neutral accent rather than implying ink is loaded.
        accent = ink_color if ink_color \
            else COLORS.get("mauve", DEFAULT_INK_COLOR)

        # Nominal capacity: calibrated stroke if available, else the syringe's
        # rated volume (so the bar/subtext still read sensibly pre-calibration).
        nominal_cap = (capacity_uL if capacity_uL and capacity_uL > 0
                       else float(syringe.volume_uL))

        live = bool(calibrated and fill_uL is not None)
        warning = bool(live and fill_uL <= LOW_INK_THRESHOLD_UL)
        # Everything that changes an actual STYLESHEET (accent, warning colour,
        # calibrated-vs-raw layout, enabled state). Gate the QSS re-parse on it.
        style_state = ("live" if live else "raw", accent, warning)
        restyle = style_state != self._style_state
        if restyle:
            self._style_state = style_state
            self._barrel.set_enabled_state(True)
            self._refresh_card_style(active=True)
            self._refresh_chip_style(active=True, ink_color=accent)
            self._barrel.set_warning(warning)
            if live:
                self._set_vol_style("red" if warning else "text",
                                    weight=800 if warning else 700)
                self._set_sub_style("red" if warning else "subtext0",
                                    weight=600 if warning else 400)
            else:
                self._set_vol_style("subtext0", weight=700)
                self._set_sub_style("overlay0", italic=True)

        if live:
            frac = 0.0
            if nominal_cap and nominal_cap > 0:
                frac = max(0.0, min(1.0, fill_uL / nominal_cap))
            self._barrel.set_fractions(
                {"ink": frac, "empty": max(0.0, 1.0 - frac)}, accent)
            # Normalise the "-0.0" that signed arithmetic produces at the empty
            # datum; keep a genuine (past-empty) negative visible.
            shown = 0.0 if round(fill_uL, 1) == 0.0 else fill_uL
            self._vol_label.setText(f"{shown:.1f} µL")
            cap_txt = f"{nominal_cap:g} µL"
            short = ink_name if (ink_name and len(ink_name) <= 14) \
                else (ink_name[:13] + "…" if ink_name else None)
            sub = f"{cap_txt}  ·  {short}" if short else f"{cap_txt} full"
            self._sub_label.setText(sub)
            self._sub_label.setToolTip(
                f"{shown:.2f} µL in a {nominal_cap:g} µL syringe"
                + (f"  (raw {raw_mm:.3f} mm)" if raw_mm is not None else ""))
        else:
            # Configured but the plunger isn't calibrated — we can't express a
            # true volume, but the raw position still updates live.
            self._barrel.set_fractions({"empty": 1.0}, accent)
            self._vol_label.setText(
                f"{raw_mm:.2f} mm" if raw_mm is not None else "—")
            self._sub_label.setText("calibrate plunger for µL")
            self._sub_label.setToolTip(
                "Run the Hardware Setup → Pump plunger calibration "
                "(Set Dispensed / Set Aspirated) to read fill in µL.")

    # ── Styling helpers ────────────────────────────────────────────

    def _set_vol_style(self, color_key: str, *, weight: int = 700) -> None:
        self._vol_label.setStyleSheet(
            f"color: {COLORS[color_key]};"
            f"font-size: {sf(13)}pt;"
            f"font-weight: {weight};"
            f"letter-spacing: 0.3px;"
        )

    def _set_sub_style(self, color_key: str, *, italic: bool = False,
                       weight: int = 400) -> None:
        self._sub_label.setStyleSheet(
            f"color: {COLORS[color_key]};"
            f"font-size: {sf(8)}pt;"
            + (f"font-weight: {weight};" if weight != 400 else "")
            + ("font-style: italic;" if italic else "")
        )

    def _refresh_card_style(self, active: bool) -> None:
        if active:
            bg = COLORS['surface0']
            border = COLORS['surface1']
        else:
            bg = COLORS['mantle']
            border = COLORS['surface0']
        self._card.setStyleSheet(
            f"QFrame#pumpColCard {{"
            f"  background-color: {bg};"
            f"  border: 1px solid {border};"
            f"  border-radius: {sp(8)};"
            f"}}"
        )

    def _refresh_chip_style(self, *, active: bool,
                            ink_color: str | None) -> None:
        if active and ink_color:
            bg = ink_color
            fg = COLORS['crust']
            border = ink_color
        elif active:
            bg = COLORS['mauve']
            fg = COLORS['crust']
            border = COLORS['mauve']
        else:
            bg = COLORS['surface0']
            fg = COLORS['overlay0']
            border = COLORS['surface1']
        self._chip.setStyleSheet(
            f"QLabel {{"
            f"  color: {fg};"
            f"  background-color: {bg};"
            f"  border: 1px solid {border};"
            f"  border-radius: {sp(5)};"
            f"  font-weight: 800;"
            f"  font-size: {sf(9)}pt;"
            f"  letter-spacing: 0.5px;"
            f"}}"
        )

    def _refresh(self) -> None:
        if self._loadout is None or self._loadout.syringe is None:
            self._refresh_card_style(active=False)
            self._refresh_chip_style(active=False, ink_color=None)
            self._barrel.set_fractions({"empty": 1.0})
            self._barrel.set_warning(False)
            self._barrel.set_enabled_state(False)
            self._vol_label.setText("—")
            self._vol_label.setStyleSheet(
                f"color: {COLORS['overlay0']};"
                f"font-size: {sf(13)}pt;"
                f"font-weight: 700;"
            )
            self._sub_label.setText("not configured")
            self._sub_label.setStyleSheet(
                f"color: {COLORS['overlay0']};"
                f"font-size: {sf(8)}pt;"
                f"font-style: italic;"
            )
            self._sub_label.setToolTip("")
            return

        syringe = self._loadout.syringe
        fc = self._loadout.fluid_column

        ink_color = DEFAULT_INK_COLOR
        if fc.ink_spec and getattr(fc.ink_spec, "display_color", None):
            ink_color = fc.ink_spec.display_color

        self._barrel.set_enabled_state(True)
        self._barrel.set_fractions(fc.volume_fractions(syringe), ink_color)
        self._refresh_card_style(active=True)
        self._refresh_chip_style(active=True, ink_color=ink_color)

        self._vol_label.setText(f"{fc.ink_volume_uL:.1f} µL")
        self._vol_label.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(13)}pt;"
            f"font-weight: 700;"
            f"letter-spacing: 0.3px;"
        )

        ink_name = (
            fc.ink_spec.name if fc.ink_spec is not None else None
        )
        if ink_name:
            short = ink_name if len(ink_name) <= 18 else ink_name[:17] + "…"
            sub = f"{syringe.volume_uL:g} µL  ·  {short}"
        else:
            sub = f"{syringe.volume_uL:g} µL capacity"
        self._sub_label.setText(sub)
        self._sub_label.setToolTip(
            f"{fc.ink_volume_uL:.2f} µL of "
            f"{ink_name or 'no ink'} in a "
            f"{syringe.volume_uL:g} µL syringe")

        # Warnings
        warning = False
        if 0 < fc.ink_volume_uL <= LOW_INK_THRESHOLD_UL:
            warning = True
        elif fc.ink_volume_uL <= 0:
            warning = True
        if fc.has_buffer and fc.buffer_volume_uL <= LOW_BUFFER_THRESHOLD_UL:
            warning = True
        self._barrel.set_warning(warning)
        if warning:
            self._vol_label.setStyleSheet(
                f"color: {COLORS['red']};"
                f"font-size: {sf(13)}pt;"
                f"font-weight: 800;"
            )
            self._sub_label.setStyleSheet(
                f"color: {COLORS['red']};"
                f"font-size: {sf(8)}pt;"
                f"font-weight: 600;"
            )
        else:
            self._sub_label.setStyleSheet(
                f"color: {COLORS['subtext0']};"
                f"font-size: {sf(8)}pt;"
            )


# ── Rack ───────────────────────────────────────────────────────────


class PumpRack(QWidget):
    """Row of three pump columns, equal stretch (P1 / P2 / P3)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        # Total height ~= barrel (~38) + chip (~18) + readouts (~36)
        # + padding (~22) ≈ 114 px. Cap with a sensible minimum so
        # nothing gets clipped.
        self.setMinimumHeight(s(124))
        self.setMaximumHeight(s(180))

        layout = QHBoxLayout(self)
        layout.setContentsMargins(s(2), s(2), s(2), s(2))
        layout.setSpacing(s(8))

        self.columns: dict[str, PumpColumn] = {}
        for pid in ("P1", "P2", "P3"):
            col = PumpColumn(pump_id=pid)
            self.columns[pid] = col
            layout.addWidget(col, stretch=1)

    def update_from_workspace(self, pumps: dict[str, PumpLoadout]) -> None:
        for pid, col in self.columns.items():
            loadout = pumps.get(pid)
            if loadout is not None:
                col.set_pump_loadout(loadout)
            else:
                col.set_unconfigured()

    def update_live_fills(self, fills: dict[str, dict | None]) -> None:
        """v7.5.x: drive each column from a live plunger-fill snapshot.

        ``fills`` maps pump id → kwargs for :meth:`PumpColumn.set_live_fill`
        (``fill_uL`` / ``capacity_uL`` / ``raw_mm`` / ``syringe`` /
        ``ink_spec`` / ``calibrated``), or ``None`` for an unconfigured pump.
        """
        for pid, col in self.columns.items():
            spec = fills.get(pid)
            if spec is None:
                col.set_unconfigured()
            else:
                col.set_live_fill(**spec)
