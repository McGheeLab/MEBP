"""
StandardJogContextPanel — reusable left context panel for jog-aware pages.

Mirrors the jog UI already proven out on the Calibration page (a
:class:`HardwareControlPanel` with the Connect group hidden and safety
limits engaged) and adds two sections for quick reference:

  - **Absolute Go To** — X / Y / Z spinboxes + Safe Travel toggle + Go
  - **Hardware Info** — read-only summary of the active hardware setup
    (needle gauge / OD / length, syringe sizes per configured pump,
    well-plate format, calibrated-well count, safe Z)

Designed to be a drop-in replacement for any page that previously
returned a bespoke jog context. Pages call:

    panel = StandardJogContextPanel(controller, settings=settings)
    panel.set_hardware_config(hw_config)         # initially + on changes
    panel.set_calibration_data(plate, wells, safe_z)
    panel.on_status_update()                     # from MainWindow tick

The inner :class:`HardwareControlPanel` owns the jog buttons + speeds +
live position bars, so this wrapper stays thin: it adds the two extra
sections, forwards status ticks, and exposes the hardware-info refresh.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from gui.pages.hardware.control_panel import HardwareControlPanel
from gui.scaling import s, sf, sp
from gui.styles import COLORS
from gui.widgets.components import Card
from SupportClasses.StageController import z_raw_to_display, z_display_to_raw

if TYPE_CHECKING:
    from SupportClasses.StageController import StageController

logger = logging.getLogger(__name__)


class StandardJogContextPanel(QWidget):
    """Reusable left context panel: HW jog controls + quick actions +
    absolute Go To + hardware info."""

    def __init__(
        self,
        controller: "StageController | None" = None,
        parent: QWidget | None = None,
        *,
        settings=None,
        show_connect: bool = False,
        bypass_safety: bool = False,
    ):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._hardware_config = None
        self._plate = None
        self._well_positions = None
        self._safe_z: float | None = None
        # v7.4.4: full set of captured Z reference heights from the
        # Needle Offset Calibration tab. Populated via
        # ``set_z_references()``. Missing entries render as "unset".
        self._z_refs: dict[str, float | None] = {
            "replace_z": None, "max_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }

        self._build_ui(show_connect=show_connect, bypass_safety=bypass_safety)

        if controller is not None:
            self._hw_panel.set_controller(controller)
        if settings is not None:
            self._hw_panel.set_settings(settings)

    # ── Public API ─────────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._controller = controller
        self._hw_panel.set_controller(controller)

    def set_settings(self, settings) -> None:
        self._settings = settings
        self._hw_panel.set_settings(settings)

    def set_hardware_config(self, config) -> None:
        self._hardware_config = config
        self._refresh_hardware_info()

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        # Keep the fast-move-z slot in z_refs in sync with the legacy
        # safe_z so the Hardware Info card never disagrees with itself.
        self._z_refs["fast_move_z"] = safe_z
        self._refresh_hardware_info()

    def set_z_references(self, refs: dict) -> None:
        """v7.4.4: Receive the full set of captured Z reference heights
        (``replace_z`` / ``max_z`` / ``fast_move_z`` / ``plate_top_z``
        / ``plate_bottom_z``). Each value is mm zero-referenced or
        None when unset."""
        for key in self._z_refs:
            if key in refs:
                self._z_refs[key] = refs[key]
        # Mirror fast_move_z back onto the legacy attr so the legacy
        # Absolute-Go-To safe-retract uses the latest value.
        if refs.get("fast_move_z") is not None:
            self._safe_z = refs["fast_move_z"]
        self._refresh_hardware_info()

    def on_status_update(self) -> None:
        """Forward MainWindow's ~300 ms tick into the inner panel."""
        self._hw_panel.on_status_update()

    def refresh_safety_limits(self) -> None:
        self._hw_panel.refresh_safety_limits()

    # ── UI ─────────────────────────────────────────────────────────

    def _build_ui(self, *, show_connect: bool, bypass_safety: bool) -> None:
        # v7.4.3: no internal QScrollArea — MainWindow already wraps the
        # context widget in one. A nested scroll would compress the jog
        # buttons. The whole panel is one tall vertical stack of
        # collapsible sections; the outer scroll handles overflow.
        layout = QVBoxLayout(self)
        layout.setContentsMargins(s(10), s(10), s(10), s(10))
        layout.setSpacing(s(10))

        # 1) Existing HW jog panel (connect hidden, safety engaged,
        #    embedded so it doesn't add its own scroll).
        self._hw_panel = HardwareControlPanel(
            show_connect=show_connect,
            bypass_safety=bypass_safety,
            embedded=True,
        )
        layout.addWidget(self._hw_panel)

        # 2) Absolute Go To (collapsible)
        layout.addWidget(self._build_goto_card())

        # 3) Hardware Info (collapsible, read-only reference)
        layout.addWidget(self._build_hardware_info_card())

        layout.addStretch(1)

    # ── Absolute Go To ─────────────────────────────────────────────

    def _build_goto_card(self) -> Card:
        card = Card("Absolute Go To", collapsible=True)

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(s(6))
        grid.setVerticalSpacing(s(4))

        grid.addWidget(QLabel("X"), 0, 0)
        self._goto_x = QDoubleSpinBox()
        self._goto_x.setRange(-999_999, 999_999)
        self._goto_x.setDecimals(1)
        self._goto_x.setSuffix(" µm")
        grid.addWidget(self._goto_x, 0, 1)

        grid.addWidget(QLabel("Y"), 1, 0)
        self._goto_y = QDoubleSpinBox()
        self._goto_y.setRange(-999_999, 999_999)
        self._goto_y.setDecimals(1)
        self._goto_y.setSuffix(" µm")
        grid.addWidget(self._goto_y, 1, 1)

        grid.addWidget(QLabel("Z"), 2, 0)
        self._goto_z = QDoubleSpinBox()
        self._goto_z.setRange(-100.0, 100.0)
        self._goto_z.setDecimals(3)
        self._goto_z.setSuffix(" mm")
        self._goto_z.setToolTip(
            "Target Z height (zero-referenced). Positive = up, matching the "
            "live Z readout and side view.")
        grid.addWidget(self._goto_z, 2, 1)

        card.add_layout(grid)

        self._chk_safe_travel = QCheckBox("Safe Travel")
        self._chk_safe_travel.setChecked(True)
        self._chk_safe_travel.setToolTip(
            "Raise Z to safe height before XY move, then lower.")
        card.add_widget(self._chk_safe_travel)

        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_go = QPushButton("Go")
        btn_go.setObjectName("accentBtn")
        btn_go.setMinimumHeight(s(28))
        btn_go.clicked.connect(self._absolute_goto)
        btn_row.addStretch(1)
        btn_row.addWidget(btn_go)
        card.add_layout(btn_row)

        return card

    def _absolute_goto(self) -> None:
        if self._controller is None:
            return
        target_x_um_zr = float(self._goto_x.value())
        target_y_um_zr = float(self._goto_y.value())
        # v7.5.x: the Z box is height-frame (up = +); convert to the raw
        # zero-ref frame the move methods expect so a positive target moves
        # UP, not down.
        target_z_mm_zr = z_display_to_raw(float(self._goto_z.value()))

        zero = self._controller.zero_position
        abs_x_um = target_x_um_zr + zero["x"]
        abs_y_um = target_y_um_zr + zero["y"]

        if self._chk_safe_travel.isChecked():
            safe_z = self._safe_z
            if safe_z is None:
                resp = QMessageBox.question(
                    self, "No Safe Z",
                    "No safe Z is configured. Travel without retract?",
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No,
                )
                if resp != QMessageBox.StandardButton.Yes:
                    return
                if self._controller.is_xy_connected:
                    # v7.5.x bugfix: move_xy_absolute(from_zero_ref=True) takes
                    # mm; the goto X/Y boxes are µm → convert (was 1000× over).
                    self._controller.move_xy_absolute(
                        target_x_um_zr / 1000.0, target_y_um_zr / 1000.0,
                        from_zero_ref=True)
                if self._controller.is_zp_connected:
                    self._controller.move_z_absolute(
                        target_z_mm_zr, from_zero_ref=True)
                return
            self._controller.safe_travel_to(
                abs_x_um, abs_y_um,
                safe_z_mm=safe_z,
                target_z_mm=target_z_mm_zr,
            )
        else:
            if self._controller.is_xy_connected:
                # v7.5.x bugfix: µm → mm for move_xy_absolute (see above).
                self._controller.move_xy_absolute(
                    target_x_um_zr / 1000.0, target_y_um_zr / 1000.0,
                    from_zero_ref=True)
            if self._controller.is_zp_connected:
                self._controller.move_z_absolute(
                    target_z_mm_zr, from_zero_ref=True)

    # ── Hardware Info (read-only quick-reference) ──────────────────

    def _build_hardware_info_card(self) -> Card:
        card = Card("Hardware Info", collapsible=True)
        self._lbl_info_needle = self._info_row(card, "Needle", "—")
        self._lbl_info_p1 = self._info_row(card, "P1 syringe", "—")
        self._lbl_info_p2 = self._info_row(card, "P2 syringe", "—")
        self._lbl_info_p3 = self._info_row(card, "P3 syringe", "—")
        self._lbl_info_plate = self._info_row(card, "Plate", "—")
        self._lbl_info_calibration = self._info_row(card, "Calibration", "—")
        # v7.4.4: five Z reference heights, top → bottom.
        self._lbl_info_replace_z = self._info_row(card, "Replace Z", "—")
        self._lbl_info_max_z = self._info_row(card, "Max Z", "—")
        self._lbl_info_safe_z = self._info_row(card, "Fast Move Z", "—")
        self._lbl_info_plate_top_z = self._info_row(card, "Plate Top Z", "—")
        self._lbl_info_plate_bottom_z = self._info_row(
            card, "Plate Bottom Z", "—")
        return card

    def _info_row(self, card: Card, label_text: str, value_text: str) -> QLabel:
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(6))
        lbl = QLabel(label_text)
        lbl.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(9)}pt;"
            f"font-weight: 600;"
        )
        lbl.setMinimumWidth(s(86))
        row.addWidget(lbl)
        value_lbl = QLabel(value_text)
        value_lbl.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(9)}pt;"
            f"font-family: Consolas, Menlo, monospace;"
        )
        value_lbl.setWordWrap(True)
        row.addWidget(value_lbl, stretch=1)
        card.add_layout(row)
        return value_lbl

    def _refresh_hardware_info(self) -> None:
        # Needle
        hw = self._hardware_config
        needle_text = "Not configured"
        if hw is not None and getattr(hw, "needle", None) is not None:
            n = hw.needle
            parts = [f"{n.gauge}G"] if getattr(n, "gauge", None) else []
            if getattr(n, "outer_diameter_um", None):
                parts.append(f"OD {n.outer_diameter_um:.0f} µm")
            if getattr(n, "inner_diameter_um", None):
                parts.append(f"ID {n.inner_diameter_um:.0f} µm")
            if getattr(n, "length_mm", None):
                parts.append(f"L {n.length_mm:.1f} mm")
            if getattr(n, "num_channels", 1) > 1:
                parts.append(f"{n.num_channels}-channel")
            needle_text = " · ".join(parts) if parts else "Configured"
        self._lbl_info_needle.setText(needle_text)

        # Pumps (P1 / P2 / P3)
        for pid, lbl in (
            ("P1", self._lbl_info_p1),
            ("P2", self._lbl_info_p2),
            ("P3", self._lbl_info_p3),
        ):
            text = "—"
            if hw is not None and pid in hw.pumps:
                pump_cfg = hw.pumps[pid]
                if pump_cfg is not None and pump_cfg.is_configured:
                    parts = []
                    if pump_cfg.syringe is not None:
                        s_obj = pump_cfg.syringe
                        parts.append(f"{s_obj.volume_uL:g} µL")
                        if getattr(s_obj, "stroke_length_mm", None):
                            parts.append(f"stroke {s_obj.stroke_length_mm:.1f} mm")
                    inks = list(getattr(pump_cfg, "inks", []) or [])
                    if inks:
                        names = [
                            getattr(i, "name", "?") for i in inks if i is not None
                        ]
                        if names:
                            parts.append(", ".join(names))
                    text = " · ".join(parts) if parts else "Configured"
                else:
                    text = "Not configured"
            lbl.setText(text)

        # Plate
        plate_text = "—"
        if self._plate is not None:
            try:
                wells = self._plate.rows * self._plate.cols
                plate_text = (
                    f"{wells}-well · {self._plate.well_diameter:.2f} mm Ø · "
                    f"depth {self._plate.well_depth_mm:.1f} mm")
            except AttributeError:
                plate_text = "Configured"
        self._lbl_info_plate.setText(plate_text)

        # Calibrated wells
        cal_text = "—"
        if self._well_positions:
            cal_text = f"{len(self._well_positions)} wells"
        self._lbl_info_calibration.setText(cal_text)

        # v7.4.4: render all five Z reference heights.
        for key, lbl in (
            ("replace_z",      self._lbl_info_replace_z),
            ("max_z",          self._lbl_info_max_z),
            ("fast_move_z",    self._lbl_info_safe_z),
            ("plate_top_z",    self._lbl_info_plate_top_z),
            ("plate_bottom_z", self._lbl_info_plate_bottom_z),
        ):
            val = self._z_refs.get(key)
            # Fast-move-z falls back to the legacy attr for back-compat.
            if val is None and key == "fast_move_z":
                val = self._safe_z
            # v7.5.x: references are stored zero-ref (raw); show as height
            # (up = +) so they agree with the rest of the jog-page Z readouts.
            lbl.setText(
                f"{z_raw_to_display(val):.2f} mm" if val is not None else "unset")
