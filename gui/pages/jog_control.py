"""
Jog Control Page — Manual movement with compact layout.

Main content: XY pad, Z/pump buttons, position readouts, quick actions
Context panel: step sizes, speed multipliers

v7.1.1: XY step sizes and positions displayed in microns (µm).
v7.2:   Pump step sizes and positions displayed in µL (microliters).
         Conversion to/from mm handled via HardwareConfig.
         Pump jog buttons disabled until hardware setup is complete.
"""

from __future__ import annotations

import logging
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QSlider, QFrame, QSizePolicy,
    QScrollArea, QGroupBox,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QKeyEvent

from SupportClasses.StageController import StageController
from SupportClasses.HardwareConfig import HardwareConfig
from gui.styles import COLORS

logger = logging.getLogger(__name__)

# Step sizes in microns (µm) for the XY stage
XY_STEPS_UM = [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]

# Z step sizes remain in mm
Z_STEPS = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]

# Pump step sizes in µL (microliters) — the core change in v7.2
P_STEPS_UL = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0]

# Pump flow rate presets in µL/s for speed slider
P_RATES_UL_S = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0]


class JogControlPage(QWidget):
    """Manual jog controls with fixed-step moves. Pumps operate in µL."""

    _page_title_text = "Jog Control"

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._context_widget = None

        # Hardware config — set by MainWindow when hardware setup completes
        self._hw_config: HardwareConfig | None = None

        # Microsteps per micron — set by MainWindow, default 10 (ProScan III typical)
        self._microsteps_per_micron: float = 10.0

        self._setup_ui()
        self._setup_shortcuts()

    def get_page_title(self) -> str:
        return "Jog Control"

    def set_microsteps_per_micron(self, value: float):
        """Called by MainWindow when the conversion factor changes."""
        self._microsteps_per_micron = max(0.001, value)

    def set_hardware_config(self, config: HardwareConfig):
        """
        Called by MainWindow when hardware setup changes.
        Enables/disables pump jog buttons based on which pumps are configured.
        """
        self._hw_config = config
        self._update_pump_buttons_state()
        self._update_pump_readouts()
        logger.info(f"JogControl: hardware config updated, {config.active_pump_count} pumps active")

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        main = QVBoxLayout(self)
        main.setContentsMargins(12, 12, 12, 12)
        main.setSpacing(12)

        # Top row: Position readouts
        readout_frame = QFrame()
        readout_frame.setObjectName("readoutFrame")
        readout_lay = QGridLayout(readout_frame)
        readout_lay.setContentsMargins(8, 8, 8, 8)

        # XY readout (µm)
        readout_lay.addWidget(self._make_label("X:", bold=True), 0, 0)
        self.x_readout = self._make_label("0.0 µm")
        readout_lay.addWidget(self.x_readout, 0, 1)

        readout_lay.addWidget(self._make_label("Y:", bold=True), 0, 2)
        self.y_readout = self._make_label("0.0 µm")
        readout_lay.addWidget(self.y_readout, 0, 3)

        # Z readout (mm)
        readout_lay.addWidget(self._make_label("Z:", bold=True), 1, 0)
        self.z_readout = self._make_label("0.000 mm")
        readout_lay.addWidget(self.z_readout, 1, 1)

        # Pump readouts (µL) — one for each pump
        self.pump_readouts: dict[str, QLabel] = {}
        for col, pid in enumerate(["P1", "P2", "P3"]):
            readout_lay.addWidget(self._make_label(f"{pid}:", bold=True), 1, 2 + col * 2)
            lbl = self._make_label("— µL")
            self.pump_readouts[pid] = lbl
            readout_lay.addWidget(lbl, 1, 3 + col * 2)

        main.addWidget(readout_frame)

        # ── Jog Grid ──────────────────────────────────────────────
        jog_grid = QGridLayout()
        jog_grid.setSpacing(8)

        # XY Pad (left side)
        xy_group = QGroupBox("XY Stage (µm)")
        xy_lay = QGridLayout(xy_group)
        xy_lay.setSpacing(4)

        self.xy_step_combo = QComboBox()
        for s in XY_STEPS_UM:
            self.xy_step_combo.addItem(f"{s:.1f} µm" if s < 1000 else f"{s/1000:.1f} mm", s)
        self.xy_step_combo.setCurrentIndex(3)  # Default 50 µm
        xy_lay.addWidget(QLabel("Step:"), 0, 0)
        xy_lay.addWidget(self.xy_step_combo, 0, 1, 1, 2)

        # Direction buttons
        self.btn_y_plus = QPushButton("▲ Y+")
        self.btn_y_plus.clicked.connect(lambda: self._jog_xy(0, 1))
        xy_lay.addWidget(self.btn_y_plus, 1, 1)

        self.btn_x_minus = QPushButton("◄ X-")
        self.btn_x_minus.clicked.connect(lambda: self._jog_xy(-1, 0))
        xy_lay.addWidget(self.btn_x_minus, 2, 0)

        self.btn_xy_home = QPushButton("⌂")
        self.btn_xy_home.setToolTip("Move to zero reference")
        self.btn_xy_home.clicked.connect(self._go_home_xy)
        xy_lay.addWidget(self.btn_xy_home, 2, 1)

        self.btn_x_plus = QPushButton("X+ ►")
        self.btn_x_plus.clicked.connect(lambda: self._jog_xy(1, 0))
        xy_lay.addWidget(self.btn_x_plus, 2, 2)

        self.btn_y_minus = QPushButton("▼ Y-")
        self.btn_y_minus.clicked.connect(lambda: self._jog_xy(0, -1))
        xy_lay.addWidget(self.btn_y_minus, 3, 1)

        jog_grid.addWidget(xy_group, 0, 0)

        # Z Pad (middle)
        z_group = QGroupBox("Z Needle (mm)")
        z_lay = QGridLayout(z_group)
        z_lay.setSpacing(4)

        self.z_step_combo = QComboBox()
        for s in Z_STEPS:
            self.z_step_combo.addItem(f"{s:.2f} mm", s)
        self.z_step_combo.setCurrentIndex(2)  # Default 0.1 mm
        z_lay.addWidget(QLabel("Step:"), 0, 0)
        z_lay.addWidget(self.z_step_combo, 0, 1)

        self.btn_z_up = QPushButton("▲ Z Up")
        self.btn_z_up.clicked.connect(lambda: self._jog_z(1))
        z_lay.addWidget(self.btn_z_up, 1, 0, 1, 2)

        self.btn_z_down = QPushButton("▼ Z Down")
        self.btn_z_down.clicked.connect(lambda: self._jog_z(-1))
        z_lay.addWidget(self.btn_z_down, 2, 0, 1, 2)

        jog_grid.addWidget(z_group, 0, 1)

        # Pump Pads (right) — one per pump, all in µL
        pump_container = QGroupBox("Pumps (µL)")
        pump_lay = QGridLayout(pump_container)
        pump_lay.setSpacing(4)

        # Shared step size for all pumps
        pump_lay.addWidget(QLabel("Step:"), 0, 0)
        self.p_step_combo = QComboBox()
        for s in P_STEPS_UL:
            if s >= 1.0:
                self.p_step_combo.addItem(f"{s:.1f} µL", s)
            else:
                self.p_step_combo.addItem(f"{s:.2f} µL", s)
        self.p_step_combo.setCurrentIndex(3)  # Default 0.25 µL
        pump_lay.addWidget(self.p_step_combo, 0, 1, 1, 2)

        # Pump rate selector
        pump_lay.addWidget(QLabel("Rate:"), 0, 3)
        self.p_rate_combo = QComboBox()
        for r in P_RATES_UL_S:
            self.p_rate_combo.addItem(f"{r:.2f} µL/s", r)
        self.p_rate_combo.setCurrentIndex(3)  # Default 0.25 µL/s
        pump_lay.addWidget(self.p_rate_combo, 0, 4)

        # Individual pump buttons
        self._pump_jog_buttons: dict[str, tuple[QPushButton, QPushButton]] = {}
        for col, pid in enumerate(["P1", "P2", "P3"]):
            lbl = QLabel(pid)
            lbl.setFont(QFont("Segoe UI", 10, QFont.Bold))
            lbl.setAlignment(Qt.AlignCenter)
            pump_lay.addWidget(lbl, 1, col * 2, 1, 2)

            btn_up = QPushButton(f"▲ Dispense")
            btn_up.setToolTip(f"{pid}: Push plunger (dispense)")
            btn_up.clicked.connect(partial(self._jog_pump, pid, 1))
            pump_lay.addWidget(btn_up, 2, col * 2, 1, 2)

            btn_down = QPushButton(f"▼ Aspirate")
            btn_down.setToolTip(f"{pid}: Pull plunger (aspirate)")
            btn_down.clicked.connect(partial(self._jog_pump, pid, -1))
            pump_lay.addWidget(btn_down, 3, col * 2, 1, 2)

            self._pump_jog_buttons[pid] = (btn_up, btn_down)

        jog_grid.addWidget(pump_container, 0, 2)

        main.addLayout(jog_grid)
        main.addStretch()

        # Initial pump state
        self._update_pump_buttons_state()

    def _make_label(self, text: str, bold: bool = False) -> QLabel:
        lbl = QLabel(text)
        if bold:
            lbl.setFont(QFont("Segoe UI", 10, QFont.Bold))
        return lbl

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL
    # ════════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        """Context panel with step sizes and speed controls."""
        if self._context_widget:
            return self._context_widget

        ctx = QWidget()
        lay = QVBoxLayout(ctx)
        lay.setContentsMargins(8, 8, 8, 8)

        # Pump capacity info
        self._ctx_pump_info = QGroupBox("Pump Syringe Info")
        self._ctx_pump_lay = QVBoxLayout(self._ctx_pump_info)
        self._ctx_pump_labels: dict[str, QLabel] = {}
        for pid in ["P1", "P2", "P3"]:
            lbl = QLabel(f"{pid}: No syringe")
            lbl.setStyleSheet(f"color: {COLORS.get('subtext0', '#6c7086')}; font-size: 11px;")
            self._ctx_pump_lay.addWidget(lbl)
            self._ctx_pump_labels[pid] = lbl
        lay.addWidget(self._ctx_pump_info)

        # Quick actions
        actions_group = QGroupBox("Quick Actions")
        actions_lay = QVBoxLayout(actions_group)

        btn_zero_xy = QPushButton("Set XY Zero Here")
        btn_zero_xy.clicked.connect(self._set_zero_xy)
        actions_lay.addWidget(btn_zero_xy)

        btn_zero_z = QPushButton("Set Z Zero Here")
        btn_zero_z.clicked.connect(self._set_zero_z)
        actions_lay.addWidget(btn_zero_z)

        for pid in ["P1", "P2", "P3"]:
            btn = QPushButton(f"Set {pid} Zero Here")
            btn.clicked.connect(partial(self._set_zero_pump, pid))
            actions_lay.addWidget(btn)

        lay.addWidget(actions_group)
        lay.addStretch()

        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  JOG COMMANDS
    # ════════════════════════════════════════════════════════════════

    def _jog_xy(self, dx_sign: int, dy_sign: int):
        """Jog XY by selected step size (µm → microsteps)."""
        step_um = self.xy_step_combo.currentData() or 50.0
        dx_um = dx_sign * step_um
        dy_um = dy_sign * step_um
        # Convert µm → microsteps
        dx_steps = dx_um * self._microsteps_per_micron
        dy_steps = dy_um * self._microsteps_per_micron
        self.controller.move_xy_relative(dx_steps, dy_steps)

    def _jog_z(self, direction: int):
        """Jog Z by selected step (mm)."""
        step = self.z_step_combo.currentData() or 0.1
        self.controller.move_z_relative(direction * step)

    def _jog_pump(self, pump: str, direction: int):
        """
        Jog a pump by selected step in µL.

        The step size is in µL. We convert to mm using the syringe spec
        from the hardware config, then send to the controller.
        Positive direction = dispense (plunger pushes), negative = aspirate.
        """
        if not self._hw_config:
            logger.warning(f"Cannot jog {pump}: no hardware config")
            return

        pump_cfg = self._hw_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            logger.warning(f"Cannot jog {pump}: not configured")
            return

        step_uL = self.p_step_combo.currentData() or 0.25
        volume_uL = direction * step_uL

        # Convert µL to mm
        try:
            distance_mm = pump_cfg.uL_to_mm(volume_uL)
        except ValueError as e:
            logger.error(f"Pump jog conversion error: {e}")
            return

        # Convert rate from µL/s to mm/min
        rate_uL_s = self.p_rate_combo.currentData() or 0.25
        try:
            feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(rate_uL_s)
        except ValueError:
            feedrate_mm_min = None

        logger.debug(
            f"Jog {pump}: {volume_uL:+.3f} µL → {distance_mm:+.5f} mm, "
            f"rate: {rate_uL_s:.2f} µL/s → {feedrate_mm_min:.1f} mm/min"
        )
        self.controller.move_pump_relative(pump, distance_mm, feedrate_mm_min)

    # ════════════════════════════════════════════════════════════════
    #  PUMP STATE MANAGEMENT
    # ════════════════════════════════════════════════════════════════

    def _update_pump_buttons_state(self):
        """Enable/disable pump jog buttons based on hardware config."""
        for pid, (btn_up, btn_down) in self._pump_jog_buttons.items():
            if self._hw_config:
                pump_cfg = self._hw_config.pumps.get(pid)
                enabled = pump_cfg is not None and pump_cfg.is_configured
            else:
                enabled = False
            btn_up.setEnabled(enabled)
            btn_down.setEnabled(enabled)

            if not enabled:
                btn_up.setToolTip(f"{pid}: No syringe configured — set up in Hardware Setup")
                btn_down.setToolTip(f"{pid}: No syringe configured — set up in Hardware Setup")

    def _update_pump_readouts(self):
        """Update context panel with syringe info."""
        if not hasattr(self, '_ctx_pump_labels'):
            return

        for pid in ["P1", "P2", "P3"]:
            lbl = self._ctx_pump_labels.get(pid)
            if not lbl:
                continue

            if self._hw_config:
                pump_cfg = self._hw_config.pumps.get(pid)
                if pump_cfg and pump_cfg.is_configured:
                    syr = pump_cfg.syringe
                    lbl.setText(
                        f"{pid}: {syr.volume_uL} µL syringe | "
                        f"{syr.uL_per_mm:.2f} µL/mm"
                    )
                    continue
            lbl.setText(f"{pid}: No syringe")

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATE (called by MainWindow timer)
    # ════════════════════════════════════════════════════════════════

    def on_status_update(self, xy_pos, zp_pos):
        """
        Update position readouts.
        xy_pos: (x_steps, y_steps) or (None, None)
        zp_pos: (z_mm, p1_mm, p2_mm, p3_mm) or (None, None, None, None)
        """
        # XY in µm
        if xy_pos and xy_pos[0] is not None:
            x_um = xy_pos[0] / self._microsteps_per_micron
            y_um = xy_pos[1] / self._microsteps_per_micron
            self.x_readout.setText(f"{x_um:.1f} µm")
            self.y_readout.setText(f"{y_um:.1f} µm")

        # Z in mm
        if zp_pos and zp_pos[0] is not None:
            self.z_readout.setText(f"{zp_pos[0]:.3f} mm")

        # Pumps in µL
        if zp_pos:
            for idx, pid in enumerate(["P1", "P2", "P3"], start=1):
                pos_mm = zp_pos[idx] if idx < len(zp_pos) else None
                lbl = self.pump_readouts.get(pid)
                if lbl is None:
                    continue

                if pos_mm is not None and self._hw_config:
                    pump_cfg = self._hw_config.pumps.get(pid)
                    if pump_cfg and pump_cfg.is_configured:
                        try:
                            pos_uL = pump_cfg.mm_to_uL(pos_mm)
                            lbl.setText(f"{pos_uL:.2f} µL")
                            continue
                        except ValueError:
                            pass
                    lbl.setText(f"{pos_mm:.3f} mm")
                elif pos_mm is not None:
                    lbl.setText(f"{pos_mm:.3f} mm")
                else:
                    lbl.setText("— µL")

    # ════════════════════════════════════════════════════════════════
    #  QUICK ACTIONS
    # ════════════════════════════════════════════════════════════════

    def _set_zero_xy(self):
        self.controller.set_zero_xy()

    def _set_zero_z(self):
        self.controller.set_zero_z()

    def _set_zero_pump(self, pump: str):
        self.controller.set_zero_pump(pump)

    def _go_home_xy(self):
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)

    # ════════════════════════════════════════════════════════════════
    #  KEYBOARD SHORTCUTS
    # ════════════════════════════════════════════════════════════════

    def _setup_shortcuts(self):
        pass  # Handled in keyPressEvent

    def keyPressEvent(self, event: QKeyEvent):
        """Arrow keys for XY, PgUp/PgDown for Z."""
        key = event.key()
        if key == Qt.Key.Key_Left:
            self._jog_xy(-1, 0)
        elif key == Qt.Key.Key_Right:
            self._jog_xy(1, 0)
        elif key == Qt.Key.Key_Up:
            self._jog_xy(0, 1)
        elif key == Qt.Key.Key_Down:
            self._jog_xy(0, -1)
        elif key == Qt.Key.Key_PageUp:
            self._jog_z(1)
        elif key == Qt.Key.Key_PageDown:
            self._jog_z(-1)
        else:
            super().keyPressEvent(event)
