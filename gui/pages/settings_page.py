"""
Settings Page — Application configuration.

PyDracula layout:
    Main content  = Card-based sections (Connection, Safety, Polling, Xbox, Logging)
    Context panel = Quick safety toggle, simulation indicators, port refresh, Apply/Reset

All original functionality preserved:
- Simulation mode toggles (require restart)
- Serial port listing + refresh
- Full safety limits editor (XY/Z/Pump min/max + feedrate limits)
- Quick-set from current position buttons
- Polling interval + watchdog
- Xbox mapping file browser
- Verbose logging toggle
- Apply + Reset to Defaults
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QCheckBox, QScrollArea, QFrame, QFileDialog,
)
from PySide6.QtCore import Qt

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings
from SupportClasses.SerialUtils import list_serial_ports
from gui.styles import COLORS

logger = logging.getLogger(__name__)


class SettingsPage(QWidget):
    """
    Application settings page.

    Interface contract:
        get_page_title()     → str
        get_context_widget() → QWidget  (quick actions + status)
        on_status_update()   → called by MainWindow timer
    """

    def __init__(self, controller: StageController,
                 settings: Settings, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        self._context_widget = None
        self._setup_ui()
        self._load_from_controller()

    # ════════════════════════════════════════════════════════════════
    #  PAGE INTERFACE
    # ════════════════════════════════════════════════════════════════

    def get_page_title(self) -> str:
        return "Settings"

    def on_status_update(self):
        """Called by MainWindow timer. No periodic refresh needed."""
        pass

    # legacy alias
    def update_data(self):
        pass

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL — Quick Actions + Status
    # ════════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        lay = QVBoxLayout(ctx)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)

        # ── Quick Safety ─────────────────────────────────────────
        lbl = QLabel("Quick Safety")
        lbl.setObjectName("contextSectionLabel")
        lay.addWidget(lbl)

        self.ctx_safety_chk = QCheckBox("Safety Limits Enabled")
        self.ctx_safety_chk.setChecked(True)
        self.ctx_safety_chk.toggled.connect(self._ctx_safety_toggled)
        lay.addWidget(self.ctx_safety_chk)

        self.ctx_safety_status = QLabel("Enabled")
        self.ctx_safety_status.setObjectName("contextLabel")
        self.ctx_safety_status.setStyleSheet(f"color: {COLORS['green']};")
        lay.addWidget(self.ctx_safety_status)

        # ── Simulation Mode ──────────────────────────────────────
        sim_lbl = QLabel("Simulation Mode")
        sim_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(sim_lbl)

        self.ctx_sim_xy_lbl = QLabel("XY: —")
        self.ctx_sim_xy_lbl.setObjectName("contextLabel")
        lay.addWidget(self.ctx_sim_xy_lbl)

        self.ctx_sim_zp_lbl = QLabel("ZP: —")
        self.ctx_sim_zp_lbl.setObjectName("contextLabel")
        lay.addWidget(self.ctx_sim_zp_lbl)

        note = QLabel("⚠ Changes require restart")
        note.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 9pt;")
        note.setWordWrap(True)
        lay.addWidget(note)

        # ── Serial Ports ─────────────────────────────────────────
        port_lbl = QLabel("Serial Ports")
        port_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(port_lbl)

        self.ctx_port_label = QLabel("—")
        self.ctx_port_label.setObjectName("contextLabel")
        self.ctx_port_label.setWordWrap(True)
        lay.addWidget(self.ctx_port_label)

        btn_refresh = QPushButton("🔄 Refresh Ports")
        btn_refresh.setMaximumHeight(26)
        btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(btn_refresh)

        # ── Actions ──────────────────────────────────────────────
        act_lbl = QLabel("Actions")
        act_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(act_lbl)

        btn_apply = QPushButton("✓ Apply Settings")
        btn_apply.setObjectName("successBtn")
        btn_apply.setMaximumHeight(28)
        btn_apply.clicked.connect(self._apply_settings)
        lay.addWidget(btn_apply)

        btn_reset = QPushButton("↺ Reset to Defaults")
        btn_reset.setObjectName("dangerBtn")
        btn_reset.setMaximumHeight(28)
        btn_reset.clicked.connect(self._reset_defaults)
        lay.addWidget(btn_reset)

        self.ctx_status_label = QLabel("")
        self.ctx_status_label.setObjectName("dimLabel")
        self.ctx_status_label.setWordWrap(True)
        lay.addWidget(self.ctx_status_label)

        lay.addStretch()
        self._context_widget = ctx

        # Sync context labels from current main content state
        self._update_context_sim_labels()
        self.ctx_safety_chk.setChecked(self.chk_safety_enabled.isChecked())
        self.ctx_port_label.setText(self.port_list_label.text())

        return ctx

    def _ctx_safety_toggled(self, checked):
        """Quick toggle from context panel — sync to main content."""
        self.chk_safety_enabled.setChecked(checked)
        if checked:
            self.ctx_safety_status.setText("Enabled")
            self.ctx_safety_status.setStyleSheet(
                f"color: {COLORS['green']};")
        else:
            self.ctx_safety_status.setText("Disabled")
            self.ctx_safety_status.setStyleSheet(
                f"color: {COLORS['red']};")

    def _update_context_sim_labels(self):
        """Refresh simulation status labels in context panel."""
        if self._context_widget is None:
            return
        xy_sim = self.chk_sim_xy.isChecked()
        zp_sim = self.chk_sim_zp.isChecked()
        self.ctx_sim_xy_lbl.setText(
            f"XY: {'Simulated' if xy_sim else 'Hardware'}")
        self.ctx_sim_xy_lbl.setStyleSheet(
            f"color: {COLORS['yellow'] if xy_sim else COLORS['green']};")
        self.ctx_sim_zp_lbl.setText(
            f"ZP: {'Simulated' if zp_sim else 'Hardware'}")
        self.ctx_sim_zp_lbl.setStyleSheet(
            f"color: {COLORS['yellow'] if zp_sim else COLORS['green']};")

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT — Card-based settings sections
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 8, 12, 8)

        self._build_connection_card(layout)
        self._build_safety_card(layout)
        self._build_polling_card(layout)
        self._build_xbox_card(layout)
        self._build_logging_card(layout)

        # Apply / Reset buttons (also in main content for convenience)
        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        btn_apply = QPushButton("✓ Apply Settings")
        btn_apply.setObjectName("successBtn")
        btn_apply.clicked.connect(self._apply_settings)
        btn_row.addWidget(btn_apply)
        btn_defaults = QPushButton("↺ Reset to Defaults")
        btn_defaults.setObjectName("dangerBtn")
        btn_defaults.clicked.connect(self._reset_defaults)
        btn_row.addWidget(btn_defaults)
        layout.addLayout(btn_row)

        layout.addStretch()
        scroll.setWidget(content)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    # ── Connection Card ───────────────────────────────────────────

    def _build_connection_card(self, parent_layout):
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("Connection")
        title.setObjectName("sectionLabel")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        # Simulation mode
        grid.addWidget(QLabel("Simulation Mode:"), row, 0)
        sim_row = QHBoxLayout()
        self.chk_sim_xy = QCheckBox("Simulate XY")
        self.chk_sim_xy.toggled.connect(
            lambda: self._update_context_sim_labels())
        self.chk_sim_zp = QCheckBox("Simulate ZP")
        self.chk_sim_zp.toggled.connect(
            lambda: self._update_context_sim_labels())
        sim_row.addWidget(self.chk_sim_xy)
        sim_row.addWidget(self.chk_sim_zp)
        grid.addLayout(sim_row, row, 1)
        row += 1

        restart_label = QLabel("⚠ Simulation changes require restart")
        restart_label.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 10pt;")
        grid.addWidget(restart_label, row, 0, 1, 2)
        row += 1

        # Serial ports
        grid.addWidget(QLabel("Available Ports:"), row, 0)
        self.port_list_label = QLabel("—")
        self.port_list_label.setWordWrap(True)
        self.port_list_label.setStyleSheet(
            f"color: {COLORS['overlay0']};")
        grid.addWidget(self.port_list_label, row, 1)
        row += 1

        btn_refresh = QPushButton("🔄 Refresh Ports")
        btn_refresh.clicked.connect(self._refresh_ports)
        grid.addWidget(btn_refresh, row, 1)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(card)

    # ── Safety Limits Card ────────────────────────────────────────

    def _build_safety_card(self, parent_layout):
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("Safety Limits (Software Endstops)")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        self.chk_safety_enabled = QCheckBox("Enable Safety Limits")
        self.chk_safety_enabled.setChecked(True)
        self.chk_safety_enabled.toggled.connect(self._safety_main_toggled)
        layout.addWidget(self.chk_safety_enabled)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        # XY limits
        grid.addWidget(QLabel("XY Min X:"), row, 0)
        self.spin_xy_min_x = QDoubleSpinBox()
        self.spin_xy_min_x.setRange(-999999, 999999)
        self.spin_xy_min_x.setDecimals(0)
        self.spin_xy_min_x.setSuffix(" steps")
        grid.addWidget(self.spin_xy_min_x, row, 1)
        grid.addWidget(QLabel("Max X:"), row, 2)
        self.spin_xy_max_x = QDoubleSpinBox()
        self.spin_xy_max_x.setRange(-999999, 999999)
        self.spin_xy_max_x.setDecimals(0)
        self.spin_xy_max_x.setSuffix(" steps")
        grid.addWidget(self.spin_xy_max_x, row, 3)
        row += 1

        grid.addWidget(QLabel("XY Min Y:"), row, 0)
        self.spin_xy_min_y = QDoubleSpinBox()
        self.spin_xy_min_y.setRange(-999999, 999999)
        self.spin_xy_min_y.setDecimals(0)
        self.spin_xy_min_y.setSuffix(" steps")
        grid.addWidget(self.spin_xy_min_y, row, 1)
        grid.addWidget(QLabel("Max Y:"), row, 2)
        self.spin_xy_max_y = QDoubleSpinBox()
        self.spin_xy_max_y.setRange(-999999, 999999)
        self.spin_xy_max_y.setDecimals(0)
        self.spin_xy_max_y.setSuffix(" steps")
        grid.addWidget(self.spin_xy_max_y, row, 3)
        row += 1

        # Quick-set XY
        btn_xy_row = QHBoxLayout()
        btn_xy_min = QPushButton("Set XY Min from Current")
        btn_xy_min.setMaximumHeight(26)
        btn_xy_min.clicked.connect(
            lambda: self._set_xy_from_current(as_max=False))
        btn_xy_row.addWidget(btn_xy_min)
        btn_xy_max = QPushButton("Set XY Max from Current")
        btn_xy_max.setMaximumHeight(26)
        btn_xy_max.clicked.connect(
            lambda: self._set_xy_from_current(as_max=True))
        btn_xy_row.addWidget(btn_xy_max)
        grid.addLayout(btn_xy_row, row, 0, 1, 4)
        row += 1

        # Z limits
        grid.addWidget(QLabel("Z Min:"), row, 0)
        self.spin_z_min = QDoubleSpinBox()
        self.spin_z_min.setRange(-100, 100)
        self.spin_z_min.setDecimals(2)
        self.spin_z_min.setSuffix(" mm")
        grid.addWidget(self.spin_z_min, row, 1)
        grid.addWidget(QLabel("Z Max:"), row, 2)
        self.spin_z_max = QDoubleSpinBox()
        self.spin_z_max.setRange(-100, 200)
        self.spin_z_max.setDecimals(2)
        self.spin_z_max.setSuffix(" mm")
        grid.addWidget(self.spin_z_max, row, 3)
        row += 1

        # Quick-set Z
        btn_z_row = QHBoxLayout()
        btn_z_min = QPushButton("Set Z Min from Current")
        btn_z_min.setMaximumHeight(26)
        btn_z_min.clicked.connect(
            lambda: self._set_z_from_current(as_max=False))
        btn_z_row.addWidget(btn_z_min)
        btn_z_max = QPushButton("Set Z Max from Current")
        btn_z_max.setMaximumHeight(26)
        btn_z_max.clicked.connect(
            lambda: self._set_z_from_current(as_max=True))
        btn_z_row.addWidget(btn_z_max)
        grid.addLayout(btn_z_row, row, 0, 1, 4)
        row += 1

        # Pump limits
        grid.addWidget(QLabel("Pump Min:"), row, 0)
        self.spin_p_min = QDoubleSpinBox()
        self.spin_p_min.setRange(-200, 200)
        self.spin_p_min.setDecimals(1)
        self.spin_p_min.setSuffix(" mm")
        grid.addWidget(self.spin_p_min, row, 1)
        grid.addWidget(QLabel("Pump Max:"), row, 2)
        self.spin_p_max = QDoubleSpinBox()
        self.spin_p_max.setRange(-200, 200)
        self.spin_p_max.setDecimals(1)
        self.spin_p_max.setSuffix(" mm")
        grid.addWidget(self.spin_p_max, row, 3)
        row += 1

        # Speed limits
        grid.addWidget(QLabel("Max Z Feedrate:"), row, 0)
        self.spin_max_z_feed = QDoubleSpinBox()
        self.spin_max_z_feed.setRange(1, 2000)
        self.spin_max_z_feed.setDecimals(0)
        self.spin_max_z_feed.setSuffix(" mm/min")
        grid.addWidget(self.spin_max_z_feed, row, 1)
        grid.addWidget(QLabel("Max Pump Feed:"), row, 2)
        self.spin_max_p_feed = QDoubleSpinBox()
        self.spin_max_p_feed.setRange(1, 1000)
        self.spin_max_p_feed.setDecimals(0)
        self.spin_max_p_feed.setSuffix(" mm/min")
        grid.addWidget(self.spin_max_p_feed, row, 3)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(card)

    # ── Polling Card ──────────────────────────────────────────────

    def _build_polling_card(self, parent_layout):
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("Polling")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        grid.addWidget(QLabel("Position Poll Interval:"), row, 0)
        self.spin_poll_interval = QSpinBox()
        self.spin_poll_interval.setRange(100, 2000)
        self.spin_poll_interval.setSuffix(" ms")
        self.spin_poll_interval.setSingleStep(50)
        grid.addWidget(self.spin_poll_interval, row, 1)
        row += 1

        grid.addWidget(QLabel("Watchdog Interval:"), row, 0)
        self.spin_watchdog = QDoubleSpinBox()
        self.spin_watchdog.setRange(1.0, 30.0)
        self.spin_watchdog.setDecimals(1)
        self.spin_watchdog.setSuffix(" s")
        grid.addWidget(self.spin_watchdog, row, 1)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(card)

    # ── Xbox Card ─────────────────────────────────────────────────

    def _build_xbox_card(self, parent_layout):
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("Xbox Controller")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        grid.addWidget(QLabel("Mapping File:"), row, 0)
        self.xbox_mapping_label = QLabel("current_button_mapping.json")
        self.xbox_mapping_label.setStyleSheet(
            f"color: {COLORS['subtext0']};")
        grid.addWidget(self.xbox_mapping_label, row, 1)
        btn_browse = QPushButton("Browse...")
        btn_browse.setMaximumHeight(26)
        btn_browse.clicked.connect(self._browse_mapping)
        grid.addWidget(btn_browse, row, 2)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(card)

    # ── Logging Card ──────────────────────────────────────────────

    def _build_logging_card(self, parent_layout):
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("Logging")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        self.chk_verbose = QCheckBox("Verbose (Debug) Logging")
        layout.addWidget(self.chk_verbose)

        parent_layout.addWidget(card)

    # ════════════════════════════════════════════════════════════════
    #  LOAD / APPLY / RESET
    # ════════════════════════════════════════════════════════════════

    def _load_from_controller(self):
        """Populate UI from controller + settings."""
        sl = self.controller.safety_limits

        # Connection
        self.chk_sim_xy.setChecked(self.controller.simulate_xy)
        self.chk_sim_zp.setChecked(self.controller.simulate_zp)

        # Safety limits
        self.chk_safety_enabled.setChecked(sl.enabled)
        self.spin_xy_min_x.setValue(sl.xy_min_x)
        self.spin_xy_max_x.setValue(sl.xy_max_x)
        self.spin_xy_min_y.setValue(sl.xy_min_y)
        self.spin_xy_max_y.setValue(sl.xy_max_y)
        self.spin_z_min.setValue(sl.z_min)
        self.spin_z_max.setValue(sl.z_max)
        self.spin_p_min.setValue(sl.p1_min)
        self.spin_p_max.setValue(sl.p1_max)
        self.spin_max_z_feed.setValue(sl.max_z_feedrate)
        self.spin_max_p_feed.setValue(sl.max_pump_feedrate)

        # Polling
        poll_ms = self.settings.get("polling.position_interval_ms", 300)
        self.spin_poll_interval.setValue(poll_ms)
        watchdog_s = self.settings.get("polling.watchdog_interval_s", 3.0)
        self.spin_watchdog.setValue(watchdog_s)

        # Xbox
        mapping = self.settings.get(
            "xbox.mapping_file", "current_button_mapping.json")
        self.xbox_mapping_label.setText(mapping)

        # Logging
        self.chk_verbose.setChecked(
            self.settings.get("logging.verbose", False))

        self._refresh_ports()
        self._update_context_sim_labels()

    def _apply_settings(self):
        """Apply UI values to controller and persist."""
        sl = self.controller.safety_limits

        # Safety limits → controller
        sl.enabled = self.chk_safety_enabled.isChecked()
        sl.xy_min_x = self.spin_xy_min_x.value()
        sl.xy_max_x = self.spin_xy_max_x.value()
        sl.xy_min_y = self.spin_xy_min_y.value()
        sl.xy_max_y = self.spin_xy_max_y.value()
        sl.z_min = self.spin_z_min.value()
        sl.z_max = self.spin_z_max.value()
        sl.p1_min = sl.p2_min = sl.p3_min = self.spin_p_min.value()
        sl.p1_max = sl.p2_max = sl.p3_max = self.spin_p_max.value()
        sl.max_z_feedrate = self.spin_max_z_feed.value()
        sl.max_pump_feedrate = self.spin_max_p_feed.value()

        self.settings.set_section("safety_limits", sl.to_dict())

        # Polling → apply immediately
        poll_ms = self.spin_poll_interval.value()
        self.controller._pos_poller.poll_interval = poll_ms / 1000.0
        self.settings.set("polling.position_interval_ms", poll_ms)
        self.settings.set(
            "polling.watchdog_interval_s", self.spin_watchdog.value())

        # Simulation flags → save only (need restart)
        self.settings.set(
            "simulation.simulate_xy", self.chk_sim_xy.isChecked())
        self.settings.set(
            "simulation.simulate_zp", self.chk_sim_zp.isChecked())

        # Xbox
        self.settings.set(
            "xbox.mapping_file", self.xbox_mapping_label.text())

        # Logging → apply immediately
        verbose = self.chk_verbose.isChecked()
        self.settings.set("logging.verbose", verbose)
        log_level = logging.DEBUG if verbose else logging.INFO
        logging.getLogger().setLevel(log_level)

        self.settings.save()
        logger.info("Settings applied and saved")

        # Update context panel status
        if self._context_widget:
            self.ctx_status_label.setText("Settings applied ✓")
            self.ctx_status_label.setStyleSheet(
                f"color: {COLORS['green']};")

    def _reset_defaults(self):
        """Reset all settings to defaults."""
        self.controller.safety_limits = SafetyLimits()
        self._load_from_controller()
        logger.info("Settings reset to defaults")

        if self._context_widget:
            self.ctx_status_label.setText("Reset to defaults ↺")
            self.ctx_status_label.setStyleSheet(
                f"color: {COLORS['yellow']};")

    def _safety_main_toggled(self, checked):
        """Sync main content safety checkbox to context panel."""
        if self._context_widget:
            self.ctx_safety_chk.setChecked(checked)

    # ════════════════════════════════════════════════════════════════
    #  HELPERS
    # ════════════════════════════════════════════════════════════════

    def _refresh_ports(self):
        """Refresh serial port list."""
        ports = list_serial_ports()
        if ports:
            text = ", ".join(
                f"{p['device']} ({p['description']})" for p in ports)
        else:
            text = "No serial ports found"
        self.port_list_label.setText(text)
        # Also update context panel
        if self._context_widget:
            self.ctx_port_label.setText(text)

    def _browse_mapping(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Select Xbox Mapping File", "",
            "JSON (*.json);;All (*)")
        if filepath:
            self.xbox_mapping_label.setText(filepath)

    def _set_xy_from_current(self, as_max=True):
        """Set XY limits from the current stage position."""
        pos = self.controller.get_xy_position(cached=True)
        if pos[0] is not None:
            zero_x = pos[0] - self.controller.zero_position["x"]
            zero_y = pos[1] - self.controller.zero_position["y"]
            if as_max:
                self.spin_xy_max_x.setValue(zero_x)
                self.spin_xy_max_y.setValue(zero_y)
            else:
                self.spin_xy_min_x.setValue(zero_x)
                self.spin_xy_min_y.setValue(zero_y)
            logger.info(
                f"XY {'max' if as_max else 'min'} set to "
                f"({zero_x:.0f}, {zero_y:.0f})")

    def _set_z_from_current(self, as_max=True):
        """Set Z limit from the current position."""
        pos = self.controller.get_zp_position(cached=True)
        if pos[0] is not None:
            zero_z = pos[0] - self.controller.zero_position["Z"]
            if as_max:
                self.spin_z_max.setValue(zero_z)
            else:
                self.spin_z_min.setValue(zero_z)
            logger.info(
                f"Z {'max' if as_max else 'min'} set to {zero_z:.2f} mm")
