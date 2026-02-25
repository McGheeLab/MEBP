"""
Settings Page - GUI for editing application settings.

Sections:
- Connection: Simulation mode, serial port selection
- Safety Limits: Software endstops for all axes (Task 3)
- Polling: Position poll interval, watchdog interval
- Xbox: Mapping file, deadzone, averaging interval
- Logging: Verbose mode, log level

Session 4 — Task 3 (Safety Limits UI) + Task 5 (Settings UI)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QCheckBox, QScrollArea, QFrame, QFileDialog, QSizePolicy,
    QSlider,
)
from PySide6.QtCore import Qt

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.Settings import Settings
from SupportClasses.SerialUtils import list_serial_ports

import logging
logger = logging.getLogger(__name__)


class SettingsPage(QWidget):
    """Application settings page with all configurable parameters."""

    def __init__(self, controller: StageController, settings: Settings, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        self._setup_ui()
        self._load_from_controller()

    def _setup_ui(self):
        # Scrollable content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        self._build_connection_section(layout)
        self._build_safety_section(layout)
        self._build_polling_section(layout)
        self._build_xbox_section(layout)
        self._build_logging_section(layout)

        # Action buttons
        btn_row = QHBoxLayout()
        btn_apply = QPushButton("✓ Apply Settings")
        btn_apply.setObjectName("connectBtn")
        btn_apply.clicked.connect(self._apply_settings)
        btn_row.addWidget(btn_apply)

        btn_defaults = QPushButton("↺ Reset to Defaults")
        btn_defaults.setObjectName("disconnectBtn")
        btn_defaults.clicked.connect(self._reset_defaults)
        btn_row.addWidget(btn_defaults)
        layout.addLayout(btn_row)

        layout.addStretch()
        scroll.setWidget(content)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    # ── Connection Section ─────────────────────────────────────────

    def _build_connection_section(self, parent_layout):
        group = QGroupBox("Connection")
        grid = QGridLayout(group)
        row = 0

        # Simulation mode checkboxes
        grid.addWidget(QLabel("Simulation Mode:"), row, 0)
        self.chk_sim_xy = QCheckBox("Simulate XY Stage")
        self.chk_sim_zp = QCheckBox("Simulate ZP Stage")
        sim_row = QHBoxLayout()
        sim_row.addWidget(self.chk_sim_xy)
        sim_row.addWidget(self.chk_sim_zp)
        grid.addLayout(sim_row, row, 1)
        row += 1

        # Info label
        restart_label = QLabel("⚠ Simulation mode changes require restart")
        restart_label.setStyleSheet("color: #f9e2af; font-size: 10pt;")
        grid.addWidget(restart_label, row, 0, 1, 2)
        row += 1

        # Serial port display
        grid.addWidget(QLabel("Available Ports:"), row, 0)
        self.port_list_label = QLabel("—")
        self.port_list_label.setWordWrap(True)
        self.port_list_label.setStyleSheet("color: #6c7086;")
        grid.addWidget(self.port_list_label, row, 1)
        row += 1

        btn_refresh = QPushButton("🔄 Refresh Ports")
        btn_refresh.clicked.connect(self._refresh_ports)
        grid.addWidget(btn_refresh, row, 1)
        row += 1

        parent_layout.addWidget(group)

    # ── Safety Limits Section ──────────────────────────────────────

    def _build_safety_section(self, parent_layout):
        group = QGroupBox("Safety Limits (Software Endstops)")
        layout = QVBoxLayout(group)

        # Enable/disable
        self.chk_safety_enabled = QCheckBox("Enable Safety Limits")
        self.chk_safety_enabled.setChecked(True)
        layout.addWidget(self.chk_safety_enabled)

        grid = QGridLayout()
        row = 0

        # XY limits
        grid.addWidget(QLabel("XY Min X:"), row, 0)
        self.spin_xy_min_x = QDoubleSpinBox()
        self.spin_xy_min_x.setRange(-999999, 999999)
        self.spin_xy_min_x.setDecimals(0)
        self.spin_xy_min_x.setSuffix(" steps")
        grid.addWidget(self.spin_xy_min_x, row, 1)
        grid.addWidget(QLabel("XY Max X:"), row, 2)
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
        grid.addWidget(QLabel("XY Max Y:"), row, 2)
        self.spin_xy_max_y = QDoubleSpinBox()
        self.spin_xy_max_y.setRange(-999999, 999999)
        self.spin_xy_max_y.setDecimals(0)
        self.spin_xy_max_y.setSuffix(" steps")
        grid.addWidget(self.spin_xy_max_y, row, 3)
        row += 1

        # Quick-set XY buttons
        btn_xy_row = QHBoxLayout()
        btn_set_xy_min = QPushButton("Set XY Min from Current")
        btn_set_xy_min.clicked.connect(lambda: self._set_xy_from_current(as_max=False))
        btn_xy_row.addWidget(btn_set_xy_min)
        btn_set_xy_max = QPushButton("Set XY Max from Current")
        btn_set_xy_max.clicked.connect(lambda: self._set_xy_from_current(as_max=True))
        btn_xy_row.addWidget(btn_set_xy_max)
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

        # Quick-set Z buttons
        btn_z_row = QHBoxLayout()
        btn_set_z_min = QPushButton("Set Z Min from Current")
        btn_set_z_min.clicked.connect(lambda: self._set_z_from_current(as_max=False))
        btn_z_row.addWidget(btn_set_z_min)
        btn_set_z_max = QPushButton("Set Z Max from Current")
        btn_set_z_max.clicked.connect(lambda: self._set_z_from_current(as_max=True))
        btn_z_row.addWidget(btn_set_z_max)
        grid.addLayout(btn_z_row, row, 0, 1, 4)
        row += 1

        # Pump limits (P1 only shown for simplicity; P2/P3 share same range)
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
        grid.addWidget(QLabel("Max Pump Feedrate:"), row, 2)
        self.spin_max_p_feed = QDoubleSpinBox()
        self.spin_max_p_feed.setRange(1, 1000)
        self.spin_max_p_feed.setDecimals(0)
        self.spin_max_p_feed.setSuffix(" mm/min")
        grid.addWidget(self.spin_max_p_feed, row, 3)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(group)

    # ── Polling Section ────────────────────────────────────────────

    def _build_polling_section(self, parent_layout):
        group = QGroupBox("Polling")
        grid = QGridLayout(group)
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

        parent_layout.addWidget(group)

    # ── Xbox Section ───────────────────────────────────────────────

    def _build_xbox_section(self, parent_layout):
        group = QGroupBox("Xbox Controller")
        grid = QGridLayout(group)
        row = 0

        grid.addWidget(QLabel("Mapping File:"), row, 0)
        self.xbox_mapping_label = QLabel("current_button_mapping.json")
        self.xbox_mapping_label.setStyleSheet("color: #a6adc8;")
        grid.addWidget(self.xbox_mapping_label, row, 1)
        btn_browse = QPushButton("Browse...")
        btn_browse.clicked.connect(self._browse_mapping)
        grid.addWidget(btn_browse, row, 2)
        row += 1

        parent_layout.addWidget(group)

    # ── Logging Section ────────────────────────────────────────────

    def _build_logging_section(self, parent_layout):
        group = QGroupBox("Logging")
        grid = QGridLayout(group)
        row = 0

        self.chk_verbose = QCheckBox("Verbose (Debug) Logging")
        grid.addWidget(self.chk_verbose, row, 0, 1, 2)
        row += 1

        parent_layout.addWidget(group)

    # ── Load/Apply ─────────────────────────────────────────────────

    def _load_from_controller(self):
        """Load current values from controller and settings into UI."""
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
        mapping = self.settings.get("xbox.mapping_file", "current_button_mapping.json")
        self.xbox_mapping_label.setText(mapping)

        # Logging
        self.chk_verbose.setChecked(self.settings.get("logging.verbose", False))

        self._refresh_ports()

    def _apply_settings(self):
        """Apply UI values to the controller and save to settings."""
        sl = self.controller.safety_limits

        # Safety limits → controller
        sl.enabled = self.chk_safety_enabled.isChecked()
        sl.xy_min_x = self.spin_xy_min_x.value()
        sl.xy_max_x = self.spin_xy_max_x.value()
        sl.xy_min_y = self.spin_xy_min_y.value()
        sl.xy_max_y = self.spin_xy_max_y.value()
        sl.z_min = self.spin_z_min.value()
        sl.z_max = self.spin_z_max.value()
        # Apply pump limits uniformly to all pumps
        sl.p1_min = sl.p2_min = sl.p3_min = self.spin_p_min.value()
        sl.p1_max = sl.p2_max = sl.p3_max = self.spin_p_max.value()
        sl.max_z_feedrate = self.spin_max_z_feed.value()
        sl.max_pump_feedrate = self.spin_max_p_feed.value()

        # Persist safety limits to settings
        self.settings.set_section("safety_limits", sl.to_dict())

        # Polling → apply immediately
        poll_ms = self.spin_poll_interval.value()
        self.controller._pos_poller.poll_interval = poll_ms / 1000.0
        self.settings.set("polling.position_interval_ms", poll_ms)
        self.settings.set("polling.watchdog_interval_s", self.spin_watchdog.value())

        # Simulation flags → save only (need restart)
        self.settings.set("simulation.simulate_xy", self.chk_sim_xy.isChecked())
        self.settings.set("simulation.simulate_zp", self.chk_sim_zp.isChecked())

        # Xbox
        self.settings.set("xbox.mapping_file", self.xbox_mapping_label.text())

        # Logging → apply immediately
        verbose = self.chk_verbose.isChecked()
        self.settings.set("logging.verbose", verbose)
        log_level = logging.DEBUG if verbose else logging.INFO
        logging.getLogger().setLevel(log_level)

        self.settings.save()
        logger.info("Settings applied and saved")

    def _reset_defaults(self):
        """Reset all settings to defaults."""
        from SupportClasses.Settings import DEFAULTS
        from SupportClasses.SafetyLimits import SafetyLimits as SL

        self.controller.safety_limits = SL()
        self._load_from_controller()
        logger.info("Settings reset to defaults")

    # ── Helpers ────────────────────────────────────────────────────

    def _refresh_ports(self):
        """Refresh serial port list."""
        ports = list_serial_ports()
        if ports:
            text = ", ".join(f"{p['device']} ({p['description']})" for p in ports)
        else:
            text = "No serial ports found"
        self.port_list_label.setText(text)

    def _browse_mapping(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Select Xbox Mapping File", "",
            "JSON (*.json);;All (*)",
        )
        if filepath:
            self.xbox_mapping_label.setText(filepath)

    def _set_xy_from_current(self, as_max=True):
        """Set XY limits from the current stage position."""
        pos = self.controller.get_xy_position(cached=True)
        if pos[0] is not None:
            # Convert to zero-ref coordinates
            zero_x = pos[0] - self.controller.zero_position["x"]
            zero_y = pos[1] - self.controller.zero_position["y"]
            if as_max:
                self.spin_xy_max_x.setValue(zero_x)
                self.spin_xy_max_y.setValue(zero_y)
            else:
                self.spin_xy_min_x.setValue(zero_x)
                self.spin_xy_min_y.setValue(zero_y)
            logger.info(f"XY {'max' if as_max else 'min'} set to ({zero_x:.0f}, {zero_y:.0f})")

    def _set_z_from_current(self, as_max=True):
        """Set Z limit from the current position."""
        pos = self.controller.get_zp_position(cached=True)
        if pos[0] is not None:
            zero_z = pos[0] - self.controller.zero_position["Z"]
            if as_max:
                self.spin_z_max.setValue(zero_z)
            else:
                self.spin_z_min.setValue(zero_z)
            logger.info(f"Z {'max' if as_max else 'min'} set to {zero_z:.2f} mm")

    def update_data(self):
        """Called by the main window's update timer (optional refresh)."""
        pass
