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
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings
from SupportClasses.SerialUtils import list_serial_ports
from SupportClasses.ControllerProtocol import (
    ControllerProtocol, discover_controller_files, DEFAULT_CONTROLLERS_DIR,
)
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.unit_helpers import (
    stage_to_um, um_to_stage, convert_safety_xy_text,
    DEFAULT_XY_POSITION_SCALE,
)
from gui.widgets.jog_button_array import JogButtonArray
from gui.scaling import s, scaled_font_size

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
        self._xy_position_scale = DEFAULT_XY_POSITION_SCALE
        self._setup_ui()
        self._load_from_controller()

    # ════════════════════════════════════════════════════════════════
    #  PAGE INTERFACE
    # ════════════════════════════════════════════════════════════════

    def get_page_title(self) -> str:
        return "Settings"

    def on_status_update(self):
        """Called by MainWindow timer. Update position readout in context panel."""
        if not hasattr(self, '_ctx_pos_labels'):
            return
        try:
            xy = self.controller.get_xy_position(cached=True)
            zp = self.controller.get_zp_position(cached=True)
            zero = self.controller.zero_position

            if xy[0] is not None:
                rel_x = stage_to_um(xy[0] - zero.get("x", 0),
                                    self._xy_position_scale)
                rel_y = stage_to_um(xy[1] - zero.get("y", 0),
                                    self._xy_position_scale)
                self._ctx_pos_labels["X"].setText(f"{rel_x:,.1f}")
                self._ctx_pos_labels["Y"].setText(f"{rel_y:,.1f}")

            if zp[0] is not None:
                rel_z = zp[0] - zero.get("Z", 0)
                self._ctx_pos_labels["Z"].setText(f"{rel_z:.3f}")
                for i, pid in enumerate(["P1", "P2", "P3"], start=1):
                    if i < len(zp) and zp[i] is not None:
                        rel_p = zp[i] - zero.get(pid, 0)
                        self._ctx_pos_labels[pid].setText(f"{rel_p:.3f}")
        except Exception:
            pass

    def set_xy_position_scale(self, value: float):
        """Update the XY position scale factor and refresh related UI."""
        self._xy_position_scale = value

    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Receive hardware config for dual mm/µL display."""
        self._hardware_config = config
        if hasattr(self, 'spin_um_factor'):
            self.spin_um_factor.blockSignals(True)
            self.spin_um_factor.setValue(value)
            self.spin_um_factor.blockSignals(False)
        if hasattr(self, 'lbl_safety_um'):
            self._update_safety_um_label()
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
        btn_refresh.setMaximumHeight(s(26))
        btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(btn_refresh)

        # ── Manual Calibration (v7.3.2) ─────────────────────────
        cal_lbl = QLabel("Manual Zero Calibration")
        cal_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(cal_lbl)

        cal_desc = QLabel("Jog to desired zero position, then set zero per axis.")
        cal_desc.setWordWrap(True)
        cal_desc.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")
        lay.addWidget(cal_desc)

        # Embedded jog controls (compact + pumps)
        self._ctx_jog = JogButtonArray(compact=True, show_pumps=True)
        self._ctx_jog.jog_xy_requested.connect(self._ctx_jog_xy)
        self._ctx_jog.jog_z_requested.connect(self._ctx_jog_z)
        self._ctx_jog.jog_pump_requested.connect(self._ctx_jog_pump)
        self._ctx_jog.home_requested.connect(self._ctx_jog_home)
        lay.addWidget(self._ctx_jog)

        # Position readout
        pos_frame = QFrame()
        pos_frame.setStyleSheet(
            f"background: {COLORS['mantle']}; border-radius: 4px; padding: 4px;")
        pos_grid = QGridLayout(pos_frame)
        pos_grid.setSpacing(2)
        pos_grid.setContentsMargins(4, 4, 4, 4)

        mono = QFont("Consolas, Courier New, monospace")
        mono.setPointSize(scaled_font_size(9))

        self._ctx_pos_labels: dict[str, QLabel] = {}
        for i, axis in enumerate(["X", "Y", "Z", "P1", "P2", "P3"]):
            lbl_name = QLabel(f"{axis}:")
            lbl_name.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
            pos_grid.addWidget(lbl_name, i // 3, (i % 3) * 2)
            lbl_val = QLabel("—")
            lbl_val.setFont(mono)
            lbl_val.setStyleSheet(f"color: {COLORS['text']}; font-size: 9pt;")
            lbl_val.setMinimumWidth(s(60))
            pos_grid.addWidget(lbl_val, i // 3, (i % 3) * 2 + 1)
            self._ctx_pos_labels[axis] = lbl_val
        lay.addWidget(pos_frame)

        # Zero-set buttons
        zero_row1 = QHBoxLayout()
        zero_row1.setSpacing(4)
        btn_zero_xy = QPushButton("Set XY Zero")
        btn_zero_xy.setMaximumHeight(s(24))
        btn_zero_xy.setToolTip("Set current XY position as zero reference")
        btn_zero_xy.clicked.connect(self._ctx_set_zero_xy)
        zero_row1.addWidget(btn_zero_xy)
        btn_zero_z = QPushButton("Set Z Zero")
        btn_zero_z.setMaximumHeight(s(24))
        btn_zero_z.setToolTip("Set current Z position as zero reference")
        btn_zero_z.clicked.connect(self._ctx_set_zero_z)
        zero_row1.addWidget(btn_zero_z)
        lay.addLayout(zero_row1)

        zero_row2 = QHBoxLayout()
        zero_row2.setSpacing(4)
        for pid in ["P1", "P2", "P3"]:
            btn = QPushButton(f"Set {pid} Zero")
            btn.setMaximumHeight(s(24))
            btn.setToolTip(f"Set current {pid} position as zero reference")
            btn.clicked.connect(lambda checked, p=pid: self._ctx_set_zero_pump(p))
            zero_row2.addWidget(btn)
        lay.addLayout(zero_row2)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"color: {COLORS['surface0']};")
        lay.addWidget(sep)

        # ── Actions ──────────────────────────────────────────────
        act_lbl = QLabel("Actions")
        act_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(act_lbl)

        btn_apply = QPushButton("✓ Apply Settings")
        btn_apply.setObjectName("successBtn")
        btn_apply.setMaximumHeight(s(28))
        btn_apply.clicked.connect(self._apply_settings)
        lay.addWidget(btn_apply)

        btn_reset = QPushButton("↺ Reset to Defaults")
        btn_reset.setObjectName("dangerBtn")
        btn_reset.setMaximumHeight(s(28))
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

    # ── v7.3.2: Manual calibration jog handlers ──────────────

    def _ctx_jog_xy(self, dx: float, dy: float):
        self.controller.move_xy_relative_um(dx, dy)

    def _ctx_jog_z(self, dz: float):
        self.controller.move_z_relative(dz)

    def _ctx_jog_pump(self, pump: str, dist: float):
        self.controller.move_pump_relative(pump, dist)

    def _ctx_jog_home(self):
        self.controller.move_to_zero()

    def _ctx_set_zero_xy(self):
        if hasattr(self.controller, 'calibrate_zero_xy'):
            self.controller.calibrate_zero_xy()
        else:
            # Fallback: set both X and Y from current position
            pos = self.controller.get_xy_position(cached=True)
            if pos[0] is not None:
                self.controller.zero_position["x"] = pos[0]
                self.controller.zero_position["y"] = pos[1]
        self.settings.set_section(
            "zero_position", dict(self.controller.zero_position))
        self.settings.save()
        logger.info("XY zero set from manual calibration")
        if self._context_widget:
            self.ctx_status_label.setText("XY zero set ✓")
            self.ctx_status_label.setStyleSheet(f"color: {COLORS['green']};")

    def _ctx_set_zero_z(self):
        self.controller.reset_z_zero()
        self.settings.set_section(
            "zero_position", dict(self.controller.zero_position))
        self.settings.save()
        logger.info("Z zero set from manual calibration")
        if self._context_widget:
            self.ctx_status_label.setText("Z zero set ✓")
            self.ctx_status_label.setStyleSheet(f"color: {COLORS['green']};")

    def _ctx_set_zero_pump(self, pump: str):
        self.controller.reset_pump_zero(pump)
        self.settings.set_section(
            "zero_position", dict(self.controller.zero_position))
        self.settings.save()
        logger.info(f"{pump} zero set from manual calibration")
        if self._context_widget:
            self.ctx_status_label.setText(f"{pump} zero set ✓")
            self.ctx_status_label.setStyleSheet(f"color: {COLORS['green']};")

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
        self._build_controller_card(layout)
        self._build_zp_stage_card(layout)
        self._build_safety_card(layout)
        self._build_polling_card(layout)
        self._build_xbox_card(layout)
        self._build_axis_flip_card(layout)
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

    # ── Controller Protocol Card (P8.22) ────────────────────────────

    def _build_controller_card(self, parent_layout):
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("XY Controller Protocol")
        title.setObjectName("sectionLabel")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        # Controller selector dropdown
        grid.addWidget(QLabel("Controller:"), row, 0)
        self.combo_controller = QComboBox()
        self._populate_controller_combo()
        grid.addWidget(self.combo_controller, row, 1)
        row += 1

        # Auto-detect button
        btn_row = QHBoxLayout()
        btn_auto = QPushButton("🔍 Auto-Detect")
        btn_auto.setToolTip("Scan all ports for a matching controller")
        btn_auto.clicked.connect(self._auto_detect_controller)
        btn_row.addWidget(btn_auto)

        # Test rate button
        btn_test = QPushButton("⏱ Test Rate")
        btn_test.setToolTip("Measure command/response latency")
        btn_test.clicked.connect(self._test_command_rate)
        btn_row.addWidget(btn_test)
        grid.addLayout(btn_row, row, 0, 1, 2)
        row += 1

        # Status label
        self.lbl_controller_status = QLabel("—")
        self.lbl_controller_status.setWordWrap(True)
        self.lbl_controller_status.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10pt;")
        grid.addWidget(self.lbl_controller_status, row, 0, 1, 2)
        row += 1

        # Rate test results label
        self.lbl_rate_results = QLabel("")
        self.lbl_rate_results.setWordWrap(True)
        self.lbl_rate_results.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10pt;")
        grid.addWidget(self.lbl_rate_results, row, 0, 1, 2)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(card)

    def _populate_controller_combo(self):
        """Populate the controller dropdown with JSON files + Auto option."""
        self.combo_controller.clear()
        self.combo_controller.addItem("Auto-Detect", "auto")
        self.combo_controller.addItem("Default (ProScan III)", None)

        json_files = discover_controller_files()
        for fp in json_files:
            try:
                proto = ControllerProtocol.load(fp)
                self.combo_controller.addItem(
                    f"{proto.controller_name}", str(fp))
            except Exception:
                self.combo_controller.addItem(
                    f"⚠ {fp.stem}", str(fp))

    def _auto_detect_controller(self):
        """Run auto-detection and update the UI."""
        self.lbl_controller_status.setText("Scanning ports...")
        self.lbl_controller_status.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 10pt;")

        # This would ideally run in a thread, but for now it's synchronous
        if self.controller.xy_stage and hasattr(self.controller.xy_stage, '_detected_controller'):
            detected = self.controller.xy_stage._detected_controller
            if detected:
                self.lbl_controller_status.setText(f"✓ Detected: {detected}")
                self.lbl_controller_status.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: 10pt;")
                # P8.23: Store auto-detect result
                self.settings.set("controller.auto_detect_result", detected)
                return

        self.lbl_controller_status.setText("No controller detected (XY not connected?)")
        self.lbl_controller_status.setStyleSheet(
            f"color: {COLORS['red']}; font-size: 10pt;")

    def _test_command_rate(self):
        """Run command rate test and display results."""
        self.lbl_rate_results.setText("Testing...")

        result = self.controller.test_command_rate()

        if "error" in result:
            self.lbl_rate_results.setText(f"⚠ {result['error']}")
            self.lbl_rate_results.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 10pt;")
            return

        self.lbl_rate_results.setText(
            f"Avg: {result['avg_round_trip_ms']:.1f}ms | "
            f"Max Hz: {result['max_command_hz']:.0f} | "
            f"Min: {result['min_round_trip_ms']:.1f}ms | "
            f"Max: {result['max_round_trip_ms']:.1f}ms"
        )
        self.lbl_rate_results.setStyleSheet(
            f"color: {COLORS['green']}; font-size: 10pt;")

        # P8.31: Store results
        self.settings.set("motion_controller.rate_test_results", result)
        self.settings.save()

    # ── ZP Stage Settings Card (v7.3.5) ─────────────────────────

    def _build_zp_stage_card(self, parent_layout):
        """Build the ZP stage feedrate and position save settings card."""
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("ZP Stage (Z-Needle / Pumps)")
        title.setObjectName("sectionLabel")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        desc = QLabel(
            "Feedrate settings for the Marlin Z/pump stage. "
            "Max feedrate sets M203 (hardware limit). Retract and insert "
            "feedrates are used by safe Z travel sequences.")
        desc.setWordWrap(True)
        desc.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(desc)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        # Max feedrate (M203 hardware limit)
        grid.addWidget(QLabel("Max Feedrate:"), row, 0)
        self.spin_zp_max_feedrate = QDoubleSpinBox()
        self.spin_zp_max_feedrate.setRange(10, 2000)
        self.spin_zp_max_feedrate.setDecimals(0)
        self.spin_zp_max_feedrate.setSuffix(" mm/min")
        self.spin_zp_max_feedrate.setSingleStep(10)
        self.spin_zp_max_feedrate.setToolTip(
            "Hardware speed limit for all ZP axes (sent via M203).\n"
            "Default: 200 mm/min")
        grid.addWidget(self.spin_zp_max_feedrate, row, 1)
        row += 1

        # Retract feedrate (safe Z up)
        grid.addWidget(QLabel("Safe Z Retract Feedrate:"), row, 0)
        self.spin_zp_retract_feedrate = QDoubleSpinBox()
        self.spin_zp_retract_feedrate.setRange(10, 2000)
        self.spin_zp_retract_feedrate.setDecimals(0)
        self.spin_zp_retract_feedrate.setSuffix(" mm/min")
        self.spin_zp_retract_feedrate.setSingleStep(10)
        self.spin_zp_retract_feedrate.setToolTip(
            "Feedrate when retracting needle to safe height.\n"
            "Should be at or near max for fastest retract.")
        grid.addWidget(self.spin_zp_retract_feedrate, row, 1)
        row += 1

        # Insert feedrate (safe Z down)
        grid.addWidget(QLabel("Safe Z Insert Feedrate:"), row, 0)
        self.spin_zp_insert_feedrate = QDoubleSpinBox()
        self.spin_zp_insert_feedrate.setRange(1, 2000)
        self.spin_zp_insert_feedrate.setDecimals(0)
        self.spin_zp_insert_feedrate.setSuffix(" mm/min")
        self.spin_zp_insert_feedrate.setSingleStep(10)
        self.spin_zp_insert_feedrate.setToolTip(
            "Feedrate when lowering needle into a well.\n"
            "Can be slower than retract for precision.")
        grid.addWidget(self.spin_zp_insert_feedrate, row, 1)
        row += 1

        # Default jog feedrate
        grid.addWidget(QLabel("Default Jog Feedrate:"), row, 0)
        self.spin_zp_jog_feedrate = QDoubleSpinBox()
        self.spin_zp_jog_feedrate.setRange(1, 2000)
        self.spin_zp_jog_feedrate.setDecimals(0)
        self.spin_zp_jog_feedrate.setSuffix(" mm/min")
        self.spin_zp_jog_feedrate.setSingleStep(10)
        self.spin_zp_jog_feedrate.setToolTip(
            "Feedrate for manual jog / step moves.")
        grid.addWidget(self.spin_zp_jog_feedrate, row, 1)
        row += 1

        # Position save to EEPROM
        grid.addWidget(QLabel("Auto-Save Position:"), row, 0)
        self.chk_zp_position_save = QCheckBox(
            "Save position to EEPROM at watchdog interval")
        self.chk_zp_position_save.setToolTip(
            "Periodically sends M500 to save the current Marlin position\n"
            "to EEPROM. On crash/reset, Marlin restores the last saved\n"
            "position. Interval is controlled by the Watchdog Interval setting.")
        grid.addWidget(self.chk_zp_position_save, row, 1)
        row += 1

        layout.addLayout(grid)
        parent_layout.addWidget(card)

    # ── Safety Limits Card ────────────────────────────────────────

    def _build_unit_conversion_card(self, parent_layout):
        """Build the XY unit conversion settings card."""
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("XY Unit Conversion")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        desc = QLabel(
            "The Prior ProScan XY stage speaks µm natively.\n"
            "Set to 1.0 for standard ProScan II/III. Only change if\n"
            "the stage has been configured for a different resolution.")
        desc.setWordWrap(True)
        desc.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(desc)

        row = QHBoxLayout()
        row.addWidget(QLabel("Stage units per µm:"))
        self.spin_um_factor = QDoubleSpinBox()
        self.spin_um_factor.setRange(0.001, 10000.0)
        self.spin_um_factor.setDecimals(3)
        self.spin_um_factor.setSingleStep(0.1)
        self.spin_um_factor.setValue(self._xy_position_scale)
        self.spin_um_factor.setToolTip(
            "Conversion factor: stage_units_per_micron\n"
            "ProScan II/III: 1.0 (stage speaks µm natively)\n"
            "Only change if stage resolution has been reconfigured.")
        row.addWidget(self.spin_um_factor)
        layout.addLayout(row)

        parent_layout.addWidget(card)

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

        # XY limits summary in µm
        self.lbl_safety_um = QLabel("")
        self.lbl_safety_um.setWordWrap(True)
        self.lbl_safety_um.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt; padding: 2px;")
        layout.addWidget(self.lbl_safety_um)

        grid = QGridLayout()
        grid.setSpacing(4)
        row = 0

        # XY limits
        grid.addWidget(QLabel("XY Min X:"), row, 0)
        self.spin_xy_min_x = QDoubleSpinBox()
        self.spin_xy_min_x.setRange(-999999, 999999)
        self.spin_xy_min_x.setDecimals(0)
        self.spin_xy_min_x.setSuffix(" µm")
        grid.addWidget(self.spin_xy_min_x, row, 1)
        grid.addWidget(QLabel("Max X:"), row, 2)
        self.spin_xy_max_x = QDoubleSpinBox()
        self.spin_xy_max_x.setRange(-999999, 999999)
        self.spin_xy_max_x.setDecimals(0)
        self.spin_xy_max_x.setSuffix(" µm")
        grid.addWidget(self.spin_xy_max_x, row, 3)
        row += 1

        grid.addWidget(QLabel("XY Min Y:"), row, 0)
        self.spin_xy_min_y = QDoubleSpinBox()
        self.spin_xy_min_y.setRange(-999999, 999999)
        self.spin_xy_min_y.setDecimals(0)
        self.spin_xy_min_y.setSuffix(" µm")
        grid.addWidget(self.spin_xy_min_y, row, 1)
        grid.addWidget(QLabel("Max Y:"), row, 2)
        self.spin_xy_max_y = QDoubleSpinBox()
        self.spin_xy_max_y.setRange(-999999, 999999)
        self.spin_xy_max_y.setDecimals(0)
        self.spin_xy_max_y.setSuffix(" µm")
        grid.addWidget(self.spin_xy_max_y, row, 3)
        row += 1

        # Quick-set XY
        btn_xy_row = QHBoxLayout()
        btn_xy_min = QPushButton("Set XY Min from Current")
        btn_xy_min.setMaximumHeight(s(26))
        btn_xy_min.clicked.connect(
            lambda: self._set_xy_from_current(as_max=False))
        btn_xy_row.addWidget(btn_xy_min)
        btn_xy_max = QPushButton("Set XY Max from Current")
        btn_xy_max.setMaximumHeight(s(26))
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
        btn_z_min.setMaximumHeight(s(26))
        btn_z_min.clicked.connect(
            lambda: self._set_z_from_current(as_max=False))
        btn_z_row.addWidget(btn_z_min)
        btn_z_max = QPushButton("Set Z Max from Current")
        btn_z_max.setMaximumHeight(s(26))
        btn_z_max.clicked.connect(
            lambda: self._set_z_from_current(as_max=True))
        btn_z_row.addWidget(btn_z_max)
        grid.addLayout(btn_z_row, row, 0, 1, 4)
        row += 1

        # v7.2.6: Per-pump limit spinboxes
        self._pump_min_spins = {}
        self._pump_max_spins = {}
        for pid in ['P1', 'P2', 'P3']:
            grid.addWidget(QLabel(f'{pid} Min:'), row, 0)
            spin_min = QDoubleSpinBox()
            spin_min.setRange(-200, 200)
            spin_min.setDecimals(1)
            spin_min.setSuffix(' mm')
            grid.addWidget(spin_min, row, 1)
            grid.addWidget(QLabel(f'{pid} Max:'), row, 2)
            spin_max = QDoubleSpinBox()
            spin_max.setRange(-200, 200)
            spin_max.setDecimals(1)
            spin_max.setSuffix(' mm')
            grid.addWidget(spin_max, row, 3)
            self._pump_min_spins[pid] = spin_min
            self._pump_max_spins[pid] = spin_max
            row += 1

        # v7.2.6: Per-pump quick-set and zero-reset buttons
        for pid in ['P1', 'P2', 'P3']:
            btn_row_p = QHBoxLayout()
            btn_p_min = QPushButton(f'Set {pid} Min from Current')
            btn_p_min.setMaximumHeight(s(26))
            btn_p_min.clicked.connect(
                lambda checked, p=pid: self._set_pump_from_current(p, as_max=False))
            btn_row_p.addWidget(btn_p_min)
            btn_p_max = QPushButton(f'Set {pid} Max from Current')
            btn_p_max.setMaximumHeight(s(26))
            btn_p_max.clicked.connect(
                lambda checked, p=pid: self._set_pump_from_current(p, as_max=True))
            btn_row_p.addWidget(btn_p_max)
            btn_p_zero = QPushButton(f'Reset {pid} Zero')
            btn_p_zero.setMaximumHeight(s(26))
            btn_p_zero.clicked.connect(
                lambda checked, p=pid: self._reset_pump_zero(p))
            btn_row_p.addWidget(btn_p_zero)
            grid.addLayout(btn_row_p, row, 0, 1, 4)
            row += 1
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
        self.spin_poll_interval.setRange(50, 2000)
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
        btn_browse.setMaximumHeight(s(26))
        btn_browse.clicked.connect(self._browse_mapping)
        grid.addWidget(btn_browse, row, 2)
        row += 1

        # v7.3.2: Stick calibration
        grid.addWidget(QLabel("Stick Calibration:"), row, 0)
        cal_row = QHBoxLayout()
        btn_cal = QPushButton("Calibrate Sticks")
        btn_cal.setMaximumHeight(s(26))
        btn_cal.setToolTip(
            "Hold sticks in neutral position, then click.\n"
            "Samples for 2 seconds to measure center offsets.")
        btn_cal.clicked.connect(self._calibrate_xbox_sticks)
        cal_row.addWidget(btn_cal)
        btn_clear_cal = QPushButton("Clear")
        btn_clear_cal.setMaximumHeight(s(26))
        btn_clear_cal.setToolTip("Remove stick calibration offsets")
        btn_clear_cal.clicked.connect(self._clear_xbox_stick_cal)
        cal_row.addWidget(btn_clear_cal)
        grid.addLayout(cal_row, row, 1, 1, 2)
        row += 1

        self.xbox_cal_label = QLabel("")
        self.xbox_cal_label.setWordWrap(True)
        self.xbox_cal_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt;")
        grid.addWidget(self.xbox_cal_label, row, 0, 1, 3)
        row += 1

        # v7.3.4: Per-axis deadzone thresholds
        from PySide6.QtWidgets import QDoubleSpinBox as _DSB
        grid.addWidget(QLabel("Stick Deadzone:"), row, 0)
        self.xbox_stick_dz_spin = _DSB()
        self.xbox_stick_dz_spin.setRange(0, 95)
        self.xbox_stick_dz_spin.setDecimals(1)
        self.xbox_stick_dz_spin.setSuffix(" %")
        self.xbox_stick_dz_spin.setValue(20.0)
        self.xbox_stick_dz_spin.setToolTip(
            "Deadzone for left and right sticks (axes 0–3).\n"
            "Input below this % of full deflection is ignored.")
        self.xbox_stick_dz_spin.setMaximumWidth(s(90))
        grid.addWidget(self.xbox_stick_dz_spin, row, 1)
        row += 1

        grid.addWidget(QLabel("Trigger Deadzone:"), row, 0)
        self.xbox_trigger_dz_spin = _DSB()
        self.xbox_trigger_dz_spin.setRange(0, 95)
        self.xbox_trigger_dz_spin.setDecimals(1)
        self.xbox_trigger_dz_spin.setSuffix(" %")
        self.xbox_trigger_dz_spin.setValue(5.0)
        self.xbox_trigger_dz_spin.setToolTip(
            "Deadzone for left and right triggers (axes 4–5).\n"
            "Input below this % of full deflection is ignored.\n"
            "Keep low — triggers often rest at a non-zero value.")
        self.xbox_trigger_dz_spin.setMaximumWidth(s(90))
        grid.addWidget(self.xbox_trigger_dz_spin, row, 1)
        row += 1

        # v7.3.4: Debug mode
        from PySide6.QtWidgets import QCheckBox as _QCB
        self.chk_xbox_debug = _QCB("Debug Mode")
        self.chk_xbox_debug.setToolTip(
            "Log trigger values, axis dispatches, and pump velocity changes.\n"
            "Reconnect the controller after toggling to apply.")
        self.chk_xbox_debug.setStyleSheet(f"color: {COLORS['subtext0']};")
        grid.addWidget(self.chk_xbox_debug, row, 0, 1, 3)
        row += 1

        layout.addLayout(grid)
        self._load_xbox_cal_label()
        parent_layout.addWidget(card)

    # ── Axis Flip Card (v7.3.2) ─────────────────────────────────

    def _build_axis_flip_card(self, parent_layout):
        """Card with checkboxes to flip positive direction for Z and pumps."""
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("Axis Direction (Flip Positive)")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        desc = QLabel(
            "If a motor moves the wrong way, flip its positive direction.\n"
            "This is a per-machine setting stored locally.")
        desc.setWordWrap(True)
        desc.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(desc)

        self._flip_checks: dict[str, QCheckBox] = {}
        row_lay = QHBoxLayout()
        row_lay.setSpacing(12)
        for axis in ["Z", "P1", "P2", "P3"]:
            chk = QCheckBox(f"Flip {axis}")
            chk.setToolTip(f"Invert the positive direction for {axis}")
            row_lay.addWidget(chk)
            self._flip_checks[axis] = chk
        row_lay.addStretch()
        layout.addLayout(row_lay)

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

        self.chk_debug_xy = QCheckBox("XY Position Debug Log (CSV)")
        self.chk_debug_xy.setToolTip(
            "Record raw serial position data and jog commands to xy_debug_*.csv in the project folder.\n"
            "Takes effect on next application startup.")
        layout.addWidget(self.chk_debug_xy)

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

        # v7.3.5: ZP Stage settings
        from SupportClasses.ZPStage import ZPStageManager
        default_fr = ZPStageManager.DEFAULT_FEEDRATE
        self.spin_zp_max_feedrate.setValue(
            self.settings.get("zp_stage.max_feedrate", default_fr))
        self.spin_zp_retract_feedrate.setValue(
            self.settings.get("zp_stage.retract_feedrate", default_fr))
        self.spin_zp_insert_feedrate.setValue(
            self.settings.get("zp_stage.insert_feedrate", default_fr / 2))
        self.spin_zp_jog_feedrate.setValue(
            self.settings.get("zp_stage.jog_feedrate", default_fr))
        self.chk_zp_position_save.setChecked(
            bool(self.settings.get("zp_stage.auto_save_position", False)))

        # Safety limits
        self.chk_safety_enabled.setChecked(sl.enabled)
        self.spin_xy_min_x.setValue(sl.xy_min_x)
        self.spin_xy_max_x.setValue(sl.xy_max_x)
        self.spin_xy_min_y.setValue(sl.xy_min_y)
        self.spin_xy_max_y.setValue(sl.xy_max_y)
        self.spin_z_min.setValue(sl.z_min)
        self.spin_z_max.setValue(sl.z_max)
        # v7.2.6: Load per-pump limits
        for pid, attr_min, attr_max in [
            ('P1', 'p1_min', 'p1_max'),
            ('P2', 'p2_min', 'p2_max'),
            ('P3', 'p3_min', 'p3_max'),
        ]:
            if pid in self._pump_min_spins:
                self._pump_min_spins[pid].setValue(getattr(sl, attr_min))
                self._pump_max_spins[pid].setValue(getattr(sl, attr_max))
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
        # v7.3.4: Deadzone spinboxes
        if hasattr(self, 'xbox_stick_dz_spin'):
            self.xbox_stick_dz_spin.setValue(
                self.settings.get("xbox.deadzones.sticks", 0.20) * 100)
        if hasattr(self, 'xbox_trigger_dz_spin'):
            self.xbox_trigger_dz_spin.setValue(
                self.settings.get("xbox.deadzones.triggers", 0.05) * 100)
        if hasattr(self, 'chk_xbox_debug'):
            self.chk_xbox_debug.setChecked(
                bool(self.settings.get("xbox.debug_mode", False)))

        # Logging
        self.chk_verbose.setChecked(
            self.settings.get("logging.verbose", False))
        self.chk_debug_xy.setChecked(
            self.settings.get("logging.debug_xy", False))

        # v7.3.2: Axis flip
        saved_flips = self.settings.get_section("axis_flip") or {}
        for axis, chk in self._flip_checks.items():
            chk.setChecked(bool(saved_flips.get(axis, False)))

        # Unit conversion factor
        um_val = self.settings.get("stage.xy_position_scale",
                                   DEFAULT_XY_POSITION_SCALE)
        self._xy_position_scale = float(um_val)
        if hasattr(self, 'spin_um_factor'):
            self.spin_um_factor.setValue(self._xy_position_scale)

        self._refresh_ports()
        self._update_context_sim_labels()

        # Controller protocol (P8.22)
        ctrl_json = self.settings.get("controller.controller_json")
        if ctrl_json == "auto":
            self.combo_controller.setCurrentIndex(0)  # Auto-Detect
        elif ctrl_json is None:
            self.combo_controller.setCurrentIndex(1)  # Default
        else:
            # Find matching item by data
            for i in range(self.combo_controller.count()):
                if self.combo_controller.itemData(i) == ctrl_json:
                    self.combo_controller.setCurrentIndex(i)
                    break

        # Show last auto-detect result
        last_detect = self.settings.get("controller.auto_detect_result")
        if last_detect:
            self.lbl_controller_status.setText(f"Last detected: {last_detect}")

        # Show last rate test
        rate_results = self.settings.get("motion_controller.rate_test_results")
        if rate_results and isinstance(rate_results, dict):
            self.lbl_rate_results.setText(
                f"Last test: {rate_results.get('avg_round_trip_ms', 0):.1f}ms avg | "
                f"{rate_results.get('max_command_hz', 0):.0f} Hz"
            )

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
        # v7.2.6: Apply per-pump limits
        for pid, attr_min, attr_max in [
            ('P1', 'p1_min', 'p1_max'),
            ('P2', 'p2_min', 'p2_max'),
            ('P3', 'p3_min', 'p3_max'),
        ]:
            if pid in self._pump_min_spins:
                setattr(sl, attr_min, self._pump_min_spins[pid].value())
                setattr(sl, attr_max, self._pump_max_spins[pid].value())
        sl.max_z_feedrate = self.spin_max_z_feed.value()
        sl.max_pump_feedrate = self.spin_max_p_feed.value()

        self.settings.set_section("safety_limits", sl.to_dict())

        # v7.3.5: ZP Stage settings → apply to hardware + persist
        zp_max_fr = self.spin_zp_max_feedrate.value()
        zp_retract_fr = self.spin_zp_retract_feedrate.value()
        zp_insert_fr = self.spin_zp_insert_feedrate.value()
        zp_jog_fr = self.spin_zp_jog_feedrate.value()
        zp_auto_save = self.chk_zp_position_save.isChecked()

        self.settings.set("zp_stage.max_feedrate", zp_max_fr)
        self.settings.set("zp_stage.retract_feedrate", zp_retract_fr)
        self.settings.set("zp_stage.insert_feedrate", zp_insert_fr)
        self.settings.set("zp_stage.jog_feedrate", zp_jog_fr)
        self.settings.set("zp_stage.auto_save_position", zp_auto_save)

        # Apply max feedrate to hardware (M203)
        if self.controller.is_zp_connected and self.controller.zp_stage:
            self.controller.zp_stage.set_max_feedrate(zp_max_fr)
            self.controller.zp_stage.feedrate = zp_jog_fr

        # Store retract/insert feedrates on controller for safe_travel_to
        self.controller._zp_retract_feedrate = zp_retract_fr
        self.controller._zp_insert_feedrate = zp_insert_fr

        # Enable/disable periodic position save
        self.controller._zp_auto_save_position = zp_auto_save

        # Polling → apply immediately (GUI timer reads setting on next tick)
        poll_ms = self.spin_poll_interval.value()
        self.controller._pos_poller.poll_interval = poll_ms / 1000.0
        self.settings.set("polling.position_interval_ms", poll_ms)
        self.controller._watchdog.check_interval = self.spin_watchdog.value()
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
        # v7.3.4: Deadzone thresholds (stored as 0–1 fractions)
        if hasattr(self, 'xbox_stick_dz_spin'):
            self.settings.set(
                "xbox.deadzones.sticks",
                round(self.xbox_stick_dz_spin.value() / 100.0, 4))
        if hasattr(self, 'xbox_trigger_dz_spin'):
            self.settings.set(
                "xbox.deadzones.triggers",
                round(self.xbox_trigger_dz_spin.value() / 100.0, 4))
        if hasattr(self, 'chk_xbox_debug'):
            self.settings.set("xbox.debug_mode", self.chk_xbox_debug.isChecked())

        # Logging → apply immediately
        verbose = self.chk_verbose.isChecked()
        self.settings.set("logging.verbose", verbose)
        log_level = logging.DEBUG if verbose else logging.INFO
        logging.getLogger().setLevel(log_level)

        self.settings.set("logging.debug_xy", self.chk_debug_xy.isChecked())

        # v7.3.2: Axis flip → controller + settings
        flip_dict = {axis: chk.isChecked()
                     for axis, chk in self._flip_checks.items()}
        self.controller.set_axis_flips(flip_dict)
        self.settings.set_section("axis_flip", flip_dict)

        # P8.23: Controller protocol selection
        ctrl_data = self.combo_controller.currentData()
        self.settings.set("controller.controller_json", ctrl_data)

        # Unit conversion factor
        if hasattr(self, 'spin_um_factor'):
            um_val = self.spin_um_factor.value()
            self.settings.set("stage.xy_position_scale", um_val)
            self._xy_position_scale = um_val

        self.settings.save()
        logger.info("Settings applied and saved")

        # Update µm summary
        self._update_safety_um_label()

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

    def _update_safety_um_label(self):
        """Update the safety limits µm summary label."""
        if not hasattr(self, 'lbl_safety_um'):
            return
        try:
            sl = self.controller.safety_limits
            self.lbl_safety_um.setText(
                convert_safety_xy_text(sl, self._xy_position_scale))
        except Exception:
            self.lbl_safety_um.setText("")

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

    def _calibrate_xbox_sticks(self):
        """v7.3.2: Run stick center calibration and save offsets."""
        self.xbox_cal_label.setText("Sampling... keep sticks centered!")
        self.xbox_cal_label.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: 9pt;")
        # Force UI repaint before blocking call
        from PySide6.QtWidgets import QApplication
        QApplication.processEvents()

        try:
            offsets = self.controller.calibrate_xbox_sticks(duration=2.0)
            # Save as string keys for JSON
            save_dict = {str(k): round(v, 6) for k, v in offsets.items()}
            self.settings.set_section("xbox_stick_offsets", save_dict)
            self.settings.save()

            parts = [f"Axis {k}: {v:+.4f}" for k, v in sorted(offsets.items())]
            self.xbox_cal_label.setText("Calibrated: " + ", ".join(parts))
            self.xbox_cal_label.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 9pt;")
            logger.info(f"Xbox stick calibration saved: {save_dict}")
        except Exception as e:
            self.xbox_cal_label.setText(f"Calibration failed: {e}")
            self.xbox_cal_label.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 9pt;")

    def _clear_xbox_stick_cal(self):
        """v7.3.2: Clear saved stick calibration offsets."""
        self.settings.set_section("xbox_stick_offsets", {})
        self.settings.save()
        self.xbox_cal_label.setText("Calibration cleared")
        self.xbox_cal_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt;")
        logger.info("Xbox stick calibration cleared")

    def _load_xbox_cal_label(self):
        """v7.3.2: Show saved calibration in label."""
        saved = self.settings.get_section("xbox_stick_offsets")
        if saved and any(v != 0 for v in saved.values()):
            parts = [f"Axis {k}: {v:+.4f}" for k, v in sorted(saved.items())]
            self.xbox_cal_label.setText("Saved: " + ", ".join(parts))
        else:
            self.xbox_cal_label.setText("No calibration (using defaults)")

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
            ux = stage_to_um(zero_x, self._xy_position_scale)
            uy = stage_to_um(zero_y, self._xy_position_scale)
            logger.info(
                f"XY {'max' if as_max else 'min'} set to "
                f"({ux:,.1f}, {uy:,.1f}) µm")

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

    def _set_pump_from_current(self, pump: str, as_max: bool = True):
        """v7.2.6: Set pump limit from the current position."""
        pos = self.controller.get_zp_position(cached=True)
        if pos[0] is not None:
            idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
            if idx < len(pos) and pos[idx] is not None:
                zero_ref = self.controller.zero_position.get(pump, 0)
                rel_mm = pos[idx] - zero_ref
                if as_max:
                    self._pump_max_spins[pump].setValue(rel_mm)
                else:
                    self._pump_min_spins[pump].setValue(rel_mm)
                logger.info(
                    f"{pump} {'max' if as_max else 'min'} set to "
                    f"{rel_mm:.2f} mm (from current)")

    def _reset_pump_zero(self, pump: str):
        """v7.2.6: Reset the zero reference for a single pump."""
        if hasattr(self.controller, 'reset_pump_zero'):
            self.controller.reset_pump_zero(pump)
            # Save the updated zero_position to settings
            self.settings.set_section(
                "zero_position", dict(self.controller.zero_position))
            self.settings.save()
            logger.info(f"{pump} zero reference reset and saved")
            if self._context_widget:
                self.ctx_status_label.setText(f"{pump} zero reset ✓")
                self.ctx_status_label.setStyleSheet(
                    f"color: {COLORS['green']};")

