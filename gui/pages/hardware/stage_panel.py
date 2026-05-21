"""
stage_panel.py — Stage sub-page of Hardware Setup (v7.4.0-b).

New home for widgets that were on the Settings page in v7.3.x but
conceptually belong to hardware setup:
    - Safety limits (XY/Z/Pump min/max + feedrate caps)
    - ZP stage feedrates (max/retract/insert/jog)
    - Axis direction flips (per-machine)
    - Zero-calibration jog (set X/Y/Z/P zero)

This widget is mounted inside the "Stage" sub-page of HardwareSetupPage.
SettingsPage no longer carries these — the Settings page is now strictly
user/system preferences.

State is read from / written to the same Settings instance used by the
rest of the app, so all existing wiring (StageController applies safety
limits, polling, etc.) continues to work without changes.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QGroupBox, QLabel, QDoubleSpinBox, QCheckBox, QPushButton, QFrame,
    QSizePolicy, QComboBox, QInputDialog, QMessageBox,
)
from PySide6.QtCore import Qt, Signal

from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.scaling import s, sf, sp, scaled_font_size
from gui.pages.hardware.device_profile import (
    DeviceProfile, list_profiles, delete_profile, DEVICES_DIR,
)

logger = logging.getLogger(__name__)


class StageHardwarePanel(QWidget):
    """v7.4.0-b: Stage hardware configuration sub-page.

    Widgets here previously lived on the Settings page. They moved to
    Hardware Setup because they describe the *physical machine* (safety
    envelope, motor direction, feedrate ceilings) rather than user
    preferences. The Settings page now hosts only simulation toggles,
    polling intervals, Xbox mapping, logging, and serial port config.
    """

    settings_applied = Signal()  # Emitted when "Apply" is pressed

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._settings = None
        self._controller = None
        # v7.4.1 hotfix: explicit background so the panel doesn't fall
        # through to Qt's default light grey. Scoped to this widget via
        # objectName so children keep their own QSS rules.
        self.setObjectName("stageHardwarePanel")
        self.setStyleSheet(
            f"#stageHardwarePanel {{ background-color: {COLORS['base']}; }}")
        self._setup_ui()

    # ── Public injection points ──────────────────────────────────

    def set_settings(self, settings):
        """Inject the Settings instance. Loads initial values into widgets."""
        self._settings = settings
        if settings is not None:
            self._load_from_settings()

    def set_controller(self, controller):
        """Inject StageController for the zero-calibration jog row.

        v7.4.2: Also drives the Connect Hardware status badges and the
        per-axis Jog + Record Limits group.
        """
        self._controller = controller
        # Initial badge sync once a controller is available
        if hasattr(self, 'badge_xy'):
            self._sync_connection_badges()

    def on_status_update(self):
        """v7.4.2: Periodic tick from MainWindow.

        Refreshes the per-axis position readouts and connection status
        badges. Cheap — uses cached positions.
        """
        if not hasattr(self, 'lbl_axis_pos'):
            return
        self._refresh_jog_positions()
        self._sync_connection_badges()

    def _sync_connection_badges(self):
        """v7.4.2: Reflect controller's live connection state in badges."""
        if self._controller is None or not hasattr(self, 'badge_xy'):
            return
        xy_ok = bool(getattr(self._controller, 'is_xy_connected', False))
        zp_ok = bool(getattr(self._controller, 'is_zp_connected', False))
        self.badge_xy.set_status("ok" if xy_ok else "pending",
                                 "Connected" if xy_ok else "Not connected")
        self.badge_zp.set_status("ok" if zp_ok else "pending",
                                 "Connected" if zp_ok else "Not connected")

    # ── UI ───────────────────────────────────────────────────────

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setSpacing(s(12))
        outer.setContentsMargins(s(12), s(12), s(12), s(12))

        # Banner: explains what's here and where it came from
        banner = QLabel(
            "<b>Initial device setup.</b> Safety envelope, motor "
            "feedrates, and axis direction describe the physical "
            "machine — independent of which experiment you're "
            "running. Pick a device profile below or save your own."
        )
        banner.setWordWrap(True)
        banner.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9.5)}pt; "
            f"padding: {sp(6)} {sp(10)};"
            f"border-left: 2px solid {COLORS['mauve']};"
        )
        outer.addWidget(banner)

        # v7.4.1: Device profile picker — sits at the top because choosing
        # a profile populates all the groups below
        outer.addWidget(self._build_device_profile_group())

        # v7.4.2: Connect Hardware + Axis Mapping + Steps Calibration
        outer.addWidget(self._build_connect_group())
        outer.addWidget(self._build_axis_mapping_group())
        outer.addWidget(self._build_steps_cal_group())

        outer.addWidget(self._build_safety_group())
        # v7.4.2: Per-axis Jog + Record Min/Max for limits discovery
        outer.addWidget(self._build_jog_limits_group())
        outer.addWidget(self._build_zp_feedrates_group())
        outer.addWidget(self._build_axis_flip_group())

        # Apply / Reset buttons
        btn_row = QHBoxLayout()
        self.btn_apply = QPushButton("Apply Settings")
        self.btn_apply.setObjectName("accentBtn")
        self.btn_apply.clicked.connect(self._apply)
        btn_row.addWidget(self.btn_apply)

        self.btn_reset = QPushButton("Reset to Defaults")
        self.btn_reset.setObjectName("dangerBtn")
        self.btn_reset.clicked.connect(self._reset_defaults)
        btn_row.addWidget(self.btn_reset)
        btn_row.addStretch()
        outer.addLayout(btn_row)

        outer.addStretch()

    # ── v7.4.1: Device profile picker ────────────────────────────

    def _build_device_profile_group(self) -> QGroupBox:
        grp = QGroupBox("Device Profile")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)

        # Row 1: profile picker + active label
        row = QHBoxLayout()
        row.addWidget(QLabel("Profile:"))
        self.cmb_profile = QComboBox()
        self.cmb_profile.setMinimumWidth(s(220))
        self.cmb_profile.currentIndexChanged.connect(self._on_profile_picked)
        row.addWidget(self.cmb_profile, 1)

        self.btn_refresh_profiles = QPushButton("🔄")
        self.btn_refresh_profiles.setFixedWidth(s(32))
        self.btn_refresh_profiles.setToolTip("Refresh profile list")
        self.btn_refresh_profiles.clicked.connect(self._refresh_profile_list)
        row.addWidget(self.btn_refresh_profiles)
        lay.addLayout(row)

        # Row 2: action buttons
        action_row = QHBoxLayout()
        self.btn_load_profile = QPushButton("Load")
        self.btn_load_profile.clicked.connect(self._load_selected_profile)
        action_row.addWidget(self.btn_load_profile)

        self.btn_save_profile = QPushButton("Save")
        self.btn_save_profile.setToolTip(
            "Save current settings back to the selected profile")
        self.btn_save_profile.clicked.connect(self._save_to_selected_profile)
        action_row.addWidget(self.btn_save_profile)

        self.btn_save_as_profile = QPushButton("Save As…")
        self.btn_save_as_profile.setObjectName("accentBtn")
        self.btn_save_as_profile.clicked.connect(self._save_as_new_profile)
        action_row.addWidget(self.btn_save_as_profile)

        self.btn_delete_profile = QPushButton("Delete")
        self.btn_delete_profile.setObjectName("dangerBtn")
        self.btn_delete_profile.clicked.connect(self._delete_selected_profile)
        action_row.addWidget(self.btn_delete_profile)
        action_row.addStretch()
        lay.addLayout(action_row)

        # Status label
        self.lbl_profile_status = QLabel(
            f"Profiles directory: {DEVICES_DIR}")
        self.lbl_profile_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self.lbl_profile_status.setWordWrap(True)
        lay.addWidget(self.lbl_profile_status)

        # Populate combo
        self._refresh_profile_list()

        return grp

    def _build_safety_group(self) -> QGroupBox:
        """v7.4.1: Uses canonical SafetyLimits field names (xy_min_x etc.)."""
        grp = QGroupBox("Safety Limits")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        form = QGridLayout(grp)
        form.setSpacing(s(6))

        def _spin(default: float, unit: str, decimals: int = 1,
                  lo: float = -1e6, hi: float = 1e6) -> QDoubleSpinBox:
            sp_w = QDoubleSpinBox()
            sp_w.setRange(lo, hi)
            sp_w.setDecimals(decimals)
            sp_w.setSuffix(f" {unit}")
            sp_w.setValue(default)
            sp_w.setMinimumWidth(s(120))
            return sp_w

        row = 0
        form.addWidget(QLabel("Enable safety limits:"), row, 0)
        self.chk_safety_enabled = QCheckBox()
        self.chk_safety_enabled.setChecked(True)
        form.addWidget(self.chk_safety_enabled, row, 1)

        # XY — separate X and Y bounds (match SafetyLimits.xy_{min,max}_{x,y})
        row += 1
        form.addWidget(QLabel("XY X min (µm):"), row, 0)
        self.spin_xy_min_x = _spin(-130000.0, "µm")
        form.addWidget(self.spin_xy_min_x, row, 1)
        form.addWidget(QLabel("XY X max (µm):"), row, 2)
        self.spin_xy_max_x = _spin(130000.0, "µm")
        form.addWidget(self.spin_xy_max_x, row, 3)

        row += 1
        form.addWidget(QLabel("XY Y min (µm):"), row, 0)
        self.spin_xy_min_y = _spin(-85000.0, "µm")
        form.addWidget(self.spin_xy_min_y, row, 1)
        form.addWidget(QLabel("XY Y max (µm):"), row, 2)
        self.spin_xy_max_y = _spin(85000.0, "µm")
        form.addWidget(self.spin_xy_max_y, row, 3)

        row += 1
        form.addWidget(QLabel("Z min (mm):"), row, 0)
        self.spin_z_min = _spin(-10.0, "mm", decimals=3)
        form.addWidget(self.spin_z_min, row, 1)
        form.addWidget(QLabel("Z max (mm):"), row, 2)
        self.spin_z_max = _spin(50.0, "mm", decimals=3)
        form.addWidget(self.spin_z_max, row, 3)

        # Per-pump limits (P1/P2/P3 — match SafetyLimits.{p1,p2,p3}_{min,max})
        self.spin_p_mins: dict[str, QDoubleSpinBox] = {}
        self.spin_p_maxs: dict[str, QDoubleSpinBox] = {}
        for pid, default_lo, default_hi in [
            ("P1", -50.0, 50.0),
            ("P2", -50.0, 50.0),
            ("P3", -50.0, 50.0),
        ]:
            row += 1
            form.addWidget(QLabel(f"{pid} min (mm):"), row, 0)
            sp_min = _spin(default_lo, "mm", decimals=3)
            form.addWidget(sp_min, row, 1)
            form.addWidget(QLabel(f"{pid} max (mm):"), row, 2)
            sp_max = _spin(default_hi, "mm", decimals=3)
            form.addWidget(sp_max, row, 3)
            self.spin_p_mins[pid] = sp_min
            self.spin_p_maxs[pid] = sp_max

        row += 1
        form.addWidget(QLabel("Max XY speed (µm/s):"), row, 0)
        self.spin_max_xy_speed = _spin(10000.0, "µm/s")
        form.addWidget(self.spin_max_xy_speed, row, 1)
        form.addWidget(QLabel("Max Z feedrate (mm/min):"), row, 2)
        self.spin_max_z_feed = _spin(500.0, "mm/min")
        form.addWidget(self.spin_max_z_feed, row, 3)

        row += 1
        form.addWidget(QLabel("Max pump feedrate (mm/min):"), row, 0)
        self.spin_max_pump_feed = _spin(200.0, "mm/min")
        form.addWidget(self.spin_max_pump_feed, row, 1)

        return grp

    def _build_zp_feedrates_group(self) -> QGroupBox:
        grp = QGroupBox("ZP Stage Feedrates")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        form = QFormLayout(grp)

        def _spin(default: float, unit: str = "mm/min") -> QDoubleSpinBox:
            sp_w = QDoubleSpinBox()
            sp_w.setRange(0.1, 100000.0)
            sp_w.setDecimals(1)
            sp_w.setSuffix(f" {unit}")
            sp_w.setValue(default)
            sp_w.setMinimumWidth(s(160))
            return sp_w

        self.spin_zp_max_feed = _spin(3000.0)
        form.addRow("Max feedrate:", self.spin_zp_max_feed)

        self.spin_zp_retract_feed = _spin(1500.0)
        form.addRow("Retract feedrate:", self.spin_zp_retract_feed)

        self.spin_zp_insert_feed = _spin(600.0)
        form.addRow("Insert feedrate:", self.spin_zp_insert_feed)

        self.spin_zp_jog_feed = _spin(900.0)
        form.addRow("Jog feedrate:", self.spin_zp_jog_feed)

        self.chk_zp_autosave = QCheckBox("Auto-save position to EEPROM")
        self.chk_zp_autosave.setChecked(False)
        form.addRow("", self.chk_zp_autosave)

        return grp

    def _build_axis_flip_group(self) -> QGroupBox:
        grp = QGroupBox("Axis Direction Flips")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)

        info = QLabel(
            "Flip the positive direction for an axis if your machine moves "
            "the opposite way of expected. Per-machine setting."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        lay.addWidget(info)

        row = QHBoxLayout()
        self.chk_flip_z = QCheckBox("Flip Z")
        self.chk_flip_p1 = QCheckBox("Flip P1")
        self.chk_flip_p2 = QCheckBox("Flip P2")
        self.chk_flip_p3 = QCheckBox("Flip P3")
        for chk in (self.chk_flip_z, self.chk_flip_p1, self.chk_flip_p2, self.chk_flip_p3):
            row.addWidget(chk)
        row.addStretch()
        lay.addLayout(row)

        return grp

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2: Connect Hardware
    # ════════════════════════════════════════════════════════════════

    def _build_connect_group(self) -> QGroupBox:
        """Connect XY / ZP / Xbox with live status badges."""
        from gui.widgets.components import StatusBadge

        grp = QGroupBox("Connect Hardware")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        grid = QGridLayout(grp)
        grid.setSpacing(s(6))

        row = 0
        grid.addWidget(QLabel("XY stage:"), row, 0)
        self.btn_connect_xy = QPushButton("Connect")
        self.btn_connect_xy.setObjectName("accentBtn")
        self.btn_connect_xy.clicked.connect(self._connect_xy)
        grid.addWidget(self.btn_connect_xy, row, 1)
        self.btn_disconnect_xy = QPushButton("Disconnect")
        self.btn_disconnect_xy.setObjectName("dangerBtn")
        self.btn_disconnect_xy.clicked.connect(self._disconnect_xy)
        grid.addWidget(self.btn_disconnect_xy, row, 2)
        self.badge_xy = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_xy, row, 3)

        row += 1
        grid.addWidget(QLabel("Z + Pumps:"), row, 0)
        self.btn_connect_zp = QPushButton("Connect")
        self.btn_connect_zp.setObjectName("accentBtn")
        self.btn_connect_zp.clicked.connect(self._connect_zp)
        grid.addWidget(self.btn_connect_zp, row, 1)
        self.btn_disconnect_zp = QPushButton("Disconnect")
        self.btn_disconnect_zp.setObjectName("dangerBtn")
        self.btn_disconnect_zp.clicked.connect(self._disconnect_zp)
        grid.addWidget(self.btn_disconnect_zp, row, 2)
        self.badge_zp = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_zp, row, 3)

        return grp

    def _connect_xy(self):
        if self._controller is None:
            return
        self.badge_xy.set_status("info", "Connecting…")
        try:
            self._controller.connect_xy()
            ok = bool(self._controller.is_xy_connected)
            self.badge_xy.set_status("ok" if ok else "err",
                                     "Connected" if ok else "Failed")
        except Exception as e:
            self.badge_xy.set_status("err", f"Error: {e}")

    def _disconnect_xy(self):
        if self._controller is None:
            return
        try:
            self._controller.disconnect_xy()
        except Exception as e:
            logger.warning(f"disconnect_xy failed: {e}")
        self.badge_xy.set_status("pending", "Not connected")

    def _connect_zp(self):
        if self._controller is None:
            return
        self.badge_zp.set_status("info", "Connecting…")
        try:
            self._controller.connect_zp()
            ok = bool(self._controller.is_zp_connected)
            self.badge_zp.set_status("ok" if ok else "err",
                                     "Connected" if ok else "Failed")
        except Exception as e:
            self.badge_zp.set_status("err", f"Error: {e}")

    def _disconnect_zp(self):
        if self._controller is None:
            return
        try:
            self._controller.disconnect_zp()
        except Exception as e:
            logger.warning(f"disconnect_zp failed: {e}")
        self.badge_zp.set_status("pending", "Not connected")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2: Axis Mapping (logical → physical Marlin axis)
    # ════════════════════════════════════════════════════════════════

    _PHYSICAL_LETTERS = ["X", "Y", "Z", "E"]
    _LOGICAL_AXES = ["Z", "P1", "P2", "P3"]

    def _build_axis_mapping_group(self) -> QGroupBox:
        grp = QGroupBox("Axis Mapping (logical → Marlin)")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)

        info = QLabel(
            "Which physical Marlin axis drives each logical axis on "
            "your machine. Defaults: Z→X, P1→Y, P2→Z, P3→E. Change "
            "these to match how your steppers are wired."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        lay.addWidget(info)

        self.cmb_axis_map: dict[str, QComboBox] = {}
        grid = QGridLayout()
        grid.setSpacing(s(6))
        for r, logical in enumerate(self._LOGICAL_AXES):
            grid.addWidget(QLabel(f"{logical}:"), r, 0)
            grid.addWidget(QLabel("→"), r, 1)
            cmb = QComboBox()
            for letter in self._PHYSICAL_LETTERS:
                cmb.addItem(f"{letter}  (Marlin {letter})", letter)
            cmb.setMinimumWidth(s(160))
            grid.addWidget(cmb, r, 2)
            self.cmb_axis_map[logical] = cmb
        lay.addLayout(grid)

        return grp

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2: Steps-per-mm Calibration
    # ════════════════════════════════════════════════════════════════

    def _build_steps_cal_group(self) -> QGroupBox:
        grp = QGroupBox("Stepper Calibration (steps per mm)")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "<b>How to calibrate:</b> Pick an axis. Click <i>Command Move</i> "
            "to send a known-distance move. Measure the actual physical "
            "movement and enter it. Click <i>Calculate & Send M92</i> — "
            "the new steps/mm is sent to Marlin and saved to this device "
            "profile. Negative values invert direction."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(info)

        # Current per-axis steps/mm grid (read-only display)
        self.lbl_steps_grid: dict[str, QLabel] = {}
        steps_row = QHBoxLayout()
        for ax in self._LOGICAL_AXES:
            cell = QLabel(f"{ax}: —")
            cell.setStyleSheet(
                f"color: {COLORS['text']}; font-family: monospace; "
                f"padding: {sp(2)} {sp(8)}; "
                f"border: 1px solid {COLORS['surface1']}; "
                f"border-radius: {sp(4)};")
            steps_row.addWidget(cell)
            self.lbl_steps_grid[ax] = cell
        steps_row.addStretch()
        outer.addLayout(steps_row)

        # Workflow row
        wf = QHBoxLayout()
        wf.addWidget(QLabel("Axis:"))
        self.cmb_cal_axis = QComboBox()
        for ax in self._LOGICAL_AXES:
            self.cmb_cal_axis.addItem(ax, ax)
        wf.addWidget(self.cmb_cal_axis)

        wf.addWidget(QLabel("Commanded:"))
        self.spin_cal_commanded = QDoubleSpinBox()
        self.spin_cal_commanded.setRange(0.001, 100.0)
        self.spin_cal_commanded.setDecimals(3)
        self.spin_cal_commanded.setValue(1.0)
        self.spin_cal_commanded.setSuffix(" mm")
        wf.addWidget(self.spin_cal_commanded)

        self.btn_cal_move = QPushButton("Command Move")
        self.btn_cal_move.clicked.connect(self._cal_command_move)
        wf.addWidget(self.btn_cal_move)

        wf.addWidget(QLabel("Measured:"))
        self.spin_cal_measured = QDoubleSpinBox()
        self.spin_cal_measured.setRange(0.001, 100.0)
        self.spin_cal_measured.setDecimals(3)
        self.spin_cal_measured.setValue(1.0)
        self.spin_cal_measured.setSuffix(" mm")
        wf.addWidget(self.spin_cal_measured)

        self.btn_cal_apply = QPushButton("Calculate && Send M92")
        self.btn_cal_apply.setObjectName("accentBtn")
        self.btn_cal_apply.clicked.connect(self._cal_apply)
        wf.addWidget(self.btn_cal_apply)

        wf.addStretch()
        outer.addLayout(wf)

        self.lbl_cal_status = QLabel("")
        self.lbl_cal_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self.lbl_cal_status.setWordWrap(True)
        outer.addWidget(self.lbl_cal_status)

        return grp

    def _cal_command_move(self):
        """Send a known-distance relative move on the selected logical axis."""
        if self._controller is None or not self._controller.is_zp_connected:
            self.lbl_cal_status.setText("ZP stage not connected.")
            return
        axis = self.cmb_cal_axis.currentData()
        distance = self.spin_cal_commanded.value()
        try:
            if axis == "Z":
                self._controller.move_z_relative(distance)
            else:
                self._controller.move_pump_relative(axis, distance)
            self.lbl_cal_status.setText(
                f"Sent {distance} mm move on {axis}. Measure actual "
                f"physical movement, then enter it on the right.")
        except Exception as e:
            self.lbl_cal_status.setText(f"Move failed: {e}")

    def _cal_apply(self):
        """Compute new steps/mm = current * (commanded / measured); send M92."""
        if self._controller is None or self._controller.zp_stage is None:
            self.lbl_cal_status.setText("ZP stage not connected.")
            return
        axis = self.cmb_cal_axis.currentData()
        commanded = self.spin_cal_commanded.value()
        measured = self.spin_cal_measured.value()
        if measured <= 0:
            self.lbl_cal_status.setText("Measured distance must be > 0.")
            return
        current = self._controller.zp_stage.steps_per_mm.get(axis, 5069)
        # Preserve sign of current calibration
        sign = -1 if current < 0 else 1
        new_abs = int(round(abs(current) * (commanded / measured)))
        new_value = sign * new_abs
        new_steps = dict(self._controller.zp_stage.steps_per_mm)
        new_steps[axis] = new_value
        self._controller.zp_stage.set_steps_per_mm(new_steps, persist=True)
        # Persist into settings + the live UI display
        if self._settings is not None:
            self._settings.set("device_profile.steps_per_mm", new_steps)
            self._settings.save()
        self._refresh_steps_grid()
        self.lbl_cal_status.setText(
            f"{axis}: {current} → {new_value} steps/mm "
            f"(commanded {commanded} / measured {measured}). M92 sent.")

    def _refresh_steps_grid(self):
        if self._controller and self._controller.zp_stage:
            steps = self._controller.zp_stage.steps_per_mm
        else:
            steps = (self._settings.get("device_profile.steps_per_mm")
                     if self._settings else {}) or {}
        for ax, lbl in self.lbl_steps_grid.items():
            val = steps.get(ax, "—")
            lbl.setText(f"{ax}: {val}")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2: Per-axis Stage Jog + Record Min/Max
    # ════════════════════════════════════════════════════════════════

    # Step sizes per axis: (fine, medium, coarse). XY in µm, Z/P in mm.
    _JOG_STEPS = {
        "X":  (10.0,    100.0,   1000.0),   # µm
        "Y":  (10.0,    100.0,   1000.0),   # µm
        "Z":  (0.01,    0.1,     1.0),       # mm
        "P1": (0.01,    0.1,     1.0),       # mm
        "P2": (0.01,    0.1,     1.0),       # mm
        "P3": (0.01,    0.1,     1.0),       # mm
    }

    def _build_jog_limits_group(self) -> QGroupBox:
        grp = QGroupBox("Per-Axis Jog && Record Limits")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "Jog each axis to its mechanical extremes. Click "
            "<b>Set as Min</b> / <b>Set as Max</b> to record the current "
            "position into the Safety Limits above. XY in µm; Z and pumps in mm."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(info)

        # v7.4.2 hotfix banner: explain the safety bypass scoped to this group
        bypass_banner = QLabel(
            "⚠ <b>Setup-mode jog.</b> These buttons bypass soft-limit "
            "clamping and the pump-enabled check so you can move freely "
            "to find the mechanical envelope. The normal Jog Control "
            "page still respects all safety limits."
        )
        bypass_banner.setWordWrap(True)
        bypass_banner.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: {sf(9)}pt; "
            f"padding: {sp(4)} {sp(8)};"
            f"border-left: 2px solid {COLORS['yellow']};"
            f"background: rgba(249, 226, 175, 18);"
        )
        outer.addWidget(bypass_banner)

        self.lbl_axis_pos: dict[str, QLabel] = {}
        for axis in ("X", "Y", "Z", "P1", "P2", "P3"):
            outer.addWidget(self._build_jog_row(axis))

        return grp

    def _build_jog_row(self, axis: str) -> QFrame:
        """Build one row of jog controls + record buttons for a single axis."""
        frame = QFrame()
        frame.setStyleSheet(
            f"QFrame {{ background: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {sp(4)}; padding: {sp(4)}; }}")
        h = QHBoxLayout(frame)
        h.setSpacing(s(4))
        h.setContentsMargins(s(6), s(4), s(6), s(4))

        unit = "µm" if axis in ("X", "Y") else "mm"

        # Axis label
        lbl_axis = QLabel(f"<b>{axis}</b>")
        lbl_axis.setMinimumWidth(s(28))
        h.addWidget(lbl_axis)

        # Position readout
        pos_lbl = QLabel("—")
        pos_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-family: monospace; "
            f"min-width: {sp(80)};")
        pos_lbl.setMinimumWidth(s(80))
        h.addWidget(pos_lbl)
        self.lbl_axis_pos[axis] = pos_lbl

        # Jog buttons: -coarse -med -fine + fine +med +coarse
        fine, med, coarse = self._JOG_STEPS[axis]
        for step, label, color_hint in [
            (-coarse, f"-{coarse:g}",  "red"),
            (-med,    f"-{med:g}",     "peach"),
            (-fine,   f"-{fine:g}",    "subtext0"),
            ( fine,   f"+{fine:g}",    "subtext0"),
            ( med,    f"+{med:g}",     "peach"),
            ( coarse, f"+{coarse:g}",  "green"),
        ]:
            btn = QPushButton(label)
            btn.setMaximumWidth(s(64))
            btn.setStyleSheet(
                f"color: {COLORS[color_hint]}; font-family: monospace;")
            btn.clicked.connect(
                lambda _checked=False, a=axis, d=step: self._jog(a, d))
            h.addWidget(btn)

        h.addWidget(QLabel(f"({unit})"))
        h.addStretch(1)

        # Record buttons
        btn_min = QPushButton("Set as Min")
        btn_min.setMaximumHeight(s(24))
        btn_min.clicked.connect(lambda _c=False, a=axis: self._record_limit(a, "min"))
        h.addWidget(btn_min)

        btn_max = QPushButton("Set as Max")
        btn_max.setMaximumHeight(s(24))
        btn_max.clicked.connect(lambda _c=False, a=axis: self._record_limit(a, "max"))
        h.addWidget(btn_max)

        return frame

    def _jog(self, axis: str, distance: float) -> None:
        """Device sub-page jog.

        v7.4.2 hotfix: passes ``bypass_safety=True`` so soft limits and
        the is_pump_enabled gate are bypassed for jog buttons originating
        from this page. The Device sub-page is the *initial* machine
        setup — you can't discover the mechanical extremes if soft
        limits stop you, and you can't jog pumps to find their range
        if HardwareConfig hasn't been set up yet.
        """
        if self._controller is None:
            return
        try:
            if axis in ("X", "Y"):
                dx = distance if axis == "X" else 0.0
                dy = distance if axis == "Y" else 0.0
                self._controller.move_xy_relative_um(
                    dx, dy, bypass_safety=True)
            elif axis == "Z":
                self._controller.move_z_relative(
                    distance, bypass_safety=True)
            else:
                self._controller.move_pump_relative(
                    axis, distance, bypass_safety=True)
        except Exception as e:
            logger.warning(f"Jog {axis} {distance} failed: {e}")
        # Refresh shown positions soon — controller updates them async
        self._refresh_jog_positions()

    def _refresh_jog_positions(self) -> None:
        """Update the per-axis position readouts. Called by jog + a 500ms tick."""
        ctrl = self._controller
        if ctrl is None or not hasattr(self, 'lbl_axis_pos'):
            return
        try:
            xy = ctrl.get_xy_position(cached=True)
            zp = ctrl.get_zp_position(cached=True)
        except Exception:
            return
        if xy and xy[0] is not None and "X" in self.lbl_axis_pos:
            self.lbl_axis_pos["X"].setText(f"{xy[0]:,.1f}")
        if xy and xy[1] is not None and "Y" in self.lbl_axis_pos:
            self.lbl_axis_pos["Y"].setText(f"{xy[1]:,.1f}")
        if zp and zp[0] is not None and "Z" in self.lbl_axis_pos:
            self.lbl_axis_pos["Z"].setText(f"{zp[0]:.3f}")
        for i, pid in enumerate(["P1", "P2", "P3"], start=1):
            if zp and i < len(zp) and zp[i] is not None and pid in self.lbl_axis_pos:
                self.lbl_axis_pos[pid].setText(f"{zp[i]:.3f}")

    def _record_limit(self, axis: str, which: str) -> None:
        """Copy current position into the matching safety spinbox."""
        ctrl = self._controller
        if ctrl is None:
            return
        try:
            xy = ctrl.get_xy_position(cached=True)
            zp = ctrl.get_zp_position(cached=True)
        except Exception:
            return
        zero = ctrl.zero_position
        if axis == "X" and xy and xy[0] is not None:
            target_spin = self.spin_xy_min_x if which == "min" else self.spin_xy_max_x
            target_spin.setValue(float(xy[0] - zero.get("x", 0)))
        elif axis == "Y" and xy and xy[1] is not None:
            target_spin = self.spin_xy_min_y if which == "min" else self.spin_xy_max_y
            target_spin.setValue(float(xy[1] - zero.get("y", 0)))
        elif axis == "Z" and zp and zp[0] is not None:
            target_spin = self.spin_z_min if which == "min" else self.spin_z_max
            target_spin.setValue(float(zp[0] - zero.get("Z", 0)))
        elif axis in ("P1", "P2", "P3"):
            idx = {"P1": 1, "P2": 2, "P3": 3}[axis]
            if zp and idx < len(zp) and zp[idx] is not None:
                target_dict = self.spin_p_mins if which == "min" else self.spin_p_maxs
                target_dict[axis].setValue(float(zp[idx] - zero.get(axis, 0)))

    # ── Settings I/O ─────────────────────────────────────────────

    def _load_from_settings(self):
        """v7.4.1: Uses canonical safety_limits.* field names."""
        s = self._settings
        if s is None:
            return
        # Safety
        self.chk_safety_enabled.setChecked(
            bool(s.get("safety_limits.enabled", True)))
        self.spin_xy_min_x.setValue(float(s.get("safety_limits.xy_min_x", -130000.0)))
        self.spin_xy_max_x.setValue(float(s.get("safety_limits.xy_max_x", 130000.0)))
        self.spin_xy_min_y.setValue(float(s.get("safety_limits.xy_min_y", -85000.0)))
        self.spin_xy_max_y.setValue(float(s.get("safety_limits.xy_max_y", 85000.0)))
        self.spin_z_min.setValue(float(s.get("safety_limits.z_min", -10.0)))
        self.spin_z_max.setValue(float(s.get("safety_limits.z_max", 50.0)))
        for pid in ("P1", "P2", "P3"):
            self.spin_p_mins[pid].setValue(
                float(s.get(f"safety_limits.{pid.lower()}_min", -50.0)))
            self.spin_p_maxs[pid].setValue(
                float(s.get(f"safety_limits.{pid.lower()}_max", 50.0)))
        self.spin_max_xy_speed.setValue(float(s.get("safety_limits.max_xy_speed", 10000.0)))
        self.spin_max_z_feed.setValue(float(s.get("safety_limits.max_z_feedrate", 500.0)))
        self.spin_max_pump_feed.setValue(float(s.get("safety_limits.max_pump_feedrate", 200.0)))

        # ZP feedrates
        self.spin_zp_max_feed.setValue(float(s.get("zp_stage.max_feedrate", 3000.0)))
        self.spin_zp_retract_feed.setValue(float(s.get("zp_stage.retract_feedrate", 1500.0)))
        self.spin_zp_insert_feed.setValue(float(s.get("zp_stage.insert_feedrate", 600.0)))
        self.spin_zp_jog_feed.setValue(float(s.get("zp_stage.jog_feedrate", 900.0)))
        self.chk_zp_autosave.setChecked(bool(s.get("zp_stage.auto_save_position", False)))

        # Axis flips
        self.chk_flip_z.setChecked(bool(s.get("axis_flip.z", False)))
        self.chk_flip_p1.setChecked(bool(s.get("axis_flip.p1", False)))
        self.chk_flip_p2.setChecked(bool(s.get("axis_flip.p2", False)))
        self.chk_flip_p3.setChecked(bool(s.get("axis_flip.p3", False)))

        # v7.4.1: Restore active profile selection
        active = s.get("device_profile.active", "")
        if active and hasattr(self, 'cmb_profile'):
            idx = self.cmb_profile.findText(active)
            if idx >= 0:
                self.cmb_profile.blockSignals(True)
                self.cmb_profile.setCurrentIndex(idx)
                self.cmb_profile.blockSignals(False)
                self.lbl_profile_status.setText(f"Active profile: {active}")

        # v7.4.2: Axis mapping
        if hasattr(self, 'cmb_axis_map'):
            axis_map = s.get("device_profile.axis_map") or {}
            for logical, cmb in self.cmb_axis_map.items():
                physical = axis_map.get(logical, "")
                if physical:
                    idx = cmb.findData(physical)
                    if idx >= 0:
                        cmb.blockSignals(True)
                        cmb.setCurrentIndex(idx)
                        cmb.blockSignals(False)

        # v7.4.2: Steps per mm grid display
        if hasattr(self, 'lbl_steps_grid'):
            self._refresh_steps_grid()

    def _apply(self):
        """v7.4.1: Write current widget values to Settings using canonical
        SafetyLimits field names."""
        s = self._settings
        if s is None:
            logger.warning("StageHardwarePanel: no Settings instance to apply to")
            return
        s.set("safety_limits.enabled", self.chk_safety_enabled.isChecked())
        s.set("safety_limits.xy_min_x", self.spin_xy_min_x.value())
        s.set("safety_limits.xy_max_x", self.spin_xy_max_x.value())
        s.set("safety_limits.xy_min_y", self.spin_xy_min_y.value())
        s.set("safety_limits.xy_max_y", self.spin_xy_max_y.value())
        s.set("safety_limits.z_min", self.spin_z_min.value())
        s.set("safety_limits.z_max", self.spin_z_max.value())
        for pid in ("P1", "P2", "P3"):
            s.set(f"safety_limits.{pid.lower()}_min", self.spin_p_mins[pid].value())
            s.set(f"safety_limits.{pid.lower()}_max", self.spin_p_maxs[pid].value())
        s.set("safety_limits.max_xy_speed", self.spin_max_xy_speed.value())
        s.set("safety_limits.max_z_feedrate", self.spin_max_z_feed.value())
        s.set("safety_limits.max_pump_feedrate", self.spin_max_pump_feed.value())

        s.set("zp_stage.max_feedrate", self.spin_zp_max_feed.value())
        s.set("zp_stage.retract_feedrate", self.spin_zp_retract_feed.value())
        s.set("zp_stage.insert_feedrate", self.spin_zp_insert_feed.value())
        s.set("zp_stage.jog_feedrate", self.spin_zp_jog_feed.value())
        s.set("zp_stage.auto_save_position", self.chk_zp_autosave.isChecked())

        s.set("axis_flip.z", self.chk_flip_z.isChecked())
        s.set("axis_flip.p1", self.chk_flip_p1.isChecked())
        s.set("axis_flip.p2", self.chk_flip_p2.isChecked())
        s.set("axis_flip.p3", self.chk_flip_p3.isChecked())

        # v7.4.2: Axis mapping — collect from combos
        if hasattr(self, 'cmb_axis_map'):
            new_map = {logical: cmb.currentData()
                       for logical, cmb in self.cmb_axis_map.items()
                       if cmb.currentData()}
            s.set("device_profile.axis_map", new_map)
            # Push live if the controller has a zp_stage
            if (self._controller is not None
                    and self._controller.zp_stage is not None):
                self._controller.zp_stage.set_axis_map(new_map)
            elif self._controller is not None:
                # Cache for next connect
                self._controller.apply_device_settings(axis_map=new_map)

        s.save()
        self.settings_applied.emit()
        logger.info("Stage hardware settings applied")

    def _reset_defaults(self):
        """v7.4.1: Reset to canonical defaults (matches Standard.json profile)."""
        self.chk_safety_enabled.setChecked(True)
        self.spin_xy_min_x.setValue(-130000.0)
        self.spin_xy_max_x.setValue(130000.0)
        self.spin_xy_min_y.setValue(-85000.0)
        self.spin_xy_max_y.setValue(85000.0)
        self.spin_z_min.setValue(-10.0)
        self.spin_z_max.setValue(50.0)
        for pid in ("P1", "P2", "P3"):
            self.spin_p_mins[pid].setValue(-50.0)
            self.spin_p_maxs[pid].setValue(50.0)
        self.spin_max_xy_speed.setValue(10000.0)
        self.spin_max_z_feed.setValue(500.0)
        self.spin_max_pump_feed.setValue(200.0)
        self.spin_zp_max_feed.setValue(3000.0)
        self.spin_zp_retract_feed.setValue(1500.0)
        self.spin_zp_insert_feed.setValue(600.0)
        self.spin_zp_jog_feed.setValue(900.0)
        self.chk_zp_autosave.setChecked(False)
        self.chk_flip_z.setChecked(False)
        self.chk_flip_p1.setChecked(False)
        self.chk_flip_p2.setChecked(False)
        self.chk_flip_p3.setChecked(False)

    # ── v7.4.1: Device profile actions ───────────────────────────

    def _refresh_profile_list(self):
        """Re-scan DEVICES_DIR and rebuild the combo."""
        if not hasattr(self, 'cmb_profile'):
            return
        prev_text = self.cmb_profile.currentText() if self.cmb_profile.count() else None
        self.cmb_profile.blockSignals(True)
        self.cmb_profile.clear()
        self._profile_paths: dict[str, str] = {}
        for name, path in list_profiles():
            self.cmb_profile.addItem(name)
            self._profile_paths[name] = str(path)
        # Restore selection if name still exists
        if prev_text:
            idx = self.cmb_profile.findText(prev_text)
            if idx >= 0:
                self.cmb_profile.setCurrentIndex(idx)
        self.cmb_profile.blockSignals(False)

    def _on_profile_picked(self, _idx: int):
        """Just track selection; doesn't auto-load (avoids accidental overwrites)."""
        name = self.cmb_profile.currentText()
        if name:
            self.lbl_profile_status.setText(
                f"Profile selected: {name} — click Load to apply.")

    def _load_selected_profile(self):
        """Load the selected profile's values into the UI + Settings."""
        from pathlib import Path
        name = self.cmb_profile.currentText()
        if not name or name not in self._profile_paths:
            return
        path = Path(self._profile_paths[name])
        try:
            profile = DeviceProfile.load(path)
        except Exception as e:
            QMessageBox.warning(self, "Load Device Profile",
                                f"Failed to load {name}: {e}")
            return
        if self._settings is not None:
            profile.apply_to_settings(self._settings)
            self._settings.set("device_profile.active", profile.profile_name)
            self._settings.save()
        # Refresh widget values from the (now updated) settings
        self._load_from_settings()
        self.lbl_profile_status.setText(
            f"Loaded profile: {profile.profile_name}")
        self.settings_applied.emit()
        logger.info(f"Loaded device profile: {profile.profile_name}")

    def _save_to_selected_profile(self):
        """Overwrite the selected profile with current widget values."""
        from pathlib import Path
        name = self.cmb_profile.currentText()
        if not name or name not in self._profile_paths:
            QMessageBox.information(self, "Save Device Profile",
                                    "No profile selected. Use Save As to create one.")
            return
        # First make sure the live settings reflect the current widgets
        self._apply()
        if self._settings is None:
            return
        profile = DeviceProfile.from_settings(self._settings, name=name)
        try:
            profile.save(Path(self._profile_paths[name]))
        except Exception as e:
            QMessageBox.warning(self, "Save Device Profile",
                                f"Failed to save {name}: {e}")
            return
        self._settings.set("device_profile.active", name)
        self._settings.save()
        self.lbl_profile_status.setText(f"Saved profile: {name}")
        logger.info(f"Saved device profile: {name}")

    def _save_as_new_profile(self):
        """Prompt for a name and save a new profile."""
        name, ok = QInputDialog.getText(
            self, "Save Device Profile As",
            "Profile name:")
        if not ok or not name.strip():
            return
        name = name.strip()
        self._apply()  # Snapshot current widget values to Settings
        if self._settings is None:
            return
        profile = DeviceProfile.from_settings(self._settings, name=name)
        try:
            path = profile.save()  # Default location in DEVICES_DIR
        except Exception as e:
            QMessageBox.warning(self, "Save Device Profile",
                                f"Failed to save {name}: {e}")
            return
        self._settings.set("device_profile.active", name)
        self._settings.save()
        self._refresh_profile_list()
        idx = self.cmb_profile.findText(name)
        if idx >= 0:
            self.cmb_profile.setCurrentIndex(idx)
        self.lbl_profile_status.setText(f"Saved new profile: {name}")
        logger.info(f"Saved new device profile {name} → {path}")

    def _delete_selected_profile(self):
        """Confirm + delete the selected profile."""
        from pathlib import Path
        name = self.cmb_profile.currentText()
        if not name or name not in self._profile_paths:
            return
        reply = QMessageBox.question(
            self, "Delete Device Profile",
            f"Delete the device profile '{name}'?\n"
            f"This cannot be undone.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        path = Path(self._profile_paths[name])
        if delete_profile(path):
            self._refresh_profile_list()
            self.lbl_profile_status.setText(f"Deleted profile: {name}")
