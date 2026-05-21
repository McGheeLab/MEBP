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
from PySide6.QtCore import Qt, Signal, QTimer

from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.scaling import s, sf, sp, scaled_font_size
from gui.pages.hardware.device_profile import (
    DeviceProfile, list_profiles, delete_profile, DEVICES_DIR,
)
from gui.widgets.jog_button_array import JogButtonArray
from SupportClasses.ZPStage import AXIS_MAP as _DEFAULT_AXIS_MAP

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

        # v7.4.2: Connect Hardware + Axis Mapping
        outer.addWidget(self._build_connect_group())
        outer.addWidget(self._build_axis_mapping_group())
        # v7.4.2 hotfix: motion calibration split into per-stage sections
        outer.addWidget(self._build_xy_cal_group())
        outer.addWidget(self._build_steps_cal_group())

        # v7.4.2 hotfix: Safety + per-axis jog + full jog pad combined
        # into one workspace. _build_safety_group + _build_jog_limits_group
        # are now legacy and only kept around as fallbacks.
        outer.addWidget(self._build_setup_jog_safety_group())
        outer.addWidget(self._build_zp_feedrates_group())
        # v7.4.2 hotfix: Axis Direction Flips section removed — direction
        # inversion now lives exclusively in steps_per_mm sign (see the
        # Stepper Calibration "Invert Axis Direction" button). axis_flip
        # is force-zeroed on every Apply to keep the two mechanisms from
        # ever fighting each other.

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

    # v7.4.2 hotfix: _build_safety_group removed.
    # Its widgets (chk_safety_enabled, spin_xy_min_x, spin_xy_max_x,
    # spin_xy_min_y, spin_xy_max_y, spin_z_min, spin_z_max,
    # spin_p_mins, spin_p_maxs, spin_max_xy_speed, spin_max_z_feed,
    # spin_max_pump_feed) are now built inside
    # _build_setup_jog_safety_group with the same attribute names so
    # _load_from_settings + _apply continue to work unchanged.


    def _build_zp_feedrates_group(self) -> QGroupBox:
        """v7.4.2 hotfix: ZP feedrates are now PERCENTAGES of Z's
        per-axis max (from Stepper Calibration → Record as Max).

        The backend still reads ``zp_stage.max_feedrate`` etc. as
        absolute mm/min — we save both the percentage (for next
        launch's UI) and the computed absolute (for the rest of the
        app). Live "= NNN mm/min" labels show the derived value
        next to each spin so the user sees what's actually being
        sent to Marlin.
        """
        grp = QGroupBox("ZP Stage Feedrates (% of Z max)")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "Each feedrate below is a percentage of the <b>Z-axis</b> "
            "maximum feedrate that you discovered via "
            "<i>Stepper Calibration → Record as Max</i>. Adjust the "
            "Z max there first; the absolute mm/min sent to Marlin is "
            "shown live next to each spin."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(info)

        form = QFormLayout()
        form.setHorizontalSpacing(s(10))

        def _pct_spin(default: float) -> QDoubleSpinBox:
            sp_w = QDoubleSpinBox()
            sp_w.setRange(0.0, 100.0)
            sp_w.setDecimals(0)
            sp_w.setSingleStep(5.0)
            sp_w.setSuffix(" %")
            sp_w.setValue(default)
            sp_w.setMinimumWidth(s(110))
            return sp_w

        def _row(label: str, spin: QDoubleSpinBox, derived_lbl: QLabel) -> QWidget:
            w = QWidget()
            h = QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(s(8))
            h.addWidget(spin)
            h.addWidget(derived_lbl, 1)
            return w

        derived_style = (
            f"color: {COLORS['text']}; font-family: monospace; "
            f"padding: {sp(2)} {sp(6)}; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {sp(4)};"
        )

        self.spin_zp_max_pct = _pct_spin(100.0)
        self.lbl_zp_max_derived = QLabel("= — mm/min")
        self.lbl_zp_max_derived.setStyleSheet(derived_style)
        self.spin_zp_max_pct.valueChanged.connect(
            self._refresh_zp_feedrate_derived)
        form.addRow("Max:",
                    _row("Max", self.spin_zp_max_pct, self.lbl_zp_max_derived))

        self.spin_zp_retract_pct = _pct_spin(50.0)
        self.lbl_zp_retract_derived = QLabel("= — mm/min")
        self.lbl_zp_retract_derived.setStyleSheet(derived_style)
        self.spin_zp_retract_pct.valueChanged.connect(
            self._refresh_zp_feedrate_derived)
        form.addRow("Retract:",
                    _row("Retract", self.spin_zp_retract_pct, self.lbl_zp_retract_derived))

        self.spin_zp_insert_pct = _pct_spin(20.0)
        self.lbl_zp_insert_derived = QLabel("= — mm/min")
        self.lbl_zp_insert_derived.setStyleSheet(derived_style)
        self.spin_zp_insert_pct.valueChanged.connect(
            self._refresh_zp_feedrate_derived)
        form.addRow("Insert:",
                    _row("Insert", self.spin_zp_insert_pct, self.lbl_zp_insert_derived))

        self.spin_zp_jog_pct = _pct_spin(40.0)
        self.lbl_zp_jog_derived = QLabel("= — mm/min")
        self.lbl_zp_jog_derived.setStyleSheet(derived_style)
        self.spin_zp_jog_pct.valueChanged.connect(
            self._refresh_zp_feedrate_derived)
        form.addRow("Jog:",
                    _row("Jog", self.spin_zp_jog_pct, self.lbl_zp_jog_derived))

        outer.addLayout(form)

        self.chk_zp_autosave = QCheckBox("Auto-save position to EEPROM")
        self.chk_zp_autosave.setChecked(False)
        outer.addWidget(self.chk_zp_autosave)

        # v7.4.2 hotfix: per-section Save — pushes derived absolute
        # mm/min to settings + sends M203 to Marlin (max feedrate)
        # and updates the controller's retract/insert feedrates.
        save_row = QHBoxLayout()
        btn_save_feedrates = QPushButton("💾 Save Feedrates")
        btn_save_feedrates.setObjectName("successBtn")
        btn_save_feedrates.setToolTip(
            "Compute absolute mm/min from the percentages, persist to "
            "settings, send M203 to Marlin for the max feedrate, and "
            "update the controller's retract/insert/jog feedrates.")
        btn_save_feedrates.clicked.connect(self._apply_zp_feedrates)
        save_row.addWidget(btn_save_feedrates)
        save_row.addStretch()
        outer.addLayout(save_row)

        # Initial derived display
        self._refresh_zp_feedrate_derived()

        return grp

    def _z_max_feedrate_mm_min(self) -> float:
        """v7.4.2 hotfix: resolve Z's per-axis max for the percentage math.

        Falls back to a sensible default if no calibration value is
        available yet.
        """
        if self._settings is not None:
            per_axis = self._settings.get("device_profile.per_axis_max_feedrate") or {}
            v = per_axis.get("Z")
            if v:
                try:
                    return float(v)
                except (TypeError, ValueError):
                    pass
        return 500.0  # Standard.json default

    def _refresh_zp_feedrate_derived(self, *_args) -> None:
        """Update the live '= NNN mm/min' labels next to each percentage spin."""
        if not hasattr(self, 'lbl_zp_max_derived'):
            return
        z_max = self._z_max_feedrate_mm_min()
        for spin, lbl in [
            (self.spin_zp_max_pct,     self.lbl_zp_max_derived),
            (self.spin_zp_retract_pct, self.lbl_zp_retract_derived),
            (self.spin_zp_insert_pct,  self.lbl_zp_insert_derived),
            (self.spin_zp_jog_pct,     self.lbl_zp_jog_derived),
        ]:
            actual = (spin.value() / 100.0) * z_max
            lbl.setText(f"= {actual:.0f} mm/min  (of Z max {z_max:.0f})")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2 hotfix: combined Jog & Safety Limits workspace
    # ════════════════════════════════════════════════════════════════
    #
    # The old "Safety Limits" group and the old "Per-Axis Jog + Record
    # Limits" group fused into one. The full jog pad from
    # JogButtonArray (XY pad + Z buttons + pumps + step selectors) is
    # embedded here so the user can run the whole find-the-mechanical-
    # envelope workflow on one screen.

    def _build_setup_jog_safety_group(self) -> QGroupBox:
        grp = QGroupBox("Jog && Safety Limits")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        # ── Bypass-mode banner ─────────────────────────────────
        bypass_banner = QLabel(
            "⚠ <b>Setup-mode jog.</b> The jog pad below bypasses "
            "soft-limit clamping and the pump-enabled check so you "
            "can move freely to discover the mechanical envelope. "
            "Use <i>Set as Min / Set as Max</i> next to each axis "
            "row to record the current position into the safety "
            "limits — then click Apply Settings at the bottom to "
            "persist."
        )
        bypass_banner.setWordWrap(True)
        bypass_banner.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: {sf(9)}pt; "
            f"padding: {sp(4)} {sp(8)};"
            f"border-left: 2px solid {COLORS['yellow']};"
            f"background: rgba(249, 226, 175, 18);"
        )
        outer.addWidget(bypass_banner)

        # ── Master safety toggle + global feedrate caps ────────
        top_row = QHBoxLayout()
        self.chk_safety_enabled = QCheckBox("Enable safety limits")
        self.chk_safety_enabled.setChecked(True)
        self.chk_safety_enabled.setToolTip(
            "Master switch for the recorded soft-limit envelope. "
            "When off, no clamping is applied anywhere in the app. "
            "(The jog pad below ignores this either way.)")
        top_row.addWidget(self.chk_safety_enabled)
        top_row.addStretch(1)
        outer.addLayout(top_row)

        # v7.4.2 hotfix: Global Feedrate Caps group REMOVED from this
        # section per user direction — per-axis max feedrates already
        # live in ZP Stage Calibration via device_profile.per_axis_max_feedrate;
        # XY max speed moves to the new XY Stage Calibration section.
        # The spin widgets themselves still exist as headless objects
        # so _load_from_settings + _apply_safety_and_zero keep working
        # without rewriting their bodies. spin_max_xy_speed is reparented
        # into the XY Calibration section below; the Z/pump caps remain
        # as silent shadows of safety_limits.* until removed in v7.4.3.

        def _cap_spin(default: float, unit: str, dec: int = 1) -> QDoubleSpinBox:
            sp_w = QDoubleSpinBox()
            sp_w.setRange(0.1, 1e6)
            sp_w.setDecimals(dec)
            sp_w.setSuffix(f" {unit}")
            sp_w.setValue(default)
            sp_w.setMinimumWidth(s(140))
            return sp_w

        self.spin_max_xy_speed = _cap_spin(10000.0, "µm/s")
        self.spin_max_z_feed = _cap_spin(500.0, "mm/min")
        self.spin_max_pump_feed = _cap_spin(200.0, "mm/min")

        # ── Full jog pad embed ──────────────────────────────────
        jog_grp = QGroupBox("Full Jog Pad (setup-mode, safety bypassed)")
        jog_grp.setStyleSheet(SECTION_TITLE_STYLE)
        jog_lay = QHBoxLayout(jog_grp)
        jog_lay.setSpacing(s(12))

        self._jog_array = JogButtonArray(compact=False, show_pumps=True)
        self._jog_array.jog_xy_requested.connect(self._on_jog_array_xy)
        self._jog_array.jog_z_requested.connect(self._on_jog_array_z)
        self._jog_array.jog_pump_requested.connect(self._on_jog_array_pump)
        self._jog_array.home_requested.connect(self._force_refresh_positions)
        jog_lay.addWidget(self._jog_array)

        # v7.4.2 hotfix: live position readout panel alongside the
        # jog pad — gives the same at-a-glance position display the
        # standalone Jog Control page has.
        side = QVBoxLayout()
        pos_title = QLabel("Live Position")
        pos_title.setStyleSheet(
            f"color: {COLORS['blue']}; font-weight: 600; "
            f"font-size: {sf(10)}pt;")
        side.addWidget(pos_title)

        pos_frame = QFrame()
        pos_frame.setStyleSheet(
            f"background: {COLORS['mantle']}; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {sp(4)}; padding: {sp(4)};")
        pos_grid = QGridLayout(pos_frame)
        pos_grid.setSpacing(s(4))
        pos_grid.setContentsMargins(s(6), s(4), s(6), s(4))

        self.lbl_jog_pos: dict[str, QLabel] = {}
        for r, (axis, unit) in enumerate([
            ("X", "µm"), ("Y", "µm"), ("Z", "mm"),
            ("P1", "mm"), ("P2", "mm"), ("P3", "mm"),
        ]):
            ax_lbl = QLabel(f"<b>{axis}</b>")
            ax_lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-size: {sf(9.5)}pt;")
            pos_grid.addWidget(ax_lbl, r, 0)

            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-family: monospace; "
                f"font-size: {sf(10)}pt;")
            val.setMinimumWidth(s(80))
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pos_grid.addWidget(val, r, 1)
            self.lbl_jog_pos[axis] = val

            unit_lbl = QLabel(f"({unit})")
            unit_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            pos_grid.addWidget(unit_lbl, r, 2)

        side.addWidget(pos_frame)

        self.btn_refresh_positions = QPushButton("↻ Refresh Positions")
        self.btn_refresh_positions.setToolTip(
            "Force a fresh position read from each connected stage.")
        self.btn_refresh_positions.clicked.connect(self._force_refresh_positions)
        side.addWidget(self.btn_refresh_positions)

        self.lbl_jog_status = QLabel("")
        self.lbl_jog_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self.lbl_jog_status.setWordWrap(True)
        side.addWidget(self.lbl_jog_status, 1)
        jog_lay.addLayout(side, 1)

        outer.addWidget(jog_grp)

        # ── Per-axis position + record + min/max spinboxes ─────
        limits_grp = QGroupBox(
            "Per-axis position && safety limits "
            "(jog to extremes → click Set Min/Max)")
        limits_grp.setStyleSheet(SECTION_TITLE_STYLE)
        limits_outer = QVBoxLayout(limits_grp)

        self.lbl_axis_pos: dict[str, QLabel] = {}
        self.spin_p_mins: dict[str, QDoubleSpinBox] = {}
        self.spin_p_maxs: dict[str, QDoubleSpinBox] = {}

        # X axis row
        limits_outer.addWidget(self._build_axis_limit_row(
            "X", "µm", -130000.0, 130000.0,
            min_attr="spin_xy_min_x", max_attr="spin_xy_max_x",
            decimals=0))
        # Y axis row
        limits_outer.addWidget(self._build_axis_limit_row(
            "Y", "µm", -85000.0, 85000.0,
            min_attr="spin_xy_min_y", max_attr="spin_xy_max_y",
            decimals=0))
        # Z axis row
        limits_outer.addWidget(self._build_axis_limit_row(
            "Z", "mm", -10.0, 50.0,
            min_attr="spin_z_min", max_attr="spin_z_max",
            decimals=3))
        # Pump rows
        for pid, lo, hi in [("P1", -50.0, 50.0), ("P2", -50.0, 50.0),
                            ("P3", -50.0, 50.0)]:
            limits_outer.addWidget(self._build_axis_limit_row(
                pid, "mm", lo, hi,
                min_attr=("spin_p_mins", pid),
                max_attr=("spin_p_maxs", pid),
                decimals=3))

        outer.addWidget(limits_grp)

        # v7.4.2 hotfix: per-section Save button — pushes safety_limits
        # to settings + the live controller in one click.
        save_row = QHBoxLayout()
        btn_save_safety = QPushButton("💾 Save Safety Limits && Zero")
        btn_save_safety.setObjectName("successBtn")
        btn_save_safety.setToolTip(
            "Persist safety_limits + zero positions to settings and "
            "push them to the live StageController.")
        btn_save_safety.clicked.connect(self._apply_safety_and_zero)
        save_row.addWidget(btn_save_safety)
        save_row.addStretch()
        outer.addLayout(save_row)

        return grp

    def _build_axis_limit_row(self, axis: str, unit: str,
                              default_min: float, default_max: float,
                              min_attr, max_attr,
                              decimals: int = 1) -> QFrame:
        """Build one axis row: position readout | Set Zero | Set Min | min spin | Set Max | max spin.

        v7.4.2 hotfix: Set Zero button added — stamps the current
        controller position as the new zero reference for this axis.
        """
        frame = QFrame()
        frame.setStyleSheet(
            f"QFrame {{ background: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {sp(4)}; padding: {sp(4)}; }}")
        h = QHBoxLayout(frame)
        h.setSpacing(s(6))
        h.setContentsMargins(s(6), s(4), s(6), s(4))

        lbl_axis = QLabel(f"<b>{axis}</b>")
        lbl_axis.setMinimumWidth(s(28))
        h.addWidget(lbl_axis)

        pos_lbl = QLabel("—")
        pos_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-family: monospace; "
            f"min-width: {sp(90)};")
        pos_lbl.setMinimumWidth(s(90))
        h.addWidget(pos_lbl)
        self.lbl_axis_pos[axis] = pos_lbl

        h.addWidget(QLabel(f"({unit})"))

        # v7.4.2 hotfix: Set Zero button
        btn_zero = QPushButton("⊙ Set Zero")
        btn_zero.setToolTip(
            f"Stamp the current {axis} position as the new zero reference "
            f"and persist it to the device settings.")
        btn_zero.setMaximumHeight(s(26))
        btn_zero.clicked.connect(lambda _c=False, a=axis: self._set_zero_axis(a))
        h.addWidget(btn_zero)

        def _spin() -> QDoubleSpinBox:
            sp_w = QDoubleSpinBox()
            sp_w.setRange(-1e6, 1e6)
            sp_w.setDecimals(decimals)
            sp_w.setSuffix(f" {unit}")
            sp_w.setMinimumWidth(s(110))
            return sp_w

        # MIN side
        btn_min = QPushButton("⤓ Set Min")
        btn_min.setToolTip(f"Stamp the current {axis} position into the min spinbox")
        btn_min.setMaximumHeight(s(26))
        btn_min.clicked.connect(lambda _c=False, a=axis: self._record_limit(a, "min"))
        h.addWidget(btn_min)
        spin_min = _spin()
        spin_min.setValue(default_min)
        h.addWidget(spin_min)

        # MAX side
        btn_max = QPushButton("⤒ Set Max")
        btn_max.setToolTip(f"Stamp the current {axis} position into the max spinbox")
        btn_max.setMaximumHeight(s(26))
        btn_max.clicked.connect(lambda _c=False, a=axis: self._record_limit(a, "max"))
        h.addWidget(btn_max)
        spin_max = _spin()
        spin_max.setValue(default_max)
        h.addWidget(spin_max)

        h.addStretch(1)

        # Store spin widgets at the canonical attribute path so _load_from_settings
        # and _apply (which use names like self.spin_xy_min_x or
        # self.spin_p_mins["P1"]) keep working unchanged.
        if isinstance(min_attr, tuple):
            container_name, key = min_attr
            getattr(self, container_name)[key] = spin_min
        else:
            setattr(self, min_attr, spin_min)
        if isinstance(max_attr, tuple):
            container_name, key = max_attr
            getattr(self, container_name)[key] = spin_max
        else:
            setattr(self, max_attr, spin_max)
        return frame

    # JogButtonArray signal adapters — pass through to the same
    # bypass-safety move methods used by the per-axis jog buttons.

    def _on_jog_array_xy(self, dx_um: float, dy_um: float) -> None:
        if self._controller is None:
            return
        try:
            self._controller.move_xy_relative_um(
                dx_um, dy_um, bypass_safety=True)
        except Exception as e:
            logger.warning(f"jog XY {(dx_um, dy_um)} failed: {e}")
        self._refresh_jog_positions()

    def _on_jog_array_z(self, dz_mm: float) -> None:
        if self._controller is None:
            return
        try:
            self._controller.move_z_relative(dz_mm, bypass_safety=True)
        except Exception as e:
            logger.warning(f"jog Z {dz_mm} failed: {e}")
        self._refresh_jog_positions()

    def _on_jog_array_pump(self, pump: str, distance: float) -> None:
        if self._controller is None:
            return
        try:
            self._controller.move_pump_relative(
                pump, distance, bypass_safety=True)
        except Exception as e:
            logger.warning(f"jog {pump} {distance} failed: {e}")
        self._refresh_jog_positions()

    # v7.4.2 hotfix: _build_axis_flip_group removed.
    # Direction is now exclusively controlled by the sign of
    # ``steps_per_mm`` (see Stepper Calibration → Invert Axis Direction).
    # Keeping the two mechanisms in sync was a source of confusion.


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
            # v7.4.2 hotfix: cache the connected port so next launch
            # can skip the rediscovery scan
            if ok and self._settings is not None:
                port = self._controller.zp_connected_port
                if port:
                    self._settings.set("zp_stage.last_port", port)
                    self._settings.save()
                    logger.info(f"ZP last_port cached: {port}")
            # v7.4.2 hotfix: auto-trigger alignment check ~1s after
            # connect succeeds so the user sees mismatches without
            # having to click the button. Debounced via QTimer so
            # Marlin has time to finish booting + responding to the
            # _setup_printer M-codes.
            if ok:
                QTimer.singleShot(
                    1000, lambda: self._check_marlin_alignment(quiet=False))
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

        # v7.4.2 hotfix: per-section Save button + status
        save_row = QHBoxLayout()
        btn_save = QPushButton("💾 Save Axis Mapping")
        btn_save.setObjectName("successBtn")
        btn_save.setToolTip(
            "Push the mapping to the live ZP stage (if connected) and "
            "persist to settings + the current device profile.")
        btn_save.clicked.connect(self._apply_axis_mapping)
        save_row.addWidget(btn_save)
        self.lbl_axis_map_status = QLabel("")
        self.lbl_axis_map_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        save_row.addWidget(self.lbl_axis_map_status, 1)
        lay.addLayout(save_row)

        return grp

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2 hotfix: XY Stage Calibration
    # ════════════════════════════════════════════════════════════════
    #
    # New section that separates XY motion calibration from the ZP
    # stage's stepper calibration. ProScan exposes set_max_speed
    # (SMS), set_acceleration (SAS), and optionally set_jerk (SCS)
    # — wrap them as user-visible spinboxes here. The units-
    # verification workflow is REPORT-ONLY: ProScan natively works
    # in µm at 1:1, so a discrepancy between commanded and measured
    # indicates a mechanical issue (belt slip, encoder mis-cal),
    # not a software setting.

    def _build_xy_cal_group(self) -> QGroupBox:
        grp = QGroupBox("XY Stage Calibration")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "Configure XY motion characteristics (ProScan). Velocity "
            "and acceleration are stored in the device profile and "
            "pushed to the controller on Save. The Units verification "
            "test (below) is report-only — ProScan moves natively in "
            "µm at 1:1, so a discrepancy means a mechanical issue, "
            "not a software fix."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(info)

        # ── Velocity / acceleration / jerk ─────────────────────
        form = QFormLayout()
        form.setHorizontalSpacing(s(10))

        self.spin_xy_velocity = QDoubleSpinBox()
        self.spin_xy_velocity.setRange(1, 100)
        self.spin_xy_velocity.setDecimals(0)
        self.spin_xy_velocity.setSuffix(" %")
        self.spin_xy_velocity.setValue(100)
        self.spin_xy_velocity.setMinimumWidth(s(120))
        self.spin_xy_velocity.setToolTip(
            "ProScan SMS — max velocity as a percentage of the "
            "controller's hardware ceiling. Default 100%.")
        form.addRow("Velocity (% max):", self.spin_xy_velocity)

        self.spin_xy_acceleration = QDoubleSpinBox()
        self.spin_xy_acceleration.setRange(1, 100)
        self.spin_xy_acceleration.setDecimals(0)
        self.spin_xy_acceleration.setValue(50)
        self.spin_xy_acceleration.setMinimumWidth(s(120))
        self.spin_xy_acceleration.setToolTip(
            "ProScan SAS — acceleration as a percentage. Default 50.")
        form.addRow("Acceleration:", self.spin_xy_acceleration)

        self.spin_xy_jerk = QDoubleSpinBox()
        self.spin_xy_jerk.setRange(0, 100)
        self.spin_xy_jerk.setDecimals(0)
        self.spin_xy_jerk.setValue(0)
        self.spin_xy_jerk.setMinimumWidth(s(120))
        self.spin_xy_jerk.setToolTip(
            "ProScan SCS — jerk. Optional; silently ignored if the "
            "controller protocol doesn't declare set_jerk. 0 = leave "
            "controller default.")
        form.addRow("Jerk (optional):", self.spin_xy_jerk)

        outer.addLayout(form)

        # ── Units verification workflow ────────────────────────
        verify_grp = QGroupBox("Units verification (report only)")
        verify_grp.setStyleSheet(SECTION_TITLE_STYLE)
        v_outer = QVBoxLayout(verify_grp)
        v_info = QLabel(
            "Pick an axis, command a distance, measure the actual "
            "displacement with a caliper, and enter the result. The "
            "comparison is shown below; no value is auto-corrected."
        )
        v_info.setWordWrap(True)
        v_info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        v_outer.addWidget(v_info)

        v_row = QHBoxLayout()
        v_row.addWidget(QLabel("Axis:"))
        self.cmb_xy_verify_axis = QComboBox()
        self.cmb_xy_verify_axis.addItem("X", "X")
        self.cmb_xy_verify_axis.addItem("Y", "Y")
        v_row.addWidget(self.cmb_xy_verify_axis)

        v_row.addWidget(QLabel("Distance:"))
        self.spin_xy_verify_commanded = QDoubleSpinBox()
        self.spin_xy_verify_commanded.setRange(1, 100000)
        self.spin_xy_verify_commanded.setDecimals(0)
        self.spin_xy_verify_commanded.setValue(1000)
        self.spin_xy_verify_commanded.setSuffix(" µm")
        v_row.addWidget(self.spin_xy_verify_commanded)

        btn_xy_move = QPushButton("Move")
        btn_xy_move.setToolTip("Send a relative move in the chosen direction.")
        btn_xy_move.clicked.connect(self._xy_verify_command_move)
        v_row.addWidget(btn_xy_move)

        v_row.addWidget(QLabel("Measured:"))
        self.spin_xy_verify_measured = QDoubleSpinBox()
        self.spin_xy_verify_measured.setRange(0, 100000)
        self.spin_xy_verify_measured.setDecimals(0)
        self.spin_xy_verify_measured.setValue(1000)
        self.spin_xy_verify_measured.setSuffix(" µm")
        v_row.addWidget(self.spin_xy_verify_measured)

        btn_xy_report = QPushButton("Compare")
        btn_xy_report.setToolTip("Show deviation between commanded and measured.")
        btn_xy_report.clicked.connect(self._xy_verify_report)
        v_row.addWidget(btn_xy_report)
        v_row.addStretch()
        v_outer.addLayout(v_row)

        self.lbl_xy_verify_result = QLabel("")
        self.lbl_xy_verify_result.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self.lbl_xy_verify_result.setWordWrap(True)
        v_outer.addWidget(self.lbl_xy_verify_result)
        outer.addWidget(verify_grp)

        # ── Save XY Calibration button ─────────────────────────
        save_row = QHBoxLayout()
        btn_save_xy = QPushButton("💾 Save XY Calibration")
        btn_save_xy.setObjectName("successBtn")
        btn_save_xy.setToolTip(
            "Send velocity/acceleration/jerk to ProScan and persist "
            "to settings + device profile.")
        btn_save_xy.clicked.connect(self._apply_xy_calibration)
        save_row.addWidget(btn_save_xy)
        self.lbl_xy_cal_status = QLabel("")
        self.lbl_xy_cal_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        save_row.addWidget(self.lbl_xy_cal_status, 1)
        outer.addLayout(save_row)
        return grp

    def _xy_verify_command_move(self) -> None:
        ctrl = self._controller
        if ctrl is None or not ctrl.is_xy_connected:
            self.lbl_xy_verify_result.setText("XY stage not connected.")
            return
        axis = self.cmb_xy_verify_axis.currentData()
        dist = float(self.spin_xy_verify_commanded.value())
        try:
            dx = dist if axis == "X" else 0.0
            dy = dist if axis == "Y" else 0.0
            ctrl.move_xy_relative_um(dx, dy, bypass_safety=True)
            self.lbl_xy_verify_result.setText(
                f"Sent {dist:.0f} µm move on {axis}. Measure with a "
                f"caliper, enter measured, then click Compare.")
        except Exception as e:
            self.lbl_xy_verify_result.setText(f"Move failed: {e}")

    def _xy_verify_report(self) -> None:
        commanded = float(self.spin_xy_verify_commanded.value())
        measured = float(self.spin_xy_verify_measured.value())
        if measured <= 0:
            self.lbl_xy_verify_result.setText("Measured must be > 0.")
            return
        dev_pct = (measured - commanded) / commanded * 100.0
        verdict = ("OK — within 1%" if abs(dev_pct) < 1.0
                   else "INVESTIGATE — likely mechanical issue")
        self.lbl_xy_verify_result.setText(
            f"Commanded {commanded:.0f} µm | Measured {measured:.0f} µm | "
            f"Deviation {dev_pct:+.1f}% — {verdict}")

    def _apply_xy_calibration(self) -> None:
        s = self._settings
        if s is None:
            return
        vel = int(self.spin_xy_velocity.value())
        acc = int(self.spin_xy_acceleration.value())
        jerk = int(self.spin_xy_jerk.value()) if self.spin_xy_jerk.value() > 0 else None
        s.set("device_profile.xy_velocity_pct", vel)
        s.set("device_profile.xy_acceleration", acc)
        s.set("device_profile.xy_jerk", jerk)
        # Push to the controller if the XY stage is live
        ctrl = self._controller
        if ctrl is not None and ctrl.xy_stage is not None:
            try:
                ctrl.xy_stage.set_velocity(vel)
            except Exception as e:
                logger.warning(f"XY set_velocity failed: {e}")
            try:
                ctrl.xy_stage.set_acceleration(acc)
            except Exception as e:
                logger.warning(f"XY set_acceleration failed: {e}")
            if jerk is not None:
                try:
                    ctrl.xy_stage.set_jerk(jerk)
                except Exception as e:
                    logger.warning(f"XY set_jerk failed: {e}")
        s.save()
        self.lbl_xy_cal_status.setText(
            f"Saved: velocity {vel}%, acceleration {acc}"
            + (f", jerk {jerk}" if jerk is not None else "")
            + " — sent to ProScan and persisted.")
        logger.info(f"XY calibration saved: vel={vel}, acc={acc}, jerk={jerk}")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2: Steps-per-mm Calibration
    # ════════════════════════════════════════════════════════════════

    def _build_steps_cal_group(self) -> QGroupBox:
        # v7.4.2 hotfix: renamed section title and added per-axis
        # acceleration spinboxes. The "Stepper Calibration & Feedrate
        # Test" name became "ZP Stage Calibration" to match the new
        # XY Stage Calibration section.
        grp = QGroupBox("ZP Stage Calibration (steps, feedrate, acceleration)")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "<b>How to calibrate:</b> Pick an axis. Use <i>Move +</i> or "
            "<i>Move −</i> to send a known-distance move at the chosen "
            "feedrate. Measure the actual physical movement and enter it. "
            "Click <i>Calculate & Send M92</i> — the new steps/mm is sent "
            "to Marlin and saved to this device profile. "
            "Negative steps/mm invert the axis direction permanently.<br>"
            "<b>To find the max feedrate:</b> raise the feedrate, send "
            "test moves, and watch for missed steps or motor stall. When "
            "you find a safe ceiling, click <i>Record as Max</i>.<br>"
            "<b>Acceleration:</b> set per-axis max acceleration (mm/s²) — "
            "sent to Marlin as M201 on Save Calibration."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(info)

        # v7.4.2 hotfix: red-state-capable label style. Stage panel
        # uses these for alignment-check mismatch highlighting.
        self._cell_style_ok = (
            f"color: {COLORS['text']}; font-family: monospace; "
            f"padding: {sp(2)} {sp(8)}; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {sp(4)};"
        )
        self._cell_style_mismatch = (
            f"color: {COLORS['red']}; font-family: monospace; "
            f"padding: {sp(2)} {sp(8)}; "
            f"border: 1px solid {COLORS['red']}; "
            f"border-radius: {sp(4)};"
            f"background-color: rgba(243, 139, 168, 28);"
        )

        # Current per-axis steps/mm grid (read-only display)
        steps_row = QHBoxLayout()
        steps_row.addWidget(QLabel("steps/mm:"))
        self.lbl_steps_grid: dict[str, QLabel] = {}
        for ax in self._LOGICAL_AXES:
            cell = QLabel(f"{ax}: —")
            cell.setStyleSheet(self._cell_style_ok)
            steps_row.addWidget(cell)
            self.lbl_steps_grid[ax] = cell
        steps_row.addStretch()
        outer.addLayout(steps_row)

        # v7.4.2: Per-axis max feedrate grid (read-only display)
        feed_row = QHBoxLayout()
        feed_row.addWidget(QLabel("max F (mm/min):"))
        self.lbl_feedrate_grid: dict[str, QLabel] = {}
        for ax in self._LOGICAL_AXES:
            cell = QLabel(f"{ax}: —")
            cell.setStyleSheet(self._cell_style_ok)
            feed_row.addWidget(cell)
            self.lbl_feedrate_grid[ax] = cell
        feed_row.addStretch()
        outer.addLayout(feed_row)

        # v7.4.2 hotfix: Per-axis max acceleration (M201)
        accel_row = QHBoxLayout()
        accel_row.addWidget(QLabel("max accel (mm/s²):"))
        self.spin_axis_accel: dict[str, QDoubleSpinBox] = {}
        default_accel = {"Z": 100.0, "P1": 1000.0, "P2": 1000.0, "P3": 1000.0}
        for ax in self._LOGICAL_AXES:
            cell = QLabel(f"{ax}:")
            cell.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            accel_row.addWidget(cell)
            sp_w = QDoubleSpinBox()
            sp_w.setRange(1.0, 100000.0)
            sp_w.setDecimals(0)
            sp_w.setSingleStep(50.0)
            sp_w.setValue(default_accel.get(ax, 1000.0))
            sp_w.setMinimumWidth(s(80))
            accel_row.addWidget(sp_w)
            self.spin_axis_accel[ax] = sp_w
        accel_row.addStretch()
        outer.addLayout(accel_row)

        # Workflow row — axis, distance, feedrate, +/- move buttons
        wf = QHBoxLayout()
        wf.addWidget(QLabel("Axis:"))
        self.cmb_cal_axis = QComboBox()
        for ax in self._LOGICAL_AXES:
            self.cmb_cal_axis.addItem(ax, ax)
        self.cmb_cal_axis.currentIndexChanged.connect(
            self._on_cal_axis_changed)
        wf.addWidget(self.cmb_cal_axis)

        wf.addWidget(QLabel("Distance:"))
        self.spin_cal_commanded = QDoubleSpinBox()
        self.spin_cal_commanded.setRange(0.001, 100.0)
        self.spin_cal_commanded.setDecimals(3)
        self.spin_cal_commanded.setValue(1.0)
        self.spin_cal_commanded.setSuffix(" mm")
        wf.addWidget(self.spin_cal_commanded)

        wf.addWidget(QLabel("Feedrate:"))
        self.spin_cal_feedrate = QDoubleSpinBox()
        self.spin_cal_feedrate.setRange(1.0, 100000.0)
        self.spin_cal_feedrate.setDecimals(0)
        self.spin_cal_feedrate.setSingleStep(50.0)
        self.spin_cal_feedrate.setValue(600.0)
        self.spin_cal_feedrate.setSuffix(" mm/min")
        wf.addWidget(self.spin_cal_feedrate)

        # +/- move buttons — flip direction without changing distance
        self.btn_cal_move_fwd = QPushButton("Move +")
        self.btn_cal_move_fwd.setToolTip("Send the distance in the positive direction")
        self.btn_cal_move_fwd.clicked.connect(
            lambda: self._cal_command_move(direction=1))
        wf.addWidget(self.btn_cal_move_fwd)

        self.btn_cal_move_rev = QPushButton("Move −")
        self.btn_cal_move_rev.setToolTip("Send the distance in the negative direction")
        self.btn_cal_move_rev.clicked.connect(
            lambda: self._cal_command_move(direction=-1))
        wf.addWidget(self.btn_cal_move_rev)

        wf.addStretch()
        outer.addLayout(wf)

        # Measure + apply + record-as-max row
        mr = QHBoxLayout()
        mr.addWidget(QLabel("Measured:"))
        self.spin_cal_measured = QDoubleSpinBox()
        self.spin_cal_measured.setRange(0.001, 100.0)
        self.spin_cal_measured.setDecimals(3)
        self.spin_cal_measured.setValue(1.0)
        self.spin_cal_measured.setSuffix(" mm")
        mr.addWidget(self.spin_cal_measured)

        self.btn_cal_apply = QPushButton("Calculate && Send M92")
        self.btn_cal_apply.setObjectName("accentBtn")
        self.btn_cal_apply.clicked.connect(self._cal_apply)
        mr.addWidget(self.btn_cal_apply)

        self.btn_cal_flip_axis = QPushButton("Invert Axis Direction")
        self.btn_cal_flip_axis.setToolTip(
            "Negate the saved steps/mm for the selected axis — flips "
            "which way 'positive' moves on Marlin without changing "
            "step count magnitude.")
        self.btn_cal_flip_axis.clicked.connect(self._cal_flip_axis)
        mr.addWidget(self.btn_cal_flip_axis)

        self.btn_cal_record_max = QPushButton("Record as Max")
        self.btn_cal_record_max.setToolTip(
            "Save the current feedrate as the experimentally-discovered "
            "max for the selected axis. Stored in the device profile "
            "as device_profile.per_axis_max_feedrate.")
        self.btn_cal_record_max.clicked.connect(self._cal_record_max_feedrate)
        mr.addWidget(self.btn_cal_record_max)

        mr.addStretch()
        outer.addLayout(mr)

        self.lbl_cal_status = QLabel("")
        self.lbl_cal_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self.lbl_cal_status.setWordWrap(True)
        outer.addWidget(self.lbl_cal_status)

        # v7.4.2 hotfix: per-section Save button. The individual buttons
        # (Calculate & Send M92, Invert Axis Direction, Record as Max)
        # already persist on click — this button is a "save everything in
        # this section now" convenience + a confirmation that values are
        # synced to Marlin.
        save_row = QHBoxLayout()
        btn_save_cal = QPushButton("💾 Save Calibration")
        btn_save_cal.setObjectName("successBtn")
        btn_save_cal.setToolTip(
            "Re-send steps_per_mm (M92), per-axis max acceleration "
            "(M201), and per_axis_max_feedrate to Marlin / device "
            "profile and persist to settings.")
        btn_save_cal.clicked.connect(self._apply_steps_cal)
        save_row.addWidget(btn_save_cal)

        # v7.4.2 hotfix: alignment check button + result label
        self.btn_check_alignment = QPushButton("🔍 Check alignment")
        self.btn_check_alignment.setToolTip(
            "Query Marlin (M503) and compare its reported steps/mm "
            "and max feedrate against this device profile. Axes that "
            "disagree turn red so you know which to re-save.")
        self.btn_check_alignment.clicked.connect(self._check_marlin_alignment)
        save_row.addWidget(self.btn_check_alignment)

        self.lbl_alignment_status = QLabel("")
        self.lbl_alignment_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        save_row.addWidget(self.lbl_alignment_status, 1)
        outer.addLayout(save_row)

        return grp

    def _on_cal_axis_changed(self, _idx: int) -> None:
        """When the cal axis changes, populate the feedrate spin with
        the recorded max for that axis (if any)."""
        if self._settings is None or not hasattr(self, 'spin_cal_feedrate'):
            return
        axis = self.cmb_cal_axis.currentData()
        per_axis = self._settings.get("device_profile.per_axis_max_feedrate") or {}
        recorded = per_axis.get(axis)
        if recorded is not None:
            self.spin_cal_feedrate.blockSignals(True)
            self.spin_cal_feedrate.setValue(float(recorded))
            self.spin_cal_feedrate.blockSignals(False)

    def _cal_command_move(self, direction: int = 1):
        """Send a known-distance relative move on the selected logical axis.

        v7.4.2: direction (+1 / -1) flips the sign; feedrate is taken from
        the spin box so users can experiment with finding the max feedrate
        for each axis. The move uses ``bypass_safety=True`` because the
        calibration workflow is initial setup — soft limits would defeat
        the point of trying to find the mechanical / motor envelope.
        """
        if self._controller is None or not self._controller.is_zp_connected:
            self.lbl_cal_status.setText("ZP stage not connected.")
            return
        axis = self.cmb_cal_axis.currentData()
        magnitude = self.spin_cal_commanded.value()
        feedrate = self.spin_cal_feedrate.value()
        distance = magnitude * (1 if direction >= 0 else -1)
        try:
            if axis == "Z":
                self._controller.move_z_relative(
                    distance, feedrate=feedrate, bypass_safety=True)
            else:
                self._controller.move_pump_relative(
                    axis, distance, feedrate=feedrate, bypass_safety=True)
            dir_label = "forward" if direction >= 0 else "reverse"
            self.lbl_cal_status.setText(
                f"Sent {dir_label} {magnitude} mm move on {axis} at "
                f"{feedrate:.0f} mm/min. Measure the actual physical "
                f"movement, then enter it on the right. If the motor "
                f"stalled or skipped steps, lower the feedrate and try again.")
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

    def _cal_flip_axis(self):
        """v7.4.2: Negate the saved steps/mm for the selected axis.

        Inverts the physical direction of the axis without changing
        the magnitude. Updates Marlin via M92 and saves to the device
        profile.
        """
        if self._controller is None or self._controller.zp_stage is None:
            self.lbl_cal_status.setText("ZP stage not connected.")
            return
        axis = self.cmb_cal_axis.currentData()
        current = self._controller.zp_stage.steps_per_mm.get(axis, 5069)
        new_steps = dict(self._controller.zp_stage.steps_per_mm)
        new_steps[axis] = -current
        self._controller.zp_stage.set_steps_per_mm(new_steps, persist=True)
        if self._settings is not None:
            self._settings.set("device_profile.steps_per_mm", new_steps)
            self._settings.save()
        self._refresh_steps_grid()
        self.lbl_cal_status.setText(
            f"{axis} direction inverted: {current} → {-current} steps/mm. "
            f"M92 sent.")

    def _cal_record_max_feedrate(self):
        """v7.4.2: Save current feedrate as the experimentally-found
        max for the selected axis. Stored in device_profile.per_axis_max_feedrate."""
        if self._settings is None:
            self.lbl_cal_status.setText("Settings not available.")
            return
        axis = self.cmb_cal_axis.currentData()
        feedrate = self.spin_cal_feedrate.value()
        per_axis = dict(
            self._settings.get("device_profile.per_axis_max_feedrate") or {})
        per_axis[axis] = feedrate
        self._settings.set("device_profile.per_axis_max_feedrate", per_axis)
        self._settings.save()
        self._refresh_max_feedrate_grid()
        self.lbl_cal_status.setText(
            f"Recorded {axis} max feedrate: {feedrate:.0f} mm/min. "
            f"You can copy this into the global Safety Limits "
            f"max_z_feedrate / max_pump_feedrate when you're confident.")

    def _refresh_max_feedrate_grid(self):
        """Refresh the per-axis max feedrate display row."""
        if not hasattr(self, 'lbl_feedrate_grid'):
            return
        per_axis = (self._settings.get("device_profile.per_axis_max_feedrate")
                    if self._settings else {}) or {}
        for ax, lbl in self.lbl_feedrate_grid.items():
            val = per_axis.get(ax, "—")
            lbl.setText(f"{ax}: {val}")

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
    #  v7.4.2 hotfix: Per-axis jog + record (now uses JogButtonArray)
    # ════════════════════════════════════════════════════════════════

    # v7.4.2 hotfix: _build_jog_limits_group removed.
    # Its content is merged into _build_setup_jog_safety_group above.

    def _force_refresh_positions(self) -> None:
        """v7.4.2 hotfix: button-driven fresh read of every connected stage."""
        ctrl = self._controller
        if ctrl is None:
            self.lbl_jog_status.setText("No controller available.")
            return
        try:
            xy = ctrl.get_xy_position(cached=False)
            zp = ctrl.get_zp_position(cached=False)
        except Exception as e:
            self.lbl_jog_status.setText(f"Read failed: {e}")
            return
        self._update_position_displays(xy, zp)
        connected = []
        if ctrl.is_xy_connected:
            connected.append("XY")
        if ctrl.is_zp_connected:
            connected.append("ZP")
        if connected:
            self.lbl_jog_status.setText(
                f"Position read OK ({', '.join(connected)} connected).")
        else:
            self.lbl_jog_status.setText(
                "No stages connected — connect XY/ZP above first.")

    # v7.4.2 hotfix: single source of truth for translating between
    # logical axis names (Z, P1, P2, P3) and the Marlin (X, Y, Z, E)
    # tuple that ``get_zp_position`` returns.
    _PHYSICAL_TO_INDEX = {"X": 0, "Y": 1, "Z": 2, "E": 3}

    def _logical_zp_value(self, zp, logical: str) -> float | None:
        """Read the ``zp`` tuple at the index for ``logical`` axis.

        ``zp`` is the tuple returned by ``StageController.get_zp_position``
        — always physical-axis order (Marlin X, Y, Z, E). The mapping
        from logical → physical comes from the live
        ``controller.zp_stage.axis_map``, falling back to the module
        default when no zp_stage exists.
        """
        axis_map = (self._controller.zp_stage.axis_map
                    if self._controller and self._controller.zp_stage
                    else _DEFAULT_AXIS_MAP)
        physical = axis_map.get(logical)
        idx = self._PHYSICAL_TO_INDEX.get(physical)
        if idx is None or zp is None or idx >= len(zp):
            return None
        v = zp[idx]
        return float(v) if v is not None else None

    def _update_position_displays(self, xy, zp) -> None:
        """v7.4.2 hotfix: update BOTH the per-axis limit-row labels
        (lbl_axis_pos) AND the live-position panel (lbl_jog_pos).

        Routes every ZP axis through the live ``axis_map`` so the
        display reflects whatever physical Marlin axis is currently
        mapped to each logical Z/P1/P2/P3.
        """
        def _set(d, key, val):
            if val is None or not hasattr(self, d) or key not in getattr(self, d):
                return
            getattr(self, d)[key].setText(val)
        if xy:
            if xy[0] is not None:
                txt = f"{xy[0]:,.1f}"
                _set("lbl_axis_pos", "X", txt)
                _set("lbl_jog_pos", "X", txt)
            if xy[1] is not None:
                txt = f"{xy[1]:,.1f}"
                _set("lbl_axis_pos", "Y", txt)
                _set("lbl_jog_pos", "Y", txt)
        if zp:
            for logical in ("Z", "P1", "P2", "P3"):
                v = self._logical_zp_value(zp, logical)
                if v is None:
                    continue
                txt = f"{v:.3f}"
                _set("lbl_axis_pos", logical, txt)
                _set("lbl_jog_pos", logical, txt)

    # v7.4.2 hotfix: _build_jog_row, _jog, and _JOG_STEPS removed.
    # Per-axis jog UI is now driven by the embedded JogButtonArray
    # via _on_jog_array_xy / _on_jog_array_z / _on_jog_array_pump
    # which still call StageController move methods with
    # bypass_safety=True.

    def _refresh_jog_positions(self) -> None:
        """Update the per-axis position readouts. Called by jog + a 500ms tick.

        v7.4.2 hotfix: routes through _update_position_displays so both
        the limit-row labels and the new live-position panel update.
        """
        ctrl = self._controller
        if ctrl is None or not hasattr(self, 'lbl_axis_pos'):
            return
        try:
            xy = ctrl.get_xy_position(cached=True)
            zp = ctrl.get_zp_position(cached=True)
        except Exception:
            return
        self._update_position_displays(xy, zp)

    # v7.4.2 hotfix: per-axis Set Zero — stamps current controller
    # position as the new zero reference for the chosen axis and
    # persists to settings.json (zero_position section).
    _ZERO_KEY = {
        "X": "x", "Y": "y", "Z": "Z",
        "P1": "P1", "P2": "P2", "P3": "P3",
    }

    def _set_zero_axis(self, axis: str) -> None:
        """v7.4.2 hotfix: zero the axis on hardware (G92 for ZP,
        ProScan set_home for XY) via ``StageController.zero_axis``,
        then snapshot ``zero_position`` to settings.json and force a
        fresh display refresh — so the live readout actually shows 0
        immediately after the click.
        """
        ctrl = self._controller
        if ctrl is None:
            self.lbl_jog_status.setText("No controller available.")
            return
        result = ctrl.zero_axis(axis)
        if not result.get("ok"):
            err = result.get("error", "unknown")
            self.lbl_jog_status.setText(f"Zero {axis} failed: {err}")
            return
        if self._settings is not None:
            self._settings.set_section("zero_position", ctrl.zero_position)
            self._settings.save()
        try:
            xy = ctrl.get_xy_position(cached=False)
            zp = ctrl.get_zp_position(cached=False)
            self._update_position_displays(xy, zp)
        except Exception as e:
            logger.warning(f"post-zero refresh failed: {e}")
        previous = result.get("previous_raw")
        unit = "µm" if axis in ("X", "Y") else "mm"
        if previous is not None:
            self.lbl_jog_status.setText(
                f"Zeroed {axis} on hardware (was {previous:.3f} {unit}). "
                f"Marlin/ProScan position counter reset to 0 and "
                f"zero_position saved to settings.")
        else:
            self.lbl_jog_status.setText(
                f"Zeroed {axis} on hardware. zero_position saved.")
        logger.info(f"Set {axis} zero on hardware (was {previous})")

    def _record_limit(self, axis: str, which: str) -> None:
        """Copy current position into the matching safety spinbox.

        v7.4.2 hotfix: uses ``cached=False`` so the position read comes
        straight from the controller at the moment of the click — the
        cached position can lag the jog by hundreds of ms otherwise.
        Also refreshes the visible position labels so the user sees
        what was just recorded.
        """
        ctrl = self._controller
        if ctrl is None:
            self.lbl_cal_status.setText("No controller available.") \
                if hasattr(self, 'lbl_cal_status') else None
            return
        try:
            xy = ctrl.get_xy_position(cached=False)
            zp = ctrl.get_zp_position(cached=False)
        except Exception as e:
            logger.warning(f"record_limit fresh read failed: {e}")
            return
        zero = ctrl.zero_position
        recorded_val: float | None = None
        if axis == "X" and xy and xy[0] is not None:
            target_spin = self.spin_xy_min_x if which == "min" else self.spin_xy_max_x
            recorded_val = float(xy[0] - zero.get("x", 0))
            target_spin.setValue(recorded_val)
        elif axis == "Y" and xy and xy[1] is not None:
            target_spin = self.spin_xy_min_y if which == "min" else self.spin_xy_max_y
            recorded_val = float(xy[1] - zero.get("y", 0))
            target_spin.setValue(recorded_val)
        elif axis == "Z":
            # v7.4.2 hotfix: route via the live axis_map so Z reads
            # from whatever physical Marlin axis is configured.
            v = self._logical_zp_value(zp, "Z")
            if v is not None:
                target_spin = self.spin_z_min if which == "min" else self.spin_z_max
                recorded_val = float(v - zero.get("Z", 0))
                target_spin.setValue(recorded_val)
        elif axis in ("P1", "P2", "P3"):
            v = self._logical_zp_value(zp, axis)
            if v is not None:
                target_dict = self.spin_p_mins if which == "min" else self.spin_p_maxs
                recorded_val = float(v - zero.get(axis, 0))
                target_dict[axis].setValue(recorded_val)
        # Update visible position labels with the fresh read
        self._update_position_displays(xy, zp)
        if recorded_val is not None and hasattr(self, 'lbl_jog_status'):
            unit = "µm" if axis in ("X", "Y") else "mm"
            self.lbl_jog_status.setText(
                f"Recorded {axis} {which}: {recorded_val:.3f} {unit} "
                f"(remember to click Apply Settings to persist)")

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
        # v7.4.2 hotfix: ZP feedrates are now percentages of Z max
        # (the user-facing UI value). The backend keys
        # ``zp_stage.max_feedrate`` etc. are derived absolute mm/min
        # for code paths that still consume them.
        #
        # Back-compat: if the user has old absolute mm/min values from
        # a previous version but no percentage stored yet, derive the
        # initial percentage from absolute / Z_max so their settings
        # don't get nuked back to defaults.
        z_max_load = self._z_max_feedrate_mm_min()

        def _load_pct(pct_key: str, abs_key: str, default_pct: float) -> float:
            v = s.get(pct_key)
            if v is not None:
                try:
                    return float(v)
                except (TypeError, ValueError):
                    pass
            abs_v = s.get(abs_key)
            if abs_v is not None and z_max_load > 0:
                try:
                    return max(0.0, min(100.0, float(abs_v) / z_max_load * 100.0))
                except (TypeError, ValueError):
                    pass
            return default_pct

        self.spin_zp_max_pct.setValue(_load_pct(
            "zp_stage.max_feedrate_pct", "zp_stage.max_feedrate", 100.0))
        self.spin_zp_retract_pct.setValue(_load_pct(
            "zp_stage.retract_feedrate_pct", "zp_stage.retract_feedrate", 50.0))
        self.spin_zp_insert_pct.setValue(_load_pct(
            "zp_stage.insert_feedrate_pct", "zp_stage.insert_feedrate", 20.0))
        self.spin_zp_jog_pct.setValue(_load_pct(
            "zp_stage.jog_feedrate_pct", "zp_stage.jog_feedrate", 40.0))
        self._refresh_zp_feedrate_derived()
        self.chk_zp_autosave.setChecked(bool(s.get("zp_stage.auto_save_position", False)))

        # Axis flips
        # v7.4.2 hotfix: axis_flip UI removed; widgets no longer exist.
        # Force-zero any stale axis_flip values on next Apply.

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

        # v7.4.2: Max-feedrate-per-axis grid + seed the feedrate spin
        # from the recorded value for the currently-selected cal axis
        if hasattr(self, 'lbl_feedrate_grid'):
            self._refresh_max_feedrate_grid()
        if hasattr(self, 'spin_cal_feedrate') and hasattr(self, 'cmb_cal_axis'):
            self._on_cal_axis_changed(0)

        # v7.4.2 hotfix: ZP per-axis acceleration + XY motion cal
        if hasattr(self, 'spin_axis_accel'):
            saved_accel = s.get("device_profile.per_axis_max_accel") or {}
            defaults = {"Z": 100.0, "P1": 1000.0, "P2": 1000.0, "P3": 1000.0}
            for ax, sp_w in self.spin_axis_accel.items():
                v = saved_accel.get(ax, defaults.get(ax, 1000.0))
                sp_w.setValue(float(v))
        if hasattr(self, 'spin_xy_velocity'):
            self.spin_xy_velocity.setValue(
                float(s.get("device_profile.xy_velocity_pct") or 100))
            self.spin_xy_acceleration.setValue(
                float(s.get("device_profile.xy_acceleration") or 50))
            saved_jerk = s.get("device_profile.xy_jerk")
            self.spin_xy_jerk.setValue(float(saved_jerk) if saved_jerk else 0)

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2 hotfix: per-section focused apply methods
    # ════════════════════════════════════════════════════════════════
    #
    # Each writes to BOTH the live StageController (so the change
    # takes effect immediately) AND settings.json (so it survives a
    # relaunch). The master ``_apply`` below still saves everything in
    # one click for users who prefer it.

    def _apply_axis_mapping(self) -> None:
        """Save the axis-mapping section: combos → settings + zp_stage."""
        if self._settings is None:
            return
        new_map = {logical: cmb.currentData()
                   for logical, cmb in self.cmb_axis_map.items()
                   if cmb.currentData()}
        self._settings.set("device_profile.axis_map", new_map)
        self._settings.save()
        if (self._controller is not None
                and self._controller.zp_stage is not None):
            self._controller.zp_stage.set_axis_map(new_map)
        elif self._controller is not None:
            self._controller.apply_device_settings(axis_map=new_map)
        if hasattr(self, 'lbl_axis_map_status'):
            self.lbl_axis_map_status.setText(
                f"Saved: {new_map}")
        logger.info(f"Axis mapping saved: {new_map}")

    def _apply_steps_cal(self) -> None:
        """Save the ZP calibration section: M92 + M201 + persist."""
        if self._settings is None:
            return
        if self._controller is not None and self._controller.zp_stage is not None:
            steps = dict(self._controller.zp_stage.steps_per_mm)
            self._settings.set("device_profile.steps_per_mm", steps)
            # Re-send M92 so Marlin matches our stored steps/mm
            self._controller.zp_stage.set_steps_per_mm(steps, persist=True)
            # v7.4.2 hotfix: also send per-axis acceleration via M201
            if hasattr(self, 'spin_axis_accel'):
                accels = {ax: sp_w.value()
                          for ax, sp_w in self.spin_axis_accel.items()}
                self._settings.set("device_profile.per_axis_max_accel", accels)
                try:
                    self._controller.zp_stage.set_axis_accelerations(
                        accels, persist=True)
                except Exception as e:
                    logger.warning(f"M201 send failed: {e}")
        else:
            # Persist UI values even if controller isn't connected yet
            if hasattr(self, 'spin_axis_accel'):
                accels = {ax: sp_w.value()
                          for ax, sp_w in self.spin_axis_accel.items()}
                self._settings.set("device_profile.per_axis_max_accel", accels)
        self._settings.save()
        self._refresh_steps_grid()
        self._refresh_max_feedrate_grid()
        # v7.4.2 hotfix: re-check alignment after a save
        try:
            self._check_marlin_alignment(quiet=True)
        except Exception:
            pass
        if hasattr(self, 'lbl_cal_status'):
            self.lbl_cal_status.setText(
                "ZP calibration saved: steps_per_mm, per_axis_max_feedrate, "
                "and per_axis_max_accel persisted; M92 + M201 sent to Marlin.")
        logger.info("ZP calibration saved (steps + accel)")

    def _check_marlin_alignment(self, quiet: bool = False) -> None:
        """v7.4.2 hotfix: query Marlin via M503 and compare its reported
        steps_per_mm + max_feedrate to this device's settings.

        Cells in the steps_per_mm + max_feedrate display grids turn
        red when Marlin's value differs from the software value by
        more than a small epsilon (1e-3). Tooltip on each mismatched
        cell shows the diff.
        """
        if (self._controller is None or self._controller.zp_stage is None
                or self._controller.simulate_zp):
            if not quiet and hasattr(self, 'lbl_alignment_status'):
                self.lbl_alignment_status.setText(
                    "Connect ZP (real hardware) to run alignment check.")
            return
        try:
            reported = self._controller.zp_stage.query_settings()
        except Exception as e:
            if hasattr(self, 'lbl_alignment_status'):
                self.lbl_alignment_status.setText(f"M503 query failed: {e}")
            return
        sw_steps = (self._controller.zp_stage.steps_per_mm
                    if self._controller.zp_stage else {})
        sw_max_feed = (self._settings.get("device_profile.per_axis_max_feedrate")
                       if self._settings else {}) or {}
        axis_map = self._controller.zp_stage.axis_map
        mismatches: list[str] = []
        # Compare steps/mm per logical axis
        for logical, physical in axis_map.items():
            cell = self.lbl_steps_grid.get(logical) \
                if hasattr(self, 'lbl_steps_grid') else None
            if cell is None:
                continue
            sw_v = sw_steps.get(logical)
            hw_v = reported.get("steps_per_mm", {}).get(physical)
            if hw_v is None or sw_v is None:
                cell.setStyleSheet(self._cell_style_ok)
                cell.setToolTip(
                    f"Could not read Marlin {physical} steps/mm — alignment unknown.")
                continue
            if abs(float(sw_v) - float(hw_v)) > 1e-3:
                cell.setStyleSheet(self._cell_style_mismatch)
                cell.setToolTip(
                    f"Mismatch: Marlin {physical}={hw_v:.2f}, "
                    f"software {logical}={sw_v:.2f}. Click "
                    f"Save Calibration to push software → Marlin, "
                    f"or edit the software side.")
                mismatches.append(f"{logical} steps/mm")
            else:
                cell.setStyleSheet(self._cell_style_ok)
                cell.setToolTip("Aligned with Marlin.")
        # Compare max feedrate per logical axis
        for logical, physical in axis_map.items():
            cell = self.lbl_feedrate_grid.get(logical) \
                if hasattr(self, 'lbl_feedrate_grid') else None
            if cell is None:
                continue
            sw_v = sw_max_feed.get(logical)
            hw_v = reported.get("max_feedrate", {}).get(physical)
            if hw_v is None or sw_v is None:
                cell.setStyleSheet(self._cell_style_ok)
                cell.setToolTip(
                    f"Could not read Marlin {physical} max feedrate — alignment unknown.")
                continue
            if abs(float(sw_v) - float(hw_v)) > 1.0:  # 1 mm/min tolerance
                cell.setStyleSheet(self._cell_style_mismatch)
                cell.setToolTip(
                    f"Mismatch: Marlin {physical} max feedrate={hw_v:.0f}, "
                    f"software {logical}={sw_v:.0f}. Click Save Calibration "
                    f"to push software → Marlin.")
                mismatches.append(f"{logical} max feedrate")
            else:
                cell.setStyleSheet(self._cell_style_ok)
                cell.setToolTip("Aligned with Marlin.")
        if hasattr(self, 'lbl_alignment_status'):
            if mismatches:
                self.lbl_alignment_status.setText(
                    f"⚠ Out of sync: {', '.join(mismatches)}. See red cells.")
                self.lbl_alignment_status.setStyleSheet(
                    f"color: {COLORS['red']}; font-size: {sf(9)}pt;")
            else:
                self.lbl_alignment_status.setText("✓ Marlin matches software.")
                self.lbl_alignment_status.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: {sf(9)}pt;")

    def _apply_safety_and_zero(self) -> None:
        """Save safety_limits + zero positions: settings + controller.

        Includes the master safety toggle, all min/max spinboxes, global
        feedrate caps, and the per-axis zero references (already in
        controller.zero_position via the Set Zero buttons; this just
        re-persists the snapshot).
        """
        s = self._settings
        if s is None:
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
        # Mirror into live controller.safety_limits if available
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "safety_limits"):
            sl = ctrl.safety_limits
            sl.enabled = self.chk_safety_enabled.isChecked()
            sl.xy_min_x = self.spin_xy_min_x.value()
            sl.xy_max_x = self.spin_xy_max_x.value()
            sl.xy_min_y = self.spin_xy_min_y.value()
            sl.xy_max_y = self.spin_xy_max_y.value()
            sl.z_min = self.spin_z_min.value()
            sl.z_max = self.spin_z_max.value()
            for pid in ("P1", "P2", "P3"):
                pid_l = pid.lower()
                if hasattr(sl, f"{pid_l}_min"):
                    setattr(sl, f"{pid_l}_min", self.spin_p_mins[pid].value())
                    setattr(sl, f"{pid_l}_max", self.spin_p_maxs[pid].value())
            sl.max_xy_speed = self.spin_max_xy_speed.value()
            sl.max_z_feedrate = self.spin_max_z_feed.value()
            sl.max_pump_feedrate = self.spin_max_pump_feed.value()
            # Persist current zero_position snapshot too
            s.set_section("zero_position", ctrl.zero_position)
        s.save()
        if hasattr(self, 'lbl_jog_status'):
            self.lbl_jog_status.setText(
                "Safety limits + zero positions saved to settings and "
                "applied to the live controller.")
        logger.info("Safety limits + zero saved")

    def _apply_zp_feedrates(self) -> None:
        """Save ZP feedrates: compute absolute mm/min from percentages,
        send M203 to Marlin, update controller retract/insert feedrates."""
        s = self._settings
        if s is None:
            return
        z_max = self._z_max_feedrate_mm_min()
        max_pct = self.spin_zp_max_pct.value()
        ret_pct = self.spin_zp_retract_pct.value()
        ins_pct = self.spin_zp_insert_pct.value()
        jog_pct = self.spin_zp_jog_pct.value()
        max_mm = max_pct / 100.0 * z_max
        ret_mm = ret_pct / 100.0 * z_max
        ins_mm = ins_pct / 100.0 * z_max
        jog_mm = jog_pct / 100.0 * z_max
        s.set("zp_stage.max_feedrate_pct", max_pct)
        s.set("zp_stage.retract_feedrate_pct", ret_pct)
        s.set("zp_stage.insert_feedrate_pct", ins_pct)
        s.set("zp_stage.jog_feedrate_pct", jog_pct)
        s.set("zp_stage.max_feedrate", max_mm)
        s.set("zp_stage.retract_feedrate", ret_mm)
        s.set("zp_stage.insert_feedrate", ins_mm)
        s.set("zp_stage.jog_feedrate", jog_mm)
        s.set("zp_stage.auto_save_position", self.chk_zp_autosave.isChecked())
        # Push to the live controller / ZP stage
        ctrl = self._controller
        if ctrl is not None:
            try:
                if ctrl.zp_stage is not None and hasattr(
                        ctrl.zp_stage, 'set_max_feedrate'):
                    ctrl.zp_stage.set_max_feedrate(max_mm)  # sends M203
                    ctrl.zp_stage.feedrate = jog_mm
                if hasattr(ctrl, '_zp_retract_feedrate'):
                    ctrl._zp_retract_feedrate = ret_mm
                if hasattr(ctrl, '_zp_insert_feedrate'):
                    ctrl._zp_insert_feedrate = ins_mm
                if hasattr(ctrl, '_zp_auto_save_position'):
                    ctrl._zp_auto_save_position = self.chk_zp_autosave.isChecked()
            except Exception as e:
                logger.warning(f"ZP feedrate push to controller failed: {e}")
        s.save()
        if hasattr(self, 'lbl_jog_status'):
            self.lbl_jog_status.setText(
                f"ZP feedrates saved: max {max_mm:.0f}, retract {ret_mm:.0f}, "
                f"insert {ins_mm:.0f}, jog {jog_mm:.0f} mm/min "
                f"(M203 sent to Marlin)")
        logger.info(
            f"ZP feedrates saved (mm/min): "
            f"max={max_mm:.0f}, retract={ret_mm:.0f}, "
            f"insert={ins_mm:.0f}, jog={jog_mm:.0f}")

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

        # v7.4.2 hotfix: ZP feedrates are stored as both percentages
        # (UI re-load) and absolute mm/min (consumed by StageController
        # etc.). Absolute = pct * Z_max_feedrate / 100.
        z_max = self._z_max_feedrate_mm_min()
        max_pct = self.spin_zp_max_pct.value()
        ret_pct = self.spin_zp_retract_pct.value()
        ins_pct = self.spin_zp_insert_pct.value()
        jog_pct = self.spin_zp_jog_pct.value()
        s.set("zp_stage.max_feedrate_pct",     max_pct)
        s.set("zp_stage.retract_feedrate_pct", ret_pct)
        s.set("zp_stage.insert_feedrate_pct",  ins_pct)
        s.set("zp_stage.jog_feedrate_pct",     jog_pct)
        s.set("zp_stage.max_feedrate",     max_pct / 100.0 * z_max)
        s.set("zp_stage.retract_feedrate", ret_pct / 100.0 * z_max)
        s.set("zp_stage.insert_feedrate",  ins_pct / 100.0 * z_max)
        s.set("zp_stage.jog_feedrate",     jog_pct / 100.0 * z_max)
        s.set("zp_stage.auto_save_position", self.chk_zp_autosave.isChecked())

        # v7.4.2 hotfix: axis_flip UI removed; force-zero any stale
        # flags so direction is exclusively controlled by steps_per_mm sign.
        s.set("axis_flip.z", False)
        s.set("axis_flip.p1", False)
        s.set("axis_flip.p2", False)
        s.set("axis_flip.p3", False)
        if self._controller is not None and hasattr(self._controller, 'set_axis_flips'):
            try:
                self._controller.set_axis_flips({
                    "z": False, "p1": False, "p2": False, "p3": False})
            except Exception:
                pass

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
        # v7.4.2 hotfix: ZP feedrates are now percentages of Z max
        self.spin_zp_max_pct.setValue(100.0)
        self.spin_zp_retract_pct.setValue(50.0)
        self.spin_zp_insert_pct.setValue(20.0)
        self.spin_zp_jog_pct.setValue(40.0)
        self._refresh_zp_feedrate_derived()
        self.chk_zp_autosave.setChecked(False)
        # v7.4.2 hotfix: axis_flip UI removed

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
