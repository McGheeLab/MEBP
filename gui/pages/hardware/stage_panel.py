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
from gui.widgets.icons import icon, icon_button, set_button_icon
from gui.widgets.jog_button_array import JogButtonArray
from gui.widgets.reorderable_sections import ReorderableSectionList
from SupportClasses.ZPStage import AXIS_MAP as _DEFAULT_AXIS_MAP
from SupportClasses.StageController import z_raw_to_display, z_display_to_raw

logger = logging.getLogger(__name__)


class _ClickableLabel(QLabel):
    """v7.4.2 hotfix: QLabel that emits ``clicked`` on left-mouse release.

    Used for the read-only display cells in ZP Stage Calibration
    (steps/mm and max feedrate grids) so the user can click any cell
    to open an edit popup instead of having to run the full
    Move + Measured + Calculate workflow.

    Hover gets a brighter border + pointer cursor so users discover
    the cells are interactive without needing a separate help string.
    """

    clicked = Signal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip("Click to edit")

    def mouseReleaseEvent(self, ev):  # noqa: N802 — Qt naming
        if ev.button() == Qt.LeftButton and self.rect().contains(ev.pos()):
            self.clicked.emit()
        super().mouseReleaseEvent(ev)


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
            self._restore_section_layout()

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

    def on_motion_tick(self):
        """v7.5.x: fast (~30 fps) display-only refresh of the per-axis position
        readouts while a jog/travel motion estimate is live."""
        if not hasattr(self, 'lbl_axis_pos'):
            return
        self._refresh_jog_positions()
        self._sync_connection_badges()

    def _sync_connection_badges(self):
        """v7.4.2: Reflect controller's live connection state in badges.

        Distinguishes "Connected" (real hardware) from "Simulated" so the
        operator can see at a glance whether the stage they're driving is
        physical or virtual.
        """
        if self._controller is None or not hasattr(self, 'badge_xy'):
            return
        xy_ok = bool(getattr(self._controller, 'is_xy_connected', False))
        zp_ok = bool(getattr(self._controller, 'is_zp_connected', False))
        xy_sim = bool(getattr(self._controller, 'simulate_xy', False)) and xy_ok
        zp_sim = bool(getattr(self._controller, 'simulate_zp', False)) and zp_ok
        if not xy_ok:
            self.badge_xy.set_status("pending", "Not connected")
        else:
            self.badge_xy.set_status(
                "ok", "Simulated" if xy_sim else "Connected")
        if not zp_ok:
            self.badge_zp.set_status("pending", "Not connected")
        else:
            self.badge_zp.set_status(
                "ok", "Simulated" if zp_sim else "Connected")
        # Xbox is tri-state (waiting / connected / reconnecting).
        if hasattr(self, 'badge_xbox'):
            xbox_st = getattr(self._controller, "xbox_status", None)
            if callable(xbox_st):
                try:
                    xbox_st = xbox_st()
                except Exception:
                    xbox_st = "disconnected"
            if xbox_st in ("connected", "alive"):
                self.badge_xbox.set_status("ok", "Connected")
            elif xbox_st == "reconnecting":
                self.badge_xbox.set_status("warn", "Reconnecting…")
            elif xbox_st == "waiting":
                self.badge_xbox.set_status("info", "Searching…")
            else:
                self.badge_xbox.set_status("pending", "Not connected")

    # ── UI ───────────────────────────────────────────────────────

    def _setup_ui(self):
        # v7.4.2 polish: roomier spacing + margins so cards breathe.
        outer = QVBoxLayout(self)
        outer.setSpacing(s(18))
        outer.setContentsMargins(s(20), s(20), s(20), s(20))

        # v7.4.2 polish: banner uses the frosted-glass panel style with
        # a mauve accent stripe, matching the look of every other card.
        banner = QLabel(
            "<b>Initial device setup.</b> Safety envelope, motor "
            "feedrates, and axis direction describe the physical "
            "machine — independent of which experiment you're "
            "running. Pick a device profile below or save your own."
        )
        banner.setWordWrap(True)
        banner.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"padding: {sp(12)} {sp(16)};"
            f"background-color: qlineargradient("
            f"  x1: 0, y1: 0, x2: 0, y2: 1,"
            f"  stop: 0 rgba(205, 214, 244, 14),"
            f"  stop: 1 rgba(203, 166, 247, 14)"
            f");"
            f"border: 1px solid rgba(166, 173, 200, 32);"
            f"border-left: 3px solid {COLORS['mauve']};"
            f"border-radius: {sp(10)};"
        )
        outer.addWidget(banner)

        # v7.5.x: every configuration section below is a collapsible,
        # user-reorderable card (ReorderableSectionList) so the operator can
        # promote the sections they reach for often and fold the rest away.
        # The order + collapsed state are persisted per-machine to settings
        # (see _persist_section_layout / _restore_section_layout). Default
        # order preserves the historical top-to-bottom layout.
        #
        # v7.4.2: Connect Hardware + Full Jog Pad + Live Position were
        # moved to the persistent HardwareControlPanel on the left edge
        # of Hardware Setup (always visible across sub-pages). The Device
        # sub-page now only carries the *configuration* sections.
        #
        # v7.4.2 hotfix: Axis Direction Flips section removed — direction
        # inversion now lives exclusively in steps_per_mm sign (see the
        # Stepper Calibration "Invert Axis Direction" button). axis_flip
        # is force-zeroed on every Apply to keep the two mechanisms from
        # ever fighting each other.
        self._section_list = ReorderableSectionList()
        _sections = [
            ("device_profile", "Device Profile",
             self._build_device_profile_group()),
            ("axis_mapping", "Axis Mapping",
             self._build_axis_mapping_group()),
            ("xy_cal", "XY Stage Calibration",
             self._build_xy_cal_group()),
            ("steps_cal", "ZP Stage Calibration",
             self._build_steps_cal_group()),
            ("jog_safety", "Jog & Safety Limits",
             self._build_setup_jog_safety_group()),
            ("override_pos", "Override / Sync Axis Position",
             self._build_override_position_group()),
            ("zp_feedrates", "ZP Stage Feedrates (% of Z max)",
             self._build_zp_feedrates_group()),
            ("recal_reminders", "Recalibration Reminders",
             self._build_recal_reminders_group()),
        ]
        for _key, _title, _widget in _sections:
            self._section_list.add_section(_key, _title, _widget)
        self._section_list.order_changed.connect(
            lambda *_a: self._persist_section_layout())
        self._section_list.collapsed_changed.connect(
            lambda *_a: self._persist_section_layout())
        outer.addWidget(self._section_list)

        # v7.4.2 polish: Apply / Reset bottom action row — right-aligned,
        # primary apply, danger reset.
        btn_row = QHBoxLayout()
        btn_row.setSpacing(s(8))
        btn_row.addStretch()
        self.btn_reset = QPushButton("Reset to Defaults")
        self.btn_reset.setObjectName("dangerBtn")
        self.btn_reset.setCursor(Qt.PointingHandCursor)
        self.btn_reset.setToolTip(
            "Discard all unsaved edits and reload the saved device profile.")
        self.btn_reset.clicked.connect(self._reset_defaults)
        btn_row.addWidget(self.btn_reset)
        self.btn_apply = icon_button(
            "Apply Settings", "save", object_name="accentBtn",
            tooltip=(
                "Apply every section in one click. Per-section buttons "
                "do the same thing scoped to that section."))
        self.btn_apply.clicked.connect(self._apply)
        btn_row.addWidget(self.btn_apply)
        outer.addLayout(btn_row)

        outer.addStretch()

    # ── v7.5.x: collapsible / reorderable section layout ─────────

    def _persist_section_layout(self) -> None:
        """Save the current section order + collapsed state to settings.

        Per-machine UI preference (which sections sit where and which are
        folded); it does not affect hardware behavior. Guarded so it is a
        no-op before settings are injected."""
        if self._settings is None or not hasattr(self, "_section_list"):
            return
        try:
            self._settings.set("device_page_layout.order",
                               self._section_list.order())
            self._settings.set("device_page_layout.collapsed",
                               self._section_list.collapsed_states())
            self._settings.save()
        except Exception as e:
            logger.warning(f"persist device-page section layout failed: {e}")

    def _restore_section_layout(self) -> None:
        """Apply the persisted section order + collapsed state (if any)."""
        if self._settings is None or not hasattr(self, "_section_list"):
            return
        try:
            order = self._settings.get("device_page_layout.order")
            if isinstance(order, list) and order:
                self._section_list.set_order(order)
            collapsed = self._settings.get("device_page_layout.collapsed")
            if isinstance(collapsed, dict) and collapsed:
                self._section_list.apply_collapsed_states(collapsed)
        except Exception as e:
            logger.warning(f"restore device-page section layout failed: {e}")

    # ── v7.4.1: Device profile picker ────────────────────────────

    def _build_device_profile_group(self) -> QGroupBox:
        grp = QGroupBox("Device Profile")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        # v7.4.2 polish: roomy rhythm inside the section.
        lay.setSpacing(s(10))

        # Row 1: profile picker + refresh icon
        row = QHBoxLayout()
        row.setSpacing(s(8))
        lbl_profile = QLabel("Profile")
        lbl_profile.setStyleSheet(f"font-weight: 600; color: {COLORS['text']};")
        row.addWidget(lbl_profile)
        self.cmb_profile = QComboBox()
        self.cmb_profile.setMinimumWidth(s(240))
        self.cmb_profile.currentIndexChanged.connect(self._on_profile_picked)
        row.addWidget(self.cmb_profile, 1)

        self.btn_refresh_profiles = icon_button(
            "", "refresh", tooltip="Refresh profile list")
        self.btn_refresh_profiles.setFixedWidth(s(36))
        self.btn_refresh_profiles.clicked.connect(self._refresh_profile_list)
        row.addWidget(self.btn_refresh_profiles)
        lay.addLayout(row)

        # Row 2: action buttons — Load left, Save / Save As primary,
        # Delete on the right (semantic grouping).
        action_row = QHBoxLayout()
        action_row.setSpacing(s(8))
        self.btn_load_profile = icon_button(
            "Load", "folder-open",
            tooltip="Apply the selected profile to every section below.")
        self.btn_load_profile.clicked.connect(self._load_selected_profile)
        action_row.addWidget(self.btn_load_profile)

        self.btn_save_profile = icon_button(
            "Save", "save", object_name="accentBtn",
            tooltip="Save current settings back to the selected profile")
        self.btn_save_profile.clicked.connect(self._save_to_selected_profile)
        action_row.addWidget(self.btn_save_profile)

        self.btn_save_as_profile = QPushButton("Save As…")
        self.btn_save_as_profile.setCursor(Qt.PointingHandCursor)
        self.btn_save_as_profile.setToolTip(
            "Save current settings as a new device profile.")
        self.btn_save_as_profile.clicked.connect(self._save_as_new_profile)
        action_row.addWidget(self.btn_save_as_profile)

        action_row.addStretch()

        self.btn_delete_profile = QPushButton("Delete")
        self.btn_delete_profile.setObjectName("dangerBtn")
        self.btn_delete_profile.setCursor(Qt.PointingHandCursor)
        self.btn_delete_profile.setToolTip(
            "Delete the selected profile from disk.")
        self.btn_delete_profile.clicked.connect(self._delete_selected_profile)
        action_row.addWidget(self.btn_delete_profile)
        lay.addLayout(action_row)

        # Status / footer
        self.lbl_profile_status = QLabel(
            f"Profiles directory: {DEVICES_DIR}")
        self.lbl_profile_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"padding-top: {sp(4)};")
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
            f"color: {COLORS['subtext0']}; ")
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
        btn_save_feedrates = icon_button(
            "Save Feedrates", "save", object_name="successBtn",
            tooltip=(
                "Compute absolute mm/min from the percentages, persist to "
                "settings, send M203 to Marlin for the max feedrate, and "
                "update the controller's retract/insert/jog feedrates."))
        btn_save_feedrates.clicked.connect(self._apply_zp_feedrates)
        save_row.addWidget(btn_save_feedrates)
        save_row.addStretch()
        outer.addLayout(save_row)

        # Initial derived display
        self._refresh_zp_feedrate_derived()

        return grp

    def _z_max_feedrate_mm_min(self) -> float:
        """Resolve Z's per-axis max for the percentage math.

        v7.5.x: prefer the controller's SINGLE common resolver
        (``get_max_z_feedrate_mm_min``) so this editor surface reflects the same
        Z max every page reads; fall back to the settings value (this panel is
        also where that value is edited) then a sensible default.
        """
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "get_max_z_feedrate_mm_min"):
            try:
                v = float(ctrl.get_max_z_feedrate_mm_min())
                if v > 0:
                    return v
            except Exception:
                pass
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

    def _build_recal_reminders_group(self) -> QGroupBox:
        """v7.5.x: recalibration-reminder thresholds (M distance / H hours) plus
        a "Calibration status…" button. These drive the usability pop-up that
        reports when XY / Z / P were last calibrated and warns when a threshold
        is crossed. Owned by the per-machine ``CalibrationStatusStore`` (not
        ``HardwareConfig``) since only the pop-up consumes them."""
        grp = QGroupBox("Recalibration Reminders")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "Warn me to recalibrate X/Y when the stage has traveled more than "
            "<b>M</b> since the last XY calibration, or to update when it has "
            "been more than <b>H</b> since a calibration. Set either to 0 to "
            "turn that warning off.")
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']};")
        outer.addWidget(info)

        form = QFormLayout()
        form.setHorizontalSpacing(s(10))

        self.spin_recal_travel_mm = QDoubleSpinBox()
        self.spin_recal_travel_mm.setRange(0.0, 1_000_000.0)
        self.spin_recal_travel_mm.setDecimals(0)
        self.spin_recal_travel_mm.setSingleStep(100.0)
        self.spin_recal_travel_mm.setSuffix(" mm")
        self.spin_recal_travel_mm.setMinimumWidth(s(130))
        self.spin_recal_travel_mm.setToolTip(
            "M — XY travel since the last XY calibration that triggers a "
            "“recalibrate X/Y” warning. 0 = off.")
        form.addRow("XY travel limit (M):", self.spin_recal_travel_mm)

        self.spin_recal_hours = QDoubleSpinBox()
        self.spin_recal_hours.setRange(0.0, 100_000.0)
        self.spin_recal_hours.setDecimals(1)
        self.spin_recal_hours.setSingleStep(12.0)
        self.spin_recal_hours.setSuffix(" h")
        self.spin_recal_hours.setMinimumWidth(s(130))
        self.spin_recal_hours.setToolTip(
            "H — hours since a calibration that triggers an “update” "
            "warning. 0 = off.")
        form.addRow("Time limit (H):", self.spin_recal_hours)

        outer.addLayout(form)

        # Seed the spinboxes from the store's persisted thresholds.
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            m_mm, h_hours = get_store().get_thresholds()
            self.spin_recal_travel_mm.setValue(float(m_mm))
            self.spin_recal_hours.setValue(float(h_hours))
        except Exception:
            pass

        row = QHBoxLayout()
        btn_save = icon_button(
            "Save Reminders", "save", object_name="successBtn",
            tooltip="Persist the M / H thresholds used by the pop-up.")
        btn_save.clicked.connect(self._apply_recal_reminders)
        row.addWidget(btn_save)
        btn_status = QPushButton("Calibration status…")
        btn_status.setCursor(Qt.PointingHandCursor)
        btn_status.setToolTip(
            "Show when XY, Z and the pump were last calibrated, with any "
            "recalibration warnings.")
        btn_status.clicked.connect(self._show_calibration_status)
        row.addWidget(btn_status)
        row.addStretch()
        outer.addLayout(row)

        return grp

    def _apply_recal_reminders(self) -> None:
        """Persist the M / H recalibration thresholds to the store."""
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            get_store().set_thresholds(
                m_mm=float(self.spin_recal_travel_mm.value()),
                h_hours=float(self.spin_recal_hours.value()))
        except Exception as e:
            logger.warning(f"Save recalibration reminders failed: {e}")

    def _show_calibration_status(self) -> None:
        """Open the calibration-status pop-up on demand (always shows)."""
        try:
            from gui.dialogs.calibration_status_dialog import (
                show_calibration_status)
            show_calibration_status(self, force=True)
        except Exception as e:
            logger.warning(f"Calibration status dialog failed: {e}")

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
            f"color: {COLORS['yellow']}; "
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

        # v7.4.2: the Full Jog Pad + Live Position panel that used to
        # sit here have moved to the persistent HardwareControlPanel on
        # the left edge of Hardware Setup. They were embedded only on
        # the Device sub-page; now they're visible across all sub-pages.
        #
        # Headless placeholders below keep _update_position_displays /
        # _force_refresh_positions / _set_zero_axis happy without
        # rewriting their bodies — they reference lbl_jog_pos +
        # lbl_jog_status + btn_refresh_positions.
        self.lbl_jog_pos: dict[str, QLabel] = {}
        for axis in ("X", "Y", "Z", "P1", "P2", "P3"):
            self.lbl_jog_pos[axis] = QLabel("—")
            self.lbl_jog_pos[axis].setVisible(False)
        self.btn_refresh_positions = QPushButton()
        self.btn_refresh_positions.setVisible(False)
        self.lbl_jog_status = QLabel("")
        self.lbl_jog_status.setVisible(False)

        # ── v7.5.x: Z Axis Setup (datum + direction + limits) ──
        outer.addWidget(self._build_z_axis_setup_group())

        # ── v7.5.x: per-pump Plunger Setup (twin of the Z Axis Setup) ──
        # One block per pump, ordered right under the Z block (Z, P1, P2, P3).
        # Capturing the empty/full extremes derives the direction AND updates +
        # saves the pump soft-limit (calibration) extents.
        self._pump_setup_dispensed_raw: dict[str, float | None] = {}
        self._pump_setup_dispensed_btns: dict[str, QPushButton] = {}
        self._pump_setup_aspirated_btns: dict[str, QPushButton] = {}
        self._pump_setup_status_lbls: dict[str, QLabel] = {}
        for pid in ("P1", "P2", "P3"):
            outer.addWidget(self._build_pump_plunger_setup_block(pid))

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
        btn_save_safety = icon_button(
            "Save Safety Limits && Zero", "save", object_name="successBtn",
            tooltip=("Persist safety_limits + zero positions to settings "
                     "and push them to the live StageController."))
        btn_save_safety.clicked.connect(self._apply_safety_and_zero)
        save_row.addWidget(btn_save_safety)
        save_row.addStretch()
        outer.addLayout(save_row)

        return grp

    # ════════════════════════════════════════════════════════════════
    #  v7.5.x: Override / sync axis position (power-cycle recovery)
    # ════════════════════════════════════════════════════════════════
    #
    # Marlin (ZP) has no absolute encoder: after the board loses power it
    # powers up reporting 0, so the position readout is wrong and a
    # Refresh just re-reads the wrong value. This card lets the operator
    # tell the firmware where each ZP axis physically is (sends a G92 to
    # rebase the counter — no motion). The established zero reference is
    # preserved, so the value entered is in the same zero-referenced mm
    # frame as the readout and the safety limits.

    def _build_override_position_group(self) -> QGroupBox:
        grp = QGroupBox("Override / Sync Axis Position")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)

        info = QLabel(
            "After a ZP-board power cycle, Marlin powers up at <b>0</b> "
            "(it has no absolute encoder), so the position is wrong and "
            "<i>Refresh</i> just re-reads the wrong value. Enter where "
            "each axis physically is and click <b>Set Position</b> — this "
            "rebases the firmware counter (G92) <b>without moving</b>. "
            "Values are zero-referenced <b>mm</b> — the same frame as the "
            "safety limits and the Jog page position (not the raw counter "
            "shown in the limit rows above)."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; padding: {sp(2)} {sp(4)};")
        outer.addWidget(info)

        self.lbl_override_pos: dict[str, QLabel] = {}
        self.spin_override: dict[str, QDoubleSpinBox] = {}

        for axis in ("Z", "P1", "P2", "P3"):
            outer.addWidget(self._build_override_row(axis))

        self.lbl_override_status = QLabel("")
        self.lbl_override_status.setWordWrap(True)
        self.lbl_override_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; padding: {sp(2)} {sp(4)};")
        outer.addWidget(self.lbl_override_status)

        return grp

    def _build_override_row(self, axis: str) -> QFrame:
        """One row: axis | current (zero-ref) readout | new-value spin | Set."""
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

        h.addWidget(QLabel("now:"))
        pos_lbl = QLabel("—")
        pos_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-family: monospace; "
            f"min-width: {sp(90)};")
        pos_lbl.setMinimumWidth(s(90))
        h.addWidget(pos_lbl)
        self.lbl_override_pos[axis] = pos_lbl
        h.addWidget(QLabel("mm"))

        h.addWidget(QLabel("→ set to:"))
        spin = QDoubleSpinBox()
        spin.setRange(-1e6, 1e6)
        spin.setDecimals(3)
        spin.setSuffix(" mm")
        spin.setMinimumWidth(s(110))
        h.addWidget(spin)
        self.spin_override[axis] = spin

        btn = QPushButton("⟳ Set Position")
        btn.setToolTip(
            f"Tell the firmware that {axis} is physically at the entered "
            f"value (sends a G92 — no motion). Use after a power cycle to "
            f"re-sync the position counter.")
        btn.setMaximumHeight(s(26))
        btn.clicked.connect(lambda _c=False, a=axis: self._override_axis_position(a))
        h.addWidget(btn)

        h.addStretch(1)
        return frame

    def _override_axis_position(self, axis: str) -> None:
        """v7.5.x: send the entered value to the firmware via G92 so the
        axis readout matches reality after a power cycle.

        Delegates to ``StageController.override_zp_position`` (which
        preserves the zero reference and rebases only the firmware
        counter), then forces a fresh read so the readouts update.
        """
        ctrl = self._controller
        if ctrl is None:
            self.lbl_override_status.setText("No controller available.")
            return
        spin = self.spin_override.get(axis)
        if spin is None:
            return
        value = float(spin.value())
        result = ctrl.override_zp_position(axis, value)
        if not result.get("ok"):
            err = result.get("error", "unknown")
            self.lbl_override_status.setText(
                f"⚠ Override {axis} failed: {err}")
            return
        try:
            xy = ctrl.get_xy_position(cached=False)
            zp = ctrl.get_zp_position(cached=False)
            self._update_position_displays(xy, zp)
        except Exception as e:
            logger.warning(f"post-override refresh failed: {e}")
        prev = result.get("previous_raw")
        if prev is not None:
            self.lbl_override_status.setText(
                f"✓ Set {axis} position to {value:.3f} mm "
                f"(firmware counter was {prev:.3f} mm).")
        else:
            self.lbl_override_status.setText(
                f"✓ Set {axis} position to {value:.3f} mm.")
        logger.info(f"Override {axis} position to {value} (was raw {prev})")

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
        # v7.5.x: dz_mm is a HEIGHT-frame delta (+ = up); route through
        # move_z_user_relative so the device-page jog follows z_up_sign too.
        try:
            self._controller.move_z_user_relative(dz_mm, bypass_safety=True)
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
        """Connect XY / ZP / Xbox with live status badges.

        v7.4.2 update: each stage row has separate ``Connect`` (real
        hardware) and ``Simulate`` buttons. The Simulate button opens a
        software simulator for that one device — no app restart needed.
        """
        from gui.widgets.components import StatusBadge

        grp = QGroupBox("Connect Hardware")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        grid = QGridLayout(grp)
        # v7.4.2 polish: roomier grid + each row reads cleanly.
        grid.setHorizontalSpacing(s(10))
        grid.setVerticalSpacing(s(10))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 0)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)
        grid.setColumnStretch(4, 1)

        def _stage_label(text: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(f"font-weight: 600; color: {COLORS['text']};")
            return lbl

        row = 0
        grid.addWidget(_stage_label("XY stage"), row, 0)
        self.btn_connect_xy = QPushButton("🔌  Connect")
        self.btn_connect_xy.setObjectName("successBtn")
        self.btn_connect_xy.setCursor(Qt.PointingHandCursor)
        self.btn_connect_xy.setToolTip(
            "Open the real Prior ProScan XY stage over serial.")
        self.btn_connect_xy.clicked.connect(self._connect_xy)
        grid.addWidget(self.btn_connect_xy, row, 1)
        self.btn_simulate_xy = QPushButton("🧪  Simulate")
        self.btn_simulate_xy.setObjectName("accentBtn")
        self.btn_simulate_xy.setCursor(Qt.PointingHandCursor)
        self.btn_simulate_xy.setToolTip(
            "Use the XY stage simulator instead of real hardware.")
        self.btn_simulate_xy.clicked.connect(self._simulate_xy)
        grid.addWidget(self.btn_simulate_xy, row, 2)
        self.btn_disconnect_xy = QPushButton("Disconnect")
        self.btn_disconnect_xy.setObjectName("dangerBtn")
        self.btn_disconnect_xy.setCursor(Qt.PointingHandCursor)
        self.btn_disconnect_xy.clicked.connect(self._disconnect_xy)
        grid.addWidget(self.btn_disconnect_xy, row, 3)
        self.badge_xy = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_xy, row, 4)

        row += 1
        grid.addWidget(_stage_label("Z + Pumps"), row, 0)
        self.btn_connect_zp = QPushButton("🔌  Connect")
        self.btn_connect_zp.setObjectName("successBtn")
        self.btn_connect_zp.setCursor(Qt.PointingHandCursor)
        self.btn_connect_zp.setToolTip(
            "Open the real Marlin Z + pumps controller over serial.")
        self.btn_connect_zp.clicked.connect(self._connect_zp)
        grid.addWidget(self.btn_connect_zp, row, 1)
        self.btn_simulate_zp = QPushButton("🧪  Simulate")
        self.btn_simulate_zp.setObjectName("accentBtn")
        self.btn_simulate_zp.setCursor(Qt.PointingHandCursor)
        self.btn_simulate_zp.setToolTip(
            "Use the Z + pumps simulator instead of real hardware.")
        self.btn_simulate_zp.clicked.connect(self._simulate_zp)
        grid.addWidget(self.btn_simulate_zp, row, 2)
        self.btn_disconnect_zp = QPushButton("Disconnect")
        self.btn_disconnect_zp.setObjectName("dangerBtn")
        self.btn_disconnect_zp.setCursor(Qt.PointingHandCursor)
        self.btn_disconnect_zp.clicked.connect(self._disconnect_zp)
        grid.addWidget(self.btn_disconnect_zp, row, 3)
        self.badge_zp = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_zp, row, 4)

        # v7.4.2: Xbox controller — moved here from the (now-removed)
        # Dashboard page. Reads optional connect parameters from settings.
        # Xbox has no simulator, so its row spans the Simulate column.
        row += 1
        grid.addWidget(_stage_label("Xbox"), row, 0)
        self.btn_connect_xbox = QPushButton("🎮  Connect")
        self.btn_connect_xbox.setObjectName("successBtn")
        self.btn_connect_xbox.setCursor(Qt.PointingHandCursor)
        self.btn_connect_xbox.setToolTip(
            "Connect an Xbox controller for continuous jogging.")
        self.btn_connect_xbox.clicked.connect(self._connect_xbox)
        grid.addWidget(self.btn_connect_xbox, row, 1, 1, 2)
        self.btn_disconnect_xbox = QPushButton("Disconnect")
        self.btn_disconnect_xbox.setObjectName("dangerBtn")
        self.btn_disconnect_xbox.setCursor(Qt.PointingHandCursor)
        self.btn_disconnect_xbox.clicked.connect(self._disconnect_xbox)
        grid.addWidget(self.btn_disconnect_xbox, row, 3)
        self.badge_xbox = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_xbox, row, 4)

        return grp

    def _connect_xy(self):
        """v7.4.2: open the real XY hardware."""
        self._open_xy(simulate=False)

    def _simulate_xy(self):
        """v7.4.2: open the XY simulator (no hardware required)."""
        self._open_xy(simulate=True)

    def _open_xy(self, *, simulate: bool):
        if self._controller is None:
            return
        if self._controller.is_xy_connected:
            # Tear down whatever is currently attached so we can switch
            # modes without leaving a stale stage object behind.
            try:
                self._controller.disconnect_xy()
            except Exception as e:
                logger.warning(f"disconnect_xy before reopen failed: {e}")
        mode_text = "Starting simulator…" if simulate else "Connecting…"
        self.badge_xy.set_status("info", mode_text)
        try:
            self._controller.connect_xy(simulate=simulate)
            ok = bool(self._controller.is_xy_connected)
            if ok:
                self.badge_xy.set_status(
                    "ok", "Simulated" if simulate else "Connected")
            else:
                self.badge_xy.set_status("err", "Failed")
            # v7.18.1: cache the winning protocol + port + baud so the next
            # connect skips the full sweep. Real HW only.
            if ok and not simulate and self._settings is not None:
                hint = self._controller.xy_connection_hint
                if hint:
                    self._settings.set("xy_stage.last_good", hint)
                    self._settings.save()
                    logger.info("XY last_good cached: %s", hint)
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
        """v7.4.2: open the real ZP hardware."""
        self._open_zp(simulate=False)

    def _simulate_zp(self):
        """v7.4.2: open the ZP simulator (no hardware required)."""
        self._open_zp(simulate=True)

    def _open_zp(self, *, simulate: bool):
        if self._controller is None:
            return
        if self._controller.is_zp_connected:
            try:
                self._controller.disconnect_zp()
            except Exception as e:
                logger.warning(f"disconnect_zp before reopen failed: {e}")
        mode_text = "Starting simulator…" if simulate else "Connecting…"
        self.badge_zp.set_status("info", mode_text)
        try:
            self._controller.connect_zp(simulate=simulate)
            ok = bool(self._controller.is_zp_connected)
            if ok:
                self.badge_zp.set_status(
                    "ok", "Simulated" if simulate else "Connected")
            else:
                self.badge_zp.set_status("err", "Failed")
            # v7.4.2 hotfix: cache the connected port so next launch
            # can skip the rediscovery scan. Only meaningful for real HW.
            if ok and not simulate and self._settings is not None:
                port = self._controller.zp_connected_port
                if port:
                    self._settings.set("zp_stage.last_port", port)
                    self._settings.save()
                    logger.info(f"ZP last_port cached: {port}")
            # v7.4.2 hotfix: auto-trigger alignment check ~1s after
            # connect succeeds so the user sees mismatches without
            # having to click the button. Debounced via QTimer so
            # Marlin has time to finish booting + responding to the
            # _setup_printer M-codes. Simulator has nothing to align
            # against, so skip in that mode.
            if ok and not simulate:
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

    # ── Xbox (moved from the deprecated Dashboard page) ──────────

    def _connect_xbox(self):
        """v7.4.2: Connect Xbox controller using saved settings.

        Mirrors the parameters DashboardPage used: mapping file, thread
        mode (macOS), reconnect timeout, stick offsets, per-axis
        deadzones, debug mode. Anything missing falls back to sensible
        defaults.
        """
        ctrl = self._controller
        if ctrl is None:
            return
        self.badge_xbox.set_status("info", "Connecting…")
        try:
            import platform
            use_thread = platform.system() == "Darwin"
            mapping = getattr(
                ctrl, "_mapping_file", "current_button_mapping.json")
            s_obj = self._settings
            if s_obj is not None:
                timeout = s_obj.get("xbox.reconnect_timeout_s", 30)
                stick_offsets = s_obj.get_section("xbox_stick_offsets") or {}
                if stick_offsets:
                    stick_offsets = {int(k): v for k, v in stick_offsets.items()}
                stick_dz = s_obj.get("xbox.deadzones.sticks", 0.20)
                trigger_dz = s_obj.get("xbox.deadzones.triggers", 0.05)
                debug_mode = bool(s_obj.get("xbox.debug_mode", False))
            else:
                timeout = 30
                stick_offsets = {}
                stick_dz = 0.20
                trigger_dz = 0.05
                debug_mode = False
            axis_deadzones = {
                0: stick_dz, 1: stick_dz, 2: stick_dz, 3: stick_dz,
                4: trigger_dz, 5: trigger_dz,
            }
            ctrl.connect_xbox(
                mapping_file=mapping,
                use_thread=use_thread,
                reconnect_timeout=timeout,
                stick_offsets=stick_offsets or None,
                axis_deadzones=axis_deadzones,
                debug_mode=debug_mode,
            )
            self.badge_xbox.set_status("ok", "Connected")
        except Exception as e:
            logger.warning(f"Xbox connect failed: {e}")
            self.badge_xbox.set_status("err", f"Error: {e}")

    def _disconnect_xbox(self):
        if self._controller is None:
            return
        try:
            self._controller.disconnect_xbox()
        except Exception as e:
            logger.warning(f"disconnect_xbox failed: {e}")
        self.badge_xbox.set_status("pending", "Not connected")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.2: Axis Mapping (logical → physical Marlin axis)
    # ════════════════════════════════════════════════════════════════

    _PHYSICAL_LETTERS = ["X", "Y", "Z", "E"]
    _LOGICAL_AXES = ["Z", "P1", "P2", "P3"]

    def _build_axis_mapping_group(self) -> QGroupBox:
        grp = QGroupBox("Axis Mapping")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(10))

        info = QLabel(
            "Which physical Marlin axis drives each logical axis on "
            "your machine. Defaults: Z→X, P1→Y, P2→Z, P3→E. Change "
            "these to match how your steppers are wired."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"padding-bottom: {sp(4)};")
        lay.addWidget(info)

        # v7.4.2 polish: tidy column-aligned grid.
        self.cmb_axis_map: dict[str, QComboBox] = {}
        grid = QGridLayout()
        grid.setHorizontalSpacing(s(10))
        grid.setVerticalSpacing(s(8))
        grid.setColumnStretch(3, 1)
        for r, logical in enumerate(self._LOGICAL_AXES):
            lbl = QLabel(logical)
            lbl.setStyleSheet(
                f"font-weight: 600; color: {COLORS['text']};"
                f"font-family: monospace;")
            lbl.setMinimumWidth(s(28))
            grid.addWidget(lbl, r, 0)
            arrow = QLabel("→")
            arrow.setStyleSheet(f"color: {COLORS['subtext0']};")
            grid.addWidget(arrow, r, 1)
            cmb = QComboBox()
            for letter in self._PHYSICAL_LETTERS:
                cmb.addItem(f"{letter}  (Marlin {letter})", letter)
            cmb.setMinimumWidth(s(170))
            grid.addWidget(cmb, r, 2)
            self.cmb_axis_map[logical] = cmb
        lay.addLayout(grid)

        # v7.4.2 polish: status takes the stretch, save right-aligned.
        save_row = QHBoxLayout()
        save_row.setSpacing(s(8))
        self.lbl_axis_map_status = QLabel("")
        self.lbl_axis_map_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; ")
        self.lbl_axis_map_status.setWordWrap(True)
        save_row.addWidget(self.lbl_axis_map_status, 1)
        btn_save = icon_button(
            "Save Mapping", "save", object_name="accentBtn",
            tooltip=(
                "Push the mapping to the live ZP stage (if connected) and "
                "persist to settings + the current device profile."))
        btn_save.clicked.connect(self._apply_axis_mapping)
        save_row.addWidget(btn_save)
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
            "Configure the XY controller type and motion characteristics. "
            "Velocity and acceleration are stored in the device profile and "
            "pushed to the controller on Save. The Units verification "
            "test (below) is report-only."
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; ")
        outer.addWidget(info)

        # ── XY controller type (per-machine) ───────────────────
        # v7.5.x: which XY controller this machine uses. Persisted into the
        # global controller.controller_json key on Save (device profile → global
        # → auto-detect). Applies on the next XY reconnect.
        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(QLabel("XY controller:"))
        self.cmb_xy_controller = QComboBox()
        self.cmb_xy_controller.setMinimumWidth(s(200))
        self.cmb_xy_controller.setToolTip(
            "The XY stage controller for THIS machine (Prior ProScan, Ludl "
            "MAC 5000, …). Saved with the device profile; takes effect on the "
            "next XY reconnect. 'Auto-Detect' identifies the controller at "
            "connect.")
        self._populate_xy_controller_combo()
        self.cmb_xy_controller.currentIndexChanged.connect(
            self._on_xy_controller_changed)
        ctrl_row.addWidget(self.cmb_xy_controller, 1)
        outer.addLayout(ctrl_row)

        self.lbl_xy_controller_hint = QLabel("")
        self.lbl_xy_controller_hint.setWordWrap(True)
        self.lbl_xy_controller_hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; ")
        outer.addWidget(self.lbl_xy_controller_hint)

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
            "Max velocity as a percentage of the controller's top speed "
            "(Prior SMS, or a fraction of the measured ceiling for Ludl). "
            "Default 100%.")
        self._lbl_xy_velocity = QLabel("Velocity (% max):")
        form.addRow(self._lbl_xy_velocity, self.spin_xy_velocity)

        self.spin_xy_acceleration = QDoubleSpinBox()
        self.spin_xy_acceleration.setRange(1, 100)
        self.spin_xy_acceleration.setDecimals(0)
        self.spin_xy_acceleration.setValue(50)
        self.spin_xy_acceleration.setMinimumWidth(s(120))
        self.spin_xy_acceleration.setToolTip(
            "Acceleration. Prior SAS = percentage 1-100; Ludl ACCEL = a "
            "1-255 ramp index (lower = snappier). Default 50.")
        self._lbl_xy_acceleration = QLabel("Acceleration:")
        form.addRow(self._lbl_xy_acceleration, self.spin_xy_acceleration)

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

        # ── v7.5.x: Live stage state — read directly from hardware ────
        # Motivated by a real incident: a bad command silently corrupted ONE
        # axis's SPEED register on a Ludl MAC 5000 while the other stayed
        # normal, and there was no way to SEE that divergence from the app —
        # only ad-hoc bench scripts caught it. This surfaces the stage's
        # ACTUAL current velocity/acceleration, independent of whatever the
        # device profile/software thinks is configured.
        live_grp = QGroupBox("Live Stage State (read from hardware)")
        live_grp.setStyleSheet(SECTION_TITLE_STYLE)
        live_outer = QVBoxLayout(live_grp)
        live_info = QLabel(
            "Query the CONNECTED stage's actual velocity/acceleration "
            "registers on demand — independent of the values above. Ludl "
            "shows X and Y separately since they are genuinely independent "
            "per-axis registers and CAN diverge; Prior has one shared value."
        )
        live_info.setWordWrap(True)
        live_info.setStyleSheet(f"color: {COLORS['subtext0']}; ")
        live_outer.addWidget(live_info)

        live_grid = QFormLayout()
        live_grid.setHorizontalSpacing(s(10))
        self.lbl_xy_live_speed_x = QLabel("—")
        self.lbl_xy_live_speed_y = QLabel("—")
        self.lbl_xy_live_accel_x = QLabel("—")
        self.lbl_xy_live_accel_y = QLabel("—")
        self._lbl_xy_live_speed_x_row = QLabel("Velocity (X):")
        self._lbl_xy_live_speed_y_row = QLabel("Velocity (Y):")
        self._lbl_xy_live_accel_x_row = QLabel("Acceleration (X):")
        self._lbl_xy_live_accel_y_row = QLabel("Acceleration (Y):")
        live_grid.addRow(self._lbl_xy_live_speed_x_row, self.lbl_xy_live_speed_x)
        live_grid.addRow(self._lbl_xy_live_speed_y_row, self.lbl_xy_live_speed_y)
        live_grid.addRow(self._lbl_xy_live_accel_x_row, self.lbl_xy_live_accel_x)
        live_grid.addRow(self._lbl_xy_live_accel_y_row, self.lbl_xy_live_accel_y)
        live_outer.addLayout(live_grid)

        live_btn_row = QHBoxLayout()
        btn_xy_read_stage = icon_button(
            "Read from stage", "refresh",
            tooltip="Query the connected stage's current velocity/"
                    "acceleration directly (no motion; read-only).")
        btn_xy_read_stage.clicked.connect(self._read_xy_stage_state)
        live_btn_row.addWidget(btn_xy_read_stage)
        self.lbl_xy_live_status = QLabel("")
        self.lbl_xy_live_status.setWordWrap(True)
        self.lbl_xy_live_status.setStyleSheet(f"color: {COLORS['subtext0']}; ")
        live_btn_row.addWidget(self.lbl_xy_live_status, 1)
        live_outer.addLayout(live_btn_row)
        outer.addWidget(live_grp)

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
            f"color: {COLORS['subtext0']}; ")
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
            f"color: {COLORS['subtext0']}; ")
        self.lbl_xy_verify_result.setWordWrap(True)
        v_outer.addWidget(self.lbl_xy_verify_result)
        outer.addWidget(verify_grp)

        # ── Save XY Calibration button ─────────────────────────
        save_row = QHBoxLayout()
        btn_save_xy = icon_button(
            "Save XY Calibration", "save", object_name="successBtn",
            tooltip=("Send velocity/acceleration/jerk to ProScan and "
                     "persist to settings + device profile."))
        btn_save_xy.clicked.connect(self._apply_xy_calibration)
        save_row.addWidget(btn_save_xy)
        self.lbl_xy_cal_status = QLabel("")
        self.lbl_xy_cal_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; ")
        save_row.addWidget(self.lbl_xy_cal_status, 1)
        outer.addLayout(save_row)
        return grp

    def _populate_xy_controller_combo(self) -> None:
        """Fill the XY-controller combo: Auto / Default / one per protocol JSON.

        Mirrors settings_page._populate_controller_combo so the same choices
        appear per-machine on the Device page.
        """
        from SupportClasses.ControllerProtocol import (
            ControllerProtocol, discover_controller_files,
        )
        cmb = self.cmb_xy_controller
        cmb.blockSignals(True)
        cmb.clear()
        cmb.addItem("Auto-Detect", "auto")
        cmb.addItem("Default (ProScan III)", None)
        for fp in discover_controller_files():
            try:
                proto = ControllerProtocol.load(fp)
                cmb.addItem(proto.controller_name, str(fp))
            except Exception:
                cmb.addItem(f"⚠ {fp.stem}", str(fp))
        cmb.blockSignals(False)

    def _select_xy_controller(self, value) -> None:
        """Select the combo entry whose data == value ('auto' / path / None)."""
        cmb = self.cmb_xy_controller
        idx = cmb.findData(value)
        if idx < 0 and isinstance(value, str):
            # Match by resolved path (settings may store a normalized string).
            for i in range(cmb.count()):
                d = cmb.itemData(i)
                if isinstance(d, str) and Path(d) == Path(value):
                    idx = i
                    break
        if idx < 0:
            idx = cmb.findData("auto")
        cmb.blockSignals(True)
        cmb.setCurrentIndex(max(0, idx))
        cmb.blockSignals(False)
        self._relabel_xy_cal_for_json(cmb.currentData())

    def _relabel_xy_cal_for_json(self, controller_json) -> None:
        """Relabel/re-range the velocity+accel spinboxes for the selected model.

        Percentage model (Prior): Velocity 1-100 %, Acceleration 1-100 (higher
        = faster ramp, the usual convention).
        Absolute model (Ludl): Velocity stays % of the measured top speed;
        Acceleration becomes a 1-255 ramp-TIME index. v7.5.x: bench-confirmed
        on a real MAC 5000 (2026-07-22) that this is BACKWARDS from the usual
        convention — LOWER is FASTER (a 200 µm test move settled in 0.61s at
        ACCEL=1 vs 0.88s at ACCEL=255, same SPEED). Only the accel RANGE
        differs, so a value round-trips through the device profile either way.
        """
        family = "prior"
        accel_max = 100
        hint = ""
        self._xy_accel_family_default = 50
        if isinstance(controller_json, str) and controller_json not in ("auto", ""):
            try:
                from SupportClasses.ControllerProtocol import ControllerProtocol
                proto = ControllerProtocol.load(controller_json)
                family = proto.family
                rng = proto.get_acceleration_range()
                if rng:
                    accel_max = int(rng[1])
                if proto.accel_model == "absolute":
                    self._lbl_xy_acceleration.setText(
                        "Acceleration (ramp 1-255, LOWER = faster):")
                else:
                    self._lbl_xy_acceleration.setText("Acceleration:")
                if family == "ludl":
                    self._xy_accel_family_default = 1
                    hint = (
                        "Ludl MAC 5000: Velocity is a % of the measured top "
                        "speed. Acceleration is a 1-255 RAMP-TIME index — "
                        "bench-confirmed BACKWARDS from the usual convention: "
                        "LOWER is FASTER/snappier (255 is the SLOWEST setting, "
                        "not the fastest). Default to a low value (~1-10); do "
                        "not 'max it out' expecting speed. Units (µm/count, "
                        "speed units) are tuned on the bench — see "
                        "config/controllers/mac5000.json. Change takes effect "
                        "on the next XY reconnect.")
            except Exception as e:
                logger.debug("relabel XY cal failed: %s", e)
        else:
            self._lbl_xy_acceleration.setText("Acceleration:")
        # Apply accel range without dropping the current value below it.
        cur = self.spin_xy_acceleration.value()
        self.spin_xy_acceleration.setRange(1, accel_max)
        self.spin_xy_acceleration.setValue(min(cur, accel_max) if cur else cur)
        self.lbl_xy_controller_hint.setText(hint)
        # v7.5.x: Prior has ONE shared velocity/acceleration value (querying
        # "axis" is meaningless — the command template ignores it) — hide the
        # redundant Y row rather than show an identical duplicate. Ludl's X/Y
        # registers are genuinely independent (can diverge — see the group's
        # motivating incident), so both rows show for it.
        per_axis = (family == "ludl")
        if hasattr(self, "_lbl_xy_live_speed_y_row"):
            self._lbl_xy_live_speed_y_row.setVisible(per_axis)
            self.lbl_xy_live_speed_y.setVisible(per_axis)
            self._lbl_xy_live_accel_y_row.setVisible(per_axis)
            self.lbl_xy_live_accel_y.setVisible(per_axis)
            self._lbl_xy_live_speed_x_row.setText(
                "Velocity (X):" if per_axis else "Velocity:")
            self._lbl_xy_live_accel_x_row.setText(
                "Acceleration (X):" if per_axis else "Acceleration:")

    def _on_xy_controller_changed(self, *_args) -> None:
        """Combo change: relabel and note that a reconnect is required.

        v7.5.x BUGFIX: this previously persisted only to ``settings.json``,
        NOT to the active device profile FILE (``_persist_active_profile``),
        unlike "Save XY Calibration" — so the on-disk profile kept the OLD
        controller. The next time that profile was loaded (a normal action,
        not just a stale-cache accident), its stale ``xy_controller_json``
        silently overwrote the fresh choice, making the switch look like it
        "didn't take" / "kept looking for" the old controller. Both persist
        paths must agree, so this now calls ``_persist_active_profile()`` too.
        """
        data = self.cmb_xy_controller.currentData()
        self._relabel_xy_cal_for_json(data)
        # Persist immediately so it survives even without a full XY-cal Save,
        # and push to the controller for the next connect.
        s = self._settings
        if s is not None:
            s.set("controller.controller_json", data)
            try:
                s.save()
            except Exception:
                pass
            # Keep the active device profile FILE in sync — see docstring.
            self._persist_active_profile()
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_controller_json"):
            ctrl.set_controller_json(data)
        if self._controller is not None and self._controller.is_xy_connected:
            base = self.lbl_xy_cal_status
            base.setText("XY controller changed — reconnect the XY stage to "
                         "apply.")

    def _read_xy_stage_state(self) -> None:
        """Query the connected stage's CURRENT velocity/acceleration directly.

        Read-only — no motion, no settings written. Best-effort per axis;
        any query that fails (unsupported/unconnected/malformed reply) shows
        'unavailable' for that field rather than raising. A MISMATCH between
        X and Y (Ludl only — Prior has one shared value) is flagged, since
        that divergence is exactly what a bad command can silently cause.
        """
        ctrl = self._controller
        xy = getattr(ctrl, "xy_stage", None) if ctrl is not None else None
        if xy is None:
            self.lbl_xy_live_status.setText("XY stage not connected.")
            for lbl in (self.lbl_xy_live_speed_x, self.lbl_xy_live_speed_y,
                        self.lbl_xy_live_accel_x, self.lbl_xy_live_accel_y):
                lbl.setText("—")
            return
        try:
            speed_x = xy.get_speed_readback("X")
            speed_y = xy.get_speed_readback("Y")
            accel_x = xy.get_acceleration_readback("X")
            accel_y = xy.get_acceleration_readback("Y")
        except Exception as e:
            self.lbl_xy_live_status.setText(f"Read failed: {e}")
            return
        self.lbl_xy_live_speed_x.setText(speed_x["display"])
        self.lbl_xy_live_speed_y.setText(speed_y["display"])
        self.lbl_xy_live_accel_x.setText(accel_x["display"])
        self.lbl_xy_live_accel_y.setText(accel_y["display"])

        family = xy.protocol.family if xy.protocol else "prior"
        warn = ""
        if family == "ludl":
            if (speed_x["raw"] is not None and speed_y["raw"] is not None
                    and speed_x["raw"] != speed_y["raw"]):
                warn += " ⚠ X/Y velocity MISMATCH."
                self.lbl_xy_live_speed_x.setStyleSheet(f"color: {COLORS['yellow']};")
                self.lbl_xy_live_speed_y.setStyleSheet(f"color: {COLORS['yellow']};")
            else:
                self.lbl_xy_live_speed_x.setStyleSheet("")
                self.lbl_xy_live_speed_y.setStyleSheet("")
            if (accel_x["raw"] is not None and accel_y["raw"] is not None
                    and accel_x["raw"] != accel_y["raw"]):
                warn += " ⚠ X/Y acceleration MISMATCH."
                self.lbl_xy_live_accel_x.setStyleSheet(f"color: {COLORS['yellow']};")
                self.lbl_xy_live_accel_y.setStyleSheet(f"color: {COLORS['yellow']};")
            else:
                self.lbl_xy_live_accel_x.setStyleSheet("")
                self.lbl_xy_live_accel_y.setStyleSheet("")
        any_unavailable = any(
            d["raw"] is None for d in (speed_x, speed_y, accel_x, accel_y)
            if family == "ludl" or d is speed_x or d is accel_x)
        status = "Read from stage." + warn
        if any_unavailable:
            status += " (some fields unavailable — see tooltip/logs.)"
        self.lbl_xy_live_status.setText(status)

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
        # v7.5.x: persist the per-machine XY controller choice into the global
        # key (device profile → global → auto) + push to the controller for the
        # next connect.
        if hasattr(self, "cmb_xy_controller"):
            ctrl_json = self.cmb_xy_controller.currentData()
            s.set("controller.controller_json", ctrl_json)
            if self._controller is not None and hasattr(
                    self._controller, "set_controller_json"):
                self._controller.set_controller_json(ctrl_json)
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
        # v7.4.2: also persist to the active profile JSON.
        self._persist_active_profile()
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
        # v7.4.2 polish: short title; in-section sub-headings carry the
        # hierarchy.
        from gui.pages.hardware._polish import sub_heading, divider

        grp = QGroupBox("ZP Stage Calibration")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        outer = QVBoxLayout(grp)
        outer.setSpacing(s(10))

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
            f"color: {COLORS['subtext0']}; "
            f"padding-bottom: {sp(4)};")
        outer.addWidget(info)

        # v7.4.2 polish: heading anchors the "live values" block
        outer.addWidget(sub_heading("Current Values"))

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

        # v7.4.2 polish: single aligned grid for all per-axis values.
        # Rows = parameter, columns = logical axis. Cells line up so
        # the eye can scan steps/feedrate/accel across Z/P1/P2/P3 at a
        # glance instead of chasing labels of varying width.
        values_grid = QGridLayout()
        values_grid.setHorizontalSpacing(s(12))
        values_grid.setVerticalSpacing(s(8))
        values_grid.setColumnStretch(0, 0)  # row label
        for c in range(1, 1 + len(self._LOGICAL_AXES)):
            values_grid.setColumnStretch(c, 1)

        # Column headers (axis names) in row 0
        for col, ax in enumerate(self._LOGICAL_AXES, start=1):
            head = QLabel(ax)
            head.setAlignment(Qt.AlignCenter)
            head.setStyleSheet(
                f"color: {COLORS['blue']}; font-weight: 700; "
                f"font-family: monospace; "
                f"letter-spacing: 0.4px;")
            values_grid.addWidget(head, 0, col)

        def _row_label(text: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-weight: 500; "
                f"")
            lbl.setMinimumWidth(s(160))
            return lbl

        # Row 1: steps/mm (clickable cells)
        values_grid.addWidget(_row_label("steps/mm"), 1, 0)
        self.lbl_steps_grid: dict[str, _ClickableLabel] = {}
        for col, ax in enumerate(self._LOGICAL_AXES, start=1):
            cell = _ClickableLabel("—")
            cell.setAlignment(Qt.AlignCenter)
            cell.setStyleSheet(self._cell_style_ok)
            cell.setToolTip(f"Click to edit {ax} steps/mm directly")
            cell.clicked.connect(
                lambda a=ax: self._edit_steps_per_mm_popup(a))
            values_grid.addWidget(cell, 1, col)
            self.lbl_steps_grid[ax] = cell

        # Row 2: max feedrate (clickable cells)
        values_grid.addWidget(_row_label("max feedrate (mm/min)"), 2, 0)
        self.lbl_feedrate_grid: dict[str, _ClickableLabel] = {}
        for col, ax in enumerate(self._LOGICAL_AXES, start=1):
            cell = _ClickableLabel("—")
            cell.setAlignment(Qt.AlignCenter)
            cell.setStyleSheet(self._cell_style_ok)
            cell.setToolTip(f"Click to edit {ax} max feedrate directly")
            cell.clicked.connect(
                lambda a=ax: self._edit_max_feedrate_popup(a))
            values_grid.addWidget(cell, 2, col)
            self.lbl_feedrate_grid[ax] = cell

        # Row 3: max acceleration (editable spinboxes — same column width)
        values_grid.addWidget(_row_label("max accel (mm/s²)"), 3, 0)
        self.spin_axis_accel: dict[str, QDoubleSpinBox] = {}
        default_accel = {"Z": 100.0, "P1": 1000.0, "P2": 1000.0, "P3": 1000.0}
        for col, ax in enumerate(self._LOGICAL_AXES, start=1):
            sp_w = QDoubleSpinBox()
            sp_w.setRange(1.0, 100000.0)
            sp_w.setDecimals(0)
            sp_w.setSingleStep(50.0)
            sp_w.setValue(default_accel.get(ax, 1000.0))
            sp_w.setAlignment(Qt.AlignCenter)
            values_grid.addWidget(sp_w, 3, col)
            self.spin_axis_accel[ax] = sp_w

        outer.addLayout(values_grid)

        # v7.4.2 polish: heading for the interactive calibration workflow
        outer.addWidget(sub_heading("Calibrate / Test"))

        # v7.4.2 polish: aligned form grid — labels in col 0, inputs in
        # col 1, action buttons in col 2 (spanning is OK). Every input
        # shares a column width so the eye lines up.
        form = QGridLayout()
        form.setHorizontalSpacing(s(12))
        form.setVerticalSpacing(s(8))
        form.setColumnStretch(0, 0)
        form.setColumnStretch(1, 0)
        form.setColumnStretch(2, 1)  # button area soaks remaining space

        def _form_label(text: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-weight: 500; "
                f"")
            lbl.setMinimumWidth(s(110))
            return lbl

        # Row 0: Axis picker
        form.addWidget(_form_label("Axis"), 0, 0)
        self.cmb_cal_axis = QComboBox()
        for ax in self._LOGICAL_AXES:
            self.cmb_cal_axis.addItem(ax, ax)
        self.cmb_cal_axis.setMinimumWidth(s(160))
        self.cmb_cal_axis.currentIndexChanged.connect(
            self._on_cal_axis_changed)
        form.addWidget(self.cmb_cal_axis, 0, 1)

        # Row 1: Distance + Move +/- buttons (compact button group on right)
        form.addWidget(_form_label("Commanded distance"), 1, 0)
        self.spin_cal_commanded = QDoubleSpinBox()
        self.spin_cal_commanded.setRange(0.001, 100.0)
        self.spin_cal_commanded.setDecimals(3)
        self.spin_cal_commanded.setValue(1.0)
        self.spin_cal_commanded.setSuffix(" mm")
        self.spin_cal_commanded.setMinimumWidth(s(160))
        form.addWidget(self.spin_cal_commanded, 1, 1)

        move_btns = QHBoxLayout()
        move_btns.setSpacing(s(6))
        move_btns.setContentsMargins(0, 0, 0, 0)
        self.btn_cal_move_fwd = QPushButton("Move +")
        self.btn_cal_move_fwd.setCursor(Qt.PointingHandCursor)
        self.btn_cal_move_fwd.setToolTip(
            "Send the distance in the positive direction")
        self.btn_cal_move_fwd.clicked.connect(
            lambda: self._cal_command_move(direction=1))
        move_btns.addWidget(self.btn_cal_move_fwd)
        self.btn_cal_move_rev = QPushButton("Move −")
        self.btn_cal_move_rev.setCursor(Qt.PointingHandCursor)
        self.btn_cal_move_rev.setToolTip(
            "Send the distance in the negative direction")
        self.btn_cal_move_rev.clicked.connect(
            lambda: self._cal_command_move(direction=-1))
        move_btns.addWidget(self.btn_cal_move_rev)
        move_btns.addStretch()
        form.addLayout(move_btns, 1, 2)

        # Row 2: Feedrate + Record-as-Max
        form.addWidget(_form_label("Feedrate"), 2, 0)
        self.spin_cal_feedrate = QDoubleSpinBox()
        self.spin_cal_feedrate.setRange(1.0, 100000.0)
        self.spin_cal_feedrate.setDecimals(0)
        self.spin_cal_feedrate.setSingleStep(50.0)
        self.spin_cal_feedrate.setValue(600.0)
        self.spin_cal_feedrate.setSuffix(" mm/min")
        self.spin_cal_feedrate.setMinimumWidth(s(160))
        form.addWidget(self.spin_cal_feedrate, 2, 1)

        feed_btns = QHBoxLayout()
        feed_btns.setSpacing(s(6))
        feed_btns.setContentsMargins(0, 0, 0, 0)
        self.btn_cal_record_max = QPushButton("Record as Max")
        self.btn_cal_record_max.setCursor(Qt.PointingHandCursor)
        self.btn_cal_record_max.setToolTip(
            "Save the current feedrate as the experimentally-discovered "
            "max for the selected axis. Stored in the device profile "
            "as device_profile.per_axis_max_feedrate.")
        self.btn_cal_record_max.clicked.connect(self._cal_record_max_feedrate)
        feed_btns.addWidget(self.btn_cal_record_max)
        feed_btns.addStretch()
        form.addLayout(feed_btns, 2, 2)

        # Row 3: Measured + Calculate & Invert
        form.addWidget(_form_label("Measured distance"), 3, 0)
        self.spin_cal_measured = QDoubleSpinBox()
        self.spin_cal_measured.setRange(0.001, 100.0)
        self.spin_cal_measured.setDecimals(3)
        self.spin_cal_measured.setValue(1.0)
        self.spin_cal_measured.setSuffix(" mm")
        self.spin_cal_measured.setMinimumWidth(s(160))
        form.addWidget(self.spin_cal_measured, 3, 1)

        apply_btns = QHBoxLayout()
        apply_btns.setSpacing(s(6))
        apply_btns.setContentsMargins(0, 0, 0, 0)
        self.btn_cal_apply = QPushButton("Calculate && Send M92")
        self.btn_cal_apply.setObjectName("accentBtn")
        self.btn_cal_apply.setCursor(Qt.PointingHandCursor)
        self.btn_cal_apply.clicked.connect(self._cal_apply)
        apply_btns.addWidget(self.btn_cal_apply)

        self.btn_cal_flip_axis = QPushButton("Invert Direction")
        self.btn_cal_flip_axis.setCursor(Qt.PointingHandCursor)
        self.btn_cal_flip_axis.setToolTip(
            "Negate the saved steps/mm for the selected axis — flips "
            "which way 'positive' moves on Marlin without changing "
            "step count magnitude.")
        self.btn_cal_flip_axis.clicked.connect(self._cal_flip_axis)
        apply_btns.addWidget(self.btn_cal_flip_axis)
        apply_btns.addStretch()
        form.addLayout(apply_btns, 3, 2)

        outer.addLayout(form)

        self.lbl_cal_status = QLabel("")
        self.lbl_cal_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; ")
        self.lbl_cal_status.setWordWrap(True)
        outer.addWidget(self.lbl_cal_status)

        # v7.4.2 polish: visual divider before the save row so the
        # save/alignment block reads as a footer, not just another row.
        outer.addWidget(divider())

        # v7.4.2 hotfix: per-section Save button. The individual buttons
        # (Calculate & Send M92, Invert Axis Direction, Record as Max)
        # already persist on click — this button is a "save everything in
        # this section now" convenience + a confirmation that values are
        # synced to Marlin.
        # v7.4.2 polish: status label takes the stretch, buttons sit
        # right-aligned where the eye expects "save / commit" actions.
        save_row = QHBoxLayout()
        save_row.setSpacing(s(8))
        self.lbl_alignment_status = QLabel("")
        self.lbl_alignment_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; ")
        self.lbl_alignment_status.setWordWrap(True)
        save_row.addWidget(self.lbl_alignment_status, 1)

        self.btn_check_alignment = icon_button(
            "Check alignment", "search",
            tooltip=(
                "Query Marlin (M503) and compare its reported steps/mm "
                "and max feedrate against this device profile. Axes "
                "that disagree turn red so you know which to re-save."))
        self.btn_check_alignment.clicked.connect(self._check_marlin_alignment)
        save_row.addWidget(self.btn_check_alignment)

        self.btn_load_from_hardware = icon_button(
            "Load values from hardware", "arrow-down",
            tooltip=(
                "Query Marlin (M503) and ADOPT its reported steps/mm, "
                "max feedrate, and max acceleration into software — the "
                "reverse of Save Calibration. Use this when the board "
                "already has the correct values (e.g. just tuned in "
                "firmware) and software should follow hardware instead "
                "of overwriting it."))
        self.btn_load_from_hardware.clicked.connect(
            self._load_calibration_from_hardware)
        save_row.addWidget(self.btn_load_from_hardware)

        btn_save_cal = icon_button(
            "Save Calibration", "save", object_name="accentBtn",
            tooltip=("Re-send steps_per_mm (M92), per-axis max acceleration "
                     "(M201), and per_axis_max_feedrate to Marlin / device "
                     "profile and persist to settings."))
        btn_save_cal.clicked.connect(self._apply_steps_cal)
        save_row.addWidget(btn_save_cal)

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
            # v7.4.2: also persist to the active profile JSON.
            self._persist_active_profile()
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
            # v7.4.2: also persist to the active profile JSON.
            self._persist_active_profile()
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
        # v7.4.2: also persist to the active profile JSON.
        self._persist_active_profile()
        self._refresh_max_feedrate_grid()
        self.lbl_cal_status.setText(
            f"Recorded {axis} max feedrate: {feedrate:.0f} mm/min. "
            f"You can copy this into the global Safety Limits "
            f"max_z_feedrate / max_pump_feedrate when you're confident.")

    def _edit_steps_per_mm_popup(self, axis: str) -> None:
        """v7.4.2 hotfix: click-to-edit popup for a steps/mm cell.

        Pops a QInputDialog with the current value; on accept,
        updates the controller's per-axis dict, sends M92 immediately,
        persists to settings, and refreshes the display grid.
        """
        if self._controller is None or self._controller.zp_stage is None:
            QMessageBox.information(
                self, "Edit steps/mm",
                "ZP stage not connected. Connect first, then edit.")
            return
        current = float(self._controller.zp_stage.steps_per_mm.get(axis, 5069))
        new_val, ok = QInputDialog.getDouble(
            self,
            f"Edit steps/mm — {axis}",
            f"Enter steps/mm for {axis} (negative inverts direction):",
            value=current, minValue=-1e6, maxValue=1e6, decimals=2,
        )
        if not ok:
            return
        new_steps = dict(self._controller.zp_stage.steps_per_mm)
        new_steps[axis] = float(new_val)
        self._controller.zp_stage.set_steps_per_mm(new_steps, persist=True)
        if self._settings is not None:
            self._settings.set("device_profile.steps_per_mm", new_steps)
            self._settings.save()
            # v7.4.2: also persist to the active profile JSON so the
            # edit survives a profile switch round-trip.
            self._persist_active_profile()
        self._refresh_steps_grid()
        if hasattr(self, 'lbl_cal_status'):
            self.lbl_cal_status.setText(
                f"{axis} steps/mm: {current:.2f} → {new_val:.2f}. "
                f"M92 sent and persisted.")
        # Re-check alignment after a direct edit
        try:
            self._check_marlin_alignment(quiet=True)
        except Exception:
            pass

    def _edit_max_feedrate_popup(self, axis: str) -> None:
        """v7.4.2 hotfix: click-to-edit popup for a max feedrate cell.

        Updates ``device_profile.per_axis_max_feedrate``, pushes the
        full per-axis dict to Marlin via M203 (using axis_map to route
        each logical axis to its physical letter), and re-runs the
        alignment check so the red cell turns green.
        """
        per_axis = ((self._settings.get("device_profile.per_axis_max_feedrate")
                     if self._settings else {}) or {}).copy()
        current = float(per_axis.get(axis, 500))
        new_val, ok = QInputDialog.getDouble(
            self,
            f"Edit max feedrate — {axis}",
            f"Enter max feedrate (mm/min) for {axis}:\n"
            f"This is the experimentally-discovered ceiling; ZP "
            f"Feedrates derives absolute mm/min as a % of this.",
            value=current, minValue=1.0, maxValue=100000.0, decimals=0,
        )
        if not ok:
            return
        per_axis[axis] = float(new_val)
        if self._settings is not None:
            self._settings.set("device_profile.per_axis_max_feedrate", per_axis)
            self._settings.save()
            # v7.4.2: also persist to the active profile JSON so the
            # edit survives a profile switch round-trip.
            self._persist_active_profile()
        # v7.4.2 hotfix: push M203 immediately so Marlin agrees and the
        # alignment cell goes green without requiring a Save Calibration.
        # v7.5.x: route through apply_device_settings so the controller's CACHED
        # per-axis max (the single resolver source — get_max_z_feedrate_mm_min)
        # updates too, not just Marlin; it pushes M203 when connected and
        # re-anchors the jog/move feedrates.
        pushed = False
        ctrl = self._controller
        if ctrl is not None:
            try:
                ctrl.apply_device_settings(
                    per_axis_max_feedrate=per_axis, persist_feedrate=True)
                pushed = ctrl.zp_stage is not None
            except Exception as e:
                logger.warning(f"apply per-axis feedrate (popup) failed: {e}")
        self._refresh_max_feedrate_grid()
        # The ZP Feedrates derived-mm/min display uses Z max — if Z
        # changed, refresh the derived labels.
        if axis == "Z" and hasattr(self, '_refresh_zp_feedrate_derived'):
            self._refresh_zp_feedrate_derived()
        # v7.5.x: a per-axis MAX changed → refresh the Control Panel speed read-
        # outs + emit safety_limits_changed so every page re-reads the single
        # common source (app fans refresh_speed_limits out to all jog surfaces).
        if axis == "Z":
            self._notify_control_panel_safety_changed()
        # Re-check alignment so the cell colour reflects the new state.
        try:
            self._check_marlin_alignment(quiet=True)
        except Exception:
            pass
        if hasattr(self, 'lbl_cal_status'):
            suffix = " (M203 sent to Marlin)" if pushed else ""
            self.lbl_cal_status.setText(
                f"{axis} max feedrate: {current:.0f} → {new_val:.0f} mm/min."
                f"{suffix}")

    def _refresh_max_feedrate_grid(self):
        """Refresh the per-axis max feedrate display row.

        v7.4.2 polish: cells now sit in a column-aligned grid under
        axis headers, so each cell displays only the value (the row
        label sits in column 0, the axis header in row 0).
        """
        if not hasattr(self, 'lbl_feedrate_grid'):
            return
        per_axis = (self._settings.get("device_profile.per_axis_max_feedrate")
                    if self._settings else {}) or {}
        for ax, lbl in self.lbl_feedrate_grid.items():
            val = per_axis.get(ax, "—")
            lbl.setText(str(val) if val != "—" else "—")

    def _refresh_steps_grid(self):
        if self._controller and self._controller.zp_stage:
            steps = self._controller.zp_stage.steps_per_mm
        else:
            steps = (self._settings.get("device_profile.steps_per_mm")
                     if self._settings else {}) or {}
        for ax, lbl in self.lbl_steps_grid.items():
            val = steps.get(ax, "—")
            lbl.setText(str(val) if val != "—" else "—")

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

        v7.4.2: Also nudges the persistent HardwareControlPanel's
        position labels so the left-side live display stays in sync
        when a Set Zero / Set Min / Set Max click forces a fresh read.
        """
        # v7.5.x: mark a readout currently showing a (display-only) motion
        # estimate with a leading '~'. The Override card stays unmarked (it
        # re-declares the firmware counter, not a live readout).
        est: set = set()
        if self._controller is not None and hasattr(
                self._controller, "motion_estimating"):
            try:
                est = self._controller.motion_estimating()
            except Exception:
                est = set()

        def _mark(axis, txt):
            return ("~" + txt) if axis in est else txt

        def _set(d, key, val):
            if val is None or not hasattr(self, d) or key not in getattr(self, d):
                return
            getattr(self, d)[key].setText(val)
        if xy:
            if xy[0] is not None:
                txt = _mark("X", f"{xy[0]:,.1f}")
                _set("lbl_axis_pos", "X", txt)
                _set("lbl_jog_pos", "X", txt)
            if xy[1] is not None:
                txt = _mark("Y", f"{xy[1]:,.1f}")
                _set("lbl_axis_pos", "Y", txt)
                _set("lbl_jog_pos", "Y", txt)
        if zp:
            for logical in ("Z", "P1", "P2", "P3"):
                v = self._logical_zp_value(zp, logical)
                if v is None:
                    continue
                # v7.5.x: the limit-row + live-position readouts show Z in the
                # unified user frame (0 at the bottom datum, + up); a calibrated
                # pump shows the signed travel from the empty datum (0 = empty,
                # + toward full) so "full" reads positive regardless of the raw
                # motor direction. (The Override card below stays zero-ref raw —
                # it re-declares the firmware counter.)
                disp = (self._z_raw_to_user(v) if logical == "Z"
                        else self._pump_raw_to_user(logical, v))
                txt = _mark(logical, f"{disp:.3f}")
                _set("lbl_axis_pos", logical, txt)
                _set("lbl_jog_pos", logical, txt)
                # v7.5.x: the Override card shows the zero-referenced
                # value (the frame the user types into), not the raw
                # firmware counter shown by the limit-row readout above.
                if self._controller is not None:
                    zero = float(self._controller.zero_position.get(logical, 0.0))
                    _set("lbl_override_pos", logical, f"{v - zero:.3f}")
        # v7.4.2: nudge the persistent left-side control panel too.
        page = self.parent()
        ctrl_panel = None
        while page is not None and ctrl_panel is None:
            ctrl_panel = getattr(page, "_control_panel", None)
            page = page.parent() if hasattr(page, "parent") else None
        if ctrl_panel is not None:
            try:
                ctrl_panel._update_position_displays(xy, zp)
            except Exception:
                pass

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
            # v7.5.x: display getters animate toward a jog/travel target, then
            # snap to the measured value (fallback to cached for stubs).
            get_xy = getattr(ctrl, "get_display_xy_position", None) or (
                lambda: ctrl.get_xy_position(cached=True))
            get_zp = getattr(ctrl, "get_display_zp_position", None) or (
                lambda: ctrl.get_zp_position(cached=True))
            xy = get_xy()
            zp = get_zp()
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

    # ── v7.5.x: unified user-facing Z conversion (0 at bottom, + up) ──
    #
    # Route every Z display/spinbox through the live controller so the whole
    # UI agrees on the per-machine datum + direction. Falls back to the module
    # height helpers (no datum subtraction) only when there is no controller
    # (headless/standalone), which is never the case in the live app.

    def _z_raw_to_user(self, raw_mm: float) -> float:
        c = self._controller
        if c is not None and hasattr(c, "raw_to_user_z"):
            return c.raw_to_user_z(raw_mm)
        return z_raw_to_display(raw_mm)

    def _z_user_to_raw(self, user_mm: float) -> float:
        c = self._controller
        if c is not None and hasattr(c, "user_z_to_raw"):
            return c.user_z_to_raw(user_mm)
        return z_display_to_raw(user_mm)

    def _pump_raw_to_user(self, logical: str, raw: float) -> float:
        """v7.5.x: a calibrated pump's displayed position is the signed travel
        from the empty (dispensed) datum — 0 at empty, growing POSITIVE toward
        full — so the readout reads 0.0 empty / +stroke full regardless of which
        raw direction the motor counts (mirrors how Z is shown in the z_up_sign
        user frame). Uncalibrated pumps fall back to the raw Marlin value."""
        c = self._controller
        if (c is not None and hasattr(c, "is_pump_plunger_calibrated")
                and c.is_pump_plunger_calibrated(logical)):
            try:
                sign = float(c.pump_aspirate_sign(logical))
                zero = float(c.zero_position.get(logical, 0.0))
                val = sign * (float(raw) - zero)
                return 0.0 if val == 0.0 else val   # normalise -0.0 → 0.0
            except Exception:
                return float(raw)
        return float(raw)

    def _pump_user_to_raw(self, logical: str, user: float) -> float:
        """Inverse of :meth:`_pump_raw_to_user`: user-fill mm (0 = empty, +
        toward full) → absolute Marlin raw mm. Used when writing the fill-frame
        limit spinboxes back to the raw-stored ``safety_limits.p*``. Since
        ``aspirate_sign`` ∈ {+1, -1}, ``raw = zero + sign·user`` inverts
        ``user = sign·(raw − zero)`` exactly. Uncalibrated pumps pass through."""
        c = self._controller
        if (c is not None and hasattr(c, "is_pump_plunger_calibrated")
                and c.is_pump_plunger_calibrated(logical)):
            try:
                sign = float(c.pump_aspirate_sign(logical))
                zero = float(c.zero_position.get(logical, 0.0))
                return zero + sign * float(user)
            except Exception:
                return float(user)
        return float(user)

    # ── v7.5.x: Z Axis Setup — one procedure for datum + direction + limits ──

    def _build_z_axis_setup_group(self) -> QGroupBox:
        """The single Z setup: jog to the mechanical extremes and capture them.
        Bottom (needle all the way DOWN) becomes user Z = 0; Top (all the way
        UP) fixes the travel. The controller DERIVES the up-direction from the
        two readings (so user Z always increases as the needle rises) and sets
        the soft-limit envelope. Re-running it is also the migration path."""
        grp = QGroupBox("Z Axis Setup — sets 0 at the bottom, direction & limits")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        v = QVBoxLayout(grp)
        info = QLabel(
            "Use the jog pad to drive the needle to each mechanical extreme, "
            "then capture it. <b>Bottom = needle all the way DOWN → Z = 0.</b> "
            "Top = all the way UP. Direction (up = +) and the Z soft limits are "
            "derived automatically. Max / Replace Z stay manual."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']};")
        v.addWidget(info)
        row = QHBoxLayout()
        self.btn_z_setup_bottom = QPushButton("1 · Set Bottom (Z = 0)")
        self.btn_z_setup_bottom.setToolTip(
            "Jog the needle ALL THE WAY DOWN first, then click. This raw Z "
            "becomes user Z = 0 (the datum).")
        self.btn_z_setup_bottom.clicked.connect(self._z_setup_capture_bottom)
        row.addWidget(self.btn_z_setup_bottom)
        self.btn_z_setup_top = QPushButton("2 · Set Top")
        self.btn_z_setup_top.setToolTip(
            "Jog the needle ALL THE WAY UP, then click. The up-direction and "
            "the Z soft-limit envelope are derived from bottom → top.")
        self.btn_z_setup_top.setEnabled(False)
        self.btn_z_setup_top.clicked.connect(self._z_setup_capture_top)
        row.addWidget(self.btn_z_setup_top)
        row.addStretch(1)
        v.addLayout(row)
        self.lbl_z_setup_status = QLabel("Bottom not captured yet.")
        self.lbl_z_setup_status.setWordWrap(True)
        self.lbl_z_setup_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        v.addWidget(self.lbl_z_setup_status)
        self._z_setup_bottom_raw: float | None = None

        # ── Standard plate offsets (mm BELOW the needle-cam fiducial) ──
        # Used by the calibration page to PRE-FILL plate Z guesses from the
        # needle-tip-camera Z captured during XY needle cal.
        off_lbl = QLabel(
            "Standard plate offsets — mm below the needle-cam Z to each "
            "feature (used to pre-fill plate Z guesses):")
        off_lbl.setWordWrap(True)
        off_lbl.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        v.addWidget(off_lbl)
        off_row = QHBoxLayout()

        def _off_spin(default: float) -> QDoubleSpinBox:
            sp_w = QDoubleSpinBox()
            sp_w.setRange(-500.0, 500.0)
            sp_w.setDecimals(2)
            sp_w.setSuffix(" mm")
            sp_w.setValue(default)
            sp_w.setMinimumWidth(s(90))
            return sp_w

        self.spin_zoff_top = _off_spin(10.0)
        self.spin_zoff_bottom = _off_spin(20.0)
        self.spin_zoff_safe = _off_spin(5.0)
        for lbl_text, spin in (("Top:", self.spin_zoff_top),
                               ("Bottom:", self.spin_zoff_bottom),
                               ("Safe:", self.spin_zoff_safe)):
            off_row.addWidget(QLabel(lbl_text))
            off_row.addWidget(spin)
        btn_off = QPushButton("Save offsets")
        btn_off.setToolTip("Persist the standard plate offsets to the device "
                           "profile and push them to the controller.")
        btn_off.clicked.connect(self._save_plate_z_offsets)
        off_row.addWidget(btn_off)
        off_row.addStretch(1)
        v.addLayout(off_row)

        # ── v7.5.x: well-plate orientation (single source of truth) ──
        # One per-machine setting drives BOTH the 180° display flip on every
        # stage-frame plate view AND the plate-local→stage geometry sign, so
        # "well A1 top-left / stage 0,0 bottom-right" is enforced everywhere.
        self.chk_plate_flip_180 = QCheckBox(
            "Plate mounted 180° to stage (well A1 top-left, stage 0,0 "
            "bottom-right)")
        self.chk_plate_flip_180.setToolTip(
            "ON for the ME3B V1 Prior stage (origin bottom-right, +X/+Y toward "
            "top-left). Drives the display flip on all plate views and the "
            "geometric well→stage mapping. Re-teach plate calibration after "
            "changing this.")
        self.chk_plate_flip_180.toggled.connect(self._on_plate_flip_toggled)
        v.addWidget(self.chk_plate_flip_180)
        return grp

    def _on_plate_flip_toggled(self, checked: bool) -> None:
        """Push the orientation to the controller + persist it. Mirrors the
        z_up_sign persistence path."""
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_plate_flip_180"):
            ctrl.set_plate_flip_180(bool(checked))
        s = self._settings
        if s is not None:
            s.set("device_profile.plate_flip_180", bool(checked))
            s.save()
            try:
                self._persist_active_profile()
            except Exception:
                pass

    def _save_plate_z_offsets(self) -> None:
        """Persist the three standard plate offsets + push to the controller."""
        top = self.spin_zoff_top.value()
        bottom = self.spin_zoff_bottom.value()
        safe = self.spin_zoff_safe.value()
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_plate_z_offsets"):
            ctrl.set_plate_z_offsets(top=top, bottom=bottom, safe=safe)
        s = self._settings
        if s is not None:
            s.set("device_profile.plate_z_offsets",
                  {"top": top, "bottom": bottom, "safe": safe})
            s.save()
            try:
                self._persist_active_profile()
            except Exception:
                pass
        if hasattr(self, "lbl_z_setup_status"):
            self.lbl_z_setup_status.setText(
                f"Plate offsets saved: top {top:.2f}, bottom {bottom:.2f}, "
                f"safe {safe:.2f} mm below needle-cam.")
            self.lbl_z_setup_status.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 9pt;")

    def _z_setup_capture_bottom(self) -> None:
        ctrl = self._controller
        if ctrl is None or not hasattr(ctrl, "capture_current_z_raw"):
            self.lbl_z_setup_status.setText("No controller available.")
            return
        raw = ctrl.capture_current_z_raw()
        if raw is None:
            self.lbl_z_setup_status.setText(
                "Could not read Z (is the ZP controller connected?).")
            return
        self._z_setup_bottom_raw = float(raw)
        self.btn_z_setup_top.setEnabled(True)
        self.lbl_z_setup_status.setText(
            f"Bottom captured (raw {raw:.3f} mm) → will become Z = 0. "
            f"Now jog the needle ALL THE WAY UP and click Set Top.")
        self.lbl_z_setup_status.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: 9pt;")

    def _z_setup_capture_top(self) -> None:
        ctrl = self._controller
        if ctrl is None or self._z_setup_bottom_raw is None:
            self.lbl_z_setup_status.setText("Capture the bottom first.")
            return
        raw_top = ctrl.capture_current_z_raw()
        if raw_top is None:
            self.lbl_z_setup_status.setText("Could not read Z.")
            return
        summary = ctrl.apply_z_setup(self._z_setup_bottom_raw, float(raw_top))
        # Reflect the derived envelope into the Z limit spinboxes (user frame).
        u_a = self._z_raw_to_user(ctrl.safety_limits.z_min)
        u_b = self._z_raw_to_user(ctrl.safety_limits.z_max)
        self.spin_z_min.setValue(min(u_a, u_b))
        self.spin_z_max.setValue(max(u_a, u_b))
        # Persist: datum (zero_position), envelope (safety_limits), direction.
        s = self._settings
        if s is not None:
            s.set_section("zero_position", ctrl.zero_position)
            s.set("safety_limits.z_min", ctrl.safety_limits.z_min)
            s.set("safety_limits.z_max", ctrl.safety_limits.z_max)
            s.set("device_profile.z_up_sign", ctrl.z_up_sign())
            s.save()
            try:
                self._persist_active_profile()
            except Exception:
                pass
        # v7.5.x: record the Z-axis calibration time for the usability pop-up.
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            get_store().mark_calibrated("z")
        except Exception:
            pass
        try:
            self._refresh_jog_positions()
        except Exception:
            pass
        if summary.get("direction_ok"):
            self.lbl_z_setup_status.setText(
                f"✅ Z setup complete. Up = {'+raw' if summary['z_up_sign'] > 0 else '−raw'} "
                f"(sign {summary['z_up_sign']:+.0f}); travel "
                f"{summary['travel_height_mm']:.2f} mm; user Z 0 → "
                f"{summary['travel_height_mm']:.2f}. Saved.")
            self.lbl_z_setup_status.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 9pt;")
        else:
            self.lbl_z_setup_status.setText(
                "⚠ Bottom and top are too close to tell the direction apart. "
                "Datum + limits were saved, but jog further apart and re-run "
                "so the up-direction is unambiguous.")
            self.lbl_z_setup_status.setStyleSheet(
                f"color: {COLORS['yellow']}; font-size: 9pt;")
        self._z_setup_bottom_raw = None
        self.btn_z_setup_top.setEnabled(False)

    # ── v7.5.x: per-pump Plunger Setup (syringe twin of the Z Axis Setup) ──
    #
    # ZERO = plunger ALL THE WAY IN (syringe empty) → "Set Dispensed" (fill 0).
    # MAX  = plunger ALL THE WAY OUT (syringe full)  → "Set Aspirated".
    # apply_pump_setup() captures both extremes, DERIVES the dispense/aspirate
    # direction (which it then OWNS, like z_up_sign owns Z), and sets the pump
    # soft-limit (calibration) extents. Each pump is its own block, placed in
    # order right beneath the Z block. ASPIRATE = draw fluid IN (toward full);
    # DISPENSE = push fluid OUT (toward empty). Jog the plunger to each extreme
    # with the jog pad first.

    def _build_pump_plunger_setup_block(self, pump: str) -> QGroupBox:
        """The per-pump plunger calibration block: jog the plunger to its
        mechanical extremes, capture them, and the controller derives the
        direction + soft-limit extents — the syringe-pump twin of the Z setup."""
        grp = QGroupBox(
            f"{pump} Plunger Setup — empty = 0, full = max, direction & limits")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        v = QVBoxLayout(grp)
        info = QLabel(
            "Use the jog pad to drive this pump's plunger to each mechanical "
            "extreme and capture it. <b>Dispensed = plunger all the way IN "
            "(syringe empty).</b> Clicking <b>Set Dispensed</b> ZEROES the pump "
            "here, so empty = position 0.0. Then jog ALL THE WAY OUT (full) and "
            "click <b>Set Aspirated</b> — full reads a positive fill. The "
            "<b>ASPIRATE</b> (draw-in) / <b>DISPENSE</b> (push-out) direction "
            "and the pump soft-limit extents are derived and saved "
            "automatically.")
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        v.addWidget(info)

        row = QHBoxLayout()
        btn_disp = QPushButton("1 · Set Dispensed (empty → zero)")
        btn_disp.setToolTip(
            "Jog this pump's plunger ALL THE WAY IN (syringe empty) first, then "
            "click. This ZEROES the pump's position here (fill = 0, the datum) "
            "so the full extreme will read positive.")
        btn_disp.clicked.connect(
            lambda _=False, p=pump: self._pump_setup_capture_dispensed(p))
        row.addWidget(btn_disp)
        btn_asp = QPushButton("2 · Set Aspirated (full)")
        btn_asp.setToolTip(
            "Jog the plunger ALL THE WAY OUT (syringe full), then click. The "
            "aspirate/dispense direction and the pump soft-limit extents are "
            "derived from dispensed → aspirated and saved.")
        btn_asp.setEnabled(False)
        btn_asp.clicked.connect(
            lambda _=False, p=pump: self._pump_setup_capture_aspirated(p))
        row.addWidget(btn_asp)
        row.addStretch(1)
        v.addLayout(row)

        status = QLabel("Plunger not zeroed yet — jog to empty and Set "
                        "Dispensed.")
        status.setWordWrap(True)
        status.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        v.addWidget(status)

        self._pump_setup_dispensed_btns[pump] = btn_disp
        self._pump_setup_aspirated_btns[pump] = btn_asp
        self._pump_setup_status_lbls[pump] = status
        self._pump_setup_dispensed_raw[pump] = None
        return grp

    def _pump_setup_capture_dispensed(self, pump: str) -> None:
        ctrl = self._controller
        status = self._pump_setup_status_lbls.get(pump)
        if ctrl is None or not hasattr(ctrl, "begin_pump_plunger_setup"):
            if status is not None:
                status.setText("No controller available.")
            return
        # The dispensed (empty) extreme is the "set zero" step: zero the pump's
        # Marlin counter HERE so the datum is genuinely raw 0.0 and the full
        # extreme reads a positive fill (G92 only rebases the counter, no move).
        result = ctrl.begin_pump_plunger_setup(pump)
        if not result.get("ok"):
            if status is not None:
                status.setText(
                    "Could not zero the pump position (is the ZP controller "
                    "connected?).")
            return
        self._pump_setup_dispensed_raw[pump] = 0.0
        self._pump_setup_persist_zero(pump)
        self._pump_setup_aspirated_btns[pump].setEnabled(True)
        if status is not None:
            prev = result.get("previous_raw")
            prev_txt = (f" (was raw {prev:.3f} mm)"
                        if isinstance(prev, (int, float)) else "")
            status.setText(
                f"{pump}: zeroed at the dispensed (empty) extreme → position "
                f"0.0{prev_txt}. Now jog the plunger ALL THE WAY OUT (full) and "
                f"click Set Aspirated — it will read a positive fill.")
            status.setStyleSheet(f"color: {COLORS['blue']}; font-size: 9pt;")

    def _pump_setup_capture_aspirated(self, pump: str) -> None:
        ctrl = self._controller
        status = self._pump_setup_status_lbls.get(pump)
        dispensed = self._pump_setup_dispensed_raw.get(pump)
        if ctrl is None or dispensed is None:
            if status is not None:
                status.setText("Capture the dispensed (empty) extreme first.")
            return
        raw_aspirated = ctrl.capture_current_pump_raw(pump)
        if raw_aspirated is None:
            if status is not None:
                status.setText("Could not read the pump position.")
            return
        # The aspirate/dispense direction is about to become authoritative —
        # confirm the operator really is at the full (all-the-way-OUT) extreme.
        resp = QMessageBox.question(
            self, "Confirm plunger setup",
            f"Set {pump} aspirated (FULL) extreme from the current position?\n\n"
            f"The ASPIRATE / DISPENSE direction and the soft-limit extents will "
            f"be derived from this and the dispensed extreme, and will OWN this "
            f"pump's direction from now on. Only proceed if the plunger is truly "
            f"at its mechanical extremes (empty → full).",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if resp != QMessageBox.Yes:
            if status is not None:
                status.setText(f"{pump}: setup cancelled.")
            return
        summary = ctrl.apply_pump_setup(pump, float(dispensed),
                                        float(raw_aspirated))
        self._pump_setup_persist(pump)
        cap = summary.get("capacity_uL")
        cap_txt = (f"{cap:.1f} µL" if cap is not None
                   else f"{summary.get('capacity_mm', 0.0):.2f} mm "
                        f"(no syringe configured)")
        if status is not None:
            if summary.get("direction_ok"):
                status.setText(
                    f"✅ {pump} plunger setup complete. Aspirate = "
                    f"{'+raw' if summary['aspirate_sign'] > 0 else '−raw'} "
                    f"(sign {summary['aspirate_sign']:+.0f}); capacity {cap_txt}; "
                    f"fill 0 (empty) → {cap_txt} (full). Limits saved.")
                status.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")
            else:
                status.setText(
                    f"⚠ {pump}: dispensed and aspirated are too close to tell "
                    f"the direction apart. Datum + limits were saved, but jog "
                    f"further apart and re-run so the direction is unambiguous.")
                status.setStyleSheet(
                    f"color: {COLORS['yellow']}; font-size: 9pt;")
        self._pump_setup_dispensed_raw[pump] = None
        self._pump_setup_aspirated_btns[pump].setEnabled(False)

    def _pump_setup_persist_zero(self, pump: str) -> None:
        """Persist just the re-zeroed datum after the 'Set Dispensed' step so the
        saved zero_position matches the firmware G92 even if the operator stops
        before capturing the aspirated extreme."""
        ctrl = self._controller
        s = self._settings
        if ctrl is None or s is None:
            return
        try:
            s.set_section("zero_position", ctrl.zero_position)
            s.save()
        except Exception as e:
            logger.warning(f"Pump plunger zero persist failed ({pump}): {e}")

    def _pump_setup_persist(self, pump: str) -> None:
        """Persist the datum + the derived pump soft-limit extents after a
        successful capture, mirror them into the visible limit spinboxes, and
        save the device profile (mirrors the Z setup persistence)."""
        ctrl = self._controller
        s = self._settings
        lo = hi = None
        if ctrl is not None:
            try:
                lo = getattr(ctrl.safety_limits, f"{pump.lower()}_min")
                hi = getattr(ctrl.safety_limits, f"{pump.lower()}_max")
            except AttributeError:
                lo = hi = None
        # Reflect the captured extents into the visible safety-limit spinboxes.
        # v7.5.x: the spinboxes are the user FILL frame (0 = empty → +full,
        # always positive); ``lo/hi`` are raw Marlin. Convert + assign min/max
        # by VALUE so a calibrated pump reads [0, +capacity] (the settings write
        # below keeps the raw envelope). ``settings`` stores raw regardless.
        if lo is not None and hi is not None and hasattr(self, "spin_p_mins") \
                and pump in self.spin_p_mins:
            _ua = self._pump_raw_to_user(pump, float(lo))
            _ub = self._pump_raw_to_user(pump, float(hi))
            self.spin_p_mins[pump].setValue(min(_ua, _ub))
            self.spin_p_maxs[pump].setValue(max(_ua, _ub))
        if ctrl is None or s is None:
            return
        try:
            s.set_section("zero_position", ctrl.zero_position)
            if lo is not None and hi is not None:
                s.set(f"safety_limits.{pump.lower()}_min", lo)
                s.set(f"safety_limits.{pump.lower()}_max", hi)
            if hasattr(ctrl, "get_pump_setup"):
                s.set("device_profile.pump_setup", ctrl.get_pump_setup())
            s.save()
        except Exception as e:
            logger.warning(f"Pump plunger setup persist failed ({pump}): {e}")
        # v7.5.x: record the pump (P) calibration time for the usability pop-up.
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            get_store().mark_calibrated("p")
        except Exception:
            pass
        try:
            self._persist_active_profile()
        except Exception:
            pass

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
        recorded_val: float | None = None
        # v7.5.x: XY *and* the Z/pump envelopes are ABSOLUTE (fixed mechanical
        # extents, independent of Set Zero), so store the raw absolute reading
        # for every axis — no `- zero`.
        if axis == "X" and xy and xy[0] is not None:
            target_spin = self.spin_xy_min_x if which == "min" else self.spin_xy_max_x
            recorded_val = float(xy[0])
            target_spin.setValue(recorded_val)
        elif axis == "Y" and xy and xy[1] is not None:
            target_spin = self.spin_xy_min_y if which == "min" else self.spin_xy_max_y
            recorded_val = float(xy[1])
            target_spin.setValue(recorded_val)
        elif axis == "Z":
            # v7.4.2 hotfix: route via the live axis_map so Z reads
            # from whatever physical Marlin axis is configured.
            v = self._logical_zp_value(zp, "Z")
            if v is not None:
                target_spin = self.spin_z_min if which == "min" else self.spin_z_max
                # v7.5.x: the Z limit spinboxes are the unified user frame
                # (0 at the bottom datum, up = +). Convert the raw Marlin
                # reading so "Set Max" at the top records the travel height
                # (not a negative). Bottom → spin_z_min (≈0), top → spin_z_max.
                recorded_val = self._z_raw_to_user(float(v))
                target_spin.setValue(recorded_val)
        elif axis in ("P1", "P2", "P3"):
            v = self._logical_zp_value(zp, axis)
            if v is not None:
                target_dict = self.spin_p_mins if which == "min" else self.spin_p_maxs
                # v7.5.x: the pump limit spinboxes are the user FILL frame (0 =
                # empty → +full, always positive), like Z. Convert the raw
                # Marlin reading so stamping at empty records ~0 and at full
                # records +capacity (not a negative). Stored back as raw on save.
                recorded_val = self._pump_raw_to_user(axis, float(v))
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
        # v7.5.x: spinboxes are the unified user frame (0 at bottom, up = +);
        # stored limits are absolute raw Marlin. Convert each raw bound to the
        # user frame and assign min/max by VALUE (polarity-general — no manual
        # swap, correct for either Z direction).
        _z_min_raw = float(s.get("safety_limits.z_min", -10.0))
        _z_max_raw = float(s.get("safety_limits.z_max", 50.0))
        _u_a = self._z_raw_to_user(_z_min_raw)
        _u_b = self._z_raw_to_user(_z_max_raw)
        self.spin_z_min.setValue(min(_u_a, _u_b))
        self.spin_z_max.setValue(max(_u_a, _u_b))
        # v7.5.x: standard plate offsets (needle-cam → plate features).
        _off = s.get("device_profile.plate_z_offsets") or {}
        if hasattr(self, "spin_zoff_top"):
            self.spin_zoff_top.setValue(float(_off.get("top", 10.0)))
            self.spin_zoff_bottom.setValue(float(_off.get("bottom", 20.0)))
            self.spin_zoff_safe.setValue(float(_off.get("safe", 5.0)))
        # v7.5.x: well-plate orientation (None ⇒ default ON for ME3B).
        if hasattr(self, "chk_plate_flip_180"):
            _flip = s.get("device_profile.plate_flip_180")
            self.chk_plate_flip_180.blockSignals(True)
            self.chk_plate_flip_180.setChecked(
                True if _flip is None else bool(_flip))
            self.chk_plate_flip_180.blockSignals(False)
        # v7.5.x: pump limit spinboxes are the user FILL frame (0 = empty →
        # +capacity = full, always positive), same as Z. Stored limits are
        # absolute raw Marlin (what the clamp uses); convert each raw bound to
        # the fill frame and assign min/max by VALUE (polarity-general — a
        # calibrated pump with aspirate_sign=-1 has raw_min↔user_max, so the
        # swap is essential to show [0, +cap] not [-cap, 0]).
        for pid in ("P1", "P2", "P3"):
            _raw_lo = float(s.get(f"safety_limits.{pid.lower()}_min", -50.0))
            _raw_hi = float(s.get(f"safety_limits.{pid.lower()}_max", 50.0))
            _ua = self._pump_raw_to_user(pid, _raw_lo)
            _ub = self._pump_raw_to_user(pid, _raw_hi)
            self.spin_p_mins[pid].setValue(min(_ua, _ub))
            self.spin_p_maxs[pid].setValue(max(_ua, _ub))
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
            # v7.5.x: restore the XY controller selection FIRST so the accel
            # range/labels match the model before the value is applied.
            if hasattr(self, 'cmb_xy_controller'):
                self._populate_xy_controller_combo()
                self._select_xy_controller(s.get("controller.controller_json"))
            self.spin_xy_velocity.setValue(
                float(s.get("device_profile.xy_velocity_pct") or 100))
            # v7.5.x: fall back to a family-aware default (Prior 50 / Ludl 1 —
            # set by _relabel_xy_cal_for_json, called just above via
            # _select_xy_controller) rather than a flat 50 for every family,
            # since 50/255 is nowhere near Ludl's bench-confirmed "low = fast"
            # sweet spot.
            fallback_accel = getattr(self, "_xy_accel_family_default", 50)
            self.spin_xy_acceleration.setValue(
                float(s.get("device_profile.xy_acceleration") or fallback_accel))
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
        # v7.4.2: also persist to the active profile JSON so the change
        # survives a profile switch round-trip.
        self._persist_active_profile()
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
        """Save the ZP calibration section: M92 + M203 + M201 + persist."""
        if self._settings is None:
            return
        # Read the current per-axis feedrate ceilings out of settings so
        # Save Calibration also re-aligns Marlin's M203 with software.
        per_axis_feed = ((self._settings.get(
            "device_profile.per_axis_max_feedrate")) or {})
        if self._controller is not None and self._controller.zp_stage is not None:
            steps = dict(self._controller.zp_stage.steps_per_mm)
            self._settings.set("device_profile.steps_per_mm", steps)
            # Re-send M92 so Marlin matches our stored steps/mm
            self._controller.zp_stage.set_steps_per_mm(steps, persist=True)
            # v7.4.2 hotfix: push per-axis M203 so Marlin matches the
            # per_axis_max_feedrate ceilings stored on the device profile.
            if per_axis_feed:
                try:
                    self._controller.zp_stage.set_per_axis_max_feedrate(
                        per_axis_feed, persist=True)
                except Exception as e:
                    logger.warning(f"M203 send failed: {e}")
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
        # v7.4.2: also persist to the active profile JSON so the change
        # survives a profile switch round-trip.
        self._persist_active_profile()
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
                "and per_axis_max_accel persisted; M92 + M203 + M201 sent "
                "to Marlin.")
        logger.info("ZP calibration saved (steps + feedrate + accel)")

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
                    f"color: {COLORS['red']}; ")
            else:
                self.lbl_alignment_status.setText("✓ Marlin matches software.")
                self.lbl_alignment_status.setStyleSheet(
                    f"color: {COLORS['green']}; ")

    def _load_calibration_from_hardware(self) -> None:
        """Query Marlin (M503) and ADOPT its reported steps_per_mm,
        per-axis max feedrate, and per-axis max acceleration into
        software — the mirror of :meth:`_apply_steps_cal` (which pushes
        software → hardware via M92/M203/M201).

        Use this when the board already carries the correct calibration
        (freshly tuned in firmware, or re-flashed with known-good values)
        and the software side has drifted and should follow the board
        instead of overwriting it.

        No G-code is re-sent — the values already live on the board;
        this only updates ``StageController``/settings.json/the active
        profile and the on-screen grids.
        """
        if self._settings is None:
            return
        if (self._controller is None or self._controller.zp_stage is None
                or self._controller.simulate_zp):
            if hasattr(self, 'lbl_alignment_status'):
                self.lbl_alignment_status.setText(
                    "Connect ZP (real hardware) to load values from hardware.")
            return
        zp = self._controller.zp_stage
        try:
            reported = zp.query_settings()
        except Exception as e:
            if hasattr(self, 'lbl_alignment_status'):
                self.lbl_alignment_status.setText(f"M503 query failed: {e}")
            return

        hw_steps = reported.get("steps_per_mm") or {}
        hw_feed = reported.get("max_feedrate") or {}
        hw_accel = reported.get("max_accel") or {}
        if not (hw_steps or hw_feed or hw_accel):
            if hasattr(self, 'lbl_alignment_status'):
                self.lbl_alignment_status.setText(
                    "M503 returned nothing usable — check the serial "
                    "connection and try again.")
            return

        axis_map = zp.axis_map
        new_steps = dict(zp.steps_per_mm)
        new_feed = dict(
            self._settings.get("device_profile.per_axis_max_feedrate") or {})
        new_accel = dict(
            self._settings.get("device_profile.per_axis_max_accel") or {})
        loaded: list[str] = []
        for logical, physical in axis_map.items():
            if physical in hw_steps:
                new_steps[logical] = hw_steps[physical]
                loaded.append(f"{logical} steps/mm")
            if physical in hw_feed:
                new_feed[logical] = hw_feed[physical]
                loaded.append(f"{logical} feedrate")
            if physical in hw_accel:
                new_accel[logical] = hw_accel[physical]
                loaded.append(f"{logical} accel")

        if not loaded:
            if hasattr(self, 'lbl_alignment_status'):
                self.lbl_alignment_status.setText(
                    "M503 didn't report any axis matching this device's "
                    "axis mapping — nothing to load.")
            return

        # Adopt locally only (persist=False) — these values already live
        # on the board, so there is nothing to re-send it.
        zp.set_steps_per_mm(new_steps, persist=False)
        zp.set_per_axis_max_feedrate(new_feed, persist=False)
        zp.set_axis_accelerations(new_accel, persist=False)

        self._settings.set("device_profile.steps_per_mm", new_steps)
        self._settings.set("device_profile.per_axis_max_feedrate", new_feed)
        self._settings.set("device_profile.per_axis_max_accel", new_accel)
        self._settings.save()
        # v7.4.2: also persist to the active profile JSON so the change
        # survives a profile switch round-trip.
        self._persist_active_profile()

        # Reflect the loaded acceleration values in the visible spinboxes.
        if hasattr(self, 'spin_axis_accel'):
            for ax, sp_w in self.spin_axis_accel.items():
                if ax in new_accel:
                    sp_w.blockSignals(True)
                    sp_w.setValue(float(new_accel[ax]))
                    sp_w.blockSignals(False)

        self._refresh_steps_grid()
        self._refresh_max_feedrate_grid()
        try:
            self._check_marlin_alignment(quiet=True)
        except Exception:
            pass

        if hasattr(self, 'lbl_cal_status'):
            self.lbl_cal_status.setText(
                "Loaded from hardware: steps_per_mm, per_axis_max_feedrate, "
                "and per_axis_max_accel adopted from Marlin's M503 report "
                "(nothing re-sent to the board).")
        logger.info(f"ZP calibration loaded from hardware (M503): {loaded}")

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
        # v7.5.x: spinboxes are the unified user frame (0 at bottom, up = +);
        # store absolute raw Marlin. Convert both bounds and assign min/max by
        # VALUE (polarity-general → z_min < z_max always, never a frozen
        # envelope, for either Z direction).
        _zr_a = self._z_user_to_raw(self.spin_z_min.value())
        _zr_b = self._z_user_to_raw(self.spin_z_max.value())
        _z_min_raw_out = min(_zr_a, _zr_b)
        _z_max_raw_out = max(_zr_a, _zr_b)
        s.set("safety_limits.z_min", _z_min_raw_out)
        s.set("safety_limits.z_max", _z_max_raw_out)
        # v7.5.x: fill-frame spinboxes → absolute raw Marlin (min/max by VALUE,
        # polarity-general), so the clamp keeps its raw envelope.
        _p_raw = {}
        for pid in ("P1", "P2", "P3"):
            _ra = self._pump_user_to_raw(pid, self.spin_p_mins[pid].value())
            _rb = self._pump_user_to_raw(pid, self.spin_p_maxs[pid].value())
            _p_raw[pid] = (min(_ra, _rb), max(_ra, _rb))
            s.set(f"safety_limits.{pid.lower()}_min", _p_raw[pid][0])
            s.set(f"safety_limits.{pid.lower()}_max", _p_raw[pid][1])
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
            # v7.5.x: user-frame spinboxes → absolute raw Marlin (computed
            # min/max above; polarity-general).
            sl.z_min = _z_min_raw_out
            sl.z_max = _z_max_raw_out
            for pid in ("P1", "P2", "P3"):
                pid_l = pid.lower()
                if hasattr(sl, f"{pid_l}_min"):
                    # v7.5.x: use the raw-converted bounds computed above so the
                    # live clamp envelope stays in the raw Marlin frame.
                    setattr(sl, f"{pid_l}_min", _p_raw[pid][0])
                    setattr(sl, f"{pid_l}_max", _p_raw[pid][1])
            sl.max_xy_speed = self.spin_max_xy_speed.value()
            sl.max_z_feedrate = self.spin_max_z_feed.value()
            sl.max_pump_feedrate = self.spin_max_pump_feed.value()
            # Persist current zero_position snapshot too
            s.set_section("zero_position", ctrl.zero_position)
        s.save()
        # v7.4.2: also persist to the active profile JSON.
        self._persist_active_profile()
        if hasattr(self, 'lbl_jog_status'):
            self.lbl_jog_status.setText(
                "Safety limits + zero positions saved to settings and "
                "applied to the live controller.")
        logger.info("Safety limits + zero saved")
        # v7.4.2: nudge the persistent left-panel position bars so the
        # slider extents track the user's new envelope.
        self._notify_control_panel_safety_changed()

    def _notify_control_panel_safety_changed(self) -> None:
        page = self.parent()
        while page is not None and not hasattr(page, "_control_panel"):
            page = page.parent() if hasattr(page, "parent") else None
        if page is not None:
            panel = getattr(page, "_control_panel", None)
            if panel is not None and hasattr(panel, "refresh_safety_limits"):
                try:
                    panel.refresh_safety_limits()
                except Exception:
                    pass
            # v7.5.x: also tell the app the envelope changed so it can
            # re-centre the default (uncalibrated) plate live. The walk
            # above already located the HardwareSetupPage, which owns the
            # safety_limits_changed signal.
            sig = getattr(page, "safety_limits_changed", None)
            if sig is not None:
                try:
                    sig.emit()
                except Exception:
                    pass

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
        # v7.4.2 hotfix: send per-axis M203 from the per_axis_max_feedrate
        # ceilings instead of broadcasting the Z-derived max_mm. The
        # percentage workflow controls travel/retract/insert/jog speeds;
        # the M203 ceiling is the per-axis raw value the user dialled in
        # via Stepper Calibration / popup edits.
        per_axis_feed = (s.get("device_profile.per_axis_max_feedrate") or {})
        if ctrl is not None:
            try:
                if ctrl.zp_stage is not None:
                    if per_axis_feed and hasattr(
                            ctrl.zp_stage, 'set_per_axis_max_feedrate'):
                        ctrl.zp_stage.set_per_axis_max_feedrate(
                            per_axis_feed, persist=True)
                    elif hasattr(ctrl.zp_stage, 'set_max_feedrate'):
                        ctrl.zp_stage.set_max_feedrate(max_mm)
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
        # v7.4.2: also persist to the active profile JSON.
        self._persist_active_profile()
        # v7.4.2 hotfix: re-check alignment after pushing M203
        try:
            self._check_marlin_alignment(quiet=True)
        except Exception:
            pass
        if hasattr(self, 'lbl_jog_status'):
            self.lbl_jog_status.setText(
                f"ZP feedrates saved: max {max_mm:.0f}, retract {ret_mm:.0f}, "
                f"insert {ins_mm:.0f}, jog {jog_mm:.0f} mm/min "
                f"(per-axis M203 sent to Marlin)")
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
        # v7.5.x: spinboxes are the unified user frame (0 at bottom, up = +);
        # store absolute raw Marlin. Convert both bounds and assign min/max by
        # VALUE (polarity-general → z_min < z_max always, never a frozen
        # envelope, for either Z direction).
        _zr_a = self._z_user_to_raw(self.spin_z_min.value())
        _zr_b = self._z_user_to_raw(self.spin_z_max.value())
        _z_min_raw_out = min(_zr_a, _zr_b)
        _z_max_raw_out = max(_zr_a, _zr_b)
        s.set("safety_limits.z_min", _z_min_raw_out)
        s.set("safety_limits.z_max", _z_max_raw_out)
        # v7.5.x: fill-frame spinboxes → absolute raw Marlin (min/max by VALUE).
        for pid in ("P1", "P2", "P3"):
            _ra = self._pump_user_to_raw(pid, self.spin_p_mins[pid].value())
            _rb = self._pump_user_to_raw(pid, self.spin_p_maxs[pid].value())
            s.set(f"safety_limits.{pid.lower()}_min", min(_ra, _rb))
            s.set(f"safety_limits.{pid.lower()}_max", max(_ra, _rb))
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
        # v7.4.2: also persist to the active profile JSON so the change
        # survives a profile switch round-trip.
        self._persist_active_profile()
        self.settings_applied.emit()
        logger.info("Stage hardware settings applied")

    def _reset_defaults(self):
        """v7.4.1: Reset to canonical defaults (matches Standard.json profile)."""
        self.chk_safety_enabled.setChecked(True)
        self.spin_xy_min_x.setValue(-130000.0)
        self.spin_xy_max_x.setValue(130000.0)
        self.spin_xy_min_y.setValue(-85000.0)
        self.spin_xy_max_y.setValue(85000.0)
        # v7.5.x: spinboxes are the unified user frame (0 at bottom, up = +);
        # show the raw defaults (z_min=-10, z_max=50) converted to user-Z by
        # value (min/max), polarity-general.
        _u_a = self._z_raw_to_user(-10.0)
        _u_b = self._z_raw_to_user(50.0)
        self.spin_z_min.setValue(min(_u_a, _u_b))
        self.spin_z_max.setValue(max(_u_a, _u_b))
        # v7.5.x: pump spinboxes are the user FILL frame; show the raw ±50
        # defaults converted by value (no-op for the symmetric default, but
        # correct + consistent with Z on either polarity).
        for pid in ("P1", "P2", "P3"):
            _pa = self._pump_raw_to_user(pid, -50.0)
            _pb = self._pump_raw_to_user(pid, 50.0)
            self.spin_p_mins[pid].setValue(min(_pa, _pb))
            self.spin_p_maxs[pid].setValue(max(_pa, _pb))
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
        previous = ""
        if self._settings is not None:
            previous = (self._settings.get("device_profile.active") or "").strip()
            profile.apply_to_settings(self._settings)
            self._settings.set("device_profile.active", profile.profile_name)
            self._settings.save()
            # v7.5.x: the profile may select a per-machine XY controller (written
            # into controller.controller_json by apply_to_settings). Push it so
            # the next XY connect uses the right protocol (Prior / Ludl).
            ctrl = self._controller
            if ctrl is not None and hasattr(ctrl, "set_controller_json"):
                ctrl.set_controller_json(
                    self._settings.get("controller.controller_json"))
        # Refresh widget values from the (now updated) settings
        self._load_from_settings()
        self.lbl_profile_status.setText(
            f"Loaded profile: {profile.profile_name}")
        self.settings_applied.emit()
        logger.info(f"Loaded device profile: {profile.profile_name}")
        # Loading a DIFFERENT profile means "this is another machine", which
        # re-points every per-machine config folder — but those paths were
        # resolved at import time, so a restart is required for it to take.
        if previous and previous != profile.profile_name:
            self._notify_identity_change(profile.profile_name)

    def _persist_active_profile(self) -> bool:
        """v7.4.2: Snapshot current settings into the active profile JSON
        on disk so the calibration survives a profile switch round-trip.

        Without this, popup edits + per-section Save buttons only updated
        settings.json — the on-disk profile file was stale, so switching
        profiles and switching back lost the user's edits.

        Returns True on success, False if no active profile is set or the
        write failed (silently — caller already showed the section's
        own status message).
        """
        from pathlib import Path
        if self._settings is None:
            return False
        active = self._settings.get("device_profile.active") or ""
        if not active or active not in self._profile_paths:
            return False
        try:
            profile = DeviceProfile.from_settings(self._settings, name=active)
            profile.save(Path(self._profile_paths[active]))
            logger.info(
                f"Active device profile auto-saved to disk: {active}")
            return True
        except Exception as e:
            logger.warning(f"Active profile auto-save failed: {e}")
            return False

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
        """Prompt for a name and save a new profile.

        ⚠ v7.17.x: the profile name IS this machine's identity — it names
        ``config/hardware/<name>/``, where every per-machine calibration
        lives (see ``SupportClasses/MachineConfig.py``). So a Save-As is
        either a RENAME of this rig, in which case the calibration must
        follow, or a genuinely DIFFERENT machine, in which case it must not.
        Only the operator knows which, so ask instead of guessing: guessing
        wrong either strands a taught plate map in a folder nothing reads, or
        silently hands one rig another rig's calibration.
        """
        from SupportClasses import MachineConfig as mc
        name, ok = QInputDialog.getText(
            self, "Save Device Profile As",
            "Machine / profile name (this names its calibration folder):")
        if not ok or not name.strip():
            return
        name = name.strip()
        valid, why = mc.is_valid_machine_name(name)
        if not valid:
            QMessageBox.warning(self, "Save Device Profile As", why)
            return
        self._apply()  # Snapshot current widget values to Settings
        if self._settings is None:
            return
        previous = (self._settings.get("device_profile.active") or "").strip()
        profile = DeviceProfile.from_settings(self._settings, name=name)
        try:
            path = profile.save()  # Default location in DEVICES_DIR
        except Exception as e:
            QMessageBox.warning(self, "Save Device Profile",
                                f"Failed to save {name}: {e}")
            return
        self._maybe_move_machine_folder(previous, name)
        self._settings.set("device_profile.active", name)
        self._settings.save()
        self._notify_identity_change(name)
        self._refresh_profile_list()
        idx = self.cmb_profile.findText(name)
        if idx >= 0:
            self.cmb_profile.setCurrentIndex(idx)
        self.lbl_profile_status.setText(f"Saved new profile: {name}")
        logger.info(f"Saved new device profile {name} → {path}")

    def _maybe_move_machine_folder(self, previous: str, new: str) -> None:
        """Offer to carry this machine's calibration to a renamed profile.

        Only asked when it is actually ambiguous: the old folder exists and
        the new one does not. If the new name already has a folder, the
        operator is switching back to a machine that already has calibration
        — nothing to move, and merging would be a guess.
        """
        from SupportClasses import MachineConfig as mc
        if not previous or previous == new:
            return
        src, dst = mc.HARDWARE_ROOT / previous, mc.HARDWARE_ROOT / new
        if not src.is_dir() or dst.exists():
            return
        reply = QMessageBox.question(
            self, "Is this the same machine?",
            f"'{previous}' has a calibration folder "
            f"(taught plate map, camera calibration, mosaics …).\n\n"
            f"Yes — this is the SAME machine being renamed to '{new}': move "
            f"its calibration across.\n\n"
            f"No — '{new}' is a DIFFERENT machine: start it with fresh "
            f"calibration and leave '{previous}' untouched.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if mc.rename_machine_folder(previous, new):
                logger.info(f"Machine calibration moved {previous} → {new}")
            else:
                QMessageBox.warning(
                    self, "Could not move calibration",
                    f"'{previous}' could not be moved to '{new}' — see the log. "
                    f"Nothing was deleted; move the folder by hand if needed.")

    def _notify_identity_change(self, name: str) -> None:
        """Tell the operator a restart is needed to re-point the config folder.

        Every per-machine store resolves its path once, at module-import
        time, so switching identity mid-session leaves them reading the
        PREVIOUS machine's folder. Saying nothing would look like the new
        machine had inherited the old one's calibration.
        """
        QMessageBox.information(
            self, "Machine identity changed",
            f"This machine is now '{name}'.\n\n"
            f"Its calibration lives in config/hardware/{name}/.\n\n"
            f"Restart MEBP so every calibration store reads from there — "
            f"until you do, they are still using the previous machine's "
            f"folder.")

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
