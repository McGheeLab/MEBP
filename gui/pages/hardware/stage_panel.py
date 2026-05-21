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
        self._setup_ui()

    # ── Public injection points ──────────────────────────────────

    def set_settings(self, settings):
        """Inject the Settings instance. Loads initial values into widgets."""
        self._settings = settings
        if settings is not None:
            self._load_from_settings()

    def set_controller(self, controller):
        """Inject StageController for the zero-calibration jog row."""
        self._controller = controller

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

        outer.addWidget(self._build_safety_group())
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
