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
    QSizePolicy,
)
from PySide6.QtCore import Qt, Signal

from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.scaling import s, sf, sp, scaled_font_size

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
            "Stage hardware settings — safety envelope, axis direction, "
            "feedrate ceilings. Moved here from Settings in v7.4.0-b."
        )
        banner.setWordWrap(True)
        banner.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
            f"padding: {sp(4)} {sp(8)};"
            f"border-left: 2px solid {COLORS['mauve']};"
        )
        outer.addWidget(banner)

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

    def _build_safety_group(self) -> QGroupBox:
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
            sp_w.setMinimumWidth(s(140))
            return sp_w

        row = 0
        form.addWidget(QLabel("Enable safety limits:"), row, 0)
        self.chk_safety_enabled = QCheckBox()
        self.chk_safety_enabled.setChecked(True)
        form.addWidget(self.chk_safety_enabled, row, 1)

        row += 1
        form.addWidget(QLabel("XY min (µm):"), row, 0)
        self.spin_xy_min = _spin(-50000.0, "µm")
        form.addWidget(self.spin_xy_min, row, 1)
        form.addWidget(QLabel("XY max (µm):"), row, 2)
        self.spin_xy_max = _spin(50000.0, "µm")
        form.addWidget(self.spin_xy_max, row, 3)

        row += 1
        form.addWidget(QLabel("Z min (mm):"), row, 0)
        self.spin_z_min = _spin(-50.0, "mm", decimals=3)
        form.addWidget(self.spin_z_min, row, 1)
        form.addWidget(QLabel("Z max (mm):"), row, 2)
        self.spin_z_max = _spin(0.0, "mm", decimals=3)
        form.addWidget(self.spin_z_max, row, 3)

        row += 1
        form.addWidget(QLabel("Pump min (mm):"), row, 0)
        self.spin_p_min = _spin(-50.0, "mm", decimals=3)
        form.addWidget(self.spin_p_min, row, 1)
        form.addWidget(QLabel("Pump max (mm):"), row, 2)
        self.spin_p_max = _spin(50.0, "mm", decimals=3)
        form.addWidget(self.spin_p_max, row, 3)

        row += 1
        form.addWidget(QLabel("Max XY feedrate (µm/s):"), row, 0)
        self.spin_xy_feed = _spin(20000.0, "µm/s")
        form.addWidget(self.spin_xy_feed, row, 1)
        form.addWidget(QLabel("Max Z feedrate (mm/s):"), row, 2)
        self.spin_z_feed = _spin(5.0, "mm/s", decimals=2)
        form.addWidget(self.spin_z_feed, row, 3)

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
        s = self._settings
        if s is None:
            return
        # Safety
        self.chk_safety_enabled.setChecked(
            bool(s.get("safety_limits.enabled", True)))
        self.spin_xy_min.setValue(float(s.get("safety_limits.xy_min_um", -50000.0)))
        self.spin_xy_max.setValue(float(s.get("safety_limits.xy_max_um", 50000.0)))
        self.spin_z_min.setValue(float(s.get("safety_limits.z_min_mm", -50.0)))
        self.spin_z_max.setValue(float(s.get("safety_limits.z_max_mm", 0.0)))
        self.spin_p_min.setValue(float(s.get("safety_limits.p_min_mm", -50.0)))
        self.spin_p_max.setValue(float(s.get("safety_limits.p_max_mm", 50.0)))
        self.spin_xy_feed.setValue(float(s.get("safety_limits.xy_max_feed_um_s", 20000.0)))
        self.spin_z_feed.setValue(float(s.get("safety_limits.z_max_feed_mm_s", 5.0)))

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

    def _apply(self):
        """Write current widget values to Settings."""
        s = self._settings
        if s is None:
            logger.warning("StageHardwarePanel: no Settings instance to apply to")
            return
        s.set("safety_limits.enabled", self.chk_safety_enabled.isChecked())
        s.set("safety_limits.xy_min_um", self.spin_xy_min.value())
        s.set("safety_limits.xy_max_um", self.spin_xy_max.value())
        s.set("safety_limits.z_min_mm", self.spin_z_min.value())
        s.set("safety_limits.z_max_mm", self.spin_z_max.value())
        s.set("safety_limits.p_min_mm", self.spin_p_min.value())
        s.set("safety_limits.p_max_mm", self.spin_p_max.value())
        s.set("safety_limits.xy_max_feed_um_s", self.spin_xy_feed.value())
        s.set("safety_limits.z_max_feed_mm_s", self.spin_z_feed.value())

        s.set("zp_stage.max_feedrate", self.spin_zp_max_feed.value())
        s.set("zp_stage.retract_feedrate", self.spin_zp_retract_feed.value())
        s.set("zp_stage.insert_feedrate", self.spin_zp_insert_feed.value())
        s.set("zp_stage.jog_feedrate", self.spin_zp_jog_feed.value())
        s.set("zp_stage.auto_save_position", self.chk_zp_autosave.isChecked())

        s.set("axis_flip.z", self.chk_flip_z.isChecked())
        s.set("axis_flip.p1", self.chk_flip_p1.isChecked())
        s.set("axis_flip.p2", self.chk_flip_p2.isChecked())
        s.set("axis_flip.p3", self.chk_flip_p3.isChecked())

        self.settings_applied.emit()
        logger.info("Stage hardware settings applied")

    def _reset_defaults(self):
        """Reset all Stage widgets to factory defaults."""
        self.chk_safety_enabled.setChecked(True)
        self.spin_xy_min.setValue(-50000.0)
        self.spin_xy_max.setValue(50000.0)
        self.spin_z_min.setValue(-50.0)
        self.spin_z_max.setValue(0.0)
        self.spin_p_min.setValue(-50.0)
        self.spin_p_max.setValue(50.0)
        self.spin_xy_feed.setValue(20000.0)
        self.spin_z_feed.setValue(5.0)
        self.spin_zp_max_feed.setValue(3000.0)
        self.spin_zp_retract_feed.setValue(1500.0)
        self.spin_zp_insert_feed.setValue(600.0)
        self.spin_zp_jog_feed.setValue(900.0)
        self.chk_zp_autosave.setChecked(False)
        self.chk_flip_z.setChecked(False)
        self.chk_flip_p1.setChecked(False)
        self.chk_flip_p2.setChecked(False)
        self.chk_flip_p3.setChecked(False)
