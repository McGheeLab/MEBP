"""
Jog Control Page — Manual movement with compact layout.

Main content: XY pad, Z/pump buttons, position readouts, quick actions
Context panel: step sizes, speed multipliers

v7.1.1: XY step sizes and positions displayed in microns (µm).
v7.3.5: ProScan speaks µm natively — microsteps_per_micron = 1.0 (identity).

v7.1.2: BUG-1 FIX — XY jog now uses relative moves (move_xy_relative) instead
         of computing absolute targets from cached (stale) position data.
         This eliminates the "position update not consistent with jog command"
         bug where stale cached positions caused incorrect move targets.
"""

from __future__ import annotations

import logging
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QSlider, QFrame, QSizePolicy,
    QScrollArea, QDoubleSpinBox, QCheckBox,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QKeyEvent

from SupportClasses.StageController import StageController
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.scaling import s as _sc, sf as _sf, sp as _sp, scaled_font_size

# v7.3.1: Optional well plate navigator
try:
    from gui.widgets.jog_well_plate import WellPlateNavigator
    WELL_NAV_AVAILABLE = True
except ImportError:
    WellPlateNavigator = None
    WELL_NAV_AVAILABLE = False

logger = logging.getLogger(__name__)

# Step sizes in microns (µm) for the XY stage
XY_STEPS_UM = [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]

# Z and pump step sizes remain in mm
Z_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]
# v7.2.5: Pump steps in µL (converted to mm via HardwareConfig)
# Fallback: used as mm if no HW config available
P_STEPS_UL = [0.1, 0.5, 1.0, 5.0, 10.0, 50.0]
# v7.2.5: Pump steps in µL (converted to mm via HardwareConfig)
# Fallback: used as mm if no HW config available
P_STEPS_UL = [0.1, 0.5, 1.0, 5.0, 10.0, 50.0]
P_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]  # legacy mm fallback  # legacy mm fallback


class JogControlPage(QWidget):
    """Manual jog controls with fixed-step moves."""

    _page_title_text = "Jog Control"

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._context_widget = None

        # v7.2.5: Hardware config for pump enable/disable and µL display
        self._hardware_config = None

        # Microsteps per micron — set by MainWindow (ProScan speaks µm natively → 1.0)
        from gui.unit_helpers import DEFAULT_XY_POSITION_SCALE
        self._xy_position_scale: float = DEFAULT_XY_POSITION_SCALE

        # v7.2.4: Step verification tracking
        self._last_jog_step_um: float = 0.0
        self._last_xy_before: tuple = (None, None)
        self._conversion_factor_set: bool = False  # True once protocol loads

        # v7.3.1: Well plate navigation state
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None
        self._plate = None
        self._well_nav: WellPlateNavigator | None = None

        self._setup_ui()
        self._setup_shortcuts()

    def get_page_title(self) -> str:
        return "Jog Control"

    def set_xy_position_scale(self, value: float):
        """Called by MainWindow when the XY position scale factor changes.

        v7.2.4: Also updates the scale display and hides
        the warning banner once a real value is set.
        """
        self._xy_position_scale = max(0.001, value)
        self._conversion_factor_set = True
        # Update context panel labels if they exist
        if hasattr(self, '_lbl_conversion_factor'):
            self._lbl_conversion_factor.setText(
                f"Scale: {self._xy_position_scale:.1f} units/µm")
        if hasattr(self, '_lbl_factor_warning'):
            self._lbl_factor_warning.setVisible(False)
        logger.info(f"Jog: xy_position_scale set to {value}")

    def get_context_widget(self) -> QWidget:
        """Context panel: step sizes + speed multipliers."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        # ── Step Sizes ───────────────────────────────────────────
        step_label = QLabel("Step Sizes")
        step_label.setObjectName("contextSectionLabel")
        layout.addWidget(step_label)

        # XY step (in µm) — v7.3.2: editable with custom option
        xy_row = QHBoxLayout()
        xy_row.addWidget(QLabel("XY:"))
        self.xy_step_combo = QComboBox()
        for s in XY_STEPS_UM:
            if s >= 1.0:
                self.xy_step_combo.addItem(f"{s:g} µm", s)
            else:
                self.xy_step_combo.addItem(f"{s:.2f} µm", s)
        self.xy_step_combo.addItem("Custom…", "custom")
        self.xy_step_combo.setCurrentIndex(3)  # Default: 50 µm
        self.xy_step_combo.currentIndexChanged.connect(self._on_xy_step_changed)
        xy_row.addWidget(self.xy_step_combo, stretch=1)
        layout.addLayout(xy_row)
        # Custom XY spinbox (hidden until "Custom…" selected)
        self._xy_custom_spin = QDoubleSpinBox()
        self._xy_custom_spin.setRange(0.1, 50000.0)
        self._xy_custom_spin.setDecimals(1)
        self._xy_custom_spin.setSuffix(" µm")
        self._xy_custom_spin.setValue(50.0)
        self._xy_custom_spin.setVisible(False)
        layout.addWidget(self._xy_custom_spin)

        # Z step (in mm) — v7.3.2: editable with custom option
        z_row = QHBoxLayout()
        z_row.addWidget(QLabel("Z:"))
        self.z_step_combo = QComboBox()
        for s in Z_STEPS:
            self.z_step_combo.addItem(f"{s} mm", s)
        self.z_step_combo.addItem("Custom…", "custom")
        self.z_step_combo.setCurrentIndex(2)
        self.z_step_combo.currentIndexChanged.connect(self._on_z_step_changed)
        z_row.addWidget(self.z_step_combo, stretch=1)
        layout.addLayout(z_row)
        # Custom Z spinbox
        self._z_custom_spin = QDoubleSpinBox()
        self._z_custom_spin.setRange(0.001, 50.0)
        self._z_custom_spin.setDecimals(3)
        self._z_custom_spin.setSuffix(" mm")
        self._z_custom_spin.setValue(0.1)
        self._z_custom_spin.setVisible(False)
        layout.addWidget(self._z_custom_spin)

        # Pump step (in mm) — v7.3.2: editable with custom option
        p_row = QHBoxLayout()
        p_row.addWidget(QLabel("Pump:"))
        self.p_step_combo = QComboBox()
        for s in P_STEPS:
            self.p_step_combo.addItem(f"{s} mm", s)
        self.p_step_combo.addItem("Custom…", "custom")
        self.p_step_combo.setCurrentIndex(2)
        self.p_step_combo.currentIndexChanged.connect(self._on_p_step_changed)
        p_row.addWidget(self.p_step_combo, stretch=1)
        layout.addLayout(p_row)
        # Custom pump spinbox
        self._p_custom_spin = QDoubleSpinBox()
        self._p_custom_spin.setRange(0.001, 100.0)
        self._p_custom_spin.setDecimals(3)
        self._p_custom_spin.setSuffix(" mm")
        self._p_custom_spin.setValue(0.1)
        self._p_custom_spin.setVisible(False)
        layout.addWidget(self._p_custom_spin)

        # ── Absolute Go To ────────────────────────────────────────
        goto_label = QLabel("Absolute Go To")
        goto_label.setObjectName("contextSectionLabel")
        layout.addWidget(goto_label)

        goto_grid = QGridLayout()
        goto_grid.setSpacing(4)
        goto_grid.addWidget(QLabel("X (µm):"), 0, 0)
        self._goto_x = QDoubleSpinBox()
        self._goto_x.setRange(-999999, 999999)
        self._goto_x.setDecimals(1)
        self._goto_x.setValue(0.0)
        goto_grid.addWidget(self._goto_x, 0, 1)
        goto_grid.addWidget(QLabel("Y (µm):"), 1, 0)
        self._goto_y = QDoubleSpinBox()
        self._goto_y.setRange(-999999, 999999)
        self._goto_y.setDecimals(1)
        self._goto_y.setValue(0.0)
        goto_grid.addWidget(self._goto_y, 1, 1)
        goto_grid.addWidget(QLabel("Z (mm):"), 2, 0)
        self._goto_z = QDoubleSpinBox()
        self._goto_z.setRange(-100.0, 100.0)
        self._goto_z.setDecimals(3)
        self._goto_z.setValue(0.0)
        goto_grid.addWidget(self._goto_z, 2, 1)
        layout.addLayout(goto_grid)

        self._goto_safe = QCheckBox("Safe Travel")
        self._goto_safe.setChecked(True)
        self._goto_safe.setToolTip(
            "Raise Z to safe height before XY move, then lower")
        layout.addWidget(self._goto_safe)

        btn_goto = QPushButton("Go To")
        btn_goto.setStyleSheet(
            f"background-color: {COLORS['blue']}; color: {COLORS['crust']}; "
            f"font-weight: bold; padding: {_sp(4)} {_sp(12)};")
        btn_goto.setMaximumHeight(_sc(28))
        btn_goto.clicked.connect(self._absolute_goto)
        layout.addWidget(btn_goto)

        # ── Speed Multipliers ────────────────────────────────────
        # v7.4.2 hotfix: this slider now drives BOTH the Xbox continuous
        # jog loop AND the button-click jog feedrate. Before the fix the
        # slider only affected Xbox jog; button clicks always used
        # ZPStageManager's default 200 mm/min regardless.
        speed_label = QLabel("Speed")
        speed_label.setObjectName("contextSectionLabel")
        layout.addWidget(speed_label)

        # XY speed
        self.lbl_xy_speed = QLabel("100")
        # v7.2.6: S6-C slider refs
        layout.addWidget(self._make_ctx_slider(
            "XY:", self.lbl_xy_speed, 1, 10000, 100, self._on_xy_speed,
            store_as="_sld_xy_speed"))

        # Z speed
        self.lbl_z_speed = QLabel("0.50")
        layout.addWidget(self._make_ctx_slider(
            "Z:", self.lbl_z_speed, 1, 500, 50, self._on_z_speed,
            store_as="_sld_z_speed"))

        # Pump speed
        self.lbl_p_speed = QLabel("0.50")
        layout.addWidget(self._make_ctx_slider(
            "P:", self.lbl_p_speed, 1, 500, 50, self._on_p_speed,
            store_as="_sld_p_speed"))

        layout.addStretch()
        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  UI SETUP
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 8, 12, 8)
        main_layout.setSpacing(8)

        # ── Position Readout ─────────────────────────────────────
        pos_frame = QFrame()
        pos_frame.setObjectName("cardFrame")
        pos_layout = QGridLayout(pos_frame)
        pos_layout.setSpacing(4)

        for col, (name, attr) in enumerate([
            ("X (µm):", "lbl_x"), ("Y (µm):", "lbl_y"),
            ("Z (mm):", "lbl_z"),
            ("P1 (µL):", "lbl_p1"), ("P2 (µL):", "lbl_p2"), ("P3 (µL):", "lbl_p3"),
        ]):
            lbl = QLabel(name)
            lbl.setStyleSheet(f"color: {COLORS['overlay0']};")
            pos_layout.addWidget(lbl, 0, col)
            val = QLabel("—")
            val.setObjectName("positionValue")
            val.setFont(QFont("Consolas", scaled_font_size(11)))
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            setattr(self, attr, val)
            pos_layout.addWidget(val, 1, col)

        main_layout.addWidget(pos_frame)

        # ── XY Direction Pad ─────────────────────────────────────
        xy_frame = QFrame()
        xy_frame.setObjectName("cardFrame")
        xy_grid = QGridLayout(xy_frame)
        xy_grid.setSpacing(4)

        btn_up = QPushButton("▲")
        btn_up.setMinimumSize(_sc(50), _sc(40))
        btn_up.clicked.connect(partial(self._jog_xy, 0, -1))
        xy_grid.addWidget(btn_up, 0, 1)

        btn_left = QPushButton("◀")
        btn_left.setMinimumSize(_sc(50), _sc(40))
        btn_left.clicked.connect(partial(self._jog_xy, -1, 0))
        xy_grid.addWidget(btn_left, 1, 0)

        btn_home = QPushButton("⌂")
        btn_home.setMinimumSize(_sc(50), _sc(40))
        btn_home.setToolTip("Move to zero reference")
        btn_home.clicked.connect(self._jog_xy_home)
        xy_grid.addWidget(btn_home, 1, 1)

        btn_right = QPushButton("▶")
        btn_right.setMinimumSize(_sc(50), _sc(40))
        btn_right.clicked.connect(partial(self._jog_xy, 1, 0))
        xy_grid.addWidget(btn_right, 1, 2)

        btn_down = QPushButton("▼")
        btn_down.setMinimumSize(_sc(50), _sc(40))
        btn_down.clicked.connect(partial(self._jog_xy, 0, 1))
        xy_grid.addWidget(btn_down, 2, 1)

        main_layout.addWidget(xy_frame)

        # ── Z and Pump Controls ──────────────────────────────────
        zp_frame = QFrame()
        zp_frame.setObjectName("cardFrame")
        zp_layout = QHBoxLayout(zp_frame)

        # Z buttons
        z_group = QVBoxLayout()
        z_group.addWidget(QLabel("Z Needle"))
        btn_z_up = QPushButton("Z ▲")
        btn_z_up.clicked.connect(partial(self._jog_z, -1))
        z_group.addWidget(btn_z_up)
        btn_z_down = QPushButton("Z ▼")
        btn_z_down.clicked.connect(partial(self._jog_z, 1))
        z_group.addWidget(btn_z_down)
        zp_layout.addLayout(z_group)

        # Pump buttons — v7.2.5: store refs for enable/disable
        self._pump_buttons = {}  # {pump_name: (label, btn_ext, btn_ret)}
        self._pump_labels = {}   # {pump_name: QLabel for position}
        for pump_name in ["P1", "P2", "P3"]:
            p_group = QVBoxLayout()
            p_label = QLabel(pump_name)
            p_group.addWidget(p_label)
            btn_ext = QPushButton(f"{pump_name} ▲")
            btn_ext.clicked.connect(partial(self._jog_pump, pump_name, 1))
            p_group.addWidget(btn_ext)
            btn_ret = QPushButton(f"{pump_name} ▼")
            btn_ret.clicked.connect(partial(self._jog_pump, pump_name, -1))
            p_group.addWidget(btn_ret)
            zp_layout.addLayout(p_group)
            self._pump_buttons[pump_name] = (p_label, btn_ext, btn_ret)

        main_layout.addWidget(zp_frame)

        # ── Quick Actions ────────────────────────────────────────
        actions_frame = QFrame()
        actions_frame.setObjectName("cardFrame")
        actions_layout = QHBoxLayout(actions_frame)

        btn_set_zero = QPushButton("Set Zero Here")
        btn_set_zero.clicked.connect(self._set_zero)
        actions_layout.addWidget(btn_set_zero)

        btn_goto_zero = QPushButton("Go to Zero")
        btn_goto_zero.clicked.connect(self._goto_zero)
        actions_layout.addWidget(btn_goto_zero)

        btn_estop = QPushButton("⚠ STOP")
        btn_estop.setStyleSheet(
            f"background-color: {COLORS['red']}; color: {COLORS['crust']}; "
            f"font-weight: bold; padding: {_sp(6)} {_sp(16)};")
        btn_estop.clicked.connect(self._emergency_stop)
        actions_layout.addWidget(btn_estop)

        main_layout.addWidget(actions_frame)

        # ── v7.3.1: Well Plate Navigator ──────────────────────
        if WELL_NAV_AVAILABLE:
            nav_frame = QFrame()
            nav_frame.setObjectName("cardFrame")
            nav_layout = QVBoxLayout(nav_frame)
            nav_layout.setContentsMargins(4, 4, 4, 4)
            nav_layout.setSpacing(2)
            nav_label = QLabel("Well Plate — Click to Fast Travel")
            nav_label.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
            nav_layout.addWidget(nav_label)
            self._well_nav = WellPlateNavigator(parent=self)
            self._well_nav.setMinimumHeight(_sc(120))
            self._well_nav.well_clicked.connect(self._on_well_nav_click)
            nav_layout.addWidget(self._well_nav, stretch=1)
            self._well_nav_status = QLabel("")
            self._well_nav_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")
            nav_layout.addWidget(self._well_nav_status)
            main_layout.addWidget(nav_frame, stretch=1)

        main_layout.addStretch()

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATE
    # ════════════════════════════════════════════════════════════════

    def set_hardware_config(self, config):  # v7.2.7: pump visibility
        """Update jog page from hardware config — show/hide pump controls."""
        self._hardware_config = config
        self._refresh_pump_step_combo()
        # Update pump section visibility based on configured pumps
        if config:
            configured = set(config.configured_pump_ids) if hasattr(config, "configured_pump_ids") else set()
        else:
            configured = set()
        for pid in ["P1", "P2", "P3"]:
            frame = getattr(self, f"_pump_frame_{pid.lower()}", None)
            if frame is not None:
                frame.setVisible(pid in configured)
            # Also update labels
            lbl = getattr(self, f"lbl_{pid.lower()}_name", None)
            if lbl and config:
                pcfg = config.pumps.get(pid)
                if pcfg and pcfg.is_configured:
                    ink = getattr(pcfg, "ink_name", "") or pid
                    lbl.setText(f"{pid}: {ink}")
                else:
                    lbl.setText(f"{pid}: (not configured)")
        logger.info(f"Jog: pump visibility updated — active: {configured or 'none'}")


    def _update_pump_states(self):
        """v7.2.5: Enable/disable pump buttons based on HardwareConfig."""
        if not hasattr(self, '_pump_buttons'):
            return

        configured_pumps = set()
        if self._hardware_config:
            configured_pumps = set(self._hardware_config.configured_pump_ids)

        for pump_name, (label, btn_ext, btn_ret) in self._pump_buttons.items():
            is_configured = pump_name in configured_pumps
            btn_ext.setEnabled(is_configured)
            btn_ret.setEnabled(is_configured)
            if is_configured:
                # v7.2.6: Bright white text for active pumps
                label.setStyleSheet(f"color: {COLORS['text']}; font-weight: bold;")
                # Also brighten the readout label
                lbl_attr = f"lbl_p{pump_name[-1]}"
                if hasattr(self, lbl_attr):
                    getattr(self, lbl_attr).setStyleSheet(
                        f"color: {COLORS['text']}; font-size: 11px; font-weight: bold;")
                btn_ext.setToolTip(f"Extend {pump_name}")
                btn_ret.setToolTip(f"Retract {pump_name}")
            else:
                # v7.2.6: Dim text for inactive pumps
                label.setStyleSheet(f"color: {COLORS.get('overlay0', '#6c7086')};")
                lbl_attr = f"lbl_p{pump_name[-1]}"
                if hasattr(self, lbl_attr):
                    getattr(self, lbl_attr).setStyleSheet(
                        f"color: {COLORS.get('overlay0', '#6c7086')}; font-size: 11px;")
                btn_ext.setToolTip(f"{pump_name} not configured")
                btn_ret.setToolTip(f"{pump_name} not configured")

        # Update pump position labels in readout
        for col, pump_name in enumerate(["P1", "P2", "P3"], start=3):
            lbl_attr = f"lbl_p{pump_name[-1]}"
            if hasattr(self, lbl_attr):
                lbl = getattr(self, lbl_attr)
                if pump_name not in configured_pumps:
                    lbl.setText("—")
                    lbl.setStyleSheet(
                        f"color: {COLORS.get('overlay0', '#6c7086')}; font-size: 11px;")



    def on_status_update(self):
        """Called by MainWindow timer (~300 ms)."""
        ctrl = self.controller

        # XY position — v7.2.5: controller reports in microns directly
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            ux = xy[0] - ctrl.zero_position["x"]
            uy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{ux:,.1f}")
            self.lbl_y.setText(f"{uy:,.1f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")

        # ZP position
        # v7.4.2 hotfix: read via logical axis (Z/P1/P2/P3) so the
        # displayed value matches the user's axis_map, not the legacy
        # physical-tuple order.
        zp = ctrl.get_zp_position(cached=True)
        z_val = ctrl.zp_logical_value(zp, "Z")
        if z_val is not None:
            self.lbl_z.setText(f"{z_val - ctrl.zero_position['Z']:.2f}")
            for pidx, pid in enumerate(["P1", "P2", "P3"], start=1):
                lbl = getattr(self, f"lbl_p{pidx}", None)
                if lbl is None:
                    continue
                p_val = ctrl.zp_logical_value(zp, pid)
                if p_val is None:
                    lbl.setText("—")
                    continue
                pos_mm = p_val - ctrl.zero_position[pid]
                if (self._hardware_config and
                        pid in self._hardware_config.configured_pump_ids):
                    try:
                        pos_uL = self._hardware_config.mm_to_uL(pid, pos_mm)
                        lbl.setText(f"{pos_uL:.2f}")
                        continue
                    except (ValueError, AttributeError):
                        pass
                lbl.setText(f"{pos_mm:.2f}")
        else:
            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:
                lbl.setText("—")

        # v7.3.1: Update well plate navigator current-well highlight
        if self._well_nav is not None and xy[0] is not None:
            self._well_nav.update_current_from_position(xy[0], xy[1])

        # v7.2.4: Step verification — show measured delta after jog
        if (self._last_jog_step_um > 0 and
                self._last_xy_before[0] is not None and
                xy[0] is not None):
            # v7.2.5: positions are already in microns
            dx_um = abs(xy[0] - self._last_xy_before[0]) + \
                    abs(xy[1] - self._last_xy_before[1])
            # Only show verification if stage has settled (delta > 0)
            if dx_um > 0.01 and hasattr(self, '_lbl_last_jog'):
                current_text = self._lbl_last_jog.text()
                if "→" not in current_text:  # Don't keep appending
                    self._lbl_last_jog.setText(
                        f"{current_text}\n→ Moved: {dx_um:.1f} µm")

    # ════════════════════════════════════════════════════════════════
    #  KEYBOARD SHORTCUTS
    # ════════════════════════════════════════════════════════════════
        # v7.2.6: S6-D sync speeds
        self._sync_speed_sliders()


    def _refresh_pump_step_combo(self):
        """v7.2.5: Update pump step combo with µL values if HW config available."""
        if not hasattr(self, 'p_step_combo'):
            return
        self.p_step_combo.blockSignals(True)
        current_data = self.p_step_combo.currentData()
        self.p_step_combo.clear()
        if self._hardware_config and self._hardware_config.configured_pump_ids:
            # Use µL step sizes
            for s in P_STEPS_UL:
                self.p_step_combo.addItem(f"{s:g} µL", s)
            self.p_step_combo.addItem("Custom…", "custom")
            # Update custom spinbox suffix
            self._p_custom_spin.setSuffix(" µL")
            self._p_custom_spin.setRange(0.01, 500.0)
            # Try to restore selection
            idx = self.p_step_combo.findData(current_data)
            if idx >= 0:
                self.p_step_combo.setCurrentIndex(idx)
            else:
                self.p_step_combo.setCurrentIndex(2)  # Default 1.0 µL
        else:
            # Fallback: mm step sizes
            for s in P_STEPS:
                self.p_step_combo.addItem(f"{s:g} mm", s)
            self.p_step_combo.addItem("Custom…", "custom")
            self._p_custom_spin.setSuffix(" mm")
            self._p_custom_spin.setRange(0.001, 100.0)
            idx = self.p_step_combo.findData(current_data)
            if idx >= 0:
                self.p_step_combo.setCurrentIndex(idx)
            else:
                self.p_step_combo.setCurrentIndex(2)  # Default 0.1 mm
        self.p_step_combo.blockSignals(False)

    def _setup_shortcuts(self):
        self._key_actions = {
            Qt.Key.Key_Left:     partial(self._jog_xy, -1,  0),
            Qt.Key.Key_Right:    partial(self._jog_xy,  1,  0),
            Qt.Key.Key_Up:       partial(self._jog_xy,  0, -1),
            Qt.Key.Key_Down:     partial(self._jog_xy,  0,  1),
            Qt.Key.Key_PageUp:   partial(self._jog_z, -1),
            Qt.Key.Key_PageDown: partial(self._jog_z,  1),
        }

    def keyPressEvent(self, event: QKeyEvent):
        action = self._key_actions.get(event.key())
        if action and not event.isAutoRepeat():
            action()
        else:
            super().keyPressEvent(event)

    # ════════════════════════════════════════════════════════════════
    #  JOG ACTIONS — BUG-1 FIX: Use relative moves
    # ════════════════════════════════════════════════════════════════

    def _jog_xy(self, dx: int, dy: int):
        """
        Move XY stage by the selected step size.

        BUG-1 FIX (v7.1.2): Now uses move_xy_relative() instead of
        computing absolute targets from cached (potentially stale) positions.

        The old approach:
            pos = controller.get_xy_position(cached=True)  ← stale!
            target = (pos - zero) + dx * steps
            controller.move_xy_absolute(target, from_zero_ref=True)

        The new approach:
            controller.move_xy_relative(dx * steps, dy * steps)  ← always correct

        This eliminates the dependency on cached position data, which could
        be up to 300ms stale (the position poller interval), causing the
        displayed position to not match the requested jog amount.
        """
        if not self.controller.is_xy_connected:
            return

        # Get step size in microns (preset or custom)
        step_um = self._get_xy_step()

        # v7.2.5: Record pre-jog position for verification display
        self._last_xy_before = self.controller.get_xy_position(cached=True)
        self._last_jog_step_um = step_um

        # v7.2.5 FIX: Send microns directly to controller.
        # The Prior ProScan expects movement values in microns.
        self.controller.move_xy_relative_um(dx * step_um, dy * step_um)

        # Update step verification display
        direction = ""
        if dx > 0: direction = "X+"
        elif dx < 0: direction = "X−"
        if dy > 0: direction += "Y+"
        elif dy < 0: direction += "Y−"
        if hasattr(self, '_lbl_last_jog'):
            self._lbl_last_jog.setText(
                f"Last: {direction} {step_um:g} µm")
        logger.debug(f"Jog {direction}: {step_um:g} µm sent directly")

    def _jog_xy_home(self):
        """Move to the zero reference position (absolute move, this is fine)."""
        if self.controller.is_xy_connected:
            self.controller.move_xy_absolute(0, 0, from_zero_ref=True)

    def _jog_z(self, direction: int):
        if self.controller.is_zp_connected:
            feedrate = self._z_feedrate_mm_min()
            self.controller.move_z_relative(
                direction * self._get_z_step(), feedrate=feedrate)

    def _jog_pump(self, pump: str, direction: int):
        """Jog pump — convert µL step to mm if HW config available."""
        if not self.controller.is_zp_connected:
            return
        if not self.controller.is_pump_enabled(pump):
            logger.warning(f"{pump} is disabled — enable it in Hardware Setup to jog")
            return
        step_val = self._get_p_step()
        feedrate = self._pump_feedrate_mm_min(pump)
        if self._hardware_config:
            pump_cfg = self._hardware_config.pumps.get(pump)
            if pump_cfg and pump_cfg.is_configured:
                try:
                    step_mm = self._hardware_config.uL_to_mm(pump, step_val)
                    self.controller.move_pump_relative(
                        pump, direction * step_mm, feedrate=feedrate)
                    logger.debug(
                        f"Pump {pump}: {step_val} µL = {step_mm:.4f} mm "
                        f"@ {feedrate:.1f} mm/min")
                    return
                except (ValueError, AttributeError) as e:
                    logger.warning(f"µL→mm conversion failed for {pump}: {e}")
        # Fallback: use raw value as mm
        self.controller.move_pump_relative(
            pump, direction * step_val, feedrate=feedrate)

    # ── v7.4.2 hotfix: derive feedrate from the speed slider ────

    def _z_feedrate_mm_min(self) -> float | None:
        """Compute Z button-jog feedrate (mm/min) from the speed slider.

        The slider stores its current value on ``controller.zp_jog.z_speed``
        as mm/s. Marlin's G-code feedrate is mm/min, so we multiply by 60.
        Returns None if no zp_jog handler is available — in that case
        ZPStageManager falls back to its default feedrate.
        """
        jog = getattr(self.controller, 'zp_jog', None)
        if jog is None:
            return None
        return max(float(jog.z_speed) * 60.0, 1.0)

    def _pump_feedrate_mm_min(self, pump: str) -> float | None:
        """Compute pump button-jog feedrate (mm/min) from the speed slider.

        zp_jog.p_speed is µL/s when HardwareConfig has the pump's syringe
        configured, otherwise raw mm/s. Convert to mm/s via syringe
        geometry, then × 60 for mm/min. Returns None if no zp_jog.
        """
        jog = getattr(self.controller, 'zp_jog', None)
        if jog is None:
            return None
        rate = abs(float(jog.p_speed))
        if self._hardware_config:
            pump_cfg = self._hardware_config.pumps.get(pump)
            if pump_cfg and pump_cfg.is_configured:
                try:
                    vel_mm_s = pump_cfg.uL_to_mm(rate)
                    return max(vel_mm_s * 60.0, 1.0)
                except (ValueError, AttributeError):
                    pass
        # Fallback: p_speed already in mm/s
        return max(rate * 60.0, 1.0)

    def _set_zero(self):
        self.controller._calibrate_zero()
        logger.info("Zero reference set from jog page")

    def _goto_zero(self):
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
        self.controller.move_z_absolute(0, from_zero_ref=True)

    # ── v7.3.1: Well Plate Navigation ────────────────────────────

    def set_calibration_data(self, plate, well_positions, safe_z):
        """Receive calibration state from the calibration page.

        Args:
            plate: WellPlate instance (or None).
            well_positions: Dict mapping well_name → (x_um, y_um), or None.
            safe_z: Safe travel Z height in mm (zero-ref), or None.
        """
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if self._well_nav is not None:
            if plate:
                self._well_nav.set_plate(plate)
            if well_positions:
                self._well_nav.set_well_positions(well_positions)
            status_parts = []
            if plate:
                status_parts.append(f"{plate.format}-well")
            if well_positions:
                status_parts.append(f"{len(well_positions)} positions")
            if safe_z is not None:
                status_parts.append(f"safe Z={safe_z:.1f}mm")
            if hasattr(self, '_well_nav_status'):
                self._well_nav_status.setText(" | ".join(status_parts) if status_parts else "")

    def load_startup_plate(self, settings):
        """v7.3.2: Populate well plate navigator with geometry-predicted positions.

        Called on startup before calibration. Uses plate format from settings
        and stage center as approximate plate center.
        """
        if self._well_nav is None:
            return
        try:
            from SupportClasses.WellPlate import WellPlate
        except ImportError:
            return

        # Get plate format from calibration settings, then workspace fallback
        plate_format = (settings.get("calibration.plate_format")
                        or settings.get("workspace.plate_format")
                        or 96)
        try:
            plate = WellPlate.from_format(int(plate_format))
        except (ValueError, TypeError):
            logger.warning(f"Invalid plate format {plate_format}, defaulting to 96")
            plate = WellPlate.from_format(96)

        # Use stage center as approximate plate center
        # ProScan typical travel: 130mm x 85mm → center at 65000 x 42500 µm
        center_x = 65000.0
        center_y = 42500.0
        approx_positions = plate.get_all_positions_from_plate_center(center_x, center_y)

        self._plate = plate
        self._well_positions = approx_positions
        self._well_nav.set_plate(plate)
        self._well_nav.set_approximate_positions(approx_positions)

        if hasattr(self, '_well_nav_status'):
            self._well_nav_status.setText(
                f"{plate.format}-well | {len(approx_positions)} approx positions")
        logger.info(f"Startup plate: {plate.format}-well with geometry-predicted positions")

    def _on_well_nav_click(self, well_name: str):
        """Handle click on well plate navigator — safe fast-travel."""
        from PySide6.QtWidgets import QMessageBox
        if self._safe_z is None:
            QMessageBox.warning(self, "No Safe Z",
                                "Set Safe Z on the Calibration page before fast-traveling.\n"
                                "This ensures the needle is raised before XY travel.")
            return
        if self._well_positions is None or well_name not in self._well_positions:
            QMessageBox.warning(self, "No Position",
                                f"Well {well_name} has no calibrated position.\n"
                                "Run calibration first.")
            return
        target_x, target_y = self._well_positions[well_name]
        self.controller.safe_travel_to(
            target_x, target_y, self._safe_z)
        if hasattr(self, '_well_nav_status'):
            rel_x = target_x - self.controller.zero_position["x"]
            rel_y = target_y - self.controller.zero_position["y"]
            self._well_nav_status.setText(
                f"Traveled to {well_name}: ({rel_x:,.0f}, {rel_y:,.0f}) µm")
        logger.info(f"Fast travel to well {well_name}")

    # ── v7.3.2: Custom Step Size Handlers ─────────────────────

    def _on_xy_step_changed(self, index):
        is_custom = self.xy_step_combo.currentData() == "custom"
        self._xy_custom_spin.setVisible(is_custom)

    def _on_z_step_changed(self, index):
        is_custom = self.z_step_combo.currentData() == "custom"
        self._z_custom_spin.setVisible(is_custom)

    def _on_p_step_changed(self, index):
        is_custom = self.p_step_combo.currentData() == "custom"
        self._p_custom_spin.setVisible(is_custom)

    def _get_xy_step(self) -> float:
        """Get current XY step size in µm (preset or custom)."""
        val = self.xy_step_combo.currentData()
        if val == "custom":
            return self._xy_custom_spin.value()
        return val

    def _get_z_step(self) -> float:
        """Get current Z step size in mm (preset or custom)."""
        val = self.z_step_combo.currentData()
        if val == "custom":
            return self._z_custom_spin.value()
        return val

    def _get_p_step(self) -> float:
        """Get current pump step size (mm or µL depending on mode)."""
        val = self.p_step_combo.currentData()
        if val == "custom":
            return self._p_custom_spin.value()
        return val

    # ── v7.3.2: Absolute Go To ─────────────────────────────────

    def _absolute_goto(self):
        """Move to absolute coordinates (zero-referenced)."""
        target_x_um = self._goto_x.value()
        target_y_um = self._goto_y.value()
        target_z_mm = self._goto_z.value()

        # Convert from zero-ref to absolute stage coords
        zero = self.controller.zero_position
        abs_x_um = target_x_um + zero["x"]
        abs_y_um = target_y_um + zero["y"]

        if self._goto_safe.isChecked():
            safe_z = self._safe_z
            if safe_z is None:
                safe_z = 0.0  # Default: fully retracted
                logger.warning("Absolute Go To: no safe Z set, using 0.0 mm")
            self.controller.safe_travel_to(
                abs_x_um, abs_y_um,
                safe_z_mm=safe_z,
                target_z_mm=target_z_mm + zero["Z"],
            )
        else:
            # Direct moves — no safe Z retract
            if self.controller.is_xy_connected:
                self.controller.move_xy_absolute(
                    target_x_um, target_y_um, from_zero_ref=True)
            if self.controller.is_zp_connected:
                self.controller.move_z_absolute(
                    target_z_mm, from_zero_ref=True)
        logger.info(f"Absolute Go To: X={target_x_um:.1f}µm, "
                    f"Y={target_y_um:.1f}µm, Z={target_z_mm:.3f}mm "
                    f"(safe={'yes' if self._goto_safe.isChecked() else 'no'})")

    def _emergency_stop(self):
        if self.controller.zp_stage:
            try:
                self.controller.zp_stage.emergency_stop()
                logger.warning("EMERGENCY STOP sent!")
            except Exception as e:
                logger.error(f"E-stop failed: {e}")
        if self.controller.xy_stage:
            try:
                self.controller.xy_stage.stop_stage()
                logger.warning("XY STOP sent!")
            except Exception as e:
                logger.error(f"XY stop failed: {e}")

    # ── Context Panel Callbacks ──────────────────────────────────

    def _on_xy_speed(self, value):
        self.lbl_xy_speed.setText(f"{value}")
        if hasattr(self.controller, 'xy_jog') and self.controller.xy_jog:
            self.controller.xy_jog.xy_speed = float(value)

    def _on_z_speed(self, value):
        speed = value / 100.0
        self.lbl_z_speed.setText(f"{speed:.2f}")
        if hasattr(self.controller, 'zp_jog') and self.controller.zp_jog:
            self.controller.zp_jog.z_speed = speed

    def _on_p_speed(self, value):
        # p_speed is natively µL/s when hw config available
        rate = value / 100.0
        jog = getattr(self.controller, 'zp_jog', None)
        if (self._hardware_config and
                self._hardware_config.configured_pump_ids):
            self.lbl_p_speed.setText(f"{rate:.2f} µL/s")
        else:
            self.lbl_p_speed.setText(f"{rate:.2f}")
        if jog:
            jog.p_speed = rate


    def _sync_speed_sliders(self):
        """Read current speeds from jog handlers and update sliders + labels.

        v7.2.6: S6-E + S7-C — Xbox speed changes reflected on jog page;
        reconnect resets handled; pump shows µL/s when hw config available.
        """
        ctrl = self.controller

        # XY speed
        if getattr(ctrl, 'xy_jog', None):
            actual_xy = ctrl.xy_jog.xy_speed
            sld = getattr(self, '_sld_xy_speed', None)
            if sld is not None and abs(sld.value() - actual_xy) > 0.5:
                sld.blockSignals(True)
                sld.setValue(max(sld.minimum(), min(sld.maximum(), int(actual_xy))))
                sld.blockSignals(False)
                self.lbl_xy_speed.setText(f"{int(actual_xy)}")

        # Z speed
        if getattr(ctrl, 'zp_jog', None):
            actual_z = ctrl.zp_jog.z_speed
            sld_z = getattr(self, '_sld_z_speed', None)
            if sld_z is not None:
                slider_z = sld_z.value() / 100.0
                if abs(slider_z - actual_z) > 0.005:
                    sld_z.blockSignals(True)
                    sld_z.setValue(max(sld_z.minimum(),
                                      min(sld_z.maximum(), int(actual_z * 100))))
                    sld_z.blockSignals(False)
                    self.lbl_z_speed.setText(f"{actual_z:.2f}")

            # Pump speed — p_speed is natively µL/s when hw config available
            actual_p = ctrl.zp_jog.p_speed
            sld_p = getattr(self, '_sld_p_speed', None)
            if sld_p is not None:
                slider_target = int(actual_p * 100)
                if abs(sld_p.value() - slider_target) > 1:
                    sld_p.blockSignals(True)
                    sld_p.setValue(max(sld_p.minimum(),
                                      min(sld_p.maximum(), slider_target)))
                    sld_p.blockSignals(False)
                hw = self._hardware_config
                if hw and hw.configured_pump_ids:
                    self.lbl_p_speed.setText(f"{actual_p:.2f} µL/s")
                else:
                    self.lbl_p_speed.setText(f"{actual_p:.2f}")


    def _make_ctx_slider(self, label_text, value_label, min_val, max_val,
                         default, callback, store_as=None):
        """Create a labeled slider row for the context panel.
        v7.2.6: S6-B store_as — when given, stores slider ref as self.<store_as>.
        """
        frame = QFrame()
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        lbl = QLabel(label_text)
        lbl.setMinimumWidth(_sc(24))
        layout.addWidget(lbl)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default)
        slider.valueChanged.connect(callback)
        layout.addWidget(slider, stretch=1)

        layout.addWidget(value_label)

        # v7.2.6: S6-B store_as — store slider ref for readback
        if store_as is not None:
            setattr(self, store_as, slider)

        return frame

