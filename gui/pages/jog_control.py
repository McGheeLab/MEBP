"""
Jog Control Page — Manual movement with compact layout.

Main content: XY pad, Z/pump buttons, position readouts, quick actions
Context panel: step sizes, speed multipliers

v7.1.1: XY step sizes and positions displayed in microns (µm).
         Conversion to/from microsteps handled internally via microsteps_per_micron.

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
    QScrollArea,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QKeyEvent

from SupportClasses.StageController import StageController
from gui.styles import COLORS

logger = logging.getLogger(__name__)

# Step sizes in microns (µm) for the XY stage
XY_STEPS_UM = [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]

# Z and pump step sizes remain in mm
Z_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]
P_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]


class JogControlPage(QWidget):
    """Manual jog controls with fixed-step moves."""

    _page_title_text = "Jog Control"

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._context_widget = None

        # Microsteps per micron — set by MainWindow, default 10 (ProScan III typical)
        self._microsteps_per_micron: float = 10.0

        self._setup_ui()
        self._setup_shortcuts()

    def get_page_title(self) -> str:
        return "Jog Control"

    def set_microsteps_per_micron(self, value: float):
        """Called by MainWindow when the conversion factor changes."""
        self._microsteps_per_micron = max(0.001, value)

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

        # XY step (in µm)
        xy_row = QHBoxLayout()
        xy_row.addWidget(QLabel("XY:"))
        self.xy_step_combo = QComboBox()
        for s in XY_STEPS_UM:
            if s >= 1.0:
                self.xy_step_combo.addItem(f"{s:g} µm", s)
            else:
                self.xy_step_combo.addItem(f"{s:.2f} µm", s)
        self.xy_step_combo.setCurrentIndex(3)  # Default: 50 µm
        xy_row.addWidget(self.xy_step_combo, stretch=1)
        layout.addLayout(xy_row)

        # Z step (in mm)
        z_row = QHBoxLayout()
        z_row.addWidget(QLabel("Z:"))
        self.z_step_combo = QComboBox()
        for s in Z_STEPS:
            self.z_step_combo.addItem(f"{s} mm", s)
        self.z_step_combo.setCurrentIndex(2)
        z_row.addWidget(self.z_step_combo, stretch=1)
        layout.addLayout(z_row)

        # Pump step (in mm)
        p_row = QHBoxLayout()
        p_row.addWidget(QLabel("Pump:"))
        self.p_step_combo = QComboBox()
        for s in P_STEPS:
            self.p_step_combo.addItem(f"{s} mm", s)
        self.p_step_combo.setCurrentIndex(2)
        p_row.addWidget(self.p_step_combo, stretch=1)
        layout.addLayout(p_row)

        # ── Speed Multipliers ────────────────────────────────────
        speed_label = QLabel("Speed (Xbox Jog)")
        speed_label.setObjectName("contextSectionLabel")
        layout.addWidget(speed_label)

        # XY speed
        self.lbl_xy_speed = QLabel("100")
        layout.addWidget(self._make_ctx_slider(
            "XY:", self.lbl_xy_speed, 1, 10000, 100, self._on_xy_speed))

        # Z speed
        self.lbl_z_speed = QLabel("0.50")
        layout.addWidget(self._make_ctx_slider(
            "Z:", self.lbl_z_speed, 1, 500, 50, self._on_z_speed))

        # Pump speed
        self.lbl_p_speed = QLabel("0.50")
        layout.addWidget(self._make_ctx_slider(
            "P:", self.lbl_p_speed, 1, 500, 50, self._on_p_speed))

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
            ("P1:", "lbl_p1"), ("P2:", "lbl_p2"), ("P3:", "lbl_p3"),
        ]):
            lbl = QLabel(name)
            lbl.setStyleSheet(f"color: {COLORS['overlay0']};")
            pos_layout.addWidget(lbl, 0, col)
            val = QLabel("—")
            val.setObjectName("positionValue")
            val.setFont(QFont("Consolas", 11))
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
        btn_up.setMinimumSize(50, 40)
        btn_up.clicked.connect(partial(self._jog_xy, 0, -1))
        xy_grid.addWidget(btn_up, 0, 1)

        btn_left = QPushButton("◀")
        btn_left.setMinimumSize(50, 40)
        btn_left.clicked.connect(partial(self._jog_xy, -1, 0))
        xy_grid.addWidget(btn_left, 1, 0)

        btn_home = QPushButton("⌂")
        btn_home.setMinimumSize(50, 40)
        btn_home.setToolTip("Move to zero reference")
        btn_home.clicked.connect(self._jog_xy_home)
        xy_grid.addWidget(btn_home, 1, 1)

        btn_right = QPushButton("▶")
        btn_right.setMinimumSize(50, 40)
        btn_right.clicked.connect(partial(self._jog_xy, 1, 0))
        xy_grid.addWidget(btn_right, 1, 2)

        btn_down = QPushButton("▼")
        btn_down.setMinimumSize(50, 40)
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

        # Pump buttons
        for pump_name in ["P1", "P2", "P3"]:
            p_group = QVBoxLayout()
            p_group.addWidget(QLabel(pump_name))
            btn_ext = QPushButton(f"{pump_name} ▲")
            btn_ext.clicked.connect(partial(self._jog_pump, pump_name, 1))
            p_group.addWidget(btn_ext)
            btn_ret = QPushButton(f"{pump_name} ▼")
            btn_ret.clicked.connect(partial(self._jog_pump, pump_name, -1))
            p_group.addWidget(btn_ret)
            zp_layout.addLayout(p_group)

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
            f"font-weight: bold; padding: 6px 16px;")
        btn_estop.clicked.connect(self._emergency_stop)
        actions_layout.addWidget(btn_estop)

        main_layout.addWidget(actions_frame)
        main_layout.addStretch()

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATE
    # ════════════════════════════════════════════════════════════════

    def on_status_update(self):
        """Called by MainWindow timer (~300 ms)."""
        ctrl = self.controller

        # XY position (convert steps → µm)
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            zx = xy[0] - ctrl.zero_position["x"]
            zy = xy[1] - ctrl.zero_position["y"]
            ux = zx / self._microsteps_per_micron
            uy = zy / self._microsteps_per_micron
            self.lbl_x.setText(f"{ux:,.1f}")
            self.lbl_y.setText(f"{uy:,.1f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")

        # ZP position (already in mm)
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            self.lbl_p1.setText(f"{zp[1] - ctrl.zero_position['P1']:.2f}")
            self.lbl_p2.setText(f"{zp[2] - ctrl.zero_position['P2']:.2f}")
            self.lbl_p3.setText(f"{zp[3] - ctrl.zero_position['P3']:.2f}")
        else:
            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:
                lbl.setText("—")

    # ════════════════════════════════════════════════════════════════
    #  KEYBOARD SHORTCUTS
    # ════════════════════════════════════════════════════════════════

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

        # Get step size in microns from combo box
        step_um = self.xy_step_combo.currentData()
        # Convert µm → microsteps
        step_steps = step_um * self._microsteps_per_micron

        # BUG-1 FIX: Use relative move — no dependency on cached position
        self.controller.move_xy_relative(dx * step_steps, dy * step_steps)

    def _jog_xy_home(self):
        """Move to the zero reference position (absolute move, this is fine)."""
        if self.controller.is_xy_connected:
            self.controller.move_xy_absolute(0, 0, from_zero_ref=True)

    def _jog_z(self, direction: int):
        if self.controller.is_zp_connected:
            self.controller.move_z_relative(direction * self.z_step_combo.currentData())

    def _jog_pump(self, pump: str, direction: int):
        if self.controller.is_zp_connected:
            self.controller.move_pump_relative(pump, direction * self.p_step_combo.currentData())

    def _set_zero(self):
        self.controller._calibrate_zero()
        logger.info("Zero reference set from jog page")

    def _goto_zero(self):
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
        self.controller.move_z_absolute(0, from_zero_ref=True)

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
        speed = value / 100.0
        self.lbl_p_speed.setText(f"{speed:.2f}")
        if hasattr(self.controller, 'zp_jog') and self.controller.zp_jog:
            self.controller.zp_jog.pump_speed = speed

    # ── Widget Helpers ───────────────────────────────────────────

    def _make_ctx_slider(self, label_text, value_label, min_val, max_val,
                         default, callback):
        """Create a labeled slider row for the context panel."""
        frame = QFrame()
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        lbl = QLabel(label_text)
        lbl.setMinimumWidth(24)
        layout.addWidget(lbl)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default)
        slider.valueChanged.connect(callback)
        layout.addWidget(slider, stretch=1)

        layout.addWidget(value_label)
        return frame
