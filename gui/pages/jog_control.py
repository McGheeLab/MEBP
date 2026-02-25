"""
Jog Control Page — Manual movement with direction buttons and speed sliders.

Provides:
    - XY direction pad (arrow buttons + diagonal)
    - Z needle up / down buttons
    - Pump forward / reverse buttons (P1, P2, P3)
    - Step size selector (discrete steps per click)
    - Speed multiplier sliders for XY, Z, and Pump
    - Live position readouts (zero-relative)
    - Keyboard shortcuts for all jog directions

The GUI jog uses fixed-step relative moves (not velocity commands like the
Xbox controller).  Each button click sends one move command.
"""

from __future__ import annotations

import logging
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QSlider, QFrame, QSizePolicy,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QKeyEvent, QShortcut, QKeySequence

from SupportClasses.StageController import StageController

logger = logging.getLogger(__name__)


# Step-size presets
XY_STEPS = [10, 50, 100, 500, 1000, 5000, 10000]
Z_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]
P_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]


class JogControlPage(QWidget):
    """Manual jog controls with fixed-step moves and speed adjustment."""

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._setup_ui()
        self._setup_shortcuts()

    # ── UI Construction ───────────────────────────────────────────

    def _setup_ui(self):
        main = QHBoxLayout(self)
        main.setSpacing(12)

        left = QVBoxLayout()
        right = QVBoxLayout()

        # ── Position readout ──────────────────────────────────────
        pos_group = QGroupBox("Current Position (zero-relative)")
        pos_grid = QGridLayout(pos_group)
        mono = QFont("Consolas", 14)

        labels = [("X:", "lbl_x"), ("Y:", "lbl_y"), ("Z:", "lbl_z"),
                  ("P1:", "lbl_p1"), ("P2:", "lbl_p2"), ("P3:", "lbl_p3")]
        for i, (name, attr) in enumerate(labels):
            row, col = divmod(i, 3)
            pos_grid.addWidget(QLabel(name), row, col * 2)
            lbl = QLabel("—")
            lbl.setFont(mono)
            lbl.setObjectName("valueLabel")
            pos_grid.addWidget(lbl, row, col * 2 + 1)
            setattr(self, attr, lbl)

        left.addWidget(pos_group)

        # ── XY Jog Pad ───────────────────────────────────────────
        xy_group = QGroupBox("XY Stage")
        xy_layout = QVBoxLayout(xy_group)

        # Step selector
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step:"))
        self.xy_step_combo = QComboBox()
        for s in XY_STEPS:
            self.xy_step_combo.addItem(f"{s:,} steps", s)
        self.xy_step_combo.setCurrentIndex(3)  # default 500
        step_row.addWidget(self.xy_step_combo)
        step_row.addStretch()
        xy_layout.addLayout(step_row)

        # Arrow pad (3x3 grid)
        pad = QGridLayout()
        pad.setSpacing(4)

        dirs = {
            (0, 0): ("↖", -1, -1), (0, 1): ("↑",  0, -1), (0, 2): ("↗",  1, -1),
            (1, 0): ("←", -1,  0), (1, 1): ("⌂",  0,  0), (1, 2): ("→",  1,  0),
            (2, 0): ("↙", -1,  1), (2, 1): ("↓",  0,  1), (2, 2): ("↘",  1,  1),
        }
        for (r, c), (sym, dx, dy) in dirs.items():
            btn = QPushButton(sym)
            btn.setObjectName("jogBtn")
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            if dx == 0 and dy == 0:
                btn.setText("⌂")
                btn.setToolTip("Home XY to zero reference")
                btn.clicked.connect(self._jog_xy_home)
            else:
                btn.setToolTip(f"Move XY ({dx:+d}, {dy:+d})")
                btn.clicked.connect(partial(self._jog_xy, dx, dy))
                btn.setAutoRepeat(True)
                btn.setAutoRepeatDelay(400)
                btn.setAutoRepeatInterval(150)
            pad.addWidget(btn, r, c)

        xy_layout.addLayout(pad)
        left.addWidget(xy_group)

        # ── Z Jog ────────────────────────────────────────────────
        z_group = QGroupBox("Z Needle")
        z_layout = QVBoxLayout(z_group)

        z_step_row = QHBoxLayout()
        z_step_row.addWidget(QLabel("Step:"))
        self.z_step_combo = QComboBox()
        for s in Z_STEPS:
            self.z_step_combo.addItem(f"{s} mm", s)
        self.z_step_combo.setCurrentIndex(2)  # default 0.1
        z_step_row.addWidget(self.z_step_combo)
        z_step_row.addStretch()
        z_layout.addLayout(z_step_row)

        z_btns = QHBoxLayout()
        btn_z_up = QPushButton("▲ Z Up")
        btn_z_up.setObjectName("jogBtn")
        btn_z_up.clicked.connect(partial(self._jog_z, -1))
        btn_z_up.setAutoRepeat(True)
        btn_z_up.setAutoRepeatDelay(400)
        btn_z_up.setAutoRepeatInterval(200)
        z_btns.addWidget(btn_z_up)

        btn_z_down = QPushButton("▼ Z Down")
        btn_z_down.setObjectName("jogBtn")
        btn_z_down.clicked.connect(partial(self._jog_z, 1))
        btn_z_down.setAutoRepeat(True)
        btn_z_down.setAutoRepeatDelay(400)
        btn_z_down.setAutoRepeatInterval(200)
        z_btns.addWidget(btn_z_down)

        z_layout.addLayout(z_btns)
        right.addWidget(z_group)

        # ── Pump Jog ─────────────────────────────────────────────
        pump_group = QGroupBox("Syringe Pumps")
        pump_layout = QVBoxLayout(pump_group)

        p_step_row = QHBoxLayout()
        p_step_row.addWidget(QLabel("Step:"))
        self.p_step_combo = QComboBox()
        for s in P_STEPS:
            self.p_step_combo.addItem(f"{s} mm", s)
        self.p_step_combo.setCurrentIndex(2)  # default 0.1
        p_step_row.addWidget(self.p_step_combo)
        p_step_row.addStretch()
        pump_layout.addLayout(p_step_row)

        for pump in ("P1", "P2", "P3"):
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{pump}:"))
            btn_fwd = QPushButton("▶ Push")
            btn_fwd.clicked.connect(partial(self._jog_pump, pump, 1))
            btn_fwd.setAutoRepeat(True)
            btn_fwd.setAutoRepeatDelay(400)
            btn_fwd.setAutoRepeatInterval(200)
            row.addWidget(btn_fwd)
            btn_rev = QPushButton("◀ Pull")
            btn_rev.clicked.connect(partial(self._jog_pump, pump, -1))
            btn_rev.setAutoRepeat(True)
            btn_rev.setAutoRepeatDelay(400)
            btn_rev.setAutoRepeatInterval(200)
            row.addWidget(btn_rev)
            pump_layout.addLayout(row)

        right.addWidget(pump_group)

        # ── Speed Sliders ────────────────────────────────────────
        speed_group = QGroupBox("Speed Multipliers (affects Xbox jogging)")
        speed_layout = QGridLayout(speed_group)

        self.lbl_xy_speed = QLabel("100")
        self._add_speed_slider(speed_layout, 0, "XY", self.lbl_xy_speed,
                               1, 10000, 100, self._on_xy_speed)

        self.lbl_z_speed = QLabel("0.50")
        self._add_speed_slider(speed_layout, 1, "Z", self.lbl_z_speed,
                               1, 500, 50, self._on_z_speed)

        self.lbl_p_speed = QLabel("0.50")
        self._add_speed_slider(speed_layout, 2, "Pump", self.lbl_p_speed,
                               1, 500, 50, self._on_p_speed)

        right.addWidget(speed_group)

        # ── Calibration quick actions ────────────────────────────
        cal_group = QGroupBox("Quick Actions")
        cal_layout = QHBoxLayout(cal_group)

        btn_zero = QPushButton("Set Zero Here")
        btn_zero.setObjectName("connectBtn")
        btn_zero.setToolTip("Set current position as the zero reference for all axes")
        btn_zero.clicked.connect(self._set_zero)
        cal_layout.addWidget(btn_zero)

        btn_goto_zero = QPushButton("Go to Zero")
        btn_goto_zero.setToolTip("Move all axes to the zero reference position")
        btn_goto_zero.clicked.connect(self._goto_zero)
        cal_layout.addWidget(btn_goto_zero)

        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setObjectName("disconnectBtn")
        btn_estop.setToolTip("Send emergency stop to ZP stage (M112)")
        btn_estop.clicked.connect(self._emergency_stop)
        cal_layout.addWidget(btn_estop)

        right.addWidget(cal_group)

        right.addStretch()
        left.addStretch()
        main.addLayout(left, stretch=1)
        main.addLayout(right, stretch=1)

    def _add_speed_slider(self, grid, row, label, value_label, min_v, max_v, default, callback):
        grid.addWidget(QLabel(f"{label}:"), row, 0)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(min_v, max_v)
        slider.setValue(default)
        slider.valueChanged.connect(callback)
        grid.addWidget(slider, row, 1)
        value_label.setMinimumWidth(60)
        value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(value_label, row, 2)

    # ── Keyboard shortcuts ────────────────────────────────────────

    def _setup_shortcuts(self):
        shortcuts = {
            Qt.Key.Key_Left:  partial(self._jog_xy, -1,  0),
            Qt.Key.Key_Right: partial(self._jog_xy,  1,  0),
            Qt.Key.Key_Up:    partial(self._jog_xy,  0, -1),
            Qt.Key.Key_Down:  partial(self._jog_xy,  0,  1),
            Qt.Key.Key_PageUp:   partial(self._jog_z, -1),
            Qt.Key.Key_PageDown: partial(self._jog_z,  1),
        }
        # Store for keyPressEvent
        self._key_actions = shortcuts

    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        action = self._key_actions.get(key)
        if action and not event.isAutoRepeat():
            action()
        else:
            super().keyPressEvent(event)

    # ── Jog Actions ───────────────────────────────────────────────

    def _jog_xy(self, dx: int, dy: int):
        if not self.controller.is_xy_connected:
            return
        step = self.xy_step_combo.currentData()
        pos = self.controller.get_xy_position(cached=True)
        if pos[0] is None:
            return
        zx = self.controller.zero_position["x"]
        zy = self.controller.zero_position["y"]
        target_x = (pos[0] - zx) + dx * step
        target_y = (pos[1] - zy) + dy * step
        self.controller.move_xy_absolute(target_x, target_y, from_zero_ref=True)

    def _jog_xy_home(self):
        if not self.controller.is_xy_connected:
            return
        self.controller.move_xy_absolute(0, 0, from_zero_ref=True)

    def _jog_z(self, direction: int):
        if not self.controller.is_zp_connected:
            return
        step = self.z_step_combo.currentData()
        self.controller.move_z_relative(direction * step)

    def _jog_pump(self, pump: str, direction: int):
        if not self.controller.is_zp_connected:
            return
        step = self.p_step_combo.currentData()
        self.controller.move_pump_relative(pump, direction * step)

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
                logger.error("E-stop failed: %s", e)

    # ── Speed callbacks ───────────────────────────────────────────

    def _on_xy_speed(self, value: int):
        self.lbl_xy_speed.setText(f"{value}")
        if self.controller.xy_jog:
            self.controller.xy_jog.xy_speed = float(value)

    def _on_z_speed(self, value: int):
        real = value / 100.0
        self.lbl_z_speed.setText(f"{real:.2f}")
        if self.controller.zp_jog:
            self.controller.zp_jog.z_speed = real

    def _on_p_speed(self, value: int):
        real = value / 100.0
        self.lbl_p_speed.setText(f"{real:.2f}")
        if self.controller.zp_jog:
            self.controller.zp_jog.p_speed = real

    # ── Timer update ──────────────────────────────────────────────

    def update_data(self):
        """Called by MainWindow timer to refresh position readouts."""
        ctrl = self.controller
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            self.lbl_x.setText(f"{xy[0] - ctrl.zero_position['x']:.0f}")
            self.lbl_y.setText(f"{xy[1] - ctrl.zero_position['y']:.0f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")

        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.3f}")
            self.lbl_p1.setText(f"{zp[1] - ctrl.zero_position['P1']:.3f}")
            self.lbl_p2.setText(f"{zp[2] - ctrl.zero_position['P2']:.3f}")
            self.lbl_p3.setText(f"{zp[3] - ctrl.zero_position['P3']:.3f}")
        else:
            for lbl in (self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3):
                lbl.setText("—")

        # Sync sliders with actual jog handler speeds (e.g. changed via Xbox)
        if ctrl.xy_jog:
            speed = int(ctrl.xy_jog.speed)
            self.lbl_xy_speed.setText(f"{speed}")
        if ctrl.zp_jog:
            zs = ctrl.zp_jog.speeds
            self.lbl_z_speed.setText(f"{zs['z']:.2f}")
            self.lbl_p_speed.setText(f"{zs['p']:.2f}")
