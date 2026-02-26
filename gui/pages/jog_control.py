"""
Jog Control Page — Manual movement with compact layout.

Main content: XY pad, Z/pump buttons, position readouts, quick actions
Context panel: step sizes, speed multipliers
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

XY_STEPS = [10, 50, 100, 500, 1000, 5000, 10000]
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
        self._setup_ui()
        self._setup_shortcuts()

    def get_page_title(self) -> str:
        return "Jog Control"

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

        # XY step
        xy_row = QHBoxLayout()
        xy_row.addWidget(QLabel("XY:"))
        self.xy_step_combo = QComboBox()
        for s in XY_STEPS:
            self.xy_step_combo.addItem(f"{s:,} steps", s)
        self.xy_step_combo.setCurrentIndex(3)
        xy_row.addWidget(self.xy_step_combo, stretch=1)
        layout.addLayout(xy_row)

        # Z step
        z_row = QHBoxLayout()
        z_row.addWidget(QLabel("Z:"))
        self.z_step_combo = QComboBox()
        for s in Z_STEPS:
            self.z_step_combo.addItem(f"{s} mm", s)
        self.z_step_combo.setCurrentIndex(2)
        z_row.addWidget(self.z_step_combo, stretch=1)
        layout.addLayout(z_row)

        # Pump step
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
            "Pump:", self.lbl_p_speed, 1, 500, 50, self._on_p_speed))

        # ── Quick Actions ────────────────────────────────────────
        actions_label = QLabel("Quick Actions")
        actions_label.setObjectName("contextSectionLabel")
        layout.addWidget(actions_label)

        btn_zero = QPushButton("Set Zero Here")
        btn_zero.setObjectName("successBtn")
        btn_zero.clicked.connect(self._set_zero)
        layout.addWidget(btn_zero)

        btn_goto = QPushButton("Go to Zero")
        btn_goto.clicked.connect(self._goto_zero)
        layout.addWidget(btn_goto)

        btn_estop = QPushButton("⚠ EMERGENCY STOP")
        btn_estop.setObjectName("dangerBtn")
        btn_estop.clicked.connect(self._emergency_stop)
        layout.addWidget(btn_estop)

        layout.addStretch()

        self._context_widget = ctx
        return ctx

    def _make_ctx_slider(self, label, value_label, min_v, max_v, default, callback):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(2)
        top = QHBoxLayout()
        top.addWidget(QLabel(label))
        top.addStretch()
        value_label.setObjectName("dimLabel")
        top.addWidget(value_label)
        lay.addLayout(top)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_v, max_v)
        slider.setValue(default)
        slider.valueChanged.connect(callback)
        lay.addWidget(slider)
        return w

    # ── Main Content UI ──────────────────────────────────────────

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        outer.addWidget(scroll)

        container = QWidget()
        main = QHBoxLayout(container)
        main.setSpacing(8)
        main.setContentsMargins(16, 12, 16, 12)
        scroll.setWidget(container)

        left = QVBoxLayout()
        right = QVBoxLayout()
        mono = QFont("Consolas", 14)

        # ── Position readout ─────────────────────────────────────
        pos_card = QFrame()
        pos_card.setObjectName("cardFrame")
        pos_grid = QGridLayout(pos_card)
        pos_grid.setSpacing(4)

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

        left.addWidget(pos_card)

        # ── XY Jog Pad ──────────────────────────────────────────
        xy_card = QFrame()
        xy_card.setObjectName("cardFrame")
        xy_layout = QVBoxLayout(xy_card)
        xy_layout.setSpacing(4)

        xy_title = QLabel("XY Stage")
        xy_title.setObjectName("sectionLabel")
        xy_layout.addWidget(xy_title)

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
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            if dx == 0 and dy == 0:
                btn.setToolTip("Home XY to zero")
                btn.clicked.connect(self._jog_xy_home)
            else:
                btn.clicked.connect(partial(self._jog_xy, dx, dy))
                btn.setAutoRepeat(True)
                btn.setAutoRepeatDelay(400)
                btn.setAutoRepeatInterval(150)
            pad.addWidget(btn, r, c)

        xy_layout.addLayout(pad)
        left.addWidget(xy_card)

        # ── Z Jog ────────────────────────────────────────────────
        z_card = QFrame()
        z_card.setObjectName("cardFrame")
        z_layout = QVBoxLayout(z_card)
        z_layout.setSpacing(4)

        z_title = QLabel("Z Needle")
        z_title.setObjectName("sectionLabel")
        z_layout.addWidget(z_title)

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
        right.addWidget(z_card)

        # ── Pump Jog ─────────────────────────────────────────────
        pump_card = QFrame()
        pump_card.setObjectName("cardFrame")
        pump_layout = QVBoxLayout(pump_card)
        pump_layout.setSpacing(4)

        pump_title = QLabel("Syringe Pumps")
        pump_title.setObjectName("sectionLabel")
        pump_layout.addWidget(pump_title)

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
        right.addWidget(pump_card)

        right.addStretch()
        left.addStretch()
        main.addLayout(left, stretch=1)
        main.addLayout(right, stretch=1)

    # ── Keyboard Shortcuts ───────────────────────────────────────

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

    # ── Jog Actions ──────────────────────────────────────────────

    def _jog_xy(self, dx: int, dy: int):
        if not self.controller.is_xy_connected:
            return
        step = self.xy_step_combo.currentData()
        pos = self.controller.get_xy_position(cached=True)
        if pos[0] is None:
            return
        zx = self.controller.zero_position["x"]
        zy = self.controller.zero_position["y"]
        self.controller.move_xy_absolute(
            (pos[0] - zx) + dx * step,
            (pos[1] - zy) + dy * step,
            from_zero_ref=True)

    def _jog_xy_home(self):
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
                logger.error("E-stop failed: %s", e)

    # ── Speed Callbacks ──────────────────────────────────────────

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

    # ── Status Update ────────────────────────────────────────────

    def on_status_update(self):
        self.update_data()

    def update_data(self):
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

        if ctrl.xy_jog:
            self.lbl_xy_speed.setText(f"{int(ctrl.xy_jog.speed)}")
        if ctrl.zp_jog:
            zs = ctrl.zp_jog.speeds
            self.lbl_z_speed.setText(f"{zs['z']:.2f}")
            self.lbl_p_speed.setText(f"{zs['p']:.2f}")
