"""
Jog Control Page — Manual movement with compact layout.

Main content: XY pad, Z/pump buttons, position readouts, quick actions
Context panel: step sizes, speed multipliers

v7.1.1: XY step sizes and positions displayed in microns (µm).
         Conversion to/from microsteps handled internally via microsteps_per_micron.
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
            "Pump:", self.lbl_p_speed, 1, 500, 50, self._on_p_speed))

        # ── Quick Actions ────────────────────────────────────────
        action_label = QLabel("Quick Actions")
        action_label.setObjectName("contextSectionLabel")
        layout.addWidget(action_label)

        btn_home = QPushButton("Home All")
        btn_home.setObjectName("successBtn")
        btn_home.clicked.connect(self._goto_zero)
        layout.addWidget(btn_home)

        btn_zero = QPushButton("Zero All")
        btn_zero.setObjectName("accentBtn")
        btn_zero.clicked.connect(self._set_zero)
        layout.addWidget(btn_zero)

        btn_estop = QPushButton("E-Stop")
        btn_estop.setObjectName("dangerBtn")
        btn_estop.clicked.connect(self._emergency_stop)
        layout.addWidget(btn_estop)

        layout.addStretch()
        self._context_widget = ctx
        return ctx

    def _make_ctx_slider(self, label_text, value_label, minimum, maximum,
                         default, callback):
        """Create a labeled slider widget for the context panel."""
        frame = QWidget()
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 2, 0, 2)
        row.setSpacing(4)

        lbl = QLabel(label_text)
        lbl.setObjectName("contextLabel")
        lbl.setFixedWidth(40)
        row.addWidget(lbl)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(minimum)
        slider.setMaximum(maximum)
        slider.setValue(default)
        slider.valueChanged.connect(callback)
        row.addWidget(slider, stretch=1)

        value_label.setFixedWidth(50)
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        value_label.setObjectName("contextLabel")
        row.addWidget(value_label)

        return frame

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(12, 8, 12, 8)

        # Left column: positions + XY pad
        left = QVBoxLayout()
        left.setSpacing(8)

        # ── Position Readout (in µm for XY, mm for Z/Pumps) ─────
        pos_card = QFrame()
        pos_card.setObjectName("cardFrame")
        pos_grid = QGridLayout(pos_card)
        pos_grid.setSpacing(4)

        mono = QFont("Consolas", 12)

        # XY positions (µm)
        pos_grid.addWidget(QLabel("X:"), 0, 0)
        self.lbl_x = QLabel("—")
        self.lbl_x.setFont(mono)
        self.lbl_x.setObjectName("valueLabel")
        pos_grid.addWidget(self.lbl_x, 0, 1)
        lbl_x_unit = QLabel("µm")
        lbl_x_unit.setObjectName("unitLabel")
        pos_grid.addWidget(lbl_x_unit, 0, 2)

        pos_grid.addWidget(QLabel("Y:"), 0, 3)
        self.lbl_y = QLabel("—")
        self.lbl_y.setFont(mono)
        self.lbl_y.setObjectName("valueLabel")
        pos_grid.addWidget(self.lbl_y, 0, 4)
        lbl_y_unit = QLabel("µm")
        lbl_y_unit.setObjectName("unitLabel")
        pos_grid.addWidget(lbl_y_unit, 0, 5)

        # Z/Pump positions (mm)
        pos_grid.addWidget(QLabel("Z:"), 1, 0)
        self.lbl_z = QLabel("—")
        self.lbl_z.setFont(mono)
        self.lbl_z.setObjectName("valueLabel")
        pos_grid.addWidget(self.lbl_z, 1, 1)
        lbl_z_unit = QLabel("mm")
        lbl_z_unit.setObjectName("unitLabel")
        pos_grid.addWidget(lbl_z_unit, 1, 2)

        pos_grid.addWidget(QLabel("P1:"), 1, 3)
        self.lbl_p1 = QLabel("—")
        self.lbl_p1.setFont(mono)
        self.lbl_p1.setObjectName("valueLabel")
        pos_grid.addWidget(self.lbl_p1, 1, 4)

        pos_grid.addWidget(QLabel("P2:"), 2, 0)
        self.lbl_p2 = QLabel("—")
        self.lbl_p2.setFont(mono)
        self.lbl_p2.setObjectName("valueLabel")
        pos_grid.addWidget(self.lbl_p2, 2, 1)

        pos_grid.addWidget(QLabel("P3:"), 2, 3)
        self.lbl_p3 = QLabel("—")
        self.lbl_p3.setFont(mono)
        self.lbl_p3.setObjectName("valueLabel")
        pos_grid.addWidget(self.lbl_p3, 2, 4)

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
        z_up = QPushButton("▲ Up")
        z_up.setObjectName("jogBtn")
        z_up.clicked.connect(partial(self._jog_z, -1))
        z_up.setAutoRepeat(True)
        z_up.setAutoRepeatDelay(400)
        z_up.setAutoRepeatInterval(150)
        z_btns.addWidget(z_up)

        z_down = QPushButton("▼ Down")
        z_down.setObjectName("jogBtn")
        z_down.clicked.connect(partial(self._jog_z, 1))
        z_down.setAutoRepeat(True)
        z_down.setAutoRepeatDelay(400)
        z_down.setAutoRepeatInterval(150)
        z_btns.addWidget(z_down)

        z_layout.addLayout(z_btns)
        left.addWidget(z_card)

        # ── Pump Jog ─────────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(8)

        for pump_name in ["P1", "P2", "P3"]:
            pump_card = QFrame()
            pump_card.setObjectName("cardFrame")
            pump_layout = QVBoxLayout(pump_card)
            pump_layout.setSpacing(4)

            pump_title = QLabel(f"Pump {pump_name}")
            pump_title.setObjectName("sectionLabel")
            pump_layout.addWidget(pump_title)

            pump_btns = QHBoxLayout()
            ext = QPushButton("Extrude ▼")
            ext.clicked.connect(partial(self._jog_pump, pump_name, 1))
            ext.setAutoRepeat(True)
            ext.setAutoRepeatDelay(400)
            ext.setAutoRepeatInterval(150)
            pump_btns.addWidget(ext)

            ret = QPushButton("Retract ▲")
            ret.clicked.connect(partial(self._jog_pump, pump_name, -1))
            ret.setAutoRepeat(True)
            ret.setAutoRepeatDelay(400)
            ret.setAutoRepeatInterval(150)
            pump_btns.addWidget(ret)

            pump_layout.addLayout(pump_btns)
            right.addWidget(pump_card)

        right.addStretch()

        main_layout.addLayout(left, stretch=3)
        main_layout.addLayout(right, stretch=2)

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATES
    # ════════════════════════════════════════════════════════════════

    def on_status_update(self):
        """Called by MainWindow timer — refresh position readouts."""
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
    #  JOG ACTIONS
    # ════════════════════════════════════════════════════════════════

    def _jog_xy(self, dx: int, dy: int):
        """Move XY stage by the selected step size (converted from µm to microsteps)."""
        if not self.controller.is_xy_connected:
            return
        # Get step size in microns from combo box
        step_um = self.xy_step_combo.currentData()
        # Convert µm → microsteps
        step_steps = step_um * self._microsteps_per_micron

        pos = self.controller.get_xy_position(cached=True)
        if pos[0] is None:
            return
        zx = self.controller.zero_position["x"]
        zy = self.controller.zero_position["y"]
        self.controller.move_xy_absolute(
            (pos[0] - zx) + dx * step_steps,
            (pos[1] - zy) + dy * step_steps,
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
                logger.error(f"E-stop failed: {e}")

    # ── Context Panel Callbacks ──────────────────────────────────

    def _on_xy_speed(self, value):
        self.lbl_xy_speed.setText(f"{value}")
        if hasattr(self.controller, '_xy_jog_handler'):
            self.controller._xy_jog_handler.xy_speed = float(value)

    def _on_z_speed(self, value):
        speed = value / 100.0
        self.lbl_z_speed.setText(f"{speed:.2f}")
        if hasattr(self.controller, '_zp_jog_handler'):
            self.controller._zp_jog_handler.z_speed = speed

    def _on_p_speed(self, value):
        speed = value / 100.0
        self.lbl_p_speed.setText(f"{speed:.2f}")
        if hasattr(self.controller, '_zp_jog_handler'):
            self.controller._zp_jog_handler.pump_speed = speed
