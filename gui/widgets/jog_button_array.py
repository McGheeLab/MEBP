"""
JogButtonArray — Reusable jog control widget with XY pad, Z buttons, and step selectors.

Emits signals when the user clicks a direction button. The parent page
is responsible for wiring signals to the StageController.

v7.3.1 — Phase 4

Supports two modes:
- **full**: All step sizes, larger buttons (default)
- **compact**: Fewer step sizes, smaller buttons for embedding in panels

Usage::

    array = JogButtonArray(compact=True)
    array.jog_xy_requested.connect(lambda dx, dy: controller.move_xy_relative_um(dx, dy))
    array.jog_z_requested.connect(lambda dz: controller.move_z_relative(dz))
"""

from __future__ import annotations

from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QFrame,
)
from PySide6.QtCore import Qt, Signal

from gui.styles import COLORS

# Step size presets
XY_STEPS_FULL = [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]
XY_STEPS_COMPACT = [1.0, 10.0, 50.0, 100.0, 500.0]
Z_STEPS_FULL = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]
Z_STEPS_COMPACT = [0.01, 0.05, 0.1, 0.5, 1.0]


class JogButtonArray(QWidget):
    """Compact jog control array with XY pad, Z buttons, and step selectors.

    Signals:
        jog_xy_requested(float, float): (dx_um, dy_um) relative XY move requested.
        jog_z_requested(float): dz_mm relative Z move requested.
        home_requested(): Move-to-zero requested.
    """

    jog_xy_requested = Signal(float, float)
    jog_z_requested = Signal(float)
    home_requested = Signal()

    def __init__(self, compact: bool = False, parent: QWidget | None = None):
        super().__init__(parent)
        self._compact = compact
        self._setup_ui()

    @property
    def xy_step_um(self) -> float:
        """Currently selected XY step size in µm."""
        return self._xy_combo.currentData() or 50.0

    @property
    def z_step_mm(self) -> float:
        """Currently selected Z step size in mm."""
        return self._z_combo.currentData() or 0.1

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        btn_size = 32 if self._compact else 44

        # ── XY Direction Pad ──────────────────────────────────
        xy_grid = QGridLayout()
        xy_grid.setSpacing(2)

        btn_up = QPushButton("▲")
        btn_up.setFixedSize(btn_size, btn_size)
        btn_up.clicked.connect(partial(self._on_xy, 0, -1))
        xy_grid.addWidget(btn_up, 0, 1)

        btn_left = QPushButton("◀")
        btn_left.setFixedSize(btn_size, btn_size)
        btn_left.clicked.connect(partial(self._on_xy, -1, 0))
        xy_grid.addWidget(btn_left, 1, 0)

        btn_home = QPushButton("⌂")
        btn_home.setFixedSize(btn_size, btn_size)
        btn_home.setToolTip("Go to zero reference")
        btn_home.clicked.connect(self.home_requested.emit)
        xy_grid.addWidget(btn_home, 1, 1)

        btn_right = QPushButton("▶")
        btn_right.setFixedSize(btn_size, btn_size)
        btn_right.clicked.connect(partial(self._on_xy, 1, 0))
        xy_grid.addWidget(btn_right, 1, 2)

        btn_down = QPushButton("▼")
        btn_down.setFixedSize(btn_size, btn_size)
        btn_down.clicked.connect(partial(self._on_xy, 0, 1))
        xy_grid.addWidget(btn_down, 2, 1)

        # Z buttons beside XY pad
        btn_z_up = QPushButton("Z▲")
        btn_z_up.setFixedSize(btn_size, btn_size)
        btn_z_up.clicked.connect(partial(self._on_z, -1))
        xy_grid.addWidget(btn_z_up, 0, 3)

        btn_z_down = QPushButton("Z▼")
        btn_z_down.setFixedSize(btn_size, btn_size)
        btn_z_down.clicked.connect(partial(self._on_z, 1))
        xy_grid.addWidget(btn_z_down, 2, 3)

        layout.addLayout(xy_grid)

        # ── Step Size Selectors ───────────────────────────────
        step_row = QHBoxLayout()
        step_row.setSpacing(4)

        step_row.addWidget(QLabel("XY:"))
        self._xy_combo = QComboBox()
        xy_steps = XY_STEPS_COMPACT if self._compact else XY_STEPS_FULL
        for s in xy_steps:
            self._xy_combo.addItem(f"{s:g} µm", s)
        # Default to 50 µm
        idx = next((i for i, s in enumerate(xy_steps) if s == 50.0), 2)
        self._xy_combo.setCurrentIndex(idx)
        self._xy_combo.setMaximumWidth(90)
        step_row.addWidget(self._xy_combo)

        step_row.addWidget(QLabel("Z:"))
        self._z_combo = QComboBox()
        z_steps = Z_STEPS_COMPACT if self._compact else Z_STEPS_FULL
        for s in z_steps:
            self._z_combo.addItem(f"{s} mm", s)
        # Default to 0.1 mm
        idx = next((i for i, s in enumerate(z_steps) if s == 0.1), 2)
        self._z_combo.setCurrentIndex(idx)
        self._z_combo.setMaximumWidth(80)
        step_row.addWidget(self._z_combo)

        step_row.addStretch()
        layout.addLayout(step_row)

    def _on_xy(self, dx: int, dy: int):
        step = self.xy_step_um
        self.jog_xy_requested.emit(dx * step, dy * step)

    def _on_z(self, direction: int):
        step = self.z_step_mm
        self.jog_z_requested.emit(direction * step)
