"""
JogButtonArray — Reusable jog control widget with XY pad, Z buttons, and step selectors.

v7.4.2: redesigned layout per user direction.

  * Step sizes for each axis (XY, Z, Pump) sit ABOVE the motion
    controls. Each axis row gets five order-of-magnitude buttons
    (1 / 10 / 100 / 1000 / 10000) in the axis's native unit plus a
    custom-value text box so the user can dial in any step.
  * XY direction pad in the middle with Z up/down stacked beside it.
  * Pump axes are stacked: P1▲ above P1▼, P2▲ above P2▼, P3▲ above
    P3▼, so each pump reads as a single vertical column.

Signals emitted: ``jog_xy_requested(dx_um, dy_um)``,
``jog_z_requested(dz_mm)``, ``jog_pump_requested(pump_id, dist)``,
``home_requested()``. Parent wires them to the controller.
"""

from __future__ import annotations

from functools import partial

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QButtonGroup, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QWidget,
)

from gui.scaling import s as _sc, scaled_font_size as _sf
from gui.styles import COLORS

# Order-of-magnitude step sizes per axis (v7.4.2).
# XY native unit: µm.  Z native unit: mm.  Pump native unit: µL.
# Z = µm / 1000  so user-facing magnitudes still read 1/10/100/1000/10000.
_MAGNITUDES = (1.0, 10.0, 100.0, 1000.0, 10000.0)

# Catppuccin Mocha pieces used by the inline styles below.
_BG = COLORS.get("surface0", "#313244")
_BG_HOVER = COLORS.get("surface1", "#45475a")
_BG_ACTIVE = COLORS.get("surface2", "#585b70")
_FG = COLORS.get("text", "#cdd6f4")
_FG_MUTED = COLORS.get("subtext0", "#a6adc8")
_ACCENT = COLORS.get("mauve", "#cba6f7")


def _build_step_row(label: str, unit: str, magnitudes: tuple[float, ...],
                    default: float, on_change) -> tuple[QHBoxLayout, "_StepGroup"]:
    """Build a step-magnitude row: label + five preset buttons + custom box.

    Returns the laid-out row and the controller object that exposes the
    currently selected step via ``current()``.
    """
    grp = _StepGroup(magnitudes, default, on_change, unit=unit)
    row = QHBoxLayout()
    row.setSpacing(_sc(4))
    row.setContentsMargins(0, 0, 0, 0)

    lbl = QLabel(label)
    lbl.setStyleSheet(
        f"color: {_FG_MUTED}; font-weight: 600; font-size: {_sf(9.5)}pt;")
    lbl.setMinimumWidth(_sc(28))
    row.addWidget(lbl)

    for btn in grp.buttons:
        row.addWidget(btn)
    row.addWidget(grp.custom_edit, 1)
    return row, grp


class _StepGroup:
    """Five preset buttons + a custom-value QLineEdit acting as one selector.

    Exactly one button is "active" at any time. Typing in the custom
    box deselects all preset buttons and uses the entered value.
    """

    _PRESET_STYLE = (
        f"QPushButton {{"
        f"  background-color: {_BG};"
        f"  border: 1px solid {_BG_HOVER};"
        f"  color: {_FG};"
        f"  padding: 2px 4px;"
        f"  border-radius: 4px;"
        f"  min-width: 36px;"
        f"  font-family: monospace;"
        f"}}"
        f"QPushButton:hover {{ background-color: {_BG_HOVER}; }}"
        f"QPushButton:checked {{"
        f"  background-color: {_ACCENT};"
        f"  border-color: {_ACCENT};"
        f"  color: {COLORS.get('base', '#1e1e2e')};"
        f"  font-weight: 700;"
        f"}}"
    )

    _CUSTOM_STYLE = (
        f"QLineEdit {{"
        f"  background-color: {_BG};"
        f"  border: 1px solid {_BG_HOVER};"
        f"  color: {_FG};"
        f"  padding: 2px 6px;"
        f"  border-radius: 4px;"
        f"}}"
        f"QLineEdit:focus {{ border-color: {_ACCENT}; }}"
    )

    def __init__(self, magnitudes: tuple[float, ...], default: float,
                 on_change, unit: str):
        self.magnitudes = magnitudes
        self._value = default
        self._on_change = on_change
        self.buttons: list[QPushButton] = []
        self._btn_group = QButtonGroup()
        self._btn_group.setExclusive(True)
        for mag in magnitudes:
            btn = QPushButton(self._format(mag))
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setStyleSheet(self._PRESET_STYLE)
            btn.setFixedHeight(_sc(28))
            btn.clicked.connect(partial(self._pick_preset, mag))
            if mag == default:
                btn.setChecked(True)
            self.buttons.append(btn)
            self._btn_group.addButton(btn)
        self.custom_edit = QLineEdit()
        self.custom_edit.setPlaceholderText(f"custom {unit}")
        self.custom_edit.setMaximumWidth(_sc(80))
        self.custom_edit.setFixedHeight(_sc(28))
        self.custom_edit.setStyleSheet(self._CUSTOM_STYLE)
        self.custom_edit.setValidator(QDoubleValidator(0.0001, 1e9, 6))
        self.custom_edit.textEdited.connect(self._on_custom_text)

    @staticmethod
    def _format(mag: float) -> str:
        if mag >= 1:
            return f"{int(mag)}"
        return f"{mag:g}"

    def _pick_preset(self, mag: float, _checked: bool = True) -> None:
        self._value = mag
        # Clear the custom box so the picked value is unambiguous.
        if self.custom_edit.text():
            self.custom_edit.blockSignals(True)
            self.custom_edit.clear()
            self.custom_edit.blockSignals(False)
        if self._on_change:
            self._on_change(mag)

    def _on_custom_text(self, text: str) -> None:
        if not text:
            return
        try:
            val = float(text)
        except ValueError:
            return
        if val <= 0:
            return
        self._value = val
        # Uncheck all presets — the custom box wins.
        for btn in self.buttons:
            btn.setChecked(False)
        if self._on_change:
            self._on_change(val)

    def current(self) -> float:
        """Return the currently selected step magnitude."""
        return self._value


class JogButtonArray(QWidget):
    """Jog control with magnitude step selectors above the direction pads.

    Signals:
        jog_xy_requested(float, float): (dx_um, dy_um) relative XY move.
        jog_z_requested(float): dz_mm relative Z move.
        jog_pump_requested(str, float): (pump_id, distance_uL) pump move.
        home_requested(): Move-to-zero requested.
    """

    jog_xy_requested = Signal(float, float)
    jog_z_requested = Signal(float)
    jog_pump_requested = Signal(str, float)
    home_requested = Signal()

    def __init__(self, compact: bool = False, show_pumps: bool = False,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._compact = compact
        self._show_pumps = show_pumps
        # Whether the pump axis is in µL mode (display unit). Step
        # magnitudes are always 1/10/100/1000/10000 of the native unit.
        self._pump_step_is_uL = True
        self._setup_ui()

    # ── Public step-value accessors ────────────────────────────

    @property
    def xy_step_um(self) -> float:
        return float(self._xy_step.current())

    @property
    def z_step_mm(self) -> float:
        # User-facing magnitudes are 1/10/100/1000/10000 µm — convert
        # to mm for the Z signal.
        return float(self._z_step.current()) / 1000.0

    @property
    def pump_step(self) -> float:
        return float(self._p_step.current())

    def set_pump_step_mode(self, use_uL: bool) -> None:
        """Compatibility shim — preset magnitudes don't change because
        the order-of-magnitude buttons cover both µL and mm ranges."""
        self._pump_step_is_uL = use_uL

    # ── UI ─────────────────────────────────────────────────────

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(_sc(8))

        # ── Step magnitude selectors (above the controls) ───────
        steps_box = QVBoxLayout()
        steps_box.setSpacing(_sc(4))
        steps_box.setContentsMargins(0, 0, 0, 0)

        xy_row, self._xy_step = _build_step_row(
            "XY", "µm", _MAGNITUDES, 100.0, on_change=None)
        z_row, self._z_step = _build_step_row(
            "Z", "µm", _MAGNITUDES, 100.0, on_change=None)
        steps_box.addLayout(xy_row)
        steps_box.addLayout(z_row)
        if self._show_pumps:
            p_row, self._p_step = _build_step_row(
                "P", "µL", _MAGNITUDES, 10.0, on_change=None)
            steps_box.addLayout(p_row)

        layout.addLayout(steps_box)

        # ── Direction controls ──────────────────────────────────
        btn_size = _sc(36) if self._compact else _sc(44)

        controls = QGridLayout()
        controls.setSpacing(_sc(3))

        # XY pad (cols 0-2, rows 0-2)
        btn_up = self._dir_btn("▲", btn_size)
        btn_up.clicked.connect(partial(self._on_xy, 0, -1))
        controls.addWidget(btn_up, 0, 1)

        btn_left = self._dir_btn("◀", btn_size)
        btn_left.clicked.connect(partial(self._on_xy, -1, 0))
        controls.addWidget(btn_left, 1, 0)

        btn_home = self._dir_btn("⌂", btn_size)
        btn_home.setToolTip("Go to zero reference")
        btn_home.clicked.connect(self.home_requested.emit)
        controls.addWidget(btn_home, 1, 1)

        btn_right = self._dir_btn("▶", btn_size)
        btn_right.clicked.connect(partial(self._on_xy, 1, 0))
        controls.addWidget(btn_right, 1, 2)

        btn_down = self._dir_btn("▼", btn_size)
        btn_down.clicked.connect(partial(self._on_xy, 0, 1))
        controls.addWidget(btn_down, 2, 1)

        # Z column (col 3) — stacked up/down beside the XY pad.
        btn_z_up = self._dir_btn("Z ▲", btn_size)
        btn_z_up.setToolTip("Move Z up")
        btn_z_up.clicked.connect(partial(self._on_z, -1))
        controls.addWidget(btn_z_up, 0, 3)

        btn_z_down = self._dir_btn("Z ▼", btn_size)
        btn_z_down.setToolTip("Move Z down")
        btn_z_down.clicked.connect(partial(self._on_z, 1))
        controls.addWidget(btn_z_down, 2, 3)

        layout.addLayout(controls)

        # ── Pump columns (P1▲/▼, P2▲/▼, P3▲/▼) — each pump stacked ─
        if self._show_pumps:
            pump_row = QHBoxLayout()
            pump_row.setSpacing(_sc(6))
            self._pump_buttons = {}
            for pump_id in ("P1", "P2", "P3"):
                col = QVBoxLayout()
                col.setSpacing(_sc(3))
                col.setContentsMargins(0, 0, 0, 0)
                btn_ext = self._dir_btn(f"{pump_id} ▲", btn_size)
                btn_ext.setToolTip(f"Extend {pump_id}")
                btn_ext.clicked.connect(partial(self._on_pump, pump_id, 1))
                col.addWidget(btn_ext)
                btn_ret = self._dir_btn(f"{pump_id} ▼", btn_size)
                btn_ret.setToolTip(f"Retract {pump_id}")
                btn_ret.clicked.connect(partial(self._on_pump, pump_id, -1))
                col.addWidget(btn_ret)
                pump_row.addLayout(col)
                self._pump_buttons[pump_id] = (btn_ext, btn_ret)
            pump_row.addStretch(1)
            layout.addLayout(pump_row)

    def _dir_btn(self, text: str, btn_size: int) -> QPushButton:
        btn = QPushButton(text)
        btn.setFixedSize(btn_size, btn_size)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: {_BG};"
            f"  border: 1px solid {_BG_HOVER};"
            f"  border-radius: 6px;"
            f"  color: {_FG};"
            f"  font-weight: 600;"
            f"  font-size: {_sf(11)}pt;"
            f"}}"
            f"QPushButton:hover {{ background-color: {_BG_HOVER}; }}"
            f"QPushButton:pressed {{ background-color: {_BG_ACTIVE}; }}"
        )
        return btn

    # ── Signal handlers ────────────────────────────────────────

    def _on_xy(self, dx: int, dy: int):
        step = self.xy_step_um
        self.jog_xy_requested.emit(dx * step, dy * step)

    def _on_z(self, direction: int):
        step = self.z_step_mm
        self.jog_z_requested.emit(direction * step)

    def _on_pump(self, pump_id: str, direction: int):
        step = self.pump_step
        self.jog_pump_requested.emit(pump_id, direction * step)
