"""
JogButtonArray — Reusable jog control widget with XY pad, Z buttons, and step selectors.

v7.4.2: redesigned layout per user direction.

  * Step sizes for each axis (XY, Z, Pump) sit ABOVE the motion
    controls. Each axis row gets five order-of-magnitude buttons
    (1 / 10 / 100 / 1000 / 10000) in the axis's native unit plus a
    custom-value text box so the user can dial in any step.
  * XY direction pad in the middle with Z up/down stacked beside it.
  * Pump axes are stacked so each pump reads as a single column.

v7.5.x: **fit-to-width, no scrolling, aligned**. The three step rows share ONE
``QGridLayout`` so the axis labels and the five magnitude buttons line up in
tidy columns (a segmented-control look). The magnitude buttons + custom fields +
pump columns are *expanding*, so the row always fills — and fits — the panel
width (no horizontal scrolling); on a narrow panel the buttons scrunch and every
label / button / field **font shrinks with the width** (``container_scale``) so
the text stays inside its control instead of clipping. The XY/Z direction pad is
a centred, uniformly-scaled cluster.

Signals emitted: ``jog_xy_requested(dx_um, dy_um)``,
``jog_z_requested(dz_mm)`` (height frame, + = up — route through
``move_z_user_relative``), ``jog_pump_requested(pump_id, dist)``,
``home_requested()``. Parent wires them to the controller.
"""

from __future__ import annotations

from functools import partial

from PySide6.QtCore import Qt, QSize, Signal
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QButtonGroup, QGridLayout, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget,
)

from gui.scaling import scale_factor as _dpi
from gui.styles import COLORS
from gui.widgets.responsive import container_scale, quantize

# Order-of-magnitude step sizes per axis (v7.4.2).
_MAGNITUDES = (1.0, 10.0, 100.0, 1000.0, 10000.0)

# v7.5.x: pump step magnitudes as a % of the syringe volume (%/µL pages).
_PUMP_PERCENT_MAGNITUDES = (0.1, 0.5, 1.0, 5.0, 10.0)

# ── Base (design-time, DPI × responsive 1.0) dimensions ────────────────────
# All sizing lives here so the "css-like" scaling is centralised. The widths are
# almost entirely PROPORTIONAL: every row item (magnitude buttons, custom field,
# pump buttons) is Expanding with an equal column stretch, so each takes a %% of
# the panel width and the row always fits (down to the 100 px minimum). The tiny
# *minimum* widths below only stop a control collapsing to nothing — they do NOT
# drive the layout. Heights + fonts + the (square) direction-pad buttons scale
# with the width via container_scale(). Actual = base × scale_factor() × scale.
_BASE_LABEL_W = 18        # axis-label column min width (keeps XY/Z/P aligned)
_BASE_PRESET_MIN_W = 5    # magnitude button min width (stretch drives the rest)
_BASE_CUSTOM_MIN_W = 12
_BASE_ROW_H = 26          # step-button / custom-edit height
_BASE_DIR_COMPACT = 34
_BASE_DIR_FULL = 42
_BASE_PUMP_MIN_W = 6      # pump button min width (stretch drives the rest)
_FONT_LABEL = 9.5
_FONT_PRESET = 9.0
_FONT_DIR = 11.0
_FONT_ACTION = 9.5

# Width (base px) the array targets at responsive scale 1.0. Chosen so the
# comfortable default context-panel width renders ~natural size; narrower than
# this shrinks toward the floor, wider grows toward the cap. The floor is low so
# text keeps getting smaller down to the 100 px minimum (operator: "its ok to
# scrunch the buttons, we just need to make sure the text gets smaller as well").
_DESIGN_BASE = 470
_MIN_SCALE = 0.42
_MAX_SCALE = 1.15
_MIN_FONT_PT = 4.5        # absolute floor so text never fully vanishes

# Catppuccin Mocha palette pieces.
_BG = COLORS.get("surface0", "#313244")
_BG_HOVER = COLORS.get("surface1", "#45475a")
_BG_ACTIVE = COLORS.get("surface2", "#585b70")
_FG = COLORS.get("text", "#cdd6f4")
_FG_MUTED = COLORS.get("subtext0", "#a6adc8")
_ACCENT = COLORS.get("mauve", "#cba6f7")
_BASE = COLORS.get("base", "#1e1e2e")


def _set_pt(widget: QWidget, pt: float) -> None:
    """Set only the point size of a widget's font, preserving its family."""
    f = widget.font()
    f.setPointSizeF(max(_MIN_FONT_PT, pt))
    widget.setFont(f)


class _StepGroup:
    """Five preset buttons + a custom-value QLineEdit acting as one selector.

    Exactly one button is "active" at any time. Typing in the custom
    box deselects all preset buttons and uses the entered value.
    """

    # v7.5.x: font-size + min-width removed from the QSS so the widget font
    # (set via _set_pt) and setMinimumWidth can be rescaled at runtime.
    _PRESET_STYLE = (
        f"QPushButton {{"
        f"  background-color: {_BG};"
        f"  border: 1px solid {_BG_HOVER};"
        f"  color: {_FG};"
        f"  padding: 1px 2px;"
        f"  border-radius: 4px;"
        f"}}"
        f"QPushButton:hover {{ background-color: {_BG_HOVER}; }}"
        f"QPushButton:checked {{"
        f"  background-color: {_ACCENT};"
        f"  border-color: {_ACCENT};"
        f"  color: {_BASE};"
        f"  font-weight: 700;"
        f"}}"
    )

    _CUSTOM_STYLE = (
        f"QLineEdit {{"
        f"  background-color: {_BG};"
        f"  border: 1px solid {_BG_HOVER};"
        f"  color: {_FG};"
        f"  padding: 1px 5px;"
        f"  border-radius: 4px;"
        f"}}"
        f"QLineEdit:focus {{ border-color: {_ACCENT}; }}"
    )

    # Mirrors the :checked preset look so the custom box reads as the
    # active step size even after it loses keyboard focus.
    _CUSTOM_ACTIVE_STYLE = (
        f"QLineEdit {{"
        f"  background-color: {_ACCENT};"
        f"  border: 1px solid {_ACCENT};"
        f"  color: {_BASE};"
        f"  padding: 1px 5px;"
        f"  border-radius: 4px;"
        f"  font-weight: 700;"
        f"}}"
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
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(partial(self._pick_preset, mag))
            if mag == default:
                btn.setChecked(True)
            self.buttons.append(btn)
            self._btn_group.addButton(btn)
        self.custom_edit = QLineEdit()
        self.custom_edit.setPlaceholderText(f"± {unit}")
        self.custom_edit.setStyleSheet(self._CUSTOM_STYLE)
        self.custom_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.custom_edit.setValidator(QDoubleValidator(0.0001, 1e9, 6))
        self.custom_edit.textEdited.connect(self._on_custom_text)

    @staticmethod
    def _format(mag: float) -> str:
        if mag >= 1:
            return f"{int(mag)}"
        return f"{mag:g}"

    def _set_custom_active(self, active: bool) -> None:
        self.custom_edit.setStyleSheet(
            self._CUSTOM_ACTIVE_STYLE if active else self._CUSTOM_STYLE)

    def _pick_preset(self, mag: float, _checked: bool = True) -> None:
        self._value = mag
        if self.custom_edit.text():
            self.custom_edit.blockSignals(True)
            self.custom_edit.clear()
            self.custom_edit.blockSignals(False)
        self._set_custom_active(False)
        if self._on_change:
            self._on_change(mag)

    def _on_custom_text(self, text: str) -> None:
        if not text:
            self._set_custom_active(False)
            return
        try:
            val = float(text)
        except ValueError:
            return
        if val <= 0:
            return
        self._value = val
        for btn in self.buttons:
            btn.setChecked(False)
        self._set_custom_active(True)
        if self._on_change:
            self._on_change(val)

    def current(self) -> float:
        return self._value


class JogButtonArray(QWidget):
    """Jog control with magnitude step selectors above the direction pads.

    Signals:
        jog_xy_requested(float, float): (dx_um, dy_um) relative XY move.
        jog_z_requested(float): dz_mm relative Z move in the HEIGHT frame
            (+ = up, toward the taught Top). v7.5.x: consumers must convert to
            a raw move via StageController.move_z_user_relative (z_up_sign), NOT
            pass it straight to move_z_relative.
        jog_pump_requested(str, float): (pump_id, distance_uL) pump move.
        home_requested(): Move-to-zero requested.
    """

    jog_xy_requested = Signal(float, float)
    jog_z_requested = Signal(float)
    jog_pump_requested = Signal(str, float)
    home_requested = Signal()

    def __init__(self, compact: bool = False, show_pumps: bool = False,
                 parent: QWidget | None = None, *,
                 pump_action_labels: bool = False):
        super().__init__(parent)
        self._compact = compact
        self._show_pumps = show_pumps
        self._pump_action_labels = pump_action_labels
        self._pump_step_is_uL = True
        self._pump_step_is_percent = pump_action_labels
        self._dir_base = _BASE_DIR_COMPACT if compact else _BASE_DIR_FULL

        # v7.5.x: responsive-sizing registries + current per-container scale.
        self._rscale = 1.0
        self._scale_labels: list[tuple[QLabel, float]] = []
        self._scale_presets: list[QPushButton] = []
        self._scale_customs: list[QLineEdit] = []
        self._scale_dir: list[QPushButton] = []
        self._scale_pump_words: list[QPushButton] = []
        self._scale_pump_arrows: list[QPushButton] = []
        self._axis_labels: list[QLabel] = []

        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self._setup_ui()
        self._apply_scale(1.0)

    # ── Public step-value accessors ────────────────────────────

    @property
    def xy_step_um(self) -> float:
        return float(self._xy_step.current())

    @property
    def z_step_mm(self) -> float:
        return float(self._z_step.current()) / 1000.0

    @property
    def pump_step(self) -> float:
        if self._pump_step_is_percent:
            return float(self._p_step.current())
        return float(self._p_step.current()) / 1000.0

    @property
    def pump_step_is_percent(self) -> bool:
        return self._pump_step_is_percent

    def set_pump_step_mode(self, use_uL: bool) -> None:
        self._pump_step_is_uL = use_uL

    # ── UI ─────────────────────────────────────────────────────

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(int(round(10 * _dpi())))

        # ── Step selectors — ONE shared grid so XY / Z / P align in columns.
        # col 0 = axis label; cols 1-5 = magnitude buttons; col 6 = custom.
        steps = QGridLayout()
        steps.setContentsMargins(0, 0, 0, 0)
        steps.setHorizontalSpacing(int(round(4 * _dpi())))
        steps.setVerticalSpacing(int(round(4 * _dpi())))

        r = 0
        self._xy_step = self._add_step_row(steps, r, "XY", "µm",
                                           _MAGNITUDES, 100.0); r += 1
        self._z_step = self._add_step_row(steps, r, "Z", "µm",
                                          _MAGNITUDES, 100.0); r += 1
        if self._show_pumps:
            if self._pump_step_is_percent:
                self._p_step = self._add_step_row(
                    steps, r, "P", "%", _PUMP_PERCENT_MAGNITUDES, 1.0)
            else:
                self._p_step = self._add_step_row(
                    steps, r, "P", "µm", _MAGNITUDES, 100.0)
            r += 1

        # Buttons share width evenly; custom gets a touch more.
        steps.setColumnStretch(0, 0)
        for c in range(1, 6):
            steps.setColumnStretch(c, 1)
        steps.setColumnStretch(6, 2)
        root.addLayout(steps)

        # ── Direction pad (XY) + Z column, centred as one cluster. ──
        root.addLayout(self._build_dir_pad())

        # ── Pump columns (aligned, expanding to fill width). ──
        if self._show_pumps and self._pump_action_labels:
            root.addLayout(self._build_pump_columns(action=True))
        elif self._show_pumps:
            root.addLayout(self._build_pump_columns(action=False))

    def _add_step_row(self, grid: QGridLayout, row: int, label: str,
                      unit: str, magnitudes: tuple[float, ...],
                      default: float) -> _StepGroup:
        grp = _StepGroup(magnitudes, default, None, unit=unit)

        lbl = QLabel(label)
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl.setStyleSheet(f"color: {_FG_MUTED}; font-weight: 600;")
        grid.addWidget(lbl, row, 0)
        self._scale_labels.append((lbl, _FONT_LABEL))
        self._axis_labels.append(lbl)

        for i, btn in enumerate(grp.buttons):
            grid.addWidget(btn, row, 1 + i)
            self._scale_presets.append(btn)
        grid.addWidget(grp.custom_edit, row, 6)
        self._scale_customs.append(grp.custom_edit)
        return grp

    def _build_dir_pad(self) -> QGridLayout:
        pad = QGridLayout()
        pad.setContentsMargins(0, 0, 0, 0)
        pad.setSpacing(int(round(4 * _dpi())))
        # Centre the cluster: stretch on the outer columns.
        pad.setColumnStretch(0, 1)
        pad.setColumnStretch(6, 1)

        # XY pad in cols 1-3.
        btn_up = self._dir_btn("▲")
        btn_up.clicked.connect(partial(self._on_xy, 0, -1))
        pad.addWidget(btn_up, 0, 2)

        btn_left = self._dir_btn("◀")
        btn_left.clicked.connect(partial(self._on_xy, -1, 0))
        pad.addWidget(btn_left, 1, 1)

        btn_home = self._dir_btn("⌂")
        btn_home.setToolTip("Go to zero reference")
        btn_home.clicked.connect(self.home_requested.emit)
        pad.addWidget(btn_home, 1, 2)

        btn_right = self._dir_btn("▶")
        btn_right.clicked.connect(partial(self._on_xy, 1, 0))
        pad.addWidget(btn_right, 1, 3)

        btn_down = self._dir_btn("▼")
        btn_down.clicked.connect(partial(self._on_xy, 0, 1))
        pad.addWidget(btn_down, 2, 2)

        # Z column in col 5 (col 4 is a small gap between pad and Z).
        # v7.5.x: HEIGHT-frame delta (+ = up). Consumers convert via
        # move_z_user_relative (z_up_sign) so "up" always retracts the needle.
        btn_z_up = self._dir_btn("Z ▲")
        btn_z_up.setToolTip("Move Z up")
        btn_z_up.clicked.connect(partial(self._on_z, 1))
        pad.addWidget(btn_z_up, 0, 5)

        btn_z_down = self._dir_btn("Z ▼")
        btn_z_down.setToolTip("Move Z down")
        btn_z_down.clicked.connect(partial(self._on_z, -1))
        pad.addWidget(btn_z_down, 2, 5)

        pad.setColumnMinimumWidth(4, int(round(6 * _dpi())))
        return pad

    def _build_pump_columns(self, *, action: bool) -> QGridLayout:
        """Three pump columns in a grid (equal stretch → fill width, aligned).

        action=True  → ASPIRATE (top, emits −) / DISPENSE (bottom, emits +).
        action=False → P{n} ▲ extend (+) / P{n} ▼ retract (−) (Hardware Setup).
        The emitted sign convention matches StageController.move_pump_uL.
        """
        pg = QGridLayout()
        pg.setContentsMargins(0, 0, 0, 0)
        pg.setHorizontalSpacing(int(round(6 * _dpi())))
        pg.setVerticalSpacing(int(round(4 * _dpi())))
        self._pump_buttons = {}
        for j, pump_id in enumerate(("P1", "P2", "P3")):
            pg.setColumnStretch(j, 1)
            if action:
                head = QLabel(pump_id)
                head.setAlignment(Qt.AlignCenter)
                head.setStyleSheet(f"color: {_FG_MUTED}; font-weight: 700;")
                pg.addWidget(head, 0, j)
                self._scale_labels.append((head, _FONT_LABEL))
                btn_asp = self._pump_btn("Aspirate", word=True)
                btn_asp.setToolTip(
                    f"Aspirate {pump_id} — draw fluid IN (raises fill)")
                btn_asp.clicked.connect(partial(self._on_pump, pump_id, -1))
                pg.addWidget(btn_asp, 1, j)
                btn_disp = self._pump_btn("Dispense", word=True)
                btn_disp.setToolTip(
                    f"Dispense {pump_id} — push fluid OUT (lowers fill)")
                btn_disp.clicked.connect(partial(self._on_pump, pump_id, 1))
                pg.addWidget(btn_disp, 2, j)
                self._pump_buttons[pump_id] = (btn_asp, btn_disp)
            else:
                btn_ext = self._pump_btn(f"{pump_id} ▲", word=False)
                btn_ext.setToolTip(f"Extend {pump_id}")
                btn_ext.clicked.connect(partial(self._on_pump, pump_id, 1))
                pg.addWidget(btn_ext, 0, j)
                btn_ret = self._pump_btn(f"{pump_id} ▼", word=False)
                btn_ret.setToolTip(f"Retract {pump_id}")
                btn_ret.clicked.connect(partial(self._on_pump, pump_id, -1))
                pg.addWidget(btn_ret, 1, j)
                self._pump_buttons[pump_id] = (btn_ext, btn_ret)
        return pg

    def _pump_btn(self, text: str, *, word: bool) -> QPushButton:
        btn = QPushButton(text)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn.setStyleSheet(self._jog_qss())
        if word:
            self._scale_pump_words.append(btn)
        else:
            self._scale_pump_arrows.append(btn)
        return btn

    def _dir_btn(self, text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setStyleSheet(self._jog_qss())
        self._scale_dir.append(btn)
        return btn

    @staticmethod
    def _jog_qss() -> str:
        # No font-size here — set (and rescaled) via _apply_scale.
        return (
            f"QPushButton {{"
            f"  background-color: {_BG};"
            f"  border: 1px solid {_BG_HOVER};"
            f"  border-radius: 6px;"
            f"  color: {_FG};"
            f"  font-weight: 600;"
            f"}}"
            f"QPushButton:hover {{ background-color: {_BG_HOVER}; }}"
            f"QPushButton:pressed {{ background-color: {_BG_ACTIVE}; }}"
        )

    # ── Responsive sizing (fit-to-width, no scroll) ─────────────

    def minimumSizeHint(self):  # noqa: N802 (Qt override)
        # Report a small minimum WIDTH so a parent scroll area / splitter can
        # drive this narrow; resizeEvent then rescales the direction pad + step
        # rows to fit whatever width they're given (they're all proportional).
        # Without this the pad's per-scale fixed buttons would floor the width.
        h = super().minimumSizeHint().height()
        return QSize(round(60 * _dpi()), h)

    def _px(self, base: float) -> int:
        return max(1, round(base * _dpi() * self._rscale))

    def _pt(self, base: float) -> float:
        return max(_MIN_FONT_PT, base * _dpi() * self._rscale)

    def resizeEvent(self, event):  # noqa: N802 (Qt override)
        super().resizeEvent(event)
        w = self.width()
        if w <= 0:
            return
        scale = quantize(container_scale(
            w, round(_DESIGN_BASE * _dpi()), _MIN_SCALE, _MAX_SCALE))
        if abs(scale - self._rscale) < 1e-6:
            return
        self._apply_scale(scale)

    def _apply_scale(self, scale: float) -> None:
        """Recompute every control's size + font for the per-container scale so
        the content fits the width (buttons scrunch, text shrinks with them)."""
        self._rscale = scale
        row_h = self._px(_BASE_ROW_H)
        label_w = self._px(_BASE_LABEL_W)
        dir_sz = self._px(self._dir_base)

        for lbl, base_pt in self._scale_labels:
            _set_pt(lbl, self._pt(base_pt))
        for lbl in self._axis_labels:
            lbl.setFixedWidth(label_w)

        for btn in self._scale_presets:
            btn.setMinimumWidth(self._px(_BASE_PRESET_MIN_W))
            btn.setFixedHeight(row_h)
            _set_pt(btn, self._pt(_FONT_PRESET))

        for edit in self._scale_customs:
            edit.setMinimumWidth(self._px(_BASE_CUSTOM_MIN_W))
            edit.setFixedHeight(row_h)
            _set_pt(edit, self._pt(_FONT_PRESET))

        for btn in self._scale_dir:
            btn.setFixedSize(dir_sz, dir_sz)
            _set_pt(btn, self._pt(_FONT_DIR))

        for btn in self._scale_pump_arrows:
            btn.setMinimumWidth(self._px(_BASE_PUMP_MIN_W))
            btn.setFixedHeight(dir_sz)
            _set_pt(btn, self._pt(_FONT_DIR))

        for btn in self._scale_pump_words:
            btn.setMinimumWidth(self._px(_BASE_PUMP_MIN_W))
            btn.setFixedHeight(dir_sz)
            _set_pt(btn, self._pt(_FONT_ACTION))

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
