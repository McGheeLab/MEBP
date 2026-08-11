"""
JogButtonArray — Reusable jog control widget with XY pad, Z buttons, and step selectors.

v7.4.2: redesigned layout per user direction.

  * Step sizes for each axis (XY, Z, Pump) sit ABOVE the motion controls.
    v7.9.x: each axis row is a **log-scale step SLIDER** with setpoint
    detents (operator: *"lets make a slider bar for each axis that we want
    to move by with setpoints close to 0.001, 0.005, 0.01, 0.05, 0.1, 0.5,
    1, 5, 10 µL"*) plus a custom-value box that doubles as the live readout;
    the five order-of-magnitude preset buttons are retired. A collapsible
    "Step slider settings" section (collapsed by default) holds the
    snap-to-nearest-setpoint checkbox and a custom min/max range per axis.
  * XY direction pad in the middle with Z up/down stacked beside it.
  * Pump axes are stacked so each pump reads as a single column.

v7.5.x: **fit-to-width, no scrolling, aligned**. The three step rows share ONE
``QGridLayout`` so the axis labels and the sliders line up in tidy columns.
The sliders + custom fields + pump columns are *expanding*, so the row always
fills — and fits — the panel width (no horizontal scrolling); on a narrow
panel the controls scrunch and every label / button / field **font shrinks
with the width** (``container_scale``) so the text stays inside its control
instead of clipping. The XY/Z direction pad is a centred, uniformly-scaled
cluster.

Signals emitted: ``jog_xy_requested(dx_um, dy_um)``,
``jog_z_requested(dz_mm)`` (height frame, + = up — route through
``move_z_user_relative``), ``jog_pump_requested(pump_id, dist)`` — v7.9.x:
``dist`` is a SIGNED µL volume on the %/µL jog pages (− = aspirate,
+ = dispense, matching ``move_pump_uL``; it was a % of syringe volume
before, which cannot express the sub-µL steps the operator asked for) and a
signed mm plunger distance on Hardware Setup — and ``home_requested()``.
Parent wires them to the controller.
"""

from __future__ import annotations

import math
from functools import partial

from PySide6.QtCore import Qt, QSize, Signal
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QCheckBox, QGridLayout, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)

from gui.scaling import scale_factor as _dpi
from gui.styles import COLORS
from gui.widgets.responsive import container_scale, quantize

# v7.9.x: step-slider setpoint ladders (the slider detents), log-spaced
# 1–5 per decade in each axis's native unit.
_XYZ_STEP_SETPOINTS_UM = (
    1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0)
# Pump steps are µL-NATIVE on the %/µL jog pages — the operator asked for
# volumes down to 0.001 µL, which a % of the syringe volume cannot express
# usefully (0.001 µL is 0.0004 % of a 250 µL syringe).
_PUMP_STEP_SETPOINTS_UL = (
    0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0)

# ── Base (design-time, DPI × responsive 1.0) dimensions ────────────────────
# All sizing lives here so the "css-like" scaling is centralised. The widths are
# almost entirely PROPORTIONAL: every row item (magnitude buttons, custom field,
# pump buttons) is Expanding with an equal column stretch, so each takes a %% of
# the panel width and the row always fits (down to the 100 px minimum). The tiny
# *minimum* widths below only stop a control collapsing to nothing — they do NOT
# drive the layout. Heights + fonts + the (square) direction-pad buttons scale
# with the width via container_scale(). Actual = base × scale_factor() × scale.
_BASE_LABEL_W = 18        # axis-label column min width (keeps XY/Z/P aligned)
_BASE_SLIDER_MIN_W = 24   # step-slider min width (stretch drives the rest)
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


class _StepSlider:
    """v7.9.x: a log-scale step SLIDER + a custom-value box acting as one
    step selector (replaces the five preset magnitude buttons — the operator
    asked for a slider per axis with setpoint detents down to 0.001 µL).

    Two slider models, switched by the snap setting:

      * snap ON (default): one slider position per SETPOINT inside the
        current range, a tick under each — the handle lands exactly on
        0.001 / 0.005 / … and never between.
      * snap OFF: ``_FREE_STEPS`` positions log-interpolated across the
        range (rounded to 3 significant figures for a readable readout).

    The custom box doubles as the live readout; typing any positive value
    makes it the step — even outside the slider's range, where the slider
    just clamps its displayed position (the typed value is what moves).
    The range is user-adjustable (``set_range``) from the collapsible
    step-settings section.
    """

    _FREE_STEPS = 1000

    # v7.5.x: font-size + min-width removed from the QSS so the widget font
    # (set via _set_pt) and setMinimumWidth can be rescaled at runtime.
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

    _SLIDER_STYLE = (
        f"QSlider::groove:horizontal {{"
        f"  background: {_BG};"
        f"  height: 6px;"
        f"  border-radius: 3px;"
        f"}}"
        f"QSlider::sub-page:horizontal {{"
        f"  background: {_BG_ACTIVE};"
        f"  border-radius: 3px;"
        f"}}"
        f"QSlider::handle:horizontal {{"
        f"  background: {_ACCENT};"
        f"  width: 12px;"
        f"  margin: -4px 0;"
        f"  border-radius: 6px;"
        f"}}"
    )

    def __init__(self, setpoints: tuple[float, ...], default: float,
                 on_change, unit: str):
        self.setpoints = tuple(sorted(float(p) for p in setpoints))
        self._lo = self.setpoints[0]
        self._hi = self.setpoints[-1]
        self._snap = True
        self._value = float(default)
        self._on_change = on_change
        self._updating = False
        self.unit = unit

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setCursor(Qt.PointingHandCursor)
        self.slider.setStyleSheet(self._SLIDER_STYLE)
        self.slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.slider.setToolTip(
            f"Step size ({unit}) — log scale. Snap-to-setpoint + range live "
            f"under 'Step slider settings'.")
        self.slider.valueChanged.connect(self._on_slider)

        self.custom_edit = QLineEdit()
        self.custom_edit.setPlaceholderText(f"± {unit}")
        self.custom_edit.setToolTip(
            f"Step size ({unit}). Type any positive value — even outside the "
            f"slider range; the typed value is what moves.")
        self.custom_edit.setStyleSheet(self._CUSTOM_STYLE)
        self.custom_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.custom_edit.setValidator(QDoubleValidator(1e-6, 1e9, 6))
        self.custom_edit.textEdited.connect(self._on_custom_text)

        self._rebuild_scale()
        self._sync_readout()

    # ── Range / setpoints ───────────────────────────────────────

    @property
    def snap(self) -> bool:
        return self._snap

    def range(self) -> tuple[float, float]:
        return self._lo, self._hi

    def active_points(self) -> list[float]:
        """The setpoints inside the current range, with the range endpoints
        always included so the ends stay reachable in snap mode."""
        pts = {p for p in self.setpoints if self._lo <= p <= self._hi}
        pts.update((self._lo, self._hi))
        return sorted(pts)

    def set_snap(self, snap: bool) -> None:
        snap = bool(snap)
        if snap == self._snap:
            return
        # The VALUE stays authoritative (a typed step must not be silently
        # re-snapped); only the slider's scale/position display changes.
        self._snap = snap
        self._rebuild_scale()

    def set_range(self, lo, hi) -> bool:
        """Custom slider range. Refused (False) unless 0 < lo < hi — a
        degenerate range would break the log scale."""
        try:
            lo, hi = float(lo), float(hi)
        except (TypeError, ValueError):
            return False
        if not (lo > 0.0 and hi > lo):
            return False
        self._lo, self._hi = lo, hi
        self._rebuild_scale()
        return True

    # ── Slider scale (position ↔ value) ─────────────────────────

    def _rebuild_scale(self) -> None:
        self._updating = True
        try:
            if self._snap:
                pts = self.active_points()
                self.slider.setRange(0, len(pts) - 1)
                self.slider.setSingleStep(1)
                self.slider.setPageStep(1)
                self.slider.setTickPosition(QSlider.TicksBelow)
                self.slider.setTickInterval(1)
            else:
                self.slider.setRange(0, self._FREE_STEPS)
                self.slider.setSingleStep(5)
                self.slider.setPageStep(50)
                self.slider.setTickPosition(QSlider.NoTicks)
            self.slider.setValue(self._value_to_pos(self._value))
        finally:
            self._updating = False

    def _pos_to_value(self, pos: int) -> float:
        if self._snap:
            pts = self.active_points()
            return pts[max(0, min(len(pts) - 1, int(pos)))]
        span = math.log(self._hi / self._lo)
        return self._lo * math.exp(span * pos / self._FREE_STEPS)

    def _value_to_pos(self, value: float) -> int:
        v = max(self._lo, min(self._hi, float(value)))
        if self._snap:
            pts = self.active_points()
            return min(range(len(pts)),
                       key=lambda i: abs(math.log(pts[i] / v)))
        span = math.log(self._hi / self._lo)
        if span <= 0:
            return 0
        return int(round(self._FREE_STEPS * math.log(v / self._lo) / span))

    # ── Handlers ────────────────────────────────────────────────

    def _on_slider(self, pos: int) -> None:
        if self._updating:
            return
        val = self._pos_to_value(pos)
        if not self._snap:
            # Free mode: 3 significant figures so the readout stays readable
            # ("setpoints close to …" — the ladder itself is exact).
            val = float(f"{val:.3g}")
        self._value = val
        self._sync_readout()
        if self._on_change:
            self._on_change(val)

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
        self._updating = True
        try:
            self.slider.setValue(self._value_to_pos(val))
        finally:
            self._updating = False
        if self._on_change:
            self._on_change(val)

    def _sync_readout(self) -> None:
        self.custom_edit.blockSignals(True)
        self.custom_edit.setText(f"{self._value:g}")
        self.custom_edit.blockSignals(False)

    def set_value(self, value: float) -> bool:
        """Adopt a step value (restore path). Any positive value is accepted
        even outside the slider range — the slider position just clamps."""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return False
        if value <= 0:
            return False
        self._value = value
        self._updating = True
        try:
            self.slider.setValue(self._value_to_pos(value))
        finally:
            self._updating = False
        self._sync_readout()
        return True

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
    # v7.9.x: the step sliders' configuration changed (snap / range / value).
    # The host panel persists it and shares it across every jog tile.
    step_settings_changed = Signal()

    def __init__(self, compact: bool = False, show_pumps: bool = False,
                 parent: QWidget | None = None, *,
                 pump_action_labels: bool = False):
        super().__init__(parent)
        self._compact = compact
        self._show_pumps = show_pumps
        self._pump_action_labels = pump_action_labels
        # v7.9.x: on the %/µL jog pages the pump step is a SIGNED µL volume
        # (was a % of syringe volume — see the module docstring).
        self._pump_step_is_uL_volume = pump_action_labels
        self._dir_base = _BASE_DIR_COMPACT if compact else _BASE_DIR_FULL

        # v7.5.x: responsive-sizing registries + current per-container scale.
        self._rscale = 1.0
        self._scale_labels: list[tuple[QLabel, float]] = []
        self._scale_sliders: list[QSlider] = []
        self._scale_customs: list[QLineEdit] = []
        self._scale_setting_edits: list[QLineEdit] = []
        self._scale_dir: list[QPushButton] = []
        self._scale_pump_words: list[QPushButton] = []
        self._scale_pump_arrows: list[QPushButton] = []
        self._axis_labels: list[QLabel] = []
        # v7.9.x: step groups by axis key ("XY"/"Z"/"P") for the settings
        # section (snap + custom range) and the public helpers.
        self._step_groups: dict[str, _StepSlider] = {}

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
        """One pump click's magnitude: µL on the %/µL jog pages (v7.9.x —
        was % of syringe), mm on Hardware Setup (µm entry ÷ 1000)."""
        if self._pump_step_is_uL_volume:
            return float(self._p_step.current())
        return float(self._p_step.current()) / 1000.0

    @property
    def pump_step_is_uL(self) -> bool:
        """True when ``jog_pump_requested`` distances are signed µL volumes
        (the %/µL jog pages); False on Hardware Setup's mm plunger jog."""
        return self._pump_step_is_uL_volume

    # ── v7.9.x: step-slider settings (snap + custom range) ─────

    def set_step_snap(self, snap: bool) -> None:
        """Snap-to-nearest-setpoint for every axis's step slider (also kept
        in sync with the settings checkbox)."""
        snap = bool(snap)
        for grp in self._step_groups.values():
            grp.set_snap(snap)
        chk = getattr(self, "_snap_check", None)
        if chk is not None and chk.isChecked() != snap:
            chk.blockSignals(True)
            chk.setChecked(snap)
            chk.blockSignals(False)

    @property
    def step_snap(self) -> bool:
        grps = self._step_groups
        return all(g.snap for g in grps.values()) if grps else True

    def set_step_range(self, axis: str, lo, hi) -> bool:
        """Custom slider range for one axis ("XY"/"Z"/"P"). Refused (False)
        unless 0 < lo < hi. Keeps the settings edits in sync."""
        grp = self._step_groups.get(axis)
        if grp is None or not grp.set_range(lo, hi):
            return False
        self._sync_range_edits(axis)
        return True

    def _sync_range_edits(self, axis: str) -> None:
        edits = getattr(self, "_range_edits", {}).get(axis)
        grp = self._step_groups.get(axis)
        if not edits or grp is None:
            return
        lo, hi = grp.range()
        for edit, val in zip(edits, (lo, hi)):
            edit.blockSignals(True)
            edit.setText(f"{val:g}")
            edit.blockSignals(False)

    # ── v7.9.x: persistable step-slider configuration ───────────

    def step_settings(self) -> dict:
        """The step sliders' full configuration — what the host panel stores
        so every jog tile agrees and the choice survives a restart."""
        return {
            "snap": self.step_snap,
            "axes": {axis: {"lo": grp.range()[0], "hi": grp.range()[1],
                            "value": grp.current()}
                     for axis, grp in self._step_groups.items()},
        }

    def apply_step_settings(self, cfg: dict | None) -> None:
        """Adopt a stored configuration. Silent + best-effort per field: a
        malformed/partial entry leaves that axis at its default rather than
        refusing the whole restore. Emits nothing (this is a restore, not an
        edit) so seeding can't loop back through the persist path."""
        if not isinstance(cfg, dict):
            return
        if "snap" in cfg:
            self.set_step_snap(bool(cfg.get("snap")))
        axes = cfg.get("axes")
        if not isinstance(axes, dict):
            return
        for axis, grp in self._step_groups.items():
            entry = axes.get(axis)
            if not isinstance(entry, dict):
                continue
            if entry.get("lo") is not None and entry.get("hi") is not None:
                if grp.set_range(entry["lo"], entry["hi"]):
                    self._sync_range_edits(axis)
            val = entry.get("value")
            if val is None:
                continue
            try:
                val = float(val)
            except (TypeError, ValueError):
                continue
            if val > 0:
                grp.set_value(val)

    # ── UI ─────────────────────────────────────────────────────

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(int(round(10 * _dpi())))

        # ── Step selectors — ONE shared grid so XY / Z / P align in columns.
        # v7.9.x: col 0 = axis label; col 1 = step slider; col 2 = the
        # value box (also the live readout); col 3 = unit.
        steps = QGridLayout()
        steps.setContentsMargins(0, 0, 0, 0)
        steps.setHorizontalSpacing(int(round(4 * _dpi())))
        steps.setVerticalSpacing(int(round(4 * _dpi())))

        r = 0
        emit = self.step_settings_changed.emit
        self._xy_step = self._add_step_row(
            steps, r, "XY", "µm", _XYZ_STEP_SETPOINTS_UM, 100.0, emit); r += 1
        self._z_step = self._add_step_row(
            steps, r, "Z", "µm", _XYZ_STEP_SETPOINTS_UM, 100.0, emit); r += 1
        if self._show_pumps:
            if self._pump_step_is_uL_volume:
                self._p_step = self._add_step_row(
                    steps, r, "P", "µL", _PUMP_STEP_SETPOINTS_UL, 1.0, emit)
            else:
                self._p_step = self._add_step_row(
                    steps, r, "P", "µm", _XYZ_STEP_SETPOINTS_UM, 100.0, emit)
            r += 1

        # The slider claims most of the row; the value box a readable share.
        steps.setColumnStretch(0, 0)
        steps.setColumnStretch(1, 5)
        steps.setColumnStretch(2, 2)
        steps.setColumnStretch(3, 0)
        root.addLayout(steps)

        # ── Step-slider settings — COLLAPSED by default (operator). ──
        root.addLayout(self._build_step_settings())

        # ── Direction pad (XY) + Z column, centred as one cluster. ──
        root.addLayout(self._build_dir_pad())

        # ── Pump columns (aligned, expanding to fill width). ──
        if self._show_pumps and self._pump_action_labels:
            root.addLayout(self._build_pump_columns(action=True))
        elif self._show_pumps:
            root.addLayout(self._build_pump_columns(action=False))

    def _add_step_row(self, grid: QGridLayout, row: int, label: str,
                      unit: str, setpoints: tuple[float, ...],
                      default: float, on_change=None) -> _StepSlider:
        grp = _StepSlider(setpoints, default,
                          (lambda _v: on_change()) if on_change else None,
                          unit=unit)
        self._step_groups[label] = grp

        lbl = QLabel(label)
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl.setStyleSheet(f"color: {_FG_MUTED}; font-weight: 600;")
        grid.addWidget(lbl, row, 0)
        self._scale_labels.append((lbl, _FONT_LABEL))
        self._axis_labels.append(lbl)

        grid.addWidget(grp.slider, row, 1)
        self._scale_sliders.append(grp.slider)
        grid.addWidget(grp.custom_edit, row, 2)
        self._scale_customs.append(grp.custom_edit)

        unit_lbl = QLabel(unit)
        unit_lbl.setStyleSheet(f"color: {_FG_MUTED};")
        grid.addWidget(unit_lbl, row, 3)
        self._scale_labels.append((unit_lbl, _FONT_PRESET))
        return grp

    # ── v7.9.x: collapsible step-slider settings ────────────────

    def _build_step_settings(self) -> QVBoxLayout:
        """Snap-to-setpoint + per-axis custom slider range, behind a
        disclosure that starts COLLAPSED (operator: *"These settings should
        be available via a collapsible settings section which is collapsed by
        default"*). A lightweight inline disclosure rather than a ``Card`` so
        it inherits this widget's own font scaling and adds no framing inside
        an already dense panel."""
        wrap = QVBoxLayout()
        wrap.setContentsMargins(0, 0, 0, 0)
        wrap.setSpacing(int(round(3 * _dpi())))

        self._settings_toggle = QPushButton("▸ Step slider settings")
        self._settings_toggle.setCursor(Qt.PointingHandCursor)
        self._settings_toggle.setFlat(True)
        self._settings_toggle.setStyleSheet(
            f"QPushButton {{ background: transparent; border: none;"
            f"  color: {_FG_MUTED}; text-align: left; padding: 0px; }}"
            f"QPushButton:hover {{ color: {_ACCENT}; }}")
        self._settings_toggle.setToolTip(
            "Snap-to-setpoint and the custom slider range for each axis.")
        self._settings_toggle.clicked.connect(self._toggle_step_settings)
        wrap.addWidget(self._settings_toggle)
        self._scale_labels.append((self._settings_toggle, _FONT_PRESET))

        body = QWidget()
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(int(round(10 * _dpi())), 0, 0, 0)
        body_lay.setSpacing(int(round(3 * _dpi())))

        self._snap_check = QCheckBox("Snap to nearest setpoint")
        self._snap_check.setChecked(True)
        self._snap_check.setToolTip(
            "On: the slider lands exactly on the setpoint ladder "
            "(…0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1, 5, 10…).\n"
            "Off: any value across the range (3 significant figures).\n"
            "A value typed in the box is always used as-is.")
        self._snap_check.setStyleSheet(f"color: {_FG};")
        self._snap_check.toggled.connect(self._on_snap_toggled)
        body_lay.addWidget(self._snap_check)
        self._scale_labels.append((self._snap_check, _FONT_PRESET))

        rng = QGridLayout()
        rng.setContentsMargins(0, 0, 0, 0)
        rng.setHorizontalSpacing(int(round(4 * _dpi())))
        rng.setVerticalSpacing(int(round(3 * _dpi())))
        hdr = QLabel("Slider range")
        hdr.setStyleSheet(f"color: {_FG_MUTED}; font-weight: 600;")
        rng.addWidget(hdr, 0, 0, 1, 4)
        self._scale_labels.append((hdr, _FONT_PRESET))

        self._range_edits: dict[str, tuple[QLineEdit, QLineEdit]] = {}
        for i, (axis, grp) in enumerate(self._step_groups.items(), start=1):
            lbl = QLabel(axis)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setStyleSheet(f"color: {_FG_MUTED}; font-weight: 600;")
            rng.addWidget(lbl, i, 0)
            self._scale_labels.append((lbl, _FONT_LABEL))
            self._axis_labels.append(lbl)
            lo_e, hi_e = self._range_edit(grp), self._range_edit(grp)
            rng.addWidget(lo_e, i, 1)
            rng.addWidget(hi_e, i, 2)
            unit_lbl = QLabel(grp.unit)
            unit_lbl.setStyleSheet(f"color: {_FG_MUTED};")
            rng.addWidget(unit_lbl, i, 3)
            self._scale_labels.append((unit_lbl, _FONT_PRESET))
            self._range_edits[axis] = (lo_e, hi_e)
            for e in (lo_e, hi_e):
                e.editingFinished.connect(partial(self._on_range_edited, axis))
            self._sync_range_edits(axis)
        rng.setColumnStretch(0, 0)
        rng.setColumnStretch(1, 1)
        rng.setColumnStretch(2, 1)
        rng.setColumnStretch(3, 0)
        body_lay.addLayout(rng)

        body.setVisible(False)          # collapsed by default
        self._settings_body = body
        wrap.addWidget(body)
        return wrap

    def _range_edit(self, grp: "_StepSlider") -> QLineEdit:
        e = QLineEdit()
        e.setStyleSheet(_StepSlider._CUSTOM_STYLE)
        e.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        e.setValidator(QDoubleValidator(1e-6, 1e9, 6))
        e.setToolTip(f"Slider min / max ({grp.unit}). Must be 0 < min < max.")
        self._scale_setting_edits.append(e)
        return e

    def _toggle_step_settings(self) -> None:
        # isHidden(), not isVisible(): the latter is False while an ancestor
        # is unshown, so a disclosure keyed on it would refuse to open on a
        # page that hasn't been displayed yet (and reads as "collapsed" to
        # anything inspecting it). isHidden() tracks the explicit flag.
        body = self._settings_body
        show = body.isHidden()
        body.setVisible(show)
        self._settings_toggle.setText(
            ("▾ " if show else "▸ ") + "Step slider settings")

    def _on_snap_toggled(self, snap: bool) -> None:
        self.set_step_snap(snap)
        self.step_settings_changed.emit()

    def _on_range_edited(self, axis: str) -> None:
        """Commit a typed range. An invalid pair (min ≥ max, ≤ 0, junk) is
        REFUSED and the edits snap back to the live range — a silently
        clamped range would leave the slider disagreeing with the boxes."""
        lo_e, hi_e = self._range_edits[axis]
        if self.set_step_range(axis, lo_e.text(), hi_e.text()):
            self.step_settings_changed.emit()
        else:
            self._sync_range_edits(axis)

    @property
    def step_settings_expanded(self) -> bool:
        body = getattr(self, "_settings_body", None)
        return bool(body is not None and not body.isHidden())

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

        # v7.9.x: the step sliders replace the preset buttons — they own the
        # row's stretch, so only a tiny minimum + the row height are set.
        for sld in self._scale_sliders:
            sld.setMinimumWidth(self._px(_BASE_SLIDER_MIN_W))
            sld.setFixedHeight(row_h)

        for edit in self._scale_customs:
            edit.setMinimumWidth(self._px(_BASE_CUSTOM_MIN_W))
            edit.setFixedHeight(row_h)
            _set_pt(edit, self._pt(_FONT_PRESET))

        for edit in self._scale_setting_edits:
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
