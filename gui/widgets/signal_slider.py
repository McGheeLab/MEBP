"""
signal_slider.py — one camera signal control as a slider + a TRUTHFUL readout.

v7.19. The fluorescence mosaic needs exposure, analog gain, frame averaging and
the display black/white points to be adjustable per filter cube while watching a
live histogram, so each one is a compact labelled slider rather than a spin box
buried in a dialog.

TWO THINGS THIS WIDGET EXISTS TO GET RIGHT
------------------------------------------
1. **Log scale, for exposure especially.** The Libra 25's declared exposure
   range is ~6.3 µs … 5760 s — nine decades. On a linear slider every useful
   fluorescence exposure (1–1000 ms) lives in the first 0.02 % of the track and
   is literally unreachable. ``LOG`` maps the track geometrically instead.

2. **The readout must never show a value the camera is not running.** Setters
   here return the ACHIEVED value and the widget re-renders from that, under
   ``blockSignals``. This is the lesson recorded in
   ``camera_settings_dialog._on_exposure_changed`` — *"the persist below must
   record the truth, not the wish"* — and it is what the v7.13 Zyla bench report
   ("exposure resets to a small number") looked like from the operator's side:
   the camera clamped the request and the UI kept showing the request.

The log mapping is the same arithmetic as ``jog_button_array._StepSlider``, and
is deliberately re-implemented rather than shared: that widget is a
hardware-verified composite built around operator-defined jog detents over a
different quantity, and refactoring it to export three lines of ``math.log``
would risk a jog regression for no behavioural gain.
"""

from __future__ import annotations

import math
from typing import Callable, Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QSlider, QVBoxLayout, QWidget

from gui.scaling import s, sf
from gui.styles import COLORS

#: Slider positions across the whole range. Fine enough that one arrow key is a
#: small change at any scale, coarse enough to stay a cheap integer widget.
_STEPS = 1000

#: How long to sit still before pushing a drag to the camera. Matches the
#: existing per-channel exposure debounce in ``_ChannelPromptDialog`` — a live
#: drag would otherwise issue one SDK write per pixel of travel.
_DEBOUNCE_MS = 300

LINEAR = "linear"
LOG = "log"


def value_to_pos(value: float, lo: float, hi: float, mode: str) -> int:
    """Map a value onto ``0.._STEPS``. Clamped, never raises."""
    lo, hi = float(lo), float(hi)
    if hi <= lo:
        return 0
    v = min(max(float(value), lo), hi)
    if mode == LOG and lo > 0.0:
        span = math.log(hi / lo)
        if span <= 0.0:
            return 0
        return int(round(_STEPS * math.log(v / lo) / span))
    return int(round(_STEPS * (v - lo) / (hi - lo)))


def pos_to_value(pos: int, lo: float, hi: float, mode: str) -> float:
    """Inverse of :func:`value_to_pos`."""
    lo, hi = float(lo), float(hi)
    if hi <= lo:
        return lo
    frac = min(max(float(pos) / _STEPS, 0.0), 1.0)
    if mode == LOG and lo > 0.0:
        return lo * math.exp(frac * math.log(hi / lo))
    return lo + frac * (hi - lo)


class SignalSlider(QWidget):
    """``label  [────●────]  readout`` for one numeric camera control.

    ``apply`` is called with the chosen value on a debounced tick and must
    return the value the camera ACTUALLY adopted (or None when it could not be
    read back, in which case the request is shown — there is nothing better to
    show, and the caller has already been told the write failed).
    """

    #: Emitted with the ACHIEVED value after a successful apply, so a host can
    #: persist the truth rather than the request.
    committed = Signal(float)

    def __init__(self, label: str, *, lo: float, hi: float,
                 mode: str = LINEAR, decimals: int = 0, suffix: str = "",
                 scale: float = 1.0, parent: QWidget | None = None):
        """``scale`` converts the CONTROL's units to the DISPLAYED units.

        Exposure is held in µs everywhere in the camera stack but read by
        humans in ms, so the exposure slider passes ``scale=1e-3`` and keeps one
        unit conversion in one place instead of at every call site.
        """
        super().__init__(parent)
        self._lo, self._hi = float(lo), float(hi)
        self._mode = mode if mode in (LINEAR, LOG) else LINEAR
        self._decimals = int(decimals)
        self._suffix = suffix
        self._scale = float(scale) or 1.0
        self._apply: Optional[Callable[[float], Optional[float]]] = None
        self._value = self._lo

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, s(2))
        outer.setSpacing(s(1))

        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        head.setSpacing(s(4))
        self._name = QLabel(label)
        self._name.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        head.addWidget(self._name)
        head.addStretch(1)
        self._readout = QLabel("—")
        self._readout.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {sf(9)}pt; font-weight: 600;")
        head.addWidget(self._readout)
        outer.addLayout(head)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, _STEPS)
        self._slider.setSingleStep(1)
        self._slider.setPageStep(max(1, _STEPS // 20))
        # A slider must be able to scrunch: the left context box goes down to
        # s(100) and the panel is proportional at every width.
        self._slider.setMinimumWidth(s(40))
        self._slider.valueChanged.connect(self._on_moved)
        outer.addWidget(self._slider)

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(_DEBOUNCE_MS)
        self._timer.timeout.connect(self._push)

    # ── configuration ────────────────────────────────────────────────
    def set_apply(self, fn: Optional[Callable[[float], Optional[float]]]):
        self._apply = fn

    def set_range(self, lo: float, hi: float):
        """Re-range without moving the value the operator chose.

        Ranges genuinely change under us — the Andor's achievable exposure
        depends on the readout rate and gain mode — so this re-renders the
        handle against the new bounds rather than resetting to a default.
        """
        self._lo, self._hi = float(lo), float(hi)
        self.set_value(self._value)

    # ── value ────────────────────────────────────────────────────────
    def value(self) -> float:
        return self._value

    def set_value(self, value: float, *, emit: bool = False):
        """Render ``value`` without pushing it to the camera.

        Used both to seed from a stored recipe and — importantly — to correct
        the display to what the camera achieved. Neither is an operator edit, so
        neither may re-enter the apply path.
        """
        try:
            v = float(value)
        except (TypeError, ValueError):
            return
        self._value = min(max(v, self._lo), self._hi)
        blocked = self._slider.blockSignals(True)
        try:
            self._slider.setValue(
                value_to_pos(self._value, self._lo, self._hi, self._mode))
        finally:
            self._slider.blockSignals(blocked)
        self._render()
        if emit:
            self.committed.emit(self._value)

    def _render(self):
        shown = self._value * self._scale
        self._readout.setText(f"{shown:.{self._decimals}f}{self._suffix}")

    # ── live apply ───────────────────────────────────────────────────
    def _on_moved(self, pos: int):
        self._value = pos_to_value(pos, self._lo, self._hi, self._mode)
        self._render()
        self._timer.start()

    def flush(self):
        """Push a pending change now (call before the panel goes away)."""
        if self._timer.isActive():
            self._timer.stop()
            self._push()

    def _push(self):
        if self._apply is None:
            self.committed.emit(self._value)
            return
        try:
            achieved = self._apply(self._value)
        except Exception:
            achieved = None
        if achieved is not None:
            # The camera is the authority. Re-render from what it adopted, not
            # from what was asked for.
            self.set_value(achieved)
        self.committed.emit(self._value)

    # ── narrow-width behaviour ───────────────────────────────────────
    def set_compact(self, compact: bool):
        """Hide the name label when the panel is too narrow to carry both."""
        self._name.setVisible(not compact)
