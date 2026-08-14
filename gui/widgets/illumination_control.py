"""
illumination_control.py — compact LED illumination control (v7.5.x).

A small reusable widget: an on/off toggle + a brightness slider that drives the
microscope illumination LED wired to the ZP board's FAN0 output (dimmed via
Marlin ``M106`` — see StageController.set_led_brightness / ZPStage.set_led_brightness).

**The slider's 100 % is not the LED's 100 %.** The emitter overheats when held
above ~30 % duty, so the 0-100 % the operator sees is scaled onto ``_MAX_DUTY_PCT``
(20 %) of the electrical maximum — full slider commands ``M106 S51``, not ``S255``.

Single source of truth used in two places:
  - embedded as a ``Card("Illumination")`` in :class:`StandardJogContextPanel`
    (so it rides the "Jog" pill everywhere that panel appears), and
  - wrapped by ``IlluminationSection`` in ``gui/widgets/context_sections.py``
    so users can drop a standalone LED module into any custom panel.

**One LED, one state.** Every page that hosts a jog context builds its OWN
``StandardJogContextPanel``, so there are many of these widgets alive at once —
and the LED has no readback (Marlin ``M106`` is write-only), so nothing would
ever reconcile them. The toggle + brightness therefore live in the process-wide
:class:`_IlluminationState` (``illumination_state()``) and each widget is only a
VIEW of it: user edits go to the state, the state re-renders every view. Without
this, each page showed its own stale copy of the light's setting, and mirrors the
shared-``MicroscopeController`` arrangement of the card directly below it.

The state also owns the single debounce timer and the hardware write, so N
mounted views produce ONE ``M106`` per change rather than N — dragging a slider
must not flood the shared, ``ok``-blocking ZP serial channel. Every write is
guarded on ``controller.is_zp_connected``. The widget is a command output, not a
live sensor — it needs the ~status tick only to grey out when the board drops.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QHBoxLayout, QLabel, QSlider, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

if TYPE_CHECKING:
    from SupportClasses.StageController import StageController

logger = logging.getLogger(__name__)

_MAX_LEVEL = 255           # Marlin PWM full scale for M106 S<v>

# ── Thermal ceiling ──────────────────────────────────────────────────
# The LED overheats when driven above ~30 % duty for long periods, so the
# slider's 100 % is REMAPPED onto _MAX_DUTY_PCT of the electrical maximum
# rather than clipped at it: the operator keeps a full-range 0-100 % control
# and simply cannot command a level that cooks the emitter. Clipping instead
# would make every setting above the ceiling look different but behave the
# same, which reads as a broken slider.
#
# This is the ONE place the ceiling is expressed — the hardware layer
# (ZPStage.set_led_brightness) still accepts the full 0-255 Marlin range, so
# a second copy of this rule there would be a redundant guard, not a backstop.
_MAX_DUTY_PCT = 20.0
_CEILING_LEVEL = max(1, round(_MAX_LEVEL * _MAX_DUTY_PCT / 100.0))  # 51 of 255

_SEND_DEBOUNCE_MS = 100    # coalesce slider drags before hitting the serial bus
_DEFAULT_ON_LEVEL = _CEILING_LEVEL  # brightness used when toggled on from a dark slider


def _level_from_pct(pct: int) -> int:
    """Slider percentage → Marlin PWM level, scaled by the thermal ceiling."""
    return max(0, min(_CEILING_LEVEL, round(pct / 100.0 * _CEILING_LEVEL)))


def _pct_from_level(level: int) -> int:
    """Marlin PWM level → slider percentage (inverse of :func:`_level_from_pct`)."""
    return max(0, min(100, round(level / _CEILING_LEVEL * 100.0)))


class _IlluminationState(QObject):
    """Process-wide illumination-LED state shared by every view.

    Holds the on/off flag, the brightness percentage, the remembered brightness
    used when toggling back on from a dark slider, the stage controller used to
    reach the board, and the ONE debounce timer that coalesces changes into a
    single ``M106``. Views connect to :attr:`changed` and re-render; they never
    hold state of their own.
    """

    changed = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._on = False
        self._pct = _pct_from_level(_DEFAULT_ON_LEVEL)
        # Remembered brightness (0-255) restored when toggled back on from 0.
        self._last_level = _DEFAULT_ON_LEVEL
        self._controller = None
        # True until anything (a user edit or a silent restore) touches the
        # state — lets a persisted custom-panel layout seed the LED at startup
        # without clobbering a setting the operator has already made.
        self._pristine = True

        # Debounce: restarted on each change, fires one command after the pause.
        self._send_timer = QTimer(self)
        self._send_timer.setSingleShot(True)
        self._send_timer.setInterval(_SEND_DEBOUNCE_MS)
        self._send_timer.timeout.connect(self.send_now)

    # ── Read ───────────────────────────────────────────────────────

    def is_on(self) -> bool:
        return self._on

    def pct(self) -> int:
        return self._pct

    def is_pristine(self) -> bool:
        return self._pristine

    def connected(self) -> bool:
        return bool(self._controller
                    and getattr(self._controller, "is_zp_connected", False))

    def level(self) -> int:
        """Brightness actually commanded (0 while the LED is off)."""
        return _level_from_pct(self._pct) if self._on else 0

    # ── Write ──────────────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        if controller is None or controller is self._controller:
            return
        self._controller = controller
        self.changed.emit()  # views re-evaluate their enabled state

    def apply(self, *, on: bool | None = None, pct: int | None = None,
              send: bool) -> None:
        """Update the shared state, re-render every view, optionally command.

        ``send=False`` is a silent restore (state + UI only, no hardware).
        """
        dirty = False
        if pct is not None:
            pct = max(0, min(100, int(pct)))
            if pct > 0:
                self._last_level = _level_from_pct(pct)
            if pct != self._pct:
                self._pct = pct
                dirty = True
        if on is not None:
            on = bool(on)
            # Toggling on from a dark slider gives a sensible default so the LED
            # actually illuminates rather than turning "on at 0 %".
            if on and self._pct == 0:
                self._pct = _pct_from_level(self._last_level or _DEFAULT_ON_LEVEL)
                dirty = True
            if on != self._on:
                self._on = on
                dirty = True
        self._pristine = False
        if dirty:
            self.changed.emit()
        if send:
            self._send_timer.start()

    def send_now(self) -> None:
        if not self.connected():
            return
        try:
            self._controller.set_led_brightness(self.level())
        except Exception as exc:  # never let a serial hiccup break the UI
            logger.warning("LED brightness set failed: %s", exc)

    def reset(self) -> None:
        """Back to a fresh process state (tests; never used by the app)."""
        self._send_timer.stop()
        self._on = False
        self._pct = _pct_from_level(_DEFAULT_ON_LEVEL)
        self._last_level = _DEFAULT_ON_LEVEL
        self._controller = None
        self._pristine = True
        self.changed.emit()


_STATE: "_IlluminationState | None" = None


def illumination_state() -> _IlluminationState:
    """The one shared LED state (lazily created; needs a QApplication)."""
    global _STATE
    if _STATE is None:
        _STATE = _IlluminationState()
    return _STATE


class IlluminationControl(QWidget):
    """On/off toggle + 0-100 % brightness slider for the illumination LED."""

    def __init__(
        self,
        controller: "StageController | None" = None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._state = illumination_state()
        if controller is not None:
            self._state.set_controller(controller)
        self._suppress = False  # block feedback during programmatic widget sets

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(s(6))

        # Row 1: on/off toggle + live percentage readout.
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(s(6))
        self._chk_on = QCheckBox("LED")
        self._chk_on.setToolTip("Turn the illumination LED on/off.")
        self._chk_on.toggled.connect(self._on_toggle)
        top.addWidget(self._chk_on)
        top.addStretch(1)
        self._value_lbl = QLabel("off")
        self._value_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        top.addWidget(self._value_lbl)
        root.addLayout(top)

        # Row 2: brightness slider (percentage).
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, 100)
        self._slider.setToolTip(
            "LED brightness. 100 % commands "
            f"{_MAX_DUTY_PCT:g} % of the LED's electrical maximum — the range is "
            "scaled to a safe ceiling, because the emitter overheats when held "
            "brighter for long periods.")
        self._slider.valueChanged.connect(self._on_slider)
        root.addWidget(self._slider)

        # Every view re-renders whenever the shared state changes, so the LED
        # reads the same on the Jog page, the Calibration page, each workflow
        # page and a Custom panel. Qt drops this connection when the widget is
        # destroyed, so a closed page can't be rendered into.
        self._state.changed.connect(self._render)
        self._render()

    # ── Public API ─────────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._state.set_controller(controller)
        self._apply_connection_state()

    def on_status_update(self) -> None:
        """MainWindow / custom-panel tick: reflect ZP connection state."""
        self._apply_connection_state()

    def is_on(self) -> bool:
        return self._state.is_on()

    def value(self) -> int:
        """Current brightness as a percentage (0-100)."""
        return self._state.pct()

    def set_on(self, on: bool) -> None:
        """Set the toggle without commanding hardware (for state restore)."""
        self._state.apply(on=on, send=False)

    def set_value(self, pct: int) -> None:
        """Set the slider without commanding hardware (for state restore)."""
        self._state.apply(pct=pct, send=False)

    # ── Internals ──────────────────────────────────────────────────

    @property
    def _send_timer(self) -> QTimer:
        """The shared debounce timer (one pending write for all views)."""
        return self._state._send_timer

    def _connected(self) -> bool:
        return self._state.connected()

    def _apply_connection_state(self) -> None:
        conn = self._connected()
        self._chk_on.setEnabled(conn)
        self._slider.setEnabled(conn)

    def _desired_level(self) -> int:
        return self._state.level()

    def _render(self) -> None:
        """Paint this view from the shared state (no hardware, no feedback)."""
        self._suppress = True
        try:
            self._chk_on.setChecked(self._state.is_on())
            self._slider.setValue(self._state.pct())
        finally:
            self._suppress = False
        self._update_label()
        self._apply_connection_state()

    def _on_toggle(self, checked: bool) -> None:
        if self._suppress:
            return
        self._state.apply(on=checked, send=True)

    def _on_slider(self, pct: int) -> None:
        if self._suppress:
            return
        # Only touch the bus when the LED is on; when off the slider just
        # pre-sets a value for the next toggle-on.
        self._state.apply(pct=pct, send=self._state.is_on())

    def _update_label(self) -> None:
        if self._state.is_on():
            self._value_lbl.setText(f"{self._state.pct()}%")
        else:
            self._value_lbl.setText("off")

    def _send_now(self) -> None:
        self._state.send_now()
