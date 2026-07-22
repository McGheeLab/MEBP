"""
illumination_control.py — compact LED illumination control (v7.5.x).

A small reusable widget: an on/off toggle + a brightness slider that drives the
microscope illumination LED wired to the ZP board's FAN0 output (dimmed via
Marlin ``M106`` — see StageController.set_led_brightness / ZPStage.set_led_brightness).

Single source of truth used in two places:
  - embedded as a ``Card("Illumination")`` in :class:`StandardJogContextPanel`
    (so it rides the "Jog" pill everywhere that panel appears), and
  - wrapped by ``IlluminationSection`` in ``gui/widgets/context_sections.py``
    so users can drop a standalone LED module into any custom panel.

Every hardware write is guarded on ``controller.is_zp_connected`` and coalesced
through a short debounce timer so dragging the slider can't flood the shared,
``ok``-blocking ZP serial channel. The widget is a command output, not a live
sensor — it needs the ~status tick only to grey out when the board drops.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox, QHBoxLayout, QLabel, QSlider, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

if TYPE_CHECKING:
    from SupportClasses.StageController import StageController

logger = logging.getLogger(__name__)

_MAX_LEVEL = 255           # Marlin PWM range for M106 S<v>
_SEND_DEBOUNCE_MS = 100    # coalesce slider drags before hitting the serial bus
_DEFAULT_ON_LEVEL = _MAX_LEVEL  # brightness used when toggled on from a dark slider


def _level_from_pct(pct: int) -> int:
    return max(0, min(_MAX_LEVEL, round(pct / 100.0 * _MAX_LEVEL)))


def _pct_from_level(level: int) -> int:
    return max(0, min(100, round(level / _MAX_LEVEL * 100.0)))


class IlluminationControl(QWidget):
    """On/off toggle + 0-100 % brightness slider for the illumination LED."""

    def __init__(
        self,
        controller: "StageController | None" = None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._controller = controller
        # Remembered brightness (0-255) restored when toggled back on from 0.
        self._last_level = _DEFAULT_ON_LEVEL
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
        self._slider.setValue(_pct_from_level(self._last_level))
        self._slider.setToolTip("LED brightness.")
        self._slider.valueChanged.connect(self._on_slider)
        root.addWidget(self._slider)

        # Debounce: restarted on each change, fires one command after the pause.
        self._send_timer = QTimer(self)
        self._send_timer.setSingleShot(True)
        self._send_timer.setInterval(_SEND_DEBOUNCE_MS)
        self._send_timer.timeout.connect(self._send_now)

        self._update_label()
        self._apply_connection_state()

    # ── Public API ─────────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._controller = controller
        self._apply_connection_state()

    def on_status_update(self) -> None:
        """MainWindow / custom-panel tick: reflect ZP connection state."""
        self._apply_connection_state()

    def is_on(self) -> bool:
        return self._chk_on.isChecked()

    def value(self) -> int:
        """Current brightness as a percentage (0-100)."""
        return int(self._slider.value())

    def set_on(self, on: bool) -> None:
        """Set the toggle without commanding hardware (for state restore)."""
        self._suppress = True
        self._chk_on.setChecked(bool(on))
        self._suppress = False
        self._update_label()

    def set_value(self, pct: int) -> None:
        """Set the slider without commanding hardware (for state restore)."""
        self._suppress = True
        self._slider.setValue(max(0, min(100, int(pct))))
        self._suppress = False
        if self._slider.value() > 0:
            self._last_level = _level_from_pct(self._slider.value())
        self._update_label()

    # ── Internals ──────────────────────────────────────────────────

    def _connected(self) -> bool:
        return bool(self._controller
                    and getattr(self._controller, "is_zp_connected", False))

    def _apply_connection_state(self) -> None:
        conn = self._connected()
        self._chk_on.setEnabled(conn)
        self._slider.setEnabled(conn)

    def _desired_level(self) -> int:
        if not self._chk_on.isChecked():
            return 0
        return _level_from_pct(self._slider.value())

    def _on_toggle(self, checked: bool) -> None:
        if self._suppress:
            return
        # Toggling on from a dark slider gives a sensible default so the LED
        # actually illuminates rather than turning "on at 0 %".
        if checked and self._slider.value() == 0:
            self.set_value(_pct_from_level(self._last_level or _DEFAULT_ON_LEVEL))
        self._update_label()
        self._queue_send()

    def _on_slider(self, pct: int) -> None:
        if pct > 0:
            self._last_level = _level_from_pct(pct)
        self._update_label()
        if self._suppress:
            return
        # Only touch the bus when the LED is on; when off the slider just
        # pre-sets a value for the next toggle-on.
        if self._chk_on.isChecked():
            self._queue_send()

    def _update_label(self) -> None:
        if self._chk_on.isChecked():
            self._value_lbl.setText(f"{self._slider.value()}%")
        else:
            self._value_lbl.setText("off")

    def _queue_send(self) -> None:
        self._send_timer.start()

    def _send_now(self) -> None:
        if not self._connected():
            return
        try:
            self._controller.set_led_brightness(self._desired_level())
        except Exception as exc:  # never let a serial hiccup break the UI
            logger.warning("LED brightness set failed: %s", exc)
