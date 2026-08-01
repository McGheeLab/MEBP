"""test_v75x_illumination_led.py — dimmable illumination LED control.

Adds a microscope illumination LED wired to the ZP board's FAN0 output, dimmed
via Marlin ``M106`` (no custom firmware needed). Covered here:

  1. ``ZPStageManager.set_led_brightness`` emits ``M106 P0 S<v>`` and clamps
     the level to 0-255.
  2. ``StageController.set_led_brightness`` forwards only when the ZP board is
     connected, else is a no-op returning False.
  3. ``IlluminationControl`` widget: toggle/slider queue a debounced send with
     the right level, gate on connection, and expose silent state restore.
  4. The ``illumination`` section is registered in the custom-panel catalog and
     builds headless (with optional persisted state).
"""

import os
import threading
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from SupportClasses.ZPStage import ZPStageManager
from SupportClasses.StageController import StageController

_app = QApplication.instance() or QApplication([])


# ── Fakes ──────────────────────────────────────────────────────────────────

class _ScriptedSerial:
    """Real-port stand-in that acks every command with 'ok' and records writes."""

    def __init__(self):
        self.is_open = True
        self.writes: list[str] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data.decode("utf-8", errors="replace").strip())

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        return b"ok\n"

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


def _bare_zp(serial):
    zp = ZPStageManager.__new__(ZPStageManager)  # bypass __init__ (no real port)
    zp.serial = serial
    zp.simulate = False
    zp._serial_lock = threading.RLock()
    zp.feedrate = 1000.0
    zp.x_pos = zp.y_pos = zp.z_pos = zp.e_pos = 0.0
    zp._last_position_read_ok = True
    zp._board_reset_detected = False
    return zp


class _FakeZP:
    """Minimal ZPStageManager stand-in for the controller passthrough test."""

    def __init__(self):
        self.simulate = True          # → StageController.is_zp_connected True
        self.serial = object()
        self.calls: list[int] = []

    def set_led_brightness(self, level):
        self.calls.append(level)
        return True


class _FakeController:
    """Widget-level controller stand-in."""

    def __init__(self, connected=True):
        self.is_zp_connected = connected
        self.led_calls: list[int] = []

    def set_led_brightness(self, level):
        self.led_calls.append(level)
        return True


# ── 1. ZPStageManager.set_led_brightness ─────────────────────────────────────

class TestZPSetLedBrightness(unittest.TestCase):
    def test_emits_m106_on_fan0(self):
        ser = _ScriptedSerial()
        zp = _bare_zp(ser)
        self.assertTrue(zp.set_led_brightness(128))
        self.assertEqual(ser.writes, ["M106 P0 S128"])

    def test_zero_turns_off(self):
        ser = _ScriptedSerial()
        _bare_zp(ser).set_led_brightness(0)
        self.assertEqual(ser.writes, ["M106 P0 S0"])

    def test_clamps_out_of_range(self):
        ser = _ScriptedSerial()
        zp = _bare_zp(ser)
        zp.set_led_brightness(-10)
        zp.set_led_brightness(999)
        self.assertEqual(ser.writes, ["M106 P0 S0", "M106 P0 S255"])

    def test_float_coerced_to_int(self):
        ser = _ScriptedSerial()
        _bare_zp(ser).set_led_brightness(200.7)
        self.assertEqual(ser.writes, ["M106 P0 S200"])

    def test_fan_index_is_a_single_named_constant(self):
        """The fan header is one constant, so moving the LED is a 1-line change."""
        from SupportClasses.ZPStage import _LED_FAN_INDEX
        ser = _ScriptedSerial()
        _bare_zp(ser).set_led_brightness(64)
        self.assertEqual(ser.writes, [f"M106 P{_LED_FAN_INDEX} S64"])


# ── 2. StageController.set_led_brightness ─────────────────────────────────────

class TestControllerPassthrough(unittest.TestCase):
    def test_forwards_when_connected(self):
        ctrl = StageController.__new__(StageController)
        ctrl.zp_stage = _FakeZP()
        self.assertTrue(ctrl.set_led_brightness(90))
        self.assertEqual(ctrl.zp_stage.calls, [90])

    def test_noop_when_disconnected(self):
        ctrl = StageController.__new__(StageController)
        ctrl.zp_stage = None
        self.assertFalse(ctrl.set_led_brightness(90))


# ── 3. IlluminationControl widget ─────────────────────────────────────────────

class TestIlluminationControl(unittest.TestCase):
    def _make(self, connected=True):
        from gui.widgets.illumination_control import IlluminationControl
        ctrl = _FakeController(connected=connected)
        return IlluminationControl(ctrl), ctrl

    def test_toggle_on_queues_and_sends_level(self):
        w, ctrl = self._make()
        w.set_value(100)                 # silent pre-set
        self.assertEqual(ctrl.led_calls, [])
        w._chk_on.setChecked(True)       # user toggles on → queues a send
        self.assertTrue(w._send_timer.isActive())
        w._send_now()                    # fire the debounced send
        self.assertEqual(ctrl.led_calls[-1], 255)

    def test_toggle_off_sends_zero(self):
        w, ctrl = self._make()
        w.set_value(100)
        w._chk_on.setChecked(True)
        w._send_now()
        w._chk_on.setChecked(False)      # off
        w._send_now()
        self.assertEqual(ctrl.led_calls[-1], 0)

    def test_toggle_on_from_dark_slider_defaults_full(self):
        w, ctrl = self._make()
        w.set_value(0)
        w._chk_on.setChecked(True)       # nothing to show → jump to full
        self.assertEqual(w.value(), 100)
        w._send_now()
        self.assertEqual(ctrl.led_calls[-1], 255)

    def test_disconnected_never_sends(self):
        w, ctrl = self._make(connected=False)
        w.set_value(100)
        w._chk_on.setChecked(True)
        w._send_now()
        self.assertEqual(ctrl.led_calls, [])

    def test_slider_move_while_off_does_not_queue(self):
        w, ctrl = self._make()
        w._slider.setValue(40)           # off → just pre-sets the value
        self.assertFalse(w._chk_on.isChecked())
        self.assertFalse(w._send_timer.isActive())   # nothing queued to the bus
        self.assertEqual(ctrl.led_calls, [])

    def test_status_update_reflects_connection(self):
        w, ctrl = self._make(connected=True)
        w.on_status_update()
        self.assertTrue(w._chk_on.isEnabled())
        self.assertTrue(w._slider.isEnabled())
        ctrl.is_zp_connected = False
        w.on_status_update()
        self.assertFalse(w._chk_on.isEnabled())
        self.assertFalse(w._slider.isEnabled())

    def test_set_state_is_silent(self):
        w, ctrl = self._make()
        w.set_value(60)
        w.set_on(True)
        self.assertEqual(ctrl.led_calls, [])   # restore must not command HW
        self.assertTrue(w.is_on())
        self.assertEqual(w.value(), 60)


# ── 4. Custom-panel registration ──────────────────────────────────────────────

class TestSectionRegistration(unittest.TestCase):
    def test_registered_in_catalog(self):
        from gui.widgets.context_sections import catalog, known_types
        self.assertIn("illumination", known_types())
        entry = [c for c in catalog() if c[0] == "illumination"]
        self.assertEqual(len(entry), 1)
        _type, label, icon = entry[0]
        self.assertEqual(label, "Illumination LED")
        self.assertEqual(icon, "💡")

    def test_builds_headless_and_ticks(self):
        from gui.widgets.context_sections import build_section, SectionContext
        ctx = SectionContext(controller=_FakeController())
        w = build_section("illumination", ctx)
        self.assertTrue(hasattr(w, "on_status_update"))
        w.on_status_update()  # must not raise

    def test_restores_persisted_options(self):
        from gui.widgets.context_sections import build_section, SectionContext
        ctx = SectionContext(controller=_FakeController())
        w = build_section("illumination", ctx, {"level": 40, "on": True})
        self.assertTrue(w._ctrl.is_on())
        self.assertEqual(w._ctrl.value(), 40)


if __name__ == "__main__":
    unittest.main()
