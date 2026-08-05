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
  5. Every view reads ONE shared state, so the LED reads the same on every page
     that hosts a jog context — and N mounted views still emit ONE ``M106``.
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
    def setUp(self):
        # The LED state is process-wide by design (one light, many views), so
        # each test starts from a fresh one.
        from gui.widgets.illumination_control import illumination_state
        illumination_state().reset()

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
    def setUp(self):
        from gui.widgets.illumination_control import illumination_state
        illumination_state().reset()

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

    def test_persisted_options_do_not_clobber_a_live_setting(self):
        """The custom panel rebuilds its sections on every layout change; a
        re-seed then would reset the LED the operator just set."""
        from gui.widgets.context_sections import build_section, SectionContext
        from gui.widgets.illumination_control import IlluminationControl
        ctrl = _FakeController()
        live = IlluminationControl(ctrl)
        live.set_value(70)
        live.set_on(True)                 # operator's current setting

        ctx = SectionContext(controller=ctrl)
        rebuilt = build_section("illumination", ctx, {"level": 10, "on": False})
        self.assertTrue(rebuilt._ctrl.is_on())
        self.assertEqual(rebuilt._ctrl.value(), 70)


# ── 5. One shared state across every page ────────────────────────────────────

class TestSharedAcrossPanels(unittest.TestCase):
    """Each page builds its own StandardJogContextPanel, hence its own
    IlluminationControl. All of them must show the one light's real setting."""

    def setUp(self):
        from gui.widgets.illumination_control import illumination_state
        illumination_state().reset()

    def _two_views(self, connected=True):
        from gui.widgets.illumination_control import IlluminationControl
        ctrl = _FakeController(connected=connected)
        return IlluminationControl(ctrl), IlluminationControl(ctrl), ctrl

    def test_toggle_on_one_view_shows_on_the_other(self):
        a, b, _ctrl = self._two_views()
        a.set_value(60)
        a._chk_on.setChecked(True)
        self.assertTrue(b.is_on())
        self.assertTrue(b._chk_on.isChecked())
        self.assertEqual(b.value(), 60)
        self.assertEqual(b._slider.value(), 60)
        self.assertEqual(b._value_lbl.text(), "60%")

    def test_brightness_drag_on_one_view_shows_on_the_other(self):
        a, b, _ctrl = self._two_views()
        a._chk_on.setChecked(True)
        a._slider.setValue(25)
        self.assertEqual(b._slider.value(), 25)
        self.assertEqual(b._value_lbl.text(), "25%")

    def test_a_view_built_later_opens_on_the_current_state(self):
        """Navigating to a page for the first time mid-session must not show a
        stale default — this is the reported bug."""
        from gui.widgets.illumination_control import IlluminationControl
        a, _b, ctrl = self._two_views()
        a.set_value(35)
        a._chk_on.setChecked(True)
        late = IlluminationControl(ctrl)          # e.g. a workflow page opened now
        self.assertTrue(late.is_on())
        self.assertEqual(late._slider.value(), 35)

    def test_many_views_emit_one_command(self):
        """N mounted views must not each write to the ok-blocking ZP channel."""
        from gui.widgets.illumination_control import IlluminationControl
        ctrl = _FakeController()
        views = [IlluminationControl(ctrl) for _ in range(4)]
        views[0].set_value(50)
        views[0]._chk_on.setChecked(True)
        views[0]._send_now()
        self.assertEqual(ctrl.led_calls, [_level_from_pct_expected(50)])

    def test_views_share_one_debounce_timer(self):
        a, b, _ctrl = self._two_views()
        a._chk_on.setChecked(True)
        self.assertTrue(b._send_timer.isActive())
        self.assertIs(a._send_timer, b._send_timer)

    def test_off_on_one_view_greys_out_all(self):
        a, b, ctrl = self._two_views(connected=True)
        ctrl.is_zp_connected = False
        b.on_status_update()                 # only ONE view gets the tick…
        self.assertFalse(b._slider.isEnabled())
        a.on_status_update()                 # …the other reflects it on its own tick
        self.assertFalse(a._slider.isEnabled())


def _level_from_pct_expected(pct: int) -> int:
    from gui.widgets.illumination_control import _level_from_pct
    return _level_from_pct(pct)


if __name__ == "__main__":
    unittest.main()
