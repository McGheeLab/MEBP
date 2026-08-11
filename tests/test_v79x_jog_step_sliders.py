"""v7.9.x tests — jog step SLIDERS with setpoint detents + collapsible
step settings + µL-native pump steps (operator, 2026-08-07):

  *"we also need lower total volume for the pumps, lets make a slider bar for
  each axis that we want to move by with setpoints close to 0.001, 0.005,
  0.01, 0.05, 0.1, 0.5, 1, 5, 10 uL. there should be a check box for snap to
  nearest, and custom range for the slider. These settings should be
  available via a collapsible settings section which is collapsed by
  default."*

What is pinned here:

  1.  Each axis row is ONE slider (the five preset magnitude buttons are
      gone), and the pump ladder is EXACTLY the requested µL setpoints.
  2.  Snap ON: every slider position IS a setpoint — walking the slider
      end to end yields the ladder verbatim (0.001 … 10 µL), so the sub-µL
      steps the operator asked for are reachable.
  3.  Snap OFF: a continuous log sweep across the range.
  4.  Custom range narrows the ladder; an invalid range (min ≥ max, ≤ 0,
      junk) is REFUSED, not clamped, and the edit boxes snap back.
  5.  The settings section is COLLAPSED by default and toggles.
  6.  **The pump step is a µL VOLUME, not a % of syringe** — the whole point
      of the request (0.001 µL is 0.0004 % of a 250 µL syringe, which the
      old % ladder could not express). Pinned end-to-end through
      `HardwareControlPanel._on_jog_pump` → `move_pump_uL`, and at the
      `settings_page` context jog, whose handler used to pass the % value
      straight to `move_pump_relative` (which expects plunger MM).
  7.  Step settings persist: an edit reaches the controller store + settings,
      a SECOND panel on the same controller adopts them, and a restart
      (fresh controller + persisted settings) re-seeds and repopulates the
      controller — the nine jog tiles cannot diverge.
  8.  Hardware Setup's mm jog is unchanged (µm ladder ÷ 1000 → mm).
"""

import sys
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.StageController import StageController


def _wait_for(predicate, timeout_s=3.0):
    from PySide6.QtWidgets import QApplication
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    QApplication.processEvents()
    return predicate()


class _Settings:
    def __init__(self):
        self._d = {}

    def get(self, key, default=None):
        return self._d.get(key, default)

    def set(self, key, value):
        self._d[key] = value

    def save(self):
        pass


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)


# ── The slider itself ───────────────────────────────────────────────

class TestStepSliders(_QtBase):

    # The operator's ladder, verbatim.
    _WANT_UL = (0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0)

    def _arr(self, **kw):
        from gui.widgets.jog_button_array import JogButtonArray
        kw.setdefault("compact", True)
        kw.setdefault("show_pumps", True)
        kw.setdefault("pump_action_labels", True)
        return JogButtonArray(**kw)

    # 1 — one slider per axis; the preset buttons are gone
    def test_each_axis_has_one_slider_and_no_preset_buttons(self):
        from PySide6.QtWidgets import QSlider
        arr = self._arr()
        self.assertEqual(set(arr._step_groups), {"XY", "Z", "P"})
        for axis in ("XY", "Z", "P"):
            self.assertIsInstance(arr._step_groups[axis].slider, QSlider)
        self.assertFalse(hasattr(arr, "_scale_presets"))

    # 1b — the pump ladder IS the requested µL setpoints
    def test_pump_setpoints_are_the_requested_uL_ladder(self):
        arr = self._arr()
        self.assertEqual(arr._step_groups["P"].setpoints, self._WANT_UL)
        self.assertEqual(arr._step_groups["P"].unit, "µL")

    # 2 — snap ON: every slider position is a setpoint, ladder reachable
    def test_snap_walks_exactly_the_setpoints(self):
        arr = self._arr()
        self.assertTrue(arr.step_snap)
        g = arr._step_groups["P"]
        seen = []
        for pos in range(g.slider.minimum(), g.slider.maximum() + 1):
            g.slider.setValue(pos)
            seen.append(arr.pump_step)
        self.assertEqual(tuple(seen), self._WANT_UL)

    # 3 — snap OFF: continuous log sweep, values between the setpoints
    def test_snap_off_is_a_continuous_log_sweep(self):
        arr = self._arr()
        arr.set_step_snap(False)
        self.assertFalse(arr.step_snap)
        g = arr._step_groups["P"]
        g.slider.setValue(g.slider.minimum())
        self.assertAlmostEqual(arr.pump_step, 0.001, places=6)
        g.slider.setValue(g.slider.maximum())
        self.assertAlmostEqual(arr.pump_step, 10.0, places=6)
        # A midpoint is the geometric mean of the ends (log scale), and is
        # NOT one of the ladder setpoints — proving snap really is off.
        g.slider.setValue((g.slider.minimum() + g.slider.maximum()) // 2)
        self.assertAlmostEqual(arr.pump_step, 0.1, places=3)
        g.slider.setValue(int(g.slider.maximum() * 0.30))
        self.assertNotIn(arr.pump_step, self._WANT_UL)

    # 4 — custom range narrows the ladder
    def test_custom_range_narrows_the_ladder(self):
        arr = self._arr()
        self.assertTrue(arr.set_step_range("P", 0.001, 0.1))
        g = arr._step_groups["P"]
        self.assertEqual(g.active_points(), [0.001, 0.005, 0.01, 0.05, 0.1])
        seen = []
        for pos in range(g.slider.minimum(), g.slider.maximum() + 1):
            g.slider.setValue(pos)
            seen.append(arr.pump_step)
        self.assertEqual(seen, [0.001, 0.005, 0.01, 0.05, 0.1])

    # 4b — an invalid range is REFUSED (not clamped) and the boxes snap back
    def test_invalid_range_is_refused_and_edits_resync(self):
        arr = self._arr()
        self.assertTrue(arr.set_step_range("P", 0.002, 2.0))
        for lo, hi in ((5.0, 5.0), (9.0, 1.0), (0.0, 1.0), (-1.0, 1.0),
                       ("junk", 1.0), (0.1, None)):
            self.assertFalse(arr.set_step_range("P", lo, hi),
                             f"{lo}..{hi} should be refused")
        self.assertEqual(arr._step_groups["P"].range(), (0.002, 2.0))
        # A refused typed range restores the boxes to the live range.
        lo_e, hi_e = arr._range_edits["P"]
        lo_e.setText("9"); hi_e.setText("1")
        arr._on_range_edited("P")
        self.assertEqual(lo_e.text(), "0.002")
        self.assertEqual(hi_e.text(), "2")

    # 4c — a typed step outside the range is honoured as-is
    def test_typed_value_outside_range_is_used_verbatim(self):
        arr = self._arr()
        arr.set_step_range("P", 0.01, 1.0)
        g = arr._step_groups["P"]
        g.custom_edit.setText("0.0005")
        g._on_custom_text("0.0005")
        self.assertAlmostEqual(arr.pump_step, 0.0005)
        self.assertEqual(g.slider.value(), g.slider.minimum())  # clamps only

    # 5 — the settings section is collapsed by default
    def test_settings_section_collapsed_by_default_and_toggles(self):
        arr = self._arr()
        self.assertFalse(arr.step_settings_expanded)
        # isHidden(), not isVisible(): offscreen and unshown, isVisible() is
        # False either way, so it could not tell collapsed from expanded.
        self.assertTrue(arr._settings_body.isHidden())
        self.assertTrue(arr._settings_toggle.text().startswith("▸"))
        arr._settings_toggle.click()
        self.assertTrue(arr.step_settings_expanded)
        self.assertTrue(arr._settings_toggle.text().startswith("▾"))
        arr._settings_toggle.click()
        self.assertFalse(arr.step_settings_expanded)

    # 5b — the snap checkbox drives the sliders and starts checked
    def test_snap_checkbox_drives_the_sliders(self):
        arr = self._arr()
        self.assertTrue(arr._snap_check.isChecked())
        arr._snap_check.setChecked(False)
        self.assertFalse(arr.step_snap)
        arr._snap_check.setChecked(True)
        self.assertTrue(arr.step_snap)
        # …and the programmatic API keeps the checkbox honest.
        arr.set_step_snap(False)
        self.assertFalse(arr._snap_check.isChecked())

    # 8 — Hardware Setup's mm jog is unchanged (µm ladder ÷ 1000)
    def test_hardware_setup_pump_step_is_still_mm(self):
        arr = self._arr(pump_action_labels=False)
        self.assertFalse(arr.pump_step_is_uL)
        g = arr._step_groups["P"]
        g.slider.setValue(g.slider.maximum())      # 10000 µm
        self.assertAlmostEqual(arr.pump_step, 10.0)   # → mm
        self.assertAlmostEqual(arr.xy_step_um, 100.0)
        self.assertAlmostEqual(arr.z_step_mm, 0.1)

    # signal contract: a pump click emits the slider's µL, signed
    def test_pump_click_emits_signed_uL(self):
        arr = self._arr()
        g = arr._step_groups["P"]
        g.slider.setValue(0)                       # 0.001 µL
        got = []
        arr.jog_pump_requested.connect(lambda p, d: got.append((p, d)))
        asp, disp = arr._pump_buttons["P1"]
        asp.click()
        disp.click()
        self.assertAlmostEqual(got[0][1], -0.001)  # aspirate
        self.assertAlmostEqual(got[1][1], 0.001)   # dispense


# ── Persistence + cross-tile sharing ────────────────────────────────

class TestStepSettingsShared(_QtBase):

    def _ctrl(self):
        c = mock.MagicMock()
        c.is_xy_connected = True
        c.is_zp_connected = True
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        c._hardware_config = None
        c.backlash_comp_enabled = None
        c.get_jog_speed_state.return_value = {}
        c.get_max_pump_feedrate.return_value = 10.0
        c.get_max_pump_feedrate_for.side_effect = lambda pid: 10.0
        c.get_pump_jog_pcts.return_value = {}
        store: dict = {}
        c._step_store = store
        c.set_jog_step_settings.side_effect = \
            lambda cfg: store.update({"cfg": dict(cfg)})
        c.get_jog_step_settings.side_effect = \
            lambda: dict(store.get("cfg", {}))
        return c

    def _panel(self, ctrl=None, settings=None):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        c = ctrl if ctrl is not None else self._ctrl()
        p = HardwareControlPanel(show_connect=False, bypass_safety=False,
                                 pump_action_labels=True)
        if settings is not None:
            p._settings = settings
        p.set_controller(c)
        return p, c

    # 7 — an edit reaches the controller + settings; a second panel adopts it
    def test_edit_shared_across_panels(self):
        st = _Settings()
        p1, c = self._panel(settings=st)
        p1._jog_array.set_step_range("P", 0.001, 0.05)
        p1._jog_array._step_groups["P"].set_value(0.005)
        p1._jog_array.step_settings_changed.emit()
        self.assertEqual(c._step_store["cfg"]["axes"]["P"]["hi"], 0.05)
        self.assertEqual(
            st.get("jog.step_settings")["axes"]["P"]["value"], 0.005)
        p2, _ = self._panel(ctrl=c)
        self.assertEqual(p2._jog_array._step_groups["P"].range(),
                         (0.001, 0.05))
        self.assertAlmostEqual(p2._jog_array.pump_step, 0.005)

    # 7b — restart: settings seed the array AND repopulate the controller
    def test_restart_seeds_and_repopulates_controller(self):
        st = _Settings()
        st.set("jog.step_settings", {
            "snap": False,
            "axes": {"P": {"lo": 0.002, "hi": 0.2, "value": 0.02}}})
        p, c = self._panel(settings=st)
        self.assertFalse(p._jog_array.step_snap)
        self.assertEqual(p._jog_array._step_groups["P"].range(), (0.002, 0.2))
        self.assertAlmostEqual(p._jog_array.pump_step, 0.02)
        self.assertEqual(c._step_store["cfg"]["axes"]["P"]["hi"], 0.2)

    # 6 — end to end: the panel dispenses the slider's µL, unscaled
    def test_panel_jog_dispenses_the_slider_uL(self):
        p, c = self._panel()
        g = p._jog_array._step_groups["P"]
        g.slider.setValue(1)                       # 0.005 µL
        asp, _disp = p._jog_array._pump_buttons["P2"]
        asp.click()
        self.assertTrue(_wait_for(lambda: c.move_pump_uL.called))
        self.assertEqual(c.move_pump_uL.call_args.args[0], "P2")
        self.assertAlmostEqual(c.move_pump_uL.call_args.args[1], -0.005)
        self.assertFalse(c.pump_pct_to_uL.called)  # NOT a % of syringe


# ── The settings-page context jog's unit bug ────────────────────────

class TestSettingsPageContextJogUnits(unittest.TestCase):
    """Its array is built with pump_action_labels=True, so the emitted
    distance is a µL VOLUME — the handler must use move_pump_uL. It used to
    call move_pump_relative, which expects plunger MM."""

    def test_handler_routes_to_move_pump_uL(self):
        import ast
        import inspect
        from gui.pages import settings_page
        src = inspect.getsource(settings_page.SettingsPage._ctx_jog_pump)
        tree = ast.parse(src.strip())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        names = {n.func.attr for n in calls
                 if isinstance(n.func, ast.Attribute)}
        self.assertIn("move_pump_uL", names)
        self.assertNotIn("move_pump_relative", names)


# ── Controller store ────────────────────────────────────────────────

class TestControllerStepStore(unittest.TestCase):

    def test_roundtrip_returns_a_copy_and_ignores_junk(self):
        f = type("F", (), {})()
        f._jog_step_settings = {}
        StageController.set_jog_step_settings(f, {"snap": False})
        out = StageController.get_jog_step_settings(f)
        self.assertEqual(out, {"snap": False})
        out["snap"] = True                         # a copy, not the store
        self.assertFalse(StageController.get_jog_step_settings(f)["snap"])
        StageController.set_jog_step_settings(f, "not-a-dict")
        self.assertEqual(StageController.get_jog_step_settings(f),
                         {"snap": False})


if __name__ == "__main__":
    unittest.main()
