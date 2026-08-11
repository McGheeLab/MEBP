"""v7.9.x tests — per-pump jog flow rate on every jog tile + position-readout
text priority (operator, 2026-08-07):

  * "on all jog tiles, each pump should get its own flow rate. Currently this
    only exists on the hardware setup page which is a special version of the
    jog controls."
  * "the current position of each axis has the scroll bar being too long so
    we cant see the text of the position since it overlays other text."

What is pinned here:

  1.  HardwareControlPanel percent mode builds ONE flow row PER PUMP
      (P1/P2/P3), each with its own sub-1%-capable spin + resolved label.
  2.  A pump jog uses the CLICKED pump's own % × that pump's OWN flow
      ceiling (StageController.get_max_pump_feedrate_for) — jogging P2 with
      P1's settings is the mutation this catches.
  3.  Edits push StageController.set_pump_jog_pct (the shared cross-panel
      state) and persist to settings ``jog.pump_jog_pct``; a SECOND panel on
      the same controller seeds the same values, so the nine jog tiles
      cannot diverge (the illumination-LED lesson).
  4.  A restart (fresh controller + persisted settings) re-seeds the spins
      AND pushes the values back onto the controller.
  5.  Rows for unconfigured pumps hide.
  6.  StageController.get_max_pump_feedrate_for: per-pump flow limit → 10
      µL/s default when no limit is computed → legacy delegate to
      _pump_jog_max_native when no hardware config exists.
  7.  PositionValueLabel's minimum width TRACKS its text (and font changes,
      capped), and the PositionBar beside it is the yielding element
      (Ignored h-policy, tiny min) — in BOTH copies (HardwareControlPanel's
      Live Position grid and the Custom-panel PositionReadoutCard).
"""

import sys
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.StageController import StageController


def _wait_for(predicate, timeout_s=3.0):
    """Pump jogs run on a daemon thread; spin the Qt event loop until
    ``predicate()`` is true (delivers the queued completion signal)."""
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


# ── Panel-level: per-pump flow rows on the jog tiles ────────────────

class TestPerPumpJogFlowRows(_QtBase):

    _PER_PUMP_MAX = {"P1": 10.0, "P2": 2.0, "P3": 4.0}

    def _ctrl(self):
        c = mock.MagicMock()
        c.is_xy_connected = True
        c.is_zp_connected = True
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        c._hardware_config = None
        c.backlash_comp_enabled = None          # not callable → comp off
        c.get_jog_speed_state.return_value = {}
        c.get_max_pump_feedrate.return_value = max(self._PER_PUMP_MAX.values())
        c.get_max_pump_feedrate_for.side_effect = \
            lambda pid: self._PER_PUMP_MAX[pid]
        c.pump_pct_to_uL.side_effect = lambda p, pct: (pct / 100.0) * 100.0
        # dict-backed per-pump % store (the real controller behaviour).
        store: dict = {}
        c._pump_pct_store = store
        c.set_pump_jog_pct.side_effect = \
            lambda p, v: store.__setitem__(p, float(v))
        c.get_pump_jog_pcts.side_effect = lambda: dict(store)
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

    # 1 — one flow row per pump, each independently editable
    def test_percent_mode_builds_one_flow_row_per_pump(self):
        p, _ = self._panel()
        self.assertEqual(set(p.spin_p_pct_pumps), {"P1", "P2", "P3"})
        self.assertEqual(set(p.lbl_p_resolved_pumps), {"P1", "P2", "P3"})
        spins = list(p.spin_p_pct_pumps.values())
        self.assertEqual(len({id(w) for w in spins}), 3)
        p.spin_p_pct_pumps["P1"].setValue(12.0)
        p.spin_p_pct_pumps["P2"].setValue(34.0)
        self.assertAlmostEqual(p.spin_p_pct_pumps["P1"].value(), 12.0)
        self.assertAlmostEqual(p.spin_p_pct_pumps["P2"].value(), 34.0)

    # 1b — resolved labels use each pump's OWN ceiling
    def test_resolved_labels_are_per_pump(self):
        p, _ = self._panel()
        p.spin_p_pct_pumps["P1"].setValue(50.0)
        p.spin_p_pct_pumps["P2"].setValue(50.0)
        p._resolve_speeds()
        self.assertIn("5.00 µL/s", p.lbl_p_resolved_pumps["P1"].text())
        self.assertIn("1.00 µL/s", p.lbl_p_resolved_pumps["P2"].text())

    # 2 — a jog uses the clicked pump's own % × its own ceiling
    def test_jog_uses_the_clicked_pumps_own_rate(self):
        p, c = self._panel()
        p.spin_p_pct_pumps["P1"].setValue(50.0)   # 50% × 10 = 5.0 µL/s
        p.spin_p_pct_pumps["P2"].setValue(25.0)   # 25% ×  2 = 0.5 µL/s
        p._on_jog_pump("P2", -10.0)
        self.assertTrue(_wait_for(lambda: c.move_pump_uL.called))
        kwargs = c.move_pump_uL.call_args.kwargs
        self.assertEqual(c.move_pump_uL.call_args.args[0], "P2")
        self.assertAlmostEqual(kwargs["rate_uL_s"], 0.5)
        # And the other pump keeps ITS rate — catches a "always read P1's
        # spin" or "always use the shared fastest-pump anchor" mutation.
        self.assertTrue(_wait_for(lambda: not p._pump_jog_busy))
        c.move_pump_uL.reset_mock()
        p._on_jog_pump("P1", -10.0)
        self.assertTrue(_wait_for(lambda: c.move_pump_uL.called))
        self.assertAlmostEqual(
            c.move_pump_uL.call_args.kwargs["rate_uL_s"], 5.0)

    # 3 — edits reach the controller store + settings, and a second panel
    #     on the same controller seeds the same values (no divergence).
    def test_edit_pushes_controller_and_second_panel_agrees(self):
        settings = _Settings()
        p1, c = self._panel(settings=settings)
        p1.spin_p_pct_pumps["P2"].setValue(7.5)
        self.assertAlmostEqual(c._pump_pct_store["P2"], 7.5)
        self.assertAlmostEqual(
            settings.get("jog.pump_jog_pct")["P2"], 7.5)
        p2, _ = self._panel(ctrl=c)
        self.assertAlmostEqual(p2.spin_p_pct_pumps["P2"].value(), 7.5)

    # 4 — restart: persisted settings seed the spins AND repopulate the
    #     (fresh) controller store so other panels agree without settings.
    def test_restart_seeds_from_settings_and_repopulates_controller(self):
        settings = _Settings()
        settings.set("jog.pump_jog_pct", {"P3": 3.25})
        p, c = self._panel(settings=settings)
        self.assertAlmostEqual(p.spin_p_pct_pumps["P3"].value(), 3.25)
        self.assertAlmostEqual(c._pump_pct_store["P3"], 3.25)

    # 4b — pre-per-pump installs: the shared 'p' group % still seeds
    def test_shared_p_pct_is_the_fallback_seed(self):
        c = self._ctrl()
        c.get_jog_speed_state.return_value = {
            "p": {"pct": 15.0, "speed": None, "unit": "µL/s"}}
        p, _ = self._panel(ctrl=c)
        for pid in ("P1", "P2", "P3"):
            self.assertAlmostEqual(p.spin_p_pct_pumps[pid].value(), 15.0)

    # 5 — unconfigured pumps' rows hide
    def test_unconfigured_pump_rows_hide(self):
        c = self._ctrl()
        hw = mock.MagicMock()
        hw.configured_pump_ids = ["P1"]
        c._hardware_config = hw
        p, _ = self._panel(ctrl=c)
        p._resolve_speeds()
        self.assertFalse(p.row_p_pct_pumps["P1"].isHidden())
        self.assertTrue(p.row_p_pct_pumps["P2"].isHidden())
        self.assertTrue(p.row_p_pct_pumps["P3"].isHidden())


# ── Controller-level: per-pump anchor + % store ─────────────────────

class _FakeSL:
    def __init__(self, rates):
        self._rates = rates

    def get_max_flow_rate(self, pid):
        return self._rates.get(pid, 0.0)


class _FakeHW:
    def __init__(self, ids):
        self.configured_pump_ids = ids


class TestControllerPerPumpAnchor(unittest.TestCase):

    def _fake(self, hw, sl):
        f = type("F", (), {})()
        f._hardware_config = hw
        f.safety_limits = sl
        f._pump_jog_max_native = \
            lambda: StageController._pump_jog_max_native(f)
        return f

    def test_per_pump_flow_limit_is_the_anchor(self):
        f = self._fake(_FakeHW(["P1", "P2"]), _FakeSL({"P1": 8.0, "P2": 0.5}))
        self.assertAlmostEqual(
            StageController.get_max_pump_feedrate_for(f, "P1"), 8.0)
        self.assertAlmostEqual(
            StageController.get_max_pump_feedrate_for(f, "P2"), 0.5)
        # The shared anchor would have been the fastest pump (8.0) for both —
        # the per-pump resolver is what fixes the 16× over-anchor on P2.

    def test_no_computed_limit_defaults_10(self):
        f = self._fake(_FakeHW(["P1"]), _FakeSL({}))
        self.assertAlmostEqual(
            StageController.get_max_pump_feedrate_for(f, "P1"), 10.0)

    def test_legacy_no_config_delegates_to_native(self):
        sl = _FakeSL({})
        sl.max_pump_feedrate = 120.0            # legacy mm/min → mm/s ÷ 60
        f = self._fake(None, sl)
        self.assertAlmostEqual(
            StageController.get_max_pump_feedrate_for(f, "P1"), 2.0)

    def test_pct_store_roundtrip_and_isolation(self):
        f = type("F", (), {})()
        f._pump_jog_pct = {}
        StageController.set_pump_jog_pct(f, "P2", 12.5)
        out = StageController.get_pump_jog_pcts(f)
        self.assertEqual(out, {"P2": 12.5})
        out["P2"] = 99.0                        # a copy, not the store
        self.assertEqual(StageController.get_pump_jog_pcts(f)["P2"], 12.5)
        StageController.set_pump_jog_pct(f, "P1", "not-a-number")
        self.assertNotIn("P1", StageController.get_pump_jog_pcts(f))


# ── Position readout: the value text keeps priority over the bar ────

class TestPositionReadoutTextPriority(_QtBase):

    def test_value_label_min_width_tracks_text_and_font(self):
        from gui.pages.hardware.control_panel import PositionValueLabel
        from gui.scaling import s
        lbl = PositionValueLabel()
        lbl.setText("-123,456.8")
        fm = lbl.fontMetrics()
        self.assertGreaterEqual(
            lbl.minimumWidth(), fm.horizontalAdvance("-123,456.8"))
        before = lbl.minimumWidth()
        f = lbl.font()
        f.setPointSizeF(f.pointSizeF() * 2)
        lbl.setFont(f)                          # FontChange → re-sync
        self.assertGreater(lbl.minimumWidth(), before)
        lbl.setText("x" * 500)                  # pathological → capped
        self.assertLessEqual(lbl.minimumWidth(), s(160))

    def test_control_panel_rows_bar_yields_text_wins(self):
        from PySide6.QtWidgets import QSizePolicy
        from gui.pages.hardware.control_panel import (
            HardwareControlPanel, PositionValueLabel)
        from gui.scaling import s
        p = HardwareControlPanel(show_connect=False, bypass_safety=False,
                                 pump_action_labels=True)
        for axis in ("X", "Y", "Z", "P1", "P2", "P3"):
            bar = p.bar_pos[axis]
            self.assertEqual(bar.sizePolicy().horizontalPolicy(),
                             QSizePolicy.Ignored)
            self.assertLessEqual(bar.minimumWidth(), s(10))
            self.assertIsInstance(p.lbl_pos[axis], PositionValueLabel)
        # A real long readout claims its full text width.
        p.lbl_pos["X"].setText("114,332.0")
        fm = p.lbl_pos["X"].fontMetrics()
        self.assertGreaterEqual(p.lbl_pos["X"].minimumWidth(),
                                fm.horizontalAdvance("114,332.0"))

    def test_position_readout_card_matches(self):
        from PySide6.QtWidgets import QSizePolicy
        from gui.pages.hardware.control_panel import PositionValueLabel
        from gui.widgets.context_sections import (
            PositionReadoutCard, SectionContext)
        from gui.scaling import s
        card = PositionReadoutCard(SectionContext(), {})
        for axis in ("X", "Y", "Z", "P1", "P2", "P3"):
            bar = card._bar[axis]
            self.assertEqual(bar.sizePolicy().horizontalPolicy(),
                             QSizePolicy.Ignored)
            self.assertLessEqual(bar.minimumWidth(), s(10))
            self.assertIsInstance(card._lbl[axis], PositionValueLabel)
        card._set("X", 114332.0, "{:,.1f}")
        fm = card._lbl["X"].fontMetrics()
        self.assertGreaterEqual(card._lbl["X"].minimumWidth(),
                                fm.horizontalAdvance("114,332.0"))


if __name__ == "__main__":
    unittest.main()
