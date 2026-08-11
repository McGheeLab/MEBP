"""v7.9.1 — two operator reports about the jog panels.

1. "on all the pump jog pannels we need independent control of rate for each
   pump" — per-pump rate DID exist, but only inside `HardwareControlPanel`.
   Two surfaces bypassed it: the Settings-page context panel sent NO rate at
   all, and the Xbox anchored all three pumps to one shared ceiling.

2. "under the jog pannel the live position pannel the current position of the
   axis is overtop of the units" — the value label's `Ignored` size policy and
   its tracking `minimumWidth` fought each other: the grid sized the column
   without the label, the label refused to shrink, and it drew over the unit.
"""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QSizePolicy       # noqa: E402

from SupportClasses.StageController import ZPJogHandler       # noqa: E402

#: Three pumps whose safe flow ceilings differ the way real bores do — v7.9
#: measured ~2000x between a 22G bore and a 30 µm pulled tip.
CEILINGS = {"P1": 8.0, "P2": 0.5, "P3": 2.0}


# ── per-pump rate ──────────────────────────────────────────────────

class TestSettingsPanelSendsAPerPumpRate(unittest.TestCase):
    """It passed no rate at all, so every pump ran at move_pump_uL's internal
    default while the same pump on every other tile ran at its own flow."""

    def _page(self, pcts=None):
        from gui.pages.settings_page import SettingsPage
        ctrl = MagicMock()
        ctrl.get_pump_jog_pcts.return_value = (
            {"P1": 50.0, "P2": 10.0, "P3": 25.0} if pcts is None else pcts)
        ctrl.get_max_pump_feedrate_for.side_effect = lambda p: CEILINGS[p]
        page = SettingsPage.__new__(SettingsPage)
        page.controller = ctrl
        return page, ctrl

    def test_each_pump_gets_its_own_rate(self):
        page, ctrl = self._page()
        for p in ("P1", "P2", "P3"):
            page._ctx_jog_pump(p, -1.0)
        got = {c.args[0]: c.kwargs.get("rate_uL_s")
               for c in ctrl.move_pump_uL.call_args_list}
        self.assertAlmostEqual(got["P1"], 4.00)      # 50 % of 8.0
        self.assertAlmostEqual(got["P2"], 0.05)      # 10 % of 0.5
        self.assertAlmostEqual(got["P3"], 0.50)      # 25 % of 2.0
        self.assertEqual(len(set(got.values())), 3,
                         "the three pumps must not share a rate")

    def test_the_volume_still_reaches_the_micro_litre_entry_point(self):
        """The v7.9.x units fix must survive: a signed µL volume, not mm."""
        page, ctrl = self._page()
        page._ctx_jog_pump("P1", -2.5)
        self.assertEqual(ctrl.move_pump_uL.call_args.args, ("P1", -2.5))
        ctrl.move_pump_relative.assert_not_called()

    def test_an_unset_percentage_keeps_the_legacy_default(self):
        """None, not 0 — a wrong rate is worse than the old behaviour."""
        page, ctrl = self._page(pcts={})
        page._ctx_jog_pump("P1", -1.0)
        self.assertIsNone(ctrl.move_pump_uL.call_args.kwargs.get("rate_uL_s"))

    def test_a_broken_controller_degrades_instead_of_raising(self):
        from gui.pages.settings_page import SettingsPage
        ctrl = MagicMock()
        ctrl.get_pump_jog_pcts.side_effect = RuntimeError("boom")
        page = SettingsPage.__new__(SettingsPage)
        page.controller = ctrl
        page._ctx_jog_pump("P1", -1.0)                  # must not raise
        self.assertIsNone(ctrl.move_pump_uL.call_args.kwargs.get("rate_uL_s"))

    def test_it_reads_the_SHARED_controller_state(self):
        """This panel has no speed widget; it must read the same cross-page
        per-pump state HardwareControlPanel writes, or the two would diverge."""
        page, ctrl = self._page()
        page._ctx_jog_pump("P2", -1.0)
        ctrl.get_pump_jog_pcts.assert_called()
        ctrl.get_max_pump_feedrate_for.assert_called_with("P2")


class TestXboxResolvesAPerPumpCeiling(unittest.TestCase):

    def _handler(self, limits=True):
        h = ZPJogHandler.__new__(ZPJogHandler)
        h.safety_limits = (
            SimpleNamespace(get_max_flow_rate=lambda p: CEILINGS[p])
            if limits else None)
        h.p_speed_pct = 50.0
        h.p_speed = 10.0
        h.p_speed_max = 10.0
        h.max_speed = 100.0
        h._hardware_config = SimpleNamespace(pumps={
            p: SimpleNamespace(is_configured=True,
                               uL_to_mm=lambda v: v / 10.0)
            for p in CEILINGS})
        return h

    def test_the_ceiling_is_per_pump(self):
        h = self._handler()
        for p, want in CEILINGS.items():
            self.assertAlmostEqual(h._xbox_pump_ceiling_uL_s(p), want)

    def test_a_full_deflection_gives_three_different_speeds(self):
        """One shared anchor drove a fine bore at a coarse bore's rate."""
        h = self._handler()
        vels = {p: h._pump_vel_mm_s(1.0, p) for p in CEILINGS}
        self.assertEqual(len(set(round(v, 9) for v in vels.values())), 3)
        # and they keep the ceilings' ratio
        self.assertAlmostEqual(vels["P1"] / vels["P2"],
                               CEILINGS["P1"] / CEILINGS["P2"], places=6)

    def test_direction_still_follows_the_stick(self):
        h = self._handler()
        self.assertLess(h._pump_vel_mm_s(-1.0, "P1"), 0)
        self.assertGreater(h._pump_vel_mm_s(1.0, "P1"), 0)

    def test_the_percentage_stays_the_shared_ladder(self):
        """The Xbox has ONE physical speed control for the pump group; per-pump
        percentages would leave its buttons with no defined meaning."""
        h = self._handler()
        h.p_speed_pct = 100.0
        fast = h._pump_vel_mm_s(1.0, "P1")
        h.p_speed_pct = 50.0
        self.assertAlmostEqual(h._pump_vel_mm_s(1.0, "P1"), fast / 2.0,
                               places=9)

    def test_no_safety_limits_falls_back_to_the_legacy_scalars(self):
        """A rig with no flow limits computed must jog exactly as before."""
        h = self._handler(limits=False)
        self.assertEqual(h._xbox_pump_ceiling_uL_s("P1"), 0.0)
        self.assertGreater(h._pump_vel_mm_s(1.0, "P1"), 0.0)

    def test_a_raising_limits_object_does_not_break_the_jog(self):
        h = self._handler()
        h.safety_limits = SimpleNamespace(
            get_max_flow_rate=lambda p: (_ for _ in ()).throw(RuntimeError()))
        self.assertEqual(h._xbox_pump_ceiling_uL_s("P1"), 0.0)
        self.assertGreater(h._pump_vel_mm_s(1.0, "P1"), 0.0)


# ── the Live Position row ──────────────────────────────────────────

class TestPositionValueDoesNotOverlapTheUnit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    #: Long, realistic readings from this machine (µm X/Y, mm Z and plungers).
    VALUES = {"X": "114332.0", "Y": "76645.0", "Z": "-101.980",
              "P1": "-42.000", "P2": "-16.550", "P3": "-13.660"}

    def _panel(self):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        from gui.widgets.standard_jog_context import StandardJogContextPanel
        panel = StandardJogContextPanel()
        self.addCleanup(panel.deleteLater)
        cp = panel.findChildren(HardwareControlPanel)[0]
        for ax, v in self.VALUES.items():
            cp.lbl_pos[ax].setText(v)
        return panel, cp

    def test_the_value_never_overlaps_the_unit_at_any_width(self):
        """The bug, measured: the label was ~108 px wide inside a ~5 px cell,
        so the number drew straight over the unit at EVERY width."""
        panel, cp = self._panel()
        for width in (540, 380, 300, 260, 200, 160):
            panel.resize(width, 900)
            panel.show()
            QApplication.processEvents()
            for ax in self.VALUES:
                val, unit = cp.lbl_pos[ax], cp.unit_lbl_pos[ax]
                v_right = val.geometry().x() + val.geometry().width()
                self.assertLessEqual(
                    v_right, unit.geometry().x(),
                    f"{ax} at width {width}: value overlaps the unit")

    def test_the_number_is_never_clipped(self):
        panel, cp = self._panel()
        for width in (540, 300, 200):
            panel.resize(width, 900)
            panel.show()
            QApplication.processEvents()
            for ax in self.VALUES:
                val = cp.lbl_pos[ax]
                need = val.fontMetrics().horizontalAdvance(val.text())
                self.assertGreaterEqual(val.width(), need,
                                        f"{ax} clipped at width {width}")

    def test_the_policy_is_not_Ignored(self):
        """⚠ The regression in one line. `Ignored` tells the LAYOUT to size the
        column without this widget, while setMinimumWidth forces the WIDGET
        to stay that wide — the two together are what produced the overlap."""
        from gui.pages.hardware.control_panel import PositionValueLabel
        lbl = PositionValueLabel()
        self.addCleanup(lbl.deleteLater)
        self.assertNotEqual(lbl.sizePolicy().horizontalPolicy(),
                            QSizePolicy.Ignored)

    def test_the_bar_is_still_the_element_that_yields(self):
        """The number keeps priority only because the BAR gives way; if the bar
        stopped yielding, a narrow panel would clip the number again."""
        panel, cp = self._panel()
        bar = cp.bar_pos["X"]
        self.assertEqual(bar.sizePolicy().horizontalPolicy(),
                         QSizePolicy.Ignored)
        panel.resize(540, 900)
        panel.show()
        QApplication.processEvents()
        wide = bar.width()
        panel.resize(240, 900)
        QApplication.processEvents()
        self.assertLess(bar.width(), wide, "the bar did not give way")

    def test_the_minimum_width_still_tracks_the_text(self):
        from gui.pages.hardware.control_panel import PositionValueLabel
        lbl = PositionValueLabel()
        self.addCleanup(lbl.deleteLater)
        short = lbl.minimumWidth()
        lbl.setText("-114332.000")
        self.assertGreater(lbl.minimumWidth(), short)

    def test_the_custom_panel_copy_is_clean_too(self):
        """PositionReadoutCard builds the same row from the same class."""
        from gui.widgets.context_sections import SectionContext, build_section
        card = build_section("positions", SectionContext())
        self.addCleanup(card.deleteLater)
        for ax, v in self.VALUES.items():
            if ax in card._lbl:
                card._lbl[ax].setText(v)
        for width in (400, 260, 160, 100):
            card.resize(width, 400)
            card.show()
            QApplication.processEvents()
            for ax in card._lbl:
                val, unit = card._lbl[ax], card._unit[ax]
                v_right = val.geometry().x() + val.geometry().width()
                self.assertLessEqual(
                    v_right, unit.geometry().x(),
                    f"{ax} at width {width}: value overlaps the unit")


if __name__ == "__main__":
    unittest.main()
