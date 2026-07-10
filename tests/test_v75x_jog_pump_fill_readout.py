"""
v7.5.x — Jog page syringe readout shows the LIVE plunger fill.

Bug: the Jog page pump rack displayed ``FluidColumn.ink_volume_uL`` from the
static hardware-config object. That column is only mutated during a
print/workflow (``aspirate_ink``/``dispense``), so on the Jog page it never
reflected the actual plunger position — the volume "did not show and did not
update". Fix: feed the rack the live plunger fill derived from the live plunger
position via the controller's plunger calibration (the same readout the
Hardware Control Panel shows), falling back to the raw plunger position (mm)
for an un-calibrated pump.

Two layers are covered:
  * the widget — :meth:`PumpColumn.set_live_fill` / :meth:`PumpRack.update_live_fills`
  * the Jog page — :meth:`JogControlPage._refresh_pump_panel` builds the live
    snapshot from a real ``StageController`` fill conversion + updates per tick.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits


# ── Fakes mirroring tests/test_v75x_pump_plunger_setup.py ──────────────────

class _Syr:
    def __init__(self, volume_uL=250, stroke_mm=35.0):
        self.volume_uL = volume_uL
        self.stroke_length_mm = stroke_mm

    def mm_to_uL(self, mm):
        return mm * (self.volume_uL / self.stroke_length_mm)

    def uL_to_mm(self, uL):
        return uL * (self.stroke_length_mm / self.volume_uL)


class _Ink:
    # Mirror the real InkSpec: the hex display colour lives on `.color`.
    def __init__(self, name="Ink A", color="#a6e3a1"):
        self.name = name
        self.color = color


class _FluidCol:
    def __init__(self, ink=None):
        self.ink_spec = ink


class _PumpCfg:
    def __init__(self, syr=None, ink=None):
        self.is_configured = syr is not None
        self.syringe = syr
        self.fluid_column = _FluidCol(ink)

    def mm_to_uL(self, mm):
        return self.syringe.mm_to_uL(mm)

    def uL_to_mm(self, uL):
        return self.syringe.uL_to_mm(uL)


class _HW:
    def __init__(self, pumps):
        self.pumps = pumps


def _controller():
    """Real StageController fill math (apply_pump_setup / raw_to_pump_fill_uL),
    with only the hardware reads (get_zp_position / zp_logical_value) stubbed."""
    c = StageController.__new__(StageController)
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0,
                       "P1": 0.0, "P2": 0.0, "P3": 0.0}
    c.safety_limits = SafetyLimits()
    c._axis_flip = {"Z": False, "P1": False, "P2": False, "P3": False}
    c._pump_aspirate_sign = {"P1": 1.0, "P2": 1.0, "P3": 1.0}
    c._pump_setup = {}
    c._hardware_config = _HW({
        "P1": _PumpCfg(_Syr(), _Ink("Ink A", "#a6e3a1")),  # calibrated below
        "P2": _PumpCfg(_Syr()),                            # configured, no cal
        "P3": _PumpCfg(None),                              # unconfigured
    })
    # Stubbed live plunger raw position per pump (mm).
    c._raw = {"P1": 40.0, "P2": 0.0, "P3": 0.0}
    c.get_zp_position = lambda cached=True: (0.0,)         # [0] not None → readable
    c.zp_logical_value = lambda zp, pid: c._raw.get(pid)
    return c


# ── Widget layer ───────────────────────────────────────────────────────────

class TestPumpColumnLiveFill(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _col(self):
        from gui.widgets.pump_rack import PumpColumn
        return PumpColumn(pump_id="P1")

    def test_calibrated_shows_uL_fill_and_bar(self):
        col = self._col()
        col.set_live_fill(fill_uL=125.0, capacity_uL=250.0, raw_mm=22.5,
                          syringe=_Syr(), ink_spec=_Ink(name="Ink A",
                          color="#ff0000"), calibrated=True)
        self.assertEqual(col._vol_label.text(), "125.0 µL")
        # Bar fills proportionally (half full).
        self.assertAlmostEqual(col._barrel._fractions["ink"], 0.5, places=3)
        self.assertAlmostEqual(col._barrel._fractions["empty"], 0.5, places=3)
        # Subtext carries capacity + ink name.
        self.assertIn("250", col._sub_label.text())
        self.assertIn("Ink A", col._sub_label.text())
        # The assigned ink's colour (InkSpec.color, not display_color) is the
        # barrel accent — regression guard for the display_color misnomer.
        self.assertEqual(col._barrel._ink_color, "#ff0000")

    def test_no_ink_uses_neutral_accent(self):
        from gui.widgets.pump_rack import DEFAULT_INK_COLOR
        col = self._col()
        col.set_live_fill(fill_uL=50.0, capacity_uL=100.0, raw_mm=5.0,
                          syringe=_Syr(volume_uL=100), ink_spec=None,
                          calibrated=True)
        # No ink assigned → neutral accent, not the green ink default.
        self.assertNotEqual(col._barrel._ink_color, DEFAULT_INK_COLOR)
        self.assertIn("full", col._sub_label.text())

    def test_fill_fraction_clamped(self):
        col = self._col()
        # Plunger slightly past the captured "full" extreme → clamp to 1.0.
        col.set_live_fill(fill_uL=260.0, capacity_uL=250.0, raw_mm=0.0,
                          syringe=_Syr(), ink_spec=None, calibrated=True)
        self.assertLessEqual(col._barrel._fractions["ink"], 1.0)
        self.assertGreaterEqual(col._barrel._fractions["empty"], 0.0)
        self.assertEqual(col._vol_label.text(), "260.0 µL")  # true value shown

    def test_low_fill_warns(self):
        col = self._col()
        col.set_live_fill(fill_uL=2.0, capacity_uL=250.0, raw_mm=39.0,
                          syringe=_Syr(), ink_spec=None, calibrated=True)
        self.assertTrue(col._barrel._warning)

    def test_uncalibrated_shows_raw_mm_and_hint(self):
        col = self._col()
        col.set_live_fill(fill_uL=None, capacity_uL=None, raw_mm=12.34,
                          syringe=_Syr(), ink_spec=None, calibrated=False)
        self.assertEqual(col._vol_label.text(), "12.34 mm")
        self.assertIn("calibrate", col._sub_label.text().lower())
        self.assertFalse(col._barrel._warning)

    def test_no_syringe_is_not_configured(self):
        col = self._col()
        col.set_live_fill(fill_uL=None, capacity_uL=None, raw_mm=None,
                          syringe=None, ink_spec=None, calibrated=False)
        self.assertEqual(col._vol_label.text(), "—")
        self.assertEqual(col._sub_label.text(), "not configured")

    def test_rack_routes_and_unconfigures_on_none(self):
        from gui.widgets.pump_rack import PumpRack
        rack = PumpRack()
        rack.update_live_fills({
            "P1": dict(fill_uL=50.0, capacity_uL=100.0, raw_mm=5.0,
                       syringe=_Syr(volume_uL=100), ink_spec=None,
                       calibrated=True),
            "P2": None,
            "P3": None,
        })
        self.assertEqual(rack.columns["P1"]._vol_label.text(), "50.0 µL")
        self.assertEqual(rack.columns["P2"]._vol_label.text(), "—")
        self.assertEqual(rack.columns["P3"]._sub_label.text(), "not configured")


# ── Perf: per-tick restyle throttle (v7.5.x slow-over-time regression) ──────

class TestPumpColumnRestyleThrottle(unittest.TestCase):
    """set_live_fill runs on EVERY 300ms + 33ms tick. setStyleSheet forces a
    full QSS re-parse; re-applying it every tick made the UI uniformly sluggish.
    The column must restyle ONLY when the visual style state changes, while text
    and bar fraction still update every tick."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _col_with_counter(self):
        from gui.widgets.pump_rack import PumpColumn
        col = PumpColumn(pump_id="P1")
        counts = {"n": 0}
        # Wrap the four stylesheet-applying helpers to count invocations.
        for name in ("_set_vol_style", "_set_sub_style",
                     "_refresh_card_style", "_refresh_chip_style"):
            orig = getattr(col, name)

            def wrap(*a, _orig=orig, **k):
                counts["n"] += 1
                return _orig(*a, **k)
            setattr(col, name, wrap)
        return col, counts

    def test_repeated_same_state_does_not_restyle(self):
        col, counts = self._col_with_counter()
        syr, ink = _Syr(), _Ink("Ink A", "#a6e3a1")
        col.set_live_fill(fill_uL=200.0, capacity_uL=250.0, raw_mm=22.5,
                          syringe=syr, ink_spec=ink, calibrated=True)
        first = counts["n"]
        self.assertGreater(first, 0)   # first call establishes the style
        # 20 more ticks: fill changes (well above the 5 µL warn line) but the
        # style-affecting state does not → NO further stylesheet application.
        for i in range(20):
            col.set_live_fill(fill_uL=200.0 - i, capacity_uL=250.0,
                              raw_mm=22.5, syringe=syr, ink_spec=ink,
                              calibrated=True)
        self.assertEqual(counts["n"], first,
                         "stylesheet re-applied despite unchanged style state")
        # …but the readout still tracks the plunger every tick.
        self.assertEqual(col._vol_label.text(), f"{200.0 - 19:.1f} µL")

    def test_warning_flip_restyles(self):
        col, counts = self._col_with_counter()
        syr, ink = _Syr(), _Ink("Ink A", "#a6e3a1")
        col.set_live_fill(fill_uL=200.0, capacity_uL=250.0, raw_mm=22.5,
                          syringe=syr, ink_spec=ink, calibrated=True)
        base = counts["n"]
        # Cross the low-ink threshold → warning colour flips → must restyle.
        col.set_live_fill(fill_uL=1.0, capacity_uL=250.0, raw_mm=39.0,
                          syringe=syr, ink_spec=ink, calibrated=True)
        self.assertGreater(counts["n"], base)

    def test_accent_change_restyles(self):
        col, counts = self._col_with_counter()
        syr = _Syr()
        col.set_live_fill(fill_uL=100.0, capacity_uL=250.0, raw_mm=22.5,
                          syringe=syr, ink_spec=_Ink("A", "#a6e3a1"),
                          calibrated=True)
        base = counts["n"]
        # Different ink colour → accent changes → must restyle.
        col.set_live_fill(fill_uL=100.0, capacity_uL=250.0, raw_mm=22.5,
                          syringe=syr, ink_spec=_Ink("B", "#f38ba8"),
                          calibrated=True)
        self.assertGreater(counts["n"], base)

    def test_unconfigured_repeated_does_not_restyle(self):
        col, counts = self._col_with_counter()
        col.set_live_fill(fill_uL=None, capacity_uL=None, raw_mm=None,
                          syringe=None, calibrated=False)
        first = counts["n"]
        for _ in range(10):
            col.set_live_fill(fill_uL=None, capacity_uL=None, raw_mm=None,
                              syringe=None, calibrated=False)
        self.assertEqual(counts["n"], first,
                         "unconfigured placeholder re-styled every tick")


# ── Jog page layer ───────────────────────────────────────────────────────

class TestJogRefreshPumpPanel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, ctrl):
        from gui.pages.jog_control import JogControlPage
        from gui.widgets.pump_rack import PumpRack
        page = JogControlPage.__new__(JogControlPage)   # skip heavy __init__
        page.controller = ctrl
        page._hardware_config = ctrl._hardware_config
        page._pump_panel = PumpRack()
        return page

    def test_calibrated_pump_shows_live_fill(self):
        ctrl = _controller()
        # Empty (all-in) at raw 40, full (all-out) at raw 5 → fill 0 at 40.
        ctrl.apply_pump_setup("P1", raw_dispensed_mm=40.0, raw_aspirated_mm=5.0)
        page = self._page(ctrl)

        page._refresh_pump_panel()
        col = page._pump_panel.columns["P1"]
        self.assertEqual(col._vol_label.text(), "0.0 µL")   # plunger at empty

    def test_fill_updates_when_plunger_moves(self):
        ctrl = _controller()
        ctrl.apply_pump_setup("P1", 40.0, 5.0)              # 250 µL over 35 mm
        page = self._page(ctrl)

        page._refresh_pump_panel()
        self.assertEqual(page._pump_panel.columns["P1"]._vol_label.text(),
                         "0.0 µL")

        # Aspirate halfway (raw → 22.5, the midpoint) and re-tick.
        ctrl._raw["P1"] = 22.5
        page._refresh_pump_panel()
        self.assertEqual(page._pump_panel.columns["P1"]._vol_label.text(),
                         "125.0 µL")

        # Fully aspirated.
        ctrl._raw["P1"] = 5.0
        page._refresh_pump_panel()
        self.assertEqual(page._pump_panel.columns["P1"]._vol_label.text(),
                         "250.0 µL")

    def test_uncalibrated_pump_shows_live_raw_mm(self):
        ctrl = _controller()
        ctrl.apply_pump_setup("P1", 40.0, 5.0)              # only P1 calibrated
        page = self._page(ctrl)

        # Non-zero datum: the fallback must show the RAW Marlin value (matching
        # the embedded control panel), NOT raw − zero_position.
        ctrl.zero_position["P2"] = 3.0
        ctrl._raw["P2"] = 7.0
        page._refresh_pump_panel()
        col = page._pump_panel.columns["P2"]
        self.assertEqual(col._vol_label.text(), "7.00 mm")   # raw, not 4.00
        self.assertIn("calibrate", col._sub_label.text().lower())

        # Still updates when the plunger moves.
        ctrl._raw["P2"] = 9.5
        page._refresh_pump_panel()
        self.assertEqual(page._pump_panel.columns["P2"]._vol_label.text(),
                         "9.50 mm")

    def test_unconfigured_pump_placeholder(self):
        ctrl = _controller()
        page = self._page(ctrl)
        page._refresh_pump_panel()
        col = page._pump_panel.columns["P3"]
        self.assertEqual(col._vol_label.text(), "—")
        self.assertEqual(col._sub_label.text(), "not configured")

    def test_no_zp_read_falls_back_gracefully(self):
        ctrl = _controller()
        ctrl.apply_pump_setup("P1", 40.0, 5.0)
        # Board not answering → get_zp_position()[0] is None.
        ctrl.get_zp_position = lambda cached=True: (None,)
        page = self._page(ctrl)
        page._refresh_pump_panel()   # must not raise
        # Calibrated pump with no live read → uncalibrated-style fallback
        # (no raw_mm → em dash), not a stale frozen value.
        self.assertEqual(page._pump_panel.columns["P1"]._vol_label.text(), "—")


if __name__ == "__main__":
    unittest.main()
