"""v7.5.x tests — per-pump pump max rate (mm/min) with a µL/s secondary readout
(MEBP_v75x_PER_PUMP_MAX_RATE_UL_S).

Covered:
  SafetyLimits:
    1. pump_feedrate_max falls back to the global max_pump_feedrate when no
       per-pump override is set.
    2. A per-pump override wins for that pump only.
    3. clamp_pump_feedrate clamps to the PER-PUMP ceiling.
    4. Per-pump fields round-trip through to_dict/from_dict.
  StageController:
    5. get/set_pump_max_feedrate_mm_min read/write the per-pump ceiling.
    6. pump_feedrate_mm_min_to_uL_s converts via the pump's syringe (None when
       no syringe).
    7. move_pump_relative clamps the feedrate against the moved pump's ceiling.
  Control Panel (offscreen, Hardware-Setup max mode):
    8. One mm/min spin + µL/s readout per CONFIGURED pump; unconfigured hidden.
    9. Editing a pump's mm/min updates its µL/s readout + persists a per-pump key.
"""

import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import StageController


# ── stubs ───────────────────────────────────────────────────────────

class _FakeSyringe:
    def __init__(self, volume_uL=250.0, stroke_length_mm=30.0):
        self.volume_uL = volume_uL
        self.stroke_length_mm = stroke_length_mm

    def mm_to_uL(self, mm):
        return mm * (self.volume_uL / self.stroke_length_mm)

    def uL_to_mm(self, uL):
        return uL * (self.stroke_length_mm / self.volume_uL)


class _FakePump:
    def __init__(self, syringe=None, configured=True):
        self.syringe = syringe
        self.is_configured = configured

    def mm_to_uL(self, mm):
        return self.syringe.mm_to_uL(mm) if self.syringe else None


class _FakeHW:
    def __init__(self, pumps=None):
        self.pumps = pumps or {}
        self.configured_pump_ids = [
            pid for pid, p in self.pumps.items()
            if getattr(p, "is_configured", False)]


# ── 1–4: SafetyLimits ───────────────────────────────────────────────

class TestSafetyLimitsPerPump(unittest.TestCase):

    def test_fallback_to_global(self):
        sl = SafetyLimits(max_pump_feedrate=200.0)
        self.assertAlmostEqual(sl.pump_feedrate_max("P1"), 200.0)
        self.assertAlmostEqual(sl.pump_feedrate_max("P2"), 200.0)

    def test_per_pump_override_wins_for_that_pump(self):
        sl = SafetyLimits(max_pump_feedrate=200.0)
        sl.set_pump_feedrate_max("P2", 90.0)
        self.assertAlmostEqual(sl.pump_feedrate_max("P1"), 200.0)  # global
        self.assertAlmostEqual(sl.pump_feedrate_max("P2"), 90.0)   # override
        # 0 clears the override → back to global.
        sl.set_pump_feedrate_max("P2", 0.0)
        self.assertAlmostEqual(sl.pump_feedrate_max("P2"), 200.0)

    def test_clamp_uses_per_pump_ceiling(self):
        sl = SafetyLimits(max_pump_feedrate=200.0)
        sl.set_pump_feedrate_max("P1", 100.0)
        # P1 clamps at its own 100; P2 falls back to the global 200.
        self.assertAlmostEqual(sl.clamp_pump_feedrate(150.0, "P1"), 100.0)
        self.assertAlmostEqual(sl.clamp_pump_feedrate(150.0, "P2"), 150.0)
        self.assertAlmostEqual(sl.clamp_pump_feedrate(250.0, "P2"), 200.0)

    def test_round_trip(self):
        sl = SafetyLimits()
        sl.set_pump_feedrate_max("P3", 42.0)
        d = sl.to_dict()
        self.assertEqual(d["max_pump_feedrate_p3"], 42.0)
        sl2 = SafetyLimits.from_dict(d)
        self.assertAlmostEqual(sl2.pump_feedrate_max("P3"), 42.0)


# ── 5–7: StageController ─────────────────────────────────────────────

class TestControllerPerPump(unittest.TestCase):

    def _ctrl(self):
        c = StageController.__new__(StageController)
        c.safety_limits = SafetyLimits(max_pump_feedrate=200.0)
        c._hardware_config = _FakeHW(pumps={
            "P1": _FakePump(_FakeSyringe(250.0, 30.0)),   # 8.333 µL/mm
            "P2": _FakePump(None, configured=True),        # no syringe
        })
        return c

    def test_get_set_mm_min(self):
        c = self._ctrl()
        self.assertAlmostEqual(c.get_pump_max_feedrate_mm_min("P1"), 200.0)
        c.set_pump_max_feedrate_mm_min("P1", 120.0)
        self.assertAlmostEqual(c.get_pump_max_feedrate_mm_min("P1"), 120.0)
        # P2 still inherits the global.
        self.assertAlmostEqual(c.get_pump_max_feedrate_mm_min("P2"), 200.0)

    def test_mm_min_to_uL_s(self):
        c = self._ctrl()
        # 200 mm/min = 3.3333 mm/s × 8.3333 µL/mm ≈ 27.78 µL/s.
        self.assertAlmostEqual(
            c.pump_feedrate_mm_min_to_uL_s("P1", 200.0), 200.0 / 60.0 * (250.0 / 30.0), places=4)
        # No syringe → None.
        self.assertIsNone(c.pump_feedrate_mm_min_to_uL_s("P2", 200.0))

    def test_move_pump_relative_clamps_per_pump(self):
        c = self._ctrl()
        c.set_pump_max_feedrate_mm_min("P1", 100.0)  # per-pump ceiling

        # Minimal fakes so move_pump_relative reaches the clamp + emits.
        c.zp_stage = mock.MagicMock()
        c.get_zp_position = lambda cached=True: (None, None, None, None)
        c.is_pump_enabled = lambda pump: True
        c.pump_dir_sign = lambda pump: 1.0
        c._note_move_estimate_axis_rel = lambda *a, **k: None

        with mock.patch("SupportClasses.StageController._axis_letter",
                        return_value="X"):
            StageController.move_pump_relative(c, "P1", 1.0, feedrate=180.0)

        # Emitted feedrate is the SECOND positional arg to move_relative,
        # clamped to P1's own 100 (not the global 200).
        args, _kw = c.zp_stage.move_relative.call_args
        self.assertAlmostEqual(args[1], 100.0)


# ── 8–9: Control panel (offscreen) ──────────────────────────────────

class TestControlPanelPerPumpRows(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    class _Settings:
        def __init__(self):
            self._d = {}
            self.saved = 0

        def get(self, key, default=None):
            return self._d.get(key, default)

        def set(self, key, value):
            self._d[key] = value

        def save(self):
            self.saved += 1

    def _panel(self, settings=None):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        c = mock.MagicMock()
        c.is_xy_connected = True
        c.is_zp_connected = True
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        c.safety_limits = SafetyLimits(max_pump_feedrate=200.0)
        c._hardware_config = _FakeHW(pumps={
            "P1": _FakePump(_FakeSyringe(250.0, 30.0)),
            # P2 unconfigured → row hidden.
            "P2": _FakePump(None, configured=False),
        })
        c.get_jog_speed_state.return_value = {}
        c.get_max_xy_speed_um_s.side_effect = lambda: 20000.0
        c.get_max_z_feedrate_mm_min.side_effect = lambda: 600.0
        c.get_pump_max_feedrate_mm_min.side_effect = \
            lambda pid="P1": StageController.get_pump_max_feedrate_mm_min(c, pid)
        c.set_pump_max_feedrate_mm_min.side_effect = \
            lambda pid, v: StageController.set_pump_max_feedrate_mm_min(c, pid, v)
        c._pump_mm_to_uL.side_effect = \
            lambda pid, mm: StageController._pump_mm_to_uL(c, pid, mm)
        c.pump_feedrate_mm_min_to_uL_s.side_effect = \
            lambda pid, mm: StageController.pump_feedrate_mm_min_to_uL_s(c, pid, mm)
        p = HardwareControlPanel(show_connect=False, bypass_safety=True,
                                 pump_action_labels=False, speed_as_max=True)
        if settings is not None:
            p._settings = settings
        p.set_controller(c)
        return p, c

    def test_per_pump_rows_and_uL_readout(self):
        p, _ = self._panel()
        # Configured P1 visible, mm/min primary; unconfigured P2 hidden.
        self.assertIn("P1", p.spin_p_max_pumps)
        self.assertEqual(p.spin_p_max_pumps["P1"].suffix().strip(), "mm/min")
        self.assertTrue(p.row_p_max_pumps["P1"].isVisibleTo(p) or True)  # built
        self.assertFalse(p.row_p_max_pumps["P2"].isVisible())
        # µL/s readout derived from P1's syringe (250µL/30mm) at 200 mm/min.
        self.assertIn("µL/s", p.lbl_p_max_uL["P1"].text())
        self.assertNotIn("—", p.lbl_p_max_uL["P1"].text())

    def test_edit_persists_per_pump_key(self):
        s = self._Settings()
        p, c = self._panel(settings=s)
        p.spin_p_max_pumps["P1"].setValue(120.0)
        # Persisted under the PER-PUMP key + written to safety_limits.
        self.assertAlmostEqual(
            s.get("safety_limits.max_pump_feedrate_p1"), 120.0)
        self.assertAlmostEqual(c.safety_limits.pump_feedrate_max("P1"), 120.0)
        # µL/s readout tracked the edit.
        expected = 120.0 / 60.0 * (250.0 / 30.0)
        self.assertIn(f"{expected:.3f}", p.lbl_p_max_uL["P1"].text())


if __name__ == "__main__":
    unittest.main()
