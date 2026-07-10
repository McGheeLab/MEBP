"""
v7.5.x — pump compliance calibration + backlash compensation.

The pressure-relief / compliance value is now an absolute µL PER PUMP (retired
the old "% of syringe" HardwareConfig field + the 3 pump_relief_on_* toggles),
measured by the Needle Location compliance calibration (c = ½ the aspirate-back
volume) and persisted in device_profile.pump_compliance_uL. When backlash
compensation is enabled (a single toggle, restored from
device_profile.backlash_comp_enabled), every discrete pump actuation is
bracketed: take-up +c (fluid direction) → fluid V → unload −c (pressure-neutral).

These exercise the pure logic:
  * DeviceProfile round-trips pump_compliance_uL + backlash_comp_enabled;
  * StageController per-pump relief state (set/get/all/apply) + the toggle;
  * move_pump_uL(compensate=...) take-up/unload bracketing, its gating
    (settle=True auto only, compensate True/False force, c=0 no-op), and the
    volume-balanced-capture exemption;
  * HardwareConfig discards the legacy relief keys on load;
  * the Needle Location compliance-calibration card (c = A/2, save → controller).
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest
from types import SimpleNamespace
from unittest import mock

from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import StageController
from gui.pages.hardware.device_profile import DeviceProfile


# ── Fakes ───────────────────────────────────────────────────────────────

class _FakePumpCfg:
    is_configured = True
    syringe = SimpleNamespace(volume_uL=100.0)

    def uL_to_mm(self, v):
        return v * 0.3

    def feedrate_uL_s_to_mm_min(self, r):
        return r * 18.0


class _FakeHW:
    def __init__(self):
        self.pump_settle_time_s = 0.0
        self.pump_prime_time_s = 0.25
        self.pumps = {p: _FakePumpCfg() for p in ("P1", "P2", "P3")}


class _FakeSettings:
    """Dict-backed Settings supporting get/set + get_section/set_section."""

    def __init__(self, data=None):
        self._d = dict(data or {})
        self.saved = 0

    def get(self, key, default=None):
        return self._d.get(key, default)

    def set(self, key, val):
        self._d[key] = val

    def get_section(self, name):
        return self._d.get(name)

    def set_section(self, name, val):
        self._d[name] = val

    def save(self):
        self.saved += 1


def _ctrl(relief=None, backlash=False):
    c = StageController.__new__(StageController)
    c._hardware_config = _FakeHW()
    c.safety_limits = SafetyLimits()
    c.safety_limits.enabled = False
    c._pump_relief_uL = dict(relief or {})
    c._backlash_comp_enabled = backlash
    c.pump_moves = []
    c.move_pump_relative = (
        lambda pump, dist, fr=None: c.pump_moves.append((pump, dist, fr)))
    return c


def _run(ctrl, vol, **kw):
    with mock.patch("SupportClasses.StageController.time.sleep"):
        ctrl.move_pump_uL("P1", vol, rate_uL_s=10.0, **kw)


# ════════════════════════════════════════════════════════════════════
#  DeviceProfile persistence
# ════════════════════════════════════════════════════════════════════

class TestDeviceProfileCompliance(unittest.TestCase):

    def test_defaults(self):
        p = DeviceProfile()
        self.assertEqual(p.pump_compliance_uL, {})
        self.assertIsNone(p.backlash_comp_enabled)

    def test_dict_round_trip(self):
        p = DeviceProfile(pump_compliance_uL={"P1": 0.5, "P2": 0.3},
                          backlash_comp_enabled=True)
        out = DeviceProfile.from_dict(p.to_dict())
        self.assertEqual(out.pump_compliance_uL, {"P1": 0.5, "P2": 0.3})
        self.assertTrue(out.backlash_comp_enabled)

    def test_settings_bridge_round_trip(self):
        s = _FakeSettings()
        DeviceProfile(pump_compliance_uL={"P3": 0.7},
                      backlash_comp_enabled=True).apply_to_settings(s)
        self.assertEqual(s.get("device_profile.pump_compliance_uL"), {"P3": 0.7})
        self.assertTrue(s.get("device_profile.backlash_comp_enabled"))
        back = DeviceProfile.from_settings(s)
        self.assertEqual(back.pump_compliance_uL, {"P3": 0.7})
        self.assertTrue(back.backlash_comp_enabled)


# ════════════════════════════════════════════════════════════════════
#  StageController relief state + toggle
# ════════════════════════════════════════════════════════════════════

class TestReliefState(unittest.TestCase):

    def test_set_get_all(self):
        c = _ctrl()
        c.set_pump_relief_uL("P1", 0.4)
        c.set_pump_relief_uL("P2", 0.9)
        self.assertAlmostEqual(c.pump_relief_uL("P1"), 0.4)
        self.assertAlmostEqual(c.pump_relief_uL("P2"), 0.9)
        self.assertEqual(c.get_pump_relief_all(), {"P1": 0.4, "P2": 0.9})

    def test_get_defaults_zero_when_unset(self):
        c = _ctrl()
        self.assertEqual(c.pump_relief_uL("P3"), 0.0)

    def test_set_clamps_negative(self):
        c = _ctrl()
        c.set_pump_relief_uL("P1", -2.0)
        self.assertEqual(c.pump_relief_uL("P1"), 0.0)

    def test_apply_pump_relief_restores(self):
        c = _ctrl()
        c.apply_pump_relief({"P1": 0.5, "P2": 0.25, "BOGUS": 9.0})
        self.assertAlmostEqual(c.pump_relief_uL("P1"), 0.5)
        self.assertAlmostEqual(c.pump_relief_uL("P2"), 0.25)
        # Unknown pump ids are ignored.
        self.assertEqual(c.get_pump_relief_all().get("BOGUS"), None)

    def test_toggle(self):
        c = _ctrl()
        self.assertFalse(c.backlash_comp_enabled())
        c.set_backlash_comp_enabled(True)
        self.assertTrue(c.backlash_comp_enabled())


# ════════════════════════════════════════════════════════════════════
#  move_pump_uL backlash compensation engine
# ════════════════════════════════════════════════════════════════════

class TestCompEngine(unittest.TestCase):

    def test_dispense_bracket(self):
        c = _ctrl(relief={"P1": 0.5}, backlash=True)
        _run(c, +2.0, settle=True)                 # compensate=None → auto (on)
        self.assertEqual(len(c.pump_moves), 3)
        self.assertAlmostEqual(c.pump_moves[0][1], +0.5 * 0.3)  # take-up
        self.assertAlmostEqual(c.pump_moves[1][1], +2.0 * 0.3)  # fluid
        self.assertAlmostEqual(c.pump_moves[2][1], -0.5 * 0.3)  # unload
        self.assertAlmostEqual(sum(m[1] for m in c.pump_moves), 2.0 * 0.3)

    def test_aspirate_bracket(self):
        c = _ctrl(relief={"P1": 0.5}, backlash=True)
        _run(c, -2.0, settle=True)
        self.assertEqual(len(c.pump_moves), 3)
        self.assertAlmostEqual(c.pump_moves[0][1], -0.5 * 0.3)
        self.assertAlmostEqual(c.pump_moves[2][1], +0.5 * 0.3)

    def test_toggle_off_no_comp(self):
        c = _ctrl(relief={"P1": 0.5}, backlash=False)
        _run(c, -2.0, settle=True)
        self.assertEqual(len(c.pump_moves), 1)

    def test_zero_relief_no_comp(self):
        c = _ctrl(relief={"P1": 0.0}, backlash=True)
        _run(c, -2.0, settle=True)
        self.assertEqual(len(c.pump_moves), 1)

    def test_compensate_false_exempts_capture(self):
        c = _ctrl(relief={"P1": 0.5}, backlash=True)
        _run(c, -2.0, settle=True, compensate=False)   # balanced capture
        self.assertEqual(len(c.pump_moves), 1)

    def test_settle_false_never_auto_comps(self):
        c = _ctrl(relief={"P1": 0.5}, backlash=True)
        _run(c, -2.0, settle=False)                    # streamed path
        self.assertEqual(len(c.pump_moves), 1)

    def test_compensate_true_forces_on(self):
        c = _ctrl(relief={"P1": 0.5}, backlash=True)
        _run(c, +2.0, settle=False, compensate=True)   # jog click
        self.assertEqual(len(c.pump_moves), 3)


# ════════════════════════════════════════════════════════════════════
#  HardwareConfig legacy relief keys ignored
# ════════════════════════════════════════════════════════════════════

class TestHardwareConfigLegacy(unittest.TestCase):

    def test_legacy_relief_keys_discarded(self):
        out = HardwareConfig.from_dict({
            "pump_relief_percent": 3.0,
            "pump_relief_volume_uL": 0.8,
            "pump_relief_on_pickup": False,
            "pump_relief_on_deposit": False,
            "pump_relief_on_quick_move": False,
        })
        for attr in ("pump_relief_percent", "pump_relief_volume_uL",
                     "pump_relief_on_pickup", "pump_relief_on_deposit",
                     "pump_relief_on_quick_move"):
            self.assertFalse(hasattr(out, attr), attr)

    def test_to_dict_has_no_relief_keys(self):
        d = HardwareConfig().to_dict()
        self.assertNotIn("pump_relief_percent", d)
        self.assertNotIn("pump_relief_on_pickup", d)


# ════════════════════════════════════════════════════════════════════
#  Needle Location compliance-calibration card (offscreen)
# ════════════════════════════════════════════════════════════════════

class TestComplianceCalCard(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _page(self, ctrl):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage(ctrl, settings=_FakeSettings())
        return page

    def _cal_ctrl(self):
        c = _ctrl(relief={})
        # calibration card reads these; provide harmless stubs.
        c.is_pump_plunger_calibrated = lambda p: False
        return c

    def test_card_builds_with_widgets(self):
        page = self._page(self._cal_ctrl())
        self.assertTrue(hasattr(page, "_compcal_pump"))
        self.assertTrue(hasattr(page, "_compcal_manual"))
        self.assertTrue(hasattr(page, "_compcal_btn_save"))
        self.assertTrue(hasattr(page, "_compcal_btn_start"))
        self.assertEqual(page._compcal_phase, "prep")

    def test_start_zeroes_counters(self):
        page = self._page(self._cal_ctrl())
        page._compcal_dispensed_uL = 1.5
        page._compcal_aspirated_uL = 0.9
        page._compcal_start()
        self.assertEqual(page._compcal_dispensed_uL, 0.0)
        self.assertEqual(page._compcal_aspirated_uL, 0.0)
        self.assertEqual(page._compcal_phase, "measure")

    def test_measure_phase_tracks_net_A(self):
        page = self._page(self._cal_ctrl())
        page._compcal_start()                      # phase → measure, A = 0
        # An aspirate step raises A …
        page._compcal_pending_dir, page._compcal_pending_uL = -1.0, 0.3
        page._compcal_on_step_done(True, "P1")
        self.assertAlmostEqual(page._compcal_aspirated_uL, 0.3)
        page._compcal_pending_dir, page._compcal_pending_uL = -1.0, 0.3
        page._compcal_on_step_done(True, "P1")
        self.assertAlmostEqual(page._compcal_aspirated_uL, 0.6)
        # … and a dispense step lowers it (net), never below zero.
        page._compcal_pending_dir, page._compcal_pending_uL = +1.0, 1.0
        page._compcal_on_step_done(True, "P1")
        self.assertEqual(page._compcal_aspirated_uL, 0.0)

    def test_prep_phase_does_not_feed_A(self):
        page = self._page(self._cal_ctrl())
        self.assertEqual(page._compcal_phase, "prep")
        page._compcal_pending_dir, page._compcal_pending_uL = -1.0, 0.5
        page._compcal_on_step_done(True, "P1")     # aspirate during prep
        self.assertEqual(page._compcal_aspirated_uL, 0.0)  # not counted

    def test_finish_computes_half_of_A(self):
        page = self._page(self._cal_ctrl())
        page._compcal_start()
        page._compcal_aspirated_uL = 0.8
        page._compcal_compute()
        self.assertAlmostEqual(page._compcal_manual.value(), 0.4)
        self.assertEqual(page._compcal_phase, "done")

    def test_finish_before_start_is_noop(self):
        page = self._page(self._cal_ctrl())
        page._compcal_aspirated_uL = 0.8           # but still in prep
        page._compcal_compute()
        self.assertEqual(page._compcal_manual.value(), 0.0)
        self.assertEqual(page._compcal_phase, "prep")

    def test_button_gating_per_phase(self):
        page = self._page(self._cal_ctrl())

        def en(name):
            return getattr(page, name).isEnabled()

        # prep: dispense on, aspirate OFF, start on, finish off, save off (c=0).
        self.assertEqual(page._compcal_phase, "prep")
        self.assertTrue(en("_compcal_btn_dispense"))
        self.assertFalse(en("_compcal_btn_aspirate"))
        self.assertTrue(en("_compcal_btn_start"))
        self.assertFalse(en("_compcal_btn_compute"))
        self.assertFalse(en("_compcal_btn_save"))
        self.assertTrue(en("_compcal_pump"))
        # measure: aspirate on, start off, finish on, pump locked.
        page._compcal_start()
        self.assertTrue(en("_compcal_btn_dispense"))
        self.assertTrue(en("_compcal_btn_aspirate"))
        self.assertFalse(en("_compcal_btn_start"))
        self.assertTrue(en("_compcal_btn_compute"))
        self.assertFalse(en("_compcal_pump"))
        # done: finish computed → save on, dispense/aspirate/finish off.
        page._compcal_aspirated_uL = 0.6
        page._compcal_compute()
        self.assertEqual(page._compcal_phase, "done")
        self.assertTrue(en("_compcal_btn_save"))
        self.assertFalse(en("_compcal_btn_compute"))
        self.assertFalse(en("_compcal_btn_aspirate"))

    def test_manual_override_enables_save_in_prep(self):
        # The manual-override path: type a value on a fresh page and Save lights
        # up even without running the guided flow; a bare 0 keeps it disabled.
        page = self._page(self._cal_ctrl())
        self.assertFalse(page._compcal_btn_save.isEnabled())
        page._compcal_manual.setValue(0.4)
        self.assertTrue(page._compcal_btn_save.isEnabled())
        page._compcal_manual.setValue(0.0)
        self.assertFalse(page._compcal_btn_save.isEnabled())

    def test_reset_returns_to_prep(self):
        page = self._page(self._cal_ctrl())
        page._compcal_start()
        page._compcal_aspirated_uL = 0.8
        page._compcal_compute()
        page._compcal_reset()
        self.assertEqual(page._compcal_phase, "prep")
        self.assertEqual(page._compcal_aspirated_uL, 0.0)
        self.assertEqual(page._compcal_manual.value(), 0.0)

    def test_save_writes_controller_and_persists(self):
        ctrl = self._cal_ctrl()
        page = self._page(ctrl)
        page._compcal_pump.setCurrentText("P2")
        page._compcal_manual.setValue(0.37)
        page._compcal_save()
        self.assertAlmostEqual(ctrl.pump_relief_uL("P2"), 0.37)
        # Persisted to the device profile settings key.
        self.assertEqual(
            page.settings.get("device_profile.pump_compliance_uL").get("P2"),
            0.37)


# ════════════════════════════════════════════════════════════════════
#  Calibration workflow tabs restructured (own Pump Compliance tab;
#  Needle Offset merged into Needle Location)
# ════════════════════════════════════════════════════════════════════

class TestCalibrationTabRestructure(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        c = _ctrl(relief={})
        c.is_pump_plunger_calibrated = lambda p: False
        return CalibrationPage(c, settings=_FakeSettings())

    def test_tabs_restructured(self):
        page = self._page()
        tabs = page._workflow_tabs
        titles = [tabs.tabText(i) for i in range(tabs.count())]
        self.assertEqual(
            titles,
            ["Needle Location", "Pump Compliance", "Plate Location",
             "Plate Z Auto-Cal", "Custom"])
        self.assertNotIn("Needle Offset Calibration", titles)
        self.assertEqual(page._compcal_tab_index, 1)
        self.assertEqual(page._zauto_tab_index, 3)
        self.assertFalse(hasattr(page, "_zoff_tab_index"))

    def test_needle_offset_merged_into_needle_location(self):
        # The reference-Z widgets are built by the (now merged) Needle Location
        # tab — they exist on the page even though there is no Offset tab.
        page = self._page()
        self.assertTrue(hasattr(page, "_zoff_xz_view"))
        self.assertTrue(hasattr(page, "_zoff_btn_estimate"))
        self.assertTrue(hasattr(page, "_zoff_lbl_plate_bottom_z"))

    def test_pump_compliance_tab_has_camera_slots(self):
        page = self._page()
        self.assertTrue(hasattr(page, "_compcal_cam_slots"))
        self.assertEqual(len(page._compcal_cam_slots), 2)


if __name__ == "__main__":
    unittest.main()
