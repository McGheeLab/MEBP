"""
v7.5.x — per-pump plunger zero/max calibration (mirror of the Z Axis Setup).

ZERO = plunger all the way IN (syringe empty) → "Set Dispensed" → fill 0.
MAX  = plunger all the way OUT (syringe full)  → "Set Aspirated".
``apply_pump_setup`` captures both extremes, DERIVES the dispense/aspirate
direction (which it then OWNS, like z_up_sign owns Z), and sets the soft limits.
ASPIRATE = draw fluid IN (toward full, raises fill); DISPENSE = push fluid OUT
(toward empty, lowers fill).

Backend tests run on a minimal controller built with __new__ (no hardware), the
same pattern as test_v75x_z_axis_unified_setup. A final offscreen smoke builds
the real Hardware Setup page and exercises the capture handlers.
"""

import os
import sys
import unittest

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits


# ── A fake syringe / pump config so capacity + fill convert to µL ──────────

class _Syr:
    def __init__(self, volume_uL=250, stroke_mm=35.0):
        self.volume_uL = volume_uL
        self.stroke_length_mm = stroke_mm

    def mm_to_uL(self, mm):
        return mm * (self.volume_uL / self.stroke_length_mm)

    def uL_to_mm(self, uL):
        return uL * (self.stroke_length_mm / self.volume_uL)


class _PumpCfg:
    def __init__(self, syr=None):
        self.is_configured = syr is not None
        self.syringe = syr

    def mm_to_uL(self, mm):
        return self.syringe.mm_to_uL(mm)

    def uL_to_mm(self, uL):
        return self.syringe.uL_to_mm(uL)


class _HW:
    def __init__(self, pumps):
        self.pumps = pumps


def _make_controller(with_syringe=True):
    c = StageController.__new__(StageController)
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0,
                       "P1": 0.0, "P2": 0.0, "P3": 0.0}
    c.safety_limits = SafetyLimits()
    c._axis_flip = {"Z": False, "P1": False, "P2": False, "P3": False}
    c._pump_aspirate_sign = {"P1": 1.0, "P2": 1.0, "P3": 1.0}
    c._pump_setup = {}
    if with_syringe:
        c._hardware_config = _HW({
            "P1": _PumpCfg(_Syr()), "P2": _PumpCfg(_Syr()),
            "P3": _PumpCfg(_Syr()),
        })
    else:
        c._hardware_config = None
    return c


# ── ME3B-ish polarity: plunger empty (all-in) at the LARGER raw ────────────

class TestApplyPumpSetupME3B(unittest.TestCase):
    def test_empty_zero_full_derives_negative_sign(self):
        c = _make_controller()
        # Dispensed (empty / all-in) at raw 40; aspirated (full / all-out) at 5.
        s = c.apply_pump_setup("P1", raw_dispensed_mm=40.0, raw_aspirated_mm=5.0)
        self.assertEqual(c.zero_position["P1"], 40.0)          # datum = empty
        self.assertEqual(c.pump_aspirate_sign("P1"), -1.0)     # full = smaller raw
        self.assertEqual(c.safety_limits.p1_min, 5.0)          # raw envelope
        self.assertEqual(c.safety_limits.p1_max, 40.0)
        self.assertTrue(s["direction_ok"])
        self.assertTrue(c.is_pump_plunger_calibrated("P1"))

    def test_dispense_direction_is_opposite_aspirate(self):
        c = _make_controller()
        c.apply_pump_setup("P1", 40.0, 5.0)   # aspirate_sign = -1
        # Dispense pushes toward empty = +raw on this machine.
        self.assertEqual(c.pump_dir_sign("P1"), 1.0)

    def test_fill_zero_at_empty_capacity_at_full(self):
        c = _make_controller()
        c.apply_pump_setup("P1", 40.0, 5.0)
        cap = c.pump_capacity_uL("P1")
        # 35 mm stroke, 250 µL / 35 mm syringe → 250 µL.
        self.assertAlmostEqual(cap, 250.0)
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P1", 40.0), 0.0)   # empty
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P1", 5.0), cap)    # full
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P1", 22.5), cap / 2)
        # Monotonic: less raw (toward full) = more fill on this polarity.
        self.assertGreater(c.raw_to_pump_fill_uL("P1", 10.0),
                           c.raw_to_pump_fill_uL("P1", 30.0))


class TestApplyPumpSetupConventional(unittest.TestCase):
    """Conventional: full (all-out) at the LARGER raw."""

    def test_derives_positive_sign(self):
        c = _make_controller()
        s = c.apply_pump_setup("P2", raw_dispensed_mm=0.0, raw_aspirated_mm=30.0)
        self.assertEqual(c.zero_position["P2"], 0.0)
        self.assertEqual(c.pump_aspirate_sign("P2"), 1.0)
        self.assertEqual(c.pump_dir_sign("P2"), -1.0)   # dispense = -raw
        self.assertEqual(c.safety_limits.p2_min, 0.0)
        self.assertEqual(c.safety_limits.p2_max, 30.0)
        self.assertTrue(s["direction_ok"])
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P2", 0.0), 0.0)
        self.assertGreater(c.raw_to_pump_fill_uL("P2", 30.0), 0.0)


class TestOffsetDatum(unittest.TestCase):
    def test_offset_empty_datum(self):
        c = _make_controller()
        c.apply_pump_setup("P1", raw_dispensed_mm=12.0, raw_aspirated_mm=-18.0)
        self.assertEqual(c.zero_position["P1"], 12.0)
        self.assertEqual(c.pump_aspirate_sign("P1"), -1.0)
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P1", 12.0), 0.0)
        cap = c.pump_capacity_uL("P1")
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P1", -18.0), cap)


class TestDirectionGuard(unittest.TestCase):
    def test_too_close_does_not_flip_sign(self):
        c = _make_controller()
        c.set_pump_aspirate_sign("P1", -1.0)
        s = c.apply_pump_setup("P1", raw_dispensed_mm=5.0, raw_aspirated_mm=5.2,
                               min_travel_mm=1.0)
        self.assertFalse(s["direction_ok"])
        self.assertEqual(c.pump_aspirate_sign("P1"), -1.0)   # unchanged
        # Datum + limits still recorded so the operator can re-run.
        self.assertEqual(c.zero_position["P1"], 5.0)
        self.assertEqual(c.safety_limits.p1_min, 5.0)
        self.assertAlmostEqual(c.safety_limits.p1_max, 5.2)


class TestCapacityGraceful(unittest.TestCase):
    def test_no_syringe_capacity_is_none(self):
        c = _make_controller(with_syringe=False)
        s = c.apply_pump_setup("P1", 40.0, 5.0)
        self.assertIsNone(s["capacity_uL"])
        self.assertIsNone(c.pump_capacity_uL("P1"))
        self.assertIsNone(c.raw_to_pump_fill_uL("P1", 22.5))
        # Datum / direction / limits still established.
        self.assertEqual(c.pump_aspirate_sign("P1"), -1.0)
        self.assertTrue(c.is_pump_plunger_calibrated("P1"))


class TestUncalibratedFallback(unittest.TestCase):
    def test_uncalibrated_pump_uses_flip_sign(self):
        c = _make_controller()
        # Not calibrated → pump_dir_sign falls back to _flip_sign (no flip → +1).
        self.assertFalse(c.is_pump_plunger_calibrated("P3"))
        self.assertEqual(c.pump_dir_sign("P3"), 1.0)
        c._axis_flip["P3"] = True
        self.assertEqual(c.pump_dir_sign("P3"), -1.0)
        self.assertIsNone(c.raw_to_pump_fill_uL("P3", 10.0))   # no fill readout


class TestDirectionOwnershipInMotion(unittest.TestCase):
    """move_pump_relative applies pump_dir_sign so a calibrated pump's dispense
    intent maps to the correct raw direction (owned by the calibration)."""

    class _FakeZP:
        axis_map = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}

        def __init__(self):
            self.last = None

        def move_relative(self, axes, feedrate=None):
            self.last = dict(axes)

    def _ctrl(self):
        c = _make_controller()
        c.zp_stage = self._FakeZP()
        return c

    def test_calibrated_dispense_intent_maps_to_minus_aspirate_sign(self):
        c = self._ctrl()
        c.apply_pump_setup("P1", raw_dispensed_mm=40.0, raw_aspirated_mm=5.0)
        # aspirate_sign = -1 → dispense dir = +1. A +1 mm dispense-intent move
        # → +1 mm raw on P1's physical letter (X).
        c.move_pump_relative("P1", +1.0, bypass_safety=True)
        self.assertAlmostEqual(c.zp_stage.last["X"], +1.0)
        # An aspirate (-1 mm intent) → toward full = -1 raw.
        c.move_pump_relative("P1", -1.0, bypass_safety=True)
        self.assertAlmostEqual(c.zp_stage.last["X"], -1.0)

    def test_conventional_polarity_inverts_raw(self):
        c = self._ctrl()
        c.apply_pump_setup("P2", raw_dispensed_mm=0.0, raw_aspirated_mm=30.0)
        # aspirate_sign = +1 → dispense dir = -1.
        c.move_pump_relative("P2", +1.0, bypass_safety=True)
        self.assertAlmostEqual(c.zp_stage.last["Y"], -1.0)

    def test_uncalibrated_unchanged(self):
        c = self._ctrl()
        # No setup for P3 → uses _flip_sign (no flip) → identity.
        c.move_pump_relative("P3", +2.5, bypass_safety=True)
        self.assertAlmostEqual(c.zp_stage.last["E"], +2.5)


class TestApplyPumpConvention(unittest.TestCase):
    def test_restore_sign_and_datum(self):
        c = _make_controller()
        c.apply_pump_convention({
            "P1": {"raw_dispensed": 40.0, "raw_aspirated": 5.0,
                   "aspirate_sign": -1.0, "capacity_uL": 250.0},
        })
        self.assertEqual(c.zero_position["P1"], 40.0)
        self.assertEqual(c.pump_aspirate_sign("P1"), -1.0)
        self.assertTrue(c.is_pump_plunger_calibrated("P1"))
        self.assertEqual(c.pump_dir_sign("P1"), 1.0)
        # v7.5.x: the soft-limit envelope is RE-DERIVED from the extremes
        # (min/max of the captured raw extremes), not left at the default.
        self.assertEqual(c.safety_limits.p1_min, 5.0)
        self.assertEqual(c.safety_limits.p1_max, 40.0)

    def test_rederive_sign_when_absent(self):
        c = _make_controller()
        c.apply_pump_convention({
            "P2": {"raw_dispensed": 0.0, "raw_aspirated": 30.0},
        })
        self.assertEqual(c.pump_aspirate_sign("P2"), 1.0)

    def test_envelope_restored_overrides_stale_mirror(self):
        """The reported bug: location restores correctly but min/max don't.

        A stale ``safety_limits`` mirror (here the sign-flipped [0, 30] seen on
        ME3B V3) must be CORRECTED by apply_pump_convention to the envelope
        implied by the authoritative captured extremes (raw 0 → -30 ⇒ [-30, 0]),
        not preserved. Mirrors the real settings.json/device-profile state."""
        c = _make_controller()
        # Pre-load the WRONG persisted mirror (what main.py loads from
        # settings.json before apply_pump_convention runs).
        c.safety_limits.p2_min = 0.0
        c.safety_limits.p2_max = 30.0
        c.apply_pump_convention({
            "P2": {"raw_dispensed": 0.0, "raw_aspirated": -30.0,
                   "aspirate_sign": -1.0, "capacity_uL": 250.0},
        })
        # Datum (location) was already right; the envelope is now corrected.
        self.assertEqual(c.zero_position["P2"], 0.0)
        self.assertEqual(c.safety_limits.p2_min, -30.0)
        self.assertEqual(c.safety_limits.p2_max, 0.0)
        # And it matches a fresh live calibration of the same extremes.
        ref = _make_controller()
        ref.apply_pump_setup("P2", 0.0, -30.0)
        self.assertEqual(c.safety_limits.p2_min, ref.safety_limits.p2_min)
        self.assertEqual(c.safety_limits.p2_max, ref.safety_limits.p2_max)

    def test_empty_is_noop(self):
        c = _make_controller()
        c.apply_pump_convention(None)
        c.apply_pump_convention({})
        self.assertFalse(c.is_pump_plunger_calibrated("P1"))


class TestDeviceProfileRoundTrip(unittest.TestCase):
    def test_pump_setup_round_trips(self):
        from gui.pages.hardware.device_profile import DeviceProfile
        setup = {"P1": {"raw_dispensed": 40.0, "raw_aspirated": 5.0,
                        "aspirate_sign": -1.0, "capacity_uL": 250.0}}
        d = DeviceProfile(profile_name="t", pump_setup=setup)
        rt = DeviceProfile.from_dict(d.to_dict())
        self.assertEqual(rt.pump_setup, setup)


class TestClobberGuard(unittest.TestCase):
    """update_from_hardware_config must not overwrite a calibrated pump's
    soft-limit envelope with the coarse syringe-stroke estimate."""

    def test_skip_pumps_preserves_envelope(self):
        sl = SafetyLimits(p1_min=5.0, p1_max=40.0)

        class _Cfg:
            is_configured = True
            syringe = _Syr()

        class _HWg:
            needle = None
            pumps = {"P1": _Cfg()}

        sl.update_from_hardware_config(_HWg(), skip_pumps={"P1"})
        self.assertEqual(sl.p1_min, 5.0)
        self.assertEqual(sl.p1_max, 40.0)
        # Without skip, the stroke estimate overwrites it.
        sl2 = SafetyLimits(p1_min=5.0, p1_max=40.0)
        sl2.update_from_hardware_config(_HWg())
        self.assertNotEqual(sl2.p1_max, 40.0)


class TestBeginPumpPlungerSetup(unittest.TestCase):
    """The 'Set Dispensed' (set-zero) step zeroes the firmware counter at the
    empty extreme so the datum is raw 0.0 and the full extreme reads a positive
    fill, then widens the soft limits for the jog out."""

    class _FakeZP:
        def __init__(self):
            self.zeroed = []

        def set_zero(self, logical):
            self.zeroed.append(logical)
            return True

    def _ctrl(self, empty_raw=70.0):
        c = _make_controller()
        c.zp_stage = self._FakeZP()
        c.capture_current_pump_raw = lambda pump, _v=empty_raw: _v
        return c

    def test_zeroes_datum_and_widens_limits(self):
        c = self._ctrl(empty_raw=70.0)
        r = c.begin_pump_plunger_setup("P2")
        self.assertTrue(r["ok"])
        self.assertEqual(r["previous_raw"], 70.0)
        self.assertEqual(c.zero_position["P2"], 0.0)           # datum zeroed
        self.assertIn("P2", c.zp_stage.zeroed)                 # G92 sent
        # Widened symmetric to the syringe stroke (35 mm × 1.2 = 42).
        self.assertAlmostEqual(c.safety_limits.p2_min, -42.0)
        self.assertAlmostEqual(c.safety_limits.p2_max, 42.0)
        # Only the datum is set so far — not yet a complete calibration.
        self.assertFalse(c.is_pump_plunger_calibrated("P2"))

    def test_begin_then_apply_gives_zero_empty_positive_full(self):
        c = self._ctrl(empty_raw=70.0)
        c.begin_pump_plunger_setup("P2")              # empty zeroed → raw 0
        # ME3B polarity: motor counts DOWN to full → full reads -30 after zero.
        s = c.apply_pump_setup("P2", 0.0, -30.0)
        self.assertEqual(c.zero_position["P2"], 0.0)          # empty = 0.0
        self.assertEqual(c.pump_aspirate_sign("P2"), -1.0)
        self.assertTrue(s["direction_ok"])
        # Fill: 0 at empty, POSITIVE toward full — "fully extended is positive".
        self.assertAlmostEqual(c.raw_to_pump_fill_uL("P2", 0.0), 0.0)
        self.assertGreater(c.raw_to_pump_fill_uL("P2", -30.0), 0.0)
        # Final tight envelope replaces the widened span.
        self.assertEqual(c.safety_limits.p2_min, -30.0)
        self.assertEqual(c.safety_limits.p2_max, 0.0)

    def test_no_zp_stage_reports_not_ok_but_sets_datum(self):
        c = _make_controller()       # no zp_stage attribute
        r = c.begin_pump_plunger_setup("P1")
        self.assertFalse(r["ok"])
        self.assertEqual(c.zero_position["P1"], 0.0)

    def test_invalid_pump_raises(self):
        c = self._ctrl()
        with self.assertRaises(ValueError):
            c.begin_pump_plunger_setup("P9")


# ── Offscreen GUI smoke: the per-pump Plunger Setup blocks + handlers ──────
#
# v7.5.x: the plunger setup moved off the Pump sub-page onto the Stage sub-page
# (StageHardwarePanel), directly beneath the Z Axis Setup block (order Z, P1,
# P2, P3) — one block per pump (no pump combo).

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class _FakeSettings:
    def __init__(self):
        self.data = {}

    def set(self, key, value):
        self.data[key] = value

    def set_section(self, key, value):
        self.data[key] = dict(value)

    def get(self, key, default=None):
        return self.data.get(key, default)

    def get_section(self, key):
        return self.data.get(key)

    def save(self):
        self.data["__saved__"] = True


class TestPumpPlungerSetupPanel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _panel_with_ctrl(self):
        from gui.pages.hardware.stage_panel import StageHardwarePanel
        panel = StageHardwarePanel()
        ctrl = _make_controller()
        # Drive captures from a settable "current raw" per pump.
        ctrl._fake_raw = {"P1": 40.0}

        def _capture(pump):
            return ctrl._fake_raw.get(pump)
        ctrl.capture_current_pump_raw = _capture

        # Set Dispensed zeroes the firmware counter (G92) — model a ZP stage
        # whose set_zero succeeds so begin_pump_plunger_setup reports ok.
        class _FakeZP:
            steps_per_mm = {}          # read by _refresh_steps_grid on load

            def set_zero(self, logical):
                return True
        ctrl.zp_stage = _FakeZP()
        panel._controller = ctrl
        panel._settings = _FakeSettings()
        return panel, ctrl

    def test_blocks_exist_one_per_pump(self):
        panel, _ = self._panel_with_ctrl()
        # One block per pump (no combo); ordered P1, P2, P3 under the Z block.
        for pid in ("P1", "P2", "P3"):
            self.assertIn(pid, panel._pump_setup_dispensed_btns)
            self.assertIn(pid, panel._pump_setup_aspirated_btns)
            self.assertIn(pid, panel._pump_setup_status_lbls)
            self.assertFalse(panel._pump_setup_aspirated_btns[pid].isEnabled())

    def test_capture_flow_calibrates_and_persists(self):
        from gui.pages.hardware import stage_panel as sp

        panel, ctrl = self._panel_with_ctrl()
        # Set Dispensed at the empty extreme ZEROES the pump → datum raw 0.0.
        ctrl._fake_raw["P1"] = 40.0
        panel._pump_setup_capture_dispensed("P1")
        self.assertTrue(panel._pump_setup_aspirated_btns["P1"].isEnabled())
        self.assertEqual(ctrl.zero_position["P1"], 0.0)         # zeroed at empty
        self.assertEqual(panel._pump_setup_dispensed_raw["P1"], 0.0)
        # After zeroing at empty (was raw 40), the full extreme reads in the
        # re-zeroed frame: 5 − 40 = −35. Confirm (auto-Yes the dialog).
        ctrl._fake_raw["P1"] = -35.0
        orig = sp.QMessageBox.question
        sp.QMessageBox.question = staticmethod(
            lambda *a, **k: sp.QMessageBox.Yes)
        try:
            panel._pump_setup_capture_aspirated("P1")
        finally:
            sp.QMessageBox.question = orig
        # The pump is now calibrated; empty = 0, full = negative raw → −1 sign.
        self.assertTrue(ctrl.is_pump_plunger_calibrated("P1"))
        self.assertEqual(ctrl.pump_aspirate_sign("P1"), -1.0)
        # Empty datum is 0.0; the FILL reads 0 at empty, POSITIVE at full.
        self.assertEqual(ctrl.zero_position["P1"], 0.0)
        self.assertAlmostEqual(ctrl.raw_to_pump_fill_uL("P1", 0.0), 0.0)
        self.assertGreater(ctrl.raw_to_pump_fill_uL("P1", -35.0), 0.0)
        # Persistence wrote zero_position + extents + device_profile.pump_setup.
        s = panel._settings
        self.assertIn("zero_position", s.data)
        self.assertEqual(s.get("safety_limits.p1_min"), -35.0)
        self.assertEqual(s.get("safety_limits.p1_max"), 0.0)
        self.assertIn("P1", s.get("device_profile.pump_setup"))
        self.assertIn("✅", panel._pump_setup_status_lbls["P1"].text())
        # v7.5.x: the visible limit spinboxes are the user FILL frame (0 =
        # empty → +capacity = full, always positive), like Z — NOT the raw
        # [-35, 0] motor frame that's persisted for the clamp. So after
        # calibration they read [0, 35].
        self.assertAlmostEqual(panel.spin_p_mins["P1"].value(), 0.0, places=3)
        self.assertAlmostEqual(panel.spin_p_maxs["P1"].value(), 35.0, places=3)

    def test_limit_spinboxes_use_fill_frame_but_persist_raw(self):
        """v7.5.x: a calibrated pump's limit spinboxes show the POSITIVE fill
        frame [0, capacity] (like Z), while settings + the live clamp keep the
        raw Marlin envelope. ME3B P2: raw [-30, 0] ⇄ fill [0, 30]. This is the
        operator report — 'loaded min=-30, max=0; should be 0 and 30'."""
        panel, ctrl = self._panel_with_ctrl()
        # Calibrate P2 the ME3B way: empty=raw0, full=raw-30 (motor counts down).
        ctrl.apply_pump_convention({
            "P2": {"raw_dispensed": 0.0, "raw_aspirated": -30.0,
                   "aspirate_sign": -1.0, "capacity_uL": 250.0},
        })
        self.assertTrue(ctrl.is_pump_plunger_calibrated("P2"))
        # Stored/clamp envelope is raw [-30, 0].
        s = panel._settings
        s.set("safety_limits.p2_min", -30.0)
        s.set("safety_limits.p2_max", 0.0)
        # LOAD → spinboxes show the positive fill frame [0, 30] (raw min/max
        # swapped by value because aspirate_sign is negative).
        panel._load_from_settings()
        self.assertAlmostEqual(panel.spin_p_mins["P2"].value(), 0.0, places=3)
        self.assertAlmostEqual(panel.spin_p_maxs["P2"].value(), 30.0, places=3)
        # SAVE → settings AND the live clamp go back to raw [-30, 0] (the
        # display never leaks into the frame the clamp compares in).
        panel._apply_safety_and_zero()
        self.assertAlmostEqual(s.get("safety_limits.p2_min"), -30.0, places=3)
        self.assertAlmostEqual(s.get("safety_limits.p2_max"), 0.0, places=3)
        self.assertAlmostEqual(ctrl.safety_limits.p2_min, -30.0, places=3)
        self.assertAlmostEqual(ctrl.safety_limits.p2_max, 0.0, places=3)

    def test_uncalibrated_pump_limits_shown_and_stored_raw(self):
        """An UNcalibrated pump has no fill frame → spinboxes show the raw
        envelope unchanged (identity conversion), and it round-trips raw."""
        panel, ctrl = self._panel_with_ctrl()  # only P1 has _fake_raw; none calibrated
        self.assertFalse(ctrl.is_pump_plunger_calibrated("P3"))
        s = panel._settings
        s.set("safety_limits.p3_min", 0.0)
        s.set("safety_limits.p3_max", 34.0)
        panel._load_from_settings()
        self.assertAlmostEqual(panel.spin_p_mins["P3"].value(), 0.0, places=3)
        self.assertAlmostEqual(panel.spin_p_maxs["P3"].value(), 34.0, places=3)
        panel._apply_safety_and_zero()
        self.assertAlmostEqual(s.get("safety_limits.p3_min"), 0.0, places=3)
        self.assertAlmostEqual(s.get("safety_limits.p3_max"), 34.0, places=3)

    def test_capture_is_independent_per_pump(self):
        from gui.pages.hardware import stage_panel as sp

        panel, ctrl = self._panel_with_ctrl()
        # Capturing P2's dispensed extreme must not arm P1's aspirated button.
        ctrl._fake_raw["P2"] = 0.0
        panel._pump_setup_capture_dispensed("P2")
        self.assertTrue(panel._pump_setup_aspirated_btns["P2"].isEnabled())
        self.assertFalse(panel._pump_setup_aspirated_btns["P1"].isEnabled())
        ctrl._fake_raw["P2"] = 30.0
        orig = sp.QMessageBox.question
        sp.QMessageBox.question = staticmethod(
            lambda *a, **k: sp.QMessageBox.Yes)
        try:
            panel._pump_setup_capture_aspirated("P2")
        finally:
            sp.QMessageBox.question = orig
        self.assertTrue(ctrl.is_pump_plunger_calibrated("P2"))
        self.assertFalse(ctrl.is_pump_plunger_calibrated("P1"))
        self.assertEqual(panel._settings.get("safety_limits.p2_min"), 0.0)
        self.assertEqual(panel._settings.get("safety_limits.p2_max"), 30.0)

    def test_cancelled_confirm_does_not_calibrate(self):
        from gui.pages.hardware import stage_panel as sp

        panel, ctrl = self._panel_with_ctrl()
        ctrl._fake_raw["P1"] = 40.0
        panel._pump_setup_capture_dispensed("P1")
        ctrl._fake_raw["P1"] = 5.0
        orig = sp.QMessageBox.question
        sp.QMessageBox.question = staticmethod(
            lambda *a, **k: sp.QMessageBox.No)
        try:
            panel._pump_setup_capture_aspirated("P1")
        finally:
            sp.QMessageBox.question = orig
        self.assertFalse(ctrl.is_pump_plunger_calibrated("P1"))


if __name__ == "__main__":
    unittest.main()
