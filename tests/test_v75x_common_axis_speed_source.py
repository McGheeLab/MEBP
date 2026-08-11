"""v7.5.x tests — ONE common per-axis speed source + needle-derived pump flow
+ µL↔% pump helpers (MEBP_v75x_COMMON_AXIS_SPEED_SOURCE_AND_PUMP_UNITS).

Covered:
  Resolvers (StageController):
    1.  get_max_xy_speed_um_s prefers the live XY stage's resolved top speed.
    2.  …falls back to the timing store, then safety_limits.max_xy_speed.
    3.  get_max_z_feedrate_mm_min prefers per_axis['Z'] over max_z_feedrate.
    4.  get_max_pump_feedrate delegates to _pump_jog_max_native (needle flow).
    5.  notify_speed_limits_changed refreshes jog limits + fires the callback.
  Needle-derived flow ceiling (SafetyLimits):
    6.  update_from_hardware_config sets the per-pump ceiling ==
        max_safe_flow_rate_uL_s(needle, InkSpec(viscosity_cP=1.0)).
    7.  No needle / degenerate bore → existing ceiling left UNTOUCHED.
  Pump µL↔% helpers (StageController):
    8.  pump_effective_capacity_uL: calibrated stroke → else nominal syringe.
    9.  pump_pct_to_uL / pump_uL_to_pct round-trip (sign preserved).
    10. helpers return None when no capacity is resolvable.
    11. pump_fill_pct is None when the live fill is unreadable.
"""

import sys
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _wait_for(predicate, timeout_s=2.0):
    """v7.5.x: pump jog moves now run on a daemon thread (see
    ``HardwareControlPanel._on_jog_pump``) so the GUI thread — and every
    QTimer-driven camera feed — can't freeze while a backlash-compensated
    move drains via blocking M400s. Spin the Qt event loop (delivering the
    queued completion signal) until ``predicate()`` is true or timeout."""
    from PySide6.QtWidgets import QApplication
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    QApplication.processEvents()
    return predicate()

from SupportClasses.SafetyLimits import (
    SafetyLimits, REFERENCE_VISCOSITY_CP,
)
from SupportClasses.StageController import StageController
from SupportClasses.FlowPhysics import (
    max_safe_flow_rate_uL_s, DEFAULT_PRESSURE_LIMIT_PA,
)
from SupportClasses.PhysicalModels import InkSpec


# ── lightweight stubs ───────────────────────────────────────────────

class _FakeNeedle:
    """Only the fields the flow calc + SafetyLimits read."""
    gauge = 22
    id_m = 0.0004          # 0.40 mm bore
    length_mm = 25.4       # 1"


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
    def __init__(self, needle=None, pumps=None):
        self.needle = needle
        self.pumps = pumps or {}
        self.configured_pump_ids = [
            pid for pid, p in self.pumps.items()
            if getattr(p, "is_configured", False)]


def _bare_controller():
    c = StageController.__new__(StageController)
    c.safety_limits = SafetyLimits()
    c._pending_per_axis_max_feedrate = None
    c._hardware_config = None
    c._jog_speed_pct = {}
    c.xy_stage = None
    c.xy_jog = None
    c.zp_jog = None
    return c


# ── 1–2: XY resolver ────────────────────────────────────────────────

class TestXYResolver(unittest.TestCase):

    def test_safety_limit_is_the_single_source(self):
        # The editable safety value is primary — editing it propagates, and it
        # wins over any persisted measured value (measurement WRITES it).
        c = _bare_controller()
        c.safety_limits.max_xy_speed = 8000.0
        store = mock.Mock()
        store.get_xy_max_speed_um_s.return_value = 21000.0
        with mock.patch(
                "SupportClasses.PrintTimingCalibrationStore.get_store",
                return_value=store):
            self.assertAlmostEqual(c.get_max_xy_speed_um_s(), 8000.0)

    def test_store_fallback_only_when_safety_unset(self):
        c = _bare_controller()
        c.safety_limits.max_xy_speed = 0.0      # unset
        store = mock.Mock()
        store.get_xy_max_speed_um_s.return_value = 21000.0
        with mock.patch(
                "SupportClasses.PrintTimingCalibrationStore.get_store",
                return_value=store):
            self.assertAlmostEqual(c.get_max_xy_speed_um_s(), 21000.0)
        # Neither set → conservative constant.
        store.get_xy_max_speed_um_s.return_value = None
        with mock.patch(
                "SupportClasses.PrintTimingCalibrationStore.get_store",
                return_value=store):
            self.assertAlmostEqual(
                c.get_max_xy_speed_um_s(),
                StageController._XY_MAX_FALLBACK_UM_S)


# ── 3: Z resolver ───────────────────────────────────────────────────

class TestZResolver(unittest.TestCase):

    def test_per_axis_wins_over_safety(self):
        c = _bare_controller()
        c.safety_limits.max_z_feedrate = 500.0
        c._pending_per_axis_max_feedrate = {"Z": 1200.0}
        self.assertAlmostEqual(c.get_max_z_feedrate_mm_min(), 1200.0)

    def test_falls_back_to_safety(self):
        c = _bare_controller()
        c.safety_limits.max_z_feedrate = 640.0
        c._pending_per_axis_max_feedrate = {}
        self.assertAlmostEqual(c.get_max_z_feedrate_mm_min(), 640.0)


# ── 4–5: pump anchor + change notifier ──────────────────────────────

class TestPumpAnchorAndNotify(unittest.TestCase):

    def test_pump_anchor_delegates_to_native(self):
        c = _bare_controller()
        c._hardware_config = _FakeHW(pumps={"P1": _FakePump()})
        c.safety_limits.set_max_flow_rate("P1", 6.0)
        self.assertAlmostEqual(c.get_max_pump_feedrate(), 6.0)

    def test_notify_fires_callback(self):
        c = _bare_controller()
        fired = []
        c.on_speed_limits_changed = lambda: fired.append(True)
        c.notify_speed_limits_changed()           # no jog handlers → no-op there
        self.assertEqual(fired, [True])


# ── 6–7: needle-derived flow ceiling ────────────────────────────────

class TestNeedleDerivedFlowCeiling(unittest.TestCase):

    def test_ceiling_matches_flow_physics(self):
        needle = _FakeNeedle()
        hw = _FakeHW(needle=needle,
                     pumps={"P1": _FakePump(_FakeSyringe()),
                            "P2": _FakePump(_FakeSyringe())})
        sl = SafetyLimits()
        sl.update_from_hardware_config(hw)
        expected = max_safe_flow_rate_uL_s(
            needle, InkSpec(name="ref", viscosity_cP=REFERENCE_VISCOSITY_CP),
            DEFAULT_PRESSURE_LIMIT_PA)
        self.assertGreater(expected, 0.0)
        self.assertAlmostEqual(sl.get_max_flow_rate("P1"), expected, places=6)
        self.assertAlmostEqual(sl.get_max_flow_rate("P2"), expected, places=6)

    def test_no_needle_leaves_existing_untouched(self):
        hw = _FakeHW(needle=None, pumps={"P1": _FakePump(_FakeSyringe())})
        sl = SafetyLimits()
        sl.set_max_flow_rate("P1", 4.2)         # pre-existing ceiling
        sl.update_from_hardware_config(hw)
        self.assertAlmostEqual(sl.get_max_flow_rate("P1"), 4.2)

    def test_reference_viscosity_is_water(self):
        self.assertAlmostEqual(REFERENCE_VISCOSITY_CP, 1.0)


# ── 8–11: µL ↔ % pump helpers ───────────────────────────────────────

class TestPumpPercentHelpers(unittest.TestCase):

    def test_effective_capacity_prefers_calibrated(self):
        c = _bare_controller()
        c._pump_setup = {"P1": {"raw_dispensed": 0.0, "raw_aspirated": 30.0}}
        c.zero_position = {"P1": 0.0}
        c._hardware_config = _FakeHW(pumps={"P1": _FakePump(_FakeSyringe(250.0, 30.0))})
        # calibrated stroke 30 mm × (250/30) µL/mm = 250 µL
        self.assertAlmostEqual(c.pump_effective_capacity_uL("P1"), 250.0)

    def test_effective_capacity_falls_back_to_syringe(self):
        c = _bare_controller()
        c._pump_setup = {}                      # uncalibrated
        c._hardware_config = _FakeHW(pumps={"P1": _FakePump(_FakeSyringe(100.0))})
        self.assertAlmostEqual(c.pump_effective_capacity_uL("P1"), 100.0)

    def test_pct_uL_round_trip_signed(self):
        c = _bare_controller()
        c._pump_setup = {}
        c._hardware_config = _FakeHW(pumps={"P1": _FakePump(_FakeSyringe(200.0))})
        self.assertAlmostEqual(c.pump_pct_to_uL("P1", 10.0), 20.0)
        self.assertAlmostEqual(c.pump_pct_to_uL("P1", -25.0), -50.0)
        self.assertAlmostEqual(c.pump_uL_to_pct("P1", 20.0), 10.0)
        self.assertAlmostEqual(c.pump_uL_to_pct("P1", -50.0), -25.0)

    def test_none_when_no_capacity(self):
        c = _bare_controller()
        c._pump_setup = {}
        c._hardware_config = _FakeHW(pumps={"P1": _FakePump(syringe=None)})
        self.assertIsNone(c.pump_effective_capacity_uL("P1"))
        self.assertIsNone(c.pump_pct_to_uL("P1", 10.0))
        self.assertIsNone(c.pump_uL_to_pct("P1", 10.0))

    def test_fill_pct_none_when_unreadable(self):
        c = _bare_controller()
        c._pump_setup = {}
        c._hardware_config = _FakeHW(pumps={"P1": _FakePump(_FakeSyringe(200.0))})
        # No live position → pump_fill_uL None → pump_fill_pct None.
        c.get_zp_position = lambda cached=True: None
        self.assertIsNone(c.pump_fill_pct("P1"))


# ── 12–15: Control Panel %-of-max speed + mm-vs-%/µL pump jog (offscreen) ──

class TestControlPanelModes(unittest.TestCase):
    """Offscreen GUI smoke: the shared HardwareControlPanel shows jog speed as a
    % of the common max, and routes pump jogs by mode (mm on Hardware Setup,
    %/µL elsewhere)."""

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _ctrl(self):
        c = mock.MagicMock()
        c.is_xy_connected = True
        c.is_zp_connected = True
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        c._pending_per_axis_max_feedrate = None
        c._hardware_config = None
        c.safety_limits = mock.MagicMock()
        c.safety_limits.max_xy_speed = 20000.0
        c.safety_limits.max_z_feedrate = 600.0
        c.safety_limits.max_pump_feedrate = 200.0
        c.get_max_xy_speed_um_s.side_effect = lambda: StageController.get_max_xy_speed_um_s(c)
        c.get_max_z_feedrate_mm_min.side_effect = lambda: StageController.get_max_z_feedrate_mm_min(c)
        c.get_max_pump_feedrate.side_effect = lambda: StageController.get_max_pump_feedrate(c)
        c._pump_jog_max_native.side_effect = lambda: StageController._pump_jog_max_native(c)
        c.get_jog_speed_state.return_value = {}
        c.pump_pct_to_uL.side_effect = lambda p, pct: (pct / 100.0) * 250.0  # 250 µL cap
        return c

    def _panel(self, *, pump_action_labels):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        c = self._ctrl()
        p = HardwareControlPanel(show_connect=False,
                                 bypass_safety=not pump_action_labels,
                                 pump_action_labels=pump_action_labels)
        p.set_controller(c)
        p._resolve_speeds()
        return p, c

    def test_resolved_labels_are_percent_of_max(self):
        p, _ = self._panel(pump_action_labels=False)
        p.spin_xy_pct.setValue(50)
        p.spin_z_pct.setValue(50)
        self.assertIn("10,000 µm/s", p.lbl_xy_resolved.text())   # 50% × 20000
        self.assertIn("300 mm/min", p.lbl_z_resolved.text())     # 50% × 600

    def test_hardware_setup_pump_jog_is_mm(self):
        p, c = self._panel(pump_action_labels=False)
        self.assertFalse(p._jog_array.pump_step_is_uL)
        p._on_jog_pump("P1", 0.1)
        self.assertTrue(_wait_for(lambda: c.move_pump_relative.called))
        self.assertFalse(c.move_pump_uL.called)

    def test_other_pages_pump_jog_is_uL(self):
        # v7.9.x: the emitted distance is a SIGNED µL VOLUME (was a % of the
        # syringe volume, which cannot express a 0.001 µL step).
        p, c = self._panel(pump_action_labels=True)
        self.assertTrue(p._jog_array.pump_step_is_uL)
        p._on_jog_pump("P1", -0.005)           # signed µL (aspirate)
        self.assertTrue(_wait_for(lambda: c.move_pump_uL.called))
        self.assertFalse(c.move_pump_relative.called)
        pump, vol = c.move_pump_uL.call_args[0][0], c.move_pump_uL.call_args[0][1]
        self.assertEqual(pump, "P1")
        self.assertLess(vol, 0.0)              # aspirate = negative µL
        self.assertAlmostEqual(vol, -0.005)    # passed through, not scaled
        self.assertFalse(c.pump_pct_to_uL.called)

    def test_percent_change_pushes_shared_jog_pct(self):
        p, c = self._panel(pump_action_labels=True)
        p.spin_xy_pct.setValue(30)
        c.set_jog_speed_pct.assert_any_call("xy", 30.0)


if __name__ == "__main__":
    unittest.main()
