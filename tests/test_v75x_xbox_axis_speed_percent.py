"""v7.5.x tests — Xbox jog speed as a percentage of the calibrated max
(MEBP_v75x_XBOX_AXIS_SPEED_PERCENT).

Each joystick-driven axis group (XY / Z / Pump) jogs at a PERCENTAGE of its
calibrated max move speed, cycled by the controller through the fixed ladder
``0.1, 0.3, 1, 3, 10, 30, 100``%. 100% = the per-axis calibrated max.

Covered:
  Ladder helper:
    1.  _jog_speed_ladder_step walks up/down and clamps at both ends.
    2.  Snaps from an arbitrary % to the nearest rung before stepping.
  ZPJogHandler:
    3.  increment_zspeed_up/down (via Processor) walks the ladder; the derived
        z_speed tracks pct/100 * z_speed_max.
    4.  Z velocity clamps at z_speed_max (100% reaches it, never exceeds).
    5.  increment_pspeed_* walks the pump ladder; derived p_speed tracks max.
    6.  set_z_speed_max / set_p_speed_pct recompute the derived scalar.
  XYJogHandler:
    7.  increment_xyspeed_* walks the ladder; xy_speed + max_speed track max.
  StageController:
    8.  refresh_jog_speed_limits converts Z mm/min→mm/s and reads max_xy_speed.
    9.  _pump_jog_max_native anchors to the largest configured flow rate.
    10. _apply_jog_speed_pct re-applies a stored % on handler (re)creation.
    11. get_jog_speed_state reports {pct, speed, unit} per present group.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.Processor import Processor
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import (
    JOG_SPEED_LADDER_PCT, _jog_speed_ladder_step,
    StageController, XYJogHandler, ZPJogHandler,
)


# ── Stage stubs ─────────────────────────────────────────────────────

class _FakeZPStage:
    def __init__(self):
        self.moves = []

    def move_relative(self, deltas, feedrate):
        self.moves.append((dict(deltas or {}), feedrate))


class _FakeXYStage:
    def __init__(self):
        self.calls = []

    def move_stage_at_velocity(self, vx, vy):
        self.calls.append((vx, vy))


# ── 1–2: ladder helper ──────────────────────────────────────────────

class TestLadderStep(unittest.TestCase):

    def test_ladder_values(self):
        self.assertEqual(JOG_SPEED_LADDER_PCT,
                         (0.1, 0.3, 1.0, 3.0, 10.0, 30.0, 100.0))

    def test_steps_up_and_down(self):
        self.assertEqual(_jog_speed_ladder_step(1.0, +1), 3.0)
        self.assertEqual(_jog_speed_ladder_step(3.0, -1), 1.0)
        self.assertEqual(_jog_speed_ladder_step(10.0, +1), 30.0)

    def test_clamps_at_both_ends(self):
        self.assertEqual(_jog_speed_ladder_step(100.0, +1), 100.0)
        self.assertEqual(_jog_speed_ladder_step(0.1, -1), 0.1)

    def test_snaps_to_nearest_rung_first(self):
        # 7% is nearest 10% → up = 30%, down = 3%
        self.assertEqual(_jog_speed_ladder_step(7.0, +1), 30.0)
        self.assertEqual(_jog_speed_ladder_step(7.0, -1), 3.0)

    def test_bad_value_falls_back(self):
        self.assertEqual(_jog_speed_ladder_step(None, +1), 0.3)


# ── 3–6: ZPJogHandler ───────────────────────────────────────────────

class TestZPJogSpeedPercent(unittest.TestCase):

    def setUp(self):
        self.proc = Processor(name="TestProcZPPct")
        self.addCleanup(self.proc.stop)
        self.jog = ZPJogHandler(self.proc, _FakeZPStage())

    def test_derived_z_speed_tracks_pct_and_max(self):
        self.jog.set_z_speed_max(10.0)   # mm/s at 100%
        self.jog.set_z_speed_pct(10.0)
        self.assertAlmostEqual(self.jog.z_speed, 1.0)   # 10% of 10
        self.jog.set_z_speed_pct(30.0)
        self.assertAlmostEqual(self.jog.z_speed, 3.0)

    def test_increment_z_walks_ladder(self):
        self.jog.set_z_speed_max(10.0)
        self.jog.z_speed_pct = 10.0
        self.jog._recompute_zp_speeds()
        self.jog._incr_z_up()
        self.assertEqual(self.jog.z_speed_pct, 30.0)
        self.assertAlmostEqual(self.jog.z_speed, 3.0)
        self.jog._incr_z_down()
        self.jog._incr_z_down()
        self.assertEqual(self.jog.z_speed_pct, 3.0)
        self.assertAlmostEqual(self.jog.z_speed, 0.3)

    def test_z_velocity_clamps_at_max(self):
        self.jog.set_z_speed_max(2.0)
        self.jog.set_z_speed_pct(100.0)   # z_speed = 2.0
        # full+ deflection must not exceed z_speed_max
        self.jog._handle_z_vel(average=1.5)
        self.assertLessEqual(abs(self.jog.vel_z), 2.0 + 1e-9)
        self.assertAlmostEqual(abs(self.jog.vel_z), 2.0)

    def test_increment_pump_walks_ladder(self):
        self.jog.set_p_speed_max(10.0)
        self.jog.p_speed_pct = 1.0
        self.jog._recompute_zp_speeds()
        self.assertAlmostEqual(self.jog.p_speed, 0.1)
        self.jog._incr_p_up()
        self.assertEqual(self.jog.p_speed_pct, 3.0)
        self.assertAlmostEqual(self.jog.p_speed, 0.3)


# ── 7: XYJogHandler ─────────────────────────────────────────────────

class TestXYJogSpeedPercent(unittest.TestCase):

    def setUp(self):
        self.proc = Processor(name="TestProcXYPct")
        self.addCleanup(self.proc.stop)
        self.jog = XYJogHandler(self.proc, _FakeXYStage())

    def test_derived_xy_speed_and_cap_track_max(self):
        self.jog.set_speed_max(10000.0)
        self.jog.set_speed_pct(10.0)
        self.assertAlmostEqual(self.jog.xy_speed, 1000.0)
        self.assertAlmostEqual(self.jog.max_speed, 10000.0)  # 100% reaches max

    def test_increment_xy_walks_ladder(self):
        self.jog.set_speed_max(10000.0)
        self.jog.speed_pct = 10.0
        self.jog._recompute_xy_speed()
        self.jog._incr_up()
        self.assertEqual(self.jog.speed_pct, 30.0)
        self.assertAlmostEqual(self.jog.xy_speed, 3000.0)
        self.jog._incr_down()
        self.assertEqual(self.jog.speed_pct, 10.0)

    def test_velocity_clamps_at_max(self):
        self.jog.set_speed_max(5000.0)
        self.jog.set_speed_pct(100.0)
        self.jog._handle_vel(average=(2.0, 0.0))   # over-deflected
        self.assertAlmostEqual(abs(self.jog.vel_x), 5000.0)


# ── 8–11: StageController plumbing ──────────────────────────────────

class TestControllerJogSpeed(unittest.TestCase):

    def _controller(self):
        # __new__ avoids the full hardware-touching __init__.
        c = StageController.__new__(StageController)
        c.safety_limits = SafetyLimits()
        c._pending_per_axis_max_feedrate = None
        c._hardware_config = None
        c._jog_speed_pct = {}
        proc = Processor(name="TestProcCtrl")
        self.addCleanup(proc.stop)
        c.xy_jog = XYJogHandler(proc, _FakeXYStage())
        c.zp_jog = ZPJogHandler(proc, _FakeZPStage())
        return c

    def test_refresh_converts_units(self):
        c = self._controller()
        c.safety_limits.max_xy_speed = 8000.0       # µm/s
        c.safety_limits.max_z_feedrate = 600.0      # mm/min
        c.refresh_jog_speed_limits()
        self.assertAlmostEqual(c.xy_jog.speed_max, 8000.0)
        self.assertAlmostEqual(c.zp_jog.z_speed_max, 10.0)  # 600 / 60

    def test_refresh_prefers_per_axis_feedrate_for_z(self):
        c = self._controller()
        c._pending_per_axis_max_feedrate = {"Z": 1200.0}
        c.refresh_jog_speed_limits()
        self.assertAlmostEqual(c.zp_jog.z_speed_max, 20.0)  # 1200 / 60

    def test_pump_anchor_uses_largest_flow_rate(self):
        c = self._controller()

        class _Pump:
            is_configured = True

        class _HW:
            configured_pump_ids = ["P1", "P2"]
            pumps = {"P1": _Pump(), "P2": _Pump()}

        c._hardware_config = _HW()
        c.zp_jog.set_hardware_config(_HW())
        c.safety_limits.set_max_flow_rate("P1", 2.0)
        c.safety_limits.set_max_flow_rate("P2", 8.0)
        self.assertAlmostEqual(c._pump_jog_max_native(), 8.0)
        c.refresh_jog_speed_limits()
        self.assertAlmostEqual(c.zp_jog.p_speed_max, 8.0)

    def test_pump_anchor_legacy_mode(self):
        c = self._controller()
        c.safety_limits.max_pump_feedrate = 300.0   # mm/min
        self.assertAlmostEqual(c._pump_jog_max_native(), 5.0)  # 300 / 60

    def test_set_jog_speed_pct_applies_to_handlers(self):
        c = self._controller()
        c.set_jog_speed_pct("xy", 30.0)
        c.set_jog_speed_pct("z", 1.0)
        c.set_jog_speed_pct("pump", 3.0)   # alias → p
        self.assertEqual(c.xy_jog.speed_pct, 30.0)
        self.assertEqual(c.zp_jog.z_speed_pct, 1.0)
        self.assertEqual(c.zp_jog.p_speed_pct, 3.0)

    def test_apply_pct_survives_handler_recreation(self):
        c = self._controller()
        c.set_jog_speed_pct("z", 30.0)
        # Simulate a reconnect that builds a fresh handler at the default %.
        proc = Processor(name="TestProcReconnect")
        self.addCleanup(proc.stop)
        c.zp_jog = ZPJogHandler(proc, _FakeZPStage())
        self.assertEqual(c.zp_jog.z_speed_pct, 10.0)  # default before re-apply
        c.refresh_jog_speed_limits()                  # re-applies stored %
        self.assertEqual(c.zp_jog.z_speed_pct, 30.0)

    def test_get_jog_speed_state(self):
        c = self._controller()
        c.safety_limits.max_xy_speed = 10000.0
        c.refresh_jog_speed_limits()
        c.set_jog_speed_pct("xy", 10.0)
        state = c.get_jog_speed_state()
        self.assertIn("xy", state)
        self.assertIn("z", state)
        self.assertIn("p", state)
        self.assertEqual(state["xy"]["pct"], 10.0)
        self.assertEqual(state["xy"]["unit"], "µm/s")
        self.assertAlmostEqual(state["xy"]["speed"], 1000.0)

    def test_get_jog_speed_state_without_handlers(self):
        c = self._controller()
        c.xy_jog = None
        c.zp_jog = None
        c.set_jog_speed_pct("z", 30.0)
        state = c.get_jog_speed_state()
        # No handler → reports the stored % with no resolved speed.
        self.assertEqual(state["z"]["pct"], 30.0)
        self.assertIsNone(state["z"]["speed"])


if __name__ == "__main__":
    unittest.main()
