"""v7.5.x — display-only jog/travel motion interpolation ("live-polling feel").

Covers:
  * MotionEstimator (pure, injected clock): linear target interpolation +
    arrival→idle, zero-distance, malformed input, continuous-jog "advance"
    accumulation with cache-locked base + settle expiry, active_channels,
    display_axes mapping.
  * StageController display getters: get_display_xy_position /
    get_display_zp_position overlay the estimate while a move is in flight and
    fall back to the raw poller cache when idle; motion_estimating maps
    "XY" → {X, Y}; move primitives register estimates (gated to the
    non-suspended poller); the Xbox jog-loop advance callbacks accumulate.

All display-only: nothing here commands hardware.
"""

import unittest

from SupportClasses.MotionEstimator import MotionEstimator
from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits

ME3B_V1_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}  # Z at tuple index 2


class _Clock:
    """Deterministic monotonic-style clock for the estimator under test."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


class _FakeZP:
    """Minimal ZPStageManager stand-in: axis_map + records move_relative."""

    def __init__(self, axis_map=ME3B_V1_MAP):
        self.axis_map = dict(axis_map)
        self.last_axes = None
        self.last_feedrate = None

    def move_relative(self, axes, feedrate=None):
        self.last_axes = dict(axes)
        self.last_feedrate = feedrate


class _FakeXY:
    """Minimal XYStageManager stand-in: records the last commanded move."""

    def __init__(self):
        self.last_abs = None
        self.last_rel = None

    def move_stage_to_position(self, x, y, fast=False):
        self.last_abs = (x, y)

    def move_stage_relative(self, dx, dy):
        self.last_rel = (dx, dy)


# ════════════════════════════════════════════════════════════════════
#  MotionEstimator — pure unit tests
# ════════════════════════════════════════════════════════════════════

class TestMotionEstimatorTarget(unittest.TestCase):
    def setUp(self):
        self.clk = _Clock()
        self.m = MotionEstimator(settle_s=0.4, clock=self.clk)

    def test_scalar_target_interpolates_linearly(self):
        # 0 → 10 mm at 5 mm/s ⇒ 2.0 s duration.
        self.m.note_target("Z", 0.0, 10.0, 5.0)
        self.assertAlmostEqual(self.m.estimate("Z", 0.0), 0.0)
        self.clk.advance(1.0)
        self.assertAlmostEqual(self.m.estimate("Z", 0.0), 5.0)
        self.clk.advance(0.5)
        self.assertAlmostEqual(self.m.estimate("Z", 0.0), 7.5)

    def test_target_lands_on_target_then_goes_idle(self):
        self.m.note_target("Z", 0.0, 10.0, 5.0)
        self.clk.advance(2.0)  # exactly arrived
        self.assertAlmostEqual(self.m.estimate("Z", 0.0), 10.0)
        # Next frame defers to the real cache (estimate cleared).
        self.assertIsNone(self.m.estimate("Z", 0.0))
        self.assertNotIn("Z", self.m.active_channels())

    def test_target_overshoot_clamps_to_target(self):
        self.m.note_target("Z", 0.0, 10.0, 5.0)
        self.clk.advance(99.0)
        self.assertAlmostEqual(self.m.estimate("Z", 0.0), 10.0)

    def test_xy_target_interpolates_both_axes(self):
        self.m.note_target("XY", (0.0, 100.0), (200.0, 100.0), 100.0)  # dur 2 s
        self.clk.advance(1.0)
        x, y = self.m.estimate("XY", (0.0, 100.0, 0.0))
        self.assertAlmostEqual(x, 100.0)
        self.assertAlmostEqual(y, 100.0)

    def test_zero_distance_target_is_not_active(self):
        self.m.note_target("Z", 5.0, 5.0, 5.0)
        # dur == 0 → returns the target once, never reported active.
        self.assertNotIn("Z", self.m.active_channels())
        self.assertAlmostEqual(self.m.estimate("Z", 5.0), 5.0)

    def test_nonpositive_speed_clears_channel(self):
        self.m.note_target("Z", 0.0, 10.0, 0.0)
        self.assertIsNone(self.m.estimate("Z", 0.0))

    def test_malformed_target_does_not_raise(self):
        self.m.note_target("Z", "nope", 10.0, 5.0)  # bad start
        self.assertIsNone(self.m.estimate("Z", 0.0))
        self.assertIsNone(self.m.estimate("BOGUS", 0.0))  # unknown channel

    def test_active_channels_reports_in_flight_only(self):
        self.m.note_target("Z", 0.0, 10.0, 5.0)
        self.assertEqual(self.m.active_channels(), {"Z"})
        self.assertTrue(self.m.is_active())
        self.clk.advance(3.0)
        self.assertEqual(self.m.active_channels(), set())
        self.assertFalse(self.m.is_active())


class TestMotionEstimatorAdvance(unittest.TestCase):
    def setUp(self):
        self.clk = _Clock()
        self.m = MotionEstimator(settle_s=0.4, clock=self.clk)

    def test_advance_locks_base_to_cache_then_accumulates(self):
        # Pre-base advance is discarded when the base locks to the live cache.
        self.m.advance("Z", 1.0)
        self.assertAlmostEqual(self.m.estimate("Z", 100.0), 100.0)  # base locked
        self.m.advance("Z", 2.0)
        self.assertAlmostEqual(self.m.estimate("Z", 100.0), 102.0)
        self.m.advance("Z", 3.0)
        self.assertAlmostEqual(self.m.estimate("Z", 100.0), 105.0)

    def test_advance_does_not_rebase_to_changing_cache(self):
        # Once the base is locked the estimate follows commanded deltas, not the
        # (lagging, 300 ms) cache — so it never sawtooths back.
        self.m.advance("XY", (10.0, 0.0))
        self.m.estimate("XY", (1000.0, 2000.0, 0.0))  # lock base 1000,2000
        self.m.advance("XY", (10.0, 0.0))
        x, y = self.m.estimate("XY", (1234.0, 2000.0, 0.0))  # cache moved; ignored
        self.assertAlmostEqual(x, 1010.0)
        self.assertAlmostEqual(y, 2000.0)

    def test_advance_expires_after_settle(self):
        self.m.advance("Z", 1.0)
        self.m.estimate("Z", 50.0)
        self.assertIn("Z", self.m.active_channels())
        self.clk.advance(0.5)  # > settle_s
        self.assertIsNone(self.m.estimate("Z", 50.0))
        self.assertNotIn("Z", self.m.active_channels())

    def test_advance_nan_ignored(self):
        self.m.advance("Z", float("nan"))
        self.assertIsNone(self.m.estimate("Z", 10.0))


class TestDisplayAxesMapping(unittest.TestCase):
    def test_xy_maps_to_x_and_y(self):
        self.assertEqual(MotionEstimator.display_axes({"XY"}), {"X", "Y"})

    def test_scalar_channels_pass_through(self):
        self.assertEqual(
            MotionEstimator.display_axes({"XY", "Z", "P1"}),
            {"X", "Y", "Z", "P1"})


# ════════════════════════════════════════════════════════════════════
#  StageController display getters + registration hooks
# ════════════════════════════════════════════════════════════════════

class _ControllerBase(unittest.TestCase):
    def setUp(self):
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        # Stop the live poller so our injected cache is stable and the fake ZP
        # never trips the liveness watchdog.
        self.ctrl._pos_poller.stop()
        self.clk = _Clock()
        self.ctrl._motion = MotionEstimator(settle_s=0.4, clock=self.clk)
        self.ctrl.safety_limits = SafetyLimits(
            z_min=0.0, z_max=60.0, enabled=True)
        self.ctrl.zero_position.update(
            {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0})
        self.ctrl.zp_stage = _FakeZP(ME3B_V1_MAP)
        self.ctrl.xy_stage = _FakeXY()
        # Inject a known poller cache (XY abs µm; ZP physical Marlin tuple).
        self.ctrl._pos_poller._xy_pos = (1000.0, 2000.0, 0.0)
        self.ctrl._pos_poller._zp_pos = (0.0, 0.0, 5.0, 0.0)  # Z at index 2

    def tearDown(self):
        try:
            self.ctrl._pos_poller.stop()
        except Exception:
            pass


class TestDisplayGetters(_ControllerBase):
    def test_xy_falls_back_to_cache_when_idle(self):
        self.assertEqual(
            self.ctrl.get_display_xy_position(), (1000.0, 2000.0, 0.0))

    def test_xy_overrides_when_target_active(self):
        self.ctrl._motion.note_target(
            "XY", (1000.0, 2000.0), (3000.0, 2000.0), 1000.0)  # dur 2 s
        self.clk.advance(1.0)
        x, y, z = self.ctrl.get_display_xy_position()
        self.assertAlmostEqual(x, 2000.0)
        self.assertAlmostEqual(y, 2000.0)
        self.assertEqual(z, 0.0)  # cache z slot preserved

    def test_zp_overrides_only_the_z_slot(self):
        self.ctrl._motion.note_target("Z", 5.0, 15.0, 10.0)  # dur 1 s
        self.clk.advance(0.5)
        out = self.ctrl.get_display_zp_position()
        self.assertAlmostEqual(out[2], 10.0)         # interpolated Z (index 2)
        self.assertEqual((out[0], out[1], out[3]), (0.0, 0.0, 0.0))  # cache rest

    def test_zp_falls_back_to_cache_when_idle(self):
        self.assertEqual(
            self.ctrl.get_display_zp_position(), (0.0, 0.0, 5.0, 0.0))

    def test_motion_estimating_maps_xy_to_x_and_y(self):
        self.ctrl._motion.note_target(
            "XY", (1000.0, 2000.0), (3000.0, 2000.0), 1000.0)
        self.assertEqual(self.ctrl.motion_estimating(), {"X", "Y"})
        self.assertTrue(self.ctrl.motion_estimate_active())

    def test_motion_estimating_scalar(self):
        self.ctrl._motion.note_target("Z", 5.0, 15.0, 10.0)
        self.assertEqual(self.ctrl.motion_estimating(), {"Z"})


class TestRegistrationHooks(_ControllerBase):
    def test_move_z_relative_registers_estimate(self):
        # 300 mm/min (under the 500 cap → not clamped) = 5 mm/s.
        self.ctrl.move_z_relative(+2.0, feedrate=300.0)
        # Z move went to the fake stage AND an estimate is now in flight.
        self.assertEqual(self.ctrl.zp_stage.last_axes, {"Z": 2.0})
        self.assertEqual(self.ctrl.motion_estimating(), {"Z"})
        # 2 mm at 5 mm/s ⇒ 0.4 s; at t=0.1 the display is 25% of the way (5→7).
        self.clk.advance(0.1)
        self.assertAlmostEqual(self.ctrl.get_display_zp_position()[2], 5.5)

    def test_move_xy_absolute_um_registers_estimate(self):
        self.ctrl.move_xy_absolute_um(5000.0, 2000.0)
        self.assertEqual(self.ctrl.motion_estimating(), {"X", "Y"})

    def test_suspended_poller_skips_registration(self):
        self.ctrl._pos_poller.suspend()
        self.ctrl.move_z_relative(+2.0, feedrate=600.0)
        # The move still happened, but NO display estimate was registered
        # (programmatic sequence — safe_travel_to / PRINT_PATH path).
        self.assertEqual(self.ctrl.zp_stage.last_axes, {"Z": 2.0})
        self.assertFalse(self.ctrl.motion_estimate_active())

    def test_xy_jog_segment_callback_accumulates(self):
        self.ctrl._note_xy_jog_segment(100.0, 0.0)   # one commanded segment (µm)
        # First display read locks the base to the cache.
        x0, _, _ = self.ctrl.get_display_xy_position()
        self.assertAlmostEqual(x0, 1000.0)
        self.ctrl._note_xy_jog_segment(100.0, 0.0)   # another segment
        x1, _, _ = self.ctrl.get_display_xy_position()
        self.assertAlmostEqual(x1, 1100.0)
        self.assertEqual(self.ctrl.motion_estimating(), {"X", "Y"})

    def test_zp_jog_segment_callback_accumulates_z(self):
        self.ctrl._note_zp_jog_segment({"Z": 0.5, "P1": 0.0, "P2": 0.0, "P3": 0.0})
        self.assertAlmostEqual(self.ctrl.get_display_zp_position()[2], 5.0)  # base
        self.ctrl._note_zp_jog_segment({"Z": 0.5, "P1": 0.0, "P2": 0.0, "P3": 0.0})
        self.assertAlmostEqual(self.ctrl.get_display_zp_position()[2], 5.5)

    def test_display_getters_never_raise_without_estimate(self):
        # Smoke: idle getters return the cache shape unchanged.
        self.assertEqual(len(self.ctrl.get_display_xy_position()), 3)
        self.assertEqual(len(self.ctrl.get_display_zp_position()), 4)


class TestPageCueSmoke(unittest.TestCase):
    """Offscreen: the readout pages mark an in-flight estimate with '~' and the
    fast on_motion_tick path runs without error against a real controller."""

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication([])

    def _controller(self):
        ctrl = StageController(simulate_xy=True, simulate_zp=True)
        ctrl._pos_poller.stop()
        ctrl._motion = MotionEstimator(settle_s=0.4, clock=_Clock())
        ctrl.safety_limits = SafetyLimits(z_min=0.0, z_max=60.0, enabled=True)
        ctrl.zero_position.update(
            {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0})
        ctrl.zp_stage = _FakeZP(ME3B_V1_MAP)
        ctrl.xy_stage = _FakeXY()
        ctrl._pos_poller._xy_pos = (1000.0, 2000.0, 0.0)
        ctrl._pos_poller._zp_pos = (0.0, 0.0, 5.0, 0.0)
        self.addCleanup(lambda: ctrl._pos_poller.stop())
        return ctrl

    def test_control_panel_marks_estimate_and_motion_tick_runs(self):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        ctrl = self._controller()
        panel = HardwareControlPanel(show_connect=False, bypass_safety=True,
                                     pump_action_labels=False)
        panel.set_controller(ctrl)
        # Idle: plain number, no '~'.
        panel.on_status_update()
        self.assertFalse(panel.lbl_pos["X"].text().startswith("~"))
        # In flight: the X/Y readouts are marked estimated.
        ctrl._motion.note_target(
            "XY", (1000.0, 2000.0), (3000.0, 2000.0), 1000.0)
        panel.on_motion_tick()  # the fast (~30 fps) path
        self.assertTrue(panel.lbl_pos["X"].text().startswith("~"))
        self.assertTrue(panel.lbl_pos["Y"].text().startswith("~"))

    def test_jog_page_motion_tick_runs(self):
        from gui.pages.jog_control import JogControlPage
        ctrl = self._controller()
        page = JogControlPage(ctrl)
        ctrl._motion.note_target("Z", 5.0, 15.0, 5.0)
        page.on_status_update()
        page.on_motion_tick()  # must not raise

    def test_stage_panel_motion_tick_runs(self):
        from gui.pages.hardware.stage_panel import StageHardwarePanel
        ctrl = self._controller()
        panel = StageHardwarePanel()
        panel.set_controller(ctrl)
        ctrl._motion.note_target("Z", 5.0, 15.0, 5.0)
        panel.on_status_update()
        panel.on_motion_tick()  # must not raise


if __name__ == "__main__":
    unittest.main()
