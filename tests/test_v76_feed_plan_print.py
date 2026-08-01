"""test_v76_feed_plan_print.py — the FEED PLAN drives the real velocity print.

One fixed follower tuning provably cannot hold a resolution element through a
corner (pure pursuit cuts corners by ≈0.4·lookahead and cannot turn a 180°
reversal at all — measured 0/18 geometry-panel shapes under 30 µm). The feed
plan splits the path at sharp corners/reversals, stops on those vertices, and
sizes each section from its own curvature — measured 18/18 under 30 µm on
hardware.

These tests pin the INTEGRATION: the flag-off byte-identity, the "not
characterised → legacy" fallback, per-section execution, the corner stops, and —
the one that would silently ruin prints — pump-volume conservation across the
section boundaries.

Reuses the integrating fake stage from the velocity-follow suite (it follows VS
and accumulates µL), so the loop physically progresses with no hardware.
"""

import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType)
from SupportClasses import XYFeedPlan as FP
from SupportClasses.XYStageModel import StageCharacteristics

from tests.test_v75x_quick_print_velocity_follow import _VelCtrl


# ME3B V1's measured dynamics, as they arrive stamped on a job.
ME3B_TUNE = {"dead_time_s": 0.067, "tau_s": 0.027}
ME3B_TOP_UM_S = 5945.6
ME3B_LOOP_MS = 31.75


def _settings(**kw):
    s = PrintSettings(velocity_follow=True, feed_plan_enabled=True,
                      print_speed_mm_s=3.0, feed_plan_element_um=30.0)
    s.vel_tuning = dict(ME3B_TUNE)
    s.xy_max_speed_um_s = ME3B_TOP_UM_S
    s.control_loop_ms = ME3B_LOOP_MS
    for k, v in kw.items():
        setattr(s, k, v)
    return s


class _Log:
    """Captures exec-log events so the plan's decisions are inspectable."""

    def __init__(self):
        self.events = []

    def log(self, ev, **kw):
        self.events.append((ev, kw))

    def xy_cmd_fields(self, *_a, **_k):
        return {}

    def of(self, ev):
        return [kw for name, kw in self.events if name == ev]


def _run(ctrl, pts, *, settings=None, flow=0.4, log=None):
    pm = PrintManager(ctrl)
    pm.job = SimpleNamespace(settings=settings or _settings())
    pm.exec_logger = log
    pm._print_pump_suckback = lambda *a, **k: None
    pm._zp_connected_at_start = True
    cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
        "points": pts, "pump": "P1",
        "flow_rate_uL_s": flow, "flow_rate": 0.01})
    handled = pm._execute_print_path_feed_plan(cmd)
    return pm, handled


class TestFallbacks(unittest.TestCase):
    def test_uncharacterised_machine_returns_false_without_motion(self):
        ctrl = _VelCtrl()
        s = _settings()
        s.vel_tuning = {}                       # no dead time → not complete
        s.xy_max_speed_um_s = 0.0
        s.control_loop_ms = 0.0
        log = _Log()
        pm, handled = _run(ctrl, [(0.0, 0.0), (3.0, 0.0)], settings=s, log=log)
        self.assertFalse(handled, "must fall back to the legacy follower")
        self.assertEqual(ctrl.vel_calls, 0, "no motion before falling back")
        self.assertEqual(log.of("feed_plan_fallback")[0]["reason"],
                         "machine_not_characterised")

    def test_degenerate_path_returns_false(self):
        ctrl = _VelCtrl()
        _pm, handled = _run(ctrl, [(1.0, 1.0)])
        self.assertFalse(handled)
        _pm2, handled2 = _run(ctrl, [])
        self.assertFalse(handled2)

    def test_feed_plan_char_from_stamped_settings(self):
        pm = PrintManager(_VelCtrl())
        char = pm._feed_plan_char(_settings())
        self.assertIsNotNone(char)
        self.assertAlmostEqual(char.dead_time_s, 0.067)
        self.assertAlmostEqual(char.tau_s, 0.027)
        self.assertAlmostEqual(char.top_speed_um_s, ME3B_TOP_UM_S)
        # incomplete → None (the fallback signal)
        s = _settings()
        s.xy_max_speed_um_s = 0.0
        self.assertIsNone(pm._feed_plan_char(s))


class TestDispatch(unittest.TestCase):
    def test_flag_off_never_calls_the_plan(self):
        """Byte-identity guarantee: with the flag off the legacy path runs and
        the plan method is never even consulted."""
        ctrl = _VelCtrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=PrintSettings(
            velocity_follow=True, print_speed_mm_s=3.0))   # flag defaults False
        pm.exec_logger = None
        pm._print_pump_suckback = lambda *a, **k: None
        pm._zp_connected_at_start = True
        pm._execute_print_path_feed_plan = MagicMock()
        pm._execute_print_path_velocity(PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": [(0.0, 0.0), (2.0, 0.0)], "pump": "P1",
                    "flow_rate_uL_s": 0.4, "flow_rate": 0.01}))
        pm._execute_print_path_feed_plan.assert_not_called()

    def test_flag_on_delegates_then_returns(self):
        ctrl = _VelCtrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=_settings())
        pm.exec_logger = None
        pm._execute_print_path_feed_plan = MagicMock(return_value=True)
        pm._execute_print_path_velocity(PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": [(0.0, 0.0), (2.0, 0.0)], "pump": "P1",
                    "flow_rate_uL_s": 0.4, "flow_rate": 0.01}))
        pm._execute_print_path_feed_plan.assert_called_once()
        self.assertEqual(ctrl.vel_calls, 0, "legacy loop must not also run")

    def test_plan_returning_false_falls_through_to_legacy(self):
        ctrl = _VelCtrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=_settings())
        pm.exec_logger = None
        pm._print_pump_suckback = lambda *a, **k: None
        pm._zp_connected_at_start = True
        pm._execute_print_path_feed_plan = MagicMock(return_value=False)
        pm._execute_print_path_velocity(PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": [(0.0, 0.0), (2.0, 0.0)], "pump": "P1",
                    "flow_rate_uL_s": 0.4, "flow_rate": 0.01}))
        self.assertGreater(ctrl.vel_calls, 0, "legacy follower must have run")


class TestSectionedExecution(unittest.TestCase):
    def _L(self):
        """An L: 3 mm right then 3 mm up — one 90° corner → 2 sections."""
        return ([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
                + [(3.0, 1.0), (3.0, 2.0), (3.0, 3.0)])

    def test_corner_becomes_two_sections_with_a_stop(self):
        """The v7.6 "stop" policy — still available, no longer the default."""
        ctrl = _VelCtrl()
        pts = self._L()
        ctrl._x, ctrl._y = 0.0, 0.0
        log = _Log()
        pm, handled = _run(ctrl, pts, log=log,
                           settings=_settings(feed_plan_corner_policy="stop"))
        self.assertTrue(handled)
        secs = log.of("plan_section")
        self.assertEqual(len(secs), 2)
        self.assertEqual([s["index"] for s in secs], [0, 1])
        # each section carries its own reasoning + numbers
        for s in secs:
            self.assertIn("reason", s)
            self.assertGreater(s["lookahead_mm"], 0.0)
            self.assertGreater(s["speed_mm_s"], 0.0)
        # a stop between the sections + the always-stop exits
        self.assertGreaterEqual(ctrl.stop_calls, 2)
        end = log.of("path_end")[0]
        self.assertTrue(end["feed_plan"])
        self.assertEqual(end["status"], "arrived")
        self.assertEqual(end["sections_done"], 2)

    def test_path_start_reports_the_plan(self):
        ctrl = _VelCtrl()
        log = _Log()
        _run(ctrl, self._L(), log=log,
             settings=_settings(feed_plan_corner_policy="stop"))
        st = log.of("path_start")[0]
        self.assertTrue(st["feed_plan"])
        self.assertEqual(st["n_sections"], 2)
        self.assertEqual(st["n_stops"], 1)
        self.assertEqual(st["element_um"], 30.0)
        self.assertEqual(st["corner_policy"], "stop")
        self.assertGreater(st["plan_est_s"], 0.0)

    # ── v7.7: "slow" is the DEFAULT policy — never pause on a corner ──

    def test_default_policy_slows_through_a_corner_in_one_section(self):
        """Operator requirement: never pause on corners, just slow down — and
        keep the pump moving for the whole path."""
        ctrl = _VelCtrl()
        log = _Log()
        _run(ctrl, self._L(), log=log)          # default settings
        st = log.of("path_start")[0]
        self.assertEqual(st["corner_policy"], "slow")
        self.assertTrue(st["continuous"])
        self.assertEqual(st["n_sections"], 1)
        self.assertEqual(st["n_stops"], 0)
        secs = log.of("plan_section")
        self.assertEqual(len(secs), 1)
        end = log.of("path_end")[0]
        self.assertEqual(end["status"], "arrived")

    def test_the_pump_keeps_advancing_across_the_corner(self):
        """With no section split there is no dwell, so deposition is monotone
        and uninterrupted from start to finish."""
        ctrl = _VelCtrl()
        log = _Log()
        pm, _ = _run(ctrl, self._L(), log=log)
        vols = [k["deposited_uL"] for k in log.of("vel_sample")
                if k.get("deposited_uL") is not None]
        self.assertGreater(len(vols), 2)
        self.assertEqual(vols, sorted(vols), "deposition must be monotone")
        self.assertGreater(vols[-1], vols[0], "the pump must keep moving")

    def test_reaches_the_final_point(self):
        ctrl = _VelCtrl()
        pts = self._L()
        _run(ctrl, pts)
        self.assertLess(math.hypot(ctrl._x / 1000.0 - pts[-1][0],
                                   ctrl._y / 1000.0 - pts[-1][1]), 0.15)

    def test_reversal_path_completes(self):
        """A 180° reversal — the geometry that STALLED the single-tuning
        follower forever — completes under the plan."""
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0),
               (1.0, 0.0), (0.0, 0.0)]
        ctrl = _VelCtrl()
        log = _Log()
        pm, handled = _run(ctrl, pts, log=log)
        self.assertTrue(handled)
        self.assertEqual(log.of("path_end")[0]["status"], "arrived")
        self.assertGreaterEqual(len(log.of("plan_section")), 2)

    def test_vel_samples_carry_the_section_index(self):
        ctrl = _VelCtrl()
        log = _Log()
        _run(ctrl, self._L(), log=log)
        samples = log.of("vel_sample")
        if samples:                     # sampling is every 5th tick
            self.assertIn("sec", samples[0])


class TestVolumeConservation(unittest.TestCase):
    """The defect that would silently ruin prints: section-local arc length
    resetting to 0 at every split, so the pump either under- or double-deposits
    at each corner."""

    def test_total_volume_matches_path_length(self):
        # square: 3 corners → 4 sections
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0),
               (2.0, 1.0), (2.0, 2.0),
               (1.0, 2.0), (0.0, 2.0),
               (0.0, 1.0), (0.0, 0.02)]
        ctrl = _VelCtrl()
        flow, speed = 0.4, 3.0
        s = _settings(print_speed_mm_s=speed)
        pm, handled = _run(ctrl, pts, settings=s, flow=flow)
        self.assertTrue(handled)
        total = FP.VC.polyline_arclength(pts)[-1]
        expected = total * (flow / speed)
        self.assertAlmostEqual(ctrl.pump_uL, expected, delta=expected * 0.05)

    def test_deposits_are_monotone(self):
        pts = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
        emitted = []

        class _Rec(_VelCtrl):
            def move_pump_uL(self, pump, uL, rate=None):
                emitted.append(uL)
                super().move_pump_uL(pump, uL, rate)

        ctrl = _Rec()
        _run(ctrl, pts)
        self.assertTrue(all(v > 0 for v in emitted),
                        "every emission must be a forward deposit")

    def test_no_volume_when_flow_is_zero(self):
        ctrl = _VelCtrl()
        _run(ctrl, [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], flow=0.0)
        self.assertAlmostEqual(ctrl.pump_uL, 0.0, places=6)

    def test_tiny_section_is_skipped_but_volume_stays_exact(self):
        """Two splits a few µm apart create a section shorter than its arrive
        tolerance: it must be skipped (undriveable) without losing its length
        from the deposited volume."""
        pts = [(0.0, 0.0), (1.0, 0.0),
               (1.0, 0.005), (1.005, 0.005),     # ~5 µm zig → tiny section
               (1.005, 1.0), (1.005, 2.0)]
        ctrl = _VelCtrl()
        flow, speed = 0.4, 3.0
        pm, handled = _run(ctrl, pts, settings=_settings(print_speed_mm_s=speed),
                           flow=flow)
        self.assertTrue(handled)
        total = FP.VC.polyline_arclength(pts)[-1]
        expected = total * (flow / speed)
        self.assertAlmostEqual(ctrl.pump_uL, expected, delta=expected * 0.05)


class TestAbortDuringPlan(unittest.TestCase):
    def test_abort_mid_section_stops_and_reports(self):
        pts = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
        ctrl = _VelCtrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=_settings())
        log = _Log()
        pm.exec_logger = log
        pm._print_pump_suckback = lambda *a, **k: None
        pm._zp_connected_at_start = True
        threading.Timer(0.25, pm._abort_flag.set).start()
        handled = pm._execute_print_path_feed_plan(PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": pts, "pump": "P1",
                    "flow_rate_uL_s": 0.4, "flow_rate": 0.01}))
        self.assertTrue(handled)
        end = log.of("path_end")[0]
        self.assertEqual(end["status"], "aborted")
        self.assertTrue(end["aborted"])
        self.assertGreaterEqual(ctrl.stop_calls, 1)

    def test_abort_before_start_issues_no_section(self):
        ctrl = _VelCtrl()
        pm = PrintManager(ctrl)
        pm.job = SimpleNamespace(settings=_settings())
        log = _Log()
        pm.exec_logger = log
        pm._print_pump_suckback = lambda *a, **k: None
        pm._zp_connected_at_start = True
        pm._abort_flag.set()
        handled = pm._execute_print_path_feed_plan(PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
                    "pump": "P1", "flow_rate_uL_s": 0.4, "flow_rate": 0.01}))
        self.assertTrue(handled)
        self.assertEqual(log.of("plan_section"), [])

    def test_zp_disconnect_mid_path_stops(self):
        class _Drop(_VelCtrl):
            def __init__(self):
                super().__init__()
                self._reads = 0

            def get_xy_position(self, cached=False):
                self._reads += 1
                if self._reads > 5:
                    self.is_zp_connected = False
                return super().get_xy_position(cached)

        ctrl = _Drop()
        log = _Log()
        pm, handled = _run(ctrl, [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)], log=log)
        self.assertTrue(handled)
        self.assertEqual(log.of("path_end")[0]["status"], "zp_disconnect")


class TestPureModuleRefactor(unittest.TestCase):
    """`tuning_for` / `section_end_taper` were extracted so the print executor
    reuses the plan's own numbers — the simulation and the print must not drift
    apart."""

    def test_tuning_for_is_public_with_alias(self):
        sec = FP.PlanSection(pts=[(0, 0), (1, 0)], lookahead_mm=0.5,
                             speed_mm_s=2.0, length_mm=1.0)
        self.assertEqual(FP.tuning_for(sec), FP._tuning_for(sec))
        t = FP.tuning_for(sec)
        self.assertEqual(t["lookahead"], 0.5)
        self.assertEqual(t["corner_angle"], 89.0)     # corners are no-ops
        self.assertEqual(t["hold_speed"], 1.0)

    def test_section_end_taper_scales_with_lag_and_speed(self):
        char = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                                    top_speed_um_s=5945.6,
                                    control_loop_ms=31.75)
        slow = FP.PlanSection(pts=[], lookahead_mm=0.2, speed_mm_s=0.5,
                              length_mm=5.0)
        fast = FP.PlanSection(pts=[], lookahead_mm=0.9, speed_mm_s=3.0,
                              length_mm=5.0)
        lag_s, floor_s = FP.section_end_taper(char, slow)
        lag_f, floor_f = FP.section_end_taper(char, fast)
        self.assertAlmostEqual(lag_s, lag_f)          # lag is machine-only
        self.assertAlmostEqual(lag_s, (0.067 + 0.027 + 0.031750 / 2)
                               * FP.END_LAG_MARGIN, places=5)
        self.assertGreater(floor_s, floor_f)          # slower → larger fraction

    def test_simulation_still_matches_after_the_refactor(self):
        char = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                                    top_speed_um_s=5945.6,
                                    control_loop_ms=31.75)
        from SupportClasses import XYChallenge as XC
        ideal = XC.make_shape("Square", 5.0, 0.5)
        plan = FP.build_plan(ideal, char, target_speed_mm_s=3.0)
        r = FP.simulate_plan(plan, char)
        self.assertTrue(r.completed)
        self.assertLessEqual(r.report["p95_um"], 30.0)


class TestResolutionFloor(unittest.TestCase):
    def test_me3b_floor_below_the_proven_element(self):
        """The floor must not claim 30 µm is unattainable — the hardware panel
        held it with a worst cell of 27 µm."""
        char = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                                    top_speed_um_s=5945.6,
                                    control_loop_ms=31.75)
        floor = FP.min_attainable_resolution_um(char)
        self.assertGreater(floor, 15.0)
        self.assertLess(floor, 30.0)

    def test_incomplete_machine_returns_zero(self):
        self.assertEqual(
            FP.min_attainable_resolution_um(StageCharacteristics()), 0.0)

    def test_margin_scales(self):
        char = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                                    top_speed_um_s=5945.6,
                                    control_loop_ms=31.75)
        base = FP.min_attainable_resolution_um(char)
        self.assertAlmostEqual(
            FP.min_attainable_resolution_um(char, margin=2.0), base * 2.0)


class TestTauStamp(unittest.TestCase):
    def test_stamp_print_settings_includes_tau(self):
        from SupportClasses import XYAutoCalibration as AC

        class FakeStore:
            def get_mode_params(self, mode):
                return {"lookahead_mm": 0.9} if mode == "velocity" else {}

            def get_xy_max_speed_um_s(self):
                return ME3B_TOP_UM_S

            def get_control_loop_ms(self):
                return ME3B_LOOP_MS

            def get_phase_lag_s(self):
                return 0.0

            def get_velocity_dead_time_s(self):
                return 0.067

            def get_velocity_dead_time_meta(self):
                return {"tau_s": 0.027}

        s = AC.stamp_print_settings(PrintSettings(), FakeStore())
        self.assertAlmostEqual(s.vel_tuning["dead_time_s"], 0.067)
        self.assertAlmostEqual(s.vel_tuning["tau_s"], 0.027)
        # and that is enough for the plan's characteristics gate
        self.assertIsNotNone(PrintManager(_VelCtrl())._feed_plan_char(s))


if __name__ == "__main__":
    unittest.main()
