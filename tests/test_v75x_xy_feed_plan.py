"""Tests for SupportClasses/XYFeedPlan.py (v7.5.x) — the deterministic
feature-aware feed plan that got all 18 geometry-panel shapes under the 30 µm
element on real hardware (2026-07-28 planned panel).
"""

import math
import threading
import unittest

from SupportClasses import XYChallenge as XC
from SupportClasses import XYFeedPlan as FP
from SupportClasses import XYPathSimulator as PS
from SupportClasses.XYStageModel import StageCharacteristics, XYStageModel


ME3B = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                            top_speed_um_s=5945.6, control_loop_ms=31.75,
                            quant_um=1.0, name="ME3B V1")


class TestGeometryAnalysis(unittest.TestCase):
    def test_local_radius_recovers_circle(self):
        circ = XC.make_shape("Circle", 4.0, 0.5)   # radius 2
        for i in (2, 5, 8):
            self.assertAlmostEqual(FP.local_radius_mm(circ, i), 2.0, delta=0.05)

    def test_local_radius_straight_is_inf(self):
        line = [(i * 0.5, 0.0) for i in range(5)]
        self.assertEqual(FP.local_radius_mm(line, 2), float("inf"))

    def test_split_indices_square_corners_not_circle(self):
        sq = XC.make_shape("Square", 5.0, 0.5)
        self.assertEqual(len(FP.split_indices(sq)), 3)   # 3 interior corners
        circ = XC.make_shape("Circle", 2.0, 0.5)         # ~29°/vertex
        self.assertEqual(FP.split_indices(circ), [])

    def test_split_indices_catch_reversals(self):
        lr = XC.make_shape("Line-Reversal", 5.0, 0.5)
        self.assertGreaterEqual(len(FP.split_indices(lr)), 3)


class TestBuildPlan(unittest.TestCase):
    def test_circle_single_section_curvature_sized(self):
        circ = XC.make_shape("Circle", 2.0, 0.5)   # radius 1 mm
        plan = FP.build_plan(circ, ME3B, target_speed_mm_s=3.0)
        self.assertEqual(len(plan.sections), 1)
        sec = plan.sections[0]
        # la = 0.8·√(2·R·δ), δ = 30/1.6 µm → ~0.155 mm
        self.assertLess(sec.lookahead_mm, 0.25)
        self.assertGreater(sec.lookahead_mm, FP.LA_MIN_MM)
        # the stability rule slows the section accordingly
        self.assertLess(sec.speed_mm_s, 3.0)

    def test_square_four_sections_full_speed(self):
        sq = XC.make_shape("Square", 10.0, 0.5)
        plan = FP.build_plan(sq, ME3B, target_speed_mm_s=3.0)
        self.assertEqual(len(plan.sections), 4)
        self.assertEqual(plan.n_stops, 3)
        for sec in plan.sections:
            self.assertAlmostEqual(sec.speed_mm_s, 3.0)
            self.assertIn("straight", sec.reason)

    def test_short_section_caps_lookahead(self):
        pts = [(0, 0), (0.5, 0.0), (0.5, 0.5)]     # 0.5 mm legs, 90° corner
        plan = FP.build_plan(pts, ME3B, target_speed_mm_s=3.0)
        self.assertEqual(len(plan.sections), 2)
        for sec in plan.sections:
            self.assertLessEqual(sec.lookahead_mm,
                                 sec.length_mm * FP.LA_SECTION_FRAC + 1e-9)

    def test_empty_and_degenerate(self):
        self.assertEqual(FP.build_plan([], ME3B,
                                       target_speed_mm_s=3.0).sections, [])
        self.assertEqual(FP.build_plan([(0, 0)], ME3B,
                                       target_speed_mm_s=3.0).sections, [])

    def test_summary_mentions_stops(self):
        sq = XC.make_shape("Square", 5.0, 0.5)
        plan = FP.build_plan(sq, ME3B, target_speed_mm_s=3.0)
        self.assertIn("stop", plan.summary())
        self.assertTrue(plan.notes)


class TestSimulatePlan(unittest.TestCase):
    """The acceptance bar the user set: every shape ≤ 30 µm from ideal.
    Locked here in simulation for the exemplar worst offenders of the
    unplanned panel; the full 18/18 hardware run is in the update plan."""

    def _check(self, shape, size):
        ideal = XC.make_shape(shape, size, 0.5)
        plan = FP.build_plan(ideal, ME3B, target_speed_mm_s=3.0)
        r = FP.simulate_plan(plan, ME3B)
        self.assertTrue(r.completed, f"{shape} {size} did not complete")
        self.assertLessEqual(r.report["p95_um"], 30.0,
                             f"{shape} {size} p95 {r.report['p95_um']:.0f}")
        self.assertLessEqual(r.report["rms_um"], 30.0)
        return r

    def test_star_2mm_previous_deadlock_now_tracks(self):
        self._check("Star", 2.0)

    def test_line_reversal_previous_stall_now_tracks(self):
        self._check("Line-Reversal", 5.0)

    def test_square_corner_cut_gone(self):
        r = self._check("Square", 5.0)
        self.assertLessEqual(r.report["max_um"], 60.0)

    def test_small_circle_curvature_sized(self):
        self._check("Circle", 2.0)

    def test_stop_event_aborts(self):
        stop = threading.Event()
        stop.set()
        sq = XC.make_shape("Square", 5.0, 0.5)
        plan = FP.build_plan(sq, ME3B, target_speed_mm_s=3.0)
        start = plan.sections[0].pts[0]
        model = XYStageModel(ME3B, x_um=start[0] * 1000, y_um=start[1] * 1000)
        samples, info = FP.run_plan(plan, PS.model_io(model), ME3B, stop=stop)
        self.assertFalse(info["completed"])
        self.assertEqual(info["stopped_reason"], "stopped")


class TestLagAwareTaper(unittest.TestCase):
    def test_section_end_overshoot_bounded(self):
        """The naive taper overshot a 3 mm/s section end by ~153 µm on the
        ME3B model; the measured-velocity lag taper must land within the
        deviation budget."""
        sec_pts = [(i * 0.5, 0.0) for i in range(11)]      # 5 mm straight
        plan = FP.build_plan(sec_pts, ME3B, target_speed_mm_s=3.0)
        model = XYStageModel(ME3B)
        samples, info = FP.run_plan(plan, PS.model_io(model), ME3B)
        self.assertTrue(info["completed"])
        overshoot_um = max(0.0, (max(s[0] for s in samples) - 5.0)) * 1000.0
        self.assertLessEqual(overshoot_um, 20.0)

    def test_naive_taper_overshoots_without_optin(self):
        """follow_path with end_lag_s left at 0 keeps the naive taper — at a
        tight arrival tolerance it measurably overshoots (the 153 µm defect
        the lag-aware taper exists to remove), which pins that the new
        parameters are genuinely opt-in rather than silently always-on."""
        sec_pts = [(i * 0.5, 0.0) for i in range(11)]
        tuning = dict(PS.TUNING_DEFAULTS, hold_speed=1.0, decel=0.5)
        resolved = PS.resolve_for(ME3B, print_speed_mm_s=3.0, tuning=tuning)
        model = XYStageModel(ME3B)
        samples, info = PS.follow_path(sec_pts, PS.model_io(model),
                                       print_speed_mm_s=3.0, tuning=tuning,
                                       resolved=resolved, arrive_mm=0.02)
        overshoot_um = max(0.0, (max(s[0] for s in samples) - 5.0)) * 1000.0
        self.assertGreater(overshoot_um, 40.0)


class TestPlanRows(unittest.TestCase):
    def test_rows_time_matched_and_volume_by_arclength(self):
        sq = XC.make_shape("Square", 5.0, 0.5)
        plan = FP.build_plan(sq, ME3B, target_speed_mm_s=3.0)
        rows = FP.plan_rows(plan, vol_per_mm_uL=0.02, z_mm=0.2, pump_index=1)
        self.assertTrue(rows)
        ts = [r[0] for r in rows]
        self.assertEqual(ts, sorted(ts))                  # time monotone
        self.assertTrue(all(r[3] == 0.2 for r in rows))   # Z constant
        total_len = 20.0
        self.assertAlmostEqual(rows[-1][5], total_len * 0.02, places=6)
        self.assertTrue(all(r[4] == 0.0 and r[6] == 0.0 for r in rows))
        # a stop dwell exists at each split (same xy, later t)
        dwells = sum(1 for a, b in zip(rows, rows[1:])
                     if a[1] == b[1] and a[2] == b[2] and b[0] - a[0] > 0.3)
        self.assertEqual(dwells, plan.n_stops)


if __name__ == "__main__":
    unittest.main()
