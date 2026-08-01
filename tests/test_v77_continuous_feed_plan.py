# -*- coding: utf-8 -*-
"""v7.7 — the never-stopping ("slow through corners") feed plan.

Operator requirement: *"We should never pause on corners, just slow down. We
always want the pump moving unless there is a break in the section that is
planned."*

The physics that makes this work, and which these tests pin:

* Slowing alone does nothing for accuracy — the pure-pursuit cut is set by the
  LOOKAHEAD, so the carrot must shrink at a corner and the speed follows from
  dead-time stability (``v ≤ la/(L·safety)``).
* At a **sharp vertex** the cut is ``≈ (la/2)·sin(θ/2)`` — LINEAR in lookahead.
  The arc formula ``√(2·R·δ)`` describes a smoothly curved path and badly
  under-constrains a corner: it asked for 0.092 mm at a 90° vertex and simulated
  40 µm against an 18.75 µm budget.
* A **smooth curve** needs the arc model even though no single vertex turns much
  (a 5 mm circle sampled at 0.5 mm turns only ~11°/vertex), so knots cannot be
  gated on turn angle.
* A **near-180° reversal** must still stop: a cusp cannot be traversed by pure
  pursuit at any speed, because a carrot placed forward in arc length sits behind
  the stage in space.
"""

import math
import unittest

from SupportClasses import XYChallenge as XC
from SupportClasses import XYFeedPlan as FP
from SupportClasses import VelocityControl as VC
from SupportClasses.XYStageModel import StageCharacteristics

#: ME3B V1 as measured 2026-07-28.
CHAR = StageCharacteristics(dead_time_s=0.0671, tau_s=0.0271,
                            top_speed_um_s=5945.6, control_loop_ms=32.0)
SPEED, ELEM = 3.0, 30.0


def _L():
    """3 mm right then 3 mm up — one 90° corner."""
    return [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0),
            (3.0, 1.0), (3.0, 2.0), (3.0, 3.0)]


class TestCornerBudgetModel(unittest.TestCase):
    def test_a_sharp_corner_needs_a_shorter_carrot_than_the_arc_model_says(self):
        delta = 0.01875                       # 30 µm / 1.6 headroom
        corner = FP.corner_budget_lookahead_mm(90.0, delta)
        arc = FP.LA_CURVE_MARGIN * math.sqrt(2.0 * 0.354 * delta)
        self.assertLess(corner, arc)
        self.assertAlmostEqual(corner, 2.0 * delta / math.sin(math.pi / 4), 6)

    def test_sharper_turns_demand_shorter_carrots(self):
        d = 0.01875
        self.assertGreater(FP.corner_budget_lookahead_mm(30.0, d),
                           FP.corner_budget_lookahead_mm(90.0, d))
        self.assertGreater(FP.corner_budget_lookahead_mm(90.0, d),
                           FP.corner_budget_lookahead_mm(180.0, d))

    def test_a_straight_vertex_constrains_nothing(self):
        self.assertEqual(FP.corner_budget_lookahead_mm(0.0, 0.01875),
                         float("inf"))

    def test_the_cut_model_matches_the_budget_it_was_inverted_from(self):
        d = 0.01875
        for ang in (30.0, 60.0, 90.0, 120.0):
            la = FP.corner_budget_lookahead_mm(ang, d)
            cut = (la / 2.0) * math.sin(math.radians(ang) / 2.0)
            self.assertAlmostEqual(cut, d, places=9)


class TestArcProfile(unittest.TestCase):
    def test_it_is_a_step_lookup_over_the_grid(self):
        p = FP.ArcProfile([0.0, 1.0, 2.0], [3.0, 2.0, 1.0], 9.0)
        self.assertEqual(p(-5.0), 3.0)
        self.assertEqual(p(0.5), 3.0)
        self.assertEqual(p(1.0), 2.0)
        self.assertEqual(p(99.0), 1.0)
        self.assertEqual(p.min_value(), 1.0)

    def test_an_empty_grid_falls_back(self):
        self.assertEqual(FP.ArcProfile([], [], 7.0)(1.0), 7.0)


class TestContinuousPlan(unittest.TestCase):
    def test_a_corner_does_not_split_the_path(self):
        plan = FP.build_continuous_plan(_L(), CHAR, target_speed_mm_s=SPEED,
                                        element_um=ELEM)
        self.assertTrue(plan.continuous)
        self.assertEqual(len(plan.sections), 1)
        self.assertEqual(plan.n_stops, 0)
        self.assertIn("continuous", plan.summary())

    def test_the_profiles_slow_and_shorten_at_the_corner(self):
        pts = _L()
        cum = VC.polyline_arclength(pts)
        plan = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                        element_um=ELEM)
        sec = plan.sections[0]
        s_corner = cum[3]                       # the 90° vertex
        self.assertLess(sec.lookahead_at(s_corner), sec.lookahead_mm)
        self.assertLess(sec.speed_at(s_corner), sec.speed_mm_s)
        # and both recover on the straight away from it
        self.assertAlmostEqual(sec.lookahead_at(0.0), sec.lookahead_mm, 6)

    def test_the_slowdown_is_commanded_BEFORE_the_corner(self):
        """Lead compensation: the stage's response trails by dead time + τ, so a
        slowdown commanded at the vertex arrives one lag-length too late."""
        pts = _L()
        cum = VC.polyline_arclength(pts)
        sec = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                       element_um=ELEM).sections[0]
        s_corner = cum[3]
        lead = SPEED * (CHAR.dead_time_s + CHAR.tau_s
                        + 0.5 * CHAR.control_loop_ms / 1000.0)
        self.assertLess(sec.speed_at(s_corner - lead), sec.speed_mm_s,
                        "must already be slowing a lag-length before the corner")

    def test_a_smooth_circle_is_slowed_by_CURVATURE_not_turn_angle(self):
        """A 5 mm circle sampled at 0.5 mm turns only ~11°/vertex — under any
        sane corner threshold — yet its radius absolutely constrains the carrot.
        Gating knots on turn angle left circles at full speed and 48 µm."""
        pts = XC.make_shape("Circle", 5.0)
        sec = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                       element_um=ELEM).sections[0]
        worst_turn = max(VC.corner_turn_angle_deg(pts, i)
                         for i in range(1, len(pts) - 1))
        self.assertLess(worst_turn, FP.CORNER_SLOW_DEG + 5.0,
                        "this shape has no vertex a turn-angle gate would catch")
        self.assertLess(sec.lookahead_at.min_value(), sec.lookahead_mm,
                        "curvature must still shorten the carrot")

    def test_a_straight_line_gets_no_slowdowns(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        sec = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                       element_um=ELEM).sections[0]
        self.assertAlmostEqual(sec.lookahead_at.min_value(), sec.lookahead_mm, 6)
        self.assertIn("straight", sec.reason)

    def test_a_reversal_still_stops_because_geometry_demands_it(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0),
               (1.0, 0.0), (0.0, 0.0)]
        plan = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                        element_um=ELEM)
        self.assertGreaterEqual(plan.n_stops, 1)
        self.assertTrue(any("reversal" in n for n in plan.notes))

    def test_degenerate_input(self):
        for pts in ([], [(0.0, 0.0)], [(1.0, 1.0), (1.0, 1.0)]):
            plan = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                            element_um=ELEM)
            self.assertEqual(plan.sections, [])

    def test_the_time_estimate_accounts_for_the_slowdowns(self):
        pts = _L()
        plan = FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                        element_um=ELEM)
        straight = plan.sections[0].length_mm / plan.sections[0].speed_mm_s
        self.assertGreater(plan.est_time_s, straight,
                           "slowing through the corner must cost time")


class TestSimulatedAccuracy(unittest.TestCase):
    """The whole point: continuous motion must still hold the element.

    Numbers are loose bands — they pin the CONCLUSION (holds the 30 µm element,
    comparably to stopping) without pinning simulator noise.
    """

    SHAPES = (("Square", 5.0), ("Star", 5.0), ("Circle", 5.0),
              ("Circle", 10.0), ("Zigzag", 5.0), ("Comb", 5.0))

    def test_every_shape_holds_the_element_without_stopping(self):
        for name, size in self.SHAPES:
            with self.subTest(shape=name, size=size):
                pts = XC.make_shape(name, size)
                plan = FP.build_continuous_plan(
                    pts, CHAR, target_speed_mm_s=SPEED, element_um=ELEM)
                self.assertEqual(plan.n_stops, 0,
                                 "no cusps here → no stops at all")
                r = FP.simulate_plan(plan, CHAR, resolution_um=ELEM)
                self.assertTrue(r.completed,
                                f"{name} {size}: {r.stopped_reason}")
                self.assertLessEqual(r.report["p95_um"], ELEM,
                                     f"{name} {size} p95 {r.report['p95_um']:.1f}")

    def test_it_is_comparable_to_stopping_on_a_cornered_shape(self):
        pts = XC.make_shape("Star", 5.0)
        stop = FP.simulate_plan(
            FP.build_plan(pts, CHAR, target_speed_mm_s=SPEED, element_um=ELEM),
            CHAR, resolution_um=ELEM)
        slow = FP.simulate_plan(
            FP.build_continuous_plan(pts, CHAR, target_speed_mm_s=SPEED,
                                     element_um=ELEM),
            CHAR, resolution_um=ELEM)
        self.assertLessEqual(slow.report["p95_um"],
                             stop.report["p95_um"] + 5.0,
                             "slowing must not cost meaningful accuracy")

    def test_the_arc_model_alone_would_MISS_the_budget_at_a_sharp_corner(self):
        """Regression on the modelling bug: with only the curvature formula the
        Square simulated ~59 µm against a 30 µm element."""
        pts = XC.make_shape("Square", 5.0)
        delta = (ELEM / 1000.0) / FP.PREDICTION_HEADROOM
        i = max(range(1, len(pts) - 1),
                key=lambda k: VC.corner_turn_angle_deg(pts, k))
        ang = VC.corner_turn_angle_deg(pts, i)
        r = FP.local_radius_mm(pts, i)
        la_arc = FP.LA_CURVE_MARGIN * math.sqrt(2.0 * r * delta)
        cut_if_arc = (la_arc / 2.0) * math.sin(math.radians(ang) / 2.0)
        self.assertGreater(cut_if_arc * 1000.0, delta * 1000.0,
                           "the arc model under-constrains a sharp vertex")


class TestSharedFollowerUnchanged(unittest.TestCase):
    def test_pursuit_step_without_a_profile_is_the_scalar_behaviour(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        cum = VC.polyline_arclength(pts)

        def _limit(_s):
            return 3.0

        a = VC.pursuit_step((0.5, 0.0), pts, cum, VC.PursuitState(),
                            lookahead=0.5, speed_cap_mm_s=3.0,
                            speed_limit_at=_limit, dt=0.03, max_ds=0.5)
        b = VC.pursuit_step((0.5, 0.0), pts, cum, VC.PursuitState(),
                            lookahead=0.5, speed_cap_mm_s=3.0,
                            speed_limit_at=_limit, dt=0.03, max_ds=0.5,
                            lookahead_at=None)
        self.assertEqual(a, b)

    def test_a_profile_overrides_the_scalar_lookahead(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        cum = VC.polyline_arclength(pts)
        seen = []

        def _at(s):
            seen.append(s)
            return 0.05
        VC.pursuit_step((0.5, 0.0), pts, cum, VC.PursuitState(),
                        lookahead=0.9, speed_cap_mm_s=3.0,
                        speed_limit_at=lambda _s: 3.0, dt=0.03, max_ds=0.5,
                        lookahead_at=_at)
        self.assertTrue(seen, "the profile must be consulted")


if __name__ == "__main__":
    unittest.main()
