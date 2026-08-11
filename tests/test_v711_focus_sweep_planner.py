"""
Focus sweep planning — and the collision guard, as a property.

The focus drive raises the objective toward the plate. Every position this module
plans must sit inside the working-distance budget AND inside the operator's soft
limits, including the backlash lead-in position that precedes it. That is asserted
here as a property over randomised inputs rather than as a handful of examples,
because it is the guard standing between an autofocus sweep and the front lens.
"""

import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.ObjectiveOptics import (        # noqa: E402
    ObjectiveOptics, wd_bounded_half_range_um, depth_of_field_um)
from SupportClasses.FocusSweepPlanner import (      # noqa: E402
    plan_sweep, plan_handoff, next_rung, choose_ladder, sweep_targets_um,
    estimate_wall_time_s, BRACKET_SHRINK, COARSE_STEP_DOF_MULT,
    BACKLASH_LEADIN_UM,
)


def optic(label, na, wd, upp):
    return ObjectiveOptics(label=label, numerical_aperture=na,
                           working_distance_mm=wd, um_per_px_sample=upp,
                           frame_wh=(2600, 2048))


O4 = optic("4x", 0.13, 16.4, 3.20)
O10 = optic("10x", 0.30, 16.0, 1.28)
O20 = optic("20x", 0.45, 1.0, 0.64)
HW = (0.0, 10000.0)


class TestCollisionGuardProperty(unittest.TestCase):
    def test_every_planned_position_respects_every_bound(self):
        """THE safety property. 400 randomised plans; zero tolerated violations."""
        random.seed(3)
        soft = (200.0, 9000.0)
        checked = 0
        for _ in range(400):
            o = random.choice([O4, O10, O20])
            centre = random.uniform(300.0, 8800.0)
            asked = random.uniform(1.0, 3000.0)
            plan, why = plan_sweep(optics=o, center_um=centre,
                                   requested_half_range_um=asked,
                                   focus_limits_um=HW, soft_limits_um=soft)
            if plan is None:
                continue
            wd_half, _known = wd_bounded_half_range_um(o)
            bound = min(asked, wd_half)
            for rung in plan.rungs:
                for z in rung.z_targets_um:
                    checked += 1
                    self.assertLessEqual(abs(z - rung.center_um), bound + 1e-6)
                    self.assertGreaterEqual(z, soft[0] - 1e-6)
                    self.assertLessEqual(z, soft[1] + 1e-6)
                    # the backlash approach must be legal too
                    self.assertGreaterEqual(rung.approach_um(z),
                                            soft[0] - 1e-6)
        self.assertGreater(checked, 1000)

    def test_short_working_distance_clamps_and_explains(self):
        plan, _ = plan_sweep(optics=O20, center_um=5000.0,
                             requested_half_range_um=1000.0,
                             focus_limits_um=HW)
        self.assertTrue(plan.clamped)
        self.assertAlmostEqual(plan.rungs[0].half_range_um, 250.0)
        self.assertTrue(any("working distance" in r for r in plan.clamp_reasons))

    def test_unknown_working_distance_says_so_in_the_reason(self):
        o = ObjectiveOptics(label="?", numerical_aperture=0.45,
                            um_per_px_sample=0.64, frame_wh=(2600, 2048))
        plan, _ = plan_sweep(optics=o, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        self.assertTrue(plan.clamped)
        self.assertTrue(any("no working distance" in r
                            for r in plan.clamp_reasons))


class TestEmptyPlanRaises(unittest.TestCase):
    def test_impossible_bounds_raise_rather_than_return_nothing(self):
        """A zero-sample sweep would be reported downstream as 'no peak found' —
        an optics diagnosis for what is really a limits problem."""
        with self.assertRaises(ValueError) as ctx:
            sweep_targets_um(center_um=100.0, half_range_um=5.0, step_um=1.0,
                             soft_lo=500.0, soft_hi=9000.0)
        self.assertIn("no room", str(ctx.exception))

    def test_plan_sweep_surfaces_that_as_a_refusal(self):
        plan, why = plan_sweep(optics=O10, center_um=100.0,
                               requested_half_range_um=50.0,
                               focus_limits_um=(500.0, 9000.0))
        self.assertIsNone(plan)
        self.assertTrue(why)

    def test_zero_step_raises(self):
        with self.assertRaises(ValueError):
            sweep_targets_um(center_um=100.0, half_range_um=5.0, step_um=0.0)


class TestLadderGeometry(unittest.TestCase):
    def test_range_shrinks_by_twice_the_previous_step(self):
        plan, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        for a, b in zip(plan.rungs, plan.rungs[1:]):
            self.assertAlmostEqual(b.half_range_um,
                                   BRACKET_SHRINK * a.step_um, places=6)

    def test_final_step_is_a_useful_fraction_of_depth_of_field(self):
        plan, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        self.assertLess(plan.rungs[-1].step_over_dof, 0.5)

    def test_coarse_step_never_exceeds_the_under_sampling_cap(self):
        """Above ~8 DOF a real peak can hide between samples."""
        for o in (O4, O10, O20):
            plan, _ = plan_sweep(optics=o, center_um=5000.0,
                                 requested_half_range_um=3000.0,
                                 focus_limits_um=HW)
            if plan is None:
                continue
            self.assertLessEqual(plan.rungs[0].step_over_dof,
                                 COARSE_STEP_DOF_MULT + 1e-6)

    def test_every_rung_carries_a_backlash_lead_in(self):
        """Each rung boundary reverses direction; without a common approach
        direction the rungs carry backlash with opposite signs."""
        plan, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        for r in plan.rungs:
            self.assertEqual(r.lead_in_um, BACKLASH_LEADIN_UM)
            for z in r.z_targets_um:
                self.assertAlmostEqual(r.approach_um(z), z - BACKLASH_LEADIN_UM)

    def test_choose_ladder_reaches_the_requested_ratio(self):
        for ratio in (5.0, 50.0, 500.0, 5000.0):
            K, m = choose_ladder(ratio)
            reach = K * (K / BRACKET_SHRINK) ** (m - 1)
            self.assertGreaterEqual(reach, ratio)

    def test_wall_time_grows_with_frames(self):
        plan, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        self.assertGreater(plan.estimated_seconds, 0)
        self.assertAlmostEqual(
            plan.estimated_seconds,
            estimate_wall_time_s(plan.rungs, settle_s=0.30), places=6)


class TestRecentringOnMeasurement(unittest.TestCase):
    def test_next_rung_recentres_on_the_measured_peak(self):
        """Re-centring on the measurement rather than the nominal centre is what
        lets the range shrink safely."""
        plan, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        nxt = next_rung(plan=plan, rung_index=0, measured_peak_um=5123.4,
                        focus_limits_um=HW)
        self.assertIsNotNone(nxt)
        self.assertAlmostEqual(nxt.center_um, 5123.4)
        mid = 0.5 * (nxt.z_targets_um[0] + nxt.z_targets_um[-1])
        self.assertAlmostEqual(mid, 5123.4, delta=nxt.step_um)

    def test_next_rung_past_the_end_is_none(self):
        plan, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        self.assertIsNone(next_rung(plan=plan, rung_index=len(plan.rungs) - 1,
                                    measured_peak_um=5000.0))


class TestHandoff(unittest.TestCase):
    def test_measured_parfocal_offset_moves_the_centre(self):
        """This is what turns the escalation from a search into a direct move."""
        plan, why = plan_handoff(to_optics=O10, prior_peak_um=5000.0,
                                 prior_sigma_um=9.5,
                                 parfocal_delta_um=-128.4, focus_limits_um=HW)
        self.assertIsNotNone(plan, why)
        self.assertAlmostEqual(plan.rungs[0].center_um, 4871.6, places=3)

    def test_unknown_parfocality_widens_the_search(self):
        known, _ = plan_handoff(to_optics=O10, prior_peak_um=5000.0,
                                prior_sigma_um=9.5, parfocal_delta_um=-128.4,
                                focus_limits_um=HW)
        unknown, _ = plan_handoff(to_optics=O10, prior_peak_um=5000.0,
                                  prior_sigma_um=9.5, parfocal_delta_um=0.0,
                                  focus_limits_um=HW)
        self.assertGreater(unknown.rungs[0].half_range_um,
                           known.rungs[0].half_range_um)

    def test_handoff_still_respects_the_working_distance(self):
        plan, _ = plan_handoff(to_optics=O20, prior_peak_um=5000.0,
                               prior_sigma_um=400.0, parfocal_delta_um=0.0,
                               focus_limits_um=HW)
        if plan is not None:
            wd_half, _ = wd_bounded_half_range_um(O20)
            for r in plan.rungs:
                for z in r.z_targets_um:
                    self.assertLessEqual(abs(z - r.center_um), wd_half + 1e-6)


class TestPriorNarrowsOnly(unittest.TestCase):
    def test_a_confident_prior_shrinks_the_first_rung(self):
        wide, _ = plan_sweep(optics=O4, center_um=5000.0,
                             requested_half_range_um=1000.0, focus_limits_um=HW)
        tight, _ = plan_sweep(optics=O4, center_um=5000.0,
                              requested_half_range_um=1000.0,
                              prior_sigma_um=5.0, focus_limits_um=HW)
        self.assertLess(tight.rungs[0].half_range_um,
                        wide.rungs[0].half_range_um)
        self.assertLess(tight.total_frames, wide.total_frames)

    def test_a_prior_can_never_widen_past_the_working_distance(self):
        plan, _ = plan_sweep(optics=O20, center_um=5000.0,
                             requested_half_range_um=1000.0,
                             prior_sigma_um=10000.0, focus_limits_um=HW)
        self.assertLessEqual(plan.rungs[0].half_range_um, 250.0 + 1e-6)


if __name__ == "__main__":
    unittest.main()
