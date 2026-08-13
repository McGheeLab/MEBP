"""
A nosepiece rotation must never happen with the objective inside the plate.

THE LOAD-BEARING TEST is ``TestRetreatBeforeRotateIsAlwaysAwayFromGlass`` — a
property test over randomised (current focus, glass focus, working distance,
polarity) combinations asserting the planned focus move is NEVER toward the
specimen, mirroring the existing 400-randomised-plan precedent in
``test_v711_focus_sweep_planner``.

Why this is the sharpest edge in the whole optics feature: the focus drive raises
the objective TOWARD the specimen and working distance varies enormously (this
rig's 4x reports 16.4 mm, a 20x about 1 mm), but **rotating does not move Z**. So a
focus height that is comfortable under the 4x can be *inside* the 20x's front lens
the instant the turret turns, and there is no move in flight to abort. The guard
has to be computed BEFORE anything is commanded, which is why it lives in pure
code with no hardware in the loop.

The counterweight is ``TestItDoesNotOverReact``: a guard that retreats on every
rotation would make the objective ladder useless, so a rotation that is already
clear must plan no focus move at all.
"""

import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.FocusSweepPlanner import (            # noqa: E402
    TurretChangePlan, plan_turret_change)
from SupportClasses.ObjectiveOptics import (              # noqa: E402
    NO_WD_FALLBACK_HALF_RANGE_UM, ObjectiveOptics, WD_SWEEP_FRACTION)

# This rig's real objectives (WD in mm, from the body on 2026-08-12).
FOUR_X = ObjectiveOptics(label="4X", position=1, magnification=4.0,
                         numerical_aperture=0.13, working_distance_mm=16.4)
TEN_X = ObjectiveOptics(label="10X", position=2, magnification=10.0,
                        numerical_aperture=0.30, working_distance_mm=4.0)
TWENTY_X = ObjectiveOptics(label="20x", position=3, magnification=20.0,
                           numerical_aperture=0.45, working_distance_mm=1.0)
NO_WD = ObjectiveOptics(label="mystery", position=4, magnification=40.0,
                        numerical_aperture=0.6, working_distance_mm=None)

GLASS = 5000.0      # focus height at which the plate glass is sharp


def _clearance(optics, focus_um, glass_um, up=True):
    """The gap this objective's front lens would have, µm. The model under test."""
    u = 1.0 if up else -1.0
    return optics.working_distance_mm * 1000.0 - u * (focus_um - glass_um)


class TestRetreatBeforeRotateIsAlwaysAwayFromGlass(unittest.TestCase):
    """Property: the planned move never reduces clearance. 600 combinations."""

    def test_over_randomised_geometry(self):
        rng = random.Random(20260812)
        checked = 0
        for _ in range(600):
            up = rng.choice((True, False))
            u = 1.0 if up else -1.0
            to_optics = rng.choice((FOUR_X, TEN_X, TWENTY_X))
            # Anywhere within the widest objective's reach, both sides of glass.
            focus = GLASS + u * rng.uniform(-5000.0, 5000.0)
            plan, why = plan_turret_change(
                to_optics=to_optics, current_focus_um=focus,
                glass_focus_um=GLASS, focus_up_is_positive=up,
                soft_limits_um=(0.0, 10000.0))
            if plan is None:
                # A refusal is always acceptable; a wrong MOVE is not.
                self.assertTrue(why)
                continue
            checked += 1
            if plan.retreat_focus_um is None:
                continue
            moved_toward_glass = u * (plan.retreat_focus_um - focus)
            self.assertLessEqual(
                moved_toward_glass, 1e-6,
                f"planned focus move went TOWARD the specimen: {focus:.0f} → "
                f"{plan.retreat_focus_um:.0f} µm (up={up}, {to_optics.label})")
        self.assertGreater(checked, 400, "the property barely exercised anything")

    def test_it_never_leaves_less_clearance_than_it_found(self):
        rng = random.Random(7)
        for _ in range(300):
            up = rng.choice((True, False))
            u = 1.0 if up else -1.0
            to_optics = rng.choice((FOUR_X, TEN_X, TWENTY_X))
            focus = GLASS + u * rng.uniform(-4000.0, 4000.0)
            plan, _ = plan_turret_change(
                to_optics=to_optics, current_focus_um=focus,
                glass_focus_um=GLASS, focus_up_is_positive=up,
                soft_limits_um=(0.0, 10000.0))
            if plan is None or plan.retreat_focus_um is None:
                continue
            before = _clearance(to_optics, focus, GLASS, up)
            after = _clearance(to_optics, plan.retreat_focus_um, GLASS, up)
            self.assertGreaterEqual(after, before - 1e-6)


class TestTheTargetObjectiveBoundsIt(unittest.TestCase):
    """The bound is the objective being rotated IN, not the one going out."""

    def test_a_4x_height_that_is_lethal_for_a_20x_forces_a_retreat(self):
        """3 mm toward the glass is fine under a 16.4 mm WD and 2 mm INSIDE a
        1 mm one. This is the case the whole guard exists for."""
        focus = GLASS + 3000.0
        self.assertGreater(_clearance(FOUR_X, focus, GLASS), 13000.0)
        self.assertLess(_clearance(TWENTY_X, focus, GLASS), 0.0)

        plan, why = plan_turret_change(
            to_optics=TWENTY_X, from_optics=FOUR_X, current_focus_um=focus,
            glass_focus_um=GLASS, soft_limits_um=(0.0, 10000.0))
        self.assertIsNotNone(plan, why)
        self.assertIsNotNone(plan.retreat_focus_um)
        self.assertLess(plan.retreat_focus_um, focus)
        self.assertGreater(plan.clearance_after_um, 0.0)

    def test_the_same_height_needs_no_retreat_going_the_other_way(self):
        """20x → 4x only ever increases clearance."""
        plan, why = plan_turret_change(
            to_optics=FOUR_X, from_optics=TWENTY_X,
            current_focus_um=GLASS + 3000.0, glass_focus_um=GLASS,
            soft_limits_um=(0.0, 10000.0))
        self.assertIsNotNone(plan, why)
        self.assertIsNone(plan.retreat_focus_um)

    def test_the_allowance_scales_with_the_target_working_distance(self):
        """Bounded by WD_SWEEP_FRACTION of the TARGET objective's WD, so the 4x
        tolerates 16x more approach than the 20x."""
        for optics in (FOUR_X, TEN_X, TWENTY_X):
            budget = WD_SWEEP_FRACTION * optics.working_distance_mm * 1000.0
            just_inside, _ = plan_turret_change(
                to_optics=optics, current_focus_um=GLASS + budget * 0.9,
                glass_focus_um=GLASS, soft_limits_um=(0.0, 10000.0))
            just_outside, _ = plan_turret_change(
                to_optics=optics, current_focus_um=GLASS + budget * 1.1,
                glass_focus_um=GLASS, soft_limits_um=(0.0, 10000.0))
            self.assertIsNone(just_inside.retreat_focus_um,
                              f"{optics.label}: inside the budget must not move")
            self.assertIsNotNone(just_outside.retreat_focus_um,
                                 f"{optics.label}: outside the budget must move")

    def test_the_4x_budget_is_16x_the_20x_budget(self):
        """The concrete numbers, so a silent policy change is visible."""
        self.assertAlmostEqual(WD_SWEEP_FRACTION * 16400.0, 4100.0, places=3)
        self.assertAlmostEqual(WD_SWEEP_FRACTION * 1000.0, 250.0, places=3)


class TestPolarityIsReadNeverAssumed(unittest.TestCase):
    def test_an_inverted_focus_axis_retreats_the_other_way(self):
        """With focus_up_is_positive False, 'toward the specimen' is DECREASING
        focus, so the retreat must increase it. A hard-coded sign would drive the
        objective into the plate on such a rig."""
        focus = GLASS - 3000.0        # 3 mm toward the specimen when up = -1
        plan, why = plan_turret_change(
            to_optics=TWENTY_X, current_focus_um=focus, glass_focus_um=GLASS,
            focus_up_is_positive=False, soft_limits_um=(0.0, 10000.0))
        self.assertIsNotNone(plan, why)
        self.assertIsNotNone(plan.retreat_focus_um)
        self.assertGreater(plan.retreat_focus_um, focus)


class TestRefusalsThatMustStayRefusals(unittest.TestCase):
    def test_no_glass_datum_refuses_rather_than_guessing(self):
        """Without it a focus reading cannot be turned into a gap at all."""
        plan, why = plan_turret_change(
            to_optics=TWENTY_X, current_focus_um=GLASS + 3000.0,
            glass_focus_um=None)
        self.assertIsNone(plan)
        self.assertIn("glass", why.lower())

    def test_an_unknown_working_distance_is_conservative_AND_says_so(self):
        """A silently narrow bound reads downstream as an optics fault."""
        budget = NO_WD_FALLBACK_HALF_RANGE_UM
        plan, why = plan_turret_change(
            to_optics=NO_WD, current_focus_um=GLASS + budget * 4,
            glass_focus_um=GLASS, soft_limits_um=(0.0, 10000.0))
        self.assertIsNotNone(plan, why)
        self.assertFalse(plan.wd_known)
        self.assertIsNotNone(plan.retreat_focus_um)
        self.assertTrue(any("working distance" in r for r in plan.reasons))
        self.assertIn("UNKNOWN", plan.describe())

    def test_soft_limits_that_forbid_a_safe_retreat_refuse(self):
        """Better to refuse than to rotate at the closest legal-but-unsafe spot."""
        plan, why = plan_turret_change(
            to_optics=TWENTY_X, current_focus_um=9000.0, glass_focus_um=GLASS,
            soft_limits_um=(8000.0, 10000.0))
        self.assertIsNone(plan)
        self.assertIn("soft limits", why)

    def test_an_unreadable_focus_refuses(self):
        plan, why = plan_turret_change(
            to_optics=TWENTY_X, current_focus_um=None, glass_focus_um=GLASS)
        self.assertIsNone(plan)
        self.assertIn("number", why)


class TestItDoesNotOverReact(unittest.TestCase):
    """A guard that fires on every rotation would make the ladder unusable."""

    def test_at_the_focused_plane_nothing_moves(self):
        for optics in (FOUR_X, TEN_X, TWENTY_X):
            plan, why = plan_turret_change(
                to_optics=optics, current_focus_um=GLASS, glass_focus_um=GLASS,
                soft_limits_um=(0.0, 10000.0))
            self.assertIsNotNone(plan, why)
            self.assertIsNone(plan.retreat_focus_um, optics.label)

    def test_already_further_away_than_needed_never_moves_closer(self):
        plan, why = plan_turret_change(
            to_optics=TWENTY_X, current_focus_um=GLASS - 4000.0,
            glass_focus_um=GLASS, soft_limits_um=(0.0, 10000.0))
        self.assertIsNotNone(plan, why)
        self.assertIsNone(plan.retreat_focus_um)


class TestParfocalIsRelativeNotAbsolute(unittest.TestCase):
    def test_the_delta_is_carried_onto_the_post_switch_target(self):
        plan, _ = plan_turret_change(
            to_optics=FOUR_X, current_focus_um=GLASS, glass_focus_um=GLASS,
            parfocal_delta_um=-180.0, soft_limits_um=(0.0, 10000.0))
        self.assertAlmostEqual(plan.post_focus_um, GLASS - 180.0)
        self.assertAlmostEqual(plan.parfocal_applied_um, -180.0)

    def test_with_a_retreat_the_delta_applies_to_the_RETREATED_position(self):
        """Applying it to the pre-retreat height would undo the retreat."""
        plan, _ = plan_turret_change(
            to_optics=TWENTY_X, current_focus_um=GLASS + 3000.0,
            glass_focus_um=GLASS, parfocal_delta_um=-20.0,
            soft_limits_um=(0.0, 10000.0))
        self.assertIsNotNone(plan.retreat_focus_um)
        self.assertAlmostEqual(plan.post_focus_um,
                               plan.retreat_focus_um - 20.0)

    def test_no_delta_leaves_the_focus_alone(self):
        plan, _ = plan_turret_change(
            to_optics=FOUR_X, current_focus_um=GLASS, glass_focus_um=GLASS,
            soft_limits_um=(0.0, 10000.0))
        self.assertIsNone(plan.post_focus_um)


class TestDescribe(unittest.TestCase):
    def test_it_names_both_objectives_and_the_clearance(self):
        plan, _ = plan_turret_change(
            to_optics=TWENTY_X, from_optics=FOUR_X,
            current_focus_um=GLASS + 3000.0, glass_focus_um=GLASS,
            soft_limits_um=(0.0, 10000.0))
        text = plan.describe()
        self.assertIn("4X", text)
        self.assertIn("20x", text)
        self.assertIn("retreat", text)

    def test_a_clear_rotation_says_so(self):
        plan, _ = plan_turret_change(
            to_optics=FOUR_X, current_focus_um=GLASS, glass_focus_um=GLASS)
        self.assertIn("already clear", plan.describe())
        self.assertFalse(plan.moves_focus)


if __name__ == "__main__":
    unittest.main()
