"""Tests for v7.4.5 PlateSketchSolver."""

from __future__ import annotations

import math
import unittest

from SupportClasses.PlateDesign import (
    PlateDesign, Point, Well, Constraint,
)
from SupportClasses.PlateSketchSolver import (
    PlateSketchSolver, SolveReport, DOFStatus,
)


def _design_two_wells() -> tuple[PlateDesign, Well, Well, int]:
    """Tiny test design: origin grounded, two wells, A1 coincident with origin."""
    d = PlateDesign(name="tiny")
    origin = d.ensure_outline_origin()      # adds ground constraint at (0,0)
    w1 = d.add_well(x=0.0, y=0.0, diameter=6.0, name="A1",
                    naming_scheme="MANUAL")
    w2 = d.add_well(x=15.0, y=0.0, diameter=6.0, name="A2",
                    naming_scheme="MANUAL")
    d.add_constraint(Constraint(
        kind="coincident_pp", refs=[w1.center, origin.id]))
    return d, w1, w2, origin.id


class TestStaticSolve(unittest.TestCase):
    def test_horizontal_plus_distance_satisfied(self):
        d, w1, w2, origin = _design_two_wells()
        d.add_constraint(Constraint(
            kind="distance_pp", refs=[w1.center, w2.center], value=20.0))
        d.add_constraint(Constraint(
            kind="horizontal", refs=[w1.center, w2.center]))

        solver = PlateSketchSolver(d)
        r = solver.solve()
        self.assertEqual(r.status, DOFStatus.WELL_DETERMINED)
        # w2 ends up 20mm along +X (horizontal axis) from origin.
        c1 = d.entities[w1.center]
        c2 = d.entities[w2.center]
        self.assertAlmostEqual(c1.x, 0.0, places=6)
        self.assertAlmostEqual(c1.y, 0.0, places=6)
        self.assertAlmostEqual(c2.x, 20.0, places=4)
        self.assertAlmostEqual(c2.y, 0.0, places=4)

    def test_under_determined_no_constraints(self):
        """Bare wells (only ground on origin) → underdetermined."""
        d, w1, w2, _ = _design_two_wells()
        solver = PlateSketchSolver(d)
        r = solver.solve()
        self.assertEqual(r.status, DOFStatus.UNDER_DETERMINED)
        # Positions unchanged.
        self.assertAlmostEqual(d.entities[w2.center].x, 15.0)

    def test_inconsistent_constraints(self):
        """Distance and explicit position conflict → over-constrained."""
        d, w1, w2, origin = _design_two_wells()
        # Move w2 to (5, 0) and lock; then ask distance(w1, w2) = 20.
        # Lock requires snapshot.
        d.entities[w2.center].x = 5.0
        d.add_constraint(Constraint(
            kind="fix", refs=[w2.center], snapshot=(5.0, 0.0)))
        d.add_constraint(Constraint(
            kind="distance_pp", refs=[w1.center, w2.center], value=20.0))
        solver = PlateSketchSolver(d)
        r = solver.solve()
        self.assertEqual(r.status, DOFStatus.INCONSISTENT)
        self.assertGreater(len(r.conflicts), 0)


class TestDragLifecycle(unittest.TestCase):
    def test_drag_pulls_well_to_target(self):
        d, w1, w2, _ = _design_two_wells()
        solver = PlateSketchSolver(d)
        solver.begin_drag(w2.center, (40.0, 30.0))
        c2 = d.entities[w2.center]
        # Drag weight (1000) dominates with no other constraints → lands on target.
        self.assertAlmostEqual(c2.x, 40.0, places=3)
        self.assertAlmostEqual(c2.y, 30.0, places=3)

        # Update drag.
        solver.update_drag((10.0, 5.0))
        c2 = d.entities[w2.center]
        self.assertAlmostEqual(c2.x, 10.0, places=3)
        self.assertAlmostEqual(c2.y, 5.0, places=3)

        # End drag with a distance constraint pulling it back to 20mm @ origin.
        d.add_constraint(Constraint(
            kind="distance_pp", refs=[w1.center, w2.center], value=20.0))
        d.add_constraint(Constraint(
            kind="horizontal", refs=[w1.center, w2.center]))
        report = solver.end_drag()
        self.assertEqual(report.status, DOFStatus.WELL_DETERMINED)

    def test_drag_under_constrained_translates_linked_wells(self):
        """Two wells with only a distance constraint between them — drag w1
        should keep w2 at the linked distance."""
        d = PlateDesign(name="link")
        w1 = d.add_well(x=0.0, y=0.0, diameter=6.0, name="A",
                        naming_scheme="MANUAL")
        w2 = d.add_well(x=10.0, y=0.0, diameter=6.0, name="B",
                        naming_scheme="MANUAL")
        d.add_constraint(Constraint(
            kind="distance_pp", refs=[w1.center, w2.center], value=10.0))
        d.add_constraint(Constraint(
            kind="horizontal", refs=[w1.center, w2.center]))
        solver = PlateSketchSolver(d)
        solver.begin_drag(w1.center, (50.0, 0.0))
        c1, c2 = d.entities[w1.center], d.entities[w2.center]
        # w1 should be near the cursor; w2 should remain ~10mm away horizontally.
        self.assertAlmostEqual(c1.x, 50.0, places=2)
        self.assertAlmostEqual(abs(c1.x - c2.x), 10.0, places=2)
        self.assertAlmostEqual(c1.y, c2.y, places=3)


class TestEqualRadiusPreSolve(unittest.TestCase):
    def test_equal_radius_propagates_leader_diameter(self):
        d = PlateDesign(name="r")
        w1 = d.add_well(x=0.0, y=0.0, diameter=8.0, name="A")
        w2 = d.add_well(x=10.0, y=0.0, diameter=4.0, name="B")
        d.add_constraint(Constraint(
            kind="equal_radius", refs=[w1.id, w2.id]))
        solver = PlateSketchSolver(d)
        solver.solve()
        self.assertAlmostEqual(d.entities[w2.id].diameter, 8.0)


if __name__ == "__main__":
    unittest.main()
