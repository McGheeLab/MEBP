"""Tests for v7.4.6 Phase 2 additions: group rebuild, line tools,
point-on-line / parallel / perpendicular constraints, downstream
WorkspaceConfig migration."""

from __future__ import annotations

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.PlateDesign import (
    PlateDesign, Well, Point, Group, Line, Constraint,
)
from SupportClasses.PlateSketchSolver import PlateSketchSolver, DOFStatus
from SupportClasses.WellPlate import WellPlate, USER_PLATES_DIR
from SupportClasses.PhysicalModels import WorkspaceConfig
from gui.widgets.plate_designer_canvas import PlateDesignerCanvas, Tool


# ─────────────────────────────────────────────────────────────────
# rebuild_group
# ─────────────────────────────────────────────────────────────────

class TestRebuildGroup(unittest.TestCase):
    def test_grid_rebuild_changes_well_count(self):
        d = PlateDesign.from_standard_format(12)
        groups = [e for e in d.entities.values() if isinstance(e, Group)]
        self.assertEqual(len(groups), 1)
        g = groups[0]
        # Original 12-well = 3×4.
        self.assertEqual(len(g.members), 12)
        new_params = dict(g.params)
        new_params["rows"] = 5
        new_params["cols"] = 8
        d.rebuild_group(g.id, new_params)
        self.assertEqual(len(g.members), 40)
        self.assertEqual(g.params["rows"], 5)
        self.assertEqual(g.params["cols"], 8)

    def test_grid_rebuild_drops_old_constraints_on_members(self):
        d = PlateDesign.from_standard_format(6)
        g = [e for e in d.entities.values() if isinstance(e, Group)][0]
        # Add an extraneous constraint touching one of the old wells.
        old_well_id = g.members[0]
        old_center = d.entities[old_well_id].center
        d.add_constraint(Constraint(kind="ground", refs=[old_center]))
        n_before = len(d.constraints)
        new_params = dict(g.params)
        new_params["rows"] = 2  # still ≥1, but different count
        new_params["cols"] = 2  # 6 → 4 wells
        d.rebuild_group(g.id, new_params)
        # That constraint referenced a now-removed entity → dropped.
        self.assertLess(len(d.constraints), n_before)

    def test_circle_rebuild(self):
        d = PlateDesign(name="r")
        d.ensure_outline_origin()
        g = d.add_circle_pattern(
            count=4, center_x=0.0, center_y=0.0,
            radius=10.0, diameter=4.0)
        self.assertEqual(len(g.members), 4)
        new_params = dict(g.params)
        new_params["count"] = 12
        new_params["radius"] = 20.0
        d.rebuild_group(g.id, new_params)
        self.assertEqual(len(g.members), 12)
        # Verify positions are on a 20mm ring.
        import math
        for mid in g.members:
            w = d.entities[mid]
            x, y = d.get_well_position(w)
            self.assertAlmostEqual(math.hypot(x, y), 20.0, places=6)


# ─────────────────────────────────────────────────────────────────
# Line tool + point-on-line / parallel constraints
# ─────────────────────────────────────────────────────────────────

class TestLineToolAndConstraints(unittest.TestCase):
    def _canvas(self) -> PlateDesignerCanvas:
        c = PlateDesignerCanvas()
        c.set_design(PlateDesign.from_standard_format(6))
        return c

    def test_two_click_line_creates_line_and_two_points(self):
        c = self._canvas()
        n_entities_before = len(c.design.entities)
        c.set_tool(Tool.DRAW_LINE)
        c._handle_draw_line((10.0, 20.0), construction=False)
        c._handle_draw_line((50.0, 20.0), construction=False)
        # 2 new Points + 1 new Line = 3 new entities.
        self.assertEqual(len(c.design.entities), n_entities_before + 3)
        lines = [e for e in c.design.entities.values() if isinstance(e, Line)]
        self.assertEqual(len(lines), 1)
        line = lines[0]
        self.assertFalse(line.construction)
        # Free-line endpoints are NOT fixed.
        self.assertFalse(c.design.entities[line.p1].fixed)
        self.assertFalse(c.design.entities[line.p2].fixed)

    def test_construction_line_fixes_endpoints(self):
        c = self._canvas()
        c.set_tool(Tool.DRAW_CONSTRUCTION_LINE)
        c._handle_draw_line((10.0, 20.0), construction=True)
        c._handle_draw_line((50.0, 20.0), construction=True)
        lines = [e for e in c.design.entities.values() if isinstance(e, Line)]
        line = lines[0]
        self.assertTrue(line.construction)
        # Construction endpoints fixed → solver can't move them.
        self.assertTrue(c.design.entities[line.p1].fixed)
        self.assertTrue(c.design.entities[line.p2].fixed)

    def test_point_on_line_snaps_well(self):
        c = self._canvas()
        # Construction line at y=20.
        c.set_tool(Tool.DRAW_CONSTRUCTION_LINE)
        c._handle_draw_line((10.0, 20.0), construction=True)
        c._handle_draw_line((50.0, 20.0), construction=True)
        line = next(e for e in c.design.entities.values()
                    if isinstance(e, Line))
        # Move a well's center off the line.
        well = c.design.get_wells()[0]
        c.design.entities[well.center].x = 30.0
        c.design.entities[well.center].y = 35.0
        # Apply point_on_line.
        c.add_constraint_explicit(
            "point_on_line", refs=[well.center, line.id])
        # Well's y should snap to ~20.
        self.assertAlmostEqual(
            c.design.entities[well.center].y, 20.0, places=3)

    def test_tangent_cc_circles_touch_externally(self):
        """tangent_cc enforces d(centers) = r1 + r2."""
        d = PlateDesign(name="t")
        w1 = d.add_well(x=0.0, y=0.0, diameter=4.0, name="A")
        w2 = d.add_well(x=10.0, y=0.0, diameter=6.0, name="B")
        # Pin one well so the test focuses on the other.
        d.add_constraint(Constraint(
            kind="fix", refs=[w1.center], snapshot=(0.0, 0.0)))
        # Lock B's y so the only free direction is its x.
        d.add_constraint(Constraint(
            kind="horizontal", refs=[w1.center, w2.center]))
        d.add_constraint(Constraint(
            kind="tangent_cc", refs=[w1.id, w2.id]))
        solver = PlateSketchSolver(d)
        report = solver.solve()
        # Distance(A center, B center) should equal (4+6)/2 = 5
        import math
        c1 = d.entities[w1.center]
        c2 = d.entities[w2.center]
        gap = math.hypot(c1.x - c2.x, c1.y - c2.y)
        self.assertAlmostEqual(gap, 5.0, places=3)

    def test_equal_length_matches_two_lines(self):
        """equal_length pulls a free line's endpoint to match a reference."""
        d = PlateDesign(name="eq")
        a1 = d.add_entity(Point(x=0.0, y=0.0, fixed=True))
        a2 = d.add_entity(Point(x=10.0, y=0.0, fixed=True))
        line_a = d.add_entity(Line(p1=a1.id, p2=a2.id))
        b1 = d.add_entity(Point(x=0.0, y=5.0, fixed=True))
        # Free endpoint horizontally constrained to b1 so length is the
        # only DOF.
        b2 = d.add_entity(Point(x=4.0, y=5.0))
        line_b = d.add_entity(Line(p1=b1.id, p2=b2.id))
        d.add_constraint(Constraint(
            kind="horizontal", refs=[b1.id, b2.id]))
        d.add_constraint(Constraint(
            kind="equal_length", refs=[line_a.id, line_b.id]))
        solver = PlateSketchSolver(d)
        solver.solve()
        # Line B's free endpoint moves so its length matches A (10mm).
        self.assertAlmostEqual(d.entities[b2.id].x, 10.0, places=3)

    def test_symmetric_pp_reflects_across_axis(self):
        """Two free points become reflections of each other across the axis."""
        d = PlateDesign(name="sym")
        # Axis = vertical line at x=10 (both endpoints fixed).
        a1 = d.add_entity(Point(x=10.0, y=0.0, fixed=True))
        a2 = d.add_entity(Point(x=10.0, y=20.0, fixed=True))
        axis = d.add_entity(Line(p1=a1.id, p2=a2.id))
        # Two points off-axis — pin one to a known position so the other
        # is well-determined to be its reflection.
        p_left = d.add_entity(Point(x=5.0, y=8.0, fixed=True))
        p_right = d.add_entity(Point(x=12.0, y=2.0))
        d.add_constraint(Constraint(
            kind="symmetric_pp",
            refs=[p_left.id, p_right.id, axis.id]))
        solver = PlateSketchSolver(d)
        solver.solve()
        # Reflection of (5,8) across vertical x=10 → (15, 8).
        self.assertAlmostEqual(d.entities[p_right.id].x, 15.0, places=3)
        self.assertAlmostEqual(d.entities[p_right.id].y, 8.0, places=3)

    def test_parallel_aligns_two_free_lines(self):
        d = PlateDesign(name="par")
        # Reference line A from (0,0) to (10,0) — both endpoints fixed
        # so it's an immovable direction reference.
        a1 = d.add_entity(Point(x=0.0, y=0.0, fixed=True))
        a2 = d.add_entity(Point(x=10.0, y=0.0, fixed=True))
        line_a = d.add_entity(Line(p1=a1.id, p2=a2.id))
        # Line B starts skew; one endpoint fixed, the other free.
        b1 = d.add_entity(Point(x=0.0, y=5.0, fixed=True))
        b2 = d.add_entity(Point(x=5.0, y=12.0))              # 2 free vars
        line_b = d.add_entity(Line(p1=b1.id, p2=b2.id))
        d.add_constraint(Constraint(
            kind="parallel", refs=[line_a.id, line_b.id]))
        solver = PlateSketchSolver(d)
        report = solver.solve()
        # parallel constrains direction (1 DOF) but b2's length along the
        # line is still free → under-determined with residual ≈ 0 and
        # b2.y pulled to 5 (matching b1.y for horizontal alignment).
        self.assertEqual(report.status, DOFStatus.UNDER_DETERMINED)
        self.assertLess(report.residual_norm, 1e-4)
        self.assertAlmostEqual(d.entities[b2.id].y, 5.0, places=3)


# ─────────────────────────────────────────────────────────────────
# WorkspaceConfig.active_plate_key
# ─────────────────────────────────────────────────────────────────

class TestWorkspaceActivePlateKey(unittest.TestCase):
    def test_default_plate_format_int(self):
        ws = WorkspaceConfig()
        self.assertEqual(ws.active_plate_key, 24)

    def test_plate_name_takes_precedence(self):
        ws = WorkspaceConfig()
        ws.plate_name = "my-coverslip-array"
        self.assertEqual(ws.active_plate_key, "my-coverslip-array")

    def test_round_trip_serialization(self):
        ws = WorkspaceConfig()
        ws.plate_name = "custom-foo"
        d = ws.to_dict()
        self.assertEqual(d.get("plate_name"), "custom-foo")
        ws2 = WorkspaceConfig.from_dict(d)
        self.assertEqual(ws2.plate_name, "custom-foo")
        self.assertEqual(ws2.active_plate_key, "custom-foo")


if __name__ == "__main__":
    unittest.main()
