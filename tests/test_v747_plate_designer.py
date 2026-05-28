"""Tests for v7.4.7 plate-designer additions: wheel-zoom toggle, blank
plate, edge-distance dimensions, and the nested rosette designer."""

from __future__ import annotations

import json
import math
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.PlateDesign import (
    PlateDesign, Well, Point, Constraint,
)
from SupportClasses.PlateSketchSolver import PlateSketchSolver, DOFStatus
from SupportClasses.PhysicalModels import RosetteInsert
from gui.widgets.plate_designer_canvas import PlateDesignerCanvas, Tool


# ─────────────────────────────────────────────────────────────────
# Feature 1 — wheel zoom toggle
# ─────────────────────────────────────────────────────────────────

class TestWheelZoomToggle(unittest.TestCase):
    def test_default_enabled(self):
        c = PlateDesignerCanvas()
        self.assertTrue(c._wheel_zoom_enabled)

    def test_setter(self):
        c = PlateDesignerCanvas()
        c.set_wheel_zoom_enabled(False)
        self.assertFalse(c._wheel_zoom_enabled)
        c.set_wheel_zoom_enabled(True)
        self.assertTrue(c._wheel_zoom_enabled)


# ─────────────────────────────────────────────────────────────────
# Feature 2 — blank plate
# ─────────────────────────────────────────────────────────────────

class TestBlankPlate(unittest.TestCase):
    def test_blank_has_no_wells_but_has_ground_origin(self):
        d = PlateDesign.blank()
        self.assertEqual(len(d.get_wells()), 0)
        self.assertEqual(d.outline.kind, "rect")
        # A ground constraint anchors the origin.
        self.assertTrue(any(c.kind == "ground" for c in d.constraints))

    def test_blank_compile_raises_until_well_added(self):
        d = PlateDesign.blank()
        with self.assertRaises(ValueError):
            d.compile()
        d.add_well(x=10.0, y=10.0, diameter=6.0, name="X1")
        plate = d.compile()           # no longer raises
        self.assertEqual(len(plate.get_all_wells()), 1)


# ─────────────────────────────────────────────────────────────────
# Feature 3 — edge-distance dimensions
# ─────────────────────────────────────────────────────────────────

class TestEdgeDimensions(unittest.TestCase):
    def test_center_mode_positions_well(self):
        d = PlateDesign.from_standard_format(96)  # a1_offset 14.38 / 11.24
        w = d.get_wells()[0]
        d.add_constraint(Constraint(
            kind="dist_left_edge", refs=[w.id], value=20.0, mode="center"))
        d.add_constraint(Constraint(
            kind="dist_top_edge", refs=[w.id], value=15.0, mode="center"))
        PlateSketchSolver(d).solve()
        c = d.entities[w.center]
        self.assertAlmostEqual(c.x, 20.0 - 14.38, places=3)
        self.assertAlmostEqual(c.y, 15.0 - 11.24, places=3)

    def test_edge_mode_adds_radius(self):
        d = PlateDesign.from_standard_format(96)  # well Ø 6.35 → r 3.175
        w = d.get_wells()[0]
        d.add_constraint(Constraint(
            kind="dist_left_edge", refs=[w.id], value=20.0, mode="edge"))
        PlateSketchSolver(d).solve()
        c = d.entities[w.center]
        # center sits radius further in than the edge dimension.
        self.assertAlmostEqual(c.x, 20.0 - 14.38 + 3.175, places=3)

    def test_two_edge_dims_fully_determine_well(self):
        d = PlateDesign.from_standard_format(24)
        # Isolate one free well: ground origin already exists; pick a well
        # and give it both edge dims.
        w = d.get_wells()[5]
        d.add_constraint(Constraint(
            kind="dist_left_edge", refs=[w.id], value=30.0))
        d.add_constraint(Constraint(
            kind="dist_top_edge", refs=[w.id], value=25.0))
        rep = PlateSketchSolver(d).solve()
        self.assertLess(rep.residual_norm, 1e-4)

    def test_mode_round_trips(self):
        c = Constraint(kind="dist_left_edge", refs=[1], value=10.0, mode="edge")
        c2 = Constraint.from_dict(json.loads(json.dumps(c.to_dict())))
        self.assertEqual(c2.mode, "edge")
        self.assertEqual(c2.value, 10.0)

    def test_dimension_tool_adds_both_dims(self):
        from PySide6.QtCore import QPointF
        from gui.widgets.plate_designer_canvas import SCALE_FACTOR
        c = PlateDesignerCanvas()
        c.set_design(PlateDesign.from_standard_format(96))
        w = c.design.get_wells()[0]
        center = c.design.entities[w.center]
        c.set_tool(Tool.DIMENSION)
        c._handle_dimension(
            QPointF(center.x * SCALE_FACTOR, center.y * SCALE_FACTOR))
        kinds = sorted(
            cc.kind for cc in c.design.constraints
            if cc.kind in ("dist_left_edge", "dist_top_edge"))
        self.assertEqual(kinds, ["dist_left_edge", "dist_top_edge"])

    def test_dimension_edit_moves_well(self):
        c = PlateDesignerCanvas()
        c.set_design(PlateDesign.from_standard_format(96))
        w = c.design.get_wells()[0]
        c.design.add_constraint(Constraint(
            kind="dist_left_edge", refs=[w.id], value=14.38, mode="center"))
        left = c.design.constraints[-1]
        c.set_constraint_value(left.id, 25.0)
        self.assertAlmostEqual(
            c.design.entities[w.center].x, 25.0 - 14.38, places=2)


# ─────────────────────────────────────────────────────────────────
# Feature 4 — rosette designer
# ─────────────────────────────────────────────────────────────────

class TestRosetteDesigner(unittest.TestCase):
    def test_blank_rosette_circular_outline(self):
        ros = PlateDesign.blank_rosette(bore_radius_mm=3.2)
        self.assertEqual(ros.outline.kind, "circle")
        self.assertAlmostEqual(ros.outline.radius, 3.2)
        self.assertEqual(len(ros.get_wells()), 0)

    def test_to_rosette_insert_ring_plus_center(self):
        ros = PlateDesign.blank_rosette(bore_radius_mm=3.2)
        ros.add_circle_pattern(
            count=6, center_x=0.0, center_y=0.0, radius=2.0,
            diameter=0.8, group_name="ring")
        ros.add_well(x=0.0, y=0.0, diameter=0.8, name="c",
                     naming_scheme="MANUAL")
        ri = ros.to_rosette_insert(name="r", well_format=96)
        self.assertIsInstance(ri, RosetteInsert)
        self.assertEqual(ri.num_subwells, 6)           # ring count (excl center)
        self.assertTrue(ri.has_center_well)
        self.assertAlmostEqual(ri.ring_radius_mm, 2.0, places=3)
        # all ring subwells at radius 2.0
        ring = [s for s in ri.subwells if s.radial_offset_mm > 1e-6]
        for s in ring:
            self.assertAlmostEqual(s.radial_offset_mm, 2.0, places=3)

    def test_angle_convention_matches_rosette_insert(self):
        """A well at +X (3 o'clock) maps to angle 90° (0° = +Y)."""
        ros = PlateDesign.blank_rosette(bore_radius_mm=3.0)
        ros.add_well(x=2.0, y=0.0, diameter=0.5, name="e",
                     naming_scheme="MANUAL")
        ri = ros.to_rosette_insert()
        self.assertAlmostEqual(ri.subwells[0].angle_deg, 90.0, places=3)

    def test_compile_flattens_rosette_to_subwells(self):
        # v7.4.8: compile() now FLATTENS rosettes into named sub-wells
        # (A1.a …) and drops the parent, instead of attaching a
        # RosetteInsert to the parent's WellInfo.
        parent = PlateDesign.from_standard_format(24)
        w0 = parent.get_wells()[0]   # "A1"
        ros = PlateDesign.blank_rosette(bore_radius_mm=5.0)
        ros.add_well(x=2.0, y=0.0, diameter=1.0, name="a",
                     naming_scheme="MANUAL")
        ros.add_well(x=0.0, y=2.0, diameter=1.0, name="b",
                     naming_scheme="MANUAL")
        w0.rosette_design = ros
        plate = parent.compile()
        names = [w.name for w in plate.get_all_wells()]
        self.assertNotIn("A1", names)
        self.assertIn("A1.a", names)
        self.assertIn("A1.b", names)

    def test_nested_rosette_round_trips(self):
        parent = PlateDesign.from_standard_format(12)
        w0 = parent.get_wells()[0]
        ros = PlateDesign.blank_rosette(bore_radius_mm=10.0)
        ros.add_well(x=2.0, y=0.0, diameter=1.0, name="s1",
                     naming_scheme="MANUAL")
        w0.rosette_design = ros
        parent2 = PlateDesign.from_dict(
            json.loads(json.dumps(parent.to_dict())))
        w0b = parent2.get_wells()[0]
        self.assertIsNotNone(w0b.rosette_design)
        self.assertEqual(w0b.rosette_design.outline.kind, "circle")
        self.assertEqual(len(w0b.rosette_design.get_wells()), 1)

    def test_circular_outline_round_trips(self):
        from SupportClasses.PlateDesign import PlateOutline
        o = PlateOutline(kind="circle", radius=4.5)
        o2 = PlateOutline.from_dict(o.to_dict())
        self.assertEqual(o2.kind, "circle")
        self.assertAlmostEqual(o2.radius, 4.5)


if __name__ == "__main__":
    unittest.main()
