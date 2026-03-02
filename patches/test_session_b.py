"""
test_session_b.py — Unit tests for Session B: Geometry Engine.

Tests:
- 2D generators: point, line, circle, square, triangle, spiral, ellipse
- 3D generators: sphere, cube, cylinder, ellipsoid (shell + solid)
- Needle-aware line spacing
- Volume-conserving extrusion rate
- Time parameterization
- PrintObject and PrintCollection dataclasses
- Full generate_object_trajectory dispatch
"""

import math
import unittest
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec, InkSpec
from SupportClasses.GeometryEngine import (
    # Dataclasses
    PrintObject, PrintCollection, ObjectType, FillPattern,
    # Constants
    COL_X, COL_Y, COL_Z, COL_P1, COL_P2, COL_P3, COL_T, NUM_COLS,
    # Helpers
    line_spacing, compute_pump_positions, compute_total_volume,
    # 2D generators
    generate_line, generate_circle, generate_square,
    generate_triangle, generate_spiral, generate_ellipse,
    # Fill generators
    generate_meander_fill, generate_circular_meander_fill,
    generate_elliptical_meander_fill,
    # 3D helpers
    compute_layer_heights,
    # Main dispatch
    generate_object_trajectory,
    # Utility
    get_available_object_types, get_default_params,
    OBJECT_TYPE_INFO,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def make_needle_22g() -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


def make_syringe_100() -> SyringeSpec:
    return SyringeSpec(volume_uL=100, stroke_length_mm=30.0, barrel_id_mm=2.060)


# ---------------------------------------------------------------------------
# Test 2D Generators
# ---------------------------------------------------------------------------

class TestGenerateLine(unittest.TestCase):

    def test_basic_line(self):
        pts = generate_line(0, 0, 10, 0, num_points=11)
        self.assertEqual(pts.shape, (11, 2))
        self.assertAlmostEqual(pts[0, 0], 0.0)
        self.assertAlmostEqual(pts[-1, 0], 10.0)
        # Y should be all zeros
        np.testing.assert_allclose(pts[:, 1], 0.0)

    def test_diagonal_line(self):
        pts = generate_line(0, 0, 3, 4, num_points=50)
        self.assertEqual(pts.shape, (50, 2))
        length = np.sqrt((pts[-1, 0] - pts[0, 0])**2 + (pts[-1, 1] - pts[0, 1])**2)
        self.assertAlmostEqual(length, 5.0, places=3)


class TestGenerateCircle(unittest.TestCase):

    def test_basic_circle(self):
        pts = generate_circle(0, 0, 1.0, 64)
        self.assertEqual(pts.shape[0], 65)  # 64 + 1 to close
        # First and last should be same (closed)
        np.testing.assert_allclose(pts[0], pts[-1], atol=1e-10)

    def test_radius(self):
        pts = generate_circle(0, 0, 5.0, 100)
        radii = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        np.testing.assert_allclose(radii, 5.0, atol=1e-10)

    def test_offset_center(self):
        pts = generate_circle(3, 4, 1.0, 32)
        cx = pts[:, 0].mean()
        cy = pts[:, 1].mean()
        self.assertAlmostEqual(cx, 3.0, places=1)
        self.assertAlmostEqual(cy, 4.0, places=1)


class TestGenerateSquare(unittest.TestCase):

    def test_basic_square(self):
        pts = generate_square(0, 0, 2.0, 10)
        self.assertGreater(len(pts), 30)
        # All points should be within bounds
        self.assertAlmostEqual(pts[:, 0].min(), -1.0, places=3)
        self.assertAlmostEqual(pts[:, 0].max(), 1.0, places=3)

    def test_closed_path(self):
        pts = generate_square(0, 0, 4.0, 20)
        np.testing.assert_allclose(pts[0], pts[-1], atol=0.01)


class TestGenerateTriangle(unittest.TestCase):

    def test_basic_triangle(self):
        pts = generate_triangle(0, 0, 2.0, 15)
        self.assertGreater(len(pts), 30)

    def test_equilateral_side_length(self):
        side = 3.0
        pts = generate_triangle(0, 0, side, 50)
        # Compute distance between first few vertices
        # The triangle has 3 sides of approximately equal length
        # Total perimeter should be ~3×side
        diffs = np.diff(pts, axis=0)
        seg_lens = np.sqrt(np.sum(diffs**2, axis=1))
        total = seg_lens.sum()
        self.assertAlmostEqual(total, 3 * side, places=1)


class TestGenerateSpiral(unittest.TestCase):

    def test_basic_spiral(self):
        pts = generate_spiral(0, 0, 2.0, 0.5, 48)
        self.assertGreater(len(pts), 100)
        # Should start near center
        self.assertAlmostEqual(pts[0, 0], 0.0, places=1)
        self.assertAlmostEqual(pts[0, 1], 0.0, places=1)

    def test_max_radius(self):
        pts = generate_spiral(0, 0, 3.0, 0.5)
        max_r = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2).max()
        self.assertLessEqual(max_r, 3.5)  # Allow small overshoot


class TestGenerateEllipse(unittest.TestCase):

    def test_basic_ellipse(self):
        pts = generate_ellipse(0, 0, 3.0, 1.0, 64)
        self.assertEqual(pts.shape[0], 65)
        # X should span from -3 to 3
        self.assertAlmostEqual(pts[:, 0].min(), -3.0, places=2)
        self.assertAlmostEqual(pts[:, 0].max(), 3.0, places=2)
        # Y should span from -1 to 1
        self.assertAlmostEqual(pts[:, 1].min(), -1.0, places=2)
        self.assertAlmostEqual(pts[:, 1].max(), 1.0, places=2)


# ---------------------------------------------------------------------------
# Test Fill Generators
# ---------------------------------------------------------------------------

class TestMeanderFill(unittest.TestCase):

    def test_basic_fill(self):
        pts = generate_meander_fill(0, 0, 4.0, 4.0, 0.5)
        self.assertGreater(len(pts), 50)
        # Should fill within bounds
        self.assertGreaterEqual(pts[:, 0].min(), -2.5)
        self.assertLessEqual(pts[:, 0].max(), 2.5)

    def test_alternating_direction(self):
        pts = generate_meander_fill(0, 0, 4.0, 2.0, 1.0, points_per_line=5)
        # Check that odd lines go in reverse
        # First line should go left-to-right
        self.assertLess(pts[0, 0], pts[4, 0])


class TestCircularMeanderFill(unittest.TestCase):

    def test_basic_fill(self):
        pts = generate_circular_meander_fill(0, 0, 2.0, 0.5)
        self.assertGreater(len(pts), 20)
        # All points should be within the circle (with tolerance)
        radii = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        self.assertTrue(np.all(radii <= 2.1))


class TestEllipticalMeanderFill(unittest.TestCase):

    def test_basic_fill(self):
        pts = generate_elliptical_meander_fill(0, 0, 3.0, 1.5, 0.5)
        self.assertGreater(len(pts), 20)


# ---------------------------------------------------------------------------
# Test Line Spacing & Extrusion
# ---------------------------------------------------------------------------

class TestLineSpacing(unittest.TestCase):

    def test_no_overlap(self):
        needle = make_needle_22g()
        s = line_spacing(needle, overlap_fraction=0.0)
        self.assertAlmostEqual(s, 0.718, places=3)

    def test_with_overlap(self):
        needle = make_needle_22g()
        s = line_spacing(needle, overlap_fraction=0.2)
        self.assertAlmostEqual(s, 0.718 * 0.8, places=3)


class TestPumpPositions(unittest.TestCase):

    def test_linear_extrusion(self):
        needle = make_needle_22g()
        syringe = make_syringe_100()
        dists = np.array([0.0, 1.0, 2.0, 3.0])
        positions = compute_pump_positions(dists, needle, syringe, layer_height_mm=0.2)

        # Volume per mm = OD × layer_h = 0.718 × 0.2 = 0.1436 µL/mm
        # At dist=3mm: volume = 0.4308 µL, pump travel = 0.4308 * (30/100) mm
        expected_vol = 3.0 * 0.718 * 0.2
        expected_pump = expected_vol * syringe.mm_per_uL
        self.assertAlmostEqual(positions[-1], expected_pump, places=4)

    def test_zero_distance(self):
        needle = make_needle_22g()
        syringe = make_syringe_100()
        dists = np.array([0.0])
        positions = compute_pump_positions(dists, needle, syringe, 0.2)
        self.assertAlmostEqual(positions[0], 0.0)


class TestComputeTotalVolume(unittest.TestCase):

    def test_known_path(self):
        needle = make_needle_22g()
        vol = compute_total_volume(10.0, needle, 0.2)
        expected = 10.0 * 0.718 * 0.2
        self.assertAlmostEqual(vol, expected, places=3)


# ---------------------------------------------------------------------------
# Test Layer Heights
# ---------------------------------------------------------------------------

class TestComputeLayerHeights(unittest.TestCase):

    def test_basic(self):
        layers = compute_layer_heights(0.0, 1.0, 0.2)
        self.assertEqual(len(layers), 5)
        self.assertAlmostEqual(layers[0], 0.0)
        self.assertAlmostEqual(layers[-1], 0.8)

    def test_single_layer(self):
        layers = compute_layer_heights(0.0, 0.1, 0.2)
        self.assertEqual(len(layers), 1)


# ---------------------------------------------------------------------------
# Test PrintObject & PrintCollection
# ---------------------------------------------------------------------------

class TestPrintObject(unittest.TestCase):

    def test_create_and_serialize(self):
        obj = PrintObject(
            name="TestCircle",
            object_type="circle",
            params={"radius": 2.0},
            position=(1.0, 2.0, 0.0),
            color="#ff0000",
        )
        d = obj.to_dict()
        obj2 = PrintObject.from_dict(d)
        self.assertEqual(obj2.name, "TestCircle")
        self.assertEqual(obj2.object_type, "circle")
        self.assertAlmostEqual(obj2.position[0], 1.0)

    def test_trajectory_serialization(self):
        obj = PrintObject(name="T", object_type="line")
        obj.trajectory = np.array([[0, 0, 0, 0, 0, 0, 0],
                                    [1, 1, 0, 0.1, 0, 0, 1.0]])
        d = obj.to_dict()
        obj2 = PrintObject.from_dict(d)
        self.assertTrue(obj2.has_trajectory)
        self.assertEqual(obj2.num_waypoints, 2)

    def test_bounds(self):
        obj = PrintObject(name="T", object_type="line")
        obj.trajectory = np.array([
            [-1, -2, 0, 0, 0, 0, 0],
            [3, 4, 1, 0, 0, 0, 1],
        ])
        b = obj.bounds
        self.assertAlmostEqual(b["x"][0], -1.0)
        self.assertAlmostEqual(b["x"][1], 3.0)
        self.assertAlmostEqual(b["y"][0], -2.0)
        self.assertAlmostEqual(b["y"][1], 4.0)


class TestPrintCollection(unittest.TestCase):

    def test_add_and_remove(self):
        coll = PrintCollection(name="Test")
        o1 = PrintObject(name="A", object_type="circle")
        o2 = PrintObject(name="B", object_type="line")
        coll.add_object(o1)
        coll.add_object(o2)
        self.assertEqual(coll.num_objects, 2)

        removed = coll.remove_object(0)
        self.assertEqual(removed.name, "A")
        self.assertEqual(coll.num_objects, 1)

    def test_move_object(self):
        coll = PrintCollection(name="Test")
        for name in ["A", "B", "C"]:
            coll.add_object(PrintObject(name=name, object_type="point"))
        coll.move_object(2, 0)
        self.assertEqual(coll.objects[0].name, "C")
        self.assertEqual(coll.objects[1].name, "A")

    def test_serialization(self):
        coll = PrintCollection(name="MyJob")
        coll.add_object(PrintObject(name="X", object_type="circle"))
        d = coll.to_dict()
        coll2 = PrintCollection.from_dict(d)
        self.assertEqual(coll2.name, "MyJob")
        self.assertEqual(coll2.num_objects, 1)


# ---------------------------------------------------------------------------
# Test Full Trajectory Generation
# ---------------------------------------------------------------------------

class TestGenerateObjectTrajectory(unittest.TestCase):

    def setUp(self):
        self.needle = make_needle_22g()
        self.syringe = make_syringe_100()
        self.syringe_map = {"P1": self.syringe}

    def _generate(self, obj_type: str, params: dict = None, **kwargs) -> PrintObject:
        obj = PrintObject(
            name=f"Test_{obj_type}",
            object_type=obj_type,
            params=params or get_default_params(obj_type),
        )
        generate_object_trajectory(
            obj, self.needle, self.syringe_map,
            print_speed_mm_s=5.0, layer_height_mm=0.2,
            **kwargs,
        )
        return obj

    # --- 2D objects ---

    def test_point(self):
        obj = self._generate("point", {"cx": 0, "cy": 0, "dwell_time_s": 0.5,
                                         "dispense_volume_uL": 0.2})
        self.assertTrue(obj.has_trajectory)
        self.assertEqual(obj.trajectory.shape[1], NUM_COLS)
        self.assertEqual(obj.num_waypoints, 2)
        self.assertAlmostEqual(obj.trajectory[-1, COL_T], 0.5)

    def test_line(self):
        obj = self._generate("line", {"x1": 0, "y1": 0, "x2": 5, "y2": 0, "num_points": 20})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_waypoints, 10)
        self.assertGreater(obj.total_time_s, 0)
        # Pump positions should increase monotonically
        p1 = obj.trajectory[:, COL_P1]
        self.assertTrue(np.all(np.diff(p1) >= -1e-10))

    def test_circle(self):
        obj = self._generate("circle")
        self.assertTrue(obj.has_trajectory)
        self.assertEqual(obj.num_layers, 1)

    def test_square(self):
        obj = self._generate("square")
        self.assertTrue(obj.has_trajectory)

    def test_triangle(self):
        obj = self._generate("triangle")
        self.assertTrue(obj.has_trajectory)

    def test_spiral(self):
        obj = self._generate("spiral")
        self.assertTrue(obj.has_trajectory)

    def test_ellipse(self):
        obj = self._generate("ellipse")
        self.assertTrue(obj.has_trajectory)

    # --- 3D shell objects ---

    def test_sphere_shell(self):
        obj = self._generate("sphere_shell", {"radius": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_layers, 1)

    def test_cube_shell(self):
        obj = self._generate("cube_shell", {"side": 2.0, "height": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_layers, 1)

    def test_cylinder_shell(self):
        obj = self._generate("cylinder_shell", {"radius": 1.0, "height": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_layers, 1)

    def test_ellipsoid_shell(self):
        obj = self._generate("ellipsoid_shell", {"a": 2, "b": 1.5, "c": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_layers, 1)

    # --- 3D solid objects ---

    def test_sphere_solid(self):
        obj = self._generate("sphere_solid", {"radius": 2.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_layers, 1)
        self.assertGreater(obj.total_volume_uL, 0)

    def test_cube_solid(self):
        obj = self._generate("cube_solid", {"side": 2.0, "height": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)
        self.assertGreater(obj.num_layers, 1)

    def test_cylinder_solid(self):
        obj = self._generate("cylinder_solid", {"radius": 1.0, "height": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)

    def test_ellipsoid_solid(self):
        obj = self._generate("ellipsoid_solid", {"a": 2, "b": 1.5, "c": 1.0, "layer_height": 0.5})
        self.assertTrue(obj.has_trajectory)

    # --- Edge cases ---

    def test_unknown_type(self):
        obj = PrintObject(name="Bad", object_type="nonexistent")
        traj = generate_object_trajectory(obj, self.needle, self.syringe_map)
        self.assertEqual(len(traj), 0)

    def test_position_offset(self):
        """Object position should offset all trajectory coordinates."""
        obj = PrintObject(
            name="OffsetCircle",
            object_type="circle",
            params={"radius": 1.0, "num_points": 32},
            position=(5.0, 10.0, 2.0),
        )
        generate_object_trajectory(obj, self.needle, self.syringe_map)
        # Center of circle trajectory should be near (5, 10)
        mean_x = obj.trajectory[:, COL_X].mean()
        mean_y = obj.trajectory[:, COL_Y].mean()
        self.assertAlmostEqual(mean_x, 5.0, places=0)
        self.assertAlmostEqual(mean_y, 10.0, places=0)
        # Z should be 2.0
        np.testing.assert_allclose(obj.trajectory[:, COL_Z], 2.0)

    def test_time_monotonic(self):
        """Time should be monotonically increasing for all types."""
        for otype in ["line", "circle", "square", "cylinder_shell"]:
            obj = self._generate(otype)
            if obj.has_trajectory and obj.num_waypoints > 1:
                times = obj.trajectory[:, COL_T]
                self.assertTrue(np.all(np.diff(times) >= 0),
                               f"Time not monotonic for {otype}")

    def test_pump_monotonic(self):
        """Pump positions should be monotonically non-decreasing."""
        for otype in ["line", "circle", "spiral"]:
            obj = self._generate(otype)
            if obj.has_trajectory and obj.num_waypoints > 1:
                p1 = obj.trajectory[:, COL_P1]
                self.assertTrue(np.all(np.diff(p1) >= -1e-10),
                               f"Pump not monotonic for {otype}")

    def test_trajectory_columns(self):
        """All trajectories should have exactly 7 columns."""
        for otype in ["point", "line", "circle", "sphere_solid", "cube_shell"]:
            obj = self._generate(otype)
            if obj.has_trajectory:
                self.assertEqual(obj.trajectory.shape[1], 7,
                               f"Wrong columns for {otype}")

    def test_volume_increases_with_path(self):
        """Longer path = more ink volume."""
        short = self._generate("line", {"x1": 0, "y1": 0, "x2": 1, "y2": 0})
        long = self._generate("line", {"x1": 0, "y1": 0, "x2": 10, "y2": 0})
        self.assertGreater(long.total_volume_uL, short.total_volume_uL)


# ---------------------------------------------------------------------------
# Test Object Type Catalog
# ---------------------------------------------------------------------------

class TestObjectTypeCatalog(unittest.TestCase):

    def test_all_types_have_info(self):
        for ot in ObjectType:
            if ot == ObjectType.CSV_IMPORT:
                continue
            self.assertIn(ot.value, OBJECT_TYPE_INFO,
                         f"Missing info for {ot.value}")

    def test_get_default_params(self):
        for ot in ObjectType:
            if ot == ObjectType.CSV_IMPORT:
                continue
            params = get_default_params(ot.value)
            self.assertIsInstance(params, dict,
                                f"Bad params for {ot.value}")

    def test_get_available_types(self):
        types = get_available_object_types()
        self.assertGreater(len(types), 10)
        for name, info in types.items():
            self.assertIn("label", info)
            self.assertIn("category", info)
            self.assertIn(info["category"], ("2D", "3D"))


if __name__ == "__main__":
    unittest.main()
