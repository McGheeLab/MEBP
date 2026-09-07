"""
test_v75x_sketch_trajectory.py — unit tests for the Print Builder Sketch
compiler (SupportClasses/SketchTrajectory.py).

No GUI / hardware required.
"""

import unittest

import numpy as np

from SupportClasses.SketchTrajectory import (
    Sketch,
    SketchShape,
    SketchInk,
    compile_to_trajectory,
    compute_fill_region,
)
from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec


def _needle() -> NeedleSpec:
    # 25G-ish: OD ~500 µm, ID ~250 µm.
    return NeedleSpec(gauge=25, od_um=500.0, id_um=250.0, wall_um=125.0)


def _syringe() -> SyringeSpec:
    return SyringeSpec(volume_uL=100, stroke_length_mm=30.0, barrel_id_mm=1.46)


class TestSketchCompiler(unittest.TestCase):

    def _assert_valid_traj(self, traj: np.ndarray):
        self.assertEqual(traj.ndim, 2)
        self.assertEqual(traj.shape[1], 7)
        self.assertGreaterEqual(len(traj), 2)
        # Time column is monotonic non-decreasing.
        t = traj[:, 6]
        self.assertTrue(np.all(np.diff(t) >= -1e-9),
                        "time column must be monotonic")
        # All values finite.
        self.assertTrue(np.all(np.isfinite(traj)))

    def test_single_circle_outline(self):
        sk = Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=5)])
        result = compile_to_trajectory(sk, _needle(), _syringe())
        self._assert_valid_traj(result.trajectory)
        self.assertFalse(result.is_empty)
        self.assertEqual(result.num_layers, 1)
        self.assertGreater(result.total_length_mm, 0.0)
        self.assertGreater(result.total_time_s, 0.0)

    def test_filled_circle_has_interior_points(self):
        outline = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0,
                                       radius=5, filled=False)]))
        filled = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0,
                                       radius=5, filled=True)],
                   line_spacing_mm=0.4))
        # A raster fill should produce substantially more travel than the
        # bare perimeter.
        self.assertGreater(filled.total_length_mm, outline.total_length_mm)

    def test_multilayer_stacks_z(self):
        sk = Sketch(
            shapes=[SketchShape(kind="rect", cx=0, cy=0, width=10, height=10)],
            num_layers=3, layer_height_mm=0.5, z_start_mm=1.0)
        result = compile_to_trajectory(sk, _needle(), _syringe())
        self._assert_valid_traj(result.trajectory)
        self.assertEqual(result.num_layers, 3)
        zs = result.trajectory[:, 2]
        # The three print layers sit at z = 1.0, 1.5, 2.0 (travel rises above).
        printed_z = set(round(float(z), 3) for z in zs)
        for expected in (1.0, 1.5, 2.0):
            self.assertIn(expected, printed_z)

    def test_pump_states_zero_on_travel(self):
        # Two separate shapes → at least one travel segment between them.
        sk = Sketch(shapes=[
            SketchShape(kind="circle", cx=0, cy=0, radius=3, ink_id=1),
            SketchShape(kind="circle", cx=20, cy=20, radius=3, ink_id=1),
        ])
        result = compile_to_trajectory(sk, _needle(), _syringe())
        self.assertEqual(len(result.pump_states), len(result.trajectory))
        # Some segments print (flow > 0) and some travel (all zero).
        any_print = any(sum(st) > 0 for st in result.pump_states)
        any_travel = any(sum(st) == 0 for st in result.pump_states)
        self.assertTrue(any_print)
        self.assertTrue(any_travel)

    def test_ink_preview_column_routing(self):
        # The trajectory's 3 pump columns are now a PREVIEW artifact keyed by
        # each ink's ORDER index % 3 (the sketch is pump-agnostic). A shape on
        # the 2nd abstract ink (order index 1) accumulates in column index 4.
        sk = Sketch(shapes=[SketchShape(kind="line",
                                        points=[(0, 0), (10, 0)], ink_id=2)])
        sk.inks = [SketchInk(1, "A", "#89b4fa"), SketchInk(2, "B", "#a6e3a1")]
        result = compile_to_trajectory(sk, _needle(), _syringe())
        final = result.trajectory[-1]
        self.assertEqual(final[3], 0.0)          # column 0 untouched
        self.assertGreater(final[4], 0.0)        # column 1 (2nd ink) advanced
        self.assertEqual(final[5], 0.0)          # column 2 untouched

    def test_pump_displacement_monotonic(self):
        sk = Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                        filled=True)])
        result = compile_to_trajectory(sk, _needle(), _syringe())
        p1 = result.trajectory[:, 3]
        self.assertTrue(np.all(np.diff(p1) >= -1e-9),
                        "cumulative pump displacement must never decrease")

    def test_empty_sketch_is_empty(self):
        result = compile_to_trajectory(Sketch(shapes=[]))
        self.assertTrue(result.is_empty)

    def test_no_hardware_still_compiles(self):
        # Without needle/syringe the compiler falls back to flow_factor.
        sk = Sketch(shapes=[SketchShape(kind="rect", cx=0, cy=0,
                                        width=8, height=4)])
        result = compile_to_trajectory(sk)
        self._assert_valid_traj(result.trajectory)
        self.assertGreater(result.trajectory[-1, 3], 0.0)

    # ── Thickness (bead width / multi-pass) ──────────────────────────

    def test_thick_outline_is_still_a_single_pass(self):
        """v7.21.4 — REPLACES ``test_thick_outline_multipass``.

        A wider declared line width no longer expands into adjacent parallel
        passes: it prints ONE bead on the drawn geometry, and the width is
        reached by raising the extrusion multiplier instead. The old contract
        (>3x the length and volume at 2.0 mm vs 0.4 mm) is exactly what must
        NOT happen any more.
        """
        thin = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=10,
                                       line_width_mm=0.4)],
                   line_spacing_mm=0.4))
        thick = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=10,
                                       line_width_mm=2.0)],
                   line_spacing_mm=0.4))
        self.assertAlmostEqual(thick.total_length_mm, thin.total_length_mm,
                               places=6)
        # v7.21.5: the width now drives that shape's FLOW instead of its
        # geometry — 5x the width, 5x the deposition, same single pass.
        self.assertAlmostEqual(thick.total_volume_uL,
                               thin.total_volume_uL * 5.0, places=9)

    # ── Paint-bucket fill ────────────────────────────────────────────

    def test_fill_region_inside_circle(self):
        circle = SketchShape(kind="circle", cx=0, cy=0, radius=10)
        pts = compute_fill_region([circle], (0.0, 0.0), spacing_mm=1.0)
        self.assertIsNotNone(pts)
        self.assertGreater(len(pts), 2)
        # All fill points lie inside the enclosing circle.
        for (x, y) in pts:
            self.assertLessEqual((x ** 2 + y ** 2) ** 0.5, 10.5)

    def test_fill_region_open_returns_none(self):
        # A bare line does not enclose anything.
        line = SketchShape(kind="line", points=[(-10, 0), (10, 0)])
        self.assertIsNone(compute_fill_region([line], (0.0, 5.0),
                                              spacing_mm=1.0))
        # No shapes at all → None.
        self.assertIsNone(compute_fill_region([], (0.0, 0.0), spacing_mm=1.0))

    def test_region_shape_compiles(self):
        circle = SketchShape(kind="circle", cx=0, cy=0, radius=8)
        pts = compute_fill_region([circle], (0.0, 0.0), spacing_mm=1.0)
        self.assertIsNotNone(pts)
        # circle → ink 1 (col 0); region → ink 2 (order index 1 → col 1).
        region = SketchShape(kind="region", points=pts, ink_id=2)
        result = compile_to_trajectory(Sketch(shapes=[circle, region]))
        self._assert_valid_traj(result.trajectory)
        # The region prints on its own preview column (index 4 = 2nd ink).
        self.assertGreater(result.trajectory[-1, 4], 0.0)


if __name__ == "__main__":
    unittest.main()
