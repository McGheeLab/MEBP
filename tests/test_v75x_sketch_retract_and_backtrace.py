"""test_v75x_sketch_retract_and_backtrace.py — Sketch retract-&-move points +
back-trace passes (v7.5.x).

Two operator features on the Print Builder → Sketch page:
  1. A colour-coded "retract & move" point — a pen-up break the needle lifts
     to and travels to; it splits the toolpath into runs.
  2. "Back-trace this path" — retrace the continuous run between retract points,
     reversed, offset in height (Z) AND in-plane (perpendicular), optionally
     extruding (toggle) or move-only.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import math
import sys
import unittest

import numpy as np

from PySide6.QtCore import QPointF
from PySide6.QtWidgets import QApplication

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory, backtrace_shape,
    offset_polyline, SHAPE_KINDS,
)


# ═══════════════════════════════════════════════════════════════════
# Model + compiler (headless)
# ═══════════════════════════════════════════════════════════════════

class TestModel(unittest.TestCase):
    def test_travel_kind_registered(self):
        self.assertIn("travel", SHAPE_KINDS)

    def test_new_fields_roundtrip(self):
        s = SketchShape(kind="line", points=[(0, 0), (10, 0)],
                        z_offset_mm=0.35, no_print=True)
        s2 = SketchShape.from_dict(s.to_dict())
        self.assertAlmostEqual(s2.z_offset_mm, 0.35)
        self.assertTrue(s2.no_print)

    def test_defaults_are_backward_compatible(self):
        # A dict WITHOUT the new keys (a pre-v7.5.x saved shape) loads cleanly.
        d = {"kind": "circle", "cx": 1.0, "cy": 2.0, "radius": 3.0}
        s = SketchShape.from_dict(d)
        self.assertEqual(s.z_offset_mm, 0.0)
        self.assertFalse(s.no_print)


class TestTravelCompile(unittest.TestCase):
    def _sketch(self):
        return Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)]),
            SketchShape(kind="travel", cx=20.0, cy=25.0),
            SketchShape(kind="line", points=[(0, 10), (10, 10)]),
        ], z_start_mm=0.2, layer_height_mm=0.2, num_layers=1,
            travel_clearance_mm=2.0)

    def test_travel_point_emits_retracted_move(self):
        r = compile_to_trajectory(self._sketch())
        traj = r.trajectory
        z_travel = 0.2 + 1 * 0.2 + 0.0 + 2.0
        # There is a waypoint at the travel XY, at the (retracted) travel Z.
        mask = ((np.abs(traj[:, 0] - 20.0) < 1e-6)
                & (np.abs(traj[:, 1] - 25.0) < 1e-6))
        self.assertTrue(mask.any())
        self.assertAlmostEqual(float(traj[mask][:, 2].max()), z_travel, places=6)

    def test_travel_point_deposits_nothing(self):
        r = compile_to_trajectory(self._sketch())
        # Pump columns must never advance while travelling to the marker: the
        # only printed length is the two 10 mm lines.
        self.assertAlmostEqual(r.total_length_mm, 20.0, places=6)

    def test_travel_as_first_shape_sets_start(self):
        sk = Sketch(shapes=[
            SketchShape(kind="travel", cx=5.0, cy=5.0),
            SketchShape(kind="line", points=[(0, 0), (10, 0)]),
        ])
        r = compile_to_trajectory(sk)
        self.assertFalse(r.is_empty)
        # First waypoint is the travel point (needle starts retracted there).
        self.assertAlmostEqual(float(r.trajectory[0, 0]), 5.0)
        self.assertAlmostEqual(float(r.trajectory[0, 1]), 5.0)


class TestPerShapeZOffset(unittest.TestCase):
    def test_offset_raises_print_z(self):
        sk = Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)]),
            SketchShape(kind="line", points=[(0, 0), (10, 0)], z_offset_mm=0.5),
        ], z_start_mm=0.2, layer_height_mm=0.2, num_layers=1)
        r = compile_to_trajectory(sk)
        printed_z = {round(float(r.trajectory[i, 2]), 4)
                     for i in range(len(r.trajectory) - 1)
                     if sum(r.pump_states[i]) > 0}
        self.assertIn(0.2, printed_z)     # base pass
        self.assertIn(0.7, printed_z)     # offset pass (0.2 + 0.5)

    def test_travel_clears_tallest_offset_pass(self):
        sk = Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)], z_offset_mm=1.5),
        ], z_start_mm=0.2, layer_height_mm=0.2, num_layers=1,
            travel_clearance_mm=2.0)
        r = compile_to_trajectory(sk)
        # Travel Z must clear the raised pass (0.2 + 0.2 + 1.5 + 2.0).
        self.assertAlmostEqual(float(r.trajectory[:, 2].max()), 3.9, places=6)

    def test_negative_offset_floored_at_plate_bottom(self):
        # A back-trace with a large negative Z offset must not drive the pass
        # below the plate bottom (z=0), only down to it.
        sk = Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)]),
            SketchShape(kind="line", points=[(0, 0), (10, 0)], z_offset_mm=-5.0),
        ], z_start_mm=0.2, layer_height_mm=0.2, num_layers=1)
        r = compile_to_trajectory(sk)
        self.assertGreaterEqual(float(r.trajectory[:, 2].min()), -1e-9)

    def test_zero_offset_is_byte_identical_to_legacy(self):
        base = Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (5, 0)])],
                      z_start_mm=0.2, layer_height_mm=0.2)
        # Explicit z_offset=0 must not change the compiled trajectory.
        withz = Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (5, 0)],
                                           z_offset_mm=0.0)],
                       z_start_mm=0.2, layer_height_mm=0.2)
        a = compile_to_trajectory(base).trajectory
        b = compile_to_trajectory(withz).trajectory
        self.assertTrue(np.allclose(a, b))


class TestOffsetPolyline(unittest.TestCase):
    def test_horizontal_line_offset(self):
        # +x direction → left normal is -y? left normal of (dx,dy)=(1,0) is
        # (-0, 1) → +y. Positive offset shifts to +y.
        op = offset_polyline([(0, 0), (10, 0)], 1.0)
        self.assertEqual(len(op), 2)
        self.assertTrue(all(abs(y - 1.0) < 1e-9 for _, y in op))

    def test_zero_offset_is_noop(self):
        pts = [(0, 0), (3, 4), (6, 0)]
        self.assertEqual(offset_polyline(pts, 0.0), pts)

    def test_sign_flips_side(self):
        pos = offset_polyline([(0, 0), (10, 0)], 1.0)
        neg = offset_polyline([(0, 0), (10, 0)], -1.0)
        self.assertAlmostEqual(pos[0][1], -neg[0][1], places=9)

    def test_closed_square_offset_grows_outward_or_in(self):
        # A CCW square offset should stay a 4-point loop with each vertex moved.
        sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
        off = offset_polyline(sq, 1.0, closed=True)
        self.assertEqual(len(off), 4)
        # Every vertex actually moved.
        for a, b in zip(sq, off):
            self.assertGreater(math.hypot(b[0] - a[0], b[1] - a[1]), 0.5)


class TestBacktraceShape(unittest.TestCase):
    def test_line_reversed_and_offset(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)], z_offset_mm=0.1)
        bt = backtrace_shape(ln, z_offset=0.2, xy_offset=1.0,
                             print_on_return=True)
        self.assertEqual(bt.kind, "line")
        # Reversed order.
        self.assertAlmostEqual(bt.points[0][0], 10.0)
        self.assertAlmostEqual(bt.points[-1][0], 0.0)
        # In-plane offset applied (+y).
        self.assertTrue(all(abs(y - 1.0) < 1e-9 for _, y in bt.points))
        # Height offset ADDS to the source's own offset.
        self.assertAlmostEqual(bt.z_offset_mm, 0.3)
        self.assertFalse(bt.no_print)

    def test_move_only_sets_no_print(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        bt = backtrace_shape(ln, z_offset=0.2, print_on_return=False)
        self.assertTrue(bt.no_print)

    def test_circle_offset_grows_radius(self):
        c = SketchShape(kind="circle", cx=0, cy=0, radius=5.0)
        bt = backtrace_shape(c, xy_offset=1.0)
        self.assertAlmostEqual(bt.radius, 6.0)

    def test_rect_offset_grows_both_dims(self):
        rct = SketchShape(kind="rect", cx=0, cy=0, width=10.0, height=6.0)
        bt = backtrace_shape(rct, xy_offset=0.5)
        self.assertAlmostEqual(bt.width, 11.0)
        self.assertAlmostEqual(bt.height, 7.0)

    def test_source_shape_not_mutated(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        backtrace_shape(ln, z_offset=1.0, xy_offset=2.0)
        self.assertEqual(ln.points, [(0, 0), (10, 0)])
        self.assertEqual(ln.z_offset_mm, 0.0)


class TestBacktraceCompile(unittest.TestCase):
    def test_printing_return_doubles_volume(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        one = compile_to_trajectory(Sketch(shapes=[ln]))
        two = compile_to_trajectory(Sketch(shapes=[
            ln, backtrace_shape(ln, z_offset=0.2, print_on_return=True)]))
        self.assertAlmostEqual(two.total_volume_uL,
                               2 * one.total_volume_uL, places=6)

    def test_move_only_return_adds_no_volume(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        one = compile_to_trajectory(Sketch(shapes=[ln]))
        moveonly = compile_to_trajectory(Sketch(shapes=[
            ln, backtrace_shape(ln, z_offset=0.2, print_on_return=False)]))
        self.assertAlmostEqual(moveonly.total_volume_uL,
                               one.total_volume_uL, places=6)


# ═══════════════════════════════════════════════════════════════════
# Canvas
# ═══════════════════════════════════════════════════════════════════

class _CanvasBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _canvas(self, shapes=None):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        c.set_sketch(Sketch(shapes=shapes or []))
        return c


class TestCanvasTravel(_CanvasBase):
    def test_place_travel_appends_point(self):
        c = self._canvas()
        c._place_travel(QPointF(3.0, 4.0))
        self.assertEqual(len(c.sketch().shapes), 1)
        sh = c.sketch().shapes[0]
        self.assertEqual(sh.kind, "travel")
        self.assertAlmostEqual(sh.cx, 3.0)
        self.assertAlmostEqual(sh.cy, 4.0)
        self.assertEqual(c.selected_indices(), [0])

    def test_travel_point_has_no_resize_handle(self):
        c = self._canvas([SketchShape(kind="travel", cx=0, cy=0)])
        self.assertEqual(c._handle_points(c.sketch().shapes[0]), {})

    def test_hit_selects_travel_point(self):
        c = self._canvas([SketchShape(kind="travel", cx=0.0, cy=0.0)])
        self.assertTrue(c._hit(c.sketch().shapes[0], QPointF(0.0, 0.0), 0.5))
        self.assertFalse(c._hit(c.sketch().shapes[0], QPointF(50.0, 50.0), 0.5))


class TestCanvasRunIndices(_CanvasBase):
    def _mixed(self):
        return self._canvas([
            SketchShape(kind="line", points=[(0, 0), (1, 0)]),      # 0
            SketchShape(kind="line", points=[(1, 0), (2, 0)]),      # 1
            SketchShape(kind="travel", cx=9, cy=9),                 # 2
            SketchShape(kind="circle", cx=5, cy=5, radius=1),       # 3
        ])

    def test_run_from_first_group(self):
        c = self._mixed()
        self.assertEqual(c._run_indices(0), [0, 1])
        self.assertEqual(c._run_indices(1), [0, 1])

    def test_run_after_travel(self):
        c = self._mixed()
        self.assertEqual(c._run_indices(3), [3])

    def test_travel_seed_has_no_run(self):
        c = self._mixed()
        self.assertEqual(c._run_indices(2), [])


class TestCanvasBacktrace(_CanvasBase):
    def test_backtrace_inserts_reversed_after_run(self):
        c = self._canvas([
            SketchShape(kind="line", points=[(0, 0), (10, 0)]),      # 0
            SketchShape(kind="line", points=[(10, 0), (10, 10)]),    # 1
            SketchShape(kind="travel", cx=20, cy=20),                # 2
        ])
        added = c.backtrace_run(0, z_offset=0.2, xy_offset=0.0,
                                print_on_return=True)
        self.assertEqual(added, 2)
        shapes = c.sketch().shapes
        # New shapes go right after the run (index 2..3), before the travel.
        self.assertEqual(shapes[2].kind, "line")
        self.assertEqual(shapes[3].kind, "line")
        self.assertEqual(shapes[4].kind, "travel")
        # Reversed order: first appended = reverse of the LAST run shape.
        self.assertAlmostEqual(shapes[2].points[0][0], 10.0)
        self.assertAlmostEqual(shapes[2].points[0][1], 10.0)
        # Offset applied.
        self.assertAlmostEqual(shapes[2].z_offset_mm, 0.2)

    def test_backtrace_move_only(self):
        c = self._canvas([SketchShape(kind="line", points=[(0, 0), (10, 0)])])
        c.backtrace_run(0, z_offset=0.2, print_on_return=False)
        self.assertTrue(c.sketch().shapes[1].no_print)

    def test_backtrace_travel_seed_is_noop(self):
        c = self._canvas([SketchShape(kind="travel", cx=0, cy=0)])
        self.assertEqual(c.backtrace_run(0), 0)
        self.assertEqual(len(c.sketch().shapes), 1)

    def test_backtrace_is_undoable(self):
        c = self._canvas([SketchShape(kind="line", points=[(0, 0), (10, 0)])])
        c.backtrace_run(0)
        self.assertEqual(len(c.sketch().shapes), 2)
        c.undo()
        self.assertEqual(len(c.sketch().shapes), 1)


# ═══════════════════════════════════════════════════════════════════
# Page (offscreen build smoke)
# ═══════════════════════════════════════════════════════════════════

class TestPage(_CanvasBase):
    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_page_builds_with_travel_tool(self):
        from gui.widgets.sketch_canvas import Tool
        pg = self._page()
        self.assertIn(Tool.TRAVEL, pg._tool_btns)

    def test_backtrace_selected_appends(self):
        pg = self._page()
        pg._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)])]))
        pg._canvas.set_selected(0)
        pg._bt_z_offset = 0.2
        pg._bt_xy_offset = 0.5
        pg._bt_extrude = True
        pg._backtrace_selected()
        self.assertEqual(len(pg._canvas.sketch().shapes), 2)
        self.assertAlmostEqual(pg._canvas.sketch().shapes[1].z_offset_mm, 0.2)

    def test_travel_point_props_card_builds(self):
        pg = self._page()
        pg._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="travel", cx=1.0, cy=2.0)]))
        pg._canvas.set_selected(0)
        pg._rebuild_props()   # must not raise; no back-trace card for travel


# ═══════════════════════════════════════════════════════════════════
# Adversarial-review follow-ups (fixes + locked-in non-bugs)
# ═══════════════════════════════════════════════════════════════════

class TestReviewFixes(_CanvasBase):
    # --- F6: multi-layer back-trace keeps correct per-layer Z + footprint ---
    def test_multilayer_backtrace_z_levels(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        bt = backtrace_shape(ln, z_offset=0.3, print_on_return=True)
        r = compile_to_trajectory(Sketch(shapes=[ln, bt], z_start_mm=0.2,
                                         layer_height_mm=0.2, num_layers=2))
        printed_z = sorted({round(float(r.trajectory[i, 2]), 3)
                            for i in range(len(r.trajectory) - 1)
                            if sum(r.pump_states[i]) > 0})
        # fwd L0=0.2, bt L0=0.5, fwd L1=0.4, bt L1=0.7 — direction doesn't
        # change WHERE material lands, only how the needle travels.
        self.assertEqual(printed_z, [0.2, 0.4, 0.5, 0.7])

    # --- F8: travel point as first shape starts RETRACTED (safe) ---
    def test_travel_first_shape_starts_retracted(self):
        sk = Sketch(shapes=[
            SketchShape(kind="travel", cx=5.0, cy=5.0),
            SketchShape(kind="line", points=[(0, 0), (10, 0)])],
            z_start_mm=0.2, layer_height_mm=0.2, num_layers=1,
            travel_clearance_mm=2.0)
        r = compile_to_trajectory(sk)
        z_travel = 0.2 + 1 * 0.2 + 0.0 + 2.0
        self.assertAlmostEqual(float(r.trajectory[0, 2]), z_travel, places=6)

    # --- F1: chained back-trace clamps the accumulated Z offset ---
    def test_chained_backtrace_offset_clamped(self):
        sh = SketchShape(kind="line", points=[(0, 0), (10, 0)], z_offset_mm=39.0)
        bt = backtrace_shape(sh, z_offset=10.0)   # 39 + 10 = 49 → clamp to 40
        self.assertLessEqual(bt.z_offset_mm, 40.0)
        self.assertGreaterEqual(bt.z_offset_mm, -40.0)

    # --- F2: offset_polyline tolerates duplicate consecutive vertices ---
    def test_offset_polyline_duplicate_vertices(self):
        pts = [(0, 0), (0, 0), (10, 0), (10, 0)]   # duplicates
        op = offset_polyline(pts, 1.0)
        # No NaN / inf; every point finite and offset applied to +y.
        self.assertTrue(all(math.isfinite(x) and math.isfinite(y)
                            for x, y in op))
        self.assertTrue(all(abs(y - 1.0) < 1e-6 for _, y in op))

    def test_offset_polyline_all_duplicate_is_safe(self):
        op = offset_polyline([(2, 2), (2, 2), (2, 2)], 1.0)
        self.assertTrue(all(math.isfinite(x) and math.isfinite(y)
                            for x, y in op))

    # --- F11: fit_view clamps zoom for a tiny-span (clustered) sketch ---
    def test_fit_view_clamps_tiny_span_zoom(self):
        c = self._canvas([
            SketchShape(kind="travel", cx=0.0, cy=0.0),
            SketchShape(kind="travel", cx=0.05, cy=0.05)])
        c.resize(400, 400)
        c.fit_view()
        self.assertLessEqual(c._scale, 200.0)


class TestBacktraceNegativeZConfirm(_CanvasBase):
    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        pg = SketchPage()
        pg._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)])]))
        pg._canvas.set_selected(0)
        return pg

    def test_negative_z_cancel_aborts_backtrace(self):
        from unittest.mock import patch
        from PySide6.QtWidgets import QMessageBox
        pg = self._page()
        pg._bt_z_offset = -0.5
        with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                   return_value=QMessageBox.No):
            pg._backtrace_selected()
        self.assertEqual(len(pg._canvas.sketch().shapes), 1)   # no pass added

    def test_negative_z_confirm_proceeds(self):
        from unittest.mock import patch
        from PySide6.QtWidgets import QMessageBox
        pg = self._page()
        pg._bt_z_offset = -0.5
        with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                   return_value=QMessageBox.Yes):
            pg._backtrace_selected()
        self.assertEqual(len(pg._canvas.sketch().shapes), 2)

    def test_positive_z_no_prompt(self):
        from unittest.mock import patch
        from PySide6.QtWidgets import QMessageBox
        pg = self._page()
        pg._bt_z_offset = 0.2
        with patch("gui.pages.print_builder_sketch.QMessageBox.warning") as w:
            pg._backtrace_selected()
        w.assert_not_called()
        self.assertEqual(len(pg._canvas.sketch().shapes), 2)


if __name__ == "__main__":
    unittest.main()
