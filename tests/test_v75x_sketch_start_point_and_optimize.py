"""test_v75x_sketch_start_point_and_optimize.py — Print Builder Sketch: manual
per-shape print start/stop control + the auto path-optimize postprocess.

Two operator-requested features:
  1. ``SketchShape.start_point`` lets any outline shape begin printing at a
     chosen point (snapping to existing lines), so it welds continuously onto a
     neighbour instead of lifting the needle.
  2. ``optimize_print_order`` reorders shapes + picks their start points to
     minimize the number of pen-up discontinuities; user-placed retract
     (``travel``) points stay as fixed breaks.

Backend tests need no GUI; the canvas/page tests run offscreen.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import math
import sys
import unittest

import numpy as np

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory, reorder_path_to_start,
    optimize_print_order, count_discontinuities,
)
from SupportClasses.GeometryEngine import generate_circle


def _line(x1, y1, x2, y2, pump=0) -> SketchShape:
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)],
                       ink_id=pump + 1)   # 0-based pump → 1-based abstract ink


def _polyline_len(pts) -> float:
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


# ═══════════════════════════════════════════════════════════════════
# reorder_path_to_start
# ═══════════════════════════════════════════════════════════════════

class TestReorderPathToStart(unittest.TestCase):

    def test_open_no_reverse_when_first_endpoint_nearer(self):
        p = np.array([[0.0, 0.0], [10.0, 0.0]])
        r = reorder_path_to_start(p, (0.0, 0.0), closed=False)
        self.assertTrue(np.allclose(r[0], [0.0, 0.0]))

    def test_open_reverses_when_far_endpoint_nearer(self):
        p = np.array([[0.0, 0.0], [10.0, 0.0]])
        r = reorder_path_to_start(p, (10.0, 0.0), closed=False)
        self.assertTrue(np.allclose(r[0], [10.0, 0.0]))
        self.assertTrue(np.allclose(r[-1], [0.0, 0.0]))

    def test_open_length_preserved(self):
        p = np.array([[0.0, 0.0], [3.0, 4.0], [3.0, 10.0]])
        r = reorder_path_to_start(p, (3.0, 10.0), closed=False)
        self.assertAlmostEqual(_polyline_len(p.tolist()),
                               _polyline_len(r.tolist()), places=9)

    def test_closed_rolls_to_nearest_and_stays_closed(self):
        c = generate_circle(0, 0, 5, 32)          # closed, starts at (5, 0)
        r = reorder_path_to_start(c, (0.0, 5.0), closed=True)   # roll to top
        self.assertTrue(np.allclose(r[0], r[-1]))              # still a loop
        self.assertLess(math.dist(r[0], (0.0, 5.0)), 0.6)      # begins near top

    def test_closed_length_preserved(self):
        c = generate_circle(0, 0, 5, 32)
        r = reorder_path_to_start(c, (-5.0, 0.0), closed=True)
        self.assertAlmostEqual(_polyline_len(c.tolist()),
                               _polyline_len(r.tolist()), places=6)

    def test_degenerate_returns_unchanged(self):
        p = np.array([[1.0, 1.0]])
        r = reorder_path_to_start(p, (9.0, 9.0), closed=False)
        self.assertTrue(np.allclose(r, p))


# ═══════════════════════════════════════════════════════════════════
# start_point in the compiler → weld continuity
# ═══════════════════════════════════════════════════════════════════

class TestStartPointWelds(unittest.TestCase):

    def _sk(self, shapes) -> Sketch:
        return Sketch(shapes=shapes, line_spacing_mm=0.4, z_start_mm=0.2,
                      layer_height_mm=0.2, num_layers=1, travel_clearance_mm=2.0)

    def test_start_point_welds_shape_onto_previous(self):
        # A line ending at (10,0); a rect whose left edge passes through (10,0).
        # By default the rect starts at a far corner → a travel between them.
        rect_default = SketchShape(kind="rect", cx=15, cy=0, width=10, height=10)
        self.assertEqual(
            count_discontinuities(self._sk([_line(0, 0, 10, 0), rect_default])),
            1)
        # With start_point on the shared node, the rect begins at (10,0) and
        # welds — no pen-up.
        rect_start = SketchShape(kind="rect", cx=15, cy=0, width=10, height=10,
                                 start_point=(10.0, 0.0))
        self.assertEqual(
            count_discontinuities(self._sk([_line(0, 0, 10, 0), rect_start])),
            0)

    def test_start_point_does_not_change_printed_length(self):
        base = SketchShape(kind="circle", cx=0, cy=0, radius=5)
        moved = SketchShape(kind="circle", cx=0, cy=0, radius=5,
                            start_point=(0.0, 5.0))
        r0 = compile_to_trajectory(self._sk([base]))
        r1 = compile_to_trajectory(self._sk([moved]))
        self.assertAlmostEqual(r0.total_length_mm, r1.total_length_mm, places=3)

    def test_start_point_ignored_on_fill(self):
        # A filled shape's raster start is fixed — start_point must be a no-op.
        f0 = SketchShape(kind="circle", cx=0, cy=0, radius=5, filled=True)
        f1 = SketchShape(kind="circle", cx=0, cy=0, radius=5, filled=True,
                         start_point=(0.0, 5.0))
        a = compile_to_trajectory(self._sk([f0])).trajectory
        b = compile_to_trajectory(self._sk([f1])).trajectory
        self.assertTrue(np.allclose(a, b))


# ═══════════════════════════════════════════════════════════════════
# Serialization
# ═══════════════════════════════════════════════════════════════════

class TestSerialization(unittest.TestCase):

    def test_roundtrip(self):
        s = SketchShape(kind="circle", cx=1, cy=2, radius=3,
                        start_point=(4.0, 5.0))
        self.assertEqual(SketchShape.from_dict(s.to_dict()).start_point,
                         (4.0, 5.0))

    def test_omitted_when_none(self):
        self.assertNotIn("start_point", SketchShape(kind="circle").to_dict())

    def test_from_dict_missing_is_none(self):
        d = SketchShape(kind="line", points=[(0, 0), (1, 1)]).to_dict()
        self.assertIsNone(SketchShape.from_dict(d).start_point)

    def test_sketch_roundtrip_carries_start_point(self):
        sk = Sketch(shapes=[SketchShape(kind="rect", start_point=(2.0, 3.0))])
        sk2 = Sketch.from_dict(sk.to_dict())
        self.assertEqual(sk2.shapes[0].start_point, (2.0, 3.0))


# ═══════════════════════════════════════════════════════════════════
# optimize_print_order
# ═══════════════════════════════════════════════════════════════════

class TestOptimizePrintOrder(unittest.TestCase):

    def _sk(self, shapes) -> Sketch:
        return Sketch(shapes=shapes, line_spacing_mm=0.4)

    def test_scrambled_chain_optimizes_to_zero(self):
        # L1->L2->L3 chain, stored scrambled [L1, L3, L2] → 2 travels.
        L1 = _line(0, 0, 10, 0)
        L2 = _line(10, 0, 10, 10)
        L3 = _line(10, 10, 0, 10)
        sk = self._sk([L1, L3, L2])
        self.assertEqual(count_discontinuities(sk), 2)
        opt = optimize_print_order(sk)
        self.assertEqual(count_discontinuities(opt), 0)

    def test_far_apart_cannot_improve(self):
        sk = self._sk([_line(0, 0, 10, 0), _line(50, 50, 60, 60),
                       _line(-30, -30, -20, -20)])
        before = count_discontinuities(sk)
        after = count_discontinuities(optimize_print_order(sk))
        self.assertEqual(before, 2)
        self.assertLessEqual(after, before)

    def test_travel_points_stay_fixed_breaks(self):
        L1 = _line(0, 0, 10, 0)
        L2 = _line(10, 0, 10, 10)
        L3 = _line(10, 10, 0, 10)
        tr = SketchShape(kind="travel", cx=99, cy=99)
        opt = optimize_print_order(self._sk([L1, tr, L3, L2]))
        self.assertEqual([s.kind for s in opt.shapes],
                         ["line", "travel", "line", "line"])

    def test_single_shape_is_noop(self):
        sk = self._sk([_line(0, 0, 5, 0)])
        opt = optimize_print_order(sk)
        self.assertEqual(len(opt.shapes), 1)

    def test_preserves_print_params(self):
        sk = Sketch(shapes=[_line(0, 0, 1, 0), _line(2, 0, 3, 0)],
                    z_start_mm=0.5, layer_height_mm=0.3, num_layers=4,
                    print_speed_mm_s=7.0, extrusion_multiplier=1.7)
        opt = optimize_print_order(sk)
        self.assertEqual(opt.z_start_mm, 0.5)
        self.assertEqual(opt.layer_height_mm, 0.3)
        self.assertEqual(opt.num_layers, 4)
        self.assertEqual(opt.print_speed_mm_s, 7.0)
        self.assertEqual(opt.extrusion_multiplier, 1.7)

    def test_returns_new_sketch_leaves_original(self):
        L1 = _line(0, 0, 10, 0)
        L2 = _line(10, 0, 10, 10)
        L3 = _line(10, 10, 0, 10)
        sk = self._sk([L1, L3, L2])
        _ = optimize_print_order(sk)
        # Original order untouched (optimizer returns a copy).
        self.assertEqual([tuple(s.points[0]) for s in sk.shapes],
                         [(0, 0), (10, 10), (10, 0)])


# ═══════════════════════════════════════════════════════════════════
# count_discontinuities
# ═══════════════════════════════════════════════════════════════════

class TestCountDiscontinuities(unittest.TestCase):

    def test_empty_is_zero(self):
        self.assertEqual(count_discontinuities(Sketch(shapes=[])), 0)

    def test_single_continuous_run_is_zero(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0), _line(10, 0, 10, 10)],
                    line_spacing_mm=0.4)
        self.assertEqual(count_discontinuities(sk), 0)

    def test_two_disconnected_is_one(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0), _line(50, 50, 60, 60)],
                    line_spacing_mm=0.4)
        self.assertEqual(count_discontinuities(sk), 1)


# ═══════════════════════════════════════════════════════════════════
# Canvas (offscreen)
# ═══════════════════════════════════════════════════════════════════

class TestCanvasStartMarker(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _canvas(self, shapes):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        c.set_sketch(Sketch(shapes=shapes, line_spacing_mm=0.4))
        c.fit_view()
        return c

    def test_supports_start_only_for_outlines(self):
        from gui.widgets.sketch_canvas import SketchCanvas
        self.assertTrue(SketchCanvas._shape_supports_start(
            SketchShape(kind="circle")))
        self.assertTrue(SketchCanvas._shape_supports_start(
            _line(0, 0, 1, 1)))
        self.assertFalse(SketchCanvas._shape_supports_start(
            SketchShape(kind="circle", filled=True)))
        self.assertFalse(SketchCanvas._shape_supports_start(
            SketchShape(kind="region", points=[(0, 0), (1, 1)])))
        self.assertFalse(SketchCanvas._shape_supports_start(
            SketchShape(kind="travel", cx=1, cy=1)))

    def test_effective_start_default_and_resolved(self):
        c = self._canvas([SketchShape(kind="circle", cx=0, cy=0, radius=5)])
        sh = c.sketch().shapes[0]
        d = c._effective_start_world(sh)            # default = angle 0 → (5,0)
        self.assertAlmostEqual(d.x(), 5.0, places=6)
        self.assertAlmostEqual(d.y(), 0.0, places=6)
        sh.start_point = (0.0, 5.0)                 # resolves to the top
        r = c._effective_start_world(sh)
        self.assertLess(math.dist((r.x(), r.y()), (0.0, 5.0)), 1e-6)

    def test_start_handle_hit_test(self):
        c = self._canvas([SketchShape(kind="circle", cx=0, cy=0, radius=5)])
        c.set_selected(0)
        sh = c.selected_shape()
        marker = c._start_marker_screen(sh)
        self.assertTrue(c._start_handle_at(sh, marker))
        # A point far from the marker is not a hit.
        from PySide6.QtCore import QPointF
        self.assertFalse(c._start_handle_at(
            sh, QPointF(marker.x() + 500, marker.y() + 500)))

    def test_press_routes_to_start_resize(self):
        from PySide6.QtCore import Qt
        c = self._canvas([SketchShape(kind="circle", cx=0, cy=0, radius=5)])
        c.set_selected(0)
        sh = c.selected_shape()
        marker = c._start_marker_screen(sh)
        world = c._s2w(marker.x(), marker.y())
        c._press_select(marker, world, Qt.NoModifier)
        self.assertEqual(c._mode, "resize")
        self.assertEqual(c._resize_kind, "start")

    def test_apply_resize_start_sets_point(self):
        from PySide6.QtCore import QPointF
        c = self._canvas([SketchShape(kind="circle", cx=0, cy=0, radius=5)])
        c.set_selected(0)
        c._mode = "resize"
        c._resize_kind = "start"
        c._apply_resize(QPointF(0.0, -5.0))
        self.assertEqual(c.sketch().shapes[0].start_point, (0.0, -5.0))

    def test_clear_start_point(self):
        c = self._canvas([SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                      start_point=(0.0, 5.0))])
        c.clear_start_point(0)
        self.assertIsNone(c.sketch().shapes[0].start_point)

    def test_optimize_reduces_and_is_undoable(self):
        c = self._canvas([_line(0, 0, 10, 0), _line(10, 10, 0, 10),
                          _line(10, 0, 10, 10)])
        before = count_discontinuities(c.sketch())
        c.optimize()
        after = count_discontinuities(c.sketch())
        self.assertLess(after, before)
        self.assertTrue(c.can_undo())


# ═══════════════════════════════════════════════════════════════════
# Page (offscreen)
# ═══════════════════════════════════════════════════════════════════

class TestPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_optimize_path_reduces_travels(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            _line(0, 0, 10, 0), _line(10, 10, 0, 10), _line(10, 0, 10, 10),
        ], line_spacing_mm=0.4))
        before = count_discontinuities(page._canvas.sketch())
        page._optimize_path()
        after = count_discontinuities(page._canvas.sketch())
        self.assertLess(after, before)
        self.assertIn("Optimized", page._status_lbl.text())

    def test_optimize_path_needs_two_shapes(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[_line(0, 0, 5, 0)]))
        page._optimize_path()
        self.assertIn("at least two", page._status_lbl.text())

    def test_shape_card_shows_print_start_row(self):
        from PySide6.QtWidgets import QLabel
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[_line(0, 0, 5, 0)]))
        page._canvas.set_selected(0)                 # → _rebuild_props
        texts = [w.text() for w in page._props_host.findChildren(QLabel)]
        self.assertTrue(any("Print start" in t for t in texts))

    def test_reset_start_point_button_clears(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="circle", cx=0, cy=0, radius=5,
                        start_point=(0.0, 5.0))]))
        page._canvas.set_selected(0)
        page._reset_start_point()
        self.assertIsNone(page._canvas.sketch().shapes[0].start_point)


if __name__ == "__main__":
    unittest.main()
