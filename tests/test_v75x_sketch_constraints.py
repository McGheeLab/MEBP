"""
test_v75x_sketch_constraints.py — parametric constraints for the Print
Builder Sketch tool (v7.5.x).

Covers:
- the SketchConstraint / shape-id data model + serialization round-trip and
  the legacy byte-identity guarantee (no constraints → no new keys),
- the SketchConstraintSolver residual kinds (coincident, horizontal,
  vertical, parallel, perpendicular, equal length/radius, tangent,
  concentric, distance/radius dimensions, point_on, fix) + the drag-ghost
  lifecycle + DOF reporting,
- id hygiene (backtrace copies, optimizer carry-through, delete pruning),
- the SketchCanvas constraint API (selection-based creation, auto-capture
  from snap provenance, delete cascade),
- the SketchPage Constraints card (buttons gate on selection, rows render,
  DOF label).

See coding plans/Update plans/MEBP_v75x_SKETCH_PARAMETRIC_CONSTRAINTS.md.
"""

import math
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.SketchTrajectory import (   # noqa: E402
    Sketch, SketchShape, SketchConstraint, backtrace_shape,
    optimize_print_order,
)
from SupportClasses.SketchConstraintSolver import (   # noqa: E402
    SketchConstraintSolver, tangent_mode_for,
)
from SupportClasses.PlateSketchSolver import DOFStatus   # noqa: E402


def _line(p0, p1, **kw):
    return SketchShape(kind="line", points=[tuple(p0), tuple(p1)], **kw)


def _circle(cx, cy, r, **kw):
    return SketchShape(kind="circle", cx=cx, cy=cy, radius=r, **kw)


# ═══════════════════════════════════════════════════════════════════
# Model + serialization
# ═══════════════════════════════════════════════════════════════════

class TestConstraintModel(unittest.TestCase):

    def test_legacy_byte_identity_without_constraints(self):
        """A sketch that never uses constraints must serialize with NO new
        keys — no per-shape 'id', no 'constraints' key."""
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _circle(2, 2, 3)])
        d = sk.to_dict()
        self.assertNotIn("constraints", d)
        for sd in d["shapes"]:
            self.assertNotIn("id", sd)

    def test_ensure_shape_ids_lazy_and_collision_safe(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _circle(2, 2, 3)])
        self.assertEqual([s.id for s in sk.shapes], [0, 0])
        sk.shapes[1].id = 7                     # pre-existing explicit id
        sk.ensure_shape_ids()
        self.assertEqual(sk.shapes[1].id, 7)
        self.assertNotEqual(sk.shapes[0].id, 0)
        self.assertNotEqual(sk.shapes[0].id, 7)
        ids = [s.id for s in sk.shapes]
        self.assertEqual(len(ids), len(set(ids)))

    def test_round_trip_with_constraints(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _circle(2, 2, 3)])
        sk.ensure_shape_ids()
        a, b = sk.shapes
        sk.add_constraint("coincident", [[a.id, "p1"], [b.id, "center"]])
        sk.add_constraint("radius", [[b.id, "shape"]], value=3.5)
        sk.add_constraint("tangent", [[a.id, "shape"], [b.id, "shape"]],
                          mode="external")
        d = sk.to_dict()
        sk2 = Sketch.from_dict(d)
        self.assertEqual(len(sk2.constraints), 3)
        c0, c1, c2 = sk2.constraints
        self.assertEqual(c0.refs, [[a.id, "p1"], [b.id, "center"]])
        self.assertEqual(c1.value, 3.5)
        self.assertEqual(c2.mode, "external")
        # Allocators re-derived past the loaded maxima.
        self.assertGreater(sk2._next_shape_id, max(a.id, b.id))
        self.assertGreater(sk2._next_constraint_id, c2.id)

    def test_prune_constraints_on_shape_removal(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _circle(2, 2, 3)])
        sk.ensure_shape_ids()
        a, b = sk.shapes
        sk.add_constraint("coincident", [[a.id, "p0"], [b.id, "center"]])
        sk.add_constraint("radius", [[b.id, "shape"]], value=2.0)
        sk.shapes.pop(0)                        # remove the line
        removed = sk.prune_constraints()
        self.assertEqual(removed, 1)
        self.assertEqual(len(sk.constraints), 1)
        self.assertEqual(sk.constraints[0].kind, "radius")

    def test_prune_drops_stale_vertex_anchor(self):
        sk = Sketch(shapes=[SketchShape(
            kind="polygon", points=[(0, 0), (5, 0), (5, 5)])])
        sk.ensure_shape_ids()
        p = sk.shapes[0]
        sk.add_constraint("horizontal", [[p.id, "p0"], [p.id, "p2"]])
        p.points = p.points[:2]                 # vertex p2 no longer exists
        self.assertEqual(sk.prune_constraints(), 1)

    def test_backtrace_copy_clears_id(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0))])
        sk.ensure_shape_ids()
        src = sk.shapes[0]
        copy_sh = backtrace_shape(src, z_offset=0.2)
        self.assertNotEqual(src.id, 0)
        self.assertEqual(copy_sh.id, 0)

    def test_optimizer_carries_ids_and_constraints(self):
        """optimize_print_order reorders the shape LIST — ids + constraints
        must survive and still resolve to the same shapes."""
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _line((20, 0), (25, 0)),
                            _line((5, 0), (5, 5))])
        sk.ensure_shape_ids()
        a = sk.shapes[0]
        sk.add_constraint("horizontal", [[a.id, "p0"], [a.id, "p1"]])
        out = optimize_print_order(sk)
        self.assertEqual(len(out.constraints), 1)
        sid = out.constraints[0].refs[0][0]
        sh = out.shape_by_id(sid)
        self.assertIsNotNone(sh)
        self.assertEqual(sh.points[0][:2], (0, 0))   # same shape, wherever it moved


# ═══════════════════════════════════════════════════════════════════
# Solver
# ═══════════════════════════════════════════════════════════════════

class TestSolver(unittest.TestCase):

    def _solve(self, sk):
        return SketchConstraintSolver(sk).solve()

    def test_coincident_endpoints(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _line((6, 1), (10, 4))])
        sk.ensure_shape_ids()
        a, b = sk.shapes
        sk.add_constraint("coincident", [[a.id, "p1"], [b.id, "p0"]])
        self._solve(sk)
        self.assertLess(math.dist(a.points[1], b.points[0]), 1e-5)

    def test_horizontal_vertical(self):
        sk = Sketch(shapes=[_line((0, 0), (8, 3)), _line((1, 1), (2, 9))])
        sk.ensure_shape_ids()
        h, v = sk.shapes
        sk.add_constraint("horizontal", [[h.id, "p0"], [h.id, "p1"]])
        sk.add_constraint("vertical", [[v.id, "p0"], [v.id, "p1"]])
        self._solve(sk)
        self.assertAlmostEqual(h.points[0][1], h.points[1][1], places=5)
        self.assertAlmostEqual(v.points[0][0], v.points[1][0], places=5)

    def test_parallel_perpendicular(self):
        sk = Sketch(shapes=[_line((0, 0), (10, 0)), _line((0, 5), (9, 7)),
                            _line((3, 3), (11, 2))])
        sk.ensure_shape_ids()
        l1, l2, l3 = sk.shapes
        sk.add_constraint("parallel", [[l1.id, "shape"], [l2.id, "shape"]])
        sk.add_constraint("perpendicular", [[l1.id, "shape"], [l3.id, "shape"]])
        self._solve(sk)

        def d(ln):
            return (ln.points[1][0] - ln.points[0][0],
                    ln.points[1][1] - ln.points[0][1])
        d1, d2, d3 = d(l1), d(l2), d(l3)
        self.assertAlmostEqual(d1[0] * d2[1] - d1[1] * d2[0], 0.0, places=4)
        self.assertAlmostEqual(d1[0] * d3[0] + d1[1] * d3[1], 0.0, places=4)

    def test_equal_length_and_equal_radius(self):
        sk = Sketch(shapes=[_line((0, 0), (10, 0)), _line((0, 5), (6, 5)),
                            _circle(0, 20, 3), _circle(10, 20, 5)])
        sk.ensure_shape_ids()
        l1, l2, c1, c2 = sk.shapes
        sk.add_constraint("equal_length", [[l1.id, "shape"], [l2.id, "shape"]])
        sk.add_constraint("equal_radius", [[c1.id, "shape"], [c2.id, "shape"]])
        self._solve(sk)
        self.assertAlmostEqual(math.dist(*l1.points), math.dist(*l2.points),
                               places=4)
        self.assertAlmostEqual(c1.radius, c2.radius, places=4)

    def test_tangent_line_circle(self):
        sk = Sketch(shapes=[_line((0, 0), (10, 0)), _circle(5, 4, 2.5)])
        sk.ensure_shape_ids()
        ln, ci = sk.shapes
        sk.add_constraint("tangent", [[ln.id, "shape"], [ci.id, "shape"]])
        self._solve(sk)
        ax, ay = ln.points[0]
        bx, by = ln.points[1]
        dist = abs((ci.cx - ax) * (by - ay) - (ci.cy - ay) * (bx - ax)) \
            / math.hypot(bx - ax, by - ay)
        self.assertAlmostEqual(dist, ci.radius, places=3)

    def test_tangent_circle_circle_modes(self):
        # External: centers apart.
        sk = Sketch(shapes=[_circle(0, 0, 3), _circle(10, 0, 2)])
        sk.ensure_shape_ids()
        c1, c2 = sk.shapes
        self.assertEqual(tangent_mode_for(c1, c2), "external")
        sk.add_constraint("tangent", [[c1.id, "shape"], [c2.id, "shape"]],
                          mode="external")
        self._solve(sk)
        self.assertAlmostEqual(math.hypot(c1.cx - c2.cx, c1.cy - c2.cy),
                               c1.radius + c2.radius, places=4)
        # Internal: one nested in the other.
        sk = Sketch(shapes=[_circle(0, 0, 5), _circle(1, 0, 2)])
        sk.ensure_shape_ids()
        c1, c2 = sk.shapes
        self.assertEqual(tangent_mode_for(c1, c2), "internal")
        sk.add_constraint("tangent", [[c1.id, "shape"], [c2.id, "shape"]],
                          mode="internal")
        self._solve(sk)
        self.assertAlmostEqual(math.hypot(c1.cx - c2.cx, c1.cy - c2.cy),
                               abs(c1.radius - c2.radius), places=4)

    def test_concentric(self):
        sk = Sketch(shapes=[_circle(0, 0, 3), _circle(8, 2, 1)])
        sk.ensure_shape_ids()
        c1, c2 = sk.shapes
        sk.add_constraint("concentric", [[c1.id, "center"], [c2.id, "center"]])
        self._solve(sk)
        self.assertLess(math.hypot(c1.cx - c2.cx, c1.cy - c2.cy), 1e-5)

    def test_distance_and_radius_dimensions_hold_value(self):
        sk = Sketch(shapes=[_circle(0, 0, 3), _circle(4, 0, 2)])
        sk.ensure_shape_ids()
        c1, c2 = sk.shapes
        sk.add_constraint("distance", [[c1.id, "center"], [c2.id, "center"]],
                          value=12.0)
        sk.add_constraint("radius", [[c1.id, "shape"]], value=4.5)
        self._solve(sk)
        self.assertAlmostEqual(math.hypot(c1.cx - c2.cx, c1.cy - c2.cy),
                               12.0, places=4)
        self.assertAlmostEqual(c1.radius, 4.5, places=5)

    def test_point_on_line_and_circle(self):
        sk = Sketch(shapes=[_line((0, 0), (10, 0)), _circle(20, 5, 3),
                            SketchShape(kind="travel", cx=4, cy=6)])
        sk.ensure_shape_ids()
        ln, ci, tp = sk.shapes
        sk.add_constraint("point_on", [[tp.id, "center"], [ln.id, "shape"]])
        self._solve(sk)
        ax, ay = ln.points[0]
        bx, by = ln.points[1]
        cross = (tp.cx - ax) * (by - ay) - (tp.cy - ay) * (bx - ax)
        self.assertAlmostEqual(cross, 0.0, places=3)

        sk2 = Sketch(shapes=[_circle(0, 0, 5),
                             SketchShape(kind="travel", cx=8, cy=1)])
        sk2.ensure_shape_ids()
        ci2, tp2 = sk2.shapes
        sk2.add_constraint("point_on", [[tp2.id, "center"], [ci2.id, "shape"]])
        SketchConstraintSolver(sk2).solve()
        self.assertAlmostEqual(math.hypot(tp2.cx - ci2.cx, tp2.cy - ci2.cy),
                               ci2.radius, places=4)

    def test_fix_pins_shape_and_partner_moves(self):
        sk = Sketch(shapes=[_circle(0, 0, 3), _circle(8, 0, 3)])
        sk.ensure_shape_ids()
        c1, c2 = sk.shapes
        sk.add_constraint("concentric", [[c1.id, "center"], [c2.id, "center"]])
        sk.add_constraint("fix", [[c1.id, "shape"]])
        self._solve(sk)
        self.assertEqual((c1.cx, c1.cy), (0, 0))       # fixed: never moves
        self.assertLess(math.hypot(c2.cx, c2.cy), 1e-5)  # partner snapped to it

    def test_drag_ghost_lifecycle(self):
        """Dragging one endpoint pulls the coincident partner along."""
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _line((5, 0), (10, 3))])
        sk.ensure_shape_ids()
        a, b = sk.shapes
        sk.add_constraint("coincident", [[a.id, "p1"], [b.id, "p0"]])
        sol = SketchConstraintSolver(sk)
        sol.begin_drag(a.id, "p1", (5, 0))
        sol.update_drag((7.0, 2.0))
        rep = sol.end_drag()
        self.assertLess(math.dist(a.points[1], (7.0, 2.0)), 1e-2)
        self.assertLess(math.dist(a.points[1], b.points[0]), 1e-5)
        # Ghost never persisted on the model.
        self.assertTrue(all(c.kind != "drag_ghost" for c in sk.constraints))
        self.assertIsNotNone(rep)

    def test_drag_fixed_shape_moves_nothing(self):
        sk = Sketch(shapes=[_circle(0, 0, 3)])
        sk.ensure_shape_ids()
        c1 = sk.shapes[0]
        sk.add_constraint("fix", [[c1.id, "shape"]])
        sol = SketchConstraintSolver(sk)
        sol.begin_drag(c1.id, "center", (50, 50))
        sol.end_drag()
        self.assertEqual((c1.cx, c1.cy), (0, 0))

    def test_rect_corner_anchor(self):
        sk = Sketch(shapes=[SketchShape(kind="rect", cx=0, cy=0,
                                        width=4, height=4),
                            _circle(10, 10, 1)])
        sk.ensure_shape_ids()
        rc, ci = sk.shapes
        sk.add_constraint("coincident", [[rc.id, "c2"], [ci.id, "center"]])
        self._solve(sk)
        br = (rc.cx + rc.width / 2, rc.cy + rc.height / 2)
        self.assertLess(math.dist(br, (ci.cx, ci.cy)), 1e-4)

    def test_inconsistent_reports_conflicts(self):
        """Two contradictory radius dimensions on one circle → INCONSISTENT
        + the conflicting constraint ids surfaced."""
        sk = Sketch(shapes=[_circle(0, 0, 3)])
        sk.ensure_shape_ids()
        c1 = sk.shapes[0]
        sk.add_constraint("radius", [[c1.id, "shape"]], value=2.0)
        sk.add_constraint("radius", [[c1.id, "shape"]], value=8.0)
        rep = self._solve(sk)
        self.assertEqual(rep.status, DOFStatus.INCONSISTENT)
        self.assertTrue(rep.conflicts)

    def test_unreferenced_shapes_never_move(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0)), _line((6, 1), (10, 4)),
                            _circle(50, 50, 4)])
        sk.ensure_shape_ids()
        a, b, c = sk.shapes
        sk.add_constraint("coincident", [[a.id, "p1"], [b.id, "p0"]])
        self._solve(sk)
        self.assertEqual((c.cx, c.cy, c.radius), (50, 50, 4))

    def test_no_constraints_solver_is_noop(self):
        sk = Sketch(shapes=[_line((0, 0), (5, 0))])
        rep = SketchConstraintSolver(sk).solve()
        self.assertEqual(rep.status, DOFStatus.EMPTY)


# ═══════════════════════════════════════════════════════════════════
# Canvas integration (offscreen)
# ═══════════════════════════════════════════════════════════════════

def _canvas(shapes):
    from PySide6.QtWidgets import QApplication
    global _APP
    _APP = QApplication.instance()
    if _APP is None:
        _APP = QApplication(sys.argv)
    from gui.widgets.sketch_canvas import SketchCanvas
    c = SketchCanvas()
    c.resize(600, 600)
    c.set_sketch(Sketch(shapes=shapes))
    return c


class TestCanvasConstraints(unittest.TestCase):

    def test_join_resolves_nearest_anchor_pair(self):
        c = _canvas([_line((0, 0), (5, 0)), _line((6, 1), (10, 4))])
        c.select_indices([0, 1])
        ok, msg = c.add_constraint_for_selection("coincident")
        self.assertTrue(ok, msg)
        sk = c.sketch()
        self.assertEqual(len(sk.constraints), 1)
        # Nearest pair is line0.p1 ↔ line1.p0 — welded after the solve.
        self.assertLess(math.dist(sk.shapes[0].points[1],
                                  sk.shapes[1].points[0]), 1e-4)

    def test_can_add_constraint_gating(self):
        c = _canvas([_line((0, 0), (5, 0)), _line((6, 1), (10, 4)),
                     _circle(3, 8, 2)])
        c.select_indices([0, 1])
        self.assertTrue(c.can_add_constraint("parallel"))
        self.assertTrue(c.can_add_constraint("perpendicular"))
        self.assertFalse(c.can_add_constraint("equal_radius"))
        self.assertFalse(c.can_add_constraint("radius"))
        c.select_indices([2])
        self.assertTrue(c.can_add_constraint("radius"))
        self.assertTrue(c.can_add_constraint("fix"))
        self.assertFalse(c.can_add_constraint("tangent"))
        c.select_indices([0, 2])
        self.assertTrue(c.can_add_constraint("tangent"))
        self.assertTrue(c.can_add_constraint("point_on"))
        c.clear_selection()
        self.assertFalse(c.can_add_constraint("coincident"))

    def test_dry_run_can_add_does_not_mutate(self):
        c = _canvas([_line((0, 0), (5, 0)), _line((6, 1), (10, 4))])
        c.select_indices([0, 1])
        before = c.sketch().to_dict()
        self.assertTrue(c.can_add_constraint("coincident"))
        self.assertEqual(c.sketch().to_dict(), before)

    def test_fix_toggle(self):
        c = _canvas([_circle(0, 0, 3)])
        c.select_indices([0])
        ok, msg = c.add_constraint_for_selection("fix")
        self.assertTrue(ok)
        self.assertEqual(len(c.sketch().constraints), 1)
        ok, msg = c.add_constraint_for_selection("fix")   # toggles off
        self.assertTrue(ok)
        self.assertEqual(len(c.sketch().constraints), 0)

    def test_delete_cascade_prunes(self):
        c = _canvas([_line((0, 0), (5, 0)), _line((6, 1), (10, 4))])
        c.select_indices([0, 1])
        c.add_constraint_for_selection("parallel")
        c.select_indices([1])
        c.delete_selected()
        self.assertEqual(len(c.sketch().constraints), 0)

    def test_auto_capture_vertex_snap_on_draw(self):
        """A committed line whose endpoint snapped onto an existing vertex
        gains a coincident constraint (auto-capture)."""
        c = _canvas([_line((0, 0), (5, 0))])
        sk = c.sketch()
        sk.shapes.append(_line((5, 0), (9, 3)))     # drawn endpoint AT p1
        new_idx = len(sk.shapes) - 1
        added = c._maybe_add_snap_constraint(new_idx, "p0", ("vertex", 0, "p1"))
        self.assertTrue(added)
        self.assertEqual(sk.constraints[0].kind, "coincident")
        # Dedup: same capture again is refused.
        self.assertFalse(
            c._maybe_add_snap_constraint(new_idx, "p0", ("vertex", 0, "p1")))

    def test_auto_capture_edge_snap_becomes_point_on(self):
        c = _canvas([_circle(0, 0, 5)])
        sk = c.sketch()
        sk.shapes.append(_line((5, 0), (9, 3)))
        added = c._maybe_add_snap_constraint(1, "p0", ("edge", 0, None))
        self.assertTrue(added)
        self.assertEqual(sk.constraints[0].kind, "point_on")

    def test_auto_capture_rect_corner_via_commit(self):
        """Drawing a rect whose press-corner snapped onto a vertex captures a
        coincident on the CORRECT corner anchor (press at top-left → c0)."""
        from PySide6.QtCore import QPointF
        from gui.widgets.sketch_canvas import Tool
        c = _canvas([_line((0, 0), (5, 0))])
        c.set_tool(Tool.RECT)
        c._mode = "draw"
        c._draw_start = QPointF(5, 0)          # pressed ON line0.p1 (snapped)
        c._draw_start_hit = ("vertex", 0, "p1")
        c._draw_cur = QPointF(9, 4)            # dragged down-right
        c._snap_hit = None
        c._commit_draw()
        sk = c.sketch()
        self.assertEqual(sk.shapes[1].kind, "rect")
        self.assertEqual(len(sk.constraints), 1)
        self.assertEqual(sk.constraints[0].kind, "coincident")
        self.assertEqual(sk.constraints[0].refs[0][1], "c0")
        # Solved: the rect's TL corner sits exactly on the line endpoint.
        rc = sk.shapes[1]
        tl = (rc.cx - rc.width / 2, rc.cy - rc.height / 2)
        self.assertLess(math.dist(tl, sk.shapes[0].points[1]), 1e-4)

    def test_auto_capture_disabled(self):
        c = _canvas([_line((0, 0), (5, 0))])
        c.set_auto_constrain(False)
        sk = c.sketch()
        sk.shapes.append(_line((5, 0), (9, 3)))
        self.assertFalse(
            c._maybe_add_snap_constraint(1, "p0", ("vertex", 0, "p1")))
        self.assertEqual(len(sk.constraints), 0)

    def test_snap_records_provenance(self):
        from PySide6.QtCore import QPointF
        c = _canvas([_line((0, 0), (5, 0))])
        c.fit_view()
        c._snap(QPointF(5.02, 0.02))
        self.assertIsNotNone(c._snap_hit)
        kind, idx, anchor = c._snap_hit
        self.assertEqual((kind, idx, anchor), ("vertex", 0, "p1"))

    def test_constrained_move_drag_solves_live(self):
        """A move-press on a constrained shape starts a ghost drag; the
        throttled flush pulls the coincident partner along."""
        c = _canvas([_line((0, 0), (5, 0)), _line((5, 0), (10, 3))])
        c.select_indices([0, 1])
        c.add_constraint_for_selection("coincident")
        sk = c.sketch()
        c.set_selected(1)
        c._snapshot()
        c._mode = "move"
        from PySide6.QtCore import QPointF
        c._move_anchor = QPointF(7.5, 1.5)
        c._begin_cdrag_ghost(1, c._drag_anchor_for(sk.shapes[1]))
        self.assertEqual(c._cdrag_mode, "ghost")
        c._queue_cdrag((c._cdrag_anchor_start[0] + 3.0,
                        c._cdrag_anchor_start[1] + 1.0))
        c._flush_cdrag()                        # bypass the 33 ms timer
        c._end_cdrag()
        # Weld held through the drag.
        self.assertLess(math.dist(sk.shapes[0].points[1],
                                  sk.shapes[1].points[0]), 1e-4)

    def test_undo_restores_constraints(self):
        c = _canvas([_line((0, 0), (5, 0)), _line((6, 1), (10, 4))])
        c.select_indices([0, 1])
        c.add_constraint_for_selection("parallel")
        self.assertEqual(len(c.sketch().constraints), 1)
        c.undo()
        self.assertEqual(len(c.sketch().constraints), 0)
        c.redo()
        self.assertEqual(len(c.sketch().constraints), 1)

    def test_glyph_paint_smoke(self):
        from PySide6.QtGui import QPixmap
        c = _canvas([_line((0, 0), (10, 0)), _circle(5, 4, 2.5),
                     _circle(15, 4, 2.0)])
        c.select_indices([0, 1])
        c.add_constraint_for_selection("tangent")
        c.select_indices([1, 2])
        c.add_constraint_for_selection("distance")
        c.select_indices([1])
        c.add_constraint_for_selection("radius")
        c.add_constraint_for_selection("fix")
        pm = QPixmap(400, 400)
        c.render(pm)                            # must not raise

    def test_set_constraint_value_re_solves(self):
        c = _canvas([_circle(0, 0, 3)])
        c.select_indices([0])
        c.add_constraint_for_selection("radius")
        cid = c.sketch().constraints[0].id
        c.set_constraint_value(cid, 6.0)
        self.assertAlmostEqual(c.sketch().shapes[0].radius, 6.0, places=4)

    def test_select_constraint_shapes(self):
        c = _canvas([_line((0, 0), (5, 0)), _line((6, 1), (10, 4)),
                     _circle(3, 8, 2)])
        c.select_indices([0, 1])
        c.add_constraint_for_selection("parallel")
        c.clear_selection()
        c.select_constraint_shapes(c.sketch().constraints[0].id)
        self.assertEqual(c.selected_indices(), [0, 1])

    def test_region_refused(self):
        c = _canvas([SketchShape(kind="region", points=[(0, 0), (5, 0)]),
                     _circle(3, 8, 2)])
        c.select_indices([0, 1])
        ok, msg = c.add_constraint_for_selection("coincident")
        self.assertFalse(ok)


# ═══════════════════════════════════════════════════════════════════
# Page card (offscreen)
# ═══════════════════════════════════════════════════════════════════

class TestPageCard(unittest.TestCase):

    def _page(self):
        from PySide6.QtWidgets import QApplication
        global _APP
        _APP = QApplication.instance()
        if _APP is None:
            _APP = QApplication(sys.argv)
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_card_builds_and_buttons_gate(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            _line((0, 0), (5, 0)), _line((6, 1), (10, 4)), _circle(3, 8, 2)]))
        self.assertEqual(len(page._constraint_btns),
                         len(page._CONSTRAINT_BUTTONS))
        page._canvas.clear_selection()
        page._refresh_constraint_buttons()
        self.assertFalse(any(b.isEnabled()
                             for b in page._constraint_btns.values()))
        page._canvas.select_indices([0, 1])
        page._refresh_constraint_buttons()
        self.assertTrue(page._constraint_btns["parallel"].isEnabled())
        self.assertTrue(page._constraint_btns["coincident"].isEnabled())
        self.assertFalse(page._constraint_btns["equal_radius"].isEnabled())

    def test_add_and_list_rows(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            _line((0, 0), (5, 0)), _line((6, 1), (10, 4))]))
        page._canvas.select_indices([0, 1])
        page._add_constraint("parallel")
        page._refresh_constraints_card()
        self.assertEqual(page._constraints_layout.count(), 1)
        self.assertIn("constraint", page._dof_lbl.text())
        # Remove through the canvas API → card empties on refresh.
        cid = page._canvas.sketch().constraints[0].id
        page._canvas.remove_constraint(cid)
        page._refresh_constraints_card()
        self.assertEqual(page._constraints_layout.count(), 0)

    def test_auto_capture_toggle_wires_canvas(self):
        page = self._page()
        self.assertTrue(page._canvas.auto_constrain())
        page._auto_capture_chk.setChecked(False)
        self.assertFalse(page._canvas.auto_constrain())

    def test_format_constraint_labels(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            _line((0, 0), (5, 0)), _circle(3, 8, 2)]))
        sk = page._canvas.sketch()
        sk.ensure_shape_ids()
        c = sk.add_constraint(
            "tangent", [[sk.shapes[0].id, "shape"], [sk.shapes[1].id, "shape"]],
            mode="external")
        text = page._format_constraint(c)
        self.assertIn("Tangent", text)
        self.assertIn("line #1", text)
        self.assertIn("circle #2", text)


if __name__ == "__main__":
    unittest.main()
