"""
test_v7214_sketch_single_pass_join_noextrude.py — three Print-Builder Sketch
changes (v7.21.4), all operator-requested:

1. *"do not auto calculate the need for multiple lines to fill in the width of
   any line. We will set that line width and just increase the extrusion
   modifier to match."* → an outline is ALWAYS one pass on the drawn geometry;
   width comes from flow, and the page prices/applies the multiplier needed.

2. *"we want to be able to join a line to the start or stop position of any
   existing print object"* → a shape's PRINT start / stop are snap targets, and
   a line drawn onto one is re-ordered so the two actually print as one bead.

3. *"we want to have the ability to do a no extrude on any section we want"* →
   ``no_print`` (previously only ever set by a back-trace) is exposed per shape,
   per multi-selection and per print SECTION.

Runs offscreen (no hardware).

See coding plans/Update plans/MEBP_v7214_SKETCH_SINGLE_PASS_JOIN_NOEXTRUDE.md.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                           # noqa: E402
from PySide6.QtCore import QPointF                            # noqa: E402
from PySide6.QtWidgets import (                               # noqa: E402
    QApplication, QCheckBox, QLabel, QPushButton, QToolButton,
)

from SupportClasses.SketchTrajectory import (                 # noqa: E402
    Sketch, SketchShape, _pass_offsets, _shape_paths,
    compile_to_trajectory, plan_print_sections,
)
from gui.widgets.sketch_canvas import SketchCanvas, Tool      # noqa: E402


def _line(x1, y1, x2, y2, **kw):
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)], **kw)


def _canvas(shapes=(), **sk_kw):
    c = SketchCanvas()
    c.set_sketch(Sketch(shapes=list(shapes), **sk_kw))
    c.resize(600, 600)
    c.fit_view()
    return c


def _draw_line(canvas, a, b, press_hit=None, release_hit=None):
    """Drive the production commit path for a LINE drawn a->b with the given
    snap provenance (what ``_snap`` would have recorded at press / release)."""
    canvas.set_tool(Tool.LINE)
    canvas._mode = "draw"
    canvas._draw_start = QPointF(*a)
    canvas._draw_cur = QPointF(*b)
    canvas._draw_start_hit = press_hit
    canvas._snap_hit = release_hit
    canvas._commit_draw()


# ═══════════════════════════════════════════════════════════════════
# 1 — one pass per outline; width comes from the extrusion multiplier
# ═══════════════════════════════════════════════════════════════════

class TestSinglePassOutline(unittest.TestCase):

    def test_pass_offsets_is_always_one_centered_pass(self):
        for lw in (0.05, 0.4, 1.0, 2.0, 12.0, 50.0):
            for bead in (0.1, 0.4, 1.0):
                self.assertEqual(_pass_offsets(lw, bead), [0.0],
                                 f"lw={lw} bead={bead}")

    def test_a_wide_outline_emits_one_path_not_several(self):
        for kind, kw in (("line", dict(points=[(0, 0), (10, 0)])),
                         ("circle", dict(cx=0, cy=0, radius=5)),
                         ("ellipse", dict(cx=0, cy=0, rx=5, ry=3)),
                         ("rect", dict(cx=0, cy=0, width=8, height=6))):
            sk = Sketch(shapes=[SketchShape(kind=kind, line_width_mm=3.0,
                                            **kw)],
                        line_spacing_mm=0.4)
            self.assertEqual(len(_shape_paths(sk.shapes[0], sk)), 1, kind)

    def test_the_printed_geometry_stays_on_what_was_drawn(self):
        """The old multi-pass expansion put the outer pass half a width OUTSIDE
        the sketched shape. A single pass sits exactly on the drawn radius."""
        sk = Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=5.0,
                                        line_width_mm=3.0)],
                    line_spacing_mm=0.4)
        path = _shape_paths(sk.shapes[0], sk)[0]
        radii = np.hypot(path[:, 0], path[:, 1])
        self.assertLess(abs(float(radii.max()) - 5.0), 1e-6)
        self.assertLess(abs(float(radii.min()) - 5.0), 1e-6)

    def test_width_changes_the_FLOW_and_never_the_geometry(self):
        """v7.21.5 refines this: the declared width still never changes the
        toolpath (one pass, same length), but it now drives that shape's own
        extrusion — 10x the width is 10x the deposition, which is what makes the
        per-segment profile sent to Quick Print mean anything."""
        def _c(lw):
            return compile_to_trajectory(
                Sketch(shapes=[_line(0, 0, 20, 0, line_width_mm=lw)],
                       line_spacing_mm=0.4))
        thin, thick = _c(0.4), _c(4.0)
        self.assertAlmostEqual(thick.total_length_mm, thin.total_length_mm, 6)
        self.assertAlmostEqual(thick.total_volume_uL,
                               thin.total_volume_uL * 10.0, places=9)

    def test_the_extrusion_multiplier_is_what_scales_deposition(self):
        def _vol(mult):
            return compile_to_trajectory(
                Sketch(shapes=[_line(0, 0, 20, 0)], line_spacing_mm=0.4,
                       extrusion_multiplier=mult)).total_volume_uL
        base = _vol(1.0)
        self.assertGreater(base, 0.0)
        self.assertAlmostEqual(_vol(3.0), base * 3.0, places=9)

    def test_guard_the_guard_a_fill_still_emits_many_lines(self):
        """The single-pass rule is about OUTLINE width, not about rasters — a
        filled shape still lays as many lines as its pitch requires, else this
        suite would pass with fills broken."""
        sk = Sketch(shapes=[SketchShape(kind="rect", cx=0, cy=0, width=10,
                                        height=10, filled=True)],
                    line_spacing_mm=0.5)
        pts = _shape_paths(sk.shapes[0], sk)[0]
        self.assertGreater(len(pts), 40)


class TestWidthMatchesExtrusionOnThePage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, sk=None):
        from gui.pages.print_builder_sketch import SketchPage
        p = SketchPage()
        if sk is not None:
            p._canvas.set_sketch(sk)
        return p

    def test_the_card_prices_the_multiplier_this_width_needs(self):
        # v7.21.5: this manual "match" affordance is the FALLBACK — it shows
        # when the sketch is not letting each width drive its own extrusion.
        sk = Sketch(shapes=[_line(0, 0, 10, 0, line_width_mm=1.2)],
                    line_spacing_mm=0.4, extrusion_multiplier=1.0,
                    width_drives_extrusion=False)
        p = self._page(sk)
        p._canvas.set_selected(0)
        p._rebuild_props()
        texts = [w.text() for w in p.findChildren(QPushButton)]
        self.assertTrue(any("3.00" in t and "match" in t for t in texts),
                        f"no 3.00x match button in {texts}")

    def test_the_button_sets_the_multiplier_so_one_bead_is_that_wide(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0, line_width_mm=0.8)],
                    line_spacing_mm=0.4, extrusion_multiplier=1.0,
                    width_drives_extrusion=False)
        p = self._page(sk)
        p._canvas.set_selected(0)
        p._rebuild_props()
        btns = [w for w in p.findChildren(QPushButton) if "match" in w.text()]
        self.assertEqual(len(btns), 1)
        btns[0].click()
        ref = p._bead_ref_mm()
        self.assertGreater(ref, 0.0)
        self.assertAlmostEqual(
            p._canvas.sketch().extrusion_multiplier * ref, 0.8, places=6)

    def test_the_shaded_band_follows_the_multiplier(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0, line_width_mm=0.8)],
                    line_spacing_mm=0.4, extrusion_multiplier=1.0)
        p = self._page(sk)
        p._canvas.set_selected(0)
        p._rebuild_props()
        before = p._canvas._bead_width_mm
        p._match_extrusion_to(2.0)
        self.assertGreater(p._canvas._bead_width_mm, before)


# ═══════════════════════════════════════════════════════════════════
# 2 — join a line to an object's print start / stop
# ═══════════════════════════════════════════════════════════════════

class TestPrintPointSnapTargets(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_a_lines_print_start_and_stop_are_targets_with_anchors(self):
        c = _canvas([_line(0, 0, 5, 0)])
        got = {(k, a): (round(x, 6), round(y, 6))
               for x, y, _i, k, a in c._print_point_targets()}
        self.assertEqual(got[("printstart", "p0")], (0.0, 0.0))
        self.assertEqual(got[("printend", "p1")], (5.0, 0.0))

    def test_a_derived_print_point_carries_no_anchor(self):
        """A circle's seam is not a solver DOF — it must report anchor None so
        no unresolvable constraint ref is ever written."""
        c = _canvas([SketchShape(kind="circle", cx=0, cy=0, radius=4)])
        tg = [t for t in c._print_point_targets() if t[3] == "printstart"]
        self.assertEqual(len(tg), 1)
        self.assertAlmostEqual(tg[0][0], 4.0, places=6)      # seam at +X
        self.assertIsNone(tg[0][4])

    def test_a_custom_start_point_moves_the_target(self):
        """It tracks the PRINT start, not merely a vertex: pinning the start to
        the far endpoint swaps which end is the entry and which the exit."""
        sh = _line(0, 0, 5, 0)
        sh.start_point = (5.0, 0.0)
        c = _canvas([sh])
        got = {k: (round(x, 6), round(y, 6))
               for x, y, _i, k, _a in c._print_point_targets()}
        self.assertEqual(got["printstart"], (5.0, 0.0))
        self.assertEqual(got["printend"], (0.0, 0.0))

    def test_fills_regions_and_travel_markers_are_not_offered(self):
        c = _canvas([SketchShape(kind="rect", cx=0, cy=0, width=5, height=5,
                                 filled=True),
                     SketchShape(kind="region", points=[(0, 0), (1, 1)]),
                     SketchShape(kind="travel", cx=9, cy=9)])
        self.assertEqual(c._print_point_targets(), [])

    def test_snap_prefers_the_print_point_over_a_coincident_vertex(self):
        c = _canvas([_line(0, 0, 5, 0)])
        c._snap(QPointF(5.001, 0.001))
        self.assertEqual(c._snap_hit, ("printend", 0, "p1"))

    def test_the_join_is_still_captured_as_a_coincident_constraint(self):
        c = _canvas([_line(0, 0, 5, 0)])
        c._auto_constrain = True
        _draw_line(c, (5, 0), (5, 5), press_hit=("printend", 0, "p1"))
        kinds = [k.kind for k in c.sketch().constraints]
        self.assertIn("coincident", kinds)

    def test_a_derived_print_point_captures_nothing(self):
        c = _canvas([SketchShape(kind="circle", cx=0, cy=0, radius=4)])
        c._auto_constrain = True
        _draw_line(c, (4, 0), (9, 0), press_hit=("printstart", 0, None))
        self.assertEqual(c.sketch().constraints, [])


class TestJoinReordersSoItPrintsAsOneBead(unittest.TestCase):
    """A coinciding endpoint only welds when the two shapes are ADJACENT in
    print order — these pin that the reorder happens and the weld follows."""

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    @staticmethod
    def _sections(sk):
        return [p for p in plan_print_sections(sk, None)
                if p["type"] == "section"]

    def _two(self):
        """The join target is shape 0; shape 1 is an unrelated far line sitting
        AFTER it, so a plain append could not weld."""
        return _canvas([_line(0, 0, 10, 0), _line(40, 40, 50, 40)])

    def test_press_on_a_stop_inserts_right_after_that_shape(self):
        c = self._two()
        _draw_line(c, (10, 0), (10, 10), press_hit=("printend", 0, "p1"))
        self.assertEqual(tuple(c.sketch().shapes[1].points),
                         ((10.0, 0.0), (10.0, 10.0)))
        self.assertEqual(len(self._sections(c.sketch())), 2)  # joined + far

    def test_release_on_a_start_inserts_right_before_that_shape(self):
        c = _canvas([_line(40, 40, 50, 40), _line(0, 0, 10, 0)])
        _draw_line(c, (-10, 0), (0, 0), release_hit=("printstart", 1, "p0"))
        self.assertEqual(tuple(c.sketch().shapes[1].points),
                         ((-10.0, 0.0), (0.0, 0.0)))
        self.assertEqual(len(self._sections(c.sketch())), 2)

    def test_release_on_a_stop_welds_via_the_compilers_own_flip(self):
        c = self._two()
        _draw_line(c, (10, 10), (10, 0), release_hit=("printend", 0, "p1"))
        self.assertEqual(len(self._sections(c.sketch())), 2)

    def test_press_on_a_start_pins_the_start_so_the_line_ENDS_there(self):
        """The one case the compiler cannot fix itself: our p0 is at the
        target's entry, so the line must run p1->p0 and print FIRST."""
        c = _canvas([_line(40, 40, 50, 40), _line(0, 0, 10, 0)])
        _draw_line(c, (0, 0), (-10, 0), press_hit=("printstart", 1, "p0"))
        joined = c.sketch().shapes[1]
        self.assertEqual(tuple(joined.points), ((0.0, 0.0), (-10.0, 0.0)))
        self.assertEqual(joined.start_point, (-10.0, 0.0))
        self.assertEqual(len(self._sections(c.sketch())), 2)

    def test_without_the_reorder_it_would_NOT_have_welded(self):
        """Guard the guard — the same geometry appended at the END stays three
        sections, so the tests above really measure the reorder."""
        c = self._two()
        c.sketch().shapes.append(_line(10, 0, 10, 10))
        self.assertEqual(len(self._sections(c.sketch())), 3)

    def test_a_plain_vertex_snap_reorders_nothing(self):
        c = self._two()
        _draw_line(c, (40, 40), (30, 20), press_hit=("vertex", 1, "p0"))
        self.assertEqual(len(c.sketch().shapes), 3)
        self.assertEqual(tuple(c.sketch().shapes[2].points),
                         ((40.0, 40.0), (30.0, 20.0)))   # still appended last

    def test_the_join_is_announced_and_names_what_it_joined(self):
        c = _canvas([SketchShape(kind="circle", cx=0, cy=0, radius=4,
                                 overlap_mode="distance",
                                 overlap_distance_mm=0.5),
                     _line(40, 40, 50, 40)])
        seen = []
        c.join_result.connect(seen.append)
        end = c._effective_end_world(c.sketch().shapes[0])
        _draw_line(c, (end.x(), end.y()), (20, 20),
                   press_hit=("printend", 0, None))
        self.assertEqual(len(seen), 1)
        self.assertIn("stop", seen[0])
        self.assertIn("circle", seen[0])        # the shape actually joined to

    def test_the_announcement_names_the_right_shape_when_joining_a_start(self):
        """A distinct target kind on the BEFORE side, so an off-by-one in which
        neighbour is reported cannot pass by naming the wrong shape."""
        c = _canvas([_line(40, 40, 50, 40),
                     SketchShape(kind="circle", cx=0, cy=0, radius=4)])
        seen = []
        c.join_result.connect(seen.append)
        _draw_line(c, (-10, 0), (4, 0), release_hit=("printstart", 1, None))
        self.assertEqual(len(seen), 1)
        self.assertIn("start", seen[0])
        self.assertIn("circle", seen[0])

    def test_snapping_then_drawing_joins_end_to_end(self):
        """The two halves wired together: ask the real ``_snap`` for the hit at
        an object's print stop, hand it to the real commit path, and the two
        must come out as one section."""
        c = self._two()
        c._snap(QPointF(10.002, 0.002))         # near shape 0's print stop
        hit = c._snap_hit
        self.assertEqual(hit[:2], ("printend", 0))
        _draw_line(c, (10, 0), (10, 10), press_hit=hit)
        self.assertEqual(len(self._sections(c.sketch())), 2)

    def test_a_join_to_a_circles_seam_welds_too(self):
        c = _canvas([SketchShape(kind="circle", cx=0, cy=0, radius=4),
                     _line(40, 40, 50, 40)])
        _draw_line(c, (-10, 0), (4, 0), release_hit=("printstart", 0, None))
        self.assertEqual(tuple(c.sketch().shapes[0].points),
                         ((-10.0, 0.0), (4.0, 0.0)))
        self.assertEqual(len(self._sections(c.sketch())), 2)

    def test_undo_puts_the_sketch_back(self):
        c = self._two()
        _draw_line(c, (10, 0), (10, 10), press_hit=("printend", 0, "p1"))
        self.assertEqual(len(c.sketch().shapes), 3)
        c.undo()
        self.assertEqual(len(c.sketch().shapes), 2)


# ═══════════════════════════════════════════════════════════════════
# 3 — no extrude (move only) on any shape / selection / section
# ═══════════════════════════════════════════════════════════════════

class TestNoExtrudeModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_set_no_print_marks_clears_and_reports(self):
        c = _canvas([_line(0, 0, 5, 0), _line(5, 0, 5, 5)])
        self.assertEqual(c.set_no_print([0, 1], True), 2)
        self.assertTrue(all(s.no_print for s in c.sketch().shapes))
        self.assertEqual(c.set_no_print([0, 1], True), 0)      # idempotent
        self.assertEqual(c.set_no_print([0], False), 1)

    def test_travel_markers_are_never_touched(self):
        c = _canvas([SketchShape(kind="travel", cx=1, cy=1)])
        self.assertEqual(c.set_no_print([0], True), 0)
        self.assertFalse(c.sketch().shapes[0].no_print)

    def test_it_is_undoable(self):
        c = _canvas([_line(0, 0, 5, 0)])
        c.set_no_print([0], True)
        c.undo()
        self.assertFalse(c.sketch().shapes[0].no_print)

    def test_a_no_extrude_shape_deposits_nothing_but_is_still_traversed(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0),
                            _line(10, 0, 20, 0, no_print=True)],
                    line_spacing_mm=0.4)
        traj = compile_to_trajectory(sk).trajectory
        # The needle reaches x=20 …
        self.assertAlmostEqual(float(np.max(traj[:, 0])), 20.0, places=3)
        # … and the pump column is flat over the move-only leg.
        first = traj[traj[:, 0] <= 10.0 + 1e-9]
        second = traj[traj[:, 0] >= 10.0 - 1e-9]
        self.assertGreater(float(first[-1, 3] - first[0, 3]), 0.0)
        self.assertAlmostEqual(
            float(second[:, 3].max() - second[:, 3].min()), 0.0, places=9)


class TestNoExtrudeOnThePage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, sk):
        from gui.pages.print_builder_sketch import SketchPage
        p = SketchPage()
        p._canvas.set_sketch(sk)
        return p

    @staticmethod
    def _boxes(root):
        return [w for w in root.findChildren(QCheckBox)
                if "No extrude" in w.text()]

    def test_the_shape_card_offers_it(self):
        p = self._page(Sketch(shapes=[_line(0, 0, 10, 0)]))
        p._canvas.set_selected(0)
        p._rebuild_props()
        boxes = self._boxes(p)
        self.assertEqual(len(boxes), 1)
        self.assertFalse(boxes[0].isChecked())
        boxes[0].setChecked(True)
        self.assertTrue(p._canvas.sketch().shapes[0].no_print)

    def test_a_travel_marker_does_not_offer_it(self):
        p = self._page(Sketch(shapes=[SketchShape(kind="travel", cx=1, cy=1)]))
        p._canvas.set_selected(0)
        p._rebuild_props()
        self.assertEqual(self._boxes(p), [])

    def test_a_multi_selection_applies_it_to_all(self):
        p = self._page(Sketch(shapes=[_line(0, 0, 5, 0), _line(5, 0, 5, 5),
                                      _line(5, 5, 0, 5)]))
        p._canvas.select_indices([0, 1, 2])
        p._rebuild_props()
        boxes = self._boxes(p)
        self.assertEqual(len(boxes), 1)
        boxes[0].setChecked(True)
        self.assertTrue(all(s.no_print for s in p._canvas.sketch().shapes))

    def test_a_whole_section_can_be_switched_off_and_back_on(self):
        sk = Sketch(shapes=[_line(0, 0, 5, 0), _line(5, 0, 5, 5),
                            SketchShape(kind="travel", cx=30, cy=30),
                            _line(30, 30, 40, 30)],
                    line_spacing_mm=0.4)
        p = self._page(sk)
        p._refresh_sequence()
        toggles = [b for b in p._seq_host.findChildren(QToolButton)
                   if b.isCheckable()]
        self.assertEqual(len(toggles), 2)                # two sections
        self.assertEqual(toggles[0].text(), "extruding")
        toggles[0].click()
        # Section 1 = the two welded lines; the far line is untouched.
        self.assertTrue(sk.shapes[0].no_print)
        self.assertTrue(sk.shapes[1].no_print)
        self.assertFalse(sk.shapes[3].no_print)
        p._refresh_sequence()
        toggles = [b for b in p._seq_host.findChildren(QToolButton)
                   if b.isCheckable()]
        self.assertTrue(toggles[0].isChecked())
        self.assertEqual(toggles[0].text(), "no extrude")
        toggles[0].click()
        self.assertFalse(sk.shapes[0].no_print)

    def test_the_section_body_names_the_move_only_shapes(self):
        sk = Sketch(shapes=[_line(0, 0, 5, 0, no_print=True)],
                    line_spacing_mm=0.4)
        p = self._page(sk)
        p._refresh_sequence()
        rows = [w.text() for w in p._seq_host.findChildren(QLabel)
                if w.text().startswith("▪")]
        self.assertEqual(len(rows), 1)
        self.assertIn("no extrude", rows[0])


if __name__ == "__main__":
    unittest.main()
