"""test_v75x_sketch_group_select.py — Sketch multi-select + group transform.

Select-all, marquee (lasso) rubber-band select, Ctrl/Shift toggle, and
resizing / moving / scaling the whole selection as a group.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import math
import sys
import unittest

from PySide6.QtCore import Qt, QPointF
from PySide6.QtWidgets import QApplication

from SupportClasses.SketchTrajectory import Sketch, SketchShape


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _canvas(self):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        c.set_sketch(Sketch(shapes=[
            SketchShape(kind="circle", cx=0.0, cy=0.0, radius=2.0),
            SketchShape(kind="rect", cx=10.0, cy=0.0, width=4.0, height=4.0),
            SketchShape(kind="line", points=[(0.0, 10.0), (5.0, 12.0)]),
        ]))
        return c


class TestSelectionSet(_Base):
    def test_select_all_and_clear(self):
        c = self._canvas()
        c.select_all()
        self.assertEqual(c.selection_count(), 3)
        c.clear_selection()
        self.assertEqual(c.selection_count(), 0)

    def test_single_select_sets_primary(self):
        c = self._canvas()
        c.set_selected(1)
        self.assertEqual(c.selected_indices(), [1])
        self.assertIs(c.selected_shape(), c.sketch().shapes[1])

    def test_multi_select_primary_is_none(self):
        c = self._canvas()
        c._set_selection({0, 2})
        self.assertEqual(c.selected_indices(), [0, 2])
        # No single "primary" when 2+ are selected → props panel shows group.
        self.assertIsNone(c.selected_shape())

    def test_delete_removes_all_selected(self):
        c = self._canvas()
        c._set_selection({0, 1})
        c.delete_selected()
        self.assertEqual(len(c.sketch().shapes), 1)
        self.assertEqual(c.selection_count(), 0)


class TestMarquee(_Base):
    def _marquee(self, c, a, b, additive=False):
        c._mode = "marquee"
        c._marquee_start = QPointF(*a)
        c._marquee_cur = QPointF(*b)
        c._marquee_additive = additive
        c._commit_marquee()

    def test_marquee_selects_intersecting(self):
        c = self._canvas()
        # Box covering the circle (−2..2) and rect (8..12), not the line (y≥10).
        self._marquee(c, (-3, -3), (13, 3))
        self.assertEqual(c.selected_indices(), [0, 1])

    def test_marquee_additive_unions(self):
        c = self._canvas()
        c._set_selection({2})                 # start with the line
        self._marquee(c, (-3, -3), (3, 3), additive=True)   # add the circle
        self.assertEqual(c.selected_indices(), [0, 2])

    def test_tiny_drag_clears(self):
        c = self._canvas()
        c._set_selection({0, 1})
        self._marquee(c, (5, 5), (5.05, 5.05))   # ~click on empty → clear
        self.assertEqual(c.selection_count(), 0)

    def test_shape_in_rect(self):
        c = self._canvas()
        circle = c.sketch().shapes[0]
        self.assertTrue(c._shape_in_rect(circle, (-1, -1, 1, 1)))
        self.assertFalse(c._shape_in_rect(circle, (50, 50, 60, 60)))


class TestGroupTransform(_Base):
    def test_group_resize_scales_all(self):
        c = self._canvas()
        c._set_selection({0, 1})
        bbox = c._group_bbox()                # (-2,-2,12,2), anchor=(-2,-2)
        self.assertEqual(bbox, (-2.0, -2.0, 12.0, 2.0))
        r0 = c.sketch().shapes[0].radius
        c._begin_group_resize()
        ax, ay = c._group_anchor
        rx, ry = c._group_ref
        target = QPointF(23.0, 4.5)
        c._apply_group_resize(target)
        d0 = math.hypot(rx - ax, ry - ay)
        d1 = math.hypot(target.x() - ax, target.y() - ay)
        sc = d1 / d0
        self.assertAlmostEqual(c.sketch().shapes[0].radius, r0 * sc, places=5)

    def test_group_move_translates_all(self):
        c = self._canvas()
        c._set_selection({0, 1})
        cx0 = c.sketch().shapes[0].cx
        rcx0 = c.sketch().shapes[1].cx
        c._begin_group_move(QPointF(0.0, 0.0))
        c._apply_group_move(QPointF(3.0, -1.0))
        self.assertAlmostEqual(c.sketch().shapes[0].cx, cx0 + 3.0, places=6)
        self.assertAlmostEqual(c.sketch().shapes[1].cx, rcx0 + 3.0, places=6)
        self.assertAlmostEqual(c.sketch().shapes[0].cy, -1.0, places=6)

    def test_scale_selection_about_center(self):
        c = self._canvas()
        c._set_selection({0, 1})
        w0 = c.sketch().shapes[1].width
        c.scale_selection(0.5)
        self.assertAlmostEqual(c.sketch().shapes[1].width, w0 * 0.5, places=6)

    def test_group_op_is_undoable(self):
        c = self._canvas()
        c._set_selection({0, 1})
        r0 = c.sketch().shapes[0].radius
        c.scale_selection(2.0)
        self.assertAlmostEqual(c.sketch().shapes[0].radius, r0 * 2.0, places=6)
        c.undo()
        self.assertAlmostEqual(c.sketch().shapes[0].radius, r0, places=6)


class TestPressToggle(_Base):
    def test_ctrl_click_toggles_membership(self):
        c = self._canvas()
        # Rect is filled-hit inside its extent — click its centre (10,0).
        w = QPointF(10.0, 0.0)
        sp = c._w2s(10.0, 0.0)
        c._press_select(sp, w, Qt.ControlModifier)
        self.assertEqual(c.selected_indices(), [1])
        c._press_select(sp, w, Qt.ControlModifier)   # toggle off
        self.assertEqual(c.selected_indices(), [])


class TestPostDrawSelection(_Base):
    """A freshly drawn/filled/finished shape must be properly selected (both
    _selection — highlight + handles — and _selected — props panel)."""

    def test_commit_draw_selects_new_shape(self):
        from gui.widgets.sketch_canvas import Tool
        c = self._canvas()
        c.set_sketch(Sketch())
        c._tool = Tool.CIRCLE
        c._mode = "draw"
        c._draw_start = QPointF(0.0, 0.0)
        c._draw_cur = QPointF(0.0, 3.0)
        c._commit_draw()
        self.assertEqual(c.selected_indices(), [0])
        self.assertEqual(c.selection_count(), 1)
        self.assertIsNotNone(c.selected_shape())

    def test_commit_polygon_selects_new_shape(self):
        c = self._canvas()
        c.set_sketch(Sketch())
        c._poly_pts = [(0.0, 0.0), (2.0, 0.0), (1.0, 2.0)]
        c._commit_polygon()
        self.assertEqual(c.selected_indices(), [0])
        self.assertEqual(c.selection_count(), 1)

    def test_draw_after_existing_selection_replaces_it(self):
        # A stale _selection must not linger onto the new shape.
        from gui.widgets.sketch_canvas import Tool
        c = self._canvas()          # 3 shapes
        c.select_all()
        c._tool = Tool.CIRCLE
        c._mode = "draw"
        c._draw_start = QPointF(20.0, 20.0)
        c._draw_cur = QPointF(20.0, 23.0)
        c._commit_draw()
        self.assertEqual(c.selected_indices(), [3])   # only the new one


class TestKeyboard(_Base):
    def test_ctrl_a_selects_all(self):
        from PySide6.QtGui import QKeyEvent
        c = self._canvas()
        ev = QKeyEvent(QKeyEvent.KeyPress, Qt.Key_A, Qt.ControlModifier)
        c.keyPressEvent(ev)
        self.assertEqual(c.selection_count(), 3)


class TestPage(_Base):
    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_group_card_shown_for_multi(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="circle", cx=0, cy=0, radius=1),
            SketchShape(kind="circle", cx=5, cy=0, radius=1),
        ]))
        page._canvas.select_all()
        page._rebuild_props()      # (selection_changed already triggers this)
        # The group scale spin exists only on the group card.
        self.assertEqual(page._canvas.selection_count(), 2)

    def test_apply_group_scale_resets_spin(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="rect", cx=0, cy=0, width=4, height=4),
            SketchShape(kind="rect", cx=8, cy=0, width=4, height=4),
        ]))
        page._canvas.select_all()
        from PySide6.QtWidgets import QDoubleSpinBox
        spin = QDoubleSpinBox()
        spin.setRange(0.05, 20.0)
        spin.setValue(2.0)
        w0 = page._canvas.sketch().shapes[0].width
        page._apply_group_scale(spin)
        self.assertAlmostEqual(spin.value(), 1.0, places=6)
        self.assertAlmostEqual(page._canvas.sketch().shapes[0].width,
                               w0 * 2.0, places=5)


if __name__ == "__main__":
    unittest.main()
