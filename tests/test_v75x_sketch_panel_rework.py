"""test_v75x_sketch_panel_rework.py — Print Builder Sketch right-panel rework.

The right context panel now scrolls as ONE unit (Save/Send pinned below), the
print-sequence card has NO nested fixed-height scroll, and each print section is
a collapsible, content-sized block whose body lists the section's operations and
grows to fit. Per-section collapse state survives a sequence rebuild.

Runs offscreen (no hardware).
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import sys
import unittest

from PySide6.QtWidgets import (
    QApplication, QScrollArea, QToolButton, QLabel, QPushButton,
)

from SupportClasses.SketchTrajectory import Sketch, SketchShape, SketchInk


def _line(x1, y1, x2, y2, ink_id=1):
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)], ink_id=ink_id)


def _two_ink_single_sketch():
    """Three connected lines on two inks → 3 sections in single-needle mode."""
    sk = Sketch(single_needle=True, line_spacing_mm=0.4)
    sk.inks = [SketchInk(1, "A", "#89b4fa"), SketchInk(2, "B", "#a6e3a1")]
    sk._next_ink_id = 3
    sk.shapes = [_line(0, 0, 10, 0, 1), _line(10, 0, 10, 10, 2),
                 _line(10, 10, 0, 10, 1)]
    return sk


class TestPanelRework(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_no_nested_fixed_height_scroll(self):
        p = self._page()
        # The old fixed-height sequence scroll is gone.
        self.assertFalse(hasattr(p, "_seq_scroll"))
        # The sequence host grows freely (no maximumHeight cap).
        self.assertGreater(p._seq_host.maximumHeight(), 5000)

    def test_panel_has_one_outer_scroll_and_pinned_buttons(self):
        p = self._page()
        panel = p._build_right_panel()
        scrolls = panel.findChildren(QScrollArea)
        self.assertEqual(len(scrolls), 1)         # exactly one outer scroll
        # Save/Send are pinned OUTSIDE the scroll's content.
        content = scrolls[0].widget()
        self.assertIsNotNone(p._send_btn)
        self.assertFalse(_is_descendant(p._send_btn, content))

    def test_sections_are_collapsible_with_operation_rows(self):
        p = self._page()
        p._canvas.set_sketch(_two_ink_single_sketch())
        p._refresh_sequence()
        # 3 sections → 3 clickable title buttons + 3 chevrons.
        self.assertEqual(len(p._seq_host.findChildren(QPushButton)), 3)
        self.assertEqual(len(p._seq_host.findChildren(QToolButton)), 3)
        # Each single-shape section lists exactly one operation row.
        op_rows = [l for l in p._seq_host.findChildren(QLabel)
                   if l.text().startswith("▪")]
        self.assertEqual(len(op_rows), 3)

    def test_collapse_state_survives_refresh(self):
        p = self._page()
        p._canvas.set_sketch(_two_ink_single_sketch())
        p._refresh_sequence()
        chev = p._seq_host.findChildren(QToolButton)[0]
        chev.click()                              # collapse section 1
        self.assertTrue(p._seq_collapsed.get(1))
        p._refresh_sequence()                     # rebuild
        chev2 = p._seq_host.findChildren(QToolButton)[0]
        self.assertEqual(chev2.text(), "▸")       # still collapsed after rebuild


def _is_descendant(widget, ancestor):
    w = widget.parentWidget()
    while w is not None:
        if w is ancestor:
            return True
        w = w.parentWidget()
    return False


if __name__ == "__main__":
    unittest.main()
