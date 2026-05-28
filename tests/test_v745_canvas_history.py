"""Tests for v7.4.5 canvas undo/redo snapshot stack."""

from __future__ import annotations

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.plate_designer_canvas import PlateDesignerCanvas, Tool
from SupportClasses.PlateDesign import PlateDesign, Well


class TestUndoRedo(unittest.TestCase):
    def _canvas(self) -> PlateDesignerCanvas:
        canvas = PlateDesignerCanvas()
        design = PlateDesign.from_standard_format(6)
        canvas.set_design(design)
        return canvas

    def test_initial_state_no_history(self):
        c = self._canvas()
        self.assertFalse(c.can_undo())
        self.assertFalse(c.can_redo())

    def test_draw_well_pushes_snapshot(self):
        c = self._canvas()
        baseline = len(c.design.get_wells())
        c.set_tool(Tool.DRAW_SINGLE_WELL)
        c._handle_draw_well((30.0, 40.0))
        self.assertEqual(len(c.design.get_wells()), baseline + 1)
        self.assertTrue(c.can_undo())
        self.assertFalse(c.can_redo())

    def test_undo_restores_pre_mutation_state(self):
        c = self._canvas()
        baseline = len(c.design.get_wells())
        c.set_tool(Tool.DRAW_SINGLE_WELL)
        c._handle_draw_well((30.0, 40.0))
        c.undo()
        self.assertEqual(len(c.design.get_wells()), baseline)
        self.assertTrue(c.can_redo())

    def test_redo_replays(self):
        c = self._canvas()
        baseline = len(c.design.get_wells())
        c.set_tool(Tool.DRAW_SINGLE_WELL)
        c._handle_draw_well((30.0, 40.0))
        c.undo()
        c.redo()
        self.assertEqual(len(c.design.get_wells()), baseline + 1)

    def test_new_mutation_clears_redo(self):
        c = self._canvas()
        c.set_tool(Tool.DRAW_SINGLE_WELL)
        c._handle_draw_well((30.0, 40.0))
        c.undo()
        self.assertTrue(c.can_redo())
        # New mutation should wipe redo.
        c._handle_draw_well((50.0, 50.0))
        self.assertFalse(c.can_redo())

    def test_circle_pattern_pushes_snapshot(self):
        c = self._canvas()
        baseline = len(c.design.get_wells())
        c.set_tool(Tool.DRAW_CIRCLE_PATTERN)
        c._handle_draw_circle_pattern((50.0, 50.0))
        self.assertGreater(len(c.design.get_wells()), baseline)
        self.assertTrue(c.can_undo())
        c.undo()
        self.assertEqual(len(c.design.get_wells()), baseline)

    def test_capacity_eviction(self):
        c = self._canvas()
        c._undo_cap = 3
        c.set_tool(Tool.DRAW_SINGLE_WELL)
        for i in range(5):
            c._handle_draw_well((10.0 + i, 10.0))
        # Stack should hold no more than 3 snapshots.
        self.assertEqual(len(c._undo_stack), 3)


class TestPlateLayoutOffset(unittest.TestCase):
    def test_outline_extends_back_by_a1_offset(self):
        """Plate outline rect places A1 inset from top-left corner."""
        canvas = PlateDesignerCanvas()
        design = PlateDesign.from_standard_format(96)
        canvas.set_design(design)
        outline = canvas._outline_item
        self.assertIsNotNone(outline)
        rect = outline.rect()
        # Outline starts at (-a1_offset_x * SCALE, -a1_offset_y * SCALE).
        from gui.widgets.plate_designer_canvas import SCALE_FACTOR
        self.assertAlmostEqual(rect.x(), -14.38 * SCALE_FACTOR, places=2)
        self.assertAlmostEqual(rect.y(), -11.24 * SCALE_FACTOR, places=2)
        # Width / height match the ANSI footprint.
        self.assertAlmostEqual(rect.width(), 127.76 * SCALE_FACTOR, places=2)
        self.assertAlmostEqual(rect.height(), 85.48 * SCALE_FACTOR, places=2)


if __name__ == "__main__":
    unittest.main()
