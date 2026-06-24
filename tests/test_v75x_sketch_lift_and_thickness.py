"""test_v75x_sketch_lift_and_thickness.py — Sketch lift + thickness preview.

Covers two v7.5.x Sketch-page additions:
- **Lift between shapes**: the `Sketch.travel_clearance_mm` knob (now exposed in
  the print card) raises the inter-shape/inter-pass travel Z; larger values lift
  the needle higher over already-printed material.
- **Show line thickness**: an optional canvas toggle that strokes print runs at
  the deposited bead width (`line_spacing_mm`, ≈ the needle Ø) so the operator
  sees how thick the printed lines will be.
"""

import sys
import unittest

import numpy as np
from PySide6.QtWidgets import QApplication

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory,
)


def _circle(cx=0.0, cy=0.0, r=2.0) -> SketchShape:
    return SketchShape(kind="circle", cx=cx, cy=cy, radius=r,
                       pump_index=0, color="#89b4fa", line_width_mm=0.4)


class TestLiftBetweenShapes(unittest.TestCase):
    def _zmax(self, clearance: float) -> float:
        sk = Sketch()
        sk.shapes = [_circle()]
        sk.travel_clearance_mm = clearance
        res = compile_to_trajectory(sk)
        return float(res.trajectory[:, 2].max())

    def test_clearance_raises_travel_z(self):
        # z_travel = z_start + num_layers*layer_height + clearance.
        z2 = self._zmax(2.0)
        z6 = self._zmax(6.0)
        self.assertAlmostEqual(z6 - z2, 4.0, places=6)

    def test_default_travel_z(self):
        # Defaults: z_start 0.2 + 1*0.2 + clearance 2.0 = 2.4.
        self.assertAlmostEqual(self._zmax(2.0), 2.4, places=6)

    def test_zero_clearance_is_top_layer(self):
        self.assertAlmostEqual(self._zmax(0.0), 0.4, places=6)


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class TestCanvasThickness(_Base):
    def test_toggle_state_and_render(self):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        self.assertFalse(c._show_thickness)
        # Feed a compiled toolpath so there is something to stroke.
        sk = Sketch()
        sk.shapes = [_circle(r=3.0)]
        sk.line_spacing_mm = 0.7
        res = compile_to_trajectory(sk)
        c.set_toolpath(res.trajectory, res.pump_states)
        c.resize(300, 300)
        c.set_show_thickness(True)
        self.assertTrue(c._show_thickness)
        c.grab()  # render at bead width — must not raise
        c.set_show_thickness(False)
        self.assertFalse(c._show_thickness)
        c.grab()


class TestPageWiring(_Base):
    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_thickness_toggle_drives_canvas(self):
        page = self._page()
        self.assertFalse(page._canvas._show_thickness)
        page._toggle_thickness(True)
        self.assertTrue(page._canvas._show_thickness)
        page._toggle_thickness(False)
        self.assertFalse(page._canvas._show_thickness)

    def test_lift_setter_writes_sketch(self):
        page = self._page()
        page._set_sketch("travel_clearance_mm", 7.5)
        self.assertAlmostEqual(page._canvas.sketch().travel_clearance_mm, 7.5,
                               places=6)


if __name__ == "__main__":
    unittest.main()
