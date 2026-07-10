"""test_v75x_sketch_well_boundary.py — Sketch page single-well boundary.

Covers the v7.5.x feature: the Print Builder → Sketch page defaults to a
zoomed-in single-well view, lets the user pick which well the boundary is
derived from, and draws a needle-safe inner boundary inset by the needle
radius (so the needle never contacts the well wall). Sketches that extend past
the safe boundary raise a non-blocking warning.

Geometry contract (verified):
- Sketch is authored in well-relative mm with (0,0) = well center.
- Well-wall Ø = well diameter; needle-safe Ø = well Ø − needle Ø
  (inset by the needle radius on each side); safe radius = needle-safe Ø / 2.
"""

import sys
import unittest

import numpy as np
from PySide6.QtWidgets import QApplication

from SupportClasses.PhysicalModels import NeedleSpec
from SupportClasses.SketchTrajectory import SketchShape


def _needle() -> NeedleSpec:
    # od_mm = 0.718
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


class _Cfg:
    """Minimal stand-in for HardwareConfig as seen by SketchPage."""

    def __init__(self, needle=None, plate_key=96):
        self.needle = needle
        self.pumps = []
        self._key = plate_key

    @property
    def active_plate_key(self):
        return self._key


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()


class TestCanvasBoundary(_Base):
    def test_set_safe_boundary_stores_and_renders(self):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        c.set_reference_well(6.35)
        c.set_safe_boundary(5.632)
        self.assertAlmostEqual(c._ref_well_d, 6.35, places=3)
        self.assertAlmostEqual(c._safe_well_d, 5.632, places=3)
        c.resize(300, 300)
        c.grab()  # triggers paintEvent — must not raise

    def test_safe_boundary_off_when_zero(self):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        c.set_safe_boundary(0.0)
        self.assertEqual(c._safe_well_d, 0.0)


class TestWellSelector(_Base):
    def test_populates_and_defaults_to_a1(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))
        self.assertEqual(page._well_combo.count(), 96)
        self.assertEqual(page._selected_well, "A1")

    def test_boundary_derived_from_selected_well(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))
        # 96-well Ø = 6.35; needle Ø = 0.718 → safe Ø = 5.632.
        self.assertAlmostEqual(page._canvas._ref_well_d, 6.35, places=2)
        self.assertAlmostEqual(page._canvas._safe_well_d, 6.35 - 0.718, places=3)
        self.assertAlmostEqual(page._safe_radius_mm, (6.35 - 0.718) / 2.0,
                               places=3)

    def test_changing_well_reapplies_boundary(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))
        idx = page._well_combo.findText("H12")
        self.assertGreaterEqual(idx, 0)
        page._well_combo.setCurrentIndex(idx)
        self.assertEqual(page._selected_well, "H12")
        # Standard plate → uniform diameter, boundary still set.
        self.assertAlmostEqual(page._canvas._ref_well_d, 6.35, places=2)

    def test_no_needle_hides_safe_boundary(self):
        page = self._page()
        page.set_hardware_config(_Cfg(needle=None, plate_key=96))
        self.assertAlmostEqual(page._canvas._ref_well_d, 6.35, places=2)
        self.assertEqual(page._canvas._safe_well_d, 0.0)
        self.assertEqual(page._safe_radius_mm, 0.0)

    def test_no_plate_clears_boundary(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key="does-not-exist"))
        self.assertEqual(page._well_combo.count(), 0)
        self.assertEqual(page._canvas._ref_well_d, 0.0)


class TestSafeBoundaryCheck(_Base):
    def _traj(self, r: float):
        # Two waypoints at radius r from origin (well center).
        return np.array([[r, 0, 0, 0, 0, 0, 0.0],
                         [0, r, 0, 0, 0, 0, 1.0]], dtype=np.float64)

    def test_max_radius(self):
        page = self._page()
        self.assertAlmostEqual(page._max_radius_mm(self._traj(5.0)), 5.0,
                               places=6)

    def test_exceeds_when_outside(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))  # safe r≈2.816
        self.assertTrue(page._exceeds_safe_boundary(self._traj(5.0)))

    def test_within_when_inside(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))
        self.assertFalse(page._exceeds_safe_boundary(self._traj(1.0)))

    def test_no_safe_radius_never_exceeds(self):
        page = self._page()
        page.set_hardware_config(_Cfg(needle=None, plate_key=96))
        self.assertFalse(page._exceeds_safe_boundary(self._traj(50.0)))


class TestPreviewWarning(_Base):
    def test_oob_circle_flags_warning(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))  # safe r≈2.816
        page._canvas.sketch().shapes.append(
            SketchShape(kind="circle", cx=0.0, cy=0.0, radius=5.0,
                        ink_id=1, color="#89b4fa", line_width_mm=0.4))
        page._recompute_preview()
        self.assertTrue(page._last_oob)
        # isHidden() reflects the explicit setVisible() state regardless of
        # whether the (offscreen) page's top-level window is shown.
        self.assertFalse(page._bounds_warn_lbl.isHidden())

    def test_in_bounds_circle_no_warning(self):
        page = self._page()
        page.set_hardware_config(_Cfg(_needle(), plate_key=96))
        page._canvas.sketch().shapes.append(
            SketchShape(kind="circle", cx=0.0, cy=0.0, radius=1.0,
                        ink_id=1, color="#89b4fa", line_width_mm=0.4))
        page._recompute_preview()
        self.assertFalse(page._last_oob)
        self.assertTrue(page._bounds_warn_lbl.isHidden())


if __name__ == "__main__":
    unittest.main()
