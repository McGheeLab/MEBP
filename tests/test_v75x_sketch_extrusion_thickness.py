"""test_v75x_sketch_extrusion_thickness.py — Sketch extrusion multiplier +
shaded print-thickness band.

The Print Builder Sketch page gets a print-level **extrusion multiplier**:
1× = a bead the width of the needle inner Ø; it scales both the deposited
volume of the compiled/baked trajectory AND the shaded thickness band drawn
around each print run on the canvas.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import sys
import unittest

import numpy as np
from PySide6.QtWidgets import QApplication

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory,
)
from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec


def _line_sketch(mult: float = 1.0) -> Sketch:
    sk = Sketch(shapes=[SketchShape(
        kind="line", points=[(0.0, 0.0), (10.0, 0.0)], line_width_mm=0.4)])
    sk.line_spacing_mm = 0.4
    sk.layer_height_mm = 0.2
    sk.extrusion_multiplier = mult
    return sk


# ═══════════════════════════════════════════════════════════════════
# Model / compiler
# ═══════════════════════════════════════════════════════════════════

class TestModel(unittest.TestCase):
    def test_default_is_one(self):
        self.assertEqual(Sketch().extrusion_multiplier, 1.0)

    def test_roundtrip(self):
        sk = _line_sketch(2.5)
        d = sk.to_dict()
        self.assertEqual(d["extrusion_multiplier"], 2.5)
        self.assertEqual(Sketch.from_dict(d).extrusion_multiplier, 2.5)

    def test_from_dict_defaults_when_missing(self):
        # Legacy sketch dicts (no key) default to 1.0.
        d = _line_sketch().to_dict()
        d.pop("extrusion_multiplier")
        self.assertEqual(Sketch.from_dict(d).extrusion_multiplier, 1.0)


class TestVolumeScaling(unittest.TestCase):
    def test_volume_scales_linearly_no_syringe(self):
        v1 = compile_to_trajectory(_line_sketch(1.0)).total_volume_uL
        v2 = compile_to_trajectory(_line_sketch(2.0)).total_volume_uL
        vh = compile_to_trajectory(_line_sketch(0.5)).total_volume_uL
        self.assertAlmostEqual(v2 / v1, 2.0, places=6)
        self.assertAlmostEqual(vh / v1, 0.5, places=6)

    def test_pump_column_scales_with_syringe(self):
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        syr = SyringeSpec(volume_uL=1000, stroke_length_mm=30.0,
                          barrel_id_mm=4.61)
        p1 = compile_to_trajectory(_line_sketch(1.0), needle, syr).trajectory
        p2 = compile_to_trajectory(_line_sketch(2.0), needle, syr).trajectory
        # Final cumulative plunger displacement (P1 col) doubles.
        disp1 = float(np.max(p1[:, 3]))
        disp2 = float(np.max(p2[:, 3]))
        self.assertGreater(disp1, 0.0)
        self.assertAlmostEqual(disp2 / disp1, 2.0, places=5)

    def test_volume_is_canonical_bore_area(self):
        # v7.5.x (Finding A): the baked volume now uses the CANONICAL bead model
        # — inner-bore cross-section × mult — identical to FlowPhysics /
        # GeometryEngine / Quick Print, so a sketch-baked print and a Quick Print
        # of the same needle+path extrude the same. It no longer depends on
        # layer_height and is independent of the fill-pitch (line_spacing).
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        sk = _line_sketch(1.0)
        sk.line_spacing_mm = 0.72          # deliberately ≠ inner Ø — must NOT matter
        c = compile_to_trajectory(sk, needle, None)
        area = np.pi * (0.413 / 2) ** 2    # bore cross-section (mm²)
        self.assertAlmostEqual(c.total_volume_uL, 10.0 * area, places=4)

    def test_volume_independent_of_layer_height(self):
        # Canonical model ignores layer_height for volume (it is only the Z step
        # between stacked layers) — the OLD Sketch model scaled with it.
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        sk_a = _line_sketch(1.0)
        sk_a.layer_height_mm = 0.1
        sk_b = _line_sketch(1.0)
        sk_b.layer_height_mm = 0.4
        va = compile_to_trajectory(sk_a, needle, None).total_volume_uL
        vb = compile_to_trajectory(sk_b, needle, None).total_volume_uL
        self.assertAlmostEqual(va, vb, places=6)

    def test_geometry_unchanged_by_multiplier(self):
        # The XY toolpath (where the needle goes) must not depend on extrusion.
        t1 = compile_to_trajectory(_line_sketch(1.0)).trajectory
        t2 = compile_to_trajectory(_line_sketch(3.0)).trajectory
        self.assertEqual(t1.shape, t2.shape)
        np.testing.assert_allclose(t1[:, :2], t2[:, :2])


# ═══════════════════════════════════════════════════════════════════
# Canvas
# ═══════════════════════════════════════════════════════════════════

class TestCanvas(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_set_bead_width_and_thickness_toggle(self):
        from gui.widgets.sketch_canvas import SketchCanvas
        c = SketchCanvas()
        c.set_bead_width_mm(0.83)
        self.assertAlmostEqual(c._bead_width_mm, 0.83, places=6)
        c.set_show_thickness(True)
        self.assertTrue(c._show_thickness)
        # Negative / None coerce to 0 (fall back to fill pitch).
        c.set_bead_width_mm(-5)
        self.assertEqual(c._bead_width_mm, 0.0)


# ═══════════════════════════════════════════════════════════════════
# Page wiring
# ═══════════════════════════════════════════════════════════════════

class TestPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _cfg(self):
        class Cfg:
            needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
            pumps = {}
            active_plate_key = 96
        return Cfg()

    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_thickness_on_by_default(self):
        page = self._page()
        self.assertTrue(page._thickness_btn.isChecked())
        self.assertTrue(page._canvas._show_thickness)

    def test_needle_inner_diameter_is_1x_reference(self):
        page = self._page()
        page.set_hardware_config(self._cfg())
        self.assertAlmostEqual(page._needle_id_mm, 0.413, places=4)
        self.assertAlmostEqual(page._bead_ref_mm(), 0.413, places=4)
        self.assertAlmostEqual(page._bead_width_mm(), 0.413, places=4)
        # Pushed to the canvas band.
        self.assertAlmostEqual(page._canvas._bead_width_mm, 0.413, places=4)

    def test_multiplier_scales_canvas_band(self):
        page = self._page()
        page.set_hardware_config(self._cfg())
        page._on_extrusion_changed(2.0)
        self.assertAlmostEqual(page._canvas._bead_width_mm, 0.826, places=4)
        self.assertEqual(page._canvas.sketch().extrusion_multiplier, 2.0)

    def test_bead_falls_back_to_fill_pitch_without_needle(self):
        page = self._page()
        # No needle configured → 1× reference = fill pitch (line_spacing_mm).
        page._canvas.sketch().line_spacing_mm = 0.5
        page._canvas.sketch().extrusion_multiplier = 1.0
        self.assertEqual(page._needle_id_mm, 0.0)
        self.assertAlmostEqual(page._bead_ref_mm(), 0.5, places=6)

    def test_extra_params_carry_multiplier(self):
        # The baked print records the multiplier for provenance.
        from unittest.mock import patch
        from PySide6.QtWidgets import QMessageBox
        page = self._page()
        page.set_hardware_config(self._cfg())
        page._canvas.sketch().shapes.append(
            SketchShape(kind="line", points=[(0.0, 0.0), (5.0, 0.0)]))
        page._canvas.sketch().extrusion_multiplier = 1.5
        captured = {}

        def _fake_save(trajectory, **kwargs):
            captured.update(kwargs.get("extra_params", {}))
            return "Sketch_1"

        # The line may cross the needle-safe ring → a confirm dialog; auto-Yes
        # so the (offscreen) modal doesn't block.
        with patch("gui.pages.print_builder_sketch."
                   "save_trajectory_as_print_object", _fake_save), \
                patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                      return_value=QMessageBox.Yes):
            page._send_to_print_setup()
        self.assertIn("extrusion_multiplier", captured)
        self.assertEqual(captured["extrusion_multiplier"], 1.5)


if __name__ == "__main__":
    unittest.main()
