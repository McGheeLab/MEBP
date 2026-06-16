"""Tests for v7.5.x print-Z reference vector.

The print-Z up-direction is derived from the calibrated plate-bottom→plate-top
vector instead of the hard-coded module ``ZDIR``, so print offsets are correct
on both a conventional machine and ME3B V1 (needle descends as raw Z increases).

Covers:
  - ``derive_z_up_sign()`` — +1 when the plate top is a larger zero-ref Z than
    the bottom, -1 when smaller, and the fallback when either point is missing
    or the two coincide;
  - ``StageController.print_z_dir()`` — derived sign when both datums are set,
    else the module ``ZDIR``;
  - datum conversions + the print floor follow the DERIVED sign even when the
    module ``ZDIR`` is set the opposite way (geometry wins over the constant);
  - ``build_well_plate_job`` layer stacking steps along ``z_up_sign`` so layers
    grow UP on either polarity;
  - ``PrintTrajectoryPlanner._well_print_z`` ink-Z is sign-correct both ways.
"""

import importlib
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Fetch the real module object (the package re-exports the StageController
# *class* under the same name) so we can monkeypatch its module-level ZDIR.
SC = importlib.import_module("SupportClasses.StageController")
from SupportClasses.StageController import StageController, derive_z_up_sign


class TestDeriveZUpSign(unittest.TestCase):
    """The pure reference-vector → up-direction helper."""

    def test_top_above_bottom_is_conventional(self):
        # plate top a LARGER zero-ref Z than the bottom → up is +Z
        self.assertEqual(derive_z_up_sign(2.5, 0.0), 1.0)

    def test_top_below_bottom_is_inverted(self):
        # ME3B V1: plate top a SMALLER zero-ref Z than the (deeper) bottom
        self.assertEqual(derive_z_up_sign(-28.0, -25.68), -1.0)

    def test_fallback_when_a_point_missing(self):
        self.assertEqual(derive_z_up_sign(None, -25.68, fallback=-1.0), -1.0)
        self.assertEqual(derive_z_up_sign(-28.0, None, fallback=7.0), 7.0)
        self.assertEqual(derive_z_up_sign(None, None, fallback=1.0), 1.0)

    def test_fallback_when_coincident(self):
        self.assertEqual(derive_z_up_sign(3.0, 3.0, fallback=-1.0), -1.0)
        # within epsilon counts as coincident
        self.assertEqual(derive_z_up_sign(3.0, 3.0 + 1e-9, fallback=1.0), 1.0)

    def test_default_fallback_is_module_zdir(self):
        self.assertEqual(derive_z_up_sign(None, None), SC.ZDIR)


class TestPrintZDirController(unittest.TestCase):
    """StageController.print_z_dir() + datum/floor routed through it."""

    def _ctrl(self):
        # Bypass the heavy __init__ (serial / pollers); the datum + clamp logic
        # only needs these attributes.
        c = StageController.__new__(StageController)
        c._plate_bottom_z_zref = None
        c._plate_top_z_zref = None
        c._print_floor_active = False
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0,
                           "P1": 0.0, "P2": 0.0, "P3": 0.0}
        return c

    def test_set_get_plate_top(self):
        c = self._ctrl()
        c.set_plate_top_z(-28.0)
        self.assertAlmostEqual(c.get_plate_top_z(), -28.0)
        c.set_plate_top_z(None)
        self.assertIsNone(c.get_plate_top_z())

    def test_dir_from_geometry(self):
        c = self._ctrl()
        c.set_plate_bottom_z(-25.68)
        c.set_plate_top_z(-28.0)        # top higher (smaller zref) → up = -1
        self.assertEqual(c.print_z_dir(), -1.0)
        c.set_plate_top_z(-20.0)        # top now larger zref → up = +1
        self.assertEqual(c.print_z_dir(), 1.0)

    def test_dir_falls_back_to_zdir(self):
        c = self._ctrl()
        c.set_plate_bottom_z(-25.68)    # no plate top → fall back to ZDIR
        self.assertEqual(c.print_z_dir(), SC.ZDIR)

    def test_geometry_overrides_module_zdir(self):
        # The derived sign must win even when the module constant is opposite.
        old = SC.ZDIR
        SC.ZDIR = 1.0
        try:
            c = self._ctrl()
            c.set_plate_bottom_z(-25.68)
            c.set_plate_top_z(-28.0)    # geometry says up = -1
            self.assertEqual(c.print_z_dir(), -1.0)
            # height-above-bottom conversion follows the derived sign
            self.assertAlmostEqual(c.print_height_to_zref(0.2), -25.88)
            self.assertAlmostEqual(c.zref_to_print_height(-25.88), 0.2)
            # floor clamp + violation follow the derived sign, not ZDIR=+1
            c.set_print_floor_active(True)
            self.assertAlmostEqual(c._apply_print_floor_raw(-20.0), -25.68)
            self.assertAlmostEqual(c._apply_print_floor_raw(-30.0), -30.0)
            self.assertTrue(c.print_floor_violation(-20.0))   # deeper
            self.assertFalse(c.print_floor_violation(-30.0))  # higher (safe)
        finally:
            SC.ZDIR = old


class TestLayerStackingDirection(unittest.TestCase):
    """build_well_plate_job steps layers along z_up_sign (build UP either way)."""

    def _layer_zs(self, up_sign):
        from SupportClasses.PrintManager import (
            build_well_plate_job, PrintSettings, CommandType)
        s = PrintSettings()
        s.print_z_height = -25.0
        s.layer_height = 0.2
        s.num_layers = 3
        s.z_up_sign = up_sign
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0)],
            settings=s, pump="P1", flow_rate=0.01,
        )
        return [c.params["z"] for c in job.commands
                if c.type == CommandType.MOVE_Z]

    def test_inverted_polarity_layers_decrease(self):
        # ME3B V1 (up = -1): each layer steps toward a SMALLER zero-ref Z (up).
        zs = self._layer_zs(-1.0)
        self.assertEqual(len(zs), 3)
        self.assertAlmostEqual(zs[0], -25.0)
        self.assertAlmostEqual(zs[1], -25.2)
        self.assertAlmostEqual(zs[2], -25.4)

    def test_conventional_polarity_layers_increase(self):
        zs = self._layer_zs(1.0)
        self.assertAlmostEqual(zs[0], -25.0)
        self.assertAlmostEqual(zs[1], -24.8)
        self.assertAlmostEqual(zs[2], -24.6)

    def test_default_settings_are_legacy_additive(self):
        # A fresh PrintSettings defaults z_up_sign=+1 → unchanged legacy behaviour.
        from SupportClasses.PrintManager import PrintSettings
        self.assertEqual(PrintSettings().z_up_sign, 1.0)


class TestWellPrintZInkDirection(unittest.TestCase):
    """_well_print_z applies ink_z_mm (rel. plate top) along z_up_sign."""

    def _planner(self):
        from SupportClasses.PrintTrajectoryPlanner import PrintTrajectoryPlanner
        return PrintTrajectoryPlanner.__new__(PrintTrajectoryPlanner)

    def _plate(self, ink_z_mm):
        class _Info:
            pass
        info = _Info()
        info.ink_z_mm = ink_z_mm

        class _Plate:
            def get_well_info(self, name):
                return info
        return _Plate()

    def _settings(self, up_sign, top_z=-28.0):
        class _S:
            pass
        s = _S()
        s.top_z_height = top_z
        s.z_up_sign = up_sign
        s.print_z_height = -25.0
        return s

    def test_into_well_is_consistent_both_polarities(self):
        # ink_z_mm = -2.0 means "2 mm into the well from the plate top".
        planner = self._planner()
        plate = self._plate(-2.0)
        # ME3B V1 (up=-1): into-well is a LARGER zero-ref Z than the top.
        z_inv = planner._well_print_z(plate, "A1", self._settings(-1.0))
        self.assertAlmostEqual(z_inv, -26.0)   # -28 + (-1)*(-2)
        self.assertGreater(z_inv, -28.0)        # deeper than the top
        # conventional (up=+1): into-well is a SMALLER zero-ref Z than the top.
        z_conv = planner._well_print_z(plate, "A1", self._settings(1.0))
        self.assertAlmostEqual(z_conv, -30.0)  # -28 + (1)*(-2)
        self.assertLess(z_conv, -28.0)

    def test_falls_back_to_print_z_when_no_ink_z(self):
        planner = self._planner()
        plate = self._plate(None)
        z = planner._well_print_z(plate, "A1", self._settings(-1.0))
        self.assertAlmostEqual(z, -25.0)       # settings.print_z_height

    def test_missing_z_up_sign_defaults_additive(self):
        # A settings object without z_up_sign → +1 (legacy additive).
        class _S:
            top_z_height = -28.0
            print_z_height = -25.0
        planner = self._planner()
        z = planner._well_print_z(self._plate(-2.0), "A1", _S())
        self.assertAlmostEqual(z, -30.0)       # -28 + 1*(-2)


if __name__ == "__main__":
    unittest.main()
