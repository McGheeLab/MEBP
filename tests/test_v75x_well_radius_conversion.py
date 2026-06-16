"""
v7.5.x — Plate-calibration well radius / well-position unit conversion.

Regression guard for the "move to well points" bug: calibration.py read
`_mm`-suffixed attribute names that don't exist on the geometry dataclasses
(`WellPlate.well_diameter_mm`, `WellInfo.x_mm/.y_mm`), so every "move to well
rim" raised AttributeError — swallowed by the surrounding try/except and logged
as "move failed", so the stage silently never moved.

These tests assert the *attribute contract* the fixed code paths depend on, and
reproduce the two arithmetic expressions verbatim:

  * radius (µm) = well_diameter (mm) * 1000 / 2          [_ploc_* + _ploc_start]
  * delta (µm)  = (target.x - a1.x) * 1000               [_predict_well_xy]

mm is the geometry unit; the *_mm names must NOT come back.
"""

import unittest

from SupportClasses.WellPlate import WellInfo, WellPlate


class TestGeometryAttributeContract(unittest.TestCase):
    """The fixed code reads .well_diameter / .x / .y — guard the names."""

    def test_wellplate_has_well_diameter_not_mm_suffix(self):
        plate = WellPlate.from_format(96)
        self.assertTrue(hasattr(plate, "well_diameter"))
        self.assertFalse(
            hasattr(plate, "well_diameter_mm"),
            "WellPlate.well_diameter_mm reappeared — the calibration radius "
            "conversion will AttributeError again.",
        )
        self.assertGreater(plate.well_diameter, 0.0)  # mm

    def test_wellinfo_has_x_y_not_mm_suffix(self):
        plate = WellPlate.from_format(96)
        well = plate.get_all_wells()[0]
        self.assertIsInstance(well, WellInfo)
        for attr in ("x", "y", "diameter"):
            self.assertTrue(hasattr(well, attr))
        for bad in ("x_mm", "y_mm"):
            self.assertFalse(
                hasattr(well, bad),
                f"WellInfo.{bad} reappeared — _predict_well_xy will "
                "AttributeError again.",
            )


class TestRadiusConversion(unittest.TestCase):
    """radius_um = well_diameter(mm) * 1000 / 2 — the rim-nudge offset."""

    def test_radius_um_matches_half_diameter(self):
        plate = WellPlate.from_format(96)
        r_um = float(plate.well_diameter) * 1000.0 / 2.0
        self.assertAlmostEqual(r_um, plate.well_diameter * 500.0)
        self.assertGreater(r_um, 0.0)

    def test_radius_um_is_micrometers(self):
        # A 96-well plate's ~6.4 mm well → ~3200 µm radius (not ~3.2).
        plate = WellPlate.from_format(96)
        r_um = float(plate.well_diameter) * 1000.0 / 2.0
        self.assertGreater(r_um, 1000.0)


class TestPredictWellDeltaConversion(unittest.TestCase):
    """delta_um = (target.x - a1.x) * 1000 — the _predict_well_xy fallback."""

    def test_column_step_converts_mm_to_um(self):
        plate = WellPlate.from_format(96)
        wells = {w.name: w for w in plate.get_all_wells()}
        a1, a2 = wells["A1"], wells["A2"]
        dx_um = (a2.x - a1.x) * 1000.0
        dy_um = (a2.y - a1.y) * 1000.0
        # A1→A2 steps one column in X, none in Y.
        self.assertAlmostEqual(dx_um, plate.well_spacing_x * 1000.0)
        self.assertAlmostEqual(dy_um, 0.0)
        self.assertGreater(dx_um, 0.0)

    def test_predicted_um_offset_is_micrometers(self):
        plate = WellPlate.from_format(96)
        wells = {w.name: w for w in plate.get_all_wells()}
        a1, target = wells["A1"], wells["B2"]
        dx_um = (target.x - a1.x) * 1000.0
        dy_um = (target.y - a1.y) * 1000.0
        # One row + one column away → both offsets are millimetre-scale
        # spacings expressed in µm (thousands), never raw mm.
        self.assertGreater(abs(dx_um), 1000.0)
        self.assertGreater(abs(dy_um), 1000.0)


if __name__ == "__main__":
    unittest.main()
