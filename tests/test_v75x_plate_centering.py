"""
v7.5.x — Default plate centering in the XY safety envelope.

Verifies:
  * SafetyLimits.xy_center() returns the ABSOLUTE envelope midpoint.
  * StageController.default_plate_center_um() returns that absolute midpoint
    directly (v7.5.x: the envelope is absolute stage µm, so no zero_position
    offset is added — the result is independent of zero_position).
  * End-to-end: seeding WellPlate.get_all_positions_from_plate_center() at the
    envelope centre and converting back to zero-ref µm translates the plate by
    exactly the bounds-centre shift (asymmetric vs symmetric envelope).
"""

import unittest
from types import SimpleNamespace

from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import StageController
from SupportClasses.WellPlate import WellPlate


# Real-world asymmetric (positive-only) envelope from config/hardware/devices/ME3B V1.json
ASYM = dict(xy_min_x=0.0, xy_max_x=114332.0, xy_min_y=0.0, xy_max_y=76645.0)
ASYM_CENTER = (114332.0 / 2.0, 76645.0 / 2.0)  # (57166.0, 38322.5)


def _bbox_center(positions: dict) -> tuple[float, float]:
    xs = [p[0] for p in positions.values()]
    ys = [p[1] for p in positions.values()]
    return ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)


def _zero_ref(positions: dict, zx: float, zy: float) -> dict:
    """Mirror of the calibration/jog display conversion: absolute → zero-ref."""
    return {n: (x - zx, y - zy) for n, (x, y) in positions.items()}


class TestSafetyLimitsCenter(unittest.TestCase):
    def test_symmetric_default_is_origin(self):
        sl = SafetyLimits()  # ±130000 / ±85000
        self.assertEqual(sl.xy_center(), (0.0, 0.0))

    def test_asymmetric_envelope_midpoint(self):
        sl = SafetyLimits(**ASYM)
        cx, cy = sl.xy_center()
        self.assertAlmostEqual(cx, ASYM_CENTER[0])
        self.assertAlmostEqual(cy, ASYM_CENTER[1])


class TestDefaultPlateCenterUm(unittest.TestCase):
    """Call the real method bound to a lightweight stand-in to avoid spinning
    up the StageController threads (watchdog/pollers)."""

    @staticmethod
    def _call(sl: SafetyLimits, zero: dict) -> tuple[float, float]:
        fake = SimpleNamespace(safety_limits=sl, zero_position=zero)
        return StageController.default_plate_center_um(fake)

    def test_symmetric_default_is_origin(self):
        # v7.5.x: the envelope is absolute, so the symmetric default midpoint
        # is (0, 0) regardless of zero_position (no offset is added).
        for zero in ({"x": 0.0, "y": 0.0}, {"x": 1234.0, "y": -567.0}):
            cx, cy = self._call(SafetyLimits(), zero)
            self.assertAlmostEqual(cx, 0.0)
            self.assertAlmostEqual(cy, 0.0)

    def test_asymmetric_is_envelope_midpoint(self):
        # The absolute midpoint of the envelope, independent of zero_position.
        for zero in ({"x": 0.0, "y": 0.0}, {"x": 1000.0, "y": 2000.0}):
            cx, cy = self._call(SafetyLimits(**ASYM), zero)
            self.assertAlmostEqual(cx, ASYM_CENTER[0])
            self.assertAlmostEqual(cy, ASYM_CENTER[1])


class TestEndToEndTranslation(unittest.TestCase):
    def test_plate_translates_by_bounds_center_shift(self):
        plate = WellPlate.from_format(96)
        zx, zy = 0.0, 0.0

        sym_center = StageController.default_plate_center_um(
            SimpleNamespace(safety_limits=SafetyLimits(),
                            zero_position={"x": zx, "y": zy}))
        asym_center = StageController.default_plate_center_um(
            SimpleNamespace(safety_limits=SafetyLimits(**ASYM),
                            zero_position={"x": zx, "y": zy}))

        sym_disp = _zero_ref(
            plate.get_all_positions_from_plate_center(*sym_center), zx, zy)
        asym_disp = _zero_ref(
            plate.get_all_positions_from_plate_center(*asym_center), zx, zy)

        sc = _bbox_center(sym_disp)
        ac = _bbox_center(asym_disp)

        # The displayed plate must shift by exactly the bounds-centre delta.
        self.assertAlmostEqual(ac[0] - sc[0], ASYM_CENTER[0], places=3)
        self.assertAlmostEqual(ac[1] - sc[1], ASYM_CENTER[1], places=3)

    def test_asymmetric_plate_centered_in_envelope(self):
        # The displayed plate footprint centre should coincide with the
        # envelope centre (zero-ref) for the asymmetric envelope.
        plate = WellPlate.from_format(96)
        zx, zy = 0.0, 0.0
        center = StageController.default_plate_center_um(
            SimpleNamespace(safety_limits=SafetyLimits(**ASYM),
                            zero_position={"x": zx, "y": zy}))
        disp = _zero_ref(
            plate.get_all_positions_from_plate_center(*center), zx, zy)
        # The well-grid bbox centre sits at the plate footprint centre, which
        # get_a1_from_plate_center pins to the envelope centre.
        gc = _bbox_center(disp)
        self.assertAlmostEqual(gc[0], ASYM_CENTER[0], places=3)
        self.assertAlmostEqual(gc[1], ASYM_CENTER[1], places=3)


if __name__ == "__main__":
    unittest.main()
