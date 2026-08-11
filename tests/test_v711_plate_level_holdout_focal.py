"""
The hold-out gate, for FOCAL-readout planes.

``PlateZPlane``'s own docstring says the leave-one-out hold-out — not R² — is the
acceptance gate, because with three points the anchor-constrained fit is exact and
R² is identically 1.0. But ``holdout_error_mm`` filters on ``z_zref_mm is not
None``, which a focal reading never has, so ``from_focal_readings`` passed
``holdout=None`` and ``_validate_plate_z_plane`` skipped its hold-out branch
entirely (``if hold is not None``).

**The gate was therefore not enforced on the one mode that measures the plate
optically.** These tests pin the fix, including the case that matters most: a
corruption small enough to pass the residual check while the hold-out catches it.
"""

import dataclasses
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.PlateZPlane import (            # noqa: E402
    ZPlanePoint, from_focal_readings, holdout_focal_error_mm)
from SupportClasses.StageController import StageController   # noqa: E402

TILT = 0.0199                       # this machine's own measured gradient
CORNERS = [(0.0, 0.0), (60000.0, 0.0), (0.0, 40000.0), (60000.0, 40000.0)]


def focal_pts(err_um=0.0, bad_index=2, n=4):
    out = []
    for i, (x, y) in enumerate(CORNERS[:n]):
        f = 1.000 + TILT * y / 1000.0
        if i == bad_index:
            f += err_um / 1000.0
        out.append(ZPlanePoint(label=f"S{i}", x_stage_um=x, y_stage_um=y,
                               focal_mm=f, source="focal_readout",
                               surface="well_glass_bottom"))
    return out


def anchor(surface="well_glass_bottom"):
    return ZPlanePoint(label="anchor", x_stage_um=0.0, y_stage_um=0.0,
                       z_zref_mm=0.090, source="needle_touch", surface=surface)


def stub_controller():
    c = StageController.__new__(StageController)
    c._plate_bottom_z_zref = 0.090
    c._plate_bottom_anchor_xy_um = (0.0, 0.0)
    c._plate_z_plane = None
    c.zero_position = {"Z": 0.0}
    return c


class TestHoldoutIsComputed(unittest.TestCase):
    def test_none_below_four_focal_points(self):
        """Dropping one must still leave two independent directions."""
        self.assertIsNone(holdout_focal_error_mm(focal_pts(n=3), 1))

    def test_about_zero_on_a_clean_planar_survey(self):
        self.assertAlmostEqual(holdout_focal_error_mm(focal_pts(), 1), 0.0,
                               places=9)

    def test_equals_the_injected_error(self):
        h = holdout_focal_error_mm(focal_pts(err_um=400.0), 1)
        self.assertAlmostEqual(h * 1000.0, 400.0, places=3)

    def test_from_focal_readings_now_stamps_it(self):
        """The regression against the shipped `holdout=None`."""
        plane, why = from_focal_readings(points=focal_pts(err_um=400.0),
                                         anchor=anchor(), focal_sign=1)
        self.assertIsNotNone(plane, why)
        self.assertIsNotNone(plane.holdout_error_mm)
        self.assertAlmostEqual(plane.holdout_error_mm * 1000.0, 400.0, places=3)

    def test_sign_does_not_change_the_magnitude(self):
        a = holdout_focal_error_mm(focal_pts(err_um=200.0), 1)
        b = holdout_focal_error_mm(focal_pts(err_um=200.0), -1)
        self.assertAlmostEqual(a, b, places=9)


class TestTheGateNowFires(unittest.TestCase):
    def test_a_corrupted_focal_survey_is_rejected(self):
        plane, _ = from_focal_readings(points=focal_pts(err_um=400.0),
                                       anchor=anchor(), focal_sign=1)
        ok, why = stub_controller().set_plate_z_plane(plane)
        self.assertFalse(ok)
        self.assertTrue(why)

    def test_the_case_only_the_holdout_can_catch(self):
        """140 µm of corruption produces a residual of ~47 µm, which PASSES the
        50 µm residual gate. Before the fix this plane was ACCEPTED. That is a
        140 µm plate-bottom error installed silently."""
        plane, _ = from_focal_readings(points=focal_pts(err_um=140.0),
                                       anchor=anchor(), focal_sign=1)
        self.assertLess(plane.residual_max_mm * 1000.0, 50.0)

        before = dataclasses.replace(plane, holdout_error_mm=None)
        ok_before, _ = stub_controller().set_plate_z_plane(before)
        self.assertTrue(ok_before, "precondition: this is what used to happen")

        ok_after, why = stub_controller().set_plate_z_plane(plane)
        self.assertFalse(ok_after)
        self.assertIn("hold-out", why)

    def test_a_clean_survey_still_passes(self):
        plane, _ = from_focal_readings(points=focal_pts(), anchor=anchor(),
                                       focal_sign=1)
        ok, why = stub_controller().set_plate_z_plane(plane)
        self.assertTrue(ok, why)


class TestSurfaceMixing(unittest.TestCase):
    def test_anchor_on_a_different_surface_is_refused(self):
        """If the needle touched the well glass bottom while the scope focused on
        the coverslip's outer face, the whole plane is offset by the substrate
        thickness — a systematic common to every site, so no numeric gate
        downstream can see it."""
        plane, why = from_focal_readings(points=focal_pts(),
                                         anchor=anchor(surface="plate_top"),
                                         focal_sign=1)
        self.assertIsNone(plane)
        self.assertIn("surfaces", why)

    def test_matching_surfaces_pass(self):
        plane, why = from_focal_readings(points=focal_pts(), anchor=anchor(),
                                         focal_sign=1)
        self.assertIsNotNone(plane, why)


class TestFocusSigmaRoundTrip(unittest.TestCase):
    def test_focus_sigma_survives_serialisation(self):
        p = ZPlanePoint(label="S", x_stage_um=1.0, y_stage_um=2.0,
                        focal_mm=1.0, focus_sigma_um=1.73)
        back = ZPlanePoint.from_dict(p.to_dict())
        self.assertAlmostEqual(back.focus_sigma_um, 1.73)

    def test_absent_when_unset_so_legacy_records_stay_compact(self):
        p = ZPlanePoint(label="S", x_stage_um=1.0, y_stage_um=2.0)
        self.assertNotIn("focus_sigma_um", p.to_dict())


if __name__ == "__main__":
    unittest.main()
