"""
Objective optics — the numbers that size a focus step and BOUND a focus sweep.

The working-distance bound is the primary guard against driving the objective's
front lens into the plate. It is tested here, in pure code, so it does not depend
on a microscope being present to be trustworthy.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.ObjectiveOptics import (        # noqa: E402
    ObjectiveOptics, depth_of_field_um, fov_um, achievable_focus_sigma_um,
    wd_bounded_half_range_um, expected_fwhm_band_um, hard_fwhm_band_um,
    check_field_number, refuse_if_incomplete, from_mounted_optic,
    WD_SWEEP_FRACTION, NO_WD_FALLBACK_HALF_RANGE_UM,
)


def optic(label, na, wd, upp, **kw):
    return ObjectiveOptics(label=label, numerical_aperture=na,
                           working_distance_mm=wd, um_per_px_sample=upp,
                           frame_wh=(2600, 2048), **kw)


O4 = optic("4x", 0.13, 16.4, 3.20)
O10 = optic("10x", 0.30, 16.0, 1.28)
O20 = optic("20x", 0.45, 1.0, 0.64)


class TestDepthOfField(unittest.TestCase):
    def test_matches_berek_by_hand(self):
        """DOF = lambda*n/NA^2 + n*(um/px)/NA, computed independently."""
        expect = 0.55 / (0.30 ** 2) + 1.28 / 0.30
        self.assertAlmostEqual(depth_of_field_um(O10), expect, places=6)

    def test_falls_with_increasing_na(self):
        d = [depth_of_field_um(o) for o in (O4, O10, O20)]
        self.assertTrue(d[0] > d[1] > d[2], d)

    def test_is_none_without_na_or_scale(self):
        self.assertIsNone(depth_of_field_um(ObjectiveOptics(label="x")))
        self.assertIsNone(depth_of_field_um(
            ObjectiveOptics(label="x", numerical_aperture=0.3)))

    def test_sigma_is_a_sixth_of_depth_of_field(self):
        self.assertAlmostEqual(achievable_focus_sigma_um(O10),
                               depth_of_field_um(O10) / 6.0, places=9)


class TestWorkingDistanceBound(unittest.TestCase):
    def test_bound_is_a_fraction_of_working_distance(self):
        half, known = wd_bounded_half_range_um(O10)
        self.assertTrue(known)
        self.assertAlmostEqual(half, 16.0 * 1000.0 * WD_SWEEP_FRACTION)

    def test_a_short_working_distance_objective_cannot_sweep_a_millimetre(self):
        """The headline safety fact: a 20x with ~1 mm WD physically cannot do the
        +/-1 mm sweep that is fine on a 4x."""
        half, known = wd_bounded_half_range_um(O20)
        self.assertTrue(known)
        self.assertLess(half, 1000.0)
        self.assertAlmostEqual(half, 250.0)

    def test_unknown_working_distance_falls_back_tiny_and_says_so(self):
        """An unknown WD is not a licence to sweep far. The caller MUST be able
        to tell, because a silently narrow sweep reads downstream as 'no peak
        found' — an optics diagnosis for a missing-datum problem."""
        half, known = wd_bounded_half_range_um(
            ObjectiveOptics(label="?", numerical_aperture=0.45,
                            um_per_px_sample=0.64))
        self.assertFalse(known)
        self.assertEqual(half, NO_WD_FALLBACK_HALF_RANGE_UM)


class TestRefusals(unittest.TestCase):
    def test_missing_numerical_aperture_refuses_by_name(self):
        why = refuse_if_incomplete(ObjectiveOptics(label="9x"))
        self.assertIn("9x", why)
        self.assertIn("numerical aperture", why)

    def test_missing_scale_refuses_by_name(self):
        why = refuse_if_incomplete(
            ObjectiveOptics(label="9x", numerical_aperture=0.3))
        self.assertIn("µm/px", why)

    def test_complete_optic_does_not_refuse(self):
        self.assertEqual(refuse_if_incomplete(O10), "")


class TestFieldNumber(unittest.TestCase):
    def test_skipped_when_not_declared(self):
        self.assertEqual(check_field_number(O4), "")

    def test_declared_and_consistent_passes(self):
        o = optic("4x", 0.13, 16.4, 3.20, magnification=4.0,
                  field_number_mm=45.0)
        self.assertEqual(check_field_number(o), "")

    def test_declared_and_contradicted_reports_the_contradiction(self):
        """A wrong µm/px propagates into the geometric term of the depth of
        field, hence into every step size in the ladder."""
        o = optic("4x", 0.13, 16.4, 3.20, magnification=4.0,
                  field_number_mm=10.0)
        why = check_field_number(o)
        self.assertIn("field number", why)
        self.assertIn("mis-sizes", why)


class TestFromMountedOptic(unittest.TestCase):
    class _Optic:
        def __init__(self, **kw):
            self.position = kw.get("position", 1)
            self.present = kw.get("present", True)
            self.label = kw.get("label", "10x")
            self.code = kw.get("code", "MRH10101")
            self.magnification = kw.get("magnification", 10.0)
            self.numerical_aperture = kw.get("numerical_aperture", 0.30)
            self.working_distance_mm = kw.get("working_distance_mm", 16.0)
            self.detail = "10x · NA 0.3 · WD 16 mm"

    def test_reads_the_numeric_fields(self):
        o, why = from_mounted_optic(self._Optic(), um_per_px_sample=1.28,
                                    frame_wh=(2600, 2048))
        self.assertEqual(why, "")
        self.assertAlmostEqual(o.numerical_aperture, 0.30)
        self.assertAlmostEqual(o.working_distance_mm, 16.0)

    def test_never_parses_the_detail_string(self):
        """A parse failure there would silently produce a plausible plan built on
        a wrong NA or a wrong collision bound, so the numerics must be required."""
        bad = self._Optic(numerical_aperture=None, working_distance_mm=None)
        o, why = from_mounted_optic(bad, um_per_px_sample=1.28)
        self.assertIsNone(o)
        self.assertIn("numerical aperture", why)

    def test_empty_slot_refuses(self):
        o, why = from_mounted_optic(self._Optic(present=False),
                                    um_per_px_sample=1.28)
        self.assertIsNone(o)
        self.assertIn("empty", why)


class TestFwhmBands(unittest.TestCase):
    def test_expected_band_brackets_the_depth_of_field(self):
        lo, hi = expected_fwhm_band_um(O10)
        dof = depth_of_field_um(O10)
        self.assertLessEqual(lo, dof)
        self.assertGreater(hi, dof)

    def test_hard_band_is_wider_than_the_expected_band(self):
        elo, ehi = expected_fwhm_band_um(O10)
        hlo, hhi = hard_fwhm_band_um(O10)
        self.assertLess(hlo, elo)
        self.assertGreater(hhi, ehi)


class TestFieldOfView(unittest.TestCase):
    def test_fov_scales_with_frame_and_scale(self):
        w, h = fov_um(O10)
        self.assertAlmostEqual(w, 2600 * 1.28)
        self.assertAlmostEqual(h, 2048 * 1.28)

    def test_higher_magnification_sees_less(self):
        self.assertLess(fov_um(O20)[0], fov_um(O4)[0])


if __name__ == "__main__":
    unittest.main()
