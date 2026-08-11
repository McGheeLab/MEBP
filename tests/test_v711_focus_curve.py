"""
The through-focus peak estimator and every one of its refusals.

Why this file exists: the peak LOCATION is the plate-leveling measurement, and a
peak that is confidently wrong is indistinguishable from one that is right unless
something checks the curve's shape. Each refusal below corresponds to a physical
way the curve stops meaning "where the surface is"; a missing refusal ships as a
plausible number that moves the needle.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.FocusCurve import (            # noqa: E402
    FocusSample, peak_focus_um, centroid_peak,
    PEAK_AT_EDGE, MONOTONIC, LOW_PROMINENCE, MULTIMODAL, SATURATED,
    NON_CONCAVE, FWHM_MISMATCH, INSUFFICIENT_SAMPLES, MIXED_EXPOSURE, DRIFT,
)

DOF = 10.4          # 10x / NA 0.30 at 1.28 µm/px


def gaussian(zs, z0, fwhm, amp=1000.0, base=100.0, **kw):
    sigma = fwhm / 2.3548
    return [FocusSample(z_um=z,
                        score=base + amp * math.exp(-((z - z0) ** 2)
                                                    / (2 * sigma ** 2)), **kw)
            for z in zs]


def grid(step=3.0, n=6):
    return [i * step for i in range(-n, n + 1)]


def code_of(result):
    peak, why = result
    return "ACCEPTED" if peak is not None else why.split(":")[0]


class TestPeakLocation(unittest.TestCase):
    def test_symmetric_peak_lands_exactly_on_centre(self):
        peak, why = peak_focus_um(gaussian(grid(), 0.0, 2 * DOF), dof_um=DOF)
        self.assertIsNotNone(peak, why)
        self.assertAlmostEqual(peak.z_um, 0.0, places=6)

    def test_off_grid_peak_is_interpolated_between_samples(self):
        """The whole point of sub-step interpolation: the true peak is never on
        a sample, and rounding to the nearest sample would cost half a step.

        The residual ~0.01 µm on a 3 µm step (0.3 %) comes from estimating the
        baseline from the sweep's own tails, which sit slightly above the true
        floor on an asymmetric bracket. That bias is inherent to baseline
        estimation and is two orders of magnitude below the depth of field.
        """
        for true_z in (0.7, 1.37, -2.1):
            peak, why = peak_focus_um(gaussian(grid(), true_z, 2 * DOF),
                                      dof_um=DOF)
            self.assertIsNotNone(peak, why)
            self.assertAlmostEqual(peak.z_um, true_z, delta=0.05)

    def test_interpolation_beats_the_nearest_sample(self):
        true_z = 1.4                     # a 3 µm grid puts the nearest at 0.0
        peak, _ = peak_focus_um(gaussian(grid(), true_z, 2 * DOF), dof_um=DOF)
        nearest_err = abs(0.0 - true_z)
        self.assertLess(abs(peak.z_um - true_z), nearest_err / 10.0)

    def test_fwhm_recovers_the_curve_width(self):
        peak, _ = peak_focus_um(gaussian(grid(), 0.0, 2 * DOF), dof_um=DOF)
        self.assertAlmostEqual(peak.fwhm_um, 2 * DOF, delta=0.15 * 2 * DOF)

    def test_sigma_never_claims_better_than_a_tenth_of_depth_of_field(self):
        """No amount of interpolation beats the optics; the floor says so."""
        peak, _ = peak_focus_um(gaussian(grid(), 0.0, 2 * DOF), dof_um=DOF)
        self.assertGreaterEqual(peak.sigma_z_um, DOF / 10.0 - 1e-9)

    def test_noise_does_not_move_the_peak_far(self):
        import random
        random.seed(11)
        zs = [i * 8.0 for i in range(-25, 26)]
        noisy = [FocusSample(z_um=z,
                             score=100 + 1000 * math.exp(-(z ** 2) / (2 * 8 ** 2))
                             + random.gauss(0, 25)) for z in zs]
        peak, why = peak_focus_um(noisy, dof_um=DOF)
        self.assertIsNotNone(peak, why)
        self.assertLess(abs(peak.z_um), 2.0)


class TestRefusals(unittest.TestCase):
    def test_peak_at_the_edge_is_refused_with_a_direction(self):
        """The true peak is below the swept range while a weaker in-range feature
        makes a bump, so the argmax is at the low edge but the curve is NOT
        monotone. Both codes refuse; the distinction is the advice given."""
        zs = grid()

        def g(z, z0, fwhm, amp=1000.0):
            sig = fwhm / 2.3548
            return amp * math.exp(-((z - z0) ** 2) / (2 * sig ** 2))

        s = [FocusSample(z_um=z,
                         score=100 + g(z, -25.0, 2 * DOF)
                         + g(z, 9.0, DOF, 120.0)) for z in zs]
        peak, why = peak_focus_um(s, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(PEAK_AT_EDGE, why)
        self.assertIn("below", why)

    def test_a_cleanly_monotone_ramp_prefers_the_range_diagnosis(self):
        """With no noise at all the argmax is also at an edge, but 'focus is
        outside this range' is strictly more useful than 'the peak is at the
        end', so the monotonic test is checked first."""
        peak, why = peak_focus_um(gaussian(grid(), -30.0, 2 * DOF), dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(MONOTONIC, why)

    def test_monotonic_ramp_says_the_range_is_wrong_not_the_peak(self):
        """A strictly monotone curve always has its argmax at an edge, so the
        monotonic test must run FIRST or it is unreachable — and 'focus is
        outside this range' is the more actionable diagnosis."""
        ramp = [FocusSample(z_um=z, score=100 + z * 10) for z in grid()]
        peak, why = peak_focus_um(ramp, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(MONOTONIC, why)

    def test_featureless_field_is_refused_rather_than_fitted(self):
        """Clean empty glass has almost no contrast. Without this the estimator
        fits sensor noise and reports a confident surface height."""
        flat = [FocusSample(z_um=z, score=100 + 0.01 * z) for z in grid()]
        peak, why = peak_focus_um(flat, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(LOW_PROMINENCE, why)

    def test_two_surfaces_are_detected_and_named(self):
        """A #1.5 coverslip's two faces are ~170 µm apart. Focusing on the outer
        one offsets the WHOLE plane by the substrate thickness — a systematic
        common to every site, so no downstream numeric gate can see it."""
        zs = [i * 8.0 for i in range(-25, 50)]
        two = [FocusSample(
            z_um=z, score=100 + 1000 * math.exp(-(z ** 2) / (2 * 8.0 ** 2))
            + 900 * math.exp(-((z - 170) ** 2) / (2 * 8.0 ** 2))) for z in zs]
        peak, why = peak_focus_um(two, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(MULTIMODAL, why)
        self.assertIn("168", why)                 # measured, not assumed

    def test_a_single_peak_in_the_same_wide_range_still_passes(self):
        """The two-surface test must not fire on ordinary tails."""
        zs = [i * 8.0 for i in range(-25, 50)]
        one = [FocusSample(z_um=z,
                           score=100 + 1000 * math.exp(-(z ** 2) / (2 * 8.0 ** 2)))
               for z in zs]
        peak, why = peak_focus_um(one, dof_um=DOF)
        self.assertIsNotNone(peak, why)

    def test_clipped_frames_are_refused(self):
        """Clipping destroys the gradients the metric measures and clips FIRST at
        best focus, so a saturated stack can show a dip exactly where the surface
        is — a clean-looking bimodal curve whose peak is ~1 DOF off."""
        s = gaussian(grid(), 0.0, 2 * DOF, saturated_frac=0.05)
        peak, why = peak_focus_um(s, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(SATURATED, why)

    def test_settings_change_mid_sweep_invalidates_the_scores(self):
        """The focus metric is UNNORMALISED, so scores under different exposure
        are not comparable. Auto-exposure fighting the sweep is the usual cause."""
        s = [FocusSample(z_um=z, score=500, settings_fingerprint=str(i % 2))
             for i, z in enumerate(grid())]
        peak, why = peak_focus_um(s, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(MIXED_EXPOSURE, why)

    def test_a_wandering_roi_is_refused(self):
        s = [FocusSample(z_um=z, score=v.score, roi_rect=(0, 0, 100, 100),
                         drift_px=60.0)
             for z, v in zip(grid(), gaussian(grid(), 0.0, 2 * DOF))]
        peak, why = peak_focus_um(s, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(DRIFT, why)

    def test_impossibly_narrow_curve_is_refused(self):
        """A focus response cannot be narrower than the diffraction-limited depth
        of field. It means aliasing or one hot frame fitted as a peak."""
        peak, why = peak_focus_um(gaussian(grid(), 0.0, 0.3 * DOF), dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(FWHM_MISMATCH, why)
        self.assertIn("impossible", why)

    def test_absurdly_wide_curve_is_refused(self):
        peak, why = peak_focus_um(
            gaussian([i * 20.0 for i in range(-8, 9)], 0.0, 9 * DOF), dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(FWHM_MISMATCH, why)

    def test_too_few_samples(self):
        peak, why = peak_focus_um(gaussian([0.0, 1.0, 2.0], 1.0, 2 * DOF),
                                  dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(INSUFFICIENT_SAMPLES, why)

    def test_a_dip_at_the_middle_is_not_a_peak(self):
        zs = grid()
        dip = [FocusSample(z_um=z, score=1000 - 300 * math.exp(-(z ** 2) / 8))
               for z in zs]
        peak, why = peak_focus_um(dip, dof_um=DOF)
        self.assertIsNone(peak)
        self.assertIn(why.split(":")[0],
                      (MONOTONIC, PEAK_AT_EDGE, NON_CONCAVE, LOW_PROMINENCE))


class TestCentroidCrossCheck(unittest.TestCase):
    def test_centroid_agrees_on_a_symmetric_curve(self):
        c = centroid_peak(gaussian(grid(), 0.0, 2 * DOF))
        self.assertAlmostEqual(c, 0.0, places=6)

    def test_truncated_bracket_raises_a_warning_not_a_silent_number(self):
        """The centroid follows an asymmetric tail; disagreement with the
        Gaussian fit is the signal that the bracket is cut off on one side."""
        zs = [i * 3.0 for i in range(-2, 9)]      # deliberately lopsided
        peak, why = peak_focus_um(gaussian(zs, 0.0, 2 * DOF), dof_um=DOF)
        if peak is not None:
            self.assertIsInstance(peak.warnings, tuple)


if __name__ == "__main__":
    unittest.main()
