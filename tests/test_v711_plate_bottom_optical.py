"""v7.11 — the non-contact plate-bottom arithmetic.

The gesture: focus on the glass, raise the focus by a margin X, park the needle
near that plane, sweep the FOCUS through the stationary tip. The tip's height
above the glass falls out as ``f_tip - f0`` — measured, not commanded.

Two properties carry the whole design and are tested first:

* the answer must NOT inherit the error in the guess that positioned the needle
  (otherwise the optical method is just a slower version of the guess); and
* a margin smaller than the longest bore's reach must be REFUSED, because the
  focus is scored on the datum bore while a longer one arrives at the glass
  first.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.PlateBottomOptical import (      # noqa: E402
    DEFAULT_OFFSETS_UM, DEFAULT_SPREAD_TOL_UM, MAX_OFFSET_UM,
    MIN_RUNGS_FOR_SLOPE, MIN_TIP_CLEARANCE_UM, RungMeasurement,
    focal_sign_and_offset, ladder_gate, measured_scale, needle_target_zref,
    plate_bottom_from_rung, reconcile_rungs, scale_verifiable,
    slope_precision_frac, tip_height_um)
from SupportClasses.PlateFocusDatumStore import SCALE_TOLERANCE  # noqa: E402

B_TRUE = 21.130          # plate bottom, zero-ref mm
F0 = 1200.0              # on-glass focus, µm


def rung(margin_um, *, zdir=-1.0, fus=1.0, height_err_um=0.0,
         focus_err_um=0.0, sigma=1.7, **kw):
    """A rung where the needle lands ``height_err_um`` off its target."""
    true_h = margin_um + height_err_um
    return RungMeasurement(
        margin_um=margin_um,
        needle_z_zref_mm=needle_target_zref(B_TRUE, true_h, zdir),
        focus_tip_um=F0 + (true_h * fus) + focus_err_um,
        focus_sigma_um=sigma, **kw)


class TestTheAnswerIsMeasuredNotCommanded(unittest.TestCase):
    """The reason the focus sweeps and the needle holds still."""

    def test_a_needle_that_lands_off_target_still_gives_the_right_bottom(self):
        for zdir in (-1.0, 1.0):
            for fus in (1.0, -1.0):
                for err in (0.0, 137.0, -137.0, 400.0):
                    r = rung(1000.0, zdir=zdir, fus=fus, height_err_um=err)
                    h = tip_height_um(r.focus_tip_um, F0, fus)
                    got = plate_bottom_from_rung(r.needle_z_zref_mm, h, zdir)
                    self.assertAlmostEqual(
                        got, B_TRUE, places=9,
                        msg=f"zdir={zdir} fus={fus} err={err}")

    def test_a_wrong_guess_costs_nothing(self):
        """The guess only decides where the needle is SENT. A 400 µm error in
        it must not appear in the result at all."""
        good = rung(1000.0)
        bad = rung(1000.0, height_err_um=-400.0)
        for r in (good, bad):
            h = tip_height_um(r.focus_tip_um, F0, 1.0)
            self.assertAlmostEqual(
                plate_bottom_from_rung(r.needle_z_zref_mm, h, -1.0),
                B_TRUE, places=9)

    def test_needle_target_and_plate_bottom_are_exact_inverses(self):
        for zdir in (-1.0, 1.0):
            for x in (50.0, 100.0, 1000.0, 2500.0):
                z = needle_target_zref(B_TRUE, x, zdir)
                self.assertAlmostEqual(
                    plate_bottom_from_rung(z, x, zdir), B_TRUE, places=9)


class TestTheLadderGate(unittest.TestCase):
    """A margin below the longest bore's reach drives it through the glass."""

    def test_a_long_bore_refuses_a_short_margin(self):
        ok, why = ladder_gate(DEFAULT_OFFSETS_UM, longest_bore_mm=0.200)
        self.assertFalse(ok)
        self.assertIn("100", why)
        self.assertIn("250", why)          # the derived floor
        self.assertIn("glass", why.lower())

    def test_the_floor_is_bore_plus_clearance(self):
        # 200 µm bore ⇒ floor 250 µm. 250 passes, 249 does not.
        self.assertTrue(ladder_gate([250.0], 0.200)[0])
        self.assertFalse(ladder_gate([249.0], 0.200)[0])
        self.assertEqual(MIN_TIP_CLEARANCE_UM, 50.0)

    def test_a_single_bore_needle_only_needs_the_clearance(self):
        self.assertTrue(ladder_gate(DEFAULT_OFFSETS_UM, 0.0)[0])
        self.assertFalse(ladder_gate([49.0], 0.0)[0])

    def test_margins_must_be_largest_first(self):
        ok, why = ladder_gate([100.0, 500.0], 0.0)
        self.assertFalse(ok)
        self.assertIn("largest first", why)
        self.assertIn("guess", why)

    def test_an_empty_or_bogus_list_refuses(self):
        self.assertFalse(ladder_gate([], 0.0)[0])
        self.assertFalse(ladder_gate(["x"], 0.0)[0])
        self.assertFalse(ladder_gate([0.0], 0.0)[0])
        self.assertFalse(ladder_gate([-100.0], 0.0)[0])
        self.assertFalse(ladder_gate([float("nan")], 0.0)[0])

    def test_a_millimetre_typed_as_a_micron_is_refused(self):
        ok, why = ladder_gate([MAX_OFFSET_UM + 1.0], 0.0)
        self.assertFalse(ok)
        self.assertIn("units", why)

    def test_duplicates_are_refused(self):
        ok, why = ladder_gate([500.0, 500.0], 0.0)
        self.assertFalse(ok)
        self.assertIn("twice", why)

    def test_a_bogus_bore_is_treated_as_zero_not_as_a_crash(self):
        self.assertTrue(ladder_gate([100.0], None)[0])
        self.assertTrue(ladder_gate([100.0], "nonsense")[0])


class TestReconcile(unittest.TestCase):
    def test_clean_rungs_agree_exactly(self):
        r = reconcile_rungs([rung(x) for x in DEFAULT_OFFSETS_UM],
                            focus_zero_um=F0, focus_up_sign=1.0, zdir=-1.0)
        self.assertTrue(r.ok, r.refusal)
        self.assertAlmostEqual(r.plate_bottom_zref_mm, B_TRUE, places=9)
        self.assertAlmostEqual(r.spread_um, 0.0, places=6)
        self.assertEqual(r.n_used, 4)

    def test_one_bad_rung_refuses_and_names_both_margins(self):
        rungs = [rung(1000.0), rung(500.0), rung(200.0),
                 rung(100.0, focus_err_um=140.0)]
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0)
        self.assertFalse(r.ok)
        self.assertAlmostEqual(r.spread_um, 140.0, places=3)
        self.assertIn("1000", r.refusal)
        self.assertIn("100", r.refusal)
        self.assertIn("backlash", r.refusal)

    def test_a_wild_outlier_is_refused_outright(self):
        """It trips BOTH gates — the rungs disagree AND the implied scale is
        nowhere near 1. Refusing is correct; a median would be a number."""
        rungs = [rung(1000.0), rung(500.0), rung(200.0),
                 rung(100.0, focus_err_um=1000.0)]
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0, spread_tol_um=float("inf"))
        self.assertIsNone(r.plate_bottom_zref_mm)
        self.assertTrue(r.refusal)

    def test_the_aggregate_is_the_median_not_the_mean(self):
        """One rung that locked onto a reflection must not drag the answer a
        quarter of the way toward itself."""
        rungs = [rung(1000.0), rung(500.0), rung(200.0),
                 rung(100.0, focus_err_um=1000.0)]
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0, spread_tol_um=float("inf"))
        vals = sorted(r.per_rung_zref_mm)
        self.assertEqual(len(vals), 4)
        median = 0.5 * (vals[1] + vals[2])
        mean = sum(vals) / 4.0
        self.assertLess(abs(median - B_TRUE) * 1000.0, 1.0)
        self.assertGreater(abs(mean - B_TRUE) * 1000.0, 200.0)

    def test_a_refused_rung_is_excluded_not_counted(self):
        rungs = [rung(1000.0), rung(500.0),
                 RungMeasurement(margin_um=100.0, needle_z_zref_mm=0.0,
                                 focus_tip_um=0.0, refusal="LOW_PROMINENCE")]
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0)
        self.assertEqual(r.n_used, 2)
        self.assertTrue(r.ok, r.refusal)

    def test_no_usable_rung_refuses_rather_than_returning_a_number(self):
        r = reconcile_rungs(
            [RungMeasurement(margin_um=100.0, needle_z_zref_mm=0.0,
                             focus_tip_um=0.0, refusal="MONOTONIC")],
            focus_zero_um=F0, focus_up_sign=1.0, zdir=-1.0)
        self.assertIsNone(r.plate_bottom_zref_mm)
        self.assertFalse(r.ok)

    def test_a_common_mode_f0_error_is_invisible_here_by_construction(self):
        """Documents the limitation the glass-focus check exists to cover: a
        wrong f0 shifts every rung EQUALLY, so the agreement check passes and
        the plate bottom is wrong by exactly that shift."""
        r = reconcile_rungs([rung(x) for x in DEFAULT_OFFSETS_UM],
                            focus_zero_um=F0 - 50.0,   # operator misfocused
                            focus_up_sign=1.0, zdir=-1.0)
        self.assertTrue(r.ok)                     # ← agrees perfectly...
        self.assertAlmostEqual(r.spread_um, 0.0, places=6)
        self.assertAlmostEqual(                   # ← ...and is 50 µm wrong
            (r.plate_bottom_zref_mm - B_TRUE) * 1000.0, 50.0, places=3)


class TestScale(unittest.TestCase):
    """Both axes measure one rigid tip, so the ratio is 1 by physics."""

    def test_clean_data_measures_exactly_one(self):
        s = measured_scale([rung(x) for x in DEFAULT_OFFSETS_UM],
                           focus_zero_um=F0, focus_up_sign=1.0, zdir=-1.0)
        self.assertAlmostEqual(s, 1.0, places=9)

    def test_it_is_one_on_every_polarity_combination(self):
        for zdir in (-1.0, 1.0):
            for fus in (1.0, -1.0):
                rungs = [rung(x, zdir=zdir, fus=fus)
                         for x in DEFAULT_OFFSETS_UM]
                s = measured_scale(rungs, focus_zero_um=F0,
                                   focus_up_sign=fus, zdir=zdir)
                self.assertAlmostEqual(s, 1.0, places=9, msg=f"{zdir} {fus}")

    def test_a_units_slip_is_caught_as_a_bad_scale(self):
        # Needle Z moves 2 % further than the focal plane says it should.
        rungs = []
        for x in DEFAULT_OFFSETS_UM:
            rungs.append(RungMeasurement(
                margin_um=x,
                needle_z_zref_mm=needle_target_zref(B_TRUE, x * 1.02, -1.0),
                focus_tip_um=F0 + x, focus_sigma_um=1.7))
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0, spread_tol_um=float("inf"))
        self.assertAlmostEqual(r.scale, 1.02, places=6)
        self.assertFalse(r.scale_ok)
        self.assertIn("bug", r.refusal.lower())

    def test_one_rung_yields_no_slope_and_says_the_scale_is_assumed(self):
        r = reconcile_rungs([rung(1000.0)], focus_zero_um=F0,
                            focus_up_sign=1.0, zdir=-1.0)
        self.assertIsNone(r.scale)
        self.assertIn("assumed", r.scale_note)
        self.assertTrue(r.ok)                     # still a usable bottom
        self.assertEqual(MIN_RUNGS_FOR_SLOPE, 2)

    def test_a_degenerate_span_returns_none_not_a_huge_number(self):
        same = [rung(500.0), rung(500.0)]
        self.assertIsNone(measured_scale(same, focus_zero_um=F0,
                                         focus_up_sign=1.0, zdir=-1.0))


class TestScaleVerifiability(unittest.TestCase):
    """A gate that could not have failed must not report as passed."""

    def test_ten_x_can_verify_and_four_x_cannot(self):
        # sigma = DOF/6: 10x DOF 10.4 µm ⇒ 1.7; 4x DOF 57 µm ⇒ 9.5.
        self.assertTrue(scale_verifiable(1.7, 900.0, 4))
        self.assertFalse(scale_verifiable(9.5, 900.0, 4))

    def test_the_precision_formula_matches_the_documented_figures(self):
        self.assertAlmostEqual(slope_precision_frac(1.7, 900.0, 4),
                               0.0033, places=4)
        self.assertAlmostEqual(slope_precision_frac(9.5, 900.0, 4),
                               0.0183, places=4)

    def test_a_wider_ladder_rescues_four_x(self):
        self.assertFalse(scale_verifiable(9.5, 900.0, 4))
        self.assertTrue(scale_verifiable(9.5, 5000.0, 4))

    def test_a_zero_span_has_no_precision_rather_than_perfect_precision(self):
        self.assertEqual(slope_precision_frac(1.7, 0.0, 4), float("inf"))
        self.assertFalse(scale_verifiable(1.7, 0.0, 4))

    def test_an_unverifiable_ladder_says_so_instead_of_claiming_pass(self):
        rungs = [rung(x, sigma=9.5) for x in DEFAULT_OFFSETS_UM]
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0)
        self.assertTrue(r.ok)                     # the bottom is still good
        self.assertFalse(r.scale_verified)
        self.assertIn("UNVERIFIED", r.scale_note)
        self.assertIn("higher-power", r.scale_note)

    def test_an_unverifiable_bad_scale_does_not_block_accept(self):
        """With 4x precision the 1 % band is meaningless, so a 1.5 % reading is
        not evidence of a fault and must not be reported as one."""
        rungs = []
        for x in DEFAULT_OFFSETS_UM:
            rungs.append(RungMeasurement(
                margin_um=x,
                needle_z_zref_mm=needle_target_zref(B_TRUE, x, -1.0),
                focus_tip_um=F0 + x * 1.015, focus_sigma_um=9.5))
        r = reconcile_rungs(rungs, focus_zero_um=F0, focus_up_sign=1.0,
                            zdir=-1.0, spread_tol_um=float("inf"))
        self.assertFalse(r.scale_ok)
        self.assertFalse(r.scale_verified)
        self.assertEqual(r.refusal, "")           # not blocked


class TestTheStoreContract(unittest.TestCase):
    """``needle_z_zref = focal_sign * (focus_um/1000) + offset_mm``."""

    def test_the_datum_reproduces_the_descent_target(self):
        for zdir in (-1.0, 1.0):
            for fus in (1.0, -1.0):
                sign, off = focal_sign_and_offset(B_TRUE, F0, fus, zdir)
                self.assertIn(sign, (1.0, -1.0))
                # At the glass focus the needle height that reaches it is the
                # plate bottom itself.
                self.assertAlmostEqual(sign * (F0 / 1000.0) + off, B_TRUE,
                                       places=9)
                # And 1 mm up the focus axis is 1 mm up the needle axis.
                f = F0 + 1000.0 * fus
                self.assertAlmostEqual(
                    sign * (f / 1000.0) + off,
                    needle_target_zref(B_TRUE, 1000.0, zdir), places=9)

    def test_the_scale_band_is_the_stores_own_constant(self):
        """Not a second copy — one of them would be retuned and drift."""
        import SupportClasses.PlateBottomOptical as pbo
        self.assertIs(pbo.SCALE_TOLERANCE, SCALE_TOLERANCE)


class TestDefaults(unittest.TestCase):
    def test_defaults_are_largest_first_and_span_enough_to_verify_at_10x(self):
        self.assertEqual(list(DEFAULT_OFFSETS_UM),
                         sorted(DEFAULT_OFFSETS_UM, reverse=True))
        span = max(DEFAULT_OFFSETS_UM) - min(DEFAULT_OFFSETS_UM)
        self.assertTrue(scale_verifiable(1.7, span, len(DEFAULT_OFFSETS_UM)))

    def test_the_spread_tolerance_is_tighter_than_the_plane_residual(self):
        from SupportClasses.PlateZPlane import DEFAULT_RESIDUAL_TOL_MM
        self.assertLess(DEFAULT_SPREAD_TOL_UM, DEFAULT_RESIDUAL_TOL_MM * 1000.0)


if __name__ == "__main__":
    unittest.main()
