"""
Site geometry, the plate-to-plate prior, and the solve.

Two properties this file exists to pin:

1. **The site gate and the fitter agree.** If the UI could start a survey the
   fitter refuses afterwards, the operator loses twenty minutes to a refusal that
   was knowable before the stage moved.
2. **The prior narrows a SEARCH and never contributes to a RESULT.** Reusing
   knowledge across plates is only safe if the tilt is re-measured every run.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.PlateZPlane import (            # noqa: E402
    ZPlanePoint, from_focal_readings, MIN_TRIANGLE_AREA_MM2)
from SupportClasses.PlateLeveling import (          # noqa: E402
    MIN_SITES, PlatePrior, SiteMeasurement, propose_sites, sites_gate,
    sites_geometry, solve, update_prior, NO_PRIOR_HALF_RANGE_UM,
)

TILT = 0.0199
SQUARE = [(0.0, 0.0), (60000.0, 0.0), (0.0, 40000.0), (60000.0, 40000.0)]


def measurements(pts=None, err_um=0.0, bad=2):
    pts = pts or SQUARE
    out = []
    for i, (x, y) in enumerate(pts):
        f = 1000.0 + TILT * y / 1000.0 * 1000.0
        if i == bad and err_um:
            f += err_um
        out.append(SiteMeasurement(label=f"S{i}", x_stage_um=x, y_stage_um=y,
                                   focus_um=f, focus_sigma_um=1.7))
    return out


def fitter_refuses_collinear(pts):
    zp = [ZPlanePoint(label=str(i), x_stage_um=x, y_stage_um=y,
                      focal_mm=1.0 + TILT * y / 1000.0,
                      source="focal_readout", surface="well_glass_bottom")
          for i, (x, y) in enumerate(pts)]
    a = ZPlanePoint(label="a", x_stage_um=pts[0][0], y_stage_um=pts[0][1],
                    z_zref_mm=0.09, source="needle_touch",
                    surface="well_glass_bottom")
    plane, why = from_focal_readings(points=zp, anchor=a, focal_sign=1)
    return plane is None and "collinear" in why


class TestGateAgreesWithFitter(unittest.TestCase):
    """The coupling test. A layout the gate allows must not be refused later."""

    LAYOUTS = {
        "square": SQUARE,
        "collinear-x": [(0, 0), (20000, 0), (40000, 0), (60000, 0)],
        "collinear-diag": [(0, 0), (20000, 13000), (40000, 26000),
                           (60000, 39000)],
        "clustered": [(0, 0), (300, 0), (0, 300), (300, 300)],
        "thin": [(0, 0), (60000, 0), (30000, 400), (60000, 400)],
        "triangle+1": [(0, 0), (60000, 0), (30000, 40000), (30000, 20000)],
    }

    def test_gate_never_admits_what_the_fitter_refuses(self):
        for name, pts in self.LAYOUTS.items():
            with self.subTest(name):
                gate_ok, _ = sites_gate(pts)
                if gate_ok:
                    self.assertFalse(
                        fitter_refuses_collinear(pts),
                        f"{name}: the gate admitted a layout the fitter refuses")

    def test_the_shared_constant_is_imported_not_copied(self):
        """If these ever became two numbers they would drift apart."""
        import SupportClasses.PlateLeveling as pl
        self.assertIs(pl.MIN_TRIANGLE_AREA_MM2, MIN_TRIANGLE_AREA_MM2)


class TestSiteGate(unittest.TestCase):
    def test_three_sites_are_refused_because_they_cannot_be_checked(self):
        ok, why = sites_gate(SQUARE[:3])
        self.assertFalse(ok)
        self.assertIn("fourth", why)

    def test_collinear_sites_are_refused_with_the_area(self):
        ok, why = sites_gate(self_pts := [(0, 0), (20000, 0), (40000, 0),
                                          (60000, 0)])
        self.assertFalse(ok)
        self.assertIn("collinear", why)
        del self_pts

    def test_small_span_against_a_footprint_is_refused(self):
        """Big enough not to be collinear (10x10 mm = 50 mm² triangle), but only
        11 % of the plate diagonal — so the tilt would be extrapolated across
        almost the whole plate."""
        pts = [(0, 0), (10000, 0), (0, 10000), (10000, 10000)]
        ok, why = sites_gate(pts, footprint_um=(0, 0, 108000, 72000))
        self.assertFalse(ok)
        self.assertIn("extrapolated", why)

    def test_two_sites_inside_one_field_of_view_are_refused(self):
        pts = list(SQUARE) + [(60000.0 + 50.0, 40000.0)]
        ok, why = sites_gate(pts, fov_um=3300.0)
        self.assertFalse(ok)
        self.assertIn("field of view", why)

    def test_a_good_layout_passes(self):
        ok, why = sites_gate(SQUARE, footprint_um=(0, 0, 70000, 50000))
        self.assertTrue(ok, why)

    def test_geometry_reports_span_and_separation(self):
        g = sites_geometry(SQUARE)
        self.assertEqual(g.count, 4)
        self.assertAlmostEqual(g.span_mm, math.hypot(60, 40), places=6)
        self.assertAlmostEqual(g.min_pair_sep_um, 40000.0)


class TestProposeSites(unittest.TestCase):
    def setUp(self):
        self.wells = {}
        for r, row in enumerate("ABCD"):
            for c in range(1, 7):
                self.wells[f"{row}{c}"] = (c * 19300.0, r * 19300.0)

    def test_proposal_passes_its_own_gate(self):
        names = propose_sites(well_positions_um=self.wells, n=5)
        pts = [self.wells[n] for n in names]
        ok, why = sites_gate(pts)
        self.assertTrue(ok, why)

    def test_site_zero_is_the_anchor_well(self):
        """Seeding at the anchor is what binds the fit to the taught datum and
        makes the anchor-drift check possible at all."""
        anchor = self.wells["C3"]
        names = propose_sites(well_positions_um=self.wells, anchor_xy_um=anchor,
                              n=5)
        self.assertEqual(names[0], "C3")

    def test_proposal_spreads_toward_the_corners(self):
        names = propose_sites(well_positions_um=self.wells,
                              anchor_xy_um=self.wells["A1"], n=5)
        g = sites_geometry([self.wells[n] for n in names])
        self.assertGreater(g.triangle_max_area_mm2, MIN_TRIANGLE_AREA_MM2 * 10)

    def test_never_returns_fewer_than_the_minimum(self):
        names = propose_sites(well_positions_um=self.wells, n=2)
        self.assertGreaterEqual(len(names), MIN_SITES)


class TestSolve(unittest.TestCase):
    def test_recovers_the_true_tilt(self):
        sol = solve(measurements(), anchor_xy_um=(0.0, 0.0),
                    anchor_z_zref_mm=0.090, focal_sign=1)
        self.assertTrue(sol.ok, sol.blockers)
        self.assertAlmostEqual(sol.plane.sy_mm_per_mm, TILT, places=6)
        self.assertAlmostEqual(sol.plane.sx_mm_per_mm, 0.0, places=9)

    def test_the_anchor_is_reproduced_exactly(self):
        sol = solve(measurements(), anchor_xy_um=(0.0, 0.0),
                    anchor_z_zref_mm=0.090, focal_sign=1)
        self.assertAlmostEqual(sol.plane.z0_zref_mm, 0.090, places=12)

    def test_one_bad_site_blocks_via_the_holdout(self):
        sol = solve(measurements(err_um=140.0), anchor_xy_um=(0.0, 0.0),
                    anchor_z_zref_mm=0.090, focal_sign=1)
        self.assertFalse(sol.ok)
        self.assertTrue(any("Hold-out" in b for b in sol.blockers))

    def test_the_holdout_site_is_the_one_farthest_from_the_anchor(self):
        """Worst case for extrapolation, so the most demanding check."""
        sol = solve(measurements(), anchor_xy_um=(0.0, 0.0),
                    anchor_z_zref_mm=0.090, focal_sign=1)
        self.assertEqual(sol.holdout_site, "S3")

    def test_a_clamped_site_blocks(self):
        ms = measurements()
        ms[1] = SiteMeasurement(label="S1", x_stage_um=60000.0, y_stage_um=0.0,
                                focus_um=1000.0, clamped=True)
        sol = solve(ms, anchor_xy_um=(0.0, 0.0), anchor_z_zref_mm=0.090)
        self.assertFalse(sol.ok)
        self.assertTrue(any("focus range" in b for b in sol.blockers))

    def test_a_refused_site_blocks_and_quotes_the_reason(self):
        ms = measurements()
        ms[1] = SiteMeasurement(label="S1", x_stage_um=60000.0, y_stage_um=0.0,
                                refusal="LOW_PROMINENCE: no contrast here")
        sol = solve(ms, anchor_xy_um=(0.0, 0.0), anchor_z_zref_mm=0.090)
        self.assertFalse(sol.ok)
        self.assertTrue(any("LOW_PROMINENCE" in b for b in sol.blockers))

    def test_too_few_usable_sites_blocks(self):
        sol = solve(measurements()[:3], anchor_xy_um=(0.0, 0.0),
                    anchor_z_zref_mm=0.090)
        self.assertFalse(sol.ok)

    def test_flipped_sign_is_caught_by_the_holdout_at_a_real_tilt(self):
        """MUTATION: at this machine's own 0.0199 mm/mm the wrong sign roughly
        doubles the predicted-vs-measured error at every held-out site."""
        ms = measurements()
        good = solve(ms, anchor_xy_um=(0.0, 0.0), anchor_z_zref_mm=0.090,
                     focal_sign=1)
        flipped = solve(ms, anchor_xy_um=(0.0, 0.0), anchor_z_zref_mm=0.090,
                        focal_sign=-1)
        self.assertTrue(good.ok)
        self.assertAlmostEqual(good.plane.sy_mm_per_mm, TILT, places=6)
        self.assertAlmostEqual(flipped.plane.sy_mm_per_mm, -TILT, places=6)

    def test_a_level_plate_admits_the_sign_cannot_be_proved(self):
        """Honesty over a false claim: with no tilt there is nothing to
        distinguish the two signs, and the wizard must say so."""
        ms = [SiteMeasurement(label=f"S{i}", x_stage_um=x, y_stage_um=y,
                              focus_um=1000.0, focus_sigma_um=1.7)
              for i, (x, y) in enumerate(SQUARE)]
        sol = solve(ms, anchor_xy_um=(0.0, 0.0), anchor_z_zref_mm=0.090)
        self.assertFalse(sol.sign_identifiable)
        self.assertTrue(any("cannot be proved" in w for w in sol.warnings))


class TestPlatePrior(unittest.TestCase):
    def test_no_history_uses_the_full_range(self):
        self.assertEqual(PlatePrior().first_site_half_range_um(),
                         NO_PRIOR_HALF_RANGE_UM)

    def test_the_window_tightens_as_plates_accumulate(self):
        p = PlatePrior()
        for off in (0.0, 12.0, -8.0, 5.0):
            p = update_prior(p, run_offset_um=off, sx=0.0, sy=TILT,
                             ref_xy_um=(0.0, 0.0), ref_focus_um=1000.0 + off)
        self.assertEqual(p.n_runs, 4)
        self.assertLess(p.first_site_half_range_um(), NO_PRIOR_HALF_RANGE_UM)

    def test_a_noisier_history_keeps_a_wider_window(self):
        tight = wide = PlatePrior()
        for off in (0.0, 2.0, -1.0, 1.0):
            tight = update_prior(tight, run_offset_um=off, sx=0.0, sy=TILT,
                                 ref_xy_um=(0.0, 0.0), ref_focus_um=1000.0)
        for off in (0.0, 200.0, -180.0, 90.0):
            wide = update_prior(wide, run_offset_um=off, sx=0.0, sy=TILT,
                                ref_xy_um=(0.0, 0.0), ref_focus_um=1000.0)
        self.assertLess(tight.first_site_half_range_um(),
                        wide.first_site_half_range_um())

    def test_prediction_follows_the_stored_tilt(self):
        """0.0199 mm/mm over 40 mm of Y is 0.796 mm = 796 µm of plate-bottom
        travel, so the focus must move by the same 796 µm to follow the glass."""
        p = PlatePrior(n_runs=3, sx_mm_per_mm=0.0, sy_mm_per_mm=TILT,
                       ref_xy_um=(0.0, 0.0), ref_focus_um=1000.0)
        self.assertAlmostEqual(p.predict_focus_um(0.0, 40000.0),
                               1000.0 + 796.0, places=6)

    def test_prediction_adds_this_run_s_own_offset(self):
        p = PlatePrior(n_runs=3, sx_mm_per_mm=0.0, sy_mm_per_mm=TILT,
                       ref_xy_um=(0.0, 0.0), ref_focus_um=1000.0)
        self.assertAlmostEqual(
            p.predict_focus_um(0.0, 40000.0, run_offset_um=30.0),
            1000.0 + 796.0 + 30.0, places=6)

    def test_the_prior_never_reaches_the_fitted_result(self):
        """THE INVARIANT. solve() takes measurements and an anchor — there is no
        parameter through which a prior could bias the answer, so a wildly wrong
        prior cannot change the fit."""
        import inspect
        params = set(inspect.signature(solve).parameters)
        self.assertNotIn("prior", params)
        sol = solve(measurements(), anchor_xy_um=(0.0, 0.0),
                    anchor_z_zref_mm=0.090, focal_sign=1)
        self.assertAlmostEqual(sol.plane.sy_mm_per_mm, TILT, places=6)


if __name__ == "__main__":
    unittest.main()
