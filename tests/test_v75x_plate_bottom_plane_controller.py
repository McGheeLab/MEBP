"""
v7.5.x — StageController plate-bottom PLANE: trust rules and degradation.

The plate bottom is flat but tilted, so plate-bottom Z is a function of (x, y).
This suite pins the two things that make consuming that safe:

  1. DEGRADE-TO-SCALAR IDENTITY. With no plane, a rejected plane, the tilt
     switched off, or no XY supplied, every accessor returns exactly the taught
     scalar — so an install that has not run the new calibration sees
     bit-identical Z. This is the guarantee that lets the feature land.
  2. TRUST RULES. A plane is only used when it validates. Each rule is pinned
     individually, including the one taken from real data: this machine's saved
     calibration holds a plate-local plane with c = 5.600 while its taught
     plate_bottom_z is 0.090, and adopting that intercept would have driven the
     needle 5.5 mm deeper — into the glass.

Controllers are built with ``__new__`` and only the attributes under test, the
same partial-stub pattern ``test_v75x_print_z_plate_bottom.py`` uses — so every
new attribute access must be getattr-guarded.
"""

import unittest

from SupportClasses.PlateZPlane import (
    PlateZPlane,
    ZPlanePoint,
    from_needle_touches,
    job_plane_z_zref_mm,
    plate_plane_z_zref_mm,
)
from SupportClasses.StageController import StageController


A1 = (105000.0, 68000.0)
A6 = (60000.0, 68000.0)
D1 = (105000.0, 10000.0)
D6 = (60000.0, 10000.0)
BBOX = (60000.0, 10000.0, 105000.0, 68000.0)

SCALAR = 0.09          # the taught plate bottom at A1 (this machine's value)


def _ctrl(*, plate_bottom=SCALAR, plate_top=None, zero_z=0.0,
          zero_xy=(0.0, 0.0), flip=None):
    """A partial controller carrying only the plate-Z state."""
    c = StageController.__new__(StageController)
    c._plate_bottom_z_zref = plate_bottom
    c._plate_top_z_zref = plate_top
    c._print_floor_active = False
    c.zero_position = {"Z": zero_z, "x": zero_xy[0], "y": zero_xy[1]}
    if flip is not None:
        c._plate_flip_180 = flip
    return c


def _plane(*, sx=0.0005, sy=-0.001, z0=SCALAR, n=4, zero_z=0.0,
           flip=None, plate_key=None):
    """A validated-shape plane anchored at A1 with a known tilt."""
    def z_of(xy):
        return z0 + sx * (xy[0] - A1[0]) / 1000.0 + sy * (xy[1] - A1[1]) / 1000.0

    pts = [ZPlanePoint(label="A1", x_stage_um=A1[0], y_stage_um=A1[1], z_zref_mm=z0),
           ZPlanePoint(label="D1", x_stage_um=D1[0], y_stage_um=D1[1],
                       z_zref_mm=z_of(D1)),
           ZPlanePoint(label="A6", x_stage_um=A6[0], y_stage_um=A6[1],
                       z_zref_mm=z_of(A6))]
    if n >= 4:
        pts.append(ZPlanePoint(label="D6", x_stage_um=D6[0], y_stage_um=D6[1],
                               z_zref_mm=z_of(D6)))
    prov = {"zero_z_mm_at_fit": zero_z, "fitted_at": "2026-07-29T12:00:00"}
    if flip is not None:
        prov["plate_flip_180"] = flip
    if plate_key is not None:
        prov["plate_key"] = plate_key
    plane, why = from_needle_touches(points=pts, anchor_label="A1",
                                    provenance=prov)
    assert plane is not None, why
    return plane


def _install(c, plane=None, *, enabled=True, bbox=BBOX):
    c.set_plate_footprint_bbox_um(bbox)
    ok, why = (True, "") if plane is None else c.set_plate_z_plane(plane)
    c.set_plate_tilt_enabled(enabled)
    return ok, why


# ── 1. degradation ────────────────────────────────────────────────────────

class TestDegradesToScalar(unittest.TestCase):
    """The whole feature must be invisible until a plane is measured+enabled."""

    GRID = [(x, y) for x in (60000.0, 82500.0, 105000.0)
            for y in (10000.0, 39000.0, 68000.0)]

    def test_no_plane_at_all(self):
        c = _ctrl()
        for xy in self.GRID:
            self.assertEqual(c.plate_bottom_z_at_um(*xy), SCALAR)
        self.assertIsNone(c.get_plate_z_plane())
        self.assertIsNone(c.active_plate_z_plane())
        self.assertFalse(c.plate_tilt_enabled())

    def test_plane_present_but_tilt_disabled(self):
        c = _ctrl()
        _install(c, _plane(), enabled=False)
        self.assertIsNotNone(c.active_plate_z_plane())
        self.assertFalse(c.plate_tilt_enabled())
        for xy in self.GRID:
            self.assertEqual(c.plate_bottom_z_at_um(*xy), SCALAR)

    def test_rejected_plane_is_equivalent_to_none(self):
        c = _ctrl(zero_z=0.0)
        ok, _ = _install(c, _plane(zero_z=9.0))       # epoch mismatch ⇒ rejected
        self.assertFalse(ok)
        self.assertIsNone(c.active_plate_z_plane())
        for xy in self.GRID:
            self.assertEqual(c.plate_bottom_z_at_um(*xy), SCALAR)

    def test_no_xy_supplied(self):
        c = _ctrl()
        _install(c, _plane())
        self.assertEqual(c.plate_bottom_z_at_um(None, None), SCALAR)
        self.assertEqual(c.plate_bottom_z_at_zref_mm(None, None), SCALAR)

    def test_outside_the_taught_region_falls_back(self):
        c = _ctrl()
        _install(c, _plane())
        # Well inside → tilted; far outside → scalar, never extrapolated.
        self.assertNotAlmostEqual(c.plate_bottom_z_at_um(*D6), SCALAR, places=4)
        self.assertEqual(c.plate_bottom_z_at_um(0.0, 0.0), SCALAR)

    def test_uncalibrated_scalar_stays_none(self):
        c = _ctrl(plate_bottom=None)
        _install(c, _plane())
        self.assertIsNone(c.plate_bottom_z_at_um(*D6))
        self.assertIsNone(c.print_height_to_zref(0.2))
        self.assertIsNone(c.print_height_to_zref(0.2, 0.0, 0.0))

    def test_print_height_to_zref_identity_without_xy(self):
        """The exact back-compat contract: no XY ⇒ the pre-existing number."""
        c = _ctrl()
        _install(c, _plane())
        for h in (0.0, 0.1, 0.2, 5.0, -0.3):
            self.assertEqual(c.print_height_to_zref(h),
                             c.print_height_to_zref(h, None, None))

    def test_zref_to_print_height_identity_without_xy(self):
        c = _ctrl()
        _install(c, _plane())
        for z in (0.0, 0.09, -1.0, 3.5):
            self.assertEqual(c.zref_to_print_height(z),
                             c.zref_to_print_height(z, None, None))

    def test_print_floor_violation_identity_without_xy(self):
        c = _ctrl()
        _install(c, _plane())
        for z in (-1.0, 0.0, 0.09, 1.0):
            self.assertEqual(c.print_floor_violation(z),
                             c.print_floor_violation(z, None, None))

    def test_level_plane_changes_nothing(self):
        """A measured-but-flat plate must be indistinguishable from the scalar."""
        c = _ctrl()
        _install(c, _plane(sx=0.0, sy=0.0))
        for xy in self.GRID:
            self.assertAlmostEqual(c.plate_bottom_z_at_um(*xy), SCALAR,
                                   places=12)

    def test_partial_stub_without_new_attributes_survives(self):
        """A __new__ stub predating this change must not explode."""
        c = StageController.__new__(StageController)
        c._plate_bottom_z_zref = SCALAR
        c._plate_top_z_zref = None
        c._print_floor_active = False
        c.zero_position = {"Z": 0.0}
        self.assertIsNone(c.get_plate_z_plane())
        self.assertIsNone(c.active_plate_z_plane())
        self.assertFalse(c.plate_tilt_enabled())
        self.assertIsNone(c.get_plate_bottom_anchor_xy_um())
        self.assertIsNone(c.get_plate_footprint_bbox_um())
        self.assertIsNone(c.plate_z_tilt_span_mm())
        self.assertIsNone(c.plate_z_plane_for_job())
        self.assertEqual(c.plate_bottom_z_at_um(*D6), SCALAR)
        self.assertEqual(c.print_height_to_zref(0.2),
                         c.print_height_to_zref(0.2, None, None))


# ── 2. trust rules ────────────────────────────────────────────────────────

class TestTrustRules(unittest.TestCase):
    def test_valid_plane_is_accepted_and_used(self):
        c = _ctrl()
        ok, why = _install(c, _plane())
        self.assertTrue(ok, why)
        self.assertEqual(c.get_plate_z_plane().status, "active")
        self.assertTrue(c.plate_tilt_enabled())
        # A1 is the anchor ⇒ exactly the taught scalar.
        self.assertAlmostEqual(c.plate_bottom_z_at_um(*A1), SCALAR, places=12)
        # D6 is 45 mm -X and 58 mm -Y away ⇒ tilted by a real amount.
        expect = SCALAR + 0.0005 * -45.0 + (-0.001) * -58.0
        self.assertAlmostEqual(c.plate_bottom_z_at_um(*D6), expect, places=9)

    def test_legacy_plate_local_plane_does_not_move_z_by_5mm(self):
        """THE regression, from this machine's own settings.json.

        Saved plate-local plane c = 5.600 vs taught plate_bottom_z = 0.090. On
        ZDIR=-1 a larger zero-ref Z is DEEPER, so adopting that intercept would
        drive the needle 5.5 mm into the glass.
        """
        c = _ctrl()
        legacy = {"a": -0.000725, "b": -0.019862, "c": 5.600, "r_squared": 1.0}
        ok, why = c.set_plate_z_plane(legacy)
        self.assertFalse(ok)
        self.assertIn("stage frame", why)
        self.assertIsNone(c.active_plate_z_plane())
        c.set_plate_tilt_enabled(True)
        for xy in (A1, A6, D1, D6, (0.0, 0.0)):
            self.assertEqual(c.plate_bottom_z_at_um(*xy), SCALAR)

    def test_epoch_mismatch_rejected(self):
        c = _ctrl(zero_z=0.0)
        ok, why = c.set_plate_z_plane(_plane(zero_z=1.5))
        self.assertFalse(ok)
        self.assertIn("needle zero changed", why)

    def test_epoch_match_within_tolerance_accepted(self):
        c = _ctrl(zero_z=1.5)
        ok, why = c.set_plate_z_plane(_plane(zero_z=1.505))
        self.assertTrue(ok, why)

    def test_orientation_mismatch_rejected(self):
        c = _ctrl(flip=True)
        ok, why = c.set_plate_z_plane(_plane(flip=False))
        self.assertFalse(ok)
        self.assertIn("orientation", why)

    def test_orientation_match_accepted(self):
        c = _ctrl(flip=True)
        ok, why = c.set_plate_z_plane(_plane(flip=True))
        self.assertTrue(ok, why)

    def test_anchor_disagreement_rejected(self):
        """Catches a manual re-teach of the scalar after the plane was fitted."""
        c = _ctrl(plate_bottom=SCALAR)
        ok, why = c.set_plate_z_plane(_plane(z0=SCALAR + 0.4))
        self.assertFalse(ok)
        self.assertIn("disagrees", why)

    def test_no_scalar_means_no_anchor_to_check(self):
        c = _ctrl(plate_bottom=None)
        ok, why = c.set_plate_z_plane(_plane())
        self.assertFalse(ok)
        self.assertIn("no taught plate-bottom", why)

    def test_residual_over_tolerance_rejected(self):
        c = _ctrl()
        bad = _plane()
        from dataclasses import replace
        ok, why = c.set_plate_z_plane(replace(bad, residual_max_mm=0.2))
        self.assertFalse(ok)
        self.assertIn("residual", why)

    def test_holdout_over_tolerance_rejected(self):
        c = _ctrl()
        from dataclasses import replace
        ok, why = c.set_plate_z_plane(replace(_plane(), holdout_error_mm=0.3))
        self.assertFalse(ok)
        self.assertIn("hold-out", why)

    def test_three_points_accepted_but_no_holdout(self):
        """R² is identically 1 at n=3, so it is allowed yet unverified."""
        c = _ctrl()
        p = _plane(n=3)
        self.assertIsNone(p.holdout_error_mm)
        ok, why = c.set_plate_z_plane(p)
        self.assertTrue(ok, why)

    def test_degenerate_plane_rejected(self):
        c = _ctrl()
        from dataclasses import replace
        ok, why = c.set_plate_z_plane(replace(_plane(), degenerate=True))
        self.assertFalse(ok)
        self.assertIn("underdetermined", why)

    def test_too_few_points_rejected(self):
        c = _ctrl()
        from dataclasses import replace
        ok, why = c.set_plate_z_plane(replace(_plane(), num_points=2))
        self.assertFalse(ok)
        self.assertIn("need ≥3", why)

    def test_implausible_tilt_rejected(self):
        c = _ctrl()
        from dataclasses import replace
        ok, why = c.set_plate_z_plane(replace(_plane(), sx_mm_per_mm=0.5))
        self.assertFalse(ok)
        self.assertIn("implausible", why)

    def test_setting_none_clears(self):
        c = _ctrl()
        _install(c, _plane())
        ok, why = c.set_plate_z_plane(None)
        self.assertTrue(ok, why)
        self.assertIsNone(c.get_plate_z_plane())
        self.assertEqual(c.plate_bottom_z_at_um(*D6), SCALAR)

    def test_rejected_plane_is_retained_for_the_ui(self):
        c = _ctrl(zero_z=0.0)
        c.set_plate_z_plane(_plane(zero_z=9.0))
        stored = c.get_plate_z_plane()
        self.assertIsNotNone(stored)                  # explainable in the UI
        self.assertEqual(stored.status, "rejected")
        self.assertTrue(stored.reject_reason)
        self.assertIsNone(c.active_plate_z_plane())   # but never used


# ── 3. reporting + stamping ───────────────────────────────────────────────

class TestReportingAndStamping(unittest.TestCase):
    def test_extremes_use_the_height_frame(self):
        """Shallowest = physically highest, on BOTH polarities."""
        # print_z_dir() derives from top-vs-bottom; make it -1 (ME3B V1).
        c = _ctrl(plate_bottom=1.0, plate_top=0.0)
        self.assertEqual(c.print_z_dir(), -1.0)
        _install(c, _plane(z0=1.0, sx=0.0, sy=0.001))
        shallow, deep = c.plate_bottom_z_extremes_zref()
        # zref larger = deeper here, so shallowest is the SMALLER number.
        self.assertLess(shallow, deep)

        c2 = _ctrl(plate_bottom=1.0, plate_top=2.0)
        self.assertEqual(c2.print_z_dir(), 1.0)
        _install(c2, _plane(z0=1.0, sx=0.0, sy=0.001))
        shallow2, deep2 = c2.plate_bottom_z_extremes_zref()
        self.assertGreater(shallow2, deep2)

    def test_extremes_without_plane_is_the_scalar_twice(self):
        c = _ctrl()
        self.assertEqual(c.plate_bottom_z_extremes_zref(), (SCALAR, SCALAR))

    def test_tilt_span_reports_the_real_magnitude(self):
        c = _ctrl()
        _install(c, _plane(sx=0.0, sy=-0.0199))
        span = c.plate_z_tilt_span_mm()
        # -0.0199 mm/mm over the 58 mm Y span ≈ 1.15 mm — the measured tilt.
        self.assertAlmostEqual(span, 0.0199 * 58.0, places=6)

    def test_tilt_span_none_without_plane(self):
        self.assertIsNone(_ctrl().plate_z_tilt_span_mm())

    def test_job_stamp_is_none_unless_enabled_and_valid(self):
        c = _ctrl()
        _install(c, _plane(), enabled=False)
        self.assertIsNone(c.plate_z_plane_for_job())
        c.set_plate_tilt_enabled(True)
        self.assertIsNotNone(c.plate_z_plane_for_job())

    def test_job_stamp_round_trips_through_the_job_evaluator(self):
        """The stamped mm form must agree with the live µm form."""
        zx, zy = 5000.0, 7000.0
        c = _ctrl(zero_xy=(zx, zy))
        _install(c, _plane())
        job = c.plate_z_plane_for_job()
        for xy in (A1, A6, D1, D6, (82500.0, 39000.0)):
            live = c.plate_bottom_z_at_um(*xy)
            stamped = job_plane_z_zref_mm(job, (xy[0] - zx) / 1000.0,
                                          (xy[1] - zy) / 1000.0)
            self.assertAlmostEqual(live, stamped, places=9)

    def test_job_form_and_um_form_are_not_interchangeable(self):
        """Mixing the two plane forms must raise, never silently mis-scale."""
        c = _ctrl()
        _install(c, _plane())
        job = c.plate_z_plane_for_job()
        with self.assertRaises(KeyError):
            plate_plane_z_zref_mm(job, 0.0, 0.0)
        with self.assertRaises(KeyError):
            job_plane_z_zref_mm(c.active_plate_z_plane().to_dict(), 0.0, 0.0)

    def test_zref_mm_accessor_honours_the_xy_zero(self):
        zx, zy = 5000.0, 7000.0
        c = _ctrl(zero_xy=(zx, zy))
        _install(c, _plane())
        self.assertAlmostEqual(
            c.plate_bottom_z_at_zref_mm((A1[0] - zx) / 1000.0,
                                        (A1[1] - zy) / 1000.0),
            SCALAR, places=12)

    def test_print_height_to_zref_with_xy_tracks_the_tilt(self):
        c = _ctrl(plate_bottom=SCALAR, plate_top=SCALAR - 10.0)   # zdir = -1
        _install(c, _plane())
        h = 0.2
        at_anchor = c.print_height_to_zref(h, A1[0] / 1000.0, A1[1] / 1000.0)
        self.assertAlmostEqual(at_anchor, c.print_height_to_zref(h), places=12)
        at_d6 = c.print_height_to_zref(h, D6[0] / 1000.0, D6[1] / 1000.0)
        self.assertNotAlmostEqual(at_d6, at_anchor, places=4)
        # The height above the (local) plate bottom is preserved everywhere.
        self.assertAlmostEqual(
            c.zref_to_print_height(at_d6, D6[0] / 1000.0, D6[1] / 1000.0),
            h, places=9)


class TestAnchorProvenance(unittest.TestCase):
    def test_anchor_xy_and_source_recorded(self):
        c = _ctrl(plate_bottom=None)
        c.set_plate_bottom_z(0.09, at_xy_um=A1, source="taught")
        self.assertEqual(c.get_plate_bottom_z(), 0.09)
        self.assertEqual(c.get_plate_bottom_anchor_xy_um(), A1)
        self.assertEqual(c.get_plate_bottom_z_source(), "taught")

    def test_legacy_single_arg_call_still_works(self):
        c = _ctrl(plate_bottom=None)
        c.set_plate_bottom_z(1.25)
        self.assertEqual(c.get_plate_bottom_z(), 1.25)
        self.assertIsNone(c.get_plate_bottom_anchor_xy_um())
        self.assertIsNone(c.get_plate_bottom_z_source())

    def test_bad_anchor_xy_is_ignored_not_fatal(self):
        c = _ctrl(plate_bottom=None)
        c.set_plate_bottom_z(1.0, at_xy_um=("x",))
        self.assertIsNone(c.get_plate_bottom_anchor_xy_um())

    def test_footprint_falls_back_to_the_planes_own_points(self):
        c = _ctrl()
        c.set_plate_z_plane(_plane())
        c.set_plate_footprint_bbox_um(None)
        self.assertEqual(c.get_plate_footprint_bbox_um(), BBOX)


if __name__ == "__main__":
    unittest.main()
