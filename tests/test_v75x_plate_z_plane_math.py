"""
v7.5.x — plate-bottom Z plane: pure math, frames, and the safety properties.

The plane is an ANCHORED GRADIENT (see SupportClasses/PlateZPlane.py). These
tests pin the properties the rest of the feature leans on:

  * the anchor is exact by construction (so a fit can never move Z at the
    taught touch-off point — the guard against the real 5.5 mm plate-local
    intercept sitting in this machine's saved calibration),
  * a zero-tilt plane is indistinguishable from the legacy single scalar,
  * "shallowest/deepest" is decided in the ZDIR-scaled HEIGHT frame, so it is
    correct on ME3B V1 (z_up_sign = -1) as well as a conventional machine,
  * degeneracy (collinear points) and implausible tilt (a mm/µm slip) are
    REFUSED rather than silently adopted,
  * the frame tag is validated on load, so a legacy plate-local block can never
    be read as if it were in the stage frame.
"""

import unittest

from SupportClasses.PlateZPlane import (
    MIN_TRIANGLE_AREA_MM2,
    PLANE_FRAME,
    PlateZPlane,
    ZPlanePoint,
    fit_gradient_through_anchor,
    focal_to_mm,
    from_focal_readings,
    from_needle_touches,
    holdout_error_mm,
    level_plane_at,
    plate_plane_z_zref_mm,
    tilt_is_plausible,
    triangle_max_area_mm2,
)


def _pt(label, x_um, y_um, z=None, **kw):
    return ZPlanePoint(label=label, x_stage_um=x_um, y_stage_um=y_um,
                       z_zref_mm=z, **kw)


# A well-spread triangle + extras, in absolute stage µm (values in the same
# ballpark as this machine's real taught wells).
A1 = (105000.0, 68000.0)
A6 = (60000.0, 68000.0)
D1 = (105000.0, 10000.0)
D6 = (60000.0, 10000.0)
CTR = (82500.0, 39000.0)


class TestEvaluator(unittest.TestCase):
    def test_level_plane_returns_anchor_everywhere(self):
        """sx=sy=0 ⇒ the plane IS the scalar. This is the degrade path."""
        p = level_plane_at(x0_um=A1[0], y0_um=A1[1], z0_zref_mm=0.09)
        for xy in (A1, A6, D1, D6, CTR, (0.0, 0.0), (1e6, -1e6)):
            self.assertEqual(p.z_zref_mm_at_stage_um(*xy), 0.09)
        self.assertTrue(p.is_level())

    def test_anchor_is_exact(self):
        p = PlateZPlane(x0_um=A1[0], y0_um=A1[1], z0_zref_mm=1.234,
                        sx_mm_per_mm=0.001, sy_mm_per_mm=-0.002)
        self.assertAlmostEqual(p.z_zref_mm_at_stage_um(*A1), 1.234, places=12)
        self.assertAlmostEqual(p.delta_from_anchor_mm(*A1), 0.0, places=12)

    def test_gradient_sign_and_scale_x(self):
        """sx is mm of Z per mm of stage X; input is µm."""
        p = PlateZPlane(x0_um=0.0, y0_um=0.0, z0_zref_mm=0.0,
                        sx_mm_per_mm=0.01)
        # +58 mm in X = +58000 µm ⇒ +0.58 mm
        self.assertAlmostEqual(p.z_zref_mm_at_stage_um(58000.0, 0.0), 0.58)
        self.assertAlmostEqual(p.z_zref_mm_at_stage_um(-58000.0, 0.0), -0.58)

    def test_gradient_sign_and_scale_y(self):
        p = PlateZPlane(x0_um=0.0, y0_um=0.0, z0_zref_mm=0.0,
                        sy_mm_per_mm=-0.0199)
        # The real machine's measured b over a 58 mm row span ≈ -1.15 mm.
        self.assertAlmostEqual(p.z_zref_mm_at_stage_um(0.0, 58000.0),
                               -1.1542, places=4)

    def test_mm_and_um_accessors_agree(self):
        p = PlateZPlane(x0_um=1000.0, y0_um=2000.0, z0_zref_mm=0.5,
                        sx_mm_per_mm=0.002, sy_mm_per_mm=0.003)
        self.assertAlmostEqual(p.z_zref_mm_at_stage_um(51000.0, 62000.0),
                               p.z_zref_mm_at_stage_mm(51.0, 62.0), places=12)

    def test_unit_magnitude_contract(self):
        """Passing mm where µm is expected must NOT silently look plausible."""
        p = PlateZPlane(x0_um=105000.0, y0_um=68000.0, z0_zref_mm=0.09,
                        sx_mm_per_mm=0.001, sy_mm_per_mm=-0.002)
        self.assertNotAlmostEqual(p.z_zref_mm_at_stage_um(105000.0, 68000.0),
                                  p.z_zref_mm_at_stage_um(105.0, 68.0),
                                  places=3)

    def test_evaluator_accepts_dict_shaped_plane(self):
        """The job-stamped copy is a plain dict; one formula serves both."""
        d = {"x0_um": 0.0, "y0_um": 0.0, "z0_zref_mm": 1.0,
             "sx_mm_per_mm": 0.01, "sy_mm_per_mm": 0.0}
        self.assertAlmostEqual(plate_plane_z_zref_mm(d, 10000.0, 0.0), 1.1)


class TestExtremesPolarity(unittest.TestCase):
    """'Shallowest' must be decided in the HEIGHT frame, not on raw zero-ref."""

    BBOX = (60000.0, 10000.0, 105000.0, 68000.0)

    def _plane(self):
        # Z increases with +Y in the zero-ref frame.
        return PlateZPlane(x0_um=60000.0, y0_um=10000.0, z0_zref_mm=0.0,
                           sy_mm_per_mm=0.01)

    def test_conventional_machine(self):
        """z_up_sign=+1: larger zero-ref Z is higher ⇒ shallowest is the max."""
        p = self._plane()
        shallow, deep = p.extremes_zref(self.BBOX, z_up_sign=+1.0)
        self.assertAlmostEqual(shallow, 0.58)
        self.assertAlmostEqual(deep, 0.0)

    def test_me3b_v1_polarity_inverts_the_choice(self):
        """z_up_sign=-1 (ME3B V1): larger zero-ref Z is LOWER ⇒ flipped."""
        p = self._plane()
        shallow, deep = p.extremes_zref(self.BBOX, z_up_sign=-1.0)
        self.assertAlmostEqual(shallow, 0.0)
        self.assertAlmostEqual(deep, 0.58)

    def test_span_is_polarity_independent_and_non_negative(self):
        p = self._plane()
        self.assertAlmostEqual(p.span_mm(self.BBOX), 0.58)
        self.assertGreaterEqual(p.span_mm(self.BBOX), 0.0)


class TestFitThroughAnchor(unittest.TestCase):
    def test_exact_fit_recovers_a_known_tilt(self):
        sx, sy, z0 = 0.001, -0.002, 0.5

        def z_of(xy):
            return z0 + sx * (xy[0] - A1[0]) / 1000.0 + sy * (xy[1] - A1[1]) / 1000.0

        fit = fit_gradient_through_anchor(
            A1, z0, [(D1[0], D1[1], z_of(D1)), (A6[0], A6[1], z_of(A6))])
        self.assertIsNotNone(fit)
        self.assertFalse(fit.degenerate)
        self.assertAlmostEqual(fit.sx_mm_per_mm, sx, places=9)
        self.assertAlmostEqual(fit.sy_mm_per_mm, sy, places=9)
        self.assertAlmostEqual(fit.residual_max_mm, 0.0, places=9)

    def test_two_points_are_not_enough(self):
        self.assertIsNone(
            fit_gradient_through_anchor(A1, 0.0, [(D1[0], D1[1], 0.1)]))

    def test_collinear_with_anchor_is_degenerate(self):
        """All offsets along one direction ⇒ the other slope is unconstrained."""
        fit = fit_gradient_through_anchor(
            (0.0, 0.0), 0.0,
            [(10000.0, 0.0, 0.01), (20000.0, 0.0, 0.02), (30000.0, 0.0, 0.03)])
        self.assertIsNotNone(fit)
        self.assertTrue(fit.degenerate)

    def test_residuals_appear_with_four_points(self):
        pts = [(D1[0], D1[1], 0.0), (A6[0], A6[1], 0.0), (CTR[0], CTR[1], 0.4)]
        fit = fit_gradient_through_anchor(A1, 0.0, pts)
        self.assertIsNotNone(fit)
        self.assertGreater(fit.residual_max_mm, 0.0)


class TestFromNeedleTouches(unittest.TestCase):
    def _tilted(self, sx=0.0005, sy=-0.001, z0=0.09):
        def z_of(xy):
            return z0 + sx * (xy[0] - A1[0]) / 1000.0 + sy * (xy[1] - A1[1]) / 1000.0
        return [_pt("A1", *A1, z=z0), _pt("D1", *D1, z=z_of(D1)),
                _pt("A6", *A6, z=z_of(A6)), _pt("D6", *D6, z=z_of(D6))]

    def test_anchor_z_is_taken_verbatim(self):
        """The taught Z at the anchor survives the fit EXACTLY."""
        plane, why = from_needle_touches(points=self._tilted(),
                                         anchor_label="A1")
        self.assertIsNotNone(plane, why)
        self.assertEqual(plane.z0_zref_mm, 0.09)
        self.assertAlmostEqual(plane.z_zref_mm_at_stage_um(*A1), 0.09,
                               places=12)

    def test_recovers_the_tilt(self):
        plane, why = from_needle_touches(points=self._tilted(),
                                         anchor_label="A1")
        self.assertIsNotNone(plane, why)
        self.assertAlmostEqual(plane.sx_mm_per_mm, 0.0005, places=9)
        self.assertAlmostEqual(plane.sy_mm_per_mm, -0.001, places=9)
        self.assertEqual(plane.mode, "needle_touch")
        self.assertFalse(plane.degenerate)

    def test_anchor_defaults_to_first_point(self):
        pts = self._tilted()
        plane, _ = from_needle_touches(points=pts)
        self.assertEqual(plane.z0_zref_mm, pts[0].z_zref_mm)

    def test_refuses_fewer_than_three(self):
        plane, why = from_needle_touches(points=self._tilted()[:2])
        self.assertIsNone(plane)
        self.assertIn("≥3", why)

    def test_refuses_mixed_surfaces(self):
        pts = self._tilted()
        pts[1] = ZPlanePoint(label="D1", x_stage_um=D1[0], y_stage_um=D1[1],
                             z_zref_mm=0.2, surface="plate_top")
        plane, why = from_needle_touches(points=pts)
        self.assertIsNone(plane)
        self.assertIn("surface", why)

    def test_refuses_near_collinear(self):
        pts = [_pt("p1", 60000.0, 10000.0, z=0.0),
               _pt("p2", 80000.0, 10100.0, z=0.01),
               _pt("p3", 100000.0, 10200.0, z=0.02)]
        plane, why = from_needle_touches(points=pts)
        self.assertIsNone(plane)
        self.assertIn("collinear", why)

    def test_refuses_implausible_tilt_from_a_unit_slip(self):
        """A µm-vs-mm slip is a 1000× slope — it must never be adopted."""
        pts = [_pt("A1", *A1, z=0.0), _pt("D1", *D1, z=58.0),
               _pt("A6", *A6, z=0.0)]
        plane, why = from_needle_touches(points=pts)
        self.assertIsNone(plane)
        self.assertIn("implausible", why)

    def test_ignores_points_with_no_z(self):
        pts = self._tilted() + [_pt("X9", 70000.0, 30000.0, z=None)]
        plane, why = from_needle_touches(points=pts, anchor_label="A1")
        self.assertIsNotNone(plane, why)
        self.assertEqual(plane.num_points, 4)


class TestHoldout(unittest.TestCase):
    def test_none_below_four_points(self):
        pts = [_pt("A1", *A1, z=0.0), _pt("D1", *D1, z=0.1),
               _pt("A6", *A6, z=0.2)]
        self.assertIsNone(holdout_error_mm(pts))

    def test_zero_on_a_perfect_plane(self):
        sx, sy = 0.0005, -0.001

        def z_of(xy):
            return sx * (xy[0] - A1[0]) / 1000.0 + sy * (xy[1] - A1[1]) / 1000.0

        pts = [_pt("A1", *A1, z=0.0), _pt("D1", *D1, z=z_of(D1)),
               _pt("A6", *A6, z=z_of(A6)), _pt("D6", *D6, z=z_of(D6)),
               _pt("CTR", *CTR, z=z_of(CTR))]
        err = holdout_error_mm(pts)
        self.assertIsNotNone(err)
        self.assertLess(err, 1e-9)

    def test_catches_one_bad_touch_off(self):
        """The case R² cannot see: a single wrong point among four."""
        sx, sy = 0.0005, -0.001

        def z_of(xy):
            return sx * (xy[0] - A1[0]) / 1000.0 + sy * (xy[1] - A1[1]) / 1000.0

        pts = [_pt("A1", *A1, z=0.0), _pt("D1", *D1, z=z_of(D1)),
               _pt("A6", *A6, z=z_of(A6)),
               _pt("D6", *D6, z=z_of(D6) + 0.4)]      # 400 µm bad touch
        err = holdout_error_mm(pts)
        self.assertIsNotNone(err)
        self.assertGreater(err, 0.05)


class TestFocalReadings(unittest.TestCase):
    def _focal_pts(self, sign):
        # Focus readings carry the stage's own offset (+12.0 here).
        sy_true = -0.001

        def f_of(xy):
            return sign * (12.0 + sy_true * (xy[1] - A1[1]) / 1000.0)

        return [ZPlanePoint(label="L1", x_stage_um=A1[0], y_stage_um=A1[1],
                            focal_raw=f_of(A1), focal_unit="mm",
                            focal_mm=f_of(A1), source="focal_readout"),
                ZPlanePoint(label="L2", x_stage_um=D1[0], y_stage_um=D1[1],
                            focal_raw=f_of(D1), focal_unit="mm",
                            focal_mm=f_of(D1), source="focal_readout"),
                ZPlanePoint(label="L3", x_stage_um=A6[0], y_stage_um=A6[1],
                            focal_raw=f_of(A6), focal_unit="mm",
                            focal_mm=f_of(A6), source="focal_readout")]

    def test_anchor_pins_the_datum_exactly(self):
        anchor = _pt("A1", *A1, z=0.09)
        plane, why = from_focal_readings(points=self._focal_pts(+1),
                                        anchor=anchor, focal_sign=+1)
        self.assertIsNotNone(plane, why)
        # The focus stage's +12 mm offset must not leak into the datum.
        self.assertAlmostEqual(plane.z_zref_mm_at_stage_um(*A1), 0.09,
                               places=12)
        self.assertEqual(plane.mode, "focal_readout")

    def test_sign_mirrors_the_tilt_but_not_the_anchor(self):
        anchor = _pt("A1", *A1, z=0.09)
        pos, _ = from_focal_readings(points=self._focal_pts(+1), anchor=anchor,
                                    focal_sign=+1)
        neg, _ = from_focal_readings(points=self._focal_pts(+1), anchor=anchor,
                                    focal_sign=-1)
        self.assertAlmostEqual(pos.sy_mm_per_mm, -neg.sy_mm_per_mm, places=12)
        # Identical at the anchor — which is exactly why one touch-off cannot
        # reveal the sign, and why the 4th-point verify is mandatory.
        self.assertAlmostEqual(pos.z_zref_mm_at_stage_um(*A1),
                               neg.z_zref_mm_at_stage_um(*A1), places=12)
        self.assertEqual(pos.focal_sign, 1)
        self.assertEqual(neg.focal_sign, -1)

    def test_refuses_without_an_anchor(self):
        plane, why = from_focal_readings(points=self._focal_pts(+1),
                                        anchor=None)
        self.assertIsNone(plane)
        self.assertIn("anchor", why)

    def test_refuses_anchor_without_z(self):
        plane, why = from_focal_readings(points=self._focal_pts(+1),
                                        anchor=_pt("A1", *A1, z=None))
        self.assertIsNone(plane)
        self.assertIn("anchor", why)

    def test_um_readout_normalises(self):
        self.assertAlmostEqual(focal_to_mm(12500.0, "um"), 12.5)
        self.assertAlmostEqual(focal_to_mm(12.5, "mm"), 12.5)
        with self.assertRaises(ValueError):
            focal_to_mm(1.0, "furlongs")


class TestPlausibilityAndGeometry(unittest.TestCase):
    def test_level_is_plausible(self):
        ok, why = tilt_is_plausible(0.0, 0.0)
        self.assertTrue(ok, why)

    def test_slope_limit_catches_a_unit_slip(self):
        # A mm-vs-µm slip is ~1000×; the gate is far below that.
        self.assertFalse(tilt_is_plausible(1.0, 0.0)[0])
        self.assertFalse(tilt_is_plausible(0.0, 20.0)[0])
        self.assertTrue(tilt_is_plausible(0.001, -0.001)[0])

    def test_slope_limit_accepts_this_machines_real_tilt(self):
        """0.0199 mm/mm (~1.14°) was actually measured on ME3B V1.

        Pinned because an earlier, tighter gate rejected it — which would have
        made the tilt plane unusable on the very rig it was written for. The
        physical bound lives in the SPAN check, not the angle.
        """
        ok, why = tilt_is_plausible(-0.000725, -0.019862,
                                    bbox_um=(60000.0, 10000.0,
                                             105000.0, 68000.0))
        self.assertTrue(ok, why)

    def test_span_limit_over_a_big_plate(self):
        ok, why = tilt_is_plausible(0.004, 0.004,
                                    bbox_um=(0.0, 0.0, 300000.0, 300000.0))
        self.assertFalse(ok)
        self.assertIn("span", why)

    def test_non_finite_rejected(self):
        self.assertFalse(tilt_is_plausible(float("nan"), 0.0)[0])
        self.assertFalse(tilt_is_plausible(float("inf"), 0.0)[0])

    def test_triangle_area(self):
        # 45 mm × 58 mm right triangle ⇒ 1305 mm²
        area = triangle_max_area_mm2([A1, A6, D1])
        self.assertAlmostEqual(area, 45.0 * 58.0 / 2.0, places=6)
        self.assertGreater(area, MIN_TRIANGLE_AREA_MM2)

    def test_triangle_area_needs_three(self):
        self.assertEqual(triangle_max_area_mm2([A1, A6]), 0.0)

    def test_triangle_area_collinear_is_zero(self):
        self.assertAlmostEqual(
            triangle_max_area_mm2([(0.0, 0.0), (1000.0, 0.0), (2000.0, 0.0)]),
            0.0, places=9)


class TestSerialisation(unittest.TestCase):
    def _plane(self):
        pts = [_pt("A1", *A1, z=0.09), _pt("D1", *D1, z=0.15),
               _pt("A6", *A6, z=0.05)]
        plane, why = from_needle_touches(
            points=pts, anchor_label="A1",
            provenance={"zero_z_mm_at_fit": 1.5, "zero_xy_um": (10.0, 20.0),
                        "plate_key": "corning-24", "plate_flip_180": True,
                        "needle_tip_length_mm": 12.7,
                        "plate_bottom_z_source": "taught",
                        "z_up_sign_at_fit": -1.0,
                        "fitted_at": "2026-07-29T12:00:00"})
        self.assertIsNotNone(plane, why)
        return plane

    def test_round_trip_preserves_geometry_and_provenance(self):
        p = self._plane()
        q = PlateZPlane.from_dict(p.to_dict())
        self.assertIsNotNone(q)
        for f in ("x0_um", "y0_um", "z0_zref_mm", "sx_mm_per_mm",
                  "sy_mm_per_mm", "mode", "num_points", "plate_key",
                  "plate_flip_180", "needle_tip_length_mm",
                  "plate_bottom_z_source", "z_up_sign_at_fit",
                  "zero_z_mm_at_fit", "zero_xy_um"):
            self.assertEqual(getattr(q, f), getattr(p, f), f)
        self.assertEqual(len(q.points), len(p.points))
        self.assertEqual(q.points[0].label, "A1")
        # Evaluates identically after a round trip.
        self.assertAlmostEqual(q.z_zref_mm_at_stage_um(*D6),
                               p.z_zref_mm_at_stage_um(*D6), places=12)

    def test_status_is_never_restored_from_disk(self):
        """Trust is re-decided by the consumer on every load."""
        p = self._plane()
        d = p.to_dict()
        d["status"] = "active"
        q = PlateZPlane.from_dict(d)
        self.assertEqual(q.status, "unvalidated")

    def test_unknown_frame_is_refused(self):
        p = self._plane()
        d = p.to_dict()
        d["frame"] = "plate_local_mm/zref_mm"
        self.assertIsNone(PlateZPlane.from_dict(d))

    def test_legacy_abc_block_can_never_load(self):
        """The real on-disk legacy plane has no frame tag and no anchor."""
        legacy = {"a": -0.000725, "b": -0.019862, "c": 5.600,
                  "r_squared": 1.0}
        self.assertIsNone(PlateZPlane.from_dict(legacy))

    def test_malformed_returns_none(self):
        self.assertIsNone(PlateZPlane.from_dict({"frame": PLANE_FRAME}))
        self.assertIsNone(PlateZPlane.from_dict(None))
        self.assertIsNone(PlateZPlane.from_dict("nope"))

    def test_point_optional_keys_omitted_when_unset(self):
        d = _pt("A1", *A1, z=0.1).to_dict()
        for k in ("focal_raw", "focal_unit", "focal_mm", "needle_conf",
                  "focus_score", "focus_axis_um_at_record"):
            self.assertNotIn(k, d)


class TestReAnchorAndBbox(unittest.TestCase):
    def test_re_anchor_keeps_tilt_and_moves_the_datum(self):
        p = PlateZPlane(x0_um=A1[0], y0_um=A1[1], z0_zref_mm=0.09,
                        sx_mm_per_mm=0.001, sy_mm_per_mm=-0.002,
                        status="active")
        q = p.re_anchored(x0_um=D6[0], y0_um=D6[1], z0_zref_mm=1.5)
        self.assertEqual(q.sx_mm_per_mm, p.sx_mm_per_mm)
        self.assertEqual(q.sy_mm_per_mm, p.sy_mm_per_mm)
        self.assertAlmostEqual(q.z_zref_mm_at_stage_um(*D6), 1.5, places=12)
        # Re-anchoring must force re-validation, never inherit trust.
        self.assertEqual(q.status, "unvalidated")

    def test_points_bbox_and_inside(self):
        pts = [_pt("A1", *A1, z=0.0), _pt("D1", *D1, z=0.0),
               _pt("A6", *A6, z=0.0)]
        plane, _ = from_needle_touches(points=pts)
        bbox = plane.points_bbox_um()
        self.assertEqual(bbox, (60000.0, 10000.0, 105000.0, 68000.0))
        self.assertTrue(plane.is_inside(*CTR))
        self.assertTrue(plane.is_inside(59000.0, 10000.0))     # inside margin
        self.assertFalse(plane.is_inside(0.0, 0.0))            # far outside

    def test_is_inside_false_without_points(self):
        p = level_plane_at(x0_um=0.0, y0_um=0.0, z0_zref_mm=0.0)
        self.assertFalse(p.is_inside(0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
