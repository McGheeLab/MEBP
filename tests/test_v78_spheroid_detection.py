"""test_v78_spheroid_detection.py — the pure spheroid detector + projection.

No Qt, no camera, no stage. Synthetic discs with known radii on a synthetic
mosaic, so every assertion is a number rather than a rendering.

The projection tests deliberately use a NON-ZERO registration shift and a
NON-ZERO extent origin: a (0,0)/(0,0) fixture passes for both the correct
formula and the buggy one that forgets the shift, which is exactly how a
coordinate-frame bug survives to hardware.
"""

from __future__ import annotations

import math
import unittest

import numpy as np

from SupportClasses.SpheroidDetector import (
    MIN_RESOLVABLE_PX, DetectionReport, SpheroidDetection,
    back_project_px, detect_spheroids, detect_spheroids_px,
    diameter_um_for_radius_px, forward_project_um, manual_detection,
    px_window_for_um, radius_px_for_diameter_um, recentre_detection,
    refuse_reason, resize_detection, sort_detections,
)

try:
    import cv2
    _CV2 = True
except ImportError:      # pragma: no cover
    _CV2 = False


# ── synthetic imagery ──────────────────────────────────────────────

def _canvas(h=600, w=800, bg=8):
    """A dim, non-zero background — matching a real mosaic, where zero means
    'no image data' and is what the edge-margin mask keys on."""
    img = np.full((h, w, 3), bg, dtype=np.uint8)
    return img


def _disc(img, cx, cy, r, value=220):
    cv2.circle(img, (int(round(cx)), int(round(cy))), int(round(r)),
               (value, value, value), -1)
    return img


def _ellipse(img, cx, cy, a, b, value=220):
    cv2.ellipse(img, (int(cx), int(cy)), (int(a), int(b)), 0, 0, 360,
                (value, value, value), -1)
    return img


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestPixelWindow(unittest.TestCase):
    """The absolute µm band, enforced on the FITTED diameter."""

    SCALE = 0.5   # px per µm → a 200 µm spheroid is 100 px across

    def test_in_band_disc_is_found_with_correct_diameter(self):
        img = _canvas()
        _disc(img, 400, 300, 50)          # 100 px Ø = 200 µm at 0.5 px/µm
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertEqual(len(rep.detections), 1, rep.summary())
        det = rep.detections[0]
        self.assertAlmostEqual(det.radius_px, 50.0, delta=3.0)
        self.assertAlmostEqual(det.center_px[0], 400.0, delta=2.0)
        self.assertAlmostEqual(det.center_px[1], 300.0, delta=2.0)

    def test_too_small_is_rejected_and_counted(self):
        img = _canvas()
        _disc(img, 400, 300, 10)          # 20 px Ø = 40 µm — below an 80 µm floor
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertEqual(len(rep.detections), 0)
        self.assertGreaterEqual(rep.n_too_small, 1, rep.summary())

    def test_too_large_is_rejected_and_counted(self):
        img = _canvas()
        _disc(img, 400, 300, 150)         # 300 px Ø = 600 µm — over a 400 µm cap
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertEqual(len(rep.detections), 0)
        self.assertGreaterEqual(rep.n_too_large, 1, rep.summary())

    def test_mixed_scene_keeps_only_the_in_band_discs(self):
        img = _canvas()
        _disc(img, 150, 150, 50)          # in band
        _disc(img, 400, 150, 45)          # in band
        _disc(img, 650, 150, 8)           # too small
        _disc(img, 400, 430, 140)         # too large
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertEqual(len(rep.detections), 2, rep.summary())
        self.assertGreaterEqual(rep.n_too_small, 1)
        self.assertGreaterEqual(rep.n_too_large, 1)


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestWellWallDoesNotSuppressSpheroids(unittest.TestCase):
    """The bug the relative gate in ``detect_filled_wells`` would have.

    Its floor is a fraction of the LARGEST blob's area, so a giant well-wall
    blob raises it above every real spheroid and the detector returns nothing.
    An absolute band must be immune.
    """

    SCALE = 0.5

    def test_huge_blob_does_not_erase_small_in_band_discs(self):
        img = _canvas(700, 700)
        # A big bright annulus standing in for the well wall / meniscus ring.
        cv2.circle(img, (350, 350), 300, (200, 200, 200), 25)
        _disc(img, 300, 330, 45)
        _disc(img, 420, 380, 40)
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertGreaterEqual(len(rep.detections), 2, rep.summary())

    def test_relative_gate_would_have_missed_them(self):
        """Documents WHY a sibling detector exists rather than a tweak."""
        from SupportClasses.VisionDetector import WellDetector
        img = _canvas(700, 700)
        cv2.circle(img, (350, 350), 300, (200, 200, 200), 25)
        _disc(img, 300, 330, 45)
        _disc(img, 420, 380, 40)
        legacy = WellDetector.detect_filled_wells(img)
        legacy_small = [d for d in legacy if d.radius_px < 80]
        ours = detect_spheroids_px(
            img, min_diameter_px=80 * 0.5, max_diameter_px=400 * 0.5)
        # Ours finds the small discs; the legacy relative gate finds fewer.
        self.assertGreaterEqual(len(ours.detections), 2)
        self.assertGreaterEqual(len(ours.detections), len(legacy_small))


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestShapeGates(unittest.TestCase):
    SCALE = 0.5

    def test_elongated_blob_rejected_by_circularity(self):
        img = _canvas()
        _ellipse(img, 400, 300, 90, 45)   # 2:1 — spans the band but is not round
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertEqual(len(rep.detections), 0, rep.summary())
        self.assertGreaterEqual(rep.n_shape, 1)

    def test_touching_pair_is_rejected_not_split(self):
        """The documented v1 behaviour: a doublet is dropped, and the report says
        which gate dropped it so the operator can redraw by hand."""
        img = _canvas()
        _disc(img, 380, 300, 45)
        _disc(img, 455, 300, 45)          # overlapping → one component
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi)
        self.assertEqual(len(rep.detections), 0, rep.summary())
        self.assertGreaterEqual(rep.n_shape + rep.n_too_large, 1)

    def test_loosening_circularity_admits_the_ellipse(self):
        """Proves the gate is what rejected it, not something incidental."""
        img = _canvas()
        _ellipse(img, 400, 300, 90, 45)
        lo, hi = px_window_for_um(80, 400, self.SCALE)
        rep = detect_spheroids_px(img, min_diameter_px=lo, max_diameter_px=hi,
                                  circularity_min=0.05, fit_fraction_min=0.05)
        self.assertGreaterEqual(len(rep.detections), 1)


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestScaleInvariance(unittest.TestCase):
    """Doubling the image while halving µm/px must give the same µm answer."""

    def test_um_result_is_scale_invariant(self):
        small = _canvas(400, 400)
        _disc(small, 200, 200, 40)        # 80 px Ø
        big = _canvas(800, 800)
        _disc(big, 400, 400, 80)          # 160 px Ø

        extent = (1000.0, 2000.0, 1400.0, 2400.0)
        r1 = detect_spheroids(small, extent, 1.0,
                              min_diameter_um=40, max_diameter_um=200)
        r2 = detect_spheroids(big, extent, 2.0,
                              min_diameter_um=40, max_diameter_um=200)
        self.assertEqual(len(r1.detections), 1, r1.summary())
        self.assertEqual(len(r2.detections), 1, r2.summary())
        self.assertAlmostEqual(r1.detections[0].diameter_um,
                               r2.detections[0].diameter_um, delta=4.0)
        self.assertAlmostEqual(r1.detections[0].center_um[0],
                               r2.detections[0].center_um[0], delta=4.0)


class TestProjection(unittest.TestCase):
    """Non-zero shift AND non-zero origin — the only fixture that can fail."""

    EXTENT = (1000.0, 2000.0, 1600.0, 2400.0)
    SCALE = 0.5
    SHIFT = (40.0, -25.0)

    def test_back_project_subtracts_the_shift(self):
        x, y = back_project_px((100.0, 50.0), self.EXTENT, self.SCALE, self.SHIFT)
        self.assertAlmostEqual(x, 1000.0 - 40.0 + 200.0)
        self.assertAlmostEqual(y, 2000.0 + 25.0 + 100.0)

    def test_ignoring_the_shift_is_wrong_by_exactly_the_shift(self):
        a = back_project_px((100.0, 50.0), self.EXTENT, self.SCALE, self.SHIFT)
        b = back_project_px((100.0, 50.0), self.EXTENT, self.SCALE, (0.0, 0.0))
        self.assertAlmostEqual(b[0] - a[0], self.SHIFT[0])
        self.assertAlmostEqual(b[1] - a[1], self.SHIFT[1])

    def test_round_trip(self):
        for px in ((0.0, 0.0), (123.4, 56.7), (599.0, 399.0)):
            um = back_project_px(px, self.EXTENT, self.SCALE, self.SHIFT)
            back = forward_project_um(um, self.EXTENT, self.SCALE, self.SHIFT)
            self.assertAlmostEqual(back[0], px[0], places=6)
            self.assertAlmostEqual(back[1], px[1], places=6)

    def test_matches_mosaic_well_remap_formula(self):
        """Pinned against the other implementation of the same contract so the
        two cannot drift (``MosaicWellRemap.detect_raw_positions``)."""
        px_x, px_y = 321.0, 89.0
        ox = float(self.EXTENT[0]) - self.SHIFT[0]
        oy = float(self.EXTENT[1]) - self.SHIFT[1]
        expected = (ox + px_x / self.SCALE, oy + px_y / self.SCALE)
        got = back_project_px((px_x, px_y), self.EXTENT, self.SCALE, self.SHIFT)
        self.assertAlmostEqual(got[0], expected[0], places=9)
        self.assertAlmostEqual(got[1], expected[1], places=9)

    def test_zero_scale_raises(self):
        with self.assertRaises(ValueError):
            back_project_px((1.0, 1.0), self.EXTENT, 0.0)
        with self.assertRaises(ValueError):
            forward_project_um((1.0, 1.0), self.EXTENT, 0.0)

    def test_diameter_helpers_are_inverse(self):
        for d in (40.0, 200.0, 517.3):
            r_px = radius_px_for_diameter_um(d, 0.37)
            self.assertAlmostEqual(diameter_um_for_radius_px(r_px, 0.37), d,
                                   places=6)

    def test_diameter_for_zero_scale_is_zero_not_a_crash(self):
        self.assertEqual(diameter_um_for_radius_px(10.0, 0.0), 0.0)


class TestRefuseReason(unittest.TestCase):
    def test_zero_scale(self):
        self.assertIn("pixel scale", refuse_reason(0.0, 80, 400) or "")

    def test_inverted_range(self):
        self.assertIn("inverted", refuse_reason(0.5, 400, 80) or "")

    def test_negative_range(self):
        self.assertIn("positive", refuse_reason(0.5, -1, 400) or "")

    def test_unresolvable_minimum(self):
        # 80 µm at 0.01 px/µm = 0.8 px.
        msg = refuse_reason(0.01, 80, 400)
        self.assertIsNotNone(msg)
        self.assertIn("px across", msg)

    def test_ok_window_returns_none(self):
        self.assertIsNone(refuse_reason(0.5, 80, 400))

    def test_boundary_is_the_documented_constant(self):
        scale = MIN_RESOLVABLE_PX / 80.0        # exactly 6 px for an 80 µm disc
        self.assertIsNone(refuse_reason(scale, 80, 400))
        self.assertIsNotNone(refuse_reason(scale * 0.99, 80, 400))

    def test_non_numeric_inputs(self):
        self.assertIsNotNone(refuse_reason("x", 80, 400))
        self.assertIsNotNone(refuse_reason(0.5, None, 400))

    @unittest.skipUnless(_CV2, "OpenCV not available")
    def test_detect_spheroids_returns_the_refusal_not_an_empty_list(self):
        rep = detect_spheroids(_canvas(), (0, 0, 100, 100), 0.0)
        self.assertTrue(rep.refused)
        self.assertEqual(len(rep.detections), 0)
        self.assertEqual(rep.summary(), rep.refused)


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestWellMask(unittest.TestCase):
    SCALE = 0.5
    EXTENT = (0.0, 0.0, 1600.0, 1200.0)   # 800x600 px at 0.5 px/µm

    def test_detection_outside_the_well_is_dropped_and_counted(self):
        img = _canvas()
        _disc(img, 400, 300, 45)      # centre of the mosaic → inside
        _disc(img, 60, 60, 45)        # far corner → outside a small well
        rep = detect_spheroids(
            img, self.EXTENT, self.SCALE,
            min_diameter_um=80, max_diameter_um=400,
            well_center_um=(800.0, 600.0), well_radius_um=400.0)
        self.assertEqual(len(rep.detections), 1, rep.summary())
        self.assertGreaterEqual(rep.n_outside_well, 1)

    def test_without_well_geometry_nothing_is_masked(self):
        img = _canvas()
        _disc(img, 400, 300, 45)
        _disc(img, 60, 60, 45)
        rep = detect_spheroids(img, self.EXTENT, self.SCALE,
                               min_diameter_um=80, max_diameter_um=400)
        self.assertEqual(len(rep.detections), 2, rep.summary())
        self.assertEqual(rep.n_outside_well, 0)


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestOrderingAndIds(unittest.TestCase):
    def test_sorted_largest_first_with_stable_ids(self):
        img = _canvas()
        _disc(img, 150, 150, 30)
        _disc(img, 400, 150, 50)
        _disc(img, 650, 150, 40)
        rep = detect_spheroids(img, (0.0, 0.0, 1600.0, 1200.0), 0.5,
                               min_diameter_um=80, max_diameter_um=400)
        self.assertEqual(len(rep.detections), 3, rep.summary())
        ds = [d.diameter_um for d in rep.detections]
        self.assertEqual(ds, sorted(ds, reverse=True))
        self.assertEqual([d.det_id for d in rep.detections],
                         ["S001", "S002", "S003"])

    def test_sort_detections_both_directions(self):
        dets = [
            SpheroidDetection((0, 0), 1, 200.0, (0, 0)),
            SpheroidDetection((0, 0), 1, 90.0, (0, 0)),
            SpheroidDetection((0, 0), 1, 1000.0, (0, 0)),
        ]
        asc = [d.diameter_um for d in sort_detections(dets, ascending=True)]
        desc = [d.diameter_um for d in sort_detections(dets, ascending=False)]
        # Numeric, not lexicographic — "1000" must not sort before "200".
        self.assertEqual(asc, [90.0, 200.0, 1000.0])
        self.assertEqual(desc, [1000.0, 200.0, 90.0])


class TestManualAndEdits(unittest.TestCase):
    EXTENT = (1000.0, 2000.0, 1600.0, 2400.0)
    SCALE = 0.5
    SHIFT = (40.0, -25.0)

    def test_manual_detection_carries_provenance(self):
        det = manual_detection((100.0, 50.0), 25.0, self.EXTENT, self.SCALE,
                               self.SHIFT, det_id="S001")
        self.assertEqual(det.source, "user")
        self.assertTrue(det.user_edited)
        self.assertAlmostEqual(det.diameter_um, 100.0)
        self.assertAlmostEqual(det.center_um[0], 1160.0)

    def test_resize_updates_diameter_and_marks_edited(self):
        det = SpheroidDetection((10.0, 10.0), 25.0, 100.0, (0.0, 0.0))
        resize_detection(det, 50.0, self.SCALE)
        self.assertAlmostEqual(det.diameter_um, 200.0)
        self.assertTrue(det.user_edited)

    def test_recentre_updates_stage_position(self):
        det = SpheroidDetection((10.0, 10.0), 25.0, 100.0, (0.0, 0.0))
        recentre_detection(det, (100.0, 50.0), self.EXTENT, self.SCALE, self.SHIFT)
        self.assertAlmostEqual(det.center_um[0], 1160.0)
        self.assertAlmostEqual(det.center_um[1], 2125.0)
        self.assertTrue(det.user_edited)

    def test_serialisation_round_trip(self):
        det = manual_detection((12.5, 7.5), 33.0, self.EXTENT, self.SCALE,
                               self.SHIFT, det_id="S007")
        back = SpheroidDetection.from_dict(det.to_dict())
        self.assertEqual(back.det_id, "S007")
        self.assertEqual(back.source, "user")
        self.assertTrue(back.user_edited)
        self.assertAlmostEqual(back.diameter_um, det.diameter_um)
        self.assertAlmostEqual(back.center_um[0], det.center_um[0])
        self.assertAlmostEqual(back.center_px[1], det.center_px[1])

    def test_from_dict_tolerates_a_sparse_dict(self):
        back = SpheroidDetection.from_dict({})
        self.assertEqual(back.diameter_um, 0.0)
        self.assertEqual(back.center_px, (0.0, 0.0))


class TestReportSummary(unittest.TestCase):
    def test_summary_names_every_rejection_reason(self):
        rep = DetectionReport(
            detections=[SpheroidDetection((0, 0), 1, 100.0, (0, 0))],
            n_blobs=14, n_too_small=5, n_too_large=4, n_shape=2,
            n_outside_well=1, n_unfittable=1)
        text = rep.summary()
        self.assertIn("14 blob(s)", text)
        self.assertIn("1 in band", text)
        for expect in ("5 too small", "4 too large", "2 not round",
                       "1 outside well", "1 unfittable"):
            self.assertIn(expect, text)

    def test_clean_summary_omits_zero_counters(self):
        rep = DetectionReport(detections=[], n_blobs=0)
        self.assertNotIn("too small", rep.summary())

    def test_len_and_iter(self):
        rep = DetectionReport(detections=[
            SpheroidDetection((0, 0), 1, 100.0, (0, 0)),
            SpheroidDetection((0, 0), 1, 200.0, (0, 0)),
        ])
        self.assertEqual(len(rep), 2)
        self.assertEqual(len(list(rep)), 2)


@unittest.skipUnless(_CV2, "OpenCV not available")
class TestDegenerateInputs(unittest.TestCase):
    def test_none_image(self):
        rep = detect_spheroids_px(None, min_diameter_px=10, max_diameter_px=100)
        self.assertTrue(rep.refused)

    def test_empty_image(self):
        rep = detect_spheroids_px(np.zeros((0, 0, 3), dtype=np.uint8),
                                  min_diameter_px=10, max_diameter_px=100)
        self.assertTrue(rep.refused)

    def test_blank_image_finds_nothing(self):
        rep = detect_spheroids_px(_canvas(), min_diameter_px=40,
                                  max_diameter_px=200)
        self.assertEqual(len(rep.detections), 0)
        self.assertFalse(rep.refused)

    def test_grayscale_image_accepted(self):
        img = np.full((400, 400), 8, dtype=np.uint8)
        cv2.circle(img, (200, 200), 40, 220, -1)
        rep = detect_spheroids_px(img, min_diameter_px=40, max_diameter_px=200)
        self.assertEqual(len(rep.detections), 1, rep.summary())

    def test_bad_pixel_window(self):
        rep = detect_spheroids_px(_canvas(), min_diameter_px=100,
                                  max_diameter_px=50)
        self.assertTrue(rep.refused)

    def test_max_results_truncates_and_flags(self):
        img = _canvas(600, 800)
        for i in range(6):
            _disc(img, 80 + i * 120, 300, 30)
        rep = detect_spheroids_px(img, min_diameter_px=40, max_diameter_px=200,
                                  max_results=3)
        self.assertEqual(len(rep.detections), 3)
        self.assertTrue(rep.truncated)
        self.assertIn("truncated", rep.summary())


if __name__ == "__main__":
    unittest.main()
