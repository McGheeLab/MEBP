"""
v7.10 — per-bore focus ROIs. Pure math, no Qt.

The overlap rule is the load-bearing one: two boxes that touch make every bore
peak at the same Z, which is BUG B (all bores in one Z plane) reproduced by a
different route — and it would look like a genuine measurement.
"""

import unittest

from SupportClasses.BoreFocusROI import (
    MIN_ROI_PX,
    bore_roi_rect,
    nearest_neighbour_px,
    rois_for_clicks,
    rois_overlap,
    view_px_to_frame_px,
)

FRAME = (1920, 1080)


class TestSizing(unittest.TestCase):

    def test_side_scales_with_the_bore_diameter(self):
        # od 718 µm at 0.5 µm/px = 1436 px; x3 = 4308 px, clamped to the frame
        # height. Use a small bore so the multiple is what is measured.
        x, y, w, h = bore_roi_rect((960, 540), FRAME, um_per_px=0.5,
                                   bore_od_um=100.0)
        self.assertEqual(w, h)
        self.assertAlmostEqual(w, 3.0 * 100.0 / 0.5, delta=1)

    def test_a_finer_scale_gives_a_bigger_box_in_pixels(self):
        coarse = bore_roi_rect((960, 540), FRAME, 1.0, 100.0)[2]
        fine = bore_roi_rect((960, 540), FRAME, 0.25, 100.0)[2]
        self.assertGreater(fine, coarse)

    def test_never_below_the_minimum(self):
        w = bore_roi_rect((960, 540), FRAME, um_per_px=50.0,
                          bore_od_um=1.0)[2]
        self.assertGreaterEqual(w, MIN_ROI_PX)

    def test_never_larger_than_the_frame(self):
        x, y, w, h = bore_roi_rect((960, 540), FRAME, 0.01, 5000.0)
        self.assertLessEqual(w, min(FRAME))
        self.assertLessEqual(h, min(FRAME))

    def test_missing_scale_falls_back_to_the_minimum(self):
        self.assertEqual(bore_roi_rect((960, 540), FRAME, 0.0, 718.0)[2],
                         MIN_ROI_PX)


class TestClamping(unittest.TestCase):

    def test_stays_inside_the_frame_at_a_corner(self):
        x, y, w, h = bore_roi_rect((2, 3), FRAME, 0.5, 100.0)
        self.assertGreaterEqual(x, 0)
        self.assertGreaterEqual(y, 0)
        self.assertLessEqual(x + w, FRAME[0])
        self.assertLessEqual(y + h, FRAME[1])

    def test_stays_inside_at_the_far_corner(self):
        x, y, w, h = bore_roi_rect((1918, 1078), FRAME, 0.5, 100.0)
        self.assertLessEqual(x + w, FRAME[0])
        self.assertLessEqual(y + h, FRAME[1])

    def test_clamping_preserves_the_size(self):
        """Boxes must stay the same size so scores are comparable per bore."""
        centre = bore_roi_rect((960, 540), FRAME, 0.5, 100.0)[2]
        corner = bore_roi_rect((5, 5), FRAME, 0.5, 100.0)[2]
        self.assertEqual(centre, corner)


class TestNeighbourSeparation(unittest.TestCase):
    """THE important behaviour."""

    def test_boxes_shrink_to_clear_a_close_neighbour(self):
        wide = bore_roi_rect((960, 540), FRAME, 0.5, 400.0)[2]
        tight = bore_roi_rect((960, 540), FRAME, 0.5, 400.0,
                              others=[(1060, 540)])[2]
        self.assertLess(tight, wide)

    def test_two_close_bores_never_overlap(self):
        """100 µm apart at 0.5 µm/px = 200 px, with a 400 µm bore whose natural
        box would be 2400 px — the worst realistic case."""
        clicks = {0: (900, 540), 1: (1100, 540)}
        rois = rois_for_clicks(clicks, FRAME, 0.5, 400.0)
        self.assertFalse(rois_overlap(rois[0], rois[1]),
                         "overlapping ROIs make every bore peak at the same Z")

    def test_a_triple_never_overlaps(self):
        clicks = {0: (900, 500), 1: (1100, 500), 2: (1000, 660)}
        rois = rois_for_clicks(clicks, FRAME, 0.5, 400.0)
        for a in rois:
            for b in rois:
                if a < b:
                    self.assertFalse(rois_overlap(rois[a], rois[b]),
                                     f"bores {a} and {b} overlap")

    def test_a_lone_bore_keeps_its_full_box(self):
        alone = bore_roi_rect((960, 540), FRAME, 0.5, 100.0)[2]
        with_far = bore_roi_rect((960, 540), FRAME, 0.5, 100.0,
                                 others=[(50, 50)])[2]
        self.assertEqual(alone, with_far)

    def test_adding_a_bore_tightens_the_existing_ones(self):
        one = rois_for_clicks({0: (960, 540)}, FRAME, 0.5, 400.0)
        two = rois_for_clicks({0: (960, 540), 1: (1060, 540)},
                              FRAME, 0.5, 400.0)
        self.assertLess(two[0][2], one[0][2])

    def test_a_coincident_click_is_ignored_not_a_zero_box(self):
        """Two clicks at the same pixel would otherwise shrink to nothing."""
        w = bore_roi_rect((960, 540), FRAME, 0.5, 100.0,
                          others=[(960, 540)])[2]
        self.assertGreaterEqual(w, MIN_ROI_PX)


class TestNearestNeighbour(unittest.TestCase):

    def test_none_when_alone(self):
        self.assertIsNone(nearest_neighbour_px((0, 0), []))

    def test_picks_the_closest(self):
        d = nearest_neighbour_px((0, 0), [(300, 0), (100, 0), (500, 0)])
        self.assertAlmostEqual(d, 100.0)

    def test_skips_a_coincident_point(self):
        self.assertAlmostEqual(
            nearest_neighbour_px((0, 0), [(0, 0), (250, 0)]), 250.0)

    def test_it_is_CHEBYSHEV_not_euclidean(self):
        """Axis-aligned squares need max(|dx|,|dy|). A diagonal neighbour at
        (100,160) is 189 px away Euclidean but only 160 px away in the sense
        that matters — sizing to 189 would let the boxes overlap."""
        self.assertAlmostEqual(nearest_neighbour_px((0, 0), [(100, 160)]),
                               160.0)
        self.assertNotAlmostEqual(nearest_neighbour_px((0, 0), [(100, 160)]),
                                  188.68, places=1)


class TestViewToFrameRescale(unittest.TestCase):

    def test_identity_when_the_sizes_match(self):
        self.assertEqual(view_px_to_frame_px((10, 20), (640, 480), (640, 480)),
                         (10.0, 20.0))

    def test_scales_up_to_the_raw_frame(self):
        self.assertEqual(view_px_to_frame_px((10, 20), (640, 480), (1280, 960)),
                         (20.0, 40.0))

    def test_a_degenerate_view_size_degrades_to_the_raw_click(self):
        self.assertEqual(view_px_to_frame_px((10, 20), (0, 0), (640, 480)),
                         (10.0, 20.0))


class TestRobustness(unittest.TestCase):

    def test_non_finite_click_does_not_raise(self):
        bore_roi_rect((float("nan"), float("inf")), FRAME, 0.5, 100.0)

    def test_empty_click_set_returns_empty(self):
        self.assertEqual(rois_for_clicks({}, FRAME, 0.5, 100.0), {})

    def test_none_clicks_are_skipped(self):
        rois = rois_for_clicks({0: (960, 540), 1: None}, FRAME, 0.5, 100.0)
        self.assertEqual(list(rois), [0])


if __name__ == "__main__":
    unittest.main()
