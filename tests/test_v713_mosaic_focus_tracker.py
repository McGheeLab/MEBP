"""
test_v713_mosaic_focus_tracker.py — checkerboard AF scheduling + surface fit.

Pure module (SupportClasses/MosaicFocusTracker.py). The load-bearing claims:
- sweeps land on a 2-D staggered LATTICE keyed on the tile's grid (col, row),
  never on the serpentine SEQUENCE index (which collapses to stripes);
- refused sweeps and outliers never enter the fit (an empty tile must not
  drag the surface);
- the running plane fit recovers a synthetic tilt and predicts new tiles.
"""

import math
import unittest

from SupportClasses.MosaicFocusTracker import (
    FocusSampleXY, MosaicFocusTracker, PlanePredictor, grid_indices)


def _serpentine_grid(cols=6, rows=6, pitch=1000.0):
    """Raster positions in SERPENTINE order (like the mosaic generator)."""
    pos = []
    for r in range(rows):
        xs = range(cols) if r % 2 == 0 else range(cols - 1, -1, -1)
        for c in xs:
            pos.append((c * pitch, r * pitch))
    return pos


def _tracker(positions, spacing=2, dof=10.0, center=(2500.0, 2500.0),
             radius=10000.0):
    return MosaicFocusTracker(
        well_center_um=center, well_radius_um=radius,
        lattice_spacing=spacing, dof_um=dof, positions=positions)


class TestGridIndices(unittest.TestCase):
    def test_serpentine_recovers_cols_rows(self):
        pos = _serpentine_grid(4, 3)
        gi = grid_indices(pos)
        # First raster row ascending, second descending.
        self.assertEqual(gi[0], (0, 0))
        self.assertEqual(gi[3], (3, 0))
        self.assertEqual(gi[4], (3, 1))     # serpentine turn
        self.assertEqual(gi[7], (0, 1))


class TestCheckerboardScheduling(unittest.TestCase):
    def test_lattice_covers_both_axes_on_serpentine(self):
        # Mutation guard: `tile_index % n == 0` over the SEQUENCE (instead of
        # the grid lattice) samples every raster row on a 6-wide serpentine
        # with n=2 — but it can NEVER produce staggered columns. The lattice
        # must sample a spread of distinct cols AND rows with gaps ≤ spacing.
        pos = _serpentine_grid(6, 6)
        tr = _tracker(pos, spacing=2)
        chosen = [(c, r) for i, (x, y) in enumerate(pos)
                  for (c, r) in [grid_indices(pos)[i]]
                  if tr.should_af(i, x, y)]
        self.assertTrue(chosen)
        rows = sorted({r for _c, r in chosen})
        cols = sorted({c for c, _r in chosen})
        self.assertEqual(rows, [0, 2, 4])
        self.assertGreaterEqual(len(cols), 3)
        # Stagger: alternate lattice rows use different column phases.
        cols_r0 = {c for c, r in chosen if r == 0}
        cols_r2 = {c for c, r in chosen if r == 2}
        self.assertNotEqual(cols_r0, cols_r2)

    def test_spacing_one_sweeps_every_in_well_tile(self):
        pos = _serpentine_grid(3, 3)
        tr = _tracker(pos, spacing=1)
        self.assertTrue(all(tr.should_af(i, x, y)
                            for i, (x, y) in enumerate(pos)))

    def test_out_of_well_never_swept(self):
        pos = _serpentine_grid(6, 6)
        tr = _tracker(pos, spacing=1, center=(2500.0, 2500.0), radius=1500.0)
        for i, (x, y) in enumerate(pos):
            if math.hypot(x - 2500.0, y - 2500.0) > 1500.0:
                self.assertFalse(tr.should_af(i, x, y), (i, x, y))

    def test_in_well_boundary_inclusive(self):
        tr = _tracker([(0.0, 0.0)], center=(0.0, 0.0), radius=100.0)
        self.assertTrue(tr.in_well(100.0, 0.0))
        self.assertFalse(tr.in_well(100.1, 0.0))


class TestFitAndPredict(unittest.TestCase):
    def _add(self, tr, x, y, f):
        return tr.add(FocusSampleXY(x_um=x, y_um=y, focus_um=f))

    def test_predict_none_then_mean_then_plane(self):
        tr = _tracker(_serpentine_grid(3, 3))
        self.assertIsNone(tr.predict(0, 0))
        self._add(tr, 0, 0, 100.0)
        self.assertAlmostEqual(tr.predict(9999, 9999), 100.0)
        self._add(tr, 1000, 0, 110.0)
        self.assertAlmostEqual(tr.predict(0, 0), 105.0)   # mean of 2
        self._add(tr, 0, 1000, 120.0)
        # Plane now: f = 100 + 0.01x + 0.02y
        self.assertAlmostEqual(tr.predict(2000, 1000), 100 + 20 + 20, places=3)

    def test_plane_recovery_on_synthetic_tilt(self):
        tr = _tracker(_serpentine_grid(5, 5))
        a, b, c = 0.004, -0.007, 1234.5
        for (x, y) in [(0, 0), (4000, 0), (0, 4000), (2000, 3000),
                       (4000, 4000), (1000, 2000)]:
            self._add(tr, x, y, a * x + b * y + c)
        self.assertAlmostEqual(tr.predict(3000, 1000),
                               a * 3000 + b * 1000 + c, places=1)
        self.assertLess(tr.rms_residual_um(), 0.1)

    def test_collinear_samples_do_not_crash(self):
        tr = _tracker(_serpentine_grid(3, 3))
        for x in (0, 1000, 2000, 3000):
            self._add(tr, x, 0, 100 + 0.01 * x)
        self.assertIsNotNone(tr.predict(4000, 0))

    def test_outlier_rejected_and_counted(self):
        tr = _tracker(_serpentine_grid(4, 4), dof=10.0)
        for (x, y, f) in [(0, 0, 100), (1000, 0, 100), (0, 1000, 100),
                          (1000, 1000, 100)]:
            self.assertTrue(self._add(tr, x, y, f))
        # 6 × DOF = 60 µm gate: a 200 µm mis-lock is rejected.
        self.assertFalse(self._add(tr, 500, 500, 300.0))
        self.assertEqual(tr.summary()["n_outlier"], 1)
        self.assertEqual(tr.summary()["n_accepted"], 4)
        # And it did NOT drag the fit.
        self.assertAlmostEqual(tr.predict(500, 500), 100.0, places=1)

    def test_refused_sweeps_never_enter_the_fit(self):
        # THE bug this module exists to prevent: an empty tile's refused curve
        # contributing to the surface.
        tr = _tracker(_serpentine_grid(3, 3))
        self._add(tr, 0, 0, 100.0)
        tr.note_refused()
        tr.note_refused()
        s = tr.summary()
        self.assertEqual(s["n_refused"], 2)
        self.assertEqual(s["n_accepted"], 1)
        self.assertAlmostEqual(tr.predict(0, 0), 100.0)

    def test_summary_fields(self):
        tr = _tracker(_serpentine_grid(3, 3), spacing=3, dof=5.0)
        for (x, y, f) in [(0, 0, 10), (2000, 0, 12), (0, 2000, 14),
                          (2000, 2000, 16)]:
            self._add(tr, x, y, f)
        s = tr.summary()
        self.assertEqual(s["n_accepted"], 4)
        self.assertEqual(s["lattice_spacing"], 3)
        self.assertEqual(s["span_x_um"], 2000.0)
        self.assertIsNotNone(s["plane"])
        self.assertIsNotNone(s["rms_residual_um"])


class TestPlanePredictor(unittest.TestCase):
    def test_replay_from_dicts(self):
        samples = [{"x_um": x, "y_um": y, "focus_um": 50 + 0.01 * x}
                   for (x, y) in [(0, 0), (1000, 0), (0, 1000), (2000, 500)]]
        pred = PlanePredictor(samples)
        self.assertEqual(pred.n_samples(), 4)
        self.assertAlmostEqual(pred.predict(3000, 0), 80.0, places=1)

    def test_empty_predicts_none(self):
        self.assertIsNone(PlanePredictor([]).predict(0, 0))


if __name__ == "__main__":
    unittest.main()
