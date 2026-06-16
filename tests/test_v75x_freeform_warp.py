"""
v7.5.x — Freeform-point plate calibration: interpolating warp.

Verifies SupportClasses.PlateWarpCalibrator (affine base + TPS residual):
  * Graceful degradation by control-point count
    (0→identity, 1→translation, 2→similarity, 3→affine, ≥4→affine+TPS).
  * The corrected plate passes EXACTLY through every control point
    (the property the global similarity fit did NOT have).
  * Raw per-point deltas / mean / max correction reporting.
  * to_dict / from_dict round-trip reproduces the correction.
  * Degenerate (collinear) layouts don't crash and stay finite.
"""

import math
import unittest

from SupportClasses.PlateWarpCalibrator import PlateWarpCalibrator


def _affine(x, y, a, b, c, d, tx, ty):
    """out = [[a,b],[c,d]] @ [x,y] + [tx,ty]."""
    return (a * x + b * y + tx, c * x + d * y + ty)


class TestDegradationModes(unittest.TestCase):
    def test_zero_points_identity(self):
        w = PlateWarpCalibrator()
        w.solve()
        self.assertEqual(w.mode, "identity")
        self.assertEqual(w.transform(1234.0, 5678.0), (1234.0, 5678.0))

    def test_one_point_translation(self):
        w = PlateWarpCalibrator()
        w.add_point(1000.0, 2000.0, 1100.0, 1950.0)  # Δ = (+100, -50)
        w.solve()
        self.assertEqual(w.mode, "translation")
        # Exact at the control point...
        gx, gy = w.transform(1000.0, 2000.0)
        self.assertAlmostEqual(gx, 1100.0, delta=0.01)
        self.assertAlmostEqual(gy, 1950.0, delta=0.01)
        # ...and the same shift everywhere else.
        gx, gy = w.transform(50000.0, 60000.0)
        self.assertAlmostEqual(gx, 50100.0, delta=0.01)
        self.assertAlmostEqual(gy, 59950.0, delta=0.01)

    def test_two_points_similarity(self):
        # Pure rotation (5°) + uniform scale (1.01) + translation.
        ang = math.radians(5.0)
        s, tx, ty = 1.01, 200.0, -150.0
        ca, sa = math.cos(ang) * s, math.sin(ang) * s
        pts = [(0.0, 0.0), (40000.0, 1000.0)]
        w = PlateWarpCalibrator()
        for px, py in pts:
            mx, my = (ca * px - sa * py + tx, sa * px + ca * py + ty)
            w.add_point(px, py, mx, my)
        w.solve()
        self.assertEqual(w.mode, "similarity")
        for px, py in pts:
            mx, my = (ca * px - sa * py + tx, sa * px + ca * py + ty)
            gx, gy = w.transform(px, py)
            self.assertAlmostEqual(gx, mx, delta=0.1)
            self.assertAlmostEqual(gy, my, delta=0.1)

    def test_three_points_affine_exact(self):
        # Non-uniform scale + shear — NOT representable by a similarity.
        a, b, c, d, tx, ty = 1.02, 0.013, -0.006, 0.985, 80.0, -40.0
        pts = [(0.0, 0.0), (50000.0, 0.0), (0.0, 30000.0)]
        w = PlateWarpCalibrator()
        for px, py in pts:
            w.add_point(px, py, *_affine(px, py, a, b, c, d, tx, ty))
        w.solve()
        self.assertEqual(w.mode, "affine")
        # Exact at the 3 control points AND at an off-grid 4th point
        # (the full affine was recovered exactly).
        for px, py in pts + [(50000.0, 30000.0)]:
            ex, ey = _affine(px, py, a, b, c, d, tx, ty)
            gx, gy = w.transform(px, py)
            self.assertAlmostEqual(gx, ex, delta=0.5)
            self.assertAlmostEqual(gy, ey, delta=0.5)


class TestInterpolatingWarp(unittest.TestCase):
    # A spread of control points with arbitrary (non-affine) deltas.
    PRED = [
        (0.0, 0.0), (60000.0, 0.0), (0.0, 40000.0),
        (60000.0, 40000.0), (30000.0, 20000.0),
    ]
    DELTAS = [
        (120.0, -45.0), (-80.0, 30.0), (15.0, 95.0),
        (-60.0, -70.0), (200.0, 10.0),
    ]

    def _build(self):
        w = PlateWarpCalibrator()
        for (px, py), (dx, dy) in zip(self.PRED, self.DELTAS):
            w.add_point(px, py, px + dx, py + dy)
        w.solve()
        return w

    def test_mode_is_affine_tps(self):
        self.assertEqual(self._build().mode, "affine_tps")

    def test_exact_at_every_control_point(self):
        w = self._build()
        for (px, py), (dx, dy) in zip(self.PRED, self.DELTAS):
            gx, gy = w.transform(px, py)
            self.assertAlmostEqual(gx, px + dx, delta=0.5,
                                   msg=f"warp not exact at ({px},{py})")
            self.assertAlmostEqual(gy, py + dy, delta=0.5)

    def test_interior_point_finite_and_bounded(self):
        w = self._build()
        gx, gy = w.transform(15000.0, 10000.0)
        self.assertTrue(math.isfinite(gx) and math.isfinite(gy))
        # Interior correction shouldn't blow far past the control deltas.
        self.assertLess(abs(gx - 15000.0), 1000.0)
        self.assertLess(abs(gy - 10000.0), 1000.0)

    def test_raw_delta_reporting(self):
        w = self._build()
        for i, (dx, dy) in enumerate(self.DELTAS):
            self.assertAlmostEqual(w.point_deltas[i][0], dx, delta=1e-6)
            self.assertAlmostEqual(w.point_deltas[i][1], dy, delta=1e-6)
        norms = [math.hypot(dx, dy) for dx, dy in self.DELTAS]
        self.assertAlmostEqual(w.max_correction_um, max(norms), delta=1e-6)
        rms = math.sqrt(sum(n * n for n in norms) / len(norms))
        self.assertAlmostEqual(w.mean_correction_um, rms, delta=1e-6)

    def test_correct_positions_preserves_keys(self):
        w = self._build()
        predicted = {f"W{i}": p for i, p in enumerate(self.PRED)}
        predicted["EXTRA"] = (12345.0, 6789.0)
        out = w.correct_positions(predicted)
        self.assertEqual(set(out.keys()), set(predicted.keys()))
        # Control points land exactly on their measured targets.
        for i, (dx, dy) in enumerate(self.DELTAS):
            px, py = self.PRED[i]
            self.assertAlmostEqual(out[f"W{i}"][0], px + dx, delta=0.5)
            self.assertAlmostEqual(out[f"W{i}"][1], py + dy, delta=0.5)

    def test_to_from_dict_roundtrip(self):
        w = self._build()
        d = w.to_dict()
        w2 = PlateWarpCalibrator.from_dict(d)
        self.assertEqual(w2.mode, w.mode)
        self.assertEqual(w2.n_points, w.n_points)
        for px, py in [(0.0, 0.0), (30000.0, 20000.0), (12345.0, 6789.0)]:
            a = w.transform(px, py)
            b = w2.transform(px, py)
            self.assertAlmostEqual(a[0], b[0], delta=1e-6)
            self.assertAlmostEqual(a[1], b[1], delta=1e-6)


class TestRobustness(unittest.TestCase):
    def test_collinear_points_no_crash(self):
        # 4 collinear control points → affine/TPS ill-posed; must not raise
        # and must stay finite (TPS falls back to affine-only internally).
        w = PlateWarpCalibrator()
        for i in range(4):
            x = i * 10000.0
            w.add_point(x, 0.0, x + 50.0, 0.0 + 25.0)
        w.solve()  # should not raise
        gx, gy = w.transform(15000.0, 5000.0)
        self.assertTrue(math.isfinite(gx) and math.isfinite(gy))

    def test_similarity_approx_recovers_rotation_scale(self):
        ang = math.radians(7.5)
        s, tx, ty = 1.02, 300.0, -200.0
        ca, sa = math.cos(ang) * s, math.sin(ang) * s
        w = PlateWarpCalibrator()
        for px, py in [(0.0, 0.0), (40000.0, 0.0), (0.0, 25000.0)]:
            w.add_point(px, py, ca * px - sa * py + tx, sa * px + ca * py + ty)
        w.solve()
        rot, scale, (rtx, rty) = w.similarity_approx()
        self.assertAlmostEqual(rot, 7.5, delta=0.05)
        self.assertAlmostEqual(scale, 1.02, delta=0.001)


if __name__ == "__main__":
    unittest.main()
