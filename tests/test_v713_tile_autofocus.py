"""
test_v713_tile_autofocus.py — the focus-drive executor for mosaic AF (v7.13).

Drives the REAL TileAutofocus against a fake scope (silently-clamping
set_focus_um, stale-drop simulation) and a fake camera whose sharpness is a
Gaussian of |focus − true_z| — the same fixture idea the v7.11 plate-level
suites use. No hardware, no Qt event loop.
"""

import math
import threading
import unittest
from types import SimpleNamespace

import numpy as np

from SupportClasses.TileAutofocus import (
    AfAbort, AfCancelled, TileAutofocus, AUTO_HALF_RANGE_DOF, AUTO_STEP_DOF,
    MIN_SWEEP_SAMPLES)


class _Op:
    def __init__(self, error=None):
        self.done = threading.Event()
        self.done.set()
        self.error = error


class FakeScope:
    """set_focus_um clamps SILENTLY to [lo, hi] — like the real Ti."""

    def __init__(self, focus=1000.0, lo=0.0, hi=20000.0):
        self.focus = float(focus)
        self.lo, self.hi = float(lo), float(hi)
        self.moves: list = []
        self.stale = False

    def set_focus_um(self, z):
        if self.stale:
            return _Op(error="dropped (stale)")
        z = max(self.lo, min(self.hi, float(z)))
        self.focus = z
        self.moves.append(z)
        return _Op()

    def state(self):
        return SimpleNamespace(focus_um=self.focus)


class FakeFocusCam:
    """Sharpness is a Gaussian of the scope's distance from ``true_z``.

    The frame is a checkerboard whose contrast scales with sharpness — the
    Laplacian-based focus metric then reproduces a clean through-focus curve
    with FWHM ≈ 2×DOF for sigma_um ≈ 1.2×DOF. Pixel max kept < 254 so the
    SATURATED refusal cannot fire.
    """

    def __init__(self, scope, true_z=1000.0, sigma_um=12.0, flat=False):
        self._scope = scope
        self.true_z = float(true_z)
        self._sigma = float(sigma_um)
        self._flat = flat
        self._n = 0
        base = np.indices((64, 64)).sum(axis=0) % 2
        self._checker = base.astype(np.float32)
        self._rng = np.random.default_rng(3)

    def frame_count_value(self):
        self._n += 1
        return self._n

    def get_current_frame(self):
        if self._flat:
            # Empty glass: noise only, no focusable structure.
            img = 40 + self._rng.normal(0, 1.0, size=(64, 64))
            gray = np.clip(img, 0, 250).astype(np.uint8)
        else:
            d = self._scope.focus - self.true_z
            sharp = math.exp(-(d * d) / (2.0 * self._sigma * self._sigma))
            amp = 10.0 + 180.0 * sharp
            img = (40 + self._checker * amp
                   + self._rng.normal(0, 0.5, (64, 64)))
            gray = np.clip(img, 0, 250).astype(np.uint8)
        # BGR, like every production frame (the mosaic builder blends 3-ch).
        return np.repeat(gray[:, :, None], 3, axis=2)


def _af(scope, cam, dof=10.0, **kw):
    kw.setdefault("settle_s", 0.0)
    return TileAutofocus(scope=scope, cam=cam, dof_um=dof, **kw)


class TestGeometry(unittest.TestCase):
    def test_auto_sizing_from_dof(self):
        af = _af(FakeScope(), FakeFocusCam(FakeScope()), dof=8.0)
        self.assertAlmostEqual(af.step_um(), AUTO_STEP_DOF * 8.0)
        self.assertAlmostEqual(af.half_range_um(),
                               max(AUTO_HALF_RANGE_DOF * 8.0,
                                   af.step_um() * (MIN_SWEEP_SAMPLES - 1) / 2))

    def test_user_step_and_range_honored(self):
        scope = FakeScope(focus=1000.0)
        cam = FakeFocusCam(scope, true_z=1000.0)
        af = _af(scope, cam, dof=10.0, step_um=5.0, half_range_um=25.0)
        self.assertEqual(af.step_um(), 5.0)
        self.assertEqual(af.half_range_um(), 25.0)
        af.micro_sweep(1000.0)
        # Sample moves (excluding the backlash lead-in) span ±25 at 5 µm.
        zs = [z for z in scope.moves if 975.0 <= z <= 1025.0 + 1e-6]
        self.assertIn(975.0, zs)
        self.assertIn(1025.0, zs)

    def test_narrow_user_range_widened_for_min_samples(self):
        # A 1 µm half-range at a 10 µm step cannot hold 5 samples — the range
        # widens rather than silently under-sampling (which would surface as
        # INSUFFICIENT_SAMPLES = "AF broken").
        af = _af(FakeScope(), FakeFocusCam(FakeScope()), dof=10.0,
                 step_um=10.0, half_range_um=1.0)
        self.assertGreaterEqual(
            af.half_range_um(), 10.0 * (MIN_SWEEP_SAMPLES - 1) / 2)


class TestMicroSweep(unittest.TestCase):
    def test_finds_peak_within_one_dof(self):
        scope = FakeScope(focus=1005.0)
        cam = FakeFocusCam(scope, true_z=1000.0, sigma_um=12.0)
        af = _af(scope, cam, dof=10.0)
        peak, why = af.micro_sweep(1005.0)
        self.assertIsNotNone(peak, why)
        self.assertLess(abs(peak.z_um - 1000.0), 10.0)

    def test_empty_glass_refused_low_prominence(self):
        scope = FakeScope(focus=1000.0)
        cam = FakeFocusCam(scope, flat=True)
        af = _af(scope, cam, dof=10.0)
        peak, why = af.micro_sweep(1000.0)
        self.assertIsNone(peak)
        self.assertTrue(why)     # a named refusal, not a silent None

    def test_clamped_readback_abandons_sweep(self):
        # Sweep centred above the drive's ceiling: every target clamps, the
        # readback disagrees, no samples collect → a refusal, never a "peak".
        scope = FakeScope(focus=990.0, hi=1000.0)
        cam = FakeFocusCam(scope, true_z=990.0)
        af = _af(scope, cam, dof=10.0)
        peak, why = af.micro_sweep(1500.0)
        self.assertIsNone(peak)

    def test_stale_drop_raises_afabort(self):
        scope = FakeScope()
        scope.stale = True
        af = _af(scope, FakeFocusCam(scope), dof=10.0)
        with self.assertRaises(AfAbort) as cm:
            af.micro_sweep(1000.0)
        self.assertIn("driving the microscope", str(cm.exception))

    def test_should_stop_cancels(self):
        scope = FakeScope()
        af = _af(scope, FakeFocusCam(scope), dof=10.0)
        with self.assertRaises(AfCancelled):
            af.micro_sweep(1000.0, should_stop=lambda: True)

    def test_lead_in_approaches_from_below(self):
        scope = FakeScope(focus=1000.0)
        af = _af(scope, FakeFocusCam(scope), dof=10.0)
        af.micro_sweep(1000.0)
        # First recorded move is the backlash lead-in BELOW the first target.
        self.assertLess(scope.moves[0], scope.moves[1])


class TestCoarseSolve(unittest.TestCase):
    def _optics(self):
        from SupportClasses.ObjectiveOptics import ObjectiveOptics
        return ObjectiveOptics(label="10x test", numerical_aperture=0.30,
                               working_distance_mm=16.0,
                               um_per_px_sample=1.0, frame_wh=(64, 64))

    def test_recovers_misfocus(self):
        scope = FakeScope(focus=1080.0)
        cam = FakeFocusCam(scope, true_z=1000.0, sigma_um=12.0)
        af = _af(scope, cam, dof=10.0)
        peak, why = af.coarse_solve(self._optics(), 1080.0, 150.0)
        self.assertIsNotNone(peak, why)
        self.assertLess(abs(peak.z_um - 1000.0), 10.0)

    def test_refuses_with_incomplete_optics(self):
        from SupportClasses.ObjectiveOptics import ObjectiveOptics
        scope = FakeScope()
        af = _af(scope, FakeFocusCam(scope), dof=10.0)
        peak, why = af.coarse_solve(
            ObjectiveOptics(label="x", numerical_aperture=None,
                            um_per_px_sample=1.0), 1000.0, 100.0)
        self.assertIsNone(peak)
        self.assertTrue(why)


if __name__ == "__main__":
    unittest.main()
