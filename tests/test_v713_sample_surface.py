"""
test_v713_sample_surface.py — the per-well CRITICAL SAMPLE SURFACE (v7.13).

SupportClasses/SampleSurface.py evaluates a well's mosaic focus survey as a
surface — plane / linear / spline — with confidence "high" only inside BOTH
the well boundary and the sample convex hull (the raster's corner tiles sit
outside the circular well, so the periphery is extrapolation). Also covers
the FluorescenceMosaicStore focus-survey persistence + model choice.

The surface is deliberately NOT the plate bottom: cells commonly sit above
the well bottom (hydrogel), and nothing here installs any datum.
"""

import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from SupportClasses.SampleSurface import (
    CONF_HIGH, CONF_LOW, MIN_SAMPLES, SampleSurfaceModel)


def _samples_plane(a=0.002, b=-0.003, c=500.0, n=4):
    """Grid samples on an exact plane f = a·x + b·y + c."""
    pts = []
    for x in np.linspace(-4000, 4000, n):
        for y in np.linspace(-4000, 4000, n):
            pts.append({"x_um": float(x), "y_um": float(y),
                        "focus_um": float(a * x + b * y + c)})
    return pts


def _samples_dome(c=500.0, amp=40.0, r=6000.0, n=5):
    """A domed surface — plane residual must SHOW it, not flatten it."""
    pts = []
    for x in np.linspace(-4000, 4000, n):
        for y in np.linspace(-4000, 4000, n):
            rr = (x * x + y * y) / (r * r)
            pts.append({"x_um": float(x), "y_um": float(y),
                        "focus_um": float(c + amp * max(0.0, 1.0 - rr))})
    return pts


def _model(samples, model="plane", radius=6000.0):
    return SampleSurfaceModel(samples, well_center_um=(0.0, 0.0),
                              well_radius_um=radius, model=model)


class TestPlaneModel(unittest.TestCase):
    def test_recovers_tilt(self):
        m = _model(_samples_plane(0.004, -0.007, 1234.5))
        sx, sy = m.tilt_mm_per_mm()
        self.assertAlmostEqual(sx, 0.004, places=6)
        self.assertAlmostEqual(sy, -0.007, places=6)
        v, conf = m.evaluate(1000.0, 2000.0)
        self.assertAlmostEqual(v, 0.004 * 1000 - 0.007 * 2000 + 1234.5,
                               places=3)
        self.assertEqual(conf, CONF_HIGH)

    def test_low_confidence_outside_well(self):
        m = _model(_samples_plane())
        _v, conf = m.evaluate(50000.0, 0.0)
        self.assertEqual(conf, CONF_LOW)

    def test_dome_shows_as_residual(self):
        m = _model(_samples_dome(amp=40.0))
        self.assertGreater(m.rms_plane_residual_um(), 5.0)

    def test_span_across_well(self):
        m = _model(_samples_plane(0.01, 0.0, 100.0), radius=5000.0)
        self.assertAlmostEqual(m.span_across_well_um(), 100.0, places=1)

    def test_too_few_samples_raises(self):
        with self.assertRaises(ValueError):
            _model(_samples_plane()[:MIN_SAMPLES - 1])


class TestInterpolatingModels(unittest.TestCase):
    def test_linear_follows_topology_inside_hull(self):
        try:
            import scipy  # noqa: F401
        except ImportError:
            self.skipTest("scipy unavailable")
        dome = _samples_dome(amp=40.0)
        m = _model(dome, model="linear")
        self.assertEqual(m.model(), "linear")
        v, conf = m.evaluate(0.0, 0.0)          # dome apex
        self.assertEqual(conf, CONF_HIGH)
        self.assertGreater(v, 520.0)            # apex ≈ 540, plane ≈ ~515
        # A plane through the same samples under-reads the apex.
        p = _model(dome, model="plane")
        vp, _ = p.evaluate(0.0, 0.0)
        self.assertGreater(v, vp)

    def test_spline_smooth_and_high_conf_in_hull(self):
        try:
            import scipy  # noqa: F401
        except ImportError:
            self.skipTest("scipy unavailable")
        m = _model(_samples_dome(), model="spline")
        self.assertEqual(m.model(), "spline")
        v, conf = m.evaluate(500.0, -500.0)
        self.assertEqual(conf, CONF_HIGH)
        self.assertTrue(np.isfinite(v))

    def test_outside_hull_falls_back_to_plane_low_conf(self):
        # Mutation guard: removing the hull test would report "high"
        # confidence for pure extrapolation at the well edge.
        try:
            import scipy  # noqa: F401
        except ImportError:
            self.skipTest("scipy unavailable")
        m = _model(_samples_plane(n=4), model="linear", radius=8000.0)
        # Inside the well but OUTSIDE the ±4000 sample hull:
        v, conf = m.evaluate(7000.0, 0.0)
        self.assertEqual(conf, CONF_LOW)
        self.assertTrue(np.isfinite(v))         # plane fallback still answers

    def test_interp_needs_enough_samples_else_plane(self):
        m = _model(_samples_plane(n=2), model="spline")   # 4 samples
        self.assertEqual(m.model(), "plane")

    def test_unknown_model_string_becomes_plane(self):
        m = _model(_samples_plane(), model="wavelet")
        self.assertEqual(m.model(), "plane")


class TestStoreFocusSurvey(unittest.TestCase):
    def setUp(self):
        import SupportClasses.FluorescenceMosaicStore as fms
        self._fms = fms
        self._orig = fms._store_singleton
        tmp = Path(tempfile.mkdtemp()) / "fluor.json"
        fms._store_singleton = fms.FluorescenceMosaicStore(tmp)

    def tearDown(self):
        self._fms._store_singleton = self._orig

    def test_round_trip_and_model_persist(self):
        st = self._fms.get_store()
        samples = _samples_plane()
        summary = {"n_accepted": len(samples), "rms_residual_um": 2.5,
                   "well_center_um": (0.0, 0.0), "well_radius_um": 6000.0}
        self.assertTrue(st.set_focus_survey("plateX", "B2", samples,
                                            summary=summary, model="plane"))
        got = st.get_focus_survey("plateX", "B2")
        self.assertEqual(len(got["samples"]), len(samples))
        self.assertEqual(got["model"], "plane")
        self.assertEqual(got["summary"]["n_accepted"], len(samples))
        # Model choice persists independently.
        self.assertTrue(st.set_surface_model("plateX", "B2", "spline"))
        self.assertEqual(st.get_focus_survey("plateX", "B2")["model"],
                         "spline")

    def test_missing_survey_is_none(self):
        st = self._fms.get_store()
        self.assertIsNone(st.get_focus_survey("plateX", "Z9"))
        self.assertFalse(st.set_surface_model("plateX", "Z9", "plane"))

    def test_survey_coexists_with_channels(self):
        st = self._fms.get_store()
        st.set_focus_survey("p", "A1", _samples_plane())
        # A survey on a well without channels must not fabricate channels.
        self.assertEqual(st.list_channels("p", "A1"), [])

    def test_stored_samples_rebuild_a_surface(self):
        st = self._fms.get_store()
        st.set_focus_survey("p", "A1", _samples_plane(0.003, 0.001, 42.0),
                            summary={"well_center_um": (0.0, 0.0),
                                     "well_radius_um": 6000.0})
        got = st.get_focus_survey("p", "A1")
        m = SampleSurfaceModel(got["samples"], well_center_um=(0, 0),
                               well_radius_um=6000.0, model=got["model"])
        v, _ = m.evaluate(1000.0, 1000.0)
        self.assertAlmostEqual(v, 0.003 * 1000 + 0.001 * 1000 + 42.0,
                               places=3)


if __name__ == "__main__":
    unittest.main()
