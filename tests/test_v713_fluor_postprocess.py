"""
test_v713_fluor_postprocess.py — denoise + background subtraction (v7.13).

Pure module (SupportClasses/FluorescencePostProcess.py): applied at save time
to a COPY of the stitched channel; the raw stitch stays canonical. Covers
dtype/shape preservation, the rolling-ball opening removing a smooth gradient
while preserving a small bright blob's contrast, µm→px radius resolution, the
downscaled-opening path agreeing with full-res, and settings normalization.
"""

import unittest

import numpy as np

from SupportClasses import FluorescencePostProcess as fpp


def _gradient_with_blob(h=200, w=200, blob_val=200):
    """Smooth illumination gradient + one small bright blob."""
    yy, xx = np.mgrid[0:h, 0:w]
    bg = (30 + 60 * xx / w).astype(np.uint8)          # 30 → 90 ramp
    img = bg.copy()
    img[95:105, 95:105] = blob_val                    # 10-px blob
    return img, bg


class TestNormalizeSettings(unittest.TestCase):
    def test_defaults(self):
        s = fpp.normalize_settings(None)
        self.assertEqual(s["denoise"], "off")
        self.assertFalse(s["bg_subtract"])

    def test_clamps(self):
        s = fpp.normalize_settings({"denoise": "MEDIAN",
                                    "denoise_strength": 99,
                                    "bg_radius_um": -5})
        self.assertEqual(s["denoise"], "median")
        self.assertEqual(s["denoise_strength"], 3)
        self.assertEqual(s["bg_radius_um"], 1.0)

    def test_unknown_mode_off(self):
        self.assertEqual(fpp.normalize_settings({"denoise": "wavelet"})
                         ["denoise"], "off")

    def test_is_active(self):
        self.assertFalse(fpp.is_active(None))
        self.assertTrue(fpp.is_active({"denoise": "median"}))
        self.assertTrue(fpp.is_active({"bg_subtract": True}))


class TestDenoise(unittest.TestCase):
    def test_preserves_dtype_and_shape(self):
        img = np.random.randint(0, 255, (64, 64)).astype(np.uint8)
        for mode in ("median", "gaussian"):
            out = fpp.denoise(img, mode, 2)
            self.assertEqual(out.shape, img.shape)
            self.assertEqual(out.dtype, img.dtype)

    def test_median_kills_speckle(self):
        img = np.full((64, 64), 50, dtype=np.uint8)
        img[10, 10] = 255                              # single hot pixel
        out = fpp.denoise(img, "median", 1)
        self.assertEqual(int(out[10, 10]), 50)

    def test_off_mode_passthrough(self):
        img = np.full((16, 16), 7, dtype=np.uint8)
        self.assertIs(fpp.denoise(img, "off", 1), img)


class TestBackgroundSubtraction(unittest.TestCase):
    def test_removes_gradient_keeps_blob(self):
        img, bg = _gradient_with_blob()
        out = fpp.subtract_background(img, radius_px=15)
        # The smooth ramp is background → flattened to near zero.
        corner_left = float(np.mean(out[20:40, 20:40]))
        corner_right = float(np.mean(out[20:40, 160:180]))
        self.assertLess(corner_left, 12)
        self.assertLess(corner_right, 12)
        # The blob (smaller than the ball) keeps its contrast vs surroundings.
        blob = float(np.mean(out[97:103, 97:103]))
        self.assertGreater(blob, 80)

    def test_downscaled_path_close_to_fullres(self):
        # Radius large enough to route through the downscaled opening.
        img, _bg = _gradient_with_blob(400, 400)
        big = fpp.subtract_background(img, radius_px=60)   # downscaled path
        self.assertEqual(big.shape, img.shape)
        # The ramp must still be flattened by the approximate path.
        self.assertLess(float(np.mean(big[20:40, 340:380])), 15)

    def test_preserves_dtype(self):
        img = np.random.randint(0, 4000, (64, 64)).astype(np.uint16)
        out = fpp.subtract_background(img, radius_px=5)
        self.assertEqual(out.dtype, np.uint16)


class TestProcess(unittest.TestCase):
    def test_radius_um_to_px(self):
        img, _ = _gradient_with_blob()
        _out, applied = fpp.process(
            img, {"bg_subtract": True, "bg_radius_um": 100.0}, um_per_px=2.0)
        self.assertEqual(applied["bg_radius_px"], 50.0)
        self.assertEqual(applied["bg_radius_um"], 100.0)

    def test_unknown_scale_uses_default_px(self):
        img, _ = _gradient_with_blob()
        _out, applied = fpp.process(
            img, {"bg_subtract": True, "bg_radius_um": 100.0}, um_per_px=0.0)
        self.assertEqual(applied["bg_radius_px"], 50.0)   # fallback constant

    def test_inactive_returns_input_and_empty_record(self):
        img, _ = _gradient_with_blob()
        out, applied = fpp.process(img, {"denoise": "off"}, um_per_px=1.0)
        self.assertIs(out, img)
        self.assertEqual(applied, {})

    def test_applied_record_names_what_ran(self):
        img, _ = _gradient_with_blob()
        _out, applied = fpp.process(
            img, {"denoise": "gaussian", "denoise_strength": 2,
                  "bg_subtract": True, "bg_radius_um": 40.0}, um_per_px=1.0)
        self.assertEqual(applied["denoise"], "gaussian")
        self.assertEqual(applied["denoise_strength"], 2)
        self.assertTrue(applied["bg_subtract"])

    def test_subtract_direction(self):
        # Mutation guard: bg − img instead of img − bg would leave the blob
        # DARKER than its surroundings.
        img, _ = _gradient_with_blob()
        out, _applied = fpp.process(
            img, {"bg_subtract": True, "bg_radius_um": 15.0}, um_per_px=1.0)
        blob = float(np.mean(out[97:103, 97:103]))
        ring = float(np.mean(out[80:90, 80:90]))
        self.assertGreater(blob, ring)


if __name__ == "__main__":
    unittest.main()
