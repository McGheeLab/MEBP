"""v7.5.x tests — mosaic memory + overlay-paint performance (items 1 & 2).

Item 1 (per-tile slowdown + RAM at 80–90% past ~300 tiles): the full-plate
MosaicBuilder retained every raw frame in ``_records`` (~½ GB) plus the float64
accumulators for the whole scan → swap thrash. Fix: frame-light mode frees each
frame after it's blended, and ``free_accumulators`` releases the float64 buffers
once the scan completes (keeping the uint8 display cache).

Item 2 (post-build lag): the overlay re-scaled the full mosaic pixmap with a 180°
transform on every paint. Fix: cache the scaled (+rotated) pixmap, keyed by
on-screen size + flip + source — reused across pan/needle-motion repaints.
"""
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

from SupportClasses.MosaicBuilder import MosaicBuilder


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


def _build(retain):
    b = MosaicBuilder(frame_size_px=(100, 100), micron_per_pixel=2.0,
                      overlap=0.1, target_mosaic_px=400, retain_frames=retain)
    b.generate_raster_positions((0, 0, 300, 300))
    for i, (x, y) in enumerate([(50, 50), (120, 50), (190, 50), (50, 120)]):
        fr = (np.random.rand(100, 100, 3) * 255).astype(np.uint8)
        b.add_raster_frame(fr, x, y, index=i)
        b.stitch_incremental()
    return b


class TestFrameLightMode(_QtBase):
    def test_frame_light_frees_frames_keeps_composite(self):
        b = _build(retain=False)
        self.assertTrue(all(r.frame is None for r in b._records))
        self.assertIsNotNone(b.composite)                 # display cache intact
        self.assertEqual(b.composite.shape[2], 3)

    def test_retain_default_keeps_frames(self):
        b = _build(retain=True)
        self.assertTrue(all(r.frame is not None for r in b._records))

    def test_free_accumulators_releases_float64_keeps_display(self):
        b = _build(retain=False)
        self.assertIsNotNone(b._composite)
        self.assertIsNotNone(b._weight_sum)
        b.free_accumulators()
        self.assertIsNone(b._composite)
        self.assertIsNone(b._weight_sum)
        self.assertIsNotNone(b.composite)                 # display cache kept

    def test_tile_images_px_skips_freed_frames(self):
        self.assertEqual(len(_build(retain=False).tile_images_px()), 0)
        self.assertEqual(len(_build(retain=True).tile_images_px()), 4)


class TestOverlayCache(_QtBase):
    def _view(self):
        from gui.widgets.jog_workspace_view import JogWorkspaceView, pixmap_from_bgr
        from SupportClasses.SafetyLimits import SafetyLimits
        v = JogWorkspaceView()
        sl = SafetyLimits()
        sl.xy_min_x, sl.xy_max_x = 0, 20000
        sl.xy_min_y, sl.xy_max_y = 0, 16000
        if hasattr(v, "set_safety_limits"):
            v.set_safety_limits(sl)
        v.resize(600, 480)
        img = (np.random.rand(800, 1000, 3) * 255).astype(np.uint8)
        v.set_mosaic_overlay(pixmap_from_bgr(img), (0.0, 0.0, 20000.0, 16000.0))
        v.set_mosaic_visible(True)
        return v

    def test_cache_built_and_reused(self):
        v = self._view()
        self.assertIsNone(v._mosaic_cache_key)   # cleared on set_mosaic_overlay
        v.grab()                                  # first paint builds the cache
        key = v._mosaic_cache_key
        self.assertIsNotNone(key)
        self.assertIsNotNone(v._mosaic_scaled_cache)
        v.grab()
        self.assertEqual(v._mosaic_cache_key, key)   # reused on repaint

    def test_cache_reused_after_pan(self):
        v = self._view()
        v.grab()
        key = v._mosaic_cache_key
        v.set_mosaic_shift(800.0, -400.0)            # pan only — size invariant
        v.grab()
        self.assertEqual(v._mosaic_cache_key, key)

    def test_cache_invalidated_on_new_overlay(self):
        from gui.widgets.jog_workspace_view import pixmap_from_bgr
        v = self._view()
        v.grab()
        self.assertIsNotNone(v._mosaic_cache_key)
        img = (np.random.rand(800, 1000, 3) * 255).astype(np.uint8)
        v.set_mosaic_overlay(pixmap_from_bgr(img), (0.0, 0.0, 20000.0, 16000.0))
        self.assertIsNone(v._mosaic_cache_key)


if __name__ == "__main__":
    unittest.main()
