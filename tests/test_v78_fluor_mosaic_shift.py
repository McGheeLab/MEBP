"""test_v78_fluor_mosaic_shift.py — the registration shift must round-trip.

``MosaicBuilder.canvas_extent_um`` ADDS the global registration shift to the
extent it returns, while the tile PIXELS stay in the trusted raw stage frame. So
px → stage-µm back-projection must use ``extent[:2] − shift``. ``MosaicStore``
has always recorded that shift; ``FluorescenceMosaicStore`` did not, which made
every saved fluorescence mosaic unusable as a motion reference (the shift is
bounded by 20 % of the FOV — hundreds of µm at 10×).

These tests pin the round-trip and, crucially, the distinction between "recorded
as zero" and "never recorded" — a legacy entry returns (0, 0) from
``get_shift_um`` and must NOT be mistaken for a registered mosaic.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from SupportClasses.FluorescenceMosaicStore import FluorescenceMosaicStore


def _img(h=40, w=60):
    a = np.zeros((h, w, 3), dtype=np.uint8)
    a[:, :, 1] = 120
    return a


class TestShiftRoundTrip(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = Path(self.dir) / "f.json"
        self.store = FluorescenceMosaicStore(self.path)
        self.ext = (1000.0, 2000.0, 1600.0, 2400.0)

    def test_shift_round_trips(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext,
                                mosaic_scale=0.5, shift_um=(37.5, -12.25))
        self.assertEqual(self.store.get_shift_um("24", "A1"), (37.5, -12.25))
        self.assertTrue(self.store.has_shift("24", "A1"))
        # And after a reload from disk.
        store2 = FluorescenceMosaicStore(self.path)
        self.assertEqual(store2.get_shift_um("24", "A1"), (37.5, -12.25))
        self.assertTrue(store2.has_shift("24", "A1"))

    def test_default_shift_is_recorded_zero(self):
        """A save with no explicit shift still RECORDS (0, 0) — which is a
        different thing from a legacy entry that never recorded one."""
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext)
        self.assertEqual(self.store.get_shift_um("24", "A1"), (0.0, 0.0))
        self.assertTrue(self.store.has_shift("24", "A1"))

    def test_legacy_entry_reads_zero_but_has_no_shift(self):
        """The gate that keeps a pre-v7.8 mosaic from commanding motion."""
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext)
        # Simulate a mosaic written before shift recording existed.
        with open(self.path, encoding="utf-8") as f:
            data = json.load(f)
        data["wells"]["24|A1"]["channels"]["DAPI"].pop("shift_um")
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(data, f)

        legacy = FluorescenceMosaicStore(self.path)
        self.assertEqual(legacy.get_shift_um("24", "A1"), (0.0, 0.0))
        self.assertFalse(legacy.has_shift("24", "A1"))

    def test_malformed_shift_degrades_to_zero(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext)
        with open(self.path, encoding="utf-8") as f:
            data = json.load(f)
        data["wells"]["24|A1"]["channels"]["DAPI"]["shift_um"] = ["nope", None]
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(data, f)
        bad = FluorescenceMosaicStore(self.path)
        self.assertEqual(bad.get_shift_um("24", "A1"), (0.0, 0.0))

    def test_per_channel_shift_and_channel_none_picks_first(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), self.ext,
                                shift_um=(5.0, 6.0))
        self.store.save_channel("24", "A1", "FITC", _img(), self.ext,
                                shift_um=(7.0, 8.0))
        self.assertEqual(self.store.get_shift_um("24", "A1", "FITC"), (7.0, 8.0))
        self.assertEqual(self.store.get_shift_um("24", "A1", "DAPI"), (5.0, 6.0))
        # channel=None → the first stored channel.
        self.assertEqual(self.store.get_shift_um("24", "A1"), (5.0, 6.0))

    def test_missing_well_is_safe(self):
        self.assertEqual(self.store.get_shift_um("24", "Z9"), (0.0, 0.0))
        self.assertFalse(self.store.has_shift("24", "Z9"))


class TestBackProjectionUsesShift(unittest.TestCase):
    """The point of recording the shift: the numbers must actually differ."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.store = FluorescenceMosaicStore(Path(self.dir) / "f.json")

    def test_shift_changes_the_back_projected_position(self):
        from SupportClasses.SpheroidDetector import back_project_px
        extent = (1000.0, 2000.0, 1600.0, 2400.0)
        scale = 0.5   # px per µm
        self.store.save_channel("24", "A1", "DAPI", _img(), extent,
                                mosaic_scale=scale, shift_um=(40.0, -25.0))

        shift = self.store.get_shift_um("24", "A1")
        with_shift = back_project_px((100.0, 50.0), extent, scale, shift)
        naive = back_project_px((100.0, 50.0), extent, scale, (0.0, 0.0))

        # extent[0] - shift[0] + px/scale  =  1000 - 40 + 200 = 1160
        self.assertAlmostEqual(with_shift[0], 1160.0, places=6)
        self.assertAlmostEqual(with_shift[1], 2125.0, places=6)
        # Ignoring the shift is off by exactly the shift — 40 µm here, and up to
        # 20 % of a FOV in reality.
        self.assertAlmostEqual(naive[0] - with_shift[0], 40.0, places=6)
        self.assertAlmostEqual(naive[1] - with_shift[1], -25.0, places=6)


class TestEnvOverride(unittest.TestCase):
    def test_env_redirects_json_and_image_dir(self):
        d = tempfile.mkdtemp()
        target = Path(d) / "nested" / "fluor.json"
        prev = os.environ.get("MEBP_FLUOR_MOSAIC_PATH")
        os.environ["MEBP_FLUOR_MOSAIC_PATH"] = str(target)
        try:
            store = FluorescenceMosaicStore()   # default arg, env wins
            ok = store.save_channel("24", "A1", "DAPI", _img(),
                                    (0.0, 0.0, 60.0, 40.0))
            self.assertTrue(ok)
            self.assertTrue(target.exists())
            # The image dir follows the JSON path, not the repo config dir.
            self.assertTrue((target.parent / "fluor_mosaics").is_dir())
        finally:
            if prev is None:
                os.environ.pop("MEBP_FLUOR_MOSAIC_PATH", None)
            else:
                os.environ["MEBP_FLUOR_MOSAIC_PATH"] = prev

    def test_get_store_with_explicit_path_rebuilds_singleton(self):
        import SupportClasses.FluorescenceMosaicStore as mod
        prev = mod._store_singleton
        try:
            d = tempfile.mkdtemp()
            p = Path(d) / "f.json"
            store = mod.get_store(p)
            self.assertEqual(store._path, p)
            self.assertIs(mod.get_store(), store)
        finally:
            mod._store_singleton = prev


class TestScaleProvenance(unittest.TestCase):
    """A channel's stored scale must be readable per channel, because the two
    are only mutually consistent per channel — ``composite_overlay`` resizes
    later channels to the first one's shape without saying which that was."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.store = FluorescenceMosaicStore(Path(self.dir) / "f.json")

    def test_per_channel_scale_and_um_per_px(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), (0, 0, 600, 400),
                                mosaic_scale=0.10, um_per_px=0.92,
                                objective="10x")
        self.store.save_channel("24", "A1", "FITC", _img(), (0, 0, 600, 400),
                                mosaic_scale=0.25, um_per_px=0.92)
        self.assertAlmostEqual(
            self.store.get_mosaic_scale("24", "A1", "DAPI"), 0.10)
        self.assertAlmostEqual(
            self.store.get_mosaic_scale("24", "A1", "FITC"), 0.25)
        self.assertAlmostEqual(self.store.get_um_per_px("24", "A1"), 0.92)
        self.assertEqual(self.store.get_objective("24", "A1"), "10x")

    def test_absent_scale_returns_none(self):
        self.store.save_channel("24", "A1", "DAPI", _img(), (0, 0, 600, 400))
        self.assertIsNone(self.store.get_mosaic_scale("24", "A1"))
        self.assertIsNone(self.store.get_um_per_px("24", "A1"))
        self.assertEqual(self.store.get_objective("24", "A1"), "")


if __name__ == "__main__":
    unittest.main()
