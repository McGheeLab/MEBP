"""
v7.5.x — NeedleFocusTemplateStore + the needle↔camera-centre offset contract.

Two things are learned from the one needle-centre click during the plate-Z
touch-off: in-focus reference IMAGES (for the "does this look like the in-focus
needle from last time?" feedback) and the needle's OFFSET from the camera centre
(reusable by workflows that put the needle on a clicked feature).

The most important test here is the offset SIGN, pinned as an inverse property
rather than a hard-coded number — a sign error would silently send the needle to
the mirror image of the clicked spot in every pick-and-place workflow.
"""

import json
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from SupportClasses.NeedleFocusTemplateStore import (
    MAX_CAPTURES_PER_KEY,
    NeedleFocusTemplateStore,
    get_store,
    template_key,
)
from SupportClasses.StageController import StageController


KEY = template_key("cam-abc|4x", "22G", 410.0)


def _patch(v=128, w=64, h=64):
    """A textured patch (cv2.imwrite needs real content, not a constant)."""
    a = np.full((h, w, 3), v, dtype=np.uint8)
    a[::4, ::4] = 255
    a[1::7, 2::5] = 0
    return a


class _StoreCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.store = NeedleFocusTemplateStore(
            self.root / "needle_focus_templates.json")

    def tearDown(self):
        self._tmp.cleanup()

    def _add(self, off_um=(120.0, -80.0), **kw):
        kw.setdefault("center_offset_px", (30.0, -20.0))
        kw.setdefault("um_per_px", 1.42)
        kw.setdefault("frame_wh", (1832, 1372))
        kw.setdefault("focus_score", 1800.0)
        return self.store.add_capture(KEY, _patch(), center_offset_um=off_um,
                                      **kw)


class TestRoundTrip(_StoreCase):
    def test_add_and_read_back(self):
        self.assertFalse(self.store.has(KEY))
        self.assertTrue(self._add(well="A1", z_zref_mm=0.09,
                                 stage_um=(105000.0, 68000.0),
                                 needle_type="22G", needle_bore_um=410.0,
                                 needle_tip_length_mm=12.7,
                                 camobj="cam-abc|4x"))
        self.assertTrue(self.store.has(KEY))
        caps = self.store.captures(KEY)
        self.assertEqual(len(caps), 1)
        self.assertEqual(caps[0]["well"], "A1")
        self.assertEqual(caps[0]["patch_wh"], [64, 64])
        self.assertAlmostEqual(caps[0]["z_zref_mm"], 0.09)
        entry = self.store.get(KEY)
        self.assertEqual(entry["needle_type"], "22G")
        self.assertAlmostEqual(entry["needle_tip_length_mm"], 12.7)

    def test_patch_loads_back_as_an_image(self):
        self._add()
        loaded = self.store.load_patches(KEY)
        self.assertEqual(len(loaded), 1)
        meta, img = loaded[0]
        self.assertEqual(img.shape[:2], (64, 64))
        self.assertIn("center_offset_um", meta)

    def test_survives_a_reopen(self):
        self._add(well="A1")
        self._add(well="D6", off_um=(122.0, -78.0))
        fresh = NeedleFocusTemplateStore(
            self.root / "needle_focus_templates.json")
        self.assertEqual(len(fresh.captures(KEY)), 2)
        self.assertEqual([c["well"] for c in fresh.captures(KEY)],
                         ["A1", "D6"])

    def test_keeps_every_well_not_just_the_last(self):
        """The operator asked for a reference at EVERY well, all kept."""
        for i, w in enumerate(("A1", "A6", "D1", "D6")):
            self._add(well=w, off_um=(120.0 + i, -80.0))
        self.assertEqual(len(self.store.captures(KEY)), 4)

    def test_history_is_bounded_and_prunes_images(self):
        for i in range(MAX_CAPTURES_PER_KEY + 4):
            self._add(well=f"W{i}")
        caps = self.store.captures(KEY)
        self.assertEqual(len(caps), MAX_CAPTURES_PER_KEY)
        # Oldest dropped, newest kept.
        self.assertEqual(caps[-1]["well"],
                         f"W{MAX_CAPTURES_PER_KEY + 3}")
        # No orphaned PNGs left behind.
        pngs = list((self.root / "needle_focus_templates").glob("*.png"))
        self.assertEqual(len(pngs), MAX_CAPTURES_PER_KEY)

    def test_clear_removes_metadata_and_images(self):
        self._add()
        self._add()
        self.store.clear(KEY)
        self.assertFalse(self.store.has(KEY))
        self.assertIsNone(self.store.get(KEY))
        self.assertEqual(
            list((self.root / "needle_focus_templates").glob("*.png")), [])

    def test_missing_image_is_not_reported_as_a_capture(self):
        """A deleted PNG must not leave a phantom capture behind."""
        self._add()
        for p in (self.root / "needle_focus_templates").glob("*.png"):
            p.unlink()
        self.assertFalse(self.store.has(KEY))
        self.assertEqual(self.store.captures(KEY), [])
        self.assertIsNone(self.store.needle_center_offset_um(KEY))


class TestValidationAndAtomicity(_StoreCase):
    def test_rejects_empty_or_missing_patch(self):
        self.assertFalse(self.store.add_capture(
            KEY, None, center_offset_px=(0, 0), center_offset_um=(0, 0),
            um_per_px=1.0))
        self.assertFalse(self.store.add_capture(
            KEY, np.zeros((0, 0, 3), dtype=np.uint8),
            center_offset_px=(0, 0), center_offset_um=(0, 0), um_per_px=1.0))

    def test_rejects_malformed_offsets(self):
        self.assertFalse(self.store.add_capture(
            KEY, _patch(), center_offset_px=(1.0,),
            center_offset_um=(1.0, 2.0), um_per_px=1.0))
        self.assertFalse(self.store.add_capture(
            KEY, _patch(), center_offset_px=(1.0, 2.0),
            center_offset_um=None, um_per_px=1.0))

    def test_write_is_atomic_and_leaves_no_tmp(self):
        self._add()
        self.assertTrue((self.root / "needle_focus_templates.json").exists())
        self.assertEqual(list(self.root.glob("*.tmp")), [])

    def test_index_is_valid_json(self):
        self._add()
        with open(self.root / "needle_focus_templates.json",
                  encoding="utf-8") as f:
            data = json.load(f)
        self.assertIn("templates", data)
        self.assertIn(KEY, data["templates"])

    def test_corrupt_index_degrades_to_empty(self):
        p = self.root / "needle_focus_templates.json"
        p.write_text("{not json", encoding="utf-8")
        s = NeedleFocusTemplateStore(p)
        self.assertEqual(s.keys(), [])
        # And is still usable afterwards.
        self.assertTrue(s.add_capture(KEY, _patch(),
                                      center_offset_px=(0.0, 0.0),
                                      center_offset_um=(1.0, 2.0),
                                      um_per_px=1.0))

    def test_key_with_path_characters_cannot_escape_the_image_dir(self):
        """The key reaches a filename, so a traversal attempt must be neutered.

        Asserts the real invariant (the written file stays inside the image
        directory) rather than the absence of a ".." substring — separators are
        stripped, so ".." can only survive as ordinary filename characters, which
        is harmless.
        """
        weird = "cam/../../x|30G:30"
        img_dir = (self.root / "needle_focus_templates").resolve()
        self.assertTrue(self.store.add_capture(
            weird, _patch(), center_offset_px=(0.0, 0.0),
            center_offset_um=(0.0, 0.0), um_per_px=1.0))
        pngs = list(img_dir.glob("*.png"))
        self.assertEqual(len(pngs), 1)
        for p in pngs:
            self.assertEqual(p.resolve().parent, img_dir)
            for sep in ("/", "\\"):
                self.assertNotIn(sep, p.name)
        self.assertTrue(self.store.has(weird))


class TestDerivedValues(_StoreCase):
    def test_offset_is_the_mean_across_captures(self):
        self._add(off_um=(100.0, -50.0))
        self._add(off_um=(110.0, -60.0))
        off = self.store.needle_center_offset_um(KEY)
        self.assertAlmostEqual(off[0], 105.0)
        self.assertAlmostEqual(off[1], -55.0)

    def test_spread_flags_a_misclick(self):
        self._add(off_um=(100.0, -50.0))
        self._add(off_um=(102.0, -51.0))
        tight = self.store.needle_center_offset_spread_um(KEY)
        self.assertLess(tight, 5.0)
        self._add(off_um=(900.0, -50.0))         # clearly a mis-click
        self.assertGreater(self.store.needle_center_offset_spread_um(KEY),
                           100.0)

    def test_spread_none_with_one_capture(self):
        self._add()
        self.assertIsNone(self.store.needle_center_offset_spread_um(KEY))

    def test_reference_focus_is_the_median(self):
        for sc in (1000.0, 1800.0, 1900.0):
            self._add(focus_score=sc)
        self.assertAlmostEqual(self.store.reference_focus_score(KEY), 1800.0)

    def test_reference_focus_none_when_unscored(self):
        self._add(focus_score=None)
        self.assertIsNone(self.store.reference_focus_score(KEY))

    def test_summary_mentions_count_and_offset(self):
        self._add(off_um=(123.0, -45.0))
        s = self.store.summary(KEY)
        self.assertIn("1 reference", s)
        self.assertIn("123", s)
        self.assertIn("no in-focus needle reference stored",
                      self.store.summary("other-key"))


class TestKeyScheme(unittest.TestCase):
    def test_key_includes_camera_objective_and_needle(self):
        """camobj already carries "<camera>|<objective>"; the needle is appended."""
        k = template_key("cam-1|10x", "30G", 159.0)
        self.assertEqual(k, "cam-1|10x|30G:159")

    def test_different_needles_do_not_share_a_key(self):
        """A 22G and a 30G look nothing alike — they must not share a template."""
        self.assertNotEqual(template_key("c|4x", "22G", 410.0),
                            template_key("c|4x", "30G", 159.0))

    def test_different_objectives_do_not_share_a_key(self):
        self.assertNotEqual(template_key("c|4x", "22G", 410.0),
                            template_key("c|10x", "22G", 410.0))

    def test_missing_needle_info_still_yields_a_key(self):
        self.assertEqual(template_key("c|4x"), "c|4x|needle:0")


class TestEnvIsolationAndSingleton(unittest.TestCase):
    def test_env_var_redirects_the_store(self):
        with tempfile.TemporaryDirectory() as d:
            old = os.environ.get("MEBP_NEEDLE_TEMPLATE_DIR")
            os.environ["MEBP_NEEDLE_TEMPLATE_DIR"] = d
            try:
                s = NeedleFocusTemplateStore()
                s.add_capture(KEY, _patch(), center_offset_px=(0.0, 0.0),
                              center_offset_um=(1.0, 2.0), um_per_px=1.0)
                self.assertTrue(
                    (Path(d) / "needle_focus_templates.json").exists())
            finally:
                if old is None:
                    os.environ.pop("MEBP_NEEDLE_TEMPLATE_DIR", None)
                else:
                    os.environ["MEBP_NEEDLE_TEMPLATE_DIR"] = old

    def test_explicit_path_replaces_the_singleton(self):
        with tempfile.TemporaryDirectory() as d:
            a = get_store(Path(d) / "a.json")
            self.assertIs(get_store(), a)
            b = get_store(Path(d) / "b.json")
            self.assertIsNot(a, b)


class TestOffsetSignContract(unittest.TestCase):
    """The sign, pinned as an inverse property rather than a magic number."""

    def _ctrl(self, off=None):
        c = StageController.__new__(StageController)
        if off is not None:
            c._needle_cam_offset_um = off
        return c

    def test_target_then_offset_returns_the_feature(self):
        """feature == needle_target_xy_for_feature_um(feature) + offset."""
        off = (123.4, -56.7)
        c = self._ctrl(off)
        for feature in ((0.0, 0.0), (105000.0, 68000.0), (-500.0, 900.0)):
            tx, ty = c.needle_target_xy_for_feature_um(*feature)
            self.assertAlmostEqual(tx + off[0], feature[0], places=9)
            self.assertAlmostEqual(ty + off[1], feature[1], places=9)

    def test_no_offset_is_a_passthrough(self):
        """Callers can adopt this unconditionally and behave as today."""
        c = self._ctrl()
        self.assertEqual(c.get_needle_camera_offset_um(), None)
        self.assertEqual(c.needle_target_xy_for_feature_um(10.0, 20.0),
                         (10.0, 20.0))

    def test_set_and_get_round_trip(self):
        c = self._ctrl()
        c.set_needle_camera_offset_um(12.0, -34.0)
        self.assertEqual(c.get_needle_camera_offset_um(), (12.0, -34.0))

    def test_clear_with_none(self):
        c = self._ctrl((1.0, 2.0))
        c.set_needle_camera_offset_um(None)
        self.assertIsNone(c.get_needle_camera_offset_um())
        self.assertEqual(c.needle_target_xy_for_feature_um(5.0, 6.0),
                         (5.0, 6.0))

    def test_malformed_offset_is_ignored_not_fatal(self):
        c = self._ctrl()
        c.set_needle_camera_offset_um("x", "y")
        self.assertIsNone(c.get_needle_camera_offset_um())

    def test_offset_moves_the_target_opposite_to_the_needle_offset(self):
        """If the needle sits +X of the crosshair, the stage must go -X."""
        c = self._ctrl((100.0, 0.0))
        tx, _ = c.needle_target_xy_for_feature_um(1000.0, 0.0)
        self.assertAlmostEqual(tx, 900.0)


if __name__ == "__main__":
    unittest.main()
