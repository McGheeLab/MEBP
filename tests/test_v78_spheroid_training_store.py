"""test_v78_spheroid_training_store.py — banked spheroid crops + crop geometry.

Pure: no Qt, no camera. The geometry half matters most — a mislabelled training
sample is worse than a missing one, so the refusals (stage not on the target, a
circle clipped by the frame edge) are pinned as hard as the happy path.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from SupportClasses.SpheroidTrainingStore import (
    DEFAULT_PAD_FRAC, SOURCE_MANUAL, SOURCE_REDRAWN, SpheroidTrainingStore,
    crop_from_frame, crop_rect_for_circle, refuse_crop_reason,
    stage_is_on_target, target_center_px,
)


def _frame(h=480, w=640, value=30):
    a = np.full((h, w, 3), value, dtype=np.uint8)
    # A gradient so a crop can be told apart from a blank array.
    a[:, :, 0] = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
    return a


class TestTargetProjection(unittest.TestCase):
    FRAME = (640, 480)

    def test_target_at_stage_centre_is_frame_centre(self):
        cx, cy = target_center_px((1000.0, 2000.0), (1000.0, 2000.0), 2.0,
                                  self.FRAME)
        self.assertAlmostEqual(cx, 320.0)
        self.assertAlmostEqual(cy, 240.0)

    def test_offset_scales_by_um_per_px(self):
        # 200 µm right of the stage centre at 2 µm/px = 100 px right of centre.
        cx, cy = target_center_px((1200.0, 2000.0), (1000.0, 2000.0), 2.0,
                                  self.FRAME)
        self.assertAlmostEqual(cx, 420.0)
        self.assertAlmostEqual(cy, 240.0)

    def test_zero_um_per_px_raises(self):
        with self.assertRaises(ValueError):
            target_center_px((0.0, 0.0), (0.0, 0.0), 0.0, self.FRAME)


class TestCropRect(unittest.TestCase):
    FRAME = (640, 480)

    def test_centred_circle_gives_a_symmetric_rect(self):
        rect = crop_rect_for_circle((320.0, 240.0), 40.0, self.FRAME,
                                    pad_frac=0.0)
        self.assertEqual((rect.x0, rect.y0, rect.x1, rect.y1),
                         (280, 200, 360, 280))
        self.assertEqual(rect.width, 80)
        self.assertEqual(rect.height, 80)
        self.assertAlmostEqual(rect.center_px_in_crop[0], 40.0)
        self.assertAlmostEqual(rect.center_px_in_crop[1], 40.0)
        self.assertFalse(rect.clipped)
        self.assertFalse(rect.circle_clipped)

    def test_pad_frac_grows_the_window_by_a_fraction_of_the_radius(self):
        rect = crop_rect_for_circle((320.0, 240.0), 40.0, self.FRAME,
                                    pad_frac=0.25)
        # half = 40 * 1.25 = 50
        self.assertEqual((rect.x0, rect.x1), (270, 370))
        self.assertAlmostEqual(rect.center_px_in_crop[0], 50.0)

    def test_default_pad_is_the_documented_constant(self):
        a = crop_rect_for_circle((320.0, 240.0), 40.0, self.FRAME)
        b = crop_rect_for_circle((320.0, 240.0), 40.0, self.FRAME,
                                 pad_frac=DEFAULT_PAD_FRAC)
        self.assertEqual((a.x0, a.y0, a.x1, a.y1), (b.x0, b.y0, b.x1, b.y1))

    def test_padding_clipped_at_the_edge_keeps_the_centre_correct(self):
        """The window is clamped, never black-padded, and the recorded centre is
        relative to the CLAMPED origin so it still points at the spheroid."""
        rect = crop_rect_for_circle((20.0, 240.0), 30.0, self.FRAME,
                                    pad_frac=0.5)
        self.assertEqual(rect.x0, 0)
        self.assertTrue(rect.clipped)
        # Padding hit the edge but the circle itself (20 ± 30 → -10) also did.
        self.assertTrue(rect.circle_clipped)
        self.assertAlmostEqual(rect.center_px_in_crop[0], 20.0)

    def test_margin_clipped_but_circle_intact_is_flagged_separately(self):
        """The distinction that lets a trimmed margin through while refusing a
        trimmed spheroid."""
        # centre 35, r 30 → circle spans 5..65 (inside); padded spans -10..80.
        rect = crop_rect_for_circle((35.0, 240.0), 30.0, self.FRAME,
                                    pad_frac=0.5)
        self.assertTrue(rect.clipped)
        self.assertFalse(rect.circle_clipped)

    def test_circle_past_the_right_edge_is_flagged(self):
        rect = crop_rect_for_circle((630.0, 240.0), 30.0, self.FRAME,
                                    pad_frac=0.0)
        self.assertTrue(rect.circle_clipped)
        self.assertEqual(rect.x1, 640)

    def test_fully_outside_frame_is_empty(self):
        rect = crop_rect_for_circle((-500.0, 240.0), 20.0, self.FRAME)
        self.assertTrue(rect.is_empty())


class TestStageOnTarget(unittest.TestCase):
    FRAME = (640, 480)   # at 2 µm/px = 1280 x 960 µm FOV

    def test_target_at_centre_passes(self):
        self.assertTrue(stage_is_on_target((1000.0, 2000.0), (1000.0, 2000.0),
                                           2.0, self.FRAME))

    def test_target_inside_half_a_fov_passes(self):
        # Half-FOV tolerance in x = 640/2 * 2 * 0.5 = 320 µm.
        self.assertTrue(stage_is_on_target((1300.0, 2000.0), (1000.0, 2000.0),
                                           2.0, self.FRAME))

    def test_target_beyond_half_a_fov_fails(self):
        self.assertFalse(stage_is_on_target((1400.0, 2000.0), (1000.0, 2000.0),
                                            2.0, self.FRAME))

    def test_uncalibrated_camera_fails_closed(self):
        self.assertFalse(stage_is_on_target((1000.0, 2000.0), (1000.0, 2000.0),
                                            0.0, self.FRAME))


class TestRefuseCropReason(unittest.TestCase):
    FRAME = (640, 480)
    STAGE = (1000.0, 2000.0)

    def test_ok_returns_none(self):
        self.assertIsNone(refuse_crop_reason(
            self.STAGE, self.STAGE, 2.0, self.FRAME, 40.0))

    def test_no_um_per_px(self):
        msg = refuse_crop_reason(self.STAGE, self.STAGE, 0.0, self.FRAME, 40.0)
        self.assertIn("µm/px", msg)

    def test_no_diameter(self):
        msg = refuse_crop_reason(self.STAGE, self.STAGE, 2.0, self.FRAME, 0.0)
        self.assertIn("no measured diameter", msg)

    def test_stage_not_on_target_is_refused(self):
        """The guard against a crop of somewhere else wearing this diameter."""
        msg = refuse_crop_reason((9000.0, 2000.0), self.STAGE, 2.0,
                                 self.FRAME, 40.0)
        self.assertIsNotNone(msg)
        self.assertIn("not on this spheroid", msg)

    # A target 300 µm right of the stage at 2 µm/px sits at cx = 470, comfortably
    # inside the half-FOV on-target tolerance; a 180 px radius then still runs
    # past the 640 px right edge. That isolates the clipping check from the
    # on-target check, which is evaluated first (a wrong LOCATION is the more
    # serious objection, so it is reported first).
    BIG_SPHEROID = ((1300.0, 2000.0), 180.0)

    def test_clipped_circle_is_refused_by_default(self):
        target, radius = self.BIG_SPHEROID
        msg = refuse_crop_reason(target, self.STAGE, 2.0, self.FRAME, radius)
        self.assertIsNotNone(msg)
        self.assertIn("edge of the frame", msg)

    def test_clipped_circle_can_be_allowed_explicitly(self):
        target, radius = self.BIG_SPHEROID
        self.assertIsNone(refuse_crop_reason(
            target, self.STAGE, 2.0, self.FRAME, radius,
            skip_if_circle_clipped=False))

    def test_off_target_is_reported_before_clipping(self):
        """Order matters: a crop of the wrong place is worse than a trimmed one."""
        msg = refuse_crop_reason((9000.0, 2000.0), self.STAGE, 2.0,
                                 self.FRAME, 180.0)
        self.assertIn("not on this spheroid", msg)


class TestCropFromFrame(unittest.TestCase):
    def test_crop_matches_the_rect_and_is_a_copy(self):
        frame = _frame()
        rect = crop_rect_for_circle((320.0, 240.0), 40.0, (640, 480),
                                    pad_frac=0.0)
        crop = crop_from_frame(frame, rect)
        self.assertEqual(crop.shape[:2], (80, 80))
        np.testing.assert_array_equal(crop, frame[200:280, 280:360])
        crop[0, 0, 0] = 7
        self.assertNotEqual(int(frame[200, 280, 0]), 7)

    def test_empty_rect_returns_none(self):
        rect = crop_rect_for_circle((-500.0, 240.0), 20.0, (640, 480))
        self.assertIsNone(crop_from_frame(_frame(), rect))

    def test_none_frame_returns_none(self):
        rect = crop_rect_for_circle((320.0, 240.0), 40.0, (640, 480))
        self.assertIsNone(crop_from_frame(None, rect))


class TestStore(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = Path(self.dir) / "spheroid_training.json"
        self.store = SpheroidTrainingStore(self.path)

    def _add(self, store=None, **kw):
        store = store or self.store
        args = dict(
            diameter_um=210.0, radius_px=52.5, um_per_px=2.0,
            center_px_in_crop=(65.0, 65.0), crop_origin_px=(255, 175),
            frame_wh=(640, 480), stage_um=(1000.0, 2000.0),
            well="A1", plate_key="24", objective="10x", channel="DAPI",
            target_id="P001")
        args.update(kw)
        return store.add_crop(np.zeros((130, 130, 3), dtype=np.uint8), **args)

    def test_writes_png_and_metadata(self):
        rel = self._add()
        self.assertIsNotNone(rel)
        self.assertTrue(rel.startswith("spheroid_training/"))
        self.assertEqual(self.store.count(), 1)
        s = self.store.samples()[0]
        self.assertAlmostEqual(s["diameter_um"], 210.0)
        self.assertAlmostEqual(s["um_per_px"], 2.0)
        self.assertEqual(s["frame_wh"], [640, 480])
        self.assertEqual(s["crop_wh"], [130, 130])
        self.assertEqual(s["well"], "A1")
        self.assertEqual(s["target_id"], "P001")
        self.assertGreater(self.store.total_bytes(), 0)

    def test_relative_path_in_json_and_image_on_disk(self):
        rel = self._add()
        with open(self.path, encoding="utf-8") as f:
            data = json.load(f)
        self.assertEqual(data["samples"][0]["image"], rel)
        # Relative, so the whole config tree can be moved.
        self.assertFalse(Path(rel).is_absolute())
        self.assertTrue((self.path.parent / rel).exists())

    def test_load_crop_round_trips(self):
        self._add()
        img = self.store.load_crop(self.store.samples()[0])
        self.assertIsNotNone(img)
        self.assertEqual(img.shape[:2], (130, 130))

    def test_detection_source_and_user_edited_recorded(self):
        self._add(detection_source=SOURCE_REDRAWN, user_edited=True)
        s = self.store.samples()[0]
        self.assertEqual(s["detection_source"], SOURCE_REDRAWN)
        self.assertTrue(s["user_edited"])

    def test_persistence_round_trip(self):
        self._add()
        self._add(detection_source=SOURCE_MANUAL)
        store2 = SpheroidTrainingStore(self.path)
        self.assertEqual(store2.count(), 2)
        self.assertEqual([s["n"] for s in store2.samples()], [1, 2])

    def test_indices_keep_increasing_across_reloads(self):
        self._add()
        store2 = SpheroidTrainingStore(self.path)
        self._add(store=store2)
        self.assertEqual([s["n"] for s in store2.samples()], [1, 2])

    def test_oldest_first_trim_unlinks_images(self):
        store = SpheroidTrainingStore(Path(self.dir) / "t.json", max_samples=2)
        first = self._add(store=store)
        self._add(store=store)
        self._add(store=store)
        self.assertEqual(store.count(), 2)
        # The trimmed sample's PNG is gone, not merely dereferenced.
        self.assertFalse((store.path.parent / first).exists())
        self.assertEqual([s["n"] for s in store.samples()], [2, 3])

    def test_samples_skips_a_deleted_file(self):
        rel = self._add()
        (self.path.parent / rel).unlink()
        self.assertEqual(self.store.count(), 0)
        self.assertEqual(self.store.total_bytes(), 0)

    def test_empty_crop_is_refused(self):
        self.assertIsNone(self.store.add_crop(
            None, diameter_um=1.0, radius_px=1.0, um_per_px=1.0,
            center_px_in_crop=(0, 0), crop_origin_px=(0, 0),
            frame_wh=(10, 10), stage_um=(0, 0)))
        self.assertIsNone(self.store.add_crop(
            np.zeros((0, 0, 3), dtype=np.uint8),
            diameter_um=1.0, radius_px=1.0, um_per_px=1.0,
            center_px_in_crop=(0, 0), crop_origin_px=(0, 0),
            frame_wh=(10, 10), stage_um=(0, 0)))
        self.assertEqual(self.store.count(), 0)

    def test_clear_removes_images(self):
        rel = self._add()
        self.store.clear()
        self.assertEqual(self.store.count(), 0)
        self.assertFalse((self.path.parent / rel).exists())

    def test_corrupt_index_degrades_to_empty(self):
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("{not json")
        store = SpheroidTrainingStore(self.path)
        self.assertEqual(store.count(), 0)


class TestEnvOverride(unittest.TestCase):
    def test_env_redirects_json_and_image_dir(self):
        d = tempfile.mkdtemp()
        prev = os.environ.get("MEBP_SPHEROID_TRAINING_DIR")
        os.environ["MEBP_SPHEROID_TRAINING_DIR"] = d
        try:
            store = SpheroidTrainingStore()
            self.assertEqual(store.path, Path(d) / "spheroid_training.json")
            self.assertEqual(store.image_dir, Path(d) / "spheroid_training")
            rel = store.add_crop(
                np.zeros((20, 20, 3), dtype=np.uint8),
                diameter_um=100.0, radius_px=25.0, um_per_px=2.0,
                center_px_in_crop=(10, 10), crop_origin_px=(0, 0),
                frame_wh=(640, 480), stage_um=(0.0, 0.0))
            self.assertIsNotNone(rel)
            self.assertTrue((Path(d) / rel).exists())
        finally:
            if prev is None:
                os.environ.pop("MEBP_SPHEROID_TRAINING_DIR", None)
            else:
                os.environ["MEBP_SPHEROID_TRAINING_DIR"] = prev

    def test_get_store_with_explicit_path_rebuilds_singleton(self):
        import SupportClasses.SpheroidTrainingStore as mod
        prev = mod._store_singleton
        try:
            p = Path(tempfile.mkdtemp()) / "s.json"
            store = mod.get_store(p)
            self.assertEqual(store.path, p)
            self.assertIs(mod.get_store(), store)
        finally:
            mod._store_singleton = prev


if __name__ == "__main__":
    unittest.main()
