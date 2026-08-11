"""v7.16 — .nd3 exporters (SupportClasses/ND3Export.py) against REAL stores.

Real ``FluorescenceMosaicStore`` isolated via ``MEBP_FLUOR_MOSAIC_PATH``; real
``MosaicStore`` via its ``path=`` constructor. Nothing touches config/.

The frame-rule tests are deliberately mutation-hostile:
* mosaic pitch is seeded DIFFERENT from the camera µm/px, so using the wrong
  one fails (`test_pitch_is_inverse_mosaic_scale_not_um_per_px`);
* the BGR→RGB swap is asserted at the pixel level;
* the legacy paths assert what is ABSENT (no fabricated plate frame, no
  shift_known=True) — reverting an honesty guard fails a named test.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None
try:
    import h5py
except Exception:  # pragma: no cover
    h5py = None

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

if h5py is not None and cv2 is not None:
    from SupportClasses.FluorescenceMosaicStore import FluorescenceMosaicStore
    from SupportClasses.MosaicStore import MosaicStore, stage_to_plate_mm
    from SupportClasses.CaptureMetadata import CaptureMeta
    from SupportClasses.ND3 import ND3ValidationError, open_nd3
    from SupportClasses.ND3Export import (
        ND3ExportError, export_capture, export_fluorescence_well,
        export_lablink_job, export_plate_mosaic, export_time_lapse,
    )

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


@unittest.skipUnless(h5py is not None and cv2 is not None,
                     "h5py + cv2 required")
class _Base(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix="nd3_export_")
        self.addCleanup(shutil.rmtree, self._tmp, ignore_errors=True)

    def path(self, name) -> Path:
        return Path(self._tmp) / name


class _FluorBase(_Base):
    """A real FluorescenceMosaicStore in a tmp dir (env-var isolation)."""

    PLATE = "plate-24"
    WELL = "B3"
    # ⚠ Deliberately DIFFERENT so pitch-vs-camera-µm/px mix-ups are visible:
    MOSAIC_SCALE = 2.0       # canvas px per µm  → image pitch = 0.5 µm/px
    UM_PER_PX = 0.65         # camera pitch at capture — WRONG for the canvas
    EXTENT = (1000.0, 2000.0, 1080.0, 2080.0)
    SHIFT = (30.0, -10.0)

    def setUp(self):
        super().setUp()
        self._store_path = self.path("fluor.json")
        self._old_env = os.environ.get("MEBP_FLUOR_MOSAIC_PATH")
        os.environ["MEBP_FLUOR_MOSAIC_PATH"] = str(self._store_path)

        def _restore():
            if self._old_env is None:
                os.environ.pop("MEBP_FLUOR_MOSAIC_PATH", None)
            else:
                os.environ["MEBP_FLUOR_MOSAIC_PATH"] = self._old_env
        self.addCleanup(_restore)
        self.store = FluorescenceMosaicStore()

    def seed_gray_channel(self, channel="DAPI", value=90, shape=(40, 40),
                          **kw):
        img = np.full(shape + (3,), value, dtype=np.uint8)  # collapses
        args = dict(extent_um=self.EXTENT, um_per_px=self.UM_PER_PX,
                    mosaic_scale=self.MOSAIC_SCALE, exposure_us=20000.0,
                    shift_um=self.SHIFT)
        args.update(kw)
        self.assertTrue(self.store.save_channel(
            self.PLATE, self.WELL, channel, img, **args))
        return img

    def seed_color_channel(self, channel="FITC", shape=(40, 40), **kw):
        img = np.zeros(shape + (3,), dtype=np.uint8)
        img[..., 0] = 200   # B
        img[..., 1] = 20    # G
        img[..., 2] = 10    # R  → RGB pixel must read (10, 20, 200)
        args = dict(extent_um=self.EXTENT, um_per_px=self.UM_PER_PX,
                    mosaic_scale=self.MOSAIC_SCALE, shift_um=self.SHIFT)
        args.update(kw)
        self.assertTrue(self.store.save_channel(
            self.PLATE, self.WELL, channel, img, **args))
        return img


class TestFluorescenceExport(_FluorBase):
    def test_bgr_to_rgb_swap_actually_happened(self):
        self.seed_color_channel("FITC")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            img = r.image("FITC")
            self.assertEqual(img.pixel_format, "RGB")
            px = img.array()[0, 0]
        # Stored BGR was (B=200, G=20, R=10); .nd3 is RGB.
        self.assertEqual(tuple(int(v) for v in px), (10, 20, 200))

    def test_gray_collapse_when_planes_identical(self):
        self.seed_gray_channel("DAPI")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            img = r.image("DAPI")
            self.assertEqual(img.axes, "YX")
            self.assertEqual(img.pixel_format, "gray")
            self.assertEqual(img.shape, (40, 40))

    def test_pitch_is_inverse_mosaic_scale_not_um_per_px(self):
        self.seed_gray_channel("DAPI")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            img = r.image("DAPI")
            scale = img.meta["scale"]
            mat = img.pixel_to_stage_um()
        pitch = 1.0 / self.MOSAIC_SCALE          # 0.5 — NOT 0.65
        self.assertAlmostEqual(scale["um_per_px"], pitch)
        self.assertAlmostEqual(scale["captured_um_per_px"], self.UM_PER_PX)
        self.assertAlmostEqual(float(mat[0, 0]), pitch)
        self.assertAlmostEqual(float(mat[1, 1]), pitch)

    def test_transform_origin_is_extent_minus_shift(self):
        self.seed_gray_channel("DAPI")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            img = r.image("DAPI")
            mat = img.pixel_to_stage_um()
            sf = img.meta["stage_frame"]
        self.assertAlmostEqual(float(mat[0, 2]),
                               self.EXTENT[0] - self.SHIFT[0])   # 970
        self.assertAlmostEqual(float(mat[1, 2]),
                               self.EXTENT[1] - self.SHIFT[1])   # 2010
        self.assertTrue(sf["shift_known"])
        self.assertEqual(sf["shift_um"], list(self.SHIFT))

    def test_legacy_entry_without_shift_reports_unknown(self):
        self.seed_gray_channel("DAPI")
        # Simulate a pre-v7.8 record: the shift key never existed.
        data = json.loads(self._store_path.read_text(encoding="utf-8"))
        wkey = f"{self.PLATE}|{self.WELL}"
        del data["wells"][wkey]["channels"]["DAPI"]["shift_um"]
        self._store_path.write_text(json.dumps(data), encoding="utf-8")
        store = FluorescenceMosaicStore()
        self.assertFalse(store.has_shift(self.PLATE, self.WELL, "DAPI"))

        out = export_fluorescence_well(store, self.PLATE, self.WELL,
                                       self.path("legacy.nd3"))
        with open_nd3(out) as r:
            img = r.image("DAPI")
            sf = img.meta["stage_frame"]
            mat = img.pixel_to_stage_um()
        self.assertFalse(sf["shift_known"])
        self.assertEqual(sf["shift_um"], [0.0, 0.0])
        # Matrix uses the display extent verbatim — the flag carries the caveat.
        self.assertAlmostEqual(float(mat[0, 2]), self.EXTENT[0])

    def test_channel_entry_fields(self):
        self.seed_gray_channel("DAPI", color_rgb=(1, 2, 3),
                               display_levels=(210.0, 1900.0), avg_frames=4)
        self.seed_color_channel("FITC")   # no display levels
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            dapi = r.image("DAPI").channels[0]
            fitc = r.image("FITC").channels[0]
        self.assertEqual(dapi["name"], "DAPI")
        self.assertEqual(dapi["color_rgb"], [1, 2, 3])
        self.assertEqual(dapi["channel_number"], 1)
        self.assertEqual(dapi["exposure_us"], 20000.0)
        self.assertEqual(dapi["avg_frames"], 4)
        self.assertEqual(dapi["display_lo"], 210.0)
        self.assertEqual(dapi["display_hi"], 1900.0)
        # Absent means UNKNOWN — never defaulted to 0/65535.
        self.assertNotIn("display_lo", fitc)
        self.assertNotIn("display_hi", fitc)

    def test_mismatched_channel_shapes_exported_verbatim(self):
        self.seed_gray_channel("DAPI", shape=(40, 40))
        self.seed_color_channel("FITC", shape=(60, 50))
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            self.assertEqual(r.image("DAPI").shape, (40, 40))
            self.assertEqual(r.image("FITC").shape, (60, 50, 3))  # NO resize

    def test_bright_field_id_mapping_keeps_true_name(self):
        self.seed_gray_channel("Bright Field")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            self.assertEqual(r.image_ids(), ["Bright_Field"])
            ch = r.image("Bright_Field").channels[0]
        self.assertEqual(ch["name"], "Bright Field")
        self.assertEqual(ch["channel_number"], 5)

    def test_unknown_invalid_channel_name_refused(self):
        self.seed_gray_channel("weird name")
        with self.assertRaises(ND3ExportError):
            export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                     self.path("bad.nd3"))
        self.assertFalse(self.path("bad.nd3").exists())

    def test_plate_frame_param_adds_plate_matrix(self):
        self.seed_gray_channel("DAPI")
        pf = {"extent_mm": [0.0, 0.0, 1.0, 1.0],
              "anchor_um": [1500.0, 2500.0], "axis_sign": [-1.0, -1.0]}
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"), plate_frame=pf)
        with open_nd3(out) as r:
            img = r.image("DAPI")
            mmat = img.pixel_to_plate_mm()
            self.assertNotIn("needs_plate_frame", img.meta)
            self.assertEqual(img.meta["plate_frame"]["anchor_um"],
                             [1500.0, 2500.0])
        self.assertIsNotNone(mmat)
        # pixel (0,0) is at stage (970, 2010) → plate mm with sign −1:
        self.assertAlmostEqual(float(mmat[0, 2]), -1 * (970.0 - 1500.0) / 1000)
        self.assertAlmostEqual(float(mmat[1, 2]), -1 * (2010.0 - 2500.0) / 1000)

    def test_no_plate_frame_is_honest(self):
        self.seed_gray_channel("DAPI")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        with open_nd3(out) as r:
            img = r.image("DAPI")
            self.assertTrue(img.meta.get("needs_plate_frame"))
            self.assertNotIn("plate_frame", img.meta)
            self.assertIsNone(img.pixel_to_plate_mm())

    def test_unreadable_channel_skipped_with_warning_or_refused(self):
        self.seed_gray_channel("DAPI")
        self.seed_color_channel("FITC")
        # Break FITC's image file on disk.
        data = json.loads(self._store_path.read_text(encoding="utf-8"))
        wkey = f"{self.PLATE}|{self.WELL}"
        rel = data["wells"][wkey]["channels"]["FITC"]["image"]
        (self._store_path.parent / rel).unlink()
        store = FluorescenceMosaicStore()
        # channels=None → skip + warn
        out = export_fluorescence_well(store, self.PLATE, self.WELL,
                                       self.path("skip.nd3"))
        with open_nd3(out) as r:
            self.assertEqual(r.image_ids(), ["DAPI"])
            self.assertTrue(any("FITC" in wtext
                                for wtext in r.dataset_meta["warnings"]))
        # explicitly requested → refused
        with self.assertRaises(ND3ExportError):
            export_fluorescence_well(store, self.PLATE, self.WELL,
                                     self.path("req.nd3"),
                                     channels=["DAPI", "FITC"])

    def test_missing_well_refused(self):
        with self.assertRaises(ND3ExportError):
            export_fluorescence_well(self.store, self.PLATE, "Z9",
                                     self.path("none.nd3"))

    def test_dataset_meta_and_preview(self):
        self.seed_gray_channel("DAPI", objective="10x NA 0.3")
        out = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"),
                                       operator="alex", notes="test run")
        with open_nd3(out) as r:
            dm = r.dataset_meta
            self.assertEqual(dm["profile"], "mebp.fluor_well/1")
            self.assertEqual(dm["plate_id"], self.PLATE)
            self.assertEqual(dm["well"], self.WELL)
            self.assertEqual(dm["objective"], "10x NA 0.3")
            self.assertEqual(dm["operator"], "alex")
            preview = r.image("DAPI").preview_png()
        self.assertIsNotNone(preview)
        self.assertTrue(preview.startswith(PNG_MAGIC))


class TestPlateMosaicExport(_Base):
    PLATE = "plate-24"
    EXTENT = (10000.0, 20000.0, 110000.0, 76000.0)
    SHIFT = (150.0, -40.0)
    ANCHOR = (105618.0, 65890.0)
    SIGN = (-1.0, -1.0)
    MOSAIC_SCALE = 0.02    # canvas px per µm → pitch 50 µm/px
    UM_PER_PX = 3.227      # camera pitch — wrong for the canvas

    def _store(self) -> "MosaicStore":
        return MosaicStore(self.path("plate_mosaics.json"))

    def _seed(self, store, *, anchored=True):
        img = np.full((112, 200, 3), 77, dtype=np.uint8)
        kwargs = dict(um_per_px=self.UM_PER_PX,
                      mosaic_scale=self.MOSAIC_SCALE,
                      frames=42, shift_um=self.SHIFT)
        if anchored:
            kwargs.update(anchor_um=self.ANCHOR, axis_sign=self.SIGN)
        self.assertTrue(store.save(self.PLATE, img, self.EXTENT, **kwargs))
        store.set_wells(self.PLATE, {"A1": self.ANCHOR,
                                     "B3": (86028.0, 28274.0)})

    def test_plate_frame_and_wells_round_trip(self):
        store = self._store()
        self._seed(store)
        out = export_plate_mosaic(store, self.PLATE, self.path("plate.nd3"))
        with open_nd3(out) as r:
            img = r.image("mosaic")
            dm = r.dataset_meta
            pf = img.meta["plate_frame"]
            m_stage = img.pixel_to_stage_um()
            m_plate = img.pixel_to_plate_mm()
        self.assertEqual(dm["profile"], "mebp.plate_mosaic/1")
        self.assertEqual(pf["anchor_um"], list(self.ANCHOR))
        self.assertEqual(pf["axis_sign"], list(self.SIGN))
        store_pf = store.plate_frame(self.PLATE)
        np.testing.assert_allclose(pf["extent_mm"], store_pf["extent_mm"])

        # The matrices must agree with the canonical converter: take the
        # mapped B3 centre (stage µm) → pixel → plate mm, and compare with
        # stage_to_plate_mm of the same point.
        wx, wy = dm["wells_um"]["B3"]
        px = np.linalg.inv(m_stage) @ np.array([wx, wy, 1.0])
        mm = m_plate @ px
        exp = stage_to_plate_mm((wx, wy, wx, wy), self.ANCHOR, self.SIGN)
        self.assertAlmostEqual(float(mm[0]), exp[0], places=9)
        self.assertAlmostEqual(float(mm[1]), exp[1], places=9)

    def test_pitch_uses_mosaic_scale(self):
        store = self._store()
        self._seed(store)
        out = export_plate_mosaic(store, self.PLATE, self.path("plate.nd3"))
        with open_nd3(out) as r:
            img = r.image("mosaic")
            scale = img.meta["scale"]
            mat = img.pixel_to_stage_um()
        self.assertAlmostEqual(scale["um_per_px"], 50.0)          # 1/0.02
        self.assertAlmostEqual(scale["captured_um_per_px"], self.UM_PER_PX)
        self.assertAlmostEqual(float(mat[0, 0]), 50.0)
        self.assertAlmostEqual(float(mat[0, 2]),
                               self.EXTENT[0] - self.SHIFT[0])

    def test_legacy_scan_without_anchor_is_honest(self):
        store = self._store()
        self._seed(store, anchored=False)
        self.assertIsNone(store.plate_frame(self.PLATE))
        out = export_plate_mosaic(store, self.PLATE, self.path("legacy.nd3"))
        with open_nd3(out) as r:
            img = r.image("mosaic")
            self.assertTrue(img.meta.get("needs_plate_frame"))
            self.assertNotIn("plate_frame", img.meta)
            self.assertIsNone(img.pixel_to_plate_mm())
            # stage frame is still present and usable
            self.assertIsNotNone(img.pixel_to_stage_um())

    def test_gray_collapse_and_shape(self):
        store = self._store()
        self._seed(store)
        out = export_plate_mosaic(store, self.PLATE, self.path("plate.nd3"))
        with open_nd3(out) as r:
            img = r.image("mosaic")
            self.assertEqual(img.axes, "YX")
            self.assertEqual(img.pixel_format, "gray")
            self.assertEqual(img.shape, (112, 200))

    def test_missing_plate_refused(self):
        store = self._store()
        with self.assertRaises(ND3ExportError):
            export_plate_mosaic(store, "nope", self.path("none.nd3"))


class TestCaptureExport(_Base):
    def _meta(self, **overrides) -> "CaptureMeta":
        base = dict(kind="still", timestamp_iso="2026-08-08T10:00:00",
                    source_mode="raw", camera_slot=1,
                    camera_name="Andor Zyla", camera_identity="andor:VSC-07863",
                    captured_w=64, captured_h=48, exposure_us=15000.0,
                    gain_pct=10.0, bit_depth="16-bit",
                    um_per_px=0.65, objective="10x NA 0.3",
                    channel="DAPI", stage_x_um=50000.0, stage_y_um=40000.0,
                    focus_um=771.175, well="B3", plate="plate-24",
                    operator="alex")
        base.update(overrides)
        return CaptureMeta(**base)

    def test_uint16_capture_gray16_with_geometry(self):
        arr = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64) + 300
        out = export_capture(arr, self._meta(), self.path("cap.nd3"))
        with open_nd3(out) as r:
            img = r.image("capture")
            self.assertEqual(img.pixel_format, "gray16")
            np.testing.assert_array_equal(img.array(), arr)
            acq = img.meta["acquisition"]
            plane = img.planes[0]
            sf = img.meta["stage_frame"]
            mat = img.pixel_to_stage_um()
            dm = r.dataset_meta
        # geometry lives in the plane record, NOT in acquisition
        self.assertEqual(plane["stage_x_um"], 50000.0)
        self.assertEqual(plane["focus_um"], 771.175)
        self.assertEqual(plane["t_iso"], "2026-08-08T10:00:00")
        self.assertNotIn("stage_x_um", acq)
        self.assertEqual(acq["camera_identity"], "andor:VSC-07863")
        self.assertEqual(acq["exposure_us"], 15000.0)
        # extent = stage centre ± half FOV at um_per_px
        self.assertAlmostEqual(sf["extent_um"][0], 50000.0 - 64 * 0.65 / 2)
        self.assertAlmostEqual(sf["extent_um"][1], 40000.0 - 48 * 0.65 / 2)
        self.assertTrue(sf["shift_known"])
        self.assertAlmostEqual(float(mat[0, 0]), 0.65)
        self.assertEqual(dm["profile"], "mebp.capture/1")
        self.assertEqual(dm["well"], "B3")
        self.assertEqual(dm["plate_id"], "plate-24")

    def test_rotated_view_gets_no_transforms(self):
        arr = np.zeros((48, 64), dtype=np.uint16)
        out = export_capture(arr, self._meta(view_rotation_deg=90.0),
                             self.path("rot.nd3"))
        with open_nd3(out) as r:
            img = r.image("capture")
            self.assertFalse(img.meta["orientation"]["pixels_stage_aligned"])
            self.assertNotIn("transforms", img.meta)
            self.assertIsNone(img.pixel_to_stage_um())

    def test_bgr_capture_converted_to_rgb(self):
        arr = np.zeros((8, 8, 3), dtype=np.uint8)
        arr[..., 0] = 250   # blue in BGR
        out = export_capture(arr, self._meta(bit_depth="8-bit"),
                             self.path("color.nd3"))
        with open_nd3(out) as r:
            img = r.image("capture")
            self.assertEqual(img.pixel_format, "RGB")
            px = img.array()[0, 0]
        self.assertEqual(tuple(int(v) for v in px), (0, 0, 250))

    def test_preview_present_and_png(self):
        arr = np.linspace(0, 65535, 48 * 64, dtype=np.uint16).reshape(48, 64)
        out = export_capture(arr, self._meta(), self.path("cap.nd3"))
        with open_nd3(out) as r:
            preview = r.image("capture").preview_png()
        self.assertIsNotNone(preview)
        self.assertTrue(preview.startswith(PNG_MAGIC))

    def test_channel_recorded(self):
        arr = np.zeros((8, 8), dtype=np.uint16)
        out = export_capture(arr, self._meta(), self.path("cap.nd3"))
        with open_nd3(out) as r:
            self.assertEqual(r.image("capture").channels,
                             [{"name": "DAPI"}])


class TestTimeLapseExport(_Base):
    def _make_sequence_dir(self, n=3) -> Path:
        seq = self.path("seq")
        seq.mkdir()
        records = []
        for i in range(1, n + 1):
            frame = np.full((10, 12), 1000 * i, dtype=np.uint16)
            name = f"frame_{i:06d}.tif"
            self.assertTrue(cv2.imwrite(str(seq / name), frame))
            records.append({"i": i, "file": name,
                            "t_wall": 1780000000.0 + i,
                            "t_mono": 100.0 + i * 2.0})
        manifest = {"version": "1.0", "kind": "raw_timelapse",
                    "interval_s": 2.0, "frames": records,
                    "meta": {"exposure_us": 5000.0, "plate": "plate-24",
                             "well": "A1", "camera_name": "Andor Zyla"}}
        (seq / "manifest.json").write_text(json.dumps(manifest),
                                           encoding="utf-8")
        return seq

    def test_sequence_dir_becomes_t_stack(self):
        seq = self._make_sequence_dir(3)
        out = export_time_lapse(seq, self.path("lapse.nd3"))
        with open_nd3(out) as r:
            img = r.image("lapse")
            dm = r.dataset_meta
            self.assertEqual(img.axes, "TYX")
            self.assertEqual(img.shape, (3, 10, 12))
            self.assertEqual(img.dtype, np.dtype(np.uint16))
            self.assertEqual(len(img.planes), 3)
            self.assertEqual([pl["t"] for pl in img.planes], [0, 1, 2])
            # relative seconds from t_mono
            self.assertEqual([pl["t_s"] for pl in img.planes],
                             [0.0, 2.0, 4.0])
            self.assertTrue(all("t_iso" in pl for pl in img.planes))
            arr = img.array()
            self.assertEqual(int(arr[1, 0, 0]), 2000)
            self.assertEqual(img.meta["time_lapse"]["interval_s"], 2.0)
            self.assertEqual(img.meta["acquisition"]["exposure_us"], 5000.0)
        self.assertEqual(dm["profile"], "mebp.time_lapse/1")
        self.assertEqual(dm["plate_id"], "plate-24")
        self.assertEqual(dm["well"], "A1")

    def test_in_memory_frames(self):
        frames = [(np.full((4, 4), i, dtype=np.uint8),
                   {"t_iso": f"2026-08-08T10:00:0{i}"}) for i in range(2)]
        out = export_time_lapse(frames, self.path("mem.nd3"))
        with open_nd3(out) as r:
            img = r.image("lapse")
            self.assertEqual(img.shape, (2, 4, 4))
            self.assertEqual(img.planes[1]["t_iso"], "2026-08-08T10:00:01")

    def test_mismatched_frame_shape_refused(self):
        frames = [(np.zeros((4, 4), dtype=np.uint8), None),
                  (np.zeros((5, 4), dtype=np.uint8), None)]
        with self.assertRaises(ND3ValidationError):
            export_time_lapse(frames, self.path("bad.nd3"))
        self.assertFalse(self.path("bad.nd3").exists())

    def test_empty_source_refused(self):
        with self.assertRaises(ND3ExportError):
            export_time_lapse([], self.path("empty.nd3"))
        seq = self.path("notseq")
        seq.mkdir()
        with self.assertRaises(ND3ExportError):
            export_time_lapse(seq, self.path("empty2.nd3"))


class TestLabLinkJobExport(_FluorBase):
    """Bridge to lablink/docs/IMAGE-JOB-FORMAT.md: TIFF + .job.json pair.

    The sidecar is the AUTHORITY over the TIFF (LabLink's own rule — a TIFF
    invents the container bit depth and a placeholder channel name), so the
    tests pin what goes INTO the sidecar, not what the TIFF claims."""

    def _capture_nd3(self, name="cap.nd3", **meta_overrides) -> Path:
        base = dict(kind="still", timestamp_iso="2026-08-08T10:00:00",
                    camera_name="Andor Zyla", captured_w=64, captured_h=48,
                    exposure_us=15000.0, bit_depth="16-bit", um_per_px=0.65,
                    objective="10x NA 0.3", magnification="10x",
                    numerical_aperture=0.3, channel="DAPI",
                    stage_x_um=50000.0, stage_y_um=40000.0,
                    well="B3", plate="plate-24")
        base.update(meta_overrides)
        base = {k: v for k, v in base.items() if v is not None}
        arr = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64) + 300
        return export_capture(arr, CaptureMeta(**base), self.path(name))

    def _read_sidecar(self, sidecar_path: Path) -> dict:
        return json.loads(sidecar_path.read_text(encoding="utf-8"))

    def test_capture_to_job_pair(self):
        nd3 = self._capture_nd3()
        tif, sidecar_path = export_lablink_job(nd3, "capture",
                                               self.path("jobs"))
        self.assertTrue(tif.exists())
        self.assertEqual(tif.stem, sidecar_path.name[:-len(".job.json")])
        # TIFF round-trips the pixels (uint16 preserved)
        back = cv2.imread(str(tif), cv2.IMREAD_UNCHANGED)
        self.assertEqual(back.dtype, np.uint16)
        self.assertEqual(int(back[0, 0]), 300)
        side = self._read_sidecar(sidecar_path)
        self.assertEqual(side["format"], "lablink.imagejob/1")
        img = side["image"]
        self.assertEqual(img["pixel_size_um"], 0.65)
        self.assertEqual(img["bit_depth"], 16)          # parsed from "16-bit"
        self.assertEqual(img["objective_magnification"], 10.0)
        self.assertEqual(img["objective_na"], 0.3)
        self.assertEqual(img["channels"], [{"name": "DAPI"}])
        self.assertEqual(side["source"]["original_name"], "cap.nd3")

    def test_wavelengths_param_fills_channel_entry(self):
        nd3 = self._capture_nd3()
        _, sidecar_path = export_lablink_job(
            nd3, "capture", self.path("jobs"),
            wavelengths={"DAPI": {"emission_nm": 461.0,
                                  "excitation_nm": 358.0}})
        ch = self._read_sidecar(sidecar_path)["image"]["channels"][0]
        self.assertEqual(ch["emission_nm"], 461.0)
        self.assertEqual(ch["excitation_nm"], 358.0)

    def test_wavelengths_absent_stays_absent_not_fabricated(self):
        nd3 = self._capture_nd3()
        _, sidecar_path = export_lablink_job(nd3, "capture", self.path("jobs"))
        ch = self._read_sidecar(sidecar_path)["image"]["channels"][0]
        self.assertNotIn("emission_nm", ch)
        self.assertNotIn("excitation_nm", ch)

    def test_unknown_bit_depth_refused_never_container_dtype(self):
        nd3 = self._capture_nd3("nobits.nd3", bit_depth=None)
        with self.assertRaisesRegex(ND3ExportError, "bit depth"):
            export_lablink_job(nd3, "capture", self.path("jobs"))
        # explicit override works (sensor 12-bit in a 16-bit container)
        _, sidecar_path = export_lablink_job(nd3, "capture",
                                             self.path("jobs2"),
                                             bit_depth=12)
        self.assertEqual(self._read_sidecar(sidecar_path)["image"]["bit_depth"],
                         12)

    def test_missing_um_per_px_refused(self):
        nd3 = self._capture_nd3("noscale.nd3", um_per_px=None)
        with self.assertRaisesRegex(ND3ExportError, "um_per_px"):
            export_lablink_job(nd3, "capture", self.path("jobs"))

    def test_stack_refused(self):
        frames = [(np.zeros((4, 4), dtype=np.uint8), None)] * 2
        nd3 = export_time_lapse(frames, self.path("lapse.nd3"))
        with self.assertRaisesRegex(ND3ExportError, "stack"):
            export_lablink_job(nd3, "lapse", self.path("jobs"), bit_depth=8)

    def test_noncompliant_stem_refused_and_custom_stem_works(self):
        nd3 = self._capture_nd3()
        with self.assertRaisesRegex(ND3ExportError, "upload-name"):
            export_lablink_job(nd3, "capture", self.path("jobs"),
                               stem="A1 (well 3)")
        tif, _ = export_lablink_job(nd3, "capture", self.path("jobs"),
                                    stem="A1 well 3")
        self.assertEqual(tif.name, "A1 well 3.tif")

    def test_recipe_and_null_knob_preserved(self):
        nd3 = self._capture_nd3()
        _, sidecar_path = export_lablink_job(
            nd3, "capture", self.path("jobs"),
            recipe="selftest-synthetic",
            knobs={"background_um": 5.0, "min_area_um2": None})
        job = self._read_sidecar(sidecar_path)["job"]
        self.assertEqual(job["recipe"], "selftest-synthetic")
        # null means "derive from the image" — very different from 0/omitted
        self.assertIn("min_area_um2", job["knobs"])
        self.assertIsNone(job["knobs"]["min_area_um2"])

    def test_fluor_channel_pixel_size_is_canvas_pitch(self):
        # The sidecar must carry THIS image's pitch (1/mosaic_scale), not the
        # camera µm/px — same trap as the transforms, pinned here too.
        self.seed_gray_channel("DAPI")
        nd3 = export_fluorescence_well(self.store, self.PLATE, self.WELL,
                                       self.path("well.nd3"))
        _, sidecar_path = export_lablink_job(nd3, "DAPI", self.path("jobs"),
                                             bit_depth=8)
        img = self._read_sidecar(sidecar_path)["image"]
        self.assertAlmostEqual(img["pixel_size_um"], 1.0 / self.MOSAIC_SCALE)
        self.assertEqual(img["channels"][0]["name"], "DAPI")


if __name__ == "__main__":
    unittest.main()
