"""v7.16 — .nd3 container format core tests (SupportClasses/ND3.py).

GUI-free. Everything runs in tmp dirs; nothing touches config/.

The refusal tests each assert the SPECIFIC error type on the SPECIFIC bad
input, so reverting any one guard in ND3.py fails a named test (the
mutation-testing convention this repo uses).
"""

import ast
import json
import os
import subprocess
import sys
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    import h5py
except Exception:  # pragma: no cover
    h5py = None

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SupportClasses import ND3
from SupportClasses.ND3 import (
    ND3Error, ND3FormatError, ND3IntegrityError, ND3ValidationError,
    ND3VersionError, ND3Writer, open_nd3, sniff, verify,
)

# A tiny valid PNG (1x1 white) so preview tests don't need an encoder.
_TINY_PNG = bytes.fromhex(
    "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489"
    "0000000d49444154789c626000010000050001a5f645400000000049454e44ae42"
    "6082")


@unittest.skipUnless(h5py is not None, "h5py not installed")
class _Base(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix="nd3_test_")
        self.addCleanup(shutil.rmtree, self._tmp, ignore_errors=True)

    def path(self, name="file.nd3") -> Path:
        return Path(self._tmp) / name


class TestRoundTrips(_Base):
    def test_every_whitelisted_dtype_round_trips(self):
        dtypes = [np.bool_, np.uint8, np.uint16, np.uint32, np.uint64,
                  np.int8, np.int16, np.int32, np.int64,
                  np.float16, np.float32, np.float64]
        rng = np.random.default_rng(42)
        p = self.path()
        arrays = {}
        with ND3Writer(p) as w:
            for dt in dtypes:
                name = f"img_{np.dtype(dt).name}"
                if np.dtype(dt).kind == "b":
                    arr = rng.integers(0, 2, (5, 7)).astype(bool)
                elif np.dtype(dt).kind == "f":
                    arr = rng.random((5, 7)).astype(dt)
                else:
                    info = np.iinfo(dt)
                    arr = rng.integers(info.min, min(info.max, 2**31 - 1),
                                       (5, 7), dtype=np.int64).astype(dt)
                arrays[name] = arr
                w.add_image(name, arr, axes="YX", pixel_format="gray")
        with open_nd3(p) as r:
            for name, arr in arrays.items():
                img = r.image(name)
                got = img.array()
                self.assertEqual(got.dtype, arr.dtype, name)
                np.testing.assert_array_equal(got, arr, err_msg=name)

    def test_uint16_values_above_255_survive(self):
        # The reason CaptureImageWriter refuses 16-bit PNG: this must be exact.
        arr = np.array([[0, 255], [256, 65535]], dtype=np.uint16)
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("mono16", arr, axes="YX", pixel_format="gray16")
        with open_nd3(p) as r:
            got = r.image("mono16").array()
        self.assertEqual(got.dtype, np.uint16)
        np.testing.assert_array_equal(got, arr)

    def test_big_endian_input_normalised_values_equal(self):
        arr_be = np.arange(12, dtype=">u2").reshape(3, 4)
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("be", arr_be, axes="YX", pixel_format="gray16")
        with open_nd3(p) as r:
            got = r.image("be").array()
        np.testing.assert_array_equal(got, arr_be.astype("<u2"))
        self.assertNotEqual(got.dtype.byteorder, ">")

    def test_cyx_stack_with_channels(self):
        arr = np.stack([np.full((4, 6), i, dtype=np.uint8) for i in range(3)])
        channels = [
            {"name": "DAPI", "color_rgb": [0, 80, 255], "channel_number": 1},
            {"name": "FITC", "color_rgb": [0, 255, 0], "channel_number": 2},
            {"name": "mCherry", "color_rgb": [255, 40, 40], "channel_number": 3},
        ]
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("stack", arr, axes="CYX", pixel_format="gray",
                        channels=channels)
        with open_nd3(p) as r:
            img = r.image("stack")
            self.assertEqual(img.axes, "CYX")
            self.assertEqual(img.channels, channels)
            np.testing.assert_array_equal(img.array(), arr)

    def test_tczyx_planes_round_trip_in_order(self):
        t, c, z = 2, 3, 2
        arr = np.arange(t * c * z * 4 * 5, dtype=np.uint16).reshape(t, c, z, 4, 5)
        planes = []
        for ti in range(t):
            for ci in range(c):
                for zi in range(z):
                    planes.append({"t": ti, "c": ci, "z": zi,
                                   "stage_x_um": 100.0 * ti,
                                   "focus_um": 5.0 * zi})
        channels = [{"name": n} for n in ("a", "b", "c")]
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("five_d", arr, axes="TCZYX", pixel_format="gray16",
                        planes=planes, channels=channels)
        with open_nd3(p) as r:
            img = r.image("five_d")
            self.assertEqual(img.planes, planes)
            np.testing.assert_array_equal(img.array(), arr)

    def test_yxs_rgb_round_trip(self):
        arr = np.zeros((4, 4, 3), dtype=np.uint8)
        arr[..., 0] = 200  # distinctly red in RGB
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("color", arr, axes="YXS", pixel_format="RGB")
        with open_nd3(p) as r:
            img = r.image("color")
            self.assertEqual(img.pixel_format, "RGB")
            np.testing.assert_array_equal(img.array(), arr)

    def test_dataset_meta_user_meta_preview_attachments(self):
        p = self.path()
        dataset_meta = {"profile": "mebp.test/1", "plate_id": "plate-24",
                        "wells_um": {"A1": [105618.0, 65890.0]}}
        with ND3Writer(p, dataset_meta=dataset_meta) as w:
            w.add_image("img", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray",
                        meta={"scale": {"um_per_px": 0.5}},
                        preview_png=_TINY_PNG)
            w.add_attachment("notes.json", b'{"hello": 1}',
                             media_type="application/json")
        with open_nd3(p) as r:
            self.assertEqual(r.dataset_meta, dataset_meta)
            img = r.image("img")
            self.assertEqual(img.meta["scale"], {"um_per_px": 0.5})
            self.assertEqual(img.preview_png(), _TINY_PNG)
            self.assertEqual(r.attachment_names(), ["notes.json"])
            self.assertEqual(r.read_attachment("notes.json"), b'{"hello": 1}')

    def test_transform_helpers(self):
        mat = [[2.0, 0.0, 10.0], [0.0, 2.0, 20.0], [0.0, 0.0, 1.0]]
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("img", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray",
                        meta={"transforms": {"pixel_to_stage_um": mat}})
            w.add_image("bare", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")
        with open_nd3(p) as r:
            got = r.image("img").pixel_to_stage_um()
            np.testing.assert_array_equal(got, np.asarray(mat))
            self.assertIsNone(r.image("img").pixel_to_plate_mm())
            self.assertIsNone(r.image("bare").pixel_to_stage_um())

    def test_section_partial_read_equals_slice(self):
        arr = np.arange(3 * 100 * 100, dtype=np.uint16).reshape(3, 100, 100)
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("big", arr, axes="CYX", pixel_format="gray16")
        with open_nd3(p) as r:
            got = r.image("big").section(np.s_[1, 10:20, 30:40])
        np.testing.assert_array_equal(got, arr[1, 10:20, 30:40])

    def test_image_ids_sorted(self):
        p = self.path()
        with ND3Writer(p) as w:
            for name in ("zeta", "alpha", "mid"):
                w.add_image(name, np.zeros((2, 2), dtype=np.uint8),
                            axes="YX", pixel_format="gray")
        with open_nd3(p) as r:
            self.assertEqual(r.image_ids(), ["alpha", "mid", "zeta"])


class TestStackAppender(_Base):
    def test_time_lapse_appends_and_planes(self):
        p = self.path()
        frames = [np.full((4, 5), i, dtype=np.uint8) for i in range(3)]
        with ND3Writer(p) as w:
            app = w.begin_stack("lapse", axes="TYX", dtype=np.uint8,
                                frame_shape=(4, 5), pixel_format="gray")
            for i, fr in enumerate(frames):
                app.append(fr, plane={"t_iso": f"2026-08-08T10:00:0{i}"})
            app.finish()
        with open_nd3(p) as r:
            img = r.image("lapse")
            self.assertEqual(img.shape, (3, 4, 5))
            self.assertEqual(len(img.planes), 3)
            # auto-filled t indices, in append order
            self.assertEqual([pl["t"] for pl in img.planes], [0, 1, 2])
            np.testing.assert_array_equal(img.array(), np.stack(frames))

    def test_array_after_reader_close_gives_clear_error(self):
        p = self.path("closed.nd3")
        with ND3Writer(p) as w:
            w.add_image("img", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")
        with open_nd3(p) as r:
            img = r.image("img")
        self.assertEqual(img.shape, (2, 2))  # cached metadata still fine
        with self.assertRaisesRegex(ND3Error, "closed"):
            img.array()

    def test_close_auto_finishes_open_appender(self):
        p = self.path()
        with ND3Writer(p) as w:
            app = w.begin_stack("lapse", axes="TYX", dtype=np.uint8,
                                frame_shape=(2, 2), pixel_format="gray")
            app.append(np.zeros((2, 2), dtype=np.uint8))
            # no finish() — close() must seal it
        with open_nd3(p) as r:
            img = r.image("lapse")
        self.assertEqual(img.shape, (1, 2, 2))
        self.assertEqual(img.meta["axes"], "TYX")

    def test_frame_shape_and_dtype_mismatch_refused(self):
        p = self.path()
        with ND3Writer(p) as w:
            app = w.begin_stack("lapse", axes="TYX", dtype=np.uint8,
                                frame_shape=(2, 2), pixel_format="gray")
            with self.assertRaises(ND3ValidationError):
                app.append(np.zeros((3, 2), dtype=np.uint8))
            with self.assertRaises(ND3ValidationError):
                app.append(np.zeros((2, 2), dtype=np.uint16))
            app.append(np.zeros((2, 2), dtype=np.uint8))

    def test_begin_stack_requires_leading_t(self):
        p = self.path()
        with ND3Writer(p) as w:
            with self.assertRaises(ND3ValidationError):
                w.begin_stack("bad", axes="YX", dtype=np.uint8,
                              frame_shape=(2,), pixel_format="gray")
            w.add_image("ok", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")


class TestRefusals(_Base):
    """Every guard, one named test — reverting a guard fails here."""

    def _writer(self):
        w = ND3Writer(self.path())
        self.addCleanup(w.abort)
        return w

    def _add(self, w, **kw):
        args = dict(image_id="img", array=np.zeros((2, 2), dtype=np.uint8),
                    axes="YX", pixel_format="gray")
        args.update(kw)
        image_id = args.pop("image_id")
        array = args.pop("array")
        w.add_image(image_id, array, **args)

    def test_bad_ids_refused_not_sanitized(self):
        w = self._writer()
        for bad in ("has space", "slash/inside", "", ".", "..", "µm", "a\\b"):
            with self.assertRaises(ND3ValidationError, msg=repr(bad)):
                self._add(w, image_id=bad)
        # nothing was silently written under a mangled name
        self.assertEqual(w._ids_lower, set())

    def test_duplicate_id_refused(self):
        w = self._writer()
        self._add(w, image_id="img")
        with self.assertRaises(ND3ValidationError):
            self._add(w, image_id="img")

    def test_case_colliding_id_refused(self):
        w = self._writer()
        self._add(w, image_id="DAPI")
        with self.assertRaises(ND3ValidationError):
            self._add(w, image_id="dapi")

    def test_axes_wrong_order_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="XY")

    def test_axes_missing_x_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="CY", array=np.zeros((2, 2), dtype=np.uint8))

    def test_axes_s_not_last_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="SYX", array=np.zeros((3, 2, 2), dtype=np.uint8),
                      pixel_format="RGB")

    def test_axes_ndim_mismatch_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="CYX", array=np.zeros((2, 2), dtype=np.uint8))

    def test_planes_count_off_by_one_refused(self):
        w = self._writer()
        arr = np.zeros((3, 2, 2), dtype=np.uint8)
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="CYX", array=arr,
                      planes=[{}, {}],  # C=3 needs exactly 3
                      channels=[{"name": n} for n in "abc"])

    def test_plane_index_out_of_range_refused(self):
        w = self._writer()
        arr = np.zeros((3, 2, 2), dtype=np.uint8)
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="CYX", array=arr,
                      planes=[{"c": 0}, {"c": 1}, {"c": 3}],
                      channels=[{"name": n} for n in "abc"])

    def test_plane_index_for_absent_axis_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, planes=[{"z": 0}])  # plain YX image has no Z axis

    def test_channels_vs_c_size_mismatch_refused(self):
        w = self._writer()
        arr = np.zeros((3, 2, 2), dtype=np.uint8)
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="CYX", array=arr, channels=[{"name": "only"}])

    def test_two_channels_without_c_axis_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, channels=[{"name": "a"}, {"name": "b"}])

    def test_non_json_meta_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, meta={"bad": {1, 2, 3}})

    def test_meta_structural_key_collision_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, meta={"axes": "sneaky"})

    def test_non_png_preview_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, preview_png=b"not a png")

    def test_object_dtype_refused(self):
        w = self._writer()
        arr = np.empty((2, 2), dtype=object)
        with self.assertRaises(ND3ValidationError):
            self._add(w, array=arr)

    def test_interleaved_pixel_format_needs_s_axis(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            self._add(w, pixel_format="RGB")  # YX with no S axis

    def test_s_axis_needs_interleaved_pixel_format(self):
        w = self._writer()
        arr = np.zeros((2, 2, 3), dtype=np.uint8)
        with self.assertRaises(ND3ValidationError):
            self._add(w, axes="YXS", array=arr, pixel_format="gray")

    def test_existing_path_without_overwrite_refused(self):
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("img", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")
        with self.assertRaises(ND3ValidationError):
            ND3Writer(p)
        # overwrite=True succeeds
        with ND3Writer(p, overwrite=True) as w:
            w.add_image("other", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")
        with open_nd3(p) as r:
            self.assertEqual(r.image_ids(), ["other"])

    def test_bad_attachment_name_and_duplicate_refused(self):
        w = self._writer()
        with self.assertRaises(ND3ValidationError):
            w.add_attachment("bad name", b"x")
        w.add_attachment("log.txt", b"x")
        with self.assertRaises(ND3ValidationError):
            w.add_attachment("LOG.TXT", b"y")

    def test_non_serializable_dataset_meta_refused_up_front(self):
        with self.assertRaises(ND3ValidationError):
            ND3Writer(self.path("dm.nd3"), dataset_meta={"bad": {1, 2}})
        self.assertFalse(self.path("dm.nd3").exists())
        self.assertFalse(Path(str(self.path("dm.nd3")) + ".tmp").exists())


class TestVersioningAndSniff(_Base):
    def _make_valid(self) -> Path:
        p = self.path()
        with ND3Writer(p) as w:
            w.add_image("img", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")
        return p

    def _patch_version(self, p: Path, version: str):
        with h5py.File(p, "r+") as f:
            f.attrs["schema_version"] = version

    def test_same_major_higher_minor_opens(self):
        p = self._make_valid()
        self._patch_version(p, "1.9")
        with open_nd3(p) as r:
            self.assertEqual(r.schema, "1.9")
            self.assertEqual(r.image_ids(), ["img"])

    def test_higher_major_refused(self):
        p = self._make_valid()
        self._patch_version(p, "2.0")
        with self.assertRaises(ND3VersionError):
            open_nd3(p)

    def test_garbage_version_is_format_error(self):
        p = self._make_valid()
        self._patch_version(p, "banana")
        with self.assertRaises(ND3FormatError):
            open_nd3(p)

    def test_plain_hdf5_is_not_nd3(self):
        p = self.path("plain.h5")
        with h5py.File(p, "w") as f:
            f.create_dataset("x", data=np.zeros(3))
        self.assertFalse(sniff(p))
        with self.assertRaises(ND3FormatError):
            open_nd3(p)

    def test_non_hdf5_file_is_not_nd3(self):
        p = self.path("text.nd3")
        p.write_text("this is not hdf5")
        self.assertFalse(sniff(p))
        with self.assertRaises(ND3FormatError):
            open_nd3(p)

    def test_missing_file_sniffs_false(self):
        self.assertFalse(sniff(self.path("nope.nd3")))

    def test_valid_file_sniffs_true(self):
        self.assertTrue(sniff(self._make_valid()))

    def test_module_level_verify_reports_bad_files_as_strings(self):
        p = self.path("text.nd3")
        p.write_text("not hdf5")
        problems = verify(p)
        self.assertEqual(len(problems), 1)
        self.assertIn("not readable HDF5", problems[0])


class TestIntegrityAndLaziness(_Base):
    _PATTERN_VALUE = 0xAB
    _SHAPE = (300, 300)

    def _make_corruptible(self) -> Path:
        """Uncompressed payload of a known byte so we can corrupt ONLY it."""
        p = self.path()
        arr = np.full(self._SHAPE, self._PATTERN_VALUE, dtype=np.uint8)
        with ND3Writer(p, compress=False) as w:
            w.add_image("victim", arr, axes="YX", pixel_format="gray",
                        meta={"scale": {"um_per_px": 1.0}})
        return p

    def _corrupt_payload(self, p: Path):
        blob = bytearray(p.read_bytes())
        run = bytes([self._PATTERN_VALUE]) * 4096
        at = blob.find(run)
        self.assertGreater(at, 0, "payload pattern not found in file")
        blob[at + 2048] ^= 0xFF
        p.write_bytes(bytes(blob))

    def test_clean_file_verifies_empty(self):
        p = self._make_corruptible()
        self.assertEqual(verify(p, deep=True), [])

    def test_corrupted_payload_named_by_deep_verify(self):
        p = self._make_corruptible()
        self._corrupt_payload(p)
        problems = verify(p, deep=True)
        self.assertTrue(problems, "deep verify missed the corruption")
        self.assertTrue(any("images/victim/data" in s for s in problems),
                        problems)

    def test_shallow_verify_does_not_read_payloads(self):
        # Proves deep=True is what does the reading.
        p = self._make_corruptible()
        self._corrupt_payload(p)
        self.assertEqual(verify(p, deep=False), [])

    def test_metadata_reads_are_lazy_only_array_raises(self):
        p = self._make_corruptible()
        self._corrupt_payload(p)
        with open_nd3(p) as r:
            img = r.image("victim")           # meta reads fine
            self.assertEqual(img.axes, "YX")
            self.assertEqual(img.meta["scale"], {"um_per_px": 1.0})
            with self.assertRaises(ND3IntegrityError):
                img.array()

    def test_atomicity_exception_leaves_nothing(self):
        p = self.path()
        with self.assertRaises(RuntimeError):
            with ND3Writer(p) as w:
                w.add_image("img", np.zeros((2, 2), dtype=np.uint8),
                            axes="YX", pixel_format="gray")
                raise RuntimeError("boom")
        self.assertFalse(p.exists())
        self.assertFalse(Path(str(p) + ".tmp").exists())

    def test_abort_idempotent_and_clean_close_leaves_no_tmp(self):
        p = self.path()
        w = ND3Writer(p)
        w.abort()
        w.abort()
        self.assertFalse(p.exists())
        p2 = self.path("second.nd3")
        with ND3Writer(p2) as w:
            w.add_image("img", np.zeros((2, 2), dtype=np.uint8), axes="YX",
                        pixel_format="gray")
        self.assertTrue(p2.exists())
        self.assertFalse(Path(str(p2) + ".tmp").exists())


class TestPurityAndConsumers(_Base):
    _ND3_PATH = Path(__file__).resolve().parent.parent / "SupportClasses" / "ND3.py"

    def test_nd3_imports_only_stdlib_numpy_h5py(self):
        tree = ast.parse(self._ND3_PATH.read_text(encoding="utf-8"))
        allowed = set(sys.stdlib_module_names) | {"numpy", "h5py"}
        offenders = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".")[0]
                    if root not in allowed:
                        offenders.append(alias.name)
            elif isinstance(node, ast.ImportFrom):
                if node.level:  # relative import ties it to the package
                    offenders.append(f"relative:{node.module}")
                elif (node.module or "").split(".")[0] not in allowed:
                    offenders.append(node.module)
        self.assertEqual(offenders, [],
                         "ND3.py must stay vendorable: stdlib + numpy + h5py only")

    def test_vendored_copy_works_alone(self):
        # Copy ND3.py by itself into a tmp dir and round-trip via a subprocess
        # whose import path contains that dir but NOT this repo.
        vendor = Path(self._tmp) / "vendor"
        vendor.mkdir()
        shutil.copy2(self._ND3_PATH, vendor / "ND3.py")
        script = (
            "import sys, json\n"
            f"sys.path.insert(0, {str(vendor)!r})\n"
            "import numpy as np\n"
            "import ND3\n"
            "assert 'SupportClasses' not in sys.modules\n"
            f"p = {str(Path(self._tmp) / 'vendored.nd3')!r}\n"
            "arr = np.arange(2000, 2012, dtype=np.uint16).reshape(3, 4)\n"
            "with ND3.ND3Writer(p) as w:\n"
            "    w.add_image('img', arr, axes='YX', pixel_format='gray16')\n"
            "with ND3.open_nd3(p) as r:\n"
            "    got = r.image('img').array()\n"
            "assert got.dtype == np.uint16 and (got == arr).all()\n"
            "print('OK')\n"
        )
        proc = subprocess.run([sys.executable, "-c", script],
                              capture_output=True, text=True, timeout=120,
                              cwd=self._tmp)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("OK", proc.stdout)

    def test_third_party_consumer_reads_with_raw_h5py_and_json(self):
        # Simulate Blender/LabLink: NO ND3.py involved on the read side.
        p = self.path()
        arr = np.arange(24, dtype=np.uint16).reshape(4, 6)
        mat = [[2.0, 0.0, 100.0], [0.0, 2.0, 200.0], [0.0, 0.0, 1.0]]
        with ND3Writer(p, dataset_meta={"profile": "mebp.test/1"}) as w:
            w.add_image("img", arr, axes="YX", pixel_format="gray16",
                        meta={"transforms": {"pixel_to_stage_um": mat}})
        with h5py.File(p, "r") as f:
            self.assertEqual(f.attrs["format"], "nd3")
            pixels = f["images/img/data"][()]
            meta = json.loads(bytes(f["images/img/meta_json"][()]))
            dataset = json.loads(bytes(f["dataset_json"][()]))
        np.testing.assert_array_equal(pixels, arr)
        self.assertEqual(meta["transforms"]["pixel_to_stage_um"], mat)
        self.assertEqual(meta["pixel_format"], "gray16")
        self.assertEqual(dataset["profile"], "mebp.test/1")


if __name__ == "__main__":
    unittest.main()
