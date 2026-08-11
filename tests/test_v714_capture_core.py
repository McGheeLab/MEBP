"""
test_v714_capture_core.py — the capture subsystem's pure core.

Settings/templating (CaptureSpec), metadata assembly (CaptureMetadata),
orientation parity with the display path (CaptureOrientation), metadata
embedding (CaptureImageWriter) and the video writer's pacing + verified open
(CaptureVideoWriter). No camera, no codec, no Qt except the orientation-parity
test which composes against the real CameraFeedView helper.
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from SupportClasses.CaptureSpec import (
    CAPTURE_DEFAULTS, TOKEN_NAMES, estimated_video_mb_per_min, merged_settings,
    open_unique, render_template, resolve_output_dir, sanitize_component,
    validate)
from SupportClasses.CaptureOrientation import orient_array
from SupportClasses.CaptureMetadata import (
    CaptureMeta, collect, describe_objective, to_imagej_description,
    to_json, to_text_pairs, to_tokens)
from SupportClasses.CaptureImageWriter import (
    CaptureWriteError, read_embedded_metadata, write_image)
from SupportClasses.CaptureVideoWriter import (
    FOURCC_CHAIN, EncodedVideoWriter, RawFrameSequenceWriter,
    plan_frame_repeats)


class _Tmp(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())


# ── Filename templating + collisions ──────────────────────────────────

class TestNaming(_Tmp):
    def test_renders_tokens(self):
        stem, unknown = render_template(
            "{date}_{camera}_{objective}",
            {"date": "2026-08-07", "camera": "Zyla", "objective": "10x NA 0.3"})
        self.assertEqual(stem, "2026-08-07_Zyla_10x_NA_0.3")
        self.assertEqual(unknown, [])

    def test_empty_token_collapses_separator(self):
        stem, _ = render_template("{date}_{channel}_{camera}",
                                  {"date": "d", "channel": "", "camera": "c"})
        self.assertEqual(stem, "d_c")

    def test_unknown_token_reported_not_swallowed(self):
        stem, unknown = render_template("{date}_{bogus}", {"date": "d"})
        self.assertEqual(stem, "d")
        self.assertEqual(unknown, ["bogus"])

    def test_illegal_characters_and_reserved_names(self):
        self.assertEqual(sanitize_component('a/b:c*d?'), "a_b_c_d")
        self.assertTrue(sanitize_component("CON").endswith("_"))

    def test_empty_template_falls_back(self):
        stem, _ = render_template("{missing}", {})
        self.assertTrue(stem.startswith("capture_"))

    def test_distinct_names_that_sanitize_alike_do_not_overwrite(self):
        """THE trap: the sanitiser is many-to-one on purpose, so two distinct
        captures can render the same stem. They must become two files."""
        a, _ = render_template("{o}", {"o": "10x NA 0.3"})
        b, _ = render_template("{o}", {"o": "10x_NA_0.3"})
        self.assertEqual(a, b)
        p1, f1 = open_unique(self.dir, a, ".png")
        f1.close()
        p2, f2 = open_unique(self.dir, b, ".png")
        f2.close()
        self.assertNotEqual(p1, p2)
        self.assertTrue(p2.name.endswith("_002.png"))
        self.assertEqual(len(list(self.dir.glob("*.png"))), 2)

    def test_open_unique_returns_an_open_handle_to_a_new_file(self):
        p, fh = open_unique(self.dir, "x", ".png")
        fh.write(b"data")
        fh.close()
        self.assertEqual(p.read_bytes(), b"data")

    def test_concurrent_open_unique_never_collides(self):
        """O_EXCL, not exists()-then-write: no TOCTOU window."""
        import threading
        paths, lock = [], threading.Lock()

        def grab():
            p, fh = open_unique(self.dir, "race", ".png")
            fh.close()
            with lock:
                paths.append(p)

        ts = [threading.Thread(target=grab) for _ in range(8)]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        self.assertEqual(len(set(paths)), 8)


class TestSettings(unittest.TestCase):
    def test_defaults_round_trip(self):
        self.assertEqual(merged_settings(None), CAPTURE_DEFAULTS)
        self.assertEqual(merged_settings({}), CAPTURE_DEFAULTS)

    def test_string_false_stays_false(self):
        """bool("false") is True — the type(default)(stored) shortcut used
        elsewhere in this repo would silently flip a persisted 'off' to 'on'."""
        out = merged_settings({"embed_metadata": "false",
                               "write_sidecar": "0",
                               "still_full_res": "true"})
        self.assertFalse(out["embed_metadata"])
        self.assertFalse(out["write_sidecar"])
        self.assertTrue(out["still_full_res"])

    def test_garbage_falls_back_to_default(self):
        out = merged_settings({"video_fps": "not a number"})
        self.assertEqual(out["video_fps"], CAPTURE_DEFAULTS["video_fps"])

    def test_env_override_wins(self):
        os.environ["MEBP_CAPTURE_DIR"] = str(Path(tempfile.mkdtemp()) / "envcap")
        try:
            d = resolve_output_dir({"output_dir": "/somewhere/else",
                                    "subfolder_by_date": False})
            self.assertIn("envcap", str(d))
            self.assertFalse(d.exists())     # never created just by resolving
        finally:
            del os.environ["MEBP_CAPTURE_DIR"]

    def test_date_subfolder(self):
        os.environ.pop("MEBP_CAPTURE_DIR", None)
        d = resolve_output_dir({"output_dir": "", "subfolder_by_date": True})
        self.assertRegex(d.name, r"^\d{4}-\d{2}-\d{2}$")

    def test_raw_png_is_refused_in_validation(self):
        msgs = validate({"still_source": "raw", "still_format": "png"})
        self.assertTrue(any("TIFF" in m for m in msgs))
        self.assertEqual(validate({"still_source": "raw",
                                   "still_format": "tiff",
                                   "video_fps": 15}), [])

    def test_size_estimate_separates_raw_from_encoded(self):
        raw = estimated_video_mb_per_min(2048, 2048, 15, "raw_timelapse")
        enc = estimated_video_mb_per_min(2048, 2048, 15, "display")
        self.assertGreater(raw, 7000)      # ~7.2 GB/min — the honest number
        self.assertLess(enc, raw / 10)


# ── Orientation parity with the display path ──────────────────────────

class TestOrientationParity(unittest.TestCase):
    """A saved 'as seen' image that is mirrored relative to the screen is the
    most likely silent defect in this feature, so the numpy path is composed
    against the REAL Qt display path across every cardinal combination."""

    def _qt_oriented(self, arr, mirrored, flip_y, rot):
        """Run the REAL display path. Deliberately NOT wrapped in a try that
        skips: this suite's first version skipped on an attribute-name typo
        and silently lost its most important guard."""
        from PySide6.QtWidgets import QApplication
        from PySide6.QtGui import QImage
        _app = QApplication.instance() or QApplication(sys.argv)
        from gui.widgets.camera_feed_view import CameraFeedView
        h, w = arr.shape[:2]
        img = QImage(np.ascontiguousarray(arr).data, w, h, 3 * w,
                     QImage.Format_BGR888).copy()
        view = CameraFeedView.__new__(CameraFeedView)
        view._view_mirror = mirrored
        view._view_flip_y = flip_y
        view._view_rot_deg = float(rot)
        view._edge_pick_mode = False
        out, _xf = CameraFeedView._orient_qimage(view, img)
        out = out.convertToFormat(QImage.Format_BGR888)
        ow, oh, stride = out.width(), out.height(), out.bytesPerLine()
        # Qt pads each row to a 4-byte boundary, so the stride is NOT
        # width*3 — slice per row rather than reshaping through it.
        buf = np.frombuffer(out.constBits(), dtype=np.uint8,
                            count=oh * stride).reshape(oh, stride)
        return buf[:, :ow * 3].reshape(oh, ow, 3).copy()

    # A blocky ASYMMETRIC pattern: red TL, green TR, blue BL, black BR. Its
    # four-corner signature is unique across all 8 dihedral transforms, so a
    # reversed rotation or a missed mirror cannot slip through (a symmetric
    # pattern would pass either way).
    _PATTERN = None

    @classmethod
    def _pattern(cls):
        if cls._PATTERN is None:
            a = np.zeros((12, 8, 3), np.uint8)
            a[0:6, 0:4] = (200, 30, 30)
            a[0:6, 4:8] = (30, 200, 30)
            a[6:12, 0:4] = (30, 30, 200)
            cls._PATTERN = a
        return cls._PATTERN

    @staticmethod
    def _corner_signature(img):
        """Colour well inside each corner quadrant — robust to Qt's ±1 canvas
        padding and its smooth-transform edge blending, while still pinning
        the orientation exactly."""
        h, w = img.shape[:2]
        qy, qx = max(1, h // 4), max(1, w // 4)
        return tuple(tuple(int(v) for v in img[y, x])
                     for y, x in ((qy, qx), (qy, w - qx - 1),
                                  (h - qy - 1, qx), (h - qy - 1, w - qx - 1)))

    def test_matches_display_transform_for_every_cardinal_combo(self):
        arr = self._pattern()
        checked = 0
        for mirrored in (False, True):
            for flip_y in (False, True):
                for rot in (0, 90, 180, 270):
                    want = self._qt_oriented(arr, mirrored, flip_y, rot)
                    got = orient_array(arr, mirrored=mirrored, flip_y=flip_y,
                                       rotation_deg=rot)
                    tag = f"mirror={mirrored} flip_y={flip_y} rot={rot}"
                    # Qt's transformed() rounds its canvas UP by a pixel at
                    # some angles — a display artifact a saved file must NOT
                    # reproduce — so allow ±1 and compare the geometry, not
                    # the buffer. A transposed result (rotation applied when
                    # it should not be) still fails: 8 vs 12 is not ±1.
                    for gi, wi, axis in zip(sorted(got.shape[:2]),
                                            sorted(want.shape[:2]), "hw"):
                        self.assertLessEqual(abs(gi - wi), 1,
                                             f"{axis} @ {tag}: {gi} vs {wi}")
                    self.assertEqual(self._corner_signature(got),
                                     self._corner_signature(want),
                                     f"orientation @ {tag}")
                    checked += 1
        self.assertEqual(checked, 16)      # a matcher that checks nothing passes

    def test_the_parity_test_would_catch_a_reversed_rotation(self):
        """Guard on the guard: the corner signature must actually distinguish
        a backwards rotation — this is what the first version got wrong."""
        arr = self._pattern()
        right = orient_array(arr, rotation_deg=90)
        wrong = np.rot90(arr, 1)            # the direction I first shipped
        self.assertEqual(right.shape, wrong.shape)
        self.assertNotEqual(self._corner_signature(right),
                            self._corner_signature(wrong))

    def test_signature_distinguishes_all_eight_dihedral_results(self):
        """If two orientations shared a signature the parity test would be
        blind to swapping them."""
        arr = self._pattern()
        seen = {self._corner_signature(
            orient_array(arr, mirrored=m, flip_y=f, rotation_deg=r))
            for m in (False, True) for f in (False, True)
            for r in (0, 90, 180, 270)}
        self.assertEqual(len(seen), 8)      # 8 distinct dihedral elements

    def test_identity_is_a_passthrough(self):
        a = np.arange(24, dtype=np.uint8).reshape(4, 6)
        np.testing.assert_array_equal(orient_array(a), a)

    def test_mono_16bit_orients_too(self):
        a = np.arange(12, dtype=np.uint16).reshape(3, 4)
        out = orient_array(a, rotation_deg=90)
        self.assertEqual(out.shape, (4, 3))
        self.assertEqual(out.dtype, np.uint16)


# ── Metadata ──────────────────────────────────────────────────────────

class TestMetadata(unittest.TestCase):
    def test_objective_rendered_the_operators_way(self):
        optic = SimpleNamespace(magnification="10", numerical_aperture=0.3,
                                working_distance_mm=16.4, name="Plan Fluor")
        disp, mag, na, wd = describe_objective(optic)
        self.assertEqual(disp, "10x NA 0.3")
        self.assertEqual(mag, "10x")
        self.assertAlmostEqual(na, 0.3)
        self.assertAlmostEqual(wd, 16.4)

    def test_missing_na_never_renders_none(self):
        disp, _m, na, _w = describe_objective(
            SimpleNamespace(magnification="4x", numerical_aperture=None))
        self.assertEqual(disp, "4x")
        self.assertIsNone(na)

    def test_every_source_raising_still_yields_a_record(self):
        class _Boom:
            def __getattr__(self, name):
                def _raise(*a, **k):
                    raise RuntimeError("hardware gone")
                return _raise
        meta = collect(camera_manager=_Boom(), cam_idx=0,
                       captured_wh=(2048, 2048), controller=_Boom(),
                       microscope=_Boom())
        self.assertIsInstance(meta, CaptureMeta)
        self.assertEqual(meta.captured_w, 2048)
        # Absent, not fabricated.
        self.assertNotIn("stage_x_um", meta.as_dict())

    def test_um_per_px_resolved_at_the_CAPTURED_width(self):
        """A full-resolution still has a different µm/px than the live feed;
        stamping the preview's scale would silently mis-size every measurement
        made from the saved image."""
        seen = []

        class _Mgr:
            def camera_identity(self, i):
                return ("andor:SN1", "Zyla")

            def get_hw_settings(self, i):
                return {"exposure_us": 500000, "resolution": (1024, 1024)}

            def effective_um_per_px(self, i, w):
                seen.append(w)
                return 0.65 if w == 2048 else 1.30

        meta = collect(camera_manager=_Mgr(), cam_idx=0,
                       captured_wh=(2048, 2048))
        self.assertEqual(seen, [2048])
        self.assertAlmostEqual(meta.um_per_px, 0.65)

    def test_context_extra_wins_over_hardware(self):
        meta = collect(extra={"channel": "FITC", "well": "A1"})
        self.assertEqual(meta.channel, "FITC")
        self.assertEqual(to_tokens(meta)["well"], "A1")

    def test_tokens_cover_every_advertised_name(self):
        tok = to_tokens(collect(), n=3)
        for name in TOKEN_NAMES:
            self.assertIn(name, tok, f"token {{{name}}} advertised but absent")
        self.assertEqual(tok["n"], "003")

    def test_imagej_description_has_header_then_json(self):
        meta = CaptureMeta(um_per_px=0.65, objective="10x NA 0.3")
        desc = to_imagej_description(meta)
        self.assertTrue(desc.startswith("ImageJ="))
        self.assertIn("unit=um", desc)
        body = desc.split("\n\n", 1)[1]
        self.assertAlmostEqual(json.loads(body)["um_per_px"], 0.65)


# ── Image writing + embedded metadata ─────────────────────────────────

class TestImageWriter(_Tmp):
    def _meta(self):
        return CaptureMeta(kind="still", objective="10x NA 0.3",
                           um_per_px=0.65, exposure_us=500000.0,
                           channel="FITC", notes="µm scale · 20 °C")

    def test_png_round_trip(self):
        p, fh = open_unique(self.dir, "shot", ".png")
        fh.close()
        write_image(p, np.zeros((8, 8, 3), np.uint8), self._meta())
        got = read_embedded_metadata(p)
        self.assertEqual(got["objective"], "10x NA 0.3")
        self.assertAlmostEqual(got["um_per_px"], 0.65)

    def test_non_latin1_survives(self):
        """tEXt is latin-1 only — 'µm' must go through iTXt or it is lost."""
        p, fh = open_unique(self.dir, "unicode", ".png")
        fh.close()
        write_image(p, np.zeros((4, 4, 3), np.uint8), self._meta())
        self.assertIn("µm", read_embedded_metadata(p)["notes"])

    def test_tiff_round_trip_and_imagej_scale(self):
        from PIL import Image
        p, fh = open_unique(self.dir, "shot", ".tif")
        fh.close()
        write_image(p, np.zeros((8, 8, 3), np.uint8), self._meta())
        self.assertEqual(read_embedded_metadata(p)["objective"], "10x NA 0.3")
        with Image.open(str(p)) as im:
            self.assertEqual(int(im.tag_v2[296]), 3)          # unit = cm
            # ImageJ's Set Scale reads this: pixels per cm from our µm/px.
            self.assertAlmostEqual(float(im.tag_v2[282]), 10000.0 / 0.65,
                                   places=1)

    def test_16bit_tiff_is_bit_exact(self):
        rng = np.random.default_rng(3)
        a = rng.integers(0, 65535, size=(16, 16)).astype(np.uint16)
        p, fh = open_unique(self.dir, "raw", ".tif")
        fh.close()
        write_image(p, a, self._meta())
        from PIL import Image
        with Image.open(str(p)) as im:
            np.testing.assert_array_equal(np.array(im), a)

    def test_16bit_png_is_refused_not_silently_rewritten(self):
        p, fh = open_unique(self.dir, "raw", ".png")
        fh.close()
        with self.assertRaises(CaptureWriteError):
            write_image(p, np.zeros((4, 4), np.uint16), self._meta())

    def test_sidecar_named_for_the_container(self):
        """foo.png.json, not foo.json — a PNG and a TIFF captured in the same
        second would otherwise fight over one sidecar."""
        p, fh = open_unique(self.dir, "s", ".png")
        fh.close()
        write_image(p, np.zeros((4, 4, 3), np.uint8), self._meta())
        self.assertTrue((self.dir / "s.png.json").exists())
        self.assertFalse((self.dir / "s.json").exists())

    def test_sidecar_written_even_when_embedding_is_off(self):
        p, fh = open_unique(self.dir, "s", ".png")
        fh.close()
        write_image(p, np.zeros((4, 4, 3), np.uint8), self._meta(), embed=False)
        side = json.loads((self.dir / "s.png.json").read_text(encoding="utf-8"))
        self.assertEqual(side["objective"], "10x NA 0.3")


# ── Video ─────────────────────────────────────────────────────────────

class _FakeWriter:
    def __init__(self, opens=True):
        self._opens = opens
        self.written = 0
        self.released = False

    def isOpened(self):
        return self._opens

    def write(self, frame):
        self.written += 1

    def release(self):
        self.released = True

    def set(self, prop, val):
        return True

    def get(self, prop):
        return 80.0


class TestFrameRatePacing(unittest.TestCase):
    """The frame rate is the PLAYBACK rate — a slow camera repeats frames so
    the file plays at real time instead of silent slow motion."""

    def test_steady_camera_writes_one(self):
        self.assertEqual(plan_frame_repeats(1.0, 15.0, 15), 1)

    def test_slow_camera_repeats(self):
        # 1 s in at 15 fps we should hold 16 frames; only 4 written.
        self.assertEqual(plan_frame_repeats(1.0, 15.0, 4), 8)   # clamped

    def test_fast_camera_drops(self):
        self.assertEqual(plan_frame_repeats(1.0, 15.0, 40), 0)

    def test_stall_is_clamped(self):
        self.assertEqual(plan_frame_repeats(60.0, 15.0, 0, max_repeat=8), 8)

    def test_start_of_recording_writes_the_first_frame(self):
        self.assertEqual(plan_frame_repeats(0.0, 15.0, 0), 1)


class TestEncodedVideoWriter(_Tmp):
    def test_avc1_is_never_attempted(self):
        """It reports isOpened() True while OpenH264 fails to load, producing
        a file that never plays. Named here so re-adding it fails a test."""
        for chain in FOURCC_CHAIN.values():
            self.assertNotIn("avc1", chain)
            self.assertNotIn("H264", [c.upper() for c in chain])

    def test_falls_through_to_a_working_fourcc(self):
        tried = []

        def factory(path, fourcc, fps, size):
            tried.append(fourcc)
            return _FakeWriter(opens=(fourcc == "XVID"))

        w = EncodedVideoWriter(self.dir / "v.mp4", 15, (64, 64),
                               writer_factory=factory)
        self.assertTrue(w.open())
        self.assertEqual(w.fourcc, "XVID")
        self.assertTrue(w.container_substituted)
        self.assertEqual(w.path.suffix, ".avi")
        self.assertEqual(tried, ["mp4v", "XVID"])

    def test_open_verified_not_assumed(self):
        """A writer that never opens must not be used."""
        w = EncodedVideoWriter(self.dir / "v.mp4", 15, (64, 64),
                               writer_factory=lambda *a: _FakeWriter(False))
        self.assertFalse(w.open())

    def test_size_mismatch_refused(self):
        w = EncodedVideoWriter(self.dir / "v.avi", 15, (64, 64),
                               writer_factory=lambda *a: _FakeWriter())
        w.open()
        self.assertTrue(w.add(np.zeros((64, 64, 3), np.uint8)))
        self.assertFalse(w.add(np.zeros((32, 32, 3), np.uint8)))

    def test_zero_frame_file_is_deleted_not_left(self):
        path = self.dir / "v.avi"
        w = EncodedVideoWriter(path, 15, (64, 64),
                               writer_factory=lambda *a: _FakeWriter())
        w.open()
        path.write_bytes(b"")
        res = w.close()
        self.assertFalse(res.ok)
        self.assertFalse(path.exists())
        self.assertIn("no frames", res.reason)

    def test_real_file_reports_ok(self):
        path = self.dir / "v.avi"
        w = EncodedVideoWriter(path, 15, (64, 64),
                               writer_factory=lambda *a: _FakeWriter())
        w.open()
        w.add(np.zeros((64, 64, 3), np.uint8), repeats=30)
        path.write_bytes(b"x" * 5000)
        res = w.close()
        self.assertTrue(res.ok)
        self.assertEqual(res.frames, 30)
        self.assertAlmostEqual(res.duration_s, 2.0)
        self.assertIn("30 frames", res.describe())


class TestRawSequence(_Tmp):
    def test_writes_frames_and_a_manifest(self):
        rec = RawFrameSequenceWriter(self.dir / "seq", CaptureMeta(),
                                     interval_s=1.0)
        for i in range(3):
            self.assertTrue(rec.add(np.full((8, 8), 100 * i + 1, np.uint16),
                                    t_wall=1000.0 + i, t_mono=float(i)))
        res = rec.close()
        self.assertTrue(res.ok)
        self.assertEqual(res.frames, 3)
        man = json.loads((self.dir / "seq" / "manifest.json").read_text())
        self.assertEqual(man["kind"], "raw_timelapse")
        self.assertEqual(len(man["frames"]), 3)
        # Real per-frame timestamps — the sequence never claims to be
        # continuous video.
        self.assertEqual([f["t_mono"] for f in man["frames"]], [0.0, 1.0, 2.0])

    def test_empty_sequence_cleans_up(self):
        rec = RawFrameSequenceWriter(self.dir / "empty")
        res = rec.close()
        self.assertFalse(res.ok)


if __name__ == "__main__":
    unittest.main()
