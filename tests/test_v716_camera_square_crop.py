"""v7.16 — square sensor crop, applied at the frame source.

Operator: *"it turns out that the image sensor is too wide, and we see some of
the dark circles of the field of view. can we crop the image into a square in
the camera calibration part of the camera detection settings page. this cropping
should be assigned to all surfaces including mosaic, live view, image, recording
etc."*

The two things that have to be true:

1. **Every surface sees the crop.** There are exactly three points where a frame
   leaves ``CameraWidget`` — the display grab (which feeds the raw cache, the
   live view and the video recorder), ``capture_fresh_frame`` (mosaic tiles,
   calibration grabs) and ``capture_raw_average`` (raw stills). All three are
   pinned here; missing one is how a raw still ends up being the only image in
   the app that still shows the vignetted edge.

2. **µm/px does not move.** Cropping removes pixels; it does not change what a
   pixel spans. Four separate sites rescale a stored µm/px by frame width, and
   feeding a cropped width into any of them scales the result by the crop
   fraction — 27 % on this rig's Tucsen, in the direction that makes a mosaic
   believe its field is wider than it is and step past its own tiles.
"""

from __future__ import annotations

import os
import sys
import threading
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np                                          # noqa: E402
from PySide6.QtWidgets import QApplication                  # noqa: E402

from SupportClasses.CameraCrop import (                     # noqa: E402
    CameraCrop, NO_CROP, MIN_SIDE_PX, MODE_NONE, MODE_SQUARE)


def _app():
    return QApplication.instance() or QApplication([])


# This rig's Tucsen Libra 25 in its default (2x2 binned) mode, and the 4x
# objective calibration measured on it.
TUCSEN_W, TUCSEN_H = 2600, 2048
TUCSEN_4X_UM_PER_PX = 1.2710


# ─────────────────────────────────────────────────────────────────────────────
#  The model
# ─────────────────────────────────────────────────────────────────────────────

class TestCropGeometry(unittest.TestCase):

    def test_square_crop_of_a_wide_sensor_keeps_the_short_side(self):
        c = CameraCrop(mode=MODE_SQUARE, scale=1.0)
        self.assertEqual(c.size_for(TUCSEN_W, TUCSEN_H), (2048, 2048))

    def test_scale_shrinks_the_square(self):
        c = CameraCrop(mode=MODE_SQUARE, scale=0.75)
        self.assertEqual(c.size_for(TUCSEN_W, TUCSEN_H), (1536, 1536))

    def test_the_centre_is_preserved_exactly(self):
        """The click→stage map measures from the frame CENTRE, so a crop that
        moved it by even half a pixel would put a constant offset into every
        conversion. Checked across odd/even frame sizes and several scales."""
        for w, h in [(2600, 2048), (2601, 2048), (2600, 2049), (2601, 2049),
                     (5200, 4096), (1920, 1080), (3664, 2748)]:
            for scale in (1.0, 0.9, 0.8, 0.75, 0.5, 0.33):
                c = CameraCrop(mode=MODE_SQUARE, scale=scale)
                x0, y0, cw, ch = c.rect_for(w, h)
                self.assertAlmostEqual(x0 + cw / 2.0, w / 2.0, places=9,
                                       msg=f"{w}x{h} @ {scale}")
                self.assertAlmostEqual(y0 + ch / 2.0, h / 2.0, places=9,
                                       msg=f"{w}x{h} @ {scale}")

    def test_crop_never_leaves_the_frame(self):
        for w, h in [(2600, 2048), (640, 480), (100, 100), (2048, 2600)]:
            for scale in (1.0, 0.5, 0.1):
                c = CameraCrop(mode=MODE_SQUARE, scale=scale)
                x0, y0, cw, ch = c.rect_for(w, h)
                self.assertGreaterEqual(x0, 0)
                self.assertGreaterEqual(y0, 0)
                self.assertLessEqual(x0 + cw, w)
                self.assertLessEqual(y0 + ch, h)

    def test_an_absurd_scale_is_clamped_not_honoured(self):
        """A degenerate frame is useless for registration and detection, so the
        model floors the side rather than delivering a handful of pixels."""
        c = CameraCrop(mode=MODE_SQUARE, scale=0.1)
        cw, ch = c.size_for(64, 64)
        self.assertGreaterEqual(min(cw, ch), MIN_SIDE_PX)

    def test_disabled_crop_returns_the_very_same_array(self):
        f = np.zeros((TUCSEN_H, TUCSEN_W, 3), np.uint8)
        self.assertIs(NO_CROP.apply(f), f)
        self.assertIsNone(NO_CROP.rect_for(TUCSEN_W, TUCSEN_H))

    def test_full_square_crop_of_an_already_square_frame_is_not_active(self):
        self.assertFalse(
            CameraCrop(mode=MODE_SQUARE, scale=1.0).is_active_for(2048, 2048))
        self.assertTrue(
            CameraCrop(mode=MODE_SQUARE, scale=1.0).is_active_for(2600, 2048))

    def test_raw_mono_and_colour_frames_crop_identically(self):
        """A raw uint16 still and a BGR display frame must land on the same
        pixels, or the two disagree about where the specimen is."""
        c = CameraCrop(mode=MODE_SQUARE, scale=0.8)
        bgr = c.apply(np.zeros((TUCSEN_H, TUCSEN_W, 3), np.uint8))
        raw = c.apply(np.zeros((TUCSEN_H, TUCSEN_W), np.uint16))
        self.assertEqual(bgr.shape[:2], raw.shape[:2])
        self.assertEqual(raw.dtype, np.uint16)

    def test_it_crops_the_pixels_the_operator_asked_about(self):
        """The dark band is at the left and right of a wide sensor; a square
        crop must remove exactly that and keep the lit centre."""
        f = np.zeros((TUCSEN_H, TUCSEN_W), np.uint8)
        f[:, 276:276 + 2048] = 200          # the illuminated square
        out = CameraCrop(mode=MODE_SQUARE, scale=1.0).apply(f)
        self.assertEqual(out.shape, (2048, 2048))
        self.assertTrue((out == 200).all(), "dark edge survived the crop")


class TestCropPlacement(unittest.TestCase):
    """*"i need to choose where the crop occurs in the frame"* — the lit circle
    is centred on the optical axis, which need not be the middle of the sensor."""

    def test_the_offset_moves_the_box(self):
        base = CameraCrop(mode=MODE_SQUARE, scale=1.0).rect_for(TUCSEN_W, TUCSEN_H)
        right = CameraCrop(mode=MODE_SQUARE, scale=1.0,
                           offset_x=0.05).rect_for(TUCSEN_W, TUCSEN_H)
        self.assertEqual(right[0] - base[0], int(round(0.05 * TUCSEN_W)))
        self.assertEqual(right[2:], base[2:], "the size must not change")

    def test_the_box_is_clamped_inside_the_frame(self):
        """A crop reaching past the sensor edge would deliver undefined pixels."""
        for ox, oy in [(0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)]:
            for scale in (1.0, 0.6, 0.25):
                c = CameraCrop(mode=MODE_SQUARE, scale=scale,
                               offset_x=ox, offset_y=oy)
                x0, y0, cw, ch = c.rect_for(TUCSEN_W, TUCSEN_H)
                self.assertGreaterEqual(x0, 0)
                self.assertGreaterEqual(y0, 0)
                self.assertLessEqual(x0 + cw, TUCSEN_W)
                self.assertLessEqual(y0 + ch, TUCSEN_H)

    def test_center_offset_px_reports_the_ACHIEVED_shift_not_the_request(self):
        """At the clamp the requested and applied offsets differ, and it is the
        applied one that has to be compensated for. Deriving the compensation
        from ``offset_x`` instead would drift from reality exactly at the edge."""
        c = CameraCrop(mode=MODE_SQUARE, scale=1.0, offset_x=0.5)
        requested = 0.5 * TUCSEN_W                       # 1300 px — impossible
        achieved = c.center_offset_px(TUCSEN_W, TUCSEN_H)[0]
        self.assertLess(achieved, requested)
        # The square is 2048 wide on a 2600 frame, so the slack each way is 276.
        self.assertAlmostEqual(achieved, (TUCSEN_W - 2048) / 2.0, places=6)

    def test_center_offset_is_zero_when_centred(self):
        for scale in (1.0, 0.8, 0.5):
            c = CameraCrop(mode=MODE_SQUARE, scale=scale)
            self.assertEqual(c.center_offset_px(TUCSEN_W, TUCSEN_H), (0.0, 0.0))
        self.assertEqual(NO_CROP.center_offset_px(TUCSEN_W, TUCSEN_H), (0.0, 0.0))

    def test_the_reference_pixel_follows_the_crop(self):
        """Where the stage is pointing, in cropped-frame coordinates — the frame
        middle only while the crop is centred."""
        c = CameraCrop(mode=MODE_SQUARE, scale=1.0)
        self.assertEqual(c.reference_pixel(TUCSEN_W, TUCSEN_H), (1024.0, 1024.0))
        c = CameraCrop(mode=MODE_SQUARE, scale=1.0, offset_x=0.05)
        rx, _ry = c.reference_pixel(TUCSEN_W, TUCSEN_H)
        dx, _dy = c.center_offset_px(TUCSEN_W, TUCSEN_H)
        self.assertAlmostEqual(rx, 1024.0 - dx, places=6)
        # And it is genuinely the sensor centre: x0 + rx == W/2
        x0, _y0, _cw, _ch = c.rect_for(TUCSEN_W, TUCSEN_H)
        self.assertAlmostEqual(x0 + rx, TUCSEN_W / 2.0, places=6)

    def test_an_offset_crop_still_crops_only_real_pixels(self):
        f = np.arange(TUCSEN_H * TUCSEN_W, dtype=np.int64).reshape(
            TUCSEN_H, TUCSEN_W)
        c = CameraCrop(mode=MODE_SQUARE, scale=0.5, offset_x=0.2, offset_y=-0.15)
        x0, y0, cw, ch = c.rect_for(TUCSEN_W, TUCSEN_H)
        out = c.apply(f)
        self.assertEqual(out.shape, (ch, cw))
        self.assertEqual(int(out[0, 0]), int(f[y0, x0]))


class TestCropPersistenceModel(unittest.TestCase):

    def test_round_trip(self):
        c = CameraCrop(mode=MODE_SQUARE, scale=0.85)
        self.assertEqual(CameraCrop.from_dict(c.to_dict()), c)

    def test_junk_never_crops_by_accident(self):
        """An unexpected crop is a silently wrong field of view; no crop is
        merely the old behaviour. Anything unreadable must mean the latter."""
        for junk in (None, "square", 5, [], {"mode": "wat"},
                     {"mode": "square", "scale": "eight"}):
            got = CameraCrop.from_dict(junk)
            if isinstance(junk, dict) and junk.get("mode") == "square":
                self.assertTrue(got.enabled)      # a valid mode with bad scale
                self.assertEqual(got.scale, 1.0)
            else:
                self.assertFalse(got.enabled, junk)

    def test_scale_is_clamped_into_range(self):
        self.assertEqual(CameraCrop(mode=MODE_SQUARE, scale=99).scale, 1.0)
        self.assertEqual(CameraCrop(mode=MODE_SQUARE, scale=-3).scale, 0.10)
        self.assertEqual(CameraCrop(mode=MODE_SQUARE,
                                    scale=float("nan")).scale, 1.0)

    def test_offsets_round_trip_and_junk_falls_back_to_centred(self):
        c = CameraCrop(mode=MODE_SQUARE, scale=0.8,
                       offset_x=-0.07, offset_y=0.02)
        self.assertEqual(CameraCrop.from_dict(c.to_dict()), c)
        junk = CameraCrop.from_dict(
            {"mode": "square", "offset_x": "left", "offset_y": None})
        self.assertEqual((junk.offset_x, junk.offset_y), (0.0, 0.0))
        self.assertEqual(
            CameraCrop(mode=MODE_SQUARE, offset_x=9.0).offset_x, 0.5)

    def test_a_centred_crop_writes_no_offset_keys(self):
        """So an entry written before offsets existed round-trips byte-identically
        and nothing has to be migrated."""
        self.assertEqual(CameraCrop(mode=MODE_SQUARE, scale=0.8).to_dict(),
                         {"mode": "square", "scale": 0.8})
        self.assertIn("offset_x", CameraCrop(mode=MODE_SQUARE,
                                             offset_x=0.05).to_dict())


# ─────────────────────────────────────────────────────────────────────────────
#  "assigned to all surfaces" — the three frame egress points
# ─────────────────────────────────────────────────────────────────────────────

class _FakeCapture:
    """An OpenCV-shaped source delivering a marked wide frame."""

    def __init__(self, w=TUCSEN_W, h=TUCSEN_H):
        self.w, self.h = w, h
        self.reads = 0

    def isOpened(self):
        return True

    def read(self):
        self.reads += 1
        f = np.zeros((self.h, self.w, 3), np.uint8)
        f[:, :] = 30                                   # dark surround
        x0 = (self.w - min(self.w, self.h)) // 2
        f[:, x0:x0 + min(self.w, self.h)] = 180        # lit field
        return True, f


class _FakeMono:
    def __init__(self, w=TUCSEN_W, h=TUCSEN_H):
        self.w, self.h = w, h

    def capture_raw_average(self, n, timeout_s=10.0):
        return np.full((self.h, self.w), 1234, np.uint16)


def _bare_widget(capture=None):
    """A ``CameraWidget`` with only the frame-path state the egress points use.

    Deliberately the PRODUCTION class rather than a stand-in: the claim under
    test is that the real widget crops, and a stand-in that crops proves
    nothing about it.
    """
    from gui.widgets.camera_widget import CameraWidget
    w = CameraWidget.__new__(CameraWidget)
    w._frame_lock = threading.Lock()
    w._current_frame = None
    w._frame_seq = 0
    w._frame_src_seq = None
    w._crop = CameraCrop()
    w._capture_size = None
    w._backend_type = "opencv"
    w._capture = capture if capture is not None else _FakeCapture()
    return w


class TestEveryFrameEgressPointCrops(unittest.TestCase):
    """One missed egress point = one surface showing different pixels."""

    def setUp(self):
        _app()

    def test_capture_fresh_frame_crops(self):
        """The mosaic tile path. This is what makes the scan honest."""
        w = _bare_widget()
        self.assertEqual(w.capture_fresh_frame().shape[:2],
                         (TUCSEN_H, TUCSEN_W))
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        self.assertEqual(w.capture_fresh_frame().shape[:2], (2048, 2048))

    def test_capture_fresh_frame_crops_on_the_discard_fallback_too(self):
        """``capture_fresh_frame`` can return the last discarded frame when the
        final read fails; that path must crop as well."""
        w = _bare_widget()
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        real = w._capture.read
        calls = {"n": 0}

        def flaky():
            calls["n"] += 1
            return real() if calls["n"] == 1 else (False, None)

        w._capture.read = flaky
        got = w.capture_fresh_frame(discard_n_frames=1)
        self.assertIsNotNone(got)
        self.assertEqual(got.shape[:2], (2048, 2048))

    def test_capture_raw_average_crops(self):
        """The raw still path reads the sensor buffer straight from the backend
        — the one that would otherwise stay uncropped."""
        w = _bare_widget()
        w._mono_display_backend = lambda: _FakeMono()
        self.assertEqual(w.capture_raw_average(2).shape, (TUCSEN_H, TUCSEN_W))
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        self.assertEqual(w.capture_raw_average(2).shape, (2048, 2048))

    def test_the_display_grab_crops_the_cache_the_pixmap_and_the_signal(self):
        """``_grab_frame`` feeds three consumers at once: the raw cache
        (detection / mosaic / calibration), the on-screen pixmap (live view) and
        the ``frame_captured`` signal (video recording). All three come off the
        same array, so all three are pinned together here."""
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget(show_controls=False)
        w._capture = _FakeCapture()
        w._backend_type = "opencv"
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))

        seen = []
        w.frame_captured.connect(lambda img: seen.append((img.width(),
                                                          img.height())))
        w._grab_frame()

        cached = w.get_current_frame()
        self.assertEqual(cached.shape[:2], (2048, 2048),
                         "raw cache (detection / mosaic) is not cropped")
        self.assertEqual(seen[-1], (2048, 2048),
                         "recording / live-view frame is not cropped")
        w.deleteLater()

    def test_capture_size_reports_the_pre_crop_frame(self):
        """µm/px is stamped against, and rescaled by, the CAPTURE size — so it
        must survive the crop."""
        w = _bare_widget()
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=0.5))
        w.capture_fresh_frame()
        self.assertEqual(w.capture_size(), (TUCSEN_W, TUCSEN_H))

    def test_no_crop_configured_leaves_the_frame_untouched(self):
        w = _bare_widget()
        self.assertEqual(w.capture_fresh_frame().shape[:2],
                         (TUCSEN_H, TUCSEN_W))
        self.assertFalse(w.crop().enabled)


# ─────────────────────────────────────────────────────────────────────────────
#  µm/px must not move — the dangerous part
# ─────────────────────────────────────────────────────────────────────────────

def _mgr_with_crop(capture=(TUCSEN_W, TUCSEN_H)):
    """A real ``CameraManager`` (``__new__``, frame-path fields only) whose
    slot 0 owns a real ``CameraWidget``."""
    from gui.widgets.camera_manager import CameraManager
    from gui.widgets.camera_widget import CameraWidget
    mgr = CameraManager.__new__(CameraManager)
    mgr._max_cameras = 4
    mgr._um_per_px = [1.67] * 4
    mgr._um_per_px_set = [False] * 4
    mgr._um_per_px_res = [None] * 4
    mgr._rotation_deg = [None] * 4
    mgr._mirrored = [False] * 4
    mgr._flip_y = [False] * 4
    mgr._crop = [None] * 4
    w = CameraWidget.__new__(CameraWidget)
    w._frame_lock = threading.Lock()
    w._crop = CameraCrop()
    w._capture_size = tuple(capture)
    mgr._cameras = [w, None, None, None]
    mgr._widget = lambda i: (w if i == 0 else None)
    return mgr, w


class TestUmPerPxIsInvariantUnderCropping(unittest.TestCase):
    """A crop removes pixels; it does not change what a pixel spans.

    Each rescale site is checked separately, because each has its own copy of
    the width ratio and one of them being wrong is enough to mis-scale a scan.
    """

    def setUp(self):
        _app()

    def test_camera_manager_effective_um_per_px(self):
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        before = mgr.effective_um_per_px(0, TUCSEN_W)
        self.assertAlmostEqual(before, TUCSEN_4X_UM_PER_PX, places=9)

        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        after = mgr.effective_um_per_px(0, 2048)
        self.assertAlmostEqual(after, TUCSEN_4X_UM_PER_PX, places=9)

        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=0.6))
        tighter = mgr.effective_um_per_px(0, 1228)
        self.assertAlmostEqual(tighter, TUCSEN_4X_UM_PER_PX, places=9)

    def test_the_naive_answer_would_have_been_visibly_wrong(self):
        """Guards the guard: if a delivered width were used, µm/px would be 27 %
        high — so a test that passes with the correct code would FAIL with the
        naive one. Without this the invariance test above could pass for the
        wrong reason (e.g. because nothing rescales at all)."""
        naive = TUCSEN_4X_UM_PER_PX * TUCSEN_W / 2048.0
        self.assertGreater(abs(naive - TUCSEN_4X_UM_PER_PX)
                           / TUCSEN_4X_UM_PER_PX, 0.25)

    def test_a_real_resolution_change_still_rescales(self):
        """The crop must not disable the rescale it shares a formula with:
        turning 2x2 binning off genuinely halves µm/px."""
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        w._capture_size = (5200, 4096)                 # 1x1, crop still on
        self.assertAlmostEqual(mgr.effective_um_per_px(0, 4096),
                               TUCSEN_4X_UM_PER_PX / 2.0, places=9)

    def test_mosaic_calibration_resolve(self):
        from SupportClasses.MosaicCalibration import resolve

        class ObjStore:
            def get_calibration(self, cam, obj):
                return {"measured_um_per_px": TUCSEN_4X_UM_PER_PX,
                        "resolution": [TUCSEN_W, TUCSEN_H]}

        mgr, w = _mgr_with_crop()
        mgr.camera_identity = lambda i: ("tucam:0", "Tucsen Libra 25")

        full = resolve(camera_manager=mgr, cam_idx=0, objective="4x",
                       live_resolution=(TUCSEN_W, TUCSEN_H),
                       obj_store=ObjStore())
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        cropped = resolve(camera_manager=mgr, cam_idx=0, objective="4x",
                          live_resolution=(2048, 2048), obj_store=ObjStore())

        self.assertAlmostEqual(full.um_per_px, cropped.um_per_px, places=9)
        self.assertAlmostEqual(cropped.um_per_px, TUCSEN_4X_UM_PER_PX,
                               places=9)

    def test_capture_width_px_falls_back_when_there_is_no_manager(self):
        """Every legacy caller — no manager, or a manager predating this — must
        get the width it passed in, unchanged."""
        from SupportClasses.MosaicCalibration import capture_width_px
        self.assertEqual(capture_width_px(None, None, 916.0), 916.0)
        self.assertEqual(capture_width_px(object(), 0, 916.0), 916.0)


class TestTheFieldOfViewDoesShrink(unittest.TestCase):
    """The whole point: the same µm/px over fewer pixels is a smaller field."""

    def setUp(self):
        _app()

    def test_fov_loses_the_dark_band_and_nothing_else(self):
        from SupportClasses.MosaicCalibration import resolve

        class ObjStore:
            def get_calibration(self, cam, obj):
                return {"measured_um_per_px": TUCSEN_4X_UM_PER_PX,
                        "resolution": [TUCSEN_W, TUCSEN_H]}

        mgr, w = _mgr_with_crop()
        mgr.camera_identity = lambda i: ("tucam:0", "Tucsen Libra 25")
        full = resolve(camera_manager=mgr, cam_idx=0, objective="4x",
                       live_resolution=(TUCSEN_W, TUCSEN_H),
                       obj_store=ObjStore())
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        cropped = resolve(camera_manager=mgr, cam_idx=0, objective="4x",
                          live_resolution=(2048, 2048), obj_store=ObjStore())

        self.assertAlmostEqual(full.fov_um[1], cropped.fov_um[1], places=6,
                               msg="the short axis was already square")
        self.assertLess(cropped.fov_um[0], full.fov_um[0])
        self.assertAlmostEqual(cropped.fov_um[0], cropped.fov_um[1], places=6)

    def test_a_crop_is_reported_as_a_crop_not_as_a_rescale(self):
        """``was_rescaled`` drives an operator-facing note. A crop changes the
        delivered width with no rescale having happened, and saying "rescaled"
        would point them at the wrong thing."""
        from SupportClasses.MosaicCalibration import MosaicCalibration
        cal = MosaicCalibration(
            um_per_px=TUCSEN_4X_UM_PER_PX, base_um_per_px=TUCSEN_4X_UM_PER_PX,
            calib_resolution=(TUCSEN_W, TUCSEN_H),
            live_resolution=(2048, 2048),
            capture_resolution=(TUCSEN_W, TUCSEN_H))
        self.assertTrue(cal.is_cropped)
        self.assertFalse(cal.was_rescaled)
        self.assertIn("cropped from", cal.describe())


class TestClickToStageSurvivesTheCrop(unittest.TestCase):
    """A centred crop keeps the optical centre at the centre, so the click→stage
    map is unchanged. If the crop were off-centre this is what would break."""

    def setUp(self):
        _app()

    def test_the_frame_centre_still_maps_to_zero_offset(self):
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        dx, dy = mgr.pixel_to_stage_offset(0, 1024.0, 1024.0, 2048, 2048)
        self.assertAlmostEqual(dx, 0.0, places=9)
        self.assertAlmostEqual(dy, 0.0, places=9)

    def test_a_feature_keeps_its_stage_offset_across_the_crop(self):
        """The same physical point, addressed in the uncropped and the cropped
        frame, must resolve to the same stage offset."""
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        crop = CameraCrop(mode=MODE_SQUARE, scale=1.0)
        x0, y0, cw, ch = crop.rect_for(TUCSEN_W, TUCSEN_H)

        for px, py in [(1300.0, 1024.0), (900.0, 700.0), (1700.0, 1500.0)]:
            full = mgr.pixel_to_stage_offset(0, px, py, TUCSEN_W, TUCSEN_H)
            w.set_crop(crop)
            crp = mgr.pixel_to_stage_offset(0, px - x0, py - y0, cw, ch)
            w.set_crop(CameraCrop())
            self.assertAlmostEqual(full[0], crp[0], places=6)
            self.assertAlmostEqual(full[1], crp[1], places=6)

    def test_the_inverse_map_agrees_too(self):
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        mgr.set_rotation_deg(0, -90.0)
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        for px, py in [(100.0, 200.0), (1900.0, 40.0), (1024.0, 1024.0)]:
            dx, dy = mgr.pixel_to_stage_offset(0, px, py, 2048, 2048)
            bx, by = mgr.stage_offset_to_pixel(0, dx, dy, 2048, 2048)
            self.assertAlmostEqual(bx, px, places=6)
            self.assertAlmostEqual(by, py, places=6)


class TestAnOffsetCropIsCompensated(unittest.TestCase):
    """Moving the crop moves the pixel that means "the stage is here".

    Up to ~1 mm at a small crop on this rig — a silently relocated plate
    calibration if it were left uncorrected. It is cancelled instead, so
    re-aiming the crop is a view change and nothing more.
    """

    def setUp(self):
        _app()

    def test_the_same_physical_point_resolves_to_the_same_stage_offset(self):
        """The invariant, over every orientation the camera can be in. If this
        holds, no taught coordinate can move when the crop is re-aimed."""
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        for rot in (0.0, -90.0, 90.0, 180.0, 37.5):
            for fx, fy in ((False, False), (True, False), (False, True),
                           (True, True)):
                mgr.set_rotation_deg(0, rot)
                # NOTE ``get_mirrored`` delegates to the WIDGET, so setting only
                # the manager's cache leaves flip_x False and silently drops
                # half this matrix (a mutation caught exactly that).
                mgr.set_mirrored(0, fx)
                mgr._flip_y[0] = fy
                for ox, oy in ((0.08, 0.0), (-0.08, 0.05), (0.5, -0.5)):
                    for scale in (1.0, 0.7):
                        crop = CameraCrop(mode=MODE_SQUARE, scale=scale,
                                          offset_x=ox, offset_y=oy)
                        x0, y0, cw, ch = crop.rect_for(TUCSEN_W, TUCSEN_H)
                        for px, py in ((x0 + 10, y0 + 10),
                                       (x0 + cw - 10, y0 + ch - 10),
                                       (x0 + cw / 2, y0 + ch / 2)):
                            w.set_crop(CameraCrop())
                            full = mgr.pixel_to_stage_offset(
                                0, px, py, TUCSEN_W, TUCSEN_H)
                            w.set_crop(crop)
                            crp = mgr.pixel_to_stage_offset(
                                0, px - x0, py - y0, cw, ch)
                            self.assertAlmostEqual(
                                full[0], crp[0], places=6,
                                msg=f"rot={rot} fx={fx} fy={fy} off=({ox},{oy})")
                            self.assertAlmostEqual(full[1], crp[1], places=6)

    def test_the_uncompensated_error_would_have_been_large(self):
        """Guards the guard: without the compensation the invariance test above
        would be off by hundreds of µm, so it cannot be passing vacuously."""
        crop = CameraCrop(mode=MODE_SQUARE, scale=0.7, offset_x=0.2)
        dx, _dy = crop.center_offset_px(TUCSEN_W, TUCSEN_H)
        self.assertGreater(abs(dx) * TUCSEN_4X_UM_PER_PX, 300.0)

    def test_the_cropped_frame_centre_is_no_longer_the_stage_position(self):
        """The honest consequence, stated: with the crop moved, the middle of the
        picture is NOT where the stage points — and the map says so."""
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        crop = CameraCrop(mode=MODE_SQUARE, scale=1.0, offset_x=0.08)
        w.set_crop(crop)
        _x0, _y0, cw, ch = crop.rect_for(TUCSEN_W, TUCSEN_H)
        dx_px, _ = crop.center_offset_px(TUCSEN_W, TUCSEN_H)
        got = mgr.pixel_to_stage_offset(0, cw / 2, ch / 2, cw, ch)
        self.assertAlmostEqual(got[0], dx_px * TUCSEN_4X_UM_PER_PX, places=6)
        self.assertNotAlmostEqual(got[0], 0.0, places=1)

    def test_forward_and_inverse_still_round_trip(self):
        mgr, w = _mgr_with_crop()
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        crop = CameraCrop(mode=MODE_SQUARE, scale=0.8,
                          offset_x=-0.06, offset_y=0.04)
        w.set_crop(crop)
        _x0, _y0, cw, ch = crop.rect_for(TUCSEN_W, TUCSEN_H)
        for rot in (0.0, -90.0, 37.5):
            mgr.set_rotation_deg(0, rot)
            for px, py in ((10.0, 10.0), (cw - 10.0, ch - 10.0),
                           (cw / 2, ch / 2), (300.0, 900.0)):
                dx, dy = mgr.pixel_to_stage_offset(0, px, py, cw, ch)
                bx, by = mgr.stage_offset_to_pixel(0, dx, dy, cw, ch)
                self.assertAlmostEqual(bx, px, places=6)
                self.assertAlmostEqual(by, py, places=6)

    def test_the_mosaic_and_the_click_path_agree_about_the_frame_centre(self):
        """Two conventions consistently applied would each be fine; ONE OF EACH
        is the bug. So the mosaic's tile displacement is pinned against what a
        click on the same pixel reports, at every orientation."""
        from SupportClasses.MosaicCalibration import resolve

        class ObjStore:
            def get_calibration(self, cam, obj):
                return {"measured_um_per_px": TUCSEN_4X_UM_PER_PX,
                        "resolution": [TUCSEN_W, TUCSEN_H]}

        mgr, w = _mgr_with_crop()
        mgr.camera_identity = lambda i: ("tucam:0", "Tucsen")
        mgr.set_um_per_px(0, TUCSEN_4X_UM_PER_PX,
                          resolution=(TUCSEN_W, TUCSEN_H))
        crop = CameraCrop(mode=MODE_SQUARE, scale=1.0,
                          offset_x=0.08, offset_y=-0.03)
        w.set_crop(crop)
        _x0, _y0, cw, ch = crop.rect_for(TUCSEN_W, TUCSEN_H)

        for rot, fx, fy in ((0.0, False, False), (-90.0, False, False),
                            (180.0, True, False), (37.5, False, True),
                            (0.0, True, True)):
            mgr.set_rotation_deg(0, rot)
            # Via set_mirrored, NOT the cache list: get_mirrored reads the
            # widget, so writing the list alone would leave the click path
            # un-flipped while the calibration was flipped — the two paths would
            # then differ for the wrong reason and the flip half of this test
            # would be dead. (A mutation dropping the flips survived until this
            # was fixed.)
            mgr.set_mirrored(0, fx)
            mgr._flip_y[0] = fy
            cal = resolve(camera_manager=mgr, cam_idx=0, objective="4x",
                          live_resolution=(cw, ch), obj_store=ObjStore(),
                          cal_store=_FakeCalStore(rot, fx, fy))
            click = mgr.pixel_to_stage_offset(0, cw / 2, ch / 2, cw, ch)
            self.assertAlmostEqual(cal.crop_offset_um[0], click[0], places=6,
                                   msg=f"rot={rot} fx={fx} fy={fy}")
            self.assertAlmostEqual(cal.crop_offset_um[1], click[1], places=6)

    def test_crop_offset_um_is_zero_for_a_centred_crop(self):
        from SupportClasses.MosaicCalibration import MosaicCalibration
        cal = MosaicCalibration(um_per_px=TUCSEN_4X_UM_PER_PX,
                                rotation_deg=-90.0)
        self.assertEqual(cal.crop_offset_um, (0.0, 0.0))


class _FakeCalStore:
    def __init__(self, rot, fx, fy):
        self._rot, self._fx, self._fy = rot, fx, fy

    def get_rotation(self, i):
        return self._rot

    def get_mirrored(self, i):
        return self._fx

    def get_flip_y(self, i):
        return self._fy

    def get_mosaic_output_rotation(self, i):
        return 0.0


class TestTheMosaicPlacesAnOffsetTileCorrectly(unittest.TestCase):
    """The other half of the compensation: a mosaic built from offset frames must
    still back-project a feature to its TRUE stage position, or every well centre
    derived from the composite inherits the offset."""

    UMPX = 1.27
    FW = FH = 200

    def _mark_centre(self):
        f = np.full((self.FH, self.FW, 3), 60, np.uint8)
        f[self.FH // 2 - 3:self.FH // 2 + 4,
          self.FW // 2 - 3:self.FW // 2 + 4] = 255
        return f

    def _back_project(self, toff):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        b = MosaicBuilder(frame_size_px=(self.FW, self.FH),
                          micron_per_pixel=self.UMPX, overlap=0.25,
                          target_mosaic_px=1200, register=False,
                          tile_center_offset_um=toff)
        b._init_composite((0.0, 0.0, 400.0, 400.0))
        b.add_raster_frame(self._mark_centre(), 200.0, 200.0, 0)
        comp = b.stitch_incremental()
        ext = b.canvas_extent_um
        g = comp.max(axis=2)
        ys, xs = np.where(g >= int(g.max()) - 5)
        return (ext[0] + xs.mean() / b._mosaic_scale,
                ext[1] + ys.mean() / b._mosaic_scale)

    def test_a_centred_crop_places_the_tile_at_the_stage_position(self):
        sx, sy = self._back_project((0.0, 0.0))
        self.assertAlmostEqual(sx, 200.0, delta=2.0)
        self.assertAlmostEqual(sy, 200.0, delta=2.0)

    def test_an_offset_tile_back_projects_to_its_TRUE_position(self):
        toff = (250.0, -120.0)
        sx, sy = self._back_project(toff)
        self.assertAlmostEqual(sx, 200.0 + toff[0], delta=3.0)
        self.assertAlmostEqual(sy, 200.0 + toff[1], delta=3.0)

    def test_the_offset_tile_is_not_clipped_off_the_canvas(self):
        """The scan bounds are the operator's region, not the tiles' footprint.
        At a small crop the displacement can EXCEED half a FOV, so the canvas has
        to grow — otherwise the tile is silently cut at the edge."""
        from SupportClasses.MosaicBuilder import MosaicBuilder
        big = (900.0, -900.0)                       # ≫ half a 254 µm FOV
        b = MosaicBuilder(frame_size_px=(self.FW, self.FH),
                          micron_per_pixel=self.UMPX, overlap=0.25,
                          target_mosaic_px=1200, register=False,
                          tile_center_offset_um=big)
        b._init_composite((0.0, 0.0, 400.0, 400.0))
        b.add_raster_frame(self._mark_centre(), 200.0, 200.0, 0)
        comp = b.stitch_incremental()
        lit = int((comp.max(axis=2) >= 250).sum())
        expect = (7 * b._mosaic_scale / (1.0 / self.UMPX)) ** 2
        self.assertGreater(lit, expect * 0.4,
                           "the marked patch was clipped at the canvas edge")

    def test_zero_offset_is_byte_identical_to_omitting_the_argument(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        kw = dict(frame_size_px=(self.FW, self.FH), micron_per_pixel=self.UMPX,
                  overlap=0.25, target_mosaic_px=1200, register=False)
        a = MosaicBuilder(**kw)
        c = MosaicBuilder(**kw, tile_center_offset_um=(0.0, 0.0))
        for b in (a, c):
            b._init_composite((0.0, 0.0, 400.0, 400.0))
            b.add_raster_frame(self._mark_centre(), 200.0, 200.0, 0)
            b.stitch_incremental()
        self.assertEqual(a.canvas_extent_um, c.canvas_extent_um)
        self.assertTrue(np.array_equal(a.composite, c.composite))


class TestTheCrosshairMarksTheStagePosition(unittest.TestCase):

    def setUp(self):
        _app()

    @staticmethod
    def _drawn_vertical_x(w, crop, cw, ch):
        """Where the crosshair's vertical line was actually PAINTED.

        Probes the rendered pixmap, not the model: asserting that
        ``reference_pixel`` moved says nothing about whether ``_draw_crosshair``
        used it (a mutation that ignored it survived a model-only test).
        """
        from PySide6.QtGui import QPixmap, QColor
        pm = QPixmap(cw, ch)
        pm.fill(QColor("white"))
        w._draw_crosshair(pm)
        img = pm.toImage()
        row = ch // 4                       # away from the horizontal line
        cols = [x for x in range(cw)
                if QColor(img.pixel(x, row)).red() < 250]
        return sum(cols) / len(cols) if cols else None

    def test_it_moves_with_an_offset_crop(self):
        """A crosshair left at the geometric middle of a moved crop would aim the
        operator at a point the stage is not on."""
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget(show_controls=False)
        w._capture_size = (TUCSEN_W, TUCSEN_H)

        centred = CameraCrop(mode=MODE_SQUARE, scale=1.0)
        w.set_crop(centred)
        _x0, _y0, cw, ch = centred.rect_for(TUCSEN_W, TUCSEN_H)
        at_centre = self._drawn_vertical_x(w, centred, cw, ch)
        self.assertIsNotNone(at_centre, "no crosshair was drawn at all")
        self.assertAlmostEqual(at_centre, cw / 2.0, delta=2.0)

        moved = CameraCrop(mode=MODE_SQUARE, scale=1.0, offset_x=0.08)
        w.set_crop(moved)
        rx, _ry = moved.reference_pixel(TUCSEN_W, TUCSEN_H)
        at_ref = self._drawn_vertical_x(w, moved, cw, ch)
        self.assertIsNotNone(at_ref)
        self.assertAlmostEqual(at_ref, rx, delta=2.0)
        # And it genuinely moved — not merely "close to something".
        self.assertGreater(abs(at_ref - cw / 2.0), 100.0)
        w.deleteLater()


class TestTheMoveBoundUsesTheCroppedFrame(unittest.TestCase):
    """The rotation dialog bounds its stage move so the tracked feature stays in
    view. A crop makes the view smaller, so the bound must shrink with it — but
    the µm/px it is computed from must not."""

    def setUp(self):
        _app()

    def test_expected_um_per_px_uses_the_capture_width(self):
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        from SupportClasses.ObjectiveCalibration import ObjectiveCalibrationStore

        store = ObjectiveCalibrationStore.__new__(ObjectiveCalibrationStore)
        store._data = {"camera_objective_calibrations": {"tucam:0": {"4x": {
            "measured_um_per_px": TUCSEN_4X_UM_PER_PX,
            "resolution": [TUCSEN_W, TUCSEN_H]}}}}

        mgr, w = _mgr_with_crop()
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))

        dlg = PixelCalibrationDialog.__new__(PixelCalibrationDialog)
        dlg._camera_manager = mgr
        dlg._cam_idx = 0
        dlg._cam_key = "tucam:0"
        dlg._objective = "4x"

        import SupportClasses.ObjectiveCalibration as OC
        orig = OC.get_store
        OC.get_store = lambda *a, **k: store
        try:
            um, src = dlg._expected_um_per_px(2048)      # delivered width
        finally:
            OC.get_store = orig
        self.assertAlmostEqual(um, TUCSEN_4X_UM_PER_PX, places=6)
        self.assertIn("stored", src)


class TestTheCalibrationStampNamesTheSensorMode(unittest.TestCase):
    """A µm/px stamp must record the CAPTURE resolution, never the cropped one.

    Two things break if it records the delivered width:

    * every rescale compares the stamp against a capture width, so a crop
      changed after calibrating would silently scale a perfectly good value;
    * ``ObjectiveCalibration.sensor_width_um`` — the v7.16 plausibility guard —
      rests on ``µm/px × magnification × width`` being ONE fixed property of the
      sensor shared by every objective. Stamp two objectives at different crops
      and that invariant reports the camera's objectives as disagreeing.
    """

    def setUp(self):
        _app()

    def _card(self, mgr):
        from gui.pages.hardware.objective_calibration_card import (
            ObjectiveCalibrationCard)
        card = ObjectiveCalibrationCard.__new__(ObjectiveCalibrationCard)
        card._camera_manager = mgr
        card._config_getter = lambda: None
        return card

    def test_the_stamp_is_the_pre_crop_size(self):
        mgr, w = _mgr_with_crop()
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))
        # A cropped live frame, exactly as every consumer would see it.
        mgr._cameras = [type("C", (), {
            "get_current_frame": lambda self: np.zeros((2048, 2048, 3),
                                                       np.uint8)})()]
        self.assertEqual(self._card(mgr)._true_capture_resolution(0),
                         (TUCSEN_W, TUCSEN_H))

    def test_the_sensor_width_invariant_survives_a_crop(self):
        """Two objectives on one camera, calibrated at different crops, must
        still agree on the imaged sensor width."""
        from SupportClasses.ObjectiveCalibration import ObjectiveCalibrationStore
        store = ObjectiveCalibrationStore.__new__(ObjectiveCalibrationStore)
        store._data = {
            "objectives": [{"name": "4x", "nominal_magnification": 4.0},
                           {"name": "10x", "nominal_magnification": 10.0}],
            "camera_objective_calibrations": {"tucam:0": {
                "4x": {"measured_um_per_px": TUCSEN_4X_UM_PER_PX,
                       "resolution": [TUCSEN_W, TUCSEN_H]},
                "10x": {"measured_um_per_px": TUCSEN_4X_UM_PER_PX * 4.0 / 10.0,
                        "resolution": [TUCSEN_W, TUCSEN_H]},
            }}}
        width = store.sensor_width_um("tucam:0")
        self.assertIsNotNone(width, "the invariant refused on consistent data")
        self.assertAlmostEqual(width, TUCSEN_4X_UM_PER_PX * 4.0 * TUCSEN_W,
                               places=3)

        # Now the same camera with the 10x stamped at a CROPPED width — what
        # would happen if the stamp named the delivered frame.
        store._data["camera_objective_calibrations"]["tucam:0"]["10x"][
            "resolution"] = [2048, 2048]
        self.assertIsNone(
            store.sensor_width_um("tucam:0"),
            "a cropped stamp silently became the camera's native scale")


# ─────────────────────────────────────────────────────────────────────────────
#  Persistence
# ─────────────────────────────────────────────────────────────────────────────

class TestCropPersistsPerCameraIdentity(unittest.TestCase):

    def setUp(self):
        import tempfile
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self._tmp = tempfile.TemporaryDirectory()
        self.store = CameraCalibrationStore(
            Path(self._tmp.name) / "camera_calibrations.json")

    def tearDown(self):
        self._tmp.cleanup()

    def test_round_trip_survives_a_reload(self):
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self.store.set_crop("tucam:0", {"mode": "square", "scale": 0.85},
                            name="Tucsen Libra 25")
        again = CameraCalibrationStore(self.store._path)
        self.assertEqual(again.get_crop("tucam:0"),
                         {"mode": "square", "scale": 0.85})

    def test_siblings_are_preserved(self):
        """Writing a crop must not disturb µm/px, rotation or the flips — the
        store is one entry per camera and each writer touches only its field."""
        self.store.set_calibration("tucam:0", 1.2710, name="T",
                                   um_per_px_resolution=(TUCSEN_W, TUCSEN_H))
        self.store.set_rotation("tucam:0", -89.97)
        self.store.set_mirrored("tucam:0", True)
        self.store.set_crop("tucam:0", {"mode": "square", "scale": 1.0})
        entry = self.store.get_calibration("tucam:0")
        self.assertAlmostEqual(entry["um_per_px"], 1.2710)
        self.assertAlmostEqual(entry["rotation_deg"], -89.97)
        self.assertTrue(entry["mirrored"])
        self.assertEqual(entry["um_per_px_resolution"], [TUCSEN_W, TUCSEN_H])

    def test_disabling_pops_the_key_so_legacy_entries_stay_identical(self):
        self.store.set_crop("tucam:0", {"mode": "square", "scale": 1.0})
        self.store.set_crop("tucam:0", None)
        self.assertIsNone(self.store.get_crop("tucam:0"))
        self.assertNotIn("crop", self.store.get_calibration("tucam:0"))

    def test_the_placement_survives_a_reload(self):
        """The offset is the difference between a crop that clears the vignette
        and one that does not, so losing it on restart would be as bad as losing
        the crop itself."""
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        crop = CameraCrop(mode=MODE_SQUARE, scale=0.9,
                          offset_x=-0.065, offset_y=0.021)
        self.store.set_crop("tucam:0", crop.to_dict(), name="Tucsen")
        again = CameraCalibrationStore(self.store._path)
        self.assertEqual(CameraCrop.from_dict(again.get_crop("tucam:0")), crop)

    def test_a_centred_crop_stores_no_offset_keys(self):
        self.store.set_crop("tucam:0",
                            CameraCrop(mode=MODE_SQUARE, scale=1.0).to_dict())
        self.assertEqual(self.store.get_crop("tucam:0"),
                         {"mode": "square", "scale": 1.0})

    def test_a_camera_that_was_never_cropped_reads_as_no_crop(self):
        self.assertIsNone(self.store.get_crop("andor:VSC-07863"))
        self.assertFalse(CameraCrop.from_dict(None).enabled)


class TestSlotRestore(unittest.TestCase):
    """A slot's ``CameraWidget`` outlives the source assigned to it, so
    restoring has to CLEAR as well as set."""

    def setUp(self):
        _app()

    def _page_with(self, entry):
        from gui.pages.hardware_setup import HardwareSetupPage
        import SupportClasses.CameraCalibrationStore as CCS

        class FakeStore:
            def get_calibration(self, ident):
                return dict(entry) if entry else None

            def get_um_per_px_resolution(self, ident):
                return None

        page = HardwareSetupPage.__new__(HardwareSetupPage)
        mgr, w = _mgr_with_crop()
        mgr.camera_identity = lambda i: ("tucam:0", "Tucsen")
        mgr.set_image_correction = lambda *a, **k: None
        page._camera_manager = mgr
        page._live_cam_source_combos = [type("C", (), {
            "currentData": lambda self: "src"})()]
        page._live_cam_crop_checks = []
        page._live_cam_crop_spins = []
        page._live_cam_crop_labels = []
        page._sync_correction_sliders = lambda i: None
        orig = CCS.get_store
        CCS.get_store = lambda *a, **k: FakeStore()
        try:
            page._restore_calibration_for_slot(0)
        finally:
            CCS.get_store = orig
        return mgr, w

    def test_a_stored_crop_is_pushed_to_the_live_camera(self):
        mgr, w = self._page_with({"crop": {"mode": "square", "scale": 0.9}})
        self.assertTrue(w.crop().enabled)
        self.assertAlmostEqual(w.crop().scale, 0.9)

    def test_a_camera_with_no_stored_crop_clears_a_leftover_one(self):
        """Reassigning a slot from a cropped camera to an uncropped one must not
        leave the new camera cropped — the widget is per SLOT, not per camera.
        This runs BEFORE the no-entry early return for exactly that reason."""
        from gui.pages.hardware_setup import HardwareSetupPage
        import SupportClasses.CameraCalibrationStore as CCS

        page = HardwareSetupPage.__new__(HardwareSetupPage)
        mgr, w = _mgr_with_crop()
        w.set_crop(CameraCrop(mode=MODE_SQUARE, scale=1.0))     # previous cam
        mgr.camera_identity = lambda i: ("toupcam:xyz", "ToupTek")
        page._camera_manager = mgr
        page._live_cam_source_combos = [type("C", (), {
            "currentData": lambda self: "src"})()]
        page._live_cam_crop_checks = []
        page._live_cam_crop_spins = []
        page._live_cam_crop_labels = []

        class Empty:
            def get_calibration(self, ident):
                return None

        orig = CCS.get_store
        CCS.get_store = lambda *a, **k: Empty()
        try:
            page._restore_calibration_for_slot(0)
        finally:
            CCS.get_store = orig
        self.assertFalse(w.crop().enabled,
                         "a crop leaked onto a camera that has none stored")


# ─────────────────────────────────────────────────────────────────────────────
#  The control the operator asked for, where they asked for it
# ─────────────────────────────────────────────────────────────────────────────

class TestTheCropControlOnTheCameraPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _app()
        from gui.pages.hardware_setup import HardwareSetupPage, MAX_LIVE_CAMERAS
        cls.page = HardwareSetupPage()
        cls.slots = MAX_LIVE_CAMERAS

    def test_every_camera_slot_has_a_crop_control(self):
        self.assertEqual(len(self.page._live_cam_crop_checks), self.slots)
        self.assertEqual(len(self.page._live_cam_crop_spins), self.slots)
        self.assertEqual(len(self.page._live_cam_crop_labels), self.slots)

    def test_the_size_is_a_percentage_not_a_pixel_count(self):
        """A percentage keeps the cropped region the same physical part of the
        field when the capture resolution changes."""
        sp = self.page._live_cam_crop_spins[0]
        self.assertEqual((sp.minimum(), sp.maximum()), (10.0, 100.0))
        self.assertIn("%", sp.suffix())

    def test_the_size_spin_is_dead_while_the_crop_is_off(self):
        page = self.page
        cb, sp = page._live_cam_crop_checks[0], page._live_cam_crop_spins[0]
        cb.setEnabled(True)
        cb.setChecked(False)
        page._sync_crop_spin_enabled(0)
        self.assertFalse(sp.isEnabled())
        cb.setChecked(True)
        self.assertTrue(sp.isEnabled())
        cb.setChecked(False)

    def test_the_ui_builds_the_crop_the_operator_selected(self):
        page = self.page
        page._live_cam_crop_checks[0].setEnabled(True)
        page._live_cam_crop_checks[0].setChecked(True)
        page._live_cam_crop_spins[0].setValue(80.0)
        crop = page._slot_crop_from_ui(0)
        self.assertTrue(crop.enabled)
        self.assertAlmostEqual(crop.scale, 0.8)
        page._live_cam_crop_checks[0].setChecked(False)
        self.assertFalse(page._slot_crop_from_ui(0).enabled)

    def test_the_readout_names_the_delivered_size(self):
        page = self.page
        mgr, w = _mgr_with_crop()
        page._camera_manager = mgr
        page._live_cam_crop_checks[0].setEnabled(True)
        page._live_cam_crop_checks[0].setChecked(True)
        page._live_cam_crop_spins[0].setValue(100.0)
        page._refresh_slot_crop_displays()
        self.assertIn("2048", page._live_cam_crop_labels[0].text())
        self.assertIn("2600", page._live_cam_crop_labels[0].text())
        page._live_cam_crop_checks[0].setChecked(False)
        page._camera_manager = None

    def test_every_slot_has_offset_controls(self):
        self.assertEqual(len(self.page._live_cam_crop_off_x_spins), self.slots)
        self.assertEqual(len(self.page._live_cam_crop_off_y_spins), self.slots)
        self.assertEqual(len(self.page._live_cam_crop_centre_btns), self.slots)
        sp = self.page._live_cam_crop_off_x_spins[0]
        self.assertEqual((sp.minimum(), sp.maximum()), (-50.0, 50.0))
        self.assertIn("%", sp.suffix())

    def test_the_offset_controls_follow_the_crop_checkbox(self):
        page = self.page
        cb = page._live_cam_crop_checks[0]
        cb.setEnabled(True)
        cb.setChecked(False)
        page._sync_crop_spin_enabled(0)
        self.assertFalse(page._live_cam_crop_off_x_spins[0].isEnabled())
        self.assertFalse(page._live_cam_crop_centre_btns[0].isEnabled())
        cb.setChecked(True)
        self.assertTrue(page._live_cam_crop_off_x_spins[0].isEnabled())
        self.assertTrue(page._live_cam_crop_centre_btns[0].isEnabled())
        cb.setChecked(False)

    def test_the_ui_builds_the_offset_the_operator_dialled(self):
        page = self.page
        page._live_cam_crop_checks[0].setEnabled(True)
        page._live_cam_crop_checks[0].setChecked(True)
        page._live_cam_crop_off_x_spins[0].setValue(-6.0)
        page._live_cam_crop_off_y_spins[0].setValue(3.0)
        crop = page._slot_crop_from_ui(0)
        self.assertAlmostEqual(crop.offset_x, -0.06)
        self.assertAlmostEqual(crop.offset_y, 0.03)
        page._on_crop_recentre(0)
        crop = page._slot_crop_from_ui(0)
        self.assertEqual((crop.offset_x, crop.offset_y), (0.0, 0.0))
        page._live_cam_crop_checks[0].setChecked(False)

    def test_the_readout_reports_the_achieved_offset(self):
        page = self.page
        mgr, w = _mgr_with_crop()
        page._camera_manager = mgr
        page._live_cam_crop_checks[0].setEnabled(True)
        page._live_cam_crop_checks[0].setChecked(True)
        page._live_cam_crop_spins[0].setValue(100.0)
        page._live_cam_crop_off_x_spins[0].setValue(5.0)
        page._refresh_slot_crop_displays()
        txt = page._live_cam_crop_labels[0].text()
        self.assertIn("offset", txt)
        self.assertIn("+130", txt)         # 0.05 x 2600
        page._on_crop_recentre(0)
        page._live_cam_crop_checks[0].setChecked(False)
        page._camera_manager = None


class TestAimingTheCropDoesNotHammerTheDisk(unittest.TestCase):
    """The offset spins AUTO-REPEAT.

    A held arrow walked the crop 1 %→23 % in seven seconds on the rig and wrote
    the calibration file forty times — file I/O on the GUI thread at Qt's
    key-repeat rate, whose cost is not bounded by anything this application
    controls (an on-access virus scanner sits in that path). The live push must
    still be immediate: the operator is aiming by eye and the preview has to
    follow the spin.
    """

    @classmethod
    def setUpClass(cls):
        _app()
        from gui.pages.hardware_setup import HardwareSetupPage
        cls.page = HardwareSetupPage()

    def setUp(self):
        self.page._camera_manager, self.widget = _mgr_with_crop()
        self.writes = []
        self.page._camera_manager.camera_identity = lambda i: ("tucam:0", "T")
        self._patch_store()
        cb = self.page._live_cam_crop_checks[0]
        cb.setEnabled(True)
        cb.blockSignals(True)
        cb.setChecked(True)
        cb.blockSignals(False)
        self.page._pending_crop_writes.clear()

    def tearDown(self):
        import SupportClasses.CameraCalibrationStore as ccs
        ccs.get_store = self._real_get_store
        cb = self.page._live_cam_crop_checks[0]
        cb.blockSignals(True)
        cb.setChecked(False)
        cb.blockSignals(False)
        self.page._pending_crop_writes.clear()
        self.page._camera_manager = None

    def _patch_store(self):
        import SupportClasses.CameraCalibrationStore as ccs
        self._real_get_store = ccs.get_store
        writes = self.writes

        class _Recording:
            def set_crop(self, identity, crop, name=""):
                writes.append((identity, crop))

        ccs.get_store = lambda: _Recording()

    def _drag(self, steps=20):
        """One auto-repeating drag of the offset spin."""
        sp = self.page._live_cam_crop_off_x_spins[0]
        for i in range(1, steps + 1):
            sp.setValue(float(i))          # fires valueChanged, like a repeat

    def test_a_drag_writes_the_store_once_not_once_per_step(self):
        self._drag(20)
        self.assertEqual(self.writes, [],
                         "the store was written during the drag")
        self.page._flush_crop_persist()
        self.assertEqual(len(self.writes), 1)

    def test_the_live_push_still_happens_on_every_step(self):
        """Debouncing the WRITE must not debounce the PREVIEW — aiming the crop
        is a visual task, so every step has to reach the frame source."""
        seen = []
        self.page._camera_manager.set_crop = lambda i, c: seen.append(c.offset_x)
        self._drag(20)
        self.assertEqual(len(seen), 20)
        self.assertAlmostEqual(seen[-1], 0.20)

    def test_the_last_value_is_the_one_persisted(self):
        self._drag(20)
        self.page._flush_crop_persist()
        self.assertAlmostEqual(self.writes[-1][1]["offset_x"], 0.20)

    def test_the_checkbox_persists_immediately(self):
        """One discrete decision, nothing to coalesce — and turning the crop OFF
        is what an operator does when something looks wrong, so it must reach
        disk even if the app closes a moment later."""
        cb = self.page._live_cam_crop_checks[0]
        cb.blockSignals(True)
        cb.setChecked(False)
        cb.blockSignals(False)
        self.writes.clear()
        self.page._on_toggle_crop(0, False)
        self.assertEqual(len(self.writes), 1)
        self.assertIsNone(self.writes[0][1])

    def test_the_recentre_button_persists_immediately(self):
        self.page._live_cam_crop_off_x_spins[0].setValue(9.0)
        self.writes.clear()
        self.page._on_crop_recentre(0)
        self.assertEqual(len(self.writes), 1)

    def test_navigating_away_flushes_a_pending_write(self):
        """A crop applied but never written would come back wrong on the next
        launch — the exact failure this store exists to prevent."""
        self._drag(3)
        self.assertEqual(self.writes, [])
        from PySide6.QtGui import QHideEvent
        self.page.hideEvent(QHideEvent())
        self.assertEqual(len(self.writes), 1)

    def test_a_pending_write_is_never_silently_dropped(self):
        self._drag(3)
        self.page._flush_crop_persist()
        self.page._flush_crop_persist()       # nothing left to write
        self.assertEqual(len(self.writes), 1)
        self.assertEqual(self.page._pending_crop_writes, {})

    def test_the_debounce_timer_is_armed_by_a_drag(self):
        self.assertFalse(self.page._crop_persist_timer.isActive())
        self._drag(2)
        self.assertTrue(self.page._crop_persist_timer.isActive())
        self.page._flush_crop_persist()
        self.assertFalse(self.page._crop_persist_timer.isActive())


if __name__ == "__main__":
    unittest.main()
