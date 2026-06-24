"""
v7.5.x — Spheroid Pick & Place click→stage conversion fixes.

Two bugs in the live-target picker / spheroid workflow are covered here:

1. **1000× unit error.** ``StageController.get_xy_position`` already returns
   absolute stage µm (``get_xy_position_mm`` divides it by 1000). The picker
   used to treat it as mm and multiply by 1000, inflating every picked target
   ~1000× — driving the stage to the XY-envelope corner (seen on HW as a
   ``wait_for_xy_arrival`` timeout to ``target≈(64735, 49557)`` mm).

2. **Resolution µm/px scaling.** µm/px is measured at a specific frame
   resolution (e.g. 916 px wide); applied to a live frame at a different
   resolution it is off by the width ratio. ``CameraManager.effective_um_per_px``
   / ``pixel_to_stage_offset`` now rescale to the live frame width.

These call the real (unbound) methods with duck-typed stubs — no Qt / camera /
event loop needed.
"""

import unittest
from types import SimpleNamespace

from gui.widgets.camera_manager import CameraManager
from gui.widgets.live_target_picker import LiveTargetPicker


def _make_manager(um_per_px, resolution=None, max_cameras=3):
    """A CameraManager with one slot calibrated, built without Qt widgets."""
    mgr = CameraManager.__new__(CameraManager)
    mgr._max_cameras = max_cameras
    mgr._um_per_px = [1.67] * max_cameras
    mgr._um_per_px_set = [False] * max_cameras
    mgr._um_per_px_res = [None] * max_cameras
    mgr.set_um_per_px(0, um_per_px, resolution=resolution)
    return mgr


# ── CameraManager resolution scaling ────────────────────────────────


class TestEffectiveUmPerPx(unittest.TestCase):
    def test_no_resolution_means_no_rescale(self):
        mgr = _make_manager(1.5)  # no calibration resolution
        self.assertAlmostEqual(mgr.effective_um_per_px(0, 1832), 1.5)

    def test_double_width_halves_um_per_px(self):
        # Calibrated at 916 px; live feed at 1832 px → each px spans half.
        mgr = _make_manager(1.54, resolution=(916, 686))
        self.assertAlmostEqual(mgr.effective_um_per_px(0, 1832), 1.54 * 916 / 1832)

    def test_same_width_is_unchanged(self):
        mgr = _make_manager(1.54, resolution=(916, 686))
        self.assertAlmostEqual(mgr.effective_um_per_px(0, 916), 1.54)

    def test_quarter_resolution_quadruples_um_per_px(self):
        # Calibrated at 3664; live at 916 → ×4.
        mgr = _make_manager(0.4, resolution=(3664, 2748))
        self.assertAlmostEqual(mgr.effective_um_per_px(0, 916), 0.4 * 3664 / 916)

    def test_pixel_to_stage_offset_uses_live_resolution(self):
        mgr = _make_manager(1.0, resolution=(916, 686))
        # Click 100 px right of centre on an 1832×1374 frame (centre=(916,687)).
        dx, dy = mgr.pixel_to_stage_offset(0, 916 + 100, 687, 1832, 1374)
        # effective µm/px = 1.0 * 916/1832 = 0.5 → 100 px → 50 µm.
        self.assertAlmostEqual(dx, 100 * 0.5)
        self.assertAlmostEqual(dy, 0.0)

    def test_set_um_per_px_without_resolution_clears_stale_res(self):
        mgr = _make_manager(1.54, resolution=(916, 686))
        mgr.set_um_per_px(0, 2.0)  # no resolution
        self.assertIsNone(mgr.get_um_per_px_resolution(0))
        self.assertAlmostEqual(mgr.effective_um_per_px(0, 1832), 2.0)


# ── LiveTargetPicker click → absolute stage µm ──────────────────────


class _StubView:
    def __init__(self, image_size):
        self.image_size = image_size
        self.um_per_px = None
        self.stage_pos = None

    def set_um_per_px(self, v):
        self.um_per_px = v

    def set_stage_position(self, x, y):
        self.stage_pos = (x, y)


def _make_picker(image_size, stage_xy_um, mgr):
    p = LiveTargetPicker.__new__(LiveTargetPicker)
    p._cam_idx = 0
    p._camera_manager = mgr
    p._view = _StubView(image_size)
    p._controller = SimpleNamespace(
        get_xy_position=lambda cached=True: (stage_xy_um[0], stage_xy_um[1], 0.0))
    p._base_um_per_px = None
    p._cal_resolution = None
    p._last_live_w = None
    p._um_per_px_label = SimpleNamespace(setText=lambda *_: None)
    return p


class TestPickerClickConversion(unittest.TestCase):
    def test_click_at_center_returns_stage_position_in_um(self):
        # Stage at 64735 µm (≈64.7 mm) — the value that, ×1000, blew up before.
        mgr = _make_manager(1.0, resolution=(640, 480))
        p = _make_picker((640, 480), (64735.0, 49557.0), mgr)
        xy = p._pixel_to_stage_um(320.0, 240.0)  # exact centre → zero offset
        self.assertIsNotNone(xy)
        # Must equal the stage position (µm), NOT 1000× it.
        self.assertAlmostEqual(xy[0], 64735.0)
        self.assertAlmostEqual(xy[1], 49557.0)

    def test_click_offset_added_in_um_not_inflated(self):
        mgr = _make_manager(2.0, resolution=(640, 480))
        p = _make_picker((640, 480), (10000.0, 20000.0), mgr)
        # 10 px right + 5 px down of centre → +20 µm, +10 µm at 2.0 µm/px.
        xy = p._pixel_to_stage_um(330.0, 245.0)
        self.assertAlmostEqual(xy[0], 10000.0 + 20.0)
        self.assertAlmostEqual(xy[1], 20000.0 + 10.0)

    def test_target_stays_within_envelope_scale(self):
        # Regression: a centre click must not produce a ~64.7-million-µm target.
        mgr = _make_manager(1.0, resolution=(640, 480))
        p = _make_picker((640, 480), (64735.0, 49557.0), mgr)
        xy = p._pixel_to_stage_um(320.0, 240.0)
        self.assertLess(xy[0], 200_000.0)  # well inside a sane stage envelope
        self.assertLess(xy[1], 200_000.0)

    def test_click_scales_with_live_resolution(self):
        # µm/px calibrated at 916 px; live feed at 1832 px. A click 200 px from
        # centre must use the rescaled 0.5× µm/px, not the raw value.
        mgr = _make_manager(1.0, resolution=(916, 686))
        p = _make_picker((1832, 1374), (0.0, 0.0), mgr)
        xy = p._pixel_to_stage_um(916.0 + 200.0, 687.0)  # centre=(916,687)
        # effective µm/px = 0.5 → 200 px → 100 µm.
        self.assertAlmostEqual(xy[0], 100.0)
        self.assertAlmostEqual(xy[1], 0.0)


# ── LiveTargetPicker live-resolution sync ───────────────────────────


class TestPickerUmPerPxSync(unittest.TestCase):
    def test_sync_pushes_rescaled_value_to_view(self):
        mgr = _make_manager(1.54, resolution=(916, 686))
        p = _make_picker((1832, 1374), (0.0, 0.0), mgr)
        p._base_um_per_px = 1.54
        p._cal_resolution = (916, 686)
        p._sync_live_um_per_px(force=True)
        # View must get the resolution-scaled value (≈0.77), not 1.54.
        self.assertAlmostEqual(p._view.um_per_px, 1.54 * 916 / 1832)

    def test_refresh_stage_position_pushes_um_not_mm(self):
        mgr = _make_manager(1.0, resolution=(640, 480))
        p = _make_picker((640, 480), (50000.0, 30000.0), mgr)
        p._refresh_stage_position()
        # Stage centre pushed to the overlay must be the raw µm, not ×1000.
        self.assertEqual(p._view.stage_pos, (50000.0, 30000.0))


if __name__ == "__main__":
    unittest.main()
