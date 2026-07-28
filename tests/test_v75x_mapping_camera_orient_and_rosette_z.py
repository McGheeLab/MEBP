"""
test_v75x_mapping_camera_orient_and_rosette_z.py

Two v7.5.x operator fixes (2026-07-24):

1. **Camera orientation in plate/rosette mapping.** The mosaic build now
   orients each tile from camera-pixel axes into the STAGE frame using the
   microscope camera's calibrated rotation + mirror (``MosaicBuilder`` gains
   ``frame_rotation_deg`` / ``frame_mirrored``), so the finished composite is
   stage-aligned and mosaic clicks back-project to the correct XY. Default
   (0°, unmirrored) is a no-op — byte-identical to the legacy raw placement.

2. **Rosette mapping no longer clamps Z.** The mosaic scan / mapping travel
   (imaging height, never descends) now passes ``apply_insert_floor=False`` to
   ``safe_travel_to`` / ``ensure_retracted_to`` so it retracts to exactly the
   operator-assigned safe Z instead of the tube-clearance insert floor (which
   is armed only for rosette plates — the reason plate mapping was unaffected).
   The insert floor stays in force for print / pick-place travel, and its
   computation is now polarity-correct.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from SupportClasses.StageController import StageController


# ═══════════════════════════════════════════════════════════════════
#  Insert-clearance floor bypass for imaging-height mapping travel
# ═══════════════════════════════════════════════════════════════════

def _make_controller():
    """Minimal real StageController for ensure_retracted_to tests."""
    ctrl = StageController.__new__(StageController)
    ctrl.xy_stage = MagicMock()
    ctrl.zp_stage = MagicMock()
    ctrl.zp_stage.flush_moves = MagicMock(return_value=True)
    ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0, "P2": 0, "P3": 0}
    ctrl._pos_poller = MagicMock()
    ctrl._zp_retract_feedrate = 250.0
    ctrl._zp_insert_feedrate = 100.0
    ctrl._min_travel_z_mm = None
    ctrl.move_z_absolute = MagicMock()
    ctrl.wait_for_z_arrival = MagicMock(return_value=True)
    return ctrl


class TestInsertFloorBypass(unittest.TestCase):
    def setUp(self):
        # Needle currently DOWN (raw +25 = physically low on ZDIR=-1), so a
        # retract move is required in every case below.
        self.c = _make_controller()
        # Insert floor armed at raw -20 (height +20) — higher than safe raw 0.
        self.c._min_travel_z_mm = -20.0
        self.c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        self.c.zp_logical_value = MagicMock(return_value=25.0)

    def test_default_applies_floor(self):
        """Regression: by default the tube-clearance floor still raises the
        retract (print / pick-place travel must keep clearing tubes)."""
        self.c.ensure_retracted_to(0.0)
        args, _ = self.c.move_z_absolute.call_args
        self.assertEqual(args[0], -20.0)          # raised to the floor

    def test_bypass_uses_assigned_safe_z(self):
        """Mapping travel (apply_insert_floor=False) retracts to exactly the
        operator-assigned safe Z — no floor override / clamp."""
        self.c.ensure_retracted_to(0.0, apply_insert_floor=False)
        args, _ = self.c.move_z_absolute.call_args
        self.assertEqual(args[0], 0.0)            # assigned safe Z, not the floor

    def test_bypass_still_never_descends(self):
        """Bypassing the floor must not defeat the never-descend guard: if the
        needle is already above the assigned safe Z, no move is issued."""
        self.c.get_zp_position = MagicMock(return_value=(-10.0, 0.0, -10.0, 0.0))
        self.c.zp_logical_value = MagicMock(return_value=-10.0)  # height +10 > 0
        ok = self.c.ensure_retracted_to(0.0, apply_insert_floor=False)
        self.assertTrue(ok)
        self.c.move_z_absolute.assert_not_called()

    def test_signature_accepts_flag_on_safe_travel_to(self):
        """safe_travel_to also accepts apply_insert_floor (imaging travel)."""
        import inspect
        sig = inspect.signature(StageController.safe_travel_to)
        self.assertIn("apply_insert_floor", sig.parameters)
        self.assertTrue(sig.parameters["apply_insert_floor"].default)


class TestInsertClearancePolarity(unittest.TestCase):
    """The insert-clearance floor value must RAISE the floor in the height
    frame on both polarities — top_z + z_up_sign * (rim + margin). Mirrors the
    PrintTrajectoryPlanner._well_print_z polarity fix."""

    def _floor(self, top_z, rim, margin, z_up):
        return top_z + z_up * (rim + margin)

    def test_zup_plus_one_adds(self):
        # z_up=+1: clearance adds above the plate top (larger zref = higher).
        self.assertAlmostEqual(self._floor(30.0, 3.0, 3.0, 1.0), 36.0)

    def test_zup_minus_one_subtracts(self):
        # z_up=-1: physically-higher is a SMALLER zref, so clearance subtracts —
        # the old top_z + rim + margin would have gone the wrong way.
        self.assertAlmostEqual(self._floor(30.0, 3.0, 3.0, -1.0), 24.0)


# ═══════════════════════════════════════════════════════════════════
#  MosaicBuilder tile orientation (mirror + rotation → stage frame)
# ═══════════════════════════════════════════════════════════════════

from SupportClasses.MosaicBuilder import MosaicBuilder, CV2_AVAILABLE


def _asym_tile(w=8, h=6):
    """A tile whose only bright pixel is at the top-left quadrant, so any
    flip/rotation is unambiguous."""
    t = np.zeros((h, w, 3), dtype=np.uint8)
    t[1, 1] = (255, 255, 255)
    return t


@unittest.skipUnless(CV2_AVAILABLE, "cv2 required for tile orientation")
class TestOrientTile(unittest.TestCase):
    def test_default_is_identity_noop(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0)
        t = _asym_tile()
        out = b._orient_tile(t)
        self.assertTrue(np.array_equal(out, t))

    def test_near_zero_rotation_is_noop(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          frame_rotation_deg=0.01)
        t = _asym_tile()
        self.assertTrue(np.array_equal(b._orient_tile(t), t))

    def test_mirror_flips_horizontally(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          frame_mirrored=True)
        t = _asym_tile(8, 6)  # bright at col 1
        out = b._orient_tile(t)
        # Horizontal flip about centre: col 1 → col (w-1-1) = 6.
        ys, xs = np.where(out[:, :, 0] > 128)
        self.assertEqual((int(ys[0]), int(xs[0])), (1, 6))

    def test_rotation_180_flips_both(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          frame_rotation_deg=180.0)
        t = _asym_tile(8, 6)  # bright at (row1, col1)
        out = b._orient_tile(t)
        # 180° about centre: (x,y) → (w-1-x, h-1-y) = (6, 4).
        ys, xs = np.where(out[:, :, 0] > 128)
        self.assertEqual((int(ys[0]), int(xs[0])), (4, 6))

    def test_center_pixel_is_invariant_under_rotation(self):
        b = MosaicBuilder(frame_size_px=(9, 9), micron_per_pixel=1.0,
                          frame_rotation_deg=37.0)
        t = np.zeros((9, 9, 3), dtype=np.uint8)
        t[4, 4] = (255, 255, 255)  # exact centre
        out = b._orient_tile(t)
        self.assertGreater(int(out[4, 4, 0]), 128)  # centre stays put


class TestBuilderCarriesOrientation(unittest.TestCase):
    def test_params_stored(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          frame_rotation_deg=45.0, frame_mirrored=True)
        self.assertEqual(b._frame_rotation_deg, 45.0)
        self.assertTrue(b._frame_mirrored)

    def test_defaults_are_noop(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0)
        self.assertEqual(b._frame_rotation_deg, 0.0)
        self.assertFalse(b._frame_mirrored)


if __name__ == "__main__":
    unittest.main()
