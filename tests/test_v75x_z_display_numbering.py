"""
test_v75x_z_display_numbering.py

Regression for the "Z reads -60 at the top / Min-Max freezes Z" bug (ME3B V1).

On this machine the needle physically DESCENDS as the raw Marlin Z counter
INCREASES. The app numbered Z by the raw value, so:
  - going up the number went negative (top read -60), and
  - recording the top into "Max" stored z_max=-60 with z_min=0 → an inverted
    envelope (z_min > z_max) that collapses clamp_z to a single point → Z froze.

Fix (v7.5.x): number Z as a *height* (up = +) at the human boundary only.
Every Z value shown on the Jog/Device pages is multiplied by ZDIR; every Z the
user types/clicks there is divided by ZDIR before motion. The Min/Max limit
spinboxes are height-frame and convert to the raw-stored envelope with a
MIN/MAX SWAP (ZDIR=-1 reverses ordering) — guaranteeing z_min < z_max so the
envelope can never invert/freeze again. The internal raw Marlin / motion frame
is UNCHANGED.

These tests assert the helpers, the limit swap (apply + load + round-trip), the
anti-freeze regression, and that motion stays in the raw frame (no ZDIR).
"""

import unittest

from SupportClasses.StageController import (
    StageController, ZDIR, z_raw_to_display, z_display_to_raw,
)
from SupportClasses.SafetyLimits import SafetyLimits

DEFAULT_MAP = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}
ME3B_V1_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}


class _FakeZP:
    """ZPStageManager stand-in: axis_map + records the last move."""

    def __init__(self, axis_map):
        self.axis_map = dict(axis_map)
        self.last_rel = None
        self.last_abs = None

    def move_relative(self, axes, feedrate=None):
        self.last_rel = dict(axes)

    def move_absolute(self, axes, fast=False, feedrate_mm_min=None):
        self.last_abs = dict(axes)


def _stop(ctrl):
    try:
        ctrl._pos_poller.stop()
    except Exception:
        pass


# ── The limit conversion the device-setup page applies. These mirror, line
# for line, the swap in StageHardwarePanel._apply / _load_from_settings so the
# contract is locked even when the GUI isn't instantiated. ──────────────────

def _apply_height_to_raw(spin_min_height, spin_max_height):
    """height spinboxes → stored raw (z_min, z_max), swapped."""
    z_min = z_display_to_raw(spin_max_height)
    z_max = z_display_to_raw(spin_min_height)
    return z_min, z_max


def _load_raw_to_height(z_min_raw, z_max_raw):
    """stored raw → height spinboxes (spin_min, spin_max), swapped."""
    spin_min = z_raw_to_display(z_max_raw)
    spin_max = z_raw_to_display(z_min_raw)
    return spin_min, spin_max


class TestHelpers(unittest.TestCase):
    def test_sign_is_negative_for_this_machine(self):
        self.assertEqual(ZDIR, -1.0)

    def test_top_reads_positive(self):
        # Needle at the top → raw most negative → height positive.
        self.assertEqual(z_raw_to_display(-60.0), 60.0)
        self.assertEqual(z_raw_to_display(0.0), 0.0)
        self.assertEqual(z_raw_to_display(50.0), -50.0)

    def test_round_trip(self):
        for x in (-60.0, -12.34, 0.0, 7.5, 50.0):
            self.assertAlmostEqual(
                z_display_to_raw(z_raw_to_display(x)), x, places=9)
            self.assertAlmostEqual(
                z_raw_to_display(z_display_to_raw(x)), x, places=9)


class TestLimitSwap(unittest.TestCase):
    def test_apply_swap_never_inverts(self):
        # User enters height Min=0 (bottom), Max=60 (top).
        z_min, z_max = _apply_height_to_raw(0.0, 60.0)
        self.assertEqual((z_min, z_max), (-60.0, 0.0))
        self.assertLess(z_min, z_max)  # never inverted → never frozen

    def test_load_swap_round_trips(self):
        # Stored raw [-60, 0] should display as height Min=0, Max=60.
        spin_min, spin_max = _load_raw_to_height(-60.0, 0.0)
        self.assertEqual((spin_min, spin_max), (0.0, 60.0))
        self.assertLess(spin_min, spin_max)

    def test_apply_then_load_is_identity(self):
        for hmin, hmax in ((0.0, 60.0), (-5.0, 42.0), (-50.0, 10.0)):
            z_min, z_max = _apply_height_to_raw(hmin, hmax)
            back_min, back_max = _load_raw_to_height(z_min, z_max)
            self.assertAlmostEqual(back_min, hmin, places=9)
            self.assertAlmostEqual(back_max, hmax, places=9)


class TestAntiFreezeRegression(unittest.TestCase):
    """The exact scenario the user hit: record top into Max, bottom into Min."""

    def test_recording_top_and_bottom_does_not_freeze(self):
        # Physical bottom = raw 0 → height "Min" 0; physical top = raw -60 →
        # height "Max" +60 (this is what _record_limit now writes).
        height_min = z_raw_to_display(0.0)     # 0
        height_max = z_raw_to_display(-60.0)    # +60
        self.assertEqual((height_min, height_max), (0.0, 60.0))

        z_min, z_max = _apply_height_to_raw(height_min, height_max)
        sl = SafetyLimits(z_min=z_min, z_max=z_max, enabled=True)

        # Envelope is non-inverted and a mid-travel raw target passes through
        # unclamped (the old [0, -60] envelope collapsed every move to 0).
        self.assertLess(sl.z_min, sl.z_max)
        self.assertAlmostEqual(sl.clamp_z(-30.0), -30.0, places=9)
        # And the ends still clamp correctly.
        self.assertAlmostEqual(sl.clamp_z(-70.0), -60.0, places=9)
        self.assertAlmostEqual(sl.clamp_z(5.0), 0.0, places=9)

    def test_old_unswapped_path_would_have_frozen(self):
        # Documents the bug: storing height values directly (no swap) inverts.
        sl_bad = SafetyLimits(z_min=0.0, z_max=-60.0, enabled=True)
        self.assertEqual(sl_bad.clamp_z(-30.0), 0.0)   # collapsed → frozen
        self.assertEqual(sl_bad.clamp_z(-10.0), 0.0)


class TestRecordLimitZBranch(unittest.TestCase):
    """Exercise the REAL StageHardwarePanel._record_limit Z branch via a
    light stand-in (no QApplication), mirroring the project's test idiom."""

    def _run(self, raw_z, which):
        from types import SimpleNamespace
        from gui.pages.hardware.stage_panel import StageHardwarePanel

        class _Spin:
            def __init__(self):
                self.v = None

            def setValue(self, x):
                self.v = x

        page = SimpleNamespace(
            _controller=SimpleNamespace(
                get_xy_position=lambda cached=False: (None, None, None),
                get_zp_position=lambda cached=False: (0, 0, raw_z, 0),
            ),
            _logical_zp_value=lambda zp, a: raw_z,
            spin_z_min=_Spin(),
            spin_z_max=_Spin(),
            _update_position_displays=lambda xy, zp: None,
            lbl_jog_status=SimpleNamespace(setText=lambda *_: None),
        )
        StageHardwarePanel._record_limit(page, "Z", which)
        return page

    def test_set_max_at_top_records_positive_height(self):
        # Needle at the top → raw -60 → "Set Max" must record +60.
        page = self._run(-60.0, "max")
        self.assertEqual(page.spin_z_max.v, 60.0)

    def test_set_min_at_bottom_records_zero(self):
        page = self._run(0.0, "min")
        self.assertEqual(page.spin_z_min.v, 0.0)


class TestMotionFrameUntouched(unittest.TestCase):
    """HARD CONSTRAINT: motion stays in the raw Marlin frame (no ZDIR)."""

    def setUp(self):
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, self.ctrl)
        self.ctrl.safety_limits = SafetyLimits(
            z_min=-60.0, z_max=0.0, enabled=True)

    def _attach(self, axis_map, physical_tuple):
        self.ctrl.zp_stage = _FakeZP(axis_map)
        self.ctrl.get_zp_position = lambda cached=True: physical_tuple

    def test_move_z_absolute_sends_raw_no_sign_flip(self):
        self._attach(ME3B_V1_MAP, (0, 0, -30.0, 0))
        self.ctrl.move_z_absolute(-30.0, from_zero_ref=False)
        sent = self.ctrl.zp_stage.last_abs["Z"]
        self.assertAlmostEqual(sent, -30.0, places=6)  # NOT +30

    def test_move_z_relative_sends_raw_no_sign_flip(self):
        self._attach(ME3B_V1_MAP, (0, 0, -30.0, 0))
        self.ctrl.move_z_relative(-5.0)
        sent = self.ctrl.zp_stage.last_rel["Z"]
        self.assertAlmostEqual(sent, -5.0, places=6)   # raw delta, no flip

    def test_flip_sign_stays_unused(self):
        # The dormant motion flip must remain identity (not repurposed).
        self.assertEqual(self.ctrl._flip_sign("Z"), 1.0)


if __name__ == "__main__":
    unittest.main()
