"""
v7.5.x — unified Z-axis convention (Phase 1: StageController foundation).

ONE canonical user-facing Z frame:  user_Z = z_up_sign · (raw − zero["Z"]),
with the datum (zero["Z"]) at the needle-all-the-way-DOWN raw position and
z_up_sign DERIVED by the single Z setup procedure so user_Z increases as the
needle rises (decisions D1–D4):
  * D1 datum = mechanical hard-bottom → user Z = 0 there;
  * D3 soft limits come from the same setup (z_min/z_max raw = min/max extreme);
  * direction is verified (sign derived from the captured extremes).

These tests exercise the pure logic on a minimal controller built with __new__
(no hardware / __init__), the same pattern as test_v75x_z_retract_before_xy_travel.
"""

import unittest

from SupportClasses.StageController import StageController, ZDIR
from SupportClasses.SafetyLimits import SafetyLimits


def _make_controller(z_up_sign=ZDIR):
    c = StageController.__new__(StageController)
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0, "P2": 0, "P3": 0}
    c.safety_limits = SafetyLimits()
    c._z_up_sign = z_up_sign
    c._needle_cam_z_user = None
    c._plate_z_offsets = {"top": 10.0, "bottom": 20.0, "safe": 5.0}
    return c


class TestApplyZSetupME3B(unittest.TestCase):
    """ME3B V1: needle descends as raw increases → bottom is the LARGER raw."""

    def test_bottom_zero_top_up(self):
        c = _make_controller()
        # Bottom (needle fully down) at raw 0; top (fully up) at raw -50.
        s = c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=-50.0)
        self.assertEqual(c.zero_position["Z"], 0.0)      # datum = bottom
        self.assertEqual(c.z_up_sign(), -1.0)            # up = smaller raw
        self.assertEqual(c.safety_limits.z_min, -50.0)   # raw envelope
        self.assertEqual(c.safety_limits.z_max, 0.0)
        self.assertAlmostEqual(s["travel_height_mm"], 50.0)
        self.assertTrue(s["direction_ok"])

    def test_user_z_zero_at_bottom_positive_up(self):
        c = _make_controller()
        c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=-50.0)
        self.assertAlmostEqual(c.raw_to_user_z(0.0), 0.0)     # bottom → 0
        self.assertAlmostEqual(c.raw_to_user_z(-50.0), 50.0)  # top → +50
        self.assertAlmostEqual(c.raw_to_user_z(-25.0), 25.0)  # mid → +25
        # Always positive in the working range, larger = higher.
        self.assertGreater(c.raw_to_user_z(-30.0), c.raw_to_user_z(-10.0))

    def test_user_z_inverse_roundtrip(self):
        c = _make_controller()
        c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=-50.0)
        for u in (0.0, 12.5, 50.0):
            self.assertAlmostEqual(c.raw_to_user_z(c.user_z_to_raw(u)), u)
        self.assertAlmostEqual(c.user_z_to_raw(0.0), 0.0)
        self.assertAlmostEqual(c.user_z_to_raw(50.0), -50.0)

    def test_zref_user_roundtrip(self):
        c = _make_controller()
        c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=-50.0)
        for zr in (-40.0, -0.0, -12.3):
            self.assertAlmostEqual(c.user_z_to_zref(c.zref_to_user_z(zr)), zr)
        # zref_to_user_z is exactly z_height_of (the user frame IS the height).
        self.assertAlmostEqual(c.zref_to_user_z(-30.0), c.z_height_of(-30.0))


class TestApplyZSetupConventional(unittest.TestCase):
    """Conventional machine: needle rises as raw increases."""

    def test_derives_plus_one(self):
        c = _make_controller()
        s = c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=50.0)
        self.assertEqual(c.z_up_sign(), 1.0)
        self.assertEqual(c.safety_limits.z_min, 0.0)
        self.assertEqual(c.safety_limits.z_max, 50.0)
        self.assertTrue(s["direction_ok"])
        self.assertAlmostEqual(c.raw_to_user_z(0.0), 0.0)
        self.assertAlmostEqual(c.raw_to_user_z(50.0), 50.0)


class TestApplyZSetupOffsetDatum(unittest.TestCase):
    """Datum need not be raw 0 — the bottom can be any raw value."""

    def test_offset_bottom(self):
        c = _make_controller()
        # ME3B-ish: bottom at raw +10, top at raw -40.
        s = c.apply_z_setup(raw_bottom_mm=10.0, raw_top_mm=-40.0)
        self.assertEqual(c.zero_position["Z"], 10.0)
        self.assertEqual(c.z_up_sign(), -1.0)
        self.assertEqual(c.safety_limits.z_min, -40.0)
        self.assertEqual(c.safety_limits.z_max, 10.0)
        self.assertAlmostEqual(s["travel_height_mm"], 50.0)
        self.assertAlmostEqual(c.raw_to_user_z(10.0), 0.0)   # bottom → 0
        self.assertAlmostEqual(c.raw_to_user_z(-40.0), 50.0)  # top → +50
        self.assertAlmostEqual(c.user_z_to_raw(0.0), 10.0)


class TestDirectionGuard(unittest.TestCase):
    """Extremes too close → direction not trusted, sign left unchanged."""

    def test_too_close_does_not_flip_sign(self):
        c = _make_controller(z_up_sign=-1.0)
        s = c.apply_z_setup(raw_bottom_mm=5.0, raw_top_mm=5.2,
                            min_travel_mm=1.0)
        self.assertFalse(s["direction_ok"])
        self.assertEqual(c.z_up_sign(), -1.0)   # unchanged
        # Datum + limits still recorded so the operator can re-run.
        self.assertEqual(c.zero_position["Z"], 5.0)
        self.assertEqual(c.safety_limits.z_min, 5.0)
        self.assertAlmostEqual(c.safety_limits.z_max, 5.2)


class TestBackwardCompat(unittest.TestCase):
    """A controller without _z_up_sign falls back to the module ZDIR."""

    def test_fallback_to_module_zdir(self):
        c = StageController.__new__(StageController)
        c.zero_position = {"Z": 0.0}
        # No _z_up_sign attribute set at all.
        self.assertEqual(c.z_up_sign(), ZDIR)
        # z_height_of unchanged from the pre-unification behavior (ZDIR=-1).
        self.assertEqual(c.z_height_of(-10.0), 10.0)
        self.assertEqual(c.z_height_of(0.0), 0.0)


class TestNeedleCamPlateEstimate(unittest.TestCase):
    """Needle-cam Z fiducial + standard offsets → coherent plate-ref guesses."""

    def _setup_me3b(self):
        c = _make_controller()
        c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=-50.0)  # sign=-1, zero=0
        c.set_plate_z_offsets(top=10.0, bottom=20.0, safe=5.0)
        return c

    def test_estimate_is_coherent_and_monotonic(self):
        c = self._setup_me3b()
        # Needle tip centered at raw -30 → user height +30 (ZDIR=-1).
        c.set_needle_cam_z_from_raw(-30.0)
        self.assertAlmostEqual(c.get_needle_cam_z_user(), 30.0)
        refs = c.estimate_plate_z_refs()
        # Returned in ZERO-REF mm (= user / sign = -user here).
        self.assertAlmostEqual(refs["plate_top_z"], -20.0)
        self.assertAlmostEqual(refs["plate_bottom_z"], -10.0)
        self.assertAlmostEqual(refs["safe_z"], -25.0)
        # The KEY property the old config violated: in the user/height frame
        # the plate BOTTOM is the lowest, top above it, safe-travel highest.
        self.assertLess(c.zref_to_user_z(refs["plate_bottom_z"]),
                        c.zref_to_user_z(refs["plate_top_z"]))
        self.assertLess(c.zref_to_user_z(refs["plate_top_z"]),
                        c.zref_to_user_z(refs["safe_z"]))

    def test_estimate_direction_independent(self):
        # Conventional machine: same user-frame guesses, sign=+1.
        c = _make_controller()
        c.apply_z_setup(raw_bottom_mm=0.0, raw_top_mm=50.0)
        c.set_plate_z_offsets(top=10.0, bottom=20.0, safe=5.0)
        c.set_needle_cam_z_from_raw(30.0)   # user +30
        refs = c.estimate_plate_z_refs()
        self.assertAlmostEqual(refs["plate_top_z"], 20.0)   # zref == user
        self.assertAlmostEqual(refs["plate_bottom_z"], 10.0)
        self.assertAlmostEqual(refs["safe_z"], 25.0)

    def test_estimate_none_without_fiducial(self):
        c = self._setup_me3b()
        self.assertIsNone(c.estimate_plate_z_refs())

    def test_apply_z_convention_restores(self):
        c = _make_controller(z_up_sign=ZDIR)
        c.apply_z_convention(z_up_sign=1.0, needle_cam_z=33.0,
                             plate_z_offsets={"top": 1, "bottom": 2, "safe": 3})
        self.assertEqual(c.z_up_sign(), 1.0)
        self.assertAlmostEqual(c.get_needle_cam_z_user(), 33.0)
        self.assertEqual(c.get_plate_z_offsets(),
                         {"top": 1.0, "bottom": 2.0, "safe": 3.0})


if __name__ == "__main__":
    unittest.main()
