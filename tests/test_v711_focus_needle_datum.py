"""
The focus↔needle-Z datum, and why its SCALE is gated rather than applied.

Once this datum exists the plate bottom can be found optically — focus on the
glass, convert — and the needle never has to be driven down onto the glass again.
That is the safety argument for the whole feature, so the conversion has to be
right.

The scale between the two axes must be exactly ±1.000 mm/mm: both measure the
same physical displacement of the same rigid object. There is no mechanism for a
genuine non-unity scale, only for bugs — a mm/µm slip, a wrong steps-per-mm, a
wrong ``focus_units_per_um`` (this rig has already hit that one as a 40× error).
So a measured scale is a CHECK, never a stored constant.
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import SupportClasses.PlateFocusDatumStore as mod   # noqa: E402
from SupportClasses.PlateFocusDatumStore import (   # noqa: E402
    PlateFocusDatumStore, check_scale, datum_key, SCALE_TOLERANCE)


class _StoreCase(unittest.TestCase):
    def setUp(self):
        self._dir = tempfile.TemporaryDirectory()
        self.addCleanup(self._dir.cleanup)
        self.path = os.path.join(self._dir.name, "plate_focus_datum.json")
        self.store = PlateFocusDatumStore(self.path)


class TestScaleGate(unittest.TestCase):
    def test_exactly_one_is_accepted(self):
        self.assertTrue(check_scale(1.0)[0])

    def test_a_negative_scale_is_legitimate(self):
        """The focus axis may run opposite to needle Z; that is a SIGN, not a
        fault."""
        self.assertTrue(check_scale(-1.0)[0])

    def test_small_deviations_inside_the_band_pass(self):
        self.assertTrue(check_scale(1.0 + SCALE_TOLERANCE / 2)[0])
        self.assertTrue(check_scale(1.0 - SCALE_TOLERANCE / 2)[0])

    def test_a_deviation_outside_the_band_is_reported_as_a_bug(self):
        ok, why = check_scale(0.987)
        self.assertFalse(ok)
        self.assertIn("bug", why)
        self.assertIn("NOT being applied", why)

    def test_the_forty_times_error_this_repo_already_hit_is_caught(self):
        ok, why = check_scale(40.0)
        self.assertFalse(ok)
        self.assertIn("focus_units_per_um", why)

    def test_a_thousand_times_units_slip_is_caught(self):
        self.assertFalse(check_scale(1000.0)[0])
        self.assertFalse(check_scale(0.001)[0])

    def test_a_near_zero_slope_refuses_because_the_sign_is_meaningless(self):
        ok, why = check_scale(0.02)
        self.assertFalse(ok)
        self.assertIn("sign cannot be trusted", why)


class TestConversion(_StoreCase):
    def test_round_trip_and_conversion(self):
        self.store.save("cam", "10x", "plate-24", focal_sign=1,
                        offset_mm=-1.0, zero_z_mm=0.0)
        self.assertAlmostEqual(
            self.store.needle_z_zref_mm("cam", "10x", "plate-24", 1500.0),
            0.5, places=9)

    def test_the_sign_flips_the_direction(self):
        self.store.save("cam", "10x", "plate-24", focal_sign=-1,
                        offset_mm=1.0, zero_z_mm=0.0)
        self.assertAlmostEqual(
            self.store.needle_z_zref_mm("cam", "10x", "plate-24", 1500.0),
            -0.5, places=9)

    def test_unknown_key_returns_none_rather_than_a_guess(self):
        self.assertIsNone(
            self.store.needle_z_zref_mm("cam", "10x", "other", 1500.0))

    def test_persists_across_reload(self):
        self.store.save("cam", "10x", "plate-24", focal_sign=1, offset_mm=-1.0)
        again = PlateFocusDatumStore(self.path)
        self.assertAlmostEqual(
            again.needle_z_zref_mm("cam", "10x", "plate-24", 1500.0), 0.5)


class TestKeying(_StoreCase):
    def test_camera_objective_and_plate_all_participate(self):
        """The focus value is where THAT optical train forms an image, and the
        height is a property of THAT plate type."""
        self.assertNotEqual(datum_key("a", "10x", "p24"),
                            datum_key("b", "10x", "p24"))
        self.assertNotEqual(datum_key("a", "10x", "p24"),
                            datum_key("a", "20x", "p24"))
        self.assertNotEqual(datum_key("a", "10x", "p24"),
                            datum_key("a", "10x", "p96"))

    def test_keys_are_filesystem_safe(self):
        self.assertNotIn("/", datum_key("../etc", "10x/../", "p"))

    def test_a_second_objective_does_not_overwrite_the_first(self):
        self.store.save("cam", "4x", "p", focal_sign=1, offset_mm=1.0)
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=2.0)
        self.assertAlmostEqual(self.store.get("cam", "4x", "p")["offset_mm"], 1.0)
        self.assertAlmostEqual(self.store.get("cam", "10x", "p")["offset_mm"], 2.0)


class TestStaleness(_StoreCase):
    def test_set_z_zero_makes_a_datum_stale(self):
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=1.0,
                        zero_z_mm=0.0)
        self.assertTrue(self.store.is_stale("cam", "10x", "p", 9.0))

    def test_an_unchanged_epoch_is_not_stale(self):
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=1.0,
                        zero_z_mm=3.0)
        self.assertFalse(self.store.is_stale("cam", "10x", "p", 3.0))

    def test_a_stale_datum_is_kept_not_deleted(self):
        """The operator needs the old number visible to judge how far things
        moved; silently discarding it destroys that evidence."""
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=1.0,
                        zero_z_mm=0.0)
        self.assertTrue(self.store.is_stale("cam", "10x", "p", 9.0))
        self.assertIsNotNone(self.store.get("cam", "10x", "p"))

    def test_unknown_epoch_is_not_claimed_to_be_fresh_or_stale(self):
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=1.0)
        self.assertFalse(self.store.is_stale("cam", "10x", "p", 9.0))


class TestDiagnostics(_StoreCase):
    def test_the_measured_scale_is_recorded_even_though_it_is_not_applied(self):
        """Recorded for diagnosis: if it drifts run to run, something changed."""
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=1.0,
                        measured_scale=0.9993, closure_um=4.2, n_points=6,
                        span_um=2000.0)
        rec = self.store.get("cam", "10x", "p")
        self.assertAlmostEqual(rec["measured_scale"], 0.9993)
        self.assertAlmostEqual(rec["closure_um"], 4.2)
        self.assertEqual(rec["n_points"], 6)

    def test_optional_fields_are_omitted_when_unset(self):
        self.store.save("cam", "10x", "p", focal_sign=1, offset_mm=1.0)
        self.assertNotIn("closure_um", self.store.get("cam", "10x", "p"))


class TestSingleton(unittest.TestCase):
    def test_env_override_isolates_the_store(self):
        with tempfile.TemporaryDirectory() as d:
            old = os.environ.get("MEBP_PLATE_FOCUS_DATUM_DIR")
            os.environ["MEBP_PLATE_FOCUS_DATUM_DIR"] = d
            mod.reset_store()
            try:
                mod.get_store().save("c", "o", "p", focal_sign=1, offset_mm=1.0)
                self.assertTrue(os.path.exists(
                    os.path.join(d, "plate_focus_datum.json")))
            finally:
                if old is None:
                    os.environ.pop("MEBP_PLATE_FOCUS_DATUM_DIR", None)
                else:
                    os.environ["MEBP_PLATE_FOCUS_DATUM_DIR"] = old
                mod.reset_store()


if __name__ == "__main__":
    unittest.main()
