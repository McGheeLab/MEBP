"""Tests for v7.5.x plate-bottom Z datum + universal "don't punch through".

Covers:
  - the polarity-general module helpers (``plate_relative_to_zref`` /
    ``zref_to_plate_relative``) for both Z directions;
  - the StageController datum API (``set/get_plate_bottom_z``,
    ``print_height_to_zref``, ``print_floor_violation``);
  - the print-floor clamp ``_apply_print_floor_raw`` — armed/disarmed,
    calibrated/uncalibrated, with a zero offset, and both polarities;
  - PrintManager arming the controller floor via ``_arm_print_floor``;
  - Sketch ``z_above_plate_bottom_mm`` metadata persisted by
    ``save_trajectory_as_print_object``.
"""

import importlib
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# NB: ``SupportClasses/__init__`` re-exports the StageController *class* under
# the name ``StageController``, shadowing the submodule attribute — so fetch
# the real module object via importlib to monkeypatch its module-level ZDIR.
SC = importlib.import_module("SupportClasses.StageController")
from SupportClasses.StageController import (
    StageController, plate_relative_to_zref, zref_to_plate_relative,
)


class TestPlateRelativeHelpers(unittest.TestCase):
    """Polarity-general height-above-bottom ↔ zero-ref conversions."""

    def test_round_trip_inverted(self):
        pb = -25.68  # ME3B V1 well floor (ZDIR=-1)
        for h in (0.0, 0.2, 5.0, 12.3):
            z = plate_relative_to_zref(pb, h, zdir=-1.0)
            self.assertAlmostEqual(z, pb - h)             # higher → smaller zref
            self.assertAlmostEqual(zref_to_plate_relative(pb, z, zdir=-1.0), h)

    def test_round_trip_conventional(self):
        pb = 2.0
        for h in (0.0, 0.2, 5.0):
            z = plate_relative_to_zref(pb, h, zdir=1.0)
            self.assertAlmostEqual(z, pb + h)             # higher → larger zref
            self.assertAlmostEqual(zref_to_plate_relative(pb, z, zdir=1.0), h)

    def test_violation_sign_inverted(self):
        pb = -25.68
        # On ME3B, "deeper than the floor" means a LARGER zero-ref Z.
        self.assertLess(zref_to_plate_relative(pb, -20.0, zdir=-1.0), 0)
        self.assertGreater(zref_to_plate_relative(pb, -30.0, zdir=-1.0), 0)


class TestPrintFloorController(unittest.TestCase):
    """Datum API + clamp on a lightweight (un-constructed) controller."""

    def _ctrl(self):
        # Avoid the heavy __init__ (serial / pollers); we only exercise the
        # pure datum/clamp logic, which needs just these attributes.
        c = StageController.__new__(StageController)
        c._plate_bottom_z_zref = None
        c._print_floor_active = False
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0,
                           "P1": 0.0, "P2": 0.0, "P3": 0.0}
        return c

    def test_set_get_datum(self):
        c = self._ctrl()
        c.set_plate_bottom_z(-25.68)
        self.assertAlmostEqual(c.get_plate_bottom_z(), -25.68)
        c.set_plate_bottom_z(None)
        self.assertIsNone(c.get_plate_bottom_z())

    def test_print_height_to_zref(self):
        c = self._ctrl()
        self.assertIsNone(c.print_height_to_zref(0.2))   # uncalibrated → None
        c.set_plate_bottom_z(-25.68)
        self.assertAlmostEqual(c.print_height_to_zref(0.0), -25.68)
        self.assertAlmostEqual(c.print_height_to_zref(0.2), -25.88)  # ZDIR=-1
        # round trip
        self.assertAlmostEqual(c.zref_to_print_height(-25.88), 0.2)

    def test_violation(self):
        c = self._ctrl()
        self.assertFalse(c.print_floor_violation(-20.0))  # uncalibrated → safe
        c.set_plate_bottom_z(-25.68)
        self.assertFalse(c.print_floor_violation(-25.88))  # above the floor
        self.assertFalse(c.print_floor_violation(-25.68))  # exactly at floor
        self.assertTrue(c.print_floor_violation(-20.0))    # below (deeper)

    def test_clamp_disarmed_is_noop(self):
        c = self._ctrl()
        c.set_plate_bottom_z(-25.68)  # calibrated but NOT armed
        self.assertEqual(c._apply_print_floor_raw(-10.0), -10.0)

    def test_clamp_uncalibrated_is_noop(self):
        c = self._ctrl()
        c.set_print_floor_active(True)  # armed but no datum
        self.assertEqual(c._apply_print_floor_raw(-10.0), -10.0)

    def test_clamp_armed_inverted(self):
        c = self._ctrl()
        c.set_plate_bottom_z(-25.68)
        c.set_print_floor_active(True)
        # raw -20 is deeper than the floor (-25.68) on ME3B → clamp to floor
        self.assertAlmostEqual(c._apply_print_floor_raw(-20.0), -25.68)
        # raw -30 is higher (safe) → unchanged
        self.assertAlmostEqual(c._apply_print_floor_raw(-30.0), -30.0)
        # exactly at floor → unchanged
        self.assertAlmostEqual(c._apply_print_floor_raw(-25.68), -25.68)

    def test_clamp_honours_zero_offset(self):
        c = self._ctrl()
        c.set_plate_bottom_z(-25.68)
        c.set_print_floor_active(True)
        c.zero_position["Z"] = 5.0           # pb_raw = -25.68 + 5 = -20.68
        self.assertAlmostEqual(c._apply_print_floor_raw(-15.0), -20.68)
        self.assertAlmostEqual(c._apply_print_floor_raw(-25.0), -25.0)

    def test_clamp_conventional_polarity(self):
        old = SC.ZDIR
        SC.ZDIR = 1.0
        try:
            c = self._ctrl()
            c.set_plate_bottom_z(2.0)
            c.set_print_floor_active(True)
            # conventional: deeper = SMALLER raw; floor at 2.0
            self.assertAlmostEqual(c._apply_print_floor_raw(1.0), 2.0)
            self.assertAlmostEqual(c._apply_print_floor_raw(5.0), 5.0)
        finally:
            SC.ZDIR = old


class TestPrintManagerArming(unittest.TestCase):
    """PrintManager arms/disarms the controller's floor."""

    def test_arm_print_floor_calls_controller(self):
        from SupportClasses.PrintManager import PrintManager

        class _StubController:
            def __init__(self):
                self.calls = []

            def set_print_floor_active(self, active):
                self.calls.append(active)

        ctrl = _StubController()
        pm = PrintManager(ctrl)
        pm._arm_print_floor(True)
        pm._arm_print_floor(False)
        self.assertEqual(ctrl.calls, [True, False])

    def test_arm_print_floor_tolerates_missing_method(self):
        from SupportClasses.PrintManager import PrintManager

        class _Bare:
            pass

        pm = PrintManager(_Bare())
        # Must not raise even if the controller lacks the method.
        pm._arm_print_floor(True)


class TestSketchMetadataPersistence(unittest.TestCase):
    """The Sketch's relative print height is persisted as object metadata."""

    def test_extra_params_written(self):
        import json
        import numpy as np
        from SupportClasses.PrintFileManager import save_trajectory_as_print_object

        traj = np.zeros((3, 7), dtype=float)
        traj[:, 0] = [0.0, 1.0, 2.0]      # x
        traj[:, 2] = -25.48               # z (already baked to zero-ref)
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                traj, base_name="MyDisc", object_name="MyDisc",
                prints_dir=d, source="SketchTrajectory",
                extra_params={"z_above_plate_bottom_mm": 0.2,
                              "z_datum": "plate_bottom"},
            )
            self.assertTrue(name.startswith("MyDisc"))
            with open(os.path.join(d, f"{name}.json")) as f:
                data = json.load(f)
            obj = next(iter(data["objects"].values()))
            params = obj["params"]
            self.assertEqual(obj["object_type"], "csv_import")
            self.assertAlmostEqual(params["z_above_plate_bottom_mm"], 0.2)
            self.assertEqual(params["z_datum"], "plate_bottom")


if __name__ == "__main__":
    unittest.main()
