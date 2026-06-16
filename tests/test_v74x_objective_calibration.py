"""
test_v74x_objective_calibration.py — Microscope objective calibration backend.

Covers the backend pieces of the per-objective µm/px calibration feature
added on top of the v7.4.4 camera-role work:

* `CameraRole.MICROSCOPE` exists, serializes, and round-trips through
  `HardwareConfig.to_dict()` / `from_dict()`.
* `CameraConfig.current_objective_name` round-trips and is backwards
  compatible with v7.4.x configs that omit the field.
* `HardwareConfig.set_camera_role()` enforces single-slot ownership for
  every role in `SINGLETON_CAMERA_ROLES`.
* `ObjectiveCalibrationStore` write/read/clear round-trip against a
  temporary JSON file.
* Pure-math sanity check for µm/px = real_world_um / pixel_distance.

No GUI or hardware required.
"""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from SupportClasses.HardwareConfig import (
    HardwareConfig,
    CameraConfig,
    CameraRole,
    SINGLETON_CAMERA_ROLES,
    MAX_LIVE_CAMERAS,
)
from SupportClasses.ObjectiveCalibration import ObjectiveCalibrationStore


# ---------------------------------------------------------------------------
# CameraRole.MICROSCOPE
# ---------------------------------------------------------------------------


class TestMicroscopeRole(unittest.TestCase):
    def test_microscope_role_value(self):
        self.assertEqual(CameraRole.MICROSCOPE.value, "microscope")

    def test_microscope_is_a_singleton_role(self):
        self.assertIn(CameraRole.MICROSCOPE, SINGLETON_CAMERA_ROLES)
        # Sanity: UNASSIGNED is *not* a singleton role.
        self.assertNotIn(CameraRole.UNASSIGNED, SINGLETON_CAMERA_ROLES)

    def test_microscope_role_round_trips_through_json(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(1, CameraRole.MICROSCOPE)
        as_dict = cfg.to_dict()
        self.assertEqual(as_dict["camera_roles"][1], "microscope")
        restored = HardwareConfig.from_dict(as_dict)
        self.assertEqual(restored.camera_roles[1], CameraRole.MICROSCOPE)
        self.assertEqual(restored.camera_for_role(CameraRole.MICROSCOPE), 1)


# ---------------------------------------------------------------------------
# Singleton enforcement on set_camera_role
# ---------------------------------------------------------------------------


class TestSingletonEnforcement(unittest.TestCase):
    def test_microscope_role_is_unique_after_reassignment(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.MICROSCOPE)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        self.assertEqual(cfg.camera_roles[0], CameraRole.UNASSIGNED)
        self.assertEqual(cfg.camera_roles[2], CameraRole.MICROSCOPE)

    def test_needle_and_microscope_roles_are_unique(self):
        for role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y, CameraRole.MICROSCOPE):
            cfg = HardwareConfig()
            cfg.set_camera_role(0, role)
            cfg.set_camera_role(1, role)
            with self.subTest(role=role):
                self.assertEqual(cfg.camera_roles[0], CameraRole.UNASSIGNED)
                self.assertEqual(cfg.camera_roles[1], role)

    def test_unassigned_does_not_clear_other_slots(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.MICROSCOPE)
        cfg.set_camera_role(1, CameraRole.NEEDLE_X)
        cfg.set_camera_role(2, CameraRole.UNASSIGNED)
        self.assertEqual(cfg.camera_roles[0], CameraRole.MICROSCOPE)
        self.assertEqual(cfg.camera_roles[1], CameraRole.NEEDLE_X)

    def test_distinct_singleton_roles_can_coexist(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.MICROSCOPE)
        cfg.set_camera_role(1, CameraRole.NEEDLE_X)
        self.assertEqual(cfg.camera_roles[0], CameraRole.MICROSCOPE)
        self.assertEqual(cfg.camera_roles[1], CameraRole.NEEDLE_X)

    def test_out_of_range_index_is_no_op(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(MAX_LIVE_CAMERAS + 5, CameraRole.MICROSCOPE)
        self.assertEqual(cfg.camera_for_role(CameraRole.MICROSCOPE), None)


# ---------------------------------------------------------------------------
# CameraConfig.current_objective_name
# ---------------------------------------------------------------------------


class TestCurrentObjectiveName(unittest.TestCase):
    def test_default_is_none(self):
        cfg = CameraConfig()
        self.assertIsNone(cfg.current_objective_name)

    def test_round_trips_through_dict(self):
        cfg = CameraConfig(current_objective_name="4x")
        restored = CameraConfig.from_dict(cfg.to_dict())
        self.assertEqual(restored.current_objective_name, "4x")

    def test_backwards_compatible_with_legacy_json(self):
        # A v7.4.4-shape dict without the new field deserializes cleanly.
        legacy_dict = {
            "camera_spec": None,
            "objective_magnification": 4.0,
            "active_resolution": [916, 686],
            "micron_per_pixel_override": None,
            "camera_to_needle_offset_um": [0.0, 0.0],
        }
        cfg = CameraConfig.from_dict(legacy_dict)
        self.assertIsNone(cfg.current_objective_name)
        self.assertEqual(cfg.objective_magnification, 4.0)

    def test_persists_through_full_hardware_config_save_load(self):
        cfg = HardwareConfig()
        cfg.camera_config.current_objective_name = "10x"
        cfg.set_camera_role(0, CameraRole.MICROSCOPE)
        restored = HardwareConfig.from_dict(cfg.to_dict())
        self.assertEqual(restored.camera_config.current_objective_name, "10x")
        self.assertEqual(restored.camera_for_role(CameraRole.MICROSCOPE), 0)


# ---------------------------------------------------------------------------
# ObjectiveCalibrationStore round-trip
# ---------------------------------------------------------------------------


class TestObjectiveCalibrationStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "objectives.json"

    def tearDown(self):
        self._tmp.cleanup()

    def test_set_and_get_round_trip(self):
        store = ObjectiveCalibrationStore(self.path)
        store.set_calibration("CAM-A", "4x", 0.823, (916, 686))
        cal = store.get_calibration("CAM-A", "4x")
        self.assertIsNotNone(cal)
        self.assertAlmostEqual(cal["measured_um_per_px"], 0.823, places=6)
        self.assertEqual(cal["resolution"], [916, 686])
        self.assertTrue(cal["date"])  # ISO date stamp

    def test_rotation_deg_round_trips_when_supplied(self):
        # v7.5.x: the stage-motion dialog reports an in-plane rotation; the
        # store persists it alongside the per-objective µm/px.
        store = ObjectiveCalibrationStore(self.path)
        store.set_calibration("CAM-A", "4x", 0.823, (916, 686), rotation_deg=45.0)
        cal = store.get_calibration("CAM-A", "4x")
        self.assertAlmostEqual(cal["rotation_deg"], 45.0, places=3)
        # Omitting it keeps the entry backwards compatible (no key).
        store.set_calibration("CAM-A", "10x", 0.421, (916, 686))
        self.assertNotIn("rotation_deg", store.get_calibration("CAM-A", "10x"))

    def test_persists_to_disk_and_reloads(self):
        store_a = ObjectiveCalibrationStore(self.path)
        store_a.set_calibration("CAM-A", "10x", 0.421, (1832, 1374))
        # Independent instance pointed at the same file picks up the entry.
        store_b = ObjectiveCalibrationStore(self.path)
        cal = store_b.get_calibration("CAM-A", "10x")
        self.assertIsNotNone(cal)
        self.assertAlmostEqual(cal["measured_um_per_px"], 0.421, places=6)

    def test_clear_removes_entry(self):
        store = ObjectiveCalibrationStore(self.path)
        store.set_calibration("CAM-A", "4x", 0.823, (916, 686))
        store.clear_calibration("CAM-A", "4x")
        self.assertIsNone(store.get_calibration("CAM-A", "4x"))

    def test_all_calibrations_for_camera(self):
        store = ObjectiveCalibrationStore(self.path)
        store.set_calibration("CAM-A", "4x", 0.823, (916, 686))
        store.set_calibration("CAM-A", "10x", 0.421, (916, 686))
        store.set_calibration("CAM-B", "4x", 0.900, (1920, 1080))
        all_a = store.all_calibrations_for_camera("CAM-A")
        self.assertEqual(set(all_a.keys()), {"4x", "10x"})
        self.assertNotIn("4x_extra", all_a)

    def test_independent_cameras_do_not_collide(self):
        store = ObjectiveCalibrationStore(self.path)
        store.set_calibration("CAM-A", "4x", 0.823, (916, 686))
        store.set_calibration("CAM-B", "4x", 0.900, (1920, 1080))
        self.assertAlmostEqual(
            store.get_calibration("CAM-A", "4x")["measured_um_per_px"],
            0.823, places=6,
        )
        self.assertAlmostEqual(
            store.get_calibration("CAM-B", "4x")["measured_um_per_px"],
            0.900, places=6,
        )

    def test_first_time_store_has_no_objectives(self):
        # v7.4.x: objectives are user-defined; no prepopulated list.
        store = ObjectiveCalibrationStore(self.path)
        self.assertEqual(store.objective_names(), [])

    def test_add_objective_persists(self):
        store_a = ObjectiveCalibrationStore(self.path)
        self.assertTrue(store_a.add_objective("My 4x", 4.0))
        self.assertIn("My 4x", store_a.objective_names())
        self.assertEqual(store_a.nominal_magnification("My 4x"), 4.0)
        # Independent reader sees the same entry.
        store_b = ObjectiveCalibrationStore(self.path)
        self.assertIn("My 4x", store_b.objective_names())

    def test_add_objective_rejects_duplicate_name(self):
        store = ObjectiveCalibrationStore(self.path)
        self.assertTrue(store.add_objective("4x", 4.0))
        self.assertFalse(store.add_objective("4x", 4.12))
        # Original entry untouched.
        self.assertEqual(store.nominal_magnification("4x"), 4.0)

    def test_remove_objective_clears_calibrations(self):
        store = ObjectiveCalibrationStore(self.path)
        store.add_objective("10x", 10.0)
        store.set_calibration("CAM-A", "10x", 0.421, (916, 686))
        self.assertTrue(store.remove_objective("10x"))
        self.assertNotIn("10x", store.objective_names())
        # Calibration for the removed objective is gone.
        self.assertIsNone(store.get_calibration("CAM-A", "10x"))

    def test_remove_objective_returns_false_when_missing(self):
        store = ObjectiveCalibrationStore(self.path)
        self.assertFalse(store.remove_objective("nonexistent"))


# ---------------------------------------------------------------------------
# Pure-math sanity check (the formula the dialog uses)
# ---------------------------------------------------------------------------


class TestCalibrationMath(unittest.TestCase):
    def _um_per_px(self, distance_value: float, unit: str, dx: float, dy: float) -> float:
        real_um = distance_value * (1000.0 if unit == "mm" else 1.0)
        pix = math.hypot(dx, dy)
        return real_um / pix

    def test_one_mm_over_1000_px_gives_one_um_per_px(self):
        self.assertAlmostEqual(
            self._um_per_px(1.0, "mm", 1000.0, 0.0),
            1.0, places=9,
        )

    def test_500_um_diagonal_gives_known_value(self):
        # 500 µm across a 300/400 right triangle (hypotenuse 500 px) → 1.0 µm/px.
        self.assertAlmostEqual(
            self._um_per_px(500.0, "µm", 300.0, 400.0),
            1.0, places=9,
        )

    def test_subpixel_endpoints_produce_continuous_result(self):
        a = self._um_per_px(1.0, "mm", 1234.5, 0.0)
        b = self._um_per_px(1.0, "mm", 1234.6, 0.0)
        self.assertNotEqual(a, b)
        # A 0.1-px shift on a ~1234.5-px baseline moves µm/px by < 1e-4
        # in absolute terms — assert the relative delta stays tiny.
        self.assertLess(abs(a - b) / a, 1e-3)


if __name__ == "__main__":
    unittest.main()
