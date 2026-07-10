"""
v7.5.x tests — last-known-good calibration snapshot (needle + plate + Z).

After a restart with an unchanged setup, operators had to redo the entire
calibration. The fix persists a single durable snapshot (needle zero + plate +
Z + a hardware fingerprint + timestamp) to config/hardware/last_calibration.json,
and on the next launch offers to restore it when the live calibration came up
empty. These tests lock down the backend the GUI prompt stands on:

  1. CalibrationSnapshotStore round-trip + atomic overwrite + clear.
  2. build_fingerprint reads the right settings keys (with plate-format
     fallback).
  3. fingerprint_diff reports device / plate / needle / axis-map changes and is
     silent for an unchanged setup or a missing saved fingerprint.
  4. CalibrationPage._has_plate_calibration — the keystone gate that stops an
     empty page from clobbering a good snapshot and decides the startup prompt.

The QMessageBox prompt itself is exercised manually (as with the ZP
position-restore prompt).
"""

import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from SupportClasses.CalibrationSnapshotStore import CalibrationSnapshotStore


class _FakeSettings:
    """Minimal Settings stand-in: dot-path .get(path, default)."""

    def __init__(self, data: dict):
        self._d = data

    def get(self, dotpath, default=None):
        node = self._d
        for key in dotpath.split("."):
            if isinstance(node, dict) and key in node:
                node = node[key]
            else:
                return default
        return node


_ZERO = {"x": 1000.0, "y": 2000.0, "Z": -37.17, "P1": 0.0, "P2": 0.01, "P3": 0.0}
_CAL = {
    "plate_format": 24,
    "taught_a1": [1000.0, 2000.0],
    "safe_z": -34.68,
    "z_plane": {"a": 0.0, "b": 0.0, "c": -25.0, "r_squared": 1.0},
}
_FP = {
    "device": "ME3B V1",
    "axis_map": {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"},
    "steps_per_mm": {"Z": 5255.0},
    "plate_format": 24,
    "needle_gauge": 27,
}


class TestSnapshotStore(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp()) / "last_calibration.json"
        self.addCleanup(self._cleanup)

    def _cleanup(self):
        try:
            if self.tmp.exists():
                self.tmp.unlink()
            self.tmp.parent.rmdir()
        except OSError:
            pass

    def test_round_trip(self):
        store = CalibrationSnapshotStore(self.tmp)
        self.assertIsNone(store.load_snapshot())
        self.assertFalse(store.has_snapshot())
        store.save_snapshot(_ZERO, _CAL, _FP, saved_at="2026-06-15T17:30:00")

        # Fresh store reads it back off disk.
        store2 = CalibrationSnapshotStore(self.tmp)
        snap = store2.load_snapshot()
        self.assertTrue(store2.has_snapshot())
        self.assertEqual(snap["saved_at"], "2026-06-15T17:30:00")
        self.assertEqual(snap["calibration"]["taught_a1"], [1000.0, 2000.0])
        self.assertEqual(snap["zero_position"]["Z"], -37.17)
        self.assertEqual(snap["fingerprint"]["device"], "ME3B V1")
        self.assertEqual(snap["version"], "1.0")

    def test_overwrite_is_atomic_and_leaves_no_temp(self):
        store = CalibrationSnapshotStore(self.tmp)
        store.save_snapshot(_ZERO, _CAL, _FP, saved_at="t1")
        store.save_snapshot(
            {**_ZERO, "Z": -10.0}, {**_CAL, "plate_format": 6}, _FP,
            saved_at="t2")
        snap = CalibrationSnapshotStore(self.tmp).load_snapshot()
        self.assertEqual(snap["saved_at"], "t2")
        self.assertEqual(snap["calibration"]["plate_format"], 6)
        self.assertEqual(snap["zero_position"]["Z"], -10.0)
        # No stray temp file left behind by the atomic replace.
        self.assertFalse(
            (self.tmp.with_name(self.tmp.name + ".tmp")).exists())

    def test_clear_removes_file(self):
        store = CalibrationSnapshotStore(self.tmp)
        store.save_snapshot(_ZERO, _CAL, _FP)
        self.assertTrue(self.tmp.exists())
        store.clear()
        self.assertFalse(self.tmp.exists())
        self.assertIsNone(store.load_snapshot())

    def test_has_snapshot_false_for_empty_calibration(self):
        store = CalibrationSnapshotStore(self.tmp)
        store.save_snapshot(_ZERO, {}, _FP)  # no calibration payload
        self.assertFalse(store.has_snapshot())

    def test_corrupt_file_loads_as_none(self):
        self.tmp.parent.mkdir(parents=True, exist_ok=True)
        self.tmp.write_text("{ not json", encoding="utf-8")
        store = CalibrationSnapshotStore(self.tmp)
        self.assertIsNone(store.load_snapshot())
        self.assertFalse(store.has_snapshot())


class TestBuildFingerprint(unittest.TestCase):
    def test_reads_settings_keys(self):
        s = _FakeSettings({
            "device_profile": {
                "active": "ME3B V1",
                "axis_map": {"Z": "Z", "P1": "X"},
                "steps_per_mm": {"Z": 5255.0},
            },
            "hardware_config": {"plate_format": 24, "needle": {"gauge": 27}},
        })
        fp = CalibrationSnapshotStore.build_fingerprint(s)
        self.assertEqual(fp["device"], "ME3B V1")
        self.assertEqual(fp["axis_map"], {"Z": "Z", "P1": "X"})
        self.assertEqual(fp["plate_format"], 24)
        self.assertEqual(fp["needle_gauge"], 27)

    def test_plate_format_falls_back_to_calibration_section(self):
        s = _FakeSettings({"calibration": {"plate_format": 96}})
        fp = CalibrationSnapshotStore.build_fingerprint(s)
        self.assertEqual(fp["plate_format"], 96)

    def test_missing_keys_are_none_or_empty(self):
        fp = CalibrationSnapshotStore.build_fingerprint(_FakeSettings({}))
        self.assertIsNone(fp["device"])
        self.assertEqual(fp["axis_map"], {})
        self.assertEqual(fp["steps_per_mm"], {})


class TestFingerprintDiff(unittest.TestCase):
    def test_unchanged_is_empty(self):
        self.assertEqual(
            CalibrationSnapshotStore.fingerprint_diff(dict(_FP), dict(_FP)), [])

    def test_missing_saved_is_empty(self):
        # An older snapshot with no fingerprint must not manufacture a warning.
        self.assertEqual(
            CalibrationSnapshotStore.fingerprint_diff(None, dict(_FP)), [])

    def test_device_change_reported(self):
        cur = {**_FP, "device": "Standard"}
        diffs = CalibrationSnapshotStore.fingerprint_diff(_FP, cur)
        self.assertEqual(len(diffs), 1)
        self.assertIn("device profile", diffs[0])
        self.assertIn("ME3B V1", diffs[0])
        self.assertIn("Standard", diffs[0])

    def test_multiple_changes_reported(self):
        cur = {**_FP, "plate_format": 6, "needle_gauge": 22}
        diffs = CalibrationSnapshotStore.fingerprint_diff(_FP, cur)
        joined = " | ".join(diffs)
        self.assertIn("plate format", joined)
        self.assertIn("needle gauge", joined)
        self.assertEqual(len(diffs), 2)

    def test_axis_map_change_reported(self):
        cur = {**_FP, "axis_map": {"Z": "X"}}
        diffs = CalibrationSnapshotStore.fingerprint_diff(_FP, cur)
        self.assertTrue(any("axis map" in d for d in diffs))


class TestHasPlateCalibration(unittest.TestCase):
    """The keystone gate (taught A1 / warp / non-identity 3-well affine).

    Imported off the GUI page but invoked unbound against a fake `self` so the
    test needs no Qt application — only the pure attribute logic is exercised.
    """

    @classmethod
    def setUpClass(cls):
        try:
            from gui.pages.calibration import CalibrationPage
        except Exception as exc:  # PySide6/Qt unavailable in this env
            raise unittest.SkipTest(f"CalibrationPage import failed: {exc}")
        cls._page_cls = CalibrationPage

    def _call(self, **attrs):
        defaults = {"_taught_a1": None, "_plate_warp": None,
                    "_three_well_calibration": None}
        defaults.update(attrs)
        # Invoke unbound against a fake `self` (no Qt construction needed).
        return self._page_cls._has_plate_calibration(SimpleNamespace(**defaults))

    def test_empty_is_false(self):
        self.assertFalse(self._call())

    def test_taught_a1_is_true(self):
        self.assertTrue(self._call(_taught_a1=(1000.0, 2000.0)))

    def test_warp_is_true(self):
        self.assertTrue(self._call(_plate_warp=object()))

    def test_identity_affine_is_false(self):
        self.assertFalse(
            self._call(_three_well_calibration=SimpleNamespace(is_identity=True)))

    def test_non_identity_affine_is_true(self):
        self.assertTrue(
            self._call(_three_well_calibration=SimpleNamespace(is_identity=False)))


class TestApplyAndSaveAfterRestore(unittest.TestCase):
    """v7.5.x: accepting a restore popup now does the Device-page 'Apply + Save'
    automatically — mirror the persisted safety_limits into the LIVE envelope
    (in place, so jog handlers' reference stays valid) and persist to disk now.

    Invoked unbound against a fake `self` (no Qt construction needed).
    """

    @classmethod
    def setUpClass(cls):
        try:
            from gui.app import MainWindow
        except Exception as exc:  # PySide6/Qt unavailable in this env
            raise unittest.SkipTest(f"MainWindow import failed: {exc}")
        cls._mw = MainWindow

    def _fake(self, save_fn=None, pump_setup=None, saved_pump_limits=None):
        from SupportClasses.SafetyLimits import SafetyLimits
        live = SafetyLimits()
        live.z_min, live.z_max, live.enabled = 999.0, 999.0, False  # stale
        saved = []

        class _Settings:
            def get_section(self, name):
                if name == "safety_limits":
                    sec = {"z_min": -48.2, "z_max": 1.8, "enabled": True,
                           "max_z_feedrate": 500.0}
                    if saved_pump_limits:
                        sec.update(saved_pump_limits)
                    return sec
                return {}

        fake = SimpleNamespace(
            controller=SimpleNamespace(
                safety_limits=live,
                get_pump_setup=(lambda: dict(pump_setup or {})),
            ),
            settings=_Settings(),
            _page_widgets=[],
            save_settings=(save_fn or (lambda: saved.append(True))),
        )
        return fake, live, saved

    def test_mirrors_limits_in_place_and_saves(self):
        fake, live, saved = self._fake()
        self._mw._apply_and_save_after_restore(fake, "test")
        # Mutated in place (same object the jog handlers reference), not rebound.
        self.assertIs(fake.controller.safety_limits, live)
        self.assertAlmostEqual(live.z_min, -48.2)
        self.assertAlmostEqual(live.z_max, 1.8)
        self.assertTrue(live.enabled)
        self.assertAlmostEqual(live.max_z_feedrate, 500.0)
        self.assertTrue(saved, "save_settings was not called on accept")

    def test_save_failure_is_non_fatal(self):
        def boom():
            raise RuntimeError("disk full")
        fake, live, _ = self._fake(save_fn=boom)
        # Best-effort: a save failure must not raise (limits already applied).
        self._mw._apply_and_save_after_restore(fake, "test")
        self.assertAlmostEqual(live.z_min, -48.2)

    def test_calibrated_pump_envelope_survives_stale_mirror(self):
        """Regression: a calibrated pump's soft-limit envelope is owned by the
        plunger calibration (pump_setup), NOT the persisted safety_limits mirror.

        Reproduces the ME3B V3 "Quick Print goes to pick up ink and nothing
        happens" bug: the mirror on disk was the sign-flipped [0, 30] while the
        captured extremes (raw 0 → -30) imply [-30, 0]. Mirroring the stale
        value here set p2_min=0, so every aspirate (toward negative raw mm)
        clamped to 0 and the shorten-only guard zeroed the move. After the fix,
        the calibrated envelope is re-derived from the captured extremes and
        wins over the stale mirror."""
        fake, live, _ = self._fake(
            pump_setup={"P2": {"raw_dispensed": 0.0, "raw_aspirated": -30.0,
                               "aspirate_sign": -1.0, "capacity_uL": 250.0}},
            saved_pump_limits={"p2_min": 0.0, "p2_max": 30.0},  # stale/flipped
        )
        self._mw._apply_and_save_after_restore(fake, "test")
        # Calibration wins: envelope re-derived from min/max(0, -30) = [-30, 0].
        self.assertAlmostEqual(live.p2_min, -30.0)
        self.assertAlmostEqual(live.p2_max, 0.0)
        # Non-pump (Device-owned) limits still mirror from disk.
        self.assertAlmostEqual(live.z_min, -48.2)


if __name__ == "__main__":
    unittest.main()
