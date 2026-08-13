"""test_v717_machine_config.py — per-machine vs shared config resolution.

The load-bearing claim of ``SupportClasses/MachineConfig.py`` is that one git
repo can be shared across several physical rigs without their calibration
colliding. The tests that matter most here are the ones pinning what must NOT
happen:

  * nothing is ever MOVED while the machine id is unconfigured — the failure
    that already happened once on the bench, where importing ``gui.app``
    (which pulls in five per-machine stores) relocated real calibration into
    the ``unassigned`` bucket before the operator could say which rig it was,
    leaving the app looking at an empty folder afterwards;
  * ``main.py`` imports ``gui.app`` only AFTER the machine-id prompt, checked
    structurally (by AST) because a future edit tidying the imports back to
    the top of the function would silently reintroduce exactly that;
  * the leftover-file sweep never files a per-machine store as SHARED, since
    "shared" means "committed to git and published to every other rig".
"""

from __future__ import annotations

import ast
import importlib
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from SupportClasses import MachineConfig as mc

REPO = Path(__file__).resolve().parent.parent


class _IsolatedRoot(unittest.TestCase):
    """Point MachineConfig at a throwaway config/hardware tree."""

    def setUp(self):
        self._tmp = Path(tempfile.mkdtemp(prefix="mebp_machine_cfg_"))
        self._old_root = mc.HARDWARE_ROOT
        self._old_settings = mc._settings_path
        self._old_env = os.environ.pop("MEBP_MACHINE_ID", None)
        mc.HARDWARE_ROOT = self._tmp / "config" / "hardware"
        mc.HARDWARE_ROOT.mkdir(parents=True)
        mc.set_settings_path(self._tmp / "settings.json")

    def tearDown(self):
        mc.HARDWARE_ROOT = self._old_root
        mc._settings_path = self._old_settings
        if self._old_env is not None:
            os.environ["MEBP_MACHINE_ID"] = self._old_env
        else:
            os.environ.pop("MEBP_MACHINE_ID", None)
        mc.reset_cache()
        shutil.rmtree(self._tmp, ignore_errors=True)

    def _legacy(self, name: str, body: str = '{"v": 1}') -> Path:
        p = mc.HARDWARE_ROOT / name
        p.write_text(body, encoding="utf-8")
        return p


# ════════════════════════════════════════════════════════════════════
#  A. Unconfigured must never relocate anything
# ════════════════════════════════════════════════════════════════════

class TestUnconfiguredNeverMigrates(_IsolatedRoot):

    def test_legacy_file_is_read_in_place_not_moved(self):
        legacy = self._legacy("objectives.json")
        self.assertFalse(mc.machine_id_is_configured())

        resolved = mc.resolve_machine_path("objectives.json")

        # The decisive assertion: the file is still where it was, and the
        # store was pointed AT it (so the app behaves exactly as pre-split).
        self.assertTrue(legacy.exists(), "unconfigured resolution MOVED the file")
        self.assertEqual(resolved, legacy)
        self.assertFalse((mc.HARDWARE_ROOT / mc.UNASSIGNED_ID).exists(),
                         "the fallback bucket must not even be created")

    def test_legacy_directory_is_not_moved_either(self):
        d = mc.HARDWARE_ROOT / "well_training" / "24_20260101_000000"
        d.mkdir(parents=True)
        (d / "labels.json").write_text("{}", encoding="utf-8")

        resolved = mc.resolve_machine_path("well_training")

        self.assertTrue((d / "labels.json").exists())
        self.assertEqual(resolved, mc.HARDWARE_ROOT / "well_training")

    def test_absent_file_points_at_the_fallback_bucket(self):
        # Nothing to read, so a WRITE still has somewhere real to land — but
        # it is namespaced, never colliding with a real rig's folder.
        resolved = mc.resolve_machine_path("microscope.json")
        self.assertEqual(resolved,
                         mc.HARDWARE_ROOT / mc.UNASSIGNED_ID / "microscope.json")

    def test_shared_config_still_migrates_when_unconfigured(self):
        # Shared config is machine-INDEPENDENT, so there is no ambiguity to
        # protect against and it may move regardless of the id.
        legacy = self._legacy("needles.json")
        resolved = mc.resolve_shared_path("needles.json")
        self.assertFalse(legacy.exists())
        self.assertTrue(resolved.exists())
        self.assertEqual(resolved.parent.name, mc.GENERAL_DIRNAME)


class TestConfiguredMigrates(_IsolatedRoot):

    def test_setting_an_id_enables_the_real_migration(self):
        legacy = self._legacy("camera_calibrations.json")
        mc.set_machine_id("ME3B_07")

        resolved = mc.resolve_machine_path("camera_calibrations.json")

        self.assertFalse(legacy.exists(), "configured resolution did not migrate")
        self.assertTrue(resolved.exists())
        self.assertEqual(resolved.parent.name, "ME3B_07")
        self.assertEqual(json.loads(resolved.read_text()), {"v": 1})

    def test_migration_is_idempotent(self):
        self._legacy("plate_mosaics.json")
        mc.set_machine_id("ME3B_07")
        first = mc.resolve_machine_path("plate_mosaics.json")
        second = mc.resolve_machine_path("plate_mosaics.json")
        self.assertEqual(first, second)
        self.assertTrue(second.exists())

    def test_an_existing_target_is_never_clobbered(self):
        mc.set_machine_id("ME3B_07")
        target = mc.machine_config_dir() / "objectives.json"
        target.write_text('{"keep": true}', encoding="utf-8")
        self._legacy("objectives.json", '{"stale": true}')

        mc.resolve_machine_path("objectives.json")

        self.assertEqual(json.loads(target.read_text()), {"keep": True})

    def test_env_var_overrides_the_marker_file(self):
        mc.set_machine_id("ME3B_07")
        os.environ["MEBP_MACHINE_ID"] = "ME3B_99"
        mc.reset_cache()
        self.assertEqual(mc.machine_id(), "ME3B_99")

    def test_a_malformed_id_is_refused(self):
        # '.' / '..' satisfy the charset but are path traversal — '..' would
        # put this rig's per-machine files directly into config/.
        for bad in ("", "   ", "a/b", ".", "..", "x\\y", "a b/../c", "C:name"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                mc.set_machine_id(bad)

    def test_reserved_names_are_refused(self):
        """A rig named ME3B_general would write its private calibration into
        the SHARED, TRACKED folder and publish it to every other rig."""
        for bad in ("ME3B_general", "me3b_general", "devices", "DEVICES"):
            with self.subTest(bad=bad), self.assertRaises(ValueError) as ctx:
                mc.set_machine_id(bad)
            self.assertIn("reserved", str(ctx.exception).lower())

    def test_real_profile_names_with_spaces_are_accepted(self):
        # Profiles are really named like "ME3B V1" — refusing spaces would
        # make the operator rename their machine to satisfy the code.
        mc.set_machine_id("ME3B V1 (bench)")
        self.assertEqual(mc.machine_id(), "ME3B V1 (bench)")
        self.assertEqual(mc.machine_config_dir().name, "ME3B V1 (bench)")


class TestIdentityComesFromTheDeviceProfile(_IsolatedRoot):
    """The active device profile NAMES the machine — one home for the fact."""

    def test_identity_follows_the_active_profile(self):
        self.assertFalse(mc.machine_id_is_configured())
        (self._tmp / "settings.json").write_text(
            json.dumps({"device_profile": {"active": "ME3B_02"}}),
            encoding="utf-8")
        mc.reset_cache()
        self.assertTrue(mc.machine_id_is_configured())
        self.assertEqual(mc.machine_id(), "ME3B_02")
        self.assertEqual(mc.machine_config_dir().name, "ME3B_02")

    def test_no_separate_marker_file_is_consulted(self):
        """Guard against a second home for the identity creeping back."""
        self.assertFalse(hasattr(mc, "MACHINE_ID_FILE"))

    def test_set_machine_id_preserves_the_rest_of_settings(self):
        """An errant whole-file rewrite once wiped settings.json (CLAUDE.md)."""
        (self._tmp / "settings.json").write_text(
            json.dumps({"keep_me": {"a": 1}, "device_profile": {"active": "old"}}),
            encoding="utf-8")
        mc.reset_cache()
        mc.set_machine_id("ME3B_09")
        data = json.loads((self._tmp / "settings.json").read_text(encoding="utf-8"))
        self.assertEqual(data["keep_me"], {"a": 1})
        self.assertEqual(data["device_profile"]["active"], "ME3B_09")

    def test_a_profile_name_that_cannot_be_a_folder_falls_back_safely(self):
        (self._tmp / "settings.json").write_text(
            json.dumps({"device_profile": {"active": "ME3B_general"}}),
            encoding="utf-8")
        mc.reset_cache()
        # Reserved → treated as unconfigured, so nothing is moved and the
        # shared folder is never used as a per-machine bucket.
        self.assertFalse(mc.machine_id_is_configured())
        self.assertEqual(mc.machine_id(), mc.UNASSIGNED_ID)

    def test_devices_dir_is_machine_independent(self):
        """Profiles NAME the machine folder, so they cannot live inside it."""
        mc.set_machine_id("ME3B_01")
        self.assertEqual(mc.devices_dir(), mc.HARDWARE_ROOT / "devices")
        self.assertNotIn(mc.machine_config_dir(), mc.devices_dir().parents)


class TestRenameCarriesTheCalibration(_IsolatedRoot):

    def test_rename_moves_the_folder(self):
        mc.set_machine_id("ME3B_01")
        (mc.machine_config_dir() / "objectives.json").write_text("{}", encoding="utf-8")
        self.assertTrue(mc.rename_machine_folder("ME3B_01", "ME3B_02"))
        self.assertFalse((mc.HARDWARE_ROOT / "ME3B_01").exists())
        self.assertTrue((mc.HARDWARE_ROOT / "ME3B_02" / "objectives.json").exists())

    def test_rename_refuses_to_merge_onto_an_existing_machine(self):
        """Both folders hold real calibration; picking one would lose the
        other, so refuse and let the operator decide."""
        mc.set_machine_id("ME3B_01")
        (mc.machine_config_dir() / "a.json").write_text("{}", encoding="utf-8")
        (mc.HARDWARE_ROOT / "ME3B_02").mkdir()
        (mc.HARDWARE_ROOT / "ME3B_02" / "b.json").write_text("{}", encoding="utf-8")
        self.assertFalse(mc.rename_machine_folder("ME3B_01", "ME3B_02"))
        self.assertTrue((mc.HARDWARE_ROOT / "ME3B_01" / "a.json").exists())
        self.assertTrue((mc.HARDWARE_ROOT / "ME3B_02" / "b.json").exists())

    def test_rename_to_the_same_name_is_a_no_op(self):
        mc.set_machine_id("ME3B_01")
        mc.machine_config_dir()
        self.assertFalse(mc.rename_machine_folder("ME3B_01", "ME3B_01"))


# ════════════════════════════════════════════════════════════════════
#  B. The sweep must never publish a rig's private data
# ════════════════════════════════════════════════════════════════════

class TestSweepSafety(_IsolatedRoot):

    def setUp(self):
        super().setUp()
        mc.set_machine_id("ME3B_07")

    def test_a_setup_file_is_swept_into_shared(self):
        self._legacy("Alexs Setup.json")
        moved = mc.sweep_remaining_flat_files()
        self.assertEqual([p.name for p in moved], ["Alexs Setup.json"])
        self.assertEqual(moved[0].parent.name, mc.GENERAL_DIRNAME)

    def test_an_unclaimed_per_machine_file_is_never_swept(self):
        """The store failed to import, so nobody claimed it. Filing it as
        shared would commit THIS rig's calibration and push it to every
        other rig — the exact failure this module exists to prevent."""
        legacy = self._legacy("camera_calibrations.json")

        moved = mc.sweep_remaining_flat_files()

        self.assertEqual(moved, [])
        self.assertTrue(legacy.exists())
        self.assertFalse((mc.shared_config_dir() / "camera_calibrations.json").exists())

    def test_allow_sweep_false_moves_nothing(self):
        legacy = self._legacy("Some Setup.json")
        moved = mc.sweep_remaining_flat_files(allow_sweep=False)
        self.assertEqual(moved, [])
        self.assertTrue(legacy.exists())

    def test_every_per_machine_store_filename_is_on_the_denylist(self):
        """Guard the guard: the denylist is only as good as its coverage."""
        for name in ("camera_calibrations.json", "objectives.json",
                     "plate_mosaics.json", "last_calibration.json",
                     "calibration_status.json", "microscope.json",
                     "plate_templates.json", "lablink.json"):
            self.assertIn(name, mc.PER_MACHINE_FILENAMES)


# ════════════════════════════════════════════════════════════════════
#  C. main.py import ordering (structural — a tidy-up would undo it)
# ════════════════════════════════════════════════════════════════════

class TestMainImportsGuiAppAfterThePrompt(unittest.TestCase):

    def _run_gui_body(self):
        tree = ast.parse((REPO / "main.py").read_text(encoding="utf-8"))
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == "run_gui":
                return node
        self.fail("run_gui not found in main.py")

    def test_gui_app_is_imported_after_ensure_machine_id(self):
        fn = self._run_gui_body()
        prompt_lines, gui_app_lines = [], []
        for node in ast.walk(fn):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "_ensure_machine_id"):
                prompt_lines.append(node.lineno)
            if isinstance(node, ast.ImportFrom) and node.module == "gui.app":
                gui_app_lines.append(node.lineno)

        self.assertTrue(prompt_lines, "run_gui never calls _ensure_machine_id")
        self.assertTrue(gui_app_lines, "run_gui never imports gui.app")
        # ⚠ EARLIEST import vs LATEST prompt. Taking the last gui.app import
        # would let an ADDED early one hide behind the correct later one —
        # mutation-verified: that version of this test survived exactly that.
        self.assertLess(
            max(prompt_lines), min(gui_app_lines),
            "main.py imports gui.app BEFORE asking for the machine id. "
            "gui.app pulls in per-machine stores that resolve their config "
            "path at import time, so a fresh rig would use the wrong bucket "
            "for the whole session.")

    def test_stores_really_do_resolve_their_path_at_import_time(self):
        """Guard the guard: the ordering test above only matters because
        these resolve at MODULE level (not lazily inside a function), so the
        path is frozen the moment the module is imported."""
        module_level_resolvers = []
        for name in ("ObjectiveCalibration", "NeedleBoreCalibrationStore",
                     "FluorescenceMosaicStore", "PrintTimingCalibrationStore"):
            tree = ast.parse(
                (REPO / "SupportClasses" / f"{name}.py").read_text(encoding="utf-8"))
            for node in tree.body:            # module level ONLY, not ast.walk
                if not isinstance(node, ast.Assign):
                    continue
                for call in ast.walk(node):
                    if (isinstance(call, ast.Call)
                            and isinstance(call.func, ast.Name)
                            and call.func.id == "resolve_machine_path"):
                        module_level_resolvers.append(name)
        self.assertGreaterEqual(
            len(set(module_level_resolvers)), 3,
            "expected several stores to resolve a per-machine path at import "
            "time; if that is no longer true the main.py ordering guard above "
            "may be relaxed — but only as a deliberate, observed change")


if __name__ == "__main__":
    unittest.main()
