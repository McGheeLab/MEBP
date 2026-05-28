"""
test_v75_migration.py — Migration from v7.2.3 print files to v7.5.0.

Verifies that migrate_print_file() correctly:
  - bumps schema_version to 7.5.0
  - tags parametric objects with source="parametric" and strips
    their trajectory
  - keeps source="csv" + trajectory on csv-imported objects
  - maps use_waste/use_wash/use_buffer → ink_swap.*
  - renames max_ink_volume_uL → pump_volume_overrides_uL
  - writes a one-time .bak-v7.2.3 backup on PrintFileManager.load()
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from SupportClasses.PrintFileManager import (
    PrintFileManager, SCHEMA_VERSION, migrate_print_file,
    write_migration_backup,
)


V723_FIXTURE = {
    "schema_version": "7.2.3",
    "metadata": {"name": "Old Print", "description": ""},
    "objects": {
        "ring": {
            "name": "ring",
            "object_type": "circle",
            "params": {"radius": 1.0},
            "trajectory": [[0, 0, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 1]],
        },
        "imported_path": {
            "name": "imported_path",
            "object_type": "csv_import",
            "params": {"source_file": "demo.csv"},
            "trajectory": [[0, 0, 0, 0, 0, 0, 0]],
        },
    },
    "collections": {},
    "layout_presets": {},
    "execution_config": {
        "use_waste": True,
        "use_wash": False,
        "use_buffer": True,
        "max_ink_volume_uL": {"P1": 250.0},
    },
}


class TestMigrateFunction(unittest.TestCase):

    def test_schema_version_bumps(self) -> None:
        mig = migrate_print_file(V723_FIXTURE)
        self.assertEqual(mig["schema_version"], SCHEMA_VERSION)
        self.assertEqual(SCHEMA_VERSION, "7.5.0")

    def test_parametric_strips_trajectory(self) -> None:
        mig = migrate_print_file(V723_FIXTURE)
        ring = mig["objects"]["ring"]
        self.assertEqual(ring["source"], "parametric")
        self.assertNotIn("trajectory", ring)

    def test_csv_keeps_trajectory(self) -> None:
        mig = migrate_print_file(V723_FIXTURE)
        csv = mig["objects"]["imported_path"]
        self.assertEqual(csv["source"], "csv")
        self.assertIn("trajectory", csv)

    def test_execution_config_flags_remap(self) -> None:
        mig = migrate_print_file(V723_FIXTURE)
        ec = mig["execution_config"]
        self.assertNotIn("use_waste", ec)
        self.assertNotIn("use_wash", ec)
        self.assertNotIn("use_buffer", ec)
        self.assertEqual(ec["ink_swap"]["waste"], True)
        self.assertEqual(ec["ink_swap"]["wash_pre"], False)
        self.assertEqual(ec["ink_swap"]["wash_post"], False)
        self.assertEqual(ec["ink_swap"]["buffer"], True)

    def test_max_ink_volume_renamed(self) -> None:
        mig = migrate_print_file(V723_FIXTURE)
        ec = mig["execution_config"]
        self.assertNotIn("max_ink_volume_uL", ec)
        self.assertEqual(ec["pump_volume_overrides_uL"], {"P1": 250.0})

    def test_idempotent_when_already_at_current(self) -> None:
        mig1 = migrate_print_file(V723_FIXTURE)
        mig2 = migrate_print_file(mig1)
        self.assertEqual(mig1, mig2)

    def test_v71_listobjects_dict_conversion(self) -> None:
        v71 = {
            "version": "7.1",
            "print_name": "Legacy",
            "objects": [{"name": "A", "object_type": "circle", "params": {}}],
        }
        mig = migrate_print_file(v71)
        self.assertEqual(mig["schema_version"], SCHEMA_VERSION)
        self.assertIn("A", mig["objects"])


class TestMigrationBackup(unittest.TestCase):

    def test_load_writes_bak_v723_once(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            prints_dir = Path(tmp)
            target = prints_dir / "OldPrint.json"
            with open(target, "w") as f:
                json.dump(V723_FIXTURE, f)

            mgr = PrintFileManager(prints_dir=prints_dir)
            loaded = mgr.load("Old Print")
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.schema_version, SCHEMA_VERSION)

            backup = target.with_suffix(target.suffix + ".bak-v7.2.3")
            self.assertTrue(backup.exists())
            with open(backup) as f:
                backup_contents = json.load(f)
            self.assertEqual(backup_contents["schema_version"], "7.2.3")

    def test_backup_helper_skips_existing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "Old.json"
            with open(target, "w") as f:
                json.dump(V723_FIXTURE, f)
            backup = target.with_suffix(target.suffix + ".bak-v7.2.3")
            with open(backup, "w") as f:
                f.write('{"already": "there"}')
            result = write_migration_backup(target, V723_FIXTURE)
            self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
