"""
test_session4_print_files.py — Tests for Session 4: Print File Infrastructure + Auto-Layout.

Tests: 69
Coverage: PrintFileManager CRUD, schema validation, migration, auto-layout generators,
          sanitize_filename, print_objects_additions structure verification.
"""

import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path

# Import the modules under test
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from SupportClasses.PrintFileManager import (
    PrintFileMetadata, PrintFileData, PrintFileManager,
    validate_print_file, migrate_print_file, _sanitize_filename,
    SCHEMA_VERSION,
)
from SupportClasses.auto_layout import (
    generate_ring, generate_square_grid, generate_hex_grid,
    generate_line, generate_concentric_rings,
    generate_layout, count_layout_objects, validate_layout_in_well,
    LAYOUT_INFO, LayoutPattern, GENERATORS,
)


# ═══════════════════════════════════════════════════════════════════
# PrintFileMetadata Tests
# ═══════════════════════════════════════════════════════════════════

class TestPrintFileMetadata(unittest.TestCase):
    """Test PrintFileMetadata dataclass."""

    def test_defaults(self):
        meta = PrintFileMetadata()
        self.assertEqual(meta.name, "Untitled")
        self.assertTrue(len(meta.created) > 0)
        self.assertTrue(len(meta.modified) > 0)

    def test_roundtrip(self):
        meta = PrintFileMetadata(name="Test", description="A test print")
        d = meta.to_dict()
        restored = PrintFileMetadata.from_dict(d)
        self.assertEqual(restored.name, "Test")
        self.assertEqual(restored.description, "A test print")

    def test_custom_values(self):
        meta = PrintFileMetadata(
            name="Custom", description="desc",
            created="2026-01-01", modified="2026-03-01", author="Alex"
        )
        self.assertEqual(meta.author, "Alex")
        self.assertEqual(meta.created, "2026-01-01")


# ═══════════════════════════════════════════════════════════════════
# PrintFileData Tests
# ═══════════════════════════════════════════════════════════════════

class TestPrintFileData(unittest.TestCase):
    """Test PrintFileData dataclass."""

    def test_defaults(self):
        pf = PrintFileData()
        self.assertEqual(pf.schema_version, SCHEMA_VERSION)
        self.assertEqual(len(pf.objects), 0)
        self.assertFalse(pf.is_dirty)

    def test_dirty_tracking(self):
        pf = PrintFileData()
        self.assertFalse(pf.is_dirty)
        pf.mark_dirty()
        self.assertTrue(pf.is_dirty)
        pf.mark_clean()
        self.assertFalse(pf.is_dirty)

    def test_name_property(self):
        pf = PrintFileData()
        pf.name = "New Name"
        self.assertEqual(pf.name, "New Name")
        self.assertTrue(pf.is_dirty)

    def test_roundtrip(self):
        pf = PrintFileData()
        pf.name = "Test"
        pf.objects["Dot_1"] = {
            "object_type": "dot", "params": {"radius": 0.5},
            "position": [0, 0, 0], "color": "#ff0000", "ink_pump": "P1",
        }
        d = pf.to_dict()
        restored = PrintFileData.from_dict(d)
        self.assertEqual(restored.name, "Test")
        self.assertIn("Dot_1", restored.objects)


# ═══════════════════════════════════════════════════════════════════
# Validation Tests
# ═══════════════════════════════════════════════════════════════════

class TestValidation(unittest.TestCase):
    """Test print file schema validation."""

    def _make_valid(self) -> dict:
        return {
            "schema_version": "7.2.3",
            "metadata": {"name": "Test", "description": "", "created": "", "modified": "", "author": ""},
            "objects": {},
            "collections": {},
            "layout_presets": {},
        }

    def test_valid_file(self):
        valid, errors = validate_print_file(self._make_valid())
        self.assertTrue(valid)
        self.assertEqual(len(errors), 0)

    def test_not_a_dict(self):
        valid, errors = validate_print_file("not a dict")
        self.assertFalse(valid)

    def test_missing_schema_version(self):
        d = self._make_valid()
        del d["schema_version"]
        valid, errors = validate_print_file(d)
        self.assertFalse(valid)

    def test_missing_metadata(self):
        d = self._make_valid()
        del d["metadata"]
        valid, errors = validate_print_file(d)
        self.assertFalse(valid)

    def test_missing_metadata_name(self):
        d = self._make_valid()
        d["metadata"]["name"] = ""
        valid, errors = validate_print_file(d)
        self.assertFalse(valid)

    def test_invalid_objects_type(self):
        d = self._make_valid()
        d["objects"] = "not a dict"
        valid, errors = validate_print_file(d)
        self.assertFalse(valid)

    def test_object_missing_type(self):
        d = self._make_valid()
        d["objects"] = {"Obj_1": {"params": {}}}
        valid, errors = validate_print_file(d)
        self.assertFalse(valid)

    def test_object_missing_params(self):
        d = self._make_valid()
        d["objects"] = {"Obj_1": {"object_type": "dot"}}
        valid, errors = validate_print_file(d)
        self.assertFalse(valid)


# ═══════════════════════════════════════════════════════════════════
# Migration Tests
# ═══════════════════════════════════════════════════════════════════

class TestMigration(unittest.TestCase):
    """Test forward migration from older schemas."""

    def test_v71_migration(self):
        old = {
            "version": "7.1",
            "print_name": "Old Print",
            "description": "Legacy",
            "objects": [
                {"name": "Dot_1", "object_type": "dot", "params": {"radius": 1.0}}
            ],
        }
        migrated = migrate_print_file(old)
        self.assertEqual(migrated["schema_version"], SCHEMA_VERSION)
        self.assertIn("metadata", migrated)
        self.assertEqual(migrated["metadata"]["name"], "Old Print")
        self.assertIn("Dot_1", migrated["objects"])
        self.assertIn("layout_presets", migrated)

    def test_v72_migration(self):
        old = {
            "schema_version": "7.2",
            "metadata": {"name": "V72 Print"},
            "objects": {},
        }
        migrated = migrate_print_file(old)
        self.assertEqual(migrated["schema_version"], SCHEMA_VERSION)
        self.assertIn("layout_presets", migrated)

    def test_current_version_passthrough(self):
        current = {
            "schema_version": "7.2.3",
            "metadata": {"name": "Current"},
            "objects": {}, "collections": {}, "layout_presets": {},
        }
        migrated = migrate_print_file(current)
        self.assertEqual(migrated["schema_version"], "7.2.3")


# ═══════════════════════════════════════════════════════════════════
# PrintFileManager Tests
# ═══════════════════════════════════════════════════════════════════

class TestPrintFileManager(unittest.TestCase):
    """Test PrintFileManager CRUD operations."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.mgr = PrintFileManager(prints_dir=self.tmpdir)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_new_file(self):
        pf = self.mgr.new_file("Test Print")
        self.assertIsNotNone(pf)
        self.assertEqual(pf.name, "Test Print")
        self.assertIsNotNone(self.mgr.current_path)
        self.assertTrue(self.mgr.current_path.exists())

    def test_new_file_auto_increment(self):
        self.mgr.new_file("Scaffold")
        pf2 = self.mgr.new_file("Scaffold")
        self.assertEqual(pf2.name, "Scaffold_2")

    def test_save_and_load(self):
        self.mgr.new_file("SaveTest")
        self.mgr.add_object("Dot_1", {
            "object_type": "dot", "params": {"radius": 1.0},
            "position": [0, 0, 0], "color": "#a6e3a1", "ink_pump": "P1",
        })
        self.mgr.save()

        mgr2 = PrintFileManager(prints_dir=self.tmpdir)
        loaded = mgr2.load("SaveTest")
        self.assertIsNotNone(loaded)
        self.assertIn("Dot_1", loaded.objects)

    def test_delete(self):
        self.mgr.new_file("ToDelete")
        path = self.mgr.current_path
        self.assertTrue(path.exists())
        result = self.mgr.delete("ToDelete")
        self.assertTrue(result)
        self.assertFalse(path.exists())
        self.assertIsNone(self.mgr.current)

    def test_delete_nonexistent(self):
        result = self.mgr.delete("DoesNotExist")
        self.assertFalse(result)

    def test_duplicate(self):
        self.mgr.new_file("Original")
        self.mgr.add_object("Ring_1", {
            "object_type": "circle", "params": {"radius": 2.0},
            "position": [0, 0, 0], "color": "#89b4fa", "ink_pump": "P1",
        })
        self.mgr.save()
        dup = self.mgr.duplicate("Copy of Original")
        self.assertIsNotNone(dup)
        self.assertEqual(dup.name, "Copy of Original")
        self.assertIn("Ring_1", dup.objects)

    def test_list_files(self):
        self.mgr.new_file("File_A")
        self.mgr.new_file("File_B")
        files = self.mgr.list_files()
        names = [f["name"] for f in files]
        self.assertIn("File_A", names)
        self.assertIn("File_B", names)

    def test_add_object(self):
        self.mgr.new_file("ObjTest")
        name = self.mgr.add_object("Dot_1", {"object_type": "dot", "params": {}})
        self.assertEqual(name, "Dot_1")
        self.assertTrue(self.mgr.current.is_dirty)

    def test_add_object_auto_increment(self):
        self.mgr.new_file("ObjTest")
        self.mgr.add_object("Dot_1", {"object_type": "dot", "params": {}})
        name2 = self.mgr.add_object("Dot_1", {"object_type": "dot", "params": {}})
        self.assertEqual(name2, "Dot_1_2")

    def test_remove_object(self):
        self.mgr.new_file("RemoveTest")
        self.mgr.add_object("Dot_1", {"object_type": "dot", "params": {}})
        self.assertTrue(self.mgr.remove_object("Dot_1"))
        self.assertNotIn("Dot_1", self.mgr.current.objects)

    def test_remove_object_clears_from_collections(self):
        self.mgr.new_file("CollTest")
        self.mgr.add_object("Dot_1", {"object_type": "dot", "params": {}})
        self.mgr.add_collection("Group_1", [{"object_name": "Dot_1", "position": [0, 0, 0]}])
        self.mgr.remove_object("Dot_1")
        items = self.mgr.current.collections["Group_1"]
        self.assertEqual(len(items), 0)

    def test_update_object(self):
        self.mgr.new_file("UpdateTest")
        self.mgr.add_object("Dot_1", {"object_type": "dot", "params": {"r": 1}})
        self.mgr.current.mark_clean()
        self.mgr.update_object("Dot_1", {"object_type": "dot", "params": {"r": 2}})
        self.assertEqual(self.mgr.current.objects["Dot_1"]["params"]["r"], 2)
        self.assertTrue(self.mgr.current.is_dirty)

    def test_add_collection(self):
        self.mgr.new_file("CollTest")
        name = self.mgr.add_collection("Group_1")
        self.assertEqual(name, "Group_1")
        self.assertIn("Group_1", self.mgr.current.collections)

    def test_remove_collection(self):
        self.mgr.new_file("CollTest")
        self.mgr.add_collection("Group_1")
        self.assertTrue(self.mgr.remove_collection("Group_1"))
        self.assertNotIn("Group_1", self.mgr.current.collections)

    def test_layout_preset_crud(self):
        self.mgr.new_file("PresetTest")
        name = self.mgr.add_layout_preset("Ring_6", "ring", {"n": 6, "radius": 2.0})
        self.assertEqual(name, "Ring_6")
        self.assertIn("Ring_6", self.mgr.current.layout_presets)
        self.assertTrue(self.mgr.remove_layout_preset("Ring_6"))
        self.assertNotIn("Ring_6", self.mgr.current.layout_presets)

    def test_auto_save_when_dirty(self):
        self.mgr.new_file("AutoTest")
        self.mgr.add_object("X", {"object_type": "dot", "params": {}})
        self.mgr._last_save_time = 0  # Force enough time elapsed
        result = self.mgr.auto_save(interval_s=0)
        self.assertTrue(result)
        self.assertFalse(self.mgr.current.is_dirty)

    def test_auto_save_skips_when_clean(self):
        self.mgr.new_file("AutoTest")
        self.mgr.current.mark_clean()
        result = self.mgr.auto_save(interval_s=0)
        self.assertFalse(result)

    def test_save_as(self):
        self.mgr.new_file("Original")
        self.mgr.save_as("Renamed")
        self.assertEqual(self.mgr.current.name, "Renamed")


# ═══════════════════════════════════════════════════════════════════
# Sanitize Filename Tests
# ═══════════════════════════════════════════════════════════════════

class TestSanitizeFilename(unittest.TestCase):
    """Test _sanitize_filename edge cases."""

    def test_normal_name(self):
        self.assertEqual(_sanitize_filename("My Print"), "My_Print")

    def test_special_chars(self):
        result = _sanitize_filename("Print <v1> (test)")
        self.assertNotIn("<", result)
        self.assertNotIn(">", result)

    def test_empty_string(self):
        self.assertEqual(_sanitize_filename(""), "Untitled")

    def test_whitespace_only(self):
        self.assertEqual(_sanitize_filename("   "), "Untitled")

    def test_long_name_truncated(self):
        long_name = "A" * 100
        result = _sanitize_filename(long_name)
        self.assertLessEqual(len(result), 60)


# ═══════════════════════════════════════════════════════════════════
# Auto-Layout Integration Tests
# ═══════════════════════════════════════════════════════════════════

class TestAutoLayoutIntegration(unittest.TestCase):
    """Test auto_layout generators and validation."""

    def test_ring_count(self):
        positions = generate_ring(n=6, radius=2.0)
        self.assertEqual(len(positions), 6)

    def test_ring_radius(self):
        positions = generate_ring(n=4, radius=3.0)
        for x, y in positions:
            dist = math.sqrt(x**2 + y**2)
            self.assertAlmostEqual(dist, 3.0, places=3)

    def test_ring_single_object(self):
        positions = generate_ring(n=1, radius=2.0, center=(1.0, 1.0))
        self.assertEqual(positions, [(1.0, 1.0)])

    def test_square_grid_count(self):
        positions = generate_square_grid(rows=3, cols=4, spacing=1.0)
        self.assertEqual(len(positions), 12)

    def test_square_grid_centered(self):
        positions = generate_square_grid(rows=3, cols=3, spacing=2.0)
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        self.assertAlmostEqual(sum(xs) / len(xs), 0.0, places=3)
        self.assertAlmostEqual(sum(ys) / len(ys), 0.0, places=3)

    def test_hex_grid_count(self):
        positions = generate_hex_grid(rows=3, cols=3, spacing=1.0)
        self.assertEqual(len(positions), 9)

    def test_line_count(self):
        positions = generate_line(n=5)
        self.assertEqual(len(positions), 5)

    def test_line_endpoints(self):
        positions = generate_line(n=3, start_x=-1, start_y=0, end_x=1, end_y=0)
        self.assertAlmostEqual(positions[0][0], -1.0)
        self.assertAlmostEqual(positions[-1][0], 1.0)

    def test_concentric_rings_count(self):
        positions = generate_concentric_rings(ring_count=2, objects_per_ring=4)
        self.assertEqual(len(positions), 8)

    def test_generate_layout_dispatch(self):
        positions = generate_layout("ring", n=4, radius=1.0)
        self.assertEqual(len(positions), 4)

    def test_count_layout_objects(self):
        self.assertEqual(count_layout_objects("ring", n=6), 6)
        self.assertEqual(count_layout_objects("square_grid", rows=3, cols=3), 9)
        self.assertEqual(count_layout_objects("hex_grid", rows=2, cols=4), 8)
        self.assertEqual(count_layout_objects("line", n=5), 5)
        self.assertEqual(count_layout_objects("concentric_rings", ring_count=3, objects_per_ring=4), 12)

    def test_validate_layout_in_well(self):
        # Small ring should fit in large well
        positions = generate_ring(n=6, radius=1.0)
        all_fit, warnings = validate_layout_in_well(positions, well_diameter_mm=10.0)
        self.assertTrue(all_fit)
        self.assertEqual(len(warnings), 0)

    def test_validate_layout_out_of_well(self):
        # Large ring should NOT fit in small well
        positions = generate_ring(n=6, radius=5.0)
        all_fit, warnings = validate_layout_in_well(positions, well_diameter_mm=2.0)
        self.assertFalse(all_fit)
        self.assertGreater(len(warnings), 0)


# ═══════════════════════════════════════════════════════════════════
# Layout Info Structure Tests
# ═══════════════════════════════════════════════════════════════════

class TestLayoutInfo(unittest.TestCase):
    """Test LAYOUT_INFO dict completeness."""

    def test_all_patterns_have_info(self):
        for pattern in GENERATORS:
            self.assertIn(pattern, LAYOUT_INFO, f"Missing LAYOUT_INFO for '{pattern}'")

    def test_all_info_has_required_fields(self):
        for pattern, info in LAYOUT_INFO.items():
            self.assertIn("label", info, f"'{pattern}' missing label")
            self.assertIn("icon", info, f"'{pattern}' missing icon")
            self.assertIn("params", info, f"'{pattern}' missing params")

    def test_params_have_required_fields(self):
        for pattern, info in LAYOUT_INFO.items():
            for param_name, param_info in info["params"].items():
                self.assertIn("label", param_info, f"'{pattern}.{param_name}' missing label")
                self.assertIn("type", param_info, f"'{pattern}.{param_name}' missing type")
                self.assertIn("default", param_info, f"'{pattern}.{param_name}' missing default")


# ═══════════════════════════════════════════════════════════════════
# Edge Cases
# ═══════════════════════════════════════════════════════════════════

class TestEdgeCases(unittest.TestCase):
    """Edge case and error handling tests."""

    def test_ring_zero_objects(self):
        self.assertEqual(generate_ring(n=0), [])

    def test_line_one_object(self):
        positions = generate_line(n=1)
        self.assertEqual(len(positions), 1)

    def test_invalid_pattern_raises(self):
        with self.assertRaises(ValueError):
            generate_layout("nonexistent_pattern")

    def test_invalid_count_pattern_raises(self):
        with self.assertRaises(ValueError):
            count_layout_objects("nonexistent_pattern")

    def test_concentric_rings_zero(self):
        self.assertEqual(generate_concentric_rings(ring_count=0), [])


if __name__ == "__main__":
    unittest.main()
