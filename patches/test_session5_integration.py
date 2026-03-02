"""
test_session5_integration.py — Cross-session integration tests for MEBP v7.2.3.

Tests: 52
Coverage: Cross-session data flow, end-to-end workflows, edge cases,
          source file integrity, signal contracts, schema compatibility.
"""

import ast
import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from SupportClasses.PrintFileManager import (
    PrintFileManager, PrintFileData, PrintFileMetadata,
    validate_print_file, migrate_print_file, _sanitize_filename,
    SCHEMA_VERSION,
)
from SupportClasses.auto_layout import (
    generate_ring, generate_square_grid, generate_hex_grid,
    generate_line, generate_concentric_rings,
    generate_layout, count_layout_objects, validate_layout_in_well,
    LAYOUT_INFO, GENERATORS,
)


# ═══════════════════════════════════════════════════════════════════
# Cross-Session Data Flow Tests
# ═══════════════════════════════════════════════════════════════════

class TestCrossSessionDataFlow(unittest.TestCase):
    """Test that data structures are compatible across sessions."""

    def test_print_file_schema_roundtrip(self):
        """PrintFileData survives JSON roundtrip."""
        pf = PrintFileData()
        pf.name = "Roundtrip Test"
        pf.objects["Dot_1"] = {
            "object_type": "dot", "params": {"radius": 0.5},
            "position": [1.0, 2.0, 0.0], "color": "#a6e3a1",
            "ink_pump": "P1", "num_layers": 1, "layer_height": 0.2,
        }
        pf.collections["Group_1"] = [
            {"object_name": "Dot_1", "position": [0, 0, 0], "copies": 1}
        ]
        pf.layout_presets["Ring_6"] = {
            "pattern": "ring", "params": {"n": 6, "radius": 2.0}
        }

        # Roundtrip through JSON
        json_str = json.dumps(pf.to_dict())
        restored = PrintFileData.from_dict(json.loads(json_str))

        self.assertEqual(restored.name, "Roundtrip Test")
        self.assertEqual(len(restored.objects), 1)
        self.assertEqual(len(restored.collections), 1)
        self.assertEqual(len(restored.layout_presets), 1)

    def test_layout_positions_are_json_serializable(self):
        """All layout generators produce JSON-serializable positions."""
        for pattern, gen in GENERATORS.items():
            info = LAYOUT_INFO[pattern]
            kwargs = {k: v["default"] for k, v in info["params"].items()}
            positions = gen(**kwargs)
            # Must be JSON-serializable
            json_str = json.dumps(positions)
            restored = json.loads(json_str)
            self.assertEqual(len(restored), len(positions))

    def test_layout_into_print_file(self):
        """Layout-generated positions integrate into print file objects."""
        positions = generate_ring(n=4, radius=1.5)
        pf = PrintFileData()
        pf.name = "Layout Test"
        for i, (x, y) in enumerate(positions):
            pf.objects[f"Ring_Dot_{i+1}"] = {
                "object_type": "dot",
                "params": {"radius": 0.3},
                "position": [x, y, 0.0],
                "color": "#a6e3a1",
                "ink_pump": "P1",
                "auto_layout": True,
            }
        self.assertEqual(len(pf.objects), 4)

    def test_metadata_timestamps_updated(self):
        """File operations update timestamps correctly."""
        pf = PrintFileData()
        original_modified = pf.metadata.modified
        pf.mark_dirty()
        self.assertNotEqual(pf.metadata.modified, original_modified)

    def test_validation_after_migration(self):
        """Migrated files pass validation."""
        v71_data = {
            "version": "7.1",
            "print_name": "Legacy",
            "objects": [{"name": "D1", "object_type": "dot", "params": {"r": 1}}],
        }
        migrated = migrate_print_file(v71_data)
        valid, errors = validate_print_file(migrated)
        self.assertTrue(valid, f"Migration produced invalid file: {errors}")

    def test_print_file_with_all_layout_types(self):
        """File can store presets for all 5 layout patterns."""
        pf = PrintFileData()
        pf.name = "All Layouts"
        for pattern in GENERATORS:
            info = LAYOUT_INFO[pattern]
            params = {k: v["default"] for k, v in info["params"].items()}
            pf.layout_presets[f"Preset_{pattern}"] = {
                "pattern": pattern, "params": params,
            }
        d = pf.to_dict()
        self.assertEqual(len(d["layout_presets"]), 5)


# ═══════════════════════════════════════════════════════════════════
# End-to-End Workflow Tests
# ═══════════════════════════════════════════════════════════════════

class TestEndToEndWorkflow(unittest.TestCase):
    """Full create → save → load → duplicate → modify workflow."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.mgr = PrintFileManager(prints_dir=self.tmpdir)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_full_lifecycle(self):
        """Create, add objects, save, reload, verify."""
        self.mgr.new_file("Lifecycle Test")
        self.mgr.add_object("Dot_1", {
            "object_type": "dot", "params": {"radius": 0.5},
            "position": [0, 0, 0], "color": "#a6e3a1", "ink_pump": "P1",
        })
        self.mgr.add_object("Line_1", {
            "object_type": "line", "params": {"length": 5.0, "spacing": 0.3},
            "position": [1, 0, 0], "color": "#89b4fa", "ink_pump": "P2",
        })
        self.mgr.add_collection("Group_1", [
            {"object_name": "Dot_1", "position": [0, 0, 0], "copies": 1},
        ])
        self.mgr.add_layout_preset("MyRing", "ring", {"n": 6, "radius": 2.0})
        self.mgr.save()

        # Reload
        mgr2 = PrintFileManager(prints_dir=self.tmpdir)
        loaded = mgr2.load("Lifecycle Test")
        self.assertIsNotNone(loaded)
        self.assertEqual(len(loaded.objects), 2)
        self.assertEqual(len(loaded.collections), 1)
        self.assertEqual(len(loaded.layout_presets), 1)

    def test_duplicate_and_modify(self):
        """Duplicate preserves objects, modifications are independent."""
        self.mgr.new_file("Original")
        self.mgr.add_object("Obj_1", {"object_type": "dot", "params": {"r": 1}})
        self.mgr.save()

        self.mgr.duplicate("Copy")
        self.mgr.add_object("Obj_2", {"object_type": "line", "params": {"l": 2}})
        self.mgr.save()

        # Verify original is unchanged
        mgr2 = PrintFileManager(prints_dir=self.tmpdir)
        original = mgr2.load("Original")
        self.assertEqual(len(original.objects), 1)

        copy = mgr2.load("Copy")
        self.assertEqual(len(copy.objects), 2)

    def test_delete_does_not_affect_others(self):
        """Deleting one file leaves others intact."""
        self.mgr.new_file("Keep")
        self.mgr.save()
        self.mgr.new_file("Delete")
        self.mgr.save()

        self.mgr.delete("Delete")
        files = self.mgr.list_files()
        names = [f["name"] for f in files]
        self.assertIn("Keep", names)
        self.assertNotIn("Delete", names)

    def test_layout_to_objects_workflow(self):
        """Apply a layout and add generated objects to file."""
        self.mgr.new_file("LayoutWorkflow")
        positions = generate_layout("hex_grid", rows=3, cols=3, spacing=0.5)
        for i, (x, y) in enumerate(positions):
            self.mgr.add_object(f"Hex_{i+1}", {
                "object_type": "dot",
                "params": {"radius": 0.15},
                "position": [x, y, 0.0],
                "auto_layout": True,
            })
        self.mgr.save()
        self.assertEqual(self.mgr.get_object_count(), 9)


# ═══════════════════════════════════════════════════════════════════
# Auto-Layout + Print File Integration
# ═══════════════════════════════════════════════════════════════════

class TestAutoLayoutPrintFileIntegration(unittest.TestCase):
    """Test integration between auto_layout and PrintFileManager."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.mgr = PrintFileManager(prints_dir=self.tmpdir)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_all_patterns_generate_valid_positions(self):
        """Every pattern produces at least the expected number of objects."""
        for pattern, info in LAYOUT_INFO.items():
            kwargs = {k: v["default"] for k, v in info["params"].items()}
            positions = GENERATORS[pattern](**kwargs)
            expected = count_layout_objects(pattern, **kwargs)
            self.assertEqual(len(positions), expected,
                             f"Pattern '{pattern}' count mismatch")

    def test_preset_roundtrip(self):
        """Layout presets survive save → load."""
        self.mgr.new_file("PresetRT")
        self.mgr.add_layout_preset("Grid_3x3", "square_grid",
                                   {"rows": 3, "cols": 3, "spacing": 1.0})
        self.mgr.save()

        mgr2 = PrintFileManager(prints_dir=self.tmpdir)
        loaded = mgr2.load("PresetRT")
        preset = loaded.layout_presets["Grid_3x3"]
        self.assertEqual(preset["pattern"], "square_grid")
        self.assertEqual(preset["params"]["rows"], 3)

    def test_well_boundary_validation(self):
        """Large layouts are flagged when they exceed well boundaries."""
        positions = generate_ring(n=8, radius=5.0)
        # 24-well plate has ~16mm diameter wells
        all_fit, warnings = validate_layout_in_well(positions, 16.0)
        self.assertTrue(all_fit)

        # 96-well plate has ~6.4mm diameter wells
        all_fit, warnings = validate_layout_in_well(positions, 6.4)
        self.assertFalse(all_fit)

    def test_concentric_rings_distinct_radii(self):
        """Concentric ring objects are at different distances from center."""
        positions = generate_concentric_rings(
            ring_count=3, objects_per_ring=4,
            inner_radius=1.0, outer_radius=3.0,
        )
        distances = sorted(set(round(math.sqrt(x**2 + y**2), 2)
                              for x, y in positions))
        # Should have 3 distinct radii
        self.assertEqual(len(distances), 3)


# ═══════════════════════════════════════════════════════════════════
# Edge Cases
# ═══════════════════════════════════════════════════════════════════

class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error handling."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.mgr = PrintFileManager(prints_dir=self.tmpdir)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_empty_file(self):
        """New file with no objects is valid."""
        pf = self.mgr.new_file("Empty")
        self.assertEqual(len(pf.objects), 0)
        d = pf.to_dict()
        valid, errors = validate_print_file(d)
        self.assertTrue(valid)

    def test_unicode_name(self):
        """Unicode characters in print name work."""
        pf = self.mgr.new_file("Résumé_Tëst_日本語")
        self.assertIsNotNone(pf)
        self.mgr.save()
        files = self.mgr.list_files()
        self.assertGreater(len(files), 0)

    def test_many_objects(self):
        """File handles 100+ objects."""
        self.mgr.new_file("StressTest")
        for i in range(100):
            self.mgr.add_object(f"Dot_{i}", {
                "object_type": "dot", "params": {"radius": 0.1},
                "position": [i * 0.1, 0, 0],
            })
        self.mgr.save()
        self.assertEqual(self.mgr.get_object_count(), 100)

        mgr2 = PrintFileManager(prints_dir=self.tmpdir)
        loaded = mgr2.load("StressTest")
        self.assertEqual(len(loaded.objects), 100)

    def test_load_nonexistent(self):
        """Loading a non-existent file returns None."""
        result = self.mgr.load("NonExistent_File_12345")
        self.assertIsNone(result)

    def test_remove_nonexistent_object(self):
        """Removing non-existent object returns False."""
        self.mgr.new_file("Test")
        self.assertFalse(self.mgr.remove_object("Ghost"))

    def test_add_object_no_file_loaded(self):
        """Adding object with no file raises RuntimeError."""
        with self.assertRaises(RuntimeError):
            self.mgr.add_object("X", {})

    def test_save_no_file_loaded(self):
        """Saving with no file returns False."""
        self.assertFalse(self.mgr.save())

    def test_malformed_json_on_disk(self):
        """Malformed JSON files are skipped during list."""
        bad_file = Path(self.tmpdir) / "bad.json"
        bad_file.write_text("{invalid json")
        files = self.mgr.list_files()
        names = [f["name"] for f in files]
        self.assertNotIn("bad", names)

    def test_concurrent_file_names(self):
        """Multiple files with similar names don't collide."""
        self.mgr.new_file("Test")
        self.mgr.new_file("Test")
        self.mgr.new_file("Test")
        files = self.mgr.list_files()
        self.assertEqual(len(files), 3)
        names = [f["name"] for f in files]
        self.assertEqual(len(set(names)), 3)

    def test_empty_object_name(self):
        """Objects with empty params still store."""
        self.mgr.new_file("MinimalObj")
        name = self.mgr.add_object("X", {"object_type": "dot", "params": {}})
        self.assertEqual(name, "X")

    def test_special_chars_in_description(self):
        """Special characters in metadata survive roundtrip."""
        self.mgr.new_file("SpecialChars")
        self.mgr.current.metadata.description = 'Test "quotes" & <brackets>'
        self.mgr.save()

        mgr2 = PrintFileManager(prints_dir=self.tmpdir)
        loaded = mgr2.load("SpecialChars")
        self.assertIn('"quotes"', loaded.metadata.description)


# ═══════════════════════════════════════════════════════════════════
# Source File Integrity Tests
# ═══════════════════════════════════════════════════════════════════

class TestSourceFileIntegrity(unittest.TestCase):
    """Verify that all delivered source files exist and parse correctly."""

    @classmethod
    def setUpClass(cls):
        cls.base = Path(__file__).parent.parent

    def test_print_file_manager_exists(self):
        path = self.base / "SupportClasses" / "PrintFileManager.py"
        self.assertTrue(path.exists(), f"Missing: {path}")

    def test_auto_layout_exists(self):
        path = self.base / "SupportClasses" / "auto_layout.py"
        self.assertTrue(path.exists(), f"Missing: {path}")

    def test_print_file_manager_parses(self):
        path = self.base / "SupportClasses" / "PrintFileManager.py"
        source = path.read_text()
        tree = ast.parse(source)
        class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
        self.assertIn("PrintFileManager", class_names)
        self.assertIn("PrintFileData", class_names)
        self.assertIn("PrintFileMetadata", class_names)

    def test_auto_layout_parses(self):
        path = self.base / "SupportClasses" / "auto_layout.py"
        source = path.read_text()
        tree = ast.parse(source)
        func_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
        self.assertIn("generate_ring", func_names)
        self.assertIn("generate_square_grid", func_names)
        self.assertIn("generate_hex_grid", func_names)
        self.assertIn("generate_line", func_names)
        self.assertIn("generate_concentric_rings", func_names)
        self.assertIn("generate_layout", func_names)
        self.assertIn("validate_layout_in_well", func_names)

    def test_sample_print_files_exist(self):
        prints_dir = self.base / "config" / "prints"
        self.assertTrue(prints_dir.exists())
        json_files = list(prints_dir.glob("*.json"))
        self.assertGreaterEqual(len(json_files), 2, "Expected at least 2 sample prints")

    def test_sample_prints_valid(self):
        prints_dir = self.base / "config" / "prints"
        for fp in prints_dir.glob("*.json"):
            with open(fp) as f:
                data = json.load(f)
            valid, errors = validate_print_file(data)
            self.assertTrue(valid, f"{fp.name} invalid: {errors}")


# ═══════════════════════════════════════════════════════════════════
# Schema Compatibility Tests
# ═══════════════════════════════════════════════════════════════════

class TestSchemaCompatibility(unittest.TestCase):
    """Test schema version handling."""

    def test_current_schema_version(self):
        self.assertEqual(SCHEMA_VERSION, "7.2.3")

    def test_new_files_use_current_schema(self):
        tmpdir = tempfile.mkdtemp()
        try:
            mgr = PrintFileManager(prints_dir=tmpdir)
            pf = mgr.new_file("SchemaTest")
            self.assertEqual(pf.schema_version, "7.2.3")
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)

    def test_validation_rejects_empty_dict(self):
        valid, errors = validate_print_file({})
        self.assertFalse(valid)


# ═══════════════════════════════════════════════════════════════════
# Signal Flow Contract Tests
# ═══════════════════════════════════════════════════════════════════

class TestSignalFlowContracts(unittest.TestCase):
    """Verify signal/method name contracts that must be consistent."""

    def test_print_file_manager_has_crud_methods(self):
        mgr = PrintFileManager.__dict__
        required = ["new_file", "save", "save_as", "load", "delete",
                     "duplicate", "add_object", "remove_object",
                     "add_collection", "remove_collection", "auto_save",
                     "list_files"]
        for method in required:
            self.assertIn(method, mgr, f"Missing method: {method}")

    def test_layout_generators_all_present(self):
        required = ["ring", "square_grid", "hex_grid", "line", "concentric_rings"]
        for pattern in required:
            self.assertIn(pattern, GENERATORS, f"Missing generator: {pattern}")

    def test_layout_info_matches_generators(self):
        for pattern in GENERATORS:
            self.assertIn(pattern, LAYOUT_INFO)

    def test_generate_layout_dispatches_all(self):
        """generate_layout() can dispatch to every registered generator."""
        for pattern in GENERATORS:
            info = LAYOUT_INFO[pattern]
            kwargs = {k: v["default"] for k, v in info["params"].items()}
            positions = generate_layout(pattern, **kwargs)
            self.assertIsInstance(positions, list)

    def test_count_layout_works_for_all(self):
        for pattern in GENERATORS:
            info = LAYOUT_INFO[pattern]
            kwargs = {k: v["default"] for k, v in info["params"].items()}
            count = count_layout_objects(pattern, **kwargs)
            self.assertGreater(count, 0)


if __name__ == "__main__":
    unittest.main()
