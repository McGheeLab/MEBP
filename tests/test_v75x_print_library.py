"""test_v75x_print_library.py — Print Library / Manager (Print Builder tab).

Backend (PrintFileManager): delete now removes the sibling CSV + backups,
new rename() (with CSV pointer rewrite), and orphan detection/cleanup.
GUI (PrintLibraryPage): offscreen build / refresh / delete / rename smoke.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PySide6.QtWidgets import QApplication, QMessageBox

from SupportClasses.PrintFileManager import (
    PrintFileManager, save_trajectory_as_print_object, read_print_objects,
)
from SupportClasses.PhysicalModels import NeedleSpec


def _traj() -> np.ndarray:
    return np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 0.2, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 0.3, 0.0, 0.0, 3.0],
        ],
        dtype=np.float64,
    )


def _needle() -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


# ═══════════════════════════════════════════════════════════════════
# Backend
# ═══════════════════════════════════════════════════════════════════

class TestDeleteCleansCompanions(unittest.TestCase):
    def test_delete_removes_sibling_csv(self):
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d,
                object_name="Sketch_1")
            csv = Path(d) / f"{name}.csv"
            js = Path(d) / f"{name}.json"
            self.assertTrue(csv.exists() and js.exists())

            mgr = PrintFileManager(d)
            self.assertTrue(mgr.delete(name))
            self.assertFalse(csv.exists())
            self.assertFalse(js.exists())

    def test_delete_removes_backup(self):
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d,
                object_name="Sketch_1")
            bak = Path(d) / f"{name}.json.bak-v7.2.3"
            bak.write_text("{}")
            mgr = PrintFileManager(d)
            mgr.delete(name)
            self.assertFalse(bak.exists())

    def test_delete_missing_returns_false(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertFalse(PrintFileManager(d).delete("nope"))

    def test_delete_keeps_external_csv(self):
        """A print referencing a CSV outside the prints dir must not delete
        that external file (delete only removes the same-stem sibling)."""
        with tempfile.TemporaryDirectory() as d, \
                tempfile.TemporaryDirectory() as ext:
            external = Path(ext) / "shared.csv"
            external.write_text("x,y,z,p1,p2,p3,t\n0,0,0,0,0,0,0\n1,1,0,0,0,0,1\n")
            mgr = PrintFileManager(d)
            (Path(d) / "P.json").write_text(json.dumps({
                "schema_version": "7.5.0",
                "metadata": {"name": "P"},
                "objects": {"o": {"object_type": "csv_import",
                                  "params": {"source_file": str(external)}}},
            }))
            mgr.delete("P")
            self.assertTrue(external.exists())


class TestRename(unittest.TestCase):
    def test_rename_moves_json_and_csv_and_rewrites_pointer(self):
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d,
                object_name="Sketch_1")
            mgr = PrintFileManager(d)
            self.assertTrue(mgr.rename(name, "My Print"))

            # Old files gone, new files present.
            self.assertFalse((Path(d) / f"{name}.json").exists())
            self.assertFalse((Path(d) / f"{name}.csv").exists())
            new_json = Path(d) / "My_Print.json"
            new_csv = Path(d) / "My_Print.csv"
            self.assertTrue(new_json.exists() and new_csv.exists())

            data = json.loads(new_json.read_text())
            self.assertEqual(data["metadata"]["name"], "My Print")
            params = data["objects"]["Sketch_1"]["params"]
            # Pointer repointed at the renamed CSV, and it resolves.
            self.assertEqual(Path(params["source_file"]).name, "My_Print.csv")
            self.assertTrue(Path(params["source_file"]).exists())

    def test_rename_fails_on_existing_name(self):
        with tempfile.TemporaryDirectory() as d:
            a = save_trajectory_as_print_object(
                _traj(), base_name="A", prints_dir=d, object_name="o")
            b = save_trajectory_as_print_object(
                _traj(), base_name="B", prints_dir=d, object_name="o")
            mgr = PrintFileManager(d)
            self.assertFalse(mgr.rename(a, b))
            # Both untouched.
            self.assertTrue((Path(d) / f"{a}.json").exists())
            self.assertTrue((Path(d) / f"{b}.json").exists())

    def test_rename_empty_name_fails(self):
        with tempfile.TemporaryDirectory() as d:
            a = save_trajectory_as_print_object(
                _traj(), base_name="A", prints_dir=d, object_name="o")
            self.assertFalse(PrintFileManager(d).rename(a, "   "))


class TestOrphans(unittest.TestCase):
    def test_orphan_detection_and_cleanup(self):
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Keep", prints_dir=d, object_name="o")
            live_csv = Path(d) / f"{name}.csv"
            # Orphans: a stray CSV with no JSON, and a migration backup.
            orphan_csv = Path(d) / "stray.csv"
            orphan_csv.write_text("x,y\n")
            bak = Path(d) / "old.json.bak-v7.2.3"
            bak.write_text("{}")

            mgr = PrintFileManager(d)
            orphans = {p.name for p in mgr.orphan_files()}
            self.assertIn("stray.csv", orphans)
            self.assertIn("old.json.bak-v7.2.3", orphans)
            self.assertNotIn(live_csv.name, orphans)   # referenced → not orphan

            self.assertEqual(mgr.cleanup_orphans(), 2)
            self.assertFalse(orphan_csv.exists())
            self.assertFalse(bak.exists())
            self.assertTrue(live_csv.exists())          # survivor
            self.assertTrue((Path(d) / f"{name}.json").exists())


class TestReadPrintObjects(unittest.TestCase):
    def test_reads_objects(self):
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d, object_name="Sketch_1")
            objs = read_print_objects(Path(d) / f"{name}.json")
            self.assertIn("Sketch_1", objs)

    def test_bad_path_returns_empty(self):
        self.assertEqual(read_print_objects("/no/such/file.json"), {})


# ═══════════════════════════════════════════════════════════════════
# GUI (offscreen)
# ═══════════════════════════════════════════════════════════════════

class _GuiBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _seed(self, d, n=2):
        names = []
        for i in range(n):
            names.append(save_trajectory_as_print_object(
                _traj(), base_name=f"P{i}", prints_dir=d, object_name="o"))
        return names


class TestObjectPolylines(_GuiBase):
    def test_csv_object_polyline_nonempty(self):
        from gui.pages.print_library import object_polylines
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d, object_name="Sketch_1")
            objs = read_print_objects(Path(d) / f"{name}.json")
            polys = object_polylines(objs, _needle(), {})
            self.assertEqual(len(polys), 1)
            _color, pts = polys[0]
            self.assertEqual(len(pts), 4)


class TestPage(_GuiBase):
    def _page(self, d):
        from gui.pages.print_library import PrintLibraryPage
        return PrintLibraryPage(prints_dir=d)

    def test_builds_one_card_per_print(self):
        with tempfile.TemporaryDirectory() as d:
            self._seed(d, 3)
            page = self._page(d)
            self.assertEqual(len(page._cards), 3)

    def test_delete_removes_card_and_emits(self):
        with tempfile.TemporaryDirectory() as d:
            names = self._seed(d, 2)
            page = self._page(d)
            fired = []
            page.print_files_changed.connect(lambda: fired.append(True))
            with patch("gui.pages.print_library.QMessageBox.question",
                       return_value=QMessageBox.Yes):
                page._on_delete(names[0])
            self.assertEqual(len(page._cards), 1)
            self.assertFalse((Path(d) / f"{names[0]}.json").exists())
            self.assertTrue(fired)

    def test_rename_via_dialog(self):
        with tempfile.TemporaryDirectory() as d:
            names = self._seed(d, 1)
            page = self._page(d)
            with patch("gui.pages.print_library.QInputDialog.getText",
                       return_value=("Renamed", True)):
                page._on_rename(names[0])
            self.assertTrue((Path(d) / "Renamed.json").exists())
            self.assertIn("Renamed", [c.name() for c in page._cards])

    def test_duplicate_is_standalone(self):
        with tempfile.TemporaryDirectory() as d:
            names = self._seed(d, 1)
            page = self._page(d)
            with patch("gui.pages.print_library.QInputDialog.getText",
                       return_value=("Copy1", True)):
                page._on_duplicate(names[0])
            # Duplicate got its own CSV (deleting the original won't break it).
            self.assertTrue((Path(d) / "Copy1.json").exists())
            self.assertTrue((Path(d) / "Copy1.csv").exists())

    def test_open_emits_print_file_created(self):
        with tempfile.TemporaryDirectory() as d:
            names = self._seed(d, 1)
            page = self._page(d)
            got = []
            page.print_file_created.connect(lambda nm: got.append(nm))
            page._on_open(names[0])
            self.assertEqual(got, [names[0]])

    def test_bulk_delete(self):
        with tempfile.TemporaryDirectory() as d:
            self._seed(d, 3)
            page = self._page(d)
            for c in page._cards:
                c._check.setChecked(True)
            with patch("gui.pages.print_library.QMessageBox.question",
                       return_value=QMessageBox.Yes):
                page._on_delete_selected()
            self.assertEqual(len(page._cards), 0)

    def test_set_hardware_config_refreshes(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        with tempfile.TemporaryDirectory() as d:
            self._seed(d, 1)
            page = self._page(d)
            # Should not raise; still one card.
            page.set_hardware_config(HardwareConfig())
            self.assertEqual(len(page._cards), 1)


if __name__ == "__main__":
    unittest.main()
