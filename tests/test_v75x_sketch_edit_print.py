"""test_v75x_sketch_edit_print.py — edit any saved print from the Library.

Sketch-created prints embed their vector Sketch (lossless re-edit); others are
imported best-effort from their baked toolpath as editable region shapes.
"Save changes" overwrites the print in place; "Send to Print Setup" saves new.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PySide6.QtWidgets import QApplication, QMessageBox

from SupportClasses.PhysicalModels import NeedleSpec
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory, regions_from_trajectory,
)
from SupportClasses.PrintFileManager import (
    save_trajectory_as_print_object, read_print_objects,
)


def _needle():
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


def _traj():
    return np.array(
        [[0, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0.1, 0, 0, 1],
         [1, 1, 0, 0.2, 0, 0, 2], [0, 1, 0, 0.3, 0, 0, 3]], dtype=float)


# ═══════════════════════════════════════════════════════════════════
# Backend
# ═══════════════════════════════════════════════════════════════════

class TestOverwrite(unittest.TestCase):
    def test_overwrite_uses_name_verbatim(self):
        with tempfile.TemporaryDirectory() as d:
            n1 = save_trajectory_as_print_object(
                _traj(), base_name="Foo", prints_dir=d, object_name="o")
            self.assertEqual(n1, "Foo_1")
            n2 = save_trajectory_as_print_object(
                _traj(), base_name=n1, prints_dir=d, object_name=n1,
                overwrite=True)
            self.assertEqual(n2, "Foo_1")               # no counter bump
            self.assertTrue((Path(d) / "Foo_1.json").exists())
            self.assertFalse((Path(d) / "Foo_1_1.json").exists())

    def test_no_overwrite_increments(self):
        with tempfile.TemporaryDirectory() as d:
            a = save_trajectory_as_print_object(
                _traj(), base_name="Foo", prints_dir=d, object_name="o")
            b = save_trajectory_as_print_object(
                _traj(), base_name="Foo", prints_dir=d, object_name="o")
            self.assertEqual((a, b), ("Foo_1", "Foo_2"))


class TestRegionsFromTrajectory(unittest.TestCase):
    def test_splits_on_travel(self):
        # Two printing runs separated by a travel move (pump flat + Z up).
        traj = np.array([
            [0, 0, 0, 0.0, 0, 0, 0], [2, 0, 0, 0.1, 0, 0, 1],   # print run A
            [2, 0, 0, 0.2, 0, 0, 2],
            [8, 8, 3, 0.2, 0, 0, 3],                            # travel (flat)
            [10, 8, 3, 0.3, 0, 0, 4], [10, 12, 3, 0.4, 0, 0, 5],  # print run B
        ], dtype=float)
        regs = regions_from_trajectory(traj)
        self.assertEqual(len(regs), 2)
        self.assertTrue(all(r.kind == "region" for r in regs))

    def test_flat_pumps_one_region(self):
        traj = np.array([[0, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 1],
                         [2, 0, 0, 0, 0, 0, 2]], dtype=float)
        regs = regions_from_trajectory(traj)
        self.assertEqual(len(regs), 1)

    def test_infers_ink_id(self):
        # Pump column 1 (P2) advances → region assigned to abstract ink id 2
        # (ink_id = inferred pump column + 1).
        traj = np.array([[0, 0, 0, 0, 0.0, 0, 0], [1, 0, 0, 0, 0.1, 0, 1],
                         [2, 0, 0, 0, 0.2, 0, 2]], dtype=float)
        regs = regions_from_trajectory(traj)
        self.assertEqual(regs[0].ink_id, 2)

    def test_degenerate_returns_empty(self):
        self.assertEqual(regions_from_trajectory(np.zeros((1, 7))), [])


# ═══════════════════════════════════════════════════════════════════
# GUI
# ═══════════════════════════════════════════════════════════════════

class _GuiBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _cfg(self):
        class Cfg:
            needle = _needle()
            pumps = {}
            active_plate_key = 96
        return Cfg()

    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        p = SketchPage()
        p.set_hardware_config(self._cfg())
        return p

    def _make_sketch_print(self, d, name="MyPrint", mult=1.5):
        sk = Sketch(shapes=[
            SketchShape(kind="circle", cx=0, cy=0, radius=3),
            SketchShape(kind="rect", cx=8, cy=0, width=4, height=4)])
        sk.extrusion_multiplier = mult
        res = compile_to_trajectory(sk, _needle(), None)
        return save_trajectory_as_print_object(
            res.trajectory, base_name=name, prints_dir=d, object_name=name,
            extra_params={"sketch": sk.to_dict()})


class TestRoundTripEdit(_GuiBase):
    def test_load_stored_sketch(self):
        with tempfile.TemporaryDirectory() as d:
            name = self._make_sketch_print(d, mult=1.5)
            page = self._page()
            page.load_print_for_edit(name, prints_dir=d)
            self.assertEqual(page._editing_name, name)
            self.assertEqual(len(page._canvas.sketch().shapes), 2)
            self.assertAlmostEqual(
                page._canvas.sketch().extrusion_multiplier, 1.5, places=6)
            self.assertFalse(page._save_btn.isHidden())
            self.assertIn(name, page._save_btn.text())

    def test_save_changes_overwrites_in_place(self):
        with tempfile.TemporaryDirectory() as d:
            name = self._make_sketch_print(d)
            page = self._page()
            page.load_print_for_edit(name, prints_dir=d)
            page._canvas.sketch().shapes[0].radius = 5.0
            fired = []
            page.print_file_saved.connect(lambda nm: fired.append(nm))
            with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                       return_value=QMessageBox.Yes):
                page._save_changes()
            # Overwrote the same file (no new one), with the edited sketch.
            files = sorted(p.name for p in Path(d).glob("MyPrint*"))
            self.assertEqual(files, ["MyPrint_1.csv", "MyPrint_1.json"])
            stored = next(iter(read_print_objects(
                Path(d) / f"{name}.json").values()))["params"]["sketch"]
            self.assertAlmostEqual(stored["shapes"][0]["radius"], 5.0, places=6)
            self.assertEqual(fired, [name])

    def test_fresh_page_has_no_save_button(self):
        self.assertTrue(self._page()._save_btn.isHidden())

    def test_load_missing_name_is_safe(self):
        page = self._page()
        page.load_print_for_edit("does_not_exist_xyz")
        self.assertIsNone(page._editing_name)


class TestBestEffortEdit(_GuiBase):
    def test_import_csv_only_print(self):
        with tempfile.TemporaryDirectory() as d:
            traj = np.array([
                [0, 0, 0, 0, 0, 0, 0], [5, 0, 0, 0.1, 0, 0, 1],
                [5, 5, 0, 0.2, 0, 0, 2],
                [10, 10, 3, 0.2, 0, 0, 3],                # travel
                [12, 10, 3, 0.3, 0, 0, 4], [12, 14, 3, 0.4, 0, 0, 5]],
                dtype=float)
            name = save_trajectory_as_print_object(
                traj, base_name="RawPrint", prints_dir=d, object_name="RawPrint")
            page = self._page()
            page.load_print_for_edit(name, prints_dir=d)
            self.assertEqual(page._editing_name, name)
            shapes = page._canvas.sketch().shapes
            self.assertTrue(shapes and all(s.kind == "region" for s in shapes))

    def test_best_effort_save_adds_sketch(self):
        with tempfile.TemporaryDirectory() as d:
            traj = np.array([[0, 0, 0, 0, 0, 0, 0], [5, 0, 0, 0.1, 0, 0, 1],
                             [5, 5, 0, 0.2, 0, 0, 2]], dtype=float)
            name = save_trajectory_as_print_object(
                traj, base_name="RawPrint", prints_dir=d, object_name="RawPrint")
            page = self._page()
            page.load_print_for_edit(name, prints_dir=d)
            with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                       return_value=QMessageBox.Yes):
                page._save_changes()
            stored = next(iter(read_print_objects(
                Path(d) / f"{name}.json").values()))["params"].get("sketch")
            self.assertIsNotNone(stored)   # now re-editable losslessly


class TestSendEmbedsSketch(_GuiBase):
    def test_new_print_carries_sketch(self):
        with tempfile.TemporaryDirectory() as d:
            page = self._page()
            page._prints_dir = d           # write into the temp dir
            page._canvas.sketch().shapes.append(
                SketchShape(kind="circle", cx=0, cy=0, radius=1))
            created = []
            page.print_file_created.connect(lambda nm: created.append(nm))
            with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                       return_value=QMessageBox.Yes):
                page._send_to_print_setup()
            self.assertTrue(created)
            stored = next(iter(read_print_objects(
                Path(d) / f"{created[0]}.json").values()))["params"].get("sketch")
            self.assertIsNotNone(stored)


class TestLibraryEditSignal(_GuiBase):
    def test_card_edit_emits_page_signal(self):
        from gui.pages.print_library import PrintLibraryPage
        with tempfile.TemporaryDirectory() as d:
            save_trajectory_as_print_object(
                _traj(), base_name="P", prints_dir=d, object_name="o")
            page = PrintLibraryPage(prints_dir=d)
            got = []
            page.edit_print_requested.connect(lambda nm: got.append(nm))
            self.assertEqual(len(page._cards), 1)
            page._cards[0].edit_requested.emit(page._cards[0].name())
            self.assertEqual(got, [page._cards[0].name()])


class TestReviewFixes(_GuiBase):
    """Fixes from the adversarial review of this feature."""

    def test_best_effort_recovers_print_height(self):
        # A csv-only print with NO z_above_plate_bottom_mm param must recover
        # its print height from the baked toolpath, not reset to the 0.2 default.
        from SupportClasses.StageController import plate_relative_to_zref
        with tempfile.TemporaryDirectory() as d:
            pb = 6.06
            zp = plate_relative_to_zref(pb, 0.5)      # print height 0.5 mm
            zt = plate_relative_to_zref(pb, 2.5)      # travel height 2.5 mm
            traj = np.array([
                [0, 0, zp, 0.0, 0, 0, 0], [2, 0, zp, 0.1, 0, 0, 1],
                [2, 0, zt, 0.1, 0, 0, 2],             # travel (pump flat)
                [5, 5, zp, 0.2, 0, 0, 3]], dtype=float)
            name = save_trajectory_as_print_object(
                traj, base_name="RawZ", prints_dir=d, object_name="o")
            page = self._page()
            page.set_z_references({"plate_bottom_z": pb})
            page.load_print_for_edit(name, prints_dir=d)
            self.assertAlmostEqual(
                page._canvas.sketch().z_start_mm, 0.5, places=5)

    def test_overwrite_targets_resolved_stem_not_display_name(self):
        # File stem 'weird' but metadata.name 'Fancy' → Save changes must
        # overwrite weird.json (not create Fancy.json / orphan the original).
        import json
        with tempfile.TemporaryDirectory() as d:
            sk = Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=2)])
            (Path(d) / "weird.json").write_text(json.dumps({
                "schema_version": "7.5.0",
                "metadata": {"name": "Fancy"},
                "objects": {"o": {"object_type": "point",
                                  "params": {"sketch": sk.to_dict()}}},
            }))
            page = self._page()
            page.load_print_for_edit("Fancy", prints_dir=d)
            self.assertEqual(page._editing_stem, "weird")
            page._canvas.sketch().shapes[0].radius = 4.0
            with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                       return_value=QMessageBox.Yes):
                page._save_changes()
            self.assertTrue((Path(d) / "weird.json").exists())
            self.assertFalse((Path(d) / "Fancy.json").exists())   # no orphan/dup
            stored = next(iter(read_print_objects(
                Path(d) / "weird.json").values()))["params"]["sketch"]
            self.assertAlmostEqual(stored["shapes"][0]["radius"], 4.0, places=6)

    def test_new_sketch_leaves_edit_mode(self):
        with tempfile.TemporaryDirectory() as d:
            name = self._make_sketch_print(d)
            page = self._page()
            page.load_print_for_edit(name, prints_dir=d)
            self.assertIsNotNone(page._editing_name)
            page._new_sketch()
            self.assertIsNone(page._editing_name)
            self.assertIsNone(page._editing_stem)
            self.assertTrue(page._save_btn.isHidden())
            self.assertEqual(len(page._canvas.sketch().shapes), 0)

    def test_send_retargets_edit_to_new_print(self):
        # After Send-as-new, "Save changes" should target the NEW print, not a
        # previously-edited one (guards the stale-overwrite trap).
        with tempfile.TemporaryDirectory() as d:
            page = self._page()
            page._prints_dir = d
            page._editing_name = "OldPrint"      # simulate a prior edit session
            page._editing_stem = "OldPrint"
            page._canvas.sketch().shapes.append(
                SketchShape(kind="circle", cx=0, cy=0, radius=1))
            created = []
            page.print_file_created.connect(lambda nm: created.append(nm))
            with patch("gui.pages.print_builder_sketch.QMessageBox.warning",
                       return_value=QMessageBox.Yes):
                page._send_to_print_setup()
            self.assertTrue(created)
            self.assertEqual(page._editing_name, created[0])
            self.assertEqual(page._editing_stem, created[0])


class TestPrintBuilderWiring(_GuiBase):
    def test_edit_request_loads_and_switches_to_sketch(self):
        from gui.pages.print_builder import PrintBuilderPage
        pb = PrintBuilderPage()
        with patch.object(pb._sketch_page, "load_print_for_edit") as m:
            pb._library_page.edit_print_requested.emit("SomePrint")
            m.assert_called_once_with("SomePrint")
        self.assertEqual(pb.get_active_index(), 0)   # Sketch tab


if __name__ == "__main__":
    unittest.main()
