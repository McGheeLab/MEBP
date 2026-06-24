"""test_v75x_sketch_print_visibility.py — sketched prints visible + previewable.

Covers two regressions where a print baked via
``PrintFileManager.save_trajectory_as_print_object`` (Sketch / Image Import)
did not surface for the operator until an app restart:

- Fix B: the writer keyed the CSV pointer as ``csv_path`` but every csv_import
  reader looked for ``source_file`` → trajectory never loaded → blank preview.
  The writer now emits ``source_file`` (with ``csv_path`` alias) and the readers
  accept either key (so already-saved files with only ``csv_path`` still load).
- Fix A: QuickPrintWorkflowPage re-scans config/prints on showEvent so prints
  created this session appear without a restart.
"""

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
from PySide6.QtGui import QShowEvent
from PySide6.QtWidgets import QApplication

from SupportClasses.PrintFileManager import save_trajectory_as_print_object
from SupportClasses.PhysicalModels import NeedleSpec


def _traj() -> np.ndarray:
    # 4-point square path, Nx7 [x,y,z,p1,p2,p3,t]
    return np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 3.0],
        ],
        dtype=np.float64,
    )


def _needle() -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


class TestWriterKey(unittest.TestCase):
    def test_writes_source_file_key(self):
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d,
                object_name="Sketch_1")
            data = json.loads((Path(d) / f"{name}.json").read_text())
            params = data["objects"]["Sketch_1"]["params"]
            self.assertIn("source_file", params)
            # csv_path alias retained for back-compat.
            self.assertEqual(params["source_file"], params["csv_path"])
            self.assertTrue(Path(params["source_file"]).exists())


class TestReaderFallback(unittest.TestCase):
    """The Quick Print path reader loads via source_file AND legacy csv_path."""

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        return QuickPrintWorkflowPage(ctrl, settings=None)

    def _write_csv(self, path: Path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "z", "p1", "p2", "p3", "t"])
            for row in _traj():
                w.writerow([f"{v:.6f}" for v in row])

    def _obj_dict(self, key: str, csv_path: str) -> dict:
        return {
            "name": "Sketch_1",
            "object_type": "csv_import",
            "params": {key: csv_path, "num_waypoints": 4},
        }

    def test_reads_via_source_file(self):
        page = self._page()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.csv"
            self._write_csv(p)
            pts = page._obj_dict_to_path_points(
                self._obj_dict("source_file", str(p)), _needle(), {})
            self.assertEqual(len(pts), 4)

    def test_reads_via_legacy_csv_path(self):
        page = self._page()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.csv"
            self._write_csv(p)
            pts = page._obj_dict_to_path_points(
                self._obj_dict("csv_path", str(p)), _needle(), {})
            self.assertEqual(len(pts), 4)


class TestEndToEnd(TestReaderFallback):
    def test_save_then_read_nonempty_path(self):
        page = self._page()
        with tempfile.TemporaryDirectory() as d:
            name = save_trajectory_as_print_object(
                _traj(), base_name="Sketch", prints_dir=d,
                object_name="Sketch_1")
            data = json.loads((Path(d) / f"{name}.json").read_text())
            od = data["objects"]["Sketch_1"]
            pts = page._obj_dict_to_path_points(od, _needle(), {})
            self.assertEqual(len(pts), 4)


class TestShowEventRefresh(TestReaderFallback):
    def test_show_event_refreshes_objects(self):
        page = self._page()
        with patch.object(page, "_refresh_objects") as mock_refresh:
            page.showEvent(QShowEvent())
            mock_refresh.assert_called_once()


if __name__ == "__main__":
    unittest.main()
