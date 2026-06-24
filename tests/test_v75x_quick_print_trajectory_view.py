"""test_v75x_quick_print_trajectory_view.py — Quick Print live trajectory view.

Covers the v7.5.x Quick Print trajectory monitor (gui/widgets/
print_trajectory_monitor.py) and its wiring into the Quick Print page
(gui/pages/workflows/quick_print_workflow.py):

Widget:
- set_planned_path computes a content bounding box (auto-fit) and clears it
  again on None.
- set_position appends a ghost-trail breadcrumb only when the needle moved
  > 1 µm, and the trail caps at the JogWorkspaceView length (24).
- The persistent executed polyline accumulates ONLY while recording, and
  reset_live() clears both the executed polyline and the breadcrumb trail.
- A well boundary is included in the framed bbox.
- paintEvent renders without raising (placeholder + populated).

Page:
- Selecting a well + object produces a planned path in zero-ref µm placed at
  the well centre.
- on_status_update pushes the live needle position (absolute µm − zero) into
  the trajectory view.
- The page still constructs without a camera_manager (backward compatible).
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from SupportClasses.WellPlate import WellPlate


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class TestTrajectoryMonitorWidget(_Base):
    def _view(self):
        from gui.widgets.print_trajectory_monitor import (
            PrintTrajectoryMonitorView,
        )
        return PrintTrajectoryMonitorView()

    def test_planned_path_sets_bbox(self):
        v = self._view()
        self.assertIsNone(v._bbox)
        v.set_planned_path([[(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]])
        self.assertIsNotNone(v._bbox)
        x_min, y_min, x_max, y_max = v._bbox
        self.assertLessEqual(x_min, 0.0)
        self.assertGreaterEqual(x_max, 1000.0)
        # None clears it back to the placeholder state.
        v.set_planned_path(None)
        self.assertIsNone(v._bbox)
        self.assertEqual(v._segments, [])

    def test_min_span_enforced_for_tiny_path(self):
        v = self._view()
        v.set_planned_path([[(0.0, 0.0), (1.0, 1.0)]])  # ~1 µm path
        x_min, y_min, x_max, y_max = v._bbox
        self.assertGreaterEqual(x_max - x_min, 499.0)
        self.assertGreaterEqual(y_max - y_min, 499.0)

    def test_well_boundary_included_in_bbox(self):
        v = self._view()
        v.set_planned_path([[(0.0, 0.0), (100.0, 0.0)]])
        v.set_well_boundary((0.0, 0.0), 3000.0)
        x_min, y_min, x_max, y_max = v._bbox
        self.assertLessEqual(x_min, -3000.0)
        self.assertGreaterEqual(x_max, 3000.0)

    def test_breadcrumb_only_on_real_move(self):
        v = self._view()
        v.set_position(0.0, 0.0)
        self.assertEqual(len(v._breadcrumbs), 1)
        # Sub-µm jitter is ignored.
        v.set_position(0.5, 0.5)
        self.assertEqual(len(v._breadcrumbs), 1)
        # A real move appends.
        v.set_position(100.0, 0.0)
        self.assertEqual(len(v._breadcrumbs), 2)

    def test_breadcrumb_caps_at_24(self):
        v = self._view()
        for i in range(40):
            v.set_position(float(i * 100), 0.0)
        self.assertEqual(len(v._breadcrumbs), 24)

    def test_executed_accumulates_only_when_recording(self):
        v = self._view()
        # Not recording → no executed polyline, but ghost trail still grows.
        v.set_position(0.0, 0.0)
        v.set_position(100.0, 0.0)
        self.assertEqual(len(v._executed), 0)
        self.assertEqual(len(v._breadcrumbs), 2)
        # Recording → executed accumulates.
        v.set_recording(True)
        v.set_position(200.0, 0.0)
        v.set_position(300.0, 0.0)
        self.assertEqual(len(v._executed), 2)

    def test_reset_live_clears_executed_and_trail(self):
        v = self._view()
        v.set_recording(True)
        for i in range(5):
            v.set_position(float(i * 100), 0.0)
        self.assertTrue(v._executed)
        self.assertTrue(v._breadcrumbs)
        v.reset_live()
        self.assertEqual(len(v._executed), 0)
        self.assertEqual(len(v._breadcrumbs), 0)

    def test_set_position_none_clears_needle(self):
        v = self._view()
        v.set_position(10.0, 20.0)
        self.assertIsNotNone(v._needle_x_um)
        v.set_position(None, None)
        self.assertIsNone(v._needle_x_um)
        self.assertIsNone(v._needle_y_um)

    def test_paint_smoke_placeholder_and_populated(self):
        v = self._view()
        v.resize(320, 260)
        # Placeholder branch (no path).
        v.grab()
        # Populated branch (path + boundary + live overlays).
        v.set_planned_path([[(0.0, 0.0), (500.0, 500.0), (1000.0, 0.0)]])
        v.set_well_boundary((0.0, 0.0), 1500.0)
        v.set_recording(True)
        v.set_position(0.0, 0.0)
        v.set_position(250.0, 250.0)
        v.grab()  # must not raise


class TestPagePlannedPath(_Base):
    def _make_page(self, ctrl):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        return QuickPrintWorkflowPage(ctrl, settings=None)

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 1000.0, "y": 2000.0, "Z": 0.0}
        # Print-Z passthrough so _build_settings stays simple if called.
        ctrl.print_height_to_zref.return_value = None
        return ctrl

    def test_planned_path_placed_at_well_center(self):
        ctrl = self._ctrl()
        page = self._make_page(ctrl)
        page._plate = WellPlate.from_format(96)
        # Calibrated A1 at 5000/7000 µm; zero 1000/2000 → centre zero-ref mm
        # (4.0, 5.0) → µm (4000, 5000).
        page._well_positions = {"A1": (5000.0, 7000.0)}
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._size_spin.setValue(1.0)  # 1 mm radius
        page._on_well_clicked("A1")

        self.assertTrue(page._traj_view._segments)
        xs = [x for seg in page._traj_view._segments for x, _ in seg]
        ys = [y for seg in page._traj_view._segments for _, y in seg]
        # All planned waypoints sit within ~1 mm (1000 µm) of the well centre.
        self.assertLess(max(xs), 4000.0 + 1100.0)
        self.assertGreater(min(xs), 4000.0 - 1100.0)
        self.assertLess(max(ys), 5000.0 + 1100.0)
        self.assertGreater(min(ys), 5000.0 - 1100.0)

    def test_no_path_without_well(self):
        ctrl = self._ctrl()
        page = self._make_page(ctrl)
        page._plate = WellPlate.from_format(96)
        page._well_positions = {"A1": (5000.0, 7000.0)}
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._refresh_planned_path()  # no well selected
        self.assertEqual(page._traj_view._segments, [])

    def test_status_update_pushes_live_position(self):
        ctrl = self._ctrl()
        ctrl.get_xy_position.return_value = (4500.0, 5500.0)  # absolute µm
        page = self._make_page(ctrl)
        page.on_status_update()
        # zero-ref µm = absolute − zero = (3500, 3500)
        self.assertAlmostEqual(page._traj_view._needle_x_um, 3500.0, places=3)
        self.assertAlmostEqual(page._traj_view._needle_y_um, 3500.0, places=3)

    def test_constructs_without_camera_manager(self):
        page = self._make_page(self._ctrl())
        self.assertIsNone(page._camera_view)
        self.assertTrue(hasattr(page, "_traj_view"))


if __name__ == "__main__":
    unittest.main()
