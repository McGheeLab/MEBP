"""test_v75x_quick_print_workflow.py — Quick Print workflow page.

Covers the no-frills Quick Print workflow (gui/pages/workflows/
quick_print_workflow.py):

- Page constructs headless; object combo lists the 3 built-in simple shapes.
- Simple shapes (dot / circle / meander) compile to a non-empty XY path via
  the GeometryEngine pipeline (default-needle fallback, no hw_config needed).
- A saved-object dict compiles to a non-empty XY path.
- Well-center resolution prefers the calibrated position (→ zero-ref mm) and
  falls back to the geometric plate position.
- Print-button gating matches connection + selection state.
- End-to-end: build_well_plate_job offsets the path by the well center.
"""

import os
import sys
import tempfile
import unittest
from unittest.mock import MagicMock

# Isolate the workflow-settings store so the page's settings dialog restores
# CODE defaults, not the operator's real saved Quick Print profile.
os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = tempfile.mkdtemp(
    prefix="mebp_qp_settings_")

from PySide6.QtWidgets import QApplication

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PhysicalModels import NeedleSpec


def _needle() -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_page(self, controller=None):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        ctrl = controller if controller is not None else MagicMock()
        # Default: not connected unless a test opts in.
        if controller is None:
            ctrl.is_xy_connected = False
            ctrl.is_zp_connected = False
            ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        return QuickPrintWorkflowPage(ctrl, settings=None)


class TestConstruction(_Base):
    def test_constructs_with_simple_shapes(self):
        page = self._make_page()
        # First three combo entries are the built-in simple shapes.
        kinds = [page._object_combo.itemData(i)
                 for i in range(min(3, page._object_combo.count()))]
        self.assertEqual(kinds[0], "simple:dot")
        self.assertEqual(kinds[1], "simple:circle")
        self.assertEqual(kinds[2], "simple:meander")

    def test_size_field_visibility(self):
        page = self._make_page()
        # Dot → size hidden; circle → visible.
        self._select_simple(page, "dot")
        self.assertFalse(page._size_spin.isVisibleTo(page))
        self._select_simple(page, "circle")
        self.assertTrue(page._size_spin.isVisibleTo(page))

    @staticmethod
    def _select_simple(page, ref):
        idx = page._object_combo.findData(f"simple:{ref}")
        page._object_combo.setCurrentIndex(idx)
        page._on_object_changed()


class TestPathGeneration(_Base):
    def _select_simple(self, page, ref):
        idx = page._object_combo.findData(f"simple:{ref}")
        self.assertGreaterEqual(idx, 0)
        page._object_combo.setCurrentIndex(idx)

    def test_circle_path_non_empty(self):
        page = self._make_page()
        page._size_spin.setValue(1.5)
        self._select_simple(page, "circle")
        pts = page._path_points_for_selection()
        self.assertGreater(len(pts), 8)
        self.assertEqual(len(pts[0]), 2)
        # Outline radius ≈ 1.5 mm → max |x| in that ballpark.
        max_x = max(abs(x) for x, _ in pts)
        self.assertGreater(max_x, 1.0)

    def test_dot_path_non_empty(self):
        page = self._make_page()
        self._select_simple(page, "dot")
        pts = page._path_points_for_selection()
        self.assertGreaterEqual(len(pts), 1)

    def test_meander_path_non_empty(self):
        page = self._make_page()
        page._size_spin.setValue(2.0)
        self._select_simple(page, "meander")
        pts = page._path_points_for_selection()
        self.assertGreater(len(pts), 4)

    def test_saved_object_dict_to_path(self):
        page = self._make_page()
        obj = {
            "name": "Ring", "object_type": "circle",
            "params": {"radius": 1.0, "num_points": 32, "filled": False},
            "source": "parametric",
        }
        pts = page._obj_dict_to_path_points(obj, _needle(), {})
        self.assertGreater(len(pts), 8)

    def test_obj_dict_input_not_mutated(self):
        page = self._make_page()
        obj = {"name": "Ring", "object_type": "circle",
               "params": {"radius": 1.0}, "source": "parametric"}
        before = dict(obj)
        page._obj_dict_to_path_points(obj, _needle(), {})
        # from_dict pops 'trajectory'; we deep-copy first, so caller dict intact.
        self.assertEqual(obj, before)


class TestWellCenter(_Base):
    def test_prefers_calibrated_position(self):
        ctrl = MagicMock()
        ctrl.zero_position = {"x": 1000.0, "y": 2000.0, "Z": 0.0}
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        page = self._make_page(controller=ctrl)
        # Calibrated A1 at 5000/7000 µm → zero-ref mm = (4.0, 5.0)
        page._well_positions = {"A1": (5000.0, 7000.0)}
        page._plate = WellPlate.from_format(96)
        cx, cy = page._well_center_zero_ref_mm("A1")
        self.assertAlmostEqual(cx, 4.0, places=6)
        self.assertAlmostEqual(cy, 5.0, places=6)

    def test_falls_back_to_geometric(self):
        page = self._make_page()
        page._well_positions = None
        plate = WellPlate.from_format(96)
        page._plate = plate
        center = page._well_center_zero_ref_mm("A2")
        self.assertEqual(center, plate.get_well_position("A2"))

    def test_none_when_unknown(self):
        page = self._make_page()
        page._well_positions = None
        page._plate = None
        self.assertIsNone(page._well_center_zero_ref_mm("A1"))


class TestButtonGating(_Base):
    def test_disabled_when_not_connected(self):
        page = self._make_page()
        page._update_button_state()
        self.assertFalse(page._print_btn.isEnabled())

    def test_enabled_when_ready(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        page = self._make_page(controller=ctrl)
        page._plate = WellPlate.from_format(96)
        page._selected_well = "A1"
        # An object is selected by default (first simple shape).
        page._update_button_state()
        self.assertTrue(page._print_btn.isEnabled())

    def test_disabled_without_well(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        page = self._make_page(controller=ctrl)
        page._plate = WellPlate.from_format(96)
        page._selected_well = None
        page._update_button_state()
        self.assertFalse(page._print_btn.isEnabled())


class TestJobBuilding(_Base):
    def test_build_well_plate_job_offsets_by_center(self):
        from SupportClasses.PrintManager import build_well_plate_job, CommandType
        page = self._make_page()
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._size_spin.setValue(1.0)
        page._safe_z = 5.0
        pts = page._path_points_for_selection()
        self.assertTrue(pts)
        settings = page._build_settings()

        well_x, well_y = 10.0, 20.0
        job = build_well_plate_job(
            well_positions=[("B3", well_x, well_y)],
            path_points=pts,
            settings=settings,
            pump="P1",
            flow_rate=0.01,
            job_name="t",
        )
        self.assertGreater(job.total_steps, 0)
        # The PRINT_PATH points are the well-relative path shifted by center.
        print_cmds = [c for c in job.commands
                      if c.type == CommandType.PRINT_PATH]
        self.assertTrue(print_cmds)
        first_px = print_cmds[0].params["points"][0]
        self.assertAlmostEqual(first_px[0], well_x + pts[0][0], places=6)
        self.assertAlmostEqual(first_px[1], well_y + pts[0][1], places=6)

    def test_settings_use_safe_z_and_flow(self):
        page = self._make_page()
        page._safe_z = 7.5
        page._auto_flow_100_uL_s = lambda: 0.4   # auto flow (needle×speed×mod)
        # v7.5.x: print-speed % scales BOTH speed and flow. At 100% the auto
        # Flow@100% passes straight through to the pump rate.
        page._speed_pct_spin.setValue(100)
        page._printz_spin.setValue(0.2)
        # v7.5.x: the print-Z spin is now a height above the plate bottom,
        # resolved via the controller. Uncalibrated (None) → the height passes
        # through as the zero-ref print Z.
        page._controller.print_height_to_zref.return_value = None
        s = page._build_settings()
        self.assertEqual(s.travel_z_height, 7.5)
        self.assertAlmostEqual(s.print_z_height, 0.2)
        self.assertAlmostEqual(s.pump_rate_uL_s, 0.4)
        self.assertAlmostEqual(s.get_pump_rate(page._pump()), 0.4)

    def test_print_z_is_plate_bottom_relative(self):
        # v7.5.x: when the plate bottom is calibrated, the spin's height is
        # converted to a zero-ref Z via the controller datum.
        page = self._make_page()
        page._printz_spin.setValue(0.2)
        # Emulate ME3B V1 (plate bottom -25.68, ZDIR=-1): 0.2 above → -25.88.
        page._controller.print_height_to_zref.return_value = -25.88
        s = page._build_settings()
        self.assertAlmostEqual(s.print_z_height, -25.88)
        page._controller.print_height_to_zref.assert_called_with(0.2)


if __name__ == "__main__":
    unittest.main()
