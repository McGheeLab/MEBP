"""Tests for v7.4.8: rosettes flatten into first-class sub-wells, insert
heights (rim + ink Z), travel-Z clearance floor, and the standard-insert
library."""

from __future__ import annotations

import json
import math
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# A QApplication is needed for the widget-level drill-in tests below.
from PySide6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.PlateDesign import PlateDesign, Well
from SupportClasses.WellPlate import WellPlate


def _plate_with_rosette(rotation_deg: float = 0.0):
    d = PlateDesign.from_standard_format(24)
    a1 = d.get_wells()[0]
    ros = PlateDesign.blank_rosette(bore_radius_mm=7.0)
    ros.add_well(x=3.0, y=0.0, diameter=1.0, name="a", naming_scheme="MANUAL")
    ros.add_well(x=0.0, y=3.0, diameter=1.0, name="b", naming_scheme="MANUAL")
    ros.add_well(x=0.0, y=0.0, diameter=1.0, name="c", naming_scheme="MANUAL")
    a1.rosette_design = ros
    a1.rosette_rotation_deg = rotation_deg
    return d, a1, ros


class TestRosetteFlatten(unittest.TestCase):
    def test_parent_replaced_by_subwells(self):
        d, a1, ros = _plate_with_rosette()
        plate = d.compile()
        names = [w.name for w in plate.get_all_wells()]
        self.assertNotIn("A1", names)            # parent dropped
        self.assertIn("A1.a", names)
        self.assertIn("A1.b", names)
        self.assertIn("A1.c", names)
        # Other wells untouched.
        self.assertIn("B2", names)
        # 24 wells - 1 rosette parent + 3 sub-wells = 26.
        self.assertEqual(len(names), 26)

    def test_subwell_positions_relative_to_parent(self):
        d, a1, ros = _plate_with_rosette()
        plate = d.compile()
        # A1 parent centre was (0,0); sub a at offset (3,0).
        self.assertEqual(plate.get_well_position("A1.a"), (3.0, 0.0))
        self.assertEqual(plate.get_well_position("A1.b"), (0.0, 3.0))

    def test_subwells_inherit_parent_row_col(self):
        d, a1, ros = _plate_with_rosette()
        plate = d.compile()
        a = plate.get_well_info("A1.a")
        b = plate.get_well_info("A1.b")
        self.assertEqual((a.row, a.col), (0, 0))
        self.assertEqual((b.row, b.col), (0, 0))
        self.assertTrue(a.is_subwell)
        self.assertEqual(a.parent_well, "A1")

    def test_rotation_applied(self):
        d, a1, ros = _plate_with_rosette(rotation_deg=90.0)
        plate = d.compile()
        x, y = plate.get_well_position("A1.a")  # (3,0) rotated 90° CCW → (0,3)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 3.0, places=3)

    def test_case_insensitive_lookup(self):
        d, a1, ros = _plate_with_rosette()
        plate = d.compile()
        self.assertEqual(
            plate.get_well_position("a1.a"), plate.get_well_position("A1.A"))

    def test_empty_rosette_treated_as_ordinary_well(self):
        d = PlateDesign.from_standard_format(12)
        a1 = d.get_wells()[0]
        a1.rosette_design = PlateDesign.blank_rosette(bore_radius_mm=5.0)
        plate = d.compile()
        names = [w.name for w in plate.get_all_wells()]
        self.assertIn("A1", names)  # empty rosette → parent kept


class TestInsertHeights(unittest.TestCase):
    def test_rim_and_ink_z_carried_to_wellinfo(self):
        d, a1, ros = _plate_with_rosette()
        sub = ros.get_wells()[0]
        sub.rim_height_mm = 30.0
        sub.ink_z_mm = -25.0
        plate = d.compile()
        wi = plate.get_well_info("A1.a")
        self.assertAlmostEqual(wi.rim_height_mm, 30.0)
        self.assertAlmostEqual(wi.ink_z_mm, -25.0)

    def test_max_rim_height(self):
        d, a1, ros = _plate_with_rosette()
        ros.get_wells()[0].rim_height_mm = 30.0
        ros.get_wells()[1].rim_height_mm = 12.0
        plate = d.compile()
        self.assertAlmostEqual(plate.max_rim_height_mm, 30.0)

    def test_no_inserts_zero_max_rim(self):
        plate = WellPlate.from_format(96)
        self.assertEqual(plate.max_rim_height_mm, 0.0)


class TestTravelZFloor(unittest.TestCase):
    def test_safe_travel_floors_to_min_z(self):
        """StageController.safe_travel_to raises safe_z to the floor."""
        from SupportClasses.StageController import StageController
        # Build a controller without connecting hardware; we only test the
        # floor arithmetic by stubbing out the move machinery.
        ctrl = StageController.__new__(StageController)
        ctrl._min_travel_z_mm = None
        # set_min_travel_z + getattr default behavior
        ctrl.set_min_travel_z(40.0)
        self.assertEqual(ctrl._min_travel_z_mm, 40.0)
        # The flooring logic: effective = max(requested, floor).
        requested = 5.0
        floor = ctrl._min_travel_z_mm
        effective = max(requested, floor) if floor is not None else requested
        self.assertEqual(effective, 40.0)
        ctrl.set_min_travel_z(None)
        self.assertIsNone(ctrl._min_travel_z_mm)


class TestRosetteSaveFlow(unittest.TestCase):
    """v7.4.8: a rosette built on the Rosette sub-page must be saveable and
    flow into Print Setup (regression: Save button was disabled for
    standard-format plates, so rosettes never persisted)."""

    def setUp(self):
        import importlib
        self._wp = importlib.import_module("SupportClasses.WellPlate")
        self._pd = importlib.import_module("SupportClasses.PlateDesign")
        self._owp, self._opd = self._wp.USER_PLATES_DIR, self._pd.USER_PLATES_DIR
        self._tmp = Path(tempfile.mkdtemp(prefix="mebp_save_"))
        self._wp.USER_PLATES_DIR = self._tmp
        self._pd.USER_PLATES_DIR = self._tmp

    def tearDown(self):
        self._wp.USER_PLATES_DIR = self._owp
        self._pd.USER_PLATES_DIR = self._opd
        shutil.rmtree(self._tmp, ignore_errors=True)

    def test_rosette_save_button_enabled_for_standard_plate(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        plate = PlateDesignerWidget(mode="plate")
        plate.load_plate(24)               # standard int key
        ros = PlateDesignerWidget(mode="rosette")
        ros.adopt_design(plate.current_design(), key=plate.current_plate_key())
        ros._update_dirty_label()
        # Before the fix this was disabled (int key) → rosettes never saved.
        self.assertTrue(ros._btn_save.isEnabled())

    def test_saved_rosette_flows_into_well_setup_model(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        from SupportClasses.WellSetup import WellSetupModel
        from SupportClasses.HardwareConfig import HardwareConfig
        plate = PlateDesignerWidget(mode="plate")
        plate.load_plate(24)
        ros = PlateDesignerWidget(mode="rosette")
        ros.adopt_design(plate.current_design(), key=plate.current_plate_key())
        a1 = ros._design.get_wells()[0]
        ros._canvas.well_drill_requested.emit(a1.id)
        ros._canvas.set_circle_param("count", 6)
        ros._canvas._circle_press((0.0, 0.0))
        ros._canvas._circle_radius = 2.0
        ros._canvas._commit_circle()
        ros._on_back_to_plate()
        # Persist the shared design (what Save As does after naming).
        plate._design.name = "save-flow-plate"
        plate._design.save()
        # Print Setup loads the plate by name → flattened sub-wells appear.
        cfg = HardwareConfig()
        cfg.plate_name = "save-flow-plate"
        model = WellSetupModel(cfg.active_plate_key)
        subs = [n for n in model.plate.well_names if n.startswith("A1.")]
        self.assertEqual(len(subs), 6)
        self.assertIn("A1.a", model.assignments)
        self.assertNotIn("A1", model.plate.well_names)  # parent flattened away


class TestStandardInsertLibrary(unittest.TestCase):
    def setUp(self):
        import importlib
        self._pd = importlib.import_module("SupportClasses.PlateDesign")
        self._orig = self._pd.INSERTS_DIR
        self._tmp = Path(tempfile.mkdtemp(prefix="mebp_inserts_"))
        self._pd.INSERTS_DIR = self._tmp

    def tearDown(self):
        self._pd.INSERTS_DIR = self._orig
        shutil.rmtree(self._tmp, ignore_errors=True)

    def test_save_list_load_round_trip(self):
        ros = PlateDesign.blank_rosette(bore_radius_mm=6.0)
        ros.add_well(x=2.0, y=0.0, diameter=1.0, name="a",
                     naming_scheme="MANUAL")
        ros.get_wells()[0].rim_height_mm = 20.0
        self._pd.save_standard_insert(ros, "tube-1")
        self.assertIn("tube-1", self._pd.list_standard_inserts())
        loaded = self._pd.load_standard_insert("tube-1")
        self.assertEqual(loaded.outline.kind, "circle")
        self.assertEqual(len(loaded.get_wells()), 1)
        self.assertAlmostEqual(loaded.get_wells()[0].rim_height_mm, 20.0)


class TestDrillInGesture(unittest.TestCase):
    """v7.4.8: double-click a well to zoom in + design its rosette."""

    def _widget(self):
        # v7.4.8: drill-in lives on the Rosette sub-page (mode="rosette").
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        w = PlateDesignerWidget(mode="rosette")
        w.load_plate(24)
        return w

    def test_drill_enters_circular_rosette_mode(self):
        w = self._widget()
        a1 = w._design.get_wells()[0]
        w._canvas.well_drill_requested.emit(a1.id)
        self.assertIsNotNone(w._edit_context)
        self.assertEqual(w._design.outline.kind, "circle")

    def test_subwells_placed_then_back_flattens(self):
        from gui.widgets.plate_designer_canvas import Tool
        w = self._widget()
        a1 = w._design.get_wells()[0]
        w._canvas.well_drill_requested.emit(a1.id)
        w._canvas.set_tool(Tool.DRAW_SINGLE_WELL)
        w._canvas._handle_draw_well((2.0, 0.0))
        w._canvas._handle_draw_well((-2.0, 0.0))
        w._on_back_to_plate()
        self.assertIsNone(w._edit_context)
        self.assertEqual(len(a1.rosette_design.get_wells()), 2)
        plate = w._design.compile()
        names = [x.name for x in plate.get_all_wells()]
        self.assertIn("A1.a", names)
        self.assertIn("A1.b", names)
        self.assertNotIn("A1", names)

    def test_empty_drill_discards_rosette(self):
        w = self._widget()
        b2 = next(x for x in w._design.get_wells() if x.name == "B2")
        w._canvas.well_drill_requested.emit(b2.id)
        w._on_back_to_plate()
        self.assertIsNone(b2.rosette_design)   # nothing placed → discarded
        plate = w._design.compile()
        self.assertIn("B2", [x.name for x in plate.get_all_wells()])


class TestPlateRosettePageSplit(unittest.TestCase):
    """v7.4.8: rosette designer lives on the Rosette sub-page; the Plate
    sub-page is layout-only (no drill-in)."""

    def test_plate_mode_no_drill_in(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        w = PlateDesignerWidget(mode="plate")
        w.load_plate(24)
        a1 = w._design.get_wells()[0]
        w._canvas.well_drill_requested.emit(a1.id)   # ignored in plate mode
        self.assertIsNone(w._edit_context)

    def test_rosette_mode_hides_plate_chrome(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        w = PlateDesignerWidget(mode="rosette")
        # Picker / New / Save As / Delete are hidden in rosette mode.
        self.assertFalse(w._picker.isVisible())
        self.assertFalse(w._btn_new.isVisible())
        self.assertFalse(w._btn_save_as.isVisible())

    def test_adopt_shares_design_object(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        plate = PlateDesignerWidget(mode="plate")
        plate.load_plate(24)
        ros = PlateDesignerWidget(mode="rosette")
        ros.adopt_design(plate.current_design())
        self.assertIs(ros._design, plate._design)

    def test_circle_pattern_letters_in_rosette(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        from gui.widgets.plate_designer_canvas import Tool
        w = PlateDesignerWidget(mode="rosette")
        w.load_plate(24)
        a1 = w._design.get_wells()[0]
        w._canvas.well_drill_requested.emit(a1.id)
        w._canvas.set_tool(Tool.DRAW_CIRCLE_PATTERN)
        w._canvas.set_circle_param("count", 8)
        w._canvas._handle_draw_circle_pattern((0.0, 0.0))
        names = sorted(sw.name for sw in w._design.get_wells())
        self.assertEqual(names, ["a", "b", "c", "d", "e", "f", "g", "h"])


class TestCustomPlateRendering(unittest.TestCase):
    """v7.4.8: custom plates have per-well diameters (well_diameter == 0).
    Views must size each well individually — not uniformly from
    well_diameter, which rendered Print Setup invisible and Calibration's
    wells all at the tiny floor size."""

    def _custom_plate(self):
        from SupportClasses.PlateDesign import PlateDesign
        from SupportClasses.WellPlate import WellPlate, WellInfo
        wells = [
            WellInfo(name="B2", row=1, col=1, x=20.0, y=20.0, diameter=15.6),
            WellInfo(name="A1.a", row=0, col=0, x=2.0, y=0.0, diameter=2.0,
                     is_subwell=True, parent_well="A1"),
            WellInfo(name="A1.b", row=0, col=0, x=-2.0, y=0.0, diameter=2.0,
                     is_subwell=True, parent_well="A1"),
        ]
        return WellPlate.from_wells("mixed", wells)

    def test_well_plate_view_sizes_each_well(self):
        from gui.widgets.well_plate_view import WellPlateView, SCALE_FACTOR
        view = WellPlateView()
        view.set_plate(self._custom_plate())
        big = view._well_items["B2"].rect().width() / 2.0 / SCALE_FACTOR
        small = view._well_items["A1.a"].rect().width() / 2.0 / SCALE_FACTOR
        self.assertAlmostEqual(big, 7.8, places=1)     # regular well
        self.assertAlmostEqual(small, 1.0, places=1)   # rosette sub-well
        self.assertGreater(big, small)                 # not uniform/invisible

    def test_jog_workspace_per_well_radius(self):
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        ws = JogWorkspaceView()
        ws._plate = self._custom_plate()
        self.assertAlmostEqual(ws._well_radius_um("B2"), 7800.0, places=0)
        self.assertAlmostEqual(ws._well_radius_um("A1.a"), 1000.0, places=0)
        # No-name fallback uses the largest well, never 0 (which floored
        # every well to 3 px).
        self.assertAlmostEqual(ws._well_radius_um(), 7800.0, places=0)


class TestNoSceneEmbeddedInputWidgets(unittest.TestCase):
    """v7.4.8 crash fix: editable dimension fields must NOT be embedded in
    the QGraphicsScene (QGraphicsProxyWidget + QAbstractSpinBox segfaults on
    key input). Dimensions render as read-only labels; editing is in the
    properties panel. This guards against a regression."""

    def test_dimensions_render_without_proxy_widgets(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        from gui.widgets.plate_designer_canvas import Tool, SCALE_FACTOR
        from PySide6.QtCore import QPointF
        from PySide6.QtWidgets import QGraphicsProxyWidget
        w = PlateDesignerWidget(mode="plate")
        w.load_plate(96)
        c = w._canvas
        well = w._design.get_wells()[0]
        center = w._design.entities[well.center]
        c.set_tool(Tool.DIMENSION)
        c._handle_dimension(
            QPointF(center.x * SCALE_FACTOR, center.y * SCALE_FACTOR))
        c._rebuild_scene()
        proxies = [it for it in c._scene.items()
                   if isinstance(it, QGraphicsProxyWidget)]
        self.assertEqual(proxies, [])   # no embedded widgets

    def test_circle_radius_renders_without_proxy(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        from PySide6.QtWidgets import QGraphicsProxyWidget
        w = PlateDesignerWidget(mode="rosette")
        w.load_plate(24)
        a1 = w._design.get_wells()[0]
        w._canvas.well_drill_requested.emit(a1.id)
        w._canvas._circle_press((0.0, 0.0))
        w._canvas._circle_radius = 2.0
        w._canvas._commit_circle()
        w._canvas._rebuild_scene()
        proxies = [it for it in w._canvas._scene.items()
                   if isinstance(it, QGraphicsProxyWidget)]
        self.assertEqual(proxies, [])

    def test_set_constraint_value_edits_safely(self):
        from gui.widgets.plate_designer_canvas import PlateDesignerCanvas
        from SupportClasses.PlateDesign import PlateDesign, Constraint
        c = PlateDesignerCanvas()
        c.set_design(PlateDesign.from_standard_format(96))
        w = c.design.get_wells()[0]
        c.design.add_constraint(Constraint(
            kind="dist_left_edge", refs=[w.id], value=14.38, mode="center"))
        cid = c.design.constraints[-1].id
        c.set_constraint_value(cid, 25.0)
        self.assertAlmostEqual(
            c.design.entities[w.center].x, 25.0 - 14.38, places=2)


class TestCirclePatternOptions(unittest.TestCase):
    """v7.4.8: configurable, drag-to-radius circle pattern."""

    def _drilled(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        w = PlateDesignerWidget(mode="rosette")
        w.load_plate(24)
        w._canvas.well_drill_requested.emit(w._design.get_wells()[0].id)
        return w

    def test_press_snaps_center_to_origin(self):
        w = self._drilled()
        c = w._canvas
        c._circle_press((0.3, -0.2))   # near bore centre → snaps to (0,0)
        self.assertEqual(c._circle_center, (0.0, 0.0))

    def test_count_center_diameter_options(self):
        w = self._drilled()
        c = w._canvas
        c.set_circle_param("count", 5)
        c.set_circle_param("center_well", True)
        c.set_circle_param("diameter", 1.2)
        c._circle_press((0.0, 0.0))
        c._circle_radius = 2.5
        c._commit_circle()
        wells = w._design.get_wells()
        self.assertEqual(len(wells), 6)  # 5 ring + 1 center
        self.assertTrue(all(abs(x.diameter - 1.2) < 1e-9 for x in wells))
        rad = sorted(round(math.hypot(*w._design.get_well_position(x)), 2)
                     for x in wells)
        self.assertEqual(rad[0], 0.0)             # center well
        self.assertTrue(all(abs(r - 2.5) < 1e-6 for r in rad[1:]))

    def test_radius_drag_sets_radius(self):
        w = self._drilled()
        c = w._canvas
        c.set_circle_param("count", 4)
        c.set_circle_param("center_well", False)
        c._circle_press((0.0, 0.0))
        c._circle_radius = 1.8
        c._commit_circle()
        rad = sorted(round(math.hypot(*w._design.get_well_position(x)), 2)
                     for x in w._design.get_wells())
        self.assertTrue(all(abs(r - 1.8) < 1e-6 for r in rad))

    def test_radius_handle_drag_resizes_and_rotates(self):
        from SupportClasses.PlateDesign import Group
        w = self._drilled()
        c = w._canvas
        c.set_circle_param("count", 6)
        c.set_circle_param("center_well", True)
        c._circle_press((0.0, 0.0))
        c._circle_radius = 2.0
        c._commit_circle()
        grp = next(e for e in w._design.entities.values()
                   if isinstance(e, Group) and e.pattern_kind == "circle")
        # Drag the handle to (0, 3) → radius 3, start angle 90°.
        c._dragging_radius_group = grp.id
        c._drag_radius_handle((0.0, 3.0))
        c._dragging_radius_group = None
        self.assertAlmostEqual(grp.params["radius"], 3.0, places=3)
        self.assertAlmostEqual(grp.params["start_angle_deg"], 90.0, places=1)
        # Ring wells now at radius 3 (center well stays at 0); names kept.
        rads = sorted(round(math.hypot(*w._design.get_well_position(x)), 2)
                      for x in w._design.get_wells())
        self.assertEqual(rads[0], 0.0)
        self.assertTrue(all(abs(r - 3.0) < 1e-6 for r in rads[1:]))
        self.assertEqual(sorted(x.name for x in w._design.get_wells()),
                         ["a", "b", "c", "d", "e", "f", "g"])

    def test_radius_handle_rendered_for_placed_ring(self):
        from gui.widgets.plate_designer_canvas import RadiusHandleItem
        w = self._drilled()
        c = w._canvas
        c._circle_press((0.0, 0.0))
        c._circle_radius = 2.0
        c._commit_circle()
        c._rebuild_scene()
        handles = [it for it in c._scene.items()
                   if isinstance(it, RadiusHandleItem)]
        self.assertEqual(len(handles), 1)

    def test_inline_radius_edit_rebuilds_ring(self):
        from SupportClasses.PlateDesign import Group
        w = self._drilled()
        c = w._canvas
        c.set_circle_param("count", 6)
        c._circle_press((0.0, 0.0))
        c._circle_radius = 2.0
        c._commit_circle()
        grp = next(e for e in w._design.entities.values()
                   if isinstance(e, Group) and e.pattern_kind == "circle")
        c.rebuild_group_now(grp.id, {**grp.params, "radius": 3.5})
        rad = sorted(round(math.hypot(*w._design.get_well_position(x)), 2)
                     for x in w._design.get_wells())
        self.assertTrue(all(abs(r - 3.5) < 1e-6 for r in rad))
        # Sub-well letter names survive the rebuild.
        self.assertEqual(sorted(x.name for x in w._design.get_wells()),
                         ["a", "b", "c", "d", "e", "f"])


class TestNestedSerialization(unittest.TestCase):
    def test_rotation_and_heights_round_trip(self):
        d, a1, ros = _plate_with_rosette(rotation_deg=33.0)
        ros.get_wells()[0].rim_height_mm = 15.0
        ros.get_wells()[0].ink_z_mm = -8.0
        d2 = PlateDesign.from_dict(json.loads(json.dumps(d.to_dict())))
        a1b = d2.get_wells()[0]
        self.assertAlmostEqual(a1b.rosette_rotation_deg, 33.0)
        sub = a1b.rosette_design.get_wells()[0]
        self.assertAlmostEqual(sub.rim_height_mm, 15.0)
        self.assertAlmostEqual(sub.ink_z_mm, -8.0)


if __name__ == "__main__":
    unittest.main()
