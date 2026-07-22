"""Tests for the rosette ink/reagent PICKUP well fix (v7.5.x).

Bug: on a plate with a rosette, the ink pickup drove to the rosette PARENT
centre instead of the assigned ink SUB-WELL. ``ink_locations`` is append-only,
so a plain well pinned before it became a rosette left a stale, now-flattened
PARENT name (e.g. ``"A2"``) in the list — usually at index 0 — while every
reagent-pickup resolver took ``wells[0]``. The calibrated ``well_positions`` map
re-adds the parent at the sub-well CENTROID, so pickup silently dipped at the
rosette centre.

Fix:
- ``resolve_pickup_well`` prefers a real (leaf) well present in the compiled
  ``plate.well_names`` over a flattened parent (falls back to ``[0]`` legacy).
- ``HardwareConfig._drop_redundant_parents`` self-cleans a redundant parent in
  ``assign_wells_to_ink`` and the ``from_dict``→``_prune_ink_locations`` migration.
- ``HardwareConfig.clear_all_ink_locations`` gives a hard fresh-start reset.
"""

from __future__ import annotations

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# A QApplication is needed for the end-to-end page test.
from PySide6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.PlateDesign import PlateDesign
from SupportClasses.WellPlate import WellPlate
from SupportClasses.PhysicalModels import NeedleSpec, InkSpec
from SupportClasses.HardwareConfig import (
    HardwareConfig, PumpChannelConfig, SyringeSpec,
)
from gui.pages.workflows._reagent_prep import (
    resolve_pickup_well, service_well_names, resolve_service_positions,
)


def _rosette_plate() -> WellPlate:
    """A 24-well plate with a 3-sub-well rosette on the FIRST well (A1).

    After ``compile()`` the parent "A1" is dropped and A1.a/b/c are present —
    the same shape as the real "rosette in A2" bug (A1 stands in for A2).
    """
    d = PlateDesign.from_standard_format(24)
    a1 = d.get_wells()[0]
    ros = PlateDesign.blank_rosette(bore_radius_mm=7.0)
    ros.add_well(x=3.0, y=0.0, diameter=1.0, name="a", naming_scheme="MANUAL")
    ros.add_well(x=0.0, y=3.0, diameter=1.0, name="b", naming_scheme="MANUAL")
    ros.add_well(x=0.0, y=0.0, diameter=1.0, name="c", naming_scheme="MANUAL")
    a1.rosette_design = ros
    return d.compile()


class TestResolvePickupWell(unittest.TestCase):
    def setUp(self):
        self.plate = _rosette_plate()
        # Sanity: the compiled rosette plate has the sub-wells but NOT the
        # bare parent — the exact condition that trips the bug.
        self.assertIn("A1.a", self.plate.well_names)
        self.assertNotIn("A1", self.plate.well_names)

    def test_prefers_subwell_over_stale_parent(self):
        # Stale parent at index 0 → the real sub-well wins.
        self.assertEqual(resolve_pickup_well(["A1", "A1.a"], self.plate), "A1.a")

    def test_single_subwell_unchanged(self):
        self.assertEqual(resolve_pickup_well(["A1.a"], self.plate), "A1.a")

    def test_lone_parent_falls_back_to_legacy(self):
        # No real leaf in the list → keep legacy [0] (no worse than before).
        self.assertEqual(resolve_pickup_well(["A1"], self.plate), "A1")

    def test_no_plate_is_byte_identical_legacy(self):
        self.assertEqual(resolve_pickup_well(["A1", "A1.a"], None), "A1")

    def test_plain_well_unchanged(self):
        self.assertEqual(resolve_pickup_well(["B2"], self.plate), "B2")

    def test_parent_name_real_on_nonrosette_plate(self):
        # On a plain 96-well plate "A2" IS a real well → returned unchanged.
        self.assertEqual(
            resolve_pickup_well(["A2"], WellPlate.from_format(96)), "A2")

    def test_empty_or_none(self):
        self.assertIsNone(resolve_pickup_well([], self.plate))
        self.assertIsNone(resolve_pickup_well(None, self.plate))


class TestServiceWellResolution(unittest.TestCase):
    def _hw(self) -> HardwareConfig:
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Buffer", ink_type="buffer"))
        # Simulate the stale append-only state directly (bypass self-clean).
        cfg.ink_locations = {"Buffer": ["A1", "A1.a"]}
        return cfg

    def test_service_prefers_subwell_with_plate(self):
        names = service_well_names(self._hw(), _rosette_plate())
        self.assertEqual(names.get("buffer"), "A1.a")

    def test_service_legacy_without_plate(self):
        names = service_well_names(self._hw())
        self.assertEqual(names.get("buffer"), "A1")

    def test_resolve_service_positions_uses_subwell(self):
        plate = _rosette_plate()
        wp = {"A1": (100.0, 200.0), "A1.a": (111.0, 222.0)}
        positions, missing = resolve_service_positions(self._hw(), wp, plate)
        self.assertEqual(positions.get("buffer"), (111.0, 222.0))
        self.assertNotIn("buffer", missing)


class TestHardwareConfigHygiene(unittest.TestCase):
    def test_drop_redundant_parents_helper(self):
        self.assertEqual(
            HardwareConfig._drop_redundant_parents(["A2", "A2.a", "B2"]),
            ["A2.a", "B2"])
        # A bare parent with no sub-well of it present is kept.
        self.assertEqual(
            HardwareConfig._drop_redundant_parents(["A2", "B2"]),
            ["A2", "B2"])
        # Two sub-wells of the same parent both survive.
        self.assertEqual(
            HardwareConfig._drop_redundant_parents(["A2", "A2.a", "A2.b"]),
            ["A2.a", "A2.b"])

    def test_assign_self_cleans_stale_parent(self):
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Ink", ink_type="hydrogel"))
        cfg.assign_wells_to_ink("Ink", ["A2"])
        cfg.assign_wells_to_ink("Ink", ["A2.a"])   # append → self-clean
        self.assertEqual(cfg.ink_locations["Ink"], ["A2.a"])

    def test_assign_cross_ink_one_reagent_per_well_preserved(self):
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Ink1", ink_type="hydrogel"))
        cfg.add_ink(InkSpec(name="Ink2", ink_type="hydrogel"))
        cfg.assign_wells_to_ink("Ink1", ["A2.a"])
        cfg.assign_wells_to_ink("Ink2", ["A2.a"])   # steals from Ink1
        self.assertNotIn("Ink1", cfg.ink_locations)
        self.assertEqual(cfg.ink_locations["Ink2"], ["A2.a"])

    def test_prune_migration_drops_stale_parent(self):
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Ink", ink_type="hydrogel"))
        cfg.ink_locations = {"Ink": ["A2", "A2.a"]}   # legacy append-only state
        cfg._prune_ink_locations()                    # runs on from_dict load
        self.assertEqual(cfg.ink_locations["Ink"], ["A2.a"])

    def test_clear_all_ink_locations(self):
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Ink", ink_type="hydrogel"))
        cfg.assign_wells_to_ink("Ink", ["A2.a"])
        self.assertTrue(cfg.ink_locations)
        cfg.clear_all_ink_locations()
        self.assertEqual(cfg.ink_locations, {})


class TestQuickPrintInkSourceEndToEnd(unittest.TestCase):
    """The reported failure, end to end: the Quick Print page resolves the ink
    source to the SUB-WELL, not the rosette-parent centroid."""

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.print_floor_violation.return_value = False
        ctrl.print_height_to_zref.return_value = -25.0
        ctrl.print_z_dir.return_value = -1.0
        ctrl.safe_travel_to.return_value = True
        ctrl._pending_per_axis_max_feedrate = None
        from SupportClasses.StageController import StageController as _SC
        ctrl.get_max_xy_speed_um_s.side_effect = (
            lambda: _SC.get_max_xy_speed_um_s(ctrl))
        ctrl.get_max_z_feedrate_mm_min.side_effect = (
            lambda: _SC.get_max_z_feedrate_mm_min(ctrl))
        return ctrl

    def _hw(self) -> HardwareConfig:
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        ink = InkSpec(name="GFP", ink_type="hydrogel")
        syr = SyringeSpec(volume_uL=250, stroke_length_mm=30.0)
        cfg.pumps = {
            "P1": PumpChannelConfig(pump_id="P1", syringe=syr, inks=[ink],
                                    enabled=True),
            "P2": PumpChannelConfig(pump_id="P2"),
            "P3": PumpChannelConfig(pump_id="P3"),
        }
        cfg.add_ink(ink)
        # Simulate the STALE append-only state directly (parent + sub-well).
        cfg.ink_locations = {"GFP": ["A1", "A1.a"]}
        return cfg

    def test_ink_source_resolves_subwell_not_centroid(self):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        plate = _rosette_plate()
        # Calibrated map contains BOTH the sub-wells and the re-added PARENT
        # centroid (the wrong target the bug used).
        wells = {
            "A1.a": (1000.0, 2000.0),
            "A1.b": (3000.0, 2000.0),
            "A1.c": (2000.0, 2000.0),
            "A1": (2000.0, 2000.0),   # parent centroid — must NOT be chosen
        }
        page = QuickPrintWorkflowPage(self._ctrl(), settings=None)
        page.set_hardware_config(self._hw())
        page.set_calibration_data(plate, wells, 5.0)
        self.assertEqual(page._selected_ink(), "GFP")
        self.assertEqual(page._ink_source_well(), "A1.a")        # not "A1"
        self.assertEqual(page._ink_source_pos(), (1000.0, 2000.0))  # not centroid


class TestClearAllWellsButton(unittest.TestCase):
    """The Hardware Setup → Ink "Clear all wells" hard reset."""

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page_with_assignments(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        pg = HardwareSetupPage()
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Red gel", ink_type="hydrogel"))
        cfg.add_ink(InkSpec(name="Mineral oil", ink_type="oil"))
        cfg.assign_wells_to_ink("Red gel", ["A1"])
        cfg.assign_wells_to_ink("Mineral oil", ["B1"])
        pg.set_config(cfg)
        return pg

    def test_clear_all_wipes_assignments_on_confirm(self):
        from PySide6.QtWidgets import QMessageBox
        pg = self._page_with_assignments()
        self.assertTrue(pg._config.ink_locations)
        with patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
            pg._loc_clear_all()
        self.assertEqual(pg._config.ink_locations, {})
        self.assertIn("No reagent locations", pg._loc_summary.text())

    def test_clear_all_cancelled_keeps_assignments(self):
        from PySide6.QtWidgets import QMessageBox
        pg = self._page_with_assignments()
        with patch.object(QMessageBox, "question", return_value=QMessageBox.No):
            pg._loc_clear_all()
        self.assertEqual(
            pg._config.ink_locations, {"Red gel": ["A1"], "Mineral oil": ["B1"]})


if __name__ == "__main__":
    unittest.main()
