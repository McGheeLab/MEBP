"""
v7.5.x — Ink / Reagent Location Assignments (Hardware Setup → Ink).

Covers:
  * HardwareConfig.ink_locations: round-trip, one-reagent-per-well,
    reverse map, clear/unassign/rename, prune-on-load.
  * well_role_for_ink_type() mapping (wash/waste/buffer → roles, oil +
    material types → INK).
  * seed_assignments_from_ink_locations() non-destructive auto-fill of a
    WellSetupModel (PRINT wells untouched; EMPTY wells filled; off-plate
    wells skipped; INK wells receive ink_name).
  * Offscreen Hardware Setup page-build smoke: the Reagent Locations
    widgets exist, the active plate loads, and recolor runs.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.PhysicalModels import (
    InkSpec, WellRole, well_role_for_ink_type,
)
from SupportClasses.WellSetup import (
    WellSetupModel, seed_assignments_from_ink_locations,
)


def _cfg_with_inks() -> HardwareConfig:
    cfg = HardwareConfig()
    cfg.add_ink(InkSpec(name="PBS wash", ink_type="wash"))
    cfg.add_ink(InkSpec(name="Sep buffer", ink_type="buffer"))
    cfg.add_ink(InkSpec(name="Mineral oil", ink_type="oil"))
    cfg.add_ink(InkSpec(name="Alginate", ink_type="hydrogel"))
    return cfg


# ───────────────────────────────────────────────────────────────────
# Backend: HardwareConfig.ink_locations
# ───────────────────────────────────────────────────────────────────

class TestInkLocationsModel(unittest.TestCase):

    def test_assign_and_reverse_map(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B2"])
        cfg.assign_wells_to_ink("Alginate", ["A1"])
        self.assertEqual(cfg.ink_locations["PBS wash"], ["B1", "B2"])
        rev = cfg.well_reagent_map
        self.assertEqual(rev["B1"], "PBS wash")
        self.assertEqual(rev["A1"], "Alginate")

    def test_one_reagent_per_well_moves(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B2"])
        # Re-assigning B2 to a different reagent moves it off "PBS wash".
        cfg.assign_wells_to_ink("Sep buffer", ["B2"])
        self.assertEqual(cfg.ink_locations["PBS wash"], ["B1"])
        self.assertEqual(cfg.ink_locations["Sep buffer"], ["B2"])
        self.assertEqual(cfg.well_reagent_map["B2"], "Sep buffer")

    def test_replace_vs_append(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1"])
        cfg.assign_wells_to_ink("PBS wash", ["B2"])  # append (default)
        self.assertEqual(cfg.ink_locations["PBS wash"], ["B1", "B2"])
        cfg.assign_wells_to_ink("PBS wash", ["C3"], replace=True)
        self.assertEqual(cfg.ink_locations["PBS wash"], ["C3"])

    def test_dedup_and_blank_drop(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B1", "", "B2"])
        self.assertEqual(cfg.ink_locations["PBS wash"], ["B1", "B2"])

    def test_clear_and_unassign(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B2"])
        cfg.unassign_well("B1")
        self.assertEqual(cfg.ink_locations["PBS wash"], ["B2"])
        cfg.clear_ink_location("PBS wash")
        self.assertNotIn("PBS wash", cfg.ink_locations)

    def test_unassign_last_well_drops_entry(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1"])
        cfg.unassign_well("B1")
        self.assertNotIn("PBS wash", cfg.ink_locations)

    def test_rename_carries_location(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B2"])
        cfg.rename_ink_location("PBS wash", "PBS 1x")
        self.assertNotIn("PBS wash", cfg.ink_locations)
        self.assertEqual(cfg.ink_locations["PBS 1x"], ["B1", "B2"])

    def test_roundtrip_serialization(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B2"])
        cfg.assign_wells_to_ink("Alginate", ["A1"])
        restored = HardwareConfig.from_dict(cfg.to_dict())
        self.assertEqual(restored.ink_locations["PBS wash"], ["B1", "B2"])
        self.assertEqual(restored.ink_locations["Alginate"], ["A1"])

    def test_empty_lists_not_serialized(self):
        cfg = _cfg_with_inks()
        cfg.ink_locations["PBS wash"] = []
        self.assertNotIn("PBS wash", cfg.to_dict().get("ink_locations", {}))

    def test_prune_drops_unknown_ink_on_load(self):
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1"])
        data = cfg.to_dict()
        # Inject a location for an ink that is not in the library.
        data["ink_locations"]["Ghost reagent"] = ["C4"]
        restored = HardwareConfig.from_dict(data)
        self.assertIn("PBS wash", restored.ink_locations)
        self.assertNotIn("Ghost reagent", restored.ink_locations)

    def test_string_value_tolerated_on_load(self):
        cfg = _cfg_with_inks()
        data = cfg.to_dict()
        data["ink_locations"] = {"PBS wash": "B1"}  # single string, not list
        restored = HardwareConfig.from_dict(data)
        self.assertEqual(restored.ink_locations["PBS wash"], ["B1"])


# ───────────────────────────────────────────────────────────────────
# Type → role mapping
# ───────────────────────────────────────────────────────────────────

class TestWellRoleForInkType(unittest.TestCase):

    def test_service_types(self):
        self.assertEqual(well_role_for_ink_type("wash"), WellRole.WASH)
        self.assertEqual(well_role_for_ink_type("waste"), WellRole.WASTE)
        self.assertEqual(well_role_for_ink_type("buffer"), WellRole.BUFFER)

    def test_oil_and_materials_are_ink(self):
        for t in ("oil", "ink", "hydrogel", "cells", "media",
                  "granular", "custom", "", None, "UNKNOWN"):
            self.assertEqual(well_role_for_ink_type(t), WellRole.INK, t)

    def test_case_insensitive(self):
        self.assertEqual(well_role_for_ink_type("WASH"), WellRole.WASH)
        self.assertEqual(well_role_for_ink_type(" Buffer "), WellRole.BUFFER)


# ───────────────────────────────────────────────────────────────────
# Auto-fill of the per-print WellSetupModel
# ───────────────────────────────────────────────────────────────────

class TestSeedAssignments(unittest.TestCase):

    def setUp(self):
        self.cfg = _cfg_with_inks()
        self.cfg.assign_wells_to_ink("PBS wash", ["B1"])
        self.cfg.assign_wells_to_ink("Sep buffer", ["B2"])
        self.cfg.assign_wells_to_ink("Mineral oil", ["B3"])
        self.cfg.assign_wells_to_ink("Alginate", ["C1"])
        self.model = WellSetupModel(24)

    def _seed(self, **kw):
        return seed_assignments_from_ink_locations(
            self.model, self.cfg.ink_locations, self.cfg.ink_library, **kw)

    def test_roles_derived_from_type(self):
        self._seed()
        self.assertEqual(self.model.get_assignment("B1").role, WellRole.WASH)
        self.assertEqual(self.model.get_assignment("B2").role, WellRole.BUFFER)
        self.assertEqual(self.model.get_assignment("B3").role, WellRole.INK)  # oil → INK
        self.assertEqual(self.model.get_assignment("C1").role, WellRole.INK)

    def test_ink_name_set_for_ink_role(self):
        self._seed()
        self.assertEqual(self.model.get_assignment("C1").ink_name, "Alginate")
        self.assertEqual(self.model.get_assignment("B3").ink_name, "Mineral oil")

    def test_non_destructive_keeps_print_well(self):
        # Pre-assign C1 as a PRINT well; seeding must not clobber it.
        self.model.set_role(["C1"], WellRole.PRINT)
        changed = self._seed()
        self.assertEqual(self.model.get_assignment("C1").role, WellRole.PRINT)
        self.assertNotIn("C1", changed)
        # The empty service wells still got filled.
        self.assertIn("B1", changed)

    def test_destructive_overwrites(self):
        self.model.set_role(["C1"], WellRole.PRINT)
        self._seed(destructive=True)
        self.assertEqual(self.model.get_assignment("C1").role, WellRole.INK)

    def test_off_plate_well_skipped(self):
        self.cfg.assign_wells_to_ink("PBS wash", ["Z99"])  # not a 24-well name
        changed = self._seed()
        self.assertNotIn("Z99", changed)

    def test_empty_locations_noop(self):
        self.assertEqual(
            seed_assignments_from_ink_locations(self.model, None), [])
        self.assertEqual(
            seed_assignments_from_ink_locations(self.model, {}), [])


# ───────────────────────────────────────────────────────────────────
# Offscreen Hardware Setup page smoke
# ───────────────────────────────────────────────────────────────────

class TestHardwareSetupReagentLocationsUI(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        return HardwareSetupPage()

    def test_widgets_exist_and_plate_loaded(self):
        pg = self._page()
        self.assertTrue(hasattr(pg, "_loc_plate_view"))
        self.assertTrue(hasattr(pg, "_loc_ink_combo"))
        # Default config is a 24-well plate → 24 wells loaded into the view.
        self.assertIsNotNone(pg._loc_plate)
        self.assertEqual(len(pg._loc_plate.well_names), 24)

    def test_config_with_locations_colors_and_combo(self):
        pg = self._page()
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1", "B2"])
        cfg.assign_wells_to_ink("Alginate", ["A1"])
        pg.set_config(cfg)
        # Combo populated from the ink library.
        self.assertEqual(pg._loc_ink_combo.count(), 4)
        # Recolor runs without error and reflects the reverse map.
        self.assertEqual(pg._config.well_reagent_map["B1"], "PBS wash")
        # Summary mentions an assigned reagent.
        self.assertIn("PBS wash", pg._loc_summary.text())

    def test_remove_ink_clears_location(self):
        pg = self._page()
        cfg = _cfg_with_inks()
        cfg.assign_wells_to_ink("PBS wash", ["B1"])
        pg.set_config(cfg)
        # Simulate removing the ink via the config + the page's clear path.
        pg._config.remove_ink("PBS wash")
        pg._config.clear_ink_location("PBS wash")
        self.assertNotIn("PBS wash", pg._config.ink_locations)


if __name__ == "__main__":
    unittest.main()
