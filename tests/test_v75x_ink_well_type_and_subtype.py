"""
v7.5.x — Ink WELL TYPE vs INK SUBTYPE split + pump ink-list filter.

Operator request:
  1. The pump setup must offer only printable INKS — service reagents
     (waste/wash/buffer/oil) must NOT appear in the per-pump ink checklist.
  2. Split the overloaded ``InkSpec.ink_type`` into a primary WELL TYPE
     (ink/wash/buffer/waste/oil, drives behavior) plus an informational
     INK SUBTYPE (granular material / cells / hydrogel monomer / … — only
     meaningful for the ``ink`` type).

Key invariant: ``ink_type`` still holds the four service strings, so the entire
service-well / prep / cleanup resolution path is unchanged. The single
behavioral branch on a material type (FlowPhysics cell-shear) is made
subtype-aware.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PhysicalModels import (
    InkSpec, NeedleSpec, SyringeSpec,
    WELL_TYPES, SERVICE_WELL_TYPES, INK_SUBTYPES,
    is_service_reagent, is_printable_ink_type, split_legacy_ink_type,
    well_role_for_ink_type, WellRole,
)
from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses import FlowPhysics


# ───────────────────────────────────────────────────────────────────
# Taxonomy constants
# ───────────────────────────────────────────────────────────────────

class TestTaxonomyConstants(unittest.TestCase):

    def test_well_types(self):
        self.assertEqual(set(WELL_TYPES),
                         {"ink", "wash", "buffer", "waste", "oil"})

    def test_service_well_types_are_the_non_ink_types(self):
        self.assertEqual(set(SERVICE_WELL_TYPES),
                         {"wash", "buffer", "waste", "oil"})
        self.assertNotIn("ink", SERVICE_WELL_TYPES)

    def test_ink_subtypes_match_operator_list(self):
        self.assertEqual(list(INK_SUBTYPES), [
            "granular material", "fluorescent stains", "cells",
            "hydrogel monomer", "media", "cell removal reagents",
            "ELISA Beads", "Growth Factor Beads",
        ])


# ───────────────────────────────────────────────────────────────────
# Service / printable helpers
# ───────────────────────────────────────────────────────────────────

class TestServiceAndPrintable(unittest.TestCase):

    def test_service_by_well_type(self):
        for t in ("wash", "waste", "buffer", "oil"):
            self.assertTrue(is_service_reagent("Some name", t), t)
            self.assertFalse(is_printable_ink_type(t), t)

    def test_ink_is_printable(self):
        self.assertFalse(is_service_reagent("Alginate", "ink"))
        self.assertTrue(is_printable_ink_type("ink"))

    def test_service_by_name_even_if_type_is_ink(self):
        # An ink literally named one of the service roles is still a service
        # reagent (mirrors the workflow-page printable-ink filters).
        self.assertTrue(is_service_reagent("Oil", "ink"))
        self.assertTrue(is_service_reagent("WASTE", "ink"))

    def test_case_insensitive_and_none_safe(self):
        self.assertTrue(is_service_reagent(None, " Wash "))
        self.assertFalse(is_service_reagent(None, None))   # empty → printable
        self.assertTrue(is_printable_ink_type(None))


# ───────────────────────────────────────────────────────────────────
# split_legacy_ink_type
# ───────────────────────────────────────────────────────────────────

class TestSplitLegacyInkType(unittest.TestCase):

    def test_legacy_material_types_become_ink_plus_subtype(self):
        self.assertEqual(split_legacy_ink_type("granular"),
                         ("ink", "granular material"))
        self.assertEqual(split_legacy_ink_type("cells"), ("ink", "cells"))
        self.assertEqual(split_legacy_ink_type("hydrogel"),
                         ("ink", "hydrogel monomer"))
        self.assertEqual(split_legacy_ink_type("media"), ("ink", "media"))

    def test_custom_becomes_ink_no_subtype(self):
        self.assertEqual(split_legacy_ink_type("custom"), ("ink", ""))

    def test_service_and_ink_pass_through(self):
        for t in ("ink", "wash", "buffer", "waste", "oil"):
            self.assertEqual(split_legacy_ink_type(t), (t, ""))

    def test_empty_or_none_defaults_to_ink(self):
        self.assertEqual(split_legacy_ink_type(""), ("ink", ""))
        self.assertEqual(split_legacy_ink_type(None), ("ink", ""))

    def test_existing_subtype_is_preserved_over_mapped_default(self):
        # If a subtype is already present it wins over the legacy mapping.
        self.assertEqual(split_legacy_ink_type("hydrogel", "ELISA Beads"),
                         ("ink", "ELISA Beads"))

    def test_case_insensitive(self):
        self.assertEqual(split_legacy_ink_type("HYDROGEL"),
                         ("ink", "hydrogel monomer"))


# ───────────────────────────────────────────────────────────────────
# InkSpec migration + persistence
# ───────────────────────────────────────────────────────────────────

class TestInkSpecMigration(unittest.TestCase):

    def test_from_dict_migrates_legacy_material(self):
        ink = InkSpec.from_dict({"name": "MSC Cells", "ink_type": "cells"})
        self.assertEqual(ink.ink_type, "ink")
        self.assertEqual(ink.ink_subtype, "cells")

    def test_from_dict_keeps_service_types(self):
        for t in ("wash", "waste", "buffer", "oil"):
            ink = InkSpec.from_dict({"name": f"{t} reagent", "ink_type": t})
            self.assertEqual(ink.ink_type, t, t)
            self.assertEqual(ink.ink_subtype, "", t)

    def test_from_dict_preserves_new_scheme(self):
        ink = InkSpec.from_dict({
            "name": "Beads", "ink_type": "ink", "ink_subtype": "ELISA Beads"})
        self.assertEqual(ink.ink_type, "ink")
        self.assertEqual(ink.ink_subtype, "ELISA Beads")

    def test_to_dict_round_trip_includes_subtype(self):
        ink = InkSpec(name="GF", ink_type="ink", ink_subtype="Growth Factor Beads")
        d = ink.to_dict()
        self.assertIn("ink_subtype", d)
        rt = InkSpec.from_dict(d)
        self.assertEqual(rt.ink_type, "ink")
        self.assertEqual(rt.ink_subtype, "Growth Factor Beads")

    def test_direct_construction_is_not_migrated(self):
        # Legacy callers/tests that build InkSpec(ink_type="hydrogel") directly
        # keep the literal value — migration happens only on from_dict (load).
        ink = InkSpec(name="Alginate", ink_type="hydrogel")
        self.assertEqual(ink.ink_type, "hydrogel")

    def test_migrated_cell_ink_still_maps_to_ink_role(self):
        ink = InkSpec.from_dict({"name": "MSC", "ink_type": "cells"})
        self.assertEqual(well_role_for_ink_type(ink.ink_type), WellRole.INK)

    def test_hardware_config_round_trip_migrates(self):
        cfg = HardwareConfig()
        cfg.ink_library["Old Gel"] = InkSpec.from_dict(
            {"name": "Old Gel", "ink_type": "hydrogel"})
        cfg.ink_library["Oil"] = InkSpec.from_dict(
            {"name": "Oil", "ink_type": "oil"})
        rt = HardwareConfig.from_dict(cfg.to_dict())
        self.assertEqual(rt.ink_library["Old Gel"].ink_type, "ink")
        self.assertEqual(rt.ink_library["Old Gel"].ink_subtype, "hydrogel monomer")
        self.assertEqual(rt.ink_library["Oil"].ink_type, "oil")


# ───────────────────────────────────────────────────────────────────
# FlowPhysics cell detection via subtype
# ───────────────────────────────────────────────────────────────────

class TestFlowPhysicsCellDetection(unittest.TestCase):

    def setUp(self):
        self.needle = NeedleSpec(gauge=22, od_um=720, id_um=410, wall_um=155)
        self.syringe = SyringeSpec(volume_uL=100, stroke_length_mm=30.0,
                                   barrel_id_mm=1.46)

    def _shear(self, ink):
        res = FlowPhysics.calculate_flow_safety(
            self.needle, self.syringe, ink, requested_flow_rate_uL_s=5.0)
        return res.wall_shear_stress_Pa

    def test_subtype_cells_triggers_shear_calc(self):
        # cell_diameter_um == 0 and ink_type == "ink" (migrated cell ink): the
        # subtype must still enable the wall-shear calculation.
        cell_ink = InkSpec(name="MSC", ink_type="ink", ink_subtype="cells",
                           cell_diameter_um=0.0, viscosity_cP=5.0)
        self.assertGreater(self._shear(cell_ink), 0.0)

    def test_non_cell_ink_no_shear_calc(self):
        plain = InkSpec(name="Alginate", ink_type="ink",
                        ink_subtype="hydrogel monomer",
                        cell_diameter_um=0.0, viscosity_cP=5.0)
        self.assertEqual(self._shear(plain), 0.0)

    def test_legacy_cells_type_still_triggers(self):
        legacy = InkSpec(name="MSC", ink_type="cells",
                         cell_diameter_um=0.0, viscosity_cP=5.0)
        self.assertGreater(self._shear(legacy), 0.0)


# ───────────────────────────────────────────────────────────────────
# InkEditorDialog (offscreen)
# ───────────────────────────────────────────────────────────────────

class TestInkEditorDialog(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _dialog(self, ink=None):
        from gui.pages.hardware_setup import InkEditorDialog
        return InkEditorDialog(ink=ink)

    def test_type_combo_offers_only_well_types(self):
        dlg = self._dialog()
        items = [dlg.type_combo.itemText(i)
                 for i in range(dlg.type_combo.count())]
        self.assertEqual(items, list(WELL_TYPES))

    def test_subtype_presets_present(self):
        dlg = self._dialog()
        items = [dlg.subtype_combo.itemText(i)
                 for i in range(dlg.subtype_combo.count())]
        for preset in INK_SUBTYPES:
            self.assertIn(preset, items)

    def test_get_ink_writes_subtype_for_ink(self):
        dlg = self._dialog()
        dlg.name_edit.setText("Beads")
        dlg.type_combo.setCurrentText("ink")
        dlg.subtype_combo.setCurrentText("ELISA Beads")
        ink = dlg.get_ink()
        self.assertEqual(ink.ink_type, "ink")
        self.assertEqual(ink.ink_subtype, "ELISA Beads")

    def test_switching_to_service_type_disables_and_clears_subtype(self):
        dlg = self._dialog()
        dlg.name_edit.setText("Buffer")
        dlg.type_combo.setCurrentText("ink")
        dlg.subtype_combo.setCurrentText("cells")
        dlg.type_combo.setCurrentText("buffer")     # → service type
        self.assertFalse(dlg.subtype_combo.isEnabled())
        ink = dlg.get_ink()
        self.assertEqual(ink.ink_type, "buffer")
        self.assertEqual(ink.ink_subtype, "")

    def test_edit_restores_migrated_subtype(self):
        # A loaded legacy "cells" ink (migrated) opens with type=ink+subtype.
        loaded = InkSpec.from_dict({"name": "MSC", "ink_type": "cells"})
        dlg = self._dialog(ink=loaded)
        self.assertEqual(dlg.type_combo.currentText(), "ink")
        self.assertEqual(dlg.subtype_combo.currentText(), "cells")
        self.assertTrue(dlg.subtype_combo.isEnabled())


# ───────────────────────────────────────────────────────────────────
# Pump ink-list filter (offscreen page)
# ───────────────────────────────────────────────────────────────────

class TestPumpInkFilter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _config(self):
        cfg = HardwareConfig()
        cfg.ink_library = {
            "Alginate": InkSpec(name="Alginate", ink_type="ink",
                                ink_subtype="hydrogel monomer"),
            "MSC": InkSpec(name="MSC", ink_type="ink", ink_subtype="cells"),
            "PBS wash": InkSpec(name="PBS wash", ink_type="wash"),
            "Sep buffer": InkSpec(name="Sep buffer", ink_type="buffer"),
            "Mineral oil": InkSpec(name="Mineral oil", ink_type="oil"),
            "Waste bin": InkSpec(name="Waste bin", ink_type="waste"),
        }
        return cfg

    def test_pump_ink_names_excludes_service_reagents(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage.__new__(HardwareSetupPage)
        page._config = self._config()
        names = page._pump_ink_names()
        self.assertEqual(set(names), {"Alginate", "MSC"})

    def test_pump_checklist_shows_only_inks(self):
        # Full page build + set_hardware_config → the per-pump checklist only
        # contains printable inks.
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        page.set_hardware_config(self._config())
        for pid, pw in page._pump_widgets.items():
            listed = [pw.ink_list.item(i).text()
                      for i in range(pw.ink_list.count())]
            self.assertEqual(set(listed), {"Alginate", "MSC"}, pid)

    def test_legacy_service_pump_assignment_no_spurious_warning(self):
        # A pre-v7.5.x config that assigned a service reagent to a pump must
        # not log a mismatch warning on every load (the service reagent is
        # intentionally dropped; only the printable inks are restored).
        from PySide6.QtWidgets import QApplication  # noqa: F401 (app already up)
        from gui.pages.hardware_setup import HardwareSetupPage, PumpChannelConfig
        cfg = self._config()
        # P1 assigned BOTH a printable ink and a service reagent (legacy).
        cfg.pumps["P1"] = PumpChannelConfig(
            pump_id="P1", enabled=True,
            inks=[InkSpec(name="Alginate"), InkSpec(name="PBS wash")],
        )
        import logging

        class _Capture(logging.Handler):
            def __init__(self):
                super().__init__()
                self.records = []

            def emit(self, record):
                self.records.append(record.getMessage())

        cap = _Capture()
        hs_logger = logging.getLogger("gui.pages.hardware_setup")
        hs_logger.addHandler(cap)
        try:
            page = HardwareSetupPage()
            page.set_hardware_config(cfg)
        finally:
            hs_logger.removeHandler(cap)
        self.assertFalse([m for m in cap.records if "ink mismatch" in m],
                         f"unexpected ink-mismatch warning: {cap.records}")
        # Only the printable ink is checked on the pump.
        self.assertEqual(set(page._pump_widgets["P1"].get_selected_ink_names()),
                         {"Alginate"})


if __name__ == "__main__":
    unittest.main()
