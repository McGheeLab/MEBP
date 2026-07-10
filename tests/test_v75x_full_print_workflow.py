"""test_v75x_full_print_workflow.py — Full Print workflow tile.

v7.5.x relocated the former top-level "Printing" mode (Setup / Monitor /
Results) into the Workflows mode as a "Full Print" tile, and modernized its
build path. These tests pin:

  * the tile is registered + enabled in the WORKFLOWS registry;
  * ``FullPrintWorkflowPage`` wraps a ``PrintingModePage`` and exposes the
    accessor surface ``gui/app.py`` reaches (setup/monitor/results + switch_to_*
    + print_manager) and the workflow-page contract (back_requested,
    sub_page_changed, get_context_widget, set_calibration_data, …);
  * ``set_common_print_settings`` forwards wrapper → wizard → legacy;
  * the build-path modernization: per-object ``path_segments`` splitting,
    µL-native retract/prime dicts, and the Quick-Print-profile import mapping.

Offscreen QApplication only — no hardware / Qt event loop.
"""

import os
import sys
import types
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication


def _mock_controller():
    ctrl = MagicMock()
    ctrl.is_xy_connected = False
    ctrl.is_zp_connected = False
    ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    ctrl.plate_axis_sign.return_value = (1.0, 1.0)
    ctrl.print_z_dir.return_value = 1.0
    ctrl.get_plate_top_z.return_value = None
    ctrl.get_plate_bottom_z.return_value = None
    ctrl.get_max_xy_speed_um_s.return_value = 50000.0   # 50 mm/s
    ctrl.get_max_z_feedrate_mm_min.return_value = 600.0  # 10 mm/s
    return ctrl


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class TestTileRegistration(_Base):
    def test_full_print_tile_registered_and_enabled(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        tiles = {t.workflow_id: t for t in WORKFLOWS}
        self.assertIn("full_print", tiles)
        self.assertTrue(tiles["full_print"].enabled)
        self.assertEqual(tiles["full_print"].title, "Full Print")

    def test_quick_print_still_present(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        ids = {t.workflow_id for t in WORKFLOWS}
        self.assertIn("quick_print", ids)

    def test_workflows_mode_imports_full_print_page(self):
        # The dispatch branch needs the class importable in workflows_mode.
        import gui.pages.workflows_mode as wm
        self.assertTrue(hasattr(wm, "FullPrintWorkflowPage"))


class TestWrapper(_Base):
    def setUp(self):
        from gui.pages.workflows.full_print_workflow import FullPrintWorkflowPage
        self.page = FullPrintWorkflowPage(_mock_controller(), None)

    def test_exposes_sub_page_accessors(self):
        self.assertIsNotNone(self.page.setup_page)
        self.assertIsNotNone(self.page.monitor_page)
        self.assertIsNotNone(self.page.results_page)

    def test_switch_helpers_callable(self):
        # Should not raise; they delegate to the inner mode page.
        self.page.switch_to_monitor()
        self.page.switch_to_results()
        self.page.switch_to_setup()

    def test_print_manager_resolves_via_setup_page(self):
        # The exact app.py access path: full_print_page.setup_page.print_manager
        sp = self.page.setup_page
        self.assertTrue(hasattr(sp, "print_manager"))
        self.assertIsNotNone(sp.print_manager)

    def test_has_workflow_contract(self):
        for attr in ("back_requested", "sub_page_changed",
                     "set_calibration_data", "set_hardware_config",
                     "set_common_print_settings", "on_status_update",
                     "get_page_title", "get_context_widget"):
            self.assertTrue(hasattr(self.page, attr), attr)

    def test_get_context_widget_none(self):
        # PrintingModePage embeds context internally → None here so the external
        # left context box stays hidden (parity with the old Printing mode).
        self.assertIsNone(self.page.get_context_widget())

    def test_set_common_print_settings_forwards_to_legacy(self):
        sentinel = object()
        self.page.set_common_print_settings(sentinel)
        legacy = self.page.setup_page._legacy
        self.assertIs(legacy._common_print_settings, sentinel)

    def test_sub_page_changed_reemitted(self):
        seen = []
        self.page.sub_page_changed.connect(lambda i: seen.append(i))
        self.page.switch_to_monitor()
        self.assertTrue(seen, "inner sub_page_changed should re-emit")


class TestBuildPathModernization(_Base):
    def setUp(self):
        from gui.pages.workflows.full_print_workflow import FullPrintWorkflowPage
        self.page = FullPrintWorkflowPage(_mock_controller(), None)
        self.legacy = self.page.setup_page._legacy

    def test_get_settings_populates_uL_dicts(self):
        # The Finalize retract/prime spins are µL-labelled; _get_settings must
        # populate the µL-native dicts build_well_plate_job reads FIRST.
        for pid, spin in self.legacy._prime_spins.items():
            spin.setValue(1.5)
        s = self.legacy._get_settings()
        self.assertTrue(hasattr(s, "prime_amounts_uL"))
        for pid in self.legacy._prime_spins:
            self.assertAlmostEqual(s.prime_amounts_uL[pid], 1.5, places=3)
            # legacy mm dict kept in sync (harmless; µL path wins downstream)
            self.assertAlmostEqual(s.prime_amounts[pid], 1.5, places=3)

    def test_get_settings_consumes_stashed_hop(self):
        self.legacy._qp_intra_well_hop_z_mm = 2.5
        s = self.legacy._get_settings()
        self.assertAlmostEqual(s.intra_well_hop_z_mm, 2.5, places=3)

    def test_path_segments_split_per_object(self):
        # Two objects → two independent segments, each with its own offset.
        tab = self.legacy.tab_objects
        tab._objects = [
            {"name": "a", "position": (1.0, 0.0)},
            {"name": "b", "position": (0.0, 2.0)},
        ]
        # Patch point generation to a known shape so we test the SPLIT, not the
        # geometry pipeline.
        self.legacy._single_object_to_points = types.MethodType(
            lambda self, obj: [(0.0, 0.0), (0.5, 0.0)], self.legacy)
        segs = self.legacy._get_path_segments_from_objects()
        self.assertEqual(len(segs), 2)
        # offsets applied per object
        self.assertEqual(segs[0][0], (1.0, 0.0))
        self.assertEqual(segs[1][0], (0.0, 2.0))

    def test_path_segments_empty_when_no_objects(self):
        self.legacy.tab_objects._objects = []
        self.legacy.tab_objects._current_file = None
        self.assertEqual(self.legacy._get_path_segments_from_objects(), [])


class TestQuickPrintProfileImport(_Base):
    def setUp(self):
        from gui.pages.workflows.full_print_workflow import FullPrintWorkflowPage
        self.page = FullPrintWorkflowPage(_mock_controller(), None)
        self.legacy = self.page.setup_page._legacy

    def test_speed_and_printz_exact(self):
        self.legacy._apply_quick_print_profile({"speed_pct": 40.0, "printz": 0.35})
        self.assertEqual(self.legacy.speed_scale_spin.value(), 40)
        self.assertAlmostEqual(self.legacy.print_height_spin.value(), 0.35, places=2)

    def test_extrusion_mod_semantic_adapter(self):
        # 0.5× extrusion → 50% fill (clamped 1..100).
        self.legacy._apply_quick_print_profile({"extrusion_mod": 0.5})
        self.assertEqual(self.legacy.volume_fraction_spin.value(), 50)
        self.legacy._apply_quick_print_profile({"extrusion_mod": 5.0})
        self.assertEqual(self.legacy.volume_fraction_spin.value(), 100)

    def test_line_retract_stashed_as_hop(self):
        self.legacy._apply_quick_print_profile({"line_retract": 1.75})
        self.assertAlmostEqual(self.legacy._qp_intra_well_hop_z_mm, 1.75, places=3)

    def test_globals_and_prep_not_imported(self):
        summary = self.legacy._apply_quick_print_profile({
            "speed_pct": 30.0, "g_settle": 0.4, "g_relief": 1.0,
            "prep": True, "ink": {"text": "Blue", "data": "blue"},
            "postclean": True,
        })
        self.assertIn("Not imported", summary)
        # global pump values must never round-trip into the page
        self.assertNotIn("g_settle", summary.split("Not imported")[0])

    def test_combo_token_decode_for_pump_target(self):
        # pump given as a {"text","data"} combo dict; preflow → prime on P-data.
        self.legacy._hardware_config = None  # derived flow falls back to 0.1
        self.legacy._apply_quick_print_profile({
            "pump": {"text": "P2", "data": "P2"}, "preflow": 0.5,
        })
        # P2 prime spin should now be > 0 (0.1 µL/s × 0.5 s = 0.05 µL)
        if "P2" in self.legacy._prime_spins:
            self.assertGreater(self.legacy._prime_spins["P2"].value(), 0.0)

    def test_combo_token_falls_back_to_text_when_data_none(self):
        # The Quick Print pump combo carries NO userData, so it serializes as
        # {"text": "P2", "data": None}. The prime must still target P2 (via the
        # text), not silently fall back to the first pump.
        if "P2" not in self.legacy._prime_spins:
            self.skipTest("rig has no P2 pump")
        # zero every prime spin first
        for sp in self.legacy._prime_spins.values():
            sp.setValue(0.0)
        self.legacy._hardware_config = None  # derived flow → 0.1 µL/s
        self.legacy._apply_quick_print_profile({
            "pump": {"text": "P2", "data": None}, "preflow": 0.5,
        })
        self.assertGreater(self.legacy._prime_spins["P2"].value(), 0.0)
        # P1 must be untouched (the bug applied the prime to the first pump).
        if "P1" in self.legacy._prime_spins:
            self.assertEqual(self.legacy._prime_spins["P1"].value(), 0.0)


if __name__ == "__main__":
    unittest.main()
