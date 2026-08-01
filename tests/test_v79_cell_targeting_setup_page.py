"""
v7.9 — Cell Targeting & Removal: the Setup tab + the embedded Well Survey viewer.

Covers ``gui/pages/workflows/cell_targeting_setup_panel.py`` (the per-bore program
table, target types, trypsin reagent) and the reworked
``gui/pages/workflows/cell_targeting_workflow.py`` (two tabs, ``_stage_busy``
including the embedded scan page, the separate absolute-vs-zero-ref travel
handlers, the mosaic-shift Start refusal, and the extra-state round trip).

Everything runs headless (``QT_QPA_PLATFORM=offscreen``) against the REAL Qt page
with a MagicMock controller — no hardware. ⚠ ``QMessageBox`` blocks forever
offscreen, so every path that can raise one is patched.
"""

import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

# Isolate the settings store BEFORE anything imports/builds a dialog, so the
# suite never reads or rewrites the operator's real config/workflows.
_SETTINGS_TMP = tempfile.mkdtemp(prefix="mebp_v79_ct_settings_")
os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", _SETTINGS_TMP)
os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = _SETTINGS_TMP

from SupportClasses import TargetTypeStore as TTS
from SupportClasses.PhysicalModels import (
    NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE, NeedleBore, NeedleSpec,
)
from SupportClasses.PickAndPlaceManager import BoreRole


# ── fixtures ─────────────────────────────────────────────────────────

def _bore(id_um=200.0, pump="P1", offset=(0.0, 0.0), z_off=0.0, gauge=22):
    return NeedleBore(id_um=id_um, od_um=id_um * 2, wall_um=id_um / 2,
                      length_mm=12.7, gauge=gauge, pump_id=pump,
                      offset_um=offset, z_offset_mm=z_off)


def _single_needle() -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=400.0, id_um=200.0, wall_um=100.0,
                      length_inches=0.5)


def _backpack(offset=(250.0, -120.0), z_off=0.03) -> NeedleSpec:
    """Two bores of DIFFERENT sizes bound together — the operator's case 2."""
    return NeedleSpec(
        gauge=22, od_um=400.0, id_um=200.0, wall_um=100.0,
        needle_form=NEEDLE_FORM_BACKPACK,
        bores=[_bore(200.0, "P1"),
               _bore(100.0, "P2", offset=offset, z_off=z_off, gauge=27)])


def _backpack_pumps(p0: str, p1: str) -> NeedleSpec:
    """A backpack whose bores declare specific pumps (or none at all)."""
    return NeedleSpec(
        gauge=22, od_um=400.0, id_um=200.0, wall_um=100.0,
        needle_form=NEEDLE_FORM_BACKPACK,
        bores=[_bore(200.0, p0),
               _bore(100.0, p1, offset=(250.0, -120.0), gauge=27)])


def _triple() -> NeedleSpec:
    return NeedleSpec(
        gauge=22, od_um=400.0, id_um=200.0, wall_um=100.0,
        needle_form=NEEDLE_FORM_TRIPLE,
        bores=[_bore(200.0, "P1"),
               _bore(150.0, "P2", offset=(200.0, 0.0)),
               _bore(100.0, "P3", offset=(0.0, 200.0))])


def _hw(needle, *, inks=None, locations=None, pumps=("P1", "P2", "P3")):
    return SimpleNamespace(
        needle=needle,
        pumps={p: SimpleNamespace(enabled=True, is_configured=True,
                                  ink_names=[]) for p in pumps},
        ink_library=dict(inks or {}),
        ink_locations=dict(locations or {}),
    )


def _mock_ctrl(*, xy=True, zp=True):
    ctrl = MagicMock()
    ctrl.is_xy_connected = xy
    ctrl.is_zp_connected = zp
    ctrl.zero_position = {"x": 1000.0, "y": 2000.0, "Z": 0.0}
    ctrl.safety_limits = None
    return ctrl


class _PageCase(unittest.TestCase):
    """Base: one QApplication, a freshly built real page per test."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        # Building a page calls ``load_last()``, so a last-used file written by an
        # earlier test would silently restore ITS settings into this one (roles
        # included). Start every test from "no last-used", whatever ran before.
        self._clear_last_used()

    @staticmethod
    def _clear_last_used():
        # Resolve the dir the way WorkflowSettingsStore does — the env var, READ
        # NOW. Several test modules set it at import time, so the winner depends
        # on load order and a module-level constant would clear the wrong tree.
        base = os.environ.get("MEBP_WORKFLOW_SETTINGS_DIR") or _SETTINGS_TMP
        try:
            os.remove(os.path.join(base, "cell_targeting", "__last__.json"))
        except OSError:
            pass

    def _make_page(self, *, ctrl=None):
        from gui.pages.workflows.cell_targeting_workflow import (
            CellTargetingWorkflowPage,
        )
        page = CellTargetingWorkflowPage(
            ctrl or _mock_ctrl(xy=False, zp=False),
            settings=None, camera_manager=None)
        # The page owns a modeless dialog + an embedded page; drop them with the
        # test so a later test can't be handed a stale widget.
        self.addCleanup(page.deleteLater)
        return page


# ── the two tabs ─────────────────────────────────────────────────────

class TestTabs(_PageCase):
    def test_exactly_two_tabs_named_setup_and_viewer(self):
        page = self._make_page()
        self.assertEqual(page._tabs.count(), 2)
        self.assertEqual(page._tabs.tabText(0), "Setup")
        self.assertIn("Survey", page._tabs.tabText(1))

    def test_setup_tab_hosts_the_bore_panel(self):
        from gui.pages.workflows.cell_targeting_setup_panel import (
            CellTargetingSetupPanel,
        )
        page = self._make_page()
        self.assertIsInstance(page._tabs.widget(0), CellTargetingSetupPanel)
        self.assertIs(page._tabs.widget(0), page._setup_panel)

    def test_viewer_tab_embeds_a_live_fluorescence_page(self):
        """A real INSTANCE, not a reimplementation (decision D4)."""
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage,
        )
        page = self._make_page()
        self.assertIsInstance(page._scan_page, FluorescenceMosaicWorkflowPage)
        # embedded=True drops only the back-button header.
        self.assertTrue(page._scan_page._embedded)

    def test_run_row_lives_outside_the_tabs(self):
        """Start / Abort / status must be reachable from either tab."""
        page = self._make_page()
        for w in (page._start_btn, page._abort_btn, page._status):
            parent = w.parent()
            seen = []
            while parent is not None:
                seen.append(parent)
                parent = parent.parent()
            self.assertNotIn(page._tabs, seen)

    def test_promoted_fields_are_laid_out_on_the_setup_tab(self):
        page = self._make_page()
        panel = page._setup_panel
        for w in (page._removal_z, page._place_z, page._reagent_combo,
                  page._push_depth, page._dwell, page._prep_check):
            self.assertTrue(panel.isAncestorOf(w), f"{w} not on the Setup tab")

    def test_promoted_fields_still_ride_along_with_the_profile(self):
        """register_external keeps a promoted control in every save/load path."""
        page = self._make_page()
        values = page._settings_dialog.collect()
        for key, _attr, _default in page._PROMOTED_DEFAULTS:
            self.assertIn(key, values, f"{key} lost from the settings profile")


# ── the per-bore program table ───────────────────────────────────────

class TestBoreTable(_PageCase):
    def test_one_row_per_bore(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_single_needle()))
        self.assertEqual(page._setup_panel.bore_count(), 1)
        page.set_hardware_config(_hw(_backpack()))
        self.assertEqual(page._setup_panel.bore_count(), 2)
        page.set_hardware_config(_hw(_triple()))
        self.assertEqual(page._setup_panel.bore_count(), 3)

    def test_rebuilds_on_a_needle_change(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_triple()))
        self.assertEqual(len(page._setup_panel.programs()), 3)
        page.set_hardware_config(_hw(_single_needle()))
        progs = page._setup_panel.programs()
        self.assertEqual(len(progs), 1)
        self.assertEqual(progs[0].bore_index, 0)

    def test_no_needle_shows_the_empty_state_and_no_rows(self):
        page = self._make_page()
        self.assertEqual(page._setup_panel.bore_count(), 0)
        self.assertTrue(page._setup_panel._bore_empty.isVisible()
                        or not page._setup_panel._bore_empty.isHidden())

    def test_pump_ids_come_from_the_assembly(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        progs = page._setup_panel.programs()
        self.assertEqual([p.pump_id for p in progs], ["P1", "P2"])

    def test_default_roles_aspirate_on_bore0_push_on_bore1(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        progs = page._setup_panel.programs()
        self.assertIs(progs[0].role, BoreRole.ASPIRATE_TARGET)
        self.assertIs(progs[1].role, BoreRole.PUSH_REAGENT)

    def test_single_bore_default_is_the_legacy_aspirate_only_program(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_single_needle()))
        progs = page._setup_panel.programs()
        self.assertEqual(len(progs), 1)
        self.assertIs(progs[0].role, BoreRole.ASPIRATE_TARGET)
        self.assertFalse(page._current_config().trypsin_enabled)

    def test_role_selection_survives_a_rebuild(self):
        """A needle push must not silently wipe the operator's assignments."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        row = page._setup_panel._rows[1]
        row.role.setCurrentIndex(row.role.findData(BoreRole.IDLE.value))
        page.set_hardware_config(_hw(_backpack()))
        self.assertIs(page._setup_panel.programs()[1].role, BoreRole.IDLE)

    def test_a_rewired_assembly_wins_over_a_remembered_pump(self):
        """Swapping the assembly must not keep driving the previous SYRINGES.

        ``_aspirate_pump_id()`` / ``trypsin_bore`` are read straight off this
        table, so a pump remembered from the old needle would command the wrong
        pump. Measured before the fix: a P1/P2 backpack swapped for a P2/P3 one
        left the table — and the executed config — on P1/P2.
        """
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack_pumps("P1", "P2")))
        self.assertEqual([p.pump_id for p in page._setup_panel.programs()],
                         ["P1", "P2"])
        page.set_hardware_config(_hw(_backpack_pumps("P2", "P3")))
        self.assertEqual([p.pump_id for p in page._setup_panel.programs()],
                         ["P2", "P3"])
        cfg = page._current_config()
        self.assertEqual(cfg.reagent_bore, "P2")
        self.assertEqual(cfg.trypsin_bore, "P3")

    def test_a_hand_wired_pump_survives_an_unrelated_rebuild(self):
        """The combo exists to wire a bore by hand; an unrelated hardware push
        rebuilds the table and must not wipe that (hard-parts (f)/4)."""
        page = self._make_page()
        # Neither bore declares a pump → the operator wires them.
        page.set_hardware_config(_hw(_backpack_pumps("", "")))
        rows = page._setup_panel._rows
        rows[0].pump.setCurrentIndex(rows[0].pump.findData("P3"))
        page.set_hardware_config(_hw(_backpack_pumps("", "")))
        self.assertEqual(page._setup_panel.programs()[0].pump_id, "P3")

    def test_a_pump_disagreeing_with_the_assembly_is_reported(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack_pumps("P1", "P2")))
        rows = page._setup_panel._rows
        rows[0].pump.setCurrentIndex(rows[0].pump.findData("P3"))
        notes = page._setup_panel.validation_notes()
        self.assertTrue(any("declares P1" in n for n in notes), notes)

    def test_measured_offset_is_shown(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack(offset=(250.0, -120.0))))
        text = page._setup_panel._rows[1].offset.text()
        self.assertIn("250", text)
        self.assertIn("120", text)
        self.assertNotIn("not measured", text)

    def test_unmeasured_offset_is_surfaced_on_the_row_and_in_the_notes(self):
        """An unmeasured offset lands the bore 100-500 µm off the cell."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack(offset=(0.0, 0.0), z_off=0.0)))
        self.assertIn("not measured",
                      page._setup_panel._rows[1].offset.text())
        notes = page._setup_panel.validation_notes()
        self.assertTrue(any("no measured offset" in n for n in notes), notes)

    def test_offsets_applied_after_the_table_was_built_are_picked_up(self):
        """The Calibration page writes the measured offsets onto the SAME
        NeedleBore objects; a show must re-read them, not stay stale."""
        from PySide6.QtGui import QShowEvent
        page = self._make_page()
        needle = _backpack(offset=(0.0, 0.0), z_off=0.0)
        page.set_hardware_config(_hw(needle))
        self.assertIn("not measured", page._setup_panel._rows[1].offset.text())
        needle.bores[1].offset_um = (330.0, -145.0)   # what apply_to_needle does
        page.showEvent(QShowEvent())
        text = page._setup_panel._rows[1].offset.text()
        self.assertIn("330", text)
        self.assertNotIn("not measured", text)
        self.assertEqual(page._setup_panel.validation_notes(), [])

    def test_bore0_offset_is_the_datum_not_a_warning(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        self.assertIn("datum", page._setup_panel._rows[0].offset.text())

    def test_only_the_pushing_row_enables_its_push_parameters(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        asp, push = page._setup_panel._rows
        self.assertFalse(asp.volume.isEnabled())
        self.assertFalse(asp.lead.isEnabled())
        self.assertTrue(push.volume.isEnabled())
        self.assertTrue(push.lead.isEnabled())

    def test_note_when_one_bore_both_pushes_and_aspirates(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        rows = page._setup_panel._rows
        rows[0].role.setCurrentIndex(
            rows[0].role.findData(BoreRole.PUSH_REAGENT.value))
        rows[1].role.setCurrentIndex(
            rows[1].role.findData(BoreRole.ASPIRATE_TARGET.value))
        rows[1].role.setCurrentIndex(
            rows[1].role.findData(BoreRole.IDLE.value))
        notes = page._setup_panel.validation_notes()
        self.assertTrue(any("aspirate" in n for n in notes), notes)

    def test_there_is_no_overlap_control_anywhere(self):
        """Decision D5 makes overlap impossible, so nothing may offer it."""
        from PySide6.QtWidgets import QCheckBox
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        for cb in page._setup_panel.findChildren(QCheckBox):
            self.assertNotIn("overlap", cb.text().lower())


# ── target types ─────────────────────────────────────────────────────

class TestTargetTypes(_PageCase):
    def setUp(self):
        super().setUp()
        root = tempfile.mkdtemp(prefix="mebp_v79_tt_")
        os.makedirs(os.path.join(root, "builtin"), exist_ok=True)
        user = os.path.join(root, "user")
        os.makedirs(user, exist_ok=True)
        import json
        with open(os.path.join(user, "gfp_only.json"), "w",
                  encoding="utf-8") as f:
            json.dump({
                "id": "gfp_only", "name": "GFP only", "color": "#a6e3a1",
                "signature_rule": {
                    "combinator": "all",
                    "clauses": [{"imaging_channel": "FITC", "state": "positive"},
                                {"imaging_channel": "mCherry",
                                 "state": "negative"}],
                },
            }, f)
        prev_env = os.environ.get("MEBP_TARGET_TYPE_DIR")
        prev_singleton = TTS._store_singleton
        os.environ["MEBP_TARGET_TYPE_DIR"] = root
        TTS._store_singleton = None

        def _restore():
            TTS._store_singleton = prev_singleton
            if prev_env is None:
                os.environ.pop("MEBP_TARGET_TYPE_DIR", None)
            else:
                os.environ["MEBP_TARGET_TYPE_DIR"] = prev_env

        self.addCleanup(_restore)

    def test_types_appear_in_every_bore_row(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        row = page._setup_panel._rows[0]
        self.assertGreaterEqual(row.target_type.findData("gfp_only"), 0)

    def test_selected_type_is_stamped_not_referenced(self):
        """The program carries the resolved name + colour, per stamp-don't-reference."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        row = page._setup_panel._rows[0]
        row.target_type.setCurrentIndex(row.target_type.findData("gfp_only"))
        prog = page._setup_panel.programs()[0]
        self.assertEqual(prog.target_type_id, "gfp_only")
        self.assertEqual(prog.target_type_name, "GFP only")
        self.assertEqual(prog.target_type_color, "#a6e3a1")

    def test_the_ui_says_the_rules_are_not_evaluated(self):
        """Decision D3: authored only. A picker that implied a working detector
        would be trusted as one."""
        from PySide6.QtWidgets import QLabel
        page = self._make_page()
        texts = " ".join(lbl.text().lower()
                         for lbl in page._setup_panel.findChildren(QLabel))
        self.assertIn("recorded intent", texts)
        self.assertFalse(TTS.RULE_EVALUATION_IMPLEMENTED)


# ── role → CellRemovalConfig ─────────────────────────────────────────

class TestConfigFromPrograms(_PageCase):
    def _configured_page(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        return page

    def test_aspirate_row_drives_index_and_reagent_pump(self):
        page = self._configured_page()
        rows = page._setup_panel._rows
        # Make bore 2 the aspirating bore and bore 1 the pusher.
        rows[0].role.setCurrentIndex(
            rows[0].role.findData(BoreRole.PUSH_REAGENT.value))
        rows[1].role.setCurrentIndex(
            rows[1].role.findData(BoreRole.ASPIRATE_TARGET.value))
        cfg = page._current_config()
        self.assertEqual(cfg.aspirate_bore_index, 1)
        self.assertEqual(cfg.reagent_bore, "P2")
        self.assertTrue(cfg.trypsin_enabled)
        self.assertEqual(cfg.trypsin_bore, "P1")
        self.assertEqual(cfg.trypsin_bore_index, 0)

    def test_push_row_params_reach_every_trypsin_field(self):
        page = self._configured_page()
        push = page._setup_panel._rows[1]
        push.volume.setValue(0.0125)
        push.depth.setValue(0.075)
        push.rate.setValue(0.35)
        push.lead.setValue(12.5)
        cfg = page._current_config()
        self.assertTrue(cfg.trypsin_enabled)
        self.assertEqual(cfg.trypsin_bore, "P2")
        self.assertEqual(cfg.trypsin_bore_index, 1)
        self.assertAlmostEqual(cfg.trypsin_volume_uL, 0.0125)
        self.assertAlmostEqual(cfg.trypsin_depth_mm, 0.075)
        self.assertAlmostEqual(cfg.trypsin_push_rate_uL_s, 0.35)
        self.assertAlmostEqual(cfg.trypsin_lead_time_s, 12.5)

    def test_no_pushing_bore_leaves_every_trypsin_field_at_its_default(self):
        """Byte-identical to the pre-v7.9 single-bore sequence."""
        from SupportClasses.PickAndPlaceManager import CellRemovalConfig
        page = self._configured_page()
        rows = page._setup_panel._rows
        rows[1].role.setCurrentIndex(rows[1].role.findData(BoreRole.IDLE.value))
        cfg = page._current_config()
        default = CellRemovalConfig()
        self.assertFalse(cfg.trypsin_enabled)
        self.assertEqual(cfg.trypsin_bore, default.trypsin_bore)
        self.assertEqual(cfg.trypsin_lead_time_s, default.trypsin_lead_time_s)
        self.assertEqual(cfg.aspirate_bore_index, 0)

    def test_a_pushing_bore_with_no_pump_does_not_enable_trypsin(self):
        page = self._configured_page()
        push = page._setup_panel._rows[1]
        push.pump.setCurrentIndex(push.pump.findData(""))
        self.assertFalse(page._current_config().trypsin_enabled)

    def test_push_volume_follows_the_ASPIRATING_bore_not_bore_0(self):
        """The readout / backstop / Start gate must meter the same bore the
        executor does.

        Reading the flat ``cross_section_area_mm2`` (bore 0, by v7.6's fail-safe
        rule) made them disagree by the bore-area ratio — 4.00× on this 200/100 µm
        backpack — the moment a bore other than the datum aspirated.
        """
        page = self._configured_page()
        needle = page._hw_config.needle
        rows = page._setup_panel._rows
        rows[0].role.setCurrentIndex(rows[0].role.findData(BoreRole.IDLE.value))
        rows[1].role.setCurrentIndex(
            rows[1].role.findData(BoreRole.ASPIRATE_TARGET.value))
        cfg = page._current_config()
        self.assertEqual(cfg.aspirate_bore_index, 1)
        gui_push, gui_pull = page._push_pull_uL()
        self.assertAlmostEqual(gui_push, cfg.compute_release_volume_uL(needle))
        self.assertAlmostEqual(gui_pull, cfg.compute_extract_volume_uL(needle))
        self.assertAlmostEqual(cfg.release_volume_uL, gui_push)
        # and the readout is refreshed by the role change, not left stale
        self.assertIn(f"{gui_push:.4f}", page._volume_label.text())

    def test_single_bore_push_volume_is_bit_identical(self):
        """The bore-resolving read must not perturb any legacy needle."""
        import math
        page = self._make_page()
        page.set_hardware_config(_hw(_single_needle()))
        page._push_depth.setValue(0.10)
        page._pull_mult.setValue(2.0)
        push, pull = page._push_pull_uL()
        self.assertEqual(push, math.pi * (0.1 ** 2) * 0.10)
        self.assertEqual(
            push, page._current_config().compute_release_volume_uL(
                page._hw_config.needle))
        self.assertEqual(pull, 2.0 * push)

    def test_promoted_spins_still_drive_the_config(self):
        """The pre-v7.9 attribute contract the partial-page suites depend on."""
        page = self._configured_page()
        page._removal_z.setValue(0.20)
        page._place_z.setValue(0.60)
        page._dwell.setValue(45.0)
        page._pull_mult.setValue(3.0)
        page._push_speed.setValue(0.25)
        page._pull_speed.setValue(7.5)
        cfg = page._current_config()
        self.assertAlmostEqual(cfg.removal_z_offset_mm, 0.20)
        self.assertAlmostEqual(cfg.place_z_offset_mm, 0.60)
        self.assertAlmostEqual(cfg.dwell_time_s, 45.0)
        self.assertAlmostEqual(cfg.extract_multiplier, 3.0)
        self.assertAlmostEqual(cfg.push_speed_uL_s, 0.25)
        self.assertAlmostEqual(cfg.pull_speed_uL_s, 7.5)


# ── stage-busy: the embedded scan page is a SECOND stage driver ──────

class TestStageBusy(_PageCase):
    def test_busy_while_the_embedded_page_reports_scanning(self):
        page = self._make_page()
        page._scan_page.is_scanning = lambda: False
        self.assertFalse(page._stage_busy())
        page._scan_page.is_scanning = lambda: True
        self.assertTrue(page._stage_busy())
        self.assertIn("scan", page._status.text().lower())

    def test_alias_is_kept(self):
        page = self._make_page()
        page._scan_page.is_scanning = lambda: True
        self.assertTrue(page._travel_blocked_by_run())

    def test_z_badge_refuses_during_a_scan(self):
        """The 4th motion entry point: the XZ view's Z-reference badge.

        The embedded scan drives Z through ``safe_travel_to``, so an absolute Z
        underneath it is a second driver on the serial channel and on the
        non-refcounted poller suspend.
        """
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: True
        page._on_go_to_z_requested(41.0)
        page._controller.move_z_absolute.assert_not_called()
        page._scan_page.is_scanning = lambda: False
        page._on_go_to_z_requested(41.0)
        page._controller.move_z_absolute.assert_called_once_with(
            41.0, from_zero_ref=True)

    def test_start_is_greyed_out_during_a_scan(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_single_needle()))
        page._scan_page.is_scanning = lambda: True
        page._update_button_state()
        self.assertFalse(page._start_btn.isEnabled())

    def test_start_refuses_during_a_scan(self):
        page = self._make_page()
        page._scan_page.is_scanning = lambda: True
        page._on_start()
        self.assertIn("scan", page._status.text().lower())
        self.assertIsNone(page._exec_thread)

    def test_travel_to_absolute_refuses_during_a_scan(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._scan_page.is_scanning = lambda: True
        page._travel_worker = MagicMock()
        self.assertFalse(page._travel_to_absolute(50000.0, 60000.0))
        page._travel_worker.start.assert_not_called()


# ── absolute vs zero-ref travel MUST stay separate handlers ─────────

class TestTravelFrames(_PageCase):
    def _armed_page(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._scan_page.is_scanning = lambda: False
        page._travel_worker = MagicMock()
        return page

    def test_same_numbers_reach_different_destinations(self):
        """Sharing one handler would add ``zero_position`` twice — millimetres off."""
        page = self._armed_page()
        page._on_workspace_position_clicked(5000.0, 7000.0)
        zero_ref_call = page._travel_worker.start.call_args
        page._travel_worker.reset_mock()
        self.assertTrue(page._travel_to_absolute(5000.0, 7000.0))
        absolute_call = page._travel_worker.start.call_args

        zero = page._controller.zero_position
        self.assertEqual(zero_ref_call.args[1], 5000.0 + zero["x"])
        self.assertEqual(zero_ref_call.args[2], 7000.0 + zero["y"])
        self.assertEqual(absolute_call.args[1], 5000.0)
        self.assertEqual(absolute_call.args[2], 7000.0)
        self.assertNotEqual(zero_ref_call.args[1:3], absolute_call.args[1:3])

    def test_absolute_travel_retracts_and_never_descends(self):
        page = self._armed_page()
        page._travel_to_absolute(1234.0, 5678.0)
        kwargs = page._travel_worker.start.call_args.kwargs
        self.assertEqual(kwargs["safe_z_mm"], 40.0)
        self.assertIsNone(kwargs["target_z_mm"])

    def test_absolute_travel_never_calls_a_bare_move(self):
        page = self._armed_page()
        page._travel_to_absolute(1234.0, 5678.0)
        page._controller.move_xy_absolute.assert_not_called()
        page._controller.move_xy_absolute_um.assert_not_called()

    def test_absolute_travel_refuses_without_the_zp_board(self):
        page = self._make_page(ctrl=_mock_ctrl(zp=False))
        page._safe_z = 40.0
        page._travel_worker = MagicMock()
        self.assertFalse(page._travel_to_absolute(1.0, 2.0))
        page._travel_worker.start.assert_not_called()
        self.assertIn("ZP", page._status.text())

    def test_absolute_travel_refuses_without_a_safe_z(self):
        page = self._armed_page()
        page._safe_z = None
        self.assertFalse(page._travel_to_absolute(1.0, 2.0))
        page._travel_worker.start.assert_not_called()


# ── the mosaic-shift Start refusal (promoted out of the GUI) ─────────

def _ctx(*, has_shift=True, scale=0.5):
    """A mosaic_context()-shaped dict. ``image`` only needs ``.shape``."""
    return {
        "plate_key": "24", "well": "A1", "channel": "FITC",
        "channels": ["FITC"],
        "image": SimpleNamespace(shape=(2000, 3000)),
        "extent_um": (10000.0, 20000.0, 16000.0, 24000.0),
        "mosaic_scale": scale, "derived_scale": scale, "scale_warning": "",
        "shift_um": (0.0, 0.0), "has_shift": has_shift,
        "um_per_px": 2.0, "objective": "10x",
        "well_center_um": (13000.0, 22000.0), "well_radius_um": 3000.0,
    }


class TestMosaicShiftGate(_PageCase):
    def _page_with_mosaic_targets(self, *, has_shift):
        from gui.widgets.live_target_picker import PROV_MOSAIC
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._scan_page.is_scanning = lambda: False
        page._scan_page.mosaic_context = lambda _ch=None: _ctx(
            has_shift=has_shift)
        page._picker.add_pick(12000.0, 21000.0, provenance=PROV_MOSAIC)
        page._picker.add_place(13000.0, 22000.0)
        return page

    def test_start_refuses_on_a_shift_less_mosaic(self):
        page = self._page_with_mosaic_targets(has_shift=False)
        page._on_start()
        text = page._status.text()
        self.assertIn("registration shift", text)
        self.assertIn("µm", text)          # the error bound, in the operator's units
        self.assertIsNone(page._exec_thread)

    def test_the_stated_bound_is_20_percent_of_the_fov_width(self):
        page = self._page_with_mosaic_targets(has_shift=False)
        page._on_start()
        # 3000 px × 2.0 µm/px × 0.2 = 1200 µm
        self.assertIn("1200", page._status.text())

    def test_start_allows_a_recorded_shift(self):
        page = self._page_with_mosaic_targets(has_shift=True)
        page._on_start()
        self.assertNotIn("registration shift", page._status.text())

    def test_live_clicked_targets_are_never_blocked(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.mosaic_context = lambda _ch=None: _ctx(has_shift=False)
        page._picker.add_pick(1.0, 2.0)      # PROV_LIVE by default
        page._picker.add_place(3.0, 4.0)
        self.assertIsNone(page._mosaic_shift_refusal())

    def test_no_context_counts_as_untrustworthy(self):
        """Matching SpheroidSurveyPanel.can_command_motion: unknown ⇒ refuse."""
        page = self._page_with_mosaic_targets(has_shift=True)
        page._scan_page.mosaic_context = lambda _ch=None: None
        self.assertIsNotNone(page._mosaic_shift_refusal())

    def test_mosaic_goto_and_transfer_are_disabled_without_a_shift(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.mosaic_context = lambda _ch=None: _ctx(has_shift=False)
        page._mosaic_point_um = (12345.0, 23456.0)
        page._refresh_mosaic_point_buttons()
        self.assertFalse(page._mosaic_goto_btn.isEnabled())
        self.assertFalse(page._mosaic_pick_btn.isEnabled())

    def test_mosaic_click_back_projects_to_absolute_stage_um(self):
        from PySide6.QtCore import QPointF
        from SupportClasses.SpheroidDetector import back_project_px
        page = self._make_page(ctrl=_mock_ctrl())
        ctx = _ctx(has_shift=True)
        page._scan_page.mosaic_context = lambda _ch=None: ctx
        page._on_mosaic_scene_clicked(QPointF(400.0, 300.0))
        expect = back_project_px((400.0, 300.0), ctx["extent_um"],
                                 ctx["mosaic_scale"], ctx["shift_um"])
        self.assertEqual(page._mosaic_point_um, expect)
        self.assertTrue(page._mosaic_goto_btn.isEnabled())

    def test_a_new_mosaic_drops_the_picked_point(self):
        """A point from another well's mosaic must not stay armed."""
        from PySide6.QtCore import QPointF
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.mosaic_context = lambda _ch=None: _ctx(has_shift=True)
        page._on_mosaic_scene_clicked(QPointF(400.0, 300.0))
        self.assertIsNotNone(page._mosaic_point_um)
        page._scan_page.mosaic_ready.emit("B2")
        self.assertIsNone(page._mosaic_point_um)
        self.assertFalse(page._mosaic_goto_btn.isEnabled())
        self.assertFalse(page._mosaic_pick_btn.isEnabled())

    def test_mosaic_goto_uses_the_absolute_handler(self):
        from PySide6.QtCore import QPointF
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._scan_page.is_scanning = lambda: False
        page._scan_page.mosaic_context = lambda _ch=None: _ctx(has_shift=True)
        page._travel_worker = MagicMock()
        page._on_mosaic_scene_clicked(QPointF(400.0, 300.0))
        page._on_mosaic_goto()
        args = page._travel_worker.start.call_args
        self.assertEqual(args.args[1:3], page._mosaic_point_um)
        self.assertIsNone(args.kwargs["target_z_mm"])


# ── persistence: the dynamic table rides on set_extra_state ─────────

class TestExtraStateRoundTrip(_PageCase):
    def test_table_round_trips_through_the_settings_profile(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        rows = page._setup_panel._rows
        rows[0].role.setCurrentIndex(
            rows[0].role.findData(BoreRole.DISPENSE_PLACE.value))
        rows[1].lead.setValue(9.5)
        rows[1].rate.setValue(0.42)
        dlg = page._settings_dialog
        saved = dlg.collect()
        self.assertIn(dlg.EXTRA_KEY, saved)

        fresh = self._make_page()
        fresh.set_hardware_config(_hw(_backpack()))
        fresh._settings_dialog.apply(saved)
        progs = fresh._setup_panel.programs()
        self.assertIs(progs[0].role, BoreRole.DISPENSE_PLACE)
        self.assertAlmostEqual(progs[1].lead_time_s, 9.5)
        self.assertAlmostEqual(progs[1].rate_uL_s, 0.42)

    def test_setup_tab_survives_a_restart_without_opening_the_popout(self):
        """The Setup tab is now the primary surface, so leaving the workflow must
        write last-used.

        ``WorkflowSettingsDialog`` auto-saves from its OWN hide/close, which was
        enough while the popout was the only place to edit anything. With 13
        fields and the bore table promoted onto the tab, the normal workflow never
        opens the popout — and nothing was saved at all: a whole session came back
        at defaults (measured: no ``__last__.json`` written).
        """
        from PySide6.QtGui import QHideEvent
        # This is the ONE test that deliberately writes last-used, so give it a
        # private store: several modules point the env at their own tmp dir and
        # whichever imported last owns it, so a leaked file here would restore
        # ITS bore roles into an unrelated test in a combined run.
        private = tempfile.mkdtemp(prefix="mebp_v79_ct_persist_")
        prev = os.environ.get("MEBP_WORKFLOW_SETTINGS_DIR")
        os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = private
        self.addCleanup(
            lambda: os.environ.__setitem__("MEBP_WORKFLOW_SETTINGS_DIR", prev)
            if prev is not None
            else os.environ.pop("MEBP_WORKFLOW_SETTINGS_DIR", None))

        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        page._dwell.setValue(123.0)
        page._removal_z.setValue(0.44)
        rows = page._setup_panel._rows
        rows[1].lead.setValue(7.5)
        rows[1].role.setCurrentIndex(rows[1].role.findData(BoreRole.IDLE.value))
        self.assertFalse(page._settings_dialog.isVisible())
        page.hideEvent(QHideEvent())

        fresh = self._make_page()
        fresh.set_hardware_config(_hw(_backpack()))
        self.assertAlmostEqual(fresh._dwell.value(), 123.0)
        self.assertAlmostEqual(fresh._removal_z.value(), 0.44)
        progs = fresh._setup_panel.programs()
        self.assertAlmostEqual(progs[1].lead_time_s, 7.5)
        self.assertIs(progs[1].role, BoreRole.IDLE)

    def test_extra_state_is_wired_to_the_dialog(self):
        page = self._make_page()
        dlg = page._settings_dialog
        self.assertIsNotNone(dlg._extra_get)
        self.assertIsNotNone(dlg._extra_set)
        self.assertEqual(dlg._extra_get(), page._collect_extra_state())

    def test_trypsin_reagent_choice_round_trips(self):
        page = self._make_page()
        page.set_hardware_config(_hw(
            _backpack(),
            inks={"Trypsin": SimpleNamespace(ink_type="ink", color="#f38ba8")},
            locations={"Trypsin": ["B1"]}))
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        state = page._collect_extra_state()
        self.assertEqual(state["trypsin_reagent"], "Trypsin")

        fresh = self._make_page()
        fresh._apply_extra_state(state)
        self.assertEqual(fresh._setup_panel.trypsin_reagent_name(), "Trypsin")

    def test_unknown_role_in_a_saved_profile_degrades_to_idle(self):
        """A bore whose purpose this build cannot read must do NOTHING."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        page._apply_extra_state({"bore_programs": [
            {"bore_index": 0, "pump_id": "P1", "role": "teleport"},
        ]})
        self.assertIs(page._setup_panel.programs()[0].role, BoreRole.IDLE)


# ── the attribute-name contract ─────────────────────────────────────

class TestAttributeContract(_PageCase):
    """The partial-page suites never build this UI, so a rename breaks them
    SILENTLY (the v7.7 lesson). Pin the names here, where it fails loudly."""

    NAMES = (
        # read by tests/test_v75x_cell_targeting_removal.py::TestPage
        "_current_config", "_push_pull_uL", "_on_start", "_status",
        "_removal_z", "_place_z", "_dwell", "_pull_mult", "_push_depth",
        "_hw_config",
        # read by the workflow-settings popout suite
        "_settings_dialog", "_open_settings",
        # the run + gating surface
        "_start_btn", "_abort_btn", "_on_abort", "_update_button_state",
        "_stage_busy", "_travel_blocked_by_run", "_travel_to_absolute",
        "_plate_offset_to_zref", "_bore", "_reagent_combo",
        "_prep_check", "_clean_check", "_wash_after_pickup_check",
        "_service_z", "_prep_rate", "_oil_needles", "_buffer_needles",
        "_wash_cycles", "_wash_z_amp", "_wash_xy_amp", "_wash_dwell",
        "_post_dispense", "_intra_retract", "_z_timeout", "_xy_timeout",
        # v7.9
        "_setup_panel", "_scan_page", "_tabs", "_collect_extra_state",
        "_apply_extra_state", "_mosaic_shift_refusal",
    )

    def test_every_name_still_exists(self):
        page = self._make_page()
        missing = [n for n in self.NAMES if not hasattr(page, n)]
        self.assertEqual(missing, [], f"renamed/removed: {missing}")

    def test_host_contract_methods_exist(self):
        page = self._make_page()
        for name in ("get_page_title", "get_sub_page_title",
                     "get_context_widget", "on_status_update", "set_settings",
                     "set_hardware_config", "set_calibration_data",
                     "set_z_references", "set_common_print_settings"):
            self.assertTrue(callable(getattr(page, name, None)), name)


# ── forwarding to the embedded page ─────────────────────────────────

class TestForwarding(_PageCase):
    def test_set_settings_is_forwarded(self):
        """The spheroid host omits this — the omission is a bug, not a pattern."""
        page = self._make_page()
        sentinel = object()
        page._scan_page.set_settings = MagicMock()
        page.set_settings(sentinel)
        page._scan_page.set_settings.assert_called_once_with(sentinel)

    def test_hardware_and_calibration_are_forwarded(self):
        page = self._make_page()
        page._scan_page.set_hardware_config = MagicMock()
        page._scan_page.set_calibration_data = MagicMock()
        page._scan_page.set_z_references = MagicMock()
        hw = _hw(_single_needle())
        page.set_hardware_config(hw)
        page.set_calibration_data(None, {"A1": (1.0, 2.0)}, 40.0)
        page.set_z_references({"plate_bottom_z": 20.0})
        page._scan_page.set_hardware_config.assert_called_once_with(hw)
        page._scan_page.set_calibration_data.assert_called_once()
        page._scan_page.set_z_references.assert_called_once()

    def test_leaving_the_page_does_not_explicitly_hide_the_embedded_page(self):
        """An explicit ``hide()`` on a child STICKS — the viewer tab came back
        BLANK on every return, which would make D4's live instance one-shot.

        Replaces an earlier test that asserted ``_scan_page.hide()`` IS called:
        that pinned the defect. Qt delivers the child's hide event (camera stop +
        its own dialog) from the parent hide on its own, and re-shows it.
        """
        page = self._make_page()
        page._tabs.setCurrentIndex(1)
        page.show()
        self._app.processEvents()
        self.assertTrue(page._scan_page.isVisible())
        page.hide()
        self._app.processEvents()
        self.assertFalse(page._scan_page.isHidden(),
                         "the embedded page was explicitly hidden")
        page.show()
        self._app.processEvents()
        self.assertTrue(page._scan_page.isVisible(),
                        "the Well Survey / Viewer tab came back blank")
        page.hide()

    def test_hide_event_still_tucks_the_settings_popout_away(self):
        from PySide6.QtGui import QHideEvent
        page = self._make_page()
        page._settings_dialog.show()
        page.hideEvent(QHideEvent())
        self.assertFalse(page._settings_dialog.isVisible())

    def test_no_dead_common_print_settings_forward(self):
        """The embedded page defines no such method; don't pretend it might."""
        page = self._make_page()
        self.assertFalse(hasattr(page._scan_page, "set_common_print_settings"))
        page.set_common_print_settings(None)   # must not raise


# ── Start gating for a dedicated pushing bore ────────────────────────

class TestPushingBoreStartGate(_PageCase):
    def _page(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: False
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            _backpack(),
            inks={"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa"),
                  "Trypsin": SimpleNamespace(ink_type="ink", color="#f38ba8")},
            locations={"Reagent": ["A1"], "Trypsin": ["B1"]}))
        page.set_calibration_data(None, {"A1": (1000.0, 2000.0),
                                         "B1": (3000.0, 4000.0)}, 40.0)
        page._prep_check.setChecked(False)
        page._clean_check.setChecked(False)
        page._wash_after_pickup_check.setChecked(False)
        idx = page._reagent_combo.findData("Reagent")
        page._reagent_combo.setCurrentIndex(idx)
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        return page

    def test_refuses_when_the_pushing_bore_has_no_reagent(self):
        page = self._page()
        page._on_start()
        self.assertIn("push reagent", page._status.text().lower())
        self.assertIsNone(page._exec_thread)

    def test_runs_and_registers_the_trypsin_well_when_resolved(self):
        page = self._page()
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            executor = Exec.return_value
            executor.execute_queue = MagicMock(return_value=True)
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
        self.assertTrue(Exec.called)
        executor.set_well_positions.assert_called_once()
        wells = executor.set_well_positions.call_args.args[0]
        self.assertIn("__trypsin__", wells)
        self.assertEqual(wells["__trypsin__"], (3000.0, 4000.0))

    def test_status_surfaces_the_unmeasured_offset_advisory_at_start(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: False
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            _backpack(offset=(0.0, 0.0), z_off=0.0),
            inks={"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa"),
                  "Trypsin": SimpleNamespace(ink_type="ink", color="#f38ba8")},
            locations={"Reagent": ["A1"], "Trypsin": ["B1"]}))
        page.set_calibration_data(None, {"A1": (1000.0, 2000.0),
                                         "B1": (3000.0, 4000.0)}, 40.0)
        page._prep_check.setChecked(False)
        page._clean_check.setChecked(False)
        page._wash_after_pickup_check.setChecked(False)
        page._reagent_combo.setCurrentIndex(
            page._reagent_combo.findData("Reagent"))
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
        self.assertIn("offset", page._status.text().lower())


if __name__ == "__main__":
    unittest.main()
