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
from SupportClasses.CellRemovalReadiness import _fmt_uL
from SupportClasses.PhysicalModels import (
    NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE, NeedleBore, NeedleSpec,
)
from SupportClasses.PickAndPlaceManager import BoreRole
from gui.pages.workflows import cell_targeting_workflow as _CT_PAGE_MOD


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


class _FakeCam:
    """A camera device that opens instantly and records what it was asked to do.

    Substituted for the ``CameraWidget`` inside a REAL ``CameraManager``, so the
    manager's own dispatch runs — in particular its "already running → return"
    guard, which is exactly what makes the host's ``_camera_started_by_us``
    bookkeeping correct. A stub manager would have let that guard go untested.
    """

    def __init__(self):
        self._running = False
        self._opening = False
        self.starts = 0
        self.stops = 0

    def start(self):
        self._running = True
        self.starts += 1

    def stop(self):
        self._running = False
        self.stops += 1


def _real_manager(n=3):
    """A real ``CameraManager`` driving ``_FakeCam`` devices."""
    from gui.widgets.camera_manager import CameraManager
    mgr = CameraManager(max_cameras=n)
    for cam in list(mgr._cameras):
        try:
            cam.deleteLater()
        except Exception:
            pass
    mgr._cameras = [_FakeCam() for _ in range(n)]
    mgr._um_per_px = [2.0] * n
    mgr._um_per_px_set = [True] * n
    return mgr


def _arm_dose(page, bore_index: int = 1):
    """Give a bore the dosing role — now an EXPLICIT operator action.

    Before the post-v7.9 audit, bore 2 DEFAULTED to the dosing role, so simply
    mounting a multi-bore needle silently armed a two-bore dose-then-shift
    sequence (a 100-500 µm lateral move with the needle ~0.1 mm above glass, plus
    reagent onto live cells) with default numbers the operator had never seen.
    Arming it is now one deliberate click, so tests that WANT the two-bore
    sequence must ask for it — which is the point.
    """
    panel = page._setup_panel
    row = next(r for r in panel._rows if r.bore_index == bore_index)
    idx = row.role.findData(BoreRole.PUSH_REAGENT.value)
    assert idx >= 0
    row.role.setCurrentIndex(idx)
    return row


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
        # ⚠ The pre-run confirmation is a MODAL dialog, and exec() blocks forever
        # under offscreen Qt — the same hazard this module's docstring records for
        # QMessageBox. Auto-confirm by default; the tests that are ABOUT the
        # confirmation patch `_confirm_run` themselves to assert it was consulted.
        patcher = patch.object(
            _CT_PAGE_MOD.CellTargetingWorkflowPage, "_confirm_run",
            lambda *a, **k: True)
        patcher.start()
        self.addCleanup(patcher.stop)

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

    def _make_page(self, *, ctrl=None, camera_manager=None):
        from gui.pages.workflows.cell_targeting_workflow import (
            CellTargetingWorkflowPage,
        )
        page = CellTargetingWorkflowPage(
            ctrl or _mock_ctrl(xy=False, zp=False),
            settings=None, camera_manager=camera_manager)
        # The page owns a modeless dialog + an embedded page; drop them with the
        # test so a later test can't be handed a stale widget.
        self.addCleanup(page.deleteLater)
        return page


# ── the two tabs ─────────────────────────────────────────────────────

class TestTabs(_PageCase):
    def test_exactly_two_tabs_named_plan_and_run(self):
        page = self._make_page()
        self.assertEqual(page._tabs.count(), 2)
        self.assertEqual(page._tabs.tabText(page._TAB_PLAN), "Plan")
        self.assertEqual(page._tabs.tabText(page._TAB_RUN), "Run")

    def test_the_plan_tab_hosts_BOTH_the_well_and_the_protocol(self):
        """Planning needs the cells and the protocol side by side — that is the
        whole reason for a Plan surface distinct from a Run surface."""
        page = self._make_page()
        plan = page._tabs.widget(page._TAB_PLAN)
        self.assertTrue(plan.isAncestorOf(page._setup_panel), "no protocol")
        self.assertTrue(plan.isAncestorOf(page._scan_page), "no well mosaic")
        self.assertTrue(plan.isAncestorOf(page._picker), "no target picker")

    def test_the_run_tab_hosts_the_watching_instruments(self):
        page = self._make_page()
        run = page._tabs.widget(page._TAB_RUN)
        self.assertTrue(run.isAncestorOf(page._workspace_view))
        self.assertTrue(run.isAncestorOf(page._xz_view))
        # …and none of the protocol, so nothing that changes WHAT the machine
        # does can be reached from the monitoring surface.
        self.assertFalse(run.isAncestorOf(page._setup_panel))

    def test_starting_a_run_selects_the_run_tab(self):
        page = self._make_page()
        page._tabs.setCurrentIndex(page._TAB_PLAN)
        page._show_run_tab()
        self.assertEqual(page._tabs.currentIndex(), page._TAB_RUN)

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

    def test_every_promoted_field_is_laid_out_on_ONE_of_the_two_tabs(self):
        """A widget has exactly one parent, so each control has exactly one
        editing surface — the structural ones on Plan, the tuning ones on Run.
        What must never happen is a promoted field laid out on NEITHER, which is
        a control that persists but cannot be reached."""
        page = self._make_page()
        plan = page._tabs.widget(page._TAB_PLAN)
        run = page._tabs.widget(page._TAB_RUN)
        for key, attr, _default in page._PROMOTED_DEFAULTS:
            w = getattr(page, attr)
            on_plan = plan.isAncestorOf(w)
            on_run = run.isAncestorOf(w)
            self.assertTrue(on_plan or on_run, f"{key} is on neither tab")
            self.assertFalse(on_plan and on_run,
                             f"{key} is laid out twice — two editing surfaces")

    def test_the_tuning_knobs_live_on_RUN_and_only_there(self):
        """Operator decision: the four tuning settings are set on Run, which is
        where you are watching the cells respond to them."""
        page = self._make_page()
        plan = page._tabs.widget(page._TAB_PLAN)
        run = page._tabs.widget(page._TAB_RUN)
        for w in (page._push_volume, page._dwell, page._pull_mult,
                  page._pull_speed, page._removal_z):
            self.assertTrue(run.isAncestorOf(w), "a tuning knob is not on Run")
            self.assertFalse(plan.isAncestorOf(w), "…and must not be on Plan too")
        # Plan still shows the whole recipe — read-only.
        page._refresh_tuning_echo()
        echo = page._tuning_echo.text()
        self.assertIn("incubation", echo.lower())
        self.assertIn("removal Z", echo)

    def test_the_structural_settings_stay_on_PLAN(self):
        page = self._make_page()
        plan = page._tabs.widget(page._TAB_PLAN)
        for w in (page._place_z, page._reagent_z, page._reagent_combo,
                  page._push_depth, page._push_speed, page._prep_check):
            self.assertTrue(plan.isAncestorOf(w), f"{w} left the Plan tab")

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
        """⚠ The previous assertion was `isVisible() or not isHidden()`, which is
        a tautology: `isHidden()` is False by default on a never-shown widget, so
        `not isHidden()` is True regardless of what the code does. It would have
        passed even if the empty state were never shown at all. Assert panel
        STATE plus `isVisibleTo`, which is the non-tautological Qt API for a
        widget whose parent has not been shown.
        """
        panel = self._make_page()._setup_panel
        self.assertEqual(panel.bore_count(), 0)
        self.assertEqual(panel.programs(), [])
        self.assertEqual(len(panel._rows), 0)
        self.assertTrue(panel._bore_empty.isVisibleTo(panel),
                        "the empty-state label must be shown when no needle "
                        "is configured")
        # And the grid must hold nothing but its header row — no orphaned
        # widgets from a previous assembly.
        self.assertEqual(panel._grid.count(), panel._header_row_count * 9,
                         "only the 9 column headers should remain in the grid")

    def test_pump_ids_come_from_the_assembly(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        progs = page._setup_panel.programs()
        self.assertEqual([p.pump_id for p in progs], ["P1", "P2"])

    def test_mounting_a_multi_bore_needle_does_NOT_arm_a_dosing_bore(self):
        """⚠ DELIBERATE BEHAVIOUR CHANGE (post-v7.9 audit).

        Bore 2 used to DEFAULT to the dosing role, so switching the needle form —
        possibly for an unrelated reason — silently armed a two-bore sequence: a
        100-500 µm lateral shift performed with the needle ~0.1 mm above the glass,
        plus a reagent dose onto live cells, with default numbers the operator had
        never seen and was never asked to confirm.

        Bore 1 aspirates; everything else idles. Arming the dose is one explicit
        click, and the panel offers that click so the intended workflow stays easy.
        """
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        progs = page._setup_panel.programs()
        self.assertIs(progs[0].role, BoreRole.ASPIRATE_TARGET)
        self.assertIs(progs[1].role, BoreRole.IDLE,
                      "a second bore must not dose without being asked")
        self.assertFalse(page._current_config().trypsin_enabled,
                         "no dosing sequence may be armed implicitly")

    def test_the_panel_offers_one_click_to_arm_the_dosing_bore(self):
        """Safe-by-default must not mean hard-to-use."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        panel = page._setup_panel
        self.assertTrue(panel._arm_dose_row.isVisibleTo(panel))
        self.assertIn("bore 2", panel._arm_dose_btn.text())
        panel._arm_dose_btn.click()
        progs = panel.programs()
        self.assertIs(progs[1].role, BoreRole.PUSH_REAGENT)
        self.assertTrue(page._current_config().trypsin_enabled)
        # Offer withdrawn once a bore is dosing.
        self.assertFalse(panel._arm_dose_row.isVisibleTo(panel))

    def test_no_arm_offer_for_a_single_bore_needle(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_single_needle()))
        panel = page._setup_panel
        self.assertFalse(panel._arm_dose_row.isVisibleTo(panel))

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
        # A dosing bore is no longer armed implicitly, so ask for it here — the
        # thing under test is which PUMP the row carries, not the role.
        _arm_dose(page)
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
        # Only a bore the run DRIVES is worth flagging, so arm it first.
        _arm_dose(page)
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
        _arm_dose(page)
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
        # These tests are about how a DOSING bore's row reaches the config, so
        # arm one explicitly — a second bore no longer defaults to that role.
        _arm_dose(page)
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
        """⚠ The dose is now ENTERED in nL and STORED in µL.

        The input moves to nL; every assertion on the CONFIG stays in µL, because
        that is the boundary being tested. Rewriting the config-side expectation
        instead would hide the unit change rather than pin it.
        """
        page = self._configured_page()
        push = page._setup_panel._rows[1]
        push.volume.setValue(12.5)             # 12.5 nL
        push.rate.setValue(0.35)
        push.lead.setValue(12.5)
        cfg = page._current_config()
        self.assertTrue(cfg.trypsin_enabled)
        self.assertEqual(cfg.trypsin_bore, "P2")
        self.assertEqual(cfg.trypsin_bore_index, 1)
        self.assertAlmostEqual(cfg.trypsin_volume_uL, 0.0125)   # µL
        self.assertAlmostEqual(cfg.trypsin_push_rate_uL_s, 0.35)
        self.assertAlmostEqual(cfg.trypsin_lead_time_s, 12.5)
        # The depth is DERIVED from that same dose through this bore's orifice,
        # so the two can never name different amounts of reagent.
        area = push._area_mm2
        self.assertGreater(area, 0)
        self.assertAlmostEqual(cfg.trypsin_depth_mm, 0.0125 / area, places=4)

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
        self.assertIn(_fmt_uL(gui_push), page._volume_label.text())
        # The DOSE is what the operator chose, so switching to a finer aspirating
        # bore holds the volume and lengthens the derived column instead of
        # silently metering a different amount of reagent.
        self.assertAlmostEqual(
            gui_push, page._push_volume.value() / 1000.0, places=6)
        self.assertGreater(page._push_depth.value(), 0.10,
                           "a finer bore needs more column for the same dose")

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
        page._tabs.setCurrentIndex(page._TAB_PLAN)
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


# ── the microscope camera belongs to the HOST, not the embedded page ──

class TestDoseIsEnteredAsAVolume(_PageCase):
    """The dose is typed in nL; the column depth is derived.

    The model layer stays µL throughout, and ``_push_pull_uL()`` is untouched, so
    the whole change is verifiable by exact equality against the value the old
    depth-primary field produced.
    """

    #: The operator's real saved profile at the time of the change: a column
    #: depth and NO volume key. Read from disk in
    #: `test_the_real_saved_profile_reproduces_its_exact_volume` when present.
    _REAL_PROFILE = "config/workflows/cell_targeting/__last__.json"

    def _page_with_needle(self, needle=None):
        page = self._make_page()
        page.set_hardware_config(_hw(needle or _single_needle()))
        return page

    def test_typing_a_dose_derives_the_column_depth(self):
        page = self._page_with_needle()
        area = page._needle_area_mm2()
        self.assertGreater(area, 0)
        page._push_volume.setValue(6.0)                     # 6 nL
        self.assertAlmostEqual(page._push_depth.value(),
                               (6.0 / 1000.0) / area, places=6)
        # The round trip is bounded by the hidden depth field's own quantum, not
        # by an arbitrary tolerance: at 6 decimals of mm that is ~2 pL here —
        # about 500× below one pump microstep, so it is invisible to the machine.
        quantum_uL = (10.0 ** -page._push_depth.decimals()) * area
        push, _pull = page._push_pull_uL()
        self.assertLess(abs(push - 6.0 / 1000.0), quantum_uL)
        self.assertLess(quantum_uL, 0.001,
                        "the derived column must be finer than one microstep")

    def test_a_bore_change_holds_the_dose_and_relengthens_the_column(self):
        """The dose is the operator's choice; the geometry follows it."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack_pumps("P1", "P2")))
        rows = page._setup_panel._rows
        page._push_volume.setValue(6.0)
        before_mm = page._push_depth.value()
        # Move the aspirating role onto the finer bore.
        rows[0].role.setCurrentIndex(rows[0].role.findData(BoreRole.IDLE.value))
        rows[1].role.setCurrentIndex(
            rows[1].role.findData(BoreRole.ASPIRATE_TARGET.value))
        self.assertAlmostEqual(page._push_volume.value(), 6.0,
                               msg="the dose must not change under the operator")
        self.assertNotAlmostEqual(page._push_depth.value(), before_mm,
                                  msg="the derived column must follow the bore")
        area_after = page._needle_area_mm2()
        push, _pull = page._push_pull_uL()
        quantum_uL = (10.0 ** -page._push_depth.decimals()) * area_after
        self.assertLess(abs(push - 6.0 / 1000.0), quantum_uL)

    def test_the_mirror_does_not_oscillate(self):
        page = self._page_with_needle()
        page._push_volume.setValue(6.0)
        for _ in range(5):
            page._sync_dose("volume")
            page._sync_dose("depth")
        self.assertAlmostEqual(page._push_volume.value(), 6.0, places=3)

    def test_an_unknown_bore_area_derives_nothing_and_says_so(self):
        page = self._make_page()          # no hardware config at all
        page._sync_dose("volume")
        self.assertIn("unknown", page._push_depth.specialValueText().lower())

    def test_a_legacy_profile_migrates_its_depth_once_then_volume_wins(self):
        """The migration cannot complete at load time — there is no bore area
        yet — so it is flagged and resolved at the first needle sync."""
        page = self._make_page()
        page._settings_dialog.apply({"push_depth": 0.10, "pull_mult": 2.0})
        self.assertTrue(page._dose_legacy_pending,
                        "a profile with no volume key must be flagged")
        page.set_hardware_config(_hw(_single_needle()))
        self.assertFalse(page._dose_legacy_pending, "must resolve exactly once")
        area = page._needle_area_mm2()
        self.assertAlmostEqual(page._push_volume.value(),
                               area * 0.10 * 1000.0, places=3)
        self.assertAlmostEqual(page._push_depth.value(), 0.10, places=6)
        # From here the volume is authoritative: a second config push must not
        # re-apply the saved depth over an edit the operator has since made.
        page._push_volume.setValue(6.0)
        page.set_hardware_config(_hw(_single_needle()))
        self.assertAlmostEqual(page._push_volume.value(), 6.0, places=3)

    def test_a_new_profile_with_a_volume_is_not_treated_as_legacy(self):
        page = self._make_page()
        page._settings_dialog.apply({"push_depth": 0.10,
                                     "push_volume_nL": 6.0})
        self.assertFalse(page._dose_legacy_pending)
        page.set_hardware_config(_hw(_single_needle()))
        self.assertAlmostEqual(page._push_volume.value(), 6.0, places=3)

    def test_the_profile_still_writes_push_depth_for_an_older_build(self):
        page = self._page_with_needle()
        page._push_volume.setValue(6.0)
        values = page._settings_dialog.collect()
        self.assertIn("push_depth", values)
        self.assertIn("push_volume_nL", values)
        self.assertAlmostEqual(values["push_depth"], page._push_depth.value(),
                               places=6)

    def test_the_real_saved_profile_reproduces_its_exact_volume(self):
        """GOLDEN FILE. The operator's own profile must run the same column.

        It carries ``push_depth: 0.1`` and no volume key, so it exercises the
        migration; the resulting ``_push_pull_uL()`` must equal what the
        depth-primary field produced, to the bit.
        """
        import json
        path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), *self._REAL_PROFILE.split("/"))
        if not os.path.exists(path):
            self.skipTest("the operator's saved profile is not on this machine")
        with open(path, encoding="utf-8") as fh:
            values = json.load(fh).get("values") or {}
        self.assertIn("push_depth", values, "fixture no longer exercises legacy")
        page = self._make_page()
        page._settings_dialog.apply(values)
        page.set_hardware_config(_hw(_single_needle()))
        area = page._needle_area_mm2()
        expected = area * float(values["push_depth"])
        push, pull = page._push_pull_uL()
        self.assertEqual(push, expected, "the metered column moved")
        self.assertEqual(pull, expected * float(values["pull_mult"]))


class TestOnlyApplicableColumnsShow(_PageCase):
    """Show only what applies — and prove each hidden cell is also UNREAD.

    HIDDEN ≠ UNREAD is the hazard: ``_collect_programs`` reads every cell
    regardless of visibility, so a hidden control whose value still reaches the
    machine would be a default silently governing a run.
    """

    def _visible(self, page, header_text):
        panel = page._setup_panel
        for h in panel._headers:
            if header_text.lower() in h.text().lower():
                return h.isVisibleTo(panel)
        self.fail(f"no {header_text!r} column")

    def _bare(self, needle=None):
        page = self._make_page()
        page.set_hardware_config(_hw(needle or _single_needle()))
        page.show()
        self._app.processEvents()
        self.addCleanup(page.hide)
        return page

    def test_a_single_bore_hides_the_mount_offset_column(self):
        page = self._bare()
        self.assertFalse(self._visible(page, "Mount offset"),
                         "bore 1 IS the datum; its offset is zero by definition")

    def test_a_backpack_shows_the_mount_offset_column(self):
        page = self._bare(_backpack())
        self.assertTrue(self._visible(page, "Mount offset"))

    def test_the_dose_columns_are_hidden_until_a_bore_is_dosing(self):
        page = self._bare(_backpack())
        for h in ("Dose", "depth", "Dose flow", "Lead time"):
            self.assertFalse(self._visible(page, h), f"{h} applies to nothing")
        _arm_dose(page)
        self._app.processEvents()
        for h in ("Dose", "depth", "Dose flow", "Lead time"):
            self.assertTrue(self._visible(page, h), f"{h} must appear when armed")

    def test_hiding_the_dose_columns_changes_NOTHING_in_the_config(self):
        """The proof that hiding is safe: with no dosing row the four cells are
        never consumed, so `trypsin_enabled` stays False whatever they hold."""
        page = self._bare(_backpack())
        before = page._current_config()
        row = page._setup_panel._rows[1]
        row.volume.setValue(row.volume.maximum())      # a wild hidden value
        row.rate.setValue(9.0)
        row.lead.setValue(120.0)
        after = page._current_config()
        self.assertFalse(after.trypsin_enabled)
        self.assertEqual(after.to_dict(), before.to_dict(),
                         "a hidden cell reached the config")

    def test_an_empty_target_type_library_hides_that_column(self):
        page = self._bare()
        page._setup_panel._target_types = []
        page._setup_panel._refresh_column_visibility()
        self.assertFalse(self._visible(page, "Target type"))

    def test_a_populated_library_shows_it(self):
        page = self._bare()
        page._setup_panel._target_types = [
            SimpleNamespace(id="t1", label="Bright FITC", color="#89b4fa")]
        page._setup_panel._refresh_column_visibility()
        self.assertTrue(self._visible(page, "Target type"))


class TestLiveTuning(_PageCase):
    """Mid-run edits reach the cells still to come, and are recorded per cell.

    Operator decision, overriding the safer fixed-recipe alternative — so the
    burden is on this code to make it thread-safe, bounded, applied only between
    operations, and knowable afterwards.
    """

    def _page(self, *, runnable=False):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: False
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            _single_needle(),
            inks={"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa")},
            locations={"Reagent": ["A1"]}))
        page.set_calibration_data(None, {"A1": (1000.0, 2000.0)}, 40.0)
        if runnable:
            page._prep_check.setChecked(False)
            page._clean_check.setChecked(False)
            page._wash_after_pickup_check.setChecked(False)
            page._reagent_combo.setCurrentIndex(
                page._reagent_combo.findData("Reagent"))
        return page

    def test_the_snapshot_is_published_and_readable_off_the_gui_thread(self):
        """The executor calls the provider from ITS OWN thread, so the provider
        must touch no Qt object — only an immutable dataclass under a lock."""
        import threading as _t
        page = self._page()
        page._dwell.setValue(90.0)
        got = {}

        def worker():
            got["tuning"] = page._current_tuning()

        th = _t.Thread(target=worker)
        th.start()
        th.join(timeout=5)
        tuning = got.get("tuning")
        self.assertIsNotNone(tuning, "nothing was published")
        self.assertAlmostEqual(tuning.incubation_s, 90.0)

    def test_the_snapshot_is_immutable(self):
        page = self._page()
        tuning = page._current_tuning()
        with self.assertRaises(Exception):
            tuning.incubation_s = 1.0

    def test_the_removal_height_is_resolved_to_zero_ref_by_the_GUI(self):
        """The executor must never have to resolve a plate frame."""
        page = self._page()
        page._removal_z.setValue(0.25)
        tuning = page._current_tuning()
        self.assertAlmostEqual(tuning.removal_z_zref_mm, 20.25)

    def test_an_out_of_bounds_tuning_is_NOT_published(self):
        """A dose larger than the bore holds would clamp the plunger and break
        the volume balance — the previous values keep running instead."""
        page = self._page()
        page._dwell.setValue(45.0)
        good = page._current_tuning()
        page._push_volume.setMaximum(1e9)          # defeat the field's own cap
        page._push_volume.setValue(900000.0)       # 900 µL through a ~0.4 µL bore
        still = page._current_tuning()
        self.assertIs(still, good, "an unbounded dose reached the machine")
        self.assertIn("more than", page._trial_status.text())

    def test_no_plate_bottom_means_no_tuning_is_published(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page.set_hardware_config(_hw(_single_needle()))
        page._controller.print_height_to_zref = lambda mm: None
        page._z_references["plate_bottom_z"] = None
        page._publish_tuning()
        self.assertIn("plate bottom", page._trial_status.text())

    def test_with_tuning_replaces_only_the_four_tunable_fields(self):
        """Structure must not change mid-run: a different reagent, bore or well
        must never reach a needle already loaded with something else, and prep
        has already conditioned the bores the original config named."""
        from SupportClasses.PickAndPlaceManager import CellRemovalConfig, LiveTuning
        cfg = CellRemovalConfig(
            reagent_bore="P1", trypsin_enabled=True, trypsin_bore="P2",
            trypsin_bore_index=1, aspirate_bore_index=0,
            dwell_time_s=60.0, extract_multiplier=2.0, pull_speed_uL_s=5.0,
            release_depth_mm=0.10)
        out = cfg.with_tuning(LiveTuning(
            incubation_s=90.0, extract_multiplier=3.0, pull_speed_uL_s=7.0,
            dose_volume_uL=0.006, release_depth_mm=0.19, removal_z_zref_mm=21.0))
        self.assertAlmostEqual(out.dwell_time_s, 90.0)
        self.assertAlmostEqual(out.extract_multiplier, 3.0)
        self.assertAlmostEqual(out.pull_speed_uL_s, 7.0)
        self.assertAlmostEqual(out.release_depth_mm, 0.19)
        # …and NOTHING structural moved.
        for attr in ("reagent_bore", "trypsin_enabled", "trypsin_bore",
                     "trypsin_bore_index", "aspirate_bore_index",
                     "trypsin_well_key", "push_speed_uL_s",
                     "place_z_offset_mm"):
            self.assertEqual(getattr(out, attr), getattr(cfg, attr), attr)

    def test_with_tuning_of_None_is_the_same_object(self):
        from SupportClasses.PickAndPlaceManager import CellRemovalConfig
        cfg = CellRemovalConfig()
        self.assertIs(cfg.with_tuning(None), cfg)

    def test_each_operation_gets_its_own_config_object(self):
        """A shared instance would let one edit retroactively rewrite the record
        of every cell already finished."""
        page = self._page(runnable=True)
        for i in range(3):
            page._picker.add_pick(5000.0 + i * 100, 6000.0)
            page._picker.add_place(7000.0 + i * 100, 8000.0)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
            self.assertTrue(Exec.return_value.execute_queue.called,
                            f"the run was refused: {page._status.text()}")
            queue = Exec.return_value.execute_queue.call_args[0][0]
        ids = [id(op.config) for op in queue.get_pending()]
        self.assertEqual(len(set(ids)), 3, "operations share one config object")
        # …and the executor was handed the provider, or nothing would apply.
        # `==`, not `is`: each attribute access builds a fresh bound method.
        self.assertEqual(Exec.return_value.tuning_provider, page._current_tuning)

    def test_a_test_try_runs_ONE_cell_and_the_next_un_tried_one(self):
        page = self._page(runnable=True)
        for i in range(3):
            page._picker.add_pick(5000.0 + i * 100, 6000.0)
            page._picker.add_place(7000.0 + i * 100, 8000.0)
        first = page._picker.picks()[0].target_id
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_test_one_cell()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
            self.assertTrue(Exec.return_value.execute_queue.called,
                            f"refused: {page._status.text()}")
            queue = Exec.return_value.execute_queue.call_args[0][0]
        ops = queue.get_pending()
        self.assertEqual(len(ops), 1, "a test try must run exactly one cell")
        self.assertEqual(ops[0].source_target.target_id, first)

    def test_a_second_try_advances_past_the_cell_already_done(self):
        page = self._page(runnable=True)
        for i in range(3):
            page._picker.add_pick(5000.0 + i * 100, 6000.0)
            page._picker.add_place(7000.0 + i * 100, 8000.0)
        picks = page._picker.picks()
        page._tried_target_ids.add(picks[0].target_id)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_test_one_cell()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
            queue = Exec.return_value.execute_queue.call_args[0][0]
        self.assertEqual(queue.get_pending()[0].source_target.target_id,
                         picks[1].target_id)

    def test_a_follow_up_try_reuses_the_needle_instead_of_re_prepping(self):
        page = self._page(runnable=True)
        page._prep_check.setChecked(True)
        page._clean_check.setChecked(True)
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        page._picker.add_pick(5100.0, 6000.0)
        page._picker.add_place(7100.0, 8000.0)
        page._service_z.setValue(0.5)
        page._tried_target_ids.add(page._picker.picks()[0].target_id)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_test_one_cell()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
            self.assertTrue(Exec.return_value.execute_queue.called,
                            f"refused: {page._status.text()}")
            ex = Exec.return_value
        self.assertFalse(ex.do_prep, "a follow-up try must reuse the needle")
        self.assertFalse(ex.do_post_clean,
                         "…and keep it loaded for the next try")

    def test_a_test_try_can_never_turn_ON_a_bracket_the_operator_disabled(self):
        page = self._page(runnable=True)
        page._prep_check.setChecked(False)
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_start(prep=True)              # a try asking for prep
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
            ex = Exec.return_value
        self.assertFalse(ex.do_prep, "the operator's own switch must win")

    def test_a_completed_cell_is_recorded_with_what_it_received(self):
        from SupportClasses.PickAndPlaceManager import LiveTuning
        page = self._page()
        op = SimpleNamespace(
            op_id="OP-1",
            source_target=SimpleNamespace(target_id="P001"),
            applied_tuning=LiveTuning(incubation_s=90.0, dose_volume_uL=0.0031))
        page._on_op_completed(op)
        self.assertEqual(page._run_log.count(), 1)
        text = page._run_log.item(0).text()
        self.assertIn("P001", text)
        self.assertIn("90.0 s", text)
        self.assertIn("3.100 nL", text)
        self.assertIn("P001", page._tried_target_ids)

    def test_the_in_progress_line_is_replaced_not_stacked(self):
        page = self._page()
        op = SimpleNamespace(op_id="OP-1",
                             source_target=SimpleNamespace(target_id="P001"),
                             applied_tuning=None)
        page._on_op_started(op)
        page._on_op_completed(op)
        self.assertEqual(page._run_log.count(), 1)
        self.assertTrue(page._run_log.item(0).text().startswith("✓"))


class TestTuningReachesOnlyTheCellsStillToCome(unittest.TestCase):
    """The executor half, with no Qt at all."""

    def _executor(self, n=3):
        from SupportClasses.PickAndPlaceManager import (
            CellRemovalConfig, OperationQueue, OperationType,
            PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
        )
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        import threading as _t
        ex._abort_flag = _t.Event()
        ex._pause_event = _t.Event()
        ex._pause_event.set()
        ex.do_prep = False
        ex.do_post_clean = False
        ex.pick_z_mm = 20.1
        ex.tuning_provider = None
        ex.on_op_started = ex.on_op_completed = None
        ex.on_op_failed = ex.on_sub_step = None
        ex._set_plate_floor = lambda on: None
        ex._retract_to_safe_z = lambda: None
        queue = OperationQueue()
        for i in range(n):
            queue.add(PickPlaceOperation(
                op_id=f"OP-{i}", op_type=OperationType.CELL_TARGET_REMOVAL,
                source_target=PickPlaceTarget(
                    target_id=f"P{i}", x_um=0.0, y_um=0.0, well_name=""),
                config=CellRemovalConfig(dwell_time_s=60.0)))
        return ex, queue

    def test_a_change_between_operations_reaches_only_the_later_ones(self):
        from SupportClasses.PickAndPlaceManager import LiveTuning
        ex, queue = self._executor(3)
        state = {"n": 0}

        def provider():
            # 60 s for the first cell, 90 s from the second on.
            return LiveTuning(incubation_s=60.0 if state["n"] == 0 else 90.0)

        ex.tuning_provider = provider

        def run_one(op):
            state["n"] += 1

        ex._execute_operation = run_one
        self.assertTrue(ex.execute_queue(queue))
        got = [op.config.dwell_time_s for op in queue.operations]
        self.assertEqual(got, [60.0, 90.0, 90.0])

    def test_what_each_cell_received_is_recorded_on_the_operation(self):
        from SupportClasses.PickAndPlaceManager import LiveTuning
        ex, queue = self._executor(2)
        ex.tuning_provider = lambda: LiveTuning(
            incubation_s=75.0, dose_volume_uL=0.0042)
        ex._execute_operation = lambda op: None
        ex.execute_queue(queue)
        for op in queue.operations:
            self.assertIsNotNone(op.applied_tuning)
            self.assertIn("75.0 s", op.applied_tuning.summary())
            self.assertIn("4.200 nL", op.applied_tuning.summary())

    def test_the_tuning_is_applied_BEFORE_the_operation_runs(self):
        """Never mid-operation: nothing may change under a needle already in a
        well. The loop top is the one point no operation is in flight."""
        from SupportClasses.PickAndPlaceManager import LiveTuning
        ex, queue = self._executor(1)
        ex.tuning_provider = lambda: LiveTuning(incubation_s=99.0)
        seen = {}
        ex._execute_operation = lambda op: seen.setdefault(
            "dwell_at_run_time", op.config.dwell_time_s)
        ex.execute_queue(queue)
        self.assertEqual(seen["dwell_at_run_time"], 99.0)

    def test_a_tuned_removal_height_goes_through_pick_z_mm(self):
        """So `_descend_z_mm`'s clearance guarantee and the armed plate-bottom
        floor apply to a tuned height exactly as to the original."""
        from SupportClasses.PickAndPlaceManager import LiveTuning
        ex, queue = self._executor(1)
        ex.tuning_provider = lambda: LiveTuning(removal_z_zref_mm=21.5)
        ex._execute_operation = lambda op: None
        ex.execute_queue(queue)
        self.assertAlmostEqual(ex.pick_z_mm, 21.5)

    def test_no_provider_leaves_the_queued_config_untouched(self):
        """Pre-v7.9 behaviour, byte-identical."""
        ex, queue = self._executor(2)
        before = [op.config for op in queue.operations]
        ex._execute_operation = lambda op: None
        ex.execute_queue(queue)
        for op, cfg in zip(queue.operations, before):
            self.assertIs(op.config, cfg)
            self.assertIsNone(op.applied_tuning)

    def test_a_provider_that_raises_does_not_stop_the_run(self):
        def boom():
            raise RuntimeError("provider exploded")
        ex, queue = self._executor(2)
        ex.tuning_provider = boom
        ex._execute_operation = lambda op: None
        self.assertTrue(ex.execute_queue(queue))


class TestTheLiveFrameOverMosaic(_PageCase):
    """The mosaic is the map; the live camera is "you are here"."""

    _EXTENT = (10000.0, 20000.0, 11000.0, 20800.0)
    _SCALE = 0.5                      # px per µm
    _SHIFT = (7.0, -3.0)              # non-zero on purpose

    def _page(self, *, has_shift=True, stage=(10400.0, 20300.0), frame=(64, 48)):
        import numpy as np
        mgr = _real_manager()
        page = self._make_page(camera_manager=mgr)
        page._safe_z = 40.0
        page.set_hardware_config(_hw(_single_needle()))
        # A real frame, so the pixmap conversion is exercised rather than faked.
        img = np.zeros((frame[1], frame[0], 3), dtype=np.uint8)
        img[:] = (32, 64, 128)
        mgr.get_current_frame = lambda idx: img
        mgr.effective_um_per_px = lambda idx, w: 4.0
        mgr.full_orientation = lambda idx: (False, False, 0.0)
        page._controller.get_xy_position = lambda cached=True: stage
        page._mosaic_context = lambda channel=None: {
            "extent_um": self._EXTENT, "mosaic_scale": self._SCALE,
            "shift_um": self._SHIFT, "has_shift": has_shift,
            "um_per_px": 4.0, "image": img, "well": "A1",
            "scale_warning": "",
        }
        return page, mgr

    def test_the_frame_is_drawn_where_the_transform_says_the_stage_is(self):
        """One transform, shared with the motion path.

        The overlay uses ``forward_project_um`` — the exact inverse of the
        ``back_project_px`` the travel handler uses — so the frame cannot be
        drawn somewhere the stage is not.
        """
        from SupportClasses.SpheroidDetector import back_project_px
        page, _mgr = self._page()
        pt = page._camera_scene_point()
        self.assertIsNotNone(pt)
        # Round-trip it back through the MOTION path's own function.
        x_um, y_um = back_project_px((pt.x(), pt.y()), self._EXTENT,
                                     self._SCALE, self._SHIFT)
        self.assertAlmostEqual(x_um, 10400.0, places=6)
        self.assertAlmostEqual(y_um, 20300.0, places=6)

    def test_the_frame_is_scaled_to_the_mosaics_own_micron_scale(self):
        page, _mgr = self._page(frame=(64, 48))
        page._refresh_live_frame_overlay()
        item = page._live_frame_item
        self.assertIsNotNone(item)
        r = item.mapRectToScene(item.boundingRect())
        # 64 camera px × 4 µm/px = 256 µm; × 0.5 px/µm = 128 mosaic px.
        self.assertAlmostEqual(r.width(), 128.0, places=3)
        self.assertAlmostEqual(r.height(), 96.0, places=3)
        # …and centred on the stage position, not cornered at it.
        pt = page._camera_scene_point()
        self.assertAlmostEqual(r.center().x(), pt.x(), places=3)
        self.assertAlmostEqual(r.center().y(), pt.y(), places=3)

    def test_a_mirrored_camera_flips_the_frame_not_its_position(self):
        """⚠ ``full_orientation`` returns (flip_x, flip_y, rotation) — flips
        FIRST. Unpacking it rotation-first swaps a mirror for an angle."""
        page, mgr = self._page()
        mgr.full_orientation = lambda idx: (True, False, 0.0)
        page._refresh_live_frame_overlay()
        rot, flip_x, flip_y = page._live_frame_orientation()
        self.assertEqual((rot, flip_x, flip_y), (0.0, True, False))
        item = page._live_frame_item
        r = item.mapRectToScene(item.boundingRect())
        pt = page._camera_scene_point()
        self.assertAlmostEqual(r.center().x(), pt.x(), places=3)
        self.assertAlmostEqual(r.width(), 128.0, places=3)

    def test_a_rotated_camera_rotates_the_frame(self):
        page, mgr = self._page(frame=(64, 48))
        mgr.full_orientation = lambda idx: (False, False, 90.0)
        page._refresh_live_frame_overlay()
        r = page._live_frame_item.mapRectToScene(
            page._live_frame_item.boundingRect())
        # 90° swaps the drawn extents.
        self.assertAlmostEqual(r.width(), 96.0, places=3)
        self.assertAlmostEqual(r.height(), 128.0, places=3)

    def test_the_overlay_is_rebuilt_after_the_mosaic_is_replaced(self):
        """``_ZoomImageView.set_image`` calls ``scene.clear()``, which DELETES
        the C++ object — the Python wrapper this page holds outlives it and
        raises on any access. So validity is checked, and the item recreated."""
        from PySide6.QtGui import QPixmap
        page, _mgr = self._page()
        page._refresh_live_frame_overlay()
        view = page._scan_page.mosaic_view()
        first = page._live_frame_item
        view.set_image(QPixmap(400, 300))            # destroys host items
        self.assertIsNone(page._live_frame_item_or_none(),
                          "a deleted item must not be reported as live")
        page._refresh_live_frame_overlay()           # must not raise
        self.assertIsNotNone(page._live_frame_item)
        self.assertIsNot(page._live_frame_item, first)
        self.assertIs(page._live_frame_item.scene(), view.scene_obj())

    def test_no_frame_removes_the_overlay_rather_than_stranding_it(self):
        page, mgr = self._page()
        page._refresh_live_frame_overlay()
        self.assertIsNotNone(page._live_frame_item.scene())
        mgr.get_current_frame = lambda idx: None
        page._refresh_live_frame_overlay()
        self.assertIsNone(page._live_frame_item.scene())

    def test_an_unregistered_mosaic_is_labelled_and_still_blocks_start(self):
        page, _mgr = self._page(has_shift=False)
        page._refresh_well_view_note()
        note = page._well_view_note.text().lower()
        self.assertIn("registration", note)
        self.assertIn("line up", note)
        self.assertFalse(page._mosaic_can_command_motion(),
                         "an unregistered mosaic must still refuse motion")


class TestFollowingTheCamera(_PageCase):
    """Map-app model: follow, drag to break away, click the toggle to resume."""

    def _view(self):
        page = self._make_page()
        return page, page._scan_page.mosaic_view()

    def test_following_is_on_by_default(self):
        page, view = self._view()
        self.assertTrue(view.is_following())
        self.assertTrue(page._follow_btn.isChecked())

    def test_a_position_update_recentres_while_following(self):
        from PySide6.QtCore import QPointF
        from PySide6.QtGui import QPixmap
        _page, view = self._view()
        view.set_image(QPixmap(2000, 2000))
        view.resize(200, 200)
        # `set_image` fits the whole mosaic, which leaves no scroll range — so
        # zoom in first, otherwise "centre on a point" is not expressible.
        view.scale(8.0, 8.0)
        view.follow_point(QPointF(1500.0, 400.0))
        c = view.mapToScene(view.viewport().rect().center())
        self.assertAlmostEqual(c.x(), 1500.0, delta=8.0)
        self.assertAlmostEqual(c.y(), 400.0, delta=8.0)

    def test_zooming_does_NOT_break_the_follow_and_stays_centred(self):
        """Zooming out to see the mosaic around the current position is the
        entire point of following, so the wheel must not unlock it."""
        from PySide6.QtCore import QPoint, QPointF, Qt
        from PySide6.QtGui import QPixmap, QWheelEvent
        _page, view = self._view()
        view.set_image(QPixmap(2000, 2000))
        view.resize(200, 200)
        view.scale(16.0, 16.0)
        view.follow_point(QPointF(1500.0, 400.0))
        ev = QWheelEvent(
            QPointF(10.0, 10.0), QPointF(10.0, 10.0), QPoint(0, 0),
            QPoint(0, -120), Qt.MouseButton.NoButton,
            Qt.KeyboardModifier.NoModifier, Qt.ScrollPhase.NoScrollPhase, False)
        view.wheelEvent(ev)
        self.assertTrue(view.is_following(), "the wheel broke the follow")
        c = view.mapToScene(view.viewport().rect().center())
        self.assertAlmostEqual(c.x(), 1500.0, delta=12.0)

    def test_a_hand_drag_breaks_the_follow_and_the_toggle_reflects_it(self):
        from PySide6.QtCore import QPoint, QPointF, Qt
        from PySide6.QtGui import QMouseEvent, QPixmap
        page, view = self._view()
        page._set_well_view_mode("hand")
        view.set_image(QPixmap(2000, 2000))
        view.resize(200, 200)
        self.assertTrue(view.is_following())
        ev = QMouseEvent(QMouseEvent.Type.MouseMove, QPointF(30.0, 30.0),
                         Qt.MouseButton.NoButton, Qt.MouseButton.LeftButton,
                         Qt.KeyboardModifier.NoModifier)
        view.mouseMoveEvent(ev)
        self.assertFalse(view.is_following(), "dragging must break away")
        self.assertFalse(page._follow_btn.isChecked(),
                         "the toggle must reflect what the view is doing")

    def test_a_middle_drag_breaks_the_follow_in_point_mode_too(self):
        from PySide6.QtCore import QPointF, Qt
        from PySide6.QtGui import QMouseEvent, QPixmap
        page, view = self._view()
        page._set_well_view_mode("point")
        view.set_image(QPixmap(2000, 2000))
        ev = QMouseEvent(QMouseEvent.Type.MouseButtonPress, QPointF(30.0, 30.0),
                         Qt.MouseButton.MiddleButton, Qt.MouseButton.MiddleButton,
                         Qt.KeyboardModifier.NoModifier)
        view.mousePressEvent(ev)
        self.assertFalse(view.is_following())

    def test_a_programmatic_recentre_does_NOT_break_the_follow(self):
        """The break must come from the DRAG, not from scrollbar changes —
        ``centerOn`` moves them too, so the view would fight its own updates."""
        from PySide6.QtCore import QPointF
        from PySide6.QtGui import QPixmap
        _page, view = self._view()
        view.set_image(QPixmap(2000, 2000))
        view.resize(200, 200)
        view.scale(8.0, 8.0)
        for x in (100.0, 900.0, 1700.0):
            view.follow_point(QPointF(x, x))
            self.assertTrue(view.is_following())

    def test_the_toggle_resumes_following(self):
        page, view = self._view()
        page._follow_btn.setChecked(False)
        self.assertFalse(view.is_following())
        page._follow_btn.setChecked(True)
        self.assertTrue(view.is_following())

    def test_hand_mode_pans_and_point_mode_clicks(self):
        page, view = self._view()
        page._set_well_view_mode("hand")
        self.assertFalse(view.interactive_items(), "hand mode must pan on left")
        page._set_well_view_mode("point")
        self.assertTrue(view.interactive_items(), "point mode must click on left")

    def test_travelling_re_arms_the_follow(self):
        """Sending the camera somewhere is the reason you panned away."""
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._follow_btn.setChecked(False)
        page._travel_worker = MagicMock()
        self.assertTrue(page._travel_to_absolute(1000.0, 2000.0))
        self.assertTrue(page._follow_btn.isChecked())
        self.assertTrue(page._scan_page.mosaic_view().is_following())

    def test_a_refused_travel_does_not_move_and_the_gate_still_holds(self):
        page = self._make_page(ctrl=_mock_ctrl(xy=False))
        page._travel_worker = MagicMock()
        self.assertFalse(page._travel_to_absolute(1000.0, 2000.0))
        page._travel_worker.start.assert_not_called()


class TestHostOwnsTheCamera(_PageCase):
    """Before this, the camera was started ONLY by the embedded scan page's
    ``showEvent``, so ``LiveTargetPicker`` got frames purely because it shared a
    tab with it. Any layout that separated them left the picker showing a dead
    feed — which is why this lands before the tabs are restructured.
    """

    def _page(self):
        mgr = _real_manager()
        page = self._make_page(camera_manager=mgr)
        return page, mgr

    def test_showing_the_page_starts_the_microscope_camera_exactly_once(self):
        page, mgr = self._page()
        slot = page._camera_slot()
        cam = mgr._cameras[slot]
        self.assertEqual(cam.starts, 0)
        page.show()
        self._app.processEvents()
        self.assertTrue(mgr.is_running(slot))
        page.hide()
        self._app.processEvents()
        page.show()
        self._app.processEvents()
        # One start per transition into view, never two for one show.
        self.assertEqual(cam.starts, 2, "show/hide/show should start twice")
        self.assertEqual(cam.stops, 1)
        page.hide()

    def test_hiding_the_page_stops_a_camera_the_host_started(self):
        page, mgr = self._page()
        page.show()
        self._app.processEvents()
        page.hide()
        self._app.processEvents()
        self.assertFalse(mgr.is_running(page._camera_slot()))
        self.assertFalse(page._camera_started_by_us)

    def test_a_camera_already_running_is_left_alone_on_hide(self):
        """Someone else's feed is not ours to switch off."""
        page, mgr = self._page()
        slot = page._camera_slot()
        mgr.start(slot)                       # a different page owns it
        page.show()
        self._app.processEvents()
        self.assertFalse(page._camera_started_by_us)
        page.hide()
        self._app.processEvents()
        self.assertTrue(mgr.is_running(slot),
                        "hid a camera this page did not start")

    def test_a_running_scan_is_not_left_without_a_camera(self):
        """`_stop_camera` must not pull frames from under a scan worker.

        The embedded page's own ``hideEvent`` stop had exactly this hazard; the
        host takes ownership, so the host is where the guard has to live.
        """
        page, mgr = self._page()
        page.show()
        self._app.processEvents()
        slot = page._camera_slot()
        page._scan_page.is_scanning = lambda: True
        page.hide()
        self._app.processEvents()
        self.assertTrue(mgr.is_running(slot),
                        "stopped the camera under a running scan")
        self.assertTrue(page._camera_started_by_us,
                        "ownership must survive a skipped stop")

    def test_the_embedded_page_does_not_claim_ownership_of_the_camera(self):
        """The latent hazard this closes.

        The embedded page only takes ownership when it finds the camera stopped.
        With the host starting it first, the embedded page's own ``hideEvent``
        stop becomes a no-op — so switching surfaces mid-scan can no longer cut
        the feed out from under its scan worker.
        """
        page, _mgr = self._page()
        page.show()
        self._app.processEvents()
        page._tabs.setCurrentIndex(page._TAB_PLAN)
        self._app.processEvents()
        self.assertTrue(page._camera_started_by_us, "the host must own it")
        self.assertFalse(page._scan_page._camera_started_by_us,
                         "the embedded page claimed a camera it did not start")
        page.hide()

    def test_ownership_is_declared_not_won_by_show_order(self):
        """Qt delivers ``showEvent`` to a CHILD BEFORE ITS PARENT (verified).

        So "whoever finds the camera stopped owns it" is a race the embedded page
        always wins once it sits on the host's visible tab — it claimed the
        camera, then stopped it from its own ``hideEvent`` while the host was
        still using it. This is what the Plan/Run split exposed, and why the
        embedded page is constructed with ``owns_camera=False``.
        """
        page, mgr = self._page()
        self.assertFalse(page._scan_page._owns_camera)
        page.show()
        self._app.processEvents()
        slot = page._camera_slot()
        # The scan page is ON the visible tab and got its showEvent first…
        self.assertTrue(page._scan_page.isVisible())
        # …and still did not claim the camera.
        self.assertFalse(page._scan_page._camera_started_by_us)
        self.assertTrue(page._camera_started_by_us)
        # Its own stop is a no-op, so a child hide cannot cut the host's feed.
        page._scan_page._stop_camera()
        self.assertTrue(mgr.is_running(slot))
        page.hide()

    def test_the_host_does_not_keep_its_own_copy_of_the_slot_resolution(self):
        """Two copies drift: a host starting slot 0 while the picker maps clicks
        through slot 2 would project them through the wrong µm/px."""
        page, _mgr = self._page()
        page._picker._cam_idx = 2
        self.assertEqual(page._camera_slot(), 2)

    def test_no_camera_manager_is_survivable(self):
        page = self._make_page(camera_manager=None)
        page.show()
        self._app.processEvents()
        page.hide()                            # must not raise


# ── Start gating for a dedicated pushing bore ────────────────────────

class TestTheGuiDoesNotLie(_PageCase):
    """Stage 4: every surface that showed one thing while the machine did another."""

    def _two_bore_page(self, needle=None):
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            needle or _backpack(),
            inks={"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa"),
                  "Trypsin": SimpleNamespace(ink_type="ink", color="#f38ba8")},
            locations={"Reagent": ["A1"], "Trypsin": ["B1"]}))
        page.set_calibration_data(None, {"A1": (1000.0, 2000.0),
                                         "B1": (3000.0, 4000.0)}, 40.0)
        return page

    # ── A3: two controls no longer share a label ──
    def test_the_bore_table_says_DOSE_not_PUSH(self):
        """Two live controls named 'Push depth' 15 cm apart, same units,
        different meanings, is a wrong-field entry waiting to happen."""
        page = self._make_page()
        panel = page._setup_panel
        headers = [panel._grid.itemAtPosition(0, c).widget().text()
                   for c in range(panel._grid.columnCount())
                   if panel._grid.itemAtPosition(0, c) is not None]
        self.assertIn("Dose", headers)
        self.assertIn("Dose flow", headers)
        self.assertNotIn("Push depth", headers)
        self.assertNotIn("Push volume", headers)
        # The derived column depth is marked as such ("≙"), not offered as a
        # second name for the dose.
        self.assertTrue(any("depth" in h.lower() for h in headers))

    def test_the_column_note_names_which_bore_it_applies_to(self):
        page = self._two_bore_page()
        self.assertIn("bore 1", page._column_note.text())

    # ── A6: the magic zero is gone, and the holdup cap is in nL ──
    def test_a_zero_dose_is_no_longer_expressible(self):
        """The magic 0 ("derive it from the depth instead") is retired.

        A field minimum is a stronger guard than a special value text: there is
        no longer a mode where the volume and the depth disagree about the dose,
        because the depth is derived from the volume.
        """
        page = self._two_bore_page()
        row = page._setup_panel._rows[1]
        self.assertGreater(row.volume.minimum(), 0.0)
        self.assertEqual(row.volume.specialValueText(), "",
                         "no magic value left to explain")
        self.assertTrue(row.depth.isReadOnly(),
                        "the depth must not be a second place to type the dose")

    def test_the_dose_is_capped_at_what_the_bore_holds(self):
        """A decimal-place slip asked far more than the bore holds; the plunger
        then clamped and broke the volume balance with no report.

        ⚠ The field is nL and the bore's holdup is µL — the ×1000 is the point.
        """
        page = self._two_bore_page()
        row = page._setup_panel._rows[1]
        bore = page._hw_config.needle.bore(1)
        cap_nL = bore.internal_volume_uL * 1000.0
        self.assertLessEqual(row.volume.maximum(), cap_nL + 1e-6)
        self.assertGreater(row.volume.maximum(), 0.0)
        # A dose at the cap is at most the bore's holdup, never over it — the cap
        # is FLOORED to the field's precision, because a cap that rounds up would
        # admit a dose fractionally larger than the bore can contain.
        row.volume.setValue(row.volume.maximum())
        self.assertLessEqual(row.volume_uL(), bore.internal_volume_uL)
        one_step_uL = (10.0 ** -row.volume.decimals()) / 1000.0
        self.assertGreater(row.volume_uL(),
                           bore.internal_volume_uL - one_step_uL,
                           "the cap gave away more than one field step")

    # ── A4: the clamped flow is shown, not just logged ──
    def test_a_clamped_dose_flow_is_reported_with_its_consequence(self):
        page = self._two_bore_page()
        _arm_dose(page)
        page._setup_panel.set_flow_ceilings({"P2": 0.05})
        row = page._setup_panel._rows[1]
        row.rate.setValue(5.0)
        notes = page._setup_panel.validation_notes()
        joined = " ".join(notes)
        self.assertIn("auto-limited", joined)
        self.assertIn("lead time", joined, "the timing consequence is the point")

    def test_the_status_shows_the_rate_that_will_actually_run(self):
        page = self._two_bore_page()
        _arm_dose(page)
        page._setup_panel.set_flow_ceilings({"P2": 0.05})
        row = page._setup_panel._rows[1]
        row.rate.setValue(5.0)
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        text = page._setup_panel.trypsin_status.text()
        self.assertIn("0.05", text)
        self.assertIn("auto-limited", text)

    def test_an_unknown_ceiling_does_not_fabricate_a_limit(self):
        """A bare MagicMock's __float__ is 1.0 — it must not become a ceiling."""
        page = self._two_bore_page()
        page._setup_panel.set_flow_ceilings({"P2": MagicMock()})
        self.assertIsNone(page._setup_panel.flow_ceiling_for("P2"))

    # ── A5: no green tick for something that cannot be checked ──
    def test_an_unmeasured_dosing_bore_is_not_painted_green(self):
        """A pump can displace a volume through a bore of unknown bore, so the
        dose IS deliverable — but nothing can check it against what the bore
        holds, or against that bore's own flow ceiling. That is what the retired
        depth-derived zero was really reporting.
        """
        page = self._two_bore_page(
            needle=NeedleSpec(
                gauge=22, od_um=400.0, id_um=200.0, wall_um=100.0,
                needle_form=NEEDLE_FORM_BACKPACK,
                bores=[_bore(200.0, "P1"),
                       NeedleBore(id_um=0.0, od_um=0.0, pump_id="P2",
                                  offset_um=(250.0, -120.0))]))
        _arm_dose(page)
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        text = page._setup_panel.trypsin_status.text()
        self.assertNotIn("✓", text)
        self.assertIn("Needle", text, "must name where to fix it")
        self.assertIn("holds", text, "must name what cannot be checked")
        # And the derived depth cell says it cannot be derived rather than
        # showing a plausible-looking number.
        row = page._setup_panel._rows[1]
        self.assertIn("unknown", row.depth.specialValueText().lower())

    # ── A2: the dosing bore REPLACES the aspirate-side push ──
    def test_the_release_controls_grey_out_when_a_dosing_bore_is_armed(self):
        page = self._two_bore_page()
        self.assertTrue(page._reagent_combo.isEnabled())
        _arm_dose(page)
        self.assertFalse(page._reagent_combo.isEnabled())
        self.assertIn("loads nothing", page._reagent_combo.toolTip())

    def test_the_reagent_status_stops_promising_a_load(self):
        page = self._two_bore_page()
        _arm_dose(page)
        self.assertIn("loads nothing", page._reagent_status.text())

    # ── The reagent DIP HEIGHT is not part of that greying ──
    def test_the_reagent_dip_z_stays_live_with_a_dosing_bore_armed(self):
        """A dosing bore does not stop the needle dipping into a reagent well.

        `_execute_cell_removal` step 0 travels the TRYPSIN bore to its own
        reagent well with `target_z_mm=self.reagent_dip_z_mm` — so arming a
        dosing bore makes this height MORE load-bearing, not less. It was
        greyed out and captioned "Not used" only because it had been filed
        under a group titled "single-bore sequence".
        """
        page = self._two_bore_page()
        _arm_dose(page)
        self.assertTrue(
            page._reagent_z.isEnabled(),
            "the dip Z governs the dosing bore's own reagent well")
        tip = page._reagent_z.toolTip().lower()
        self.assertNotIn("not used", tip)

    def test_the_reagent_dip_z_is_laid_out_with_the_other_heights(self):
        """Where a control lives is why the greying bug happened at all."""
        from gui.widgets.components import Card
        page = self._two_bore_page()
        panel = page._setup_panel
        titles = [c.title() for c in panel.findChildren(Card)
                  if c.isAncestorOf(page._reagent_z)]
        self.assertTrue(titles, "_reagent_z must be laid out on the panel")
        self.assertTrue(
            any("height" in t.lower() for t in titles),
            f"the dip Z belongs with the heights, not in {titles}")
        # And it still reaches the executor unchanged.
        self.assertTrue(any(key == "reagent_z"
                            for key, _a, _d in page._PROMOTED_DEFAULTS))

    def test_start_no_longer_demands_an_aspirate_side_reagent(self):
        page = self._two_bore_page()
        _arm_dose(page)
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        page._prep_check.setChecked(False)
        page._clean_check.setChecked(False)
        page._wash_after_pickup_check.setChecked(False)
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
        self.assertTrue(Exec.called,
                        f"run refused with no aspirate reagent: "
                        f"{page._status.text()}")

    def test_start_STILL_demands_it_without_a_dosing_bore(self):
        page = self._two_bore_page()
        page._prep_check.setChecked(False)
        page._clean_check.setChecked(False)
        page._wash_after_pickup_check.setChecked(False)
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        page._on_start()
        self.assertIsNone(page._exec_thread)
        self.assertIn("cell-release reagent", page._status.text().lower())


class TestReadinessSurfaces(_PageCase):
    """One evaluation must drive the card, the status line AND the button, so a
    green tick beside something Start refuses becomes structurally impossible."""

    def test_a_bare_page_has_a_readiness_list_and_a_goto_button(self):
        page = self._make_page()
        self.assertIsNotNone(page._ready_list)
        self.assertIsNotNone(page._setup_panel.goto_targets_btn)

    def test_the_status_line_states_the_blocker_instead_of_Idle(self):
        page = self._make_page()
        page._refresh_readiness()
        text = page._status.text()
        self.assertNotEqual(text, "Idle.")
        self.assertIn("Not ready", text)

    def test_the_start_button_ALWAYS_carries_a_tooltip(self):
        """A disabled button with no tooltip was the dead end this replaces."""
        page = self._make_page()
        page._refresh_readiness()
        self.assertFalse(page._start_btn.isEnabled())
        self.assertTrue(page._start_btn.toolTip().strip(),
                        "a disabled Start must explain itself")

    def test_the_tooltip_says_where_to_pick_the_cells(self):
        """A disabled Start must name a surface that exists and what to do on
        it. It must also never be EMPTY — a disabled button with no tooltip and
        an "Idle." status line was the original dead end."""
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page.set_hardware_config(_hw(_single_needle()))
        page._refresh_readiness()
        tip = page._start_btn.toolTip()
        self.assertTrue(tip.strip(), "a disabled Start with no explanation")
        self.assertIn("Plan", tip)
        self.assertNotIn("Well Survey", tip, "names a tab that no longer exists")

    def test_the_goto_button_brings_the_well_into_view(self):
        """The cells are chosen on Plan, beside the protocol."""
        page = self._make_page()
        page._tabs.setCurrentIndex(page._TAB_RUN)
        page._setup_panel.goto_targets_btn.click()
        self.assertEqual(page._tabs.currentIndex(), page._TAB_PLAN)

    def test_the_readiness_context_survives_a_bare_page(self):
        """The context builder is where mock tolerance belongs."""
        page = self._make_page()
        ctx = page._readiness_context()
        self.assertIsNotNone(ctx)
        page._refresh_readiness()          # must not raise

    def test_the_context_reports_the_unmeasured_offset(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page.set_hardware_config(_hw(_backpack(offset=(0.0, 0.0), z_off=0.0)))
        ctx = page._readiness_context()
        second = [b for b in ctx.bores if b.index == 1]
        self.assertTrue(second)
        self.assertFalse(second[0].offset_measured)

    def test_the_context_reports_a_measured_offset(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page.set_hardware_config(_hw(_backpack()))
        ctx = page._readiness_context()
        second = [b for b in ctx.bores if b.index == 1]
        self.assertTrue(second)
        self.assertTrue(second[0].offset_measured)

    def test_a_mock_controller_cannot_fabricate_a_flow_ceiling(self):
        """A bare MagicMock's __float__ is 1.0 — it must not become a limit."""
        page = self._make_page(ctrl=_mock_ctrl())
        page.set_hardware_config(_hw(_backpack()))
        ctx = page._readiness_context()
        for b in ctx.bores:
            self.assertIsNone(b.flow_ceiling_uL_s)


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
        # These tests are ABOUT the dosing bore, so arm it — a second bore no
        # longer takes that role implicitly.
        _arm_dose(page)
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

    def test_an_unmeasured_two_bore_assembly_is_BLOCKED_not_merely_warned(self):
        """⚠ DELIBERATE BEHAVIOUR CHANGE (post-v7.9 audit).

        This used to be advisory text appended to the status line — and appended
        AFTER the executor thread had already started. Its failure mode is total:
        with a zero measured separation the shift between the dose and the
        aspirate moves nothing, so every cell is dosed and then aspirated
        100-500 µm away from. The whole run destroys its cells and collects none,
        with no error. So it now blocks Start, like the mosaic-shift gate whose
        reasoning it shares.
        """
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
        _arm_dose(page)
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.return_value.execute_queue = MagicMock(return_value=True)
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
        self.assertFalse(Exec.called, "the run must NOT have started")
        self.assertIsNone(page._exec_thread)
        msg = page._status.text().lower()
        self.assertIn("offset", msg)
        self.assertIn("needle location", msg, "must name where to fix it")

    def test_prep_bores_is_wired_so_simultaneous_prep_actually_runs(self):
        """⚠ THE MISSING LINK. `prep_bores` had no production writer at all, so
        `_prep_bore_plan()` always returned [] and `move_pumps_uL` had ZERO live
        call sites — a dosing bore was never conditioned and arrived full of AIR.
        """
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: False
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        # Prep needs the four service wells assigned AND calibrated.
        service = {"Waste": "waste", "Oil": "oil", "Wash": "wash",
                   "Buffer": "buffer"}
        inks = {"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa"),
                "Trypsin": SimpleNamespace(ink_type="ink", color="#f38ba8")}
        locations = {"Reagent": ["A1"], "Trypsin": ["B1"]}
        wells = {"A1": (1000.0, 2000.0), "B1": (3000.0, 4000.0)}
        for i, (name, role) in enumerate(service.items()):
            inks[name] = SimpleNamespace(ink_type=role, color="#6c7086")
            cell = f"C{i + 1}"
            locations[name] = [cell]
            wells[cell] = (5000.0 + i * 100, 6000.0)
        page.set_hardware_config(_hw(_backpack(), inks=inks,
                                     locations=locations))
        page.set_calibration_data(None, wells, 40.0)
        page._wash_after_pickup_check.setChecked(False)
        page._reagent_combo.setCurrentIndex(
            page._reagent_combo.findData("Reagent"))
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        _arm_dose(page)
        page._prep_check.setChecked(True)
        page._clean_check.setChecked(True)
        with patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            executor = Exec.return_value
            executor.execute_queue = MagicMock(return_value=True)
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
        self.assertTrue(Exec.called, f"run refused: {page._status.text()}")
        bores = executor.prep_bores
        self.assertIsInstance(bores, list)
        self.assertEqual({e["pump_id"] for e in bores}, {"P1", "P2"},
                         "BOTH the aspirating and the dosing bore must be prepped")

    def test_a_single_bore_run_still_preps_only_its_one_bore(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: False
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            _single_needle(),
            inks={"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa")},
            locations={"Reagent": ["A1"]}))
        page.set_calibration_data(None, {"A1": (1000.0, 2000.0)}, 40.0)
        page._prep_check.setChecked(False)
        page._clean_check.setChecked(False)
        page._wash_after_pickup_check.setChecked(False)
        page._reagent_combo.setCurrentIndex(
            page._reagent_combo.findData("Reagent"))
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        cfg = page._current_config()
        self.assertEqual(len(cfg.active_bores()), 1)


class TestHelpAndSilentLoss(_PageCase):
    """Stage 5: help that was unreachable, and target-type assignments that were
    permanently erased by a renamed file."""

    # ── B3: the help_text was dead weight ──
    def test_the_panel_collects_its_form_rows(self):
        page = self._make_page()
        self.assertTrue(page._setup_panel._help_rows,
                        "the promoted rows must be collected for the Help toggle")

    def test_register_help_rows_walks_up_to_a_registrar(self):
        """FormRow hides its help label at construction; ONLY
        MainWindow.register_form_row ever reveals it."""
        from PySide6.QtWidgets import QWidget as _QW
        page = self._make_page()
        seen = []

        class _Host(_QW):
            def register_form_row(self, row):
                seen.append(row)
        host = _Host()
        page.setParent(host)
        page._setup_panel.register_help_rows()
        self.assertTrue(seen, "no rows reached the registrar")
        self.assertEqual(len(seen), len(page._setup_panel._help_rows))
        page.setParent(None)

    def test_a_page_with_no_registrar_does_not_raise(self):
        page = self._make_page()
        page._setup_panel.register_help_rows()      # best-effort

    def test_help_mode_toggles_the_bore_table_legend(self):
        """The table's cells cannot be FormRows (shared grid for column
        alignment), so help mode gets an honest substitute."""
        page = self._make_page()
        panel = page._setup_panel
        self.assertFalse(panel._help_legend.isVisibleTo(panel))
        panel.set_help_mode(True)
        self.assertTrue(panel._help_legend.isVisibleTo(panel))
        panel.set_help_mode(False)
        self.assertFalse(panel._help_legend.isVisibleTo(panel))

    # ── B4: the offset warning names its page ──
    def test_the_unmeasured_offset_names_where_to_measure_it(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page.set_hardware_config(_hw(_backpack(offset=(0.0, 0.0), z_off=0.0)))
        row = page._setup_panel._rows[1]
        self.assertIn("Needle Location", row.offset.text())
        self.assertIn("Needle Location", row.offset.toolTip())

    # ── B5: silent loss of a target-type assignment ──
    def test_a_missing_target_type_survives_as_a_placeholder(self):
        """⚠ PERMANENT-LOSS FIX. An unresolvable id used to be silently dropped;
        the next row edit then rewrote the profile with an empty id, so renaming
        one JSON file erased every assignment with no message."""
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        panel = page._setup_panel
        panel.apply_state({
            "bore_programs": [
                {"bore_index": 0, "role": "aspirate_target", "pump_id": "P1"},
                {"bore_index": 1, "role": "push_reagent", "pump_id": "P2",
                 "target_type_id": "gone-from-disk",
                 "target_type_name": "My cells"},
            ],
        })
        row = panel._rows[1]
        self.assertEqual(row.target_type_id(), "gone-from-disk",
                         "the assignment must survive the missing definition")
        self.assertIn("missing", row.target_type.currentText().lower())

    def test_the_assignment_round_trips_instead_of_being_erased(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        panel = page._setup_panel
        panel.apply_state({
            "bore_programs": [
                {"bore_index": 1, "role": "push_reagent", "pump_id": "P2",
                 "target_type_id": "gone-from-disk"},
            ],
        })
        # Any row edit re-harvests the table into the remembered dict.
        panel._rows[1].rate.setValue(0.4)
        state = panel.to_state()
        ids = [p.get("target_type_id") for p in state["bore_programs"]]
        self.assertIn("gone-from-disk", ids,
                      "a re-save must not blank the id")

    def test_a_reload_keeps_a_now_missing_selection(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        panel = page._setup_panel
        panel.apply_state({
            "bore_programs": [
                {"bore_index": 1, "role": "push_reagent", "pump_id": "P2",
                 "target_type_id": "gone-from-disk"},
            ],
        })
        panel.reload_target_types()
        self.assertEqual(panel._rows[1].target_type_id(), "gone-from-disk")

    def test_a_store_failure_keeps_the_types_it_already_had(self):
        """One transient failure must not make the card claim the library is
        empty while three built-ins sit on disk."""
        page = self._make_page()
        panel = page._setup_panel
        self.assertTrue(panel.target_types(), "built-ins should have loaded")
        before = len(panel.target_types())
        with patch("SupportClasses.TargetTypeStore.get_store",
                   side_effect=RuntimeError("disk on fire")):
            panel.reload_target_types()
        self.assertEqual(len(panel.target_types()), before,
                         "the previous list must be kept")
        self.assertTrue(panel._load_errors, "the failure must be recorded")

    def test_a_malformed_file_is_named_with_its_reason(self):
        import json
        import tempfile
        import pathlib
        from SupportClasses import TargetTypeStore as TTS
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="mebp_tt_bad_"))
        (tmp / "user").mkdir()
        (tmp / "builtin").mkdir()
        (tmp / "user" / "broken.json").write_text("{ oops,,, }", encoding="utf-8")
        (tmp / "user" / "fine.json").write_text(
            json.dumps({"id": "ok-one", "label": "Fine"}), encoding="utf-8")
        store = TTS.TargetTypeStore(builtin_dir=tmp / "builtin",
                                    user_dir=tmp / "user")
        names = [n for (n, _r) in store.load_errors]
        self.assertIn("broken.json", names)
        self.assertTrue(store.get("ok-one"), "the good file must still load")

    # ── B8: a not-enabled pump is flagged ──
    def test_an_assembly_declared_but_not_enabled_pump_is_labelled(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack_pumps("P1", "P9"),
                                     pumps=("P1", "P2")))
        row = page._setup_panel._rows[1]
        idx = row.pump.findData("P9")
        self.assertGreaterEqual(idx, 0)
        self.assertIn("not enabled", row.pump.itemText(idx))

    # ── B7: the disabled target-type combo explains itself ──
    def test_an_idle_rows_target_type_combo_says_why_it_is_disabled(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        idle = page._setup_panel._rows[1]
        self.assertFalse(idle.target_type.isEnabled())
        self.assertIn("role", idle.target_type.toolTip().lower())

    # ── U15a: no nonsense notes at zero bores ──
    def test_no_validation_notes_with_no_needle_configured(self):
        page = self._make_page()
        panel = page._setup_panel
        self.assertEqual(panel.validation_notes(), [])
        self.assertEqual(panel._bore_notes.text(), "")
        self.assertFalse(panel._bore_notes.isVisibleTo(panel))


class TestTargetTypeEditorWiring(_PageCase):
    """The card's editor buttons — the authoring path that did not exist."""

    def test_the_card_offers_new_edit_duplicate_delete(self):
        panel = self._make_page()._setup_panel
        for attr in ("_tt_new_btn", "_tt_edit_btn", "_tt_dup_btn", "_tt_del_btn"):
            self.assertTrue(hasattr(panel, attr), attr)

    def test_edit_and_delete_are_disabled_with_nothing_selected(self):
        panel = self._make_page()._setup_panel
        self.assertFalse(panel._tt_edit_btn.isEnabled())
        self.assertFalse(panel._tt_dup_btn.isEnabled())
        self.assertFalse(panel._tt_del_btn.isEnabled())

    def test_selecting_a_builtin_enables_edit_but_NOT_delete(self):
        """A built-in is never deleted — your version shadows it."""
        panel = self._make_page()._setup_panel
        types = panel.target_types()
        self.assertTrue(types, "expected the bundled built-ins")
        builtin = next(t for t in types if t.builtin)
        panel._select_target_type(builtin.id)
        self.assertTrue(panel._tt_edit_btn.isEnabled())
        self.assertTrue(panel._tt_dup_btn.isEnabled())
        self.assertFalse(panel._tt_del_btn.isEnabled())
        self.assertIn("Built-in", panel._tt_del_btn.toolTip())

    def test_the_empty_state_points_at_the_editor_not_at_json(self):
        """It used to say "add JSON files under …/user/"."""
        panel = self._make_page()._setup_panel
        panel._target_types = []
        panel._load_errors = []
        panel._refresh_types_list()
        from PySide6.QtWidgets import QLabel
        texts = " ".join(w.text() for w in panel.findChildren(QLabel))
        self.assertIn("New…", texts)

    def test_a_delete_with_users_names_them_and_can_be_cancelled(self):
        page = self._make_page()
        page.set_hardware_config(_hw(_backpack()))
        panel = page._setup_panel
        _arm_dose(page)
        types = panel.target_types()
        tid = types[0].id
        # Point bore 2 at it, then try to delete it.
        row = panel._rows[1]
        idx = row.target_type.findData(tid)
        if idx >= 0:
            row.target_type.setCurrentIndex(idx)
        panel._select_target_type(tid)
        from PySide6.QtWidgets import QMessageBox as QMB
        with patch("gui.pages.workflows.cell_targeting_setup_panel."
                   "QMessageBox.question",
                   return_value=QMB.StandardButton.Cancel) as q:
            panel._delete_selected_target_type()
        if types[0].builtin:
            self.assertFalse(q.called, "a built-in should not even ask")
        else:
            self.assertTrue(q.called)
            self.assertIn("Bore 2", q.call_args.args[2])


class TestPreRunConfirmation(_PageCase):
    """Every advisory used to arrive AFTER the run started, appended to the status
    label on the same line as "Running N cell removals…"."""

    def _runnable_page(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._scan_page.is_scanning = lambda: False
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            _single_needle(),
            inks={"Reagent": SimpleNamespace(ink_type="ink", color="#89b4fa")},
            locations={"Reagent": ["A1"]}))
        page.set_calibration_data(None, {"A1": (1000.0, 2000.0)}, 40.0)
        page._prep_check.setChecked(False)
        page._clean_check.setChecked(False)
        page._wash_after_pickup_check.setChecked(False)
        page._reagent_combo.setCurrentIndex(
            page._reagent_combo.findData("Reagent"))
        page._picker.add_pick(5000.0, 6000.0)
        page._picker.add_place(7000.0, 8000.0)
        return page

    def test_the_run_is_confirmed_BEFORE_any_executor_is_built(self):
        page = self._runnable_page()
        order = []
        with patch.object(type(page), "_confirm_run",
                          lambda *a, **k: (order.append("confirm"), True)[1]), \
             patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            Exec.side_effect = lambda *a, **k: (order.append("executor"),
                                                MagicMock())[1]
            page._on_start()
            if page._exec_thread is not None:
                page._exec_thread.join(timeout=5)
        self.assertEqual(order[:2], ["confirm", "executor"],
                         "the operator must be asked before anything is built")

    def test_declining_the_confirmation_starts_nothing(self):
        page = self._runnable_page()
        with patch.object(type(page), "_confirm_run", lambda *a, **k: False), \
             patch("gui.pages.workflows.cell_targeting_workflow."
                   "PickPlaceExecutor") as Exec:
            page._on_start()
        self.assertFalse(Exec.called, "nothing may be built after a decline")
        self.assertIsNone(page._exec_thread)
        self.assertIn("nothing has moved", page._status.text().lower())

    def test_the_narrative_states_the_resolved_numbers(self):
        page = self._runnable_page()
        cfg = page._current_config()
        lines = page._run_narrative(cfg, False, False)
        joined = " ".join(lines).lower()
        self.assertIn("plate bottom", joined)
        self.assertIn("incubate", joined)
        self.assertIn("pull", joined)
        self.assertIn("dispense", joined)

    def test_the_narrative_describes_the_TWO_BORE_sequence_when_armed(self):
        page = self._make_page(ctrl=_mock_ctrl())
        page._safe_z = 40.0
        page._controller.print_height_to_zref = lambda mm: 20.0 + mm
        page.set_hardware_config(_hw(
            _backpack(),
            inks={"Trypsin": SimpleNamespace(ink_type="ink", color="#f38ba8")},
            locations={"Trypsin": ["B1"]}))
        page.set_calibration_data(None, {"B1": (3000.0, 4000.0)}, 40.0)
        _arm_dose(page)
        combo = page._setup_panel.trypsin_reagent
        combo.setCurrentIndex(combo.findData("Trypsin"))
        lines = page._run_narrative(page._current_config(), False, False)
        joined = " ".join(lines)
        self.assertIn("Shift", joined, "the XY shift must be disclosed")
        self.assertIn("bore 2", joined)
        self.assertIn("total dose→aspirate", joined,
                      "the additive interval must be stated")

    def test_a_dialog_failure_does_not_block_an_approved_run(self):
        """The gates already approved this run; a broken dialog must not veto it."""
        page = self._runnable_page()
        with patch("gui.dialogs.cell_removal_confirm_dialog."
                   "CellRemovalConfirmDialog", side_effect=RuntimeError("boom")):
            self.assertTrue(page._confirm_run(page._current_config(), 1,
                                              20.1, 20.5, False, False))

    def test_the_dialog_requires_an_acknowledgement(self):
        from gui.dialogs.cell_removal_confirm_dialog import (
            CellRemovalConfirmDialog)
        dlg = CellRemovalConfirmDialog(
            n_targets=40, steps=["do a thing"], clearance_mm=0.10)
        self.addCleanup(dlg.deleteLater)
        self.assertFalse(dlg.acknowledged())
        self.assertFalse(dlg._start.isEnabled(),
                         "Start must be gated on the acknowledgement")
        dlg._ack.setChecked(True)
        self.assertTrue(dlg._start.isEnabled())

    def test_the_dialog_defaults_to_cancel(self):
        from gui.dialogs.cell_removal_confirm_dialog import (
            CellRemovalConfirmDialog)
        dlg = CellRemovalConfirmDialog(n_targets=40, clearance_mm=0.10)
        self.addCleanup(dlg.deleteLater)
        self.assertTrue(dlg._cancel.isDefault())

    def test_the_dialog_states_the_clearance_and_the_cell_count(self):
        from PySide6.QtWidgets import QLabel
        from gui.dialogs.cell_removal_confirm_dialog import (
            CellRemovalConfirmDialog)
        dlg = CellRemovalConfirmDialog(n_targets=40, clearance_mm=0.10,
                                       steps=["x"])
        self.addCleanup(dlg.deleteLater)
        texts = " ".join(w.text() for w in dlg.findChildren(QLabel))
        self.assertIn("0.100 mm", texts)
        self.assertIn("40", texts)

    def test_there_is_no_dont_ask_again(self):
        from PySide6.QtWidgets import QCheckBox
        from gui.dialogs.cell_removal_confirm_dialog import (
            CellRemovalConfirmDialog)
        dlg = CellRemovalConfirmDialog(n_targets=1, clearance_mm=0.10)
        self.addCleanup(dlg.deleteLater)
        labels = [c.text().lower() for c in dlg.findChildren(QCheckBox)]
        self.assertFalse(any("again" in t or "remember" in t for t in labels),
                         f"a hands-free run over live cells earns a click: {labels}")


if __name__ == "__main__":
    unittest.main()
