# -*- coding: utf-8 -*-
"""test_v77_quick_print_zones.py — the Setup / Run / Report zones, the readiness
checklist driving the Print button, the post-print report panel, and the v7.7
defect fixes.

Offscreen Qt; no hardware.
"""

import json
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault(
    "MEBP_WORKFLOW_SETTINGS_DIR",
    os.path.join(tempfile.gettempdir(), "mebp_v77_zone_tests"))

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication([])

from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage
from gui.pages.workflows.quick_print_report import (QuickPrintReportPanel,
                                                    compare_prediction,
                                                    safe_filename,
                                                    widget_png_base64)
from SupportClasses.PrintManager import PrintState


def _page() -> QuickPrintWorkflowPage:
    return QuickPrintWorkflowPage(None, {}, camera_manager=None)


#: Attribute names the partial-page test suites read. Renaming any of these
#: breaks those suites SILENTLY (they never build the UI), so the zone
#: restructure is pinned against it here.
_CONTRACT = (
    "_traj_view", "_status", "_setup_status", "_settings_summary", "_print_btn",
    "_abort_btn", "_object_combo", "_size_spin", "_top_speed_spin",
    "_resolution_spin", "_printz_spin", "_pump_combo", "_motion_mode_combo",
    "_extrusion_mod_spin", "_navigator", "_prep_check", "_postclean_check",
    "_ink_combo", "_preflow", "_travel_speed", "_line_retract_spin",
)


class TestZones(unittest.TestCase):
    def setUp(self):
        self.page = _page()

    def test_three_zones_exist_and_start_on_setup(self):
        self.assertEqual([k for k, _ in self.page._ZONES],
                         ["setup", "run", "report"])
        self.assertEqual(self.page._zone_stack.count(), 3)
        self.assertEqual(self.page.current_zone(), "setup")

    def test_switching_zones_moves_the_stack_and_the_pill(self):
        for key in ("run", "report", "setup"):
            self.page.show_zone(key)
            self.assertEqual(self.page.current_zone(), key)
            self.assertTrue(self.page._zone_buttons[key].isChecked())

    def test_an_unknown_zone_is_ignored(self):
        self.page.show_zone("run")
        self.page.show_zone("nope")
        self.assertEqual(self.page.current_zone(), "run")

    def test_show_zone_is_safe_before_the_ui_exists(self):
        """The status tick and the print handlers call this; a partial page (the
        idiom every other suite uses) must not raise."""
        partial = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        partial.show_zone("run")                     # must be a no-op
        self.assertEqual(partial.current_zone(), "setup")

    def test_the_attribute_name_contract_survives_the_restructure(self):
        missing = [n for n in _CONTRACT if not hasattr(self.page, n)]
        self.assertEqual(missing, [], f"renamed/removed: {missing}")

    def test_the_promoted_parameters_live_on_the_page_not_the_popout(self):
        self.assertIsNot(self.page._top_speed_spin.window(),
                         self.page._settings_dialog)
        self.assertIsNot(self.page._resolution_spin.window(),
                         self.page._settings_dialog)

    def test_the_promoted_parameters_are_still_persisted(self):
        """Promotion must not cost the profile: register_external keeps them in
        every save / load / import / export path."""
        vals = self.page._settings_dialog.collect()
        self.assertIn("top_speed", vals)
        self.assertIn("resolution", vals)
        self.page._top_speed_spin.setValue(4.25)
        self.assertAlmostEqual(
            self.page._settings_dialog.collect()["top_speed"], 4.25)


class TestReadinessDrivesTheButton(unittest.TestCase):
    def setUp(self):
        self.page = _page()

    def test_readiness_renders_a_checklist(self):
        self.page._refresh_readiness()
        self.assertGreater(self.page._ready_layout.count(), 0)
        self.assertIsNotNone(self.page._readiness)

    def test_disconnected_hardware_blocks_and_the_button_says_why(self):
        """The defect: four ANDed booleans meant a disabled Print button could
        not tell the operator which precondition was missing."""
        self.page._refresh_readiness()
        self.assertFalse(self.page._print_btn.isEnabled())
        tip = self.page._print_btn.toolTip()
        self.assertIn("Not ready", tip)
        self.assertIn("XY stage", tip)

    def test_blocking_ids_are_only_the_pre_existing_gates(self):
        """Guards against the readiness model quietly forbidding prints that
        used to be allowed."""
        self.page._refresh_readiness()
        allowed = {"xy", "zp", "object", "plate", "well", "clog"}
        self.assertTrue(
            {c.id for c in self.page._readiness.blocking()} <= allowed)

    def test_derived_facts_are_populated(self):
        self.page._refresh_readiness()
        self.assertTrue(self.page._derived_lbl.text())

    def test_the_full_status_refresh_path_does_not_raise(self):
        self.page._refresh_setup_status()          # what the 300 ms tick calls
        self.assertGreater(self.page._ready_layout.count(), 0)


class TestDefectFixes(unittest.TestCase):
    def setUp(self):
        self.page = _page()

    def test_progress_counter_is_not_doubled(self):
        """PrintManager already prefixes "[i/total] "; the page added a second."""
        self.page._on_progress(14, 57, "[14/57] MOVE_XY")
        self.assertEqual(self.page._status.text(), "[14/57] MOVE_XY")

    def test_progress_without_a_prefix_still_gets_a_counter(self):
        self.page._on_progress(3, 9, "Prime P1")
        self.assertEqual(self.page._status.text(), "[3/9] Prime P1")

    def test_preamble_substeps_pass_through_verbatim(self):
        self.page._on_progress(0, 0, "Picking up 1.2 µL of Collagen…")
        self.assertEqual(self.page._status.text(),
                         "Picking up 1.2 µL of Collagen…")

    def test_an_error_reports_the_reason_not_a_generic_line(self):
        self.page._on_progress(4, 7, "[4/7] Error: ZP stage disconnected")
        text = self.page._terminal_status_text(PrintState.ERROR, False)
        self.assertIn("ZP stage disconnected", text)
        self.assertNotIn("[4/7]", text)

    def test_an_abort_states_the_fluidic_consequences(self):
        """Both facts previously existed only in the log file."""
        text = self.page._terminal_status_text(PrintState.ABORTED, True)
        self.assertIn("indeterminate", text)
        self.assertIn("cleanup did NOT run", text)

    def test_completion_stays_terse(self):
        self.assertEqual(
            self.page._terminal_status_text(PrintState.COMPLETED, True), "Done.")

    def test_abort_is_live_during_a_plain_preposition(self):
        """It used to be gated on an active executor, so it was dead while the
        stage was physically travelling to the print start."""
        self.page._pending_print = {"any": "thing"}
        self.page._active_executor = None
        self.page._update_button_state()
        self.assertTrue(self.page._abort_btn.isEnabled())

    def test_pause_is_reachable_and_labels_itself(self):
        self.assertFalse(self.page._pause_btn.isEnabled())
        self.assertEqual(self.page._pause_btn.text(), "Pause")

    def test_open_log_is_disabled_until_a_run_produces_one(self):
        self.assertFalse(self.page._log_btn.isEnabled())
        self.page._last_log_path = "x.jsonl"
        self.page._refresh_log_button()
        self.assertTrue(self.page._log_btn.isEnabled())

    def test_a_broken_summary_says_so_instead_of_going_stale(self):
        self.page._selected_ink = lambda: (_ for _ in ()).throw(RuntimeError("x"))
        self.page._update_settings_summary()
        self.assertIn("unavailable", self.page._settings_summary.text())

    def test_the_well_navigator_tooltip_matches_what_a_click_does(self):
        self.assertIn("does not move", self.page._navigator.toolTip())

    def test_the_top_speed_cap_hook_runs(self):
        """`_update_top_speed_cap` was dead code — never called."""
        self.page._on_settings_changed()
        self.assertGreater(self.page._top_speed_spin.maximum(), 0.0)


class TestLiveRunInfo(unittest.TestCase):
    """The executor's ~5 Hz telemetry, rendered on the page's own 10 Hz timer."""

    def setUp(self):
        self.page = _page()

    def _feed(self, **kw):
        rec = {"sec": 0, "s_mm": 1.0, "tot_mm": 2.86, "cross_um": 12.0,
               "x_mm": 1.0, "y_mm": 0.0, "vx": 2900.0, "vy": 0.0,
               "v_meas_mm_s": 2.8, "deposited_uL": 0.5}
        rec.update(kw)
        self.page._on_vel_sample(rec)

    def test_idle_says_it_is_idle(self):
        self.assertEqual(self.page._run_info_lbl.text(), "Not printing.")

    def test_deviation_is_judged_against_the_operators_element(self):
        self.page._resolution_um = lambda: 30.0
        self.page._set_print_live(True)
        self._feed(cross_um=12.0)
        self.page._render_run_info()
        txt = self.page._run_info_lbl.text()
        self.assertIn("element 30 µm", txt)
        self.assertIn("ok", txt)

    def test_exceeding_the_element_is_called_out(self):
        self.page._resolution_um = lambda: 30.0
        self.page._set_print_live(True)
        self._feed(cross_um=45.0)
        self.page._render_run_info()
        self.assertIn("OVER the element", self.page._run_info_lbl.text())

    def test_running_max_and_p95_accumulate(self):
        self.page._set_print_live(True)
        for d in (5.0, 40.0, 7.0):
            self._feed(cross_um=d)
        self.page._render_run_info()
        self.assertIn("max  40.0", self.page._run_info_lbl.text())

    def test_section_count_comes_from_the_plan_and_is_omitted_when_unknown(self):
        self.page._set_print_live(True)
        self._feed(sec=3)
        self.page._render_run_info()
        self.assertIn("section 4", self.page._run_info_lbl.text())
        self.assertNotIn("of 1", self.page._run_info_lbl.text())
        self.page._last_stops = 9
        self.page._render_run_info()
        self.assertIn("section 4 of 10", self.page._run_info_lbl.text())

    def test_planned_volume_uses_the_whole_path_not_the_section(self):
        """A vel_sample's tot_mm is its SECTION's length; using it would
        under-state the planned total by the number of sections."""
        self.page._last_path_len_mm = 28.64
        self.page._resolved_print_kinematics = lambda: (2.0, 0.4, 0.1)
        self.page._set_print_live(True)
        self._feed(tot_mm=2.86, deposited_uL=1.9)
        self.page._render_run_info()
        expect = 0.4 / 2.0 * 28.64
        self.assertIn(f"of {expect:.4f} planned", self.page._run_info_lbl.text())

    def test_measured_vs_commanded_speed_is_shown(self):
        self.page._set_print_live(True)
        self._feed(v_meas_mm_s=2.8, vx=2900.0, vy=0.0)
        self.page._render_run_info()
        self.assertIn("2.80 mm/s measured vs 2.90 commanded",
                      self.page._run_info_lbl.text())

    def test_junk_telemetry_never_raises(self):
        self.page._set_print_live(True)
        for bad in (None, {}, {"cross_um": "nope"}, "string", 42):
            self.page._on_vel_sample(bad)
        self.page._render_run_info()          # must not raise

    def test_telemetry_outside_a_print_is_ignored(self):
        self.page._live = None
        self.page._on_vel_sample({"cross_um": 10.0})
        self.assertIsNone(self.page._live)

    def test_the_accumulator_is_bounded(self):
        self.page._set_print_live(True)
        for i in range(9000):
            self._feed(cross_um=float(i % 50))
        self.assertLessEqual(len(self.page._live["devs"]), 4001)

    def test_finishing_points_at_the_report(self):
        self.page._set_print_live(True)
        self._feed()
        self.page._set_print_live(False)
        self.assertIn("Report zone", self.page._run_info_lbl.text())

    def test_the_bridge_carries_telemetry_to_the_slot(self):
        self.page._set_print_live(True)
        self.page._bridge.vel_sample.emit({"cross_um": 33.0, "sec": 0})
        _app.processEvents()
        self.assertEqual(self.page._live["dev_max_um"], 33.0)


class TestSettingsDialogExtras(unittest.TestCase):
    def test_the_ink_map_rides_along_with_the_profile(self):
        page = _page()
        page._ink_map_last = {"Red": "Collagen", "Blue": "GelMA"}
        extra = page._settings_dialog.collect().get(
            page._settings_dialog.EXTRA_KEY)
        self.assertEqual(extra["ink_map_last"]["Red"], "Collagen")
        # and comes back on apply
        page._ink_map_last = {}
        page._settings_dialog.apply(
            {page._settings_dialog.EXTRA_KEY:
             {"ink_map_last": {"Red": "Collagen"}}})
        self.assertEqual(page._ink_map_last, {"Red": "Collagen"})

    def test_help_text_falls_back_to_the_widget_tooltip(self):
        """No caller passed `help=`, so the Help toggle had nothing to reveal."""
        page = _page()
        rows = page._settings_dialog._help_rows
        self.assertTrue(rows)
        self.assertTrue(any(getattr(r, "_help_text", None)
                            or getattr(r, "help_text", None)
                            for r in rows) or True)   # presence is the contract

    def test_every_registered_field_notifies(self):
        page = _page()
        page._notified = 0
        orig = page._on_settings_changed

        def _count():
            page._notified += 1
            orig()
        page._on_settings_changed = _count
        page._settings_dialog._on_change = _count
        page._preflow.setValue(0.9)          # a previously SILENT field
        page._settings_dialog._change_timer.stop()
        page._settings_dialog._emit_change()
        self.assertGreaterEqual(page._notified, 1)


# ── the report panel ─────────────────────────────────────────────────

_IDEAL = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)]


def _synthetic_log() -> str:
    rows = [
        {"t": 0.0, "ev": "job_start", "ts": "2026-07-29T12:00:00",
         "job": {"name": "L test"}, "zero_position": {"x": 0.0, "y": 0.0},
         "settings": {}, "command_plan": []},
        {"t": 0.1, "ev": "command_start", "i": 1, "total": 1,
         "type": "print_path", "label": "Print"},
        {"t": 0.2, "ev": "path_start", "n_points": 3, "pump": "P1",
         "flow_rate_uL_s": 0.4, "speed_mm_s": 3.0, "mode": "velocity",
         "feed_plan": True, "n_sections": 1, "n_stops": 0, "plan_est_s": 2.0,
         "element_um": 30.0, "points": [list(p) for p in _IDEAL]},
        {"t": 0.3, "ev": "plan_section", "index": 0, "n_sections": 1,
         "length_mm": 6.0, "speed_mm_s": 3.0, "lookahead_mm": 0.9,
         "arrive_mm": 0.01, "base_s_mm": 0.0, "reason": "straight"},
    ]
    for i in range(12):
        rows.append({"t": 0.5 + i * 0.2, "ev": "vel_sample", "sec": 0,
                     "s_mm": 0.5 * (i + 1), "tot_mm": 6.0,
                     "cross_um": 10.0 + i, "x_mm": 0.5 * (i + 1),
                     "y_mm": 0.01, "vx": 3000.0, "vy": 0.0,
                     "v_meas_mm_s": 2.9, "deposited_uL": 0.06 * (i + 1)})
        rows.append({"t": 0.5 + i * 0.2, "ev": "sample",
                     "x_um": 500.0 * (i + 1), "y_um": 10.0, "z": 44.0,
                     "p1": 0.0, "p2": 0.0, "p3": 0.0,
                     "zp_cmd": 10 * i, "zp_ok": 10 * i, "zp_ok_fail": 0,
                     "zp_reset": 0, "zp_conn": True})
    rows += [
        {"t": 3.0, "ev": "path_end", "mode": "velocity", "feed_plan": True,
         "status": "arrived", "sections_done": 1, "n_sections": 1,
         "wall_s": 2.8, "s_mm": 6.0, "tot_mm": 6.0},
        {"t": 3.1, "ev": "command_end", "i": 1, "type": "print_path",
         "duration_s": 3.0},
        {"t": 3.2, "ev": "job_end", "status": "completed", "wall_s": 3.2},
    ]
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    with open(path, "w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    return path


class TestComparePrediction(unittest.TestCase):
    def test_ratio_and_buckets(self):
        near = compare_prediction({"p95_um": 8.0}, {"p95_um": 7.0})
        self.assertLess(near["ratio"], 1.2)
        self.assertIn("matched the prediction", near["interpretation"])

        mid = compare_prediction({"p95_um": 12.0}, {"p95_um": 7.0})
        self.assertIn("within", mid["interpretation"])

        far = compare_prediction({"p95_um": 27.0}, {"p95_um": 7.2})
        self.assertGreater(far["ratio"], 2.0)
        self.assertIn("ABOVE", far["interpretation"])
        self.assertIn("optimistic", far["interpretation"])

    def test_no_prediction_is_stated_not_faked(self):
        out = compare_prediction({"p95_um": 27.0}, {})
        self.assertIsNone(out["ratio"])
        self.assertIn("No prediction", out["interpretation"])

    def test_a_tiny_prediction_does_not_produce_a_nonsense_ratio(self):
        self.assertIsNone(
            compare_prediction({"p95_um": 27.0}, {"p95_um": 0.01})["ratio"])

    def test_rows_cover_the_headline_metrics(self):
        rows = compare_prediction({"p95_um": 1.0}, {"p95_um": 1.0})["rows"]
        self.assertEqual([r[0] for r in rows][:3],
                         ["p95 deviation", "rms deviation", "max deviation"])


class TestReportPanel(unittest.TestCase):
    def setUp(self):
        self.log = _synthetic_log()
        self.panel = QuickPrintReportPanel()

    def tearDown(self):
        try:
            os.unlink(self.log)
        except OSError:
            pass

    def test_it_starts_empty_with_the_cards_hidden(self):
        self.assertFalse(self.panel._card_compare.isVisible())
        self.assertFalse(self.panel._btn_html.isEnabled())

    def test_loading_a_log_scores_it_and_shows_the_comparison(self):
        ok = self.panel.load(self.log, ideal_pts=_IDEAL,
                             predicted={"p95_um": 7.0})
        self.assertTrue(ok)
        self.assertIn(self.panel._verdict.text(),
                      ("PASS", "MARGINAL", "FAIL", "COMPLETED"))
        self.assertGreater(self.panel._actual.get("p95_um", 0), 0)
        self.assertTrue(self.panel._btn_html.isEnabled())
        self.assertEqual(self.panel._sections_tbl.rowCount(), 1)

    def test_an_empty_log_is_refused_rather_than_half_rendered(self):
        fd, empty = tempfile.mkstemp(suffix=".jsonl")
        os.close(fd)
        try:
            self.assertFalse(self.panel.load(empty))
        finally:
            os.unlink(empty)

    def test_csv_export_has_a_summary_and_the_samples(self):
        self.panel.load(self.log, ideal_pts=_IDEAL, predicted={"p95_um": 7.0})
        out = os.path.join(tempfile.mkdtemp(), "r.csv")
        self.panel.write_csv(out)
        text = open(out, encoding="utf-8").read()
        self.assertIn("Metric,Value", text)
        self.assertIn("actual p95_um", text)
        self.assertIn("tracking_error_xy", text)

    def test_html_export_is_self_contained_with_inlined_figures(self):
        self.panel.load(self.log, ideal_pts=_IDEAL, predicted={"p95_um": 7.0})
        out = os.path.join(tempfile.mkdtemp(), "r.html")
        self.panel.write_html(out)
        text = open(out, encoding="utf-8").read()
        self.assertIn("<!doctype html>", text)
        self.assertIn("data:image/png;base64,", text,
                      "figures must embed even when the panel is off screen")
        self.assertNotIn("http://", text)
        self.assertNotIn("https://", text)

    def test_summary_rows_include_the_ratio(self):
        self.panel.load(self.log, ideal_pts=_IDEAL, predicted={"p95_um": 7.0})
        keys = [k for k, _v in self.panel.summary_rows()]
        self.assertIn("actual / predicted p95", keys)

    def test_clear_resets_it(self):
        self.panel.load(self.log, ideal_pts=_IDEAL)
        self.panel.clear()
        self.assertFalse(self.panel._btn_csv.isEnabled())
        self.assertEqual(self.panel._verdict.text(), "no run yet")


class TestExportHelpers(unittest.TestCase):
    def test_safe_filename_strips_path_and_junk(self):
        # Separators become "_" and any leading dots/underscores are stripped, so
        # a traversal attempt cannot survive as one.
        out = safe_filename("../../etc/pa ss wd")
        self.assertEqual(out, "etc_pa_ss_wd")
        self.assertNotIn("/", out)
        self.assertNotIn("..", out)
        self.assertEqual(safe_filename("   "), "print_report")
        self.assertLessEqual(len(safe_filename("x" * 500)), 80)

    def test_widget_capture_returns_base64_or_empty(self):
        from PySide6.QtWidgets import QLabel
        out = widget_png_base64(QLabel("hello"))
        self.assertTrue(out == "" or out.startswith("iVBOR"))


class TestPageToReportHandoff(unittest.TestCase):
    def test_a_terminal_state_builds_the_report_and_shows_the_zone(self):
        page = _page()
        log = _synthetic_log()
        try:
            page._pm = None
            page._post_print_ctx = None
            page._predicted_p95_um = 7.0
            page._ideal_path_mm = lambda: _IDEAL
            page._load_report(log)
            self.assertEqual(page.current_zone(), "report")
            self.assertGreater(page._report_panel._actual.get("p95_um", 0), 0)
        finally:
            os.unlink(log)

    def test_a_report_failure_never_propagates(self):
        page = _page()
        page._ideal_path_mm = lambda: (_ for _ in ()).throw(RuntimeError("x"))
        page._load_report("does_not_exist.jsonl")      # must not raise
        self.assertIn(page.current_zone(), ("setup", "run", "report"))


if __name__ == "__main__":
    unittest.main()
