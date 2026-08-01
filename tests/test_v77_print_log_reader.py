# -*- coding: utf-8 -*-
"""v7.7 — reading a print's JSONL execution log back.

The correctness question that matters most here is which arc length a sample
belongs to. A feed-plan ``vel_sample`` records **section-local** ``s_mm`` while
the ideal path it must be compared against is the WHOLE toolpath, so without the
per-section offset every section after the first is projected onto the wrong part
of the path. That is asserted directly, along with the adapter's error channel
agreeing with the executor's own ``cross_um``.
"""

import glob
import json
import math
import os
import tempfile
import unittest

from SupportClasses import PrintLogReader as PLR
from SupportClasses.PrintExecutionLogger import PrintExecutionLogger


def _write(rows, *, truncate_last=False) -> str:
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    with open(path, "w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
        if truncate_last:
            fh.write('{"t": 9.9, "ev": "sample", "x_um": 1')   # cut mid-write
    return path


#: A two-section L path: 3 mm along +X then 3 mm along +Y.
_IDEAL = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)]


def _feed_plan_log():
    """A minimal but structurally faithful feed-plan run."""
    return [
        {"t": 0.0, "ev": "job_start", "ts": "2026-07-29T12:00:00",
         "job": {"name": "L test", "num_commands": 3},
         "zero_position": {"x": 1000.0, "y": 2000.0},
         "settings": {"print_speed_mm_s": 3.0},
         "command_plan": [{"i": 1, "type": "print_path", "n_points": 3}]},
        {"t": 0.1, "ev": "command_start", "i": 1, "total": 3,
         "type": "print_path", "label": "Print L"},
        {"t": 0.2, "ev": "path_start", "n_points": 3, "pump": "P1",
         "flow_rate_uL_s": 0.4, "speed_mm_s": 3.0, "mode": "velocity",
         "feed_plan": True, "n_sections": 2, "n_stops": 1, "plan_est_s": 3.3,
         "element_um": 30.0, "path_len_mm": 6.0,
         "points": [list(p) for p in _IDEAL]},
        {"t": 0.3, "ev": "plan_section", "index": 0, "n_sections": 2,
         "length_mm": 3.0, "speed_mm_s": 3.0, "lookahead_mm": 0.89,
         "arrive_mm": 0.01, "base_s_mm": 0.0, "reason": "straight"},
        # section 0: section-local s == global s
        {"t": 0.5, "ev": "vel_sample", "sec": 0, "s_mm": 1.0, "tot_mm": 3.0,
         "cross_um": 12.0, "x_mm": 1.0, "y_mm": 0.012, "vx": 3000.0, "vy": 0.0,
         "v_meas_mm_s": 2.9, "deposited_uL": 0.134},
        {"t": 1.0, "ev": "sample", "x_um": 2000.0, "y_um": 2010.0, "z": 44.0,
         "p1": 0.0, "p2": 0.0, "p3": 0.0, "zp_cmd": 10, "zp_ok": 10,
         "zp_ok_fail": 0, "zp_reset": 0, "zp_conn": True},
        {"t": 1.6, "ev": "plan_section", "index": 1, "n_sections": 2,
         "length_mm": 3.0, "speed_mm_s": 3.0, "lookahead_mm": 0.89,
         "arrive_mm": 0.01, "base_s_mm": 3.0, "reason": "straight"},
        # section 1: section-local s = 1.0 → GLOBAL s = 4.0
        {"t": 1.8, "ev": "vel_sample", "sec": 1, "s_mm": 1.0, "tot_mm": 3.0,
         "cross_um": 25.0, "x_mm": 3.02, "y_mm": 1.0, "vx": 0.0, "vy": 3000.0,
         "v_meas_mm_s": 3.0, "deposited_uL": 0.536},
        {"t": 2.0, "ev": "sample", "x_um": 4000.0, "y_um": 3000.0, "z": 44.0,
         "p1": 0.0, "p2": 0.0, "p3": 0.0, "zp_cmd": 40, "zp_ok": 39,
         "zp_ok_fail": 1, "zp_reset": 0, "zp_conn": True},
        {"t": 3.4, "ev": "path_end", "mode": "velocity", "feed_plan": True,
         "status": "arrived", "sections_done": 2, "n_sections": 2,
         "wall_s": 3.2, "s_mm": 6.0, "tot_mm": 6.0},
        {"t": 3.5, "ev": "command_end", "i": 1, "type": "print_path",
         "duration_s": 3.4},
        {"t": 3.6, "ev": "job_end", "status": "completed", "wall_s": 3.6},
    ]


class TestReading(unittest.TestCase):
    def test_events_are_indexed_and_views_resolve(self):
        p = _write(_feed_plan_log())
        try:
            log = PLR.read_log(p)
            self.assertEqual(log.n_bad_lines, 0)
            self.assertFalse(log.truncated)
            self.assertEqual(log.job_name, "L test")
            self.assertEqual(log.status, "completed")
            self.assertAlmostEqual(log.duration_s, 3.6)
            self.assertEqual(log.mode, "feed_plan")
            self.assertEqual(len(log.sections), 2)
            self.assertEqual(len(log.vel_samples), 2)
            self.assertEqual(log.zero_position["x"], 1000.0)
        finally:
            os.unlink(p)

    def test_a_truncated_final_line_is_tolerated(self):
        """The logger flushes per line, so a bad LAST line means the run was cut
        off — not that the file is corrupt."""
        p = _write(_feed_plan_log(), truncate_last=True)
        try:
            log = PLR.read_log(p)
            self.assertTrue(log.truncated)
            self.assertEqual(log.n_bad_lines, 1)
            self.assertEqual(len(log.vel_samples), 2, "good lines still parsed")
        finally:
            os.unlink(p)

    def test_missing_file_returns_an_empty_log_rather_than_raising(self):
        log = PLR.read_log(os.path.join(tempfile.gettempdir(), "nope.jsonl"))
        self.assertEqual(log.events, {})
        self.assertEqual(log.status, "incomplete")

    def test_a_log_with_no_job_end_reports_incomplete(self):
        rows = [r for r in _feed_plan_log() if r["ev"] != "job_end"]
        p = _write(rows)
        try:
            self.assertEqual(PLR.read_log(p).status, "incomplete")
        finally:
            os.unlink(p)

    def test_commands_are_paired_with_durations(self):
        p = _write(_feed_plan_log())
        try:
            cmds = PLR.read_log(p).commands()
            self.assertEqual(len(cmds), 1)
            self.assertTrue(cmds[0]["completed"])
            self.assertAlmostEqual(cmds[0]["duration_s"], 3.4)
        finally:
            os.unlink(p)


class TestSectionArcLength(unittest.TestCase):
    """The bug this class exists to prevent: section-local vs global s."""

    def test_bases_come_from_the_plan_sections(self):
        p = _write(_feed_plan_log())
        try:
            self.assertEqual(PLR.section_bases(PLR.read_log(p)),
                             {0: 0.0, 1: 3.0})
        finally:
            os.unlink(p)

    def test_planned_point_uses_the_GLOBAL_arc_length(self):
        p = _write(_feed_plan_log())
        try:
            log = PLR.read_log(p)
            samples, meta = PLR.to_recorder_samples(log, _IDEAL)
            self.assertTrue(meta["has_planned"])
            # section 0, s=1.0 → 1 mm along the first leg → (1, 0)
            self.assertAlmostEqual(samples[0]["planned_x"], 1.0, places=6)
            self.assertAlmostEqual(samples[0]["planned_y"], 0.0, places=6)
            # section 1, s_local=1.0 → GLOBAL 4.0 → 1 mm up the second leg
            self.assertAlmostEqual(samples[1]["planned_x"], 3.0, places=6)
            self.assertAlmostEqual(samples[1]["planned_y"], 1.0, places=6)
            # the offset is recorded, and the section-local value kept
            self.assertAlmostEqual(samples[1]["s_mm"], 4.0)
            self.assertAlmostEqual(samples[1]["sec_s_mm"], 1.0)
        finally:
            os.unlink(p)

    def test_point_at_arclength_clamps_at_both_ends(self):
        cum = PLR._cumulative(_IDEAL)
        self.assertEqual(PLR.point_at_arclength(_IDEAL, cum, -5.0), (0.0, 0.0))
        self.assertEqual(PLR.point_at_arclength(_IDEAL, cum, 99.0), (3.0, 3.0))


class TestAdapter(unittest.TestCase):
    def test_tracking_error_equals_the_executors_own_cross_track(self):
        """``cross_um`` is what the control loop actually acted on; the report's
        error channel must be the same number, in mm."""
        p = _write(_feed_plan_log())
        try:
            samples, _ = PLR.to_recorder_samples(PLR.read_log(p), _IDEAL)
            self.assertAlmostEqual(samples[0]["tracking_error_xy"] * 1000.0,
                                   12.0, places=6)
            self.assertAlmostEqual(samples[1]["tracking_error_xy"] * 1000.0,
                                   25.0, places=6)
        finally:
            os.unlink(p)

    def test_sample_keys_match_the_recorder_contract(self):
        p = _write(_feed_plan_log())
        try:
            samples, meta = PLR.to_recorder_samples(PLR.read_log(p), _IDEAL)
            for key in ("t", "planned_x", "planned_y", "planned_z", "actual_x",
                        "actual_y", "actual_z", "tracking_error_xy",
                        "tracking_error_z", "segment_id", "is_travel",
                        "is_retract"):
                self.assertIn(key, samples[0])
            self.assertEqual(samples[0]["segment_id"], 0)
            self.assertEqual(samples[1]["segment_id"], 1)
            self.assertIn("summary", meta)
        finally:
            os.unlink(p)

    def test_z_is_merged_from_the_nearest_sampler_row(self):
        p = _write(_feed_plan_log())
        try:
            samples, _ = PLR.to_recorder_samples(PLR.read_log(p), _IDEAL)
            self.assertAlmostEqual(samples[0]["actual_z"], 44.0)
        finally:
            os.unlink(p)

    def test_without_an_ideal_the_planned_channel_is_not_invented(self):
        rows = [r for r in _feed_plan_log()]
        for r in rows:
            r.pop("points", None)
        p = _write(rows)
        try:
            log = PLR.read_log(p)
            self.assertIsNone(log.ideal_points())
            samples, meta = PLR.to_recorder_samples(log)
            self.assertFalse(meta["has_planned"])
            for sm in samples:
                self.assertEqual(sm["tracking_error_xy"], 0.0)
                self.assertEqual(sm["planned_x"], sm["actual_x"])
        finally:
            os.unlink(p)

    def test_open_loop_falls_back_to_the_sampler_and_says_so(self):
        """Open-loop reads no position during the path, so there is nothing to
        compare — the report must not draw a flat zero-error trace as success."""
        rows = [r for r in _feed_plan_log() if r["ev"] != "vel_sample"]
        p = _write(rows)
        try:
            samples, meta = PLR.to_recorder_samples(PLR.read_log(p), _IDEAL)
            self.assertFalse(meta["has_position_feedback"])
            self.assertFalse(meta["has_planned"])
            self.assertEqual(len(samples), 2)          # the two `sample` rows
            # absolute µm → zero-ref mm using the manifest's zero_position
            self.assertAlmostEqual(samples[0]["actual_x"], 1.0, places=6)
            self.assertAlmostEqual(samples[0]["actual_y"], 0.01, places=6)
        finally:
            os.unlink(p)


class TestAnalyses(unittest.TestCase):
    def test_section_stats_pair_the_plan_with_the_measurement(self):
        p = _write(_feed_plan_log())
        try:
            rows = PLR.section_stats(PLR.read_log(p))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["reason"], "straight")
            self.assertAlmostEqual(rows[0]["p95_um"], 12.0)
            self.assertAlmostEqual(rows[1]["p95_um"], 25.0)
        finally:
            os.unlink(p)

    def test_volume_reconciliation_uses_flow_over_speed(self):
        p = _write(_feed_plan_log())
        try:
            v = PLR.volume_reconciliation(PLR.read_log(p))
            self.assertAlmostEqual(v["vol_per_mm_uL"], 0.4 / 3.0)
            self.assertAlmostEqual(v["planned_path_uL"], 0.4 / 3.0 * 6.0)
            self.assertAlmostEqual(v["deposited_path_uL"], 0.536)
        finally:
            os.unlink(p)

    def test_comm_health_reports_the_delta_over_the_run(self):
        p = _write(_feed_plan_log())
        try:
            h = PLR.read_log(p).comm_health()
            self.assertEqual(h["commands"], 30)
            self.assertEqual(h["ok_failures"], 1)
            self.assertEqual(h["board_resets"], 0)
        finally:
            os.unlink(p)

    def test_problems_lists_aborts_errors_and_stalls_in_time_order(self):
        rows = _feed_plan_log() + [
            {"t": 2.5, "ev": "vel_stall", "sec": 1, "s_mm": 1.2, "held_s": 1.1},
            {"t": 2.6, "ev": "abort_requested", "step": 2},
            {"t": 2.7, "ev": "error", "message": "boom", "traceback": "tb"},
        ]
        p = _write(rows)
        try:
            probs = PLR.problems(PLR.read_log(p))
            kinds = [x["kind"] for x in probs]
            self.assertEqual(kinds, ["stall", "abort", "error"])
            self.assertIn("indeterminate",
                          next(x for x in probs if x["kind"] == "abort")["text"])
        finally:
            os.unlink(p)

    def test_list_logs_is_newest_first(self):
        d = tempfile.mkdtemp()
        for name in ("a.jsonl", "b.jsonl", "c.txt"):
            with open(os.path.join(d, name), "w", encoding="utf-8") as fh:
                fh.write("{}\n")
        rows = PLR.list_logs(d)
        self.assertEqual({r["name"] for r in rows}, {"a.jsonl", "b.jsonl"})


class TestLoggerRecordsThePath(unittest.TestCase):
    """v7.7: the logger now stores the path so a SAVED log can be scored."""

    def test_short_paths_are_stored_verbatim(self):
        out = PrintExecutionLogger.path_points(_IDEAL)
        self.assertEqual(out["points"], [[0.0, 0.0], [3.0, 0.0], [3.0, 3.0]])
        self.assertNotIn("decimated_from", out)

    def test_long_paths_are_decimated_but_keep_both_endpoints(self):
        pts = [(i * 0.01, 0.0) for i in range(12_001)]
        out = PrintExecutionLogger.path_points(pts, max_points=1000)
        self.assertLessEqual(len(out["points"]), 1002)
        self.assertEqual(out["decimated_from"], 12_001)
        self.assertEqual(out["points"][0], [0.0, 0.0])
        self.assertEqual(out["points"][-1], [120.0, 0.0])

    def test_degenerate_input_yields_no_field(self):
        for bad in ([], [(0.0, 0.0)], "nonsense", [("a", "b")]):
            self.assertEqual(PrintExecutionLogger.path_points(bad), {})


class TestRealHardwareLog(unittest.TestCase):
    """Integration against the log the v7.6 hardware run actually produced."""

    def setUp(self):
        found = sorted(glob.glob(os.path.join("logs", "prints", "*.jsonl")))
        if not found:
            self.skipTest("no recorded print logs in logs/prints")
        self.path = found[-1]

    def test_it_parses_and_reconstructs_the_section_offsets(self):
        log = PLR.read_log(self.path)
        self.assertEqual(log.n_bad_lines, 0)
        self.assertGreater(log.n_lines, 50)
        bases = PLR.section_bases(log)
        if bases:
            self.assertEqual(min(bases.values()), 0.0)
            # monotone, and each base equals the running sum of section lengths
            acc = 0.0
            for sec in log.sections:
                self.assertAlmostEqual(bases[int(sec["index"])], acc, places=3)
                acc += float(sec["length_mm"])

    def test_the_adapter_produces_usable_samples(self):
        log = PLR.read_log(self.path)
        samples, meta = PLR.to_recorder_samples(log)
        self.assertGreater(len(samples), 10)
        self.assertIn("status", meta)
        for sm in samples:
            self.assertTrue(math.isfinite(sm["actual_x"]))
            self.assertTrue(math.isfinite(sm["tracking_error_xy"]))


if __name__ == "__main__":
    unittest.main()
