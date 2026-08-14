"""v7.20 — Print Calibrator workflow + prime-time calibration.

The measurement is ``tau = prime_used + (A->C along the line) / velocity``, so the
tests are organised around the four ways it can be wrong:

* the projection (a click off the line, behind A, past B, a degenerate line);
* the velocity (which source was used, and never the settle-contaminated one);
* ``prime_used`` (dropping it makes a Verify run restart from zero instead of
  converging);
* the endpoints (recomputing them instead of reading the run's own log).

Plus the embedding contract on ``QuickPrintWorkflowPage``, where every hook has a
specific failure it prevents — see the class docstrings.
"""

from __future__ import annotations

import ast
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses import PrimeTimeCalibration as ptc  # noqa: E402
import gui.pages.workflows.print_calibrator_workflow as mod_cal  # noqa: E402
from SupportClasses.PrintLogReader import read_log      # noqa: E402


# ════════════════════════════════════════════════════════════════════
#  Pure geometry / arithmetic (no Qt)
# ════════════════════════════════════════════════════════════════════

class TestProjection(unittest.TestCase):
    def test_along_and_perp_on_the_x_axis(self):
        fit = ptc.project_onto_line((0.0, 0.0), (4.0, 0.0), (1.25, 0.3))
        self.assertAlmostEqual(fit.length_mm, 4.0)
        self.assertAlmostEqual(fit.along_mm, 1.25)
        self.assertAlmostEqual(fit.perp_mm, 0.3)
        self.assertAlmostEqual(fit.frac, 1.25 / 4.0)

    def test_rotated_lines_project_exactly(self):
        """The along-track distance must not depend on the line's direction —
        it is what maps to elapsed time."""
        for deg in (0.0, 37.0, 90.0, 143.0, 180.0, -90.0):
            a = math.radians(deg)
            ux, uy = math.cos(a), math.sin(a)
            b = (4.0 * ux, 4.0 * uy)
            c = (1.5 * ux, 1.5 * uy)
            fit = ptc.project_onto_line((0.0, 0.0), b, c)
            self.assertAlmostEqual(fit.along_mm, 1.5, places=9, msg=f"{deg}°")
            self.assertAlmostEqual(fit.perp_mm, 0.0, places=9, msg=f"{deg}°")

    def test_perp_is_measured_perpendicular_not_euclidean(self):
        """|C-A| would be 2.0; the along-track component is 1.0."""
        fit = ptc.project_onto_line((0.0, 0.0), (4.0, 0.0),
                                    (1.0, math.sqrt(3.0)))
        self.assertAlmostEqual(fit.along_mm, 1.0)
        self.assertAlmostEqual(fit.perp_mm, math.sqrt(3.0))

    def test_offset_frame_does_not_matter(self):
        """Only distances matter, so a translated frame gives the same fit."""
        f1 = ptc.project_onto_line((0.0, 0.0), (4.0, 0.0), (1.0, 0.0))
        f2 = ptc.project_onto_line((100.0, -50.0), (104.0, -50.0),
                                   (101.0, -50.0))
        self.assertAlmostEqual(f1.along_mm, f2.along_mm)

    def test_degenerate_line_is_none(self):
        self.assertIsNone(
            ptc.project_onto_line((1.0, 1.0), (1.0, 1.0), (2.0, 2.0)))
        self.assertIsNone(
            ptc.project_onto_line((0.0, 0.0), (0.01, 0.0), (0.0, 0.0)))

    def test_malformed_input_is_none_not_an_exception(self):
        for bad in (None, (), ("x", "y"), (float("nan"), 0.0)):
            self.assertIsNone(
                ptc.project_onto_line((0.0, 0.0), (4.0, 0.0), bad),
                msg=repr(bad))

    def test_off_line_limit_scales_with_length(self):
        self.assertAlmostEqual(ptc.off_line_limit_mm(1.0), 0.5)     # floor
        self.assertAlmostEqual(ptc.off_line_limit_mm(10.0), 1.5)    # fraction


class TestComparePrimes(unittest.TestCase):
    """The two lines measure TWO DIFFERENT PRIMES and the code has ONE knob.

    Line 1 = the COLD START after wash & clean (relaxed column ⇒ the long prime).
    Line 2 = the RESTART after the pump merely paused (still pressurised ⇒ the
    short prime). So ``initial >= restart`` is the expected direction, and the
    required value is the larger because `prime_amounts_uL` primes every segment.
    """

    def _res(self, tau):
        return ptc.PrimeTimeResult(
            prime_time_s=tau, added_s=tau, prime_used_s=0.0,
            prime_volume_uL=None, velocity_mm_s=3.0,
            velocity_source=ptc.VEL_MEASURED, fit=None, refusal=None,
            warning=None, message="m")

    def test_the_expected_direction_costs_an_over_prime_on_restarts(self):
        """Cold start needs more: one knob at 0.50 over-primes every restart."""
        cmp_ = ptc.compare_primes(self._res(0.50), self._res(0.20))
        self.assertAlmostEqual(cmp_.difference_s, 0.30)
        self.assertAlmostEqual(cmp_.required_s, 0.50)
        self.assertFalse(cmp_.restart_binds)
        self.assertIn("MORE", cmp_.message)
        self.assertIn("over-primes", cmp_.message)

    def test_a_restart_needing_more_is_flagged_as_unexpected(self):
        """The wrong way round — that points at the pressure relief / the pause
        length, not at the prime, so it must not read as a normal result."""
        cmp_ = ptc.compare_primes(self._res(0.20), self._res(0.45))
        self.assertAlmostEqual(cmp_.difference_s, -0.25)
        self.assertAlmostEqual(cmp_.required_s, 0.45)
        self.assertTrue(cmp_.restart_binds)
        self.assertIn("Unexpected", cmp_.message)
        self.assertIn("relief", cmp_.message)

    def test_equal_primes_waste_nothing(self):
        cmp_ = ptc.compare_primes(self._res(0.30), self._res(0.30))
        self.assertEqual(cmp_.difference_s, 0.0)
        self.assertAlmostEqual(cmp_.required_s, 0.30)
        self.assertIn("nothing wasted", cmp_.message)

    def test_required_is_always_the_max(self):
        for a, b in ((0.5, 0.2), (0.2, 0.5), (0.3, 0.3)):
            self.assertAlmostEqual(
                ptc.compare_primes(self._res(a), self._res(b)).required_s,
                max(a, b), msg=f"{a}/{b}")

    def test_none_unless_both_lines_measured(self):
        good, bad = self._res(0.3), ptc.evaluate(
            (0.0, 0.0), (4.0, 0.0), (1.0, 1.0), velocity_mm_s=3.0)
        self.assertIsNone(ptc.compare_primes(good, None))
        self.assertIsNone(ptc.compare_primes(None, good))
        self.assertIsNone(ptc.compare_primes(good, bad))
        self.assertIsNone(ptc.compare_primes(bad, good))


class TestEvaluate(unittest.TestCase):
    A = (0.0, 0.0)
    B = (4.0, 0.0)

    def test_baseline_measurement(self):
        r = ptc.evaluate(self.A, self.B, (1.26, 0.02), velocity_mm_s=3.0,
                         velocity_source=ptc.VEL_MEASURED, flow_uL_s=0.35)
        self.assertTrue(r.ok)
        self.assertAlmostEqual(r.added_s, 1.26 / 3.0)
        self.assertAlmostEqual(r.prime_time_s, 1.26 / 3.0)
        self.assertAlmostEqual(r.prime_volume_uL, 0.35 * (1.26 / 3.0))

    def test_prime_used_is_ADDED_so_a_verify_run_converges(self):
        """THE load-bearing arithmetic: a Verify run measures what is still
        MISSING, so the answer is used + residual. Dropping ``prime_used``
        would make every verify run restart from zero and never converge."""
        r = ptc.evaluate(self.A, self.B, (0.36, 0.0), velocity_mm_s=3.0,
                         prime_used_s=0.30)
        self.assertAlmostEqual(r.added_s, 0.12)
        self.assertAlmostEqual(r.prime_time_s, 0.42)

    def test_over_primed_verify_run_REDUCES_the_prime(self):
        """Ink before A on a primed run is legitimate — and must lower the
        prime, not be refused."""
        r = ptc.evaluate(self.A, self.B, (-0.30, 0.0), velocity_mm_s=3.0,
                         prime_used_s=0.50)
        self.assertIsNone(r.refusal)
        self.assertAlmostEqual(r.prime_time_s, 0.40)

    def test_over_primed_result_clamps_at_zero_and_warns(self):
        r = ptc.evaluate(self.A, self.B, (-3.0, 0.0), velocity_mm_s=3.0,
                         prime_used_s=0.10)
        self.assertEqual(r.prime_time_s, 0.0)
        self.assertIsNotNone(r.warning)

    def test_behind_A_with_no_prime_is_refused(self):
        """Physically impossible: ink cannot precede the pump. The click is
        wrong, so returning a (negative) number would be worse than refusing."""
        r = ptc.evaluate(self.A, self.B, (-0.5, 0.0), velocity_mm_s=3.0,
                         prime_used_s=0.0)
        self.assertEqual(r.refusal, ptc.REFUSAL_BEFORE_START)
        self.assertIsNone(r.prime_time_s)

    def test_past_B_is_refused_and_says_what_to_change(self):
        r = ptc.evaluate(self.A, self.B, (4.5, 0.0), velocity_mm_s=3.0)
        self.assertEqual(r.refusal, ptc.REFUSAL_BEYOND_END)
        self.assertIn("longer line", r.message)

    def test_off_the_line_is_refused(self):
        r = ptc.evaluate(self.A, self.B, (1.0, 1.0), velocity_mm_s=3.0)
        self.assertEqual(r.refusal, ptc.REFUSAL_OFF_LINE)

    def test_a_small_miss_is_accepted(self):
        r = ptc.evaluate(self.A, self.B, (1.0, 0.2), velocity_mm_s=3.0)
        self.assertIsNone(r.refusal)

    def test_no_velocity_is_refused_not_defaulted(self):
        for v in (0.0, -1.0, float("nan")):
            r = ptc.evaluate(self.A, self.B, (1.0, 0.0), velocity_mm_s=v)
            self.assertEqual(r.refusal, ptc.REFUSAL_NO_VELOCITY, msg=repr(v))

    def test_degenerate_line_is_refused(self):
        r = ptc.evaluate((0.0, 0.0), (0.0, 0.0), (1.0, 0.0), velocity_mm_s=3.0)
        self.assertEqual(r.refusal, ptc.REFUSAL_DEGENERATE_LINE)

    def test_near_the_end_warns_that_tau_may_be_clipped(self):
        r = ptc.evaluate(self.A, self.B, (3.8, 0.0), velocity_mm_s=3.0)
        self.assertIsNone(r.refusal)
        self.assertIsNotNone(r.warning)
        self.assertIn("longer line", r.warning)

    def test_endpoint_tolerance_admits_a_hair_before_A(self):
        r = ptc.evaluate(self.A, self.B, (-0.01, 0.0), velocity_mm_s=3.0)
        self.assertIsNone(r.refusal)

    def test_message_names_the_velocity_source(self):
        r = ptc.evaluate(self.A, self.B, (1.0, 0.0), velocity_mm_s=3.0,
                         velocity_source=ptc.VEL_MEASURED)
        self.assertIn("measured", r.message)
        r2 = ptc.evaluate(self.A, self.B, (1.0, 0.0), velocity_mm_s=3.0,
                          velocity_source=ptc.VEL_COMMANDED)
        self.assertIn("commanded", r2.message)

    def test_refusals_carry_no_number(self):
        """A refusal must not also hand back a prime time — a caller that
        forgets to check would otherwise apply a wrong one."""
        for c in ((1.0, 1.0), (4.9, 0.0), (-0.5, 0.0)):
            r = ptc.evaluate(self.A, self.B, c, velocity_mm_s=3.0)
            self.assertIsNotNone(r.refusal, msg=repr(c))
            self.assertIsNone(r.prime_time_s)
            self.assertIsNone(r.prime_volume_uL)
            self.assertFalse(r.ok)


# ════════════════════════════════════════════════════════════════════
#  Reading a run back out of its execution log
# ════════════════════════════════════════════════════════════════════

def _write_log(rows) -> Path:
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    return Path(path)


def _job_start(**settings):
    base = {"print_speed_mm_s": 3.5, "prime_amounts_uL": {"P1": 0.0},
            "pump_rates_uL_s": {"P1": 0.35}}
    base.update(settings)
    return {"ev": "job_start", "t": 0.0, "settings": base}


class TestLogReading(unittest.TestCase):
    def setUp(self):
        self._paths = []

    def tearDown(self):
        for p in self._paths:
            try:
                p.unlink()
            except OSError:
                pass

    def _log(self, rows):
        p = _write_log(rows)
        self._paths.append(p)
        return read_log(p)

    def test_velocity_prefers_the_measured_samples(self):
        """vel_sample.s_mm is the ACTUAL position projected onto the path, so it
        is the only source that reflects what the stage really did."""
        log = self._log([
            _job_start(),
            {"ev": "path_start", "t": 1.0, "points": [[0, 0], [4, 0]]},
            {"ev": "vel_sample", "t": 2.0, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 4.0, "s_mm": 6.0},
            {"ev": "path_end", "t": 5.0},
        ])
        v, src = ptc.resolve_velocity(log)
        self.assertAlmostEqual(v, 3.0)
        self.assertEqual(src, ptc.VEL_MEASURED)

    def test_open_loop_samples_are_labelled_as_a_schedule(self):
        """openvel_sample.s_mm is the time-paced COMMANDED target, so it must
        not be presented as a measurement."""
        log = self._log([
            _job_start(),
            {"ev": "openvel_sample", "t": 1.0, "s_mm": 0.0},
            {"ev": "openvel_sample", "t": 3.0, "s_mm": 5.0},
        ])
        v, src = ptc.resolve_velocity(log)
        self.assertAlmostEqual(v, 2.5)
        self.assertEqual(src, ptc.VEL_COMMANDED_SCHEDULE)

    def test_vel_samples_win_over_openvel_samples(self):
        log = self._log([
            _job_start(),
            {"ev": "openvel_sample", "t": 1.0, "s_mm": 0.0},
            {"ev": "openvel_sample", "t": 3.0, "s_mm": 99.0},
            {"ev": "vel_sample", "t": 1.0, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 3.0, "s_mm": 6.0},
        ])
        v, src = ptc.resolve_velocity(log)
        self.assertAlmostEqual(v, 3.0)
        self.assertEqual(src, ptc.VEL_MEASURED)

    def test_falls_back_to_the_commanded_speed(self):
        log = self._log([_job_start(print_speed_mm_s=2.75)])
        v, src = ptc.resolve_velocity(log)
        self.assertAlmostEqual(v, 2.75)
        self.assertEqual(src, ptc.VEL_COMMANDED)

    def test_the_settle_contaminated_window_is_NOT_used(self):
        """path_start is logged BEFORE the move-to-points[0] and its settle, so
        (path_end.t - path_start.t) over-states the traverse and would
        UNDER-state the velocity → a prime time that is too large → a blob at A.

        Here that window would give 4 mm / 10 s = 0.4 mm/s; the samples say
        3.0 mm/s. The resolver must not pick 0.4."""
        log = self._log([
            _job_start(print_speed_mm_s=3.0),
            {"ev": "path_start", "t": 1.0, "points": [[0, 0], [4, 0]]},
            {"ev": "vel_sample", "t": 7.0, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 8.0, "s_mm": 3.0},
            {"ev": "path_end", "t": 11.0},
        ])
        v, _src = ptc.resolve_velocity(log)
        self.assertAlmostEqual(v, 3.0)
        self.assertNotAlmostEqual(v, 0.4)

    def test_single_sample_is_not_a_velocity(self):
        log = self._log([
            _job_start(print_speed_mm_s=1.0),
            {"ev": "vel_sample", "t": 2.0, "s_mm": 1.0},
        ])
        v, src = ptc.resolve_velocity(log)
        self.assertAlmostEqual(v, 1.0)
        self.assertEqual(src, ptc.VEL_COMMANDED)

    def test_zero_duration_or_zero_distance_samples_are_rejected(self):
        for rows in (
            [{"ev": "vel_sample", "t": 2.0, "s_mm": 0.0},
             {"ev": "vel_sample", "t": 2.0, "s_mm": 3.0}],       # dt == 0
            [{"ev": "vel_sample", "t": 1.0, "s_mm": 2.0},
             {"ev": "vel_sample", "t": 3.0, "s_mm": 2.0}],       # ds == 0
        ):
            log = self._log([_job_start(print_speed_mm_s=9.0)] + rows)
            v, src = ptc.resolve_velocity(log)
            self.assertEqual(src, ptc.VEL_COMMANDED)
            self.assertAlmostEqual(v, 9.0)

    def test_per_segment_velocity_windows_the_samples(self):
        """THE two-line requirement. Each PRINT_PATH restarts ``s_mm`` at 0, so a
        whole-log first-to-last span crosses the reset AND swallows the hop:
        (3.0−0.0)/(7.0−1.0) = 0.5 mm/s against a true 3.0 — a 6× error that would
        multiply straight into every prime time."""
        log = self._log([
            _job_start(),
            {"ev": "path_start", "t": 1.0, "points": [[-2, 0], [2, 0]]},
            {"ev": "vel_sample", "t": 1.0, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 2.0, "s_mm": 3.0},
            {"ev": "path_end", "t": 3.0},
            {"ev": "path_start", "t": 6.0, "points": [[-2, -1], [2, -1]]},
            {"ev": "vel_sample", "t": 6.0, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 7.0, "s_mm": 3.0},
            {"ev": "path_end", "t": 9.0},
        ])
        segs = ptc.segments_from_log(log)
        self.assertEqual(len(segs), 2)
        for sg in segs:
            self.assertAlmostEqual(sg.velocity_mm_s, 3.0, msg=str(sg.index))
            self.assertEqual(sg.velocity_source, ptc.VEL_MEASURED)
        self.assertNotAlmostEqual(segs[1].velocity_mm_s, 0.5)
        # …and the whole-log helper is the one that gets it wrong, which is why
        # the page must not use it for a multi-segment run.
        self.assertAlmostEqual(ptc.resolve_velocity(log)[0], 0.5)

    def test_per_segment_endpoints(self):
        log = self._log([
            _job_start(),
            {"ev": "path_start", "t": 1.0, "points": [[0, 0], [4, 0]]},
            {"ev": "path_end", "t": 3.0},
            {"ev": "path_start", "t": 4.0, "points": [[0, -1], [4, -1]]},
            {"ev": "path_end", "t": 6.0},
        ])
        segs = ptc.segments_from_log(log)
        self.assertEqual([sg.a_mm for sg in segs], [(0.0, 0.0), (0.0, -1.0)])
        self.assertEqual([sg.b_mm for sg in segs], [(4.0, 0.0), (4.0, -1.0)])

    def test_a_missing_path_end_closes_at_the_next_start(self):
        """An aborted run may have no path_end; the segment must still appear
        rather than being dropped."""
        log = self._log([
            _job_start(print_speed_mm_s=2.0),
            {"ev": "path_start", "t": 1.0, "points": [[0, 0], [4, 0]]},
            {"ev": "vel_sample", "t": 1.5, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 2.5, "s_mm": 4.0},
            {"ev": "path_start", "t": 5.0, "points": [[0, -1], [4, -1]]},
            {"ev": "vel_sample", "t": 5.5, "s_mm": 0.0},
            {"ev": "vel_sample", "t": 6.5, "s_mm": 2.0},
        ])
        segs = ptc.segments_from_log(log)
        self.assertEqual(len(segs), 2)
        self.assertAlmostEqual(segs[0].velocity_mm_s, 4.0)
        self.assertAlmostEqual(segs[1].velocity_mm_s, 2.0)

    def test_no_path_start_means_no_segments(self):
        self.assertEqual(ptc.segments_from_log(self._log([_job_start()])), [])
        self.assertEqual(ptc.segments_from_log(None), [])

    def test_hop_settings_come_from_the_run_not_the_widgets(self):
        log = self._log([_job_start(intra_well_hop_z_mm=1.5,
                                    line_move_z_speed_mm_s=2.0,
                                    line_move_xy_speed_mm_s=25.0)])
        hop = ptc.hop_settings_from_log(log)
        self.assertAlmostEqual(hop["hop_z_mm"], 1.5)
        self.assertAlmostEqual(hop["hop_z_speed_mm_s"], 2.0)
        self.assertAlmostEqual(hop["hop_xy_speed_mm_s"], 25.0)
        txt = ptc.describe_hop_settings(hop)
        self.assertIn("1.50 mm", txt)
        self.assertIn("25.0 mm/s", txt)

    def test_unstamped_hop_settings_read_as_defaults(self):
        hop = ptc.hop_settings_from_log(self._log([_job_start()]))
        self.assertEqual(hop, {"hop_z_mm": 0.0, "hop_z_speed_mm_s": 0.0,
                               "hop_xy_speed_mm_s": 0.0})
        self.assertIn("default", ptc.describe_hop_settings(hop))

    def test_endpoints_come_from_the_logged_points(self):
        log = self._log([
            _job_start(),
            {"ev": "path_start", "t": 1.0,
             "points": [[10.0, 20.0], [12.0, 20.0], [14.0, 20.0]]},
        ])
        a, b = ptc.line_endpoints_from_log(log)
        self.assertEqual(tuple(a), (10.0, 20.0))
        self.assertEqual(tuple(b), (14.0, 20.0))

    def test_endpoints_none_when_the_log_recorded_no_points(self):
        log = self._log([_job_start(),
                         {"ev": "path_start", "t": 1.0, "n_points": 40}])
        self.assertIsNone(ptc.line_endpoints_from_log(log))

    def test_prime_used_is_recovered_as_volume_over_rate(self):
        log = self._log([_job_start(prime_amounts_uL={"P1": 0.105},
                                    pump_rates_uL_s={"P1": 0.35})])
        self.assertAlmostEqual(ptc.prime_used_s_from_log(log, "P1"), 0.3)

    def test_prime_used_is_zero_for_a_baseline_run(self):
        log = self._log([_job_start(prime_amounts_uL={"P1": 0.0})])
        self.assertEqual(ptc.prime_used_s_from_log(log, "P1"), 0.0)

    def test_flow_comes_from_the_per_pump_rate(self):
        log = self._log([_job_start(pump_rates_uL_s={"P1": 0.42})])
        self.assertAlmostEqual(ptc.flow_uL_s_from_log(log, "P1"), 0.42)

    def test_none_log_and_garbage_never_raise(self):
        self.assertEqual(ptc.resolve_velocity(None), (0.0, ptc.VEL_COMMANDED))
        self.assertIsNone(ptc.line_endpoints_from_log(None))
        self.assertEqual(ptc.prime_used_s_from_log(None, "P1"), 0.0)
        self.assertEqual(ptc.flow_uL_s_from_log(None, "P1"), 0.0)
        log = self._log([{"ev": "job_start", "t": 0.0, "settings": "not a dict"}])
        self.assertEqual(ptc.prime_used_s_from_log(log, "P1"), 0.0)


# ════════════════════════════════════════════════════════════════════
#  GUI — the embedding hooks on QuickPrintWorkflowPage
# ════════════════════════════════════════════════════════════════════

def _qt_app():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _controller():
    c = MagicMock()
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    c.is_xy_connected = True
    c.is_zp_connected = True
    c.is_position_poller_suspended.return_value = False
    return c


class _QpBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _qt_app()

    def _page(self, **kw):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        from SupportClasses.Settings import Settings
        kw.setdefault("settings_id", "print_calibrator_test")
        return QuickPrintWorkflowPage(_controller(), Settings(), **kw)

    LINE = {"object_type": "line", "source": "parametric",
            "params": {"x1": -2.0, "y1": 0.0, "x2": 2.0, "y2": 0.0,
                       "num_points": 41}}


class TestExternalObject(_QpBase):
    def test_pushed_object_is_selected_and_locked(self):
        p = self._page()
        p.set_external_object(self.LINE, label="⟂ line")
        self.assertEqual(p._object_combo.currentData(), p._EXTERNAL_DATA)
        self.assertEqual(p._object_combo.currentText(), "⟂ line")
        self.assertFalse(p._object_combo.isEnabled())

    def test_it_yields_one_print_segment(self):
        """A fixed-Z line is flat in both Z and pump, so _travel_mask returns
        None and the whole line is ONE PRINT_PATH — no seam handling."""
        p = self._page()
        p.set_external_object(self.LINE)
        segs = p._path_segments_for_selection()
        self.assertEqual(len(segs), 1)
        self.assertEqual(len(segs[0]), 41)
        self.assertEqual(segs[0][0], (-2.0, 0.0))
        self.assertEqual(segs[0][-1], (2.0, 0.0))

    def test_it_survives_a_refresh(self):
        """_refresh_objects runs on EVERY showEvent — i.e. every tab-in. If the
        external entry were not re-added AND re-selected, the calibration line
        would silently revert to a dot."""
        p = self._page()
        p.set_external_object(self.LINE)
        p._refresh_objects()
        self.assertEqual(p._object_combo.currentData(), p._EXTERNAL_DATA)
        self.assertEqual(len(p._path_segments_for_selection()), 1)
        self.assertFalse(p._object_combo.isEnabled())

    def test_clearing_restores_the_normal_combo(self):
        p = self._page()
        p.set_external_object(self.LINE)
        p.set_external_object(None)
        self.assertTrue(p._object_combo.isEnabled())
        self.assertEqual(p._object_combo.currentData(), "simple:dot")
        self.assertIsNone(p.external_object())

    def test_the_stored_dict_is_copied_not_aliased(self):
        """PrintObject.from_dict pops "trajectory" off the dict it is handed."""
        p = self._page()
        src = dict(self.LINE)
        p.set_external_object(src)
        src["params"] = {}
        segs = p._path_segments_for_selection()
        self.assertEqual(len(segs), 1)

    def test_no_external_object_leaves_the_combo_untouched(self):
        p = self._page()
        data = [p._object_combo.itemData(i)
                for i in range(p._object_combo.count())]
        self.assertEqual(data[:3],
                         ["simple:dot", "simple:circle", "simple:meander"])
        self.assertTrue(p._object_combo.isEnabled())


class TestPrimeOverride(_QpBase):
    def test_override_zeroes_the_stamped_prime(self):
        """THE point of a baseline run: with any prime at all, the measured
        distance would be the RESIDUAL, not the prime time."""
        p = self._page()
        p.set_prime_override_s(0.0)
        self.assertEqual(p._preflow_s(), 0.0)
        _speed, _flow, prime_uL = p.resolved_print_kinematics()
        self.assertEqual(prime_uL, 0.0)
        self.assertEqual(p._build_settings().prime_amounts_uL[p.pump()], 0.0)

    def test_override_disables_the_spin_so_it_cannot_be_defeated(self):
        p = self._page()
        p.set_prime_override_s(0.0)
        self.assertFalse(p._preflow.isEnabled())
        p._preflow.setValue(1.0)          # even if something sets it
        self.assertEqual(p._preflow_s(), 0.0)

    def test_clearing_the_override_restores_the_spin(self):
        p = self._page()
        before = p._preflow_s()
        p.set_prime_override_s(0.0)
        p.set_prime_override_s(None)
        self.assertTrue(p._preflow.isEnabled())
        self.assertEqual(p._preflow_s(), before)
        self.assertIsNone(p.prime_override_s())

    def test_a_nonzero_override_is_honoured(self):
        p = self._page()
        p.set_prime_override_s(0.75)
        self.assertAlmostEqual(p._preflow_s(), 0.75)

    def test_negative_overrides_clamp_to_zero(self):
        p = self._page()
        p.set_prime_override_s(-1.0)
        self.assertEqual(p._preflow_s(), 0.0)


class TestPrimeIsLinkedToTheGlobal(_QpBase):
    """v7.20: the pre-flow spin was SEEDED from ``pump_prime_time_s`` once, at
    dialog-build time — when ``_hw_config`` is still None. So applying a measured
    prime time reached every workflow that links it and silently did NOT reach
    Quick Print: the calibration would appear to succeed and change nothing."""

    def _wire(self):
        from SupportClasses.CommonPrintSettings import CommonPrintSettings
        from SupportClasses.HardwareConfig import HardwareConfig
        hw = HardwareConfig()
        common = CommonPrintSettings()
        common.set_hardware_config(hw)
        p = self._page()
        p.set_hardware_config(hw)
        p.set_common_print_settings(common)
        return p, hw, common

    def test_the_field_is_registered_as_a_common_link(self):
        p, _hw, _common = self._wire()
        link = p._settings_dialog._common_links.get("preflow")
        self.assertIsNotNone(link, "preflow must be linked, not merely seeded")
        self.assertEqual(link["common_key"], "pump_prime_time_s")

    def test_applying_the_global_reaches_the_page(self):
        p, hw, common = self._wire()
        common.set("pump_prime_time_s", 0.42)
        p.set_common_print_settings(common)        # what app.py's fan-out does
        self.assertAlmostEqual(hw.pump_prime_time_s, 0.42)
        self.assertAlmostEqual(p._preflow_s(), 0.42)
        _speed, flow, prime_uL = p.resolved_print_kinematics()
        self.assertAlmostEqual(prime_uL, flow * 0.42)


class TestEmbeddingFlags(_QpBase):
    def test_default_page_keeps_its_back_button_and_owns_the_camera(self):
        p = self._page()
        self.assertFalse(p._embedded)
        self.assertTrue(p._owns_camera)
        self.assertTrue(any(
            isinstance(w, type(w)) and w.text() == "← Back to Workflows"
            for w in p.findChildren(type(p._print_btn))))

    def test_embedded_drops_the_back_button_and_the_title(self):
        p = self._page(embedded=True)
        texts = [w.text() for w in p.findChildren(type(p._print_btn))]
        self.assertNotIn("← Back to Workflows", texts)
        self.assertIn("⚙ Settings", texts,
                      "the settings popout must stay reachable when embedded")

    def test_not_owning_the_camera_still_BINDS_the_feed(self):
        """The gate must not skip set_camera: the embedded view has to show the
        right slot even though the host starts/stops the device."""
        mgr = MagicMock()
        mgr.is_running.return_value = False
        p = self._page(embedded=True, owns_camera=False)
        p._camera_manager = mgr
        p._camera_view = MagicMock()
        p._camera_view.cam_idx = 5
        p._start_camera()
        p._camera_view.set_camera.assert_called_once_with(0)
        mgr.start.assert_not_called()

    def test_not_owning_the_camera_never_stops_it(self):
        """A QTabWidget hides the inactive tab, so an embedded page gets a
        hideEvent whenever the operator looks elsewhere. Stopping there would
        blank the host's own live view mid-measurement."""
        mgr = MagicMock()
        p = self._page(embedded=True, owns_camera=False)
        p._camera_manager = mgr
        p._camera_view = MagicMock()
        p._camera_started_by_us = True
        p._stop_camera()
        mgr.stop.assert_not_called()

    def test_owning_the_camera_still_starts_and_stops_it(self):
        mgr = MagicMock()
        mgr.is_running.return_value = False
        p = self._page()
        p._camera_manager = mgr
        p._camera_view = MagicMock()
        p._camera_view.cam_idx = 0
        p._start_camera()
        mgr.start.assert_called_once_with(0)
        p._stop_camera()
        mgr.stop.assert_called_once()

    def test_settings_store_is_isolated_per_instance(self):
        """Both instances exist for the whole session and the popout auto-saves
        __last__.json on hide, so a shared id would let the calibrator overwrite
        the real Quick Print page's last-used values."""
        a = self._page(settings_id="quick_print")
        b = self._page(embedded=True, settings_id="print_calibrator")
        self.assertNotEqual(a._settings_dialog._store.workflow_id,
                            b._settings_dialog._store.workflow_id)
        self.assertEqual(b._settings_dialog._store.workflow_id,
                         "print_calibrator")

    def test_embedded_hides_the_plate_queue(self):
        """QueuedPrint.object_data stores the combo userData as a REFERENCE and
        the per-well snapshot re-applies stored widget values on every plate
        click — either would overwrite, or dangle a reference to, the host's
        object."""
        p = self._page(embedded=True)
        for attr in ("_queue_card", "_apply_btn", "_run_all_btn", "_hold_btn"):
            w = getattr(p, attr, None)
            if w is not None:
                self.assertFalse(w.isVisibleTo(p), f"{attr} must be hidden")

    def test_not_embedded_keeps_the_queue(self):
        p = self._page()
        card = getattr(p, "_queue_card", None)
        if card is not None:
            self.assertTrue(card.isVisibleTo(p))


class TestTerminalSignals(_QpBase):
    def _terminal(self, page, state):
        seen = []
        page.print_state_changed.connect(seen.append)
        page._pm = None
        page._post_print_ctx = None
        page._last_log_path = None
        page._on_state(state)
        return seen

    def test_emitted_for_every_terminal_state(self):
        from SupportClasses.PrintManager import PrintState
        for st in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            p = self._page()
            self.assertEqual(self._terminal(p, st), [st], msg=str(st))

    def test_log_path_is_readable_when_the_signal_fires(self):
        """The host reads last_log_path() inside the slot, so it must already be
        set — _on_state captures it before clearing _pm."""
        from SupportClasses.PrintManager import PrintState
        p = self._page()
        p._pm = SimpleNamespace(exec_logger=SimpleNamespace(path=Path("x.jsonl")))
        p._post_print_ctx = None
        seen = []
        p.print_state_changed.connect(
            lambda st: seen.append(p.last_log_path()))
        p._on_state(PrintState.COMPLETED)
        self.assertEqual(seen, [Path("x.jsonl")])

    def test_cleanup_finished_reports_the_outcome(self):
        p = self._page()
        seen = []
        p.cleanup_finished.connect(seen.append)
        p._on_cleanup_done("")
        p._on_cleanup_done("boom")
        self.assertEqual(seen, ["", "boom"])


# ════════════════════════════════════════════════════════════════════
#  GUI — the Print Calibrator page
# ════════════════════════════════════════════════════════════════════

class _CalBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _qt_app()

    def _page(self, controller=None):
        from gui.pages.workflows.print_calibrator_workflow import (
            PrintCalibratorWorkflowPage,
        )
        from SupportClasses.Settings import Settings
        p = PrintCalibratorWorkflowPage(controller or _controller(), Settings())
        # A page built outside MainWindow gets no store isolation from the
        # dispatch, so keep the real Quick Print profile out of harm's way.
        return p


class TestCalibrationLine(_CalBase):
    def test_two_lines_become_two_print_segments(self):
        """SEPARATE segments on purpose: build_well_plate_job then inserts the
        lift → hop → lower → re-prime between them, which IS what the second
        line measures. One merged path would extrude straight across."""
        p = self._page()
        self.assertTrue(p._second_check.isChecked())
        self.assertEqual(p._qp._object_combo.currentData(),
                         p._qp._EXTERNAL_DATA)
        segs = p._qp._path_segments_for_selection()
        self.assertEqual(len(segs), 2)
        self.assertEqual(segs[0][0], (-2.0, 0.0))
        self.assertEqual(segs[1][0], (-2.0, -1.0))
        self.assertIn("hop", p._qp._object_combo.currentText())

    def test_the_second_line_is_parallel_and_offset_by_the_gap(self):
        p = self._page()
        p._angle_spin.setValue(90.0)
        p._gap_spin.setValue(2.0)
        pairs, _length, _note = p._line_pair_mm()
        self.assertEqual(len(pairs), 2)
        (a1, b1), (a2, b2) = pairs
        # Same direction…
        self.assertAlmostEqual(math.atan2(b1[1] - a1[1], b1[0] - a1[0]),
                               math.atan2(b2[1] - a2[1], b2[0] - a2[0]),
                               places=9)
        # …offset perpendicular by the gap.
        self.assertAlmostEqual(math.hypot(a2[0] - a1[0], a2[1] - a1[1]), 2.0,
                               places=9)

    def test_one_line_when_the_second_is_off(self):
        p = self._page()
        p._second_check.setChecked(False)
        self.assertEqual(len(p._qp._path_segments_for_selection()), 1)
        self.assertNotIn("hop", p._qp._object_combo.currentText())

    def test_a_second_line_outside_the_well_is_dropped_with_a_warning(self):
        """Only one line then prints, so the hop cannot be measured — say so
        rather than silently printing a single line."""
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 2600.0    # ~5.2 mm well
        p._length_spin.setValue(4.0)
        p._gap_spin.setValue(10.0)
        pairs, _length, note = p._line_pair_mm()
        self.assertEqual(len(pairs), 1)
        self.assertIn("second line", note.lower())
        self.assertIn("⚠", note)

    def test_geometry_is_centred_on_the_offsets(self):
        p = self._page()
        p._length_spin.setValue(6.0)
        p._angle_spin.setValue(90.0)
        p._off_x_spin.setValue(1.0)
        p._off_y_spin.setValue(-2.0)
        a, b, length, _note = p._line_geometry_mm()
        self.assertAlmostEqual(length, 6.0)
        self.assertAlmostEqual(a[0], 1.0, places=6)
        self.assertAlmostEqual(a[1], -5.0, places=6)
        self.assertAlmostEqual(b[0], 1.0, places=6)
        self.assertAlmostEqual(b[1], 1.0, places=6)

    def test_the_line_is_clamped_to_the_well_AND_says_so(self):
        """Clamping can only SHORTEN the line, and a shorter line may not
        contain the ink start — so a silent clamp would be a silently different
        measurement."""
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 3200.0     # 6.4 mm well
        p._length_spin.setValue(20.0)
        _a, _b, length, note = p._line_geometry_mm()
        self.assertLess(length, 20.0)
        self.assertIn("clamped", note.lower())

    def test_an_offset_outside_the_well_is_refused_not_clamped(self):
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 3200.0
        p._off_x_spin.setValue(10.0)
        a, b, length, note = p._line_geometry_mm()
        self.assertIsNone(a)
        self.assertIsNone(b)
        self.assertEqual(length, 0.0)
        self.assertIn("⚠", note)
        self.assertIn("outside the usable well radius", note)

    def test_an_offset_leaving_only_a_sliver_is_refused(self):
        """Reachable, unlike an "h_max <= 0" case: an offset just INSIDE the
        usable radius leaves room for a fraction of a millimetre. Clamping down
        to that would print a line too short to see an ink start in, and the
        measurement would only reject it later as degenerate."""
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 3200.0     # r_safe = 2.95 mm
        p._angle_spin.setValue(0.0)
        p._off_x_spin.setValue(2.8)                         # 0.15 mm of room
        a, b, length, note = p._line_geometry_mm()
        self.assertIsNone(a)
        self.assertEqual(length, 0.0)
        self.assertIn("only", note)
        self.assertIn(f"{mod_cal.MIN_USABLE_LINE_MM:.2f} mm is needed", note)

    def test_an_offset_with_just_enough_room_is_clamped_not_refused(self):
        """The other side of the same threshold, so the guard cannot pass by
        refusing everything."""
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 3200.0     # r_safe = 2.95 mm
        p._angle_spin.setValue(0.0)
        p._off_x_spin.setValue(2.0)                         # 0.95 mm of room
        p._length_spin.setValue(8.0)
        a, _b, length, note = p._line_geometry_mm()
        self.assertIsNotNone(a)
        self.assertGreaterEqual(length, mod_cal.MIN_USABLE_LINE_MM)
        self.assertIn("clamped", note.lower())

    def test_a_refused_line_pushes_no_object(self):
        """With no object the embedded page's own readiness blocks Print and
        names the reason, instead of printing a line outside the well."""
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 3200.0
        p._off_x_spin.setValue(10.0)
        p._push_line_object()
        self.assertIsNone(p._qp.external_object())

    def test_unknown_well_size_applies_no_clamp(self):
        p = self._page()
        p._qp.well_radius_um = lambda well=None: 0.0
        p._length_spin.setValue(30.0)
        _a, _b, length, note = p._line_geometry_mm()
        self.assertAlmostEqual(length, 30.0)
        self.assertEqual(note, "")

    def test_run_mode_drives_the_prime_override(self):
        p = self._page()
        self.assertEqual(p._qp.prime_override_s(), 0.0)   # baseline default
        p._mode_verify.setChecked(True)
        p._apply_run_mode()
        self.assertIsNone(p._qp.prime_override_s())

    def test_stepping_the_offset_moves_perpendicular_to_the_line(self):
        p = self._page()
        p._angle_spin.setValue(0.0)
        p._step_spin.setValue(1.5)
        p._step_check.setChecked(True)
        p._second_check.setChecked(False)
        x0, y0 = p._off_x_spin.value(), p._off_y_spin.value()
        p._step_offset_for_next_run()
        self.assertAlmostEqual(p._off_x_spin.value(), x0, places=6)
        self.assertAlmostEqual(p._off_y_spin.value(), y0 + 1.5, places=6)

    def test_the_step_clears_the_whole_PAIR_not_one_line(self):
        """A bare 1 mm step at a 1 mm gap would land the next run's line 2
        exactly on this run's line 1, and a bead printed over a bead has no
        readable start — which is the only thing this workflow measures."""
        p = self._page()
        p._angle_spin.setValue(0.0)
        p._second_check.setChecked(True)
        p._gap_spin.setValue(1.0)
        p._step_spin.setValue(1.0)
        y0 = p._off_y_spin.value()
        before = [a[1] for a, _b in p._line_pair_mm()[0]]   # {0.0, -1.0}
        p._step_offset_for_next_run()
        self.assertAlmostEqual(p._off_y_spin.value(), y0 + 2.0, places=6)
        after = [a[1] for a, _b in p._line_pair_mm()[0]]    # {2.0, 1.0}
        for y in after:
            for prev in before:
                self.assertGreater(abs(y - prev), 0.5,
                                   "a new line lands on a previous bead")

    def test_stepping_is_skipped_when_unticked(self):
        p = self._page()
        p._step_check.setChecked(False)
        y0 = p._off_y_spin.value()
        p._step_offset_for_next_run()
        self.assertAlmostEqual(p._off_y_spin.value(), y0)


class TestMeasurementFlow(_CalBase):
    """Drive the whole flow: a finished run's log → a clicked C → a prime time."""

    def setUp(self):
        self._paths = []

    def tearDown(self):
        for p in self._paths:
            try:
                p.unlink()
            except OSError:
                pass

    def _finish_run(self, page, *, segments, prime_uL=0.0, rate=0.35,
                    speeds=(3.0,), hop=None):
        """Write a synthetic exec log with one path_start/end pair per segment
        and per-segment samples, then let the page capture it."""
        extra = dict(hop or {})
        rows = [_job_start(prime_amounts_uL={"P1": prime_uL},
                           pump_rates_uL_s={"P1": rate}, **extra)]
        t = 0.5
        for i, pts in enumerate(segments):
            v = speeds[i] if i < len(speeds) else speeds[-1]
            rows.append({"ev": "path_start", "t": t, "points": pts})
            rows.append({"ev": "vel_sample", "t": t, "s_mm": 0.0})
            rows.append({"ev": "vel_sample", "t": t + 1.0, "s_mm": v})
            rows.append({"ev": "path_end", "t": t + 2.0})
            t += 5.0        # a gap, standing in for the inter-segment hop
        path = _write_log(rows)
        self._paths.append(path)
        page._qp._last_log_path = path
        page._capture_run()

    def test_endpoints_are_taken_from_the_LOG_not_recomputed(self):
        """The log's path_start.points IS what the executor was handed, so a
        desync between what printed and what is measured is impossible. Here the
        page's own arithmetic would give lines through the well centre; the log
        says they were elsewhere."""
        p = self._page()
        self._finish_run(p, segments=[[[50.0, 60.0], [54.0, 60.0]],
                                      [[50.0, 59.0], [54.0, 59.0]]])
        self.assertEqual(len(p._lines), 2)
        self.assertEqual(p._lines[0]["a_um"], (50000.0, 60000.0))
        self.assertEqual(p._lines[1]["a_um"], (50000.0, 59000.0))

    def test_a_clicked_C_yields_the_prime_time(self):
        p = self._page()
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]])
        self.assertAlmostEqual(p._lines[0]["velocity"], 3.0)
        p._lines[0]["c_um"] = (1200.0, 10.0)     # 1.2 mm along, 0.01 mm off
        p._refresh_result()
        res = p._lines[0]["result"]
        self.assertTrue(res.ok)
        self.assertAlmostEqual(res.prime_time_s, 1.2 / 3.0, places=6)
        self.assertTrue(p._apply_btn.isEnabled())

    def test_both_lines_compared_and_apply_takes_the_max(self):
        """The EXPECTED shape: the cold start (line 1) needs more than the
        restart (line 2), so the required prime is line 1's and every restart
        over-primes by the difference."""
        p = self._page()
        self._finish_run(
            p, segments=[[[0.0, 0.0], [4.0, 0.0]], [[0.0, -1.0], [4.0, -1.0]]],
            hop={"intra_well_hop_z_mm": 1.5})
        p._lines[0]["c_um"] = (1200.0, 0.0)      # 1.2 mm → 0.40 s cold start
        p._lines[1]["c_um"] = (600.0, -1000.0)   # 0.6 mm → 0.20 s restart
        p._refresh_result()
        self.assertAlmostEqual(p._lines[0]["result"].prime_time_s, 0.4, places=6)
        self.assertAlmostEqual(p._lines[1]["result"].prime_time_s, 0.2, places=6)
        self.assertIsNotNone(p._compare)
        self.assertAlmostEqual(p._compare.difference_s, 0.2, places=6)
        self.assertAlmostEqual(p._compare.required_s, 0.4, places=6)
        self.assertFalse(p._compare.restart_binds)
        # The Apply button names the value it will write, and it is the max.
        self.assertIn("0.400", p._apply_btn.text())
        self.assertNotIn("RESTART binds", p._apply_btn.text())
        self.assertAlmostEqual(p._applicable_result().prime_time_s, 0.4,
                               places=6)
        self.assertIn("1.50 mm", p._run_provenance)

    def test_a_restart_needing_more_is_surfaced_on_the_button(self):
        p = self._page()
        self._finish_run(
            p, segments=[[[0.0, 0.0], [4.0, 0.0]], [[0.0, -1.0], [4.0, -1.0]]])
        p._lines[0]["c_um"] = (600.0, 0.0)       # 0.20 s cold start
        p._lines[1]["c_um"] = (1200.0, -1000.0)  # 0.40 s restart — wrong way
        p._refresh_result()
        self.assertTrue(p._compare.restart_binds)
        self.assertLess(p._compare.difference_s, 0.0)
        self.assertIn("RESTART binds", p._apply_btn.text())
        self.assertIn("Unexpected", p._result_lbl.text())

    def test_the_result_names_which_line_is_which(self):
        """⚠ Assert on the ROW LABELS, not on the phrases alone: the comparison
        message also says "wash & clean" and "pump pause", so a bare
        ``assertIn("wash & clean", txt)`` passes even with the per-line labels
        stripped — a mutation run caught exactly that."""
        p = self._page()
        self._finish_run(
            p, segments=[[[0.0, 0.0], [4.0, 0.0]], [[0.0, -1.0], [4.0, -1.0]]])
        p._lines[0]["c_um"] = (1200.0, 0.0)
        p._lines[1]["c_um"] = (600.0, -1000.0)
        p._refresh_result()
        txt = p._result_lbl.text()
        self.assertIn("Line 1 — flow start after wash & clean", txt)
        self.assertIn("Line 2 — restart after the pump pause", txt)

    def test_the_provenance_states_whether_prep_ran(self):
        """Line 1 is DEFINED as the cold start after wash & clean, so whether
        prep ran is part of what the number means."""
        p = self._page()
        p._qp._prep_check.setChecked(True)
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]])
        self.assertIn("prep", p._run_provenance.lower())
        self.assertIn("RAN", p._run_provenance)
        p._qp._prep_check.setChecked(False)
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]])
        self.assertIn("did NOT run", p._run_provenance)
        self.assertIn("⚠", p._run_provenance)

    def test_each_line_uses_its_OWN_velocity(self):
        """The samples restart per segment; a shared velocity would be wrong."""
        p = self._page()
        self._finish_run(
            p, segments=[[[0.0, 0.0], [4.0, 0.0]], [[0.0, -1.0], [4.0, -1.0]]],
            speeds=(3.0, 1.5))
        self.assertAlmostEqual(p._lines[0]["velocity"], 3.0)
        self.assertAlmostEqual(p._lines[1]["velocity"], 1.5)
        p._lines[0]["c_um"] = (600.0, 0.0)       # 0.6 / 3.0 = 0.20 s
        p._lines[1]["c_um"] = (600.0, -1000.0)   # 0.6 / 1.5 = 0.40 s
        p._refresh_result()
        self.assertAlmostEqual(p._lines[0]["result"].prime_time_s, 0.2, places=6)
        self.assertAlmostEqual(p._lines[1]["result"].prime_time_s, 0.4, places=6)

    def test_a_verify_run_adds_the_prime_it_used(self):
        p = self._page()
        # prime 0.105 µL at 0.35 µL/s == 0.30 s of pre-flow.
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]], prime_uL=0.105)
        self.assertAlmostEqual(p._run_prime_used_s, 0.3)
        p._lines[0]["c_um"] = (360.0, 0.0)       # 0.36 mm → 0.12 s at 3 mm/s
        p._refresh_result()
        self.assertAlmostEqual(p._lines[0]["result"].prime_time_s, 0.42,
                               places=6)

    def test_an_off_line_click_is_refused_and_apply_stays_disabled(self):
        p = self._page()
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]])
        p._lines[0]["c_um"] = (1000.0, 1500.0)
        p._refresh_result()
        self.assertEqual(p._lines[0]["result"].refusal, ptc.REFUSAL_OFF_LINE)
        self.assertFalse(p._apply_btn.isEnabled())

    def test_a_click_past_B_is_refused(self):
        p = self._page()
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]])
        p._lines[0]["c_um"] = (4500.0, 0.0)
        p._refresh_result()
        self.assertEqual(p._lines[0]["result"].refusal, ptc.REFUSAL_BEYOND_END)
        self.assertFalse(p._apply_btn.isEnabled())

    def test_one_refused_line_does_not_block_the_other(self):
        """A bad click on line 2 must not throw away a good line-1 measurement —
        but the hop cost needs both, so it stays absent."""
        p = self._page()
        self._finish_run(
            p, segments=[[[0.0, 0.0], [4.0, 0.0]], [[0.0, -1.0], [4.0, -1.0]]])
        p._lines[0]["c_um"] = (600.0, 0.0)
        p._lines[1]["c_um"] = (2000.0, 2000.0)      # nowhere near line 2
        p._refresh_result()
        self.assertTrue(p._lines[0]["result"].ok)
        self.assertEqual(p._lines[1]["result"].refusal, ptc.REFUSAL_OFF_LINE)
        self.assertIsNone(p._compare)
        self.assertAlmostEqual(p._applicable_result().prime_time_s, 0.2,
                               places=6)
        self.assertTrue(p._apply_btn.isEnabled())

    def test_clearing_the_marks_drops_the_results(self):
        p = self._page()
        self._finish_run(p, segments=[[[0.0, 0.0], [4.0, 0.0]]])
        p._lines[0]["c_um"] = (1200.0, 0.0)
        p._refresh_result()
        p._clear_marks()
        self.assertIsNone(p._lines[0]["result"])
        self.assertIsNone(p._compare)
        self.assertFalse(p._apply_btn.isEnabled())

    def test_no_run_yet_means_no_measurement(self):
        p = self._page()
        p._refresh_result()
        self.assertEqual(p._lines, [])
        self.assertIsNone(p._applicable_result())
        self.assertFalse(p._apply_btn.isEnabled())
        self.assertFalse(p._mark_btns[0].isEnabled())

    def test_completing_a_run_advances_to_the_measure_step(self):
        from SupportClasses.PrintManager import PrintState
        p = self._page()
        rows = [_job_start(), {"ev": "path_start", "t": 0.5,
                              "points": [[0.0, 0.0], [4.0, 0.0]]}]
        path = _write_log(rows)
        self._paths.append(path)
        p._qp._last_log_path = path
        p._on_print_state(PrintState.COMPLETED)
        self.assertEqual(p.current_step(), "measure")

    def test_marking_targets_the_armed_line(self):
        p = self._page()
        self._finish_run(
            p, segments=[[[0.0, 0.0], [4.0, 0.0]], [[0.0, -1.0], [4.0, -1.0]]])
        p._click_to_abs_um = lambda *_: (700.0, -1000.0)
        p._arm_click("mark:1")
        p._on_feed_clicked(0.0, 0.0)
        self.assertIsNone(p._lines[0]["c_um"])
        self.assertEqual(p._lines[1]["c_um"], (700.0, -1000.0))

    def test_an_aborted_run_does_not_advance_and_says_so(self):
        from SupportClasses.PrintManager import PrintState
        p = self._page()
        p._on_print_state(PrintState.ABORTED)
        self.assertEqual(p.current_step(), "print")
        self.assertIn("aborted", p._instr.text().lower())

    def test_the_log_falls_back_to_the_pages_geometry(self):
        """A pre-v7.7 log records only n_points; the measurement must still be
        possible, and must SAY where the endpoints came from."""
        p = self._page()
        p._qp._well_center_zero_ref_mm = lambda well: (10.0, 10.0)
        p._qp._selected_well = "A1"
        rows = [_job_start(), {"ev": "path_start", "t": 0.5, "n_points": 40}]
        path = _write_log(rows)
        self._paths.append(path)
        p._qp._last_log_path = path
        p._capture_run()
        self.assertEqual(p._lines[0]["a_um"], (8000.0, 10000.0))  # centre − 2 mm
        self.assertIn("no points", p._run_provenance)


class TestClickToStage(_CalBase):
    def test_click_maps_through_pixel_to_stage_offset_with_no_x1000(self):
        """get_xy_position already returns µm; the ×1000 was a documented 1000×
        bug that drove the stage into the envelope corner."""
        ctrl = _controller()
        ctrl.get_xy_position.return_value = (50000.0, 60000.0, 0.0)
        mgr = MagicMock()
        mgr.pixel_to_stage_offset.return_value = (120.0, -40.0)
        p = self._page(controller=ctrl)
        p._camera_manager = mgr
        p._feed = MagicMock()
        p._feed.image_size = (2600, 2048)
        p._feed.cam_idx = 2
        got = p._click_to_abs_um(1300.0, 1024.0)
        self.assertEqual(got, (50120.0, 59960.0))
        # the LIVE frame width must be passed — µm/px resolves against it
        mgr.pixel_to_stage_offset.assert_called_once_with(
            2, 1300.0, 1024.0, 2600, 2048)

    def test_no_frame_yet_returns_none(self):
        p = self._page()
        p._camera_manager = MagicMock()
        p._feed = MagicMock()
        p._feed.image_size = (0, 0)
        self.assertIsNone(p._click_to_abs_um(1.0, 1.0))

    def test_marking_C_requires_arming(self):
        p = self._page()
        p._lines = [{"index": 0, "a_um": (0.0, 0.0), "b_um": (4000.0, 0.0),
                     "velocity": 3.0, "velocity_source": ptc.VEL_MEASURED,
                     "c_um": None, "result": None}]
        p._click_to_abs_um = lambda *_: (1.0, 2.0)
        p._on_feed_clicked(10.0, 10.0)
        self.assertIsNone(p._lines[0]["c_um"])
        p._arm_click("mark:0")
        p._on_feed_clicked(10.0, 10.0)
        self.assertEqual(p._lines[0]["c_um"], (1.0, 2.0))
        self.assertIsNone(p._arm, "the arm must be one-shot")

    def test_a_click_can_NEVER_move_the_line(self):
        """v7.20: the line's location is COMPUTED from the spins and only DRAWN
        on the view. A click that rewrote the offsets would make the printed
        geometry and the on-screen geometry two different things, so no such
        affordance exists and an unknown arm is inert."""
        p = self._page()
        p._qp._well_center_zero_ref_mm = lambda well: (0.0, 0.0)
        p._qp._selected_well = "A1"
        p._length_spin.setValue(4.0)
        p._angle_spin.setValue(0.0)
        p._off_x_spin.setValue(0.0)
        p._off_y_spin.setValue(0.0)
        self.assertFalse(hasattr(p, "_set_start_btn"))
        self.assertFalse(hasattr(p, "_place_line_start"))
        p._arm_click("set_start")           # the retired arm, if anyone re-adds it
        p._click_to_abs_um = lambda *_: (1000.0, 500.0)
        p._on_feed_clicked(0.0, 0.0)
        self.assertEqual(p._off_x_spin.value(), 0.0)
        self.assertEqual(p._off_y_spin.value(), 0.0)


def _line(i, a, b, c=None, v=3.0):
    return {"index": i, "a_um": a, "b_um": b, "velocity": v,
            "velocity_source": ptc.VEL_MEASURED, "c_um": c, "result": None}


class TestMarkers(_CalBase):
    def test_markers_use_the_rotation_safe_projection(self):
        """set_bore_markers routes through stage_offset_to_pixel (the exact
        inverse of the click path); set_reference_markers uses a naive divide
        that mis-places every marker on a rotated or mirrored camera."""
        ctrl = _controller()
        ctrl.get_xy_position.return_value = (1000.0, 2000.0, 0.0)
        p = self._page(controller=ctrl)
        p._feed = MagicMock()
        p._lines = [_line(0, (1500.0, 2000.0), (5500.0, 2000.0),
                          c=(2200.0, 2010.0))]
        p._refresh_markers()
        p._feed.set_reference_markers.assert_not_called()
        marks = p._feed.set_bore_markers.call_args[0][0]
        self.assertEqual([m[0] for m in marks], ["A1", "B1", "C1"])
        self.assertEqual((marks[0][1], marks[0][2]), (500.0, 0.0))
        self.assertEqual((marks[2][1], marks[2][2]), (1200.0, 10.0))

    def test_both_lines_are_labelled_distinctly(self):
        ctrl = _controller()
        ctrl.get_xy_position.return_value = (0.0, 0.0, 0.0)
        p = self._page(controller=ctrl)
        p._feed = MagicMock()
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0)),
                    _line(1, (0.0, -1000.0), (4000.0, -1000.0))]
        p._refresh_markers()
        labels = [m[0] for m in p._feed.set_bore_markers.call_args[0][0]]
        self.assertEqual(labels, ["A1", "B1", "A2", "B2"])

    def test_no_stage_position_pushes_nothing(self):
        ctrl = _controller()
        ctrl.get_xy_position.return_value = None
        p = self._page(controller=ctrl)
        p._feed = MagicMock()
        p._lines = [_line(0, (1.0, 2.0), (3.0, 4.0))]
        p._refresh_markers()
        self.assertEqual(p._feed.set_bore_markers.call_args[0][0], [])


class TestLineOverlay(_CalBase):
    """v7.20: the calibration line is DRAWN on the live view at the absolute XY
    it occupies on the plate — never placed by clicking.

    The overlay is pushed camera-centre-relative against the LIVE stage position
    and re-pushed every tick, so jogging slides it across the frame and it stays
    on the glass. That property is what makes the overlay a CHECK: the drawn
    stroke landing on the real bead means the camera↔stage frame is right.
    """

    def _page_at(self, stage=(1000.0, 2000.0)):
        ctrl = _controller()
        ctrl.get_xy_position.return_value = (stage[0], stage[1], 0.0)
        p = self._page(controller=ctrl)
        p._feed = MagicMock()
        p._qp._well_center_zero_ref_mm = lambda well: (0.0, 0.0)
        p._qp._selected_well = "A1"
        return p, ctrl

    @staticmethod
    def _paths(p):
        return p._feed.set_stage_paths.call_args[0][0]

    def test_the_printed_stroke_is_drawn_between_A_and_B(self):
        p, _c = self._page_at()
        p._lines = [_line(0, (1500.0, 2000.0), (5500.0, 2000.0))]
        p._refresh_markers()
        solid = [q for q in self._paths(p) if not q[2]]
        self.assertEqual(len(solid), 1)
        # camera-centre-relative: A is +500 µm in x, B is +4500
        self.assertEqual(solid[0][0], ((500.0, 0.0), (4500.0, 0.0)))

    def test_the_planned_line_is_drawn_dashed_before_anything_is_printed(self):
        """This is what replaces clicking to place the line: the operator moves
        Offset X/Y and watches the dashed line move over the real glass."""
        p, _c = self._page_at(stage=(0.0, 0.0))
        p._lines = []
        p._length_spin.setValue(4.0)
        p._angle_spin.setValue(0.0)
        p._second_check.setChecked(False)
        p._off_x_spin.setValue(0.0)
        p._off_y_spin.setValue(0.0)
        p._refresh_markers()
        dashed = [q for q in self._paths(p) if q[2]]
        self.assertEqual(len(dashed), 1)
        self.assertEqual(dashed[0][0], ((-2000.0, 0.0), (2000.0, 0.0)))

    def test_the_planned_overlay_follows_the_offset_spins(self):
        p, _c = self._page_at(stage=(0.0, 0.0))
        p._second_check.setChecked(False)
        p._off_x_spin.setValue(0.0)
        p._off_y_spin.setValue(0.0)
        p._refresh_markers()
        before = [q for q in self._paths(p) if q[2]][0][0]
        p._off_y_spin.setValue(1.0)          # 1 mm
        p._refresh_markers()
        after = [q for q in self._paths(p) if q[2]][0][0]
        self.assertEqual([(b[0] - a[0], b[1] - a[1])
                          for a, b in zip(before, after)],
                         [(0.0, 1000.0), (0.0, 1000.0)])

    def test_both_lines_of_a_pair_are_drawn(self):
        p, _c = self._page_at(stage=(0.0, 0.0))
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0)),
                    _line(1, (0.0, -1000.0), (4000.0, -1000.0))]
        p._refresh_markers()
        solid = [q for q in self._paths(p) if not q[2]]
        self.assertEqual(len(solid), 2)
        self.assertEqual(solid[1][0], ((0.0, -1000.0), (4000.0, -1000.0)))

    def test_the_overlay_tracks_the_stage(self):
        """The load-bearing property: jog 1 mm in +X and every drawn point
        shifts 1 mm in −X within the frame, so the overlay stays on the glass
        instead of riding along with the camera."""
        p, ctrl = self._page_at(stage=(1000.0, 2000.0))
        p._lines = [_line(0, (1500.0, 2000.0), (5500.0, 2000.0))]
        p._refresh_markers()
        before = [q[0] for q in self._paths(p)]
        ctrl.get_xy_position.return_value = (2000.0, 2000.0, 0.0)
        p._refresh_markers()
        after = [q[0] for q in self._paths(p)]
        self.assertEqual(len(before), len(after))
        for pts_b, pts_a in zip(before, after):
            for (bx, by), (ax, ay) in zip(pts_b, pts_a):
                self.assertAlmostEqual(ax - bx, -1000.0, places=6)
                self.assertAlmostEqual(ay - by, 0.0, places=6)

    def test_no_stage_position_draws_nothing(self):
        ctrl = _controller()
        ctrl.get_xy_position.return_value = None
        p = self._page(controller=ctrl)
        p._feed = MagicMock()
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0))]
        p._refresh_markers()
        self.assertEqual(self._paths(p), [])

    def test_no_well_centre_still_draws_the_printed_stroke(self):
        """A run's endpoints come out of its own log, so they must not depend on
        a well centre resolving now."""
        p, _c = self._page_at(stage=(0.0, 0.0))
        p._qp._well_center_zero_ref_mm = lambda well: None
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0))]
        p._refresh_markers()
        kinds = [q[2] for q in self._paths(p)]
        self.assertEqual(kinds, [False])

    def test_a_feed_without_the_api_is_not_fatal(self):
        p, _c = self._page_at()
        p._feed = MagicMock(spec=["set_bore_markers", "set_reference_markers"])
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0))]
        p._refresh_markers()                       # must not raise
        p._feed.set_bore_markers.assert_called()

    def test_the_status_tick_repushes_the_paths(self):
        """Without this the overlay freezes where it was when the stage last
        stopped, which is precisely the bug the overlay exists to reveal."""
        p, _c = self._page_at()
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0))]
        p._feed.reset_mock()
        p.on_status_update()
        p._feed.set_stage_paths.assert_called()


class TestGoToPoint(_CalBase):
    def _armed(self, p):
        p._safe_z = 44.0
        p._lines = [_line(0, (12345.0, 6789.0), (16345.0, 6789.0)),
                    _line(1, (12345.0, 5789.0), (16345.0, 5789.0))]
        p._travel = MagicMock()
        p._travel.start.return_value = True

    def test_travel_never_lowers_the_needle(self):
        p = self._page()
        self._armed(p)
        p._goto_point(0, "a")
        _args, kwargs = p._travel.start.call_args
        self.assertIsNone(kwargs["target_z_mm"],
                          "target_z_mm must be None — retract, travel, never "
                          "descend")
        self.assertEqual(kwargs["safe_z_mm"], 44.0)

    def test_the_second_button_targets_the_second_line(self):
        p = self._page()
        self._armed(p)
        p._goto_point(1, "a")
        args, _kwargs = p._travel.start.call_args
        self.assertEqual((args[1], args[2]), (12345.0, 5789.0))

    def test_no_safe_z_refuses_and_explains(self):
        p = self._page()
        self._armed(p)
        p._safe_z = None
        p._goto_point(0, "a")
        p._travel.start.assert_not_called()
        self.assertIn("Safe Z", p._warn_lbl.text())

    def test_a_busy_needle_refuses(self):
        """On a clean completion Quick Print starts the post-print cleanup
        immediately, which drives to waste → wash → oil. Moving then would fight
        it, and the live view is not showing the line anyway."""
        p = self._page()
        self._armed(p)
        p._qp.cleanup_running = lambda: True
        p._goto_point(0, "a")
        p._travel.start.assert_not_called()
        self.assertIn("busy", p._warn_lbl.text().lower())

    def test_another_sequence_driving_the_stage_refuses(self):
        ctrl = _controller()
        ctrl.is_position_poller_suspended.return_value = True
        p = self._page(controller=ctrl)
        self._armed(p)
        p._goto_point(0, "a")
        p._travel.start.assert_not_called()

    def test_goto_is_disabled_while_the_cleanup_runs(self):
        p = self._page()
        self._armed(p)
        p._qp.cleanup_running = lambda: True
        p._refresh_buttons()
        self.assertFalse(p._goto_a_btn.isEnabled())
        p._qp.cleanup_running = lambda: False
        p._refresh_buttons()
        self.assertTrue(p._goto_a_btn.isEnabled())

    def test_the_second_goto_is_disabled_with_only_one_line(self):
        p = self._page()
        self._armed(p)
        p._lines = p._lines[:1]
        p._refresh_buttons()
        self.assertTrue(p._goto_a_btn.isEnabled())
        self.assertFalse(p._goto_b_btn.isEnabled())


class TestApply(_CalBase):
    def _ready(self, page, tau=0.42, tau2=None):
        def _res(t):
            return ptc.PrimeTimeResult(
                prime_time_s=t, added_s=t, prime_used_s=0.0,
                prime_volume_uL=0.1, velocity_mm_s=3.0,
                velocity_source=ptc.VEL_MEASURED, fit=None, refusal=None,
                warning=None, message="msg")
        page._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0))]
        page._lines[0]["result"] = _res(tau)
        if tau2 is not None:
            page._lines.append(_line(1, (0.0, -1000.0), (4000.0, -1000.0)))
            page._lines[1]["result"] = _res(tau2)
            page._compare = ptc.compare_primes(page._lines[0]["result"],
                                          page._lines[1]["result"])

    def test_apply_writes_the_shared_global(self):
        from SupportClasses.CommonPrintSettings import CommonPrintSettings
        from SupportClasses.HardwareConfig import HardwareConfig
        from PySide6.QtWidgets import QMessageBox
        import gui.pages.workflows.print_calibrator_workflow as mod

        hw = HardwareConfig()
        common = CommonPrintSettings()
        common.set_hardware_config(hw)
        p = self._page()
        p.set_hardware_config(hw)
        p.set_common_print_settings(common)
        self._ready(p, 0.42)

        orig = mod.QMessageBox.question
        mod.QMessageBox.question = staticmethod(
            lambda *a, **k: QMessageBox.StandardButton.Yes)
        try:
            p._on_apply()
        finally:
            mod.QMessageBox.question = orig
        self.assertAlmostEqual(hw.pump_prime_time_s, 0.42)
        # …and it reaches the embedded page, which is the whole point.
        self.assertAlmostEqual(p._qp._preflow_s(), 0.42)

    def test_declining_the_dialog_changes_nothing(self):
        from SupportClasses.CommonPrintSettings import CommonPrintSettings
        from SupportClasses.HardwareConfig import HardwareConfig
        from PySide6.QtWidgets import QMessageBox
        import gui.pages.workflows.print_calibrator_workflow as mod

        hw = HardwareConfig()
        common = CommonPrintSettings()
        common.set_hardware_config(hw)
        p = self._page()
        p.set_hardware_config(hw)
        p.set_common_print_settings(common)
        self._ready(p, 0.9)
        before = hw.pump_prime_time_s

        orig = mod.QMessageBox.question
        mod.QMessageBox.question = staticmethod(
            lambda *a, **k: QMessageBox.StandardButton.No)
        try:
            p._on_apply()
        finally:
            mod.QMessageBox.question = orig
        self.assertEqual(hw.pump_prime_time_s, before)

    def test_apply_uses_the_LARGER_of_the_two_lines(self):
        """One pump_prime_time_s primes every segment, so it must cover the
        worst of them — and the dialog says the hop is what binds."""
        from SupportClasses.CommonPrintSettings import CommonPrintSettings
        from SupportClasses.HardwareConfig import HardwareConfig
        from PySide6.QtWidgets import QMessageBox
        import gui.pages.workflows.print_calibrator_workflow as mod

        hw = HardwareConfig()
        common = CommonPrintSettings()
        common.set_hardware_config(hw)
        p = self._page()
        p.set_hardware_config(hw)
        p.set_common_print_settings(common)
        self._ready(p, tau=0.45, tau2=0.20)   # cold start > restart (expected)

        seen = {}
        orig = mod.QMessageBox.question

        def _q(_parent, _title, text, *a, **k):
            seen["text"] = text
            return QMessageBox.StandardButton.Yes
        mod.QMessageBox.question = staticmethod(_q)
        try:
            p._on_apply()
        finally:
            mod.QMessageBox.question = orig
        self.assertAlmostEqual(hw.pump_prime_time_s, 0.45)
        self.assertIn("0.450", seen["text"])
        self.assertIn("over-primes", seen["text"],
                      "the dialog must state what choosing the max costs")

    def test_apply_does_nothing_on_a_refusal(self):
        """⚠ The modal MUST be patched even though the guard should return
        early: without it, a broken guard makes this test HANG offscreen instead
        of failing — which is exactly what a mutation run found."""
        from PySide6.QtWidgets import QMessageBox
        import gui.pages.workflows.print_calibrator_workflow as mod
        p = self._page()
        p._lines = [_line(0, (0.0, 0.0), (4000.0, 0.0), c=(1000.0, 1000.0))]
        p._refresh_result()
        self.assertIsNotNone(p._lines[0]["result"].refusal)

        calls = []
        orig = mod.QMessageBox.question
        mod.QMessageBox.question = staticmethod(
            lambda *a, **k: (calls.append(1),
                             QMessageBox.StandardButton.No)[1])
        try:
            p._on_apply()
        finally:
            mod.QMessageBox.question = orig
        self.assertEqual(calls, [], "a refusal must not even raise the dialog")
        self.assertFalse(p._apply_btn.isEnabled())


class TestPageContract(_CalBase):
    def test_titles_track_the_step(self):
        p = self._page()
        self.assertEqual(p.get_page_title(), "Print Calibrator")
        self.assertIn("Print", p.get_sub_page_title())
        p.show_step("measure")
        self.assertIn("Measure", p.get_sub_page_title())

    def test_step_change_emits_sub_page_changed(self):
        p = self._page()
        seen = []
        p.sub_page_changed.connect(seen.append)
        p.show_step("measure")
        self.assertEqual(seen, [1])

    def test_context_widget_is_the_embedded_pages_panel(self):
        """One jog panel, not two — and the operator needs it to hunt for the
        ink start."""
        p = self._page()
        self.assertIs(p.get_context_widget(), p._qp.get_context_widget())

    def test_contract_methods_forward(self):
        p = self._page()
        p._qp = MagicMock()
        p.set_z_references({"plate_bottom_z": 1.0})
        p._qp.set_z_references.assert_called_once()
        p.set_settings("S")
        p._qp.set_settings.assert_called_once_with("S")
        p.set_well_list(["A1"])
        p._qp.set_well_list.assert_called_once_with(["A1"])

    def test_status_tick_forwards_and_refreshes_markers(self):
        p = self._page()
        p._qp = MagicMock()
        p._feed = MagicMock()
        p.on_status_update()
        p._qp.on_status_update.assert_called_once()
        p._feed.set_bore_markers.assert_called()

    def test_the_page_owns_the_camera_for_the_whole_shell(self):
        mgr = MagicMock()
        mgr.is_running.return_value = False
        p = self._page()
        p._camera_manager = mgr
        p._feed = MagicMock()
        p._feed.cam_idx = 0
        p._start_camera()
        mgr.start.assert_called_once_with(0)
        p._stop_camera()
        mgr.stop.assert_called_once()


class TestRegistration(unittest.TestCase):
    def test_tile_is_registered_and_enabled(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        tiles = {t.workflow_id: t for t in WORKFLOWS}
        self.assertIn("print_calibrator", tiles)
        self.assertTrue(tiles["print_calibrator"].enabled)
        self.assertEqual(tiles["print_calibrator"].title, "Print Calibrator")

    def test_workflows_mode_constructs_the_page(self):
        """AST, not a substring: a comment mentioning the class would pass a
        `in getsource()` check."""
        import gui.pages.workflows_mode as mod
        tree = ast.parse(Path(mod.__file__).read_text(encoding="utf-8"))
        names = {
            n.func.id for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
        }
        self.assertIn("PrintCalibratorWorkflowPage", names)

    def test_the_page_is_reachable_through_the_mode(self):
        app = _qt_app()
        self.assertIsNotNone(app)
        from gui.pages.workflows_mode import WorkflowsModePage
        from gui.pages.workflows.print_calibrator_workflow import (
            PrintCalibratorWorkflowPage,
        )
        from SupportClasses.Settings import Settings
        m = WorkflowsModePage(_controller(), Settings())
        self.assertTrue(m.open_workflow("print_calibrator"))
        self.assertIsInstance(m._stack.currentWidget(),
                              PrintCalibratorWorkflowPage)


class _OrientedMgr:
    """A camera manager with a rotated + mirrored calibration, implementing the
    REAL forward/inverse pair (the same math as CameraManager). Copied in shape
    from test_v713_bore_dot_overlay: an UNROTATED stub cannot tell the
    calibrated inverse apart from a naive divide, so it would prove nothing."""

    def __init__(self, um_per_px=0.5, rot_deg=37.5, mirrored=True):
        self._upp, self._rot, self._mir = float(um_per_px), float(rot_deg), mirrored
        self.cameras = [None] * 4

    def effective_um_per_px(self, idx, w):
        return self._upp

    def pixel_to_stage_offset(self, idx, px, py, w, h):
        dx, dy = px - w / 2.0, py - h / 2.0
        if self._mir:
            dx = -dx
        dx, dy = dx * self._upp, dy * self._upp
        t = math.radians(self._rot)
        c, s = math.cos(t), math.sin(t)
        return (dx * c - dy * s, dx * s + dy * c)

    def stage_offset_to_pixel(self, idx, dx_um, dy_um, w, h):
        t = math.radians(-self._rot)
        c, s = math.cos(t), math.sin(t)
        x, y = dx_um * c - dy_um * s, dx_um * s + dy_um * c
        x, y = x / self._upp, y / self._upp
        if self._mir:
            x = -x
        return (w / 2.0 + x, h / 2.0 + y)


class TestSetStagePaths(unittest.TestCase):
    """The CameraFeedView side of the overlay."""

    W, H = 640, 480

    @classmethod
    def setUpClass(cls):
        cls.app = _qt_app()

    def _view(self, mgr=None, count=False):
        from gui.widgets.camera_feed_view import CameraFeedView
        v = CameraFeedView(camera_manager=mgr, cam_idx=0, show_crosshair=False)
        self.addCleanup(v.deleteLater)
        if count:
            v.renders = 0

            def _c():
                v.renders += 1
            v._rerender_last = _c
        return v

    # ── normalisation / change gate ──

    def test_paths_are_stored_normalised(self):
        v = self._view(count=True)
        v.set_stage_paths([([(1, 2), (3, 4)], "#94e2d5", 0)])
        self.assertEqual(v._stage_paths,
                         [(((1.0, 2.0), (3.0, 4.0)), "#94e2d5", False)])

    def test_an_identical_push_does_not_rerender(self):
        """Pushed from the ~300 ms status tick, so an unchanged push must be
        free or the overlay becomes a per-tick repaint of a 10 Mpx frame."""
        v = self._view(count=True)
        p = [([(0.0, 0.0), (100.0, 0.0)], "#94e2d5", False)]
        v.set_stage_paths(p)
        v.set_stage_paths([([(0.0, 0.0), (100.0, 0.0)], "#94e2d5", False)])
        self.assertEqual(v.renders, 1)

    def test_none_clears(self):
        v = self._view(count=True)
        v.set_stage_paths([([(0.0, 0.0), (1.0, 1.0)], "#fff", False)])
        v.set_stage_paths(None)
        self.assertEqual(v._stage_paths, [])
        self.assertEqual(v.renders, 2)

    def test_garbage_and_degenerate_paths_are_dropped_not_fatal(self):
        v = self._view(count=True)
        v.set_stage_paths([
            ([(0.0, 0.0), (1.0, 1.0)], "#fff", False),
            ([(0.0, 0.0)], "#fff", False),        # a single point is not a line
            ("nonsense",),
            ([(0.0, "x")], "#fff", False),
        ])
        self.assertEqual(len(v._stage_paths), 1)

    # ── projection ──

    def test_paths_use_the_calibrated_inverse_not_a_naive_divide(self):
        """Same contract as the bore dots, and it must hold for the PATH too —
        otherwise the line and the dots at its ends would be drawn in two
        different frames on a rotated camera."""
        mgr = _OrientedMgr()
        v = self._view(mgr)
        off = (320.0, -140.0)
        px, py = v._bore_marker_raw_px(off[0], off[1], self.W, self.H)
        back = mgr.pixel_to_stage_offset(0, px, py, self.W, self.H)
        self.assertAlmostEqual(back[0], off[0], places=6)
        self.assertAlmostEqual(back[1], off[1], places=6)
        naive = (self.W / 2.0 + off[0] / 0.5, self.H / 2.0 + off[1] / 0.5)
        self.assertGreater(math.hypot(px - naive[0], py - naive[1]), 50.0)

    # ── rendering ──

    def _render(self, v, paths):
        """Drive the REAL paint chain and return the rendered QImage.

        Deliberately ``_render_frame`` and not ``_draw_stage_paths`` directly:
        calling the drawing helper by hand would leave "the overlay is never
        wired into the paint chain" invisible to every test here (a mutation run
        proved exactly that), so the frame goes in the front door.
        """
        from PySide6.QtGui import QImage, QColor
        v.resize(self.W, self.H)
        img = QImage(self.W, self.H, QImage.Format.Format_RGB32)
        img.fill(QColor("#000000"))
        v._stage_paths = []
        v.set_stage_paths(paths)
        v._render_frame(img)
        pm = v._last_pixmap
        self.assertIsNotNone(pm, "the view did not render")
        return pm.toImage()

    def _painted(self, v, paths):
        from PySide6.QtGui import QColor
        img = self._render(v, paths)
        black = QColor("#000000")
        n = 0
        for y in range(0, img.height(), 2):
            for x in range(0, img.width(), 2):
                if img.pixelColor(x, y) != black:
                    n += 1
        return n

    @staticmethod
    def _near(img, x: float, y: float, r: int = 4) -> bool:
        """Is anything painted within ``r`` px of (x, y) on the rendered image?"""
        from PySide6.QtGui import QColor
        black = QColor("#000000")
        sx = img.width() / float(TestSetStagePaths.W)
        sy = img.height() / float(TestSetStagePaths.H)
        cx, cy = int(round(x * sx)), int(round(y * sy))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                px, py = cx + dx, cy + dy
                if 0 <= px < img.width() and 0 <= py < img.height():
                    if img.pixelColor(px, py) != black:
                        return True
        return False

    def test_a_segment_crossing_the_view_is_drawn_even_when_BOTH_ends_are_outside(self):
        """The reason there is no bounding-box cull: the useful case is a line
        whose endpoints are off-frame and whose middle crosses the view. A
        per-point off-frame skip (right for a DOT) would blank the overlay
        exactly when the operator most needs it."""
        v = self._view(SimpleNamespace(effective_um_per_px=lambda i, w: 1.0,
                                       cameras=[None] * 4))
        # ±5000 µm at 1 µm/px = ±5000 px, far outside a 640×480 frame.
        n = self._painted(v, [([(-5000.0, 0.0), (5000.0, 0.0)],
                               "#94e2d5", False)])
        self.assertGreater(n, 30, "the crossing segment must be visible")

    def test_a_path_entirely_off_frame_paints_nothing(self):
        """Guard the guard: the test above must not be passing because
        everything paints."""
        v = self._view(SimpleNamespace(effective_um_per_px=lambda i, w: 1.0,
                                       cameras=[None] * 4))
        n = self._painted(v, [([(-5000.0, -5000.0), (-5000.0, 5000.0)],
                               "#94e2d5", False)])
        self.assertEqual(n, 0)

    def test_the_drawn_line_lands_where_the_CALIBRATED_map_puts_it(self):
        """On a rotated + mirrored camera the naive divide and the calibrated
        inverse disagree by hundreds of pixels. Pin the PAINTED line against the
        calibrated projection and assert nothing is painted where the naive one
        would have drawn it — otherwise the path and the A/B dots at its ends
        would be in two different frames."""
        mgr = _OrientedMgr(um_per_px=0.5, rot_deg=37.5, mirrored=True)
        v = self._view(mgr)
        a, b = (-200.0, -60.0), (200.0, 60.0)
        img = self._render(v, [([a, b], "#94e2d5", False)])
        pa = v._bore_marker_raw_px(a[0], a[1], self.W, self.H)
        pb = v._bore_marker_raw_px(b[0], b[1], self.W, self.H)
        mid = ((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0)
        self.assertTrue(self._near(img, *mid),
                        "nothing painted at the calibrated midpoint")
        # These endpoints are symmetric about the centre, so BOTH maps agree at
        # the midpoint — the discriminating sample is a quarter point.
        qa = ((pa[0] * 3 + pb[0]) / 4.0, (pa[1] * 3 + pb[1]) / 4.0)
        naive_q = (self.W / 2.0 + (a[0] * 3 + b[0]) / 4.0 / 0.5,
                   self.H / 2.0 + (a[1] * 3 + b[1]) / 4.0 / 0.5)
        self.assertGreater(math.hypot(qa[0] - naive_q[0], qa[1] - naive_q[1]),
                           30.0, "the two maps must differ here or this "
                                 "proves nothing")
        self.assertTrue(self._near(img, *qa))
        self.assertFalse(self._near(img, *naive_q),
                         "painted where the NAIVE divide would put it")

    def test_dashed_paints_less_than_solid(self):
        """The dashed/solid distinction is the whole planned-vs-printed signal,
        so pin that it reaches the rasteriser rather than only the model."""
        v = self._view(SimpleNamespace(effective_um_per_px=lambda i, w: 1.0,
                                       cameras=[None] * 4))
        seg = [(-200.0, 0.0), (200.0, 0.0)]
        solid = self._painted(v, [(seg, "#94e2d5", False)])
        dashed = self._painted(v, [(seg, "#94e2d5", True)])
        self.assertGreater(solid, 0)
        self.assertLess(dashed, solid)


if __name__ == "__main__":
    unittest.main()
