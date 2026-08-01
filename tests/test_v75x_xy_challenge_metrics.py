"""test_v75x_xy_challenge_metrics.py — the honest scoring metrics.

The old score, ``XYChallenge.path_error``, is a ONE-SIDED, UNSIGNED, UNTIMED
minimum distance from each VISITED sample to the ideal path. It therefore cannot
see any of the failure modes the operator actually hit on ME3B V1:

  • the deadlock (logs/prints/print_20260727_195339_*.jsonl): the follower
    commanded ~1 mm/s for 37 s while arc-length progress sat frozen at
    0.121 → 0.509 mm of a 59.3 mm path, travelling 12.4 mm to net 1.0 mm.
    Every dithering sample sat ON the line, so the old metric scored it ~0.
  • an exact retrace — "back-and-forth over the same line" — also scores ~0.
  • a run truncated by the wall-time cap is scored on the prefix it reached, so
    "didn't finish" reads as "tracked beautifully".
  • a crawl beats a correct-speed run, because there is no time term. This is
    what made the auto-tune "just bring the velocity down".

These tests pin BOTH halves of each case: that the old metric is blind, and that
the new report/score is not.
"""

import math
import unittest

from SupportClasses import XYChallenge as XC


def _timed(pts, dt=0.02):
    return XC.as_samples([(p[0], p[1]) for p in pts], dt=dt)


def _walk(ideal, arclengths, dt=0.02):
    """Samples placed at given arc lengths along the ideal path."""
    cum = XC._cum(ideal)
    out = []
    for i, s in enumerate(arclengths):
        x, y = XC.point_at(ideal, cum, s)
        out.append(XC.Sample(x, y, i * dt))
    return out


def _report(samples, ideal, speed=5.0, res=30.0, **kw):
    return XC.path_report(samples, ideal, commanded_speed_mm_s=speed,
                          resolution_um=res, **kw)


class TestSampleRecord(unittest.TestCase):
    """Sample must stay tuple-compatible so no existing consumer needs changing."""

    def test_indexes_and_unpacks_like_a_tuple(self):
        s = XC.Sample(1.5, -2.5, 0.25)
        self.assertEqual(s[0], 1.5)
        self.assertEqual(s[1], -2.5)
        self.assertEqual((s.x, s.y, s.t), (1.5, -2.5, 0.25))
        x, y, t = s
        self.assertEqual((x, y, t), (1.5, -2.5, 0.25))

    def test_two_tuple_form_unpacks_as_xy(self):
        s = XC.Sample(3.0, 4.0)
        self.assertEqual(len(s), 2)
        x, y = s                              # the form the overlay painter uses
        self.assertEqual((x, y), (3.0, 4.0))
        self.assertIsNone(s.t)

    def test_as_samples_accepts_every_form(self):
        got = XC.as_samples([(1, 2), (3, 4, 0.5), XC.Sample(5, 6, 1.0)])
        self.assertEqual([(p[0], p[1]) for p in got],
                         [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)])
        self.assertIsNone(got[0].t)
        self.assertEqual(got[1].t, 0.5)

    def test_as_samples_can_synthesise_timestamps(self):
        got = XC.as_samples([(0, 0), (1, 0), (2, 0)], dt=0.1)
        self.assertAlmostEqual(got[2].t, 0.2)

    def test_legacy_xy_input_still_works_in_path_error(self):
        ideal = XC.make_shape("Square", 6.0, 0.5)
        plain = [(p[0], p[1]) for p in ideal]
        self.assertLess(XC.path_error(plain, ideal)["rms_um"], 1.0)


class TestFastScoringIsExact(unittest.TestCase):
    """The spatial index replaces an O(N·M) scan — it must be EXACT, not close."""

    def test_matches_bruteforce_on_every_shape(self):
        for name in XC.CHALLENGE_SHAPES:
            ideal = XC.make_shape(name, 10.0, 0.5)
            off = [(p[0] + 0.037, p[1] - 0.021) for p in ideal]
            fast = XC._path_error_fast(off, ideal)
            brute = XC._path_error_bruteforce(off, ideal)
            for k in ("rms_um", "max_um", "mean_um", "n"):
                self.assertAlmostEqual(fast[k], brute[k], delta=1e-9,
                                       msg=f"{name}/{k}")

    def test_matches_bruteforce_for_far_off_path_samples(self):
        """A sample well outside the shape must still get its true distance —
        the ring search has to keep expanding, not give up."""
        ideal = XC.make_shape("Circle", 8.0, 0.4)
        far = [(50.0, 50.0), (-30.0, 4.0), (0.0, 0.0)]
        fast = XC._path_error_fast(far, ideal)
        brute = XC._path_error_bruteforce(far, ideal)
        self.assertAlmostEqual(fast["rms_um"], brute["rms_um"], delta=1e-6)
        self.assertAlmostEqual(fast["max_um"], brute["max_um"], delta=1e-6)

    def test_degenerate_inputs(self):
        self.assertEqual(XC.path_error([], [(0, 0), (1, 1)])["n"], 0)
        self.assertEqual(XC.path_error([(0, 0)], [(0, 0)])["n"], 0)


class TestPerfectRunPasses(unittest.TestCase):
    """Guards against a metric so strict it fails everything — including on the
    CLOSED shapes, where a naive global projection wraps the closing point back to
    s=0 and reports a perfect run as 98.8 % complete."""

    def test_every_shape_scores_zero_and_passes(self):
        for name in XC.CHALLENGE_SHAPES:
            ideal = XC.make_shape(name, 10.0, 0.5)
            rep = _report(_timed(ideal), ideal)
            sc = XC.composite_score(rep)
            self.assertAlmostEqual(rep["completion_frac"], 1.0, places=3,
                                   msg=name)
            self.assertAlmostEqual(rep["dither_ratio"], 1.0, delta=0.05,
                                   msg=name)
            self.assertTrue(sc["pass"], msg=f"{name}: {sc}")
            self.assertLess(sc["score"], 0.1, msg=name)


class TestDitherIsInvisibleToTheOldMetric(unittest.TestCase):
    """THE headline regression: the logged deadlock, rebuilt synthetically."""

    def _deadlock(self):
        ideal = XC.make_shape("Square", 15.0, 0.5)      # ~60 mm perimeter
        # Oscillate between s=0.12 and s=0.51 mm for ~37 s, as logged.
        arcs = []
        for k in range(400):
            frac = (k % 20) / 20.0
            arcs.append(0.121 + frac * 0.388)
        return ideal, _walk(ideal, arcs, dt=0.0925)

    def test_old_metric_is_blind(self):
        ideal, samples = self._deadlock()
        old = XC.path_error(samples, ideal)
        # Every dithering sample sits ON the line → the old score is ~zero.
        self.assertLess(old["rms_um"], 5.0)
        self.assertLess(old["max_um"], 5.0)

    def test_new_score_fails_it_on_completion_and_dither(self):
        ideal, samples = self._deadlock()
        rep = _report(samples, ideal)
        sc = XC.composite_score(rep)
        self.assertFalse(sc["pass"])
        self.assertIn("incomplete", sc["fail_reasons"])
        self.assertIn("dither", sc["fail_reasons"])
        self.assertLess(rep["completion_frac"], 0.05)
        self.assertGreater(rep["dither_ratio"], 5.0)
        self.assertGreater(sc["score"], 1000.0)

    def test_reversals_are_counted(self):
        ideal, samples = self._deadlock()
        self.assertGreater(XC.reversal_count(samples, ideal)["n"], 10)

    def test_travel_vastly_exceeds_progress(self):
        ideal, samples = self._deadlock()
        L = XC.path_length_ratio(samples, ideal)
        self.assertGreater(L["travel_mm"], 5.0 * L["progress_mm"])


class TestExactRetraceScoresZeroOnTheOldMetric(unittest.TestCase):
    """"Back-and-forth over the same line" — the operator's own words."""

    def _retrace(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        fwd = [k * 0.05 for k in range(20)]
        arcs = fwd + fwd[::-1] + fwd
        return ideal, _walk(ideal, arcs, dt=0.05)

    def test_old_metric_reports_no_error_at_all(self):
        ideal, samples = self._retrace()
        self.assertLess(XC.path_error(samples, ideal)["rms_um"], 1.0)

    def test_new_score_fails_it(self):
        ideal, samples = self._retrace()
        sc = XC.composite_score(_report(samples, ideal))
        self.assertFalse(sc["pass"])
        self.assertIn("incomplete", sc["fail_reasons"])


class TestTruncatedRunFails(unittest.TestCase):
    """Every one of the operator's best-scoring tuning runs logged "wall-time cap
    hit" — truncated, then scored anyway, and it WON."""

    def test_half_a_path_perfectly_tracked_still_fails(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        half = _timed(ideal[:len(ideal) // 2])
        old = XC.path_error(half, ideal)
        self.assertLess(old["rms_um"], 1.0)            # old: looks perfect
        rep = _report(half, ideal)
        sc = XC.composite_score(rep)
        self.assertFalse(sc["pass"])                  # new: incomplete
        self.assertIn("incomplete", sc["fail_reasons"])
        self.assertAlmostEqual(rep["completion_frac"], 0.5, delta=0.06)

    def test_a_driver_status_other_than_ok_is_a_hard_fail(self):
        ideal = XC.make_shape("Square", 6.0, 0.5)
        rep = _report(_timed(ideal), ideal, status="stalled")
        sc = XC.composite_score(rep)
        self.assertFalse(sc["pass"])
        self.assertIn("status:stalled", sc["fail_reasons"])


class TestCoverageCatchesSkippedFeatures(unittest.TestCase):
    def test_ideal_to_actual_direction_is_measured(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        # Trace only three of the four sides.
        part = _timed(ideal[:int(len(ideal) * 0.75)])
        ts = XC.two_sided_deviation(part, ideal)
        self.assertLess(ts["a2i_p95_um"], 50.0)       # what it DID was accurate
        self.assertGreater(ts["i2a_max_um"], 1000.0)  # but a side is missing
        self.assertGreater(ts["hausdorff_um"], 1000.0)

    def test_coverage_fraction(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        full = _timed(ideal)
        self.assertGreater(XC.coverage(full, ideal, 30.0), 0.99)
        half = _timed(ideal[:len(ideal) // 2])
        self.assertLess(XC.coverage(half, ideal, 30.0), 0.6)


class TestSignedErrorDistinguishesCutFromOvershoot(unittest.TestCase):
    def test_inside_and_outside_are_separated(self):
        circ = XC.make_shape("Circle", 10.0, 0.3)
        inner = XC.as_samples([(p[0] * 0.97, p[1] * 0.97) for p in circ])
        outer = XC.as_samples([(p[0] * 1.03, p[1] * 1.03) for p in circ])
        a = XC.signed_cross_track(inner, circ)
        b = XC.signed_cross_track(outer, circ)
        # One side is loaded for each; the old unsigned metric cannot tell them
        # apart at all (both would report the same magnitude).
        self.assertNotEqual((a["inside_um"] > 0.0), (a["outside_um"] > 0.0))
        self.assertNotEqual((b["inside_um"] > 0.0), (b["outside_um"] > 0.0))
        self.assertAlmostEqual(XC.path_error(inner, circ)["rms_um"],
                               XC.path_error(outer, circ)["rms_um"], delta=25.0)


class TestCornerOvershoot(unittest.TestCase):
    def test_a_bulge_at_one_corner_is_attributed_to_that_corner(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        cum = XC._cum(ideal)
        # Push samples outward only near the second corner.
        target = cum[len(cum) // 4]
        samples = []
        for i, p in enumerate(ideal):
            s = cum[i]
            bulge = 0.2 if abs(s - target) < 0.6 else 0.0
            samples.append(XC.Sample(p[0] + bulge, p[1] + bulge, i * 0.02))
        c = XC.corner_overshoot(samples, ideal, corner_angle_deg=30.0)
        self.assertGreater(c["n"], 0)
        self.assertGreater(c["max_um"], 150.0)

    def test_no_corners_on_a_circle(self):
        circ = XC.make_shape("Circle", 10.0, 0.3)
        c = XC.corner_overshoot(_timed(circ), circ, corner_angle_deg=30.0)
        self.assertEqual(c["n"], 0)


class TestTimeTermStopsWinningByCrawling(unittest.TestCase):
    """The direct fix for "the tuning sweep is just bringing the velocity down"."""

    def test_a_slower_run_scores_worse_for_identical_geometry(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        fast = _report(_timed(ideal, dt=0.02), ideal, speed=5.0)
        slow = _report(_timed(ideal, dt=0.20), ideal, speed=5.0)
        s_fast = XC.composite_score(fast)["score"]
        s_slow = XC.composite_score(slow)["score"]
        self.assertGreater(s_slow, s_fast)

    def test_throughput_reports_efficiency(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        # 40 mm at 5 mm/s should take 8 s; take 16 → efficiency ~0.5.
        tp = XC.throughput(_timed(ideal), ideal, 5.0, wall_s=16.0)
        self.assertAlmostEqual(tp["efficiency"], 0.5, delta=0.08)
        self.assertAlmostEqual(tp["time_ratio"], 2.0, delta=0.1)

    def test_untimed_trace_returns_none_instead_of_guessing(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        plain = XC.as_samples([(p[0], p[1]) for p in ideal])
        self.assertIsNone(XC.along_track_lag(plain, ideal, 5.0)["mean_mm"])
        self.assertIsNone(XC.throughput(plain, ideal, 5.0)["achieved_mm_s"])


class TestLagAndDwell(unittest.TestCase):
    def test_lag_grows_when_the_stage_runs_at_half_speed(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        # Commanded 5 mm/s, actually achieving 2.5 → a growing along-track lag.
        cum = XC._cum(ideal)
        samples = []
        for i in range(0, len(ideal)):
            s = min(cum[-1], 2.5 * (i * 0.04))
            x, y = XC.point_at(ideal, cum, s)
            samples.append(XC.Sample(x, y, i * 0.04))
        lag = XC.along_track_lag(samples, ideal, 5.0)
        self.assertIsNotNone(lag["mean_mm"])
        self.assertGreater(lag["mean_mm"], 0.5)

    def test_dwell_event_detected(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        cum = XC._cum(ideal)
        samples = []
        t = 0.0
        for i in range(10):
            x, y = XC.point_at(ideal, cum, i * 0.5)
            samples.append(XC.Sample(x, y, t)); t += 0.05
        hold = XC.point_at(ideal, cum, 5.0)
        for _ in range(12):                          # ~0.6 s stationary
            samples.append(XC.Sample(hold[0], hold[1], t)); t += 0.05
        for i in range(10, 20):
            x, y = XC.point_at(ideal, cum, i * 0.5)
            samples.append(XC.Sample(x, y, t)); t += 0.05
        d = XC.dwell_events(samples, min_dwell_s=0.25)
        self.assertGreaterEqual(d["n"], 1)
        self.assertGreater(d["total_s"], 0.4)


class TestCompositeScoreShape(unittest.TestCase):
    def test_failures_are_graded_not_infinite(self):
        """A flat ``inf`` across an all-failing grid would leave coordinate
        descent unable to move — and the shipped tuning IS in a failing region."""
        ideal = XC.make_shape("Square", 10.0, 0.5)
        worse = XC.composite_score(_report(_timed(ideal[:5]), ideal))
        better = XC.composite_score(_report(_timed(ideal[:len(ideal) // 2]), ideal))
        self.assertFalse(worse["pass"])
        self.assertFalse(better["pass"])
        self.assertTrue(math.isfinite(worse["score"]))
        self.assertTrue(math.isfinite(better["score"]))
        self.assertGreater(worse["score"], better["score"])   # gradient survives

    def test_score_is_monotone_in_deviation(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        scores = []
        for off_mm in (0.005, 0.025, 0.060, 0.150):
            s = XC.as_samples([(p[0] + off_mm, p[1], i * 0.02)
                               for i, p in enumerate(ideal)])
            scores.append(XC.composite_score(_report(s, ideal))["score"])
        self.assertEqual(scores, sorted(scores))

    def test_pass_boundary_is_the_resolution_element(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        good = XC.as_samples([(p[0] + 0.020, p[1], i * 0.02)
                              for i, p in enumerate(ideal)])
        bad = XC.as_samples([(p[0] + 0.150, p[1], i * 0.02)
                             for i, p in enumerate(ideal)])
        self.assertTrue(XC.composite_score(_report(good, ideal, res=30.0))["pass"])
        self.assertFalse(XC.composite_score(_report(bad, ideal, res=30.0))["pass"])

    def test_weights_are_overridable_and_defaulted(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        rep = _report(_timed(ideal, dt=0.20), ideal, speed=5.0)
        base = XC.composite_score(rep)["score"]
        heavy = XC.composite_score(rep, weights={"w_time": 2.0})["score"]
        self.assertGreater(heavy, base)
        self.assertIn("w_time", XC.OBJECTIVE_DEFAULTS)

    def test_report_keeps_the_legacy_numbers_for_comparison(self):
        ideal = XC.make_shape("Square", 10.0, 0.5)
        rep = _report(_timed(ideal), ideal)
        self.assertIn("legacy", rep)
        self.assertEqual(set(rep["legacy"]), {"rms_um", "max_um", "mean_um", "n"})


class TestVerdictForRobustness(unittest.TestCase):
    """RMS alone lets a single huge corner blow-out pass — which is exactly what a
    robustness sweep exists to catch."""

    def test_requires_both_rms_and_max(self):
        self.assertEqual(
            XC.verdict_for({"rms_um": 20.0, "max_um": 50.0, "n": 100,
                            "completion_frac": 1.0}, 30.0), "pass")
        self.assertEqual(
            XC.verdict_for({"rms_um": 20.0, "max_um": 500.0, "n": 100,
                            "completion_frac": 1.0}, 30.0), "fail")

    def test_marginal_band(self):
        self.assertEqual(
            XC.verdict_for({"rms_um": 45.0, "max_um": 100.0, "n": 100,
                            "completion_frac": 1.0}, 30.0), "marginal")

    def test_never_run_and_incomplete_are_failures_not_passes(self):
        self.assertEqual(XC.verdict_for({"rms_um": 0.0, "max_um": 0.0, "n": 0},
                                        30.0), "fail")
        self.assertEqual(
            XC.verdict_for({"rms_um": 5.0, "max_um": 5.0, "n": 100,
                            "completion_frac": 0.5}, 30.0), "fail")


class TestNewDiagnosticShapes(unittest.TestCase):
    def test_all_shapes_build_centred_and_within_size(self):
        for name in XC.CHALLENGE_SHAPES:
            pts = XC.make_shape(name, 10.0, 0.5)
            self.assertGreaterEqual(len(pts), 8, msg=name)
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            self.assertAlmostEqual(0.5 * (min(xs) + max(xs)), 0.0, delta=0.6,
                                   msg=name)
            self.assertAlmostEqual(0.5 * (min(ys) + max(ys)), 0.0, delta=0.6,
                                   msg=name)
            self.assertLessEqual(max(xs) - min(xs), 10.5, msg=name)

    def test_line_reversal_is_pure_reversals_with_no_curvature(self):
        pts = XC.make_shape("Line-Reversal", 10.0, 0.5, passes=4)
        self.assertTrue(all(abs(p[1]) < 1e-9 for p in pts))   # single axis
        meta = XC.shape_meta("Line-Reversal", 10.0, 0.5, passes=4)
        self.assertEqual(len(meta["corner_s_mm"]), 3)         # 4 legs → 3 flips

    def test_comb_pitch_reaches_below_the_resolution_element(self):
        """The finest tooth pair is 0.75× the requested pitch, so asking for the
        element itself puts the last pair deliberately BELOW it — that is the
        point of the shape."""
        meta = XC.shape_meta("Comb", 10.0, 0.5, pitch_mm=0.030)
        self.assertLess(meta["feature_pitch_mm"] * 1000.0, 30.0)
        self.assertAlmostEqual(meta["feature_pitch_mm"] * 1000.0, 22.5, places=6)

    def test_dwell_stitch_reports_its_commanded_stops(self):
        meta = XC.shape_meta("Dwell-Stitch", 10.0, 0.5, stops=5)
        self.assertEqual(len(meta["dwell_s_mm"]), 5)
        self.assertTrue(all(0.0 < s < 10.0 for s in meta["dwell_s_mm"]))

    def test_sub_params_are_reachable(self):
        self.assertGreater(len(XC.make_shape("Star", 10.0, 0.5, points=8)),
                           len(XC.make_shape("Star", 10.0, 0.5, points=3)))
        self.assertGreater(len(XC.make_shape("Spiral", 10.0, 0.5, turns=6.0)),
                           len(XC.make_shape("Spiral", 10.0, 0.5, turns=2.0)))

    def test_shape_params_and_hints_cover_every_shape(self):
        for name in XC.CHALLENGE_SHAPES:
            self.assertIn(name, XC.SHAPE_HINTS, msg=name)
        for name in XC.SHAPE_PARAMS:
            self.assertIn(name, XC.CHALLENGE_SHAPES, msg=name)

    def test_unknown_shape_still_raises(self):
        with self.assertRaises(ValueError):
            XC.make_shape("nope", 10.0, 0.5)


if __name__ == "__main__":
    unittest.main()
