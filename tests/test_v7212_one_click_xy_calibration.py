"""test_v7212_one_click_xy_calibration.py — the ONE XY calibration.

Covers the change that replaced a row of single-setting probe buttons with a single
"Run XY Calibration", and fixed the loop that made the old one-click path
impossible to complete.

The load-bearing tests, and why each exists:

* ``TestTheOperatorsBug`` — with an empty timing store the derivation cannot run,
  which is what made the old one-click abort telling the operator to press a button
  that no longer wrote the store. One call to ``commit_top_speed`` must unstick it.
* ``TestGridStraddlesTheCornerBudget`` — the grid's lookahead floor must come from
  the shape's sharpest corner, not from a fraction of the closed-form value. Guarded
  by its own inverse, so it cannot pass because nothing varies.
* ``TestGridAxesActuallyChangeTheScore`` — pins the silently-inert-axis traps (the
  simulator's key names differ from the store's; ``min_lookahead_frac`` re-raises
  every candidate's lookahead). A flat grid must fail.
* ``TestGridIsSimulationOnly`` — the grid must not touch the stage. This is the
  8-to-22-minute regression.
"""

import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses import XYAutoCalibration as AC
from SupportClasses import XYCalibrationRun as CR
from SupportClasses import XYChallenge as XC
from SupportClasses import XYPathSimulator as PS
from SupportClasses.PrintTimingCalibrationStore import PrintTimingCalibrationStore
from SupportClasses.XYFeedPlan import corner_budget_lookahead_mm
from SupportClasses.XYStageModel import StageCharacteristics


# Real ME3B_01 numbers, from config/hardware/ME3B_01/print_timing_calibration.json
# plus logs/timing/topspeed_20260817_171958.jsonl. Using the machine's own measured
# dynamics means these tests describe a stage that exists.
ME3B = dict(dead_time_s=0.0817, tau_s=0.0378, top_speed_um_s=6036.0,
            control_loop_ms=58.03, quant_um=1.0)


def _char():
    return StageCharacteristics(**ME3B)


def _measured():
    return AC.MeasuredMachine(
        control_loop_ms=ME3B["control_loop_ms"], dead_time_s=ME3B["dead_time_s"],
        tau_s=ME3B["tau_s"], top_speed_um_s=ME3B["top_speed_um_s"])


def _derived(target=2.0, res=30.0):
    return AC.derive_settings(
        _measured(), AC.CalibrationPolicy(target_speed_mm_s=target,
                                          resolution_um=res))


def _store():
    d = tempfile.mkdtemp()
    return PrintTimingCalibrationStore(Path(d) / "t.json")


class _FakeSettings:
    def __init__(self):
        self.data = {}
        self.saves = 0

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value

    def save(self):
        self.saves += 1


class _RecordingCtrl:
    """Records every write, so a test can assert the ORDER as well as the effect."""

    def __init__(self, declared=None):
        self.calls = []
        self._declared = declared
        self.safety_limits = type("SL", (), {"max_xy_speed": 10000.0})()
        self.notified = 0

    def declared_xy_top_speed_um_s(self):
        return self._declared

    def set_xy_top_speed_um_s(self, v):
        self.calls.append(("declaration", v))
        self._declared = v
        self.safety_limits.max_xy_speed = v

    def notify_speed_limits_changed(self):
        self.notified += 1


class _RecordingStore:
    def __init__(self, inner):
        self.inner = inner
        self.calls = []

    def set_xy_max_speed_um_s(self, v):
        self.calls.append(("store", v))
        self.inner.set_xy_max_speed_um_s(v)

    def __getattr__(self, name):
        return getattr(self.inner, name)


# ── 1. the operator's bug ─────────────────────────────────────────────

class TestTheOperatorsBug(unittest.TestCase):
    def test_empty_store_cannot_derive_then_commit_unsticks_it(self):
        st = _store()
        st.set_control_loop_ms(ME3B["control_loop_ms"])
        st.set_velocity_dead_time_s(ME3B["dead_time_s"], n=5,
                                    tau_s=ME3B["tau_s"])

        # BEFORE: comms + dead time are known, the top speed is not, so the
        # derivation refuses — this is precisely the state the old one-click hit,
        # and it told the operator to press a button that wrote settings.json.
        before = AC.measured_from_store(st)
        self.assertFalse(before.is_complete())
        self.assertIn("top speed", before.missing())
        self.assertFalse(StageCharacteristics.from_store(st).is_complete())

        # ONE call fixes it.
        applied = CR.commit_top_speed(_RecordingCtrl(), _FakeSettings(), st,
                                      6036.0, fit={"r2": 0.999})
        self.assertTrue(applied["ok"])

        after = AC.measured_from_store(st)
        self.assertTrue(after.is_complete(),
                        "commit_top_speed must unblock the derivation")
        self.assertTrue(AC.derive_settings(
            after, AC.CalibrationPolicy()).values)
        self.assertTrue(StageCharacteristics.from_store(st).is_complete(),
                        "and unblock the simulator, which otherwise models an "
                        "UNCLAMPED stage (command_velocity clamps only if top>0)")

    def test_an_empty_store_models_an_unclamped_stage(self):
        """Guard the guard: show WHY the store write matters, not just that it
        happens. With no top speed the model imposes no velocity clamp at all, so
        every simulated candidate looks better than it can possibly be."""
        from SupportClasses.XYStageModel import XYStageModel
        blind = XYStageModel(StageCharacteristics(
            dead_time_s=0.08, tau_s=0.03, control_loop_ms=58.0,
            top_speed_um_s=0.0))
        blind.command_velocity(500000.0, 0.0)      # 500 mm/s — absurd
        blind.advance(1.0)
        self.assertGreater(abs(blind.velocity()[0]), 6036.0,
                           "an uncharacterised model accepts any speed")

        clamped = XYStageModel(_char())
        clamped.command_velocity(500000.0, 0.0)
        clamped.advance(1.0)
        self.assertLessEqual(abs(clamped.velocity()[0]), 6036.0 + 1.0)


# ── 2. the write-through ──────────────────────────────────────────────

class TestWriteThroughHitsEveryHome(unittest.TestCase):
    def test_all_homes_and_the_store_goes_first(self):
        ctrl = _RecordingCtrl()
        cfg = _FakeSettings()
        st = _RecordingStore(_store())
        applied = CR.commit_top_speed(ctrl, cfg, st, 6036.0, fit={"r2": 0.999})

        self.assertTrue(applied["store"])
        self.assertTrue(applied["declaration"])
        self.assertTrue(applied["settings_safety"])
        self.assertTrue(applied["settings_device_profile"])
        self.assertEqual(st.get_xy_max_speed_um_s(), 6036.0)
        self.assertEqual(ctrl.declared_xy_top_speed_um_s(), 6036.0)
        self.assertEqual(ctrl.safety_limits.max_xy_speed, 6036.0)
        self.assertEqual(cfg.data["safety_limits.max_xy_speed"], 6036.0)
        self.assertEqual(cfg.data["device_profile.xy_max_speed_um_s"], 6036.0)
        self.assertEqual(cfg.data["device_profile.xy_max_speed_source"],
                         "measured")
        self.assertGreaterEqual(cfg.saves, 1)

        order = [k for k, _ in (st.calls + ctrl.calls)]
        self.assertEqual(order[0], "store",
                         "the store is written FIRST — it is the home that "
                         "unblocks the derivation and the simulator")

    def test_notify_is_left_to_the_gui_thread(self):
        ctrl = _RecordingCtrl()
        applied = CR.commit_top_speed(ctrl, _FakeSettings(), _store(), 6036.0)
        self.assertTrue(applied["notify_pending"])
        self.assertEqual(ctrl.notified, 0,
                         "notify_speed_limits_changed runs refresh_jog_speed_"
                         "limits inline and is not thread-safe — the caller "
                         "must do it on the GUI thread")

    def test_refuses_a_wild_measurement_on_a_poor_fit(self):
        ctrl = _RecordingCtrl(declared=6000.0)
        st = _store()
        applied = CR.commit_top_speed(ctrl, _FakeSettings(), st, 60000.0,
                                      fit={"r2": 0.4})
        self.assertFalse(applied["ok"])
        self.assertIsNone(st.get_xy_max_speed_um_s())
        self.assertEqual(ctrl.declared_xy_top_speed_um_s(), 6000.0)

    def test_a_good_fit_is_applied_even_when_it_disagrees(self):
        # The gate is about MEASUREMENT QUALITY, not about protecting a typed
        # guess: a clean fit that disagrees is exactly the case where the typed
        # estimate was wrong, which is the whole point of measuring.
        ctrl = _RecordingCtrl(declared=1000.0)
        applied = CR.commit_top_speed(ctrl, _FakeSettings(), _store(), 6036.0,
                                      fit={"r2": 0.999})
        self.assertTrue(applied["ok"])
        self.assertEqual(ctrl.declared_xy_top_speed_um_s(), 6036.0)

    def test_rejects_nonsense(self):
        for bad in (0.0, -5.0, None, "fast"):
            applied = CR.commit_top_speed(_RecordingCtrl(), _FakeSettings(),
                                          _store(), bad)
            self.assertFalse(applied["ok"], f"{bad!r} must not be applied")


# ── 3. the grid ───────────────────────────────────────────────────────

class TestGridStraddlesTheCornerBudget(unittest.TestCase):
    def test_floor_is_the_corner_budget_not_a_fraction_of_la0(self):
        req = CR.CalibrationRequest(target_speed_mm_s=2.0, grid_level="medium",
                                    feature_mm=2.0, step_mm=0.1)
        ideal = XC.make_shape("Star", 2.0, 0.1)
        d = _derived(target=2.0)
        la_axis, _cf = CR.build_grid(d, req, ideal)
        la0 = d.values["lookahead_mm"]

        budget = corner_budget_lookahead_mm(
            CR.sharpest_turn_deg(ideal), req.resolution_um / 1.6 / 1000.0)
        self.assertAlmostEqual(la_axis[0], max(CR.MIN_LOOKAHEAD_MM, budget),
                               places=6)
        self.assertLess(la_axis[0], la0 / 4.0,
                        "the corner budget must sit BELOW the la0/4 floor that "
                        "misses the optimum")
        self.assertLess(la_axis[0], la0)
        self.assertGreater(la_axis[-1], la0,
                           "the axis must still straddle the closed-form value")

    def test_the_la0_over_4_floor_would_have_missed_it(self):
        """Guard the guard.

        Measured head to head on ME3B_01 with a 2 mm star at 2 mm/s: an axis
        starting at ``la0/4`` yields p95 40.2 µm (FAIL) while one starting at the
        corner budget yields 15.3 µm (PASS). Without this the straddle test above
        could pass while the grid still found nothing usable.
        """
        ideal = XC.make_shape("Star", 2.0, 0.1)
        ch, d = _char(), _derived(target=2.0)
        base = dict(d.values)
        la0 = base["lookahead_mm"]

        def _best(axis):
            best = None
            for la in axis:
                for cf in (0.2, 0.6, 1.0):
                    r, _ = CR._sim_one(ideal, ch, base_values=base,
                                       lookahead_mm=la, corner_factor=cf,
                                       speed_mm_s=2.0, resolution_um=30.0)
                    p95 = r.report["p95_um"]
                    if best is None or p95 < best:
                        best = p95
            return best

        budget = corner_budget_lookahead_mm(CR.sharpest_turn_deg(ideal),
                                            30.0 / 1.6 / 1000.0)
        naive = _best(CR._geomspace(la0 / 4.0, 2 * la0, 7))
        good = _best(CR._geomspace(max(CR.MIN_LOOKAHEAD_MM, budget), 2 * la0, 7))
        self.assertGreater(naive, 30.0, "the la0/4 axis must FAIL the element")
        self.assertLessEqual(good, 30.0, "the corner-budget axis must PASS")


class TestGridAxesActuallyChangeTheScore(unittest.TestCase):
    def test_lookahead_changes_the_result(self):
        """Pins BOTH silently-inert-axis traps at once.

        (a) the simulator's tuning keys are ``lookahead``/``corner_factor``, not the
        store's ``lookahead_mm``/``corner_speed_factor`` — a store-named key is
        silently ignored; (b) ``min_lookahead_frac`` carried over from the
        derivation re-raises EVERY candidate's lookahead back to ``la0``. Either
        one alone makes every grid cell score identically, and the grid then
        returns an arbitrary point that looks exactly like a result.
        """
        ideal = XC.make_shape("Star", 2.0, 0.1)
        ch, base = _char(), dict(_derived(target=2.0).values)
        scores = []
        for la in (0.06, 0.2, 0.8):
            r, _ = CR._sim_one(ideal, ch, base_values=base, lookahead_mm=la,
                               corner_factor=0.4, speed_mm_s=2.0,
                               resolution_um=30.0)
            scores.append(round(r.report["p95_um"], 3))
        self.assertEqual(len(set(scores)), len(scores),
                         f"lookahead must change the outcome; got {scores}")

    def test_store_named_keys_are_ignored_by_the_simulator(self):
        """The trap itself, stated as a fact about the simulator."""
        t_wrong = dict(PS.TUNING_DEFAULTS)
        t_wrong["lookahead_mm"] = 0.9            # store name — ignored
        t_right = CR.tuning_from_store_keys({"lookahead_mm": 0.9})
        self.assertNotEqual(
            PS.resolve_for(_char(), print_speed_mm_s=2.0,
                           tuning=t_wrong)["lookahead_mm"],
            PS.resolve_for(_char(), print_speed_mm_s=2.0,
                           tuning=t_right)["lookahead_mm"])
        self.assertAlmostEqual(
            PS.resolve_for(_char(), print_speed_mm_s=2.0,
                           tuning=t_right)["lookahead_mm"], 0.9, places=6)

    def test_candidate_values_neutralise_the_two_overriding_levers(self):
        vals = CR.candidate_values({"min_lookahead_frac": 5.0,
                                    "hold_speed": 1.0, "decel_mm": 1.5},
                                   lookahead_mm=0.08, corner_factor=0.3)
        self.assertEqual(vals["min_lookahead_frac"], 0.0)
        self.assertEqual(vals["hold_speed"], 0.0)
        self.assertEqual(vals["decel_mm"], 1.5, "other derived values survive")

    def test_finalise_reproduces_the_simulated_lookahead(self):
        """What is persisted must reproduce what was simulated: the neutralised
        levers are re-derived from the WINNER, not simply dropped."""
        d = _derived(target=2.0)
        best = CR.GridCandidate(lookahead_mm=0.12, corner_speed_factor=0.5,
                                score=1.0, passed=True, p95_um=20.0,
                                max_um=30.0, rms_um=10.0, completion=1.0,
                                dither=1.0)
        vals = CR.finalise_values(d, best, speed_mm_s=2.0, char=_char())
        resolved = PS.resolve_for(_char(), print_speed_mm_s=2.0,
                                  tuning=CR.tuning_from_store_keys(vals))
        self.assertAlmostEqual(resolved["lookahead_mm"], 0.12, places=3)


class TestGridIsSimulationOnly(unittest.TestCase):
    def test_the_grid_never_touches_the_stage(self):
        """The 8-to-22-minute regression: the descent this replaces drove every
        candidate on hardware."""
        class _Boom:
            def __getattr__(self, name):
                raise AssertionError(
                    f"the grid must not touch the controller (called {name})")

        ideal = XC.make_shape("Star", 2.0, 0.25)
        req = CR.CalibrationRequest(target_speed_mm_s=2.0,
                                    grid_level="very_coarse", feature_mm=2.0)
        # _Boom is simply never passed in — score_grid takes no controller at all,
        # which is the structural guarantee. Assert that signature explicitly.
        import inspect
        params = inspect.signature(CR.score_grid).parameters
        self.assertNotIn("controller", params)
        g = CR.score_grid(ideal, _char(), derived=_derived(target=2.0),
                          request=req)
        self.assertGreater(g.n, 0)

    def test_every_coarseness_finishes_quickly(self):
        import time
        ideal = XC.make_shape("Star", 2.0, 0.1)
        d = _derived(target=2.0)
        for level in ("very_coarse", "medium"):
            req = CR.CalibrationRequest(target_speed_mm_s=2.0,
                                        grid_level=level, feature_mm=2.0,
                                        step_mm=0.1)
            t0 = time.monotonic()
            g = CR.score_grid(ideal, _char(), derived=d, request=req)
            self.assertLess(time.monotonic() - t0, 60.0)
            self.assertIsNotNone(g.best)

    def test_grid_levels_map_to_points_per_axis(self):
        self.assertEqual(
            [CR.GRID_LEVELS[k] for k in CR.GRID_LEVEL_ORDER],
            [3, 5, 7, 11, 15])
        for k in CR.GRID_LEVEL_ORDER:
            self.assertIn(k, CR.GRID_LEVEL_LABELS)


class TestSpeedIsAnOutput(unittest.TestCase):
    def test_the_winner_reports_the_resolved_cap_not_the_request(self):
        """``resolve_control`` caps the commanded speed, so quoting the request
        would be false: on this rig a requested 2 mm/s runs at well under 1."""
        ideal = XC.make_shape("Star", 2.0, 0.1)
        req = CR.CalibrationRequest(target_speed_mm_s=2.0, grid_level="coarse",
                                    feature_mm=2.0, step_mm=0.1)
        g = CR.score_grid(ideal, _char(), derived=_derived(target=2.0),
                          request=req)
        self.assertIsNotNone(g.best)
        self.assertGreater(g.resolved_cap_mm_s, 0.0)
        self.assertLess(g.resolved_cap_mm_s, g.speed_mm_s,
                        "the achievable speed is BELOW the request on a 2 mm "
                        "feature — that is the honest number to report")
        self.assertTrue(g.cap_reason)

    def test_ranking_prefers_the_fastest_passing_candidate(self):
        caps = {(0.10, 0.5): 0.36, (0.20, 0.5): 0.72}

        def _c(la, passed, p95):
            return CR.GridCandidate(lookahead_mm=la, corner_speed_factor=0.5,
                                    score=p95, passed=passed, p95_um=p95,
                                    max_um=p95, rms_um=p95 / 2,
                                    completion=1.0, dither=1.0)
        # The slower candidate has the BETTER deviation; the faster one must still
        # win, or the tune reproduces "just bring the velocity down".
        best = CR._pick_best([_c(0.10, True, 5.0), _c(0.20, True, 25.0)],
                             caps=caps, la0=0.4)
        self.assertAlmostEqual(best.lookahead_mm, 0.20)

    def test_falls_back_to_least_bad_when_nothing_passes(self):
        caps = {(0.10, 0.5): 0.36, (0.20, 0.5): 0.72}

        def _c(la, p95):
            return CR.GridCandidate(lookahead_mm=la, corner_speed_factor=0.5,
                                    score=p95, passed=False, p95_um=p95,
                                    max_um=p95, rms_um=p95, completion=1.0,
                                    dither=1.0)
        best = CR._pick_best([_c(0.10, 50.0), _c(0.20, 900.0)],
                             caps=caps, la0=0.4)
        self.assertAlmostEqual(best.lookahead_mm, 0.10)


# ── 4. safety + abort ─────────────────────────────────────────────────

class _StageSpy:
    """Enough controller for the orchestrator, and it FAILS on any Z motion."""

    def __init__(self):
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.retracts = []
        self.stops = 0
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        # A REAL envelope: ME3B_01 is 116.3 x 74.3 mm. `fit_shape_size_mm`
        # needs xy_min/max to size (and shrink) the test shape, so a thin
        # stub makes the run refuse with "shape does not fit".
        self.safety_limits = type("SL", (), {
            "max_xy_speed": 6036.0, "enabled": True,
            "xy_min_x": 0.0, "xy_max_x": 116327.0,
            "xy_min_y": 0.0, "xy_max_y": 74300.0})()

    def default_plate_center_um(self):
        return (50000.0, 40000.0)

    def ensure_retracted_to(self, z):
        self.retracts.append(z)
        return True

    def send_velocity_xy(self, vx, vy):
        if vx == 0 and vy == 0:
            self.stops += 1

    def move_z_absolute(self, *a, **k):
        raise AssertionError("the calibration must never command Z directly")

    def move_z_relative(self, *a, **k):
        raise AssertionError("the calibration must never command Z directly")

    def declared_xy_top_speed_um_s(self):
        return None

    def set_xy_top_speed_um_s(self, v):
        pass


class TestSafetyAndAbort(unittest.TestCase):
    def test_refuses_without_safe_z_when_zp_is_connected(self):
        """A run that cannot retract has no business moving the stage."""
        ctrl = _StageSpy()
        res = CR.run_calibration(
            ctrl, settings=_FakeSettings(), store=_store(),
            request=CR.CalibrationRequest(safe_z_mm=None))
        self.assertFalse(res.ok)
        self.assertIn("Safe Z", res.error)
        self.assertEqual(ctrl.retracts, [],
                         "it must refuse BEFORE any motion")

    def test_abort_before_the_first_step_saves_nothing(self):
        ctrl = _StageSpy()
        st = _store()
        stop = threading.Event()
        stop.set()
        res = CR.run_calibration(
            ctrl, settings=_FakeSettings(), store=st,
            request=CR.CalibrationRequest(safe_z_mm=5.0), stop_evt=stop)
        self.assertTrue(res.aborted or not res.ok)
        self.assertIsNone(st.get_xy_max_speed_um_s(),
                          "an aborted run must not persist a top speed")

    def test_step_order_is_the_documented_one(self):
        self.assertEqual(
            CR.STEP_ORDER,
            ("centre", "comms", "deadtime", "topspeed", "commit",
             "derive", "grid", "verify", "persist"))
        for k in CR.STEP_ORDER:
            self.assertIn(k, CR.STEP_TITLES)
            self.assertIn(k, CR.STEP_NOMINAL_S)

    def test_the_whole_nominal_run_fits_the_three_minute_budget(self):
        self.assertLess(sum(CR.STEP_NOMINAL_S.values()),
                        CR.CalibrationRequest().time_budget_s)


# ── 5. the poller suspend is refcounted ───────────────────────────────

class TestPollerSuspendIsRefcounted(unittest.TestCase):
    def test_nested_suspend_survives_an_inner_resume(self):
        """The calibration nests suspends: the orchestrator holds one while each
        sub-probe owns its own. With the old plain bool, the FIRST sub-probe's
        `finally` resumed polling for everything after it, so every later step was
        measured against a contended serial read path."""
        from SupportClasses.StageController import PositionPoller
        p = PositionPoller.__new__(PositionPoller)
        p._lock = threading.Lock()
        p._suspend_depth = 0
        p._zp_fail_count = 0

        p.suspend()
        p.suspend()
        p.resume()
        self.assertTrue(p._suspended,
                        "an inner resume must NOT un-suspend the outer caller")
        p.resume()
        self.assertFalse(p._suspended)

    def test_an_unbalanced_resume_cannot_poison_a_later_suspend(self):
        from SupportClasses.StageController import PositionPoller
        p = PositionPoller.__new__(PositionPoller)
        p._lock = threading.Lock()
        p._suspend_depth = 0
        p._zp_fail_count = 0
        p.resume()
        p.resume()
        p.suspend()
        self.assertTrue(p._suspended)


# ── 6. the whole run, end to end ──────────────────────────────────────

class _FullCtrl(_StageSpy):
    """Enough controller to drive the orchestration. The three PROBES are
    monkeypatched (they have their own suites); what is exercised here is the
    ORDER, the commit, the derivation, the grid, the persist and the retract."""

    def measure_control_loop_rate(self, **kw):
        return {"avg_period_ms": ME3B["control_loop_ms"],
                "control_hz": 1000.0 / ME3B["control_loop_ms"], "moved": True}


class TestTheWholeRun(unittest.TestCase):
    def _patched(self):
        from unittest.mock import patch
        return (
            patch.object(CR.AC, "center_stage",
                         lambda ctrl, **k: {"ok": True,
                                            "center_um": (50000.0, 40000.0)}),
            patch.object(CR.DT, "measure_velocity_dead_time",
                         lambda ctrl, **k: {
                             "dead_time_s": ME3B["dead_time_s"],
                             "apparent_lag_s": ME3B["dead_time_s"],
                             "apparent_lag_spread_s": 0.0015,
                             "tau_s": ME3B["tau_s"], "cruise_um_s": 3000.0,
                             "n": 5}),
            patch.object(CR.TS, "measure_top_speed",
                         lambda ctrl, **k: {
                             "top_speed_um_s": ME3B["top_speed_um_s"],
                             "slope_s_per_mm": 0.16567, "intercept_s": 0.264,
                             "r2": 0.9997, "n": 5, "rows": [],
                             "distances_mm": []}),
        )

    def test_happy_path_completes_and_persists(self):
        ctrl, st, cfg = _FullCtrl(), _store(), _FakeSettings()
        req = CR.CalibrationRequest(target_speed_mm_s=2.0, safe_z_mm=5.0,
                                    grid_level="very_coarse", feature_mm=2.0,
                                    step_mm=0.25, verify_on_hardware=False)
        a, b, c = self._patched()
        with a, b, c:
            res = CR.run_calibration(ctrl, settings=cfg, store=st, request=req)

        self.assertTrue(res.ok, res.error or res.summary)
        self.assertFalse(res.aborted)

        # every step reached a terminal state, in order
        self.assertEqual([s.step for s in res.steps], list(CR.STEP_ORDER))
        for s in res.steps:
            self.assertIn(s.state, ("done", "skipped"), f"{s.step}: {s.state}")

        # the three measurements reached the store …
        self.assertAlmostEqual(st.get_control_loop_ms(),
                               ME3B["control_loop_ms"], places=2)
        self.assertGreater(st.get_velocity_dead_time_s(), 0)
        self.assertAlmostEqual(st.get_xy_max_speed_um_s(),
                               ME3B["top_speed_um_s"], places=1)
        # … and both settings homes
        self.assertAlmostEqual(cfg.data["safety_limits.max_xy_speed"],
                               ME3B["top_speed_um_s"], places=1)
        self.assertAlmostEqual(cfg.data["device_profile.xy_max_speed_um_s"],
                               ME3B["top_speed_um_s"], places=1)

        # the tuning is persisted, and it is the WINNER's, not the default
        vel = st.get_mode_params("velocity")
        self.assertAlmostEqual(vel["lookahead_mm"], res.grid.best.lookahead_mm,
                               places=4)
        self.assertNotAlmostEqual(vel["lookahead_mm"], 0.2, places=4)

        # safety: retracted on the way in AND in the finally
        self.assertGreaterEqual(len(ctrl.retracts), 2)
        self.assertEqual(set(ctrl.retracts), {5.0})
        self.assertGreaterEqual(ctrl.stops, 1)
        self.assertLess(res.wall_s, 180.0)

    def test_quick_print_would_now_inherit_the_tuning(self):
        """The operator's follow-up: after calibration a print must pick these up.

        `stamp_print_settings` is the ONE stamper both Quick Print and Full Print
        call at build time, so proving it reads the persisted values proves the
        whole path — no restart, no extra wiring.
        """
        ctrl, st, cfg = _FullCtrl(), _store(), _FakeSettings()
        req = CR.CalibrationRequest(target_speed_mm_s=2.0, safe_z_mm=5.0,
                                    grid_level="very_coarse", feature_mm=2.0,
                                    step_mm=0.25, verify_on_hardware=False)
        a, b, c = self._patched()
        with a, b, c:
            res = CR.run_calibration(ctrl, settings=cfg, store=st, request=req)
        self.assertTrue(res.ok)

        class _PS:
            pass
        ps = AC.stamp_print_settings(_PS(), st)
        self.assertAlmostEqual(ps.vel_lookahead_mm, res.grid.best.lookahead_mm,
                               places=4)
        self.assertAlmostEqual(ps.xy_max_speed_um_s, ME3B["top_speed_um_s"],
                               places=1)
        self.assertGreater(ps.vel_tuning.get("lookahead_mm", 0), 0)
        # and the simulator/feed-plan path is unblocked
        self.assertTrue(StageCharacteristics.from_store(st).is_complete())

    def test_an_abort_midway_persists_nothing(self):
        ctrl, st, cfg = _FullCtrl(), _store(), _FakeSettings()
        stop = threading.Event()
        req = CR.CalibrationRequest(target_speed_mm_s=2.0, safe_z_mm=5.0,
                                    grid_level="very_coarse", feature_mm=2.0,
                                    step_mm=0.25)

        def _abort_at_deadtime(ctrl_, **k):
            stop.set()
            return {"dead_time_s": 0.08, "apparent_lag_s": 0.08,
                    "apparent_lag_spread_s": 0.0, "tau_s": 0.03,
                    "cruise_um_s": 3000.0, "n": 5}
        from unittest.mock import patch
        a, _b, c = self._patched()
        with a, c, patch.object(CR.DT, "measure_velocity_dead_time",
                                _abort_at_deadtime):
            res = CR.run_calibration(ctrl, settings=cfg, store=st,
                                     request=req, stop_evt=stop)
        self.assertTrue(res.aborted)
        self.assertIsNone(st.get_xy_max_speed_um_s(),
                          "an aborted run must not declare a top speed")
        self.assertIn(5.0, ctrl.retracts)


if __name__ == "__main__":
    unittest.main()
