"""test_v75x_xy_auto_calibration.py — one-shot systematic XY calibration.

The old auto-tune SEARCHED for parameters. It was slow (26 hardware runs for a
velocity tune), non-repeatable, and searched over quantities that are not free
variables. Almost every follower parameter is a DERIVED CONSEQUENCE of three
measurements — loop period, dead time, top speed — through closed-form physics:

    L = dead_time + loop_period
    Ku = π/(2L), Tu = 4L                 (integrator + delay plant)
    speed_cap = lookahead/(L·safety)     (pure-pursuit stability)
    lookahead = target·L·safety          (...inverted, to ALLOW a target speed)
    decel ≥ target·L                     (the stage coasts for one dead time)

So calibration is measure → derive → verify, not search. These tests pin the
derivation, its validation warnings, and the envelope-centring safety rules.
"""

import math
import unittest
from types import SimpleNamespace

from SupportClasses import XYAutoCalibration as AC
from SupportClasses import VelocityControl as VC


#: The operator's real measured machine (ME3B V1, 2026-07-28).
def _me3b(**kw):
    d = dict(control_loop_ms=31.75, dead_time_s=0.0672,
             dead_time_spread_s=0.000546, tau_s=0.0281,
             top_speed_um_s=5945.611449038091)
    d.update(kw)
    return AC.MeasuredMachine(**d)


class TestMeasuredMachine(unittest.TestCase):
    def test_effective_delay_is_dead_time_plus_loop(self):
        self.assertAlmostEqual(_me3b().effective_delay_s(), 0.0672 + 0.03175,
                               places=9)

    def test_completeness_and_missing_list(self):
        self.assertTrue(_me3b().is_complete())
        m = AC.MeasuredMachine()
        self.assertFalse(m.is_complete())
        self.assertEqual(set(m.missing()),
                         {"comms rate", "dead time", "top speed"})
        self.assertEqual(_me3b(top_speed_um_s=0.0).missing(), ["top speed"])


class TestDerivationIsDeterministic(unittest.TestCase):
    def test_same_inputs_give_the_same_settings(self):
        """The property the searching tuner lacked."""
        p = AC.CalibrationPolicy()
        a = AC.derive_settings(_me3b(), p).values
        b = AC.derive_settings(_me3b(), p).values
        self.assertEqual(a, b)

    def test_refuses_when_not_measured(self):
        d = AC.derive_settings(AC.MeasuredMachine(), AC.CalibrationPolicy())
        self.assertEqual(d.values, {})
        self.assertTrue(any(w.level == "error" for w in d.warnings))

    def test_every_derived_value_carries_a_reason(self):
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        noted = {n.param for n in d.notes}
        for key in ("lookahead_mm", "pid_kp", "decel_mm", "control_hz"):
            self.assertIn(key, noted, msg=key)
        self.assertTrue(all(n.reason for n in d.notes))


class TestLookaheadIsSolvedForTheTargetSpeed(unittest.TestCase):
    """The crux. The legacy code let the lookahead SET the speed, so an
    error-minimising search drove it to the smallest value in its grid."""

    def test_lookahead_scales_with_the_target_speed(self):
        m = _me3b()
        slow = AC.derive_settings(m, AC.CalibrationPolicy(target_speed_mm_s=1.0))
        fast = AC.derive_settings(m, AC.CalibrationPolicy(target_speed_mm_s=5.0))
        self.assertGreater(fast.values["lookahead_mm"],
                           slow.values["lookahead_mm"])

    def test_the_derived_lookahead_actually_permits_the_target(self):
        """The contract: after derivation the runtime must be able to command the
        requested speed — the binding term should be the request itself, not the
        dead-time limit."""
        for target in (0.5, 1.0, 2.5, 5.0):
            p = AC.CalibrationPolicy(target_speed_mm_s=target)
            m = _me3b()
            d = AC.derive_settings(m, p)
            r = VC.resolve_control(
                print_speed_mm_s=target,
                lookahead_mm=d.values["lookahead_mm"],
                xy_max_speed_um_s=m.top_speed_um_s,
                control_loop_ms=m.control_loop_ms,
                dead_time_s=m.dead_time_s,
                lead_time_frac=d.values["lead_time_frac"],
                min_lookahead_frac=d.values["min_lookahead_frac"],
                max_speed_frac=d.values["max_speed_frac"],
                safety=d.values["deadtime_safety"])
            self.assertGreaterEqual(
                r["speed_cap_mm_s"], target - 1e-9,
                msg=f"target {target}: cap {r['speed_cap_mm_s']:.4f} "
                    f"({r['cap_reason']})")
            self.assertEqual(r["cap_reason"], "print_speed",
                             msg=f"target {target} should not be dead-time "
                                 f"limited after derivation")

    def test_lookahead_is_floored_at_the_noise_level(self):
        """A carrot inside the encoder noise floor is meaningless."""
        p = AC.CalibrationPolicy(target_speed_mm_s=0.1, resolution_um=30.0)
        d = AC.derive_settings(_me3b(), p)
        self.assertGreaterEqual(d.values["lookahead_mm"], 2 * 0.030 - 1e-9)

    def test_target_above_the_stage_capability_is_capped_and_warned(self):
        p = AC.CalibrationPolicy(target_speed_mm_s=50.0)
        m = _me3b()
        d = AC.derive_settings(m, p)
        ceiling = (m.top_speed_um_s / 1000.0) * p.max_speed_frac
        self.assertAlmostEqual(d.summary["target_speed_mm_s"], ceiling,
                               places=9)
        self.assertTrue(any("exceeds" in w.message for w in d.warnings))


class TestGainsAreAnalytic(unittest.TestCase):
    """The gain is computed in closed form, but only APPLIED once the control law
    can bound it — see TestGainsAreWithheldUntilTheLawCanBoundThem."""

    def _unlocked(self):
        """Pretend the bounded composition has landed."""
        self._orig = VC.SUPPORTS_BOUNDED_CROSS_TRACK
        VC.SUPPORTS_BOUNDED_CROSS_TRACK = True
        self.addCleanup(setattr, VC, "SUPPORTS_BOUNDED_CROSS_TRACK", self._orig)

    def test_kp_matches_the_closed_form_and_is_stable(self):
        self._unlocked()
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        ku, _tu = VC.plant_ultimate_gain(0.0672, 31.75)
        self.assertAlmostEqual(d.values["pid_kp"], round(0.33 * ku, 3), places=3)
        self.assertLess(d.values["pid_kp"], ku)
        self.assertAlmostEqual(d.summary["kp_stability_limit"], ku, places=6)

    def test_kd_and_ki_are_zero_by_default(self):
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        self.assertEqual(d.values["pid_kd"], 0.0)
        self.assertEqual(d.values["pid_ki"], 0.0)

    def test_gain_margin_reduces_kp(self):
        self._unlocked()
        base = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        safe = AC.derive_settings(_me3b(),
                                  AC.CalibrationPolicy(gain_margin=2.0))
        self.assertLess(safe.values["pid_kp"], base.values["pid_kp"])


class TestGainsAreWithheldUntilTheLawCanBoundThem(unittest.TestCase):
    """An automated calibration must not hand the machine a gain the control law
    will mishandle.

    While ``pursuit_step`` adds the PD correction to a full-magnitude pursuit
    vector with no re-clamp, a *correct* kp produces a mostly-perpendicular
    command (76 % sideways at the logged 0.64 mm error) and the follower runs
    away. A bench simulation of the one-click flow reproduced exactly that: the
    derived speed/geometry were right, and verification failed with
    completion 0.000 and 5.3 mm of cross-track. So the gain is computed, reported,
    and withheld until the composition is bounded.
    """

    def test_kp_is_zero_while_the_law_is_unbounded(self):
        self.assertFalse(VC.SUPPORTS_BOUNDED_CROSS_TRACK,
                         "flip this test's expectation when Stage 3.1 lands")
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        self.assertEqual(d.values["pid_kp"], 0.0)
        self.assertEqual(d.values["pid_kd"], 0.0)

    def test_the_withheld_gain_is_still_reported(self):
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        ku, _ = VC.plant_ultimate_gain(0.0672, 31.75)
        self.assertAlmostEqual(d.summary["pid_kp_pending"], round(0.33 * ku, 3),
                               places=3)
        self.assertTrue(any("PURE PURSUIT" in n.reason for n in d.notes))

    def test_speed_and_geometry_are_still_fully_calibrated(self):
        """Withholding the gain must not compromise the rest — the speed unlock
        is the larger win and is independent of the cross-track gain."""
        d = AC.derive_settings(_me3b(),
                               AC.CalibrationPolicy(target_speed_mm_s=5.0))
        self.assertGreater(d.values["lookahead_mm"], 0.9)
        self.assertEqual(d.values["hold_speed"], 1.0)
        self.assertGreater(d.values["decel_mm"], 0.4)

    def test_gain_switches_on_when_the_flag_flips(self):
        orig = VC.SUPPORTS_BOUNDED_CROSS_TRACK
        VC.SUPPORTS_BOUNDED_CROSS_TRACK = True
        try:
            d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
            self.assertGreater(d.values["pid_kp"], 5.0)
        finally:
            VC.SUPPORTS_BOUNDED_CROSS_TRACK = orig


class TestPaceCorrectionIsNotGuessed(unittest.TestCase):
    def test_not_derived_from_the_probe_cruise_speed(self):
        """The dead-time probe raises SMS to ~1.5× its own step magnitude, so its
        cruise reads back the PROBE speed, not the stage maximum — a ratio of the
        two is meaningless (it read 1.98 in simulation purely because the probe ran
        at 3 mm/s against a 5.9 mm/s stage)."""
        d = AC.derive_settings(_me3b(cruise_um_s=3000.0), AC.CalibrationPolicy())
        self.assertNotIn("pace_correction", d.values)

    def test_derivative_filter_tracks_the_loop_rate(self):
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        self.assertAlmostEqual(d.values["d_filter_hz"],
                               round(d.values["control_hz"] / 8.0, 1), places=1)


class TestDecelCoversTheCoastDistance(unittest.TestCase):
    """A decel window shorter than the dead-time coast guarantees endpoint
    overshoot — and the shipped value (0.3 mm) is short for 5 mm/s."""

    def test_decel_is_at_least_the_coast_distance(self):
        for target in (1.0, 2.5, 5.0):
            m = _me3b()
            d = AC.derive_settings(m, AC.CalibrationPolicy(
                target_speed_mm_s=target))
            coast = d.summary["target_speed_mm_s"] * m.effective_delay_s()
            self.assertGreaterEqual(d.values["decel_mm"], coast - 1e-9,
                                    msg=f"target {target}")

    def test_the_shipped_value_is_flagged_as_too_short(self):
        m = _me3b()
        p = AC.CalibrationPolicy(target_speed_mm_s=5.0)
        d = AC.derive_settings(m, p)
        shipped = dict(d.values, decel_mm=0.3)      # what the store held
        ws = AC.validate(m, p, shipped, summary=d.summary)
        self.assertTrue(any("coasts" in w.message for w in ws),
                        msg=[w.message for w in ws])


class TestHoldSpeedAndCornerInteraction(unittest.TestCase):
    def test_hold_speed_is_enabled_once_the_lookahead_supports_the_target(self):
        d = AC.derive_settings(_me3b(),
                               AC.CalibrationPolicy(target_speed_mm_s=5.0))
        self.assertEqual(d.values["hold_speed"], 1.0)

    def test_corner_factor_derived_from_a_measured_lateral_accel(self):
        m = _me3b(lateral_accel_mm_s2=200.0)
        d = AC.derive_settings(m, AC.CalibrationPolicy(target_speed_mm_s=5.0))
        la = d.values["lookahead_mm"]
        expect = math.sqrt(200.0 * la) / d.summary["target_speed_mm_s"]
        self.assertAlmostEqual(d.values["corner_speed_factor"],
                               round(min(1.0, max(0.05, expect)), 3), places=3)

    def test_corner_factor_falls_back_to_a_default_when_unmeasured(self):
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        self.assertEqual(d.values["corner_speed_factor"], 0.4)
        self.assertTrue(any("lateral-acceleration" in n.reason
                            for n in d.notes))

    def test_validate_flags_an_inert_corner_setting(self):
        """The operator's exact failure: the corner limit above the speed cap, so
        min(cap, corner) is always the cap and corner tuning does nothing."""
        m = _me3b()
        p = AC.CalibrationPolicy(target_speed_mm_s=5.0)
        d = AC.derive_settings(m, p)
        inert = dict(d.values, corner_speed_factor=0.2)
        bad_summary = dict(d.summary, achievable_cap_mm_s=0.314)
        ws = AC.validate(m, p, inert, summary=bad_summary)
        self.assertTrue(any("INERT" in w.message for w in ws),
                        msg=[w.message for w in ws])


class TestValidationCatchesRealFailures(unittest.TestCase):
    def test_unstable_kp_is_an_error(self):
        m = _me3b()
        p = AC.CalibrationPolicy()
        d = AC.derive_settings(m, p)
        ku, _ = VC.plant_ultimate_gain(m.dead_time_s, m.control_loop_ms)
        ws = AC.validate(m, p, dict(d.values, pid_kp=ku * 1.1),
                         summary=d.summary)
        self.assertTrue(any(w.level == "error" and "stability" in w.message
                            for w in ws))

    def test_tight_dead_time_measurement_is_reported_as_good(self):
        d = AC.derive_settings(_me3b(), AC.CalibrationPolicy())
        self.assertTrue(any("tight measurement" in w.message
                            for w in d.warnings))

    def test_noisy_dead_time_measurement_is_warned(self):
        m = _me3b(dead_time_spread_s=0.030)         # 45 % of the value
        d = AC.derive_settings(m, AC.CalibrationPolicy())
        self.assertTrue(any("re-measure" in w.message for w in d.warnings))

    def test_declared_vs_measured_top_speed_discrepancy_is_flagged(self):
        m = _me3b(declared_max_speed_um_s=50000.0)
        d = AC.derive_settings(m, AC.CalibrationPolicy())
        self.assertTrue(any("declares" in w.message for w in d.warnings),
                        msg=[w.message for w in d.warnings])

    def test_cruise_disagreeing_with_top_speed_is_flagged(self):
        m = _me3b(cruise_um_s=500.0)                # 12× disagreement
        d = AC.derive_settings(m, AC.CalibrationPolicy())
        self.assertTrue(any("disagrees" in w.message for w in d.warnings))


# ── Envelope centring (operator: never hit the travel extents) ─────────

def _ctrl(min_x=0.0, max_x=114332.0, min_y=0.0, max_y=76645.0, x=500.0, y=400.0,
          with_safe_travel=True):
    calls = {"safe_travel": [], "abs": [], "waits": []}

    def safe_travel_to(cx, cy, safe_z_mm=None, target_z_mm=None, **kw):
        calls["safe_travel"].append((cx, cy, safe_z_mm, target_z_mm))
        c.pos = (cx, cy)
        return True

    def move_xy_absolute_um(cx, cy):
        calls["abs"].append((cx, cy))
        c.pos = (cx, cy)

    c = SimpleNamespace(
        pos=(x, y), calls=calls,
        is_xy_connected=True,
        safety_limits=SimpleNamespace(
            max_xy_speed=50000.0, xy_min_x=min_x, xy_max_x=max_x,
            xy_min_y=min_y, xy_max_y=max_y,
            xy_center=lambda: ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)),
        default_plate_center_um=lambda: ((min_x + max_x) / 2.0,
                                        (min_y + max_y) / 2.0),
        move_xy_absolute_um=move_xy_absolute_um,
        wait_for_xy_arrival=lambda *a, **k: calls["waits"].append(a) or True,
    )
    if with_safe_travel:
        c.safe_travel_to = safe_travel_to
    return c


class TestEnvelopeCentring(unittest.TestCase):
    def test_centre_is_the_envelope_midpoint(self):
        c = _ctrl()
        self.assertEqual(AC.envelope_center_um(c), (57166.0, 38322.5))

    def test_usable_radius_keeps_a_margin_from_the_walls(self):
        c = _ctrl()
        half = min(114332.0, 76645.0) / 2.0
        self.assertAlmostEqual(AC.usable_test_radius_um(c),
                               half - AC.ENVELOPE_MARGIN_UM, places=6)

    def test_check_fits_accepts_and_rejects(self):
        c = _ctrl()
        self.assertTrue(AC.check_fits(c, 1000.0)["ok"])
        bad = AC.check_fits(c, 90000.0)
        self.assertFalse(bad["ok"])
        self.assertIn("travel limits", bad["reason"])

    def test_unknown_envelope_is_refused_not_guessed(self):
        c = SimpleNamespace(safety_limits=SimpleNamespace(max_xy_speed=1.0),
                            default_plate_center_um=lambda: None)
        self.assertEqual(AC.usable_test_radius_um(c), 0.0)
        r = AC.check_fits(c, 100.0)
        self.assertFalse(r["ok"])
        self.assertIn("envelope unknown", r["reason"])

    def test_shape_size_is_shrunk_to_fit(self):
        c = _ctrl(min_x=0.0, max_x=20000.0, min_y=0.0, max_y=20000.0)
        usable = AC.usable_test_radius_um(c)          # 10000 - 2000 = 8000
        self.assertAlmostEqual(AC.fit_shape_size_mm(c, 40.0),
                               2 * usable / 1000.0, places=6)
        # A request that already fits is left alone.
        self.assertAlmostEqual(AC.fit_shape_size_mm(c, 5.0), 5.0, places=6)

    def test_centring_retracts_z_before_the_cross_position_xy_move(self):
        """CLAUDE.md invariant: Z must retract BEFORE a cross-position XY move,
        and must never descend (target_z_mm=None)."""
        c = _ctrl()
        r = AC.center_stage(c, safe_z_mm=40.0)
        self.assertTrue(r["ok"])
        self.assertEqual(len(c.calls["safe_travel"]), 1)
        cx, cy, safe_z, target_z = c.calls["safe_travel"][0]
        self.assertEqual((cx, cy), (57166.0, 38322.5))
        self.assertEqual(safe_z, 40.0)
        self.assertIsNone(target_z)                  # never lowers
        self.assertEqual(c.calls["abs"], [])         # did NOT bypass safe travel

    def test_centring_refuses_when_the_retract_is_not_confirmed(self):
        c = _ctrl()
        c.safe_travel_to = lambda *a, **k: False     # retract not confirmed
        r = AC.center_stage(c, safe_z_mm=40.0)
        self.assertFalse(r["ok"])
        self.assertIn("retract", r["reason"])

    def test_xy_only_rig_falls_back_to_an_absolute_move(self):
        c = _ctrl(with_safe_travel=False)
        r = AC.center_stage(c, safe_z_mm=None)
        self.assertTrue(r["ok"])
        self.assertEqual(c.calls["abs"], [(57166.0, 38322.5)])

    def test_centring_verifies_arrival_instead_of_assuming(self):
        """Observed on ME3B V1: centring reported ok=True while the stage was
        still ~0.7 mm away. Every probe would then be measured about the WRONG
        origin — exactly what centring exists to prevent."""
        c = _ctrl(with_safe_travel=False)
        c.get_xy_position = lambda cached=False: (57000.0, 37000.0, 0.0)  # 1.2 mm off
        r = AC.center_stage(c, safe_z_mm=None)
        self.assertFalse(r["ok"])
        self.assertIn("did not land", r["reason"])

    def test_centring_accepts_arrival_within_tolerance(self):
        c = _ctrl(with_safe_travel=False)
        cx, cy = AC.envelope_center_um(c)
        c.get_xy_position = lambda cached=False: (cx + 20.0, cy - 15.0, 0.0)
        self.assertTrue(AC.center_stage(c, safe_z_mm=None)["ok"])

    def test_centring_fails_when_the_position_cannot_be_read(self):
        """A corrupted read must not pass as a successful centring."""
        c = _ctrl(with_safe_travel=False)
        c.get_xy_position = lambda cached=False: (None, None, None)
        r = AC.center_stage(c, safe_z_mm=None)
        self.assertFalse(r["ok"])
        self.assertIn("could not read", r["reason"])

    def test_centring_reports_an_unknown_envelope(self):
        c = SimpleNamespace(safety_limits=SimpleNamespace(),
                            default_plate_center_um=lambda: None)
        r = AC.center_stage(c)
        self.assertFalse(r["ok"])
        self.assertIn("centre", r["reason"])


class TestPlanAndStore(unittest.TestCase):
    def test_the_plan_is_ordered_by_dependency(self):
        keys = [s.key for s in AC.CALIBRATION_STEPS]
        seen = set()
        for s in AC.CALIBRATION_STEPS:
            for need in s.needs:
                self.assertIn(need, seen,
                              msg=f"{s.key} needs {need} but it runs later")
            seen.add(s.key)

    def test_derive_and_accept_do_not_move_the_stage(self):
        by = {s.key: s for s in AC.CALIBRATION_STEPS}
        self.assertFalse(by["derive"].moves_stage)
        self.assertFalse(by["accept"].moves_stage)

    def test_plan_summary_is_human_readable(self):
        lines = AC.plan_summary(AC.CalibrationPolicy())
        self.assertEqual(len(lines), len(AC.CALIBRATION_STEPS))
        self.assertTrue(all("—" in ln for ln in lines))

    def test_round_trip_through_the_store(self):
        from tests.support.store_fixture import use_temp_store
        store = use_temp_store(self)
        store.set_control_loop_ms(31.75)
        store.set_velocity_dead_time_s(0.0672, n=5, spread_s=0.000546,
                                       tau_s=0.0281)
        store.set_xy_max_speed_um_s(5945.611449038091)

        m = AC.measured_from_store(store)
        self.assertTrue(m.is_complete())
        self.assertAlmostEqual(m.dead_time_s, 0.0672, places=6)
        self.assertAlmostEqual(m.tau_s, 0.0281, places=6)

        d = AC.derive_settings(m, AC.CalibrationPolicy(target_speed_mm_s=5.0))
        AC.apply_to_store(store, d)
        v = store.get_mode_params("velocity")
        self.assertAlmostEqual(v["lookahead_mm"], d.values["lookahead_mm"],
                               places=6)
        self.assertAlmostEqual(v["pid_kp"], d.values["pid_kp"], places=6)
        self.assertEqual(store.get_mode_params("confirm")["settle_tol_um"],
                         d.values["settle_tol_um"])


class TestStampReachesEveryPrintPath(unittest.TestCase):
    """Before this, ONLY Quick Print stamped the tuning — so a bench session had
    no effect whatsoever on a Full Print."""

    def _store(self):
        from tests.support.store_fixture import use_temp_store
        store = use_temp_store(self)
        store.set_control_loop_ms(31.75)
        store.set_velocity_dead_time_s(0.0672, n=5)
        store.set_xy_max_speed_um_s(5945.6)
        store.set_mode_params("velocity", {"lookahead_mm": 0.99, "pid_kp": 5.24,
                                           "hold_speed": 1.0})
        return store

    def test_stamps_every_field_a_follower_reads(self):
        from SupportClasses.PrintManager import PrintSettings
        store = self._store()
        s = AC.stamp_print_settings(PrintSettings(), store)
        self.assertAlmostEqual(s.vel_lookahead_mm, 0.99)
        self.assertAlmostEqual(s.vel_pid_kp, 5.24)
        self.assertAlmostEqual(s.control_loop_ms, 31.75)
        self.assertAlmostEqual(s.xy_max_speed_um_s, 5945.6)
        self.assertEqual(s.vel_tuning["hold_speed"], 1.0)
        self.assertAlmostEqual(s.vel_tuning["dead_time_s"], 0.0672, places=6)

    def test_measured_dead_time_beats_the_by_phase_settle_time(self):
        store = self._store()
        from SupportClasses.PrintManager import PrintSettings
        s = AC.stamp_print_settings(PrintSettings(), store)
        self.assertIn("dead_time_s", s.vel_tuning)
        self.assertAlmostEqual(s.vel_tuning["dead_time_s"], 0.0672, places=6)

    def test_is_best_effort_on_a_broken_store(self):
        from SupportClasses.PrintManager import PrintSettings
        s = AC.stamp_print_settings(PrintSettings(), object())
        self.assertEqual(s.vel_tuning, {})           # untouched legacy defaults


if __name__ == "__main__":
    unittest.main()
