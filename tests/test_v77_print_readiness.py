# -*- coding: utf-8 -*-
"""v7.7 — the pure readiness model.

Two invariants matter more than any individual check and are asserted hardest:

1. **Unknown data never blocks.** The model runs on every status tick against a
   possibly half-configured machine; a readiness panel that disables Print
   because a getter returned ``None`` is worse than no panel.
2. **``can_print()`` blocks on exactly the pre-existing gates** (XY+ZP, object,
   plate, well) plus the one physically-impossible case. Anything else must be a
   visible warning the operator can proceed past, so this cannot start refusing
   prints that previously ran.
"""

import unittest

from SupportClasses.PrintReadiness import (BLOCK, INFO, OK, WARN,
                                           ReadinessContext, evaluate)


def _ok_ctx(**kw) -> ReadinessContext:
    """A context whose structural preconditions are all satisfied."""
    base = dict(xy_connected=True, zp_connected=True, object_selected=True,
                object_label="Star", plate_available=True, well="B3")
    base.update(kw)
    return ReadinessContext(**base)


class TestStructuralBlocks(unittest.TestCase):
    def test_empty_context_blocks_only_the_five_preconditions(self):
        r = evaluate(ReadinessContext())
        self.assertEqual({c.id for c in r.blocking()},
                         {"xy", "zp", "object", "plate", "well"})
        self.assertFalse(r.can_print())

    def test_satisfied_preconditions_can_print(self):
        r = evaluate(_ok_ctx())
        self.assertEqual(r.blocking(), [])
        self.assertTrue(r.can_print())

    def test_headline_names_the_first_blocker(self):
        r = evaluate(ReadinessContext(zp_connected=True))
        self.assertIn("XY stage", r.headline())

    def test_headline_counts_warnings_when_ready(self):
        r = evaluate(_ok_ctx(safe_z=None))
        self.assertTrue(r.can_print())
        self.assertIn("warning", r.headline())

    def test_all_clear_headline(self):
        r = evaluate(_ok_ctx(safe_z=44.0, plate_bottom_z=21.13,
                             needle_configured=True, char_complete=True))
        self.assertEqual(r.headline(), "Ready.")


class TestUnknownDataNeverBlocks(unittest.TestCase):
    def test_every_optional_field_left_none_produces_no_extra_block(self):
        r = evaluate(_ok_ctx())
        self.assertTrue(r.can_print())
        # and nothing invented a number out of None
        for c in r.checks:
            self.assertNotIn("None", c.detail)

    def test_bool_and_junk_are_not_treated_as_numbers(self):
        # A bare MagicMock has __float__ == 1.0; bools are ints. Neither may
        # fabricate a limit.
        r = evaluate(_ok_ctx(object_radius_mm=True, well_radius_mm=True))
        self.assertIsNone(r.get("fit"))

    def test_grouping_is_stable_and_covers_every_check(self):
        r = evaluate(_ok_ctx())
        grouped = sum(len(rows) for _g, rows in r.by_group())
        self.assertEqual(grouped, len(r.checks))


class TestInkBoreBlock(unittest.TestCase):
    """The single hard block beyond the pre-existing gates."""

    def test_particles_larger_than_the_bore_block(self):
        r = evaluate(_ok_ctx(ink_particle_um=300.0, bore_id_um=210.0))
        clog = r.get("clog")
        self.assertEqual(clog.state, BLOCK)
        self.assertFalse(r.can_print())
        self.assertIn("cannot pass", clog.detail)

    def test_particles_that_fit_do_not_block(self):
        r = evaluate(_ok_ctx(ink_particle_um=20.0, bore_id_um=210.0))
        self.assertEqual(r.get("clog").state, OK)
        self.assertTrue(r.can_print())
        self.assertIn("10.5", r.get("clog").detail)      # clearance ratio

    def test_unset_particle_size_never_blocks(self):
        for particle in (None, 0.0, False):
            r = evaluate(_ok_ctx(ink_particle_um=particle, bore_id_um=210.0))
            self.assertTrue(r.can_print(),
                            f"particle={particle!r} must not block work")

    def test_unknown_bore_never_blocks(self):
        r = evaluate(_ok_ctx(ink_particle_um=300.0, bore_id_um=None))
        self.assertTrue(r.can_print())

    def test_inkspec_detail_is_surfaced_as_a_warning_not_a_block(self):
        r = evaluate(_ok_ctx(
            bore_id_um=410.0, ink_particle_um=230.0,
            clog={"status": "risk_clogging", "severity": "warn",
                  "message": "22G ID is only 1.8x your 230 um spheroids"}))
        c = r.get("clog")
        self.assertEqual(c.state, WARN)
        self.assertIn("1.8x", c.detail)
        self.assertTrue(r.can_print())


class TestBioprintingAdvisories(unittest.TestCase):
    def test_shear_above_the_cell_limit_warns(self):
        r = evaluate(_ok_ctx(wall_shear_pa=8.4, shear_limit_pa=5.0))
        c = r.get("shear")
        self.assertEqual(c.state, WARN)
        self.assertIn("cell-viability", c.detail)
        self.assertTrue(r.can_print(), "shear is advisory, never blocking")

    def test_shear_below_the_limit_is_ok(self):
        r = evaluate(_ok_ctx(wall_shear_pa=1.1, shear_limit_pa=5.0))
        self.assertEqual(r.get("shear").state, OK)

    def test_reference_fluid_ceiling_is_called_out_when_the_ink_differs(self):
        r = evaluate(_ok_ctx(flow_ceiling_ink_uL_s=0.10,
                             flow_ceiling_ref_uL_s=0.40))
        c = r.get("visc")
        self.assertEqual(c.state, WARN)
        self.assertIn("reference fluid", c.detail)
        self.assertIn("75 % lower", c.detail)

    def test_similar_viscosity_is_not_flagged(self):
        r = evaluate(_ok_ctx(flow_ceiling_ink_uL_s=0.38,
                             flow_ceiling_ref_uL_s=0.40))
        self.assertEqual(r.get("visc").state, OK)


class TestSpeedAndResolution(unittest.TestCase):
    def test_flow_ceiling_is_named_when_it_binds(self):
        r = evaluate(_ok_ctx(requested_speed_mm_s=5.0, resolved_speed_mm_s=3.14,
                             stage_max_mm_s=5.95, flow_ceiling_speed_mm_s=3.14))
        c = r.get("speed")
        self.assertEqual(c.state, WARN)
        self.assertIn("flow ceiling", c.detail)

    def test_stage_max_is_named_when_it_binds(self):
        r = evaluate(_ok_ctx(requested_speed_mm_s=12.0, resolved_speed_mm_s=5.95,
                             stage_max_mm_s=5.95, flow_ceiling_speed_mm_s=9.0))
        self.assertIn("measured stage max", r.get("speed").detail)

    def test_headroom_is_reported_when_nothing_binds(self):
        """The gap the old surface left: a normal run said nothing at all about
        how close to the limits it was."""
        r = evaluate(_ok_ctx(requested_speed_mm_s=3.0, resolved_speed_mm_s=3.0,
                             stage_max_mm_s=6.0, flow_ceiling_speed_mm_s=9.0))
        c = r.get("speed")
        self.assertEqual(c.state, OK)
        self.assertIn("50 % below", c.detail)

    def test_resolution_below_the_machine_floor_warns(self):
        r = evaluate(_ok_ctx(resolution_um=10.0, resolution_floor_um=23.0))
        c = r.get("resolution")
        self.assertEqual(c.state, WARN)
        self.assertIn("23 µm floor", c.detail)

    def test_resolution_above_the_floor_reports_the_floor(self):
        r = evaluate(_ok_ctx(resolution_um=30.0, resolution_floor_um=23.0))
        c = r.get("resolution")
        self.assertEqual(c.state, OK)
        self.assertIn("23", c.detail)

    def test_time_estimate_prices_the_accuracy_trade(self):
        r = evaluate(_ok_ctx(est_time_s=42.0, n_corner_stops=12))
        c = r.get("time")
        self.assertEqual(c.state, INFO)
        self.assertIn("12 corner stops", c.detail)
        self.assertIn("costs time", c.detail)


class TestCalibrationAndMachine(unittest.TestCase):
    def test_a_synthesised_travel_height_is_disclosed(self):
        r = evaluate(_ok_ctx(print_z_zref=0.2, travel_z_zref=10.2,
                             travel_z_synthesised=True))
        c = r.get("heights")
        self.assertEqual(c.state, WARN)
        self.assertIn("DERIVED", c.detail)

    def test_geometric_well_centre_is_disclosed(self):
        r = evaluate(_ok_ctx(well_center_calibrated=False))
        self.assertEqual(r.get("well_cal").state, WARN)
        self.assertIn("GEOMETRY", r.get("well_cal").detail)

    def test_stale_calibration_warns_on_travel_or_time(self):
        far = evaluate(_ok_ctx(xy_travel_since_cal_mm=4000.0,
                               xy_recal_travel_mm=1000.0))
        self.assertEqual(far.get("stale").state, WARN)
        old = evaluate(_ok_ctx(hours_since_xy_cal=500.0,
                               recal_interval_hours=168.0))
        self.assertEqual(old.get("stale").state, WARN)
        fresh = evaluate(_ok_ctx(xy_travel_since_cal_mm=10.0,
                                 xy_recal_travel_mm=1000.0,
                                 hours_since_xy_cal=2.0,
                                 recal_interval_hours=168.0))
        self.assertEqual(fresh.get("stale").state, OK)

    def test_uncharacterised_machine_lists_what_is_missing(self):
        r = evaluate(_ok_ctx(char_complete=False,
                             char_missing=("dead time", "top speed")))
        self.assertIn("dead time, top speed", r.get("char").detail)

    def test_dead_time_provenance_is_reported(self):
        measured = evaluate(_ok_ctx(dead_time_s=0.067,
                                    dead_time_source="measured"))
        self.assertEqual(measured.get("dead_time").state, OK)
        inferred = evaluate(_ok_ctx(dead_time_s=0.287,
                                    dead_time_source="phase_lag"))
        self.assertEqual(inferred.get("dead_time").state, WARN)
        self.assertIn("not measured", inferred.get("dead_time").detail)
        none = evaluate(_ok_ctx(dead_time_s=0.1,
                                dead_time_source="unmeasured"))
        self.assertEqual(none.get("dead_time").state, WARN)

    def test_auto_selected_mode_explains_itself(self):
        r = evaluate(_ok_ctx(motion_mode="auto", resolved_motion_mode="velocity",
                             char_complete=True))
        self.assertIn("auto-selected", r.get("mode").detail)
        self.assertIn("characterised", r.get("mode").detail)


class TestGeometryFit(unittest.TestCase):
    def test_object_larger_than_the_well_warns(self):
        r = evaluate(_ok_ctx(object_radius_mm=4.0, well_radius_mm=3.0))
        c = r.get("fit")
        self.assertEqual(c.state, WARN)
        self.assertIn("exceeds", c.detail)
        self.assertTrue(r.can_print(), "advisory, not a block")

    def test_object_inside_the_well_reports_the_margin(self):
        r = evaluate(_ok_ctx(object_radius_mm=1.5, well_radius_mm=3.0))
        c = r.get("fit")
        self.assertEqual(c.state, OK)
        self.assertIn("50 % of the radius", c.detail)

    def test_no_geometry_no_check(self):
        self.assertIsNone(evaluate(_ok_ctx()).get("fit"))


class TestFluidics(unittest.TestCase):
    def test_headroom_is_shown_when_the_budget_passes(self):
        """Previously the budget check returned silently on success, so 0.2 µL
        and 200 µL of margin looked identical."""
        r = evaluate(_ok_ctx(syringe_capacity_uL=250.0, budget_span_uL=12.0,
                             budget_peak_fill_uL=100.0, budget_min_fill_uL=88.0))
        c = r.get("budget")
        self.assertEqual(c.state, OK)
        self.assertIn("spare", c.detail)

    def test_overflow_warns_with_the_amount(self):
        r = evaluate(_ok_ctx(syringe_capacity_uL=250.0, budget_span_uL=300.0,
                             budget_overflow_uL=50.0))
        c = r.get("budget")
        self.assertEqual(c.state, WARN)
        self.assertIn("over-fill by 50.00", c.detail)

    def test_pickup_breakdown_is_itemised(self):
        r = evaluate(_ok_ctx(pickup_uL=1.234, pickup_dispense_uL=0.9,
                             pickup_prime_uL=0.1, pickup_dead_volume_uL=0.2,
                             pickup_padding_uL=0.034))
        c = r.get("pickup")
        for part in ("path", "prime", "bore reserve", "your padding"):
            self.assertIn(part, c.detail)

    def test_uncalibrated_plunger_warns(self):
        r = evaluate(_ok_ctx(pump_plunger_calibrated=False, pump="P2"))
        self.assertEqual(r.get("plunger").state, WARN)
        self.assertTrue(r.can_print())


class TestRoutine(unittest.TestCase):
    def test_prep_missing_wells_warn(self):
        r = evaluate(_ok_ctx(prep_enabled=True,
                             prep_missing=("waste", "oil")))
        self.assertEqual(r.get("prep").state, WARN)
        self.assertIn("waste, oil", r.get("prep").detail)

    def test_prep_off_is_informational(self):
        r = evaluate(_ok_ctx(prep_enabled=False))
        self.assertEqual(r.get("prep").state, INFO)

    def test_cleanup_states(self):
        on = evaluate(_ok_ctx(cleanup_enabled=True))
        self.assertEqual(on.get("cleanup").state, OK)
        bad = evaluate(_ok_ctx(cleanup_enabled=True, cleanup_missing=("wash",)))
        self.assertEqual(bad.get("cleanup").state, WARN)


if __name__ == "__main__":
    unittest.main()
