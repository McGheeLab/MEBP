"""test_v79_cell_removal_readiness.py — the readiness model (Stage 3).

The Cell Targeting page used to keep every refusal as a literal string inside
``_on_start``, reachable only by clicking a button that was already disabled —
while the status line said "Idle.", the Setup tab printed a green ✓ beside
configurations Start would refuse, and the most consequential advisory of all (an
unmeasured bore mount offset) was appended to the status label AFTER the executor
thread had started.

These tests pin the two governing rules inherited from ``PrintReadiness``:

1. unknown data is never blocking;
2. ``can_start()`` mirrors the pre-existing gates, plus exactly two promotions —
   an unmeasured mount offset and a bore wired to a pump the assembly does not
   declare. Both were advisory-only and both silently destroy a run's material.
"""

from __future__ import annotations

import unittest

from SupportClasses.CellRemovalReadiness import (
    BLOCK, INFO, OK, WARN, BoreView, CellRemovalContext, evaluate,
)
from SupportClasses.PrintReadiness import GROUPS as PRINT_GROUPS


def _ready_ctx(**over) -> CellRemovalContext:
    """A context that is fully ready — every test perturbs ONE thing."""
    ctx = CellRemovalContext(
        xy_connected=True, zp_connected=True,
        needle_configured=True, bore_count=1,
        bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                        declared_pump="P1", offset_measured=True)],
        n_picks=3, n_places=3,
        release_reagent="Trypsin", release_well_calibrated=True,
        column_volume_uL=0.0134,
        safe_z_mm=40.0, plate_bottom_calibrated=True,
        removal_clearance_mm=0.10, place_clearance_mm=0.50,
    )
    for k, v in over.items():
        setattr(ctx, k, v)
    return ctx


class TestTheBaselineIsReady(unittest.TestCase):
    def test_a_fully_configured_single_bore_run_can_start(self):
        r = evaluate(_ready_ctx())
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        self.assertEqual(r.headline(), "Ready.")

    def test_every_group_it_emits_is_registered_for_display_order(self):
        """An unregistered group falls into 'Other' and reads as a bug."""
        r = evaluate(_ready_ctx())
        for c in r.checks:
            self.assertIn(c.group, PRINT_GROUPS,
                          f"group {c.group!r} is not in PrintReadiness.GROUPS")


class TestUnknownDataNeverBlocks(unittest.TestCase):
    """Rule 1. A model that can disable Start forever because a getter returned
    None is worse than no model."""

    def test_an_empty_context_does_not_crash(self):
        r = evaluate(CellRemovalContext())
        self.assertIsNotNone(r.headline())

    def test_unknown_offset_measurement_does_not_block(self):
        r = evaluate(_ready_ctx(
            bore_count=2,
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True),
                   BoreView(index=1, role="push_reagent", pump_id="P2",
                            declared_pump="P2", offset_measured=None)],
            dosing_enabled=True, dosing_reagent="Trypsin",
            dosing_well_calibrated=True, dose_volume_uL=0.004))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")

    def test_unknown_well_calibration_does_not_block(self):
        r = evaluate(_ready_ctx(release_well_calibrated=None))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")

    def test_unknown_volumes_do_not_block(self):
        r = evaluate(_ready_ctx(column_volume_uL=None, pull_volume_uL=None))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")

    def test_an_unknown_flow_ceiling_does_not_warn(self):
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True,
                            dose_rate_uL_s=5.0, flow_ceiling_uL_s=None)]))
        self.assertIsNone(r.get("flow_0"))


class TestThePreExistingGatesStillBlock(unittest.TestCase):
    """Rule 2, first half — nothing that used to be allowed becomes blocked."""

    def test_no_zp_board(self):
        r = evaluate(_ready_ctx(zp_connected=False))
        self.assertFalse(r.can_start())
        self.assertIn("Z + pump", r.headline())

    def test_no_safe_z(self):
        self.assertFalse(evaluate(_ready_ctx(safe_z_mm=None)).can_start())

    def test_uncalibrated_plate_bottom(self):
        self.assertFalse(
            evaluate(_ready_ctx(plate_bottom_calibrated=False)).can_start())

    def test_no_targets(self):
        r = evaluate(_ready_ctx(n_picks=0, n_places=0))
        self.assertFalse(r.can_start())
        # And it must say WHERE to go — the single most missing instruction.
        fix = r.get("targets").fix
        # Name a surface that EXISTS and the action to take there. Asserting on
        # the literal tab name is what broke when the tabs were renamed, so pin
        # the two things that must stay true instead.
        self.assertIn("Plan", fix)
        self.assertIn("click", fix.lower())

    def test_unbalanced_targets(self):
        r = evaluate(_ready_ctx(n_picks=3, n_places=2))
        self.assertFalse(r.can_start())
        self.assertIn("paired placement", r.get("targets").detail)

    def test_no_release_reagent_when_no_dosing_bore(self):
        self.assertFalse(evaluate(_ready_ctx(release_reagent=None)).can_start())

    def test_uncalibrated_release_well(self):
        self.assertFalse(
            evaluate(_ready_ctx(release_well_calibrated=False)).can_start())

    def test_unshifted_mosaic_targets(self):
        r = evaluate(_ready_ctx(n_unshifted_mosaic=2,
                                mosaic_error_bound_um=337.0))
        self.assertFalse(r.can_start())
        self.assertIn("337", r.get("mosaic_shift").detail)

    def test_missing_service_wells_when_prep_is_on(self):
        r = evaluate(_ready_ctx(prep_enabled=True,
                                missing_service_wells=["oil", "wash"]))
        self.assertFalse(r.can_start())

    def test_missing_service_wells_are_ignored_when_prep_is_off(self):
        r = evaluate(_ready_ctx(prep_enabled=False, clean_enabled=False,
                                missing_service_wells=["oil", "wash"]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")

    def test_a_zero_column_volume_blocks_because_nothing_is_collected(self):
        r = evaluate(_ready_ctx(column_volume_uL=0.0))
        self.assertFalse(r.can_start())
        self.assertIn("nothing would be collected", r.get("column_volume").detail)

    def test_a_sub_microstep_column_warns_about_the_PULL_too(self):
        """A 30 µm bore at the historic 0.100 mm default column depth is ~71 pL —
        14× under one pump microstep — and nothing warned. The column also sizes
        the extraction pull, so the cell is never collected either."""
        r = evaluate(_ready_ctx(column_volume_uL=0.000071))
        self.assertTrue(r.can_start(), "advisory, not a block")
        c = r.get("column_volume")
        self.assertEqual(c.state, WARN)
        self.assertIn("microstep", c.detail)
        self.assertIn("pull", c.detail, "the extraction consequence must be named")
        self.assertIn("0.071 nL", c.detail, "must not print as 0.0000 µL")
        self.assertTrue(c.fix, "a warning the operator can act on needs a fix")

    def test_a_meaningful_column_volume_is_plain_OK(self):
        r = evaluate(_ready_ctx(column_volume_uL=0.0134))
        c = r.get("column_volume")
        self.assertEqual(c.state, OK)
        self.assertNotIn("microstep", c.detail)

    def test_a_stage_busy_reason_blocks(self):
        r = evaluate(_ready_ctx(stage_busy_reason="A mosaic scan is running."))
        self.assertFalse(r.can_start())

    def test_no_needle(self):
        self.assertFalse(evaluate(_ready_ctx(needle_configured=False)).can_start())


class TestThePromotions(unittest.TestCase):
    """Rule 2, second half — the two advisories that silently destroy a run."""

    def _two_bore(self, **bore1):
        kw = dict(index=1, role="push_reagent", pump_id="P2",
                  declared_pump="P2", offset_measured=True,
                  offset_um=(320.0, -140.0))
        kw.update(bore1)
        return _ready_ctx(
            bore_count=2,
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True),
                   BoreView(**kw)],
            dosing_enabled=True, dosing_reagent="Trypsin",
            dosing_well_calibrated=True, dose_volume_uL=0.004)

    def test_an_unmeasured_offset_now_BLOCKS(self):
        r = evaluate(self._two_bore(offset_measured=False,
                                    offset_um=(0.0, 0.0)))
        self.assertFalse(r.can_start())
        c = r.get("bore_offsets")
        self.assertEqual(c.state, BLOCK)
        self.assertIn("without collecting any", c.detail,
                      "must state the consequence, not just the condition")
        self.assertIn("Needle Location", c.fix)

    def test_a_measured_offset_is_reported_with_its_numbers(self):
        r = evaluate(self._two_bore())
        c = r.get("bore_offsets")
        self.assertEqual(c.state, OK)
        self.assertIn("+320", c.detail)
        self.assertIn("-140", c.detail)

    def test_the_datum_bore_never_needs_a_measured_offset(self):
        """Bore 1 IS the datum — its offset is (0,0) by definition."""
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=False)]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")

    def test_an_IDLE_bore_with_no_offset_does_not_block(self):
        """Only bores the run actually drives matter."""
        r = evaluate(_ready_ctx(
            bore_count=2,
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True),
                   BoreView(index=1, role="idle", pump_id="P2",
                            declared_pump="P2", offset_measured=False)]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")

    def test_a_rewired_pump_now_BLOCKS(self):
        r = evaluate(self._two_bore(pump_id="P1", declared_pump="P3"))
        self.assertFalse(r.can_start())
        c = r.get("bore_pumps")
        self.assertEqual(c.state, BLOCK)
        self.assertIn("not plumbed", c.detail)

    def test_pump_comparison_is_case_insensitive(self):
        r = evaluate(self._two_bore(pump_id="p2", declared_pump="P2"))
        self.assertIsNone(r.get("bore_pumps"))

    def test_an_undeclared_pump_does_not_block(self):
        """The assembly declaring nothing is unknown data, not a conflict."""
        r = evaluate(self._two_bore(declared_pump=None))
        self.assertIsNone(r.get("bore_pumps"))

    def test_an_active_bore_with_NO_pump_blocks(self):
        r = evaluate(self._two_bore(pump_id=None, declared_pump=None))
        self.assertFalse(r.can_start())
        self.assertEqual(r.get("bore_unassigned").state, BLOCK)


class TestDosingReplacesTheReleasePush(unittest.TestCase):
    """The operator's decision: a dosing bore REPLACES the aspirating bore's
    push. The GUI must stop demanding a cell-release reagent, which is what used
    to force the double-dose configuration."""

    def _dosing(self, **over):
        kw = dict(
            bore_count=2,
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True),
                   BoreView(index=1, role="push_reagent", pump_id="P2",
                            declared_pump="P2", offset_measured=True,
                            offset_um=(320.0, -140.0))],
            dosing_enabled=True, dosing_reagent="Trypsin",
            dosing_well_calibrated=True, dose_volume_uL=0.004,
            release_reagent=None, release_well_calibrated=None)
        kw.update(over)
        return _ready_ctx(**kw)

    def test_no_release_reagent_needed_when_a_dosing_bore_is_armed(self):
        r = evaluate(self._dosing())
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        c = r.get("release_reagent")
        self.assertEqual(c.state, INFO)
        self.assertIn("only pulls", c.detail)

    def test_a_missing_dosing_reagent_blocks(self):
        self.assertFalse(evaluate(self._dosing(dosing_reagent=None)).can_start())

    def test_an_uncalibrated_dosing_well_blocks(self):
        self.assertFalse(
            evaluate(self._dosing(dosing_well_calibrated=False)).can_start())

    def test_a_zero_dose_volume_blocks(self):
        r = evaluate(self._dosing(dose_volume_uL=0.0))
        self.assertFalse(r.can_start())
        self.assertIn("Needle", r.get("dose_volume").fix)

    def test_a_sub_microstep_dose_warns_rather_than_blocking(self):
        r = evaluate(self._dosing(dose_volume_uL=0.00007))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        c = r.get("dose_volume")
        self.assertEqual(c.state, WARN)
        self.assertIn("microstep", c.detail)
        # And it must be readable at that scale, not printed as 0.0000.
        self.assertIn("nL", c.detail)

    def test_the_additive_timing_total_is_stated(self):
        r = evaluate(self._dosing(lead_time_s=10.0, incubation_s=60.0))
        c = r.get("timing")
        self.assertIn("70.0 s", c.detail)
        self.assertIn("10.0 s lead", c.detail)
        self.assertIn("60.0 s incubation", c.detail)


class TestAdvisoriesStayAdvisory(unittest.TestCase):
    def test_a_clamped_DOSE_flow_warns_and_names_the_lead_time_consequence(self):
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="push_reagent", pump_id="P1",
                            declared_pump="P1", offset_measured=True,
                            dose_rate_uL_s=5.0, flow_ceiling_uL_s=0.05)]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        c = r.get("flow_0")
        self.assertEqual(c.state, WARN)
        self.assertIn("100×", c.detail)
        self.assertIn("lead time", c.detail)
        self.assertIn("dose flow", c.label)

    def test_a_clamped_flow_on_a_NON_dosing_bore_claims_no_lead_time(self):
        """The page fills `dose_rate_uL_s` from every active program's rate, so
        this check used to call an aspirating bore's pull rate a "dose flow" and
        cite a lead time that has nothing to do with it."""
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True,
                            dose_rate_uL_s=5.0, flow_ceiling_uL_s=0.05)]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        c = r.get("flow_0")
        self.assertEqual(c.state, WARN)
        self.assertIn("100×", c.detail)
        self.assertNotIn("dose", c.label.lower())
        self.assertNotIn("lead time", c.detail)

    def test_a_dose_over_the_bore_holdup_warns(self):
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True,
                            dose_volume_uL=10.0, internal_volume_uL=0.1)]))
        self.assertTrue(r.can_start())
        c = r.get("holdup_0")
        self.assertEqual(c.state, WARN)
        self.assertIn("volume balance", c.detail)

    def test_no_aspirating_bore_warns_rather_than_blocking(self):
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="idle", pump_id="P1",
                            declared_pump="P1", offset_measured=True)]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        self.assertEqual(r.get("aspirate_role").state, WARN)

    def test_duplicate_roles_warn(self):
        r = evaluate(_ready_ctx(
            bore_count=2,
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True),
                   BoreView(index=1, role="aspirate_target", pump_id="P2",
                            declared_pump="P2", offset_measured=True,
                            offset_um=(320.0, 0.0))]))
        self.assertTrue(r.can_start(), f"blocked by {r.blocking()}")
        self.assertEqual(r.get("aspirate_role").state, WARN)

    def test_a_missing_target_type_warns_and_keeps_the_assignment(self):
        r = evaluate(_ready_ctx(
            bores=[BoreView(index=0, role="aspirate_target", pump_id="P1",
                            declared_pump="P1", offset_measured=True,
                            target_type_id="gfp-cells",
                            target_type_missing=True)]))
        self.assertTrue(r.can_start())
        c = r.get("tt_0")
        self.assertEqual(c.state, WARN)
        self.assertIn("kept", c.detail)

    def test_a_zero_removal_height_warns(self):
        r = evaluate(_ready_ctx(removal_clearance_mm=0.0))
        self.assertTrue(r.can_start())
        self.assertIn("touch the glass", r.get("removal_z").detail)


class TestHeadlineNamesTheFirstProblem(unittest.TestCase):
    def test_the_headline_leads_with_a_blocker(self):
        h = evaluate(_ready_ctx(n_picks=0, n_places=0)).headline()
        self.assertTrue(h.startswith("Not ready"), h)
        self.assertIn("Removal targets", h)

    def test_the_headline_counts_warnings_when_ready(self):
        h = evaluate(_ready_ctx(removal_clearance_mm=0.0)).headline()
        self.assertIn("Ready, with 1 warning", h)

    def test_every_blocking_check_names_a_fix_or_states_the_condition(self):
        """A blocker the operator cannot act on is a dead end."""
        for ctx in (_ready_ctx(n_picks=0), _ready_ctx(safe_z_mm=None),
                    _ready_ctx(zp_connected=False),
                    _ready_ctx(plate_bottom_calibrated=False),
                    _ready_ctx(release_reagent=None),
                    _ready_ctx(column_volume_uL=0.0)):
            for c in evaluate(ctx).blocking():
                self.assertTrue(c.fix or c.detail,
                                f"{c.id} blocks with no fix and no detail")


if __name__ == "__main__":
    unittest.main()
