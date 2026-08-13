"""
tests/test_v76_pulled_capillary_needle.py — v7.6 pulled glass capillary needles.

A needle may now be TWO flow stages: a bulk barrel (D1, L1) feeding a pulled tip
(D2, L2). The overriding constraint is that a straight hypodermic cannula must be
byte-identical in serialization and float-identical in physics to pre-v7.6 — the
capillary is purely additive. Most of this file exists to hold that line.
"""

from __future__ import annotations

import ast
import json
import math
import os
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PhysicalModels import (          # noqa: E402
    NeedleSpec, InkSpec, BoreProfile, FlowSegment,
    NEEDLE_TYPE_HYPODERMIC, NEEDLE_TYPE_CAPILLARY,
    TIP_PROFILE_CYLINDER, TIP_PROFILE_CONE,
    needle_orifice_area_mm2, needle_orifice_id_um, needle_bore_profile,
    needle_flow_segments, default_fallback_needle,
)
from SupportClasses.FlowPhysics import (             # noqa: E402
    max_safe_flow_rate_uL_s, DEFAULT_PRESSURE_LIMIT_PA,
    limiting_flow_segment, classify_granular_regime, GranularRegime,
)
from SupportClasses.HardwareConfig import HardwareConfig  # noqa: E402
from SupportClasses.CalibrationSnapshotStore import (  # noqa: E402
    CalibrationSnapshotStore,
)
from SupportClasses.NeedleTypeStore import (         # noqa: E402
    NeedleType, NeedleTypeStore, safe_id,
)

REPO = Path(__file__).resolve().parent.parent

# The exact on-disk shape of a saved needle block. Note the JSON string key in
# channel_pump_map — an int key survives a JSON round-trip as "1".
LEGACY_PAYLOAD = {
    "gauge": 27, "od_um": 413, "id_um": 210, "wall_um": 102,
    "length_inches": 2.0, "num_channels": 1, "channel_pump_map": {"1": "P1"},
}

LEGACY_KEYS = {"gauge", "od_um", "id_um", "wall_um", "length_inches",
               "num_channels", "channel_pump_map"}


def _capillary(**kw) -> NeedleSpec:
    """A representative pulled capillary: 580 µm bore blank, 30 µm tip."""
    base = dict(gauge=None, od_um=1000.0, id_um=580.0, wall_um=210.0,
                length_inches=100.0 / 25.4, needle_type=NEEDLE_TYPE_CAPILLARY,
                tip_id_um=30.0, tip_length_mm=5.0)
    base.update(kw)
    return NeedleSpec(**base)


# ════════════════════════════════════════════════════════════════════
#  A. Serialization byte-identity for a legacy straight needle
# ════════════════════════════════════════════════════════════════════

class TestLegacySerializationIdentity(unittest.TestCase):
    """The regression gate: adding two-stage support must not perturb a single
    byte of an existing saved needle."""

    def test_legacy_needle_round_trip_is_byte_identical(self):
        spec = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        self.assertEqual(json.dumps(spec.to_dict()), json.dumps(LEGACY_PAYLOAD))

    def test_hypodermic_to_dict_has_exactly_the_seven_legacy_keys(self):
        spec = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        self.assertEqual(set(spec.to_dict()), LEGACY_KEYS)

    def test_every_on_disk_setup_needle_round_trips(self):
        # v7.17.x: hardware setup files are SHARED config, so they live in
        # config/hardware/ME3B_general/ — globbing the old flat root made this
        # iterate zero files and pass vacuously.
        from SupportClasses.MachineConfig import shared_config_dir
        checked = 0
        for path in sorted(shared_config_dir().glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            needle = data.get("needle") if isinstance(data, dict) else None
            if not needle:
                continue
            checked += 1
            self.assertEqual(
                json.dumps(NeedleSpec.from_dict(needle).to_dict()),
                json.dumps(needle),
                f"{path.name} needle block did not round-trip byte-identically")
        if checked == 0:
            self.skipTest("no on-disk setup files carry a needle block")

    def test_hardware_config_round_trip_preserves_straight_needle(self):
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        again = HardwareConfig.from_dict(cfg.to_dict())
        self.assertEqual(json.dumps(again.needle.to_dict()),
                         json.dumps(LEGACY_PAYLOAD))

    def test_from_dict_ignores_unknown_keys(self):
        # Pre-v7.6 this was cls(**data) and raised TypeError, which a broad
        # except in gui/app.py turned into "lost the entire hardware config".
        spec = NeedleSpec.from_dict({**LEGACY_PAYLOAD, "future_key": 1})
        self.assertEqual(spec, NeedleSpec.from_dict(LEGACY_PAYLOAD))

    def test_capillary_round_trip(self):
        cap = _capillary(needle_type_id="borosilicate-1b100f-30um")
        d = cap.to_dict()
        for key in ("needle_type", "tip_id_um", "tip_length_mm", "needle_type_id"):
            self.assertIn(key, d)
        self.assertNotIn("tip_od_um", d)        # None → omitted
        self.assertNotIn("tip_profile", d)      # default → omitted
        self.assertEqual(NeedleSpec.from_dict(d), cap)
        self.assertTrue(cap.is_capillary)
        self.assertIsNone(cap.gauge)


# ════════════════════════════════════════════════════════════════════
#  B. Geometry + volumes
# ════════════════════════════════════════════════════════════════════

class TestGeometryAndVolumes(unittest.TestCase):

    def test_straight_needle_volume_is_barrel_only(self):
        n = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        self.assertEqual(n.internal_volume_uL, n.barrel_volume_uL)
        self.assertEqual(n.tip_volume_uL, 0.0)
        # the pre-v7.6 formula
        self.assertAlmostEqual(n.internal_volume_uL,
                               math.pi * (n.id_mm / 2) ** 2 * n.length_mm, places=12)

    def test_capillary_volume_is_barrel_plus_tip(self):
        cap = _capillary()
        self.assertAlmostEqual(cap.internal_volume_uL,
                               cap.barrel_volume_uL + cap.tip_volume_uL, places=12)
        self.assertGreater(cap.tip_volume_uL, 0.0)

    def test_orifice_is_the_tip_when_pulled_and_the_bore_otherwise(self):
        straight = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        self.assertEqual(straight.orifice_id_um, straight.id_um)
        self.assertEqual(straight.cross_section_area_mm2, straight.barrel_area_mm2)

        cap = _capillary()
        self.assertEqual(cap.orifice_id_um, 30.0)
        self.assertLess(cap.cross_section_area_mm2, cap.barrel_area_mm2)

    def test_ink_reserve_unchanged_for_straight_needle(self):
        n = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        self.assertEqual(n.ink_reserve_volume_uL, n.internal_volume_uL)

    def test_ink_reserve_is_tip_only_for_a_capillary(self):
        cap = _capillary()
        self.assertEqual(cap.ink_reserve_volume_uL, cap.tip_volume_uL)
        self.assertLess(cap.ink_reserve_volume_uL, cap.barrel_volume_uL)

    def test_total_length_includes_the_tip(self):
        cap = _capillary()
        self.assertAlmostEqual(cap.total_length_mm, cap.length_mm + 5.0, places=9)
        straight = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        self.assertEqual(straight.total_length_mm, straight.length_mm)

    def test_orifice_od_falls_back_to_the_barrel_ratio(self):
        cap = _capillary()                      # tip_od unset
        self.assertAlmostEqual(cap.orifice_od_um,
                               30.0 * (1000.0 / 580.0), places=9)
        measured = _capillary(tip_od_um=48.0)
        self.assertEqual(measured.orifice_od_um, 48.0)

    def test_non_positive_tip_dims_collapse_to_no_tip(self):
        for kw in ({"tip_id_um": 0.0}, {"tip_length_mm": 0.0},
                   {"tip_id_um": None}, {"tip_id_um": "not a number"}):
            with self.subTest(**kw):
                self.assertFalse(_capillary(**kw).has_tip)

    def test_display_label_never_renders_noneG(self):
        self.assertNotIn("NoneG", _capillary().display_label)
        self.assertEqual(NeedleSpec.from_dict(LEGACY_PAYLOAD).display_label, "27G")

    def test_summary_line_shows_both_stages(self):
        text = _capillary().summary_line()
        self.assertIn("barrel", text)
        self.assertIn("tip", text)
        straight = NeedleSpec.from_dict(LEGACY_PAYLOAD).summary_line()
        self.assertIn("27G", straight)
        self.assertIn("OD 413", straight)
        self.assertIn("ID 210", straight)

    def test_default_fallback_needle_is_never_a_capillary(self):
        # A silent 30 µm default would under-extrude ~190x and read as a
        # hardware fault rather than a configuration gap.
        self.assertFalse(default_fallback_needle().has_tip)


# ════════════════════════════════════════════════════════════════════
#  C. Flow physics — series resistance
# ════════════════════════════════════════════════════════════════════

class _FakeNeedle:
    """The duck-typed stub other suites pass into the physics layer."""
    gauge = 22
    id_m = 0.0004
    length_mm = 25.4


class TestSeriesResistance(unittest.TestCase):

    def setUp(self):
        self.ref = InkSpec(name="__reference__", viscosity_cP=1.0)

    def test_straight_ceiling_is_bit_identical_to_the_legacy_closed_form(self):
        n = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        d, mu = n.id_m, self.ref.viscosity_Pa_s
        length_m = n.length_mm / 1000.0
        legacy = ((DEFAULT_PRESSURE_LIMIT_PA * math.pi * d ** 4)
                  / (128 * mu * length_m)) * 1e9
        # assertEqual, NOT assertAlmostEqual: the ceiling is stored and compared
        # downstream, so a last-ULP drift would propagate.
        self.assertEqual(
            max_safe_flow_rate_uL_s(n, self.ref, DEFAULT_PRESSURE_LIMIT_PA),
            legacy)

    def test_duck_typed_needle_still_resolves(self):
        q = max_safe_flow_rate_uL_s(_FakeNeedle(), self.ref,
                                    DEFAULT_PRESSURE_LIMIT_PA)
        self.assertGreater(q, 0.0)
        self.assertEqual(len(needle_flow_segments(_FakeNeedle())), 1)

    def test_cone_resistance_reduces_to_the_cylinder_form(self):
        # Validates the 1/3 factor and the (d1^2 + d1 d2 + d2^2) numerator.
        cyl = FlowSegment("t", 5.0, 200.0, 200.0)
        cone = FlowSegment("t", 5.0, 200.0, 200.0 * (1 + 1e-9))
        self.assertTrue(cone.is_taper)
        # A dropped 1/3 or a wrong numerator would be a factor-of-3 error, so
        # 1e-6 relative is a decisive check on a taper this close to uniform.
        self.assertAlmostEqual(cone.resistance_factor, cyl.resistance_factor,
                               delta=cyl.resistance_factor * 1e-6)

    def test_tip_dominates_and_is_reported_as_the_limiting_stage(self):
        cap = _capillary()
        barrel_only = NeedleSpec(gauge=None, od_um=1000.0, id_um=580.0,
                                 wall_um=210.0, length_inches=100.0 / 25.4)
        self.assertLess(
            max_safe_flow_rate_uL_s(cap, self.ref, DEFAULT_PRESSURE_LIMIT_PA),
            max_safe_flow_rate_uL_s(barrel_only, self.ref,
                                    DEFAULT_PRESSURE_LIMIT_PA) / 1000.0)
        name, tip_fraction = limiting_flow_segment(cap)
        self.assertEqual(name, "tip")
        self.assertGreater(tip_fraction, 0.99)

    def test_no_tip_reports_zero_tip_pressure_fraction(self):
        name, tip_fraction = limiting_flow_segment(
            NeedleSpec.from_dict(LEGACY_PAYLOAD))
        self.assertEqual(name, "barrel")
        self.assertEqual(tip_fraction, 0.0)

    def test_ceiling_falls_monotonically_as_the_tip_narrows(self):
        previous = None
        for tip in (100.0, 50.0, 30.0, 10.0):
            q = max_safe_flow_rate_uL_s(_capillary(tip_id_um=tip), self.ref,
                                        DEFAULT_PRESSURE_LIMIT_PA)
            if previous is not None:
                self.assertLess(q, previous)
            previous = q

    def test_cylinder_profile_is_conservative_versus_cone(self):
        # A straight tip puts the whole pulled length at the orifice Ø, so it
        # over-estimates resistance — the safe direction for a glass tip.
        common = dict(tip_id_um=30.0, tip_length_mm=5.0)
        q_cyl = max_safe_flow_rate_uL_s(
            _capillary(tip_profile=TIP_PROFILE_CYLINDER, **common),
            self.ref, DEFAULT_PRESSURE_LIMIT_PA)
        q_cone = max_safe_flow_rate_uL_s(
            _capillary(tip_profile=TIP_PROFILE_CONE, **common),
            self.ref, DEFAULT_PRESSURE_LIMIT_PA)
        self.assertLessEqual(q_cyl, q_cone)

    def test_clogging_ratio_uses_the_orifice(self):
        # A 1 mm barrel with a 30 µm tip is a JAMMING needle for a 200 µm
        # spheroid; pre-v7.6 the barrel made it read as "free flow".
        ink = InkSpec(name="cells", cell_diameter_um=200.0)
        regime, ratio = classify_granular_regime(_capillary(), ink)
        self.assertEqual(regime, GranularRegime.JAMMING)
        self.assertLess(ratio, 1.0)


# ════════════════════════════════════════════════════════════════════
#  D. BoreProfile — piecewise volume ↔ lift
# ════════════════════════════════════════════════════════════════════

class TestBoreProfile(unittest.TestCase):

    def setUp(self):
        self.profile = _capillary().bore_profile()

    def test_single_area_profile_reproduces_the_legacy_arithmetic(self):
        flat = BoreProfile.from_area(0.05)
        self.assertFalse(flat.has_tip)
        self.assertAlmostEqual(flat.lift_for_volume(0.2), 0.2 / 0.05, places=12)
        self.assertAlmostEqual(flat.volume_for_lift(4.0), 4.0 * 0.05, places=12)
        self.assertEqual(flat.near_tip_area_mm2, 0.05)

    def test_inverses_hold_across_the_tip_knee(self):
        cap = self.profile.tip_capacity_uL
        for volume in (cap * 0.25, cap * 0.9, cap, cap * 1.5, cap * 20):
            with self.subTest(volume=volume):
                lift = self.profile.lift_for_volume(volume)
                self.assertAlmostEqual(self.profile.volume_for_lift(lift),
                                       volume, places=12)

    def test_continuous_at_the_knee(self):
        cap = self.profile.tip_capacity_uL
        eps = cap * 1e-9
        below = self.profile.lift_for_volume(cap - eps)
        above = self.profile.lift_for_volume(cap + eps)
        self.assertAlmostEqual(below, self.profile.tip_length_mm, places=6)
        self.assertAlmostEqual(above, self.profile.tip_length_mm, places=6)

    def test_lift_flattens_once_past_the_tip(self):
        # The whole point: beyond the tip the spheroid enters the wide barrel
        # and barely rises, so the naive V/A_tip massively over-predicts.
        cap = self.profile.tip_capacity_uL
        lift = self.profile.lift_for_volume(cap * 10)
        naive = (cap * 10) / self.profile.tip_area_mm2
        self.assertLess(lift, self.profile.tip_length_mm * 1.1)
        self.assertGreater(naive, self.profile.tip_length_mm * 5)

    def test_needle_bore_profile_falls_back_for_a_duck_typed_needle(self):
        profile = needle_bore_profile(_FakeNeedle())
        self.assertFalse(profile.has_tip)
        self.assertTrue(profile.is_usable())


# ════════════════════════════════════════════════════════════════════
#  E. Executor — sink timing with the two-stage bore
# ════════════════════════════════════════════════════════════════════

class _Curve:
    """Monotone stand-in for a calibrated SinkCurve."""
    RATE = 0.5   # mm per second

    def lift_for_time(self, t):
        return max(0.0, float(t)) * self.RATE

    def time_for_lift(self, lift):
        return max(0.0, float(lift)) / self.RATE


class TestExecutorSinkTiming(unittest.TestCase):

    def _executor(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex.bore_area_mm2 = 0.0
        ex.bore_profile = None
        ex.sink_curve = None
        ex._estimate_travel_time_s = lambda *a, **k: 4.0
        return ex

    def _cfg(self, **kw):
        from SupportClasses.PickAndPlaceManager import SpheroidPickupConfig
        cfg = SpheroidPickupConfig()
        cfg.sink_timing_enabled = True
        cfg.travel_margin_s = 0.0
        for key, value in kw.items():
            setattr(cfg, key, value)
        return cfg

    def test_no_bore_returns_the_carrier_volume_with_no_wait(self):
        ex = self._executor()
        ex.sink_curve = _Curve()
        cfg = self._cfg()
        volume, wait = ex._planned_aspirate_uL(cfg, object(), object())
        self.assertEqual(volume, cfg.compute_volume_uL())
        self.assertEqual(wait, 0.0)

    def test_legacy_scalar_area_still_drives_the_conversion(self):
        ex = self._executor()
        ex.sink_curve = _Curve()
        ex.bore_area_mm2 = 0.05
        lift = _Curve().lift_for_time(4.0)
        volume, _ = ex._planned_aspirate_uL(self._cfg(), object(), object())
        self.assertAlmostEqual(volume, max(self._cfg().compute_volume_uL(),
                                           lift * 0.05), places=12)

    def test_two_stage_profile_plans_far_less_volume_than_the_barrel_would(self):
        cap = _capillary()
        lift_needed = _Curve().lift_for_time(4.0)

        barrel = self._executor()
        barrel.sink_curve = _Curve()
        barrel.bore_area_mm2 = cap.barrel_area_mm2
        v_barrel, _ = barrel._planned_aspirate_uL(
            self._cfg(spheroid_diameter_um=1.0), object(), object())

        tipped = self._executor()
        tipped.sink_curve = _Curve()
        tipped.bore_profile = cap.bore_profile()
        v_tipped, _ = tipped._planned_aspirate_uL(
            self._cfg(spheroid_diameter_um=1.0), object(), object())

        self.assertLess(v_tipped, v_barrel)
        self.assertAlmostEqual(
            v_tipped, cap.bore_profile().volume_for_lift(lift_needed), places=12)


# ════════════════════════════════════════════════════════════════════
#  F. Spheroid feasibility — advisory, never blocking
# ════════════════════════════════════════════════════════════════════

class TestSpheroidFeasibility(unittest.TestCase):

    def test_spheroid_wider_than_the_tip_warns(self):
        detail = _capillary().spheroid_pickup_detail(120.0)
        self.assertEqual(detail["status"], "too_large")
        self.assertEqual(detail["severity"], "warning")
        self.assertLess(detail["ratio"], 1.0)

    def test_tight_clearance_warns(self):
        detail = _capillary(tip_id_um=110.0).spheroid_pickup_detail(100.0)
        self.assertEqual(detail["status"], "tight")
        self.assertEqual(detail["severity"], "warning")

    def test_comfortable_fit_is_ok(self):
        detail = _capillary(tip_id_um=400.0).spheroid_pickup_detail(100.0)
        self.assertEqual(detail["status"], "ok")
        self.assertEqual(detail["severity"], "info")

    def test_volume_that_overruns_the_tip_warns_past_tip(self):
        cap = _capillary(tip_id_um=400.0)
        overrun = cap.bore_profile().tip_capacity_uL * 5
        detail = cap.spheroid_pickup_detail(100.0, volume_uL=overrun)
        self.assertEqual(detail["status"], "past_tip")

    def test_nothing_is_ever_an_error_severity(self):
        # Operator decision: a deformable spheroid can squeeze through, so the
        # workflow warns and proceeds rather than blocking the run.
        for diameter in (1.0, 30.0, 100.0, 5000.0):
            with self.subTest(diameter=diameter):
                detail = _capillary().spheroid_pickup_detail(diameter)
                self.assertNotEqual(detail["severity"], "error")


# ════════════════════════════════════════════════════════════════════
#  G. Validation + fingerprint
# ════════════════════════════════════════════════════════════════════

def _valid_config(needle) -> HardwareConfig:
    from SupportClasses.PhysicalModels import SyringeSpec
    cfg = HardwareConfig()
    cfg.needle = needle
    ink = InkSpec(name="Ink A")
    cfg.ink_library["Ink A"] = ink
    pump = cfg.pumps["P1"]
    pump.enabled = True
    pump.syringe = SyringeSpec(volume_uL=100)
    pump.inks = [ink]                 # `ink_names` is a read-only property
    cfg.needle_channel_pump_map = {0: "P1"}
    return cfg


class TestValidation(unittest.TestCase):

    def test_capillary_does_not_report_no_needle_gauge_selected(self):
        ok, issues = _valid_config(_capillary()).validate()
        self.assertNotIn("No needle gauge selected", issues)
        self.assertTrue(ok, issues)

    def test_hypodermic_without_a_gauge_keeps_the_legacy_message(self):
        cfg = _valid_config(NeedleSpec(gauge=None, od_um=413, id_um=210,
                                       wall_um=102))
        self.assertIn("No needle gauge selected", cfg.validate()[1])

    def test_missing_needle_still_reports(self):
        cfg = _valid_config(None)
        self.assertIn("No needle gauge selected", cfg.validate()[1])

    def test_tip_wider_than_the_barrel_is_rejected(self):
        cfg = _valid_config(_capillary(tip_id_um=900.0))
        self.assertTrue(any("cannot exceed" in i for i in cfg.validate()[1]))

    def test_multibore_capillary_is_accepted(self):
        # v7.9 DELIBERATELY INVERTS v7.6's rule. A pulled capillary used to be
        # hard-gated to one bore ("A pulled glass capillary is single-bore — set
        # channels to 1"), but the assembly FORM (how many needles are bound
        # together) is orthogonal to the taper of ONE bore, so a backpack of two
        # pulled capillaries is a legitimate build. Validation moved to per-bore
        # geometry instead.
        from SupportClasses.PhysicalModels import (
            NeedleBore, NEEDLE_FORM_BACKPACK, SyringeSpec,
        )

        def _bore(pump_id):
            return NeedleBore(id_um=580.0, od_um=1000.0, wall_um=210.0,
                              length_mm=100.0, needle_type=NEEDLE_TYPE_CAPILLARY,
                              tip_id_um=30.0, tip_length_mm=5.0, pump_id=pump_id)

        cfg = _valid_config(NeedleSpec(needle_form=NEEDLE_FORM_BACKPACK,
                                       bores=[_bore("P1"), _bore("P2")]))
        # Two bores need two pumps; the map is DERIVED from the bores.
        cfg.pumps["P2"].enabled = True
        cfg.pumps["P2"].syringe = SyringeSpec(volume_uL=100)
        cfg.pumps["P2"].inks = [cfg.ink_library["Ink A"]]
        ok, issues = cfg.validate()
        self.assertTrue(ok, issues)
        self.assertFalse(any("single-bore" in i for i in issues), issues)

    def test_two_bores_claiming_one_pump_is_rejected(self):
        # The replacement rule: one pump can only push one volume, so whichever
        # bore was addressed second would be driven blind.
        from SupportClasses.PhysicalModels import NeedleBore, NEEDLE_FORM_BACKPACK

        def _bore(tip):
            return NeedleBore(id_um=580.0, od_um=1000.0, wall_um=210.0,
                              length_mm=100.0, needle_type=NEEDLE_TYPE_CAPILLARY,
                              tip_id_um=tip, tip_length_mm=5.0, pump_id="P1")

        cfg = _valid_config(NeedleSpec(needle_form=NEEDLE_FORM_BACKPACK,
                                       bores=[_bore(30.0), _bore(50.0)]))
        issues = cfg.validate()[1]
        self.assertTrue(any("more than one bore" in i for i in issues), issues)

    def test_repr_never_prints_noneG(self):
        text = repr(_valid_config(_capillary()))
        self.assertNotIn("NoneG", text)
        self.assertIn("apillary", text)


class TestFingerprint(unittest.TestCase):

    class _Settings:
        def __init__(self, values):
            self._values = values

        def get(self, key, default=None):
            return self._values.get(key, default)

    def test_dimension_added_after_a_snapshot_produces_no_diff(self):
        # Otherwise every existing user gets a bogus "needle bore: None → 210"
        # the first time they launch this build.
        saved = {"device": "ME3B V1", "needle_gauge": 27}
        current = {"device": "ME3B V1", "needle_gauge": 27,
                   "needle_bore_um": 210.0, "needle_type": None}
        self.assertEqual(
            CalibrationSnapshotStore.fingerprint_diff(saved, current), [])

    def test_two_capillaries_with_different_tips_differ(self):
        def fp(tip):
            return CalibrationSnapshotStore.build_fingerprint(self._Settings({
                "hardware_config.needle.needle_type": NEEDLE_TYPE_CAPILLARY,
                "hardware_config.needle.tip_id_um": tip,
                "hardware_config.needle.id_um": 580.0,
                "hardware_config.needle.tip_length_mm": 5.0,
            }))
        diffs = CalibrationSnapshotStore.fingerprint_diff(fp(30.0), fp(50.0))
        self.assertTrue(any("needle bore" in d for d in diffs), diffs)

    def test_tip_length_change_is_flagged(self):
        # A pulled needle is barrel + tip long, so this invalidates the
        # plate-bottom Z touch-off exactly as a barrel-length change does.
        def fp(tip_len):
            return CalibrationSnapshotStore.build_fingerprint(self._Settings({
                "hardware_config.needle.tip_id_um": 30.0,
                "hardware_config.needle.tip_length_mm": tip_len,
            }))
        diffs = CalibrationSnapshotStore.fingerprint_diff(fp(5.0), fp(8.0))
        self.assertTrue(any("tip length" in d for d in diffs), diffs)

    def test_straight_needle_fingerprint_has_no_spurious_tip_dimensions(self):
        fp = CalibrationSnapshotStore.build_fingerprint(self._Settings({
            "hardware_config.needle.gauge": 27,
            "hardware_config.needle.id_um": 210.0,
        }))
        self.assertIsNone(fp["needle_type"])
        self.assertIsNone(fp["needle_tip_length_mm"])
        self.assertEqual(fp["needle_bore_um"], 210.0)


# ════════════════════════════════════════════════════════════════════
#  H. NeedleTypeStore
# ════════════════════════════════════════════════════════════════════

class TestNeedleTypeStore(unittest.TestCase):

    def setUp(self):
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.builtin = root / "builtin"
        self.user = root / "user"
        self.builtin.mkdir()
        self.user.mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, directory, nt_id, **kw):
        data = NeedleType(id=nt_id, **kw).to_dict()
        # Let the DIRECTORY decide builtin-ness, as a bundled file does.
        data.pop("builtin", None)
        (directory / f"{nt_id}.json").write_text(json.dumps(data),
                                                 encoding="utf-8")

    def test_user_entry_shadows_the_builtin(self):
        self._write(self.builtin, "x", tip_id_um=30.0)
        self._write(self.user, "x", tip_id_um=55.0)
        store = NeedleTypeStore(self.builtin, self.user)
        self.assertEqual(store.get("x").tip_id_um, 55.0)
        self.assertFalse(store.get("x").builtin)

    def test_delete_user_resurfaces_the_builtin(self):
        self._write(self.builtin, "x", tip_id_um=30.0)
        self._write(self.user, "x", tip_id_um=55.0)
        store = NeedleTypeStore(self.builtin, self.user)
        self.assertTrue(store.delete_user("x"))
        self.assertEqual(store.get("x").tip_id_um, 30.0)
        self.assertTrue(store.get("x").builtin)

    def test_save_user_is_atomic_and_leaves_no_tmp(self):
        store = NeedleTypeStore(self.builtin, self.user)
        self.assertTrue(store.save_user(NeedleType(id="fresh", tip_id_um=20.0)))
        self.assertEqual(list(self.user.glob("*.tmp")), [])
        self.assertEqual(store.get("fresh").tip_id_um, 20.0)

    def test_safe_id_sanitises(self):
        token = safe_id("P-1000 prog #12 / 30µm")
        self.assertRegex(token, r"^[A-Za-z0-9_.-]+$")

    def test_matches_detects_divergence(self):
        nt = NeedleType(id="x", barrel_id_um=580.0, barrel_od_um=1000.0,
                        barrel_length_mm=100.0, tip_id_um=30.0,
                        tip_length_mm=5.0, tip_od_um=None,
                        tip_profile=TIP_PROFILE_CYLINDER)
        same = dict(barrel_id_um=580.0, barrel_od_um=1000.0,
                    barrel_length_mm=100.0, tip_id_um=30.0, tip_length_mm=5.0,
                    tip_od_um=None, tip_profile=TIP_PROFILE_CYLINDER)
        self.assertTrue(nt.matches(**same))
        self.assertFalse(nt.matches(**{**same, "tip_id_um": 31.0}))

    def test_bundled_builtins_load(self):
        from SupportClasses.NeedleTypeStore import get_store
        types = get_store().all()
        self.assertGreaterEqual(len(types), 1)
        self.assertTrue(all(nt.tip_id_um > 0 for nt in types))

    def test_store_module_has_no_gui_dependency(self):
        src = (REPO / "SupportClasses" / "NeedleTypeStore.py").read_text(
            encoding="utf-8")
        for node in ast.walk(ast.parse(src)):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for name in names:
                self.assertFalse(name.startswith(("PySide6", "gui.")),
                                 f"NeedleTypeStore must stay GUI-free: {name}")


# ════════════════════════════════════════════════════════════════════
#  I. Hardware Setup page round-trip
# ════════════════════════════════════════════════════════════════════

class TestNeedlePageRoundTrip(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        from gui.pages.hardware_setup import HardwareSetupPage
        cls.page = HardwareSetupPage()

    def test_straight_needle_survives_set_then_get_unchanged(self):
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        before = cfg.needle.to_dict()
        self.page.set_config(cfg)
        after = self.page.get_config().needle.to_dict()
        # The page rebuilds a fresh NeedleSpec, whose default channel_pump_map
        # uses an int key while a JSON round-trip yields "1" — a pre-existing
        # (and harmless) key-type drift, so compare the rest exactly.
        self.assertEqual(set(after), LEGACY_KEYS)
        for key in LEGACY_KEYS - {"channel_pump_map"}:
            self.assertEqual(after[key], before[key], key)
        self.assertEqual({str(k): v for k, v in after["channel_pump_map"].items()},
                         {str(k): v for k, v in before["channel_pump_map"].items()})

    def test_unusual_length_is_not_silently_coerced(self):
        # Pre-v7.6 a length outside {1.0, 1.5, 2.0} findData'd to -1 and was
        # rewritten to 1.0" on the next save.
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(gauge=27, od_um=413, id_um=210, wall_um=102,
                                length_inches=0.5)
        self.page.set_config(cfg)
        self.assertEqual(self.page.get_config().needle.length_inches, 0.5)

    def test_capillary_survives_set_then_get(self):
        cfg = HardwareConfig()
        cfg.needle = _capillary(tip_od_um=48.0, tip_profile=TIP_PROFILE_CONE)
        self.page.set_config(cfg)
        got = self.page.get_config().needle
        self.assertTrue(got.is_capillary)
        for attr in ("id_um", "od_um", "tip_id_um", "tip_length_mm", "tip_od_um"):
            self.assertAlmostEqual(getattr(got, attr), getattr(cfg.needle, attr),
                                   places=6, msg=attr)
        self.assertAlmostEqual(got.length_mm, cfg.needle.length_mm, places=6)
        self.assertEqual(got.tip_profile, TIP_PROFILE_CONE)

    def test_preset_provenance_survives_when_the_geometry_still_matches(self):
        cfg = HardwareConfig()
        # Geometry identical to the bundled preset, so the picker keeps it.
        cfg.needle = _capillary(needle_type_id="borosilicate-1b100f-30um")
        self.page.set_config(cfg)
        got = self.page.get_config().needle
        if self.page._needle_type_preset_combo.findData(
                "borosilicate-1b100f-30um") < 0:
            self.skipTest("bundled preset not present")
        self.assertEqual(got.needle_type_id, "borosilicate-1b100f-30um")

    def test_provenance_is_dropped_once_the_geometry_diverges(self):
        cfg = HardwareConfig()
        # Same preset id but a tip Ø the preset never had — claiming provenance
        # would be a lie, so the page must clear it.
        cfg.needle = _capillary(tip_id_um=31.0,
                                needle_type_id="borosilicate-1b100f-30um")
        self.page.set_config(cfg)
        self.assertIsNone(self.page.get_config().needle.needle_type_id)

    def test_type_combo_toggles_visibility_without_clamping_the_bore_count(self):
        # v7.9 DELIBERATELY INVERTS the v7.6 clamp (see
        # TestValidation.test_multibore_capillary_is_accepted): the bore count is
        # the assembly's FORM and applies to both needle types, so a capillary no
        # longer forces it to 1. Only the geometry rows still toggle.
        combo = self.page._needle_type_combo
        combo.setCurrentIndex(combo.findData(NEEDLE_TYPE_CAPILLARY))
        self.assertFalse(self.page._cap_row.isHidden())
        self.assertTrue(self.page._hypo_row.isHidden())
        self.assertEqual(self.page.channels_spin.maximum(), 3)
        self.assertTrue(self.page.channels_spin.isEnabled())

        combo.setCurrentIndex(combo.findData(NEEDLE_TYPE_HYPODERMIC))
        self.assertTrue(self.page._cap_row.isHidden())
        self.assertFalse(self.page._hypo_row.isHidden())
        self.assertEqual(self.page.channels_spin.maximum(), 3)

    def test_info_label_shows_both_stages_for_a_capillary(self):
        cfg = HardwareConfig()
        cfg.needle = _capillary()
        self.page.set_config(cfg)
        text = self.page.needle_info_label.text()
        self.assertIn("Barrel", text)
        self.assertIn("tip", text.lower())
        self.assertIn("µL", text)

    def test_max_flow_label_names_the_limiting_stage(self):
        cfg = HardwareConfig()
        cfg.needle = _capillary()
        self.page.set_config(cfg)
        text = self.page._pump_maxflow_lbl.text()
        self.assertIn("capillary", text)
        self.assertIn("tip", text)

        cfg2 = HardwareConfig()
        cfg2.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        self.page.set_config(cfg2)
        self.assertRegex(self.page._pump_maxflow_lbl.text(),
                         r"Max safe pump flow \(\d+G, water ref\)")

    def test_preset_selection_stamps_geometry_and_edits_flip_to_custom(self):
        combo = self.page._needle_type_combo
        combo.setCurrentIndex(combo.findData(NEEDLE_TYPE_CAPILLARY))
        presets = self.page._needle_type_preset_combo
        idx = presets.findData("borosilicate-1b100f-30um")
        if idx < 0:
            self.skipTest("bundled preset not present")
        presets.setCurrentIndex(idx)
        self.assertEqual(self.page._cap_tip_id_spin.value(), 30.0)
        # a builtin cannot be deleted
        self.assertFalse(self.page._needle_preset_del_btn.isEnabled())
        # editing away from the recipe drops back to "(custom)"
        self.page._cap_tip_id_spin.setValue(31.0)
        self.assertIsNone(self.page._needle_type_preset_combo.currentData())


# ════════════════════════════════════════════════════════════════════
#  J. Readout bugfix regressions (all three failed before v7.6)
# ════════════════════════════════════════════════════════════════════

class TestNeedleReadoutBugfixes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    class _Lbl:
        """Minimal label stand-in that just records what it was told to show."""

        def __init__(self):
            self.text = ""

        def setText(self, text):
            self.text = text

    def _panel_with(self, needle):
        """Drive the real ``_refresh_hardware_info`` with stand-in labels.

        The panel writes to a dozen ``_lbl_info_*`` labels; only the needle row
        is under test, so any label it reaches for is auto-created rather than
        enumerated (enumerating would make this test brittle against unrelated
        rows being added).
        """
        from gui.widgets.standard_jog_context import StandardJogContextPanel
        outer = self

        class _Panel(StandardJogContextPanel):
            def __getattr__(self, name):     # only called when NOT found
                if name.startswith("_lbl_info_"):
                    lbl = outer._Lbl()
                    object.__setattr__(self, name, lbl)
                    return lbl
                raise AttributeError(name)

        panel = _Panel.__new__(_Panel)
        cfg = HardwareConfig()
        cfg.needle = needle
        panel._hardware_config = cfg
        panel._controller = None
        panel._plate = None
        panel._well_positions = {}
        panel._z_refs = {}
        # The needle row is written FIRST; the pump / plate / calibration rows
        # after it need state a bare __new__ panel doesn't have. Letting those
        # bail is safe here because the caller asserts on the needle text — if
        # the needle row itself raised, the label stays "" and the test fails.
        try:
            panel._refresh_hardware_info()
        except Exception:
            pass
        return panel._lbl_info_needle.text

    def test_standard_jog_context_shows_od_and_id(self):
        # Pre-v7.6 this panel read `outer_diameter_um`/`inner_diameter_um`,
        # names NeedleSpec has never had, so it only ever showed "27G".
        text = self._panel_with(NeedleSpec.from_dict(LEGACY_PAYLOAD))
        self.assertIn("27G", text)
        self.assertIn("OD 413", text)
        self.assertIn("ID 210", text)

    def test_standard_jog_context_shows_the_capillary_tip(self):
        text = self._panel_with(_capillary())
        self.assertIn("tip", text.lower())
        self.assertIn("30.0", text)

    def test_context_section_shows_od_and_id(self):
        from gui.widgets import context_sections
        rows = {"needle": self._Lbl()}

        section = context_sections.HardwareInfoSection.__new__(
            context_sections.HardwareInfoSection)
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)

        class _Ctx:
            hardware_config = cfg

        section._ctx = _Ctx()
        section._rows = rows
        for pid in ("P1", "P2", "P3"):
            rows[pid] = self._Lbl()
        rows["plate"] = self._Lbl()
        section._refresh()
        self.assertIn("OD 413", rows["needle"].text)
        self.assertIn("ID 210", rows["needle"].text)

    def test_jog_control_pushes_a_real_od_to_the_views(self):
        from gui.pages.jog_control import JogControlPage
        page = JogControlPage.__new__(JogControlPage)
        seen = {}

        class _View:
            def set_needle(self, od, length=None):
                seen.setdefault("od", od)
                seen["length"] = length

            def set_safety_limits(self, *a, **k):
                pass

            def set_plate_flip_180(self, *a, **k):
                pass

        page._workspace_view = _View()
        page._xz_view = _View()

        class _Ctrl:
            safety_limits = None

            def plate_flip_180(self):
                return False

        page.controller = _Ctrl()
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        # Only the needle-push prologue is under test; the rest of the method
        # touches widgets a bare __new__ page doesn't have.
        try:
            page.set_hardware_config(cfg)
        except AttributeError:
            pass
        # Pre-v7.6 this was ALWAYS None: it read `needle.outer_diameter_um`,
        # an attribute NeedleSpec has never had, inside a swallowing except.
        self.assertIsNotNone(seen.get("od"))
        self.assertAlmostEqual(seen["od"], 413.0, places=6)
        self.assertAlmostEqual(seen["length"], 50.8, places=6)

    def test_jog_control_prefers_the_orifice_od_for_a_capillary(self):
        from gui.pages.jog_control import JogControlPage
        page = JogControlPage.__new__(JogControlPage)
        seen = {}

        class _View:
            def set_needle(self, od, length=None):
                seen.setdefault("od", od)
                seen["length"] = length

            def set_safety_limits(self, *a, **k):
                pass

            def set_plate_flip_180(self, *a, **k):
                pass

        page._workspace_view = _View()
        page._xz_view = _View()

        class _Ctrl:
            safety_limits = None

            def plate_flip_180(self):
                return False

        page.controller = _Ctrl()
        cap = _capillary(tip_od_um=48.0)
        cfg = HardwareConfig()
        cfg.needle = cap
        try:
            page.set_hardware_config(cfg)
        except AttributeError:
            pass
        # The pulled tip is what approaches the plate, and the needle is
        # barrel + tip long.
        self.assertAlmostEqual(seen["od"], 48.0, places=6)
        self.assertAlmostEqual(seen["length"], cap.total_length_mm, places=6)


if __name__ == "__main__":
    unittest.main()
