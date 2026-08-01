"""
tests/test_v79_hardware_config_bores.py — v7.9 Stage 2: HardwareConfig carries a
multi-bore needle assembly, and each PUMP gets its own flow ceiling.

Three things are load-bearing here:

  1. A single-bore setup must be BIT-IDENTICAL to pre-v7.9 — the same seven
     needle keys, the same stored bore→pump map, and the same three flow
     ceilings to the last ULP (they are stored and compared downstream).
  2. Each bore of a multi-bore assembly has its OWN pump, i.e. N independent
     pressure sources, so one shared ceiling would let a narrow bore be driven
     tens-to-thousands of times over-pressure. Measured on the fixtures below:
     45.5× for 22G-vs-30G, 7071× for 22G-vs-a-30 µm-pulled-tip.
  3. The operator's bore→pump assignments must survive both a config load and a
     bore-count change in the UI.
"""

from __future__ import annotations

import json
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PhysicalModels import (          # noqa: E402
    NeedleSpec, NeedleBore, InkSpec, SyringeSpec,
    NEEDLE_FORM_SINGLE, NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE,
    NEEDLE_TYPE_CAPILLARY,
)
from SupportClasses.HardwareConfig import HardwareConfig    # noqa: E402
from SupportClasses.SafetyLimits import (            # noqa: E402
    SafetyLimits, REFERENCE_VISCOSITY_CP,
)
from SupportClasses.FlowPhysics import (             # noqa: E402
    max_safe_flow_rate_uL_s, DEFAULT_PRESSURE_LIMIT_PA,
)

# The exact on-disk shape of a saved needle block (mirrors the v7.6 suite).
LEGACY_PAYLOAD = {
    "gauge": 27, "od_um": 413, "id_um": 210, "wall_um": 102,
    "length_inches": 2.0, "num_channels": 1, "channel_pump_map": {"1": "P1"},
}
LEGACY_KEYS = list(LEGACY_PAYLOAD)


def _bore22(**kw) -> NeedleBore:
    base = dict(id_um=413.0, od_um=718.0, wall_um=152.0, length_mm=25.4, gauge=22)
    base.update(kw)
    return NeedleBore(**base)


def _bore30(**kw) -> NeedleBore:
    base = dict(id_um=159.0, od_um=311.0, wall_um=76.0, length_mm=25.4, gauge=30)
    base.update(kw)
    return NeedleBore(**base)


def _tip_bore(**kw) -> NeedleBore:
    """A pulled capillary as ONE bore of an assembly: 580 µm blank, 30 µm tip."""
    base = dict(id_um=580.0, od_um=1000.0, wall_um=210.0, length_mm=100.0,
                needle_type=NEEDLE_TYPE_CAPILLARY, tip_id_um=30.0,
                tip_length_mm=5.0)
    base.update(kw)
    return NeedleBore(**base)


def _backpack(*bores, form=NEEDLE_FORM_BACKPACK) -> NeedleSpec:
    return NeedleSpec(needle_form=form, bores=list(bores))


def _cfg(needle, pumps=("P1", "P2", "P3")) -> HardwareConfig:
    cfg = HardwareConfig()
    cfg.needle = needle
    ink = InkSpec(name="Ink A")
    cfg.ink_library["Ink A"] = ink
    for pid in pumps:
        cfg.pumps[pid].enabled = True
        cfg.pumps[pid].syringe = SyringeSpec(volume_uL=250, stroke_length_mm=30.0)
        cfg.pumps[pid].inks = [ink]
    return cfg


def _reference_ceiling(bore_like) -> float:
    return max_safe_flow_rate_uL_s(
        bore_like, InkSpec(name="r", viscosity_cP=REFERENCE_VISCOSITY_CP),
        DEFAULT_PRESSURE_LIMIT_PA)


# ════════════════════════════════════════════════════════════════════
#  A. HardwareConfig serializes the bores (task 1)
# ════════════════════════════════════════════════════════════════════

class TestHardwareConfigBoreSerialization(unittest.TestCase):
    """`to_dict`/`from_dict` delegate the needle to NeedleSpec, so the bores
    flow through with no HardwareConfig code of their own — confirmed here
    rather than assumed."""

    def test_backpack_survives_a_json_round_trip(self):
        cfg = _cfg(_backpack(
            _bore22(pump_id="P1", label="big"),
            _bore30(pump_id="P2", label="small",
                    offset_um=(1200.0, -30.0), z_offset_mm=0.25),
        ))
        rt = HardwareConfig.from_dict(json.loads(json.dumps(cfg.to_dict())))
        n = rt.needle
        self.assertEqual(n.needle_form, NEEDLE_FORM_BACKPACK)
        self.assertEqual(n.bore_count, 2)
        self.assertEqual(n.bore(1).gauge, 30)
        self.assertEqual(n.bore(1).pump_id, "P2")
        self.assertEqual(n.bore(1).label, "small")
        self.assertEqual(n.bore_offset_um(1), (1200.0, -30.0))
        self.assertAlmostEqual(n.bore(1).z_offset_mm, 0.25)
        # Bore 0 stays the datum through the round-trip.
        self.assertEqual(n.bore_offset_um(0), (0.0, 0.0))

    def test_capillary_bore_of_a_backpack_keeps_its_tip(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"),
                             _tip_bore(pump_id="P2", offset_um=(900.0, 0.0))))
        rt = HardwareConfig.from_dict(json.loads(json.dumps(cfg.to_dict())))
        tip = rt.needle.bore(1)
        self.assertTrue(tip.has_tip)
        self.assertTrue(tip.is_capillary)
        self.assertAlmostEqual(tip.tip_id_um, 30.0)
        self.assertAlmostEqual(tip.tip_length_mm, 5.0)
        # Bore 0 is still the plain 22G cannula — a mixed assembly is legal.
        self.assertFalse(rt.needle.bore(0).has_tip)

    def test_triple_round_trips_all_three(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"),
                             _bore30(pump_id="P2", offset_um=(500.0, 0.0)),
                             _bore30(pump_id="P3", offset_um=(0.0, 500.0)),
                             form=NEEDLE_FORM_TRIPLE))
        rt = HardwareConfig.from_dict(json.loads(json.dumps(cfg.to_dict())))
        self.assertEqual(rt.needle.needle_form, NEEDLE_FORM_TRIPLE)
        self.assertEqual(rt.needle.bore_count, 3)
        self.assertEqual([b.pump_id for b in rt.needle.bores_resolved()],
                         ["P1", "P2", "P3"])

    def test_single_bore_needle_block_is_byte_identical(self):
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        emitted = cfg.to_dict()["needle"]
        self.assertEqual(list(emitted), LEGACY_KEYS)
        self.assertEqual(emitted, LEGACY_PAYLOAD)

    def test_from_dict_still_filters_unknown_keys(self):
        data = HardwareConfig().to_dict()
        data["needle"] = dict(LEGACY_PAYLOAD, from_a_future_build=True)
        # Forward-compat: an unknown key must not destroy the whole load.
        self.assertEqual(HardwareConfig.from_dict(data).needle.gauge, 27)


# ════════════════════════════════════════════════════════════════════
#  B. The bore→pump map is derived, never emptied (task 2)
# ════════════════════════════════════════════════════════════════════

class TestBorePumpMapDerivation(unittest.TestCase):

    def test_multi_bore_map_is_derived_from_the_bores(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"), _bore30(pump_id="P3")))
        cfg.needle_channel_pump_map = {0: "P2"}       # stale / contradictory
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1", 1: "P3"})
        self.assertEqual(cfg.to_dict()["needle_channel_pump_map"],
                         {"0": "P1", "1": "P3"})

    def test_map_stays_zero_based(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"), _bore30(pump_id="P2")))
        # 0-based, unlike the deprecated 1-based NeedleSpec.channel_pump_map.
        self.assertEqual(min(cfg.resolved_bore_pump_map()), 0)

    def test_single_bore_hand_set_map_is_preserved(self):
        # A pre-v7.9 config: bores=None + a map set by the UI. Nothing on the
        # synthesized bore can reproduce it, so deriving must not touch it.
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        cfg.needle_channel_pump_map = {0: "P2"}
        self.assertIsNone(cfg.needle.bores)
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P2"})
        self.assertEqual(cfg.to_dict()["needle_channel_pump_map"], {"0": "P2"})
        self.assertFalse(cfg.sync_bore_pump_map_from_needle())

    def test_unwired_multi_bore_falls_back_rather_than_emptying(self):
        # Mid-migration the bores may exist with no pump_id yet; emptying the map
        # would destroy the assignment the operator just made in the UI.
        cfg = _cfg(_backpack(_bore22(), _bore30()))
        cfg.needle_channel_pump_map = {0: "P1", 1: "P3"}
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1", 1: "P3"})

    def test_partially_wired_multi_bore_reports_only_what_is_claimed(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"), _bore30()))
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1"})

    def test_load_syncs_the_live_field(self):
        # PrintPlanOfAction reads the live field's len(), so a stale saved map
        # must not keep disagreeing with the bore that feeds each pump.
        data = _cfg(_backpack(_bore22(pump_id="P1"),
                              _bore30(pump_id="P3"))).to_dict()
        data["needle_channel_pump_map"] = {"0": "P2"}   # as an older build wrote
        rt = HardwareConfig.from_dict(data)
        self.assertEqual(rt.needle_channel_pump_map, {0: "P1", 1: "P3"})

    def test_sync_reports_whether_it_changed(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"), _bore30(pump_id="P2")))
        cfg.needle_channel_pump_map = {}
        self.assertTrue(cfg.sync_bore_pump_map_from_needle())
        self.assertEqual(cfg.needle_channel_pump_map, {0: "P1", 1: "P2"})
        self.assertFalse(cfg.sync_bore_pump_map_from_needle())

    def test_no_needle_leaves_the_map_alone(self):
        cfg = HardwareConfig()
        cfg.needle = None
        cfg.needle_channel_pump_map = {0: "P1"}
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1"})

    def test_derived_ids_are_normalized_like_the_bore_lookup(self):
        # `bore_for_pump` strips + upper-cases, so a bore holding "p1" resolves
        # fine for the flow ceiling. The DERIVED map must normalize identically
        # or the two disagree: `validate` calls an enabled pump "not enabled",
        # `to_dict` persists the raw id, and every consumer comparing against
        # "P1" misses. The STORED map is still returned exactly as saved.
        cfg = _cfg(_backpack(_bore22(pump_id="p1"), _bore30(pump_id=" p2 ")),
                   pumps=("P1", "P2"))
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1", 1: "P2"})
        ok, issues = cfg.validate()
        self.assertTrue(ok, issues)
        self.assertEqual(cfg.to_dict()["needle_channel_pump_map"],
                         {"0": "P1", "1": "P2"})

    def test_a_stored_map_is_never_rewritten_by_normalization(self):
        # Legacy files must round-trip byte-for-byte, oddities included.
        cfg = _cfg(NeedleSpec.from_dict(LEGACY_PAYLOAD))
        cfg.needle_channel_pump_map = {0: "p1"}
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "p1"})
        self.assertEqual(cfg.to_dict()["needle_channel_pump_map"], {"0": "p1"})


# ════════════════════════════════════════════════════════════════════
#  C. Per-PUMP flow ceiling — SAFETY CRITICAL (task 3)
# ════════════════════════════════════════════════════════════════════

class TestPerPumpFlowCeiling(unittest.TestCase):

    def test_single_bore_is_bit_identical_to_the_assembly_value(self):
        # assertEqual, not assertAlmostEqual: the ceiling is stored and compared
        # downstream, so a last-ULP drift would propagate.
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152,
                            length_inches=1.0)
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(needle))
        expected = _reference_ceiling(needle)
        self.assertGreater(expected, 0.0)
        for pid in ("P1", "P2", "P3"):
            self.assertEqual(sl.get_max_flow_rate(pid), expected, pid)

    def test_synthesized_bores_claim_no_pump_so_nothing_changes(self):
        # A pre-v7.9 multi-CHANNEL theta needle (num_channels=2, bores=None):
        # its synthesized bores carry no pump_id, so every pump still gets the
        # assembly value — byte-identical legacy behaviour.
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152,
                            num_channels=2)
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(needle))
        expected = _reference_ceiling(needle)
        for pid in ("P1", "P2", "P3"):
            self.assertEqual(sl.get_max_flow_rate(pid), expected, pid)

    def test_backpack_gives_each_pump_its_own_bores_ceiling(self):
        bp = _backpack(_bore22(pump_id="P1"), _bore30(pump_id="P2"))
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(bp))
        self.assertEqual(sl.get_max_flow_rate("P1"),
                         _reference_ceiling(bp.bore(0)))
        self.assertEqual(sl.get_max_flow_rate("P2"),
                         _reference_ceiling(bp.bore(1)))
        # The whole point: they are NOT the same number. 22G vs 30G ≈ 45×.
        ratio = sl.get_max_flow_rate("P1") / sl.get_max_flow_rate("P2")
        self.assertGreater(ratio, 20.0)

    def test_a_pulled_tip_bore_is_ceilinged_orders_of_magnitude_lower(self):
        # This is the shattered-glass case: pre-v7.9 P2 inherited the 22G bore's
        # ceiling, which is ~7000× what its 30 µm tip can take.
        bp = _backpack(_bore22(pump_id="P1"),
                       _tip_bore(pump_id="P2", offset_um=(900.0, 0.0)))
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(bp))
        assembly = _reference_ceiling(bp)          # == bore 0's, the OLD answer
        self.assertEqual(sl.get_max_flow_rate("P1"), assembly)
        self.assertLess(sl.get_max_flow_rate("P2"), assembly / 1000.0)
        self.assertGreater(sl.get_max_flow_rate("P2"), 0.0)

    def test_an_unclaimed_pump_on_a_backpack_gets_the_NARROWEST_bore(self):
        # P3 is configured but claimed by neither a bore nor the map, so which
        # bore it feeds is unknown. On a HETEROGENEOUS assembly the assembly
        # value is bore 0's — i.e. the WIDEST — which is the over-pressure
        # direction; the fail-safe answer is the narrowest bore. (Too slow is
        # recoverable; over-pressure shatters a glass tip.) Single-bore and
        # synthesized-bore needles are unaffected — see the two tests above,
        # which pin them to the assembly value bit-for-bit.
        bp = _backpack(_bore22(pump_id="P1"), _bore30(pump_id="P2"))
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(bp))
        self.assertEqual(sl.get_max_flow_rate("P3"),
                         _reference_ceiling(bp.bore(1)))
        self.assertLess(sl.get_max_flow_rate("P3"), _reference_ceiling(bp))

    def test_the_map_resolves_a_pump_the_bores_have_not_claimed_yet(self):
        # THE hole this whole change exists to close, in the one configuration
        # `resolved_bore_pump_map`'s own docstring calls reachable: the bores are
        # not wired up yet, so the operator's GUI-set map is the only statement
        # of which pump feeds which bore. Resolving from the bores ALONE handed
        # P2 — a 30 µm pulled glass tip — bore 0's 22 G ceiling, measured 7071×
        # over-pressure, in a config `validate()` passes clean.
        bp = _backpack(_bore22(), _tip_bore())          # no pump_id anywhere
        cfg = _cfg(bp, pumps=("P1", "P2"))
        cfg.needle_channel_pump_map = {0: "P1", 1: "P2"}
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1", 1: "P2"})
        self.assertTrue(cfg.validate()[0], cfg.validate()[1])

        sl = SafetyLimits()
        sl.update_from_hardware_config(cfg)
        self.assertEqual(sl.get_max_flow_rate("P1"),
                         _reference_ceiling(bp.bore(0)))
        self.assertEqual(sl.get_max_flow_rate("P2"),
                         _reference_ceiling(bp.bore(1)))
        self.assertLess(sl.get_max_flow_rate("P2"),
                        sl.get_max_flow_rate("P1") / 1000.0)
        # And it is the ceiling that actually clamps the move.
        self.assertLess(sl.clamp_flow_rate(1000.0, "P2"), 1.0)

    def test_pump_id_match_is_case_insensitive(self):
        bp = _backpack(_bore22(pump_id="p1"), _bore30(pump_id=" p2 "))
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(bp))
        self.assertEqual(sl.get_max_flow_rate("P2"),
                         _reference_ceiling(bp.bore(1)))

    def test_unconfigured_pump_is_untouched(self):
        bp = _backpack(_bore22(pump_id="P1"), _bore30(pump_id="P2"))
        cfg = _cfg(bp, pumps=("P1", "P2"))          # P3 has no syringe
        sl = SafetyLimits()
        sl.set_max_flow_rate("P3", 4.2)
        sl.update_from_hardware_config(cfg)
        self.assertAlmostEqual(sl.get_max_flow_rate("P3"), 4.2)

    def test_a_degenerate_bore_keeps_the_assembly_value_not_zero(self):
        # 0 means "no limit enforced" — widening to it would be the unsafe
        # direction, so an unusable bore must fall back, never clear.
        bp = _backpack(_bore22(pump_id="P1"), _bore30(pump_id="P2", id_um=0.0))
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(bp))
        self.assertEqual(sl.get_max_flow_rate("P2"), _reference_ceiling(bp))

    def test_no_needle_still_leaves_existing_ceilings_alone(self):
        cfg = _cfg(None)
        sl = SafetyLimits()
        sl.set_max_flow_rate("P1", 4.2)
        sl.update_from_hardware_config(cfg)
        self.assertAlmostEqual(sl.get_max_flow_rate("P1"), 4.2)

    def test_clamp_uses_the_per_pump_ceiling(self):
        # End to end: the derived ceiling is what actually clamps a move.
        bp = _backpack(_bore22(pump_id="P1"), _tip_bore(pump_id="P2"))
        sl = SafetyLimits()
        sl.update_from_hardware_config(_cfg(bp))
        wide = sl.get_max_flow_rate("P1")
        self.assertEqual(sl.clamp_flow_rate(wide / 2.0, "P1"), wide / 2.0)
        # The SAME request on the glass tip is clamped hard.
        self.assertLess(sl.clamp_flow_rate(wide / 2.0, "P2"), wide / 1000.0)


# ════════════════════════════════════════════════════════════════════
#  D. Per-bore validation (task 5)
# ════════════════════════════════════════════════════════════════════

class TestPerBoreValidation(unittest.TestCase):

    def test_multi_bore_capillary_is_accepted(self):
        # v7.9: FORM (how many needles are bound together) is orthogonal to the
        # taper of one bore, so a backpack of two pulled capillaries is legal.
        cfg = _cfg(_backpack(_tip_bore(pump_id="P1"),
                             _tip_bore(pump_id="P2", offset_um=(900.0, 0.0))),
                   pumps=("P1", "P2"))
        ok, issues = cfg.validate()
        self.assertTrue(ok, issues)
        self.assertFalse(any("single-bore" in i for i in issues), issues)

    def test_two_bores_on_one_pump_is_rejected(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"), _bore30(pump_id="P1")),
                   pumps=("P1", "P2"))
        ok, issues = cfg.validate()
        self.assertFalse(ok)
        self.assertTrue(any("more than one bore" in i for i in issues), issues)

    def test_a_bad_bore_is_named_by_number(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"),
                             _tip_bore(pump_id="P2", tip_id_um=900.0)),
                   pumps=("P1", "P2"))
        issues = cfg.validate()[1]
        self.assertTrue(any(i.startswith("Bore 2:") and "cannot exceed" in i
                            for i in issues), issues)
        # Bore 1 is fine, so nothing is attributed to it.
        self.assertFalse(any(i.startswith("Bore 1:") for i in issues), issues)

    def test_single_bore_messages_carry_no_bore_prefix(self):
        # The v7.6 strings are asserted elsewhere and shown to the operator.
        cfg = _cfg(NeedleSpec(gauge=None, od_um=413, id_um=210, wall_um=102))
        self.assertIn("No needle gauge selected", cfg.validate()[1])

    def test_zero_bore_diameter_is_flagged(self):
        cfg = _cfg(NeedleSpec(gauge=22, od_um=718, id_um=0.0))
        self.assertTrue(any("inner Ø must be greater than 0" in i
                            for i in cfg.validate()[1]))

    def test_outer_not_exceeding_inner_is_flagged(self):
        cfg = _cfg(NeedleSpec(gauge=22, od_um=200.0, id_um=413.0))
        self.assertTrue(any("outer Ø must exceed its inner Ø" in i
                            for i in cfg.validate()[1]))

    def test_a_healthy_backpack_of_mixed_gauges_validates(self):
        cfg = _cfg(_backpack(_bore22(pump_id="P1"),
                             _bore30(pump_id="P2", offset_um=(1200.0, 0.0))),
                   pumps=("P1", "P2"))
        ok, issues = cfg.validate()
        self.assertTrue(ok, issues)

    def test_a_duck_typed_needle_stub_does_not_crash_validate(self):
        # Several call sites pass needle-LIKE objects exposing only a couple of
        # attributes; `num_channels` / `bores_resolved` are not among them.
        class _Stub:
            gauge = 22
            id_um = 413.0
            od_um = 718.0
            length_mm = 25.4
        cfg = _cfg(_Stub(), pumps=("P1",))
        cfg.needle_channel_pump_map = {0: "P1"}
        ok, issues = cfg.validate()
        self.assertTrue(ok, issues)


# ════════════════════════════════════════════════════════════════════
#  E. The bore-count spin no longer wipes the map (task 4)
# ════════════════════════════════════════════════════════════════════

class TestBoreCountSpinPreservesTheMap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        ink = InkSpec(name="Ink A")
        cfg.ink_library["Ink A"] = ink
        for pid in ("P1", "P2"):
            cfg.pumps[pid].enabled = True
            cfg.pumps[pid].syringe = SyringeSpec(volume_uL=250,
                                                 stroke_length_mm=30.0)
            cfg.pumps[pid].inks = [ink]
        cfg.needle_channel_pump_map = {0: "P1"}
        page.set_config(cfg)
        return page

    def test_bumping_the_count_keeps_bore_0s_pump(self):
        page = self._page()
        self.assertEqual(page.get_config().needle_channel_pump_map, {0: "P1"})
        page.channels_spin.setValue(2)               # 1 → 2 bores
        # Bore 0 keeps P1; the NEW bore 1 is simply unassigned.
        self.assertEqual(page.get_config().needle_channel_pump_map, {0: "P1"})
        self.assertEqual(len(page._channel_map_widgets), 2)

    def test_assignments_survive_a_round_trip_of_the_count(self):
        page = self._page()
        page.channels_spin.setValue(3)
        combos = [c for _lbl, c in page._channel_map_widgets]
        combos[1].setCurrentIndex(combos[1].findData("P2"))
        self.assertEqual(page.get_config().needle_channel_pump_map,
                         {0: "P1", 1: "P2"})
        # Down to 2 then back to 3 — bore 2's row is gone, 0 and 1 remain.
        page.channels_spin.setValue(2)
        self.assertEqual(page.get_config().needle_channel_pump_map,
                         {0: "P1", 1: "P2"})
        page.channels_spin.setValue(3)
        self.assertEqual(page.get_config().needle_channel_pump_map,
                         {0: "P1", 1: "P2"})

    def test_loading_a_config_does_not_inherit_stale_selections(self):
        # The combos hold the OUTGOING setup while a new one is applied, so the
        # incoming map must win for every index.
        page = self._page()
        page.channels_spin.setValue(2)
        combos = [c for _lbl, c in page._channel_map_widgets]
        combos[1].setCurrentIndex(combos[1].findData("P2"))
        self.assertEqual(page.get_config().needle_channel_pump_map,
                         {0: "P1", 1: "P2"})

        fresh = HardwareConfig()
        fresh.needle = NeedleSpec.from_dict(LEGACY_PAYLOAD)
        for pid in ("P1", "P2"):
            fresh.pumps[pid].enabled = True
            fresh.pumps[pid].syringe = SyringeSpec(volume_uL=250,
                                                   stroke_length_mm=30.0)
        fresh.needle_channel_pump_map = {0: "P2"}
        page.set_config(fresh)
        self.assertEqual(page.get_config().needle_channel_pump_map, {0: "P2"})

    def test_a_capillary_is_no_longer_clamped_to_one_bore(self):
        page = self._page()
        combo = page._needle_type_combo
        combo.setCurrentIndex(combo.findData(NEEDLE_TYPE_CAPILLARY))
        self.assertEqual(page.channels_spin.maximum(), 3)
        self.assertTrue(page.channels_spin.isEnabled())
        page.channels_spin.setValue(2)
        needle = page.get_config().needle
        self.assertTrue(needle.is_capillary)
        self.assertEqual(needle.num_channels, 2)
        self.assertEqual(needle.bore_count, 2)


if __name__ == "__main__":
    unittest.main()
