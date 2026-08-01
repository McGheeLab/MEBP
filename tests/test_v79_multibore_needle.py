"""test_v79_multibore_needle.py — the v7.9 multi-bore needle model.

Pins the Stage 1 foundation: `NeedleForm`, `NeedleBore`, `NeedleSpec.bores` and
the duck-typed per-bore accessors.

The load-bearing property is BYTE-IDENTITY: a single-bore needle must serialize
to exactly the seven legacy keys, in their original order, so every saved setup
on disk round-trips unchanged. `needle_form` and `bores` are therefore
conditional-emit. This suite asserts that against the real files in
`config/hardware/`, not just synthetic ones.

The second load-bearing property is the v7.6 FAIL-SAFE direction:
`cross_section_area_mm2` and friends resolve BORE 0 rather than aggregating, so
a call site that has not been taught about multi-bore assemblies gets ONE REAL
BORE's number instead of a fictional sum.
"""

from __future__ import annotations

import glob
import json
import math
import os
import unittest
from unittest.mock import MagicMock

from SupportClasses.PhysicalModels import (
    NEEDLE_FORM_BACKPACK, NEEDLE_FORM_BORE_COUNT, NEEDLE_FORM_SINGLE,
    NEEDLE_FORM_TRIPLE, NEEDLE_FORMS, NEEDLE_TYPE_CAPILLARY,
    NEEDLE_TYPE_HYPODERMIC, TIP_PROFILE_CONE, InkSpec, NeedleBore, NeedleSpec,
    needle_bore_at, needle_bore_count, needle_bore_internal_volume_uL,
    needle_bore_offset_um, needle_bore_z_offset_mm, needle_flow_segments,
    needle_max_bore_z_offset_mm, needle_orifice_area_mm2, needle_orifice_id_um,
)
from SupportClasses.FlowPhysics import max_safe_flow_rate_uL_s

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGACY_KEYS = ["gauge", "od_um", "id_um", "wall_um", "length_inches",
               "num_channels", "channel_pump_map"]


def _straight(**over) -> NeedleSpec:
    base = dict(gauge=27, od_um=413.0, id_um=210.0, wall_um=102.0,
                length_inches=2.0)
    base.update(over)
    return NeedleSpec(**base)


def _backpack() -> NeedleSpec:
    """Two needles of DIFFERENT sizes bound together — the case that was
    literally unrepresentable before v7.9."""
    return NeedleSpec(
        needle_form=NEEDLE_FORM_BACKPACK,
        bores=[
            NeedleBore(gauge=22, od_um=718.0, id_um=413.0, wall_um=152.0,
                       length_mm=50.8, pump_id="P1", label="coarse"),
            NeedleBore(gauge=30, od_um=311.0, id_um=159.0, wall_um=76.0,
                       length_mm=50.8, pump_id="P2", label="fine",
                       offset_um=(320.0, -140.0), z_offset_mm=0.040),
        ],
    )


# ── Byte-identity: the whole reason bores are conditional-emit ───────

class TestLegacyByteIdentity(unittest.TestCase):
    def test_single_bore_emits_exactly_the_seven_legacy_keys(self):
        d = _straight().to_dict()
        self.assertEqual(list(d), LEGACY_KEYS)

    def test_single_bore_emits_neither_new_key(self):
        d = _straight().to_dict()
        self.assertNotIn("bores", d)
        self.assertNotIn("needle_form", d)

    def test_every_real_on_disk_setup_round_trips_identically(self):
        """The operator's actual saved setups. Key ORDER matters too — this is
        what a diff of the settings file would show."""
        paths = sorted(glob.glob(os.path.join(_REPO, "config", "hardware",
                                              "*.json")))
        checked = 0
        for p in paths:
            with open(p, encoding="utf-8") as fh:
                try:
                    data = json.load(fh)
                except json.JSONDecodeError:
                    continue
            if not isinstance(data, dict):
                continue
            block = data.get("needle")
            if not isinstance(block, dict):
                continue
            checked += 1
            out = NeedleSpec.from_dict(block).to_dict()
            with self.subTest(setup=os.path.basename(p)):
                self.assertEqual(out, block)
                self.assertEqual(list(out), list(block))
        self.assertGreater(checked, 0, "no real setups found to check")

    def test_legacy_multi_channel_count_emits_no_bore_list(self):
        """A pre-v7.9 needle with num_channels=3 carries no per-bore data, so
        adding one would be an invention — and would break byte-identity."""
        n = _straight(num_channels=3)
        self.assertNotIn("bores", n.to_dict())
        self.assertEqual(n.to_dict()["num_channels"], 3)

    def test_capillary_conditional_keys_still_conditional(self):
        n = NeedleSpec(id_um=1000.0, od_um=1500.0, length_inches=2.0,
                       needle_type=NEEDLE_TYPE_CAPILLARY,
                       tip_id_um=30.0, tip_length_mm=3.0)
        d = n.to_dict()
        self.assertEqual(d["needle_type"], NEEDLE_TYPE_CAPILLARY)
        self.assertNotIn("tip_profile", d, "default profile must stay implicit")
        self.assertNotIn("bores", d)


# ── bores_resolved() always yields at least one real bore ────────────

class TestBoresResolved(unittest.TestCase):
    def test_single_bore_needle_resolves_to_itself(self):
        n = _straight()
        bores = n.bores_resolved()
        self.assertEqual(len(bores), 1)
        self.assertEqual(bores[0].id_um, n.id_um)
        self.assertAlmostEqual(bores[0].orifice_area_mm2,
                               n.cross_section_area_mm2, places=15)

    def test_legacy_count_synthesizes_that_many_identical_bores(self):
        n = _straight(num_channels=3)
        bores = n.bores_resolved()
        self.assertEqual(len(bores), 3)
        self.assertEqual(len({b.id_um for b in bores}), 1)

    def test_never_returns_none_or_empty(self):
        for n in (NeedleSpec(), _straight(), _backpack(),
                  _straight(num_channels=0)):
            self.assertGreaterEqual(len(n.bores_resolved()), 1)

    def test_bores_are_authoritative_and_mirror_bore_zero(self):
        n = _backpack()
        self.assertEqual(n.id_um, 413.0)
        self.assertEqual(n.od_um, 718.0)
        self.assertEqual(n.gauge, 22)
        self.assertEqual(n.num_channels, 2)

    def test_bore_zero_is_forced_to_be_the_datum(self):
        """Bore 0 IS needle_origin_um; a non-zero offset there is meaningless
        and would double-count."""
        n = NeedleSpec(needle_form=NEEDLE_FORM_BACKPACK, bores=[
            NeedleBore(id_um=400.0, od_um=700.0, length_mm=50.0, pump_id="P1",
                       offset_um=(99.0, 99.0), z_offset_mm=9.0),
            NeedleBore(id_um=150.0, od_um=300.0, length_mm=50.0, pump_id="P2",
                       offset_um=(320.0, 0.0)),
        ])
        self.assertEqual(n.bore(0).offset_um, (0.0, 0.0))
        self.assertEqual(n.bore(0).z_offset_mm, 0.0)
        self.assertEqual(n.bore(1).offset_um, (320.0, 0.0))

    def test_out_of_range_index_clamps_to_the_datum(self):
        """A stale index from a saved profile must not abort a run; bore 0 is
        the fail-safe answer."""
        n = _backpack()
        for bad in (2, 99, -1, None, "x"):
            self.assertEqual(n.bore(bad).id_um, n.bore(0).id_um)

    def test_dict_bores_are_coerced(self):
        n = NeedleSpec(needle_form=NEEDLE_FORM_BACKPACK, bores=[
            {"id_um": 400.0, "od_um": 700.0, "length_mm": 50.0, "pump_id": "P1"},
            {"id_um": 150.0, "od_um": 300.0, "length_mm": 50.0, "pump_id": "P2"},
        ])
        self.assertIsInstance(n.bore(1), NeedleBore)
        self.assertEqual(n.bore(1).pump_id, "P2")


# ── The v7.6 fail-safe direction must survive ────────────────────────

class TestFailSafeResolvesBoreZero(unittest.TestCase):
    def test_area_is_bore_zero_not_a_sum(self):
        n = _backpack()
        total = sum(b.orifice_area_mm2 for b in n.bores_resolved())
        self.assertAlmostEqual(n.cross_section_area_mm2,
                               n.bore(0).orifice_area_mm2, places=15)
        self.assertNotAlmostEqual(n.cross_section_area_mm2, total, places=9)

    def test_free_function_agrees_with_bore_zero(self):
        n = _backpack()
        self.assertAlmostEqual(needle_orifice_area_mm2(n),
                               n.bore(0).orifice_area_mm2, places=15)
        self.assertAlmostEqual(needle_orifice_id_um(n), n.bore(0).orifice_id_um,
                               places=9)

    def test_internal_volume_is_per_bore_and_assembly_is_separate(self):
        """Prep multiples are "1 BORE's worth"; summing would silently inflate
        every prep volume on a multi-bore assembly."""
        n = _backpack()
        self.assertAlmostEqual(n.internal_volume_uL,
                               n.bore(0).internal_volume_uL, places=12)
        self.assertAlmostEqual(
            n.assembly_internal_volume_uL,
            sum(b.internal_volume_uL for b in n.bores_resolved()), places=12)
        self.assertGreater(n.assembly_internal_volume_uL, n.internal_volume_uL)


# ── Per-bore geometry genuinely differs ──────────────────────────────

class TestPerBoreGeometry(unittest.TestCase):
    def test_a_backpack_holds_two_different_diameters(self):
        n = _backpack()
        self.assertEqual(n.bore(0).id_um, 413.0)
        self.assertEqual(n.bore(1).id_um, 159.0)

    def test_per_bore_volumes_differ_substantially(self):
        n = _backpack()
        v0 = needle_bore_internal_volume_uL(n, 0)
        v1 = needle_bore_internal_volume_uL(n, 1)
        self.assertGreater(v0, 2 * v1)

    def test_bore_lookup_by_pump_is_case_insensitive(self):
        n = _backpack()
        self.assertEqual(n.bore_for_pump("p2").id_um, 159.0)
        self.assertEqual(n.bore_for_pump(" P2 ").id_um, 159.0)
        self.assertEqual(n.bore_index_for_pump("P2"), 1)
        self.assertIsNone(n.bore_for_pump("P3"))
        self.assertIsNone(n.bore_for_pump(""))

    def test_max_bore_z_offset_is_the_longest_bore(self):
        """A descend must be planned against the bore that reaches lowest."""
        n = _backpack()
        self.assertAlmostEqual(n.max_bore_z_offset_mm, 0.040, places=12)
        self.assertAlmostEqual(needle_max_bore_z_offset_mm(n), 0.040, places=12)

    def test_a_bore_may_be_a_pulled_capillary_independently(self):
        """Form and taper are ORTHOGONAL — a backpack of capillaries is legal."""
        n = NeedleSpec(needle_form=NEEDLE_FORM_BACKPACK, bores=[
            NeedleBore(id_um=1000.0, od_um=1500.0, length_mm=50.0, pump_id="P1",
                       needle_type=NEEDLE_TYPE_CAPILLARY, tip_id_um=30.0,
                       tip_length_mm=3.0),
            NeedleBore(gauge=30, id_um=159.0, od_um=311.0, length_mm=50.0,
                       pump_id="P2"),
        ])
        self.assertTrue(n.bore(0).has_tip)
        self.assertFalse(n.bore(1).has_tip)
        self.assertAlmostEqual(n.bore(0).orifice_id_um, 30.0, places=9)
        self.assertEqual(len(n.bore(0).flow_segments()), 2)
        self.assertEqual(len(n.bore(1).flow_segments()), 1)

    def test_cone_tip_reduces_to_the_cylinder_form_at_equal_diameters(self):
        b = NeedleBore(id_um=200.0, od_um=400.0, length_mm=10.0,
                       needle_type=NEEDLE_TYPE_CAPILLARY, tip_id_um=200.0,
                       tip_length_mm=5.0, tip_profile=TIP_PROFILE_CONE)
        tip = b.flow_segments()[-1]
        expected = (tip.length_mm / 1000.0) / (200e-6 ** 4)
        # Relative, not absolute: these are ~3e12, so `places=` is meaningless.
        self.assertLess(abs(tip.resistance_factor - expected) / expected, 1e-12)


# ── Per-bore flow ceilings — the safety payoff ───────────────────────

class TestPerBoreFlowCeiling(unittest.TestCase):
    def setUp(self):
        self.ink = InkSpec(name="water", viscosity_cP=1.0)

    def test_bore_zero_ceiling_is_bit_identical_to_an_equivalent_needle(self):
        """Not almost-equal — the ceiling is stored and compared downstream."""
        n = _backpack()
        single = NeedleSpec(gauge=22, od_um=718.0, id_um=413.0, wall_um=152.0,
                            length_inches=50.8 / 25.4)
        self.assertEqual(max_safe_flow_rate_uL_s(n.bore(0), self.ink),
                         max_safe_flow_rate_uL_s(single, self.ink))

    def test_a_coarse_and_a_fine_bore_have_very_different_ceilings(self):
        n = _backpack()
        q0 = max_safe_flow_rate_uL_s(n.bore(0), self.ink)
        q1 = max_safe_flow_rate_uL_s(n.bore(1), self.ink)
        self.assertGreater(q0 / q1, 10.0,
                           "if these were close, one shared ceiling would be "
                           "defensible; they are not")

    def test_a_pulled_tip_ceiling_is_orders_below_a_hypodermic_bore(self):
        """Applying the coarse bore's ceiling to this pump shatters glass."""
        coarse = NeedleBore(gauge=22, id_um=413.0, od_um=718.0, length_mm=50.8)
        tip = NeedleBore(id_um=1000.0, od_um=1500.0, length_mm=50.0,
                         needle_type=NEEDLE_TYPE_CAPILLARY, tip_id_um=30.0,
                         tip_length_mm=3.0)
        ratio = (max_safe_flow_rate_uL_s(coarse, self.ink)
                 / max_safe_flow_rate_uL_s(tip, self.ink))
        self.assertGreater(ratio, 100.0)


# ── Duck-typing tolerance (≈30 call sites pass stubs) ────────────────

class TestDuckTypingTolerance(unittest.TestCase):
    def test_a_bore_is_accepted_by_the_legacy_accessors(self):
        b = _backpack().bore(1)
        self.assertGreater(needle_orifice_area_mm2(b), 0.0)
        self.assertEqual(len(needle_flow_segments(b)), 1)
        self.assertGreater(needle_bore_internal_volume_uL(b, 0), 0.0)

    def test_minimal_stub_still_works(self):
        class _Stub:
            gauge = 27
            id_m = 210e-6
            length_mm = 50.8

        s = _Stub()
        self.assertEqual(needle_bore_count(s), 1)
        self.assertEqual(needle_bore_offset_um(s, 1), (0.0, 0.0))
        self.assertEqual(needle_bore_z_offset_mm(s, 1), 0.0)
        self.assertIs(needle_bore_at(s, 3), s)
        self.assertGreater(needle_orifice_id_um(s), 0.0)

    def test_magicmock_does_not_explode(self):
        m = MagicMock()
        self.assertGreaterEqual(needle_bore_count(m), 1)
        self.assertEqual(needle_bore_offset_um(m, 1), (0.0, 0.0))
        self.assertEqual(needle_bore_z_offset_mm(m, 1), 0.0)

    def test_offsets_default_to_zero_so_the_motion_path_is_a_no_op(self):
        """This is what lets the offset-aware move be applied unconditionally."""
        n = _straight()
        for k in range(4):
            self.assertEqual(needle_bore_offset_um(n, k), (0.0, 0.0))


# ── Form enum ────────────────────────────────────────────────────────

class TestNeedleForm(unittest.TestCase):
    def test_default_is_single(self):
        self.assertEqual(_straight().needle_form, NEEDLE_FORM_SINGLE)

    def test_unknown_form_degrades_to_single(self):
        self.assertEqual(NeedleSpec(needle_form="octopus").needle_form,
                         NEEDLE_FORM_SINGLE)

    def test_form_is_orthogonal_to_needle_type(self):
        """A form must never be smuggled into needle_type: __post_init__
        silently rewrites an unknown needle_type to hypodermic, so the whole
        assembly would vanish with no error."""
        n = NeedleSpec(needle_type=NEEDLE_FORM_BACKPACK)
        self.assertEqual(n.needle_type, NEEDLE_TYPE_HYPODERMIC)
        self.assertEqual(n.needle_form, NEEDLE_FORM_SINGLE)

    def test_nominal_bore_counts(self):
        self.assertEqual(NEEDLE_FORM_BORE_COUNT[NEEDLE_FORM_SINGLE], 1)
        self.assertEqual(NEEDLE_FORM_BORE_COUNT[NEEDLE_FORM_BACKPACK], 2)
        self.assertEqual(NEEDLE_FORM_BORE_COUNT[NEEDLE_FORM_TRIPLE], 3)
        self.assertEqual(set(NEEDLE_FORM_BORE_COUNT), set(NEEDLE_FORMS))


# ── Serialization of a real assembly ─────────────────────────────────

class TestMultiBoreSerialization(unittest.TestCase):
    def test_round_trip_is_stable_and_preserves_everything(self):
        n = _backpack()
        rt = NeedleSpec.from_dict(n.to_dict())
        self.assertEqual(rt.to_dict(), n.to_dict())
        self.assertEqual(rt.needle_form, NEEDLE_FORM_BACKPACK)
        self.assertEqual(rt.bore(1).pump_id, "P2")
        self.assertEqual(rt.bore(1).offset_um, (320.0, -140.0))
        self.assertAlmostEqual(rt.bore(1).z_offset_mm, 0.040, places=12)
        self.assertEqual(rt.bore(1).label, "fine")

    def test_json_survives_the_tuple_to_list_trip(self):
        n = _backpack()
        rt = NeedleSpec.from_dict(json.loads(json.dumps(n.to_dict())))
        self.assertEqual(rt.bore(1).offset_um, (320.0, -140.0))

    def test_unknown_keys_are_filtered_on_both_levels(self):
        d = _backpack().to_dict()
        d["some_future_needle_key"] = 1
        d["bores"][1]["some_future_bore_key"] = 2
        rt = NeedleSpec.from_dict(d)      # must not raise
        self.assertEqual(rt.bore(1).id_um, 159.0)

    def test_bore_defaults_are_not_emitted(self):
        b = NeedleBore(id_um=200.0, od_um=400.0, wall_um=100.0, length_mm=25.4)
        d = b.to_dict()
        for absent in ("pump_id", "label", "offset_um", "z_offset_mm",
                       "needle_type", "tip_profile", "gauge"):
            self.assertNotIn(absent, d)


if __name__ == "__main__":
    unittest.main()
