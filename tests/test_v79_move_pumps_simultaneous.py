"""
v7.9 — ``StageController.move_pumps_uL``: ONE coordinated multi-pump move.

Operator requirement: during needle prep the bores this run uses must do the
same thing SIMULTANEOUSLY. Calling ``move_pump_uL`` per pump cannot deliver
that — Marlin executes planner blocks in order, so two ``G0``s serialise — and
it is actively unsafe besides (the M400 drain would be sized for one move while
draining both; the poller suspend is a plain bool, not refcounted, so whichever
call finishes first re-enables the poller under the other → the v7.5.x false
"ZP disconnected"). One ``G0`` naming several axes is a single planner block:
all named axes start and stop together.

These exercise the pure logic against fakes:
  * exactly ONE ``move_relative`` naming every active axis;
  * per-pump direction sign / µL→mm scale / absolute-raw soft-limit clamp;
  * the VECTOR feedrate clamp — no axis may exceed its own ceiling, including
    an E-mapped pump, whose real speed is |Δ_E|/|Δ_xyz| × F;
  * backlash compensation is never applied (ill-defined across opposed bores);
  * sub-resolution / empty requests are clean no-ops (never a bare ``G0``);
  * disconnected board and an already-set abort refuse (return False);
  * the single-pump ``move_pump_uL`` path is byte-for-byte unchanged.
"""

import math
import unittest
from types import SimpleNamespace
from unittest import mock

from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import (
    StageController,
    _marlin_move_length_mm,
    _PUMP_MOVE_DRAIN_MARGIN_S,
    _PUMP_SETTLE_FALLBACK_RATE_UL_S,
    logger as _sc_logger,
)


# ── Minimal fakes ──────────────────────────────────────────────────────

# ME3B V1's real mapping: the pumps land on Marlin X / Y / E.
ME3B_AXIS_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}


class _FakePumpCfg:
    """One configured pump. ``mm_per_uL`` is the syringe scale; ``fr_per_rate``
    the µL/s → mm/min conversion (= mm_per_uL × 60 for a real syringe)."""

    def __init__(self, mm_per_uL=0.3):
        self.is_configured = True
        self.syringe = SimpleNamespace(volume_uL=100.0)
        self.mm_per_uL = mm_per_uL

    def uL_to_mm(self, v):
        return v * self.mm_per_uL

    def mm_to_uL(self, mm):
        return mm / self.mm_per_uL

    def feedrate_uL_s_to_mm_min(self, r):
        return r * self.mm_per_uL * 60.0


class _FakeHW:
    def __init__(self, settle=0.0, scales=None):
        self.pump_settle_time_s = settle
        scales = scales or {}
        self.pumps = {p: _FakePumpCfg(scales.get(p, 0.3))
                      for p in ("P1", "P2", "P3")}


class _FakeZP:
    """Records every ``move_relative`` / ``flush_moves`` call."""

    def __init__(self, axis_map=None, flush_result=True):
        self.axis_map = dict(axis_map or ME3B_AXIS_MAP)
        self.simulate = True          # → StageController.is_zp_connected True
        self.moves = []               # [(deltas_dict, feedrate)]
        self.flush_calls = []
        self.flush_result = flush_result
        # Bound as an INSTANCE attribute so a test can `del` it to simulate an
        # older controller / fake with no board confirmation.
        self.flush_moves = self._flush_impl

    def move_relative(self, axes, feedrate=None):
        # Mirror the real drop of sub-resolution deltas so a test can assert on
        # what the board would actually see.
        active = {a: d for a, d in axes.items() if abs(d) >= 1e-4}
        if not active:
            return
        self.moves.append((dict(active), feedrate))

    def _flush_impl(self, timeout_s=15.0, abort_event=None):
        self.flush_calls.append(timeout_s)
        return self.flush_result


def _ctrl(*, settle=0.0, scales=None, axis_map=None, limits=None,
          zp_pos=None, dir_signs=None, backlash=False, relief_uL=0.0,
          flush_result=True, connect=True):
    """StageController with just enough state for move_pumps_uL."""
    c = StageController.__new__(StageController)
    c._hardware_config = _FakeHW(settle=settle, scales=scales)
    c.zp_stage = _FakeZP(axis_map=axis_map, flush_result=flush_result) if connect else None
    if limits is None:
        limits = SafetyLimits()
        limits.enabled = False
    c.safety_limits = limits
    c._pos_poller = None
    c._backlash_comp_enabled = backlash
    c._pump_relief_uL = {p: relief_uL for p in ("P1", "P2", "P3")}
    signs = dir_signs or {}
    c.pump_dir_sign = lambda p: float(signs.get(p, 1.0))
    c.get_zp_position = lambda cached=True: zp_pos
    return c


def _axis_speed(feedrate, delta, length_mm):
    """The real speed (mm/min) Marlin gives one axis of a coordinated move."""
    return abs(feedrate) * abs(delta) / length_mm


# ════════════════════════════════════════════════════════════════════
#  The move length Marlin's feedrate applies to
# ════════════════════════════════════════════════════════════════════

class TestMarlinMoveLength(unittest.TestCase):

    def test_cartesian_norm(self):
        self.assertAlmostEqual(
            _marlin_move_length_mm({"X": 3.0, "Y": 4.0}), 5.0)

    def test_e_excluded_when_a_cartesian_axis_moves(self):
        # Marlin's block->millimeters is the XYZ norm; E does not contribute.
        # This is why an E-mapped pump can outrun the commanded vector rate.
        self.assertAlmostEqual(
            _marlin_move_length_mm({"X": 0.3, "E": 5.0}), 0.3)

    def test_e_only_move_uses_the_e_distance(self):
        self.assertAlmostEqual(_marlin_move_length_mm({"E": -2.5}), 2.5)

    def test_zero_cartesian_falls_through_to_e(self):
        self.assertAlmostEqual(
            _marlin_move_length_mm({"X": 0.0, "E": 1.25}), 1.25)


# ════════════════════════════════════════════════════════════════════
#  ONE coordinated G0
# ════════════════════════════════════════════════════════════════════

class TestSingleCoordinatedMove(unittest.TestCase):

    def test_one_move_naming_every_axis(self):
        c = _ctrl()
        self.assertTrue(c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0))
        self.assertEqual(len(c.zp_stage.moves), 1)       # ONE planner block
        deltas, _ = c.zp_stage.moves[0]
        self.assertEqual(set(deltas), {"X", "Y"})        # both bores in it
        self.assertAlmostEqual(deltas["X"], -1.5)
        self.assertAlmostEqual(deltas["Y"], -1.5)

    def test_three_bores_one_move(self):
        c = _ctrl()
        c.move_pumps_uL({"P1": -2.0, "P2": -2.0, "P3": -2.0}, rate_uL_s=1.0)
        self.assertEqual(len(c.zp_stage.moves), 1)
        self.assertEqual(set(c.zp_stage.moves[0][0]), {"X", "Y", "E"})

    def test_axis_letters_come_from_the_live_axis_map(self):
        # Default map (Z→X, P1→Y, P2→Z, P3→E) must route differently.
        c = _ctrl(axis_map={"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"})
        c.move_pumps_uL({"P1": 1.0, "P2": 1.0}, rate_uL_s=1.0)
        self.assertEqual(set(c.zp_stage.moves[0][0]), {"Y", "Z"})

    def test_per_pump_direction_sign(self):
        # One bore's plunger counts up to dispense, the other's counts down;
        # the same dispense intent must produce opposite RAW deltas.
        c = _ctrl(dir_signs={"P1": 1.0, "P2": -1.0})
        c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertAlmostEqual(deltas["X"], +0.9)
        self.assertAlmostEqual(deltas["Y"], -0.9)

    def test_per_pump_syringe_scale(self):
        c = _ctrl(scales={"P1": 0.3, "P2": 0.05})
        c.move_pumps_uL({"P1": -10.0, "P2": -10.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertAlmostEqual(deltas["X"], -3.0)
        self.assertAlmostEqual(deltas["Y"], -0.5)

    def test_opposed_bores_in_one_move(self):
        # One dispensing while another aspirates is legal for a coordinated
        # move (and is exactly why backlash bracketing is refused).
        c = _ctrl()
        c.move_pumps_uL({"P1": +4.0, "P2": -4.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertGreater(deltas["X"], 0)
        self.assertLess(deltas["Y"], 0)


# ════════════════════════════════════════════════════════════════════
#  Vector feedrate — every pump's ceiling honoured at once
# ════════════════════════════════════════════════════════════════════

class TestVectorFeedrateClamp(unittest.TestCase):
    """A coordinated move has ONE feedrate over the vector, so axis i runs at
    F·|Δi|/L. F = min_i(f_i·L/|Δi|) keeps every axis at or below its own
    ceiling; the most-constrained bore therefore paces the whole move."""

    def _limits(self, **flow):
        lim = SafetyLimits()
        lim.enabled = True
        # Wide travel envelope so only the FLOW ceiling binds here.
        for p in ("p1", "p2", "p3"):
            setattr(lim, f"{p}_min", -1000.0)
            setattr(lim, f"{p}_max", 1000.0)
        lim.max_pump_feedrate = 100000.0
        for k, v in flow.items():
            lim.set_max_flow_rate(k, v)
        return lim

    def test_equal_bores_each_run_at_the_requested_rate(self):
        # Identical bores, identical volumes: each axis must end up at exactly
        # the requested plunger speed (18 mm/min for 1 µL/s at 0.3 mm/µL).
        c = _ctrl()
        c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        self.assertAlmostEqual(fr, 18.0 * L / 1.5)
        for d in deltas.values():
            self.assertAlmostEqual(_axis_speed(fr, d, L), 18.0)

    def test_fine_bore_low_ceiling_drags_the_whole_vector_down(self):
        # P2 is the fine bore: its flow ceiling is 0.1 µL/s while 5.0 µL/s is
        # requested. The vector must slow so P2 stays at its ceiling — and P1,
        # sharing the duration, ends up far below its own.
        lim = self._limits(P1=10.0, P2=0.1)
        c = _ctrl(limits=lim, zp_pos=(0.0, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=5.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        v_p1 = _axis_speed(fr, deltas["X"], L)
        v_p2 = _axis_speed(fr, deltas["Y"], L)
        self.assertLessEqual(v_p2, 0.1 * 18.0 + 1e-6)   # at its ceiling
        self.assertLessEqual(v_p1, 5.0 * 18.0 + 1e-6)   # under its own
        self.assertAlmostEqual(v_p1, v_p2)              # same Δ ⇒ same speed
        # And it really is SLOWER than the unconstrained pair would have been.
        c2 = _ctrl(limits=self._limits(P1=10.0, P2=10.0),
                   zp_pos=(0.0, 0.0, 0.0, 0.0))
        c2.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=5.0)
        self.assertLess(fr, c2.zp_stage.moves[0][1])

    def test_no_axis_exceeds_its_ceiling_with_unequal_deltas(self):
        # Different syringes ⇒ different Δ for the same µL. Every axis must
        # still be at or below its own ceiling.
        lim = self._limits(P1=2.0, P2=2.0)
        c = _ctrl(limits=lim, scales={"P1": 0.3, "P2": 0.05},
                  zp_pos=(0.0, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": -6.0, "P2": -6.0}, rate_uL_s=2.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        self.assertLessEqual(_axis_speed(fr, deltas["X"], L), 2.0 * 0.3 * 60 + 1e-6)
        self.assertLessEqual(_axis_speed(fr, deltas["Y"], L), 2.0 * 0.05 * 60 + 1e-6)

    def test_e_mapped_pump_respects_its_own_ceiling(self):
        # THE mapping trap: Marlin's move length excludes E, so an E-mapped
        # pump with the larger delta runs at |ΔE|/|Δxyz| × F. Sizing the vector
        # against the full 4D norm would have let it exceed its ceiling.
        lim = self._limits(P1=2.0, P3=2.0)
        c = _ctrl(limits=lim, scales={"P1": 0.05, "P3": 0.3},
                  zp_pos=(0.0, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": -4.0, "P3": -4.0}, rate_uL_s=2.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        self.assertAlmostEqual(L, abs(deltas["X"]))       # E excluded
        self.assertGreater(abs(deltas["E"]), abs(deltas["X"]))
        self.assertLessEqual(_axis_speed(fr, deltas["E"], L), 2.0 * 0.3 * 60 + 1e-6)
        # A naive full-norm sizing WOULD have over-driven E — pin that this is
        # a real hazard, not a hypothetical.
        naive_L = math.sqrt(deltas["X"] ** 2 + deltas["E"] ** 2)
        naive_fr = min(2.0 * 0.05 * 60 * naive_L / abs(deltas["X"]),
                       2.0 * 0.3 * 60 * naive_L / abs(deltas["E"]))
        self.assertGreater(_axis_speed(naive_fr, deltas["E"], L),
                           2.0 * 0.3 * 60 * 1.05)

    def test_plunger_feedrate_ceiling_also_binds(self):
        lim = self._limits()
        lim.set_pump_feedrate_max("P2", 6.0)     # mm/min
        c = _ctrl(limits=lim, zp_pos=(0.0, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        self.assertLessEqual(_axis_speed(fr, deltas["Y"], L), 6.0 + 1e-6)

    def test_no_rate_leaves_the_board_default(self):
        # Parity with move_pump_uL: no requested rate ⇒ no explicit F.
        c = _ctrl()
        c.move_pumps_uL({"P1": -1.0, "P2": -1.0})
        self.assertIsNone(c.zp_stage.moves[0][1])

    def test_feedrate_floored_at_one(self):
        # A pathologically small ceiling must not produce F ≤ 0 (Marlin treats
        # F0 as an infinite-time move and stalls the planner).
        lim = self._limits(P1=1e-9, P2=1e-9)
        c = _ctrl(limits=lim, zp_pos=(0.0, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)
        self.assertGreaterEqual(c.zp_stage.moves[0][1], 1.0)

    def test_sub_one_mm_min_ceiling_is_not_raised_to_one(self):
        # REGRESSION: the F ≥ 1 floor belongs on the VECTOR, not on a per-axis
        # ceiling. A very fine pulled-glass tip (0.005 µL/s ⇒ 0.09 mm/min)
        # alongside a fast bore is a coordinated move whose vector feedrate is
        # far above 1, so Marlin's own floor is irrelevant — flooring the
        # CEILING at 1.0 drove that tip at 1.0 mm/min = 11× its limit.
        lim = self._limits(P1=0.005, P2=10.0)
        c = _ctrl(limits=lim, zp_pos=(0.0, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": -0.5, "P2": -20.0}, rate_uL_s=10.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        self.assertGreaterEqual(fr, 1.0)        # vector still safe for Marlin
        # …and the board's floor genuinely could not have saved us here.
        self.assertGreater(fr, 1.0)
        self.assertLessEqual(_axis_speed(fr, deltas["X"], L),
                             0.005 * 18.0 + 1e-9)
        self.assertLessEqual(_axis_speed(fr, deltas["Y"], L),
                             10.0 * 18.0 + 1e-6)

    def test_axis_speed_never_exceeds_ceiling_over_a_sweep(self):
        # Property sweep: across mixed ceilings, syringe scales, volumes and
        # mappings (incl. E), EVERY axis must land at or under its own ceiling.
        for flows in ((0.005, 10.0), (0.02, 0.02), (2.0, 0.3), (50.0, 0.001)):
            for scales in ({"P1": 0.3, "P2": 0.05}, {"P1": 0.05, "P2": 0.3}):
                for vols in ((-0.5, -20.0), (-20.0, -0.5), (-6.0, -6.0)):
                    lim = self._limits(P1=flows[0], P2=flows[1])
                    c = _ctrl(limits=lim, scales=scales,
                              zp_pos=(0.0, 0.0, 0.0, 0.0))
                    c.move_pumps_uL({"P1": vols[0], "P2": vols[1]},
                                    rate_uL_s=100.0)
                    deltas, fr = c.zp_stage.moves[0]
                    L = _marlin_move_length_mm(deltas)
                    for letter, pump in (("X", "P1"), ("Y", "P2")):
                        ceiling = (flows[0] if pump == "P1" else flows[1]) \
                            * scales[pump] * 60.0
                        # Marlin's unavoidable F ≥ 1 can still exceed a ceiling
                        # below 1 mm/min when the WHOLE vector is that slow.
                        allowed = max(ceiling, 1.0 if fr <= 1.0 else ceiling)
                        self.assertLessEqual(
                            _axis_speed(fr, deltas[letter], L),
                            allowed * (1 + 1e-9),
                            f"{pump} over its ceiling: flows={flows} "
                            f"scales={scales} vols={vols} F={fr}")


# ════════════════════════════════════════════════════════════════════
#  Soft-limit clamp (absolute raw, shorten-only)
# ════════════════════════════════════════════════════════════════════

class TestSoftLimitClamp(unittest.TestCase):

    def _limits(self):
        lim = SafetyLimits()
        lim.enabled = True
        lim.p1_min, lim.p1_max = -30.0, 0.0
        lim.p2_min, lim.p2_max = -30.0, 0.0
        lim.p3_min, lim.p3_max = -30.0, 0.0
        lim.max_pump_feedrate = 100000.0
        return lim

    def test_one_pump_clamped_the_other_untouched(self):
        # P1 sits 0.6 mm from its max; a 3 µL (0.9 mm) dispense must shorten to
        # 0.6 mm while P2, mid-envelope, moves its full 0.9 mm.
        c = _ctrl(limits=self._limits(), zp_pos=(-0.6, -15.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertAlmostEqual(deltas["X"], 0.6)
        self.assertAlmostEqual(deltas["Y"], 0.9)

    def test_pinned_axis_drops_out_and_the_rest_still_move(self):
        # P1 is exactly at its max: shorten-only makes it a no-op (it must NOT
        # synthesise an opposite-direction snap), and the move still carries P2.
        c = _ctrl(limits=self._limits(), zp_pos=(0.0, -15.0, 0.0, 0.0))
        self.assertTrue(c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0))
        deltas, _ = c.zp_stage.moves[0]
        self.assertEqual(set(deltas), {"Y"})
        self.assertAlmostEqual(deltas["Y"], 0.9)

    def test_clamp_indexes_through_the_live_axis_map(self):
        # P1→X→slot 0, P2→Y→slot 1. A positional (Z,P1,P2,P3) read would clamp
        # the wrong motor's position on this map.
        c = _ctrl(limits=self._limits(), zp_pos=(-0.3, 0.0, 0.0, 0.0))
        c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertAlmostEqual(deltas["X"], 0.3)     # P1 clamped by slot 0
        self.assertEqual(set(deltas), {"X"})         # P2 pinned at slot 1 → 0

    def test_all_clamped_to_nothing_is_a_clean_no_op(self):
        c = _ctrl(limits=self._limits(), zp_pos=(0.0, 0.0, 0.0, 0.0))
        self.assertTrue(c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0))
        self.assertEqual(c.zp_stage.moves, [])       # never a bare G0

    def test_partial_shortening_is_logged_requested_vs_delivered(self):
        # The docstring promises "anything the clamp shortens away is logged
        # with requested-vs-delivered µL". A PARTIAL shortening under-delivers a
        # prep volume just as silently as one clamped to nothing, so it must
        # warn too (SafetyLimits' own warning only names raw mm).
        c = _ctrl(limits=self._limits(), zp_pos=(-0.6, -15.0, 0.0, 0.0))
        with self.assertLogs("SupportClasses.StageController",
                             level="WARNING") as cm:
            c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0)
        msg = "\n".join(cm.output)
        self.assertIn("SHORTENED", msg)
        self.assertIn("P1", msg)
        self.assertNotIn("P2 requested", msg)   # P2 moved in full

    def test_full_delivery_does_not_warn(self):
        c = _ctrl(limits=self._limits(), zp_pos=(-15.0, -15.0, 0.0, 0.0))
        with mock.patch.object(_sc_logger, "warning") as warn:
            c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0)
        self.assertEqual(warn.call_args_list, [])

    def test_unreadable_position_skips_the_clamp(self):
        c = _ctrl(limits=self._limits(), zp_pos=(None, None, None, None))
        c.move_pumps_uL({"P1": 3.0, "P2": 3.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertAlmostEqual(deltas["X"], 0.9)
        self.assertAlmostEqual(deltas["Y"], 0.9)


# ════════════════════════════════════════════════════════════════════
#  Degenerate input / refusals
# ════════════════════════════════════════════════════════════════════

class TestDegenerateAndRefusals(unittest.TestCase):

    def test_empty_dict_is_a_no_op(self):
        c = _ctrl()
        self.assertTrue(c.move_pumps_uL({}, rate_uL_s=1.0))
        self.assertEqual(c.zp_stage.moves, [])

    def test_none_and_zero_entries_ignored(self):
        c = _ctrl()
        self.assertTrue(c.move_pumps_uL({"P1": 0.0, "P2": None}, rate_uL_s=1.0))
        self.assertEqual(c.zp_stage.moves, [])

    def test_zero_entries_dropped_but_real_ones_kept(self):
        c = _ctrl()
        c.move_pumps_uL({"P1": 0.0, "P2": -5.0}, rate_uL_s=1.0)
        self.assertEqual(set(c.zp_stage.moves[0][0]), {"Y"})

    def test_all_sub_resolution_emits_no_move(self):
        # 1e-5 µL × 0.3 mm/µL = 3e-6 mm, far below one Marlin step (1e-4 mm).
        c = _ctrl()
        self.assertTrue(c.move_pumps_uL({"P1": 1e-5, "P2": 1e-5}, rate_uL_s=1.0))
        self.assertEqual(c.zp_stage.moves, [])

    def test_sub_resolution_axis_dropped_others_still_correct(self):
        c = _ctrl()
        c.move_pumps_uL({"P1": 1e-5, "P2": -5.0}, rate_uL_s=1.0)
        deltas, fr = c.zp_stage.moves[0]
        self.assertEqual(set(deltas), {"Y"})
        # The feedrate must be sized against the axes that SURVIVED, so P2 still
        # runs at its requested plunger speed.
        self.assertAlmostEqual(
            _axis_speed(fr, deltas["Y"], _marlin_move_length_mm(deltas)), 18.0)

    def test_disconnected_returns_false_without_raising(self):
        c = _ctrl(connect=False)
        self.assertFalse(c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0))

    def test_dead_serial_returns_false(self):
        # A ZPStageManager can exist with a dead serial after a failed
        # reconnect — is_zp_connected is False, so refuse.
        c = _ctrl()
        c.zp_stage.simulate = False
        c.zp_stage.serial = None
        self.assertFalse(c.move_pumps_uL({"P1": -5.0}, rate_uL_s=1.0))
        self.assertEqual(c.zp_stage.moves, [])

    def test_no_hardware_config_raises(self):
        c = _ctrl()
        c._hardware_config = None
        with self.assertRaises(ValueError):
            c.move_pumps_uL({"P1": -5.0}, rate_uL_s=1.0)

    def test_unconfigured_pump_raises(self):
        # Same contract as move_pump_uL: naming a pump with no syringe is a
        # caller bug, not a runtime condition.
        c = _ctrl()
        c._hardware_config.pumps["P2"].is_configured = False
        with self.assertRaises(ValueError):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)

    def test_two_pumps_on_one_motor_raises(self):
        # Colliding deltas in a single G0 would silently drop one bore.
        c = _ctrl(axis_map={"Z": "Z", "P1": "X", "P2": "X", "P3": "E"})
        with self.assertRaises(ValueError):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)
        self.assertEqual(c.zp_stage.moves, [])

    def test_abort_already_set_commands_nothing(self):
        c = _ctrl()
        ev = SimpleNamespace(is_set=lambda: True)
        self.assertFalse(
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0,
                            abort_event=ev))
        self.assertEqual(c.zp_stage.moves, [])


# ════════════════════════════════════════════════════════════════════
#  Settle / drain — ONE wait, sized to the coordinated duration
# ════════════════════════════════════════════════════════════════════

class TestSettleAndDrain(unittest.TestCase):

    def test_no_settle_no_wait(self):
        c = _ctrl(settle=0.2)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)
        self.assertEqual(sleeps, [])
        self.assertEqual(c.zp_stage.flush_calls, [])

    def test_one_flush_sized_to_the_vector_duration(self):
        # 5 µL at 1 µL/s per bore; simultaneous ⇒ the pair takes 5 s, not 10.
        # ONE M400, its timeout sized to that (not to a single pump's move).
        c = _ctrl(settle=0.0)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0,
                            settle=True)
        self.assertEqual(len(c.zp_stage.flush_calls), 1)
        self.assertAlmostEqual(c.zp_stage.flush_calls[0],
                               5.1 + _PUMP_MOVE_DRAIN_MARGIN_S, places=5)
        self.assertEqual(sleeps, [])            # board confirmed → no sleep

    def test_slowest_bore_sets_the_duration(self):
        # Unequal volumes: the coordinated move lasts as long as the bore that
        # needs the most time (10 µL at 1 µL/s = 10 s), not the shortest.
        c = _ctrl(settle=0.0)
        with mock.patch("SupportClasses.StageController.time.sleep"):
            c.move_pumps_uL({"P1": -10.0, "P2": -2.0}, rate_uL_s=1.0,
                            settle=True)
        self.assertAlmostEqual(c.zp_stage.flush_calls[0],
                               10.1 + _PUMP_MOVE_DRAIN_MARGIN_S, places=5)

    def test_settle_dwell_after_the_drain(self):
        c = _ctrl(settle=0.25)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0,
                            settle=True)
        self.assertEqual(sleeps, [0.25])        # post-move dwell only

    def test_open_loop_fallback_when_no_board_confirmation(self):
        c = _ctrl(settle=0.0)
        del c.zp_stage.flush_moves              # older controller / fake
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0,
                            settle=True)
        self.assertEqual(len(sleeps), 1)
        self.assertAlmostEqual(sleeps[0], 5.1, places=5)

    def test_fallback_duration_without_a_rate(self):
        c = _ctrl(settle=0.0)
        del c.zp_stage.flush_moves
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pumps_uL({"P1": -3.0, "P2": -3.0}, settle=True)
        expected = 3.0 / _PUMP_SETTLE_FALLBACK_RATE_UL_S + 0.1
        self.assertAlmostEqual(sleeps[0], expected, places=5)

    def test_abort_raised_mid_drain_skips_the_dwell(self):
        # Abort arriving after the move is commanded: the settle dwell is
        # skipped so the abort unwinds promptly.
        c = _ctrl(settle=0.5)
        state = {"set": False}
        ev = SimpleNamespace(is_set=lambda: state["set"])

        def _flush(timeout_s=15.0, abort_event=None):
            state["set"] = True                 # abort lands during the M400
            return False
        c.zp_stage.flush_moves = _flush
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0,
                            settle=True, abort_event=ev)
        self.assertEqual(len(c.zp_stage.moves), 1)
        self.assertEqual(sleeps, [])


# ════════════════════════════════════════════════════════════════════
#  Backlash compensation is never applied
# ════════════════════════════════════════════════════════════════════

class TestNoBacklashCompensation(unittest.TestCase):
    """The take-up/unload bracket is per-axis-and-direction, so it is undefined
    when one bore dispenses while another aspirates. Forced off — a caller that
    needs it must issue sequential single-pump moves."""

    def test_still_exactly_one_move_with_compensation_enabled(self):
        c = _ctrl(settle=0.1, backlash=True, relief_uL=0.5)
        with mock.patch("SupportClasses.StageController.time.sleep"):
            c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0,
                            settle=True)
        self.assertEqual(len(c.zp_stage.moves), 1)      # no take-up, no unload

    def test_deltas_are_the_bare_volumes(self):
        c = _ctrl(backlash=True, relief_uL=0.5)
        c.move_pumps_uL({"P1": -5.0}, rate_uL_s=1.0)
        deltas, _ = c.zp_stage.moves[0]
        self.assertAlmostEqual(deltas["X"], -1.5)       # not -1.5 ± relief

    def test_motion_estimate_gets_each_axis_own_speed(self):
        # The estimator drives the ETA readout, so it wants THIS axis's speed
        # (F·|Δ|/L), not the vector feedrate — which over-states every axis and
        # shows the move finishing early (1.41× with two equal bores).
        c = _ctrl()
        notes = []
        c._note_move_estimate_axis_rel = (
            lambda ch, d, fr: notes.append((ch, d, fr)))
        c.move_pumps_uL({"P1": -5.0, "P2": -5.0}, rate_uL_s=1.0)
        deltas, fr = c.zp_stage.moves[0]
        L = _marlin_move_length_mm(deltas)
        self.assertEqual(len(notes), 2)
        for ch, d, axis_fr in notes:
            self.assertAlmostEqual(axis_fr, _axis_speed(fr, d, L))
            self.assertAlmostEqual(axis_fr, 18.0)      # the requested rate
            self.assertLess(axis_fr, fr)               # NOT the vector feedrate

    def test_no_compensate_kwarg_is_accepted(self):
        # The signature deliberately omits it, so a caller cannot ask for a
        # bracket that the coordinated move cannot honour.
        c = _ctrl()
        with self.assertRaises(TypeError):
            c.move_pumps_uL({"P1": -5.0}, rate_uL_s=1.0, compensate=True)


# ════════════════════════════════════════════════════════════════════
#  The single-pump path must be untouched
# ════════════════════════════════════════════════════════════════════

class TestSinglePumpPathUnchanged(unittest.TestCase):
    """move_pump_uL / _finish_pump_submove keep their exact pre-v7.9
    behaviour — the new est_s parameter is opt-in and defaults to the old
    volume/rate derivation."""

    def _single(self, settle_time=0.0):
        c = _ctrl(settle=settle_time)
        c.pump_moves = []
        c.move_pump_relative = (
            lambda pump, dist, fr=None: c.pump_moves.append((pump, dist, fr)))
        del c.zp_stage.flush_moves          # exercise the open-loop fallback
        return c

    def test_one_axis_one_move(self):
        c = self._single()
        c.move_pump_uL("P1", 2.0, rate_uL_s=10.0)
        self.assertEqual(len(c.pump_moves), 1)
        pump, dist, fr = c.pump_moves[0]
        self.assertEqual(pump, "P1")
        self.assertAlmostEqual(dist, 0.6)               # 2 µL × 0.3 mm/µL
        self.assertAlmostEqual(fr, 10.0 * 0.3 * 60.0)   # plunger mm/min

    def test_settle_false_is_still_a_passthrough(self):
        c = self._single(settle_time=0.2)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 2.0, rate_uL_s=10.0, settle=False)
        self.assertEqual(sleeps, [])

    def test_settle_true_sleep_sequence_unchanged(self):
        # Regression-locked by the pump-settle suite: completion wait derived
        # from vol/rate (2/10 + 0.1), then one post-move dwell.
        c = self._single(settle_time=0.2)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 2.0, rate_uL_s=10.0, settle=True)
        self.assertEqual(len(sleeps), 2)
        self.assertAlmostEqual(sleeps[0], 0.3)
        self.assertAlmostEqual(sleeps[1], 0.2)

    def test_finish_submove_without_est_s_derives_from_vol_rate(self):
        c = self._single()
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c._finish_pump_submove(4.0, 2.0, block=True, dwell_s=0.0)
        self.assertAlmostEqual(sleeps[0], 4.0 / 2.0 + 0.1)

    def test_finish_submove_est_s_overrides(self):
        c = self._single()
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c._finish_pump_submove(4.0, 2.0, block=True, dwell_s=0.0,
                                   est_s=7.5)
        self.assertAlmostEqual(sleeps[0], 7.5)


if __name__ == "__main__":
    unittest.main()
