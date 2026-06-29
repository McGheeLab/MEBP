"""
v7.5.x — Quick Print: syringe-budget pre-flight, per-line retract, buffer
default, and reset-to-initial cleanup.

Covers:
  * ``compute_pump_budget`` — feasible / overflow→waste / underflow→add-oil /
    span>capacity→infeasible / cleanup-trough fold-in / shift invariance.
  * ``StageController.pump_volume_to_reach_uL`` — polarity-correct on BOTH
    pump_dir_sign = +1 and −1 (the latent oil-reset sign bug).
  * ``StageController.simulate_pump_budget`` — uncalibrated / unreadable skips.
  * ``run_print_cleanup`` reset-to-initial — waste = live leftover + margin and
    the plunger returns to its baseline on either polarity.
  * ``build_well_plate_job`` — tags the inter-segment hop MOVE_XY/MOVE_Z with the
    fast line speeds; single-object plan is unchanged.
  * Buffer-needles default is 1 on the executor.

Pure / duck-typed — no Qt, no hardware.
"""

import unittest

from SupportClasses.StageController import compute_pump_budget, StageController
from SupportClasses.PrintManager import (
    build_well_plate_job, PrintSettings, CommandType,
)
from SupportClasses.PickAndPlaceManager import PickPlaceExecutor


# ── compute_pump_budget (pure) ────────────────────────────────────────

class TestComputePumpBudget(unittest.TestCase):
    def test_feasible_within_envelope(self):
        # Start at 5, aspirate 3 (→8), dispense 4 (→4); cap 10.
        b = compute_pump_budget([-3.0, +4.0], start_fill_uL=5.0,
                                capacity_uL=10.0)
        self.assertTrue(b["ok"])
        self.assertAlmostEqual(b["peak_fill_uL"], 8.0)
        self.assertAlmostEqual(b["min_fill_uL"], 4.0)
        self.assertIsNone(b["remedy"])

    def test_overflow_offers_waste_oil(self):
        # Aspirate 8 from start 5 → peak 13 > cap 10. span = 13-5 = 8 ≤ 10.
        b = compute_pump_budget([-8.0], start_fill_uL=5.0, capacity_uL=10.0)
        self.assertFalse(b["ok"])
        self.assertTrue(b["feasible_by_shift"])
        self.assertEqual(b["remedy"], "waste_oil")
        self.assertAlmostEqual(b["remedy_uL"], 3.0)          # 13 − 10
        self.assertAlmostEqual(b["recommended_start_fill_uL"], 2.0)  # 5 − 3

    def test_underflow_offers_add_oil(self):
        # Dispense 8 from start 5 → min −3 < 0. span = 5-(-3)=8 ≤ 10.
        b = compute_pump_budget([+8.0], start_fill_uL=5.0, capacity_uL=10.0)
        self.assertFalse(b["ok"])
        self.assertTrue(b["feasible_by_shift"])
        self.assertEqual(b["remedy"], "add_oil")
        self.assertAlmostEqual(b["remedy_uL"], 3.0)          # −(−3)
        self.assertAlmostEqual(b["recommended_start_fill_uL"], 8.0)  # 5 + 3

    def test_span_exceeds_capacity_is_infeasible(self):
        # Aspirate 9 then dispense 18 → peak 14, min −4, span 18 > cap 10.
        b = compute_pump_budget([-9.0, +18.0], start_fill_uL=5.0,
                                capacity_uL=10.0)
        self.assertFalse(b["ok"])
        self.assertFalse(b["feasible_by_shift"])
        self.assertIsNone(b["remedy"])
        self.assertGreater(b["span_uL"], 10.0)

    def test_cleanup_trough_folds_into_min(self):
        # No moves, but a cleanup dip to (start − margin) below empty.
        b = compute_pump_budget([], start_fill_uL=3.0, capacity_uL=20.0,
                                extra_min_fill_uL=3.0 - 5.0)  # margin 5 > start
        self.assertFalse(b["ok"])
        self.assertAlmostEqual(b["min_fill_uL"], -2.0)
        self.assertEqual(b["remedy"], "add_oil")
        self.assertAlmostEqual(b["remedy_uL"], 2.0)

    def test_shift_makes_overflow_feasible(self):
        # After applying the recommended start, the same moves fit exactly.
        b = compute_pump_budget([-8.0], start_fill_uL=5.0, capacity_uL=10.0)
        new_start = b["recommended_start_fill_uL"]
        b2 = compute_pump_budget([-8.0], start_fill_uL=new_start,
                                 capacity_uL=10.0)
        self.assertTrue(b2["ok"])


# ── pump_volume_to_reach_uL polarity (duck-typed self) ────────────────

class _FakePumpSelf:
    """Minimal stand-in exposing only what pump_volume_to_reach_uL touches."""

    def __init__(self, current_uL, dir_sign):
        self._cur = current_uL
        self._dir = dir_sign

    def get_pump_position_uL(self, pump):
        return self._cur

    def pump_dir_sign(self, pump):
        return self._dir


class TestPumpVolumeToReach(unittest.TestCase):
    def test_polarity_plus_one(self):
        # move_pump_uL changes get_pump_position_uL by +1 × V, so the volume to
        # reach `target` from `current` is target − current.
        f = _FakePumpSelf(current_uL=40.0, dir_sign=+1.0)
        v = StageController.pump_volume_to_reach_uL(f, "P1", 50.0)
        self.assertAlmostEqual(v, 10.0)

    def test_polarity_minus_one_is_corrected(self):
        # On pump_dir_sign = −1 the move changes position by −V, so reaching
        # `target` needs V = −(target − current). The OLD code (baseline −
        # current) would have driven the WRONG way.
        f = _FakePumpSelf(current_uL=40.0, dir_sign=-1.0)
        v = StageController.pump_volume_to_reach_uL(f, "P1", 50.0)
        self.assertAlmostEqual(v, -10.0)
        self.assertNotEqual(v, 50.0 - 40.0)  # ≠ the naive (baseline − current)

    def test_unreadable_returns_none(self):
        class _Blind(_FakePumpSelf):
            def get_pump_position_uL(self, pump):
                return None
        f = _Blind(0.0, +1.0)
        self.assertIsNone(StageController.pump_volume_to_reach_uL(f, "P1", 5.0))


# ── simulate_pump_budget skip paths (duck-typed self) ─────────────────

class TestSimulatePumpBudgetSkips(unittest.TestCase):
    def _fake(self, cap, fill):
        f = type("F", (), {})()
        f.pump_capacity_uL = lambda pump: cap
        f.pump_fill_uL = lambda pump: fill
        return f

    def test_uncalibrated_skips(self):
        f = self._fake(cap=None, fill=5.0)
        r = StageController.simulate_pump_budget(f, "P1", [-1.0])
        self.assertFalse(r["ok"])
        self.assertEqual(r["reason"], "uncalibrated")

    def test_fill_unreadable_skips(self):
        f = self._fake(cap=10.0, fill=None)
        r = StageController.simulate_pump_budget(f, "P1", [-1.0])
        self.assertFalse(r["ok"])
        self.assertEqual(r["reason"], "fill_unreadable")

    def test_calibrated_in_bounds_ok(self):
        f = self._fake(cap=10.0, fill=5.0)
        r = StageController.simulate_pump_budget(f, "P1", [-3.0, +3.0])
        self.assertTrue(r["ok"])
        self.assertEqual(r["reason"], "ok")


# ── run_print_cleanup reset-to-initial (real executor + fake ctrl) ────

class _CleanupCtrl:
    """Recording controller that tracks the plunger position in the
    get_pump_position_uL frame and updates it on each move (delta =
    pump_dir_sign × volume), mirroring the real StageController."""

    def __init__(self, start_pos, dir_sign):
        self.calls = []
        self.pos = float(start_pos)
        self.dir = float(dir_sign)
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    def safe_travel_to(self, **kw):
        self.calls.append(("safe_travel_to", kw))
        return True

    def move_xy_absolute_um(self, x_um, y_um, fast=False):
        pass

    def move_z_relative(self, dz):
        pass

    def move_z_user_relative(self, dz):
        pass

    def move_z_absolute(self, z, from_zero_ref=False, feedrate_mm_min=None):
        pass

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def wait_for_z_arrival(self, *a, **k):
        return True

    def ensure_retracted_to(self, *a, **k):
        return True

    # Pump frame (no `settle` kwarg → _settled_pump_move TypeError fallback).
    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
        self.calls.append(("pump", volume_uL))
        self.pos += self.dir * float(volume_uL)

    def get_pump_position_uL(self, pump):
        return self.pos

    def pump_dir_sign(self, pump):
        return self.dir

    def pump_volume_to_reach_uL(self, pump, target):
        # Mirrors StageController.pump_volume_to_reach_uL.
        return self.dir * (float(target) - self.pos)


class TestResetToInitialCleanup(unittest.TestCase):
    def _run(self, dir_sign):
        baseline = 50.0
        leftover = 10.0
        margin = 2.0
        # "Extra fluid" above baseline = a higher FILL; in the get-position
        # frame that's +leftover when dir=−1 (aspirate raises position) and
        # −leftover when dir=+1.
        start_pos = baseline + (leftover if dir_sign < 0 else -leftover)
        ctrl = _CleanupCtrl(start_pos=start_pos, dir_sign=dir_sign)
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.prep_bore = "P1"
        ex.needle_volume_uL = 1.0
        ex.service_z_mm = -16.0
        ex.wash_cycles = 0
        ex.waste_well_pos = (1000.0, 2000.0)
        ex.wash_well_pos = (3000.0, 4000.0)
        ex.oil_well_pos = (5000.0, 6000.0)
        ex.cleanup_reset_to_initial = True
        ex.cleanup_oil_margin_uL = margin
        ex.cleanup_oil_baseline_uL = baseline
        ex.run_print_cleanup()
        pumps = [v for (tag, v) in ctrl.calls if tag == "pump"]
        return ctrl, pumps, baseline, leftover, margin

    def test_reset_returns_to_baseline_dir_plus(self):
        ctrl, pumps, baseline, leftover, margin = self._run(+1.0)
        # Waste = leftover + margin (a dispense, +); oil reset = −margin.
        self.assertEqual(len(pumps), 2)
        self.assertAlmostEqual(pumps[0], leftover + margin)
        self.assertGreater(pumps[0], 0.0)
        self.assertAlmostEqual(pumps[1], -margin)
        self.assertAlmostEqual(ctrl.pos, baseline)  # back to initial

    def test_reset_returns_to_baseline_dir_minus(self):
        ctrl, pumps, baseline, leftover, margin = self._run(-1.0)
        self.assertEqual(len(pumps), 2)
        self.assertAlmostEqual(pumps[0], leftover + margin)
        self.assertGreater(pumps[0], 0.0)
        self.assertAlmostEqual(pumps[1], -margin)
        # The polarity-correct reset still lands exactly on baseline.
        self.assertAlmostEqual(ctrl.pos, baseline)

    def test_prepare_starting_oil_waste_then_aspirate(self):
        # Waste-oil remedy dispenses (+) to waste; add-oil aspirates (−) at oil.
        ctrl = _CleanupCtrl(start_pos=20.0, dir_sign=+1.0)
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.prep_bore = "P1"
        ex.service_z_mm = -16.0
        ex.waste_well_pos = (1.0, 2.0)
        ex.oil_well_pos = (3.0, 4.0)
        ex.prepare_starting_oil(3.0, dispense_to_waste=True)
        ex.prepare_starting_oil(2.0, dispense_to_waste=False)
        pumps = [v for (tag, v) in ctrl.calls if tag == "pump"]
        self.assertAlmostEqual(pumps[0], +3.0)   # waste = dispense
        self.assertAlmostEqual(pumps[1], -2.0)   # add = aspirate


# ── build_well_plate_job per-line hop tagging ─────────────────────────

class TestLineRetractTagging(unittest.TestCase):
    def _job(self, segments, **kw):
        s = PrintSettings(num_layers=1, print_z_height=-16.0, z_up_sign=-1.0,
                          intra_well_hop_z_mm=3.0, **kw)
        return build_well_plate_job(
            well_positions=[("A1", 100.0, 200.0)],
            path_points=[p for seg in segments for p in seg],
            settings=s, pump="P1", path_segments=segments)

    def test_hop_move_xy_and_move_z_carry_fast_speeds(self):
        job = self._job(
            [[(0.0, 0.0), (1.0, 0.0)], [(2.0, 0.0), (3.0, 0.0)]],
            line_move_z_speed_mm_s=30.0, line_move_xy_speed_mm_s=20.0)
        move_xys = [c for c in job.commands if c.type == CommandType.MOVE_XY]
        # First object: full approach MOVE_XY (no hop_z / speed overrides).
        self.assertNotIn("hop_z", move_xys[0].params)
        self.assertNotIn("xy_speed_mm_s", move_xys[0].params)
        # Second object: hop MOVE_XY with the fast line speeds.
        hop = move_xys[1]
        self.assertIn("hop_z", hop.params)
        self.assertAlmostEqual(hop.params["xy_speed_mm_s"], 20.0)
        self.assertAlmostEqual(hop.params["retract_feedrate_mm_min"], 1800.0)
        # The lower MOVE_Z after the hop carries the fast Z feedrate.
        move_zs = [c for c in job.commands if c.type == CommandType.MOVE_Z]
        self.assertNotIn("feedrate_mm_min", move_zs[0].params)   # first object
        self.assertAlmostEqual(move_zs[1].params["feedrate_mm_min"], 1800.0)

    def test_hop_height_uses_intra_well_hop_z_mm(self):
        job = self._job([[(0.0, 0.0)], [(2.0, 0.0)]])  # no fast speeds set
        hop = [c for c in job.commands
               if c.type == CommandType.MOVE_XY and "hop_z" in c.params][0]
        # z_up_sign = −1, print_z = −16, hop 3 mm "up" → −16 + (−1)·3 = −19.
        self.assertAlmostEqual(hop.params["hop_z"], -19.0)
        # No fast speeds → no overrides on the hop.
        self.assertNotIn("xy_speed_mm_s", hop.params)
        self.assertNotIn("retract_feedrate_mm_min", hop.params)

    def test_single_object_plan_unchanged(self):
        # path_segments=None / one segment → no per-line params anywhere.
        s = PrintSettings(num_layers=1, line_move_z_speed_mm_s=30.0,
                          line_move_xy_speed_mm_s=20.0)
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0)], settings=s, pump="P1")
        for c in job.commands:
            if c.type == CommandType.MOVE_XY:
                self.assertNotIn("hop_z", c.params)
                self.assertNotIn("xy_speed_mm_s", c.params)
            if c.type == CommandType.MOVE_Z:
                self.assertNotIn("feedrate_mm_min", c.params)


# ── Buffer-needles default ────────────────────────────────────────────

class TestBufferDefault(unittest.TestCase):
    def test_executor_buffer_default_is_one(self):
        class _C:
            is_xy_connected = is_zp_connected = True
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ex = PickPlaceExecutor(_C(), hw_config=None)
        self.assertEqual(ex.buffer_needles, 1.0)
        self.assertEqual(ex.oil_needles, 1.0)


if __name__ == "__main__":
    unittest.main()
