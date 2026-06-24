"""
v7.5.x — Spheroid pick & place: pump API + distinct pick/place Z.

Covers the execution bugs surfaced on HW:
  * `move_pump_uL() got an unexpected keyword argument 'feedrate_mm_min'` — the
    executor passed a pre-converted feedrate under the wrong kwarg; it now
    passes `rate_uL_s` (which `move_pump_uL` converts + flow-clamps itself).
  * Z never reached the operate height — the pick/place used a single unset
    `operating_z_mm=0.0`, and the first move (empty well name) was treated as
    an intra-well 1 mm jiggle. Now the source lowers to `pick_z_mm`, the dest
    to `place_z_mm`, and empty-well-name targets always do a full safe travel.

These drive the real `PickPlaceExecutor` against a recording fake controller —
no Qt / hardware.
"""

import math
import unittest

from SupportClasses.PickAndPlaceManager import (
    AbortException, OperationQueue, OperationType, PickPlaceExecutor,
    PickPlaceOperation, PickPlaceTarget, SpheroidPickupConfig,
)
from SupportClasses.PhysicalModels import NeedleSpec


class _RecCtrl:
    """Recording stand-in for StageController."""

    def __init__(self):
        self.calls = []
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    # movement primitives the executor uses
    def safe_travel_to(self, **kw):
        self.calls.append(("safe_travel_to", kw))
        return True

    def move_xy_absolute_um(self, x_um, y_um, fast=False):
        self.calls.append(("move_xy_absolute_um", x_um, y_um))

    def move_z_relative(self, dz):
        self.calls.append(("move_z_relative", dz))

    def move_z_user_relative(self, dz):
        self.calls.append(("move_z_user_relative", dz))

    def move_z_absolute(self, z, from_zero_ref=False, feedrate_mm_min=None):
        self.calls.append(("move_z_absolute", z))

    def wait_for_xy_arrival(self, *a, **k):
        self.calls.append(("wait_for_xy_arrival", a, k))
        return True

    def wait_for_z_arrival(self, *a, **k):
        self.calls.append(("wait_for_z_arrival", a, k))
        return True

    def ensure_retracted_to(self, safe_z_zero_ref_mm, *a, **k):
        self.calls.append(("ensure_retracted_to", safe_z_zero_ref_mm))
        return True

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
        # Mirrors the REAL signature — a `feedrate_mm_min=` kwarg would TypeError.
        self.calls.append(("move_pump_uL", pump, volume_uL, rate_uL_s))


def _spheroid_op(pick_off=0.10, place_off=0.50, well_a="", well_b=""):
    cfg = SpheroidPickupConfig(
        spheroid_diameter_um=200.0, safety_factor=1.5, pickup_bore="P1",
        pickup_speed_uL_s=1.0, release_speed_uL_s=2.0,
        pick_z_offset_mm=pick_off, place_z_offset_mm=place_off,
    )
    src = PickPlaceTarget(target_id="P001", x_um=47583.0, y_um=48698.0,
                          well_name=well_a)
    dst = PickPlaceTarget(target_id="D001", x_um=67439.0, y_um=47722.0,
                          well_name=well_b)
    return PickPlaceOperation(
        op_id="OP1", op_type=OperationType.SPHEROID_PICKUP,
        source_target=src, dest_target=dst, config=cfg)


class TestSpheroidExecution(unittest.TestCase):
    def _run(self, pick_z=-16.0, place_z=-14.0, **op_kw):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = pick_z
        ex.place_z_mm = place_z
        q = OperationQueue()
        q.add(_spheroid_op(**op_kw))
        ok = ex.execute_queue(q)
        return ctrl, ok

    def test_no_pump_kwarg_crash_and_uses_rate_uL_s(self):
        ctrl, ok = self._run()
        self.assertTrue(ok)  # would be False if move_pump_uL raised TypeError
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertEqual(len(pumps), 2)
        # Aspirate negative @ pickup speed, dispense positive @ release speed.
        self.assertLess(pumps[0][2], 0.0)
        self.assertEqual(pumps[0][3], 1.0)
        self.assertGreater(pumps[1][2], 0.0)
        self.assertEqual(pumps[1][3], 2.0)

    def test_source_and_dest_use_distinct_operate_z(self):
        ctrl, _ = self._run(pick_z=-16.0, place_z=-14.0)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 2)  # full safe travel for both
        self.assertAlmostEqual(travels[0][1]["target_z_mm"], -16.0)  # pick
        self.assertAlmostEqual(travels[1][1]["target_z_mm"], -14.0)  # place

    def test_empty_well_names_always_full_safe_travel(self):
        # Both targets have well_name="" — must NOT be treated as intra-well
        # (no relative jiggle); each is a full safe_travel_to.
        ctrl, _ = self._run()
        self.assertEqual(
            sum(1 for c in ctrl.calls if c[0] == "safe_travel_to"), 2)
        self.assertEqual(
            sum(1 for c in ctrl.calls if c[0] == "move_z_relative"), 0)

    def test_safe_z_passed_to_travel(self):
        ctrl, _ = self._run()
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        for _, kw in travels:
            self.assertAlmostEqual(kw["safe_z_mm"], -35.0)

    def test_run_ends_at_safe_z(self):
        # The op ends at place_z (down); execute_queue's finally must retract
        # the needle to the safe travel height on completion.
        ctrl, ok = self._run()
        self.assertTrue(ok)
        retracts = [c for c in ctrl.calls if c[0] == "ensure_retracted_to"]
        self.assertEqual(len(retracts), 1)
        self.assertAlmostEqual(retracts[0][1], -35.0)
        # The retract is the LAST motion call (after the place dispense).
        self.assertEqual(ctrl.calls[-1][0], "ensure_retracted_to")

    def test_retract_on_safe_z_runs_even_after_failure(self):
        # A mid-op failure must still leave the needle retracted.
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
        def _boom(**kw):
            raise RuntimeError("simulated move failure")
        ctrl.safe_travel_to = _boom
        q = OperationQueue()
        q.add(_spheroid_op())
        ex.execute_queue(q)
        self.assertTrue(
            any(c[0] == "ensure_retracted_to" for c in ctrl.calls))

    def test_zp_disconnected_refuses_xy_move(self):
        # If the ZP board has dropped (XY still connected), safe_travel_to would
        # skip its retract and drive XY with an unretracted needle. The executor
        # must refuse the move — NO safe_travel_to / move_xy* is issued.
        ctrl = _RecCtrl()
        ctrl.is_zp_connected = False
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
        q = OperationQueue()
        q.add(_spheroid_op())
        ex.execute_queue(q)  # aborts internally; must not raise out
        self.assertEqual(
            [c for c in ctrl.calls
             if c[0] in ("safe_travel_to", "move_xy_absolute_um")], [])

    def test_zp_drop_midrun_blocks_place_leg(self):
        # ZP connected for the pick, then drops before the place: the place XY
        # move must NOT happen (only the pick's safe_travel_to ran).
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0

        # Drop ZP the moment the aspirate pump call happens (between pick & place).
        real_pump = ctrl.move_pump_uL
        def _drop_then_pump(*a, **k):
            ctrl.is_zp_connected = False
            return real_pump(*a, **k)
        ctrl.move_pump_uL = _drop_then_pump

        q = OperationQueue()
        q.add(_spheroid_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 1)  # pick only; place refused

    def test_pick_z_falls_back_to_operating_when_unset(self):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.operating_z_mm = -10.0  # pick_z_mm/place_z_mm left None
        q = OperationQueue()
        q.add(_spheroid_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        for _, kw in travels:
            self.assertAlmostEqual(kw["target_z_mm"], -10.0)


class TestNeedleVolume(unittest.TestCase):
    def test_internal_volume_is_bore_cylinder(self):
        # id=200 µm (0.2 mm), length=0.5 in (12.7 mm).
        n = NeedleSpec(gauge=22, od_um=400.0, id_um=200.0, wall_um=100.0,
                       length_inches=0.5)
        expected = math.pi * (0.1 ** 2) * 12.7  # π(id/2)² · length, mm³ = µL
        self.assertAlmostEqual(n.internal_volume_uL, expected)
        # Uses the INNER bore, not the outer diameter.
        self.assertLess(n.internal_volume_uL,
                        math.pi * (0.2 ** 2) * 12.7)  # < outer-Ø cylinder


class TestPrepRoutine(unittest.TestCase):
    def _exec(self, needle_uL=0.40, cycles=2, **over):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
        ex.do_prep = True
        ex.prep_bore = "P1"
        ex.needle_volume_uL = needle_uL
        ex.oil_needles = 1.0
        ex.buffer_needles = 4.0
        ex.service_z_mm = -20.0
        ex.wash_cycles = cycles
        ex.wash_dwell_s = 0.0  # no real sleeps in the test
        ex.waste_well_pos = (60000.0, 30000.0)
        ex.oil_well_pos = (50000.0, 30000.0)
        ex.wash_well_pos = (40000.0, 30000.0)
        ex.buffer_well_pos = (30000.0, 30000.0)
        for k, v in over.items():
            setattr(ex, k, v)
        return ctrl, ex

    def test_prep_runs_before_pick_place_in_order(self):
        ctrl, ex = self._exec()
        q = OperationQueue(); q.add(_spheroid_op())
        self.assertTrue(ex.execute_queue(q))
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        # waste, oil, wash, buffer (prep), then pick, place (loop) = 6.
        self.assertEqual(len(travels), 6)
        xy = [(round(t[1]["target_x_um"]), round(t[1]["target_y_um"])) for t in travels]
        self.assertEqual(xy[:4], [(60000, 30000), (50000, 30000),
                                  (40000, 30000), (30000, 30000)])
        # The four prep travels dip to the service Z.
        for t in travels[:4]:
            self.assertAlmostEqual(t[1]["target_z_mm"], -20.0)
        # Pick/place keep their own heights.
        self.assertAlmostEqual(travels[4][1]["target_z_mm"], -16.0)
        self.assertAlmostEqual(travels[5][1]["target_z_mm"], -14.0)

    def test_prep_pump_volumes(self):
        ctrl, ex = self._exec(needle_uL=0.40)
        q = OperationQueue(); q.add(_spheroid_op())
        ex.execute_queue(q)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        # expel +1 needle oil, draw -1 needle oil, draw -4 needles buffer,
        # then the spheroid aspirate(-)/dispense(+).
        self.assertAlmostEqual(pumps[0][2], +0.40)   # waste expel
        self.assertAlmostEqual(pumps[1][2], -0.40)   # oil draw
        self.assertAlmostEqual(pumps[2][2], -1.60)   # 4 needles buffer
        self.assertEqual(pumps[0][1], "P1")

    def test_wash_jiggles_and_recenters(self):
        ctrl, ex = self._exec(cycles=3)
        q = OperationQueue(); q.add(_spheroid_op())
        ex.execute_queue(q)
        # One lift per cycle, always upward (height-frame positive).
        lifts = [c for c in ctrl.calls if c[0] == "move_z_user_relative"]
        self.assertEqual(len(lifts), 3)
        self.assertTrue(all(c[1] > 0 for c in lifts))
        # Each cycle returns to the EXACT dip Z (absolute), not a symmetric -amp.
        returns = [c for c in ctrl.calls
                   if c[0] == "move_z_absolute" and abs(c[1] - (-20.0)) < 1e-9]
        self.assertEqual(len(returns), 3)
        # XY jiggles: one per cycle + a final recenter over the wash well.
        xy = [c for c in ctrl.calls if c[0] == "move_xy_absolute_um"]
        self.assertGreaterEqual(len(xy), 3 + 1)
        self.assertEqual((round(xy[-1][1]), round(xy[-1][2])), (40000, 30000))

    def test_prep_disabled_runs_only_pick_place(self):
        ctrl, ex = self._exec()
        ex.do_prep = False
        q = OperationQueue(); q.add(_spheroid_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 2)  # pick + place only

    def test_missing_service_well_aborts_and_retracts(self):
        ctrl, ex = self._exec()
        ex.oil_well_pos = None  # oil unresolved
        q = OperationQueue(); q.add(_spheroid_op())
        with self.assertRaises(RuntimeError):
            ex.execute_queue(q)
        # Stopped at oil (waste travel happened, no buffer/pick), still retracted.
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 1)  # waste only
        self.assertTrue(any(c[0] == "ensure_retracted_to" for c in ctrl.calls))

    def test_prep_refuses_when_zp_disconnected(self):
        ctrl, ex = self._exec()
        ctrl.is_zp_connected = False
        q = OperationQueue(); q.add(_spheroid_op())
        with self.assertRaises(AbortException):
            ex.execute_queue(q)
        # No unretracted travel issued.
        self.assertEqual([c for c in ctrl.calls if c[0] == "safe_travel_to"], [])


if __name__ == "__main__":
    unittest.main()
