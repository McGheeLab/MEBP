"""test_v79_per_bore_cell_targeting.py — multi-bore Cell Targeting & Removal.

Covers the v7.9 rework driven by the operator's request:

  * needle assemblies with more than one bore (single / backpack / triple),
  * placing a CHOSEN bore on a target rather than "the needle",
  * a DEDICATED trypsin bore that doses the cell before the aspirate, with
    operator-controlled push volume, push rate and lead time,
  * per-BORE volume balance (the pre-v7.9 test summed across all pumps and so
    passed vacuously on an unbalanced two-bore operation),
  * the intra-well Z-polarity fix, which v7.9 is the first thing to activate.
"""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock

from SupportClasses.PhysicalModels import (
    NeedleSpec, NeedleBore, NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE,
)
from SupportClasses.PickAndPlaceManager import (
    BoreRole, BoreProgram, BORE_ROLE_ORDER, CellRemovalConfig, OperationType,
    PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
)


# ── Fakes ────────────────────────────────────────────────────────────

class _FakeCtrl:
    """Records every motion/pump call in order, like the v7.5.x suites do."""

    def __init__(self, z_up_sign=1.0):
        self.calls = []
        self._z_up_sign_val = z_up_sign
        self.is_zp_connected = True

    def z_up_sign(self):
        return self._z_up_sign_val

    def safe_travel_to(self, target_x_um=None, target_y_um=None, **k):
        self.calls.append(("safe_travel_to", round(target_x_um, 3),
                           round(target_y_um, 3), k.get("target_z_mm")))
        return True

    def move_xy_absolute_um(self, x_um, y_um):
        self.calls.append(("move_xy_absolute_um", round(x_um, 3), round(y_um, 3)))

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def wait_for_z_arrival(self, target=None, *a, **k):
        self.calls.append(("wait_for_z_arrival", target))
        return True

    def move_z_user_relative(self, dz):
        self.calls.append(("move_z_user_relative", round(dz, 6)))

    def move_z_relative(self, dz):
        self.calls.append(("move_z_relative", round(dz, 6)))

    def move_z_absolute(self, z, from_zero_ref=False):
        self.calls.append(("move_z_absolute", round(z, 6), from_zero_ref))

    def ensure_retracted_to(self, z, *a, **k):
        self.calls.append(("ensure_retracted_to", z))
        return True

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, **k):
        self.calls.append(("pump", pump, round(volume_uL, 8), rate_uL_s))

    # -- helpers ---------------------------------------------------------
    def pumps(self, pump=None):
        out = [(p, v, r) for (kind, p, v, r) in
               (c for c in self.calls if c[0] == "pump")]
        return [c for c in out if pump is None or c[0] == pump]

    def net_uL(self, pump):
        return round(sum(v for (p, v, _r) in self.pumps() if p == pump), 10)

    def kinds(self):
        return [c[0] for c in self.calls]


def _backpack():
    """Bore 0 = coarse (P1, datum). Bore 1 = fine (P2), offset +320 µm in X."""
    return NeedleSpec(
        needle_form=NEEDLE_FORM_BACKPACK,
        bores=[
            NeedleBore(gauge=22, od_um=718, id_um=413, wall_um=152,
                       length_mm=50.8, pump_id="P1", label="coarse"),
            NeedleBore(gauge=30, od_um=311, id_um=159, wall_um=76,
                       length_mm=50.8, pump_id="P2", label="fine",
                       offset_um=(320.0, -140.0), z_offset_mm=0.040),
        ],
    )


def _hw(needle):
    hw = MagicMock()
    hw.needle = needle
    return hw


def _exec(ctrl, needle=None, **attrs):
    ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
    ex.controller = ctrl
    ex.hw_config = _hw(needle) if needle is not None else None
    ex.on_sub_step = None
    ex.on_dwell_tick = None
    ex.safe_z_mm = 40.0
    ex.operating_z_mm = 10.0
    ex.pick_z_mm = 10.0
    ex.place_z_mm = 12.0
    ex.reagent_well_pos = (1000.0, 2000.0)
    ex.reagent_dip_z_mm = 15.0
    ex.wash_after_pickup = False
    ex.intra_well_retract_mm = 1.0
    ex.z_timeout_s = 5.0
    ex.xy_timeout_s = 5.0
    ex._current_well = None
    ex.bore_area_mm2 = 0.0
    ex.bore_profile = None
    ex._well_positions = {}
    import threading
    ex._abort_flag = threading.Event()
    ex._pause_event = threading.Event()
    ex._pause_event.set()
    for k, v in attrs.items():
        setattr(ex, k, v)
    return ex


def _cfg(**over):
    base = dict(
        reagent_bore="P1", release_depth_mm=0.10, release_volume_uL=0.012,
        extract_multiplier=2.0, dwell_time_s=0.0,
        push_speed_uL_s=0.5, pull_speed_uL_s=5.0,
        removal_z_offset_mm=0.10, place_z_offset_mm=0.50,
    )
    base.update(over)
    return CellRemovalConfig(**base)


def _op(cfg, well="A1"):
    src = PickPlaceTarget(target_id="P001", x_um=50000.0, y_um=60000.0,
                          well_name=well)
    dst = PickPlaceTarget(target_id="D001", x_um=70000.0, y_um=60000.0,
                          well_name=well)
    return PickPlaceOperation(op_id="OP1",
                              op_type=OperationType.CELL_TARGET_REMOVAL,
                              source_target=src, dest_target=dst, config=cfg)


# ── Bore roles ───────────────────────────────────────────────────────

class TestBoreRoles(unittest.TestCase):
    def test_idle_is_the_only_inactive_role(self):
        self.assertFalse(BoreRole.IDLE.is_active)
        for r in (BoreRole.ASPIRATE_TARGET, BoreRole.PUSH_REAGENT,
                  BoreRole.DISPENSE_PLACE):
            self.assertTrue(r.is_active)

    def test_role_order_doses_before_aspirating(self):
        self.assertLess(BORE_ROLE_ORDER.index(BoreRole.PUSH_REAGENT),
                        BORE_ROLE_ORDER.index(BoreRole.ASPIRATE_TARGET))
        self.assertLess(BORE_ROLE_ORDER.index(BoreRole.ASPIRATE_TARGET),
                        BORE_ROLE_ORDER.index(BoreRole.DISPENSE_PLACE))

    def test_program_round_trip(self):
        p = BoreProgram(bore_index=1, pump_id="P2",
                        role=BoreRole.PUSH_REAGENT,
                        target_type_id="tt-dbl", target_type_name="FITC+mCherry+",
                        target_type_color="#f5c2e7", rate_uL_s=0.25,
                        lead_time_s=30.0)
        back = BoreProgram.from_dict(p.to_dict())
        self.assertEqual(back, p)

    def test_unknown_role_degrades_to_idle_not_a_crash(self):
        """A bore whose purpose this build doesn't understand must do NOTHING."""
        p = BoreProgram.from_dict({"bore_index": 2, "role": "vortex_the_cell"})
        self.assertIs(p.role, BoreRole.IDLE)
        self.assertEqual(p.bore_index, 2)

    def test_unknown_keys_are_filtered(self):
        p = BoreProgram.from_dict({"bore_index": 1, "role": "idle",
                                   "some_future_key": 123})
        self.assertEqual(p.bore_index, 1)


# ── Bore offsets → stage XY ──────────────────────────────────────────

class TestBoreOffsetGeometry(unittest.TestCase):
    def test_datum_bore_lands_on_the_target_itself(self):
        ex = _exec(_FakeCtrl(), _backpack())
        t = PickPlaceTarget(target_id="T", x_um=50000.0, y_um=60000.0,
                            well_name="A1")
        self.assertEqual(ex._bore_target_xy_um(t, 0), (50000.0, 60000.0))

    def test_offset_bore_subtracts_its_offset(self):
        """stage = target - offset, so THAT bore ends up over the target."""
        ex = _exec(_FakeCtrl(), _backpack())
        t = PickPlaceTarget(target_id="T", x_um=50000.0, y_um=60000.0,
                            well_name="A1")
        self.assertEqual(ex._bore_target_xy_um(t, 1), (50000.0 - 320.0,
                                                       60000.0 + 140.0))

    def test_round_trip_puts_the_bore_back_on_the_target(self):
        """The sign convention: stage + offset == the target. A mis-signed
        offset lands the right DISTANCE on the WRONG SIDE, so pin it."""
        n = _backpack()
        ex = _exec(_FakeCtrl(), n)
        t = PickPlaceTarget(target_id="T", x_um=50000.0, y_um=60000.0,
                            well_name="A1")
        for k in range(n.bore_count):
            sx, sy = ex._bore_target_xy_um(t, k)
            ox, oy = n.bore_offset_um(k)
            self.assertAlmostEqual(sx + ox, t.x_um, places=9)
            self.assertAlmostEqual(sy + oy, t.y_um, places=9)

    def test_single_bore_needle_is_a_no_op(self):
        ex = _exec(_FakeCtrl(), NeedleSpec(gauge=27, id_um=210, od_um=413))
        t = PickPlaceTarget(target_id="T", x_um=1.0, y_um=2.0, well_name="A1")
        for k in (0, 1, 2, 99):
            self.assertEqual(ex._bore_target_xy_um(t, k), (1.0, 2.0))

    def test_no_needle_at_all_is_a_no_op(self):
        ex = _exec(_FakeCtrl(), None)
        t = PickPlaceTarget(target_id="T", x_um=5.0, y_um=6.0, well_name="A1")
        self.assertEqual(ex._bore_target_xy_um(t, 1), (5.0, 6.0))

    def test_longer_bore_gets_a_HIGHER_stage_z(self):
        """A bore that reaches lower must not be driven into the glass."""
        ex = _exec(_FakeCtrl(z_up_sign=1.0), _backpack())
        base = 10.0
        self.assertEqual(ex._bore_z_mm(base, 0), base)
        self.assertAlmostEqual(ex._bore_z_mm(base, 1), base + 0.040, places=9)

    def test_z_offset_follows_polarity(self):
        ex = _exec(_FakeCtrl(z_up_sign=-1.0), _backpack())
        self.assertAlmostEqual(ex._bore_z_mm(10.0, 1), 10.0 - 0.040, places=9)

    def test_z_offset_none_passes_through(self):
        ex = _exec(_FakeCtrl(), _backpack())
        self.assertIsNone(ex._bore_z_mm(None, 1))


# ── The intra-well polarity fix (v7.9) ───────────────────────────────

class TestIntraWellPolarity(unittest.TestCase):
    def test_lift_is_away_from_the_plate_on_z_up_sign_plus_one(self):
        """The pre-v7.9 code sent a RAW -1.0 mm here, which on z_up_sign=+1
        (both shipped profiles) drove the tip 1 mm DOWN into the glass from
        ~0.1 mm off the bottom."""
        ctrl = _FakeCtrl(z_up_sign=1.0)
        ex = _exec(ctrl, _backpack())
        ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        lifts = [c for c in ctrl.calls if c[0] == "move_z_user_relative"]
        self.assertTrue(lifts, "expected a HEIGHT-frame lift")
        self.assertGreater(lifts[0][1], 0.0, "lift must be AWAY from the plate")

    def test_no_raw_negative_z_move_is_ever_emitted(self):
        for sign in (1.0, -1.0):
            ctrl = _FakeCtrl(z_up_sign=sign)
            ex = _exec(ctrl, _backpack())
            ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
            raw = [c for c in ctrl.calls if c[0] == "move_z_relative"]
            self.assertEqual(raw, [], f"raw Z deltas used at z_up_sign={sign}")

    def test_lift_precedes_the_xy_move(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, _backpack())
        ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        kinds = ctrl.kinds()
        self.assertLess(kinds.index("move_z_user_relative"),
                        kinds.index("move_xy_absolute_um"),
                        "Z must be retracted BEFORE the XY move")

    def test_return_is_absolute_so_the_tip_cannot_walk_down(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, _backpack())
        ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        abs_moves = [c for c in ctrl.calls if c[0] == "move_z_absolute"]
        self.assertEqual(abs_moves, [("move_z_absolute", 10.0, True)])

    def test_zero_amplitude_never_descends(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, _backpack(), intra_well_retract_mm=0.0)
        ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        self.assertNotIn("move_z_user_relative", ctrl.kinds())
        self.assertNotIn("move_z_relative", ctrl.kinds())
        self.assertIn("move_xy_absolute_um", ctrl.kinds())

    def test_fallback_controller_without_user_relative_still_lifts_correctly(self):
        class _Old(_FakeCtrl):
            move_z_user_relative = None

            def __getattribute__(self, name):
                if name == "move_z_user_relative":
                    raise AttributeError(name)
                return object.__getattribute__(self, name)

        ctrl = _Old(z_up_sign=1.0)
        ex = _exec(ctrl, _backpack())
        ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        raw = [c for c in ctrl.calls if c[0] == "move_z_relative"]
        self.assertTrue(raw)
        self.assertGreater(raw[0][1], 0.0,
                           "raw fallback must be scaled by z_up_sign, not negated")


# ── The trypsin bore ─────────────────────────────────────────────────

class TestTrypsinBore(unittest.TestCase):
    def _run(self, **over):
        ctrl = _FakeCtrl()
        n = _backpack()
        kw = dict(trypsin_enabled=True, trypsin_bore="P2",
                  trypsin_bore_index=1, trypsin_depth_mm=0.10,
                  trypsin_push_rate_uL_s=0.25, trypsin_lead_time_s=0.0,
                  aspirate_bore_index=0)
        kw.update(over)
        cfg = _cfg(**kw)
        ex = _exec(ctrl, n, _well_positions={"__trypsin__": (500.0, 600.0)})
        ex._execute_cell_removal(_op(cfg))
        return ctrl, n, cfg

    def test_disabled_by_default_touches_only_one_pump(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, _backpack())
        ex._execute_cell_removal(_op(_cfg()))
        self.assertEqual({p for (p, _v, _r) in ctrl.pumps()}, {"P1"})

    def test_trypsin_volume_uses_its_OWN_bore_area(self):
        """A backpack's bores differ in diameter, so one area cannot size both."""
        n = _backpack()
        cfg = _cfg(trypsin_enabled=True, trypsin_bore="P2",
                   trypsin_bore_index=1, trypsin_depth_mm=0.10)
        expected = n.bore(1).orifice_area_mm2 * 0.10
        self.assertAlmostEqual(cfg.compute_trypsin_volume_uL(n), expected,
                               places=12)
        # and it is NOT the aspirating bore's number
        self.assertNotAlmostEqual(cfg.compute_trypsin_volume_uL(n),
                                  n.bore(0).orifice_area_mm2 * 0.10, places=9)

    def test_explicit_volume_overrides_the_depth(self):
        n = _backpack()
        cfg = _cfg(trypsin_enabled=True, trypsin_bore="P2",
                   trypsin_volume_uL=0.5, trypsin_depth_mm=0.10)
        self.assertEqual(cfg.compute_trypsin_volume_uL(n), 0.5)

    def test_disabled_reports_zero_volume(self):
        self.assertEqual(_cfg().compute_trypsin_volume_uL(_backpack()), 0.0)

    def test_push_then_shift_then_aspirate_in_that_order(self):
        ctrl, n, cfg = self._run()
        kinds = [(c[0], c[1] if len(c) > 1 else None) for c in ctrl.calls]
        pump_idx = [i for i, c in enumerate(ctrl.calls) if c[0] == "pump"]
        # find the trypsin push (+ on P2) and the cell pull (- on P1)
        push_i = next(i for i in pump_idx
                      if ctrl.calls[i][1] == "P2" and ctrl.calls[i][2] > 0)
        pull_i = next(i for i in pump_idx
                      if ctrl.calls[i][1] == "P1" and ctrl.calls[i][2] < 0
                      and i > push_i)
        shift_i = next(i for i, c in enumerate(ctrl.calls)
                       if c[0] == "move_xy_absolute_um" and i > push_i)
        self.assertLess(push_i, shift_i, "must push trypsin before shifting")
        self.assertLess(shift_i, pull_i, "must shift the aspirating bore on "
                                        "target before pulling the cell")

    def test_the_shift_moves_by_the_inter_bore_spacing(self):
        ctrl, n, cfg = self._run()
        # The shift is an intra-well XY move to (target - offset(bore 0)).
        shifts = [c for c in ctrl.calls if c[0] == "move_xy_absolute_um"]
        self.assertTrue(shifts)
        self.assertAlmostEqual(shifts[0][1], 50000.0, places=6)
        self.assertAlmostEqual(shifts[0][2], 60000.0, places=6)

    def test_trypsin_bore_is_parked_on_the_target_for_the_push(self):
        ctrl, n, cfg = self._run()
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        # the travel to the source with the trypsin bore offset applied
        ox, oy = n.bore_offset_um(1)
        self.assertTrue(
            any(abs(t[1] - (50000.0 - ox)) < 1e-6 and
                abs(t[2] - (60000.0 - oy)) < 1e-6 for t in travels),
            f"no travel placed bore 2 on the target; travels={travels}")

    def test_per_bore_volume_balance(self):
        """v7.9: balance must be checked PER BORE. The pre-v7.9 test summed
        across all pumps, so +x on one bore and -x on another cancelled and an
        unbalanced two-bore op passed vacuously."""
        ctrl, n, cfg = self._run()
        self.assertAlmostEqual(ctrl.net_uL("P1"), 0.0, places=10)
        self.assertAlmostEqual(ctrl.net_uL("P2"), 0.0, places=10)
        # and prove the vacuous version would have been fooled: both pumps moved
        self.assertTrue(ctrl.pumps("P1") and
                        [c for c in ctrl.pumps() if c[0] == "P2"])

    def test_trypsin_push_uses_its_own_rate(self):
        ctrl, n, cfg = self._run(trypsin_push_rate_uL_s=0.25)
        p2 = [c for c in ctrl.pumps() if c[0] == "P2"]
        self.assertTrue(p2)
        for (_p, _v, rate) in p2:
            self.assertEqual(rate, 0.25)

    def test_lead_time_is_a_true_no_op_at_zero(self):
        """Every executor suite sets dwells to 0 to avoid real sleeps."""
        import time
        t0 = time.monotonic()
        self._run(trypsin_lead_time_s=0.0)
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_unsized_trypsin_push_is_skipped_not_silently_zero(self):
        ctrl = _FakeCtrl()
        n = NeedleSpec(gauge=27, id_um=0.0, od_um=0.0)   # no usable area
        cfg = _cfg(trypsin_enabled=True, trypsin_bore="P2",
                   trypsin_bore_index=1, trypsin_depth_mm=0.10,
                   trypsin_volume_uL=0.0)
        ex = _exec(ctrl, n, _well_positions={"__trypsin__": (1.0, 2.0)})
        ex._execute_cell_removal(_op(cfg))
        self.assertEqual([c for c in ctrl.pumps() if c[0] == "P2"], [])

    def test_missing_trypsin_well_refuses_rather_than_running_dry(self):
        ctrl = _FakeCtrl()
        cfg = _cfg(trypsin_enabled=True, trypsin_bore="P2",
                   trypsin_bore_index=1)
        ex = _exec(ctrl, _backpack(), _well_positions={})
        with self.assertRaises(RuntimeError):
            ex._execute_cell_removal(_op(cfg))


# ── The volume-guard asymmetry fix ───────────────────────────────────

class TestVolumeGuardSymmetry(unittest.TestCase):
    def test_no_reagent_well_means_no_push_and_a_balanced_pump(self):
        """Pre-v7.9 the LOAD was gated on having a reagent well but the PUSH was
        not, so the executor dispensed a volume it had never aspirated."""
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, _backpack(), reagent_well_pos=None)
        ex._execute_cell_removal(_op(_cfg()))
        self.assertAlmostEqual(ctrl.net_uL("P1"), 0.0, places=10)

    def test_with_a_reagent_well_the_push_still_happens(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, _backpack())
        ex._execute_cell_removal(_op(_cfg()))
        p1 = ctrl.pumps("P1")
        self.assertTrue(any(v > 0 for (_p, v, _r) in ctrl.pumps()
                            if _p == "P1"))
        self.assertAlmostEqual(ctrl.net_uL("P1"), 0.0, places=10)


# ── Legacy single-bore path is unchanged ─────────────────────────────

class TestSingleBoreUnchanged(unittest.TestCase):
    def test_single_bore_uses_exactly_one_pump_and_balances(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, NeedleSpec(gauge=27, id_um=210, od_um=413,
                                    length_inches=2.0))
        ex._execute_cell_removal(_op(_cfg()))
        self.assertEqual({p for (p, _v, _r) in ctrl.pumps()}, {"P1"})
        self.assertAlmostEqual(ctrl.net_uL("P1"), 0.0, places=10)

    def test_no_bore_shift_without_a_trypsin_bore(self):
        """No extra positioning move is introduced for a single-bore needle.

        Source and dest are put in DIFFERENT wells so the pre-existing
        same-named-well intra-well shortcut (which legitimately emits an XY move)
        does not mask the thing under test: v7.9 must add no bore shift.
        """
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, NeedleSpec(gauge=27, id_um=210, od_um=413))
        src = PickPlaceTarget(target_id="P001", x_um=50000.0, y_um=60000.0,
                              well_name="A1")
        dst = PickPlaceTarget(target_id="D001", x_um=70000.0, y_um=60000.0,
                              well_name="B2")
        op = PickPlaceOperation(op_id="OP1",
                                op_type=OperationType.CELL_TARGET_REMOVAL,
                                source_target=src, dest_target=dst,
                                config=_cfg())
        ex._execute_cell_removal(op)
        self.assertNotIn("move_xy_absolute_um", ctrl.kinds())

    def test_shift_helper_reports_no_move_for_a_single_bore(self):
        ctrl = _FakeCtrl()
        ex = _exec(ctrl, NeedleSpec(gauge=27, id_um=210, od_um=413))
        t = PickPlaceTarget(target_id="T", x_um=1.0, y_um=2.0, well_name="A1")
        self.assertFalse(ex._shift_to_bore(_op(_cfg()), t, 0, 1, 10.0))
        self.assertEqual(ctrl.calls, [])


if __name__ == "__main__":
    unittest.main()


# ── Simultaneous multi-bore prep (operator decision D8) ──────────────

class TestSimultaneousPrep(unittest.TestCase):
    """D8: "only the bores this run uses. but the should be doing the same
    thing simultaneously"."""

    def _ex(self, ctrl, prep_bores):
        ex = _exec(ctrl, _backpack(),
                   prep_bore="P1", prep_rate_uL_s=1.0, needle_volume_uL=5.0,
                   oil_needles=1.0, buffer_needles=2.0, service_z_mm=20.0,
                   wash_cycles=0, wash_xy_amplitude_um=0.0,
                   wash_z_amplitude_mm=0.0, wash_dwell_s=0.0,
                   prep_bores=prep_bores,
                   waste_well_pos=(1.0, 2.0), oil_well_pos=(1.0, 2.0),
                   wash_well_pos=(1.0, 2.0), buffer_well_pos=(1.0, 2.0),
                   _well_positions={k: (1.0, 2.0) for k in
                                    ("__waste__", "__oil__", "__wash__",
                                     "__buffer__")})
        return ex

    def test_legacy_path_when_no_bores_declared(self):
        ctrl = _FakeCtrl()
        self._ex(ctrl, []).run_prep()
        self.assertEqual({p for (p, _v, _r) in ctrl.pumps()}, {"P1"})
        self.assertEqual(getattr(ctrl, "coordinated", []), [])

    def test_both_bores_conditioned_in_ONE_coordinated_move(self):
        """Not two G0 blocks — Marlin runs blocks in order, which would be
        serialization, the opposite of what was asked for."""
        calls = []

        class _C(_FakeCtrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                calls.append(dict(volumes))
                return True

        ctrl = _C()
        self._ex(ctrl, [{"pump_id": "P1", "bore_index": 0},
                        {"pump_id": "P2", "bore_index": 1}]).run_prep()
        self.assertEqual(len(calls), 3, "oil-out, oil-in, buffer-in")
        for v in calls:
            self.assertEqual(set(v), {"P1", "P2"},
                             "both bores must move in the SAME G0")
        # no single-pump moves leaked out
        self.assertEqual(ctrl.pumps(), [])

    def test_each_bore_gets_its_OWN_needle_volume(self):
        """A backpack's bores hold very different volumes; one scalar cannot
        size both."""
        calls = []

        class _C(_FakeCtrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                calls.append(dict(volumes))
                return True

        ctrl = _C()
        n = _backpack()
        self._ex(ctrl, [{"pump_id": "P1", "bore_index": 0},
                        {"pump_id": "P2", "bore_index": 1}]).run_prep()
        v0 = n.bore(0).internal_volume_uL
        v1 = n.bore(1).internal_volume_uL
        self.assertGreater(v0, v1 * 2, "test needs genuinely different bores")
        self.assertAlmostEqual(calls[0]["P1"], +v0, places=9)
        self.assertAlmostEqual(calls[0]["P2"], +v1, places=9)
        # buffer step uses buffer_needles = 2
        self.assertAlmostEqual(calls[2]["P1"], -2.0 * v0, places=9)
        self.assertAlmostEqual(calls[2]["P2"], -2.0 * v1, places=9)

    def test_signs_are_dispense_then_aspirate_then_aspirate(self):
        calls = []

        class _C(_FakeCtrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                calls.append(dict(volumes))
                return True

        ctrl = _C()
        self._ex(ctrl, [{"pump_id": "P1", "bore_index": 0},
                        {"pump_id": "P2", "bore_index": 1}]).run_prep()
        self.assertTrue(all(v > 0 for v in calls[0].values()), "oil→waste dispenses")
        self.assertTrue(all(v < 0 for v in calls[1].values()), "oil aspirates")
        self.assertTrue(all(v < 0 for v in calls[2].values()), "buffer aspirates")

    def test_only_the_declared_bores_are_conditioned(self):
        calls = []

        class _C(_FakeCtrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                calls.append(dict(volumes))
                return True

        ctrl = _C()
        self._ex(ctrl, [{"pump_id": "P2", "bore_index": 1}]).run_prep()
        for v in calls:
            self.assertEqual(set(v), {"P2"})

    def test_duplicate_pump_is_collapsed(self):
        """One pump cannot be driven twice in a single coordinated move."""
        ex = self._ex(_FakeCtrl(), [{"pump_id": "P1", "bore_index": 0},
                                    {"pump_id": "p1", "bore_index": 0}])
        self.assertEqual(len(ex._prep_bore_plan()), 1)

    def test_falls_back_to_sequential_on_an_older_controller(self):
        """Degrade to correct-but-not-simultaneous, never to failing."""
        ctrl = _FakeCtrl()   # has no move_pumps_uL
        self._ex(ctrl, [{"pump_id": "P1", "bore_index": 0},
                        {"pump_id": "P2", "bore_index": 1}]).run_prep()
        self.assertEqual({p for (p, _v, _r) in ctrl.pumps()}, {"P1", "P2"})


class TestSimultaneousPostClean(unittest.TestCase):
    """run_post_clean mirrors run_prep, including the v7.9 simultaneous
    treatment — a trypsin bore's residual must be cleared too."""

    def _ex(self, ctrl, prep_bores):
        ex = _exec(ctrl, _backpack(),
                   prep_bore="P1", prep_rate_uL_s=1.0, needle_volume_uL=5.0,
                   post_dispense_needles=6.0, buffer_needles=1.0,
                   service_z_mm=20.0, wash_cycles=0, wash_xy_amplitude_um=0.0,
                   wash_z_amplitude_mm=0.0, wash_dwell_s=0.0,
                   prep_bores=prep_bores,
                   waste_well_pos=(1.0, 2.0), oil_well_pos=(1.0, 2.0),
                   wash_well_pos=(1.0, 2.0), buffer_well_pos=(1.0, 2.0),
                   _well_positions={k: (1.0, 2.0) for k in
                                    ("__waste__", "__oil__", "__wash__",
                                     "__buffer__")})
        return ex

    def test_legacy_path_unchanged(self):
        ctrl = _FakeCtrl()
        self._ex(ctrl, []).run_post_clean()
        self.assertEqual({p for (p, _v, _r) in ctrl.pumps()}, {"P1"})

    def test_both_bores_cleared_simultaneously(self):
        calls = []

        class _C(_FakeCtrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                calls.append(dict(volumes))
                return True

        ctrl = _C()
        self._ex(ctrl, [{"pump_id": "P1", "bore_index": 0},
                        {"pump_id": "P2", "bore_index": 1}]).run_post_clean()
        self.assertEqual(len(calls), 2, "waste-dispense, buffer-aspirate")
        for v in calls:
            self.assertEqual(set(v), {"P1", "P2"})
        self.assertTrue(all(v > 0 for v in calls[0].values()))
        self.assertTrue(all(v < 0 for v in calls[1].values()))
