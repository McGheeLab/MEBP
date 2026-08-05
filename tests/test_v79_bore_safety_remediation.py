"""test_v79_bore_safety_remediation.py — the v7.9 hardware-safety fixes.

Each class here pins ONE defect found by the post-v7.9 audit. All of them were
reachable with the v7.9 suite fully green, so every test states the physical
consequence it prevents.

  D5a  a non-finite bore offset survived into `SafetyLimits.clamp_xy`, whose
       `max(lo, min(hi, nan))` returns `hi` — a NaN offset commanded the stage to
       the FAR CORNER of the travel envelope with the needle down.
  D5b  the offset magnitude was unbounded, so a mis-measured 5 mm offset drove
       an in-well XY move into the well wall.
  D2   the descend was planned against the bore being placed, not the LOWEST
       bore, so a protruding bore was driven into the glass; and the
       plate-bottom floor was never armed for pick & place, so nothing caught it.
  D4   `_shift_to_bore` called `_intra_well_move` directly, bypassing the
       ZP-disconnect guard — a mid-shift board drop dragged the needle laterally
       at ~0.1 mm above the plate.
  D3   `_intra_well_move` passed ABSOLUTE mm to a ZERO-REF comparison, so the XY
       arrival wait could never succeed: +30 s per shift, and the Z descent ran
       with XY unconfirmed. Its return value was discarded.
"""

from __future__ import annotations

import math
import unittest
from unittest.mock import MagicMock

from SupportClasses.PhysicalModels import (
    NeedleSpec, NeedleBore, NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE,
    MAX_BORE_OFFSET_UM, MAX_BORE_Z_OFFSET_MM,
    needle_bore_offset_um, needle_bore_z_offset_mm, needle_max_bore_z_offset_mm,
)
from SupportClasses.PickAndPlaceManager import (
    AbortException, CellRemovalConfig, OperationQueue, OperationType,
    PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
)


# ── Fakes ────────────────────────────────────────────────────────────

class _Ctrl:
    """Records calls; models the real frames (zero_position!) faithfully."""

    def __init__(self, *, z_up_sign=1.0, zero=(20000.0, 30000.0),
                 xy_ok=True, z_ok=True, zp=True):
        self.calls = []
        self._z_up_sign_val = z_up_sign
        self.zero_position = {"x": zero[0], "y": zero[1], "Z": 0.0}
        self.is_zp_connected = zp
        self._xy_ok = xy_ok
        self._z_ok = z_ok
        self.floor_calls = []

    def z_up_sign(self):
        return self._z_up_sign_val

    def set_print_floor_active(self, active):
        self.floor_calls.append(bool(active))

    def safe_travel_to(self, target_x_um=None, target_y_um=None, **k):
        self.calls.append(("safe_travel_to", round(target_x_um, 3),
                           round(target_y_um, 3), k.get("target_z_mm")))
        return True

    def move_xy_absolute_um(self, x_um, y_um):
        self.calls.append(("move_xy_absolute_um", round(x_um, 3), round(y_um, 3)))

    def wait_for_xy_arrival(self, x_mm, y_mm, tolerance_mm=0.1, timeout_s=30.0):
        self.calls.append(("wait_for_xy_arrival", round(x_mm, 6), round(y_mm, 6),
                           tolerance_mm))
        return self._xy_ok

    def wait_for_z_arrival(self, target=None, *a, **k):
        self.calls.append(("wait_for_z_arrival", target))
        return self._z_ok

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

    def kinds(self):
        return [c[0] for c in self.calls]


def _needle(*, offsets=((0.0, 0.0), (320.0, -140.0)), z_offsets=(0.0, 0.040)):
    bores = []
    for k, (off, dz) in enumerate(zip(offsets, z_offsets)):
        bores.append(NeedleBore(gauge=22 if k == 0 else 30,
                                od_um=718 if k == 0 else 311,
                                id_um=413 if k == 0 else 159,
                                length_mm=50.8, pump_id=f"P{k + 1}",
                                offset_um=off, z_offset_mm=dz))
    form = NEEDLE_FORM_BACKPACK if len(bores) == 2 else NEEDLE_FORM_TRIPLE
    return NeedleSpec(needle_form=form, bores=bores)


def _single():
    return NeedleSpec(gauge=27, id_um=210, od_um=413, length_inches=2.0)


def _exec(ctrl, needle=None, **attrs):
    ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
    ex.controller = ctrl
    hw = MagicMock()
    hw.needle = needle
    ex.hw_config = hw if needle is not None else None
    ex.on_sub_step = None
    ex.on_dwell_tick = None
    ex.on_op_started = None
    ex.on_op_completed = None
    ex.on_op_failed = None
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
    ex.do_prep = False
    ex.do_post_clean = False
    import threading
    ex._abort_flag = threading.Event()
    ex._pause_event = threading.Event()
    ex._pause_event.set()
    for k, v in attrs.items():
        setattr(ex, k, v)
    return ex


def _target(x=50000.0, y=60000.0, well="A1"):
    return PickPlaceTarget(target_id="T1", x_um=x, y_um=y, well_name=well)


# ── D5a: non-finite offsets ──────────────────────────────────────────

class TestNonFiniteOffsetsCollapse(unittest.TestCase):
    """A NaN offset must never reach clamp_xy: it returns the envelope MAXIMUM."""

    def test_nan_offset_collapses_to_the_datum_on_construction(self):
        b = NeedleBore(gauge=30, id_um=159, od_um=311,
                       offset_um=(float("nan"), 5.0))
        self.assertEqual(b.offset_um, (0.0, 0.0))

    def test_inf_offset_collapses_to_the_datum(self):
        b = NeedleBore(gauge=30, id_um=159, od_um=311,
                       offset_um=(float("inf"), float("-inf")))
        self.assertEqual(b.offset_um, (0.0, 0.0))

    def test_nan_z_offset_collapses(self):
        b = NeedleBore(gauge=30, id_um=159, od_um=311,
                       z_offset_mm=float("nan"))
        self.assertEqual(b.z_offset_mm, 0.0)

    def test_the_clamp_hazard_is_real_so_the_guard_matters(self):
        """Documents WHY: min/max silently pass NaN through as the bound."""
        lo, hi = -130000.0, 130000.0
        self.assertEqual(max(lo, min(hi, float("nan"))), hi)

    def test_reader_refuses_a_mock_which_would_otherwise_report_1um(self):
        """float(MagicMock()) is 1.0 — a stub must not claim an offset."""
        n = MagicMock()
        n.bore_offset_um = MagicMock(return_value=(MagicMock(), MagicMock()))
        self.assertEqual(needle_bore_offset_um(n, 1), (0.0, 0.0))

    def test_reader_refuses_bool_and_non_numeric(self):
        b = MagicMock()
        b.offset_um = (True, "12")
        n = MagicMock()
        n.bore_offset_um = None
        n.bores_resolved = MagicMock(return_value=[b, b])
        n.bore = MagicMock(return_value=b)
        self.assertEqual(needle_bore_offset_um(n, 1), (0.0, 0.0))

    def test_max_z_offset_is_never_negative(self):
        """The clearance guarantee is raise-only only if this can't go negative."""
        n = _needle(z_offsets=(0.0, -0.5))
        self.assertGreaterEqual(needle_max_bore_z_offset_mm(n), 0.0)

    def test_real_values_still_pass_through(self):
        n = _needle()
        self.assertEqual(needle_bore_offset_um(n, 1), (320.0, -140.0))
        self.assertAlmostEqual(needle_bore_z_offset_mm(n, 1), 0.040, places=9)


# ── D5b: implausible offset magnitude ────────────────────────────────

class TestOffsetMagnitudeBound(unittest.TestCase):
    def test_a_five_millimetre_offset_is_refused_before_any_motion(self):
        """A 5 mm in-well move would hit the well wall — refuse, don't clamp."""
        ctrl = _Ctrl()
        ex = _exec(ctrl, _needle(offsets=((0.0, 0.0), (5000.0, 0.0))))
        with self.assertRaises(AbortException) as cm:
            ex._bore_offset_um(1)
        self.assertEqual(ctrl.calls, [], "must refuse BEFORE commanding motion")
        self.assertIn("Needle Location", str(cm.exception))
        self.assertIn("5000", str(cm.exception))

    def test_a_realistic_offset_is_accepted(self):
        ex = _exec(_Ctrl(), _needle(offsets=((0.0, 0.0), (400.0, -250.0))))
        self.assertEqual(ex._bore_offset_um(1), (400.0, -250.0))

    def test_the_bound_is_generous_enough_for_a_real_triple(self):
        """A fused triple spans ~2.6 mm end-to-end; ~1.3 mm from the datum."""
        self.assertGreaterEqual(MAX_BORE_OFFSET_UM, 1300.0)

    def test_travel_planning_is_also_covered_not_just_the_shift(self):
        """_bore_target_xy_um shares the same read, so parking is guarded too."""
        ex = _exec(_Ctrl(), _needle(offsets=((0.0, 0.0), (9000.0, 0.0))))
        with self.assertRaises(AbortException):
            ex._bore_target_xy_um(_target(), 1)

    def test_an_implausible_z_offset_is_refused_before_descending(self):
        ctrl = _Ctrl()
        ex = _exec(ctrl, _needle(z_offsets=(0.0, 25.0)))
        with self.assertRaises(AbortException) as cm:
            ex._descend_z_mm(10.0, 1)
        self.assertEqual(ctrl.calls, [])
        self.assertIn("Needle Location", str(cm.exception))

    def test_the_z_bound_is_generous_versus_measured_coplanarity(self):
        """Measured coplanarity is ~50 µm; the bound must not fire on that."""
        self.assertGreater(MAX_BORE_Z_OFFSET_MM, 0.05 * 10)


# ── D2: descend planned against the LOWEST bore ──────────────────────

class TestDescendClearsEveryBore(unittest.TestCase):
    def test_datum_descend_accounts_for_a_longer_second_bore(self):
        """THE GLASS BUG. Bore 1 reaches 0.040 mm lower; lowering the datum to a
        0.10 mm clearance previously put bore 1 at 0.060 mm and, with a larger
        offset, straight through the plate."""
        ex = _exec(_Ctrl(z_up_sign=1.0), _needle(z_offsets=(0.0, 0.040)))
        self.assertAlmostEqual(ex._descend_z_mm(10.0, 0), 10.040, places=9)

    def test_no_bore_is_ever_computed_below_the_plate_bottom(self):
        """With a 0.20 mm protrusion and a 0.10 mm clearance, the old code put the
        long bore 0.10 mm INTO the glass. Check every bore, both polarities."""
        for sign in (1.0, -1.0):
            n = _needle(z_offsets=(0.0, 0.20))
            ex = _exec(_Ctrl(z_up_sign=sign), n)
            plate_bottom = 0.0
            clearance = 0.10
            base = plate_bottom + sign * clearance
            for k in range(n.bore_count):
                stage_z = ex._descend_z_mm(base, k)
                for j in range(n.bore_count):
                    dz = needle_bore_z_offset_mm(n, j)
                    tip_h = sign * (stage_z - sign * dz - plate_bottom)
                    self.assertGreaterEqual(
                        round(tip_h, 9), 0.0,
                        f"sign={sign} placing bore {k}: bore {j} tip is "
                        f"{tip_h:.4f} mm relative to the plate bottom")

    def test_requested_bore_gets_the_clearance_when_it_IS_the_lowest(self):
        ex = _exec(_Ctrl(z_up_sign=1.0), _needle(z_offsets=(0.0, 0.040)))
        self.assertAlmostEqual(ex._descend_z_mm(10.0, 1), 10.040, places=9)

    def test_polarity(self):
        ex = _exec(_Ctrl(z_up_sign=-1.0), _needle(z_offsets=(0.0, 0.040)))
        self.assertAlmostEqual(ex._descend_z_mm(10.0, 0), 10.0 - 0.040, places=9)

    def test_single_bore_needle_is_exactly_a_no_op(self):
        """The byte-identity guarantee for every pre-v7.9 setup."""
        ex = _exec(_Ctrl(), _single())
        for k in (0, 1, 2, 99):
            self.assertEqual(ex._descend_z_mm(10.0, k), 10.0)
            self.assertEqual(ex._descend_z_mm(-3.25, k), -3.25)

    def test_no_needle_at_all_is_a_no_op(self):
        ex = _exec(_Ctrl(), None)
        self.assertEqual(ex._descend_z_mm(7.5, 1), 7.5)

    def test_none_passes_through(self):
        ex = _exec(_Ctrl(), _needle())
        self.assertIsNone(ex._descend_z_mm(None, 1))

    def test_it_is_raise_only_never_a_descent(self):
        """max(dz_k, dz_max) >= 0 always, so the stage only ever goes UP."""
        for z_offsets in ((0.0, 0.04), (0.0, -0.04), (0.0, 0.0)):
            for sign in (1.0, -1.0):
                ex = _exec(_Ctrl(z_up_sign=sign), _needle(z_offsets=z_offsets))
                for k in (0, 1):
                    got = ex._descend_z_mm(10.0, k)
                    height_delta = sign * (got - 10.0)
                    self.assertGreaterEqual(round(height_delta, 9), 0.0)

    def test_the_helper_it_replaced_still_answers_its_own_question(self):
        """_bore_z_mm keeps its honest single meaning; it was not repurposed."""
        ex = _exec(_Ctrl(z_up_sign=1.0), _needle(z_offsets=(0.0, 0.040)))
        self.assertEqual(ex._bore_z_mm(10.0, 0), 10.0)
        self.assertAlmostEqual(ex._bore_z_mm(10.0, 1), 10.040, places=9)


class TestPlateFloorArmedForPickAndPlace(unittest.TestCase):
    def _queue(self):
        q = OperationQueue()
        cfg = CellRemovalConfig(reagent_bore="P1", release_depth_mm=0.10,
                                dwell_time_s=0.0)
        q.add(PickPlaceOperation(
            op_id="OP1", op_type=OperationType.CELL_TARGET_REMOVAL,
            source_target=_target(), dest_target=_target(x=70000.0),
            config=cfg))
        return q

    def test_armed_for_the_run_and_disarmed_on_completion(self):
        ctrl = _Ctrl()
        ex = _exec(ctrl, _single())
        ex.execute_queue(self._queue())
        self.assertEqual(ctrl.floor_calls, [True, False])

    def test_disarmed_even_when_an_operation_raises(self):
        ctrl = _Ctrl()
        ex = _exec(ctrl, _single())
        ex._execute_operation = MagicMock(side_effect=RuntimeError("boom"))
        ex.execute_queue(self._queue())
        self.assertEqual(ctrl.floor_calls, [True, False])

    def test_disarmed_on_abort(self):
        ctrl = _Ctrl()
        ex = _exec(ctrl, _single())
        ex._execute_operation = MagicMock(side_effect=AbortException("stop"))
        ex.execute_queue(self._queue())
        self.assertEqual(ctrl.floor_calls, [True, False])

    def test_a_controller_without_the_method_still_runs(self):
        """An older controller / partial stub must not break the run."""
        class _Older(_Ctrl):
            set_print_floor_active = None      # not callable → skipped
        ctrl = _Older()
        ex = _exec(ctrl, _single())
        self.assertTrue(ex.execute_queue(self._queue()))
        self.assertEqual(ctrl.floor_calls, [], "nothing should have been armed")


# ── D4: ZP-disconnect guard in the primitive ─────────────────────────

class TestZpGuardInIntraWellMove(unittest.TestCase):
    def test_a_dropped_board_refuses_the_in_well_move(self):
        ctrl = _Ctrl(zp=False)
        ex = _exec(ctrl, _needle())
        with self.assertRaises(AbortException) as cm:
            ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        self.assertEqual(ctrl.calls, [], "must emit NOTHING")
        self.assertIn("ZP board", str(cm.exception))

    def test_the_shift_path_is_covered_too(self):
        """_shift_to_bore calls _intra_well_move DIRECTLY, bypassing
        _safe_move_to's guard — which is why the guard moved into the primitive."""
        ctrl = _Ctrl(zp=False)
        ex = _exec(ctrl, _needle())
        op = MagicMock()
        with self.assertRaises(AbortException):
            ex._shift_to_bore(op, _target(), 1, 0, 10.0)
        self.assertNotIn("move_xy_absolute_um", ctrl.kinds())

    def test_a_board_that_drops_DURING_the_lift_still_refuses_the_xy(self):
        """wait_for_z_arrival returns True when disconnected, so the confirm
        cannot fail closed on its own — the flag is re-read before the XY."""
        ctrl = _Ctrl()

        real_wait = ctrl.wait_for_z_arrival

        def drop_then_confirm(target=None, *a, **k):
            ctrl.is_zp_connected = False
            return real_wait(target, *a, **k)
        ctrl.wait_for_z_arrival = drop_then_confirm

        ex = _exec(ctrl, _needle())
        with self.assertRaises(AbortException) as cm:
            ex._intra_well_move(1000.0, 2000.0, target_z_mm=10.0)
        self.assertNotIn("move_xy_absolute_um", ctrl.kinds())
        self.assertIn("dropped", str(cm.exception))


# ── D3: XY frame + acted-on confirms ─────────────────────────────────

class TestIntraWellXyFrameAndConfirms(unittest.TestCase):
    def test_the_wait_receives_ZERO_REF_mm_not_absolute_mm(self):
        """THE 30-SECONDS-PER-CELL BUG. The wait compares against
        get_xy_position_mm, which is zero-ref; passing absolute mm meant it could
        never succeed on any machine whose zero is away from stage home."""
        ctrl = _Ctrl(zero=(20000.0, 30000.0))
        ex = _exec(ctrl, _needle())
        ex._intra_well_move(50000.0, 60000.0, target_z_mm=10.0)
        waits = [c for c in ctrl.calls if c[0] == "wait_for_xy_arrival"]
        self.assertEqual(len(waits), 1)
        self.assertAlmostEqual(waits[0][1], (50000.0 - 20000.0) / 1000.0, places=9)
        self.assertAlmostEqual(waits[0][2], (60000.0 - 30000.0) / 1000.0, places=9)

    def test_the_old_absolute_mm_form_would_have_been_wrong_by_the_zero(self):
        """Pin the magnitude of the bug so a regression is unmistakable."""
        ctrl = _Ctrl(zero=(20000.0, 30000.0))
        ex = _exec(ctrl, _needle())
        ex._intra_well_move(50000.0, 60000.0, target_z_mm=10.0)
        w = [c for c in ctrl.calls if c[0] == "wait_for_xy_arrival"][0]
        self.assertNotAlmostEqual(w[1], 50000.0 / 1000.0, places=6)
        self.assertAlmostEqual(50000.0 / 1000.0 - w[1], 20.0, places=6)

    def test_do_wash_still_converts_the_same_way(self):
        """The shared helper must not have changed the two correct call sites."""
        ctrl = _Ctrl(zero=(20000.0, 30000.0))
        ex = _exec(ctrl, _needle())
        self.assertTrue(ex._wait_xy_arrival_um(50000.0, 60000.0))
        w = [c for c in ctrl.calls if c[0] == "wait_for_xy_arrival"][0]
        self.assertAlmostEqual(w[1], 30.0, places=9)
        self.assertAlmostEqual(w[2], 30.0, places=9)

    def test_a_missing_zero_position_degrades_to_no_offset(self):
        ctrl = _Ctrl()
        del ctrl.zero_position
        ex = _exec(ctrl, _needle())
        ex._wait_xy_arrival_um(50000.0, 60000.0)
        w = [c for c in ctrl.calls if c[0] == "wait_for_xy_arrival"][0]
        self.assertAlmostEqual(w[1], 50.0, places=9)

    def test_an_unconfirmed_xy_arrival_aborts_and_does_NOT_lower(self):
        """Lowering onto an unverified in-well position is how a 0.1 mm
        clearance meets a well wall."""
        ctrl = _Ctrl(xy_ok=False)
        ex = _exec(ctrl, _needle())
        with self.assertRaises(AbortException) as cm:
            ex._intra_well_move(50000.0, 60000.0, target_z_mm=10.0)
        self.assertIn("retracted", str(cm.exception))
        lowers = [c for c in ctrl.calls if c[0] == "move_z_absolute"]
        self.assertEqual(lowers, [], "must not lower after a failed XY confirm")

    def test_an_unconfirmed_lift_aborts_before_moving_xy(self):
        ctrl = _Ctrl(z_ok=False)
        ex = _exec(ctrl, _needle())
        with self.assertRaises(AbortException):
            ex._intra_well_move(50000.0, 60000.0, target_z_mm=10.0)
        self.assertNotIn("move_xy_absolute_um", ctrl.kinds())

    def test_the_lift_confirm_uses_the_ORIGIN_height_not_the_target(self):
        """The lift is relative to where the tool IS; confirming against the
        TARGET height fails whenever the two differ by more than the 0.05 mm
        default tolerance."""
        ctrl = _Ctrl(z_up_sign=1.0)
        ex = _exec(ctrl, _needle())
        ex._intra_well_move(50000.0, 60000.0, target_z_mm=10.0,
                            from_z_mm=10.10)
        zwaits = [c for c in ctrl.calls if c[0] == "wait_for_z_arrival"]
        self.assertAlmostEqual(zwaits[0][1], 10.10 + 1.0, places=9)

    def test_the_shift_tolerance_is_tighter_than_the_shift_itself(self):
        """The 0.1 mm controller default is the same order as a 100 µm shift, so
        it would 'confirm' arrival at the pre-move position."""
        ctrl = _Ctrl()
        ex = _exec(ctrl, _needle(offsets=((0.0, 0.0), (120.0, 0.0))))
        op = MagicMock()
        self.assertTrue(ex._shift_to_bore(op, _target(), 1, 0, 10.0))
        w = [c for c in ctrl.calls if c[0] == "wait_for_xy_arrival"][0]
        shift_mm = 120.0 / 1000.0
        self.assertLess(w[3], shift_mm,
                        "tolerance must be smaller than the commanded shift")
        self.assertGreaterEqual(w[3], 0.01)

    def test_the_shift_still_reports_no_move_for_a_single_bore(self):
        ctrl = _Ctrl()
        ex = _exec(ctrl, _single())
        self.assertFalse(ex._shift_to_bore(MagicMock(), _target(), 0, 1, 10.0))
        self.assertEqual(ctrl.calls, [])


# ── D6: ONE dose, not two ────────────────────────────────────────────

class TestTheCellGetsExactlyOneDose(unittest.TestCase):
    """A dedicated dosing bore REPLACES the aspirating bore's push.

    Before this fix the cell received the dosing bore's volume AND the
    aspirating bore's own reagent column — up to 2× the intended reagent, the
    second half out of the same orifice about to pull the cell in. Cells were
    over-digested and the un-recovered excess kept digesting neighbours.
    """

    def _run(self, **over):
        ctrl = _Ctrl()
        n = _needle()
        kw = dict(reagent_bore="P1", release_depth_mm=0.10,
                  extract_multiplier=2.0, dwell_time_s=0.0,
                  push_speed_uL_s=0.5, pull_speed_uL_s=5.0,
                  trypsin_enabled=True, trypsin_bore="P2",
                  trypsin_bore_index=1, trypsin_depth_mm=0.10,
                  trypsin_push_rate_uL_s=0.25, trypsin_lead_time_s=0.0,
                  aspirate_bore_index=0)
        kw.update(over)
        cfg = CellRemovalConfig(**kw)
        ex = _exec(ctrl, n, _well_positions={"__trypsin__": (500.0, 600.0)})
        # Distinct wells so the placement is a real inter-well travel — a
        # same-well dest takes the intra-well branch and emits no safe_travel_to,
        # which would make "everything before the placement" unfindable.
        op = PickPlaceOperation(
            op_id="OP1", op_type=OperationType.CELL_TARGET_REMOVAL,
            source_target=_target(), dest_target=_target(x=70000.0, well="B2"),
            config=cfg)
        ex._execute_cell_removal(op)
        return ctrl

    def _pumps(self, ctrl, pump):
        return [(c[2], c[3]) for c in ctrl.calls
                if c[0] == "pump" and c[1] == pump]

    def test_the_aspirating_bore_never_pushes_when_a_dosing_bore_is_armed(self):
        ctrl = self._run()
        # At the target the aspirating bore may only PULL (negative).
        p1 = self._pumps(ctrl, "P1")
        self.assertTrue(p1, "the aspirating bore must still pull the cell")
        pulls = [v for (v, _r) in p1 if v < 0]
        pushes_before_placement = [v for (v, _r) in p1[:-1] if v > 0]
        self.assertTrue(pulls, "expected a pull")
        self.assertEqual(pushes_before_placement, [],
                         "the aspirating bore must not dose the cell too")

    def test_exactly_one_positive_dose_reaches_the_cell(self):
        ctrl = self._run()
        # Sequence up to the placement travel: only ONE positive pump move.
        idx = next(i for i, c in enumerate(ctrl.calls)
                   if c[0] == "safe_travel_to" and abs(c[1] - 70000.0) < 1e-6)
        doses = [c for c in ctrl.calls[:idx]
                 if c[0] == "pump" and c[2] > 0]
        self.assertEqual(len(doses), 1, f"expected 1 dose, got {doses}")
        self.assertEqual(doses[0][1], "P2", "the DOSING bore must deliver it")

    def test_the_aspirating_bore_does_not_visit_its_reagent_well(self):
        """It loads nothing, so the trip is pure wasted motion."""
        ctrl = self._run()
        travels = [(c[1], c[2]) for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertNotIn((1000.0, 2000.0), travels)

    def test_both_pumps_stay_volume_balanced(self):
        ctrl = self._run()
        for pump in ("P1", "P2"):
            net = round(sum(v for (v, _r) in self._pumps(ctrl, pump)), 10)
            self.assertAlmostEqual(net, 0.0, places=10,
                                   msg=f"{pump} drifted by {net} µL")

    def test_single_bore_behaviour_is_untouched(self):
        """With no dosing bore the aspirating bore still loads and pushes."""
        ctrl = self._run(trypsin_enabled=False)
        p1 = self._pumps(ctrl, "P1")
        self.assertTrue(any(v > 0 for (v, _r) in p1), "expected a push")
        self.assertTrue(any(v < 0 for (v, _r) in p1), "expected a pull")
        self.assertAlmostEqual(round(sum(v for (v, _r) in p1), 10), 0.0,
                               places=10)
        travels = [(c[1], c[2]) for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertIn((1000.0, 2000.0), travels, "must still load its reagent")

    def test_the_pull_is_still_sized_from_the_aspirating_bores_column(self):
        ctrl = self._run(extract_multiplier=3.0)
        n = _needle()
        col = CellRemovalConfig(
            reagent_bore="P1", release_depth_mm=0.10
        ).compute_release_volume_uL(n)
        pulls = [v for (v, _r) in self._pumps(ctrl, "P1") if v < 0]
        # The fake rounds recorded volumes to 8 dp, so compare at that precision.
        self.assertAlmostEqual(abs(pulls[0]), col * 3.0, places=7)


# ── D9: an unmeasured assembly is refused BEFORE it doses ────────────

class TestUnmeasuredAssemblyIsRefusedBeforeDosing(unittest.TestCase):
    def _run(self, needle, **over):
        ctrl = _Ctrl()
        kw = dict(reagent_bore="P1", release_depth_mm=0.10, dwell_time_s=0.0,
                  trypsin_enabled=True, trypsin_bore="P2",
                  trypsin_bore_index=1, trypsin_depth_mm=0.10,
                  aspirate_bore_index=0)
        kw.update(over)
        cfg = CellRemovalConfig(**kw)
        ex = _exec(ctrl, needle, _well_positions={"__trypsin__": (500.0, 600.0)})
        op = PickPlaceOperation(
            op_id="OP1", op_type=OperationType.CELL_TARGET_REMOVAL,
            source_target=_target(), dest_target=_target(x=70000.0),
            config=cfg)
        return ctrl, ex, op

    def test_zero_separation_refuses_with_nothing_dosed(self):
        """The whole point: by the time _shift_to_bore returns False the cell is
        already dosed. So the check must precede the load."""
        unmeasured = _needle(offsets=((0.0, 0.0), (0.0, 0.0)))
        ctrl, ex, op = self._run(unmeasured)
        with self.assertRaises(AbortException) as cm:
            ex._execute_cell_removal(op)
        self.assertEqual([c for c in ctrl.calls if c[0] == "pump"], [],
                         "must refuse before ANY pump move")
        self.assertEqual([c for c in ctrl.calls if c[0] == "safe_travel_to"], [],
                         "must refuse before ANY travel")
        msg = str(cm.exception)
        self.assertIn("Needle Location", msg)
        self.assertIn("not been measured", msg)

    def test_a_measured_assembly_runs(self):
        ctrl, ex, op = self._run(_needle())
        ex._execute_cell_removal(op)
        self.assertTrue([c for c in ctrl.calls if c[0] == "pump"])

    def test_same_bore_for_both_roles_is_not_blocked_by_this_gate(self):
        """Dosing and aspirating through ONE bore needs no shift at all."""
        unmeasured = _needle(offsets=((0.0, 0.0), (0.0, 0.0)))
        ctrl, ex, op = self._run(unmeasured, trypsin_bore_index=0,
                                 trypsin_bore="P1")
        ex._execute_cell_removal(op)
        self.assertTrue([c for c in ctrl.calls if c[0] == "pump"])

    def test_single_bore_runs_are_never_blocked(self):
        ctrl, ex, op = self._run(_single(), trypsin_enabled=False)
        ex._execute_cell_removal(op)
        self.assertTrue([c for c in ctrl.calls if c[0] == "pump"])


# ── D1/D8: simultaneous prep is WIRED, and fails loudly ──────────────

class TestActiveBoresDrivesPrep(unittest.TestCase):
    """`prep_bores` had NO production writer, so `move_pumps_uL` had zero live
    call sites and a dosing bore reached its reagent well full of AIR."""

    def _cfg(self, **over):
        kw = dict(reagent_bore="P1", release_depth_mm=0.10,
                  aspirate_bore_index=0)
        kw.update(over)
        return CellRemovalConfig(**kw)

    def test_single_bore_reports_exactly_one_driven_pump(self):
        got = self._cfg().active_bores()
        self.assertEqual(got, [{"pump_id": "P1", "bore_index": 0}])

    def test_a_dosing_bore_is_included_and_comes_first(self):
        """Load order: the dosing bore is the first thing the executor fills."""
        got = self._cfg(trypsin_enabled=True, trypsin_bore="P3",
                        trypsin_bore_index=2).active_bores()
        self.assertEqual(got, [{"pump_id": "P3", "bore_index": 2},
                               {"pump_id": "P1", "bore_index": 0}])

    def test_a_disabled_dosing_bore_is_excluded(self):
        got = self._cfg(trypsin_enabled=False, trypsin_bore="P3").active_bores()
        self.assertEqual([e["pump_id"] for e in got], ["P1"])

    def test_one_pump_serving_both_roles_appears_once(self):
        got = self._cfg(trypsin_enabled=True, trypsin_bore="p1",
                        trypsin_bore_index=0).active_bores()
        self.assertEqual(len(got), 1, "a pump cannot be driven twice in one G0")

    def test_pump_ids_are_normalised(self):
        got = self._cfg(reagent_bore=" p2 ").active_bores()
        self.assertEqual(got[0]["pump_id"], "P2")

    def test_it_does_NOT_include_a_recorded_but_undriven_bore(self):
        """Only what the executor drives — prepping an undriven bore wastes
        reagent and leaves it dripping into the plate."""
        got = self._cfg(trypsin_enabled=True, trypsin_bore="P2",
                        trypsin_bore_index=1).active_bores()
        self.assertEqual({e["pump_id"] for e in got}, {"P1", "P2"})


class TestPrepFailsLoudly(unittest.TestCase):
    def _ex(self, ctrl, bores, **over):
        kw = dict(prep_bore="P1", prep_rate_uL_s=1.0, needle_volume_uL=5.0,
                  oil_needles=1.0, buffer_needles=1.0, prep_bores=bores,
                  waste_well_pos=(1.0, 2.0), oil_well_pos=(3.0, 4.0),
                  wash_well_pos=(5.0, 6.0), buffer_well_pos=(7.0, 8.0),
                  service_z_mm=20.0, wash_cycles=0, wash_z_amplitude_mm=0.0,
                  wash_xy_amplitude_um=0.0, wash_dwell_s=0.0)
        kw.update(over)
        return _exec(ctrl, _needle(), **kw)

    def _bores(self):
        return [{"pump_id": "P1", "bore_index": 0},
                {"pump_id": "P2", "bore_index": 1}]

    def test_a_refused_coordinated_move_stops_the_prep(self):
        """False = nothing moved. Continuing means dosing air onto the cells."""
        class _C(_Ctrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                return False
        ctrl = _C()
        with self.assertRaises(RuntimeError) as cm:
            self._ex(ctrl, self._bores()).run_prep()
        self.assertIn("unconditioned", str(cm.exception))

    def test_an_abort_during_prep_raises_AbortException_not_RuntimeError(self):
        """execute_queue returns immediately on abort but drives on to the next
        target after a RuntimeError — the distinction matters."""
        class _C(_Ctrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                return False
        ctrl = _C()
        ex = self._ex(ctrl, self._bores())
        ex._abort_flag.set()
        with self.assertRaises(AbortException):
            ex.run_prep()

    def test_a_move_that_delivered_nothing_is_caught_despite_returning_True(self):
        """The bool cannot express 'True but nothing delivered' — hence the
        `delivered` out-param. An all-clamped prep leaves the bores full of air."""
        class _C(_Ctrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, *, delivered=None,
                              **k):
                if delivered is not None:
                    delivered.clear()          # cleared → authoritative, empty
                return True
        ctrl = _C()
        with self.assertRaises(RuntimeError) as cm:
            self._ex(ctrl, self._bores()).run_prep()
        msg = str(cm.exception)
        self.assertIn("delivered", msg)
        self.assertIn("air", msg)

    def test_a_partially_delivered_move_is_caught(self):
        class _C(_Ctrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, *, delivered=None,
                              **k):
                if delivered is not None:
                    delivered.clear()
                    for p, v in volumes.items():
                        delivered[p] = v * 0.01       # 1 % delivered
                return True
        ctrl = _C()
        with self.assertRaises(RuntimeError):
            self._ex(ctrl, self._bores()).run_prep()

    def test_a_fully_delivered_move_proceeds(self):
        class _C(_Ctrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, *, delivered=None,
                              **k):
                if delivered is not None:
                    delivered.clear()
                    delivered.update(volumes)
                return True
        ctrl = _C()
        self._ex(ctrl, self._bores()).run_prep()      # must not raise

    def test_a_controller_that_cannot_report_is_not_treated_as_a_failure(self):
        """A controller accepting `delivered` via **kwargs and ignoring it leaves
        an empty dict — indistinguishable from 'nothing delivered' without the
        sentinel. Treating it as failure would break every prep on an older build."""
        class _C(_Ctrl):
            def move_pumps_uL(self, volumes, rate_uL_s=None, **k):
                return True                    # swallows `delivered` silently
        ctrl = _C()
        self._ex(ctrl, self._bores()).run_prep()      # must not raise

    def test_an_older_controller_falls_back_to_sequential_moves(self):
        ctrl = _Ctrl()          # no move_pumps_uL at all
        self._ex(ctrl, self._bores()).run_prep()
        pumped = {c[1] for c in ctrl.calls if c[0] == "pump"}
        self.assertEqual(pumped, {"P1", "P2"})


# ── D11: the config serializer no longer drops the v7.9 fields ───────

class TestCellRemovalConfigRoundTrip(unittest.TestCase):
    def test_a_trypsin_config_round_trips_every_field(self):
        cfg = CellRemovalConfig(
            reagent_bore="P1", release_depth_mm=0.10, extract_multiplier=2.5,
            aspirate_bore_index=2, trypsin_enabled=True, trypsin_bore="P3",
            trypsin_bore_index=1, trypsin_depth_mm=0.08,
            trypsin_volume_uL=0.004, trypsin_push_rate_uL_s=0.25,
            trypsin_lead_time_s=30.0)
        back = CellRemovalConfig.from_dict(cfg.to_dict())
        for f in ("aspirate_bore_index", "trypsin_enabled", "trypsin_bore",
                  "trypsin_bore_index", "trypsin_well_key", "trypsin_depth_mm",
                  "trypsin_volume_uL", "trypsin_push_rate_uL_s",
                  "trypsin_lead_time_s"):
            self.assertEqual(getattr(back, f), getattr(cfg, f), f)

    def test_a_single_bore_config_emits_none_of_the_new_keys(self):
        """Conditional emit keeps a legacy block byte-identical."""
        d = CellRemovalConfig(reagent_bore="P1").to_dict()
        for f in ("aspirate_bore_index", "trypsin_enabled", "trypsin_bore",
                  "trypsin_depth_mm"):
            self.assertNotIn(f, d)
        self.assertEqual(len(d), 9, "exactly the nine legacy keys")

    def test_unknown_keys_from_a_newer_build_are_dropped_not_fatal(self):
        d = CellRemovalConfig(reagent_bore="P1").to_dict()
        d["some_v8_field"] = 42
        back = CellRemovalConfig.from_dict(d)
        self.assertEqual(back.reagent_bore, "P1")


if __name__ == "__main__":
    unittest.main()
