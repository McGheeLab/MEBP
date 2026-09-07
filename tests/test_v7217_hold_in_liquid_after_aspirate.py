"""
v7.21.7 — HOLD THE NEEDLE IN THE LIQUID AFTER ASPIRATING.

Operator report (Quick Print): *"whenever the needle aspirates a liquid (e.g.
oil or ink), the Z is moving up before the pump is done aspirating. This results
in the needle being outside of the liquid and into the air. When the needle is
the air, it aspirates air -- this is bad."*

The plunger side was already synchronous: ``_settled_pump_move`` →
``move_pump_uL(settle=True)`` blocks on an M400 drain, so the PLUNGER is
provably finished before the caller advances. The FLUID is not — a compliant
column (fine bore, viscous ink, long tube) keeps drawing in after the plunger
stops, and the retract that follows lifts the tip out of the well inside that
window, so the tail of the aspirate is air.

The fix is a hold, held with the tip STILL SUBMERGED, between the aspirate and
the travel that lifts it: ``HardwareConfig.pump_post_aspirate_dwell_s`` →
``StageController.pump_post_aspirate_dwell_s()`` → the executor's
``_settle_in_liquid`` at every reagent aspirate.

What these pin, in the order the value flows:
  1. the config field round-trips, tolerates junk, and an ABSENT key takes the
     2 s default (the deliberate migration — a setup saved before this existed
     was written by code that had the defect);
  2. the controller reads it, clamped ≥ 0;
  3. the executor RESOLVES it from the controller when its own field is None
     (this is what makes every workflow inherit it with no per-page wiring) and
     degrades to 0 rather than raising;
  4. ⭐ the ORDERING claim, which is the whole point: the hold happens AFTER the
     pump move and BEFORE the next travel/retract, at every reagent aspirate
     (ink, prep oil, prep buffer, post-clean buffer, cleanup oil reset,
     starting-oil top-up);
  5. a DISPENSE is never held (leaving early costs nothing there), and 0 s is
     byte-identical to the legacy sequence.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest
from types import SimpleNamespace
from unittest import mock

from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.StageController import StageController
from SupportClasses.PickAndPlaceManager import PickPlaceExecutor, AbortException
from SupportClasses.CommonPrintSettings import GLOBAL_KEYS, GLOBAL_DEFAULTS


# ══════════════════════════════════════════════════════════════════════
#  A recording executor: every motion / pump call lands on ONE ordered
#  event list, so a test can assert on the SEQUENCE rather than on a
#  count. The defect being fixed is purely an ordering defect, so a
#  fixture that cannot see order cannot see the bug.
# ══════════════════════════════════════════════════════════════════════

class _FakeCtrl:
    """Minimal StageController stand-in that records every pump actuation.

    Deliberately implements ``move_pump_uL`` rather than letting the test
    override ``_pump_move``: the prep's ``compensate=None`` aspirates go through
    the module-level ``_settled_pump_move`` instead, so an override would leave
    exactly the reagent aspirates this change is about un-recorded.
    """

    def __init__(self, events):
        self._events = events
        self.pump_position_uL = 0.0

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, **kw):
        self._events.append(("pump", round(float(volume_uL), 4)))

    def get_pump_position_uL(self, pump):
        return self.pump_position_uL

    def pump_dir_sign(self, pump):
        return 1.0


class _Recorder(PickPlaceExecutor):
    def __init__(self, dwell=None, ctrl=None):
        events: list = []
        super().__init__(ctrl if ctrl is not None else _FakeCtrl(events), None)
        self.events = events
        self.post_aspirate_dwell_s = dwell
        self.needle_volume_uL = 1.0
        self.service_z_mm = 0.5
        self.prep_rate_uL_s = 1.0
        self.safe_z_mm = 10.0
        self.waste_well_pos = (1000.0, 1000.0)
        self.oil_well_pos = (2000.0, 1000.0)
        self.wash_well_pos = (3000.0, 1000.0)
        self.buffer_well_pos = (4000.0, 1000.0)

    # ── stubbed motion / pump primitives ──
    def _safe_move_to_well(self, key, target_z_mm=None):
        self.events.append(("travel", key))
        return True

    def _safe_move_to(self, target, target_z_mm=None):
        self.events.append(("travel", getattr(target, "well_name", "?")))
        return True

    def _prep_pump_move(self, plan, needles, sign, rate, **kw):
        self.events.append(("pump", round(sign * needles, 4)))

    def _do_wash(self):
        self.events.append(("wash", None))

    def _dwell(self, op, duration_s, label):
        self.events.append(("hold", round(float(duration_s), 3)))

    def _orbit_xy(self, *a, **kw):
        return None

    # convenience views over the event list
    @property
    def kinds(self):
        return [k for k, _ in self.events]

    def index_of(self, kind, after=0):
        for i in range(after, len(self.events)):
            if self.events[i][0] == kind:
                return i
        return -1


def _executor(dwell=None, ctrl=None):
    return _Recorder(dwell=dwell, ctrl=ctrl)


# ══════════════════════════════════════════════════════════════════════
#  1. The config field
# ══════════════════════════════════════════════════════════════════════

class TestConfigField(unittest.TestCase):

    def test_default_is_a_real_hold_not_zero(self):
        """A fresh config holds. Shipping the knob defaulted to 0 would leave
        the reported defect in place until the operator found the setting."""
        self.assertGreater(HardwareConfig().pump_post_aspirate_dwell_s, 0.0)

    def test_round_trips(self):
        cfg = HardwareConfig()
        cfg.pump_post_aspirate_dwell_s = 4.5
        back = HardwareConfig.from_dict(cfg.to_dict())
        self.assertAlmostEqual(back.pump_post_aspirate_dwell_s, 4.5)

    def test_zero_round_trips_as_zero(self):
        """0 must survive the trip — it is how an operator turns the hold OFF,
        and a falsy-value bug would silently restore the 2 s default."""
        cfg = HardwareConfig()
        cfg.pump_post_aspirate_dwell_s = 0.0
        back = HardwareConfig.from_dict(cfg.to_dict())
        self.assertEqual(back.pump_post_aspirate_dwell_s, 0.0)

    def test_absent_key_takes_the_default(self):
        """The deliberate migration: a setup saved before v7.21.7 was written
        by code carrying the defect, so it INHERITS the fix rather than
        silently keeping the old behaviour."""
        d = HardwareConfig().to_dict()
        d.pop("pump_post_aspirate_dwell_s", None)
        self.assertEqual(
            HardwareConfig.from_dict(d).pump_post_aspirate_dwell_s, 2.0)

    def test_junk_and_negative_fall_back(self):
        for bad in ("nonsense", None, -3.0):
            d = HardwareConfig().to_dict()
            d["pump_post_aspirate_dwell_s"] = bad
            self.assertEqual(
                HardwareConfig.from_dict(d).pump_post_aspirate_dwell_s, 2.0,
                f"bad value {bad!r} should fall back to the default")

    def test_registered_as_a_shared_global(self):
        """It is edited from Hardware Setup → Pump AND the Common Print
        Settings page, so it has to be in the shared global registry or the
        two surfaces drift."""
        self.assertIn("pump_post_aspirate_dwell_s", GLOBAL_KEYS)
        self.assertEqual(GLOBAL_DEFAULTS["pump_post_aspirate_dwell_s"], 2.0)


# ══════════════════════════════════════════════════════════════════════
#  2. The controller reader
# ══════════════════════════════════════════════════════════════════════

class TestControllerReader(unittest.TestCase):

    def _ctrl(self, cfg):
        c = StageController.__new__(StageController)
        c._hardware_config = cfg
        return c

    def test_reads_the_config(self):
        cfg = HardwareConfig()
        cfg.pump_post_aspirate_dwell_s = 3.25
        self.assertAlmostEqual(
            self._ctrl(cfg).pump_post_aspirate_dwell_s(), 3.25)

    def test_no_config_is_zero(self):
        self.assertEqual(self._ctrl(None).pump_post_aspirate_dwell_s(), 0.0)

    def test_negative_is_clamped(self):
        cfg = HardwareConfig()
        cfg.pump_post_aspirate_dwell_s = -5.0
        self.assertEqual(self._ctrl(cfg).pump_post_aspirate_dwell_s(), 0.0)

    def test_is_not_the_settle_dwell(self):
        """Guard-the-guard: the two dwells are separate values covering
        different halves of the same move (plunger vs fluid). If a later tidy-up
        aliased one onto the other this fails."""
        cfg = HardwareConfig()
        cfg.pump_settle_time_s = 0.1
        cfg.pump_post_aspirate_dwell_s = 7.0
        c = self._ctrl(cfg)
        self.assertAlmostEqual(c.pump_settle_time_s(), 0.1)
        self.assertAlmostEqual(c.pump_post_aspirate_dwell_s(), 7.0)


# ══════════════════════════════════════════════════════════════════════
#  3. The executor's resolution (None ⇒ inherit the global)
# ══════════════════════════════════════════════════════════════════════

class TestResolution(unittest.TestCase):

    def test_inherits_the_controller_global(self):
        """The load-bearing one: no workflow page assigns the field, so if
        inheritance breaks, EVERY workflow silently loses the hold."""
        ctrl = SimpleNamespace(pump_post_aspirate_dwell_s=lambda: 2.5)
        self.assertAlmostEqual(
            _executor(dwell=None, ctrl=ctrl)._post_aspirate_dwell_s(), 2.5)

    def test_explicit_value_overrides_the_global(self):
        ctrl = SimpleNamespace(pump_post_aspirate_dwell_s=lambda: 2.5)
        self.assertAlmostEqual(
            _executor(dwell=9.0, ctrl=ctrl)._post_aspirate_dwell_s(), 9.0)

    def test_explicit_zero_beats_the_global(self):
        """0 is a real choice ("no hold"), not "unset" — it must not fall
        through to the global."""
        ctrl = SimpleNamespace(pump_post_aspirate_dwell_s=lambda: 2.5)
        self.assertEqual(
            _executor(dwell=0.0, ctrl=ctrl)._post_aspirate_dwell_s(), 0.0)

    def test_older_controller_degrades_to_no_hold(self):
        """A controller/fake without the getter is the pre-v7.21.7 behaviour,
        not a crash mid-run."""
        self.assertEqual(
            _executor(dwell=None, ctrl=SimpleNamespace())
            ._post_aspirate_dwell_s(), 0.0)

    def test_a_raising_getter_degrades_to_no_hold(self):
        def _boom():
            raise RuntimeError("board gone")
        ctrl = SimpleNamespace(pump_post_aspirate_dwell_s=_boom)
        self.assertEqual(
            _executor(dwell=None, ctrl=ctrl)._post_aspirate_dwell_s(), 0.0)

    def test_zero_dwell_emits_no_hold_at_all(self):
        """Not merely "sleeps 0" — the sub-step / dwell-tick chatter is skipped
        too, so turning it off gives the legacy sequence exactly."""
        ex = _executor(dwell=0.0)
        ex._settle_in_liquid(SimpleNamespace(op_id="X", sub_step=""), "ink")
        self.assertEqual(ex.events, [])


# ══════════════════════════════════════════════════════════════════════
#  4. ⭐ ORDERING — the actual defect
# ══════════════════════════════════════════════════════════════════════

class TestAspirateIsHeldBeforeTheNeedleLeaves(unittest.TestCase):
    """Each of these asserts the hold sits BETWEEN the aspirate and the next
    travel. Asserting only that a hold happened somewhere would pass with the
    hold on the wrong side of the retract — i.e. with the bug still shipping."""

    HOLD = 3.0

    def _hold_is_between_pump_and_next_travel(self, ex, pump_index):
        kinds = ex.kinds
        self.assertEqual(kinds[pump_index], "pump")
        self.assertLess(pump_index + 1, len(kinds),
                        "the aspirate is the last event — nothing was held")
        self.assertEqual(
            kinds[pump_index + 1], "hold",
            f"expected a hold immediately after the aspirate, got "
            f"{kinds[pump_index + 1]!r}; sequence = {kinds}")
        self.assertEqual(ex.events[pump_index + 1][1], self.HOLD)
        # …and the next travel (which retracts Z) comes after the hold.
        nxt = ex.index_of("travel", after=pump_index + 1)
        if nxt != -1:
            self.assertGreater(nxt, pump_index + 1)

    def test_ink_pickup_holds_before_the_caller_retracts(self):
        ex = _executor(dwell=self.HOLD)
        ex.aspirate_ink((5000.0, 5000.0), 3.0, bore="P1", z_mm=0.4)
        self.assertEqual(ex.kinds, ["travel", "pump", "hold"])
        self.assertEqual(ex.events[1][1], -3.0)      # − = aspirate

    def test_ink_pickup_with_tip_prime_holds_after_the_dispense_back(self):
        """With priming on, the aspirate is followed by a dispense-back INTO
        the same well with no Z move between — so the single hold belongs at
        the end of the in-well sequence, immediately before the needle leaves.
        """
        ex = _executor(dwell=self.HOLD)
        ex.aspirate_ink((5000.0, 5000.0), 3.0, bore="P1", z_mm=0.4,
                        prime_uL=1.0)
        self.assertEqual(ex.kinds, ["travel", "pump", "pump", "hold"])
        self.assertEqual(ex.events[1][1], -4.0)      # vol + prime
        self.assertEqual(ex.events[2][1], +1.0)      # prime dispensed back

    def test_prep_holds_in_the_oil_and_in_the_buffer(self):
        ex = _executor(dwell=self.HOLD)
        ex.run_prep()
        kinds = ex.kinds
        # waste (dispense, no hold) → oil (aspirate + hold) → wash →
        # buffer (aspirate + hold)
        self.assertEqual(
            kinds,
            ["travel", "pump",                    # waste: dispense oil
             "travel", "pump", "hold",            # oil: aspirate + HOLD
             "travel", "wash",                    # wash
             "travel", "pump", "hold"],           # buffer: aspirate + HOLD
            f"unexpected prep sequence: {kinds}")
        self._hold_is_between_pump_and_next_travel(ex, 3)   # oil
        self._hold_is_between_pump_and_next_travel(ex, 8)   # buffer

    def test_prep_dispense_to_waste_is_NOT_held(self):
        """The defect is about leaving liquid too early during an ASPIRATE.
        Holding on a dispense would add dead time for nothing."""
        ex = _executor(dwell=self.HOLD)
        ex.run_prep()
        self.assertEqual(ex.kinds[1], "pump")
        self.assertNotEqual(
            ex.kinds[2], "hold",
            "the dispense-to-waste must not be followed by a hold")

    def test_post_clean_holds_in_the_buffer(self):
        ex = _executor(dwell=self.HOLD)
        ex.buffer_needles = 1.0
        ex.post_dispense_needles = 1.0
        ex.run_post_clean()
        kinds = ex.kinds
        self.assertEqual(kinds[-2:], ["pump", "hold"],
                         f"buffer reload must be held: {kinds}")

    def test_starting_oil_top_up_is_held_but_the_waste_dump_is_not(self):
        ex = _executor(dwell=self.HOLD)
        ex.prepare_starting_oil(5.0, dispense_to_waste=False)   # ASPIRATE oil
        self.assertEqual(ex.kinds, ["travel", "pump", "hold"])

        ex2 = _executor(dwell=self.HOLD)
        ex2.prepare_starting_oil(5.0, dispense_to_waste=True)   # DISPENSE
        self.assertEqual(ex2.kinds, ["travel", "pump"])

    def test_cleanup_oil_reset_is_held_only_when_it_aspirates(self):
        """The reset is SIGNED — it can go either way depending on where the
        plunger ended up — so the hold is gated on the direction, not on the
        step."""
        for baseline_delta, expect_hold in ((-4.0, True), (+4.0, False)):
            with self.subTest(delta=baseline_delta):
                ex = _executor(dwell=self.HOLD)
                ex.cleanup_oil_baseline_uL = baseline_delta
                ex.cleanup_reset_to_initial = False
                ex.cleanup_waste_needles = 0.0
                ex.run_print_cleanup()
                held = ex.kinds[-1] == "hold"
                self.assertEqual(
                    held, expect_hold,
                    f"oil reset {baseline_delta:+} µL: hold={held}, "
                    f"sequence={ex.kinds}")


# ══════════════════════════════════════════════════════════════════════
#  5. Zero = the legacy sequence, exactly
# ══════════════════════════════════════════════════════════════════════

class TestZeroIsLegacy(unittest.TestCase):

    def test_prep_sequence_is_unchanged_at_zero(self):
        ex = _executor(dwell=0.0)
        ex.run_prep()
        self.assertNotIn("hold", ex.kinds)
        self.assertEqual(
            ex.kinds,
            ["travel", "pump", "travel", "pump", "travel", "wash",
             "travel", "pump"])

    def test_ink_pickup_is_unchanged_at_zero(self):
        ex = _executor(dwell=0.0)
        ex.aspirate_ink((1.0, 2.0), 3.0, bore="P1", z_mm=0.4)
        self.assertEqual(ex.kinds, ["travel", "pump"])

    def test_travel_only_pickup_still_holds_nothing(self):
        """volume 0 + no prime is a travel-only pickup: no aspirate happened,
        so there is nothing to wait for."""
        ex = _executor(dwell=5.0)
        ex.aspirate_ink((1.0, 2.0), 0.0, bore="P1", z_mm=0.4)
        self.assertEqual(ex.kinds, ["travel"])


# ══════════════════════════════════════════════════════════════════════
#  6. Abort responsiveness
# ══════════════════════════════════════════════════════════════════════

class TestAbortDuringTheHold(unittest.TestCase):

    def test_abort_interrupts_the_hold(self):
        """The hold can be seconds long and sits in the middle of a run, so it
        must not swallow an Abort. It goes through the ordinary `_dwell`, which
        raises — this pins that it really does route through it."""
        ex = PickPlaceExecutor(
            SimpleNamespace(pump_post_aspirate_dwell_s=lambda: 30.0), None)
        ex._abort_flag.set()
        with self.assertRaises(AbortException):
            ex._settle_in_liquid(SimpleNamespace(op_id="X", sub_step=""), "ink")

    def test_prep_still_checks_abort_when_nothing_is_actuated(self):
        """Regression guard on my own first cut: routing the hold through
        `actuate` must not remove the unconditional abort check that ran on the
        no-op branch before v7.21.7."""
        ex = _executor(dwell=0.0)
        ex.needle_volume_uL = 0.0        # ⇒ actuate() actuates nothing
        ex._abort_flag.set()
        with self.assertRaises(AbortException):
            ex.run_prep()


if __name__ == "__main__":
    unittest.main()
