"""
v7.5.x — pump settle time + prime time (Hardware Setup → Pump).

Operator request: every *discrete* pump move gets a blocking settle dwell
before AND after (so the workflow doesn't advance until the pump has finished
and settled), plus a configurable prime time that ports into the printing
workflows. Both live on the Hardware Setup → Pump tab (persisted in
HardwareConfig).

These exercise the pure logic:
  * HardwareConfig round-trips the two new global fields (with default /
    bad-value tolerance);
  * StageController.pump_settle_time_s() reads the config (clamped ≥ 0);
  * move_pump_uL(settle=True) sleeps the settle dwell pre + post and blocks
    for the (open-loop) move to complete in between;
  * move_pump_uL(settle=False) is an unchanged passthrough (no sleeps) — the
    streamed print path and manual jog are not affected;
  * PickAndPlaceManager._settled_pump_move passes settle=True when supported
    and falls back cleanly on controllers/fakes that lack the kwarg;
  * Quick Print's prime default ports in from pump_prime_time_s.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest
from types import SimpleNamespace
from unittest import mock

from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import (
    StageController,
    _PUMP_MOVE_DRAIN_TIMEOUT_CAP_S,
    _PUMP_MOVE_DRAIN_MARGIN_S,
    _PUMP_SETTLE_FALLBACK_RATE_UL_S,
)
from SupportClasses.PickAndPlaceManager import _settled_pump_move


# ── Minimal fakes ──────────────────────────────────────────────────────

class _FakePumpCfg:
    is_configured = True
    syringe = SimpleNamespace(volume_uL=100.0)

    def uL_to_mm(self, v):
        return v * 0.3

    def feedrate_uL_s_to_mm_min(self, r):
        return r * 18.0


class _FakeHW:
    def __init__(self, settle=0.0, prime=0.25):
        self.pump_settle_time_s = settle
        self.pump_prime_time_s = prime
        self.pumps = {p: _FakePumpCfg() for p in ("P1", "P2", "P3")}


def _ctrl(settle=0.0, relief_uL=0.0, backlash=False):
    """StageController with just enough state for move_pump_uL.

    move_pump_relative is monkeypatched to a recorder so the test isolates the
    settle/completion-wait logic (the relative-move safety machinery is covered
    elsewhere). safety_limits.enabled = False skips the flow-rate clamp branch.

    v7.5.x: ``relief_uL`` seeds P1's per-pump compliance value and ``backlash``
    the global enable, so the take-up/unload comp path can be exercised.
    """
    c = StageController.__new__(StageController)
    c._hardware_config = _FakeHW(settle=settle)
    c.safety_limits = SafetyLimits()
    c.safety_limits.enabled = False
    c._pump_relief_uL = {"P1": relief_uL, "P2": relief_uL, "P3": relief_uL}
    c._backlash_comp_enabled = backlash
    c.pump_moves = []
    c.move_pump_relative = (
        lambda pump, dist, fr=None: c.pump_moves.append((pump, dist, fr)))
    return c


# ════════════════════════════════════════════════════════════════════
#  HardwareConfig serialization
# ════════════════════════════════════════════════════════════════════

class TestHardwareConfigFields(unittest.TestCase):

    def test_defaults(self):
        cfg = HardwareConfig()
        self.assertEqual(cfg.pump_settle_time_s, 0.0)
        self.assertEqual(cfg.pump_prime_time_s, 0.25)

    def test_round_trip(self):
        cfg = HardwareConfig()
        cfg.pump_settle_time_s = 0.4
        cfg.pump_prime_time_s = 0.6
        out = HardwareConfig.from_dict(cfg.to_dict())
        self.assertAlmostEqual(out.pump_settle_time_s, 0.4)
        self.assertAlmostEqual(out.pump_prime_time_s, 0.6)

    def test_missing_keys_use_defaults(self):
        out = HardwareConfig.from_dict({})
        self.assertEqual(out.pump_settle_time_s, 0.0)
        self.assertEqual(out.pump_prime_time_s, 0.25)

    def test_bad_or_negative_values_fall_back(self):
        out = HardwareConfig.from_dict(
            {"pump_settle_time_s": -1.0, "pump_prime_time_s": "nope"})
        self.assertEqual(out.pump_settle_time_s, 0.0)
        self.assertEqual(out.pump_prime_time_s, 0.25)

    def test_legacy_relief_keys_are_ignored(self):
        # v7.5.x: pressure relief moved to per-pump µL (device profile). A config
        # carrying the legacy percent / toggle / absolute-µL keys loads cleanly
        # with those keys DISCARDED (no HardwareConfig relief fields anymore).
        out = HardwareConfig.from_dict({
            "pump_relief_volume_uL": 0.8,
            "pump_relief_percent": 3.0,
            "pump_relief_on_pickup": False,
        })
        self.assertFalse(hasattr(out, "pump_relief_volume_uL"))
        self.assertFalse(hasattr(out, "pump_relief_percent"))
        self.assertFalse(hasattr(out, "pump_relief_on_pickup"))


# ════════════════════════════════════════════════════════════════════
#  StageController.pump_settle_time_s reader
# ════════════════════════════════════════════════════════════════════

class TestSettleReader(unittest.TestCase):

    def test_no_config_is_zero(self):
        c = StageController.__new__(StageController)
        c._hardware_config = None
        self.assertEqual(c.pump_settle_time_s(), 0.0)

    def test_reads_config(self):
        c = _ctrl(settle=0.3)
        self.assertAlmostEqual(c.pump_settle_time_s(), 0.3)

    def test_negative_clamped_to_zero(self):
        c = _ctrl(settle=-5.0)
        self.assertEqual(c.pump_settle_time_s(), 0.0)


# ════════════════════════════════════════════════════════════════════
#  move_pump_uL settle behaviour
# ════════════════════════════════════════════════════════════════════

class TestMovePumpSettle(unittest.TestCase):

    def _run(self, ctrl, **kw):
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            ctrl.move_pump_uL("P1", 2.0, rate_uL_s=10.0, **kw)
        return sleeps

    def test_settle_false_is_passthrough_no_sleep(self):
        # The streamed print path / manual jog path: NO dwell, NO blocking wait.
        c = _ctrl(settle=0.2)
        sleeps = self._run(c, settle=False)
        self.assertEqual(sleeps, [])
        self.assertEqual(len(c.pump_moves), 1)

    def test_settle_true_brackets_and_waits(self):
        # settle=0.2; vol=2 µL at 10 µL/s → completion wait = 2/10 + 0.1 = 0.3.
        # v7.5.x: the pre-move settle was dropped — a non-compensated settled
        # move now dwells only AFTER it drains (completion wait, then a single
        # post-move settle).
        c = _ctrl(settle=0.2)
        sleeps = self._run(c, settle=True)
        self.assertEqual(len(sleeps), 2)            # complete, post
        self.assertAlmostEqual(sleeps[0], 0.3)      # completion wait
        self.assertAlmostEqual(sleeps[1], 0.2)      # post-move settle
        self.assertEqual(len(c.pump_moves), 1)

    def test_settle_true_zero_dwell_still_waits_for_completion(self):
        # settle time 0 but settle=True → only the completion wait (preserves
        # the legacy EXTRUDE blocking semantics this path subsumes). The fake
        # _ctrl has no zp_stage.flush_moves, so this exercises the open-loop
        # fallback wait (the full estimate, no longer truncated at 10 s).
        c = _ctrl(settle=0.0)
        sleeps = self._run(c, settle=True)
        self.assertEqual(len(sleeps), 1)
        self.assertAlmostEqual(sleeps[0], 0.3)

    def test_fallback_completion_wait_not_truncated_at_10s(self):
        # No board confirmation (fake without flush_moves) → the open-loop
        # fallback now waits the FULL estimated move duration, not a 10 s cap.
        # A 4-needle buffer aspirate (~27 µL at 1 µL/s) is the real-world case
        # the old 10 s truncation broke (pump still running into the next M400).
        c = _ctrl(settle=0.0)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P2", -27.222, rate_uL_s=1.0, settle=True)
        self.assertEqual(len(sleeps), 1)
        expected = 27.222 / 1.0 + 0.1
        self.assertAlmostEqual(sleeps[0], expected)   # ~27.3 s, NOT 10 s

    def test_fallback_completion_wait_is_capped_at_backstop(self):
        # Pathological volume / tiny rate → the fallback wait clamps at the
        # absolute backstop so it can never hang forever.
        c = _ctrl(settle=0.0)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 1e6, rate_uL_s=0.01, settle=True)
        self.assertEqual(len(sleeps), 1)
        self.assertAlmostEqual(sleeps[0], _PUMP_MOVE_DRAIN_TIMEOUT_CAP_S)

    def test_completion_wait_uses_fallback_rate_when_none(self):
        # rate_uL_s=None → completion estimate uses the fallback rate.
        c = _ctrl(settle=0.0)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 1.0, rate_uL_s=None, settle=True)
        expected = 1.0 / _PUMP_SETTLE_FALLBACK_RATE_UL_S + 0.1
        self.assertAlmostEqual(sleeps[0], expected)


# ════════════════════════════════════════════════════════════════════
#  move_pump_uL settle — board-confirmed (M400) completion drain
# ════════════════════════════════════════════════════════════════════

class _FakeZP:
    """Records flush_moves(timeout_s=) calls; returns ``result``."""

    def __init__(self, result=True):
        self.result = result
        self.flush_calls = []

    def flush_moves(self, timeout_s=15.0):
        self.flush_calls.append(timeout_s)
        return self.result


def _ctrl_with_zp(settle=0.0, flush_result=True):
    c = _ctrl(settle=settle)
    c.zp_stage = _FakeZP(result=flush_result)
    # Poller suspend/resume must be no-ops on this bare stub.
    c._pos_poller = None
    return c


class TestMovePumpSettleM400Confirm(unittest.TestCase):
    """When the board can confirm completion, the long discrete pump move is
    drained via M400 (scaled timeout) instead of an open-loop sleep — so a
    multi-needle buffer aspirate finishes before the next safe_travel_to M400
    (the bug: the pump bled into that M400 and tripped its 15 s timeout)."""

    def test_buffer_move_drained_via_flush_not_sleep(self):
        c = _ctrl_with_zp(settle=0.0, flush_result=True)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P2", -27.222, rate_uL_s=1.0, settle=True)
        # flush_moves confirmed completion → NO open-loop completion sleep.
        self.assertEqual(sleeps, [])
        self.assertEqual(len(c.zp_stage.flush_calls), 1)
        # Timeout scales with the move (~27.3 s) + margin, well over 15 s.
        expected_to = (27.222 / 1.0 + 0.1) + _PUMP_MOVE_DRAIN_MARGIN_S
        self.assertAlmostEqual(c.zp_stage.flush_calls[0], expected_to)

    def test_flush_timeout_does_not_double_wait(self):
        # flush_moves available but times out → it already waited ~move_s, so
        # the caller must NOT sleep again on top of it.
        c = _ctrl_with_zp(settle=0.0, flush_result=False)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P2", -27.222, rate_uL_s=1.0, settle=True)
        self.assertEqual(sleeps, [])
        self.assertEqual(len(c.zp_stage.flush_calls), 1)

    def test_settle_dwell_still_brackets_flush_path(self):
        # v7.5.x: the pre-move settle was dropped; a non-compensated settled move
        # now dwells only AFTER the M400 drain (a single post-move settle, no
        # pre-move and no mid sleep — flush confirms so no fallback sleep).
        c = _ctrl_with_zp(settle=0.2, flush_result=True)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 2.0, rate_uL_s=10.0, settle=True)
        self.assertEqual(sleeps, [0.2])               # post-move only
        self.assertEqual(len(c.zp_stage.flush_calls), 1)

    def test_flush_drain_timeout_capped_at_backstop(self):
        c = _ctrl_with_zp(settle=0.0, flush_result=True)
        with mock.patch("SupportClasses.StageController.time.sleep"):
            c.move_pump_uL("P1", 1e6, rate_uL_s=0.01, settle=True)
        self.assertAlmostEqual(c.zp_stage.flush_calls[0],
                               _PUMP_MOVE_DRAIN_TIMEOUT_CAP_S)

    def test_settle_false_never_confirms(self):
        # Streamed print path / manual jog: no flush, no sleep, pure passthrough.
        c = _ctrl_with_zp(settle=0.2, flush_result=True)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 2.0, rate_uL_s=10.0, settle=False)
        self.assertEqual(sleeps, [])
        self.assertEqual(len(c.zp_stage.flush_calls), 0)


# ════════════════════════════════════════════════════════════════════
#  PickAndPlaceManager._settled_pump_move
# ════════════════════════════════════════════════════════════════════

class TestSettledPumpHelper(unittest.TestCase):

    def test_passes_settle_when_supported(self):
        calls = []

        class _Ctrl:
            def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, *,
                             settle=False, compensate=None):
                calls.append((pump, volume_uL, rate_uL_s, settle, compensate))

        # Default: compensate=False (captures + dispense-to-waste safe).
        _settled_pump_move(_Ctrl(), "P1", 1.5, 2.0)
        self.assertEqual(calls, [("P1", 1.5, 2.0, True, False)])

    def test_passes_compensate_when_requested(self):
        calls = []

        class _Ctrl:
            def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, *,
                             settle=False, compensate=None):
                calls.append((pump, volume_uL, rate_uL_s, settle, compensate))

        # Reagent pickups pass compensate=None → auto per the global toggle.
        _settled_pump_move(_Ctrl(), "P1", -1.5, 2.0, compensate=None)
        self.assertEqual(calls, [("P1", -1.5, 2.0, True, None)])

    def test_falls_back_when_settle_unsupported(self):
        calls = []

        class _OldCtrl:
            def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
                calls.append((pump, volume_uL, rate_uL_s))

        # Must not raise — the TypeError from the settle/compensate kwargs is
        # swallowed and the call is retried plain (keeps recording-fake test
        # sequences stable).
        _settled_pump_move(_OldCtrl(), "P1", 1.5, 2.0, compensate=None)
        self.assertEqual(calls, [("P1", 1.5, 2.0)])

    def test_falls_back_to_settle_only_when_compensate_unsupported(self):
        # A controller that supports settle but not compensate degrades to a
        # settle-only move rather than raising.
        calls = []

        class _SettleOnly:
            def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, *,
                             settle=False):
                calls.append((pump, volume_uL, rate_uL_s, settle))

        _settled_pump_move(_SettleOnly(), "P1", -1.5, 2.0, compensate=None)
        self.assertEqual(calls, [("P1", -1.5, 2.0, True)])


# ════════════════════════════════════════════════════════════════════
#  move_pump_uL backlash / compliance compensation
# ════════════════════════════════════════════════════════════════════

class TestBacklashCompensation(unittest.TestCase):
    """v7.5.x: take-up on reversal (+c before the fluid move) + unload on stop
    (−c after), sized per-pump from pump_relief_uL. Auto-fires only for discrete
    actuations (settle=True) when the global toggle is on; compensate=True/False
    force it. c = 0 / toggle off / compensate=False ⇒ no comp. The fake _ctrl
    has no zp_stage.flush_moves, so completion falls back to an open-loop sleep,
    mocked here. uL_to_mm = ×0.3."""

    def _run(self, ctrl, vol, **kw):
        with mock.patch("SupportClasses.StageController.time.sleep"):
            ctrl.move_pump_uL("P1", vol, rate_uL_s=10.0, **kw)

    def test_dispense_brackets_takeup_fluid_unload(self):
        c = _ctrl(settle=0.0, relief_uL=0.5, backlash=True)
        self._run(c, +2.0, settle=True)          # compensate=None → auto (on)
        # take-up +0.5 (fluid dir), fluid +2.0, unload -0.5.
        self.assertEqual(len(c.pump_moves), 3)
        self.assertAlmostEqual(c.pump_moves[0][1], +0.5 * 0.3)   # take-up
        self.assertAlmostEqual(c.pump_moves[1][1], +2.0 * 0.3)   # fluid
        self.assertAlmostEqual(c.pump_moves[2][1], -0.5 * 0.3)   # unload
        # Net plunger travel == the commanded volume.
        self.assertAlmostEqual(sum(m[1] for m in c.pump_moves), 2.0 * 0.3)

    def test_aspirate_brackets_in_fluid_direction(self):
        c = _ctrl(settle=0.0, relief_uL=0.5, backlash=True)
        self._run(c, -2.0, settle=True)
        self.assertEqual(len(c.pump_moves), 3)
        self.assertAlmostEqual(c.pump_moves[0][1], -0.5 * 0.3)   # take-up (−)
        self.assertAlmostEqual(c.pump_moves[1][1], -2.0 * 0.3)   # fluid
        self.assertAlmostEqual(c.pump_moves[2][1], +0.5 * 0.3)   # unload (+)

    def test_no_comp_when_toggle_off(self):
        c = _ctrl(settle=0.0, relief_uL=0.5, backlash=False)
        self._run(c, -2.0, settle=True)          # auto resolves to off
        self.assertEqual(len(c.pump_moves), 1)

    def test_no_comp_when_relief_zero(self):
        c = _ctrl(settle=0.0, relief_uL=0.0, backlash=True)
        self._run(c, -2.0, settle=True)
        self.assertEqual(len(c.pump_moves), 1)

    def test_compensate_false_forces_off_even_with_toggle_on(self):
        # Volume-balanced captures pass compensate=False → exact net-zero kept.
        c = _ctrl(settle=0.0, relief_uL=0.5, backlash=True)
        self._run(c, -2.0, settle=True, compensate=False)
        self.assertEqual(len(c.pump_moves), 1)

    def test_settle_false_never_auto_comps(self):
        # Streamed print path (settle=False) must never bracket even when the
        # toggle is on and a relief value is set.
        c = _ctrl(settle=0.0, relief_uL=0.5, backlash=True)
        self._run(c, -2.0, settle=False)
        self.assertEqual(len(c.pump_moves), 1)

    def test_compensate_true_forces_on_even_when_settle_false(self):
        # A jog click (settle=False) explicitly requests comp → brackets.
        c = _ctrl(settle=0.0, relief_uL=0.5, backlash=True)
        self._run(c, +2.0, settle=False, compensate=True)
        self.assertEqual(len(c.pump_moves), 3)

    def test_relief_reader_and_setter(self):
        c = _ctrl(relief_uL=0.0)
        c.set_pump_relief_uL("P2", 0.9)
        self.assertAlmostEqual(c.pump_relief_uL("P2"), 0.9)
        # Negative clamps to 0.
        c.set_pump_relief_uL("P2", -1.0)
        self.assertEqual(c.pump_relief_uL("P2"), 0.0)

    def test_relief_reader_defaults_zero(self):
        # No per-pump value set → 0 (⇒ no compensation for that pump).
        c = StageController.__new__(StageController)
        c._hardware_config = None
        self.assertEqual(c.pump_relief_uL("P1"), 0.0)


# ════════════════════════════════════════════════════════════════════
#  Quick Print prime default ports in from the hardware config
# ════════════════════════════════════════════════════════════════════

class TestQuickPrintPrimeDefault(unittest.TestCase):

    def _page(self, hw):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage)
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._hw_config = hw
        return page

    def test_reads_config_prime_time(self):
        page = self._page(SimpleNamespace(pump_prime_time_s=0.5))
        self.assertAlmostEqual(page._prime_default_s(), 0.5)

    def test_falls_back_to_constant(self):
        page = self._page(None)
        self.assertAlmostEqual(page._prime_default_s(), page._PREFLOW_S)

    def test_negative_falls_back(self):
        page = self._page(SimpleNamespace(pump_prime_time_s=-2.0))
        self.assertAlmostEqual(page._prime_default_s(), page._PREFLOW_S)


if __name__ == "__main__":
    unittest.main()
