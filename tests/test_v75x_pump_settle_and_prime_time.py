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
    _PUMP_MOVE_WAIT_CAP_S,
    _PUMP_SETTLE_FALLBACK_RATE_UL_S,
)
from SupportClasses.PickAndPlaceManager import _settled_pump_move


# ── Minimal fakes ──────────────────────────────────────────────────────

class _FakePumpCfg:
    is_configured = True

    def uL_to_mm(self, v):
        return v * 0.3

    def feedrate_uL_s_to_mm_min(self, r):
        return r * 18.0


class _FakeHW:
    def __init__(self, settle=0.0, prime=0.25):
        self.pump_settle_time_s = settle
        self.pump_prime_time_s = prime
        self.pumps = {p: _FakePumpCfg() for p in ("P1", "P2", "P3")}


def _ctrl(settle=0.0):
    """StageController with just enough state for move_pump_uL.

    move_pump_relative is monkeypatched to a recorder so the test isolates the
    settle/completion-wait logic (the relative-move safety machinery is covered
    elsewhere). safety_limits.enabled = False skips the flow-rate clamp branch.
    """
    c = StageController.__new__(StageController)
    c._hardware_config = _FakeHW(settle=settle)
    c.safety_limits = SafetyLimits()
    c.safety_limits.enabled = False
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
        c = _ctrl(settle=0.2)
        sleeps = self._run(c, settle=True)
        self.assertEqual(len(sleeps), 3)            # pre, complete, post
        self.assertAlmostEqual(sleeps[0], 0.2)      # pre-move settle
        self.assertAlmostEqual(sleeps[1], 0.3)      # completion wait
        self.assertAlmostEqual(sleeps[2], 0.2)      # post-move settle
        self.assertEqual(len(c.pump_moves), 1)

    def test_settle_true_zero_dwell_still_waits_for_completion(self):
        # settle time 0 but settle=True → only the completion wait (preserves
        # the legacy EXTRUDE blocking semantics this path subsumes).
        c = _ctrl(settle=0.0)
        sleeps = self._run(c, settle=True)
        self.assertEqual(len(sleeps), 1)
        self.assertAlmostEqual(sleeps[0], 0.3)

    def test_completion_wait_is_capped(self):
        # Huge volume / tiny rate → the completion wait clamps at the cap.
        c = _ctrl(settle=0.0)
        sleeps = []
        with mock.patch("SupportClasses.StageController.time.sleep",
                        side_effect=sleeps.append):
            c.move_pump_uL("P1", 1e6, rate_uL_s=0.01, settle=True)
        self.assertEqual(len(sleeps), 1)
        self.assertAlmostEqual(sleeps[0], _PUMP_MOVE_WAIT_CAP_S)

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
#  PickAndPlaceManager._settled_pump_move
# ════════════════════════════════════════════════════════════════════

class TestSettledPumpHelper(unittest.TestCase):

    def test_passes_settle_when_supported(self):
        calls = []

        class _Ctrl:
            def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, *,
                             settle=False):
                calls.append((pump, volume_uL, rate_uL_s, settle))

        _settled_pump_move(_Ctrl(), "P1", 1.5, 2.0)
        self.assertEqual(calls, [("P1", 1.5, 2.0, True)])

    def test_falls_back_when_settle_unsupported(self):
        calls = []

        class _OldCtrl:
            def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
                calls.append((pump, volume_uL, rate_uL_s))

        # Must not raise — the TypeError from the settle kwarg is swallowed and
        # the call is retried plain (keeps recording-fake test sequences stable).
        _settled_pump_move(_OldCtrl(), "P1", 1.5, 2.0)
        self.assertEqual(calls, [("P1", 1.5, 2.0)])


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
