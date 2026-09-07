"""Tests for v7.20 — THE NEEDLE MUST NEVER DESCEND BEFORE XY HAS ARRIVED.

BENCH FAILURE THIS PREVENTS: during a print the needle started moving DOWN
before the stage reached the print location, and it was destroyed against the
plate.

ROOT CAUSE (one defect, five sites): the XY-arrival confirmation was computed
and then THROWN AWAY, so the descent that follows ran regardless.

  * ``PrintManager._wait_for_xy_settle`` returned ``None`` on EVERY path — no
    caller *could* act on it — and its own timeout branch even carried the note
    "a settle TIMEOUT silently continues execution … a prime suspect for desync
    bugs". The discrete plan is ``MOVE_XY → MOVE_Z``, so the descent ran next.
  * ``StageController.safe_travel_to`` logged "XY arrival timed out — proceeding
    with Z descent anyway" and lowered the needle. Note the asymmetry it had:
    an unconfirmed Z RETRACT already aborted, an unconfirmed XY ARRIVAL did not.
  * ``HybridPlanExecutor._execute_print_step`` warned, then descended.
  * ``DirectCommandExecutor.travel_to_well`` warned, then descended.
  * ``SimplePrintManager._confirmed_xy`` — named for the confirmation it
    discarded — let MOVE_XY hand off to MOVE_Z's descent.

The invariant under test is behavioural and identical everywhere: **when XY
arrival is not confirmed, no Z-lowering command is issued.** Each test therefore
asserts on the MOTION COMMANDS actually sent, not on a return value — a fake
that merely agrees with the code would prove nothing.

``ZDIR = -1`` on ME3B V1 (the needle descends as raw Z INCREASES), so "descend"
is asserted in the polarity-safe HEIGHT frame rather than by the sign of Z.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Don't write real print-execution JSONL into logs/prints/ from the suite.
os.environ["MEBP_PRINT_LOG"] = "0"

from SupportClasses.StageController import StageController
import SupportClasses.PrintManager as PM
from SupportClasses.PrintManager import (
    CommandType, PrintCommand, PrintJob, PrintManager, PrintSettings,
    PrintState, DirectCommandExecutor, MoveNotConfirmedError,
)


# ── Fakes ───────────────────────────────────────────────────────────


class _FakeCtrl:
    """Records every motion command; XY arrival is switchable.

    ``xy_arrives=False`` models the real failure: the stage does not reach the
    commanded position within the timeout (a stalled/slow stage, or the
    documented Prior stale-``R``-ack read failure that makes every position poll
    return garbage until the wait gives up).
    """

    def __init__(self, xy_arrives=True, xy_pos_mm=(0.0, 0.0)):
        self.is_xy_connected = True
        self.is_zp_connected = True
        self._xy_arrives = xy_arrives
        self._xy_pos_mm = xy_pos_mm
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self.z_moves = []          # every commanded Z target (zero-ref mm)
        self.xy_moves = []
        self.pump_moves = []
        self.xy_stage = MagicMock()
        self.zp_stage = MagicMock()
        self.zp_stage.flush_moves = MagicMock(return_value=True)
        self._zp_insert_feedrate = 100.0
        self._zp_retract_feedrate = 250.0
        self.position_logger = None
        self.safety_limits = MagicMock(max_xy_speed=50000.0)

    # -- position reads --
    def get_xy_position(self, cached=True):
        # µm, absolute stage frame.
        return (self._xy_pos_mm[0] * 1000.0, self._xy_pos_mm[1] * 1000.0, 0.0)

    def get_xy_position_mm(self, cached=True):
        return (self._xy_pos_mm[0], self._xy_pos_mm[1], 0.0)

    def get_zp_position(self, cached=True):
        return (0.0, 0.0, 0.0, 0.0)

    def get_zp_position_logical_tuple(self, cached=True):
        return (0.0, 0.0, 0.0, 0.0)

    # -- motion --
    def move_xy_absolute(self, x, y, from_zero_ref=True, fast=False):
        self.xy_moves.append((x, y))
        if self._xy_arrives:
            self._xy_pos_mm = (x, y)

    def move_z_absolute(self, z, from_zero_ref=True, feedrate_mm_min=None):
        self.z_moves.append(float(z))
        return float(z)

    def emit_descent_moves(self, z, fr=None):
        self.z_moves.append(float(z))

    def ensure_retracted_to(self, z, timeout_s=None, feedrate_mm_min=None):
        # Raise-only: recorded separately from descents so the tests can tell a
        # protective retract from the dangerous lowering.
        self.z_moves.append(float(z))
        return True

    def wait_for_xy_arrival(self, x, y, tolerance_mm=0.1, timeout_s=10.0):
        return self._xy_arrives

    def wait_for_z_arrival(self, z, tolerance_mm=0.05, timeout_s=10.0,
                           abort_event=None):
        return True

    def move_pump_uL(self, pump, vol, rate=None, **kw):
        self.pump_moves.append((pump, vol))

    # -- misc plumbing the handlers touch --
    def suspend_position_poller(self):
        pass

    def resume_position_poller(self):
        pass

    def set_print_floor_active(self, on):
        pass

    def estimate_gentle_z_time_s(self, z, fr, cur_zref_mm=None):
        return 1.0


def _settings():
    s = PrintSettings()
    s.travel_z_height = -20.0     # retracted (ZDIR=-1 → height +20)
    s.print_z_height = 5.0        # lowered   (height -5)
    return s


def _pm(ctrl, settings=None, fast_timeout=True):
    pm = PrintManager.__new__(PrintManager)
    pm.controller = ctrl
    pm.exec_logger = None
    pm.recorder = None
    pm._last_xy_target = None
    pm._active_pump = "P1"
    pm._current_step = 0
    import threading
    pm._abort_flag = threading.Event()
    pm._pause_event = threading.Event()
    pm._pause_event.set()
    pm.job = PrintJob(name="t", commands=[],
                      settings=settings or _settings())
    if fast_timeout:
        # Keep the suite quick: the production helper is exercised on its own in
        # TestDistanceScaledTimeout; here we only care about the GATE.
        pm._xy_settle_timeout_s = lambda *a, **k: 0.2
    return pm


def _descents(ctrl, settings):
    """Z commands that LOWER the needle, in the polarity-safe height frame.

    ZDIR=-1 here, so a *larger* raw Z is physically lower. Comparing against the
    travel height rather than against zero is what keeps this test correct on
    either polarity.
    """
    from SupportClasses.StageController import ZDIR
    travel_h = ZDIR * settings.travel_z_height
    return [z for z in ctrl.z_moves if (ZDIR * z) < travel_h - 1e-9]


# ── The core invariant: the discrete print path ─────────────────────


class TestDiscreteMoveXyGatesTheDescent(unittest.TestCase):
    """``MOVE_XY`` must not hand off to ``MOVE_Z`` unless XY arrived."""

    def test_unconfirmed_xy_raises_and_commands_no_descent(self):
        ctrl = _FakeCtrl(xy_arrives=False)
        s = _settings()
        pm = _pm(ctrl, s)

        with self.assertRaises(RuntimeError) as cm:
            pm._execute_command(PrintCommand(
                type=CommandType.MOVE_XY, params={"x": 10.0, "y": 12.0}))

        # The message must name the danger, not just "timeout".
        self.assertIn("not confirmed", str(cm.exception).lower())
        # THE ASSERTION THAT MATTERS: nothing was lowered.
        self.assertEqual(_descents(ctrl, s), [],
                         "a descent was commanded despite unconfirmed XY")

    def test_confirmed_xy_proceeds_normally(self):
        """Guard-the-guard: the gate must not block the healthy path."""
        ctrl = _FakeCtrl(xy_arrives=True)
        s = _settings()
        pm = _pm(ctrl, s)
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_XY, params={"x": 10.0, "y": 12.0}))
        self.assertEqual(ctrl.xy_moves[-1], (10.0, 12.0))
        # And the descent that follows IS allowed.
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_Z, params={"z": s.print_z_height}))
        self.assertIn(s.print_z_height, ctrl.z_moves)

    def test_settle_helper_reports_its_verdict(self):
        """``_wait_for_xy_settle`` returned None on every path — no caller
        COULD act on it. It must now distinguish arrival from giving up."""
        ok_ctrl = _FakeCtrl(xy_arrives=True, xy_pos_mm=(4.0, 4.0))
        self.assertIs(_pm(ok_ctrl)._wait_for_xy_settle(4.0, 4.0, timeout=0.2),
                      True)

        bad_ctrl = _FakeCtrl(xy_arrives=False, xy_pos_mm=(0.0, 0.0))
        self.assertIs(_pm(bad_ctrl)._wait_for_xy_settle(9.0, 9.0, timeout=0.2),
                      False)

    def test_abort_during_settle_is_not_an_arrival(self):
        ctrl = _FakeCtrl(xy_arrives=False)
        pm = _pm(ctrl)
        pm._abort_flag.set()
        self.assertIs(pm._wait_for_xy_settle(9.0, 9.0, timeout=0.2), False)


class TestDescentHasItsOwnIndependentGate(unittest.TestCase):
    """``MOVE_Z`` re-checks XY at the point of danger.

    ``MOVE_XY`` already refuses to hand off, so this is redundant on a
    well-formed plan — deliberately. The original failure had exactly ONE
    enforcement point and it was discarded, so the descent now verifies for
    itself and cannot be bypassed by anything upstream.
    """

    def test_descent_refused_when_stage_is_not_at_the_confirmed_target(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        s = _settings()
        pm = _pm(ctrl, s)
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_XY, params={"x": 10.0, "y": 12.0}))
        before = list(ctrl.z_moves)

        # The stage drifts far away (stall / soft-limit clamp / lost steps).
        ctrl._xy_pos_mm = (40.0, 12.0)

        with self.assertRaises(RuntimeError) as cm:
            pm._execute_command(PrintCommand(
                type=CommandType.MOVE_Z, params={"z": s.print_z_height}))
        self.assertIn("refusing to lower", str(cm.exception).lower())
        self.assertEqual(ctrl.z_moves, before,
                         "MOVE_Z lowered the needle at the wrong XY")

    def test_small_settle_jitter_does_not_false_abort(self):
        """The backstop is loose on purpose — MOVE_XY owns precision."""
        ctrl = _FakeCtrl(xy_arrives=True)
        s = _settings()
        pm = _pm(ctrl, s)
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_XY, params={"x": 10.0, "y": 12.0}))
        ctrl._xy_pos_mm = (10.05, 12.05)          # 70 µm — well within tol
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_Z, params={"z": s.print_z_height}))
        self.assertIn(s.print_z_height, ctrl.z_moves)

    def test_print_path_clears_the_record_so_it_cannot_false_abort(self):
        """A PRINT_PATH drives XY across the well; comparing a later descent
        against the pre-path target would be a FALSE abort."""
        ctrl = _FakeCtrl(xy_arrives=True)
        pm = _pm(ctrl)
        pm._last_xy_target = (10.0, 12.0)
        pm._execute_command(PrintCommand(
            type=CommandType.PRINT_PATH,
            params={"points": [], "pump": "P1"}))
        self.assertIsNone(pm._last_xy_target)

    def test_a_fresh_job_does_not_inherit_the_previous_target(self):
        """A reused PrintManager must not cross-check this job's first descent
        against the PREVIOUS job's position (the stage has been jogged since)."""
        from unittest.mock import patch
        ctrl = _FakeCtrl(xy_arrives=True)
        pm = _pm(ctrl)
        pm._last_xy_target = (99.0, 99.0)
        pm.state = PrintState.IDLE
        pm._thread = None
        pm._start_time = None
        pm.on_state_changed = None
        pm.on_progress = None
        pm.exec_logger = None
        # Don't actually run the print thread — only the reset is under test.
        with patch("threading.Thread") as _T:
            _T.return_value = MagicMock()
            pm.start()
        self.assertIsNone(pm._last_xy_target)


# ── safe_travel_to: the primitive every workflow funnels through ────


class TestSafeTravelToRefusesTheDescent(unittest.TestCase):
    def _ctrl(self, xy_arrives):
        # ``is_xy_connected`` / ``is_zp_connected`` are read-only properties on
        # the real class, so a connected controller is modelled by a subclass
        # rather than by assignment. Everything else is the PRODUCTION
        # StageController — safe_travel_to's real body is what runs.
        class _Connected(StageController):
            is_xy_connected = property(lambda self: True)
            is_zp_connected = property(lambda self: True)

        c = _Connected.__new__(_Connected)
        c.xy_stage = MagicMock()
        c.zp_stage = MagicMock()
        c.zp_stage.flush_moves = MagicMock(return_value=True)
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        c._pos_poller = MagicMock()
        c._zp_retract_feedrate = 250.0
        c._zp_insert_feedrate = 100.0
        c._min_travel_z_mm = None
        c._retract_slow_dist_mm = 0.0
        c._descend_slow_dist_mm = 0.0
        c.descents = []
        c._retract_z_slow_then_fast = MagicMock(return_value=True)
        c._descend_z_moves_only = lambda cur, tgt, fr: c.descents.append(tgt)
        c.move_xy_absolute = MagicMock()
        c.wait_for_z_arrival = MagicMock(return_value=True)
        c.wait_for_xy_arrival = MagicMock(return_value=xy_arrives)
        c.effective_z_target_zref = lambda z: z
        c.estimate_gentle_z_time_s = lambda *a, **k: 1.0
        return c

    def test_unconfirmed_arrival_means_no_descent(self):
        c = self._ctrl(xy_arrives=False)
        ok = c.safe_travel_to(1000.0, 2000.0, safe_z_mm=-20.0,
                              target_z_mm=5.0, xy_timeout_s=0.1)
        self.assertFalse(ok)
        self.assertEqual(c.descents, [],
                         "safe_travel_to lowered the needle after an "
                         "unconfirmed XY arrival")

    def test_confirmed_arrival_still_descends(self):
        c = self._ctrl(xy_arrives=True)
        ok = c.safe_travel_to(1000.0, 2000.0, safe_z_mm=-20.0,
                              target_z_mm=5.0, xy_timeout_s=0.1)
        self.assertTrue(ok)
        self.assertEqual(c.descents, [5.0])

    def test_retract_and_arrival_failures_are_now_symmetric(self):
        """An unconfirmed RETRACT already aborted; an unconfirmed ARRIVAL is no
        less dangerous and must abort the same way."""
        no_retract = self._ctrl(xy_arrives=True)
        no_retract._retract_z_slow_then_fast = MagicMock(return_value=False)
        self.assertFalse(no_retract.safe_travel_to(
            1.0, 2.0, safe_z_mm=-20.0, target_z_mm=5.0))
        self.assertEqual(no_retract.descents, [])

        no_arrival = self._ctrl(xy_arrives=False)
        self.assertFalse(no_arrival.safe_travel_to(
            1.0, 2.0, safe_z_mm=-20.0, target_z_mm=5.0, xy_timeout_s=0.1))
        self.assertEqual(no_arrival.descents, [])


# ── The hybrid / blocking executors ─────────────────────────────────


class TestDirectExecutorPrimitivesRaise(unittest.TestCase):
    """v7.20: the blocking primitives RAISE rather than return a status a
    caller may ignore.

    An audit found the confirmation discarded at a dozen call sites
    (``raise_z`` before every service travel, ``move_z`` before every well
    entry, ``travel_to_well`` before every dispense). Patching each was not the
    fix — making the result impossible to ignore is.
    """

    def test_unconfirmed_xy_raises(self):
        ctrl = _FakeCtrl(xy_arrives=False)
        with self.assertRaises(MoveNotConfirmedError):
            DirectCommandExecutor(ctrl).move_xy(10.0, 12.0, timeout_s=0.1)

    def test_unconfirmed_retract_raises_naming_the_drag_hazard(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.ensure_retracted_to = lambda *a, **k: False
        with self.assertRaises(MoveNotConfirmedError) as cm:
            DirectCommandExecutor(ctrl).raise_z(-20.0)
        self.assertIn("drag", str(cm.exception).lower())

    def test_unconfirmed_z_raises(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.wait_for_z_arrival = lambda *a, **k: False
        with self.assertRaises(MoveNotConfirmedError):
            DirectCommandExecutor(ctrl).move_z(5.0, timeout_s=0.1)

    def test_travel_to_well_does_not_lower_when_xy_is_unconfirmed(self):
        ctrl = _FakeCtrl(xy_arrives=False)
        s = _settings()
        with self.assertRaises(MoveNotConfirmedError):
            DirectCommandExecutor(ctrl).travel_to_well(10.0, 12.0, s)
        self.assertEqual(_descents(ctrl, s), [],
                         "travel_to_well descended into the well without a "
                         "confirmed XY arrival")

    def test_travel_to_well_does_not_move_xy_when_retract_is_unconfirmed(self):
        """The drag hazard: Phase 2 must not start if Phase 1 is unconfirmed."""
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.ensure_retracted_to = lambda *a, **k: False
        s = _settings()
        with self.assertRaises(MoveNotConfirmedError):
            DirectCommandExecutor(ctrl).travel_to_well(10.0, 12.0, s)
        self.assertEqual(ctrl.xy_moves, [],
                         "XY travelled while the retract was unconfirmed — "
                         "this drags the needle across the plate")

    def test_confirmed_travel_still_lowers(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        s = _settings()
        s.top_z_height = 0.0
        d = DirectCommandExecutor(ctrl)
        self.assertTrue(d.travel_to_well(10.0, 12.0, s))
        self.assertIn(s.print_z_height, ctrl.z_moves)

    def test_service_step_cannot_dispense_after_an_unconfirmed_travel(self):
        """``_execute_service`` ignored ``travel_to_well``'s result, so a failed
        travel was followed by a dispense — into whatever well the stage was
        actually over, at whatever height."""
        from SupportClasses.PrintManager import HybridPlanExecutor
        import threading
        ctrl = _FakeCtrl(xy_arrives=False)
        s = _settings()
        ex = HybridPlanExecutor.__new__(HybridPlanExecutor)
        ex.controller, ex.settings, ex.exec_logger = ctrl, s, None
        ex._abort_flag = threading.Event()
        ex._find_well = lambda role: ("B1", 0.0, 0.0)
        ex._well_xy_mm = lambda name: (10.0, 12.0)
        d = DirectCommandExecutor(ctrl)

        with self.assertRaises(MoveNotConfirmedError):
            ex._execute_service(d, MagicMock(volume_uL=50.0), "P1", "waste")
        self.assertEqual(ctrl.pump_moves, [],
                         "dispensed after an unconfirmed travel")


class TestHybridPrintStep(unittest.TestCase):
    """``HybridPlanExecutor._execute_print_step`` — the plan-driven print path.

    ⚠ This class exists because a mutation SURVIVED without it: removing the
    guard from ``_execute_print_step`` left every test green, so that path was
    protected by code no test exercised. Covering ``travel_to_well`` was not the
    same thing.
    """

    def _executor(self, ctrl, settings):
        from SupportClasses.PrintManager import HybridPlanExecutor
        import threading
        ex = HybridPlanExecutor.__new__(HybridPlanExecutor)
        ex.controller = ctrl
        ex.settings = settings
        ex.path_points = [(0.0, 0.0)]
        ex.exec_logger = None
        ex.recorder = None
        ex.hw_config = None
        ex.plate = None
        ex.well_model = None
        ex.plan = None
        ex._abort_flag = threading.Event()
        ex._pause_event = threading.Event()
        ex._pause_event.set()
        ex._well_xy_mm = lambda name: (10.0, 12.0)
        return ex

    def test_unconfirmed_travel_aborts_without_descending(self):
        ctrl = _FakeCtrl(xy_arrives=False)
        s = _settings()
        s.top_z_height = 0.0
        s.dwell_after_move = 0
        ex = self._executor(ctrl, s)
        step = MagicMock(target_wells=["A1"])

        with self.assertRaises(MoveNotConfirmedError):
            ex._execute_print_step(step, "P1", ex._pause_event, None)

        self.assertEqual(_descents(ctrl, s), [],
                         "the hybrid print step lowered the needle into the "
                         "well without a confirmed XY arrival")

    def test_the_fault_is_not_swallowed_by_any_broad_handler(self):
        """An unconfirmed MOVE must never be caught by a blanket
        ``except Exception``. Both hybrid paths wrap their bodies in one, and
        swallowing there would log the fault and carry on to the next well —
        which is how the needle gets broken on THAT one.

        Checked by AST over the real source (a substring search matches an
        unrelated inner handler and proved nothing — it passed while the
        ordering was untested).
        """
        import ast
        import inspect
        import textwrap
        from SupportClasses.PrintManager import HybridPlanExecutor

        for fn in (HybridPlanExecutor._execute_print_step,
                   HybridPlanExecutor.execute):
            tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
            broad_guarded = False
            for node in ast.walk(tree):
                if not isinstance(node, ast.Try):
                    continue
                names = []
                for h in node.handlers:
                    t = h.type
                    names.append(t.id if isinstance(t, ast.Name)
                                 else getattr(t, "attr", None))
                if "Exception" not in names:
                    continue
                # A try that catches Exception must let MoveNotConfirmedError
                # through FIRST, or not wrap any motion at all.
                body = ast.dump(ast.Module(body=node.body, type_ignores=[]))
                if not any(k in body for k in ("move_xy", "move_z", "raise_z",
                                               "travel_to_well")):
                    continue  # not a motion-wrapping try
                self.assertIn("MoveNotConfirmedError", names,
                              f"{fn.__name__}: a blanket 'except Exception' "
                              "wraps motion without re-raising "
                              "MoveNotConfirmedError first")
                idx = names.index("MoveNotConfirmedError")
                self.assertLess(idx, names.index("Exception"),
                                f"{fn.__name__}: the specific handler must "
                                "precede the broad one")
                # ⚠ Ordering alone is NOT enough — a mutation that replaced the
                # handler's `raise` with `pass` kept it first and SURVIVED.
                # The body must actually re-raise.
                self.assertTrue(
                    any(isinstance(st, ast.Raise) and st.exc is None
                        for st in ast.walk(node.handlers[idx])),
                    f"{fn.__name__}: the MoveNotConfirmedError handler does "
                    "not re-raise — the fault is swallowed and the plan "
                    "carries on to the next descent")
                broad_guarded = True
            self.assertTrue(broad_guarded,
                            f"{fn.__name__}: found no motion-wrapping "
                            "try/except — the guard may have moved; re-check")

    def test_confirmed_travel_still_descends(self):
        """Guard-the-guard: the skip must not fire on the healthy path."""
        ctrl = _FakeCtrl(xy_arrives=True)
        s = _settings()
        s.top_z_height = 0.0
        s.dwell_after_move = 0
        ex = self._executor(ctrl, s)
        step = MagicMock(target_wells=["A1"])

        ex._execute_print_step(step, "P1", ex._pause_event, None)

        self.assertIn(s.print_z_height, ctrl.z_moves,
                      "the healthy path no longer descends to print height")


class TestSimplePrintManagerGate(unittest.TestCase):
    def test_confirmed_xy_returns_its_verdict(self):
        from SupportClasses.SimplePrintManager import SimplePrintManager
        spm = SimplePrintManager.__new__(SimplePrintManager)
        spm.controller = _FakeCtrl(xy_arrives=False)
        spm._XY_TIMEOUT_S = 0.1
        self.assertIs(spm._confirmed_xy(1.0, 2.0, 10.0), False)

        spm.controller = _FakeCtrl(xy_arrives=True)
        self.assertIs(spm._confirmed_xy(1.0, 2.0, 10.0), True)

    def test_move_xy_raises_before_the_descent(self):
        from SupportClasses.SimplePrintManager import SimplePrintManager
        import threading
        ctrl = _FakeCtrl(xy_arrives=False)
        s = _settings()
        spm = SimplePrintManager.__new__(SimplePrintManager)
        spm.controller = ctrl
        spm._XY_TIMEOUT_S = 0.1
        spm._Z_TIMEOUT_S = 0.1
        spm._active_pump = "P1"
        spm._abort_flag = threading.Event()
        spm.job = PrintJob(name="t", commands=[], settings=s)

        with self.assertRaises(RuntimeError):
            spm._exec(PrintCommand(
                type=CommandType.MOVE_XY, params={"x": 10.0, "y": 12.0}))
        self.assertEqual(_descents(ctrl, s), [])


class TestDiscreteZMovesAreConfirmedNotSlept(unittest.TestCase):
    """``TRAVEL_UP`` (fallback), ``TRAVEL_DOWN`` and ``MOVE_Z_REL`` commanded Z
    and then ``time.sleep(0.3)`` — a fixed guess unrelated to how long the move
    takes, with no verification. The next command ran regardless, which after a
    DESCENT means extruding at an unknown height.
    """

    def _pm_for(self, ctrl, s):
        pm = _pm(ctrl, s)
        pm.exec_logger = None
        return pm

    def test_travel_up_raises_when_the_retract_is_unconfirmed(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.ensure_retracted_to = lambda *a, **k: False
        s = _settings()
        with self.assertRaises(MoveNotConfirmedError) as cm:
            self._pm_for(ctrl, s)._execute_command(
                PrintCommand(type=CommandType.TRAVEL_UP))
        self.assertIn("drag", str(cm.exception).lower())

    def test_travel_down_raises_when_the_descent_is_unconfirmed(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.wait_for_z_arrival = lambda *a, **k: False
        s = _settings()
        pm = self._pm_for(ctrl, s)
        with self.assertRaises(MoveNotConfirmedError):
            pm._execute_command(PrintCommand(type=CommandType.TRAVEL_DOWN))

    def test_travel_down_is_fine_when_confirmed(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        s = _settings()
        pm = self._pm_for(ctrl, s)
        pm._execute_command(PrintCommand(type=CommandType.TRAVEL_DOWN))
        self.assertIn(s.print_z_height, ctrl.z_moves)

    def test_move_z_rel_raises_when_unconfirmed(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.wait_for_z_arrival = lambda *a, **k: False
        ctrl.move_z_relative = lambda d, f=None: ctrl.z_moves.append(("rel", d))
        ctrl.zp_logical_value = lambda pos, ax: 0.0
        s = _settings()
        pm = self._pm_for(ctrl, s)
        with self.assertRaises(MoveNotConfirmedError):
            pm._execute_command(PrintCommand(
                type=CommandType.MOVE_Z_REL, params={"distance": -1.0}))

    def test_unreadable_height_degrades_to_the_legacy_settle(self):
        """A relative move has no absolute target of its own; if the current
        height cannot be read we must not invent a failure."""
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.move_z_relative = lambda d, f=None: ctrl.z_moves.append(("rel", d))
        ctrl.zp_logical_value = lambda pos, ax: None      # unreadable
        s = _settings()
        pm = self._pm_for(ctrl, s)
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_Z_REL, params={"distance": -1.0}))
        self.assertIn(("rel", -1.0), ctrl.z_moves)


class TestPickAndPlaceActsOnTheTravelVerdict(unittest.TestCase):
    """``_safe_move_to`` discarded ``safe_travel_to``'s verdict, so a refused or
    unconfirmed travel was followed by a descent / aspirate / dispense.

    Its intra-well sibling ``_intra_well_move`` already raised — the longer,
    more dangerous inter-well travel must not use a weaker policy.
    """

    def _executor(self, travel_result):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex.controller = _FakeCtrl(xy_arrives=True)
        ex.hw_config = None
        ex.safe_z_mm = -20.0
        ex.operating_z_mm = 5.0
        ex.z_timeout_s = 1.0
        ex.xy_timeout_s = 1.0
        ex._current_well = ""
        ex.intra_well_retract_mm = 1.0
        import threading
        ex._abort_flag = threading.Event()
        ex._safe_travel = lambda **kw: travel_result
        return ex

    def _target(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceTarget
        return PickPlaceTarget(target_id="T1", x_um=1000.0, y_um=2000.0,
                               well_name="A1")

    def test_refused_travel_aborts(self):
        from SupportClasses.PickAndPlaceManager import AbortException
        ex = self._executor(travel_result=False)
        with self.assertRaises(AbortException) as cm:
            ex._safe_move_to(self._target(), target_z_mm=5.0)
        self.assertIn("not confirmed", str(cm.exception).lower())

    def test_confirmed_travel_proceeds(self):
        ex = self._executor(travel_result=True)
        ex._safe_move_to(self._target(), target_z_mm=5.0)
        self.assertEqual(ex._current_well, "A1")

    def test_a_stub_that_cannot_report_is_not_treated_as_a_failure(self):
        """'Absent' degrades to permitted — the convention `_wait_xy_arrival_um`
        already documents. Only an explicit False is a failure."""
        ex = self._executor(travel_result=None)
        ex._safe_move_to(self._target(), target_z_mm=5.0)
        self.assertEqual(ex._current_well, "A1")


# ── The timeout must not create a NEW failure mode ──────────────────


class TestDistanceScaledTimeout(unittest.TestCase):
    """Failing closed only helps if healthy long moves still pass.

    A flat 10 s on a stage measured at ~2.5× slower than commanded would turn
    legitimate full-plate traverses into spurious aborts — trading a broken
    needle for an unusable machine.
    """

    def test_a_long_move_gets_more_time_than_a_short_one(self):
        pm = _pm(_FakeCtrl(xy_arrives=True, xy_pos_mm=(0.0, 0.0)),
                 fast_timeout=False)
        near = pm._xy_settle_timeout_s(1.0, 0.0)
        far = pm._xy_settle_timeout_s(120.0, 0.0)
        self.assertGreater(far, near)

    def test_it_never_drops_below_the_floor_or_above_the_cap(self):
        pm = _pm(_FakeCtrl(xy_arrives=True, xy_pos_mm=(0.0, 0.0)),
                 fast_timeout=False)
        self.assertGreaterEqual(pm._xy_settle_timeout_s(0.0, 0.0), 10.0)
        self.assertLessEqual(pm._xy_settle_timeout_s(1e6, 1e6), 45.0)

    def test_an_unreadable_position_degrades_to_the_floor(self):
        ctrl = _FakeCtrl(xy_arrives=True)
        ctrl.get_xy_position_mm = lambda cached=True: (None, None, None)
        self.assertEqual(
            _pm(ctrl, fast_timeout=False)._xy_settle_timeout_s(50.0, 50.0),
            10.0)


if __name__ == "__main__":
    unittest.main()
