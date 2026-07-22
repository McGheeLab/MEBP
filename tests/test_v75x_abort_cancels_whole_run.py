#!/usr/bin/env python3
"""v7.5.x regression: Abort must cancel the WHOLE print, not just the
current task.

Root cause of the reported bug ("Sometimes abort requested only aborted the
current task rather than the entire run"): Full Print's DEFAULT execution path
is hybrid — ``gui/app.py`` runs a ``HybridPlanExecutor`` on a worker thread.
The executor was created as a LOCAL variable and never registered on the
``PrintManager``, so ``PrintManager.abort()`` (which does
``if self._trajectory_executor: self._trajectory_executor.abort()``) could not
reach it. The executor kept iterating the rest of the plan (every remaining
well / plan step) to completion even though the run was marked ABORTED.

These tests lock:
  1. ``PrintManager.abort()`` forwards to a registered ``_trajectory_executor``
     (the mechanism the fix relies on).
  2. ``HybridPlanExecutor`` actually stops when its abort flag is set.
  3. ``gui/app.py``'s hybrid branch registers the executor on the PrintManager
     (guards the exact regressed line without a full MainWindow boot).
"""

import io
import os
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PrintManager import (  # noqa: E402
    HybridPlanExecutor, PrintManager, PrintSettings, PrintState)


class _FakeController:
    """Minimal stand-in — PrintManager.abort() only reads is_zp_connected."""
    is_zp_connected = False
    is_xy_connected = False


class _FakePlan:
    def __init__(self, steps):
        self.steps = steps


class _DummyStep:
    # step_type is never dereferenced when the abort flag is already set
    step_type = None


class TestPrintManagerAbortForwarding(unittest.TestCase):
    def test_abort_forwards_to_registered_trajectory_executor(self):
        """A registered hybrid/trajectory executor gets its abort flag set by
        PrintManager.abort() — the wiring the Full Print fix depends on."""
        pm = PrintManager(_FakeController())
        settings = PrintSettings()
        executor = HybridPlanExecutor(
            controller=pm.controller, plan=_FakePlan([]), well_model=None,
            plate=None, path_points=[], settings=settings)

        # Simulate app.py's hybrid branch registering the executor.
        pm._trajectory_executor = executor
        pm.state = PrintState.RUNNING  # abort() is gated on RUNNING/PAUSED

        self.assertFalse(executor._abort_flag.is_set())
        pm.abort()
        self.assertTrue(
            executor._abort_flag.is_set(),
            "PrintManager.abort() must set the registered executor's abort flag")
        self.assertEqual(pm.state, PrintState.ABORTED)

    def test_abort_noop_when_not_running(self):
        pm = PrintManager(_FakeController())
        executor = HybridPlanExecutor(
            controller=pm.controller, plan=_FakePlan([]), well_model=None,
            plate=None, path_points=[], settings=PrintSettings())
        pm._trajectory_executor = executor
        pm.state = PrintState.IDLE
        pm.abort()
        self.assertFalse(executor._abort_flag.is_set())


class TestHybridExecutorStopsOnAbort(unittest.TestCase):
    def test_execute_returns_false_and_runs_no_steps_when_aborted(self):
        """With the abort flag pre-set, execute() must bail before running any
        plan step (the top-of-loop check), i.e. cancel the whole run."""
        steps = [_DummyStep(), _DummyStep(), _DummyStep()]
        executor = HybridPlanExecutor(
            controller=_FakeController(), plan=_FakePlan(steps),
            well_model=None, plate=None, path_points=[],
            settings=PrintSettings())

        seen = []
        executor.abort()  # sets _abort_flag
        result = executor.execute(
            pause_event=threading.Event(),
            on_progress=lambda i, t, m: seen.append(i))

        self.assertFalse(result, "aborted execute() must return False")
        self.assertEqual(
            seen, [], "no plan step should run once abort is requested")


class TestAppHybridBranchRegistersExecutor(unittest.TestCase):
    """Full MainWindow boot is intractable headless, so guard the exact
    regressed line by source inspection: the hybrid branch must register the
    HybridPlanExecutor on the PrintManager so pm.abort() can reach it."""

    def test_hybrid_executor_assigned_to_print_manager(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with io.open(os.path.join(here, "..", "gui", "app.py"),
                     encoding="utf-8") as fh:
            app_src = fh.read()
        # The executor local must be handed to the PrintManager.
        self.assertIn(
            "pm._trajectory_executor = executor", app_src,
            "gui/app.py hybrid branch must register the HybridPlanExecutor on "
            "the PrintManager (else Monitor Abort cannot stop the run)")


if __name__ == "__main__":
    unittest.main()
