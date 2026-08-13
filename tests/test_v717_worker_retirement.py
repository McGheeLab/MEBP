"""v7.17.0 tests — a completed background scan must not abort the process.

Operator: *"I just finished the workflow->mosaic builder. it finished it
correctly, then after it was complete python crashed."*

ROOT CAUSE (reproduced deterministically, 5 crashes in 5 runs):
every worker in this app emits its completion signal from **inside** ``run()``
and is constructed with **no Qt parent**, so the page attribute holding it is
the only strong reference and Python owns the C++ object. The completion slot
did::

    def _on_channel_finished(self, ...):
        self._worker = None          # ← last reference

which hands a still-running QThread to the garbage collector: the fluorescence
worker's ``finally`` block then restores the microscope focus (a Nikon Ti COM
move), releases the scope lease and resumes the poller — hundreds of ms of work
*after* the signal was emitted. ``~QThread()`` running under the live thread
aborts the process with **exit 0xC0000409, no traceback, no stderr, no Qt
warning** — i.e. "python just closed", which is exactly what was reported and
why the app log ends mid-session with the scan recorded as successful.

THE FIX: ``gui.worker_retirement.retire_worker`` holds the worker until
``QThread.finished`` is delivered on the GUI thread. Non-blocking on purpose —
``wait()`` would freeze the UI for the length of that focus move.

⚠ The direction of error matters and is asserted below: holding a finished
worker one event-loop turn too long costs a few bytes; releasing one turn too
early kills the process.
"""
import ast
import os
import subprocess
import sys
import textwrap
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QThread, Signal                        # noqa: E402
from PySide6.QtWidgets import QApplication                        # noqa: E402

import gui.worker_retirement as wr                                # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        wr._reset_for_tests()

    def _pump(self, seconds=2.0, until=None):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._app.processEvents()
            if until is not None and until():
                return True
            time.sleep(0.005)
        return until() if until is not None else False


class _SlowFinallyWorker(QThread):
    """The production shape: emit from inside run(), then keep working."""

    done = Signal(object)

    def __init__(self):
        super().__init__()          # deliberately NO parent — Python owns it
        self.finally_ran = False

    def run(self):
        try:
            self.done.emit("payload")
        finally:
            time.sleep(0.25)        # the microscope focus restore / lease release
            self.finally_ran = True


class TestRetireWorkerHoldsUntilFinished(_QtBase):
    def test_worker_is_held_while_run_is_still_executing(self):
        w = _SlowFinallyWorker()
        w.start()
        # Wait until run() has emitted and is inside its finally block.
        self.assertTrue(self._pump(3.0, lambda: w.isRunning() and not w.finally_ran)
                        or w.isRunning(), "worker never got going")
        wr.retire_worker(w)
        self.assertEqual(wr.pending_worker_count(), 1,
                         "a still-running worker must be held, not released")
        self.assertTrue(self._pump(5.0, lambda: not w.isRunning()))

    def test_worker_is_released_once_the_thread_has_finished(self):
        w = _SlowFinallyWorker()
        w.start()
        wr.retire_worker(w)
        released = self._pump(6.0, lambda: wr.pending_worker_count() == 0)
        self.assertTrue(w.finally_ran, "the finally block never completed")
        self.assertTrue(released, "worker was never released after finishing")

    def test_none_and_double_retire_are_safe(self):
        wr.retire_worker(None)
        self.assertEqual(wr.pending_worker_count(), 0)
        w = _SlowFinallyWorker()
        w.start()
        wr.retire_worker(w)
        wr.retire_worker(w)
        self.assertEqual(wr.pending_worker_count(), 1, "held twice")
        self.assertTrue(self._pump(6.0, lambda: wr.pending_worker_count() == 0))

    def test_already_finished_worker_is_not_held_forever(self):
        w = _SlowFinallyWorker()
        w.start()
        self.assertTrue(self._pump(6.0, lambda: w.isFinished()))
        wr.retire_worker(w)
        self.assertTrue(self._pump(3.0, lambda: wr.pending_worker_count() == 0),
                        "a worker retired after it finished was never released")

    def test_retire_does_not_block_the_caller(self):
        """It must not degrade into wait(): the finally block is a real
        microscope move and blocking it freezes the UI."""
        w = _SlowFinallyWorker()
        w.start()
        t0 = time.time()
        wr.retire_worker(w)
        self.assertLess(time.time() - t0, 0.10,
                        "retire_worker blocked — that is wait(), not retirement")
        self.assertTrue(self._pump(6.0, lambda: not w.isRunning()))


# ── The end-to-end proof, in a subprocess so a regression cannot kill the
#    test runner (the failure mode is a hard abort, not an exception). ──

_SCRIPT = textwrap.dedent(
    """
    import os, sys, time
    sys.path.insert(0, r"{repo}")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import QObject, QThread, Signal
    from PySide6.QtWidgets import QApplication
    MODE = sys.argv[1]
    app = QApplication.instance() or QApplication([])

    class W(QThread):
        done = Signal(object)
        def run(self):
            try:
                self.done.emit("x")
            finally:
                for _ in range(20):
                    time.sleep(0.01)
                    _ = [i * i for i in range(5000)]

    class Page(QObject):
        def __init__(self):
            super().__init__()
            self._worker = None
        def start(self):
            self._worker = W()          # no parent -> Python owns it
            self._worker.done.connect(self._on_done)
            self._worker.start()
        def _on_done(self, _p):
            if MODE == "fixed":
                from gui.worker_retirement import retire_worker
                retire_worker(self._worker)
            self._worker = None
            for _ in range(50):
                _ = [i * i for i in range(2000)]

    p = Page(); p.start()
    t0 = time.time()
    while time.time() - t0 < 6.0:
        app.processEvents(); time.sleep(0.005)
    print("survived")
    """
)


def _run_subprocess(mode):
    script = _SCRIPT.format(repo=REPO)
    return subprocess.run([sys.executable, "-c", script, mode],
                          capture_output=True, text=True, timeout=180)


class TestEndToEndProcessSurvival(_QtBase):
    def test_the_bug_really_does_abort_the_process(self):
        """Guard the guard. If this ever stops crashing, the sibling test below
        proves nothing and the whole fix needs re-justifying."""
        r = _run_subprocess("buggy")
        self.assertNotEqual(
            r.returncode, 0,
            "dropping the last reference to a running QThread no longer aborts "
            "— re-verify why retire_worker is needed")
        self.assertNotIn("survived", r.stdout)

    def test_retire_worker_keeps_the_process_alive(self):
        r = _run_subprocess("fixed")
        self.assertEqual(r.returncode, 0,
                         f"crashed with {r.returncode}: {r.stdout}{r.stderr}")
        self.assertIn("survived", r.stdout)


# ── Structural guard: catch a NEW completion slot that forgets. ──

_WORKER_ATTR_HINTS = ("_worker",)
# Functions allowed to clear a worker attribute without retiring it: these
# INITIALISE the attribute at construction time, when no thread has been
# started, so there is no reference to release. Anything that runs in response
# to a worker signal must retire (or have waited).
_ALLOWED_EXACT = {"__init__"}
_ALLOWED_PREFIXES = ("_build",)


def _is_initialiser(name: str) -> bool:
    return name in _ALLOWED_EXACT or name.startswith(_ALLOWED_PREFIXES)


def _clears_worker(fn: ast.AST) -> list[str]:
    out = []
    for node in ast.walk(fn):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant) \
                and node.value.value is None:
            for tgt in node.targets:
                if (isinstance(tgt, ast.Attribute)
                        and any(h in tgt.attr for h in _WORKER_ATTR_HINTS)):
                    out.append(tgt.attr)
    return out


def _calls(fn: ast.AST, name: str) -> bool:
    for node in ast.walk(fn):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Name) and f.id == name:
                return True
            if isinstance(f, ast.Attribute) and f.attr == name:
                return True
    return False


class TestEveryWorkerReleaseIsSafe(_QtBase):
    """Walk the GUI tree: any function that clears a ``*_worker`` attribute must
    either retire it or have waited for the thread first."""

    FILES = [
        "gui/pages/workflows/fluorescence_mosaic_workflow.py",
        "gui/pages/calibration.py",
        "gui/widgets/spheroid_survey_panel.py",
        "gui/widgets/plate_level_wizard.py",
        "gui/widgets/needle_bore_wizard.py",
        "gui/widgets/spheroid_crop_worker.py",
    ]

    def test_no_unsafe_worker_release(self):
        offenders = []
        checked = 0
        for rel in self.FILES:
            path = os.path.join(REPO, rel)
            if not os.path.exists(path):
                continue
            with open(path, encoding="utf-8") as fh:
                tree = ast.parse(fh.read())
            for fn in ast.walk(tree):
                if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                cleared = _clears_worker(fn)
                if not cleared:
                    continue
                checked += 1
                if _is_initialiser(fn.name):
                    continue
                if _calls(fn, "retire_worker") or _calls(fn, "wait"):
                    continue
                offenders.append(f"{rel}::{fn.name} clears {cleared}")
        self.assertGreaterEqual(
            checked, 5, "the AST scan found almost nothing — matcher is broken")
        self.assertEqual(
            offenders, [],
            "these clear a worker reference without retiring it or waiting for "
            "the thread — a completed scan can abort the process:\n  "
            + "\n  ".join(offenders))


class TestProductionSitesAreWired(_QtBase):
    """Pin the specific slots the operator's crash came from."""

    EXPECTED = [
        ("gui/pages/workflows/fluorescence_mosaic_workflow.py",
         "_on_channel_finished"),
        ("gui/pages/workflows/fluorescence_mosaic_workflow.py",
         "_on_channel_failed"),
        ("gui/pages/calibration.py", "_ploc_on_mosaic_finished"),
        ("gui/pages/calibration.py", "_ploc_on_mosaic_failed"),
        ("gui/widgets/spheroid_survey_panel.py", "_on_detect_done"),
        ("gui/widgets/spheroid_survey_panel.py", "_on_detect_failed"),
    ]

    def test_each_completion_slot_retires_its_worker(self):
        for rel, fname in self.EXPECTED:
            with open(os.path.join(REPO, rel), encoding="utf-8") as fh:
                tree = ast.parse(fh.read())
            found = [fn for fn in ast.walk(tree)
                     if isinstance(fn, ast.FunctionDef) and fn.name == fname]
            self.assertTrue(found, f"{rel}::{fname} not found")
            self.assertTrue(
                _calls(found[0], "retire_worker"),
                f"{rel}::{fname} does not call retire_worker — a completed scan "
                f"can abort the process")


if __name__ == "__main__":       # pragma: no cover
    unittest.main()
