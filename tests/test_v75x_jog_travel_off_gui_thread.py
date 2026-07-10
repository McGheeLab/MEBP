"""test_v75x_jog_travel_off_gui_thread.py — click-to-travel no longer freezes.

The Jog page (and the Spheroid / Cell Targeting / Cell Labeling workflow pages,
and the Absolute-Go-To panel) used to call ``StageController.safe_travel_to``
SYNCHRONOUSLY inside a Qt signal handler. When the needle is DOWN, step-1 Z
retract is a real multi-second (up to ~60-120 s, or effectively forever on a
stuck axis) blocking operation, so the whole app froze until it finished —
whereas hitting "Move to Safe Z" first (a fire-and-forget move_z_absolute) made
the subsequent click's retract a fast no-op, so no freeze. The fix runs the
blocking travel on a daemon thread via :class:`SafeTravelWorker`, with a
busy-guard so re-clicks are ignored, not queued.

Serial hardening: ``ZPStage._read_until_ok`` now has an absolute wall-clock cap
(``READ_OK_HARD_CAP_S``) so a never-completing move (whose Marlin 'busy'
keep-alives used to reset the wait window forever) fails after a ceiling instead
of hanging the caller — and, under the shared serial lock, every other thread —
indefinitely.

No hardware needed: a fake controller / fake serial + an offscreen QApplication.
"""

import os
import sys
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from gui.widgets.safe_travel_worker import SafeTravelWorker  # noqa: E402
from SupportClasses.ZPStage import ZPStageManager  # noqa: E402

_app = None


def setUpModule():
    global _app
    _app = QApplication.instance() or QApplication(sys.argv)


def _pump_until(predicate, timeout_s=3.0):
    """Spin the Qt event loop (delivering queued cross-thread signals) until
    ``predicate()`` is true or the timeout elapses."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    QApplication.processEvents()
    return predicate()


class _FakeController:
    """Records safe_travel_to calls; optionally gates on an Event and/or raises."""

    def __init__(self, *, result=True, gate=None, raises=False):
        self.calls = []
        self._result = result
        self._gate = gate
        self._raises = raises

    def safe_travel_to(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self._gate is not None:
            self._gate.wait(5.0)
        if self._raises:
            raise RuntimeError("boom")
        return self._result


# ════════════════════════════════════════════════════════════════════
#  SafeTravelWorker
# ════════════════════════════════════════════════════════════════════

class TestSafeTravelWorker(unittest.TestCase):
    def test_runs_off_thread_and_emits_result(self):
        worker = SafeTravelWorker()
        results = []
        worker.finished.connect(results.append)
        ctrl = _FakeController(result=True)

        started = worker.start(ctrl, 1000.0, 2000.0, safe_z_mm=30.0,
                               target_z_mm=None)
        self.assertTrue(started)
        # The blocking call ran on a worker thread, not the calling (GUI) thread.
        self.assertNotEqual(worker._thread.ident,
                            threading.current_thread().ident)

        self.assertTrue(_pump_until(lambda: results == [True]))
        self.assertFalse(worker.busy)
        # Args forwarded verbatim to safe_travel_to.
        self.assertEqual(ctrl.calls[0][0], (1000.0, 2000.0))
        self.assertEqual(ctrl.calls[0][1],
                         {"safe_z_mm": 30.0, "target_z_mm": None})

    def test_busy_guard_ignores_reentrant_start(self):
        gate = threading.Event()
        worker = SafeTravelWorker()
        ctrl = _FakeController(result=True, gate=gate)

        self.assertTrue(worker.start(ctrl, 0.0, 0.0, safe_z_mm=1.0))
        self.assertTrue(worker.busy)
        # A second click while the first travel is in flight is IGNORED (not
        # queued) — this is what stops N serialized travels stacking up.
        self.assertFalse(worker.start(ctrl, 9.0, 9.0, safe_z_mm=1.0))

        gate.set()  # let the first travel finish
        self.assertTrue(_pump_until(lambda: not worker.busy))
        self.assertEqual(len(ctrl.calls), 1)  # the 2nd start never called through

        # Once idle, a fresh travel is accepted again.
        self.assertTrue(worker.start(ctrl, 5.0, 5.0, safe_z_mm=1.0))
        self.assertTrue(_pump_until(lambda: not worker.busy))
        self.assertEqual(len(ctrl.calls), 2)

    def test_exception_reports_false_and_clears_busy(self):
        worker = SafeTravelWorker()
        results = []
        worker.finished.connect(results.append)
        ctrl = _FakeController(raises=True)

        self.assertTrue(worker.start(ctrl, 0.0, 0.0, safe_z_mm=1.0))
        self.assertTrue(_pump_until(lambda: results == [False]))
        self.assertFalse(worker.busy)

    def test_timeout_result_reported(self):
        worker = SafeTravelWorker()
        results = []
        worker.finished.connect(results.append)
        ctrl = _FakeController(result=False)  # safe_travel_to timed out

        self.assertTrue(worker.start(ctrl, 0.0, 0.0, safe_z_mm=1.0))
        self.assertTrue(_pump_until(lambda: results == [False]))
        self.assertFalse(worker.busy)

    def test_missing_controller_refused(self):
        worker = SafeTravelWorker()
        self.assertFalse(worker.start(None, 0.0, 0.0))
        self.assertFalse(worker.start(object(), 0.0, 0.0))  # no safe_travel_to
        self.assertFalse(worker.busy)


# ════════════════════════════════════════════════════════════════════
#  ZPStage._read_until_ok wall-clock cap
# ════════════════════════════════════════════════════════════════════

class _FakeSerial:
    """readline() returns a fixed line each call (with a tiny sleep so a
    busy-only stream doesn't hot-spin the CPU during the test)."""

    def __init__(self, line: bytes, sleep_s: float = 0.005):
        self._line = line
        self._sleep = sleep_s

    def readline(self):
        time.sleep(self._sleep)
        return self._line

    def stop(self):  # for the simulate-path stop() at GC (keeps teardown quiet)
        pass


def _bare_zp(serial_line: bytes):
    """A ZPStageManager without opening a serial port (bypasses __init__).

    ``simulate=True`` + a fake serial with ``stop()`` keeps the __del__→stop()
    teardown quiet; _read_until_ok itself ignores ``simulate``.
    """
    zp = ZPStageManager.__new__(ZPStageManager)
    zp.serial = _FakeSerial(serial_line)
    zp.simulate = True
    return zp


class TestReadUntilOkHardCap(unittest.TestCase):
    def test_busy_forever_hits_hard_cap(self):
        zp = _bare_zp(b"busy: processing\n")
        zp.READ_OK_HARD_CAP_S = 0.3  # shrink the ceiling for the test

        t0 = time.monotonic()
        ok, _text, stats = zp._read_until_ok(ok_timeout=0.1, collect=False)
        elapsed = time.monotonic() - t0

        # WITHOUT the cap this loops forever (every 'busy' resets the window);
        # WITH it, it fails at the ceiling.
        self.assertFalse(ok)
        self.assertEqual(stats["outcome"], "hard_timeout")
        self.assertGreater(stats["busy"], 0)
        self.assertLess(elapsed, 2.0)          # bounded, not infinite
        self.assertGreaterEqual(elapsed, 0.25)  # ~ the 0.3 s cap

    def test_ok_returns_immediately(self):
        zp = _bare_zp(b"ok\n")
        zp.READ_OK_HARD_CAP_S = 0.3
        ok, _text, stats = zp._read_until_ok(ok_timeout=1.0, collect=False)
        self.assertTrue(ok)
        self.assertEqual(stats["outcome"], "ok")

    def test_silence_uses_normal_timeout_not_hard_cap(self):
        # A silent board (readline → b"") still times out at ok_timeout via the
        # normal deadline — the hard cap only bounds the busy-reset path.
        zp = _bare_zp(b"")
        zp.READ_OK_HARD_CAP_S = 5.0
        t0 = time.monotonic()
        ok, _text, stats = zp._read_until_ok(ok_timeout=0.2, collect=False)
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertEqual(stats["outcome"], "timeout")
        self.assertLess(elapsed, 1.0)  # bounded by ok_timeout, well under the cap


if __name__ == "__main__":
    unittest.main()
