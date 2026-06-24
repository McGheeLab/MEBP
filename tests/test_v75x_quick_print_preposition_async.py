"""test_v75x_quick_print_preposition_async.py — Quick Print pre-position runs
off the GUI thread + ``wait_for_z_arrival`` fast-fails on a dead ZP board.

Two v7.5.x fixes motivated by a bench report ("live needle view wasn't live
until halfway through the print" + "ZP disconnected at the end and didn't move
up"):

1) The Quick Print pre-position (``safe_travel_to``) used to run on the GUI
   thread, freezing the live microscope feed for the whole move — the entire
   15 s Z timeout when the board misbehaved. It now runs on a worker thread and
   resumes in ``_on_prepositioned`` via a queued Qt signal, so the feed keeps
   painting. The continuation also re-checks ZP connectivity before printing
   (a board drop during positioning now surfaces in ~1 s, see #2).

2) ``StageController.wait_for_z_arrival`` read ``get_current_position()``, which
   returns the last *stale* floats when the board stops answering M114 — so a
   genuine ZP drop made it wait out the full timeout comparing a frozen position
   to the target. It now watches ``_last_position_read_ok`` and bails after a
   short debounced streak of read failures.
"""

import sys
import time
import unittest
from unittest.mock import MagicMock, patch

from PySide6.QtWidgets import QApplication, QMessageBox

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PrintManager import CommandType


def _types(job):
    return [c.type for c in job.commands]


class _FakePM:
    instances: list = []

    def __init__(self, controller):
        self.controller = controller
        self.job = None
        self.started = False
        self.state = None
        self.exec_logger = None
        self.on_progress = None
        self.on_state_changed = None
        _FakePM.instances.append(self)

    def load_job(self, job):
        self.job = job

    def start(self):
        self.started = True


# ── Part 1: pre-position runs on a worker thread ─────────────────────

class TestPrepositionAsync(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _ready_controller(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 1000.0, "y": 2000.0, "Z": 0.0}
        ctrl.print_floor_violation.return_value = False
        ctrl.print_height_to_zref.return_value = -16.0
        ctrl.print_z_dir.return_value = -1.0
        ctrl.safe_travel_to.return_value = True
        return ctrl

    def _ready_page(self, ctrl):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        page = QuickPrintWorkflowPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(96)
        page._well_positions = None
        page._safe_z = 5.0
        # These tests exercise the plain async-preposition path (no hardware /
        # reagents). v7.5.x defaults prep + clean-after ON; turn both OFF here so
        # there is no preamble/cleanup to gate on — isolating the preposition.
        page._prep_check.setChecked(False)
        page._postclean_check.setChecked(False)
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._size_spin.setValue(1.0)
        page._selected_well = "A1"
        return page

    def _drain(self, page):
        t = page._preposition_thread
        if t is not None:
            t.join(timeout=5.0)
        self._app.processEvents()

    def test_safe_travel_runs_off_gui_thread(self):
        """The blocking safe_travel_to runs on a worker thread, NOT the GUI
        thread, so the event loop (and the live camera feed) stays responsive."""
        ctrl = self._ready_controller()
        gui_thread_id = None

        import threading
        main_id = threading.get_ident()
        seen = {}

        def _slow_travel(*a, **kw):
            seen["thread"] = threading.get_ident()
            time.sleep(0.05)
            return True

        ctrl.safe_travel_to.side_effect = _slow_travel
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.No):
            page._on_print()
            # _on_print must return immediately (move is off-thread).
            self.assertIsNotNone(page._preposition_thread)
            self.assertTrue(page._preposition_thread.is_alive()
                            or seen.get("thread"))
            self._drain(page)
        self.assertIn("thread", seen)
        self.assertNotEqual(seen["thread"], main_id,
                            "safe_travel_to must not run on the GUI thread")

    def test_print_button_disabled_while_positioning(self):
        """The Print button stays disabled during positioning so the periodic
        status tick can't re-enable it and launch a second move."""
        ctrl = self._ready_controller()

        import threading
        gate = threading.Event()

        def _blocked_travel(*a, **kw):
            gate.wait(timeout=2.0)
            return True

        ctrl.safe_travel_to.side_effect = _blocked_travel
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.No):
            page._on_print()
            # Mid-positioning: a status tick must NOT re-enable Print.
            page._update_button_state()
            self.assertFalse(page._print_btn.isEnabled())
            gate.set()
            self._drain(page)

    def test_aborts_print_if_zp_dropped_during_positioning(self):
        """If the board disconnects during positioning, the continuation must
        NOT start a print (it would dry-run with the needle parked down)."""
        ctrl = self._ready_controller()

        def _travel_then_drop(*a, **kw):
            ctrl.is_zp_connected = False  # board dies mid-move
            return False

        ctrl.safe_travel_to.side_effect = _travel_then_drop
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakePM.instances), 0)
        self.assertIn("disconnect", page._status.text().lower())

    def test_confirm_yes_runs_when_connected(self):
        """Happy path through the async flow still builds + starts the job."""
        _FakePM.instances.clear()
        ctrl = self._ready_controller()
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakePM.instances), 1)
        self.assertTrue(_FakePM.instances[0].started)
        self.assertNotIn(CommandType.HOME_XY, _types(_FakePM.instances[0].job))


# ── Part 2: wait_for_z_arrival fast-fails on a dead board ────────────

class _FakeZP:
    """Minimal ZP stand-in: answers M114 with a fixed position, then 'dies'
    (read_ok=False, stale position) after ``die_after`` reads."""

    def __init__(self, z_raw=68.23, die_after=None):
        self._z = z_raw
        self._reads = 0
        self._die_after = die_after
        self._last_position_read_ok = True
        # axis_map: logical Z → physical index 2 (X, Y, Z, E)
        self.axis_map = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}

    def get_current_position(self):
        self._reads += 1
        if self._die_after is not None and self._reads > self._die_after:
            self._last_position_read_ok = False  # stale floats, no answer
        else:
            self._last_position_read_ok = True
        return (1.0, 0.32, self._z, 0.0)


class TestWaitForZArrivalFastFail(unittest.TestCase):
    def _controller(self, zp):
        """Build a StageController without running __init__, wired just enough
        for wait_for_z_arrival."""
        from SupportClasses.StageController import StageController
        ctrl = StageController.__new__(StageController)
        ctrl.zp_stage = zp
        ctrl._zp_stage = zp
        ctrl.zero_position = {"Z": 0.0}
        # is_zp_connected is a property reading _zp_stage's serial; force True.
        ctrl._pos_poller = MagicMock()
        return ctrl

    def test_fast_fails_when_board_stops_answering(self):
        from SupportClasses.StageController import StageController
        zp = _FakeZP(z_raw=68.23, die_after=0)  # dead from the first read
        ctrl = self._controller(zp)
        with patch.object(type(ctrl), "is_zp_connected",
                          property(lambda s: True)):
            t0 = time.monotonic()
            ok = StageController.wait_for_z_arrival(
                ctrl, target_z_mm=39.8, tolerance_mm=0.1, timeout_s=15.0)
            elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        # Must bail on the read-fail streak, NOT wait out the 15 s timeout.
        self.assertLess(elapsed, 5.0)

    def test_confirms_arrival_when_board_alive(self):
        from SupportClasses.StageController import StageController
        zp = _FakeZP(z_raw=39.8)  # already at target, answering
        ctrl = self._controller(zp)
        with patch.object(type(ctrl), "is_zp_connected",
                          property(lambda s: True)):
            ok = StageController.wait_for_z_arrival(
                ctrl, target_z_mm=39.8, tolerance_mm=0.1, timeout_s=5.0)
        self.assertTrue(ok)

    def test_transient_single_miss_does_not_fast_fail(self):
        """A single read failure (busy board) is ridden out, not treated as a
        disconnect — the board recovers and arrival is confirmed."""
        from SupportClasses.StageController import StageController

        class _Flaky(_FakeZP):
            def get_current_position(self):
                self._reads += 1
                # miss once on read #2, otherwise answer at target
                self._last_position_read_ok = self._reads != 2
                return (1.0, 0.32, 39.8, 0.0)

        zp = _Flaky(z_raw=39.8)
        ctrl = self._controller(zp)
        with patch.object(type(ctrl), "is_zp_connected",
                          property(lambda s: True)):
            ok = StageController.wait_for_z_arrival(
                ctrl, target_z_mm=39.8, tolerance_mm=0.1, timeout_s=5.0)
        self.assertTrue(ok)


if __name__ == "__main__":
    unittest.main()
