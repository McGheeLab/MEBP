"""test_v75x_quick_print_position_confirm.py — Quick Print pre-print
positioning (hands-free) + no return-to-origin.

Covers two v7.5.x Quick Print requirements:

1) Before the print starts the needle/XY is moved (retracted) to the print-start
   point. ``QuickPrintWorkflowPage._preposition_for_print`` delegates to
   ``StageController.safe_travel_to`` with ``target_z_mm=None`` (raise→XY, never
   lowers). v7.5.x: the run is HANDS-FREE after the up-front "confirm setup"
   gate — there is NO per-print position confirmation; once positioned, the
   print starts directly (subject to a ZP-still-connected + arrival-confirmed
   safety check).

2) CRITICAL: the job must NOT drive XY back to 0,0 at the end — it only retracts
   the needle out of the well to the travel Z. ``build_well_plate_job`` gains a
   ``return_home`` flag; Quick Print passes ``return_home=False`` so the trailing
   ``HOME_XY`` (return to origin) is omitted while the final ``TRAVEL_UP`` stays.
"""

import sys
import unittest
from unittest.mock import MagicMock, patch

from PySide6.QtWidgets import QApplication, QMessageBox

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PrintManager import (
    build_well_plate_job, PrintSettings, CommandType,
)


def _types(job):
    return [c.type for c in job.commands]


class _FakePM:
    """Records load_job/start without spawning the executor thread."""

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


# ── Part 2: build_well_plate_job return_home flag ────────────────────

class TestReturnHomeFlag(unittest.TestCase):
    def _job(self, **kw):
        return build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            settings=PrintSettings(num_layers=1),
            **kw,
        )

    def test_default_returns_home(self):
        """Legacy/default behavior: ends with TRAVEL_UP then HOME_XY."""
        types = _types(self._job())
        self.assertIn(CommandType.HOME_XY, types)
        self.assertEqual(types[-1], CommandType.HOME_XY)
        self.assertEqual(types[-2], CommandType.TRAVEL_UP)

    def test_return_home_false_omits_home_xy(self):
        """return_home=False: no HOME_XY anywhere; ends on the final TRAVEL_UP."""
        types = _types(self._job(return_home=False))
        self.assertNotIn(CommandType.HOME_XY, types)
        self.assertEqual(types[-1], CommandType.TRAVEL_UP)

    def test_return_home_false_only_drops_trailing_home(self):
        """Dropping HOME_XY removes exactly one command vs the default job and
        keeps every other command identical (the final TRAVEL_UP included)."""
        home = _types(self._job(return_home=True))
        no_home = _types(self._job(return_home=False))
        self.assertEqual(home, no_home + [CommandType.HOME_XY])


# ── Part 1: pre-position + confirm in the Quick Print page ───────────

class _PageBase(unittest.TestCase):
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
        page._well_positions = None  # geometric center
        page._safe_z = 5.0
        # v7.5.x defaults prep + clean-after ON; these tests exercise the plain
        # preposition path (no reagents), so turn both OFF to isolate it.
        page._prep_check.setChecked(False)
        page._postclean_check.setChecked(False)
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._size_spin.setValue(1.0)
        page._selected_well = "A1"
        return page


class TestPreposition(_PageBase):
    def test_preposition_calls_safe_travel_never_lowers(self):
        ctrl = self._ready_controller()
        page = self._ready_page(ctrl)
        ok = page._preposition_for_print((4.0, 5.0), 7.5)
        self.assertTrue(ok)
        ctrl.safe_travel_to.assert_called_once()
        args, kwargs = ctrl.safe_travel_to.call_args
        # zero-ref mm → absolute µm: x=4.0*1000+1000=5000, y=5.0*1000+2000=7000
        self.assertAlmostEqual(args[0], 5000.0)
        self.assertAlmostEqual(args[1], 7000.0)
        self.assertAlmostEqual(kwargs["safe_z_mm"], 7.5)
        # target_z_mm=None → safe_travel_to raises/travels but NEVER lowers.
        self.assertIsNone(kwargs["target_z_mm"])

    def test_preposition_returns_false_on_timeout(self):
        ctrl = self._ready_controller()
        ctrl.safe_travel_to.return_value = False
        page = self._ready_page(ctrl)
        self.assertFalse(page._preposition_for_print((0.0, 0.0), 5.0))

    def test_preposition_swallows_exception(self):
        ctrl = self._ready_controller()
        ctrl.safe_travel_to.side_effect = RuntimeError("boom")
        page = self._ready_page(ctrl)
        self.assertFalse(page._preposition_for_print((0.0, 0.0), 5.0))


class TestOnPrintFlow(_PageBase):
    def setUp(self):
        _FakePM.instances.clear()

    def _run_on_print(self, page):
        """Drive the v7.5.x async pre-position flow to completion.

        ``_on_print`` now launches ``safe_travel_to`` on a worker thread (so the
        live microscope feed keeps updating) and resumes in ``_on_prepositioned``
        via a queued Qt signal. Join the worker, then pump the event loop so the
        continuation (confirm dialog + job launch) runs on this thread.
        """
        page._on_print()
        t = page._preposition_thread
        if t is not None:
            t.join(timeout=5.0)
        self._app.processEvents()

    def test_positions_then_runs_without_home(self):
        ctrl = self._ready_controller()
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM):
            self._run_on_print(page)
        # Positioned (retract→travel, never lower) BEFORE running.
        ctrl.safe_travel_to.assert_called_once()
        self.assertIsNone(ctrl.safe_travel_to.call_args.kwargs["target_z_mm"])
        # A job was loaded + started.
        self.assertEqual(len(_FakePM.instances), 1)
        pm = _FakePM.instances[0]
        self.assertTrue(pm.started)
        self.assertIsNotNone(pm.job)
        # CRITICAL: no return-to-origin; ends on the final retract.
        types = _types(pm.job)
        self.assertNotIn(CommandType.HOME_XY, types)
        self.assertEqual(types[-1], CommandType.TRAVEL_UP)

    def test_plain_print_runs_hands_free(self):
        """No prep + no ink → no dialog at all: the print runs hands-free once
        positioned (the per-print position confirmation was removed in v7.5.x)."""
        ctrl = self._ready_controller()
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM), \
                patch.object(QMessageBox, "question") as q:
            self._run_on_print(page)
        # No confirmation dialog was shown anywhere in the plain path.
        q.assert_not_called()
        ctrl.safe_travel_to.assert_called_once()
        self.assertEqual(len(_FakePM.instances), 1)
        self.assertTrue(_FakePM.instances[0].started)

    def test_unconfirmed_positioning_aborts(self):
        """Hands-free safety: if positioning does not confirm arrival, the print
        must NOT start (no operator to verify)."""
        ctrl = self._ready_controller()
        ctrl.safe_travel_to.return_value = False  # timeout / unsettled
        page = self._ready_page(ctrl)
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM):
            self._run_on_print(page)
        ctrl.safe_travel_to.assert_called_once()
        self.assertEqual(len(_FakePM.instances), 0)

    def test_no_motion_when_geometry_empty(self):
        """A guard failure (no printable path) must not move the stage."""
        ctrl = self._ready_controller()
        page = self._ready_page(ctrl)
        with patch.object(page, "_path_segments_for_selection",
                          return_value=[]), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            self._run_on_print(page)
        ctrl.safe_travel_to.assert_not_called()


if __name__ == "__main__":
    unittest.main()
