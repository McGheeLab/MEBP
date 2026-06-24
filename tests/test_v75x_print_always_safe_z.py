"""test_v75x_print_always_safe_z.py — every print always ends at a safe Z.

v7.5.x CRITICAL SAFETY: `PrintManager._retract_to_safe_z` is the single
guarantee that the needle is retracted to the travel / safe Z at the END of a
print — on completion, error, OR abort — regardless of execution mode
(discrete / hybrid / trajectory) and regardless of whether the plan happened to
end with a retract step. It is polarity-safe and RAISE-ONLY (delegates to
`StageController.ensure_retracted_to`), so it is a confirmed no-op when already
retracted and never lowers the needle.

Covered here:
- `_retract_to_safe_z` calls `ensure_retracted_to(travel_z)` (explicit override
  and job-settings fallback), no-ops when ZP is disconnected, falls back to
  `move_z_absolute` on an older controller, and never raises.
- The discrete `_execute_loop` `finally` retracts on completion, on error, and
  on the abort early-return.
"""

import unittest
from unittest.mock import MagicMock, patch

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, build_well_plate_job,
)


def _zp_controller(with_ensure: bool = True):
    """A mock controller that looks ZP-connected."""
    ctrl = MagicMock()
    ctrl.is_zp_connected = True
    ctrl.is_xy_connected = True
    ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    ctrl.position_logger = None
    ctrl.get_xy_position.return_value = (1.0, 2.0)
    ctrl.get_zp_position_logical_tuple.return_value = (0.0, 0.0, 0.0, 0.0)
    ctrl.ensure_retracted_to.return_value = True
    if not with_ensure:
        # Simulate an older controller without ensure_retracted_to.
        del ctrl.ensure_retracted_to
    return ctrl


class TestRetractToSafeZ(unittest.TestCase):
    def _pm(self, ctrl, travel_z=7.0):
        pm = PrintManager(ctrl)
        pm.job = build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0)],
            settings=PrintSettings(num_layers=1, travel_z_height=travel_z),
        )
        return pm

    def test_uses_explicit_travel_z(self):
        ctrl = _zp_controller()
        pm = self._pm(ctrl, travel_z=7.0)
        pm._retract_to_safe_z("t", travel_z=12.5)
        ctrl.ensure_retracted_to.assert_called_once_with(12.5)

    def test_falls_back_to_job_travel_z(self):
        ctrl = _zp_controller()
        pm = self._pm(ctrl, travel_z=9.0)
        pm._retract_to_safe_z("t")
        ctrl.ensure_retracted_to.assert_called_once_with(9.0)

    def test_noop_when_zp_disconnected(self):
        ctrl = _zp_controller()
        ctrl.is_zp_connected = False
        pm = self._pm(ctrl)
        pm._retract_to_safe_z("t", travel_z=5.0)
        ctrl.ensure_retracted_to.assert_not_called()

    def test_noop_when_no_travel_z_available(self):
        ctrl = _zp_controller()
        pm = PrintManager(ctrl)
        pm.job = None  # no job → no settings → nothing to retract to
        pm._retract_to_safe_z("t")
        ctrl.ensure_retracted_to.assert_not_called()

    def test_fallback_move_z_on_older_controller(self):
        ctrl = _zp_controller(with_ensure=False)
        pm = self._pm(ctrl, travel_z=6.0)
        pm._retract_to_safe_z("t")
        ctrl.move_z_absolute.assert_called_once_with(6.0, from_zero_ref=True)

    def test_never_raises(self):
        ctrl = _zp_controller()
        ctrl.ensure_retracted_to.side_effect = RuntimeError("boom")
        pm = self._pm(ctrl, travel_z=5.0)
        # Must not propagate — the print is already ending.
        pm._retract_to_safe_z("t")
        ctrl.ensure_retracted_to.assert_called_once()


class TestExecuteLoopAlwaysRetracts(unittest.TestCase):
    """The discrete `_execute_loop` finally retracts in every outcome."""

    def _pm(self, ctrl, travel_z=7.0):
        pm = PrintManager(ctrl)
        pm.load_job(build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0)],
            settings=PrintSettings(num_layers=1, travel_z_height=travel_z),
            return_home=False,
        ))
        # Isolate from recorder / history / resume-file IO.
        pm._stop_recorder = MagicMock()
        pm._record_history = MagicMock()
        return pm

    def _run_loop(self, pm):
        with patch("SupportClasses.PrintManager.save_print_progress"), \
                patch("SupportClasses.PrintManager.clear_print_progress"):
            pm._execute_loop()

    def test_retracts_on_completion(self):
        ctrl = _zp_controller()
        pm = self._pm(ctrl, travel_z=7.0)
        pm._execute_command = MagicMock()  # every command succeeds (no-op)
        self._run_loop(pm)
        self.assertEqual(pm.state, PrintState.COMPLETED)
        ctrl.ensure_retracted_to.assert_called_with(7.0)

    def test_retracts_on_error(self):
        ctrl = _zp_controller()
        pm = self._pm(ctrl, travel_z=8.0)
        pm._execute_command = MagicMock(side_effect=RuntimeError("mid-print"))
        self._run_loop(pm)
        self.assertEqual(pm.state, PrintState.ERROR)
        # Needle lifted out of the well even though the print blew up.
        ctrl.ensure_retracted_to.assert_called_with(8.0)

    def test_retracts_on_abort_early_return(self):
        ctrl = _zp_controller()
        pm = self._pm(ctrl, travel_z=6.5)
        pm._execute_command = MagicMock()
        pm._set_state(PrintState.RUNNING)
        pm._abort_flag.set()  # loop returns immediately at the abort check
        self._run_loop(pm)
        # The command never ran (aborted before the first step)...
        pm._execute_command.assert_not_called()
        # ...but the finally still retracted to safe Z.
        ctrl.ensure_retracted_to.assert_called_with(6.5)


if __name__ == "__main__":
    unittest.main()
