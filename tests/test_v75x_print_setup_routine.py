"""test_v75x_print_setup_routine.py — Quick Print "print setup routine".

After the operator confirms the needle position, a print runs a defined setup
routine before extruding along the path:
  1) move Z to safe Z          (plan TRAVEL_UP)
  2) move XY to print start     (plan MOVE_XY)
  3) move Z to print Z          (plan MOVE_Z) — and CONFIRM the needle arrived
  4) start pump flow, wait 0.25 s  (plan EXTRUDE prime at the flow rate)
  5) begin the print trajectory (plan PRINT_PATH)

Covered here:
- Step 3: the discrete MOVE_Z handler confirms Z arrival (flush_moves +
  wait_for_z_arrival, poller suspended) before continuing; falls back to a
  fixed settle when the controller can't confirm.
- Step 4: Quick Print sets a pre-flow prime = flow × 0.25 s at the flow rate.
- Ordering: the built plan emits TRAVEL_UP → MOVE_XY → MOVE_Z → EXTRUDE(prime)
  → PRINT_PATH (steps 1→5).
"""

import sys
import unittest
from unittest.mock import MagicMock, patch

from PySide6.QtWidgets import QApplication

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PrintManager import (
    PrintManager, PrintCommand, CommandType, PrintSettings,
    build_well_plate_job,
)


# ── Step 3: MOVE_Z confirms the needle reached print Z ───────────────

class TestMoveZConfirmsArrival(unittest.TestCase):
    def _pm(self, connected=True):
        ctrl = MagicMock()
        ctrl.is_zp_connected = connected
        ctrl.wait_for_z_arrival.return_value = True
        pm = PrintManager(ctrl)
        pm.job = MagicMock()
        return pm, ctrl

    def test_move_z_waits_for_arrival_with_poller_suspended(self):
        pm, ctrl = self._pm(connected=True)
        pm._execute_command(PrintCommand(type=CommandType.MOVE_Z,
                                         params={"z": -16.17}))
        ctrl.move_z_absolute.assert_called_once()
        # Confirmed arrival (step 3) with the poller suspended for the wait.
        ctrl.suspend_position_poller.assert_called_once()
        ctrl.zp_stage.flush_moves.assert_called_once()
        ctrl.wait_for_z_arrival.assert_called_once()
        self.assertAlmostEqual(
            ctrl.wait_for_z_arrival.call_args.args[0], -16.17)
        ctrl.resume_position_poller.assert_called_once()

    def test_move_z_resumes_poller_even_if_wait_raises(self):
        pm, ctrl = self._pm(connected=True)
        ctrl.wait_for_z_arrival.side_effect = RuntimeError("z fail")
        with self.assertRaises(RuntimeError):
            pm._execute_command(PrintCommand(type=CommandType.MOVE_Z,
                                             params={"z": -16.0}))
        ctrl.resume_position_poller.assert_called_once()

    def test_move_z_falls_back_when_zp_disconnected(self):
        pm, ctrl = self._pm(connected=False)
        with patch("SupportClasses.PrintManager.time.sleep") as _sleep:
            pm._execute_command(PrintCommand(type=CommandType.MOVE_Z,
                                             params={"z": -16.0}))
        ctrl.move_z_absolute.assert_called_once()
        ctrl.wait_for_z_arrival.assert_not_called()
        ctrl.suspend_position_poller.assert_not_called()
        _sleep.assert_called()  # fixed settle fallback


# ── Step 4 + ordering: Quick Print pre-flow prime in the plan ────────

class TestRoutineOrderingAndPreflow(unittest.TestCase):
    def test_plan_order_is_travelup_movexy_movez_prime_printpath(self):
        s = PrintSettings(num_layers=1)
        s.prime_amounts_uL["P1"] = 0.1   # pre-flow prime (step 4)
        s.pump_rates_uL_s["P1"] = 0.4
        job = build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)],
            path_points=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            settings=s, pump="P1", return_home=False,
        )
        types = [c.type for c in job.commands]
        pp = types.index(CommandType.PRINT_PATH)
        # Step 4 (prime) immediately precedes the path; step 3 (descent) before that.
        self.assertEqual(types[pp - 1], CommandType.DISPENSE)
        self.assertEqual(types[pp - 2], CommandType.MOVE_Z)
        # Steps 1 + 2 appear earlier in the approach.
        self.assertIn(CommandType.MOVE_XY, types[:pp - 2])
        self.assertIn(CommandType.TRAVEL_UP, types[:pp - 2])
        # The prime runs the pump at the flow rate (≈0.25 s of flow).
        prime = job.commands[pp - 1]
        self.assertEqual(prime.params["pump"], "P1")
        self.assertAlmostEqual(prime.params["amount_uL"], 0.1)
        self.assertAlmostEqual(prime.params["rate_uL_s"], 0.4)


class TestQuickPrintPreflowSetting(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        page = QuickPrintWorkflowPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(96)
        page._safe_z = 5.0
        return page

    def test_build_settings_sets_preflow_prime_at_flow_rate(self):
        page = self._page()
        page._flow_spin.setValue(0.4)
        page._speed_pct_spin.setValue(100)  # Flow@100% = the resolved flow
        s = page._build_settings()
        pump = page._pump()
        # flow × 0.25 s of pre-flow, at the flow rate.
        self.assertAlmostEqual(s.prime_amounts_uL[pump], 0.4 * page._PREFLOW_S)
        self.assertAlmostEqual(s.pump_rates_uL_s[pump], 0.4)

    def test_preflow_scales_with_flow(self):
        page = self._page()
        page._flow_spin.setValue(2.0)
        page._speed_pct_spin.setValue(100)
        s = page._build_settings()
        pump = page._pump()
        self.assertAlmostEqual(s.prime_amounts_uL[pump], 2.0 * 0.25)


if __name__ == "__main__":
    unittest.main()
