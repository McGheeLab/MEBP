"""test_v75x_simple_print_manager.py — the minimal, fully-confirmed debug
print executor (SimplePrintManager).

Verifies the SIMPLEST baseline keeps every safety guarantee while consuming the
same build_well_plate_job command plan as the full PrintManager:
  * retract (confirmed) BEFORE any cross-position XY move
  * descend with an EXPLICIT feedrate and CONFIRM arrival (M400 + M114)
  * abort the print if a Z descent can't be confirmed
  * abort if the ZP board drops mid-print
  * ALWAYS retract to safe Z at the end (completion / error)
  * per-segment confirmed PRINT_PATH (pump dispense + XY, then M400 + XY settle)
"""

import unittest
from types import SimpleNamespace

from SupportClasses.PrintManager import (
    PrintState, PrintSettings, build_well_plate_job,
)
from SupportClasses.SimplePrintManager import SimplePrintManager


class FakeCtrl:
    """Records the ordered sequence of motion calls SimplePrintManager makes."""

    def __init__(self, *, xy=True, zp=True, arrival=True, m400=True,
                 disconnect_after_descent=False):
        self._zp = zp
        self.is_xy_connected = xy
        self._arrival = arrival
        self._m400 = m400
        self._disconnect_after_descent = disconnect_after_descent
        self.calls: list[tuple] = []
        self._zp_insert_feedrate = 300.0
        self._zp_retract_feedrate = 500.0
        self.xy_stage = SimpleNamespace(
            set_speed_mm_s=lambda s: self.calls.append(("set_speed", s)))
        self.zp_stage = SimpleNamespace(flush_moves=self._flush)

    @property
    def is_zp_connected(self):
        return self._zp

    def ensure_retracted_to(self, z, tol_mm=0.1, timeout_s=15.0):
        self.calls.append(("retract", round(float(z), 3)))
        return True

    def move_z_absolute(self, z, from_zero_ref=True, feedrate_mm_min=None):
        self.calls.append(("move_z", round(float(z), 3), feedrate_mm_min))
        if self._disconnect_after_descent:
            self._zp = False

    def _flush(self, timeout_s=10.0):
        self.calls.append(("flush",))
        return self._m400

    def wait_for_z_arrival(self, z, tolerance_mm=0.05, timeout_s=10.0):
        self.calls.append(("wait_z", round(float(z), 3)))
        return self._arrival

    def move_xy_absolute(self, x, y, from_zero_ref=True):
        self.calls.append(("move_xy", round(float(x), 3), round(float(y), 3)))

    def wait_for_xy_arrival(self, x, y, tolerance_mm=0.5, timeout_s=10.0):
        self.calls.append(("wait_xy", round(float(x), 3), round(float(y), 3)))
        return True

    def move_pump_uL(self, pump, vol, rate):
        self.calls.append(("pump", pump, round(float(vol), 6), rate))

    def set_print_floor_active(self, active):
        self.calls.append(("floor", active))

    # convenience
    def names(self):
        return [c[0] for c in self.calls]


def _job(flow_rate=0.25, n=6):
    """Single-well square path job (return_home=False)."""
    import math
    path = [(math.cos(2 * math.pi * k / n),
             math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
    settings = PrintSettings(
        num_layers=1, travel_z_height=10.0, print_z_height=2.0,
        print_speed_mm_s=5.0, travel_speed_mm_s=10.0,
        pump_rate_uL_s=flow_rate,
        pump_rates_uL_s={"P1": flow_rate, "P2": flow_rate, "P3": flow_rate})
    return build_well_plate_job(
        well_positions=[("w", 0.0, 0.0)], path_points=path, settings=settings,
        pump="P1", flow_rate=flow_rate, job_name="simple test",
        return_home=False)


def _run(ctrl, job):
    pm = SimplePrintManager(ctrl)
    pm.load_job(job)
    pm.start()
    pm._thread.join(timeout=15)
    return pm


class TestSimplePrintManager(unittest.TestCase):

    def test_completes_and_keeps_safety(self):
        ctrl = FakeCtrl()
        pm = _run(ctrl, _job())
        self.assertEqual(pm.state, PrintState.COMPLETED)
        names = ctrl.names()
        self.assertIn("retract", names)         # retracted before travel
        self.assertIn("move_xy", names)         # moved XY
        self.assertIn("move_z", names)          # descended
        self.assertIn("flush", names)           # confirmed (M400)
        self.assertIn("pump", names)            # extruded (wet)
        # floor armed then disarmed
        floor_states = [c[1] for c in ctrl.calls if c[0] == "floor"]
        self.assertIn(True, floor_states)
        self.assertEqual(floor_states[-1], False)

    def test_every_z_move_has_explicit_feedrate(self):
        ctrl = FakeCtrl()
        _run(ctrl, _job())
        z_moves = [c for c in ctrl.calls if c[0] == "move_z"]
        self.assertTrue(z_moves)
        for c in z_moves:
            self.assertIsNotNone(c[2], f"bare Z move (no feedrate): {c}")
        # the descent specifically uses the controlled INSERT feedrate
        self.assertEqual(z_moves[0][2], 300.0)

    def test_retract_precedes_first_xy_travel(self):
        ctrl = FakeCtrl()
        _run(ctrl, _job())
        names = ctrl.names()
        self.assertIn("retract", names)
        self.assertIn("move_xy", names)
        self.assertLess(names.index("retract"), names.index("move_xy"),
                        "needle must retract before the first XY travel")

    def test_descent_is_confirmed_before_extrusion(self):
        ctrl = FakeCtrl()
        _run(ctrl, _job())
        names = ctrl.names()
        # the descent (move_z) + its confirm (flush/wait_z) happen before the
        # first pump dispense
        self.assertIn("pump", names)
        first_pump = names.index("pump")
        self.assertIn("move_z", names[:first_pump])
        self.assertIn("wait_z", names[:first_pump])

    def test_aborts_when_descent_unconfirmed(self):
        ctrl = FakeCtrl(arrival=False)       # wait_for_z_arrival fails
        pm = _run(ctrl, _job())
        self.assertEqual(pm.state, PrintState.ERROR)
        # no extrusion happened (we aborted before the path)
        self.assertNotIn("pump", ctrl.names())
        # final safe-Z retract still attempted
        self.assertEqual(ctrl.names().count("retract") >= 1, True)

    def test_aborts_when_m400_times_out(self):
        ctrl = FakeCtrl(m400=False)          # flush_moves times out
        pm = _run(ctrl, _job())
        self.assertEqual(pm.state, PrintState.ERROR)
        self.assertNotIn("pump", ctrl.names())

    def test_aborts_on_zp_disconnect_midprint(self):
        ctrl = FakeCtrl(disconnect_after_descent=True)
        pm = _run(ctrl, _job())
        self.assertEqual(pm.state, PrintState.ERROR)

    def test_dry_run_no_pump(self):
        ctrl = FakeCtrl()
        pm = _run(ctrl, _job(flow_rate=0.0))
        self.assertEqual(pm.state, PrintState.COMPLETED)
        self.assertNotIn("pump", ctrl.names())   # nothing dispensed

    def test_print_path_confirms_every_segment(self):
        """Per segment: XY move + a confirm (flush + xy wait). The number of
        confirmations should match the number of XY moves in the path."""
        ctrl = FakeCtrl()
        _run(ctrl, _job(n=4))
        # at least as many flush (M400) confirms as PRINT_PATH segments
        n_xy = ctrl.names().count("move_xy")
        n_flush = ctrl.names().count("flush")
        self.assertGreaterEqual(n_xy, 4)
        self.assertGreaterEqual(n_flush, 4)

    def test_no_zp_connected_completes_gracefully(self):
        ctrl = FakeCtrl(zp=False)            # ZP never connected (XY-only)
        pm = _run(ctrl, _job(flow_rate=0.0))
        # not connected at start → no mid-print abort; runs the XY parts
        self.assertEqual(pm.state, PrintState.COMPLETED)


if __name__ == "__main__":
    unittest.main()
