"""test_v75x_ludl_simulator.py — end-to-end Ludl path through the simulator.

Drives XYStageManager(simulate=True, controller_json=mac5000.json) so the
LudlStageSimulator is selected and the full µm→count→µm round-trip
(MOVE / WHERE / MOVREL / SPEED / ACCEL / HALT) is exercised.
"""

import time
import unittest

from SupportClasses.XYStage import XYStageManager
from SupportClasses.LudlStageSimulator import LudlStageSimulator

MAC5000 = "config/controllers/mac5000.json"

# Isolate simulator state from the real config/sim_xy_state_ludl.json.
import tempfile
from pathlib import Path


class TestLudlSimulatorViaManager(unittest.TestCase):
    def setUp(self):
        self.xy = XYStageManager(simulate=True, controller_json=MAC5000)

    def tearDown(self):
        try:
            self.xy.stop()
        except Exception:
            pass

    def test_backend_is_ludl(self):
        self.assertIsInstance(self.xy.spo, LudlStageSimulator)
        self.assertEqual(self.xy._protocol.family, "ludl")
        self.assertEqual(self.xy._position_scale(), 10.0)

    def test_move_absolute_round_trips_in_um(self):
        self.xy.move_stage_to_position(1000, 2000)  # blocking (sim waits for idle)
        x, y, _ = self.xy.get_current_position()
        self.assertAlmostEqual(x, 1000.0, delta=1.0)
        self.assertAlmostEqual(y, 2000.0, delta=1.0)

    def test_where_wire_is_counts(self):
        self.xy.move_stage_to_position(1000, 2000)
        raw = self.xy.spo.send_command("WHERE X Y")
        self.assertEqual(raw, ":A 10000 20000")   # counts on the wire
        # And the manager parses it back to µm.
        self.assertEqual(self.xy._parse_position_response(raw), (1000.0, 2000.0, 0.0))

    def test_move_relative(self):
        self.xy.move_stage_to_position(1000, 0)
        self.xy.move_stage_relative(-500, 0)
        x, _, _ = self.xy.get_current_position()
        self.assertAlmostEqual(x, 500.0, delta=1.0)

    def test_set_speed_and_accel_accepted(self):
        # Should not raise; sim returns :A.
        self.xy.set_speed_mm_s(5.0)
        self.xy.set_acceleration(100)

    def test_readback_round_trip_through_real_sim(self):
        """Real end-to-end check (no mocking) of the read-back added after
        the real-hardware incident where X's SPEED silently diverged from
        Y's — set genuinely different per-axis values and read them back."""
        self.xy.spo.send_command("SPEED X=84 Y=230400")
        self.xy.spo.send_command("ACCEL X=1 Y=255")
        speed_x = self.xy.get_speed_readback("X")
        speed_y = self.xy.get_speed_readback("Y")
        accel_x = self.xy.get_acceleration_readback("X")
        accel_y = self.xy.get_acceleration_readback("Y")
        self.assertEqual(speed_x["raw"], 84)
        self.assertEqual(speed_y["raw"], 230400)
        self.assertNotEqual(speed_x["raw"], speed_y["raw"])  # the divergence itself
        self.assertEqual(accel_x["raw"], 1)
        self.assertEqual(accel_y["raw"], 255)

    def test_halt_stops(self):
        resp = self.xy.spo.send_command("HALT")
        self.assertEqual(resp, ":A")

    def test_version_identifies_ludl(self):
        resp = self.xy.spo.send_command("VER")
        self.assertTrue(resp.startswith(":A"))

    def test_unknown_command_errors(self):
        self.assertEqual(self.xy.spo.send_command("NONSENSE"), ":N -1")


class TestLudlSimulatorDirect(unittest.TestCase):
    def _sim(self):
        # Fresh state file so persisted position doesn't leak between runs.
        tmp = Path(tempfile.mkdtemp()) / "ludl_state.json"
        sim = LudlStageSimulator(position_scale=10.0, state_file=tmp)
        sim.start()
        return sim

    def test_here_sets_position(self):
        sim = self._sim()
        try:
            self.assertEqual(sim.send_command("HERE X=0 Y=0"), ":A")
            sim.send_command("MOVE X=5000 Y=5000")  # 500 µm
            time.sleep(0.3)
            x, y, _ = sim.get_current_position()
            self.assertAlmostEqual(x, 500.0, delta=2.0)
        finally:
            sim.stop()

    def test_status_poll(self):
        sim = self._sim()
        try:
            self.assertIn(sim.send_command("/"), ("B", "N"))
        finally:
            sim.stop()


if __name__ == "__main__":
    unittest.main()
