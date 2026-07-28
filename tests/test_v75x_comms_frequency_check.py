"""
test_v75x_comms_frequency_check.py — XY-stage communication-frequency checker.

Covers:
  • StageController.measure_control_loop_rate — the interleaved send-velocity +
    read-position cadence measured WHILE the stage is moving (the loop_period
    that governs the velocity-following print's max stable speed). Verifies it
    drives motion, reverses at the amplitude bound (net ~0), ALWAYS stops the
    stage + returns to start, suspends/resumes the poller, and reports stats.
  • PrintTimingCalibrationStore control-loop persistence + the derived max
    stable velocity-follow speed (v ≈ lookahead / (loop_period · safety)).

No GUI / hardware required.
"""

import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.StageController import StageController
from SupportClasses.PrintTimingCalibrationStore import PrintTimingCalibrationStore


class _Sim:
    """Deterministic 1-D stage sim: position advances by v·SIM_DT each read
    (simulated clock, independent of wall time), so motion/reversal is
    reproducible without sleeping."""

    SIM_DT = 0.02  # simulated seconds per read

    def __init__(self, start_x=10000.0, start_y=5000.0):
        self.x = start_x
        self.y = start_y
        self.vx = 0.0
        self.vel_cmds = []

    def send_velocity(self, vx, vy):
        self.vx = vx
        self.vel_cmds.append((vx, vy))

    def read(self, cached=True):
        self.x += self.vx * self.SIM_DT
        return (self.x, self.y, 0.0)


def _ctrl(sim):
    c = StageController.__new__(StageController)
    c.xy_stage = MagicMock()          # makes is_xy_connected True
    c.safety_limits = SimpleNamespace(max_xy_speed=50000.0, enabled=False)
    c.get_xy_position = lambda cached=True: sim.read(cached)
    c.send_velocity_xy = lambda vx, vy: sim.send_velocity(vx, vy)
    c.move_xy_absolute_um = MagicMock()
    c.suspend_position_poller = MagicMock()
    c.resume_position_poller = MagicMock()
    return c


class TestMeasureControlLoopRate(unittest.TestCase):
    def test_returns_stats_and_moves(self):
        sim = _Sim()
        c = _ctrl(sim)
        res = c.measure_control_loop_rate(iterations=40, speed_um_s=2000.0,
                                          amplitude_um=600.0)
        self.assertNotIn("error", res)
        self.assertEqual(res["iterations"], 40)
        self.assertTrue(res["moved"])
        self.assertGreater(res["control_hz"], 0.0)
        self.assertGreaterEqual(res["avg_period_ms"], 0.0)
        # excursion stayed bounded near the requested amplitude (never ran away)
        self.assertLess(res["max_excursion_um"], 600.0 + 200.0)

    def test_reverses_direction_at_bound(self):
        sim = _Sim()
        c = _ctrl(sim)
        c.measure_control_loop_rate(iterations=40, speed_um_s=2000.0,
                                    amplitude_um=600.0)
        signs = {1 if vx > 0 else (-1 if vx < 0 else 0)
                 for vx, _vy in sim.vel_cmds}
        self.assertIn(1, signs)
        self.assertIn(-1, signs)   # it reversed → net motion ~0

    def test_always_stops_and_returns_to_start(self):
        sim = _Sim()
        c = _ctrl(sim)
        c.measure_control_loop_rate(iterations=10)
        self.assertEqual(sim.vel_cmds[-1], (0.0, 0.0))   # stopped on exit
        c.move_xy_absolute_um.assert_called_once()
        args = c.move_xy_absolute_um.call_args[0]
        self.assertAlmostEqual(args[0], 10000.0, places=3)
        self.assertAlmostEqual(args[1], 5000.0, places=3)

    def test_suspends_and_resumes_poller(self):
        sim = _Sim()
        c = _ctrl(sim)
        c.measure_control_loop_rate(iterations=5)
        c.suspend_position_poller.assert_called_once()
        c.resume_position_poller.assert_called_once()

    def test_speed_clamped_to_safety_envelope(self):
        sim = _Sim()
        c = _ctrl(sim)
        c.safety_limits.max_xy_speed = 1500.0   # cap below requested 5000
        c.measure_control_loop_rate(iterations=6, speed_um_s=5000.0)
        # every commanded magnitude respected the cap
        for vx, _vy in sim.vel_cmds:
            self.assertLessEqual(abs(vx), 1500.0 + 1e-6)

    def test_not_connected_returns_error(self):
        c = StageController.__new__(StageController)
        c.xy_stage = None
        res = c.measure_control_loop_rate()
        self.assertIn("error", res)

    def test_bad_position_read_returns_error(self):
        c = StageController.__new__(StageController)
        c.xy_stage = MagicMock()
        c.safety_limits = SimpleNamespace(max_xy_speed=50000.0, enabled=False)
        c.get_xy_position = lambda cached=True: (None, None, None)
        c.send_velocity_xy = MagicMock()
        c.suspend_position_poller = MagicMock()
        c.resume_position_poller = MagicMock()
        res = c.measure_control_loop_rate()
        self.assertIn("error", res)


class TestStoreControlLoop(unittest.TestCase):
    def _store(self):
        fd, path = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        os.remove(path)
        self.addCleanup(lambda: os.path.exists(path) and os.remove(path))
        return PrintTimingCalibrationStore(path=path)

    def test_none_before_measured(self):
        st = self._store()
        self.assertIsNone(st.get_control_loop_ms())
        self.assertIsNone(st.stable_velocity_speed_mm_s())

    def test_set_get_roundtrip(self):
        st = self._store()
        st.set_control_loop_ms(150.0)
        self.assertEqual(st.get_control_loop_ms(), 150.0)

    def test_stable_speed_formula(self):
        st = self._store()
        st.set_control_loop_ms(150.0)   # 0.15 s
        # v = lookahead / (loop_s · safety) = 0.6 / (0.15 · 2) = 2.0 mm/s
        v = st.stable_velocity_speed_mm_s(lookahead_mm=0.6, safety=2.0)
        self.assertAlmostEqual(v, 2.0, places=3)

    def test_stable_speed_uses_stored_lookahead_default(self):
        st = self._store()
        st.set_control_loop_ms(100.0)   # 0.1 s
        st.set_mode_params("velocity", {"lookahead_mm": 1.0})
        # default lookahead from store (1.0): 1.0 / (0.1 · 2) = 5.0 mm/s
        v = st.stable_velocity_speed_mm_s(safety=2.0)
        self.assertAlmostEqual(v, 5.0, places=3)

    def test_persists_across_reload(self):
        st = self._store()
        st.set_control_loop_ms(123.0)
        st2 = PrintTimingCalibrationStore(path=st._path)
        self.assertEqual(st2.get_control_loop_ms(), 123.0)


if __name__ == "__main__":
    unittest.main()
