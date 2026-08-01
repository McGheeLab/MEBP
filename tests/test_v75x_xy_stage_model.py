"""Tests for SupportClasses/XYStageModel.py — the saved-characteristics stage
motion model (v7.5.x).

The fixture numbers are ME3B V1's real measured dynamics from the 2026-07-28
hardware session (dead time 67 ms, τ 27 ms, top speed 5945.6 µm/s, loop
31.75 ms), so the tests exercise the model exactly where it will be used.
"""

import math
import unittest

from SupportClasses.XYStageModel import StageCharacteristics, XYStageModel


ME3B = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                            top_speed_um_s=5945.6, control_loop_ms=31.75,
                            quant_um=1.0, name="ME3B V1")


def _ideal_char(**kw):
    d = dict(dead_time_s=0.0, tau_s=0.0, top_speed_um_s=1e9,
             control_loop_ms=10.0, quant_um=1e-9)
    d.update(kw)
    return StageCharacteristics(**d)


class TestIntegration(unittest.TestCase):
    def test_ideal_stage_tracks_command_exactly(self):
        m = XYStageModel(_ideal_char())
        m.command_velocity(1000.0, -500.0)
        m.advance(2.0)
        x, y = m.position_exact()
        self.assertAlmostEqual(x, 2000.0, places=6)
        self.assertAlmostEqual(y, -1000.0, places=6)

    def test_dead_time_delays_motion(self):
        m = XYStageModel(_ideal_char(dead_time_s=0.067))
        m.command_velocity(1000.0, 0.0)
        m.advance(0.060)   # still inside the dead time
        self.assertAlmostEqual(m.position_exact()[0], 0.0, places=6)
        m.advance(0.040)   # 0.100 s total → 33 ms of motion
        self.assertAlmostEqual(m.position_exact()[0], 33.0, delta=1.5)

    def test_commands_pipeline_not_resettable_timer(self):
        """Re-commanding FASTER than the dead time must not freeze the stage.

        (An earlier ad-hoc simulation restarted a single delay timer on every
        command, so a 25 Hz command stream against a 67 ms dead time produced
        zero motion forever — this locks the correct pipeline behaviour.)
        """
        m = XYStageModel(_ideal_char(dead_time_s=0.067))
        for _ in range(50):                # 0.5 s of 100 Hz re-commanding
            m.command_velocity(1000.0, 0.0)
            m.advance(0.010)
        # ~(0.5 - 0.067) s of motion at 1000 µm/s
        self.assertGreater(m.position_exact()[0], 400.0)

    def test_top_speed_clamp(self):
        m = XYStageModel(_ideal_char(top_speed_um_s=5945.6))
        m.command_velocity(20000.0, 0.0)
        m.advance(1.0)
        self.assertAlmostEqual(m.position_exact()[0], 5945.6, delta=1.0)

    def test_top_speed_clamp_preserves_direction(self):
        m = XYStageModel(_ideal_char(top_speed_um_s=1000.0))
        m.command_velocity(3000.0, 4000.0)   # mag 5000 → scaled to 1000
        m.advance(1.0)
        x, y = m.position_exact()
        self.assertAlmostEqual(x, 600.0, delta=1.0)
        self.assertAlmostEqual(y, 800.0, delta=1.0)

    def test_first_order_rise(self):
        """After 1τ the velocity is ~63 % of the command; after 5τ ~99 %."""
        m = XYStageModel(_ideal_char(tau_s=0.027))
        m.command_velocity(1000.0, 0.0)
        m.advance(0.027)
        self.assertAlmostEqual(m.velocity()[0], 632.0, delta=15.0)
        m.advance(0.027 * 4)
        self.assertGreater(m.velocity()[0], 990.0)

    def test_quantised_readout(self):
        m = XYStageModel(_ideal_char(quant_um=1.0))
        m.command_velocity(100.0, 0.0)
        m.advance(0.0155)                    # exact x = 1.55 µm
        self.assertEqual(m.read_position()[0], 2.0)
        self.assertAlmostEqual(m.position_exact()[0], 1.55, places=6)

    def test_teleport_clears_pipeline(self):
        m = XYStageModel(ME3B)
        m.command_velocity(5000.0, 0.0)
        m.teleport(100.0, 200.0)
        m.advance(1.0)
        self.assertEqual(m.position_exact(), (100.0, 200.0))

    def test_travel_odometer(self):
        m = XYStageModel(_ideal_char())
        m.command_velocity(1000.0, 0.0)
        m.advance(0.5)
        m.command_velocity(-1000.0, 0.0)
        m.advance(0.5)
        # net ≈ 0 but travel ≈ 1000 µm
        self.assertLess(abs(m.position_exact()[0]), 1.0)
        self.assertAlmostEqual(m.travel_um, 1000.0, delta=5.0)

    def test_me3b_step_response_matches_measurement(self):
        """The full ME3B model: a 3000 µm/s step shows no motion for the dead
        time, then rises with τ — apparent 50 %-speed crossing near
        dead + τ·ln2 ≈ 86 ms."""
        m = XYStageModel(ME3B)
        m.command_velocity(3000.0, 0.0)
        t_half = None
        for _ in range(300):
            m.advance(0.001)
            if t_half is None and m.velocity()[0] >= 1500.0:
                t_half = m.t
        self.assertIsNotNone(t_half)
        self.assertAlmostEqual(t_half, 0.067 + 0.027 * math.log(2), delta=0.005)


class TestCharacteristics(unittest.TestCase):
    def test_round_trip(self):
        d = ME3B.to_dict()
        back = StageCharacteristics.from_dict(d)
        self.assertEqual(back, ME3B)

    def test_from_dict_garbage_safe(self):
        c = StageCharacteristics.from_dict({"dead_time_s": "bogus",
                                            "quant_um": None})
        self.assertEqual(c.dead_time_s, 0.0)
        self.assertEqual(c.quant_um, 1.0)
        self.assertFalse(c.is_complete())
        self.assertIn("dead time", c.missing())

    def test_from_store(self):
        class FakeStore:
            def effective_dead_time_s(self):
                return (0.067, "measured")

            def get_velocity_dead_time_meta(self):
                return {"tau_s": 0.027, "last_updated": "2026-07-28T14:52:42"}

            def get_xy_max_speed_um_s(self):
                return 5945.6

            def get_control_loop_ms(self):
                return 31.75

        c = StageCharacteristics.from_store(FakeStore(), name="ME3B V1")
        self.assertAlmostEqual(c.dead_time_s, 0.067)
        self.assertAlmostEqual(c.tau_s, 0.027)
        self.assertAlmostEqual(c.top_speed_um_s, 5945.6)
        self.assertAlmostEqual(c.control_loop_ms, 31.75)
        self.assertEqual(c.measured_at, "2026-07-28T14:52:42")
        self.assertTrue(c.is_complete())

    def test_from_store_unmeasured(self):
        class Empty:
            def effective_dead_time_s(self):
                return (0.0, "")

            def get_velocity_dead_time_meta(self):
                return {}

            def get_xy_max_speed_um_s(self):
                return 0.0

            def get_control_loop_ms(self):
                return 0.0

        c = StageCharacteristics.from_store(Empty())
        self.assertFalse(c.is_complete())
        self.assertEqual(set(c.missing()),
                         {"comms rate", "dead time", "top speed"})

    def test_from_machine(self):
        class M:
            dead_time_s = 0.04
            tau_s = 0.027
            top_speed_um_s = 5945.6
            control_loop_ms = 32.0

        c = StageCharacteristics.from_machine(M(), name="bench")
        self.assertAlmostEqual(c.dead_time_s, 0.04)
        self.assertEqual(c.name, "bench")
        self.assertTrue(c.is_complete())


if __name__ == "__main__":
    unittest.main()
