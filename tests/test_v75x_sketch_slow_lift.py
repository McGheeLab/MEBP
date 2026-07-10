"""
test_v75x_sketch_slow_lift.py — Print Builder Sketch compiler retracts the
needle's first ``lift_slow_dist_mm`` slowly (so the bead doesn't lift off with
the needle), then finishes the lift at travel speed.

No GUI / hardware required.
"""

import unittest

from SupportClasses.SketchTrajectory import (
    Sketch,
    SketchShape,
    compile_to_trajectory,
)


def _line(x1, y1, x2, y2) -> SketchShape:
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)])


class TestSlowLift(unittest.TestCase):

    def _sketch(self, *, clearance=2.0, slow_dist=1.0, slow_speed=1.0,
                travel_speed=20.0, print_speed=5.0) -> Sketch:
        return Sketch(
            shapes=[_line(0, 0, 10, 0)],
            z_start_mm=0.2, layer_height_mm=0.2, num_layers=1,
            print_speed_mm_s=print_speed, travel_speed_mm_s=travel_speed,
            travel_clearance_mm=clearance,
            lift_slow_dist_mm=slow_dist, lift_slow_speed_mm_s=slow_speed)

    def test_lift_split_into_slow_then_fast(self):
        sk = self._sketch()
        traj = compile_to_trajectory(sk).trajectory
        z_print = 0.2
        z_travel = 0.2 + 0.2 + 2.0          # = 2.4
        z_slow_top = z_print + 1.0          # = 1.2
        # Final retract = last two segments: slow to z_slow_top, fast to z_travel.
        self.assertAlmostEqual(traj[-2, 2], z_slow_top, places=6)
        self.assertAlmostEqual(traj[-1, 2], z_travel, places=6)
        # XY held constant through the lift (vertical retract).
        self.assertAlmostEqual(traj[-3, 0], traj[-1, 0], places=6)
        self.assertAlmostEqual(traj[-3, 1], traj[-1, 1], places=6)
        # Slow segment timed at the slow speed (1 mm / 1 mm/s = 1.0 s).
        dt_slow = traj[-2, 6] - traj[-3, 6]
        self.assertAlmostEqual(dt_slow, 1.0 / 1.0, places=4)
        # Remainder timed at travel speed (1.2 mm / 20 mm/s = 0.06 s).
        dt_fast = traj[-1, 6] - traj[-2, 6]
        self.assertAlmostEqual(dt_fast, (z_travel - z_slow_top) / 20.0, places=4)

    def test_slow_segment_is_slower_than_fast(self):
        sk = self._sketch()
        traj = compile_to_trajectory(sk).trajectory
        dz_slow = traj[-2, 2] - traj[-3, 2]
        dt_slow = traj[-2, 6] - traj[-3, 6]
        dz_fast = traj[-1, 2] - traj[-2, 2]
        dt_fast = traj[-1, 6] - traj[-2, 6]
        v_slow = dz_slow / dt_slow
        v_fast = dz_fast / dt_fast
        self.assertLess(v_slow, v_fast)
        self.assertAlmostEqual(v_slow, 1.0, places=3)
        self.assertAlmostEqual(v_fast, 20.0, places=3)

    def test_short_lift_is_entirely_slow(self):
        # Total lift (0.5 mm) is shorter than the slow distance → all slow,
        # no separate fast segment.
        sk = self._sketch(clearance=0.3)          # z_travel = 0.7, lift = 0.5
        traj = compile_to_trajectory(sk).trajectory
        z_travel = 0.2 + 0.2 + 0.3
        self.assertAlmostEqual(traj[-1, 2], z_travel, places=6)
        # The final retract is a single segment at the slow speed.
        dt = traj[-1, 6] - traj[-2, 6]
        self.assertAlmostEqual(dt, 0.5 / 1.0, places=4)

    def test_zero_slow_dist_lifts_all_at_travel_speed(self):
        # Disabling the slow lift reproduces the legacy single fast retract.
        sk = self._sketch(slow_dist=0.0)
        traj = compile_to_trajectory(sk).trajectory
        z_print = 0.2
        z_travel = 0.2 + 0.2 + 2.0
        self.assertAlmostEqual(traj[-1, 2], z_travel, places=6)
        dt = traj[-1, 6] - traj[-2, 6]
        self.assertAlmostEqual(dt, (z_travel - z_print) / 20.0, places=4)

    def test_serialization_round_trip(self):
        sk = self._sketch(slow_dist=1.5, slow_speed=2.0)
        sk2 = Sketch.from_dict(sk.to_dict())
        self.assertAlmostEqual(sk2.lift_slow_dist_mm, 1.5)
        self.assertAlmostEqual(sk2.lift_slow_speed_mm_s, 2.0)


if __name__ == "__main__":
    unittest.main()
