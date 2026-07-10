"""
test_v75x_sketch_continuous_connected_lines.py — Print Builder Sketch compiler
welds paths that connect at a shared node into one continuous bead (no pen-up /
travel between them).

No GUI / hardware required.
"""

import unittest

import numpy as np

from SupportClasses.SketchTrajectory import (
    Sketch,
    SketchShape,
    compile_to_trajectory,
)


def _line(x1, y1, x2, y2, pump=0) -> SketchShape:
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)],
                       ink_id=pump + 1)   # 0-based pump → 1-based abstract ink


class TestContinuousConnectedLines(unittest.TestCase):

    Z_START = 0.2
    LAYER_H = 0.2
    CLEAR = 2.0

    def _sketch(self, shapes) -> Sketch:
        return Sketch(shapes=shapes, z_start_mm=self.Z_START,
                      layer_height_mm=self.LAYER_H, num_layers=1,
                      travel_clearance_mm=self.CLEAR)

    def _z_travel(self) -> float:
        return self.Z_START + 1 * self.LAYER_H + self.CLEAR

    def _n_lifts(self, traj: np.ndarray) -> int:
        """Count waypoints raised to the travel-clearance Z."""
        return int(np.sum(np.isclose(traj[:, 2], self._z_travel())))

    def test_connected_lines_print_continuously(self):
        # Two lines sharing the node (10, 0) — drawn head-to-tail.
        sk = self._sketch([_line(0, 0, 10, 0), _line(10, 0, 10, 10)])
        res = compile_to_trajectory(sk)
        # Only the final retract should reach travel Z — no lift between them.
        self.assertEqual(self._n_lifts(res.trajectory), 1)

    def test_disconnected_lines_travel_between(self):
        # Same two lines but the second is far away — must lift + travel.
        sk = self._sketch([_line(0, 0, 10, 0), _line(50, 50, 60, 60)])
        res = compile_to_trajectory(sk)
        # Inter-line travel (lift + move-over) plus the final retract.
        self.assertGreater(self._n_lifts(res.trajectory), 1)

    def test_weld_flips_path_when_only_far_endpoint_connects(self):
        # Second line stored reversed (P2 -> shared node) — still welds by
        # flipping the path so the shared node leads.
        sk = self._sketch([_line(0, 0, 10, 0), _line(10, 10, 10, 0)])
        res = compile_to_trajectory(sk)
        self.assertEqual(self._n_lifts(res.trajectory), 1)

    def test_chain_of_lines_is_one_continuous_run(self):
        # A 4-segment square drawn node-to-node prints with no lift at all
        # until the final retract.
        sk = self._sketch([
            _line(0, 0, 10, 0),
            _line(10, 0, 10, 10),
            _line(10, 10, 0, 10),
            _line(0, 10, 0, 0),
        ])
        res = compile_to_trajectory(sk)
        self.assertEqual(self._n_lifts(res.trajectory), 1)

    def test_connected_lines_preserve_printed_length(self):
        # Welding must not drop printed material — two 10 mm lines still total
        # ~20 mm of printed path.
        sk = self._sketch([_line(0, 0, 10, 0), _line(10, 0, 10, 10)])
        res = compile_to_trajectory(sk)
        self.assertAlmostEqual(res.total_length_mm, 20.0, delta=0.1)

    def test_layer_change_still_lifts_even_if_xy_coincides(self):
        # A single line over 2 layers: the second layer must lift to reposition
        # (different print Z) — welding only applies within a layer.
        sk = Sketch(shapes=[_line(0, 0, 10, 0)], z_start_mm=self.Z_START,
                    layer_height_mm=self.LAYER_H, num_layers=2,
                    travel_clearance_mm=self.CLEAR)
        res = compile_to_trajectory(sk)
        z_travel = self.Z_START + 2 * self.LAYER_H + self.CLEAR
        n_lifts = int(np.sum(np.isclose(res.trajectory[:, 2], z_travel)))
        self.assertGreater(n_lifts, 1)


if __name__ == "__main__":
    unittest.main()
