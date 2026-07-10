"""v7.5.x tests — Quick Print travel-split + quick-move relief (item 5).

Bug: "the pump isn't stopping between quick moves and the Z doesn't retract — it
doesn't know about the quick moves to the new location." Root cause: a single
object's path was flattened to XY only, so internal travel (pen-up) moves printed
at fixed Z. Fix: split each object into print sub-paths at travel moves so
``build_well_plate_job`` inserts a lift→hop→lower→prime between them; the
``_print_pump_suckback`` hook then sucks back before the hop + re-primes after
(v7.5.x: gated on the single ``backlash_comp_enabled`` toggle).
"""
import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage as QP
from SupportClasses.PrintManager import build_well_plate_job, PrintSettings, CommandType
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory,
)


def _split(traj):
    """Replicate the page split loop using the static _travel_mask."""
    mask = QP._travel_mask(np.asarray(traj, float))
    pts = [(float(traj[i][0]), float(traj[i][1])) for i in range(len(traj))]
    if mask is None:
        return [pts] if len(pts) >= 2 else []
    subs, cur = [], [pts[0]]
    for i in range(len(pts) - 1):
        if bool(mask[i]):
            if len(cur) >= 2:
                subs.append(cur)
            cur = [pts[i + 1]]
        else:
            cur.append(pts[i + 1])
    if len(cur) >= 2:
        subs.append(cur)
    return subs


class TestTravelMask(unittest.TestCase):
    def test_flat_path_is_one_segment(self):
        # constant Z, no pump advance → no travel info → whole path one run.
        traj = np.column_stack([np.linspace(0, 10, 8), np.zeros(8),
                                np.full(8, 0.2), np.zeros((8, 3)),
                                np.arange(8.0)])
        self.assertIsNone(QP._travel_mask(traj))
        self.assertEqual(len(_split(traj)), 1)

    def test_two_shape_sketch_splits_per_shape(self):
        sk = Sketch(num_layers=1)
        sk.shapes = [SketchShape(kind="circle", cx=-5, cy=0, radius=2),
                     SketchShape(kind="circle", cx=5, cy=0, radius=2)]
        res = compile_to_trajectory(sk)
        self.assertEqual(len(_split(res.trajectory)), 2)

    def test_multi_layer_two_shape_splits(self):
        sk = Sketch(num_layers=2)
        sk.shapes = [SketchShape(kind="circle", cx=-5, cy=0, radius=2),
                     SketchShape(kind="circle", cx=5, cy=0, radius=2)]
        res = compile_to_trajectory(sk)
        self.assertEqual(len(_split(res.trajectory)), 4)   # 2 shapes × 2 layers

    def test_pump_advance_marks_print_not_travel(self):
        # Z constant; pump advances on seg 0 (print) but not seg 1 (travel).
        traj = np.array([[0, 0, 0.2, 0, 0, 0, 0],
                         [1, 0, 0.2, 0.5, 0, 0, 1],
                         [2, 0, 0.2, 0.5, 0, 0, 2]], dtype=float)
        mask = QP._travel_mask(traj)
        self.assertIsNotNone(mask)
        self.assertFalse(bool(mask[0]))   # pump advanced → print
        self.assertTrue(bool(mask[1]))    # pump flat → travel

    def test_connected_shapes_print_continuously(self):
        # Regression: shapes sharing a node ("connected lines") must print as
        # ONE continuous bead. The compiler welds them (no pen-up), emitting a
        # zero-distance, pump-flat coincident waypoint at the shared node. That
        # is the join, NOT a "quick move to a new location" — flagging it as
        # travel splits the print and lifts the needle mid-stroke (the reported
        # bug). A travel requires the pump to be flat AND the needle to move.
        sk = Sketch(num_layers=1)
        sk.shapes = [SketchShape(kind="line", points=[(0.0, 0.0), (5.0, 0.0)]),
                     SketchShape(kind="line", points=[(5.0, 0.0), (5.0, 5.0)])]
        res = compile_to_trajectory(sk)
        self.assertEqual(len(_split(res.trajectory)), 1)   # continuous "L"

        # A closed square drawn as four connected lines is still one run.
        sk2 = Sketch(num_layers=1)
        sk2.shapes = [SketchShape(kind="line", points=[(0, 0), (5, 0)]),
                      SketchShape(kind="line", points=[(5, 0), (5, 5)]),
                      SketchShape(kind="line", points=[(5, 5), (0, 5)]),
                      SketchShape(kind="line", points=[(0, 5), (0, 0)])]
        self.assertEqual(len(_split(compile_to_trajectory(sk2).trajectory)), 1)

    def test_disconnected_shapes_still_split(self):
        # Sanity: genuinely separate lines (no shared node) DO split — the
        # travel between them physically repositions the needle.
        sk = Sketch(num_layers=1)
        sk.shapes = [SketchShape(kind="line", points=[(0, 0), (5, 0)]),
                     SketchShape(kind="line", points=[(20, 20), (25, 20)])]
        self.assertEqual(len(_split(compile_to_trajectory(sk).trajectory)), 2)

    def test_zero_distance_pump_flat_segment_is_not_travel(self):
        # A coincident waypoint (duplicate point, pump flat) in the middle of a
        # print run is a weld/join, not a travel — it must not split the run.
        traj = np.array([[0, 0, 0.2, 0.0, 0, 0, 0],
                         [1, 0, 0.2, 0.5, 0, 0, 1],
                         [1, 0, 0.2, 0.5, 0, 0, 2],   # coincident, pump flat
                         [2, 0, 0.2, 1.0, 0, 0, 3]], dtype=float)
        mask = QP._travel_mask(traj)
        self.assertIsNotNone(mask)
        self.assertFalse(bool(mask[1]))   # zero-distance join, NOT travel
        self.assertEqual(len(_split(traj)), 1)

    def test_reversed_z_polarity_splits_by_pump_not_top_zband(self):
        # Regression: some saved sketch/CSV files lift to a LOWER Z — the print
        # plane is numerically ABOVE the travel/lift band (opposite of what the
        # current compiler emits). The old mask OR'd in a "travel = top Z band"
        # heuristic on TOP of the pump signal, so for these files the entire
        # print plane (the HIGH Z) was flagged as travel → every sub-path
        # dropped → blank well preview + an empty, no-op Print. The pump column
        # is polarity-independent and must win whenever it carries info.
        PRINT_Z, LIFT_Z = 12.29, 10.09          # print ABOVE the lift
        rows, p = [], 0.0
        for x in (0, 1, 2):                      # shape 1: pump advances
            p += 0.5
            rows.append([x, 0, PRINT_Z, p, 0, 0, len(rows)])
        rows.append([2, 0, LIFT_Z, p, 0, 0, len(rows)])   # dip to lift, pump flat
        rows.append([5, 0, LIFT_Z, p, 0, 0, len(rows)])   # travel, pump flat
        for x in (5, 6, 7):                      # shape 2: pump advances
            p += 0.5
            rows.append([x, 0, PRINT_Z, p, 0, 0, len(rows)])
        traj = np.array(rows, dtype=float)

        mask = QP._travel_mask(traj)
        self.assertIsNotNone(mask)
        # Only the two pump-flat dip/travel segments are travel — NOT the whole
        # (high-Z) print plane.
        self.assertLess(mask.mean(), 0.5)
        # Both print runs are recovered (the bug collapsed this to []).
        self.assertEqual(len(_split(traj)), 2)


class TestSplitProducesHops(unittest.TestCase):
    """The split's job builds one PRINT_PATH per sub-path with an inter-object
    HOP (MOVE_XY carrying hop_z) between them. The hop is what stops the pump +
    retracts Z; the EXISTING PrintManager._print_pump_suckback("quick_move")
    fires on each such hop (re-primed by the next segment's prime) — so the
    split alone delivers "stop pump + retract Z + relief on quick moves" with no
    extra plan commands."""

    def _job(self):
        st = PrintSettings(num_layers=1, travel_z_height=5.0,
                           print_z_height=0.2, pump_rate_uL_s=0.5)
        st.pump_rates_uL_s["P1"] = 0.5
        segs = [[(0, 0), (1, 0), (2, 0)], [(5, 5), (6, 5), (7, 5)]]
        return build_well_plate_job([("A1", 0.0, 0.0)], [(0, 0)], settings=st,
                                    path_segments=segs, return_home=False)

    def test_one_print_path_per_subpath(self):
        cmds = self._job().commands
        n_paths = sum(1 for c in cmds if c.type == CommandType.PRINT_PATH)
        self.assertEqual(n_paths, 2)

    def test_inter_object_hop_present(self):
        cmds = self._job().commands
        hops = [c for c in cmds
                if c.type == CommandType.MOVE_XY and "hop_z" in (c.params or {})]
        self.assertEqual(len(hops), 1)            # one hop between the 2 sub-paths

    def test_first_object_full_travel_up_then_hop(self):
        cmds = self._job().commands
        # First sub-path approaches from a full TRAVEL_UP; the second is a hop.
        i_travel = next(i for i, c in enumerate(cmds)
                        if c.type == CommandType.TRAVEL_UP)
        i_hop = next(i for i, c in enumerate(cmds)
                     if c.type == CommandType.MOVE_XY and "hop_z" in (c.params or {}))
        self.assertLess(i_travel, i_hop)


if __name__ == "__main__":
    unittest.main()
