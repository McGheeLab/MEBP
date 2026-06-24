"""
v7.5.x — Multi-object print seam fix (build_well_plate_job path_segments).

Bug: a Quick Print of a saved multi-spiral file concatenated all objects into
ONE PRINT_PATH, so the seams between spirals were executed as EXTRUDED moves
at print height — the needle drove across already-printed material laying down
a line ("jump to a new location, drive through printed material, continue").

Fix: build_well_plate_job accepts path_segments (one sub-path per object) and
emits one PRINT_PATH per segment with a full TRAVEL_UP → MOVE_XY → MOVE_Z
prologue between them. The MOVE_XY handler retracts Z and confirms arrival
before the XY move, so the seam becomes a safe lift→travel→lower hop with NO
extrusion (PRINT_PATH is the only extruding command; MOVE_XY does not extrude).

These tests assert the command structure; the logging integration test
(test_v75x_print_execution_logging) covers execution.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SupportClasses.PrintManager import (
    build_well_plate_job, PrintSettings, CommandType,
)


def _types(job):
    return [c.type for c in job.commands]


def _print_paths(job):
    return [c for c in job.commands if c.type == CommandType.PRINT_PATH]


class TestSingleSegmentBackCompat(unittest.TestCase):
    """path_segments=None must reproduce the legacy single-PRINT_PATH plan."""

    def test_none_equals_legacy(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        st = PrintSettings(num_layers=1)
        job_legacy = build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)], path_points=pts, settings=st)
        job_seg_none = build_well_plate_job(
            well_positions=[("A1", 10.0, 20.0)], path_points=pts, settings=st,
            path_segments=None)
        self.assertEqual(_types(job_legacy), _types(job_seg_none))
        # Exactly one PRINT_PATH, offset by the well center
        pp = _print_paths(job_legacy)
        self.assertEqual(len(pp), 1)
        self.assertEqual(pp[0].params["points"][0], (10.0, 20.0))
        self.assertEqual(pp[0].params["points"][1], (11.0, 20.0))

    def test_single_explicit_segment_equals_legacy(self):
        pts = [(0.0, 0.0), (1.0, 0.0)]
        st = PrintSettings(num_layers=1)
        job_legacy = build_well_plate_job(
            well_positions=[("A1", 5.0, 5.0)], path_points=pts, settings=st)
        job_seg = build_well_plate_job(
            well_positions=[("A1", 5.0, 5.0)], path_points=pts, settings=st,
            path_segments=[pts])
        self.assertEqual(_types(job_legacy), _types(job_seg))


class TestMultiSegment(unittest.TestCase):
    """Multiple objects → one PRINT_PATH each with travel between them."""

    def setUp(self):
        # Two distinct objects at different offsets within the well.
        self.seg_a = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)]
        self.seg_b = [(3.0, 3.0), (3.5, 3.0), (3.5, 3.5)]
        self.st = PrintSettings(num_layers=1)
        self.job = build_well_plate_job(
            well_positions=[("A1", 70.0, 48.0)],
            path_points=self.seg_a + self.seg_b,  # flattened (guard only)
            settings=self.st,
            path_segments=[self.seg_a, self.seg_b],
            job_name="multi",
        )

    def test_one_print_path_per_object(self):
        pp = _print_paths(self.job)
        self.assertEqual(len(pp), 2)

    def test_travel_prologue_between_objects(self):
        """Each PRINT_PATH is immediately preceded by MOVE_Z + MOVE_XY (a
        travel to the object's start). The FIRST object approaches from the
        full travel height (TRAVEL_UP); SUBSEQUENT objects use a small intra-
        well hop (MOVE_XY carries hop_z, no TRAVEL_UP)."""
        types = _types(self.job)
        pp_idx = [i for i, t in enumerate(types)
                  if t == CommandType.PRINT_PATH]
        self.assertEqual(len(pp_idx), 2)
        for i in pp_idx:
            # ... MOVE_XY, MOVE_Z, PRINT_PATH ...
            self.assertEqual(types[i - 1], CommandType.MOVE_Z)
            self.assertEqual(types[i - 2], CommandType.MOVE_XY)
        # First object: full TRAVEL_UP approach.
        self.assertEqual(types[pp_idx[0] - 3], CommandType.TRAVEL_UP)
        # Second object: NO TRAVEL_UP — just the hop MOVE_XY.
        self.assertNotEqual(types[pp_idx[1] - 3], CommandType.TRAVEL_UP)

    def test_intra_well_hop_is_small_lift(self):
        """The seam between objects uses a small hop_z (≈ print Z + hop_mm in
        the up direction), NOT the full travel height."""
        moves = [c for c in self.job.commands
                 if c.type == CommandType.MOVE_XY]
        # First object's MOVE_XY = full travel retract (no hop_z).
        self.assertNotIn("hop_z", moves[0].params)
        # Second object's MOVE_XY = small hop.
        self.assertIn("hop_z", moves[1].params)
        z_up = getattr(self.st, "z_up_sign", 1.0)
        expected = self.st.print_z_height + z_up * self.st.intra_well_hop_z_mm
        self.assertAlmostEqual(moves[1].params["hop_z"], expected)
        # The hop is far from the full travel height.
        self.assertNotAlmostEqual(moves[1].params["hop_z"],
                                  self.st.travel_z_height)

    def test_hop_respects_setting(self):
        seg_a = [(0.0, 0.0), (0.5, 0.0)]
        seg_b = [(3.0, 3.0), (3.5, 3.0)]
        st = PrintSettings(num_layers=1, print_z_height=0.0,
                           intra_well_hop_z_mm=2.5)
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)], path_points=seg_a + seg_b,
            settings=st, path_segments=[seg_a, seg_b])
        moves = [c for c in job.commands if c.type == CommandType.MOVE_XY]
        z_up = getattr(st, "z_up_sign", 1.0)
        self.assertAlmostEqual(moves[1].params["hop_z"], 0.0 + z_up * 2.5)

    def test_second_object_travels_to_its_own_start(self):
        """The seam MOVE_XY targets the SECOND object's first point (offset by
        the well center) — not a continuation of the first object."""
        moves = [c for c in self.job.commands
                 if c.type == CommandType.MOVE_XY]
        # First MOVE_XY → seg_a start; second → seg_b start.
        self.assertAlmostEqual(moves[0].params["x"], 70.0 + 0.0)
        self.assertAlmostEqual(moves[0].params["y"], 48.0 + 0.0)
        self.assertAlmostEqual(moves[1].params["x"], 70.0 + 3.0)
        self.assertAlmostEqual(moves[1].params["y"], 48.0 + 3.0)

    def test_no_extruding_segment_across_the_seam(self):
        """The 4+ mm seam must NOT appear as a segment inside any PRINT_PATH.

        Within each PRINT_PATH, consecutive points stay within the object's own
        extent; the large inter-object gap is bridged only by the (non-
        extruding) MOVE_XY travel command.
        """
        import math
        for pp in _print_paths(self.job):
            pts = pp.params["points"]
            for j in range(1, len(pts)):
                seg = math.hypot(pts[j][0] - pts[j - 1][0],
                                 pts[j][1] - pts[j - 1][1])
                self.assertLess(seg, 1.0,
                                "an inter-object jump leaked into a PRINT_PATH")

    def test_empty_segments_skipped(self):
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)],
            path_points=self.seg_a,
            settings=self.st,
            path_segments=[self.seg_a, [], self.seg_b],
        )
        self.assertEqual(len(_print_paths(job)), 2)


class TestHopExecution(unittest.TestCase):
    """The MOVE_XY handler routes a hop_z param through ensure_retracted_to
    with the SMALL target (not the full travel height)."""

    def test_hop_z_retracts_to_small_target(self):
        from SupportClasses.PrintManager import (
            PrintManager, PrintJob, PrintCommand, CommandType, PrintSettings,
        )

        retract_targets = []

        class Ctrl:
            is_xy_connected = True
            is_zp_connected = True

            def __init__(self):
                self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
                self.xy_stage = None

            def ensure_retracted_to(self, z, **kw):
                retract_targets.append(round(float(z), 4))
                return True

            def move_xy_absolute(self, x, y, from_zero_ref=True, fast=False):
                pass

            def get_xy_position(self, cached=True):
                return (None, None)

        ctrl = Ctrl()
        pm = PrintManager(ctrl)
        pm.job = PrintJob(settings=PrintSettings(travel_z_height=-37.5,
                                                 print_z_height=-16.0))
        # Full travel MOVE_XY → ensure_retracted_to(travel_z)
        pm._execute_command(PrintCommand(CommandType.MOVE_XY,
                                         {"x": 1.0, "y": 1.0}))
        # Hop MOVE_XY → ensure_retracted_to(hop_z), NOT travel_z
        pm._execute_command(PrintCommand(CommandType.MOVE_XY,
                                         {"x": 2.0, "y": 2.0, "hop_z": -17.0}))
        self.assertEqual(retract_targets, [-37.5, -17.0])


class TestMultiSegmentMultiLayer(unittest.TestCase):
    def test_segments_repeat_per_layer(self):
        st = PrintSettings(num_layers=2)
        seg_a = [(0.0, 0.0), (1.0, 0.0)]
        seg_b = [(2.0, 2.0), (3.0, 2.0)]
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)],
            path_points=seg_a + seg_b, settings=st,
            path_segments=[seg_a, seg_b])
        # 2 objects × 2 layers = 4 PRINT_PATHs
        self.assertEqual(len(_print_paths(job)), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
