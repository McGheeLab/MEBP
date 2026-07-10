"""test_v75x_extrusion_volume_calc.py — extrusion / print-volume audit fixes.

Covers the three fixes from MEBP_v75x_EXTRUSION_VOLUME_CALC_AUDIT.md, motivated
by the "27 G needs ~7× extrusion to print anything" report:

  (C) PrintManager._execute_print_path ACCUMULATES sub-threshold per-segment
      pump volume instead of silently dropping it — so a finely-sampled path at
      low flow dispenses the SAME total volume as a coarse path of equal length
      (before: tiny segments < 0.001 µL were dropped and NOT carried forward →
      cumulative under-extrusion, worst at low extrusion).

  (A) SketchTrajectory.compile_to_trajectory uses the CANONICAL bore-area × mult
      bead model (matching FlowPhysics / GeometryEngine / Quick Print), so a
      sketch-baked print and a Quick Print of the same needle+path agree; the
      baked volume no longer depends on layer_height.

  (E) generate_object_trajectory now HONORS PrintObject.extrusion_modifier and
      threads it to the pump columns / total volume (the parametric / trajectory
      print path was previously locked at 1× with the field ignored).
"""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType,
)
from SupportClasses.GeometryEngine import (
    PrintObject, generate_object_trajectory,
    compute_pump_positions, compute_total_volume,
)
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory,
)
from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec


NEEDLE_27G = NeedleSpec(gauge=27, od_um=413, id_um=210, wall_um=102)
SYR_100 = SyringeSpec(volume_uL=100, stroke_length_mm=30.0, barrel_id_mm=2.06)


# ═══════════════════════════════════════════════════════════════════
# (C) PrintManager per-segment pump accumulation
# ═══════════════════════════════════════════════════════════════════

class _RecordZP:
    def __init__(self):
        self.axis_map = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}

    def flush_moves(self, timeout_s=10.0):
        return True

    def move_relative(self, axes, feedrate=None):
        pass


class _RecordCtrl:
    """Minimal controller that records the µL of every pump move."""

    def __init__(self):
        self.is_zp_connected = True
        self.zp_stage = _RecordZP()
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.safety_limits = SimpleNamespace(enabled=False)
        self.pump_moves = []          # list of (pump, volume_uL, rate)

    def move_pump_uL(self, pump, volume_uL, rate):
        self.pump_moves.append((pump, float(volume_uL), rate))

    def move_xy_absolute(self, x, y, from_zero_ref=True):
        pass


def _run_path(points, flow, speed=5.0):
    ctrl = _RecordCtrl()
    pm = PrintManager(ctrl)
    pm.job = SimpleNamespace(settings=PrintSettings(
        print_speed_mm_s=speed, print_feedrate=speed * 60.0))
    pm.exec_logger = None
    pm._set_xy_speed_for_print = lambda *a, **k: None
    pm._wait_for_xy_settle = lambda *a, **k: None
    cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
        "points": points, "pump": "P2",
        "flow_rate_uL_s": flow, "flow_rate": 0.01})
    with patch("time.sleep"):
        pm._execute_print_path(cmd)
    return ctrl


def _line_points(total_mm, n_segments):
    """Collinear points spanning total_mm in n_segments equal steps."""
    return [(total_mm * i / n_segments, 0.0) for i in range(n_segments + 1)]


class TestPumpAccumulation(unittest.TestCase):
    FLOW = 0.02      # µL/s
    SPEED = 5.0      # mm/s → vol/mm = 0.004 µL/mm
    LEN = 2.0        # mm  → expected total = 0.008 µL

    def _expected(self):
        return self.FLOW * (self.LEN / self.SPEED)     # 0.008 µL

    def test_fine_path_conserves_volume(self):
        # 200 tiny segments (0.01 mm each): each segment's nominal volume
        # (0.00004 µL) is far below the 0.001 µL emit floor. Pre-fix they were
        # all dropped → ~0 dispensed. Post-fix the residual accumulates + the
        # tail flushes → the FULL expected volume is dispensed.
        ctrl = _run_path(_line_points(self.LEN, 200), self.FLOW, self.SPEED)
        total = sum(v for _p, v, _r in ctrl.pump_moves)
        self.assertAlmostEqual(total, self._expected(), places=6)

    def test_fine_matches_coarse(self):
        # Same total length + flow, wildly different sampling → same volume.
        fine = _run_path(_line_points(self.LEN, 200), self.FLOW, self.SPEED)
        coarse = _run_path(_line_points(self.LEN, 2), self.FLOW, self.SPEED)
        tf = sum(v for _p, v, _r in fine.pump_moves)
        tc = sum(v for _p, v, _r in coarse.pump_moves)
        self.assertAlmostEqual(tf, tc, places=6)

    def test_fine_path_emits_fewer_moves_than_segments(self):
        # Accumulation means far fewer emitted moves than segments (they batch
        # up to the emit floor), but every emitted move is a real (>= floor)
        # move plus one tail flush.
        ctrl = _run_path(_line_points(self.LEN, 200), self.FLOW, self.SPEED)
        self.assertGreater(len(ctrl.pump_moves), 0)
        self.assertLess(len(ctrl.pump_moves), 200)

    def test_no_flow_emits_nothing(self):
        ctrl = _run_path(_line_points(self.LEN, 50), 0.0, self.SPEED)
        self.assertEqual(ctrl.pump_moves, [])

    def test_coarse_path_still_emits_per_segment(self):
        # Each 1 mm segment at 0.004 µL/mm = 0.004 µL > floor → emits every
        # segment (regression: normal prints are unchanged).
        ctrl = _run_path(_line_points(10.0, 10), self.FLOW, self.SPEED)
        self.assertEqual(len(ctrl.pump_moves), 10)
        total = sum(v for _p, v, _r in ctrl.pump_moves)
        self.assertAlmostEqual(total, self.FLOW * (10.0 / self.SPEED), places=6)


# ═══════════════════════════════════════════════════════════════════
# (A) Sketch uses the canonical bore-area bead model
# ═══════════════════════════════════════════════════════════════════

class TestSketchCanonicalBeadModel(unittest.TestCase):
    def _line(self, mult=1.0, lh=0.2):
        sk = Sketch(shapes=[SketchShape(
            kind="line", points=[(0.0, 0.0), (10.0, 0.0)])])
        sk.line_spacing_mm = 0.72       # ≈ outer Ø — must NOT enter the volume
        sk.layer_height_mm = lh
        sk.extrusion_multiplier = mult
        return sk

    def test_volume_equals_bore_area_times_length(self):
        c = compile_to_trajectory(self._line(1.0), NEEDLE_27G, None)
        expected = 10.0 * NEEDLE_27G.cross_section_area_mm2
        self.assertAlmostEqual(c.total_volume_uL, expected, places=6)

    def test_matches_geometry_engine_compute_total_volume(self):
        # Sketch and the canonical GeometryEngine helper agree for the same
        # needle + printed length (the whole point of the unification).
        c = compile_to_trajectory(self._line(1.0), NEEDLE_27G, None)
        gen = compute_total_volume(c.total_length_mm, NEEDLE_27G)
        self.assertAlmostEqual(c.total_volume_uL, gen, places=6)

    def test_volume_independent_of_layer_height(self):
        v_thin = compile_to_trajectory(
            self._line(1.0, lh=0.1), NEEDLE_27G, None).total_volume_uL
        v_thick = compile_to_trajectory(
            self._line(1.0, lh=0.5), NEEDLE_27G, None).total_volume_uL
        self.assertAlmostEqual(v_thin, v_thick, places=6)

    def test_multiplier_scales_linearly(self):
        v1 = compile_to_trajectory(self._line(1.0), NEEDLE_27G, None).total_volume_uL
        v3 = compile_to_trajectory(self._line(3.0), NEEDLE_27G, None).total_volume_uL
        self.assertAlmostEqual(v3 / v1, 3.0, places=6)

    def test_no_needle_fallback_still_previewable(self):
        # Without a needle the bore is unknown → falls back to the previewable
        # bead × layer_height × mult (monotonic, non-zero).
        c = compile_to_trajectory(self._line(1.0), None, None)
        self.assertGreater(c.total_volume_uL, 0.0)


# ═══════════════════════════════════════════════════════════════════
# (E) Parametric trajectory honors PrintObject.extrusion_modifier
# ═══════════════════════════════════════════════════════════════════

class TestParametricModifierThreading(unittest.TestCase):
    def _circle(self, mod):
        obj = PrintObject(
            name="c", object_type="circle",
            params={"radius": 2.0, "num_points": 64},
            extrusion_modifier=mod)
        generate_object_trajectory(
            obj, NEEDLE_27G, {"P1": SYR_100},
            print_speed_mm_s=5.0, layer_height_mm=0.2, pump_id="P1")
        return obj

    def test_pump_column_scales_with_modifier(self):
        o1 = self._circle(1.0)
        o2 = self._circle(2.0)
        disp1 = float(o1.trajectory[:, 3].max())   # COL_P1
        disp2 = float(o2.trajectory[:, 3].max())
        self.assertGreater(disp1, 0.0)
        self.assertAlmostEqual(disp2 / disp1, 2.0, places=5)

    def test_total_volume_scales_with_modifier(self):
        o1 = self._circle(1.0)
        o35 = self._circle(3.5)
        self.assertAlmostEqual(o35.total_volume_uL / o1.total_volume_uL,
                               3.5, places=5)

    def test_one_x_is_canonical_bore_area(self):
        o1 = self._circle(1.0)
        expected = o1.total_length_mm * NEEDLE_27G.cross_section_area_mm2
        self.assertAlmostEqual(o1.total_volume_uL, expected, places=6)

    def test_default_object_is_unchanged_at_1x(self):
        # An object with no explicit modifier (default 1.0) matches the
        # canonical formula — i.e. legacy parametric prints are byte-identical.
        obj = PrintObject(name="c", object_type="circle",
                          params={"radius": 2.0, "num_points": 64})
        self.assertEqual(obj.extrusion_modifier, 1.0)
        generate_object_trajectory(
            obj, NEEDLE_27G, {"P1": SYR_100},
            print_speed_mm_s=5.0, layer_height_mm=0.2, pump_id="P1")
        expected = compute_total_volume(obj.total_length_mm, NEEDLE_27G)
        self.assertAlmostEqual(obj.total_volume_uL, expected, places=6)


if __name__ == "__main__":
    unittest.main()
