"""test_v75x_sketch_overlap_travel.py — Print Builder Sketch: optimizer
"retrace along an existing bead" (path flexibility).

When enabled, if the next shape's start is a brief distance back along the
already-printed bead (same ink, same print Z), the optimizer marks that
connection (``retrace_from``) and the compiler keeps the needle DOWN and
retraces there instead of lifting (a pen-up). During the retrace the pump can be
PAUSED (deposit ~nothing, never relieve pressure) and/or the traverse can be
FASTER. All OFF by default → byte-identical legacy behaviour.

The key correctness constraint is the bake→Quick-Print round trip: a paused
retrace must NOT be re-classified as a pen-up travel by Quick Print's
``_travel_mask`` (which would lift + relieve pressure).

Backend tests need no GUI; the round-trip guard imports the (offscreen) page.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest

import numpy as np

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory, count_discontinuities,
    optimize_print_order, _retrace_polyline,
)
from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec


def _needle():
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


def _syringe():
    return SyringeSpec(volume_uL=100, stroke_length_mm=30.0, barrel_id_mm=1.46)


def _retrace_scene(enabled=True, pause=True, factor=1.0, max_mm=5.0):
    """Line A (0,0)->(10,0) then line B starting 3 mm back on A, same ink."""
    sk = Sketch(shapes=[
        SketchShape(kind="line", points=[(0, 0), (10, 0)], ink_id=1),
        SketchShape(kind="line", points=[(7, 0), (7, 5)], ink_id=1),
    ], line_spacing_mm=0.4)
    sk.overlap_travel_enabled = enabled
    sk.overlap_travel_pause_pump = pause
    sk.overlap_travel_speed_factor = factor
    sk.overlap_travel_max_mm = max_mm
    return optimize_print_order(sk)


# ═══════════════════════════════════════════════════════════════════
# _retrace_polyline
# ═══════════════════════════════════════════════════════════════════

class TestRetracePolyline(unittest.TestCase):

    def setUp(self):
        self.path = np.array([[0, 0], [2, 0], [4, 0], [6, 0], [8, 0], [10, 0]],
                             float)

    def test_walks_backward_along_path(self):
        self.assertEqual(_retrace_polyline(self.path, (10, 0), (6, 0), 5.0),
                         [(8.0, 0.0), (6.0, 0.0)])

    def test_none_when_over_cap(self):
        self.assertIsNone(_retrace_polyline(self.path, (10, 0), (2, 0), 3.0))

    def test_none_when_same_vertex(self):
        self.assertIsNone(_retrace_polyline(self.path, (10, 0), (10, 0), 5.0))

    def test_forward_direction(self):
        self.assertEqual(_retrace_polyline(self.path, (0, 0), (4, 0), 5.0),
                         [(2.0, 0.0), (4.0, 0.0)])


# ═══════════════════════════════════════════════════════════════════
# Optimizer stamping
# ═══════════════════════════════════════════════════════════════════

class TestOptimizerStamps(unittest.TestCase):

    def test_stamps_when_within_cap_same_ink(self):
        opt = _retrace_scene(enabled=True, max_mm=5.0)
        self.assertEqual([s.retrace_from for s in opt.shapes],
                         [None, (10.0, 0.0)])

    def test_no_stamp_when_over_cap(self):
        opt = _retrace_scene(enabled=True, max_mm=1.0)   # 3 mm gap > 1 mm cap
        self.assertTrue(all(s.retrace_from is None for s in opt.shapes))

    def test_no_stamp_across_different_ink(self):
        sk = Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)], ink_id=1),
            SketchShape(kind="line", points=[(7, 0), (7, 5)], ink_id=2),
        ], line_spacing_mm=0.4)
        sk.overlap_travel_enabled = True
        opt = optimize_print_order(sk)
        self.assertTrue(all(s.retrace_from is None for s in opt.shapes))

    def test_disabled_clears_and_no_stamp(self):
        opt = _retrace_scene(enabled=False)
        self.assertTrue(all(s.retrace_from is None for s in opt.shapes))


# ═══════════════════════════════════════════════════════════════════
# Compiler emission
# ═══════════════════════════════════════════════════════════════════

class TestCompilerRetrace(unittest.TestCase):

    def test_feature_off_is_unchanged(self):
        # Enabling WITHOUT a stamped retrace_from must be byte-identical to the
        # legacy pen-up (nothing to retrace along yet).
        base = _retrace_scene(enabled=False)
        base.overlap_travel_enabled = True          # flip flag, no retrace marks
        for s in base.shapes:
            s.retrace_from = None
        off = _retrace_scene(enabled=False)
        a = compile_to_trajectory(base, _needle(), _syringe()).trajectory
        b = compile_to_trajectory(off, _needle(), _syringe()).trajectory
        self.assertTrue(np.allclose(a, b))
        self.assertEqual(count_discontinuities(off, _needle(), _syringe()), 1)

    def test_retrace_removes_the_lift(self):
        opt = _retrace_scene(enabled=True, pause=True)
        self.assertEqual(count_discontinuities(opt, _needle(), _syringe()), 0)

    def test_deposit_on_return_adds_length(self):
        dep = _retrace_scene(enabled=True, pause=False)
        pau = _retrace_scene(enabled=True, pause=True)
        rd = compile_to_trajectory(dep, _needle(), _syringe())
        rp = compile_to_trajectory(pau, _needle(), _syringe())
        # Depositing on the return lays a bead over the ~3 mm retrace.
        self.assertAlmostEqual(rd.total_length_mm - rp.total_length_mm, 3.0,
                               delta=0.3)

    def test_paused_volume_is_negligible(self):
        off = _retrace_scene(enabled=False)
        pau = _retrace_scene(enabled=True, pause=True)
        vo = compile_to_trajectory(off, _needle(), _syringe()).total_volume_uL
        vp = compile_to_trajectory(pau, _needle(), _syringe()).total_volume_uL
        # Pausing deposits ~nothing over the retrace: < 1 % of the print volume.
        self.assertLess(abs(vp - vo), 0.01 * max(vo, 1e-9))

    def test_faster_retrace_is_quicker(self):
        slow = compile_to_trajectory(_retrace_scene(factor=1.0),
                                     _needle(), _syringe()).total_time_s
        fast = compile_to_trajectory(_retrace_scene(factor=4.0),
                                     _needle(), _syringe()).total_time_s
        self.assertLess(fast, slow)


# ═══════════════════════════════════════════════════════════════════
# Bake → Quick-Print round-trip guard (the critical constraint)
# ═══════════════════════════════════════════════════════════════════

class TestTravelMaskRoundTrip(unittest.TestCase):

    def _split(self, arr):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage as QP)
        mask = QP._travel_mask(arr)
        pts = [(float(arr[i, 0]), float(arr[i, 1]))
               for i in range(arr.shape[0])]
        if mask is None:
            return [pts]
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

    def test_paused_retrace_not_split_as_travel(self):
        # A paused retrace at print Z must stay in ONE print run (A→retrace→B);
        # if it were flagged travel, Quick Print would lift + relieve pressure.
        arr = compile_to_trajectory(
            _retrace_scene(enabled=True, pause=True),
            _needle(), _syringe()).trajectory
        subs = self._split(arr)
        self.assertEqual(len(subs), 1)             # one continuous run
        # The single run spans from A's start to B's end.
        self.assertLess(abs(subs[0][0][0]) + abs(subs[0][0][1]), 0.5)   # ~(0,0)
        self.assertGreater(subs[0][-1][1], 4.0)                         # B end y

    def test_off_scene_splits_into_two(self):
        arr = compile_to_trajectory(_retrace_scene(enabled=False),
                                    _needle(), _syringe()).trajectory
        self.assertEqual(len(self._split(arr)), 2)  # pen-up between A and B


# ═══════════════════════════════════════════════════════════════════
# Serialization
# ═══════════════════════════════════════════════════════════════════

class TestSerialization(unittest.TestCase):

    def test_fields_omitted_when_off(self):
        d = Sketch(shapes=[SketchShape(kind="circle")]).to_dict()
        self.assertNotIn("overlap_travel_enabled", d)

    def test_fields_roundtrip_when_on(self):
        sk = Sketch(shapes=[SketchShape(kind="circle")])
        sk.overlap_travel_enabled = True
        sk.overlap_travel_max_mm = 7.5
        sk.overlap_travel_pause_pump = False
        sk.overlap_travel_speed_factor = 3.0
        sk2 = Sketch.from_dict(sk.to_dict())
        self.assertTrue(sk2.overlap_travel_enabled)
        self.assertEqual(sk2.overlap_travel_max_mm, 7.5)
        self.assertFalse(sk2.overlap_travel_pause_pump)
        self.assertEqual(sk2.overlap_travel_speed_factor, 3.0)

    def test_retrace_from_roundtrip(self):
        s = SketchShape(kind="line", points=[(0, 0), (5, 0)],
                        retrace_from=(3.0, 4.0))
        self.assertEqual(SketchShape.from_dict(s.to_dict()).retrace_from,
                         (3.0, 4.0))
        self.assertNotIn("retrace_from",
                         SketchShape(kind="line").to_dict())


if __name__ == "__main__":
    unittest.main()
