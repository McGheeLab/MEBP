"""
test_v75x_sketch_overlap_start_end_markers.py — closure overlap (distance /
needle-Ø) + draggable start/end markers for the Print Builder Sketch tool
(v7.5.x).

Covers:
- the overlap model (`overlap_mode` / `overlap_distance_mm`), serialization +
  legacy `overlap_closure` migration, and `overlap_amount_mm`,
- `trim_open_path` + the arc-length helpers (full-range = byte-identical
  no-op; start-only = legacy flip; both = sub-segment),
- the compiler: closure overlap extends a closed loop by the resolved amount;
  open-shape start/end trim the printed sub-segment,
- the canvas markers: end marker gated per shape kind, drag semantics
  (open → trim end; closed → distance overlap), reset.

See coding plans/Update plans/MEBP_v75x_SKETCH_OVERLAP_AND_START_END_MARKERS.md.
"""

import math
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np   # noqa: E402

from SupportClasses.SketchTrajectory import (   # noqa: E402
    Sketch, SketchShape, compile_to_trajectory, trim_open_path,
    _cumlen, _project_arclen, _point_at_arclen, _subpath_arclen,
)


class _Needle:
    od_mm = 0.8
    id_mm = 0.4
    cross_section_area_mm2 = math.pi * (0.2 ** 2)
    num_channels = 1


def _circle(**kw):
    return SketchShape(kind="circle", cx=0.0, cy=0.0, radius=5.0, **kw)


# ═══════════════════════════════════════════════════════════════════
# Model + serialization
# ═══════════════════════════════════════════════════════════════════

class TestOverlapModel(unittest.TestCase):

    def test_legacy_byte_identity(self):
        d = _circle().to_dict()
        for k in ("overlap_mode", "overlap_distance_mm", "end_point"):
            self.assertNotIn(k, d)

    def test_migrate_overlap_closure(self):
        rt = SketchShape.from_dict({"kind": "circle", "overlap_closure": True})
        self.assertEqual(rt.overlap_mode, "needle")
        self.assertNotIn("overlap_closure", rt.to_dict())

    def test_serialize_roundtrip(self):
        sh = _circle(overlap_mode="distance", overlap_distance_mm=1.25,
                     end_point=None)
        rt = SketchShape.from_dict(sh.to_dict())
        self.assertEqual(rt.overlap_mode, "distance")
        self.assertAlmostEqual(rt.overlap_distance_mm, 1.25)
        # distance emitted only in distance mode
        d = _circle(overlap_mode="needle").to_dict()
        self.assertEqual(d["overlap_mode"], "needle")
        self.assertNotIn("overlap_distance_mm", d)

    def test_end_point_emitted_only_when_set(self):
        self.assertNotIn("end_point", _circle().to_dict())
        d = SketchShape(kind="line", points=[(0, 0), (5, 0)],
                        end_point=(3.0, 0.0)).to_dict()
        self.assertEqual(d["end_point"], [3.0, 0.0])

    def test_overlap_amount_mm(self):
        self.assertEqual(_circle(overlap_mode="needle").overlap_amount_mm(0.8),
                         0.8)                                     # full Ø
        self.assertEqual(_circle(overlap_mode="needle").overlap_amount_mm(
            0.0, 0.4), 0.4)                                       # bead fallback
        self.assertEqual(_circle(overlap_mode="distance",
                                 overlap_distance_mm=2.0).overlap_amount_mm(0.8),
                         2.0)
        self.assertEqual(_circle().overlap_amount_mm(0.8), 0.0)


# ═══════════════════════════════════════════════════════════════════
# Arc-length helpers + trim_open_path
# ═══════════════════════════════════════════════════════════════════

class TestArcLength(unittest.TestCase):

    def setUp(self):
        self.line = np.array([(0.0, 0.0), (10.0, 0.0)])

    def test_full_range_is_noop(self):
        self.assertTrue(np.allclose(trim_open_path(self.line, None, None),
                                    self.line))
        self.assertTrue(np.allclose(trim_open_path(self.line, (0, 0), None),
                                    self.line))

    def test_start_only_reproduces_flip(self):
        self.assertTrue(np.allclose(trim_open_path(self.line, (10, 0), None),
                                    self.line[::-1]))

    def test_trim_both_ends(self):
        out = trim_open_path(self.line, (2, 0), (7, 0))
        self.assertTrue(np.allclose(out[0], [2, 0]))
        self.assertTrue(np.allclose(out[-1], [7, 0]))
        # arc length of the trimmed segment ≈ 5
        self.assertAlmostEqual(float(_cumlen(out)[-1]), 5.0, places=6)

    def test_trim_reversed_when_start_after_end(self):
        out = trim_open_path(self.line, (8, 0), (3, 0))
        self.assertTrue(np.allclose(out[0], [8, 0]))
        self.assertTrue(np.allclose(out[-1], [3, 0]))

    def test_polyline_arclen_helpers(self):
        poly = np.array([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)])
        cum = _cumlen(poly)
        self.assertAlmostEqual(float(cum[-1]), 7.0)
        self.assertAlmostEqual(_project_arclen(poly, cum, (3, 2)), 5.0, places=6)
        x, y = _point_at_arclen(poly, cum, 3.0)
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, 0.0)

    def test_subpath_interpolates_endpoints(self):
        poly = np.array([(0.0, 0.0), (10.0, 0.0)])
        cum = _cumlen(poly)
        sub = _subpath_arclen(poly, cum, 2.5, 6.0)
        self.assertTrue(np.allclose(sub[0], [2.5, 0]))
        self.assertTrue(np.allclose(sub[-1], [6.0, 0]))


# ═══════════════════════════════════════════════════════════════════
# Compiler
# ═══════════════════════════════════════════════════════════════════

class TestCompilerOverlapTrim(unittest.TestCase):

    def test_closure_overlap_extends_closed_loop(self):
        base = compile_to_trajectory(Sketch(shapes=[_circle()]), _Needle())
        needle_ov = compile_to_trajectory(
            Sketch(shapes=[_circle(overlap_mode="needle")]), _Needle())
        dist_ov = compile_to_trajectory(
            Sketch(shapes=[_circle(overlap_mode="distance",
                                   overlap_distance_mm=3.0)]), _Needle())
        self.assertAlmostEqual(needle_ov.total_length_mm - base.total_length_mm,
                               0.8, places=2)          # full needle Ø
        self.assertAlmostEqual(dist_ov.total_length_mm - base.total_length_mm,
                               3.0, places=2)

    def test_overlap_ignored_on_open_and_filled(self):
        base = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="line",
                                       points=[(0, 0), (10, 0)])]), _Needle())
        ov = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (10, 0)],
                                       overlap_mode="needle")]), _Needle())
        self.assertAlmostEqual(base.total_length_mm, ov.total_length_mm,
                               places=6)

    def test_open_trim_shortens_path(self):
        full = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="line",
                                       points=[(0, 0), (10, 0)])]))
        trim = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (10, 0)],
                                       start_point=(2, 0), end_point=(7, 0))]))
        self.assertAlmostEqual(full.total_length_mm, 10.0, delta=0.3)
        self.assertAlmostEqual(trim.total_length_mm, 5.0, delta=0.3)

    def test_open_start_only_is_legacy_flip(self):
        """start_point at an endpoint + no end → full path (byte-identical)."""
        a = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (10, 0)])]))
        b = compile_to_trajectory(
            Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (10, 0)],
                                       start_point=(0, 0))]))
        self.assertAlmostEqual(a.total_length_mm, b.total_length_mm, places=6)


# ═══════════════════════════════════════════════════════════════════
# Canvas markers
# ═══════════════════════════════════════════════════════════════════

def _canvas(shapes, needle_od=0.8):
    from PySide6.QtWidgets import QApplication
    global _APP
    _APP = QApplication.instance() or QApplication(sys.argv)
    from gui.widgets.sketch_canvas import SketchCanvas
    c = SketchCanvas()
    c.resize(600, 600)
    c.set_needle_od(needle_od)
    c.set_sketch(Sketch(shapes=shapes))
    c.fit_view()
    return c


class TestCanvasMarkers(unittest.TestCase):

    def test_end_marker_gating(self):
        circ = _circle()
        c = _canvas([circ])
        c.set_selected(0)
        self.assertFalse(c._shape_supports_end(circ))    # closed, overlap off
        circ.overlap_mode = "needle"
        self.assertTrue(c._shape_supports_end(circ))     # closed, overlap on
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        c2 = _canvas([ln])
        self.assertTrue(c2._shape_supports_end(ln))      # open → always

    def test_closed_end_drag_sets_distance(self):
        circ = _circle(overlap_mode="needle")
        c = _canvas([circ])
        c.set_selected(0)
        from PySide6.QtCore import QPointF
        arr, cum = c._closed_ring(circ)
        x, y = _point_at_arclen(arr, cum, 2.5)
        c._set_end_from_drag(circ, QPointF(x, y))
        self.assertEqual(circ.overlap_mode, "distance")
        self.assertAlmostEqual(circ.overlap_distance_mm, 2.5, delta=0.2)

    def test_closed_end_marker_tracks_overlap(self):
        circ = _circle(overlap_mode="distance", overlap_distance_mm=0.0)
        c = _canvas([circ])
        c.set_selected(0)
        # zero overlap → marker at the seam (5,0)
        ew0 = c._effective_end_world(circ)
        self.assertAlmostEqual(ew0.x(), 5.0, delta=1e-3)
        circ.overlap_distance_mm = 3.0
        ew1 = c._effective_end_world(circ)
        self.assertGreater(abs(ew1.y()), 0.5)            # moved along the ring

    def test_open_end_drag_sets_end_point(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        c = _canvas([ln])
        c.set_selected(0)
        from PySide6.QtCore import QPointF
        c._set_end_from_drag(ln, QPointF(7, 0))
        self.assertEqual(ln.end_point, (7.0, 0.0))

    def test_open_end_default_is_far_endpoint(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        c = _canvas([ln])
        c.set_selected(0)
        self.assertAlmostEqual(c._effective_end_world(ln).x(), 10.0, delta=1e-3)

    def test_start_marker_trims_mid_line(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)])
        c = _canvas([ln])
        c.set_selected(0)
        from PySide6.QtCore import QPointF
        c._selected = 0
        c._resize_kind = "start"
        c._apply_resize(QPointF(3, 0))
        self.assertEqual(ln.start_point, (3.0, 0.0))

    def test_clear_end_point(self):
        ln = SketchShape(kind="line", points=[(0, 0), (10, 0)],
                         end_point=(7.0, 0.0))
        c = _canvas([ln])
        c.clear_end_point(0)
        self.assertIsNone(ln.end_point)

    def test_marker_paint_smoke(self):
        from PySide6.QtGui import QPixmap
        c = _canvas([_circle(overlap_mode="needle"),
                     SketchShape(kind="line", points=[(0, 0), (8, 4)],
                                 end_point=(6, 3))])
        for i in range(2):
            c.set_selected(i)
            pm = QPixmap(400, 400)
            c.render(pm)                                 # must not raise


# ═══════════════════════════════════════════════════════════════════
# Page card
# ═══════════════════════════════════════════════════════════════════

class TestPageCard(unittest.TestCase):

    def _page(self):
        from PySide6.QtWidgets import QApplication
        global _APP
        _APP = QApplication.instance() or QApplication(sys.argv)
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_overlap_combo_changes_mode_and_seeds_distance(self):
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[_circle()]))
        page._canvas.set_selected(0)          # builds the shape card
        sh = page._canvas.sketch().shapes[0]
        # simulate choosing "Custom distance"
        spin = page._dspin(0.0, 0.0, 100.0, 0.1)
        page._on_overlap_mode_changed(sh, "distance", spin)
        self.assertEqual(sh.overlap_mode, "distance")
        self.assertGreater(sh.overlap_distance_mm, 0.0)   # seeded visible default

    def test_end_row_only_for_open_shapes(self):
        from PySide6.QtWidgets import QPushButton
        page = self._page()
        page._canvas.set_sketch(Sketch(shapes=[
            SketchShape(kind="line", points=[(0, 0), (10, 0)])]))
        page._canvas.set_selected(0)
        self.assertTrue(any("Reset end point" in b.text()
                            for b in page._props_host.findChildren(QPushButton)))
        page._canvas.set_sketch(Sketch(shapes=[_circle()]))
        page._canvas.set_selected(0)
        self.assertFalse(any("Reset end point" in b.text()
                             for b in page._props_host.findChildren(QPushButton)))


if __name__ == "__main__":
    unittest.main()
