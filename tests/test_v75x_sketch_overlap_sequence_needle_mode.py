"""test_v75x_sketch_overlap_sequence_needle_mode.py — Print Builder Sketch:
closed-loop over-closure toggle, single/multi-needle mode, and the print
sequence (color-coded sections) panel.

Three operator-requested additions:
  1. ``SketchShape.overlap_closure`` — continue a closed loop past its seam by
     ~the needle radius so it fully closes (the needle pushes ink aside on
     re-entry).
  2. single-needle vs multi-needle mode (``Sketch.single_needle``, auto-detected
     from ``needle.num_channels``): in single mode a channel change breaks the
     bead (ink replacement); multi mode welds across channels.
  3. ``plan_print_sections`` — the ordered, sectioned plan the right-panel
     sequence view renders.

Backend tests need no GUI; canvas/page tests run offscreen.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import math
import sys
import unittest

import numpy as np

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory, count_discontinuities,
    optimize_print_order, plan_print_sections, extend_closed_path,
)
from SupportClasses.GeometryEngine import generate_circle
from SupportClasses.PhysicalModels import NeedleSpec


def _needle(num_channels=1):
    n = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)  # od 0.718
    n.num_channels = num_channels
    return n


def _line(x1, y1, x2, y2, pump=0):
    # ``pump`` is a 0-based abstract-ink index; map to a 1-based ink id
    # (P1→ink 1, P2→ink 2, …) to match the model's migration convention.
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)],
                       ink_id=pump + 1)


def _len(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


# ═══════════════════════════════════════════════════════════════════
# Over-closure
# ═══════════════════════════════════════════════════════════════════

class TestExtendClosedPath(unittest.TestCase):

    def test_extends_closed_loop_by_overshoot(self):
        c = generate_circle(0, 0, 5, 64)
        ext = extend_closed_path(c, 0.5)
        self.assertAlmostEqual(_len(ext.tolist()) - _len(c.tolist()), 0.5,
                               places=3)

    def test_open_path_unchanged(self):
        p = np.array([[0.0, 0.0], [10.0, 0.0]])   # not a closed loop
        self.assertTrue(np.allclose(extend_closed_path(p, 0.5), p))

    def test_zero_overshoot_unchanged(self):
        c = generate_circle(0, 0, 5, 32)
        self.assertTrue(np.allclose(extend_closed_path(c, 0.0), c))

    def test_never_walks_more_than_one_lap(self):
        c = generate_circle(0, 0, 1, 16)          # perimeter ~6.28
        ext = extend_closed_path(c, 1000.0)       # huge overshoot
        # capped at one lap → added length < original perimeter
        self.assertLess(_len(ext.tolist()) - _len(c.tolist()),
                        _len(c.tolist()) + 1e-6)


class TestOverlapClosureCompile(unittest.TestCase):

    def _sk(self, shapes):
        return Sketch(shapes=shapes, line_spacing_mm=0.4)

    def test_overlap_needle_adds_full_diameter_of_path(self):
        # v7.5.x: "needle" overlap now overshoots one full needle OUTER Ø
        # (the operator asked for "needle diameter"; the retired overlap_closure
        # bool used the radius).
        n = _needle()
        base = compile_to_trajectory(
            self._sk([SketchShape(kind="circle", cx=0, cy=0, radius=5)]), n)
        ov = compile_to_trajectory(
            self._sk([SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                  overlap_mode="needle")]), n)
        self.assertAlmostEqual(ov.total_length_mm - base.total_length_mm,
                               n.od_mm, places=2)

    def test_overlap_needle_falls_back_to_one_bead_without_needle(self):
        base = compile_to_trajectory(
            self._sk([SketchShape(kind="rect", cx=0, cy=0,
                                  width=10, height=10)]), None)
        ov = compile_to_trajectory(
            self._sk([SketchShape(kind="rect", cx=0, cy=0, width=10, height=10,
                                  overlap_mode="needle")]), None)
        self.assertAlmostEqual(ov.total_length_mm - base.total_length_mm,
                               0.4, places=2)

    def test_overlap_distance_adds_typed_amount(self):
        base = compile_to_trajectory(
            self._sk([SketchShape(kind="circle", cx=0, cy=0, radius=5)]), None)
        ov = compile_to_trajectory(
            self._sk([SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                  overlap_mode="distance",
                                  overlap_distance_mm=2.5)]), None)
        self.assertAlmostEqual(ov.total_length_mm - base.total_length_mm,
                               2.5, places=2)

    def test_overlap_noop_on_open_line(self):
        a = compile_to_trajectory(self._sk([_line(0, 0, 10, 0)]), _needle())
        b = compile_to_trajectory(
            self._sk([SketchShape(kind="line", points=[(0, 0), (10, 0)],
                                  overlap_mode="needle")]), _needle())
        self.assertAlmostEqual(a.total_length_mm, b.total_length_mm, places=6)

    def test_overlap_noop_on_filled(self):
        a = compile_to_trajectory(
            self._sk([SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                  filled=True)]), _needle())
        b = compile_to_trajectory(
            self._sk([SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                  filled=True, overlap_mode="needle")]),
            _needle())
        self.assertAlmostEqual(a.total_length_mm, b.total_length_mm, places=6)


class TestOverlapSerialize(unittest.TestCase):

    def test_roundtrip_and_omitted_when_none(self):
        on = SketchShape(kind="circle", overlap_mode="distance",
                         overlap_distance_mm=1.5)
        rt = SketchShape.from_dict(on.to_dict())
        self.assertEqual(rt.overlap_mode, "distance")
        self.assertAlmostEqual(rt.overlap_distance_mm, 1.5)
        self.assertNotIn("overlap_mode", SketchShape(kind="circle").to_dict())

    def test_legacy_overlap_closure_migrates_to_needle(self):
        rt = SketchShape.from_dict({"kind": "circle", "overlap_closure": True})
        self.assertEqual(rt.overlap_mode, "needle")
        self.assertNotIn("overlap_closure", rt.to_dict())


# ═══════════════════════════════════════════════════════════════════
# single / multi needle mode
# ═══════════════════════════════════════════════════════════════════

class TestNeedleMode(unittest.TestCase):

    def test_is_single_needle_resolution(self):
        self.assertTrue(Sketch().is_single_needle(_needle(1)))
        self.assertFalse(Sketch().is_single_needle(_needle(2)))
        self.assertFalse(Sketch().is_single_needle(None))
        self.assertTrue(Sketch(single_needle=True).is_single_needle(_needle(2)))
        self.assertFalse(Sketch(single_needle=False).is_single_needle(_needle(1)))

    def test_serialize_single_needle(self):
        self.assertNotIn("single_needle", Sketch().to_dict())
        self.assertEqual(Sketch.from_dict(Sketch().to_dict()).single_needle,
                         None)
        self.assertTrue(
            Sketch.from_dict(Sketch(single_needle=True).to_dict()).single_needle)
        self.assertIs(
            Sketch.from_dict(Sketch(single_needle=False).to_dict()).single_needle,
            False)

    def test_single_mode_breaks_weld_on_channel_change(self):
        # Two connected lines on different pumps.
        sk = Sketch(shapes=[_line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 1)],
                    line_spacing_mm=0.4)
        self.assertEqual(count_discontinuities(sk, _needle(1)), 1)   # single
        self.assertEqual(count_discontinuities(sk, None), 0)         # multi
        self.assertEqual(count_discontinuities(sk, _needle(2)), 0)   # multi

    def test_single_mode_same_pump_still_welds(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 0)],
                    line_spacing_mm=0.4)
        self.assertEqual(count_discontinuities(sk, _needle(1)), 0)

    def test_explicit_multi_overrides_single_channel_needle(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 1)],
                    line_spacing_mm=0.4, single_needle=False)
        self.assertEqual(count_discontinuities(sk, _needle(1)), 0)


# ═══════════════════════════════════════════════════════════════════
# plan_print_sections
# ═══════════════════════════════════════════════════════════════════

class TestPlanPrintSections(unittest.TestCase):

    def _sk(self, shapes, **kw):
        return Sketch(shapes=shapes, line_spacing_mm=0.4, **kw)

    def test_multi_connected_is_one_section(self):
        sk = self._sk([_line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 1),
                       _line(10, 10, 0, 10, 0)])
        plan = plan_print_sections(sk, None)          # multi
        secs = [p for p in plan if p["type"] == "section"]
        self.assertEqual(len(secs), 1)
        self.assertEqual(secs[0]["shape_indices"], [0, 1, 2])

    def test_single_channel_change_splits_with_ink_change(self):
        sk = self._sk([_line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 1),
                       _line(10, 10, 0, 10, 0)])
        plan = plan_print_sections(sk, _needle(1))    # single
        secs = [p for p in plan if p["type"] == "section"]
        self.assertEqual(len(secs), 3)
        self.assertEqual(secs[1]["break_before"], "ink_change")
        self.assertEqual(secs[2]["break_before"], "ink_change")

    def test_travel_is_a_move_item_between_sections(self):
        sk = self._sk([_line(0, 0, 10, 0), SketchShape(kind="travel", cx=9, cy=9),
                       _line(50, 50, 60, 60)])
        plan = plan_print_sections(sk, None)
        self.assertEqual([p["type"] for p in plan],
                         ["section", "move", "section"])
        self.assertEqual(plan[1]["reason"], "travel")

    def test_disconnected_same_pump_is_a_move_break(self):
        sk = self._sk([_line(0, 0, 10, 0), _line(50, 50, 60, 60)])
        plan = plan_print_sections(sk, None)
        secs = [p for p in plan if p["type"] == "section"]
        self.assertEqual(len(secs), 2)
        self.assertEqual(secs[1]["break_before"], "move")

    def test_length_and_indices(self):
        sk = self._sk([_line(0, 0, 10, 0)])
        sec = plan_print_sections(sk, None)[0]
        self.assertEqual(sec["shape_indices"], [0])
        self.assertAlmostEqual(sec["length_mm"], 10.0, delta=0.2)


class TestReviewFixes(unittest.TestCase):
    """Regression tests for the adversarial-review findings."""

    def test_no_print_pass_does_not_block_single_mode_weld(self):
        # Finding 1: a move-only (no_print) pass must not change the last-printed
        # channel nor require an ink swap. L1(P1 print) → L2(P2 move-only) →
        # L3(P1 print), all connected → one continuous run in single mode.
        sk = Sketch(shapes=[
            _line(0, 0, 10, 0, 0),
            SketchShape(kind="line", points=[(10, 0), (20, 0)],
                        ink_id=2, no_print=True),
            _line(20, 0, 30, 0, 0)], line_spacing_mm=0.4)
        self.assertEqual(count_discontinuities(sk, _needle(1)), 0)
        secs = [p for p in plan_print_sections(sk, _needle(1))
                if p["type"] == "section"]
        self.assertEqual(len(secs), 1)

    def test_plan_uses_last_pass_end_for_thick_outline(self):
        # Finding 3: a thick outline's exit is its LAST (outer) pass end, so a
        # shape at the outer edge welds in the plan (not the inner-pass end).
        thick = SketchShape(kind="circle", cx=0, cy=0, radius=5.0,
                            line_width_mm=1.2)   # passes at r=4.6/5.0/5.4
        at_outer = _line(5.4, 0.0, 5.4, 5.0, 0)
        secs = [p for p in plan_print_sections(
            Sketch(shapes=[thick, at_outer], line_spacing_mm=0.4), None)
            if p["type"] == "section"]
        self.assertEqual(len(secs), 1)          # line welds to the outer pass

    def test_plan_mirrors_overlap_extended_exit(self):
        # Finding 2: a shape sitting at a closed loop's over-closure end welds in
        # the plan only when the closure overlap is on (the exit extends past
        # the seam). Use a 0.36 mm distance overlap so the exit lands exactly at
        # ``pt`` (independent of the needle Ø).
        pt = _line(4.987, 0.359, 4.987, 5.0, 0)  # ~0.36 mm arc past the seam
        ov = Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=5,
                                        overlap_mode="distance",
                                        overlap_distance_mm=0.36), pt],
                    line_spacing_mm=0.4)
        no = Sketch(shapes=[SketchShape(kind="circle", cx=0, cy=0, radius=5),
                            pt], line_spacing_mm=0.4)
        n = _needle(1)
        n_ov = len([p for p in plan_print_sections(ov, n) if p["type"] == "section"])
        n_no = len([p for p in plan_print_sections(no, n) if p["type"] == "section"])
        self.assertEqual(n_ov, 1)               # welds via extended exit
        self.assertEqual(n_no, 2)               # doesn't reach the seam


class TestOptimizerPumpAware(unittest.TestCase):

    def test_single_mode_groups_by_pump(self):
        # Interleaved pumps, far apart → single mode should group same-pump.
        sk = Sketch(shapes=[_line(0, 0, 1, 0, 0), _line(20, 0, 21, 0, 1),
                            _line(2, 0, 3, 0, 0), _line(22, 0, 23, 0, 1)],
                    line_spacing_mm=0.4)
        opt = optimize_print_order(sk, needle=_needle(1))
        inks = [s.ink_id for s in opt.shapes]
        # same-ink shapes are adjacent (1,1,2,2 or 2,2,1,1)
        self.assertIn(inks, ([1, 1, 2, 2], [2, 2, 1, 1]))


# ═══════════════════════════════════════════════════════════════════
# Page (offscreen)
# ═══════════════════════════════════════════════════════════════════

class _Ink:
    def __init__(self, name, color):
        self.name = name
        self.color = color


class _Pump:
    def __init__(self, inks, syringe=None):
        self.inks = inks
        self.syringe = syringe


class _Cfg:
    def __init__(self, num_channels=1, syringe=None):
        self.needle = _needle(num_channels)
        self.pumps = {
            "P1": _Pump([_Ink("trypsin", "#89b4fa")], syringe),
            "P2": _Pump([_Ink("collagen", "#a6e3a1")]),
            "P3": _Pump([]),
        }
        self.active_plate_key = 96


class TestPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, cfg=None):
        from gui.pages.print_builder_sketch import SketchPage
        p = SketchPage()
        if cfg is not None:
            p.set_hardware_config(cfg)
        return p

    def test_channel_info_from_pumps_dict(self):
        p = self._page(_Cfg())
        self.assertEqual(p._channel_info[0], ("trypsin", "#89b4fa"))
        self.assertEqual(p._channel_info[1], ("collagen", "#a6e3a1"))
        self.assertEqual(p._channel_info[2], (None, None))

    def test_single_needle_checkbox_autodetects(self):
        self.assertTrue(self._page(_Cfg(num_channels=1))._single_needle_chk.isChecked())
        self.assertFalse(self._page(_Cfg(num_channels=2))._single_needle_chk.isChecked())

    def test_toggle_sets_explicit_mode(self):
        p = self._page(_Cfg(num_channels=2))
        p._single_needle_chk.setChecked(True)      # user forces single
        self.assertIs(p._canvas.sketch().single_needle, True)

    def test_sequence_rows_single_vs_multi(self):
        from PySide6.QtWidgets import QPushButton
        p = self._page(_Cfg(num_channels=1))
        p._canvas.set_sketch(Sketch(shapes=[
            _line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 1),
            _line(10, 10, 0, 10, 0)], line_spacing_mm=0.4))
        p._refresh_sequence()
        self.assertEqual(len(p._seq_host.findChildren(QPushButton)), 3)  # single
        p._single_needle_chk.setChecked(False)     # multi
        p._refresh_sequence()
        self.assertEqual(len(p._seq_host.findChildren(QPushButton)), 1)

    def test_section_click_selects_shapes(self):
        from PySide6.QtWidgets import QPushButton
        p = self._page(_Cfg(num_channels=1))
        p._canvas.set_sketch(Sketch(shapes=[
            _line(0, 0, 10, 0, 0), _line(10, 0, 10, 10, 1)],
            line_spacing_mm=0.4))
        p._refresh_sequence()
        rows = p._seq_host.findChildren(QPushButton)
        rows[1].click()                            # 2nd section (shape 1)
        self.assertEqual(p._canvas.selected_indices(), [1])

    def test_closure_overlap_combo_only_for_closed_loops(self):
        from PySide6.QtWidgets import QComboBox

        def has_overlap_combo(page):
            return any(cb.findData("needle") >= 0
                       for cb in page._props_host.findChildren(QComboBox))
        p = self._page(_Cfg())
        # Circle → closure-overlap combo present.
        p._canvas.set_sketch(Sketch(shapes=[SketchShape(kind="circle",
                                                        cx=0, cy=0, radius=5)]))
        p._canvas.set_selected(0)
        self.assertTrue(has_overlap_combo(p))
        # Line → no closure-overlap combo (open shape).
        p._canvas.set_sketch(Sketch(shapes=[_line(0, 0, 10, 0)]))
        p._canvas.set_selected(0)
        self.assertFalse(has_overlap_combo(p))

    def test_shape_is_closed_loop(self):
        from gui.pages.print_builder_sketch import SketchPage
        self.assertTrue(SketchPage._shape_is_closed_loop(
            SketchShape(kind="circle")))
        self.assertFalse(SketchPage._shape_is_closed_loop(
            SketchShape(kind="circle", filled=True)))
        self.assertFalse(SketchPage._shape_is_closed_loop(_line(0, 0, 1, 1)))
        self.assertTrue(SketchPage._shape_is_closed_loop(
            SketchShape(kind="polygon", points=[(0, 0), (1, 0), (1, 1)])))
        self.assertFalse(SketchPage._shape_is_closed_loop(
            SketchShape(kind="polygon", points=[(0, 0), (1, 0)])))

    def test_syringe_found_from_pumps_dict(self):
        # A pump dict with a syringe on P1 — the page must find it (iterating
        # dict VALUES, not keys).
        class Syr:
            mm_per_uL = 0.5
        p = self._page(_Cfg(syringe=Syr()))
        self.assertIsNotNone(p._syringe)


if __name__ == "__main__":
    unittest.main()
