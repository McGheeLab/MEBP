"""
v7.12 — custom parametric plates must render (and calibrate) correctly.

Background
----------
Every custom plate is built through ``WellPlate.from_wells``, which stores
``well_spacing_x/y = 0.0``, ``well_diameter = 0.0`` and synthesises
``rows``/``cols`` as a PSEUDO-grid: several wells can share one ``(row, col)``
cell. On the operator's 12-well plate (a 2x3 grid of 28 mm inserts plus two
3-well rings of 5.5 mm bores) that is ``rows=3, cols=5`` with C1/C2/C3 all at
``(1, 1)`` and D1/D2/D3 all at ``(1, 3)``.

Consumers that laid wells out on that pseudo-grid drew each 3-well ring as a
single circle and sized every well the same; consumers that derived a radius or
a search window from the plate-level ``well_diameter`` got 0.0; and the
calibration page synthesised well NAMES from ``rows``/``cols``, asking for
"A5" / "C5" — wells that do not exist, so ``get_well_position`` raised
``KeyError`` and 3-well auto-calibration could not run.

These tests pin the fix from both ends: the custom plate must come out right,
and every standard plate must come out exactly as it always did.
"""

from __future__ import annotations

import ast
import inspect
import math
import os
import textwrap
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QPoint

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PlateDocument import PlateDocument, NamingSpec, WellStyle
from SupportClasses.PlateDocumentStore import (
    plate_footprint_extent_mm, _doc_footprint_extent_mm,
)
from gui.widgets.plate_layout import PlateTransform, fit_wells, wells_from_plate


_app = QApplication.instance() or QApplication([])


# ── Fixture ───────────────────────────────────────────────────────

GRID_DIAMETER_MM = 28.0
RING_DIAMETER_MM = 5.5
RING_COUNT = 3
N_GRID_WELLS = 6
N_WELLS = N_GRID_WELLS + 2 * RING_COUNT


def make_insert_plate_doc() -> PlateDocument:
    """The operator's layout, rebuilt from scratch.

    A 2x3 grid of 28 mm inserts at 40 mm pitch plus two 3-well rings of 5.5 mm
    bores tucked between them — the shape that exposed every bug here. Built in
    code rather than read from ``config/`` so the suite does not depend on one
    machine's files.
    """
    doc = PlateDocument.new_plate("Test 6 insert with rosette")
    doc.add_grid(
        7.388, 9.894, rows=2, cols=3, pitch_x_mm=40.0, pitch_y_mm=40.0,
        style=WellStyle(diameter_mm=GRID_DIAMETER_MM, well_depth_mm=9.5),
        name="G1")
    for cx, prefix in ((26.35, "C"), (67.38, "D")):
        doc.add_ring(
            cx, 29.07, count=RING_COUNT, ring_diameter_mm=8.0,
            style=WellStyle(diameter_mm=RING_DIAMETER_MM, well_depth_mm=17.4,
                            rim_height_mm=11.82),
            naming=NamingSpec(scheme="numbers", prefix=prefix))
    return doc


def make_insert_plate() -> WellPlate:
    return make_insert_plate_doc().compile()


class PlateFixtureMixin:
    @classmethod
    def setUpClass(cls):
        cls.custom = make_insert_plate()
        cls.std24 = WellPlate.from_format(24)
        cls.std96 = WellPlate.from_format(96)


# ── The premise ───────────────────────────────────────────────────

class TestTheFixtureReproducesTheDefect(PlateFixtureMixin, unittest.TestCase):
    """Guard the guard: if the fixture stopped being a pseudo-grid plate, every
    test below would pass vacuously against an ordinary grid."""

    def test_plate_level_aggregates_are_meaningless(self):
        self.assertEqual(self.custom.well_diameter, 0.0)
        self.assertEqual(self.custom.well_spacing_x, 0.0)
        self.assertEqual(self.custom.well_spacing_y, 0.0)

    def test_several_wells_share_one_pseudo_grid_cell(self):
        cells: dict[tuple[int, int], list[str]] = {}
        for well in self.custom.get_all_wells():
            cells.setdefault((well.row, well.col), []).append(well.name)
        collisions = {c: n for c, n in cells.items() if len(n) > 1}
        self.assertTrue(
            collisions,
            "fixture no longer collides in the pseudo-grid — the bug it "
            "reproduces cannot be observed through it")
        self.assertTrue(any(len(n) == RING_COUNT for n in collisions.values()))

    def test_wells_have_mixed_diameters(self):
        diameters = {w.diameter for w in self.custom.get_all_wells()}
        self.assertEqual(diameters, {GRID_DIAMETER_MM, RING_DIAMETER_MM})

    def test_well_count(self):
        self.assertEqual(len(self.custom.well_names), N_WELLS)


# ── WellPlate helpers ─────────────────────────────────────────────

class TestWellDiameterResolution(PlateFixtureMixin, unittest.TestCase):

    def test_per_well_diameter_wins(self):
        self.assertAlmostEqual(
            self.custom.well_diameter_of("A1"), GRID_DIAMETER_MM)
        self.assertAlmostEqual(
            self.custom.well_diameter_of("C1"), RING_DIAMETER_MM)

    def test_representative_is_the_largest_when_plate_level_is_zero(self):
        self.assertAlmostEqual(
            self.custom.representative_well_diameter, GRID_DIAMETER_MM)

    def test_unknown_and_none_fall_back_rather_than_raise(self):
        self.assertAlmostEqual(
            self.custom.well_diameter_of("ZZ99"), GRID_DIAMETER_MM)
        self.assertAlmostEqual(
            self.custom.well_diameter_of(None), GRID_DIAMETER_MM)

    def test_case_insensitive(self):
        self.assertAlmostEqual(self.custom.well_diameter_of("c2"),
                               RING_DIAMETER_MM)

    def test_standard_plates_unchanged(self):
        for plate in (self.std24, self.std96):
            self.assertAlmostEqual(plate.representative_well_diameter,
                                   plate.well_diameter)
            for name in ("A1", plate.well_names[-1]):
                self.assertAlmostEqual(plate.well_diameter_of(name),
                                       plate.well_diameter)

    def test_never_zero_for_a_plate_with_wells(self):
        for plate in (self.custom, self.std24, self.std96):
            self.assertGreater(plate.representative_well_diameter, 0.0)


class TestWellPitch(PlateFixtureMixin, unittest.TestCase):

    def test_standard_plate_reports_its_declared_spacing(self):
        self.assertAlmostEqual(self.std24.nearest_neighbour_pitch_mm(),
                               min(self.std24.well_spacing_x,
                                   self.std24.well_spacing_y))

    def test_custom_plate_reports_a_usable_pitch(self):
        pitch = self.custom.nearest_neighbour_pitch_mm()
        self.assertGreater(pitch, 0.0)
        # Between the ring's own spacing and the grid's 40 mm pitch.
        self.assertLess(pitch, 40.0)

    def test_single_well_plate_reports_zero_rather_than_raising(self):
        one = WellPlate.from_wells(
            "solo", [self.custom.get_well_info("A1")])
        self.assertEqual(one.nearest_neighbour_pitch_mm(), 0.0)


class TestCalibrationTriangle(PlateFixtureMixin, unittest.TestCase):

    def test_standard_plates_keep_their_historical_triangles(self):
        # The exact wells the rows/cols name arithmetic produced.
        self.assertEqual(self.std24.calibration_triangle(),
                         ["A1", "A6", "D6"])
        self.assertEqual(self.std96.calibration_triangle(),
                         ["A1", "A12", "H12"])

    def test_custom_plate_triangle_wells_all_exist(self):
        tri = self.custom.calibration_triangle()
        self.assertEqual(len(tri), 3)
        for name in tri:
            self.assertIn(name, self.custom.well_names)

    def test_custom_plate_triangle_never_raises_on_lookup(self):
        for name in self.custom.calibration_triangle():
            self.custom.get_well_position(name)   # must not raise KeyError

    def test_triangle_spans_the_plate(self):
        tri = self.custom.calibration_triangle()
        pts = [self.custom.get_well_position(n) for n in tri]
        area = abs((pts[1][0] - pts[0][0]) * (pts[2][1] - pts[0][1])
                   - (pts[2][0] - pts[0][0]) * (pts[1][1] - pts[0][1])) / 2.0
        self.assertGreater(area, 100.0, "degenerate teaching triangle")

    def test_extreme_well_picks_the_four_corners(self):
        self.assertEqual(self.custom.extreme_well(-1, -1), "A1")
        self.assertEqual(self.custom.extreme_well(1, -1), "A3")
        self.assertEqual(self.custom.extreme_well(1, 1), "B3")
        self.assertEqual(self.custom.extreme_well(-1, 1), "B1")

    def test_single_well_plate_degrades_gracefully(self):
        one = WellPlate.from_wells("solo", [self.custom.get_well_info("A1")])
        self.assertEqual(one.calibration_triangle(), ["A1"])


# ── Footprint ─────────────────────────────────────────────────────

class TestPlateFootprint(PlateFixtureMixin, unittest.TestCase):

    def _margins(self, plate, footprint):
        wells = plate.get_all_wells()
        x0 = min(w.x - w.diameter / 2 for w in wells)
        x1 = max(w.x + w.diameter / 2 for w in wells)
        y0 = min(w.y - w.diameter / 2 for w in wells)
        y1 = max(w.y + w.diameter / 2 for w in wells)
        return (x0 - footprint[0], footprint[2] - x1,
                y0 - footprint[1], footprint[3] - y1)

    def test_standard_format_footprint(self):
        self.assertIsNotNone(plate_footprint_extent_mm(24))
        self.assertEqual(plate_footprint_extent_mm(24),
                         plate_footprint_extent_mm("24"))

    def test_wells_sit_inside_the_footprint_with_positive_margins(self):
        for key, plate in ((24, self.std24), (96, self.std96)):
            footprint = plate_footprint_extent_mm(key)
            for margin in self._margins(plate, footprint):
                self.assertGreater(
                    margin, 0.0,
                    f"{key}-well plate overflows its own footprint")

    def test_footprint_is_in_the_A1_WELL_frame_not_the_A1_datum_frame(self):
        """The frame distinction that the first cut of this fix got wrong.

        ``PlateBoundary.extent_a1`` is measured from the document's A1
        *reference datum*; every ``WellPlate`` coordinate is measured from the
        A1 *well*. On a parametric plate the author places the first pattern
        wherever they like, so the two differ — here by the grid seed
        (7.388, 9.894) — and handing a renderer the raw extent shifts the
        outline off the wells by exactly that much.
        """
        doc = make_insert_plate_doc()
        raw = doc.boundary.extent_a1()
        shifted = _doc_footprint_extent_mm(doc)
        self.assertNotEqual(
            tuple(round(v, 3) for v in raw),
            tuple(round(v, 3) for v in shifted),
            "footprint was not translated into the A1-well frame")

        seed_x, seed_y = 7.388, 9.894
        self.assertAlmostEqual(shifted[0], raw[0] - seed_x, places=3)
        self.assertAlmostEqual(shifted[1], raw[1] - seed_y, places=3)

        plate = doc.compile()
        for margin in self._margins(plate, shifted):
            self.assertGreater(margin, 0.0)

    def test_standard_plate_needs_no_shift(self):
        """A1 well IS the A1 datum on a standard plate — the shift is a no-op
        there, which is why this bug was invisible on every stock format."""
        doc = PlateDocument.from_standard_format(24)
        self.assertEqual(
            tuple(round(v, 6) for v in doc.boundary.extent_a1()),
            tuple(round(v, 6) for v in _doc_footprint_extent_mm(doc)))

    def test_unresolvable_keys_return_none_rather_than_raising(self):
        for key in (None, "", "no-such-plate-xyz", True):
            self.assertIsNone(plate_footprint_extent_mm(key))


# ── The shared fit ────────────────────────────────────────────────

class TestFitWells(PlateFixtureMixin, unittest.TestCase):

    def _fit(self, plate, w=560, h=380, **kw):
        wells = wells_from_plate(plate)
        return wells, fit_wells(wells, w, h, (4, 4, 4, 4), **kw)

    def test_every_well_gets_its_own_centre(self):
        wells, t = self._fit(self.custom)
        centres = {t.to_px(x, y) for _n, x, y, _d in wells}
        self.assertEqual(len(centres), N_WELLS)

    def test_every_well_gets_its_own_radius(self):
        wells, t = self._fit(self.custom)
        radii = {round(t.radius_px(d), 4) for _n, _x, _y, d in wells}
        self.assertEqual(len(radii), 2, "mixed diameters collapsed to one size")

    def test_nothing_is_clipped(self):
        """The print monitor's fit used max_x/max_y only, with no radius and no
        minimum corner, so wells ran off every edge."""
        for plate in (self.custom, self.std24, self.std96):
            wells, t = self._fit(plate)
            for _n, x, y, d in wells:
                px, py = t.to_px(x, y)
                r = t.radius_px(d)
                self.assertGreaterEqual(px - r, -0.5)
                self.assertGreaterEqual(py - r, -0.5)
                self.assertLessEqual(px + r, 560.5)
                self.assertLessEqual(py + r, 380.5)

    def test_minimum_radius_floor(self):
        wells, t = self._fit(self.custom, w=60, h=40, min_radius_px=4.0)
        for _n, _x, _y, d in wells:
            self.assertGreaterEqual(t.radius_px(d), 4.0)

    def test_round_trip_px_mm(self):
        _wells, t = self._fit(self.custom)
        for x, y in ((0.0, 0.0), (12.5, -3.25), (80.0, 40.0)):
            rx, ry = t.to_mm(*t.to_px(x, y))
            self.assertAlmostEqual(rx, x, places=6)
            self.assertAlmostEqual(ry, y, places=6)

    def test_footprint_framing_leaves_a_border(self):
        wells = wells_from_plate(self.custom)
        footprint = _doc_footprint_extent_mm(make_insert_plate_doc())
        t = fit_wells(wells, 560, 380, (4, 4, 4, 4), footprint=footprint)
        xs = [t.to_px(x, y)[0] - t.radius_px(d) for _n, x, y, d in wells]
        self.assertGreater(min(xs), 4.0,
                           "wells touch the edge — footprint not applied")

    def test_degenerate_inputs(self):
        self.assertIsNone(fit_wells([], 100, 100))
        self.assertIsNone(fit_wells([("A1", 0, 0, 5)], 0, 100))
        solo = fit_wells([("A1", 0.0, 0.0, 5.0)], 100, 100)
        self.assertIsNotNone(solo)
        self.assertTrue(math.isfinite(solo.scale))
        self.assertGreater(solo.scale, 0.0)

    def test_standard_grid_stays_a_grid(self):
        wells, t = self._fit(self.std24)
        rows = {round(t.to_px(x, y)[1], 3) for _n, x, y, _d in wells}
        cols = {round(t.to_px(x, y)[0], 3) for _n, x, y, _d in wells}
        self.assertEqual(len(rows), 4)
        self.assertEqual(len(cols), 6)


# ── WellPlateNavigator (Quick Print + Fluorescence Mosaic) ────────

class TestNavigator(PlateFixtureMixin, unittest.TestCase):

    def _nav(self, plate, w=560, h=380):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        nav = WellPlateNavigator()
        nav.resize(w, h)
        nav.set_plate(plate)
        nav.set_well_positions(plate.get_all_positions_from_a1(0.0, 0.0))
        return nav

    def test_every_well_is_drawn_at_its_own_place(self):
        """Before the fix this was 8 circles for 12 wells: each 3-well ring
        collapsed onto one pseudo-grid cell."""
        nav = self._nav(self.custom)
        layout = nav._well_layout()
        centres = {layout["transform"].to_px(x, y)
                   for _n, x, y, _d in layout["wells"]}
        self.assertEqual(len(layout["wells"]), N_WELLS)
        self.assertEqual(len(centres), N_WELLS)

    def test_ring_wells_are_smaller_than_insert_wells(self):
        nav = self._nav(self.custom)
        layout = nav._well_layout()
        by_name = {n: nav._well_radius(layout, d)
                   for n, _x, _y, d in layout["wells"]}
        self.assertLess(by_name["C1"], by_name["A1"])

    def test_every_well_is_individually_clickable(self):
        nav = self._nav(self.custom)
        layout = nav._well_layout()
        for name, x, y, _d in layout["wells"]:
            px, py = layout["transform"].to_px(x, y)
            self.assertEqual(
                nav._hit_test(QPoint(int(round(px)), int(round(py)))), name,
                f"{name} is not reachable by a click at its own centre")

    def test_ring_members_are_distinct_click_targets(self):
        nav = self._nav(self.custom)
        layout = nav._well_layout()
        hits = set()
        for name, x, y, _d in layout["wells"]:
            if not name.startswith(("C", "D")):
                continue
            px, py = layout["transform"].to_px(x, y)
            hits.add(nav._hit_test(QPoint(int(round(px)), int(round(py)))))
        self.assertEqual(len(hits), 2 * RING_COUNT)

    def test_headers_only_on_a_real_grid(self):
        self.assertFalse(self._nav(self.custom)._well_layout()["headers"])
        self.assertTrue(self._nav(self.std24)._well_layout()["headers"])

    def test_standard_plate_still_renders_every_well(self):
        for plate, n in ((self.std24, 24), (self.std96, 96)):
            layout = self._nav(plate)._well_layout()
            self.assertEqual(len(layout["wells"]), n)
            centres = {layout["transform"].to_px(x, y)
                       for _n, x, y, _d in layout["wells"]}
            self.assertEqual(len(centres), n)

    def test_a1_is_top_left(self):
        nav = self._nav(self.std24)
        layout = nav._well_layout()
        pts = {n: layout["transform"].to_px(x, y)
               for n, x, y, _d in layout["wells"]}
        for name, (px, py) in pts.items():
            if name == "A1":
                continue
            self.assertTrue(px >= pts["A1"][0] - 1e-6
                            and py >= pts["A1"][1] - 1e-6)

    def test_current_well_highlight_works_on_a_custom_plate(self):
        """The proximity gate was ``well_spacing_x * 1000 * 0.6`` — zero on a
        parametric plate, so the current well could never light up."""
        nav = self._nav(self.custom)
        positions = self.custom.get_all_positions_from_a1(0.0, 0.0)
        wx, wy = positions["C2"]
        nav.update_current_from_position(wx, wy)
        self.assertEqual(nav._current_well, "C2")

    def test_proximity_gate_uses_the_pitch_not_the_well_size(self):
        """A discriminating case for the gate itself.

        On the plate above the largest well (28 mm) is wider than the measured
        pitch, so a diameter-based gate happens to be generous enough and the
        distinction is invisible. Here the wells are 4 mm on a 9 mm pitch, so
        there is a band — wider than ``diameter * 0.6`` (2.4 mm), narrower than
        ``pitch * 0.6`` (5.4 mm) — in which only a pitch-sized gate matches.
        The probe also has to stay inside half the pitch, or the NEIGHBOURING
        well becomes the nearest one and the assertion means nothing.
        """
        doc = PlateDocument.new_plate("Sparse small wells")
        doc.add_grid(0.0, 0.0, rows=3, cols=3,
                     pitch_x_mm=9.0, pitch_y_mm=9.0,
                     style=WellStyle(diameter_mm=4.0))
        plate = doc.compile()
        self.assertEqual(plate.well_spacing_x, 0.0)
        self.assertAlmostEqual(plate.nearest_neighbour_pitch_mm(), 9.0)

        pitch = plate.nearest_neighbour_pitch_mm()
        pitch_gate = pitch * 0.6
        diam_gate = plate.representative_well_diameter * 0.6
        upper = min(pitch_gate, pitch / 2.0 - 0.2)
        probe_mm = (diam_gate + upper) / 2.0
        self.assertTrue(diam_gate < probe_mm < upper,
                        "fixture cannot separate the two gates")

        nav = self._nav(plate)
        wx, wy = plate.get_all_positions_from_a1(0.0, 0.0)["B2"]
        nav.update_current_from_position(wx + probe_mm * 1000.0, wy)
        self.assertEqual(
            nav._current_well, "B2",
            "proximity gate is not sized from the measured well pitch")

    def test_current_well_clears_when_far_away(self):
        nav = self._nav(self.custom)
        nav.update_current_from_position(1_000_000.0, 1_000_000.0)
        self.assertIsNone(nav._current_well)

    def test_paints_without_error(self):
        for plate in (self.custom, self.std24, self.std96):
            nav = self._nav(plate)
            nav.set_current_well(plate.well_names[0])
            nav.set_raster_grid(3, 2)
            self.assertFalse(nav.grab().isNull())

    def test_raster_preview_uses_the_current_wells_own_radius(self):
        """It used to index the pseudo-grid and reuse `r` left over from the
        well loop, so the preview was drawn at the last well's size."""
        nav = self._nav(self.custom)
        nav.set_current_well("C1")
        nav.set_raster_grid(3, 3)
        self.assertFalse(nav.grab().isNull())

    def test_no_plate_is_safe(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        nav = WellPlateNavigator()
        nav.resize(200, 150)
        self.assertIsNone(nav._well_layout())
        self.assertIsNone(nav._hit_test(QPoint(10, 10)))
        self.assertFalse(nav.grab().isNull())

    def test_a_plate_whose_wells_carry_no_geometry_does_not_crash_paint(self):
        """``paintEvent`` must not raise.

        The widget now needs real ``x``/``y``; a duck-typed plate that carries
        only ``row``/``col`` cannot be laid out. Skipping those wells is the
        honest outcome — defaulting them to the origin would stack every well
        on one point, which is the bug this module exists to prevent.
        """
        class _Well:
            def __init__(self, name):
                self.name = name
                self.row = self.col = 0

        class _Plate:
            rows = cols = 1
            well_spacing_x = well_spacing_y = 0.0
            well_diameter = 0.0

            def get_all_wells(self):
                return [_Well("A1"), _Well("A2")]

        from gui.widgets.jog_well_plate import WellPlateNavigator
        self.assertEqual(wells_from_plate(_Plate()), [])
        nav = WellPlateNavigator()
        nav.resize(200, 150)
        nav.set_plate(_Plate())
        nav.set_raster_grid(3, 3)
        self.assertIsNone(nav._well_layout())
        self.assertFalse(nav.grab().isNull())


# ── Print monitor overview ────────────────────────────────────────

class TestPrintMonitorOverview(PlateFixtureMixin, unittest.TestCase):

    def _widget(self, plate, w=400, h=300):
        from gui.pages.print_monitor import PlateOverviewWidget
        widget = PlateOverviewWidget()
        widget.resize(w, h)
        widget.set_plate(plate)
        return widget

    def test_each_well_keeps_its_own_diameter(self):
        """It sized every well from ``self._wells[0]["diameter"]``, drawing
        5.5 mm bores as 28 mm blobs overlapping their neighbours."""
        widget = self._widget(self.custom)
        self.assertEqual({w["diameter"] for w in widget._wells},
                         {GRID_DIAMETER_MM, RING_DIAMETER_MM})

    def test_headers_only_on_a_real_grid(self):
        self.assertFalse(self._widget(self.custom)._grid_headers)
        self.assertTrue(self._widget(self.std24)._grid_headers)

    def test_nothing_is_clipped(self):
        widget = self._widget(self.custom)
        t = fit_wells(
            [(w["name"], w["x"], w["y"], w["diameter"]) for w in widget._wells],
            400, 300,
            (widget._margin,) * 4, footprint=widget._footprint,
            min_radius_px=3.0)
        for w in widget._wells:
            px, py = t.to_px(w["x"], w["y"])
            r = t.radius_px(w["diameter"]) * 0.75
            self.assertGreaterEqual(px - r, -0.5)
            self.assertGreaterEqual(py - r, -0.5)
            self.assertLessEqual(px + r, 400.5)
            self.assertLessEqual(py + r, 300.5)

    def test_paints_without_error(self):
        for plate in (self.custom, self.std24):
            widget = self._widget(plate)
            widget.set_needle_position(0.0, 0.0)
            self.assertFalse(widget.grab().isNull())

    def test_a_small_well_is_PAINTED_small(self):
        """Checks the pixels, not the model.

        Asserting on ``widget._wells`` only proves the data carries per-well
        diameters — it says nothing about the radius the painter uses, which is
        where the bug lived (``self._wells[0]["diameter"]`` for every well).
        This probes a point that must be empty background beside a 5.5 mm bore
        but would be inside it were it drawn at the 28 mm insert's radius.
        """
        widget = self._widget(self.custom, w=800, h=600)
        t = fit_wells(
            [(w["name"], w["x"], w["y"], w["diameter"]) for w in widget._wells],
            800, 600, (widget._margin,) * 4,
            footprint=widget._footprint, min_radius_px=3.0)

        def draw_r(diameter):
            return t.radius_px(diameter) * 0.75

        small = next(w for w in widget._wells if w["name"] == "C1")
        cx, cy = t.to_px(small["x"], small["y"])
        r_small = draw_r(RING_DIAMETER_MM)
        r_big = draw_r(GRID_DIAMETER_MM)
        self.assertGreater(r_big, r_small + 6.0, "fixture cannot discriminate")

        # A probe just outside C1 but well inside a 28 mm circle, and clear of
        # every other well's true footprint.
        probe = None
        for step in range(int(r_small) + 4, int(r_big) - 2):
            for ang in range(0, 360, 5):
                px = cx + step * math.cos(math.radians(ang))
                py = cy + step * math.sin(math.radians(ang))
                if not (2 < px < 798 and 2 < py < 598):
                    continue
                if all(math.hypot(px - t.to_px(w["x"], w["y"])[0],
                                  py - t.to_px(w["x"], w["y"])[1])
                       > draw_r(w["diameter"]) + 3.0
                       for w in widget._wells):
                    probe = (int(round(px)), int(round(py)))
                    break
            if probe:
                break
        self.assertIsNotNone(probe, "no clear probe point found")

        image = widget.grab().toImage()
        colour = image.pixelColor(*probe)
        self.assertEqual(
            (colour.red(), colour.green(), colour.blue()), (24, 24, 37),
            f"pixel at {probe} is not background — a 5.5 mm bore is being "
            f"painted at another well's radius")


# ── Calibration page: the motion half ─────────────────────────────

class TestCalibrationWellPicks(PlateFixtureMixin, unittest.TestCase):
    """These are not cosmetic: the names feed ``get_well_position`` and then
    the stage."""

    def _page(self, plate):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage.__new__(CalibrationPage)
        page._plate = plate
        page._corner_well = plate.extreme_well(1.0, 1.0)
        return page

    def test_calibration_wells_exist_on_a_custom_plate(self):
        page = self._page(self.custom)
        wells = page._get_calibration_wells()
        self.assertEqual(len(wells), 3)
        for name in wells:
            self.assertIn(name, self.custom.well_names)
            self.custom.get_well_position(name)   # must not raise

    def test_third_well_exists_and_is_distinct(self):
        page = self._page(self.custom)
        third = page._pick_third_well()
        self.assertIn(third, self.custom.well_names)
        self.assertNotIn(third, ("A1", page._corner_well))

    def test_corner_well_exists(self):
        page = self._page(self.custom)
        self.assertIn(page._corner_well, self.custom.well_names)

    def test_standard_plates_keep_their_historical_picks(self):
        for plate, tri, corner, third in (
                (self.std24, ["A1", "A6", "D6"], "D6", "A6"),
                (self.std96, ["A1", "A12", "H12"], "H12", "A12")):
            page = self._page(plate)
            self.assertEqual(page._get_calibration_wells(), tri)
            self.assertEqual(page._corner_well, corner)
            self.assertEqual(page._pick_third_well(), third)

    def test_diameters_and_pitch_are_never_zero(self):
        from gui.pages.calibration import _well_diameter_mm, _well_pitch_mm
        self.assertAlmostEqual(_well_diameter_mm(self.custom, "A1"),
                               GRID_DIAMETER_MM)
        self.assertAlmostEqual(_well_diameter_mm(self.custom, "C1"),
                               RING_DIAMETER_MM)
        self.assertGreater(_well_diameter_mm(self.custom), 0.0)
        self.assertGreater(_well_pitch_mm(self.custom), 0.0)

    def test_helpers_are_free_functions_so_stub_pages_keep_working(self):
        """The suite drives these handlers through ``__new__`` /
        ``SimpleNamespace`` stubs carrying only ``_plate``. A bound helper
        would break every one of them, so these take the plate explicitly."""
        import types
        from gui.pages.calibration import (
            CalibrationPage, _well_diameter_mm, _well_pitch_mm,
        )
        self.assertFalse(hasattr(CalibrationPage, "_well_diameter_mm"))
        self.assertFalse(hasattr(CalibrationPage, "_well_pitch_mm"))
        stub = types.SimpleNamespace(_plate=self.custom)
        self.assertGreater(_well_diameter_mm(stub._plate, "A1"), 0.0)

    def test_helpers_are_safe_without_a_plate(self):
        from gui.pages.calibration import _well_diameter_mm, _well_pitch_mm
        page = self._page(self.custom)
        page._plate = None
        self.assertEqual(_well_diameter_mm(None, "A1"), 0.0)
        self.assertEqual(_well_pitch_mm(None), 0.0)
        self.assertEqual(page._get_calibration_wells(), [])


# ── Remaining consumers ───────────────────────────────────────────

class TestJogAndContextConsumers(PlateFixtureMixin, unittest.TestCase):

    def test_jog_workspace_resolves_per_well_radii(self):
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        view = JogWorkspaceView()
        view.set_plate(self.custom)
        self.assertAlmostEqual(view._well_radius_um("A1"),
                               GRID_DIAMETER_MM * 500.0)
        self.assertAlmostEqual(view._well_radius_um("C1"),
                               RING_DIAMETER_MM * 500.0)
        self.assertGreater(view._well_radius_um(), 0.0)

    def test_jog_workspace_unchanged_on_a_standard_plate(self):
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        view = JogWorkspaceView()
        view.set_plate(self.std24)
        self.assertAlmostEqual(view._well_radius_um("A1"),
                               self.std24.well_diameter * 500.0)

    def test_xz_well_under_needle_finds_a_custom_plate_well(self):
        """``well_r_um`` came from the plate-level diameter (0.0), so the
        ``abs(...) <= 0`` test never matched and the XZ side view never showed
        a well under the needle."""
        from gui.pages.jog_control import JogControlPage

        class _Ctrl:
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        class _XZ:
            def __init__(self):
                self.calls = []

            def set_well_under_needle(self, diameter, depth):
                self.calls.append((diameter, depth))

        page = JogControlPage.__new__(JogControlPage)
        page._plate = self.custom
        page.controller = _Ctrl()
        page._xz_view = _XZ()
        page._well_positions = self.custom.get_all_positions_from_a1(0.0, 0.0)

        cx, cy = page._well_positions["C2"]
        page._update_xz_well_under_needle(cx, cy)
        self.assertEqual(page._xz_view.calls[-1][0], RING_DIAMETER_MM)

        ax, ay = page._well_positions["A1"]
        page._update_xz_well_under_needle(ax, ay)
        self.assertEqual(page._xz_view.calls[-1][0], GRID_DIAMETER_MM)

    def test_xz_clears_when_between_wells(self):
        from gui.pages.jog_control import JogControlPage

        class _Ctrl:
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        class _XZ:
            def __init__(self):
                self.calls = []

            def set_well_under_needle(self, diameter, depth):
                self.calls.append((diameter, depth))

        page = JogControlPage.__new__(JogControlPage)
        page._plate = self.custom
        page.controller = _Ctrl()
        page._xz_view = _XZ()
        page._well_positions = self.custom.get_all_positions_from_a1(0.0, 0.0)
        page._update_xz_well_under_needle(500_000.0, 500_000.0)
        self.assertEqual(page._xz_view.calls[-1], (None, None))

    def _hardware_info_plate_text(self, plate) -> str:
        """Drive the REAL ``_refresh_hardware_info`` and read the plate row.

        Building the whole panel needs a live controller, so the method is
        called unbound against a stub carrying only what it touches — the
        pattern the calibration suites use. Asserting against a copy of the
        expression would test the copy, not the panel.
        """
        from gui.widgets.standard_jog_context import StandardJogContextPanel

        class _Label:
            def __init__(self):
                self.text = "—"

            def setText(self, value):
                self.text = value

        class _Stub:
            """Auto-vivifies the readout labels the method writes to."""

            def __getattr__(self, name):
                if name.startswith("_lbl_"):
                    label = _Label()
                    setattr(self, name, label)
                    return label
                raise AttributeError(name)

        stub = _Stub()
        stub._plate = plate
        stub._hardware_config = None
        stub._well_positions = None
        stub._z_refs = {}
        stub._safe_z = None
        StandardJogContextPanel._refresh_hardware_info(stub)
        return stub._lbl_info_plate.text

    def test_hardware_info_plate_line_counts_real_wells(self):
        """It printed ``rows * cols`` and the plate-level diameter — "15-well ·
        0.00 mm Ø" for this 12-well plate."""
        text = self._hardware_info_plate_text(self.custom)
        self.assertIn(f"{N_WELLS}-well", text)
        self.assertNotIn("15-well", text)
        self.assertIn("5.50", text)
        self.assertIn("28.00", text)
        self.assertNotIn("0.00 mm", text)

    def test_hardware_info_plate_line_unchanged_on_a_standard_plate(self):
        text = self._hardware_info_plate_text(self.std24)
        self.assertIn("24-well", text)
        self.assertIn("15.60 mm", text)


class TestWellPlateViewStillCorrect(PlateFixtureMixin, unittest.TestCase):
    """This widget was already right; the de-duplication must not change it."""

    def test_renders_every_well_at_its_own_size(self):
        from gui.widgets.well_plate_view import WellPlateView
        view = WellPlateView()
        view.resize(620, 420)
        view.set_plate(self.custom)
        view.update_all_wells({})
        self.assertEqual(len(view._well_items), N_WELLS)
        widths = {round(i.rect().width(), 3) for i in view._well_items.values()}
        self.assertEqual(len(widths), 2)

    def test_standard_plate_unchanged(self):
        from gui.widgets.well_plate_view import WellPlateView
        view = WellPlateView()
        view.resize(620, 420)
        view.set_plate(self.std24)
        view.update_all_wells({})
        self.assertEqual(len(view._well_items), 24)
        widths = {round(i.rect().width(), 3) for i in view._well_items.values()}
        self.assertEqual(len(widths), 1)


# ── Anti-drift ────────────────────────────────────────────────────

class TestOneDiameterImplementation(PlateFixtureMixin, unittest.TestCase):
    """The per-well → plate-level → largest chain was written out four times.

    Three copies agreeing the day they are written says nothing about the next
    edit, so these pin that the copies are gone AND that the survivors behave
    identically to the one implementation.
    """

    _DELEGATES = [
        ("gui.widgets.jog_workspace_view", "JogWorkspaceView",
         "_well_radius_um"),
        ("gui.pages.print_builder_sketch", "SketchPage",
         "_well_diameter_for_selected"),
        ("gui.pages.calibration", None, "_well_diameter_mm"),
    ]

    def test_no_module_reimplements_the_largest_well_fallback(self):
        import importlib
        for module_name, cls_name, method in self._DELEGATES:
            module = importlib.import_module(module_name)
            owner = getattr(module, cls_name) if cls_name else module
            src = inspect.getsource(getattr(owner, method))
            tree = ast.parse(textwrap.dedent(src))
            calls_helper = any(
                isinstance(node, ast.Attribute)
                and node.attr in ("well_diameter_of",
                                  "representative_well_diameter")
                for node in ast.walk(tree))
            self.assertTrue(
                calls_helper,
                f"{module_name}.{cls_name}.{method} no longer delegates to "
                f"WellPlate — the fallback chain has been re-implemented")

    def test_jog_workspace_matches_the_one_implementation(self):
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        view = JogWorkspaceView()
        view.set_plate(self.custom)
        for name in self.custom.well_names + ["ZZ99", None]:
            self.assertAlmostEqual(
                view._well_radius_um(name),
                self.custom.well_diameter_of(name) * 500.0)

    def test_calibration_matches_the_one_implementation(self):
        from gui.pages.calibration import _well_diameter_mm
        for name in self.custom.well_names + ["ZZ99", None]:
            self.assertAlmostEqual(_well_diameter_mm(self.custom, name),
                                   self.custom.well_diameter_of(name))


class TestNoConsumerDerivesAGridFromRowsCols(unittest.TestCase):
    """``rows``/``cols`` describe a real grid only when the spacing does.

    A new renderer that lays wells out on ``plate.rows`` × ``plate.cols`` cells
    reintroduces exactly this bug, so the two widgets that used to are pinned
    against the pattern returning.
    """

    _WIDGETS = [
        ("gui/widgets/jog_well_plate.py", "_well_layout"),
        ("gui/pages/print_monitor.py", "paintEvent"),
    ]

    def test_layout_functions_do_not_read_rows_or_cols(self):
        for path, func_name in self._WIDGETS:
            with open(path, encoding="utf-8") as handle:
                tree = ast.parse(handle.read())
            found = False
            for node in ast.walk(tree):
                if not isinstance(node, ast.FunctionDef) \
                        or node.name != func_name:
                    continue
                found = True
                for sub in ast.walk(node):
                    if isinstance(sub, ast.Attribute) \
                            and sub.attr in ("rows", "cols"):
                        self.fail(
                            f"{path}:{func_name} reads .{sub.attr} — the "
                            f"pseudo-grid layout is back")
            self.assertTrue(found, f"{func_name} not found in {path}")


if __name__ == "__main__":
    unittest.main()
