"""Tests for v7.4.5 PlateDesign + WellPlate generalization."""

from __future__ import annotations

import json
import shutil
import tempfile
import unittest
from pathlib import Path

from SupportClasses.PlateDesign import (
    PlateDesign, Point, Well, Group, Constraint,
)
from SupportClasses.WellPlate import WellPlate, WellInfo, PLATE_DEFINITIONS


class TestWellPlateGeneralization(unittest.TestCase):
    def test_from_format_unchanged(self):
        """Standard-format construction works exactly as before."""
        p = WellPlate.from_format(96)
        self.assertEqual(p.format, 96)
        self.assertEqual(p.rows, 8)
        self.assertEqual(p.cols, 12)
        self.assertEqual(len(p.get_all_wells()), 96)
        self.assertEqual(p.get_well_position("A1"), (0.0, 0.0))
        self.assertEqual(p.get_well_position("H12"), (99.0, 63.0))
        self.assertFalse(p.is_custom)

    def test_from_wells_custom_plate(self):
        wells = [
            WellInfo(name="A1", row=0, col=0, x=0.0, y=0.0, diameter=8.0),
            WellInfo(name="A2", row=0, col=1, x=15.0, y=0.0, diameter=10.0),
            WellInfo(name="B1", row=1, col=0, x=0.0, y=12.0, diameter=8.0),
        ]
        p = WellPlate.from_wells(
            "test-plate", wells, a1_offset_x=10.0, a1_offset_y=10.0,
            description="three irregular wells")
        self.assertEqual(p.format, "custom:test-plate")
        self.assertTrue(p.is_custom)
        self.assertEqual(p.rows, 2)
        self.assertEqual(p.cols, 2)
        self.assertEqual(p.well_diameter, 0.0)  # varies — per-well
        self.assertEqual(p.well_spacing_x, 0.0)
        self.assertEqual(p.get_well_position("A2"), (15.0, 0.0))
        self.assertAlmostEqual(p._max_well_radius_mm(), 5.0)
        # Bounding box accounts for per-well radii.
        bbox = p.get_bounding_box()
        self.assertAlmostEqual(bbox[0], -5.0)
        self.assertAlmostEqual(bbox[2], 20.0)

    def test_load_with_int_matches_from_format(self):
        a = WellPlate.from_format(12)
        b = WellPlate.load(12)
        self.assertEqual(a.format, b.format)
        self.assertEqual(a.rows, b.rows)
        self.assertEqual(a.cols, b.cols)
        self.assertEqual(a.get_well_position("A1"), b.get_well_position("A1"))
        self.assertEqual(a.get_well_position("C4"), b.get_well_position("C4"))

    def test_load_with_digit_string_matches_int(self):
        a = WellPlate.from_format(6)
        b = WellPlate.load("6")
        self.assertEqual(a.format, b.format)


class TestPlateDesignBasics(unittest.TestCase):
    def test_from_standard_format_compiles_to_equivalent_plate(self):
        """compile() of a cloned standard yields the same well positions."""
        for fmt in (6, 12, 24, 48, 96, 384):
            d = PlateDesign.from_standard_format(fmt)
            p_from_design = d.compile()
            p_native = WellPlate.from_format(fmt)
            with self.subTest(fmt=fmt):
                self.assertEqual(len(p_from_design.get_all_wells()), fmt)
                self.assertEqual(
                    p_native.get_well_position("A1"),
                    p_from_design.get_well_position("A1"))
                last = p_native.get_all_wells()[-1].name
                self.assertEqual(
                    p_native.get_well_position(last),
                    p_from_design.get_well_position(last))

    def test_compile_empty_design_raises(self):
        d = PlateDesign(name="empty")
        with self.assertRaises(ValueError):
            d.compile()

    def test_add_well_creates_center_point(self):
        d = PlateDesign(name="t")
        w = d.add_well(x=5.0, y=7.0, diameter=4.5, name="X1")
        self.assertIsInstance(w, Well)
        center = d.entities[w.center]
        self.assertIsInstance(center, Point)
        self.assertEqual(center.x, 5.0)
        self.assertEqual(center.y, 7.0)

    def test_remove_well_drops_orphan_center_and_constraints(self):
        d = PlateDesign(name="t")
        w = d.add_well(x=5.0, y=7.0, diameter=4.5, name="X1")
        center_id = w.center
        d.add_constraint(Constraint(kind="ground", refs=[center_id]))
        self.assertIn(center_id, d.entities)
        self.assertEqual(len(d.constraints), 1)
        d.remove_entity(w.id)
        self.assertNotIn(w.id, d.entities)
        self.assertNotIn(center_id, d.entities)   # orphan center removed
        self.assertEqual(len(d.constraints), 0)   # constraint touching it removed

    def test_grid_assigns_ansi_names(self):
        d = PlateDesign(name="g")
        g = d.add_grid(
            rows=2, cols=3, spacing_x=9.0, spacing_y=9.0,
            origin_x=0.0, origin_y=0.0, diameter=6.0,
            group_name="ts")
        # Two rows of three wells named A1, A2, A3, B1, B2, B3.
        names = sorted(d.entities[wid].name for wid in g.members
                       if isinstance(d.entities[wid], Well))
        self.assertEqual(names, ["A1", "A2", "A3", "B1", "B2", "B3"])

    def test_round_trip_serialization(self):
        d = PlateDesign.from_standard_format(24)
        as_dict = d.to_dict()
        as_json = json.loads(json.dumps(as_dict))
        d2 = PlateDesign.from_dict(as_json)
        self.assertEqual(d2.name, d.name)
        self.assertEqual(len(d2.get_wells()), len(d.get_wells()))
        # Spot-check a well from each design.
        a1 = next(w for w in d.get_wells() if w.name == "A1")
        a1b = next(w for w in d2.get_wells() if w.name == "A1")
        self.assertEqual(d.get_well_position(a1), d2.get_well_position(a1b))

    def test_compile_clusters_arbitrary_layout(self):
        """Wells placed irregularly still get sensible row/col indices."""
        d = PlateDesign(name="irreg")
        d.ensure_outline_origin()
        # Three wells: two in the same row, one below.
        d.add_well(x=0.0, y=0.0, diameter=4.0, name="A1")
        d.add_well(x=10.0, y=0.05, diameter=4.0, name="A2")  # within 0.5 tol
        d.add_well(x=0.0, y=10.0, diameter=4.0, name="B1")
        p = d.compile()
        a1 = p.get_well_info("A1")
        a2 = p.get_well_info("A2")
        b1 = p.get_well_info("B1")
        self.assertEqual(a1.row, a2.row)
        self.assertNotEqual(a1.row, b1.row)
        # A1 should be col 0; A2 should be col 1 (sorted by X within row).
        self.assertEqual(a1.col, 0)
        self.assertEqual(a2.col, 1)

    def test_add_circle_pattern_places_n_wells_evenly(self):
        """N wells on a ring of given radius and start angle."""
        d = PlateDesign(name="ring")
        d.ensure_outline_origin()
        group = d.add_circle_pattern(
            count=4, center_x=10.0, center_y=10.0,
            radius=5.0, diameter=2.0, start_angle_deg=0.0,
            group_name="r1")
        wells = [d.entities[m] for m in group.members
                 if isinstance(d.entities[m], Well)]
        self.assertEqual(len(wells), 4)
        # At start_angle=0 the first well should be at (15, 10).
        x0, y0 = d.get_well_position(wells[0])
        self.assertAlmostEqual(x0, 15.0, places=6)
        self.assertAlmostEqual(y0, 10.0, places=6)
        # Each well is `radius` from the center.
        import math
        for w in wells:
            x, y = d.get_well_position(w)
            self.assertAlmostEqual(
                math.hypot(x - 10.0, y - 10.0), 5.0, places=6)

    def test_circle_pattern_raises_on_zero_count(self):
        d = PlateDesign(name="ring")
        with self.assertRaises(ValueError):
            d.add_circle_pattern(
                count=0, center_x=0.0, center_y=0.0,
                radius=5.0, diameter=2.0)


class TestWellPlateLoadFromUserDir(unittest.TestCase):
    """Verify load() works against a freshly-written user JSON file."""

    def setUp(self):
        # Redirect USER_PLATES_DIR to a temp area for hermetic testing.
        # Use `importlib` to grab the modules (not the WellPlate class
        # that shadows the name when imported as `from SupportClasses import WellPlate`).
        import importlib
        wp_mod = importlib.import_module("SupportClasses.WellPlate")
        pd_mod = importlib.import_module("SupportClasses.PlateDesign")
        self._wp_mod = wp_mod
        self._pd_mod = pd_mod
        self._orig_user_dir = wp_mod.USER_PLATES_DIR
        self._tmp = Path(tempfile.mkdtemp(prefix="mebp_plates_test_"))
        wp_mod.USER_PLATES_DIR = self._tmp
        pd_mod.USER_PLATES_DIR = self._tmp

    def tearDown(self):
        self._wp_mod.USER_PLATES_DIR = self._orig_user_dir
        self._pd_mod.USER_PLATES_DIR = self._orig_user_dir
        shutil.rmtree(self._tmp, ignore_errors=True)

    def test_save_then_load_returns_equivalent_plate(self):
        d = PlateDesign.from_standard_format(12)
        d.name = "round-trip-12"
        d.save()
        # Now load by name through WellPlate.load(str).
        p = WellPlate.load("round-trip-12")
        self.assertTrue(p.is_custom)
        self.assertEqual(p.format, "custom:round-trip-12")
        self.assertEqual(len(p.get_all_wells()), 12)
        # Positions match the native 12-well exactly.
        p_native = WellPlate.from_format(12)
        for name in p_native.well_names:
            self.assertEqual(
                p.get_well_position(name),
                p_native.get_well_position(name))

    def test_load_missing_file_raises(self):
        with self.assertRaises(FileNotFoundError):
            WellPlate.load("does-not-exist-anywhere")


if __name__ == "__main__":
    unittest.main()
