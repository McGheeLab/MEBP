"""v7.12 Phase 1 — the geometry gate for the new plate document.

THE point of this file is the golden-position matrix: for every origin mode and
every bundled format, the compiled plate must be byte-identical to
``WellPlate.from_format``. Origin is an authoring affordance; if it can reach
the compiled frame, every well lands ~0.9 well-pitch off while Go-To-A1 still
looks right (A1 being the anchor), and nothing downstream can tell.

Written before the builder UI exists, and deliberately asserting the FULL well
set rather than a couple of corners — a pure row inversion preserves both
endpoints on some formats.
"""
from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

from SupportClasses.PlateDocument import (
    BoundaryKind, Constraint, FutureSchemaError, GridPattern, LegacySchemaError,
    MemberOverride, NamingScheme, NamingSpec, OriginRef, PlateBoundary,
    PlateDocument, RingPattern, RosettePlacement, WellStyle, ansi_label,
    is_store_key_safe, letter_label, new_doc_id,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS, WellPlate

ALL_FORMATS = sorted(PLATE_DEFINITIONS.keys())
ALL_ORIGINS = [OriginRef.A1, OriginRef.BOTTOM_LEFT, OriginRef.BOTTOM_RIGHT,
               OriginRef.TOP_LEFT, OriginRef.TOP_RIGHT, OriginRef.CENTER]


# ═══════════════════════════════════════════════════════════════════
# The golden-position matrix
# ═══════════════════════════════════════════════════════════════════

class TestGoldenPositions(unittest.TestCase):
    def test_every_format_compiles_to_the_bundled_plate(self):
        for fmt in ALL_FORMATS:
            with self.subTest(fmt=fmt):
                want = WellPlate.from_format(fmt)
                got = PlateDocument.from_standard_format(fmt).compile()
                self.assertEqual(sorted(want.well_names),
                                 sorted(got.well_names))
                for name in want.well_names:
                    wx, wy = want.get_well_position(name)
                    gx, gy = got.get_well_position(name)
                    self.assertAlmostEqual(wx, gx, places=9, msg=name)
                    self.assertAlmostEqual(wy, gy, places=9, msg=name)

    def test_origin_mode_never_reaches_the_compiled_frame(self):
        """The one that stops a needle going into glass."""
        for fmt in ALL_FORMATS:
            want = WellPlate.from_format(fmt)
            for origin in ALL_ORIGINS:
                with self.subTest(fmt=fmt, origin=origin):
                    doc = PlateDocument.from_standard_format(fmt)
                    doc.boundary.origin_ref = origin
                    got = doc.compile()
                    for name in want.well_names:
                        wx, wy = want.get_well_position(name)
                        gx, gy = got.get_well_position(name)
                        self.assertAlmostEqual(wx, gx, places=9, msg=f"{name}/{origin}")
                        self.assertAlmostEqual(wy, gy, places=9, msg=f"{name}/{origin}")

    def test_derived_a1_offsets_match_the_bundled_definition(self):
        for fmt in ALL_FORMATS:
            with self.subTest(fmt=fmt):
                pdef = PLATE_DEFINITIONS[fmt]
                got = PlateDocument.from_standard_format(fmt).compile()
                self.assertAlmostEqual(pdef["a1_offset_x"], got.a1_offset_x,
                                       places=9)
                self.assertAlmostEqual(pdef["a1_offset_y"], got.a1_offset_y,
                                       places=9)

    def test_a1_is_always_the_origin_of_the_compiled_frame(self):
        for fmt in ALL_FORMATS:
            with self.subTest(fmt=fmt):
                plate = PlateDocument.from_standard_format(fmt).compile()
                self.assertEqual((0.0, 0.0), plate.get_well_position("A1"))

    def test_switching_origin_leaves_the_document_byte_identical(self):
        """Origin is presentation. If it mutated geometry, every saved
        dimension would change meaning on load."""
        doc = PlateDocument.from_standard_format(24)
        doc.boundary.origin_ref = OriginRef.BOTTOM_LEFT
        baseline = doc.to_dict()
        baseline["boundary"].pop("origin_ref")
        values = [c.value for c in doc.constraints]

        for origin in ALL_ORIGINS:
            doc.boundary.origin_ref = origin
            blob = doc.to_dict()
            self.assertEqual(origin, blob["boundary"]["origin_ref"])
            blob["boundary"].pop("origin_ref")
            self.assertEqual(baseline, blob, f"origin {origin} mutated geometry")
            self.assertEqual(values, [c.value for c in doc.constraints])

    def test_translating_the_whole_plate_does_not_move_compiled_wells(self):
        """Compiled positions are A1-relative, so where the design sits in its
        own frame is irrelevant."""
        base = PlateDocument.from_standard_format(24).compile()
        doc = PlateDocument.from_standard_format(24)
        grid = doc.patterns()[0]
        anchor = doc.anchor_point(grid)
        anchor.x += 37.5
        anchor.y -= 12.25
        moved = doc.compile()
        for name in base.well_names:
            bx, by = base.get_well_position(name)
            mx, my = moved.get_well_position(name)
            # Almost-equal, not equal: subtracting a translated anchor is
            # ordinary float arithmetic. Bit-equality is asserted where it is
            # actually required — save → reload → recompile.
            self.assertAlmostEqual(bx, mx, places=9, msg=name)
            self.assertAlmostEqual(by, my, places=9, msg=name)

    def test_save_reload_recompile_is_bit_identical(self):
        """Guards the drift that would silently stale a taught calibration
        overnight with no user action."""
        doc = PlateDocument.from_standard_format(24)
        before = doc.compile()
        after = PlateDocument.from_dict(doc.to_dict()).compile()
        for name in before.well_names:
            self.assertEqual(before.get_well_position(name),
                             after.get_well_position(name), name)


# ═══════════════════════════════════════════════════════════════════
# The two prediction paths must agree
# ═══════════════════════════════════════════════════════════════════

class TestPredictionPathsAgree(unittest.TestCase):
    """``WellPlate.get_all_positions_from_a1`` assumes A1 sits at the model
    origin; ``calibration._estimate_well_position`` explicitly subtracts A1.
    They agree only while that assumption holds — and if they ever diverge the
    plate view shows one grid while Go-To drives to another.
    """

    def test_paths_agree_for_both_axis_signs(self):
        plate = PlateDocument.from_standard_format(24).compile()
        a1 = (105617.6, 65890.4)
        for sign in ((1, 1), (-1, -1)):
            with self.subTest(sign=sign):
                bulk = plate.get_all_positions_from_a1(
                    a1[0], a1[1], plate_axis_sign=sign)
                a1x, a1y = plate.get_well_position("A1")
                for name in plate.well_names:
                    wx, wy = plate.get_well_position(name)
                    ex = a1[0] + sign[0] * (wx - a1x) * 1000.0
                    ey = a1[1] + sign[1] * (wy - a1y) * 1000.0
                    gx, gy = bulk[name]
                    self.assertAlmostEqual(ex, gx, places=6, msg=name)
                    self.assertAlmostEqual(ey, gy, places=6, msg=name)


# ═══════════════════════════════════════════════════════════════════
# Patterns are non-destructive
# ═══════════════════════════════════════════════════════════════════

class TestPatternStability(unittest.TestCase):
    """``rebuild_group`` deleted every member and every constraint touching
    them on any parameter change. Derived members make that impossible.
    """

    def _ring_doc(self):
        doc = PlateDocument.new_plate("ring test")
        ring = doc.add_ring(0.0, 0.0, count=6, ring_diameter_mm=10.0)
        return doc, ring

    def test_growing_a_ring_preserves_existing_member_payloads(self):
        doc, ring = self._ring_doc()
        ov = doc.override(ring, "r2")
        ov.name = "special"
        ov.rim_height_mm = 3.5
        ov.well_type_id = "pcr-tube-0.1ml"
        ov.rosette = RosettePlacement(rosette_id="ros_abc", rotation_deg=30.0)

        before = {w.key: w for w in doc.evaluate()}
        ring.count = 8
        after = {w.key: w for w in doc.evaluate()}

        for i in range(6):
            key = (ring.id, f"r{i}")
            self.assertIn(key, after)
        kept = after[(ring.id, "r2")]
        self.assertEqual("special", kept.name)
        self.assertEqual(3.5, kept.rim_height_mm)
        self.assertEqual("pcr-tube-0.1ml", kept.well_type_id)
        self.assertEqual("ros_abc", kept.rosette.rosette_id)
        self.assertEqual(30.0, kept.rosette.rotation_deg)
        self.assertEqual(8, len([k for k in after if k[1].startswith("r")]))
        self.assertNotEqual(before[(ring.id, "r1")].x, after[(ring.id, "r1")].x)

    def test_shrinking_then_growing_restores_dormant_overrides(self):
        doc, ring = self._ring_doc()
        doc.override(ring, "r5").name = "last"
        ring.count = 3
        self.assertNotIn((ring.id, "r5"), {w.key for w in doc.evaluate()})
        ring.count = 6
        got = {w.key: w for w in doc.evaluate()}[(ring.id, "r5")]
        self.assertEqual("last", got.name)

    def test_prune_orphan_overrides_is_explicit(self):
        doc, ring = self._ring_doc()
        doc.override(ring, "r5").name = "last"
        ring.count = 3
        self.assertEqual(1, doc.prune_orphan_overrides(ring))
        ring.count = 6
        self.assertNotEqual(
            "last", {w.key: w for w in doc.evaluate()}[(ring.id, "r5")].name)

    def test_parameter_round_trip_returns_exact_positions(self):
        doc, ring = self._ring_doc()
        before = [(w.key, w.name, w.x, w.y) for w in doc.evaluate()]
        ring.ring_diameter_mm = 17.25
        ring.start_angle_deg = 37.0
        ring.ring_diameter_mm = 10.0
        ring.start_angle_deg = 0.0
        self.assertEqual(before,
                         [(w.key, w.name, w.x, w.y) for w in doc.evaluate()])

    def test_grid_edit_preserves_surviving_member_payloads(self):
        doc = PlateDocument.new_plate("grid test")
        grid = doc.add_grid(0.0, 0.0, rows=4, cols=6, pitch_x_mm=19.3,
                            pitch_y_mm=19.3)
        doc.override(grid, "g1_2").rim_height_mm = 4.25
        grid.cols = 8
        got = {w.key: w for w in doc.evaluate()}
        self.assertEqual(4.25, got[(grid.id, "g1_2")].rim_height_mm)
        self.assertEqual(32, len(got))

    def test_suppressing_a_member_removes_only_that_well(self):
        doc, ring = self._ring_doc()
        doc.override(ring, "r0").suppressed = True
        keys = {w.key for w in doc.evaluate()}
        self.assertNotIn((ring.id, "r0"), keys)
        self.assertEqual(5, len(keys))
        self.assertEqual(6, len(doc.evaluate(include_suppressed=True)))


# ═══════════════════════════════════════════════════════════════════
# Ring / grid geometry, with the frame stated
# ═══════════════════════════════════════════════════════════════════

class TestPatternGeometry(unittest.TestCase):
    def test_ring_zero_degrees_is_plus_y(self):
        """0° = +Y matches RosetteInsert's polar convention, which the legacy
        rosette code already used. Storage is Y-DOWN, so +Y is toward the
        bottom of the plate."""
        doc = PlateDocument.new_plate("t")
        ring = doc.add_ring(0.0, 0.0, count=4, ring_diameter_mm=10.0)
        pos = {w.key[1]: (round(w.x, 9), round(w.y, 9)) for w in doc.evaluate()}
        self.assertEqual((0.0, 5.0), pos["r0"])
        self.assertEqual((5.0, 0.0), pos["r1"])
        self.assertEqual((0.0, -5.0), pos["r2"])
        self.assertEqual((-5.0, 0.0), pos["r3"])

    def test_ring_direction_reverses_the_sweep(self):
        doc = PlateDocument.new_plate("t")
        ring = doc.add_ring(0.0, 0.0, count=4, ring_diameter_mm=10.0)
        ring.direction = -1
        pos = {w.key[1]: (round(w.x, 9), round(w.y, 9)) for w in doc.evaluate()}
        self.assertEqual((-5.0, 0.0), pos["r1"])

    def test_ring_centre_well_is_first_and_at_the_anchor(self):
        doc = PlateDocument.new_plate("t")
        ring = doc.add_ring(3.0, 4.0, count=3, ring_diameter_mm=6.0,
                            center_well=True)
        wells = doc.evaluate()
        self.assertEqual("c", wells[0].key[1])
        self.assertEqual((3.0, 4.0), (wells[0].x, wells[0].y))
        self.assertEqual("a", wells[0].name)     # letters: centre lands first

    def test_ring_arc_spreads_endpoints_inclusively(self):
        doc = PlateDocument.new_plate("t")
        ring = doc.add_ring(0.0, 0.0, count=3, ring_diameter_mm=10.0)
        ring.sweep_deg = 90.0
        pos = {w.key[1]: (round(w.x, 6), round(w.y, 6)) for w in doc.evaluate()}
        self.assertEqual((0.0, 5.0), pos["r0"])
        self.assertEqual((5.0, 0.0), pos["r2"])

    def test_grid_rows_march_down_the_plate(self):
        doc = PlateDocument.new_plate("t")
        grid = doc.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0,
                            pitch_y_mm=9.0)
        pos = {w.name: (w.x, w.y) for w in doc.evaluate()}
        self.assertEqual((0.0, 0.0), pos["A1"])
        self.assertEqual((9.0, 0.0), pos["A2"])
        self.assertEqual((0.0, 9.0), pos["B1"])   # +Y = down = next row

    def test_grid_rotation_is_about_the_seed(self):
        doc = PlateDocument.new_plate("t")
        grid = doc.add_grid(0.0, 0.0, rows=1, cols=2, pitch_x_mm=10.0)
        grid.rotation_deg = 90.0
        pos = {w.name: (round(w.x, 9), round(w.y, 9)) for w in doc.evaluate()}
        self.assertEqual((0.0, 0.0), pos["A1"])
        self.assertEqual((0.0, 10.0), pos["A2"])

    def test_grid_direction_flips(self):
        doc = PlateDocument.new_plate("t")
        grid = doc.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0,
                            pitch_y_mm=9.0)
        grid.col_dir = -1
        grid.row_dir = -1
        pos = {w.name: (w.x, w.y) for w in doc.evaluate()}
        self.assertEqual((-9.0, 0.0), pos["A2"])
        self.assertEqual((0.0, -9.0), pos["B1"])

    def test_rotation_round_trips(self):
        for deg in (30.0, 90.0, 137.0, 180.0, 359.0):
            with self.subTest(deg=deg):
                doc = PlateDocument.new_plate("t")
                grid = doc.add_grid(0.0, 0.0, rows=2, cols=3, pitch_x_mm=9.0)
                base = [(w.x, w.y) for w in doc.evaluate()]
                grid.rotation_deg = deg
                grid.rotation_deg = 0.0
                for (bx, by), w in zip(base, doc.evaluate()):
                    self.assertAlmostEqual(bx, w.x, places=9)
                    self.assertAlmostEqual(by, w.y, places=9)

    def test_naming_schemes(self):
        self.assertEqual("A1", ansi_label(0, 0))
        self.assertEqual("H12", ansi_label(7, 11))
        self.assertEqual("a", letter_label(0))
        self.assertEqual("z", letter_label(25))
        self.assertEqual("aa", letter_label(26))


# ═══════════════════════════════════════════════════════════════════
# Boundary + display transform
# ═══════════════════════════════════════════════════════════════════

class TestBoundaryAndDisplay(unittest.TestCase):
    def test_extent_is_anchored_by_the_a1_placement(self):
        b = PlateBoundary(width_mm=127.76, height_mm=85.48,
                          a1_x_mm=14.38, a1_y_mm=11.24)
        x0, y0, x1, y1 = b.extent_a1()
        self.assertAlmostEqual(-14.38, x0)
        self.assertAlmostEqual(-11.24, y0)
        self.assertAlmostEqual(113.38, x1)
        self.assertAlmostEqual(74.24, y1)

    def test_all_four_edges_and_both_centrelines_resolve(self):
        b = PlateBoundary()
        for datum in ("edge_left", "edge_right", "edge_top", "edge_bottom",
                      "axis_v", "axis_h", "origin"):
            self.assertIsNotNone(b.datum_value(datum), datum)
        self.assertIsNone(b.datum_value("bore_wall"))
        self.assertIsNotNone(
            PlateBoundary(kind=BoundaryKind.CIRCLE,
                          radius_mm=7.8).datum_value("bore_wall"))

    def test_bottom_left_origin_reports_positive_y_upward(self):
        doc = PlateDocument.from_standard_format(24)
        doc.boundary.origin_ref = OriginRef.BOTTOM_LEFT
        # A1 is at storage (0,0); the bottom edge is below it in a Y-down frame.
        _, y1 = doc.boundary.extent_a1()[3], doc.boundary.extent_a1()[3]
        dx, dy = doc.to_display(0.0, 0.0)
        self.assertAlmostEqual(doc.boundary.a1_x_mm, dx)
        self.assertGreater(dy, 0.0)      # A1 sits ABOVE the bottom edge
        self.assertAlmostEqual(
            doc.boundary.height_mm - doc.boundary.a1_y_mm, dy)

    def test_display_round_trips_for_every_origin(self):
        doc = PlateDocument.from_standard_format(24)
        for origin in ALL_ORIGINS:
            doc.boundary.origin_ref = origin
            for pt in ((0.0, 0.0), (37.5, -12.25), (-9.0, 63.0)):
                back = doc.from_display(*doc.to_display(*pt))
                self.assertAlmostEqual(pt[0], back[0], places=9)
                self.assertAlmostEqual(pt[1], back[1], places=9)

    def test_centre_origin_puts_the_plate_middle_at_zero(self):
        doc = PlateDocument.from_standard_format(24)
        doc.boundary.origin_ref = OriginRef.CENTER
        x0, y0, x1, y1 = doc.boundary.extent_a1()
        dx, dy = doc.to_display((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        self.assertAlmostEqual(0.0, dx, places=9)
        self.assertAlmostEqual(0.0, dy, places=9)

    def test_rosette_boundary_is_a_centred_bore(self):
        doc = PlateDocument.new_rosette(bore_diameter_mm=15.6)
        self.assertTrue(doc.boundary.is_circle())
        self.assertEqual(OriginRef.CENTER, doc.boundary.origin_ref)
        self.assertAlmostEqual(7.8, doc.boundary.radius_mm)
        self.assertTrue(doc.boundary.contains(0.0, 7.0, 0.5))
        self.assertFalse(doc.boundary.contains(0.0, 7.6, 0.5))


# ═══════════════════════════════════════════════════════════════════
# Rosette placement + flattening
# ═══════════════════════════════════════════════════════════════════

class TestRosettePlacement(unittest.TestCase):
    def _library(self):
        ros = PlateDocument.new_rosette(bore_diameter_mm=15.6, name="Ring of 3")
        ros.add_ring(0.0, 0.0, count=3, ring_diameter_mm=6.0,
                     style=WellStyle(diameter_mm=1.5, well_depth_mm=20.0,
                                     rim_height_mm=3.0, ink_z_mm=1.0),
                     naming=NamingSpec(scheme=NamingScheme.LETTERS))
        return {ros.meta.id: ros}, ros

    def test_placement_flattens_to_the_dotted_protocol(self):
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        grid = doc.patterns()[0]
        doc.place_rosette((grid.id, "g0_1"), ros.meta.id,
                          rosette_name=ros.meta.name)

        plate = doc.compile(loader=lib.get)
        names = plate.well_names
        self.assertNotIn("A2", names)
        for sub in ("A2.a", "A2.b", "A2.c"):
            self.assertIn(sub, names)
            info = plate.get_well_info(sub)
            self.assertTrue(info.is_subwell)
            self.assertEqual("A2", info.parent_well)
            self.assertEqual(3.0, info.rim_height_mm)
            self.assertEqual(1.0, info.ink_z_mm)
        self.assertEqual(24 - 1 + 3, len(names))

    def test_subwells_inherit_the_parent_row_and_col(self):
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        grid = doc.patterns()[0]
        doc.place_rosette((grid.id, "g1_2"), ros.meta.id)
        plate = doc.compile(loader=lib.get)
        parent = plate.get_well_info("B3.a")
        self.assertEqual(1, parent.row)
        self.assertEqual(2, parent.col)

    def test_subwell_lookup_is_case_insensitive(self):
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        doc.place_rosette((doc.patterns()[0].id, "g0_0"), ros.meta.id)
        plate = doc.compile(loader=lib.get)
        self.assertEqual(plate.get_well_position("A1.A"),
                         plate.get_well_position("a1.a"))

    def test_rotation_turns_the_subwells_about_the_parent(self):
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        grid = doc.patterns()[0]
        doc.place_rosette((grid.id, "g0_0"), ros.meta.id)
        flat = doc.compile(loader=lib.get)
        doc.place_rosette((grid.id, "g0_0"), ros.meta.id, rotation_deg=90.0)
        turned = doc.compile(loader=lib.get)

        bx, by = flat.get_well_position("A1.a")
        tx, ty = turned.get_well_position("A1.a")
        # Storage frame is Y-down and the rotation matrix is [c -s; s c].
        self.assertAlmostEqual(bx * math.cos(math.pi / 2)
                               - by * math.sin(math.pi / 2), tx, places=9)
        self.assertAlmostEqual(bx * math.sin(math.pi / 2)
                               + by * math.cos(math.pi / 2), ty, places=9)

    def test_the_same_rosette_in_two_wells_is_independent(self):
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        grid = doc.patterns()[0]
        doc.place_rosette((grid.id, "g0_0"), ros.meta.id)
        doc.place_rosette((grid.id, "g0_1"), ros.meta.id, rotation_deg=45.0)
        plate = doc.compile(loader=lib.get)
        self.assertNotEqual(plate.get_well_position("A1.a"),
                            plate.get_well_position("A2.a"))

    def test_editing_the_rosette_updates_every_plate_using_it(self):
        """The operator chose a LIVE link over a frozen copy."""
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        doc.place_rosette((doc.patterns()[0].id, "g0_0"), ros.meta.id)
        before = doc.compile(loader=lib.get).get_well_position("A1.a")

        ros.patterns()[0].ring_diameter_mm = 10.0     # library edit
        after = doc.compile(loader=lib.get).get_well_position("A1.a")
        self.assertNotEqual(before, after)

    def test_a_rosette_edit_changes_the_calibration_fingerprint(self):
        """...which is what makes the live link safe: a changed rosette must
        force a re-teach rather than drive to stale taught positions."""
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        doc.place_rosette((doc.patterns()[0].id, "g0_0"), ros.meta.id)

        before = doc.rosette_revision_fingerprint(lib.get)
        ros.meta.rev += 1                              # a save bumps rev
        after = doc.rosette_revision_fingerprint(lib.get)
        self.assertNotEqual(before, after)

    def test_a_missing_rosette_compiles_as_an_ordinary_well_and_is_reported(self):
        doc = PlateDocument.from_standard_format(24)
        doc.place_rosette((doc.patterns()[0].id, "g0_0"), "ros_gone",
                          rosette_name="Vanished")
        plate = doc.compile(loader=lambda _rid: None)
        self.assertIn("A1", plate.well_names)          # not silently dropped
        self.assertEqual(24, len(plate.well_names))
        problems = doc.validate(loader=lambda _rid: None)
        self.assertTrue(any("Vanished" in p for p in problems), problems)

    def test_clear_rosette(self):
        lib, ros = self._library()
        doc = PlateDocument.from_standard_format(24)
        key = (doc.patterns()[0].id, "g0_0")
        doc.place_rosette(key, ros.meta.id)
        self.assertTrue(doc.clear_rosette(key))
        self.assertFalse(doc.clear_rosette(key))
        self.assertIn("A1", doc.compile(loader=lib.get).well_names)

    def test_a_dot_in_a_subwell_name_is_refused(self):
        lib, ros = self._library()
        ros.add_well(0.0, 0.0, "bad.name")
        doc = PlateDocument.from_standard_format(24)
        doc.place_rosette((doc.patterns()[0].id, "g0_0"), ros.meta.id)
        with self.assertRaises(ValueError):
            doc.compile(loader=lib.get)


# ═══════════════════════════════════════════════════════════════════
# Serialization
# ═══════════════════════════════════════════════════════════════════

class TestSerialization(unittest.TestCase):
    def test_round_trip_preserves_everything(self):
        doc = PlateDocument.from_standard_format(24)
        grid = doc.patterns()[0]
        doc.override(grid, "g1_1").rim_height_mm = 2.5
        doc.place_rosette((grid.id, "g0_2"), "ros_x", rotation_deg=15.0)
        doc.add_constraint("distance_to_datum", [(grid.id, "g0_0")],
                           value=17.05, datum="edge_left")
        blob = doc.to_dict()
        back = PlateDocument.from_dict(blob)
        self.assertEqual(blob, back.to_dict())

    def test_a_384_well_plate_is_tiny(self):
        """Only the pattern is stored, not 384 wells plus 384 points."""
        import json
        blob = json.dumps(PlateDocument.from_standard_format(384).to_dict())
        self.assertLess(len(blob), 4096, f"{len(blob)} bytes")
        self.assertEqual(384, len(PlateDocument.from_standard_format(384)
                                  .evaluate()))

    def test_legacy_file_is_refused_by_name(self):
        with self.assertRaises(LegacySchemaError):
            PlateDocument.from_dict({"schema_version": "1.0", "name": "old"})

    def test_a_newer_schema_is_refused_rather_than_silently_truncated(self):
        blob = PlateDocument.from_standard_format(6).to_dict()
        blob["schema_version"] = 99
        with self.assertRaises(FutureSchemaError):
            PlateDocument.from_dict(blob)

    def test_save_is_atomic_and_reloads(self):
        with tempfile.TemporaryDirectory() as d:
            doc = PlateDocument.from_standard_format(6)
            p = doc.save(Path(d) / f"{doc.meta.id}.json")
            self.assertTrue(p.exists())
            self.assertEqual([], list(Path(d).glob("*.tmp")))
            self.assertEqual(doc.to_dict(), PlateDocument.load(p).to_dict())

    def test_optional_fields_are_omitted(self):
        blob = PlateDocument.from_standard_format(24).to_dict()
        grid = next(e for e in blob["entities"] if e["type"] == "GridPattern")
        for absent in ("overrides", "rotation_deg", "col_dir", "row_dir",
                       "stagger_x_mm"):
            self.assertNotIn(absent, grid)

    def test_a_hand_written_ghost_cannot_be_loaded(self):
        """`drag_ghost` is not a document kind at all in v7.12."""
        blob = PlateDocument.from_standard_format(6).to_dict()
        blob["constraints"].append({"id": -1, "kind": "drag_ghost",
                                    "refs": [[1, ""]], "weight": 1000.0})
        back = PlateDocument.from_dict(blob)
        self.assertEqual([], [c for c in back.constraints
                              if c.kind == "drag_ghost"])


# ═══════════════════════════════════════════════════════════════════
# Identity
# ═══════════════════════════════════════════════════════════════════

class TestIdentity(unittest.TestCase):
    def test_ids_are_unique_and_store_key_safe(self):
        ids = {new_doc_id("plate") for _ in range(500)}
        self.assertEqual(500, len(ids))
        for i in ids:
            self.assertTrue(is_store_key_safe(i), i)

    def test_distinct_ids_stay_distinct_under_the_store_sanitiser(self):
        """Every per-plate store maps its key through
        ``re.sub(r'[^A-Za-z0-9_.-]', '_', key)``, which is many-to-one. Two ids
        colliding there would make one plate overwrite another's mosaic."""
        import re
        safe = lambda k: re.sub(r"[^A-Za-z0-9_.-]", "_", k)
        ids = [new_doc_id("plate") for _ in range(300)]
        self.assertEqual(len(set(ids)), len({safe(i) for i in ids}))

    def test_renaming_changes_no_identity(self):
        doc = PlateDocument.new_plate("original")
        before = doc.meta.id
        doc.meta.name = "a completely different name"
        self.assertEqual(before, doc.meta.id)
        self.assertEqual(before, PlateDocument.from_dict(doc.to_dict()).meta.id)

    def test_rosette_and_plate_ids_are_distinguishable(self):
        self.assertTrue(new_doc_id("plate").startswith("plt_"))
        self.assertTrue(new_doc_id("rosette").startswith("ros_"))


# ═══════════════════════════════════════════════════════════════════
# Validation
# ═══════════════════════════════════════════════════════════════════

class TestValidation(unittest.TestCase):
    def test_a_standard_plate_is_clean(self):
        for fmt in ALL_FORMATS:
            with self.subTest(fmt=fmt):
                self.assertEqual(
                    [], PlateDocument.from_standard_format(fmt).validate())

    def test_duplicate_names_are_reported(self):
        """`WellPlate.from_wells` keys by name.upper(), so a duplicate is
        silently dropped at runtime."""
        doc = PlateDocument.new_plate("dup")
        doc.add_well(0.0, 0.0, "A1")
        doc.add_well(9.0, 0.0, "a1")
        self.assertTrue(any("A1" in p for p in doc.validate()))

    def test_a_well_outside_the_boundary_is_reported(self):
        doc = PlateDocument.new_plate("oob")
        doc.add_well(500.0, 0.0, "A1")
        self.assertTrue(any("outside" in p for p in doc.validate()))

    def test_empty_design_cannot_compile(self):
        with self.assertRaises(ValueError):
            PlateDocument.new_plate("empty").compile()

    def test_a_dot_in_a_well_name_is_reported(self):
        doc = PlateDocument.new_plate("dot")
        doc.add_well(0.0, 0.0, "A1.a")
        self.assertTrue(any("reserved" in p for p in doc.validate()))


# ═══════════════════════════════════════════════════════════════════
# Row/column assignment
# ═══════════════════════════════════════════════════════════════════

class TestRowColAssignment(unittest.TestCase):
    def test_arbitrary_layout_clusters_into_rows_and_columns(self):
        doc = PlateDocument.new_plate("scatter")
        doc.add_well(0.0, 0.0, "A1")
        doc.add_well(10.1, 0.2, "A2")
        doc.add_well(0.15, 9.9, "B1")
        doc.add_well(10.0, 10.1, "B2")
        plate = doc.compile()
        self.assertEqual((0, 0), (plate.get_well_info("A1").row,
                                  plate.get_well_info("A1").col))
        self.assertEqual((1, 1), (plate.get_well_info("B2").row,
                                  plate.get_well_info("B2").col))

    def test_declared_indices_are_used_for_a_plain_grid(self):
        doc = PlateDocument.from_standard_format(96)
        plate = doc.compile()
        self.assertEqual((7, 11), (plate.get_well_info("H12").row,
                                   plate.get_well_info("H12").col))


class TestMemberNaming(unittest.TestCase):
    """Operator: *"when i put multiple grids or rings, the names of the wells
    are the same and there is no way to change that."*

    Every pattern used to be born with the same default spec, so the second
    grid re-emitted ``A1…``. That is not cosmetic — ``WellPlate.from_wells``
    keys a dict by ``name.upper()``, so the duplicates are silently DROPPED
    and most of the second pattern vanishes from the compiled plate.
    """

    # ── the default must not have changed for a single pattern ────

    def test_a_lone_grid_is_named_exactly_as_before(self):
        d = PlateDocument.new_plate("One")
        g = d.add_grid(0.0, 0.0, rows=2, cols=3,
                       pitch_x_mm=9.0, pitch_y_mm=9.0)
        self.assertEqual(["A1", "A2", "A3", "B1", "B2", "B3"],
                         d.feature_member_names(g))

    def test_a_lone_ring_is_named_exactly_as_before(self):
        d = PlateDocument.new_rosette()
        r = d.add_ring(0.0, 0.0, count=4, ring_diameter_mm=6.0)
        self.assertEqual(["a", "b", "c", "d"], d.feature_member_names(r))

    def test_standard_formats_are_untouched(self):
        for n in (6, 24, 96, 384):
            plate = PlateDocument.from_standard_format(n).compile()
            ref = WellPlate.from_format(n)
            self.assertEqual(sorted(w.name for w in ref.get_all_wells()),
                             sorted(w.name for w in plate.get_all_wells()),
                             f"{n}-well naming changed")

    def test_numbers_and_letters_defaults_are_byte_identical(self):
        """The generalisation added prefix/offset support to LETTERS and
        NUMBERS; at their defaults the output must be unchanged."""
        d = PlateDocument.new_plate("S")
        r = d.add_ring(0.0, 0.0, count=3, ring_diameter_mm=6.0,
                       naming=NamingSpec(scheme=NamingScheme.NUMBERS))
        self.assertEqual(["1", "2", "3"], d.feature_member_names(r))
        r2 = d.add_ring(40.0, 0.0, count=3, ring_diameter_mm=6.0,
                        naming=NamingSpec(scheme=NamingScheme.LETTERS))
        self.assertEqual(["a", "b", "c"], d.feature_member_names(r2))

    def test_every_scheme_honours_the_prefix_and_offset(self):
        """Grid-with-LETTERS and grid-with-NUMBERS are separate branches from
        the ring ones — a mutation reverting only the grid branch survived
        until this existed."""
        d = PlateDocument.new_plate("Schemes")
        cases = {
            NamingScheme.ANSI: ["XB2", "XB3"],
            NamingScheme.LETTERS: ["Xb", "Xc"],
            NamingScheme.NUMBERS: ["X2", "X3"],
        }
        for scheme, expect in cases.items():
            g = d.add_grid(0.0, 0.0, rows=1, cols=2,
                           pitch_x_mm=9.0, pitch_y_mm=9.0,
                           naming=NamingSpec(scheme=scheme, prefix="X",
                                             start_row=1, start_col=2))
            self.assertEqual(expect, d.feature_member_names(g), str(scheme))
            r = d.add_ring(0.0, 0.0, count=2, ring_diameter_mm=6.0,
                           naming=NamingSpec(scheme=scheme, prefix="X",
                                             start_row=1, start_col=2))
            want = ["XB2", "XB3"] if scheme == NamingScheme.ANSI else expect
            self.assertEqual(want, d.feature_member_names(r), str(scheme))

    # ── the actual complaint ──────────────────────────────────────

    def test_two_grids_do_not_collide(self):
        d = PlateDocument.new_plate("Two")
        d.add_grid(0.0, 0.0, rows=4, cols=6, pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = d.add_grid(70.0, 0.0, rows=2, cols=3,
                        pitch_x_mm=9.0, pitch_y_mm=9.0)
        self.assertEqual(["E1", "E2", "E3", "F1", "F2", "F3"],
                         d.feature_member_names(g2))
        names = [w.name.upper() for w in d.evaluate()]
        self.assertEqual(len(names), len(set(names)))

    def test_two_rings_do_not_collide(self):
        d = PlateDocument.new_plate("Rings")
        d.add_ring(20.0, 20.0, count=6, ring_diameter_mm=8.0)
        r2 = d.add_ring(60.0, 20.0, count=4, ring_diameter_mm=8.0)
        self.assertEqual(["g", "h", "i", "j"], d.feature_member_names(r2))

    def test_no_well_is_lost_in_the_compiled_plate(self):
        """The failure that made this more than cosmetic."""
        d = PlateDocument.new_plate("Mixed")
        d.add_grid(0.0, 0.0, rows=3, cols=4, pitch_x_mm=9.0, pitch_y_mm=9.0)
        d.add_grid(60.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0, pitch_y_mm=9.0)
        d.add_ring(30.0, 60.0, count=6, ring_diameter_mm=10.0)
        expected = 3 * 4 + 2 * 2 + 6
        self.assertEqual(expected, len(d.evaluate()))
        self.assertEqual(expected, len(d.compile().get_all_wells()))
        self.assertEqual([], d.validate())

    def test_five_patterns_all_stay_unique(self):
        d = PlateDocument.new_plate("Many")
        for i in range(5):
            d.add_grid(i * 30.0, 0.0, rows=2, cols=2,
                       pitch_x_mm=9.0, pitch_y_mm=9.0)
        names = [w.name.upper() for w in d.evaluate()]
        self.assertEqual(20, len(names))
        self.assertEqual(20, len(set(names)))

    # ── the operator's escape hatches ─────────────────────────────

    def test_a_prefix_separates_two_patterns(self):
        d = PlateDocument.new_plate("Prefixed")
        d.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = d.add_grid(60.0, 0.0, rows=2, cols=2,
                        pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2.naming.start_row = 0            # deliberately back onto grid 1
        g2.naming.prefix = "L"
        self.assertEqual(["LA1", "LA2", "LB1", "LB2"],
                         d.feature_member_names(g2))
        self.assertEqual([], d.validate())

    def test_autoname_repairs_a_hand_made_collision(self):
        d = PlateDocument.new_plate("Repair")
        d.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = d.add_grid(60.0, 0.0, rows=2, cols=2,
                        pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2.naming.start_row = 0
        self.assertTrue(d.validate(), "the collision should be reported")
        self.assertTrue(d.autoname_feature(g2))
        self.assertEqual([], d.validate())

    def test_an_explicit_naming_spec_is_respected_verbatim(self):
        """Auto-numbering must not silently override a deliberate choice."""
        d = PlateDocument.new_plate("Explicit")
        d.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = d.add_grid(60.0, 0.0, rows=1, cols=2,
                        pitch_x_mm=9.0, pitch_y_mm=9.0,
                        naming=NamingSpec(scheme=NamingScheme.ANSI))
        self.assertEqual(["A1", "A2"], d.feature_member_names(g2))

    def test_member_overrides_survive_and_are_counted(self):
        d = PlateDocument.new_plate("Override")
        g = d.add_grid(0.0, 0.0, rows=1, cols=2,
                       pitch_x_mm=9.0, pitch_y_mm=9.0)
        g.overrides["g0_0"] = MemberOverride(name="INLET")
        self.assertEqual(["INLET", "A2"], d.feature_member_names(g))
        g2 = d.add_grid(60.0, 0.0, rows=1, cols=1, pitch_x_mm=9.0,
                        pitch_y_mm=9.0)
        self.assertNotIn(d.feature_member_names(g2)[0], ("INLET", "A2"))

    def test_a_renamed_member_is_not_treated_as_free(self):
        """``names_in_use`` must see override names, or a later pattern could
        be auto-numbered straight onto one."""
        d = PlateDocument.new_plate("Seen")
        g = d.add_grid(0.0, 0.0, rows=1, cols=1,
                       pitch_x_mm=9.0, pitch_y_mm=9.0)
        g.overrides["g0_0"] = MemberOverride(name="Z9")
        self.assertIn("Z9", d.names_in_use())
        self.assertNotIn("Z9", d.names_in_use(exclude=g.id))

    def test_the_unique_prefix_fallback_actually_works(self):
        """MANUAL names come from the member key, so shifting the offsets can
        never break a tie — the fallback is the only thing that can, and it is
        the last line of defence against silently dropped wells."""
        d = PlateDocument.new_plate("Stuck")
        spec = NamingSpec(scheme=NamingScheme.MANUAL)
        g1 = d.add_grid(0.0, 0.0, rows=1, cols=2, pitch_x_mm=9.0,
                        pitch_y_mm=9.0, naming=spec)
        g2 = d.add_grid(60.0, 0.0, rows=1, cols=2, pitch_x_mm=9.0,
                        pitch_y_mm=9.0,
                        naming=NamingSpec(scheme=NamingScheme.MANUAL))
        self.assertEqual(d.feature_member_names(g1),
                         d.feature_member_names(g2),
                         "the fixture failed to create a real deadlock")
        self.assertTrue(d.autoname_feature(g2, max_tries=5))
        self.assertEqual([], d.validate())
        self.assertEqual(4, len(d.compile().get_all_wells()))

    def test_naming_round_trips_through_save(self):
        d = PlateDocument.new_plate("RT")
        d.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = d.add_grid(60.0, 0.0, rows=2, cols=2,
                        pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2.naming.prefix = "L"
        before = [w.name for w in d.evaluate()]
        again = PlateDocument.from_dict(d.to_dict())
        self.assertEqual(before, [w.name for w in again.evaluate()])


if __name__ == "__main__":
    unittest.main()
