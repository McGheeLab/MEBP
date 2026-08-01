"""
v7.5.x — per-well print Z from the plate-bottom tilt plane.

The plate bottom is flat but tilted (~1.15 mm across a 24-well plate on ME3B V1),
so a plate-wide print Z prints too high at some wells and into the glass at
others. A job may now carry a stamped tilt plane and the operator's intended
height above the plate bottom, and each well's Z is derived from that well's LOCAL
plate bottom.

The load-bearing test here is BYTE-IDENTITY: with no stamped plane the emitted
command plan must be exactly what it was before, so this can land without
touching any existing print. Everything else is additive.
"""

import unittest

from SupportClasses.PlateZPlane import job_plane_z_zref_mm
from SupportClasses.PrintManager import (
    CommandType,
    PrintSettings,
    build_well_plate_job,
    well_print_z_zref_mm,
)


# Three wells in zero-ref mm. A1 is the plane anchor.
WELLS = [("A1", 100.0, 60.0), ("A6", 55.0, 60.0), ("D6", 55.0, 2.0)]
PATH = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

# Tilt stamped in the JOB form: zero-ref mm XY, mm/mm slopes.
PLANE = {"x0_mm": 100.0, "y0_mm": 60.0, "z0_zref_mm": 0.09,
         "sx_mm_per_mm": 0.0005, "sy_mm_per_mm": -0.001,
         "plane_id": "test"}


def _settings(**kw):
    s = PrintSettings()
    s.print_z_height = 0.29        # = plate bottom 0.09 + 0.20 height, zdir=+1
    s.layer_height = 0.1
    s.num_layers = 1
    s.travel_z_height = 5.0
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def _z_moves(job):
    return [c.params.get("z") for c in job.commands
            if c.type == CommandType.MOVE_Z]


def _plan_signature(job):
    """Everything about the plan that a byte-identity check should cover."""
    return [(c.type, tuple(sorted((c.params or {}).items())), c.label)
            for c in job.commands]


class _ZAssert:
    """Mixin: compare Z lists with a tolerance (plain == trips on float repr)."""

    def assertZs(self, got, expected, places=9):
        self.assertEqual(len(got), len(expected),
                         f"expected {len(expected)} MOVE_Z, got {len(got)}")
        for i, (g, e) in enumerate(zip(got, expected)):
            self.assertAlmostEqual(g, e, places=places,
                                   msg=f"MOVE_Z[{i}]")


class TestByteIdentityWithoutAPlane(_ZAssert, unittest.TestCase):
    """No stamped plane ⇒ the plan is exactly the legacy plan."""

    def test_plan_signature_unchanged_single_layer(self):
        s = _settings()
        job = build_well_plate_job(WELLS, PATH, s)
        # Every MOVE_Z is the plate-wide print Z.
        self.assertZs(_z_moves(job), [0.29, 0.29, 0.29])
        for c in job.commands:
            self.assertNotIn("floor_x_mm", (c.params or {}))
            self.assertNotIn("floor_y_mm", (c.params or {}))

    def test_plan_signature_unchanged_multi_layer(self):
        s = _settings(num_layers=3)
        job = build_well_plate_job(WELLS, PATH, s)
        self.assertZs(_z_moves(job), [0.29, 0.29, 0.29, 0.39, 0.39, 0.39, 0.49, 0.49, 0.49])

    def test_plan_signature_unchanged_negative_polarity(self):
        s = _settings(z_up_sign=-1.0, num_layers=2)
        job = build_well_plate_job(WELLS, PATH, s)
        self.assertZs(_z_moves(job), [0.29, 0.29, 0.29, 0.19, 0.19, 0.19])

    def test_multi_segment_hop_z_unchanged(self):
        s = _settings(intra_well_hop_z_mm=1.0)
        job = build_well_plate_job(WELLS, PATH, s,
                                   path_segments=[PATH, PATH])
        hops = [c.params.get("hop_z") for c in job.commands
                if c.type == CommandType.MOVE_XY and "hop_z" in (c.params or {})]
        self.assertTrue(hops)
        for h in hops:
            self.assertAlmostEqual(h, 0.29 + 1.0, places=9)

    def test_plane_without_height_is_ignored(self):
        """Both stamps are required; a plane alone must not shift anything."""
        s = _settings(plate_z_plane_zref_mm=PLANE)
        job = build_well_plate_job(WELLS, PATH, s)
        self.assertZs(_z_moves(job), [0.29, 0.29, 0.29])

    def test_height_without_plane_is_ignored(self):
        s = _settings(print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        self.assertZs(_z_moves(job), [0.29, 0.29, 0.29])

    def test_signature_identical_with_and_without_unset_fields(self):
        """Defaulted new fields must not perturb the plan at all."""
        a = build_well_plate_job(WELLS, PATH, _settings())
        s = _settings(plate_z_plane_zref_mm=None,
                      print_height_above_bottom_mm=None)
        b = build_well_plate_job(WELLS, PATH, s)
        self.assertEqual(_plan_signature(a), _plan_signature(b))


class TestPerWellZWithAPlane(_ZAssert, unittest.TestCase):
    def test_anchor_well_keeps_the_legacy_z(self):
        """The anchor is where the scalar was taught ⇒ Z is unchanged there."""
        s = _settings(plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        self.assertAlmostEqual(_z_moves(job)[0], 0.29, places=9)

    def test_far_well_tracks_the_tilt(self):
        s = _settings(plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        zs = _z_moves(job)
        # D6 is -45 mm in X and -58 mm in Y from the anchor.
        local_bottom = 0.09 + 0.0005 * -45.0 + (-0.001) * -58.0
        self.assertAlmostEqual(zs[2], local_bottom + 0.2, places=9)
        # And it is a real difference, not noise.
        self.assertGreater(abs(zs[2] - zs[0]), 0.02)

    def test_standoff_is_constant_across_wells(self):
        """The POINT of the feature: same gap to the glass at every well."""
        s = _settings(plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        for (name, wx, wy), z in zip(WELLS, _z_moves(job)):
            local_bottom = job_plane_z_zref_mm(PLANE, wx, wy)
            self.assertAlmostEqual(z - local_bottom, 0.2, places=9, msg=name)

    def test_negative_polarity_standoff_is_constant(self):
        s = _settings(z_up_sign=-1.0, plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        for (name, wx, wy), z in zip(WELLS, _z_moves(job)):
            local_bottom = job_plane_z_zref_mm(PLANE, wx, wy)
            # zdir = -1 ⇒ "above the plate bottom" is a SMALLER zero-ref Z.
            self.assertAlmostEqual(local_bottom - z, 0.2, places=9, msg=name)

    def test_layers_still_step_up_from_the_local_bottom(self):
        s = _settings(num_layers=2, plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        zs = _z_moves(job)
        for i in range(3):
            self.assertAlmostEqual(zs[i + 3] - zs[i], 0.1, places=9)

    def test_hop_z_follows_the_per_well_z(self):
        s = _settings(intra_well_hop_z_mm=1.0, plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s,
                                   path_segments=[PATH, PATH])
        # Pair each hop with the print Z of the same well.
        hops, zs = [], []
        for c in job.commands:
            if c.type == CommandType.MOVE_XY and "hop_z" in (c.params or {}):
                hops.append(c.params["hop_z"])
            elif c.type == CommandType.MOVE_Z:
                zs.append(c.params["z"])
        self.assertEqual(len(hops), len(WELLS))
        for hop, z in zip(hops, zs[::2]):
            self.assertAlmostEqual(hop - z, 1.0, places=9)

    def test_floor_xy_hint_emitted_per_well(self):
        """The clamp gets a deterministic XY instead of a cached position."""
        s = _settings(plate_z_plane_zref_mm=PLANE,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        hints = [(c.params.get("floor_x_mm"), c.params.get("floor_y_mm"))
                 for c in job.commands if c.type == CommandType.MOVE_Z]
        self.assertEqual(hints, [(100.0, 60.0), (55.0, 60.0), (55.0, 2.0)])


class TestHelperAndRobustness(_ZAssert, unittest.TestCase):
    def test_helper_matches_legacy_formula_without_a_plane(self):
        s = _settings(num_layers=1)
        for layer in (0, 1, 2):
            self.assertAlmostEqual(
                well_print_z_zref_mm(s, 55.0, 2.0, layer),
                s.print_z_height + layer * s.layer_height, places=12)

    def test_malformed_plane_falls_back_and_does_not_raise(self):
        """A bad stamp must degrade to the plate-wide Z, never break a print."""
        for bad in ({"x0_mm": 1.0},                       # missing keys
                    {"x0_mm": "a", "y0_mm": 0.0, "z0_zref_mm": 0.0},
                    {"x0_um": 0.0, "y0_um": 0.0, "z0_zref_mm": 0.0}):
            s = _settings(plate_z_plane_zref_mm=bad,
                          print_height_above_bottom_mm=0.2)
            self.assertAlmostEqual(well_print_z_zref_mm(s, 55.0, 2.0, 0),
                                   0.29, places=9)
            job = build_well_plate_job(WELLS, PATH, s)
            self.assertZs(_z_moves(job), [0.29, 0.29, 0.29])

    def test_live_um_plane_is_rejected_as_a_job_stamp(self):
        """Passing the live µm-keyed plane where the mm form belongs must not
        silently produce a 1000×-wrong Z — it degrades to the scalar."""
        um_form = {"x0_um": 100000.0, "y0_um": 60000.0, "z0_zref_mm": 0.09,
                   "sx_mm_per_mm": 0.0005, "sy_mm_per_mm": -0.001}
        s = _settings(plate_z_plane_zref_mm=um_form,
                      print_height_above_bottom_mm=0.2)
        self.assertAlmostEqual(well_print_z_zref_mm(s, 55.0, 2.0, 0), 0.29,
                               places=9)

    def test_zero_tilt_plane_is_the_scalar(self):
        flat = dict(PLANE, sx_mm_per_mm=0.0, sy_mm_per_mm=0.0)
        s = _settings(plate_z_plane_zref_mm=flat,
                      print_height_above_bottom_mm=0.2)
        job = build_well_plate_job(WELLS, PATH, s)
        for z in _z_moves(job):
            self.assertAlmostEqual(z, 0.29, places=9)


if __name__ == "__main__":
    unittest.main()
