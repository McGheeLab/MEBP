"""
v7.10 — the MICROSCOPE bore-offset measurement, composed with the executor.

This is the test the microscope method exists for. Both halves of a sign
convention can be internally consistent and jointly wrong; only the composition
— measure a bore, store it, apply it to the needle, command a target, check
where the tip ends up — proves the bore lands on the cell.

A mis-signed offset is a *right-distance-wrong-way* error: the bore lands the
correct distance from the target on the WRONG SIDE, missing by twice the bore
spacing, and it reads as a calibration problem rather than a bug. So every
assertion here is written to fail loudly on a flip, and
``test_the_flipped_sign_is_twice_the_spacing_away`` pins the magnitude of that
failure explicitly.

THE MODEL BEING SIMULATED — stated so the test cannot silently drift from the
derivation in ``NeedleBoreCalibrationStore``:

    a PLATE feature at pixel P has stage-coordinate label  c = stage + pto(P)

so at stage S bore k, appearing at pixel P_k, sits over the plate point labelled
``S + pto(P_k)``. Everything below is generated from that one rule.
"""

import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from SupportClasses.NeedleBoreCalibrationStore import (
    NeedleBoreCalibrationStore,
    needle_camera_offset_from_click,
    offset_from_centred_positions,
    offset_from_frame_clicks,
    z_offset_from_centred_heights,
)
from SupportClasses.PhysicalModels import NeedleBore, NeedleSpec
from SupportClasses.PickAndPlaceManager import PickPlaceExecutor


# ── fixtures ────────────────────────────────────────────────────────

def _backpack() -> NeedleSpec:
    return NeedleSpec(
        needle_form="backpack",
        bores=[
            NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
            NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P2"),
        ],
    )


def _triple() -> NeedleSpec:
    return NeedleSpec(
        needle_form="triple",
        bores=[
            NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
            NeedleBore(id_um=210, od_um=413, length_mm=50.8, pump_id="P2"),
            NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P3"),
        ],
    )


class _FakeCtrl:
    def __init__(self, z_up_sign=1.0):
        self._sign = z_up_sign
        self.is_zp_connected = True

    def z_up_sign(self):
        return self._sign


def _executor(needle, z_up_sign=1.0) -> PickPlaceExecutor:
    ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
    ex.controller = _FakeCtrl(z_up_sign)
    hw = MagicMock()
    hw.needle = needle
    ex.hw_config = hw
    return ex


class _Target:
    def __init__(self, x_um, y_um):
        self.x_um = float(x_um)
        self.y_um = float(y_um)


class _IsolatedStore(unittest.TestCase):
    """A throwaway store file, so a test never touches the real calibration."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_mic_sign_")
        self.addCleanup(self._tmp.cleanup)

    def store(self) -> NeedleBoreCalibrationStore:
        return NeedleBoreCalibrationStore(Path(self._tmp.name) / "bore.json")


# ── a simulated microscope, generated from the pto contract ─────────

class _Scope:
    """Turns bore label-space positions into pixels and back via a real-shaped
    ``pixel_to_stage_offset``: mirror/flip parity, scale, then CCW rotation.

    ``bore_labels`` are where each bore sits in stage-label space relative to the
    frame centre — i.e. exactly what ``pto(P_k)`` must return for that bore.
    """

    def __init__(self, um_per_px=0.5, rotation_deg=0.0,
                 mirrored=False, flip_y=False, w=1920, h=1080):
        self.um_per_px = float(um_per_px)
        self.theta = math.radians(float(rotation_deg))
        self.mirrored = bool(mirrored)
        self.flip_y = bool(flip_y)
        self.w, self.h = int(w), int(h)

    def pto(self, px, py):
        """Mirror of CameraManager.pixel_to_stage_offset."""
        dx = px - self.w / 2.0
        dy = py - self.h / 2.0
        if self.mirrored:
            dx = -dx
        if self.flip_y:
            dy = -dy
        dx *= self.um_per_px
        dy *= self.um_per_px
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (dx * c - dy * s, dx * s + dy * c)

    def pixel_for(self, label_um):
        """Inverse: the pixel at which a bore with this label offset appears."""
        lx, ly = float(label_um[0]), float(label_um[1])
        c, s = math.cos(-self.theta), math.sin(-self.theta)
        dx, dy = lx * c - ly * s, lx * s + ly * c
        dx /= self.um_per_px
        dy /= self.um_per_px
        if self.mirrored:
            dx = -dx
        if self.flip_y:
            dy = -dy
        return (dx + self.w / 2.0, dy + self.h / 2.0)

    def round_trip_ok(self, label_um):
        got = self.pto(*self.pixel_for(label_um))
        return (math.isclose(got[0], label_um[0], abs_tol=1e-6)
                and math.isclose(got[1], label_um[1], abs_tol=1e-6))


class TestTheSimulatorItself(unittest.TestCase):
    """If the simulated optics are wrong every downstream test is meaningless."""

    def test_pixel_round_trips_through_pto(self):
        for rot in (0, 37.5, 90, 180, 270):
            for mir in (False, True):
                for fy in (False, True):
                    sc = _Scope(rotation_deg=rot, mirrored=mir, flip_y=fy)
                    self.assertTrue(sc.round_trip_ok((320.0, -140.0)),
                                    f"rot={rot} mir={mir} fy={fy}")

    def test_centre_pixel_is_zero_offset(self):
        sc = _Scope()
        self.assertEqual(sc.pto(sc.w / 2.0, sc.h / 2.0), (0.0, 0.0))


# ── the composed test ───────────────────────────────────────────────

class TestMicroscopeSignComposedWithTheExecutor(_IsolatedStore):

    SPACING = (320.0, -140.0)      # bore 1 relative to the datum, label µm

    def _measure(self, store, needle, scope, bore_labels, z_by_bore=None):
        """Click every bore in one frame, then store, exactly as the wizard does."""
        pto = {k: scope.pto(*scope.pixel_for(lab))
               for k, lab in enumerate(bore_labels)}
        store.set_bore(0, (0.0, 0.0), 0.0,
                       stage_um=pto[0],
                       z_user_mm=(z_by_bore or {}).get(0), needle=needle)
        for k in range(1, len(bore_labels)):
            off = offset_from_frame_clicks(pto[0], pto[k])
            dz = 0.0
            if z_by_bore:
                dz = z_offset_from_centred_heights(z_by_bore[0], z_by_bore[k])
            store.set_bore(k, off, dz, stage_um=pto[k],
                           z_user_mm=(z_by_bore or {}).get(k), needle=needle)
        self.assertEqual(store.apply_to_needle(needle), len(bore_labels) - 1)
        return pto

    def test_a_clicked_bore_lands_ON_the_target(self):
        """THE go/no-go. Bore 1 sits SPACING away from the datum in label space.
        Commanding the executor's stage must put bore 1 exactly on the target."""
        needle, store, scope = _backpack(), self.store(), _Scope()
        labels = [(0.0, 0.0), self.SPACING]
        self._measure(store, needle, scope, labels)

        ex = _executor(needle)
        target = _Target(50_000.0, 25_000.0)
        sx, sy = ex._bore_target_xy_um(target, 1)

        # At stage (sx, sy) bore 1 is over the plate point labelled
        # stage + pto(P_1) — which must be the target.
        self.assertAlmostEqual(sx + labels[1][0], target.x_um, places=6)
        self.assertAlmostEqual(sy + labels[1][1], target.y_um, places=6)

    def test_the_datum_bore_is_unaffected(self):
        needle, store, scope = _backpack(), self.store(), _Scope()
        self._measure(store, needle, scope, [(0.0, 0.0), self.SPACING])
        ex = _executor(needle)
        target = _Target(50_000.0, 25_000.0)
        self.assertEqual(ex._bore_target_xy_um(target, 0),
                         (target.x_um, target.y_um))

    def test_the_flipped_sign_is_twice_the_spacing_away(self):
        """MUTATION CHECK. Swap the subtraction order — the earlier draft's
        formula — and the bore misses by 2x the spacing, on the wrong side."""
        needle, store, scope = _backpack(), self.store(), _Scope()
        labels = [(0.0, 0.0), self.SPACING]
        pto = {k: scope.pto(*scope.pixel_for(lab)) for k, lab in enumerate(labels)}

        good = offset_from_frame_clicks(pto[0], pto[1])
        bad = offset_from_frame_clicks(pto[1], pto[0])          # the flip
        self.assertEqual(bad, (-good[0], -good[1]))

        miss = math.hypot(good[0] - bad[0], good[1] - bad[1])
        spacing = math.hypot(*self.SPACING)
        self.assertAlmostEqual(miss, 2 * spacing, places=6)
        self.assertGreater(miss, 600.0, "a flip must be grossly visible")

    def test_it_holds_across_rotation_mirror_and_flip(self):
        """At theta=0 with no flips a sign error can hide, because several wrong
        forms agree there — the exact trap the v7.8 `to_px` bug fell into. Sweep
        the orientations that break the tie."""
        for rot in (0, 90, 180, 270, 37.5):
            for mir in (False, True):
                for fy in (False, True):
                    with self.subTest(rot=rot, mirrored=mir, flip_y=fy):
                        needle, store = _backpack(), self.store()
                        scope = _Scope(rotation_deg=rot, mirrored=mir, flip_y=fy)
                        labels = [(0.0, 0.0), self.SPACING]
                        self._measure(store, needle, scope, labels)
                        ex = _executor(needle)
                        t = _Target(50_000.0, 25_000.0)
                        sx, sy = ex._bore_target_xy_um(t, 1)
                        self.assertAlmostEqual(sx + labels[1][0], t.x_um, places=5)
                        self.assertAlmostEqual(sy + labels[1][1], t.y_um, places=5)

    def test_every_bore_of_a_triple_lands_on_the_target(self):
        needle, store, scope = _triple(), self.store(), _Scope()
        labels = [(0.0, 0.0), (300.0, 0.0), (150.0, 260.0)]
        self._measure(store, needle, scope, labels)
        ex = _executor(needle)
        t = _Target(70_000.0, 40_000.0)
        for k, lab in enumerate(labels):
            sx, sy = ex._bore_target_xy_um(t, k)
            self.assertAlmostEqual(sx + lab[0], t.x_um, places=5, msg=f"bore {k}")
            self.assertAlmostEqual(sy + lab[1], t.y_um, places=5, msg=f"bore {k}")

    def test_the_datum_need_not_be_centred_in_the_frame(self):
        """The datum's own position cancels, so an off-centre park is fine —
        which is what makes the one-frame method practical."""
        needle, store, scope = _backpack(), self.store(), _Scope()
        off_centre = (-880.0, 415.0)
        labels = [off_centre,
                  (off_centre[0] + self.SPACING[0],
                   off_centre[1] + self.SPACING[1])]
        self._measure(store, needle, scope, labels)
        self.assertAlmostEqual(needle.bores[1].offset_um[0], self.SPACING[0], places=5)
        self.assertAlmostEqual(needle.bores[1].offset_um[1], self.SPACING[1], places=5)


class TestBothMethodsAgree(_IsolatedStore):
    """Consistency between the two measurement paths.

    ⚠ HONEST SCOPE. This is a CONSISTENCY check, not an independent validation
    of the sign. It assumes the side-camera premise (a bore sitting at
    label-offset L from the datum is centred by backing the stage off by L) and
    then shows the two formulas agree under it — so the two paths cannot silently
    drift apart, but a shared misconception would not be caught here.

    The load-bearing test is
    ``TestMicroscopeSignComposedWithTheExecutor.test_a_clicked_bore_lands_ON_the_target``,
    which is anchored to the code-verified ``pto`` contract that the pick
    workflow itself uses to build a target from a click — no assumed premise.

    The real cross-validation is on hardware: measure one bore both ways in the
    app and drive every bore to a single target, checking DIRECTION.
    """

    def test_microscope_matches_side_cameras_under_the_shared_premise(self):
        scope = _Scope(rotation_deg=25.0, mirrored=True)
        labels = [(0.0, 0.0), (320.0, -140.0)]
        pto = {k: scope.pto(*scope.pixel_for(l)) for k, l in enumerate(labels)}
        by_scope = offset_from_frame_clicks(pto[0], pto[1])

        s0 = (100_000.0, 50_000.0)
        s1 = (s0[0] - labels[1][0], s0[1] - labels[1][1])   # the assumed premise
        by_cams = offset_from_centred_positions(s0, s1)

        self.assertAlmostEqual(by_scope[0], by_cams[0], places=6)
        self.assertAlmostEqual(by_scope[1], by_cams[1], places=6)


class TestNeedleCameraOffset(_IsolatedStore):
    """The single-bore needle's reason to run the wizard at all."""

    def test_it_is_the_raw_click_with_no_negation(self):
        scope = _Scope()
        p = scope.pto(*scope.pixel_for((-880.0, 415.0)))
        self.assertEqual(needle_camera_offset_from_click(p), p)

    def test_it_puts_the_needle_on_a_clicked_feature(self):
        """Composed with StageController.needle_target_xy_for_feature_um."""
        from SupportClasses.StageController import StageController
        scope = _Scope()
        datum_label = (-880.0, 415.0)
        p0 = scope.pto(*scope.pixel_for(datum_label))

        ctrl = StageController.__new__(StageController)
        ctrl.set_needle_camera_offset_um(*needle_camera_offset_from_click(p0))

        # Operator clicks a cell; its label is the feature position.
        feature = (61_234.0, 22_100.0)
        sx, sy = ctrl.needle_target_xy_for_feature_um(*feature)
        # At that stage the datum bore is over label stage + pto(P_0).
        self.assertAlmostEqual(sx + datum_label[0], feature[0], places=6)
        self.assertAlmostEqual(sy + datum_label[1], feature[1], places=6)

    def test_bore_offset_is_the_difference_of_two_centre_offsets(self):
        """Invariant tying step 3 to step 4: both are built from the same clicks."""
        scope = _Scope(rotation_deg=17.0, flip_y=True)
        labels = [(-880.0, 415.0), (-560.0, 275.0)]
        pto = [scope.pto(*scope.pixel_for(l)) for l in labels]
        c0 = needle_camera_offset_from_click(pto[0])
        c1 = needle_camera_offset_from_click(pto[1])
        off = offset_from_frame_clicks(pto[0], pto[1])
        self.assertAlmostEqual(off[0], c1[0] - c0[0], places=9)
        self.assertAlmostEqual(off[1], c1[1] - c0[1], places=9)


class TestPerBoreZ(_IsolatedStore):
    """BUG B: a real per-bore Z, on both machine polarities."""

    def test_dz_is_the_height_difference(self):
        self.assertAlmostEqual(
            z_offset_from_centred_heights(22.500, 22.548), 0.048, places=9)

    def test_a_longer_bore_gets_a_higher_stage_on_both_polarities(self):
        for sign in (1.0, -1.0):
            with self.subTest(z_up_sign=sign):
                needle, store, scope = _backpack(), self.store(), _Scope()
                labels = [(0.0, 0.0), (320.0, 0.0)]
                pto = {k: scope.pto(*scope.pixel_for(l))
                       for k, l in enumerate(labels)}
                store.set_bore(0, (0.0, 0.0), 0.0, stage_um=pto[0],
                               z_user_mm=22.500, needle=needle)
                store.set_bore(
                    1, offset_from_frame_clicks(pto[0], pto[1]),
                    z_offset_from_centred_heights(22.500, 22.548),
                    stage_um=pto[1], z_user_mm=22.548, needle=needle)
                store.apply_to_needle(needle)
                ex = _executor(needle, z_up_sign=sign)
                base = 10.0
                self.assertAlmostEqual(ex._bore_z_mm(base, 0), base, places=9)
                self.assertAlmostEqual(ex._bore_z_mm(base, 1),
                                       base + sign * 0.048, places=9)

    def test_all_zero_dz_is_exactly_todays_behaviour(self):
        """MUTATION GUARD for BUG B: if a future change reverts to writing dz=0,
        the descend planner silently becomes an identity again. Pin that a
        measured spread is what makes it non-trivial."""
        needle, store, scope = _backpack(), self.store(), _Scope()
        labels = [(0.0, 0.0), (320.0, 0.0)]
        pto = {k: scope.pto(*scope.pixel_for(l)) for k, l in enumerate(labels)}
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=pto[0],
                       z_user_mm=22.500, needle=needle)
        store.set_bore(1, offset_from_frame_clicks(pto[0], pto[1]), 0.0,
                       stage_um=pto[1], z_user_mm=22.500, needle=needle)
        store.apply_to_needle(needle)
        ex = _executor(needle)
        self.assertEqual(ex._descend_z_mm(10.0, 0), 10.0)
        self.assertEqual(needle.max_bore_z_offset_mm, 0.0)

    def test_a_measured_spread_wakes_the_descend_planner(self):
        needle, store, scope = _backpack(), self.store(), _Scope()
        labels = [(0.0, 0.0), (320.0, 0.0)]
        pto = {k: scope.pto(*scope.pixel_for(l)) for k, l in enumerate(labels)}
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=pto[0],
                       z_user_mm=22.500, needle=needle)
        store.set_bore(1, offset_from_frame_clicks(pto[0], pto[1]),
                       z_offset_from_centred_heights(22.500, 22.548),
                       stage_um=pto[1], z_user_mm=22.548, needle=needle)
        store.apply_to_needle(needle)
        ex = _executor(needle)
        # Descending for the DATUM must still clear the longer bore.
        self.assertAlmostEqual(ex._descend_z_mm(10.0, 0), 10.048, places=9)


if __name__ == "__main__":
    unittest.main()
