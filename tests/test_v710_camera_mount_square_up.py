"""v7.10 — square up a camera mount (live rotation alignment aid).

Covers the pure sign/geometry, the Fourier-Mellin-based live estimator, the
ghost overlay on CameraFeedView, the dialog's live-state handling, and the
Stage-0 fix to the mirrored-camera rotation measurement.

THE TEST THAT MATTERS MOST is ``TestClosedLoopSign`` — the ghost and the numeric
readout both derive from ``target_image_rotation_deg``, so they are consistent
BY CONSTRUCTION and would lie together if its mirror condition were backwards.
A ghost/estimator round-trip cannot catch that. The closed-loop test never
mentions phi's sign: it drives two pixel positions through the REAL
``CameraManager.pixel_to_stage_offset`` and asserts the stage offsets match.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2  # noqa: E402

from SupportClasses.CameraRotationTracker import (          # noqa: E402
    MIN_CONF, SQUARE_NOMINALS, TRACK_SIZE, RotationTracker, edge_rgba,
    estimate_rotation, fold_parallel_deg, frame_is_trackable, make_ghost,
    nearest_nominal, nearest_square_rotation, prepare_frame,
    target_image_rotation_deg, wrap_deg,
)

FLIP_COMBOS = ((False, False), (False, True), (True, False), (True, True))
THETAS = (0.0, 12.0, -33.0, 100.0, 170.0, 179.9, -90.0, 45.0)


def textured(n: int = 640, seed: int = 11) -> np.ndarray:
    """A synthetic scene with texture at many scales and orientations."""
    rng = np.random.default_rng(seed)
    img = np.full((n, n), 40, np.uint8)
    for _ in range(80):
        c = (int(rng.integers(25, n - 25)), int(rng.integers(25, n - 25)))
        cv2.circle(img, c, int(rng.integers(6, 26)),
                   int(rng.integers(90, 250)), -1)
    for _ in range(16):
        cv2.line(img, (int(rng.integers(0, n)), int(rng.integers(0, n))),
                 (int(rng.integers(0, n)), int(rng.integers(0, n))),
                 int(rng.integers(120, 235)), 2)
    return cv2.GaussianBlur(img, (3, 3), 0)


# ══════════════════════════════════════════════════════════════════════════
# Pure sign + geometry
# ══════════════════════════════════════════════════════════════════════════

class TestWrapAndNominals(unittest.TestCase):

    def test_wrap_deg_half_open(self):
        self.assertEqual(wrap_deg(180.0), 180.0)
        self.assertEqual(wrap_deg(-180.0), 180.0)
        self.assertAlmostEqual(wrap_deg(190.0), -170.0)
        self.assertAlmostEqual(wrap_deg(-190.0), 170.0)
        self.assertAlmostEqual(wrap_deg(720.0 + 12.0), 12.0)

    def test_fold_parallel_deg(self):
        self.assertAlmostEqual(fold_parallel_deg(179.0), -1.0)
        self.assertAlmostEqual(fold_parallel_deg(-135.0), 45.0)
        self.assertAlmostEqual(fold_parallel_deg(90.0), 90.0)
        self.assertAlmostEqual(fold_parallel_deg(90.1), -89.9)

    def test_nearest_square_is_always_within_45(self):
        for th in np.arange(-180.0, 180.0, 0.7):
            nom, delta = nearest_square_rotation(float(th))
            self.assertIn(nom, SQUARE_NOMINALS)
            self.assertLessEqual(abs(delta), 45.0 + 1e-9)
            self.assertAlmostEqual(wrap_deg(nom + delta), wrap_deg(float(th)),
                                   places=9)

    def test_knife_edge_is_deterministic(self):
        # Exactly 45 keeps nominal 0 (strict <); a hair past flips to 90. This
        # reproduces hardware_setup.nominal_rotation_delta's long-standing
        # behaviour, which the slot-card readout depends on.
        self.assertEqual(nearest_square_rotation(45.0)[0], 0.0)
        self.assertEqual(nearest_square_rotation(45.01)[0], 90.0)
        self.assertEqual(nearest_square_rotation(-45.0)[0], 0.0)
        # 135 is equidistant from 90 and 180; the strict `<` keeps whichever
        # the tuple reaches first, so 90 wins. Either is a legitimate target
        # and the operator can pin one from the combo — what matters is that
        # it never oscillates between them.
        self.assertEqual(nearest_square_rotation(135.0), (90.0, 45.0))
        self.assertEqual(nearest_square_rotation(180.0), (180.0, 0.0))

    def test_hardware_setup_shares_one_implementation(self):
        # A duplicated sign-carrying helper is how this repo accumulated its
        # sign-bug history; assert there is exactly one object.
        import gui.pages.hardware_setup as hs
        import SupportClasses.CameraRotationTracker as T
        self.assertIs(hs.wrap_deg, T.wrap_deg)
        self.assertIs(hs.nearest_nominal, T.nearest_nominal)
        from gui.dialogs import pixel_calibration_dialog as pcd
        self.assertIs(pcd.fold_parallel_deg, T.fold_parallel_deg)

    def test_nominal_rotation_delta_behaviour_unchanged(self):
        from gui.pages.hardware_setup import nominal_rotation_delta
        from SupportClasses.HardwareConfig import CameraRole
        self.assertEqual(nominal_rotation_delta(87.0, CameraRole.MICROSCOPE),
                         (90.0, -3.0))
        # A needle role still uses the DIAGONAL nominals for its readout.
        self.assertEqual(nominal_rotation_delta(87.0, CameraRole.NEEDLE_X)[0],
                         45.0)


class TestTargetRotationSign(unittest.TestCase):

    def test_unmirrored_is_plus_delta_mirrored_is_minus(self):
        for th in THETAS:
            nom, delta = nearest_square_rotation(th)
            self.assertAlmostEqual(
                target_image_rotation_deg(th, False, nom), wrap_deg(delta), 9)
            self.assertAlmostEqual(
                target_image_rotation_deg(th, True, nom), wrap_deg(-delta), 9)

    def test_already_square_needs_no_rotation(self):
        for nom in SQUARE_NOMINALS:
            for mir in (False, True):
                self.assertAlmostEqual(
                    target_image_rotation_deg(nom, mir, nom), 0.0, 9)

    def test_handedness_parked_on_either_axis_gives_the_same_answer(self):
        # derive_camera_stage_orientation always returns flip_x=False and puts
        # handedness on flip_y. Only det F is read, so (T,F) and (F,T) must
        # agree — otherwise that canonicalisation would matter.
        for th in THETAS:
            nom, _ = nearest_square_rotation(th)
            a = target_image_rotation_deg(th, True != False, nom)
            b = target_image_rotation_deg(th, False != True, nom)
            self.assertAlmostEqual(a, b, 12)


class TestClosedLoopSign(unittest.TestCase):
    """The one test that can catch a wrong answer in the sign helper.

    Builds the post-rotation camera state from ``theta' = theta -/+ phi`` and
    checks, through the REAL ``pixel_to_stage_offset``, that a scene point which
    moved from ``p`` to ``R(phi)p`` reports the SAME stage offset. Never names
    phi's sign, so it cannot be satisfied by a self-consistent-but-wrong pair.
    """

    @staticmethod
    def _mgr(theta, flip_x, flip_y, um_per_px=3.227):
        from gui.widgets.camera_manager import CameraManager
        m = CameraManager.__new__(CameraManager)
        m._max_cameras = 1
        m._um_per_px = [um_per_px]
        m._um_per_px_set = [True]
        m._um_per_px_res = [None]
        m._rotation_deg = [theta]
        m._mirrored = [flip_x]
        m._flip_y = [flip_y]
        m._cameras = []          # `cameras` is a read-only property
        return m

    @staticmethod
    def _rot(p, deg):
        t = math.radians(deg)
        c, s = math.cos(t), math.sin(t)
        return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)

    def test_rotating_to_the_target_preserves_every_stage_offset(self):
        rng = np.random.default_rng(3)
        w = h = 1024
        checked = 0
        for flip_x, flip_y in FLIP_COMBOS:
            net = flip_x != flip_y
            for theta in THETAS:
                nom, _ = nearest_square_rotation(theta)
                phi = target_image_rotation_deg(theta, net, nom)
                theta_after = wrap_deg(theta + phi) if net \
                    else wrap_deg(theta - phi)
                # (a) the turn lands exactly on the nominal
                self.assertAlmostEqual(wrap_deg(theta_after - nom), 0.0, 6)
                before = self._mgr(theta, flip_x, flip_y)
                after = self._mgr(theta_after, flip_x, flip_y)
                # (b) every scene point reports the same stage offset
                for _ in range(20):
                    px = float(rng.uniform(0, w))
                    py = float(rng.uniform(0, h))
                    q = self._rot((px - w / 2.0, py - h / 2.0), phi)
                    a = before.pixel_to_stage_offset(0, px, py, w, h)
                    b = after.pixel_to_stage_offset(
                        0, q[0] + w / 2.0, q[1] + h / 2.0, w, h)
                    self.assertAlmostEqual(a[0], b[0], places=6)
                    self.assertAlmostEqual(a[1], b[1], places=6)
                    checked += 1
        self.assertEqual(checked, len(FLIP_COMBOS) * len(THETAS) * 20)

    def test_mutation_a_flipped_mirror_condition_is_caught(self):
        # Force the wrong branch and assert this test would fail: the bore-Z
        # lesson is that a sign test must be shown to catch its own mutation.
        def wrong(theta, mirrored_net, nominal):
            delta = wrap_deg(float(theta) - float(nominal))
            return wrap_deg(delta if mirrored_net else -delta)

        theta, nom = 12.0, 0.0
        for flip_x, flip_y in ((False, True), (False, False)):
            net = flip_x != flip_y
            phi = wrong(theta, net, nom)
            after = wrap_deg(theta + phi) if net else wrap_deg(theta - phi)
            self.assertGreater(abs(wrap_deg(after - nom)), 20.0,
                               "a flipped mirror condition must miss the "
                               "target by roughly twice the tilt")


# ══════════════════════════════════════════════════════════════════════════
# Stage 0 — the mirrored-camera rotation measurement
# ══════════════════════════════════════════════════════════════════════════

class TestMirroredRotationMeasurement(unittest.TestCase):
    """``plus_column_direction_deg`` assumes F = I (its own docstring says so).

    Forward-simulates ``d = -(1/u)·F^-1·R(-theta)·m`` — which follows from
    ``pto``'s contract that a plate feature's stage LABEL is invariant under
    stage motion — and checks the conversion recovers theta for all four flip
    combinations and every direction preset.
    """

    @staticmethod
    def _disp(theta, mx, my, u, commanded, dist=200.0):
        m = (dist * math.cos(math.radians(commanded)),
             dist * math.sin(math.radians(commanded)))
        t = math.radians(-theta)
        c, s = math.cos(t), math.sin(t)
        v = (m[0] * c - m[1] * s, m[0] * s + m[1] * c)
        v = (mx * v[0], my * v[1])          # F^-1 == F for diag(+-1, +-1)
        return (-v[0] / u, -v[1] / u)

    def test_recovers_theta_for_every_flip_and_preset(self):
        from gui.dialogs.pixel_calibration_dialog import (
            column_direction_to_camera_rotation_deg, plus_column_direction_deg)
        for flip_x, flip_y in FLIP_COMBOS:
            mx = -1.0 if flip_x else 1.0
            my = -1.0 if flip_y else 1.0
            for theta in THETAS:
                for c in (0.0, 90.0, 45.0, -45.0):
                    dx, dy = self._disp(theta, mx, my, 3.227, c)
                    raw = plus_column_direction_deg(c, dx, dy)
                    got = column_direction_to_camera_rotation_deg(
                        c, raw, mirrored=flip_x, flip_y=flip_y)
                    self.assertAlmostEqual(
                        abs(wrap_deg(got - theta)), 0.0, places=6,
                        msg=f"flip=({flip_x},{flip_y}) theta={theta} c={c} "
                            f"raw={raw} got={got}")

    def test_is_exactly_a_no_op_for_an_unmirrored_camera(self):
        from gui.dialogs.pixel_calibration_dialog import (
            column_direction_to_camera_rotation_deg)
        for raw in (-179.0, -33.0, 0.0, 12.0, 179.0):
            for c in (0.0, 45.0, 90.0, -45.0):
                self.assertAlmostEqual(
                    column_direction_to_camera_rotation_deg(
                        c, raw, mirrored=False, flip_y=False),
                    wrap_deg(raw), places=9)

    def test_the_uncorrected_value_is_preset_dependent(self):
        # The tell that made this invisible: with a net mirror the raw reading
        # changes with whichever direction preset was clicked, so no single
        # reading looks wrong.
        from gui.dialogs.pixel_calibration_dialog import (
            plus_column_direction_deg)
        raws = {c: plus_column_direction_deg(
            c, *self._disp(30.0, 1.0, -1.0, 3.227, c))
            for c in (0.0, 45.0, 90.0)}
        self.assertGreater(len(set(round(v, 3) for v in raws.values())), 1)

    def test_the_real_microscope_entry_recovers_its_stored_angle(self):
        # andor:VSC-07863 — rotation_deg 180, mirrored true, no flip_y.
        from gui.dialogs.pixel_calibration_dialog import (
            column_direction_to_camera_rotation_deg, plus_column_direction_deg)
        dx, dy = self._disp(180.0, -1.0, 1.0, 3.227061, 45.0)
        raw = plus_column_direction_deg(45.0, dx, dy)
        fixed = column_direction_to_camera_rotation_deg(
            45.0, raw, mirrored=True, flip_y=False)
        self.assertAlmostEqual(abs(wrap_deg(fixed - 180.0)), 0.0, places=6)
        self.assertGreater(abs(wrap_deg(raw - 180.0)), 45.0,
                           "the uncorrected value must be visibly wrong, "
                           "otherwise this fix is untested")


# ══════════════════════════════════════════════════════════════════════════
# Estimator
# ══════════════════════════════════════════════════════════════════════════

class TestPrepareFrame(unittest.TestCase):

    def test_any_resolution_becomes_the_same_square(self):
        for (w, h) in ((3664, 2748), (640, 480), (1024, 1024), (2600, 2048)):
            g = prepare_frame(np.dstack([textured(max(w, h))[:h, :w]] * 3))
            self.assertEqual(g.shape, (TRACK_SIZE, TRACK_SIZE))
            self.assertEqual(g.dtype, np.uint8)

    def test_square_crop_is_centred(self):
        a = np.zeros((100, 200), np.uint8)
        a[40:60, 90:110] = 255            # a blob at the frame centre
        g = prepare_frame(a, size=50)
        self.assertGreater(g[20:30, 20:30].mean(), 100)

    def test_uses_inter_area_when_downscaling(self):
        # Bilinear aliases badly at ~14x decimation and the log-polar transform
        # weights all radii, so the aliasing lands in the angle estimate.
        with patch("SupportClasses.CameraRotationTracker.cv2.resize",
                   wraps=cv2.resize) as spy:
            prepare_frame(textured(1024), size=256)
        self.assertEqual(spy.call_args.kwargs["interpolation"], cv2.INTER_AREA)

    def test_16_bit_input_is_normalised_to_uint8(self):
        a = (textured(512).astype(np.uint16) * 257)
        self.assertEqual(prepare_frame(a).dtype, np.uint8)

    def test_flat_frame_is_not_trackable(self):
        self.assertFalse(frame_is_trackable(
            np.full((TRACK_SIZE, TRACK_SIZE), 128, np.uint8)))
        self.assertTrue(frame_is_trackable(prepare_frame(textured())))


class TestEstimator(unittest.TestCase):

    def setUp(self):
        self.ref = prepare_frame(textured())

    def test_round_trip_ghost_then_estimate(self):
        for phi in (-45.0, -30.0, -12.0, -5.0, -2.0, 2.0, 5.0, 12.0, 30.0, 45.0):
            s = estimate_rotation(self.ref, make_ghost(self.ref, phi))
            self.assertIsNotNone(s, f"phi={phi} rejected")
            self.assertAlmostEqual(s.deg, phi, delta=0.7)

    def test_accuracy_is_exact_at_zero_and_bounded_elsewhere(self):
        """Pins the measured accuracy so a regression cannot quietly widen it.

        The estimate is EXACTLY 0 when the frames coincide — which is the
        reading the operator actually stops on — and reads slightly SHORT
        (toward zero) at small non-zero angles, peaking near 2°. That is the
        benign direction: the residual reads a little large, so the operator
        turns a touch further rather than stopping early. Step 4's stage-motion
        measurement is the authority regardless.
        """
        self.assertAlmostEqual(
            estimate_rotation(self.ref, self.ref.copy()).deg, 0.0, places=6)
        worst = 0.0
        for phi in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 20.0):
            for sign in (1.0, -1.0):
                s = estimate_rotation(self.ref, make_ghost(self.ref, sign * phi))
                self.assertIsNotNone(s)
                worst = max(worst, abs(s.deg - sign * phi))
        self.assertLess(worst, 0.7, f"small-angle error grew to {worst:.3f}°")

    def test_absolute_anchor_independent_of_make_ghost(self):
        # NOT circular: rotates with a hand-written getRotationMatrix2D call,
        # so a global sign inversion inside the log-polar block cannot be
        # absorbed by make_ghost the way the round-trip would absorb it.
        h, w = self.ref.shape
        m = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), 8.0, 1.0)
        s = estimate_rotation(self.ref, cv2.warpAffine(self.ref, m, (w, h)))
        self.assertIsNotNone(s)
        self.assertAlmostEqual(s.deg, -8.0, delta=0.3)

    def test_linearity(self):
        xs = list(range(0, 46, 5))
        ys = [estimate_rotation(self.ref, make_ghost(self.ref, float(x))).deg
              for x in xs]
        slope, intercept = np.polyfit(xs, ys, 1)
        self.assertAlmostEqual(slope, 1.0, delta=0.01)
        self.assertLess(abs(intercept), 0.2)

    def test_flat_field_is_rejected(self):
        self.assertIsNone(estimate_rotation(
            self.ref, np.full_like(self.ref, 128)))

    def test_a_wrapped_branch_is_rejected_not_unwrapped(self):
        # The estimator works on the FFT magnitude, so its output is only
        # defined mod 180. A wrapped sample de-rotates wrongly and the
        # translation response collapses — which is why this module folds and
        # gates instead of carrying an unwrapper that could itself be wrong.
        for phi in (100.0, 135.0, 180.0):
            self.assertIsNone(
                estimate_rotation(self.ref, make_ghost(self.ref, phi)),
                f"phi={phi} should not pass the confidence gate")

    def test_survives_motion_blur(self):
        blurred = cv2.GaussianBlur(make_ghost(self.ref, 12.0), (41, 41), 0)
        s = estimate_rotation(self.ref, blurred)
        self.assertIsNotNone(s)
        self.assertAlmostEqual(s.deg, 12.0, delta=0.8)

    def test_mismatched_shapes_return_none(self):
        self.assertIsNone(estimate_rotation(
            self.ref, np.zeros((128, 128), np.uint8)))

    def test_confidence_gate_is_enforced(self):
        with patch("SupportClasses.MosaicBuilder."
                   "_register_overlap_fourier_mellin",
                   return_value=(0.0, 0.0, 5.0, 1.0, MIN_CONF - 0.01)):
            self.assertIsNone(estimate_rotation(self.ref, self.ref))
        with patch("SupportClasses.MosaicBuilder."
                   "_register_overlap_fourier_mellin",
                   return_value=(0.0, 0.0, 5.0, 1.0, MIN_CONF + 0.01)):
            self.assertIsNotNone(estimate_rotation(self.ref, self.ref))


class TestEdgeGhost(unittest.TestCase):

    def test_edge_rgba_is_transparent_off_the_edges(self):
        out = edge_rgba(prepare_frame(textured()))
        self.assertEqual(out.shape[2], 4)
        self.assertEqual(out.dtype, np.uint8)
        self.assertGreater((out[..., 3] > 0).sum(), 50)     # some edges
        self.assertGreater((out[..., 3] == 0).sum(),
                           out[..., 3].size * 0.5)          # mostly clear

    def test_edge_rgba_carries_the_requested_colour(self):
        out = edge_rgba(prepare_frame(textured()), color_rgb=(255, 0, 128))
        on = out[..., 3] > 0
        self.assertTrue((out[..., 0][on] == 255).all())
        self.assertTrue((out[..., 2][on] == 128).all())


class TestRotationTracker(unittest.TestCase):

    def setUp(self):
        self.ref = prepare_frame(textured())

    def test_refuses_a_flat_reference(self):
        tr = RotationTracker()
        self.assertFalse(tr.set_reference(
            np.full((TRACK_SIZE, TRACK_SIZE), 128, np.uint8)))
        self.assertFalse(tr.has_reference)

    def test_residual_reaches_zero_at_the_target(self):
        tr = RotationTracker(target_deg=10.0)
        self.assertTrue(tr.set_reference(self.ref))
        for i, phi in enumerate((0.0, 4.0, 8.0, 10.0, 10.0, 10.0)):
            tr.update(make_ghost(self.ref, phi), i * 0.12)
        self.assertAlmostEqual(tr.residual_deg, 0.0, delta=0.6)

    def test_direction_hint_latches_from_measurement_only(self):
        for sign, expect in ((+1.0, "good"), (-1.0, "reverse")):
            tr = RotationTracker(target_deg=12.0)
            tr.set_reference(self.ref)
            hints = []
            for i in range(8):
                tr.update(make_ghost(self.ref, sign * i * 1.5), i * 0.12)
                hints.append(tr.direction_hint())
            self.assertEqual(hints[0], "unknown")
            self.assertEqual(hints[-1], expect)

    def test_staleness(self):
        tr = RotationTracker()
        tr.set_reference(self.ref)
        tr.update(self.ref, 100.0)
        self.assertFalse(tr.is_stale(100.2))
        self.assertTrue(tr.is_stale(101.0))

    def test_retarget_keeps_the_reference_and_restarts_the_latch(self):
        tr = RotationTracker(target_deg=5.0)
        tr.set_reference(self.ref)
        for i in range(6):
            tr.update(make_ghost(self.ref, i * 1.5), i * 0.12)
        tr.set_target(-5.0)
        self.assertTrue(tr.has_reference)
        self.assertEqual(tr.direction_hint(), "unknown")
        self.assertAlmostEqual(tr.target_deg, -5.0)

    def test_a_resolution_change_is_refused_not_hidden(self):
        # prepare_frame normalises every resolution to the same square, so a
        # mid-session resolution change would otherwise be invisible to the
        # estimator even though the scene scale has changed.
        tr = RotationTracker()
        tr.set_reference(self.ref)
        self.assertIsNone(tr.update(np.zeros((64, 64), np.uint8), 1.0))


# ══════════════════════════════════════════════════════════════════════════
# Qt — the ghost overlay
# ══════════════════════════════════════════════════════════════════════════

_app = None


def setUpModule():
    global _app
    from PySide6.QtWidgets import QApplication
    _app = QApplication.instance() or QApplication([])


def _qimg(w, h, val):
    from PySide6.QtGui import QImage
    a = np.full((h, w, 3), val, np.uint8)
    return QImage(a.copy().data, w, h, w * 3,
                  QImage.Format.Format_RGB888).copy()


class TestAlignmentGhost(unittest.TestCase):

    def setUp(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        self.view = CameraFeedView(camera_manager=None, cam_idx=0,
                                   show_crosshair=False, enable_settings=False)
        self.view.resize(320, 240)
        self.view.show()
        self.frame = _qimg(640, 480, 0)
        self.ghost = _qimg(640, 480, 255)

    def tearDown(self):
        self.view.deleteLater()

    def _centre_corners(self):
        im = self.view._last_pixmap.toImage()
        return (im.pixelColor(im.width() // 2, im.height() // 2).red(),
                im.pixelColor(2, 2).red(),
                im.pixelColor(im.width() - 3, im.height() - 3).red())

    def test_no_ghost_by_default(self):
        self.view._on_frame(self.frame)
        self.assertEqual(self._centre_corners()[0], 0)

    def test_ghost_covers_the_whole_frame(self):
        # Regression: compositing a square tracking crop with drawImage(0, 0)
        # would land a small patch in the corner instead of covering the frame.
        self.view.set_alignment_ghost(self.ghost, opacity=0.5)
        self.view._on_frame(self.frame)
        centre, tl, br = self._centre_corners()
        for v in (centre, tl, br):
            self.assertGreater(v, 100)
            self.assertLess(v, 200)

    def test_clearing_restores_the_plain_frame(self):
        self.view.set_alignment_ghost(self.ghost, opacity=0.5)
        self.view._on_frame(self.frame)
        self.view.set_alignment_ghost(None)
        self.view._on_frame(self.frame)
        self.assertEqual(self._centre_corners()[0], 0)

    def test_opacity_is_honoured(self):
        for op, lo, hi in ((0.1, 1, 60), (0.9, 200, 254)):
            self.view.set_alignment_ghost(self.ghost, opacity=op)
            self.view._on_frame(self.frame)
            c = self._centre_corners()[0]
            self.assertTrue(lo <= c <= hi, f"opacity {op} -> {c}")

    def test_offset_moves_the_ghost(self):
        self.view.set_alignment_ghost(self.ghost, opacity=1.0,
                                      offset_px=(10000, 10000))
        self.view._on_frame(self.frame)
        self.assertEqual(self._centre_corners()[0], 0)

    def test_the_ghost_is_cached_across_frames(self):
        # Load-bearing: re-orienting a full-res ghost per frame costs ~33 ms on
        # a 10 Mpx camera, which would halve the GUI thread's frame budget.
        self.view.set_alignment_ghost(self.ghost, opacity=0.5)
        self.view._on_frame(self.frame)
        key = self.view._ghost_cache_key
        cached = self.view._ghost_cache
        for _ in range(10):
            self.view._on_frame(self.frame)
        self.assertIs(self.view._ghost_cache, cached)
        self.assertEqual(self.view._ghost_cache_key, key)

    def test_cache_is_invalidated_by_every_input_it_depends_on(self):
        self.view.set_alignment_ghost(self.ghost, opacity=0.5)
        self.view._on_frame(self.frame)
        keys = [self.view._ghost_cache_key]

        def note():
            self.view._on_frame(self.frame)
            keys.append(self.view._ghost_cache_key)

        self.view.set_view_orientation(mirrored=True)
        note()
        self.view.set_edge_pick_mode(True)
        note()
        self.view.resize(400, 300)
        note()
        self.view.set_alignment_ghost(_qimg(640, 480, 10), opacity=0.5)
        note()
        self.assertEqual(len(set(keys)), len(keys),
                         "every input change must produce a new cache key")

    def test_rgba_ghost_blends_only_where_opaque(self):
        from PySide6.QtGui import QImage
        a = np.zeros((480, 640, 4), np.uint8)
        a[:, :, 0] = 255                       # red
        a[200:280, :, 3] = 255                 # an opaque band
        img = QImage(a.copy().data, 640, 480, 640 * 4,
                     QImage.Format.Format_RGBA8888).copy()
        self.view.set_alignment_ghost(img, opacity=1.0)
        self.view._on_frame(self.frame)
        im = self.view._last_pixmap.toImage()
        self.assertGreater(im.pixelColor(im.width() // 2,
                                         im.height() // 2).red(), 200)
        self.assertEqual(im.pixelColor(im.width() // 2, 3).red(), 0)


# ══════════════════════════════════════════════════════════════════════════
# Qt — the dialog
# ══════════════════════════════════════════════════════════════════════════

class _FakeCam:
    """A CameraWidget stand-in that serves a rotating scene."""

    def __init__(self, base=None):
        self._base = textured(512) if base is None else base
        self._phi = 0.0
        self._seq = 0
        self._frozen = False

    def turn_to(self, phi):
        self._phi = float(phi)
        if not self._frozen:
            self._seq += 1

    def freeze_feed(self):
        self._frozen = True

    def get_current_frame(self):
        g = make_ghost(self._base, self._phi)
        return np.dstack([g, g, g])

    def frame_count_value(self):
        if not self._frozen:
            self._seq += 1
        return self._seq


class _FakeMgr:
    def __init__(self, cam, theta=12.0, flip_x=False, flip_y=False):
        self.cameras = [cam]
        self._theta = theta
        self._fx = flip_x
        self._fy = flip_y
        self.started = 0

    def is_running(self, i):
        return True

    def start(self, i):
        self.started += 1

    def get_rotation_deg(self, i):
        return self._theta

    def get_mirrored(self, i):
        return self._fx

    def get_flip_y(self, i):
        return self._fy

    def get_column_dir_deg(self, i):
        return 0.0

    def full_orientation(self, i):
        return (self._fx, self._fy, float(self._theta or 0.0))


class TestAlignDialog(unittest.TestCase):

    def _dlg(self, mgr, remeasure=None):
        from gui.dialogs.camera_rotation_align_dialog import (
            CameraRotationAlignDialog)
        d = CameraRotationAlignDialog(mgr, 0, remeasure=remeasure)
        self.addCleanup(d.deleteLater)
        self.addCleanup(d._stop_worker)
        return d

    def test_target_is_the_nearest_cardinal(self):
        d = self._dlg(_FakeMgr(_FakeCam(), theta=12.0))
        self.assertAlmostEqual(d._nominal, 0.0)
        self.assertAlmostEqual(d._phi_target, 12.0, places=6)
        d2 = self._dlg(_FakeMgr(_FakeCam(), theta=87.0))
        self.assertAlmostEqual(d2._nominal, 90.0)
        self.assertAlmostEqual(d2._phi_target, -3.0, places=6)

    def test_mirrored_camera_inverts_the_target_and_says_so(self):
        d = self._dlg(_FakeMgr(_FakeCam(), theta=12.0, flip_x=True))
        self.assertAlmostEqual(d._phi_target, -12.0, places=6)
        # isVisible() is False for every child of a dialog that was never
        # shown, so it would be a tautology here — isHidden() is the real
        # question (v7.9 recorded exactly that trap).
        self.assertFalse(d._lbl_warn.isHidden())
        self.assertIn("mirrored", d._lbl_warn.text())

    def test_toggling_the_mirror_mid_session_flips_the_target(self):
        # The mirror checkbox lives on the slot card BEHIND this dialog, and
        # flipping it inverts det F and therefore phi_target — with no visual
        # cue if the value were snapshotted at construction.
        mgr = _FakeMgr(_FakeCam(), theta=12.0)
        d = self._dlg(mgr)
        self.assertAlmostEqual(d._phi_target, 12.0, places=6)
        mgr._fx = True
        d._tick()
        self.assertAlmostEqual(d._phi_target, -12.0, places=6)

    def test_theta_rewritten_mid_session_is_picked_up(self):
        mgr = _FakeMgr(_FakeCam(), theta=12.0)
        d = self._dlg(mgr)
        mgr._theta = 87.0
        d._tick()
        self.assertAlmostEqual(d._nominal, 90.0)
        self.assertAlmostEqual(d._phi_target, -3.0, places=6)

    def test_flip_ghost_direction_negates_the_target(self):
        d = self._dlg(_FakeMgr(_FakeCam(), theta=12.0))
        d._on_flip()
        self.assertAlmostEqual(d._phi_target, -12.0, places=6)
        d._on_flip()
        self.assertAlmostEqual(d._phi_target, 12.0, places=6)

    def test_uncalibrated_camera_is_reported_not_crashed(self):
        d = self._dlg(_FakeMgr(_FakeCam(), theta=None))
        self.assertIn("not been measured", d._lbl_head.text())
        self.assertEqual(d._phi_target, 0.0)

    def test_the_view_corrects_flips_but_never_the_rotation(self):
        # Correct what is NOT changing; show what IS.
        d = self._dlg(_FakeMgr(_FakeCam(), theta=33.0, flip_x=True,
                               flip_y=True))
        self.assertTrue(d._feed._view_mirror)
        self.assertTrue(d._feed._view_flip_y)
        self.assertAlmostEqual(d._feed._view_rot_deg, 0.0)

    def test_feed_watchdog_blanks_the_readout(self):
        cam = _FakeCam()
        d = self._dlg(_FakeMgr(cam, theta=12.0))
        d._tick()
        cam.freeze_feed()
        d._last_frame_t = time.monotonic() - 99.0
        d._tick()
        self.assertIn("feed stopped", d._lbl_state.text())
        self.assertIsNone(d._dial._residual)

    def test_remeasure_cancelled_reports_not_re_measured(self):
        calls = []

        def cancelled():
            calls.append(1)
            return False

        d = self._dlg(_FakeMgr(_FakeCam(), theta=12.0), remeasure=cancelled)
        d._on_remeasure()
        self.assertEqual(len(calls), 1)
        self.assertIn("Not re-measured", d._lbl_result.text())

    def test_remeasure_committed_reports_the_new_delta(self):
        mgr = _FakeMgr(_FakeCam(), theta=12.0)

        def committed():
            mgr._theta = 0.4
            return True

        d = self._dlg(mgr, remeasure=committed)
        d._on_remeasure()
        self.assertIn("+0.40", d._lbl_result.text())
        self.assertIn("squared up", d._lbl_result.text())

    def test_remeasure_stops_the_worker_before_moving_the_stage(self):
        seen = {}

        def check():
            seen["worker"] = self._d._worker
            return False

        self._d = self._dlg(_FakeMgr(_FakeCam(), theta=12.0), remeasure=check)
        self._d._on_remeasure()
        self.assertIsNone(seen["worker"],
                          "the calibration moves the stage — nothing may be "
                          "sampling frames while it runs")

    def test_worker_starts_and_stops_cleanly(self):
        d = self._dlg(_FakeMgr(_FakeCam(), theta=12.0))
        self.assertIsNotNone(d._worker)
        names = [t.name for t in threading.enumerate()]
        self.assertIn("camera-square-up", names)
        t0 = time.monotonic()
        d._stop_worker()
        self.assertLess(time.monotonic() - t0, 1.0)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if "camera-square-up" not in [t.name for t in threading.enumerate()]:
                break
            time.sleep(0.02)
        self.assertNotIn("camera-square-up",
                         [t.name for t in threading.enumerate()])

    def test_a_live_turn_drives_the_residual_toward_zero(self):
        from PySide6.QtWidgets import QApplication
        cam = _FakeCam()
        d = self._dlg(_FakeMgr(cam, theta=12.0))
        seen = []
        d._bridge.sample.connect(lambda *a: seen.append(a[1]))
        deadline = time.monotonic() + 6.0
        for phi in (0.0, 3.0, 6.0, 9.0, 12.0, 12.0, 12.0):
            cam.turn_to(phi)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.5 and time.monotonic() < deadline:
                QApplication.processEvents()
                time.sleep(0.02)
        self.assertTrue(seen, "the worker produced no samples")
        self.assertGreater(abs(seen[0]), 6.0)
        self.assertLess(abs(seen[-1]), 1.5)


class TestSquareUpGate(unittest.TestCase):
    """The refusals. A gate evaluated once at page build and never re-run is
    exactly the v7.10 bore-calibration bug, so these are checked as pure
    predicates AND wired into the same refresh paths as the rotation button."""

    def _page(self, roles, theta=12.0, column_dir=0.0):
        from gui.pages.hardware_setup import HardwareSetupPage
        from SupportClasses.HardwareConfig import HardwareConfig
        p = HardwareSetupPage.__new__(HardwareSetupPage)
        p._config = HardwareConfig()
        p._config.camera_roles = list(roles)

        class M:
            def get_rotation_deg(self, i):
                return theta

            def get_column_dir_deg(self, i):
                return column_dir

            def is_running(self, i):
                return True
        p._camera_manager = M()
        return p

    def test_allows_a_calibrated_running_microscope(self):
        from SupportClasses.HardwareConfig import CameraRole
        p = self._page([CameraRole.MICROSCOPE])
        self.assertEqual(p.square_up_refusal(0, True), "")

    def test_refuses_when_the_camera_is_not_running(self):
        from SupportClasses.HardwareConfig import CameraRole
        p = self._page([CameraRole.MICROSCOPE])
        self.assertIn("Start the camera", p.square_up_refusal(0, False))

    def test_refuses_when_rotation_is_unmeasured(self):
        from SupportClasses.HardwareConfig import CameraRole
        p = self._page([CameraRole.MICROSCOPE], theta=None)
        self.assertIn("Calibrate", p.square_up_refusal(0, True))

    def test_refuses_a_needle_cam_with_no_separate_mount_direction(self):
        # The pre-v7.5.x conflated state: rotation_deg still holds the +-45
        # MOUNT. Squaring that up would command a 45 degree physical roll,
        # scaling the column um/px by cos 45 and sending the needle aligner to
        # the wrong XY. Three such entries still sit in this rig's store.
        from SupportClasses.HardwareConfig import CameraRole
        p = self._page([CameraRole.NEEDLE_X], theta=45.0, column_dir=None)
        self.assertIn("mount direction", p.square_up_refusal(0, True))

    def test_allows_a_needle_cam_once_mount_and_roll_are_separate(self):
        from SupportClasses.HardwareConfig import CameraRole
        p = self._page([CameraRole.NEEDLE_X], theta=1.2, column_dir=-44.6)
        self.assertEqual(p.square_up_refusal(0, True), "")


class TestImportHygiene(unittest.TestCase):

    def test_the_tracker_pulls_in_no_heavy_or_gui_dependencies(self):
        # It sits in the camera path; MosaicBuilder costs ~0.5 s to import
        # because of skimage, so the estimator is lazy-imported instead.
        code = (
            "import sys;"
            "assert 'SupportClasses.CameraRotationTracker' not in sys.modules;"
            "import SupportClasses.CameraRotationTracker as T;"
            "bad=[m for m in ('skimage','scipy','PySide6') if m in sys.modules];"
            "assert not bad, bad;"
            "print('clean')"
        )
        import subprocess
        out = subprocess.run([sys.executable, "-c", code],
                             capture_output=True, text=True,
                             cwd=os.path.dirname(os.path.dirname(
                                 os.path.abspath(__file__))))
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("clean", out.stdout)


if __name__ == "__main__":
    unittest.main()
