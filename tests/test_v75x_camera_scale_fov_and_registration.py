"""
test_v75x_camera_scale_fov_and_registration.py

v7.5.x operator batch on the mosaic/camera calibration:
  * auto-calibrate µm/px + FOV from stage motion (edge-feature tracking);
  * a BETTER mosaic registration algorithm (pairwise phase correlation + global
    least-squares optimize) that tolerates stage drift;
  * the camera video matching the camera's true aspect (no fixed box);
  * FOV anchored to the ACTUAL captured resolution.

Covers the load-bearing new primitives:
  1. VisionDetector.select_trackable_patch + find_template round-trip (the
     large-baseline feature tracker behind the scale/FOV auto-cal).
  2. MosaicBuilder._register_overlap_cv2 sign + confidence.
  3. MosaicBuilder.optimize_registration CORRECTS an injected tile-position
     error (the decisive end-to-end test — also locks the phase-corr sign).
  4. CameraFeedView reports the camera's true aspect via heightForWidth.
  5. ScaleFovCalibrationDialog derives µm/px + FOV from measured displacements.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.MosaicBuilder import (
    MosaicBuilder, CV2_AVAILABLE, _register_overlap_cv2)
from SupportClasses.VisionDetector import select_trackable_patch, find_template


def _texture(h, w, seed=7):
    """A reproducible richly-textured grayscale->BGR image (phase correlation
    needs structure). Blurred noise + a couple of bright blobs."""
    rng = np.random.RandomState(seed)
    g = rng.randint(0, 255, (h, w), dtype=np.uint8)
    if CV2_AVAILABLE:
        import cv2
        g = cv2.GaussianBlur(g, (5, 5), 0)
        cv2.circle(g, (w // 3, h // 2), max(6, h // 12), 255, -1)
        cv2.rectangle(g, (2 * w // 3, h // 3), (2 * w // 3 + 20, h // 3 + 20),
                      30, -1)
    return np.dstack([g, g, g])


# ═══════════════════════════════════════════════════════════════════
#  Feature tracking (scale/FOV auto-cal substrate)
# ═══════════════════════════════════════════════════════════════════

@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestPatchTracking(unittest.TestCase):
    def test_select_and_track_recovers_shift(self):
        base = _texture(300, 400)
        sel = select_trackable_patch(base)
        self.assertIsNotNone(sel)
        cx, cy, patch = sel
        self.assertGreater(patch.shape[0], 8)
        # Shift the WHOLE scene right by 37 px; the same patch must be found 37
        # px to the right with high confidence.
        shifted = np.zeros_like(base)
        shifted[:, 37:] = base[:, :-37]
        res = find_template(shifted, patch)
        self.assertIsNotNone(res)
        fx, fy, conf = res
        self.assertAlmostEqual(fx, cx + 37, delta=2.0)
        self.assertAlmostEqual(fy, cy, delta=2.0)
        self.assertGreater(conf, 0.6)

    def test_flat_frame_has_no_feature(self):
        flat = np.full((120, 160, 3), 128, np.uint8)
        sel = select_trackable_patch(flat)
        # A patch is still returned (variance 0 winner), but tracking a flat
        # scene is meaningless — the variance is ~0.
        if sel is not None:
            _cx, _cy, patch = sel
            self.assertLess(float(patch.var()), 1.0)


# ═══════════════════════════════════════════════════════════════════
#  Pairwise registration estimator
# ═══════════════════════════════════════════════════════════════════

@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestRegisterOverlap(unittest.TestCase):
    def test_recovers_known_shift_with_confidence(self):
        import cv2
        base = _texture(200, 260)
        a = cv2.cvtColor(base[:, 0:200], cv2.COLOR_BGR2GRAY)   # cols 0..200
        b = cv2.cvtColor(base[:, 7:207], cv2.COLOR_BGR2GRAY)   # cols 7..207
        # a(x) = base[:, x]; b(x) = base[:, x+7]  → a is b shifted; the estimator
        # returns the shift to ADD to B to align onto A. Magnitude must be ~7.
        res = _register_overlap_cv2(a, b)
        self.assertIsNotNone(res)
        dx, dy, conf = res
        self.assertAlmostEqual(abs(dx), 7.0, delta=1.5)
        self.assertLess(abs(dy), 1.5)
        self.assertGreater(conf, 0.2)

    def test_flat_overlap_returns_none(self):
        flat = np.full((40, 40), 100, np.uint8)
        self.assertIsNone(_register_overlap_cv2(flat, flat))


# ═══════════════════════════════════════════════════════════════════
#  Full pairwise + global-optimize registration
# ═══════════════════════════════════════════════════════════════════

@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestOptimizeRegistration(unittest.TestCase):
    def _builder_with_two_tiles(self, err_px):
        """A 2-tile mosaic where tile 1's stored position is wrong by err_px in
        x. optimize_registration should pull it back toward truth."""
        base = _texture(200, 400, seed=11)
        tile0 = base[0:200, 0:200].copy()      # true canvas x = 10
        tile1 = base[0:200, 150:350].copy()    # true canvas x = 160
        b = MosaicBuilder(frame_size_px=(200, 200), micron_per_pixel=1.0,
                          overlap=0.25, retain_for_reorient=True)
        cw, chh = 380, 220
        b._composite = np.zeros((chh, cw, 3), np.float64)
        b._weight_sum = np.zeros((chh, cw), np.float64)
        b._display_cache = np.zeros((chh, cw, 3), np.uint8)
        b._mosaic_scale = 1.0
        b._canvas_origin_um = (0.0, 0.0)
        # tile0 correct at x=10; tile1 stored err_px too far right of x=160.
        b._reorient_tiles = [
            (tile0, 10, 10, 200, 200),
            (tile1, 160 + err_px, 10, 200, 200),
        ]
        return b

    def test_optimize_registration_corrects_injected_error(self):
        err = 7
        b = self._builder_with_two_tiles(err)
        comp, n_edges, max_corr = b.optimize_registration(min_conf=0.03)
        self.assertGreaterEqual(n_edges, 1)
        opt = b._optimized_positions
        # Tile 0 is the anchor — stays put.
        self.assertAlmostEqual(opt[0][0], 10, delta=1.5)
        # Tile 1 corrected toward its TRUE x=160 (from the wrong 167).
        self.assertLess(abs(opt[1][0] - 160.0), abs((160 + err) - 160.0))
        self.assertLess(abs(opt[1][0] - 160.0), 2.5)

    def test_no_tiles_is_noop(self):
        b = MosaicBuilder(frame_size_px=(50, 50), micron_per_pixel=1.0)
        comp, n_edges, corr = b.optimize_registration()
        self.assertEqual(n_edges, 0)

    def test_reblend_at_positions_moves_content(self):
        b = self._builder_with_two_tiles(0)
        # Re-blend at a deliberately shifted position for tile 1, and confirm the
        # display cache changed vs the nominal blend.
        b._reblend_at_positions(np.array([[10, 10], [160, 10]], float))
        before = b.composite.copy()
        b._reblend_at_positions(np.array([[10, 10], [175, 10]], float))
        after = b.composite
        self.assertFalse(np.array_equal(before, after))


# ═══════════════════════════════════════════════════════════════════
#  CameraFeedView aspect ratio
# ═══════════════════════════════════════════════════════════════════

class TestFeedViewAspect(unittest.TestCase):
    def _view(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        return CameraFeedView(camera_manager=None, cam_idx=0,
                              enable_settings=False)

    def test_height_for_width_tracks_camera_aspect(self):
        v = self._view()
        self.assertTrue(v.hasHeightForWidth())
        # No frame yet → no constraint.
        self.assertEqual(v.heightForWidth(400), -1)
        v._last_image_size = (1600, 900)          # 16:9 camera
        self.assertEqual(v.heightForWidth(1600), 900)
        self.assertEqual(v.heightForWidth(800), 450)

    def test_rotated_view_swaps_aspect(self):
        v = self._view()
        v._last_image_size = (1600, 900)
        v.set_view_orientation(mirrored=False, rotation_deg=90.0)
        # A 90° view rotation swaps the displayed W:H → heightForWidth inverts.
        self.assertEqual(v.heightForWidth(900), 1600)


# ═══════════════════════════════════════════════════════════════════
#  Scale + FOV dialog math
# ═══════════════════════════════════════════════════════════════════

class TestScaleFovDialogCompute(unittest.TestCase):
    def _dlg(self):
        from gui.dialogs.scale_fov_calibration_dialog import (
            ScaleFovCalibrationDialog)
        # mgr=None → placeholder feed, no camera needed for the compute math.
        return ScaleFovCalibrationDialog(None, None, cam_idx=0)

    def test_compute_derives_um_per_px_and_fov(self):
        dlg = self._dlg()
        dlg._spin_move.setValue(1000.0)           # 1000 µm baseline
        dlg._frame_wh = (1920, 1080)
        # 500 px displacement on each axis → 2.0 µm/px.
        dlg._disp_x = (500.0, 0.0, 0.9)
        dlg._disp_y = (0.0, 500.0, 0.9)
        dlg._compute()
        self.assertAlmostEqual(dlg.result_um_per_px, 2.0, places=3)
        self.assertEqual(dlg.result_resolution, (1920, 1080))
        fw, fh = dlg.result_fov_um
        self.assertAlmostEqual(fw, 1920 * 2.0, delta=1.0)   # FOV extent
        self.assertAlmostEqual(fh, 1080 * 2.0, delta=1.0)

    def test_result_resolution_is_measured_frame(self):
        dlg = self._dlg()
        dlg._spin_move.setValue(800.0)
        dlg._frame_wh = (912, 686)
        dlg._disp_x = (400.0, 0.0, 0.8)
        dlg._disp_y = (0.0, 400.0, 0.8)
        dlg._compute()
        # Resolution comes from the CAPTURED frame, not an assumed value.
        self.assertEqual(dlg.result_resolution, (912, 686))
        self.assertAlmostEqual(dlg.result_um_per_px, 2.0, places=3)


# ═══════════════════════════════════════════════════════════════════
#  Motion-derived camera→stage orientation (rotation + HANDEDNESS)
# ═══════════════════════════════════════════════════════════════════

class TestDeriveCameraStageOrientation(unittest.TestCase):
    """Lock the two-axis-motion → (rotation, flip_x, flip_y) decomposition to
    the SAME R(θ)·diag(mx,my) convention _orient_tile / pixel_to_stage_offset
    use. The operator can't determine a mirror by eye (a mirror is not a
    rotation) — this recovers it from the sign of the cross product of the +X
    and +Y content displacements. For a given true camera→stage map M, the
    auto-cal measures content displacements P = -M⁻¹ (unit move, unit µm/px);
    the derived (θ, flips) must reconstruct M exactly."""

    def _reconstruct(self, theta_deg, flip_x, flip_y):
        import math
        t = math.radians(theta_deg)
        c, s = math.cos(t), math.sin(t)
        mx = -1.0 if flip_x else 1.0
        my = -1.0 if flip_y else 1.0
        # R(θ)·diag(mx,my)
        return np.array([[c * mx, -s * my], [s * mx, c * my]], dtype=float)

    def _synthesize_and_check(self, M):
        """Given a true camera→stage M, synthesize the measured displacements,
        derive (θ, flips), and assert they reconstruct M."""
        from gui.dialogs.scale_fov_calibration_dialog import (
            derive_camera_stage_orientation)
        Minv = np.linalg.inv(M)
        # P columns = content displacement for +X, +Y stage moves = -M⁻¹·axis.
        dxX, dyX = (-Minv[:, 0]).tolist()
        dxY, dyY = (-Minv[:, 1]).tolist()
        got = derive_camera_stage_orientation(dxX, dyX, dxY, dyY)
        self.assertIsNotNone(got)
        theta, fx, fy = got
        A = self._reconstruct(theta, fx, fy)
        self.assertTrue(np.allclose(A, M, atol=1e-6),
                        f"reconstructed\n{A}\n!= true\n{M}\n(θ={theta}, "
                        f"flip_x={fx}, flip_y={fy})")
        return theta, fx, fy

    def test_identity(self):
        theta, fx, fy = self._synthesize_and_check(np.eye(2))
        self.assertAlmostEqual(theta, 0.0, places=3)
        self.assertFalse(fx)
        self.assertFalse(fy)

    def test_mirror_x_becomes_flip(self):
        # image-X = -stage-X  →  a genuine mirror (odd handedness).
        theta, fx, fy = self._synthesize_and_check(np.diag([-1.0, 1.0]))
        # Canonical form puts the single flip on Y with a 180° rotation
        # (≡ flip-X); the KEY assertion is that a flip IS detected.
        self.assertTrue(fx or fy)

    def test_mirror_y_is_detected(self):
        theta, fx, fy = self._synthesize_and_check(np.diag([1.0, -1.0]))
        self.assertTrue(fx or fy)

    def test_180_is_pure_rotation_no_flip(self):
        # A 180° MOUNT (stage +X = camera -X AND stage +Y = camera -Y) is a
        # proper rotation, NOT a mirror — must come out with NO flip.
        theta, fx, fy = self._synthesize_and_check(np.diag([-1.0, -1.0]))
        self.assertAlmostEqual(abs(theta), 180.0, places=3)
        self.assertFalse(fx)
        self.assertFalse(fy)

    def test_rotation_90_no_flip(self):
        import math
        M = np.array([[math.cos(math.radians(90)), -math.sin(math.radians(90))],
                      [math.sin(math.radians(90)), math.cos(math.radians(90))]])
        theta, fx, fy = self._synthesize_and_check(M)
        self.assertFalse(fx)
        self.assertFalse(fy)

    def test_90_plus_mirror_is_detected(self):
        import math
        R = np.array([[math.cos(math.radians(90)), -math.sin(math.radians(90))],
                      [math.sin(math.radians(90)), math.cos(math.radians(90))]])
        M = R @ np.diag([-1.0, 1.0])
        theta, fx, fy = self._synthesize_and_check(M)
        self.assertTrue(fx or fy)

    def test_no_regression_vs_pure_rotation_model(self):
        # For a non-mirrored camera the derived rotation must EQUAL the legacy
        # plus_column_direction_deg (so uncalibrated/aligned cameras are
        # byte-identical). Check a few proper rotations.
        import math
        from gui.dialogs.scale_fov_calibration_dialog import (
            derive_camera_stage_orientation)
        from gui.dialogs.pixel_calibration_dialog import (
            plus_column_direction_deg)
        for phi in (0.0, 30.0, 90.0, 170.0, -45.0):
            M = np.array([[math.cos(math.radians(phi)), -math.sin(math.radians(phi))],
                          [math.sin(math.radians(phi)), math.cos(math.radians(phi))]])
            Minv = np.linalg.inv(M)
            dxX, dyX = (-Minv[:, 0]).tolist()
            dxY, dyY = (-Minv[:, 1]).tolist()
            theta, fx, fy = derive_camera_stage_orientation(dxX, dyX, dxY, dyY)
            legacy = plus_column_direction_deg(0.0, dxX, dyX)
            self.assertFalse(fx or fy)
            # Compare on the circle (avoid ±180 wrap mismatches).
            d = ((theta - legacy + 180.0) % 360.0) - 180.0
            self.assertAlmostEqual(d, 0.0, places=3)

    def test_degenerate_collinear_returns_none(self):
        from gui.dialogs.scale_fov_calibration_dialog import (
            derive_camera_stage_orientation)
        # Both moves produce parallel displacements → unsolvable.
        self.assertIsNone(
            derive_camera_stage_orientation(10.0, 0.0, 20.0, 0.0))


class TestScaleFovComputeDerivesFlips(unittest.TestCase):
    """The dialog's _compute stores the measured flips, and a mirrored camera
    (measured via a handedness-reversing pair of moves) sets a flip flag."""

    def _dlg(self):
        from gui.dialogs.scale_fov_calibration_dialog import (
            ScaleFovCalibrationDialog)
        return ScaleFovCalibrationDialog(None, None, cam_idx=0)

    def test_aligned_camera_no_flip(self):
        dlg = self._dlg()
        dlg._spin_move.setValue(1000.0)
        dlg._frame_wh = (1920, 1080)
        # +X move → content moves -X (aligned camera), +Y → content -Y.
        dlg._disp_x = (-500.0, 0.0, 0.9)
        dlg._disp_y = (0.0, -500.0, 0.9)
        dlg._compute()
        self.assertFalse(dlg.result_flip_x)
        self.assertFalse(dlg.result_flip_y)
        self.assertAlmostEqual(dlg.result_rotation_deg, 0.0, places=3)

    def test_mirrored_camera_sets_a_flip(self):
        dlg = self._dlg()
        dlg._spin_move.setValue(1000.0)
        dlg._frame_wh = (1920, 1080)
        # Handedness-reversed pair (cross product flips sign) → a flip.
        dlg._disp_x = (-500.0, 0.0, 0.9)
        dlg._disp_y = (0.0, 500.0, 0.9)     # +Y content moves the OTHER way
        dlg._compute()
        self.assertTrue(bool(dlg.result_flip_x) or bool(dlg.result_flip_y))


# ═══════════════════════════════════════════════════════════════════
#  Issue 1 — auto-orient keeps the calibration view consistent
# ═══════════════════════════════════════════════════════════════════

class TestAutoOrient(unittest.TestCase):
    class _Mgr:
        def __init__(self, mir, rot):
            self._m, self._r = mir, rot
            self.cameras = []

        def view_orientation(self, i):
            return (self._m, self._r)

    def test_auto_orient_syncs_from_manager(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        v = CameraFeedView(camera_manager=self._Mgr(True, 90.0), cam_idx=0,
                           enable_settings=False, auto_orient=True)
        v._sync_auto_orientation()
        self.assertTrue(v._view_mirror)
        self.assertAlmostEqual(v._view_rot_deg, 90.0)

    def test_edge_pick_suppresses_auto_orient(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        v = CameraFeedView(camera_manager=self._Mgr(True, 90.0), cam_idx=0,
                           enable_settings=False, auto_orient=True)
        v.set_edge_pick_mode(True)
        v._sync_auto_orientation()
        self.assertFalse(v._view_mirror)          # not applied under edge-pick
        self.assertAlmostEqual(v._view_rot_deg, 0.0)

    def test_auto_orient_off_by_default(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        v = CameraFeedView(camera_manager=self._Mgr(True, 90.0), cam_idx=0,
                           enable_settings=False)   # auto_orient defaults off
        v._sync_auto_orientation()
        self.assertFalse(v._view_mirror)


# ═══════════════════════════════════════════════════════════════════
#  Issue 2 — a fresh calibration clears the stale learned mosaic FOV
# ═══════════════════════════════════════════════════════════════════

class TestObjectiveCardClearsStaleFov(unittest.TestCase):
    def _fresh_align_store(self):
        import tempfile
        from pathlib import Path
        import SupportClasses.MosaicAlignmentStore as MAS
        MAS._store_singleton = MAS.MosaicAlignmentStore(
            Path(tempfile.mkdtemp()) / "align.json")
        return MAS

    def _card(self):
        from gui.pages.hardware.objective_calibration_card import (
            ObjectiveCalibrationCard)

        class _Spec:
            name = "TestCam"

        class _CamCfg:
            current_objective_name = "10x"
            camera_spec = _Spec()
            active_resolution = (1920, 1080)

        class _Cfg:
            camera_config = _CamCfg()

            def camera_for_role(self, role):
                return 0

        class _Mgr:
            def camera_identity(self, i):
                return ("CAMKEY", "TestCam")

        return ObjectiveCalibrationCard(_Mgr(), lambda: _Cfg())

    def test_align_key_matches_ploc_convention(self):
        card = self._card()
        self.assertEqual(card._mosaic_align_key(0, "10x"), "CAMKEY|10x")

    def test_fresh_calibration_clears_stale_learned_fov(self):
        MAS = self._fresh_align_store()
        MAS.get_store().set_um_per_px("CAMKEY|10x", 2.5, resolution=(1920, 1080))
        self.assertEqual(MAS.get_store().get_um_per_px("CAMKEY|10x"), 2.5)
        card = self._card()
        card._clear_stale_mosaic_fov(0, "10x")
        # The stale "Store FOV/spacing" value is gone → the mosaic falls through
        # to the fresh objective/scale calibration.
        self.assertIsNone(MAS.get_store().get_um_per_px("CAMKEY|10x"))

    def test_learned_value_records_resolution_for_rescale(self):
        # The mosaic build rescales a learned value by cal_w/frame_w; verify the
        # store keeps the resolution needed for that rescale.
        MAS = self._fresh_align_store()
        MAS.get_store().set_um_per_px("K", 3.0, resolution=(912, 686))
        self.assertEqual(MAS.get_store().get_resolution("K"), (912.0, 686.0))
        # rescale to a 1824-wide scan → half the µm/px.
        lv = MAS.get_store().get_um_per_px("K")
        lres = MAS.get_store().get_resolution("K")
        eff = lv * float(lres[0]) / 1824.0
        self.assertAlmostEqual(eff, 1.5, places=4)


# ═══════════════════════════════════════════════════════════════════
#  Issue 3 — Scale/FOV dialog self-check mosaic UI
# ═══════════════════════════════════════════════════════════════════

class TestScaleFovVerifyUI(unittest.TestCase):
    def _dlg(self):
        from gui.dialogs.scale_fov_calibration_dialog import (
            ScaleFovCalibrationDialog)
        return ScaleFovCalibrationDialog(None, None, cam_idx=0)

    def test_has_verify_button_and_mosaic_label(self):
        d = self._dlg()
        self.assertTrue(hasattr(d, "_btn_verify"))
        self.assertTrue(hasattr(d, "_mosaic_label"))
        # Verify is gated on a measurement existing.
        d.result_um_per_px = None
        d._busy(False)
        self.assertFalse(d._btn_verify.isEnabled())
        d.result_um_per_px = 2.0
        d._busy(False)
        self.assertTrue(d._btn_verify.isEnabled())

    def test_show_mosaic_renders_composite(self):
        d = self._dlg()
        comp = np.zeros((40, 60, 3), np.uint8)
        comp[:, :, 1] = 200
        d._show_mosaic(comp)
        pm = d._mosaic_label.pixmap()
        self.assertIsNotNone(pm)
        self.assertFalse(pm.isNull())


# ═══════════════════════════════════════════════════════════════════
#  Issue 1 — device resolution change updates the camera-setup block
# ═══════════════════════════════════════════════════════════════════

class TestResolutionSyncToBlock(unittest.TestCase):
    def test_signals_exist(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        from gui.widgets.camera_feed_view import CameraFeedView
        self.assertTrue(hasattr(CameraSettingsDialog, "resolution_applied"))
        self.assertTrue(hasattr(CameraFeedView, "resolution_changed"))

    def test_handler_updates_active_resolution_and_combo(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from PySide6.QtWidgets import QComboBox

        class _CamCfg:
            active_resolution = (916, 686)

        class _Cfg:
            camera_config = _CamCfg()

            def camera_for_role(self, role):
                return 0

        class _Fake:
            _config = _Cfg()
            cam_resolution_combo = QComboBox()

            def _update_camera_info_labels(self):
                pass

        fake = _Fake()
        HardwareSetupPage._on_slot_resolution_changed(fake, 0, 1832, 1374)
        self.assertEqual(tuple(fake._config.camera_config.active_resolution),
                         (1832, 1374))
        self.assertEqual(fake.cam_resolution_combo.currentData(), (1832, 1374))

    def test_handler_ignores_non_microscope_slot(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from PySide6.QtWidgets import QComboBox

        class _CamCfg:
            active_resolution = (916, 686)

        class _Cfg:
            camera_config = _CamCfg()

            def camera_for_role(self, role):
                return 0                     # microscope is slot 0

        class _Fake:
            _config = _Cfg()
            cam_resolution_combo = QComboBox()

            def _update_camera_info_labels(self):
                pass

        fake = _Fake()
        HardwareSetupPage._on_slot_resolution_changed(fake, 2, 1832, 1374)
        self.assertEqual(tuple(fake._config.camera_config.active_resolution),
                         (916, 686))        # unchanged — slot 2 ≠ microscope


# ═══════════════════════════════════════════════════════════════════
#  Issue 3 — correction dialog: flip-to-X/Y + propagate as ground truth
# ═══════════════════════════════════════════════════════════════════

class TestMosaicCorrectionPropagation(unittest.TestCase):
    class _Mgr:
        def __init__(self):
            self.set_calls = []
            self._rot = 12.0

        def view_orientation(self, i):
            return (False, 0.0)

        def set_um_per_px(self, i, v, resolution=None):
            self.set_calls.append((i, v, resolution))

        def get_rotation_deg(self, i):
            return self._rot

    class _Ctrl:
        is_zp_connected = False
        safety_limits = None

        def get_xy_position(self, cached=False):
            return (0.0, 0.0)

    def _dlg(self, mgr, cam_key="TestCamProp", objective="10x"):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        return MosaicCalibrationDialog(
            self._Ctrl(), mgr, 0, safe_z=1.0, align_key="K|10x", store=None,
            settings={}, center_um=(0.0, 0.0), frame_size=(912, 686),
            um_per_px_camera=1.5, cam_key=cam_key, objective=objective)

    def test_flip_buttons_labeled_x_and_y(self):
        d = self._dlg(self._Mgr())
        self.assertIn("X", d._o_fliph.text())
        self.assertIn("Y", d._o_flipv.text())

    def test_propagate_writes_manager_and_objective(self):
        from SupportClasses.ObjectiveCalibration import get_store
        mgr = self._Mgr()
        d = self._dlg(mgr)
        try:
            d._propagate_um_per_px(2.0)
            # Live manager (click mapping) got the corrected value + resolution.
            self.assertTrue(mgr.set_calls)
            self.assertAlmostEqual(mgr.set_calls[-1][1], 2.0)
            self.assertEqual(mgr.set_calls[-1][2], (912, 686))
            # Objective store (mosaic fallback + restore) is ground truth.
            cal = get_store().get_calibration("TestCamProp", "10x")
            self.assertIsNotNone(cal)
            self.assertAlmostEqual(cal["measured_um_per_px"], 2.0, places=4)
            self.assertAlmostEqual(float(cal.get("rotation_deg")), 12.0)
        finally:
            try:
                get_store().clear_calibration("TestCamProp", "10x")
            except Exception:
                pass

    def test_propagate_without_objective_only_hits_manager(self):
        mgr = self._Mgr()
        d = self._dlg(mgr, cam_key=None, objective=None)
        d._propagate_um_per_px(3.0)
        self.assertTrue(mgr.set_calls)
        self.assertAlmostEqual(mgr.set_calls[-1][1], 3.0)


class TestScaleFovThreadsKeys(unittest.TestCase):
    def test_dialog_stores_objective_keys(self):
        from gui.dialogs.scale_fov_calibration_dialog import (
            ScaleFovCalibrationDialog)
        d = ScaleFovCalibrationDialog(None, None, cam_idx=0,
                                      align_key="K|10x", objective="10x",
                                      cam_key="K")
        self.assertEqual(d._align_key, "K|10x")
        self.assertEqual(d._objective, "10x")
        self.assertEqual(d._cam_key, "K")


# ═══════════════════════════════════════════════════════════════════
#  Fourier-Mellin registration + method selection (manual backup)
# ═══════════════════════════════════════════════════════════════════

@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestFourierMellin(unittest.TestCase):
    def _tex(self):
        import cv2
        rng = np.random.RandomState(3)
        g = rng.randint(0, 255, (200, 200), np.uint8)
        g = cv2.GaussianBlur(g, (5, 5), 0)
        cv2.circle(g, (70, 90), 18, 255, -1)
        cv2.rectangle(g, (120, 60), (150, 100), 20, -1)
        return g

    def test_recovers_translation(self):
        from SupportClasses.MosaicBuilder import (
            _register_overlap_fourier_mellin as fm)
        g = self._tex()
        h, w = g.shape
        b = np.zeros_like(g)
        b[4:, 9:] = g[:h - 4, :w - 9]           # content shifted +9,+4
        r = fm(g, b)
        self.assertIsNotNone(r)
        dx, dy, rot, scale, conf = r
        # Correction to ADD to B to align onto A = −9, −4.
        self.assertAlmostEqual(dx, -9, delta=1.5)
        self.assertAlmostEqual(dy, -4, delta=1.5)
        self.assertLess(abs(rot), 1.5)
        self.assertLess(abs(scale - 1.0), 0.05)
        self.assertGreater(conf, 0.3)

    def test_recovers_rotation_and_scale(self):
        import cv2
        from SupportClasses.MosaicBuilder import (
            _register_overlap_fourier_mellin as fm)
        g = self._tex()
        h, w = g.shape
        c = (w / 2.0, h / 2.0)
        m = cv2.getRotationMatrix2D(c, 8.0, 1.1)   # +8°, ×1.1
        b = cv2.warpAffine(g, m, (w, h))
        r = fm(g, b)
        self.assertIsNotNone(r)
        _dx, _dy, rot, scale, _conf = r
        # The recovered transform is the CORRECTION (inverse of what was applied).
        self.assertAlmostEqual(rot, -8.0, delta=2.5)
        self.assertAlmostEqual(scale, 1.0 / 1.1, delta=0.06)

    def test_flat_returns_none(self):
        from SupportClasses.MosaicBuilder import (
            _register_overlap_fourier_mellin as fm)
        flat = np.full((64, 64), 100, np.uint8)
        self.assertIsNone(fm(flat, flat))


@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestRegistrationMethodSelection(unittest.TestCase):
    def _two_tiles(self, method, err=7):
        base = _texture(200, 400, seed=11)
        tile0 = base[0:200, 0:200].copy()
        tile1 = base[0:200, 150:350].copy()
        b = MosaicBuilder(frame_size_px=(200, 200), micron_per_pixel=1.0,
                          overlap=0.25, retain_for_reorient=True,
                          registration_method=method)
        b._composite = np.zeros((220, 380, 3), np.float64)
        b._weight_sum = np.zeros((220, 380), np.float64)
        b._display_cache = np.zeros((220, 380, 3), np.uint8)
        b._mosaic_scale = 1.0
        b._canvas_origin_um = (0.0, 0.0)
        b._reorient_tiles = [(tile0, 10, 10, 200, 200),
                             (tile1, 160 + err, 10, 200, 200)]
        return b

    def test_off_method_is_noop(self):
        b = self._two_tiles("off")
        _comp, n_edges, corr = b.optimize_registration()
        self.assertEqual(n_edges, 0)             # manual/stage-only → no registration
        self.assertEqual(corr, 0.0)

    def test_fourier_mellin_default_corrects(self):
        b = self._two_tiles("fourier_mellin")
        _comp, n_edges, _corr = b.optimize_registration()
        self.assertGreaterEqual(n_edges, 1)
        self.assertLess(abs(b._optimized_positions[1][0] - 160.0), 2.5)

    def test_phase_method_corrects(self):
        b = self._two_tiles("phase")
        _comp, n_edges, _corr = b.optimize_registration()
        self.assertGreaterEqual(n_edges, 1)
        self.assertLess(abs(b._optimized_positions[1][0] - 160.0), 2.5)


class TestMosaicSettingsRegMethod(unittest.TestCase):
    def test_default_is_fourier_mellin(self):
        from gui.dialogs.mosaic_settings_dialog import merged_settings
        self.assertEqual(merged_settings({})["reg_method"], "fourier_mellin")

    def test_dialog_round_trips_reg_method(self):
        from gui.dialogs.mosaic_settings_dialog import MosaicScanSettingsDialog
        d = MosaicScanSettingsDialog({"reg_method": "off"})
        self.assertEqual(d.values()["reg_method"], "off")


# ═══════════════════════════════════════════════════════════════════
#  Mosaic reads the calibrated orientation as ground truth
# ═══════════════════════════════════════════════════════════════════

class TestMosaicOrientationGroundTruth(unittest.TestCase):
    def _fresh_store(self):
        import tempfile
        from pathlib import Path
        import SupportClasses.CameraCalibrationStore as CCS
        CCS._store = CCS.CameraCalibrationStore(
            Path(tempfile.mkdtemp()) / "cam.json")
        return CCS

    class _HW:
        def camera_for_role(self, role):
            return 0

    def test_reads_from_store_when_manager_unsynced(self):
        CCS = self._fresh_store()
        CCS.get_store().set_rotation("ID1", 90.0, name="Cam")
        CCS.get_store().set_mirrored("ID1", True, name="Cam")
        from gui.pages.calibration import CalibrationPage

        class _Mgr:
            def camera_identity(self, i):
                return ("ID1", "Cam")

            def get_rotation_deg(self, i):
                return 0.0                    # manager came up un-synced

            def get_mirrored(self, i):
                return False

        class _Fake:
            _camera_manager = _Mgr()
            _hardware_config = TestMosaicOrientationGroundTruth._HW()

            def _ploc_microscope_cam_idx(self):
                return 0

        rot, mir, _fy = CalibrationPage._ploc_microscope_frame_orientation(_Fake())
        self.assertAlmostEqual(rot, 90.0)     # ground truth, not the un-synced mgr
        self.assertTrue(mir)

    def test_manager_fills_field_absent_from_store(self):
        CCS = self._fresh_store()
        CCS.get_store().set_rotation("ID1", 45.0, name="Cam")   # rotation only
        from gui.pages.calibration import CalibrationPage

        class _Mgr:
            def camera_identity(self, i):
                return ("ID1", "Cam")

            def get_rotation_deg(self, i):
                return 45.0

            def get_mirrored(self, i):
                return True                   # live mirror, not yet persisted

        class _Fake:
            _camera_manager = _Mgr()
            _hardware_config = TestMosaicOrientationGroundTruth._HW()

            def _ploc_microscope_cam_idx(self):
                return 0

        rot, mir, _fy = CalibrationPage._ploc_microscope_frame_orientation(_Fake())
        self.assertAlmostEqual(rot, 45.0)     # store
        self.assertTrue(mir)                  # manager fills the absent field


class TestOrientationPersistsImmediately(unittest.TestCase):
    """The correction dialog's flip/rotate must persist to the per-identity store
    IMMEDIATELY (not only on a separate Apply) so the full mosaic applies it when
    it places each tile — operator: "when I run the mosaic it is not flipping the
    x axis"."""

    class _Mgr:
        def view_orientation(self, i):
            return (False, 0.0)

        def set_mirrored(self, i, v):
            pass

        def set_rotation_deg(self, i, v):
            pass

        def camera_identity(self, i):
            return ("IDMOS", "Cam")

    class _Ctrl:
        is_zp_connected = False
        safety_limits = None

        def get_xy_position(self, cached=False):
            return (0.0, 0.0)

    def _dlg(self):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        return MosaicCalibrationDialog(
            self._Ctrl(), self._Mgr(), 0, safe_z=1.0, align_key="K", store=None,
            settings={}, center_um=(0.0, 0.0), frame_size=(912, 686),
            um_per_px_camera=1.5)

    def _fresh_store(self):
        import tempfile
        from pathlib import Path
        import SupportClasses.CameraCalibrationStore as CCS
        CCS._store = CCS.CameraCalibrationStore(
            Path(tempfile.mkdtemp()) / "cam.json")
        return CCS

    def test_flip_x_persists_mirror_to_store(self):
        CCS = self._fresh_store()
        d = self._dlg()
        d._orient_flip_h()                       # "Flip X axis"
        entry = CCS.get_store().get_calibration("IDMOS")
        self.assertIsNotNone(entry)
        self.assertTrue(entry.get("mirrored"))

    def test_rotate_persists_rotation_to_store(self):
        CCS = self._fresh_store()
        d = self._dlg()
        d._orient_rotate(90.0)
        entry = CCS.get_store().get_calibration("IDMOS")
        self.assertIsNotNone(entry)
        self.assertAlmostEqual(float(entry.get("rotation_deg")), 90.0)


# ═══════════════════════════════════════════════════════════════════
#  Flip Y — independent vertical flip across mosaic / mapping / display
# ═══════════════════════════════════════════════════════════════════

@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestFlipYMosaic(unittest.TestCase):
    def test_orient_tile_flips_vertically(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          frame_flip_y=True)
        f = np.zeros((6, 8, 3), np.uint8)
        f[1, 3] = (255, 255, 255)             # row 1, col 3
        o = b._orient_tile(f)
        ys, xs = np.where(o[:, :, 0] > 128)
        self.assertEqual((int(ys[0]), int(xs[0])), (4, 3))   # row 6-1-1=4, col 3

    def test_flip_x_and_flip_y_independent(self):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          frame_mirrored=True, frame_flip_y=True)
        f = np.zeros((6, 8, 3), np.uint8)
        f[1, 3] = (255, 255, 255)
        o = b._orient_tile(f)
        ys, xs = np.where(o[:, :, 0] > 128)
        # both flips: row 4, col 8-1-3=4
        self.assertEqual((int(ys[0]), int(xs[0])), (4, 4))


class TestFlipYManagerAndStore(unittest.TestCase):
    def test_manager_get_set_and_full_orientation(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr.set_flip_y(0, True)
        mgr.set_mirrored(0, True)
        mgr.set_rotation_deg(0, 30.0)
        self.assertTrue(mgr.get_flip_y(0))
        fx, fy, rot = mgr.full_orientation(0)
        self.assertTrue(fx)
        self.assertTrue(fy)
        self.assertAlmostEqual(rot, 30.0)

    def test_store_flip_y_roundtrip_and_pops(self):
        import tempfile
        from pathlib import Path
        import SupportClasses.CameraCalibrationStore as CCS
        CCS._store = CCS.CameraCalibrationStore(
            Path(tempfile.mkdtemp()) / "cam.json")
        st = CCS.get_store()
        st.set_rotation("ID", 10.0, name="Cam")
        st.set_flip_y("ID", True, name="Cam")
        self.assertTrue(st.get_flip_y("ID"))
        self.assertTrue(st.get_calibration("ID").get("flip_y"))
        st.set_flip_y("ID", False)             # False pops the key
        self.assertFalse(st.get_flip_y("ID"))
        self.assertNotIn("flip_y", st.get_calibration("ID"))
        self.assertAlmostEqual(
            st.get_calibration("ID")["rotation_deg"], 10.0)   # sibling kept

    def test_pixel_to_stage_offset_flips_dy(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr.set_um_per_px(0, 2.0)
        mgr.set_flip_y(0, True)
        # a point 10 px BELOW centre → flip Y negates dy → dy_um = -(10)·2 = -20.
        dx, dy = mgr.pixel_to_stage_offset(0, 50.0, 60.0, 100, 100)
        self.assertAlmostEqual(dx, 0.0, places=3)
        self.assertAlmostEqual(dy, -20.0, places=3)


class TestFlipYDisplay(unittest.TestCase):
    def test_feed_view_flip_y_transform(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        from PySide6.QtGui import QImage
        from PySide6.QtCore import QPointF
        v = CameraFeedView(camera_manager=None, cam_idx=0,
                           enable_settings=False)
        v._last_image_size = (6, 4)
        v.set_view_orientation(mirrored=False, rotation_deg=0.0, flip_y=True)
        img = QImage(6, 4, QImage.Format_RGB888)
        img.fill(0)
        disp, xf = v._orient_qimage(img)
        self.assertIsNotNone(xf)
        self.assertEqual((disp.width(), disp.height()), (6, 4))
        p = xf.map(QPointF(2.0, 1.0))          # y flips: 1 → (4 − 1) = 3
        self.assertAlmostEqual(p.x(), 2.0, places=3)
        self.assertAlmostEqual(p.y(), 3.0, places=3)


if __name__ == "__main__":
    unittest.main()
