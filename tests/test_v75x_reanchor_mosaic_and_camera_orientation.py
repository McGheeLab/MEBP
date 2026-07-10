"""
test_v75x_reanchor_mosaic_and_camera_orientation.py

v7.5.x — two operator-requested plate-calibration changes:

  Part A — the manual **Re-anchor map** now moves the mosaic IMAGE and the well
  centres AS ONE UNIT (previously it shifted only the well centres), and runs a
  NO-TRAVEL flow: the operator clicks a feature in the live microscope view (its
  true absolute stage position) then clicks the same feature on the plate
  overview (its map position); the difference E shifts everything. The stage
  never moves during re-anchor.

  Part B — the microscope camera's ROTATION relative to the stage axes is
  calibrated (arbitrary angle) and APPLIED in
  ``CameraManager.pixel_to_stage_offset`` so a live-view click maps to the
  correct XY direction even when the camera is mounted rotated (≈180°, but "not
  exactly"). With no rotation it is byte-identical to the legacy identity map.

Covers:
  * pixel_to_stage_offset — identity at θ=0/None; rotates by θ; a "sign-lock"
    round-trip that drives the real ``plus_column_direction_deg`` measurement and
    proves a click centres the feature.
  * CameraCalibrationStore.set_rotation / get_rotation — rotation-only write that
    preserves µm/px + image_correction siblings, persists, and clears on None.
  * CalibrationPage._ploc_shift_mosaic_by — shifts + persists the mosaic overlay.
  * CalibrationPage._ploc_apply_global_translation — shifts wells AND mosaic by
    the same E; still shifts wells when no mosaic is loaded.
  * The no-travel re-anchor flow — live-then-overview computes E = live − map with
    no stage motion.
"""

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

# A QApplication must exist before ANY QObject (CameraManager is a QObject) is
# constructed, or PySide6 aborts natively. Create it at import time.
_APP = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.camera_manager import CameraManager  # noqa: E402
from gui.dialogs.pixel_calibration_dialog import (  # noqa: E402
    plus_column_direction_deg)
from SupportClasses.CameraCalibrationStore import CameraCalibrationStore  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402


def _app():
    return QApplication.instance() or QApplication(sys.argv)


def _norm(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


# ── Part B — pixel_to_stage_offset rotation ──────────────────────────

class TestPixelToStageRotation(unittest.TestCase):
    def setUp(self):
        self.mgr = CameraManager(max_cameras=3)

    def test_identity_when_no_rotation(self):
        """θ None → the legacy identity mapping (dx_px·µm/px), unchanged."""
        self.mgr.set_um_per_px(0, 2.0)
        # click 10 px right, 20 px down of centre in a 100×100 frame
        dx, dy = self.mgr.pixel_to_stage_offset(0, 60.0, 70.0, 100, 100)
        self.assertAlmostEqual(dx, 20.0, places=6)
        self.assertAlmostEqual(dy, 40.0, places=6)

    def test_zero_rotation_is_identity(self):
        self.mgr.set_um_per_px(0, 2.0)
        self.mgr.set_rotation_deg(0, 0.0)
        dx, dy = self.mgr.pixel_to_stage_offset(0, 60.0, 70.0, 100, 100)
        self.assertAlmostEqual(dx, 20.0, places=6)
        self.assertAlmostEqual(dy, 40.0, places=6)

    def test_rotate_90(self):
        """R(90)·(dx,dy) = (−dy, dx)."""
        self.mgr.set_um_per_px(0, 2.0)
        self.mgr.set_rotation_deg(0, 90.0)
        dx, dy = self.mgr.pixel_to_stage_offset(0, 60.0, 70.0, 100, 100)
        # raw (20,40) → R(90) → (-40, 20)
        self.assertAlmostEqual(dx, -40.0, places=6)
        self.assertAlmostEqual(dy, 20.0, places=6)

    def test_rotate_180(self):
        self.mgr.set_um_per_px(0, 2.0)
        self.mgr.set_rotation_deg(0, 180.0)
        dx, dy = self.mgr.pixel_to_stage_offset(0, 60.0, 70.0, 100, 100)
        self.assertAlmostEqual(dx, -20.0, places=6)
        self.assertAlmostEqual(dy, -40.0, places=6)

    def test_plus_column_direction_formula(self):
        """Lock the measurement convention the rotation sign is derived from."""
        # commanded +X, content moves +X → θ = 0 − 0 − 180 = −180
        self.assertAlmostEqual(
            plus_column_direction_deg(0.0, 10.0, 0.0), -180.0, places=4)
        # commanded +X, content moves −X → θ = 0 − 180 − 180 ≡ 0
        self.assertAlmostEqual(
            _norm(plus_column_direction_deg(0.0, -10.0, 0.0)), 0.0, places=4)

    def test_centering_roundtrip_signlock(self):
        """SIGN LOCK. Simulate the dialog's forward model for a chosen camera
        rotation, feed it through the REAL ``plus_column_direction_deg`` to get
        the stored angle θ, set it, then prove that the stage move
        ``pixel_to_stage_offset`` returns for a clicked feature makes the
        content shift exactly cancel the feature offset (i.e. centres it).

        Forward model (dialog docstring): content shift p = −scale·R(α)·m, with
        the stored θ = −α and scale = 1/µm_per_px. So a stage move m produces
        content p = −(1/upp)·R(−θ)·m; substituting m = pixel_to_stage_offset must
        give p = −p_feature.
        """
        upp = 2.0
        for theta_true in (0.0, 30.0, 90.0, 180.0, -45.0, 178.5):
            self.mgr.set_um_per_px(0, upp)
            self.mgr.set_rotation_deg(0, theta_true)

            # Sanity: the measurement inverts to the same angle.
            commanded = 17.0
            phi = commanded - theta_true - 180.0
            r = 42.0
            meas_dx = r * math.cos(math.radians(phi))
            meas_dy = r * math.sin(math.radians(phi))
            measured_theta = plus_column_direction_deg(commanded, meas_dx, meas_dy)
            self.assertAlmostEqual(
                _norm(measured_theta), _norm(theta_true), places=4,
                msg=f"measurement did not recover θ={theta_true}")

            # A feature 15 px right, 25 px up of centre in a 200×200 frame.
            W = H = 200
            fx, fy = 15.0, -25.0            # p_feature (px, image convention)
            Sx, Sy = self.mgr.pixel_to_stage_offset(
                0, W / 2 + fx, H / 2 + fy, W, H)
            # content shift produced by moving the stage by (Sx,Sy):
            #   p = −(1/upp)·R(−θ)·S
            a = math.radians(-theta_true)
            ca, sa = math.cos(a), math.sin(a)
            px_c = -(1.0 / upp) * (Sx * ca - Sy * sa)
            py_c = -(1.0 / upp) * (Sx * sa + Sy * ca)
            self.assertAlmostEqual(px_c, -fx, places=5,
                                   msg=f"θ={theta_true}: x not centred")
            self.assertAlmostEqual(py_c, -fy, places=5,
                                   msg=f"θ={theta_true}: y not centred")


# ── Part B — CameraCalibrationStore rotation-only write ──────────────

class TestStoreRotation(unittest.TestCase):
    def setUp(self):
        self._dir = tempfile.mkdtemp()
        self.path = Path(self._dir) / "cams.json"
        self.store = CameraCalibrationStore(self.path)

    def test_set_get_rotation_roundtrip_and_persist(self):
        self.store.set_calibration("id1", 5.0, name="Cam")   # µm/px only
        self.store.set_rotation("id1", 178.5)
        self.assertAlmostEqual(self.store.get_rotation("id1"), 178.5, places=4)
        # µm/px sibling preserved
        self.assertAlmostEqual(
            self.store.get_calibration("id1")["um_per_px"], 5.0, places=6)
        # persists across a fresh load
        store2 = CameraCalibrationStore(self.path)
        self.assertAlmostEqual(store2.get_rotation("id1"), 178.5, places=4)
        self.assertAlmostEqual(
            store2.get_calibration("id1")["um_per_px"], 5.0, places=6)

    def test_rotation_preserves_image_correction_sibling(self):
        self.store.set_image_correction(
            "id2", brightness=5, contrast=1.2, gamma=0.9)
        self.store.set_rotation("id2", 42.0)
        self.assertAlmostEqual(self.store.get_rotation("id2"), 42.0, places=4)
        corr = self.store.get_image_correction("id2")
        self.assertIsNotNone(corr)
        self.assertEqual(corr["brightness"], 5)

    def test_none_clears_rotation_keeps_siblings(self):
        self.store.set_calibration("id3", 3.0)
        self.store.set_rotation("id3", 10.0)
        self.store.set_rotation("id3", None)
        self.assertIsNone(self.store.get_rotation("id3"))
        # µm/px survives the clear
        self.assertAlmostEqual(
            self.store.get_calibration("id3")["um_per_px"], 3.0, places=6)

    def test_get_rotation_missing_is_none(self):
        self.assertIsNone(self.store.get_rotation("nope"))
        self.assertIsNone(self.store.get_rotation(""))


class TestObjectiveStoreRotationPreserved(unittest.TestCase):
    """Adversarial-review fix: a µm/px-only set_calibration (rotation_deg=None)
    must NOT silently drop a previously measured per-objective rotation."""

    def setUp(self):
        from SupportClasses.ObjectiveCalibration import ObjectiveCalibrationStore
        self._dir = tempfile.mkdtemp()
        self.store = ObjectiveCalibrationStore(
            Path(self._dir) / "objectives.json")
        self.store.add_objective("4x", 4.0)

    def test_umpx_only_update_keeps_rotation(self):
        self.store.set_calibration("CamA", "4x", 0.8, (900, 600),
                                   rotation_deg=178.5)
        # µm/px-only re-calibration (rotation not measured this time)
        self.store.set_calibration("CamA", "4x", 0.82, (900, 600))
        cal = self.store.get_calibration("CamA", "4x")
        self.assertAlmostEqual(cal["rotation_deg"], 178.5, places=3)
        self.assertAlmostEqual(cal["measured_um_per_px"], 0.82, places=6)

    def test_explicit_rotation_still_wins(self):
        self.store.set_calibration("CamA", "4x", 0.8, (900, 600),
                                   rotation_deg=10.0)
        self.store.set_calibration("CamA", "4x", 0.8, (900, 600),
                                   rotation_deg=20.0)
        self.assertAlmostEqual(
            self.store.get_calibration("CamA", "4x")["rotation_deg"], 20.0,
            places=3)


# ── Part A — re-anchor mosaic shift + no-travel flow ─────────────────

class _FakeLiveView:
    image_size = (100, 100)

    def set_overlay_vector(self, *a, **k):
        pass


class _FakeMgr:
    """Minimal CameraManager stand-in for the re-anchor click math."""
    def __init__(self, offset=(500.0, -300.0)):
        self._offset = offset

    def pixel_to_stage_offset(self, idx, px, py, w, h):
        return self._offset

    def is_um_per_px_calibrated(self, idx):
        return True

    def get_rotation_deg(self, idx):
        return None


class TestReanchorMosaicShift(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.get_xy_position.return_value = (10000.0, 20000.0)
        ctrl.default_plate_center_um.return_value = (56000.0, 39000.0)
        ctrl.plate_axis_sign.return_value = (1.0, 1.0)
        ctrl.plate_flip_180.return_value = False
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(24)
        page._ploc_plate_key = lambda: "test-plate"
        return page, ctrl

    def _wire_overlay(self, page, ext=(0.0, 0.0, 100.0, 200.0), has=True):
        """Stub the mosaic overlay + store so the shift can be observed without
        the real GUI overlay pipeline."""
        page._ploc_overlay_img = object()
        page._ploc_overlay_ext = ext
        self.saved = []
        store = MagicMock()
        store.has.return_value = has
        store.get_meta.return_value = {
            "um_per_px": 3.0, "mosaic_scale": 0.01, "frames": 5}
        store.save.side_effect = lambda *a, **k: self.saved.append((a, k))
        page._ploc_mosaic_store = lambda: store
        self.pushed = []

        def _fake_set_overlay(img, e):
            page._ploc_overlay_img = img
            page._ploc_overlay_ext = tuple(e)
            self.pushed.append(tuple(e))
        page._ploc_set_overlay_image = _fake_set_overlay
        return store

    def test_shift_mosaic_by_updates_extent_and_persists(self):
        page, _ = self._page()
        store = self._wire_overlay(page)
        ok = page._ploc_shift_mosaic_by(10.0, 20.0)
        self.assertTrue(ok)
        self.assertEqual(page._ploc_overlay_ext, (10.0, 20.0, 110.0, 220.0))
        self.assertEqual(self.pushed, [(10.0, 20.0, 110.0, 220.0)])
        # persisted with the shifted extent
        self.assertEqual(len(self.saved), 1)
        args, _ = self.saved[0]
        self.assertEqual(args[2], (10.0, 20.0, 110.0, 220.0))

    def test_shift_mosaic_by_zero_is_noop(self):
        page, _ = self._page()
        self._wire_overlay(page)
        self.assertFalse(page._ploc_shift_mosaic_by(0.0, 0.0))
        self.assertEqual(self.saved, [])
        self.assertEqual(self.pushed, [])

    def test_shift_mosaic_by_no_overlay_returns_false(self):
        page, _ = self._page()
        page._ploc_overlay_img = None
        page._ploc_overlay_ext = None
        page._ploc_mosaic_store = lambda: MagicMock()
        self.assertFalse(page._ploc_shift_mosaic_by(10.0, 20.0))

    def test_shift_mosaic_by_malformed_extent_is_noop(self):
        """Adversarial-review fix: a malformed cached extent (not 4-element)
        must degrade to a no-op, not an IndexError."""
        page, _ = self._page()
        page._ploc_overlay_img = object()
        page._ploc_overlay_ext = (1.0, 2.0)      # 2-element = malformed
        page._ploc_mosaic_store = lambda: MagicMock()
        self.assertFalse(page._ploc_shift_mosaic_by(10.0, 20.0))

    def test_shift_mosaic_by_in_memory_only_still_shifts_view(self):
        """Store has no mosaic for this plate (in-memory composite) → view
        shifts but nothing persisted."""
        page, _ = self._page()
        store = self._wire_overlay(page, has=False)
        ok = page._ploc_shift_mosaic_by(5.0, -5.0)
        self.assertTrue(ok)
        self.assertEqual(page._ploc_overlay_ext, (5.0, -5.0, 105.0, 195.0))
        store.save.assert_not_called()

    def _wire_wells(self, page):
        predicted = {"A1": (0.0, 0.0), "A2": (19300.0, 0.0), "B1": (0.0, 19300.0)}
        page._predicted_positions = dict(predicted)
        page._calibrated_positions = dict(predicted)
        page._reference_markers = {}
        page._xy_teach_points = {}
        page._refresh_ploc_view = lambda: None
        page._emit_calibration_data_changed = lambda: None
        page._save_calibration = lambda: None
        return predicted

    def test_reanchor_shifts_wells_and_mosaic_together(self):
        page, _ = self._page()
        self._wire_overlay(page)
        predicted = self._wire_wells(page)
        n = page._ploc_apply_global_translation(50.0, -50.0)
        self.assertEqual(n, len(predicted))
        # wells shifted by E
        self.assertAlmostEqual(page._calibrated_positions["A1"][0], 50.0)
        self.assertAlmostEqual(page._calibrated_positions["A1"][1], -50.0)
        self.assertAlmostEqual(page._calibrated_positions["A2"][0], 19350.0)
        # mosaic shifted by the SAME E
        self.assertEqual(page._ploc_overlay_ext, (50.0, -50.0, 150.0, 150.0))

    def test_reanchor_no_mosaic_still_shifts_wells(self):
        page, _ = self._page()
        page._ploc_overlay_img = None
        page._ploc_overlay_ext = None
        page._ploc_mosaic_store = lambda: MagicMock(has=lambda *a: False)
        predicted = self._wire_wells(page)
        n = page._ploc_apply_global_translation(50.0, -50.0)
        self.assertEqual(n, len(predicted))
        self.assertAlmostEqual(page._calibrated_positions["A1"][0], 50.0)
        self.assertIsNone(page._ploc_overlay_ext)


class TestReanchorNoTravelFlow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _page(self, mgr):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.get_xy_position.return_value = (10000.0, 20000.0)
        ctrl.default_plate_center_um.return_value = (56000.0, 39000.0)
        ctrl.plate_axis_sign.return_value = (1.0, 1.0)
        ctrl.plate_flip_180.return_value = False
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        # Build with no camera_manager (the real one is a QObject with a
        # .cameras list the page reads in __init__), then inject the fake used
        # only for the click-math methods under test.
        page = CalibrationPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(24)
        page._camera_manager = mgr
        page._ploc_live_view = _FakeLiveView()
        page._ploc_live_cam_idx = 0
        return page, ctrl

    def test_live_then_overview_computes_offset_with_no_travel(self):
        mgr = _FakeMgr(offset=(500.0, -300.0))
        page, ctrl = self._page(mgr)
        # travel primitive must never be called during no-travel re-anchor
        page._ploc_safe_goto = MagicMock()
        recorded = {}
        page._ploc_apply_global_translation = (
            lambda ex, ey: recorded.setdefault("E", (ex, ey)) or 3)

        page._ploc_reanchor_stage = "await_live"
        page._ploc_reanchor_live_abs = None

        # Step 1: live click. feature_abs = (10000+500, 20000-300)
        page._ploc_reanchor_live_click(60.0, 45.0)
        self.assertEqual(page._ploc_reanchor_live_abs, (10500.0, 19700.0))
        self.assertEqual(page._ploc_reanchor_stage, "await_overview")

        # Step 2: overview click at the feature's MAP position (10450, 19750).
        page._ploc_reanchor_overview(10450.0, 19750.0, "pt")
        # E = live − map = (50, −50)
        self.assertIn("E", recorded)
        self.assertAlmostEqual(recorded["E"][0], 50.0, places=6)
        self.assertAlmostEqual(recorded["E"][1], -50.0, places=6)
        # re-anchor ends (state cleared) and the stage never moved
        self.assertIsNone(page._ploc_reanchor_stage)
        page._ploc_safe_goto.assert_not_called()

    def test_overview_first_without_live_is_aborted(self):
        mgr = _FakeMgr()
        page, _ = self._page(mgr)
        page._ploc_apply_global_translation = MagicMock(return_value=0)
        page._ploc_reanchor_stage = "await_overview"
        page._ploc_reanchor_live_abs = None       # no live pick yet
        page._ploc_reanchor_overview(1.0, 2.0, "pt")
        page._ploc_apply_global_translation.assert_not_called()


if __name__ == "__main__":
    unittest.main()
