"""
test_v75x_mosaic_orientation_adjust.py — interactive mosaic orientation fix.

v7.5.x (operator): fix a mirrored/rotated camera by flipping/rotating the mosaic
tiles until the composite reads correctly, re-rendering instantly, then saving
the result as the camera's mirror + rotation calibration.

Covers the two enabling primitives:
  1. MosaicBuilder retains the small canvas-res tiles and re-blends the whole
     composite with a NEW orientation (``set_frame_orientation`` +
     ``reblend_reoriented``) without re-scanning.
  2. CameraFeedView.set_view_orientation transforms the DISPLAYED feed and
     inverts clicks back to raw frame coords (so pixel_to_stage_offset is
     unaffected).
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QImage

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.MosaicBuilder import MosaicBuilder, CV2_AVAILABLE


# ═══════════════════════════════════════════════════════════════════
#  MosaicBuilder retain + reblend
# ═══════════════════════════════════════════════════════════════════

@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestBuilderReblend(unittest.TestCase):
    def _built(self, **kw):
        """A tiny 2-tile mosaic with a bright marker off-centre in each tile."""
        b = MosaicBuilder(frame_size_px=(20, 20), micron_per_pixel=1.0,
                          overlap=0.1, target_mosaic_px=200,
                          retain_for_reorient=True, **kw)
        b.generate_raster_positions((0.0, 0.0, 20.0, 0.0))  # inits the canvas
        for i, sx in enumerate((0.0, 18.0)):
            f = np.zeros((20, 20, 3), dtype=np.uint8)
            f[4, 4] = (255, 255, 255)     # top-left quadrant marker
            b.add_raster_frame(f, sx, 0.0, index=i)
            b.stitch_incremental()
        return b

    def test_tiles_retained(self):
        b = self._built()
        self.assertTrue(b.has_reorient_tiles())
        self.assertGreaterEqual(len(b._reorient_tiles), 2)

    def test_reblend_changes_composite(self):
        b = self._built()
        before = b.composite.copy()
        b.set_frame_orientation(rotation_deg=180.0, mirrored=False)
        after = b.reblend_reoriented()
        self.assertIsNotNone(after)
        # 180° moves every marker to the opposite corner → composite differs.
        self.assertFalse(np.array_equal(before, after))

    def test_reblend_identity_restores(self):
        b = self._built()
        before = b.composite.copy()
        b.set_frame_orientation(rotation_deg=90.0, mirrored=True)
        b.reblend_reoriented()
        b.set_frame_orientation(rotation_deg=0.0, mirrored=False)
        restored = b.reblend_reoriented()
        # Back to the original orientation → close to the original composite.
        self.assertTrue(np.allclose(before, restored, atol=2))

    def test_free_accumulators_kept_when_retaining(self):
        b = self._built()
        b.free_accumulators()          # no-op while retain_for_reorient
        self.assertIsNotNone(b._composite)
        self.assertTrue(b.has_reorient_tiles())
        b.free_reorient()              # explicit release
        self.assertFalse(b.has_reorient_tiles())
        self.assertIsNone(b._composite)


# ═══════════════════════════════════════════════════════════════════
#  CameraFeedView view orientation + click inversion
# ═══════════════════════════════════════════════════════════════════

def _qimage(w, h):
    img = QImage(w, h, QImage.Format_RGB888)
    img.fill(0)
    return img


@unittest.skipUnless(CV2_AVAILABLE, "cv2 required")
class TestTileImagesOrientation(unittest.TestCase):
    """tile_images_px applies the calibrated orientation to each tile, so the
    calibration dialog's per-image registration view reflects flips/rotations."""

    def _built(self, **kw):
        b = MosaicBuilder(frame_size_px=(8, 6), micron_per_pixel=1.0,
                          overlap=0.1, target_mosaic_px=100, **kw)
        b.generate_raster_positions((0.0, 0.0, 8.0, 0.0))
        f = np.zeros((6, 8, 3), dtype=np.uint8)
        f[1, 1] = (255, 255, 255)   # bright at (row 1, col 1)
        b.add_raster_frame(f, 0.0, 0.0, index=0)
        b.stitch_incremental()
        return b

    def test_default_returns_raw(self):
        tiles = self._built().tile_images_px()
        self.assertEqual(len(tiles), 1)
        ys, xs = np.where(tiles[0][0][:, :, 0] > 128)
        self.assertEqual((int(ys[0]), int(xs[0])), (1, 1))  # unchanged

    def test_mirror_flips_tile(self):
        frame = self._built(frame_mirrored=True).tile_images_px()[0][0]
        ys, xs = np.where(frame[:, :, 0] > 128)
        self.assertEqual((int(ys[0]), int(xs[0])), (1, 6))  # col 1 → 8-1-1=6

    def test_set_frame_orientation_reorients(self):
        b = self._built()
        b.set_frame_orientation(rotation_deg=0.0, mirrored=True)
        ys, xs = np.where(b.tile_images_px()[0][0][:, :, 0] > 128)
        self.assertEqual((int(ys[0]), int(xs[0])), (1, 6))


class TestCalibrationDialogOrientation(unittest.TestCase):
    """The mosaic-calibration dialog seeds orientation from the camera, its
    controls compose flip/rotate, and Apply persists to the calibration store."""

    class _Ctrl:
        is_zp_connected = False
        safety_limits = None

        def get_xy_position(self, cached=False):
            return (0.0, 0.0)

    def _dlg(self, mgr):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        return MosaicCalibrationDialog(
            self._Ctrl(), mgr, 0, safe_z=1.0, align_key="k", store=None,
            settings={}, center_um=(0.0, 0.0), frame_size=(916, 686),
            um_per_px_camera=1.5)

    def test_seeds_from_camera(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr.set_mirrored(0, True)
        mgr.set_rotation_deg(0, 90.0)
        dlg = self._dlg(mgr)
        self.assertTrue(dlg._orient_mir)
        self.assertAlmostEqual(dlg._orient_rot, 90.0)
        self.assertFalse(dlg._orient_box.isEnabled())   # until a build

    def test_flip_and_rotate_compose(self):
        from gui.widgets.camera_manager import CameraManager
        dlg = self._dlg(CameraManager())
        dlg._orient_rotate(90.0)
        self.assertAlmostEqual(dlg._orient_rot, 90.0)
        # v7.5.x: flip X and flip Y are INDEPENDENT axis flips — flip X toggles
        # the horizontal mirror and leaves the rotation untouched.
        dlg._orient_flip_h()
        self.assertAlmostEqual(dlg._orient_rot, 90.0)
        self.assertTrue(dlg._orient_mir)
        self.assertFalse(dlg._orient_fy)
        dlg._orient_flip_v()                    # independent vertical flip
        self.assertTrue(dlg._orient_fy)
        self.assertTrue(dlg._orient_mir)        # flip X unchanged
        self.assertAlmostEqual(dlg._orient_rot, 90.0)

    def test_apply_persists_to_store(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        import SupportClasses.CameraCalibrationStore as CCS
        CCS._store = CCS.CameraCalibrationStore(
            Path(tempfile.mkdtemp()) / "cam.json")
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr._ds_cameras = [{"index": 0, "name": "Cam", "device_path": "P"}]
        mgr.get_source = lambda i: ("opencv", 0)
        dlg = self._dlg(mgr)
        dlg._orient_set(90.0, True, True)       # rot, flip_x, flip_y
        with patch("PySide6.QtWidgets.QMessageBox.information"):
            dlg._orient_apply_to_camera()   # modal confirmation is mocked
        ident = mgr.camera_identity(0)
        entry = CCS.get_store().get_calibration(ident[0])
        self.assertTrue(entry["mirrored"])
        self.assertTrue(entry["flip_y"])
        self.assertAlmostEqual(entry["rotation_deg"], 90.0)


class TestCalibrationViewPlateFrame(unittest.TestCase):
    """v7.5.x: the calibration mosaic view renders in the SAME plate-display
    frame as the full plate view (180° when plate_flip_180), so an orientation
    tuned in the objective/calibration dialog is WYSIWYG vs the full mosaic —
    it no longer looks 'upside down / on the wrong side' there."""

    def _view_with_tiles(self):
        from gui.widgets.mosaic_registration_view import MosaicRegistrationView
        v = MosaicRegistrationView()
        v.resize(300, 240)
        frame = np.zeros((20, 30, 3), dtype=np.uint8)
        frame[5, 5] = (255, 255, 255)
        v.set_tiles([(frame, 0.0, 0.0, 30.0, 20.0)], 1.0)
        return v

    def test_no_flip_by_default(self):
        v = self._view_with_tiles()
        self.assertFalse(v._flip_180)
        self.assertGreater(v.transform().m11(), 0.0)   # upright

    def test_flip_180_rotates_the_view(self):
        v = self._view_with_tiles()
        v.set_flip_180(True)
        self.assertTrue(v._flip_180)
        # A 180° whole-view rotation flips the vertical scale sign.
        self.assertLess(v.transform().m11(), 0.0)

    def test_flip_toggles_back(self):
        v = self._view_with_tiles()
        v.set_flip_180(True)
        v.set_flip_180(False)
        self.assertFalse(v._flip_180)
        self.assertGreater(v.transform().m11(), 0.0)


class TestDialogSeedsPlateFrame(unittest.TestCase):
    """MosaicCalibrationDialog resolves plate_flip_180 from the controller and
    seeds the view, so the frame matches the plate view with no extra wiring."""

    class _CtrlFlip:
        is_zp_connected = False
        safety_limits = None

        def get_xy_position(self, cached=False):
            return (0.0, 0.0)

        def plate_flip_180(self):
            return True

    def test_dialog_seeds_flip_from_controller(self):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        from gui.widgets.camera_manager import CameraManager
        dlg = MosaicCalibrationDialog(
            self._CtrlFlip(), CameraManager(), 0, safe_z=1.0, align_key="k",
            store=None, settings={}, center_um=(0.0, 0.0), frame_size=(916, 686),
            um_per_px_camera=1.5)
        self.assertTrue(dlg._plate_flip_180)
        self.assertTrue(dlg._view._flip_180)

    def test_dialog_explicit_flip_overrides(self):
        from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        from gui.widgets.camera_manager import CameraManager
        dlg = MosaicCalibrationDialog(
            self._CtrlFlip(), CameraManager(), 0, safe_z=1.0, align_key="k",
            store=None, settings={}, center_um=(0.0, 0.0), frame_size=(916, 686),
            um_per_px_camera=1.5, plate_flip_180=False)
        self.assertFalse(dlg._plate_flip_180)
        self.assertFalse(dlg._view._flip_180)


class TestFeedViewOrientation(unittest.TestCase):
    def _view(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        return CameraFeedView(camera_manager=None, cam_idx=0,
                              enable_settings=False)

    def test_noop_when_unset(self):
        v = self._view()
        img = _qimage(6, 4)
        disp, xf = v._orient_qimage(img)
        self.assertIs(disp, img)
        self.assertIsNone(xf)

    def test_mirror_transform_and_click_inverse(self):
        v = self._view()
        v._last_image_size = (6, 4)
        v.set_view_orientation(mirrored=True, rotation_deg=0.0)
        img = _qimage(6, 4)
        disp, xf = v._orient_qimage(img)
        self.assertIsNotNone(xf)
        self.assertEqual((disp.width(), disp.height()), (6, 4))
        # Horizontal flip in continuous image coords: x → (w − x), y unchanged.
        # This is what matters for click round-tripping (widget→displayed→raw).
        from PySide6.QtCore import QPointF
        p = xf.map(QPointF(1.0, 2.0))
        self.assertAlmostEqual(p.x(), 5.0, places=3)   # 6 − 1
        self.assertAlmostEqual(p.y(), 2.0, places=3)
        inv, ok = xf.inverted()
        self.assertTrue(ok)
        r = inv.map(p)                                  # round-trips back to raw
        self.assertAlmostEqual(r.x(), 1.0, places=3)
        self.assertAlmostEqual(r.y(), 2.0, places=3)

    def test_rotate_90_swaps_dims(self):
        v = self._view()
        v._last_image_size = (6, 4)
        v.set_view_orientation(mirrored=False, rotation_deg=90.0)
        disp, xf = v._orient_qimage(_qimage(6, 4))
        self.assertEqual((disp.width(), disp.height()), (4, 6))  # w/h swapped

    def test_edge_pick_mode_suppresses(self):
        v = self._view()
        v.set_view_orientation(mirrored=True, rotation_deg=90.0)
        v.set_edge_pick_mode(True)
        img = _qimage(6, 4)
        disp, xf = v._orient_qimage(img)
        self.assertIs(disp, img)
        self.assertIsNone(xf)


if __name__ == "__main__":
    unittest.main()
