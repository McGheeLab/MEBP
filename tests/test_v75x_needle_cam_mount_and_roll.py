"""
test_v75x_needle_cam_mount_and_roll.py
======================================

v7.5.x — Needle cameras re-mounted symmetric ±45° about stage +X: the conflated
per-camera ``rotation_deg`` is split into ``column_dir_deg`` (the ±45° mount
direction, aligner-only) and ``rotation_deg`` (the small display ROLL —
deviation of the measured stage-motion vector from parallel).

See `coding plans/Update plans/MEBP_v75x_NEEDLE_CAM_SYMMETRIC_45_AND_VIEW_ROLL.md`.

Covers:
- ``fold_parallel_deg`` folds to (−90, 90] (a vector and its negation give the
  same roll).
- SIGN PIN: composing ``view_roll_from_displacement`` with the actual display
  transform (``camera_feed_view.view_transform_coeffs``) renders the measured
  motion vector LEVEL, for any vector angle incl. mirror/flip-Y combos.
- ``CameraCalibrationStore``: column_dir round-trip, sibling preservation,
  None-clears, ``set_calibration(column_dir_deg=)``.
- Store migration v1.0 → v1.1: a needle-assigned legacy ``rotation_deg`` MOVES
  to ``column_dir_deg`` (display roll becomes absent → level view); non-needle
  entries untouched; idempotent.
- ``CameraManager`` column-dir get/set semantics.
- Hardware Setup commit paths: ``set_calibrated_um_per_px`` writes both fields;
  ``_apply_slot_column_dir`` pushes manager + store and never touches rotation.
- The needle-location offset compute REFUSES when either camera's column
  direction is unmeasured (no silent fall back to the legacy 90°/0° mapping),
  and recovers a known offset with the ±45° symmetric pair.
- ``needle_role_label`` display names.
"""

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.dialogs.pixel_calibration_dialog import (
    fold_parallel_deg, view_roll_from_displacement,
)
from gui.widgets.camera_feed_view import view_transform_coeffs
from gui.widgets.camera_manager import CameraManager
from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
import SupportClasses.CameraCalibrationStore as CCS
from SupportClasses.HardwareConfig import (
    HardwareConfig, CameraRole, needle_role_label,
)
from SupportClasses.VisionDetector import (
    TwoCameraNeedleAligner, TwoCameraEdgePicks,
)


class TestFoldParallel(unittest.TestCase):
    def test_fold_table(self):
        cases = [
            (0.0, 0.0),
            (45.0, 45.0),
            (90.0, 90.0),
            (91.0, -89.0),
            (-90.0, 90.0),
            (135.0, -45.0),
            (-135.0, 45.0),
            (179.0, -1.0),
            (180.0, 0.0),
            (-179.0, 1.0),
            (360.0, 0.0),
        ]
        for angle, want in cases:
            self.assertAlmostEqual(
                fold_parallel_deg(angle), want, places=9,
                msg=f"fold({angle}) != {want}")

    def test_vector_and_negation_fold_equal(self):
        for phi in (-170.0, -95.0, -30.0, 10.0, 60.0, 120.0):
            self.assertAlmostEqual(
                fold_parallel_deg(phi), fold_parallel_deg(phi + 180.0),
                places=9)


class TestRollSignPin(unittest.TestCase):
    """THE sign anchor: applying the display transform with the computed roll
    must render the measured motion vector horizontal. If either side's sign
    convention drifts, this breaks — do not weaken it."""

    def _displayed(self, dx, dy, mirrored, flip_y, rot_deg):
        m11, m12, m21, m22 = view_transform_coeffs(mirrored, flip_y, rot_deg)
        return (dx * m11 + dy * m21, dx * m12 + dy * m22)

    def test_roll_levels_measured_vector(self):
        for mirrored in (False, True):
            for flip_y in (False, True):
                for phi in (-160.0, -91.0, -45.0, -10.0, 0.0,
                            25.0, 89.0, 90.0, 135.0, 178.0):
                    dx = math.cos(math.radians(phi))
                    dy = math.sin(math.radians(phi))
                    roll = view_roll_from_displacement(
                        dx, dy, mirrored=mirrored, flip_y=flip_y)
                    _dx2, dy2 = self._displayed(
                        dx, dy, mirrored, flip_y, roll)
                    self.assertAlmostEqual(
                        dy2, 0.0, places=9,
                        msg=(f"phi={phi} mir={mirrored} fy={flip_y} "
                             f"roll={roll} -> dy'={dy2}"))

    def test_roll_is_small_for_near_level_vector(self):
        # A nearly-horizontal measured vector (small sensor roll) must yield a
        # small correction — NOT ±45 (the mount angle never enters here).
        self.assertAlmostEqual(
            view_roll_from_displacement(10.0, 0.35), -2.0, places=1)
        self.assertAlmostEqual(
            view_roll_from_displacement(-10.0, -0.35), -2.0, places=1)

    def test_roll_bounded_to_quarter_turn(self):
        for phi in range(-180, 181, 7):
            dx = math.cos(math.radians(phi))
            dy = math.sin(math.radians(phi))
            r = view_roll_from_displacement(dx, dy)
            self.assertTrue(-90.0 <= r < 90.0 + 1e-9, f"phi={phi} -> {r}")


class TestStoreColumnDir(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        self.store = CameraCalibrationStore(self.tmp)

    def test_round_trip(self):
        self.store.set_column_dir("id_a", -44.6, name="Cam A")
        self.assertAlmostEqual(self.store.get_column_dir("id_a"), -44.6)
        reloaded = CameraCalibrationStore(self.tmp)
        self.assertAlmostEqual(reloaded.get_column_dir("id_a"), -44.6)

    def test_none_clears(self):
        self.store.set_column_dir("id_a", 45.0)
        self.store.set_column_dir("id_a", None)
        self.assertIsNone(self.store.get_column_dir("id_a"))
        entry = self.store.get_calibration("id_a")
        self.assertNotIn("column_dir_deg", entry)

    def test_siblings_preserved(self):
        self.store.set_calibration("id_a", 5.54, rotation_deg=1.2, name="A")
        self.store.set_mirrored("id_a", True)
        self.store.set_column_dir("id_a", 44.0)
        entry = self.store.get_calibration("id_a")
        self.assertAlmostEqual(entry["um_per_px"], 5.54)
        self.assertAlmostEqual(entry["rotation_deg"], 1.2)
        self.assertTrue(entry["mirrored"])
        self.assertAlmostEqual(entry["column_dir_deg"], 44.0)

    def test_set_calibration_kwarg_and_preserve_on_umpx_only(self):
        self.store.set_calibration(
            "id_a", 5.54, rotation_deg=0.8, column_dir_deg=-45.3)
        entry = self.store.get_calibration("id_a")
        self.assertAlmostEqual(entry["column_dir_deg"], -45.3)
        # µm/px-only update preserves both angles.
        self.store.set_calibration("id_a", 5.60)
        entry = self.store.get_calibration("id_a")
        self.assertAlmostEqual(entry["um_per_px"], 5.60)
        self.assertAlmostEqual(entry["rotation_deg"], 0.8)
        self.assertAlmostEqual(entry["column_dir_deg"], -45.3)

    def test_unknown_identity_none(self):
        self.assertIsNone(self.store.get_column_dir("nope"))
        self.assertIsNone(self.store.get_column_dir(""))


class TestStoreMigration(unittest.TestCase):
    def _write(self, data):
        tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
        return tmp

    def test_legacy_needle_rotation_moves_to_column_dir(self):
        tmp = self._write({
            "version": "1.0",
            "cameras": {
                "id_nx": {"um_per_px": 5.5, "rotation_deg": -44.0,
                          "name": "N1"},
                "id_ny": {"um_per_px": 5.4, "rotation_deg": 136.0,
                          "name": "N2"},
                "id_scope": {"um_per_px": 1.6, "rotation_deg": 2.0,
                             "name": "Scope"},
            },
            "assignments": {
                "needle_x": "id_nx",
                "needle_y": "id_ny",
                "microscope": "id_scope",
            },
        })
        store = CameraCalibrationStore(tmp)
        # Needle entries: rotation MOVED to column_dir (legacy semantic).
        self.assertAlmostEqual(store.get_column_dir("id_nx"), -44.0)
        self.assertIsNone(store.get_rotation("id_nx"))
        self.assertAlmostEqual(store.get_column_dir("id_ny"), 136.0)
        self.assertIsNone(store.get_rotation("id_ny"))
        # Non-needle entry untouched.
        self.assertAlmostEqual(store.get_rotation("id_scope"), 2.0)
        self.assertIsNone(store.get_column_dir("id_scope"))
        # Version bumped to CURRENT + persisted. Asserted against the constant,
        # not a literal: a legacy file walks the whole migration chain (1.0 →
        # 1.1 → 1.2 → …), so pinning "1.1" here would break on every future
        # schema bump even though this test is only about the needle split.
        self.assertEqual(store._data["version"], CCS.SCHEMA_VERSION)
        with open(tmp, encoding="utf-8") as f:
            on_disk = json.load(f)
        self.assertEqual(on_disk["version"], CCS.SCHEMA_VERSION)
        self.assertAlmostEqual(
            on_disk["cameras"]["id_nx"]["column_dir_deg"], -44.0)

    def test_migration_idempotent(self):
        tmp = self._write({
            "version": "1.0",
            "cameras": {"id_nx": {"rotation_deg": 45.0}},
            "assignments": {"needle_x": "id_nx"},
        })
        CameraCalibrationStore(tmp)
        # Second load: a fresh needle ROLL written post-migration must survive.
        store2 = CameraCalibrationStore(tmp)
        store2.set_rotation("id_nx", 1.5)  # new-semantics roll
        store3 = CameraCalibrationStore(tmp)
        self.assertAlmostEqual(store3.get_rotation("id_nx"), 1.5)
        self.assertAlmostEqual(store3.get_column_dir("id_nx"), 45.0)

    def test_v11_store_not_migrated(self):
        tmp = self._write({
            "version": "1.1",
            "cameras": {"id_nx": {"rotation_deg": 1.2}},  # a genuine roll
            "assignments": {"needle_x": "id_nx"},
        })
        store = CameraCalibrationStore(tmp)
        self.assertAlmostEqual(store.get_rotation("id_nx"), 1.2)
        self.assertIsNone(store.get_column_dir("id_nx"))

    def test_unassigned_needle_entries_untouched(self):
        tmp = self._write({
            "version": "1.0",
            "cameras": {"id_x": {"rotation_deg": 44.0}},
            "assignments": {},
        })
        store = CameraCalibrationStore(tmp)
        self.assertAlmostEqual(store.get_rotation("id_x"), 44.0)
        self.assertIsNone(store.get_column_dir("id_x"))


class TestManagerColumnDir(unittest.TestCase):
    def setUp(self):
        self.mgr = CameraManager(max_cameras=3)

    def test_default_none(self):
        for i in range(3):
            self.assertIsNone(self.mgr.get_column_dir_deg(i))

    def test_set_get_independent_of_rotation(self):
        self.mgr.set_column_dir_deg(1, -45.0)
        self.mgr.set_rotation_deg(1, 1.2)
        self.assertEqual(self.mgr.get_column_dir_deg(1), -45.0)
        self.assertEqual(self.mgr.get_rotation_deg(1), 1.2)
        self.assertIsNone(self.mgr.get_column_dir_deg(0))

    def test_clear_with_none(self):
        self.mgr.set_column_dir_deg(2, 45.0)
        self.mgr.set_column_dir_deg(2, None)
        self.assertIsNone(self.mgr.get_column_dir_deg(2))

    def test_out_of_range_safe(self):
        self.assertIsNone(self.mgr.get_column_dir_deg(99))
        self.mgr.set_column_dir_deg(99, 10.0)  # must not raise

    def test_view_orientation_never_returns_column_dir(self):
        # THE display fix: the ±45° mount direction must not reach the view.
        self.mgr.set_column_dir_deg(0, 45.0)
        _mir, rot = self.mgr.view_orientation(0)
        self.assertEqual(rot, 0.0)
        self.mgr.set_rotation_deg(0, 1.5)  # the small roll DOES reach it
        _mir, rot = self.mgr.view_orientation(0)
        self.assertEqual(rot, 1.5)


class TestCommitPaths(unittest.TestCase):
    """Hardware Setup commit paths write the split fields."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        self._orig_store = CCS._store
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def tearDown(self):
        CCS._store = self._orig_store

    def _fake_mgr(self):
        mgr = SimpleNamespace()
        mgr.calls = []
        # v7.5.x: set_um_per_px gained a ``resolution=`` kwarg (the frame size the
        # value was measured at, so it can be rescaled).
        mgr.set_um_per_px = (
            lambda i, v, resolution=None: mgr.calls.append(("umpx", i, v)))
        mgr.set_rotation_deg = lambda i, v: mgr.calls.append(("rot", i, v))
        mgr.set_column_dir_deg = lambda i, v: mgr.calls.append(("cdir", i, v))
        mgr.camera_identity = lambda i: ("id_test", "Test Cam")
        return mgr

    def _bind(self, stub, *names):
        from gui.pages.hardware_setup import HardwareSetupPage
        for n in names:
            setattr(stub, n, getattr(HardwareSetupPage, n).__get__(stub))

    def test_set_calibrated_um_per_px_writes_both(self):
        mgr = self._fake_mgr()
        stub = SimpleNamespace(_camera_manager=mgr)
        self._bind(stub, "set_calibrated_um_per_px")
        stub.set_calibrated_um_per_px(
            0, 5.54, rotation_deg=1.2, column_dir_deg=-44.6)
        self.assertIn(("umpx", 0, 5.54), mgr.calls)
        self.assertIn(("rot", 0, 1.2), mgr.calls)
        self.assertIn(("cdir", 0, -44.6), mgr.calls)
        entry = CCS.get_store().get_calibration("id_test")
        self.assertAlmostEqual(entry["rotation_deg"], 1.2)
        self.assertAlmostEqual(entry["column_dir_deg"], -44.6)

    def test_set_calibrated_um_per_px_without_angles_leaves_them(self):
        # The microscope/objective path passes neither — pre-existing values
        # must be preserved (None = untouched).
        CCS.get_store().set_calibration(
            "id_test", 5.0, rotation_deg=0.7, column_dir_deg=44.0)
        mgr = self._fake_mgr()
        stub = SimpleNamespace(_camera_manager=mgr)
        self._bind(stub, "set_calibrated_um_per_px")
        stub.set_calibrated_um_per_px(0, 6.0)
        entry = CCS.get_store().get_calibration("id_test")
        self.assertAlmostEqual(entry["um_per_px"], 6.0)
        self.assertAlmostEqual(entry["rotation_deg"], 0.7)
        self.assertAlmostEqual(entry["column_dir_deg"], 44.0)
        self.assertNotIn(("rot", 0, None), mgr.calls)

    def test_apply_slot_column_dir_pushes_manager_and_store(self):
        mgr = self._fake_mgr()
        stub = SimpleNamespace(
            _camera_manager=mgr,
            _refresh_slot_rotation_displays=lambda: None,
        )
        self._bind(stub, "_apply_slot_column_dir")
        stub._apply_slot_column_dir(1, 45.2)
        self.assertIn(("cdir", 1, 45.2), mgr.calls)
        self.assertAlmostEqual(
            CCS.get_store().get_column_dir("id_test"), 45.2)
        # Never touches the display rotation.
        self.assertFalse(any(c[0] == "rot" for c in mgr.calls))

    def test_restore_pushes_both_fields(self):
        CCS.get_store().set_calibration(
            "id_test", 5.5, rotation_deg=0.9, column_dir_deg=-45.1)
        mgr = self._fake_mgr()
        mgr.set_image_correction = lambda *a, **k: None
        mgr.set_mirrored = lambda *a, **k: None
        mgr.set_flip_y = lambda *a, **k: None
        mgr.set_um_per_px_with_resolution = None
        combo = SimpleNamespace(currentData=lambda: ("src", 0))
        stub = SimpleNamespace(
            _camera_manager=mgr,
            _live_cam_source_combos=[combo],
            _sync_correction_sliders=lambda i: None,
            _mark_camera_calibrated=lambda i, v: None,
        )
        from gui.pages.hardware_setup import HardwareSetupPage
        stub._restore_calibration_for_slot = (
            HardwareSetupPage._restore_calibration_for_slot.__get__(stub))
        stub._restore_calibration_for_slot(0)
        self.assertIn(("rot", 0, 0.9), mgr.calls)
        self.assertIn(("cdir", 0, -45.1), mgr.calls)


class _FakeCam:
    def __init__(self, h=480, w=640):
        self._frame = np.zeros((h, w, 3), dtype=np.uint8)

    def get_current_frame(self):
        return self._frame


class TestOffsetComputeRefusal(unittest.TestCase):
    """The needle-location offset compute must REFUSE when either camera's
    column direction is unmeasured — never fall back to the legacy 90°/0°."""

    def _page(self, dirs=(None, None)):
        from gui.pages.calibration import CalibrationPage
        mgr = CameraManager(max_cameras=3)
        mgr._cameras[0] = _FakeCam()
        mgr._cameras[1] = _FakeCam()
        mgr.set_um_per_px(0, 5.0)
        mgr.set_um_per_px(1, 5.0)
        mgr.set_column_dir_deg(0, dirs[0])
        mgr.set_column_dir_deg(1, dirs[1])
        cal = CalibrationPage(None, camera_manager=mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        cal._hardware_config = cfg
        return cal

    def test_refuses_when_both_unmeasured(self):
        cal = self._page((None, None))
        cal._needle_loc_picks = TwoCameraEdgePicks(310, 330, 315, 325)
        with self.assertRaises(ValueError) as ctx:
            cal._needle_loc_compute_offset_um()
        self.assertIn("calibrat", str(ctx.exception).lower())

    def test_refuses_when_one_unmeasured(self):
        cal = self._page((45.0, None))
        cal._needle_loc_picks = TwoCameraEdgePicks(310, 330, 315, 325)
        with self.assertRaises(ValueError):
            cal._needle_loc_compute_offset_um()

    def test_computes_with_measured_pm45(self):
        cal = self._page((45.0, -45.0))
        # Perfectly centered picks → zero move.
        cal._needle_loc_picks = TwoCameraEdgePicks(300, 340, 300, 340)
        dx, dy = cal._needle_loc_compute_offset_um()
        self.assertAlmostEqual(dx, 0.0, places=6)
        self.assertAlmostEqual(dy, 0.0, places=6)

    def test_ignores_display_rotation(self):
        # A stored display ROLL must not change the centering math.
        cal = self._page((45.0, -45.0))
        cal._camera_manager.set_rotation_deg(0, 30.0)
        cal._camera_manager.set_rotation_deg(1, -12.0)
        cal._needle_loc_picks = TwoCameraEdgePicks(300, 340, 300, 340)
        dx, dy = cal._needle_loc_compute_offset_um()
        self.assertAlmostEqual(dx, 0.0, places=6)
        self.assertAlmostEqual(dy, 0.0, places=6)


class TestSymmetric45EndToEnd(unittest.TestCase):
    def test_pm45_pair_recovers_known_offset(self):
        """Cams at +45°/−45° about +X recover an arbitrary needle offset."""
        W = 100
        c = W / 2.0
        a1, a2 = 45.0, -45.0
        for n in ((100.0, 0.0), (0.0, 80.0), (-30.0, 55.0)):
            s1 = (n[0] * math.cos(math.radians(a1))
                  + n[1] * math.sin(math.radians(a1)))
            s2 = (n[0] * math.cos(math.radians(a2))
                  + n[1] * math.sin(math.radians(a2)))
            picks = TwoCameraEdgePicks(
                x_view_left_px=c + s1, x_view_right_px=c + s1,
                y_view_left_px=c + s2, y_view_right_px=c + s2)
            a = TwoCameraNeedleAligner(
                1.0, 1.0, W, W, angle_x_view_deg=a1, angle_y_view_deg=a2)
            dx, dy = a.offset_from_edge_clicks(picks)
            self.assertAlmostEqual(dx, n[0], places=6, msg=f"n={n}")
            self.assertAlmostEqual(dy, n[1], places=6, msg=f"n={n}")

    def test_swapped_pair_also_works(self):
        """The cameras are interchangeable — swapping which role carries which
        measured direction still recovers the same offset."""
        W = 100
        c = W / 2.0
        n = (60.0, -40.0)
        a1, a2 = -45.0, 45.0  # roles swapped vs the sibling test
        s1 = (n[0] * math.cos(math.radians(a1))
              + n[1] * math.sin(math.radians(a1)))
        s2 = (n[0] * math.cos(math.radians(a2))
              + n[1] * math.sin(math.radians(a2)))
        picks = TwoCameraEdgePicks(c + s1, c + s1, c + s2, c + s2)
        a = TwoCameraNeedleAligner(
            1.0, 1.0, W, W, angle_x_view_deg=a1, angle_y_view_deg=a2)
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, n[0], places=6)
        self.assertAlmostEqual(dy, n[1], places=6)


class TestRoleLabels(unittest.TestCase):
    def test_needle_labels(self):
        self.assertEqual(needle_role_label(CameraRole.NEEDLE_X),
                         "Needle cam 1")
        self.assertEqual(needle_role_label(CameraRole.NEEDLE_Y),
                         "Needle cam 2")

    def test_enum_values_unchanged(self):
        # Serialized in configs + the store's assignments — must not change.
        self.assertEqual(CameraRole.NEEDLE_X.value, "needle_x")
        self.assertEqual(CameraRole.NEEDLE_Y.value, "needle_y")


if __name__ == "__main__":
    unittest.main()
