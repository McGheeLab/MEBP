"""
test_v75x_camera_calibration_store.py — Per-machine camera calibration store.

v7.5.x: Needle/plate camera µm/px + rotation are stored per *physical camera*
(device identity) in a machine-level JSON store, NOT in the swappable
HardwareConfig — so loading a saved hardware-setup file no longer wipes them,
and they auto-restore on the next start. The store also remembers which camera
identity plays each role so the source assignment auto-restores after a detect.

Covers:
- CameraCalibrationStore: set/get/clear, rotation preservation, assignments,
  disk persistence round-trip.
- camera_identity.source_for_identity (inverse of identity_for_source).
- Integration: a calibration survives a hardware-setup-file load and
  auto-restores into a fresh CameraManager on the next session.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
from gui.widgets import camera_identity as ci


class TestStore(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        self.store = CameraCalibrationStore(self.tmp)

    def test_set_get(self):
        self.store.set_calibration("dshow:A", 5.54, rotation_deg=45.0,
                                   name="Teslong")
        e = self.store.get_calibration("dshow:A")
        self.assertAlmostEqual(e["um_per_px"], 5.54, places=6)
        self.assertEqual(e["rotation_deg"], 45.0)
        self.assertEqual(e["name"], "Teslong")
        self.assertIsNone(self.store.get_calibration("dshow:missing"))

    def test_rotation_preserved_on_umpx_only_update(self):
        self.store.set_calibration("dshow:A", 5.5, rotation_deg=45.0)
        self.store.set_calibration("dshow:A", 6.0)  # no rotation given
        e = self.store.get_calibration("dshow:A")
        self.assertAlmostEqual(e["um_per_px"], 6.0, places=6)
        self.assertEqual(e["rotation_deg"], 45.0)   # preserved

    def test_disk_round_trip(self):
        self.store.set_calibration("dshow:A", 5.54, rotation_deg=45.0,
                                   name="Teslong")
        self.store.set_assignment("needle_x", "dshow:A")
        reloaded = CameraCalibrationStore(self.tmp)
        self.assertAlmostEqual(
            reloaded.get_calibration("dshow:A")["um_per_px"], 5.54, places=6)
        self.assertEqual(reloaded.get_assignment("needle_x"), "dshow:A")

    def test_clear(self):
        self.store.set_calibration("dshow:A", 1.0)
        self.store.clear_calibration("dshow:A")
        self.assertIsNone(self.store.get_calibration("dshow:A"))
        self.store.clear_calibration("dshow:A")  # idempotent, no raise

    def test_assignments(self):
        self.store.set_assignment("needle_x", "dshow:A")
        self.assertEqual(self.store.get_assignment("needle_x"), "dshow:A")
        self.store.set_assignment("needle_x", None)  # clear
        self.assertIsNone(self.store.get_assignment("needle_x"))


class TestSourceForIdentity(unittest.TestCase):
    def setUp(self):
        self.ds = [
            {"index": 0, "name": "Teslong", "device_path": "PATH_A"},
            {"index": 1, "name": "Teslong", "device_path": "PATH_B"},
        ]

    def test_dshow_resolves_to_present_index(self):
        self.assertEqual(
            ci.source_for_identity("dshow:PATH_B", self.ds), ("opencv", 1))

    def test_dshow_absent_returns_none(self):
        self.assertIsNone(ci.source_for_identity("dshow:PATH_X", self.ds))

    def test_other_kinds(self):
        self.assertEqual(
            ci.source_for_identity("opencv:2", self.ds), ("opencv", 2))
        self.assertEqual(
            ci.source_for_identity("toupcam:ABC", self.ds), ("toupcam", "ABC"))
        self.assertEqual(
            ci.source_for_identity("simulated:micro", self.ds),
            ("simulated", "micro"))
        self.assertIsNone(ci.source_for_identity("", self.ds))

    def test_round_trip_with_identity_for_source(self):
        ident = ci.identity_for_source(("opencv", 0), self.ds)
        self.assertEqual(
            ci.source_for_identity(ident[0], self.ds), ("opencv", 0))


class TestStorePersistAndAutoRestore(unittest.TestCase):
    """Integration: calibration survives a setup-file load and auto-restores
    into a fresh CameraManager on the next session (via the store)."""

    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)
        self.DS = [
            {"index": 0, "name": "Teslong", "device_path": "PATH_A"},
            {"index": 1, "name": "Teslong", "device_path": "PATH_B"},
        ]

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from gui.widgets.camera_manager import CameraManager
        from SupportClasses.HardwareConfig import HardwareConfig, CameraRole
        pg = HardwareSetupPage()
        mgr = CameraManager(max_cameras=3)
        pg.set_camera_manager(mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        pg.set_config(cfg)
        mgr._ds_cameras = self.DS
        mgr.get_source = lambda i: ("opencv", i)
        return pg, mgr

    def test_persist_survives_file_load_and_auto_restores(self):
        from SupportClasses.HardwareConfig import HardwareConfig

        # Session 1: assign sources + calibrate.
        pg, mgr = self._page()
        for i in (0, 1):
            c = pg._live_cam_source_combos[i]
            c.blockSignals(True)
            c.addItem(f"cam{i}", ("opencv", i))
            c.setCurrentIndex(c.count() - 1)
            c.blockSignals(False)
            pg._remember_assignment(i)
        pg.set_calibrated_um_per_px(0, 5.54, rotation_deg=45.0)
        pg.set_calibrated_um_per_px(1, 5.49, rotation_deg=-45.0)

        # Loading a hardware-setup file (no camera cal) must NOT wipe the store.
        pg.set_config(HardwareConfig())
        self.assertIsNotNone(
            self.CCS.get_store().get_calibration("dshow:PATH_A"))

        # Session 2: fresh page/manager; only the on-disk store persists.
        self.CCS._store = self.CCS.CameraCalibrationStore(self.tmp)
        pg2, mgr2 = self._page()
        for i in (0, 1):
            c = pg2._live_cam_source_combos[i]
            c.clear()
            c.addItem("— None —", None)
            c.addItem(f"cam{i}", ("opencv", i))
        pg2._auto_assign_sources_from_store()

        self.assertTrue(mgr2.is_um_per_px_calibrated(0))
        self.assertAlmostEqual(mgr2.get_um_per_px(0), 5.54, places=6)
        self.assertEqual(mgr2.get_rotation_deg(0), 45.0)
        self.assertTrue(mgr2.is_um_per_px_calibrated(1))
        self.assertAlmostEqual(mgr2.get_um_per_px(1), 5.49, places=6)
        self.assertEqual(mgr2.get_rotation_deg(1), -45.0)


class TestAutoDetectOnShow(unittest.TestCase):
    """The Hardware Setup page auto-detects cameras on first display ONLY when
    the store remembers a previous setup, so a calibrated needle setup restores
    without a manual Detect — but a first-ever run doesn't probe unprompted."""

    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from gui.widgets.camera_manager import CameraManager
        pg = HardwareSetupPage()
        mgr = CameraManager(max_cameras=3)
        calls = []
        # Stub the (blocking, hardware-touching) detect so we just record it.
        pg._on_detect_live_cameras = lambda: calls.append(True)
        pg.set_camera_manager(mgr)
        return pg, calls

    def test_no_assignments_does_not_auto_detect(self):
        pg, calls = self._page()
        pg._maybe_auto_detect_cameras()
        self.assertEqual(calls, [])
        self.assertFalse(pg._auto_detect_done)

    def test_remembered_setup_schedules_detect_once(self):
        self.CCS.get_store().set_assignment("needle_x", "dshow:PATH_A")
        pg, calls = self._page()
        # The deferred detect is scheduled via QTimer; drive the event loop.
        self.assertTrue(pg._auto_detect_done)
        _app.processEvents()
        self.assertEqual(calls, [True])
        # Re-entry is a no-op (guard already tripped).
        pg._maybe_auto_detect_cameras()
        _app.processEvents()
        self.assertEqual(calls, [True])


if __name__ == "__main__":
    unittest.main()
