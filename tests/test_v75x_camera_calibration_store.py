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

    def test_autostart_round_trip(self):
        self.assertFalse(self.store.get_autostart("dshow:A"))
        self.assertFalse(self.store.any_autostart())
        self.store.set_autostart("dshow:A", True)
        self.assertTrue(self.store.get_autostart("dshow:A"))
        self.assertTrue(self.store.any_autostart())
        # Survives reload and is a sibling that does not disturb µm/px.
        self.store.set_calibration("dshow:A", 5.0)
        reloaded = CameraCalibrationStore(self.tmp)
        self.assertTrue(reloaded.get_autostart("dshow:A"))
        self.assertAlmostEqual(
            reloaded.get_calibration("dshow:A")["um_per_px"], 5.0, places=6)
        self.store.set_autostart("dshow:A", False)
        self.assertFalse(self.store.get_autostart("dshow:A"))
        self.assertFalse(self.store.any_autostart())


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
        # v7.5.x (rotated rig): the needle-card commit passes the DISPLAY
        # roll as rotation_deg and the ±45° mount direction separately.
        pg.set_calibrated_um_per_px(
            0, 5.54, rotation_deg=1.2, column_dir_deg=45.0)
        pg.set_calibrated_um_per_px(
            1, 5.49, rotation_deg=-0.8, column_dir_deg=-45.0)

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
        self.assertEqual(mgr2.get_rotation_deg(0), 1.2)
        self.assertEqual(mgr2.get_column_dir_deg(0), 45.0)
        self.assertTrue(mgr2.is_um_per_px_calibrated(1))
        self.assertAlmostEqual(mgr2.get_um_per_px(1), 5.49, places=6)
        self.assertEqual(mgr2.get_rotation_deg(1), -0.8)
        self.assertEqual(mgr2.get_column_dir_deg(1), -45.0)


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
        # Stub the (blocking, hardware-touching) auto-load so we just record
        # that _maybe_auto_detect_cameras scheduled it.
        pg._auto_load_cameras = lambda: calls.append(True)
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


class TestSaveLoadCameraSetup(unittest.TestCase):
    """Save snapshots which cameras are running (autostart); Load / startup
    detect+restore and start exactly those cameras."""

    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

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

        # Stub the manager's hardware-touching surface.
        idents = {0: ("dshow:PATH_A", "Teslong"),
                  1: ("dshow:PATH_B", "Teslong")}
        self.running = {}
        self.started = []
        mgr.camera_identity = lambda i: idents.get(i)
        mgr.is_running = lambda i: bool(self.running.get(i))
        mgr.set_source = lambda i, src: None
        mgr.image_correction = lambda i: {
            "brightness": 0, "contrast": 1.0, "gamma": 1.0}
        mgr.get_hw_settings = lambda i: {"source": "none"}

        def _start(i):
            self.running[i] = True
            self.started.append(i)
        mgr.start = _start
        # v7.5.x: the startup auto-start (_start_saved_cameras) now opens OFF the
        # GUI thread via start_async to avoid freezing the UI. Stub it with the
        # same synchronous fake so the test intercepts the real code path.
        mgr.start_async = _start

        # Assign sources to slots 0 and 1.
        for i in (0, 1):
            c = pg._live_cam_source_combos[i]
            c.blockSignals(True)
            c.addItem(f"cam{i}", ("opencv", i))
            c.setCurrentIndex(c.count() - 1)
            c.blockSignals(False)
        return pg, mgr

    def test_save_flags_running_cameras_only(self):
        pg, mgr = self._page()
        self.running = {0: True, 1: False}
        pg._on_save_camera_settings()
        store = self.CCS.get_store()
        self.assertTrue(store.get_autostart("dshow:PATH_A"))
        self.assertFalse(store.get_autostart("dshow:PATH_B"))
        # Roles were remembered for both assigned slots.
        self.assertEqual(store.get_assignment("needle_x"), "dshow:PATH_A")
        self.assertEqual(store.get_assignment("needle_y"), "dshow:PATH_B")

    def test_start_saved_starts_only_flagged(self):
        pg, mgr = self._page()
        self.CCS.get_store().set_autostart("dshow:PATH_A", True)
        self.CCS.get_store().set_autostart("dshow:PATH_B", False)
        n = pg._start_saved_cameras()
        self.assertEqual(n, 1)
        self.assertEqual(self.started, [0])

    def test_already_running_not_restarted(self):
        pg, mgr = self._page()
        self.CCS.get_store().set_autostart("dshow:PATH_A", True)
        self.running = {0: True}  # already on
        n = pg._start_saved_cameras()
        self.assertEqual(n, 0)
        self.assertEqual(self.started, [])

    def test_save_then_start_round_trip(self):
        pg, mgr = self._page()
        self.running = {0: True, 1: True}
        pg._on_save_camera_settings()
        # New "session": both stopped; loading should start both.
        self.running = {}
        self.started = []
        n = pg._start_saved_cameras()
        self.assertEqual(n, 2)
        self.assertEqual(sorted(self.started), [0, 1])

    def test_set_source_runs_before_identity_check(self):
        # Regression: _start_saved_cameras must sync the source BEFORE resolving
        # camera_identity (which reads the manager's selected source). Model an
        # identity that only resolves once set_source has run for that slot.
        pg, mgr = self._page()
        store = self.CCS.get_store()
        store.set_autostart("dshow:PATH_A", True)
        store.set_autostart("dshow:PATH_B", False)
        idents = {0: ("dshow:PATH_A", "Teslong"),
                  1: ("dshow:PATH_B", "Teslong")}
        synced = {}
        mgr.set_source = lambda i, src: synced.__setitem__(i, src)
        mgr.camera_identity = lambda i: (idents.get(i) if i in synced else None)
        n = pg._start_saved_cameras()
        self.assertEqual(n, 1)            # only the flagged slot
        self.assertEqual(self.started, [0])
        self.assertIn(0, synced)          # set_source ran before identity check


class TestMicroscopeResolutionOnStart(unittest.TestCase):
    """The microscope applies its last-used (persisted) resolution to the
    device on start — even without an explicit Save — from the spec combo
    (camera_config.active_resolution). A per-camera hw_controls resolution,
    when present, takes precedence and the spec fallback is skipped."""

    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from gui.widgets.camera_manager import CameraManager
        from SupportClasses.HardwareConfig import HardwareConfig, CameraRole
        pg = HardwareSetupPage()
        mgr = CameraManager(max_cameras=3)
        pg.set_camera_manager(mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.MICROSCOPE)
        pg.set_config(cfg)

        self.res_calls = []
        mgr.camera_identity = lambda i: ("toupcam:TC", "ToupCam") if i == 0 else None
        mgr.is_running = lambda i: i == 0
        mgr.set_capture_resolution = lambda i, w, h: self.res_calls.append((i, w, h))
        mgr.log_hw_settings = lambda i, prefix="": None
        # _apply_hw_controls fans out to these — make them no-ops here.
        for name in ("set_hw_auto_exposure", "set_hw_exposure_us",
                     "set_hw_exposure_gain", "set_hw_gamma",
                     "set_hw_brightness", "set_hw_contrast"):
            setattr(mgr, name, lambda *a, **k: None)

        # The spec combo carries the persisted last-used resolution.
        pg.cam_resolution_combo.blockSignals(True)
        pg.cam_resolution_combo.clear()
        pg.cam_resolution_combo.addItem("916 × 686", [916, 686])
        pg.cam_resolution_combo.setCurrentIndex(0)
        pg.cam_resolution_combo.blockSignals(False)
        return pg, mgr

    def test_spec_resolution_applied_on_microscope_start(self):
        pg, mgr = self._page()
        pg._on_camera_started_hw(0)
        self.assertEqual(self.res_calls, [(0, 916, 686)])

    def test_active_resolution_is_single_source_for_microscope(self):
        # v7.5.x: the microscope resolution has ONE source of truth —
        # camera_config.active_resolution. A per-identity hw_controls resolution
        # must NOT override it (the camera is driven to active_resolution as
        # ground truth), so a stale per-identity value can't diverge from the
        # "Microscope Camera Setup" block.
        pg, mgr = self._page()
        self.CCS.get_store().set_hw_controls(
            "toupcam:TC", {"resolution": [1832, 1374]})
        pg._on_camera_started_hw(0)
        # active_resolution (916×686) wins; the hw_controls resolution is ignored
        # for the microscope.
        self.assertEqual(self.res_calls, [(0, 916, 686)])

    def test_non_microscope_start_does_not_apply_spec_resolution(self):
        pg, mgr = self._page()
        pg._on_camera_started_hw(1)  # slot 1 isn't the microscope
        self.assertEqual(self.res_calls, [])

    def test_microscope_start_flags_autostart(self):
        # Implicit "last-used": starting the microscope flags it to auto-start
        # next launch, so no explicit "Save Camera Settings" is needed.
        pg, mgr = self._page()
        store = self.CCS.get_store()
        self.assertFalse(store.get_autostart("toupcam:TC"))
        pg._on_camera_started_hw(0)
        self.assertTrue(store.get_autostart("toupcam:TC"))
        self.assertTrue(store.any_autostart())  # → startup auto-load triggers

    def test_non_microscope_start_does_not_flag_autostart(self):
        pg, mgr = self._page()
        pg._on_camera_started_hw(1)
        self.assertFalse(self.CCS.get_store().any_autostart())


class TestBackgroundStartupProbe(unittest.TestCase):
    """Startup auto-load probes OpenCV indices off the GUI thread, then finishes
    on the GUI thread (populate combos from the pre-probe + start saved
    cameras) — so the slow device opens never freeze the window."""

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
        pg.set_camera_manager(mgr)
        return pg, mgr

    def test_auto_load_backgrounds_probe_then_finishes_on_gui(self):
        pg, mgr = self._page()
        detect_args, started = [], []
        # Background probe → fire the callback synchronously with indices; the
        # callback (self._cameras_probed.emit) marshals to _finish_auto_load.
        mgr.detect_cameras_async = lambda cb: cb([0, 1])
        pg._on_detect_live_cameras = lambda idx=None: detect_args.append(idx)
        pg._start_saved_cameras = lambda: (started.append(True), 0)[1]
        pg._auto_load_cameras()
        # The pre-probed indices flowed through to detect (no GUI-thread reprobe)
        # and the saved cameras were started.
        self.assertEqual(detect_args, [[0, 1]])
        self.assertEqual(started, [True])

    def test_manager_detect_async_runs_off_thread_and_returns_indices(self):
        # The real async probe runs on a worker thread and delivers a result
        # (a list, or None on failure) — never raising on the caller's thread.
        pg, mgr = self._page()
        import threading
        done = threading.Event()
        box = {}

        def _cb(indices):
            box["indices"] = indices
            box["thread"] = threading.current_thread().name
            done.set()

        mgr.detect_cameras_async(_cb)
        self.assertTrue(done.wait(10.0), "background probe never completed")
        self.assertTrue(box["indices"] is None or isinstance(box["indices"], list))
        self.assertNotEqual(box["thread"], "MainThread")  # ran off the GUI thread


class TestSpecComboRestore(unittest.TestCase):
    """The Microscope 'Camera spec' combo auto-restores the saved camera on
    launch. Regression: the combo stores the catalog KEY ("BUC3D-1000C") as
    item data, but set_config restored by the spec's long .name → findData never
    matched → the dropdown reset to "None" and the operator re-picked it every
    start."""

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        return HardwareSetupPage()

    def _spec(self, name=None):
        import copy
        from SupportClasses.HardwareConfig import load_camera_catalog
        spec = copy.copy(load_camera_catalog()["BUC3D-1000C"])
        if name is not None:
            spec.name = name
        return spec

    def test_spec_combo_autoselects_buc3d(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        spec = self._spec()
        # Bug precondition: the catalog key differs from the spec's long name.
        self.assertNotEqual(spec.name, "BUC3D-1000C")
        cfg = HardwareConfig()
        cfg.camera_config.camera_spec = spec
        cfg.camera_config.active_resolution = (916, 686)
        pg = self._page()
        pg.set_config(cfg)
        self.assertEqual(pg.camera_combo.currentData(), "BUC3D-1000C")
        # Resolution restore is independent of the spec lookup and still works.
        self.assertEqual(
            tuple(pg.cam_resolution_combo.currentData() or ()), (916, 686))

    def test_spec_combo_legacy_name_equals_key(self):
        # Older saves that stored the catalog KEY as the spec name still resolve.
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.camera_config.camera_spec = self._spec(name="BUC3D-1000C")
        pg = self._page()
        pg.set_config(cfg)
        self.assertEqual(pg.camera_combo.currentData(), "BUC3D-1000C")

    def test_spec_combo_unknown_spec_stays_none(self):
        # A spec no longer in the catalog → combo stays unset (index 0), no crash.
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.camera_config.camera_spec = self._spec(name="Totally Unknown Cam XYZ")
        pg = self._page()
        pg.set_config(cfg)
        self.assertEqual(pg.camera_combo.currentIndex(), 0)

    def test_spec_round_trips_through_rebuild(self):
        # After restore selects BUC3D-1000C, saving (rebuild) yields the same
        # long-name spec — proving the KEY-as-data selection round-trips.
        from SupportClasses.HardwareConfig import HardwareConfig
        spec = self._spec()
        cfg = HardwareConfig()
        cfg.camera_config.camera_spec = spec
        cfg.camera_config.active_resolution = (916, 686)
        pg = self._page()
        pg.set_config(cfg)
        pg._rebuild_config()
        self.assertIsNotNone(pg._config.camera_config.camera_spec)
        self.assertEqual(pg._config.camera_config.camera_spec.name, spec.name)


class TestMicroscopeAutostartCleared(unittest.TestCase):
    """A deliberate user stop clears the microscope autostart flag, so it stays
    off next launch — but only via the toggle handler, not app shutdown."""

    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from gui.widgets.camera_manager import CameraManager
        from SupportClasses.HardwareConfig import HardwareConfig, CameraRole
        pg = HardwareSetupPage()
        mgr = CameraManager(max_cameras=3)
        pg.set_camera_manager(mgr)
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.MICROSCOPE)
        pg.set_config(cfg)
        self.running = {0: True}
        mgr.camera_identity = lambda i: ("toupcam:TC", "ToupCam") if i == 0 else None
        mgr.is_running = lambda i: bool(self.running.get(i))
        mgr.stop = lambda i: self.running.__setitem__(i, False)
        return pg, mgr

    def test_user_stop_clears_microscope_autostart(self):
        pg, mgr = self._page()
        store = self.CCS.get_store()
        store.set_autostart("toupcam:TC", True)
        pg._on_toggle_camera(0)  # microscope is running → this stops it
        self.assertFalse(store.get_autostart("toupcam:TC"))


if __name__ == "__main__":
    unittest.main()
