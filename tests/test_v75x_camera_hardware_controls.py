"""
test_v75x_camera_hardware_controls.py — Camera-side (hardware) controls.

v7.5.x: The BUC3D-1000C (ToupTek C3CMOS10000KPA) exposes exposure / gain /
gamma / brightness / contrast and resolution via the ToupCam SDK. This adds a
cross-backend hardware-control API (CameraWidget/CameraManager), an
authoritative readback (source-labelled), per-identity persistence, and a
gear-button pop-out dialog on controllable camera feeds.

The real SDK path was validated against hardware; these tests drive the
cross-backend logic with a fake ToupCam backend injected into a real
CameraWidget (the manager builds real widgets when OpenCV is present).

Covers:
- CameraCalibrationStore.get/set_hw_controls (round-trip, disk, None-drop,
  sibling preservation with um_per_px).
- CameraWidget hardware API: capabilities, source-labelled readback, set_*
  delegation, nearest-resolution mapping, stopped-camera safety, log readout.
- CameraManager delegation + log_hw_settings.
- CameraSettingsDialog: reload populates from device, live-apply, persistence.
- CameraFeedView gear: visible only when controllable; absent when disabled.
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
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager


class FakeToupCam:
    """Mimics ToupCamBackend's hardware-control surface (no DLL/hardware)."""

    HW_RANGES = {"brightness": (-64, 64, 0), "contrast": (-100, 100, 0),
                 "gamma": (20, 180, 100)}

    def __init__(self):
        self._device_id = "fake:C3CMOS10000KPA"
        self._res = [(3664, 2748), (1832, 1374), (912, 686)]
        self._esize = 2
        self.released = False
        self.state = {"brightness": 0, "contrast": 0, "gamma": 100,
                      "exposure_us": 70000, "gain": 100, "auto": True}

    def release(self):
        self.released = True

    # getters
    def get_brightness(self): return self.state["brightness"]
    def get_contrast(self): return self.state["contrast"]
    def get_gamma(self): return self.state["gamma"]
    def get_exposure_time(self): return self.state["exposure_us"]
    def get_exposure_gain(self): return self.state["gain"]
    def get_auto_exposure(self): return self.state["auto"]
    def get_exposure_time_range(self): return (200, 2000000, 70000)
    def get_exposure_gain_range(self): return (100, 800, 100)
    def get_eSize(self): return self._esize
    def get_resolution(self): return self._res[self._esize]
    def get_resolution_list(self): return list(self._res)

    # setters
    def put_brightness(self, v): self.state["brightness"] = int(v); return True
    def put_contrast(self, v): self.state["contrast"] = int(v); return True
    def put_gamma(self, v): self.state["gamma"] = int(v); return True
    def put_exposure_time(self, v): self.state["exposure_us"] = int(v); return True
    def put_exposure_gain(self, v): self.state["gain"] = int(v); return True
    def set_auto_exposure(self, e): self.state["auto"] = bool(e); return True

    def set_resolution_index(self, idx):
        if 0 <= idx < len(self._res):
            self._esize = idx
            return True
        return False

    def get_settings(self):
        return {
            "brightness": self.state["brightness"],
            "contrast": self.state["contrast"],
            "gamma": self.state["gamma"],
            "exposure_us": self.state["exposure_us"],
            "exposure_gain_pct": self.state["gain"],
            "auto_exposure": self.state["auto"],
            "exposure_range_us": (200, 2000000, 70000),
            "gain_range_pct": (100, 800, 100),
            "resolution": self._res[self._esize],
            "eSize": self._esize,
            "resolutions": list(self._res),
            "device_id": self._device_id,
        }


def _toupcam_widget():
    """A real CameraWidget with a fake ToupCam backend wired in + running."""
    mgr = CameraManager(max_cameras=1)
    cam = mgr.cameras[0]
    cam._toupcam = FakeToupCam()
    cam._backend_type = "toupcam"
    cam._running = True
    cam._camera_index = 0
    return mgr, cam


# ── Store ────────────────────────────────────────────────────────────

class TestStoreHwControls(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        self.store = CameraCalibrationStore(self.tmp)

    def test_round_trip_and_none_drop(self):
        self.store.set_hw_controls("dshow:A", {
            "gamma": 120, "brightness": -10, "exposure_us": 50000,
            "auto_exposure": False, "resolution": [1832, 1374],
            "contrast": None,  # dropped
        }, name="BUC3D")
        c = self.store.get_hw_controls("dshow:A")
        self.assertEqual(c["gamma"], 120)
        self.assertEqual(c["exposure_us"], 50000)
        self.assertEqual(c["resolution"], [1832, 1374])
        self.assertNotIn("contrast", c)  # None dropped
        self.assertIsNone(self.store.get_hw_controls("dshow:missing"))

    def test_disk_and_sibling_preserved(self):
        self.store.set_calibration("dshow:A", 5.54, rotation_deg=45.0)
        self.store.set_hw_controls("dshow:A", {"gamma": 80})
        reloaded = CameraCalibrationStore(self.tmp)
        self.assertEqual(reloaded.get_hw_controls("dshow:A")["gamma"], 80)
        cal = reloaded.get_calibration("dshow:A")
        self.assertAlmostEqual(cal["um_per_px"], 5.54, places=4)
        self.assertEqual(cal["rotation_deg"], 45.0)


# ── CameraWidget cross-backend ───────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestWidgetHardware(unittest.TestCase):
    def test_capabilities_when_running(self):
        _, cam = _toupcam_widget()
        caps = cam.hardware_capabilities()
        self.assertEqual(caps["source"], "toupcam")
        self.assertTrue(caps["controllable"])
        self.assertTrue(caps["resolution"])
        self.assertEqual(caps["controls"]["gamma"]["range"], (20, 180, 100))
        self.assertEqual(caps["controls"]["exposure_us"]["range"],
                         (200, 2000000, 70000))

    def test_capabilities_stopped(self):
        _, cam = _toupcam_widget()
        cam._running = False
        caps = cam.hardware_capabilities()
        self.assertFalse(caps["controllable"])
        self.assertEqual(cam.get_hw_settings()["source"], "none")

    def test_readback_source_labelled(self):
        _, cam = _toupcam_widget()
        st = cam.get_hw_settings()
        self.assertEqual(st["source"], "toupcam")
        self.assertEqual(st["gamma"], 100)
        self.assertEqual(st["resolution"], (912, 686))

    def test_setters_delegate(self):
        _, cam = _toupcam_widget()
        self.assertTrue(cam.set_hw_gamma(140))
        self.assertTrue(cam.set_hw_brightness(-20))
        self.assertTrue(cam.set_hw_exposure_us(30000))
        self.assertTrue(cam.set_hw_auto_exposure(False))
        st = cam.get_hw_settings()
        self.assertEqual(st["gamma"], 140)
        self.assertEqual(st["brightness"], -20)
        self.assertEqual(st["exposure_us"], 30000)
        self.assertFalse(st["auto_exposure"])

    def test_resolution_nearest_mapping(self):
        _, cam = _toupcam_widget()
        # exact mid resolution -> eSize 1
        self.assertEqual(cam.set_capture_resolution(1832, 1374), (1832, 1374))
        self.assertEqual(cam.get_hw_settings()["eSize"], 1)
        # tiny target -> nearest is the smallest
        self.assertEqual(cam.set_capture_resolution(10, 10), (912, 686))

    def test_log_readout(self):
        _, cam = _toupcam_widget()
        st, text = cam.log_hw_settings(prefix="X ")
        self.assertEqual(st["source"], "toupcam")
        self.assertIn("source = toupcam", text)
        self.assertIn("gamma", text)


# ── Manager delegation ───────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestManagerHardware(unittest.TestCase):
    def test_delegation(self):
        mgr, _ = _toupcam_widget()
        self.assertTrue(mgr.hardware_capabilities(0)["controllable"])
        self.assertTrue(mgr.set_hw_gamma(0, 77))
        self.assertEqual(mgr.get_hw_settings(0)["gamma"], 77)
        st, text = mgr.log_hw_settings(0)
        self.assertEqual(st["source"], "toupcam")

    def test_invalid_index_safe(self):
        mgr, _ = _toupcam_widget()
        self.assertFalse(mgr.set_hw_gamma(99, 100))
        self.assertEqual(mgr.get_hw_settings(99)["source"], "none")


# ── Settings dialog ──────────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestSettingsDialog(unittest.TestCase):
    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def _dialog(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        mgr, cam = _toupcam_widget()
        mgr.camera_identity = lambda i: ("dshow:BUC3D", "BUC3D")
        dlg = CameraSettingsDialog(
            mgr, 0, identity_getter=lambda: ("dshow:BUC3D", "BUC3D"))
        return dlg, mgr, cam

    def test_reload_populates(self):
        dlg, _, _ = self._dialog()
        self.assertEqual(dlg._gamma_sld.value(), 100)
        self.assertEqual(dlg._bri_sld.value(), 0)
        self.assertTrue(dlg._auto_chk.isChecked())
        # exposure disabled while auto on
        self.assertFalse(dlg._exp_spin.isEnabled())
        # resolution combo populated with device modes
        self.assertEqual(dlg._res_combo.count(), 3)

    def test_live_apply_and_persist(self):
        """v7.17.1: the two halves now have different timing, deliberately.

        The live push to the camera stays IMMEDIATE — aiming a control is a
        visual task, so the preview must follow the slider. The store WRITE is
        debounced, because this slider is wired on ``valueChanged`` and one
        drag used to serialise + atomically replace the calibration file on
        every mouse tick (~50 writes in 9 s, on the GUI thread).
        """
        dlg, mgr, cam = self._dialog()
        dlg._gamma_sld.setValue(150)
        # Immediate: the camera has it already.
        self.assertEqual(cam.get_hw_settings()["gamma"], 150)
        # Deferred: not yet on disk, but pending.
        self.assertTrue(dlg._persist_pending)
        # Flush the way hide/close/quit do — the value must not be lost.
        dlg._flush_persist()
        stored = self.CCS.get_store().get_hw_controls("dshow:BUC3D")
        self.assertEqual(stored["gamma"], 150)

    def test_auto_toggle_enables_exposure(self):
        dlg, mgr, cam = self._dialog()
        dlg._auto_chk.setChecked(False)
        self.assertFalse(cam.get_hw_settings()["auto_exposure"])
        self.assertTrue(dlg._exp_spin.isEnabled())

    def test_resolution_change_applies(self):
        dlg, mgr, cam = self._dialog()
        # pick the 1832x1374 entry (index 1)
        idx = [tuple(dlg._res_combo.itemData(i)) for i in range(dlg._res_combo.count())].index((1832, 1374))
        dlg._res_combo.setCurrentIndex(idx)
        self.assertEqual(cam.get_hw_settings()["eSize"], 1)


# ── Feed-view gear ───────────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestFeedViewGear(unittest.TestCase):
    def test_gear_visible_when_controllable(self):
        """v7.15: the toolbar is hover-revealed, so 'available' and 'on
        screen' are now two things. A controllable camera makes the gear
        available; the pointer being over the feed puts it up."""
        from gui.widgets.camera_feed_view import CameraFeedView
        mgr, _ = _toupcam_widget()
        fv = CameraFeedView(camera_manager=mgr, cam_idx=0)
        fv.show()
        fv._update_settings_visibility()
        self.assertIsNotNone(fv._settings_btn)
        self.assertTrue(fv._settings_available,
                        "a controllable camera did not enable the gear")
        fv._hovering = True
        fv._position_settings_btn()
        # isHidden(), not isVisible(): the latter is False while any ancestor
        # is unshown, which would make this pass regardless.
        self.assertFalse(fv._settings_btn.isHidden())

    def test_gear_hidden_when_not_controllable(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        mgr = CameraManager(max_cameras=1)  # stopped → not controllable
        fv = CameraFeedView(camera_manager=mgr, cam_idx=0)
        fv.show()
        fv._update_settings_visibility()
        self.assertFalse(fv._settings_btn.isVisible())

    def test_gear_absent_when_disabled(self):
        from gui.widgets.camera_feed_view import CameraFeedView
        mgr, _ = _toupcam_widget()
        fv = CameraFeedView(camera_manager=mgr, cam_idx=0, enable_settings=False)
        self.assertIsNone(fv._settings_btn)


# ── Review-driven regressions ────────────────────────────────────────

class _FakeCap:
    """Minimal cv2.VideoCapture stand-in for the OpenCV readback branch."""

    def __init__(self, vals):
        self._vals = vals

    def isOpened(self):
        return True

    def get(self, prop):
        return self._vals.get(prop, -1.0)

    def release(self):
        pass


@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestReviewRegressions(unittest.TestCase):
    def test_stop_releases_toupcam_and_resets_backend(self):
        """stop() must release the ToupCam + reset backend so a reused slot
        can't report stale source='toupcam' (review finding #4)."""
        _, cam = _toupcam_widget()
        fake = cam._toupcam
        cam.stop()
        self.assertTrue(fake.released)
        self.assertIsNone(cam._toupcam)
        self.assertEqual(cam._backend_type, "opencv")
        self.assertEqual(cam.get_hw_settings()["source"], "none")

    def test_opencv_sentinels_become_none_and_auto_tristate(self):
        """OpenCV -1 'unsupported' reads → None; auto-exposure tri-state so a
        manual/unsupported cam isn't shown as auto-ON (review finding #9)."""
        import cv2
        _, cam = _toupcam_widget()
        cam._toupcam = None
        cam._backend_type = "opencv"
        cam._capture = _FakeCap({
            cv2.CAP_PROP_GAMMA: -1.0,           # unsupported
            cv2.CAP_PROP_BRIGHTNESS: 128.0,     # real
            cv2.CAP_PROP_CONTRAST: -1.0,
            cv2.CAP_PROP_EXPOSURE: -1.0,
            cv2.CAP_PROP_GAIN: -1.0,
            cv2.CAP_PROP_AUTO_EXPOSURE: 0.25,   # manual
            cv2.CAP_PROP_FRAME_WIDTH: 640,
            cv2.CAP_PROP_FRAME_HEIGHT: 480,
        })
        st = cam.get_hw_settings()
        self.assertEqual(st["source"], "opencv")
        self.assertIsNone(st["gamma"])
        self.assertEqual(st["brightness"], 128.0)
        self.assertIs(st["auto_exposure"], False)   # 0.25 = manual, not truthy-auto
        self.assertEqual(st["resolution"], (640, 480))

    def test_opencv_auto_unsupported_is_none(self):
        import cv2
        _, cam = _toupcam_widget()
        cam._toupcam = None
        cam._backend_type = "opencv"
        cam._capture = _FakeCap({cv2.CAP_PROP_AUTO_EXPOSURE: -1.0,
                                 cv2.CAP_PROP_FRAME_WIDTH: 640,
                                 cv2.CAP_PROP_FRAME_HEIGHT: 480})
        self.assertIsNone(cam.get_hw_settings()["auto_exposure"])

    def test_dialog_shows_placeholder_for_none_reading(self):
        """A supported control that reads None from the device must show '—'
        and be disabled, not the slider's default (review finding #8)."""
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        import SupportClasses.CameraCalibrationStore as CCS
        CCS._store = CCS.CameraCalibrationStore(
            Path(tempfile.mkdtemp()) / "c.json")
        mgr, cam = _toupcam_widget()
        cam._toupcam.state["gamma"] = None   # device read returns None
        mgr.camera_identity = lambda i: ("dshow:X", "X")
        dlg = CameraSettingsDialog(mgr, 0, identity_getter=lambda: ("dshow:X", "X"))
        self.assertFalse(dlg._gamma_sld.isEnabled())
        self.assertEqual(dlg._gamma_val.text(), "—")
        # a real reading stays enabled
        self.assertTrue(dlg._bri_sld.isEnabled())

    def test_target_overlay_keyword_forwarding(self):
        """TargetOverlayCameraView must not bind parent onto enable_settings
        and must suppress the gear (review low finding)."""
        from gui.widgets.target_overlay_camera_view import TargetOverlayCameraView
        mgr, _ = _toupcam_widget()
        v = TargetOverlayCameraView(mgr, 0, True, "lbl")
        self.assertFalse(v._enable_settings)
        self.assertIsNone(v._settings_btn)


if __name__ == "__main__":
    unittest.main()
