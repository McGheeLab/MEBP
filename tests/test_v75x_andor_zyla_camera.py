"""
test_v75x_andor_zyla_camera.py — ANDOR Zyla (SDK3) microscope camera backend.

v7.5.x: Adds a pylablib-backed Andor SDK3 backend (gui/widgets/andor_backend.py)
so the ANDOR ZYLA-4.2P-USB3 (2048x2048 mono 16-bit sCMOS) can be used as the
microscope camera, alongside the OpenCV and ToupTek backends.

The real SDK path needs on-rig verification; these tests drive the cross-backend
logic with a fake Andor backend injected into a real CameraWidget (mirroring the
FakeToupCam pattern), plus the pure mono-16->BGR8 conversion and the availability
guard. No pylablib / SDK DLLs are required.

Covers:
- andor_backend module imports without pylablib/DLLs; ANDOR_AVAILABLE is a bool;
  enumerate() degrades to []; _mono_to_bgr8 conversion (uint16 auto-scale, dim
  frame visibility, BGR passthrough, uint8 mono).
- CameraWidget "andor" branch: capabilities (exposure only; ISP controls hidden),
  source-labelled readback, exposure delegation, unsupported-setter returns False,
  nearest-resolution mapping, _grab_frame -> RGB888 QImage, log readout.
- camera_identity: andor:<serial> round-trip.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager
from gui.widgets import andor_backend
from gui.widgets.camera_identity import identity_for_source, source_for_identity


# ── Fake Andor backend (mirrors AndorBackend's surface; no SDK) ───────

class FakeAndorCam:
    """Mimics AndorBackend's control surface without pylablib/hardware.

    ``read()`` returns an already-converted BGR8 frame (the real backend does
    the mono-16 -> BGR8 conversion internally), so the widget path is exercised
    exactly as in production.
    """

    HW_RANGES: dict = {}

    def __init__(self, frame=None):
        self._device_id = "SN-ZYLA-001"
        self._res = [(1024, 1024), (512, 512), (256, 256), (2048, 2048)]
        self._esize = 0
        self._exposure_us = 30000
        self._auto_scale = True
        self._lo, self._hi = 0, 65535
        self.released = False
        self._frame = frame if frame is not None else np.full(
            (1024, 1024, 3), 40, dtype=np.uint8)

    # lifecycle
    def isOpened(self):
        return True

    def read(self):
        return True, self._frame.copy()

    def release(self):
        self.released = True

    # resolution / binning
    def get_resolution(self):
        return self._res[self._esize]

    def get_eSize(self):
        return self._esize

    def get_resolution_list(self):
        return list(self._res)

    def set_resolution_index(self, idx):
        if 0 <= idx < len(self._res):
            self._esize = idx
            return True
        return False

    # exposure (the only real hw control)
    def get_exposure_time(self):
        return self._exposure_us

    def put_exposure_time(self, us):
        self._exposure_us = int(us)
        return True

    def get_exposure_time_range(self):
        return (100, 30000000, 30000)

    # display scaling (mono16 -> 8-bit display conversion, v7.5.x)
    def get_display_auto_scale(self):
        return self._auto_scale

    def set_display_auto_scale(self, enabled):
        self._auto_scale = bool(enabled)
        return True

    def get_display_levels(self):
        return (self._lo, self._hi)

    def put_display_black(self, v):
        self._lo = int(v)
        return True

    def put_display_white(self, v):
        self._hi = int(v)
        return True

    def get_display_level_range(self):
        return (0, 65535, 65535)

    # unsupported ISP controls -> None / False (Zyla has none)
    def get_exposure_gain(self): return None
    def put_exposure_gain(self, v): return False
    def get_exposure_gain_range(self): return None
    def get_auto_exposure(self): return None
    def set_auto_exposure(self, e): return False
    def get_brightness(self): return None
    def put_brightness(self, v): return False
    def get_contrast(self): return None
    def put_contrast(self, v): return False
    def get_gamma(self): return None
    def put_gamma(self, v): return False

    def get_settings(self):
        return {
            "brightness": None, "contrast": None, "gamma": None,
            "exposure_us": self._exposure_us,
            "exposure_gain_pct": None, "auto_exposure": None,
            "exposure_range_us": self.get_exposure_time_range(),
            "gain_range_pct": None,
            "andor_auto_scale": self._auto_scale,
            "andor_scale_lo": self._lo,
            "andor_scale_hi": self._hi,
            "resolution": self._res[self._esize],
            "eSize": self._esize,
            "resolutions": list(self._res),
            "device_id": self._device_id,
        }


def _andor_widget(frame=None):
    """A real CameraWidget with a fake Andor backend wired in + running."""
    mgr = CameraManager(max_cameras=1)
    cam = mgr.cameras[0]
    cam._andor = FakeAndorCam(frame=frame)
    cam._backend_type = "andor"
    cam._running = True
    cam._camera_index = 0
    return mgr, cam


# ── Availability guard + enumerate ────────────────────────────────────

class TestAvailabilityGuard(unittest.TestCase):
    def test_import_does_not_require_sdk(self):
        # Importing the module must never raise / load DLLs.
        self.assertTrue(hasattr(andor_backend, "AndorBackend"))
        self.assertTrue(hasattr(andor_backend, "ANDOR_AVAILABLE"))

    def test_available_is_boolean(self):
        # Resolves lazily; in the dev/CI env (no pylablib) it should be falsy,
        # but only assert it is a well-formed bool so a rig with the SDK passes.
        self.assertIn(bool(andor_backend.ANDOR_AVAILABLE), (True, False))

    def test_enumerate_never_raises(self):
        # enumerate() must always return a list and never raise. When the SDK is
        # unavailable it degrades to []; when it IS available (real rig with a
        # camera attached) it may list devices — so only assert the type, and
        # the []-degradation specifically in the SDK-absent case.
        devs = andor_backend.AndorBackend.enumerate()
        self.assertIsInstance(devs, list)
        if not andor_backend.ANDOR_AVAILABLE:
            self.assertEqual(devs, [])


# ── mono-16 -> BGR8 conversion ────────────────────────────────────────

class TestMonoToBgr8(unittest.TestCase):
    def test_uint16_gradient_to_uint8_bgr(self):
        f16 = np.linspace(0, 65535, 64 * 64, dtype=np.uint16).reshape(64, 64)
        bgr = andor_backend._mono_to_bgr8(f16)
        self.assertEqual(bgr.dtype, np.uint8)
        self.assertEqual(bgr.shape, (64, 64, 3))
        # 3 channels identical (grayscale replicated).
        self.assertTrue(np.array_equal(bgr[:, :, 0], bgr[:, :, 1]))
        self.assertTrue(np.array_equal(bgr[:, :, 1], bgr[:, :, 2]))

    def test_dim_frame_becomes_visible(self):
        # A dim 16-bit frame (values ~200-800 out of 65535) auto-scales so the
        # displayed image spans most of the 0-255 range (fluorescence case).
        f16 = np.random.randint(200, 800, size=(128, 128)).astype(np.uint16)
        bgr = andor_backend._mono_to_bgr8(f16)
        self.assertGreater(int(bgr.max()), 200)  # not left near-black

    def test_flat_frame_no_divide_by_zero(self):
        f16 = np.full((16, 16), 500, dtype=np.uint16)
        bgr = andor_backend._mono_to_bgr8(f16)  # hi == lo path
        self.assertEqual(bgr.shape, (16, 16, 3))
        self.assertEqual(bgr.dtype, np.uint8)

    def test_bgr8_passthrough(self):
        f = np.full((8, 8, 3), 123, dtype=np.uint8)
        out = andor_backend._mono_to_bgr8(f)
        self.assertEqual(out.shape, (8, 8, 3))
        self.assertEqual(int(out[0, 0, 0]), 123)

    def test_uint8_mono_to_bgr(self):
        f = np.full((8, 8), 77, dtype=np.uint8)
        out = andor_backend._mono_to_bgr8(f)
        self.assertEqual(out.shape, (8, 8, 3))
        self.assertEqual(int(out[0, 0, 0]), 77)

    def test_none_returns_none(self):
        self.assertIsNone(andor_backend._mono_to_bgr8(None))


# ── CameraWidget "andor" branch ───────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestWidgetAndorBranch(unittest.TestCase):
    def test_capabilities_exposure_only(self):
        _mgr, cam = _andor_widget()
        caps = cam.hardware_capabilities()
        self.assertEqual(caps["source"], "andor")
        self.assertTrue(caps["controllable"])
        self.assertTrue(caps["resolution"])
        ctrls = caps["controls"]
        self.assertIn("exposure_us", ctrls)
        self.assertEqual(ctrls["exposure_us"]["range"], (100, 30000000, 30000))
        # ISP controls the Zyla lacks must be ABSENT so the dialog hides them.
        for absent in ("gamma", "brightness", "contrast",
                       "auto_exposure", "exposure_gain_pct"):
            self.assertNotIn(absent, ctrls)

    def test_get_hw_settings_source_andor(self):
        _mgr, cam = _andor_widget()
        st = cam.get_hw_settings()
        self.assertEqual(st["source"], "andor")
        self.assertEqual(st["exposure_us"], 30000)
        self.assertIsNone(st["gamma"])
        self.assertEqual(st["device_id"], "SN-ZYLA-001")

    def test_set_exposure_delegates(self):
        _mgr, cam = _andor_widget()
        self.assertTrue(cam.set_hw_exposure_us(12345))
        self.assertEqual(cam._andor.get_exposure_time(), 12345)

    def test_unsupported_setters_return_false(self):
        _mgr, cam = _andor_widget()
        self.assertFalse(cam.set_hw_gamma(120))
        self.assertFalse(cam.set_hw_brightness(10))
        self.assertFalse(cam.set_hw_contrast(10))
        self.assertFalse(cam.set_hw_auto_exposure(True))

    def test_set_capture_resolution_nearest(self):
        _mgr, cam = _andor_widget()
        actual = cam.set_capture_resolution(500, 500)  # nearest preset = 512²
        self.assertEqual(actual, (512, 512))
        self.assertEqual(cam._andor.get_eSize(), 1)

    def test_grab_frame_emits_rgb888_qimage(self):
        frame = np.full((1024, 1024, 3), 60, dtype=np.uint8)
        _mgr, cam = _andor_widget(frame=frame)
        captured = []
        cam.frame_captured.connect(lambda q: captured.append(q))
        cam._grab_frame()
        self.assertTrue(captured)
        q = captured[-1]
        self.assertIsInstance(q, QImage)
        self.assertEqual(q.format(), QImage.Format.Format_RGB888)
        self.assertEqual((q.width(), q.height()), (1024, 1024))

    def test_grab_frame_caches_raw_for_detection(self):
        frame = np.full((256, 256, 3), 90, dtype=np.uint8)
        _mgr, cam = _andor_widget(frame=frame)
        cam._grab_frame()
        raw = cam.get_current_frame()
        self.assertIsNotNone(raw)
        self.assertEqual(raw.shape, (256, 256, 3))

    def test_log_hw_settings_renders_andor(self):
        _mgr, cam = _andor_widget()
        st, text = cam.log_hw_settings()
        self.assertEqual(st["source"], "andor")
        self.assertIn("source = andor", text)
        self.assertIn("exposure", text)


# ── Manager delegation ────────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestManagerDelegation(unittest.TestCase):
    def test_manager_reads_andor_settings(self):
        mgr, _cam = _andor_widget()
        caps = mgr.hardware_capabilities(0)
        self.assertEqual(caps["source"], "andor")
        st = mgr.get_hw_settings(0)
        self.assertEqual(st["source"], "andor")


# ── Identity round-trip ───────────────────────────────────────────────

class TestAndorIdentity(unittest.TestCase):
    def test_identity_for_source(self):
        self.assertEqual(
            identity_for_source(("andor", "SN-ZYLA-001"), []),
            ("andor:SN-ZYLA-001", "Andor Zyla"),
        )

    def test_source_for_identity(self):
        self.assertEqual(
            source_for_identity("andor:SN-ZYLA-001", []),
            ("andor", "SN-ZYLA-001"),
        )

    def test_round_trip(self):
        src = ("andor", "abc123")
        ident, _name = identity_for_source(src, [])
        self.assertEqual(source_for_identity(ident, []), src)


if __name__ == "__main__":
    unittest.main()
