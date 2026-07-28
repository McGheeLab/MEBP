"""
test_v75x_camera_async_open_and_context_width.py

Two startup-usability fixes:

  (1) Camera boot-up freezing the UI — the blocking device open (cv2.VideoCapture
      / ToupCam / Andor, several seconds each) used to run on the GUI thread when
      the saved cameras auto-started at launch. ``CameraWidget.start_async`` now
      opens on a daemon thread and finalizes (assigns handle + starts the display
      timer) back on the GUI thread via the queued ``_open_result`` signal;
      ``CameraManager.start_async`` drives it from the startup path and emits
      ``camera_started`` only once the camera is actually running. ``start`` stays
      synchronous for callers that need a frame immediately (mosaic scan).

  (2) Left context panel couldn't be widened enough to show its buttons on a
      modest window. ``MainWindow._update_context_panel_bounds`` must never cap
      the panel's maximum width below its own minimum (== the designed panel
      width), so all controls are always reachable.

The async open is exercised with a fake ToupCam-style backend injected into a
real CameraWidget (no DLL/hardware). The bounds math is exercised by calling the
MainWindow method unbound against a fake self holding a real QSplitter (a full
MainWindow boot is intractable headless).
"""

import os
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QSplitter, QFrame
from PySide6.QtCore import Qt

_app = QApplication.instance() or QApplication(sys.argv)

import gui.widgets.camera_widget as cw
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager
from gui.scaling import s
from gui.app import MainWindow
from gui.ui_functions import AppSettings


# ── Fake backends (mimic ToupCamBackend.open/read/release surface) ───────────

class _FakeBackend:
    """Opens successfully; delivers no frames (read → (False, None))."""
    instances = []

    def __init__(self):
        _FakeBackend.instances.append(self)
        self._device_id = "fake"
        self.released = False
        self._open_delay = 0.0

    def open(self, device_id, resolution_index=None):
        if self._open_delay:
            time.sleep(self._open_delay)
        return True

    def get_resolution(self):
        return (640, 480)

    def isOpened(self):
        return not self.released

    def read(self):
        return False, None

    def release(self):
        self.released = True


class _FakeBackendSlow(_FakeBackend):
    def __init__(self):
        super().__init__()
        self._open_delay = 0.25


class _FakeBackendFail(_FakeBackend):
    def open(self, device_id, resolution_index=None):
        return False


def _select_toupcam(cam):
    """Point a (headless) CameraWidget's combo at a fake ToupCam source."""
    cam.camera_combo.clear()
    cam.camera_combo.addItem("TC: fake", ("toupcam", "fake-id"))
    cam.camera_combo.setCurrentIndex(0)
    cam._cameras_detected = True  # skip start_async's lazy re-probe


def _pump_until(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while not predicate() and time.time() < deadline:
        _app.processEvents()
        time.sleep(0.01)
    _app.processEvents()
    return predicate()


@unittest.skipUnless(CAMERA_AVAILABLE, "camera support not available")
class TestAsyncCameraOpen(unittest.TestCase):
    def setUp(self):
        _FakeBackend.instances.clear()
        self._orig_avail = cw.TOUPCAM_AVAILABLE
        self._orig_backend = cw.ToupCamBackend
        cw.TOUPCAM_AVAILABLE = True

    def tearDown(self):
        cw.TOUPCAM_AVAILABLE = self._orig_avail
        cw.ToupCamBackend = self._orig_backend

    def _widget(self):
        mgr = CameraManager(max_cameras=1)
        self.assertTrue(mgr.cameras, "manager built no camera widgets")
        return mgr, mgr.cameras[0]

    def test_start_async_opens_off_thread_and_finalizes(self):
        cw.ToupCamBackend = _FakeBackend
        mgr, cam = self._widget()
        _select_toupcam(cam)

        done = []
        cam.start_async(on_done=lambda ok: done.append(ok))
        # Returns immediately (open is on a worker thread) — not yet running.
        self.assertFalse(cam._running)
        self.assertTrue(cam._opening)

        self.assertTrue(_pump_until(lambda: bool(done)),
                        "async open never finalized")
        self.assertEqual(done, [True])
        self.assertTrue(cam._running)
        self.assertFalse(cam._opening)
        self.assertEqual(cam._backend_type, "toupcam")
        self.assertIsNotNone(cam._toupcam)
        cam.stop()

    def test_start_async_failed_open_reports_false(self):
        cw.ToupCamBackend = _FakeBackendFail
        mgr, cam = self._widget()
        _select_toupcam(cam)

        done = []
        cam.start_async(on_done=lambda ok: done.append(ok))
        self.assertTrue(_pump_until(lambda: bool(done)))
        self.assertEqual(done, [False])
        self.assertFalse(cam._running)
        self.assertIsNone(cam._toupcam)

    def test_stop_during_open_supersedes_and_releases(self):
        cw.ToupCamBackend = _FakeBackendSlow
        mgr, cam = self._widget()
        _select_toupcam(cam)

        done = []
        cam.start_async(on_done=lambda ok: done.append(ok))
        self.assertTrue(cam._opening)
        # Stop BEFORE the (slow) worker open finishes → supersede it.
        cam.stop()
        self.assertFalse(cam._opening)

        # Let the worker finish + its queued result be delivered.
        self.assertTrue(_pump_until(lambda: bool(done), timeout=3.0))
        # The superseded open must NOT have been adopted…
        self.assertFalse(cam._running)
        self.assertIsNone(cam._toupcam)
        # …and its handle must have been released, not leaked/left streaming.
        self.assertTrue(_FakeBackend.instances)
        self.assertTrue(_FakeBackend.instances[-1].released)

    def test_manager_start_async_emits_started_when_running(self):
        cw.ToupCamBackend = _FakeBackend
        mgr, cam = self._widget()
        _select_toupcam(cam)

        events = []
        mgr.camera_started.connect(lambda i: events.append(i))
        mgr.start_async(0)
        self.assertFalse(mgr.is_running(0))  # not yet — opening off-thread

        self.assertTrue(_pump_until(lambda: bool(events)))
        self.assertEqual(events, [0])
        self.assertTrue(mgr.is_running(0))
        mgr.stop(0)

    def test_manager_start_stays_synchronous_for_simulated(self):
        # Sanity: the blocking start() path (used by mosaic scan etc.) is intact
        # and the simulated backend still opens synchronously.
        if not getattr(cw, "SIM_AVAILABLE", False):
            self.skipTest("simulated camera backend not available")
        mgr, cam = self._widget()
        cam.camera_combo.clear()
        cam.camera_combo.addItem("SIM", ("simulated", "microscope"))
        cam.camera_combo.setCurrentIndex(0)
        cam._cameras_detected = True

        events = []
        mgr.camera_started.connect(lambda i: events.append(i))
        mgr.start(0)                       # synchronous
        self.assertTrue(mgr.is_running(0))  # running immediately on return
        self.assertEqual(events, [0])
        mgr.stop(0)


class TestContextPanelBounds(unittest.TestCase):
    """MainWindow._update_context_panel_bounds must never cap the panel below its
    minimum width (== the designed width), so all its buttons stay reachable."""

    def _bounds(self, total_px, min_px, reserve_px):
        sp = QSplitter(Qt.Horizontal)
        left, mid, right = QFrame(), QFrame(), QFrame()
        left.setMinimumWidth(min_px)
        for w in (left, mid, right):
            sp.addWidget(w)
        sp.resize(total_px, 400)

        fake = type("W", (), {})()
        fake._context_splitter = sp
        fake.ui_extraLeftBox = left
        fake.ui_extraRightBox = right     # not shown → isVisible() False
        fake._content_reserve_px = reserve_px

        MainWindow._update_context_panel_bounds(fake)
        return left.maximumWidth()

    def test_max_never_below_min_on_narrow_window(self):
        min_px = s(AppSettings.LEFT_BOX_WIDTH)   # designed width
        reserve = s(240)
        # A window so narrow that (total - reserve) < min: the panel must still
        # be allowed up to its full designed width (all buttons visible), never
        # clamped narrower.
        narrow_total = min_px + reserve - s(100)
        self.assertEqual(self._bounds(narrow_total, min_px, reserve), min_px)

    def test_max_grows_with_window(self):
        min_px = s(AppSettings.LEFT_BOX_WIDTH)
        reserve = s(240)
        wide_total = min_px + reserve + s(400)
        got = self._bounds(wide_total, min_px, reserve)
        self.assertEqual(got, wide_total - reserve)
        self.assertGreater(got, min_px)   # draggable beyond the designed width

    def test_designed_width_reachable_on_modest_window(self):
        # Regression: the OLD reserve (460) + old min (340) left the panel
        # clamped below its designed width on modest windows. With the new
        # min == designed width, the designed width is always reachable.
        min_px = s(AppSettings.LEFT_BOX_WIDTH)
        reserve = s(240)
        modest_total = min_px + s(300)   # a not-especially-wide window
        self.assertGreaterEqual(self._bounds(modest_total, min_px, reserve),
                                min_px)


if __name__ == "__main__":
    unittest.main()
