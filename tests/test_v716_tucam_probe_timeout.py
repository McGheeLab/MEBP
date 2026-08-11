"""v7.16 — camera detection must not freeze the GUI on a wedged Tucsen.

MEASURED ON ME3B V1 (logs/app.log + logs/freeze.log, 2026-08-11): the camera is
present to Windows (`Status: OK`) but the SDK cannot claim it, and
``TUCAM_Api_Init`` then blocks for **~30 seconds** before answering "0 cameras".
Detection runs on the GUI thread, so the whole application froze — on startup
AND on every press of Detect. The watchdog's dump named the frame:

    tucam_backend.py, line 433 in _api_init
    tucam_backend.py, line 531 in enumerate
    camera_widget.py, line 187 in detect_tucam_cameras
    camera_manager.py, line 224 in detect_cameras
    hardware_setup.py, line 5308 in _on_detect_live_cameras
    main.py, line 273 in run_gui
"""

import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _mgr():
    """A real CameraManager with only the fields the probe touches."""
    from gui.widgets.camera_manager import CameraManager
    m = CameraManager.__new__(CameraManager)
    m._tucam_cache = None
    return m


class TestAWedgedCameraCannotFreezeDetection(unittest.TestCase):

    def test_a_blocking_sdk_is_abandoned_after_the_timeout(self):
        m = _mgr()
        m.TUCAM_PROBE_TIMEOUT_S = 0.3
        release = threading.Event()

        def _wedged_api_init():
            release.wait(30.0)              # what the real SDK does
            return [{"id": "0", "displayname": "Libra 25"}]

        t0 = time.monotonic()
        try:
            got = m._probe_tucam_bounded(_wedged_api_init)
        finally:
            release.set()
        elapsed = time.monotonic() - t0
        self.assertEqual(got, [])
        self.assertLess(elapsed, 3.0,
                        "detection waited on the blocked SDK — this is the "
                        "30-second application freeze")

    def test_a_healthy_sdk_is_used_normally(self):
        m = _mgr()
        cams = [{"id": "0", "displayname": "Libra 25"}]
        self.assertEqual(m._probe_tucam_bounded(lambda: cams), cams)

    def test_a_late_answer_is_kept_for_the_next_round(self):
        """A merely SLOW sdk must cost one detection, not the camera."""
        m = _mgr()
        m.TUCAM_PROBE_TIMEOUT_S = 0.2
        cams = [{"id": "0", "displayname": "Libra 25"}]
        started = threading.Event()

        def _slow():
            started.set()
            time.sleep(0.6)
            return cams

        self.assertEqual(m._probe_tucam_bounded(_slow), [])
        self.assertTrue(started.wait(2.0))
        deadline = time.monotonic() + 3.0
        while m._tucam_cache is None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(m._tucam_cache, cams)
        # Second round: the cache is warm, so the camera is found.
        m.TUCAM_PROBE_TIMEOUT_S = 5.0
        self.assertEqual(m._probe_tucam_bounded(lambda: cams), cams)

    def test_the_cached_result_is_used_while_the_sdk_is_still_wedged(self):
        m = _mgr()
        m.TUCAM_PROBE_TIMEOUT_S = 0.2
        cams = [{"id": "0", "displayname": "Libra 25"}]
        m._tucam_cache = cams
        release = threading.Event()
        try:
            got = m._probe_tucam_bounded(lambda: release.wait(30.0))
        finally:
            release.set()
        self.assertEqual(got, cams)

    def test_a_raising_probe_is_survivable(self):
        m = _mgr()

        def _boom():
            raise RuntimeError("SDK not installed")

        self.assertEqual(m._probe_tucam_bounded(_boom), [])

    def test_the_probe_thread_is_a_daemon(self):
        """It may still be inside a 30-second C call at shutdown; a non-daemon
        thread would hold the process open for that long."""
        m = _mgr()
        m.TUCAM_PROBE_TIMEOUT_S = 0.2
        seen = {}
        release = threading.Event()

        def _wedged():
            seen["daemon"] = threading.current_thread().daemon
            release.wait(10.0)
            return []

        try:
            m._probe_tucam_bounded(_wedged)
        finally:
            release.set()
        self.assertTrue(seen.get("daemon"))

    def test_the_timeout_is_far_below_the_measured_block(self):
        from gui.widgets.camera_manager import CameraManager
        self.assertLessEqual(CameraManager.TUCAM_PROBE_TIMEOUT_S, 10.0)
        self.assertGreaterEqual(CameraManager.TUCAM_PROBE_TIMEOUT_S, 1.0)


class TestDetectionStillReportsTheOtherBackends(unittest.TestCase):

    def test_detect_cameras_does_not_call_tucam_on_the_calling_thread(self):
        """The point of the fix: the enumeration must happen on a worker."""
        import inspect
        from gui.widgets.camera_manager import CameraManager
        src = inspect.getsource(CameraManager.detect_cameras)
        self.assertIn("_probe_tucam_bounded", src)
        self.assertNotIn("tucam = detect_tucam_cameras()", src)

    def test_the_worker_thread_is_where_the_probe_runs(self):
        m = _mgr()
        where = {}
        m._probe_tucam_bounded(
            lambda: where.setdefault("thread", threading.current_thread().name)
            or [])
        self.assertEqual(where["thread"], "TucamProbe")


if __name__ == "__main__":
    unittest.main()
