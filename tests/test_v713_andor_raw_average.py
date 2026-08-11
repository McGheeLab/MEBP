"""
test_v713_andor_raw_average.py — averaged raw capture for mosaic tiles (v7.13).

`capture_raw_average(n)` returns the per-pixel uint16 mean of n consecutive
NEW raw frames, serviced by the backend's own reader thread (a second
wait_for_frame waiter would race it). This is what gives the fluorescence
mosaic its √N SNR boost with quantitatively consistent tiles (fixed display
levels applied AFTER averaging).

Covers: exact means (mutation guard: a uint16 accumulator overflows near
60000), rint rounding, √N noise reduction, timeout / concurrent-request /
release / resolution-change semantics (nothing may hang), the fixed-level
tile-consistency contract, and the widget/manager passthrough.
"""

import os
import sys
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.andor_backend import AndorBackend, _RawAverageRequest
from gui.widgets.mono_display import mono_to_bgr8
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager

from tests.test_v75x_andor_zyla_camera import _andor_widget
from tests.test_v713_andor_sensor_features import _StubCam, _full_features


def _live_backend(stub):
    """Real AndorBackend wired to the stub with the reader loop RUNNING."""
    be = AndorBackend()
    be._cam = stub
    be._probe_sensor_features()
    be._running = True
    t = threading.Thread(target=be._reader_loop, daemon=True)
    t.start()
    return be, t


def _stop(be, t):
    be._running = False
    t.join(timeout=2.0)


def _wait_for_request(be, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with be._lock:
            if be._avg_request is not None:
                return True
        time.sleep(0.005)
    return False


class TestRawAverageRequestUnit(unittest.TestCase):
    def test_exact_mean_bright_frames(self):
        # Values near 60000: a uint16 accumulator overflows on the SECOND
        # frame (59000+60000 > 65535) — float64 must be exact.
        req = _RawAverageRequest(4)
        for v in (59000, 60000, 61000, 62000):
            req.add(np.full((8, 8), v, dtype=np.uint16))
        self.assertTrue(req.done.is_set())
        out = req.result()
        self.assertEqual(out.dtype, np.uint16)
        self.assertTrue(np.all(out == 60500))

    def test_rint_rounding(self):
        req = _RawAverageRequest(2)
        req.add(np.full((4, 4), 1, dtype=np.uint16))
        req.add(np.full((4, 4), 2, dtype=np.uint16))
        self.assertEqual(int(req.result()[0, 0]), 2)   # rint(1.5) = 2

    def test_shape_change_fails(self):
        req = _RawAverageRequest(3)
        req.add(np.zeros((8, 8), dtype=np.uint16))
        req.add(np.zeros((4, 4), dtype=np.uint16))
        self.assertTrue(req.done.is_set())
        self.assertIsNotNone(req.error)
        self.assertIsNone(req.result())

    def test_incomplete_returns_none(self):
        req = _RawAverageRequest(3)
        req.add(np.zeros((4, 4), dtype=np.uint16))
        req.fail("timeout")
        self.assertIsNone(req.result())

    def test_add_after_done_ignored(self):
        req = _RawAverageRequest(1)
        req.add(np.full((4, 4), 10, dtype=np.uint16))
        req.add(np.full((4, 4), 999, dtype=np.uint16))   # ignored
        self.assertTrue(np.all(req.result() == 10))

    def test_sqrt_n_noise_reduction(self):
        rng = np.random.default_rng(7)
        req = _RawAverageRequest(16)
        base = np.full((64, 64), 2000.0)
        singles = []
        for _ in range(16):
            f = (base + rng.normal(0, 100, size=base.shape)).clip(0, 65535)
            f = f.astype(np.uint16)
            singles.append(f)
            req.add(f)
        avg = req.result().astype(np.float64)
        std_single = float(np.std(singles[0].astype(np.float64)))
        std_avg = float(np.std(avg))
        self.assertLess(std_avg, std_single / 3.0)   # √16 = 4× (allow margin)


class TestCaptureRawAverageLoop(unittest.TestCase):
    def test_exact_mean_through_reader_loop(self):
        stub = _StubCam(features=_full_features())
        be, t = _live_backend(stub)
        try:
            result = {}

            def _call():
                result["out"] = be.capture_raw_average(4, timeout_s=5.0)

            worker = threading.Thread(target=_call, daemon=True)
            worker.start()
            self.assertTrue(_wait_for_request(be))
            for v in (59000, 60000, 61000, 62000):
                stub.stage_frame(np.full((16, 16), v, dtype=np.uint16))
            worker.join(timeout=5.0)
            self.assertFalse(worker.is_alive())
            out = result["out"]
            self.assertIsNotNone(out)
            self.assertEqual(out.dtype, np.uint16)
            self.assertTrue(np.all(out == 60500))
            # The request slot is cleared for the next caller.
            with be._lock:
                self.assertIsNone(be._avg_request)
        finally:
            _stop(be, t)

    def test_timeout_returns_none_and_clears(self):
        stub = _StubCam(features=_full_features())
        be, t = _live_backend(stub)
        try:
            t0 = time.monotonic()
            out = be.capture_raw_average(4, timeout_s=0.3)
            self.assertIsNone(out)
            self.assertLess(time.monotonic() - t0, 3.0)   # bounded, no hang
            with be._lock:
                self.assertIsNone(be._avg_request)
        finally:
            _stop(be, t)

    def test_concurrent_request_refused(self):
        stub = _StubCam(features=_full_features())
        be, t = _live_backend(stub)
        try:
            def _slow_call():
                be.capture_raw_average(1000, timeout_s=5.0)

            worker = threading.Thread(target=_slow_call, daemon=True)
            worker.start()
            self.assertTrue(_wait_for_request(be))
            self.assertIsNone(be.capture_raw_average(2, timeout_s=0.2))
            be._fail_pending_average("test done")
            worker.join(timeout=3.0)
            self.assertFalse(worker.is_alive())
        finally:
            _stop(be, t)

    def test_release_fails_pending_request(self):
        stub = _StubCam(features=_full_features())
        be, t = _live_backend(stub)
        result = {}

        def _call():
            result["out"] = be.capture_raw_average(1000, timeout_s=10.0)

        worker = threading.Thread(target=_call, daemon=True)
        worker.start()
        self.assertTrue(_wait_for_request(be))
        be.release()                       # must unblock the caller promptly
        worker.join(timeout=3.0)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(result["out"])
        t.join(timeout=2.0)

    def test_bad_inputs(self):
        stub = _StubCam(features=_full_features())
        be, t = _live_backend(stub)
        try:
            self.assertIsNone(be.capture_raw_average(0, timeout_s=0.2))
            self.assertIsNone(be.capture_raw_average("x", timeout_s=0.2))
        finally:
            _stop(be, t)

    def test_closed_backend_returns_none(self):
        be = AndorBackend()
        self.assertIsNone(be.capture_raw_average(4, timeout_s=0.2))


class TestFixedLevelTileContract(unittest.TestCase):
    def test_same_count_same_gray_across_tiles(self):
        # The mosaic freezes ONE (lo, hi) per channel; the same raw count must
        # map to the same display gray in every tile regardless of tile
        # content — that is what makes stitched tiles quantitatively
        # consistent.
        lo, hi = 200.0, 8200.0
        tile_a = np.full((32, 32), 5000, dtype=np.uint16)
        tile_b = np.random.randint(0, 3000, size=(32, 32)).astype(np.uint16)
        tile_b[4, 4] = 5000
        a = mono_to_bgr8(tile_a, levels=(lo, hi))
        b = mono_to_bgr8(tile_b, levels=(lo, hi))
        self.assertEqual(int(a[0, 0, 0]), int(b[4, 4, 0]))


@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestPassthrough(unittest.TestCase):
    def test_widget_and_manager_delegate(self):
        mgr, cam = _andor_widget()
        out = mgr.capture_raw_average(0, 6, timeout_s=2.5)
        self.assertIsNotNone(out)
        self.assertEqual(out.dtype, np.uint16)
        self.assertEqual(cam._andor.raw_average_calls, [(6, 2.5)])

    def test_none_for_slot_without_backend(self):
        mgr = CameraManager(max_cameras=1)
        self.assertIsNone(mgr.capture_raw_average(0, 4))


if __name__ == "__main__":
    unittest.main()
