"""
test_v714_post_move_frame.py — a mosaic tile must be EXPOSED after the move.

THE BUG THIS PINS
-----------------
``CameraWidget._frame_seq`` was incremented by the DISPLAY TIMER on every tick,
and the SDK backends' ``read()`` is non-blocking — it hands back the same cached
frame as often as it is asked. So "wait for 3 fresh frames" counted timer ticks,
not sensor frames.

The timer runs at a fixed 15 fps. While the camera outran it, every tick really
was a new frame and the guard worked by luck. At 2048x2048 with a 200 ms
exposure the Zyla delivers ~5 fps: the timer grabs the SAME frame three times in
200 ms, the wait is satisfied, and the mosaic stitches the frame exposed DURING
the stage move. Scanning at full resolution is exactly what triggers it.

Two independent failure modes are covered here:

  1. the counter advancing without a new frame  (silent motion blur)
  2. the flat 2.5 s timeout expiring on a slow camera, after which the old code
     fell through to ``get_current_frame()`` and stitched a STALE tile — the
     same "duplicate image at the wrong canvas spot" corruption the arrival
     check exists to prevent, except with nothing said about it.

Ordering (grab before the next move) is NOT tested here because it is
structural: the scan loop is a single worker thread and the grab call blocks,
so the next move cannot be issued until the frame is in hand. A test would only
restate the control flow.
"""

import ast
import inspect
import os
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from SupportClasses import CaptureTiming as ct


# ── Pure timing math ──────────────────────────────────────────────────

class TestFramePeriod(unittest.TestCase):

    def test_frame_rate_is_preferred_over_exposure(self):
        """The camera's own reported rate accounts for readout and the link
        transfer limit; the exposure is only a lower bound."""
        p = ct.frame_period_s({"frame_rate": 25.0, "exposure_us": 200_000})
        self.assertAlmostEqual(p, 1 / 25.0, places=6)

    def test_exposure_fallback_exceeds_the_exposure_itself(self):
        """A frame can never take LESS than its exposure, so the estimate has
        to sit above it — a period shorter than the exposure would produce a
        timeout that cannot be met."""
        exp_s = 0.200
        p = ct.frame_period_s({"exposure_us": exp_s * 1e6})
        self.assertGreater(p, exp_s)

    def test_absent_or_zero_inputs_give_none_not_a_guess(self):
        for settings in ({}, {"frame_rate": 0}, {"exposure_us": 0},
                         {"frame_rate": None, "exposure_us": None}, None):
            self.assertIsNone(ct.frame_period_s(settings))

    def test_non_numeric_values_do_not_raise(self):
        self.assertIsNone(ct.frame_period_s({"frame_rate": "fast"}))
        self.assertIsNone(ct.frame_period_s({"exposure_us": object()}))

    def test_out_of_band_period_is_rejected(self):
        """A mis-mapped SDK property id reads as a number but does not describe
        frames. Better to fall back than to size a timeout from nonsense."""
        self.assertIsNone(ct.frame_period_s({"frame_rate": 1e9}))     # too fast
        self.assertIsNone(ct.frame_period_s({"exposure_us": 1e9}))    # 1000 s


class TestFreshFrameTimeout(unittest.TestCase):

    def test_never_shorter_than_configured(self):
        """This may only ever LENGTHEN the wait — a rig that works today must
        not start dropping tiles because of this function."""
        for period in (None, 1e-4, 0.001, 0.03, 0.2, 1.0, 10.0):
            self.assertGreaterEqual(
                ct.fresh_frame_timeout_s(3, period, 2.5), 2.5)

    def test_fast_camera_keeps_the_configured_value(self):
        """At preview rates the derived need is tiny, so nothing changes."""
        self.assertEqual(ct.fresh_frame_timeout_s(3, 1 / 30.0, 2.5), 2.5)

    def test_slow_camera_gets_a_longer_wait(self):
        """A 1 s exposure needs >2.5 s for three frames. The OLD flat timeout
        expired here, which is the silent-stale-tile path."""
        t = ct.fresh_frame_timeout_s(3, 1.0, 2.5)
        self.assertGreater(t, 3.0)
        self.assertGreater(t, 2.5)

    def test_scales_with_frame_count(self):
        self.assertGreater(ct.fresh_frame_timeout_s(6, 1.0, 2.5),
                           ct.fresh_frame_timeout_s(3, 1.0, 2.5))

    def test_capped(self):
        self.assertLessEqual(ct.fresh_frame_timeout_s(50, 30.0, 2.5),
                             ct.TIMEOUT_CAP_S)

    def test_bad_configured_value_does_not_raise(self):
        self.assertGreaterEqual(ct.fresh_frame_timeout_s(3, 0.2, None), 0.0)
        self.assertGreaterEqual(ct.fresh_frame_timeout_s(3, 0.2, -5.0), 0.0)


class TestResolveGrabTiming(unittest.TestCase):

    class _Cam:
        def __init__(self, settings=None, boom=False):
            self._s = settings
            self._boom = boom

        def get_hw_settings(self):
            if self._boom:
                raise RuntimeError("no camera")
            return self._s

    def test_enforces_the_two_frame_floor(self):
        """One new frame may have STARTED integrating before the stage
        stopped; only the frame after it is guaranteed clean."""
        n, _t, _p = ct.resolve_grab_timing(self._Cam({"frame_rate": 30}), 1, 2.5)
        self.assertGreaterEqual(n, 2)
        self.assertEqual(ct.MIN_FRESH_FRAMES, 2)

    def test_configured_count_above_the_floor_is_respected(self):
        n, _t, _p = ct.resolve_grab_timing(self._Cam({"frame_rate": 30}), 5, 2.5)
        self.assertEqual(n, 5)

    def test_camera_that_raises_degrades_to_configured_timeout(self):
        n, t, p = ct.resolve_grab_timing(self._Cam(boom=True), 3, 2.5)
        self.assertEqual((n, t, p), (3, 2.5, None))

    def test_camera_without_the_accessor_degrades(self):
        n, t, p = ct.resolve_grab_timing(object(), 3, 2.5)
        self.assertEqual((n, t, p), (3, 2.5, None))

    def test_slow_camera_lengthens_the_timeout_through_the_resolver(self):
        cam = self._Cam({"exposure_us": 1_000_000})
        _n, t, p = ct.resolve_grab_timing(cam, 3, 2.5)
        self.assertIsNotNone(p)
        self.assertGreater(t, 2.5)


# ── The counter: sensor frames, not timer ticks ───────────────────────

class _SlowSdk:
    """SDK backend shaped like the real ones: read() is NON-BLOCKING and
    returns the cached frame, while a separate counter tracks what the sensor
    actually delivered."""

    def __init__(self, h=4, w=6):
        self._frame = np.zeros((h, w, 3), np.uint8)
        self._n = 0
        self.deliver()          # first frame

    def deliver(self):
        """Simulate the reader thread receiving one new sensor frame."""
        self._n += 1
        self._frame = np.full_like(self._frame, self._n % 256)

    def isOpened(self):
        return True

    def read(self):
        return True, self._frame.copy()

    def frames_acquired(self):
        return self._n


class _MuteSdk:
    """A backend that predates the v7.14 accounting: no frames_acquired()."""

    def __init__(self, h=4, w=6):
        self._frame = np.zeros((h, w, 3), np.uint8)

    def isOpened(self):
        return True

    def read(self):
        return True, self._frame.copy()


class TestFrameCounterCountsSensorFrames(unittest.TestCase):
    """Drives the PRODUCTION ``CameraWidget._grab_frame``."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _widget(self, sdk, backend="andor"):
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget(camera_label="test", show_controls=False)
        w._backend_type = backend
        setattr(w, "_" + backend, sdk)
        return w

    def test_repeated_grabs_of_the_same_frame_do_not_advance(self):
        """THE BUG. Ten display-timer ticks against a camera that delivered
        nothing new must leave the counter where it started — otherwise a
        worker is told a post-move frame arrived when none did."""
        sdk = _SlowSdk()
        w = self._widget(sdk)
        w._grab_frame()
        start = w.frame_count_value()
        for _ in range(10):
            w._grab_frame()
        self.assertEqual(w.frame_count_value(), start)

    def test_counter_advances_once_per_delivered_frame(self):
        sdk = _SlowSdk()
        w = self._widget(sdk)
        w._grab_frame()
        start = w.frame_count_value()
        for _ in range(3):
            sdk.deliver()
            w._grab_frame()
            w._grab_frame()      # extra ticks, same frame
        self.assertEqual(w.frame_count_value() - start, 3)

    def test_current_frame_matches_the_counter(self):
        """The pairing invariant: when the counter says a new frame arrived,
        get_current_frame() must actually hold it. This is why the gate lives
        in _grab_frame and not in frame_count_value()."""
        sdk = _SlowSdk()
        w = self._widget(sdk)
        w._grab_frame()
        sdk.deliver()
        w._grab_frame()
        self.assertEqual(int(w.get_current_frame()[0, 0, 0]), sdk._n % 256)

    def test_backend_without_the_accessor_still_counts(self):
        """An older/other backend must not freeze the counter at zero — that
        would hang every post-move wait until it timed out."""
        w = self._widget(_MuteSdk())
        before = w.frame_count_value()
        w._grab_frame()
        w._grab_frame()
        self.assertEqual(w.frame_count_value() - before, 2)

    def test_opencv_path_counts_every_read(self):
        """cv2 read() advances the driver FIFO, so each grab IS a new frame
        and no accounting is needed."""
        class _Cap:
            def isOpened(self):
                return True

            def read(self):
                return True, np.zeros((4, 6, 3), np.uint8)

        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget(camera_label="cv", show_controls=False)
        w._backend_type = "opencv"
        w._capture = _Cap()
        before = w.frame_count_value()
        w._grab_frame()
        w._grab_frame()
        self.assertEqual(w.frame_count_value() - before, 2)


class TestBackendCounters(unittest.TestCase):
    """Every SDK backend exposes the counter, at the ONE place a new frame
    lands. A backend that gained a reader path without it would silently
    reintroduce the blur."""

    def test_all_three_backends_expose_frames_acquired(self):
        from gui.widgets.andor_backend import AndorBackend
        from gui.widgets.toupcam_backend import ToupCamBackend
        from gui.widgets.tucam_backend import TUCamBackend
        for cls in (AndorBackend, ToupCamBackend, TUCamBackend):
            self.assertTrue(callable(getattr(cls, "frames_acquired", None)),
                            f"{cls.__name__} cannot report sensor frames")

    def test_counter_increments_where_the_frame_is_stored(self):
        """AST: the increment must sit in the same block that assigns
        ``self._frame``. Incrementing anywhere else (a read(), a timer) is the
        bug — so this is checked structurally, not by substring."""
        import gui.widgets.andor_backend as ab
        import gui.widgets.toupcam_backend as tb
        import gui.widgets.tucam_backend as ub

        for mod in (ab, tb, ub):
            src = inspect.getsource(mod)
            tree = ast.parse(src)
            found = False
            for node in ast.walk(tree):
                body = getattr(node, "body", None)
                if not isinstance(body, list):
                    continue
                stores = incs = False
                for stmt in body:
                    if (isinstance(stmt, ast.Assign)
                            and any(isinstance(t, ast.Attribute)
                                    and t.attr == "_frame" for t in stmt.targets)):
                        stores = True
                    if (isinstance(stmt, ast.AugAssign)
                            and isinstance(stmt.target, ast.Attribute)
                            and stmt.target.attr == "_frames_acquired"):
                        incs = True
                if stores and incs:
                    found = True
                    break
            self.assertTrue(
                found,
                f"{mod.__name__}: _frames_acquired is not incremented in the "
                f"same block that stores _frame")


# ── Workers drop the tile rather than stitch a stale one ──────────────

class _CountingCam:
    """Camera whose frame counter advances only when told to."""

    def __init__(self, settings=None, advance=0):
        self._n = 0
        self._advance = advance
        self._settings = settings if settings is not None else {}
        self.frame = np.zeros((4, 6, 3), np.uint8)

    def frame_count_value(self):
        n = self._n
        if self._advance:
            self._n += self._advance
        return n

    def get_hw_settings(self):
        return self._settings

    def get_current_frame(self):
        return self.frame


class TestWorkersDropStaleTiles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _plate_worker(self, cam, timeout=0.2):
        from gui.pages.calibration import _MosaicScanWorker
        return _MosaicScanWorker(
            controller=object(), cam=cam, builder=object(), positions=[],
            safe_z=0.0, expected_d_px=10.0, min_dist_px=5.0,
            fresh_frames=3, fresh_timeout_s=timeout, settle_ms=0)

    def test_plate_worker_returns_none_when_no_new_frame(self):
        """Timeout must DROP the tile. Returning the buffered frame stitches
        an image exposed before the move at this tile's canvas position."""
        w = self._plate_worker(_CountingCam(advance=0))
        t0 = time.time()
        self.assertIsNone(w._grab_post_move_frame())
        self.assertLess(time.time() - t0, 5.0)

    def test_plate_worker_returns_the_frame_when_fresh(self):
        w = self._plate_worker(_CountingCam(advance=5))
        self.assertIsNotNone(w._grab_post_move_frame())

    def test_plate_worker_tolerates_a_camera_that_cannot_count(self):
        """No counter => fall back to the settle alone, as before, rather
        than refusing to scan at all."""
        class _NoCount:
            def frame_count_value(self):
                raise AttributeError
            def get_current_frame(self):
                return np.zeros((4, 6, 3), np.uint8)
        self.assertIsNotNone(self._plate_worker(_NoCount())._grab_post_move_frame())

    def test_plate_worker_waits_longer_for_a_slow_camera(self):
        """A 1 s exposure must not be abandoned at the configured 2.5 s."""
        cam = _CountingCam(settings={"exposure_us": 1_000_000}, advance=0)
        w = self._plate_worker(cam, timeout=2.5)
        _n, t, _p = __import__(
            "SupportClasses.CaptureTiming", fromlist=["x"]
        ).resolve_grab_timing(cam, 3, 2.5)
        self.assertGreater(t, 2.5)

    def test_stop_short_circuits_without_waiting(self):
        w = self._plate_worker(_CountingCam(advance=0), timeout=30.0)
        w.stop()
        t0 = time.time()
        self.assertIsNone(w._grab_post_move_frame())
        self.assertLess(time.time() - t0, 1.0)

    def test_fluor_wait_settled_reports_failure(self):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            _SingleWellMosaicWorker as W)
        w = W.__new__(W)
        w._cam = _CountingCam(advance=0)
        w._stop = False
        w._settle_ms = 0
        w._fresh_frames = 3
        w._fresh_timeout_s = 0.2
        self.assertFalse(w._wait_settled())
        self.assertIsNone(w._grab_post_move_frame())

    def test_fluor_wait_settled_reports_success(self):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            _SingleWellMosaicWorker as W)
        w = W.__new__(W)
        w._cam = _CountingCam(advance=5)
        w._stop = False
        w._settle_ms = 0
        w._fresh_frames = 3
        w._fresh_timeout_s = 0.2
        self.assertTrue(w._wait_settled())

    def test_fluor_capture_tile_refuses_before_averaging(self):
        """The refusal has to come BEFORE the averaged read, not after — an
        averaged stack begun mid-move is smeared in every frame."""
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            _SingleWellMosaicWorker as W)
        called = []

        class _Cam(_CountingCam):
            def capture_raw_average(self, n, timeout_s=None):
                called.append(n)
                return np.zeros((4, 6), np.uint16)

        w = W.__new__(W)
        w._cam = _Cam(advance=0)
        w._stop = False
        w._settle_ms = 0
        w._fresh_frames = 3
        w._fresh_timeout_s = 0.2
        w._avg_frames = 4
        self.assertIsNone(w._capture_tile())
        self.assertEqual(called, [], "averaged capture ran despite no "
                                     "confirmed post-move frame")


class TestFeedWatchdogFollowsExposure(unittest.TestCase):
    """A consequence of counting sensor frames: a long exposure advances the
    counter slowly BY DESIGN, so a flat 1.5 s "feed stopped" watchdog would
    accuse a healthy camera and tell the operator to restart it."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _dlg(self):
        from gui.dialogs.camera_rotation_align_dialog import (
            CameraRotationAlignDialog as D)
        return D.__new__(D)

    def test_fast_camera_keeps_the_flat_threshold(self):
        from gui.dialogs.camera_rotation_align_dialog import FEED_WATCHDOG_S
        cam = TestResolveGrabTiming._Cam({"frame_rate": 30.0})
        self.assertEqual(self._dlg()._feed_watchdog_s(cam), FEED_WATCHDOG_S)

    def test_long_exposure_extends_the_threshold(self):
        from gui.dialogs.camera_rotation_align_dialog import FEED_WATCHDOG_S
        cam = TestResolveGrabTiming._Cam({"exposure_us": 2_000_000})
        self.assertGreater(self._dlg()._feed_watchdog_s(cam),
                           FEED_WATCHDOG_S * 2)

    def test_unknown_and_broken_cameras_use_the_flat_threshold(self):
        from gui.dialogs.camera_rotation_align_dialog import FEED_WATCHDOG_S
        d = self._dlg()
        for cam in (TestResolveGrabTiming._Cam({}),
                    TestResolveGrabTiming._Cam(boom=True), None, object()):
            self.assertEqual(d._feed_watchdog_s(cam), FEED_WATCHDOG_S)


if __name__ == "__main__":
    unittest.main()
