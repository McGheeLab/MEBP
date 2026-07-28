"""test_v75x_timing_encoder_detector.py — camera-free timing calibration.

The XY↔ZP timing calibrator measured stage motion with the microscope
(frame-difference). On rigs where the camera is a poor motion sensor, that path
is unusable. This adds a CAMERA-FREE detector that senses motion from the stage's
OWN reported position (_EncoderMotion), a drop-in for the camera tracker
(available/reset/metric), selected by a Detector combo (default = stage position).
All existing measurements (top speed, settle sweep) then run with no camera.
"""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from gui.pages.workflows.timing_calibration_workflow import (  # noqa: E402
    _EncoderMotion, _FrameMotion, TimingCalibrationWorkflowPage,
)


class _Ctrl:
    def __init__(self, xy_connected=True):
        self.is_xy_connected = xy_connected
        self._pos = [1000.0, 2000.0]     # µm
        self.reads = 0

    def get_xy_position(self, cached=False):
        self.reads += 1
        return (self._pos[0], self._pos[1], 0.0)

    def move_to(self, x, y):
        self._pos = [x, y]


class TestEncoderMotion(unittest.TestCase):
    def test_first_metric_is_none_then_delta(self):
        c = _Ctrl()
        m = _EncoderMotion(c)
        self.assertIsNone(m.metric())          # no previous sample yet
        c.move_to(1000.0, 2000.0)
        self.assertEqual(m.metric(), 0.0)      # at rest → 0 µm change

    def test_metric_reports_distance_moved(self):
        c = _Ctrl()
        m = _EncoderMotion(c)
        m.metric()                              # seed
        c.move_to(1300.0, 2400.0)               # +300, +400 → 500 µm
        self.assertAlmostEqual(m.metric(), 500.0, places=3)

    def test_reset_clears_previous(self):
        c = _Ctrl()
        m = _EncoderMotion(c)
        m.metric(); c.move_to(1100.0, 2000.0); m.metric()
        m.reset()
        self.assertIsNone(m.metric())           # first sample after reset

    def test_available_requires_connected_stage(self):
        self.assertTrue(_EncoderMotion(_Ctrl(True)).available())
        self.assertFalse(_EncoderMotion(_Ctrl(False)).available())
        self.assertFalse(_EncoderMotion(None).available())

    def test_metric_none_on_unreadable_position(self):
        class _Bad:
            is_xy_connected = True

            def get_xy_position(self, cached=False):
                return (None, None, None)
        m = _EncoderMotion(_Bad())
        self.assertIsNone(m.metric())


class _Combo:
    def __init__(self, data):
        self._data = data

    def currentData(self):
        return self._data


class TestDetectorSelection(unittest.TestCase):
    def _page(self):
        return TimingCalibrationWorkflowPage.__new__(TimingCalibrationWorkflowPage)

    def test_default_is_encoder_when_no_combo(self):
        p = self._page()
        self.assertEqual(p._detector_mode(), "encoder")

    def test_make_tracker_encoder(self):
        p = self._page()
        p._detector_combo = _Combo("encoder")
        p._controller = _Ctrl()
        self.assertIsInstance(p._make_tracker(), _EncoderMotion)

    def test_make_tracker_camera(self):
        p = self._page()
        p._detector_combo = _Combo("camera")
        p._camera_manager = object()
        p._optical_cam_idx = 0
        self.assertIsInstance(p._make_tracker(), _FrameMotion)


class TestEncoderThreshold(unittest.TestCase):
    """The settle sweep failed on the encoder because the camera-style
    scene-relative threshold (calibrated from a short-segment move whose 'peak'
    caught one poll spanning the whole move) sat ABOVE the sweep's smaller
    per-poll motions → motion never detected → timeouts. The stage-position
    detector instead uses a fixed threshold placed just above the at-rest
    jitter, so real motion (tens–hundreds of µm/poll) always clears it."""

    def test_fixed_threshold_is_small_and_physical(self):
        m = _EncoderMotion(_Ctrl())
        floor, peak, still = m.fixed_threshold()
        self.assertEqual(floor, 0.0)
        self.assertLessEqual(still, 5.0)         # small vs any real motion
        self.assertGreater(peak, still)

    def test_encoder_threshold_sits_above_jitter_below_motion(self):
        page = TimingCalibrationWorkflowPage.__new__(TimingCalibrationWorkflowPage)
        # ~1-2 µm at-rest jitter with a couple of small spikes
        rest = [1.0] * 16 + [2.0, 2.5, 5.0, 6.0]
        page._dwell = lambda tracker, dur: list(rest)   # no real sleeping/moving

        class _C:
            def move_xy_absolute(self, *a, **k):
                pass

            def wait_for_xy_arrival(self, *a, **k):
                return True
        tracker = _EncoderMotion(_Ctrl())
        floor, peak, thresh = page._encoder_threshold(_C(), tracker, 0.0, 0.0)
        # threshold above the jitter…
        self.assertGreaterEqual(thresh, max(rest))
        # …and far below a real per-poll motion (e.g. 40 µm at ~1 mm/s)
        self.assertLess(thresh, 40.0)

    def test_encoder_threshold_floors_at_still_um(self):
        page = TimingCalibrationWorkflowPage.__new__(TimingCalibrationWorkflowPage)
        page._dwell = lambda tracker, dur: [0.0] * 20   # perfectly still stage

        class _C:
            def move_xy_absolute(self, *a, **k):
                pass

            def wait_for_xy_arrival(self, *a, **k):
                return True
        _f, _p, thresh = page._encoder_threshold(_C(), _EncoderMotion(_Ctrl()),
                                                 0.0, 0.0)
        self.assertGreaterEqual(thresh, _EncoderMotion.STILL_UM)


if __name__ == "__main__":
    unittest.main()
