"""test_v75x_print_timing_calibration.py — segment settle-delay sweep.

Issue N collinear line-segment moves back-to-back, watch the camera frames
(plain frame-difference — no optical flow) until they stop changing, and log the
delay from the last command to stillness. Sweep N = 1, 2, 3, … ; the delay's
growth per segment is the controller's per-segment phase lag.

Covers: the phase-model store (intercept + slope, refined across runs), the
delay-vs-N line fit, the tile registration / page build / safety gate, the
plot widgets, and the worker sweep against a fake stage whose 'frames' stay
'moving' until a queued settle elapses (so the backlog — and the measured delay
— grows with N).
"""

import math
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from PySide6.QtWidgets import QApplication

from SupportClasses.PrintTimingCalibrationStore import PrintTimingCalibrationStore
import gui.pages.workflows.timing_calibration_workflow as tcw
from gui.pages.workflows.timing_calibration_workflow import (
    TimingCalibrationWorkflowPage, _TimingConfig, _fit_line,
)


# ── 1. Phase-model store ──────────────────────────────────────────────

class TestPhaseStore(unittest.TestCase):
    def _store(self):
        d = tempfile.mkdtemp()
        return PrintTimingCalibrationStore(Path(d) / "t.json"), d

    def test_update_and_get_phase(self):
        st, _ = self._store()
        st.update_phase(5.0, 1.0, intercept_s=0.12, slope_s_per_seg=0.03,
                        n_points=8)
        e = st.get_phase(5.0, 1.0)
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e["slope_s_per_seg"], 0.03, places=4)
        self.assertAlmostEqual(e["intercept_s"], 0.12, places=4)
        self.assertEqual(e["runs"], 1)

    def test_refines_across_runs(self):
        st, _ = self._store()
        st.update_phase(5.0, 1.0, 0.10, 0.02, 8)
        e = st.update_phase(5.0, 1.0, 0.20, 0.04, 8)   # run 2
        self.assertEqual(e["runs"], 2)
        self.assertAlmostEqual(e["slope_s_per_seg"], 0.03, places=4)  # mean
        self.assertAlmostEqual(e["intercept_s"], 0.15, places=4)

    def test_buckets_by_speed_and_seg(self):
        st, _ = self._store()
        st.update_phase(5.0, 1.0, 0.1, 0.02, 4)
        st.update_phase(10.0, 1.0, 0.1, 0.02, 4)
        st.update_phase(5.0, 2.0, 0.1, 0.02, 4)
        self.assertIsNotNone(st.get_phase(5.0, 1.0))
        self.assertIsNotNone(st.get_phase(10.0, 1.0))
        self.assertIsNotNone(st.get_phase(5.0, 2.0))

    def test_persistence_round_trip(self):
        st, d = self._store()
        st.update_phase(5.0, 1.0, 0.12, 0.03, 8)
        st2 = PrintTimingCalibrationStore(Path(d) / "t.json")
        self.assertAlmostEqual(st2.get_phase(5.0, 1.0)["slope_s_per_seg"],
                               0.03, places=4)


# ── 2. delay-vs-N line fit ────────────────────────────────────────────

class TestFitLine(unittest.TestCase):
    def test_recovers_slope_intercept(self):
        # delay = 0.1 + 0.05·N
        pts = [(n, 0.1 + 0.05 * n) for n in range(1, 9)]
        slope, intercept = _fit_line(pts)
        self.assertAlmostEqual(slope, 0.05, places=4)
        self.assertAlmostEqual(intercept, 0.1, places=4)

    def test_single_point(self):
        slope, intercept = _fit_line([(1, 0.2)])
        self.assertEqual(slope, 0.0)
        self.assertAlmostEqual(intercept, 0.2, places=4)

    def test_empty(self):
        self.assertEqual(_fit_line([]), (0.0, 0.0))


# ── Qt base ───────────────────────────────────────────────────────────

class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class _GateCtrl:
    is_xy_connected = True
    is_zp_connected = True
    zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    def default_plate_center_um(self):
        return (0.0, 0.0)


# ── 3. Registration + build + gate ────────────────────────────────────

class TestRegistrationAndGate(_QtBase):
    def test_registered_enabled(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        tile = next((t for t in WORKFLOWS
                     if t.workflow_id == "timing_calibration"), None)
        self.assertIsNotNone(tile)
        self.assertTrue(tile.enabled)

    def test_page_builds(self):
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        self.assertEqual(page.get_page_title(), "XY↔ZP Timing Calibration")
        cfg = page._gather_config()
        self.assertEqual(cfg.max_segments, 8)

    def test_refuses_without_safe_z(self):
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        page._safe_z = None
        page._on_start()
        self.assertFalse(page._running())
        self.assertIn("Safe Z", page._status.text())

    def test_refuses_without_camera_in_camera_mode(self):
        # Only the 'Microscope camera' detector requires the camera. Select it,
        # then with no camera_manager the preflight must refuse.
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        page._safe_z = 5.0
        page._plate = object()                 # so center resolves
        page._detector_combo.setCurrentIndex(1)   # 'Microscope camera'
        page._on_start()                       # camera_manager is None
        self.assertFalse(page._running())
        self.assertIn("microscope", page._status.text().lower())

    def test_default_detector_is_encoder_no_camera_required(self):
        # The default 'Stage position' detector needs no camera — the preflight
        # must NOT bounce on a missing camera (it clears the camera gate).
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        self.assertEqual(page._detector_mode(), "encoder")


# ── 4. Plot widgets ───────────────────────────────────────────────────

class TestPlots(_QtBase):
    def test_motion_strip(self):
        from gui.pages.workflows.timing_calibration_workflow import _MotionStrip
        st = _MotionStrip(window_s=1.0)
        st.set_threshold(0.5)
        st.add(0.0, 5.0)
        st.add(0.5, 0.1)
        st.mark(0.5, "still")
        self.assertEqual(len(st._pts), 2)
        st.add(2.0, 0.1)                       # window trims old
        self.assertTrue(all(t >= 1.0 for t, _ in st._pts))
        st.clear()
        self.assertEqual(len(st._pts), 0)

    def test_delay_plot(self):
        from gui.pages.workflows.timing_calibration_workflow import _DelayPlot
        dp = _DelayPlot()
        dp.set_result([(1, 0.1), (2, 0.15), (3, 0.2)], slope=0.05,
                      intercept=0.05)
        self.assertEqual(len(dp._pts), 3)
        self.assertAlmostEqual(dp._slope, 0.05)
        dp.clear()
        self.assertEqual(len(dp._pts), 0)


# ── Worker fakes ──────────────────────────────────────────────────────

class _SweepCtrl:
    """Fake stage modelling a command QUEUE with distance-dependent move time:
    each move is busy for ``base + distance/speed``; if a move arrives while
    still busy it extends the busy time (backlog). So streaming N equal segments
    faster than they execute → the settle delay grows with N (positive slope);
    and timing single moves of increasing distance → time grows ∝ distance with
    slope 1/speed (so the top-speed measurement recovers ``speed``)."""

    def __init__(self, settle_s=0.12, speed_mm_s=20.0):
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self.xy_max_set = None
        self.xy_stage = SimpleNamespace(
            set_speed_mm_s=lambda v: None,
            set_velocity=lambda v: None,
            set_acceleration=lambda v: None,
            set_max_speed_um_s=lambda v: setattr(self, "xy_max_set", v))
        self.moves = []
        self.retracts = []
        self._base = settle_s
        self._speed = speed_mm_s
        self._busy_until = 0.0
        self._last = None

    def default_plate_center_um(self):
        return (0.0, 0.0)

    def get_xy_position(self, cached=True):
        return (3000.0, 4000.0)        # absolute µm → 3.0, 4.0 mm zero-ref

    def ensure_retracted_to(self, z, *a, **k):
        self.retracts.append(z)
        return True

    def move_xy_absolute(self, x, y, from_zero_ref=True, **k):
        now = time.monotonic()
        cur = (float(x), float(y))
        dist = (math.hypot(cur[0] - self._last[0], cur[1] - self._last[1])
                if self._last else 0.0)
        move_time = self._base + dist / max(self._speed, 0.01)
        self._busy_until = max(self._busy_until, now) + move_time
        self._last = cur
        self.moves.append(cur)

    def wait_for_xy_arrival(self, x, y, tolerance_mm=0.1, timeout_s=15.0, **k):
        return True

    def suspend_position_poller(self): pass
    def resume_position_poller(self): pass
    def suspend_zp_watchdog(self): pass
    def resume_zp_watchdog(self): pass

    def moving(self):
        return time.monotonic() < self._busy_until


class _FakeMotion:
    """Frame-difference stand-in: high 'change' while the fake stage is busy,
    near-zero when still. (Receives the controller as its camera_manager.)"""

    def __init__(self, mgr, cam_idx, **kw):
        self._ctrl = mgr

    def available(self): return True
    def reset(self): pass
    def metric(self): return 10.0 if self._ctrl.moving() else 0.05
    def frame_count(self): return 0


# ── 5. Worker sweep ───────────────────────────────────────────────────

class TestWorkerSweep(_QtBase):
    def test_sweep_measures_stores_and_fits(self):
        ctrl = _SweepCtrl(settle_s=0.12)
        page = TimingCalibrationWorkflowPage(
            ctrl, settings=None, camera_manager=ctrl)
        page._safe_z = 5.0
        page._plate = object()
        page._optical_cam_idx = 0
        page._detector_combo.setCurrentIndex(1)   # camera detector (patched _FrameMotion)
        results = []
        finished = []
        page._bridge.result.connect(lambda d: results.append(d))
        page._bridge.finished.connect(lambda ok, m: finished.append((ok, m)))
        d = tempfile.mkdtemp()
        store = PrintTimingCalibrationStore(Path(d) / "t.json")
        # seg 1mm @ 20mm/s → seg_time 0.05s < settle 0.12s → backlog grows
        cfg = _TimingConfig(max_segments=3, seg_len_mm=1.0, speed_mm_s=20.0,
                            repeats=1, still_window_s=0.12)
        with patch.object(tcw, "_FrameMotion", _FakeMotion), \
                patch.object(tcw, "get_store", return_value=store), \
                patch.object(tcw, "_LOG_DIR", Path(d) / "tl"):
            page._run(cfg)
        self.assertTrue(finished and finished[-1][0])      # finished ok
        self.assertGreater(len(ctrl.moves), 0)
        self.assertIn(5.0, ctrl.retracts)                  # retracted at end
        self.assertTrue(results)                           # result emitted
        pts = results[-1]["points"]
        self.assertGreaterEqual(len(pts), 2)
        # backlog → delay grows with N → positive slope
        self.assertGreater(results[-1]["slope"], 0.0)
        # phase model persisted
        self.assertIsNotNone(store.get_phase(20.0, 1.0))

    def test_stop_before_start(self):
        ctrl = _SweepCtrl()
        page = TimingCalibrationWorkflowPage(
            ctrl, settings=None, camera_manager=ctrl)
        page._safe_z = 5.0
        page._plate = object()
        page._optical_cam_idx = 0
        page._detector_combo.setCurrentIndex(1)   # camera detector (patched _FrameMotion)
        page._stop.set()
        finished = []
        page._bridge.finished.connect(lambda ok, m: finished.append((ok, m)))
        d = tempfile.mkdtemp()
        store = PrintTimingCalibrationStore(Path(d) / "t.json")
        cfg = _TimingConfig(max_segments=3, seg_len_mm=1.0, speed_mm_s=20.0,
                            repeats=1, still_window_s=0.12)
        with patch.object(tcw, "_FrameMotion", _FakeMotion), \
                patch.object(tcw, "get_store", return_value=store), \
                patch.object(tcw, "_LOG_DIR", Path(d) / "tl"):
            page._run(cfg)
        self.assertTrue(finished)
        self.assertFalse(finished[-1][0])


# ── 6. Detector robustness (duplicate skip + motion-first stillness) ──

class _ScriptTracker:
    """Returns a scripted sequence of frame-change values (last value repeats)."""

    def __init__(self, seq):
        self._seq = list(seq)
        self._i = 0

    def metric(self):
        if self._i < len(self._seq):
            v = self._seq[self._i]
            self._i += 1
            return v
        return self._seq[-1] if self._seq else 0.0


class TestDetectorRobustness(_QtBase):
    def test_metric_skips_duplicate_frames(self):
        import importlib
        m = importlib.import_module(
            "gui.pages.workflows.timing_calibration_workflow")
        if m.cv2 is None or m.np is None:
            self.skipTest("cv2/numpy unavailable")
        np = m.np
        holder = {"f": np.zeros((120, 160, 3), dtype=np.uint8)}
        mgr = SimpleNamespace(get_current_frame=lambda i: holder["f"])
        fm = m._FrameMotion(mgr, 0)
        self.assertIsNone(fm.metric())                 # prime
        self.assertIsNone(fm.metric())                 # duplicate → skipped
        f2 = np.zeros((120, 160, 3), dtype=np.uint8)
        f2[:60, :80] = 200
        holder["f"] = f2
        d = fm.metric()                                # changed → diff > 0
        self.assertIsNotNone(d)
        self.assertGreater(d, 0)
        self.assertIsNone(fm.metric())                 # duplicate of f2 → skipped

    def test_watch_requires_motion_first(self):
        # all-quiet sequence: motion never seen → returns None (timeout), NOT a
        # bogus immediate 'still' (the 0 ms-delay bug).
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        page._live_t0 = 0.0
        tr = _ScriptTracker([0.1] * 1000)
        out = page._watch_until_still(tr, thresh=1.0, window=0.08, timeout=0.3)
        self.assertIsNone(out)

    def test_watch_returns_after_motion_then_still(self):
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        page._live_t0 = 0.0
        tr = _ScriptTracker([5.0, 5.0, 5.0] + [0.1] * 1000)   # move, then still
        out = page._watch_until_still(tr, thresh=1.0, window=0.08, timeout=2.0)
        self.assertIsNotNone(out)

    def test_wait_quiet_true_when_below(self):
        page = TimingCalibrationWorkflowPage(_GateCtrl(), settings=None)
        page._live_t0 = 0.0
        tr = _ScriptTracker([0.1] * 1000)
        self.assertTrue(page._wait_quiet(tr, thresh=1.0, window=0.08,
                                         timeout=1.0))


# ── 7. Start location + top-speed measurement ────────────────────────

class TestStartLocation(_QtBase):
    def test_capture_and_reset(self):
        ctrl = _SweepCtrl()
        page = TimingCalibrationWorkflowPage(ctrl, settings=None)
        self.assertIsNone(page._start_xy_mm)              # default: plate centre
        page._capture_start()                             # current = (3000,4000)µm
        self.assertEqual(page._start_xy_mm, (3.0, 4.0))   # zero-ref mm
        self.assertEqual(page._resolve_start_mm(), (3.0, 4.0))
        page._reset_start()
        self.assertIsNone(page._start_xy_mm)


class TestTopSpeed(_QtBase):
    def test_measures_stores_and_applies(self):
        ctrl = _SweepCtrl(settle_s=0.1, speed_mm_s=20.0)   # true top = 20 mm/s
        page = TimingCalibrationWorkflowPage(
            ctrl, settings=None, camera_manager=ctrl)
        page._safe_z = 5.0
        page._plate = object()
        page._optical_cam_idx = 0
        page._detector_combo.setCurrentIndex(1)   # camera detector (patched _FrameMotion)
        results = []
        finished = []
        page._bridge.result.connect(lambda d: results.append(d))
        page._bridge.finished.connect(lambda ok, m: finished.append((ok, m)))
        d = tempfile.mkdtemp()
        store = PrintTimingCalibrationStore(Path(d) / "t.json")
        cfg = _TimingConfig(max_segments=3, seg_len_mm=1.0, speed_mm_s=20.0,
                            repeats=1, still_window_s=0.12, top_max_dist_mm=4.0)
        with patch.object(tcw, "_FrameMotion", _FakeMotion), \
                patch.object(tcw, "get_store", return_value=store), \
                patch.object(tcw, "_LOG_DIR", Path(d) / "tl"):
            page._run_top_speed(cfg)
        self.assertTrue(finished and finished[-1][0])
        self.assertTrue(results)
        # recovered top speed ≈ 20 mm/s → 20000 µm/s (tolerant — frame timing)
        stored = store.get_xy_max_speed_um_s()
        self.assertIsNotNone(stored)
        self.assertGreater(stored, 12000)               # in the right ballpark
        self.assertLess(stored, 32000)
        self.assertIsNotNone(ctrl.xy_max_set)           # applied to the stage
        self.assertIn(5.0, ctrl.retracts)               # retracted at end


if __name__ == "__main__":
    unittest.main()
