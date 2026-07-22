"""
v7.5.x tests — calibration usability pop-up.

Covers the backend the pop-up stands on (operator request, changes.txt line 22 —
Cason 7.21): a per-machine CalibrationStatusStore holding an XY travel odometer +
per-type (XY/Z/P) last-calibrated timestamps + the M/H thresholds, the
PositionPoller odometer hook that feeds it, and the pure threshold evaluation the
QMessageBox renders.

  1. CalibrationStatusStore — defaults, thresholds round-trip, timestamps,
     hours_since (injected now), odometer accumulate + sanity cap + guards,
     travel-since-cal, throttled save + flush, partial-file merge on load.
  2. PositionPoller._accumulate_xy_travel — chord math, None/first-sample guard,
     frame-reset on set_stages.
  3. evaluate_calibration_status — never-calibrated / over-M / over-H / all-clean.

The QMessageBox itself is exercised manually (as with the other startup prompts).
"""

import os
import tempfile
import time
import unittest
from datetime import datetime, timedelta
from pathlib import Path

from SupportClasses.CalibrationStatusStore import (
    CalibrationStatusStore, _ODOM_MAX_STEP_UM)


class TestCalibrationStatusStore(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "calibration_status.json"
        self.store = CalibrationStatusStore(path=self.path)

    def tearDown(self):
        self._tmp.cleanup()

    def test_defaults(self):
        m, h = self.store.get_thresholds()
        self.assertGreater(m, 0)
        self.assertGreater(h, 0)
        self.assertEqual(self.store.get_xy_travel_um(), 0.0)
        for kind in ("xy", "z", "p"):
            self.assertIsNone(self.store.get_calibrated_at(kind))
            self.assertIsNone(self.store.hours_since(kind))
        self.assertIsNone(self.store.xy_travel_since_cal_um())

    def test_thresholds_round_trip_and_clamp(self):
        self.store.set_thresholds(m_mm=500.0, h_hours=72.0)
        self.assertEqual(self.store.get_thresholds(), (500.0, 72.0))
        # Negatives clamp to 0 (= disabled).
        self.store.set_thresholds(m_mm=-5.0, h_hours=-1.0)
        self.assertEqual(self.store.get_thresholds(), (0.0, 0.0))
        # Persisted across reload.
        reloaded = CalibrationStatusStore(path=self.path)
        self.assertEqual(reloaded.get_thresholds(), (0.0, 0.0))

    def test_mark_calibrated_and_persist(self):
        when = "2026-07-20T09:00:00"
        self.store.mark_calibrated("z", when=when)
        self.assertEqual(self.store.get_calibrated_at("z"), when)
        reloaded = CalibrationStatusStore(path=self.path)
        self.assertEqual(reloaded.get_calibrated_at("z"), when)
        with self.assertRaises(ValueError):
            self.store.mark_calibrated("bogus")

    def test_hours_since_injected_now(self):
        now = datetime(2026, 7, 21, 12, 0, 0)
        self.store.mark_calibrated("p", when="2026-07-21T09:00:00")
        self.assertAlmostEqual(self.store.hours_since("p", now=now), 3.0, places=3)
        # Unparseable / missing → None.
        self.assertIsNone(self.store.hours_since("xy", now=now))

    def test_odometer_accumulate_and_guards(self):
        self.store.add_xy_travel_um(100.0)
        self.store.add_xy_travel_um(50.0)
        self.assertAlmostEqual(self.store.get_xy_travel_um(), 150.0)
        # Non-positive / bad / over-cap deltas ignored.
        self.store.add_xy_travel_um(0.0)
        self.store.add_xy_travel_um(-10.0)
        self.store.add_xy_travel_um(_ODOM_MAX_STEP_UM + 1.0)
        self.store.add_xy_travel_um(float("nan"))
        # Sub-µm jitter at rest is dropped (below the min-step floor).
        self.store.add_xy_travel_um(0.4)
        self.assertAlmostEqual(self.store.get_xy_travel_um(), 150.0)

    def test_travel_since_cal_resets_on_mark(self):
        self.store.add_xy_travel_um(1000.0)
        self.assertIsNone(self.store.xy_travel_since_cal_um())  # never cal'd yet
        self.store.mark_calibrated("xy")
        self.assertAlmostEqual(self.store.xy_travel_since_cal_um(), 0.0)
        self.store.add_xy_travel_um(2500.0)
        self.assertAlmostEqual(self.store.xy_travel_since_cal_um(), 2500.0)

    def test_throttled_save_then_flush(self):
        # add_xy_travel_um is throttled — the first small accumulation may not
        # hit disk yet, but flush() forces it.
        self.store.add_xy_travel_um(42.0)
        self.store.flush()
        reloaded = CalibrationStatusStore(path=self.path)
        self.assertAlmostEqual(reloaded.get_xy_travel_um(), 42.0)

    def test_partial_file_merge(self):
        # A partial / older file still yields a complete structure.
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text('{"xy_travel_um": 77.0, "z": {"at": "x"}}')
        store = CalibrationStatusStore(path=self.path)
        self.assertAlmostEqual(store.get_xy_travel_um(), 77.0)
        self.assertEqual(store.get_calibrated_at("z"), "x")
        m, h = store.get_thresholds()  # defaults filled in
        self.assertGreater(m, 0)
        self.assertGreater(h, 0)


class TestPositionPollerOdometer(unittest.TestCase):
    """Drive PositionPoller._accumulate_xy_travel directly (no threads)."""

    def _make_poller(self):
        from SupportClasses.StageController import PositionPoller
        poller = PositionPoller(poll_interval=0.01)
        seen = []
        poller.on_xy_travel = lambda d: seen.append(d)
        return poller, seen

    def test_chord_sum_and_first_sample_guard(self):
        poller, seen = self._make_poller()
        # First sample: no previous → no callback.
        poller._accumulate_xy_travel(0.0, 0.0)
        self.assertEqual(seen, [])
        # 3-4-5 triangle then a pure-x hop.
        poller._accumulate_xy_travel(3.0, 4.0)   # dist 5
        poller._accumulate_xy_travel(3.0, 4.0)   # dist 0 → no callback
        poller._accumulate_xy_travel(13.0, 4.0)  # dist 10
        self.assertEqual(len(seen), 2)
        self.assertAlmostEqual(seen[0], 5.0)
        self.assertAlmostEqual(seen[1], 10.0)

    def test_set_stages_resets_frame(self):
        poller, seen = self._make_poller()
        poller._accumulate_xy_travel(0.0, 0.0)
        poller._accumulate_xy_travel(100.0, 0.0)
        self.assertEqual(len(seen), 1)
        # A stage (re)assignment drops the previous sample so we don't diff
        # across a coordinate re-frame.
        poller.set_stages(xy_stage=None, zp_stage=None)
        self.assertIsNone(poller._last_odom_xy)
        poller._accumulate_xy_travel(5000.0, 0.0)  # first after reset → no cb
        self.assertEqual(len(seen), 1)


class TestEvaluateCalibrationStatus(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "cs.json"
        self.store = CalibrationStatusStore(path=self.path)

    def tearDown(self):
        self._tmp.cleanup()

    def _eval(self, now=None):
        from gui.dialogs.calibration_status_dialog import (
            evaluate_calibration_status)
        return evaluate_calibration_status(self.store, now=now)

    def test_never_calibrated_needs_attention(self):
        st = self._eval()
        self.assertTrue(st["attention"])
        self.assertEqual(len(st["warnings"]), 3)  # XY, Z, P all never done

    def test_all_clean(self):
        now = datetime(2026, 7, 21, 12, 0, 0)
        recent = (now - timedelta(hours=1)).isoformat(timespec="seconds")
        for kind in ("xy", "z", "p"):
            self.store.mark_calibrated(kind, when=recent)
        self.store.set_thresholds(m_mm=1000.0, h_hours=168.0)
        st = self._eval(now=now)
        self.assertFalse(st["attention"])
        self.assertEqual(st["warnings"], [])

    def test_over_hours_warns_update(self):
        now = datetime(2026, 7, 21, 12, 0, 0)
        old = (now - timedelta(hours=200)).isoformat(timespec="seconds")
        for kind in ("xy", "z", "p"):
            self.store.mark_calibrated(kind, when=old)
        self.store.set_thresholds(m_mm=0.0, h_hours=168.0)  # travel off, time on
        st = self._eval(now=now)
        self.assertTrue(st["attention"])
        self.assertTrue(any("Update" in w for w in st["warnings"]))

    def test_over_travel_warns_recalibrate_xy(self):
        now = datetime(2026, 7, 21, 12, 0, 0)
        recent = (now - timedelta(hours=1)).isoformat(timespec="seconds")
        for kind in ("xy", "z", "p"):
            self.store.mark_calibrated(kind, when=recent)
        self.store.set_thresholds(m_mm=100.0, h_hours=0.0)  # time off, travel on
        # 250 mm of travel since the XY cal (mark_calibrated snapshotted 0).
        self.store.add_xy_travel_um(250_000.0)
        st = self._eval(now=now)
        self.assertTrue(st["attention"])
        self.assertTrue(any("Recalibrate X/Y" in w for w in st["warnings"]))


if __name__ == "__main__":
    unittest.main()
