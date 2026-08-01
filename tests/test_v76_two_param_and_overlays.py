"""test_v76_two_param_and_overlays.py — the two driving parameters
(resolution + top speed), the hardware-limit warnings, the path overlays, and
the never-stalling live position.

Accuracy through a corner is bought with TIME, so the operator states the trade
directly: how fine, and how fast. Everything else — per-section lookahead and
speed, pump flow, corner stops, the time estimate — derives from those two.

Offscreen Qt; no hardware.
"""

import os
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication([])

from gui.pages.workflows.quick_print_workflow import (
    QuickPrintWorkflowPage, _overlay_channels)
from gui.widgets.print_trajectory_monitor import PrintTrajectoryMonitorView
from SupportClasses import XYFeedPlan as FP
from SupportClasses.XYStageModel import StageCharacteristics


ME3B = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                            top_speed_um_s=5945.6, control_loop_ms=31.75)


# ── The kinematics resolution + warnings (no full page build) ─────────

class _Page(QuickPrintWorkflowPage):
    """A partially-constructed page: the real methods under test, with the few
    widgets/inputs they read stubbed. Mirrors the existing suites' idiom."""

    def __init__(self, *, top_speed=2.5, res_um=30.0, area=0.000134,
                 mod=1.0, max_flow=0.0, xy_max=5.95):
        self._top = SimpleNamespace(value=lambda: top_speed)
        self._res = SimpleNamespace(value=lambda: res_um)
        self._top_speed_spin = self._top
        self._resolution_spin = self._res
        self._area = area
        self._mod = mod
        self._max_flow = max_flow
        self._xy_max = xy_max

    # inputs the code under test reads
    def _needle_cross_section_mm2(self):
        return self._area

    def _extrusion_modifier(self):
        return self._mod

    def _max_pump_flow_uL_s(self):
        return self._max_flow

    def _xy_max_mm_s(self):
        return self._xy_max

    def _preflow_s(self):
        return 0.25

    def _motion_mode(self):
        return "velocity"

    def _path_segments_for_selection(self):
        return []


class TestResolvedKinematics(unittest.TestCase):
    def test_speed_is_the_min_of_request_stage_and_flow(self):
        # request below every limit → honoured exactly
        p = _Page(top_speed=2.0)
        speed, flow, prime = p._resolved_print_kinematics()
        self.assertAlmostEqual(speed, 2.0)
        # flow FOLLOWS the speed: area × speed × mod
        self.assertAlmostEqual(flow, 0.000134 * 2.0)
        self.assertAlmostEqual(prime, flow * 0.25)

    def test_stage_max_binds(self):
        p = _Page(top_speed=12.0, xy_max=5.95)
        speed, _f, _p = p._resolved_print_kinematics()
        self.assertAlmostEqual(speed, 5.95)

    def test_flow_ceiling_binds(self):
        # ceiling 0.0002 µL/s ÷ (area × 1) = ~1.49 mm/s
        p = _Page(top_speed=5.0, max_flow=0.0002)
        speed, flow, _p = p._resolved_print_kinematics()
        self.assertLess(speed, 1.6)
        self.assertLessEqual(flow, 0.0002 + 1e-9)

    def test_volume_per_mm_is_speed_invariant(self):
        """Changing the speed must change the TIME, never the bead."""
        slow = _Page(top_speed=1.0)._resolved_print_kinematics()
        fast = _Page(top_speed=4.0)._resolved_print_kinematics()
        self.assertAlmostEqual(slow[1] / slow[0], fast[1] / fast[0], places=9)

    def test_no_needle_uses_the_fallback_flow(self):
        p = _Page(top_speed=2.0, area=0.0)
        speed, flow, _p = p._resolved_print_kinematics()
        self.assertAlmostEqual(speed, 2.0)
        self.assertGreater(flow, 0.0)

    def test_extrusion_modifier_scales_flow_only(self):
        thin = _Page(top_speed=2.0, mod=1.0)._resolved_print_kinematics()
        thick = _Page(top_speed=2.0, mod=2.0)._resolved_print_kinematics()
        self.assertAlmostEqual(thin[0], thick[0])
        self.assertAlmostEqual(thick[1], thin[1] * 2.0)


class TestLimitWarnings(unittest.TestCase):
    def _warn(self, page):
        msgs = []
        warned = page._append_limit_warnings(
            msgs, page._top_speed_mm_s(), page._resolution_um())
        return msgs, warned

    def test_flow_ceiling_warning_names_the_numbers(self):
        p = _Page(top_speed=5.0, max_flow=0.0002)
        msgs, warned = self._warn(p)
        self.assertTrue(warned)
        txt = " ".join(msgs)
        self.assertIn("flow ceiling", txt)
        self.assertIn("auto-limited", txt)

    def test_stage_max_warning(self):
        p = _Page(top_speed=12.0, xy_max=5.95)
        msgs, warned = self._warn(p)
        self.assertTrue(warned)
        self.assertIn("measured stage max", " ".join(msgs))

    def test_no_warning_when_within_limits(self):
        p = _Page(top_speed=2.0, xy_max=5.95, max_flow=1.0)
        msgs, warned = self._warn(p)
        self.assertFalse(any("⚠" in m for m in msgs) and warned,
                         "a valid request must not be flagged")

    def test_resolution_floor_warning_only_below_the_floor(self):
        """Uses a stamped temp store so the assertion is unconditional — an
        earlier version skipped when the store looked uncharacterised, which
        silently hid a NameError that disabled ALL of these warnings."""
        from tests.support.store_fixture import use_temp_store
        store = use_temp_store(self)
        store.set_velocity_dead_time_s(0.067, n=5, spread_s=0.0005, tau_s=0.027)
        store.set_xy_max_speed_um_s(5945.6)
        store.set_control_loop_ms(31.75)
        floor = FP.min_attainable_resolution_um(ME3B)
        self.assertGreater(floor, 15.0)

        # a request BELOW the floor warns…
        msgs, warned = self._warn(_Page(top_speed=2.0, res_um=floor / 2.0))
        self.assertTrue(warned)
        self.assertIn("finer than this machine", " ".join(msgs))
        # …and the 30 µm element the hardware PROVED it can hold must not.
        msgs2, _w2 = self._warn(_Page(top_speed=2.0, res_um=30.0))
        self.assertNotIn("finer than this machine", " ".join(msgs2))

    def test_uncharacterised_machine_points_at_the_calibration(self):
        from tests.support.store_fixture import use_temp_store
        use_temp_store(self)          # empty store → not characterised
        msgs, warned = self._warn(_Page(top_speed=2.0))
        self.assertTrue(warned)
        self.assertIn("not characterised", " ".join(msgs))


# ── Overlay channels (pure) ──────────────────────────────────────────

class _FakeSim:
    def __init__(self, samples, cross_profile=None):
        self.samples = samples
        self.cross_profile = cross_profile or []


class TestOverlayChannels(unittest.TestCase):
    def test_channels_align_with_the_point_count(self):
        samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.5), (2.0, 0.0, 1.0)]
        sp, fl, er, tm = _overlay_channels(_FakeSim(samples),
                                           vol_per_mm=0.05)
        for ch in (sp, fl, er, tm):
            self.assertEqual(len(ch), len(samples))

    def test_speed_and_flow(self):
        samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.5), (2.0, 0.0, 1.0)]
        sp, fl, _er, _tm = _overlay_channels(_FakeSim(samples),
                                            vol_per_mm=0.05)
        self.assertAlmostEqual(sp[1], 2.0)        # 1 mm in 0.5 s
        self.assertAlmostEqual(sp[0], sp[1])      # first copies second
        self.assertAlmostEqual(fl[1], 2.0 * 0.05)

    def test_time_offset_makes_one_print_clock(self):
        samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.4)]
        _sp, _fl, _er, tm = _overlay_channels(_FakeSim(samples),
                                              vol_per_mm=0.0, t_base=7.0)
        self.assertAlmostEqual(tm[0], 7.0)
        self.assertAlmostEqual(tm[1], 7.4)

    def test_error_from_cross_profile(self):
        samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.5), (2.0, 0.0, 1.0)]
        prof = [(0.0, 5.0), (1.0, -40.0), (2.0, 12.0)]
        _sp, _fl, er, _tm = _overlay_channels(_FakeSim(samples, prof),
                                              vol_per_mm=0.0)
        self.assertTrue(all(v >= 0 for v in er), "error is a magnitude")
        self.assertAlmostEqual(max(er), 40.0)

    def test_empty_and_missing_profile(self):
        self.assertEqual(_overlay_channels(_FakeSim([]), vol_per_mm=1.0),
                         ([], [], [], []))
        sp, fl, er, tm = _overlay_channels(
            _FakeSim([(0.0, 0.0, 0.0), (1.0, 0.0, 1.0)]), vol_per_mm=1.0)
        self.assertEqual(er, [0.0, 0.0])          # no profile → zeros


# ── Monitor overlay rendering ────────────────────────────────────────

def _seg(n=5, y=0.0):
    return [(i * 500.0, y) for i in range(n)]


class TestMonitorOverlay(unittest.TestCase):
    def test_set_and_clear(self):
        v = PrintTrajectoryMonitorView()
        v.set_predicted_path([_seg()])
        v.set_overlay("speed", [[1.0, 2.0, 3.0, 4.0, 5.0]], "mm/s")
        self.assertEqual(v.overlay_mode(), "speed")
        self.assertAlmostEqual(v._overlay_vmin, 1.0)
        self.assertAlmostEqual(v._overlay_vmax, 5.0)
        v.set_overlay(None)
        self.assertIsNone(v.overlay_mode())

    def test_explicit_range_wins(self):
        v = PrintTrajectoryMonitorView()
        v.set_predicted_path([_seg()])
        v.set_overlay("flow", [[1.0] * 5], "µL/s", vmin=0.0, vmax=10.0)
        self.assertAlmostEqual(v._overlay_vmin, 0.0)
        self.assertAlmostEqual(v._overlay_vmax, 10.0)

    def test_degenerate_range_does_not_divide_by_zero(self):
        v = PrintTrajectoryMonitorView()
        v.set_predicted_path([_seg()])
        v.set_overlay("time", [[4.0] * 5], "s")     # all equal
        self.assertGreater(v._overlay_vmax, v._overlay_vmin)
        v.resize(200, 200)
        v.grab()                                    # must paint, not crash

    def test_new_prediction_clears_stale_values(self):
        v = PrintTrajectoryMonitorView()
        v.set_predicted_path([_seg()])
        v.set_overlay("speed", [[1.0, 2.0, 3.0, 4.0, 5.0]], "mm/s")
        v.set_predicted_path([_seg(7)])
        self.assertEqual(v._overlay_vals, [],
                         "values from the old prediction must not be reused")

    def test_mismatched_counts_fall_back_to_plain(self):
        v = PrintTrajectoryMonitorView()
        v.set_predicted_path([_seg(5)])
        v.set_overlay("speed", [[1.0, 2.0]], "mm/s")   # 2 values, 5 points
        v.resize(200, 200)
        v.grab()                                        # renders dashed, no crash

    def test_paints_with_overlay_and_legend(self):
        v = PrintTrajectoryMonitorView()
        v.set_planned_path([_seg()])
        v.set_predicted_path([_seg(5, y=20.0)])
        v.set_overlay("error", [[0.0, 10.0, 30.0, 60.0, 120.0]], "µm")
        v.resize(320, 240)
        img = v.grab().toImage()
        self.assertFalse(img.isNull())


class TestPagePillRouting(unittest.TestCase):
    def _page(self):
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._traj_view = MagicMock()
        page._pred_overlays = {
            "speed": {"values": [[1.0, 2.0]], "unit": "mm/s",
                      "vmin": 1.0, "vmax": 2.0}}
        return page

    def test_none_clears(self):
        page = self._page()
        page._overlay_group = None
        page._apply_overlay_mode()
        page._traj_view.set_overlay.assert_called_once_with(None)

    def test_selected_channel_is_pushed(self):
        page = self._page()
        btn = SimpleNamespace(_overlay_key="speed")
        page._overlay_group = SimpleNamespace(checkedButton=lambda: btn)
        page._apply_overlay_mode()
        page._traj_view.set_overlay.assert_called_once_with(
            "speed", [[1.0, 2.0]], "mm/s", 1.0, 2.0)

    def test_missing_channel_clears(self):
        page = self._page()
        btn = SimpleNamespace(_overlay_key="flow")     # not simulated
        page._overlay_group = SimpleNamespace(checkedButton=lambda: btn)
        page._apply_overlay_mode()
        page._traj_view.set_overlay.assert_called_once_with(None)

    def test_predicted_slot_stores_overlays_and_applies(self):
        page = self._page()
        page._pred_gen = 4
        page._apply_overlay_mode = MagicMock()
        page._on_predicted(4, [[(0.0, 0.0), (1.0, 1.0)]], "note",
                           {"time": {"values": [[0.0, 1.0]], "unit": "s"}})
        self.assertIn("time", page._pred_overlays)
        page._apply_overlay_mode.assert_called_once()

    def test_stale_generation_is_dropped(self):
        page = self._page()
        page._pred_gen = 9
        before = page._pred_overlays
        page._on_predicted(3, [], "stale", {"speed": {}})
        self.assertIs(page._pred_overlays, before)


# ── Profile migration (speed_pct → top_speed) ────────────────────────

class TestProfileMigration(unittest.TestCase):
    def _page(self, anchor=5.95):
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._flow_limited_xy_max_mm_s = lambda: anchor
        return page

    def test_percent_becomes_mm_s(self):
        out = self._page()._migrate_legacy_settings({"speed_pct": 50.0})
        self.assertAlmostEqual(out["top_speed"], 2.98, places=2)

    def test_existing_top_speed_is_untouched(self):
        out = self._page()._migrate_legacy_settings(
            {"speed_pct": 10.0, "top_speed": 3.5})
        self.assertEqual(out["top_speed"], 3.5)

    def test_absent_key_is_a_noop(self):
        out = self._page()._migrate_legacy_settings({"printz": 0.2})
        self.assertNotIn("top_speed", out)

    def test_garbage_value_is_survivable(self):
        out = self._page()._migrate_legacy_settings({"speed_pct": "bogus"})
        self.assertNotIn("top_speed", out)

    def test_dialog_invokes_the_migrator_on_apply(self):
        from gui.dialogs.workflow_settings_dialog import WorkflowSettingsDialog
        seen = {}

        def _mig(values):
            seen["called"] = dict(values)
            values["printz"] = 9.0
            return values

        dlg = WorkflowSettingsDialog("unit_test_v76", "Unit", migrate=_mig)
        from PySide6.QtWidgets import QDoubleSpinBox
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 40.0)
        sec = dlg.add_section("S")
        sec.add("printz", "Z", spin, 0.2)
        dlg.apply({"printz": 1.0})
        self.assertEqual(seen["called"], {"printz": 1.0})
        self.assertAlmostEqual(spin.value(), 9.0)      # migrator's value won
        dlg.close()

    def test_dialog_survives_a_broken_migrator(self):
        from gui.dialogs.workflow_settings_dialog import WorkflowSettingsDialog

        def _boom(_values):
            raise RuntimeError("bad")

        dlg = WorkflowSettingsDialog("unit_test_v76b", "Unit", migrate=_boom)
        from PySide6.QtWidgets import QDoubleSpinBox
        spin = QDoubleSpinBox()
        sec = dlg.add_section("S")
        sec.add("printz", "Z", spin, 0.2)
        dlg.apply({"printz": 1.0})                     # must not raise
        self.assertAlmostEqual(spin.value(), 1.0)
        dlg.close()


# ── Live position: cache back-fill + fast timer ──────────────────────

class TestPositionBackfill(unittest.TestCase):
    def _ctrl(self):
        from SupportClasses.StageController import StageController, PositionPoller
        c = StageController.__new__(StageController)
        c._pos_poller = PositionPoller.__new__(PositionPoller)
        c._pos_poller._lock = threading.RLock()
        c._pos_poller._xy_pos = (None, None, None)
        c._pos_poller._zp_pos = (None, None, None, None)
        c._pos_poller._last_odom_xy = None
        c._pos_poller.on_xy_travel = MagicMock()
        return c

    def test_direct_read_backfills_the_cache(self):
        c = self._ctrl()
        c.xy_stage = SimpleNamespace(
            get_current_position=lambda: (1234.0, 5678.0, 0.0))
        self.assertEqual(c.get_xy_position(cached=True), (None, None, None))
        self.assertEqual(c.get_xy_position(cached=False),
                         (1234.0, 5678.0, 0.0))
        # the frozen cache is now live — this is what unfreezes the monitor
        self.assertEqual(c.get_xy_position(cached=True), (1234.0, 5678.0, 0.0))

    def test_backfill_does_not_touch_the_odometer(self):
        c = self._ctrl()
        c.xy_stage = SimpleNamespace(
            get_current_position=lambda: (1000.0, 0.0, 0.0))
        c.get_xy_position(cached=False)
        c.xy_stage.get_current_position = lambda: (9000.0, 0.0, 0.0)
        c.get_xy_position(cached=False)
        c._pos_poller.on_xy_travel.assert_not_called()

    def test_none_and_failure_are_ignored(self):
        c = self._ctrl()
        c._pos_poller.note_xy((None, None, None))
        c._pos_poller.note_xy(None)
        self.assertEqual(c._pos_poller.xy_position, (None, None, None))
        c.xy_stage = SimpleNamespace(
            get_current_position=lambda: (_ for _ in ()).throw(OSError("x")))
        self.assertEqual(c.get_xy_position(cached=False), (None, None, None))

    def test_note_xy_is_thread_safe(self):
        c = self._ctrl()
        stop = time.monotonic() + 0.2

        def _writer(v):
            while time.monotonic() < stop:
                c._pos_poller.note_xy((v, v, 0.0))
        ts = [threading.Thread(target=_writer, args=(i,), daemon=True)
              for i in (1.0, 2.0, 3.0)]
        for t in ts:
            t.start()
        while time.monotonic() < stop:
            pos = c._pos_poller.xy_position
            self.assertEqual(pos[0], pos[1])   # never a torn tuple
        for t in ts:
            t.join(1.0)

    def test_note_zp_backfill(self):
        c = self._ctrl()
        c._pos_poller.note_zp((1.0, 2.0, 3.0, 4.0))
        self.assertEqual(c._pos_poller.zp_position, (1.0, 2.0, 3.0, 4.0))


class TestLiveTimer(unittest.TestCase):
    def _page(self):
        from PySide6.QtCore import QTimer
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._traj_view = MagicMock()
        page._live_pos_timer = QTimer()
        page._live_pos_timer.setInterval(page._LIVE_POS_MS)
        return page

    def test_start_and_stop(self):
        page = self._page()
        page._set_print_live(True)
        self.assertTrue(page._live_pos_timer.isActive())
        page._traj_view.reset_live.assert_called_once()
        page._traj_view.set_recording.assert_called_with(True)
        page._set_print_live(False)
        self.assertFalse(page._live_pos_timer.isActive())
        page._traj_view.set_recording.assert_called_with(False)

    def test_interval_is_faster_than_the_app_tick(self):
        page = self._page()
        page._set_print_live(True)
        self.assertLessEqual(page._live_pos_timer.interval(), 150)
        page._set_print_live(False)

    def test_camera_is_throttled_during_the_print(self):
        page = self._page()
        cam = MagicMock()
        page._camera_view = cam
        page._set_print_live(True)
        cam.set_throttled.assert_called_with(True)
        page._set_print_live(False)
        cam.set_throttled.assert_called_with(False)

    def test_no_timer_is_survivable(self):
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._traj_view = MagicMock()
        page._set_print_live(True)          # must not raise


class TestCameraThrottle(unittest.TestCase):
    def test_widget_halves_and_restores_the_rate(self):
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget.__new__(CameraWidget)
        w._fps = 20
        w._throttled = False
        w._running = True
        intervals = []
        w._timer = SimpleNamespace(setInterval=intervals.append)
        w.set_throttled(True)
        self.assertEqual(intervals[-1], 100)     # 10 FPS
        w.set_throttled(True)                    # idempotent
        self.assertEqual(len(intervals), 1)
        w.set_throttled(False)
        self.assertEqual(intervals[-1], 50)      # back to 20 FPS

    def test_not_running_only_records_the_flag(self):
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget.__new__(CameraWidget)
        w._fps = 15
        w._throttled = False
        w._running = False
        w._timer = SimpleNamespace(
            setInterval=lambda _v: self.fail("must not touch a stopped timer"))
        w.set_throttled(True)
        self.assertTrue(w._throttled)


if __name__ == "__main__":
    unittest.main()
