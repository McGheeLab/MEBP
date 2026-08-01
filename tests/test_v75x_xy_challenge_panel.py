"""test_v75x_xy_challenge_panel.py — challenge bench UI behaviours + follower
consuming measured calibration + corner-aware velocity.

Covers:
  • _PathOverlay role→colour bookkeeping (best = lowest RMS, newest = last).
  • Per-mode parameter visibility (only the selected mode's rows shown).
  • Coordinate-descent auto-tune keeps the best value per param + persists.
  • _execute_print_path_velocity consumes the stamped machine calibration
    (control_loop_ms → speed cap) + slows into corners (corner scheduling).
No hardware; a synchronous fake stage runs the follower.
"""

import math
import os
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from SupportClasses import XYChallenge as XC
from SupportClasses.PrintTimingCalibrationStore import (
    PrintTimingCalibrationStore, get_store)
from SupportClasses import PrintTimingCalibrationStore as TCS
from tests.support.store_fixture import use_temp_store
from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintCommand, CommandType)

_app = QApplication.instance() or QApplication([])


def _fresh_store(testcase=None):
    """A throwaway store wired into ``get_store()``.

    Pass the TestCase so the previous global is restored via ``addCleanup`` —
    without it the swap leaks into every later test in the process.
    """
    if testcase is not None:
        return use_temp_store(testcase)
    p = os.path.join(tempfile.mkdtemp(), "tc.json")
    st = PrintTimingCalibrationStore(p)
    TCS._store = st          # get_store() → this
    return st


# ── 1. Overlay role bookkeeping ──────────────────────────────────────

class TestOverlayRoles(unittest.TestCase):
    def test_best_is_min_rms_newest_is_last(self):
        from gui.dialogs.xy_challenge_dialog import _PathOverlay
        ov = _PathOverlay()
        ov.add_actual([(0, 0), (1, 0)], rms=100.0)
        ov.add_actual([(0, 0), (1, 0)], rms=30.0)     # best
        ov.add_actual([(0, 0), (1, 0)], rms=60.0)     # newest
        best_i = min(range(len(ov._actuals)), key=lambda i: ov._actuals[i][1])
        self.assertEqual(best_i, 1)
        self.assertEqual(len(ov._actuals) - 1, 2)     # newest = last added
        ov.clear()
        self.assertEqual(ov._actuals, [])


# ── 1b. PARAM_SPECS generates the lookup tables ──────────────────────

class TestParamSpecGeneration(unittest.TestCase):
    """The dialog used to describe every tunable in seven parallel places.
    They are now generated from PARAM_SPECS; these are the values they had
    before the refactor, so a bad spec row cannot silently change behaviour."""

    def test_generated_tables_match_the_originals(self):
        from gui.dialogs import xy_challenge_dialog as d
        self.assertEqual(d.MODE_PARAMS, {
            "open_loop": ["speed", "resolution_um", "pace", "control_hz",
                          "decel"],
            "confirm": ["speed", "resolution_um", "tol_um", "corner_angle"],
            "velocity": ["speed", "resolution_um", "lookahead", "control_hz",
                         "decel", "corner_angle", "corner_factor", "kp", "kd"],
        })
        # NOTE: `pid_kp` is the ONE grid deliberately changed from the original
        # [0, 0.5, 1, 2, 4]. The analytically correct gain for ME3B V1 is π/(2L)
        # × 0.33 ≈ 5.24, which sits ABOVE the old grid's maximum — the descent was
        # structurally unable to reach the right answer. The new grid straddles it
        # (see test_v75x_pid_from_dead_time.py). Everything else is unchanged.
        self.assertEqual(d.AUTOTUNE, {
            "open_loop": [("pace_correction",
                           [1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0])],
            "confirm": [("settle_tol_um", [10.0, 20.0, 30.0, 45.0, 60.0, 90.0]),
                        ("corner_angle_deg", [15.0, 25.0, 35.0, 50.0])],
            "velocity": [("lookahead_mm", [0.2, 0.35, 0.5, 0.65, 0.8, 1.0, 1.3]),
                         ("corner_speed_factor", [0.2, 0.3, 0.4, 0.55, 0.7]),
                         ("pid_kp", [0.0, 1.0, 2.0, 3.5, 5.0, 7.0, 9.0]),
                         ("pid_kd", [0.0, 0.02, 0.05, 0.1])],
        })
        self.assertEqual(d._PARAM_KEY, {
            "pace_correction": "pace", "settle_tol_um": "tol_um",
            "corner_angle_deg": "corner_angle", "lookahead_mm": "lookahead",
            "corner_speed_factor": "corner_factor", "pid_kp": "kp",
            "pid_kd": "kd", "control_hz": "control_hz", "decel_mm": "decel",
        })

    def test_every_spec_gets_a_widget_and_a_row(self):
        from gui.dialogs.xy_challenge_dialog import XYChallengeDialog, PARAM_SPECS
        _fresh_store(self)
        dlg = XYChallengeDialog(
            SimpleNamespace(is_xy_connected=False, is_zp_connected=False),
            safe_z=None, start_xy_mm=(0.0, 0.0))
        try:
            for p in PARAM_SPECS:
                self.assertIsNotNone(getattr(dlg, p.attr, None), msg=p.attr)
                self.assertIn(p.key, dlg._param_rows, msg=p.key)
            self.assertEqual(sorted(dlg._mode_params("velocity")),
                             sorted(p.key for p in PARAM_SPECS))
        finally:
            dlg.close()

    def test_params_round_trip_through_the_store(self):
        from gui.dialogs.xy_challenge_dialog import XYChallengeDialog, PARAM_SPECS
        _fresh_store(self)
        dlg = XYChallengeDialog(
            SimpleNamespace(is_xy_connected=False, is_zp_connected=False),
            safe_z=None, start_xy_mm=(0.0, 0.0))
        try:
            want = {}
            for p in PARAM_SPECS:
                spin = getattr(dlg, p.attr)
                spin.setValue(p.lo + (p.hi - p.lo) * 0.37)
                want[p.key] = spin.value()
            dlg._save_params_to_store()
            for p in PARAM_SPECS:            # scramble, then reload
                getattr(dlg, p.attr).setValue(p.default)
            dlg._load_params_from_store()
            for p in PARAM_SPECS:
                if p.kind == "ui":           # speed is deliberately not persisted
                    continue
                self.assertAlmostEqual(getattr(dlg, p.attr).value(),
                                       want[p.key], places=6, msg=p.key)
        finally:
            dlg.close()


# ── 2. Per-mode param visibility ─────────────────────────────────────

class TestParamVisibility(unittest.TestCase):
    def _dlg(self):
        from gui.dialogs.xy_challenge_dialog import XYChallengeDialog
        _fresh_store(self)
        return XYChallengeDialog(
            SimpleNamespace(is_xy_connected=False, is_zp_connected=False),
            safe_z=None, start_xy_mm=(0.0, 0.0))

    def test_velocity_shows_pid_and_corner_rows(self):
        dlg = self._dlg()
        dlg._mode_combo.setCurrentIndex(2)            # velocity
        dlg._refresh_param_visibility()
        self.assertTrue(dlg._param_rows["kp"].isVisibleTo(dlg))
        self.assertTrue(dlg._param_rows["corner_factor"].isVisibleTo(dlg))
        self.assertTrue(dlg._param_rows["lookahead"].isVisibleTo(dlg))
        self.assertFalse(dlg._param_rows["pace"].isVisibleTo(dlg))
        self.assertFalse(dlg._param_rows["tol_um"].isVisibleTo(dlg))
        dlg.close()

    def test_confirm_shows_tol_and_corner_only(self):
        dlg = self._dlg()
        dlg._mode_combo.setCurrentIndex(1)            # confirm
        dlg._refresh_param_visibility()
        self.assertTrue(dlg._param_rows["tol_um"].isVisibleTo(dlg))
        self.assertTrue(dlg._param_rows["corner_angle"].isVisibleTo(dlg))
        self.assertFalse(dlg._param_rows["kp"].isVisibleTo(dlg))
        self.assertFalse(dlg._param_rows["lookahead"].isVisibleTo(dlg))
        dlg.close()

    def test_open_loop_shows_pace(self):
        dlg = self._dlg()
        dlg._mode_combo.setCurrentIndex(0)            # open_loop
        dlg._refresh_param_visibility()
        self.assertTrue(dlg._param_rows["pace"].isVisibleTo(dlg))
        self.assertFalse(dlg._param_rows["kp"].isVisibleTo(dlg))
        self.assertFalse(dlg._param_rows["tol_um"].isVisibleTo(dlg))
        dlg.close()


# ── 3. Coordinate-descent auto-tune (synchronous, teleport stage) ────

class TestCoordinateDescent(unittest.TestCase):
    """Exercise the coordinate-descent tuner instantly by stubbing the physical
    drive with a synthetic error surface (no real-time hardware loop). The error
    is minimised at lookahead=0.5 and corner_factor=0.4, and grows with Kp/Kd —
    so the tuner must converge to and PERSIST those per-param bests."""

    def test_tune_converges_and_persists_bests(self):
        store = _fresh_store(self)
        from gui.dialogs.xy_challenge_dialog import XYChallengeDialog, _Bridge
        import threading
        dlg = XYChallengeDialog.__new__(XYChallengeDialog)
        dlg._ctrl = SimpleNamespace(is_xy_connected=True, is_zp_connected=False)
        dlg._safe_z = None
        dlg._start = (10.0, 10.0)
        dlg._store = store
        dlg._stop = threading.Event()
        dlg._bridge = _Bridge()
        dlg._shape_combo = SimpleNamespace(currentData=lambda: "Square")
        dlg._size_spin = SimpleNamespace(value=lambda: 4.0)
        dlg._step_spin = SimpleNamespace(value=lambda: 0.5)
        dlg._mode_combo = SimpleNamespace(currentData=lambda: "velocity",
                                          currentText=lambda: "Velocity")
        dlg._speed_spin = SimpleNamespace(value=lambda: 5.0)
        dlg._res_spin = SimpleNamespace(value=lambda: 30.0)
        dlg._look_spin = SimpleNamespace(value=lambda: 0.6)
        dlg._hz_spin = SimpleNamespace(value=lambda: 25.0)
        dlg._decel_spin = SimpleNamespace(value=lambda: 1.0)
        dlg._corner_angle_spin = SimpleNamespace(value=lambda: 30.0)
        dlg._corner_fac_spin = SimpleNamespace(value=lambda: 0.5)
        dlg._kp_spin = SimpleNamespace(value=lambda: 2.0)
        dlg._kd_spin = SimpleNamespace(value=lambda: 0.05)
        dlg._pace_spin = SimpleNamespace(value=lambda: 1.0)
        dlg._tol_spin = SimpleNamespace(value=lambda: 40.0)

        # synthetic error: minimum at lookahead=0.5, corner_factor=0.4, kp=kd=0.
        def fake_drive(mode, ideal, p):
            err_mm = (abs(p["lookahead"] - 0.5) + abs(p["corner_factor"] - 0.4)
                      + p["kp"] * 0.01 + p["kd"] * 0.1)
            return [(x, y + err_mm) for (x, y) in ideal]
        dlg._drive = fake_drive

        dlg._worker_tune()          # instant (no hardware loop)

        v = store.get_mode_params("velocity")
        self.assertAlmostEqual(v["lookahead_mm"], 0.5)
        self.assertAlmostEqual(v["corner_speed_factor"], 0.4)
        self.assertEqual(v["pid_kp"], 0.0)
        self.assertEqual(v["pid_kd"], 0.0)


# ── 4. Follower consumes measured calibration + corner slowdown ──────

class _VelStage:
    def set_acceleration(self, a): pass
    def set_speed_mm_s(self, v): pass


class _VelCtrl:
    def __init__(self):
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.safety_limits = SimpleNamespace(max_xy_speed=50000.0)
        self.xy_stage = _VelStage()
        self.zp_stage = SimpleNamespace(axis_map={"P1": "Y"})
        self._x = self._y = 0.0
        self._vx = self._vy = 0.0
        self._last = time.monotonic()
        self.vs = []                       # commanded VS magnitudes (µm/s)

    def _integrate(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        self._x += self._vx * dt
        self._y += self._vy * dt

    def send_velocity_xy(self, vx, vy):
        self._integrate()
        self._vx, self._vy = vx, vy
        self.vs.append(math.hypot(vx, vy))

    def get_xy_position(self, cached=False):
        self._integrate()
        return (self._x, self._y, 0.0)

    def move_xy_absolute(self, x, y, from_zero_ref=True):
        self._integrate()
        self._x, self._y = x * 1000.0, y * 1000.0
        self._vx = self._vy = 0.0

    def move_pump_uL(self, *a, **k):
        pass


def _run_vel(ctrl, pts, settings):
    pm = PrintManager(ctrl)
    pm.job = SimpleNamespace(settings=settings)
    pm.exec_logger = None
    pm._print_pump_suckback = lambda *a, **k: None
    pm._zp_connected_at_start = True
    cmd = PrintCommand(type=CommandType.PRINT_PATH, params={
        "points": pts, "pump": "P1", "flow_rate_uL_s": 0.4, "flow_rate": 0.01})
    pm._execute_print_path_velocity(cmd)
    return pm


class TestFollowerConsumesCalibration(unittest.TestCase):
    def test_dead_time_speed_cap_limits_commanded_velocity(self):
        # control_loop_ms 200 ms + lookahead 0.6 → cap ≈ 0.6/(0.2·2)=1.5 mm/s
        ctrl = _VelCtrl()
        pts = [(0.0, 0.0), (4.0, 0.0)]
        settings = PrintSettings(
            print_speed_mm_s=20.0, velocity_follow=True,
            vel_lookahead_mm=0.6, control_loop_ms=200.0,
            xy_max_speed_um_s=40000.0)
        _run_vel(ctrl, pts, settings)
        cruise = [v for v in ctrl.vs if v > 1.0]
        self.assertTrue(cruise)
        # commanded velocity capped near 1.5 mm/s (1500 µm/s), well below 20 mm/s
        self.assertLess(max(cruise), 3000.0)

    def test_corner_scheduling_slows_into_corner(self):
        ctrl = _VelCtrl()
        # long +x, sharp 90° corner, long +y — fine-sampled
        pts = ([(i * 0.25, 0.0) for i in range(0, 21)]          # 0→5 mm
               + [(5.0, j * 0.25) for j in range(1, 21)])       # up 0→5 mm
        settings = PrintSettings(
            print_speed_mm_s=8.0, velocity_follow=True,
            vel_lookahead_mm=0.6, vel_corner_angle_deg=30.0,
            vel_corner_speed_factor=0.25)
        _run_vel(ctrl, pts, settings)
        # the follower reached the end (corner didn't stall it)
        self.assertLess(math.hypot(ctrl._x / 1000.0 - 5.0,
                                   ctrl._y / 1000.0 - 5.0), 0.5)


if __name__ == "__main__":
    unittest.main()
