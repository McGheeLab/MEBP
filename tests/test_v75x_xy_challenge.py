"""test_v75x_xy_challenge.py — XY Printing Challenge (shapes, error, store,
param plumbing, and the three mode drivers).

The challenge drives a test shape (needle retracted), records the ACTUAL path
from the encoder, and compares it to the IDEAL path so the three print modes
(open-loop / confirm / velocity) can be compared + tuned. Tuned per-mode params
persist to PrintTimingCalibrationStore and are read by the real print.
"""

import math
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses import XYChallenge as XC
from SupportClasses.PrintTimingCalibrationStore import (
    PrintTimingCalibrationStore, get_store)
from SupportClasses.PrintManager import PrintSettings


# ── 1. Shapes + error metric (pure) ──────────────────────────────────

class TestShapes(unittest.TestCase):
    def test_all_shapes_build_centred(self):
        for name in XC.CHALLENGE_SHAPES:
            pts = XC.make_shape(name, 10.0, 0.5)
            self.assertGreaterEqual(len(pts), 8, name)
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            # centred on origin, within the requested size
            self.assertLessEqual(max(xs) - min(xs), 10.5, name)
            self.assertLessEqual(max(abs(min(xs)), abs(max(xs))), 6.0, name)
            self.assertLess(abs((max(xs) + min(xs)) / 2), 1.0, name)
            self.assertLess(abs((max(ys) + min(ys)) / 2), 1.0, name)

    def test_unknown_shape_raises(self):
        with self.assertRaises(ValueError):
            XC.make_shape("nope", 10, 0.5)

    def test_resample_keeps_vertices_and_endpoints(self):
        pts = XC.resample_polyline([(0, 0), (10, 0)], 1.0)
        self.assertEqual(pts[0], (0.0, 0.0))
        self.assertEqual(pts[-1], (10.0, 0.0))
        # spacing ~1mm
        self.assertGreaterEqual(len(pts), 10)

    def test_error_zero_for_perfect_follow(self):
        ideal = XC.make_shape("Circle", 10, 0.4)
        err = XC.path_error(ideal, ideal)
        self.assertLess(err["rms_um"], 1.0)
        self.assertEqual(err["n"], len(ideal))

    def test_error_scales_with_offset(self):
        ideal = XC.make_shape("Square", 10, 0.5)
        off_small = [(x, y + 0.1) for x, y in ideal]   # 100 µm
        off_big = [(x, y + 0.3) for x, y in ideal]     # 300 µm
        e1 = XC.path_error(off_small, ideal)["rms_um"]
        e2 = XC.path_error(off_big, ideal)["rms_um"]
        self.assertGreater(e1, 10.0)
        self.assertGreater(e2, e1)

    def test_offset_path(self):
        pts = XC.offset_path([(0, 0), (1, 1)], 5.0, 7.0)
        self.assertEqual(pts, [(5.0, 7.0), (6.0, 8.0)])


# ── 2. Store per-mode params ──────────────────────────────────────────

class TestStoreModeParams(unittest.TestCase):
    def _store(self):
        p = os.path.join(tempfile.mkdtemp(), "tc.json")
        return PrintTimingCalibrationStore(p), p

    def test_defaults(self):
        s, _ = self._store()
        self.assertEqual(s.get_mode_params("velocity"), {
            "lookahead_mm": 0.6, "control_hz": 25.0, "decel_mm": 1.5,
            "corner_angle_deg": 30.0, "corner_speed_factor": 0.4,
            "pid_kp": 0.0, "pid_kd": 0.0})
        self.assertEqual(s.get_mode_params("open_loop"),
                         {"pace_correction": 1.0})
        self.assertEqual(s.get_mode_params("confirm"),
                         {"settle_tol_um": 40.0, "corner_angle_deg": 30.0})

    def test_set_merges_and_persists(self):
        s, path = self._store()
        s.set_mode_params("velocity", {"lookahead_mm": 0.9})
        s2 = PrintTimingCalibrationStore(path)          # reload
        v = s2.get_mode_params("velocity")
        self.assertEqual(v["lookahead_mm"], 0.9)         # changed
        self.assertEqual(v["control_hz"], 25.0)          # default preserved

    def test_unknown_key_ignored(self):
        s, _ = self._store()
        s.set_mode_params("confirm", {"bogus": 1.0, "settle_tol_um": 25.0})
        cf = s.get_mode_params("confirm")
        self.assertNotIn("bogus", cf)
        self.assertEqual(cf["settle_tol_um"], 25.0)

    def test_unknown_mode_ignored(self):
        s, _ = self._store()
        s.set_mode_params("nope", {"x": 1})              # must not raise


# ── 3. PrintSettings plumbing + Quick Print stamping ──────────────────

class TestSettingsPlumbing(unittest.TestCase):
    def test_defaults_preserve_legacy(self):
        ps = PrintSettings()
        self.assertEqual(ps.pace_correction, 1.0)
        self.assertEqual(ps.segment_settle_tol_um, 0.0)
        self.assertEqual(ps.vel_lookahead_mm, 0.0)
        self.assertEqual(ps.vel_control_hz, 0.0)
        self.assertEqual(ps.vel_decel_mm, 0.0)

    def test_quick_print_stamps_tuned_params(self):
        # A tuned store value must flow onto PrintSettings via _build_settings.
        from SupportClasses import PrintTimingCalibrationStore as TCS
        d = tempfile.mkdtemp()
        store = TCS.PrintTimingCalibrationStore(os.path.join(d, "tc.json"))
        store.set_mode_params("velocity", {"lookahead_mm": 0.35})
        store.set_mode_params("open_loop", {"pace_correction": 2.0})
        orig = TCS._store
        TCS._store = store                               # get_store() → our store
        try:
            from gui.pages.workflows import quick_print_workflow as qpw
            page = qpw.QuickPrintWorkflowPage.__new__(qpw.QuickPrintWorkflowPage)

            class _C:
                def print_z_dir(self): return 1.0
                def plate_axis_sign(self): return (1.0, 1.0)
                def default_travel_z(self, z, margin_mm=10.0): return z + margin_mm
            page._controller = _C()
            page._safe_z = 40.0
            page._resolved_print_kinematics = lambda: (2.5, 0.4, 0.1)
            page._resolve_print_z = lambda: 21.0
            page._pump = lambda: "P1"
            page._travel_speed = None
            page._line_retract_spin = None
            page._line_z_speed_spin = None
            page._line_xy_speed_spin = None
            page._motion_mode = lambda: "velocity"
            s = page._build_settings()
            self.assertAlmostEqual(s.vel_lookahead_mm, 0.35)
            self.assertAlmostEqual(s.pace_correction, 2.0)
        finally:
            TCS._STORE = orig


# ── 4. The three mode drivers (with fake stages) ──────────────────────

class _FakeXY:
    def set_speed_mm_s(self, v): pass
    def set_acceleration(self, a): pass


class _FakeCtrl:
    """move_xy_absolute teleports (a stage that reaches its target);
    send_velocity_xy integrates over wall-time (a stage that follows VS)."""
    def __init__(self, velocity=False):
        self.is_xy_connected = True
        self.is_zp_connected = False
        self.zero_position = {"x": 0.0, "y": 0.0}
        self.safety_limits = SimpleNamespace(max_xy_speed=50000.0)
        self.xy_stage = _FakeXY()
        self._x = 0.0
        self._y = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._last = time.monotonic()
        self._vel = velocity

    def _integrate(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        self._x += self._vx * dt
        self._y += self._vy * dt

    def move_xy_absolute(self, x, y, from_zero_ref=True):
        self._x = x * 1000.0
        self._y = y * 1000.0
        self._vx = self._vy = 0.0
        self._last = time.monotonic()

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def get_xy_position(self, cached=False):
        if self._vel:
            self._integrate()
        return (self._x, self._y, 0.0)

    def send_velocity_xy(self, vx, vy):
        if self._vel:
            self._integrate()
        self._vx, self._vy = vx, vy


def _dialog(ctrl):
    from gui.dialogs.xy_challenge_dialog import XYChallengeDialog, _Bridge
    import threading
    dlg = XYChallengeDialog.__new__(XYChallengeDialog)
    dlg._ctrl = ctrl
    dlg._safe_z = None
    dlg._stop = threading.Event()
    dlg._bridge = _Bridge()
    dlg._store = get_store()
    return dlg


class TestDrivers(unittest.TestCase):
    def _ideal(self):
        return XC.offset_path(XC.make_shape("Square", 2.0, 0.5), 10.0, 10.0)

    def test_open_loop_records_and_tracks(self):
        dlg = _dialog(_FakeCtrl(velocity=False))
        ideal = self._ideal()
        p = {"speed": 20.0, "pace": 1.0, "tol_um": 40, "lookahead": 0.6,
             "control_hz": 25.0, "decel": 1.5}
        samples = dlg._drive_open_loop(ideal, p)
        self.assertTrue(samples)
        err = XC.path_error(samples, ideal)
        self.assertLess(err["rms_um"], 50.0)     # teleport stage → on the path

    def test_confirm_records_and_tracks(self):
        dlg = _dialog(_FakeCtrl(velocity=False))
        ideal = self._ideal()
        p = {"speed": 20.0, "pace": 1.0, "tol_um": 40, "lookahead": 0.6,
             "control_hz": 25.0, "decel": 1.5}
        samples = dlg._drive_confirm(ideal, p)
        self.assertTrue(samples)
        self.assertLess(XC.path_error(samples, ideal)["rms_um"], 50.0)

    def test_velocity_follows_and_stops(self):
        ctrl = _FakeCtrl(velocity=True)
        dlg = _dialog(ctrl)
        ideal = self._ideal()
        p = {"speed": 30.0, "pace": 1.0, "tol_um": 40, "lookahead": 0.5,
             "control_hz": 25.0, "decel": 1.0}
        samples = dlg._drive_velocity(ideal, p)
        self.assertTrue(samples)
        # a VS-following stage should trace the square reasonably (lookahead cuts
        # the corners a little, so allow a looser bound than the teleport modes)
        self.assertLess(XC.path_error(samples, ideal)["rms_um"], 600.0)
        self.assertEqual((ctrl._vx, ctrl._vy), (0.0, 0.0))   # stopped on exit


if __name__ == "__main__":
    unittest.main()
