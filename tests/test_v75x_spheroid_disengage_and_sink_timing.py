"""
v7.5.x — Spheroid Pick & Place: disengagement flow + sink-timing calibration.

Covers:
  * SpheroidSinkCalibrationStore + SinkCurve: round-trip, monotone/clamped
    inversion, effective rate, is_calibrated.
  * PickPlaceExecutor._planned_aspirate_uL: sink-timing volume selection with
    the capture floor + remaining sink wait; disabled/uncalibrated fall-through.
  * _estimate_travel_time_s geometry + divide-by-zero safety.
  * _execute_spheroid_pickup sequence with a recording fake controller:
    disengage as the fast leading portion; release vs balanced dispense; the
    "Sink to tip" dwell fires only when a wait is due.
  * Offscreen calibration dialog: preflight gating, synchronous staircase drive
    (fake clock) → samples → Save writes the store; copy-trial write-back.

These drive the REAL executor / store — no hardware. The dialog test uses an
offscreen Qt platform + a lightweight fake page.
"""

import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PickAndPlaceManager import (
    OperationQueue, OperationType, PickPlaceExecutor, PickPlaceOperation,
    PickPlaceTarget, SpheroidPickupConfig,
)
from SupportClasses.SpheroidSinkCalibrationStore import (
    SpheroidSinkCalibrationStore, SinkCurve,
)


# ── Store + curve ──────────────────────────────────────────────────

class TestSinkCurve(unittest.TestCase):
    def test_monotone_interp_and_inverse(self):
        # constant sink velocity 0.5 mm/s → lift = 0.5 * t
        c = SinkCurve([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]])
        self.assertTrue(c.is_valid)
        self.assertEqual(c.n_points, 3)
        self.assertAlmostEqual(c.time_for_lift(2.0), 4.0)
        self.assertAlmostEqual(c.lift_for_time(4.0), 2.0)
        # interpolation between samples
        self.assertAlmostEqual(c.time_for_lift(1.5), 3.0)
        self.assertAlmostEqual(c.lift_for_time(3.0), 1.5)
        self.assertAlmostEqual(c.effective_rate_mm_s(), 0.5, places=3)

    def test_clamped_outside_range(self):
        c = SinkCurve([[1.0, 2.0], [2.0, 4.0]])
        # below the smallest sample → toward origin (0,0) prepended
        self.assertAlmostEqual(c.lift_for_time(0.0), 0.0)
        self.assertAlmostEqual(c.time_for_lift(0.0), 0.0)
        # above the largest → clamp to the max measured lift (never extrapolate)
        self.assertAlmostEqual(c.lift_for_time(1000.0), 2.0)
        self.assertAlmostEqual(c.time_for_lift(1000.0), 4.0)

    def test_noisy_time_made_monotone_for_inverse(self):
        # a non-monotone time sample must not break the inverse
        c = SinkCurve([[1.0, 5.0], [2.0, 3.0], [3.0, 8.0]])
        # lift_for_time must be non-decreasing in t
        prev = -1.0
        for t in (0.0, 1.0, 4.0, 6.0, 20.0):
            lift = c.lift_for_time(t)
            self.assertGreaterEqual(lift + 1e-9, prev)
            prev = lift

    def test_empty_curve_invalid(self):
        self.assertFalse(SinkCurve([]).is_valid)
        self.assertFalse(SinkCurve([[0.0, 0.0], [-1.0, 2.0]]).is_valid)


class TestStore(unittest.TestCase):
    def _store(self):
        d = tempfile.mkdtemp()
        return SpheroidSinkCalibrationStore(path=os.path.join(d, "sink.json"))

    def test_round_trip(self):
        st = self._store()
        self.assertFalse(st.is_calibrated())
        self.assertIsNone(st.get_curve())
        st.set_curve([[1.0, 2.0], [2.0, 4.0]],
                     bore_area_mm2=0.073, needle_gauge=24,
                     needle_id_um=305.0, spheroid_diameter_um=200.0)
        self.assertTrue(st.is_calibrated())
        # reload from disk
        st2 = SpheroidSinkCalibrationStore(path=st._path)
        curve = st2.get_curve()
        self.assertIsNotNone(curve)
        self.assertEqual(curve.n_points, 2)
        self.assertAlmostEqual(curve.time_for_lift(2.0), 4.0)
        meta = st2.get_meta()
        self.assertAlmostEqual(meta["bore_area_mm2"], 0.073)
        self.assertEqual(meta["needle_gauge"], 24)

    def test_clear(self):
        st = self._store()
        st.set_curve([[1.0, 2.0]])
        self.assertTrue(st.is_calibrated())
        st.clear()
        self.assertFalse(st.is_calibrated())


# ── Recording fake controller ──────────────────────────────────────

class _RecCtrl:
    def __init__(self):
        self.calls = []
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    def safe_travel_to(self, **kw):
        self.calls.append(("safe_travel_to", kw))
        return True

    def move_xy_absolute_um(self, x_um, y_um, fast=False):
        self.calls.append(("move_xy_absolute_um", x_um, y_um))

    def move_z_relative(self, dz):
        self.calls.append(("move_z_relative", dz))

    def move_z_absolute(self, z, from_zero_ref=False, feedrate_mm_min=None):
        self.calls.append(("move_z_absolute", z))

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def wait_for_z_arrival(self, *a, **k):
        return True

    def ensure_retracted_to(self, safe_z, *a, **k):
        self.calls.append(("ensure_retracted_to", safe_z))
        return True

    def get_xy_position(self, cached=True):
        return (1000.0, 2000.0)

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
        # REAL signature has settle/relieve — _settled_pump_move falls back here.
        self.calls.append(("move_pump_uL", pump, volume_uL, rate_uL_s))

    @property
    def pumps(self):
        return {}


def _op(cfg, well_a="", well_b="", src_xy=(0.0, 0.0), dst_xy=(10000.0, 0.0)):
    src = PickPlaceTarget(target_id="P1", x_um=src_xy[0], y_um=src_xy[1],
                          well_name=well_a)
    dst = PickPlaceTarget(target_id="D1", x_um=dst_xy[0], y_um=dst_xy[1],
                          well_name=well_b)
    return PickPlaceOperation(op_id="OP1", op_type=OperationType.SPHEROID_PICKUP,
                              source_target=src, dest_target=dst, config=cfg)


def _executor(sink_curve=None, bore_area=0.5):
    ctrl = _RecCtrl()
    ex = PickPlaceExecutor(ctrl, hw_config=None)
    ex.safe_z_mm = -35.0
    ex.pick_z_mm = -16.0
    ex.place_z_mm = -14.0
    ex.sink_curve = sink_curve
    ex.bore_area_mm2 = bore_area
    return ex, ctrl


# ── Planned aspirate / travel time ─────────────────────────────────

class TestPlannedAspirate(unittest.TestCase):
    def test_disabled_returns_carrier(self):
        ex, _ = _executor(sink_curve=SinkCurve([[1.0, 2.0], [2.0, 4.0]]))
        cfg = SpheroidPickupConfig(sink_timing_enabled=False)
        op = _op(cfg)
        total, wait = ex._planned_aspirate_uL(cfg, op.source_target, op.dest_target)
        self.assertAlmostEqual(total, cfg.compute_volume_uL())
        self.assertEqual(wait, 0.0)

    def test_uncalibrated_returns_carrier(self):
        ex, _ = _executor(sink_curve=None)
        cfg = SpheroidPickupConfig(sink_timing_enabled=True)
        op = _op(cfg)
        total, wait = ex._planned_aspirate_uL(cfg, op.source_target, op.dest_target)
        self.assertAlmostEqual(total, cfg.compute_volume_uL())
        self.assertEqual(wait, 0.0)

    def test_sink_timing_volume_and_remaining(self):
        # rate 0.5 mm/s; bore area 0.5 mm²
        ex, _ = _executor(sink_curve=SinkCurve([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]]),
                          bore_area=0.5)
        ex._estimate_travel_time_s = lambda a, b: 3.0
        cfg = SpheroidPickupConfig(sink_timing_enabled=True, travel_margin_s=1.0)
        op = _op(cfg)
        total, wait = ex._planned_aspirate_uL(cfg, op.source_target, op.dest_target)
        # target_sink = 3+1 = 4 → lift 2 mm → vol = 2*0.5 = 1.0 µL
        self.assertAlmostEqual(total, 1.0, places=4)
        # total sink for lift(1.0/0.5=2mm) = 4 s; remaining = 4-3 = 1
        self.assertAlmostEqual(wait, 1.0, places=4)

    def test_capture_floor_on_short_travel(self):
        ex, _ = _executor(sink_curve=SinkCurve([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]]),
                          bore_area=0.5)
        ex._estimate_travel_time_s = lambda a, b: 0.001
        cfg = SpheroidPickupConfig(sink_timing_enabled=True, travel_margin_s=0.0)
        op = _op(cfg)
        total, wait = ex._planned_aspirate_uL(cfg, op.source_target, op.dest_target)
        # timing volume is tiny → floored at the sphere-capture volume
        self.assertAlmostEqual(total, cfg.compute_volume_uL(), places=6)
        self.assertGreaterEqual(wait, 0.0)

    def test_zero_bore_area_returns_carrier(self):
        ex, _ = _executor(sink_curve=SinkCurve([[1.0, 2.0]]), bore_area=0.0)
        cfg = SpheroidPickupConfig(sink_timing_enabled=True)
        op = _op(cfg)
        total, wait = ex._planned_aspirate_uL(cfg, op.source_target, op.dest_target)
        self.assertAlmostEqual(total, cfg.compute_volume_uL())
        self.assertEqual(wait, 0.0)


class TestTravelTime(unittest.TestCase):
    def test_geometry_and_farther_is_slower(self):
        ex, _ = _executor()
        ex._xy_travel_speed_um_s = lambda: 10000.0   # 10 mm/s
        ex._z_travel_speed_mm_s = lambda: 10.0
        ex._travel_overhead_s = 0.0
        near = _op(SpheroidPickupConfig(), dst_xy=(1000.0, 0.0))
        far = _op(SpheroidPickupConfig(), dst_xy=(50000.0, 0.0))
        t_near = ex._estimate_travel_time_s(near.source_target, near.dest_target)
        t_far = ex._estimate_travel_time_s(far.source_target, far.dest_target)
        self.assertGreater(t_far, t_near)
        # z: |(-35)-(-16)| + |(-35)-(-14)| = 19 + 21 = 40 mm / 10 = 4.0 s
        # near xy: 1000 µm / 10000 µm/s = 0.1 s → total 4.1
        self.assertAlmostEqual(t_near, 4.1, places=3)

    def test_zero_distance_no_div_by_zero(self):
        ex, _ = _executor()
        ex._xy_travel_speed_um_s = lambda: 10000.0
        ex._z_travel_speed_mm_s = lambda: 10.0
        ex._travel_overhead_s = 1.0
        op = _op(SpheroidPickupConfig(), src_xy=(5.0, 5.0), dst_xy=(5.0, 5.0))
        t = ex._estimate_travel_time_s(op.source_target, op.dest_target)
        self.assertAlmostEqual(t, 4.0 + 1.0, places=3)


# ── Execution sequence ─────────────────────────────────────────────

class TestSpheroidSequence(unittest.TestCase):
    def _run(self, cfg, sink_curve=None, bore_area=0.5):
        ex, ctrl = _executor(sink_curve=sink_curve, bore_area=bore_area)
        dwells = []
        ex._dwell = lambda op, dur, label: dwells.append((label, dur))
        q = OperationQueue()
        q.add(_op(cfg))
        ok = ex.execute_queue(q)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        return ok, pumps, dwells

    def test_no_new_features_matches_legacy(self):
        cfg = SpheroidPickupConfig(spheroid_diameter_um=200.0, safety_factor=1.5,
                                   pickup_speed_uL_s=1.0, release_speed_uL_s=2.0)
        ok, pumps, dwells = self._run(cfg)
        self.assertTrue(ok)
        self.assertEqual(len(pumps), 2)  # aspirate + dispense
        carrier = cfg.compute_volume_uL()
        self.assertAlmostEqual(pumps[0][2], -carrier)   # aspirate
        self.assertEqual(pumps[0][3], 1.0)
        self.assertAlmostEqual(pumps[1][2], carrier)    # full dispense
        self.assertEqual(pumps[1][3], 2.0)
        self.assertFalse(any(l == "Sink to tip" for l, _ in dwells))

    def test_disengage_is_fast_leading_portion(self):
        cfg = SpheroidPickupConfig(
            spheroid_diameter_um=200.0, safety_factor=1.5,
            pickup_speed_uL_s=1.0, release_speed_uL_s=2.0,
            disengage_enabled=True, disengage_volume_uL=0.5, disengage_rate_uL_s=5.0)
        ok, pumps, _ = self._run(cfg)
        self.assertTrue(ok)
        # disengage (0.5 µL @ 5 µL/s), the carrier is tiny so the disengage IS the
        # whole aspirate → one leading aspirate then the dispense (no remainder).
        aspirates = [p for p in pumps if p[2] < 0]
        self.assertGreaterEqual(len(aspirates), 1)
        self.assertAlmostEqual(aspirates[0][2], -0.5)   # leading disengage volume
        self.assertEqual(aspirates[0][3], 5.0)          # at the disengage rate

    def test_disengage_leading_then_remainder(self):
        # planned total forced large via sink timing so a remainder aspirate exists
        cfg = SpheroidPickupConfig(
            spheroid_diameter_um=200.0, safety_factor=1.5,
            pickup_speed_uL_s=1.0, release_speed_uL_s=2.0, travel_margin_s=1.0,
            sink_timing_enabled=True,
            disengage_enabled=True, disengage_volume_uL=0.3, disengage_rate_uL_s=5.0)
        curve = SinkCurve([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]])
        ex, ctrl = _executor(sink_curve=curve, bore_area=0.5)
        ex._estimate_travel_time_s = lambda a, b: 3.0
        ex._dwell = lambda op, dur, label: None
        q = OperationQueue(); q.add(_op(cfg))
        self.assertTrue(ex.execute_queue(q))
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        aspirates = [p for p in pumps if p[2] < 0]
        # total = 1.0 µL; leading 0.3 @ 5 µL/s, remainder 0.7 @ 1 µL/s
        self.assertEqual(len(aspirates), 2)
        self.assertAlmostEqual(aspirates[0][2], -0.3)
        self.assertEqual(aspirates[0][3], 5.0)
        self.assertAlmostEqual(aspirates[1][2], -0.7, places=4)
        self.assertEqual(aspirates[1][3], 1.0)

    def test_release_mode_dispenses_small_volume(self):
        cfg = SpheroidPickupConfig(
            spheroid_diameter_um=200.0, safety_factor=1.5,
            release_speed_uL_s=2.0, release_enabled=True, release_volume_uL=0.05)
        ok, pumps, _ = self._run(cfg)
        self.assertTrue(ok)
        dispenses = [p for p in pumps if p[2] > 0]
        self.assertEqual(len(dispenses), 1)
        self.assertAlmostEqual(dispenses[0][2], 0.05)   # small release, not carrier
        self.assertEqual(dispenses[0][3], 2.0)

    def test_sink_dwell_fires_when_wait_due(self):
        cfg = SpheroidPickupConfig(sink_timing_enabled=True, travel_margin_s=1.0,
                                   release_enabled=True, release_volume_uL=0.05)
        curve = SinkCurve([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]])
        ex, ctrl = _executor(sink_curve=curve, bore_area=0.5)
        ex._estimate_travel_time_s = lambda a, b: 3.0
        dwells = []
        ex._dwell = lambda op, dur, label: dwells.append((label, dur))
        q = OperationQueue(); q.add(_op(cfg))
        self.assertTrue(ex.execute_queue(q))
        sink = [d for d in dwells if d[0] == "Sink to tip"]
        self.assertEqual(len(sink), 1)
        self.assertAlmostEqual(sink[0][1], 1.0, places=3)


# ── Offscreen calibration dialog ───────────────────────────────────

class TestCalibrationDialog(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from PySide6.QtWidgets import QApplication
        except Exception as e:  # pragma: no cover
            raise unittest.SkipTest(f"Qt unavailable: {e}")
        cls.app = QApplication.instance() or QApplication([])

    def _fake_page(self, tmp_path):
        from PySide6.QtWidgets import QDoubleSpinBox, QComboBox, QCheckBox
        os.environ["MEBP_SPHEROID_SINK_CAL_PATH"] = tmp_path
        # reset the store singleton so it picks up the temp path
        import SupportClasses.SpheroidSinkCalibrationStore as m
        m._store = None

        class _FakeNeedle:
            gauge = 24
            id_um = 305.0
            cross_section_area_mm2 = 0.073

        class _FakeHW:
            needle = _FakeNeedle()
            pumps = {}

        ctrl = _RecCtrl()

        class _FakePage:
            def __init__(self):
                self._controller = ctrl
                self._camera_manager = None
                self._hw_config = _FakeHW()
                self._safe_z = -35.0
                self._pick_z = QDoubleSpinBox(); self._pick_z.setRange(0, 20); self._pick_z.setValue(0.1)
                self._pick_flow = QDoubleSpinBox(); self._pick_flow.setValue(1.0)
                self._diameter = QDoubleSpinBox(); self._diameter.setRange(1, 5000); self._diameter.setValue(200.0)
                self._bore = QComboBox(); self._bore.addItem("P1")
                self._disengage_vol = QDoubleSpinBox(); self._disengage_vol.setRange(0, 200)
                self._disengage_rate = QDoubleSpinBox(); self._disengage_rate.setRange(0.01, 50); self._disengage_rate.setValue(2.0)
                self._disengage_enabled = QCheckBox()
                self.settings_changed = 0
                self.sink_refreshed = 0

            def _plate_offset_to_zref(self, mm):
                return -16.0 + (-1.0) * float(mm)

            def _bore_area_mm2(self):
                return 0.073

            def _current_config(self):
                return SpheroidPickupConfig(spheroid_diameter_um=200.0, safety_factor=1.5)

            def _on_settings_changed(self):
                self.settings_changed += 1

            def _refresh_sink_status(self):
                self.sink_refreshed += 1

        return _FakePage(), ctrl

    def _dialog(self, page):
        from gui.pages.workflows.spheroid_sink_calibration import (
            SinkDisengageCalibrationDialog)
        dlg = SinkDisengageCalibrationDialog(page)
        # run workers synchronously so the staircase drives on the test thread
        dlg._launch = lambda target, msg="": target()
        return dlg

    def test_preflight_gates(self):
        d = tempfile.mkdtemp()
        page, ctrl = self._fake_page(os.path.join(d, "s.json"))
        dlg = self._dialog(page)
        # ZP disconnected
        ctrl.is_zp_connected = False
        self.assertIsNone(dlg._preflight())
        ctrl.is_zp_connected = True
        # no safe Z
        page._safe_z = None
        self.assertIsNone(dlg._preflight())
        page._safe_z = -35.0
        # ok
        self.assertIsNotNone(dlg._preflight())

    def test_staircase_records_and_saves(self):
        import gui.pages.workflows.spheroid_sink_calibration as mod
        d = tempfile.mkdtemp()
        page, ctrl = self._fake_page(os.path.join(d, "s.json"))
        dlg = self._dialog(page)
        # fake monotonic clock
        clock = [0.0]
        mod.time.monotonic = lambda: clock[0]
        # configure a 3-step staircase (start 1 µL, step 1 µL) on a 0.073 mm² bore
        dlg._start_vol_spin.setValue(1.0)
        dlg._step_vol_spin.setValue(1.0)
        dlg._steps_spin.setValue(3)
        dlg._pull_rate_spin.setValue(1.0)
        dlg._on_start_staircase()   # runs stair_worker(0) synchronously → armed
        self.assertTrue(dlg._timing_active)
        # step 1: 2 s to sink
        clock[0] = 2.0
        dlg._on_visible()
        # step 2: 4 s
        clock[0] = 2.0 + 4.0
        dlg._on_visible()
        # step 3: 6 s
        clock[0] = 6.0 + 6.0
        dlg._on_visible()   # last click → finish worker runs, dumps, finished
        self.assertEqual(len(dlg._samples), 3)
        # lifts = ΔV/area = 1/0.073, 2/0.073, 3/0.073
        self.assertAlmostEqual(dlg._samples[0][0], 1.0 / 0.073, places=3)
        self.assertAlmostEqual(dlg._samples[0][1], 2.0, places=3)
        self.assertAlmostEqual(dlg._samples[2][1], 6.0, places=3)
        # cumulative fluid dumped back (1+2+3=6 µL dispensed) → net zero
        pump_sum = sum(c[2] for c in ctrl.calls if c[0] == "move_pump_uL")
        self.assertAlmostEqual(pump_sum, 0.0, places=4)
        # save writes the store
        dlg._on_save_curve()
        curve = mod.get_store().get_curve()
        self.assertIsNotNone(curve)
        self.assertEqual(curve.n_points, 3)
        self.assertGreaterEqual(page.sink_refreshed, 1)

    def test_copy_trial_writes_settings(self):
        d = tempfile.mkdtemp()
        page, _ = self._fake_page(os.path.join(d, "s.json"))
        dlg = self._dialog(page)
        dlg._trial_vol_spin.setValue(0.4)
        dlg._trial_rate_spin.setValue(6.0)
        dlg._on_copy_trial()
        self.assertAlmostEqual(page._disengage_vol.value(), 0.4, places=3)
        self.assertAlmostEqual(page._disengage_rate.value(), 6.0, places=3)
        self.assertTrue(page._disengage_enabled.isChecked())
        self.assertGreaterEqual(page.settings_changed, 1)


if __name__ == "__main__":
    unittest.main()
