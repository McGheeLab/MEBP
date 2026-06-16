"""
v7.5.x — Z-bottom auto-cal: manual first-spot seed + polarity-general sweep.

The operator calibrates the first calibration well's bottom by hand (jog Z while
watching the live microscope feed), and that taught Z seeds the focus search for
the remaining wells. The sweep is parametrized by a *height above the reference
bottom* ``h`` (+ = away from plate / safe) so "toward the plate" (decreasing h)
is physically correct on both a conventional machine (ZDIR=+1) and ME3B V1
(ZDIR=-1, where the needle descends as zero-ref Z increases).

These tests call the real (unbound) CalibrationPage methods with duck-typed
SimpleNamespace stubs so no Qt widget / camera / event loop is needed (same
pattern as test_v75x_plate_location_manual_click_rim.py). QMessageBox is patched
so dialog paths don't require a QApplication, and QTimer is patched so
_start_auto_z_cal doesn't spin up a real timer.
"""

import types
import unittest
from types import SimpleNamespace

from gui.pages.calibration import CalibrationPage
import gui.pages.calibration as calmod
from SupportClasses.StageController import plate_relative_to_zref, ZDIR


class _FakeMB:
    Yes = 1
    No = 0
    warned = 0

    @classmethod
    def reset(cls):
        cls.warned = 0

    @classmethod
    def warning(cls, *a, **k):
        cls.warned += 1
        return cls.No

    @classmethod
    def question(cls, *a, **k):
        return cls.Yes

    @classmethod
    def information(cls, *a, **k):
        return cls.Yes


class _RecCtrl:
    """Records move_z_absolute calls."""
    def __init__(self):
        self.z_moves = []
        self.is_zp_connected = True

    def move_z_absolute(self, z, from_zero_ref=True, **k):
        self.z_moves.append((z, from_zero_ref))


class _FakeLabel:
    def __init__(self):
        self.text = ""
        self.style = ""

    def setText(self, t):
        self.text = t

    def setStyleSheet(self, s):
        self.style = s


class _FakeButton:
    def __init__(self):
        self.enabled = None

    def setEnabled(self, v):
        self.enabled = v


class _FakeTimer:
    def __init__(self, *a, **k):
        self.timeout = SimpleNamespace(connect=lambda *a, **k: None)

    def setSingleShot(self, *a):
        pass

    def start(self, *a):
        pass

    def stop(self):
        pass


class TestMoveToHeightPolarity(unittest.TestCase):
    """_auto_z_move_to_h converts height-above-bottom → zero-ref Z and clamps."""

    def _stub(self, ref, floor_h):
        return SimpleNamespace(
            _auto_z_ref_z=ref,
            _auto_z_floor_h=floor_h,
            _auto_z_h=None,
            _auto_z_current_z=None,
            controller=_RecCtrl(),
        )

    def test_height_maps_via_plate_relative(self):
        s = self._stub(ref=10.0, floor_h=-0.5)
        out = CalibrationPage._auto_z_move_to_h(s, 1.0)
        self.assertAlmostEqual(out, 1.0)
        self.assertAlmostEqual(s._auto_z_h, 1.0)
        expected_z = plate_relative_to_zref(10.0, 1.0)
        self.assertAlmostEqual(s._auto_z_current_z, expected_z)
        self.assertEqual(len(s.controller.z_moves), 1)
        self.assertAlmostEqual(s.controller.z_moves[0][0], expected_z)
        self.assertTrue(s.controller.z_moves[0][1])  # from_zero_ref

    def test_toward_plate_is_polarity_correct(self):
        # Decreasing h = toward the plate. On ME3B (ZDIR=-1) that must
        # *increase* zero-ref Z (needle descends as Z grows).
        s = self._stub(ref=10.0, floor_h=-1.0)
        CalibrationPage._auto_z_move_to_h(s, 1.0)
        z_high = s._auto_z_current_z
        CalibrationPage._auto_z_move_to_h(s, 0.0)
        z_low = s._auto_z_current_z
        if ZDIR < 0:
            self.assertGreater(z_low, z_high)  # toward plate ⇒ larger Z
        else:
            self.assertLess(z_low, z_high)

    def test_clamps_to_floor(self):
        s = self._stub(ref=10.0, floor_h=-0.5)
        out = CalibrationPage._auto_z_move_to_h(s, -2.0)
        self.assertAlmostEqual(out, -0.5)  # clamped
        self.assertAlmostEqual(s._auto_z_h, -0.5)
        self.assertAlmostEqual(s._auto_z_current_z,
                               plate_relative_to_zref(10.0, -0.5))


class TestNavigatePhaseSeed(unittest.TestCase):
    """The navigate phase chooses the reference bottom + search window from
    the manual seed (else the legacy estimate)."""

    def _stub(self, seed):
        moves = []
        return SimpleNamespace(
            _auto_z_scanning=True,
            _auto_z_phase="navigate",
            _auto_z_well_idx=0,
            _auto_z_wells=["A12"],
            _calibrated_positions={"A12": (5000.0, 6000.0)},
            _predicted_positions=None,
            _zauto_seed_z=seed,
            _zauto_approach_margin=1.0,
            _zauto_tilt_margin=0.5,
            _top_z=2.0,
            _auto_z_well_depth=17.4,
            controller=_RecCtrl(),
            _auto_z_timer=_FakeTimer(),
            # side-effect methods stubbed:
            _safe_navigate_to=lambda *a, **k: None,
            _auto_z_set_progress=lambda *a, **k: None,
            _auto_z_capture_baseline=lambda *a, **k: None,
            _auto_z_move_to_h=lambda h: moves.append(h),
            _move_log=moves,
        )

    def test_seeded_window(self):
        s = self._stub(seed=12.0)
        CalibrationPage._auto_z_tick(s)
        self.assertEqual(s._auto_z_ref_z, 12.0)
        self.assertAlmostEqual(s._auto_z_floor_h, -0.5)
        # First fast move is to the approach margin above the bottom.
        self.assertEqual(s._move_log, [1.0])
        self.assertEqual(s._auto_z_phase, "coarse")

    def test_legacy_estimate_when_no_seed(self):
        s = self._stub(seed=None)
        CalibrationPage._auto_z_tick(s)
        self.assertAlmostEqual(s._auto_z_ref_z, 2.0 - 17.4)  # top_z - well_depth
        self.assertAlmostEqual(s._auto_z_floor_h, 0.0)
        self.assertEqual(s._move_log, [2.0])  # legacy 2 mm approach margin


class TestStartAutoZSeededSkipsFirst(unittest.TestCase):
    """When a seed is present, _start_auto_z_cal pre-seeds the first well's
    result and starts scanning at the next well."""

    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB
        self._orig_vision = calmod.VISION_AVAILABLE
        calmod.VISION_AVAILABLE = False
        import PySide6.QtCore as _qtc
        self._qtc = _qtc
        self._orig_timer = _qtc.QTimer
        _qtc.QTimer = _FakeTimer

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb
        calmod.VISION_AVAILABLE = self._orig_vision
        self._qtc.QTimer = self._orig_timer

    def _stub(self, seed, first="A1"):
        plate = SimpleNamespace(well_depth_mm=17.4)
        return SimpleNamespace(
            _plate=plate,
            _safe_z=5.0,
            _top_z=2.0,
            _calibrated_positions={"A1": (0, 0), "A12": (1, 1), "H12": (2, 2)},
            _predicted_positions=None,
            _zoff_ensure_live_camera=lambda: None,
            _get_primary_camera=lambda: SimpleNamespace(is_running=True),
            _get_calibration_wells=lambda: ["A1", "A12", "H12"],
            _zauto_seed_z=seed,
            _zauto_first_well=first,
            _auto_z_set_running=lambda v: None,
            _auto_z_set_progress=lambda *a, **k: None,
            _auto_z_results=None,
            _auto_z_well_idx=None,
            _auto_z_scanning=False,
            _auto_z_phase="idle",
            _auto_z_wells=None,
            _auto_z_well_depth=None,
            _auto_z_timer=None,
            _auto_z_tick=lambda: None,
        )

    def test_seeded_skips_first_well(self):
        s = self._stub(seed=13.5, first="A1")
        CalibrationPage._start_auto_z_cal(s)
        self.assertEqual(s._auto_z_results, {"A1": 13.5})
        self.assertEqual(s._auto_z_well_idx, 1)
        self.assertTrue(s._auto_z_scanning)
        self.assertEqual(s._auto_z_phase, "navigate")
        self.assertEqual(_FakeMB.warned, 0)

    def test_legacy_scans_all_wells(self):
        s = self._stub(seed=None)
        CalibrationPage._start_auto_z_cal(s)
        self.assertEqual(s._auto_z_results, {})
        self.assertEqual(s._auto_z_well_idx, 0)

    def test_missing_xy_map_warns_and_aborts(self):
        s = self._stub(seed=13.5)
        s._calibrated_positions = {}
        s._predicted_positions = None
        CalibrationPage._start_auto_z_cal(s)
        self.assertGreaterEqual(_FakeMB.warned, 1)
        self.assertFalse(s._auto_z_scanning)


class TestRecordFirstSpot(unittest.TestCase):
    """Recording the first spot sets the seed + result + Z-plane teach point."""

    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb

    def _stub(self, z):
        return SimpleNamespace(
            _get_calibration_wells=lambda: ["A1", "A12", "H12"],
            _zauto_first_well=None,
            _zoff_capture_current_z=lambda: z,
            _zauto_seed_z=None,
            _auto_z_results={},
            _z_teach_points={},
            _zauto_lbl_first=_FakeLabel(),
            _emit_calibration_data_changed=lambda: None,
        )

    def test_records_seed_and_teach_point(self):
        s = self._stub(z=14.25)
        CalibrationPage._zauto_record_first_spot(s)
        self.assertAlmostEqual(s._zauto_seed_z, 14.25)
        self.assertEqual(s._zauto_first_well, "A1")
        self.assertAlmostEqual(s._auto_z_results["A1"], 14.25)
        self.assertAlmostEqual(s._z_teach_points["A1"], 14.25)
        self.assertIn("A1", s._zauto_lbl_first.text)
        self.assertEqual(_FakeMB.warned, 0)

    def test_no_z_reading_warns(self):
        s = self._stub(z=None)
        CalibrationPage._zauto_record_first_spot(s)
        self.assertIsNone(s._zauto_seed_z)
        self.assertGreaterEqual(_FakeMB.warned, 1)


class TestProgressHelpers(unittest.TestCase):
    """Dual-widget progress helpers write to whichever labels/buttons exist."""

    def test_set_progress_both_labels(self):
        s = SimpleNamespace(
            _lbl_auto_z_progress=_FakeLabel(),
            _zoff_lbl_auto_z=_FakeLabel(),
        )
        CalibrationPage._auto_z_set_progress(s, "hello", "#fff")
        self.assertEqual(s._lbl_auto_z_progress.text, "hello")
        self.assertEqual(s._zoff_lbl_auto_z.text, "hello")
        self.assertIn("#fff", s._zoff_lbl_auto_z.style)

    def test_set_progress_tolerates_missing(self):
        s = SimpleNamespace(_zoff_lbl_auto_z=_FakeLabel())
        # No _lbl_auto_z_progress attribute — must not raise.
        CalibrationPage._auto_z_set_progress(s, "hi")
        self.assertEqual(s._zoff_lbl_auto_z.text, "hi")

    def test_set_running_toggles_present_buttons(self):
        s = SimpleNamespace(
            _zoff_btn_run_z=_FakeButton(),
            _zoff_btn_cancel_z=_FakeButton(),
        )
        CalibrationPage._auto_z_set_running(s, True)
        self.assertFalse(s._zoff_btn_run_z.enabled)
        self.assertTrue(s._zoff_btn_cancel_z.enabled)
        CalibrationPage._auto_z_set_running(s, False)
        self.assertTrue(s._zoff_btn_run_z.enabled)
        self.assertFalse(s._zoff_btn_cancel_z.enabled)


if __name__ == "__main__":
    unittest.main()
