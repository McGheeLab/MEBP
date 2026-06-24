"""
v7.5.x — Plate Z Auto-Cal: per-well guided refocus + needle best-focus.

Each calibration well is taught the same way: retract + travel there, the
operator refocuses the microscope on the glass (manual knob) and confirms,
then the needle lowers and the best-focus Z (sharpest needle circle) is
recorded (auto-detect peak, with a Record-now manual override). The operator
Accepts (advance + fit the plane on the last well) or Redoes the well.

These tests call the real (unbound) CalibrationPage methods with duck-typed
SimpleNamespace stubs so no Qt widget / camera / event loop is needed (same
pattern as test_v75x_plate_location_manual_click_rim.py). The polarity of the
height→zero-ref descent helper is also re-covered here (it moved from the old
manual-seed flow).
"""

import unittest
from types import SimpleNamespace

from gui.pages.calibration import CalibrationPage
from SupportClasses.StageController import plate_relative_to_zref, ZDIR


class _RecCtrl:
    """Records move_z_absolute calls; reports ZP connected."""
    def __init__(self):
        self.z_moves = []
        self.is_zp_connected = True

    def move_z_absolute(self, z, from_zero_ref=True, **k):
        self.z_moves.append((z, from_zero_ref))


class _FakeTimer:
    def __init__(self):
        self.started = []

    def start(self, ms=0):
        self.started.append(ms)

    def stop(self):
        pass


def _base_stub(**over):
    """A stub carrying every attribute the per-well state machine touches.

    Side-effect methods (navigation, progress, button toggles, baseline,
    focus-score) are no-ops/recorders so the state transitions can be
    asserted directly.
    """
    moves = []
    s = SimpleNamespace(
        _auto_z_scanning=True,
        _auto_z_phase="navigate",
        _auto_z_well_idx=0,
        _auto_z_wells=["A1", "A12", "H12"],
        _auto_z_results={},
        _auto_z_last_z=None,
        _auto_z_pending_z=None,
        _auto_z_ref_z=0.0,
        _auto_z_h=0.0,
        _auto_z_floor_h=0.0,
        _auto_z_current_z=0.0,
        _auto_z_best_z=None,
        _auto_z_best_h=None,
        _auto_z_best_score=0.0,
        _auto_z_baseline_score=0.0,
        _auto_z_decline_count=0,
        _auto_z_coarse_step=0.15,
        _auto_z_fine_step=0.03,
        _auto_z_well_depth=17.4,
        _zauto_approach_margin=1.0,
        _zauto_tilt_margin=0.5,
        _calibrated_positions={"A1": (0, 0), "A12": (1, 1), "H12": (2, 2)},
        _predicted_positions=None,
        _safe_z=5.0,
        _top_z=2.0,
        _plate_bottom_z=None,
        _z_teach_points={},
        controller=_RecCtrl(),
        _auto_z_timer=_FakeTimer(),
        _move_log=moves,
        # side-effect stubs
        _safe_navigate_to=lambda *a, **k: None,
        _auto_z_set_progress=lambda *a, **k: None,
        _auto_z_phase_buttons=lambda: None,
        _auto_z_capture_baseline=lambda: None,
        _auto_z_get_focus_score=lambda: 0.0,
        _auto_z_finish=lambda: setattr(s, "_finished", True),
        _auto_z_move_to_h=lambda h: _move_to_h(s, h),
        _finished=False,
    )
    for k, v in over.items():
        setattr(s, k, v)
    return s


def _move_to_h(s, h):
    h = max(h, s._auto_z_floor_h)
    s._auto_z_h = h
    s._auto_z_current_z = plate_relative_to_zref(s._auto_z_ref_z, h)
    s._move_log.append(h)
    return h


class TestMoveToHeightPolarity(unittest.TestCase):
    """_auto_z_move_to_h converts height-above-bottom → zero-ref Z and clamps."""

    def _stub(self, ref, floor_h):
        return SimpleNamespace(
            _auto_z_ref_z=ref, _auto_z_floor_h=floor_h,
            _auto_z_h=None, _auto_z_current_z=None, controller=_RecCtrl())

    def test_maps_via_plate_relative_and_commands_move(self):
        s = self._stub(ref=10.0, floor_h=-0.5)
        out = CalibrationPage._auto_z_move_to_h(s, 1.0)
        self.assertAlmostEqual(out, 1.0)
        self.assertAlmostEqual(s._auto_z_current_z,
                               plate_relative_to_zref(10.0, 1.0))
        self.assertEqual(len(s.controller.z_moves), 1)
        self.assertTrue(s.controller.z_moves[0][1])  # from_zero_ref

    def test_toward_plate_is_polarity_correct(self):
        s = self._stub(ref=10.0, floor_h=-1.0)
        CalibrationPage._auto_z_move_to_h(s, 1.0)
        z_high = s._auto_z_current_z
        CalibrationPage._auto_z_move_to_h(s, 0.0)
        z_low = s._auto_z_current_z
        if ZDIR < 0:
            self.assertGreater(z_low, z_high)  # toward plate ⇒ larger raw Z
        else:
            self.assertLess(z_low, z_high)

    def test_clamps_to_floor(self):
        s = self._stub(ref=10.0, floor_h=-0.5)
        out = CalibrationPage._auto_z_move_to_h(s, -2.0)
        self.assertAlmostEqual(out, -0.5)


class TestNavigatePausesForFocus(unittest.TestCase):
    """navigate travels then pauses in await_focus — no needle descent yet,
    no timer restart (operator-gated)."""

    def test_navigate_enters_await_focus(self):
        s = _base_stub()
        # use the real enter-await helper to exercise the transition
        s._auto_z_enter_await_focus = lambda: \
            CalibrationPage._auto_z_enter_await_focus(s)
        CalibrationPage._auto_z_tick(s)
        self.assertEqual(s._auto_z_phase, "await_focus")
        # No Z move issued during navigate (needle stays retracted).
        self.assertEqual(s.controller.z_moves, [])
        # No further timer tick scheduled — waiting on Confirm.
        self.assertEqual(s._auto_z_timer.started, [])

    def test_navigate_skips_well_without_position(self):
        s = _base_stub(_calibrated_positions={}, _predicted_positions=None)
        CalibrationPage._auto_z_tick(s)
        self.assertEqual(s._auto_z_well_idx, 1)
        self.assertEqual(s._auto_z_phase, "navigate")


class TestConfirmFocusStartsDescent(unittest.TestCase):
    """Confirm sets up the bounded descent window + starts lowering."""

    def test_window_seeded_from_plate_bottom(self):
        s = _base_stub(_auto_z_phase="await_focus", _plate_bottom_z=12.0,
                       _auto_z_last_z=None)
        CalibrationPage._zauto_confirm_focus(s)
        self.assertEqual(s._auto_z_ref_z, 12.0)
        self.assertAlmostEqual(s._auto_z_floor_h, -0.5)
        self.assertEqual(s._auto_z_phase, "coarse")
        # First move is to the approach height (above the glass).
        self.assertEqual(s._move_log, [1.0])
        self.assertEqual(s._auto_z_timer.started, [300])

    def test_window_seeded_from_prev_accepted_z(self):
        s = _base_stub(_auto_z_phase="await_focus", _plate_bottom_z=12.0,
                       _auto_z_last_z=13.7)
        CalibrationPage._zauto_confirm_focus(s)
        self.assertEqual(s._auto_z_ref_z, 13.7)  # prev Z wins (tracks tilt)

    def test_window_falls_back_to_estimate(self):
        s = _base_stub(_auto_z_phase="await_focus", _plate_bottom_z=None,
                       _auto_z_last_z=None, _top_z=2.0, _auto_z_well_depth=17.4)
        CalibrationPage._zauto_confirm_focus(s)
        self.assertAlmostEqual(s._auto_z_ref_z, 2.0 - 17.4)

    def test_confirm_ignored_when_not_awaiting(self):
        s = _base_stub(_auto_z_phase="coarse")
        CalibrationPage._zauto_confirm_focus(s)
        # No approach move appended (handler returned early).
        self.assertEqual(s._move_log, [])


class TestDescentAutoRecord(unittest.TestCase):
    """The fine sweep records the best-focus Z after the peak (3 declines)."""

    def test_fine_records_best_on_decline(self):
        s = _base_stub(_auto_z_phase="fine", _auto_z_ref_z=12.0,
                       _auto_z_floor_h=-0.5)
        s._auto_z_best_z = 12.05
        s._auto_z_best_h = -0.05
        s._auto_z_best_score = 40.0
        s._auto_z_decline_count = 3
        s._auto_z_h = -0.2
        recorded = {}
        s._auto_z_enter_recorded = lambda z: recorded.setdefault("z", z)
        CalibrationPage._auto_z_tick(s)
        self.assertAlmostEqual(recorded["z"], 12.05)

    def test_fine_floor_records_best(self):
        s = _base_stub(_auto_z_phase="fine", _auto_z_ref_z=12.0,
                       _auto_z_floor_h=-0.5)
        s._auto_z_best_z = 12.4
        s._auto_z_best_h = -0.4
        s._auto_z_best_score = 30.0
        s._auto_z_decline_count = 0
        s._auto_z_h = -0.5  # at the floor
        recorded = {}
        s._auto_z_enter_recorded = lambda z: recorded.setdefault("z", z)
        CalibrationPage._auto_z_tick(s)
        self.assertAlmostEqual(recorded["z"], 12.4)


class TestManualRecordOverride(unittest.TestCase):
    """Record-now captures the current best (or live) Z during the descent."""

    def test_record_now_uses_best(self):
        s = _base_stub(_auto_z_phase="coarse")
        s._auto_z_best_z = 11.9
        s._auto_z_current_z = 12.3
        out = {}
        s._auto_z_enter_recorded = lambda z: out.setdefault("z", z)
        CalibrationPage._zauto_record_now(s)
        self.assertAlmostEqual(out["z"], 11.9)

    def test_record_now_falls_back_to_live_z(self):
        s = _base_stub(_auto_z_phase="fine")
        s._auto_z_best_z = None
        s._auto_z_current_z = 12.3
        out = {}
        s._auto_z_enter_recorded = lambda z: out.setdefault("z", z)
        CalibrationPage._zauto_record_now(s)
        self.assertAlmostEqual(out["z"], 12.3)

    def test_record_now_ignored_outside_descent(self):
        s = _base_stub(_auto_z_phase="await_focus")
        out = {}
        s._auto_z_enter_recorded = lambda z: out.setdefault("z", z)
        CalibrationPage._zauto_record_now(s)
        self.assertNotIn("z", out)


class TestEnterRecordedRetracts(unittest.TestCase):
    """Recording retracts to safe Z and holds the Z pending Accept/Redo."""

    def test_retract_and_pending(self):
        s = _base_stub()
        CalibrationPage._auto_z_enter_recorded(s, 12.34)
        self.assertEqual(s._auto_z_phase, "recorded")
        self.assertAlmostEqual(s._auto_z_pending_z, 12.34)
        # Retracted to safe Z (5.0), nothing committed yet.
        self.assertIn((5.0, True), s.controller.z_moves)
        self.assertEqual(s._auto_z_results, {})

    def test_none_z_still_retracts(self):
        s = _base_stub()
        CalibrationPage._auto_z_enter_recorded(s, None)
        self.assertEqual(s._auto_z_phase, "recorded")
        self.assertIsNone(s._auto_z_pending_z)
        self.assertIn((5.0, True), s.controller.z_moves)


class TestAcceptAndRedo(unittest.TestCase):
    """Accept commits + advances (fit on last well); Redo re-arms the well."""

    def test_accept_commits_and_advances(self):
        s = _base_stub(_auto_z_phase="recorded", _auto_z_well_idx=0,
                       _auto_z_pending_z=12.5)
        CalibrationPage._zauto_accept_well(s)
        self.assertAlmostEqual(s._auto_z_results["A1"], 12.5)
        self.assertAlmostEqual(s._z_teach_points["A1"], 12.5)
        self.assertAlmostEqual(s._auto_z_last_z, 12.5)  # seeds next window
        self.assertEqual(s._auto_z_well_idx, 1)
        self.assertEqual(s._auto_z_phase, "navigate")
        self.assertFalse(s._finished)
        self.assertEqual(s._auto_z_timer.started, [200])

    def test_accept_last_well_finishes(self):
        s = _base_stub(_auto_z_phase="recorded", _auto_z_well_idx=2,
                       _auto_z_pending_z=12.9)
        CalibrationPage._zauto_accept_well(s)
        self.assertAlmostEqual(s._auto_z_results["H12"], 12.9)
        self.assertEqual(s._auto_z_well_idx, 3)
        self.assertTrue(s._finished)

    def test_redo_returns_to_await_focus_same_well(self):
        s = _base_stub(_auto_z_phase="recorded", _auto_z_well_idx=1,
                       _auto_z_pending_z=12.5)
        s._auto_z_enter_await_focus = lambda: \
            CalibrationPage._auto_z_enter_await_focus(s)
        CalibrationPage._zauto_redo_well(s)
        self.assertEqual(s._auto_z_phase, "await_focus")
        self.assertEqual(s._auto_z_well_idx, 1)  # same well
        self.assertIsNone(s._auto_z_pending_z)
        self.assertEqual(s._auto_z_results, {})  # nothing committed

    def test_accept_ignored_when_not_recorded(self):
        s = _base_stub(_auto_z_phase="coarse", _auto_z_pending_z=9.9)
        CalibrationPage._zauto_accept_well(s)
        self.assertEqual(s._auto_z_results, {})


class TestPhaseButtons(unittest.TestCase):
    """_auto_z_phase_buttons enables exactly the buttons for the phase."""

    def _stub(self, phase, scanning=True):
        class _B:
            def __init__(self):
                self.enabled = None

            def setEnabled(self, v):
                self.enabled = v
        return SimpleNamespace(
            _auto_z_phase=phase, _auto_z_scanning=scanning,
            _zauto_btn_confirm=_B(), _zauto_btn_record=_B(),
            _zauto_btn_accept=_B(), _zauto_btn_redo=_B())

    def test_await_focus_enables_confirm_only(self):
        s = self._stub("await_focus")
        CalibrationPage._auto_z_phase_buttons(s)
        self.assertTrue(s._zauto_btn_confirm.enabled)
        self.assertFalse(s._zauto_btn_record.enabled)
        self.assertFalse(s._zauto_btn_accept.enabled)

    def test_coarse_enables_record_only(self):
        s = self._stub("coarse")
        CalibrationPage._auto_z_phase_buttons(s)
        self.assertFalse(s._zauto_btn_confirm.enabled)
        self.assertTrue(s._zauto_btn_record.enabled)

    def test_recorded_enables_accept_and_redo(self):
        s = self._stub("recorded")
        CalibrationPage._auto_z_phase_buttons(s)
        self.assertTrue(s._zauto_btn_accept.enabled)
        self.assertTrue(s._zauto_btn_redo.enabled)
        self.assertFalse(s._zauto_btn_record.enabled)

    def test_not_scanning_disables_all(self):
        s = self._stub("await_focus", scanning=False)
        CalibrationPage._auto_z_phase_buttons(s)
        self.assertFalse(s._zauto_btn_confirm.enabled)


if __name__ == "__main__":
    unittest.main()
