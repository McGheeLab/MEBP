"""
v7.10 — the microscope bore wizard: measurement state, gates, and safety.

The sign arithmetic itself is pinned in ``test_v710_microscope_bore_sign.py``
(composed end to end with the executor, with a mutation check). This file covers
the wizard around it: what it refuses, what it writes, and — the part that can
break hardware — where it arms the plate-bottom floor and how it moves.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtGui import QHideEvent
from PySide6.QtWidgets import QApplication

import gui.widgets.needle_bore_wizard as wizmod
from SupportClasses.PhysicalModels import NeedleBore, NeedleSpec
from gui.widgets.needle_bore_wizard import (
    MIN_SURVEY_CLEARANCE_MM,
    STEP_BORES,
    STEP_TOUCHOFF,
    BoreMeasurement,
    NeedleBoreWizard,
)


def _backpack():
    return NeedleSpec(needle_form="backpack", bores=[
        NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
        NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P2")])


def _single():
    return NeedleSpec(gauge=27, id_um=210, od_um=413)


class _FakeMB:
    """Modal stand-in — an unpatched QMessageBox blocks forever offscreen."""
    warned: list = []
    questions: list = []
    answer = None

    class StandardButton:
        Yes = 1
        No = 0

    @classmethod
    def reset(cls):
        cls.warned, cls.questions = [], []
        cls.answer = cls.StandardButton.Yes

    @classmethod
    def warning(cls, *a, **k):
        cls.warned.append(a[2] if len(a) > 2 else "")

    @classmethod
    def question(cls, *a, **k):
        cls.questions.append(a[2] if len(a) > 2 else "")
        return cls.answer


class _Ctrl:
    def __init__(self, *, xy=True, zp=True, xy_pos=(100_000.0, 50_000.0),
                 raw_z=-22.5):
        self.is_xy_connected = xy
        self.is_zp_connected = zp
        self._xy = xy_pos
        self._raw_z = raw_z
        self.floor_calls = []
        self.z_moves = []
        self.cam_offset = None

    def get_xy_position(self, cached=False):
        return self._xy

    def capture_current_z_raw(self):
        return self._raw_z

    def raw_to_user_z(self, raw):
        return -float(raw)              # a z_up_sign = -1 machine

    def z_up_sign(self):
        return -1.0

    def get_needle_camera_offset_um(self):
        return self.cam_offset

    def print_z_dir(self):
        return 1.0

    def print_height_to_zref(self, h):
        return 10.0 + float(h)

    def move_z_absolute(self, z, from_zero_ref=False):
        self.z_moves.append(float(z))

    def set_print_floor_active(self, on):
        self.floor_calls.append(bool(on))

    def set_needle_camera_offset_um(self, dx, dy=None):
        self.cam_offset = (dx, dy)


class _FakeCam:
    """Enough of a CameraWidget for CameraFeedView to bind to."""

    def __init__(self):
        self.is_running = False
        self.frame_captured = _Sig()
        self.detection_overlay = None

    def get_current_frame(self):
        return None


class _Sig:
    def connect(self, *a, **k):
        pass

    def disconnect(self, *a, **k):
        pass


class _Mgr:
    """Camera manager stand-in: a calibrated microscope at index 2."""

    def __init__(self, calibrated=True):
        self._cal = calibrated
        self.started = []
        self.cameras = [_FakeCam() for _ in range(4)]

    def is_um_per_px_calibrated(self, idx):
        return self._cal

    def effective_um_per_px(self, idx, w):
        return 0.5

    def start(self, idx):
        self.started.append(idx)

    def pixel_to_stage_offset(self, idx, px, py, w, h):
        # Identity-ish optics: 0.5 µm/px, no rotation/mirror.
        return ((px - w / 2.0) * 0.5, (py - h / 2.0) * 0.5)


class _Base(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = wizmod.QMessageBox
        wizmod.QMessageBox = _FakeMB
        self.addCleanup(lambda: setattr(wizmod, "QMessageBox", self._orig_mb))

        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_wiz_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
            Path(self._tmp.name) / "bore.json")
        import SupportClasses.NeedleBoreCalibrationStore as mod
        self._prev_singleton = mod._store
        mod._store = None

        def _restore():
            mod._store = self._prev_singleton
            if self._prev is None:
                os.environ.pop("MEBP_NEEDLE_BORE_CAL_PATH", None)
            else:
                os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = self._prev
        self.addCleanup(_restore)

    def _wizard(self, needle=None, *, ctrl=None, mgr=None,
                plate_bottom=10.0, safe_z=44.0, mic_idx=2, origin=(1.0, 2.0)):
        needle = _backpack() if needle is None else needle
        self.ctrl = ctrl if ctrl is not None else _Ctrl()
        self.mgr = mgr if mgr is not None else _Mgr()
        host = SimpleNamespace(
            controller=self.ctrl,
            _hardware_config=SimpleNamespace(needle=needle),
            _camera_manager=self.mgr,
            _plate_bottom_z=plate_bottom,
            _safe_z=safe_z,
            _top_z=None,
            _max_z=None,
            _needle_origin_um=origin,
            _needle_loc_xy_um=(100_000.0, 50_000.0),
            _ploc_microscope_cam_idx=lambda: mic_idx,
            _ploc_camera_objective_key=lambda: "cam0|10x",
            _safe_navigate_to=lambda x, y, target_z_mm=None, lower_z=True: (
                self.nav.append((x, y, target_z_mm, lower_z))),
            _zoff_capture_current_z=lambda: 12.345,
            _zoff_set_plate_bottom_z=lambda: self.bottom_set.append(True),
            _apply_plate_type_z_estimates=lambda force=False: (
                self.autofill.append(force)),
        )
        self.nav, self.bottom_set, self.autofill = [], [], []
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        return w


# ── pure measurement state ──────────────────────────────────────────

class TestBoreMeasurement(unittest.TestCase):

    def test_datum_offset_is_always_zero(self):
        m = BoreMeasurement(2)
        m.record_click(0, (10, 10), (123.0, -45.0))
        self.assertEqual(m.offset_for(0), (0.0, 0.0))

    def test_offset_needs_both_clicks(self):
        m = BoreMeasurement(2)
        m.record_click(1, (10, 10), (320.0, 0.0))
        self.assertIsNone(m.offset_for(1))

    def test_offset_is_bore_minus_datum(self):
        m = BoreMeasurement(2)
        m.record_click(0, (0, 0), (0.0, 0.0))
        m.record_click(1, (0, 0), (320.0, -140.0))
        self.assertEqual(m.offset_for(1), (320.0, -140.0))

    def test_dz_is_zero_until_both_z_are_recorded(self):
        m = BoreMeasurement(2)
        m.record_z(1, 22.548)
        self.assertEqual(m.dz_for(1), 0.0)

    def test_dz_is_the_height_difference(self):
        m = BoreMeasurement(2)
        m.record_z(0, 22.500)
        m.record_z(1, 22.548)
        self.assertAlmostEqual(m.dz_for(1), 0.048, places=9)

    def test_single_bore_needs_no_z(self):
        """There is nothing to be coplanar WITH."""
        m = BoreMeasurement(1)
        m.record_click(0, (0, 0), (5.0, 6.0))
        self.assertEqual(m.missing_z(), [])
        self.assertTrue(m.is_complete())

    def test_multi_bore_needs_a_z_per_bore(self):
        m = BoreMeasurement(2)
        m.record_click(0, (0, 0), (0.0, 0.0))
        m.record_click(1, (0, 0), (320.0, 0.0))
        self.assertEqual(m.missing_z(), [0, 1])
        self.assertFalse(m.is_complete())

    def test_needle_camera_offset_is_the_datum_click(self):
        m = BoreMeasurement(1)
        m.record_click(0, (0, 0), (-880.0, 415.0))
        self.assertEqual(m.needle_camera_offset(), (-880.0, 415.0))

    def test_needle_camera_offset_is_none_before_the_datum(self):
        self.assertIsNone(BoreMeasurement(2).needle_camera_offset())

    def test_peak_tracks_the_sharpest(self):
        m = BoreMeasurement(2)
        m.note_score(1, 10.0, 22.40)
        m.note_score(1, 90.0, 22.51)
        m.note_score(1, 30.0, 22.60)
        self.assertEqual(m.peak[1], (90.0, 22.51))

    def test_peak_ignores_a_frame_with_no_z(self):
        m = BoreMeasurement(2)
        m.note_score(1, 90.0, None)
        self.assertNotIn(1, m.peak)

    def test_forget_clears_everything_for_one_bore(self):
        m = BoreMeasurement(2)
        m.record_click(1, (0, 0), (1.0, 2.0))
        m.record_z(1, 22.5)
        m.note_score(1, 5.0, 22.5)
        m.forget(1)
        self.assertEqual(m.missing_clicks(), [0, 1])
        self.assertNotIn(1, m.z_user)
        self.assertNotIn(1, m.peak)


# ── gates ───────────────────────────────────────────────────────────

class TestGates(_Base):

    def test_ready_rig_passes(self):
        w = self._wizard()
        ok, why = w.gate(STEP_BORES)
        self.assertTrue(ok, why)

    def test_xy_disconnected_is_refused(self):
        w = self._wizard(ctrl=_Ctrl(xy=False))
        ok, why = w.gate(STEP_BORES)
        self.assertFalse(ok)
        self.assertIn("XY stage", why)

    def test_no_microscope_role_is_refused_by_name(self):
        w = self._wizard(mic_idx=None)
        ok, why = w.gate(STEP_BORES)
        self.assertFalse(ok)
        self.assertIn("Microscope role", why)

    def test_uncalibrated_microscope_is_refused(self):
        w = self._wizard(mgr=_Mgr(calibrated=False))
        ok, why = w.gate(STEP_BORES)
        self.assertFalse(ok)
        self.assertIn("µm/px", why)

    def test_a_SINGLE_bore_needle_is_NOT_refused(self):
        """The v7.9 group hid itself for a single bore. The wizard must not:
        that needle still needs its offset from the microscope centre."""
        w = self._wizard(needle=_single())
        ok, why = w.gate(STEP_BORES)
        self.assertTrue(ok, why)


class TestMeasuringNeedsNoMotionPrerequisites(_Base):
    """The bore offsets are differences between clicks in ONE camera frame, so
    the measurement gate must ask ONLY for what turns a click into a distance.

    The reference heights and the saved needle location exist for the optional
    park *travel*; requiring them to MEASURE made an un-taught height block a
    procedure that never moves the stage.
    """

    def test_missing_plate_bottom_does_not_block_measuring(self):
        w = self._wizard(plate_bottom=None)
        ok, why = w.gate(STEP_BORES)
        self.assertTrue(ok, why)

    def test_missing_safe_z_does_not_block_measuring(self):
        w = self._wizard(safe_z=None)
        ok, why = w.gate(STEP_BORES)
        self.assertTrue(ok, why)

    def test_no_saved_needle_location_does_not_block_measuring(self):
        w = self._wizard()
        w._host._needle_loc_xy_um = None
        ok, why = w.gate(STEP_BORES)
        self.assertTrue(ok, why)

    def test_a_session_starts_and_clicks_land_with_no_reference_heights(self):
        """End to end: the whole point of the change. No plate bottom, no safe
        Z, no saved needle location — the offsets still measure, with no motion."""
        w = self._wizard(plate_bottom=None, safe_z=None)
        w._host._needle_loc_xy_um = None
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_start_session()
        self.assertIsNotNone(w._meas, "the session must arm")
        w.on_view_clicked(960, 540)
        w.on_view_clicked(960 + 640, 540)
        self.assertAlmostEqual(w._meas.offset_for(1)[0], 320.0, places=6)
        self.assertEqual(self.nav, [], "measuring commands no travel")
        self.assertEqual(self.ctrl.z_moves, [], "measuring commands no Z move")

    def test_the_still_needed_prerequisites_are_kept(self):
        """Relaxing the gate must not drop what a click genuinely needs."""
        for kwargs, expect in (({"mic_idx": None}, "Microscope role"),
                               ({"mgr": _Mgr(calibrated=False)}, "µm/px"),
                               ({"ctrl": _Ctrl(xy=False)}, "XY stage")):
            with self.subTest(**kwargs):
                ok, why = self._wizard(**kwargs).gate(STEP_BORES)
                self.assertFalse(ok)
                self.assertIn(expect, why)

    def test_touchoff_still_requires_both_reference_heights(self):
        """Step 4 really does descend toward the glass — it keeps them."""
        ok, why = self._wizard(plate_bottom=None).gate(STEP_TOUCHOFF)
        self.assertFalse(ok)
        self.assertIn("Plate Bottom Z", why)
        ok, why = self._wizard(safe_z=None).gate(STEP_TOUCHOFF)
        self.assertFalse(ok)
        self.assertIn("Safe", why)


class TestParkGate(_Base):
    """The park is the one stage-moving action in step 3, so it keeps every
    prerequisite the measurement shed — on its own gate."""

    def test_ready_rig_may_park(self):
        ok, why = self._wizard().park_gate()
        self.assertTrue(ok, why)

    def test_park_needs_the_plate_bottom(self):
        ok, why = self._wizard(plate_bottom=None).park_gate()
        self.assertFalse(ok)
        self.assertIn("Plate Bottom Z", why)

    def test_park_needs_the_safe_z(self):
        ok, why = self._wizard(safe_z=None).park_gate()
        self.assertFalse(ok)
        self.assertIn("Safe", why)

    def test_park_needs_a_destination(self):
        w = self._wizard()
        w._host._needle_loc_xy_um = None
        ok, why = w.park_gate()
        self.assertFalse(ok)
        self.assertIn("needle location", why)

    def test_every_park_refusal_names_the_no_motion_alternative(self):
        """A blocked park must not read as 'step 3 is blocked'."""
        for kwargs in ({"plate_bottom": None}, {"safe_z": None}):
            with self.subTest(**kwargs):
                ok, why = self._wizard(**kwargs).park_gate()
                self.assertFalse(ok)
                self.assertIn("Start measuring here", why)

    def test_the_park_itself_refuses_without_a_safe_z(self):
        """⚠ The park must be wired to park_gate, not the relaxed measurement
        gate. Asserting on park_gate() alone would not catch _on_park calling
        gate(): with no safe Z, _survey_target_zref still returns a number and
        the destination still exists, so the travel would proceed with nothing
        to retract to."""
        w = self._wizard(safe_z=None)
        w.go_to_step(STEP_BORES)
        w._on_park()
        self.assertEqual(self.nav, [], "no travel may be commanded")
        self.assertIsNone(w._meas, "and no session may be armed")
        self.assertIn("Safe", w._refusal_text or "")

    def test_the_park_itself_refuses_without_a_destination(self):
        w = self._wizard()
        w._host._needle_loc_xy_um = None
        w.go_to_step(STEP_BORES)
        w._on_park()
        self.assertEqual(self.nav, [])
        self.assertIn("needle location", w._refusal_text or "")

    def test_a_blocked_park_leaves_start_measuring_enabled(self):
        """The button states are the operator-visible half of the split."""
        w = self._wizard(plate_bottom=None, safe_z=None)
        w.go_to_step(STEP_BORES)
        self.assertFalse(w._s3_park_btn.isEnabled())
        self.assertTrue(w._s3_start_btn.isEnabled())
        self.assertIn("Plate Bottom Z", w._s3_park_btn.toolTip())

    def test_a_blocked_measurement_disables_both(self):
        w = self._wizard(mic_idx=None)
        w.go_to_step(STEP_BORES)
        self.assertFalse(w._s3_start_btn.isEnabled())
        self.assertFalse(w._s3_park_btn.isEnabled())

    def test_an_inactive_plate_floor_is_disclosed(self):
        """Recording focus Z means jogging down by hand, and the clamp is a
        no-op with no datum — say so instead of implying protection."""
        w = self._wizard(plate_bottom=None)
        w.go_to_step(STEP_BORES)
        self.assertIn("clamp is inactive", w._status.text().replace("  ", " "))
        self.assertEqual(self._wizard()._floor_advisory(), "",
                         "silent once the datum exists")


# ── safety ──────────────────────────────────────────────────────────

class TestSafety(_Base):

    def test_the_park_goes_through_safe_travel_not_a_bare_move(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._on_park()
        self.assertEqual(len(self.nav), 1)
        x, y, z, lower = self.nav[0]
        self.assertIsNotNone(z, "the park must specify a target Z")
        self.assertTrue(lower)
        self.assertEqual(self.ctrl.z_moves, [],
                         "no bare Z move may precede the travel")

    def test_the_park_is_refused_when_gated(self):
        """v7.13: refusals are INLINE (sticky status), never a modal."""
        w = self._wizard(plate_bottom=None)
        w._on_park()
        self.assertEqual(self.nav, [])
        self.assertEqual(_FakeMB.warned, [], "no modal on a refusal")
        self.assertTrue(w._refusal_text)
        self.assertIn("Plate Bottom", w._refusal_text)

    def test_survey_height_is_floored(self):
        """A typed clearance below the minimum must not reach the stage."""
        w = self._wizard()
        w._s3_clearance.setMinimum(0.0)
        w._s3_clearance.setValue(0.0)
        target = w._survey_target_zref()
        self.assertAlmostEqual(target, 10.0 + MIN_SURVEY_CLEARANCE_MM, places=9)

    def test_survey_height_is_above_the_plate_bottom(self):
        w = self._wizard()
        self.assertGreater(w._survey_target_zref(), 10.0)

    def test_touchoff_is_planned_against_the_LONGEST_bore(self):
        """Without this, a bore protruding further than the datum ends up below
        the glass while the datum sits at its nominal clearance."""
        needle = _backpack()
        needle.bores[1].z_offset_mm = 0.200
        w = self._wizard(needle=needle)
        w._s4_clearance.setValue(0.100)
        self.assertAlmostEqual(w._touch_target_zref(), 10.0 + 0.300, places=9)

    def test_touchoff_reduces_to_the_clearance_with_coplanar_bores(self):
        w = self._wizard()
        w._s4_clearance.setValue(0.100)
        self.assertAlmostEqual(w._touch_target_zref(), 10.1, places=9)

    def test_the_approach_is_z_only_so_there_is_no_xy_to_retract_for(self):
        w = self._wizard()
        w.go_to_step(STEP_TOUCHOFF)
        w._on_goto_approx_bottom()
        self.assertEqual(len(self.ctrl.z_moves), 1)
        self.assertEqual(self.nav, [], "the approach must not move XY")

    def test_floor_is_armed_on_entering_step3(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        self.assertEqual(self.ctrl.floor_calls, [True])

    def test_floor_is_disarmed_on_leaving(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w.go_to_step("z_refs")
        self.assertEqual(self.ctrl.floor_calls, [True, False])

    def test_floor_is_disarmed_on_hide(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w.hideEvent(QHideEvent())
        self.assertEqual(self.ctrl.floor_calls, [True, False])
        self.assertFalse(w._floor_armed)

    def test_a_controller_without_the_floor_api_still_works(self):
        """Older controllers / partial test doubles lack the method entirely."""
        class _Old(_Ctrl):
            set_print_floor_active = None
        w = self._wizard(ctrl=_Old())
        w.go_to_step(STEP_BORES)             # must not raise
        self.assertFalse(w._floor_armed)

    def test_moving_between_the_two_armed_steps_never_drops_the_floor(self):
        """Disarm-then-rearm would open a window with the clamp off."""
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w.go_to_step(STEP_TOUCHOFF)
        self.assertEqual(self.ctrl.floor_calls, [True])
        w.hideEvent(QHideEvent())
        self.assertEqual(self.ctrl.floor_calls, [True, False])


# ── clicking and committing ─────────────────────────────────────────

class _View:
    def __init__(self, w=1920, h=1080):
        self.image_size = (w, h)
        self.cam_idx = 2

    def set_camera(self, idx):
        self.cam_idx = idx


class TestClicking(_Base):

    def _armed(self, **kw):
        w = self._wizard(**kw)
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        return w

    def test_a_click_records_the_active_bore(self):
        w = self._armed()
        w.on_view_clicked(960 + 640, 540)          # +640 px = +320 µm
        self.assertIn(0, w._meas.pto)
        self.assertAlmostEqual(w._meas.pto[0][0], 320.0, places=6)

    def test_clicking_advances_to_the_next_unclicked_bore(self):
        w = self._armed()
        self.assertEqual(w._active_bore, 0)
        w.on_view_clicked(960, 540)
        self.assertEqual(w._active_bore, 1)

    def test_the_measured_offset_matches_the_pixel_separation(self):
        w = self._armed()
        w.on_view_clicked(960, 540)                # datum at centre
        w.on_view_clicked(960 + 640, 540)          # bore 2, +320 µm
        self.assertAlmostEqual(w._meas.offset_for(1)[0], 320.0, places=6)

    def test_a_drifted_stage_refuses_the_click(self):
        """The one-frame method's validity rests on the stage being still.
        v7.13: the refusal is inline (status + red drift readout), not a modal."""
        w = self._armed()
        self.ctrl._xy = (100_100.0, 50_000.0)      # 100 µm drift
        w.on_view_clicked(960, 540)
        self.assertEqual(w._meas.pto, {})
        self.assertEqual(_FakeMB.warned, [], "no modal on a refusal")
        self.assertTrue(w._refusal_text)
        self.assertIn("moved", w._refusal_text)
        self.assertIn("µm", w._s3_drift_lbl.text())

    def test_the_drift_readout_goes_red_above_the_limit(self):
        w = self._armed()
        w._update_drift_label()
        self.assertIn("OK to click", w._s3_drift_lbl.text())
        self.ctrl._xy = (100_100.0, 50_000.0)
        w._update_drift_label()
        self.assertIn("clicks paused", w._s3_drift_lbl.text())

    def test_start_measuring_anchors_the_session_at_the_current_position(self):
        """The park is a convenience; a hand-jogged stage anchors the same
        stationarity guarantee via 'Start measuring'."""
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        self.ctrl._xy = (77_000.0, 33_000.0)
        w._on_start_session()
        self.assertEqual(self.nav, [], "no motion is commanded")
        self.assertEqual(w._park_xy, (77_000.0, 33_000.0))
        self.assertIsNotNone(w._meas)
        # Clicks measure against the session anchor, wherever it is.
        w.on_view_clicked(960 + 640, 540)
        self.assertAlmostEqual(w._meas.pto[0][0], 320.0, places=6)

    def test_start_measuring_is_refused_when_gated(self):
        """Gated on what a CLICK needs (a µm/px-calibrated microscope), not on
        the park's reference heights — see
        TestMeasuringNeedsNoMotionPrerequisites."""
        w = self._wizard(mgr=_Mgr(calibrated=False))
        w._on_start_session()
        self.assertIsNone(w._meas)
        self.assertTrue(w._refusal_text)

    def test_clicks_outside_step3_are_ignored(self):
        w = self._armed()
        w.go_to_step(STEP_TOUCHOFF)
        w.on_view_clicked(960, 540)
        self.assertEqual(w._meas.pto, {})

    def test_recording_z_stores_the_height_frame_value(self):
        w = self._armed()
        w.on_view_clicked(960, 540)
        w._on_record_z(0)
        self.assertAlmostEqual(w._meas.z_user[0], 22.5, places=9)

    def test_use_peak_commands_no_motion(self):
        w = self._armed()
        w.on_view_clicked(960, 540)
        w._meas.note_score(0, 99.0, 22.531)
        before = list(self.ctrl.z_moves)
        w._on_use_peak(0)
        self.assertAlmostEqual(w._meas.z_user[0], 22.531, places=9)
        self.assertEqual(self.ctrl.z_moves, before)

    def test_redo_clears_that_bore_only(self):
        w = self._armed()
        w.on_view_clicked(960, 540)
        w.on_view_clicked(1600, 540)
        w._on_redo_bore(1)
        self.assertIn(0, w._meas.pto)
        self.assertNotIn(1, w._meas.pto)


class TestCommit(_Base):

    def _measured(self, needle=None, dz=(22.500, 22.548)):
        w = self._wizard(needle=needle)
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        n = w._bore_count()
        for k in range(n):
            w._active_bore = k
            w.on_view_clicked(960 + 640 * k, 540)
            if n > 1:
                w._meas.record_z(k, dz[k])
        return w

    def test_commit_writes_every_bore(self):
        w = self._measured()
        w._on_commit_bores()
        store = w._store()
        self.assertEqual(sorted(store.measured_bore_indices()), [0, 1])
        self.assertAlmostEqual(store.offset_um(1)[0], 320.0, places=5)

    def test_commit_writes_a_real_per_bore_z(self):
        """BUG B: this is the number that used to be 0.0 every time."""
        w = self._measured()
        w._on_commit_bores()
        self.assertAlmostEqual(w._store().z_offset_mm(1), 0.048, places=6)

    def test_commit_sets_the_needle_camera_offset(self):
        w = self._measured()
        w._on_commit_bores()
        self.assertIsNotNone(self.ctrl.cam_offset)

    def test_commit_reaches_the_live_needle(self):
        w = self._measured()
        w._on_commit_bores()
        needle = w._needle()
        self.assertAlmostEqual(needle.bores[1].offset_um[0], 320.0, places=5)
        self.assertAlmostEqual(needle.bores[1].z_offset_mm, 0.048, places=6)

    def test_a_missing_click_refuses_and_writes_nothing(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        w.on_view_clicked(960, 540)          # datum only
        w._on_commit_bores()
        self.assertEqual(w._store().all_bores(), [])
        self.assertEqual(_FakeMB.warned, [], "no modal on a refusal")
        self.assertIn("Still to click", w._refusal_text)

    def test_the_commit_button_is_disabled_with_the_reason_as_tooltip(self):
        """v7.13: the operator never has to click to find out what's missing."""
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        w.refresh()
        self.assertFalse(w._s3_commit.isEnabled())
        self.assertIn("bore 1", w._s3_commit.toolTip())
        for k in (0, 1):
            w._active_bore = k
            w.on_view_clicked(960 + 640 * k, 540)
        w.refresh()
        self.assertTrue(w._s3_commit.isEnabled())

    def test_no_datum_refuses(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        w._active_bore = 1
        w.on_view_clicked(1600, 540)
        w._on_commit_bores()
        self.assertEqual(w._store().all_bores(), [])

    def test_an_implausible_offset_is_refused_before_writing(self):
        """A clamped offset is a wrong move that looks right — refuse instead."""
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        w.on_view_clicked(960, 540)
        w._active_bore = 1
        w.on_view_clicked(960 + 12000, 540)     # 6 mm — far past the limit
        w._meas.record_z(0, 22.5)
        w._meas.record_z(1, 22.5)
        w._on_commit_bores()
        self.assertEqual(w._store().all_bores(), [])
        self.assertIn("beyond the", w._refusal_text)

    def test_an_implausible_dz_is_refused_before_writing(self):
        w = self._measured(dz=(22.500, 25.000))     # 2.5 mm apart
        w._on_commit_bores()
        self.assertEqual(w._store().all_bores(), [])
        self.assertIn("Z offset", w._refusal_text)

    def test_a_single_bore_commit_records_only_the_camera_offset(self):
        w = self._measured(needle=_single())
        w._on_commit_bores()
        self.assertIsNotNone(self.ctrl.cam_offset)
        self.assertEqual(w._store().measured_bore_indices(), [0])

    def test_restart_forgets_everything(self):
        w = self._measured()
        w._on_restart_bores()
        self.assertEqual(w._meas.pto, {})


class TestStepStates(_Base):

    def test_needle_zero_is_ok_once_the_origin_is_set(self):
        self.assertEqual(self._wizard().step_state("needle_zero"), "ok")

    def test_needle_zero_is_todo_without_an_origin(self):
        self.assertEqual(self._wizard(origin=None).step_state("needle_zero"),
                         "todo")

    def test_z_refs_needs_both_heights(self):
        self.assertEqual(self._wizard().step_state("z_refs"), "ok")
        self.assertEqual(self._wizard(safe_z=None).step_state("z_refs"), "todo")

    def test_bores_is_ok_once_every_bore_is_stored(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed = _View()
        w._on_park()
        for k in (0, 1):
            w._active_bore = k
            w.on_view_clicked(960 + 640 * k, 540)
            w._meas.record_z(k, 22.5 + 0.048 * k)
        w._on_commit_bores()
        self.assertEqual(w.step_state(STEP_BORES), "ok")

    def test_autofill_asks_before_overwriting_a_taught_value(self):
        w = self._wizard()
        w._on_autofill_z()
        self.assertEqual(len(_FakeMB.questions), 1)
        self.assertEqual(self.autofill, [True])

    def test_declining_the_autofill_changes_nothing(self):
        _FakeMB.answer = _FakeMB.StandardButton.No
        w = self._wizard()
        w._on_autofill_z()
        self.assertEqual(self.autofill, [])


if __name__ == "__main__":
    unittest.main()
