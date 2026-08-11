"""
test_v75x_camera_rotation_cal_and_monitor.py — per-camera rotation-vs-stage
calibration on Hardware Setup → Cameras + the new MONITOR overview camera.

v7.5.x (operator request, 2026-07-24):
1. Every camera slot carries a calibration saying what the camera's rotation
   is relative to the microscope stage. The microscope views along Z, so its
   rotation is in the stage XY plane (nominal axis-aligned); the needle
   cameras are mounted at ~45° to the stage X or Y axis (nominal ±45°).
   The slot strip commits ROTATION ONLY — µm/px keeps its own flows.
2. New CameraRole.MONITOR — a camera resting on the stage that overviews the
   entire operation — with a 4th camera slot (MAX_LIVE_CAMERAS 3 → 4).
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.HardwareConfig import (
    HardwareConfig, CameraRole, SINGLETON_CAMERA_ROLES, MAX_LIVE_CAMERAS,
)


# ═══════════════════════════════════════════════════════════════════
#  Monitor role + 4th slot (config layer)
# ═══════════════════════════════════════════════════════════════════

class TestMonitorRole(unittest.TestCase):
    def test_enum_value(self):
        self.assertEqual(CameraRole.MONITOR.value, "monitor")

    def test_monitor_is_singleton(self):
        self.assertIn(CameraRole.MONITOR, SINGLETON_CAMERA_ROLES)

    def test_max_live_cameras_is_four(self):
        self.assertEqual(MAX_LIVE_CAMERAS, 4)
        self.assertEqual(len(HardwareConfig().camera_roles), 4)

    def test_serialization_round_trip(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.MICROSCOPE)
        cfg.set_camera_role(3, CameraRole.MONITOR)
        data = cfg.to_dict()
        self.assertEqual(data["camera_roles"],
                         ["needle_x", "needle_y", "microscope", "monitor"])
        back = HardwareConfig.from_dict(data)
        self.assertEqual(back.camera_roles[3], CameraRole.MONITOR)

    def test_legacy_three_slot_config_pads_to_four(self):
        """Pre-v7.5.x configs saved 3 roles — they load with the 4th slot
        UNASSIGNED (no settings migration needed)."""
        back = HardwareConfig.from_dict(
            {"camera_roles": ["needle_x", "needle_y", "microscope"]})
        self.assertEqual(len(back.camera_roles), 4)
        self.assertEqual(back.camera_roles[3], CameraRole.UNASSIGNED)

    def test_monitor_singleton_enforced(self):
        """Assigning MONITOR to a second slot clears it from the first."""
        cfg = HardwareConfig()
        cfg.set_camera_role(3, CameraRole.MONITOR)
        cfg.set_camera_role(0, CameraRole.MONITOR)
        self.assertEqual(cfg.camera_roles[0], CameraRole.MONITOR)
        self.assertEqual(cfg.camera_roles[3], CameraRole.UNASSIGNED)

    def test_unknown_role_still_degrades(self):
        back = HardwareConfig.from_dict({"camera_roles": ["bogus"] * 4})
        self.assertTrue(
            all(r == CameraRole.UNASSIGNED for r in back.camera_roles))


# ═══════════════════════════════════════════════════════════════════
#  Nominal-rotation helpers (pure)
# ═══════════════════════════════════════════════════════════════════

class TestNominalRotationHelpers(unittest.TestCase):
    def setUp(self):
        from gui.pages.hardware_setup import (
            nominal_rotation_delta, role_nominal_rotations, role_rotation_hint,
        )
        self.delta = nominal_rotation_delta
        self.nominals = role_nominal_rotations
        self.hint = role_rotation_hint

    def test_role_nominal_sets(self):
        self.assertEqual(self.nominals(CameraRole.NEEDLE_X),
                         self.nominals(CameraRole.NEEDLE_Y))
        self.assertIn(45.0, self.nominals(CameraRole.NEEDLE_X))
        self.assertIn(0.0, self.nominals(CameraRole.MICROSCOPE))
        self.assertIn(0.0, self.nominals(CameraRole.MONITOR))
        self.assertNotIn(45.0, self.nominals(CameraRole.MICROSCOPE))

    def test_microscope_near_axis(self):
        nom, d = self.delta(2.0, CameraRole.MICROSCOPE)
        self.assertEqual(nom, 0.0)
        self.assertAlmostEqual(d, 2.0)
        nom, d = self.delta(91.0, CameraRole.MICROSCOPE)
        self.assertEqual(nom, 90.0)
        self.assertAlmostEqual(d, 1.0)

    def test_needle_near_diagonal(self):
        nom, d = self.delta(44.0, CameraRole.NEEDLE_X)
        self.assertEqual(nom, 45.0)
        self.assertAlmostEqual(d, -1.0)
        nom, d = self.delta(-46.0, CameraRole.NEEDLE_Y)
        self.assertEqual(nom, -45.0)
        self.assertAlmostEqual(d, -1.0)
        nom, d = self.delta(133.0, CameraRole.NEEDLE_X)
        self.assertEqual(nom, 135.0)
        self.assertAlmostEqual(d, -2.0)

    def test_wraparound(self):
        """-178° is 2° past the 180° axis-aligned nominal, not -178° from 0."""
        nom, d = self.delta(-178.0, CameraRole.MICROSCOPE)
        self.assertEqual(nom, 180.0)
        self.assertAlmostEqual(d, 2.0)

    def test_hints_are_role_specific(self):
        mic = self.hint(CameraRole.MICROSCOPE)
        needle = self.hint(CameraRole.NEEDLE_X)
        mon = self.hint(CameraRole.MONITOR)
        self.assertIn("Z", mic)
        self.assertIn("45", needle)
        self.assertIn("overview", mon)
        self.assertNotEqual(mic, needle)
        # Unassigned still gets a generic description (never raises).
        self.assertTrue(self.hint(CameraRole.UNASSIGNED))


# ═══════════════════════════════════════════════════════════════════
#  Slot rotation strip (offscreen page)
# ═══════════════════════════════════════════════════════════════════

class _PageHarness(unittest.TestCase):
    """Offscreen HardwareSetupPage + real CameraManager + temp cal store
    (mirrors test_v75x_camera_calibration_store's harness)."""

    DS = [
        {"index": 0, "name": "Teslong", "device_path": "PATH_A"},
        {"index": 1, "name": "Teslong", "device_path": "PATH_B"},
        {"index": 2, "name": "Scope", "device_path": "PATH_C"},
        {"index": 3, "name": "Overview", "device_path": "PATH_D"},
    ]

    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self.CCS = CCS
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

        from gui.pages.hardware_setup import HardwareSetupPage
        from gui.widgets.camera_manager import CameraManager
        self.pg = HardwareSetupPage()
        self.pg._auto_load_cameras = lambda: None  # never probe hardware
        self.mgr = CameraManager()  # default slot count = MAX_LIVE_CAMERAS
        self.pg.set_camera_manager(self.mgr)
        self.cfg = HardwareConfig()
        self.pg.set_config(self.cfg)
        self.mgr._ds_cameras = list(self.DS)
        self.mgr.get_source = lambda i: ("opencv", i)


class TestSlotRotationStrip(_PageHarness):
    def test_four_slot_rows_with_rotation_widgets(self):
        self.assertEqual(len(self.pg._live_cam_source_combos), 4)
        self.assertEqual(len(self.pg._live_cam_rot_labels), 4)
        self.assertEqual(len(self.pg._live_cam_rot_btns), 4)

    def test_role_combo_offers_monitor(self):
        for combo in self.pg._live_cam_role_combos:
            self.assertGreaterEqual(combo.findData(CameraRole.MONITOR), 0)

    def test_monitor_badge_props(self):
        label, variant = self.pg._role_badge_props(CameraRole.MONITOR)
        self.assertEqual(label, "Monitor")

    def test_readout_not_calibrated_then_value(self):
        self.pg._config.set_camera_role(0, CameraRole.NEEDLE_X)
        self.pg._refresh_slot_rotation_displays()
        self.assertIn("not calibrated", self.pg._live_cam_rot_labels[0].text())
        self.assertIn("45", self.pg._live_cam_rot_labels[0].text())  # hint

        # v7.5.x (rotated rig): a needle cam's Δ-vs-nominal readout tracks the
        # MOUNT direction (column_dir_deg); rotation_deg is the display roll,
        # shown separately.
        self.mgr.set_column_dir_deg(0, 44.5)
        self.mgr.set_rotation_deg(0, 1.2)
        self.pg._refresh_slot_rotation_displays()
        txt = self.pg._live_cam_rot_labels[0].text()
        self.assertIn("44.5°", txt)
        self.assertIn("45", txt)          # nearest nominal
        self.assertIn("-0.5", txt)        # Δ from nominal
        self.assertIn("roll +1.2°", txt)  # display roll shown separately

    def test_needle_roll_never_gets_nominal_delta(self):
        # A needle cam with ONLY a roll must not present the roll as a
        # rotation-vs-stage with a Δ-from-±45 sanity figure.
        self.pg._config.set_camera_role(0, CameraRole.NEEDLE_X)
        self.mgr.set_rotation_deg(0, 44.5)  # roll only (no mount measured)
        self.pg._refresh_slot_rotation_displays()
        txt = self.pg._live_cam_rot_labels[0].text()
        self.assertIn("mount: not calibrated", txt)
        self.assertIn("roll +44.5°", txt)

    def test_readout_hint_tracks_role(self):
        self.mgr.set_rotation_deg(2, 1.2)
        self.pg._config.set_camera_role(2, CameraRole.MICROSCOPE)
        self.pg._refresh_slot_rotation_displays()
        self.assertIn("XY plane", self.pg._live_cam_rot_labels[2].text())
        self.pg._config.set_camera_role(2, CameraRole.MONITOR)
        self.pg._refresh_slot_rotation_displays()
        self.assertIn("overview", self.pg._live_cam_rot_labels[2].text())

    def test_rotation_button_gated_on_running(self):
        self.pg._refresh_camera_preview_state()
        for btn in self.pg._live_cam_rot_btns:
            self.assertFalse(btn.isEnabled())
        self.mgr.is_running = lambda i: i == 1
        self.pg._refresh_camera_preview_state()
        self.assertTrue(self.pg._live_cam_rot_btns[1].isEnabled())
        self.assertFalse(self.pg._live_cam_rot_btns[0].isEnabled())


class TestApplySlotRotation(_PageHarness):
    def test_pushes_live_and_persists_rotation_only(self):
        """Commit path: live manager updated + identity-keyed store write
        that PRESERVES an existing µm/px sibling (rotation-only policy)."""
        store = self.CCS.get_store()
        store.set_calibration("dshow:PATH_A", 5.54, name="Teslong")

        self.pg._apply_slot_rotation(0, 44.2)

        self.assertEqual(self.mgr.get_rotation_deg(0), 44.2)
        entry = store.get_calibration("dshow:PATH_A")
        self.assertAlmostEqual(entry["rotation_deg"], 44.2, places=3)
        self.assertAlmostEqual(entry["um_per_px"], 5.54, places=6)  # untouched

    def test_readout_refreshes_after_apply(self):
        self.pg._config.set_camera_role(1, CameraRole.NEEDLE_Y)
        self.pg._apply_slot_rotation(1, -44.0)
        self.assertIn("-44.0°", self.pg._live_cam_rot_labels[1].text())

    def test_microscope_slot_syncs_objectives(self):
        """A rotation calibrated on the MICROSCOPE slot flows into the
        objective card's per-objective sync (stale-rotation guard)."""
        calls = []

        class _CardStub:
            def adopt_camera_rotation(self, deg):
                calls.append(deg)

        self.pg._objective_cal_card = _CardStub()
        self.pg._config.set_camera_role(2, CameraRole.MICROSCOPE)
        self.pg._apply_slot_rotation(2, 1.7)
        self.assertEqual(calls, [1.7])
        # Non-microscope slots do NOT sync objectives.
        self.pg._apply_slot_rotation(0, 45.0)
        self.assertEqual(calls, [1.7])

    def test_rotation_restores_on_slot_assignment(self):
        """The stored per-identity rotation flows back through the existing
        _restore_calibration_for_slot path (regression: rotation restores
        independent of µm/px)."""
        self.pg._apply_slot_rotation(3, 0.8)

        # Fresh session: new manager, same on-disk store.
        self.CCS._store = self.CCS.CameraCalibrationStore(self.tmp)
        from gui.widgets.camera_manager import CameraManager
        mgr2 = CameraManager()
        self.pg._camera_manager = mgr2
        mgr2._ds_cameras = list(self.DS)
        mgr2.get_source = lambda i: ("opencv", i)
        combo = self.pg._live_cam_source_combos[3]
        combo.blockSignals(True)
        combo.addItem("cam3", ("opencv", 3))
        combo.setCurrentIndex(combo.count() - 1)
        combo.blockSignals(False)

        self.assertIsNone(mgr2.get_rotation_deg(3))
        self.pg._restore_calibration_for_slot(3)
        self.assertAlmostEqual(mgr2.get_rotation_deg(3), 0.8, places=3)


# ═══════════════════════════════════════════════════════════════════
#  Mirrored view (horizontal handedness flip)
# ═══════════════════════════════════════════════════════════════════

class TestMirroredMapping(unittest.TestCase):
    """Frames are RAW; the click→stage map applies mirror (dx→−dx) then R(θ)."""

    def _mgr(self, um_per_px=2.0, resolution=None):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager.__new__(CameraManager)
        mgr._max_cameras = 2
        mgr._um_per_px = [um_per_px] * 2
        mgr._um_per_px_set = [True] * 2
        mgr._um_per_px_res = [resolution] * 2
        mgr._rotation_deg = [None] * 2
        mgr._mirrored = [False] * 2
        return mgr

    def test_default_identity(self):
        mgr = self._mgr(2.0)
        dx, dy = mgr.pixel_to_stage_offset(0, 60, 50, 100, 100)
        self.assertAlmostEqual(dx, 20.0)
        self.assertAlmostEqual(dy, 0.0)

    def test_mirror_negates_x_only(self):
        mgr = self._mgr(2.0)
        mgr.set_mirrored(0, True)
        dx, dy = mgr.pixel_to_stage_offset(0, 60, 70, 100, 100)
        self.assertAlmostEqual(dx, -20.0)   # x flipped
        self.assertAlmostEqual(dy, 40.0)    # y untouched

    def test_mirror_composes_with_rotation(self):
        """flip-x THEN R(90°): (10,0)px → (-10,0) → R(90°) → (0,-10)."""
        mgr = self._mgr(1.0)
        mgr.set_rotation_deg(0, 90.0)
        mgr.set_mirrored(0, True)
        dx, dy = mgr.pixel_to_stage_offset(0, 60, 50, 100, 100)
        self.assertAlmostEqual(dx, 0.0, places=6)
        self.assertAlmostEqual(dy, -10.0, places=6)

    def test_getattr_guard_for_test_doubles(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager.__new__(CameraManager)
        mgr._max_cameras = 1
        mgr._um_per_px = [2.0]
        mgr._um_per_px_set = [True]
        mgr._um_per_px_res = [None]
        mgr._rotation_deg = [None]
        self.assertFalse(mgr.get_mirrored(0))
        dx, dy = mgr.pixel_to_stage_offset(0, 60, 50, 100, 100)
        self.assertAlmostEqual(dx, 20.0)


class TestCameraWidgetMirrorFlag(unittest.TestCase):
    """The CameraWidget mirror is a stored flag ONLY — frames stay raw
    (orientation is applied per consumer)."""

    def test_flag_stored_no_frame_method(self):
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget.__new__(CameraWidget)
        w._mirrored = False
        self.assertFalse(w.mirrored)
        w.set_mirrored(True)
        self.assertTrue(w.mirrored)
        # The old source-flip helper is gone (frames are raw now).
        self.assertFalse(hasattr(w, "_maybe_mirror_frame"))


class TestManagerMirrorDelegates(unittest.TestCase):
    """CameraManager.get/set_mirrored delegate to the owning CameraWidget."""

    def test_set_get_roundtrip(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr.set_mirrored(0, True)
        self.assertTrue(mgr.get_mirrored(0))
        cam = mgr._widget(0)
        if cam is not None:
            self.assertTrue(cam.mirrored)
        mgr.set_mirrored(0, False)
        self.assertFalse(mgr.get_mirrored(0))

    def test_view_orientation_convenience(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr.set_mirrored(0, True)
        mgr.set_rotation_deg(0, 37.0)
        mir, rot = mgr.view_orientation(0)
        self.assertTrue(mir)
        self.assertAlmostEqual(rot, 37.0)


class TestMirrorStore(unittest.TestCase):
    def setUp(self):
        import tempfile
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        self.store = CameraCalibrationStore(self.tmp)

    def test_default_false_and_absent(self):
        self.assertFalse(self.store.get_mirrored("id"))

    def test_set_true_then_read_back_persisted(self):
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self.store.set_mirrored("id", True, name="Cam")
        self.assertTrue(self.store.get_mirrored("id"))
        self.assertTrue(CameraCalibrationStore(self.tmp).get_mirrored("id"))

    def test_false_pops_key_but_keeps_siblings(self):
        self.store.set_calibration("id", 5.5, rotation_deg=44.0, name="Cam")
        self.store.set_mirrored("id", True)
        self.store.set_mirrored("id", False)
        self.assertFalse(self.store.get_mirrored("id"))
        entry = self.store.get_calibration("id")
        self.assertNotIn("mirrored", entry)              # key popped
        self.assertAlmostEqual(entry["um_per_px"], 5.5)  # sibling kept
        self.assertAlmostEqual(entry["rotation_deg"], 44.0)

    def test_mirror_preserves_um_per_px_and_rotation(self):
        self.store.set_calibration("id", 3.3, rotation_deg=-45.0, name="Cam")
        self.store.set_mirrored("id", True)
        entry = self.store.get_calibration("id")
        self.assertTrue(entry["mirrored"])
        self.assertAlmostEqual(entry["um_per_px"], 3.3)
        self.assertAlmostEqual(entry["rotation_deg"], -45.0)


class TestSlotMirrorUI(_PageHarness):
    def test_checkbox_per_slot(self):
        self.assertEqual(len(self.pg._live_cam_mirror_checks), 4)

    def test_apply_mirror_pushes_live_and_persists(self):
        """v7.16: these controls set the LIVE VIEW, not the measured geometry.

        They used to write ``mirrored`` — the value the mosaic orients tiles by
        and every click is mapped through — which gave that one number two
        writers (here and the objective calibration's stage-motion measurement).
        The measurement won, so setting the view up and then calibrating flipped
        the view back. That is the defect this asserts is gone.
        """
        store = self.CCS.get_store()
        store.set_calibration("dshow:PATH_A", 5.54, name="Teslong")
        self.pg._apply_slot_mirror(0, True)
        self.assertTrue(self.mgr.display_orientation(0)[0])   # live view flipped
        self.assertFalse(self.mgr.get_mirrored(0))            # geometry untouched
        entry = store.get_calibration("dshow:PATH_A")
        self.assertTrue(entry["view_orientation"]["flip_x"])
        self.assertNotIn("mirrored", entry)                   # not the measured one
        self.assertAlmostEqual(entry["um_per_px"], 5.54)      # µm/px untouched

    def test_a_measurement_does_not_move_the_live_view(self):
        """THE reported bug, end to end: set the view up, then calibrate.

        Drives the real store + manager through the page's own commit path, then
        applies a measurement the way ``objective_calibration_card`` does.
        """
        store = self.CCS.get_store()
        store.set_calibration("dshow:PATH_A", 5.54, name="Teslong")
        # 1) operator dials in the live view they want
        self.pg._apply_slot_mirror(0, True)
        self.pg._apply_slot_rotation_value(0, 180.0)
        before = self.mgr.display_orientation(0)
        self.assertEqual(before, (True, False, 180.0))
        # 2) the objective/mosaic calibration measures camera→stage and commits
        self.mgr.set_rotation_deg(0, -90.0)
        self.mgr.set_mirrored(0, False)
        self.mgr.set_flip_y(0, True)
        store.set_rotation("dshow:PATH_A", -90.0)
        store.set_mirrored("dshow:PATH_A", False)
        store.set_flip_y("dshow:PATH_A", True)
        # 3) the live view must not have moved…
        self.assertEqual(self.mgr.display_orientation(0), before)
        self.assertEqual(
            store.get_view_orientation("dshow:PATH_A"),
            {"rotation_deg": 180.0, "flip_x": True, "flip_y": False})
        # …while the mosaic/click geometry took the measurement.
        self.assertEqual(self.mgr.full_orientation(0), (False, True, -90.0))

    def test_the_split_is_what_makes_this_pass(self):
        """Guard the guard: with no view preference the feed still follows the
        measurement, so the tests above cannot be passing merely because the
        display ignores orientation altogether."""
        self.mgr.clear_display_orientation(0)
        self.mgr.set_rotation_deg(0, -90.0)
        self.mgr.set_mirrored(0, True)
        self.assertFalse(self.mgr.has_display_orientation(0))
        self.assertEqual(self.mgr.display_orientation(0),
                         self.mgr.full_orientation(0))
        self.assertEqual(self.mgr.display_orientation(0), (True, False, -90.0))

    def test_mirror_flips_the_live_preview(self):
        """Regression: toggling the mirror checkbox must mirror the slot's live
        PREVIEW (CameraFeedView.set_view_orientation), not just set the flag."""
        calls = []

        class _FakeView:
            def set_view_orientation(self, mir, rot, flip_y=False):
                calls.append((bool(mir), float(rot), bool(flip_y)))

        self.pg._live_cam_previews[0] = _FakeView()
        self.pg._apply_slot_mirror(0, True)
        self.assertTrue(calls)
        self.assertEqual(calls[-1][0], True)     # mirrored pushed to the preview
        calls.clear()
        self.pg._apply_slot_mirror(0, False)
        self.assertEqual(calls[-1][0], False)    # un-mirror pushed too

    def test_readout_names_the_live_view_separately(self):
        """v7.16: the readout reports the MEASURED orientation, so it must say
        so when the live view differs — otherwise "Rotation vs stage: 180°" reads
        as a claim about the feed, which is exactly the ambiguity the split
        introduces."""
        self.pg._config.set_camera_role(0, CameraRole.NEEDLE_X)
        self.pg._apply_slot_mirror(0, True)
        txt = self.pg._live_cam_rot_labels[0].text()
        self.assertIn("live view", txt)
        self.assertIn("flip X", txt)
        # Checkbox state tracks the live-view orientation.
        self.assertTrue(self.pg._live_cam_mirror_checks[0].isChecked())
        self.pg._apply_slot_mirror(0, False)
        self.assertNotIn("flip X", self.pg._live_cam_rot_labels[0].text())
        self.assertFalse(self.pg._live_cam_mirror_checks[0].isChecked())

    def test_readout_still_reports_a_measured_mirror(self):
        """The measured mirror keeps its own word in the readout — it is what the
        mosaic uses, so it must stay visible independently of the view."""
        self.pg._config.set_camera_role(0, CameraRole.NEEDLE_X)
        self.mgr.set_mirrored(0, True)          # a MEASURED mirror
        self.pg._refresh_slot_rotation_displays()
        self.assertIn("mirrored", self.pg._live_cam_rot_labels[0].text())

    def test_flip_y_and_rotation_controls_exist(self):
        self.assertEqual(len(self.pg._live_cam_flip_y_checks), 4)
        self.assertEqual(len(self.pg._live_cam_rot_spins), 4)

    def test_apply_flip_y_pushes_live_and_persists(self):
        store = self.CCS.get_store()
        store.set_calibration("dshow:PATH_A", 5.54, name="Teslong")
        self.pg._apply_slot_flip_y(0, True)
        self.assertTrue(self.mgr.display_orientation(0)[1])   # live view
        self.assertFalse(self.mgr.get_flip_y(0))              # geometry untouched
        entry = store.get_calibration("dshow:PATH_A")
        self.assertTrue(entry["view_orientation"]["flip_y"])
        self.assertNotIn("flip_y", entry)
        self.assertAlmostEqual(entry["um_per_px"], 5.54)      # µm/px untouched

    def test_apply_rotation_value_pushes_live_and_persists(self):
        store = self.CCS.get_store()
        store.set_calibration("dshow:PATH_A", 5.54, name="Teslong")
        self.pg._apply_slot_rotation_value(0, 42.0)
        self.assertAlmostEqual(self.mgr.display_orientation(0)[2], 42.0)
        entry = store.get_calibration("dshow:PATH_A")
        self.assertAlmostEqual(entry["view_orientation"]["rotation_deg"], 42.0)
        # The MEASURED rotation is untouched — a hand-typed viewing angle must
        # not move the angle the mosaic orients tiles by.
        self.assertIsNone(entry.get("rotation_deg"))
        self.assertIsNone(self.mgr.get_rotation_deg(0))

    def test_one_control_does_not_clear_the_others(self):
        """The view is stored as one triple, so a partial commit must preserve
        the fields it does not name."""
        self.pg._apply_slot_mirror(0, True)
        self.pg._apply_slot_rotation_value(0, 90.0)
        self.pg._apply_slot_flip_y(0, True)
        self.assertEqual(self.mgr.display_orientation(0), (True, True, 90.0))

    def test_flip_y_persists_per_identity(self):
        self.pg._apply_slot_flip_y(1, True)
        ident = self.mgr.camera_identity(1)
        self.assertTrue(ident and ident[0])
        vo = self.CCS.get_store().get_view_orientation(ident[0])
        self.assertTrue(vo and vo["flip_y"])

    def test_view_orientation_restores_on_slot_assignment(self):
        self.pg._apply_slot_mirror(1, True)
        self.CCS._store = self.CCS.CameraCalibrationStore(self.tmp)
        from gui.widgets.camera_manager import CameraManager
        mgr2 = CameraManager()
        self.pg._camera_manager = mgr2
        mgr2._ds_cameras = list(self.DS)
        mgr2.get_source = lambda i: ("opencv", i)
        combo = self.pg._live_cam_source_combos[1]
        combo.blockSignals(True)
        combo.addItem("cam1", ("opencv", 1))
        combo.setCurrentIndex(combo.count() - 1)
        combo.blockSignals(False)
        self.assertFalse(mgr2.has_display_orientation(1))
        self.pg._restore_calibration_for_slot(1)
        self.assertTrue(mgr2.has_display_orientation(1))
        self.assertTrue(mgr2.display_orientation(1)[0])

    def test_restore_clears_a_stale_view_when_the_new_camera_has_none(self):
        """⚠ A slot's CameraWidget outlives the source assigned to it, so
        reassigning from a camera WITH a view preference to one WITHOUT must hand
        the view back to the measurement — otherwise the new camera silently
        inherits the old one's rotation. (Same hazard as the v7.16 crop restore.)
        """
        # _restore_calibration_for_slot only acts on a slot with an assigned
        # source, so assign one first (as the sibling test does).
        self.mgr.get_source = lambda i: ("opencv", i)
        combo = self.pg._live_cam_source_combos[1]
        combo.blockSignals(True)
        combo.addItem("cam1", ("opencv", 1))
        combo.setCurrentIndex(combo.count() - 1)
        combo.blockSignals(False)
        ident = self.mgr.camera_identity(1)
        self.assertTrue(ident and ident[0])
        # The newly-assigned camera has a calibration but NO view preference,
        # while the slot still carries the previous camera's view.
        self.CCS.get_store().set_calibration(ident[0], 4.0, name="Other")
        self.CCS.get_store().clear_view_orientation(ident[0])
        self.mgr.set_display_orientation(1, True, True, 180.0)
        self.assertTrue(self.mgr.has_display_orientation(1))
        self.pg._restore_calibration_for_slot(1)
        self.assertFalse(self.mgr.has_display_orientation(1))


if __name__ == "__main__":
    unittest.main()
