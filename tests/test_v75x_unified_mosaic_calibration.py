"""
tests/test_v75x_unified_mosaic_calibration.py — ONE mosaic calibration.

v7.5.x. Operator: *"We have multiple surfaces for calibrating mosaics; there
should only be ONE … All mosaic building should follow exactly the same pattern
as the calibrated settings. Rosette, full plate overview, and fluorescence
workflow should all have the exact same behavior. Currently everything is very
messed up."*

Covers, per the five deviation types the operator enumerated:

* (1)(2)(3) orientation — one measured ``(rotation_deg, flip_x, flip_y)`` triple
  resolved store-first and reaching the builder that places the tiles.
* (4) pixel size vs resolution — the rescale, and the REFUSAL to guess when the
  measurement resolution is unknown; plus the ``CameraCalibrationStore`` v1.1→1.2
  backfill that recovers the stamp for existing machines.
* (5) overlap — one shared value, clamped, with the too-low advisory.

Plus the invariants that make the pipelines identical:
* ``build_mosaic_builder`` produces the SAME builder arguments for every pipeline.
* the retired ``fov_um`` / ``spacing_um`` / learned-value overrides are gone from
  the precedence.
* the whole-mosaic output rotation NEVER reaches the builder (display-only).
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import SupportClasses.CameraCalibrationStore as CCS
from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
from SupportClasses.MosaicCalibration import (
    MosaicCalibration, SCAN_DEFAULTS, RECOMMENDED_OVERLAP_FRAC,
    build_mosaic_builder, refuse_reason, resolve, warnings_for,
)


# ── Fakes ────────────────────────────────────────────────────────────────────

class _Mgr:
    """Minimal CameraManager stand-in."""

    def __init__(self, um=0.0, res=None, rot=None, mir=False, fy=False,
                 identity="id_scope", calibrated=True):
        self._um, self._res = um, res
        self._rot, self._mir, self._fy = rot, mir, fy
        self._identity = identity
        self._calibrated = calibrated

    def camera_identity(self, i):
        return (self._identity, "Scope") if self._identity else None

    def get_um_per_px(self, i):
        return self._um

    def get_um_per_px_resolution(self, i):
        return self._res

    def is_um_per_px_calibrated(self, i):
        return self._calibrated

    def get_rotation_deg(self, i):
        return self._rot

    def get_mirrored(self, i):
        return self._mir

    def get_flip_y(self, i):
        return self._fy


class _ObjStore:
    def __init__(self, cals=None):
        self._cals = cals or {}

    def get_calibration(self, cam, obj):
        return self._cals.get((cam, obj))


class _CalStore:
    def __init__(self, entry=None, res=None, out_rot=0.0):
        self._entry, self._res, self._out = entry, res, out_rot

    def get_calibration(self, ident):
        return self._entry

    def get_um_per_px_resolution(self, ident):
        return self._res

    def get_mosaic_output_rotation(self, ident):
        return self._out


# The real machine's microscope: Andor Zyla, 4x @ 1024x1024, mounted 180 + flipY.
#
# v7.16: the objective store is keyed by the camera's DEVICE IDENTITY (what
# ``_Mgr.camera_identity`` reports), not the configured spec name. See
# ``MosaicCalibration.objective_camera_key`` — keying by name let two physical
# cameras share one calibration block. ``camera_name=`` is still passed by
# every caller and is still the key when there is no identity at all.
_ANDOR_OBJ = {("id_scope", "4x"): {
    "measured_um_per_px": 3.227061, "resolution": [1024, 1024]}}
_ANDOR_CAM = {"um_per_px": 3.227061, "rotation_deg": 180.0, "flip_y": True}


def _resolve_andor(live=(1024, 1024), scan=None):
    return resolve(
        camera_manager=_Mgr(um=3.227061, res=(1024, 1024)), cam_idx=0,
        camera_name="Andor Zyla 4.2P (USB3)", objective="4x",
        live_resolution=live, scan_settings=scan,
        cal_store=_CalStore(_ANDOR_CAM, (1024, 1024)),
        obj_store=_ObjStore(_ANDOR_OBJ))


# ── (4) pixel size vs capture resolution ─────────────────────────────────────

class TestScaleResolution(unittest.TestCase):
    """Deviation (4): 'the current camera resolution doesn't set the correct
    real-world distance'."""

    def test_uses_objective_calibration_and_records_provenance(self):
        cal = _resolve_andor()
        self.assertAlmostEqual(cal.um_per_px, 3.227061, places=6)
        self.assertEqual(cal.calib_resolution, (1024, 1024))
        self.assertIn("objective calibration", cal.provenance["um_per_px"])
        self.assertTrue(cal.scale_is_calibrated)
        self.assertFalse(cal.was_rescaled)

    def test_rescales_when_live_resolution_differs(self):
        """The whole point of the stamp: 3.227 @1024 must halve at 2048."""
        cal = _resolve_andor(live=(2048, 2048))
        self.assertAlmostEqual(cal.um_per_px, 3.227061 / 2.0, places=6)
        self.assertAlmostEqual(cal.base_um_per_px, 3.227061, places=6)
        self.assertTrue(cal.was_rescaled)
        # FOV is therefore unchanged in microns — same optics, more pixels.
        self.assertAlmostEqual(cal.fov_um[0], 3.227061 * 1024, places=3)

    def test_unstamped_value_is_refused_not_guessed(self):
        """An unstamped µm/px must NOT be silently assumed valid at the live
        width — that is exactly how a restored camera scaled its mosaic wrong
        with no visible symptom."""
        cal = resolve(
            camera_manager=_Mgr(um=3.227061, res=None), cam_idx=0,
            live_resolution=(2048, 2048), cal_store=_CalStore({}, None),
            obj_store=_ObjStore())
        self.assertIsNone(cal.calib_resolution)
        self.assertFalse(cal.scale_is_calibrated)
        self.assertIn("UNSTAMPED", cal.provenance["um_per_px_resolution"])
        self.assertIsNotNone(refuse_reason(cal))
        # And it did NOT invent a rescale.
        self.assertAlmostEqual(cal.um_per_px, 3.227061, places=6)

    def test_uncalibrated_slot_is_not_mistaken_for_a_measurement(self):
        """An un-calibrated slot still carries a seed default; treating it as a
        measurement would let a mosaic scan at a wrong scale silently."""
        cal = resolve(
            camera_manager=_Mgr(um=1.67, res=None, calibrated=False),
            cam_idx=0, live_resolution=(1024, 1024),
            cal_store=_CalStore({}, None), obj_store=_ObjStore())
        self.assertEqual(cal.um_per_px, 0.0)
        self.assertEqual(cal.provenance["um_per_px"], "UNCALIBRATED")
        self.assertIsNotNone(refuse_reason(cal))

    def test_objective_calibration_beats_the_live_manager(self):
        cal = resolve(
            camera_manager=_Mgr(um=9.99, res=(1024, 1024)), cam_idx=0,
            camera_name="Andor Zyla 4.2P (USB3)", objective="4x",
            live_resolution=(1024, 1024),
            cal_store=_CalStore({}, None), obj_store=_ObjStore(_ANDOR_OBJ))
        self.assertAlmostEqual(cal.um_per_px, 3.227061, places=6)


# ── (1)(2)(3) orientation ────────────────────────────────────────────────────

class TestOrientation(unittest.TestCase):
    """Deviations (1) mirrored, (2) stage vs pixel axis direction, (3) rotation
    about +Z — ONE measured triple, not three independent guesses."""

    def test_store_is_ground_truth(self):
        cal = _resolve_andor()
        self.assertAlmostEqual(cal.rotation_deg, 180.0)
        self.assertTrue(cal.flip_y)
        self.assertFalse(cal.flip_x)
        self.assertEqual(cal.provenance["rotation_deg"], "camera store")

    def test_live_manager_fills_fields_the_store_lacks(self):
        """Per-FIELD precedence: a camera with a stored rotation but a live-only
        flip must get both."""
        cal = resolve(
            camera_manager=_Mgr(um=1.0, res=(100, 100), rot=42.0, fy=True),
            cam_idx=0, live_resolution=(100, 100),
            cal_store=_CalStore({"rotation_deg": 180.0}, (100, 100)),
            obj_store=_ObjStore())
        self.assertAlmostEqual(cal.rotation_deg, 180.0)   # store wins
        self.assertTrue(cal.flip_y)                       # manager fills
        self.assertEqual(cal.provenance["flip_y"], "live camera manager")

    def test_unmeasured_orientation_is_neutral_and_advertised(self):
        cal = resolve(
            camera_manager=_Mgr(um=1.0, res=(100, 100)), cam_idx=0,
            live_resolution=(100, 100), cal_store=_CalStore({}, (100, 100)),
            obj_store=_ObjStore())
        self.assertEqual(cal.rotation_deg, 0.0)
        self.assertFalse(cal.flip_x)
        self.assertFalse(cal.flip_y)
        self.assertIn("unmeasured", cal.provenance["rotation_deg"])
        self.assertTrue(any("never been measured" in w
                            for w in warnings_for(cal)))

    def test_orientation_reaches_the_builder(self):
        """The fluorescence regression: measuring it is useless if it doesn't
        reach the object that places the tiles."""
        cal = _resolve_andor()
        b = build_mosaic_builder(cal)
        self.assertAlmostEqual(b._frame_rotation_deg, 180.0)
        self.assertFalse(b._frame_mirrored)
        self.assertTrue(b._frame_flip_y)


# ── (5) overlap ──────────────────────────────────────────────────────────────

class TestOverlap(unittest.TestCase):
    def test_default_is_the_recommended_value(self):
        cal = _resolve_andor()
        self.assertAlmostEqual(cal.overlap_frac, RECOMMENDED_OVERLAP_FRAC)
        self.assertEqual(SCAN_DEFAULTS["overlap_pct"], 25)

    def test_spacing_follows_fov_and_overlap(self):
        cal = _resolve_andor(scan={"overlap_pct": 25})
        fov_w = cal.fov_um[0]
        self.assertAlmostEqual(cal.spacing_um[0], fov_w * 0.75, places=3)

    def test_low_overlap_warns_but_does_not_block(self):
        """The operator's machine was on 5 %: low enough that a small µm/px error
        opens gaps and registration has too little texture."""
        cal = _resolve_andor(scan={"overlap_pct": 5})
        self.assertAlmostEqual(cal.overlap_frac, 0.05)
        self.assertIsNone(refuse_reason(cal))            # advisory, not blocking
        self.assertTrue(any("overlap" in w.lower() for w in warnings_for(cal)))

    def test_overlap_is_clamped_to_a_sane_band(self):
        self.assertGreaterEqual(
            _resolve_andor(scan={"overlap_pct": -50}).overlap_frac, 0.05)
        self.assertLessEqual(
            _resolve_andor(scan={"overlap_pct": 500}).overlap_frac, 0.60)

    def test_low_settle_warns(self):
        """settle_ms was 2 on the real machine (default 300) — tiles can be
        grabbed before the stage stops."""
        cal = _resolve_andor(scan={"settle_ms": 2})
        self.assertTrue(any("settle" in w.lower() for w in warnings_for(cal)))


# ── The retired overrides ────────────────────────────────────────────────────

class TestRetiredOverrides(unittest.TestCase):
    def test_fov_um_and_spacing_um_are_not_in_the_precedence(self):
        """They used to beat every measured value, forever, from a hidden
        Advanced submenu."""
        for key in ("fov_um", "spacing_um", "frame_orient"):
            self.assertNotIn(key, SCAN_DEFAULTS)
        cal = _resolve_andor(scan={"fov_um": 99999, "spacing_um": 1500,
                                   "frame_orient": "rot180"})
        # Still the measured value, and still the measured orientation.
        self.assertAlmostEqual(cal.um_per_px, 3.227061, places=6)
        self.assertAlmostEqual(cal.rotation_deg, 180.0)
        self.assertTrue(cal.flip_y)


# ── Pipelines identical by construction ──────────────────────────────────────

class TestBuilderFactory(unittest.TestCase):
    def test_every_pipeline_gets_the_same_builder_arguments(self):
        """Full-plate, rosette, calibration-preview and fluorescence previously
        disagreed on target_mosaic_px (3000/1800/2500/2000), registration method
        and whether retain_for_reorient was set at all."""
        cal = _resolve_andor()
        a = build_mosaic_builder(cal, retain_for_reorient=True)
        b = build_mosaic_builder(cal, retain_for_reorient=True)
        for attr in ("_um_per_px", "_overlap", "_target_mosaic_px",
                     "_registration_method", "_frame_rotation_deg",
                     "_frame_mirrored", "_frame_flip_y", "_frame_size_px"):
            self.assertEqual(getattr(a, attr), getattr(b, attr), attr)
        self.assertEqual(a._target_mosaic_px, SCAN_DEFAULTS["target_px"])
        self.assertEqual(a._registration_method, "fourier_mellin")

    def test_retain_for_reorient_enables_optimize_registration(self):
        """Without it the caller's optimize_registration silently does nothing —
        which is why the calibration dialog stitched more weakly than the scan."""
        cal = _resolve_andor()
        self.assertTrue(
            build_mosaic_builder(cal, retain_for_reorient=True)
            ._retain_for_reorient)
        self.assertFalse(build_mosaic_builder(cal)._retain_for_reorient)

    def test_output_rotation_never_reaches_the_builder(self):
        """DISPLAY-ONLY. Baking it into the composite/extent would break the
        back-projection every consumer uses to derive well centres for MOTION."""
        cal = MosaicCalibration(
            um_per_px=1.0, live_resolution=(100, 100),
            output_rotation_deg=90.0)
        b = build_mosaic_builder(cal)
        self.assertAlmostEqual(b._frame_rotation_deg, 0.0)
        for attr in vars(b):
            self.assertNotIn("output_rotation", attr)


# ── Store: the v1.1 -> v1.2 resolution backfill ──────────────────────────────

class TestStoreResolutionStamp(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.path = self.dir / "cams.json"

    def _write(self, data):
        self.path.write_text(json.dumps(data), encoding="utf-8")
        return self.path

    def test_round_trip_and_travels_with_the_value(self):
        st = CameraCalibrationStore(self.path)
        st.set_calibration("id", 3.227061, um_per_px_resolution=(1024, 1024))
        self.assertEqual(st.get_um_per_px_resolution("id"), (1024, 1024))
        # A NEW value measured at an unknown resolution must CLEAR the old stamp,
        # never inherit it — a fresh value paired with a stale resolution would
        # rescale confidently and wrongly.
        st.set_calibration("id", 1.5)
        self.assertIsNone(st.get_um_per_px_resolution("id"))

    def test_backfill_from_matching_objective_calibration(self):
        """The real recovery path for this machine: the Andor's 3.227061 matches
        its 4x objective entry, which HAS always recorded 1024x1024."""
        import SupportClasses.ObjectiveCalibration as oc

        class _Objs:
            def all_calibrations(self):
                return {"Andor Zyla 4.2P (USB3)": {
                    "4x": {"measured_um_per_px": 3.227061,
                           "resolution": [1024, 1024]}}}

        orig = oc.get_store
        oc.get_store = lambda *a, **k: _Objs()
        try:
            p = self._write({"version": "1.1", "cameras": {
                "andor": {"um_per_px": 3.227061, "name": "Andor Zyla"}}})
            st = CameraCalibrationStore(p)
            self.assertEqual(st.get_um_per_px_resolution("andor"), (1024, 1024))
            self.assertEqual(st._data["version"], CCS.SCHEMA_VERSION)
        finally:
            oc.get_store = orig

    def test_backfill_falls_back_to_hw_controls_resolution(self):
        import SupportClasses.ObjectiveCalibration as oc
        orig = oc.get_store
        oc.get_store = lambda *a, **k: type(
            "E", (), {"all_calibrations": lambda s: {}})()
        try:
            p = self._write({"version": "1.1", "cameras": {
                "cam": {"um_per_px": 5.7, "name": "Teslong",
                        "hw_controls": {"resolution": [640, 480]}}}})
            st = CameraCalibrationStore(p)
            self.assertEqual(st.get_um_per_px_resolution("cam"), (640, 480))
        finally:
            oc.get_store = orig

    def test_backfill_leaves_unknowable_entries_absent(self):
        """Absent must mean UNKNOWN. Substituting a guess here would defeat the
        purpose of the stamp."""
        import SupportClasses.ObjectiveCalibration as oc
        orig = oc.get_store
        oc.get_store = lambda *a, **k: type(
            "E", (), {"all_calibrations": lambda s: {}})()
        try:
            p = self._write({"version": "1.1", "cameras": {
                "cam": {"um_per_px": 5.27, "name": "Teslong"}}})
            st = CameraCalibrationStore(p)
            self.assertIsNone(st.get_um_per_px_resolution("cam"))
            self.assertNotIn(
                "um_per_px_resolution", st.get_calibration("cam"))
        finally:
            oc.get_store = orig

    def test_migration_is_idempotent(self):
        import SupportClasses.ObjectiveCalibration as oc
        orig = oc.get_store
        oc.get_store = lambda *a, **k: type(
            "E", (), {"all_calibrations": lambda s: {}})()
        try:
            p = self._write({"version": "1.1", "cameras": {
                "cam": {"um_per_px": 5.7,
                        "hw_controls": {"resolution": [640, 480]}}}})
            CameraCalibrationStore(p)
            first = json.loads(p.read_text(encoding="utf-8"))
            CameraCalibrationStore(p)
            self.assertEqual(first, json.loads(p.read_text(encoding="utf-8")))
        finally:
            oc.get_store = orig

    def test_ambiguous_value_match_is_rejected(self):
        """Two objectives sharing a µm/px at DIFFERENT resolutions is not
        evidence; fall through rather than pick one."""
        import SupportClasses.ObjectiveCalibration as oc

        class _Objs:
            def all_calibrations(self):
                return {"C": {"2x": {"measured_um_per_px": 1.0,
                                     "resolution": [916, 686]},
                              "4x": {"measured_um_per_px": 1.0,
                                     "resolution": [2048, 2048]}}}

        orig = oc.get_store
        oc.get_store = lambda *a, **k: _Objs()
        try:
            p = self._write({"version": "1.1",
                             "cameras": {"cam": {"um_per_px": 1.0}}})
            st = CameraCalibrationStore(p)
            self.assertIsNone(st.get_um_per_px_resolution("cam"))
        finally:
            oc.get_store = orig


# ── Display-only whole-mosaic output rotation ────────────────────────────────

class TestOutputRotation(unittest.TestCase):
    def setUp(self):
        self.path = Path(tempfile.mkdtemp()) / "cams.json"

    def test_quantised_and_zero_pops_the_key(self):
        st = CameraCalibrationStore(self.path)
        st.set_mosaic_output_rotation("id", 100.0)      # snaps to 90
        self.assertAlmostEqual(st.get_mosaic_output_rotation("id"), 90.0)
        st.set_mosaic_output_rotation("id", 360.0)      # wraps to 0 -> popped
        self.assertAlmostEqual(st.get_mosaic_output_rotation("id"), 0.0)
        self.assertNotIn(
            "mosaic_output_rotation_deg", st.get_calibration("id") or {})

    def test_resolved_onto_the_calibration(self):
        cal = resolve(
            camera_manager=_Mgr(um=1.0, res=(10, 10)), cam_idx=0,
            live_resolution=(10, 10),
            cal_store=_CalStore({"um_per_px": 1.0}, (10, 10), out_rot=180.0),
            obj_store=_ObjStore())
        self.assertAlmostEqual(cal.output_rotation_deg, 180.0)


class TestOutputRotationIsDisplayOnly(unittest.TestCase):
    """The safety invariant: rotating the OUTPUT must not disturb the stored
    extent, because every consumer back-projects it to drive the stage."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication([])

    def _view(self):
        import numpy as np
        from gui.widgets.jog_workspace_view import (
            JogWorkspaceView, pixmap_from_bgr)
        from SupportClasses.SafetyLimits import SafetyLimits
        v = JogWorkspaceView()
        sl = SafetyLimits()
        sl.xy_min_x, sl.xy_max_x = 0, 20000
        sl.xy_min_y, sl.xy_max_y = 0, 16000
        if hasattr(v, "set_safety_limits"):
            v.set_safety_limits(sl)
        v.resize(600, 480)
        img = np.zeros((80, 100, 3), dtype=np.uint8)
        img[0, 0] = (255, 255, 255)
        v.set_mosaic_overlay(pixmap_from_bgr(img), (0.0, 0.0, 20000.0, 16000.0))
        v.set_mosaic_visible(True)
        return v

    def test_extent_is_identical_at_every_quadrant(self):
        v = self._view()
        base = v._mosaic_extent_abs
        for rot in (0, 90, 180, 270):
            v.set_mosaic_output_rotation(rot)
            self.assertEqual(v.mosaic_output_rotation(), rot)
            self.assertEqual(v._mosaic_extent_abs, base)

    def test_rotation_is_quantised_and_wraps(self):
        v = self._view()
        v.set_mosaic_output_rotation(100.0)
        self.assertEqual(v.mosaic_output_rotation(), 90)
        v.set_mosaic_output_rotation(360.0)
        self.assertEqual(v.mosaic_output_rotation(), 0)
        v.set_mosaic_output_rotation(-90.0)
        self.assertEqual(v.mosaic_output_rotation(), 270)

    def test_changing_rotation_invalidates_the_render_cache(self):
        """The scaled pixmap is cached; a stale cache would ignore the rotation."""
        v = self._view()
        v.grab()                                  # first paint builds the cache
        self.assertIsNotNone(v._mosaic_cache_key)
        v.set_mosaic_output_rotation(90)
        self.assertIsNone(v._mosaic_cache_key)
        # And it rebuilds keyed on the new rotation.
        v.grab()
        self.assertIsNotNone(v._mosaic_cache_key)
        self.assertIn(90, v._mosaic_cache_key)

    def test_registration_view_composes_flip_and_rotation(self):
        from gui.widgets.mosaic_registration_view import MosaicRegistrationView
        v = MosaicRegistrationView()
        self.assertEqual(v.output_rotation(), 0)
        v.set_output_rotation(270)
        self.assertEqual(v.output_rotation(), 270)
        v.set_flip_180(True)          # composes; must not raise
        v.set_output_rotation(90)
        self.assertEqual(v.output_rotation(), 90)


class TestConfirmDialogGating(unittest.TestCase):
    """Stage C: the final check, and 'Cancel writes NOTHING'."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication([])

    def _dlg(self, cal=None, runner=None):
        from gui.dialogs.mosaic_calibration_confirm_dialog import (
            MosaicCalibrationConfirmDialog)
        cal = cal or _resolve_andor(scan={"overlap_pct": 5})
        return MosaicCalibrationConfirmDialog(
            cal, test_mosaic_runner=runner,
            tile_count_getter=lambda c: 42)

    def test_operator_overlap_and_rotation_land_on_the_result(self):
        d = self._dlg()
        d._spin_overlap.setValue(25)
        d._cmb_rot.setCurrentIndex(d._cmb_rot.findData(180))
        out = d.result_calibration()
        self.assertAlmostEqual(out.overlap_frac, 0.25)
        self.assertAlmostEqual(out.output_rotation_deg, 180.0)
        # The measurement itself is carried through untouched.
        self.assertAlmostEqual(out.um_per_px, 3.227061, places=6)
        self.assertAlmostEqual(out.rotation_deg, 180.0)
        self.assertTrue(out.flip_y)

    def test_low_overlap_advisory_clears_when_raised(self):
        d = self._dlg()
        self.assertTrue(d._lbl_warn.isVisibleTo(d))
        self.assertIn("overlap", d._lbl_warn.text().lower())
        d._spin_overlap.setValue(25)
        self.assertNotIn("overlap", d._lbl_warn.text().lower())

    def test_overlap_choice_is_priced_in_tiles(self):
        """Raising overlap costs scan time; the count makes that visible."""
        d = self._dlg()
        self.assertIn("42", d._lbl_tiles.text())
        self.assertIn("step", d._lbl_tiles.text().lower())

    def test_test_mosaic_runner_receives_the_edited_candidate(self):
        seen = {}
        d = self._dlg(runner=lambda cal: seen.setdefault("cal", cal))
        d._spin_overlap.setValue(30)
        d._on_test()
        self.assertAlmostEqual(seen["cal"].overlap_frac, 0.30)
        self.assertTrue(d._tested)

    def test_dialog_persists_nothing(self):
        """The gating promise: constructing / editing / cancelling must not touch
        either store. Previously a measurement was committed the moment it
        finished, and the mirror checkbox persisted on its first click."""
        import SupportClasses.CameraCalibrationStore as ccs
        import SupportClasses.ObjectiveCalibration as oc
        writes = []

        class _Spy:
            def __getattr__(self, name):
                if name.startswith("set_"):
                    return lambda *a, **k: writes.append(name)
                raise AttributeError(name)

        c_orig, o_orig = ccs.get_store, oc.get_store
        ccs.get_store = lambda *a, **k: _Spy()
        oc.get_store = lambda *a, **k: _Spy()
        try:
            d = self._dlg(runner=lambda cal: None)
            d._spin_overlap.setValue(40)
            d._cmb_rot.setCurrentIndex(d._cmb_rot.findData(90))
            d._on_test()
            d.reject()
            self.assertEqual(writes, [])
        finally:
            ccs.get_store, oc.get_store = c_orig, o_orig


class TestMirrorCheckboxNoLongerPersists(unittest.TestCase):
    """The two dialogs that used to write the mirror on the FIRST CLICK — even if
    the operator then pressed Cancel — must now only flip the preview."""

    def _run_toggle(self, cls):
        """Invoke the handler unbound against a minimal stand-in (the dialogs are
        too heavy to construct headless) with both stores spied on."""
        import SupportClasses.CameraCalibrationStore as ccs
        writes = []

        class _Feed:
            def __init__(self):
                self.oriented = []

            def set_view_orientation(self, mir, rot):
                self.oriented.append((mir, rot))

        class _Mgr2:
            def set_mirrored(self, i, v):
                writes.append(("manager", v))

            def camera_identity(self, i):
                return ("id", "Cam")

        class _Spy:
            def set_mirrored(self, *a, **k):
                writes.append(("store", a))

        stub = type("S", (), {})()
        stub._mgr = stub._camera_manager = _Mgr2()
        stub._cam_idx = 0
        stub._view_rot = 0.0
        stub._view_mir = False
        stub._feed = _Feed()
        orig = ccs.get_store
        ccs.get_store = lambda *a, **k: _Spy()
        try:
            cls._on_mirror_toggled(stub, True)
        finally:
            ccs.get_store = orig
        return writes, stub

    def test_scale_fov_mirror_toggle_is_preview_only(self):
        from gui.dialogs.scale_fov_calibration_dialog import (
            ScaleFovCalibrationDialog)
        writes, stub = self._run_toggle(ScaleFovCalibrationDialog)
        self.assertEqual(writes, [])                     # nothing persisted/pushed
        self.assertEqual(stub._feed.oriented, [(True, 0.0)])   # preview flipped
        self.assertTrue(stub._view_mir)

    def test_pixel_calibration_mirror_toggle_is_preview_only(self):
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        writes, stub = self._run_toggle(PixelCalibrationDialog)
        self.assertEqual(writes, [])
        self.assertEqual(stub._feed.oriented, [(True, 0.0)])


if __name__ == "__main__":
    unittest.main()
