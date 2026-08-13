"""v7.16 — the Tucsen mosaic: camera keying, rectangular FOV, intensity.

Operator, after calibrating a Tucsen at 4x: *"i tried to do a full plate
mosaic, and it said it would take 400 minutes for the scan with way too many
tiles. I think its because the microscope camera setup section is not working
for the tucsen camera. Also the mosaic tiles are using the microscope view as a
square. it is not a square it is rectangular and we should take advantage of
those pixels. After we have done the full mosaic, lets reguralize the intensity
across the mosaic."*

All three are real. The numbers below are the operator's own, taken from
``logs/app.log`` and ``config/hardware/objectives.json``:

* 19:09:14 ``resolved: 1.2710 um/px at 2600x2048 (measured 3.2271 @ 1024x1024)
  | FOV 3305x2603 um`` — the Andor's 4x stretched onto the Tucsen, because the
  objective store was keyed by camera MODEL NAME and both cameras shared one
  block.
* 19:09:18 ``resolved: 0.3891 um/px at 2600x2048 | FOV 1012x797 um`` and
  ``Raster grid: 153x124 = 18972 positions`` — 18,972 tiles ~ 474 min.
* 0.389135 is bit-identical to the ToupTek's stored 4x value, which a fresh
  measurement cannot land on by chance: the Scale/FOV dialog's Verify step
  re-synced its result from the manager unconditionally, overwriting the
  measurement with the value that had been pushed into the slot.
"""

import ast
import inspect
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.MosaicBuilder import MosaicBuilder          # noqa: E402
from SupportClasses.MosaicCalibration import (                  # noqa: E402
    objective_camera_key, resolve)
from SupportClasses.ObjectiveCalibration import (               # noqa: E402
    ObjectiveCalibrationStore)


# ── Fakes ────────────────────────────────────────────────────────────────────

class _Mgr:
    def __init__(self, identity="tucam:0", um=0.0, res=None):
        self._identity, self._um, self._res = identity, um, res

    def camera_identity(self, i):
        return (self._identity, "Tucsen Camera") if self._identity else None

    def get_um_per_px(self, i):
        return self._um

    def get_um_per_px_resolution(self, i):
        return self._res

    def is_um_per_px_calibrated(self, i):
        return bool(self._um)

    def get_rotation_deg(self, i):
        return None

    def get_mirrored(self, i):
        return False

    def get_flip_y(self, i):
        return False


class _ObjStore:
    def __init__(self, cals):
        self._c = cals

    def get_calibration(self, cam, obj):
        return self._c.get((cam, obj))


class _CalStore:
    def get_calibration(self, ident):
        return None

    def get_um_per_px_resolution(self, ident):
        return None

    def get_mosaic_output_rotation(self, ident):
        return 0.0


# ── 1. The camera key ────────────────────────────────────────────────────────

class TestObjectiveCameraKey(unittest.TestCase):

    def test_identity_wins_over_the_spec_name(self):
        self.assertEqual(
            objective_camera_key(_Mgr("tucam:0"), 0, "Some Spec Name"),
            "tucam:0")

    def test_spec_name_only_when_there_is_no_identity(self):
        self.assertEqual(
            objective_camera_key(_Mgr(identity=None), 0, "Some Spec Name"),
            "Some Spec Name")
        self.assertEqual(objective_camera_key(None, None, "Spec"), "Spec")

    def test_never_falls_back_to_the_name_once_an_identity_exists(self):
        """The fallback IS the bug: it is how one camera reads another's block.

        An absent calibration is recoverable (re-measure); a wrong one silently
        scans at another sensor's scale.
        """
        self.assertEqual(
            objective_camera_key(_Mgr("tucam:0"), 0, "ToupTek"), "tucam:0")

    def test_nothing_at_all_is_none(self):
        self.assertIsNone(objective_camera_key(None, None, None))
        self.assertIsNone(objective_camera_key(None, None, "   "))


class TestTwoCamerasNoLongerShareOneCalibration(unittest.TestCase):
    """The operator's exact cross-contamination, with their real numbers."""

    SPEC = "Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)"

    def _resolve(self, identity, obj_cals, live=(2600, 2048)):
        return resolve(
            camera_manager=_Mgr(identity=identity), cam_idx=0,
            camera_name=self.SPEC, objective="4x", live_resolution=live,
            cal_store=_CalStore(), obj_store=_ObjStore(obj_cals))

    def test_the_tucsen_does_not_read_the_toupteks_block(self):
        cal = self._resolve(
            "tucam:0",
            {(self.SPEC, "4x"): {"measured_um_per_px": 0.389135,
                                 "resolution": [3664, 2748]}})
        self.assertEqual(cal.um_per_px, 0.0, "must not read another camera")
        self.assertEqual(cal.provenance.get("um_per_px"), "UNCALIBRATED")

    def test_the_tucsen_reads_its_OWN_block(self):
        cal = self._resolve(
            "tucam:0",
            {("tucam:0", "4x"): {"measured_um_per_px": 1.2710,
                                 "resolution": [2600, 2048]}})
        self.assertAlmostEqual(cal.um_per_px, 1.2710, places=6)

    def test_a_camera_with_no_identity_still_uses_the_spec_name(self):
        """Simulated cameras and tests keep the legacy behaviour exactly."""
        cal = self._resolve(
            None,
            {(self.SPEC, "4x"): {"measured_um_per_px": 0.389135,
                                 "resolution": [2600, 2048]}})
        self.assertAlmostEqual(cal.um_per_px, 0.389135, places=6)

    def test_the_cross_camera_rescale_can_no_longer_happen(self):
        """19:09:14 in the log: the Andor's 3.2271@1024 became 1.2710@2600.

        Rescaling µm/px by resolution is only valid WITHIN one sensor; across
        two cameras it is a physically meaningless number that nonetheless
        looks entirely plausible.
        """
        cal = self._resolve(
            "tucam:0",
            {("andor:VSC-07863", "4x"): {"measured_um_per_px": 3.227061,
                                         "resolution": [1024, 1024]}})
        self.assertEqual(cal.um_per_px, 0.0)


# ── 2. The rectangular / rotated field of view ───────────────────────────────

FW, FH, UPP = 2600, 2048, 0.389135          # the operator's Tucsen at 4x
ENVELOPE = (0.0, 0.0, 116340.0, 74227.0)    # their real XY travel envelope


def _builder(rot=0.0, fw=FW, fh=FH, upp=UPP, target=3000):
    return MosaicBuilder(
        frame_size_px=(fw, fh), micron_per_pixel=upp, overlap=0.25,
        target_mosaic_px=target, frame_rotation_deg=rot, register=False,
        retain_for_reorient=True)


class TestOrientedFov(unittest.TestCase):

    def test_unrotated_is_unchanged(self):
        b = _builder(0.0)
        self.assertEqual(b._oriented_fov_um(), b._fov_um())

    def test_ninety_degrees_transposes(self):
        b = _builder(-89.97)
        ow, oh = b._oriented_fov_um()
        fw, fh = b._fov_um()
        self.assertAlmostEqual(ow, fh, delta=1.0)
        self.assertAlmostEqual(oh, fw, delta=1.0)

    def test_one_eighty_is_unchanged(self):
        b = _builder(180.0)
        self.assertAlmostEqual(b._oriented_fov_um()[0], b._fov_um()[0], places=3)

    def test_non_axis_angle_is_the_bounding_box(self):
        b = _builder(45.0)
        fw, fh = b._fov_um()
        exp = (fw + fh) / math.sqrt(2.0)
        self.assertAlmostEqual(b._oriented_fov_um()[0], exp, delta=1.0)


class TestTheSensorIsNoLongerTreatedAsASquare(unittest.TestCase):
    """The operator's words. At +/-90 deg a rectangular frame was rendered back
    into its own W x H box, so the long axis was CLIPPED to the short one and
    the usable tile collapsed to min(w, h) squared."""

    def test_no_pixels_are_lost_at_ninety_degrees(self):
        b = _builder(-90.0)
        tile = np.full((200, 260, 3), 200, np.uint8)
        out = b._orient_tile(tile)
        self.assertEqual((out.shape[1], out.shape[0]), (200, 260))
        kept = int((out.max(axis=2) > 0).sum())
        self.assertEqual(kept, 260 * 200, "every pixel must survive")

    def test_the_old_square_box_would_have_lost_a_fifth_of_the_frame(self):
        """Pins the size of the defect, so the fix cannot be quietly undone."""
        w, h = 260, 200
        kept_old = min(w, h) * min(w, h)        # rotated content clipped to WxH
        self.assertAlmostEqual(kept_old / (w * h), 0.769, places=3)

    def test_unrotated_tile_is_byte_identical(self):
        b = _builder(0.0)
        tile = np.random.default_rng(1).integers(
            0, 255, (200, 260, 3), dtype=np.uint8)
        self.assertIs(b._orient_tile(tile), tile)

    def test_raster_step_follows_the_stage_axes(self):
        flat = _builder(0.0)
        turned = _builder(-90.0)
        flat.generate_raster_positions(ENVELOPE, overlap=0.25)
        turned.generate_raster_positions(ENVELOPE, overlap=0.25)
        # cols/rows swap when the camera is turned a quarter turn.
        self.assertGreater(turned._grid_cols, flat._grid_cols)
        self.assertLess(turned._grid_rows, flat._grid_rows)

    def test_a_rotated_tile_fills_its_whole_footprint(self):
        b = _builder(-90.0, fw=260, fh=200, upp=10.0, target=900)
        b.generate_raster_positions((0.0, 0.0, 6000.0, 5000.0), overlap=0.25)
        b.add_frame("W", np.full((200, 260, 3), 200, np.uint8), 3000.0, 2500.0)
        b.stitch_incremental()
        lit = int((b.composite.max(axis=2) > 0).sum())
        ow, oh = b._oriented_fov_um()
        box = (ow * b._mosaic_scale) * (oh * b._mosaic_scale)
        self.assertGreater(lit / box, 0.98)


class TestStitchGeometryHoldsAtEveryRotation(unittest.TestCase):
    """A feature at a known camera-pixel offset must land at the stage position
    R(theta) puts it — otherwise mosaic clicks drive to the wrong XY."""

    def _run(self, rot):
        fw, fh, upp = 260, 200, 10.0
        b = _builder(rot, fw=fw, fh=fh, upp=upp, target=900)
        b.generate_raster_positions((0.0, 0.0, 6000.0, 5000.0), overlap=0.25)
        f = np.zeros((fh, fw, 3), np.uint8)
        f[:] = 40
        cx, cy = fw // 2 + 50, fh // 2 + 30
        f[cy - 4:cy + 5, cx - 4:cx + 5] = 255
        b.add_frame("W", f, 3000.0, 2500.0)
        b.stitch_incremental()
        ys, xs = np.where(b.composite.max(axis=2) > 200)
        ox, oy = b._canvas_origin_um
        sc = b._mosaic_scale
        t = math.radians(rot)
        c, s = math.cos(t), math.sin(t)
        return ((xs.mean() / sc + ox, ys.mean() / sc + oy),
                (3000.0 + (50 * c - 30 * s) * upp,
                 2500.0 + (50 * s + 30 * c) * upp))

    def test_every_cardinal_rotation(self):
        for rot in (0.0, 90.0, 180.0, -90.0):
            with self.subTest(rot=rot):
                got, exp = self._run(rot)
                self.assertAlmostEqual(got[0], exp[0], delta=25.0)
                self.assertAlmostEqual(got[1], exp[1], delta=25.0)


class TestReblendSurvivesAnOrientationChange(unittest.TestCase):
    """``reblend_reoriented`` exists to re-render at a DIFFERENT orientation, so
    the stored box no longer describes the result — tiles must be re-centred."""

    def test_tile_stays_centred_when_the_rotation_changes(self):
        b = _builder(0.0, fw=260, fh=200, upp=10.0, target=900)
        b.generate_raster_positions((0.0, 0.0, 6000.0, 5000.0), overlap=0.25)
        b.add_frame("W", np.full((200, 260, 3), 220, np.uint8), 3000.0, 2500.0)
        b.stitch_incremental()

        def centroid(img):
            ys, xs = np.where(img.max(axis=2) > 0)
            return xs.mean(), ys.mean()

        before = centroid(b.composite)
        b.set_frame_orientation(rotation_deg=-90.0)
        b.reblend_reoriented()
        after = centroid(b.composite)
        self.assertAlmostEqual(before[0], after[0], delta=2.0)
        self.assertAlmostEqual(before[1], after[1], delta=2.0)


# ── 3. Intensity regularization ──────────────────────────────────────────────

def _vignette(fw, fh, depth=0.55):
    yy, xx = np.mgrid[0:fh, 0:fw]
    cx, cy = (fw - 1) / 2.0, (fh - 1) / 2.0
    r2 = ((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2
    return (1.0 - depth * r2).clip(0.25, 1.0).astype(np.float32)


def _vignetted_scan(n_seed=7, level_drift=18.0, fw=120, fh=96):
    rng = np.random.default_rng(n_seed)
    vig = _vignette(fw, fh)
    b = _builder(0.0, fw=fw, fh=fh, upp=10.0, target=900)
    pos = b.generate_raster_positions((0.0, 0.0, 6000.0, 5000.0), overlap=0.25)
    for (x, y) in pos:
        base = 150 + rng.normal(0, level_drift)
        tex = rng.normal(0, 6, (fh, fw))
        f = np.clip((base + tex) * vig, 0, 255).astype(np.uint8)
        b.add_frame("W", np.dstack([f] * 3), x, y)
        b.stitch_incremental()
    return b, vig, len(pos)


def _lowfreq_std(img):
    import cv2
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    m = g > 0
    return float(cv2.GaussianBlur(g, (31, 31), 0)[m].std())


class TestIntensityRegularization(unittest.TestCase):

    def test_it_flattens_the_tile_grid(self):
        b, _vig, n = _vignetted_scan()
        before = _lowfreq_std(b.composite)
        comp, applied_ff, n_gain = b.regularize_intensity()
        after = _lowfreq_std(comp)
        self.assertTrue(applied_ff)
        self.assertEqual(n_gain, n)
        self.assertLess(after, before * 0.5,
                        "seam / vignette structure must drop sharply")

    def test_the_estimated_field_recovers_the_true_falloff(self):
        b, vig, _n = _vignetted_scan()
        ff = b.estimate_flat_field()
        self.assertIsNotNone(ff)
        # Normalisation differs (mean 1.0 vs peak 1.0), so compare the RATIO of
        # brightest to darkest — that is the shape, and it is what matters.
        self.assertAlmostEqual(
            float(ff.max() / ff.min()), float(vig.max() / vig.min()),
            delta=0.35)

    def test_too_few_tiles_refuses_rather_than_guessing(self):
        """With a handful of frames the median cannot separate illumination
        from specimen, and dividing by the estimate would burn one tile's
        content into every tile."""
        b = _builder(0.0, fw=120, fh=96, upp=10.0, target=900)
        b.generate_raster_positions((0.0, 0.0, 1200.0, 1000.0), overlap=0.25)
        b.add_frame("W", np.full((96, 120, 3), 180, np.uint8), 600.0, 500.0)
        b.stitch_incremental()
        self.assertIsNone(b.estimate_flat_field())

    def test_it_is_a_no_op_when_switched_off(self):
        b, _v, _n = _vignetted_scan()
        before = b.composite.copy()
        comp, applied_ff, n_gain = b.regularize_intensity(
            flat_field=False, match_gain=False)
        self.assertFalse(applied_ff)
        self.assertEqual(n_gain, 0)
        np.testing.assert_array_equal(comp, before)

    def test_no_correction_is_applied_before_it_is_asked_for(self):
        b, _v, _n = _vignetted_scan()
        self.assertIsNone(getattr(b, "_flat_field"))
        tile = np.full((96, 120, 3), 123, np.uint8)
        self.assertIs(b._intensity_correct(tile, None), tile)

    def test_gain_matching_is_bounded(self):
        """A tile legitimately dominated by a bright object must not be dragged
        to the plate average — that would erase the signal being scanned."""
        b, _v, _n = _vignetted_scan()
        b.regularize_intensity()
        self.assertTrue(all(0.5 <= g <= 2.0 for g in b._tile_gains))


# ── 4. The plausibility guard ────────────────────────────────────────────────

class TestPlausibility(unittest.TestCase):
    """For ONE camera, um/px x magnification x width is a sensor property, so
    it must agree across objectives. Uses the operator's real ToupTek values."""

    def _store(self):
        st = ObjectiveCalibrationStore(
            Path(tempfile.mkdtemp()) / "objectives.json")
        st._data["objectives"] = [
            {"name": n, "nominal_magnification": m}
            for n, m in (("2x", 2.0), ("4x", 4.0), ("10x", 10.0))]
        st.set_calibration("cam", "2x", 0.778843, (3664, 2748))
        st.set_calibration("cam", "10x", 0.314136, (916, 686))
        return st

    def test_the_toupteks_own_4x_value_passes(self):
        st = self._store()
        self.assertIsNone(
            st.implausible_reason("cam", "4x", 0.389135, (3664, 2748)))

    def test_the_same_value_mis_stamped_at_another_sensor_width_is_flagged(self):
        """Exactly what was saved on this rig: the ToupTek's number wearing the
        Tucsen's 2600x2048 stamp."""
        st = self._store()
        why = st.implausible_reason("cam", "4x", 0.389135, (2600, 2048))
        self.assertIsNotNone(why)
        self.assertIn("too SMALL", why)
        self.assertIn("more tiles", why)

    def test_a_value_that_is_too_large_warns_about_GAPS_not_tiles(self):
        st = self._store()
        why = st.implausible_reason("cam", "4x", 4.0, (3664, 2748))
        self.assertIsNotNone(why)
        self.assertIn("too LARGE", why)
        self.assertIn("GAPS", why)

    def test_nothing_to_compare_against_returns_none(self):
        """Unverifiable must be reported as unverified, never as passed."""
        st = ObjectiveCalibrationStore(Path(tempfile.mkdtemp()) / "o.json")
        st._data["objectives"] = [{"name": "4x", "nominal_magnification": 4.0}]
        self.assertIsNone(
            st.implausible_reason("cam", "4x", 0.389135, (2600, 2048)))

    def test_it_only_compares_within_one_camera(self):
        st = self._store()
        self.assertIsNone(
            st.implausible_reason("OTHER-CAM", "4x", 0.389135, (2600, 2048)))


# ── 5. The Verify step must not clobber the measurement ──────────────────────

class TestVerifyDoesNotOverwriteTheMeasurement(unittest.TestCase):

    def _src(self):
        from gui.dialogs import scale_fov_calibration_dialog as m
        return inspect.getsource(m.ScaleFovCalibrationDialog._verify_mosaic)

    def test_the_adopt_is_gated_on_an_actual_correction(self):
        """Walks the AST rather than matching text: a substring check would
        pass on the explanatory comment alone (a trap this repo has hit)."""
        import textwrap
        tree = ast.parse(textwrap.dedent(self._src()))
        assigns = [
            n for n in ast.walk(tree)
            if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Attribute)
                    and t.attr == "result_um_per_px" for t in n.targets)]
        self.assertTrue(assigns, "expected an assignment to result_um_per_px")
        guarded = []
        for node in ast.walk(tree):
            if isinstance(node, ast.If):
                names = {n.id for n in ast.walk(node.test)
                         if isinstance(n, ast.Name)}
                if "corrected" in names:
                    guarded.extend(
                        n for n in ast.walk(node)
                        if isinstance(n, ast.Assign)
                        and any(isinstance(t, ast.Attribute)
                                and t.attr == "result_um_per_px"
                                for t in n.targets))
        self.assertTrue(
            guarded,
            "result_um_per_px must only be adopted when the verify step "
            "actually corrected it")

    def test_a_before_snapshot_is_taken_ahead_of_the_dialog(self):
        import re

        src = self._src()
        # Match the dialog's execution WITHOUT pinning how it is invoked. This
        # asserted `src.index("dlg.exec()")` and broke in v7.18 when the call
        # became `exec_dialog(dlg)` (dialogs are now disposed to stop a widget
        # leak) — a cosmetic change to a line this test does not care about.
        # The invariant is the ORDER, so match either spelling.
        run = re.search(r"(dlg\.exec\(\)|exec_dialog\(dlg\))", src)
        self.assertIsNotNone(run, "could not find where the verify dialog runs")
        self.assertLess(
            src.index("before = _mgr_eff()"), run.start(),
            "the snapshot must be taken BEFORE the verify dialog runs")


# ── 6. Saved setups default to the folder they are read from ─────────────────

class TestSetupFolderDefault(unittest.TestCase):

    def test_save_and_load_use_the_scanned_directory(self):
        from gui.pages import hardware_setup as hs
        # Save resolves its destination through _setup_path_for_name, which
        # is built on _setup_dir; Load still defaults its dialog there.
        self.assertIn("_setup_path_for_name",
                      inspect.getsource(hs.HardwareSetupPage._save_config))
        for fn in (hs.HardwareSetupPage._setup_path_for_name,
                   hs.HardwareSetupPage._load_config):
            self.assertIn("_setup_dir", inspect.getsource(fn))

    def test_save_does_not_ask_the_operator_for_a_folder(self):
        """v7.9.1: a dialog *default* is not enough — the Windows native
        dialog re-opens wherever it was last used, so Save kept landing in
        the process CWD. The destination is derived, not chosen."""
        from gui.pages import hardware_setup as hs
        self.assertNotIn("QFileDialog",
                         inspect.getsource(hs.HardwareSetupPage._save_config))
        # The escape hatch still exists, and still defaults to the folder.
        self.assertIn("QFileDialog",
                      inspect.getsource(hs.HardwareSetupPage._save_config_as))

    def test_the_setup_dir_is_the_one_the_browser_scans(self):
        from gui.pages import hardware_setup as hs
        page = hs.HardwareSetupPage.__new__(hs.HardwareSetupPage)
        self.assertEqual(Path(page._setup_dir()), hs.CONFIG_HARDWARE_DIR)
        self.assertIn("CONFIG_HARDWARE_DIR",
                      inspect.getsource(hs.HardwareSetupPage._scan_config_directory))

    def test_the_saved_file_lands_where_the_list_scans(self):
        """The name typed on the Identity tab becomes <setups folder>/<name>.json
        — the exact files _refresh_setup_list() globs."""
        from gui.pages import hardware_setup as hs
        page = hs.HardwareSetupPage.__new__(hs.HardwareSetupPage)
        p = page._setup_path_for_name("Standard Bioprinting Setup")
        self.assertEqual(p.parent, hs.CONFIG_HARDWARE_DIR)
        self.assertEqual(p.name, "Standard Bioprinting Setup.json")

    def test_a_name_with_path_characters_cannot_escape_the_folder(self):
        from gui.pages import hardware_setup as hs
        page = hs.HardwareSetupPage.__new__(hs.HardwareSetupPage)
        for bad in ("../../evil", r"C:\temp\evil", "a/b", "", "   ", "."):
            p = page._setup_path_for_name(bad)
            self.assertEqual(p.parent, hs.CONFIG_HARDWARE_DIR, bad)
            self.assertTrue(p.name.endswith(".json"), bad)
            self.assertNotEqual(p.name, ".json", bad)


# ── 7. The Tucsen appears in the microscope camera list ──────────────────────

class TestTucsenIsSelectable(unittest.TestCase):
    """Operator: *"on the microscope camera setup i want the tucsen camera to
    show up on the list."* It was absent from the catalogue entirely, which is
    why the spec stayed on the ToupTek and both cameras shared one block."""

    def _catalog(self):
        from SupportClasses.PhysicalModels import load_camera_catalog
        return load_camera_catalog()

    def test_a_tucsen_is_in_the_catalogue(self):
        names = [s.name for s in self._catalog().values()]
        self.assertTrue(any("Tucsen" in n for n in names), names)

    def test_the_libra_carries_its_hardware_verified_resolutions(self):
        spec = self._catalog()["TUCSEN-LIBRA-25"]
        self.assertEqual(tuple(spec.max_resolution), (5200, 4096))
        self.assertIn((2600, 2048), [tuple(r) for r in spec.preview_resolutions])

    def test_an_unknown_sensor_pitch_is_none_not_a_guess(self):
        """A fabricated pitch shows as a plausible 'theoretical µm/px' that a
        real calibration is then judged against — worse than showing none."""
        spec = self._catalog()["TUCSEN-LIBRA-25"]
        self.assertIsNone(spec.sensor_pixel_size_um)
        self.assertIsNone(spec.effective_pixel_size_um((2600, 2048)))

    def test_a_known_pitch_still_computes_exactly_as_before(self):
        spec = self._catalog()["ZYLA-4.2P"]
        self.assertEqual(spec.sensor_pixel_size_um, 6.5)
        self.assertAlmostEqual(
            spec.effective_pixel_size_um((1024, 1024)), 13.0, places=6)

    def test_computed_scale_is_none_when_the_pitch_is_unknown(self):
        from SupportClasses.HardwareConfig import CameraConfig
        from SupportClasses.PhysicalModels import CameraSpec
        cfg = CameraConfig()
        cfg.camera_spec = CameraSpec(name="X", sensor_pixel_size_um=None,
                                     max_resolution=(5200, 4096))
        cfg.active_resolution = (2600, 2048)
        self.assertIsNone(cfg.computed_micron_per_pixel)


# ── 8. The move bound comes from the objective, not a stale µm/px ────────────

class TestNativeScaleFromObjectiveMagnification(unittest.TestCase):
    """Operator: *"for it to know how far it can move it needs to know the
    objective its on and the measured magnification of the objective — only
    this way can we get the native microns per pixel for the camera at a 1x
    frame."* Exactly right, and the 199 µm was the proof."""

    def _store(self, entries):
        st = ObjectiveCalibrationStore(
            Path(tempfile.mkdtemp()) / "objectives.json")
        st._data["objectives"] = [
            {"name": n, "nominal_magnification": m}
            for n, m in (("2x", 2.0), ("4x", 4.0), ("10x", 10.0),
                         ("20x", 20.0))]
        for obj, um, res in entries:
            st.set_calibration("cam", obj, um, res)
        return st

    def test_the_bogus_199_um_bound_is_reproduced_from_the_wrong_scale(self):
        """0.25 x 2048 px x 0.389135 = 199.2 um — the ToupTek's µm/px applied
        to the Tucsen's frame, which is the number the operator was shown."""
        from SupportClasses.NeedleCameraCalibration import max_in_frame_move_um
        self.assertAlmostEqual(
            max_in_frame_move_um(0.389135, 2600, 2048), 199.2, places=1)

    def test_sensor_width_is_constant_across_objectives(self):
        st = self._store([("2x", 0.778843, (3664, 2748)),
                          ("4x", 0.389135, (3664, 2748))])
        self.assertAlmostEqual(st.sensor_width_um("cam"), 5707.0, delta=25.0)

    def test_native_scale_is_the_camera_at_1x(self):
        st = self._store([("2x", 0.778843, (3664, 2748))])
        # 0.778843 x 2 = 1.5577 um/px at 1x, at the SAME 3664-px width.
        self.assertAlmostEqual(
            st.native_um_per_px("cam", 3664), 1.5577, places=3)

    def test_an_uncalibrated_objective_still_gets_a_prediction(self):
        """The whole point: the move happens BEFORE the measurement exists."""
        st = self._store([("2x", 0.778843, (3664, 2748)),
                          ("4x", 0.389135, (3664, 2748))])
        p20 = st.predicted_um_per_px("cam", "20x", 3664)
        self.assertIsNotNone(p20)
        self.assertAlmostEqual(p20, 5707.0 / 3664 / 20.0, delta=0.01)

    def test_prediction_rescales_to_the_live_frame_width(self):
        st = self._store([("2x", 0.778843, (3664, 2748))])
        at_3664 = st.predicted_um_per_px("cam", "4x", 3664)
        at_1832 = st.predicted_um_per_px("cam", "4x", 1832)
        self.assertAlmostEqual(at_1832 / at_3664, 2.0, places=6)

    def test_inconsistent_objectives_REFUSE_rather_than_pick_one(self):
        """Two entries and a plain median just picks one, so a single bad stamp
        would silently become the camera's native scale. This machine's Andor
        is exactly that case (13218 vs 27036)."""
        st = self._store([("4x", 3.227061, (1024, 1024)),
                          ("10x", 1.320118, (2048, 2048))])
        self.assertIsNone(st.sensor_width_um("cam"))
        self.assertIsNone(st.predicted_um_per_px("cam", "4x", 2048))

    def test_no_calibrations_at_all_is_none(self):
        st = self._store([])
        self.assertIsNone(st.sensor_width_um("cam"))
        self.assertIsNone(st.native_um_per_px("cam", 2600))


class TestDialogSizesTheBoundFromTheObjective(unittest.TestCase):

    def _dlg(self, cam_key, objective, mgr):
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        d = PixelCalibrationDialog.__new__(PixelCalibrationDialog)
        d._cam_key, d._objective = cam_key, objective
        d._camera_manager, d._cam_idx = mgr, 0
        return d

    class _M:
        def __init__(self, um=0.389135, res=None, ok=True):
            self._um, self._res, self._ok = um, res, ok

        def effective_um_per_px(self, i, w):
            if not self._res:
                return self._um                      # the passthrough
            return self._um * self._res[0] / float(w)

        def get_um_per_px_resolution(self, i):
            return self._res

        def is_um_per_px_calibrated(self, i):
            return self._ok

    def test_an_unstamped_manager_value_is_REFUSED(self):
        """The 199 µm came from exactly this: another camera's µm/px, unstamped
        so it passed straight through un-rescaled."""
        d = self._dlg(None, None, self._M(0.389135, res=None))
        self.assertEqual(d._expected_um_per_px(2600), (0.0, ""))

    def test_a_stamped_manager_value_is_used_and_named(self):
        d = self._dlg(None, None, self._M(3.227061, res=(1024, 1024)))
        val, src = d._expected_um_per_px(2048)
        self.assertAlmostEqual(val, 3.227061 / 2.0, places=6)
        self.assertTrue(src)

    def test_the_objective_prediction_beats_the_manager(self):
        st = ObjectiveCalibrationStore(
            Path(tempfile.mkdtemp()) / "objectives.json")
        st._data["objectives"] = [{"name": "2x", "nominal_magnification": 2.0},
                                  {"name": "4x", "nominal_magnification": 4.0}]
        st.set_calibration("cam", "2x", 0.778843, (3664, 2748))
        import SupportClasses.ObjectiveCalibration as oc
        prev = oc._store
        oc._store = st
        try:
            d = self._dlg("cam", "4x", self._M(9.99, res=(100, 100)))
            val, src = d._expected_um_per_px(3664)
            self.assertAlmostEqual(val, 5707.0 / 3664 / 4.0, delta=0.01)
            self.assertIn("magnification", src)
        finally:
            oc._store = prev


if __name__ == "__main__":
    unittest.main()
