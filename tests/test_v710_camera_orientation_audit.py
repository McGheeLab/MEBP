"""
v7.10 — camera orientation agreement + needle-camera two-leg calibration.

Operator: *"do a full audit of the microscope camera live view and hardware
calibration and mosaic calibration. All of these surfaces should agree on the
same camera orientation and frame of reference for the stage etc. the live view
should always be non mirrored looking to the user … everything should be
referencing the smallest number of variables possible so we ensure its all the
same."* And: *"The needle cameras need the needle as a reference object … the
needle must move on the needle cameras axis to be able to judge rotation. we
should check z motion, and a 45 degree stage motion to ensure it stays within
the camera frame."*

The tests are grouped by the defect each one pins, because several of these
would pass just as well against the broken code if written the obvious way:

* ``TestOneOrientationResolver`` — that there is exactly ONE implementation of
  the store→manager precedence, by identity, not by behaviour. Three copies
  agreeing today says nothing about three copies agreeing after the next edit.
* ``TestRotationHasOneLiveSource`` — that an objective switch cannot move the
  live rotation. A "does the mosaic still work" test cannot see this: the mosaic
  reads the store, which stayed correct throughout.
* ``TestEveryFeedIsOriented`` — walks the constructor call sites in the source,
  so a NEW un-oriented feed is caught. Instantiating the feeds we already know
  about would only re-test the ones already fixed.
* ``TestNeedleTwoLeg`` — forward-simulates a side camera and checks the solver
  recovers what was put in, including the cases it must REFUSE.
"""

from __future__ import annotations

import ast
import inspect
import textwrap
import math
import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.NeedleCameraCalibration import (  # noqa: E402
    MAX_SCALE_RATIO, NeedleCameraAxes, fold_perpendicular_deg,
    in_frame_refusal, max_in_frame_move_um, solve_needle_camera_axes,
    wrap_deg,
)

REPO = Path(__file__).resolve().parents[1]

_app = None


def setUpModule():
    """A QApplication must exist before any widget is built, or construction
    hangs under offscreen Qt rather than failing."""
    global _app
    from PySide6.QtWidgets import QApplication
    _app = QApplication.instance() or QApplication([])


# ─────────────────────────────────────────────────────────────────────
# One resolver
# ─────────────────────────────────────────────────────────────────────

class TestOneOrientationResolver(unittest.TestCase):
    """The store→manager precedence must exist exactly once."""

    def test_mosaic_calibration_delegates_to_the_shared_resolver(self):
        import SupportClasses.MosaicCalibration as MC
        src = inspect.getsource(MC._resolve_orientation)
        self.assertIn("resolve_camera_orientation(", src)

    def test_the_plate_location_page_delegates_too(self):
        """Pinned by SOURCE, not behaviour.

        The page's copy agreed with the shared one when it was written; that is
        exactly why nobody noticed there were two. What must hold is that the
        page cannot answer this question on its own any more.
        """
        src = (REPO / "gui" / "pages" / "calibration.py").read_text(
            encoding="utf-8")
        tree = ast.parse(src)
        fn = next(
            (n for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef)
             and n.name == "_ploc_microscope_frame_orientation"), None)
        self.assertIsNotNone(fn, "_ploc_microscope_frame_orientation vanished")
        body = ast.get_source_segment(src, fn) or ""
        self.assertIn("resolve_camera_orientation", body)
        # It must not read the store itself any more — that WAS the duplicate.
        self.assertNotIn("get_calibration(", body)

    def test_store_wins_over_a_stale_manager(self):
        from SupportClasses.MosaicCalibration import resolve_camera_orientation

        class _Mgr:
            def get_rotation_deg(self, i):
                return 12.0

            def get_mirrored(self, i):
                return False

            def get_flip_y(self, i):
                return False

        class _Store:
            def get_calibration(self, ident):
                return {"rotation_deg": 180.0, "mirrored": True,
                        "flip_y": False}

        rot, fx, fy = resolve_camera_orientation(
            camera_manager=_Mgr(), cam_idx=0, identity="cam:1",
            cal_store=_Store())
        self.assertEqual(rot, 180.0)
        self.assertTrue(fx)
        self.assertFalse(fy)

    def test_manager_fills_fields_the_store_lacks(self):
        from SupportClasses.MosaicCalibration import resolve_camera_orientation

        class _Mgr:
            def get_rotation_deg(self, i):
                return 12.0

            def get_mirrored(self, i):
                return True

            def get_flip_y(self, i):
                return True

        class _Store:
            def get_calibration(self, ident):
                return {"rotation_deg": 90.0}      # no flips recorded

        prov: dict = {}
        rot, fx, fy = resolve_camera_orientation(
            camera_manager=_Mgr(), cam_idx=0, identity="cam:1",
            cal_store=_Store(), provenance=prov)
        self.assertEqual(rot, 90.0)
        self.assertTrue(fx)
        self.assertTrue(fy)
        self.assertEqual(prov["rotation_deg"], "camera store")
        self.assertEqual(prov["flip_x"], "live camera manager")

    def test_a_broken_store_degrades_instead_of_raising(self):
        from SupportClasses.MosaicCalibration import resolve_camera_orientation

        class _Boom:
            def get_calibration(self, ident):
                raise RuntimeError("disk on fire")

        rot, fx, fy = resolve_camera_orientation(
            camera_manager=None, cam_idx=None, identity="cam:1",
            cal_store=_Boom())
        self.assertEqual((rot, fx, fy), (0.0, False, False))


# ─────────────────────────────────────────────────────────────────────
# One live source for rotation
# ─────────────────────────────────────────────────────────────────────

class TestRotationHasOneLiveSource(unittest.TestCase):
    """Switching objectives must not move the camera's rotation.

    Rotation is a property of the MOUNT. The objective store keeps a copy for
    backwards compatibility, and two call sites used to push that copy into the
    live manager on every objective change — so the mosaic (store) and the live
    view (manager) could hold different angles for the same camera with nothing
    on screen to say so.
    """

    def _push_source(self, path, func_name):
        src = (REPO / path).read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == func_name:
                return ast.get_source_segment(src, node) or ""
        self.fail(f"{func_name} not found in {path}")

    def test_objective_switch_does_not_push_rotation(self):
        body = self._push_source(
            "gui/pages/hardware/objective_calibration_card.py",
            "_push_stored_um_per_px_to_manager")
        self.assertIn("set_um_per_px", body)
        self.assertNotIn("set_rotation_deg", body)

    def test_fluorescence_objective_switch_does_not_push_rotation(self):
        body = self._push_source(
            "gui/pages/workflows/fluorescence_mosaic_workflow.py",
            "_on_objective_changed")
        self.assertIn("set_um_per_px", body)
        self.assertNotIn("set_rotation_deg", body)

    def test_one_commit_path_writes_manager_and_mount_store(self):
        """A measured rotation must reach BOTH live and persisted homes.

        The per-objective µm/px calibration measured a rotation, pushed it to
        the manager and the objective copy, and never wrote the per-identity
        mount store — so the mosaic kept the old angle and a restart threw the
        measurement away entirely.
        """
        from gui.pages.hardware.objective_calibration_card import (
            ObjectiveCalibrationCard)
        src = inspect.getsource(ObjectiveCalibrationCard.commit_camera_rotation)
        self.assertIn("set_rotation_deg", src)      # live
        self.assertIn("set_rotation(", src)         # persisted, per identity
        self.assertIn("adopt_camera_rotation", src)  # per-objective mirror

        cal_src = inspect.getsource(
            ObjectiveCalibrationCard._on_calibrate_clicked)
        self.assertIn("commit_camera_rotation", cal_src)
        orient_src = inspect.getsource(
            ObjectiveCalibrationCard._on_calibrate_orientation_clicked)
        self.assertIn("commit_camera_rotation", orient_src)


# ─────────────────────────────────────────────────────────────────────
# Every feed oriented
# ─────────────────────────────────────────────────────────────────────

class TestEveryFeedIsOriented(unittest.TestCase):
    """Walk the SOURCE for CameraFeedView constructions.

    Written against the call sites rather than against instantiated widgets so
    that a feed added in future is caught too. Only feeds that deliberately show
    raw pixels are allowed to opt out, and each has to say why here.
    """

    #: call sites that legitimately show an un-oriented image.
    EXEMPT = {
        # The square-up tool sets orientation explicitly: it corrects the flips
        # (a fixed property of the optical path) but deliberately leaves the
        # ROTATION showing, because the rotation is the thing being changed.
        ("gui/dialogs/camera_rotation_align_dialog.py", "auto_orient=False"),
        # The hardware-settings dialog previews the RAW device output while the
        # operator changes exposure/resolution on the device itself.
        ("gui/dialogs/camera_settings_dialog.py", None),
    }

    def _feed_calls(self):
        out = []
        for path in sorted((REPO / "gui").rglob("*.py")):
            rel = path.relative_to(REPO).as_posix()
            src = path.read_text(encoding="utf-8")
            if "CameraFeedView(" not in src:
                continue
            try:
                tree = ast.parse(src)
            except SyntaxError:      # pragma: no cover
                continue
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                fn = node.func
                name = getattr(fn, "id", None) or getattr(fn, "attr", None)
                if name != "CameraFeedView":
                    continue
                kw = {k.arg: k for k in node.keywords if k.arg}
                out.append((rel, node.lineno, kw))
        return out

    def test_every_live_feed_applies_the_calibrated_orientation(self):
        offenders = []
        for rel, lineno, kw in self._feed_calls():
            if any(rel == p for p, _ in self.EXEMPT):
                continue
            ao = kw.get("auto_orient")
            ok = (ao is not None
                  and isinstance(ao.value, ast.Constant)
                  and ao.value.value is True)
            if not ok:
                offenders.append(f"{rel}:{lineno}")
        self.assertFalse(
            offenders,
            "CameraFeedView built without auto_orient=True — these feeds show "
            "the camera's RAW image while every other surface shows it "
            "corrected:\n  " + "\n  ".join(offenders))

    def test_at_least_the_known_feeds_were_found(self):
        """Guard the walker itself: a matcher that finds nothing passes."""
        calls = self._feed_calls()
        self.assertGreaterEqual(len(calls), 8, f"only found {len(calls)}")
        rels = {c[0] for c in calls}
        self.assertIn("gui/pages/hardware_setup.py", rels)
        self.assertIn("gui/pages/calibration.py", rels)

    def test_target_overlay_view_forwards_auto_orient_and_defaults_on(self):
        from gui.widgets.target_overlay_camera_view import (
            TargetOverlayCameraView)
        sig = inspect.signature(TargetOverlayCameraView.__init__)
        self.assertIn("auto_orient", sig.parameters)
        self.assertIs(sig.parameters["auto_orient"].default, True)
        src = inspect.getsource(TargetOverlayCameraView.__init__)
        self.assertIn("auto_orient=auto_orient", src)

    def test_picker_maps_raw_pixels_through_the_view_transform(self):
        """``_image_to_widget`` must be a true inverse of ``_widget_to_image``.

        With the picker feed now oriented, treating a raw pixel as if it were
        already a displayed pixel puts every target ring in the wrong place on
        a rotated or mirrored camera.

        v7.15: this asserted the substring ``_view_true_xform`` appeared in the
        method. Both directions now delegate to the shared ``ViewGeometry``, so
        the name is no longer written there — and a substring check could never
        have shown the inverse was *correct* anyway, only that it mentioned the
        transform. Replaced by the round trip it was standing in for.
        """
        from PySide6.QtGui import QTransform
        from gui.widgets.view_geometry import ViewGeometry
        from gui.widgets.camera_feed_view import view_transform_coeffs

        for rot in (0.0, 90.0, 180.0, 270.0):
            for mirror in (False, True):
                m11, m12, m21, m22 = view_transform_coeffs(mirror, False, rot)
                base = QTransform(m11, m12, m21, m22, 0.0, 0.0)
                # Qt's own fit offset, exactly as _orient_qimage takes it.
                from PySide6.QtGui import QImage
                true_xf = QImage.trueMatrix(base, 640, 480)
                disp = (640, 480) if rot in (0.0, 180.0) else (480, 640)
                geo = ViewGeometry.build(
                    raw_size=(640, 480), disp_size=disp,
                    pixmap_size=(disp[0] // 2, disp[1] // 2),
                    label_size=(disp[0] // 2 + 40, disp[1] // 2 + 24),
                    true_xform=true_xf)
                # Interior points: a pixel exactly ON the border can land a
                # hair outside the bounds check after the fit offset, which is
                # a legitimate "click was in the letterbox" answer, not a
                # mapping error.
                for raw in ((8.0, 8.0), (600.0, 440.0), (100.0, 250.0)):
                    w = geo.to_widget(*raw)
                    self.assertIsNotNone(w, f"rot={rot} mirror={mirror}")
                    back = geo.to_image(*w)
                    self.assertIsNotNone(back)
                    self.assertAlmostEqual(back[0], raw[0], places=3,
                                           msg=f"rot={rot} mirror={mirror}")
                    self.assertAlmostEqual(back[1], raw[1], places=3,
                                           msg=f"rot={rot} mirror={mirror}")


class TestNeedleFeedsDoNotRotate(unittest.TestCase):
    """A needle feed must sit at a fixed quarter-turn, never at the residual.

    Operator: *"the entire point of this is that the live views do not rotate.
    they should be at fixed rotations on our screen corresponding to
    0,90,180,270 if needed. then the rotation needed is set and we try to match
    it."* Un-rotating the residual in software destroys the operator's only
    feedback while they turn the camera in its mount — the tilt disappears and
    the correction tracks them, so the picture never appears to change.
    """

    def _view(self, rot, snap):
        from gui.widgets.camera_feed_view import CameraFeedView

        class _Mgr:
            cameras = []                       # no live widget to subscribe to

            def full_orientation(self, i):
                return (True, False, rot)      # mirrored, no flip-Y

        v = CameraFeedView(camera_manager=_Mgr(), cam_idx=0,
                           enable_settings=False, auto_orient=True,
                           snap_rotation_to_cardinal=snap)
        v._sync_auto_orientation()
        return v

    def test_a_residual_roll_never_reaches_the_display(self):
        v = self._view(3.7, snap=True)
        self.assertEqual(v._view_rot_deg, 0.0)
        # ...but the mirror correction still applies: un-mirrored, as asked.
        self.assertTrue(v._view_mirror)

    def test_it_snaps_to_the_nearest_quarter_turn_not_to_zero(self):
        for rot, expect in ((87.0, 90.0), (-93.0, -90.0), (177.0, 180.0),
                            (-2.5, 0.0)):
            self.assertEqual(self._view(rot, snap=True)._view_rot_deg, expect)

    def test_turning_the_camera_does_not_move_the_view(self):
        """Sweep the measured angle across a physical turn: the DISPLAY must
        hold still, which is what makes the residual readout meaningful."""
        seen = {self._view(r, snap=True)._view_rot_deg
                for r in (0.0, 1.0, 2.5, 4.0, 6.0, 8.5, 11.0)}
        self.assertEqual(seen, {0.0})

    def test_the_microscope_path_is_unchanged(self):
        """Snapping is opt-in. The microscope keeps the exact measured angle —
        its mosaic tiles have to be stage-aligned, and the operator reported
        that path working well."""
        self.assertEqual(self._view(3.7, snap=False)._view_rot_deg, 3.7)
        self.assertEqual(self._view(180.0, snap=False)._view_rot_deg, 180.0)

    def test_snapping_is_off_unless_asked_for(self):
        """Built WITHOUT the parameter — so flipping the default is caught.

        The previous test passes ``snap=False`` explicitly and therefore never
        exercises the default; a mutation that flipped it survived, which is how
        this test came to exist. A flipped default would quietly start snapping
        the microscope feed too.
        """
        from gui.widgets.camera_feed_view import CameraFeedView

        class _Mgr:
            cameras = []

            def full_orientation(self, i):
                return (False, False, 3.7)

        v = CameraFeedView(camera_manager=_Mgr(), cam_idx=0,
                           enable_settings=False, auto_orient=True)
        v._sync_auto_orientation()
        self.assertEqual(v._view_rot_deg, 3.7)
        self.assertFalse(v._snap_rotation)
        sig = inspect.signature(CameraFeedView.__init__)
        self.assertIs(
            sig.parameters["snap_rotation_to_cardinal"].default, False)

    def test_both_needle_feeds_snap(self):
        src = (REPO / "gui" / "pages" / "calibration.py").read_text(
            encoding="utf-8")
        tree = ast.parse(src)
        snapped = 0
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if getattr(node.func, "id", None) != "CameraFeedView":
                continue
            kw = {k.arg: k for k in node.keywords if k.arg}
            v = kw.get("snap_rotation_to_cardinal")
            if v is not None and isinstance(v.value, ast.Constant) \
                    and v.value.value is True:
                snapped += 1
        self.assertGreaterEqual(
            snapped, 2,
            "the Needle Location and Pump Compliance feeds must both hold a "
            "fixed quarter-turn")

    def test_the_geometry_still_uses_the_exact_angle(self):
        """Only the DISPLAY snaps. A click must still map through the measured
        rotation, or squaring up the view would move the stage wrongly."""
        from gui.widgets.camera_manager import CameraManager
        m = CameraManager.__new__(CameraManager)
        m._max_cameras = 1
        m._um_per_px = [2.0]
        m._um_per_px_res = [None]
        m._um_per_px_set = [True]
        m._rotation_deg = [90.0]
        m._mirrored = [False]
        m._flip_y = [False]
        m._cameras = []
        dx, dy = m.pixel_to_stage_offset(0, 110.0, 100.0, 200, 200)
        # A pure +90 deg rotation sends +x to +y.
        self.assertAlmostEqual(dx, 0.0, places=9)
        self.assertAlmostEqual(dy, 20.0, places=9)


class TestViewOrientationUnMirrors(unittest.TestCase):
    """The display transform must REVERSE handedness for a mirrored camera."""

    def test_a_mirrored_camera_is_shown_un_mirrored(self):
        from gui.widgets.camera_feed_view import view_transform_coeffs
        m11, m12, m21, m22 = view_transform_coeffs(True, False, 0.0)
        det = m11 * m22 - m12 * m21
        self.assertLess(det, 0, "a mirrored camera's view must be flipped back")
        self.assertAlmostEqual(m11, -1.0)

    def test_an_unmirrored_camera_is_left_alone(self):
        from gui.widgets.camera_feed_view import view_transform_coeffs
        m11, m12, m21, m22 = view_transform_coeffs(False, False, 0.0)
        self.assertEqual((m11, m12, m21, m22), (1.0, 0.0, -0.0, 1.0))

    def test_both_flips_is_a_rotation_not_a_mirror(self):
        """diag(-1,-1) == R(180): handedness is preserved, so the view is
        still un-mirrored. This is the identity behind the Stage 0 fix."""
        from gui.widgets.camera_feed_view import view_transform_coeffs
        m11, m12, m21, m22 = view_transform_coeffs(True, True, 0.0)
        self.assertGreater(m11 * m22 - m12 * m21, 0)


# ─────────────────────────────────────────────────────────────────────
# Needle two-leg calibration
# ─────────────────────────────────────────────────────────────────────

def _simulate_side_camera(psi_deg, opt_az_deg, u, hand, D, c_deg, dz):
    """Image displacements a side camera would report for the two legs.

    ``psi`` sensor roll, ``opt_az`` optical-axis azimuth in the stage XY plane,
    ``u`` true µm/px, ``hand`` which perpendicular sense is image +column.
    The XY leg moves the CAMERA (it rides the stage), so stationary content —
    including the needle — shifts by −m. The Z leg moves the NEEDLE only.
    """
    psi = math.radians(psi_deg)
    lat_img = (math.cos(psi), math.sin(psi))
    up_img = (math.sin(psi), -math.cos(psi))
    n = (math.cos(math.radians(opt_az_deg + 90.0)),
         math.sin(math.radians(opt_az_deg + 90.0)))
    m = (D * math.cos(math.radians(c_deg)), D * math.sin(math.radians(c_deg)))
    proj = -(m[0] * n[0] + m[1] * n[1]) * hand
    lat = (lat_img[0] * proj / u, lat_img[1] * proj / u)
    zl = (up_img[0] * dz / u, up_img[1] * dz / u)
    return lat, zl


class TestNeedleTwoLeg(unittest.TestCase):

    def test_recovers_scale_roll_and_orthogonality_exactly(self):
        checked = 0
        for psi in (-12.0, -3.5, 0.0, 2.0, 8.0):
            for az in (0.0, 37.0, 90.0, 135.0, -60.0):
                for hand in (1, -1):
                    for c in (45.0, -45.0, 0.0, 90.0):
                        lat, zl = _simulate_side_camera(
                            psi, az, 2.35, hand, 200.0, c, 200.0)
                        ax = solve_needle_camera_axes(
                            lateral_um=200.0, lateral_dx_px=lat[0],
                            lateral_dy_px=lat[1], z_um=200.0,
                            z_dx_px=zl[0], z_dy_px=zl[1])
                        if ax.refusal():
                            continue          # covered by the refusal tests
                        checked += 1
                        self.assertAlmostEqual(ax.um_per_px, 2.35, places=9)
                        self.assertAlmostEqual(ax.roll_deg, psi, places=9)
                        self.assertAlmostEqual(
                            ax.orthogonality_err_deg, 0.0, places=9)
                        self.assertEqual(ax.z_row_sign, 1.0)
        self.assertGreater(checked, 100, "simulation covered too little")

    def test_the_z_leg_measures_roll_the_lateral_leg_can_only_assume(self):
        """The Z leg's roll must be right even when the XY leg is unusable.

        Lab-vertical is a KNOWN direction, so the roll falls out of the Z leg
        alone; the lateral leg needs to know where lateral IS, which is what it
        was supposed to be measuring.
        """
        # Commanded XY move straight down the optical axis: no lateral signal.
        lat, zl = _simulate_side_camera(7.5, 0.0, 2.0, 1, 200.0, 0.0, 200.0)
        self.assertLess(math.hypot(*lat), 1e-6)
        ax = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
        self.assertIsNotNone(ax.refusal())          # must not be committed
        self.assertAlmostEqual(ax.roll_deg, 7.5, places=9)   # still measured

    def test_lateral_only_um_per_px_is_over_estimated_and_corrected(self):
        """The defect the Z leg exists to fix.

        A 45°-off preset makes the single-leg µm/px read 1/cos(45°) = 1.414x
        high, and nothing in the single-leg flow can detect it. Here it is both
        detected and corrected.
        """
        lat, zl = _simulate_side_camera(0.0, 0.0, 2.0, 1, 200.0, 45.0, 200.0)
        ax = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
        self.assertIsNone(ax.refusal())
        self.assertAlmostEqual(ax.um_per_px, 2.0, places=9)
        self.assertAlmostEqual(ax.um_per_px_lateral, 2.0 * math.sqrt(2),
                               places=9)
        self.assertAlmostEqual(ax.scale_ratio, math.sqrt(2), places=9)
        self.assertAlmostEqual(ax.off_lateral_deg, 45.0, places=6)
        self.assertTrue(any("off this camera's lateral" in a
                            for a in ax.advisories()))

    def test_an_inverted_mount_is_reported_as_a_sign_not_a_180_roll(self):
        lat, zl = _simulate_side_camera(0.0, 0.0, 2.0, 1, 200.0, 90.0, 200.0)
        ax = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
            z_um=200.0, z_dx_px=-zl[0], z_dy_px=-zl[1])
        self.assertIsNone(ax.refusal())
        self.assertEqual(ax.z_row_sign, -1.0)
        self.assertAlmostEqual(ax.roll_deg, 0.0, places=9)

    def test_a_commanded_downward_z_move_gives_the_same_sign(self):
        """The sign describes the CAMERA, not which way the operator jogged."""
        lat, zl = _simulate_side_camera(3.0, 0.0, 2.0, 1, 200.0, 90.0, 200.0)
        up = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
        down = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
            z_um=-200.0, z_dx_px=-zl[0], z_dy_px=-zl[1])
        self.assertEqual(up.z_row_sign, down.z_row_sign)
        self.assertAlmostEqual(up.roll_deg, down.roll_deg, places=9)

    def test_background_lock_on_the_z_leg_is_refused_and_named(self):
        """A Z move moves ONLY the needle, so ~zero displacement means the
        tracker held onto the stationary background. Whole-frame phase
        correlation would report that zero perfectly happily."""
        ax = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=85.0, lateral_dy_px=0.0,
            z_um=200.0, z_dx_px=0.3, z_dy_px=-0.2)
        why = ax.refusal()
        self.assertIsNotNone(why)
        self.assertIn("background", why)

    def test_legs_tracking_different_objects_are_refused(self):
        ax = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=80.0, lateral_dy_px=0.0,
            z_um=200.0, z_dx_px=-56.0, z_dy_px=-56.0)   # 45 deg, not 90
        self.assertIn("90", ax.refusal() or "")

    def test_a_wildly_off_lateral_preset_is_refused_not_absorbed(self):
        lat, zl = _simulate_side_camera(0.0, 0.0, 2.0, 1, 200.0, 20.0, 200.0)
        ax = solve_needle_camera_axes(
            lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
        self.assertGreater(ax.scale_ratio, MAX_SCALE_RATIO)
        self.assertIn("disagree about scale", ax.refusal() or "")

    def test_the_committed_display_rotation_is_the_negated_roll(self):
        """``rotation_deg`` renders the view level, so it is −roll.

        Composed against the legacy single-leg helper so the two-leg path
        cannot silently invert the display relative to every camera calibrated
        before it.
        """
        from gui.dialogs.pixel_calibration_dialog import (
            view_roll_from_displacement)
        for psi in (-9.0, -1.0, 0.0, 4.0, 11.0):
            lat, zl = _simulate_side_camera(psi, 0.0, 2.0, 1, 200.0, 90.0, 200.0)
            ax = solve_needle_camera_axes(
                lateral_um=200.0, lateral_dx_px=lat[0], lateral_dy_px=lat[1],
                z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
            legacy = view_roll_from_displacement(lat[0], lat[1])
            self.assertAlmostEqual(
                fold_perpendicular_deg(-ax.roll_deg - legacy), 0.0, places=6)

    def test_the_z_leg_alone_is_a_complete_result(self):
        """The Z leg must not require the XY leg to be usable.

        It is the independent and more trustworthy measurement; gating a
        rotation correction on the lateral one would make the good measurement
        depend on the one it supersedes.
        """
        for psi in (-8.0, 0.0, 5.5):
            _lat, zl = _simulate_side_camera(
                psi, 37.0, 2.35, 1, 200.0, 45.0, 200.0)
            ax = solve_needle_camera_axes(
                z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
            self.assertIsNone(ax.refusal())
            self.assertFalse(ax.has_lateral)
            self.assertAlmostEqual(ax.um_per_px, 2.35, places=9)
            self.assertAlmostEqual(ax.roll_deg, psi, places=9)
            self.assertEqual(ax.z_row_sign, 1.0)
            self.assertIsNone(ax.um_per_px_lateral)
            self.assertIsNone(ax.scale_ratio)
            self.assertIsNone(ax.orthogonality_err_deg)

    def test_z_only_says_the_mount_direction_was_not_measured(self):
        _lat, zl = _simulate_side_camera(0.0, 0.0, 2.0, 1, 200.0, 90.0, 200.0)
        ax = solve_needle_camera_axes(
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
        self.assertTrue(any("MOUNT direction" in a for a in ax.advisories()))

    def test_z_only_still_refuses_a_background_lock(self):
        """Dropping the XY requirement must not drop the Z leg's own gate."""
        ax = solve_needle_camera_axes(z_um=200.0, z_dx_px=0.2, z_dy_px=-0.1)
        self.assertIn("background", ax.refusal() or "")

    def test_a_bad_xy_leg_cannot_veto_a_good_z_leg_when_omitted(self):
        """The same Z data: refused with a hopeless XY leg attached, accepted
        on its own. Which is the point — the operator can drop the XY leg."""
        _lat, zl = _simulate_side_camera(0.0, 0.0, 2.0, 1, 200.0, 90.0, 200.0)
        with_bad_xy = solve_needle_camera_axes(
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1],
            lateral_um=200.0, lateral_dx_px=0.4, lateral_dy_px=0.1)
        self.assertIsNotNone(with_bad_xy.refusal())
        z_only = solve_needle_camera_axes(
            z_um=200.0, z_dx_px=zl[0], z_dy_px=zl[1])
        self.assertIsNone(z_only.refusal())
        self.assertAlmostEqual(z_only.um_per_px, 2.0, places=9)

    def test_the_dialog_does_not_gate_the_z_button_on_the_xy_leg(self):
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        src = inspect.getsource(PixelCalibrationDialog._start_z_leg)
        self.assertNotIn("_lateral_px is None", src)

    def test_a_zero_input_returns_a_refusal_rather_than_raising(self):
        ax = solve_needle_camera_axes(
            lateral_um=0.0, lateral_dx_px=0.0, lateral_dy_px=0.0,
            z_um=0.0, z_dx_px=0.0, z_dy_px=0.0)
        self.assertIsInstance(ax, NeedleCameraAxes)
        self.assertIsNotNone(ax.refusal())
        self.assertFalse(ax.is_trustworthy)


class TestInFrameGuard(unittest.TestCase):
    """A too-large move and an optical-axis move produce the SAME symptom."""

    def test_a_move_that_would_leave_the_frame_is_refused(self):
        why = in_frame_refusal(2000.0, 2.0, 1280, 720, what="Z move")
        self.assertIsNotNone(why)
        self.assertIn("out of the frame", why)
        self.assertIn("Z move", why)

    def test_a_move_that_stays_in_frame_is_allowed(self):
        self.assertIsNone(in_frame_refusal(200.0, 2.0, 1280, 720))

    def test_unknown_scale_never_blocks_the_first_calibration(self):
        """An uncalibrated camera has to be able to make its first
        measurement; refusing on unknown scale would be a deadlock."""
        self.assertIsNone(in_frame_refusal(500.0, 0.0, 1280, 720))
        self.assertIsNone(in_frame_refusal(500.0, 2.0, 0, 0))
        self.assertEqual(max_in_frame_move_um(0.0, 1280, 720), 0.0)
        self.assertEqual(max_in_frame_move_um(2.0, 0, 0), 0.0)

    def test_the_limit_scales_with_the_shorter_frame_dimension(self):
        self.assertAlmostEqual(max_in_frame_move_um(2.0, 1280, 720),
                               720 * 0.25 * 2.0)
        self.assertAlmostEqual(max_in_frame_move_um(2.0, 720, 1280),
                               720 * 0.25 * 2.0)

    def test_both_dialog_legs_consult_the_guard(self):
        """Look for the CALL, not the import.

        A substring check on the function source passes on the import line
        alone, so deleting the actual call and hard-coding "no refusal" would
        slip through — mutation-confirmed, which is how this test came to be
        written this way.
        """
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog

        def calls_guard(fn):
            tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
            return any(
                isinstance(n, ast.Call)
                and (getattr(n.func, "id", None) == "in_frame_refusal"
                     or getattr(n.func, "attr", None) == "in_frame_refusal")
                for n in ast.walk(tree))

        self.assertTrue(calls_guard(PixelCalibrationDialog._start_calibration),
                        "the XY leg never calls in_frame_refusal")
        self.assertTrue(calls_guard(PixelCalibrationDialog._start_z_leg),
                        "the Z leg never calls in_frame_refusal")

    def test_the_refusal_actually_stops_the_move(self):
        """The guard's answer must gate a return, not just be computed."""
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        for fn in (PixelCalibrationDialog._start_calibration,
                   PixelCalibrationDialog._start_z_leg):
            tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
            guarded = [
                n for n in ast.walk(tree)
                if isinstance(n, ast.If)
                and isinstance(n.test, ast.Name) and n.test.id == "why"
                and any(isinstance(b, ast.Return) for b in n.body)]
            self.assertTrue(
                guarded, f"{fn.__name__} computes a refusal but does not "
                         f"return on it")


class TestAngleHelpers(unittest.TestCase):

    def test_wrap_deg_is_half_open_at_180(self):
        self.assertEqual(wrap_deg(180.0), 180.0)
        self.assertEqual(wrap_deg(-180.0), 180.0)
        self.assertEqual(wrap_deg(190.0), -170.0)

    def test_fold_perpendicular_is_half_open_at_90(self):
        self.assertEqual(fold_perpendicular_deg(90.0), 90.0)
        self.assertEqual(fold_perpendicular_deg(-90.0), 90.0)
        self.assertAlmostEqual(fold_perpendicular_deg(100.0), -80.0)
        self.assertAlmostEqual(fold_perpendicular_deg(-180.0), 0.0)


class TestPureModuleStaysPure(unittest.TestCase):

    def test_no_qt_or_cv2_at_import(self):
        for mod in ("PySide6", "cv2", "skimage", "scipy"):
            sys.modules.pop(mod, None)
        sys.modules.pop("SupportClasses.NeedleCameraCalibration", None)
        import SupportClasses.NeedleCameraCalibration  # noqa: F401
        self.assertNotIn("PySide6", sys.modules)
        self.assertNotIn("skimage", sys.modules)


# ─────────────────────────────────────────────────────────────────────
# Measured Z sign supersedes the manual guess
# ─────────────────────────────────────────────────────────────────────

class TestMeasuredZSign(unittest.TestCase):

    def _store(self, tmp):
        from SupportClasses.CameraCalibrationStore import (
            CameraCalibrationStore)
        return CameraCalibrationStore(path=tmp / "cams.json")

    def test_round_trips_and_absent_means_never_measured(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            st = self._store(Path(d))
            self.assertIsNone(st.get_z_row_sign("cam:1"))
            st.set_z_row_sign("cam:1", -1.0, name="Needle 1")
            self.assertEqual(st.get_z_row_sign("cam:1"), -1.0)
            st.set_z_row_sign("cam:1", None)
            self.assertIsNone(st.get_z_row_sign("cam:1"))

    def test_it_preserves_its_siblings(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            st = self._store(Path(d))
            st.set_calibration("cam:1", 2.5, rotation_deg=3.0,
                               column_dir_deg=-45.0, name="Needle 1")
            st.set_z_row_sign("cam:1", 1.0)
            entry = st.get_calibration("cam:1")
            self.assertAlmostEqual(entry["um_per_px"], 2.5)
            self.assertAlmostEqual(entry["rotation_deg"], 3.0)
            self.assertAlmostEqual(entry["column_dir_deg"], -45.0)

    def test_the_page_prefers_the_measurement_over_the_checkbox(self):
        from gui.pages.calibration import CalibrationPage
        src = inspect.getsource(CalibrationPage._needle_loc_z_sign)
        self.assertIn("get_z_row_sign", src)
        # The checkbox must remain the fallback, so an un-remeasured camera
        # behaves exactly as it did before.
        self.assertIn("invert", src)

        page = CalibrationPage.__new__(CalibrationPage)
        page._camera_manager = None
        self.assertEqual(page._needle_loc_z_sign(0, False), 1.0)
        self.assertEqual(page._needle_loc_z_sign(0, True), -1.0)

    def test_a_measured_sign_beats_a_wrong_checkbox(self):
        from gui.pages.calibration import CalibrationPage

        class _Store:
            def get_z_row_sign(self, ident):
                return -1.0

        class _Mgr:
            def camera_identity(self, i):
                return ("cam:1", "Needle 1")

        page = CalibrationPage.__new__(CalibrationPage)
        page._camera_manager = _Mgr()
        import SupportClasses.CameraCalibrationStore as CCS
        real = CCS.get_store
        CCS.get_store = lambda: _Store()
        try:
            # Checkbox says "don't invert"; the measurement says otherwise.
            self.assertEqual(page._needle_loc_z_sign(0, False), -1.0)
        finally:
            CCS.get_store = real


if __name__ == "__main__":
    unittest.main()
