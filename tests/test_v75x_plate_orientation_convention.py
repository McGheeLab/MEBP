"""
v7.5.x — ONE canonical well-plate orientation convention.

Convention enforced everywhere a well plate is referenced:
  * well A1 displays TOP-LEFT;
  * the stage's physical 0,0 is BOTTOM-RIGHT (ME3B V1 Prior ProScan II:
    origin bottom-right, +X/+Y toward top-left).

A single per-machine setting — ``plate_flip_180`` on ``StageController`` — is
the source of truth. It drives BOTH the 180° display flip on stage-frame plate
views AND the plate-local→stage geometry sign (``plate_axis_sign``). The plate
data model (A1 at relative (0,0), +col→+X, +row→+Y) is UNCHANGED; the sign is
applied only at the plate-local→stage mapping boundary.

These tests exercise the pure logic on minimal objects (``__new__`` stubs +
offscreen Qt), mirroring test_v75x_z_axis_unified_setup.
"""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.StageController import (
    StageController, DEFAULT_PLATE_FLIP_180)
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.WellPlate import WellPlate
from SupportClasses.PlateWarpCalibrator import PlateWarpCalibrator


# ── 1. The single source of truth ───────────────────────────────────

class TestOrientationSetting(unittest.TestCase):
    def _ctrl(self):
        return StageController.__new__(StageController)

    def test_default_is_flipped(self):
        # A controller whose profile predates the setting falls back to the
        # ME3B default (True) via the getattr guard.
        c = self._ctrl()
        self.assertEqual(c.plate_flip_180(), DEFAULT_PLATE_FLIP_180)
        self.assertTrue(c.plate_flip_180())

    def test_axis_sign_follows_flip(self):
        c = self._ctrl()
        c.set_plate_flip_180(True)
        self.assertEqual(c.plate_axis_sign(), (-1.0, -1.0))
        c.set_plate_flip_180(False)
        self.assertEqual(c.plate_axis_sign(), (1.0, 1.0))

    def test_apply_z_convention_restores_flip(self):
        c = self._ctrl()
        c.set_plate_flip_180(False)
        c.apply_z_convention(plate_flip_180=True)
        self.assertTrue(c.plate_flip_180())
        # None leaves it unchanged.
        c.apply_z_convention(plate_flip_180=None)
        self.assertTrue(c.plate_flip_180())

    def test_device_profile_roundtrip(self):
        from gui.pages.hardware.device_profile import DeviceProfile
        p = DeviceProfile(profile_name="X", plate_flip_180=True)
        self.assertIs(DeviceProfile.from_dict(p.to_dict()).plate_flip_180, True)
        p2 = DeviceProfile(profile_name="Y", plate_flip_180=False)
        self.assertIs(DeviceProfile.from_dict(p2.to_dict()).plate_flip_180, False)
        # Unset round-trips to None (fall back to the controller default).
        p3 = DeviceProfile(profile_name="Z")
        self.assertIsNone(DeviceProfile.from_dict(p3.to_dict()).plate_flip_180)

    def test_device_profile_settings_bridge(self):
        from gui.pages.hardware.device_profile import DeviceProfile

        class _Settings:
            def __init__(self):
                self._d = {}

            def get(self, k, default=None):
                return self._d.get(k, default)

            def set(self, k, v):
                self._d[k] = v

            def get_section(self, k):
                return self._d.get(k, {})

        s = _Settings()
        DeviceProfile(profile_name="A", plate_flip_180=True).apply_to_settings(s)
        self.assertIs(s.get("device_profile.plate_flip_180"), True)
        self.assertIs(
            DeviceProfile.from_settings(s, "A").plate_flip_180, True)


# ── 2. Geometry sign (plate-local → stage mapping) ──────────────────

class TestGeometrySign(unittest.TestCase):
    def setUp(self):
        self.plate = WellPlate.from_format(96)  # A1 (0,0); A2 +X; B1 +Y

    def test_aligned_is_legacy(self):
        a1 = (10_000.0, 20_000.0)
        pos = self.plate.get_all_positions_from_a1(*a1)  # default (1,1)
        # A2 is +X of A1, B1 is +Y of A1 (legacy aligned behaviour).
        self.assertGreater(pos["A2"][0], pos["A1"][0])
        self.assertAlmostEqual(pos["A2"][1], pos["A1"][1])
        self.assertGreater(pos["B1"][1], pos["A1"][1])

    def test_flip_mirrors_about_a1(self):
        a1 = (10_000.0, 20_000.0)
        pos = self.plate.get_all_positions_from_a1(*a1, plate_axis_sign=(-1.0, -1.0))
        # A1 itself is unmoved (the anchor / zero offset).
        self.assertAlmostEqual(pos["A1"][0], a1[0])
        self.assertAlmostEqual(pos["A1"][1], a1[1])
        # On a 180° mount A2 is now on the LOWER-X side of A1, B1 LOWER-Y.
        self.assertLess(pos["A2"][0], pos["A1"][0])
        self.assertLess(pos["B1"][1], pos["A1"][1])
        # Exactly the negation of the aligned offset.
        aligned = self.plate.get_all_positions_from_a1(*a1)
        self.assertAlmostEqual(pos["A2"][0] - a1[0], -(aligned["A2"][0] - a1[0]))

    def test_a1_from_plate_center_sign(self):
        center = (60_000.0, 40_000.0)
        a1_aligned = self.plate.get_a1_from_plate_center(*center)
        a1_flipped = self.plate.get_a1_from_plate_center(
            *center, plate_axis_sign=(-1.0, -1.0))
        # The A1-offset term reflects about the plate centre.
        self.assertAlmostEqual(a1_flipped[0] - center[0],
                               -(a1_aligned[0] - center[0]))
        self.assertAlmostEqual(a1_flipped[1] - center[1],
                               -(a1_aligned[1] - center[1]))

    def test_well_area_bounds_normalised_under_flip(self):
        # A negative sign mirrors the box; min must still be < max.
        a1 = (60_000.0, 40_000.0)
        mn_x, mn_y, mx_x, mx_y = self.plate.get_well_area_bounds_from_a1_um(
            *a1, plate_axis_sign=(-1.0, -1.0))
        self.assertLess(mn_x, mx_x)
        self.assertLess(mn_y, mx_y)

    def test_data_model_unchanged(self):
        # The plate-local data model is NEVER touched by the sign.
        self.assertEqual(self.plate.get_well_position("A1"), (0.0, 0.0))
        self.assertEqual(self.plate.get_well_position("H12"), (99.0, 63.0))


# ── 3. Warp must not be double-flipped ──────────────────────────────

class TestWarpNoDoubleApply(unittest.TestCase):
    """A fresh 3-well teach reproduces the MEASURED positions exactly whether
    the geometry is flipped or not — the warp absorbs the prediction sign."""

    def _teach_and_check(self, sign):
        plate = WellPlate.from_format(96)
        taught_a1 = (50_000.0, 30_000.0)
        predicted = plate.get_all_positions_from_a1(*taught_a1, plate_axis_sign=sign)
        # The operator physically teaches 3 wells (arbitrary measured points).
        measured = {
            "A1": (50_010.0, 29_990.0),
            "A12": (38_900.0, 30_050.0),
            "H12": (38_850.0, 18_300.0),
        }
        warp = PlateWarpCalibrator()
        for name, m in measured.items():
            p = predicted[name]
            warp.add_point(p[0], p[1], m[0], m[1])
        warp.solve()
        corrected = warp.correct_positions(predicted)
        for name, m in measured.items():
            self.assertAlmostEqual(corrected[name][0], m[0], places=3)
            self.assertAlmostEqual(corrected[name][1], m[1], places=3)

    def test_taught_wells_exact_when_flipped(self):
        self._teach_and_check((-1.0, -1.0))

    def test_taught_wells_exact_when_aligned(self):
        self._teach_and_check((1.0, 1.0))


# ── 4. Display flip involution + orientation on screen ──────────────

class TestDisplayFlip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _view(self, flip):
        from gui.widgets.jog_workspace_view import JogWorkspaceView
        v = JogWorkspaceView()
        v.resize(400, 300)
        v.set_safety_limits(SafetyLimits(
            xy_min_x=0.0, xy_max_x=114332.0,
            xy_min_y=0.0, xy_max_y=76645.0))
        v.set_plate_flip_180(flip)
        return v

    def test_flip_is_involution_roundtrip(self):
        for flip in (True, False):
            v = self._view(flip)
            for (x, y) in [(0.0, 0.0), (10_000.0, 5_000.0), (114332.0, 76645.0)]:
                px = v._um_to_px(x, y)
                rx, ry = v._px_to_um(px.x(), px.y())
                self.assertAlmostEqual(rx, x, places=2)
                self.assertAlmostEqual(ry, y, places=2)

    def test_stage_origin_renders_bottom_right_when_flipped(self):
        # Envelope min corner (stage 0,0 in zero-ref) should render at the
        # bottom-right when flipped, top-left when not.
        flipped = self._view(True)
        aligned = self._view(False)
        # Same envelope, so the content rect / scale are identical.
        p_flip = flipped._um_to_px(0.0, 0.0)
        p_align = aligned._um_to_px(0.0, 0.0)
        # Flipped origin is to the right of and below the aligned origin.
        self.assertGreater(p_flip.x(), p_align.x())
        self.assertGreater(p_flip.y(), p_align.y())


# ── 5. Print monitor needle registers with plate-local wells ────────

class TestPlateOverviewNeedle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_flip_negates_needle(self):
        from gui.pages.print_monitor import PlateOverviewWidget
        w = PlateOverviewWidget()
        w.set_plate_flip_180(True)
        self.assertTrue(w._flip_180)
        w.set_plate_flip_180(False)
        self.assertFalse(w._flip_180)


# ── 6. Print path geometric well centre is sign-mapped ──────────────

class TestPrintPathSign(unittest.TestCase):
    def test_planner_well_xy_uses_sign(self):
        from SupportClasses.PrintTrajectoryPlanner import PrintTrajectoryPlanner
        plate = WellPlate.from_format(96)
        planner = PrintTrajectoryPlanner()
        # Aligned: A2 is +X of A1.
        planner._plate_axis_sign = (1.0, 1.0)
        a1 = planner._well_xy(plate, "A1")
        a2 = planner._well_xy(plate, "A2")
        self.assertGreater(a2[0], a1[0])
        # Flipped: A2 is now -X of A1 (physically mirrored).
        planner._plate_axis_sign = (-1.0, -1.0)
        a1f = planner._well_xy(plate, "A1")
        a2f = planner._well_xy(plate, "A2")
        self.assertLess(a2f[0], a1f[0])

    def test_plan_of_action_signed_well_xy(self):
        from SupportClasses.PrintPlanOfAction import _signed_well_xy
        from SupportClasses.PrintManager import PrintSettings
        plate = WellPlate.from_format(96)
        flipped = PrintSettings(plate_axis_sign=(-1.0, -1.0))
        aligned = PrintSettings(plate_axis_sign=(1.0, 1.0))
        self.assertLess(_signed_well_xy(plate, "A2", flipped)[0], 0.0)
        self.assertGreater(_signed_well_xy(plate, "A2", aligned)[0], 0.0)


if __name__ == "__main__":
    unittest.main()
