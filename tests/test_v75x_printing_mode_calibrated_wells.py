"""
v7.5.x — Printing-mode print path must use CALIBRATED well positions.

Operator bug: after calibrating the plate, pressing Run in Printing mode drove
the stage to the origin corner (0,0) and the needle never reached the assigned
wells. Log proof:

    PRINT: well C2 — first point (-22.5, -42.9)
    XY clamped: (-22518.5,-42936.4) → (0.0,0.0)

Root cause: every well centre in the Printing-mode print path was computed
GEOMETRICALLY — ``plate.get_well_position`` (A1 at origin) × ``plate_axis_sign``
(-1,-1 on ME3B V1) — and fed to ``move_xy_absolute(from_zero_ref=True)`` as a
zero-ref mm stage coordinate. With the needle-zero origin, the well grid was
anchored at the stage origin and the (-1,-1) flip made every well NEGATIVE →
clamped to 0,0. The taught/calibrated positions were never plumbed into the
print path (only Jog + Workflows received them).

Fix: one resolver — ``PrintTrajectoryPlanner.resolve_well_xy_mm`` — that PREFERS
the per-job calibrated map (``settings.well_positions_mm``, zero-ref mm) and
falls back to geometric × sign. Used by the planner (``_well_xy``) and the
``HybridPlanExecutor`` (``_well_xy_mm``). The discrete builder resolves the same
way in ``print_setup_legacy._well_xy_zref_mm``.

These tests pin the resolver + the byte-identical geometric fallback (so
uncalibrated jobs / existing tests are unaffected).
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SupportClasses.PrintManager import (
    build_well_plate_job, PrintSettings, CommandType, HybridPlanExecutor,
)
from SupportClasses.PrintTrajectoryPlanner import (
    PrintTrajectoryPlanner, resolve_well_xy_mm,
)


class _FakePlate:
    """Plate-local geometry: A1 at (0,0), +col → +X, +row → +Y, 19.3 mm pitch.
    Mirrors the real WellPlate contract used by the print path."""

    PITCH = 19.3
    _COLS = {n: c for c, n in enumerate(range(1, 7))}

    def get_well_position(self, name):
        # e.g. "C2" → row index 2 (A=0,B=1,C=2), col index 1
        row = "ABCD".index(name[0].upper())
        col = int(name[1:]) - 1
        return (col * self.PITCH, row * self.PITCH)


# C2 = (col 1, row 2) → geometric (19.3, 38.6) mm
GEO_C2 = (19.3, 2 * 19.3)
# A taught/calibrated C2 in ABSOLUTE stage µm (as the calibration stores it),
# already converted to zero-ref mm for settings.well_positions_mm.
CAL_C2_ZREF_MM = (86.0287, 28.2744)   # from the real ME3B V1 last_calibration


class TestResolver(unittest.TestCase):
    def setUp(self):
        self.plate = _FakePlate()

    def test_prefers_calibrated_map(self):
        s = PrintSettings(plate_axis_sign=(-1.0, -1.0),
                          well_positions_mm={"C2": CAL_C2_ZREF_MM})
        x, y = resolve_well_xy_mm("C2", self.plate, s)
        self.assertAlmostEqual(x, CAL_C2_ZREF_MM[0], places=4)
        self.assertAlmostEqual(y, CAL_C2_ZREF_MM[1], places=4)
        # Crucially POSITIVE — never the negative geometric value that clamps.
        self.assertGreater(x, 0.0)
        self.assertGreater(y, 0.0)

    def test_calibrated_lookup_is_case_insensitive(self):
        s = PrintSettings(well_positions_mm={"C2": CAL_C2_ZREF_MM})
        x, y = resolve_well_xy_mm("c2", self.plate, s)
        self.assertAlmostEqual(x, CAL_C2_ZREF_MM[0], places=4)

    def test_geometric_fallback_when_no_map(self):
        # No calibrated map → geometric × sign (legacy behaviour).
        s = PrintSettings(plate_axis_sign=(-1.0, -1.0))
        x, y = resolve_well_xy_mm("C2", self.plate, s)
        self.assertAlmostEqual(x, -GEO_C2[0], places=4)
        self.assertAlmostEqual(y, -GEO_C2[1], places=4)

    def test_geometric_fallback_aligned_sign(self):
        s = PrintSettings(plate_axis_sign=(1.0, 1.0))
        x, y = resolve_well_xy_mm("C2", self.plate, s)
        self.assertAlmostEqual(x, GEO_C2[0], places=4)
        self.assertAlmostEqual(y, GEO_C2[1], places=4)

    def test_uncalibrated_well_falls_through_to_geometric(self):
        # Map present but missing this well → geometric × sign.
        s = PrintSettings(plate_axis_sign=(-1.0, -1.0),
                          well_positions_mm={"A1": (107.5, 68.6)})
        x, y = resolve_well_xy_mm("C2", self.plate, s)
        self.assertAlmostEqual(x, -GEO_C2[0], places=4)
        self.assertAlmostEqual(y, -GEO_C2[1], places=4)

    def test_malformed_sign_degrades_to_aligned(self):
        s = PrintSettings(plate_axis_sign=("bad", None))
        x, y = resolve_well_xy_mm("C2", self.plate, s)
        self.assertAlmostEqual(x, GEO_C2[0], places=4)
        self.assertAlmostEqual(y, GEO_C2[1], places=4)


class TestPlannerWellXY(unittest.TestCase):
    """PrintTrajectoryPlanner._well_xy must honour the calibrated map once
    settings are captured (generate stores self._settings)."""

    def setUp(self):
        self.plate = _FakePlate()
        self.planner = PrintTrajectoryPlanner()

    def test_pre_generate_is_geometric(self):
        # Before generate(), self._settings is None → legacy geometric path.
        self.planner._plate_axis_sign = (-1.0, -1.0)
        x, y = self.planner._well_xy(self.plate, "C2")
        self.assertAlmostEqual(x, -GEO_C2[0], places=4)

    def test_calibrated_after_settings_capture(self):
        self.planner._settings = PrintSettings(
            plate_axis_sign=(-1.0, -1.0),
            well_positions_mm={"C2": CAL_C2_ZREF_MM})
        x, y = self.planner._well_xy(self.plate, "C2")
        self.assertAlmostEqual(x, CAL_C2_ZREF_MM[0], places=4)
        self.assertAlmostEqual(y, CAL_C2_ZREF_MM[1], places=4)


class TestHybridExecutorWellXY(unittest.TestCase):
    """HybridPlanExecutor._well_xy_mm resolves via the same resolver (no
    controller / hardware needed — settings + plate only)."""

    def _executor(self, settings):
        return HybridPlanExecutor(
            controller=None, plan=None, well_model=None,
            plate=_FakePlate(), path_points=[(0.0, 0.0)], settings=settings)

    def test_calibrated(self):
        ex = self._executor(PrintSettings(
            plate_axis_sign=(-1.0, -1.0),
            well_positions_mm={"C2": CAL_C2_ZREF_MM}))
        x, y = ex._well_xy_mm("C2")
        self.assertAlmostEqual(x, CAL_C2_ZREF_MM[0], places=4)
        self.assertGreater(x, 0.0)

    def test_geometric_fallback(self):
        ex = self._executor(PrintSettings(plate_axis_sign=(-1.0, -1.0)))
        x, y = ex._well_xy_mm("C2")
        self.assertAlmostEqual(x, -GEO_C2[0], places=4)


class TestDiscreteJobLandsOnCalibrated(unittest.TestCase):
    """The discrete builder receives already-resolved zero-ref mm well
    positions; confirm the MOVE_XY lands on the calibrated centre (not the
    negative geometric value that clamps to 0,0)."""

    def test_move_xy_uses_calibrated_center(self):
        s = PrintSettings(num_layers=1)
        # As print_setup_legacy._build_job now does: calibrated zero-ref mm.
        job = build_well_plate_job(
            well_positions=[("C2", CAL_C2_ZREF_MM[0], CAL_C2_ZREF_MM[1])],
            path_points=[(0.0, 0.0), (1.0, 0.0)],
            settings=s)
        move_xy = [c for c in job.commands if c.type == CommandType.MOVE_XY]
        self.assertTrue(move_xy)
        # First MOVE_XY = well centre + first path point (0,0)
        self.assertAlmostEqual(move_xy[0].params["x"], CAL_C2_ZREF_MM[0], places=4)
        self.assertAlmostEqual(move_xy[0].params["y"], CAL_C2_ZREF_MM[1], places=4)
        self.assertGreater(move_xy[0].params["x"], 0.0)
        self.assertGreater(move_xy[0].params["y"], 0.0)


class TestWizardShellForwardsCalibration(unittest.TestCase):
    """Offscreen GUI integration: set_calibration_data on the wizard shell
    reaches the legacy page, and the discrete-builder helper then resolves the
    calibrated well to zero-ref mm (the exact path app.py wires at startup)."""

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        try:
            from PySide6.QtWidgets import QApplication
        except Exception as e:  # pragma: no cover - no Qt available
            raise unittest.SkipTest(f"PySide6 unavailable: {e}")
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _controller(self):
        from unittest.mock import MagicMock
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.plate_axis_sign = lambda: (-1.0, -1.0)
        return ctrl

    def test_forward_and_resolve(self):
        from gui.pages.print_setup import PrintSetupPage  # wizard shell
        page = PrintSetupPage(self._controller(), settings=None)
        # Absolute taught µm (as the Calibration page stores it).
        cal = {"C2": (86028.69, 28274.39), "A1": (107561.6, 68633.6)}
        page.set_calibration_data(_FakePlate(), cal, 27.7)
        # Reached the legacy page that owns the resolver.
        self.assertEqual(page._legacy._calibrated_well_positions, cal)
        # Resolves to the calibrated zero-ref mm (POSITIVE — not the negative
        # geometric value that clamped to 0,0).
        x, y = page._legacy._well_xy_zref_mm("C2", _FakePlate())
        self.assertAlmostEqual(x, 86.02869, places=3)
        self.assertAlmostEqual(y, 28.27439, places=3)
        self.assertGreater(x, 0.0)
        self.assertGreater(y, 0.0)

    def test_clearing_calibration_falls_back_to_geometric(self):
        from gui.pages.print_setup import PrintSetupPage
        page = PrintSetupPage(self._controller(), settings=None)
        page.set_calibration_data(_FakePlate(), {}, None)  # empty → None
        self.assertIsNone(page._legacy._calibrated_well_positions)
        x, y = page._legacy._well_xy_zref_mm("C2", _FakePlate())
        # Geometric × (-1,-1).
        self.assertAlmostEqual(x, -GEO_C2[0], places=3)
        self.assertAlmostEqual(y, -GEO_C2[1], places=3)


class TestTravelAndPrintZFromCalibration(unittest.TestCase):
    """The detached wizard page can't reach app settings, so the safe/travel Z
    and the plate-bottom-derived print Z used to fall back to their defaults
    (travel 5.0 = BELOW the plate on ME3B V1 → 'retract to safe Z' drove the
    needle DOWN; print Z 0.1). They must now come from the calibration:
    travel_z ← set_calibration_data(safe_z); print/top Z ← controller datums."""

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        try:
            from PySide6.QtWidgets import QApplication
        except Exception as e:  # pragma: no cover
            raise unittest.SkipTest(f"PySide6 unavailable: {e}")
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _controller(self):
        from unittest.mock import MagicMock
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.plate_axis_sign.return_value = (-1.0, -1.0)
        # ME3B V1 calibrated datums (zero-ref mm): plate bottom 6.06, top 25.87,
        # safe/fast-move 27.71. z_up = +1 (top > bottom).
        ctrl.get_plate_bottom_z.return_value = 6.06
        ctrl.get_plate_top_z.return_value = 25.87
        ctrl.print_z_dir.return_value = 1.0
        return ctrl

    def _legacy_page(self):
        from gui.pages.print_setup_legacy import PrintSetupPage as _Legacy
        return _Legacy(self._controller(), settings=None)

    def test_safe_z_drives_travel_spin_and_settings(self):
        page = self._legacy_page()
        # Before calibration: the broken default (5.0).
        self.assertAlmostEqual(page.travel_z_spin.value(), 5.0, places=3)
        page.set_calibration_data(None, {"C2": (86028.69, 28274.39)}, 27.71)
        # The travel spin shows 1 decimal, so 27.71 → 27.7 (a 0.01 mm rounding,
        # negligible for a retract height; still far above the plate bottom).
        self.assertAlmostEqual(page.travel_z_spin.value(), 27.7, places=1)
        s = page._get_settings()
        # travel_z is the calibrated safe height — NOT the 5.0 default that
        # would have driven the needle below the plate.
        self.assertAlmostEqual(s.travel_z_height, 27.7, places=1)
        self.assertGreater(s.travel_z_height, 6.06)  # above the plate bottom

    def test_top_z_and_print_z_from_controller(self):
        page = self._legacy_page()
        page.set_calibration_data(None, {}, 27.71)
        s = page._get_settings()
        self.assertAlmostEqual(s.top_z_height, 25.87, places=3)
        # print_z derived from the plate-bottom datum (6.06) + height above it,
        # NOT the bare 0.1 default. With z_up=+1 it sits at/above the bottom.
        self.assertGreaterEqual(s.print_z_height, 6.06)
        self.assertNotAlmostEqual(s.print_z_height, 0.1, places=3)

    def test_plate_bottom_prefers_controller_datum(self):
        page = self._legacy_page()
        self.assertAlmostEqual(page._calibration_plate_bottom_z(), 6.06, places=3)


if __name__ == "__main__":
    unittest.main()
