"""
v7.5.x — Re-derive wells from the saved mosaic ground truth under the plate
orientation convention, + explicit calibrated-positions persistence.

The full-plate mosaic is captured at TRUSTED absolute stage positions, so the
detected well CENTRES are ground truth; only the NAME→position binding depends
on the per-machine orientation (StageController.plate_axis_sign). When the
orientation flips 180° (ME3B V1) the previous taught/warp calibration is stale;
this re-derives every well straight from the ground-truth centres, relabelled
for the current orientation, stored DIRECTLY (no warp), and persisted via a new
explicit ``calibrated_positions`` key.

Covers:
  * MosaicWellRemap.label_positions — orientation-aware relabel of fixed
    ground-truth positions ((-1,-1) → A1 at max corner; (1,1) → min corner).
  * CalibrationPage._ploc_rederive_from_saved_mosaic — clears stale cal, stores
    the relabelled dict directly, no warp, A1 at the max corner.
  * _save_calibration / _load_calibration — explicit calibrated_positions
    round-trips with NO warp and SUPPRESSES warp/affine reconstruction.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from SupportClasses.WellPlate import WellPlate  # noqa: E402
from SupportClasses.MosaicWellRemap import label_positions  # noqa: E402


def _app():
    return QApplication.instance() or QApplication(sys.argv)


def _aligned_grid(plate, ox=8000.0, oy=10000.0, pitch_um=19300.0):
    """A clean ground-truth grid in the ALIGNED frame: A1 at (ox,oy)=min
    corner, +col→+X, +row→+Y. Returns {name:(x,y)} (the detected centres)."""
    g = {}
    for w in plate.get_all_wells():
        g[w.name] = (ox + w.col * pitch_um, oy + w.row * pitch_um)
    return g


# ── 1. label_positions: orientation-aware relabel of fixed positions ──

class TestLabelPositions(unittest.TestCase):
    def setUp(self):
        self.plate = WellPlate.from_format(24)   # 4 rows × 6 cols
        self.grid = _aligned_grid(self.plate)
        self.positions = list(self.grid.values())

    def test_aligned_sign_is_identity_labeling(self):
        out = label_positions(self.positions, self.plate, (1.0, 1.0))
        self.assertEqual(len(out), 24)
        # A1 at the min corner; matches the aligned grid exactly.
        for name, xy in self.grid.items():
            self.assertAlmostEqual(out[name][0], xy[0], places=3)
            self.assertAlmostEqual(out[name][1], xy[1], places=3)

    def test_flip_puts_a1_at_max_corner(self):
        out = label_positions(self.positions, self.plate, (-1.0, -1.0))
        self.assertEqual(len(out), 24)
        xs = [p[0] for p in self.positions]
        ys = [p[1] for p in self.positions]
        # New A1 = the (max X, max Y) detection = the aligned grid's D6.
        self.assertAlmostEqual(out["A1"][0], max(xs), places=3)
        self.assertAlmostEqual(out["A1"][1], max(ys), places=3)
        self.assertEqual(out["A1"], self.grid["D6"])

    def test_flip_is_180_rotation_of_names(self):
        out = label_positions(self.positions, self.plate, (-1.0, -1.0))
        R, C = self.plate.rows, self.plate.cols
        from SupportClasses.WellPlate import ROW_LABELS
        # NEW(r,c) position == OLD(R-1-r, C-1-c) position.
        for w in self.plate.get_all_wells():
            old_name = f"{ROW_LABELS[R - 1 - w.row]}{(C - 1 - w.col) + 1}"
            self.assertEqual(out[w.name], self.grid[old_name],
                             f"{w.name} should map to old {old_name}")

    def test_positions_are_a_permutation(self):
        out = label_positions(self.positions, self.plate, (-1.0, -1.0))
        self.assertEqual(sorted(out.values()), sorted(self.positions))

    def test_real_ground_truth_a1_is_max_corner(self):
        # The actual ME3B V1 24-well detections (min-corner A1 in the old frame).
        real = {
            "A1": (8595.0, 10054.2), "A6": (104866.7, 10200.0),
            "D1": (8481.6, 67969.7), "D6": (104915.0, 68043.9),
        }
        # Fill the rest with a synthetic grid so it's a complete 24.
        full = _aligned_grid(self.plate)
        full.update(real)
        out = label_positions(list(full.values()), self.plate, (-1.0, -1.0))
        # New A1 must be the old max-corner (old D6).
        self.assertEqual(out["A1"], full["D6"])


# ── 2. CalibrationPage._ploc_rederive_from_saved_mosaic ──

class TestRederive(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _page(self, sign=(-1.0, -1.0)):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.default_plate_center_um.return_value = (56000.0, 39000.0)
        ctrl.plate_axis_sign.return_value = sign
        ctrl.plate_flip_180.return_value = (sign[0] < 0)
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(24)
        # Isolate from the on-disk mosaic store (use only Source A markers).
        page._ploc_mosaic_store = lambda: None
        return page, ctrl

    def test_rederive_relabels_and_drops_warp(self):
        page, _ = self._page(sign=(-1.0, -1.0))
        grid = _aligned_grid(page._plate)
        # Pre-existing (stale) state: old labeling markers + a bogus warp.
        page._reference_markers = dict(grid)
        page._plate_warp = MagicMock()           # the stale 79mm warp
        page._taught_a1 = grid["A1"]             # old min-corner A1

        page._ploc_rederive_from_saved_mosaic()

        cp = page._calibrated_positions
        self.assertIsNotNone(cp)
        self.assertEqual(len(cp), 24)
        # New A1 = old max corner; warp discarded.
        self.assertEqual(cp["A1"], grid["D6"])
        self.assertIsNone(page._plate_warp)
        self.assertIsNone(page._three_well_calibration)
        # taught_a1 follows the new A1 so _has_plate_calibration stays True.
        self.assertEqual(tuple(page._taught_a1), grid["D6"])
        self.assertTrue(page._has_plate_calibration())

    def test_rederive_aligned_keeps_min_corner(self):
        page, _ = self._page(sign=(1.0, 1.0))
        grid = _aligned_grid(page._plate)
        page._reference_markers = dict(grid)
        page._ploc_rederive_from_saved_mosaic()
        self.assertEqual(page._calibrated_positions["A1"], grid["A1"])

    def test_rederive_no_source_is_safe(self):
        page, _ = self._page()
        page._reference_markers = {}
        page._ploc_well_results = {}
        # No markers and no mosaic store → no-op, no exception, no cal.
        page._ploc_rederive_from_saved_mosaic()
        self.assertFalse(page._calibrated_positions)


# ── 3. Explicit calibrated_positions persistence round-trip ──

class _Settings:
    def __init__(self):
        self._sections = {}

    def get_section(self, name):
        return self._sections.get(name)

    def set_section(self, name, value):
        self._sections[name] = value

    def get(self, key, default=None):
        return default

    def save(self):
        pass


class TestCalibratedPositionsPersistence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.plate_axis_sign.return_value = (-1.0, -1.0)
        ctrl.plate_flip_180.return_value = True
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        ctrl.default_plate_center_um.return_value = (56000.0, 39000.0)
        return ctrl

    def setUp(self):
        # Isolate the durable snapshot store to a tempdir (so _save_calibration
        # doesn't clobber the real last_calibration.json).
        import SupportClasses.CalibrationSnapshotStore as snapmod
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self._prev_snap = getattr(snapmod, "_store", None)
        snapmod._store = snapmod.CalibrationSnapshotStore(
            Path(self._tmp.name) / "lc.json")
        self.addCleanup(lambda: setattr(snapmod, "_store", self._prev_snap))

    def _page(self, settings):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage(self._ctrl(), settings=settings)
        page._plate = WellPlate.from_format(24)
        return page

    def test_explicit_positions_roundtrip_no_warp(self):
        settings = _Settings()
        plate = WellPlate.from_format(24)
        grid = label_positions(list(_aligned_grid(plate).values()),
                               plate, (-1.0, -1.0))

        p1 = self._page(settings)
        p1._calibrated_positions = dict(grid)
        p1._reference_markers = dict(grid)
        p1._taught_a1 = grid["A1"]
        p1._plate_warp = None
        p1._save_calibration()

        cal = settings.get_section("calibration")
        self.assertIn("calibrated_positions", cal)
        self.assertEqual(len(cal["calibrated_positions"]), 24)
        self.assertNotIn("plate_warp", cal)
        self.assertEqual(cal.get("plate_flip_180"), True)

        # Fresh page loads it back — explicit dict restored, NO warp.
        p2 = self._page(settings)
        p2._load_calibration()
        self.assertIsNotNone(p2._calibrated_positions)
        self.assertEqual(len(p2._calibrated_positions), 24)
        self.assertIsNone(p2._plate_warp)
        self.assertAlmostEqual(p2._calibrated_positions["A1"][0],
                               grid["A1"][0], places=2)
        self.assertAlmostEqual(p2._calibrated_positions["A1"][1],
                               grid["A1"][1], places=2)

    def test_explicit_positions_dropped_on_orientation_flip(self):
        # Saved under flip=True; loaded under flip=False → orientation guard
        # discards the explicit dict (orientation-dependent) and forces re-teach.
        settings = _Settings()
        plate = WellPlate.from_format(24)
        grid = label_positions(list(_aligned_grid(plate).values()),
                               plate, (-1.0, -1.0))
        p1 = self._page(settings)
        p1._calibrated_positions = dict(grid)
        p1._taught_a1 = grid["A1"]
        p1._plate_warp = None
        p1._save_calibration()
        self.assertEqual(settings.get_section("calibration")
                         .get("plate_flip_180"), True)

        # Load with a controller whose orientation is now aligned (flip False).
        from gui.pages.calibration import CalibrationPage
        ctrl2 = self._ctrl()
        ctrl2.plate_axis_sign.return_value = (1.0, 1.0)
        ctrl2.plate_flip_180.return_value = False
        p2 = CalibrationPage(ctrl2, settings=settings)
        p2._plate = WellPlate.from_format(24)
        p2._load_calibration()
        # The explicit calibrated_positions was popped by the guard.
        self.assertFalse(p2._calibrated_positions)


# ── 4. _CalibrationPlateView draws dots BY NAME (robust to taught_a1 desync) ──

class TestCalibrationPlateViewDots(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _dot_centers(self, view, kind):
        """Map well name → (scene_x, scene_y) for dots of the given kind."""
        out = {}
        for it in view._scene.items():
            tip = it.toolTip() if hasattr(it, "toolTip") else ""
            if tip.endswith(f"({kind})"):
                name = tip.split(" ")[0]
                r = it.rect()
                out[name] = (r.center().x(), r.center().y())
        return out

    def test_calibrated_dots_on_grid_cells_regardless_of_taught_a1(self):
        from gui.pages.calibration import _CalibrationPlateView
        plate = WellPlate.from_format(24)
        # Calibrated A1 at the MAX corner (post-remap); D6 at the min corner.
        cp = label_positions(list(_aligned_grid(plate).values()),
                             plate, (-1.0, -1.0))
        S = _CalibrationPlateView.SCALE

        def centers_for(taught_a1):
            v = _CalibrationPlateView()
            v.set_plate(plate)
            v.set_taught_a1(taught_a1)          # may be stale / desynced
            v.set_calibrated_positions(cp)
            return self._dot_centers(v, "calibrated")

        a1w = plate.get_well_info("A1")
        d6w = plate.get_well_info("D6")
        # A deliberately DESYNCED taught_a1 (the min corner = D6's position).
        c = centers_for((8595.0, 10054.0))
        self.assertIn("A1", c)
        self.assertIn("D6", c)
        # A1 dot sits on A1's grid cell (top-left, 0,0); D6 on its cell.
        self.assertAlmostEqual(c["A1"][0], a1w.x * S, places=3)
        self.assertAlmostEqual(c["A1"][1], a1w.y * S, places=3)
        self.assertAlmostEqual(c["D6"][0], d6w.x * S, places=3)
        self.assertAlmostEqual(c["D6"][1], d6w.y * S, places=3)
        # A1 is top-left of D6.
        self.assertLess(c["A1"][0], c["D6"][0])
        self.assertLess(c["A1"][1], c["D6"][1])
        # Dots are INDEPENDENT of taught_a1 (by name) — a different anchor
        # yields identical placement.
        c2 = centers_for((104915.0, 68044.0))
        self.assertEqual(c, c2)


# ── 5. Whole-plate mosaic scan defaults to the full XY envelope ──

class TestWholePlateScanBounds(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=None)
        page._plate = WellPlate.from_format(24)
        return page

    def test_default_bounds_are_full_envelope(self):
        page = self._page()
        env = (0.0, 0.0, 114332.0, 76645.0)
        # Well centres occupy only part of the envelope...
        positions = {"A1": (8595.0, 10054.0), "D6": (104915.0, 68044.0)}
        bounds = page._ploc_whole_plate_scan_bounds(positions, env)
        # ...but the scan region is the WHOLE reachable envelope.
        self.assertEqual(bounds, env)

    def test_fallback_to_well_extent_without_envelope(self):
        page = self._page()
        positions = {"A1": (8595.0, 10054.0), "D6": (104915.0, 68044.0)}
        bounds = page._ploc_whole_plate_scan_bounds(positions, None)
        # No envelope → well-centre extent + margin (well radius + 1 mm).
        self.assertIsNotNone(bounds)
        self.assertLess(bounds[0], 8595.0)   # min x padded below A1
        self.assertGreater(bounds[2], 104915.0)  # max x padded above D6

    def test_full_envelope_raster_spans_xy(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        page = self._page()
        env = (0.0, 0.0, 114332.0, 76645.0)
        bounds = page._ploc_whole_plate_scan_bounds({"A1": (0.0, 0.0)}, env)
        b = MosaicBuilder(frame_size_px=(640, 480), micron_per_pixel=3.0,
                          overlap=0.25, register=False)
        grid = b.generate_raster_positions(bounds, overlap=0.25)
        xs = [g[0] for g in grid]
        ys = [g[1] for g in grid]
        # The raster reaches well into both far ends of the envelope (not just
        # a small well cluster) — half-FOV inset keeps centres inside.
        self.assertGreater(max(xs), env[2] * 0.7)
        self.assertGreater(max(ys), env[3] * 0.7)
        self.assertLess(min(xs), env[2] * 0.3)
        self.assertLess(min(ys), env[3] * 0.3)


if __name__ == "__main__":
    unittest.main()
