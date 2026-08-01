"""
v7.5.x — the calibrated well map must survive a restart aligned to the
full-plate mosaic (operator: "every time I start the software, the wells are
not mapped correctly to the mosaic … I have to press map wells each time").

Root cause: the ``plate_warp`` persistence contract re-applies the warp to a
RECOMPUTED predicted grid at load time, but ``set_hardware_config`` re-seeded
``_predicted_positions`` with the geometry-only ENVELOPE-CENTRED grid on
every config push — so the Map-wells warp was fitted against the envelope
frame while ``_load_calibration`` rebuilt an A1-anchored frame and evaluated
the warp there, shifting every well by the A1 prediction error (~0.68 mm on
the reporting machine) after each restart.

Covers the two fixes:
  * set_hardware_config keeps the prediction frame stable — envelope seed
    only when NOTHING is calibrated; taught-A1-anchored rebuild otherwise.
  * _save_calibration persists the explicit ``calibrated_positions`` also
    when a warp exists; _load_calibration restores them as the authoritative
    positions while still restoring the warp object (re-anchor needs it),
    and only reconstructs positions from the warp for legacy saves.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PySide6.QtWidgets import QApplication  # noqa: E402

from SupportClasses.WellPlate import WellPlate  # noqa: E402

# The operator's real numbers (settings.json, 2026-07-29): the warp source
# grid was envelope-anchored while taught_a1 was the measured A1 — off by:
DELTA = (624.055, -277.0)
ENVELOPE_CENTER = (57166.0, 38322.5)


def _app():
    return QApplication.instance() or QApplication(sys.argv)


def _ctrl(sign=(1.0, 1.0)):
    ctrl = MagicMock()
    ctrl.is_xy_connected = False
    ctrl.is_zp_connected = False
    ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
    ctrl.get_xy_position.return_value = (None, None)
    ctrl.default_plate_center_um.return_value = ENVELOPE_CENTER
    ctrl.plate_axis_sign.return_value = sign
    ctrl.plate_flip_180.return_value = (sign[0] < 0)
    ctrl.z_up_sign.return_value = -1.0
    ctrl.raw_to_user_z.side_effect = lambda v: -v
    return ctrl


class _Settings:
    """Minimal Settings stand-in for the save/load funnel."""

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


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def setUp(self):
        # Isolate the durable stores so _save_calibration never touches the
        # machine's real last_calibration.json / calibration_status.json.
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        import SupportClasses.CalibrationSnapshotStore as snapmod
        self._prev_snap = getattr(snapmod, "_store", None)
        snapmod._store = snapmod.CalibrationSnapshotStore(
            Path(self._tmp.name) / "lc.json")
        self.addCleanup(lambda: setattr(snapmod, "_store", self._prev_snap))
        import SupportClasses.CalibrationStatusStore as statmod
        self._prev_stat = getattr(statmod, "_store", None)
        statmod._store = statmod.CalibrationStatusStore(
            Path(self._tmp.name) / "status")
        self.addCleanup(lambda: setattr(statmod, "_store", self._prev_stat))

    def _page(self, settings=None, sign=(1.0, 1.0)):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage(_ctrl(sign), settings=settings)
        page._plate = WellPlate.from_format(24)
        # Keep the tests off the machine's real plate_mosaics.json.
        page._ploc_mosaic_store = lambda: None
        return page


# ── 1. set_hardware_config keeps the prediction frame stable ──────────

def _config24():
    return SimpleNamespace(active_plate_key=24, geometry_plate_key=None,
                           plate_format=24, camera_config=None,
                           camera_for_role=lambda role: None)


class TestConfigPushPredictionFrame(_Base):
    def test_calibrated_page_keeps_a1_anchored_frame(self):
        page = self._page()
        page._loaded_cal_key = "24"          # same-key push (no re-activate)
        a1 = (105617.55, 65890.35)           # measured A1 (taught)
        page._taught_a1 = a1
        page._calibrated_positions = {
            n: (x + DELTA[0], y + DELTA[1])
            for n, (x, y) in page._plate.get_all_positions_from_a1(
                *a1).items()}

        page.set_hardware_config(_config24())

        # Predictions rebuilt ANCHORED AT THE TAUGHT A1 — the same frame
        # _load_calibration reconstructs — not the envelope-centred default.
        self.assertIsNotNone(page._predicted_positions)
        self.assertAlmostEqual(page._predicted_positions["A1"][0], a1[0],
                               places=3)
        self.assertAlmostEqual(page._predicted_positions["A1"][1], a1[1],
                               places=3)

    def test_uncalibrated_page_keeps_envelope_seed(self):
        page = self._page()
        page._loaded_cal_key = "24"
        page._taught_a1 = None
        page._calibrated_positions = None
        page._three_well_calibration = None

        page.set_hardware_config(_config24())

        expected = page._plate.get_all_positions_from_plate_center(
            *ENVELOPE_CENTER)
        self.assertIsNotNone(page._predicted_positions)
        self.assertAlmostEqual(page._predicted_positions["A1"][0],
                               expected["A1"][0], places=3)
        self.assertAlmostEqual(page._predicted_positions["A1"][1],
                               expected["A1"][1], places=3)

    def test_calibrated_without_a1_leaves_predictions_alone(self):
        page = self._page()
        page._loaded_cal_key = "24"
        page._taught_a1 = None
        # Calibrated map present (matches the plate's wells) but no taught A1
        # — the grid must not be moved under it.
        page._calibrated_positions = dict(
            page._plate.get_all_positions_from_plate_center(
                *ENVELOPE_CENTER))
        sentinel = {"A1": (123.0, 456.0)}
        page._predicted_positions = dict(sentinel)

        page.set_hardware_config(_config24())

        self.assertEqual(page._predicted_positions, sentinel)


# ── 2. Warp save persists the explicit positions; load restores them ──

class TestWarpSaveRestoresExactPositions(_Base):
    def _fit_against_envelope_frame(self, page):
        """Reproduce the operator's on-disk state: the warp fitted against
        the ENVELOPE-anchored predicted grid, taught_a1 = the MEASURED A1."""
        predicted = page._plate.get_all_positions_from_plate_center(
            *ENVELOPE_CENTER)
        measured = {n: (x + DELTA[0], y + DELTA[1])
                    for n, (x, y) in predicted.items()}
        page._predicted_positions = dict(predicted)
        page._xy_teach_points = dict(measured)
        page._taught_a1 = measured["A1"]          # what _ploc_feed_affine does
        page._manual_fit_xy()
        return measured

    def test_save_writes_both_warp_and_positions(self):
        settings = _Settings()
        page = self._page(settings)
        measured = self._fit_against_envelope_frame(page)
        self.assertIsNotNone(page._plate_warp)

        page._save_calibration()
        cal = settings.get_section("calibration")
        self.assertIn("plate_warp", cal)
        self.assertIn("calibrated_positions", cal)
        self.assertEqual(len(cal["calibrated_positions"]), 24)
        self.assertAlmostEqual(cal["calibrated_positions"]["A1"][0],
                               measured["A1"][0], delta=1.0)

    def test_reload_reproduces_mapping_despite_frame_shift(self):
        """The operator's exact failure: fit frame ≠ load frame. The reload
        must still land every well on the MEASURED mapping."""
        settings = _Settings()
        page1 = self._page(settings)
        measured = self._fit_against_envelope_frame(page1)
        # In-session result is exact at the control wells.
        for n in ("A1", "D6", "B3"):
            self.assertAlmostEqual(page1._calibrated_positions[n][0],
                                   measured[n][0], delta=1.0)
        page1._save_calibration()

        page2 = self._page(settings)
        page2._load_calibration()

        # The warp object is restored (re-anchor folding needs it) …
        self.assertIsNotNone(page2._plate_warp)
        # … and the legacy reconstruction (warp on the RECOMPUTED A1-anchored
        # grid) WOULD have been off by ~DELTA — the pre-fix startup bug:
        legacy = page2._plate_warp.correct_positions(
            page2._predicted_positions)
        self.assertGreater(abs(legacy["A1"][0] - measured["A1"][0]), 100.0)
        # … but the restored calibration is the explicit measured mapping.
        for n in ("A1", "D6", "B3"):
            self.assertAlmostEqual(page2._calibrated_positions[n][0],
                                   measured[n][0], delta=1.0)
            self.assertAlmostEqual(page2._calibrated_positions[n][1],
                                   measured[n][1], delta=1.0)

    def test_legacy_save_without_positions_still_reconstructs(self):
        """A pre-fix settings.json (warp only, frames consistent) keeps
        loading through the warp reconstruction."""
        settings = _Settings()
        page1 = self._page(settings)
        # Frame-consistent legacy save: predicted anchored at the taught A1.
        a1 = (105617.55, 65890.35)
        predicted = page1._plate.get_all_positions_from_a1(*a1)
        measured = {n: (x + (5.0 if n != "A1" else 0.0), y)
                    for n, (x, y) in predicted.items()}
        page1._predicted_positions = dict(predicted)
        page1._xy_teach_points = dict(measured)
        page1._taught_a1 = a1
        page1._manual_fit_xy()
        page1._save_calibration()
        # Simulate the legacy file: no explicit positions stored.
        cal = settings.get_section("calibration")
        cal.pop("calibrated_positions", None)

        page2 = self._page(settings)
        page2._load_calibration()
        self.assertIsNotNone(page2._plate_warp)
        self.assertIsNotNone(page2._calibrated_positions)
        self.assertAlmostEqual(page2._calibrated_positions["D6"][0],
                               measured["D6"][0], delta=1.5)


if __name__ == "__main__":
    unittest.main()
