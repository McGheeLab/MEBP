"""
v7.5.x — Wash needle after reagent pickup (before deposit).

Covers the ``wash_after_pickup`` step inserted into the Cell Targeting &
Removal and Cell Labeling handlers of ``PickAndPlaceManager``: after the needle
aspirates its reagent (trypsin / stain) from the reagent well, it rinses its
EXTERIOR at the wash well before travelling to deposit the reagent. The
aspirated volume stays in the bore, so the reagent-bore pump moves are
unchanged; only an extra wash-well travel leg (+ jiggle) is added.

Executor tests drive the REAL ``PickPlaceExecutor`` against a recording fake
controller (mirrors the sibling cell-removal / cell-labeling suites). The page
tests build the real Qt pages headless (offscreen) and assert the new checkbox
exists + defaults ON.
"""

import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.PickAndPlaceManager import (
    CellLabelingConfig, CellRemovalConfig, OperationQueue, OperationType,
    PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
)


class _RecCtrl:
    """Recording stand-in for StageController (mirrors the sibling tests)."""

    def __init__(self):
        self.calls = []
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    def safe_travel_to(self, **kw):
        self.calls.append(("safe_travel_to", kw))
        return True

    def move_xy_absolute_um(self, x_um, y_um, fast=False):
        self.calls.append(("move_xy_absolute_um", x_um, y_um))

    def move_z_relative(self, dz):
        self.calls.append(("move_z_relative", dz))

    def move_z_user_relative(self, dz):
        self.calls.append(("move_z_user_relative", dz))

    def move_z_absolute(self, z, from_zero_ref=False, feedrate_mm_min=None):
        self.calls.append(("move_z_absolute", z))

    def wait_for_xy_arrival(self, *a, **k):
        self.calls.append(("wait_for_xy_arrival", a, k))
        return True

    def wait_for_z_arrival(self, *a, **k):
        self.calls.append(("wait_for_z_arrival", a, k))
        return True

    def ensure_retracted_to(self, safe_z_zero_ref_mm, *a, **k):
        self.calls.append(("ensure_retracted_to", safe_z_zero_ref_mm))
        return True

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
        self.calls.append(("move_pump_uL", pump, volume_uL, rate_uL_s))


def _travels_xy(ctrl):
    return [(round(t[1]["target_x_um"]), round(t[1]["target_y_um"]))
            for t in ctrl.calls if t[0] == "safe_travel_to"]


# Known coordinates used by the op helpers below.
_REAGENT = (30000.0, 20000.0)
_WASH = (40000.0, 30000.0)
_SRC = (47583.0, 48698.0)      # removal / stain region
_DST = (67439.0, 47722.0)      # cell placement (removal only)
_WASTE = (60000.0, 30000.0)


# ── Cell Targeting & Removal ─────────────────────────────────────────

def _removal_cfg():
    return CellRemovalConfig(
        reagent_bore="P1", release_depth_mm=0.10, release_volume_uL=0.012,
        extract_multiplier=2.0, dwell_time_s=0.0,
        push_speed_uL_s=0.5, pull_speed_uL_s=5.0,
        removal_z_offset_mm=0.10, place_z_offset_mm=0.50)


def _removal_op():
    src = PickPlaceTarget(target_id="P001", x_um=_SRC[0], y_um=_SRC[1],
                          well_name="")
    dst = PickPlaceTarget(target_id="D001", x_um=_DST[0], y_um=_DST[1],
                          well_name="")
    return PickPlaceOperation(
        op_id="OP1", op_type=OperationType.CELL_TARGET_REMOVAL,
        source_target=src, dest_target=dst, config=_removal_cfg())


class TestCellRemovalWashAfterPickup(unittest.TestCase):
    def _run(self, wash_after_pickup, *, wash_pos=_WASH):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0     # removal
        ex.place_z_mm = -14.0
        ex.reagent_well_pos = _REAGENT
        ex.reagent_dip_z_mm = -18.0
        ex.wash_after_pickup = wash_after_pickup
        ex.wash_well_pos = wash_pos
        ex.service_z_mm = -20.0
        ex.wash_cycles = 1
        ex.wash_dwell_s = 0.0
        q = OperationQueue()
        q.add(_removal_op())
        return ctrl, ex.execute_queue(q)

    def test_wash_leg_inserted_between_load_and_removal(self):
        ctrl, ok = self._run(True)
        self.assertTrue(ok)
        xy = _travels_xy(ctrl)
        # reagent load → WASH → removal → placement
        self.assertEqual(len(xy), 4)
        self.assertEqual(
            xy,
            [(30000, 20000), (40000, 30000), (47583, 48698), (67439, 47722)])

    def test_wash_off_by_default_no_extra_leg(self):
        ctrl, _ = self._run(False)
        self.assertEqual(len(_travels_xy(ctrl)), 3)  # reagent, removal, place

    def test_wash_leg_uses_service_dip_z(self):
        ctrl, _ = self._run(True)
        travels = [t for t in ctrl.calls if t[0] == "safe_travel_to"]
        self.assertAlmostEqual(travels[1][1]["target_z_mm"], -20.0)

    def test_wash_noop_without_wash_well(self):
        ctrl, ok = self._run(True, wash_pos=None)
        self.assertTrue(ok)  # graceful no-op, run still completes
        self.assertEqual(len(_travels_xy(ctrl)), 3)

    def test_reagent_pump_moves_unchanged(self):
        ctrl, _ = self._run(True)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        # The wash does NOT pump: still load, push, pull, dispense (4), balanced.
        self.assertEqual(len(pumps), 4)
        self.assertTrue(all(p[1] == "P1" for p in pumps))
        self.assertAlmostEqual(sum(p[2] for p in pumps), 0.0)

    def test_wash_actually_jiggles(self):
        ctrl, _ = self._run(True)
        self.assertTrue(
            any(c[0] == "move_z_user_relative" for c in ctrl.calls))


# ── Cell Labeling ────────────────────────────────────────────────────

def _label_cfg():
    return CellLabelingConfig(
        stain_bore="P1", deposit_depth_mm=0.10, deposit_volume_uL=0.012,
        aspirate_multiplier=2.0, stain_dwell_time_s=0.0,
        deposit_speed_uL_s=0.5, aspirate_speed_uL_s=0.3,
        label_z_offset_mm=0.10)


def _label_op():
    src = PickPlaceTarget(target_id="P001", x_um=_SRC[0], y_um=_SRC[1],
                          well_name="")
    return PickPlaceOperation(
        op_id="OP1", op_type=OperationType.CELL_LABELING,
        source_target=src, dest_target=None, config=_label_cfg())


class TestCellLabelingWashAfterPickup(unittest.TestCase):
    def _run(self, wash_after_pickup, *, wash_pos=_WASH):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0     # label region
        ex.reagent_well_pos = _REAGENT
        ex.reagent_dip_z_mm = -18.0
        ex.waste_well_pos = _WASTE
        ex.service_z_mm = -20.0
        ex.wash_after_pickup = wash_after_pickup
        ex.wash_well_pos = wash_pos
        ex.wash_cycles = 1
        ex.wash_dwell_s = 0.0
        q = OperationQueue()
        q.add(_label_op())
        return ctrl, ex.execute_queue(q)

    def test_wash_leg_inserted_between_load_and_region(self):
        ctrl, ok = self._run(True)
        self.assertTrue(ok)
        xy = _travels_xy(ctrl)
        # stain load → WASH → region → waste dump
        self.assertEqual(len(xy), 4)
        self.assertEqual(
            xy,
            [(30000, 20000), (40000, 30000), (47583, 48698), (60000, 30000)])

    def test_wash_off_by_default_no_extra_leg(self):
        ctrl, _ = self._run(False)
        self.assertEqual(len(_travels_xy(ctrl)), 3)  # stain, region, waste

    def test_wash_noop_without_wash_well(self):
        ctrl, ok = self._run(True, wash_pos=None)
        self.assertTrue(ok)
        self.assertEqual(len(_travels_xy(ctrl)), 3)

    def test_stain_pump_moves_unchanged(self):
        ctrl, _ = self._run(True)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        # load, deposit, aspirate, waste-dispense (4) — balanced, no wash pump.
        self.assertEqual(len(pumps), 4)
        self.assertAlmostEqual(sum(p[2] for p in pumps), 0.0)


# ── GUI pages (offscreen) ────────────────────────────────────────────

class TestPages(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls._tmpdir = tempfile.mkdtemp(prefix="mebp_wf_settings_")
        os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = cls._tmpdir
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.safety_limits = None
        return ctrl

    def test_removal_page_wash_check_default_on(self):
        from gui.pages.workflows.cell_targeting_workflow import (
            CellTargetingWorkflowPage,
        )
        page = CellTargetingWorkflowPage(
            self._ctrl(), settings=None, camera_manager=None)
        self.assertTrue(page._wash_after_pickup_check.isChecked())

    def test_labeling_page_wash_check_default_on(self):
        from gui.pages.workflows.cell_labeling_workflow import (
            CellLabelingWorkflowPage,
        )
        page = CellLabelingWorkflowPage(
            self._ctrl(), settings=None, camera_manager=None)
        self.assertTrue(page._wash_after_pickup_check.isChecked())

    def test_removal_wash_check_persists_off(self):
        # Turning it off + saving via the dialog registry round-trips.
        from gui.pages.workflows.cell_targeting_workflow import (
            CellTargetingWorkflowPage,
        )
        page = CellTargetingWorkflowPage(
            self._ctrl(), settings=None, camera_manager=None)
        page._wash_after_pickup_check.setChecked(False)
        vals = page._settings_dialog.collect()
        self.assertIn("wash_after_pickup", vals)
        self.assertFalse(vals["wash_after_pickup"])


if __name__ == "__main__":
    unittest.main()
