"""
v7.5.x — Cell Labeling / staining workflow.

Covers the deposit-incubate-aspirate-to-waste operation (the new
``OperationType.CELL_LABELING`` handler + ``CellLabelingConfig`` in
``PickAndPlaceManager``) and the workflow page gating
(``gui/pages/workflows/cell_labeling_workflow.py``), plus the ``pick_only``
mode added to ``LiveTargetPicker``.

The executor tests drive the REAL ``PickPlaceExecutor`` against a recording
fake controller — no Qt / hardware. The page tests build the real Qt page
headless (offscreen).
"""

import math
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.PickAndPlaceManager import (
    CellLabelingConfig, OperationQueue, OperationType,
    PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
)
from SupportClasses.PhysicalModels import NeedleSpec


def _needle(id_um=200.0, length_inches=0.5) -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=400.0, id_um=id_um, wall_um=100.0,
                      length_inches=length_inches)


class _RecCtrl:
    """Recording stand-in for StageController (mirrors the cell-removal test)."""

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


def _cfg(**over) -> CellLabelingConfig:
    base = dict(
        stain_bore="P1", deposit_depth_mm=0.10, deposit_volume_uL=0.012,
        aspirate_multiplier=2.0, stain_dwell_time_s=0.0,  # 0 dwell = no sleep
        deposit_speed_uL_s=0.5, aspirate_speed_uL_s=0.3,
        label_z_offset_mm=0.10,
    )
    base.update(over)
    return CellLabelingConfig(**base)


def _op(cfg=None, well_a=""):
    cfg = cfg or _cfg()
    src = PickPlaceTarget(target_id="P001", x_um=47583.0, y_um=48698.0,
                          well_name=well_a)
    return PickPlaceOperation(
        op_id="OP1", op_type=OperationType.CELL_LABELING,
        source_target=src, dest_target=None, config=cfg)


# ── Config math ──────────────────────────────────────────────────────

class TestConfigVolumes(unittest.TestCase):
    def test_deposit_volume_from_needle_bore(self):
        n = _needle(id_um=200.0)  # 0.2 mm bore
        cfg = _cfg(deposit_depth_mm=0.10)
        area = math.pi * (0.1 ** 2)  # π(id/2)², mm²
        self.assertAlmostEqual(cfg.compute_deposit_volume_uL(n), area * 0.10)

    def test_deposit_volume_fallback_without_needle(self):
        cfg = _cfg(deposit_volume_uL=0.034)
        self.assertAlmostEqual(cfg.compute_deposit_volume_uL(None), 0.034)

    def test_aspirate_is_multiple_of_deposit(self):
        n = _needle()
        cfg = _cfg(aspirate_multiplier=2.0)
        self.assertAlmostEqual(
            cfg.compute_aspirate_volume_uL(n),
            2.0 * cfg.compute_deposit_volume_uL(n))

    def test_to_dict_round_trip(self):
        cfg = _cfg(stain_dwell_time_s=600.0)
        d = cfg.to_dict()
        self.assertEqual(d["stain_dwell_time_s"], 600.0)
        self.assertEqual(d["aspirate_multiplier"], 2.0)
        self.assertEqual(CellLabelingConfig(**d).stain_dwell_time_s, 600.0)


# ── Operation handler ────────────────────────────────────────────────

class TestCellLabelingExecution(unittest.TestCase):
    def _run(self, *, reagent_pos=(30000.0, 20000.0), reagent_z=-18.0,
             label_z=-16.0, waste_pos=(60000.0, 30000.0), service_z=-20.0,
             cfg=None):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = label_z
        ex.reagent_well_pos = reagent_pos
        ex.reagent_dip_z_mm = reagent_z
        ex.waste_well_pos = waste_pos
        ex.service_z_mm = service_z
        q = OperationQueue()
        q.add(_op(cfg=cfg))
        ok = ex.execute_queue(q)
        return ctrl, ok

    def test_pump_sequence_load_deposit_aspirate_waste(self):
        cfg = _cfg(deposit_volume_uL=0.012, aspirate_multiplier=2.0)
        ctrl, ok = self._run(cfg=cfg)
        self.assertTrue(ok)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        # load(-deposit), deposit(+deposit), aspirate(-asp), waste-dispense(+asp)
        self.assertEqual(len(pumps), 4)
        self.assertAlmostEqual(pumps[0][2], -0.012)  # load (draw stain)
        self.assertAlmostEqual(pumps[1][2], +0.012)  # deposit (slow)
        self.assertAlmostEqual(pumps[2][2], -0.024)  # aspirate back (slow)
        self.assertAlmostEqual(pumps[3][2], +0.024)  # dump to waste
        self.assertTrue(all(p[1] == "P1" for p in pumps))

    def test_pump_is_volume_balanced(self):
        ctrl, _ = self._run(cfg=_cfg(deposit_volume_uL=0.02))
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertAlmostEqual(sum(p[2] for p in pumps), 0.0)

    def test_deposit_and_aspirate_both_slow(self):
        cfg = _cfg(deposit_speed_uL_s=0.5, aspirate_speed_uL_s=0.3)
        ctrl, _ = self._run(cfg=cfg)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertEqual(pumps[0][3], 0.3)   # load @ aspirate speed (slow)
        self.assertEqual(pumps[1][3], 0.5)   # deposit @ deposit speed (slow)
        self.assertEqual(pumps[2][3], 0.3)   # aspirate @ aspirate speed (slow)
        self.assertEqual(pumps[3][3], 0.5)   # waste dispense @ deposit speed

    def test_three_full_safe_travels_reagent_region_waste(self):
        ctrl, _ = self._run(reagent_z=-18.0, label_z=-16.0, service_z=-20.0)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        # reagent load, stain region, waste dump = 3 (no place target).
        self.assertEqual(len(travels), 3)
        self.assertAlmostEqual(travels[0][1]["target_z_mm"], -18.0)  # reagent dip
        self.assertAlmostEqual(travels[1][1]["target_z_mm"], -16.0)  # label/region
        self.assertAlmostEqual(travels[2][1]["target_z_mm"], -20.0)  # waste dip

    def test_waste_dump_targets_waste_well(self):
        ctrl, _ = self._run(waste_pos=(61234.0, 29876.0))
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(
            (round(travels[-1][1]["target_x_um"]),
             round(travels[-1][1]["target_y_um"])), (61234, 29876))

    def test_no_place_travel(self):
        # dest_target is None — only reagent + region + waste travels, never a
        # 4th "placement" leg.
        ctrl, _ = self._run()
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 3)

    def test_ends_at_safe_z(self):
        ctrl, ok = self._run()
        self.assertTrue(ok)
        self.assertEqual(ctrl.calls[-1][0], "ensure_retracted_to")
        self.assertAlmostEqual(ctrl.calls[-1][1], -35.0)

    def test_no_waste_well_skips_dump(self):
        # waste_well_pos unset → the aspirate still happens but the waste dump
        # leg is skipped (GUI gates this up front; graceful backstop).
        ctrl, _ = self._run(waste_pos=None)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 2)  # reagent + region only
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertEqual(len(pumps), 3)    # load, deposit, aspirate (no dump)

    def test_zp_disconnected_refuses(self):
        ctrl = _RecCtrl()
        ctrl.is_zp_connected = False
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.reagent_well_pos = (30000.0, 20000.0)
        ex.reagent_dip_z_mm = -18.0
        ex.waste_well_pos = (60000.0, 30000.0)
        ex.service_z_mm = -20.0
        q = OperationQueue()
        q.add(_op())
        ex.execute_queue(q)  # internal abort, must not raise
        self.assertEqual(
            [c for c in ctrl.calls
             if c[0] in ("safe_travel_to", "move_xy_absolute_um")], [])

    def test_multiple_regions(self):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.reagent_well_pos = (30000.0, 20000.0)
        ex.reagent_dip_z_mm = -18.0
        ex.waste_well_pos = (60000.0, 30000.0)
        ex.service_z_mm = -20.0
        q = OperationQueue()
        for i in range(3):
            src = PickPlaceTarget(target_id=f"P00{i}", x_um=1000.0 * i,
                                  y_um=2000.0 * i, well_name="")
            q.add(PickPlaceOperation(
                op_id=f"OP{i}", op_type=OperationType.CELL_LABELING,
                source_target=src, dest_target=None, config=_cfg()))
        self.assertTrue(ex.execute_queue(q))
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 9)  # 3 per region × 3 regions


# ── Prep + post-clean bracketing ─────────────────────────────────────

class TestPrepAndPostClean(unittest.TestCase):
    def _exec(self, needle_uL=0.40, cycles=2):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.reagent_well_pos = (25000.0, 25000.0)
        ex.reagent_dip_z_mm = -18.0
        ex.prep_bore = "P1"
        ex.needle_volume_uL = needle_uL
        ex.oil_needles = 1.0
        ex.buffer_needles = 4.0
        ex.post_dispense_needles = 1.0
        ex.service_z_mm = -20.0
        ex.wash_cycles = cycles
        ex.wash_dwell_s = 0.0
        ex.waste_well_pos = (60000.0, 30000.0)
        ex.oil_well_pos = (50000.0, 30000.0)
        ex.wash_well_pos = (40000.0, 30000.0)
        ex.buffer_well_pos = (30000.0, 30000.0)
        return ctrl, ex

    def test_prep_then_op_then_clean_travel_order(self):
        ctrl, ex = self._exec()
        ex.do_prep = True
        ex.do_post_clean = True
        q = OperationQueue()
        q.add(_op())
        self.assertTrue(ex.execute_queue(q))
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        xy = [(round(t[1]["target_x_um"]), round(t[1]["target_y_um"]))
              for t in travels]
        # prep: waste, oil, wash, buffer (4)
        # op:   reagent, region, waste-dump (3)
        # clean: waste (intra-well — needle is already at waste from the op's
        #        dump, so NOT a safe_travel), wash, buffer (2 safe_travels)
        self.assertEqual(len(travels), 9)
        self.assertEqual(xy[:4], [(60000, 30000), (50000, 30000),
                                  (40000, 30000), (30000, 30000)])
        self.assertEqual(xy[4], (25000, 25000))   # reagent load first in the op
        self.assertEqual(xy[6], (60000, 30000))   # op's waste dump
        self.assertEqual(xy[7:], [(40000, 30000),
                                  (30000, 30000)])  # clean: wash, buffer

    def test_staining_only_no_prep_no_clean(self):
        ctrl, ex = self._exec()
        ex.do_prep = False
        ex.do_post_clean = False
        q = OperationQueue()
        q.add(_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 3)  # reagent + region + waste only


# ── pick_only LiveTargetPicker ───────────────────────────────────────

class TestPickOnlyPicker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_pick_only_forces_pick_mode(self):
        from gui.widgets.live_target_picker import LiveTargetPicker
        ctrl = MagicMock()
        ctrl.get_xy_position.return_value = (0.0, 0.0)
        picker = LiveTargetPicker(ctrl, None, pick_only=True)
        self.assertTrue(picker._pick_only)
        self.assertEqual(picker._mode, LiveTargetPicker.MODE_PICK)
        # The place list is hidden in pick_only mode (its section was hidden).
        self.assertFalse(picker._place_list.isVisibleTo(picker))
        # Adding a target stays in the pick list; the place list stays empty.
        picker._add_pick(100.0, 200.0)
        self.assertEqual(len(picker.picks()), 1)
        self.assertEqual(len(picker.places()), 0)

    def test_normal_picker_shows_place(self):
        from gui.widgets.live_target_picker import LiveTargetPicker
        ctrl = MagicMock()
        ctrl.get_xy_position.return_value = (0.0, 0.0)
        picker = LiveTargetPicker(ctrl, None)
        self.assertFalse(picker._pick_only)


# ── GUI page (offscreen) ─────────────────────────────────────────────

class TestPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls._tmpdir = tempfile.mkdtemp(prefix="mebp_wf_settings_")
        os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = cls._tmpdir
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_page(self):
        from gui.pages.workflows.cell_labeling_workflow import (
            CellLabelingWorkflowPage,
        )
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.safety_limits = None
        return CellLabelingWorkflowPage(ctrl, settings=None, camera_manager=None)

    def test_constructs(self):
        page = self._make_page()
        self.assertEqual(page.get_page_title(), "Cell Labeling")

    def test_picker_is_pick_only(self):
        page = self._make_page()
        self.assertTrue(page._picker._pick_only)

    def test_deposit_aspirate_volume_from_needle(self):
        page = self._make_page()
        page._hw_config = SimpleNamespace(needle=_needle(id_um=200.0))
        page._deposit_depth.setValue(0.10)
        page._aspirate_mult.setValue(2.0)
        deposit, aspirate = page._deposit_aspirate_uL()
        self.assertAlmostEqual(deposit, math.pi * (0.1 ** 2) * 0.10)
        self.assertAlmostEqual(aspirate, 2.0 * deposit)

    def test_current_config_reflects_spins(self):
        page = self._make_page()
        page._hw_config = SimpleNamespace(needle=_needle())
        page._label_z.setValue(0.20)
        page._dwell.setValue(900.0)
        page._aspirate_mult.setValue(3.0)
        page._deposit_speed.setValue(0.4)
        page._aspirate_speed.setValue(0.2)
        cfg = page._current_config()
        self.assertAlmostEqual(cfg.label_z_offset_mm, 0.20)
        self.assertAlmostEqual(cfg.stain_dwell_time_s, 900.0)
        self.assertAlmostEqual(cfg.aspirate_multiplier, 3.0)
        self.assertAlmostEqual(cfg.deposit_speed_uL_s, 0.4)
        self.assertAlmostEqual(cfg.aspirate_speed_uL_s, 0.2)

    def test_start_blocks_without_regions(self):
        page = self._make_page()
        page._on_start()
        self.assertIn("region", page._status.text().lower())


if __name__ == "__main__":
    unittest.main()
