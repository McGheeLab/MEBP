"""
v7.5.x — Cell Targeting & Removal workflow.

Covers the trypsinize-in-place-then-extract operation (the new
``OperationType.CELL_TARGET_REMOVAL`` handler + ``CellRemovalConfig`` +
``run_post_clean`` in ``PickAndPlaceManager``) and the workflow page gating
(``gui/pages/workflows/cell_targeting_workflow.py``).

The executor tests drive the REAL ``PickPlaceExecutor`` against a recording
fake controller — no Qt / hardware. The page tests build the real Qt page
headless (offscreen).
"""

import math
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.PickAndPlaceManager import (
    AbortException, CellRemovalConfig, OperationQueue, OperationType,
    PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
)
from SupportClasses.PhysicalModels import NeedleSpec


def _needle(id_um=200.0, length_inches=0.5) -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=400.0, id_um=id_um, wall_um=100.0,
                      length_inches=length_inches)


class _RecCtrl:
    """Recording stand-in for StageController (mirrors the spheroid test)."""

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


def _cfg(**over) -> CellRemovalConfig:
    base = dict(
        reagent_bore="P1", release_depth_mm=0.10, release_volume_uL=0.012,
        extract_multiplier=2.0, dwell_time_s=0.0,  # 0 dwell = no real sleep
        push_speed_uL_s=0.5, pull_speed_uL_s=5.0,
        removal_z_offset_mm=0.10, place_z_offset_mm=0.50,
    )
    base.update(over)
    return CellRemovalConfig(**base)


def _op(cfg=None, well_a="", well_b=""):
    cfg = cfg or _cfg()
    src = PickPlaceTarget(target_id="P001", x_um=47583.0, y_um=48698.0,
                          well_name=well_a)
    dst = PickPlaceTarget(target_id="D001", x_um=67439.0, y_um=47722.0,
                          well_name=well_b)
    return PickPlaceOperation(
        op_id="OP1", op_type=OperationType.CELL_TARGET_REMOVAL,
        source_target=src, dest_target=dst, config=cfg)


# ── Config math ──────────────────────────────────────────────────────

class TestConfigVolumes(unittest.TestCase):
    def test_push_volume_from_needle_bore(self):
        n = _needle(id_um=200.0)  # 0.2 mm bore
        cfg = _cfg(release_depth_mm=0.10)
        area = math.pi * (0.1 ** 2)  # π(id/2)², mm²
        self.assertAlmostEqual(cfg.compute_release_volume_uL(n), area * 0.10)

    def test_push_volume_fallback_without_needle(self):
        cfg = _cfg(release_volume_uL=0.034)
        self.assertAlmostEqual(cfg.compute_release_volume_uL(None), 0.034)

    def test_extract_is_multiple_of_push(self):
        n = _needle()
        cfg = _cfg(extract_multiplier=2.0)
        self.assertAlmostEqual(
            cfg.compute_extract_volume_uL(n),
            2.0 * cfg.compute_release_volume_uL(n))


# ── Operation handler ────────────────────────────────────────────────

class TestCellRemovalExecution(unittest.TestCase):
    def _run(self, *, hw_needle=None, reagent_pos=(30000.0, 20000.0),
             reagent_z=-18.0, removal_z=-16.0, place_z=-14.0, cfg=None):
        ctrl = _RecCtrl()
        hw = SimpleNamespace(needle=hw_needle) if hw_needle else None
        ex = PickPlaceExecutor(ctrl, hw_config=hw)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = removal_z
        ex.place_z_mm = place_z
        ex.reagent_well_pos = reagent_pos
        ex.reagent_dip_z_mm = reagent_z
        q = OperationQueue()
        q.add(_op(cfg=cfg))
        ok = ex.execute_queue(q)
        return ctrl, ok

    def test_pump_sequence_load_push_pull_dispense(self):
        cfg = _cfg(release_volume_uL=0.012, extract_multiplier=2.0)
        ctrl, ok = self._run(cfg=cfg)
        self.assertTrue(ok)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        # load(-push), push(+push), pull(-pull), dispense(+pull)
        self.assertEqual(len(pumps), 4)
        self.assertAlmostEqual(pumps[0][2], -0.012)  # load (draw)
        self.assertAlmostEqual(pumps[1][2], +0.012)  # push (slow expel)
        self.assertAlmostEqual(pumps[2][2], -0.024)  # pull (fast draw)
        self.assertAlmostEqual(pumps[3][2], +0.024)  # dispense
        # All on the reagent bore.
        self.assertTrue(all(p[1] == "P1" for p in pumps))

    def test_pump_is_volume_balanced(self):
        ctrl, _ = self._run(cfg=_cfg(release_volume_uL=0.02))
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertAlmostEqual(sum(p[2] for p in pumps), 0.0)

    def test_speeds_slow_push_fast_pull(self):
        cfg = _cfg(push_speed_uL_s=0.5, pull_speed_uL_s=5.0)
        ctrl, _ = self._run(cfg=cfg)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertEqual(pumps[0][3], 0.5)   # load @ push (slow)
        self.assertEqual(pumps[1][3], 0.5)   # push @ slow
        self.assertEqual(pumps[2][3], 5.0)   # pull @ fast
        self.assertEqual(pumps[3][3], 0.5)   # dispense @ gentle (push speed)

    def test_push_volume_from_hw_needle_overrides_config_value(self):
        # With a needle in hw_config the push volume comes from the bore area,
        # not the config's stamped release_volume_uL.
        n = _needle(id_um=300.0)  # 0.3 mm bore
        cfg = _cfg(release_depth_mm=0.10, release_volume_uL=999.0)
        ctrl, _ = self._run(hw_needle=n, cfg=cfg)
        push = math.pi * (0.15 ** 2) * 0.10
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertAlmostEqual(pumps[1][2], +push)

    def test_three_full_safe_travels_with_distinct_z(self):
        ctrl, _ = self._run(reagent_z=-18.0, removal_z=-16.0, place_z=-14.0)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        # reagent load, removal, placement = 3 (empty well names → full travel).
        self.assertEqual(len(travels), 3)
        self.assertAlmostEqual(travels[0][1]["target_z_mm"], -18.0)  # reagent dip
        self.assertAlmostEqual(travels[1][1]["target_z_mm"], -16.0)  # removal
        self.assertAlmostEqual(travels[2][1]["target_z_mm"], -14.0)  # placement
        # No intra-well jiggle for empty-well-name targets.
        self.assertEqual(
            sum(1 for c in ctrl.calls if c[0] == "move_z_relative"), 0)

    def test_reagent_well_targeted_first(self):
        ctrl, _ = self._run(reagent_pos=(30000.0, 20000.0))
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(
            (round(travels[0][1]["target_x_um"]),
             round(travels[0][1]["target_y_um"])), (30000, 20000))

    def test_ends_at_safe_z(self):
        ctrl, ok = self._run()
        self.assertTrue(ok)
        self.assertEqual(ctrl.calls[-1][0], "ensure_retracted_to")
        self.assertAlmostEqual(ctrl.calls[-1][1], -35.0)

    def test_zp_disconnected_refuses(self):
        ctrl = _RecCtrl()
        ctrl.is_zp_connected = False
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
        ex.reagent_well_pos = (30000.0, 20000.0)
        ex.reagent_dip_z_mm = -18.0
        q = OperationQueue()
        q.add(_op())
        ex.execute_queue(q)  # internal abort, must not raise
        self.assertEqual(
            [c for c in ctrl.calls
             if c[0] in ("safe_travel_to", "move_xy_absolute_um")], [])

    def test_no_reagent_well_skips_load(self):
        # reagent_well_pos unset → no load travel/pump, but the removal +
        # placement legs still run (the GUI gates the reagent up front; this is
        # the graceful backstop).
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
        # reagent_well_pos left None
        q = OperationQueue()
        q.add(_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 2)  # removal + placement only


# ── Prep + post-clean bracketing ─────────────────────────────────────

class TestPrepAndPostClean(unittest.TestCase):
    def _exec(self, needle_uL=0.40, cycles=2):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
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
        # op:   reagent, removal, placement (3)
        # clean: waste, wash, buffer (3)
        self.assertEqual(len(travels), 10)
        self.assertEqual(xy[:4], [(60000, 30000), (50000, 30000),
                                  (40000, 30000), (30000, 30000)])
        self.assertEqual(xy[4], (25000, 25000))  # reagent load first in the op
        self.assertEqual(xy[7:], [(60000, 30000), (40000, 30000),
                                  (30000, 30000)])  # clean: waste, wash, buffer

    def test_post_clean_only(self):
        ctrl, ex = self._exec()
        ex.do_prep = False
        ex.do_post_clean = True
        q = OperationQueue()
        q.add(_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        # op (reagent, removal, placement = 3) + clean (waste, wash, buffer = 3)
        self.assertEqual(len(travels), 6)

    def test_post_clean_pump_volumes(self):
        ctrl, ex = self._exec(needle_uL=0.40)
        ex.do_prep = False
        ex.do_post_clean = True
        q = OperationQueue()
        q.add(_op(cfg=_cfg(release_volume_uL=0.0)))  # isolate clean pumps
        ex.execute_queue(q)
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        # clean: expel +1 needle to waste, then draw -4 needles buffer.
        self.assertAlmostEqual(pumps[-2][2], +0.40)
        self.assertAlmostEqual(pumps[-1][2], -1.60)

    def test_post_clean_skipped_when_disabled(self):
        ctrl, ex = self._exec()
        ex.do_prep = False
        ex.do_post_clean = False
        q = OperationQueue()
        q.add(_op())
        ex.execute_queue(q)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        self.assertEqual(len(travels), 3)  # op only (reagent + removal + place)

    def test_missing_service_well_aborts_in_clean(self):
        ctrl, ex = self._exec()
        ex.do_prep = False
        ex.do_post_clean = True
        ex.wash_well_pos = None  # clean can't resolve wash
        q = OperationQueue()
        q.add(_op())
        with self.assertRaises(RuntimeError):
            ex.execute_queue(q)
        # Still retracts on the way out.
        self.assertTrue(
            any(c[0] == "ensure_retracted_to" for c in ctrl.calls))


# ── GUI page (offscreen) ─────────────────────────────────────────────

class TestPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_page(self):
        from gui.pages.workflows.cell_targeting_workflow import (
            CellTargetingWorkflowPage,
        )
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.safety_limits = None
        return CellTargetingWorkflowPage(ctrl, settings=None, camera_manager=None)

    def test_constructs(self):
        page = self._make_page()
        self.assertEqual(page.get_page_title(), "Cell Targeting & Removal")

    def test_push_pull_volume_from_needle(self):
        page = self._make_page()
        page._hw_config = SimpleNamespace(needle=_needle(id_um=200.0))
        page._push_depth.setValue(0.10)
        page._pull_mult.setValue(2.0)
        push, pull = page._push_pull_uL()
        self.assertAlmostEqual(push, math.pi * (0.1 ** 2) * 0.10)
        self.assertAlmostEqual(pull, 2.0 * push)

    def test_current_config_reflects_spins(self):
        page = self._make_page()
        page._hw_config = SimpleNamespace(needle=_needle())
        page._removal_z.setValue(0.20)
        page._place_z.setValue(0.60)
        page._dwell.setValue(45.0)
        page._pull_mult.setValue(3.0)
        cfg = page._current_config()
        self.assertAlmostEqual(cfg.removal_z_offset_mm, 0.20)
        self.assertAlmostEqual(cfg.place_z_offset_mm, 0.60)
        self.assertAlmostEqual(cfg.dwell_time_s, 45.0)
        self.assertAlmostEqual(cfg.extract_multiplier, 3.0)

    def test_start_blocks_when_unbalanced(self):
        page = self._make_page()
        # No picked targets → not balanced → Start refuses with a message.
        page._on_start()
        self.assertIn("paired", page._status.text().lower())


if __name__ == "__main__":
    unittest.main()
