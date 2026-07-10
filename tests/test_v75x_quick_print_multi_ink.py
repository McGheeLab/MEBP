"""test_v75x_quick_print_multi_ink.py — Quick Print: abstract-ink mapping +
sequential ink swaps.

A sketch print now stores ABSTRACT inks (not pumps). When it uses ≥2 inks,
Quick Print maps each abstract ink → a configured ink and runs sequential ink
swaps: prep once → per group (pick up mapped ink → print) → waste/wash/buffer
between inks → clean once. Single-ink / non-sketch prints take the legacy path.

Driven against fakes — no hardware.
"""

import os
import sys
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, patch

os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = tempfile.mkdtemp(
    prefix="mebp_qpmi_settings_")

from PySide6.QtWidgets import QApplication, QMessageBox

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PhysicalModels import NeedleSpec, InkSpec
from SupportClasses.HardwareConfig import (
    HardwareConfig, PumpChannelConfig, SyringeSpec,
)
from SupportClasses.SketchTrajectory import Sketch, SketchShape, SketchInk

_QP = "gui.pages.workflows.quick_print_workflow"


def _needle():
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


def _make_hw():
    """Two printable inks on two enabled pumps (Alginate/P1 #89b4fa,
    Gelatin/P2 #a6e3a1) + the four service reagents."""
    cfg = HardwareConfig()
    cfg.needle = _needle()
    alg = InkSpec(name="Alginate", ink_type="hydrogel", color="#89b4fa")
    gel = InkSpec(name="Gelatin", ink_type="hydrogel", color="#a6e3a1")
    syr = SyringeSpec(volume_uL=250, stroke_length_mm=30.0)
    cfg.pumps = {
        "P1": PumpChannelConfig(pump_id="P1", syringe=syr, inks=[alg],
                                enabled=True),
        "P2": PumpChannelConfig(pump_id="P2", syringe=syr, inks=[gel],
                                enabled=True),
        "P3": PumpChannelConfig(pump_id="P3"),
    }
    for ink in (alg, gel):
        cfg.add_ink(ink)
    cfg.add_ink(InkSpec(name="PBS wash", ink_type="wash"))
    cfg.add_ink(InkSpec(name="Sep buffer", ink_type="buffer"))
    cfg.add_ink(InkSpec(name="Mineral oil", ink_type="oil"))
    cfg.add_ink(InkSpec(name="Waste bin", ink_type="waste"))
    cfg.assign_wells_to_ink("Alginate", ["A2"])
    cfg.assign_wells_to_ink("Gelatin", ["A3"])
    cfg.assign_wells_to_ink("Waste bin", ["B1"])
    cfg.assign_wells_to_ink("Mineral oil", ["B2"])
    cfg.assign_wells_to_ink("PBS wash", ["B3"])
    cfg.assign_wells_to_ink("Sep buffer", ["B4"])
    return cfg


_WELLS = {
    "A1": (10000.0, 12000.0),   # print target
    "A2": (20000.0, 12000.0),   # Alginate source
    "A3": (30000.0, 12000.0),   # Gelatin source
    "B1": (10000.0, 22000.0),   # waste
    "B2": (20000.0, 22000.0),   # oil
    "B3": (30000.0, 22000.0),   # wash
    "B4": (40000.0, 22000.0),   # buffer
}


def _two_ink_sketch():
    """Shapes A(ink1) B(ink2) C(ink1) — three ink-contiguous groups."""
    sk = Sketch(line_spacing_mm=0.4)
    sk.inks = [SketchInk(1, "Struct", "#89b4fa"),
               SketchInk(2, "Support", "#a6e3a1")]
    sk._next_ink_id = 3
    sk.shapes = [
        SketchShape(kind="line", points=[(-3, 0), (-1, 0)], ink_id=1),
        SketchShape(kind="line", points=[(0, 0), (2, 0)], ink_id=2),
        SketchShape(kind="line", points=[(3, 0), (5, 0)], ink_id=1),
    ]
    return sk


# ════════════════════════════════════════════════════════════════════
#  Fakes (module-patched) + shared ordered call log
# ════════════════════════════════════════════════════════════════════

CALLS: list = []


class _FakeExecutor:
    instances: list = []

    def __init__(self, controller, hw_config=None):
        self.controller = controller
        self._abort_flag = threading.Event()
        self.on_sub_step = None
        self.prep_bore = None
        self.retracted = False
        _FakeExecutor.instances.append(self)

    def __setattr__(self, k, v):      # accept every param the page sets
        object.__setattr__(self, k, v)

    def run_prep(self):
        CALLS.append(("prep",))

    def run_post_clean(self):
        CALLS.append(("swap", self.prep_bore))

    def aspirate_ink(self, well_pos, volume_uL, *, bore, z_mm, rate_uL_s=None):
        CALLS.append(("aspirate", bore, well_pos, round(float(volume_uL), 4)))

    def run_print_cleanup(self):
        CALLS.append(("cleanup",))

    def _retract_to_safe_z(self):
        self.retracted = True
        CALLS.append(("retract",))


class _FakePM:
    instances: list = []

    def __init__(self, controller):
        self.controller = controller
        self.on_progress = None
        self.on_state_changed = None
        self.state = None
        _FakePM.instances.append(self)

    def load_job(self, job):
        self.job = job

    def start(self):
        from SupportClasses.PrintManager import PrintState
        self.state = PrintState.COMPLETED
        if self.on_state_changed:
            self.on_state_changed(PrintState.COMPLETED)


_JOBS: list = []


def _fake_build_job(**kw):
    CALLS.append(("job", kw.get("pump")))
    _JOBS.append(kw)
    return dict(kw)


# ════════════════════════════════════════════════════════════════════
#  Tests
# ════════════════════════════════════════════════════════════════════

class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.print_floor_violation.return_value = False
        ctrl.print_height_to_zref.return_value = -25.0
        ctrl.print_z_dir.return_value = -1.0
        ctrl.plate_axis_sign.return_value = (1, 1)
        ctrl.safe_travel_to.return_value = True
        ctrl._pending_per_axis_max_feedrate = None
        from SupportClasses.StageController import StageController as _SC
        ctrl.get_max_xy_speed_um_s.side_effect = (
            lambda: _SC.get_max_xy_speed_um_s(ctrl))
        ctrl.get_max_z_feedrate_mm_min.side_effect = (
            lambda: _SC.get_max_z_feedrate_mm_min(ctrl))
        return ctrl

    def _page(self, sketch=None):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        page = QuickPrintWorkflowPage(self._ctrl(), settings=None)
        page.set_hardware_config(_make_hw())
        page.set_calibration_data(WellPlate.from_format(96), _WELLS, -35.0)
        page._selected_well = "A1"
        if sketch is not None:
            page._loaded_sketch = sketch
            page._rebuild_ink_mapping_ui()
        return page


class TestDetect(_QtBase):
    def test_two_ink_is_multi(self):
        self.assertTrue(self._page(_two_ink_sketch())._is_multi_ink())

    def test_single_ink_not_multi(self):
        sk = Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (5, 0)],
                                        ink_id=1)])
        self.assertFalse(self._page(sk)._is_multi_ink())

    def test_non_sketch_not_multi(self):
        self.assertFalse(self._page()._is_multi_ink())


class TestInkGroups(_QtBase):
    def test_partition_order(self):
        groups = self._page(_two_ink_sketch())._ink_groups()
        self.assertEqual([iid for iid, _ in groups], [1, 2, 1])

    def test_travel_rides_with_following_group(self):
        sk = _two_ink_sketch()
        sk.shapes.insert(1, SketchShape(kind="travel", cx=9, cy=9))
        groups = self._page(sk)._ink_groups()
        # travel is not a printing shape, but the group set is unchanged.
        self.assertEqual([iid for iid, _ in groups], [1, 2, 1])

    def test_no_print_only_group_dropped(self):
        sk = _two_ink_sketch()
        # Make the middle (ink 2) shape move-only → its group has no print.
        sk.shapes[1].no_print = True
        groups = self._page(sk)._ink_groups()
        self.assertNotIn(2, [iid for iid, _ in groups])


class TestGroupRecompile(_QtBase):
    def test_group_segments_nonempty(self):
        page = self._page(_two_ink_sketch())
        for iid, sub in page._ink_groups():
            segs = page._group_segments(sub, "P1")
            self.assertTrue(segs and all(len(s) >= 2 for s in segs))


class TestMapping(_QtBase):
    def test_default_maps_by_colour(self):
        page = self._page(_two_ink_sketch())
        # ink 1 (#89b4fa) → Alginate, ink 2 (#a6e3a1) → Gelatin (colour match).
        self.assertEqual(page._ink_map[1], "Alginate")
        self.assertEqual(page._ink_map[2], "Gelatin")

    def test_validate_clean_when_mapped(self):
        self.assertEqual(self._page(_two_ink_sketch())._validate_ink_map(), [])

    def test_validate_flags_unmapped(self):
        page = self._page(_two_ink_sketch())
        page._ink_map.pop(2, None)
        self.assertTrue(page._validate_ink_map())

    def test_pump_resolution(self):
        hw = _make_hw()
        self.assertEqual(hw.get_pump_for_ink("Alginate"), "P1")
        self.assertEqual(hw.get_pump_for_ink("Gelatin"), "P2")


class TestWorkerSequence(_QtBase):
    def _run(self, page):
        CALLS.clear()
        _JOBS.clear()
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        with patch(f"{_QP}.PickPlaceExecutor", _FakeExecutor), \
                patch(f"{_QP}.PrintManager", _FakePM), \
                patch(f"{_QP}.build_well_plate_job", _fake_build_job), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._start_multi_ink_run()
            t = page._multi_thread
            if t is not None:      # join INSIDE the patch so the worker uses fakes
                t.join(timeout=10.0)

    def test_sequence_prep_swap_aspirate_print_cleanup(self):
        page = self._page(_two_ink_sketch())     # prep + cleanup default ON
        self._run(page)
        kinds = [c[0] for c in CALLS]
        # prep once, up front; cleanup once, at end.
        self.assertEqual(kinds.count("prep"), 1)
        self.assertEqual(kinds.count("cleanup"), 1)
        self.assertEqual(kinds[0], "prep")
        self.assertEqual(kinds[-1], "retract")     # always ends retracted
        # one swap between each of the 3 groups (2 swaps).
        self.assertEqual(kinds.count("swap"), 2)
        # 3 aspirates + 3 prints, aspirate before its print each group.
        self.assertEqual(kinds.count("aspirate"), 3)
        self.assertEqual(kinds.count("job"), 3)
        # First group prints with its mapped pump P1, second P2, third P1.
        job_pumps = [c[1] for c in CALLS if c[0] == "job"]
        self.assertEqual(job_pumps, ["P1", "P2", "P1"])
        asp_pumps = [c[1] for c in CALLS if c[0] == "aspirate"]
        self.assertEqual(asp_pumps, ["P1", "P2", "P1"])

    def test_return_home_false_and_calibrated_well(self):
        page = self._page(_two_ink_sketch())
        self._run(page)
        self.assertTrue(_JOBS)
        for kw in _JOBS:
            self.assertFalse(kw["return_home"])
        # The aspirate for ink 1 (Alginate ← A2) gets the calibrated µm verbatim.
        alg = next(c for c in CALLS if c[0] == "aspirate" and c[1] == "P1")
        self.assertEqual(alg[2], _WELLS["A2"])       # byte-for-byte, no re-sign

    def test_abort_before_next_group(self):
        page = self._page(_two_ink_sketch())
        CALLS.clear(); _JOBS.clear()
        _FakeExecutor.instances.clear(); _FakePM.instances.clear()

        # A FakePM whose start() requests abort, then completes → the worker
        # should not start a further group.
        class _AbortingPM(_FakePM):
            def start(self):
                page._multi_abort_requested = True
                super().start()

        with patch(f"{_QP}.PickPlaceExecutor", _FakeExecutor), \
                patch(f"{_QP}.PrintManager", _AbortingPM), \
                patch(f"{_QP}.build_well_plate_job", _fake_build_job), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._start_multi_ink_run()
            t = page._multi_thread
            if t is not None:
                t.join(timeout=10.0)
        # Only the first group printed; no swap/second aspirate/cleanup.
        self.assertEqual([c[0] for c in CALLS].count("job"), 1)
        self.assertNotIn("cleanup", [c[0] for c in CALLS])
        self.assertIn("retract", [c[0] for c in CALLS])   # ended at safe Z


class TestSingleInkUnchanged(_QtBase):
    def test_single_ink_takes_legacy_path(self):
        sk = Sketch(shapes=[SketchShape(kind="line", points=[(0, 0), (5, 0)],
                                        ink_id=1)])
        page = self._page(sk)
        CALLS.clear()
        _FakeExecutor.instances.clear()
        with patch(f"{_QP}.PickPlaceExecutor", _FakeExecutor), \
                patch(f"{_QP}.PrintManager", _FakePM), \
                patch(f"{_QP}.build_well_plate_job", _fake_build_job), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes), \
                patch.object(QMessageBox, "warning",
                             return_value=QMessageBox.StandardButton.Ok):
            page._start_multi_ink_run()          # guard: single ink → refuses
        self.assertIsNone(page._multi_thread)
        self.assertNotIn("swap", [c[0] for c in CALLS])


if __name__ == "__main__":
    unittest.main()
