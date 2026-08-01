"""test_v75x_quick_print_pick_and_place.py — Quick Print needle-prep + ink-pickup.

v7.5.x: Quick Print now runs the full bioprint sequence — (optional) condition
the needle (waste → oil → wash → buffer), pick up the print's ink from its
reagent well, then print — instead of assuming the needle is pre-loaded.

Covers:
  * the shared reagent helpers (gui/pages/workflows/_reagent_prep.py);
  * PickPlaceExecutor.aspirate_ink (safe-Z travel + aspirate, ZP-down guarded);
  * the Quick Print page: ink-override combo membership + pump default, the
    computed pickup volume, the setup-status gate, _on_print gating, and the
    preflight worker wiring (prep params + service positions + aspirate_ink
    called with the computed volume), driven against fakes — no hardware.
"""

import os
import sys
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, patch

# Isolate the workflow-settings store BEFORE the page is built so its settings
# dialog restores CODE defaults — not the operator's real saved Quick Print
# profile (prep/preflow/etc.), which would make these tests machine-dependent.
os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = tempfile.mkdtemp(
    prefix="mebp_qp_settings_")

from PySide6.QtWidgets import QApplication, QMessageBox

from SupportClasses.WellPlate import WellPlate
from SupportClasses.PhysicalModels import NeedleSpec, InkSpec
from SupportClasses.HardwareConfig import (
    HardwareConfig, PumpChannelConfig, SyringeSpec,
)
from SupportClasses.PickAndPlaceManager import (
    AbortException, PickPlaceExecutor,
)


def _needle() -> NeedleSpec:
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


def _make_hw(*, print_ink_loc="A2") -> HardwareConfig:
    """A config with a printable ink (Alginate ← A2) on P1 and the four service
    reagents assigned to wells (waste B1 / oil B2 / wash B3 / buffer B4)."""
    cfg = HardwareConfig()
    cfg.needle = _needle()
    alginate = InkSpec(name="Alginate", ink_type="hydrogel")
    syr = SyringeSpec(volume_uL=250, stroke_length_mm=30.0)
    cfg.pumps = {
        "P1": PumpChannelConfig(pump_id="P1", syringe=syr, inks=[alginate],
                                enabled=True),
        "P2": PumpChannelConfig(pump_id="P2"),
        "P3": PumpChannelConfig(pump_id="P3"),
    }
    cfg.add_ink(alginate)
    cfg.add_ink(InkSpec(name="PBS wash", ink_type="wash"))
    cfg.add_ink(InkSpec(name="Sep buffer", ink_type="buffer"))
    cfg.add_ink(InkSpec(name="Mineral oil", ink_type="oil"))
    cfg.add_ink(InkSpec(name="Waste bin", ink_type="waste"))
    if print_ink_loc:
        cfg.assign_wells_to_ink("Alginate", [print_ink_loc])
    cfg.assign_wells_to_ink("Waste bin", ["B1"])
    cfg.assign_wells_to_ink("Mineral oil", ["B2"])
    cfg.assign_wells_to_ink("PBS wash", ["B3"])
    cfg.assign_wells_to_ink("Sep buffer", ["B4"])
    return cfg


# Absolute stage µm for the wells the tests use.
_WELLS = {
    "A1": (10000.0, 12000.0),   # print target
    "A2": (20000.0, 12000.0),   # ink source
    "B1": (10000.0, 22000.0),   # waste
    "B2": (20000.0, 22000.0),   # oil
    "B3": (30000.0, 22000.0),   # wash
    "B4": (40000.0, 22000.0),   # buffer
}


# ════════════════════════════════════════════════════════════════════
#  Shared reagent helpers (no Qt)
# ════════════════════════════════════════════════════════════════════

class TestReagentHelpers(unittest.TestCase):
    def test_needle_volume(self):
        from gui.pages.workflows._reagent_prep import needle_volume_uL
        self.assertGreater(needle_volume_uL(_make_hw()), 0.0)
        self.assertEqual(needle_volume_uL(None), 0.0)

    def test_service_well_names_by_ink_type(self):
        from gui.pages.workflows._reagent_prep import service_well_names
        names = service_well_names(_make_hw())
        self.assertEqual(names["waste"], "B1")
        self.assertEqual(names["oil"], "B2")
        self.assertEqual(names["wash"], "B3")
        self.assertEqual(names["buffer"], "B4")

    def test_resolve_service_positions(self):
        from gui.pages.workflows._reagent_prep import resolve_service_positions
        positions, missing = resolve_service_positions(_make_hw(), _WELLS)
        self.assertEqual(missing, [])
        self.assertEqual(positions["oil"], _WELLS["B2"])

    def test_resolve_reports_missing(self):
        from gui.pages.workflows._reagent_prep import resolve_service_positions
        # Drop wash/buffer wells from the calibrated map → reported missing.
        partial = {k: v for k, v in _WELLS.items() if k in ("A1", "B1", "B2")}
        _positions, missing = resolve_service_positions(_make_hw(), partial)
        self.assertIn("wash", missing)
        self.assertIn("buffer", missing)


# ════════════════════════════════════════════════════════════════════
#  PickPlaceExecutor.aspirate_ink (no Qt)
# ════════════════════════════════════════════════════════════════════

class _RecCtrl:
    """Recording stand-in for StageController."""

    def __init__(self, zp_connected=True, pump_pos_uL=None):
        self.calls = []
        self.is_xy_connected = True
        self.is_zp_connected = zp_connected
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self._pump_pos_uL = pump_pos_uL  # for get_pump_position_uL

    def safe_travel_to(self, **kw):
        self.calls.append(("safe_travel_to", kw))
        return True

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None):
        self.calls.append(("move_pump_uL", pump, volume_uL, rate_uL_s))

    def move_xy_absolute_um(self, x_um, y_um, fast=False):
        self.calls.append(("move_xy_absolute_um", x_um, y_um))

    def move_z_user_relative(self, dz):
        self.calls.append(("move_z_user_relative", dz))

    def wait_for_xy_arrival(self, *a, **k):
        self.calls.append(("wait_for_xy_arrival", a, k))
        return True

    def get_pump_position_uL(self, pump):
        return self._pump_pos_uL

    def ensure_retracted_to(self, safe_z, *a, **k):
        self.calls.append(("ensure_retracted_to", safe_z))
        return True


class TestAspirateInk(unittest.TestCase):
    def test_travels_then_aspirates_negative(self):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.aspirate_ink((20000.0, 12000.0), 0.5, bore="P1", z_mm=-16.0,
                        rate_uL_s=1.0)
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertEqual(len(travels), 1)
        self.assertEqual(travels[0][1]["target_x_um"], 20000.0)
        self.assertAlmostEqual(travels[0][1]["target_z_mm"], -16.0)
        self.assertEqual(len(pumps), 1)
        self.assertEqual(pumps[0][1], "P1")
        self.assertLess(pumps[0][2], 0.0)        # draws IN (negative)
        self.assertAlmostEqual(abs(pumps[0][2]), 0.5)
        self.assertEqual(pumps[0][3], 1.0)

    def test_zero_volume_travels_but_no_pump(self):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.aspirate_ink((1.0, 2.0), 0.0, bore="P1", z_mm=-16.0)
        self.assertTrue(any(c[0] == "safe_travel_to" for c in ctrl.calls))
        self.assertFalse(any(c[0] == "move_pump_uL" for c in ctrl.calls))

    def test_refuses_when_zp_disconnected(self):
        ctrl = _RecCtrl(zp_connected=False)
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        with self.assertRaises(AbortException):
            ex.aspirate_ink((1.0, 2.0), 0.5, bore="P1", z_mm=-16.0)


class TestRunPrintCleanup(unittest.TestCase):
    def _executor(self, **ctrl_kw):
        ctrl = _RecCtrl(**ctrl_kw)
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.prep_bore = "P1"
        ex.needle_volume_uL = 0.5
        ex.service_z_mm = -16.0
        ex.wash_cycles = 0  # skip the jiggle so the pump trace is just waste+oil
        ex.waste_well_pos = (10000.0, 22000.0)
        ex.wash_well_pos = (30000.0, 22000.0)
        ex.oil_well_pos = (20000.0, 22000.0)
        ex.cleanup_waste_needles = 6.0
        ex.cleanup_oil_needles = 1.0
        return ctrl, ex

    def test_waste_wash_oil_order_and_fallback_signs(self):
        ctrl, ex = self._executor()
        ex.cleanup_oil_baseline_uL = None  # no baseline → fixed fallback draw
        ex.run_print_cleanup()
        travels = [c for c in ctrl.calls if c[0] == "safe_travel_to"]
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertEqual(len(travels), 3)              # waste, wash, oil
        self.assertEqual(len(pumps), 2)
        self.assertAlmostEqual(pumps[0][2], +6.0 * 0.5)  # expel 6 needles (+)
        self.assertAlmostEqual(pumps[1][2], -1.0 * 0.5)  # draw 1 needle (−)

    def test_oil_resets_to_baseline(self):
        # Plunger reads 2.0 µL at the oil step; baseline 0.5 → reset = −1.5.
        ctrl, ex = self._executor(pump_pos_uL=2.0)
        ex.cleanup_oil_baseline_uL = 0.5
        ex.run_print_cleanup()
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertAlmostEqual(pumps[0][2], +3.0)   # 6 × 0.5 expel
        self.assertAlmostEqual(pumps[-1][2], -1.5)  # baseline − current

    def test_implausible_baseline_falls_back(self):
        # current 0, baseline 100 → reset 100 µL > cap (20×0.5=10) → fallback.
        ctrl, ex = self._executor(pump_pos_uL=0.0)
        ex.cleanup_oil_baseline_uL = 100.0
        ex.run_print_cleanup()
        pumps = [c for c in ctrl.calls if c[0] == "move_pump_uL"]
        self.assertAlmostEqual(pumps[-1][2], -0.5)  # fallback draw 1 needle

    def test_refuses_when_zp_disconnected(self):
        ctrl, ex = self._executor(zp_connected=False)
        with self.assertRaises(AbortException):
            ex.run_print_cleanup()


# ════════════════════════════════════════════════════════════════════
#  Quick Print page (Qt)
# ════════════════════════════════════════════════════════════════════

class _FakeExecutor:
    """Records prep params + aspirate_ink; substitutes PickPlaceExecutor."""

    instances: list = []

    def __init__(self, controller, hw_config=None):
        self.controller = controller
        self.hw_config = hw_config
        self.safe_z_mm = None
        self.prep_bore = None
        self.needle_volume_uL = None
        self.service_z_mm = None
        self.wash_cycles = None
        self.waste_well_pos = None
        self.oil_well_pos = None
        self.wash_well_pos = None
        self.buffer_well_pos = None
        self.on_sub_step = None
        self.prep_ran = False
        self.aspirate_calls: list = []
        self.retracted = False
        # Post-print cleanup recorder.
        self.cleanup_ran = False
        self.cleanup_waste_needles = None
        self.cleanup_oil_needles = None
        self.cleanup_oil_baseline_uL = None
        self.cleanup_reset_to_initial = None
        self.cleanup_oil_margin_uL = None
        self._abort_flag = threading.Event()
        _FakeExecutor.instances.append(self)

    def run_prep(self):
        self.prep_ran = True

    def aspirate_ink(self, well_pos, volume_uL, *, bore, z_mm, rate_uL_s=None,
                     prime_uL=0.0, prime_rate_uL_s=None, orbit=False,
                     orbit_diameter_mm=1.0, orbit_speed_mm_s=2.0):
        self.aspirate_calls.append(
            {"well_pos": well_pos, "volume_uL": volume_uL, "bore": bore,
             "z_mm": z_mm})

    def run_print_cleanup(self):
        self.cleanup_ran = True

    def _retract_to_safe_z(self):
        self.retracted = True


class _FakePM:
    instances: list = []

    def __init__(self, controller):
        self.controller = controller
        self.job = None
        self.started = False
        self.state = None
        self.exec_logger = None
        self.on_progress = None
        self.on_state_changed = None
        _FakePM.instances.append(self)

    def load_job(self, job):
        self.job = job

    def start(self):
        self.started = True


class _CompletingFakePM(_FakePM):
    """FakePM that immediately signals COMPLETED on start() — drives the
    post-print cleanup trigger in _on_state."""

    def start(self):
        self.started = True
        from SupportClasses.PrintManager import PrintState
        self.state = PrintState.COMPLETED
        if self.on_state_changed:
            self.on_state_changed(PrintState.COMPLETED)


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
        ctrl.safe_travel_to.return_value = True
        # v7.5.x: bind the REAL single-source resolvers so the page reads the
        # same safety-primary XY/Z max it would in production (a bare MagicMock
        # method returns 1.0 via __float__). Z reads _pending_per_axis then
        # safety_limits.max_z_feedrate.
        ctrl._pending_per_axis_max_feedrate = None
        from SupportClasses.StageController import StageController as _SC
        ctrl.get_max_xy_speed_um_s.side_effect = (
            lambda: _SC.get_max_xy_speed_um_s(ctrl))
        ctrl.get_max_z_feedrate_mm_min.side_effect = (
            lambda: _SC.get_max_z_feedrate_mm_min(ctrl))
        return ctrl

    def _page(self, ctrl=None, *, hw=True, cal=True):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        ctrl = ctrl or self._ctrl()
        page = QuickPrintWorkflowPage(ctrl, settings=None)
        if hw:
            page.set_hardware_config(_make_hw())
        if cal:
            page.set_calibration_data(WellPlate.from_format(96), _WELLS, 5.0)
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._size_spin.setValue(1.0)
        page._selected_well = "A1"
        return page


class TestPrepDefault(_QtBase):
    def test_prep_defaults_on(self):
        """Headline behaviour: a freshly built page has prep ON, so the preamble
        (prep + ink pickup) is active by default."""
        page = self._page()
        self.assertTrue(page._prep_check.isChecked())
        self.assertTrue(page._preamble_active())

    def test_prep_off_and_no_ink_is_plain(self):
        page = self._page()
        page._prep_check.setChecked(False)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData(""))
        self.assertFalse(page._preamble_active())


class TestInkCombo(_QtBase):
    def test_lists_only_printable_inks_with_location(self):
        page = self._page()
        datas = [page._ink_combo.itemData(i)
                 for i in range(page._ink_combo.count())]
        self.assertIn("", datas)                 # the "(none)" entry
        self.assertIn("Alginate", datas)         # printable + located
        # Service reagents are excluded even though they have locations.
        for svc in ("PBS wash", "Sep buffer", "Mineral oil", "Waste bin"):
            self.assertNotIn(svc, datas)

    def test_defaults_to_pump_assigned_ink(self):
        page = self._page()
        self.assertEqual(page._selected_ink(), "Alginate")

    def test_no_location_ink_not_listed(self):
        page = self._page(ctrl=None, hw=False, cal=True)
        page.set_hardware_config(_make_hw(print_ink_loc=""))  # Alginate unlocated
        datas = [page._ink_combo.itemData(i)
                 for i in range(page._ink_combo.count())]
        self.assertNotIn("Alginate", datas)
        self.assertIsNone(page._selected_ink())

    def test_source_well_and_pos_resolution(self):
        page = self._page()
        self.assertEqual(page._ink_source_well(), "A2")
        self.assertEqual(page._ink_source_pos(), _WELLS["A2"])


class TestAutoFlow(_QtBase):
    """v7.5.x: flow @100% is AUTO-calculated = needle bore cross-section × XY
    max × extrusion modifier. The % scales speed + flow together; the modifier
    thickens the bead and feeds the pickup."""

    def _flow_page(self, area=0.01, xy_um_s=20000.0):
        ctrl = self._ctrl()
        ctrl.safety_limits = MagicMock()
        ctrl.safety_limits.max_xy_speed = xy_um_s
        page = self._page(ctrl=ctrl)
        page._needle_cross_section_mm2 = lambda: area   # deterministic bore
        page._extrusion_mod_spin.setValue(1.0)
        return page

    def test_flow_100_is_area_times_xymax_times_modifier(self):
        page = self._flow_page(area=0.01, xy_um_s=20000.0)   # xy_max = 20 mm/s
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            self.assertAlmostEqual(page._auto_flow_100_uL_s(), 0.2)  # 0.01×20×1
            page._extrusion_mod_spin.setValue(2.0)
            self.assertAlmostEqual(page._auto_flow_100_uL_s(), 0.4)  # thicker

    def test_flow_falls_back_without_needle(self):
        page = self._flow_page(area=0.0)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            self.assertAlmostEqual(page._auto_flow_100_uL_s(),
                                   page._FLOW_FALLBACK_UL_S)

    def test_xy_max_from_safety_when_no_measured(self):
        page = self._flow_page(xy_um_s=20000.0)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            self.assertAlmostEqual(page._xy_max_mm_s(), 20.0)

    def test_safety_limit_is_the_single_xy_source(self):
        # v7.5.x: safety_limits.max_xy_speed is the ONE editable XY max every
        # page (incl. the print) reads; the timing tool's "Measure top speed"
        # WRITES it. A persisted measured value no longer silently overrides it.
        page = self._flow_page(xy_um_s=20000.0)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = 60000.0
            self.assertAlmostEqual(page._xy_max_mm_s(), 20.0)

    def test_flow_follows_the_top_speed(self):
        # v7.6 CONTRACT CHANGE: the speed is an absolute mm/s ("top speed"),
        # not a % of the measured max — so the flow follows the resolved speed
        # instead of both scaling off one percentage.
        page = self._flow_page(area=0.01, xy_um_s=20000.0)
        page._top_speed_spin.setValue(10.0)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            s = page._build_settings()
        self.assertAlmostEqual(s.print_speed_mm_s, 10.0)    # honoured exactly
        self.assertAlmostEqual(s.pump_rate_uL_s, 0.1)       # area × speed
        self.assertAlmostEqual(s.get_pump_rate(page._pump()), 0.1)

    def test_bead_volume_per_mm_is_bore_area_invariant_to_speed(self):
        page = self._flow_page(area=0.01, xy_um_s=20000.0)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            page._top_speed_spin.setValue(5.0)
            sp1, fl1, _ = page._resolved_print_kinematics()
            page._top_speed_spin.setValue(16.0)
            sp2, fl2, _ = page._resolved_print_kinematics()
        self.assertGreater(sp2, sp1)
        self.assertGreater(fl2, fl1)
        # volume per mm == bore area (× modifier 1), independent of the speed:
        # changing the speed changes the TIME, never the bead.
        self.assertAlmostEqual(fl1 / sp1, 0.01, places=6)
        self.assertAlmostEqual(fl1 / sp1, fl2 / sp2, places=6)

    def test_modifier_thickens_bead(self):
        page = self._flow_page(area=0.01, xy_um_s=20000.0)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            _, fl1, _ = page._resolved_print_kinematics()
            page._extrusion_mod_spin.setValue(2.0)
            _, fl2, _ = page._resolved_print_kinematics()
        self.assertAlmostEqual(fl2, 2.0 * fl1)   # 2× flow at same speed = 2× bead


class TestPickupVolume(_QtBase):
    def _pk_page(self, area=0.01, xy_um_s=20000.0):
        ctrl = self._ctrl()
        ctrl.safety_limits = MagicMock()
        ctrl.safety_limits.max_xy_speed = xy_um_s
        page = self._page(ctrl=ctrl)
        page._needle_cross_section_mm2 = lambda: area
        page._preflow.setValue(0.0)          # isolate the prime
        page._extrusion_mod_spin.setValue(1.0)
        page._ink_padding_spin.setValue(0.0)
        return page

    def test_pickup_is_dispense_plus_reserve_plus_padding(self):
        page = self._pk_page()
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            dispensed = page._print_dispense_volume_uL()
            self.assertGreater(dispensed, 0.0)      # a circle has path length
            # A needle-bore reserve of ink is kept behind the deposit so the
            # print never reaches the buffer/oil (esp. after a prep).
            reserve = page._needle_dead_volume_uL()
            self.assertGreater(reserve, 0.0)
            self.assertAlmostEqual(
                page._compute_pickup_volume_uL(), dispensed + reserve)  # pad 0
            page._ink_padding_spin.setValue(2.0)
            self.assertAlmostEqual(
                page._compute_pickup_volume_uL(),
                dispensed + reserve + 2.0)          # +operator padding

    def test_modifier_scales_deposit_not_reserve(self):
        page = self._pk_page()
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            reserve = page._needle_dead_volume_uL()
            v1 = page._compute_pickup_volume_uL()
            page._extrusion_mod_spin.setValue(2.0)
            v2 = page._compute_pickup_volume_uL()
        self.assertGreater(v2, v1)
        # The DEPOSIT doubles with the modifier; the bore reserve is constant.
        self.assertAlmostEqual(v2 - reserve, 2.0 * (v1 - reserve))

    def test_dot_picks_up_reserve_plus_padding(self):
        page = self._pk_page()
        idx = page._object_combo.findData("simple:dot")
        page._object_combo.setCurrentIndex(idx)
        page._ink_padding_spin.setValue(1.5)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            reserve = page._needle_dead_volume_uL()
            # dot → no path length → dispensed 0, prime 0 → reserve + padding.
            self.assertAlmostEqual(
                page._compute_pickup_volume_uL(), reserve + 1.5)


class TestLineMoveSpeedPercent(_QtBase):
    """v7.5.x: the per-line Z/XY quick-move speeds are entered as a % of each
    stage's calibrated max and resolved to mm/s in _build_settings."""

    def _spd_page(self, xy_um_s=20000.0, z_mm_min=600.0):
        ctrl = self._ctrl()
        ctrl.safety_limits = MagicMock()
        ctrl.safety_limits.max_xy_speed = xy_um_s
        ctrl.safety_limits.max_z_feedrate = z_mm_min
        ctrl._pending_per_axis_max_feedrate = None
        return self._page(ctrl=ctrl)

    def test_z_max_mm_s_from_safety_feedrate(self):
        page = self._spd_page(z_mm_min=600.0)   # 600 mm/min ÷ 60 = 10 mm/s
        self.assertAlmostEqual(page._z_max_mm_s(), 10.0)

    def test_line_speeds_resolved_from_percent(self):
        page = self._spd_page(xy_um_s=20000.0, z_mm_min=600.0)
        page._line_z_speed_spin.setValue(50)    # 50% × 10 mm/s = 5
        page._line_xy_speed_spin.setValue(25)   # 25% × 20 mm/s = 5
        page._line_retract_spin.setValue(2.5)
        with patch("SupportClasses.PrintTimingCalibrationStore.get_store") as gs:
            gs.return_value.get_xy_max_speed_um_s.return_value = None
            s = page._build_settings()
        self.assertAlmostEqual(s.line_move_z_speed_mm_s, 5.0)
        self.assertAlmostEqual(s.line_move_xy_speed_mm_s, 5.0)
        self.assertAlmostEqual(s.intra_well_hop_z_mm, 2.5)


class TestSetupStatus(_QtBase):
    def test_warns_when_ink_has_no_location(self):
        page = self._page(ctrl=None, hw=False, cal=True)
        page.set_hardware_config(_make_hw())
        # Force the ink selection to an ink with no reagent location by clearing
        # its locations after populate.
        page._hw_config.clear_ink_location("Alginate")
        # Re-add it to the combo as a selectable (locationless) entry.
        page._ink_combo.addItem("Alginate", "Alginate")
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        page._refresh_setup_status()
        self.assertIn("⚠", page._setup_status.text())

    def test_prep_on_missing_wells_warns(self):
        page = self._page()
        page._prep_check.setChecked(True)
        # No needle/service issue here (full hw), so it should NOT warn; then
        # break it by dropping the service wells from the calibrated map.
        page.set_calibration_data(WellPlate.from_format(96),
                                  {"A1": _WELLS["A1"], "A2": _WELLS["A2"]}, 5.0)
        page._prep_check.setChecked(True)
        page._refresh_setup_status()
        self.assertIn("⚠", page._setup_status.text())

    def test_all_set_is_clean(self):
        page = self._page()
        page._prep_check.setChecked(True)
        page._refresh_setup_status()
        self.assertNotIn("⚠", page._setup_status.text())


class TestGating(_QtBase):
    def _drain(self, page):
        t = page._preposition_thread
        if t is not None:
            t.join(timeout=5.0)
        self._app.processEvents()

    def test_prep_on_missing_service_wells_blocks(self):
        page = self._page()
        # Ink "(none)" so only prep drives the gate.
        page._ink_combo.setCurrentIndex(page._ink_combo.findData(""))
        page._prep_check.setChecked(True)
        page.set_calibration_data(WellPlate.from_format(96),
                                  {"A1": _WELLS["A1"]}, 5.0)
        page._selected_well = "A1"
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            _FakeExecutor.instances.clear()
            page._on_print()
        self.assertIsNone(page._preposition_thread)
        self.assertEqual(len(_FakeExecutor.instances), 0)
        self.assertIn("reagent wells", page._status.text())

    def test_ink_without_calibrated_well_blocks(self):
        page = self._page()
        # Select Alginate but remove its source well from the calibrated map.
        page.set_calibration_data(WellPlate.from_format(96),
                                  {"A1": _WELLS["A1"]}, 5.0)  # no A2
        page._selected_well = "A1"
        idx = page._ink_combo.findData("Alginate")
        page._ink_combo.setCurrentIndex(idx)
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            _FakeExecutor.instances.clear()
            page._on_print()
        self.assertIsNone(page._preposition_thread)
        self.assertEqual(len(_FakeExecutor.instances), 0)
        self.assertIn("reagent well", page._status.text())


class TestPreflightWorker(_QtBase):
    def _drain(self, page):
        t = page._preposition_thread
        if t is not None:
            t.join(timeout=5.0)
        self._app.processEvents()

    def test_prep_and_pickup_then_print(self):
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(True)
        # v7.5.x: flow @100% is auto-calculated; expected_vol is read back from
        # the same computation the run uses.
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        expected_vol = page._compute_pickup_volume_uL()

        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self.assertIsNotNone(page._preposition_thread)
            self._drain(page)

        self.assertEqual(len(_FakeExecutor.instances), 1)
        ex = _FakeExecutor.instances[0]
        # Prep ran with service positions wired from the reagent map.
        self.assertTrue(ex.prep_ran)
        self.assertEqual(ex.prep_bore, "P1")
        self.assertEqual(ex.waste_well_pos, _WELLS["B1"])
        self.assertEqual(ex.oil_well_pos, _WELLS["B2"])
        self.assertEqual(ex.wash_well_pos, _WELLS["B3"])
        self.assertEqual(ex.buffer_well_pos, _WELLS["B4"])
        self.assertGreater(ex.needle_volume_uL, 0.0)
        # Ink picked up at the source well with the computed volume.
        self.assertEqual(len(ex.aspirate_calls), 1)
        call = ex.aspirate_calls[0]
        self.assertEqual(call["well_pos"], _WELLS["A2"])
        self.assertEqual(call["bore"], "P1")
        self.assertAlmostEqual(call["volume_uL"], expected_vol, places=6)
        # Always retract on exit; then the print starts.
        self.assertTrue(ex.retracted)
        self.assertEqual(len(_FakePM.instances), 1)
        self.assertTrue(_FakePM.instances[0].started)

    def test_pickup_only_no_prep(self):
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(False)  # ink pickup only
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakeExecutor.instances), 1)
        ex = _FakeExecutor.instances[0]
        self.assertFalse(ex.prep_ran)
        self.assertEqual(len(ex.aspirate_calls), 1)
        self.assertEqual(len(_FakePM.instances), 1)

    def test_plain_print_no_preamble_no_executor(self):
        """No prep + ink '(none)' → behaves like the legacy plain print: no
        executor, straight to preposition + PrintManager."""
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(False)
        page._postclean_check.setChecked(False)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData(""))  # none
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakeExecutor.instances), 0)
        self.assertEqual(len(_FakePM.instances), 1)

    def test_upfront_confirm_no_cancels(self):
        """Saying No to the up-front 'confirm setup' dialog cancels before any
        motion — the preflight worker never launches."""
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(True)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.No):
            page._on_print()
            self._drain(page)
        self.assertIsNone(page._preposition_thread)
        self.assertEqual(len(_FakeExecutor.instances), 0)
        self.assertEqual(len(_FakePM.instances), 0)
        self.assertEqual(page._controller.safe_travel_to.call_count, 0)

    def test_hands_free_no_midrun_confirm(self):
        """With prep+ink (preamble), exactly ONE dialog is shown — the up-front
        setup confirm; there is no per-print position confirmation."""
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(True)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes) as q:
            page._on_print()
            self._drain(page)
        self.assertEqual(q.call_count, 1)          # up-front confirm only
        self.assertEqual(len(_FakePM.instances), 1)
        self.assertTrue(_FakePM.instances[0].started)

    def test_prep_on_no_ink_runs_prep_only(self):
        """Prep ON + ink '(none)' (default-adjacent): prep runs, NO aspirate,
        and the print still starts."""
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(True)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData(""))  # none
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakeExecutor.instances), 1)
        ex = _FakeExecutor.instances[0]
        self.assertTrue(ex.prep_ran)
        self.assertEqual(len(ex.aspirate_calls), 0)   # no ink → no aspirate
        self.assertEqual(len(_FakePM.instances), 1)

    def test_abort_during_preamble_no_print(self):
        """If the preamble raises AbortException, the continuation must NOT
        start a print."""
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()

        class _AbortingExecutor(_FakeExecutor):
            def run_prep(self):
                raise AbortException("user abort")

        page = self._page()
        page._prep_check.setChecked(True)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _AbortingExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakePM.instances), 0)
        self.assertIn("abort", page._status.text().lower())
        # The executor still retracted on the way out.
        self.assertTrue(_FakeExecutor.instances[0].retracted)

    def test_abort_flag_set_during_blocking_move_no_print(self):
        """HIGH: an Abort during a blocking preamble move only SETS the abort
        flag (no AbortException). The print must STILL NOT start — the worker
        captures the set flag as 'aborted'."""
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()

        class _FlagSettingExecutor(_FakeExecutor):
            def run_prep(self):
                # Simulate the operator clicking Abort during a blocking pump
                # move: the flag is set but no _check_abort() runs to raise.
                self._abort_flag.set()
                self.prep_ran = True

        page = self._page()
        page._prep_check.setChecked(True)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FlagSettingExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _FakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain(page)
        self.assertEqual(len(_FakePM.instances), 0)        # print did NOT start
        self.assertIn("abort", page._status.text().lower())
        self.assertTrue(_FakeExecutor.instances[0].retracted)

    def test_sticky_abort_request_blocks_print(self):
        """A sticky abort request set on the GUI thread (the _on_abort path)
        gates _on_prepositioned even if the worker reported success."""
        page = self._page()
        page._pending_print = {
            "well": "A1", "center": (0.0, 0.0), "path_points": [(0.0, 0.0)],
            "path_segments": [[(0.0, 0.0)]],
            "settings": page._build_settings(), "pump": "P1", "obj_label": "x",
        }
        page._preflight_abort_requested = True
        with patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                   _FakePM):
            _FakePM.instances.clear()
            page._on_prepositioned(True)   # worker said "positioned ok"
        self.assertEqual(len(_FakePM.instances), 0)
        self.assertIn("abort", page._status.text().lower())


class TestPostPrintCleanup(_QtBase):
    def _drain_full(self, page):
        t = page._preposition_thread
        if t is not None:
            t.join(timeout=5.0)
        self._app.processEvents()      # _on_prepositioned → pm.start → _on_state
        ct = page._cleanup_thread
        if ct is not None:
            ct.join(timeout=5.0)
        self._app.processEvents()      # cleanup_done

    def test_cleanup_runs_on_completion(self):
        _FakeExecutor.instances.clear()
        _CompletingFakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(True)
        page._postclean_check.setChecked(True)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _CompletingFakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain_full(page)
        cleanups = [e for e in _FakeExecutor.instances if e.cleanup_ran]
        self.assertEqual(len(cleanups), 1)
        # v7.5.x: cleanup is now "reset to initial condition" (waste computed
        # live from the leftover + oil margin), not a fixed needle count.
        self.assertTrue(cleanups[0].cleanup_reset_to_initial)
        self.assertTrue(cleanups[0].retracted)

    def test_no_cleanup_when_disabled(self):
        _FakeExecutor.instances.clear()
        _CompletingFakePM.instances.clear()
        page = self._page()
        page._prep_check.setChecked(True)
        page._postclean_check.setChecked(False)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData("Alginate"))
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _CompletingFakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
            self._drain_full(page)
        self.assertEqual([e for e in _FakeExecutor.instances if e.cleanup_ran], [])

    def test_cleanup_gate_blocks_missing_oil_well(self):
        _FakeExecutor.instances.clear()
        page = self._page()
        page._prep_check.setChecked(False)
        page._ink_combo.setCurrentIndex(page._ink_combo.findData(""))  # no preamble
        page._postclean_check.setChecked(True)
        # Drop the oil well (B2) from the calibrated map.
        wells = {k: v for k, v in _WELLS.items() if k != "B2"}
        page.set_calibration_data(WellPlate.from_format(96), wells, 5.0)
        page._selected_well = "A1"
        with patch("gui.pages.workflows.quick_print_workflow.PickPlaceExecutor",
                   _FakeExecutor), \
                patch("gui.pages.workflows.quick_print_workflow.PrintManager",
                      _CompletingFakePM), \
                patch.object(QMessageBox, "question",
                             return_value=QMessageBox.StandardButton.Yes):
            page._on_print()
        self.assertIsNone(page._preposition_thread)
        self.assertIn("oil", page._status.text().lower())


if __name__ == "__main__":
    unittest.main()
