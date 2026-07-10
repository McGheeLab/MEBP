"""v7.5.x tests — needle-derived max flow display + flow-bounded print speed
(changes_needed item 6, the parts added on top of the existing WIP backend).

The per-pump max-flow CEILING + hard clamp + the StageController speed resolvers
already exist; these tests cover the two additions: (a) the Hardware Setup pump
panel READOUT of the computed max flow, and (b) Quick Print bounding its max
print speed by the flow ceiling so the auto flow never exceeds it (avoids the
over-pressure that ingests air) — guarded against MagicMock stubs.
"""
import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from tests.test_v75x_quick_print_pick_and_place import _make_hw, _WELLS
from SupportClasses.WellPlate import WellPlate
from SupportClasses.StageController import StageController as _SC


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class TestQuickPrintFlowBound(_QtBase):
    def _page(self, *, max_flow=None, xy_um_s=20000.0):
        from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl._pending_per_axis_max_feedrate = None
        ctrl.safety_limits = MagicMock()
        ctrl.safety_limits.max_xy_speed = xy_um_s
        ctrl.get_max_xy_speed_um_s.side_effect = lambda: _SC.get_max_xy_speed_um_s(ctrl)
        if max_flow is None:
            # real safety_limits returns a MagicMock for get_max_flow_rate — the
            # guard must treat that as "no limit" (not the 1.0 a __float__ gives).
            pass
        else:
            ctrl.safety_limits.get_max_flow_rate = lambda pump: max_flow
        page = QuickPrintWorkflowPage(ctrl, settings=None)
        page.set_hardware_config(_make_hw())
        page.set_calibration_data(WellPlate.from_format(96), _WELLS, 5.0)
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._size_spin.setValue(1.0)
        page._selected_well = "A1"
        return page

    def test_magicmock_guard_returns_zero(self):
        page = self._page(max_flow=None)
        self.assertEqual(page._max_pump_flow_uL_s(), 0.0)
        # No limit → flow-limited XY max == plain XY max (unchanged behaviour).
        self.assertAlmostEqual(page._flow_limited_xy_max_mm_s(),
                               page._xy_max_mm_s())

    def test_speed_bounded_by_flow_ceiling(self):
        page = self._page(max_flow=None)
        xy0 = page._xy_max_mm_s()
        area = page._needle_cross_section_mm2()
        self.assertGreater(area, 0)
        unbounded = area * xy0 * page._extrusion_modifier()
        # Impose a ceiling at half the unbounded flow@100%.
        ceiling = unbounded / 2.0
        page._controller.safety_limits.get_max_flow_rate = lambda pump: ceiling
        self.assertAlmostEqual(page._max_pump_flow_uL_s(), ceiling)
        # XY max halves; flow@100% lands exactly on the ceiling (not above).
        self.assertAlmostEqual(page._flow_limited_xy_max_mm_s(), xy0 / 2.0, places=4)
        self.assertLessEqual(page._auto_flow_100_uL_s(), ceiling + 1e-9)

    def test_high_ceiling_does_not_raise_speed(self):
        page = self._page(max_flow=None)
        xy0 = page._xy_max_mm_s()
        page._controller.safety_limits.get_max_flow_rate = lambda pump: 1e6
        # A huge ceiling never binds → XY max unchanged.
        self.assertAlmostEqual(page._flow_limited_xy_max_mm_s(), xy0)


class TestHardwareSetupMaxFlowDisplay(_QtBase):
    def test_label_shows_flow_for_needle(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        pg = HardwareSetupPage()
        pg._config = _make_hw()
        pg._refresh_max_flow_display()
        txt = pg._pump_maxflow_lbl.text()
        self.assertIn("µL/s", txt)
        self.assertNotIn("configure the needle", txt)

    def test_label_prompts_when_no_needle_bore(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        from SupportClasses.HardwareConfig import HardwareConfig
        pg = HardwareSetupPage()
        pg._config = HardwareConfig()       # no needle bore configured
        pg._refresh_max_flow_display()
        self.assertIn("—", pg._pump_maxflow_lbl.text())


if __name__ == "__main__":
    unittest.main()
