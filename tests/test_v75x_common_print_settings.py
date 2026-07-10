"""v7.5.x — Common Print Settings model + dialog inherit/override.

The Common Print Settings page collects the settings common to all workflows:
  * GLOBAL params (pump settle/dwell, pressure relief, prime) — canonically on
    HardwareConfig; the model PROXIES them.
  * PROMOTED prep defaults (service Z, prep rate, oil/buffer needles, wash …) —
    shared defaults owned by the model; each workflow inherits but can override.

These exercise the pure model + the dialog's inherit/override field.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest
from types import SimpleNamespace

from SupportClasses.CommonPrintSettings import (
    CommonPrintSettings, GLOBAL_KEYS, PROMOTED_DEFAULTS, GLOBAL_DEFAULTS,
)


# ════════════════════════════════════════════════════════════════════
#  Model
# ════════════════════════════════════════════════════════════════════

class TestCommonPrintSettingsModel(unittest.TestCase):

    def test_promoted_defaults_present(self):
        c = CommonPrintSettings()
        for key, default in PROMOTED_DEFAULTS.items():
            self.assertEqual(c.get(key), default)

    def test_globals_default_without_hw_config(self):
        c = CommonPrintSettings()
        for key in GLOBAL_KEYS:
            self.assertEqual(c.get(key), GLOBAL_DEFAULTS[key])

    def test_globals_proxy_hardware_config(self):
        hw = SimpleNamespace(pump_settle_time_s=0.4,
                             pump_prime_time_s=0.3)
        c = CommonPrintSettings()
        c.set_hardware_config(hw)
        self.assertAlmostEqual(c.get("pump_settle_time_s"), 0.4)
        self.assertAlmostEqual(c.get("pump_prime_time_s"), 0.3)
        # Setting a global writes through to the HardwareConfig attr.
        c.set("pump_prime_time_s", 0.5)
        self.assertAlmostEqual(hw.pump_prime_time_s, 0.5)
        self.assertAlmostEqual(c.get("pump_prime_time_s"), 0.5)

    def test_set_promoted_updates_dict(self):
        c = CommonPrintSettings()
        c.set("service_z", 0.8)
        self.assertAlmostEqual(c.get("service_z"), 0.8)
        self.assertAlmostEqual(c.promoted_dict()["service_z"], 0.8)

    def test_wash_cycles_coerced_to_int(self):
        c = CommonPrintSettings()
        c.set("wash_cycles", 4.0)
        self.assertEqual(c.get("wash_cycles"), 4)
        self.assertIsInstance(c.get("wash_cycles"), int)

    def test_negative_clamped(self):
        c = CommonPrintSettings()
        c.set("service_z", -3.0)
        self.assertEqual(c.get("service_z"), 0.0)

    def test_listener_fires_with_key(self):
        c = CommonPrintSettings()
        seen = []
        c.add_listener(lambda model, key: seen.append(key))
        c.set("service_z", 0.7)
        c.set("pump_settle_time_s", 0.2)
        self.assertEqual(seen, ["service_z", "pump_settle_time_s"])

    def test_set_no_notify(self):
        c = CommonPrintSettings()
        seen = []
        c.add_listener(lambda model, key: seen.append(key))
        c.set("service_z", 0.7, notify=False)
        self.assertEqual(seen, [])

    def test_load_promoted_round_trip(self):
        c = CommonPrintSettings()
        c.set("service_z", 0.9, notify=False)
        c.set("wash_cycles", 7, notify=False)
        d = c.promoted_dict()
        c2 = CommonPrintSettings()
        c2.load_promoted(d)
        self.assertAlmostEqual(c2.get("service_z"), 0.9)
        self.assertEqual(c2.get("wash_cycles"), 7)

    def test_load_promoted_ignores_globals_and_junk(self):
        c = CommonPrintSettings()
        c.load_promoted({"pump_settle_time_s": 9.0, "bogus": 1, "service_z": 0.6})
        # Globals are NOT loaded from the promoted dict (they live on HW config).
        self.assertEqual(c.get("pump_settle_time_s"), GLOBAL_DEFAULTS["pump_settle_time_s"])
        self.assertAlmostEqual(c.get("service_z"), 0.6)

    def test_unknown_key_set_is_noop(self):
        c = CommonPrintSettings()
        seen = []
        c.add_listener(lambda m, k: seen.append(k))
        c.set("does_not_exist", 5)
        self.assertEqual(seen, [])

    def test_bad_listener_does_not_break_set(self):
        c = CommonPrintSettings()
        c.add_listener(lambda m, k: (_ for _ in ()).throw(RuntimeError("boom")))
        c.set("service_z", 0.5)            # must not raise
        self.assertAlmostEqual(c.get("service_z"), 0.5)


# ════════════════════════════════════════════════════════════════════
#  Dialog inherit / override
# ════════════════════════════════════════════════════════════════════

class TestDialogCommonField(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _dlg(self):
        from gui.dialogs.workflow_settings_dialog import WorkflowSettingsDialog
        from PySide6.QtWidgets import QDoubleSpinBox
        dlg = WorkflowSettingsDialog("test_wf", "Test")
        sec = dlg.add_section("Prep")
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 30.0)
        sec.add_common("service_z", "Service Z", spin, 0.50)
        dlg.finalize()
        return dlg, spin

    def test_inherits_common_when_not_overridden(self):
        dlg, spin = self._dlg()
        common = CommonPrintSettings()
        common.set("service_z", 0.8, notify=False)
        dlg.set_common(common)
        # Not overridden → widget mirrors common and is disabled.
        self.assertFalse(spin.isEnabled())
        self.assertAlmostEqual(spin.value(), 0.8)

    def test_override_enables_local_value(self):
        dlg, spin = self._dlg()
        common = CommonPrintSettings()
        common.set("service_z", 0.8, notify=False)
        dlg.set_common(common)
        # Turn override ON via the registered checkbox.
        ovr = dlg._fields["service_z__ovr"][0]
        ovr.setChecked(True)
        self.assertTrue(spin.isEnabled())
        spin.setValue(1.25)
        # Effective (widget) value is the local override now.
        self.assertAlmostEqual(spin.value(), 1.25)

    def test_common_change_repropagates_to_inheriting_widget(self):
        dlg, spin = self._dlg()
        common = CommonPrintSettings()
        common.set("service_z", 0.8, notify=False)
        dlg.set_common(common)
        self.assertAlmostEqual(spin.value(), 0.8)
        common.set("service_z", 0.2, notify=False)
        dlg.set_common(common)            # re-fan-out
        self.assertAlmostEqual(spin.value(), 0.2)

    def test_migration_preserves_customized_value_as_override(self):
        # Old saved settings (pre-feature) have "service_z" but no "__ovr".
        dlg, spin = self._dlg()
        common = CommonPrintSettings()
        common.set("service_z", 0.50, notify=False)   # default
        dlg.set_common(common)
        dlg.apply({"service_z": 0.9})    # legacy save with a customized value
        ovr = dlg._fields["service_z__ovr"][0]
        self.assertTrue(ovr.isChecked())              # migrated to override
        self.assertAlmostEqual(spin.value(), 0.9)

    def test_migration_inherits_when_value_equals_default(self):
        dlg, spin = self._dlg()
        common = CommonPrintSettings()
        common.set("service_z", 0.50, notify=False)
        dlg.set_common(common)
        dlg.apply({"service_z": 0.50})   # legacy save at the common default
        ovr = dlg._fields["service_z__ovr"][0]
        self.assertFalse(ovr.isChecked())             # inherits

    def test_global_field_writes_through_to_common(self):
        from gui.dialogs.workflow_settings_dialog import WorkflowSettingsDialog
        from PySide6.QtWidgets import QDoubleSpinBox
        dlg = WorkflowSettingsDialog("test_wf2", "Test2")
        sec = dlg.add_section("Pump (global)")
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 10.0)
        sec.add_common("pump_prime_time_s", "Prime", spin, 0.0,
                       overridable=False)
        dlg.finalize()
        hw = SimpleNamespace(pump_settle_time_s=0.0, pump_prime_time_s=0.25)
        common = CommonPrintSettings()
        common.set_hardware_config(hw)
        dlg.set_common(common)
        # Global field is editable (no override checkbox) and writes through.
        self.assertTrue(spin.isEnabled())
        self.assertNotIn("pump_prime_time_s__ovr", dlg._fields)
        spin.setValue(1.7)
        self.assertAlmostEqual(hw.pump_prime_time_s, 1.7)


# ════════════════════════════════════════════════════════════════════
#  Common Print Settings page
# ════════════════════════════════════════════════════════════════════

class TestCommonPrintSettingsPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _page(self):
        from gui.pages.workflows.common_print_settings_workflow import (
            CommonPrintSettingsWorkflowPage)
        return CommonPrintSettingsWorkflowPage(controller=None, settings=None)

    def test_page_shows_common_values(self):
        page = self._page()
        hw = SimpleNamespace(pump_settle_time_s=0.4,
                             pump_prime_time_s=0.25)
        c = CommonPrintSettings()
        c.set_hardware_config(hw)
        c.set("service_z", 0.9, notify=False)
        page.set_hardware_config(hw)
        page.set_common_print_settings(c)
        self.assertAlmostEqual(page._widgets["pump_settle_time_s"].value(), 0.4)
        self.assertAlmostEqual(page._widgets["service_z"].value(), 0.9)

    def test_editing_promoted_updates_model(self):
        page = self._page()
        c = CommonPrintSettings()
        page.set_common_print_settings(c)
        page._widgets["service_z"].setValue(1.1)
        self.assertAlmostEqual(c.get("service_z"), 1.1)

    def test_editing_global_writes_through_to_hw_config(self):
        page = self._page()
        hw = SimpleNamespace(pump_settle_time_s=0.0,
                             pump_prime_time_s=0.25)
        c = CommonPrintSettings()
        c.set_hardware_config(hw)
        page.set_hardware_config(hw)
        page.set_common_print_settings(c)
        page._widgets["pump_prime_time_s"].setValue(1.3)
        self.assertAlmostEqual(hw.pump_prime_time_s, 1.3)


# ════════════════════════════════════════════════════════════════════
#  WorkflowsModePage fan-out
# ════════════════════════════════════════════════════════════════════

class TestWorkflowsModeFanout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_tile_registered_and_fanout_reaches_pages(self):
        from gui.pages.workflows_mode import WorkflowsModePage
        wm = WorkflowsModePage(controller=None, settings=None,
                               camera_manager=None)
        self.assertIn("common_print_settings", wm._workflow_index)
        hw = SimpleNamespace(pump_settle_time_s=0.0,
                             pump_prime_time_s=0.25, pumps={})
        c = CommonPrintSettings()
        c.set_hardware_config(hw)
        c.set("service_z", 0.66, notify=False)
        wm.set_hardware_config(hw)
        wm.set_common_print_settings(c)
        # The Common page reflects the value …
        cp = wm._stack.widget(wm._workflow_index["common_print_settings"])
        self.assertAlmostEqual(cp._widgets["service_z"].value(), 0.66)
        # … and an inheriting workflow field mirrors it.
        sp = wm._stack.widget(wm._workflow_index["spheroid_pickup"])
        self.assertAlmostEqual(sp._service_z.value(), 0.66)


if __name__ == "__main__":
    unittest.main()
