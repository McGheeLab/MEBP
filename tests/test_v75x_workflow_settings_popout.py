"""test_v75x_workflow_settings_popout.py — per-workflow settings popout.

Covers the v7.5.x comprehensive, saveable settings popout shared by every
workflow:

  * WorkflowSettingsStore  — named profile + last-used JSON round-trip,
    list/delete, import/export, filename sanitising, bare-dict tolerance.
  * WorkflowSettingsDialog — generic field registry get/set/reset for each
    widget type, pending-combo resolution, add_check, locations panel.
  * Backend — SpheroidPickupConfig pick/place dwell (default 0 = no-op).
  * Each workflow page builds offscreen, exposes its config widgets, opens the
    popout, and round-trips a saved profile through the page.

No hardware / Qt event loop is needed beyond an offscreen QApplication.
"""

import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Isolate the on-disk settings dir so pages' load_last/save_last never touch the
# repo's config/workflows during the test run.
_SETTINGS_TMP = tempfile.mkdtemp(prefix="mebp_wf_settings_")
os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = _SETTINGS_TMP

from PySide6.QtWidgets import (  # noqa: E402
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QSpinBox,
)

from SupportClasses.WorkflowSettingsStore import WorkflowSettingsStore  # noqa: E402
from gui.dialogs.workflow_settings_dialog import (  # noqa: E402
    WorkflowSettingsDialog, build_locations_widget, set_widget_value,
    widget_value,
)
from SupportClasses.PickAndPlaceManager import (  # noqa: E402
    OperationQueue, OperationType, PickPlaceExecutor, PickPlaceOperation,
    PickPlaceTarget, SpheroidPickupConfig,
)

_app = None


def setUpModule():
    global _app
    _app = QApplication.instance() or QApplication(sys.argv)


# ════════════════════════════════════════════════════════════════════
#  Store
# ════════════════════════════════════════════════════════════════════

class TestWorkflowSettingsStore(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="mebp_store_")
        self.store = WorkflowSettingsStore("spheroid_pickup", base_dir=self.dir)

    def test_save_load_round_trip(self):
        self.store.save_profile("Big slow", {"diameter": 300.0, "prep": True})
        self.assertEqual(self.store.load_profile("Big slow"),
                         {"diameter": 300.0, "prep": True})

    def test_list_profiles_excludes_last(self):
        self.store.save_profile("A", {"x": 1})
        self.store.save_profile("B", {"x": 2})
        self.store.save_last({"x": 9})
        self.assertEqual(self.store.list_profiles(), ["A", "B"])

    def test_load_last(self):
        self.store.save_last({"flow": 0.4})
        self.assertEqual(self.store.load_last(), {"flow": 0.4})

    def test_delete_profile(self):
        self.store.save_profile("Temp", {"x": 1})
        self.assertIn("Temp", self.store.list_profiles())
        self.assertTrue(self.store.delete_profile("Temp"))
        self.assertNotIn("Temp", self.store.list_profiles())

    def test_missing_profile_returns_none(self):
        self.assertIsNone(self.store.load_profile("nope"))
        self.assertIsNone(self.store.load_last())

    def test_filename_sanitised(self):
        # A name with path separators must not escape the workflow directory.
        self.store.save_profile("a/b:c", {"x": 1})
        names = self.store.list_profiles()
        self.assertEqual(len(names), 1)
        self.assertEqual(self.store.load_profile("a/b:c"), {"x": 1})

    def test_import_export_arbitrary_file(self):
        path = os.path.join(self.dir, "shared.json")
        self.store.write_file(path, {"speed": 7.0})
        self.assertEqual(self.store.read_file(path), {"speed": 7.0})

    def test_tolerates_bare_values_dict(self):
        # A hand-edited file that is just the values dict still loads.
        import json
        path = os.path.join(self.dir, "bare.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"a": 1, "b": 2}, f)
        self.assertEqual(self.store.read_file(path), {"a": 1, "b": 2})

    def test_separate_workflows_isolated(self):
        other = WorkflowSettingsStore("quick_print", base_dir=self.dir)
        self.store.save_profile("X", {"k": 1})
        self.assertEqual(other.list_profiles(), [])

    def test_reserved_last_name_does_not_clobber_last(self):
        # A user profile literally named "__last__" must not overwrite the
        # auto-saved last-used file, and must still round-trip + list.
        self.store.save_last({"auto": 1})
        self.store.save_profile("__last__", {"user": 2})
        self.assertEqual(self.store.load_last(), {"auto": 1})
        self.assertEqual(self.store.load_profile("__last__"), {"user": 2})
        self.assertIn("__last__", self.store.list_profiles())

    def test_write_failure_raises(self):
        # An explicit save to an unwritable location raises (so the UI can warn)
        # rather than silently succeeding. A regular FILE used as the base dir
        # makes the store's mkdir(parents=True) fail portably.
        blocker = os.path.join(self.dir, "not_a_dir")
        with open(blocker, "w", encoding="utf-8") as f:
            f.write("x")
        bad = WorkflowSettingsStore("spheroid_pickup", base_dir=blocker)
        with self.assertRaises(Exception):
            bad.save_profile("P", {"a": 1})


# ════════════════════════════════════════════════════════════════════
#  Generic widget get/set
# ════════════════════════════════════════════════════════════════════

class TestWidgetValue(unittest.TestCase):
    def test_double_spin(self):
        w = QDoubleSpinBox()
        w.setRange(0, 100)
        w.setValue(3.5)
        self.assertEqual(widget_value(w), 3.5)
        self.assertTrue(set_widget_value(w, 9.0))
        self.assertEqual(w.value(), 9.0)

    def test_int_spin(self):
        w = QSpinBox()
        w.setRange(0, 100)
        self.assertTrue(set_widget_value(w, 7))
        self.assertEqual(widget_value(w), 7)

    def test_checkbox(self):
        w = QCheckBox()
        self.assertTrue(set_widget_value(w, True))
        self.assertIs(widget_value(w), True)

    def test_combo_by_data_then_text(self):
        w = QComboBox()
        w.addItem("Alpha", "a")
        w.addItem("Beta", "b")
        self.assertTrue(set_widget_value(w, {"text": "Beta", "data": "b"}))
        self.assertEqual(w.currentData(), "b")
        tok = widget_value(w)
        self.assertEqual(tok["data"], "b")

    def test_combo_not_present_is_unresolved(self):
        w = QComboBox()  # empty
        self.assertFalse(set_widget_value(w, {"text": "X", "data": "x"}))


# ════════════════════════════════════════════════════════════════════
#  Dialog field registry
# ════════════════════════════════════════════════════════════════════

class TestSettingsDialog(unittest.TestCase):
    def _dialog(self):
        dlg = WorkflowSettingsDialog("spheroid_pickup", "Test")
        sec = dlg.add_section("S")
        self.sp = QDoubleSpinBox()
        self.sp.setRange(0, 100)
        self.sp.setValue(5.0)
        self.cb = QCheckBox()
        self.cb.setChecked(True)
        self.co = QComboBox()
        self.co.addItems(["P1", "P2"])
        sec.add("flow", "Flow", self.sp, 1.0)
        sec.add_check("on", self.cb, False)
        sec.add("bore", "Bore", self.co, "P1")
        dlg.finalize()
        return dlg

    def test_collect(self):
        dlg = self._dialog()
        v = dlg.collect()
        self.assertEqual(v["flow"], 5.0)
        self.assertIs(v["on"], True)

    def test_apply(self):
        dlg = self._dialog()
        dlg.apply({"flow": 9.5, "on": False, "bore": {"text": "P2", "data": None}})
        self.assertEqual(self.sp.value(), 9.5)
        self.assertFalse(self.cb.isChecked())
        self.assertEqual(self.co.currentText(), "P2")

    def test_reset_defaults(self):
        dlg = self._dialog()
        dlg.apply({"flow": 9.5, "on": True})
        dlg.reset_defaults()
        self.assertEqual(self.sp.value(), 1.0)
        self.assertFalse(self.cb.isChecked())  # default False

    def test_pending_combo_resolution(self):
        dlg = WorkflowSettingsDialog("quick_print", "T")
        sec = dlg.add_section("S")
        combo = QComboBox()  # empty at load time
        sec.add("ink", "Ink", combo, "")
        dlg.finalize()
        dlg.apply({"ink": {"text": "trypsin", "data": "trypsin"}})
        self.assertIn("ink", dlg._pending)        # not resolvable yet
        combo.addItem("trypsin", "trypsin")
        dlg.resolve_pending()
        self.assertNotIn("ink", dlg._pending)
        self.assertEqual(combo.currentText(), "trypsin")

    def test_reset_restores_none_combo(self):
        # A combo whose default is the empty "(none)" selection must be restored
        # by Reset (the "(none)" item carries userData "").
        dlg = WorkflowSettingsDialog("quick_print", "T")
        sec = dlg.add_section("S")
        combo = QComboBox()
        combo.addItem("(none)", "")
        combo.addItem("Trypsin", "trypsin")
        sec.add("ink", "Ink", combo, "")   # default = "(none)"
        dlg.finalize()
        combo.setCurrentIndex(1)            # user picks Trypsin
        self.assertEqual(combo.currentData(), "trypsin")
        dlg.reset_defaults()
        self.assertEqual(combo.currentData(), "")   # back to "(none)"

    def test_reapply_combo_survives_repopulate_autodefault(self):
        # The restored "(none)" must survive a combo repopulate that auto-defaults
        # (e.g. to the pump's assigned ink): resolve_pending re-asserts it once.
        dlg = WorkflowSettingsDialog("quick_print", "T")
        sec = dlg.add_section("S")
        combo = QComboBox()
        combo.addItem("(none)", "")
        combo.addItem("CellInk", "CellInk")
        sec.add("ink", "Ink", combo, "")
        dlg.finalize()
        dlg.apply({"ink": {"text": "(none)", "data": ""}}, reapply_combos=True)
        self.assertEqual(combo.currentData(), "")
        combo.setCurrentIndex(1)            # page auto-default to assigned ink
        dlg.resolve_pending()              # re-asserts the restored "(none)"
        self.assertEqual(combo.currentData(), "")

    def test_on_change_called(self):
        calls = []
        dlg = WorkflowSettingsDialog("spheroid_pickup", "T",
                                     on_change=lambda: calls.append(1))
        sec = dlg.add_section("S")
        sp = QDoubleSpinBox()
        sec.add("x", "X", sp, 0.0)
        dlg.finalize()
        dlg.apply({"x": 1.0})
        dlg.reset_defaults()
        self.assertGreaterEqual(len(calls), 2)


# ════════════════════════════════════════════════════════════════════
#  Locations & Hardware panel
# ════════════════════════════════════════════════════════════════════

def _fake_hw():
    ink = lambda t, c: SimpleNamespace(ink_type=t, color=c)  # noqa: E731
    needle = SimpleNamespace(
        gauge=27, id_um=210.0, od_um=410.0, length_mm=25.4,
        internal_volume_uL=0.88, cross_section_area_mm2=0.0346)
    pump = SimpleNamespace(enabled=True, is_configured=True,
                           syringe=SimpleNamespace(volume_uL=250),
                           ink_names=["CellInk"])
    return SimpleNamespace(
        needle=needle, pumps={"P1": pump},
        ink_library={
            "CellInk": ink("cells", "#a6e3a1"),
            "Waste": ink("waste", "#888"),
            "Oil": ink("oil", "#cc0"),
            "Wash": ink("wash", "#0cc"),
            "Buffer": ink("buffer", "#00c"),
        },
        ink_locations={"CellInk": ["B2"], "Waste": ["A1"], "Oil": ["A2"],
                       "Wash": ["A3"], "Buffer": ["A4"]},
    )


class TestLocationsPanel(unittest.TestCase):
    def test_builds_from_fake_hw(self):
        wp = {"A1": (1000, 1000), "A2": (2000, 1000), "A3": (3000, 1000),
              "A4": (4000, 1000), "B2": (2000, 2000)}
        w = build_locations_widget(
            MagicMock(), _fake_hw(), wp,
            z_references={"plate_bottom_z": -26.0}, safe_z=-35.0,
            extras=[("Print ink", "CellInk ← B2")])
        self.assertIsNotNone(w)

    def test_builds_with_none_config(self):
        # Defensive: a None hw_config / empty positions must not raise.
        w = build_locations_widget(MagicMock(), None, None)
        self.assertIsNotNone(w)


# ════════════════════════════════════════════════════════════════════
#  Backend — spheroid pick/place dwell
# ════════════════════════════════════════════════════════════════════

class _RecCtrl:
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
        return True

    def wait_for_z_arrival(self, *a, **k):
        return True

    def ensure_retracted_to(self, safe_z, *a, **k):
        self.calls.append(("ensure_retracted_to", safe_z))
        return True

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, settle=False):
        self.calls.append(("move_pump_uL", pump, volume_uL, rate_uL_s))


class TestSpheroidDwell(unittest.TestCase):
    def test_config_has_dwell_fields(self):
        cfg = SpheroidPickupConfig(pick_dwell_s=2.0, place_dwell_s=3.0)
        d = cfg.to_dict()
        self.assertEqual(d["pick_dwell_s"], 2.0)
        self.assertEqual(d["place_dwell_s"], 3.0)

    def test_dwell_default_zero(self):
        cfg = SpheroidPickupConfig()
        self.assertEqual(cfg.pick_dwell_s, 0.0)
        self.assertEqual(cfg.place_dwell_s, 0.0)

    def _run(self, **cfg_kw):
        ctrl = _RecCtrl()
        ex = PickPlaceExecutor(ctrl, hw_config=None)
        ex.safe_z_mm = -35.0
        ex.pick_z_mm = -16.0
        ex.place_z_mm = -14.0
        steps = []
        ex.on_sub_step = lambda op, txt: steps.append(txt)
        cfg = SpheroidPickupConfig(
            spheroid_diameter_um=200.0, pickup_bore="P1", **cfg_kw)
        op = PickPlaceOperation(
            op_id="OP1", op_type=OperationType.SPHEROID_PICKUP,
            source_target=PickPlaceTarget("S", 1000.0, 1000.0, ""),
            dest_target=PickPlaceTarget("D", 2000.0, 2000.0, ""),
            config=cfg)
        q = OperationQueue()
        q.add(op)
        ok = ex.execute_queue(q)
        return ok, steps

    def test_dwell_invoked_when_set(self):
        ok, steps = self._run(pick_dwell_s=0.02, place_dwell_s=0.02)
        self.assertTrue(ok)
        joined = " ".join(steps)
        self.assertIn("Pick pause", joined)
        self.assertIn("Place pause", joined)

    def test_no_dwell_step_when_zero(self):
        ok, steps = self._run(pick_dwell_s=0.0, place_dwell_s=0.0)
        self.assertTrue(ok)
        joined = " ".join(steps)
        self.assertNotIn("pause", joined.lower())


# ════════════════════════════════════════════════════════════════════
#  Workflow pages — build, expose config, popout round-trip
# ════════════════════════════════════════════════════════════════════

def _mock_ctrl():
    ctrl = MagicMock()
    ctrl.is_xy_connected = True
    ctrl.is_zp_connected = True
    ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    ctrl.safety_limits = SimpleNamespace(
        xy_min_x=0, xy_max_x=100000, xy_min_y=0, xy_max_y=80000,
        z_min=-60, z_max=0, max_xy_speed=50000)
    ctrl.get_xy_position.return_value = (100.0, 100.0)
    ctrl.get_zp_position.return_value = None
    ctrl.print_height_to_zref.return_value = -25.0
    ctrl.print_z_dir.return_value = -1.0
    return ctrl


class TestWorkflowPages(unittest.TestCase):
    def setUp(self):
        # Per-test settings dir so hiding a page's dialog (which auto-saves
        # __last__) can't pollute other tests' page constructions.
        self._prev_env = os.environ.get("MEBP_WORKFLOW_SETTINGS_DIR")
        os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = tempfile.mkdtemp(
            prefix="mebp_pagetest_")

    def tearDown(self):
        if self._prev_env is None:
            os.environ.pop("MEBP_WORKFLOW_SETTINGS_DIR", None)
        else:
            os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = self._prev_env

    def _page(self, kind):
        if kind == "spheroid_pickup":
            from gui.pages.workflows.spheroid_pickup_workflow import (
                SpheroidPickupWorkflowPage as P)
            return P(controller=_mock_ctrl(), settings=None, camera_manager=None)
        if kind == "cell_targeting":
            from gui.pages.workflows.cell_targeting_workflow import (
                CellTargetingWorkflowPage as P)
            return P(controller=_mock_ctrl(), settings=None, camera_manager=None)
        if kind == "quick_print":
            from gui.pages.workflows.quick_print_workflow import (
                QuickPrintWorkflowPage as P)
            return P(_mock_ctrl(), settings=None)
        if kind == "stress_test":
            from gui.pages.workflows.stress_test_workflow import (
                StressTestWorkflowPage as P)
            return P(_mock_ctrl(), settings=None)
        if kind == "timing_calibration":
            from gui.pages.workflows.timing_calibration_workflow import (
                TimingCalibrationWorkflowPage as P)
            return P(_mock_ctrl(), settings=None)
        raise ValueError(kind)

    ALL = ("spheroid_pickup", "cell_targeting", "quick_print",
           "stress_test", "timing_calibration")

    def test_all_pages_build_and_have_dialog(self):
        for kind in self.ALL:
            page = self._page(kind)
            self.assertTrue(hasattr(page, "_settings_dialog"))
            self.assertGreater(len(page._settings_dialog.collect()), 0,
                               f"{kind} has no registered settings")

    def test_open_settings_and_info(self):
        for kind in self.ALL:
            page = self._page(kind)
            page._open_settings()
            self.assertTrue(page._settings_dialog.isVisible())
            page._settings_dialog._refresh_info()  # must not raise
            page._settings_dialog.hide()

    def test_profile_round_trip_through_page(self):
        # Save the current values as a profile, change a field, reload → restored.
        page = self._page("spheroid_pickup")
        dlg = page._settings_dialog
        page._pick_flow.setValue(0.4)
        page._place_flow.setValue(0.7)
        dlg._store.save_profile("RT", dlg.collect())
        page._pick_flow.setValue(2.0)
        page._place_flow.setValue(2.0)
        dlg.apply(dlg._store.load_profile("RT"))
        self.assertAlmostEqual(page._pick_flow.value(), 0.4)
        self.assertAlmostEqual(page._place_flow.value(), 0.7)
        # And those values flow into the config used by the run.
        cfg = page._current_config()
        self.assertAlmostEqual(cfg.pickup_speed_uL_s, 0.4)
        self.assertAlmostEqual(cfg.release_speed_uL_s, 0.7)

    def test_reset_restores_defaults(self):
        page = self._page("spheroid_pickup")
        page._diameter.setValue(999.0)
        page._settings_dialog.reset_defaults()
        self.assertAlmostEqual(page._diameter.value(), 200.0)

    def test_spheroid_wash_widgets_enabled_for_clean_only(self):
        # Wash + buffer knobs are shared by prep AND post-clean, so they must be
        # editable when post-clean is on even if prep is off (review finding #10).
        page = self._page("spheroid_pickup")
        page._prep_check.setChecked(False)
        page._post_clean_check.setChecked(True)
        page._on_prep_toggled()
        self.assertTrue(page._wash_cycles.isEnabled())
        self.assertTrue(page._buffer_needles.isEnabled())
        self.assertTrue(page._post_dispense.isEnabled())
        # …and disabled when neither prep nor clean is on.
        page._post_clean_check.setChecked(False)
        page._on_prep_toggled()
        self.assertFalse(page._wash_cycles.isEnabled())
        self.assertFalse(page._post_dispense.isEnabled())

    def test_quick_print_size_stays_inline(self):
        # The object size control stays on the page (drives the live preview),
        # so its page-relative visibility still toggles with the object kind.
        page = self._page("quick_print")
        idx = page._object_combo.findData("simple:dot")
        if idx >= 0:
            page._object_combo.setCurrentIndex(idx)
            page._on_object_changed()
            self.assertFalse(page._size_spin.isVisibleTo(page))
        idx = page._object_combo.findData("simple:circle")
        if idx >= 0:
            page._object_combo.setCurrentIndex(idx)
            page._on_object_changed()
            self.assertTrue(page._size_spin.isVisibleTo(page))


class TestACheckableButtonIsABooleanField(unittest.TestCase):
    """v7.19.3 — ``widget_value`` recognised only ``QCheckBox``.

    ``register_external`` accepts any widget, so the fluorescence panel's
    checkable ``QPushButton`` preset toggle was registered, silently read back
    as ``None``, never written, and reset on every launch. A checkable button IS
    a boolean control.
    """

    def test_a_checkable_button_round_trips(self):
        from gui.dialogs.workflow_settings_dialog import (
            set_widget_value, widget_value)
        from PySide6.QtWidgets import QPushButton
        b = QPushButton("toggle")
        b.setCheckable(True)
        b.setChecked(True)
        self.assertIs(widget_value(b), True)
        self.assertTrue(set_widget_value(b, False))
        self.assertIs(widget_value(b), False)

    def test_a_plain_button_is_still_unrecognised(self):
        """Widening this must not start persisting every action button."""
        from gui.dialogs.workflow_settings_dialog import (
            connect_widget_changed, set_widget_value, widget_value)
        from PySide6.QtWidgets import QPushButton
        b = QPushButton("Run")           # not checkable
        self.assertIsNone(widget_value(b))
        self.assertFalse(set_widget_value(b, True))
        self.assertFalse(connect_widget_changed(b, lambda *_: None))

    def test_a_checkbox_behaves_exactly_as_before(self):
        from gui.dialogs.workflow_settings_dialog import (
            connect_widget_changed, set_widget_value, widget_value)
        from PySide6.QtWidgets import QCheckBox
        c = QCheckBox("x")
        c.setChecked(False)
        self.assertIs(widget_value(c), False)
        self.assertTrue(set_widget_value(c, True))
        self.assertIs(widget_value(c), True)
        seen = []
        self.assertTrue(connect_widget_changed(c, lambda *_: seen.append(1)))
        c.setChecked(False)
        self.assertEqual(len(seen), 1)


if __name__ == "__main__":
    unittest.main()
