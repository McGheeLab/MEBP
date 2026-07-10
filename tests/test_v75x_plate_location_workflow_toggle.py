"""
test_v75x_plate_location_workflow_toggle.py

v7.5.x — the Plate Location tab gains a workflow-VERSION toggle:
"Mosaic workflow (recommended)" (the default) vs the deprecated
"Legacy well-fit / target queue". Only widget visibility switches — every
legacy widget/handler stays alive and wired until the flow is actually
removed. The choice persists in settings.json
("plate_location_prefs" → "workflow").

Covered:
  * default = mosaic (queue group / well-fit combo / queue status hidden;
    mosaic box + live mosaic preview shown);
  * switching flips visibility + banner text (idempotent both ways);
  * persistence: switching writes the pref + save(); a stored "legacy"
    pref restores legacy mode at build; garbage → mosaic;
  * run guard: switching is refused (combo reverts) while a queue run or
    mosaic scan is active;
  * click gating: plate-view clicks only mutate the queue in legacy mode;
    re-anchor overview clicks still route in mosaic mode;
  * legacy flow still fully wired when shown.

Offscreen note: ``isVisible()`` lies for never-shown widgets — assert via
``isHidden()`` (the explicit-hide flag) instead.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

_APP = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.WellPlate import WellPlate  # noqa: E402


class _FakeSettings:
    """Minimal settings stand-in recording section writes."""

    def __init__(self, sections=None):
        self.sections = dict(sections or {})
        self.values = {}
        self.saved = 0

    def get_section(self, name):
        return self.sections.get(name)

    def set_section(self, name, value):
        self.sections[name] = value

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value

    def save(self):
        self.saved += 1


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(sys.argv)

    def _make_page(self, settings=None):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.get_zp_position.return_value = (None, None, None)
        ctrl.default_plate_center_um.return_value = (50000.0, 40000.0)
        ctrl.plate_axis_sign.return_value = (1.0, 1.0)
        ctrl.plate_flip_180.return_value = False
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        page = CalibrationPage(ctrl, settings=settings)
        page._plate = WellPlate.from_format(6)
        return page

    def _combo_data(self, page):
        return page._ploc_workflow_combo.currentData()

    def _set_combo(self, page, mode):
        """Drive the combo like the user would (signals fire)."""
        idx = page._ploc_workflow_combo.findData(mode)
        self.assertGreaterEqual(idx, 0)
        page._ploc_workflow_combo.setCurrentIndex(idx)


class TestDefaultMosaic(_Base):
    def test_defaults_to_mosaic(self):
        page = self._make_page()
        self.assertEqual(page._ploc_workflow_mode, "mosaic")
        self.assertEqual(self._combo_data(page), "mosaic")
        # Legacy widgets hidden…
        self.assertTrue(page._ploc_queue_group.isHidden())
        self.assertTrue(page._ploc_mode_row_w.isHidden())
        self.assertTrue(page._ploc_status.isHidden())
        # …mosaic widgets not.
        self.assertFalse(page._ploc_mosaic_box.isHidden())
        self.assertFalse(page._ploc_mosaic_prev_col.isHidden())
        self.assertIn("Mosaic workflow", page._ploc_banner.text())

    def test_legacy_widgets_still_exist(self):
        page = self._make_page()
        for attr in ("_ploc_queue_list", "_ploc_btn_clear", "_ploc_btn_run",
                     "_ploc_mode_combo", "_ploc_legacy_note",
                     "_ploc_btn_confirm", "_ploc_btn_skip",
                     "_ploc_btn_cancel"):
            self.assertIsNotNone(getattr(page, attr), attr)


class TestSwitchModes(_Base):
    def test_switch_to_legacy_flips_visibility(self):
        page = self._make_page()
        page._ploc_set_workflow_mode("legacy", persist=False)
        self.assertEqual(page._ploc_workflow_mode, "legacy")
        self.assertFalse(page._ploc_queue_group.isHidden())
        self.assertFalse(page._ploc_mode_row_w.isHidden())
        self.assertFalse(page._ploc_status.isHidden())
        self.assertTrue(page._ploc_mosaic_box.isHidden())
        self.assertTrue(page._ploc_mosaic_prev_col.isHidden())
        self.assertIn("Deprecated", page._ploc_banner.text())

    def test_switch_back_is_idempotent(self):
        page = self._make_page()
        for _ in range(2):
            page._ploc_set_workflow_mode("legacy", persist=False)
            page._ploc_set_workflow_mode("mosaic", persist=False)
        self.assertEqual(page._ploc_workflow_mode, "mosaic")
        self.assertTrue(page._ploc_queue_group.isHidden())
        self.assertFalse(page._ploc_mosaic_box.isHidden())
        self.assertIn("Mosaic workflow", page._ploc_banner.text())

    def test_legacy_hides_align_group_and_unchecks_action(self):
        page = self._make_page()
        # Simulate the operator having opened the align sliders.
        page._ploc_act_manual_align.setChecked(True)
        page._ploc_align_group.setChecked(True)
        page._ploc_set_workflow_mode("legacy", persist=False)
        self.assertTrue(page._ploc_align_group.isHidden())
        self.assertFalse(page._ploc_align_group.isChecked())
        self.assertFalse(page._ploc_act_manual_align.isChecked())

    def test_combo_drives_mode(self):
        page = self._make_page()
        self._set_combo(page, "legacy")
        self.assertEqual(page._ploc_workflow_mode, "legacy")
        self._set_combo(page, "mosaic")
        self.assertEqual(page._ploc_workflow_mode, "mosaic")

    def test_bogus_mode_coerces_to_mosaic(self):
        page = self._make_page()
        page._ploc_set_workflow_mode("nonsense", persist=False)
        self.assertEqual(page._ploc_workflow_mode, "mosaic")


class TestPersistence(_Base):
    def test_switch_persists_pref(self):
        st = _FakeSettings()
        page = self._make_page(settings=st)
        saved_before = st.saved
        self._set_combo(page, "legacy")
        prefs = st.sections.get("plate_location_prefs")
        self.assertIsInstance(prefs, dict)
        self.assertEqual(prefs.get("workflow"), "legacy")
        self.assertGreater(st.saved, saved_before)

    def test_persist_merges_existing_prefs(self):
        st = _FakeSettings({"plate_location_prefs": {"other": 1}})
        page = self._make_page(settings=st)
        self._set_combo(page, "legacy")
        prefs = st.sections["plate_location_prefs"]
        self.assertEqual(prefs.get("workflow"), "legacy")
        self.assertEqual(prefs.get("other"), 1)

    def test_stored_legacy_restores_at_build(self):
        st = _FakeSettings({"plate_location_prefs": {"workflow": "legacy"}})
        page = self._make_page(settings=st)
        self.assertEqual(page._ploc_workflow_mode, "legacy")
        self.assertEqual(self._combo_data(page), "legacy")
        self.assertFalse(page._ploc_queue_group.isHidden())
        self.assertTrue(page._ploc_mosaic_box.isHidden())
        # The restore itself didn't rewrite the pref.
        self.assertEqual(st.sections["plate_location_prefs"],
                         {"workflow": "legacy"})

    def test_garbage_pref_defaults_to_mosaic(self):
        st = _FakeSettings({"plate_location_prefs": {"workflow": "bogus"}})
        page = self._make_page(settings=st)
        self.assertEqual(page._ploc_workflow_mode, "mosaic")

    def test_missing_pref_defaults_to_mosaic(self):
        st = _FakeSettings()
        page = self._make_page(settings=st)
        self.assertEqual(page._ploc_workflow_mode, "mosaic")


class TestRunGuard(_Base):
    def test_refused_during_queue_run(self):
        page = self._make_page()
        page._ploc_running = True
        with patch("gui.pages.calibration.QMessageBox") as mb:
            self._set_combo(page, "legacy")
        self.assertEqual(page._ploc_workflow_mode, "mosaic")
        self.assertEqual(self._combo_data(page), "mosaic")   # reverted
        mb.information.assert_called()

    def test_refused_during_mosaic_scan(self):
        page = self._make_page()
        page._ploc_mosaic_running = True
        with patch("gui.pages.calibration.QMessageBox") as mb:
            self._set_combo(page, "legacy")
        self.assertEqual(page._ploc_workflow_mode, "mosaic")
        self.assertEqual(self._combo_data(page), "mosaic")
        mb.information.assert_called()

    def test_allowed_when_idle(self):
        page = self._make_page()
        with patch("gui.pages.calibration.QMessageBox") as mb:
            self._set_combo(page, "legacy")
        self.assertEqual(page._ploc_workflow_mode, "legacy")
        mb.information.assert_not_called()


class TestClickGating(_Base):
    def test_well_click_ignored_in_mosaic_mode(self):
        page = self._make_page()
        page._ploc_on_well_clicked("A1")
        self.assertEqual(page._ploc_queue, [])

    def test_well_click_enqueues_in_legacy_mode(self):
        page = self._make_page()
        page._ploc_set_workflow_mode("legacy", persist=False)
        page._ploc_on_well_clicked("A1")
        self.assertEqual(page._ploc_queue, ["A1"])
        page._ploc_on_well_clicked("A1")           # toggle off
        self.assertEqual(page._ploc_queue, [])

    def test_reanchor_overview_click_still_routes_in_mosaic_mode(self):
        page = self._make_page()
        page._ploc_reanchor_stage = "await_overview"
        page._predict_well_xy = lambda name: (111.0, 222.0)
        page._ploc_reanchor_overview = MagicMock()
        page._ploc_on_well_clicked("A1")
        page._ploc_reanchor_overview.assert_called_once_with(
            111.0, 222.0, "A1")
        self.assertEqual(page._ploc_queue, [])

    def test_position_click_ignored_in_mosaic_mode(self):
        page = self._make_page()
        page._ploc_plate_view.target_mode = lambda: "free"
        page._ploc_on_position_clicked(10.0, 20.0)
        self.assertEqual(page._ploc_queue, [])

    def test_position_click_enqueues_in_legacy_free_mode(self):
        page = self._make_page()
        page._ploc_set_workflow_mode("legacy", persist=False)
        page._ploc_plate_view.target_mode = lambda: "free"
        page._ploc_on_position_clicked(10.0, 20.0)
        self.assertEqual(len(page._ploc_queue), 1)
        self.assertEqual(page._ploc_queue[0].get("kind"), "free")


class TestLegacyStillWired(_Base):
    def test_run_button_enables_on_enqueue(self):
        page = self._make_page()
        page._ploc_set_workflow_mode("legacy", persist=False)
        self.assertFalse(page._ploc_btn_run.isEnabled())
        page._ploc_on_well_clicked("A1")
        self.assertTrue(page._ploc_btn_run.isEnabled())
        self.assertEqual(page._ploc_queue_list.count(), 1)


if __name__ == "__main__":
    unittest.main()
