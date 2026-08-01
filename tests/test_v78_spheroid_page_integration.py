"""test_v78_spheroid_page_integration.py — the page wiring, built for real.

The whole Spheroid Pick & Place page is constructed offscreen (both tabs, the
embedded fluorescence scan page, the survey panel and the mosaic overlay), then
its motion paths are exercised against a recording fake.

The load-bearing assertions are the safety ones:

* every new travel path goes through ``SafeTravelWorker`` with
  ``target_z_mm=None`` (retract, wait, travel, never descend) and NEVER calls a
  bare ``move_xy_absolute*``;
* absolute-µm and zero-ref-µm handlers stay separate, so the zero offset is
  never added twice;
* nothing travels while the executor thread or a mosaic scan is driving the stage.
"""

from __future__ import annotations

import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

try:
    from PySide6.QtWidgets import QApplication
    _QT = True
except Exception:      # pragma: no cover
    _QT = False


ZERO = {"x": 5000.0, "y": 7000.0, "Z": 0.0}


class _Ctrl:
    """Recording controller stub — never actually moves anything."""

    def __init__(self):
        self.zero_position = dict(ZERO)
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.calls: list = []
        self.safety_limits = SimpleNamespace(
            xy_min_x=0.0, xy_min_y=0.0, xy_max_x=120000.0, xy_max_y=90000.0,
            z_min=-60.0, z_max=0.0, max_xy_speed=20000.0)

    def get_xy_position(self, cached=True):
        return (10000.0, 12000.0, 0.0)

    def get_zp_position(self, cached=True):
        return (0.0, 0.0, 0.0, 0.0)

    def zp_logical_value(self, zp, axis):
        return 0.0

    def suspend_position_poller(self):
        pass

    def resume_position_poller(self):
        pass

    def safe_travel_to(self, *a, **k):
        self.calls.append(("safe_travel_to", a, k))
        return True

    def move_xy_absolute(self, *a, **k):
        self.calls.append(("move_xy_absolute", a, k))

    def move_xy_absolute_um(self, *a, **k):
        self.calls.append(("move_xy_absolute_um", a, k))

    def move_z_absolute(self, *a, **k):
        self.calls.append(("move_z_absolute", a, k))

    def print_height_to_zref(self, offset_mm):
        return -10.0 + float(offset_mm)

    def print_z_dir(self):
        return -1.0


class _RecordingWorker:
    """Stands in for SafeTravelWorker, recording what it was asked to do."""

    def __init__(self):
        self.starts: list = []

    def start(self, controller, x, y, **kw):
        self.starts.append({"x": x, "y": y, **kw})
        return True


def _manager(um_per_px=2.0):
    from gui.widgets.camera_manager import CameraManager
    mgr = CameraManager.__new__(CameraManager)
    mgr._max_cameras = 3
    mgr._cameras = []
    mgr._um_per_px = [um_per_px] * 3
    mgr._um_per_px_res = [None] * 3
    mgr._um_per_px_set = [True] * 3
    mgr._rotation_deg = [None] * 3
    mgr._column_dir_deg = [None] * 3
    mgr._mirrored = [False] * 3
    mgr._flip_y = [False] * 3
    return mgr


def _settings():
    store: dict = {}
    return SimpleNamespace(
        get_section=lambda k, d=None: store.get(k, d if d is not None else {}),
        set_section=lambda k, v: store.__setitem__(k, v),
        save=lambda: None)


@unittest.skipUnless(_QT, "PySide6 not available")
class _PageCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.app = QApplication.instance() or QApplication([])
        except Exception as e:      # pragma: no cover
            raise unittest.SkipTest(f"Qt unavailable: {e}")

    def _page(self):
        from gui.pages.workflows.spheroid_pickup_workflow import (
            SpheroidPickupWorkflowPage)
        ctrl = _Ctrl()
        page = SpheroidPickupWorkflowPage(ctrl, _settings(), _manager())
        page._safe_z = -40.0
        page._travel_worker = _RecordingWorker()
        return page, ctrl


class TestStructure(_PageCase):
    def test_two_tabs(self):
        page, _c = self._page()
        self.assertEqual(page._tabs.count(), 2)
        # "&&" is Qt's escape for a literal ampersand in a tab label.
        self.assertEqual(page._tabs.tabText(0), "Pick && Place")
        self.assertEqual(page._tabs.tabText(1), "Spheroid survey")

    def test_survey_components_exist(self):
        page, _c = self._page()
        self.assertIsNotNone(page._scan_page)
        self.assertIsNotNone(page._survey)
        self.assertIsNotNone(page._mosaic_overlay)

    def test_embedded_scan_page_has_no_back_button(self):
        """embedded=True drops the header so the host owns navigation."""
        page, _c = self._page()
        self.assertTrue(page._scan_page._embedded)

    def test_mosaic_view_is_interactive(self):
        page, _c = self._page()
        self.assertTrue(page._scan_page.mosaic_view().interactive_items())

    def test_defaults_match_the_operator_decisions(self):
        page, _c = self._page()
        self.assertTrue(page._per_target_volume.isChecked())
        self.assertAlmostEqual(page._clearance.value(), 1.5)
        self.assertAlmostEqual(page._safety.minimum(), 0.01)
        self.assertAlmostEqual(page._safety.maximum(), 10.0)


class TestTravelSafety(_PageCase):
    def test_absolute_goto_retracts_and_never_descends(self):
        page, ctrl = self._page()
        self.assertTrue(page._travel_to_absolute(12345.0, 6789.0))
        self.assertEqual(len(page._travel_worker.starts), 1)
        call = page._travel_worker.starts[0]
        # EXACTLY the absolute numbers — no zero offset added.
        self.assertAlmostEqual(call["x"], 12345.0)
        self.assertAlmostEqual(call["y"], 6789.0)
        self.assertAlmostEqual(call["safe_z_mm"], -40.0)
        self.assertIsNone(call["target_z_mm"])
        # And no raw XY move slipped past the retract.
        self.assertEqual(
            [c[0] for c in ctrl.calls if "move_xy" in c[0]], [])

    def test_zero_ref_click_adds_the_zero_offset(self):
        """The other frame: a workspace click IS zero-ref and must be offset."""
        page, _c = self._page()
        page._on_workspace_position_clicked(1000.0, 2000.0)
        call = page._travel_worker.starts[0]
        self.assertAlmostEqual(call["x"], 1000.0 + ZERO["x"])
        self.assertAlmostEqual(call["y"], 2000.0 + ZERO["y"])

    def test_the_two_frames_do_not_share_a_handler(self):
        """Feeding the same numbers to both must give DIFFERENT destinations."""
        page, _c = self._page()
        page._travel_to_absolute(1000.0, 2000.0)
        page._travel_worker.starts.clear()
        page._on_workspace_position_clicked(1000.0, 2000.0)
        self.assertNotAlmostEqual(page._travel_worker.starts[0]["x"], 1000.0)

    def test_refused_without_a_safe_z(self):
        page, _c = self._page()
        page._safe_z = None
        self.assertFalse(page._travel_to_absolute(1000.0, 2000.0))
        self.assertEqual(page._travel_worker.starts, [])
        self.assertIn("safe Z", page._status.text())

    def test_refused_when_the_zp_board_is_down(self):
        """Without ZP, safe_travel_to silently skips its retract."""
        page, ctrl = self._page()
        ctrl.is_zp_connected = False
        self.assertFalse(page._travel_to_absolute(1000.0, 2000.0))
        self.assertEqual(page._travel_worker.starts, [])
        self.assertIn("ZP", page._status.text())

    def test_refused_when_xy_is_down(self):
        page, ctrl = self._page()
        ctrl.is_xy_connected = False
        self.assertFalse(page._travel_to_absolute(1000.0, 2000.0))
        self.assertEqual(page._travel_worker.starts, [])

    def test_refused_while_the_executor_thread_runs(self):
        page, _c = self._page()
        page._exec_thread = SimpleNamespace(is_alive=lambda: True)
        self.assertFalse(page._travel_to_absolute(1000.0, 2000.0))
        self.assertIn("Busy running", page._status.text())

    def test_refused_while_a_mosaic_scan_runs(self):
        """Two stage drivers, plus a NON-refcounted poller suspend."""
        page, _c = self._page()
        page._scan_page.is_scanning = lambda: True
        self.assertFalse(page._travel_to_absolute(1000.0, 2000.0))
        self.assertIn("mosaic scan", page._status.text())

    def test_start_is_disabled_while_a_scan_runs(self):
        page, _c = self._page()
        page._scan_page.is_scanning = lambda: True
        page._update_button_state()
        self.assertFalse(page._start_btn.isEnabled())

    def test_survey_goto_uses_the_absolute_path(self):
        page, _c = self._page()
        page._on_spheroid_goto(54321.0, 12345.0)
        self.assertEqual(len(page._travel_worker.starts), 1)
        self.assertAlmostEqual(page._travel_worker.starts[0]["x"], 54321.0)
        self.assertIsNone(page._travel_worker.starts[0]["target_z_mm"])

    def test_row_goto_uses_the_targets_absolute_position(self):
        page, _c = self._page()
        tid = page._picker.add_pick(31000.0, 42000.0, 200.0)
        page._on_target_goto(tid)
        self.assertAlmostEqual(page._travel_worker.starts[0]["x"], 31000.0)
        self.assertAlmostEqual(page._travel_worker.starts[0]["y"], 42000.0)

    def test_row_goto_for_an_unknown_id_is_silent(self):
        page, _c = self._page()
        page._on_target_goto("nope")
        self.assertEqual(page._travel_worker.starts, [])


class TestTransfer(_PageCase):
    def test_transfer_adds_picks_with_size_and_mosaic_provenance(self):
        from gui.widgets.live_target_picker import PROV_MOSAIC
        page, _c = self._page()
        page._on_transfer_to_picks([
            {"x_um": 11000.0, "y_um": 22000.0, "diameter_um": 312.0,
             "det_id": "S001"},
            {"x_um": 11500.0, "y_um": 22500.0, "diameter_um": 150.0,
             "det_id": "S002"},
        ])
        picks = page._picker.picks()
        self.assertEqual(len(picks), 2)
        self.assertAlmostEqual(picks[0].size_um, 312.0)
        self.assertAlmostEqual(picks[0].x_um, 11000.0)
        # Flagged as an unconfirmed mosaic position.
        self.assertEqual(page._picker.provenance(picks[0].target_id),
                         PROV_MOSAIC)

    def test_transfer_switches_to_the_pick_tab(self):
        page, _c = self._page()
        page._tabs.setCurrentIndex(1)
        page._on_transfer_to_picks(
            [{"x_um": 1.0, "y_um": 2.0, "diameter_um": 100.0, "det_id": "S1"}])
        self.assertEqual(page._tabs.currentIndex(), 0)

    def test_transfer_says_positions_need_confirming(self):
        page, _c = self._page()
        page._on_transfer_to_picks(
            [{"x_um": 1.0, "y_um": 2.0, "diameter_um": 100.0, "det_id": "S1"}])
        self.assertIn("confirm", page._survey._status.text().lower())

    def test_empty_transfer_is_a_no_op(self):
        page, _c = self._page()
        page._on_transfer_to_picks([])
        self.assertEqual(len(page._picker.picks()), 0)

    def test_transferred_diameters_drive_per_pick_volumes(self):
        """End to end: transfer → per-pair config → different aspirate volumes."""
        page, _c = self._page()
        page._on_transfer_to_picks([
            {"x_um": 1.0, "y_um": 2.0, "diameter_um": 150.0, "det_id": "S1"},
            {"x_um": 3.0, "y_um": 4.0, "diameter_um": 320.0, "det_id": "S2"},
        ])
        base = page._current_config()
        cfgs = [page._config_for_pick(base, pk) for pk in page._picker.picks()]
        v = [c.compute_volume_uL() for c in cfgs]
        self.assertAlmostEqual(cfgs[0].spheroid_diameter_um, 150.0)
        self.assertAlmostEqual(cfgs[1].spheroid_diameter_um, 320.0)
        self.assertGreater(v[1], v[0] * 5)      # (320/150)^3 ≈ 9.7


class TestMosaicCircleWiring(_PageCase):
    def _with_mosaic(self):
        page, ctrl = self._page()
        extent = (1000.0, 2000.0, 2600.0, 3200.0)
        ctx = {
            "plate_key": "24", "well": "A1", "channel": "DAPI",
            "channels": ["DAPI"], "image": np.full((600, 800, 3), 8, np.uint8),
            "extent_um": extent, "mosaic_scale": 0.5, "derived_scale": 0.5,
            "scale_warning": "", "shift_um": (30.0, -20.0), "has_shift": True,
            "um_per_px": 2.0, "objective": "10x",
            "well_center_um": (1800.0, 2600.0), "well_radius_um": 700.0,
        }
        page._mosaic_context_for_channel = lambda channel=None: ctx
        page._survey.set_mosaic_context(ctx)
        return page, ctx

    def test_empty_mosaic_click_adds_a_default_sized_spheroid(self):
        page, _ctx = self._with_mosaic()
        page._diameter.setValue(240.0)
        page._on_mosaic_empty_click(400.0, 300.0)
        dets = page._survey.detections()
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0].diameter_um, 240.0, places=6)
        self.assertEqual(dets[0].source, "user")

    def test_circles_are_rebuilt_from_the_detections(self):
        page, _ctx = self._with_mosaic()
        page._on_mosaic_empty_click(400.0, 300.0)
        det = page._survey.detections()[0]
        self.assertIsNotNone(page._mosaic_overlay.item(det.det_id))

    def test_radius_drag_flows_through_to_the_diameter(self):
        page, _ctx = self._with_mosaic()
        page._on_mosaic_empty_click(400.0, 300.0)
        det_id = page._survey.detections()[0].det_id
        page._mosaic_overlay.radius_committed.emit(det_id, 100.0)
        # 100 px at 0.5 px/µm → Ø 400 µm.
        self.assertAlmostEqual(
            page._survey.detection(det_id).diameter_um, 400.0, places=6)

    def test_rim_fit_sizes_the_selected_spheroid(self):
        page, _ctx = self._with_mosaic()
        page._on_mosaic_empty_click(400.0, 300.0)
        det_id = page._survey.detections()[0].det_id
        page._survey.select_detection(det_id)
        page._on_mosaic_rim_fitted(420.0, 310.0, 75.0)
        det = page._survey.detection(det_id)
        self.assertAlmostEqual(det.diameter_um, 300.0, places=6)

    def test_rim_fit_adds_one_when_nothing_is_selected(self):
        page, _ctx = self._with_mosaic()
        page._on_mosaic_rim_fitted(420.0, 310.0, 75.0)
        self.assertEqual(len(page._survey.detections()), 1)

    def test_a_failing_needle_fit_colours_the_circle(self):
        from SupportClasses.PhysicalModels import NeedleSpec
        page, _ctx = self._with_mosaic()
        page._hw_config = SimpleNamespace(
            needle=NeedleSpec(gauge=22, od_um=400.0, id_um=250.0, wall_um=50.0))
        page._on_mosaic_empty_click(400.0, 300.0)
        det_id = page._survey.detections()[0].det_id
        # Ø 200 µm through a 250 µm bore fails the 1.5× rule.
        page._survey.apply_radius_px(det_id, 50.0)
        page._refresh_mosaic_circles()
        badge, _msg = page._fit_badge(200.0)
        self.assertTrue(badge)


class TestCropPath(_PageCase):
    def test_refused_without_a_camera(self):
        page, _c = self._page()
        page._on_save_training_crop(1.0, 2.0, 200.0, "S001")
        self.assertIn("No microscope camera", page._survey._status.text())

    def test_crop_refused_when_the_stage_is_elsewhere(self):
        """Guards against filing an image of somewhere else under this Ø."""
        page, _c = self._page()
        page._pending_crop = {"x_um": 999999.0, "y_um": 999999.0,
                              "diameter_um": 200.0, "det_id": "S001"}
        frame = np.full((480, 640, 3), 20, np.uint8)
        page._on_crop_captured(frame, (10000.0, 12000.0), 2.0)
        self.assertIn("not on this spheroid", page._survey._status.text())

    def test_crop_saved_when_the_stage_is_on_target(self):
        import tempfile
        import SupportClasses.SpheroidTrainingStore as sts
        prev = sts._store_singleton
        try:
            store = sts.get_store(
                os.path.join(tempfile.mkdtemp(), "spheroid_training.json"))
            page, _c = self._page()
            page._pending_crop = {"x_um": 10000.0, "y_um": 12000.0,
                                  "diameter_um": 200.0, "det_id": "S001"}
            frame = np.full((480, 640, 3), 20, np.uint8)
            page._on_crop_captured(frame, (10000.0, 12000.0), 2.0)
            self.assertEqual(store.count(), 1)
            sample = store.samples()[0]
            self.assertAlmostEqual(sample["diameter_um"], 200.0)
            self.assertAlmostEqual(sample["um_per_px"], 2.0)
            self.assertIn("Saved training crop", page._survey._status.text())
        finally:
            sts._store_singleton = prev

    def test_saved_crop_size_matches_the_padded_circle(self):
        import tempfile
        import SupportClasses.SpheroidTrainingStore as sts
        prev = sts._store_singleton
        try:
            store = sts.get_store(
                os.path.join(tempfile.mkdtemp(), "s.json"))
            page, _c = self._page()
            page._pending_crop = {"x_um": 10000.0, "y_um": 12000.0,
                                  "diameter_um": 200.0, "det_id": "S001"}
            page._on_crop_captured(
                np.full((480, 640, 3), 20, np.uint8), (10000.0, 12000.0), 2.0)
            # r = 100/2 = 50 px, pad 25% → half = 62.5 → 125-126 px square.
            w, h = store.samples()[0]["crop_wh"]
            self.assertGreaterEqual(w, 124)
            self.assertLessEqual(w, 128)
            self.assertEqual(w, h)
        finally:
            sts._store_singleton = prev


class TestForwarding(_PageCase):
    def test_calibration_reaches_the_embedded_scan_page(self):
        page, _c = self._page()
        seen = []
        page._scan_page.set_calibration_data = (
            lambda *a: seen.append(a))
        page.set_calibration_data(None, {"A1": (1.0, 2.0)}, -40.0)
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0][2], -40.0)

    def test_hardware_config_reaches_the_embedded_scan_page(self):
        page, _c = self._page()
        seen = []
        page._scan_page.set_hardware_config = lambda cfg: seen.append(cfg)
        page.set_hardware_config(None)
        self.assertEqual(len(seen), 1)

    def test_a_broken_forward_never_breaks_the_host(self):
        page, _c = self._page()
        page._scan_page.set_hardware_config = lambda cfg: 1 / 0
        page.set_hardware_config(None)      # must not raise

    def test_hide_event_does_not_EXPLICITLY_hide_the_scan_page(self):
        """v7.9: this test previously asserted the opposite, and in doing so
        pinned a real bug.

        The host used to call ``self._scan_page.hide()``. The scan page is a
        CHILD widget, so Qt already delivers it a hide event when the host hides
        — that is what stops its camera. An EXPLICIT ``hide()`` additionally sets
        the child's own hidden flag, which STICKS, so Qt never re-shows it with
        the parent and the survey tab came back permanently BLANK on the second
        visit.

        What matters is that the child stops (not visible), not that it is
        explicitly hidden — so that is what is asserted now.
        """
        from PySide6.QtGui import QHideEvent
        page, _c = self._page()
        page.hideEvent(QHideEvent())
        self.assertFalse(page._scan_page.isVisible(),
                         "the embedded scan page must not keep running")
        self.assertFalse(page._scan_page.isHidden(),
                         "an EXPLICIT hide sticks and leaves the tab blank on "
                         "the next visit")


if __name__ == "__main__":
    unittest.main()


class TestSettingsRoundTrip(_PageCase):
    """Detection parameters live on the survey panel and the picker, so they
    round-trip through the dialog's extra-state hook rather than being
    duplicated in the popout (one owner per value)."""

    def test_extra_state_captures_the_detection_band_and_mode(self):
        page, _c = self._page()
        page._survey._min_d.setValue(120.0)
        page._survey._max_d.setValue(480.0)
        page._survey._restrict_well.setChecked(False)
        page._picker.set_measure_mode(True)
        state = page._collect_extra_state()
        self.assertAlmostEqual(state["det_min_diameter"], 120.0)
        self.assertAlmostEqual(state["det_max_diameter"], 480.0)
        self.assertFalse(state["det_restrict_to_well"])
        self.assertTrue(state["measure_mode"])

    def test_extra_state_restores(self):
        page, _c = self._page()
        page._apply_extra_state({
            "det_min_diameter": 60.0, "det_max_diameter": 900.0,
            "det_restrict_to_well": False, "measure_mode": True})
        self.assertAlmostEqual(page._survey._min_d.value(), 60.0)
        self.assertAlmostEqual(page._survey._max_d.value(), 900.0)
        self.assertFalse(page._survey._restrict_well.isChecked())
        self.assertTrue(page._picker.measure_mode())

    def test_round_trip_is_lossless(self):
        page, _c = self._page()
        page._survey._min_d.setValue(95.0)
        page._picker.set_measure_mode(True)
        state = page._collect_extra_state()
        page._survey._min_d.setValue(200.0)
        page._picker.set_measure_mode(False)
        page._apply_extra_state(state)
        self.assertAlmostEqual(page._survey._min_d.value(), 95.0)
        self.assertTrue(page._picker.measure_mode())

    def test_garbage_state_is_ignored(self):
        page, _c = self._page()
        page._apply_extra_state(None)
        page._apply_extra_state("nonsense")
        page._apply_extra_state({})          # must not raise

    def test_detected_list_is_deliberately_not_persisted(self):
        """A restored coordinate from another plate would be a crash."""
        page, _c = self._page()
        state = page._collect_extra_state()
        self.assertNotIn("detections", state)
        self.assertNotIn("picks", state)

    def test_crop_knobs_exist_with_the_documented_defaults(self):
        page, _c = self._page()
        self.assertTrue(page._crop_enabled.isChecked())
        self.assertAlmostEqual(page._crop_pad.value(), 0.25)
        self.assertEqual(page._crop_settle.value(), 300)
        self.assertEqual(page._crop_fresh.value(), 3)


class TestAutoCropGating(_PageCase):
    """The automatic crop must not fire for an ordinary un-measured click, nor
    for a transferred mosaic position the stage may not be on."""

    def _spy(self, page):
        seen = []
        page._on_save_training_crop = (
            lambda x, y, d, tid: seen.append((x, y, d, tid)))
        return seen

    def test_fires_for_a_live_measured_pick(self):
        page, _c = self._page()
        seen = self._spy(page)
        page._picker.add_pick(1000.0, 2000.0, 250.0)
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0][2], 250.0)

    def test_silent_for_an_unmeasured_pick(self):
        page, _c = self._page()
        seen = self._spy(page)
        page._picker.add_pick(1000.0, 2000.0)
        self.assertEqual(seen, [])

    def test_silent_for_a_transferred_mosaic_pick(self):
        from gui.widgets.live_target_picker import PROV_MOSAIC
        page, _c = self._page()
        seen = self._spy(page)
        page._picker.add_pick(1000.0, 2000.0, 250.0, provenance=PROV_MOSAIC)
        self.assertEqual(seen, [])

    def test_silent_when_crops_are_disabled(self):
        page, _c = self._page()
        page._crop_enabled.setChecked(False)
        seen = self._spy(page)
        page._picker.add_pick(1000.0, 2000.0, 250.0)
        self.assertEqual(seen, [])

    def test_place_targets_never_trigger_a_crop(self):
        page, _c = self._page()
        seen = self._spy(page)
        page._picker.add_place(1000.0, 2000.0, 250.0)
        self.assertEqual(seen, [])


class TestStartInterlock(_PageCase):
    """Start must refuse while a mosaic scan is driving the stage, not merely be
    greyed out — a programmatic call or a race must not get through."""

    def _ready_queue(self, page):
        page._picker.add_pick(11000.0, 22000.0, 200.0)
        page._picker.add_place(11500.0, 22500.0)

    def test_start_refused_during_a_scan(self):
        page, ctrl = self._page()
        self._ready_queue(page)
        page._scan_page.is_scanning = lambda: True
        page._on_start()
        self.assertIsNone(page._exec_thread)
        self.assertIn("mosaic scan", page._status.text())

    def test_start_refused_while_already_running(self):
        page, _c = self._page()
        self._ready_queue(page)
        page._exec_thread = SimpleNamespace(is_alive=lambda: True)
        page._on_start()
        self.assertIn("Busy running", page._status.text())

    def test_start_still_reaches_its_own_gates_when_idle(self):
        """Not blocked by the interlock — it should fall through to the real
        preconditions (here: no plate-bottom calibration)."""
        page, ctrl = self._page()
        self._ready_queue(page)
        ctrl.print_height_to_zref = lambda mm: None
        page._z_references["plate_bottom_z"] = None
        page._on_start()
        self.assertIn("Plate bottom Z", page._status.text())
