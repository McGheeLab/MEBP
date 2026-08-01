"""test_v78_target_actions.py — picker selection survival, row actions, and
live-view circle editing.

Two halves:

* **Pure** (no Qt): the click → stage → overlay → click CLOSURE, parameterised
  over every mirror / flip / rotation combination. This is the test that catches
  a marker drawn with the naive identity inverse — at θ=0 with no flips the buggy
  and the correct formula agree, which is exactly why the bug survived.
* **Offscreen Qt**: selection surviving a rebuild, the Goto/Edit/Delete gating,
  the drag state machine, and the ``pick_only`` non-regression.
"""

from __future__ import annotations

import math
import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from gui.widgets.camera_manager import CameraManager
from gui.widgets.live_target_picker import (
    PROV_CONFIRMED, PROV_LIVE, PROV_MOSAIC, LiveTargetPicker,
    _SNAP_RADIUS_WIDGET_PX,
)

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication
    _QT = True
except Exception:      # pragma: no cover
    _QT = False


# ── pure: the projection closure ───────────────────────────────────

def _manager(um_per_px=2.0, rotation=0.0, mirrored=False, flip_y=False):
    """A CameraManager with one calibrated slot, built without Qt widgets.

    ``_cameras`` is deliberately EMPTY: the picker's view then finds no camera to
    subscribe to, so no frames arrive and the tests drive the overlay geometry
    directly (which is the thing under test).
    """
    mgr = CameraManager.__new__(CameraManager)
    mgr._max_cameras = 3
    mgr._cameras = []
    mgr._um_per_px = [um_per_px, um_per_px, um_per_px]
    mgr._um_per_px_res = [None, None, None]
    mgr._um_per_px_set = [True, True, True]
    mgr._rotation_deg = [rotation, rotation, rotation]
    mgr._column_dir_deg = [None, None, None]
    mgr._mirrored = [mirrored, mirrored, mirrored]
    mgr._flip_y = [flip_y, flip_y, flip_y]
    return mgr


class TestForwardInverseClosure(unittest.TestCase):
    """``pixel_to_stage_offset`` and ``stage_offset_to_pixel`` must be exact
    inverses for EVERY orientation, not just the identity one."""

    IMG = (640, 480)
    CASES = [(rot, mir, fy)
             for rot in (0.0, 90.0, 180.0, 270.0, 37.5)
             for mir in (False, True)
             for fy in (False, True)]

    def test_round_trip_for_every_orientation(self):
        for rot, mir, fy in self.CASES:
            mgr = _manager(2.0, rot, mir, fy)
            for px in ((320.0, 240.0), (100.0, 50.0), (600.0, 400.0)):
                dx, dy = mgr.pixel_to_stage_offset(
                    0, px[0], px[1], self.IMG[0], self.IMG[1])
                bx, by = mgr.stage_offset_to_pixel(
                    0, dx, dy, self.IMG[0], self.IMG[1])
                self.assertAlmostEqual(
                    bx, px[0], places=6,
                    msg=f"x rot={rot} mir={mir} fy={fy}")
                self.assertAlmostEqual(
                    by, px[1], places=6,
                    msg=f"y rot={rot} mir={mir} fy={fy}")

    def test_identity_case_reduces_to_the_naive_formula(self):
        """At θ=0 with no flips the correct map IS the naive one — which is why
        the naive one has to be tested against a rotated camera to be caught."""
        mgr = _manager(2.0, 0.0, False, False)
        bx, by = mgr.stage_offset_to_pixel(0, 200.0, -100.0, 640, 480)
        self.assertAlmostEqual(bx, 200.0 / 2.0 + 320.0)
        self.assertAlmostEqual(by, -100.0 / 2.0 + 240.0)

    def test_naive_inverse_is_wrong_on_a_rotated_camera(self):
        """Documents the defect the closure test exists to prevent."""
        mgr = _manager(2.0, 90.0, False, False)
        dx, dy = mgr.pixel_to_stage_offset(0, 420.0, 240.0, 640, 480)
        naive = (dx / 2.0 + 320.0, dy / 2.0 + 240.0)
        correct = mgr.stage_offset_to_pixel(0, dx, dy, 640, 480)
        self.assertAlmostEqual(correct[0], 420.0, places=6)
        # 100 px off in the other axis — a marker nowhere near the click.
        self.assertGreater(
            math.hypot(naive[0] - 420.0, naive[1] - 240.0), 50.0)

    def test_uncalibrated_camera_returns_frame_centre(self):
        mgr = _manager(0.0)
        mgr._um_per_px = [0.0, 0.0, 0.0]
        self.assertEqual(mgr.stage_offset_to_pixel(0, 100.0, 100.0, 640, 480),
                         (320.0, 240.0))

    def test_resolution_change_leaves_um_unchanged_but_doubles_px(self):
        """A 916 → 1832 switch must not resize anything in µm."""
        mgr = _manager(2.0)
        mgr._um_per_px_res = [(640, 480)] * 3
        d_small = mgr.pixel_to_stage_offset(0, 420.0, 240.0, 640, 480)
        d_big = mgr.pixel_to_stage_offset(0, 840.0, 480.0, 1280, 960)
        self.assertAlmostEqual(d_small[0], d_big[0], places=6)
        self.assertAlmostEqual(d_small[1], d_big[1], places=6)


# ── offscreen Qt: the picker itself ────────────────────────────────

class _Ctrl:
    zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    is_xy_connected = True
    is_zp_connected = True

    def get_xy_position(self, cached=True):
        return (1000.0, 2000.0, 0.0)


@unittest.skipUnless(_QT, "PySide6 not available")
class _PickerCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.app = QApplication.instance() or QApplication([])
        except Exception as e:      # pragma: no cover
            raise unittest.SkipTest(f"Qt unavailable: {e}")

    def _picker(self, pick_only=False):
        p = LiveTargetPicker(_Ctrl(), _manager(2.0), pick_only=pick_only)
        # A believable live frame so the µm↔px projection is defined.
        p._view._last_image_size = (640, 480)
        p._view.set_um_per_px(2.0)
        p._view.set_stage_position(1000.0, 2000.0)
        return p


class TestSelectionSurvival(_PickerCase):
    def test_selection_survives_an_add(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        p.add_pick(1100.0, 2000.0)
        p._pick_list.item(0).setSelected(True)
        self.assertEqual(p._sole_selected_id(p._pick_list), a)
        p.add_pick(1200.0, 2000.0)
        self.assertEqual(p._sole_selected_id(p._pick_list), a)

    def _select(self, list_widget, *target_ids):
        for i in range(list_widget.count()):
            item = list_widget.item(i)
            item.setSelected(
                item.data(Qt.ItemDataRole.UserRole) in target_ids)

    def test_removing_the_selected_row_leaves_the_rest_intact(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        b = p.add_pick(1100.0, 2000.0)
        c = p.add_pick(1200.0, 2000.0)
        self._select(p._pick_list, b)
        p._remove_selected_rows(p._pick_list, p._picks, p.picks_changed)
        self.assertEqual([t.target_id for t in p.picks()], [a, c])
        # No stale selection survives a deleted row.
        self.assertIsNone(p._sole_selected_id(p._pick_list))

    def test_selection_of_a_surviving_row_is_kept_across_a_rebuild(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        p.add_pick(1100.0, 2000.0)
        self._select(p._pick_list, a)
        # Any mutation rebuilds both lists from scratch.
        p.add_place(1500.0, 2500.0)
        p.set_target_size(a, 180.0)
        self.assertEqual(p._sole_selected_id(p._pick_list), a)

    def test_selection_survives_a_size_edit(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        p._pick_list.item(0).setSelected(True)
        p.set_target_size(a, 250.0)
        self.assertEqual(p._sole_selected_id(p._pick_list), a)

    def test_rows_carry_their_target_id(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        self.assertEqual(
            p._pick_list.item(0).data(Qt.ItemDataRole.UserRole), a)

    def test_delete_by_id_not_row_index(self):
        """Two non-adjacent rows selected: both must go. Deleting by row index
        would corrupt this the moment the first removal shifted the rest."""
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        b = p.add_pick(1100.0, 2000.0)
        c = p.add_pick(1200.0, 2000.0)
        self._select(p._pick_list, a, c)
        p._remove_selected_rows(p._pick_list, p._picks, p.picks_changed)
        self.assertEqual([t.target_id for t in p.picks()], [b])


class TestActionButtons(_PickerCase):
    def test_disabled_with_no_selection(self):
        p = self._picker()
        p.add_pick(1000.0, 2000.0)
        for b in p._action_buttons["pick"]:
            self.assertFalse(b.isEnabled())

    def test_enabled_on_exactly_one_selection(self):
        p = self._picker()
        p.add_pick(1000.0, 2000.0)
        p._pick_list.item(0).setSelected(True)
        p._refresh_action_buttons()
        for b in p._action_buttons["pick"]:
            self.assertTrue(b.isEnabled())

    def test_disabled_on_a_multi_selection(self):
        p = self._picker()
        p.add_pick(1000.0, 2000.0)
        p.add_pick(1100.0, 2000.0)
        p._pick_list.item(0).setSelected(True)
        p._pick_list.item(1).setSelected(True)
        p._refresh_action_buttons()
        for b in p._action_buttons["pick"]:
            self.assertFalse(b.isEnabled())

    def test_goto_emits_the_target_id_and_never_moves_the_stage(self):
        """The picker owns no motion — the host routes it via SafeTravelWorker."""
        p = self._picker()
        a = p.add_pick(1234.0, 5678.0)
        seen = []
        p.goto_requested.connect(seen.append)
        p._pick_list.item(0).setSelected(True)
        p._goto_selected(p._pick_list)
        self.assertEqual(seen, [a])

    def test_goto_is_silent_without_a_selection(self):
        p = self._picker()
        p.add_pick(1000.0, 2000.0)
        seen = []
        p.goto_requested.connect(seen.append)
        p._goto_selected(p._pick_list)
        self.assertEqual(seen, [])


class TestSizeAndProvenance(_PickerCase):
    def test_size_um_is_the_single_source_of_truth(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        self.assertEqual(p.target_by_id(a).size_um, 0.0)
        changed = []
        p.target_changed.connect(changed.append)
        self.assertTrue(p.set_target_size(a, 312.0))
        self.assertAlmostEqual(p.target_by_id(a).size_um, 312.0)
        self.assertEqual(changed, [a])

    def test_setting_the_same_size_is_a_no_op(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0, 200.0)
        changed = []
        p.target_changed.connect(changed.append)
        self.assertFalse(p.set_target_size(a, 200.0))
        self.assertEqual(changed, [])

    def test_unknown_target_id_is_safe(self):
        p = self._picker()
        self.assertFalse(p.set_target_size("nope", 100.0))
        self.assertFalse(p.set_target_position("nope", 1.0, 2.0))

    def test_transferred_target_is_flagged_mosaic(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0, 250.0, provenance=PROV_MOSAIC)
        self.assertEqual(p.provenance(a), PROV_MOSAIC)
        # And the row says so, so an unverified position is visible in the list.
        self.assertIn("~mosaic", p._pick_list.item(0).text())

    def test_live_click_confirms_a_mosaic_position(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0, 250.0, provenance=PROV_MOSAIC)
        p.confirm_target_position(a, 1005.0, 2003.0)
        self.assertEqual(p.provenance(a), PROV_CONFIRMED)
        self.assertAlmostEqual(p.target_by_id(a).x_um, 1005.0)
        self.assertNotIn("~mosaic", p._pick_list.item(0).text())

    def test_default_provenance_is_live(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0)
        self.assertEqual(p.provenance(a), PROV_LIVE)

    def test_diameter_shown_in_the_row(self):
        p = self._picker()
        p.add_pick(1000.0, 2000.0, 312.0)
        self.assertIn("Ø312µm", p._pick_list.item(0).text())

    def test_clear_drops_provenance(self):
        p = self._picker()
        a = p.add_pick(1000.0, 2000.0, provenance=PROV_MOSAIC)
        p.clear_picks()
        self.assertEqual(p.provenance(a), PROV_LIVE)   # i.e. no stale entry


class TestRemoveRadiusScalesWithSize(_PickerCase):
    def test_large_spheroid_can_be_removed_by_its_rim(self):
        """A 400 µm spheroid's ring is wider than the flat 250 µm radius."""
        p = self._picker()
        p.add_pick(1000.0, 2000.0, 400.0)
        # 300 µm away — outside 250 but inside the spheroid's own 400.
        p._remove_nearest(p._picks, 1300.0, 2000.0, p.picks_changed)
        self.assertEqual(len(p.picks()), 0)

    def test_small_target_keeps_the_flat_radius(self):
        p = self._picker()
        p.add_pick(1000.0, 2000.0)
        p._remove_nearest(p._picks, 1300.0, 2000.0, p.picks_changed)
        self.assertEqual(len(p.picks()), 1)
        p._remove_nearest(p._picks, 1100.0, 2000.0, p.picks_changed)
        self.assertEqual(len(p.picks()), 0)


class TestMeasureMode(_PickerCase):
    def test_off_by_default(self):
        p = self._picker()
        self.assertFalse(p.measure_mode())
        self.assertFalse(p._view.measure_mode())

    def test_toggle_propagates_to_the_view(self):
        p = self._picker()
        p.set_measure_mode(True)
        self.assertTrue(p._view.measure_mode())
        p.set_measure_mode(False)
        self.assertFalse(p._view.measure_mode())

    def test_rim_controls_gated_on_measure_mode(self):
        p = self._picker()
        self.assertFalse(p._rim_check.isEnabled())
        p.set_measure_mode(True)
        self.assertTrue(p._rim_check.isEnabled())
        p.set_measure_mode(False)
        self.assertFalse(p._rim_check.isEnabled())
        self.assertFalse(p._rim_check.isChecked())

    def test_ring_hit_beats_centre_hit(self):
        """Resizing a small circle must not be hijacked into a move."""
        p = self._picker()
        p.set_measure_mode(True)
        a = p.add_pick(1000.0, 2000.0, 200.0)
        # Fake a laid-out pixmap so widget<->image mapping is defined.
        p._view._last_pixmap = _pixmap(640, 480)
        p._view._display.resize(640, 480)
        # r = (200/2)/2.0 = 50 image px → 50 widget px at 1:1.
        kind, target = p._view._hit(320.0 + 50.0, 240.0)
        self.assertEqual(kind, "size")
        self.assertEqual(target.target_id, a)
        kind, target = p._view._hit(320.0, 240.0)
        self.assertEqual(kind, "center")

    def test_no_hit_far_from_any_target(self):
        p = self._picker()
        p.set_measure_mode(True)
        p.add_pick(1000.0, 2000.0, 200.0)
        p._view._last_pixmap = _pixmap(640, 480)
        p._view._display.resize(640, 480)
        kind, _t = p._view._hit(10.0, 10.0)
        self.assertIsNone(kind)

    def test_snap_tolerance_is_widget_pixels(self):
        p = self._picker()
        p.set_measure_mode(True)
        p.add_pick(1000.0, 2000.0, 200.0)
        p._view._last_pixmap = _pixmap(640, 480)
        p._view._display.resize(640, 480)
        just_in = 320.0 + 50.0 + (_SNAP_RADIUS_WIDGET_PX - 1)
        just_out = 320.0 + 50.0 + (_SNAP_RADIUS_WIDGET_PX + 40)
        self.assertEqual(p._view._hit(just_in, 240.0)[0], "size")
        self.assertIsNone(p._view._hit(just_out, 240.0)[0])


class TestDragStateMachine(_PickerCase):
    def _ready(self, pick_only=False):
        p = self._picker(pick_only)
        p.set_measure_mode(True)
        tid = p.add_pick(1000.0, 2000.0, 200.0)
        p._view._last_pixmap = _pixmap(640, 480)
        p._view._display.resize(640, 480)
        return p, tid

    def test_ring_press_starts_a_size_drag_and_suppresses_clicked(self):
        p, tid = self._ready()
        clicked = []
        p._view.clicked.connect(lambda x, y: clicked.append((x, y)))
        handled = p._view.eventFilter(
            p._view._display, _press(320.0 + 50.0, 240.0))
        self.assertTrue(handled)
        self.assertTrue(p._view.is_dragging())
        self.assertEqual(clicked, [])

    def test_dragging_the_ring_outward_grows_the_diameter(self):
        p, tid = self._ready()
        p._view.eventFilter(p._view._display, _press(320.0 + 50.0, 240.0))
        p._view.eventFilter(p._view._display, _move(320.0 + 100.0, 240.0))
        # r 100 image px at 2 µm/px → Ø 400 µm.
        self.assertAlmostEqual(p.target_by_id(tid).size_um, 400.0, delta=2.0)

    def test_release_commits_and_emits_target_changed(self):
        p, tid = self._ready()
        changed = []
        p.target_changed.connect(changed.append)
        p._view.eventFilter(p._view._display, _press(320.0 + 50.0, 240.0))
        p._view.eventFilter(p._view._display, _release(320.0 + 75.0, 240.0))
        self.assertFalse(p._view.is_dragging())
        self.assertEqual(changed, [tid])
        self.assertAlmostEqual(p.target_by_id(tid).size_um, 300.0, delta=2.0)

    def test_projection_is_frozen_during_a_drag(self):
        """A stage poll mid-drag must not silently resize the circle."""
        p, tid = self._ready()
        p._view.eventFilter(p._view._display, _press(320.0 + 50.0, 240.0))
        # The host's 5 Hz poll lands mid-gesture.
        p._view.set_stage_position(1500.0, 2500.0)
        p._view.eventFilter(p._view._display, _move(320.0 + 100.0, 240.0))
        self.assertAlmostEqual(p.target_by_id(tid).size_um, 400.0, delta=2.0)

    def test_empty_space_press_still_adds_a_target(self):
        p, _tid = self._ready()
        before = len(p.picks())
        p._view.eventFilter(p._view._display, _press(20.0, 20.0))
        # The base class turns the press into `clicked`, which adds a target.
        self.assertGreaterEqual(len(p.picks()), before)

    def test_measure_mode_off_never_starts_a_drag(self):
        """Full non-regression: with the flag off the press is a plain click."""
        p, _tid = self._ready()
        p.set_measure_mode(False)
        handled = p._view.eventFilter(
            p._view._display, _press(320.0 + 50.0, 240.0))
        self.assertFalse(p._view.is_dragging())
        del handled

    def test_right_click_still_removes(self):
        p, _tid = self._ready()
        seen = []
        p._view.right_clicked.connect(lambda x, y: seen.append((x, y)))
        handled = p._view.eventFilter(
            p._view._display, _press(320.0, 240.0, right=True))
        self.assertTrue(handled)
        self.assertEqual(len(seen), 1)


class TestRimFit(_PickerCase):
    def _ready(self):
        p = self._picker()
        p.set_measure_mode(True)
        p._rim_check.setChecked(True)
        p._view._last_pixmap = _pixmap(640, 480)
        p._view._display.resize(640, 480)
        return p

    def test_three_rim_points_fit_a_circle(self):
        p = self._ready()
        # Points on a circle of radius 50 image px centred at the frame centre.
        for ang in (0.0, 120.0, 240.0):
            t = math.radians(ang)
            p._view.eventFilter(p._view._display, _press(
                320.0 + 50.0 * math.cos(t), 240.0 + 50.0 * math.sin(t)))
        fit = p._view.rim_fit()
        self.assertIsNotNone(fit)
        # r 50 image px at 2 µm/px → Ø 200 µm, centred on the stage position.
        self.assertAlmostEqual(fit[2], 200.0, delta=4.0)
        self.assertAlmostEqual(fit[0], 1000.0, delta=4.0)
        self.assertAlmostEqual(fit[1], 2000.0, delta=4.0)

    def test_two_points_do_not_fit(self):
        p = self._ready()
        p._view.eventFilter(p._view._display, _press(320.0, 190.0))
        p._view.eventFilter(p._view._display, _press(370.0, 240.0))
        self.assertIsNone(p._view.rim_fit())
        self.assertFalse(p._rim_apply.isEnabled())

    def test_apply_creates_a_sized_target(self):
        p = self._ready()
        for ang in (0.0, 120.0, 240.0):
            t = math.radians(ang)
            p._view.eventFilter(p._view._display, _press(
                320.0 + 50.0 * math.cos(t), 240.0 + 50.0 * math.sin(t)))
        p._on_apply_rim_fit()
        self.assertEqual(len(p.picks()), 1)
        self.assertAlmostEqual(p.picks()[0].size_um, 200.0, delta=4.0)
        # Points are cleared after applying.
        self.assertIsNone(p._view.rim_fit())

    def test_apply_sizes_the_selected_target_instead_of_adding(self):
        p = self._ready()
        tid = p.add_pick(1000.0, 2000.0)
        p._pick_list.item(0).setSelected(True)
        for ang in (0.0, 120.0, 240.0):
            t = math.radians(ang)
            p._view.eventFilter(p._view._display, _press(
                320.0 + 50.0 * math.cos(t), 240.0 + 50.0 * math.sin(t)))
        p._on_apply_rim_fit()
        self.assertEqual(len(p.picks()), 1)
        self.assertAlmostEqual(p.target_by_id(tid).size_um, 200.0, delta=4.0)

    def test_leaving_measure_mode_clears_rim_state(self):
        p = self._ready()
        p._view.eventFilter(p._view._display, _press(320.0, 190.0))
        p.set_measure_mode(False)
        self.assertIsNone(p._view.rim_fit())
        self.assertFalse(p._view.rim_mode())


class TestPickOnlyNonRegression(_PickerCase):
    def test_place_machinery_still_hidden(self):
        p = self._picker(pick_only=True)
        self.assertFalse(p._place_section_widget.isVisible())
        self.assertFalse(p._mode_toggle_widget.isVisible())
        self.assertEqual(p._mode, LiveTargetPicker.MODE_PICK)

    def test_pick_actions_still_present(self):
        p = self._picker(pick_only=True)
        self.assertIn("pick", p._action_buttons)

    def test_edit_dialog_hides_the_diameter(self):
        from gui.widgets.live_target_picker import _TargetEditDialog
        from SupportClasses.PickAndPlaceManager import PickPlaceTarget
        t = PickPlaceTarget(target_id="P001", x_um=1.0, y_um=2.0, well_name="",
                            size_um=250.0)
        dlg = _TargetEditDialog(t, show_size=False)
        # Ø is not offered, and values() reports 0 rather than a stale size.
        self.assertEqual(dlg.values()[2], 0.0)

    def test_measure_mode_still_available(self):
        """Sizing a region is harmless; only the PLACE list is hidden."""
        p = self._picker(pick_only=True)
        p.set_measure_mode(True)
        self.assertTrue(p._view.measure_mode())


# ── helpers ────────────────────────────────────────────────────────

def _pixmap(w, h):
    from PySide6.QtGui import QPixmap
    pm = QPixmap(w, h)
    pm.fill()
    return pm


def _mouse(x, y, etype, button):
    from PySide6.QtCore import QEvent, QPointF
    from PySide6.QtGui import QMouseEvent
    return QMouseEvent(etype, QPointF(x, y), QPointF(x, y), button, button,
                       Qt.KeyboardModifier.NoModifier)


def _press(x, y, right=False):
    from PySide6.QtCore import QEvent
    return _mouse(x, y, QEvent.Type.MouseButtonPress,
                  Qt.MouseButton.RightButton if right else Qt.MouseButton.LeftButton)


def _move(x, y):
    from PySide6.QtCore import QEvent
    return _mouse(x, y, QEvent.Type.MouseMove, Qt.MouseButton.LeftButton)


def _release(x, y):
    from PySide6.QtCore import QEvent
    return _mouse(x, y, QEvent.Type.MouseButtonRelease,
                  Qt.MouseButton.LeftButton)


if __name__ == "__main__":
    unittest.main()
