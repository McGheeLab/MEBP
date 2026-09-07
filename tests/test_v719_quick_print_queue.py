"""test_v719_quick_print_queue.py — Quick Print: a plate-wide queue of
INDEPENDENT prints, per-well toolpath glyphs, and the run-all engine.

Three surfaces, three failure modes worth pinning:

* the NAVIGATOR gains per-well glyphs and opt-in multi-selection, and must be
  PIXEL-IDENTICAL with neither in use — the Fluorescence Mosaic page shares it;
* the PAGE resolves a snapshot that is not on screen by transiently applying it,
  which is only safe if the restore is guaranteed and nothing re-enters the event
  loop while it is applied;
* the WORKER must prep once, swap only on an ink change, clean once, and always
  end at safe Z — including when it is aborted while HELD.

Driven against fakes — no hardware.
"""

import os
import sys
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, patch

os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = tempfile.mkdtemp(
    prefix="mebp_qpq_settings_")
os.environ.setdefault("MEBP_UI_SCALE", "1.0")

from PySide6.QtCore import Qt, QEvent, QPointF
from PySide6.QtGui import QMouseEvent, QPixmap
from PySide6.QtWidgets import QApplication, QListWidget, QMessageBox, QSplitter

from SupportClasses.PhysicalModels import InkSpec, NeedleSpec
from SupportClasses.HardwareConfig import (
    HardwareConfig, PumpChannelConfig, SyringeSpec,
)
from SupportClasses.PrintQueue import QueuedPrint
from SupportClasses.SketchTrajectory import Sketch, SketchInk, SketchShape
from SupportClasses.WellPlate import WellPlate

_QP = "gui.pages.workflows.quick_print_workflow"

_WELLS = {
    "A1": (10000.0, 12000.0), "A2": (20000.0, 12000.0),
    "A3": (30000.0, 12000.0), "A4": (40000.0, 12000.0),
    "B1": (10000.0, 22000.0), "B2": (20000.0, 22000.0),
    "B3": (30000.0, 22000.0), "B4": (40000.0, 22000.0),
    "C1": (10000.0, 32000.0), "C2": (20000.0, 32000.0),
}


def _make_hw():
    """Two printable inks on two enabled pumps + the four service reagents."""
    cfg = HardwareConfig()
    cfg.needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
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
    cfg.assign_wells_to_ink("Alginate", ["A3"])
    cfg.assign_wells_to_ink("Gelatin", ["A4"])
    cfg.assign_wells_to_ink("Waste bin", ["B1"])
    cfg.assign_wells_to_ink("Mineral oil", ["B2"])
    cfg.assign_wells_to_ink("PBS wash", ["B3"])
    cfg.assign_wells_to_ink("Sep buffer", ["B4"])
    return cfg


# ════════════════════════════════════════════════════════════════════
#  Fakes (module-patched) + a shared ordered call log
# ════════════════════════════════════════════════════════════════════

CALLS: list = []


class _FakeExecutor:
    instances: list = []

    def __init__(self, controller, hw_config=None):
        self.controller = controller
        self._abort_flag = threading.Event()
        self.on_sub_step = None
        self.prep_bore = None
        _FakeExecutor.instances.append(self)

    def __setattr__(self, k, v):
        object.__setattr__(self, k, v)

    def run_prep(self):
        CALLS.append(("prep",))

    def run_post_clean(self):
        CALLS.append(("swap", self.prep_bore))

    def aspirate_ink(self, well_pos, volume_uL, *, bore, z_mm, **kw):
        CALLS.append(("aspirate", bore, round(float(volume_uL), 4)))

    def run_print_cleanup(self):
        CALLS.append(("cleanup",))

    def _retract_to_safe_z(self):
        CALLS.append(("retract",))


class _FakeLogger:
    def __init__(self, name):
        self.path = f"/tmp/{name}.jsonl"


class _FakePM:
    """Completes immediately. `abort_at` (a job-name substring) makes the job
    report ABORTED instead, so the worker's abort path can be exercised."""
    instances: list = []
    abort_at = None

    def __init__(self, controller):
        self.controller = controller
        self.on_progress = None
        self.on_state_changed = None
        self.on_vel_sample = None
        self.state = None
        self.exec_logger = None
        _FakePM.instances.append(self)

    def load_job(self, job):
        self.job = job

    def abort(self):
        CALLS.append(("pm_abort",))

    def start(self):
        from SupportClasses.PrintManager import PrintState
        name = (self.job or {}).get("job_name", "")
        CALLS.append(("job", name))
        self.exec_logger = _FakeLogger(name.replace(" ", "_")[:40])
        st = (PrintState.ABORTED
              if _FakePM.abort_at and _FakePM.abort_at in name
              else PrintState.COMPLETED)
        self.state = st
        if self.on_state_changed:
            self.on_state_changed(st)


def _fake_build_job(**kw):
    return dict(kw)


def _patches():
    return (patch(f"{_QP}.PickPlaceExecutor", _FakeExecutor),
            patch(f"{_QP}.PrintManager", _FakePM),
            patch(f"{_QP}.build_well_plate_job", _fake_build_job),
            patch.object(QMessageBox, "question",
                         staticmethod(
                             lambda *a, **k: QMessageBox.StandardButton.Yes)),
            patch.object(QMessageBox, "warning",
                         staticmethod(
                             lambda *a, **k: QMessageBox.StandardButton.Ok)))


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        CALLS.clear()
        _FakeExecutor.instances.clear()
        _FakePM.instances.clear()
        _FakePM.abort_at = None

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.print_floor_violation.return_value = False
        # Must VARY with the height, or "each unit carries its own settings"
        # cannot be observed at all.
        ctrl.print_height_to_zref.side_effect = (
            lambda h, *a, **k: -25.0 - float(h))
        ctrl.print_z_dir.return_value = -1.0
        ctrl.plate_axis_sign.return_value = (1, 1)
        ctrl.plate_flip_180.return_value = False
        ctrl.safe_travel_to.return_value = True
        ctrl.is_position_poller_suspended.return_value = False
        ctrl.simulate_pump_budget.return_value = {
            "ok": True, "span_uL": 12.0, "capacity_uL": 250.0, "reason": "ok"}
        ctrl._pending_per_axis_max_feedrate = None
        from SupportClasses.StageController import StageController as _SC
        ctrl.get_max_xy_speed_um_s.side_effect = (
            lambda: _SC.get_max_xy_speed_um_s(ctrl))
        ctrl.get_max_z_feedrate_mm_min.side_effect = (
            lambda: _SC.get_max_z_feedrate_mm_min(ctrl))
        return ctrl

    def _page(self, ink="Alginate"):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        page = QuickPrintWorkflowPage(self._ctrl(), settings=None)
        page.set_hardware_config(_make_hw())
        page.set_calibration_data(WellPlate.from_format(96), _WELLS, -35.0)
        page._selected_well = "A1"
        if ink:
            page._set_combo_data(page._ink_combo, ink)
        return page

    def _queue(self, page, spec):
        """spec = [(well, ink, print_z)] → the page's queue."""
        page._queue = []
        for well, ink, pz in spec:
            page._set_combo_data(page._ink_combo, ink)
            qp = page._snapshot_from_widgets(well)
            qp.print_z_mm = pz
            page._queue = page._queue + [qp]
        page._refresh_queue_ui()
        return page._queue


# ════════════════════════════════════════════════════════════════════
#  1. The navigator
# ════════════════════════════════════════════════════════════════════

class TestNavigatorIsUnchangedWhenUnused(_QtBase):
    """The Fluorescence Mosaic page shares this widget and never opts in."""

    def _nav(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        n = WellPlateNavigator()
        n.set_plate(WellPlate.from_format(24))
        return n

    @staticmethod
    def _render(nav):
        """Returns the raw bytes, not the QImage: a QImage from a local QPixmap
        can outlive the buffer it shares, which makes the comparison meaningless."""
        nav.resize(400, 300)
        pm = QPixmap(nav.size())
        pm.fill()
        nav.render(pm)
        return pm.toImage().copy().constBits().tobytes()

    def test_pixel_identical_with_no_overlays_and_no_multi_select(self):
        nav = self._nav()
        # Warm the font cache first: the row/column headers use QFont("Consolas"),
        # and the FIRST paint in a process can fall back to a different face while
        # it loads, so an unwarmed baseline differs from every later render for
        # reasons that have nothing to do with this widget. (Only showed up when
        # another suite ran first.)
        self._render(nav)
        base = self._render(nav)
        nav.set_well_paths(None)
        nav.set_multi_select_enabled(False)
        nav.set_selected_wells([])
        self.assertEqual(base, self._render(nav))
        # Guard the guard: this comparison must be capable of failing.
        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, 2.0)]], None, True)})
        self.assertNotEqual(base, self._render(nav))

    def test_multi_select_is_off_by_default(self):
        self.assertFalse(self._nav().multi_select_enabled())

    def test_a_plain_click_still_emits_well_clicked(self):
        nav = self._nav()
        self._render(nav)
        seen = []
        nav.well_clicked.connect(seen.append)
        cx, cy = nav._well_center(nav._well_layout(), "A1")
        nav.mousePressEvent(QMouseEvent(
            QEvent.Type.MouseButtonPress, QPointF(cx, cy),
            Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier))
        self.assertEqual(seen, ["A1"])
        self.assertEqual(nav.selected_wells(), [])

    def test_the_raster_grid_preview_still_paints(self):
        """Regression for the Fluorescence Mosaic page's only overlay."""
        nav = self._nav()
        nav.set_current_well("A1")
        plain = self._render(nav)
        nav.set_raster_grid(4, 4)
        self.assertNotEqual(plain, self._render(nav))


class TestNavigatorGlyphs(_QtBase):
    def _nav(self, fmt=24):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        n = WellPlateNavigator()
        n.set_plate(WellPlate.from_format(fmt))
        n.resize(400, 300)
        pm = QPixmap(n.size())
        pm.fill()
        n.render(pm)
        return n

    @staticmethod
    def _render(nav):
        """`.copy()` is load-bearing: a QImage taken from a local QPixmap shares
        that pixmap's buffer, which is freed when it goes out of scope, so
        comparing two of them compares freed memory. A clip-removal mutation
        SURVIVED until this was fixed."""
        pm = QPixmap(nav.size())
        pm.fill()
        nav.render(pm)
        return pm.toImage().copy()

    @staticmethod
    def _tealish(px):
        r, g, b = (px >> 16) & 255, (px >> 8) & 255, px & 255
        return g > 150 and b > 140 and g > r + 40

    def test_the_glyph_is_not_y_flipped(self):
        """Plate-local is Y-DOWN (PlateTransform.to_px). A flip here mirrors every
        well's print. Checked in BOTH directions so the probe cannot pass by
        accident."""
        nav = self._nav()
        lay = nav._well_layout()
        cx, cy = nav._well_center(lay, "A1")
        r = nav._well_radius(lay, lay["wells"][0][3])
        rows = range(int(cy - r) - 2, int(cy + r) + 2)

        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, 3.0)]], None, True)})
        down = self._render(nav)
        hit = [y for y in rows if self._tealish(down.pixel(int(cx), y))]
        self.assertTrue(hit, "no glyph painted at all")
        self.assertGreater(len([y for y in hit if y > cy]),
                           len([y for y in hit if y < cy]),
                           "+y must paint BELOW the well centre")

        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, -3.0)]], None, True)})
        up = self._render(nav)
        hit_up = [y for y in rows if self._tealish(up.pixel(int(cx), y))]
        self.assertGreater(len([y for y in hit_up if y < cy]),
                           len([y for y in hit_up if y > cy]),
                           "the -y mirror did not land above centre — the probe "
                           "is not measuring the glyph")

    def test_the_glyph_is_clipped_to_its_own_well(self):
        """A HORIZONTAL line, so it provably crosses A2's centre row. (A diagonal
        one misses that row by a whole well pitch, which is why a clip-removal
        mutation SURVIVED the first version of this test.)"""
        nav = self._nav()
        before = self._render(nav)
        nav.set_well_paths(
            {"A1": ([[(-100.0, 0.0), (100.0, 0.0)]], None, True)})
        after = self._render(nav)
        lay = nav._well_layout()
        a1x, a1y = nav._well_center(lay, "A1")
        a2x, a2y = nav._well_center(lay, "A2")
        self.assertAlmostEqual(a1y, a2y, delta=1.0,
                               msg="A1 and A2 must share a row for this probe")
        r = nav._well_radius(lay, lay["wells"][0][3])
        # Guard the guard: the line must actually be drawn inside A1.
        self.assertTrue(
            any(before.pixel(int(a1x + dx), int(a1y))
                != after.pixel(int(a1x + dx), int(a1y))
                for dx in range(-int(r) + 1, int(r))),
            "nothing was drawn in A1 at all — the probe proves nothing")
        changed = sum(1 for dx in range(-int(r) + 1, int(r))
                      if before.pixel(int(a2x + dx), int(a2y))
                      != after.pixel(int(a2x + dx), int(a2y)))
        self.assertEqual(changed, 0,
                         "an oversized path spilled into a neighbouring well")

    def test_a_spilling_glyph_rings_its_well(self):
        nav = self._nav()
        nav.set_well_paths({"A1": ([[(-50.0, 0.0), (50.0, 0.0)]], None, True)})
        img = self._render(nav)
        lay = nav._well_layout()
        cx, cy = nav._well_center(lay, "A1")
        r = nav._well_radius(lay, lay["wells"][0][3])

        def reddish(px):
            rr, g, b = (px >> 16) & 255, (px >> 8) & 255, px & 255
            return rr > 150 and rr > g + 40 and rr > b + 20
        self.assertTrue(
            any(reddish(img.pixel(int(cx + dx), int(cy)))
                for dx in range(int(r) - 3, int(r) + 4)))

    def test_glyphs_are_keyed_by_the_NAMED_well_not_the_current_one(self):
        """set_raster_grid keys off `_current_well` because it is a single-well
        preview; a queue is plate-wide and must not."""
        nav = self._nav()
        nav.set_current_well("B1")
        plain = self._render(nav)
        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, 2.0)]], None, True)})
        after = self._render(nav)
        lay = nav._well_layout()
        cx, cy = nav._well_center(lay, "A1")
        r = nav._well_radius(lay, lay["wells"][0][3])
        self.assertTrue(any(plain.pixel(int(cx), y) != after.pixel(int(cx), y)
                            for y in range(int(cy - r), int(cy + r))))

    def test_junk_overlays_never_raise_in_paint(self):
        nav = self._nav()
        for junk in (None, {}, "x", 5, {"NOPE": ([[(1.0, 1.0), (2.0, 2.0)]],
                                                 None, True)},
                     {"A1": "garbage"}, {"A1": [[(1,)]]},
                     {"A1": ([[]], None, True)}, {"A1": ()},
                     {"A1": ([[(0.0, 0.0), (float("nan"), 1.0)]], None, True)},
                     {5: ([[(0.0, 0.0), (1.0, 1.0)]], None, True)}):
            nav.set_well_paths(junk)
            self._render(nav)

    def test_a_bare_list_and_a_2_tuple_are_accepted(self):
        nav = self._nav()
        nav.set_well_paths({"A1": [[(0.0, 0.0), (0.0, 2.0)]]})
        self.assertIn("A1", nav._well_paths)
        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, 2.0)]], "#ff0000")})
        self.assertEqual(nav._well_paths["A1"][1], "#ff0000")

    def test_a_tiny_well_degrades_to_a_dot(self):
        """At the radius floor a scribble carries no shape and costs thousands of
        lineTo calls, so the polyline is never walked."""
        nav = self._nav(384)
        nav.resize(200, 140)
        lay = nav._well_layout()
        r = nav._well_radius(lay, lay["wells"][0][3])
        self.assertLess(r, nav._GLYPH_MIN_R_PX)
        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, 1.0)]], None, True)})
        self._render(nav)          # must not raise

    def test_set_plate_clears_the_overlays_and_the_selection(self):
        nav = self._nav()
        nav.set_multi_select_enabled(True)
        nav.set_well_paths({"A1": ([[(0.0, 0.0), (0.0, 2.0)]], None, True)})
        nav.set_selected_wells(["A1", "A2"])
        nav.set_plate(WellPlate.from_format(96))
        self.assertEqual(nav._well_paths, {})
        self.assertEqual(nav.selected_wells(), [])


class TestNavigatorSelection(_QtBase):
    def _nav(self):
        from gui.widgets.jog_well_plate import WellPlateNavigator
        n = WellPlateNavigator()
        n.set_plate(WellPlate.from_format(24))
        n.resize(400, 300)
        pm = QPixmap(n.size())
        pm.fill()
        n.render(pm)
        n.set_multi_select_enabled(True)
        return n

    @staticmethod
    def _pt(nav, well):
        return QPointF(*nav._well_center(nav._well_layout(), well))

    def _press(self, nav, well, mods=Qt.KeyboardModifier.NoModifier):
        nav.mousePressEvent(QMouseEvent(
            QEvent.Type.MouseButtonPress, self._pt(nav, well),
            Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton, mods))

    def _move(self, nav, well, buttons=Qt.MouseButton.LeftButton):
        nav.mouseMoveEvent(QMouseEvent(
            QEvent.Type.MouseMove, self._pt(nav, well),
            Qt.MouseButton.NoButton, buttons, Qt.KeyboardModifier.NoModifier))

    def _release(self, nav, well):
        nav.mouseReleaseEvent(QMouseEvent(
            QEvent.Type.MouseButtonRelease, self._pt(nav, well),
            Qt.MouseButton.LeftButton, Qt.MouseButton.NoButton,
            Qt.KeyboardModifier.NoModifier))

    def test_a_drag_across_three_wells_selects_three(self):
        nav = self._nav()
        self._press(nav, "A1")
        self._move(nav, "A2")
        self._move(nav, "A3")
        self._release(nav, "A3")
        self.assertEqual(nav.selected_wells(), ["A1", "A2", "A3"])

    def test_selection_changed_fires_once_per_new_well_not_per_move(self):
        nav = self._nav()
        seen = []
        nav.selection_changed.connect(lambda w: seen.append(list(w)))
        self._press(nav, "A1")
        for _ in range(6):
            self._move(nav, "A2")
            self._move(nav, "A3")
        self._release(nav, "A3")
        self.assertEqual(len(seen), 3, f"expected 3 emits, got {seen}")

    def test_re_setting_the_SAME_selection_does_not_re_emit(self):
        """`_set_selection`'s equality gate, pinned DIRECTLY. The drag path has
        its own "already touched" guard, so a drag test alone leaves this gate
        unpinned — a mutation removing it SURVIVED until this test existed."""
        nav = self._nav()
        nav.set_selected_wells(["A1", "A2"])
        seen = []
        nav.selection_changed.connect(lambda w: seen.append(list(w)))
        nav.set_selected_wells(["A1", "A2"])
        nav.set_selected_wells(["A2", "A1"])          # same SET, other order
        self.assertEqual(seen, [], f"an unchanged selection re-emitted: {seen}")
        nav.set_selected_wells(["A1"])
        self.assertEqual(len(seen), 1)

    def test_a_plain_press_emits_well_clicked_but_ctrl_does_not(self):
        """Adding to a selection must not move what the operator is editing."""
        nav = self._nav()
        seen = []
        nav.well_clicked.connect(seen.append)
        self._press(nav, "B2")
        self._release(nav, "B2")
        self.assertEqual(seen, ["B2"])
        self._press(nav, "B3", Qt.KeyboardModifier.ControlModifier)
        self._release(nav, "B3")
        self.assertEqual(seen, ["B2"])
        self.assertEqual(nav.selected_wells(), ["B2", "B3"])

    def test_a_plain_press_replaces_the_selection(self):
        nav = self._nav()
        nav.set_selected_wells(["A1", "A2"])
        self._press(nav, "C1")
        self._release(nav, "C1")
        self.assertEqual(nav.selected_wells(), ["C1"])

    def test_hover_without_a_button_does_not_select(self):
        nav = self._nav()
        self._press(nav, "A1")
        self._release(nav, "A1")
        self._move(nav, "A2", buttons=Qt.MouseButton.NoButton)
        self.assertEqual(nav.selected_wells(), ["A1"])

    def test_selected_wells_is_plate_order_not_click_order(self):
        nav = self._nav()
        nav.set_selected_wells(["B3", "A1", "A2"])
        self.assertEqual(nav.selected_wells(), ["A1", "A2", "B3"])

    def test_shift_range_takes_the_rectangular_span_on_a_real_grid(self):
        nav = self._nav()
        self._press(nav, "A1")
        self._release(nav, "A1")
        self._press(nav, "B2", Qt.KeyboardModifier.ShiftModifier)
        self._release(nav, "B2")
        self.assertEqual(set(nav.selected_wells()),
                         {"A1", "A2", "B1", "B2"})

    def test_leave_event_does_not_cancel_a_live_drag(self):
        """The mouse is grabbed during a drag, so moves keep arriving after the
        pointer leaves; cancelling would drop wells on every overshoot."""
        nav = self._nav()
        self._press(nav, "A1")
        self._move(nav, "A2")
        nav.leaveEvent(None)
        self._move(nav, "A3")
        self.assertEqual(nav.selected_wells(), ["A1", "A2", "A3"])

    def test_disabling_multi_select_clears_the_selection(self):
        nav = self._nav()
        nav.set_selected_wells(["A1"])
        nav.set_multi_select_enabled(False)
        self.assertEqual(nav.selected_wells(), [])

    def test_unknown_names_are_ignored_and_do_not_raise(self):
        nav = self._nav()
        nav.set_selected_wells(["A1", "ZZ99", None, 5])
        self.assertIn("A1", nav.selected_wells())


# ════════════════════════════════════════════════════════════════════
#  2. The page: layout, snapshots, write-back
# ════════════════════════════════════════════════════════════════════

class TestSetupZoneLayout(_QtBase):
    def test_the_right_pane_is_a_vertical_splitter_of_preview_over_plate(self):
        page = self._page()
        setup = page._zone_stack.widget(0)
        right = setup.widget(1)
        self.assertIsInstance(right, QSplitter)
        self.assertEqual(right.orientation(), Qt.Vertical)
        # The preview is above; the plate below.
        self.assertIn(page._setup_preview, right.widget(0).findChildren(
            type(page._setup_preview)))
        self.assertIn(page._navigator,
                      right.widget(1).findChildren(type(page._navigator)))

    def test_the_setup_preview_is_a_SECOND_monitor_view(self):
        page = self._page()
        self.assertIsNot(page._setup_preview, page._traj_view)
        self.assertIsInstance(page._setup_preview, type(page._traj_view))

    def test_the_attribute_contract_still_holds(self):
        """Imported, not copied, so it can never drift from the original."""
        from tests.test_v77_quick_print_zones import _CONTRACT
        page = self._page()
        missing = [n for n in _CONTRACT if not hasattr(page, n)]
        self.assertEqual(missing, [])

    def test_run_all_and_hold_live_on_the_run_row_not_the_zone_stack(self):
        """They must be reachable from the Run zone, which is where you are when
        you want to know whether to keep going."""
        page = self._page()
        for btn in (page._run_all_btn, page._hold_btn, page._print_btn):
            parent = btn.parent()
            seen = []
            while parent is not None:
                seen.append(parent)
                parent = parent.parent()
            self.assertNotIn(page._zone_stack, seen)

    def test_the_queue_list_is_a_read_only_readout(self):
        """The plate is the map, and the map is the single answer to "which
        wells" — a second selection model here would be two homes for one fact."""
        page = self._page()
        self.assertEqual(page._queue_list.selectionMode(),
                         QListWidget.SelectionMode.NoSelection)

    def test_a_partial_page_survives_every_new_entry_point(self):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        p = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        p._queue = []
        p._glyph_cache = {}
        p._applying_snapshot = False
        p._selected_well = None
        for call in (p._refresh_queue_ui, p._refresh_apply_button,
                     p._write_back, p._refresh_queue_overlay,
                     p._plan_views):
            call()
        self.assertEqual(p._plan_views(), [])


class TestPlanFanOut(_QtBase):
    def test_the_planned_path_reaches_both_views(self):
        page = self._page()
        page._refresh_planned_path()
        self.assertTrue(page._traj_view._segments)
        self.assertTrue(page._setup_preview._segments)
        self.assertEqual(page._traj_view._segments,
                         page._setup_preview._segments)

    def test_the_live_trace_reaches_only_the_run_view(self):
        """During a queue run the Setup preview shows the ACTIVE print, which may
        not be the well printing — a needle dot there would be FALSE."""
        page = self._page()
        page._refresh_planned_path()
        page._traj_view.set_recording(True)
        for i in range(5):
            page._traj_view.set_position(10000.0 + i * 500, 12000.0)
            page._setup_preview.set_position(10000.0 + i * 500, 12000.0)
        self.assertTrue(page._traj_view._executed)
        self.assertFalse(page._setup_preview._executed,
                         "the Setup preview must never record a trace")

    def test_the_live_position_push_targets_only_the_run_view(self):
        """`_push_live_position` runs at 10 Hz during a print. Fanning it out
        would double an off-screen widget's repaint cost for information that is
        actively FALSE mid-queue (the Setup preview shows the ACTIVE print, which
        need not be the well printing)."""
        page = self._page()
        page._refresh_planned_path()
        page._traj_view.set_recording(True)
        page._setup_preview.set_recording(True)      # even if armed…
        page._controller.get_xy_position.return_value = (10500.0, 12000.0)
        for _ in range(4):
            page._push_live_position()
        self.assertFalse(page._setup_preview._executed,
                         "_push_live_position must not reach the Setup preview")


class TestSnapshots(_QtBase):
    def test_capture_then_apply_round_trips_every_field(self):
        page = self._page()
        page._size_spin.setValue(3.5)
        page._top_speed_spin.setValue(1.75)
        page._resolution_spin.setValue(55.0)
        page._printz_spin.setValue(0.42)
        page._extrusion_mod_spin.setValue(1.6)
        page._ink_z_spin.setValue(1.1)
        snap = page._snapshot_from_widgets("A1")
        # Move everything away, then apply the snapshot back.
        page._size_spin.setValue(1.0)
        page._top_speed_spin.setValue(0.5)
        page._resolution_spin.setValue(30.0)
        page._printz_spin.setValue(0.1)
        page._extrusion_mod_spin.setValue(1.0)
        page._ink_z_spin.setValue(0.5)
        page._apply_snapshot(snap)
        got = page._snapshot_from_widgets("A1")
        self.assertEqual(got.to_dict(), snap.to_dict())

    def test_a_missing_object_is_reported_not_coerced_to_index_zero(self):
        """Coercing would make merely OPENING an entry silently EDIT it."""
        page = self._page()
        before = page._object_combo.currentIndex()
        snap = page._snapshot_from_widgets("A1")
        snap.object_data = "file:deleted_print"
        warnings = page._apply_snapshot(snap)
        self.assertEqual(page._object_combo.currentIndex(), before)
        self.assertTrue(any("no longer in the print library" in w
                            for w in warnings), warnings)

    def test_a_clamped_value_is_disclosed(self):
        """`_update_top_speed_cap` puts a dynamic maximum on the speed spin, so a
        snapshot queued faster than the machine can now go would quietly run
        slower."""
        page = self._page()
        page._top_speed_spin.setMaximum(2.0)
        snap = page._snapshot_from_widgets("A1")
        snap.top_speed_mm_s = 9.0
        warnings = page._apply_snapshot(snap)
        self.assertTrue(any("top speed" in w and "limited" in w
                            for w in warnings), warnings)

    def test_the_ink_map_survives_the_object_change_fan_out(self):
        """_rebuild_ink_mapping_ui hard-resets `_ink_map` and re-derives it from
        the name-keyed `_ink_map_last`, so a snapshot must seed THAT."""
        page = self._page()
        sk = Sketch(line_spacing_mm=0.4)
        sk.inks = [SketchInk(1, "Struct", "#89b4fa"),
                   SketchInk(2, "Support", "#a6e3a1")]
        sk._next_ink_id = 3
        sk.shapes = [
            SketchShape(kind="line", points=[(-3, 0), (-1, 0)], ink_id=1),
            SketchShape(kind="line", points=[(0, 0), (2, 0)], ink_id=2),
        ]
        page._loaded_sketch = sk
        page._rebuild_ink_mapping_ui()
        snap = page._snapshot_from_widgets("A1")
        snap.ink_map_by_name = {"Struct": "Gelatin", "Support": "Alginate"}
        page._apply_snapshot(snap)
        # In production `_apply_snapshot` re-derives this from the snapshot's own
        # object; the combo here points at a simple shape, so stand it back up.
        page._loaded_sketch = sk
        page._rebuild_ink_mapping_ui()          # the fan-out that used to wipe it
        self.assertEqual(page._ink_map.get(1), "Gelatin")
        self.assertEqual(page._ink_map.get(2), "Alginate")

    def test_the_snapshot_seeds_ink_map_last_not_ink_map(self):
        """The mechanism, pinned directly: `_rebuild_ink_mapping_ui` hard-resets
        `_ink_map`, so assigning it would be wiped by the very refresh that
        follows applying a snapshot."""
        page = self._page()
        page._ink_map_last = {}
        snap = page._snapshot_from_widgets("A1")
        snap.ink_map_by_name = {"Struct": "Gelatin"}
        page._apply_snapshot(snap)
        self.assertEqual(page._ink_map_last.get("Struct"), "Gelatin")

    def test_applying_a_snapshot_does_not_write_back(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2)])
        page._selected_well = "A1"
        other = page._snapshot_from_widgets("A1")
        other.print_z_mm = 7.7
        with page._snapshot_applied(other):
            page._write_back()          # must be a no-op while applying
        self.assertAlmostEqual(page._queue_get("A1").print_z_mm, 0.2)


class TestWriteBack(_QtBase):
    def test_editing_updates_the_active_entry(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2)])
        page._on_well_clicked("A1")
        page._printz_spin.setValue(0.66)
        page._flush_write_back()
        self.assertAlmostEqual(page._queue_get("A1").print_z_mm, 0.66)

    def test_editing_an_unqueued_well_creates_nothing(self):
        """Editing an un-queued active well is COMPOSING a new print; it becomes
        an entry only when Apply says so."""
        page = self._page()
        page._queue = []
        page._on_well_clicked("C2")
        page._printz_spin.setValue(0.77)
        page._flush_write_back()
        self.assertIsNone(page._queue_get("C2"))

    def test_switching_wells_flushes_into_the_OUTGOING_well(self):
        """A debounce that has not fired would otherwise land in the wrong entry
        or be lost outright."""
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])
        page._on_well_clicked("A1")
        page._printz_spin.setValue(0.88)      # timer pending, NOT fired
        page._on_well_clicked("A2")
        self.assertAlmostEqual(page._queue_get("A1").print_z_mm, 0.88)
        self.assertAlmostEqual(page._queue_get("A2").print_z_mm, 0.2)

    def test_the_debounce_timer_is_actually_wired_to_the_write_back(self):
        """Every other test in this class calls `_flush_write_back()`, which
        invokes `_write_back` DIRECTLY — so they ALL pass with the timer left
        unconnected. Pinned by source, because that is the wiring, plus the
        behavioural test below. (A mutation unwiring the timer SURVIVED until
        both existed — the same weakness the v7.18 setpoint keeper had.)"""
        import ast
        import inspect
        from gui.pages.workflows import quick_print_workflow as mod
        tree = ast.parse(inspect.getsource(mod))
        found = False
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            if not (isinstance(f, ast.Attribute) and f.attr == "connect"):
                continue
            owner = f.value
            if not (isinstance(owner, ast.Attribute)
                    and owner.attr == "timeout"
                    and isinstance(owner.value, ast.Attribute)
                    and owner.value.attr == "_writeback_timer"):
                continue
            arg = node.args[0] if node.args else None
            if isinstance(arg, ast.Attribute) and arg.attr == "_write_back":
                found = True
        self.assertTrue(
            found, "_writeback_timer.timeout must connect to self._write_back")

    def test_letting_the_TIMER_fire_writes_back(self):
        """The behavioural half: no direct call, no flush — only the timer."""
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2)])
        page._on_well_clicked("A1")
        page._printz_spin.setValue(0.51)      # starts the debounce
        self.assertTrue(page._writeback_timer.isActive(),
                        "editing did not start the debounce at all")
        for _ in range(60):
            self._app.processEvents()
            if abs(page._queue_get("A1").print_z_mm - 0.51) < 1e-6:
                break
            threading.Event().wait(0.02)
        self.assertAlmostEqual(page._queue_get("A1").print_z_mm, 0.51)

    def test_clicking_a_queued_well_loads_its_snapshot(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Gelatin", 1.45)])
        page._on_well_clicked("A2")
        self.assertAlmostEqual(page._printz_spin.value(), 1.45)
        self.assertEqual(page._selected_ink(), "Gelatin")

    def test_loading_a_queued_print_refreshes_exactly_once(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.9)])
        calls = []
        orig = page._refresh_planned_path
        page._refresh_planned_path = lambda *a, **k: (calls.append(1),
                                                      orig(*a, **k))[1]
        page._on_well_clicked("A2")
        self.assertEqual(len(calls), 1, f"{len(calls)} refreshes, expected 1")


class TestApplyToWells(_QtBase):
    def test_apply_stamps_an_independent_copy_into_every_selected_well(self):
        page = self._page()
        page._navigator.set_selected_wells(["A1", "A2", "B1"])
        with _patches()[3]:
            page._on_apply_print_to_wells()
        self.assertEqual(sorted(q.well for q in page._queue),
                         ["A1", "A2", "B1"])
        page._queue_get("A1").print_z_mm = 9.99
        self.assertNotAlmostEqual(page._queue_get("A2").print_z_mm, 9.99)

    def test_apply_is_disabled_with_no_selection_and_names_the_count(self):
        page = self._page()
        page._navigator.set_selected_wells([])
        page._refresh_apply_button()
        self.assertFalse(page._apply_btn.isEnabled())
        page._navigator.set_selected_wells(["A1", "A2", "A3"])
        page._refresh_apply_button()
        self.assertTrue(page._apply_btn.isEnabled())
        self.assertIn("3", page._apply_btn.toolTip())

    def test_apply_works_with_the_stage_disconnected(self):
        """Laying out a plate offline is the whole point of a queue."""
        page = self._page()
        page._controller.is_xy_connected = False
        page._controller.is_zp_connected = False
        page._navigator.set_selected_wells(["A1"])
        page._refresh_apply_button()
        self.assertTrue(page._apply_btn.isEnabled())

    def test_remove_falls_back_to_the_active_well(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])
        page._selected_well = "A1"
        page._navigator.set_selected_wells([])
        page._on_queue_remove()
        self.assertEqual([q.well for q in page._queue], ["A2"])

    def test_remove_acts_on_the_plate_selection(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2),
                           ("B1", "Alginate", 0.2)])
        page._navigator.set_selected_wells(["A1", "B1"])
        page._on_queue_remove()
        self.assertEqual([q.well for q in page._queue], ["A2"])


class TestQueueGlyphCache(_QtBase):
    """384 wells × `generate_object_trajectory` on a 120 ms debounce is not
    viable. The cache is keyed by CONFIG identity, not by well, because a real
    queue is usually "the same print in forty wells" — so it normally holds ONE
    entry however many wells are queued."""

    def test_many_identical_prints_share_ONE_cache_entry(self):
        """Keyed by config, so N identical prints cost one geometry build. Keying
        by well would make this N — the property M24 mutates."""
        page = self._page()
        page._glyph_cache.clear()
        misses = []
        orig = page._glyph_paths_for

        def traced(qp):
            key = (qp.object_data, round(float(qp.size_mm), 4),
                   round(float(qp.resolution_um), 3))
            if key not in page._glyph_cache:
                misses.append(qp.well)
            return orig(qp)
        page._glyph_paths_for = traced
        page._navigator.set_selected_wells(
            [w for w in _WELLS if w not in ("A3", "A4")])
        with _patches()[3]:
            page._on_apply_print_to_wells()
        self.assertGreaterEqual(len(page._queue), 6)
        self.assertEqual(len(page._glyph_cache), 1,
                         f"{len(page._queue)} identical prints produced "
                         f"{len(page._glyph_cache)} cache entries")
        self.assertEqual(len(misses), 1, f"rebuilt for {misses}")

    def test_a_different_size_is_a_different_cache_entry(self):
        page = self._page()
        page._glyph_cache.clear()
        a = page._snapshot_from_widgets("A1")
        b = a.copy()
        b.well = "A2"
        b.size_mm = a.size_mm + 1.5
        page._queue = [a, b]
        page._refresh_queue_ui()
        self.assertEqual(len(page._glyph_cache), 2)

    def test_the_cache_is_capped(self):
        page = self._page()
        page._glyph_cache = {("k%d" % i,): [] for i in range(33)}
        page._glyph_paths_for(page._snapshot_from_widgets("A1"))
        self.assertLessEqual(len(page._glyph_cache), 33)


class TestPersistence(_QtBase):
    def test_the_queue_round_trips_and_ink_map_last_still_does(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.31), ("B2", "Gelatin", 0.62)])
        page._ink_map_last = {"Struct": "Alginate"}
        page._group_by_ink_check.setChecked(True)
        extra = page._collect_extra_state()

        page2 = self._page()
        page2._restore_extra_state(extra)
        self.assertEqual(sorted(q.well for q in page2._queue), ["A1", "B2"])
        self.assertAlmostEqual(page2._queue_get("B2").print_z_mm, 0.62)
        self.assertEqual(page2._ink_map_last, {"Struct": "Alginate"})
        self.assertTrue(page2._group_by_ink_check.isChecked())

    def test_garbage_extra_state_never_raises(self):
        page = self._page()
        for junk in (None, {}, "str", 5, {"print_queue": "nope"},
                     {"print_queue": {"prints": ["x", 5, {}]}},
                     {"ink_map_last": 5}):
            page._restore_extra_state(junk)

    def test_an_entry_for_a_well_not_on_this_plate_SURVIVES_and_is_flagged(self):
        """The operator may be loading a 96-well profile with a 24-well plate
        mounted; destroying their layout silently would be unrecoverable."""
        page = self._page()
        page.set_calibration_data(WellPlate.from_format(24), _WELLS, -35.0)
        page._restore_extra_state({"print_queue": {"prints": [
            {"well": "H12", "object_data": "simple:dot"}]}})
        self.assertEqual([q.well for q in page._queue], ["H12"])
        self.assertTrue(page._queue_entry_problem(page._queue[0]))
        self.assertIs(page._navigator._well_paths.get("H12", (None, None,
                                                              True))[2], False)


# ════════════════════════════════════════════════════════════════════
#  3. The resolve pass
# ════════════════════════════════════════════════════════════════════

class TestResolve(_QtBase):
    def test_each_unit_carries_its_OWN_settings(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.20),
                               ("A2", "Alginate", 0.90)])
        q[0].top_speed_mm_s = 0.05
        q[1].top_speed_mm_s = 0.09
        units, skipped, _w = page._resolve_queue(q)
        self.assertEqual(len(units), 2, skipped)
        self.assertNotAlmostEqual(units[0]["settings"].print_z_height,
                                  units[1]["settings"].print_z_height)

    def test_the_widgets_are_restored_afterwards(self):
        page = self._page()
        q = self._queue(page, [("A2", "Gelatin", 1.9)])
        page._printz_spin.setValue(0.15)
        page._set_combo_data(page._ink_combo, "Alginate")
        # Let the real speed cap settle first: it is derived from the needle's
        # flow ceiling, so a pre-cap value could not survive ANY settings refresh
        # and comparing against one would test the cap, not the restore.
        page._on_settings_changed()
        before = page._snapshot_from_widgets("X").to_dict()
        page._resolve_queue(q)
        after = page._snapshot_from_widgets("X").to_dict()
        self.assertEqual(before, after)

    def test_the_widgets_are_restored_even_when_a_middle_entry_raises(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.3),
                               ("B1", "Alginate", 0.4)])
        page._on_settings_changed()
        before = page._snapshot_from_widgets("X").to_dict()
        n = {"i": 0}
        orig = page._build_settings

        def boom(*a, **k):
            n["i"] += 1
            if n["i"] == 2:
                raise RuntimeError("boom")
            return orig(*a, **k)
        page._build_settings = boom
        units, skipped, _w = page._resolve_queue(q)
        page._build_settings = orig
        self.assertEqual(page._snapshot_from_widgets("X").to_dict(), before)
        self.assertTrue(any("boom" in why for _w2, why in skipped), skipped)
        self.assertEqual(len(units), 2, "the other entries must still resolve")

    def test_the_print_floor_is_checked_PER_UNIT(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.3),
                               ("B1", "Alginate", 0.4)])
        page._controller.print_floor_violation.reset_mock()
        page._resolve_queue(q)
        self.assertGreaterEqual(
            page._controller.print_floor_violation.call_count, 3,
            "each snapshot has its own height — one check cannot cover them")

    def test_a_floor_violation_is_named_by_well_in_the_warnings(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.3)])
        page._controller.print_floor_violation.side_effect = (
            lambda *a, **k: True)
        _u, _s, warnings = page._resolve_queue(q)
        self.assertTrue(any("plate bottom" in w for w in warnings), warnings)
        self.assertTrue(any("A1" in w for w in warnings), warnings)

    def test_the_floor_check_uses_EACH_units_own_height(self):
        """Not just "it is called N times" — the VALUE passed must be that
        snapshot's own resolved height, or resolving it once outside the
        transient-apply block would go unnoticed."""
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.20),
                               ("A2", "Alginate", 0.90)])
        # Only the deeper print violates. print_height_to_zref is -25 - h, so
        # A2's height resolves to -25.9 and A1's to -25.2.
        page._controller.print_floor_violation.side_effect = (
            lambda z, *a, **k: float(z) < -25.5)
        _u, _s, warnings = page._resolve_queue(q)
        floor = [w for w in warnings if "plate bottom" in w]
        self.assertTrue(floor, warnings)
        self.assertIn("A2", floor[0])
        self.assertNotIn("A1", floor[0],
                         "A1 does not violate the floor — the check cannot be "
                         "using one shared height for every unit")

    def test_a_missing_print_file_is_skipped_by_name_and_the_rest_still_run(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])
        bad = q[0].copy()
        bad.well = "B1"
        bad.object_data = "file:definitely_not_a_print"
        page._queue = q + [bad]
        units, skipped, _w = page._resolve_queue(page._queue)
        self.assertTrue(any(w == "B1" for w, _ in skipped), skipped)
        self.assertEqual(len(units), 2)

    def test_an_uncalibrated_well_is_refused_not_run_at_a_guess(self):
        """`_well_center_zero_ref_mm` has a GEOMETRIC fallback; a batch that dips
        into ink wells and prints at a computed height must not run against it."""
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2)])
        q[0].well = "C2"
        page.set_calibration_data(
            WellPlate.from_format(96),
            {k: v for k, v in _WELLS.items() if k != "C2"}, -35.0)
        units, skipped, _w = page._resolve_queue(q)
        self.assertEqual(units, [])
        self.assertTrue(any("calibrat" in why for _w2, why in skipped), skipped)

    def test_an_uncalibrated_queued_well_is_FLAGGED_on_the_plate(self):
        """The display half of the same rule, which is what
        `_queue_entry_problem` owns: without it the well would look perfectly
        runnable right up until Run all refused it."""
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2)])
        qp = q[0].copy()
        qp.well = "C2"
        page.set_calibration_data(
            WellPlate.from_format(96),
            {k: v for k, v in _WELLS.items() if k != "C2"}, -35.0)
        page._queue = [qp]
        page._refresh_queue_ui()
        self.assertTrue(page._queue_entry_problem(qp),
                        "an uncalibrated well must report a problem")
        self.assertIs(page._navigator._well_paths["C2"][2], False,
                      "it must be drawn as NOT runnable")

    def test_expand_ALSO_refuses_an_uncalibrated_well_on_its_own(self):
        """Two enforcement points, deliberately, on a path that would otherwise
        drive to a GUESSED position: `_resolve_queue`'s cheap pre-check skips the
        entry, and `_expand_queued_print` refuses independently. Each is pinned,
        so neither can become dead code that a future second caller relies on."""
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2)])
        qp = q[0].copy()
        qp.well = "C2"
        page.set_calibration_data(
            WellPlate.from_format(96),
            {k: v for k, v in _WELLS.items() if k != "C2"}, -35.0)
        from gui.pages.workflows.quick_print_workflow import _QueueResolveError
        with page._snapshot_applied(qp):
            with self.assertRaises(_QueueResolveError):
                page._expand_queued_print(qp, 0)

    def test_no_ink_is_refused_while_prep_is_on(self):
        """After a swap the needle has been washed out, so "print with whatever is
        loaded" is a continuity claim this engine has already invalidated."""
        page = self._page(ink=None)
        page._prep_check.setChecked(True)
        q = self._queue(page, [("A1", None, 0.2)])
        q[0].ink_name = ""
        units, skipped, _w = page._resolve_queue(q)
        self.assertEqual(units, [])
        self.assertTrue(any("no ink" in why for _w2, why in skipped), skipped)

    def test_no_ink_is_allowed_when_nothing_is_serviced(self):
        page = self._page(ink=None)
        page._prep_check.setChecked(False)
        page._postclean_check.setChecked(False)
        q = self._queue(page, [("A1", None, 0.2)])
        q[0].ink_name = ""
        units, skipped, _w = page._resolve_queue(q)
        self.assertEqual(len(units), 1, skipped)

    def test_a_multi_ink_entry_expands_in_SKETCH_order(self):
        """The units' order is the sketch's deposition order (support before
        structure); regrouping them globally would print it wrong."""
        page = self._page()
        sk = Sketch(line_spacing_mm=0.4)
        sk.inks = [SketchInk(1, "Struct", "#89b4fa"),
                   SketchInk(2, "Support", "#a6e3a1")]
        sk._next_ink_id = 3
        sk.shapes = [
            SketchShape(kind="line", points=[(-3, 0), (-1, 0)], ink_id=1),
            SketchShape(kind="line", points=[(0, 0), (2, 0)], ink_id=2),
            SketchShape(kind="line", points=[(3, 0), (5, 0)], ink_id=1),
        ]
        page._loaded_sketch = sk
        page._ink_map_last = {"Struct": "Alginate", "Support": "Gelatin"}
        page._rebuild_ink_mapping_ui()
        qp = page._snapshot_from_widgets("A1")
        with page._snapshot_applied(qp):
            page._loaded_sketch = sk
            page._ink_map = {1: "Alginate", 2: "Gelatin"}
            units = page._expand_queued_print(qp, 0)
        self.assertEqual([u["ink_name"] for u in units],
                         ["Alginate", "Gelatin", "Alginate"])
        self.assertTrue(all(u["sub_total"] == 3 for u in units))

    def test_a_single_ink_entry_makes_exactly_one_unit(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2)])
        units, skipped, _w = page._resolve_queue(q)
        self.assertEqual(len(units), 1, skipped)
        self.assertEqual(units[0]["sub_total"], 1)

    def test_every_unit_carries_its_own_ideal_points_for_the_report(self):
        page = self._page()
        q = self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])
        units, _s, _w = page._resolve_queue(q)
        self.assertNotEqual(units[0]["ideal_pts"], units[1]["ideal_pts"])
        self.assertTrue(all(u["ideal_pts"] for u in units))


# ════════════════════════════════════════════════════════════════════
#  4. The run-all worker
# ════════════════════════════════════════════════════════════════════

class TestRunAllWorker(_QtBase):
    def _run(self, page, spec, *, group_by_ink=False, clean_between=False):
        self._queue(page, spec)
        page._group_by_ink_check.setChecked(group_by_ink)
        page._clean_between_check.setChecked(clean_between)
        p = _patches()
        with p[0], p[1], p[2], p[3], p[4]:
            page._on_run_all()
            t = page._queue_thread
            self.assertIsNotNone(t, "the worker never started")
            t.join(10)
            self.assertFalse(t.is_alive(), "the worker did not finish")
            # The worker reports through QUEUED cross-thread signals, so the
            # GUI-thread slots (status text, live toggle, report load) have not
            # run until the event loop is pumped.
            for _ in range(5):
                self._app.processEvents()
        return list(CALLS)

    def test_one_prep_first_one_cleanup_last_and_always_a_retract(self):
        page = self._page()
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Alginate", 0.2)])
        kinds = [c[0] for c in calls]
        self.assertEqual(kinds.count("prep"), 1)
        self.assertEqual(kinds.count("cleanup"), 1)
        self.assertEqual(kinds[0], "prep")
        self.assertEqual(kinds[-1], "retract")
        self.assertEqual(kinds[-2], "cleanup")

    def test_one_job_per_unit(self):
        page = self._page()
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Alginate", 0.2),
                                 ("B1", "Alginate", 0.2)])
        self.assertEqual(len([c for c in calls if c[0] == "job"]), 3)

    def test_a_same_ink_queue_costs_ZERO_swaps(self):
        page = self._page()
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Alginate", 0.2),
                                 ("B1", "Alginate", 0.2)])
        self.assertEqual(len([c for c in calls if c[0] == "swap"]), 0)

    def test_a_swap_happens_only_on_an_ink_CHANGE(self):
        page = self._page()
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Gelatin", 0.2),
                                 ("B1", "Gelatin", 0.2),
                                 ("B2", "Alginate", 0.2)])
        self.assertEqual(len([c for c in calls if c[0] == "swap"]), 2)

    def test_clean_between_every_print_forces_a_swap_at_every_boundary(self):
        page = self._page()
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Alginate", 0.2),
                                 ("B1", "Alginate", 0.2)],
                          clean_between=True)
        self.assertEqual(len([c for c in calls if c[0] == "swap"]), 2)

    def test_group_by_ink_reduces_the_swaps(self):
        page = self._page()
        spec = [("A1", "Alginate", 0.2), ("A2", "Gelatin", 0.2),
                ("B1", "Alginate", 0.2), ("B2", "Gelatin", 0.2)]
        plate = self._run(page, spec)
        n_plate = len([c for c in plate if c[0] == "swap"])
        CALLS.clear()
        page2 = self._page()
        ink = self._run(page2, spec, group_by_ink=True)
        n_ink = len([c for c in ink if c[0] == "swap"])
        self.assertEqual((n_plate, n_ink), (3, 1))

    def test_aspirate_immediately_precedes_its_unit_job(self):
        page = self._page()
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Gelatin", 0.2)])
        seq = [c[0] for c in calls if c[0] in ("aspirate", "job")]
        self.assertEqual(seq, ["aspirate", "job", "aspirate", "job"])

    def test_the_single_print_state_machine_is_never_touched(self):
        """`bridge.state` would fire _on_state's single-print teardown, which
        nulls `_pm`, stops live sampling and launches a post-print cleanup."""
        page = self._page()
        seen = []
        page._bridge.state.connect(seen.append)
        self._run(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])
        self.assertEqual(seen, [])
        self.assertIsNone(page._post_print_ctx)
        self.assertIsNone(page._cleanup_thread)

    def test_the_last_units_log_is_what_the_report_gets(self):
        page = self._page()
        loaded = {}
        page._load_report = lambda p, **kw: loaded.update(
            {"path": p, **kw})
        self._run(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])
        self.assertTrue(page._last_log_path)
        self.assertIn("A2", str(loaded.get("path")))
        # And with the LAST unit's own geometry, not the on-screen object's.
        self.assertTrue(loaded.get("ideal_pts"))
        self.assertEqual(loaded["context"]["well"], "A2")
        self.assertIn("queue", loaded["context"])

    def test_set_print_live_is_only_ever_called_on_the_main_thread(self):
        page = self._page()
        idents = []
        orig = page._set_print_live
        page._set_print_live = lambda on: (
            idents.append(threading.get_ident()), orig(on))[1]
        self._run(page, [("A1", "Alginate", 0.2)])
        self.assertTrue(idents)
        self.assertEqual(set(idents), {threading.get_ident()},
                         "a QTimer was touched from the worker thread")

    def test_an_aborted_unit_stops_the_run_and_skips_the_cleanup(self):
        page = self._page()
        _FakePM.abort_at = "@ A2"
        calls = self._run(page, [("A1", "Alginate", 0.2),
                                 ("A2", "Alginate", 0.2),
                                 ("B1", "Alginate", 0.2)])
        kinds = [c[0] for c in calls]
        self.assertEqual(kinds.count("job"), 2)     # A1 ran, A2 aborted
        self.assertEqual(kinds.count("cleanup"), 0)
        self.assertEqual(kinds[-1], "retract")
        self.assertIn("aborted", page._status.text().lower())
        self.assertIn("still holds", page._status.text())

    def test_an_abort_in_the_load_job_window_stops_the_unit_from_STARTING(self):
        """The documented start race: `PrintManager.abort()` is gated on
        RUNNING/PAUSED and `start()` clears the abort flag, so an abort landing
        between `self._pm = pm` and `pm.start()` would otherwise be a no-op and
        that unit would print anyway. Gating INSIDE start() cannot see this — the
        gate has to be in load_job."""
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2)])

        reached = threading.Event()
        release = threading.Event()

        class _RacePM(_FakePM):
            def load_job(self, job):
                super().load_job(job)
                if "@ A2" in (job or {}).get("job_name", ""):
                    reached.set()
                    release.wait(10)

        p = _patches()
        with p[0], patch(f"{_QP}.PrintManager", _RacePM), p[2], p[3], p[4]:
            page._on_run_all()
            t = page._queue_thread
            self.assertTrue(reached.wait(10), "never reached unit 2")
            page._queue_abort_requested = True      # the abort lands HERE
            release.set()
            t.join(10)
            self.assertFalse(t.is_alive())
        started = [c for c in CALLS if c[0] == "job"]
        self.assertEqual(len(started), 1,
                         "unit 2 must NOT have started after the abort")

    def test_an_empty_queue_refuses_before_any_worker(self):
        page = self._page()
        page._queue = []
        page._on_run_all()
        self.assertIsNone(page._queue_thread)

    def test_a_no_fit_syringe_budget_blocks_the_run(self):
        page = self._page()
        page._controller.simulate_pump_budget.return_value = {
            "ok": False, "reason": "out_of_bounds", "span_uL": 900.0,
            "capacity_uL": 250.0}
        self._queue(page, [("A1", "Alginate", 0.2)])
        p = _patches()
        with p[0], p[1], p[2], p[3], p[4]:
            page._on_run_all()
        self.assertIsNone(page._queue_thread)
        self.assertIn("syringe", page._status.text().lower())

    def test_an_unchecked_budget_still_proceeds(self):
        page = self._page()
        page._controller.simulate_pump_budget.return_value = {
            "ok": False, "reason": "uncalibrated", "capacity_uL": None}
        calls = self._run(page, [("A1", "Alginate", 0.2)])
        self.assertEqual(len([c for c in calls if c[0] == "job"]), 1)


class _GatedPM(_FakePM):
    """Blocks inside start() until released, so a test can be certain the worker
    is mid-batch when it asks for a hold. Without this the fakes complete the
    whole queue before the request lands and the hold assertions pass vacuously —
    the exact trap this project keeps re-recording."""
    reached = threading.Event()
    release = threading.Event()
    gate_on = None                      # job-name substring to block at

    def start(self):
        name = (self.job or {}).get("job_name", "")
        if _GatedPM.gate_on and _GatedPM.gate_on in name:
            _GatedPM.reached.set()
            _GatedPM.release.wait(10)
        super().start()


class TestHold(_QtBase):
    """A held batch retracts to safe Z and holds no hardware, so the operator can
    go run a fluorescence mosaic and come back."""

    def setUp(self):
        super().setUp()
        _GatedPM.reached.clear()
        _GatedPM.release.clear()
        _GatedPM.gate_on = "@ A1"        # block inside the FIRST unit's print

    def _start(self, page, spec):
        self._queue(page, spec)
        self._ctx = (patch(f"{_QP}.PickPlaceExecutor", _FakeExecutor),
                     patch(f"{_QP}.PrintManager", _GatedPM),
                     patch(f"{_QP}.build_well_plate_job", _fake_build_job),
                     patch.object(QMessageBox, "question", staticmethod(
                         lambda *a, **k: QMessageBox.StandardButton.Yes)),
                     patch.object(QMessageBox, "warning", staticmethod(
                         lambda *a, **k: QMessageBox.StandardButton.Ok)))
        for c in self._ctx:
            c.__enter__()
        page._on_run_all()
        # The worker is now blocked INSIDE unit 1's print.
        self.assertTrue(_GatedPM.reached.wait(10), "the worker never started")
        return page._queue_thread

    def _stop(self):
        _GatedPM.release.set()
        for c in reversed(self._ctx):
            c.__exit__(None, None, None)

    def _wait_held(self, page, t):
        for _ in range(500):
            if page._queue_is_held or not t.is_alive():
                break
            self._app.processEvents()
            threading.Event().wait(0.01)

    def test_hold_parks_the_worker_and_resume_finishes_the_batch(self):
        page = self._page()
        t = self._start(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2),
                               ("B1", "Alginate", 0.2)])
        try:
            page._hold_requested = True         # as the operator would click it
            _GatedPM.release.set()              # let unit 1 finish
            self._wait_held(page, t)
            self.assertTrue(page._queue_is_held, "the worker never parked")
            self.assertEqual(len([c for c in CALLS if c[0] == "job"]), 1,
                             "it must hold BEFORE the second unit")
            self.assertIn(("retract",), CALLS,
                          "a hold must retract to safe Z")
            self.assertTrue(t.is_alive(), "the worker must still be parked")
            page._on_hold_clicked()             # Resume
            t.join(10)
            for _ in range(5):
                self._app.processEvents()
            self.assertFalse(t.is_alive())
            self.assertEqual(len([c for c in CALLS if c[0] == "job"]), 3,
                             "resume must finish the whole batch")
        finally:
            self._stop()

    def test_abort_while_held_wakes_the_worker(self):
        page = self._page()
        t = self._start(page, [("A1", "Alginate", 0.2), ("A2", "Alginate", 0.2),
                               ("B1", "Alginate", 0.2)])
        try:
            page._hold_requested = True
            _GatedPM.release.set()
            self._wait_held(page, t)
            self.assertTrue(page._queue_is_held, "the worker never parked")
            page._on_abort()                    # must set _hold_event too
            t.join(10)
            self.assertFalse(t.is_alive(),
                             "an abort while held never woke the worker")
            self.assertEqual(len([c for c in CALLS if c[0] == "job"]), 1)
            self.assertEqual(len([c for c in CALLS if c[0] == "cleanup"]), 0)
        finally:
            self._stop()

    def test_resume_is_refused_while_something_else_drives_the_stage(self):
        """A per-page `_stage_busy` cannot see another page, so this is the one
        cheap observable — a PROXY, not a lease."""
        page = self._page()
        page._queue_thread = MagicMock()
        page._queue_thread.is_alive.return_value = True
        page._queue_is_held = True
        page._controller.is_position_poller_suspended.return_value = True
        page._hold_event.clear()
        page._on_hold_clicked()
        self.assertFalse(page._hold_event.is_set())
        self.assertIn("driving the stage", page._status.text())
        page._controller.is_position_poller_suspended.return_value = False
        page._on_hold_clicked()
        self.assertTrue(page._hold_event.is_set())
        page._queue_thread = None


class TestSharedAbortPredicate(_QtBase):
    def test_either_engine_satisfies_it(self):
        """`_run_group_job_blocking` reads this to close the documented
        pm.start() race; a queue with its own flag would leave that guard dead."""
        page = self._page()
        self.assertFalse(page._sequence_abort_requested())
        page._queue_abort_requested = True
        self.assertTrue(page._sequence_abort_requested())
        page._queue_abort_requested = False
        page._multi_abort_requested = True
        self.assertTrue(page._sequence_abort_requested())


class TestReentrancy(_QtBase):
    def test_print_refuses_while_a_queued_run_is_alive(self):
        page = self._page()
        page._queue_thread = MagicMock()
        page._queue_thread.is_alive.return_value = True
        with patch(f"{_QP}.PrintManager", _FakePM):
            page._on_print()
        self.assertEqual(_FakePM.instances, [])
        page._queue_thread = None

    def test_run_all_refuses_while_a_print_is_alive(self):
        page = self._page()
        self._queue(page, [("A1", "Alginate", 0.2)])
        page._multi_thread = MagicMock()
        page._multi_thread.is_alive.return_value = True
        page._on_run_all()
        self.assertIsNone(page._queue_thread)
        page._multi_thread = None

    def test_a_live_queue_marks_the_page_busy(self):
        page = self._page()
        page._queue_thread = MagicMock()
        page._queue_thread.is_alive.return_value = True
        page._update_button_state()
        self.assertFalse(page._print_btn.isEnabled())
        self.assertFalse(page._run_all_btn.isEnabled())
        self.assertTrue(page._abort_btn.isEnabled())
        page._queue_thread = None


if __name__ == "__main__":
    unittest.main()
