"""v7.12 Phase 0 — de-risking the plate designer before the redesign.

One guard per defect. These are written against the OLD designer deliberately:
they prove the fix rather than describe it, and they carry forward unchanged
into the redesigned builder.

Each test names the failure it prevents. Reverting the corresponding fix must
fail the test — that property was checked by hand for every one of them.
"""
from __future__ import annotations

import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import (  # noqa: E402
    QApplication, QLineEdit, QVBoxLayout, QWidget,
)
from PySide6.QtGui import QShortcut  # noqa: E402
from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402

from SupportClasses.PlateDesign import (  # noqa: E402
    PERSISTABLE_CONSTRAINT_KINDS, Constraint, PlateDesign, USER_PLATES_DIR,
)
from SupportClasses.PlateSketchSolver import PlateSketchSolver  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402


def _app() -> QApplication:
    return QApplication.instance() or QApplication([])


# ═══════════════════════════════════════════════════════════════════
# D2 — the drag ghost must never reach disk
# ═══════════════════════════════════════════════════════════════════

class TestDragGhostNeverPersists(unittest.TestCase):
    """`PlateSketchSolver.begin_drag` appends a weight-1000 `drag_ghost` into
    `design.constraints`. Any save or undo snapshot taken before `end_drag`
    used to serialize it, and reloading restored it as an INVISIBLE hard pin
    that fought every later edit. Seven had already reached disk.
    """

    def setUp(self):
        self.design = PlateDesign.from_standard_format(24)
        self.well = self.design.get_wells()[0]

    def test_to_dict_mid_drag_emits_no_ghost(self):
        solver = PlateSketchSolver(self.design)
        solver.begin_drag(self.well.center, (5.0, 5.0))
        try:
            # Precondition: the ghost really is live on the model right now.
            self.assertTrue(
                any(c.kind == "drag_ghost" for c in self.design.constraints),
                "expected begin_drag to place a ghost on the design")
            blob = self.design.to_dict()
        finally:
            solver.end_drag()

        kinds = [c.get("kind") for c in blob["constraints"]]
        self.assertNotIn("drag_ghost", kinds)
        self.assertTrue(all(c.get("id", 0) > 0 for c in blob["constraints"]))

    def test_save_mid_drag_writes_no_ghost(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "midrag.json"
            solver = PlateSketchSolver(self.design)
            solver.begin_drag(self.well.center, (5.0, 5.0))
            try:
                self.design.save(path)
            finally:
                solver.end_drag()
            self.assertNotIn("drag_ghost", path.read_text())

    def test_from_dict_drops_a_contaminated_blob(self):
        """A file written before the fix (or hand-edited) heals on load."""
        blob = self.design.to_dict()
        real = len(blob["constraints"])
        blob["constraints"].append({
            "id": -1, "kind": "drag_ghost", "refs": [self.well.center],
            "value": None, "weight": 1000.0, "snapshot": [1.0, 2.0],
        })
        back = PlateDesign.from_dict(blob)
        self.assertEqual(
            [], [c for c in back.constraints if c.kind == "drag_ghost"])
        self.assertEqual(real, len(back.constraints))

    def test_round_trip_preserves_real_constraints(self):
        """The filter must not be over-eager."""
        before = [(c.id, c.kind) for c in self.design.constraints]
        self.assertTrue(before, "fixture should carry at least one constraint")
        back = PlateDesign.from_dict(self.design.to_dict())
        self.assertEqual(before, [(c.id, c.kind) for c in back.constraints])

    def test_drag_ghost_is_not_a_persistable_kind(self):
        self.assertNotIn("drag_ghost", PERSISTABLE_CONSTRAINT_KINDS)

    def test_no_shipped_plate_file_contains_a_ghost(self):
        """The seven that already reached disk are gone and stay gone."""
        if not USER_PLATES_DIR.exists():
            self.skipTest("no user plates on this machine")
        offenders = []
        for p in sorted(USER_PLATES_DIR.glob("*.json")):
            try:
                blob = json.loads(p.read_text())
            except (OSError, ValueError):
                continue
            for c in blob.get("constraints", []):
                if c.get("kind") == "drag_ghost" or c.get("id", 0) <= 0:
                    offenders.append(f"{p.name}:{c.get('kind')}")
        self.assertEqual([], offenders)


# ═══════════════════════════════════════════════════════════════════
# D1 — a rosette in A1 must not crash calibration
# ═══════════════════════════════════════════════════════════════════

class TestRosetteInA1DoesNotCrash(unittest.TestCase):
    """`compile()` DROPS a well that carries a rosette, replacing it with
    `A1.a`, `A1.b`, … So a plate with a rosette in A1 has no `"A1"` well, and
    `get_well_position("A1")` raises `KeyError` — it never returns None, which
    is why the `is None` guards at the call sites were dead code.
    """

    def _plate_with_rosette_in_a1(self) -> WellPlate:
        design = PlateDesign.from_standard_format(24)
        a1 = next(w for w in design.get_wells() if w.name == "A1")
        ros = PlateDesign.blank_rosette(bore_radius_mm=a1.diameter / 2.0,
                                        name="A1-rosette")
        ros.add_well(0.0, 0.0, 1.5, "a")
        ros.add_well(2.0, 0.0, 1.5, "b")
        a1.rosette_design = ros
        return design.compile()

    def test_compile_drops_the_parent_well(self):
        plate = self._plate_with_rosette_in_a1()
        names = plate.well_names
        self.assertNotIn("A1", names)
        self.assertIn("A1.a", names)
        with self.assertRaises(KeyError):
            plate.get_well_position("A1")

    def test_calculate_alignment_reports_instead_of_raising(self):
        from gui.pages.calibration import CalibrationPage
        _app()
        page = CalibrationPage.__new__(CalibrationPage)
        page._plate = self._plate_with_rosette_in_a1()

        # The helper is the fix: None instead of KeyError.
        self.assertIsNone(page._well_position_mm("A1"))
        self.assertIsNotNone(page._well_position_mm("B2"))

        class _Lbl:
            text = ""

            def setText(self, t):
                self.text = t

        page.lbl_alignment = _Lbl()
        page._taught_a1 = (1000.0, 2000.0)
        page._taught_corner = (50000.0, 40000.0)
        page._corner_well = "D6"

        page._calculate_alignment()          # must not raise
        self.assertIn("Cannot compute alignment", page.lbl_alignment.text)

    def test_helper_tolerates_no_plate(self):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage.__new__(CalibrationPage)
        page._plate = None
        self.assertIsNone(page._well_position_mm("A1"))


# ═══════════════════════════════════════════════════════════════════
# D4 — single-letter shortcuts must not steal keystrokes
# ═══════════════════════════════════════════════════════════════════

class TestShortcutScope(unittest.TestCase):
    """The tool letters were window-scoped QShortcuts, so typing "s" or "g"
    into the plate name — or any field on Hardware Setup — switched tools and
    ate the character.
    """

    def setUp(self):
        self.app = _app()
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        self.host = QWidget()
        lay = QVBoxLayout(self.host)
        self.edit = QLineEdit()
        lay.addWidget(self.edit)
        self.designer = PlateDesignerWidget()
        lay.addWidget(self.designer)
        self.host.show()
        self.app.processEvents()

    def tearDown(self):
        self.host.close()
        self.host.deleteLater()
        self.app.processEvents()

    def test_typing_tool_letters_into_a_field_does_not_switch_tools(self):
        from gui.widgets.plate_designer_canvas import Tool
        canvas = self.designer._canvas
        canvas.set_tool(Tool.SELECT)
        self.edit.setFocus()
        self.app.processEvents()

        for ch in "swgcldk":
            QTest.keyClicks(self.edit, ch)
        self.app.processEvents()

        self.assertEqual("swgcldk", self.edit.text())
        self.assertEqual(Tool.SELECT, canvas._tool)

    def test_tool_letters_still_work_on_the_canvas(self):
        """The veto must not disable the feature it is protecting."""
        from gui.widgets.plate_designer_canvas import Tool
        canvas = self.designer._canvas
        canvas.set_tool(Tool.SELECT)
        canvas.setFocus()
        self.app.processEvents()

        QTest.keyClick(canvas, ord("G"))
        self.app.processEvents()
        self.assertEqual(Tool.DRAW_GRID, canvas._tool)

        QTest.keyClick(canvas, ord("S"))
        self.app.processEvents()
        self.assertEqual(Tool.SELECT, canvas._tool)

    def test_focus_veto_reports_text_widgets(self):
        """`_tool_key_allowed` is the third leg of the fix and is redundant
        while the first (letters on the canvas) holds — so it needs a test of
        its OWN contract, or removing it would go unnoticed.
        """
        from PySide6.QtWidgets import QComboBox, QDoubleSpinBox
        from gui.widgets.plate_designer_canvas import _tool_key_allowed

        self.designer._canvas.setFocus()
        self.app.processEvents()
        self.assertTrue(_tool_key_allowed())

        self.edit.setFocus()
        self.app.processEvents()
        self.assertFalse(_tool_key_allowed())

        spin = QDoubleSpinBox(self.host)
        spin.show()
        spin.setFocus()
        self.app.processEvents()
        self.assertFalse(_tool_key_allowed())

        combo = QComboBox(self.host)
        combo.setEditable(True)
        combo.show()
        combo.setFocus()
        self.app.processEvents()
        self.assertFalse(_tool_key_allowed())

    def test_canvas_key_handler_obeys_the_veto(self):
        """Even delivered straight to the canvas, a tool letter must not fire
        while a text widget holds focus.
        """
        from gui.widgets.plate_designer_canvas import Tool
        canvas = self.designer._canvas
        canvas.set_tool(Tool.SELECT)
        self.edit.setFocus()
        self.app.processEvents()

        QTest.keyClick(canvas, ord("G"))
        self.app.processEvents()
        self.assertEqual(Tool.SELECT, canvas._tool)

    def test_every_remaining_shortcut_is_widget_scoped(self):
        window_scoped = [
            sc.key().toString()
            for sc in self.designer.findChildren(QShortcut)
            if sc.context() == Qt.ShortcutContext.WindowShortcut
        ]
        self.assertEqual([], window_scoped)


# ═══════════════════════════════════════════════════════════════════
# D3 — Save must honour the typed plate name
# ═══════════════════════════════════════════════════════════════════

class TestSavePreservesTypedName(unittest.TestCase):
    """`_on_save` used to do `self._design.name = self._current_key`, throwing
    the typed name away on every Save and making the Name field decorative.
    """

    def test_name_rejection_rules(self):
        _app()
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        d = PlateDesignerWidget()
        try:
            self.assertTrue(d._name_rejection(""))
            self.assertTrue(d._name_rejection("96"))       # standard format
            self.assertTrue(d._name_rejection("24"))
            self.assertEqual("", d._name_rejection("My 24-well glass"))
        finally:
            d.deleteLater()

    def test_typing_a_new_name_and_saving_renames_the_plate(self):
        """The behaviour the operator asked for: type a name, press Save, and
        the plate is called that. Before v7.12 Save discarded it silently.
        """
        from unittest import mock
        from PySide6.QtWidgets import QMessageBox
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        _app()

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            with mock.patch(
                    "gui.pages.hardware.plate_designer.USER_PLATES_DIR", tmp), \
                 mock.patch(
                     "SupportClasses.PlateDesign.USER_PLATES_DIR", tmp):
                design = PlateDesign.from_standard_format(24)
                design.name = "original"
                design.save(tmp / "original.json")

                w = PlateDesignerWidget()
                try:
                    w._design = design
                    w._current_key = "original"
                    # The operator edits the Name field.
                    w._set_design_name("My 24-well glass")
                    self.assertEqual("My 24-well glass", w._design.name)

                    with mock.patch.object(QMessageBox, "question",
                                           return_value=QMessageBox.Yes):
                        w._on_save()

                    # The typed name survived, and IS the plate now.
                    self.assertEqual("My 24-well glass", w._design.name)
                    self.assertEqual("My 24-well glass", w._current_key)
                    self.assertTrue((tmp / "My 24-well glass.json").exists())
                    self.assertFalse((tmp / "original.json").exists())
                    reloaded = PlateDesign.load(tmp / "My 24-well glass.json")
                    self.assertEqual("My 24-well glass", reloaded.name)
                finally:
                    w.deleteLater()

    def test_declining_the_rename_restores_the_file_name(self):
        """Cancelling must not leave the card showing a name nothing has."""
        from unittest import mock
        from PySide6.QtWidgets import QMessageBox
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        _app()

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            with mock.patch(
                    "gui.pages.hardware.plate_designer.USER_PLATES_DIR", tmp), \
                 mock.patch(
                     "SupportClasses.PlateDesign.USER_PLATES_DIR", tmp):
                design = PlateDesign.from_standard_format(24)
                design.name = "original"
                design.save(tmp / "original.json")

                w = PlateDesignerWidget()
                try:
                    w._design = design
                    w._current_key = "original"
                    w._set_design_name("something else")
                    with mock.patch.object(QMessageBox, "question",
                                           return_value=QMessageBox.No):
                        w._on_save()
                    self.assertEqual("original", w._design.name)
                    self.assertEqual("original", w._current_key)
                    self.assertFalse((tmp / "something else.json").exists())
                finally:
                    w.deleteLater()

    def test_saving_an_unchanged_name_is_a_plain_save(self):
        from unittest import mock
        from PySide6.QtWidgets import QMessageBox
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        _app()

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            with mock.patch(
                    "gui.pages.hardware.plate_designer.USER_PLATES_DIR", tmp), \
                 mock.patch(
                     "SupportClasses.PlateDesign.USER_PLATES_DIR", tmp):
                design = PlateDesign.from_standard_format(24)
                design.name = "steady"
                w = PlateDesignerWidget()
                try:
                    w._design = design
                    w._current_key = "steady"
                    with mock.patch.object(QMessageBox, "question") as q:
                        w._on_save()
                    q.assert_not_called()      # no rename prompt
                    self.assertEqual("steady", w._design.name)
                    self.assertTrue((tmp / "steady.json").exists())
                finally:
                    w.deleteLater()


# ═══════════════════════════════════════════════════════════════════
# D6 — a learned plate-type override must be removable
# ═══════════════════════════════════════════════════════════════════

class TestPlateTypeDeleteUser(unittest.TestCase):
    def test_delete_user_resurfaces_the_builtin(self):
        from SupportClasses.PlateTypeStore import PlateType, PlateTypeStore
        with tempfile.TemporaryDirectory() as d:
            builtin = Path(d) / "builtin"
            user = Path(d) / "user"
            builtin.mkdir()
            user.mkdir()
            (builtin / "acme-24.json").write_text(json.dumps({
                "id": "acme-24", "base_format": 24,
                "display_name": "ACME 24", "builtin": True,
                "z_offsets": {"top": 1.0, "bottom": 2.0,
                              "safe": 3.0, "max": 4.0},
            }))
            store = PlateTypeStore(builtin_dir=builtin, user_dir=user)
            self.assertEqual(1.0, store.get("acme-24").z_offsets["top"])

            learned = PlateType(id="acme-24", base_format=24,
                                display_name="ACME 24",
                                z_offsets={"top": 9.9, "bottom": 2.0,
                                           "safe": 3.0, "max": 4.0})
            self.assertTrue(store.save_user(learned))
            self.assertEqual(9.9, store.get("acme-24").z_offsets["top"])

            self.assertTrue(store.delete_user("acme-24"))
            self.assertEqual(1.0, store.get("acme-24").z_offsets["top"])
            self.assertTrue(store.get("acme-24").builtin)

            # Idempotent; built-ins are never removed.
            self.assertFalse(store.delete_user("acme-24"))
            self.assertFalse(store.delete_user(""))


# ═══════════════════════════════════════════════════════════════════
# D9 — sub-page titles / deep links must follow the real tab order
# ═══════════════════════════════════════════════════════════════════

class TestSubPageTitles(unittest.TestCase):
    """The label list had nine entries for ten tabs, in the wrong order from
    index 3 on, so most tabs reported the wrong name and onboarding's
    `switch_to(3)` — commented "3=Pump" — actually landed on Rosette.
    """

    def setUp(self):
        self.app = _app()
        from gui.pages.hardware_setup import HardwareSetupPage
        self.page = HardwareSetupPage()

    def tearDown(self):
        self.page.deleteLater()
        self.app.processEvents()

    def test_every_tab_reports_its_own_name(self):
        self.assertEqual(len(self.page._sub_pages),
                         len(self.page._sub_titles))
        for i, title in enumerate(self.page._sub_titles):
            self.page.switch_to(i)
            self.assertEqual(f"Hardware: {title}",
                             self.page.get_sub_page_title())

    def test_pump_deep_link_resolves_by_name(self):
        idx = self.page.sub_page_index("Pump")
        self.assertGreaterEqual(idx, 0)
        self.assertEqual("Pump", self.page._sub_titles[idx])
        self.assertEqual(-1, self.page.sub_page_index("Nonexistent"))

    def test_onboarding_no_longer_hardcodes_a_stale_index(self):
        import inspect
        from gui.app import MainWindow
        src = inspect.getsource(MainWindow._on_onboarding_completed)
        self.assertIn('sub_page_index("Pump")', src)
        self.assertNotIn("hw_page.switch_to(3)", src)


# ═══════════════════════════════════════════════════════════════════
# D5 — geometry edits must dirty the config
# ═══════════════════════════════════════════════════════════════════

class TestInvalidationCoversPlateIdentity(unittest.TestCase):
    """`_emit_invalidation` diffed only `plate_format` + `pumps`, so switching
    between two custom plates — or two products of the same base format — never
    invalidated anything, even though every per-plate store keys off exactly
    the fields that changed.
    """

    class _Cfg:
        def __init__(self, fmt=24, name="", type_id="", doc_id=""):
            self.plate_format = fmt
            self.plate_name = name
            self.plate_type_id = type_id
            self.plate_doc_id = doc_id
            self.pumps = {}

    def _emit(self, prev, cur):
        from gui.app import MainWindow
        seen = {}

        class _Sig:
            @staticmethod
            def emit(changed):
                seen.update(changed)

        win = MainWindow.__new__(MainWindow)
        win.hw_config_invalidated = _Sig()
        win._show_invalidation_banner = lambda changed: None
        MainWindow._emit_invalidation(win, prev, cur)
        return seen

    def test_plate_name_change_invalidates(self):
        changed = self._emit(self._Cfg(name="plate-a"),
                             self._Cfg(name="plate-b"))
        self.assertIn("plate_name", changed)

    def test_plate_type_change_invalidates(self):
        changed = self._emit(self._Cfg(type_id="corning-glass-24"),
                             self._Cfg(type_id="nest-plastic-24"))
        self.assertIn("plate_type_id", changed)

    def test_plate_doc_change_invalidates(self):
        """v7.12: `plate_doc_id` is what the per-plate stores key on for a
        parametric design, and two plates may legitimately share a display
        name — so watching `plate_name` alone would let a switch between them
        pass unnoticed and leave the calibration page on the old plate's
        taught wells."""
        changed = self._emit(self._Cfg(name="Same", doc_id="plt_aaa"),
                             self._Cfg(name="Same", doc_id="plt_bbb"))
        self.assertIn("plate_doc_id", changed)

    def test_identical_config_invalidates_nothing(self):
        cfg = self._Cfg(name="same", type_id="t")
        self.assertEqual({}, self._emit(cfg, self._Cfg(name="same", type_id="t")))

    def test_plate_edits_are_wired_to_the_config(self):
        """Editing the plate must reach `_on_config_changed`.

        In Phase 0 this was the plate designer's unconnected `design_edited`.
        Phase 5 replaced that surface with the library workspace, so the
        assertion follows the wiring rather than the widget.
        """
        import inspect
        from gui.pages.hardware_setup import HardwareSetupPage
        src = inspect.getsource(HardwareSetupPage)
        self.assertIn("_plate_workspace.library_changed.connect", src)
        self.assertIn("_plate_workspace.active_plate_changed.connect", src)
        self.assertIn("_rosette_placement.placements_changed.connect", src)


# ═══════════════════════════════════════════════════════════════════
# D7 / D8 — canvas hygiene
# ═══════════════════════════════════════════════════════════════════

class TestCanvasHygiene(unittest.TestCase):
    def setUp(self):
        self.app = _app()
        from gui.widgets.plate_designer_canvas import PlateDesignerCanvas
        self.canvas = PlateDesignerCanvas()
        self.canvas.set_design(PlateDesign.from_standard_format(24))

    def tearDown(self):
        self.canvas.deleteLater()
        self.app.processEvents()

    def test_zoom_is_clamped_in_both_directions(self):
        from PySide6.QtCore import QPoint, QPointF
        from PySide6.QtGui import QWheelEvent

        def wheel(delta):
            ev = QWheelEvent(
                QPointF(10, 10), QPointF(10, 10), QPoint(0, 0),
                QPoint(0, delta), Qt.MouseButton.NoButton,
                Qt.KeyboardModifier.NoModifier,
                Qt.ScrollPhase.NoScrollPhase, False)
            self.canvas.wheelEvent(ev)

        for _ in range(200):
            wheel(120)
        self.assertLessEqual(self.canvas.transform().m11(),
                             self.canvas._ZOOM_MAX + 1e-6)
        for _ in range(400):
            wheel(-120)
        self.assertGreaterEqual(self.canvas.transform().m11(),
                                self.canvas._ZOOM_MIN - 1e-6)

    def test_switching_tool_ends_an_in_flight_drag(self):
        """An orphaned drag left a live weight-1000 pin on the design."""
        from gui.widgets.plate_designer_canvas import Tool
        well = self.canvas.design.get_wells()[0]
        self.canvas._dragging_id = well.id
        self.canvas.solver.begin_drag(well.center, (5.0, 5.0))
        self.assertTrue(
            any(c.kind == "drag_ghost" for c in self.canvas.design.constraints))

        self.canvas.set_tool(Tool.DRAW_GRID)

        self.assertIsNone(self.canvas._dragging_id)
        self.assertEqual(
            [], [c for c in self.canvas.design.constraints
                 if c.kind == "drag_ghost"])

    def test_escape_also_ends_an_in_flight_drag(self):
        well = self.canvas.design.get_wells()[0]
        self.canvas._dragging_id = well.id
        self.canvas.solver.begin_drag(well.center, (5.0, 5.0))
        self.canvas._cancel_tool()
        self.assertIsNone(self.canvas._dragging_id)
        self.assertEqual(
            [], [c for c in self.canvas.design.constraints
                 if c.kind == "drag_ghost"])

    def test_dead_add_constraint_path_is_gone(self):
        from gui.widgets.plate_designer_canvas import PlateDesignerCanvas, Tool
        self.assertFalse(hasattr(Tool, "ADD_CONSTRAINT"))
        for attr in ("begin_add_constraint", "_handle_add_constraint_pick",
                     "_enough_picks_for_kind", "_commit_pending_constraint"):
            self.assertFalse(hasattr(PlateDesignerCanvas, attr), attr)

    def test_minor_grid_is_skipped_when_it_would_be_a_wash(self):
        import inspect
        from gui.widgets.plate_designer_canvas import PlateDesignerCanvas
        src = inspect.getsource(PlateDesignerCanvas.drawBackground)
        self.assertIn("_GRID_MIN_DEVICE_PX", src)


if __name__ == "__main__":
    unittest.main()
