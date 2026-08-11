"""v7.12 Phases 2-4 — library, builder and rosette placement, offscreen.

Everything here builds with nothing but a ``QApplication``: no hardware, no
camera, no modal in a constructor or a teardown path.

The load-bearing assertions are the ones tied to what the operator actually
asked for:

* the ring's rotate handle changes the ANGLE and not the size, and its radius
  handle the size and not the angle (one conflated handle could not orient
  without resizing — the complaint);
* a pattern member click selects its PATTERN, so the visible controls are the
  ones that drive the geometry;
* switching the origin changes the readout and nothing else;
* both rosette placement gestures reach the same document state.
"""
from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF, Qt  # noqa: E402
from PySide6.QtWidgets import QApplication, QLineEdit  # noqa: E402

from gui.pages.hardware.plate_library import (  # noqa: E402
    PlateLibraryPage, is_standard, standard_id,
)
from gui.pages.hardware.plate_workspace import PlateWorkspacePage  # noqa: E402
from gui.pages.hardware.rosette_placement import (  # noqa: E402
    RosettePlacementPage,
)
from gui.widgets.plate_canvas_gizmos import (  # noqa: E402
    AnchorMarkerItem, RadiusHandleItem, RotationHandleItem, SeedHandleItem,
)
from gui.widgets.plate_document_canvas import (  # noqa: E402
    SCALE, PlateDocumentCanvas, Tool,
)
from SupportClasses.PlateDocument import (  # noqa: E402
    NamingScheme, OriginRef, PlateDocument, WellStyle,
)
from SupportClasses.PlateDocumentStore import PlateDocumentStore  # noqa: E402

NO_MOD = Qt.KeyboardModifier.NoModifier


def _app() -> QApplication:
    return QApplication.instance() or QApplication([])


class _TmpStores(unittest.TestCase):
    def setUp(self):
        self.app = _app()
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.plates = PlateDocumentStore(kind="plate", user_dir=root / "p")
        self.rosettes = PlateDocumentStore(kind="rosette", user_dir=root / "r")

    def tearDown(self):
        self.app.processEvents()
        self._tmp.cleanup()

    def _rosette(self, name="Ring of 3"):
        doc = self.rosettes.create(name)
        doc.add_ring(0.0, 0.0, count=3, ring_diameter_mm=6.0,
                     style=WellStyle(diameter_mm=1.5, well_depth_mm=20.0,
                                     rim_height_mm=3.0))
        self.rosettes.save(doc)
        return doc


# ═══════════════════════════════════════════════════════════════════
# Phase 2 — library
# ═══════════════════════════════════════════════════════════════════

class TestPlateLibrary(_TmpStores):
    def test_standards_are_listed_read_only(self):
        page = PlateLibraryPage(kind="plate", store=self.plates)
        std = [c for c in page._cards if is_standard(c.doc_id())]
        self.assertEqual(6, len(std))
        for c in std:
            self.assertTrue(c._read_only)
            self.assertFalse(c._check.isEnabled())

    def test_user_plates_appear_alongside(self):
        doc = self.plates.create("Mine", template=24)
        page = PlateLibraryPage(kind="plate", store=self.plates)
        self.assertIn(doc.meta.id, [c.doc_id() for c in page._cards])

    def test_thumbnail_needs_no_hardware_config(self):
        """A plate design is already geometry — unlike a print thumbnail, which
        needs a needle and syringes. Pinned so nobody wires one in."""
        doc = self.plates.create("Thumb", template=96)
        page = PlateLibraryPage(kind="plate", store=self.plates)
        card = next(c for c in page._cards if c.doc_id() == doc.meta.id)
        self.assertEqual(96, len(card.thumbnail()._wells))
        self.assertFalse(hasattr(page, "_hw_config"))
        self.assertFalse(hasattr(page, "set_hardware_config"))

    def test_thumbnail_does_not_flip_y(self):
        """Plate space is Y-DOWN. Copying PrintThumbnail's Y-up flip would
        paint every plate upside-down with A1 in the wrong corner."""
        import inspect
        from gui.pages.hardware.plate_library import PlateThumbnail
        src = inspect.getsource(PlateThumbnail.paintEvent)
        self.assertIn("+ (y - cy) * scale", src)
        self.assertNotIn("- (y - cy) * scale", src)

    def test_active_plate_gets_a_star(self):
        from PySide6.QtWidgets import QLabel
        doc = self.plates.create("Active one", template=6)
        other = self.plates.create("Not active", template=6)
        page = PlateLibraryPage(kind="plate", store=self.plates)
        page.set_active_id(doc.meta.id)

        def starred(doc_id: str) -> bool:
            card = next(c for c in page._cards if c.doc_id() == doc_id)
            return any("★" in lbl.text() for lbl in card.findChildren(QLabel))

        self.assertTrue(starred(doc.meta.id))
        self.assertFalse(starred(other.meta.id))

    def test_activating_a_card_reports_the_id(self):
        doc = self.plates.create("Pick me", template=6)
        page = PlateLibraryPage(kind="plate", store=self.plates)
        seen: list[str] = []
        page.active_changed.connect(seen.append)
        page._on_activate(doc.meta.id)
        self.assertEqual([doc.meta.id], seen)
        self.assertEqual(doc.meta.id, page.active_id())

    def test_cards_carry_ids_not_names(self):
        """A rename must not change which plate an action targets."""
        doc = self.plates.create("Before", template=6)
        page = PlateLibraryPage(kind="plate", store=self.plates)
        seen = []
        page.open_requested.connect(seen.append)
        card = next(c for c in page._cards if c.doc_id() == doc.meta.id)
        card.open_requested.emit(card.doc_id())
        self.assertEqual([doc.meta.id], seen)

    def test_grid_reflows_with_width(self):
        for i in range(8):
            self.plates.create(f"P{i}", template=6)
        page = PlateLibraryPage(kind="plate", store=self.plates)
        page.resize(900, 600)
        page.show()
        self.app.processEvents()
        page._grid._relayout(force=True)
        wide = page._grid.columns()
        page.resize(360, 600)
        self.app.processEvents()
        page._grid._relayout(force=True)
        self.assertGreater(wide, page._grid.columns())
        self.assertGreaterEqual(page._grid.columns(), 1)

    def test_empty_user_library_is_still_usable(self):
        page = PlateLibraryPage(kind="plate", store=self.plates)
        self.assertTrue(page._cards)          # standards carry it

    def test_rosette_library_is_the_same_class(self):
        self._rosette()
        page = PlateLibraryPage(kind="rosette", store=self.rosettes)
        self.assertEqual(1, len(page._cards))
        self.assertFalse(any(is_standard(c.doc_id()) for c in page._cards))


# ═══════════════════════════════════════════════════════════════════
# Phase 3 — builder canvas
# ═══════════════════════════════════════════════════════════════════

class TestBuilderCanvas(_TmpStores):
    def setUp(self):
        super().setUp()
        self.canvas = PlateDocumentCanvas()
        self.doc = PlateDocument.from_standard_format(24)
        self.canvas.set_document(self.doc)
        self.canvas.resize(800, 600)
        self.canvas.show()
        self.app.processEvents()

    def tearDown(self):
        self.canvas.deleteLater()
        super().tearDown()

    # ── the operator's request 2 ──────────────────────────────────

    def test_ring_gets_separate_radius_and_rotation_handles(self):
        ring = self.doc.add_ring(50.0, 30.0, count=6, ring_diameter_mm=10.0)
        self.canvas.rebuild()
        self.canvas.select_refs([(ring.id, "")])
        kinds = {type(i).__name__ for i in self.canvas.scene().items()}
        self.assertIn("RadiusHandleItem", kinds)
        self.assertIn("RotationHandleItem", kinds)
        self.assertIn("SeedHandleItem", kinds)

    def test_rotating_a_ring_does_not_resize_it(self):
        ring = self.doc.add_ring(50.0, 30.0, count=6, ring_diameter_mm=10.0)
        self.canvas.rebuild()
        self.canvas.select_refs([(ring.id, "")])
        # +X of the anchor is 90 degrees in the 0-deg-is-+Y convention.
        self.canvas._apply_gizmo(("rotate", ring.id), (72.0, 30.0), NO_MOD)
        self.assertAlmostEqual(10.0, ring.ring_diameter_mm, places=9)
        self.assertAlmostEqual(90.0, ring.start_angle_deg, places=6)

    def test_resizing_a_ring_does_not_rotate_it(self):
        ring = self.doc.add_ring(50.0, 30.0, count=6, ring_diameter_mm=10.0,
                                 start_angle_deg=37.0)
        self.canvas.rebuild()
        self.canvas.select_refs([(ring.id, "")])
        self.canvas._apply_gizmo(("radius", ring.id), (50.0, 38.0), NO_MOD)
        self.assertAlmostEqual(16.0, ring.ring_diameter_mm, places=9)
        self.assertAlmostEqual(37.0, ring.start_angle_deg, places=9)

    def test_ring_tool_click_then_drag_creates_one(self):
        before = len(self.doc.patterns())
        self.canvas.set_tool(Tool.RING)
        self.canvas._pending_pt = (40.0, 20.0)
        from PySide6.QtGui import QMouseEvent
        from PySide6.QtCore import QEvent
        pos = self.canvas.mapFromScene(QPointF(40.0 * 6.0, 28.0 * 6.0))
        ev = QMouseEvent(QEvent.MouseButtonRelease, QPointF(pos),
                         Qt.LeftButton, Qt.LeftButton, NO_MOD)
        self.canvas.mouseReleaseEvent(ev)
        self.assertEqual(before + 1, len(self.doc.patterns()))

    # ── the operator's request 3 ──────────────────────────────────

    def test_grid_gets_a_rotation_and_seed_handle(self):
        grid = self.doc.patterns()[0]
        self.canvas.select_refs([(grid.id, "")])
        kinds = {type(i).__name__ for i in self.canvas.scene().items()}
        self.assertIn("RotationHandleItem", kinds)
        self.assertIn("SeedHandleItem", kinds)
        self.assertNotIn("RadiusHandleItem", kinds)

    def test_grid_seed_drag_moves_the_whole_pattern(self):
        grid = self.doc.patterns()[0]
        self.canvas.select_refs([(grid.id, "")])
        before = [(w.x, w.y) for w in self.doc.evaluate()]
        self.canvas._apply_gizmo(("seed", grid.id), (10.0, 5.0), NO_MOD)
        after = [(w.x, w.y) for w in self.doc.evaluate()]
        deltas = {(round(a[0] - b[0], 9), round(a[1] - b[1], 9))
                  for a, b in zip(after, before)}
        self.assertEqual(1, len(deltas))         # rigid translation

    # ── selection ─────────────────────────────────────────────────

    def test_selection_has_one_mutator(self):
        import inspect
        src = inspect.getsource(PlateDocumentCanvas)
        self.assertEqual(1, src.count("def _set_selection"))
        self.assertNotIn("self._selection =", src.split(
            "def _set_selection")[0])

    def test_marquee_selects_patterns_not_members(self):
        grid = self.doc.patterns()[0]
        self.canvas._marquee_from = QPointF(-500, -500)
        from PySide6.QtGui import QMouseEvent
        from PySide6.QtCore import QEvent
        ev = QMouseEvent(QEvent.MouseButtonRelease, QPointF(0, 0),
                         Qt.LeftButton, Qt.LeftButton, NO_MOD)
        self.canvas._commit_marquee(QPointF(5000, 5000), ev)
        self.assertEqual([(grid.id, "")], self.canvas.selection())

    def test_pattern_members_are_not_individually_draggable(self):
        """Anchor-only dragging — parameters and geometry can never drift."""
        import inspect
        src = inspect.getsource(PlateDocumentCanvas._press_select)
        self.assertIn("if not item.ref[1]:", src)

    def _click(self, scene_xy, modifiers=NO_MOD):
        from PySide6.QtCore import QEvent
        from PySide6.QtGui import QMouseEvent
        pos = self.canvas.mapFromScene(QPointF(scene_xy[0] * 6.0,
                                               scene_xy[1] * 6.0))
        ev = QMouseEvent(QEvent.MouseButtonPress, QPointF(pos),
                         Qt.LeftButton, Qt.LeftButton, modifiers)
        self.canvas.mousePressEvent(ev)

    def test_clicking_a_pattern_member_selects_the_pattern(self):
        """Otherwise the panel offers X/Y spin boxes that write a point the
        next evaluation overwrites — the old designer's silent revert."""
        grid = self.doc.patterns()[0]
        self.canvas.fit_view()
        self.app.processEvents()
        a2 = next(w for w in self.doc.evaluate() if w.name == "A2")
        self._click((a2.x, a2.y))
        self.assertEqual([(grid.id, "")], self.canvas.selection())

    def test_alt_click_reaches_the_individual_member(self):
        grid = self.doc.patterns()[0]
        self.canvas.fit_view()
        self.app.processEvents()
        a2 = next(w for w in self.doc.evaluate() if w.name == "A2")
        self._click((a2.x, a2.y), Qt.KeyboardModifier.AltModifier)
        self.assertEqual([a2.key], self.canvas.selection())
        self.assertEqual("g0_1", self.canvas.selection()[0][1])

    # ── the operator's request 1 ──────────────────────────────────

    def test_origin_changes_the_readout_only(self):
        seen: list[tuple] = []
        self.canvas.hover_moved.connect(lambda x, y: seen.append((x, y)))
        blob = self.doc.to_dict()

        self.doc.boundary.origin_ref = OriginRef.A1
        a1_reading = self.doc.to_display(0.0, 0.0)
        self.doc.boundary.origin_ref = OriginRef.BOTTOM_LEFT
        bl_reading = self.doc.to_display(0.0, 0.0)

        self.assertNotEqual(a1_reading, bl_reading)
        after = self.doc.to_dict()
        blob["boundary"].pop("origin_ref")
        after["boundary"].pop("origin_ref")
        self.assertEqual(blob, after)

    # ── hygiene carried forward ───────────────────────────────────

    def test_no_scene_embedded_input_widgets(self):
        """QGraphicsProxyWidget + QAbstractSpinBox segfaults on key input."""
        from PySide6.QtWidgets import QGraphicsProxyWidget
        ring = self.doc.add_ring(50.0, 30.0, count=6, ring_diameter_mm=10.0)
        self.doc.add_constraint("distance_to_datum",
                                [(self.doc.patterns()[0].id, "g0_0")],
                                value=17.0, datum="edge_left")
        self.canvas.rebuild()
        self.canvas.select_refs([(ring.id, "")])
        self.assertEqual([], [i for i in self.canvas.scene().items()
                              if isinstance(i, QGraphicsProxyWidget)])

    def test_zoom_is_clamped(self):
        from PySide6.QtCore import QPoint
        from PySide6.QtGui import QWheelEvent
        for delta in (120, -120):
            for _ in range(300):
                self.canvas.wheelEvent(QWheelEvent(
                    QPointF(10, 10), QPointF(10, 10), QPoint(0, 0),
                    QPoint(0, delta), Qt.NoButton, NO_MOD,
                    Qt.ScrollPhase.NoScrollPhase, False))
            z = self.canvas.transform().m11()
            self.assertGreaterEqual(z, 0.05 - 1e-6)
            self.assertLessEqual(z, 40.0 + 1e-6)

    def test_tool_keys_do_not_reach_a_focused_field(self):
        from PySide6.QtTest import QTest
        from PySide6.QtWidgets import QVBoxLayout, QWidget
        host = QWidget()
        lay = QVBoxLayout(host)
        edit = QLineEdit()
        lay.addWidget(edit)
        canvas = PlateDocumentCanvas()
        canvas.set_document(PlateDocument.from_standard_format(6))
        lay.addWidget(canvas)
        host.show()
        self.app.processEvents()
        canvas.set_tool(Tool.SELECT)
        edit.setFocus()
        self.app.processEvents()
        for ch in "swcgld":
            QTest.keyClicks(edit, ch)
        self.app.processEvents()
        self.assertEqual("swcgld", edit.text())
        self.assertEqual(Tool.SELECT, canvas.tool())
        host.close()

    def test_undo_restores_and_keeps_references_valid(self):
        before = len(self.doc.evaluate())
        self.canvas.snapshot()
        self.doc.add_well(5.0, 5.0, "ZZ")
        self.canvas.rebuild()
        self.assertEqual(before + 1, len(self.doc.evaluate()))
        self.canvas.undo()
        self.assertEqual(before, len(self.doc.evaluate()))
        self.assertIs(self.doc, self.canvas.document)   # same object


# ═══════════════════════════════════════════════════════════════════
# Phase 3 — workspace + builder page
# ═══════════════════════════════════════════════════════════════════

class TestWorkspace(_TmpStores):
    def setUp(self):
        super().setUp()
        self.ws = PlateWorkspacePage(plate_store=self.plates,
                                     rosette_store=self.rosettes)
        self.ws.resize(1100, 700)
        self.ws.show()
        self.app.processEvents()

    def tearDown(self):
        self.ws.deleteLater()
        super().tearDown()

    def test_opens_as_the_library(self):
        self.assertEqual(0, self.ws._stack.currentIndex())
        self.assertEqual("plate", self.ws.mode())

    def test_opening_a_standard_forks_it(self):
        self.ws._open("plate", standard_id(24))
        b = self.ws.builder()
        self.assertEqual(1, self.ws._stack.currentIndex())
        self.assertTrue(b.is_dirty())
        self.assertTrue(b._fork_note.isVisible())
        self.assertEqual(24, len(b.document().evaluate()))

    def test_saving_a_fork_creates_a_user_plate(self):
        self.ws._open("plate", standard_id(24))
        self.assertTrue(self.ws.builder().save())
        self.assertFalse(self.ws.builder().is_dirty())
        self.assertEqual(1, len(self.plates.list()))

    def test_mode_pill_switches_collection(self):
        self._rosette()
        self.ws._on_mode("rosette")
        self.assertEqual("rosette", self.ws.mode())
        self.assertEqual(0, self.ws._stack.currentIndex())
        self.assertEqual(1, len(self.ws.library("rosette")._cards))

    def test_one_builder_serves_both_kinds(self):
        """No `mode=` argument — the builder reads the boundary."""
        import inspect
        from gui.pages.hardware.plate_builder import PlateBuilderPage
        sig = inspect.signature(PlateBuilderPage.__init__)
        self.assertNotIn("mode", sig.parameters)
        ros = self._rosette()
        self.ws._open("rosette", ros.meta.id)
        self.assertTrue(self.ws.builder().document().boundary.is_circle())

    def test_builder_has_no_save_as(self):
        """Duplicate lives in the library; having both is what produced
        'Save behaves as Save As when the key is an int'."""
        from gui.pages.hardware.plate_builder import PlateBuilderPage
        self.assertFalse(hasattr(PlateBuilderPage, "_on_save_as"))

    def test_title_edit_renames_the_document(self):
        self.ws._open("plate", standard_id(6))
        b = self.ws.builder()
        b._title.setText("Renamed in the header")
        b._on_title_edited()
        self.assertEqual("Renamed in the header", b.document().meta.name)


# ═══════════════════════════════════════════════════════════════════
# Phase 4 — rosette placement
# ═══════════════════════════════════════════════════════════════════

class TestRosettePlacementPage(_TmpStores):
    def setUp(self):
        super().setUp()
        self.ros = self._rosette()
        self.plate = self.plates.create("Bench plate", template=24)
        self.page = RosettePlacementPage(plate_store=self.plates,
                                         rosette_store=self.rosettes)
        self.page.set_plate(self.plate)
        self.page.resize(1100, 700)
        self.page.show()
        self.app.processEvents()
        self.grid = self.plate.patterns()[0]

    def tearDown(self):
        self.page.deleteLater()
        super().tearDown()

    def test_palette_lists_the_rosette_library(self):
        self.assertEqual(1, len(self.page._chips))
        self.assertEqual(self.ros.meta.id, self.page._chips[0].doc_id())

    def test_stamp_gesture_places_and_stays_armed(self):
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0")])
        self.page._stamp([(self.grid.id, "g0_1")])
        self.assertEqual(2, len(self.plate.placements()))
        self.assertEqual(self.ros.meta.id, self.page._armed)   # still armed

    def test_multi_select_assign_gesture(self):
        self.page._arm("")
        self.page._canvas.select_refs([(self.grid.id, "g1_0"),
                                       (self.grid.id, "g1_1")])
        self.page._assign_combo.setCurrentIndex(0)
        self.page._assign_selected()
        self.assertEqual(2, len(self.plate.placements()))

    def test_both_gestures_reach_the_same_state(self):
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0")])
        stamped = dict(self.plate.placements())[(self.grid.id, "g0_0")]
        self.page._arm("")
        self.page._canvas.select_refs([(self.grid.id, "g0_1")])
        self.page._assign_selected()
        assigned = dict(self.plate.placements())[(self.grid.id, "g0_1")]
        self.assertEqual(stamped.rosette_id, assigned.rosette_id)

    def test_rotation_applies_to_the_selection(self):
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0")])
        self.page._canvas.select_refs([(self.grid.id, "g0_0")])
        self.page._rot_spin.setValue(45.0)
        self.page._rotate_selected()
        self.assertEqual(
            45.0, dict(self.plate.placements())[(self.grid.id, "g0_0")]
            .rotation_deg)

    def test_placement_compiles_to_subwells(self):
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0")])
        plate = self.plate.compile(loader=self.rosettes.get)
        self.assertNotIn("A1", plate.well_names)
        self.assertIn("A1.a", plate.well_names)

    def test_clearing_removes_the_placement(self):
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0")])
        self.page._clear_one((self.grid.id, "g0_0"))
        self.assertEqual([], self.plate.placements())

    def test_editing_a_rosette_asks_the_host_to_open_its_design(self):
        """Rosettes are documents now — the old gesture opened an anonymous
        nested design in place."""
        seen: list[str] = []
        self.page.edit_rosette_requested.connect(seen.append)
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0")])
        item = self.page._table.item(0, 1)
        self.page._on_table_double(item)
        self.assertEqual([self.ros.meta.id], seen)

    def test_a_live_edit_of_the_rosette_reaches_every_plate(self):
        self.page._arm(self.ros.meta.id)
        self.page._stamp([(self.grid.id, "g0_0"), (self.grid.id, "g0_1")])
        before = self.plate.compile(
            loader=self.rosettes.get).get_well_position("A1.a")
        self.ros.patterns()[0].ring_diameter_mm = 11.0
        self.rosettes.save(self.ros)
        self.rosettes.invalidate()
        after = self.plate.compile(
            loader=self.rosettes.get).get_well_position("A1.a")
        self.assertNotEqual(before, after)

    def test_a_missing_rosette_is_flagged_in_the_table(self):
        self.plate.place_rosette((self.grid.id, "g0_0"), "ros_gone",
                                 rosette_name="Vanished")
        self.page._reload_table()
        self.assertIn("missing", self.page._table.item(0, 1).text())


# ═══════════════════════════════════════════════════════════════════
# Phase 5 — host wiring
# ═══════════════════════════════════════════════════════════════════

class TestHardwareSetupWiring(unittest.TestCase):
    """NOTE the store isolation. ``HardwareSetupPage`` builds its own
    ``PlateDocumentStore`` from ``MEBP_PLATES_DIR``, so a test that creates a
    plate through the page writes into the OPERATOR'S real library — nine junk
    plates ("Wired plate", "Bench plate"…) accumulated there before this was
    isolated, and they showed up as cards on their Plate tab."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._env = {k: os.environ.get(k)
                     for k in ("MEBP_PLATES_DIR", "MEBP_ROSETTES_DIR")}
        os.environ["MEBP_PLATES_DIR"] = str(Path(self._tmp.name) / "p")
        from gui.pages.hardware_setup import HardwareSetupPage
        self.page = HardwareSetupPage()

    def tearDown(self):
        self.page.deleteLater()
        self.app.processEvents()
        for k, v in self._env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        self._tmp.cleanup()

    def test_tabs_are_plate_and_layout(self):
        self.assertEqual("Plate", self.page._sub_titles[2])
        self.assertEqual("Layout", self.page._sub_titles[3])
        self.assertEqual(10, len(self.page._sub_titles))

    def test_retired_surfaces_are_gone(self):
        for attr in ("_plate_designer", "_rosette_designer",
                     "_plate_type_combo", "_plate_type_format_combo",
                     "_selected_plate_type_id", "rosette_table"):
            self.assertFalse(hasattr(self.page, attr), attr)
        import gui.pages.hardware_setup as hs
        self.assertFalse(hasattr(hs, "RosetteEditorDialog"))

    def test_every_tab_can_be_shown(self):
        for i in range(len(self.page._sub_titles)):
            self.page.switch_to(i)
        self.app.processEvents()

    def test_layout_tab_follows_the_active_plate(self):
        store = self.page._plate_workspace.plate_store()
        doc = store.create("Wired plate", template=24)
        self.page._on_active_plate_changed(doc.meta.id)
        self.page._on_hw_sub_page_changed(self.page._rosette_sub_index)
        self.assertIs(doc, self.page._rosette_placement._doc)


class TestPatternAnchorIsDimensionable(_TmpStores):
    """Operator, after Phase 5: *"the grids and ring of wells objects have no
    way to dimension the reference point to the edge of the plate."*

    The solver could always satisfy ``distance_to_datum`` on a pattern anchor;
    what did not exist was any way to CLICK one. A ring with ``center_well``
    off has no well at its centre, so the canvas drew nothing there and
    ``_dim_target`` — which only hit-tested wells — returned None.
    """

    def setUp(self):
        super().setUp()
        self.canvas = PlateDocumentCanvas()
        self.doc = PlateDocument.new_plate("Anchors")
        self.canvas.set_document(self.doc)
        self.canvas.resize(800, 600)
        self.canvas.show()
        self.app.processEvents()

    def tearDown(self):
        self.canvas.deleteLater()
        super().tearDown()

    def _sp(self, x_mm, y_mm) -> QPointF:
        return QPointF(x_mm * SCALE, y_mm * SCALE)

    def _edges(self):
        return self.doc.boundary.extent_a1()

    def test_ring_without_a_centre_well_still_has_a_marker(self):
        ring = self.doc.add_ring(30.0, 30.0, count=6, ring_diameter_mm=12.0)
        ring.center_well = False
        self.canvas.rebuild()
        marks = [i for i in self.canvas.scene().items()
                 if isinstance(i, AnchorMarkerItem)]
        self.assertEqual(1, len(marks))
        self.assertEqual((ring.id, ""), marks[0].ref)

    def test_ring_centre_is_a_dimension_target(self):
        ring = self.doc.add_ring(30.0, 30.0, count=6, ring_diameter_mm=12.0)
        ring.center_well = False
        self.canvas.rebuild()
        self.assertEqual(("ref", (ring.id, "")),
                         self.canvas._dim_target(self._sp(30.0, 30.0)))

    def test_grid_seed_picks_the_anchor_not_the_member(self):
        """The seed WELL is coincident with the anchor, but its member key
        (``g0_0``) changes when the A1-corner direction flips, so a dimension
        hung on it would silently retarget. The anchor is the stable ref."""
        grid = self.doc.add_grid(20.0, 15.0, rows=3, cols=4,
                                 pitch_x_mm=19.3, pitch_y_mm=19.3)
        self.canvas.rebuild()
        kind, ref = self.canvas._dim_target(self._sp(20.0, 15.0))
        self.assertEqual("ref", kind)
        self.assertEqual((grid.id, ""), ref)

    def test_dimensioning_a_ring_centre_to_an_edge_drives_it(self):
        ring = self.doc.add_ring(30.0, 30.0, count=6, ring_diameter_mm=12.0)
        ring.center_well = False
        self.canvas.rebuild()
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self._edges()[0]
        self.canvas._dimension_pick(self._sp(30.0, 30.0))
        self.canvas._dimension_pick(self._sp(x0, 30.0))

        cons = [c for c in self.doc.constraints
                if c.kind == "distance_to_datum"]
        self.assertEqual(1, len(cons))
        self.assertEqual([(ring.id, "")], list(cons[0].refs))
        self.assertEqual("edge_left", cons[0].datum)
        # Seeded from current geometry, so adding it moves nothing.
        self.assertAlmostEqual(30.0 - x0, cons[0].value, places=3)

        cons[0].value = 20.0
        self.canvas.solve()
        anchor = self.doc.entities[self.doc.entities[ring.id].anchor]
        self.assertAlmostEqual(20.0, anchor.x - x0, places=3)

    def test_the_whole_ring_translates_with_its_anchor(self):
        ring = self.doc.add_ring(30.0, 30.0, count=6, ring_diameter_mm=12.0)
        self.canvas.rebuild()
        before = [(w.x, w.y) for w in self.doc.evaluate()]
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self._edges()[0]
        self.canvas._dimension_pick(self._sp(30.0, 30.0))
        self.canvas._dimension_pick(self._sp(x0, 30.0))
        con = [c for c in self.doc.constraints
               if c.kind == "distance_to_datum"][0]
        con.value = con.value - 5.0
        self.canvas.solve()
        after = [(w.x, w.y) for w in self.doc.evaluate()]
        self.assertEqual(len(before), len(after))
        for (ax, ay), (bx, by) in zip(before, after):
            self.assertAlmostEqual(-5.0, bx - ax, places=3)
            self.assertAlmostEqual(0.0, by - ay, places=3)

    def test_a_datum_dimension_on_an_anchor_is_drawn(self):
        """``_draw_dimensions`` indexed ``evaluate()``, which has no entry for
        an anchor, so the dimension would have been invisible."""
        ring = self.doc.add_ring(30.0, 30.0, count=6, ring_diameter_mm=12.0)
        ring.center_well = False
        self.canvas.rebuild()
        self.canvas.set_tool(Tool.DIMENSION)
        self.canvas._dimension_pick(self._sp(30.0, 30.0))
        self.canvas._dimension_pick(self._sp(self._edges()[0], 30.0))
        self.canvas.rebuild()
        cid = [c for c in self.doc.constraints][0].id
        pills = [i for i in self.canvas.scene().items()
                 if i.data(0) == cid]
        self.assertTrue(pills, "no dimension pill drawn for the anchor")

    def test_well_to_well_dimensions_are_drawn_too(self):
        a = self.doc.add_well(20.0, 20.0, "A")
        b = self.doc.add_well(40.0, 20.0, "B")
        self.canvas.rebuild()
        self.canvas.set_tool(Tool.DIMENSION)
        self.canvas._dimension_pick(self._sp(20.0, 20.0))
        self.canvas._dimension_pick(self._sp(40.0, 20.0))
        con = [c for c in self.doc.constraints if c.kind == "distance"]
        self.assertEqual(1, len(con), "no distance constraint created")
        self.canvas.rebuild()
        pills = [i for i in self.canvas.scene().items()
                 if i.data(0) == con[0].id]
        self.assertTrue(pills, "a well-to-well dimension drew nothing")
        self.assertIn(a.id, {a.id, b.id})

    def test_anchor_marker_is_screen_constant(self):
        """Otherwise it is unclickable on a zoomed-out 384-well plate."""
        self.doc.add_ring(30.0, 30.0, count=6, ring_diameter_mm=12.0)
        self.canvas.rebuild()
        mark = [i for i in self.canvas.scene().items()
                if isinstance(i, AnchorMarkerItem)][0]
        from PySide6.QtWidgets import QGraphicsItem
        self.assertTrue(
            mark.flags() & QGraphicsItem.ItemIgnoresTransformations)


class TestPlateSelection(_TmpStores):
    """Operator: *"there is no way to select a plate to use for the actual
    setup"* and *"I do not want to see all those extra well plates."*

    Phase 5 retired the Format→Type card and made the library's ★ the plate
    selector — but ``PlateWorkspacePage._on_active_changed`` dropped standard
    formats (``and not is_standard(...)``), and `PlateType` products had no
    card at all. On a fresh install the library is *nothing but* bundled
    entries, so the ★ was inert and there was no way to choose a plate.
    """

    def setUp(self):
        super().setUp()
        from gui.pages.hardware.plate_library import PlateLibraryPage
        self.page = PlateLibraryPage(kind="plate", store=self.plates)
        self.app.processEvents()

    def tearDown(self):
        self.page.deleteLater()
        super().tearDown()

    def _ids(self):
        return [e.card_id for e in self.page._entries()]

    # ── the library offers all three kinds ────────────────────────

    def test_all_six_standard_formats_are_offered(self):
        from SupportClasses.WellPlate import PLATE_DEFINITIONS
        from gui.pages.hardware.plate_library import standard_id
        self.assertEqual([6, 12, 24, 48, 96, 384],
                         sorted(PLATE_DEFINITIONS.keys()))
        ids = self._ids()
        for fmt in (6, 12, 24, 48, 96, 384):
            self.assertIn(standard_id(fmt), ids)

    def test_plate_type_products_are_offered(self):
        from gui.pages.hardware.plate_library import product_id
        ids = self._ids()
        self.assertIn(product_id("corning-glass-24"), ids)
        self.assertIn(product_id("nest-plastic-24"), ids)

    def test_a_card_maps_to_the_key_the_stores_use(self):
        from gui.pages.hardware.plate_library import (
            plate_key_for, product_id, standard_id)
        self.assertEqual("24", plate_key_for(standard_id(24)))
        self.assertEqual("nest-plastic-24",
                         plate_key_for(product_id("nest-plastic-24")))
        doc = self.plates.create("Mine", template=24)
        self.assertEqual(doc.meta.id, plate_key_for(doc.meta.id))

    def test_a_product_shares_its_base_geometry(self):
        """The operator's own read: these plates are the same, and it is the
        attached mosaic that differs."""
        from gui.pages.hardware.plate_library import product_id, standard_id
        by_id = {e.card_id: e for e in self.page._entries()}
        std = by_id[standard_id(24)]
        prod = by_id[product_id("nest-plastic-24")]
        self.assertEqual(std.wells, prod.wells)
        self.assertEqual(std.extent, prod.extent)

    # ── activation actually reaches the config ────────────────────

    def test_activating_emits_for_a_standard(self):
        from gui.pages.hardware.plate_library import standard_id
        seen = []
        self.page.active_changed.connect(seen.append)
        self.page._on_activate(standard_id(96))
        self.assertEqual([standard_id(96)], seen)

    def test_activating_emits_for_a_product(self):
        from gui.pages.hardware.plate_library import product_id
        seen = []
        self.page.active_changed.connect(seen.append)
        self.page._on_activate(product_id("corning-glass-24"))
        self.assertEqual([product_id("corning-glass-24")], seen)

    def test_the_workspace_forwards_bundled_activations(self):
        """The exact regression: the workspace filtered standards out."""
        from gui.pages.hardware.plate_library import product_id, standard_id
        ws = PlateWorkspacePage(plate_store=self.plates,
                                rosette_store=self.rosettes)
        seen = []
        ws.active_plate_changed.connect(seen.append)
        ws._on_active_changed(standard_id(24))
        ws._on_active_changed(product_id("nest-plastic-24"))
        self.assertEqual([standard_id(24), product_id("nest-plastic-24")],
                         seen)
        ws.deleteLater()

    # ── hiding ────────────────────────────────────────────────────

    def test_hiding_removes_a_card_and_persists(self):
        from gui.pages.hardware.plate_library import (
            PlateLibraryPage, load_hidden, standard_id)
        target = standard_id(384)
        self.assertIn(target, [c.doc_id() for c in self.page._cards])
        self.page._on_hide(target)
        self.assertNotIn(target, [c.doc_id() for c in self.page._cards])
        self.assertIn(target, load_hidden(self.plates.user_dir))
        fresh = PlateLibraryPage(kind="plate", store=self.plates)
        self.assertNotIn(target, [c.doc_id() for c in fresh._cards])
        fresh.deleteLater()

    def test_hiding_never_deletes_the_underlying_plate(self):
        from gui.pages.hardware.plate_library import standard_id
        self.page._on_hide(standard_id(384))
        self.assertIn(standard_id(384), self._ids())

    def test_hiding_is_reversible(self):
        from gui.pages.hardware.plate_library import standard_id
        target = standard_id(384)
        self.page._on_hide(target)
        self.page._on_hide(target)
        self.assertIn(target, [c.doc_id() for c in self.page._cards])

    def test_the_active_plate_can_never_be_hidden(self):
        """An invisible ★ would leave the operator unable to see what the
        setup is using."""
        from gui.pages.hardware.plate_library import standard_id
        target = standard_id(24)
        self.page.set_active_id(target)
        self.page._on_hide(target)
        self.assertIn(target, [c.doc_id() for c in self.page._cards])

    def test_hiding_the_active_plate_is_not_merely_deferred(self):
        """Two guards protect this: ``_on_hide`` refuses, and ``refresh``
        always shows the active card. The second alone is not enough — if the
        hide were *recorded* and only masked, switching the active plate away
        would make the old one silently vanish."""
        from gui.pages.hardware.plate_library import standard_id
        target, other = standard_id(24), standard_id(96)
        self.page.set_active_id(target)
        self.page._on_hide(target)
        self.assertNotIn(target, self.page._hidden)
        self.page.set_active_id(other)
        self.assertIn(target, [c.doc_id() for c in self.page._cards])

    def test_a_hidden_plate_reappears_when_it_becomes_active(self):
        from gui.pages.hardware.plate_library import standard_id
        target = standard_id(384)
        self.page._on_hide(target)
        self.page.set_active_id(target)
        self.assertIn(target, [c.doc_id() for c in self.page._cards])

    def test_a_corrupt_hidden_file_shows_everything(self):
        from gui.pages.hardware.plate_library import hidden_path, load_hidden
        d = self.plates.user_dir
        d.mkdir(parents=True, exist_ok=True)
        hidden_path(d).write_text("{not json", encoding="utf-8")
        self.assertEqual(set(), load_hidden(d))

    def test_the_hidden_list_lives_beside_its_own_store(self):
        """Resolving this path globally let a test that injected a temp store
        still write into the operator's real config directory."""
        from gui.pages.hardware.plate_library import (
            hidden_path, load_hidden, standard_id)
        target = standard_id(384)
        self.page._on_hide(target)
        mine, default = hidden_path(self.plates.user_dir), hidden_path()
        self.assertNotEqual(mine, default)
        self.assertIn(target, load_hidden(self.plates.user_dir))
        # The operator's real list may legitimately exist and be non-empty —
        # what must not happen is our write landing in it.
        self.assertNotIn(target, load_hidden(default.parent),
                         "the hidden list escaped the injected store")

    # ── mosaic ────────────────────────────────────────────────────

    def test_a_plate_with_no_scan_reports_none(self):
        from gui.pages.hardware.plate_library import standard_id
        self.assertEqual("", self.page._mosaic_label(standard_id(12)))


class TestActivePlateReachesTheConfig(unittest.TestCase):
    """The host half: which HardwareConfig field each kind writes."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("MEBP_PLATES_DIR")
        os.environ["MEBP_PLATES_DIR"] = self._tmp.name
        from gui.pages.hardware_setup import HardwareSetupPage
        self.page = HardwareSetupPage()
        self.app.processEvents()

    def tearDown(self):
        self.page.deleteLater()
        if self._prev is None:
            os.environ.pop("MEBP_PLATES_DIR", None)
        else:
            os.environ["MEBP_PLATES_DIR"] = self._prev
        self._tmp.cleanup()

    def _activate(self, card_id):
        self.page._plate_workspace._libs["plate"]._on_activate(card_id)
        self.app.processEvents()
        return self.page._config

    def test_a_standard_is_carried_by_plate_format(self):
        from gui.pages.hardware.plate_library import standard_id
        cfg = self._activate(standard_id(96))
        self.assertEqual(96, cfg.plate_format)
        self.assertEqual("", cfg.plate_type_id)
        self.assertEqual("", cfg.plate_doc_id)
        self.assertEqual(96, cfg.active_plate_key)

    def test_a_product_is_carried_by_plate_type_id(self):
        from gui.pages.hardware.plate_library import product_id
        cfg = self._activate(product_id("nest-plastic-24"))
        self.assertEqual("nest-plastic-24", cfg.plate_type_id)
        self.assertEqual("", cfg.plate_doc_id)
        self.assertEqual(24, cfg.plate_format)
        self.assertEqual("nest-plastic-24", cfg.active_plate_key)

    def test_a_design_is_carried_by_plate_doc_id(self):
        doc = self.page._plate_workspace.plate_store().create("Mine",
                                                              template=24)
        cfg = self._activate(doc.meta.id)
        self.assertEqual(doc.meta.id, cfg.plate_doc_id)
        self.assertEqual(doc.meta.id, cfg.active_plate_key)

    def test_switching_back_to_a_standard_clears_the_others(self):
        from gui.pages.hardware.plate_library import product_id, standard_id
        self._activate(product_id("corning-glass-24"))
        cfg = self._activate(standard_id(24))
        self.assertEqual("", cfg.plate_type_id)
        self.assertEqual("", cfg.plate_doc_id)
        self.assertEqual(24, cfg.active_plate_key)

    def test_a_rebuild_does_not_revert_the_choice(self):
        """`_rebuild_config` falls back to the shim combo whenever
        `plate_doc_id` is empty, so a bundled choice that left the combo stale
        would be silently undone on the very next rebuild."""
        from gui.pages.hardware.plate_library import product_id, standard_id
        for card, key in ((standard_id(96), 96),
                          (product_id("nest-plastic-24"), "nest-plastic-24")):
            self._activate(card)
            self.page._rebuild_config()
            self.assertEqual(key, self.page._config.active_plate_key, card)

    def test_the_star_lands_on_the_active_card(self):
        from gui.pages.hardware.plate_library import product_id, standard_id
        doc = self.page._plate_workspace.plate_store().create("D", template=6)
        for card in (standard_id(48), product_id("corning-glass-96"),
                     doc.meta.id):
            self._activate(card)
            self.assertEqual(card, self.page._active_library_id())


class TestLearnLoopSavesToADesign(unittest.TestCase):
    """The writer half of *"the needle offsets wont save to the plate because
    it has no type"*. Drives the real ``CalibrationPage`` handler."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("MEBP_PLATES_DIR")
        os.environ["MEBP_PLATES_DIR"] = self._tmp.name
        from SupportClasses.PlateDocumentStore import (
            get_plate_store, reset_stores)
        reset_stores()
        self.store = get_plate_store()

    def tearDown(self):
        from SupportClasses.PlateDocumentStore import reset_stores
        if self._prev is None:
            os.environ.pop("MEBP_PLATES_DIR", None)
        else:
            os.environ["MEBP_PLATES_DIR"] = self._prev
        reset_stores()
        self._tmp.cleanup()

    def _page(self, cfg):
        from gui.pages.calibration import CalibrationPage

        class _Ctrl:
            pushed = None

            def get_needle_cam_z_user(self):
                return 30.0

            def zref_to_user_z(self, z):
                return float(z)

            def set_plate_z_offsets(self, **kw):
                self.pushed = kw

        class _Lbl:
            text = ""

            def setText(self, t):
                self.text = t

            def setStyleSheet(self, _s):
                pass

        page = CalibrationPage.__new__(CalibrationPage)
        page._hardware_config = cfg
        page.controller = _Ctrl()
        page._top_z, page._plate_bottom_z = 20.0, 8.0
        page._safe_z, page._max_z = 34.0, 36.0
        page._zoff_lbl_save_to_type = _Lbl()
        return page

    def _cfg(self, **kw):
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.plate_format = kw.pop("fmt", 24)
        for k, v in kw.items():
            setattr(cfg, k, v)
        return cfg

    def test_saving_to_a_design_writes_the_document(self):
        doc = self.store.create("Custom 6 insert with rosette", template=6)
        page = self._page(self._cfg(plate_doc_id=doc.meta.id, fmt=6))
        page._zoff_save_offsets_to_plate_type()
        saved = self.store.get(doc.meta.id).meta.z_offsets
        self.assertEqual({"top", "bottom", "safe", "max"}, set(saved))
        self.assertIn("Saved offsets", page._zoff_lbl_save_to_type.text)
        self.assertIn(doc.meta.name, page._zoff_lbl_save_to_type.text)

    def test_the_saved_offsets_are_pushed_live(self):
        doc = self.store.create("Live", template=6)
        page = self._page(self._cfg(plate_doc_id=doc.meta.id, fmt=6))
        page._zoff_save_offsets_to_plate_type()
        self.assertEqual(self.store.get(doc.meta.id).meta.z_offsets,
                         page.controller.pushed)

    def test_a_bundled_standard_still_refuses_but_says_what_to_do(self):
        from unittest import mock
        page = self._page(self._cfg())
        with mock.patch("gui.pages.calibration.QMessageBox") as mb:
            page._zoff_save_offsets_to_plate_type()
        self.assertTrue(mb.information.called)
        msg = mb.information.call_args[0][2]
        self.assertIn("Duplicate", msg)
        self.assertNotIn("select a specific plate TYPE", msg)

    def test_saving_merges_rather_than_replacing(self):
        doc = self.store.create("Merge", template=6)
        doc.meta.z_offsets = {"custom": 1.0}
        self.store.save(doc)
        page = self._page(self._cfg(plate_doc_id=doc.meta.id, fmt=6))
        page._zoff_save_offsets_to_plate_type()
        self.assertEqual(1.0,
                         self.store.get(doc.meta.id).meta.z_offsets["custom"])


class TestNamingControls(_TmpStores):
    """Operator: *"there is no way to change that."* ``validate()`` reported
    the duplicate-name collision, but the panel had no naming control at all,
    so the report named a problem the operator could not act on."""

    def setUp(self):
        super().setUp()
        from gui.pages.hardware.plate_property_panel import PlatePropertyPanel
        self.canvas = PlateDocumentCanvas()
        self.doc = PlateDocument.new_plate("Naming")
        self.canvas.set_document(self.doc)
        self.panel = PlatePropertyPanel()
        self.panel.attach(self.canvas)
        self.panel.set_document(self.doc)
        self.app.processEvents()

    def tearDown(self):
        self.panel.deleteLater()
        self.canvas.deleteLater()
        super().tearDown()

    def _select(self, feat):
        self.canvas.select_refs([(feat.id, "")])
        self.panel.bind()
        self.panel._do_refresh(force=True)

    def test_the_grid_card_exposes_naming(self):
        g = self.doc.add_grid(0.0, 0.0, rows=2, cols=2,
                              pitch_x_mm=9.0, pitch_y_mm=9.0)
        self._select(g)
        for attr in ("_grid_scheme", "_grid_prefix", "_grid_start_row",
                     "_grid_start_col", "_grid_name_preview"):
            self.assertTrue(hasattr(self.panel, attr), attr)

    def test_the_ring_card_exposes_naming(self):
        r = self.doc.add_ring(20.0, 20.0, count=4, ring_diameter_mm=8.0)
        self._select(r)
        for attr in ("_ring_scheme", "_ring_prefix", "_ring_start_row",
                     "_ring_start_col", "_ring_name_preview"):
            self.assertTrue(hasattr(self.panel, attr), attr)

    def test_the_preview_shows_the_names_that_will_be_used(self):
        g = self.doc.add_grid(0.0, 0.0, rows=2, cols=2,
                              pitch_x_mm=9.0, pitch_y_mm=9.0)
        self._select(g)
        self.assertIn("A1", self.panel._grid_name_preview.text())

    def test_a_collision_is_reported_on_the_card(self):
        self.doc.add_grid(0.0, 0.0, rows=2, cols=2,
                          pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = self.doc.add_grid(60.0, 0.0, rows=2, cols=2,
                               pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2.naming.start_row = 0
        self._select(g2)
        text = self.panel._grid_name_preview.text()
        self.assertIn("clash", text)
        self.assertIn("A1", text)

    def test_the_auto_number_button_repairs_it(self):
        self.doc.add_grid(0.0, 0.0, rows=2, cols=2,
                          pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = self.doc.add_grid(60.0, 0.0, rows=2, cols=2,
                               pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2.naming.start_row = 0
        self._select(g2)
        self.panel._autoname()
        self.assertEqual([], self.doc.validate())
        self.assertNotIn("clash", self.panel._grid_name_preview.text())

    def test_typing_a_prefix_reaches_the_document(self):
        g = self.doc.add_grid(0.0, 0.0, rows=1, cols=2,
                              pitch_x_mm=9.0, pitch_y_mm=9.0)
        self._select(g)
        self.panel._set_naming("grid", "prefix", "L")
        self.assertEqual(["LA1", "LA2"], self.doc.feature_member_names(g))

    def test_the_row_field_is_disabled_for_schemes_without_rows(self):
        g = self.doc.add_grid(0.0, 0.0, rows=1, cols=2,
                              pitch_x_mm=9.0, pitch_y_mm=9.0)
        self._select(g)
        self.assertTrue(self.panel._grid_start_row.isEnabled())
        self.panel._set_naming("grid", "scheme", NamingScheme.NUMBERS)
        self.panel._do_refresh(force=True)
        self.assertFalse(self.panel._grid_start_row.isEnabled())

    def test_autoname_is_one_undo_step(self):
        self.doc.add_grid(0.0, 0.0, rows=2, cols=2,
                          pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2 = self.doc.add_grid(60.0, 0.0, rows=2, cols=2,
                               pitch_x_mm=9.0, pitch_y_mm=9.0)
        g2.naming.start_row = 0
        self._select(g2)
        depth = len(self.canvas._undo)
        self.panel._autoname()
        self.assertEqual(depth + 1, len(self.canvas._undo))
        self.canvas.undo()
        feat = self.doc.entities[g2.id]
        self.assertEqual(0, feat.naming.start_row)


class TestDimensionAndLineUsability(_TmpStores):
    """Operator: *"in general the dimension and line drawing tools are not
    easy to use."* Both reduce to not being able to see what a click will do.
    """

    def setUp(self):
        super().setUp()
        self.canvas = PlateDocumentCanvas()
        self.doc = PlateDocument.from_standard_format(24)
        self.canvas.set_document(self.doc)
        self.canvas.resize(800, 600)
        self.canvas.show()
        self.app.processEvents()

    def tearDown(self):
        self.canvas.deleteLater()
        super().tearDown()

    def _sp(self, x_mm, y_mm) -> QPointF:
        return QPointF(x_mm * SCALE, y_mm * SCALE)

    def test_hovering_a_plate_edge_highlights_it(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0, y0, _x1, _y1 = self.doc.boundary.extent_a1()
        self.canvas._update_preview(self._sp(x0, 40.0), (x0, 40.0))
        self.assertTrue(self.canvas._preview,
                        "hovering an edge produced no highlight")

    def test_moving_the_real_mouse_drives_the_preview(self):
        """Goes through ``mouseMoveEvent`` rather than calling the preview
        directly — a mutation that simply unhooked the call from the mouse
        survived the direct-call tests, which would have left the operator
        with exactly the invisible targets they complained about."""
        from PySide6.QtCore import QEvent
        from PySide6.QtGui import QMouseEvent
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self.doc.boundary.extent_a1()[0]
        pos = self.canvas.mapFromScene(self._sp(x0, 40.0))
        ev = QMouseEvent(QEvent.MouseMove, QPointF(pos),
                         Qt.NoButton, Qt.NoButton, NO_MOD)
        self.canvas.mouseMoveEvent(ev)
        self.assertTrue(self.canvas._preview,
                        "a real mouse move over a plate edge highlighted "
                        "nothing")

    def test_empty_space_highlights_nothing(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0, y0, x1, y1 = self.doc.boundary.extent_a1()
        far = (x1 + 60.0, y1 + 60.0)
        self.canvas._update_preview(self._sp(*far), far)
        self.assertFalse(self.canvas._preview)

    def test_the_first_pick_stays_visible_while_choosing_the_second(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self.doc.boundary.extent_a1()[0]
        self.canvas._dimension_pick(self._sp(x0, 40.0))
        far = (x0 + 40.0, 40.0)
        self.canvas._update_preview(self._sp(*far), far)
        self.assertTrue(self.canvas._preview,
                        "the committed first reference stopped being shown")

    def test_escape_cancels_the_pick_but_keeps_the_tool(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self.doc.boundary.extent_a1()[0]
        self.canvas._dimension_pick(self._sp(x0, 40.0))
        self.assertEqual(1, len(self.canvas._dim_pick))
        self.canvas.cancel_tool()
        self.assertEqual(0, len(self.canvas._dim_pick))
        self.assertIs(Tool.DIMENSION, self.canvas.tool())
        self.canvas.cancel_tool()
        self.assertIs(Tool.SELECT, self.canvas.tool())

    def test_escape_cancels_a_half_drawn_line(self):
        self.canvas.set_tool(Tool.LINE)
        self.canvas._pending_pt = (10.0, 10.0)
        self.canvas.cancel_tool()
        self.assertIsNone(self.canvas._pending_pt)
        self.assertIs(Tool.LINE, self.canvas.tool())

    def test_a_half_drawn_line_previews_to_the_cursor(self):
        self.canvas.set_tool(Tool.LINE)
        self.canvas._pending_pt = (10.0, 10.0)
        self.canvas._update_preview(self._sp(40.0, 25.0), (40.0, 25.0))
        self.assertTrue(self.canvas._preview,
                        "the line tool drew no rubber band")

    def test_ring_drag_previews_its_circle(self):
        self.canvas.set_tool(Tool.RING)
        self.canvas._pending_pt = (30.0, 30.0)
        self.canvas._update_preview(self._sp(38.0, 30.0), (38.0, 30.0))
        self.assertTrue(self.canvas._preview)

    def test_preview_never_touches_the_document(self):
        before = self.doc.to_dict()
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self.doc.boundary.extent_a1()[0]
        for i in range(12):
            self.canvas._update_preview(self._sp(x0, 10.0 + i), (x0, 10.0 + i))
        self.assertEqual(before, self.doc.to_dict())

    def test_preview_items_do_not_leak(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self.doc.boundary.extent_a1()[0]
        for i in range(25):
            self.canvas._update_preview(self._sp(x0, 10.0 + i), (x0, 10.0 + i))
        self.assertLessEqual(len(self.canvas._preview), 2)

    def test_the_edge_grab_band_is_screen_constant(self):
        """A fixed 1.5 mm floor is a couple of device pixels when zoomed out
        on a 128 mm plate — an invisible target."""
        self.canvas.resetTransform()
        self.canvas.scale(0.2, 0.2)
        wide = self.canvas._datum_tol_mm()
        self.canvas.resetTransform()
        self.canvas.scale(4.0, 4.0)
        tight = self.canvas._datum_tol_mm()
        self.assertGreater(wide, tight * 4.0)

    def test_a_second_click_on_the_same_reference_is_refused(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0 = self.doc.boundary.extent_a1()[0]
        self.canvas._dimension_pick(self._sp(x0, 40.0))
        self.canvas._dimension_pick(self._sp(x0, 40.0))
        self.assertEqual(1, len(self.canvas._dim_pick))
        self.assertFalse([c for c in self.doc.constraints
                          if c.kind == "distance_to_datum"])

    def test_two_edges_cannot_be_dimensioned_to_each_other(self):
        self.canvas.set_tool(Tool.DIMENSION)
        x0, y0, x1, _y1 = self.doc.boundary.extent_a1()
        msgs = []
        self.canvas.status_message.connect(msgs.append)
        self.canvas._dimension_pick(self._sp(x0, 40.0))
        self.canvas._dimension_pick(self._sp(x1, 40.0))
        self.assertFalse(self.doc.constraints)
        self.assertTrue(msgs and "edges" in msgs[-1])

    def test_targets_describe_themselves_usefully(self):
        ring = self.doc.add_ring(50.0, 30.0, count=6, ring_diameter_mm=10.0)
        self.canvas.rebuild()
        self.assertEqual("ring centre",
                         self.canvas._describe(("ref", (ring.id, ""))))
        self.assertEqual("edge left",
                         self.canvas._describe(("datum", "edge_left")))


if __name__ == "__main__":
    unittest.main()
