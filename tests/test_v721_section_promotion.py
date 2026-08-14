"""v7.21 — moving a settings section onto the workflow page, and reordering it.

Three surfaces:

* :class:`SectionStack` — the reorderable column. Entries are EXTERNALLY OWNED,
  so the tests care most about what happens to the caller's widget on ``take``.
* :class:`WorkflowSettingsDialog` promotion — the checkbox, the reparent, and the
  fact that a promoted section's fields are still saved. That last one is the
  whole reason this design is a *move* and not a rebuild: the dialog persists by
  widget IDENTITY, so placement is irrelevant to ``collect`` / ``apply``.
* the pages — all seven with a popout get a host; the six whose main column does
  not scroll get a drawer that is hidden while empty and bounded when not.
"""

from __future__ import annotations

import ast
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MEBP_UI_SCALE", "1.0")
os.environ.setdefault(
    "MEBP_WORKFLOW_SETTINGS_DIR",
    os.path.join(os.environ.get("TEMP", "."), "v721_wf_cfg"))

from PySide6.QtWidgets import (                              # noqa: E402
    QApplication, QDoubleSpinBox, QFrame, QLabel,
)

from gui.widgets.components import Card                      # noqa: E402
from gui.widgets.section_stack import (                      # noqa: E402
    PromotedSectionsPanel, SectionStack, wire_section_promotion,
)
from gui.dialogs.workflow_settings_dialog import (            # noqa: E402
    WorkflowSettingsDialog, section_id_for,
)
from SupportClasses.WorkflowLayoutStore import (              # noqa: E402
    SECTION, WorkflowLayoutStore,
)


def _app():
    return QApplication.instance() or QApplication(sys.argv)


class FakeSettings:
    def __init__(self):
        self.data = {}
        self.saves = 0

    def get_section(self, section):
        got = self.data.get(section, {})
        return dict(got) if isinstance(got, dict) else {}

    def set_section(self, section, value):
        self.data[section] = dict(value)

    def save(self):
        self.saves += 1

    def get(self, *a, **k):
        return None

    def set(self, *a, **k):
        pass

    def stored(self, wf, key):
        return self.data.get(SECTION, {}).get(wf, {}).get(key)


def _flush_deletions():
    """Actually carry out pending ``deleteLater`` calls.

    ⚠ Load-bearing in the ``take()`` tests: ``deleteLater`` is DEFERRED to the
    event loop, so a widget that has been scheduled for destruction still answers
    its methods perfectly well. A mutation that deleted the caller's widget along
    with the stack's wrapper therefore SURVIVED until these tests flushed the
    queue first — the test was passing on a corpse.
    """
    from PySide6.QtCore import QCoreApplication, QEvent
    app = QApplication.instance()
    if app is None:
        return
    app.processEvents()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()


# ════════════════════════════════════════════════════════════════════
#  SectionStack
# ════════════════════════════════════════════════════════════════════

class TestSectionStack(_Base):
    def _stack(self):
        st = SectionStack()
        self.addCleanup(st.deleteLater)
        return st

    def test_titled_cards_get_their_controls_in_their_OWN_header(self):
        """No wrapper, so no card-inside-a-card double border."""
        st = self._stack()
        card = Card("Alpha")
        before = card._header_layout.count()
        st.add("a", card)
        self.assertEqual(card._header_layout.count(), before + 2)
        self.assertIs(st._entries[0].host, card)

    def test_a_headerless_widget_gets_a_control_strip_instead(self):
        st = self._stack()
        frame = QFrame()
        st.add("a", frame, label="Bare")
        self.assertIsNot(st._entries[0].host, frame)
        self.assertIs(st.widget_for("a"), frame)

    def test_reordering_moves_the_layout_not_just_the_list(self):
        st = self._stack()
        cards = {k: Card(k) for k in ("a", "b", "c")}
        for k, c in cards.items():
            st.add(k, c)
        st.move("c", -2)
        self.assertEqual(st.ids(), ["c", "a", "b"])
        idx = [st._lay.indexOf(cards[k]) for k in ("c", "a", "b")]
        self.assertEqual(idx, sorted(idx), "layout order did not follow")

    def test_move_at_the_edge_is_a_no_op_and_emits_nothing(self):
        st = self._stack()
        seen = []
        st.order_changed.connect(seen.append)
        st.add("a", Card("a"))
        st.add("b", Card("b"))
        st.move("a", -1)
        self.assertEqual(seen, [])
        st.move("b", +1)
        self.assertEqual(seen, [])
        st.move("b", -1)
        self.assertEqual(seen, [["b", "a"]])

    def test_edge_buttons_are_greyed(self):
        """A live button that cannot do anything reads as broken."""
        st = self._stack()
        for k in ("a", "b", "c"):
            st.add(k, Card(k))
        ups = [e.buttons[0].isEnabled() for e in st._entries]
        downs = [e.buttons[1].isEnabled() for e in st._entries]
        self.assertEqual(ups, [False, True, True])
        self.assertEqual(downs, [True, True, False])

    def test_take_returns_the_caller_widget_UNPARENTED_and_alive(self):
        """The property that lets a promoted section go back to the popout."""
        st = self._stack()
        card = Card("Alpha")
        card.add_widget(QLabel("body"))
        st.add("a", card, returnable=True)
        got = st.take("a")
        self.assertIs(got, card)
        self.assertIsNone(card.parent())
        _flush_deletions()                    # deleteLater is DEFERRED
        self.assertEqual(card.title(), "Alpha")      # genuinely not deleted

    def test_take_strips_the_controls_the_stack_injected(self):
        st = self._stack()
        card = Card("Alpha")
        before = card._header_layout.count()
        st.add("a", card, returnable=True)
        self.assertGreater(card._header_layout.count(), before)
        st.take("a")
        self.assertEqual(card._header_layout.count(), before)

    def test_taking_a_wrapped_widget_does_not_delete_it(self):
        """The wrapper is destroyed; the caller's widget must survive it.

        ⚠ The flush is what gives this test teeth: without it the mutation that
        deletes the widget along with its wrapper passes, because deleteLater has
        not run yet and a doomed widget still answers objectName()."""
        st = self._stack()
        frame = QFrame()
        frame.setObjectName("mine")
        frame.setToolTip("still here")
        st.add("a", frame, label="Bare")
        got = st.take("a")
        self.assertIs(got, frame)
        self.assertIsNone(got.parent(), "still parented to the dead wrapper")
        _flush_deletions()
        self.assertEqual(got.objectName(), "mine")
        self.assertEqual(got.toolTip(), "still here")

    def test_set_order_cannot_drop_a_card(self):
        st = self._stack()
        for k in ("a", "b", "c"):
            st.add(k, Card(k))
        st.set_order(["c"])                     # a deliberately partial order
        self.assertEqual(sorted(st.ids()), ["a", "b", "c"])
        self.assertEqual(st.ids()[0], "c")

    def test_unknown_ids_in_set_order_are_ignored(self):
        st = self._stack()
        st.add("a", Card("a"))
        st.set_order(["zzz", "a"])
        self.assertEqual(st.ids(), ["a"])

    def test_adding_the_same_id_twice_is_ignored(self):
        st = self._stack()
        c1, c2 = Card("one"), Card("two")
        st.add("a", c1)
        st.add("a", c2)
        self.assertEqual(st.ids(), ["a"])
        self.assertIs(st.widget_for("a"), c1)

    def test_contents_changed_fires_on_membership_not_on_reorder(self):
        st = self._stack()
        seen = []
        st.contents_changed.connect(lambda: seen.append(1))
        st.add("a", Card("a"))
        st.add("b", Card("b"))
        self.assertEqual(len(seen), 2)
        st.move("b", -1)
        self.assertEqual(len(seen), 2)
        st.take("a")
        self.assertEqual(len(seen), 3)


class TestCardHeaderSlot(_Base):
    """``Card.add_header_widget`` is public API the promotion checkbox and the
    stack's ▲▼ both depend on, so its refusal path is pinned directly — no
    ``add_section`` caller can reach it (a title of ``""`` still builds a
    header), and an unreachable branch is exactly the kind that rots."""

    def test_a_control_lands_in_the_header(self):
        card = Card("Titled")
        self.addCleanup(card.deleteLater)
        self.assertTrue(card.has_header())
        lbl = QLabel("x")
        self.assertIs(card.add_header_widget(lbl), lbl)
        self.assertEqual(card._header_layout.indexOf(lbl), 1)

    def test_a_headerless_card_REFUSES_rather_than_hiding_it_in_the_body(self):
        """Returning the widget would let a caller believe it was placed; it
        would then be unparented and invisible, or worse land in the body where
        a control reads as content."""
        card = Card(None)
        self.addCleanup(card.deleteLater)
        self.assertFalse(card.has_header())
        lbl = QLabel("x")
        self.assertIsNone(card.add_header_widget(lbl))
        self.assertIsNone(lbl.parent())
        self.assertEqual(card.body_layout().count(), 0)

    def test_the_collapse_chevron_stays_rightmost(self):
        """So its position does not shift as callers add their own buttons."""
        card = Card("Titled", collapsible=True)
        self.addCleanup(card.deleteLater)
        card.add_header_widget(QLabel("a"))
        card.add_header_widget(QLabel("b"))
        self.assertEqual(card._header_layout.indexOf(card._toggle_btn),
                         card._header_layout.count() - 1)


class TestPromotedDrawer(_Base):
    def test_hidden_while_empty_and_shown_when_filled(self):
        """A page with nothing promoted must have its pre-v7.21 geometry."""
        p = PromotedSectionsPanel()
        self.addCleanup(p.deleteLater)
        self.assertTrue(p.isHidden())
        p.stack.add("a", Card("a"))
        self.assertFalse(p.isHidden())
        p.stack.take("a")
        self.assertTrue(p.isHidden())

    def test_the_drawer_bounds_its_height(self):
        """So no number of promoted sections can push a page's run row — and
        therefore Abort — out of reach on a page that does not scroll."""
        p = PromotedSectionsPanel(max_height_px=210)
        self.addCleanup(p.deleteLater)
        self.assertEqual(p._scroll.maximumHeight(), 210)
        d = PromotedSectionsPanel()
        self.addCleanup(d.deleteLater)
        self.assertLess(d._scroll.maximumHeight(), 100000)


# ════════════════════════════════════════════════════════════════════
#  Dialog promotion
# ════════════════════════════════════════════════════════════════════

class TestSectionIds(unittest.TestCase):
    def test_slug_is_stable_and_readable(self):
        self.assertEqual(section_id_for("Needle prep"), "needle_prep")
        self.assertEqual(section_id_for("Common — Pump (global)"),
                         "common_pump_global")
        self.assertEqual(section_id_for("  Ink  pickup  "), "ink_pickup")

    def test_an_unusable_title_still_yields_an_id(self):
        self.assertEqual(section_id_for("—"), "section")
        self.assertEqual(section_id_for(""), "section")


class TestDialogPromotion(_Base):
    def _dialog(self, n=3, wf="v721_wf"):
        dlg = WorkflowSettingsDialog(wf, "Test")
        self.addCleanup(dlg.deleteLater)
        for i, title in enumerate(("Alpha", "Beta", "Gamma", "Delta")[:n]):
            sec = dlg.add_section(title)
            sec.add(f"f{i}", title, QDoubleSpinBox(), float(i))
        dlg.finalize()
        return dlg

    def _wired(self, n=3, wf="v721_wf"):
        dlg = self._dialog(n, wf)
        st = FakeSettings()
        stack = SectionStack()
        self.addCleanup(stack.deleteLater)
        store = WorkflowLayoutStore(st)
        dlg.set_promotion_host(stack, layout_store=store)
        return dlg, stack, st, store

    def test_every_section_gets_a_checkbox(self):
        dlg = self._dialog()
        self.assertEqual([sid for sid, _t in dlg.promotable_sections()],
                         ["alpha", "beta", "gamma"])
        for info in dlg._sections.values():
            self.assertIsNotNone(info["check"])

    def test_the_checkbox_is_hidden_until_a_host_exists(self):
        """isHidden(), not isVisible(): the dialog itself is unshown in a test,
        so isVisible() is False either way and would prove nothing (a trap this
        repo already records)."""
        dlg = self._dialog()
        self.assertTrue(all(i["check"].isHidden()
                            for i in dlg._sections.values()))
        stack = SectionStack()
        self.addCleanup(stack.deleteLater)
        dlg.set_promotion_host(stack)
        self.assertTrue(all(not i["check"].isHidden()
                            for i in dlg._sections.values()))

    def test_promoting_moves_the_card_out_of_the_popout(self):
        dlg, stack, _st, _store = self._wired()
        card = dlg._sections["beta"]["card"]
        n = dlg._content_layout.count()
        self.assertTrue(dlg.set_section_promoted("beta", True))
        self.assertEqual(dlg._content_layout.count(), n - 1)
        self.assertTrue(stack.has("beta"))
        self.assertIs(card.parent(), stack)

    def test_a_promoted_sections_fields_are_STILL_saved(self):
        """The reason this is a move and not a rebuild: the dialog persists by
        widget identity, so placement cannot affect collect/apply."""
        dlg, _stack, _st, _store = self._wired()
        dlg.set_section_promoted("beta", True)
        dlg._fields["f1"][0].setValue(4.25)
        self.assertEqual(dlg.collect()["f1"], 4.25)
        dlg.apply({"f1": 9.5})
        self.assertEqual(dlg._fields["f1"][0].value(), 9.5)
        dlg.reset_defaults()
        self.assertEqual(dlg._fields["f1"][0].value(), 1.0)

    def test_ticking_the_checkbox_promotes(self):
        dlg, stack, _st, _store = self._wired()
        dlg._sections["gamma"]["check"].setChecked(True)
        self.assertTrue(stack.has("gamma"))

    def test_un_promoting_returns_it_between_its_ORIGINAL_neighbours(self):
        dlg, _stack, _st, _store = self._wired()
        dlg.set_section_promoted("beta", True)
        dlg.set_section_promoted("beta", False)
        titles = []
        for i in range(dlg._content_layout.count()):
            w = dlg._content_layout.itemAt(i).widget()
            if w is not None:
                titles.append(w.title())
        self.assertEqual(titles, ["Alpha", "Beta", "Gamma"])

    def test_it_returns_correctly_even_when_a_NEIGHBOUR_is_promoted(self):
        """A stored absolute index would drift here; the position has to be
        resolved against the sections still present."""
        dlg, _stack, _st, _store = self._wired()
        dlg.set_section_promoted("alpha", True)
        dlg.set_section_promoted("gamma", True)
        dlg.set_section_promoted("beta", True)
        dlg.set_section_promoted("beta", False)
        titles = [dlg._content_layout.itemAt(i).widget().title()
                  for i in range(dlg._content_layout.count())
                  if dlg._content_layout.itemAt(i).widget() is not None]
        self.assertEqual(titles, ["Beta"])
        dlg.set_section_promoted("gamma", False)
        dlg.set_section_promoted("alpha", False)
        titles = [dlg._content_layout.itemAt(i).widget().title()
                  for i in range(dlg._content_layout.count())
                  if dlg._content_layout.itemAt(i).widget() is not None]
        self.assertEqual(titles, ["Alpha", "Beta", "Gamma"])

    def test_the_returned_card_stays_BEFORE_the_trailing_stretch(self):
        dlg, _stack, _st, _store = self._wired()
        dlg.set_section_promoted("gamma", True)
        dlg.set_section_promoted("gamma", False)
        last = dlg._content_layout.count() - 1
        self.assertIsNone(dlg._content_layout.itemAt(last).widget(),
                          "a card landed after finalize()'s stretch")

    def test_promotion_persists_and_restores_on_a_fresh_dialog(self):
        dlg, _stack, st, store = self._wired()
        dlg.set_section_promoted("beta", True)
        self.assertEqual(st.stored("v721_wf", "promoted"), ["beta"])
        dlg2 = self._dialog()
        stack2 = SectionStack()
        self.addCleanup(stack2.deleteLater)
        dlg2.set_promotion_host(stack2, layout_store=store)
        self.assertTrue(stack2.has("beta"))
        self.assertTrue(dlg2._sections["beta"]["check"].isChecked())

    def test_a_stored_id_that_no_longer_exists_is_ignored(self):
        st = FakeSettings()
        store = WorkflowLayoutStore(st)
        store.set_promoted("v721_wf", "renamed_away", True)
        dlg = self._dialog()
        stack = SectionStack()
        self.addCleanup(stack.deleteLater)
        dlg.set_promotion_host(stack, layout_store=store)   # must not raise
        self.assertEqual(stack.ids(), [])

    def test_no_host_means_promotion_is_refused_not_crashed(self):
        dlg = self._dialog()
        self.assertFalse(dlg.set_section_promoted("beta", True))

    def test_a_duplicate_title_gets_its_own_id(self):
        """Two sections sharing one id would fight over one checkbox."""
        dlg = WorkflowSettingsDialog("v721_dup", "Dup")
        self.addCleanup(dlg.deleteLater)
        a = dlg.add_section("Same")
        b = dlg.add_section("Same")
        self.assertNotEqual(a.section_id, b.section_id)
        self.assertEqual(len(dlg.promotable_sections()), 2)

    def test_promotable_false_gets_no_checkbox(self):
        dlg = WorkflowSettingsDialog("v721_np", "NP")
        self.addCleanup(dlg.deleteLater)
        dlg.add_section("Fixed", promotable=False)
        self.assertEqual(dlg.promotable_sections(), [])
        self.assertIsNone(dlg._sections["fixed"]["check"])


class TestWiringHelper(_Base):
    """``wire_section_promotion`` owns the restore ORDER, which is why it is
    shared rather than copied into seven pages."""

    def _setup(self, wf="v721_wire"):
        dlg = WorkflowSettingsDialog(wf, "Wire")
        self.addCleanup(dlg.deleteLater)
        for t in ("Alpha", "Beta"):
            dlg.add_section(t).add(t.lower(), t, QDoubleSpinBox(), 1.0)
        dlg.finalize()
        stack = SectionStack()
        self.addCleanup(stack.deleteLater)
        for k in ("builtin_one", "builtin_two"):
            stack.add(k, Card(k))
        return dlg, stack

    def test_restoring_promotions_does_NOT_overwrite_the_stored_order(self):
        """🐞 The bug this pins: restore fired the host's on_change per section,
        which persisted the CURRENT (default) order over the stored one — so the
        arrangement silently reset to default on every launch while the
        promotions themselves survived."""
        st = FakeSettings()
        store = WorkflowLayoutStore(st)
        store.set_promoted("v721_wire", "beta", True)
        store.set_order("v721_wire", ["beta", "builtin_two", "builtin_one"])
        dlg, stack = self._setup()
        wire_section_promotion(None, dlg, stack, settings=st,
                              workflow_id="v721_wire", store=store)
        self.assertEqual(stack.ids(),
                         ["beta", "builtin_two", "builtin_one"])
        self.assertEqual(st.stored("v721_wire", "order"),
                         ["beta", "builtin_two", "builtin_one"])

    def test_a_move_persists_the_new_order(self):
        st = FakeSettings()
        dlg, stack = self._setup()
        wire_section_promotion(None, dlg, stack, settings=st,
                               workflow_id="v721_wire")
        stack.move("builtin_two", -1)
        self.assertEqual(st.stored("v721_wire", "order"),
                         ["builtin_two", "builtin_one"])

    def test_the_return_signal_sends_a_card_home(self):
        st = FakeSettings()
        dlg, stack = self._setup()
        wire_section_promotion(None, dlg, stack, settings=st,
                               workflow_id="v721_wire")
        dlg.set_section_promoted("alpha", True)
        self.assertTrue(stack.has("alpha"))
        stack.return_requested.emit("alpha")          # what ↩ does
        self.assertFalse(stack.has("alpha"))
        self.assertEqual(st.stored("v721_wire", "promoted"), [])

    def test_promotions_are_restored_BEFORE_the_order_is_applied(self):
        """An order applied first could not place a card that does not exist
        yet, so the promoted section would end up appended at the bottom."""
        st = FakeSettings()
        store = WorkflowLayoutStore(st)
        store.set_promoted("v721_wire", "alpha", True)
        store.set_order("v721_wire", ["alpha", "builtin_one", "builtin_two"])
        dlg, stack = self._setup()
        wire_section_promotion(None, dlg, stack, settings=st,
                               workflow_id="v721_wire", store=store)
        self.assertEqual(stack.ids()[0], "alpha")

    def test_a_missing_dialog_or_stack_is_inert(self):
        st = FakeSettings()
        self.assertIsNone(wire_section_promotion(
            None, None, SectionStack(), settings=st, workflow_id="x"))
        self.assertIsNone(wire_section_promotion(
            None, MagicMock(), None, settings=st, workflow_id="x"))


# ════════════════════════════════════════════════════════════════════
#  Pages
# ════════════════════════════════════════════════════════════════════

def _ctrl():
    from SupportClasses.StageController import StageController as _SC
    c = MagicMock()
    c.is_xy_connected = True
    c.is_zp_connected = True
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    c.print_floor_violation.return_value = False
    c.print_height_to_zref.side_effect = lambda h, *a, **k: -25.0 - float(h)
    c.print_z_dir.return_value = -1.0
    c.plate_axis_sign.return_value = (1, 1)
    c.plate_flip_180.return_value = False
    c._pending_per_axis_max_feedrate = None
    c.get_max_xy_speed_um_s.side_effect = lambda: _SC.get_max_xy_speed_um_s(c)
    c.get_max_z_feedrate_mm_min.side_effect = (
        lambda: _SC.get_max_z_feedrate_mm_min(c))
    return c


def _cam():
    m = MagicMock()
    m.cameras = [None] * 4
    m.camera_for_role.return_value = None
    return m


class TestEveryWorkflowGetsAHost(_Base):
    """The operator asked for the checkbox on every workflow's sections, so a
    page that forgets to register a host would show a checkbox that moves a
    section nowhere. This walks all seven."""

    CASES = [
        ("spheroid_pickup", "spheroid_pickup_workflow",
         "SpheroidPickupWorkflowPage", True),
        ("cell_targeting", "cell_targeting_workflow",
         "CellTargetingWorkflowPage", True),
        ("cell_labeling", "cell_labeling_workflow",
         "CellLabelingWorkflowPage", True),
        ("stress_test", "stress_test_workflow",
         "StressTestWorkflowPage", False),
        ("timing_calibration", "timing_calibration_workflow",
         "TimingCalibrationWorkflowPage", False),
        ("fluorescence_mosaic", "fluorescence_mosaic_workflow",
         "FluorescenceMosaicWorkflowPage", True),
        ("quick_print", "quick_print_workflow",
         "QuickPrintWorkflowPage", False),
    ]

    def _build(self, module, cls_name, needs_cam, settings):
        import importlib
        mod = importlib.import_module(f"gui.pages.workflows.{module}")
        cls = getattr(mod, cls_name)
        page = cls(_ctrl(), settings, _cam()) if needs_cam \
            else cls(_ctrl(), settings)
        self.addCleanup(page.deleteLater)
        return page

    def test_all_seven_register_a_host_and_can_promote(self):
        for wf, module, cls_name, needs_cam in self.CASES:
            with self.subTest(wf):
                st = FakeSettings()
                page = self._build(module, cls_name, needs_cam, st)
                dlg = page._settings_dialog
                self.assertIsNotNone(dlg._promo_host, f"{wf}: no host")
                secs = dlg.promotable_sections()
                self.assertTrue(secs, f"{wf}: no promotable sections")
                sid = secs[0][0]
                n = dlg._content_layout.count()
                self.assertTrue(dlg.set_section_promoted(sid, True))
                self.assertTrue(dlg._promo_host.has(sid))
                self.assertEqual(dlg._content_layout.count(), n - 1)
                self.assertEqual(st.stored(wf, "promoted"), [sid])

    def test_the_drawer_pages_hide_it_while_empty(self):
        """Quick Print is excluded on purpose: its column already scrolls, so it
        hosts sections directly instead of in a nested drawer."""
        for wf, module, cls_name, needs_cam in self.CASES:
            if wf == "quick_print":
                continue
            with self.subTest(wf):
                page = self._build(module, cls_name, needs_cam,
                                   FakeSettings())
                panel = getattr(page, "_promoted_panel", None)
                self.assertIsNotNone(panel, f"{wf}: no drawer")
                self.assertTrue(panel.isHidden(), f"{wf}: drawer not hidden")
                sid = page._settings_dialog.promotable_sections()[0][0]
                page._settings_dialog.set_section_promoted(sid, True)
                self.assertFalse(panel.isHidden())
                page._settings_dialog.set_section_promoted(sid, False)
                self.assertTrue(panel.isHidden(), f"{wf}: drawer did not re-hide")

    def test_quick_print_hosts_in_its_own_scrolling_column(self):
        page = self._build("quick_print_workflow", "QuickPrintWorkflowPage",
                           False, FakeSettings())
        self.assertIsNone(getattr(page, "_promoted_panel", None))
        self.assertIs(page._settings_dialog._promo_host, page._section_stack)


class TestQuickPrintColumn(_Base):
    def _page(self, settings=None):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage)
        p = QuickPrintWorkflowPage(_ctrl(), settings or FakeSettings())
        self.addCleanup(p.deleteLater)
        return p

    def test_the_default_order_is_the_pre_v721_one(self):
        p = self._page()
        self.assertEqual(
            p._section_stack.ids(),
            [p.SEC_OBJECT, p.SEC_QUEUE, p.SEC_PARAMS, p.SEC_READINESS,
             p.SEC_STATUS])

    def test_every_builtin_card_is_still_reachable(self):
        """Reordering must not be a rebuild — the page's own handlers hold these
        references."""
        p = self._page()
        for attr in ("_queue_card", "_ready_card", "_ready_host",
                     "_setup_status", "_derived_lbl", "_object_combo"):
            self.assertIsNotNone(getattr(p, attr, None), attr)

    def test_builtin_cards_reorder_and_persist(self):
        st = FakeSettings()
        p = self._page(st)
        p._section_stack.move(p.SEC_READINESS, -4)
        self.assertEqual(p._section_stack.ids()[0], p.SEC_READINESS)
        self.assertEqual(st.stored("quick_print", "order")[0],
                         p.SEC_READINESS)
        p2 = self._page(st)
        self.assertEqual(p2._section_stack.ids()[0], p.SEC_READINESS)

    def test_readiness_still_renders_after_being_moved(self):
        p = self._page()
        p._section_stack.move(p.SEC_READINESS, -4)
        p._refresh_setup_status()               # must not raise
        self.assertIsNotNone(p._ready_layout)

    def test_a_promoted_section_interleaves_with_the_builtins(self):
        st = FakeSettings()
        p = self._page(st)
        sid = p._settings_dialog.promotable_sections()[0][0]
        p._settings_dialog.set_section_promoted(sid, True)
        p._section_stack.move(sid, -10)
        self.assertEqual(p._section_stack.ids()[0], sid)
        p2 = self._page(st)
        self.assertEqual(p2._section_stack.ids()[0], sid)

    def test_an_embedded_instance_gets_NO_promotion_host(self):
        """An embedded page's layout belongs to its host; moving a card onto it
        would put a section inside someone else's page invisibly."""
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage)
        p = QuickPrintWorkflowPage(
            _ctrl(), FakeSettings(), embedded=True, owns_camera=False,
            settings_id="v721_embedded", settings_title="Emb")
        self.addCleanup(p.deleteLater)
        self.assertIsNone(p._settings_dialog._promo_host)
        self.assertFalse(p._settings_dialog.set_section_promoted(
            p._settings_dialog.promotable_sections()[0][0], True))

    def test_a_stored_order_naming_a_gone_card_cannot_hide_the_rest(self):
        st = FakeSettings()
        WorkflowLayoutStore(st).set_order(
            "quick_print", ["builtin_ghost", "builtin_status"])
        p = self._page(st)
        self.assertEqual(sorted(p._section_stack.ids()),
                         sorted([p.SEC_OBJECT, p.SEC_QUEUE, p.SEC_PARAMS,
                                 p.SEC_READINESS, p.SEC_STATUS]))
        self.assertEqual(p._section_stack.ids()[0], p.SEC_STATUS)


class TestSourceContracts(unittest.TestCase):
    ROOT = Path(__file__).resolve().parent.parent

    def test_every_popout_page_calls_the_shared_wiring_helper(self):
        """AST, not a substring: a comment naming the helper would pass a
        `in getsource()` check. A page that builds a popout but never registers
        a host shows checkboxes that move a section nowhere."""
        pages = [
            "quick_print_workflow", "spheroid_pickup_workflow",
            "cell_targeting_workflow", "cell_labeling_workflow",
            "stress_test_workflow", "timing_calibration_workflow",
            "fluorescence_mosaic_workflow",
        ]
        for name in pages:
            with self.subTest(name):
                path = self.ROOT / "gui" / "pages" / "workflows" / f"{name}.py"
                tree = ast.parse(path.read_text(encoding="utf-8"))
                called = {
                    n.func.id for n in ast.walk(tree)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                }
                self.assertIn("wire_section_promotion", called)

    def test_the_stack_never_deletes_a_caller_widget(self):
        """Entries are externally owned. A deleteLater on the caller's widget
        would take the settings section with it and the fields it registered."""
        src = (self.ROOT / "gui" / "widgets" / "section_stack.py").read_text(
            encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            if isinstance(f, ast.Attribute) and f.attr == "deleteLater":
                target = ast.unparse(f.value)
                self.assertNotIn(
                    target, ("e.widget", "entry.widget", "self._entries"),
                    f"stack deletes a caller widget: {target}")


if __name__ == "__main__":
    unittest.main()
