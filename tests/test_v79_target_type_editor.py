"""test_v79_target_type_editor.py — the target-type authoring dialog.

Before this, target types could ONLY be created by hand-editing JSON:
``save_user``/``delete_user`` existed and were called from nothing in ``gui/``,
the user directory did not exist, and a trailing comma made a type vanish with no
message. These tests pin the editor AND the two contracts that make it safe:

* an id is derived once on create and NEVER re-derived on rename, because it is
  what every bore assignment and every stamped run record points at;
* a built-in is never modified — a user entry SHADOWS it.

⚠ ``QMessageBox`` blocks forever offscreen, so every path that can raise one is
patched.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

_TT_TMP = tempfile.mkdtemp(prefix="mebp_v79_tt_editor_")
os.environ["MEBP_TARGET_TYPE_DIR"] = _TT_TMP
(Path(_TT_TMP) / "builtin").mkdir(exist_ok=True)
(Path(_TT_TMP) / "user").mkdir(exist_ok=True)

from SupportClasses.TargetTypeStore import (              # noqa: E402
    COMBINATOR_ALL, COMBINATOR_ANY, METRIC_AUTO, METRIC_BACKGROUND_SIGMA,
    STATE_ANY, STATE_NEGATIVE, STATE_POSITIVE, SignatureClause, SignatureRule,
    TargetType, TargetTypeStore,
)


def _seed_builtin():
    (Path(_TT_TMP) / "builtin" / "seed.json").write_text(json.dumps({
        "id": "seed-type",
        "name": "Seeded built-in",
        "color": "#89b4fa",
        "signature_rule": {
            "combinator": "all",
            "clauses": [
                {"imaging_channel": "FITC", "state": "positive",
                 "threshold": 3.0, "metric": "background_sigma"},
                {"imaging_channel": "mCherry", "state": "negative"},
            ],
        },
    }), encoding="utf-8")


class _DlgCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)
        _seed_builtin()

    def setUp(self):
        for f in (Path(_TT_TMP) / "user").glob("*.json"):
            f.unlink()
        self.store = TargetTypeStore()

    def _dlg(self, **kw):
        from gui.dialogs.target_type_dialog import TargetTypeDialog
        kw.setdefault("store", self.store)
        kw.setdefault("existing_ids", [t.id for t in self.store.all()])
        d = TargetTypeDialog(**kw)
        self.addCleanup(d.deleteLater)
        return d


class TestTheFormBuilds(_DlgCase):
    def test_a_new_dialog_starts_with_one_clause_row(self):
        d = self._dlg()
        self.assertEqual(len(d._rows), 1)
        self.assertTrue(d._all_radio.isChecked(), "ALL is the default")

    def test_the_deferral_banner_is_present_and_self_retiring(self):
        """It must be gated on RULE_EVALUATION_IMPLEMENTED, not hard-coded."""
        import gui.dialogs.target_type_dialog as mod
        texts = self._collect_labels(self._dlg())
        self.assertTrue(any("Recorded intent only" in t for t in texts))
        with patch.object(mod, "RULE_EVALUATION_IMPLEMENTED", True):
            texts2 = self._collect_labels(self._dlg())
        self.assertFalse(any("Recorded intent only" in t for t in texts2),
                         "the banner must disappear when an evaluator lands")

    @staticmethod
    def _collect_labels(widget):
        from PySide6.QtWidgets import QLabel
        return [w.text() for w in widget.findChildren(QLabel)]

    def test_there_is_no_test_this_rule_button(self):
        """An inert 'test' button reads as 'coming soon' and would get clicked."""
        from PySide6.QtWidgets import QPushButton
        labels = [b.text().lower() for b in self._dlg().findChildren(QPushButton)]
        for word in ("test", "detect", "find", "select matching"):
            self.assertFalse(any(word in t for t in labels),
                             f"unexpected {word!r} affordance: {labels}")

    def test_the_channel_combo_is_seeded_from_the_imaging_channels(self):
        from SupportClasses.FluorescenceMosaicStore import CHANNELS
        row = self._dlg()._rows[0]
        offered = [row.channel.itemData(i) for i in range(row.channel.count())]
        for name in CHANNELS:
            self.assertIn(name, offered)


class TestTheWidgetsCannotExpressAnImpossibleState(_DlgCase):
    """`SignatureClause.__post_init__` collapses a metric with no number and a
    number with no metric. The widget must not imply otherwise."""

    def test_the_metric_is_disabled_while_the_level_is_auto(self):
        row = self._dlg()._rows[0]
        self.assertEqual(row.level.value(), 0.0)
        self.assertFalse(row.metric.isEnabled())
        row.level.setValue(3.0)
        self.assertTrue(row.metric.isEnabled())

    def test_a_dont_care_clause_disables_both_level_and_metric(self):
        row = self._dlg()._rows[0]
        row.state.setCurrentIndex(row.state.findData(STATE_ANY))
        self.assertFalse(row.level.isEnabled())
        self.assertFalse(row.metric.isEnabled())

    def test_auto_round_trips_as_threshold_None(self):
        row = self._dlg()._rows[0]
        clause = row.to_clause()
        self.assertIsNone(clause.threshold)
        self.assertEqual(clause.metric, METRIC_AUTO)

    def test_a_level_round_trips_with_its_metric(self):
        row = self._dlg()._rows[0]
        row.level.setValue(3.0)
        row.metric.setCurrentIndex(row.metric.findData(METRIC_BACKGROUND_SIGMA))
        clause = row.to_clause()
        self.assertEqual(clause.threshold, 3.0)
        self.assertEqual(clause.metric, METRIC_BACKGROUND_SIGMA)

    def test_the_level_shows_its_meaning_rather_than_0_00(self):
        row = self._dlg()._rows[0]
        self.assertIn("choose", row.level.specialValueText().lower())


class TestValidationWarnsWithoutBlocking(_DlgCase):
    def test_a_rule_that_asserts_nothing_says_it_matches_NOTHING(self):
        """The store's fail-safe is 'matches nothing, never everything' — this
        dialog is the only place the operator will ever read why."""
        d = self._dlg()
        d._rows[0].state.setCurrentIndex(d._rows[0].state.findData(STATE_ANY))
        note = d._rule_note.text()
        self.assertIn("matches NOTHING", note)
        self.assertIn("not", note.lower())

    def test_a_complete_rule_has_no_warning(self):
        """Regression: `is_empty` is a METHOD here while its siblings are
        properties, so `if rule.is_empty:` was always truthy and this warning
        fired on every rule, including complete ones."""
        d = self._dlg()
        self.assertEqual(d._rule_note.text(), "",
                         "a single positive clause IS a gating rule")

    def test_a_contradictory_channel_is_flagged(self):
        d = self._dlg()
        d._add_clause()
        # Both rows default to the same (first) channel.
        d._rows[1].state.setCurrentIndex(d._rows[1].state.findData(STATE_NEGATIVE))
        d._refresh_preview()
        self.assertIn("both bright and dim", d._rule_note.text())

    def test_the_preview_uses_the_stores_own_describe(self):
        d = self._dlg()
        rule = d._build_rule()
        self.assertIn(rule.describe(), d._preview.text())

    def test_an_empty_name_is_refused_with_a_reason(self):
        d = self._dlg()
        d._name.setText("   ")
        with patch("gui.dialogs.target_type_dialog.QMessageBox.warning") as warn:
            d._on_save()
        self.assertTrue(warn.called)
        self.assertIsNone(d.result_type)


class TestSaveAndLoadRoundTrip(_DlgCase):
    def test_saving_creates_a_user_type_with_a_derived_id(self):
        d = self._dlg()
        d._name.setText("GFP only cells")
        d._rows[0].level.setValue(3.0)
        d._on_save()
        self.assertIsNotNone(d.result_type)
        self.assertEqual(d.result_type.id, "gfp-only-cells")
        self.assertFalse(d.result_type.builtin)
        # And it is on disk, visible to a FRESH store.
        self.assertIsNotNone(TargetTypeStore().get("gfp-only-cells"))

    def test_the_saved_rule_round_trips_through_disk(self):
        d = self._dlg()
        d._name.setText("Double positive")
        d._rows[0].level.setValue(3.0)
        d._rows[0].metric.setCurrentIndex(
            d._rows[0].metric.findData(METRIC_BACKGROUND_SIGMA))
        d._add_clause()
        d._rows[1].state.setCurrentIndex(
            d._rows[1].state.findData(STATE_NEGATIVE))
        d._on_save()
        back = TargetTypeStore().get("double-positive")
        self.assertIsNotNone(back)
        self.assertEqual(len(back.signature_rule.clauses), 2)
        self.assertEqual(back.signature_rule.clauses[0].threshold, 3.0)
        self.assertEqual(back.signature_rule.clauses[0].metric,
                         METRIC_BACKGROUND_SIGMA)
        self.assertEqual(back.signature_rule.clauses[1].state, STATE_NEGATIVE)

    def test_the_ANY_combinator_round_trips(self):
        d = self._dlg()
        d._name.setText("Either channel")
        d._any_radio.setChecked(True)
        d._on_save()
        back = TargetTypeStore().get("either-channel")
        self.assertEqual(back.signature_rule.combinator, COMBINATOR_ANY)

    def test_editing_a_builtin_creates_an_override_and_leaves_it_intact(self):
        builtin = self.store.get("seed-type")
        self.assertTrue(builtin.builtin)
        d = self._dlg(target_type=builtin)
        d._name.setText("My seeded override")
        d._on_save()
        # Same id (so assignments follow), now a USER type.
        self.assertEqual(d.result_type.id, "seed-type")
        self.assertFalse(d.result_type.builtin)
        fresh = TargetTypeStore()
        self.assertEqual(fresh.get("seed-type").name, "My seeded override")
        self.assertFalse(fresh.get("seed-type").builtin)
        # The bundled file is untouched.
        raw = json.loads((Path(_TT_TMP) / "builtin" / "seed.json")
                         .read_text(encoding="utf-8"))
        self.assertEqual(raw["name"], "Seeded built-in")

    def test_a_RENAME_never_changes_the_id(self):
        """⚠ The id is what every bore assignment and every stamped run record
        points at. Re-deriving it on rename would orphan all of them."""
        d = self._dlg(target_type=self.store.get("seed-type"))
        d._name.setText("Completely different name")
        d._on_save()
        self.assertEqual(d.result_type.id, "seed-type")

    def test_a_duplicate_gets_a_FRESH_id_not_the_originals(self):
        src = self.store.get("seed-type")
        d = self._dlg(prefill=src)
        self.assertTrue(d._is_new)
        self.assertIn("copy", d._name.text().lower())
        self.assertEqual(len(d._rows), 2, "the source's rule is pre-filled")
        d._name.setText("Seed variant")
        d._on_save()
        self.assertEqual(d.result_type.id, "seed-variant")
        self.assertIsNotNone(TargetTypeStore().get("seed-type"),
                             "the original must survive")

    def test_a_duplicate_id_asks_before_overwriting(self):
        d = self._dlg()
        d._name.setText("Seed type")            # → id "seed-type", a built-in
        from PySide6.QtWidgets import QMessageBox as QMB
        with patch("gui.dialogs.target_type_dialog.QMessageBox.question",
                   return_value=QMB.StandardButton.Cancel) as q:
            d._on_save()
        self.assertTrue(q.called)
        self.assertIsNone(d.result_type, "Cancel must not save")

    def test_a_failed_save_reports_instead_of_claiming_success(self):
        d = self._dlg()
        d._name.setText("Doomed")
        with patch.object(self.store, "save_user", return_value=False), \
             patch("gui.dialogs.target_type_dialog.QMessageBox.critical") as crit:
            d._on_save()
        self.assertTrue(crit.called)
        self.assertIsNone(d.result_type)

    def test_a_raising_save_is_reported_not_swallowed(self):
        d = self._dlg()
        d._name.setText("Doomed too")
        with patch.object(self.store, "save_user",
                          side_effect=OSError("read-only")), \
             patch("gui.dialogs.target_type_dialog.QMessageBox.critical") as crit:
            d._on_save()
        self.assertTrue(crit.called)
        self.assertIsNone(d.result_type)


class TestForeignChannelsAreKeptNotRepaired(_DlgCase):
    def test_a_channel_this_rig_lacks_survives_an_edit(self):
        """The store's contract is REPORT, NEVER REPAIR — a rule authored on
        another rig must not be silently rewritten by opening it here."""
        tt = TargetType(
            id="foreign", name="From another rig",
            signature_rule=SignatureRule(clauses=[
                SignatureClause(imaging_channel="Brainbow7",
                                state=STATE_POSITIVE)]))
        d = self._dlg(target_type=tt)
        self.assertEqual(d._rows[0].channel.currentData(), "Brainbow7")
        self.assertIn("not on this rig", d._rows[0].channel.currentText())
        d._on_save()
        back = TargetTypeStore().get("foreign")
        self.assertEqual(back.signature_rule.clauses[0].imaging_channel,
                         "Brainbow7")


class TestClauseRowManagement(_DlgCase):
    def test_adding_and_removing_rows(self):
        d = self._dlg()
        d._add_clause()
        d._add_clause()
        self.assertEqual(len(d._rows), 3)
        d._remove_clause(d._rows[1])
        self.assertEqual(len(d._rows), 2)

    def test_removing_every_row_leaves_an_empty_rule_that_warns(self):
        d = self._dlg()
        d._remove_clause(d._rows[0])
        self.assertEqual(len(d._rows), 0)
        self.assertIn("matches NOTHING", d._rule_note.text())

    def test_removing_an_unknown_row_is_a_no_op(self):
        d = self._dlg()
        before = len(d._rows)
        d._remove_clause(object())
        self.assertEqual(len(d._rows), before)


if __name__ == "__main__":
    unittest.main()
