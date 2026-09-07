"""v7.21 — the pure layout model: promoted sections + card order.

No Qt. The whole point of the store being duck-typed on ``Settings`` is that
these run in milliseconds against a dict, so a broken order-merge cannot hide
behind a GUI test.

The load-bearing property is :func:`merge_order`: a stored order is a HINT, never
an authority, so no stale or hand-edited value can make a card disappear from a
page. Every test here is really about that.
"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.WorkflowLayoutStore import (   # noqa: E402
    SECTION, WorkflowLayoutStore, merge_order, move_in_order,
)


class FakeSettings:
    """Only what the store touches."""

    def __init__(self, data=None):
        self.data = dict(data or {})
        self.saves = 0

    def get_section(self, section):
        got = self.data.get(section, {})
        return dict(got) if isinstance(got, dict) else {}

    def set_section(self, section, value):
        self.data[section] = dict(value)

    def save(self):
        self.saves += 1


class TestMergeOrder(unittest.TestCase):
    def test_the_result_is_always_a_permutation_of_what_exists(self):
        for stored in ([], ["a"], ["c", "b", "a"], ["z", "a"], None,
                       ["a", "a", "b"]):
            got = merge_order(stored, ["a", "b", "c"])
            self.assertEqual(sorted(got), ["a", "b", "c"],
                             msg=f"stored={stored!r}")

    def test_stored_order_is_honoured(self):
        self.assertEqual(merge_order(["c", "a", "b"], ["a", "b", "c"]),
                         ["c", "a", "b"])

    def test_an_id_that_no_longer_exists_is_dropped(self):
        self.assertEqual(merge_order(["gone", "b", "a"], ["a", "b"]),
                         ["b", "a"])

    def test_a_NEW_card_appears_instead_of_vanishing(self):
        """The property that lets a later version add a card without every
        operator's stored order hiding it."""
        self.assertEqual(merge_order(["b", "a"], ["a", "b", "brand_new"]),
                         ["b", "a", "brand_new"])

    def test_a_duplicate_in_the_stored_order_cannot_duplicate_a_card(self):
        self.assertEqual(merge_order(["a", "a", "b"], ["a", "b"]), ["a", "b"])

    def test_ids_are_compared_as_strings(self):
        self.assertEqual(merge_order([1, 2], ["2", "1"]), ["1", "2"])


class TestMoveInOrder(unittest.TestCase):
    def test_moves_up_and_down(self):
        self.assertEqual(move_in_order(["a", "b", "c"], "c", -1),
                         ["a", "c", "b"])
        self.assertEqual(move_in_order(["a", "b", "c"], "a", +1),
                         ["b", "a", "c"])

    def test_clamped_at_both_ends_never_wraps(self):
        """A card that jumped from top to bottom on one click reads as a bug."""
        self.assertEqual(move_in_order(["a", "b"], "a", -1), ["a", "b"])
        self.assertEqual(move_in_order(["a", "b"], "b", +1), ["a", "b"])
        self.assertEqual(move_in_order(["a", "b", "c"], "a", -9),
                         ["a", "b", "c"])
        self.assertEqual(move_in_order(["a", "b", "c"], "a", +9),
                         ["b", "c", "a"])

    def test_an_unknown_id_is_a_no_op(self):
        self.assertEqual(move_in_order(["a", "b"], "zzz", -1), ["a", "b"])


class TestPromotion(unittest.TestCase):
    def _store(self, data=None):
        st = FakeSettings(data)
        return WorkflowLayoutStore(st), st

    def test_nothing_promoted_by_default(self):
        store, _ = self._store()
        self.assertEqual(store.promoted("wf"), [])
        self.assertFalse(store.is_promoted("wf", "anything"))

    def test_promote_then_demote_round_trips(self):
        store, st = self._store()
        store.set_promoted("wf", "prep", True)
        self.assertEqual(store.promoted("wf"), ["prep"])
        self.assertTrue(store.is_promoted("wf", "prep"))
        store.set_promoted("wf", "prep", False)
        self.assertEqual(store.promoted("wf"), [])
        self.assertGreaterEqual(st.saves, 2)

    def test_a_redundant_set_does_not_write(self):
        """Promotion is re-applied on every launch (restore_promotions); writing
        each time would churn settings.json on startup."""
        store, st = self._store()
        store.set_promoted("wf", "prep", True)
        n = st.saves
        store.set_promoted("wf", "prep", True)
        store.set_promoted("wf", "other", False)
        self.assertEqual(st.saves, n)

    def test_workflows_are_independent(self):
        store, _ = self._store()
        store.set_promoted("a", "one", True)
        store.set_promoted("b", "two", True)
        self.assertEqual(store.promoted("a"), ["one"])
        self.assertEqual(store.promoted("b"), ["two"])

    def test_order_and_promotion_coexist_in_one_entry(self):
        store, st = self._store()
        store.set_promoted("wf", "prep", True)
        store.set_order("wf", ["prep", "queue"])
        self.assertEqual(store.promoted("wf"), ["prep"])
        self.assertEqual(store.order("wf"), ["prep", "queue"])
        self.assertIn("wf", st.data[SECTION])

    def test_collapsed_is_tracked_per_section(self):
        store, _ = self._store()
        self.assertFalse(store.is_collapsed("wf", "prep"))
        store.set_collapsed("wf", "prep", True, save=False)
        self.assertTrue(store.is_collapsed("wf", "prep"))

    def test_resolved_order_goes_through_merge(self):
        store, _ = self._store()
        store.set_order("wf", ["c", "gone", "a"])
        self.assertEqual(store.resolved_order("wf", ["a", "b", "c"]),
                         ["c", "a", "b"])


class TestTolerance(unittest.TestCase):
    """A hand-edited or older settings.json must never raise."""

    def test_garbage_section_reads_as_empty(self):
        for junk in ("nope", 5, None, [], {"wf": "not-a-dict"}):
            store = WorkflowLayoutStore(FakeSettings({SECTION: junk}))
            self.assertEqual(store.promoted("wf"), [])
            self.assertEqual(store.order("wf"), [])
            self.assertEqual(store.collapsed("wf"), {})

    def test_garbage_lists_are_filtered(self):
        store = WorkflowLayoutStore(FakeSettings(
            {SECTION: {"wf": {"promoted": ["ok", "", "ok", 7],
                              "order": "not-a-list",
                              "collapsed": {"a": "yes"}}}}))
        self.assertEqual(store.promoted("wf"), ["ok", "7"])
        self.assertEqual(store.order("wf"), [])
        self.assertEqual(store.collapsed("wf"), {"a": True})

    def test_no_settings_object_is_inert_not_fatal(self):
        """A page can be built before settings exist; the store must degrade to
        'remembers nothing' rather than raising into the ctor."""
        store = WorkflowLayoutStore(None)
        store.set_promoted("wf", "x", True)
        store.set_order("wf", ["a"])
        self.assertEqual(store.promoted("wf"), [])
        self.assertEqual(store.resolved_order("wf", ["a", "b"]), ["a", "b"])

    def test_a_raising_settings_object_is_swallowed(self):
        class Boom:
            def get_section(self, _s):
                raise RuntimeError("boom")

            def set_section(self, _s, _v):
                raise RuntimeError("boom")

            def save(self):
                raise RuntimeError("boom")

        store = WorkflowLayoutStore(Boom())
        self.assertEqual(store.promoted("wf"), [])
        store.set_promoted("wf", "x", True)          # must not raise


class TestNoQtDependency(unittest.TestCase):
    def test_the_store_module_imports_no_qt(self):
        import SupportClasses.WorkflowLayoutStore as mod
        src = open(mod.__file__, encoding="utf-8").read()
        self.assertNotIn("PySide6", src)
        self.assertNotIn("QtWidgets", src)


if __name__ == "__main__":
    unittest.main()
