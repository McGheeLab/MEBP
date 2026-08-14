"""test_v719_print_queue_model.py — the pure plate-wide print-queue model.

``SupportClasses.PrintQueue`` is Qt-free and controller-free, so ordering, ink-swap
counting and the (de)serialization tolerance are all testable here in milliseconds
with no ``QApplication``, no plate and no hardware. That matters because this is
where the expensive mistakes live: a wrong order costs an hour of wash cycles, and
a snapshot that loses a field silently prints the wrong thing.

No Qt import anywhere in this file — that is itself part of the contract.
"""

import unittest

from SupportClasses.PrintQueue import (
    ORDER_INK, ORDER_PLATE, QueuedPrint, count_ink_swaps, get,
    plate_index_from_names, order_queue, queue_from_state, queue_to_state,
    remove, upsert, wells,
)


def _qp(well, **kw):
    return QueuedPrint(well=well, **kw)


class TestSnapshotRoundTrip(unittest.TestCase):
    def test_every_field_round_trips(self):
        qp = QueuedPrint(
            well="B4", object_data="file:spiral", object_label="spiral",
            size_mm=2.75, pump="P3", ink_name="Alginate", ink_dip_z_mm=1.25,
            ink_padding_uL=0.5, top_speed_mm_s=4.5, resolution_um=45.0,
            print_z_mm=0.35, extrusion_mod=1.4, motion_mode="confirm",
            ink_map_by_name={"Struct": "Alginate", "Support": "Gelatin"},
            enabled=False)
        self.assertEqual(QueuedPrint.from_dict(qp.to_dict()), qp)

    def test_an_unknown_key_is_filtered_not_splatted(self):
        """A newer build's extra key must not crash an older one — the hazard
        NeedleSpec.from_dict and PickPlaceTarget.from_dict were hardened against."""
        d = _qp("A1").to_dict()
        d["some_future_field"] = {"nested": [1, 2]}
        self.assertEqual(QueuedPrint.from_dict(d).well, "A1")

    def test_copy_is_independent(self):
        """Stamping one print into forty wells must not share one object, or
        editing A17 edits all forty."""
        a = _qp("A1", ink_map_by_name={"x": "y"})
        b = a.copy()
        b.well = "A2"
        b.print_z_mm = 9.0
        b.ink_map_by_name["x"] = "CHANGED"
        self.assertEqual(a.well, "A1")
        self.assertNotEqual(a.print_z_mm, 9.0)
        self.assertEqual(a.ink_map_by_name["x"], "y")

    def test_label_names_the_well_object_and_ink(self):
        lbl = _qp("C3", object_label="grid", ink_name="Gelatin").label()
        for part in ("C3", "grid", "Gelatin"):
            self.assertIn(part, lbl)


class TestFromDictTolerance(unittest.TestCase):
    """A queue restored from a profile runs inside a page's settings-apply, so a
    malformed entry must degrade to defaults rather than take the page down."""

    def test_non_dict_inputs(self):
        for junk in (None, "", 5, [], (), object()):
            qp = QueuedPrint.from_dict(junk)
            self.assertEqual(qp.well, "")

    def test_empty_dict_gives_defaults(self):
        qp = QueuedPrint.from_dict({})
        self.assertEqual(qp.well, "")
        self.assertEqual(qp.pump, "P1")
        self.assertAlmostEqual(qp.print_z_mm, 0.20)

    def test_wrongly_typed_values_fall_back(self):
        qp = QueuedPrint.from_dict({
            "well": 5, "top_speed_mm_s": "fast", "size_mm": None,
            "print_z_mm": [], "ink_map_by_name": "nope", "pump": "",
        })
        self.assertEqual(qp.well, "5")
        self.assertAlmostEqual(qp.top_speed_mm_s, 2.5)
        self.assertAlmostEqual(qp.size_mm, 1.0)
        self.assertEqual(qp.ink_map_by_name, {})
        self.assertEqual(qp.pump, "P1")

    def test_nan_and_inf_are_refused(self):
        """A NaN print height would reach a Z move; inf would reach a clamp."""
        qp = QueuedPrint.from_dict({
            "well": "A1", "print_z_mm": float("nan"),
            "top_speed_mm_s": float("inf")})
        self.assertAlmostEqual(qp.print_z_mm, 0.20)
        self.assertAlmostEqual(qp.top_speed_mm_s, 2.5)

    def test_ink_map_keys_survive_a_json_string_round_trip(self):
        """The map is keyed by ink NAME precisely because JSON turns int keys
        into strings — an int-keyed map would silently break on reload."""
        import json
        qp = _qp("A1", ink_map_by_name={"Struct": "Alginate"})
        back = QueuedPrint.from_dict(json.loads(json.dumps(qp.to_dict())))
        self.assertEqual(back.ink_map_by_name, {"Struct": "Alginate"})


class TestQueueState(unittest.TestCase):
    def test_round_trip(self):
        q = [_qp("A1", print_z_mm=0.4), _qp("B2", ink_name="Gelatin")]
        back = queue_from_state(queue_to_state(q))
        self.assertEqual(back, q)

    def test_garbage_states(self):
        for junk in (None, "str", 5, {"schema": 1}, {"prints": "nope"},
                     {"prints": 5}, object()):
            self.assertEqual(queue_from_state(junk), [])

    def test_a_bare_list_is_accepted(self):
        self.assertEqual(
            [q.well for q in queue_from_state([{"well": "A1"}])], ["A1"])

    def test_only_the_unparseable_entries_are_dropped(self):
        state = {"prints": [{"well": "A1"}, "junk", 5, {"well": ""},
                            {"well": "B2"}]}
        self.assertEqual([q.well for q in queue_from_state(state)],
                         ["A1", "B2"])

    def test_a_duplicate_well_is_collapsed(self):
        """The well IS the key — two entries for one well would mean two stacked
        prints, which is a different (unbuilt) feature."""
        state = {"prints": [{"well": "A1", "print_z_mm": 1.0},
                            {"well": "A1", "print_z_mm": 2.0}]}
        got = queue_from_state(state)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0].print_z_mm, 1.0)


class TestUpsert(unittest.TestCase):
    def test_replaces_in_place(self):
        q = [_qp("A1"), _qp("A2"), _qp("A3")]
        q2 = upsert(q, _qp("A2", print_z_mm=5.0))
        self.assertEqual(wells(q2), ["A1", "A2", "A3"])   # position preserved
        self.assertAlmostEqual(get(q2, "A2").print_z_mm, 5.0)
        self.assertEqual(len(q2), 3)

    def test_appends_a_new_well(self):
        self.assertEqual(wells(upsert([_qp("A1")], _qp("B9"))), ["A1", "B9"])

    def test_does_not_mutate_the_input(self):
        q = [_qp("A1")]
        upsert(q, _qp("A2"))
        self.assertEqual(len(q), 1)

    def test_remove_and_get(self):
        q = [_qp("A1"), _qp("A2")]
        self.assertEqual(wells(remove(q, "A1")), ["A2"])
        self.assertIsNone(get(q, "ZZ"))
        self.assertEqual(wells(remove(q, "nope")), ["A1", "A2"])


class TestOrdering(unittest.TestCase):
    def setUp(self):
        # Two inks alternating across two rows — the case where grouping pays.
        self.q = [_qp("A1", ink_name="X"), _qp("A2", ink_name="Y"),
                  _qp("B1", ink_name="X"), _qp("B2", ink_name="Y")]
        self.pi = plate_index_from_names(["A1", "A2", "B1", "B2"])

    def test_plate_order_is_row_major(self):
        self.assertEqual(
            wells(order_queue(self.q, ORDER_PLATE, self.pi)),
            ["A1", "A2", "B1", "B2"])

    def test_ink_order_groups_and_breaks_ties_by_plate_order(self):
        self.assertEqual(
            wells(order_queue(self.q, ORDER_INK, self.pi)),
            ["A1", "B1", "A2", "B2"])

    def test_grouping_by_ink_cuts_the_swap_count(self):
        """The whole justification for offering the toggle."""
        as_units = lambda order: [{"ink_name": qp.ink_name} for qp in order]
        plate = count_ink_swaps(
            as_units(order_queue(self.q, ORDER_PLATE, self.pi)))
        ink = count_ink_swaps(
            as_units(order_queue(self.q, ORDER_INK, self.pi)))
        self.assertEqual((plate, ink), (3, 1))

    def test_both_modes_are_pure_permutations(self):
        for mode in (ORDER_PLATE, ORDER_INK):
            self.assertEqual(sorted(wells(order_queue(self.q, mode, self.pi))),
                             sorted(wells(self.q)))

    def test_ink_ranks_come_from_first_appearance_so_it_is_stable(self):
        a = wells(order_queue(self.q, ORDER_INK, self.pi))
        b = wells(order_queue(list(self.q), ORDER_INK, self.pi))
        self.assertEqual(a, b)

    def test_an_unknown_well_sorts_last_and_is_kept(self):
        """Dropping it here would hide it; the run gating reports it by name."""
        q = self.q + [_qp("ZZ99", ink_name="X")]
        out = wells(order_queue(q, ORDER_PLATE, self.pi))
        self.assertEqual(out[-1], "ZZ99")
        self.assertEqual(len(out), 5)

    def test_no_plate_index_keeps_the_authored_order(self):
        self.assertEqual(wells(order_queue(self.q, ORDER_PLATE, None)),
                         wells(self.q))

    def test_a_raising_plate_index_does_not_propagate(self):
        def boom(_well):
            raise RuntimeError("no plate")
        self.assertEqual(len(order_queue(self.q, ORDER_PLATE, boom)), 4)

    def test_empty(self):
        self.assertEqual(order_queue([], ORDER_INK, self.pi), [])
        self.assertEqual(order_queue(None, ORDER_PLATE, self.pi), [])


class TestSwapCounting(unittest.TestCase):
    def test_the_first_unit_never_costs_a_swap(self):
        """After the single up-front prep the needle is clean and buffer-loaded."""
        self.assertEqual(count_ink_swaps([{"ink_name": "X"}]), 0)

    def test_counts_changes_only(self):
        self.assertEqual(
            count_ink_swaps([{"ink_name": "X"}, {"ink_name": "X"},
                             {"ink_name": "Y"}, {"ink_name": "Y"}]), 1)

    def test_a_sketch_using_one_ink_non_contiguously_really_costs_two(self):
        """A(1) B(2) C(1) — its own cost, which must be counted, not optimised."""
        self.assertEqual(
            count_ink_swaps([{"ink_name": "a"}, {"ink_name": "b"},
                             {"ink_name": "a"}]), 2)

    def test_force_clean_between_charges_every_boundary(self):
        units = [{"ink_name": "X"}] * 4
        self.assertEqual(count_ink_swaps(units), 0)
        self.assertEqual(count_ink_swaps(units, force_clean_between=True), 3)

    def test_empty_and_none(self):
        self.assertEqual(count_ink_swaps([]), 0)
        self.assertEqual(count_ink_swaps(None), 0)


class TestPlateIndex(unittest.TestCase):
    def test_maps_names_to_positions(self):
        pi = plate_index_from_names(["A1", "A2", "B1"])
        self.assertEqual((pi("A1"), pi("B1")), (0, 2))
        self.assertIsNone(pi("nope"))

    def test_empty_names(self):
        self.assertIsNone(plate_index_from_names(None)("A1"))


class TestNoQtDependency(unittest.TestCase):
    def test_the_module_imports_no_qt(self):
        """Qt-free is the contract that makes every branch above cheap to test."""
        import inspect
        import SupportClasses.PrintQueue as mod
        src = inspect.getsource(mod)
        self.assertNotIn("PySide6", src)
        self.assertNotIn("QtCore", src)


if __name__ == "__main__":
    unittest.main()
