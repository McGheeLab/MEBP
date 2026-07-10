"""test_v75x_sketch_abstract_inks.py — Print Builder Sketch: abstract inks.

The Sketch is now pump-AGNOSTIC. It carries an ordered list of abstract inks
(``Sketch.inks`` of :class:`SketchInk`, stable ids) and each shape references one
by ``ink_id`` (NOT a physical pump). Legacy sketches with ``pump_index`` migrate
(P1→1, P2→2, P3→3). The compiler's 3 trajectory pump columns are a PREVIEW
artifact keyed by each ink's order index % 3; the single-needle weld break keys
on the true ``ink_id``.

Backend tests need no GUI; the page test runs offscreen.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import json
import sys
import unittest

import numpy as np

from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, SketchInk, compile_to_trajectory,
    count_discontinuities, optimize_print_order, plan_print_sections,
)
from SupportClasses.PhysicalModels import NeedleSpec


def _needle(num_channels=1):
    n = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
    n.num_channels = num_channels
    return n


def _line(x1, y1, x2, y2, ink_id=1):
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)], ink_id=ink_id)


# ═══════════════════════════════════════════════════════════════════
# Model + serialization + migration
# ═══════════════════════════════════════════════════════════════════

class TestSketchInk(unittest.TestCase):

    def test_ink_roundtrip(self):
        ink = SketchInk(id=3, name="support", color="#abcdef")
        self.assertEqual(SketchInk.from_dict(ink.to_dict()),
                         SketchInk(3, "support", "#abcdef"))

    def test_empty_sketch_has_default_ink(self):
        sk = Sketch()
        self.assertEqual(len(sk.inks), 1)
        self.assertEqual(sk.inks[0].id, 1)
        self.assertEqual(sk._next_ink_id, 2)

    def test_add_ink_ids_stable_and_not_recycled(self):
        sk = Sketch()
        a = sk.add_ink("A")
        b = sk.add_ink("B")
        self.assertEqual([i.id for i in sk.inks], [1, a.id, b.id])
        self.assertEqual([a.id, b.id], [2, 3])
        # Delete the highest id, add another — the id must NOT be recycled.
        sk.inks = [i for i in sk.inks if i.id != b.id]
        c = sk.add_ink("C")
        self.assertEqual(c.id, 4)               # 3 is gone forever
        self.assertNotIn(c.id, (a.id, b.id))


class TestLegacyMigration(unittest.TestCase):

    def test_pump_index_migrates_to_ink_id(self):
        d = {"shapes": [
            {"kind": "line", "points": [[0, 0], [5, 0]], "pump_index": 0},
            {"kind": "line", "points": [[5, 0], [10, 0]], "pump_index": 1},
            {"kind": "line", "points": [[10, 0], [15, 0]], "pump_index": 2},
        ]}
        sk = Sketch.from_dict(d)
        self.assertEqual([s.ink_id for s in sk.shapes], [1, 2, 3])
        # Inks synthesized from the migrated ids.
        self.assertEqual(sorted(i.id for i in sk.inks), [1, 2, 3])

    def test_legacy_compiles_same_as_ink_ids(self):
        legacy = Sketch.from_dict({"shapes": [
            {"kind": "circle", "cx": 0, "cy": 0, "radius": 4, "pump_index": 0},
            {"kind": "circle", "cx": 20, "cy": 0, "radius": 4, "pump_index": 1},
        ]})
        explicit = Sketch(shapes=[
            SketchShape(kind="circle", cx=0, cy=0, radius=4, ink_id=1),
            SketchShape(kind="circle", cx=20, cy=0, radius=4, ink_id=2),
        ])
        rl = compile_to_trajectory(legacy)
        re = compile_to_trajectory(explicit)
        self.assertAlmostEqual(rl.total_length_mm, re.total_length_mm, places=6)
        self.assertAlmostEqual(rl.total_volume_uL, re.total_volume_uL, places=6)

    def test_real_saved_prints_load_and_compile(self):
        base = os.path.join(os.path.dirname(__file__), "..", "config", "prints")
        loaded = 0
        for name in os.listdir(base) if os.path.isdir(base) else []:
            if not name.endswith(".json"):
                continue
            try:
                with open(os.path.join(base, name), encoding="utf-8") as fh:
                    doc = json.load(fh)
            except Exception:
                continue
            for sk_dict in _find_sketches(doc):
                sk = Sketch.from_dict(sk_dict)
                self.assertTrue(sk.inks)                    # never empty
                res = compile_to_trajectory(sk, _needle())  # must not raise
                self.assertIsNotNone(res)
                loaded += 1
        # At least the known embedded-sketch prints should have been exercised.
        self.assertGreaterEqual(loaded, 1)


def _find_sketches(obj):
    """Yield every embedded sketch dict (has a ``shapes`` list) in a print doc."""
    if isinstance(obj, dict):
        if isinstance(obj.get("shapes"), list) and "z_start_mm" in obj:
            yield obj
        for v in obj.values():
            yield from _find_sketches(v)
    elif isinstance(obj, list):
        for v in obj:
            yield from _find_sketches(v)


# ═══════════════════════════════════════════════════════════════════
# Compiler: preview column vs weld gate
# ═══════════════════════════════════════════════════════════════════

class TestPreviewColumn(unittest.TestCase):

    def _sk4(self):
        sk = Sketch()
        sk.inks = [SketchInk(i, f"Ink {i}", "#89b4fa") for i in range(1, 5)]
        sk._next_ink_id = 5
        sk.shapes = [_line(0, 0, 5, 0, ink_id=i) for i in range(1, 5)]
        return sk

    def test_column_is_order_index_mod3(self):
        sk = self._sk4()
        self.assertEqual([sk.ink_order_index(i) % 3 for i in (1, 2, 3, 4)],
                         [0, 1, 2, 0])

    def test_volume_independent_of_ink_count(self):
        one = compile_to_trajectory(
            Sketch(shapes=[_line(0, 0, 10, 0, ink_id=1)]), _needle())
        two = compile_to_trajectory(
            Sketch(shapes=[_line(0, 0, 10, 0, ink_id=2)]), _needle())
        self.assertAlmostEqual(one.total_volume_uL, two.total_volume_uL,
                               places=6)


class TestSingleNeedleWeldGate(unittest.TestCase):

    def test_gate_keys_on_ink_not_column(self):
        # Inks whose order indices are 0 and 3 (ids 1 and 4) collide to the SAME
        # preview column 0, but a single-needle print must still break between
        # them (they are different materials).
        sk = Sketch(single_needle=True)
        sk.inks = [SketchInk(i, f"Ink {i}", "#89b4fa") for i in range(1, 5)]
        sk._next_ink_id = 5
        sk.shapes = [_line(0, 0, 5, 0, ink_id=1), _line(5, 0, 10, 0, ink_id=4)]
        self.assertEqual(count_discontinuities(sk, _needle(1)), 1)

    def test_same_ink_connected_welds(self):
        sk = Sketch(single_needle=True,
                    shapes=[_line(0, 0, 5, 0, ink_id=1),
                            _line(5, 0, 10, 0, ink_id=1)])
        self.assertEqual(count_discontinuities(sk, _needle(1)), 0)


# ═══════════════════════════════════════════════════════════════════
# plan_print_sections + optimizer
# ═══════════════════════════════════════════════════════════════════

class TestPlanAndOptimize(unittest.TestCase):

    def test_section_carries_ink_id_and_valid_column(self):
        sk = Sketch(shapes=[_line(0, 0, 5, 0, ink_id=1),
                            _line(5, 0, 10, 0, ink_id=2)])
        secs = [p for p in plan_print_sections(sk, _needle(1))
                if p["type"] == "section"]
        self.assertEqual([s["ink_id"] for s in secs], [1, 2])
        for s in secs:
            self.assertIn(s["pump_index"], (0, 1, 2))
        self.assertEqual(secs[1]["break_before"], "ink_change")

    def test_optimizer_groups_by_ink(self):
        sk = Sketch(shapes=[_line(0, 0, 1, 0, ink_id=1),
                            _line(20, 0, 21, 0, ink_id=2),
                            _line(2, 0, 3, 0, ink_id=1),
                            _line(22, 0, 23, 0, ink_id=2)],
                    line_spacing_mm=0.4)
        opt = optimize_print_order(sk, needle=_needle(1))
        inks = [s.ink_id for s in opt.shapes]
        self.assertIn(inks, ([1, 1, 2, 2], [2, 2, 1, 1]))
        self.assertEqual([i.id for i in opt.inks], [i.id for i in sk.inks])


class TestSerialization(unittest.TestCase):

    def test_to_dict_carries_inks_and_next_id(self):
        sk = Sketch()
        sk.add_ink("second")
        d = sk.to_dict()
        self.assertEqual(len(d["inks"]), 2)
        self.assertEqual(d["_next_ink_id"], 3)

    def test_explicit_inks_take_precedence_over_legacy_synth(self):
        d = {"shapes": [{"kind": "line", "points": [[0, 0], [5, 0]],
                         "pump_index": 0}],
             "inks": [{"id": 7, "name": "custom", "color": "#010203"}],
             "_next_ink_id": 8}
        sk = Sketch.from_dict(d)
        self.assertEqual([i.id for i in sk.inks], [7])
        self.assertEqual(sk._next_ink_id, 8)


# ═══════════════════════════════════════════════════════════════════
# Page (offscreen) — ink manager
# ═══════════════════════════════════════════════════════════════════

class TestPageInkManager(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.print_builder_sketch import SketchPage
        return SketchPage()

    def test_add_ink_grows_the_list(self):
        p = self._page()
        n0 = len(p._canvas.sketch().inks)
        p._add_ink()
        self.assertEqual(len(p._canvas.sketch().inks), n0 + 1)

    def test_per_shape_ink_combo_lists_all_inks(self):
        p = self._page()
        sk = p._canvas.sketch()
        sk.add_ink("B")
        sh = SketchShape(kind="line", points=[(0, 0), (5, 0)], ink_id=1)
        combo = p._make_ink_combo(sh)
        self.assertEqual(combo.count(), len(sk.inks))

    def test_delete_ink_reassigns_shapes(self):
        p = self._page()
        sk = p._canvas.sketch()
        b = sk.add_ink("B")
        sk.shapes = [SketchShape(kind="line", points=[(0, 0), (5, 0)],
                                 ink_id=b.id)]
        p._delete_ink(b)
        self.assertNotIn(b.id, [i.id for i in sk.inks])
        self.assertEqual(sk.shapes[0].ink_id, sk.inks[0].id)  # reassigned


if __name__ == "__main__":
    unittest.main()
