"""
v7.10 — the optical↔needle Z datum stored with the plate-bottom touch-off.

At the moment the operator confirms the needle tip is sitting on the plate
bottom, two heights are true at once: the microscope's focus-axis reading (the
scope is focused on that same plane) and the needle's own Z. Their difference
ties the two axes together, so a later refocus predicts a needle height instead
of requiring another touch-off.

It lives in ``NeedleFocusTemplateStore`` rather than a new store or
``HardwareConfig`` because that store is already per-machine, already keyed by
camera **and objective** — which is exactly what the focus height depends on —
already records ``needle_tip_length_mm`` as invalidating metadata, and already
captures at precisely this instant. Putting it in ``HardwareConfig`` would ship
one rig's optics datum to another in a swapped setup file (the
CAMERA_CAL_PERSIST_STORE rule).

Both numbers are stored RAW rather than pre-differenced, so an inverted or
differently-scaled focus axis stays recoverable by hand.
"""

import json
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from SupportClasses.NeedleFocusTemplateStore import (
    NeedleFocusTemplateStore,
    template_key,
)

KEY = template_key("cam0|10x", "hypodermic", 210.0)


def _patch(n=64):
    return np.full((n, n, 3), 128, dtype=np.uint8)


class _Base(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_optical_datum_")
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.store = NeedleFocusTemplateStore(
            self.root / "needle_focus_templates.json")

    def _add(self, *, focus_um=None, needle_z=None, offset=(0.0, 0.0)):
        return self.store.add_capture(
            KEY, _patch(),
            center_offset_px=(0.0, 0.0),
            center_offset_um=offset,
            um_per_px=1.32,
            microscope_focus_um=focus_um,
            needle_z_user_mm=needle_z)


class TestRoundTrip(_Base):

    def test_the_datum_survives_a_reload(self):
        self.assertTrue(self._add(focus_um=771.175, needle_z=22.564))
        fresh = NeedleFocusTemplateStore(
            self.root / "needle_focus_templates.json")
        cap = fresh.captures(KEY)[0]
        self.assertAlmostEqual(cap["microscope_focus_um"], 771.175, places=6)
        self.assertAlmostEqual(cap["needle_z_user_mm"], 22.564, places=6)

    def test_both_numbers_are_stored_raw_not_differenced(self):
        """A stored difference would be unrecoverable if the focus axis turned
        out to be inverted; raw values can be re-derived by hand."""
        self._add(focus_um=771.175, needle_z=22.564)
        blob = json.loads(
            (self.root / "needle_focus_templates.json").read_text("utf-8"))
        cap = blob["templates"][KEY]["captures"][0]
        self.assertIn("microscope_focus_um", cap)
        self.assertIn("needle_z_user_mm", cap)

    def test_K_is_needle_z_minus_focus_in_mm(self):
        self._add(focus_um=1000.0, needle_z=22.000)
        self.assertAlmostEqual(self.store.focus_to_needle_z_mm(KEY),
                               21.000, places=9)

    def test_K_is_the_mean_across_captures(self):
        self._add(focus_um=1000.0, needle_z=22.000)     # K = 21.000
        self._add(focus_um=2000.0, needle_z=23.020)     # K = 21.020
        self.assertAlmostEqual(self.store.focus_to_needle_z_mm(KEY),
                               21.010, places=9)

    def test_the_invariant_predicts_a_needle_height(self):
        """The whole point: refocus the scope, get the needle Z that reaches it."""
        self._add(focus_um=1000.0, needle_z=22.000)
        k = self.store.focus_to_needle_z_mm(KEY)
        predicted = k + 1500.0 / 1000.0
        self.assertAlmostEqual(predicted, 22.500, places=9)


class TestDegradation(_Base):
    """Absent is absent — never zero, never a guess."""

    def test_no_capture_at_all_is_None(self):
        self.assertIsNone(self.store.focus_to_needle_z_mm(KEY))

    def test_a_capture_without_a_focus_axis_is_None(self):
        """A rig with no motorised focus still writes a useful capture."""
        self.assertTrue(self._add(focus_um=None, needle_z=22.0))
        self.assertIsNone(self.store.focus_to_needle_z_mm(KEY))
        self.assertEqual(len(self.store.captures(KEY)), 1)

    def test_a_capture_without_a_needle_z_is_skipped(self):
        self._add(focus_um=1000.0, needle_z=None)
        self.assertIsNone(self.store.focus_to_needle_z_mm(KEY))

    def test_a_pre_v710_capture_is_skipped_not_crashed(self):
        """Simulate a file written before the fields existed."""
        self._add(focus_um=None, needle_z=None)
        blob_path = self.root / "needle_focus_templates.json"
        blob = json.loads(blob_path.read_text("utf-8"))
        cap = blob["templates"][KEY]["captures"][0]
        cap.pop("microscope_focus_um", None)
        cap.pop("needle_z_user_mm", None)
        blob_path.write_text(json.dumps(blob), encoding="utf-8")
        fresh = NeedleFocusTemplateStore(blob_path)
        self.assertIsNone(fresh.focus_to_needle_z_mm(KEY))
        self.assertEqual(len(fresh.captures(KEY)), 1)

    def test_mixed_captures_use_only_the_ones_that_have_it(self):
        self._add(focus_um=None, needle_z=None)
        self._add(focus_um=1000.0, needle_z=22.000)
        self.assertAlmostEqual(self.store.focus_to_needle_z_mm(KEY),
                               21.000, places=9)

    def test_a_non_finite_value_is_rejected(self):
        self._add(focus_um=float("nan"), needle_z=22.0)
        self.assertIsNone(self.store.focus_to_needle_z_mm(KEY))


class TestSpread(_Base):
    """A disagreeing focus axis must be surfaced, not averaged away."""

    def test_none_below_two_samples(self):
        self._add(focus_um=1000.0, needle_z=22.0)
        self.assertIsNone(self.store.focus_to_needle_z_spread_mm(KEY))

    def test_consistent_captures_have_a_tiny_spread(self):
        self._add(focus_um=1000.0, needle_z=22.000)
        self._add(focus_um=2000.0, needle_z=23.000)     # same K
        self.assertAlmostEqual(
            self.store.focus_to_needle_z_spread_mm(KEY), 0.0, places=9)

    def test_an_inverted_focus_axis_shows_a_large_spread(self):
        """If the scope's focus counts the other way, K wanders — which is
        exactly the signature this reader exists to expose."""
        self._add(focus_um=1000.0, needle_z=22.000)     # K = 21.000
        self._add(focus_um=2000.0, needle_z=21.000)     # K = 19.000
        spread = self.store.focus_to_needle_z_spread_mm(KEY)
        self.assertGreater(spread, 0.9)


class TestBackCompat(_Base):

    def test_omitting_the_new_kwargs_still_works(self):
        """Every existing caller passes neither — they must be optional."""
        ok = self.store.add_capture(
            KEY, _patch(),
            center_offset_px=(1.0, 2.0),
            center_offset_um=(3.0, 4.0),
            um_per_px=1.32)
        self.assertTrue(ok)
        self.assertEqual(self.store.needle_center_offset_um(KEY), (3.0, 4.0))

    def test_the_offset_reader_is_unaffected(self):
        self._add(focus_um=1000.0, needle_z=22.0, offset=(10.0, -20.0))
        self.assertEqual(self.store.needle_center_offset_um(KEY), (10.0, -20.0))


if __name__ == "__main__":
    unittest.main()
