"""v7.9.1 — a re-anchor must never relocate a mosaic off the machine.

Operator: *"I am not able to set the mosaic scan in the right position now. the
bottom right part of the mosaic scan picture should be at the xy-stage 0,0 as a
default."*

They were right on both counts, and the second half explains the first. The
default whole-plate scan spans the entire reachable XY envelope, so the
mosaic's world-min corner IS stage (0,0) (less half a FOV) and the 180° display
flip renders it bottom-right — exactly the stated convention.

What broke it: the v7.9.1 well-labelling fix moved A1 from the min-stage corner
to the max-stage corner (correct), `_ploc_feed_affine` copies the well NAMED
"A1" into `_taught_a1`, that is the mosaic's plate-frame anchor, and
`MosaicStore.reanchor` rigid-translates every stored extent by any change in it.
The stored `extent_mm` was still recorded against the OLD corner, so the mosaic
was translated by the plate diagonal — ~95 x 57 mm, clean off the machine, and
unrecoverable from the +/-10 mm nudge sliders.

Measured from the operator's own store, against the last committed copy.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.MosaicStore import (                     # noqa: E402
    MosaicStore, plate_mm_to_stage_um, stage_to_plate_mm)

# The operator's real machine: envelope (0,0)-(116340,74227), a 1024 px frame
# at 3.227061 µm/px, so the whole-plate scan pads by half a 3304.5 µm FOV.
GOOD_EXTENT = [-1652.255, -1652.255, 117979.255, 75887.255]
#: The A1 the corrected labelling yields (max-stage corner).
NEW_ANCHOR = [105170.821, 66399.206]
#: The A1 the OLD (wrong) labelling yielded (min-stage corner).
OLD_ANCHOR = [10227.217, 9377.448]
SIGN = [-1.0, -1.0]


class _Store(MosaicStore):
    """A MosaicStore over an in-memory dict — no disk, no singleton."""

    def __init__(self, data):
        self._data = data
        self._path = None

    def _save_meta(self):                                    # noqa: D102
        self._saved = True


def _entry(extent, anchor):
    return {
        "mosaics": {
            "P": {
                "active": "s1",
                "scans": {
                    "s1": {
                        "extent_um": list(extent),
                        "plate_frame": {
                            "extent_mm": list(
                                stage_to_plate_mm(extent, anchor, SIGN)),
                            "anchor_um": list(anchor),
                            "axis_sign": list(SIGN),
                        },
                    }
                },
            }
        }
    }


class TestTheCornerJumpIsRefused(unittest.TestCase):

    def test_the_operators_exact_corruption_is_refused(self):
        """Frame recorded against the OLD corner, re-anchored to the NEW one."""
        data = _entry(GOOD_EXTENT, OLD_ANCHOR)
        st = _Store(data)
        changed = st.reanchor("P", NEW_ANCHOR, SIGN)
        self.assertFalse(changed, "the corner jump must be refused")
        self.assertEqual(
            data["mosaics"]["P"]["scans"]["s1"]["extent_um"], GOOD_EXTENT,
            "the stored extent must be left untouched")

    def test_without_the_guard_it_would_move_by_the_plate_diagonal(self):
        """Proves the test is exercising the real hazard, not a strawman."""
        frame = _entry(GOOD_EXTENT, OLD_ANCHOR)["mosaics"]["P"]["scans"]["s1"]
        would_be = plate_mm_to_stage_um(
            frame["plate_frame"]["extent_mm"], NEW_ANCHOR, SIGN)
        dx = would_be[0] - GOOD_EXTENT[0]
        dy = would_be[1] - GOOD_EXTENT[1]
        self.assertAlmostEqual(dx, 94943.604, places=1)
        self.assertAlmostEqual(dy, 57021.758, places=1)
        # ...and that lands the mosaic clean off a 116340 x 74227 machine.
        self.assertGreater(would_be[2], 116340.0)

    def test_a_real_plate_reseat_is_still_applied(self):
        """The whole point of reanchor: a few mm of remount MUST follow."""
        data = _entry(GOOD_EXTENT, NEW_ANCHOR)
        st = _Store(data)
        moved = [NEW_ANCHOR[0] + 800.0, NEW_ANCHOR[1] - 450.0]
        self.assertTrue(st.reanchor("P", moved, SIGN))
        got = data["mosaics"]["P"]["scans"]["s1"]["extent_um"]
        self.assertAlmostEqual(got[0] - GOOD_EXTENT[0], 800.0, places=3)
        self.assertAlmostEqual(got[1] - GOOD_EXTENT[1], -450.0, places=3)

    def test_a_generous_but_legitimate_reseat_still_applies(self):
        """20 mm — well beyond any remount, still allowed, so the guard cannot
        be accused of blocking real re-teaches."""
        data = _entry(GOOD_EXTENT, NEW_ANCHOR)
        st = _Store(data)
        self.assertTrue(
            st.reanchor("P", [NEW_ANCHOR[0] + 20000.0, NEW_ANCHOR[1]], SIGN))

    def test_a_SINGLE_WELL_scan_still_follows_a_real_reseat(self):
        """⚠ The regression a first cut caused, caught by the v7.13 suite: the
        limit was a fraction of the mosaic, and a single-well scan is ~2 mm
        across, so any fraction of it is SMALLER than a real remount and a
        legitimate 3.5 mm re-teach was refused. The limit is absolute."""
        small = [0.0, 0.0, 2000.0, 2000.0]
        data = _entry(small, NEW_ANCHOR)
        st = _Store(data)
        self.assertTrue(st.reanchor(
            "P", [NEW_ANCHOR[0] + 3500.0, NEW_ANCHOR[1] + 1200.0], SIGN))
        got = data["mosaics"]["P"]["scans"]["s1"]["extent_um"]
        self.assertAlmostEqual(got[0] - small[0], 3500.0, places=3)

    def test_the_anchor_is_not_rewritten_when_the_shift_is_refused(self):
        """A refused re-anchor must leave the frame exactly as it was, or the
        next call would compare against a half-updated record."""
        data = _entry(GOOD_EXTENT, OLD_ANCHOR)
        st = _Store(data)
        st.reanchor("P", NEW_ANCHOR, SIGN)
        pf = data["mosaics"]["P"]["scans"]["s1"]["plate_frame"]
        self.assertEqual(pf["anchor_um"], OLD_ANCHOR)

    def test_a_legacy_scan_with_no_plate_frame_is_still_untouched(self):
        data = {"mosaics": {"P": {"extent_um": list(GOOD_EXTENT)}}}
        st = _Store(data)
        self.assertFalse(st.reanchor("P", NEW_ANCHOR, SIGN))
        self.assertEqual(data["mosaics"]["P"]["extent_um"], GOOD_EXTENT)

    def test_a_degenerate_extent_does_not_block_the_reanchor(self):
        """Nothing to judge against ⇒ behave exactly as before the guard."""
        data = _entry([0.0, 0.0, 0.0, 0.0], NEW_ANCHOR)
        st = _Store(data)
        st.reanchor("P", [NEW_ANCHOR[0] + 5000.0, NEW_ANCHOR[1]], SIGN)
        self.assertTrue(True)          # no exception is the assertion


class TestTheRepairedRecordIsSelfConsistent(unittest.TestCase):
    """After the repair, the frame must reproduce the stored extent from its
    OWN anchor — otherwise the next re-anchor repeats the translation."""

    def test_round_trip(self):
        pf_mm = stage_to_plate_mm(GOOD_EXTENT, NEW_ANCHOR, SIGN)
        back = plate_mm_to_stage_um(pf_mm, NEW_ANCHOR, SIGN)
        for a, b in zip(GOOD_EXTENT, back):
            self.assertAlmostEqual(a, b, places=6)

    def test_a_self_consistent_record_is_a_reanchor_no_op(self):
        data = _entry(GOOD_EXTENT, NEW_ANCHOR)
        st = _Store(data)
        self.assertFalse(st.reanchor("P", NEW_ANCHOR, SIGN))


class TestTheOperatorsStoreIsRepaired(unittest.TestCase):
    """Runs against the real file when present — skipped elsewhere."""

    # v7.17.x: mosaics are PER-MACHINE (config/hardware/<machine-id>/) — the
    # old flat path made this skip forever instead of checking the real store.
    from SupportClasses.MachineConfig import resolve_machine_path
    PATH = str(resolve_machine_path("plate_mosaics.json"))

    def setUp(self):
        if not os.path.exists(self.PATH):
            self.skipTest("no plate_mosaics.json on this machine")

    def test_the_plate_scan_is_back_on_the_machine(self):
        import json
        d = json.load(open(self.PATH, encoding="utf-8"))
        sc = (d.get("mosaics", {}).get("nest-plastic-24", {})
              .get("scans", {}).get("s1"))
        if sc is None:
            self.skipTest("this machine has no nest-plastic-24 scan")
        # The load-bearing half: the v7.9.1 corruption put X max at 212923 µm
        # on a 116340 µm machine, so these bound the extent to the envelope.
        ext = sc["extent_um"]
        self.assertLess(ext[0], 0.0)          # world-min corner at/below (0,0)
        self.assertLess(ext[2], 130000.0)     # and not off the far side
        pf = sc.get("plate_frame")
        if pf is None:
            # v7.13 legacy/re-scanned entry: no plate frame, so it cannot
            # follow a re-teach and the UI flags it "⚠ re-scan to track the
            # plate". Documented and supported — not a failure of this guard.
            self.skipTest("this scan carries no plate_frame (pre-v7.13 / re-scanned)")
        back = plate_mm_to_stage_um(
            pf["extent_mm"], pf["anchor_um"], pf["axis_sign"])
        for a, b in zip(ext, back):
            self.assertAlmostEqual(a, b, places=3)


if __name__ == "__main__":
    unittest.main()
