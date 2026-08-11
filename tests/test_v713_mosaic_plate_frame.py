"""
v7.13 — a mosaic is referenced to the PLATE as well as the stage, and a plate
may hold several scans.

Operator: *"in the plate selection tool, we have the ability to select the
mosaic that will be used for that plate if there are multiple. make sure that
the mosaic is referenced to the plate coordinate system and the xy stage
coordinate system. this allows a recalibration later to offset the mosaic to
the live view which we already have. in the plate selection tool, we should be
able to see the mosaic scan that we are selecting."*

Before this, ``MosaicStore`` recorded only where a mosaic sat on the STAGE, and
one scan per plate. Nothing recorded what the image was OF, so re-seating a
plate meant dragging the mosaic back by hand.

The load-bearing invariant, and the reason this was shippable: **the public
store API did not change.** ``get_meta`` and everything built on it resolve to
the active scan and return the pre-v7.13 shape, so the nine consumer modules
and eleven other test files that use this store were not touched.
"""

from __future__ import annotations

import ast
import inspect
import json
import os
import shutil
import tempfile
import textwrap
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt
import numpy as np

try:
    import cv2
    _CV2 = True
except ImportError:                                        # pragma: no cover
    _CV2 = False

from SupportClasses.MosaicStore import (
    MosaicStore, FIRST_SCAN_ID, SCHEMA_VERSION,
    stage_to_plate_mm, plate_mm_to_stage_um,
)
from SupportClasses.WellPlate import (
    WellPlate, PLATE_FOOTPRINT_X_MM, PLATE_FOOTPRINT_Y_MM,
)
from SupportClasses.PlateDocument import PlateDocument, WellStyle


ANCHOR = (105617.6, 65890.4)          # this machine's taught A1, absolute µm
SIGN = (-1.0, -1.0)                   # ME3B V1: plate mounted 180° to the stage
EXTENT = (10000.0, 20000.0, 110000.0, 70000.0)


def _img(w=30, h=20, value=128):
    return np.full((h, w, 3), value, np.uint8)


class _StoreCase(unittest.TestCase):
    def setUp(self):
        if not _CV2:                                       # pragma: no cover
            self.skipTest("cv2 unavailable")
        self.dir = Path(tempfile.mkdtemp())
        self.store = MosaicStore(self.dir / "plate_mosaics.json")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def _pngs(self):
        d = self.dir / "mosaics"
        return sorted(p.name for p in d.iterdir()) if d.exists() else []


# ── The boundary that must not move ───────────────────────────────

class TestPreV713FilesReadUnchanged(_StoreCase):
    """A v1.0 file must behave exactly as it always did.

    Nine production modules read this store through ``get_meta`` and friends.
    If any of them saw a different shape after this change, the change would
    have to touch all nine — which is how a refactor of this size goes wrong.
    """

    def setUp(self):
        super().setUp()
        (self.dir / "mosaics").mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.dir / "mosaics" / "24.png"), _img())
        (self.dir / "plate_mosaics.json").write_text(json.dumps({
            "version": "1.0",
            "mosaics": {
                "24": {"extent_um": list(EXTENT), "um_per_px": 3.34,
                       "mosaic_scale": 0.02, "image": "mosaics/24.png",
                       "frames": 9, "shift_um": [11.0, 22.0],
                       "date": "2026-01-01",
                       "wells_um": {"A1": [1500.0, 2500.0]}},
                "24#A1": {"extent_um": [1.0, 2.0, 3.0, 4.0],
                          "image": "mosaics/24.png"},
            }}))
        self.store = MosaicStore(self.dir / "plate_mosaics.json")

    def test_every_legacy_reader_still_works(self):
        s = self.store
        self.assertTrue(s.has("24"))
        self.assertEqual(s.get_extent_um("24"), EXTENT)
        self.assertEqual(s.get_shift_um("24"), (11.0, 22.0))
        self.assertEqual(s.get_wells("24"), {"A1": (1500.0, 2500.0)})
        self.assertIsNotNone(s.image_path("24"))
        self.assertIsNotNone(s.load_image("24"))
        self.assertEqual(s.list_plate_keys(), ["24"])
        self.assertEqual(s.get_meta("24")["frames"], 9)

    def test_get_meta_still_returns_a_flat_scan_dict(self):
        meta = self.store.get_meta("24")
        self.assertIn("extent_um", meta)
        self.assertNotIn("scans", meta)
        self.assertNotIn("active", meta)

    def test_single_well_entries_stay_bare_leaves(self):
        entry = self.store._data["mosaics"]["24#A1"]
        self.assertNotIn("scans", entry)
        self.assertEqual(self.store.get_extent_um("24#A1"), (1.0, 2.0, 3.0, 4.0))

    def test_legacy_scan_is_surfaced_as_one_named_scan(self):
        scans = self.store.list_scans("24")
        self.assertEqual(len(scans), 1)
        self.assertEqual(scans[0]["id"], FIRST_SCAN_ID)
        self.assertTrue(scans[0]["active"])
        self.assertTrue(scans[0]["needs_rescan"])

    def test_legacy_scan_reports_needs_rescan(self):
        """A plate frame cannot be invented for it: doing so would assume the
        calibration live when it was taken, and a wrong assumption puts every
        well centre mapped off the mosaic silently in the wrong place."""
        self.assertTrue(self.store.needs_rescan("24"))
        self.assertIsNone(self.store.plate_frame("24"))

    def test_writing_does_not_disturb_the_well_entry(self):
        self.store.save("24", _img(), EXTENT, anchor_um=ANCHOR, axis_sign=SIGN)
        reloaded = MosaicStore(self.dir / "plate_mosaics.json")
        self.assertEqual(reloaded.get_extent_um("24#A1"), (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(reloaded._data["version"], SCHEMA_VERSION)


# ── Frame arithmetic ──────────────────────────────────────────────

class TestFrameMath(unittest.TestCase):

    def test_round_trip_is_exact_for_every_axis_sign(self):
        for sign in ((1, 1), (-1, -1), (1, -1), (-1, 1)):
            mm = stage_to_plate_mm(EXTENT, ANCHOR, sign)
            back = plate_mm_to_stage_um(mm, ANCHOR, sign)
            for a, b in zip(EXTENT, back):
                self.assertAlmostEqual(a, b, places=6, msg=f"sign={sign}")

    def test_a_negative_sign_still_yields_a_normalised_rectangle(self):
        """A flip swaps the corners; an un-normalised extent silently breaks
        every ``min_x``-style consumer downstream."""
        mm = stage_to_plate_mm(EXTENT, ANCHOR, (-1.0, -1.0))
        self.assertLess(mm[0], mm[2])
        self.assertLess(mm[1], mm[3])

    def test_translation_follows_the_anchor_one_to_one(self):
        """A rigid plate translation moves the mosaic identically — the axis
        sign cancels across the round trip. (It matters only if the plate is
        MOUNTED differently between capture and replay.)"""
        for sign in ((1, 1), (-1, -1)):
            mm = stage_to_plate_mm(EXTENT, ANCHOR, sign)
            moved = plate_mm_to_stage_um(
                mm, (ANCHOR[0] + 3500.0, ANCHOR[1] - 1200.0), sign)
            self.assertAlmostEqual(moved[0] - EXTENT[0], 3500.0, places=6)
            self.assertAlmostEqual(moved[1] - EXTENT[1], -1200.0, places=6)


# ── Several scans per plate ───────────────────────────────────────

class TestMultipleScans(_StoreCase):

    def _two(self):
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR,
                        axis_sign=SIGN, name="Brightfield 4x", frames=100)
        self.store.save("plt", _img(value=200),
                        (12000.0, 22000.0, 112000.0, 72000.0),
                        anchor_um=ANCHOR, axis_sign=SIGN, scan_id="s2",
                        name="Brightfield 10x", frames=400)

    def test_a_second_scan_does_not_overwrite_the_first(self):
        self._two()
        self.assertEqual(len(self.store.list_scans("plt")), 2)
        self.assertEqual(len(self._pngs()), 2)

    def test_the_newest_scan_becomes_active(self):
        self._two()
        self.assertEqual(self.store.active_scan_id("plt"), "s2")
        self.assertEqual(self.store.get_extent_um("plt"),
                         (12000.0, 22000.0, 112000.0, 72000.0))

    def test_switching_active_switches_what_every_reader_sees(self):
        self._two()
        self.assertTrue(self.store.set_active_scan("plt", "s1"))
        self.assertEqual(self.store.get_extent_um("plt"), EXTENT)
        self.assertEqual(self.store.get_meta("plt")["frames"], 100)

    def test_switching_to_an_unknown_scan_is_refused(self):
        self._two()
        self.assertFalse(self.store.set_active_scan("plt", "nope"))
        self.assertEqual(self.store.active_scan_id("plt"), "s2")

    def test_rename(self):
        self._two()
        self.assertTrue(self.store.rename_scan("plt", "s1", "Overview"))
        names = {x["id"]: x["name"] for x in self.store.list_scans("plt")}
        self.assertEqual(names["s1"], "Overview")
        self.assertFalse(self.store.rename_scan("plt", "s1", "   "))

    def test_delete_removes_the_scan_and_its_image(self):
        self._two()
        self.assertTrue(self.store.delete_scan("plt", "s2"))
        self.assertEqual([x["id"] for x in self.store.list_scans("plt")], ["s1"])
        self.assertEqual(len(self._pngs()), 1)

    def test_deleting_the_active_scan_promotes_a_survivor(self):
        self._two()
        self.store.delete_scan("plt", "s2")
        self.assertEqual(self.store.active_scan_id("plt"), "s1")
        self.assertTrue(self.store.has("plt"))

    def test_deleting_the_last_scan_leaves_the_plate_with_no_mosaic(self):
        self.store.save("plt", _img(), EXTENT)
        self.store.delete_scan("plt", FIRST_SCAN_ID)
        self.assertFalse(self.store.has("plt"))
        self.assertEqual(self.store.list_scans("plt"), [])

    def test_clear_removes_every_scan_and_every_image(self):
        self._two()
        self.store.clear("plt")
        self.assertFalse(self.store.has("plt"))
        self.assertEqual(self._pngs(), [])

    def test_scans_survive_a_reload(self):
        self._two()
        self.store.set_active_scan("plt", "s1")
        again = MosaicStore(self.dir / "plate_mosaics.json")
        self.assertEqual(len(again.list_scans("plt")), 2)
        self.assertEqual(again.active_scan_id("plt"), "s1")
        self.assertEqual(again.get_extent_um("plt"), EXTENT)

    def test_no_scans_lists_empty_rather_than_raising(self):
        self.assertEqual(self.store.list_scans("nothing"), [])
        self.assertEqual(self.store.active_scan_id("nothing"), "")
        self.assertFalse(self.store.set_active_scan("nothing", "s1"))


# ── The plate reference ───────────────────────────────────────────

class TestPlateFrameReference(_StoreCase):

    def test_saving_with_an_anchor_records_both_frames(self):
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR, axis_sign=SIGN)
        self.assertEqual(self.store.get_extent_um("plt"), EXTENT)
        frame = self.store.plate_frame("plt")
        self.assertIsNotNone(frame)
        self.assertEqual(tuple(frame["anchor_um"]), ANCHOR)
        self.assertEqual(tuple(frame["axis_sign"]), SIGN)
        self.assertFalse(self.store.needs_rescan("plt"))

    def test_saving_without_an_anchor_stays_stage_only(self):
        """An uncalibrated plate has no anchor to reference against, and
        inventing one would be worse than recording none."""
        self.store.save("plt", _img(), EXTENT)
        self.assertIsNone(self.store.plate_frame("plt"))
        self.assertTrue(self.store.needs_rescan("plt"))
        self.assertEqual(self.store.get_extent_um("plt"), EXTENT)

    def test_a_malformed_anchor_degrades_to_stage_only(self):
        self.store.save("plt", _img(), EXTENT, anchor_um=("x", None))
        self.assertIsNone(self.store.plate_frame("plt"))
        self.assertEqual(self.store.get_extent_um("plt"), EXTENT)

    def test_get_extent_um_is_byte_identical_without_an_anchor(self):
        """Every pre-v7.13 caller passes no anchor and must be unaffected."""
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR, axis_sign=SIGN)
        self.assertEqual(self.store.get_extent_um("plt"), EXTENT)
        self.assertEqual(self.store.get_extent_um("plt", anchor_um=None),
                         EXTENT)

    def test_get_extent_um_replaces_against_a_supplied_anchor(self):
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR, axis_sign=SIGN)
        moved = self.store.get_extent_um(
            "plt", anchor_um=(ANCHOR[0] + 1000.0, ANCHOR[1]))
        self.assertAlmostEqual(moved[0] - EXTENT[0], 1000.0, places=6)

    def test_a_legacy_scan_ignores_a_supplied_anchor(self):
        self.store.save("plt", _img(), EXTENT)
        self.assertEqual(
            self.store.get_extent_um("plt", anchor_um=(1.0, 2.0)), EXTENT)


class TestReanchor(_StoreCase):
    """The point of the whole change: a re-teach carries the mosaic."""

    def setUp(self):
        super().setUp()
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR,
                        axis_sign=SIGN, name="4x")
        self.store.save("plt", _img(), (12000.0, 22000.0, 112000.0, 72000.0),
                        anchor_um=ANCHOR, axis_sign=SIGN, scan_id="s2",
                        name="10x")
        self.store.save("plt#A1", _img(), (5000.0, 6000.0, 7000.0, 8000.0),
                        anchor_um=ANCHOR, axis_sign=SIGN)
        self.store.save("old", _img(), EXTENT)          # legacy, stage only
        self.moved = (ANCHOR[0] + 3500.0, ANCHOR[1] - 1200.0)

    def test_every_scan_of_the_plate_moves_with_the_anchor(self):
        self.assertTrue(self.store.reanchor("plt", self.moved, SIGN))
        for scan_id, was in (("s1", EXTENT),
                             ("s2", (12000.0, 22000.0, 112000.0, 72000.0))):
            self.store.set_active_scan("plt", scan_id)
            now = self.store.get_extent_um("plt")
            self.assertAlmostEqual(now[0] - was[0], 3500.0, places=3)
            self.assertAlmostEqual(now[1] - was[1], -1200.0, places=3)

    def test_single_well_scans_follow_too(self):
        """They are bare leaves rather than containers — the shape the first
        cut of ``reanchor`` did not handle."""
        self.assertTrue(self.store.reanchor("plt#A1", self.moved, SIGN))
        now = self.store.get_extent_um("plt#A1")
        self.assertAlmostEqual(now[0] - 5000.0, 3500.0, places=3)

    def test_legacy_scans_are_left_alone(self):
        self.assertFalse(self.store.reanchor("old", self.moved, SIGN))
        self.assertEqual(self.store.get_extent_um("old"), EXTENT)

    def test_reanchoring_twice_changes_nothing_the_second_time(self):
        self.assertTrue(self.store.reanchor("plt", self.moved, SIGN))
        self.assertFalse(self.store.reanchor("plt", self.moved, SIGN))

    def test_the_plate_frame_itself_is_unchanged_by_a_reanchor(self):
        """Where the image sits ON THE PLATE is what a re-teach does not
        alter; only its stage placement is re-derived."""
        before = list(self.store.plate_frame("plt")["extent_mm"])
        self.store.reanchor("plt", self.moved, SIGN)
        after = list(self.store.plate_frame("plt")["extent_mm"])
        for a, b in zip(before, after):
            self.assertAlmostEqual(a, b, places=6)

    def test_reanchor_survives_a_reload(self):
        self.store.reanchor("plt", self.moved, SIGN)
        expected = self.store.get_extent_um("plt")
        again = MosaicStore(self.dir / "plate_mosaics.json")
        self.assertEqual(again.get_extent_um("plt"), expected)

    def test_no_anchor_is_a_no_op(self):
        self.assertFalse(self.store.reanchor("plt", None))

    def test_copy_carries_the_plate_reference(self):
        """A copy that arrived looking legacy would report needs_rescan even
        though its source was fully referenced."""
        self.assertGreater(self.store.copy_plate("plt", "other"), 0)
        self.assertIsNotNone(self.store.plate_frame("other"))
        self.assertFalse(self.store.needs_rescan("other"))


# ── The footprint bug ─────────────────────────────────────────────

class TestFootprintAwareCentring(unittest.TestCase):
    """``get_a1_from_plate_center`` centred every plate using the hardcoded
    ANSI footprint, so a carrier that is not 127.76 x 85.48 landed off by half
    the difference."""

    CX, CY = 50000.0, 40000.0

    def _outline_centre(self, plate):
        a1x, a1y = plate.get_a1_from_plate_center(self.CX, self.CY)
        fw, fh = plate.footprint_mm
        return (a1x - plate.a1_offset_x * 1000.0 + fw * 1000.0 / 2.0,
                a1y - plate.a1_offset_y * 1000.0 + fh * 1000.0 / 2.0)

    def test_standard_plates_are_unchanged(self):
        for fmt in (6, 24, 96, 384):
            plate = WellPlate.from_format(fmt)
            self.assertAlmostEqual(plate.footprint_mm[0], PLATE_FOOTPRINT_X_MM)
            self.assertAlmostEqual(plate.footprint_mm[1], PLATE_FOOTPRINT_Y_MM)

    def test_the_plate_outline_lands_exactly_on_the_requested_centre(self):
        for fmt in (6, 24, 96):
            cx, cy = self._outline_centre(WellPlate.from_format(fmt))
            self.assertAlmostEqual(cx, self.CX, places=3)
            self.assertAlmostEqual(cy, self.CY, places=3)

    def test_a_non_ansi_carrier_is_centred_by_its_own_footprint(self):
        doc = PlateDocument.new_plate("Small carrier")
        doc.boundary.width_mm, doc.boundary.height_mm = 60.0, 40.0
        doc.boundary.a1_x_mm, doc.boundary.a1_y_mm = 10.0, 8.0
        doc.add_grid(0.0, 0.0, rows=2, cols=2, pitch_x_mm=20.0,
                     pitch_y_mm=15.0, style=WellStyle(diameter_mm=8.0))
        plate = doc.compile()
        # Unsaved doc: nothing to resolve from, so it falls back to ANSI.
        self.assertAlmostEqual(plate.footprint_mm[0], PLATE_FOOTPRINT_X_MM)
        object.__setattr__(plate, "_footprint_cache", (60.0, 40.0))
        cx, cy = self._outline_centre(plate)
        self.assertAlmostEqual(cx, self.CX, places=3)
        self.assertAlmostEqual(cy, self.CY, places=3)

    def test_the_old_hardcoded_constant_would_have_been_far_out(self):
        """Quantifies what the fix buys, so the test says why it exists."""
        err_x = (PLATE_FOOTPRINT_X_MM - 60.0) / 2.0
        err_y = (PLATE_FOOTPRINT_Y_MM - 40.0) / 2.0
        self.assertGreater(err_x, 30.0)
        self.assertGreater(err_y, 20.0)

    def test_footprint_is_cached(self):
        plate = WellPlate.from_format(24)
        self.assertEqual(plate.footprint_mm, plate.footprint_mm)
        self.assertIsNotNone(getattr(plate, "_footprint_cache", None))


# ── The selector ──────────────────────────────────────────────────

class TestPlateSelectorUI(_StoreCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _doc(self):
        doc = PlateDocument.new_plate("Selector fixture")
        doc.add_grid(7.388, 9.894, rows=2, cols=2, pitch_x_mm=40.0,
                     pitch_y_mm=40.0, style=WellStyle(diameter_mm=28.0))
        return doc

    def test_mosaic_pixmap_round_trips_an_image(self):
        from gui.pages.hardware.plate_library import mosaic_pixmap
        self.store.save("plt", _img(60, 40), EXTENT)
        pm = mosaic_pixmap(self.store, "plt")
        self.assertIsNotNone(pm)
        self.assertEqual((pm.width(), pm.height()), (60, 40))

    def test_mosaic_pixmap_is_downscaled(self):
        from gui.pages.hardware.plate_library import mosaic_pixmap
        self.store.save("plt", _img(2000, 1000), EXTENT)
        pm = mosaic_pixmap(self.store, "plt", max_px=200)
        self.assertLessEqual(max(pm.width(), pm.height()), 200)

    def test_mosaic_pixmap_is_none_without_a_scan(self):
        from gui.pages.hardware.plate_library import mosaic_pixmap
        self.assertIsNone(mosaic_pixmap(self.store, "nothing"))
        self.assertIsNone(mosaic_pixmap(None, "plt"))

    def test_extent_is_converted_out_of_the_A1_WELL_frame(self):
        """The scan's plate frame is relative to the A1 WELL; the thumbnail
        draws in the document's storage frame. On this fixture the grid seed
        sits 7.388 / 9.894 mm from the datum, so the two differ by exactly
        that — the frame mix-up this conversion exists to prevent."""
        from gui.pages.hardware.plate_library import mosaic_extent_in_doc_frame
        from SupportClasses.PlateDocumentStore import a1_well_offset_mm
        doc = self._doc()
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR, axis_sign=SIGN)
        raw = self.store.plate_frame("plt")["extent_mm"]
        converted = mosaic_extent_in_doc_frame(self.store, "plt", doc)
        dx, dy = a1_well_offset_mm(doc)
        self.assertAlmostEqual(dx, 7.388, places=3)
        self.assertAlmostEqual(dy, 9.894, places=3)
        self.assertAlmostEqual(converted[0], raw[0] + dx, places=6)
        self.assertAlmostEqual(converted[1], raw[1] + dy, places=6)

    def test_a_legacy_scan_has_no_placeable_extent(self):
        """Stretching it to the outline would show a registration that does
        not exist."""
        from gui.pages.hardware.plate_library import mosaic_extent_in_doc_frame
        self.store.save("plt", _img(), EXTENT)
        self.assertIsNone(
            mosaic_extent_in_doc_frame(self.store, "plt", self._doc()))

    def test_thumbnail_ignores_a_mosaic_with_no_extent(self):
        from gui.pages.hardware.plate_library import (
            PlateThumbnail, mosaic_pixmap)
        self.store.save("plt", _img(), EXTENT)
        th = PlateThumbnail()
        th.resize(200, 120)
        th.set_geometry([(0.0, 0.0, 28.0)], (-20.0, -20.0, 60.0, 60.0))
        th.set_mosaic(mosaic_pixmap(self.store, "plt"), None)
        self.assertIsNone(th._mosaic)
        self.assertFalse(th.grab().isNull())

    def test_thumbnail_paints_with_a_mosaic(self):
        from gui.pages.hardware.plate_library import (
            PlateThumbnail, mosaic_extent_in_doc_frame, mosaic_pixmap)
        doc = self._doc()
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR, axis_sign=SIGN)
        th = PlateThumbnail()
        th.resize(240, 140)
        th.set_geometry([(w.x, w.y, w.diameter_mm) for w in doc.evaluate()],
                        doc.boundary.extent_a1())
        th.set_mosaic(mosaic_pixmap(self.store, "plt"),
                      mosaic_extent_in_doc_frame(self.store, "plt", doc))
        self.assertIsNotNone(th._mosaic)
        self.assertFalse(th.grab().isNull())

    def test_picker_lists_every_scan(self):
        from gui.pages.hardware.plate_library import MosaicPickerDialog
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR,
                        axis_sign=SIGN, name="4x")
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR,
                        axis_sign=SIGN, scan_id="s2", name="10x")
        dlg = MosaicPickerDialog(self.store, "plt")
        self.assertEqual(dlg._list.count(), 2)
        self.assertFalse(dlg.grab().isNull())

    def test_previewing_a_scan_does_not_change_which_one_the_plate_uses(self):
        """Preview resolves through the active slot, so it has to put the
        active slot back — otherwise merely LOOKING at a scan would silently
        re-point the plate at it."""
        from gui.pages.hardware.plate_library import MosaicPickerDialog
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR,
                        axis_sign=SIGN, name="4x")
        self.store.save("plt", _img(), EXTENT, anchor_um=ANCHOR,
                        axis_sign=SIGN, scan_id="s2", name="10x")
        self.assertEqual(self.store.active_scan_id("plt"), "s2")
        dlg = MosaicPickerDialog(self.store, "plt")

        # Land on the row that is NOT active and stay there. Ending the sweep
        # on the active row would leave the right answer for the wrong reason —
        # a mutation removing the restore survived exactly that.
        target = None
        for row in range(dlg._list.count()):
            dlg._list.setCurrentRow(row)
            dlg._render()
            if dlg._list.item(row).data(Qt.UserRole) == "s1":
                target = row
        self.assertIsNotNone(target)
        dlg._list.setCurrentRow(target)
        dlg._render()
        self.assertEqual(dlg._selected(), "s1", "fixture is not discriminating")
        self.assertEqual(self.store.active_scan_id("plt"), "s2")

    def test_use_this_scan_switches_the_active_slot(self):
        from gui.pages.hardware.plate_library import MosaicPickerDialog
        self.store.save("plt", _img(), EXTENT, name="4x")
        self.store.save("plt", _img(), EXTENT, scan_id="s2", name="10x")
        dlg = MosaicPickerDialog(self.store, "plt")
        for row in range(dlg._list.count()):
            if dlg._list.item(row).data(Qt.UserRole) == "s1":
                dlg._list.setCurrentRow(row)
        dlg._use()
        self.assertEqual(self.store.active_scan_id("plt"), "s1")
        self.assertTrue(dlg.changed)

    def test_picker_on_a_plate_with_no_scan(self):
        from gui.pages.hardware.plate_library import MosaicPickerDialog
        dlg = MosaicPickerDialog(self.store, "empty")
        self.assertEqual(dlg._list.count(), 0)
        self.assertFalse(dlg.grab().isNull())


# ── Wiring ────────────────────────────────────────────────────────

class TestCalibrationPageWiring(unittest.TestCase):

    def test_every_mosaic_save_records_the_plate_anchor(self):
        """Checked by AST, not by a substring: a source-text search matches an
        import line or a comment, which has produced a green-but-blind guard in
        this repo twice before."""
        with open("gui/pages/calibration.py", encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        # A mosaic save is `store.save(key, image, extent, mosaic_scale=…)`.
        # Matching on the name `save` alone also catches
        # PlateDocumentStore.save(doc) and the objective store's save, so key
        # off `mosaic_scale`, which only this signature takes.
        saves = []
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "save"
                    and len(node.args) >= 3):
                continue
            if "mosaic_scale" in {k.arg for k in node.keywords}:
                saves.append(node)
        # Self-guard: a matcher that finds nothing would pass vacuously.
        self.assertEqual(len(saves), 3,
                         "expected exactly the 3 known mosaic save sites; "
                         "a new one must also record the plate anchor")
        for call in saves:
            kwargs = {k.arg for k in call.keywords}
            self.assertIn("anchor_um", kwargs,
                          f"save at line {call.lineno} does not record the "
                          f"plate anchor")

    def test_the_page_exposes_a_plate_anchor_helper(self):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage.__new__(CalibrationPage)
        page._taught_a1 = None
        self.assertEqual(page._ploc_plate_anchor(), (None, (1.0, 1.0)))

    def test_no_taught_a1_means_no_plate_frame(self):
        """An uncalibrated plate must store stage-frame-only rather than
        anchoring the mosaic to a guess."""
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage.__new__(CalibrationPage)
        page._taught_a1 = None
        anchor, _sign = page._ploc_plate_anchor()
        self.assertIsNone(anchor)

    def test_reanchor_is_called_from_the_calibration_commit(self):
        from gui.pages.calibration import CalibrationPage
        src = textwrap.dedent(
            inspect.getsource(CalibrationPage._save_calibration))
        tree = ast.parse(src)
        self.assertTrue(
            any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "reanchor" for n in ast.walk(tree)),
            "_save_calibration no longer re-anchors the plate's mosaics")


if __name__ == "__main__":
    unittest.main()
