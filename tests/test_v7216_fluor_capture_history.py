"""v7.21.6 — a fluorescence capture is never overwritten.

Before this release ``save_channel`` wrote ``<plate>_<well>_<channel>.png`` and
assigned ``channels[channel] = <fresh dict>``, so re-scanning A1/FITC truncated
the previous PNG and replaced its whole metadata record. These tests pin the
three properties that fix it:

  * a second capture of the same (plate, well, channel) writes a NEW file and
    the first one's pixels are still readable,
  * the superseded RECORD is archived (so the file is not an orphan), and
  * every existing reader still resolves the ACTIVE capture unchanged.

They drive the production store against a temp path (``MEBP_FLUOR_MOSAIC_PATH``)
so a run never touches the rig's real config/hardware.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import cv2

from SupportClasses import FluorescenceMosaicStore as fms
from SupportClasses.FluorescenceMosaicStore import FluorescenceMosaicStore


def img(value, h=8, w=10):
    a = np.zeros((h, w, 3), np.uint8)
    a[:] = value
    return a


class _Base(unittest.TestCase):
    PLATE = "24"
    WELL = "A1"

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._json = Path(self._tmp.name) / "fluorescence_mosaics.json"
        self._prev = os.environ.get("MEBP_FLUOR_MOSAIC_PATH")
        os.environ["MEBP_FLUOR_MOSAIC_PATH"] = str(self._json)
        self.dir = Path(self._tmp.name) / "fluor_mosaics"

    def tearDown(self):
        if self._prev is None:
            os.environ.pop("MEBP_FLUOR_MOSAIC_PATH", None)
        else:
            os.environ["MEBP_FLUOR_MOSAIC_PATH"] = self._prev
        self._tmp.cleanup()

    def store(self):
        return FluorescenceMosaicStore()

    def save(self, s, value, channel="FITC", **kw):
        """Save with an explicit distinct stamp (no sleep in the test suite)."""
        return s.save_channel(self.PLATE, self.WELL, channel, img(value),
                              (0.0, 0.0, 100.0, 80.0), **kw)

    def pngs(self):
        return sorted(p.name for p in self.dir.glob("*.png"))


class TestNoOverwrite(_Base):
    def test_two_captures_are_two_files_and_both_survive(self):
        s = self.store()
        self.assertTrue(self.save(s, 10, exposure_us=1))
        first = s._channel_meta(self.PLATE, self.WELL, "FITC")["image"]
        self.assertTrue(self.save(s, 200, exposure_us=2))
        second = s._channel_meta(self.PLATE, self.WELL, "FITC")["image"]
        self.assertNotEqual(first, second)
        self.assertEqual(len(self.pngs()), 2, self.pngs())
        # The FIRST capture's pixels are still on disk and unmodified — the
        # regression this release exists for.
        old = cv2.imread(str(self.dir / first.split("/")[-1]))
        self.assertIsNotNone(old)
        self.assertTrue(np.all(old == 10))

    def test_filename_carries_the_capture_stamp(self):
        s = self.store()
        self.save(s, 10)
        name = self.pngs()[0]
        self.assertTrue(name.startswith("24_A1_FITC_"), name)
        stamp = name[len("24_A1_FITC_"):-len(".png")]
        datetime.strptime(stamp, fms.CAPTURE_STAMP_FMT)   # parses => real stamp

    def test_stamp_is_last_so_channel_grouping_still_globs(self):
        s = self.store()
        self.save(s, 10, channel="FITC")
        self.save(s, 10, channel="mCherry")
        self.assertEqual(len(list(self.dir.glob("24_A1_FITC_*.png"))), 1)

    def test_the_stamp_alone_separates_two_captures(self):
        """The no-overwrite guarantee has TWO independent legs; pin each.

        Leg 1 (this test) — the STAMP differs whenever the capture times do, so
        names are meaningful and sort chronologically. Leg 2
        (``test_same_second_capture_does_not_collide``) — the ``-2`` fallback,
        which makes uniqueness absolute even inside one second.

        Driven through ``_unique_image_name`` with explicit times rather than
        two rapid saves: the stamp is second-resolution, so back-to-back saves
        in a unit test genuinely land in the same second and are separated by
        leg 2. A real capture is minutes of rastering, so in production it is
        always leg 1 doing the work — but a test must pin the leg it names.
        """
        s = self.store()
        a = s._unique_image_name(self.PLATE, self.WELL, "FITC",
                                 datetime(2026, 8, 19, 16, 7, 0))
        b = s._unique_image_name(self.PLATE, self.WELL, "FITC",
                                 datetime(2026, 8, 20, 16, 57, 30))
        self.assertNotEqual(a, b, "the stamp is not in the name at all")
        self.assertIn("20260819-160700", a)
        self.assertIn("20260820-165730", b)
        self.assertNotIn("-2.png", a + b)   # leg 1, not leg 2, did this

    def test_same_second_capture_does_not_collide(self):
        s = self.store()
        when = datetime(2026, 8, 20, 17, 0, 0)
        self.dir.mkdir(parents=True, exist_ok=True)
        a = s._unique_image_name(self.PLATE, self.WELL, "FITC", when)
        cv2.imwrite(str(self.dir / a), img(1))
        b = s._unique_image_name(self.PLATE, self.WELL, "FITC", when)
        self.assertNotEqual(a, b)
        self.assertFalse((self.dir / b).exists())

    def test_the_old_naming_would_have_collided(self):
        """Guard the guard.

        The pre-v7.21.6 name had no per-capture component at all, so N captures
        of one channel could only ever be 1 file. Assert both halves: the new
        names are distinct, and neither is the old collision-prone name (a
        change that merely *added* a suffix while still writing the old name
        would leave the overwrite in place).
        """
        legacy = f"{self.PLATE}_{self.WELL}_FITC.png"
        s = self.store()
        self.save(s, 10)
        self.save(s, 20)
        names = self.pngs()
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(len(names), 2)
        self.assertNotIn(legacy, names)


class TestHistory(_Base):
    def test_superseded_record_is_archived_newest_first(self):
        s = self.store()
        for i, v in enumerate((10, 20, 30)):
            self.save(s, v, exposure_us=1000 + i)
        self.assertEqual(s.get_exposure_us(self.PLATE, self.WELL, "FITC"), 1002.0)
        hist = s.list_history(self.PLATE, self.WELL, "FITC")
        self.assertEqual([h["exposure_us"] for h in hist], [1001.0, 1000.0])
        self.assertEqual(s.history_count(self.PLATE, self.WELL, "FITC"), 2)

    def test_history_is_flat_not_nested(self):
        s = self.store()
        for v in (1, 2, 3, 4):
            self.save(s, v)
        for h in s.list_history(self.PLATE, self.WELL, "FITC"):
            self.assertNotIn("history", h)

    def test_history_survives_a_reload(self):
        s = self.store()
        self.save(s, 10)
        self.save(s, 20)
        self.assertEqual(self.store().history_count(self.PLATE, self.WELL, "FITC"), 1)

    def test_every_archived_record_points_at_a_file_that_exists(self):
        s = self.store()
        for v in (10, 20, 30):
            self.save(s, v)
        for h in s.list_history(self.PLATE, self.WELL, "FITC"):
            self.assertTrue((self.dir / h["image"].split("/")[-1]).exists())

    def test_archived_record_keeps_its_own_georeference(self):
        s = self.store()
        s.save_channel(self.PLATE, self.WELL, "FITC", img(10),
                       (1.0, 2.0, 3.0, 4.0), shift_um=(5.0, 6.0), um_per_px=0.5)
        s.save_channel(self.PLATE, self.WELL, "FITC", img(20),
                       (9.0, 9.0, 9.0, 9.0))
        h = s.list_history(self.PLATE, self.WELL, "FITC")[0]
        self.assertEqual(h["extent_um"], [1.0, 2.0, 3.0, 4.0])
        self.assertEqual(h["shift_um"], [5.0, 6.0])
        self.assertEqual(h["um_per_px"], 0.5)

    def test_captured_at_distinguishes_two_captures_on_one_day(self):
        s = self.store()
        self.save(s, 10)
        self.save(s, 20)
        a = s._channel_meta(self.PLATE, self.WELL, "FITC")["captured_at"]
        b = s.list_history(self.PLATE, self.WELL, "FITC")[0]["captured_at"]
        datetime.fromisoformat(a)
        datetime.fromisoformat(b)
        # `date` is still written for every pre-v7.21.6 reader.
        self.assertIn("date", s._channel_meta(self.PLATE, self.WELL, "FITC"))

    def test_restore_history_is_a_reversible_swap(self):
        s = self.store()
        self.save(s, 10, exposure_us=1)
        self.save(s, 20, exposure_us=2)
        self.assertTrue(s.restore_history(self.PLATE, self.WELL, "FITC", 0))
        self.assertEqual(s.get_exposure_us(self.PLATE, self.WELL, "FITC"), 1.0)
        self.assertTrue(np.all(
            s.load_channel_image(self.PLATE, self.WELL, "FITC") == 10))
        # nothing dropped — the displaced capture is now the archived one
        self.assertEqual(s.history_count(self.PLATE, self.WELL, "FITC"), 1)
        self.assertEqual(
            s.list_history(self.PLATE, self.WELL, "FITC")[0]["exposure_us"], 2.0)
        self.assertEqual(len(self.pngs()), 2)

    def test_restore_history_by_image_name(self):
        s = self.store()
        self.save(s, 10, exposure_us=1)
        self.save(s, 20, exposure_us=2)
        target = s.list_history(self.PLATE, self.WELL, "FITC")[0]["image"]
        self.assertTrue(s.restore_history(self.PLATE, self.WELL, "FITC",
                                          target.split("/")[-1]))
        self.assertEqual(s.get_exposure_us(self.PLATE, self.WELL, "FITC"), 1.0)

    def test_restore_history_refuses_a_bad_index(self):
        s = self.store()
        self.save(s, 10)
        self.assertFalse(s.restore_history(self.PLATE, self.WELL, "FITC", 0))
        self.assertFalse(s.restore_history(self.PLATE, self.WELL, "FITC", 99))
        self.assertFalse(s.restore_history(self.PLATE, self.WELL, "nope", 0))

    def test_add_history_entry_sorts_older_captures_into_place(self):
        s = self.store()
        self.save(s, 10)
        rec = dict(s._channel_meta(self.PLATE, self.WELL, "FITC"))
        older = dict(rec, image=rec["image"], date="2020-01-01",
                     captured_at="2020-01-01T00:00:00", exposure_us=7.0)
        newer = dict(rec, image=rec["image"], date="2025-01-01",
                     captured_at="2025-01-01T00:00:00", exposure_us=8.0)
        self.assertTrue(s.add_history_entry(self.PLATE, self.WELL, "FITC", older))
        self.assertTrue(s.add_history_entry(self.PLATE, self.WELL, "FITC", newer))
        self.assertEqual(
            [h["exposure_us"] for h in s.list_history(self.PLATE, self.WELL, "FITC")],
            [8.0, 7.0])


class TestActiveReadersUnchanged(_Base):
    """The whole point of keeping an ACTIVE pointer: no consumer changes."""

    def test_single_capture_reads_exactly_as_before(self):
        s = self.store()
        s.save_channel(self.PLATE, self.WELL, "FITC", img(42),
                       (1.0, 2.0, 3.0, 4.0), shift_um=(5.0, 6.0),
                       um_per_px=0.7, mosaic_scale=0.3, objective="10x")
        self.assertEqual(s.get_extent_um(self.PLATE, self.WELL, "FITC"),
                         (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(s.get_shift_um(self.PLATE, self.WELL, "FITC"), (5.0, 6.0))
        self.assertTrue(s.has_shift(self.PLATE, self.WELL, "FITC"))
        self.assertEqual(s.get_um_per_px(self.PLATE, self.WELL, "FITC"), 0.7)
        self.assertEqual(s.get_mosaic_scale(self.PLATE, self.WELL, "FITC"), 0.3)
        self.assertEqual(s.get_objective(self.PLATE, self.WELL), "10x")
        self.assertEqual(s.list_channels(self.PLATE, self.WELL), ["FITC"])
        self.assertTrue(s.has(self.PLATE, self.WELL))
        self.assertTrue(np.all(
            s.load_channel_image(self.PLATE, self.WELL, "FITC") == 42))

    def test_newest_capture_is_what_readers_see(self):
        s = self.store()
        self.save(s, 10)
        self.save(s, 250)
        self.assertTrue(np.all(
            s.load_channel_image(self.PLATE, self.WELL, "FITC") == 250))
        blended, _ = s.composite_overlay(self.PLATE, self.WELL)
        self.assertIsNotNone(blended)
        self.assertEqual(len(s.list_channels(self.PLATE, self.WELL)), 1)


class TestLegacyEntries(_Base):
    """Pre-v7.21.6 metadata (untimestamped name, no ``history`` key)."""

    def _seed_legacy(self, value=7):
        self.dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.dir / "24_A1_FITC.png"), img(value))
        self._json.write_text(json.dumps({"version": "1.0", "wells": {
            "24|A1": {"plate_key": "24", "well_name": "A1", "channels": {
                "FITC": {"image": "fluor_mosaics/24_A1_FITC.png",
                         "color": [0, 230, 0], "extent_um": [0, 0, 10, 10],
                         "date": "2026-01-01"}}}}}), encoding="utf-8")

    def test_legacy_entry_still_reads(self):
        self._seed_legacy()
        s = self.store()
        self.assertTrue(np.all(
            s.load_channel_image(self.PLATE, self.WELL, "FITC") == 7))
        self.assertEqual(s.get_extent_um(self.PLATE, self.WELL, "FITC"),
                         (0.0, 0.0, 10.0, 10.0))
        self.assertEqual(s.history_count(self.PLATE, self.WELL, "FITC"), 0)
        self.assertFalse(s.has_shift(self.PLATE, self.WELL, "FITC"))

    def test_rescan_over_a_legacy_entry_preserves_the_legacy_file(self):
        self._seed_legacy()
        s = self.store()
        self.save(s, 99)
        self.assertTrue((self.dir / "24_A1_FITC.png").exists())
        self.assertTrue(np.all(
            cv2.imread(str(self.dir / "24_A1_FITC.png")) == 7))
        hist = s.list_history(self.PLATE, self.WELL, "FITC")
        self.assertEqual(len(hist), 1)
        self.assertEqual(hist[0]["image"], "fluor_mosaics/24_A1_FITC.png")


class TestProcessedSiblingPairing(_Base):
    def test_processed_name_derives_from_the_raw_stem(self):
        s = self.store()
        self.save(s, 10)
        self.assertTrue(s.attach_processed(self.PLATE, self.WELL, "FITC",
                                           img(11), {"denoise": True}))
        ch = s._channel_meta(self.PLATE, self.WELL, "FITC")
        self.assertEqual(ch["processed_image"], ch["image"][:-4] + "_proc.png")

    def test_a_new_raw_never_inherits_the_previous_processed_copy(self):
        s = self.store()
        self.save(s, 10)
        s.attach_processed(self.PLATE, self.WELL, "FITC", img(11), {"d": 1})
        stale = s._channel_meta(self.PLATE, self.WELL, "FITC")["processed_image"]
        self.save(s, 200)
        ch = s._channel_meta(self.PLATE, self.WELL, "FITC")
        self.assertNotIn("processed_image", ch)
        # the display path falls back to the NEW raw, not the old processed one
        self.assertTrue(np.all(s.load_channel_image(
            self.PLATE, self.WELL, "FITC", prefer_processed=True) == 200))
        # and the old processed file was not clobbered by a same-named write
        self.assertTrue((self.dir / stale.split("/")[-1]).exists())


class TestClearing(_Base):
    def test_clear_channel_removes_the_archive_too(self):
        s = self.store()
        for v in (10, 20, 30):
            self.save(s, v)
        s.attach_processed(self.PLATE, self.WELL, "FITC", img(1), {})
        self.assertEqual(len(self.pngs()), 4)
        s.clear_channel(self.PLATE, self.WELL, "FITC")
        self.assertEqual(self.pngs(), [])
        self.assertFalse(s.has(self.PLATE, self.WELL))

    def test_clear_channel_can_keep_the_archive_and_promotes_the_next(self):
        s = self.store()
        self.save(s, 10, exposure_us=1)
        self.save(s, 20, exposure_us=2)
        self.save(s, 30, exposure_us=3)
        s.clear_channel(self.PLATE, self.WELL, "FITC", include_history=False)
        self.assertEqual(s.get_exposure_us(self.PLATE, self.WELL, "FITC"), 2.0)
        self.assertEqual(s.history_count(self.PLATE, self.WELL, "FITC"), 1)
        self.assertEqual(len(self.pngs()), 2)

    def test_clear_well_removes_archived_images(self):
        s = self.store()
        self.save(s, 10)
        self.save(s, 20)
        self.save(s, 30, channel="mCherry")
        s.clear_well(self.PLATE, self.WELL)
        self.assertEqual(self.pngs(), [])


class TestConcurrentWriters(_Base):
    """A long-lived store must not revert an external edit.

    ⚠ This is the OTHER overwrite hazard, and it bit for real: the app holds one
    store for the whole session and ``_save_meta`` dumps the entire dict, so an
    import that completed at 17:16 was wiped by the app's 17:28 capture writing
    the file back from startup-era memory.
    """

    def test_external_edit_survives_this_stores_next_write(self):
        live = self.store()
        self.save(live, 10)
        edited = json.loads(self._json.read_text(encoding="utf-8"))
        edited["wells"]["other|Z9"] = {"plate_key": "other", "well_name": "Z9",
                                       "channels": {}}
        self._json.write_text(json.dumps(edited), encoding="utf-8")
        self.save(live, 20)                       # the app's next capture
        after = json.loads(self._json.read_text(encoding="utf-8"))
        self.assertIn("other|Z9", after["wells"])
        # and this store's own new capture is there too — a merge, not a pick
        self.assertEqual(len(after["wells"]["24|A1"]["channels"]["FITC"]
                             .get("history") or []), 1)

    def test_a_second_store_instance_sees_the_first_ones_writes(self):
        a, b = self.store(), self.store()
        self.save(a, 10)
        self.save(b, 20)          # b must archive a's capture, not drop it
        self.assertEqual(b.history_count(self.PLATE, self.WELL, "FITC"), 1)
        self.assertEqual(len(self.pngs()), 2)

    def test_every_mutator_rereads_first(self):
        """AST pin: a new mutator must not forget the guard."""
        import ast
        src = Path(fms.__file__).read_text(encoding="utf-8")
        cls = next(n for n in ast.parse(src).body
                   if isinstance(n, ast.ClassDef)
                   and n.name == "FluorescenceMosaicStore")
        missing = []
        for fn in [n for n in cls.body if isinstance(n, ast.FunctionDef)]:
            calls = {c.func.attr for c in ast.walk(fn)
                     if isinstance(c, ast.Call)
                     and isinstance(c.func, ast.Attribute)}
            if "_save_meta" in calls and "_reload_if_changed" not in calls:
                missing.append(fn.name)
        self.assertEqual(missing, [])


class TestImporter(_Base):
    """The ME3B_2-style folder import: active-if-absent, history-if-present."""

    def _make_source(self):
        src_root = Path(self._tmp.name) / "src"
        (src_root / "fluor_mosaics").mkdir(parents=True)
        cv2.imwrite(str(src_root / "fluor_mosaics" / "24_A1_FITC.png"), img(11))
        cv2.imwrite(str(src_root / "fluor_mosaics" / "24_A1_DAPI.png"), img(22))
        (src_root / "fluorescence_mosaics.json").write_text(json.dumps({
            "version": "1.0", "wells": {"24|A1": {
                "plate_key": "24", "well_name": "A1", "objective": "4x",
                "channels": {
                    "FITC": {"image": "fluor_mosaics/24_A1_FITC.png",
                             "color": [0, 230, 0], "extent_um": [1, 2, 3, 4],
                             "date": "2026-07-08"},
                    "DAPI": {"image": "fluor_mosaics/24_A1_DAPI.png",
                             "color": [0, 0, 255], "extent_um": [5, 6, 7, 8],
                             "date": "2026-07-09"}}}}}), encoding="utf-8")
        return src_root

    def _run_import(self, src_root, extra=()):
        import subprocess
        tool = Path(__file__).resolve().parents[1] / "tools_import_fluor_mosaics.py"
        env = dict(os.environ, MEBP_FLUOR_MOSAIC_PATH=str(self._json))
        return subprocess.run(
            [sys.executable, str(tool), "--from", str(src_root), *extra],
            capture_output=True, text=True, env=env,
            cwd=str(Path(__file__).resolve().parents[1]))

    def test_absent_channel_imports_active_present_one_goes_to_history(self):
        src = self._make_source()
        s = self.store()
        self.save(s, 200, exposure_us=99)          # live FITC, newer
        live_img = s._channel_meta(self.PLATE, self.WELL, "FITC")["image"]
        res = self._run_import(src)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        s2 = self.store()
        # FITC collided → the live capture is still the active one
        self.assertEqual(s2._channel_meta(self.PLATE, self.WELL, "FITC")["image"],
                         live_img)
        self.assertEqual(s2.history_count(self.PLATE, self.WELL, "FITC"), 1)
        self.assertTrue(np.all(np.asarray(cv2.imread(str(
            self.dir / s2.list_history(self.PLATE, self.WELL, "FITC")[0]
            ["image"].split("/")[-1]))) == 11))
        # DAPI was absent → imported active and now visible
        self.assertIn("DAPI", s2.list_channels(self.PLATE, self.WELL))
        self.assertTrue(np.all(
            s2.load_channel_image(self.PLATE, self.WELL, "DAPI") == 22))
        self.assertEqual(s2.get_extent_um(self.PLATE, self.WELL, "DAPI"),
                         (5.0, 6.0, 7.0, 8.0))
        # source untouched
        self.assertTrue((src / "fluor_mosaics" / "24_A1_FITC.png").exists())
        self.assertTrue(np.all(cv2.imread(
            str(src / "fluor_mosaics" / "24_A1_FITC.png")) == 11))

    def test_dry_run_writes_nothing(self):
        src = self._make_source()
        s = self.store()
        self.save(s, 200)
        before = self.pngs()
        res = self._run_import(src, ["--dry-run"])
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        self.assertEqual(self.pngs(), before)
        self.assertNotIn("DAPI", self.store().list_channels(self.PLATE, self.WELL))

    def test_history_only_never_changes_what_is_displayed(self):
        src = self._make_source()
        s = self.store()
        self.save(s, 200)
        active = s._channel_meta(self.PLATE, self.WELL, "FITC")["image"]
        res = self._run_import(src, ["--history-only"])
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        s2 = self.store()
        self.assertEqual(s2._channel_meta(self.PLATE, self.WELL, "FITC")["image"],
                         active)
        self.assertEqual(s2.history_count(self.PLATE, self.WELL, "FITC"), 1)

    def test_a_second_run_is_a_no_op(self):
        """Re-running must not duplicate — the app can revert an import."""
        src = self._make_source()
        self.assertEqual(self._run_import(src).returncode, 0)
        after_first = self.pngs()
        res = self._run_import(src)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        self.assertIn("SKIP-DONE", res.stdout)
        self.assertEqual(self.pngs(), after_first)

    def test_imported_stamp_comes_from_the_records_own_date(self):
        src = self._make_source()
        res = self._run_import(src)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        names = self.pngs()
        self.assertTrue(any("20260708" in n for n in names), names)
        self.assertTrue(any("20260709" in n for n in names), names)


if __name__ == "__main__":
    unittest.main()
