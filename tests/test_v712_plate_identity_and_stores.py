"""v7.12 Phase 1 — document store, identity, and per-plate store keys.

The invariant under test throughout: **a plate's identity is its id, and
renaming it must cost nothing.** Under the old scheme the display name WAS the
key of eight per-plate stores, so a rename silently orphaned every taught
calibration, mosaic and training set — and two such orphans are still sitting in
this machine's ``settings.json``.
"""
from __future__ import annotations

import re
import tempfile
import unittest
from pathlib import Path

from SupportClasses.PlateDocument import PlateDocument, WellStyle, new_doc_id
from SupportClasses.PlateDocumentStore import (
    DocSummary, PlateDocumentStore, plate_store_keys,
)
from SupportClasses.WellPlate import WellPlate


class _StoreCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.plates = PlateDocumentStore(kind="plate", user_dir=root / "plates")
        self.rosettes = PlateDocumentStore(kind="rosette",
                                           user_dir=root / "rosettes",
                                           builtin_dir=root / "ros_builtin")

    def tearDown(self):
        self._tmp.cleanup()


# ═══════════════════════════════════════════════════════════════════

class TestStoreCrud(_StoreCase):
    def test_create_list_get(self):
        doc = self.plates.create("My 24-well", template=24)
        self.assertTrue(self.plates.exists(doc.meta.id))
        summaries = self.plates.list()
        self.assertEqual(1, len(summaries))
        s = summaries[0]
        self.assertEqual("My 24-well", s.name)
        self.assertEqual(24, s.well_count)
        self.assertEqual(0, s.rosette_count)
        self.assertIs(doc, self.plates.get(doc.meta.id))

    def test_create_blank_and_from_template_id(self):
        base = self.plates.create("base", template=96)
        forked = self.plates.create("forked", template=base.meta.id)
        self.assertNotEqual(base.meta.id, forked.meta.id)
        self.assertEqual(96, len(forked.evaluate()))

    def test_writes_are_atomic(self):
        doc = self.plates.create("atomic", template=6)
        self.assertEqual([], list(self.plates.user_dir.glob("*.tmp")))
        self.assertTrue((self.plates.user_dir / f"{doc.meta.id}.json").exists())

    def test_the_filename_is_the_id_not_the_name(self):
        doc = self.plates.create("Name With Spaces & Punctuation!", template=6)
        files = [p.stem for p in self.plates.user_dir.glob("*.json")]
        self.assertEqual([doc.meta.id], files)

    def test_delete(self):
        doc = self.plates.create("gone", template=6)
        self.assertTrue(self.plates.delete(doc.meta.id))
        self.assertFalse(self.plates.delete(doc.meta.id))
        self.assertEqual([], self.plates.list())

    def test_malformed_and_legacy_files_are_skipped_not_fatal(self):
        good = self.plates.create("good", template=6)
        (self.plates.user_dir / "garbage.json").write_text("{not json")
        (self.plates.user_dir / "legacy.json").write_text(
            '{"schema_version": "1.0", "name": "old design"}')
        names = [s.name for s in self.plates.list()]
        self.assertEqual(["good"], names)

    def test_thumbnail_wells_needs_no_compile(self):
        doc = self.plates.create("thumb", template=24)
        wells = self.plates.thumbnail_wells(doc.meta.id)
        self.assertEqual(24, len(wells))
        self.assertEqual(3, len(wells[0]))

    def test_rosette_store_shadowing(self):
        self.rosettes._builtin_dir.mkdir(parents=True, exist_ok=True)
        b = PlateDocument.new_rosette(name="Builtin ring")
        b.meta.id = "ros_builtin1"
        b.add_ring(0.0, 0.0, count=3, ring_diameter_mm=6.0)
        b.save(self.rosettes._builtin_dir / "ros_builtin1.json")

        self.assertEqual("Builtin ring", self.rosettes.list()[0].name)
        self.assertTrue(self.rosettes.is_builtin("ros_builtin1"))

        override = PlateDocument.from_dict(b.to_dict())
        override.meta.name = "User override"
        self.rosettes.save(override)
        self.rosettes.invalidate()
        self.assertEqual(["User override"],
                         [s.name for s in self.rosettes.list()])
        self.assertFalse(self.rosettes.is_builtin("ros_builtin1"))


# ═══════════════════════════════════════════════════════════════════

class TestRenamePreservesIdentity(_StoreCase):
    def test_rename_changes_nothing_but_the_label(self):
        doc = self.plates.create("before", template=24)
        doc_id, path = doc.meta.id, self.plates._path_for(doc.meta.id)

        self.assertTrue(self.plates.rename(doc_id, "after"))

        self.plates.invalidate()
        again = self.plates.get(doc_id)
        self.assertEqual("after", again.meta.name)
        self.assertEqual(doc_id, again.meta.id)
        self.assertTrue(path.exists())                    # no file moved
        self.assertEqual([path.name],
                         [p.name for p in self.plates.user_dir.glob("*.json")])

    def test_rename_leaves_the_store_key_untouched(self):
        """The whole point. Under the old scheme this is where the taught
        calibration was lost."""
        from SupportClasses.HardwareConfig import HardwareConfig
        doc = self.plates.create("before", template=24)
        cfg = HardwareConfig()
        cfg.plate_doc_id = doc.meta.id
        key_before = str(cfg.active_plate_key)

        self.plates.rename(doc.meta.id, "a totally different name")
        self.assertEqual(key_before, str(cfg.active_plate_key))

    def test_duplicate_display_names_are_allowed(self):
        a = self.plates.create("Same", template=6)
        b = self.plates.create("Same", template=6)
        self.assertNotEqual(a.meta.id, b.meta.id)
        self.assertEqual(2, len(self.plates.list()))

    def test_rename_refuses_empty(self):
        doc = self.plates.create("keep", template=6)
        self.assertFalse(self.plates.rename(doc.meta.id, "   "))
        self.assertEqual("keep", self.plates.get(doc.meta.id).meta.name)


# ═══════════════════════════════════════════════════════════════════

class TestDuplicateIsStandalone(_StoreCase):
    def test_editing_a_copy_never_touches_the_original(self):
        src = self.plates.create("original", template=24)
        src.override(src.patterns()[0], "g0_0").rim_height_mm = 5.0
        self.plates.save(src)

        copy = self.plates.duplicate(src.meta.id, "the copy")
        copy.patterns()[0].pitch_x_mm = 25.0
        copy.override(copy.patterns()[0], "g0_0").rim_height_mm = 9.0
        self.plates.save(copy)

        self.plates.invalidate()
        original = self.plates.get(src.meta.id)
        self.assertEqual(19.3, original.patterns()[0].pitch_x_mm)
        self.assertEqual(
            5.0, original.patterns()[0].overrides["g0_0"].rim_height_mm)

    def test_a_copy_inherits_no_store_history(self):
        """A fork must not claim the original's legacy keys, or two plates
        would both read the same taught calibration."""
        src = self.plates.create("original", template=6)
        self.plates.adopt_legacy_key(src.meta.id, "plate-6")
        copy = self.plates.duplicate(src.meta.id)
        self.assertEqual([], copy.meta.legacy_keys)

    def test_default_copy_name(self):
        src = self.plates.create("Plate A", template=6)
        self.assertEqual("Plate A copy",
                         self.plates.duplicate(src.meta.id).meta.name)


# ═══════════════════════════════════════════════════════════════════

class TestStoreKeyResolution(_StoreCase):
    def test_key_precedence(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        self.assertEqual(24, cfg.active_plate_key)
        cfg.plate_name = "legacy-plate"
        self.assertEqual("legacy-plate", cfg.active_plate_key)
        cfg.plate_doc_id = "plt_abc123"
        self.assertEqual("plt_abc123", cfg.active_plate_key)
        cfg.plate_type_id = "nest-plastic-24"
        self.assertEqual("nest-plastic-24", cfg.active_plate_key)

    def test_plate_doc_id_round_trips(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.plate_doc_id = "plt_deadbeef0001"
        back = HardwareConfig.from_dict(cfg.to_dict())
        self.assertEqual("plt_deadbeef0001", back.plate_doc_id)

    def test_legacy_keys_are_offered_after_the_current_one(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        import SupportClasses.PlateDocumentStore as store_mod

        doc = self.plates.create("re-homed", template=24)
        self.plates.adopt_legacy_key(doc.meta.id, "plate-24_Rosette in A2")

        cfg = HardwareConfig()
        cfg.plate_doc_id = doc.meta.id
        real = store_mod.get_plate_store
        store_mod.get_plate_store = lambda: self.plates
        try:
            keys = plate_store_keys(cfg)
        finally:
            store_mod.get_plate_store = real

        self.assertEqual(doc.meta.id, keys[0])            # always WRITE this
        self.assertIn("plate-24_Rosette in A2", keys)     # ...but READ these too
        self.assertEqual(len(keys), len(set(keys)))

    def test_adopt_legacy_key_is_idempotent_and_refuses_self(self):
        doc = self.plates.create("x", template=6)
        self.assertTrue(self.plates.adopt_legacy_key(doc.meta.id, "old-name"))
        self.assertFalse(self.plates.adopt_legacy_key(doc.meta.id, "old-name"))
        self.assertFalse(self.plates.adopt_legacy_key(doc.meta.id, doc.meta.id))


# ═══════════════════════════════════════════════════════════════════

class TestSafeKeyCollision(unittest.TestCase):
    """Six stores sanitise their key with the SAME lossy, many-to-one map
    before using it as a filename. Two ids colliding there would make one
    plate silently overwrite another's mosaic PNG while their metadata stayed
    distinct."""

    @staticmethod
    def _safe(key: str) -> str:
        return re.sub(r"[^A-Za-z0-9_.-]", "_", str(key))

    def test_ids_are_fixed_points_of_the_sanitiser(self):
        for _ in range(200):
            doc_id = new_doc_id("plate")
            self.assertEqual(doc_id, self._safe(doc_id))

    def test_distinct_ids_stay_distinct_after_sanitising(self):
        ids = [new_doc_id("plate") for _ in range(400)]
        self.assertEqual(len(set(ids)), len({self._safe(i) for i in ids}))

    def test_the_legacy_name_scheme_really_could_collide(self):
        """Documents the hazard the id design removes — these two plate NAMES
        map to one filename."""
        self.assertEqual(self._safe("plate-24_Rosette in A2&A3"),
                         self._safe("plate-24_Rosette_in_A2_A3"))


# ═══════════════════════════════════════════════════════════════════

class TestWellPlateLoadIntegration(_StoreCase):
    def setUp(self):
        super().setUp()
        import SupportClasses.PlateDocumentStore as store_mod
        self._real_plate = store_mod.get_plate_store
        self._real_ros = store_mod.get_rosette_store
        store_mod.get_plate_store = lambda: self.plates
        store_mod.get_rosette_store = lambda: self.rosettes

    def tearDown(self):
        import SupportClasses.PlateDocumentStore as store_mod
        store_mod.get_plate_store = self._real_plate
        store_mod.get_rosette_store = self._real_ros
        super().tearDown()

    def test_load_by_document_id(self):
        doc = self.plates.create("loadable", template=24)
        plate = WellPlate.load(doc.meta.id)
        self.assertEqual(24, len(plate.well_names))
        self.assertEqual((0.0, 0.0), plate.get_well_position("A1"))

    def test_load_by_display_name_falls_back(self):
        """Without this, a config written before v7.12 silently reverts to the
        96-well default instead of loading the operator's plate."""
        self.plates.create("My Special Plate", template=6)
        self.assertEqual(6, len(WellPlate.load("My Special Plate").well_names))

    def test_load_resolves_a_live_rosette_reference(self):
        ros = self.rosettes.create("Ring of 3")
        ros.add_ring(0.0, 0.0, count=3, ring_diameter_mm=6.0,
                     style=WellStyle(diameter_mm=1.5, well_depth_mm=20.0))
        self.rosettes.save(ros)

        doc = self.plates.create("with rosette", template=24)
        doc.place_rosette((doc.patterns()[0].id, "g0_0"), ros.meta.id)
        self.plates.save(doc)

        plate = WellPlate.load(doc.meta.id)
        self.assertNotIn("A1", plate.well_names)
        self.assertIn("A1.a", plate.well_names)

    def test_standard_int_formats_still_win(self):
        for fmt in (6, 24, 96):
            self.assertEqual(fmt, WellPlate.load(fmt).format)
        self.assertEqual(24, WellPlate.load("24").format)

    def test_unknown_key_still_raises(self):
        with self.assertRaises(FileNotFoundError):
            WellPlate.load("no-such-plate-anywhere")


class TestValidateFollowsActivePlateKey(unittest.TestCase):
    """🔴 Operator: *"the software doesn't let me continue to calibration
    because the hardware says its not fully defined. It should be defined."*

    ``HardwareConfig.validate`` checks that the plate identity resolves, so it
    has to walk the SAME precedence as ``active_plate_key``. v7.12 inserted
    ``plate_doc_id`` into the property and not into the check, so a v2 design
    fell through to the legacy ``plate_name`` branch and was looked up as a v1
    file named after its DISPLAY NAME — a file that by design never exists,
    since the whole identity fix is "the filename is the stable id". Every
    operator using a custom plate was locked out of Calibration by a plate
    that was perfectly valid.
    """

    def setUp(self):
        import os
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("MEBP_PLATES_DIR")
        os.environ["MEBP_PLATES_DIR"] = self._tmp.name
        from SupportClasses.PlateDocumentStore import (
            get_plate_store, reset_stores)
        reset_stores()                     # rebuild against the temp dir
        self.store = get_plate_store()

    def tearDown(self):
        import os
        from SupportClasses.PlateDocumentStore import reset_stores
        if self._prev is None:
            os.environ.pop("MEBP_PLATES_DIR", None)
        else:
            os.environ["MEBP_PLATES_DIR"] = self._prev
        reset_stores()                     # back to the real directory
        self._tmp.cleanup()

    def _config(self):
        """A config complete in every respect EXCEPT the plate, so the only
        issues that can appear are the ones under test."""
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.plate_format = 24
        return cfg

    @staticmethod
    def _plate_issues(cfg) -> list[str]:
        return [i for i in cfg.validate()[1] if "late" in i]

    def test_a_v2_design_validates(self):
        doc = self.store.create("Custom 6 insert with rosette", template=6)
        cfg = self._config()
        cfg.plate_doc_id = doc.meta.id
        cfg.plate_name = doc.meta.name        # the display cache
        self.assertEqual([], self._plate_issues(cfg))

    def test_the_display_name_is_never_looked_up_as_a_file(self):
        """The exact failure: a name with spaces that is not a v1 filename."""
        doc = self.store.create("Custom 6 insert with rosette", template=6)
        cfg = self._config()
        cfg.plate_doc_id = doc.meta.id
        cfg.plate_name = doc.meta.name
        self.assertNotIn("not found in", " ".join(self._plate_issues(cfg)))

    def test_a_missing_document_is_still_reported(self):
        cfg = self._config()
        cfg.plate_doc_id = "plt_doesnotexist"
        cfg.plate_name = "Gone"
        issues = self._plate_issues(cfg)
        self.assertTrue(issues)
        self.assertIn("Gone", issues[0])

    def test_a_standard_and_a_product_still_validate(self):
        cfg = self._config()
        self.assertEqual([], self._plate_issues(cfg))
        cfg.plate_type_id = "nest-plastic-24"
        self.assertEqual([], self._plate_issues(cfg))

    def test_validate_accepts_whatever_active_plate_key_returns(self):
        """The contract that keeps the two from drifting again: if the key
        resolves to a real plate, validate must not object to it."""
        doc = self.store.create("Roundtrip", template=24)
        for setup in ({"plate_format": 24},
                      {"plate_format": 24, "plate_type_id": "nest-plastic-24"},
                      {"plate_format": 24, "plate_doc_id": doc.meta.id,
                       "plate_name": doc.meta.name}):
            cfg = self._config()
            for k, v in setup.items():
                setattr(cfg, k, v)
            key = cfg.active_plate_key
            self.assertIsNotNone(WellPlate.load(key), f"{setup} -> {key}")
            self.assertEqual([], self._plate_issues(cfg), str(setup))


class TestPlateZOffsetHome(unittest.TestCase):
    """Operator: *"the needle offsets wont save to the plate because it has no
    type."*

    The v7.5.x learn loop could only write ``z_offsets`` to a `PlateType`, and
    three sites reached for ``plate_type_id`` independently — the writer, the
    controller's adopter, and the calibration page's auto-fill. A v7.12 design
    is a plate identity in its own right (it is what ``active_plate_key``
    returns) but had nowhere to put them, so the operator was told to "select a
    specific plate TYPE" for a plate that already was specific.
    """

    def setUp(self):
        import os
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("MEBP_PLATES_DIR")
        os.environ["MEBP_PLATES_DIR"] = self._tmp.name
        from SupportClasses.PlateDocumentStore import (
            get_plate_store, reset_stores)
        reset_stores()
        self.store = get_plate_store()

    def tearDown(self):
        import os
        from SupportClasses.PlateDocumentStore import reset_stores
        if self._prev is None:
            os.environ.pop("MEBP_PLATES_DIR", None)
        else:
            os.environ["MEBP_PLATES_DIR"] = self._prev
        reset_stores()
        self._tmp.cleanup()

    @staticmethod
    def _cfg(**kw):
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.plate_format = kw.pop("fmt", 24)
        for k, v in kw.items():
            setattr(cfg, k, v)
        return cfg

    def test_a_design_is_a_home_for_offsets(self):
        doc = self.store.create("Mine", template=6)
        cfg = self._cfg(plate_doc_id=doc.meta.id, fmt=6)
        self.assertEqual(("document", doc.meta.id), cfg.plate_z_offset_home())

    def test_a_product_is_still_a_home(self):
        cfg = self._cfg(plate_type_id="nest-plastic-24")
        self.assertEqual(("type", "nest-plastic-24"),
                         cfg.plate_z_offset_home())

    def test_a_bare_standard_has_no_home(self):
        """The one case where refusing is right — and the message now says
        what to do about it."""
        self.assertEqual(("", ""), self._cfg().plate_z_offset_home())

    def test_offsets_round_trip_through_the_document(self):
        doc = self.store.create("Mine", template=6)
        doc.meta.z_offsets = {"top": 12.5, "bottom": 23.17}
        self.store.save(doc)
        cfg = self._cfg(plate_doc_id=doc.meta.id, fmt=6)
        self.assertEqual({"top": 12.5, "bottom": 23.17}, cfg.plate_z_offsets())

    def test_offsets_survive_a_reload_from_disk(self):
        from SupportClasses.PlateDocumentStore import reset_stores
        doc = self.store.create("Mine", template=6)
        doc.meta.z_offsets = {"safe": 4.25}
        self.store.save(doc)
        cfg = self._cfg(plate_doc_id=doc.meta.id, fmt=6)
        reset_stores()
        self.assertEqual({"safe": 4.25}, cfg.plate_z_offsets())

    def test_a_fork_inherits_its_product_per_key(self):
        """Teaching one reference must not discard the product's guesses for
        the others."""
        doc = self.store.create("Fork", template=24)
        doc.meta.plate_type_id = "nest-plastic-24"
        self.store.save(doc)
        cfg = self._cfg(plate_doc_id=doc.meta.id,
                        plate_type_id="nest-plastic-24")
        inherited = cfg.plate_z_offsets()
        self.assertTrue(inherited)

        doc.meta.z_offsets = {"bottom": 99.0}
        self.store.save(doc)
        merged = cfg.plate_z_offsets()
        self.assertEqual(99.0, merged["bottom"])
        for k, v in inherited.items():
            if k != "bottom":
                self.assertEqual(v, merged[k], f"lost inherited '{k}'")

    def test_no_offsets_anywhere_is_an_empty_dict_not_a_raise(self):
        self.assertEqual({}, self._cfg().plate_z_offsets())
        self.assertEqual({}, self._cfg(plate_doc_id="plt_gone").plate_z_offsets())

    def test_a_document_with_no_offsets_serializes_unchanged(self):
        """Conditional emit — an existing plate file must not gain a key."""
        doc = self.store.create("Plain", template=6)
        self.assertNotIn("z_offsets", doc.to_dict()["meta"])

    def test_offsets_round_trip_through_to_dict(self):
        doc = self.store.create("RT", template=6)
        doc.meta.z_offsets = {"top": 1.5, "bottom": 2.25}
        again = PlateDocument.from_dict(doc.to_dict())
        self.assertEqual({"top": 1.5, "bottom": 2.25}, again.meta.z_offsets)

    def test_the_controller_adopts_a_designs_offsets(self):
        """The read side: `StageController` used to read `plate_type_id`
        directly, so a design's offsets could be saved and never applied."""
        from SupportClasses.StageController import StageController
        doc = self.store.create("Adopted", template=6)
        doc.meta.z_offsets = {"top": 3.0, "bottom": 21.0,
                              "safe": -1.0, "max": -2.0}
        self.store.save(doc)
        cfg = self._cfg(plate_doc_id=doc.meta.id, fmt=6)

        ctrl = StageController.__new__(StageController)
        ctrl._plate_z_offsets = {}
        pushed = {}
        ctrl.set_plate_z_offsets = lambda **kw: pushed.update(kw)
        StageController._apply_active_plate_type_offsets(ctrl, cfg)
        self.assertEqual({"top": 3.0, "bottom": 21.0,
                          "safe": -1.0, "max": -2.0}, pushed)


if __name__ == "__main__":
    unittest.main()
