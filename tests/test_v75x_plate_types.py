"""
v7.5.x — Selectable plate TYPES: per-type Z offsets + auto-loading mosaics.

A plate type (Corning glass-bottom, NEST plastic, …) is a thin overlay on a
base standard format:
  * geometry  — inherited from the base format (WellPlate.format stays the int);
  * identity  — the type id becomes HardwareConfig.active_plate_key, so the
    per-plate mosaic / template / training stores auto-segregate;
  * Z offsets — {top,bottom,safe,max} mm below the needle-cam fiducial, the
    guess source the calibration "Estimate plate Z" inherits.

Covers:
  * PlateTypeStore   — built-in load, list_for_format, user override shadow,
                       save_user round-trip + persistence, generic sentinel.
  * HardwareConfig   — active_plate_key precedence, serialize, validate.
  * WellPlate.load   — type id → base-format geometry (format stays int);
                       well_depth override; unknown id is safe.
  * StageController  — estimate_plate_z_refs returns plate_max_z;
                       _apply_active_plate_type_offsets adopts the type offsets.
  * CalibrationPage  — _ploc_plate_key keys by identity; auto-fill respects
                       taught values (no-clobber); learn-loop write-back.
  * HardwareSetupPage — two-step Format→Type card drives plate_type_id.
"""

import os
import sys
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import SupportClasses.PlateTypeStore as PTS  # noqa: E402
from SupportClasses.PlateTypeStore import PlateType, PlateTypeStore  # noqa: E402
from SupportClasses.WellPlate import WellPlate  # noqa: E402
from SupportClasses.HardwareConfig import HardwareConfig  # noqa: E402
from SupportClasses.StageController import StageController  # noqa: E402


def _app():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication(sys.argv)


def _write_type(directory: Path, **kw) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    data = {
        "id": kw["id"], "base_format": kw.get("base_format", 24),
        "display_name": kw.get("display_name", kw["id"]),
        "manufacturer": kw.get("manufacturer", ""),
        "model": kw.get("model", ""),
        "bottom_material": kw.get("bottom_material", ""),
        "well_depth_mm": kw.get("well_depth_mm"),
        "z_offsets": kw.get("z_offsets",
                            {"top": 1.0, "bottom": 2.0, "safe": 3.0, "max": 4.0}),
        "builtin": kw.get("builtin", True),
    }
    with open(directory / f"{kw['id']}.json", "w") as f:
        json.dump(data, f)


class _TempStoreMixin:
    """Create a temp builtin+user PlateTypeStore and install it as the
    process-wide singleton (so get_store() callers use it); restore on teardown."""

    def _install_temp_store(self, builtin_types=(), user_types=()):
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self._builtin_dir = root / "builtin"
        self._user_dir = root / "user"
        for t in builtin_types:
            _write_type(self._builtin_dir, builtin=True, **t)
        for t in user_types:
            _write_type(self._user_dir, builtin=False, **t)
        store = PlateTypeStore(self._builtin_dir, self._user_dir)
        self._prev_singleton = PTS._store_singleton
        PTS._store_singleton = store
        return store

    def _restore_store(self):
        PTS._store_singleton = getattr(self, "_prev_singleton", None)
        if hasattr(self, "_tmp"):
            self._tmp.cleanup()


# ── PlateTypeStore ─────────────────────────────────────────────────

class TestPlateTypeStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.builtin = root / "builtin"
        self.user = root / "user"
        _write_type(self.builtin, id="corning-24", base_format=24,
                    bottom_material="glass",
                    z_offsets={"top": 10, "bottom": 28, "safe": 5, "max": 0})
        _write_type(self.builtin, id="nest-24", base_format=24,
                    bottom_material="plastic")
        _write_type(self.builtin, id="corning-96", base_format=96)
        self.store = PlateTypeStore(self.builtin, self.user)

    def tearDown(self):
        self._tmp.cleanup()

    def test_builtin_load(self):
        ids = {p.id for p in self.store.all()}
        self.assertEqual(ids, {"corning-24", "nest-24", "corning-96"})
        self.assertTrue(self.store.get("corning-24").builtin)

    def test_list_for_format_filters(self):
        self.assertEqual({p.id for p in self.store.list_for_format(24)},
                         {"corning-24", "nest-24"})
        self.assertEqual({p.id for p in self.store.list_for_format(96)},
                         {"corning-96"})
        self.assertEqual(self.store.list_for_format(6), [])

    def test_user_override_shadows_builtin_by_id(self):
        # A user file with the same id wins; the built-in file is untouched.
        _write_type(self.user, id="corning-24", base_format=24,
                    bottom_material="glass", builtin=False,
                    z_offsets={"top": 11, "bottom": 29, "safe": 6, "max": 1})
        store2 = PlateTypeStore(self.builtin, self.user)
        pt = store2.get("corning-24")
        self.assertFalse(pt.builtin)
        self.assertEqual(pt.z_offsets["bottom"], 29.0)
        # built-in file on disk unchanged
        with open(self.builtin / "corning-24.json") as f:
            self.assertEqual(json.load(f)["z_offsets"]["bottom"], 28)

    def test_save_user_round_trip_and_persist(self):
        pt = self.store.get("nest-24")
        pt.z_offsets = {"top": 9, "bottom": 25, "safe": 4, "max": 2}
        self.assertTrue(self.store.save_user(pt))
        # live cache updated + a user file written
        self.assertEqual(self.store.get("nest-24").z_offsets["bottom"], 25.0)
        self.assertTrue((self.user / "nest-24.json").exists())
        # persists across instances; built-in left pristine
        store2 = PlateTypeStore(self.builtin, self.user)
        self.assertEqual(store2.get("nest-24").z_offsets["bottom"], 25.0)
        self.assertFalse(store2.get("nest-24").builtin)

    def test_is_generic(self):
        self.assertTrue(PlateTypeStore.is_generic(""))
        self.assertTrue(PlateTypeStore.is_generic(None))
        self.assertTrue(PlateTypeStore.is_generic("generic-24"))
        self.assertFalse(PlateTypeStore.is_generic("corning-24"))

    def test_normalises_partial_offsets(self):
        _write_type(self.builtin, id="partial-24", base_format=24,
                    z_offsets={"bottom": 30})   # missing top/safe/max
        store2 = PlateTypeStore(self.builtin, self.user)
        off = store2.get("partial-24").z_offsets
        self.assertEqual(set(off), {"top", "bottom", "safe", "max"})
        self.assertEqual(off["bottom"], 30.0)
        self.assertEqual(off["top"], 0.0)


# ── HardwareConfig precedence / serialize / validate ───────────────

class TestHardwareConfigPlateType(_TempStoreMixin, unittest.TestCase):
    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "corning-24", "base_format": 24, "bottom_material": "glass"},
        ])

    def tearDown(self):
        self._restore_store()

    def test_active_plate_key_precedence(self):
        c = HardwareConfig()
        c.plate_format = 24
        self.assertEqual(c.active_plate_key, 24)
        c.plate_name = "my-custom"
        self.assertEqual(c.active_plate_key, "my-custom")
        c.plate_type_id = "corning-24"
        self.assertEqual(c.active_plate_key, "corning-24")   # type wins

    def test_serialize_round_trip(self):
        c = HardwareConfig()
        c.plate_type_id = "corning-24"
        c.plate_format = 24
        c2 = HardwareConfig.from_dict(c.to_dict())
        self.assertEqual(c2.plate_type_id, "corning-24")
        self.assertEqual(c2.active_plate_key, "corning-24")

    def test_validate_flags_dangling_type_id(self):
        c = HardwareConfig()
        c.plate_type_id = "ghost-plate"
        _, issues = c.validate()
        self.assertTrue(any("Plate type" in i for i in issues))
        c.plate_type_id = "corning-24"
        _, issues = c.validate()
        self.assertFalse(any("Plate type" in i for i in issues))


# ── Compose: plate TYPE (identity) + custom design (geometry) ──────

class TestGeometryPlateKeyCompose(_TempStoreMixin, unittest.TestCase):
    """A plate TYPE carries identity/Z-offsets but resolves to a PLAIN base
    format; a custom design (e.g. a rosette saved under plate_name) layered
    under it must still supply GEOMETRY. `geometry_plate_key` composes them
    while `active_plate_key` keeps the type's identity."""

    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "nest-24", "base_format": 24, "bottom_material": "plastic"},
        ])
        self._plates_root = tempfile.mkdtemp()
        self._tmp_plates = Path(self._plates_root) / "plates"
        self._tmp_plates.mkdir(parents=True, exist_ok=True)
        # Redirect BOTH module refs (WellPlate.load reads WellPlate's, and it
        # delegates to PlateDesign.load which reads PlateDesign's).
        self._patchers = [
            patch("SupportClasses.WellPlate.USER_PLATES_DIR", self._tmp_plates),
            patch("SupportClasses.PlateDesign.USER_PLATES_DIR", self._tmp_plates),
        ]
        for p in self._patchers:
            p.start()
        # Build + save a rosette design (3 sub-wells in A1) to the temp dir.
        from SupportClasses.PlateDesign import PlateDesign
        d = PlateDesign.from_standard_format(24)
        a1 = d.get_wells()[0]
        ros = PlateDesign.blank_rosette(bore_radius_mm=7.0)
        ros.add_well(x=3.0, y=0.0, diameter=1.0, name="a", naming_scheme="MANUAL")
        ros.add_well(x=0.0, y=3.0, diameter=1.0, name="b", naming_scheme="MANUAL")
        ros.add_well(x=0.0, y=0.0, diameter=1.0, name="c", naming_scheme="MANUAL")
        a1.rosette_design = ros
        d.name = "rosette-plate"
        d.save()

    def tearDown(self):
        for p in self._patchers:
            p.stop()
        self._restore_store()

    def _cfg_both(self):
        c = HardwareConfig()
        c.plate_format = 24
        c.plate_name = "rosette-plate"
        c.plate_type_id = "nest-24"
        return c

    def test_identity_stays_the_type_geometry_is_the_design(self):
        c = self._cfg_both()
        self.assertEqual(c.active_plate_key, "nest-24")       # identity
        self.assertEqual(c.geometry_plate_key, "rosette-plate")  # geometry

    def test_geometry_key_loads_rosette_subwells(self):
        c = self._cfg_both()
        plate = WellPlate.load(c.geometry_plate_key)
        names = plate.well_names
        self.assertNotIn("A1", names)         # parent flattened away
        self.assertIn("A1.a", names)
        self.assertIn("A1.b", names)
        self.assertIn("A1.c", names)
        # The type-id path (identity) still yields the PLAIN 24-well plate.
        plain = WellPlate.load(c.active_plate_key)
        self.assertIn("A1", plain.well_names)
        self.assertEqual(len(plain.well_names), 24)

    def test_no_custom_design_falls_back_to_type(self):
        c = HardwareConfig()
        c.plate_format = 24
        c.plate_type_id = "nest-24"           # type only, no plate_name
        self.assertEqual(c.geometry_plate_key, "nest-24")

    def test_stale_plate_name_falls_back_to_type(self):
        c = self._cfg_both()
        c.plate_name = "does-not-exist"       # file missing
        self.assertEqual(c.geometry_plate_key, "nest-24")

    def test_no_type_geometry_equals_active_key(self):
        # Plain custom plate (no type) — geometry == active_plate_key.
        c = HardwareConfig()
        c.plate_format = 24
        c.plate_name = "rosette-plate"
        self.assertEqual(c.geometry_plate_key, c.active_plate_key)
        self.assertEqual(c.geometry_plate_key, "rosette-plate")


# ── WellPlate.load type resolution ─────────────────────────────────

class TestWellPlateLoadType(_TempStoreMixin, unittest.TestCase):
    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "corning-24", "base_format": 24, "bottom_material": "glass"},
            {"id": "deep-24", "base_format": 24, "well_depth_mm": 22.5},
        ])

    def tearDown(self):
        self._restore_store()

    def test_type_id_resolves_to_base_format_int(self):
        wp = WellPlate.load("corning-24")
        self.assertEqual(wp.format, 24)            # stays the base INT
        self.assertFalse(wp.is_custom)
        self.assertEqual(len(wp.well_names), 24)
        # identical geometry to the bare standard
        self.assertEqual(wp.well_names, WellPlate.from_format(24).well_names)

    def test_well_depth_override(self):
        wp = WellPlate.load("deep-24")
        self.assertEqual(wp.format, 24)
        self.assertAlmostEqual(wp.well_depth_mm, 22.5)

    def test_unknown_id_falls_through_safely(self):
        with self.assertRaises(FileNotFoundError):
            WellPlate.load("no-such-type-xyz")

    def test_two_types_same_format_identical_geometry(self):
        a = WellPlate.load("corning-24")
        b = WellPlate.load("deep-24")
        self.assertEqual(a.well_names, b.well_names)


# ── StageController Z "max" + offset adoption ──────────────────────

class TestControllerZMax(_TempStoreMixin, unittest.TestCase):
    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "tt-24", "base_format": 24,
             "z_offsets": {"top": 1.0, "bottom": 2.0, "safe": 3.0, "max": 4.0}},
        ])

    def tearDown(self):
        self._restore_store()

    def _stub(self):
        sc = StageController.__new__(StageController)
        sc._needle_cam_z_user = 50.0
        sc._plate_z_offsets = {"top": 10.0, "bottom": 20.0, "safe": 5.0, "max": 0.0}
        # ME3B V1 polarity: user_z grows as raw shrinks; here just identity.
        sc.user_z_to_zref = lambda u: u
        return sc

    def test_estimate_returns_plate_max_z(self):
        sc = self._stub()
        refs = sc.estimate_plate_z_refs()
        self.assertIn("plate_max_z", refs)
        self.assertAlmostEqual(refs["plate_max_z"], 50.0 - 0.0)
        sc.set_plate_z_offsets(max=8.0)
        self.assertAlmostEqual(sc.estimate_plate_z_refs()["plate_max_z"], 42.0)

    def test_apply_active_plate_type_offsets_adopts(self):
        sc = self._stub()
        cfg = HardwareConfig()
        cfg.plate_type_id = "tt-24"
        sc._apply_active_plate_type_offsets(cfg)
        self.assertEqual(sc._plate_z_offsets,
                         {"top": 1.0, "bottom": 2.0, "safe": 3.0, "max": 4.0})

    def test_apply_offsets_noop_for_generic(self):
        sc = self._stub()
        before = dict(sc._plate_z_offsets)
        cfg = HardwareConfig()   # no plate_type_id
        sc._apply_active_plate_type_offsets(cfg)
        self.assertEqual(sc._plate_z_offsets, before)


# ── CalibrationPage: identity key + auto-fill + learn loop ─────────

class _CalBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _make_page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.get_zp_position.return_value = (None, None, None)
        ctrl.default_plate_center_um.return_value = (50000.0, 40000.0)
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        # Identity zref↔user so the (guess) label format strings work.
        ctrl.zref_to_user_z.side_effect = lambda v: v
        page = CalibrationPage(ctrl, settings=None)
        return page, ctrl


class TestIdentityPlateKey(_CalBase, _TempStoreMixin):
    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "corning-24", "base_format": 24},
            {"id": "nest-24", "base_format": 24},
        ])

    def tearDown(self):
        self._restore_store()

    def test_plate_key_prefers_active_plate_key(self):
        page, _ = self._make_page()
        page._plate = WellPlate.from_format(24)
        cfg = HardwareConfig()
        cfg.plate_type_id = "corning-24"
        page._hardware_config = cfg
        self.assertEqual(page._ploc_plate_key(), "corning-24")
        cfg.plate_type_id = "nest-24"
        self.assertEqual(page._ploc_plate_key(), "nest-24")

    def test_plate_key_falls_back_to_format_without_hw(self):
        page, _ = self._make_page()
        page._plate = WellPlate.from_format(24)
        page._hardware_config = None
        self.assertEqual(page._ploc_plate_key(), "24")


class TestRosetteVisibleUnderPlateType(_CalBase, _TempStoreMixin):
    """Regression: a rosette (custom design) layered under a plate TYPE must be
    seen by the calibration page — set_hardware_config loads GEOMETRY from the
    custom design while identity/segregation stays the type id. Reproduces the
    reported 'Rosettes tab does not see the rosette' bug."""

    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "nest-24", "base_format": 24, "bottom_material": "plastic"},
        ])
        self._plates_root = tempfile.mkdtemp()
        self._tmp_plates = Path(self._plates_root) / "plates"
        self._tmp_plates.mkdir(parents=True, exist_ok=True)
        self._patchers = [
            patch("SupportClasses.WellPlate.USER_PLATES_DIR", self._tmp_plates),
            patch("SupportClasses.PlateDesign.USER_PLATES_DIR", self._tmp_plates),
        ]
        for p in self._patchers:
            p.start()
        from SupportClasses.PlateDesign import PlateDesign
        d = PlateDesign.from_standard_format(24)
        a1 = d.get_wells()[0]
        ros = PlateDesign.blank_rosette(bore_radius_mm=7.0)
        ros.add_well(x=3.0, y=0.0, diameter=1.0, name="a", naming_scheme="MANUAL")
        ros.add_well(x=0.0, y=3.0, diameter=1.0, name="b", naming_scheme="MANUAL")
        a1.rosette_design = ros
        d.name = "rosette-plate"
        d.save()

    def tearDown(self):
        for p in self._patchers:
            p.stop()
        self._restore_store()

    def test_set_hardware_config_loads_rosette_geometry(self):
        page, _ = self._make_page()
        cfg = HardwareConfig()
        cfg.plate_format = 24
        cfg.plate_name = "rosette-plate"
        cfg.plate_type_id = "nest-24"       # type layered over the rosette
        page.set_hardware_config(cfg)
        names = [w.name for w in page._plate.get_all_wells()]
        self.assertIn("A1.a", names)
        self.assertIn("A1.b", names)
        self.assertNotIn("A1", names)       # parent flattened
        # The Rosettes tab picker sources parents from this plate.
        self.assertEqual(page._ploc_rosette_parent_wells(), ["A1"])
        # Identity/segregation still keys on the plate type.
        self.assertEqual(page._ploc_plate_key(), "nest-24")


class TestAutoFillNoClobber(_CalBase):
    def test_taught_value_survives_estimate(self):
        page, ctrl = self._make_page()
        ctrl.estimate_plate_z_refs.return_value = {
            "plate_top_z": 1.0, "plate_bottom_z": 2.0,
            "safe_z": 3.0, "plate_max_z": 4.0}
        page._top_z = 99.0           # already taught
        page._plate_bottom_z = None  # un-taught
        page._safe_z = None
        page._max_z = None
        page._apply_plate_type_z_estimates(force=False)
        self.assertEqual(page._top_z, 99.0)          # taught wins
        self.assertEqual(page._plate_bottom_z, 2.0)  # filled from guess
        self.assertEqual(page._safe_z, 3.0)
        self.assertEqual(page._max_z, 4.0)

    def test_force_overrides(self):
        page, ctrl = self._make_page()
        ctrl.estimate_plate_z_refs.return_value = {
            "plate_top_z": 1.0, "plate_bottom_z": 2.0,
            "safe_z": 3.0, "plate_max_z": 4.0}
        page._top_z = 99.0
        page._apply_plate_type_z_estimates(force=True)
        self.assertEqual(page._top_z, 1.0)

    def test_no_fiducial_is_noop(self):
        page, ctrl = self._make_page()
        ctrl.estimate_plate_z_refs.return_value = None
        page._top_z = None
        page._apply_plate_type_z_estimates(force=False)
        self.assertIsNone(page._top_z)


class TestLearnLoop(_CalBase, _TempStoreMixin):
    def setUp(self):
        self.store = self._install_temp_store(builtin_types=[
            {"id": "corning-24", "base_format": 24, "bottom_material": "glass",
             "z_offsets": {"top": 0, "bottom": 0, "safe": 0, "max": 0}},
        ])

    def tearDown(self):
        self._restore_store()

    def test_saves_measured_offsets_to_user_override(self):
        page, ctrl = self._make_page()
        ctrl.get_needle_cam_z_user.return_value = 50.0
        ctrl.zref_to_user_z.side_effect = lambda v: v
        cfg = HardwareConfig()
        cfg.plate_type_id = "corning-24"
        page._hardware_config = cfg
        # Taught zero-ref references (user_z == zref via the identity lambda).
        page._top_z = 23.0
        page._plate_bottom_z = 10.0
        page._safe_z = 45.0
        page._max_z = 49.0
        with patch("gui.pages.calibration.QMessageBox"):
            page._zoff_save_offsets_to_plate_type()
        pt = self.store.get("corning-24")
        # offset = fiducial(50) − user_z(ref)
        self.assertAlmostEqual(pt.z_offsets["top"], 27.0)
        self.assertAlmostEqual(pt.z_offsets["bottom"], 40.0)
        self.assertAlmostEqual(pt.z_offsets["safe"], 5.0)
        self.assertAlmostEqual(pt.z_offsets["max"], 1.0)
        self.assertFalse(pt.builtin)                       # user override
        self.assertTrue((self._user_dir / "corning-24.json").exists())
        # built-in file left pristine (all zeros)
        with open(self._builtin_dir / "corning-24.json") as f:
            self.assertEqual(json.load(f)["z_offsets"]["bottom"], 0)
        # live guess source refreshed
        ctrl.set_plate_z_offsets.assert_called()

    def test_abort_without_fiducial(self):
        page, ctrl = self._make_page()
        ctrl.get_needle_cam_z_user.return_value = None
        cfg = HardwareConfig()
        cfg.plate_type_id = "corning-24"
        page._hardware_config = cfg
        page._top_z = 23.0
        with patch("gui.pages.calibration.QMessageBox"):
            page._zoff_save_offsets_to_plate_type()
        # nothing written → built-in offsets still zero, no user file
        self.assertEqual(self.store.get("corning-24").z_offsets["top"], 0.0)
        self.assertFalse((self._user_dir / "corning-24.json").exists())

    def test_abort_when_generic(self):
        page, ctrl = self._make_page()
        ctrl.get_needle_cam_z_user.return_value = 50.0
        cfg = HardwareConfig()           # generic — no plate_type_id
        page._hardware_config = cfg
        page._top_z = 23.0
        with patch("gui.pages.calibration.QMessageBox"):
            page._zoff_save_offsets_to_plate_type()
        self.assertFalse((self._user_dir / "corning-24.json").exists())


# ── HardwareSetupPage two-step card ────────────────────────────────

class TestHardwareSetupCard(unittest.TestCase, _TempStoreMixin):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def setUp(self):
        self._install_temp_store(builtin_types=[
            {"id": "corning-24", "base_format": 24, "bottom_material": "glass"},
            {"id": "nest-24", "base_format": 24, "bottom_material": "plastic"},
            {"id": "corning-96", "base_format": 96, "bottom_material": "glass"},
        ])

    def tearDown(self):
        self._restore_store()

    def test_card_builds_and_filters_by_format(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        self.assertTrue(hasattr(page, "_plate_type_format_combo"))
        self.assertTrue(hasattr(page, "_plate_type_combo"))
        # default format 24 → Generic + 2 products
        ids = [page._plate_type_combo.itemData(i)
               for i in range(page._plate_type_combo.count())]
        self.assertEqual(ids, ["", "corning-24", "nest-24"])

    def test_selecting_type_sets_plate_type_id(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        ci = page._plate_type_combo.findData("corning-24")
        page._plate_type_combo.setCurrentIndex(ci)
        self.assertEqual(page._selected_plate_type_id, "corning-24")
        self.assertEqual(page._config.plate_type_id, "corning-24")
        self.assertEqual(page._config.active_plate_key, "corning-24")

    def test_format_change_resets_to_generic_and_refilters(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        # select a 24 product, then switch format to 96
        page._plate_type_combo.setCurrentIndex(
            page._plate_type_combo.findData("nest-24"))
        page._plate_type_format_combo.setCurrentIndex(
            page._plate_type_format_combo.findData(96))
        self.assertEqual(page._selected_plate_type_id, "")       # reset
        ids = [page._plate_type_combo.itemData(i)
               for i in range(page._plate_type_combo.count())]
        self.assertEqual(ids, ["", "corning-96"])
        self.assertEqual(page._config.plate_type_id, "")
        self.assertEqual(page._config.active_plate_key, 96)

    def test_set_hardware_config_syncs_card(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        cfg = HardwareConfig()
        cfg.plate_format = 24
        cfg.plate_type_id = "corning-24"
        page.set_hardware_config(cfg)
        self.assertEqual(page._selected_plate_type_id, "corning-24")
        self.assertEqual(page._plate_type_combo.currentData(), "corning-24")
        self.assertEqual(page._plate_type_format_combo.currentData(), 24)


# ── Per-plate-key calibration archive ──────────────────────────────

class _CalSettings:
    """Minimal dict-backed Settings stub (get/set_section + get + save)."""
    def __init__(self):
        self._sections = {}

    def get_section(self, name):
        return self._sections.get(name)

    def set_section(self, name, value):
        self._sections[name] = value

    def get(self, key, default=None):
        return default

    def save(self):
        pass


class TestPerPlateCalibration(unittest.TestCase):
    """The taught calibration is archived per active_plate_key so it follows
    the plate; the flat ``calibration`` section keeps mirroring the active
    plate (legacy readers)."""

    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def setUp(self):
        # Isolate the durable snapshot store to a tempdir so _save_calibration
        # doesn't touch the real last_calibration.json.
        import SupportClasses.CalibrationSnapshotStore as snapmod
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self._prev_snap = getattr(snapmod, "_store", None)
        snapmod._store = snapmod.CalibrationSnapshotStore(
            Path(self._tmp.name) / "lc.json")
        self.addCleanup(lambda: setattr(snapmod, "_store", self._prev_snap))

    def _ctrl(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (None, None)
        ctrl.get_zp_position.return_value = (None, None, None)
        ctrl.default_plate_center_um.return_value = (50000.0, 40000.0)
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        ctrl.zref_to_user_z.side_effect = lambda v: v
        ctrl.plate_flip_180.return_value = True
        ctrl.plate_axis_sign.return_value = (-1.0, -1.0)
        return ctrl

    def _page(self, settings):
        from gui.pages.calibration import CalibrationPage
        page = CalibrationPage(self._ctrl(), settings=settings)
        page._plate = WellPlate.from_format(24)
        return page

    def test_save_writes_flat_mirror_and_keyed_archive(self):
        settings = _CalSettings()
        page = self._page(settings)
        page._loaded_cal_key = "corning-glass-24"
        cfg = HardwareConfig()
        cfg.plate_type_id = "corning-glass-24"
        page._hardware_config = cfg
        page._taught_a1 = (1000.0, 2000.0)
        page._safe_z = 5.0
        page._save_calibration()
        # Flat section mirrors the active plate (legacy readers keep working).
        flat = settings.get_section("calibration")
        self.assertEqual(flat["taught_a1"], [1000.0, 2000.0])
        # Archived under the active plate identity.
        arch = settings.get_section("calibration_by_plate")
        self.assertIn("corning-glass-24", arch)
        self.assertEqual(arch["corning-glass-24"]["taught_a1"], [1000.0, 2000.0])

    def test_switch_flushes_outgoing_and_restores_incoming(self):
        settings = _CalSettings()
        page = self._page(settings)
        # Teach + archive plate A.
        page._loaded_cal_key = "corning-glass-24"
        page._taught_a1 = (1000.0, 2000.0)
        page._save_calibration(key="corning-glass-24")

        # Switch to plate B (no stored cal) → blank.
        page._activate_calibration_for_key("nest-plastic-24")
        self.assertIsNone(page._taught_a1)
        self.assertEqual(page._loaded_cal_key, "nest-plastic-24")

        # Teach B + archive it.
        page._taught_a1 = (7000.0, 8000.0)
        page._save_calibration(key="nest-plastic-24")

        # Switch back to A → A's taught A1 comes back (not B's).
        page._activate_calibration_for_key("corning-glass-24")
        self.assertEqual(page._loaded_cal_key, "corning-glass-24")
        self.assertEqual(tuple(page._taught_a1), (1000.0, 2000.0))

        # And forward to B again → B's value.
        page._activate_calibration_for_key("nest-plastic-24")
        self.assertEqual(tuple(page._taught_a1), (7000.0, 8000.0))

    def test_set_hardware_config_switch_restores_per_plate(self):
        settings = _CalSettings()
        page = self._page(settings)
        # Activate generic 24 and teach it.
        a = HardwareConfig(); a.plate_format = 24
        page.set_hardware_config(a)
        page._taught_a1 = (1111.0, 2222.0)
        page._save_calibration()
        # Switch to a 24-well TYPE (same base format) — no cal yet → blank.
        b = HardwareConfig(); b.plate_format = 24; b.plate_type_id = "corning-glass-24"
        page.set_hardware_config(b)
        self.assertEqual(page._loaded_cal_key, "corning-glass-24")
        self.assertIsNone(page._taught_a1)
        # Back to generic 24 → its taught A1 restored.
        a2 = HardwareConfig(); a2.plate_format = 24
        page.set_hardware_config(a2)
        self.assertEqual(page._loaded_cal_key, "24")
        self.assertEqual(tuple(page._taught_a1), (1111.0, 2222.0))

    def test_migrates_legacy_flat_block_into_archive(self):
        settings = _CalSettings()
        # Pre-existing legacy FLAT calibration (no by_plate archive).
        settings.set_section("calibration", {
            "plate_format": 24, "taught_a1": [500.0, 600.0], "safe_z": 4.0})
        page = self._page(settings)
        arch = page._calibration_archive()
        self.assertIn("24", arch)
        self.assertEqual(arch["24"]["taught_a1"], [500.0, 600.0])
        # And it's persisted to the archive section.
        self.assertIn("24", settings.get_section("calibration_by_plate"))


if __name__ == "__main__":
    unittest.main()
