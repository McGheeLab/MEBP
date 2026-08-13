"""test_v718_filter_cube_catalogue.py — selectable filter cubes with real bands.

Operator: *"lets make the filter cubes standard as drop downs that auto fill in
everything. the excitation and emission are actually ranges, but we are makeing
a single number. Lets look up the most common filter cubes and add them as a
list of selectable ones, the add a custom option that saves into our list."*

Covered here:

  1. ``FilterCubeStore``: the bundled catalogue, center+FWHM → derived edges,
     refusal of junk, user-shadows-builtin, save/delete, and the deliberately
     exact (never fuzzy) name matcher.
  2. ``MicroscopeConfigStore``: the additive range/provenance fields, and — the
     load-bearing one — that a pre-v7.18 entry round-trips BYTE-IDENTICALLY, so
     the ``lablink.imagejob/1`` sidecar and ``OpticsRegistry`` are unaffected.
  3. The setup panel: picking a cube auto-fills everything, a nominal value is
     VISIBLY nominal, editing promotes it off nominal, and "save as a new cube"
     joins the pick-list.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from gui.pages.hardware import microscope_setup_panel as setup_panel_mod

from SupportClasses import FilterCubeStore as fcs_mod
from SupportClasses.FilterCubeStore import (
    FilterCube, FilterCubeStore, PROV_DATASHEET, PROV_MEASURED, PROV_NOMINAL,
    band_edges, clean_nm, clean_provenance, clean_width_nm, format_band,
    safe_id,
)
from SupportClasses.MicroscopeConfigStore import MicroscopeConfigStore

_app = QApplication.instance() or QApplication([])


class _CubeCase(unittest.TestCase):
    """Real bundled catalogue + an isolated user dir (writes never escape)."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.user_dir = Path(self._tmp.name) / "user"
        self.store = FilterCubeStore(
            builtin_dir=fcs_mod._DEFAULT_BUILTIN_DIR, user_dir=self.user_dir)
        # Point the process singleton at it, so the panel sees the same store.
        self._prev = fcs_mod._store_singleton
        fcs_mod._store_singleton = self.store
        self.addCleanup(lambda: setattr(fcs_mod, "_store_singleton", self._prev))


# ── 1. The catalogue ───────────────────────────────────────────────

class TestFilterCubeStore(_CubeCase):
    def test_bundled_catalogue_loads(self):
        cubes = self.store.all()
        self.assertGreaterEqual(len(cubes), 10)
        self.assertTrue(all(c.builtin for c in cubes))
        ids = {c.id for c in cubes}
        for expected in ("dapi", "fitc", "gfp", "txred", "cy5", "mcherry"):
            self.assertIn(expected, ids)

    def test_every_bundled_cube_is_self_consistent(self):
        """A center with no width, or a width with no center, is a half-spec."""
        for cube in self.store.all():
            for prefix in ("excitation", "emission"):
                center = getattr(cube, f"{prefix}_nm")
                width = getattr(cube, f"{prefix}_width_nm")
                if width:
                    self.assertIsNotNone(
                        center, f"{cube.id}: {prefix} width with no center")
                lo, hi = band_edges(center, width)
                if lo is not None:
                    self.assertLess(lo, hi)
                    self.assertGreater(lo, 0)

    def test_center_and_width_derive_the_edges(self):
        """A part marked 470/40 spans 450-490 — the whole point of the model."""
        self.assertEqual(band_edges(470, 40), (450.0, 490.0))
        cube = self.store.get("gfp")
        self.assertEqual(cube.excitation_nm, 470.0)
        self.assertEqual(cube.excitation_width_nm, 40.0)
        self.assertEqual(cube.excitation_range_nm, (450.0, 490.0))

    def test_a_center_with_no_width_has_no_edges(self):
        """Reporting the center twice would invent a zero-width band."""
        self.assertEqual(band_edges(470, 0), (None, None))
        self.assertEqual(band_edges(470, None), (None, None))
        self.assertEqual(format_band(470, 0), "470 nm")
        self.assertEqual(format_band(None, 40), "—")
        self.assertIn("450–490", format_band(470, 40))

    def test_brightfield_carries_no_wavelengths(self):
        """There is no band to report, and a fabricated one would reach the
        LabLink sidecar and pick a PSF."""
        bf = self.store.get("bright-field")
        self.assertIsNotNone(bf)
        self.assertFalse(bf.has_wavelengths)
        entry = bf.optics_entry()
        self.assertNotIn("excitation_nm", entry)
        self.assertNotIn("emission_nm", entry)
        self.assertNotIn("provenance", entry)   # nothing to qualify

    def test_optics_entry_keeps_the_legacy_key_names(self):
        """The CENTER stays under excitation_nm/emission_nm, because that is
        what OpticsRegistry and the LabLink sidecar read."""
        entry = self.store.get("cy5").optics_entry()
        self.assertEqual(entry["excitation_nm"], 620.0)
        self.assertEqual(entry["emission_nm"], 700.0)
        self.assertEqual(entry["excitation_width_nm"], 60.0)
        self.assertEqual(entry["provenance"], PROV_NOMINAL)
        self.assertEqual(entry["cube_id"], "cy5")

    def test_junk_values_are_refused_not_coerced(self):
        self.assertIsNone(clean_nm("banana"))
        self.assertIsNone(clean_nm(0.519))       # µm typed as if nm
        self.assertIsNone(clean_nm(5_000_000))
        self.assertEqual(clean_nm("519"), 519.0)
        self.assertIsNone(clean_width_nm(-5))
        self.assertIsNone(clean_width_nm(9999))
        self.assertEqual(clean_width_nm(0), 0.0)   # 0 = width unknown

    def test_unknown_provenance_is_never_promoted(self):
        """Reading junk and calling it 'measured' is the overclaim this field
        exists to prevent."""
        self.assertEqual(clean_provenance("measured"), PROV_MEASURED)
        self.assertEqual(clean_provenance("MEASURED"), PROV_MEASURED)
        for junk in ("verified", "", None, "true", 7):
            self.assertEqual(clean_provenance(junk), PROV_NOMINAL)

    def test_name_matching_is_exact_never_fuzzy(self):
        """OpticsRegistry documents why at length: TxRed and mCherry are NOT
        the same cube, and a wrong match silently mislabels a channel."""
        self.assertEqual(self.store.find_by_name("Cy5").id, "cy5")
        self.assertEqual(self.store.find_by_name("  cy5 ").id, "cy5")
        self.assertEqual(self.store.find_by_name("GFPHQ").id, "gfp")  # block
        self.assertIsNone(self.store.find_by_name("Cy"))       # no prefix tier
        self.assertIsNone(self.store.find_by_name("Cy5-ish"))
        self.assertIsNone(self.store.find_by_name(""))
        # TxRed and mCherry are separate entries and must not collapse.
        self.assertNotEqual(self.store.find_by_name("TxRed").id,
                            self.store.find_by_name("mCherry").id)

    def test_user_cube_saves_and_shadows_a_builtin(self):
        custom = FilterCube(id="cy5", display_name="Cy5 (our part)",
                            excitation_nm=628.0, excitation_width_nm=40.0,
                            emission_nm=692.0, emission_width_nm=40.0,
                            provenance=PROV_DATASHEET)
        self.assertTrue(self.store.save_user(custom))
        got = self.store.get("cy5")
        self.assertEqual(got.display_name, "Cy5 (our part)")
        self.assertEqual(got.excitation_nm, 628.0)
        self.assertFalse(got.builtin)
        # Survives a reload from disk, and the builtin re-surfaces on delete.
        self.store.reload()
        self.assertEqual(self.store.get("cy5").excitation_nm, 628.0)
        self.assertTrue(self.store.delete_user("cy5"))
        self.assertEqual(self.store.get("cy5").excitation_nm, 620.0)
        self.assertTrue(self.store.get("cy5").builtin)

    def test_delete_user_never_touches_a_builtin(self):
        self.assertFalse(self.store.delete_user("dapi"))
        self.assertIsNotNone(self.store.get("dapi"))

    def test_a_file_may_hold_one_cube_or_a_list(self):
        """The shipped catalogue is one readable file; a saved cube is its own."""
        import json
        d = Path(self._tmp.name) / "many"
        d.mkdir()
        (d / "pair.json").write_text(json.dumps([
            {"id": "a", "display_name": "A", "emission_nm": 500},
            {"id": "b", "display_name": "B", "emission_nm": 600},
        ]), encoding="utf-8")
        (d / "single.json").write_text(json.dumps(
            {"id": "c", "display_name": "C", "emission_nm": 700}),
            encoding="utf-8")
        st = FilterCubeStore(builtin_dir=d, user_dir=Path(self._tmp.name) / "u2")
        self.assertEqual({c.id for c in st.all()}, {"a", "b", "c"})

    def test_corrupt_file_is_skipped_not_fatal(self):
        d = Path(self._tmp.name) / "bad"
        d.mkdir()
        (d / "broken.json").write_text("{not json", encoding="utf-8")
        (d / "ok.json").write_text('{"id": "ok", "emission_nm": 500}',
                                   encoding="utf-8")
        st = FilterCubeStore(builtin_dir=d, user_dir=Path(self._tmp.name) / "u3")
        self.assertEqual([c.id for c in st.all()], ["ok"])

    def test_safe_id_is_filesystem_safe(self):
        self.assertEqual(safe_id("Cy5 / custom"), "Cy5___custom")
        self.assertEqual(safe_id(""), "filter_cube")


# ── 2. The config store ────────────────────────────────────────────

class TestConfigStoreRanges(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "microscope.json"
        self.store = MicroscopeConfigStore(self.path)

    def test_pre_v718_entry_round_trips_byte_identically(self):
        """⚠ LOAD-BEARING. Every existing consumer — OpticsRegistry and the
        lablink.imagejob/1 sidecar — reads emission_nm/excitation_nm. A v7.18
        read/write cycle must not add, drop or reshape a single key, and in
        particular must NOT stamp a provenance onto values the operator typed
        themselves (absent is a third state, distinct from 'nominal')."""
        legacy = {"FITC": {"emission_nm": 550.0, "excitation_nm": 488.0}}
        self.store.set_all_filter_optics(legacy)
        self.assertEqual(self.store.filter_optics(), legacy)
        reloaded = MicroscopeConfigStore(self.path)
        self.assertEqual(reloaded.filter_optics(), legacy)
        self.assertNotIn("provenance", reloaded.filter_optics()["FITC"])

    def test_ranges_and_provenance_persist(self):
        self.store.set_filter_optics(
            "Cy5", excitation_nm=620, excitation_width_nm=60,
            emission_nm=700, emission_width_nm=75, dichroic_nm=660,
            provenance=PROV_NOMINAL, cube_id="cy5")
        entry = MicroscopeConfigStore(self.path).filter_optics_for("cy5")
        self.assertEqual(entry["excitation_nm"], 620.0)
        self.assertEqual(entry["excitation_width_nm"], 60.0)
        self.assertEqual(entry["emission_width_nm"], 75.0)
        self.assertEqual(entry["dichroic_nm"], 660.0)
        self.assertEqual(entry["provenance"], PROV_NOMINAL)
        self.assertEqual(entry["cube_id"], "cy5")

    def test_a_width_with_no_center_is_dropped(self):
        """A width alone describes no band; keeping it implies knowledge we
        do not have."""
        self.store.set_all_filter_optics(
            {"X": {"emission_nm": 500.0, "excitation_width_nm": 40.0}})
        entry = self.store.filter_optics()["X"]
        self.assertNotIn("excitation_width_nm", entry)
        self.assertEqual(entry["emission_nm"], 500.0)

    def test_an_entry_with_no_band_is_dropped_entirely(self):
        """A cube_id/provenance on its own says nothing measurable."""
        self.store.set_all_filter_optics(
            {"Bright Field": {"cube_id": "bright-field",
                              "provenance": "nominal"}})
        self.assertEqual(self.store.filter_optics(), {})

    def test_junk_provenance_degrades_and_never_promotes(self):
        self.store.set_all_filter_optics(
            {"A": {"emission_nm": 500.0, "provenance": "verified"}})
        self.assertEqual(self.store.filter_optics()["A"]["provenance"],
                         PROV_NOMINAL)

    def test_bad_width_is_refused_not_clamped(self):
        with self.assertRaises(ValueError):
            self.store.set_filter_optics("A", emission_nm=500,
                                         emission_width_nm=0.04)
        with self.assertRaises(ValueError):
            self.store.set_filter_optics("A", emission_nm=500,
                                         emission_width_nm=99999)

    def test_bad_dichroic_is_refused(self):
        with self.assertRaises(ValueError):
            self.store.set_filter_optics("A", emission_nm=500, dichroic_nm=0.5)


# ── 3. The setup panel ─────────────────────────────────────────────

class TestPanelCubePicker(_CubeCase):
    def setUp(self):
        super().setUp()
        self._mtmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._mtmp.cleanup)
        self.cfg = MicroscopeConfigStore(
            Path(self._mtmp.name) / "microscope.json")

    def _panel(self):
        from SupportClasses.MicroscopeControl import MicroscopeController
        ctrl = MicroscopeController(store=self.cfg, threaded=False)
        panel = setup_panel_mod.MicroscopeSetupPanel(self.cfg, controller=ctrl)
        return panel

    def _pick(self, panel, pos, cube_id):
        """Drive the row's combo exactly as an operator's pick does."""
        combo = panel._filter_table._rows[pos]["combo"]
        idx = combo.findData(cube_id)
        self.assertGreaterEqual(idx, 0, f"{cube_id} not in the pick-list")
        combo.setCurrentIndex(idx)
        panel._filter_table._on_cube_activated(pos)
        return combo

    def test_the_pick_list_holds_the_catalogue(self):
        panel = self._panel()
        combo = panel._filter_table._rows[1]["combo"]
        ids = {combo.itemData(i) for i in range(combo.count())}
        for expected in ("dapi", "fitc", "cy5", "txred"):
            self.assertIn(expected, ids)
        self.assertIn(setup_panel_mod._SlotTable._SAVE_CUSTOM, ids)

    def test_picking_a_cube_autofills_everything(self):
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        row = panel._filter_table._rows[4]
        self.assertEqual(row["edit"].text(), "Cy5")
        self.assertEqual(row["ex"].value(), 620)
        self.assertEqual(row["ex_width"].value(), 60)
        self.assertEqual(row["em"].value(), 700)
        self.assertEqual(row["em_width"].value(), 75)
        self.assertEqual(row["cube_id"], "cy5")
        self.assertEqual(row["provenance"], PROV_NOMINAL)

    def test_a_nominal_value_is_visibly_nominal(self):
        """The operator must be able to SEE that a number is not verified —
        that is the whole justification for auto-filling at all."""
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        prov = panel._filter_table._rows[4]["prov"]
        self.assertIn(PROV_NOMINAL, prov.text())
        self.assertIn("⚠", prov.text())
        self.assertIn("datasheet", prov.toolTip())

    def test_brightfield_pick_fills_a_name_but_no_numbers(self):
        panel = self._panel()
        self._pick(panel, 5, "bright-field")
        row = panel._filter_table._rows[5]
        self.assertEqual(row["edit"].text(), "Bright Field")
        self.assertEqual(row["ex"].value(), 0)
        self.assertEqual(row["em"].value(), 0)
        self.assertEqual(row["provenance"], "")
        self.assertEqual(row["prov"].text(), "—")

    def test_editing_a_number_promotes_it_off_nominal(self):
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        row = panel._filter_table._rows[4]
        row["em"].setValue(692)
        panel._filter_table.mark_edited(4)
        self.assertEqual(row["provenance"], PROV_DATASHEET)
        self.assertIn("✓", row["prov"].text())
        # ...but never all the way to "measured": nothing measured anything.
        self.assertNotEqual(row["provenance"], PROV_MEASURED)

    def test_commit_writes_the_ranges_to_the_store(self):
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        self.assertTrue(panel.commit())
        entry = self.cfg.filter_optics_for("Cy5")
        self.assertEqual(entry["excitation_nm"], 620.0)
        self.assertEqual(entry["emission_width_nm"], 75.0)
        self.assertEqual(entry["dichroic_nm"], 660.0)
        self.assertEqual(entry["provenance"], PROV_NOMINAL)
        self.assertEqual(entry["cube_id"], "cy5")

    def test_commit_does_not_mistake_provenance_for_a_wavelength(self):
        """⚠ REGRESSION, and it surfaced as a HANG rather than a failure.

        ``commit()`` validated EVERY value in an optics entry as a wavelength.
        Once entries carry ``provenance`` / ``cube_id`` strings, that check fails
        on a string, fires a modal QMessageBox and blocks forever under
        offscreen Qt. Asserted here as a fast, explicit check so the next
        regression is a red test and not a wedged suite."""
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        with mock.patch.object(setup_panel_mod.QMessageBox, "warning") as warn:
            self.assertTrue(panel.commit())
        warn.assert_not_called()

    def test_commit_still_refuses_an_implausible_wavelength(self):
        """The guard must not have been loosened into uselessness."""
        panel = self._panel()
        row = panel._filter_table._rows[4]
        row["edit"].setText("Odd")
        row["em"].setValue(700)
        # Reach past the spin's own range clamp, as a stored file could.
        with mock.patch.object(panel._filter_table, "optics",
                               return_value={"Odd": {"emission_nm": 0.7}}):
            with mock.patch.object(setup_panel_mod.QMessageBox,
                                   "warning") as warn:
                self.assertFalse(panel.commit())
        warn.assert_called_once()

    def test_a_saved_slot_reloads_with_its_bands_and_source(self):
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        panel.commit()
        again = self._panel()
        row = again._filter_table._rows[4]
        self.assertEqual(row["edit"].text(), "Cy5")
        self.assertEqual(row["ex_width"].value(), 60)
        self.assertEqual(row["provenance"], PROV_NOMINAL)
        self.assertEqual(row["combo"].currentData(), "cy5")

    def test_typing_a_custom_name_does_not_disturb_the_numbers(self):
        """An unlisted cube must stay typeable, and typing is not a pick."""
        panel = self._panel()
        self._pick(panel, 4, "cy5")
        row = panel._filter_table._rows[4]
        row["edit"].setText("Our odd cube")
        self.assertEqual(row["ex"].value(), 620)
        self.assertTrue(panel.commit())
        self.assertEqual(
            self.cfg.filter_optics_for("Our odd cube")["excitation_nm"], 620.0)

    def test_save_as_custom_joins_the_pick_list(self):
        panel = self._panel()
        row = panel._filter_table._rows[6]
        row["edit"].setText("House Cy5")
        row["ex"].setValue(628)
        row["ex_width"].setValue(40)
        row["em"].setValue(692)
        row["em_width"].setValue(40)
        with mock.patch.object(setup_panel_mod.QInputDialog, "getText",
                               return_value=("House Cy5", True)):
            panel._filter_table._save_slot_as_cube(6)
        saved = self.store.get("house_cy5")
        self.assertIsNotNone(saved)
        self.assertEqual(saved.excitation_nm, 628.0)
        self.assertEqual(saved.emission_width_nm, 40.0)
        self.assertFalse(saved.builtin)
        self.assertEqual(saved.provenance, PROV_DATASHEET)
        # It is now pickable on every row, not just the one it came from.
        combo = panel._filter_table._rows[1]["combo"]
        self.assertGreaterEqual(combo.findData("house_cy5"), 0)

    def test_save_as_custom_refuses_a_cube_with_no_bands(self):
        panel = self._panel()
        panel._filter_table._rows[6]["edit"].setText("Nothing")
        with mock.patch.object(setup_panel_mod.QInputDialog, "getText",
                               return_value=("Nothing", True)):
            with mock.patch.object(setup_panel_mod.QMessageBox,
                                   "warning") as warn:
                panel._filter_table._save_slot_as_cube(6)
        warn.assert_called_once()
        self.assertIsNone(self.store.get("nothing"))

    def test_save_as_custom_cancelled_writes_nothing(self):
        panel = self._panel()
        row = panel._filter_table._rows[6]
        row["edit"].setText("Nope")
        row["em"].setValue(700)
        with mock.patch.object(setup_panel_mod.QInputDialog, "getText",
                               return_value=("", False)):
            panel._filter_table._save_slot_as_cube(6)
        self.assertIsNone(self.store.get("nope"))

    def test_slot_count_change_keeps_the_bands(self):
        panel = self._panel()
        self._pick(panel, 6, "cy5")
        panel._filter_slots_spin.setValue(3)     # row 6 destroyed
        panel._filter_slots_spin.setValue(6)     # rebuilt
        row = panel._filter_table._rows[6]
        self.assertEqual(row["edit"].text(), "Cy5")
        self.assertEqual(row["em_width"].value(), 75)
        self.assertEqual(row["provenance"], PROV_NOMINAL)

    def test_objective_NAMES_stay_plain_text(self):
        """The nosepiece NAME column must stay a plain line edit.

        ⚠ Updated for v7.18, which gave objectives their own catalogue picker —
        so "the objective row has no combo at all" is no longer the contract.
        The surviving, and stronger, requirement is that the picker is a SEPARATE
        widget: an editable combo rewrites its own text on selection, which would
        rename the objective and orphan the µm/px calibration filed under that
        name. See ``test_v718_objective_catalogue
        .TestPickingAnObjectiveNeverRenamesTheSlot``.

        Objectives also still carry none of the filter-cube WAVELENGTH widgets —
        that half of the original assertion is unchanged.
        """
        from PySide6.QtWidgets import QLineEdit
        panel = self._panel()
        row = panel._objective_table._rows[1]
        self.assertIsInstance(row["edit"], QLineEdit)
        combo = row.get("combo")
        self.assertIsNotNone(combo, "the v7.18 objective picker is missing")
        self.assertFalse(combo.isEditable())
        self.assertIsNot(row["edit"], combo.lineEdit())
        # No filter-cube band widgets leaked onto the nosepiece rows.
        for key in ("em", "ex", "em_width", "ex_width"):
            self.assertNotIn(key, row)
        row["edit"].setText("40x")
        self.assertTrue(panel.commit())
        self.assertEqual(self.cfg.objective_label(1), "40x")


if __name__ == "__main__":
    unittest.main()
