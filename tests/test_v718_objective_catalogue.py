"""
Objectives are classified, so depth of field and the collision bound are real.

Two figures do actual work here, and until v7.18 both came only from the body —
which knows nothing except the product code programmed into its nosepiece:

* **NA sets the depth of field**, which sizes every focus step. The rig's real
  20x is a Plan Apo VC /0.75; assuming a /0.45 gives 3.54 µm of DOF where the
  truth is 1.47 µm, so the sweep steps 2.4x too coarsely and "a real peak can
  hide between samples" — the planner's own words.
* **Working distance is the collision bound.** A Plan Fluor 10x/0.30 has 16.0 mm;
  a Plan Achromat 20x/0.40 has 1.2 mm. Confusing them is a 13x error in the
  direction that drives a front lens into glass.

THE LOAD-BEARING TESTS are ``TestTheWorkingDistanceFailSafe`` (a body-vs-spec
disagreement keeps the SHORTER value, so a rotation can never be authorised on the
more generous of two numbers) and
``TestPickingAnObjectiveNeverRenamesTheSlot`` — the slot label keys this
objective's µm/px calibration, its parfocal offset and its own spec, so a picker
that rewrote it would orphan the operator's measurements.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.MicroscopeConfigStore import (        # noqa: E402
    MicroscopeConfigStore)
from SupportClasses.ObjectiveCatalogue import (           # noqa: E402
    IMMERSION_N, PROV_DATASHEET, PROV_MEASURED, PROV_NOMINAL, Objective,
    ObjectiveCatalogue, clean_na, clean_wd_mm, immersion_n, safe_id)
from SupportClasses.ObjectiveOptics import (              # noqa: E402
    ObjectiveOptics, depth_of_field_um, wd_bounded_half_range_um)
from SupportClasses.OpticsRegistry import (               # noqa: E402
    resolve_objectives)

REPO = Path(__file__).resolve().parent.parent
BUNDLED = (REPO / "config/hardware/ME3B_general/objective_types/builtin"
           / "nikon_cfi.json")

# The rig's real parts and their measured scales, 2026-08-12.
RIG = {
    "4X": ("nikon-cfi-plan-fluor-4x-013", 0.13, 17.1, 1.896833),
    "10X": ("nikon-cfi-plan-fluor-10x-030-dic", 0.30, 16.0, 0.752684),
    "20x": ("nikon-cfi-plan-apo-vc-20x-075", 0.75, 1.0, 0.370371),
}


def _optic(pos, label, mag, na, wd):
    return SimpleNamespace(position=pos, present=True, label=label, code=f"C{pos}",
                           magnification=mag, numerical_aperture=na,
                           working_distance_mm=wd)


def _state(objectives=None):
    """The body reporting the numbers it reported before the catalogue existed."""
    return SimpleNamespace(
        objective_count=5, objective_position=1,
        native_objective_names=("4x", "10x", "20x", "", ""),
        mounted_objectives=tuple(objectives if objectives is not None else (
            _optic(1, "4x", 4.0, 0.13, 16.4),
            _optic(2, "10x", 10.0, 0.30, 4.0),
            _optic(3, "20x", 20.0, 0.45, 1.0))))


class _CfgCase(unittest.TestCase):
    def setUp(self):
        self._dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._dir, ignore_errors=True)
        self.cfg = MicroscopeConfigStore(Path(self._dir) / "microscope.json")
        self.cfg.set_objective_labels({"1": "4X", "2": "10X", "3": "20x"})
        self.cat = ObjectiveCatalogue(
            builtin_dir=BUNDLED.parent, user_dir=Path(self._dir) / "user")

    def _assign(self, *labels):
        for label in labels:
            obj = self.cat.get(RIG[label][0])
            self.cfg.set_objective_spec(label, **obj.spec_entry())


class TestTheBundledCatalogue(_CfgCase):
    def test_it_loads(self):
        self.assertGreater(len(self.cat.all()), 20)

    def test_the_rigs_three_objectives_are_present_with_their_real_numbers(self):
        for label, (oid, na, wd, _upp) in RIG.items():
            obj = self.cat.get(oid)
            self.assertIsNotNone(obj, f"{label} missing from the catalogue")
            self.assertAlmostEqual(obj.numerical_aperture, na)
            self.assertAlmostEqual(obj.working_distance_mm, wd)

    def test_they_are_marked_datasheet_not_nominal(self):
        """The operator read these off the parts, so they outrank a guess."""
        for label, (oid, *_rest) in RIG.items():
            self.assertEqual(self.cat.get(oid).provenance, PROV_DATASHEET, label)

    def test_every_other_entry_is_marked_NOMINAL(self):
        """Bundled series figures are not verified for anyone's actual part, and
        claiming otherwise is how a wrong working distance becomes trusted."""
        rig_ids = {oid for oid, *_ in RIG.values()}
        others = [o for o in self.cat.all() if o.id not in rig_ids]
        self.assertTrue(others)
        for obj in others:
            self.assertEqual(obj.provenance, PROV_NOMINAL, obj.id)

    def test_every_entry_can_size_a_sweep_AND_bound_a_rotation(self):
        for obj in self.cat.all():
            self.assertTrue(obj.has_optics, f"{obj.id} lacks NA or WD")

    def test_oil_and_water_entries_carry_a_real_refractive_index(self):
        wet = [o for o in self.cat.all() if o.immersion != "air"]
        self.assertTrue(wet, "the catalogue should include immersion objectives")
        for obj in wet:
            self.assertGreater(obj.immersion_n, 1.0, obj.id)
            self.assertEqual(obj.immersion_n, IMMERSION_N[obj.immersion])

    def test_no_entry_invents_a_product_code(self):
        """A wrong code silently invalidates a parfocal offset via the
        product-code check in MicroscopeConfigStore.parfocal_offset_um."""
        for obj in self.cat.all():
            self.assertEqual(obj.product_code, "", obj.id)

    def test_the_elwd_entries_record_the_SHORTEST_collar_position(self):
        """WD varies with the correction collar and software cannot read it, so
        the collision bound must assume the worst case."""
        elwd = [o for o in self.cat.all() if "ELWD" in o.series]
        self.assertTrue(elwd)
        for obj in elwd:
            self.assertIn("shortest", obj.notes.lower())

    def test_the_bundled_file_is_valid_json_and_a_list(self):
        raw = json.loads(BUNDLED.read_text(encoding="utf-8"))
        self.assertIsInstance(raw, list)
        self.assertEqual(len({e["id"] for e in raw}), len(raw),
                         "duplicate ids in the bundled catalogue")


class TestDepthOfFieldIsNowRight(_CfgCase):
    def _dof(self, label):
        oid, na, wd, upp = RIG[label]
        obj = self.cat.get(oid)
        return depth_of_field_um(ObjectiveOptics(
            label=label, position=1, magnification=obj.magnification,
            numerical_aperture=obj.numerical_aperture,
            working_distance_mm=obj.working_distance_mm,
            um_per_px_sample=upp, immersion_n=obj.immersion_n))

    def test_the_20x_is_much_shallower_than_the_assumed_045_would_give(self):
        """The correction that matters: the sweep step was 2.4x too coarse."""
        real = self._dof("20x")
        assumed = depth_of_field_um(ObjectiveOptics(
            label="20x", position=3, magnification=20.0,
            numerical_aperture=0.45, working_distance_mm=1.0,
            um_per_px_sample=RIG["20x"][3]))
        self.assertLess(real, assumed / 2.0,
                        f"real {real:.2f} vs assumed {assumed:.2f} µm")
        self.assertAlmostEqual(real, 1.47, places=1)

    def test_the_4x_and_10x_are_unchanged_because_their_NA_was_already_right(self):
        self.assertAlmostEqual(self._dof("4X"), 47.1, places=0)
        self.assertAlmostEqual(self._dof("10X"), 8.6, places=0)

    def test_an_oil_objective_uses_its_own_refractive_index(self):
        """Left at 1.0 the diffraction term is wrong by that factor."""
        oil = self.cat.get("nikon-cfi-plan-apo-lambda-100x-145-oil")
        common = dict(label="100x", position=1, magnification=100.0,
                      numerical_aperture=oil.numerical_aperture,
                      working_distance_mm=oil.working_distance_mm,
                      um_per_px_sample=0.07)
        wet = depth_of_field_um(ObjectiveOptics(**common,
                                               immersion_n=oil.immersion_n))
        dry = depth_of_field_um(ObjectiveOptics(**common, immersion_n=1.0))
        self.assertGreater(wet, dry * 1.4)

    def test_the_index_reaches_the_slot_THROUGH_the_registry(self):
        """⚠ Added after a mutation SURVIVED: the test above builds
        ``ObjectiveOptics`` straight from the catalogue, so it says nothing about
        whether ``resolve_slots`` actually carries the index. Forcing the
        registry's index to 1.0 passed everything until this existed."""
        self.cfg.set_objective_labels({"1": "100x oil"})
        oil = self.cat.get("nikon-cfi-plan-apo-lambda-100x-145-oil")
        self.cfg.set_objective_spec("100x oil", **oil.spec_entry())
        st = SimpleNamespace(
            objective_count=1, objective_position=1,
            native_objective_names=("100x",),
            mounted_objectives=(_optic(1, "100x", 100.0, 1.45, 0.13),))
        slot = resolve_objectives(scope_state=st, config_store=self.cfg)[0]
        self.assertAlmostEqual(slot.immersion_n, 1.515, places=3)

    def test_a_dry_objective_stays_at_one_through_the_registry(self):
        self._assign("20x")
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "20x"][0]
        self.assertAlmostEqual(slot.immersion_n, 1.0)

    def test_the_LADDER_carries_the_index_too(self):
        """The ladder is what the focus sweep is sized from, so a rung that
        dropped the index would compute a dry depth of field for an oil lens."""
        self.cfg.set_objective_labels({"1": "100x oil"})
        oil = self.cat.get("nikon-cfi-plan-apo-lambda-100x-145-oil")
        self.cfg.set_objective_spec("100x oil", **oil.spec_entry())

        class _Store:
            def get_calibration(self, cam, obj):
                return {"measured_um_per_px": 0.07, "resolution": [2600, 2048]}

        from SupportClasses.ObjectiveLadder import resolve_ladder
        st = SimpleNamespace(
            objective_count=1, objective_position=1,
            native_objective_names=("100x",),
            mounted_objectives=(_optic(1, "100x", 100.0, 1.45, 0.13),))
        rung = resolve_ladder(scope_state=st, config_store=self.cfg,
                              objective_store=_Store(), camera_name="tucam:0",
                              live_resolution=(2600, 2048), positions=[1])[0]
        self.assertAlmostEqual(rung.immersion_n, 1.515, places=3)


class TestTheWorkingDistanceFailSafe(_CfgCase):
    """A rotation must never be authorised on the more generous of two numbers."""

    def test_the_shorter_value_wins_when_they_disagree(self):
        self._assign("10X")
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "10X"][0]
        # Catalogue says 16.0, the body insists 4.0.
        self.assertAlmostEqual(slot.working_distance_mm, 4.0)
        self.assertTrue(slot.optics_conflict)
        self.assertIn("4", slot.optics_conflict)
        self.assertIn("16", slot.optics_conflict)

    def test_the_conflict_says_how_to_settle_it(self):
        self._assign("10X")
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "10X"][0]
        self.assertIn("measured", slot.optics_conflict.lower())

    def test_a_MEASURED_spec_wins_outright(self):
        """The escape hatch: without it, a body reporting a wrong product code
        would permanently cap a correct objective's travel."""
        self.cfg.set_objective_spec(
            "10X", numerical_aperture=0.30, working_distance_mm=16.0,
            provenance=PROV_MEASURED)
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "10X"][0]
        self.assertAlmostEqual(slot.working_distance_mm, 16.0)
        self.assertIn("MEASURED", slot.optics_conflict)

    def test_a_datasheet_spec_does_NOT_win_outright(self):
        """Picking from the catalogue must not silently raise the bound."""
        self.cfg.set_objective_spec(
            "10X", numerical_aperture=0.30, working_distance_mm=16.0,
            provenance=PROV_DATASHEET)
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "10X"][0]
        self.assertAlmostEqual(slot.working_distance_mm, 4.0)

    def test_agreement_raises_no_conflict(self):
        self._assign("20x")
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "20x"][0]
        self.assertAlmostEqual(slot.working_distance_mm, 1.0)
        self.assertEqual(slot.optics_conflict, "")

    def test_the_spec_wins_on_NA_even_while_WD_is_fail_safed(self):
        """Different rules on purpose: NA is not a collision bound."""
        self._assign("20x")
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "20x"][0]
        self.assertAlmostEqual(slot.numerical_aperture, 0.75,
                               msg="the body's 0.45 should have been overridden")

    def test_a_body_reporting_nothing_still_gets_the_spec(self):
        self._assign("4X")
        st = _state(objectives=(_optic(1, "4x", None, None, None),))
        slot = [s for s in resolve_objectives(scope_state=st,
                                              config_store=self.cfg)
                if s.name == "4X"][0]
        self.assertAlmostEqual(slot.numerical_aperture, 0.13)
        self.assertAlmostEqual(slot.working_distance_mm, 17.1)
        self.assertEqual(slot.optics_conflict, "")

    def test_no_spec_leaves_the_body_untouched(self):
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "10X"][0]
        self.assertAlmostEqual(slot.working_distance_mm, 4.0)
        self.assertEqual(slot.optics_source, "body")

    def test_the_bound_follows_the_resolved_working_distance(self):
        self._assign("20x")
        slot = [s for s in resolve_objectives(scope_state=_state(),
                                              config_store=self.cfg)
                if s.name == "20x"][0]
        half, known = wd_bounded_half_range_um(ObjectiveOptics(
            label="20x", position=3, numerical_aperture=slot.numerical_aperture,
            working_distance_mm=slot.working_distance_mm,
            um_per_px_sample=0.37))
        self.assertTrue(known)
        self.assertAlmostEqual(half, 250.0, places=0)


class TestTheStoreRefusesUnitSlips(_CfgCase):
    def test_a_working_distance_in_MICRONS_is_refused(self):
        """17100 would authorise a 17-metre focus excursion."""
        with self.assertRaises(ValueError) as cm:
            self.cfg.set_objective_spec("4X", working_distance_mm=17100)
        self.assertIn("MILLIMETRES", str(cm.exception))

    def test_an_NA_as_a_percentage_is_refused(self):
        with self.assertRaises(ValueError):
            self.cfg.set_objective_spec("20x", numerical_aperture=75)

    def test_an_entry_with_neither_NA_nor_WD_is_not_stored(self):
        """The cleaner drops it on reload, so accepting it would look saved and
        vanish on the next launch."""
        self.cfg.set_objective_spec("4X", coverslip_mm=0.17)
        self.assertEqual(self.cfg.objective_spec_for("4X"), {})

    def test_a_spec_is_matched_case_insensitively(self):
        self._assign("4X")
        self.assertTrue(self.cfg.objective_spec_for("4x"))
        self.assertTrue(self.cfg.objective_spec_for("  4X "))

    def test_it_survives_a_reload(self):
        self._assign("4X", "10X", "20x")
        again = MicroscopeConfigStore(Path(self._dir) / "microscope.json")
        self.assertEqual(sorted(again.objective_specs()), ["10X", "20x", "4X"])
        self.assertAlmostEqual(
            again.objective_spec_for("20x")["numerical_aperture"], 0.75)

    def test_a_malformed_block_is_dropped_not_coerced(self):
        path = Path(self._dir) / "junk.json"
        path.write_text(
            '{"objective_specs": {"a": {"numerical_aperture": "x"},'
            ' "b": {"coverslip_mm": 0.17}, "c": "nope",'
            ' "d": {"working_distance_mm": 99999}}}', encoding="utf-8")
        cfg = MicroscopeConfigStore(path)
        self.assertEqual(cfg.objective_specs(), {})

    def test_clearing_removes_the_entry(self):
        self._assign("4X")
        self.cfg.set_objective_spec("4X")
        self.assertEqual(self.cfg.objective_spec_for("4X"), {})


class TestCatalogueHelpers(unittest.TestCase):
    def test_clean_na_rejects_a_percentage_and_accepts_a_real_NA(self):
        self.assertIsNone(clean_na(75))
        self.assertIsNone(clean_na("x"))
        self.assertAlmostEqual(clean_na(0.75), 0.75)

    def test_clean_wd_rejects_microns_and_accepts_millimetres(self):
        self.assertIsNone(clean_wd_mm(17100))
        self.assertAlmostEqual(clean_wd_mm(17.1), 17.1)

    def test_the_immersion_table_has_exactly_ONE_home(self):
        """It lives in OpticsRegistry (which must import nothing from the repo)
        and is re-exported here — a second copy is the two-homes trap."""
        from SupportClasses import ObjectiveCatalogue as cat
        from SupportClasses import OpticsRegistry as reg
        self.assertIs(cat.IMMERSION_N, reg.IMMERSION_N)
        self.assertIs(cat.immersion_n, reg.immersion_n)
        self.assertIs(cat.clean_immersion, reg.clean_immersion)

    def test_immersion_defaults_to_dry_for_anything_unknown(self):
        self.assertEqual(immersion_n("unobtainium"), 1.0)
        self.assertEqual(immersion_n(None), 1.0)
        self.assertAlmostEqual(immersion_n("oil"), 1.515)

    def test_safe_id_survives_a_filename(self):
        self.assertEqual(safe_id("CFI Plan Fluor 4x/0.13"),
                         "CFI_Plan_Fluor_4x_0.13")

    def test_no_fuzzy_name_match(self):
        """'20x' names a magnification, not a part — and this rig's 20x/0.75 and
        a Plan Achromat 20x/0.40 differ by 13x in working distance."""
        cat = ObjectiveCatalogue(builtin_dir=BUNDLED.parent,
                                 user_dir=Path(tempfile.mkdtemp()) / "u")
        self.assertIsNone(cat.find_by_name("20x"))
        self.assertIsNone(cat.find_by_name("Plan"))
        self.assertIsNotNone(cat.find_by_name("CFI Plan Fluor 4x/0.13"))


class TestUserEntriesShadowBuiltins(_CfgCase):
    def test_a_saved_user_entry_wins_over_a_builtin_of_the_same_id(self):
        oid = RIG["4X"][0]
        self.assertAlmostEqual(self.cat.get(oid).working_distance_mm, 17.1)
        self.assertTrue(self.cat.save_user(Objective(
            id=oid, name="my 4x", numerical_aperture=0.13,
            working_distance_mm=16.9, provenance=PROV_MEASURED)))
        self.assertAlmostEqual(self.cat.get(oid).working_distance_mm, 16.9)
        self.assertEqual(self.cat.get(oid).provenance, PROV_MEASURED)

    def test_deleting_the_user_entry_restores_the_builtin(self):
        oid = RIG["4X"][0]
        self.cat.save_user(Objective(id=oid, name="my 4x",
                                     numerical_aperture=0.13,
                                     working_distance_mm=16.9))
        self.assertTrue(self.cat.delete_user(oid))
        self.assertAlmostEqual(self.cat.get(oid).working_distance_mm, 17.1)

    def test_deleting_a_builtin_is_a_no_op(self):
        self.assertFalse(self.cat.delete_user(RIG["20x"][0]))


class TestPickingAnObjectiveNeverRenamesTheSlot(unittest.TestCase):
    """The slot label keys the µm/px calibration, the parfocal offset and the
    spec itself, so a picker that rewrote it would orphan every measurement."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self._dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._dir, ignore_errors=True)
        from gui.pages.hardware.microscope_setup_panel import _SlotTable
        self.table = _SlotTable("objective", lambda _p: None)
        self.addCleanup(self.table.deleteLater)
        self.table.set_slot_count(3)
        self.table.set_labels({1: "4X", 2: "10X", 3: "20x"})
        self.cat = ObjectiveCatalogue(
            builtin_dir=BUNDLED.parent, user_dir=Path(self._dir) / "user")

    def _pick(self, pos, label):
        obj = self.cat.get(RIG[label][0])
        self.table._apply_objective(pos, obj)
        return obj

    def test_the_name_is_untouched(self):
        self._pick(1, "4X")
        self.assertEqual(self.table._rows[1]["edit"].text(), "4X")

    def test_the_optics_ARE_filled_in(self):
        self._pick(1, "4X")
        row = self.table._rows[1]
        self.assertAlmostEqual(row["na"].value(), 0.13, places=2)
        self.assertAlmostEqual(row["wd"].value(), 17.1, places=2)
        self.assertEqual(row["provenance"], PROV_DATASHEET)

    def test_specs_are_keyed_by_the_OPERATORS_label(self):
        for pos, label in ((1, "4X"), (2, "10X"), (3, "20x")):
            self._pick(pos, label)
        self.assertEqual(sorted(self.table.specs()), ["10X", "20x", "4X"])

    def test_a_blank_slot_is_filled_in_as_a_convenience(self):
        self.table.set_labels({1: "", 2: "", 3: ""})
        obj = self._pick(1, "20x")
        self.assertEqual(self.table._rows[1]["edit"].text(),
                         obj.name or obj.label)

    def test_the_picker_is_not_editable(self):
        """An editable combo rewrites its own text on selection — which is what
        clobbered the name, and no handler can undo it."""
        self.assertFalse(self.table._rows[1]["combo"].isEditable())

    def test_the_name_column_is_still_a_plain_line_edit(self):
        from PySide6.QtWidgets import QLineEdit
        self.assertIsInstance(self.table._rows[1]["edit"], QLineEdit)

    def test_a_round_trip_through_the_table_preserves_everything(self):
        for pos, label in ((1, "4X"), (2, "10X"), (3, "20x")):
            self._pick(pos, label)
        saved = self.table.specs()
        self.table.set_specs(saved)
        self.assertEqual(self.table.specs(), saved)
        self.assertEqual(self.table.labels()[1], "4X")

    def test_a_hand_edit_promotes_a_nominal_row_off_nominal(self):
        obj = self.cat.get("nikon-cfi-plan-fluor-20x-050")   # nominal
        self.table._apply_objective(1, obj)
        self.assertEqual(self.table._rows[1]["provenance"], PROV_NOMINAL)
        self.table._rows[1]["na"].setValue(0.52)
        self.table.mark_edited(1)
        self.assertEqual(self.table._rows[1]["provenance"], PROV_DATASHEET)

    def test_an_empty_row_reports_no_spec(self):
        self.assertEqual(self.table.specs(), {})


if __name__ == "__main__":
    unittest.main()
