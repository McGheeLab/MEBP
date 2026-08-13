"""
Objective names resolve across the case split that broke this rig's ladder.

THE LOAD-BEARING TEST is ``TestTheRigsActualDisagreement`` — it seeds the stores
with the EXACT shapes found on ME3B_01 on 2026-08-12 (nosepiece labels
``{1:"4X", 2:"10X", 3:"20x"}`` against calibration keys ``4x``/``10x``/``20x``,
all measured 2026-08-11 on camera ``tucam:0``) and asserts every rung resolves.
Before v7.18 ``ObjectiveCalibration.get_calibration`` was an exact,
case-sensitive nested ``dict.get``, so ``resolve_ladder`` reported *"no µm/px
calibration for '4X'"* for two objectives that were calibrated the day before,
and ``ladder_gate`` refused a survey over two spellings of the same name.

The counterweight is ``TestNoFallbackSurvivesTheFix``: folding case must not
become folding meaning. A name that is genuinely absent still refuses, and there
is still no fallback to the live manager or to ``current_objective_name`` — that
fallback is how "I literally just calibrated it" happened.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.ObjectiveCalibration import (      # noqa: E402
    ObjectiveCalibrationStore)
from SupportClasses.ObjectiveLadder import (           # noqa: E402
    ladder_gate, resolve_ladder)
from SupportClasses.OpticsRegistry import (            # noqa: E402
    normalize_optic_name)

# ── This rig's real data, 2026-08-12 (config/hardware/ME3B_01/) ──────────────
RIG_LABELS = {1: "4X", 2: "10X", 3: "20x"}          # microscope.json
RIG_CALS = {                                         # objectives.json, tucam:0
    "4x": {"measured_um_per_px": 1.896833, "resolution": [2600, 2048],
           "date": "2026-08-11", "rotation_deg": -0.191},
    "10x": {"measured_um_per_px": 0.752684, "resolution": [2600, 2048],
            "date": "2026-08-11", "rotation_deg": -0.191},
    "20x": {"measured_um_per_px": 0.370371, "resolution": [2600, 2048],
            "date": "2026-08-11", "rotation_deg": -0.141},
}
RIG_LIBRARY = [
    {"name": "4x", "nominal_magnification": 4.0},
    {"name": "10x", "nominal_magnification": 10.0},
    {"name": "20x", "nominal_magnification": 20.0},
]
CAM = "tucam:0"


class _Cfg:
    """Stands in for MicroscopeConfigStore's label half only."""

    def __init__(self, labels, aliases=None):
        self._labels = dict(labels)
        self._aliases = dict(aliases or {})

    def objective_labels(self):
        return dict(self._labels)

    def objective_slots(self):
        return len(self._labels)

    def optic_aliases(self, kind):
        return dict(self._aliases)


def _optic(pos, label, mag=None, na=None, wd=None, present=True):
    return SimpleNamespace(position=pos, present=present, label=label,
                           code=f"MRH{pos:05d}", magnification=mag,
                           numerical_aperture=na, working_distance_mm=wd)


def _state(count=5, position=1):
    """The body's own answer on this rig: lowercase labels, WD 16.4 → 1.0."""
    return SimpleNamespace(
        objective_count=count,
        objective_position=position,
        native_objective_names=("4x", "10x", "20x", "", ""),
        mounted_objectives=(
            _optic(1, "4x", 4.0, 0.13, 16.4),
            _optic(2, "10x", 10.0, 0.30, 4.0),
            _optic(3, "20x", 20.0, 0.45, 1.0),
        ))


def _store(cals=None, library=None):
    """A real ObjectiveCalibrationStore over a temp file — not a stand-in."""
    d = tempfile.mkdtemp()
    path = os.path.join(d, "objectives.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"version": "1.0",
                   "objectives": list(RIG_LIBRARY if library is None else library),
                   "camera_objective_calibrations": {
                       CAM: dict(RIG_CALS if cals is None else cals)}}, f)
    return ObjectiveCalibrationStore(__import__("pathlib").Path(path)), d


class _StoreCase(unittest.TestCase):
    def setUp(self):
        self.store, self._dir = _store()
        self.addCleanup(shutil.rmtree, self._dir, ignore_errors=True)


class TestNormalize(unittest.TestCase):
    def test_case_and_spacing_fold(self):
        for a, b in (("4X", "4x"), (" 4x ", "4x"), ("4  X", "4 x"),
                     ("Bright Field", "BRIGHT   FIELD")):
            self.assertEqual(normalize_optic_name(a), normalize_optic_name(b),
                             f"{a!r} should fold onto {b!r}")

    def test_different_names_do_not_fold(self):
        """Typography folds; meaning does not."""
        self.assertNotEqual(normalize_optic_name("mCherry"),
                            normalize_optic_name("TxRed"))
        self.assertNotEqual(normalize_optic_name("4x"),
                            normalize_optic_name("40x"))

    def test_empty_and_none_are_empty(self):
        self.assertEqual(normalize_optic_name(None), "")
        self.assertEqual(normalize_optic_name("   "), "")


class TestTheRigsActualDisagreement(_StoreCase):
    """The exact on-disk pair that shipped, and the ladder that refused it."""

    def test_uppercase_label_finds_the_lowercase_calibration(self):
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "4X")["measured_um_per_px"],
            1.896833, places=6)
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "10X")["measured_um_per_px"],
            0.752684, places=6)

    def test_exact_match_still_wins_untouched(self):
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "20x")["measured_um_per_px"],
            0.370371, places=6)

    def test_nominal_magnification_folds_too(self):
        self.assertEqual(self.store.nominal_magnification("4X"), 4.0)
        self.assertEqual(self.store.nominal_magnification("10X"), 10.0)

    def test_every_rung_of_this_rigs_ladder_is_calibrated(self):
        """The plate-bed-leveling ladder was unusable for 4X and 10X."""
        rungs = resolve_ladder(scope_state=_state(), config_store=_Cfg(RIG_LABELS),
                               objective_store=self.store, camera_name=CAM,
                               live_resolution=(2600, 2048),
                               positions=[1, 2, 3])
        for r in rungs:
            self.assertTrue(r.calibrated,
                            f"position {r.turret_position} ({r.objective_name}) "
                            f"still refuses: {r.why_not}")
        self.assertEqual([round(r.um_per_px, 6) for r in rungs],
                         [1.896833, 0.752684, 0.370371])

    def test_ladder_gate_no_longer_refuses_over_letter_case(self):
        """Both statements were true and nothing was actually wrong."""
        rungs = resolve_ladder(scope_state=_state(), config_store=_Cfg(RIG_LABELS),
                               objective_store=self.store, camera_name=CAM,
                               live_resolution=(2600, 2048), positions=[1, 2, 3])
        ok, why = ladder_gate(rungs, current_objective_name="4x",
                              live_turret_position=1)
        self.assertTrue(ok, why)

    def test_a_real_disagreement_still_refuses(self):
        """Folding case must not fold meaning: 20x at position 1 is a fault."""
        rungs = resolve_ladder(scope_state=_state(), config_store=_Cfg(RIG_LABELS),
                               objective_store=self.store, camera_name=CAM,
                               live_resolution=(2600, 2048), positions=[1, 2, 3])
        ok, why = ladder_gate(rungs, current_objective_name="20x",
                              live_turret_position=1)
        self.assertFalse(ok)
        self.assertIn("4X", why)
        self.assertIn("20x", why)

    def test_an_alias_can_settle_a_genuine_rename(self):
        rungs = resolve_ladder(scope_state=_state(), config_store=_Cfg(RIG_LABELS),
                               objective_store=self.store, camera_name=CAM,
                               live_resolution=(2600, 2048), positions=[1, 2, 3])
        ok, _ = ladder_gate(rungs, current_objective_name="Plan Fluor 4x",
                            live_turret_position=1,
                            aliases={"Plan Fluor 4x": "4X"})
        self.assertTrue(ok)


class TestAmbiguityRefuses(unittest.TestCase):
    """Two keys that fold together make the FOLDED lookup unusable.

    Note what is deliberately NOT ambiguous: an EXACT key always wins, because a
    character-for-character match is unambiguous by definition and folding must
    never take a correct answer away. Ambiguity is only reachable at the
    normalized tier — a spelling that matches neither key exactly.
    """

    def setUp(self):
        cals = dict(RIG_CALS)          # has "4x"
        cals["4X"] = {"measured_um_per_px": 99.0, "resolution": [2600, 2048]}
        self.store, d = _store(cals=cals)
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)

    def test_an_exact_key_is_still_returned(self):
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "4x")["measured_um_per_px"],
            1.896833, places=6)
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "4X")["measured_um_per_px"], 99.0)

    def test_a_folded_lookup_picks_NEITHER(self):
        """' 4x ' matches both keys once folded — a coin toss would silently make
        one measurement stand in for the other."""
        self.assertIsNone(self.store.get_calibration(CAM, " 4x "))

    def test_it_says_so_naming_both_keys(self):
        with self.assertLogs("SupportClasses.ObjectiveCalibration",
                             level="WARNING") as cm:
            self.store.get_calibration(CAM, " 4x ")
        joined = " ".join(cm.output)
        self.assertIn("'4X'", joined)
        self.assertIn("'4x'", joined)

    def test_an_unambiguous_sibling_is_unaffected(self):
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "10X")["measured_um_per_px"],
            0.752684, places=6)


class TestNoFallbackSurvivesTheFix(_StoreCase):
    def test_an_absent_objective_still_refuses(self):
        self.assertIsNone(self.store.get_calibration(CAM, "40x"))
        self.assertIsNone(self.store.get_calibration(CAM, ""))

    def test_an_uncalibrated_rung_refuses_and_names_the_camera(self):
        rungs = resolve_ladder(
            scope_state=_state(), config_store=_Cfg({1: "60x"}),
            objective_store=self.store, camera_name=CAM, positions=[1])
        self.assertFalse(rungs[0].calibrated)
        self.assertIn("60x", rungs[0].why_not)
        self.assertIn(CAM, rungs[0].why_not)

    def test_an_unnamed_slot_refuses_and_points_at_the_setup_page(self):
        rungs = resolve_ladder(
            scope_state=_state(), config_store=_Cfg({}),
            objective_store=self.store, camera_name=CAM, positions=[4])
        self.assertFalse(rungs[0].calibrated)
        self.assertIn("no name", rungs[0].why_not)

    def test_another_camera_is_not_borrowed_from(self):
        self.assertIsNone(self.store.get_calibration("andor:VSC-1", "4x"))


class TestWritesDoNotCreateCaseSiblings(_StoreCase):
    def test_saving_under_a_variant_reuses_the_existing_key(self):
        """A sibling would make every later read ambiguous, i.e. uncalibrated —
        the save would appear to succeed and then be ignored."""
        self.store.set_calibration(CAM, "4X", 2.0, (2600, 2048))
        keys = sorted(self.store.all_calibrations_for_camera(CAM))
        self.assertEqual(keys, ["10x", "20x", "4x"])
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "4x")["measured_um_per_px"], 2.0)

    def test_a_genuinely_new_objective_still_gets_its_own_key(self):
        self.store.set_calibration(CAM, "60x", 0.12, (2600, 2048))
        self.assertIn("60x", self.store.all_calibrations_for_camera(CAM))

    def test_rotation_is_still_preserved_on_a_um_per_px_only_update(self):
        self.store.set_calibration(CAM, "4X", 2.0, (2600, 2048))
        self.assertAlmostEqual(
            self.store.get_calibration(CAM, "4x")["rotation_deg"], -0.191)

    def test_add_objective_refuses_a_case_only_duplicate(self):
        """Adding '4X' beside '4x' is how the split was created."""
        self.assertFalse(self.store.add_objective("4X", 4.0))
        self.assertEqual(self.store.objective_names(), ["4x", "10x", "20x"])

    def test_add_objective_still_accepts_a_real_new_name(self):
        self.assertTrue(self.store.add_objective("60x", 60.0))
        self.assertIn("60x", self.store.objective_names())

    def test_clear_resolves_the_variant_it_can_read(self):
        self.store.clear_calibration(CAM, "4X")
        self.assertIsNone(self.store.get_calibration(CAM, "4x"))

    def test_remove_objective_leaves_no_case_variant_orphan(self):
        cals = dict(RIG_CALS)
        cals["4X"] = {"measured_um_per_px": 9.0, "resolution": [2600, 2048]}
        store, d = _store(cals=cals)
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        store.remove_objective("4x")
        left = sorted(store.all_calibrations_for_camera(CAM))
        self.assertEqual(left, ["10x", "20x"])


if __name__ == "__main__":
    unittest.main()
