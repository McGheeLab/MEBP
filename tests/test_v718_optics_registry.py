"""
Name → turret slot, the reverse lookup that existed nowhere before v7.18.

THE LOAD-BEARING TEST is ``TestMCherryDoesNotResolveToTxRed``. This rig's
cassette holds DAPI(1) FITC(2) **TxRed**(3) Cy5(4) with slots 5 and 6 empty,
while the app's imaging-channel vocabulary — and the built-in target-type rules —
say *mCherry*. TxRed is a red cube but it is **not** mCherry, and a TxRed image
filed as an mCherry channel is a result no downstream consumer can detect and no
one can un-publish. So the matcher must REFUSE until the operator declares the
equivalence, and it must never reach for the legacy ``CHANNEL_NUMBERS`` ordinal
table (whose "mCherry → 3" names the wrong cube and whose "Bright Field → 5"
names an EMPTY slot on this machine).

Its counterweight is ``TestTheCaseFixStillWorks``: refusing to guess must not
also refuse the one difference that IS safe to fold — "4X" vs "4x".

``NikonTiSdkBackend._set_turret`` already documents this SDK clamping slot 999 to
6 and **reporting success**; every refusal here exists because a wrong answer that
looks like a right answer is the failure mode this hardware actually has.
"""

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.MicroscopeConfigStore import (     # noqa: E402
    MicroscopeConfigStore)
from SupportClasses.OpticsRegistry import (            # noqa: E402
    FILTER, HOW_ALIAS, HOW_EXACT, HOW_NATIVE, HOW_NORMALIZED, OBJECTIVE,
    OpticSlot, find_slot, optic_at, resolve_filters, resolve_objectives,
    snapshot)

# ── This rig, 2026-08-12 (config/hardware/ME3B_01/microscope.json) ───────────
RIG_CUBES = {"1": "DAPI", "2": "FITC", "3": "TxRed", "4": "Cy5"}
RIG_OBJECTIVES = {"1": "4X", "2": "10X", "3": "20x"}


def _optic(pos, label, present=True, mag=None, na=None, wd=None):
    return SimpleNamespace(position=pos, present=present, label=label,
                           code=f"C{pos}", magnification=mag,
                           numerical_aperture=na, working_distance_mm=wd)


def _state(**over):
    """What the real Ti reports on this rig, including the '-----' empties."""
    base = dict(
        connected=True, backend="nikon_ti", busy=False, error=None,
        filter_count=6, objective_count=5,
        filter_position=2, objective_position=1,
        native_filter_names=("DAPI", "FITC", "TxRed", "Cy5", "-----", "-----"),
        native_objective_names=("4x", "10x", "20x", "", ""),
        mounted_filters=(
            _optic(1, "DAPI"), _optic(2, "FITC"), _optic(3, "TxRed"),
            _optic(4, "Cy5"), _optic(5, "-----", present=False),
            _optic(6, "-----", present=False)),
        mounted_objectives=(
            _optic(1, "4x", mag=4.0, na=0.13, wd=16.4),
            _optic(2, "10x", mag=10.0, na=0.30, wd=4.0),
            _optic(3, "20x", mag=20.0, na=0.45, wd=1.0)),
    )
    base.update(over)
    return SimpleNamespace(**base)


class _CfgCase(unittest.TestCase):
    """A REAL MicroscopeConfigStore over a temp file, seeded with the rig's data."""

    def setUp(self):
        self._dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._dir, ignore_errors=True)
        self.cfg = MicroscopeConfigStore(Path(self._dir) / "microscope.json")
        self.cfg.set_filter_labels(dict(RIG_CUBES))
        self.cfg.set_objective_labels(dict(RIG_OBJECTIVES))
        self.filters = resolve_filters(scope_state=_state(), config_store=self.cfg)
        self.objectives = resolve_objectives(scope_state=_state(),
                                             config_store=self.cfg)


class TestResolveSlots(_CfgCase):
    def test_it_joins_labels_optics_and_presence(self):
        s = self.filters[2]
        self.assertEqual((s.position, s.name, s.native_name), (3, "TxRed", "TxRed"))
        self.assertTrue(s.present)
        self.assertTrue(s.usable)

    def test_empty_cassette_slots_are_not_usable(self):
        """The SDK reports an empty slot as a '-----' placeholder."""
        for s in self.filters[4:]:
            self.assertFalse(s.usable, f"slot {s.position} should be unusable")
            self.assertEqual(s.name, "")

    def test_objective_optics_carry_working_distance(self):
        wds = [s.working_distance_mm for s in self.objectives[:3]]
        self.assertEqual(wds, [16.4, 4.0, 1.0])

    def test_a_slot_the_operator_never_named_keeps_the_bodys_name(self):
        cfg2 = MicroscopeConfigStore(Path(self._dir) / "m2.json")
        cfg2.set_filter_labels({"1": "DAPI"})
        slots = resolve_filters(scope_state=_state(), config_store=cfg2)
        self.assertEqual(slots[1].name, "")
        self.assertEqual(slots[1].native_name, "FITC")
        self.assertEqual(slots[1].label, "FITC")

    def test_filter_wavelengths_are_joined_when_stored(self):
        self.cfg.set_filter_optics("FITC", excitation_nm=488.0, emission_nm=550.0)
        slots = resolve_filters(scope_state=_state(), config_store=self.cfg)
        self.assertEqual(slots[1].excitation_nm, 488.0)
        self.assertEqual(slots[1].emission_nm, 550.0)

    def test_a_body_that_answered_oddly_degrades_rather_than_raising(self):
        for bad in (SimpleNamespace(), SimpleNamespace(filter_count="nonsense"),
                    SimpleNamespace(filter_count=2, mounted_filters=None)):
            resolve_filters(scope_state=bad, config_store=self.cfg)

    def test_optic_at_walks_either_turret(self):
        st = _state()
        self.assertEqual(optic_at(st, 3, FILTER).label, "TxRed")
        self.assertEqual(optic_at(st, 3, OBJECTIVE).label, "20x")
        self.assertIsNone(optic_at(st, 9, FILTER))
        self.assertIsNone(optic_at(st, None, FILTER))


class TestMCherryDoesNotResolveToTxRed(_CfgCase):
    """A red cube is not the red cube you asked for."""

    def test_it_refuses(self):
        m = find_slot(self.filters, "mCherry")
        self.assertFalse(m.ok)
        self.assertIsNone(m.position)

    def test_the_refusal_names_what_IS_in_the_cassette(self):
        m = find_slot(self.filters, "mCherry")
        for expected in ("DAPI", "FITC", "TxRed", "Cy5"):
            self.assertIn(expected, m.why_not)

    def test_it_points_at_where_to_fix_it(self):
        self.assertIn("Hardware Setup", find_slot(self.filters, "mCherry").why_not)

    def test_an_explicit_operator_alias_resolves_it(self):
        self.cfg.set_optic_alias("filter", "mCherry", "TxRed")
        m = find_slot(self.filters, "mCherry",
                      aliases=self.cfg.optic_aliases("filter"))
        self.assertTrue(m.ok)
        self.assertEqual(m.position, 3)
        self.assertEqual(m.how, HOW_ALIAS)
        self.assertEqual(m.resolved_name, "TxRed",
                         "the resolved name must be what the CUBE is called")

    def test_bright_field_never_drives_the_empty_slot_5(self):
        """The legacy ordinal table said 'Bright Field -> 5'. Slot 5 is empty."""
        m = find_slot(self.filters, "Bright Field")
        self.assertFalse(m.ok)
        self.assertNotEqual(m.position, 5)

    def test_no_fuzzy_tier_exists(self):
        """Substring / prefix / near-miss must all refuse."""
        for wanted in ("DAP", "DAPI2", "FIT", "Cy", "Cy3", "TxRe", "xRed",
                       "dapi-long-pass"):
            self.assertFalse(find_slot(self.filters, wanted).ok,
                             f"{wanted!r} must not resolve")


class TestTheCaseFixStillWorks(_CfgCase):
    def test_lowercase_finds_the_uppercase_label(self):
        m = find_slot(self.objectives, "4x")
        self.assertTrue(m.ok)
        self.assertEqual(m.position, 1)
        self.assertEqual(m.how, HOW_NORMALIZED)

    def test_an_exact_label_reports_exact(self):
        m = find_slot(self.objectives, "4X")
        self.assertEqual((m.position, m.how), (1, HOW_EXACT))

    def test_surrounding_space_is_ignored(self):
        self.assertEqual(find_slot(self.filters, "  DAPI ").position, 1)

    def test_an_absent_objective_still_refuses(self):
        self.assertFalse(find_slot(self.objectives, "40x").ok)

    def test_the_bodys_own_name_resolves_an_unnamed_slot(self):
        cfg2 = MicroscopeConfigStore(Path(self._dir) / "m3.json")
        slots = resolve_filters(scope_state=_state(), config_store=cfg2)
        m = find_slot(slots, "TxRed")
        self.assertTrue(m.ok)
        self.assertEqual((m.position, m.how), (3, HOW_NATIVE))


class TestAmbiguityRefusesNamingBoth(_CfgCase):
    def test_two_slots_with_one_name_is_not_a_coin_toss(self):
        dup = self.filters + (OpticSlot(kind=FILTER, position=5, name="FITC",
                                        native_name="FITC", present=True),)
        m = find_slot(dup, "FITC")
        self.assertFalse(m.ok)
        self.assertIn("(2)", m.why_not)
        self.assertIn("(5)", m.why_not)
        self.assertIn("ambiguous", m.why_not)


class TestDegenerateRequests(_CfgCase):
    def test_an_empty_request_refuses(self):
        for wanted in ("", "   ", None):
            self.assertFalse(find_slot(self.filters, wanted).ok)

    def test_no_turret_refuses_without_pretending(self):
        m = find_slot((), "FITC", kind=FILTER)
        self.assertFalse(m.ok)
        self.assertIn("no filter", m.why_not)

    def test_an_empty_turret_refusal_names_the_turret_it_was_asked_about(self):
        """With no slots the kind cannot be inferred, so a caller states it —
        otherwise a question about the cassette is answered about the nosepiece."""
        self.assertIn("filter", find_slot((), "FITC", kind=FILTER).why_not)
        self.assertIn("objective", find_slot((), "4x", kind=OBJECTIVE).why_not)

    def test_a_named_but_unfitted_slot_says_nothing_is_fitted_there(self):
        slots = (OpticSlot(kind=FILTER, position=5, name="Bright Field",
                           present=False),)
        m = find_slot(slots, "Bright Field")
        self.assertFalse(m.ok)
        self.assertIn("nothing fitted", m.why_not)


class TestAliasStoreRefusals(_CfgCase):
    def test_it_refuses_a_target_that_is_not_a_named_slot(self):
        """An alias pointing at nothing reads as configured and fails later."""
        with self.assertRaises(ValueError) as cm:
            self.cfg.set_optic_alias("filter", "mPlum", "NoSuchCube")
        self.assertIn("TxRed", str(cm.exception))

    def test_it_refuses_an_alias_that_is_only_a_case_difference(self):
        with self.assertRaises(ValueError):
            self.cfg.set_optic_alias("filter", "fitc", "FITC")

    def test_it_refuses_an_unknown_turret(self):
        with self.assertRaises(ValueError):
            self.cfg.set_optic_alias("nosepiece", "a", "DAPI")

    def test_an_alias_survives_a_reload(self):
        self.cfg.set_optic_alias("filter", "mCherry", "TxRed")
        again = MicroscopeConfigStore(Path(self._dir) / "microscope.json")
        self.assertEqual(again.optic_aliases("filter"), {"mCherry": "TxRed"})

    def test_clearing_it_removes_it(self):
        self.cfg.set_optic_alias("filter", "mCherry", "TxRed")
        self.cfg.clear_optic_alias("filter", "mCherry")
        self.assertEqual(self.cfg.optic_aliases("filter"), {})

    def test_a_camera_with_no_aliases_stays_byte_identical(self):
        """optic_aliases must not appear in a file that never set one."""
        path = Path(self._dir) / "clean.json"
        MicroscopeConfigStore(path).save()
        self.assertNotIn('"optic_aliases": {\n', path.read_text(encoding="utf-8"))

    def test_a_malformed_alias_block_is_dropped_not_coerced(self):
        path = Path(self._dir) / "junk.json"
        path.write_text('{"optic_aliases": {"filter": {"a": "", "": "b"},'
                        ' "bogus": {"x": "y"}, "objective": "nope"}}',
                        encoding="utf-8")
        cfg = MicroscopeConfigStore(path)
        self.assertEqual(cfg.optic_aliases("filter"), {})
        self.assertEqual(cfg.optic_aliases("objective"), {})


class TestSnapshot(_CfgCase):
    class _Ctrl:
        def __init__(self, st, owner=None):
            self._st, self._owner = st, owner

        def state(self):
            return self._st

        def lease_owner(self):
            return self._owner

    def test_it_reports_both_turrets_and_the_lease(self):
        snap = snapshot(controller=self._Ctrl(_state(), "plate_level"),
                        config_store=self.cfg)
        self.assertTrue(snap.connected)
        self.assertFalse(snap.simulated)
        self.assertEqual(snap.lease_owner, "plate_level")
        self.assertEqual(snap.current(FILTER).label, "FITC")
        self.assertEqual(snap.current(OBJECTIVE).label, "4X")

    def test_a_simulated_backend_is_flagged_as_such(self):
        """A simulated switch reported as real would be a fabricated fact."""
        snap = snapshot(controller=self._Ctrl(_state(backend="simulated")),
                        config_store=self.cfg)
        self.assertTrue(snap.simulated)

    def test_an_unreadable_body_degrades_to_disconnected_with_a_reason(self):
        class _Boom:
            def state(self):
                raise RuntimeError("COM is gone")

        snap = snapshot(controller=_Boom(), config_store=self.cfg)
        self.assertFalse(snap.connected)
        self.assertIn("COM is gone", snap.error)

    def test_an_unknown_position_yields_no_current_slot(self):
        snap = snapshot(controller=self._Ctrl(_state(filter_position=None)),
                        config_store=self.cfg)
        self.assertIsNone(snap.current(FILTER))
        self.assertEqual(snap.current_name(FILTER), "")


if __name__ == "__main__":
    unittest.main()
