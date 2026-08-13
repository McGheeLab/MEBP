"""test_v79_target_types.py — user-defined target types + authored signature rules.

Pure: no Qt, no OpenCV, no image. The rules in this suite are AUTHORED metadata —
v7.9 decision D3 defers evaluation — so the tests that matter most are the ones
pinning what must NOT happen: an unfinished rule must match NOTHING (not
everything), ``evaluate()`` must refuse loudly rather than return a bool a caller
could read as "no match", a partial/hand-edited file must still open, and an
unknown key must be ignored so a file written by a future build still loads.
"""

from __future__ import annotations

import ast
import json
import os
import tempfile
import unittest
from pathlib import Path

from SupportClasses.FluorescenceMosaicStore import CHANNELS
from SupportClasses.TargetTypeStore import (
    COMBINATOR_ALL, COMBINATOR_ANY, METRIC_AUTO, METRIC_BACKGROUND_SIGMA,
    METRIC_INTENSITY, METRIC_PERCENTILE, RULE_EVALUATION_IMPLEMENTED,
    STATE_ANY, STATE_NEGATIVE, STATE_POSITIVE, SignatureClause, SignatureRule,
    TargetType, TargetTypeStore, get_store, safe_id, stamp_target_type,
    _DEFAULT_BUILTIN_DIR,
)

REPO = Path(__file__).resolve().parent.parent


def _single_positive(imaging_channel="FITC", other="mCherry") -> SignatureRule:
    """"Bright in ONE channel" — positive there, negative in the other."""
    return SignatureRule(combinator=COMBINATOR_ALL, clauses=[
        SignatureClause(imaging_channel=imaging_channel, state=STATE_POSITIVE),
        SignatureClause(imaging_channel=other, state=STATE_NEGATIVE),
    ])


def _double_positive() -> SignatureRule:
    """"Some brightness in BOTH channels" — the operator's first example."""
    return SignatureRule(combinator=COMBINATOR_ALL, clauses=[
        SignatureClause(imaging_channel="FITC", state=STATE_POSITIVE,
                        threshold=3.0, metric=METRIC_BACKGROUND_SIGMA),
        SignatureClause(imaging_channel="mCherry", state=STATE_POSITIVE,
                        threshold=3.0, metric=METRIC_BACKGROUND_SIGMA),
    ])


def _tt(id_="my-cells", **kw) -> TargetType:
    kw.setdefault("name", "My cells")
    kw.setdefault("signature_rule", _double_positive())
    return TargetType(id=id_, **kw)


class _IsolatedStore(unittest.TestCase):
    """Base: every store test runs against its own temp builtin/user pair."""

    def setUp(self):
        self.root = Path(tempfile.mkdtemp(prefix="mebp_target_types_"))
        self.builtin = self.root / "builtin"
        self.user = self.root / "user"
        self.builtin.mkdir(parents=True)
        self.user.mkdir(parents=True)

    def write(self, directory: Path, payload: dict, name=None) -> Path:
        path = directory / f"{name or payload.get('id', 'x')}.json"
        # Real bundled files carry ``"builtin": true``; an explicit false in a
        # file deliberately wins over the directory it was found in (same rule as
        # NeedleTypeStore / WellTypeStore / PlateTypeStore), so the fixture has
        # to stamp it or every "is it a builtin" assertion tests the wrong thing.
        if directory == self.builtin and "builtin" in payload:
            payload = {**payload, "builtin": True}
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return path

    def store(self) -> TargetTypeStore:
        return TargetTypeStore(builtin_dir=self.builtin, user_dir=self.user)


# ════════════════════════════════════════════════════════════════════
#  A. Clause normalization
# ════════════════════════════════════════════════════════════════════

class TestSignatureClause(unittest.TestCase):

    def test_defaults(self):
        c = SignatureClause()
        self.assertEqual(c.state, STATE_POSITIVE)
        self.assertIsNone(c.threshold)
        self.assertEqual(c.metric, METRIC_AUTO)
        self.assertFalse(c.is_gating)          # no channel named yet

    def test_unknown_state_falls_back_to_positive(self):
        self.assertEqual(SignatureClause(state="glowing").state, STATE_POSITIVE)

    def test_unknown_metric_falls_back_to_auto(self):
        c = SignatureClause(imaging_channel="FITC", metric="furlongs")
        self.assertEqual(c.metric, METRIC_AUTO)

    def test_bare_number_reads_as_absolute_intensity(self):
        # A threshold with no metric is the plainest reading: an absolute level,
        # matching SpheroidDetector._threshold(mode="fixed", abs_threshold=...).
        c = SignatureClause(imaging_channel="FITC", threshold=120)
        self.assertEqual(c.metric, METRIC_INTENSITY)
        self.assertEqual(c.threshold, 120.0)

    def test_metric_without_a_number_collapses_to_auto(self):
        c = SignatureClause(imaging_channel="FITC", metric=METRIC_PERCENTILE)
        self.assertEqual(c.metric, METRIC_AUTO)

    def test_unparseable_threshold_becomes_none(self):
        c = SignatureClause(imaging_channel="FITC", threshold="bright-ish")
        self.assertIsNone(c.threshold)
        self.assertEqual(c.metric, METRIC_AUTO)

    def test_any_state_never_gates(self):
        c = SignatureClause(imaging_channel="Cy5", state=STATE_ANY)
        self.assertFalse(c.is_gating)

    def test_round_trip_state_only_is_two_keys(self):
        c = SignatureClause(imaging_channel="FITC", state=STATE_NEGATIVE)
        d = c.to_dict()
        self.assertEqual(set(d), {"imaging_channel", "state"})
        self.assertEqual(SignatureClause.from_dict(d), c)

    def test_round_trip_with_threshold(self):
        c = SignatureClause(imaging_channel="Cy5", state=STATE_POSITIVE,
                            threshold=95, metric=METRIC_PERCENTILE)
        self.assertEqual(SignatureClause.from_dict(c.to_dict()), c)

    def test_from_dict_ignores_unknown_keys(self):
        base = {"imaging_channel": "FITC", "state": STATE_POSITIVE}
        self.assertEqual(SignatureClause.from_dict({**base, "roi_px": [1, 2]}),
                         SignatureClause.from_dict(base))

    def test_from_dict_tolerates_non_dict(self):
        self.assertEqual(SignatureClause.from_dict("FITC+"), SignatureClause())


# ════════════════════════════════════════════════════════════════════
#  B. Rule shape + describe()
# ════════════════════════════════════════════════════════════════════

class TestSignatureRule(unittest.TestCase):

    def test_empty_rule_matches_nothing_not_everything(self):
        # THE fail-safe invariant: a half-authored rule that selected every
        # object would, once an evaluator lands, remove the whole well.
        rule = SignatureRule()
        self.assertTrue(rule.is_empty())
        self.assertIn("nothing", rule.describe().lower())

    def test_any_only_clauses_are_still_empty(self):
        rule = SignatureRule(clauses=[
            SignatureClause(imaging_channel="DAPI", state=STATE_ANY)])
        self.assertTrue(rule.is_empty())

    def test_describe_single_positive(self):
        self.assertEqual(_single_positive().describe(), "FITC+ and mCherry−")

    def test_describe_double_positive(self):
        self.assertEqual(
            _double_positive().describe(),
            "FITC+ (≥ 3σ over background) and mCherry+ (≥ 3σ over background)")

    def test_describe_any_combinator_uses_or(self):
        rule = SignatureRule(combinator=COMBINATOR_ANY, clauses=[
            SignatureClause(imaging_channel="FITC", state=STATE_POSITIVE),
            SignatureClause(imaging_channel="Cy5", state=STATE_POSITIVE),
        ])
        self.assertEqual(rule.describe(), "FITC+ or Cy5+")

    def test_describe_negative_threshold_is_an_upper_bound(self):
        # The comparison direction follows the state — a "dim in mCherry" clause
        # is a ceiling, and rendering it as "≥" would read as its own opposite.
        rule = SignatureRule(clauses=[
            SignatureClause(imaging_channel="mCherry", state=STATE_NEGATIVE,
                            threshold=40, metric=METRIC_INTENSITY)])
        self.assertEqual(rule.describe(), "mCherry− (< 40)")

    def test_describe_percentile(self):
        rule = SignatureRule(clauses=[
            SignatureClause(imaging_channel="FITC", state=STATE_POSITIVE,
                            threshold=95, metric=METRIC_PERCENTILE)])
        self.assertEqual(rule.describe(), "FITC+ (≥ 95th percentile)")

    def test_describe_skips_non_gating_clauses(self):
        rule = SignatureRule(clauses=[
            SignatureClause(imaging_channel="FITC", state=STATE_POSITIVE),
            SignatureClause(imaging_channel="Cy5", state=STATE_ANY),
            SignatureClause(imaging_channel="", state=STATE_POSITIVE),
        ])
        self.assertEqual(rule.describe(), "FITC+")

    def test_unknown_combinator_falls_back_to_all(self):
        self.assertEqual(SignatureRule(combinator="xor").combinator,
                         COMBINATOR_ALL)

    def test_clauses_accept_dicts(self):
        rule = SignatureRule(clauses=[{"imaging_channel": "FITC",
                                       "state": STATE_POSITIVE}])
        self.assertEqual(len(rule.clauses), 1)
        self.assertIsInstance(rule.clauses[0], SignatureClause)

    def test_garbage_clause_entries_are_dropped_not_raised(self):
        rule = SignatureRule(clauses=["FITC+", None, 7,
                                      {"imaging_channel": "FITC"}])
        self.assertEqual(len(rule.clauses), 1)

    def test_clauses_not_a_list_degrades_to_empty(self):
        self.assertEqual(SignatureRule(clauses="FITC").clauses, [])

    def test_imaging_channels_are_deduped_in_order(self):
        rule = SignatureRule(clauses=[
            SignatureClause(imaging_channel="mCherry", state=STATE_POSITIVE),
            SignatureClause(imaging_channel="FITC", state=STATE_POSITIVE),
            SignatureClause(imaging_channel="mCherry", state=STATE_NEGATIVE),
        ])
        self.assertEqual(rule.imaging_channels(), ["mCherry", "FITC"])

    def test_builtin_rules_reference_only_known_channels(self):
        self.assertEqual(_double_positive().unknown_imaging_channels(CHANNELS), [])

    def test_unknown_channel_is_reported_not_repaired(self):
        rule = SignatureRule(clauses=[
            SignatureClause(imaging_channel="Texas Red", state=STATE_POSITIVE)])
        self.assertEqual(rule.unknown_imaging_channels(CHANNELS), ["Texas Red"])
        # Preserved: it may be a cube this rig just doesn't have mounted today.
        self.assertEqual(rule.imaging_channels(), ["Texas Red"])

    def test_round_trip(self):
        rule = _double_positive()
        self.assertEqual(SignatureRule.from_dict(rule.to_dict()), rule)

    def test_from_dict_ignores_unknown_keys(self):
        d = _single_positive().to_dict()
        self.assertEqual(SignatureRule.from_dict({**d, "morphology": "round"}),
                         SignatureRule.from_dict(d))

    def test_from_dict_tolerates_non_dict(self):
        self.assertTrue(SignatureRule.from_dict("FITC+").is_empty())


# ════════════════════════════════════════════════════════════════════
#  C. Evaluation is DEFERRED (v7.9 D3) — pinned so it cannot creep in
# ════════════════════════════════════════════════════════════════════

class TestEvaluationDeferred(unittest.TestCase):

    def test_module_flag_is_false(self):
        self.assertFalse(RULE_EVALUATION_IMPLEMENTED)

    def test_is_evaluatable_is_false_even_for_a_complete_rule(self):
        self.assertFalse(_double_positive().is_evaluatable())

    def test_evaluate_refuses_rather_than_returning_a_bool(self):
        # A False return would be indistinguishable from a genuine "no match"
        # and would silently skip every target the operator asked to remove.
        rule = _double_positive()
        with self.assertRaises(NotImplementedError) as ctx:
            rule.evaluate(object())
        msg = str(ctx.exception)
        self.assertIn("not implemented", msg.lower())
        self.assertIn("D3", msg)               # names the deferral
        self.assertIn("FITC+", msg)            # and states what was authored

    def test_no_image_libraries_are_imported(self):
        # An evaluator would need cv2/numpy; their absence is the structural
        # proof that this module only authors rules.
        src = (REPO / "SupportClasses" / "TargetTypeStore.py").read_text(
            encoding="utf-8")
        for node in ast.walk(ast.parse(src)):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for name in names:
                self.assertFalse(
                    name.split(".")[0] in ("cv2", "numpy", "np"),
                    f"TargetTypeStore must not import an imaging library: {name}")


# ════════════════════════════════════════════════════════════════════
#  D. TargetType + provenance stamping
# ════════════════════════════════════════════════════════════════════

class TestTargetType(unittest.TestCase):

    def test_round_trip_including_the_rule(self):
        tt = _tt(notes="double-positive cells")
        again = TargetType.from_dict(tt.to_dict())
        self.assertEqual(again, tt)
        self.assertEqual(again.signature_rule.describe(),
                         tt.signature_rule.describe())

    def test_json_round_trip(self):
        tt = _tt(id_="weird id/name", signature_rule=_single_positive())
        again = TargetType.from_dict(json.loads(json.dumps(tt.to_dict())))
        self.assertEqual(again, tt)

    def test_from_dict_ignores_unknown_keys(self):
        d = _tt().to_dict()
        self.assertEqual(TargetType.from_dict({**d, "future_field": 3}),
                         TargetType.from_dict(d))

    def test_partial_file_yields_an_empty_rule_not_an_exception(self):
        tt = TargetType.from_dict({"id": "half-done"})
        self.assertEqual(tt.id, "half-done")
        self.assertTrue(tt.signature_rule.is_empty())
        self.assertEqual(tt.label, "half-done")        # falls back to the id

    def test_rule_as_a_string_degrades_to_empty(self):
        tt = TargetType.from_dict({"id": "x", "signature_rule": "FITC+"})
        self.assertTrue(tt.signature_rule.is_empty())

    def test_from_dict_tolerates_non_dict(self):
        self.assertEqual(TargetType.from_dict(None).id, "")

    def test_bad_colour_degrades_to_the_default(self):
        # Cosmetic, so it must not refuse to load an authored type.
        self.assertEqual(_tt(color="chartreuse").color,
                         TargetType(id="x").color)

    def test_short_hex_colour_is_expanded(self):
        self.assertEqual(_tt(color="#ABC").color, "#aabbcc")

    def test_colour_without_hash_is_accepted(self):
        self.assertEqual(_tt(color="A6E3A1").color, "#a6e3a1")

    def test_describe_is_one_line(self):
        self.assertEqual(_tt(name="Doubles", signature_rule=_single_positive())
                         .describe(), "Doubles — FITC+ and mCherry−")

    def test_stamp_carries_the_resolved_name_and_colour(self):
        # STAMP, DON'T REFERENCE: a later rename must not relabel saved run data.
        tt = _tt(id_="dp", name="Double positive", color="#f9e2af")
        self.assertEqual(tt.stamp(), {
            "target_type_id": "dp",
            "target_type_name": "Double positive",
            "target_type_color": "#f9e2af",
        })

    def test_stamp_survives_a_rename_of_the_live_type(self):
        tt = _tt(id_="dp", name="Double positive")
        stamped = tt.stamp()
        tt.name = "Renamed after the run"
        self.assertEqual(stamped["target_type_name"], "Double positive")

    def test_stamp_target_type_none_is_empty(self):
        # An untyped target is legal: a single-bore run needs no classification.
        self.assertEqual(stamp_target_type(None), {})

    def test_stamp_target_type_duck_typed_stub(self):
        class Stub:
            id = "stub"
            name = "Stub cells"
            color = "#a6e3a1"
        self.assertEqual(stamp_target_type(Stub())["target_type_name"],
                         "Stub cells")

    def test_stamp_target_type_stub_without_an_id_is_empty(self):
        class Stub:
            pass
        self.assertEqual(stamp_target_type(Stub()), {})

    def test_stamp_target_type_always_returns_a_real_dict(self):
        # A MagicMock's ``.stamp()`` RETURNS a mock instead of raising, so the
        # try/except cannot catch it. Handing that back would make
        # ``record.update(...)`` silently add NOTHING (an unlabelled run — the
        # very thing stamping prevents) and then fail in json.dumps at save
        # time, far from the cause. It must be a dict, and serializable.
        from unittest.mock import MagicMock
        stamped = stamp_target_type(MagicMock())
        self.assertIsInstance(stamped, dict)
        json.dumps(stamped)                    # must not raise at save time
        record: dict = {}
        record.update(stamped)
        self.assertTrue(record, "provenance was silently dropped")


# ════════════════════════════════════════════════════════════════════
#  E. Store: builtin/user shadow, atomic write, graceful degradation
# ════════════════════════════════════════════════════════════════════

class TestStore(_IsolatedStore):

    def test_loads_builtins(self):
        self.write(self.builtin, _tt(id_="a").to_dict())
        self.write(self.builtin, _tt(id_="b").to_dict())
        store = self.store()
        self.assertEqual({t.id for t in store.all()}, {"a", "b"})
        self.assertTrue(all(t.builtin for t in store.all()))

    def test_user_shadows_builtin_by_id(self):
        self.write(self.builtin, _tt(id_="cells", name="Bundled").to_dict())
        self.write(self.user, _tt(id_="cells", name="Mine").to_dict())
        store = self.store()
        self.assertEqual(len(store.all()), 1)
        self.assertEqual(store.get("cells").name, "Mine")
        self.assertFalse(store.get("cells").builtin)

    def test_delete_user_resurfaces_the_builtin(self):
        self.write(self.builtin, _tt(id_="cells", name="Bundled").to_dict())
        store = self.store()
        store.save_user(_tt(id_="cells", name="Mine"))
        self.assertEqual(store.get("cells").name, "Mine")
        self.assertTrue(store.delete_user("cells"))
        self.assertEqual(store.get("cells").name, "Bundled")
        self.assertTrue(store.get("cells").builtin)

    def test_delete_user_on_a_builtin_only_id_is_a_no_op(self):
        self.write(self.builtin, _tt(id_="cells").to_dict())
        store = self.store()
        self.assertFalse(store.delete_user("cells"))
        self.assertIsNotNone(store.get("cells"))

    def test_save_user_persists_and_reloads(self):
        store = self.store()
        self.assertTrue(store.save_user(_tt(id_="dp", name="Doubles")))
        again = self.store()
        self.assertEqual(again.get("dp").name, "Doubles")
        self.assertEqual(again.get("dp").signature_rule.describe(),
                         _double_positive().describe())

    def test_save_user_forces_builtin_false(self):
        store = self.store()
        store.save_user(_tt(id_="dp", builtin=True))
        self.assertFalse(json.loads((self.user / "dp.json").read_text(
            encoding="utf-8"))["builtin"])

    def test_save_user_without_an_id_is_refused(self):
        store = self.store()
        self.assertFalse(store.save_user(TargetType(id="")))
        self.assertEqual(store.all(), [])

    def test_save_user_leaves_no_temp_file(self):
        # Atomic write = tmp + os.replace; a stray .tmp would also be globbed
        # back in as a duplicate target type on the next reload.
        store = self.store()
        store.save_user(_tt(id_="dp"))
        self.assertEqual([p.name for p in self.user.iterdir()], ["dp.json"])

    def test_save_user_replaces_rather_than_appends(self):
        store = self.store()
        store.save_user(_tt(id_="dp", name="First"))
        store.save_user(_tt(id_="dp", name="Second"))
        self.assertEqual(len(list(self.user.glob("*.json"))), 1)
        self.assertEqual(self.store().get("dp").name, "Second")

    def test_unsafe_id_is_filename_sanitized(self):
        store = self.store()
        store.save_user(_tt(id_="a/b:c*d"))
        self.assertTrue((self.user / f"{safe_id('a/b:c*d')}.json").exists())
        self.assertIsNotNone(self.store().get("a/b:c*d"))

    def test_get_unknown_and_none(self):
        store = self.store()
        self.assertIsNone(store.get("nope"))
        self.assertIsNone(store.get(None))

    def test_all_is_sorted_by_label(self):
        for i, name in enumerate(("Zeta", "alpha", "Mu")):
            self.write(self.builtin, _tt(id_=f"t{i}", name=name).to_dict())
        self.assertEqual([t.name for t in self.store().all()],
                         ["alpha", "Mu", "Zeta"])

    def test_missing_directories_are_not_an_error(self):
        store = TargetTypeStore(builtin_dir=self.root / "nope",
                                user_dir=self.root / "also-nope")
        self.assertEqual(store.all(), [])

    def test_stamp_for_resolves_once(self):
        store = self.store()
        store.save_user(_tt(id_="dp", name="Doubles", color="#f9e2af"))
        self.assertEqual(store.stamp_for("dp"), {
            "target_type_id": "dp",
            "target_type_name": "Doubles",
            "target_type_color": "#f9e2af",
        })

    def test_stamp_for_unknown_id_is_empty(self):
        self.assertEqual(self.store().stamp_for("ghost"), {})


class TestStoreDegradesGracefully(_IsolatedStore):

    def test_corrupt_json_is_skipped_not_raised(self):
        (self.builtin / "broken.json").write_text("{not json", encoding="utf-8")
        self.write(self.builtin, _tt(id_="good").to_dict())
        store = self.store()                      # must not raise
        self.assertEqual([t.id for t in store.all()], ["good"])

    def test_entry_without_an_id_is_skipped(self):
        self.write(self.builtin, {"name": "Nameless"}, name="nameless")
        self.write(self.builtin, _tt(id_="good").to_dict())
        self.assertEqual([t.id for t in self.store().all()], ["good"])

    def test_partial_entry_loads_with_an_empty_rule(self):
        self.write(self.builtin, {"id": "half"}, name="half")
        tt = self.store().get("half")
        self.assertIsNotNone(tt)
        self.assertTrue(tt.signature_rule.is_empty())

    def test_future_keys_do_not_block_loading(self):
        payload = _tt(id_="future").to_dict()
        payload["morphology_rule"] = {"circularity_min": 0.8}
        payload["signature_rule"]["clauses"][0]["roi_px"] = [0, 0, 10, 10]
        self.write(self.builtin, payload)
        tt = self.store().get("future")
        self.assertEqual(tt.signature_rule.describe(),
                         _double_positive().describe())

    def test_a_json_list_instead_of_an_object_is_skipped(self):
        (self.builtin / "list.json").write_text("[1, 2, 3]", encoding="utf-8")
        self.write(self.builtin, _tt(id_="good").to_dict())
        self.assertEqual([t.id for t in self.store().all()], ["good"])


# ════════════════════════════════════════════════════════════════════
#  F. Env override (test isolation) + bundled built-ins + GUI-free
# ════════════════════════════════════════════════════════════════════

class TestEnvOverride(_IsolatedStore):

    def test_env_dir_redirects_both_halves(self):
        self.write(self.builtin, _tt(id_="env-builtin").to_dict())
        self.write(self.user, _tt(id_="env-user").to_dict())
        prev = os.environ.get("MEBP_TARGET_TYPE_DIR")
        os.environ["MEBP_TARGET_TYPE_DIR"] = str(self.root)
        try:
            store = TargetTypeStore()             # no explicit dirs
            self.assertEqual({t.id for t in store.all()},
                             {"env-builtin", "env-user"})
            # And a write lands inside the isolated dir, never in config/hardware.
            store.save_user(_tt(id_="env-written"))
            self.assertTrue((self.user / "env-written.json").exists())
        finally:
            if prev is None:
                os.environ.pop("MEBP_TARGET_TYPE_DIR", None)
            else:
                os.environ["MEBP_TARGET_TYPE_DIR"] = prev

    def test_explicit_dirs_beat_the_env(self):
        self.write(self.builtin, _tt(id_="explicit").to_dict())
        prev = os.environ.get("MEBP_TARGET_TYPE_DIR")
        os.environ["MEBP_TARGET_TYPE_DIR"] = str(self.root / "elsewhere")
        try:
            store = self.store()
            self.assertEqual([t.id for t in store.all()], ["explicit"])
        finally:
            if prev is None:
                os.environ.pop("MEBP_TARGET_TYPE_DIR", None)
            else:
                os.environ["MEBP_TARGET_TYPE_DIR"] = prev



# v7.17.x: read the store's own resolved builtin dir (config/hardware/<machine
# id>/target_types/builtin or .../ME3B_general/target_types/builtin — see
# SupportClasses/MachineConfig.py) rather than a hardcoded legacy flat path,
# which the per-machine/shared config split moved out from under.
BUILTIN_DIR = _DEFAULT_BUILTIN_DIR


class TestBundledBuiltins(unittest.TestCase):
    """Assertions about the BUNDLED files only.

    These must read the builtin directory with the user half pointed at a path
    that does not exist — NOT the merged ``get_store()``. On any real operator
    machine the merged store also contains their saved user types, so asserting
    "every type is builtin / has a finished rule / names a known cube" over the
    merge fails the moment the operator uses the feature as intended (verified:
    one half-authored user type breaks two of these).
    """

    @classmethod
    def setUpClass(cls):
        cls.bundled = TargetTypeStore(builtin_dir=BUILTIN_DIR,
                                      user_dir=REPO / "does-not-exist")

    def test_bundled_builtins_load(self):
        types = self.bundled.all()
        self.assertGreaterEqual(len(types), 2)
        self.assertTrue(all(t.builtin for t in types))
        self.assertTrue(all(t.name for t in types))

    def test_get_store_finds_the_bundled_examples(self):
        # The singleton still resolves them (it reads the same builtin dir);
        # this is the only claim about the merged store that stays true in the
        # field, because a user type can shadow but never remove a builtin id.
        self.assertIsNotNone(get_store().get("fitc-single-positive"))

    def test_bundled_builtins_cover_the_operators_two_examples(self):
        store = self.bundled
        # "brightness in one channel"
        single = store.get("fitc-single-positive")
        self.assertIsNotNone(single)
        self.assertEqual(single.signature_rule.describe(), "FITC+ and mCherry−")
        # "some brightness in both channels"
        double = store.get("fitc-mcherry-double-positive")
        self.assertIsNotNone(double)
        self.assertEqual(len(double.signature_rule.gating_clauses), 2)
        self.assertTrue(all(c.state == STATE_POSITIVE
                            for c in double.signature_rule.gating_clauses))

    def test_bundled_builtins_reference_only_known_imaging_channels(self):
        for tt in self.bundled.all():
            self.assertEqual(
                tt.signature_rule.unknown_imaging_channels(CHANNELS), [],
                f"{tt.id} references an unknown imaging channel")

    def test_bundled_builtins_are_not_empty_rules(self):
        for tt in self.bundled.all():
            self.assertFalse(tt.signature_rule.is_empty(), tt.id)

    def test_bundled_builtins_round_trip_byte_identically(self):
        # A re-save must not rewrite a bundled example (the file on disk is the
        # documentation of the format).
        for path in sorted(BUILTIN_DIR.glob("*.json")):
            raw = json.loads(path.read_text(encoding="utf-8"))
            payload = {k: v for k, v in raw.items() if not k.startswith("_")}
            again = TargetType.from_dict(payload).to_dict()
            self.assertEqual(json.dumps(again, sort_keys=True),
                             json.dumps(payload, sort_keys=True), path.name)


class TestGuiFree(unittest.TestCase):

    def test_store_module_has_no_gui_dependency(self):
        src = (REPO / "SupportClasses" / "TargetTypeStore.py").read_text(
            encoding="utf-8")
        for node in ast.walk(ast.parse(src)):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for name in names:
                self.assertFalse(name.startswith(("PySide6", "gui.")),
                                 f"TargetTypeStore must stay GUI-free: {name}")

    def test_no_hardware_config_coupling(self):
        # Per-machine data: a setup file copied from another rig must never
        # carry another lab's target definitions (CAMERA_CAL_PERSIST_STORE).
        src = (REPO / "SupportClasses" / "TargetTypeStore.py").read_text(
            encoding="utf-8")
        self.assertNotIn("HardwareConfig", src.split('"""', 2)[-1])

    def test_no_banned_bare_channel_identifier(self):
        # v7.9 vocabulary: the bare word "channel" is banned in new identifiers
        # (it means five different things in this repo) — use imaging_channel.
        # This suite is scanned TOO: a rule that exempts the file asserting it
        # is not a rule, and the first violation was in fact a helper kwarg here.
        offenders: list[str] = []
        for rel in ("SupportClasses/TargetTypeStore.py",
                    "tests/test_v79_target_types.py"):
            tree = ast.parse((REPO / rel).read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    for arg in list(node.args.args) + list(node.args.kwonlyargs):
                        if arg.arg == "channel":
                            offenders.append(f"{rel}: {node.name}(channel=)")
                elif isinstance(node, ast.AnnAssign) and isinstance(
                        node.target, ast.Name):
                    if node.target.id == "channel":
                        offenders.append(f"{rel}: field 'channel'")
                elif isinstance(node, ast.keyword) and node.arg == "channel":
                    offenders.append(f"{rel}: passed channel=")
        self.assertEqual(offenders, [])

    def test_no_banned_bare_channel_in_a_displayed_string(self):
        # The ban covers UI strings and log lines, not just identifiers:
        # ``describe()`` is rendered in a readout AND logged by ``save_user``.
        clause = SignatureClause(imaging_channel="", state=STATE_POSITIVE)
        self.assertIn("imaging channel", clause.describe())


if __name__ == "__main__":
    unittest.main()
