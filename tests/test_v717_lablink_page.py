"""
test_v717_lablink_page.py — the LabLink workflow page, the Qt bridge, and the
filter-cube wavelength fields on Hardware Setup → Microscope.

Run offscreen. Nothing here opens a socket: the page's only network path is
``_HubProbe``, and every test drives ``_on_probe_finished`` with a document
shaped like a real ``GET /workflows`` response instead.

The knob tests carry most of the weight. A knob map has three states —
omitted / null / a value — and two of the three are falsy, so the failure mode
is not a crash but a job that runs with a different number than the operator
chose and reports success. `test_pinned_zero_survives_a_round_trip` is the one
that catches it.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QComboBox, QSpinBox  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ── A hub document shaped like the real thing ─────────────────────
#
# Field names taken from `lablink/recipes.py::knob_wire` / `recipe_wire` /
# `workflows_doc` rather than invented, so a test passing here means the page
# parses what the hub actually publishes.

def _knob(name, **kw):
    knob = {"name": name, "label": kw.pop("label", name), "help": "",
            "type": "float", "unit": None, "kind": "param", "min": None,
            "max": None, "enum": None, "default": None, "unset_means": None,
            "max_items": None, "applies_when": None}
    knob.update(kw)
    return knob


def _hub_workflows():
    return [{
        "name": "nd2studios",
        "title": "ND2Studios",
        "recipes": [{
            "name": "cell-segmentation",
            "title": "Cell segmentation and measurement",
            "requires_metadata": ["pixel_size_um", "objective_na"],
            "knobs": [
                _knob("channel", type="channel_list", max_items=1, min=0,
                      max=8),
                _knob("dim", type="enum", enum=["2D", "3D"], default="2D"),
                _knob("background_um", type="float", unit="um", min=0.1,
                      max=200.0, default=5.0),
                _knob("min_area_um2", type="float", min=0.0, max=1e5,
                      default=10.0,
                      applies_when={"knob": "dim", "equals": "2D"}),
                _knob("min_volume_um3", type="float", min=0.0, max=1e6,
                      default=20.0,
                      applies_when={"knob": "dim", "equals": "3D"}),
                _knob("label", type="string", max_len=32),
                _knob("invert", type="bool", default=False),
            ],
        }],
        "unusable_recipes": [],
    }]


def _hello(**caps):
    base = {"sessions": True, "longpoll_max_s": 25,
            "recipe_metadata_requirements": True}
    base.update(caps)
    return {"protocol": 1, "max_file_bytes": 512 * 1024 * 1024,
            "capabilities": base}


class _PageCase(unittest.TestCase):
    """Each test gets its own config dir, so none of them see the real rig."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("MEBP_LABLINK_CONFIG_DIR")
        os.environ["MEBP_LABLINK_CONFIG_DIR"] = self._tmp.name
        from SupportClasses import LabLinkConfigStore as cfg
        cfg.reset_store()
        self.store = cfg.get_store()
        from gui.pages.workflows.lablink_workflow import LabLinkWorkflowPage
        self.page = LabLinkWorkflowPage(config=self.store)

    def tearDown(self):
        self.page.deleteLater()
        _app.processEvents()
        from SupportClasses import LabLinkConfigStore as cfg
        cfg.reset_store()
        if self._prev is None:
            os.environ.pop("MEBP_LABLINK_CONFIG_DIR", None)
        else:
            os.environ["MEBP_LABLINK_CONFIG_DIR"] = self._prev
        self._tmp.cleanup()

    # helpers
    def _configure(self):
        self.store.set_base_url("http://hub.example:8765")
        self.store.set_token("secret-token")
        self.page.load()

    def _discover(self, hello=None, flows=None):
        self.page._on_probe_finished(
            True, "", hello if hello is not None else _hello(),
            flows if flows is not None else _hub_workflows())

    def _panel(self, name="fluorescence_well"):
        return self.page._panels[name]

    def _choose(self, name="fluorescence_well", workflow="nd2studios",
                recipe="cell-segmentation"):
        """Pick a workflow + recipe the way the operator has to.

        Discovery deliberately does not choose for them, so every knob test has
        to do this first.
        """
        panel = self._panel(name)
        panel.workflow.setCurrentIndex(panel.workflow.findData(workflow))
        panel.recipe.setCurrentIndex(panel.recipe.findData(recipe))
        return panel

    def _rows(self, panel):
        return {r.name: r for r in panel._knob_rows}

    def _pin(self, row, value=None):
        if value is not None:
            row._set_value(value)
        row.mode.setCurrentIndex(row.mode.findData("pinned"))

    def _derive(self, row):
        row.mode.setCurrentIndex(row.mode.findData("derive"))


class TestPageBuilds(_PageCase):

    def test_builds_with_all_four_sources(self):
        from SupportClasses.LabLinkConfigStore import SOURCES
        self.assertEqual(set(self.page._panels), set(SOURCES))

    def test_no_jog_context_widget(self):
        """The page commands no motion, so it must not claim the left box."""
        self.assertIsNone(self.page.get_context_widget())

    def test_status_says_not_configured_before_a_url(self):
        self.assertIn("Not configured", self.page._status.text())

    def test_status_says_off_when_configured_but_disabled(self):
        self._configure()
        self.assertEqual(self.page._status.text(), "Off")

    def test_status_flags_on_with_no_armed_source(self):
        self._configure()
        self.store.set_enabled(True)
        self.page.refresh_status()
        self.assertIn("no source is armed", self.page._status.text())

    def test_page_title(self):
        self.assertEqual(self.page.get_page_title(), "LabLink Processing")


class TestTokenHandling(_PageCase):

    def test_token_field_is_masked(self):
        from PySide6.QtWidgets import QLineEdit
        self.assertEqual(self.page._token_edit.echoMode(), QLineEdit.Password)

    def test_environment_token_disables_the_field_and_says_so(self):
        os.environ["LABLINK_TOKEN"] = "from-env"
        try:
            self.page.load()
            self.assertFalse(self.page._token_edit.isEnabled())
            self.assertIn("LABLINK_TOKEN", self.page._token_note.text())
            # And it must not be echoed into the widget.
            self.assertEqual(self.page._token_edit.text(), "")
        finally:
            os.environ.pop("LABLINK_TOKEN", None)

    def test_no_token_is_named_as_such(self):
        self.assertIn("No token set", self.page._token_note.text())


class TestDiscovery(_PageCase):

    def test_refuses_to_probe_without_a_url_and_token(self):
        self.page.discover()
        self.assertIn("address and a token", self.page._hub_note.text())

    def test_success_populates_workflows_and_recipes(self):
        self._configure()
        self._discover()
        panel = self._panel()
        self.assertGreaterEqual(panel.workflow.findData("nd2studios"), 0)
        # Plus the deliberate empty choice at the top, and nothing else.
        self.assertEqual(panel.workflow.count(), 2)
        # Recipes belong to a workflow, so they appear once one is chosen.
        self.assertEqual(panel.recipe.count(), 1)
        panel.workflow.setCurrentIndex(panel.workflow.findData("nd2studios"))
        self.assertGreaterEqual(panel.recipe.findData("cell-segmentation"), 0)
        self.assertEqual(panel.recipe.count(), 2)

    def test_hub_note_quotes_the_hubs_own_limits(self):
        self._configure()
        self._discover()
        note = self.page._hub_note.text()
        self.assertIn("512 MB", note)       # from max_file_bytes
        self.assertIn("25", note)           # from longpoll_max_s
        self.assertIn("1 recipe", note)

    def test_missing_metadata_capability_fails_closed(self):
        """Absent means "cannot tell you", never "requires nothing".

        Reading it the other way fails OPEN against an older hub: the run
        finishes with different numbers and no warning anywhere.
        """
        self._configure()
        self._discover(hello=_hello(recipe_metadata_requirements=False))
        self.assertIn("unknown, not none", self.page._hub_note.text())

    def test_a_file_exchange_is_reported_not_used(self):
        """`capabilities.sessions` unset = no analyses here."""
        self.page._on_probe_finished(
            False,
            "This hub answered, but it does not run analyses "
            "(capabilities.sessions is not set) — it is a file exchange only.",
            _hello(sessions=False), None)
        self.assertIn("does not run analyses", self.page._hub_note.text())
        self.assertEqual(self.page.discovered, [])

    def test_unusable_recipes_are_surfaced(self):
        flows = _hub_workflows()
        flows[0]["unusable_recipes"] = [
            {"name": "broken", "problem": "knob 'x' declares unit 'nm'"}]
        self._configure()
        self._discover(flows=flows)
        self.assertIn("broken", self.page._hub_note.text())
        self.assertIn("unit 'nm'", self.page._hub_note.text())

    def test_a_stored_recipe_stays_visible_before_discovery(self):
        """A configured choice must not vanish just because the hub has not
        been asked yet — that reads as "nothing was ever configured"."""
        self.store.set_source("still", {"workflow": "nd2studios",
                                        "recipe": "cell-segmentation"})
        self.page.load()
        panel = self._panel("still")
        self.assertEqual(panel.recipe.currentData(), "cell-segmentation")

    def test_probe_failure_leaves_no_stale_recipe_list(self):
        self._configure()
        self._discover()
        self.assertTrue(self.page.discovered)
        self.page._on_probe_finished(False, "Connection refused", None, None)
        self.assertEqual(self.page.discovered, [])
        self.assertIn("Connection refused", self.page._hub_note.text())

    def test_discovery_is_never_written_to_disk(self):
        """*"Cache them for the length of a user's session if you like; do not
        ship them."* A recipe rename on the hub must not be shadowed by a
        stored copy."""
        self._configure()
        self._discover()
        raw = json.loads(Path(self.store.path).read_text(encoding="utf-8"))
        blob = json.dumps(raw)
        self.assertNotIn("knobs\": [", blob.replace(" ", ""))
        self.assertNotIn("cell segmentation and measurement", blob.lower())
        self.assertNotIn("unusable", blob)


class TestKnobTriState(_PageCase):

    def setUp(self):
        super().setUp()
        self._configure()
        self._discover()
        self.panel = self._choose()
        self.panel.enable.setChecked(True)
        self.rows = self._rows(self.panel)

    def test_every_published_knob_gets_a_row(self):
        self.assertEqual(len(self.panel._knob_rows), 7)

    def test_default_mode_omits_the_key_entirely(self):
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertEqual(stored, {})

    def test_derive_stores_an_explicit_null(self):
        self._derive(self.rows["background_um"])
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertIn("background_um", stored)
        self.assertIsNone(stored["background_um"])

    def test_pinned_value_is_stored(self):
        self._pin(self.rows["background_um"], 7.5)
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertAlmostEqual(stored["background_um"], 7.5, places=3)

    def test_pinned_channel_zero_survives_a_round_trip(self):
        """`channel` is a zero-based index, so 0 is both legal and likely.

        ⚠ This one is NOT the falsy case. A `channel_list` is stored as `[0]`,
        a non-empty list, which is truthy — a mutation replacing the membership
        test with `.get()` sails straight past it. See
        `test_a_pinned_scalar_zero_survives_a_round_trip`, which is the test
        that actually holds the line.
        """
        self._pin(self.rows["channel"], 0)
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertEqual(stored["channel"], [0])

        self.panel.load()
        row = self._rows(self.panel)["channel"]
        self.assertEqual(row.mode.currentData(), "pinned")
        self.assertEqual(row.value.value(), 0)

    def test_a_pinned_scalar_zero_survives_a_round_trip(self):
        """The test this class exists for.

        Omitted / null / 0 are three different instructions to the hub and two
        of them are falsy, so membership is the only test that separates them.
        Reading the stored map with `.get()` collapses a pinned zero into
        "recipe default" — the job runs with the recipe's own number instead of
        the operator's, and reports success either way.

        ⚠ Written after a mutation slipped past the `channel` test above: `[0]`
        is truthy, so only a *scalar* zero exercises the rule.
        """
        row = self.rows["min_area_um2"]      # min 0.0, so 0 is in range
        self._pin(row, 0.0)
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertIn("min_area_um2", stored)
        self.assertEqual(stored["min_area_um2"], 0.0)

        self.panel.load()
        reloaded = self._rows(self.panel)["min_area_um2"]
        self.assertEqual(reloaded.mode.currentData(), "pinned")
        self.assertEqual(reloaded.value.value(), 0.0)

    def test_a_pinned_false_survives_a_round_trip(self):
        """The other falsy value. `False` is a real instruction, not an absence."""
        row = self.rows["invert"]
        row.value.setCurrentIndex(row.value.findData(False))
        self._pin(row)
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertIn("invert", stored)
        self.assertIs(stored["invert"], False)

        self.panel.load()
        reloaded = self._rows(self.panel)["invert"]
        self.assertEqual(reloaded.mode.currentData(), "pinned")
        self.assertIs(reloaded.value.currentData(), False)

    def test_a_stored_null_reloads_as_derive_not_default(self):
        self.store.set_source("fluorescence_well",
                              {"knobs": {"background_um": None}})
        self.panel.load()
        row = self._rows(self.panel)["background_um"]
        self.assertEqual(row.mode.currentData(), "derive")

    def test_the_value_widget_is_only_live_when_pinned(self):
        row = self.rows["background_um"]
        self.assertFalse(row.value.isEnabled())
        self._pin(row, 3.0)
        self.assertTrue(row.value.isEnabled())
        self._derive(row)
        self.assertFalse(row.value.isEnabled())

    def test_bounds_come_from_the_hub(self):
        row = self.rows["background_um"]
        self.assertAlmostEqual(row.value.minimum(), 0.1, places=3)
        self.assertAlmostEqual(row.value.maximum(), 200.0, places=3)

    def test_enum_renders_as_a_combo_of_the_published_choices(self):
        row = self.rows["dim"]
        self.assertIsInstance(row.value, QComboBox)
        self.assertEqual(
            [row.value.itemText(i) for i in range(row.value.count())],
            ["2D", "3D"])

    def test_channel_list_is_sent_as_a_list(self):
        self._pin(self.rows["channel"], 2)
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertEqual(stored["channel"], [2])

    def test_string_knob_respects_max_len(self):
        self.assertEqual(self.rows["label"].value.maxLength(), 32)

    def test_bool_knob_stores_a_real_bool(self):
        row = self.rows["invert"]
        row.value.setCurrentIndex(row.value.findData(True))
        self._pin(row)
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertIs(stored["invert"], True)

    def test_help_and_bounds_reach_the_tooltip(self):
        tip = self.rows["background_um"].mode.toolTip()
        self.assertIn("0.1", tip)
        self.assertIn("200", tip)


class TestAppliesWhen(_PageCase):

    def setUp(self):
        super().setUp()
        self._configure()
        self._discover()
        self.panel = self._choose()
        self.rows = self._rows(self.panel)

    def test_condition_met_leaves_the_row_live(self):
        # dim defaults to 2D, so the 2D-only knob applies.
        self.assertTrue(self.rows["min_area_um2"].mode.isEnabled())

    def test_condition_unmet_disables_and_explains(self):
        """*"Read applies_when and disable the control rather than letting
        someone set it"* — the knob is refused, not ignored, so an editable
        control manufactures a 400."""
        row = self.rows["min_volume_um3"]
        self.assertFalse(row.mode.isEnabled())
        self.assertIn("not used", row.note.text())
        self.assertIn("3D", row.note.text())

    def test_changing_the_controlling_knob_flips_which_rows_apply(self):
        dim = self.rows["dim"]
        dim.value.setCurrentIndex(dim.value.findData("3D"))
        self._pin(dim)
        self.assertFalse(self.rows["min_area_um2"].mode.isEnabled())
        self.assertTrue(self.rows["min_volume_um3"].mode.isEnabled())

    def test_an_inapplicable_knob_is_not_sent(self):
        row = self.rows["min_volume_um3"]
        self._pin(row, 42.0)            # the operator cannot, but force it
        stored = self.store.source("fluorescence_well")["knobs"]
        self.assertNotIn("min_volume_um3", stored)

    def test_in_form_conditions_are_understood(self):
        from gui.pages.workflows.lablink_workflow import (
            _condition_holds, _condition_text,
        )
        cond = {"knob": "dim", "in": ["2D", "2.5D"]}
        self.assertTrue(_condition_holds(cond, {"dim": "2.5D"}))
        self.assertFalse(_condition_holds(cond, {"dim": "3D"}))
        self.assertIn("one of", _condition_text(cond))


class TestSourceCommit(_PageCase):

    def test_arming_a_source_needs_a_recipe_to_count(self):
        """A source armed with no recipe would queue work that can never run,
        and the operator would watch a queue grow instead of being asked for a
        recipe."""
        self._configure()
        self.store.set_enabled(True)
        panel = self._panel()
        panel.enable.setChecked(True)
        self.assertFalse(self.store.source_enabled("fluorescence_well"))
        self._discover()
        # Discovery alone must NOT arm it — the operator still chooses.
        self.assertFalse(self.store.source_enabled("fluorescence_well"))
        panel = self._panel()
        panel.workflow.setCurrentIndex(panel.workflow.findData("nd2studios"))
        panel.recipe.setCurrentIndex(
            panel.recipe.findData("cell-segmentation"))
        self.assertTrue(self.store.source_enabled("fluorescence_well"))

    def test_discovery_never_picks_a_recipe_for_the_operator(self):
        """⚠ The regression this class caught.

        Auto-selecting the first discovered recipe left the combo *showing* an
        analysis while the store held "" — so a source read as armed with a
        recipe named and silently sent nothing. It also decides which analysis
        runs on the data; a deconvolution is not a segmentation.
        """
        self._configure()
        self._discover()
        panel = self._panel()
        self.assertEqual(panel.recipe.currentData(), "")
        self.assertEqual(self.store.source("fluorescence_well")["recipe"], "")
        # What it shows and what is stored must agree.
        self.assertIn("choose", panel.recipe.currentText().lower())

    def test_an_armed_source_with_no_recipe_says_why(self):
        self._configure()
        self.store.set_enabled(True)
        panel = self._panel()
        panel.enable.setChecked(True)
        self.assertTrue(panel.gate.isVisible() or panel.gate.text())
        self.assertIn("no recipe", panel.gate.text().lower())

    def test_a_recipe_the_hub_no_longer_offers_is_shown_as_such(self):
        self.store.set_source("still", {"workflow": "nd2studios",
                                        "recipe": "retired-recipe"})
        self._configure()
        self._discover()
        panel = self._panel("still")
        self.assertEqual(panel.recipe.currentData(), "retired-recipe")
        self.assertIn("not offered", panel.recipe.currentText())

    def test_ceiling_is_persisted_in_mb(self):
        self._configure()
        self._panel("video").ceiling.setValue(64)
        self.assertEqual(self.store.source("video")["max_upload_mb"], 64)
        self.assertEqual(self.store.max_upload_bytes("video"), 64 * 1024 * 1024)

    def test_zero_ceiling_means_no_local_limit(self):
        self._configure()
        self._panel("video").ceiling.setValue(0)
        self.assertEqual(self.store.max_upload_bytes("video"), 0)

    def test_sources_are_independent(self):
        self._configure()
        self._discover()
        self._panel("still").enable.setChecked(True)
        self.assertTrue(self.store.source("still")["enabled"])
        self.assertFalse(self.store.source("video")["enabled"])

    def test_changing_the_workflow_clears_stale_knobs(self):
        """Knob names belong to a recipe. Carrying them across a recipe change
        would send a knob the new recipe never declared, which is refused."""
        self._configure()
        self._discover()
        panel = self._choose()
        rows = self._rows(panel)
        self._pin(rows["background_um"], 9.0)
        self.assertIn("background_um",
                      self.store.source("fluorescence_well")["knobs"])
        panel._on_workflow_changed()
        self.assertEqual(self.store.source("fluorescence_well")["knobs"], {})


class TestQueueView(_PageCase):

    def _service(self, jobs, **extra):
        class _Fake:
            def __init__(self):
                self.snap = {"jobs": jobs, "counts": {}, "dropped_full": 0,
                             "last_error": "", "running": True, "pending": 0}
                self.snap.update(extra)

            def snapshot(self, limit=50):
                return self.snap

            def add_listener(self, cb):
                pass

            def remove_listener(self, cb):
                pass
        return _Fake()

    def test_empty_queue_says_so(self):
        self.page.refresh_queue()
        self.assertIn("Nothing sent", self.page._queue_summary.text())

    def test_a_failure_shows_the_hubs_own_words(self):
        """Unknown error codes must be shown by MESSAGE, never dropped through
        a switch that only knows the codes we happened to think of."""
        self.page._service = self._service(
            [{"job_id": "j1", "state": "failed",
              "detail": "unsupported_input: no recipe matches '.nd3'",
              "spec": {"source": "still", "path": "x.nd3"},
              "recipe": "cell-segmentation"}],
            counts={"failed": 1})
        self.page.refresh_queue()
        texts = self._queue_texts()
        self.assertTrue(any("unsupported_input" in t for t in texts))
        self.assertTrue(any(".nd3" in t for t in texts))

    def test_dropped_jobs_are_stated_not_hidden(self):
        self.page._service = self._service([], dropped_full=3)
        self.page.refresh_queue()
        self.assertIn("3 dropped", self.page._queue_summary.text())
        self.assertIn("NOT sent", self.page._queue_summary.text())

    def test_missing_sidecar_fields_are_named(self):
        self.page._service = self._service(
            [{"job_id": "j1", "state": "running", "detail": "",
              "spec": {"source": "fluorescence_well", "well": "C5"},
              "missing_metadata": ["objective_na", "emission_nm"]}],
            counts={"running": 1})
        self.page.refresh_queue()
        joined = " ".join(self._queue_texts())
        self.assertIn("objective_na", joined)
        self.assertIn("emission_nm", joined)

    def _queue_texts(self):
        from PySide6.QtWidgets import QLabel
        out = []
        for i in range(self.page._queue_lay.count()):
            w = self.page._queue_lay.itemAt(i).widget()
            if isinstance(w, QLabel):
                out.append(w.text())
        return out


class TestFormatBanner(_PageCase):
    """What is sent, and why the sidecar decides.

    ⚠ Do NOT re-add a "the hub cannot read .nd3" warning here. It was written
    on a misreading — the recipe's top-level `match` is null, but the field that
    governs acceptance is `inputs[].match`, and every `nd2studios` recipe lists
    `*.nd3` there. A permanent warning about a working format trains the
    operator to ignore the banner.
    """

    def _blob(self):
        from PySide6.QtWidgets import QLabel
        return " ".join(w.text() for w in self.page.findChildren(QLabel))

    def test_the_page_states_what_is_sent(self):
        blob = self._blob()
        self.assertIn(".nd3", blob)
        self.assertIn("job.json", blob)

    def test_it_says_the_sidecar_overrides_the_image(self):
        """The one rule a reader of the format doc must not get backwards."""
        self.assertIn("authoritative", self._blob().lower())

    def test_the_metadata_stake_is_quantified_not_hand_waved(self):
        """A number the operator can weigh beats "metadata is important"."""
        blob = self._blob()
        self.assertIn("2855", blob)
        self.assertIn("2660", blob)

    def test_no_standing_warning_while_the_format_is_accepted(self):
        from SupportClasses.LabLinkJob import ND3_NOT_YET_ACCEPTED
        if ND3_NOT_YET_ACCEPTED:
            self.skipTest("pointed at a hub too old for .nd3")
        self.assertNotIn("cannot read", self._blob().lower())


class TestBridge(unittest.TestCase):
    """The service notifies from its worker thread; widgets may only be touched
    on the GUI thread."""

    class _FakeService:
        def __init__(self):
            self.listeners = []

        def add_listener(self, cb):
            self.listeners.append(cb)

        def remove_listener(self, cb):
            if cb in self.listeners:
                self.listeners.remove(cb)

        def fire(self):
            for cb in list(self.listeners):
                cb(self)

    def test_subscribes_and_re_emits(self):
        from gui.widgets.lablink_bridge import LabLinkBridge
        svc = self._FakeService()
        bridge = LabLinkBridge(svc)
        seen = []
        bridge.changed.connect(lambda: seen.append(1))
        self.assertEqual(len(svc.listeners), 1)
        svc.fire()
        _app.processEvents()
        self.assertEqual(len(seen), 1)

    def test_the_signal_carries_no_payload(self):
        """It must not hand the worker's live JobRecord across a thread. The
        page re-reads snapshot(), which takes the lock and deep-copies."""
        from gui.widgets.lablink_bridge import LabLinkBridge
        import inspect
        src = inspect.getsource(LabLinkBridge)
        self.assertIn("changed = Signal()", src)

    def test_unsubscribe_stops_delivery(self):
        from gui.widgets.lablink_bridge import LabLinkBridge
        svc = self._FakeService()
        bridge = LabLinkBridge(svc)
        seen = []
        bridge.changed.connect(lambda: seen.append(1))
        bridge.unsubscribe()
        self.assertEqual(svc.listeners, [])
        svc.fire()
        _app.processEvents()
        self.assertEqual(seen, [])

    def test_double_subscribe_is_idempotent(self):
        from gui.widgets.lablink_bridge import LabLinkBridge
        svc = self._FakeService()
        bridge = LabLinkBridge(svc)
        bridge.subscribe()
        self.assertEqual(len(svc.listeners), 1)

    def test_a_dead_qobject_does_not_raise_into_the_worker(self):
        """The service is a process-wide singleton that outlives every page, so
        a notify can land after the C++ object is gone. Raising there would
        take down the upload worker."""
        from gui.widgets.lablink_bridge import LabLinkBridge
        svc = self._FakeService()
        bridge = LabLinkBridge(svc)
        import shiboken6
        shiboken6.delete(bridge)
        svc.fire()          # must not raise


class TestFilterWavelengths(unittest.TestCase):
    """Emission / excitation on Hardware Setup → Microscope.

    MEBP records these nowhere else, and a deconvolution recipe needs them.
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("MEBP_MICROSCOPE_CONFIG_DIR")
        os.environ["MEBP_MICROSCOPE_CONFIG_DIR"] = self._tmp.name
        from SupportClasses import MicroscopeConfigStore as mcs
        if hasattr(mcs, "reset_store"):
            mcs.reset_store()

        # ⚠ Patched for the WHOLE class, not inside the one test that expects a
        # refusal. `commit()` has several message-box paths, and a real modal
        # under offscreen Qt blocks forever — a hanging test gets disabled
        # rather than fixed. Found the hard way: a mutation moved the refusal to
        # a different test and the run stopped producing output at all.
        from PySide6.QtWidgets import QMessageBox
        self._warnings: list = []
        self._real_warning = QMessageBox.warning
        QMessageBox.warning = staticmethod(
            lambda *a, **k: self._warnings.append(a[2] if len(a) > 2 else ""))

        from gui.pages.hardware.microscope_setup_panel import MicroscopeSetupPanel
        self.panel = MicroscopeSetupPanel()
        self.panel.load()
        self.table = self.panel._filter_table

    def tearDown(self):
        from PySide6.QtWidgets import QMessageBox
        QMessageBox.warning = self._real_warning
        self.panel.deleteLater()
        _app.processEvents()
        from SupportClasses import MicroscopeConfigStore as mcs
        if hasattr(mcs, "reset_store"):
            mcs.reset_store()
        if self._prev is None:
            os.environ.pop("MEBP_MICROSCOPE_CONFIG_DIR", None)
        else:
            os.environ["MEBP_MICROSCOPE_CONFIG_DIR"] = self._prev
        self._tmp.cleanup()

    def _set(self, pos, name, em=0, ex=0):
        self.table._rows[pos]["edit"].setText(name)
        self.table._rows[pos]["em"].setValue(em)
        self.table._rows[pos]["ex"].setValue(ex)

    def test_round_trips_through_the_store(self):
        self._set(1, "DAPI", 461, 359)
        self.assertTrue(self.panel.commit())
        self.assertEqual(self.panel._store.filter_optics(),
                         {"DAPI": {"emission_nm": 461.0,
                                   "excitation_nm": 359.0}})
        self.panel.load()
        self.assertEqual(self.table._rows[1]["em"].value(), 461)
        self.assertEqual(self.table._rows[1]["ex"].value(), 359)

    def test_keyed_by_cube_name_not_slot(self):
        """A wavelength is a property of the cube. Moving it to another slot
        must not lose it."""
        self._set(1, "FITC", 519, 495)
        self.panel.commit()
        self._set(1, "", 0, 0)
        self._set(4, "FITC", 519, 495)
        self.panel.commit()
        self.assertIn("FITC", self.panel._store.filter_optics())

    def test_unset_stays_absent_never_zero(self):
        """LabLink names a missing field, which is recoverable. A fabricated
        wavelength looks measured and silently changes the result — measured on
        real data as a segmented object count moving 2855 → 2660."""
        self._set(1, "Cy5", 0, 0)
        self.panel.commit()
        self.assertEqual(self.panel._store.filter_optics(), {})
        self.assertEqual(self.panel._store.filter_optics_for("Cy5"), {})

    def test_emission_alone_is_enough(self):
        self._set(1, "TxRed", 617, 0)
        self.panel.commit()
        entry = self.panel._store.filter_optics_for("TxRed")
        self.assertEqual(entry, {"emission_nm": 617.0})

    def test_clearing_a_row_clears_the_store(self):
        self._set(1, "DAPI", 461, 359)
        self.panel.commit()
        self._set(1, "DAPI", 0, 0)
        self.panel.commit()
        self.assertEqual(self.panel._store.filter_optics(), {})

    def test_out_of_band_is_refused_and_nothing_is_saved(self):
        """Refused, not rounded: a wavelength nudged into range still looks
        measured."""
        self._set(1, "Weird", 100, 0)         # below the store's 200 nm floor
        self.assertFalse(self.panel.commit())
        self.assertTrue(self._warnings, "the operator was never told")
        self.assertIn("100", self._warnings[-1])
        self.assertEqual(self.panel._store.filter_optics(), {})

    def test_a_valid_commit_shows_no_warning(self):
        """Guards the guard: a test class that swallows every message box would
        pass even if commit() refused everything."""
        self._set(1, "DAPI", 461, 359)
        self.assertTrue(self.panel.commit())
        self.assertEqual(self._warnings, [])

    def test_a_nameless_slot_contributes_nothing(self):
        self._set(2, "", 500, 400)
        self.assertEqual(self.table.optics(), {})

    def test_values_survive_a_slot_count_rebuild(self):
        self._set(2, "Cy5", 670, 650)
        self.panel._filter_slots_spin.setValue(3)
        self.panel._filter_slots_spin.setValue(6)
        self.assertEqual(self.table._rows[2]["edit"].text(), "Cy5")
        self.assertEqual(self.table._rows[2]["em"].value(), 670)

    def test_lookup_is_case_and_space_insensitive(self):
        """The scan's channel name comes from FluorescenceMosaicStore.CHANNELS
        while the cube name is typed by hand; "FITC " must not cost a
        deconvolution its emission wavelength."""
        self._set(1, "FITC", 519, 495)
        self.panel.commit()
        self.assertTrue(self.panel._store.filter_optics_for("  fitc  "))

    def test_objectives_have_no_wavelength_columns(self):
        """Only filter cubes carry wavelengths; an objective's NA is reported
        by the body itself."""
        self.assertNotIn("em", self.panel._objective_table._rows[1])
        self.assertEqual(self.panel._objective_table.optics(), {})

    def test_the_spin_upper_bound_comes_from_the_store(self):
        """One owner of the plausible band. A second copy here would let an
        operator type a value they cannot save."""
        from SupportClasses.MicroscopeConfigStore import WAVELENGTH_BAND_NM
        self.assertEqual(self.table._rows[1]["em"].maximum(),
                         int(WAVELENGTH_BAND_NM[1]))

    def test_zero_reads_as_unknown_not_as_a_number(self):
        self.assertEqual(self.table._rows[1]["em"].specialValueText(), "—")

    def test_the_sidecar_builder_reads_this_store(self):
        """End to end: what the operator types here is what LabLink is told."""
        self._set(1, "FITC", 519, 495)
        self.panel.commit()
        from SupportClasses.MicroscopeConfigStore import get_store
        lookup = get_store().filter_optics_for
        self.assertEqual(lookup("FITC")["emission_nm"], 519.0)


if __name__ == "__main__":
    unittest.main()
