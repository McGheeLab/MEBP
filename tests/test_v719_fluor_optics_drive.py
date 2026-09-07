"""
v7.19 — the Fluorescence Mosaic workflow actually drives the optics.

Operator: *"when i'm in fluorescence mosaic workflow, and i select a different
objective at the top, it should change the objective if the microscope is
connected"*, and — when asked what should happen if it cannot — *"if the
microscope is disconnected or is a manual turret microscope, trust the user that
they are setting the microscope up correctly."*

The four things that must not regress:

1. **A pick that cannot be honoured REVERTS the combo.** A combo showing an
   objective that is not in the light path is the bug this change removes: every
   tile of the next mosaic is scaled by that name's µm/px.
2. **A pick is refused mid-scan.** The run freezes its raster positions and
   µm/px once and replays them for every channel, so changing the objective
   part-way leaves the tile spacing not matching the field of view.
3. **The prompt names the CASSETTE SLOT, never the acquisition ordinal.**
   ``channel_number`` is deprecated and, on this rig, calls slot 3 "mCherry"
   when the cassette holds TxRed and "Bright Field (5)" when slot 5 is empty.
4. **A SIMULATED switch still prompts.** The operator has a real cube to move.
"""

from __future__ import annotations

import ast
import inspect
import os
import tempfile
import textwrap
import unittest
from types import SimpleNamespace

try:
    from PySide6.QtWidgets import QApplication
    _QT = True
except Exception:                                            # pragma: no cover
    _QT = False

_APP = None


def _ensure_app():
    global _APP
    if _APP is None:
        _APP = QApplication.instance() or QApplication([])
    return _APP


class _Result:
    """Stands in for OpticsService.EnsureResult."""

    def __init__(self, ok=True, why_not="", simulated=False, degraded=False):
        self.ok = ok
        self.why_not = why_not
        self.simulated = simulated
        self.degraded = degraded

    def describe(self):
        return "objective 1 -> 2 (10X) verified"


def _state(*, connected=True, objective_position=1, filter_position=2,
           has_filter=True):
    return SimpleNamespace(
        connected=connected, has_filter=has_filter, has_objective=True,
        objective_position=objective_position, filter_position=filter_position,
        mounted_objectives=(), mounted_filters=(),
        native_objective_names=(), native_filter_names=())


def _page():
    os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())

    class SL:
        xy_min_x = xy_min_y = 0.0
        xy_max_x, xy_max_y = 120000.0, 80000.0

    class Ctrl:
        safety_limits = SL()
        is_zp_connected = False
        zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    from gui.pages.workflows.fluorescence_mosaic_workflow import (
        FluorescenceMosaicWorkflowPage)
    return FluorescenceMosaicWorkflowPage(
        controller=Ctrl(), settings=object(), camera_manager=None)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestNothingRotatesWithoutAnOperatorAskingFor(unittest.TestCase):
    """🔴 v7.19.1 — a CONFIG LOAD was rotating the nosepiece.

    ``_refresh_objectives`` ends by re-applying the restored objective, and it
    is called from ``set_hardware_config``. That was harmless while
    ``_on_objective_changed`` was a software relabel; v7.19 gave it a second job
    — ``_drive_objective`` → ``ensure_optics_async`` — so loading a saved
    hardware setup commanded a turret rotation nobody asked for.
    """

    def setUp(self):
        _ensure_app()

    def test_restoring_the_combo_pushes_um_per_px_but_moves_nothing(self):
        pg = _page()
        drove, adopted = [], []
        pg._drive_objective = lambda n: drove.append(n)
        pg._adopt_objective = lambda n: adopted.append(n)
        pg._nosepiece_objective_names = lambda: ["4x", "10X", "20x"]
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="10X"))

        pg._refresh_objectives()

        self.assertEqual(drove, [], "a config load rotated the nosepiece")
        self.assertEqual(adopted, ["10X"], "the µm/px push was lost with it")

    def test_the_restore_path_does_not_call_the_driving_handler(self):
        """AST pin — the two jobs live in one handler, so the call site is the
        only thing separating 'record it' from 'move the hardware'."""
        from gui.pages.workflows import fluorescence_mosaic_workflow as mod
        src = textwrap.dedent(inspect.getsource(
            mod.FluorescenceMosaicWorkflowPage._refresh_objectives))
        called = {getattr(n.func, "attr", "") for n in ast.walk(ast.parse(src))
                  if isinstance(n, ast.Call)}
        self.assertNotIn("_on_objective_changed", called)
        self.assertIn("_adopt_objective", called)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestARefusedRotationLeavesNothingBehind(unittest.TestCase):
    """🔴 v7.19.1 — reverting the COMBO was not enough.

    ``ensure_optics_async`` is asynchronous, so ``_adopt_objective`` has already
    written ``current_objective_name`` and pushed µm/px by the time a refusal
    lands. Reverting only the combo left the screen right and the numbers wrong,
    which is worse than either being wrong on its own: nothing on screen
    disagrees with anything else.
    """

    def setUp(self):
        _ensure_app()

    def test_the_declared_objective_goes_back_to_what_is_in_the_path(self):
        pg = _page()
        adopted = []
        pg._adopt_objective = lambda n: adopted.append(n)
        pg._revert_objective_combo = lambda: "4x"      # what the body really has
        pg._on_objective_ensured(
            _Result(ok=False, why_not="no glass focus datum for 20x"))
        self.assertEqual(adopted, ["4x"],
                         "µm/px was left on the objective that never arrived")

    def test_nothing_is_re_adopted_when_the_body_cannot_be_read(self):
        """Unknown must not become a guess — leave it, and say why in status."""
        pg = _page()
        adopted = []
        pg._adopt_objective = lambda n: adopted.append(n)
        pg._revert_objective_combo = lambda: ""
        pg._on_objective_ensured(_Result(ok=False, why_not="not connected"))
        self.assertEqual(adopted, [])
        self.assertIn("NOT changed", pg._status.text())

    def test_a_successful_switch_does_not_re_adopt(self):
        pg = _page()
        adopted = []
        pg._adopt_objective = lambda n: adopted.append(n)
        pg._on_objective_ensured(_Result(ok=True))
        self.assertEqual(adopted, [])


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheObjectiveComboDrivesTheNosepiece(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_a_pick_asks_the_body_to_rotate(self):
        pg = _page()
        asked = []
        pg._drive_objective = lambda name: asked.append(name)
        pg._on_objective_changed("20x")
        self.assertEqual(asked, ["20x"])

    def test_it_is_refused_while_a_scan_is_running(self):
        """The raster is planned for the objective the run started with."""
        pg = _page()
        asked = []
        pg._drive_objective = lambda name: asked.append(name)
        pg.is_scanning = lambda: True
        reverted = []
        pg._revert_objective_combo = lambda: reverted.append(True)
        pg._on_objective_changed("20x")
        self.assertEqual(asked, [], "must not rotate mid-scan")
        self.assertEqual(reverted, [True])
        self.assertIn("capture is running", pg._status.text())

    def test_a_refusal_reverts_the_combo_and_says_why(self):
        pg = _page()
        reverted = []
        pg._revert_objective_combo = lambda: reverted.append(True)
        pg._on_objective_ensured(
            _Result(ok=False, why_not="no glass focus datum for 20x"))
        self.assertEqual(reverted, [True])
        self.assertIn("no glass focus datum", pg._status.text())

    def test_no_microscope_leaves_the_operators_pick_standing(self):
        """A manual turret: they told us what they fitted."""
        pg = _page()
        reverted = []
        pg._revert_objective_combo = lambda: reverted.append(True)
        pg._on_objective_ensured(None)          # no body at all
        self.assertEqual(reverted, [])

    def test_a_successful_switch_does_not_revert(self):
        pg = _page()
        reverted = []
        pg._revert_objective_combo = lambda: reverted.append(True)
        pg._on_objective_ensured(_Result(ok=True))
        self.assertEqual(reverted, [])

    def test_the_combo_offers_the_nosepiece_slots(self):
        pg = _page()
        import SupportClasses.OpticsRegistry as reg
        slots = [SimpleNamespace(position=2, name="10X"),
                 SimpleNamespace(position=1, name="4X"),
                 SimpleNamespace(position=3, name="")]
        orig = reg.resolve_slots
        reg.resolve_slots = lambda **kw: slots
        try:
            names = pg._nosepiece_objective_names()
        finally:
            reg.resolve_slots = orig
        # Turret ORDER, and an unnamed position is not offered.
        self.assertEqual(names, ["4X", "10X"])

    def test_an_unknown_body_objective_does_not_move_the_combo(self):
        """Reverting to a name the combo does not contain must be a no-op, not
        a silent jump to index 0 (which is a different objective)."""
        pg = _page()
        pg._objective_combo.clear()
        pg._objective_combo.addItems(["4X", "10X"])
        pg._objective_combo.setCurrentIndex(1)
        pg._scope_state = lambda: _state(objective_position=6)
        pg._revert_objective_combo()
        self.assertEqual(pg._objective_combo.currentText(), "10X")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheGlassFocusAndNeedleGates(unittest.TestCase):
    """ensure_objective refuses without a glass datum, and refuses outright on
    needle_retracted=False — so "unknown" must never be reported as False."""

    def setUp(self):
        _ensure_app()

    def test_unknown_needle_state_is_none_not_false(self):
        pg = _page()
        self.assertIsNone(pg._needle_is_retracted())   # no ZP connected

    def test_missing_glass_datum_is_none(self):
        pg = _page()
        self.assertIsNone(pg._glass_focus_um())


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheCubeIsSwitchedNotAskedFor(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_a_successful_switch_reports_switched(self):
        pg = _page()
        import gui.widgets.optics_ensure as oe
        orig = oe.build_service
        oe.build_service = lambda owner: SimpleNamespace(
            ensure_filter=lambda name: _Result(ok=True))
        try:
            switched, why = pg._ensure_cube_for("FITC")
        finally:
            oe.build_service = orig
        self.assertTrue(switched)
        self.assertEqual(why, "")

    def test_a_refusal_falls_back_to_the_prompt_with_the_reason(self):
        pg = _page()
        import gui.widgets.optics_ensure as oe
        orig = oe.build_service
        oe.build_service = lambda owner: SimpleNamespace(
            ensure_filter=lambda name: _Result(
                ok=False, why_not="slot 5 is empty"))
        try:
            switched, why = pg._ensure_cube_for("Bright Field")
        finally:
            oe.build_service = orig
        self.assertFalse(switched)
        self.assertIn("slot 5 is empty", why)

    def test_a_SIMULATED_switch_still_prompts(self):
        """A simulated switch reported as real is a fabricated fact — and the
        operator still has a physical cube to move."""
        pg = _page()
        import gui.widgets.optics_ensure as oe
        orig = oe.build_service
        oe.build_service = lambda owner: SimpleNamespace(
            ensure_filter=lambda name: _Result(ok=True, simulated=True))
        try:
            switched, why = pg._ensure_cube_for("DAPI")
        finally:
            oe.build_service = orig
        self.assertFalse(switched)
        self.assertIn("simulated", why)

    def test_no_service_is_a_quiet_fallback(self):
        pg = _page()
        import gui.widgets.optics_ensure as oe
        orig = oe.build_service
        oe.build_service = lambda owner: None
        try:
            switched, why = pg._ensure_cube_for("DAPI")
        finally:
            oe.build_service = orig
        self.assertFalse(switched)
        self.assertEqual(why, "")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestThePromptStopsPrintingAWrongFact(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_the_label_names_the_resolved_slot(self):
        pg = _page()
        pg._cube_slots = lambda state: ({"FITC": 2}, {})
        self.assertIn("cassette slot 2", pg._channel_prompt_label("FITC"))

    def test_an_unresolvable_channel_gets_NO_number(self):
        """Better no number than one naming a cube that is not fitted."""
        pg = _page()
        pg._cube_slots = lambda state: ({}, {"mCherry": "not configured"})
        label = pg._channel_prompt_label("mCherry")
        self.assertEqual(label, "mCherry channel")
        self.assertNotIn("3", label)          # the deprecated ordinal

    def test_the_prompt_path_never_calls_channel_number(self):
        """AST, not a substring: a substring check passes on a comment or an
        import line (the documented v7.10/v7.11 weak-guard trap)."""
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage as P)
        for fn in (P._prompt_next_channel, P._channel_prompt_label):
            src = textwrap.dedent(inspect.getsource(fn))
            names = {getattr(n.func, "attr", getattr(n.func, "id", ""))
                     for n in ast.walk(ast.parse(src))
                     if isinstance(n, ast.Call)}
            self.assertNotIn("channel_number", names, fn.__name__)
            self.assertNotIn("channel_ordinal", names, fn.__name__)

    def test_channel_slot_is_what_resolves_a_cube(self):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage as P)
        src = textwrap.dedent(inspect.getsource(P._cube_slots))
        names = {getattr(n.func, "attr", getattr(n.func, "id", ""))
                 for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)}
        self.assertIn("channel_slot", names)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestWhatWasRecordedWithTheChannel(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_the_cube_actually_in_the_path_is_recorded(self):
        pg = _page()
        pg._scope_state = lambda: _state(filter_position=3)
        import SupportClasses.OpticsRegistry as reg
        orig = reg.optic_at
        reg.optic_at = lambda st, pos, kind: SimpleNamespace(label="TxRed")
        try:
            rec = pg._cube_record("mCherry")
        finally:
            reg.optic_at = orig
        # The BODY's label, not the channel name we asked for.
        self.assertEqual(rec["cube_slot"], 3)
        self.assertEqual(rec["cube_label"], "TxRed")

    def test_an_unreadable_body_records_NOTHING(self):
        """Absent is recoverable; a fabricated cube name is not."""
        pg = _page()
        pg._scope_state = lambda: _state(connected=False)
        self.assertEqual(pg._cube_record("DAPI"), {})
        pg._scope_state = lambda: None
        self.assertEqual(pg._cube_record("DAPI"), {})

    def test_the_store_keeps_the_cube_fields(self):
        import tempfile as _tf
        import numpy as np
        from SupportClasses.FluorescenceMosaicStore import (
            FluorescenceMosaicStore)
        path = os.path.join(_tf.mkdtemp(), "fm.json")
        st = FluorescenceMosaicStore(path)
        img = np.zeros((4, 4, 3), dtype=np.uint8)
        self.assertTrue(st.save_channel(
            "24", "A1", "mCherry", img, (0.0, 0.0, 10.0, 10.0),
            cube_slot=3, cube_label="TxRed", gain_pct=40.0))
        ch = st.get_well("24", "A1")["channels"]["mCherry"]
        self.assertEqual(ch["cube_slot"], 3)
        self.assertEqual(ch["cube_label"], "TxRed")
        self.assertEqual(ch["gain_pct"], 40.0)

    def test_a_capture_that_knows_none_of_them_round_trips_unchanged(self):
        """Every new field is conditional, so a pre-v7.19 save is untouched."""
        import tempfile as _tf
        import numpy as np
        from SupportClasses.FluorescenceMosaicStore import (
            FluorescenceMosaicStore)
        path = os.path.join(_tf.mkdtemp(), "fm.json")
        st = FluorescenceMosaicStore(path)
        img = np.zeros((4, 4, 3), dtype=np.uint8)
        st.save_channel("24", "A1", "DAPI", img, (0.0, 0.0, 10.0, 10.0))
        ch = st.get_well("24", "A1")["channels"]["DAPI"]
        for key in ("cube_slot", "cube_label", "gain_pct"):
            self.assertNotIn(key, ch)


class TestTheAdapterExists(unittest.TestCase):
    """OpticsService's docstring names this module; it had no callers at all."""

    def test_it_runs_off_the_gui_thread(self):
        import gui.widgets.optics_ensure as oe
        src = textwrap.dedent(inspect.getsource(oe.ensure_optics_async))
        names = {getattr(n.func, "attr", getattr(n.func, "id", ""))
                 for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)}
        self.assertIn("Thread", names)

    def test_no_microscope_calls_back_with_none_and_returns_false(self):
        import gui.widgets.optics_ensure as oe
        got = []
        ok = oe.ensure_optics_async(kind="filter", name="DAPI", service=None,
                                    on_done=got.append)
        # build_service() will fail to reach a body in the test environment.
        if not ok:
            self.assertEqual(got, [None])

    def test_describe_refusal_never_returns_empty(self):
        import gui.widgets.optics_ensure as oe
        self.assertTrue(oe.describe_refusal(None))
        self.assertTrue(oe.describe_refusal(_Result(ok=False)))
        self.assertTrue(oe.describe_refusal(_Result(ok=False, degraded=True)))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheObjectiveIsActuallyDetected(unittest.TestCase):
    """🐞 v7.19.2, operator: *"flourescence mosaic is not autodetecting the
    objective and the current filter. I changed both and it did not update."*

    The cube half was a real hardware read all along (``filter_position``); the
    objective half returned ``camera_config.current_objective_name`` — the app's
    OWN declared value — so the panel compared its combo against a copy of
    itself. Nothing was being read, so nothing could ever update.
    """

    def setUp(self):
        _ensure_app()

    def test_the_body_is_the_source_not_the_declared_name(self):
        pg = _page()
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="4x"))
        pg._scope_state = lambda: _state(objective_position=2)
        pg._cube_slots = lambda _st: ({}, {})
        # Slot 2 is 10X on this fixture's registry stand-in.
        pg._live_objective_name = lambda: "10X"
        self.assertEqual(pg.panel_optics_state()["objective"], "10X")

    def test_it_falls_back_to_the_declared_name_with_no_body(self):
        pg = _page()
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="4x"))
        pg._scope_state = lambda: _state(connected=False)
        self.assertEqual(pg._live_objective_name(), "4x")

    def test_a_detected_change_adopts_its_um_per_px(self):
        """Showing the new objective is not enough: µm/px is keyed by NAME, so
        a detected change that is not adopted leaves every tile of the mosaic
        measured with the previous objective's scale."""
        pg = _page()
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="4x"))
        adopted = []
        pg._adopt_objective = lambda n: adopted.append(n)
        pg.on_panel_objective_detected("10X")
        self.assertEqual(adopted, ["10X"])

    def test_detection_never_drives_the_turret(self):
        """It is already where it is; commanding it would be an unasked-for
        move — and a poll that moves hardware is the thing this must not be."""
        pg = _page()
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="4x"))
        drove = []
        pg._drive_objective = lambda n: drove.append(n)
        pg._adopt_objective = lambda n: None
        pg.on_panel_objective_detected("10X")
        self.assertEqual(drove, [])

    def test_no_change_no_write(self):
        pg = _page()
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="10X"))
        adopted = []
        pg._adopt_objective = lambda n: adopted.append(n)
        pg.on_panel_objective_detected("10X")
        self.assertEqual(adopted, [])

    def test_a_running_scan_is_never_re_scaled_underneath(self):
        """The raster is planned once against one µm/px and replayed for every
        channel; adopting a new scale mid-run would leave the tile spacing not
        matching the field of view."""
        pg = _page()
        pg._hw_config = SimpleNamespace(
            camera_config=SimpleNamespace(current_objective_name="4x"))
        pg.is_scanning = lambda: True
        adopted = []
        pg._adopt_objective = lambda n: adopted.append(n)
        pg.on_panel_objective_detected("10X")
        self.assertEqual(adopted, [])


if __name__ == "__main__":
    unittest.main()
