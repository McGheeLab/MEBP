"""
v7.19 — the objective-calibration table is populated from the Microscope page.

Operator: *"in the microscope camera objective calibration the objectives in the
table should be auto populated based on the objectives defined in the microscope
page. it should also know what the current objective is."*

Two records of "which objective" have existed side by side since v7.11 and were
never joined: ``MicroscopeConfigStore.objectives`` (turret position → operator
name) and ``ObjectiveCalibration`` (camera → name → µm/px). This joins them for
display and selection **without touching either store** — they already agree by
string, case-insensitively.

⚠ THE RULE THAT MUST NOT BE RELAXED: normalize at LOOKUP, never rewrite a
measurement's key. This rig's nosepiece labels are ``4X``/``10X``/``20x`` while
the calibration keys are ``4x``/``10x``/``20x``; renaming either side to match
would orphan the operator's measurements — the v7.16 class of change.
"""

from __future__ import annotations

import ast
import inspect
import textwrap
import unittest
from types import SimpleNamespace

try:
    from PySide6.QtGui import QHideEvent, QShowEvent
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


def _card():
    """The real card, with no camera manager and no config."""
    from gui.pages.hardware.objective_calibration_card import (
        ObjectiveCalibrationCard)
    return ObjectiveCalibrationCard(
        camera_manager=None,
        config_getter=lambda: None,
        controller_getter=lambda: None)


def _slots(*pairs):
    return [SimpleNamespace(position=p, name=n) for p, n in pairs]


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheRowsComeFromTheNosepiece(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def _with_slots(self, card, slots, live=None):
        card.nosepiece_objectives = lambda: [
            (s.position, s.name) for s in sorted(slots, key=lambda x: x.position)
            if s.name]
        card.live_objective_position = lambda: live

    def test_rows_are_the_named_slots_in_turret_order(self):
        card = _card()
        self._with_slots(card, _slots((3, "20x"), (1, "4X"), (2, "10X")))
        self.assertEqual(card._objective_rows(),
                         [(1, "4X"), (2, "10X"), (3, "20x")])

    def test_an_unnamed_slot_is_not_offered(self):
        card = _card()
        self._with_slots(card, _slots((1, "4X"), (2, ""), (3, "20x")))
        self.assertEqual([n for _p, n in card._objective_rows()],
                         ["4X", "20x"])

    def test_no_nosepiece_falls_back_to_the_cards_own_list(self):
        """A rig with no motorised body keeps exactly the old behaviour."""
        card = _card()
        card.nosepiece_objectives = lambda: []
        card._store = SimpleNamespace(
            objective_names=lambda: ["4x", "40x oil"],
            nominal_magnification=lambda _n: None,
            all_calibrations_for_camera=lambda _k: {},
            get_calibration=lambda *_a: None)
        self.assertEqual(card._objective_rows(), [(None, "4x"), (None, "40x oil")])

    def test_the_combo_marks_the_objective_in_the_light_path(self):
        card = _card()
        self._with_slots(card, _slots((1, "4X"), (2, "10X")), live=2)
        card._reload_objective_combo()
        texts = [card._cmb_objective.itemText(i)
                 for i in range(card._cmb_objective.count())]
        self.assertIn("in path", texts[1])
        self.assertNotIn("in path", texts[0])

    def test_a_disconnected_body_marks_nothing(self):
        card = _card()
        self._with_slots(card, _slots((1, "4X"), (2, "10X")), live=None)
        card._reload_objective_combo()
        texts = [card._cmb_objective.itemText(i)
                 for i in range(card._cmb_objective.count())]
        self.assertFalse(any("in path" in t for t in texts))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheRawNameSurvivesTheDecoration(unittest.TestCase):
    """The displayed label is decorated; the KEY must not be."""

    def setUp(self):
        _ensure_app()

    def test_combo_userdata_is_the_raw_name(self):
        card = _card()
        card.nosepiece_objectives = lambda: [(1, "4X"), (2, "10X")]
        card.live_objective_position = lambda: 2
        card._reload_objective_combo(preferred="10X")
        self.assertEqual(card._cmb_objective.currentData(), "10X")
        self.assertIn("in path", card._cmb_objective.currentText())

    def test_write_to_config_stores_the_raw_name(self):
        """🐞 It used to store currentText() — "4x (4×)" — which matches no
        calibration key and no nosepiece label."""
        card = _card()
        card.nosepiece_objectives = lambda: [(1, "4X")]
        card.live_objective_position = lambda: 1
        card._reload_objective_combo(preferred="4X")
        cfg = SimpleNamespace(camera_config=SimpleNamespace(
            current_objective_name=None))
        card.write_to_config(cfg)
        self.assertEqual(cfg.camera_config.current_objective_name, "4X")

    def test_write_to_config_never_uses_currentText(self):
        from gui.pages.hardware.objective_calibration_card import (
            ObjectiveCalibrationCard as C)
        src = textwrap.dedent(inspect.getsource(C.write_to_config))
        names = {getattr(n.func, "attr", "") for n in ast.walk(ast.parse(src))
                 if isinstance(n, ast.Call)}
        self.assertNotIn("currentText", names)

    def test_selected_objective_name_reads_userdata_not_the_cell_text(self):
        card = _card()
        card.nosepiece_objectives = lambda: [(1, "4X"), (2, "10X")]
        card.live_objective_position = lambda: 2
        card._store = SimpleNamespace(
            objective_names=lambda: [],
            nominal_magnification=lambda _n: 4.0,
            all_calibrations_for_camera=lambda _k: {},
            get_calibration=lambda *_a: None)
        card._camera_key = lambda: "tucam:0"
        card._refresh_table()
        card._table.selectRow(1)
        self.assertEqual(card._selected_objective_name(), "10X")
        self.assertIn("in path", card._table.item(1, 0).text())


@unittest.skipUnless(_QT, "PySide6 not available")
class TestNoStoreIsRewritten(unittest.TestCase):
    """4X ↔ 4x must resolve, and neither key may be renamed."""

    def setUp(self):
        _ensure_app()

    def test_a_case_variant_slot_label_finds_its_calibration(self):
        import os
        import tempfile
        from SupportClasses.ObjectiveCalibration import ObjectiveCalibrationStore
        from pathlib import Path
        path = Path(tempfile.mkdtemp()) / "objectives.json"
        store = ObjectiveCalibrationStore(path)
        store.add_objective("4x", 4.0)
        store.set_calibration("tucam:0", "4x", 1.8968, (2600, 2048))

        card = _card()
        card._store = store
        card._camera_key = lambda: "tucam:0"
        card.nosepiece_objectives = lambda: [(1, "4X")]   # the panel's label
        card.live_objective_position = lambda: 1
        card._refresh_table()
        # The µm/px column must NOT read "—".
        self.assertEqual(card._table.item(0, 2).text(), "1.8968")
        # ...and the on-disk key is untouched.
        self.assertEqual(list(store.all_calibrations_for_camera("tucam:0")),
                         ["4x"])

    def test_the_table_does_not_add_or_remove_objectives(self):
        from gui.pages.hardware.objective_calibration_card import (
            ObjectiveCalibrationCard as C)
        for fn in (C._objective_rows, C.nosepiece_objectives,
                   C._reload_objective_combo, C._refresh_table):
            src = textwrap.dedent(inspect.getsource(fn))
            names = {getattr(n.func, "attr", "")
                     for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)}
            for forbidden in ("add_objective", "remove_objective",
                              "set_calibration", "clear_calibration"):
                self.assertNotIn(forbidden, names, f"{fn.__name__} → {forbidden}")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheMarkerFollowsTheBody(unittest.TestCase):
    """🐞 v7.19.1 — operator: *"the physical objective changed, but didnt
    update the in path"*.

    The ``← in path`` marker was resolved at build time and on a combo change,
    and nowhere else. This card is not the only thing that moves the nosepiece
    (the jog microscope card, the Microscope setup panel, the fluorescence panel
    and the operator's hand all do), so the marker went stale — and a stale
    marker does not merely fail to update, it keeps asserting a false fact about
    the hardware.
    """

    def setUp(self):
        _ensure_app()

    def _card_at(self, live_holder):
        card = _card()
        card.nosepiece_objectives = lambda: [(1, "4X"), (2, "10X"), (3, "20x")]
        card.live_objective_position = lambda: live_holder["pos"]
        card._camera_key = lambda: "tucam:0"
        card._store = SimpleNamespace(
            objective_names=lambda: ["4x", "10x", "20x"],
            nominal_magnification=lambda _n: None,
            all_calibrations_for_camera=lambda _k: {},
            get_calibration=lambda *_a: None)
        return card

    def _marked(self, card):
        return [card._cmb_objective.itemText(i)
                for i in range(card._cmb_objective.count())
                if "in path" in card._cmb_objective.itemText(i)]

    def test_the_marker_moves_when_the_turret_is_driven_elsewhere(self):
        """Fired through the TIMER, never by calling the poll by hand.

        Calling ``_poll_light_path()`` directly would pass with the timer
        completely unwired — the exact weakness that let the v7.18 setpoint
        keeper ship broken with green tests.
        """
        live = {"pos": 1}
        card = self._card_at(live)
        card.showEvent(QShowEvent())          # seeds + starts the timer
        self.assertEqual(self._marked(card), ["1. 4X  ← in path"])

        live["pos"] = 3                            # somebody else rotated it
        card._path_timer.timeout.emit()

        self.assertEqual(self._marked(card), ["3. 20x  ← in path"],
                         "the marker did not follow the body")

    def test_the_timer_is_started_and_stopped_with_visibility(self):
        live = {"pos": 1}
        card = self._card_at(live)
        self.assertFalse(card._path_timer.isActive())
        card.showEvent(QShowEvent())
        self.assertTrue(card._path_timer.isActive())
        card.hideEvent(QHideEvent())
        self.assertFalse(card._path_timer.isActive(),
                         "an off-screen card must not keep polling")

    def test_the_poll_is_connected_to_the_timer(self):
        """AST pin: the behavioural test above rides on this connection."""
        from gui.pages.hardware import objective_calibration_card as mod
        src = textwrap.dedent(inspect.getsource(
            mod.ObjectiveCalibrationCard.__init__))
        tree = ast.parse(src)
        wired = any(
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == "connect"
            and isinstance(n.func.value, ast.Attribute)
            and n.func.value.attr == "timeout"
            and any(getattr(a, "attr", "") == "_poll_light_path" for a in n.args)
            for n in ast.walk(tree))
        self.assertTrue(wired, "_path_timer.timeout is not connected to "
                               "_poll_light_path")

    def test_the_poll_never_declares_the_objective_it_finds(self):
        """Following the body is a DISPLAY act.

        Adopting a polled position as ``current_objective_name`` would be a
        background write of the key the whole µm/px chain hangs off — the
        mutation the v7.18 optics design names as the one most likely to be
        proposed in good faith.
        """
        live = {"pos": 1}
        card = self._card_at(live)
        cfg = SimpleNamespace(
            camera_for_role=lambda _r: None,
            camera_config=SimpleNamespace(current_objective_name="4X",
                                          active_resolution=(0, 0)))
        card._config_getter = lambda: cfg
        pushed = []
        card._push_stored_um_per_px_to_manager = lambda n: pushed.append(n)

        card.showEvent(QShowEvent())
        live["pos"] = 3
        card._path_timer.timeout.emit()

        self.assertEqual(cfg.camera_config.current_objective_name, "4X")
        self.assertEqual(pushed, [], "the poll pushed µm/px")

    def test_a_disagreement_is_stated_not_left_looking_calibrated(self):
        live = {"pos": 3}
        card = self._card_at(live)
        card._store.get_calibration = lambda *_a: {
            "measured_um_per_px": 1.2345, "date": "2026-01-01",
            "resolution": (0, 0)}
        card._cmb_objective.blockSignals(True)
        card._cmb_objective.clear()
        card._cmb_objective.addItem("1. 4X", "4X")     # declared 4X…
        card._cmb_objective.blockSignals(False)

        card._refresh_objective_note()                  # …body says 20x

        note = card._lbl_obj_note.text()
        self.assertIn("20x", note)
        self.assertIn("4X", note)
        self.assertNotIn("Calibrated:", note,
                         "a green 'Calibrated' tick over a µm/px that is not "
                         "forming the image is the wrong reassurance")

    def test_an_open_dropdown_is_not_yanked_shut_by_the_poll(self):
        """v7.5.x Nikon Ti: *"every time it reads it cancels the dropdown box
        I have opened"*. A closed combo still tracks."""
        live = {"pos": 1}
        card = self._card_at(live)
        card.showEvent(QShowEvent())
        reloads = []
        card._reload_objective_combo = lambda **kw: reloads.append(kw)

        card._cmb_objective.addItem("1. 4X", "4X")
        card._cmb_objective.showPopup()                 # a REAL popup
        self.assertTrue(card._cmb_objective.view().isVisible())
        live["pos"] = 2
        card._path_timer.timeout.emit()
        self.assertEqual(reloads, [], "the poll rebuilt an open drop-down")

        card._cmb_objective.hidePopup()
        live["pos"] = 3
        card._path_timer.timeout.emit()
        self.assertEqual(len(reloads), 1, "a closed combo stopped tracking")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestPollingAStaleCacheIsNotTracking(unittest.TestCase):
    """🔴 v7.19.1 — ``MicroscopeController`` does not poll itself.

    ``state()`` is a snapshot updated by ``_read_all()`` after an op or by an
    explicit ``refresh()``, and repo-wide ``refresh()`` had exactly two callers:
    ``microscope_panel._tick`` (only while on screen) and ``OpticsService``
    (only around a switch we command). So a surface can poll ``state()``
    forever and never see a HAND-turned turret — while still passing every test
    that drives the change through the app, because an app-driven switch
    re-reads afterwards. That asymmetry is why this is pinned separately.
    """

    def setUp(self):
        _ensure_app()

    def test_the_card_asks_the_body_for_a_fresh_read(self):
        import gui.widgets.optics_ensure as oe
        card = _card()
        card.nosepiece_objectives = lambda: [(1, "4X")]
        card.live_objective_position = lambda: 1
        card._camera_key = lambda: "tucam:0"
        asked = []
        real = oe.request_state_refresh
        oe.request_state_refresh = lambda *a, **k: asked.append(1)
        try:
            card.showEvent(QShowEvent())
            card._path_timer.timeout.emit()
        finally:
            oe.request_state_refresh = real
        self.assertTrue(asked, "the poll read a cache nothing refreshes")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheSharedFreshener(unittest.TestCase):
    """Its three refusals are each a real cost, so each is pinned."""

    def setUp(self):
        _ensure_app()
        import gui.widgets.optics_ensure as oe
        oe._last_refresh[0] = 0.0

    def _scope(self, **kw):
        st = SimpleNamespace(connected=kw.pop("connected", True),
                             busy=kw.pop("busy", False))
        calls = []
        return SimpleNamespace(
            state=lambda: st,
            lease_owner=lambda: kw.pop("lease", None),
            refresh=lambda: calls.append(1)), calls

    def _run(self, scope, **kw):
        import gui.widgets.optics_ensure as oe
        import SupportClasses.MicroscopeControl as mc
        real = mc.get_microscope
        mc.get_microscope = lambda: scope
        try:
            return oe.request_state_refresh(**kw)
        finally:
            mc.get_microscope = real

    def test_a_healthy_idle_body_is_refreshed(self):
        scope, calls = self._scope()
        self.assertTrue(self._run(scope))
        self.assertEqual(len(calls), 1)

    def test_it_is_throttled_process_wide(self):
        """Several panels can be alive at once; a per-widget throttle would
        multiply the hardware reads by the number of visible surfaces."""
        scope, calls = self._scope()
        self._run(scope)
        self._run(scope)
        self._run(scope)
        self.assertEqual(len(calls), 1)

    def test_a_busy_body_is_left_alone(self):
        scope, calls = self._scope(busy=True)
        self.assertFalse(self._run(scope))
        self.assertEqual(calls, [])

    def test_someone_elses_lease_is_never_polled_under(self):
        """STALE_OP_S drops a queued op silently — the one displaced could be
        the one that mattered."""
        scope, calls = self._scope(lease="fluor_mosaic_af")
        self.assertFalse(self._run(scope))
        self.assertEqual(calls, [])

    def test_a_disconnected_body_is_not_polled(self):
        scope, calls = self._scope(connected=False)
        self.assertFalse(self._run(scope))
        self.assertEqual(calls, [])

    def test_it_does_not_block_on_the_op(self):
        """refresh() returns an op handle; waiting on it from a GUI timer would
        stall the event loop behind the microscope's command queue."""
        from gui.widgets import optics_ensure as oe
        src = textwrap.dedent(inspect.getsource(oe.request_state_refresh))
        names = {getattr(n.func, "attr", "") for n in ast.walk(ast.parse(src))
                 if isinstance(n, ast.Call)}
        for forbidden in ("wait", "wait_for_op", "join"):
            self.assertNotIn(forbidden, names)


if __name__ == "__main__":
    unittest.main()
