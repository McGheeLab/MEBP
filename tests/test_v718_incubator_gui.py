"""
test_v718_incubator_gui.py — the Incubator GUI surfaces, offscreen, driving
the PRODUCTION widgets (a stand-in that agrees with itself proves nothing).

Covers:
  * the Workflows tile registration + dispatch (AST, so a removed elif is
    caught without constructing every workflow page);
  * IncubatorWorkflowPage: builds against the real controller singleton,
    connects to the SIMULATOR through its own worker-thread path (the
    GUI-thread-freeze fix), zone-card prefs from the store, refusal wording;
  * IncubatorSetupPanel: load()/commit() round-trip, live ceiling push;
  * HardwareSetupPage: the Incubator tab exists and resolves by NAME;
  * HardwareControlPanel: the Connect-card Incubator row + badge sync.

Modal dialogs are patched — QMessageBox blocks forever under offscreen Qt
(the documented headless-test trap).
"""

from __future__ import annotations

import os
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def _isolate_env():
    td = tempfile.mkdtemp(prefix="incu_gui_")
    os.environ["MEBP_INCUBATOR_CONFIG_DIR"] = td
    os.environ["MEBP_INCUBATOR_CAL_DIR"] = td
    os.environ["MEBP_INCUBATOR_SIM_DIR"] = td
    os.environ["MEBP_INCUBATOR_LOG"] = "0"
    return td


def _app():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _fresh_singletons():
    from SupportClasses.incubator.config_store import reset_store
    from SupportClasses.incubator.service import reset_service
    reset_store()
    reset_service()


class TestTileRegistration(unittest.TestCase):

    def test_tile_present_and_enabled(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        tiles = {t.workflow_id: t for t in WORKFLOWS}
        self.assertIn("incubator", tiles)
        self.assertTrue(tiles["incubator"].enabled)
        self.assertEqual("Incubator", tiles["incubator"].title)

    def test_dispatch_constructs_the_real_page(self):
        """AST over workflows_mode: the incubator elif must construct
        IncubatorWorkflowPage (constructing the whole WorkflowsModePage here
        would drag in every camera-touching workflow page)."""
        import ast
        import inspect
        import gui.pages.workflows_mode as wm
        tree = ast.parse(inspect.getsource(wm))
        found = False
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "IncubatorWorkflowPage"):
                found = True
        self.assertTrue(found,
                        "workflows_mode no longer constructs "
                        "IncubatorWorkflowPage")
        self.assertTrue(hasattr(wm.WorkflowsModePage, "incubator_page"))

    def test_icon_key_exists(self):
        from gui.widgets.icons import _ICONS
        self.assertIn("incubator", _ICONS)


class TestIncubatorWorkflowPage(unittest.TestCase):

    def setUp(self):
        _isolate_env()
        _fresh_singletons()
        self.app = _app()
        from gui.pages.workflows.incubator_workflow import (
            IncubatorWorkflowPage,
        )
        self.page = IncubatorWorkflowPage(controller=None, settings=None)

    def tearDown(self):
        try:
            if self.page.ctrl.connected:
                self.page.ctrl.disconnect()
        except Exception:
            pass
        self.page.deleteLater()
        self.app.processEvents()
        _fresh_singletons()

    def _pump(self, seconds: float, until=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.app.processEvents()
            if until is not None and until():
                return True
            time.sleep(0.02)
        return until() if until is not None else True

    def _sim_transport(self):
        from SupportClasses.incubator.config_store import get_store
        get_store().set("transport", "simulate")

    def test_contract(self):
        self.assertEqual("Incubator", self.page.get_page_title())
        self.assertIsNone(self.page.get_context_widget())
        self.assertTrue(hasattr(self.page, "back_requested"))
        self.page.on_status_update()   # must never raise

    def test_auto_session_does_not_block_the_gui_thread(self):
        """No connect button: the session starts itself — and the trigger
        must return without waiting for the probe (blocking here is the
        GUI-freeze class this rework removes)."""
        self._sim_transport()
        t0 = time.monotonic()
        self.page._ensure_session(force=True)
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertTrue(self.page._connecting)
        self.assertTrue(
            self._pump(15.0, until=lambda: self.page.ctrl.connected))
        self.assertTrue(
            self._pump(5.0, until=lambda: not self.page._connecting))
        self.assertEqual("simulated", self.page.ctrl.transport)

    def test_ensure_session_is_reentry_guarded_and_throttled(self):
        self._sim_transport()
        self.page._ensure_session(force=True)
        self.assertTrue(self.page._connecting)
        self.page._ensure_session(force=True)    # re-entry: dropped
        self._pump(15.0, until=lambda: self.page.ctrl.connected)
        self._pump(5.0, until=lambda: not self.page._connecting)
        # Once connected, further ticks are no-ops.
        self.page._ensure_session()
        self.assertFalse(self.page._connecting)

    def test_the_status_tick_itself_starts_the_session(self):
        """The wiring, not just the helper: MainWindow's on_status_update
        tick must be what joins the board — that is what replaces the
        connect button."""
        self._sim_transport()
        self.page._last_auto_attempt = 0.0
        self.page.on_status_update()
        self.assertTrue(
            self._pump(15.0, until=lambda: self.page.ctrl.connected))
        self.assertEqual("simulated", self.page.ctrl.transport)

    def test_shared_transport_waits_for_the_zp_board(self):
        """The whole point of the no-button design: with the ZP board down,
        the page attempts nothing and the hint names the REAL connect
        (the ZP one). No banner spam either."""
        self.page._ensure_session(force=True)   # transport defaults 'shared'
        self.assertFalse(self.page._connecting)
        self.assertFalse(self.page.ctrl.connected)
        self.page._refresh_transport_hint()
        self.assertIn("connect the zp board",
                      self.page._transport_hint.text().lower())
        self.assertFalse(self.page._banner.isVisible())

    def test_shared_transport_joins_a_live_zp_automatically(self):
        from types import SimpleNamespace
        from tests.test_v718_incubator_shared_link import _FakeMarlinZP
        zp = _FakeMarlinZP()
        sc = SimpleNamespace(zp_stage=zp, xy_stage=None,
                             zp_connected_port="COM6",
                             _preferred_zp_port=None, _pos_poller=None)
        self.page._stage_controller = sc
        # connect_from_store resolves the ZP through the stage controller.
        from SupportClasses.incubator.service import (
            make_exclusion_provider, make_zp_getter, make_poll_gate,
        )
        self.page._reserved_ports = make_exclusion_provider(sc)
        self.page._zp_getter = make_zp_getter(sc)
        self.page._shared_poll_gate = make_poll_gate(sc)
        self.page._ensure_session(force=True)
        self.assertTrue(
            self._pump(15.0, until=lambda: self.page.ctrl.connected))
        self.assertEqual("shared", self.page.ctrl.transport)
        self.assertIn("M115", zp.commands)

    def test_switching_transport_away_retires_a_simulator_session(self):
        from SupportClasses.incubator.config_store import get_store
        self._sim_transport()
        self.page._ensure_session(force=True)
        self.assertTrue(
            self._pump(15.0, until=lambda: self.page.ctrl.connected))
        self._pump(5.0, until=lambda: not self.page._connecting)
        get_store().set("transport", "shared")
        self.page._ensure_session(force=True)
        self.assertTrue(
            self._pump(10.0, until=lambda: not self.page.ctrl.connected))

    def test_set_target_through_the_card_with_confirm(self):
        from unittest.mock import patch
        from PySide6.QtWidgets import QMessageBox
        self._sim_transport()
        self.page._ensure_session(force=True)
        self.assertTrue(
            self._pump(15.0, until=lambda: self.page.ctrl.connected))
        card = self.page._cards["bed"]
        card._sp.setValue(37.0)
        # The first Set is a >10 °C jump from 0 → confirm dialog; a real
        # modal blocks forever offscreen, so answer Yes programmatically.
        with patch.object(QMessageBox, "question",
                          return_value=QMessageBox.Yes):
            card._on_set()
        self.assertEqual(
            37.0, self.page.ctrl.zone_runtime("bed").requested_c)

    def test_serial_controls_hidden_unless_serial_transport(self):
        """The manual Port/Detect/Connect strip exists only for the future
        dedicated (ESP32) board — never for the shared/simulator paths."""
        from SupportClasses.incubator.config_store import get_store
        page = self.page
        page.show()
        self.app.processEvents()
        # default transport 'shared'
        self.assertTrue(page._port.isHidden())
        self.assertTrue(page._connect_btn.isHidden())
        get_store().set("transport", "serial")
        page._apply_store_prefs()
        self.app.processEvents()
        self.assertFalse(page._port.isHidden())
        self.assertFalse(page._baud.isHidden())
        self.assertFalse(page._connect_btn.isHidden())
        get_store().set("transport", "simulate")
        page._apply_store_prefs()
        self.app.processEvents()
        self.assertTrue(page._connect_btn.isHidden())

    def test_store_prefs_reach_the_zone_cards(self):
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        st.set_zone("hotend", enabled=False, save=False)
        st.set_zone("bed", label="Water bath", preset_c=36.5, save=False)
        st.set("max_setpoint_c", 42.0)
        self.page._apply_store_prefs()
        self.assertEqual("Water bath", self.page._cards["bed"].title())
        self.assertFalse(self.page._cards["hotend"].isVisibleTo(self.page))
        self.assertEqual(42.0, self.page._cards["bed"]._sp.maximum())
        self.assertEqual(36.5, self.page._cards["bed"]._sp.value())

    def test_revisit_while_connected_keeps_the_typed_setpoint(self):
        """showEvent re-applies store prefs — but re-seeding the preset
        while a session is live would overwrite the operator's typed
        setpoint on every navigation back to the page."""
        self._sim_transport()
        self.page._ensure_session(force=True)
        self.assertTrue(
            self._pump(15.0, until=lambda: self.page.ctrl.connected))
        card = self.page._cards["bed"]
        card._sp.setValue(31.5)                     # operator's own value
        self.page._apply_store_prefs(
            seed_presets=not self.page.ctrl.connected)  # the showEvent path
        self.assertEqual(31.5, card._sp.value())
        # Idle again → presets may seed.
        self.page.ctrl.disconnect()
        self._pump(2.0, until=lambda: not self.page.ctrl.connected)
        self.page._apply_store_prefs(
            seed_presets=not self.page.ctrl.connected)
        self.assertEqual(37.0, card._sp.value())

    def test_shared_wordings(self):
        self.page.ctrl._transport = "shared"
        try:
            self.assertIn("SHARED", self.page._shared_board_note())
        finally:
            self.page.ctrl._transport = ""
        self.assertEqual("", self.page._shared_board_note())


class TestIncubatorSetupPanel(unittest.TestCase):

    def setUp(self):
        _isolate_env()
        _fresh_singletons()
        self.app = _app()

    def tearDown(self):
        _fresh_singletons()

    def test_load_commit_round_trip(self):
        from SupportClasses.incubator.config_store import get_store
        from gui.pages.hardware.incubator_panel import IncubatorSetupPanel
        panel = IncubatorSetupPanel()
        idx = panel._transport.findData("serial")
        panel._transport.setCurrentIndex(idx)
        panel._port.setText("COM9")
        panel._ceiling.setValue(41.0)
        panel._ramp_step.setValue(2.0)
        panel._zone_rows["bed"]["label"].setText("Bath")
        panel._zone_rows["hotend"]["enabled"].setChecked(False)
        self.assertTrue(panel.commit())

        st = get_store()
        self.assertEqual("serial", st.get("transport"))
        self.assertEqual("COM9", st.get("dedicated_port"))
        self.assertEqual(41.0, st.get("max_setpoint_c"))
        self.assertEqual(2.0, st.get("ramp_step_c"))
        self.assertEqual("Bath", st.zone("bed")["label"])
        self.assertFalse(st.zone("hotend")["enabled"])

        # A fresh panel loads what was saved.
        panel2 = IncubatorSetupPanel()
        self.assertEqual("serial", panel2._transport.currentData())
        self.assertEqual("COM9", panel2._port.text())
        self.assertEqual("Bath", panel2._zone_rows["bed"]["label"].text())

    def test_commit_pushes_ceiling_onto_a_live_controller(self):
        """The ceiling is safety-relevant — it must not wait for a
        reconnect."""
        from SupportClasses.incubator.service import get_incubator
        from gui.pages.hardware.incubator_panel import IncubatorSetupPanel
        ctrl = get_incubator()
        self.assertEqual(50.0, ctrl.MAX_SETPOINT_C)
        panel = IncubatorSetupPanel()
        panel._ceiling.setValue(40.0)
        self.assertTrue(panel.commit())
        self.assertEqual(40.0, ctrl.MAX_SETPOINT_C)

    def test_nothing_reaches_the_store_before_save(self):
        from SupportClasses.incubator.config_store import get_store
        from gui.pages.hardware.incubator_panel import IncubatorSetupPanel
        panel = IncubatorSetupPanel()
        panel._ceiling.setValue(33.0)
        self.assertEqual(50.0, get_store().get("max_setpoint_c"))


class TestHardwareSetupTab(unittest.TestCase):
    """The full HardwareSetupPage carries the new tab. Heavier build, so one
    page for the class (the v7.12 suite's pattern)."""

    @classmethod
    def setUpClass(cls):
        _isolate_env()
        _fresh_singletons()
        cls.app = _app()
        os.environ.setdefault("MEBP_PLATES_DIR", tempfile.mkdtemp())
        from gui.pages.hardware_setup import HardwareSetupPage
        cls.page = HardwareSetupPage()

    @classmethod
    def tearDownClass(cls):
        cls.page.deleteLater()
        cls.app.processEvents()
        _fresh_singletons()

    def test_tab_registered_and_resolves_by_name(self):
        self.assertTrue(hasattr(self.page, "_incubator_panel"))
        self.assertIn("incubator", self.page._sub_scrolls)
        idx = self.page.sub_page_index("Incubator")
        self.assertGreaterEqual(idx, 0)
        self.page.switch_to(idx)
        self.app.processEvents()
        self.assertEqual("Hardware: Incubator",
                         self.page.get_sub_page_title())


class TestNoSeparateConnectRow(unittest.TestCase):
    """Operator decision (v7.18 round 2): the heaters are wired to the ZP
    board itself, so the ZP Connect IS the incubator connect — the Connect
    Hardware card must NOT grow a second button for the same board. (A row
    returns only when the planned ESP32 sensor board exists as its own
    device.)"""

    def setUp(self):
        _isolate_env()
        _fresh_singletons()
        self.app = _app()
        from gui.pages.hardware.control_panel import HardwareControlPanel
        self.panel = HardwareControlPanel()

    def tearDown(self):
        self.panel.deleteLater()
        self.app.processEvents()
        _fresh_singletons()

    def test_no_incubator_row_on_the_connect_card(self):
        for attr in ("badge_incubator", "btn_connect_incubator",
                     "btn_simulate_incubator", "btn_disconnect_incubator"):
            self.assertFalse(hasattr(self.panel, attr), attr)

    def test_tick_never_constructs_an_incubator(self):
        # The card must not even TOUCH the incubator service on its tick.
        from SupportClasses.incubator.service import peek_incubator
        self.panel._controller = None
        self.panel.on_status_update()
        self.panel._sync_badges()
        self.assertIsNone(peek_incubator())

    def test_microscope_row_still_present(self):
        # The removal must not have taken the neighbouring row with it.
        self.assertTrue(hasattr(self.panel, "badge_scope"))


class TestAppShutdownHook(unittest.TestCase):

    def test_close_event_calls_shutdown_incubator(self):
        """AST over MainWindow.closeEvent: the incubator teardown must be
        present and must run BEFORE controller.shutdown() (in shared mode
        the heater-off commands ride the ZP link the controller is about to
        close)."""
        import ast
        import inspect
        from gui.app import MainWindow
        src = inspect.getsource(MainWindow.closeEvent)
        tree = ast.parse("class _T:\n" + src.replace("\n", "\n"))
        order = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = ""
                if isinstance(node.func, ast.Name):
                    name = node.func.id
                elif isinstance(node.func, ast.Attribute):
                    name = node.func.attr
                if name in ("shutdown_incubator", "shutdown"):
                    order.append((node.lineno, name))
        names = [n for _ln, n in sorted(order)]
        self.assertIn("shutdown_incubator", names)
        self.assertLess(names.index("shutdown_incubator"),
                        len(names) - 1 - names[::-1].index("shutdown"),
                        "shutdown_incubator must run before "
                        "controller.shutdown()")


class TestZoneCardRefusalDisplay(unittest.TestCase):
    """v7.18 bench fault: a firmware-REFUSED setpoint must be visible ON THE
    CARD, persistently — the transient status line is how `M104 S37` stayed
    refused for 48 minutes with a healthy-looking UI."""

    def setUp(self):
        _isolate_env()
        _fresh_singletons()
        self.app = _app()
        from SupportClasses.incubator.controller import IncubatorController
        from SupportClasses.incubator.zones import HOTEND_ZONE
        from gui.widgets.incubator_widgets import ZoneCard
        self.ctrl = IncubatorController()   # unconnected is fine for display
        self.card = ZoneCard(HOTEND_ZONE, self.ctrl)

    def tearDown(self):
        self.card.deleteLater()
        self.app.processEvents()
        _fresh_singletons()

    @staticmethod
    def _report(**kw):
        from types import SimpleNamespace
        base = dict(temp_c=24.5, target_c=37.0, error_c=-12.5, in_band=False,
                    settled=False, duty_pct=0.0, rate_c_per_min=0.0,
                    ripple_c=0.0, steady_duty_pct=None)
        base.update(kw)
        return SimpleNamespace(**base)

    def test_refusal_shows_pill_and_names_the_heater_port(self):
        rt = self.ctrl.zone_runtime("hotend")
        rt.refused = "M104 S37 → Error:heater does not exist"
        self.card.update_state(self._report(), rt)
        self.assertEqual("REFUSED", self.card._pill.text())
        self.assertFalse(self.card._refused_lbl.isHidden())
        text = self.card._refused_lbl.text()
        self.assertIn("M104 S37", text)
        self.assertIn("Error:heater does not exist", text)
        self.assertIn("HE0", text,
                      "the message must name the physical heater output — "
                      "'is my heater on the right port' is the operator's "
                      "actual question")

    def test_cleared_refusal_hides_the_label_again(self):
        rt = self.ctrl.zone_runtime("hotend")
        rt.refused = "M104 S37 → Error:x"
        self.card.update_state(self._report(), rt)
        rt.refused = ""
        self.card.update_state(self._report(), rt)
        self.assertTrue(self.card._refused_lbl.isHidden())
        self.assertNotEqual("REFUSED", self.card._pill.text())

    def test_sensor_fault_outranks_refusal_on_the_pill(self):
        rt = self.ctrl.zone_runtime("hotend")
        rt.refused = "M104 S37 → Error:x"
        rt.sensor_ok = False
        self.card.update_state(self._report(), rt)
        self.assertEqual("SENSOR FAULT", self.card._pill.text())
        # ...but the refusal text stays readable.
        self.assertFalse(self.card._refused_lbl.isHidden())


class TestIncubatorJogPanelSection(unittest.TestCase):
    """The 🌡️ Incubator readout on the Custom jog panel: registered,
    peek-only (it must NEVER construct or connect a session — session policy
    lives with the Incubator page), honest when there is no session."""

    def setUp(self):
        self.tmp = _isolate_env()
        _fresh_singletons()
        self.app = _app()

    def tearDown(self):
        _fresh_singletons()

    def _build(self):
        from gui.widgets.context_sections import (
            SectionContext, build_section,
        )
        return build_section("incubator", SectionContext())

    def test_registered_in_the_catalog(self):
        from gui.widgets.context_sections import catalog, known_types
        self.assertIn("incubator", known_types())
        entry = [c for c in catalog() if c[0] == "incubator"][0]
        self.assertIn("Incubator", entry[1])

    def test_no_session_shows_the_pointer_not_blank(self):
        w = self._build()
        w._last_render = 0.0
        w.on_status_update()
        self.assertFalse(w._hint.isHidden())
        self.assertIn("Workflows", w._hint.text())
        self.assertIn("Incubator", w._hint.text())
        w.deleteLater()

    def test_the_section_never_starts_a_session(self):
        """Peek-only, pinned: ticking the readout with no session must not
        construct the singleton or connect anything (the same rule the
        Connect-card tests pin — one home for session policy)."""
        import SupportClasses.incubator.service as service
        calls = []
        orig_get = service.get_incubator
        orig_conn = service.connect_from_store
        service.get_incubator = lambda *a, **k: calls.append("get")
        service.connect_from_store = lambda *a, **k: calls.append("connect")
        try:
            w = self._build()
            for _ in range(3):
                w._last_render = 0.0
                w.on_status_update()
            w.deleteLater()
        finally:
            service.get_incubator = orig_get
            service.connect_from_store = orig_conn
        self.assertEqual([], calls,
                         "the readout must never construct or connect a "
                         "session")

    def _live_ctrl(self):
        from SupportClasses.incubator.controller import IncubatorController
        ctrl = IncubatorController()
        self.assertTrue(ctrl.connect(simulate=True, sim_time_scale=600))
        self.addCleanup(ctrl.disconnect)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if ctrl.hub.marlin_channel("B") is not None:
                break
            time.sleep(0.05)
        self.assertIsNotNone(ctrl.hub.marlin_channel("B"),
                             "simulator never produced a bed channel")
        return ctrl

    def test_live_session_renders_temp_target_and_state(self):
        import SupportClasses.incubator.service as service
        ctrl = self._live_ctrl()
        ctrl.set_target("bed", 37.0)
        orig = service.peek_incubator
        service.peek_incubator = lambda: ctrl
        try:
            w = self._build()
            w._last_render = 0.0
            w.on_status_update()
            row, _name, value, state = w._rows["bed"]
            self.assertFalse(row.isHidden())
            self.assertIn("→ 37", value.text())
            self.assertIn(state.text(),
                          ("heating", "cooling", "at target", "stale"))
            w.deleteLater()
        finally:
            service.peek_incubator = orig

    def test_refused_zone_reads_REFUSED_in_red(self):
        import SupportClasses.incubator.service as service
        ctrl = self._live_ctrl()
        ctrl.zone_runtime("bed").refused = "M140 S37 → Error:x"
        orig = service.peek_incubator
        service.peek_incubator = lambda: ctrl
        try:
            w = self._build()
            w._last_render = 0.0
            w.on_status_update()
            _row, _name, _value, state = w._rows["bed"]
            self.assertEqual("REFUSED", state.text())
            w.deleteLater()
        finally:
            service.peek_incubator = orig

    def test_a_store_disabled_zone_row_is_hidden(self):
        """This rig's config disables Zone B (no usable heater on HE0) —
        the readout must respect that, not offer a phantom zone."""
        import SupportClasses.incubator.service as service
        from SupportClasses.incubator.config_store import get_store
        get_store().set_zone("hotend", enabled=False)
        ctrl = self._live_ctrl()
        orig = service.peek_incubator
        service.peek_incubator = lambda: ctrl
        try:
            w = self._build()
            w._last_render = 0.0
            w.on_status_update()
            self.assertTrue(w._rows["hotend"][0].isHidden())
            self.assertFalse(w._rows["bed"][0].isHidden())
            w.deleteLater()
        finally:
            service.peek_incubator = orig


if __name__ == "__main__":
    unittest.main()
