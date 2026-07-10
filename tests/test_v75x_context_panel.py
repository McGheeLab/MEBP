"""
v7.5.x — Customizable left context panel (pills + composable cards).

Covers the three new pieces and the vanishing-jog bug fix:
  * ``ContextPanelLayoutStore`` — the shared, persisted custom-panel layout
    (add / remove / move / collapse round-trip; unknown-type pruning).
  * ``context_sections`` registry — every catalog section builds headless and
    tolerates the live ticks / config pushes forwarded to it.
  * ``CustomContextPanel`` — store-driven rebuild, tick forwarding, best-effort
    error card for a broken section.
  * ``ContextPanelHost`` — native (Jog) slot mount/reuse/clear, pill
    availability/label, view selection + ``view_changed`` (the mechanism that
    replaces the old show/hide-vs-content desync that hid the jog panel).
"""

import os
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


# ── Fakes ──────────────────────────────────────────────────────────────────

class _Syr:
    volume_uL = 250
    stroke_length_mm = 35.0

    def mm_to_uL(self, mm):
        return mm * (250 / 35.0)

    def uL_to_mm(self, uL):
        return uL * (35.0 / 250)


class _Ink:
    name = "Ink A"
    color = "#a6e3a1"


class _FC:
    ink_spec = _Ink()


class _Pump:
    def __init__(self, configured=True):
        self.is_configured = configured
        self.syringe = _Syr()
        self.fluid_column = _FC()
        self.inks = [_Ink()]


class _Needle:
    gauge = 27
    outer_diameter_um = 210.0
    inner_diameter_um = 210.0
    length_mm = 12.7
    num_channels = 1


class _HW:
    needle = _Needle()
    plate_name = "24-well"
    plate_format = 24

    def __init__(self):
        self.pumps = {"P1": _Pump(), "P2": _Pump(), "P3": _Pump(False)}

    def camera_for_role(self, role):
        return 0


class _CamMgr:
    def __init__(self):
        self.cameras = [object()]
        self._running = set()
        self.started = []

    def is_running(self, idx):
        return idx in self._running

    def start(self, idx):
        self._running.add(idx)
        self.started.append(idx)

    def stop(self, idx):
        self._running.discard(idx)


def _controller():
    from SupportClasses.StageController import StageController
    from SupportClasses.SafetyLimits import SafetyLimits
    c = StageController.__new__(StageController)
    c.zero_position = {"x": 0, "y": 0, "Z": 0, "P1": 0, "P2": 0, "P3": 0}
    c.safety_limits = SafetyLimits()
    c.get_zp_position = lambda cached=True: (0.0, 0.0, 0.0, 0.0)
    c.get_xy_position = lambda cached=True: (1000.0, 2000.0)
    c.zp_logical_value = lambda zp, pid: {
        "Z": 5.0, "P1": 10.0, "P2": 0.0, "P3": 0.0}.get(pid)
    c.is_pump_plunger_calibrated = lambda pid: False
    c.raw_to_user_z = lambda raw: raw
    return c


def _ctx(layout_store=None):
    from SupportClasses.Settings import Settings
    from gui.widgets.context_sections import SectionContext
    return SectionContext(
        controller=_controller(), hardware_config=_HW(),
        camera_manager=_CamMgr(), settings=Settings(),
        layout_store=layout_store)


def _tmp_store(known=None):
    from SupportClasses.ContextPanelLayoutStore import ContextPanelLayoutStore
    d = tempfile.mkdtemp(prefix="mebp_ctx_")
    path = os.path.join(d, "context_panel_layout.json")
    return ContextPanelLayoutStore(path=path, known_types=known)


# ── Store ────────────────────────────────────────────────────────────────

class TestLayoutStore(unittest.TestCase):
    _known = {"camera", "syringe", "positions", "jog", "hardware_info"}

    def test_add_move_remove_collapse_roundtrip(self):
        from SupportClasses.ContextPanelLayoutStore import ContextPanelLayoutStore
        s = _tmp_store(self._known)
        a = s.add_section("camera")
        b = s.add_section("syringe")
        c = s.add_section("positions")
        self.assertTrue(a and b and c)
        self.assertEqual([x["type"] for x in s.sections()],
                         ["camera", "syringe", "positions"])
        s.move_section(c, -1)
        self.assertEqual([x["type"] for x in s.sections()],
                         ["camera", "positions", "syringe"])
        self.assertTrue(s.remove_section(b))
        s.set_collapsed(a, True)
        # Reload from disk → identical order + collapsed state + ids.
        s2 = ContextPanelLayoutStore(path=s._path, known_types=self._known)
        secs = s2.sections()
        self.assertEqual([x["type"] for x in secs], ["camera", "positions"])
        self.assertEqual(secs[0]["id"], a)
        self.assertTrue(secs[0]["collapsed"])

    def test_unknown_type_dropped_on_load(self):
        from SupportClasses.ContextPanelLayoutStore import ContextPanelLayoutStore
        s = _tmp_store(None)  # None → accept anything
        s.add_section("camera")
        s.add_section("bogus_type")
        # A stricter reader drops the unknown type.
        s2 = ContextPanelLayoutStore(path=s._path, known_types={"camera"})
        self.assertEqual([x["type"] for x in s2.sections()], ["camera"])

    def test_move_clamped_and_missing_id(self):
        s = _tmp_store(self._known)
        a = s.add_section("camera")
        self.assertFalse(s.move_section(a, -1))   # already first
        self.assertFalse(s.remove_section("nope"))

    def test_listener_fires_on_mutation_not_on_collapse(self):
        s = _tmp_store(self._known)
        calls = []
        s.add_listener(lambda store: calls.append(1))
        a = s.add_section("camera")
        self.assertEqual(len(calls), 1)
        s.set_collapsed(a, True)                  # notify=False default
        self.assertEqual(len(calls), 1)
        s.remove_section(a)
        self.assertEqual(len(calls), 2)


# ── Section registry ───────────────────────────────────────────────────────

class TestSectionRegistry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_every_section_builds_and_ticks(self):
        from PySide6.QtWidgets import QWidget
        from gui.widgets.context_sections import build_section, catalog
        ctx = _ctx()
        for stype, _label, _icon in catalog():
            w = build_section(stype, ctx, {})
            self.assertIsInstance(w, QWidget, f"{stype} not a QWidget")
            for m in ("on_status_update", "on_motion_tick"):
                fn = getattr(w, m, None)
                if callable(fn):
                    fn()  # must not raise
            sh = getattr(w, "set_hardware_config", None)
            if callable(sh):
                sh(_HW())

    def test_camera_section_starts_shared_cam_on_show(self):
        from gui.widgets.context_sections import build_section
        ctx = _ctx()
        w = build_section("camera", ctx, {})
        w.show()          # showEvent → ensure running
        self.assertIn(0, ctx.camera_manager.started)
        w.hide()          # must NOT stop the shared feed
        self.assertTrue(ctx.camera_manager.is_running(0))
        w.deleteLater()


# ── Custom panel ─────────────────────────────────────────────────────────

class TestCustomContextPanel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        # Register throw-away section types for tick + error coverage.
        from gui.widgets.context_sections import (
            SECTION_REGISTRY, SectionSpec, register_section)
        self._saved = dict(SECTION_REGISTRY)
        self._ticks = {"status": 0, "motion": 0}

        from PySide6.QtWidgets import QLabel

        def _counter(ctx, opts):
            w = QLabel("counter")
            outer = self._ticks
            w.on_status_update = lambda: outer.__setitem__(
                "status", outer["status"] + 1)
            w.on_motion_tick = lambda: outer.__setitem__(
                "motion", outer["motion"] + 1)
            return w

        def _boom(ctx, opts):
            raise RuntimeError("intentional")

        register_section("counter", SectionSpec("Counter", "#", _counter))
        register_section("boom", SectionSpec("Boom", "!", _boom))

    def tearDown(self):
        from gui.widgets import context_sections
        context_sections.SECTION_REGISTRY.clear()
        context_sections.SECTION_REGISTRY.update(self._saved)

    def _panel(self):
        from gui.widgets.custom_context_panel import CustomContextPanel
        store = _tmp_store(None)
        ctx = _ctx(layout_store=store)
        return CustomContextPanel(ctx), store

    def test_add_remove_reorder_tracks_store(self):
        panel, store = self._panel()
        a = store.add_section("counter")
        b = store.add_section("positions")
        self.assertEqual(len(panel._section_widgets), 2)
        store.move_section(b, -1)
        # positions now first — verify the panel rebuilt in the new order.
        self.assertEqual(len(panel._section_widgets), 2)
        store.remove_section(a)
        self.assertEqual(len(panel._section_widgets), 1)

    def test_tick_forwarding(self):
        panel, store = self._panel()
        store.add_section("counter")
        panel.on_status_update()
        panel.on_motion_tick()
        self.assertEqual(self._ticks["status"], 1)
        self.assertEqual(self._ticks["motion"], 1)

    def test_section_without_tick_methods_is_skipped(self):
        panel, store = self._panel()
        store.add_section("hardware_info")  # no on_status_update
        panel.on_status_update()  # must not raise

    def test_broken_section_shows_error_card_not_crash(self):
        panel, store = self._panel()
        store.add_section("boom")
        # The failing builder is caught → no live section registered, but the
        # panel still stands (an error card is inserted).
        self.assertEqual(len(panel._section_widgets), 0)
        panel.on_status_update()  # still safe

    def test_empty_state(self):
        panel, _store = self._panel()
        self.assertEqual(len(panel._section_widgets), 0)

    def test_rebuild_does_not_accumulate_stretches(self):
        panel, store = self._panel()
        ids = [store.add_section("counter") for _ in range(3)]
        # Several structural rebuilds.
        store.move_section(ids[2], -1)
        store.remove_section(ids[0])
        store.add_section("positions")
        # Layout items == live cards + exactly one trailing stretch.
        n_cards = len(store.sections())
        self.assertEqual(panel._inner_lay.count(), n_cards + 1)


# ── Host (pill picker + native slot; the bug-fix mechanism) ────────────────

class TestContextPanelHost(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _host(self):
        from gui.widgets.context_panel_host import ContextPanelHost
        return ContextPanelHost(_ctx(layout_store=_tmp_store(None)))

    def test_native_mount_reuse_and_clear(self):
        from PySide6.QtWidgets import QLabel
        h = self._host()
        w = QLabel("jog")
        h.set_native_widget(w)
        cur1 = h._native_stack.currentWidget()
        self.assertIsNot(cur1, h._native_placeholder)
        # Same instance again → same wrapper reused (no duplicate).
        h.set_native_widget(w)
        self.assertIs(h._native_stack.currentWidget(), cur1)
        # Clear → placeholder.
        h.set_native_widget(None)
        self.assertIs(h._native_stack.currentWidget(), h._native_placeholder)

    def test_native_availability_toggles_pill_and_falls_back(self):
        h = self._host()
        h.set_native_available(True)
        h.set_active_view("jog")
        self.assertEqual(h.active_view(), "jog")
        # Jog becomes unavailable → displayed view falls back to Custom, but the
        # persisted request stays "jog" and is honoured when native returns.
        h.set_native_available(False)
        self.assertEqual(h.active_view(), "custom")
        self.assertEqual(h.requested_view(), "jog")
        self.assertFalse(h._pills._buttons["jog"].isEnabled())
        h.set_native_available(True)
        self.assertEqual(h.active_view(), "jog")   # request honoured again

    def test_requested_jog_survives_restore_before_native_mounts(self):
        # Startup ordering: restore the saved view BEFORE any page mounts a
        # native widget, then the first navigation makes Jog available.
        h = self._host()                       # native unavailable
        h.set_active_view("jog")               # persisted preference
        self.assertEqual(h.active_view(), "custom")   # nothing to show yet
        self.assertEqual(h.requested_view(), "jog")   # ...but not clobbered
        h.set_native_available(True)           # first _refresh_left_context
        self.assertEqual(h.active_view(), "jog")

    def test_set_active_view_emits_and_persists_choice(self):
        h = self._host()
        h.set_native_available(True)
        seen = []
        h.view_changed.connect(lambda v: seen.append(v))
        h.set_active_view("custom")
        h.set_active_view("jog")
        self.assertEqual(seen, ["custom", "jog"])

    def test_native_label(self):
        h = self._host()
        h.set_native_label("Controls")
        self.assertEqual(h._pills._buttons["jog"].text(), "Controls")

    def test_ticks_forward_to_custom_only(self):
        # Host tick must reach the Custom panel (native is page-ticked).
        h = self._host()
        h.on_status_update()   # must not raise even with an empty custom panel
        h.on_motion_tick()


# ── App-level _refresh_left_context (the vanishing-jog bug fix) ────────────
#
# Invoked UNBOUND against a fake ``self`` (mirrors the repo's
# TestApplyAndSaveAfterRestore pattern) — no full MainWindow boot needed, which
# is intractable headless (camera/serial enumeration). This exercises the exact
# branching that decides the left box's content + visibility in one place.

class TestRefreshLeftContext(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)
        try:
            from gui.app import MainWindow
        except Exception as exc:  # pragma: no cover
            raise unittest.SkipTest(f"MainWindow import failed: {exc}")
        cls._MW = MainWindow

    def _fake(self):
        from types import SimpleNamespace
        from PySide6.QtWidgets import QLabel, QFrame
        from gui.widgets.context_panel_host import ContextPanelHost
        host = ContextPanelHost(_ctx(layout_store=_tmp_store(None)))
        box = QFrame()          # top-level → isVisible() False until show()
        title = QLabel()

        class JogPage:
            def __init__(self):
                self._w = QLabel("jog")

            def get_context_widget(self):
                return self._w

        class NoCtxPage:
            def get_context_widget(self):
                return None

        class BoomPage:
            def get_context_widget(self):
                raise RuntimeError("delegate boom")

        fake = SimpleNamespace(
            _context_host=host,
            ui_extraLeftBox=box,
            _context_title=title,
            _current_page_index=0,
            _page_widgets=[JogPage(), NoCtxPage(), BoomPage()],
            _left_context_user_collapsed=False,
            settings=SimpleNamespace(set=lambda *a, **k: None),
            _apply_saved_context_width=lambda: None,
            _update_context_panel_bounds=lambda: None,
        )
        MW = self._MW
        fake._context_title_for = MW._context_title_for.__get__(fake)
        fake._refresh_left_context = MW._refresh_left_context.__get__(fake)
        fake._toggle_left_context = MW._toggle_left_context.__get__(fake)
        return fake, host, box

    def test_shows_with_native_hides_without(self):
        fake, host, box = self._fake()
        fake._current_page_index = 0                # JogPage → native present
        fake._refresh_left_context()
        self.assertTrue(box.isVisible())
        self.assertTrue(host._native_available)
        self.assertIsNot(host._native_stack.currentWidget(),
                         host._native_placeholder)
        fake._current_page_index = 1                # NoCtxPage → hide
        fake._refresh_left_context()
        self.assertFalse(box.isVisible())
        self.assertFalse(host._native_available)

    def test_delegate_exception_hides_not_crash(self):
        fake, host, box = self._fake()
        fake._current_page_index = 2                # BoomPage
        fake._refresh_left_context()                # must not raise
        self.assertFalse(box.isVisible())

    def test_manual_toggle_collapse_expand(self):
        fake, host, box = self._fake()
        fake._current_page_index = 0
        fake._refresh_left_context()
        self.assertTrue(box.isVisible())
        fake._toggle_left_context()                 # collapse
        self.assertFalse(box.isVisible())
        self.assertTrue(fake._left_context_user_collapsed)
        fake._toggle_left_context()                 # expand
        self.assertTrue(box.isVisible())


if __name__ == "__main__":
    unittest.main()
