"""
v7.19 — the Fluorescence Mosaic workflow's own left context panel.

Operator: *"lets make all of these settings on the left context menu for the
fluorescence mosaic workflow. this is a special left context panel just for this
workflow page."*

What matters here, and why each is a test rather than a look:

* The left box only appears for a page that RETURNS a context widget, and the
  fluorescence page returned None for its whole life. If ``get_context_widget``
  regresses to None the panel silently vanishes with no error anywhere.
* An EMBEDDED instance must return None. Its host owns the left box with its own
  jog panel, and handing this panel over would either steal that box or mount
  one widget in two places.
* The native pill is labelled from a map keyed on the PAGE CLASS, which for
  every workflow is ``WorkflowsModePage`` — so without the ``context_label``
  delegate these controls would be labelled "Jog".
* The box goes down to s(100) and disables horizontal scrolling, so a panel that
  refuses to shrink clips its own controls.
"""

from __future__ import annotations

import ast
import inspect
import textwrap
import os
import tempfile
import unittest

try:
    from PySide6.QtWidgets import QApplication, QScrollArea
    _QT = True
except Exception:                                            # pragma: no cover
    _QT = False

_APP = None


def _ensure_app():
    global _APP
    if _APP is None:
        _APP = QApplication.instance() or QApplication([])
    return _APP


class _StubPage:
    """The whole page surface the panel touches (``_PAGE_API``)."""

    def __init__(self, active="FITC", refusals=None, scanning=False):
        self.applied = []
        self.recipes = {}
        self.activated = []
        self._active = active
        self._refusals = refusals or {}
        self._scanning = scanning

    def channels(self):
        return ("DAPI", "FITC", "Cy5")

    def channel_recipe(self, ch):
        return self.recipes.get(ch, {})

    def set_channel_recipe(self, ch, key, value):
        self.recipes.setdefault(ch, {})[key] = value

    def camera_manager_for_panel(self):
        return None

    def microscope_cam_idx(self):
        return 0

    def panel_optics_state(self):
        return {"active_channel": self._active, "objective": "",
                "cube_refusals": self._refusals, "note": ""}

    def is_scanning(self):
        return self._scanning

    def apply_panel_control(self, key, value):
        self.applied.append((key, value))
        return value

    def on_panel_activate_channel(self, ch):
        self.activated.append(ch)

    def on_panel_objective_detected(self, name):
        self.detected = name


def _panel(**kw):
    from gui.widgets.fluorescence_controls_panel import (
        FluorescenceControlsPanel)
    page = _StubPage(**kw)
    p = FluorescenceControlsPanel(page)
    p.refresh_ranges()
    p.load_recipes()
    p.refresh_optics()
    return page, p


def _page(embedded=False):
    os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())

    class SL:
        xy_min_x = 0.0
        xy_min_y = 0.0
        xy_max_x = 120000.0
        xy_max_y = 80000.0

    class Ctrl:
        safety_limits = SL()
        is_zp_connected = False
        zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

    from gui.pages.workflows.fluorescence_mosaic_workflow import (
        FluorescenceMosaicWorkflowPage)
    return FluorescenceMosaicWorkflowPage(
        controller=Ctrl(), settings=object(), camera_manager=None,
        embedded=embedded)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestThePageOffersThePanel(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_standalone_page_returns_the_panel(self):
        pg = _page()
        self.assertIs(pg.get_context_widget(), pg._panel)

    def test_embedded_page_returns_none(self):
        """Its host owns the left box — see the class docstring."""
        pg = _page(embedded=True)
        self.assertIsNone(pg.get_context_widget())

    def test_embedded_page_still_shows_the_same_panel_inline(self):
        """One class, two mount points: the embedded case must not lose the
        controls, and must not build a SECOND set of them."""
        pg = _page(embedded=True)
        self.assertIsNotNone(pg._panel)
        self.assertIs(pg._panel.parent(), pg._panel.parentWidget())
        # The pills the page reasons about are the panel's, in both cases.
        self.assertIs(pg._channel_checks, pg._panel.pills())

    def test_context_label_is_not_jog(self):
        self.assertEqual(_page().context_label(), "Signal")

    def test_pills_and_objective_combo_are_the_panel_s(self):
        pg = _page()
        self.assertIs(pg._channel_checks, pg._panel.pills())
        self.assertIs(pg._objective_combo, pg._panel.objective_combo())

    def test_one_pill_class_not_two(self):
        """The page re-exports the panel's class; a second definition would be
        a silently diverging idea of "a filter cube in the UI"."""
        from gui.pages.workflows import fluorescence_mosaic_workflow as page_mod
        from gui.widgets import fluorescence_controls_panel as panel_mod
        self.assertIs(page_mod._ChannelPill, panel_mod._ChannelPill)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheHostCanLabelIt(unittest.TestCase):
    """The delegate chain page -> WorkflowsModePage -> MainWindow."""

    def setUp(self):
        _ensure_app()

    def test_workflows_mode_delegates_context_label(self):
        from gui.pages.workflows_mode import WorkflowsModePage
        src = inspect.getsource(WorkflowsModePage.context_label)
        tree = ast.parse(textwrap.dedent(src))
        names = {getattr(n.func, "attr", "") for n in ast.walk(tree)
                 if isinstance(n, ast.Call)}
        self.assertIn("context_label", names)

    def test_main_window_asks_the_page_before_its_class_map(self):
        """A substring check would pass on a comment — walk the AST instead.

        MainWindow's map is keyed on the page CLASS and every workflow shares
        one class, so if this call is dropped the signal panel is labelled
        "Jog" and nothing fails.
        """
        from gui.app import MainWindow
        src = inspect.getsource(MainWindow._refresh_left_context)
        tree = ast.parse(textwrap.dedent(src))
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        self.assertTrue(
            any(getattr(c.func, "attr", "") == "context_label" for c in calls),
            "_refresh_left_context must consult page.context_label()")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestItSurvivesTheNarrowBox(unittest.TestCase):
    """The context box shrinks to s(100) and disables horizontal scrolling."""

    def setUp(self):
        _ensure_app()

    def test_no_horizontal_scrollbar_at_any_width(self):
        app = _ensure_app()
        _pg, p = _panel()
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setWidget(p)
        sa.show()
        for w in (340, 200, 130, 100):
            sa.resize(w, 800)
            app.processEvents()
            self.assertFalse(
                sa.horizontalScrollBar().isVisible(),
                f"panel forced a horizontal scrollbar at host width {w}")

    def test_it_goes_compact_when_narrow(self):
        app = _ensure_app()
        _pg, p = _panel()
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setWidget(p)
        sa.show()
        sa.resize(340, 800)
        app.processEvents()
        self.assertFalse(p._compact)
        sa.resize(120, 800)
        app.processEvents()
        self.assertTrue(p._compact)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestOnlyTheFittedCubeIsEditable(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def test_one_strip_named_for_the_cube_it_writes_to(self):
        """v7.19.2 — one slider set, applied to the active filter.

        Per-cube exposure sliders meant N-1 permanently disabled sliders, and
        the four controls you reach for while judging a histogram were behind a
        disclosure. What replaces "which slider is enabled" as the guard against
        editing the wrong cube is that the strip NAMES the cube it writes to.
        """
        _pg, p = _panel(active="FITC")
        self.assertIn("FITC", p._signal_hdr.text())
        for key in ("exposure_us", "gain_pct", "avg_frames",
                    "display_lo", "display_hi"):
            self.assertTrue(p._sliders[key].isEnabled(), key)

    def test_with_no_cube_in_the_path_nothing_is_editable(self):
        _pg, p = _panel(active=None)
        self.assertIn("no cube", p._signal_hdr.text())
        self.assertFalse(p._sliders["exposure_us"].isEnabled())
        self.assertFalse(p._auto_btn.isEnabled())

    def test_an_inactive_channel_can_never_drive_the_camera(self):
        """Setting "Cy5's exposure" while DAPI is fitted would change the live
        image and record the value against the wrong cube."""
        page, p = _panel(active="FITC")
        self.assertIsNone(p._apply_control("DAPI", "exposure_us", 1234.0))
        self.assertEqual(page.applied, [])

    def test_a_scan_disables_every_control(self):
        _pg, p = _panel(active="FITC", scanning=True)
        self.assertFalse(p._sliders["exposure_us"].isEnabled())
        self.assertFalse(p._objective_combo.isEnabled())
        self.assertFalse(p._order_combo.isEnabled())
        self.assertFalse(p._preset_btn.isEnabled())

    def test_an_unresolvable_cube_cannot_be_activated_and_says_why(self):
        _pg, p = _panel(active="FITC", refusals={"Cy5": "slot 5 is empty"})
        self.assertFalse(p._activate["Cy5"].isEnabled())
        self.assertIn("empty", p._activate["Cy5"].toolTip())
        self.assertTrue(p._activate["DAPI"].isEnabled())

    def test_the_active_channel_follows_the_body_not_a_ui_flag(self):
        """The operator can turn the cassette by hand."""
        page, p = _panel(active="FITC")
        self.assertEqual(p._active, "FITC")
        page._active = "DAPI"
        p.refresh_optics()
        self.assertEqual(p._active, "DAPI")
        self.assertIn("DAPI", p._signal_hdr.text())


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheHighlightBox(unittest.TestCase):
    """v7.19.2, operator: *"it should have a highligh box around the current
    filter on the signal pannel"*.

    Every slider writes to the active channel's recipe, so mistaking which one
    is active records an exposure against the wrong cube. The green ``◉`` alone
    was too quiet for that.
    """

    def setUp(self):
        _ensure_app()

    def test_the_live_cube_is_boxed_and_the_others_are_not(self):
        _pg, p = _panel(active="FITC")
        self.assertTrue(p._pills["FITC"].property("liveCube"))
        self.assertIn("border", p._pills["FITC"].styleSheet())
        for ch in ("DAPI", "Cy5"):
            self.assertFalse(p._pills[ch].property("liveCube"), ch)
            self.assertNotIn("border", p._pills[ch].styleSheet(), ch)

    def test_the_box_moves_with_the_cassette(self):
        page, p = _panel(active="FITC")
        page._active = "Cy5"
        p.refresh_optics()
        self.assertTrue(p._pills["Cy5"].property("liveCube"))
        self.assertFalse(p._pills["FITC"].property("liveCube"))

    def test_every_signal_control_is_visible_without_a_disclosure(self):
        """The five controls are one strip now — no "More signal" to open."""
        _pg, p = _panel(active="FITC")
        self.assertFalse(hasattr(p, "_more"))
        self.assertEqual(
            sorted(p._sliders),
            sorted(["avg_frames", "display_hi", "display_lo",
                    "exposure_us", "gain_pct"]))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestThePanelPollDoesNotFightTheOperator(unittest.TestCase):
    """v7.19.1 — two things the audit found this panel was missing.

    ``microscope_panel`` has guarded both since v7.5.x; this panel, written for
    v7.19, copied neither.
    """

    def setUp(self):
        _ensure_app()

    def test_an_open_dropdown_is_left_alone(self):
        """Re-indexing a combo whose list is open moves the highlight under the
        operator's cursor mid-selection."""
        page, panel = _panel()
        page.panel_optics_state = lambda: {
            "active_channel": "FITC", "objective": "20x",
            "cube_refusals": {}, "note": ""}
        panel._objective_combo.addItems(["4x", "20x"])
        panel._objective_combo.setCurrentIndex(0)
        panel._objective_combo.showPopup()
        self.assertTrue(panel._objective_combo.view().isVisible())

        panel.refresh_optics()
        self.assertEqual(panel._objective_combo.currentText(), "4x",
                         "the poll re-indexed an open drop-down")

        panel._objective_combo.hidePopup()
        panel.refresh_optics()
        self.assertEqual(panel._objective_combo.currentText(), "20x",
                         "a closed combo stopped tracking the body")

    def test_a_detected_objective_is_reported_to_the_page(self):
        """v7.19.2 — showing the new objective is not enough. µm/px is keyed by
        NAME, so a detected change the page never hears about leaves every tile
        measured with the previous objective's scale."""
        page, panel = _panel()
        page.detected = None
        page.panel_optics_state = lambda: {
            "active_channel": "FITC", "objective": "20x",
            "cube_refusals": {}, "note": ""}
        panel._objective_combo.addItems(["4x", "20x"])
        panel._objective_combo.setCurrentIndex(0)

        panel.refresh_optics()

        self.assertEqual(page.detected, "20x")
        self.assertEqual(panel._objective_combo.currentText(), "20x")

    def test_no_change_is_not_reported(self):
        page, panel = _panel()
        page.detected = None
        panel._objective_combo.addItems(["20x"])
        page.panel_optics_state = lambda: {
            "active_channel": "FITC", "objective": "20x",
            "cube_refusals": {}, "note": ""}
        panel.refresh_optics()
        self.assertIsNone(page.detected)

    def test_the_poll_asks_the_body_for_a_fresh_read(self):
        """Standalone, this panel IS the left box — microscope_panel is not on
        screen, so nothing else refreshes the cache it reads."""
        import gui.widgets.optics_ensure as oe
        _page_stub, panel = _panel()
        asked = []
        real = oe.request_state_refresh
        oe.request_state_refresh = lambda *a, **k: asked.append(1)
        try:
            panel.refresh_optics()
        finally:
            oe.request_state_refresh = real
        self.assertTrue(asked, "the panel polled a cache nothing refreshes")


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheStripFollowsTheRunningMosaic(unittest.TestCase):
    """v7.19.2, operator: *"as we build the mosic these signals automatically
    update"*.

    In tile-major the worker changes the cube at every tile. The panel reads the
    ACTIVE channel from ``filter_position``, so it follows for free — but only
    because ``OpticsService`` refreshes the cache around its own switches. The
    shared freshener deliberately refuses under a lease, and a scan holds one,
    so without that the strip would freeze for the whole run.
    """

    def setUp(self):
        _ensure_app()

    def test_the_box_and_the_values_track_the_channel_being_captured(self):
        page, p = _panel(active="DAPI", scanning=True)
        page.recipes = {"DAPI": {"exposure_us": 20_000.0},
                        "Cy5": {"exposure_us": 800_000.0}}
        p.load_recipes()          # what showEvent does
        self.assertAlmostEqual(p._sliders["exposure_us"].value(),
                               20_000.0, delta=500.0)

        page._active = "Cy5"                       # the worker moved the cube
        p.refresh_optics()

        self.assertTrue(p._pills["Cy5"].property("liveCube"))
        self.assertIn("Cy5", p._signal_hdr.text())
        self.assertAlmostEqual(p._sliders["exposure_us"].value(),
                               800_000.0, delta=20_000.0)

    def test_but_they_stay_READ_ONLY_while_the_scan_runs(self):
        """Following the run is a readout. Editing mid-run would change the
        camera under a raster already planned around the old value."""
        _pg, p = _panel(active="Cy5", scanning=True)
        for key in ("exposure_us", "gain_pct", "avg_frames",
                    "display_lo", "display_hi"):
            self.assertFalse(p._sliders[key].isEnabled(), key)
        self.assertFalse(p._auto_btn.isEnabled())


if __name__ == "__main__":
    unittest.main()
