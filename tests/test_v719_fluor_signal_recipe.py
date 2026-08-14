"""
v7.19 — the per-channel signal recipe and the sliders that edit it.

Operator: *"under each filter cube i want an exposure slider to set the camera,
the filter cube that is currently active is the only slider that can be set.
moving the slider changes the cameras active value"* and, when asked whether
exposure alone is enough: *"is exposure enough to make a good image, should we
also allow a way to open other options as sliders"* → exposure · analog gain ·
frame averaging · display black/white, remembered PER CUBE.

The two properties worth defending:

* **The readout must show what the camera RAN, not what was asked for.** A
  camera that clamps a request and a UI that keeps showing the request is
  exactly what the v7.13 Zyla bench report ("exposure resets to a small
  number") looked like from the operator's side.
* **The slider and the settings popout are two views of ONE value.** A second
  store would drift, and the popout is what gives the recipe profiles,
  import/export and last-used for free.
"""

from __future__ import annotations

import math
import os
import tempfile
import unittest

try:
    from PySide6.QtWidgets import QApplication, QDoubleSpinBox, QSpinBox
    _QT = True
except Exception:                                            # pragma: no cover
    _QT = False

from gui.widgets.signal_slider import LINEAR, LOG, pos_to_value, value_to_pos

_APP = None


def _ensure_app():
    global _APP
    if _APP is None:
        _APP = QApplication.instance() or QApplication([])
    return _APP


class TestTheLogScale(unittest.TestCase):
    """Exposure spans nine decades; a linear track cannot express it."""

    LO, HI = 6.3, 5.76e9        # the Libra 25's declared range, in µs

    def test_the_useful_band_is_reachable(self):
        """1–1000 ms must occupy a usable stretch of the track. On a LINEAR
        slider the whole band sits inside the first 0.02 % of it.
        """
        p1 = value_to_pos(1_000.0, self.LO, self.HI, LOG)
        p1000 = value_to_pos(1_000_000.0, self.LO, self.HI, LOG)
        self.assertGreater(p1000 - p1, 200)
        lin1 = value_to_pos(1_000.0, self.LO, self.HI, LINEAR)
        lin1000 = value_to_pos(1_000_000.0, self.LO, self.HI, LINEAR)
        self.assertLess(lin1000 - lin1, 2)      # the naive answer, unusable

    def test_round_trip_is_close(self):
        for v in (10.0, 1_000.0, 20_000.0, 500_000.0, 5.0e8):
            back = pos_to_value(
                value_to_pos(v, self.LO, self.HI, LOG), self.LO, self.HI, LOG)
            self.assertLess(abs(math.log(back / v)), 0.02)

    def test_ends_are_exact(self):
        for mode in (LOG, LINEAR):
            self.assertEqual(
                pos_to_value(value_to_pos(self.LO, self.LO, self.HI, mode),
                             self.LO, self.HI, mode), self.LO)
            self.assertAlmostEqual(
                pos_to_value(value_to_pos(self.HI, self.LO, self.HI, mode),
                             self.LO, self.HI, mode), self.HI, places=3)

    def test_it_clamps_and_never_raises(self):
        self.assertEqual(value_to_pos(-5.0, self.LO, self.HI, LOG), 0)
        self.assertEqual(value_to_pos(1e30, self.LO, self.HI, LOG), 1000)
        # Degenerate ranges must not divide by zero.
        self.assertEqual(value_to_pos(5.0, 10.0, 10.0, LOG), 0)
        self.assertEqual(pos_to_value(500, 10.0, 10.0, LOG), 10.0)
        # A zero lower bound has no logarithm — fall back to linear.
        self.assertAlmostEqual(pos_to_value(500, 0.0, 100.0, LOG), 50.0)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheReadoutTellsTheTruth(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def _slider(self, apply_fn):
        from gui.widgets.signal_slider import SignalSlider
        sl = SignalSlider("exposure", lo=50.0, hi=10_000_000.0, mode=LOG,
                          decimals=1, suffix=" ms", scale=1e-3)
        sl.set_apply(apply_fn)
        return sl

    def test_a_clamped_request_is_shown_as_the_ACHIEVED_value(self):
        seen = []

        def apply(v):
            seen.append(v)
            return 33_000.0            # the camera clamps to 33 ms

        sl = self._slider(apply)
        sl.set_value(500_000.0)        # ask for 500 ms
        sl._push()
        self.assertEqual(sl.value(), 33_000.0)
        self.assertIn("33.0", sl._readout.text())

    def test_an_unreadable_camera_leaves_the_request_shown(self):
        sl = self._slider(lambda v: None)
        sl.set_value(120_000.0)
        sl._push()
        self.assertEqual(sl.value(), 120_000.0)

    def test_set_value_never_re_enters_the_apply_path(self):
        """Correcting the display to the achieved value must not push again —
        that is an infinite loop against a clamping camera."""
        calls = []
        sl = self._slider(lambda v: (calls.append(v), 1000.0)[1])
        sl.set_value(500_000.0)
        self.assertEqual(calls, [])           # seeding is not an edit
        sl._push()
        self.assertEqual(len(calls), 1)       # exactly one write

    def test_a_failing_apply_does_not_raise(self):
        def boom(_v):
            raise RuntimeError("SDK said no")

        sl = self._slider(boom)
        sl.set_value(100_000.0)
        sl._push()                            # must not propagate

    def test_re_ranging_keeps_the_chosen_value(self):
        """The Andor's achievable exposure moves with readout rate and gain
        mode; a re-range must not silently reset what the operator picked."""
        sl = self._slider(lambda v: v)
        sl.set_value(200_000.0)
        sl.set_range(50.0, 30_000_000.0)
        self.assertEqual(sl.value(), 200_000.0)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheRecipeIsOneValueInTwoViews(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def _page(self):
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

    def test_exposure_is_stored_in_us_and_typed_in_ms(self):
        """One conversion, in one place — the camera stack is µs everywhere."""
        pg = self._page()
        pg._channel_exposure["FITC"].setValue(150.0)          # ms
        self.assertEqual(pg.channel_recipe("FITC")["exposure_us"], 150_000.0)
        pg.set_channel_recipe("FITC", "exposure_us", 33_000.0)
        self.assertAlmostEqual(pg._channel_exposure["FITC"].value(), 33.0)

    def test_writing_the_recipe_lands_on_the_popout_spin(self):
        pg = self._page()
        pg.set_channel_recipe("Cy5", "gain_pct", 42.0)
        self.assertAlmostEqual(pg._channel_gain["Cy5"].value(), 42.0)
        pg.set_channel_recipe("Cy5", "avg_frames", 8.0)
        self.assertEqual(pg._channel_avg["Cy5"].value(), 8)

    def test_integer_controls_round_rather_than_truncate(self):
        pg = self._page()
        pg.set_channel_recipe("DAPI", "display_hi", 4300.6)
        self.assertEqual(pg._channel_lo["DAPI"].value(), 0)
        self.assertEqual(pg._channel_hi["DAPI"].value(), 4301)

    def test_zero_means_not_set_and_is_omitted(self):
        """0 = "leave the camera as it is" / "use the scan default" / "measure
        it at the probe" — the convention the exposure spin has had since
        v7.13, extended to the controls added beside it.
        """
        pg = self._page()
        self.assertEqual(pg.channel_recipe("DAPI"), {})
        pg._channel_avg["DAPI"].setValue(0)
        self.assertNotIn("avg_frames", pg.channel_recipe("DAPI"))
        pg._channel_avg["DAPI"].setValue(4)
        self.assertEqual(pg.channel_recipe("DAPI")["avg_frames"], 4.0)

    def test_every_channel_keeps_its_own_recipe(self):
        pg = self._page()
        pg.set_channel_recipe("DAPI", "exposure_us", 20_000.0)
        pg.set_channel_recipe("Cy5", "exposure_us", 800_000.0)
        self.assertEqual(pg.channel_recipe("DAPI")["exposure_us"], 20_000.0)
        self.assertEqual(pg.channel_recipe("Cy5")["exposure_us"], 800_000.0)

    def test_the_panel_shows_the_ACTIVE_channels_recipe(self):
        """v7.19.2 — one slider strip, re-rendered from whichever cube is in
        the light path. Seeding "every channel's slider" is meaningless now:
        there is one, and it must follow the cassette."""
        pg = self._page()
        pg.set_channel_recipe("FITC", "exposure_us", 150_000.0)
        pg.set_channel_recipe("Cy5", "exposure_us", 800_000.0)
        panel = pg._panel

        panel._active = "FITC"
        panel._render_active_recipe()
        self.assertAlmostEqual(panel._sliders["exposure_us"].value(),
                               150_000.0, delta=3000.0)

        panel._active = "Cy5"
        panel._render_active_recipe()
        self.assertAlmostEqual(panel._sliders["exposure_us"].value(),
                               800_000.0, delta=20_000.0)

    def test_there_is_no_third_store(self):
        """The recipe must live in the popout's WorkflowSettingsStore keys —
        that is what makes it a saveable/exportable profile."""
        pg = self._page()
        keys = set(pg._settings_dialog._fields) \
            if hasattr(pg._settings_dialog, "_fields") else None
        if keys is None:
            self.skipTest("settings dialog field registry not exposed")
        for tok in ("dapi", "fitc"):
            for stem in ("exposure_ms", "gain_pct", "avg_frames",
                         "disp_lo", "disp_hi"):
                self.assertIn(f"{stem}_{tok}", keys)


@unittest.skipUnless(_QT, "PySide6 not available")
class TestApplyingAControl(unittest.TestCase):
    def setUp(self):
        _ensure_app()

    def _page(self, readback=None):
        os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())

        class Mgr:
            def __init__(self):
                self.calls = []
                self.st = readback or {}

            def get_hw_settings(self, _i):
                return dict(self.st)

            def set_hw_exposure_us(self, _i, v):
                self.calls.append(("exposure_us", v))

            def set_hw_exposure_gain(self, _i, v):
                self.calls.append(("gain", v))

            def set_hw_andor_scale_lo(self, _i, v):
                self.calls.append(("lo", v))

            def set_hw_andor_scale_hi(self, _i, v):
                self.calls.append(("hi", v))

        class SL:
            xy_min_x = xy_min_y = 0.0
            xy_max_x, xy_max_y = 120000.0, 80000.0

        class Ctrl:
            safety_limits = SL()
            is_zp_connected = False
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        pg = FluorescenceMosaicWorkflowPage(
            controller=Ctrl(), settings=object(), camera_manager=None)
        mgr = Mgr()
        pg._camera_manager = mgr
        return pg, mgr

    def test_exposure_is_pushed_and_read_back(self):
        pg, mgr = self._page(readback={"exposure_us": 33_000.0})
        got = pg.apply_panel_control("exposure_us", 500_000.0)
        self.assertEqual(mgr.calls, [("exposure_us", 500_000)])
        self.assertEqual(got, 33_000.0)      # the camera's answer, not ours

    def test_averaging_is_ours_and_touches_no_camera(self):
        """avg_frames is applied by the mosaic worker, not the sensor."""
        pg, mgr = self._page()
        self.assertEqual(pg.apply_panel_control("avg_frames", 8.0), 8.0)
        self.assertEqual(mgr.calls, [])

    def test_display_levels_reach_the_camera(self):
        pg, mgr = self._page(readback={"andor_scale_lo": 120.0})
        self.assertEqual(pg.apply_panel_control("display_lo", 120.0), 120.0)
        self.assertEqual(mgr.calls, [("lo", 120)])

    def test_an_unknown_key_is_refused(self):
        pg, mgr = self._page()
        self.assertIsNone(pg.apply_panel_control("binning", 2))
        self.assertEqual(mgr.calls, [])

    def test_no_camera_manager_is_a_quiet_no_op(self):
        pg, _mgr = self._page()
        pg._camera_manager = None
        self.assertIsNone(pg.apply_panel_control("exposure_us", 1000.0))


@unittest.skipUnless(_QT, "PySide6 not available")
class TestTheValuesSurviveARestart(unittest.TestCase):
    """🐞 v7.19.3, operator: *"the values assigned here should be persistant on
    this page"*.

    The values were being WRITTEN into the right widgets all along — every
    recipe spin is a registered settings field — but nothing ever wrote the
    FILE. ``WorkflowSettingsDialog.save_last`` is reached only from that
    dialog's own hideEvent/closeEvent, and the page hides the popout only ``if
    self._settings_dialog.isVisible()``. So the save ran only for an operator
    who opened the ⚙ popout and closed it again — and the whole point of the
    v7.19 left panel is that they never need to.
    """

    def setUp(self):
        _ensure_app()
        self._dir = tempfile.mkdtemp()
        os.environ["MEBP_WORKFLOW_SETTINGS_DIR"] = self._dir

    def _fresh_page(self):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)

        class SL:
            xy_min_x = xy_min_y = 0.0
            xy_max_x, xy_max_y = 120000.0, 80000.0

        class Ctrl:
            safety_limits = SL()
            is_zp_connected = False
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        return FluorescenceMosaicWorkflowPage(
            controller=Ctrl(), settings=object(), camera_manager=None)

    def test_a_slider_edit_reaches_disk_without_opening_the_popout(self):
        """Driven through the TIMER, never by calling the flush by hand.

        ⚠ My first version of this test called ``_flush_recipe_save()``
        directly and passed with ``set_channel_recipe`` scheduling NOTHING —
        i.e. with the reported bug still shipping. A mutation caught it. Same
        weakness that let the v7.18 setpoint keeper ship green.
        """
        pg = self._fresh_page()
        self.assertFalse(pg._settings_dialog.isVisible())

        pg.set_channel_recipe("FITC", "exposure_us", 150_000.0)
        pg.set_channel_recipe("FITC", "gain_pct", 12.0)
        self.assertTrue(pg._save_timer.isActive(),
                        "the edit scheduled no save at all")
        pg._save_timer.timeout.emit()

        pg2 = self._fresh_page()                       # "restart"
        rec = pg2.channel_recipe("FITC")
        self.assertAlmostEqual(rec["exposure_us"], 150_000.0, delta=1.0)
        self.assertAlmostEqual(rec["gain_pct"], 12.0, delta=0.01)

    def test_leaving_the_page_writes_the_pending_edit(self):
        from PySide6.QtGui import QHideEvent
        pg = self._fresh_page()
        pg.set_channel_recipe("Cy5", "exposure_us", 800_000.0)
        pg.hideEvent(QHideEvent())                     # no explicit flush

        rec = self._fresh_page().channel_recipe("Cy5")
        self.assertAlmostEqual(rec["exposure_us"], 800_000.0, delta=1.0)

    def test_the_scan_order_and_preset_survive_too(self):
        """Both were plain attributes with hardcoded defaults, so they reset on
        every launch however carefully the recipe was saved."""
        pg = self._fresh_page()
        pg._panel.set_scan_order("channel")
        pg._panel.set_preset_is_fluorescence(False)
        pg._flush_recipe_save()

        pg2 = self._fresh_page()
        self.assertEqual(pg2.scan_order(), "channel")
        self.assertFalse(pg2._panel.preset_is_fluorescence())

    def test_the_scan_order_is_read_from_the_widget_not_a_mirror(self):
        """Two ways the combo can move (an operator pick, and the restore), and
        only one of them updated the old page-level mirror."""
        pg = self._fresh_page()
        pg._scan_order = "tile"                 # stale mirror
        pg._panel.set_scan_order("channel")     # widget moved, no signal
        self.assertEqual(pg.scan_order(), "channel")

    def test_the_save_is_debounced_not_one_write_per_step(self):
        """⚡Auto and a channel switch commit several values in a burst; the
        v7.16 camera-crop incident is the recorded cost of a file write per
        widget step."""
        pg = self._fresh_page()
        writes = []
        pg._settings_dialog.save_last = lambda: writes.append(1)
        for v in (10_000.0, 20_000.0, 30_000.0, 40_000.0):
            pg.set_channel_recipe("DAPI", "exposure_us", v)
        self.assertEqual(writes, [], "wrote the file on every step")
        pg._flush_recipe_save()
        self.assertEqual(len(writes), 1)


if __name__ == "__main__":
    unittest.main()
