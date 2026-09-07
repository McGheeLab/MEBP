"""
v7.21.3 - Hardware Setup / Needle: typing a pulled-capillary dimension.

Two operator reports, one page:

1. **"under pulled glass capillary it freezes every time i input a number."**
   Root-caused from the operator's own ``logs/freeze.log`` - six GUI-thread
   stalls on 2026-08-19 between 16:21 and 16:22, EVERY one of them rooted at
   ``hardware_setup._on_needle_changed`` and stuck several frames deep inside a
   different page:

       _on_needle_changed -> _on_config_changed -> config_changed.emit
         -> MainWindow._on_hardware_config_changed
           -> _propagate_hardware_config              (every page)
             -> print_builder_sketch._refresh_sequence._seq_section_row
             -> calibration.set_hardware_config -> _emit_calibration_data_changed
                 -> workflows -> print_calibrator -> quick_print._render_readiness

   ``config_changed`` is not a local notification: the receiver writes
   settings.json AND rebuilds widget trees across the whole app. A
   ``QDoubleSpinBox`` emits ``valueChanged`` on every keystroke, so typing a
   three-digit tip diameter ran that whole cascade three times.

   Fixed at the SOURCE, in one place, because the page has ~25 call sites of
   ``_on_config_changed`` and a per-call-site fix would leave the next one to be
   written unprotected. The page's own readouts stay synchronous (measured
   ~1 ms); only the app-wide fan-out is coalesced.

2. **"it wont allow me to input the pulled tip diameter it puts a range on the
   value, i should be able to input anything i want."** The spin range was
   0.5-500 um. A spin box does not merely refuse an out-of-range number, it
   REWRITES it as you type, so the value that reaches the config is one the
   operator never entered and nothing on screen says so. Every constraint the
   range encoded is already checked by ``HardwareConfig._needle_bore_issues``
   and reported by name, so the bounds now exist only to keep the widget finite.

What these tests pin, in order of how much damage the failure would do:

* A burst of edits produces exactly ONE fan-out (the freeze), while the page's
  own readout still tracks every edit - a debounce that also froze the readout
  would trade one bug for another, and the burst test alone cannot see that.
* A pending emit is never DROPPED. The receiver persists to settings.json, so
  an edit swallowed by the debounce would be a silently lost setting - worse
  than the freeze. Both exits flush: leaving the page, and quitting.
* A value outside the old range survives verbatim into the config and is
  REPORTED, not clamped.
* Typed text commits once, not per digit - so the live needle never passes
  through "3 um" on its way to 300.

WARNING: ``QMessageBox`` blocks forever under offscreen Qt, so nothing here may
take a path that opens one.
"""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.HardwareConfig import HardwareConfig     # noqa: E402
from SupportClasses.PhysicalModels import (                  # noqa: E402
    NEEDLE_TYPE_CAPILLARY,
)


def _app():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _capillary_page():
    """The REAL page, switched to the pulled-capillary card.

    A stand-in that debounces proves nothing about the widget the operator
    actually types into, so every test here drives the production page.
    """
    app = _app()
    from gui.pages.hardware_setup import HardwareSetupPage
    page = HardwareSetupPage()
    idx = page._needle_type_combo.findData(NEEDLE_TYPE_CAPILLARY)
    assert idx >= 0, "the capillary needle type is not offered"
    page._needle_type_combo.setCurrentIndex(idx)
    page._flush_config_changed()          # settle the type switch itself
    return app, page


def _pump(app, ms):
    """Turn the event loop for ``ms`` so a singleShot QTimer can fire."""
    from PySide6.QtCore import QEventLoop, QTimer
    loop = QEventLoop()
    QTimer.singleShot(ms, loop.quit)
    loop.exec()
    app.processEvents()


class _Sink:
    """Counts config_changed emissions."""

    def __init__(self, page):
        self.n = 0
        self.last = None
        page.config_changed.connect(self._on)

    def _on(self, cfg):
        self.n += 1
        self.last = cfg


# ===================================================================
#  1. The freeze: one fan-out per burst, not one per keystroke
# ===================================================================

class TestConfigFanOutIsDebounced(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app, cls.page = _capillary_page()

    def test_a_burst_of_edits_fans_out_once(self):
        """Typing 3 -> 30 -> 300 must cost ONE app-wide propagation.

        Before this fix each of the three emitted, and each emission rebuilt
        the sketch sequence, the calibration fan-out and Quick Print's
        readiness - which is the stall in freeze.log.
        """
        sink = _Sink(self.page)
        for v in (3.0, 30.0, 300.0):
            self.page._cap_tip_id_spin.setValue(v)
        self.assertEqual(sink.n, 0,
                         "the fan-out ran while the operator was still typing")
        _pump(self._app, 400)
        self.assertEqual(sink.n, 1,
                         "expected one coalesced fan-out, got %d" % sink.n)
        self.assertAlmostEqual(sink.last.needle.tip_id_um, 300.0)

    def test_the_page_readout_tracks_every_edit_immediately(self):
        """Guard the guard: debouncing the FAN-OUT must not debounce the page.

        If this passed while the readout also waited, the fix would have
        replaced a freeze with a stale panel - and the burst test above could
        not tell the difference.
        """
        for v in (11.0, 22.0, 33.0):
            self.page._cap_tip_id_spin.setValue(v)
            self.assertIn("%.1f" % v, self.page.needle_info_label.text(),
                          "the needle readout lagged the spin box")

    def test_the_local_config_is_rebuilt_immediately(self):
        """``_config`` is the page's own state - a Save must never write the
        value from before the last keystroke."""
        self.page._cap_tip_id_spin.setValue(77.0)
        self.assertAlmostEqual(self.page._config.needle.tip_id_um, 77.0)


# ===================================================================
#  2. A pending emit is never dropped
# ===================================================================

class TestPendingEmitSurvivesEveryExit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app, cls.page = _capillary_page()

    def test_leaving_the_page_flushes(self):
        from PySide6.QtGui import QHideEvent
        sink = _Sink(self.page)
        self.page._cap_tip_id_spin.setValue(64.0)
        self.assertEqual(sink.n, 0)
        self.page.hideEvent(QHideEvent())
        self.assertEqual(sink.n, 1, "an edit was lost by navigating away")
        self.assertAlmostEqual(sink.last.needle.tip_id_um, 64.0)

    def test_flush_is_a_no_op_with_nothing_pending(self):
        """The flush is called from three places including aboutToQuit, so it
        must never manufacture an emission the operator did not cause."""
        self.page._cap_tip_id_spin.setValue(51.0)
        self.page._flush_config_changed()
        sink = _Sink(self.page)
        self.page._flush_config_changed()
        self.page._flush_config_changed()
        self.assertEqual(sink.n, 0)

    def test_loading_a_whole_setup_propagates_at_once(self):
        """A setup LOAD replaces the plate, needle and pumps together. That is
        one deliberate decision, not a burst of typing, so it must not wait out
        the keystroke debounce -- anything downstream reading the config right
        after a load would otherwise see the previous setup."""
        cfg = HardwareConfig()
        sink = _Sink(self.page)
        self.page.set_config(cfg)
        self.assertGreaterEqual(
            sink.n, 1, "a setup load did not propagate until the debounce")

    def test_quit_is_wired_to_the_flush(self):
        """Closing the app while still ON this page is the one exit no
        page-level event reports; ``aboutToQuit`` covers it."""
        sink = _Sink(self.page)
        self.page._cap_tip_id_spin.setValue(43.0)
        self.assertEqual(sink.n, 0)
        self._app.aboutToQuit.emit()
        self.assertEqual(sink.n, 1, "an edit was lost by quitting")


# ===================================================================
#  3. The range no longer rewrites what was typed
# ===================================================================

class TestAnyTipDiameterIsAccepted(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app, cls.page = _capillary_page()

    def test_the_old_range_would_have_clipped_these(self):
        """Guard the guard: the values below are outside the retired
        0.5-500 um band, so these tests cannot pass merely because nothing
        constrains the spin at all."""
        from gui.pages.hardware_setup import _CAP_SPIN_SPECS
        lo, hi = _CAP_SPIN_SPECS["tip_id"][0], _CAP_SPIN_SPECS["tip_id"][1]
        self.assertLess(lo, 0.5)
        self.assertGreater(hi, 500.0)

    def test_a_sub_micron_tip_survives_verbatim(self):
        self.page._cap_tip_id_spin.setValue(0.05)
        self.page._flush_config_changed()
        self.assertAlmostEqual(self.page._cap_tip_id_spin.value(), 0.05)
        self.assertAlmostEqual(self.page._config.needle.tip_id_um, 0.05)

    def test_a_very_wide_tip_survives_verbatim(self):
        self.page._cap_tip_id_spin.setValue(5000.0)
        self.page._flush_config_changed()
        self.assertAlmostEqual(self.page._config.needle.tip_id_um, 5000.0)

    def test_every_capillary_field_admits_zero(self):
        """0 reads as "not set" and validate() says so - the alternative is a
        floor the operator has to fight while typing a leading zero."""
        from gui.pages.hardware_setup import _CAP_SPIN_SPECS
        for key, spec in _CAP_SPIN_SPECS.items():
            self.assertEqual(spec[0], 0.0, "%s still has a floor" % key)

    def test_an_implausible_geometry_is_REPORTED_not_clamped(self):
        """The reason the clamp could be removed: the check still exists, it
        just names the number instead of quietly replacing it."""
        self.page._cap_barrel_id_spin.setValue(100.0)
        self.page._cap_tip_id_spin.setValue(900.0)     # tip wider than barrel
        self.page._flush_config_changed()
        self.assertAlmostEqual(self.page._config.needle.tip_id_um, 900.0)
        issues = HardwareConfig._needle_bore_issues(self.page._config.needle)
        self.assertTrue(
            any("cannot exceed" in i for i in issues),
            "an impossible tip was accepted in silence: %r" % (issues,))


# ===================================================================
#  4. Typed text commits once
# ===================================================================

class TestTypingDoesNotWalkTheLiveNeedle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app, cls.page = _capillary_page()

    def test_capillary_spins_commit_on_enter_not_per_keystroke(self):
        for name in ("_cap_barrel_id_spin", "_cap_barrel_od_spin",
                     "_cap_barrel_len_spin", "_cap_tip_id_spin",
                     "_cap_tip_od_spin", "_cap_tip_len_spin"):
            sb = getattr(self.page, name)
            self.assertFalse(
                sb.keyboardTracking(),
                "%s still commits a partial number on every keystroke" % name)

    def test_arrow_steps_still_apply_at_once(self):
        """Only the typed half is deferred - stepping must stay live or the
        spin feels broken."""
        self.page._cap_tip_id_spin.setValue(30.0)
        before = self.page._cap_tip_id_spin.value()
        self.page._cap_tip_id_spin.stepBy(1)
        self.assertGreater(self.page._cap_tip_id_spin.value(), before)
        self.assertAlmostEqual(self.page._config.needle.tip_id_um,
                               self.page._cap_tip_id_spin.value())


if __name__ == "__main__":
    unittest.main()
