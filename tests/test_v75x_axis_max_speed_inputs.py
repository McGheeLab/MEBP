"""v7.5.x tests — Hardware Setup edits ABSOLUTE per-axis max speeds + pump jog %
below 1% (MEBP_v75x_AXIS_MAX_SPEED_INPUTS_AND_SUBPERCENT_PUMP).

Covered (offscreen GUI smoke on the shared HardwareControlPanel):
  Percent mode (jog/workflow pages, speed_as_max=False):
    1. The pump jog % spinbox floor is below 1% (fine plunger jogs).
  Absolute-max mode (Hardware Setup, speed_as_max=True):
    2. The speed section uses absolute units (µm/s, mm/min, mm/min), seeded
       from the single common resolvers.
    3. Editing a max writes the one shared source (safety_limits) + persists to
       settings + fans out via notify_speed_limits_changed.
    4. Editing the Z max routes through apply_device_settings (per_axis['Z']).
    5. Jogging uses the entered ABSOLUTE speed (XY set_velocity / pump feedrate),
       not a % of a max.
"""

import sys
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.StageController import StageController


def _wait_for(predicate, timeout_s=2.0):
    """v7.5.x: pump jog moves now run on a daemon thread (see
    ``HardwareControlPanel._on_jog_pump``) so the GUI thread — and every
    QTimer-driven camera feed — can't freeze while a backlash-compensated
    move drains via blocking M400s. Spin the Qt event loop (delivering the
    queued completion signal) until ``predicate()`` is true or timeout."""
    from PySide6.QtWidgets import QApplication
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    QApplication.processEvents()
    return predicate()


class _Settings:
    """Minimal dict-backed Settings stub (get/set/save)."""

    def __init__(self, data=None):
        self._d = dict(data or {})
        self.saved = 0

    def get(self, key, default=None):
        return self._d.get(key, default)

    def set(self, key, value):
        self._d[key] = value

    def save(self):
        self.saved += 1


class TestAxisMaxSpeedInputs(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _ctrl(self):
        c = mock.MagicMock()
        c.is_xy_connected = True
        c.is_zp_connected = True
        c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        c._pending_per_axis_max_feedrate = None
        c._hardware_config = None
        c.safety_limits = mock.MagicMock()
        c.safety_limits.max_xy_speed = 20000.0
        c.safety_limits.max_z_feedrate = 600.0
        c.safety_limits.max_pump_feedrate = 200.0
        c.get_max_xy_speed_um_s.side_effect = \
            lambda: StageController.get_max_xy_speed_um_s(c)
        c.get_max_z_feedrate_mm_min.side_effect = \
            lambda: StageController.get_max_z_feedrate_mm_min(c)
        c.get_max_pump_feedrate.side_effect = \
            lambda: StageController.get_max_pump_feedrate(c)
        c._pump_jog_max_native.side_effect = \
            lambda: StageController._pump_jog_max_native(c)
        # v7.5.x: per-pump max feedrate (mm/min) + µL/s readout. safety_limits is
        # a MagicMock here, so return the real global directly; no syringe → µL/s
        # readout is '—'.
        c.get_pump_max_feedrate_mm_min.side_effect = \
            lambda pid="P1": float(c.safety_limits.max_pump_feedrate)
        c.pump_feedrate_mm_min_to_uL_s.side_effect = lambda pid, mm: None
        c.get_jog_speed_state.return_value = {}
        return c

    def _panel(self, *, speed_as_max, pump_action_labels=None, settings=None):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        if pump_action_labels is None:
            # Hardware Setup calibration mode ↔ absolute-max; %/µL elsewhere.
            pump_action_labels = not speed_as_max
        c = self._ctrl()
        p = HardwareControlPanel(show_connect=False, bypass_safety=speed_as_max,
                                 pump_action_labels=pump_action_labels,
                                 speed_as_max=speed_as_max)
        if settings is not None:
            p._settings = settings
        p.set_controller(c)
        return p, c

    # 1 — sub-1% pump jog on the %/µL jog pages
    def test_pump_percent_spin_allows_below_one_percent(self):
        p, _ = self._panel(speed_as_max=False)
        self.assertLess(p.spin_p_pct.minimum(), 1.0)
        p.spin_p_pct.setValue(0.05)
        self.assertAlmostEqual(p.spin_p_pct.value(), 0.05, places=3)
        # XY/Z keep the 1% floor (scoped to the pump per the request).
        self.assertGreaterEqual(p.spin_xy_pct.minimum(), 1.0)

    # 2 — absolute units + seeded from the common resolvers
    def test_max_mode_units_and_seed(self):
        p, _ = self._panel(speed_as_max=True)
        self.assertEqual(p.spin_xy_pct.suffix().strip(), "µm/s")
        self.assertEqual(p.spin_z_pct.suffix().strip(), "mm/min")
        self.assertAlmostEqual(p.spin_xy_pct.value(), 20000.0)
        self.assertAlmostEqual(p.spin_z_pct.value(), 600.0)
        # v7.5.x: pump max rate is now PER-PUMP (mm/min primary + µL/s readout).
        self.assertEqual(p.spin_p_max_pumps["P1"].suffix().strip(), "mm/min")
        self.assertAlmostEqual(p.spin_p_max_pumps["P1"].value(), 200.0)

    # 3 — editing XY max writes the shared source + persists + notifies
    def test_max_mode_edit_writes_source(self):
        s = _Settings()
        p, c = self._panel(speed_as_max=True, settings=s)
        p.spin_xy_pct.setValue(35000)
        self.assertEqual(c.safety_limits.max_xy_speed, 35000.0)
        self.assertEqual(s.get("safety_limits.max_xy_speed"), 35000.0)
        self.assertTrue(c.notify_speed_limits_changed.called)

    # 4 — editing Z max routes through apply_device_settings (per_axis['Z'])
    def test_max_mode_edit_z_routes_apply_device_settings(self):
        s = _Settings()
        p, c = self._panel(speed_as_max=True, settings=s)
        p.spin_z_pct.setValue(900)
        self.assertTrue(c.apply_device_settings.called)
        kwargs = c.apply_device_settings.call_args.kwargs
        self.assertEqual(kwargs["per_axis_max_feedrate"]["Z"], 900.0)
        self.assertEqual(
            s.get("device_profile.per_axis_max_feedrate")["Z"], 900.0)

    # 5a — max-mode XY jog uses the absolute µm/s (not % of max)
    def test_max_mode_xy_jog_absolute(self):
        p, c = self._panel(speed_as_max=True, settings=_Settings())
        p.spin_xy_pct.setValue(40000)
        p._on_jog_xy(10.0, 0.0)
        # v7.5.x: XY jog now runs the speed-set + move on a worker thread (so it
        # can't freeze the camera feed) — wait for the backgrounded call.
        self.assertTrue(_wait_for(lambda: c.xy_stage.set_velocity.called))
        c.xy_stage.set_velocity.assert_called_with(40000)

    # 5b — max-mode pump jog is raw mm at the absolute feedrate
    def test_max_mode_pump_jog_absolute_feed(self):
        p, c = self._panel(speed_as_max=True, settings=_Settings())
        p.spin_p_max_pumps["P1"].setValue(150)
        p._on_jog_pump("P1", 0.1)
        self.assertTrue(_wait_for(lambda: c.move_pump_relative.called))
        self.assertFalse(c.move_pump_uL.called)
        self.assertEqual(c.move_pump_relative.call_args.kwargs["feedrate"], 150.0)


if __name__ == "__main__":
    unittest.main()
