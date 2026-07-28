"""
test_v75x_stage_jog_off_gui_thread.py

Fix: "the camera freezes whenever I use any other stages in jog mode."

The XY/Z incremental-jog handlers on the shared ``HardwareControlPanel`` ran the
move AND a fresh ``cached=False`` position read synchronously on the GUI thread —
blocking the Qt event loop (and every QTimer-driven camera feed) for the serial
round-trips. They now run on a daemon thread (mirroring the pump-jog fix): the
speed-set + move + fresh reads happen off-thread, and the readout is updated via
the queued ``_stage_jog_done`` signal (no serial I/O on the GUI thread). A
per-axis busy guard drops overlapping clicks instead of queuing blocking moves.
"""

import os
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)


def _wait_for(predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    QApplication.processEvents()
    return predicate()


def _ctrl():
    c = mock.MagicMock()
    c.is_xy_connected = True
    c.is_zp_connected = True
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    c.safety_limits = mock.MagicMock()
    c.safety_limits.max_xy_speed = 20000.0
    c.safety_limits.max_z_feedrate = 600.0
    c.safety_limits.max_pump_feedrate = 200.0
    c.get_max_xy_speed_um_s.return_value = 20000.0
    c.get_max_z_feedrate_mm_min.return_value = 600.0
    c.get_max_pump_feedrate.return_value = 200.0
    c.get_jog_speed_state.return_value = {}
    c.get_xy_position.return_value = (0.0, 0.0)
    c.get_zp_position.return_value = (0.0, 0.0, 0.0, 0.0)
    return c


def _panel(c):
    from gui.pages.hardware.control_panel import HardwareControlPanel
    p = HardwareControlPanel(show_connect=False, bypass_safety=False,
                             pump_action_labels=True)
    p.set_controller(c)
    return p


class TestStageJogOffGuiThread(unittest.TestCase):

    def test_xy_jog_runs_off_thread_and_does_not_block(self):
        c = _ctrl()
        gate = threading.Event()
        started = threading.Event()

        def _slow_move(*a, **k):
            started.set()
            gate.wait(2.0)          # hold the "serial" busy
        c.move_xy_relative_um.side_effect = _slow_move

        p = _panel(c)
        t0 = time.monotonic()
        p._on_jog_xy(100.0, 0.0)
        elapsed = time.monotonic() - t0

        # The handler returned promptly — the blocking move is on the worker.
        self.assertLess(elapsed, 0.5)
        self.assertTrue(started.wait(1.0))
        self.assertTrue(p._xy_jog_busy)

        # A second click while busy is dropped (no queued blocking move).
        p._on_jog_xy(100.0, 0.0)

        gate.set()
        self.assertTrue(_wait_for(lambda: not p._xy_jog_busy))
        self.assertEqual(c.move_xy_relative_um.call_count, 1)
        # Fresh reads happened OFF the GUI thread (in the worker).
        self.assertTrue(c.get_xy_position.called)
        self.assertTrue(c.get_zp_position.called)

    def test_z_jog_runs_off_thread(self):
        c = _ctrl()
        gate = threading.Event()

        def _slow_move(*a, **k):
            gate.wait(2.0)
        c.move_z_user_relative.side_effect = _slow_move

        p = _panel(c)
        t0 = time.monotonic()
        p._on_jog_z(0.5)
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertTrue(p._z_jog_busy)
        gate.set()
        self.assertTrue(_wait_for(lambda: not p._z_jog_busy))
        self.assertTrue(c.move_z_user_relative.called)

    def test_xy_and_z_guards_are_independent(self):
        # A Z jog in flight must not block starting an XY jog (different stage).
        c = _ctrl()
        zgate = threading.Event()
        c.move_z_user_relative.side_effect = lambda *a, **k: zgate.wait(2.0)
        p = _panel(c)
        p._on_jog_z(0.5)
        self.assertTrue(p._z_jog_busy)
        # XY jog still runs (its own guard is independent).
        p._on_jog_xy(50.0, 0.0)
        self.assertTrue(_wait_for(lambda: c.move_xy_relative_um.called))
        zgate.set()
        self.assertTrue(_wait_for(lambda: not p._z_jog_busy))


if __name__ == "__main__":
    unittest.main()
