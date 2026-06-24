"""test_v75x_zp_auto_reconnect_and_fast_z.py — faster Z moves + ZP auto-reconnect.

After the print-path planner-buffer fix, a single print runs clean but a
sustained back-to-back stress loop still trips the USB-layer drop, with the Z
moves slowing first ("only Z gets hot"). Two hardening changes:

1. Faster Z: `_refresh_zp_move_feedrates()` derives the retract/insert feedrates
   from the configured Z max (retract = Z max; insert = 0.6×), so Z moves finish
   in ~2.5 s instead of ~6 s — less motor on-time/heat, completes within the
   M400 flush timeout. The print MOVE_Z descent uses the (controlled) insert
   feedrate instead of inheriting Marlin's last.
2. Auto-reconnect: an UNEXPECTED ZP loss (USB drop; the CH340 re-enumerates on
   the same COM port) spawns a background reconnect via connect_zp(). Gated to
   real hardware + enabled + not shutting down; never on a manual disconnect.
"""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintCommand, CommandType,
)


def _bare_sc():
    sc = StageController.__new__(StageController)
    sc._zp_retract_feedrate = 200.0
    sc._zp_insert_feedrate = 100.0
    sc._pending_per_axis_max_feedrate = None
    sc.safety_limits = SimpleNamespace(max_z_feedrate=500.0)
    return sc


# ── 1. Fast-Z feedrate derivation ────────────────────────────────────

class TestFastZFeedrates(unittest.TestCase):
    def test_derives_from_safety_z_max(self):
        sc = _bare_sc()
        sc._refresh_zp_move_feedrates()
        self.assertEqual(sc._zp_retract_feedrate, 500.0)      # full Z max (up)
        self.assertAlmostEqual(sc._zp_insert_feedrate, 300.0)  # 0.6× (down)

    def test_per_axis_max_takes_priority(self):
        sc = _bare_sc()
        sc._pending_per_axis_max_feedrate = {"Z": 400.0}
        sc._refresh_zp_move_feedrates()
        self.assertEqual(sc._zp_retract_feedrate, 400.0)

    def test_no_z_max_keeps_defaults(self):
        sc = _bare_sc()
        sc.safety_limits = SimpleNamespace(max_z_feedrate=0)
        sc._refresh_zp_move_feedrates()
        self.assertEqual(sc._zp_retract_feedrate, 200.0)  # unchanged fallback


# ── 2. Print MOVE_Z uses the insert feedrate ─────────────────────────

class TestMoveZUsesInsertFeedrate(unittest.TestCase):
    def test_descent_passes_insert_feedrate(self):
        ctrl = MagicMock()
        ctrl._zp_insert_feedrate = 300.0
        ctrl.is_zp_connected = True
        pm = PrintManager(ctrl)
        pm.job = MagicMock()
        pm.exec_logger = None
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_Z, params={"z": 18.44}))
        # move_z_absolute called with the controlled insert feedrate
        _, kwargs = ctrl.move_z_absolute.call_args
        self.assertEqual(kwargs.get("feedrate_mm_min"), 300.0)


# ── 3. ZP auto-reconnect ─────────────────────────────────────────────

def _reconnect_sc():
    sc = StageController.__new__(StageController)
    sc.auto_reconnect_zp = True
    sc._default_simulate_zp = False
    sc._shutting_down = False
    sc._zp_reconnecting = False
    sc._zp_reconnect_attempts = 4
    sc._zp_reconnect_delay_s = 0.01
    sc._disconnect_lock = threading.RLock()
    sc.on_disconnect = None
    sc.xy_stage = None
    sc.zp_stage = object()
    return sc


class TestAutoReconnect(unittest.TestCase):
    def test_handle_disconnect_schedules_reconnect_on_real_zp(self):
        sc = _reconnect_sc()
        sc.disconnect_zp = lambda: setattr(sc, "zp_stage", None)
        sc._schedule_zp_reconnect = MagicMock()
        sc._handle_disconnect("ZP")
        sc._schedule_zp_reconnect.assert_called_once()

    def test_no_reconnect_when_simulated(self):
        sc = _reconnect_sc()
        sc._default_simulate_zp = True
        sc.disconnect_zp = lambda: setattr(sc, "zp_stage", None)
        sc._schedule_zp_reconnect = MagicMock()
        sc._handle_disconnect("ZP")
        sc._schedule_zp_reconnect.assert_not_called()

    def test_no_reconnect_when_disabled(self):
        sc = _reconnect_sc()
        sc.auto_reconnect_zp = False
        sc.disconnect_zp = lambda: setattr(sc, "zp_stage", None)
        sc._schedule_zp_reconnect = MagicMock()
        sc._handle_disconnect("ZP")
        sc._schedule_zp_reconnect.assert_not_called()

    def test_reconnect_loop_stops_on_success(self):
        sc = _reconnect_sc()
        state = {"connected": False, "attempts": 0}

        def fake_connect(simulate=False):
            state["attempts"] += 1
            state["connected"] = True   # succeeds on the first try
        sc.connect_zp = fake_connect
        with patch.object(StageController, "is_zp_connected",
                          property(lambda s: state["connected"])):
            sc._zp_reconnect_loop()
        self.assertEqual(state["attempts"], 1)
        self.assertFalse(sc._zp_reconnecting)   # reset in finally

    def test_reconnect_loop_gives_up_after_max(self):
        sc = _reconnect_sc()
        sc._zp_reconnect_attempts = 3
        attempts = {"n": 0}

        def fake_connect(simulate=False):
            attempts["n"] += 1   # never connects
        sc.connect_zp = fake_connect
        with patch.object(StageController, "is_zp_connected",
                          property(lambda s: False)):
            sc._zp_reconnect_loop()
        self.assertEqual(attempts["n"], 3)
        self.assertFalse(sc._zp_reconnecting)

    def test_reconnect_loop_bails_when_shutting_down(self):
        sc = _reconnect_sc()
        sc._shutting_down = True
        sc.connect_zp = MagicMock()
        with patch.object(StageController, "is_zp_connected",
                          property(lambda s: False)):
            sc._zp_reconnect_loop()
        sc.connect_zp.assert_not_called()


if __name__ == "__main__":
    unittest.main()
