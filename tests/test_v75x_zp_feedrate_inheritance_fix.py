"""test_v75x_zp_feedrate_inheritance_fix.py — ROOT CAUSE of the recurring
"ZP board drops on the Z retract" / "only Z gets hot" / "fails in the print but
not in jog/travel" failure.

Decisive evidence (logs/zp_serial.log + app.log, 2026-06-18 15:04 run):

    +85.863s  G0 Z18.4400 F300        ok        ← descent (explicit F, fine)
    +97.863s  M400         timeout 11999ms rx=6 busy=6 no-ok   ← Marlin stuck "busy"
    ...then M114 also times out (busy), then the board falls off USB.

and the retract right before it was emitted as a BARE move:

    +80.999s  G0 Z39.5900   ok    ← NO feedrate

The pump moves stream as ``G0 F1.7999… Y0.0016`` — i.e. **F1.8 mm/min**. Z and
the pump P2 are the SAME Marlin board, sharing Marlin's *modal* feedrate. A Z
move emitted without an explicit ``F`` inherits that F1.8, so a ~21 mm full
retract becomes a ~12-MINUTE crawl. Marlin sits ``busy`` executing it, M400
blocks for minutes (→ timeout), the Z motor creeps/overheats, and the board
ultimately drops off USB. Jog/standalone travel always set their own F, which is
why ONLY the print (with interleaved slow pump moves) tripped it.

Fix (two chokepoints):
  1. ``ZPStageManager.move_absolute`` ALWAYS emits an explicit F (falls back to
     the configured default feedrate, never to the stale modal value).
  2. ``StageController.move_z_absolute`` resolves a proper Z feedrate
     (retract feedrate → Z max → 200) whenever the caller passes None — so the
     many bare ``move_z_absolute(...)`` call sites (TRAVEL_UP, etc.) can never
     inherit the pump's flow rate.
Plus: a Z move that can't be confirmed at print height now ABORTS the print
(raise) instead of dry-dragging at the wrong Z.
"""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.ZPStage import ZPStageManager
from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintCommand, CommandType,
)


def _capture_zp():
    """A real ZPStageManager (via __new__) with send_data captured."""
    zp = ZPStageManager.__new__(ZPStageManager)
    zp.feedrate = 200.0
    zp.simulate = False
    sent: list[str] = []
    zp.send_data = lambda line, *a, **k: sent.append(line)
    return zp, sent


# ── 1. The serial-layer chokepoint: move_absolute always emits F ──────

class TestMoveAbsoluteAlwaysEmitsFeedrate(unittest.TestCase):
    def test_bare_absolute_move_emits_default_F_not_blank(self):
        zp, sent = _capture_zp()
        zp.move_absolute({"X": 39.59})           # no feedrate given
        g0 = next(l for l in sent if l.startswith("G0 "))
        self.assertIn(" F", g0, f"bare absolute move must carry an F: {g0!r}")
        self.assertIn("F200", g0)                # falls back to self.feedrate

    def test_explicit_feedrate_is_used(self):
        zp, sent = _capture_zp()
        zp.move_absolute({"X": 18.44}, feedrate_mm_min=300)
        g0 = next(l for l in sent if l.startswith("G0 "))
        self.assertIn("F300", g0)

    def test_z_move_does_not_inherit_pump_modal_feedrate(self):
        """The exact bug: a slow pump move then a 'bare' Z move. The Z move
        must still carry its OWN feedrate, never relying on Marlin's modal F."""
        zp, sent = _capture_zp()
        # Pump extrusion move leaves Marlin's modal F at 1.8 mm/min:
        zp.move_relative({"Y": 0.0016}, feedrate=1.8)
        pump_line = next(l for l in sent if "Y0.0016" in l)
        self.assertIn("F1", pump_line)           # ~F1.8
        sent.clear()
        # A subsequent absolute Z retract must NOT be bare:
        zp.move_absolute({"X": 39.59})
        g0 = next(l for l in sent if l.startswith("G0 "))
        self.assertIn(" F", g0)
        self.assertNotIn("F1.8", g0)             # did NOT inherit the pump's F
        self.assertIn("F200", g0)


# ── 2. StageController.move_z_absolute resolves a Z feedrate ──────────

def _z_ctrl(retract=500.0, max_z=None):
    sc = StageController.__new__(StageController)
    sc.zero_position = {"Z": 0.0}
    sc.safety_limits = SimpleNamespace(enabled=False, max_z_feedrate=max_z)
    sc._apply_print_floor_raw = lambda p: p
    if retract is not None:
        sc._zp_retract_feedrate = retract
    captured = {}
    sc.zp_stage = SimpleNamespace(
        move_absolute=lambda axes, fast, feedrate_mm_min=None: captured.update(
            axes=axes, fast=fast, feedrate_mm_min=feedrate_mm_min))
    return sc, captured


class TestMoveZAbsoluteResolvesFeedrate(unittest.TestCase):
    def test_none_feedrate_resolves_to_retract_feedrate(self):
        sc, cap = _z_ctrl(retract=500.0)
        sc.move_z_absolute(39.59, from_zero_ref=True)   # no feedrate
        self.assertEqual(cap["feedrate_mm_min"], 500.0)

    def test_explicit_feedrate_passes_through(self):
        sc, cap = _z_ctrl(retract=500.0)
        sc.move_z_absolute(18.44, from_zero_ref=True, feedrate_mm_min=300.0)
        self.assertEqual(cap["feedrate_mm_min"], 300.0)

    def test_falls_back_to_z_max_when_no_retract_feedrate(self):
        sc, cap = _z_ctrl(retract=None, max_z=450.0)
        sc.move_z_absolute(39.59, from_zero_ref=True)
        self.assertEqual(cap["feedrate_mm_min"], 450.0)

    def test_last_resort_default_when_nothing_configured(self):
        sc, cap = _z_ctrl(retract=None, max_z=None)
        sc.move_z_absolute(39.59, from_zero_ref=True)
        self.assertEqual(cap["feedrate_mm_min"], 200.0)


# ── 3. TRAVEL_UP / TRAVEL_DOWN carry explicit feedrates ───────────────

def _pm_with_ctrl():
    ctrl = MagicMock()
    ctrl._zp_retract_feedrate = 500.0
    ctrl._zp_insert_feedrate = 300.0
    ctrl.is_zp_connected = True
    pm = PrintManager(ctrl)
    pm.job = MagicMock()
    pm.job.settings = SimpleNamespace(travel_z_height=39.59, print_z_height=18.44)
    pm.exec_logger = None
    return pm, ctrl


class TestTravelCommandsCarryFeedrate(unittest.TestCase):
    def test_travel_up_uses_retract_feedrate(self):
        pm, ctrl = _pm_with_ctrl()
        pm._execute_command(PrintCommand(type=CommandType.TRAVEL_UP))
        _, kwargs = ctrl.move_z_absolute.call_args
        self.assertEqual(kwargs.get("feedrate_mm_min"), 500.0)

    def test_travel_down_uses_insert_feedrate(self):
        pm, ctrl = _pm_with_ctrl()
        pm._execute_command(PrintCommand(type=CommandType.TRAVEL_DOWN))
        _, kwargs = ctrl.move_z_absolute.call_args
        self.assertEqual(kwargs.get("feedrate_mm_min"), 300.0)


# ── 4. MOVE_Z aborts (raises) when Z can't be confirmed ───────────────

class TestMoveZAbortsOnUnconfirmed(unittest.TestCase):
    def _ctrl(self, arrival_ok: bool, m400_ok: bool = True):
        ctrl = MagicMock()
        ctrl._zp_insert_feedrate = 300.0
        ctrl.is_zp_connected = True
        ctrl.suspend_position_poller = MagicMock()
        ctrl.resume_position_poller = MagicMock()
        ctrl.wait_for_z_arrival = MagicMock(return_value=arrival_ok)
        ctrl.zp_stage = SimpleNamespace(
            flush_moves=lambda timeout_s=10.0: m400_ok)
        return ctrl

    def test_raises_when_arrival_not_confirmed(self):
        ctrl = self._ctrl(arrival_ok=False)
        pm = PrintManager(ctrl)
        pm.job = MagicMock(); pm.exec_logger = None
        with self.assertRaises(RuntimeError):
            pm._execute_command(PrintCommand(
                type=CommandType.MOVE_Z, params={"z": 18.44}))
        ctrl.resume_position_poller.assert_called_once()  # poller restored

    def test_raises_when_m400_times_out(self):
        ctrl = self._ctrl(arrival_ok=True, m400_ok=False)
        pm = PrintManager(ctrl)
        pm.job = MagicMock(); pm.exec_logger = None
        with self.assertRaises(RuntimeError):
            pm._execute_command(PrintCommand(
                type=CommandType.MOVE_Z, params={"z": 18.44}))

    def test_no_raise_when_confirmed(self):
        ctrl = self._ctrl(arrival_ok=True, m400_ok=True)
        pm = PrintManager(ctrl)
        pm.job = MagicMock(); pm.exec_logger = None
        pm._execute_command(PrintCommand(
            type=CommandType.MOVE_Z, params={"z": 18.44}))  # must not raise
        # descent used the controlled insert feedrate
        _, kwargs = ctrl.move_z_absolute.call_args
        self.assertEqual(kwargs.get("feedrate_mm_min"), 300.0)


if __name__ == "__main__":
    unittest.main()
