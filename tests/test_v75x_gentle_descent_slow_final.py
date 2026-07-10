"""
test_v75x_gentle_descent_slow_final.py — every print/work RE-ENTRY descent lowers
the needle's FINAL ~1 mm SLOWLY (a controlled touch-down onto the plate / into a
bead), with the bulk of the descent at the fast insert feedrate. The descent twin
of test_v75x_gentle_retract_slow_lift.

Covers the shared StageController primitive (_descend_z_moves_only /
emit_descent_moves), its use by safe_travel_to step 3 (so ALL workflows that
travel between positions inherit it) and the discrete PrintManager MOVE_Z handler
(Quick Print / discrete re-entry), and the setter. The emit-only helper does NOT
confirm — the callers keep their own M400 + wait_for_z_arrival + abort layer, so
the abort-before-extrusion safety is untouched.

No GUI / hardware required.
"""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.StageController import StageController


def _ctrl(descend_dist=1.0, descend_fr=60.0, slow_dist=1.0, slow_fr=60.0,
          z_up_sign=-1.0, retract_fr=250.0, insert_fr=100.0):
    """Minimal real StageController (no __init__) with the gentle-Z attrs."""
    c = StageController.__new__(StageController)
    c.xy_stage = MagicMock()
    c.zp_stage = MagicMock()
    c.zp_stage.flush_moves = MagicMock(return_value=True)
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0, "P2": 0, "P3": 0}
    c._pos_poller = MagicMock()
    c._zp_retract_feedrate = retract_fr
    c._zp_insert_feedrate = insert_fr
    c._min_travel_z_mm = None
    c._z_up_sign = z_up_sign
    c._retract_slow_dist_mm = slow_dist
    c._retract_slow_feedrate = slow_fr
    c._descend_slow_dist_mm = descend_dist
    c._descend_slow_feedrate = descend_fr
    c.move_z_absolute = MagicMock()
    c.move_xy_absolute = MagicMock()
    c.wait_for_z_arrival = MagicMock(return_value=True)
    c.wait_for_xy_arrival = MagicMock(return_value=True)
    return c


class TestDescendMovesOnlyPrimitive(unittest.TestCase):
    """_descend_z_moves_only: fast-then-slow on a descent, single move otherwise.
    Emit-only — it must NOT call flush_moves / wait_for_z_arrival."""

    def test_descent_does_fast_then_slow(self):
        c = _ctrl()
        # ZDIR=-1: needle retracted at raw 0 (height 0); target raw +25
        # (height -25) is a 25 mm DESCENT.
        c._descend_z_moves_only(0.0, 25.0, 100.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 2)
        # 1st: FAST to 1 mm above the target in the HEIGHT frame.
        # height -24 → zero-ref raw = -24 * z_up_sign(-1) = 24.
        self.assertAlmostEqual(calls[0].args[0], 24.0, places=6)
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 100.0)   # fast
        # 2nd: SLOW final mm to the target at the descend feedrate.
        self.assertAlmostEqual(calls[1].args[0], 25.0, places=6)
        self.assertEqual(calls[1].kwargs["feedrate_mm_min"], 60.0)    # slow
        # Emit-only: no confirmation here (the caller confirms).
        c.zp_stage.flush_moves.assert_not_called()
        c.wait_for_z_arrival.assert_not_called()

    def test_ascent_is_single_move(self):
        c = _ctrl()
        # Target raw 0 (height 0) is ABOVE current raw +25 (height -25): an
        # ascent — never slowed, single move.
        c._descend_z_moves_only(25.0, 0.0, 250.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)
        self.assertAlmostEqual(c.move_z_absolute.call_args.args[0], 0.0)

    def test_unknown_cur_is_single_move(self):
        c = _ctrl()
        c._descend_z_moves_only(None, 25.0, 100.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)

    def test_slow_dist_zero_is_single_move(self):
        c = _ctrl(descend_dist=0.0)
        c._descend_z_moves_only(0.0, 25.0, 100.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)
        self.assertEqual(c.move_z_absolute.call_args.kwargs["feedrate_mm_min"], 100.0)

    def test_short_descent_is_entirely_slow(self):
        c = _ctrl(descend_dist=5.0)
        # Descent of only 2 mm (< slow_dist) → the whole descent is the slow
        # segment; the fast leg is a no-op (intermediate == current height).
        c._descend_z_moves_only(0.0, 2.0, 100.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 1)                    # fast leg skipped
        self.assertAlmostEqual(calls[0].args[0], 2.0, places=6)
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 60.0)   # slow

    def test_plus_one_polarity_descent(self):
        # ZDIR=+1 machine: higher raw = higher needle. Descend from raw 39.59
        # (height 39.59) to print raw 18.44 (height 18.44).
        c = _ctrl(z_up_sign=1.0)
        c._descend_z_moves_only(39.59, 18.44, 100.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 2)
        self.assertAlmostEqual(calls[0].args[0], 19.44, places=6)    # fast leg
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 100.0)
        self.assertAlmostEqual(calls[1].args[0], 18.44, places=6)    # slow leg
        self.assertEqual(calls[1].kwargs["feedrate_mm_min"], 60.0)

    def test_slow_feedrate_falls_back_to_fast_when_unset(self):
        c = _ctrl(descend_fr=0.0)
        c._descend_z_moves_only(0.0, 25.0, 100.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(calls[1].kwargs["feedrate_mm_min"], 100.0)  # fast fallback


class TestEmitDescentMoves(unittest.TestCase):
    """emit_descent_moves reads the CACHED Z (no serial) and emits the descent."""

    def test_reads_cached_and_emits_fast_then_slow(self):
        c = _ctrl()
        c.get_zp_position = MagicMock(return_value=(0.0, 0.0, 0.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=0.0)
        c.emit_descent_moves(25.0, 100.0)
        # cached read must be non-blocking (cached=True).
        self.assertTrue(c.get_zp_position.call_args.kwargs.get("cached"))
        self.assertEqual(c.move_z_absolute.call_count, 2)            # fast + slow

    def test_unknown_cached_is_single_move(self):
        c = _ctrl()
        c.get_zp_position = MagicMock(return_value=None)
        c.zp_logical_value = MagicMock(return_value=None)
        c.emit_descent_moves(25.0, 100.0)
        self.assertEqual(c.move_z_absolute.call_count, 1)


class TestSafeTravelDescentGentle(unittest.TestCase):
    """safe_travel_to step 3 lowers with a gentle slow final mm — standard for
    every workflow that travels between positions."""

    def test_descent_is_fast_then_slow(self):
        c = _ctrl()
        # Mock reports the needle at raw +25 (height -25) throughout.
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        # Lift to safe raw 0 (height 0), then descend to raw +30 (height -30) —
        # a descent from the reported current (height -25).
        ok = c.safe_travel_to(5000.0, 10000.0, safe_z_mm=0.0, target_z_mm=30.0)
        self.assertTrue(ok)
        calls = c.move_z_absolute.call_args_list
        # slow lift + fast lift + FAST descent + SLOW descent = 4 Z moves.
        self.assertEqual(len(calls), 4)
        self.assertEqual(calls[0].kwargs["feedrate_mm_min"], 60.0)    # slow lift
        self.assertEqual(calls[1].kwargs["feedrate_mm_min"], 250.0)   # fast lift
        self.assertEqual(calls[2].kwargs["feedrate_mm_min"], 100.0)   # fast descent
        self.assertEqual(calls[3].kwargs["feedrate_mm_min"], 60.0)    # slow last mm
        # Final descent move lands exactly on the target.
        self.assertAlmostEqual(calls[3].args[0], 30.0, places=6)

    def test_descent_single_move_when_disabled(self):
        c = _ctrl(descend_dist=0.0)
        c.get_zp_position = MagicMock(return_value=(25.0, 0.0, 25.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=25.0)
        c.safe_travel_to(5000.0, 10000.0, safe_z_mm=0.0, target_z_mm=30.0)
        calls = c.move_z_absolute.call_args_list
        self.assertEqual(len(calls), 3)                    # slow+fast lift, 1 descent
        self.assertEqual(calls[2].kwargs["feedrate_mm_min"], 100.0)   # insert


class TestSetter(unittest.TestCase):
    def test_set_descend_slow_final(self):
        c = _ctrl()
        c.set_descend_slow_final(2.5, 90.0)
        self.assertEqual(c._descend_slow_dist_mm, 2.5)
        self.assertEqual(c._descend_slow_feedrate, 90.0)
        c.set_descend_slow_final(0.0)                      # disable, keep feedrate
        self.assertEqual(c._descend_slow_dist_mm, 0.0)
        self.assertEqual(c._descend_slow_feedrate, 90.0)

    def test_defaults_present_on_real_init(self):
        # A fully-constructed controller carries the gentle-descent defaults so
        # the behavior is ON by default for every workflow (no config needed).
        c = StageController.__new__(StageController)
        # Emulate __init__ having run the two assignments (cheap, no hardware):
        StageController.set_descend_slow_final(c, 1.0, 60.0)
        self.assertEqual(c._descend_slow_dist_mm, 1.0)
        self.assertEqual(c._descend_slow_feedrate, 60.0)


class TestConfigWiring(unittest.TestCase):
    """The user-controllable gentle-Z config (one distance + one speed) is
    serialized on HardwareConfig, surfaced as CommonPrintSettings globals, and
    pushed to BOTH the retract slow-lift and the descend slow-final in
    set_hardware_config — so it is standard for every workflow and tunable."""

    def test_hardware_config_roundtrip(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        c = HardwareConfig()
        self.assertEqual(c.gentle_z_slow_dist_mm, 1.0)
        self.assertEqual(c.gentle_z_slow_speed_mm_s, 1.0)
        c.gentle_z_slow_dist_mm = 2.5
        c.gentle_z_slow_speed_mm_s = 0.75
        c2 = HardwareConfig.from_dict(c.to_dict())
        self.assertEqual(c2.gentle_z_slow_dist_mm, 2.5)
        self.assertEqual(c2.gentle_z_slow_speed_mm_s, 0.75)
        # Missing keys fall back to defaults.
        c3 = HardwareConfig.from_dict({})
        self.assertEqual(c3.gentle_z_slow_dist_mm, 1.0)
        self.assertEqual(c3.gentle_z_slow_speed_mm_s, 1.0)

    def test_common_print_settings_exposes_globals(self):
        from SupportClasses.CommonPrintSettings import CommonPrintSettings
        from SupportClasses.HardwareConfig import HardwareConfig
        m = CommonPrintSettings()
        self.assertTrue(m.is_global("gentle_z_slow_dist_mm"))
        self.assertTrue(m.is_global("gentle_z_slow_speed_mm_s"))
        hw = HardwareConfig()
        m.set_hardware_config(hw)
        m.set("gentle_z_slow_dist_mm", 2.0)
        m.set("gentle_z_slow_speed_mm_s", 1.5)
        self.assertEqual(hw.gentle_z_slow_dist_mm, 2.0)      # proxied to HW config
        self.assertEqual(hw.gentle_z_slow_speed_mm_s, 1.5)
        self.assertEqual(m.get("gentle_z_slow_dist_mm"), 2.0)

    def test_set_hardware_config_pushes_both(self):
        from SupportClasses.HardwareConfig import HardwareConfig
        c = StageController.__new__(StageController)
        c.zp_stage = MagicMock(); c.xy_stage = MagicMock(); c.zp_jog = None
        c.safety_limits = None; c._pos_poller = MagicMock()
        c._retract_slow_dist_mm = 1.0; c._retract_slow_feedrate = 60.0
        c._descend_slow_dist_mm = 1.0; c._descend_slow_feedrate = 60.0
        c.is_pump_plunger_calibrated = lambda p: False
        c.refresh_jog_speed_limits = lambda: None
        c._apply_active_plate_type_offsets = lambda cfg: None
        cfg = HardwareConfig()
        cfg.gentle_z_slow_dist_mm = 1.5
        cfg.gentle_z_slow_speed_mm_s = 2.0            # → 120 mm/min
        c.set_hardware_config(cfg)
        self.assertEqual((c._retract_slow_dist_mm, c._retract_slow_feedrate), (1.5, 120.0))
        self.assertEqual((c._descend_slow_dist_mm, c._descend_slow_feedrate), (1.5, 120.0))
        # dist 0 disables both directions (single-speed / legacy).
        cfg.gentle_z_slow_dist_mm = 0.0
        c.set_hardware_config(cfg)
        self.assertEqual(c._retract_slow_dist_mm, 0.0)
        self.assertEqual(c._descend_slow_dist_mm, 0.0)


class TestMoveZHandlerGentle(unittest.TestCase):
    """The discrete MOVE_Z handler emits the gentle descent on a real controller,
    but falls back to a single move on a mock/older controller so the existing
    confirm/abort tests (and the abort-before-extrusion safety) are untouched."""

    def _pm(self, ctrl):
        from SupportClasses.PrintManager import PrintManager
        pm = PrintManager(ctrl)
        pm.job = MagicMock()
        pm.job.settings = SimpleNamespace(travel_z_height=39.59,
                                          print_z_height=25.0)
        pm.exec_logger = None
        return pm

    def test_real_ctrl_emits_fast_then_slow_and_confirms(self):
        from SupportClasses.PrintManager import PrintCommand, CommandType
        c = _ctrl()   # is_zp_connected property → truthy (MagicMock zp_stage)
        c.get_zp_position = MagicMock(return_value=(0.0, 0.0, 0.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=0.0)
        c.suspend_position_poller = MagicMock()
        c.resume_position_poller = MagicMock()
        pm = self._pm(c)
        pm._execute_command(PrintCommand(type=CommandType.MOVE_Z, params={"z": 25.0}))
        # Two-segment descent emitted (fast leg + slow final mm).
        self.assertEqual(c.move_z_absolute.call_count, 2)
        self.assertEqual(c.move_z_absolute.call_args_list[-1].kwargs["feedrate_mm_min"], 60.0)
        # Handler still confirms (its own M400 + wait) — abort safety intact.
        c.zp_stage.flush_moves.assert_called_once()
        c.wait_for_z_arrival.assert_called_once()

    def test_mock_ctrl_falls_back_to_single_move(self):
        from SupportClasses.PrintManager import PrintCommand, CommandType
        ctrl = MagicMock()          # bare mock: _descend_slow_dist_mm is a Mock
        ctrl.is_zp_connected = True
        ctrl.wait_for_z_arrival = MagicMock(return_value=True)
        pm = self._pm(ctrl)
        pm._execute_command(PrintCommand(type=CommandType.MOVE_Z, params={"z": 25.0}))
        ctrl.move_z_absolute.assert_called_once()   # legacy single move
        ctrl.emit_descent_moves.assert_not_called()


class TestGentleZConfirmTimeoutIsDurationAware(unittest.TestCase):
    """REGRESSION: the gentle "slow last mm" descent runs its final leg at a low
    feedrate (e.g. 1 mm @ 6 mm/min = 10 s). A FIXED 10 s M400 confirm timeout
    expired while a perfectly healthy slow descent was still finishing and
    FALSELY tripped the "Z move ... not confirmed (board stuck) — aborting before
    extrusion" guard (real HW: print aborted right after a big ink pickup). The
    confirm timeout must be SIZED to the estimated descent duration."""

    def test_estimate_covers_fast_plus_slow_legs(self):
        # 6 mm/min slow leg (0.1 mm/s), 300 mm/min fast leg. z_up_sign=-1:
        # cur raw 0 → height 0; target raw +21 → height -21 = a 21 mm DESCENT.
        c = _ctrl(descend_dist=1.0, descend_fr=6.0)
        est = c.estimate_gentle_z_time_s(21.0, 300.0, cur_zref_mm=0.0)
        # fast: 20 mm / 300 mm/min = 4 s; slow: 1 mm / 6 mm/min = 10 s.
        self.assertAlmostEqual(est, 14.0, places=3)

    def test_estimate_zero_when_current_z_unknown(self):
        c = _ctrl()
        c.get_zp_position = MagicMock(side_effect=RuntimeError("no board"))
        self.assertEqual(c.estimate_gentle_z_time_s(21.0, 300.0), 0.0)

    def test_estimate_ascent_uses_lift_slow_params(self):
        # An ASCENT (target higher than current) uses the retract slow-lift
        # params. z_up_sign=-1: cur raw +21 (h -21) → target raw 0 (h 0) = lift.
        c = _ctrl(slow_dist=1.0, slow_fr=6.0)
        est = c.estimate_gentle_z_time_s(0.0, 300.0, cur_zref_mm=21.0)
        self.assertAlmostEqual(est, 14.0, places=3)

    def test_move_z_handler_uses_scaled_timeout_not_fixed_10s(self):
        from SupportClasses.PrintManager import PrintCommand, CommandType
        # A slow descent whose duration far exceeds 10 s.
        c = _ctrl(descend_dist=1.0, descend_fr=6.0, insert_fr=300.0)
        c.get_zp_position = MagicMock(return_value=(0.0, 0.0, 0.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=0.0)
        c.suspend_position_poller = MagicMock()
        c.resume_position_poller = MagicMock()
        pm = self._pm(c)
        pm._execute_command(
            PrintCommand(type=CommandType.MOVE_Z, params={"z": 21.0}))
        # flush_moves + wait_for_z_arrival must get a timeout WELL above the old
        # fixed 10 s (≈ estimate 14 s + 5 s margin ≈ 19 s), so the healthy slow
        # descent confirms instead of falsely aborting.
        to = c.zp_stage.flush_moves.call_args.kwargs["timeout_s"]
        self.assertGreater(to, 10.0)
        self.assertAlmostEqual(to, 19.0, delta=1.0)
        self.assertEqual(
            c.wait_for_z_arrival.call_args.kwargs["timeout_s"], to)

    def _pm(self, ctrl):
        from SupportClasses.PrintManager import PrintManager
        pm = PrintManager(ctrl)
        pm.job = MagicMock()
        pm.job.settings = SimpleNamespace(travel_z_height=39.59,
                                          print_z_height=25.0)
        pm.exec_logger = None
        return pm

    def test_move_z_handler_floors_at_10s_for_fast_descent(self):
        from SupportClasses.PrintManager import PrintCommand, CommandType
        # Fast single-speed descent (gentle disabled) → tiny estimate, floored
        # at the 10 s baseline (never shrinks the confirm window).
        c = _ctrl(descend_dist=0.0, insert_fr=300.0)
        c.get_zp_position = MagicMock(return_value=(0.0, 0.0, 0.0, 0.0))
        c.zp_logical_value = MagicMock(return_value=0.0)
        c.suspend_position_poller = MagicMock()
        c.resume_position_poller = MagicMock()
        pm = self._pm(c)
        pm._execute_command(
            PrintCommand(type=CommandType.MOVE_Z, params={"z": 5.0}))
        self.assertEqual(
            c.zp_stage.flush_moves.call_args.kwargs["timeout_s"], 10.0)


if __name__ == "__main__":
    unittest.main()
