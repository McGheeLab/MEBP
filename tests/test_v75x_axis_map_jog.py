"""
v7.5.x regression tests — logical-axis indexing under a non-default axis_map.

Reproduces the ME3B V1 jog-Z bug: the move_z_relative safety clamp read
the ZP position tuple at the hardcoded physical index 0, which under the
ME3B V1 axis_map (Z->Z = index 2, P1->X = index 0) is the P1 pump, not Z.
That (a) bypassed the Z soft-limit and (b) could drive Z a fixed direction
regardless of which jog button was pressed.

These tests drive move_z_relative through a fake ZP stage so we can assert
the exact relative distance the controller sends to the physical Z motor.
"""

import unittest

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits

# Active machine's mapping: logical -> physical Marlin letter.
ME3B_V1_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}
DEFAULT_MAP = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}


class _FakeZP:
    """Minimal stand-in for ZPStageManager: carries an axis_map and
    records the last move_relative call."""

    def __init__(self, axis_map):
        self.axis_map = dict(axis_map)
        self.last_axes = None
        self.last_feedrate = None

    def move_relative(self, axes, feedrate=None):
        self.last_axes = dict(axes)
        self.last_feedrate = feedrate


class _AxisMapJogBase(unittest.TestCase):
    def setUp(self):
        # simulate flags avoid any real serial connect; connect is explicit.
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.ctrl.safety_limits = SafetyLimits(z_min=0.0, z_max=60.0, enabled=True)
        self.ctrl.zero_position["Z"] = 0.0

    def tearDown(self):
        try:
            self.ctrl._pos_poller.stop()
        except Exception:
            pass

    def _attach(self, axis_map, physical_tuple):
        """Attach a fake ZP stage and stub the cached position read."""
        self.ctrl.zp_stage = _FakeZP(axis_map)
        self.ctrl.get_zp_position = lambda cached=True: physical_tuple

    def _z_sent(self):
        """The relative distance commanded to the physical Z letter."""
        axes = self.ctrl.zp_stage.last_axes
        letter = self.ctrl.zp_stage.axis_map["Z"]
        return axes.get(letter)


class TestMe3bV1JogZDirection(_AxisMapJogBase):
    """The reported bug: ME3B V1 map, real Z at tuple index 2."""

    def test_in_range_moves_in_requested_direction(self):
        # physical (X=P1=80, Y=P2=0, Z=realZ=10, E=P3=0)
        # Note P1 (index 0) sits well above the Z envelope [0,60]; the old
        # code would have read it as Z and driven both directions down.
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 10.0, 0.0))

        self.ctrl.move_z_relative(+1.0)
        up = self._z_sent()
        self.ctrl.move_z_relative(-1.0)
        down = self._z_sent()

        self.assertAlmostEqual(up, +1.0, places=6,
            msg=f"Z up should send +1.0, got {up}")
        self.assertAlmostEqual(down, -1.0, places=6,
            msg=f"Z down should send -1.0, got {down}")
        self.assertGreater(up, 0.0)
        self.assertLess(down, 0.0)

    def test_soft_limit_clamps_real_z_not_a_pump(self):
        # Real Z near its max (59.5); P1 (index 0) parked at 80 (>z_max).
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 59.5, 0.0))

        # Up by 1.0 must clamp to the remaining 0.5 mm headroom.
        self.ctrl.move_z_relative(+1.0)
        self.assertAlmostEqual(self._z_sent(), 0.5, places=6,
            msg="Z should clamp to z_max=60 (0.5 mm headroom)")

        # Down by 1.0 is well within range — full move.
        self.ctrl.move_z_relative(-1.0)
        self.assertAlmostEqual(self._z_sent(), -1.0, places=6)

    def test_below_min_does_not_invert_direction(self):
        # Real Z at the bottom (0.5). Down by 1.0 clamps to -0.5; up is full.
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 0.5, 0.0))

        self.ctrl.move_z_relative(-1.0)
        self.assertAlmostEqual(self._z_sent(), -0.5, places=6)
        self.ctrl.move_z_relative(+1.0)
        self.assertAlmostEqual(self._z_sent(), +1.0, places=6)


class TestDefaultMapUnchanged(_AxisMapJogBase):
    """The fix must be a strict no-op under the default axis_map."""

    def test_default_map_identity(self):
        # Default map: Z->X = index 0. Real Z = 10 at index 0.
        self._attach(DEFAULT_MAP, (10.0, 0.0, 0.0, 0.0))

        self.ctrl.move_z_relative(+1.0)
        self.assertAlmostEqual(self._z_sent(), +1.0, places=6)
        self.ctrl.move_z_relative(-1.0)
        self.assertAlmostEqual(self._z_sent(), -1.0, places=6)


class TestCalibrateZeroAxisMap(_AxisMapJogBase):
    """_calibrate_zero / reset_z_zero must record Z's zero from the
    correct physical slot under a non-default map."""

    def test_calibrate_zero_uses_axis_map(self):
        self.ctrl.zp_stage = _FakeZP(ME3B_V1_MAP)
        # physical (X=P1=7, Y=P2=3, Z=realZ=12, E=P3=0)
        self.ctrl.zp_stage.get_current_position = lambda: (7.0, 3.0, 12.0, 0.0)
        self.ctrl._calibrate_zero()
        self.assertAlmostEqual(self.ctrl.zero_position["Z"], 12.0, places=6,
            msg="Z zero must come from physical Z (index 2), not P1 (index 0)")
        self.assertAlmostEqual(self.ctrl.zero_position["P1"], 7.0, places=6,
            msg="P1 zero must come from physical X (index 0)")
        self.assertAlmostEqual(self.ctrl.zero_position["P2"], 3.0, places=6)


class TestLogicalPositionLogging(_AxisMapJogBase):
    """v7.5.x: PositionLogger.record's zp_pos contract is logical
    (Z, P1, P2, P3). Callers must supply a logical-ordered tuple so the
    diagnostic position log stays correctly labeled under a non-default
    axis_map."""

    def test_logical_tuple_me3b_v1(self):
        # physical (X=P1=80, Y=P2=0, Z=realZ=10, E=P3=5)
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 10.0, 5.0))
        t = self.ctrl.get_zp_position_logical_tuple(cached=True)
        self.assertEqual(t, (10.0, 80.0, 0.0, 5.0))  # (Z, P1, P2, P3)

    def test_logical_tuple_default_map_identity(self):
        self._attach(DEFAULT_MAP, (10.0, 1.0, 2.0, 3.0))
        t = self.ctrl.get_zp_position_logical_tuple(cached=True)
        self.assertEqual(t, (10.0, 1.0, 2.0, 3.0))

    def test_reset_z_zero_logs_logical_order(self):
        # Under ME3B V1, the real Z is physical index 2 and P1 is index 0.
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 10.0, 5.0))
        self.ctrl.reset_z_zero()
        rec = self.ctrl.position_logger.records[-1]
        self.assertEqual(rec.event, "zero_reset_z")
        self.assertAlmostEqual(rec.z, 10.0)   # real Z, not the P1 pump (80)
        self.assertAlmostEqual(rec.p1, 80.0)  # P1 (physical X)
        self.assertAlmostEqual(rec.p2, 0.0)
        self.assertAlmostEqual(rec.p3, 5.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
