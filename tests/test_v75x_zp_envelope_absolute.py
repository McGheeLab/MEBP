"""
v7.5.x tests — ZP (Z + pump) safety envelope is the ABSOLUTE Marlin raw frame.

Reported bug: after calibration (which re-anchors the needle/pump zero via
Set Z Zero), Jog-page ZP moves were wrongly clamped. The Z/pump envelope was
zero-referenced, so re-anchoring zero_position pushed the working range outside
the stored limits and valid jogs clamped (the Hardware Setup jog only escaped
this by passing bypass_safety=True).

Fix (mirrors the XY envelope rework): clamp the ABSOLUTE Marlin raw destination
against the absolute envelope, so Set Z Zero / needle-zero no longer invalidates
the limits. These tests assert:
  1. A small jog that stays inside the absolute mechanical envelope is NOT
     clamped, even after a mid-travel re-zero (the bug).
  2. A jog that would exceed the absolute envelope IS clamped.
  3. zero_position no longer affects the clamp (decoupled).
  4. Pump jog behaves the same; axis_map is respected.
"""

import unittest

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits

DEFAULT_MAP = {"Z": "X", "P1": "Y", "P2": "Z", "P3": "E"}
ME3B_V1_MAP = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}


class _FakeZP:
    """ZPStageManager stand-in: axis_map + records the last move_relative."""

    def __init__(self, axis_map):
        self.axis_map = dict(axis_map)
        self.last_axes = None

    def move_relative(self, axes, feedrate=None):
        self.last_axes = dict(axes)


def _stop(ctrl):
    try:
        ctrl._pos_poller.stop()
    except Exception:
        pass


class _Base(unittest.TestCase):
    def setUp(self):
        self.ctrl = StageController(simulate_xy=True, simulate_zp=True)
        self.addCleanup(_stop, self.ctrl)
        # Absolute Marlin-raw envelope: Z in [10, 60], pumps in [0, 34].
        self.ctrl.safety_limits = SafetyLimits(
            z_min=10.0, z_max=60.0,
            p1_min=0.0, p1_max=34.0, p2_min=0.0, p2_max=34.0,
            p3_min=0.0, p3_max=34.0, enabled=True)

    def _attach(self, axis_map, physical_tuple):
        self.ctrl.zp_stage = _FakeZP(axis_map)
        self.ctrl.get_zp_position = lambda cached=True: physical_tuple

    def _z_sent(self):
        return self.ctrl.zp_stage.last_axes[self.ctrl.zp_stage.axis_map["Z"]]

    def _sent(self, axis):
        return self.ctrl.zp_stage.last_axes.get(
            self.ctrl.zp_stage.axis_map[axis])


class TestZAbsoluteClamp(_Base):
    def test_small_jog_not_clamped_after_midtravel_rezero(self):
        # Raw Z = 30 (mid-travel). Set Z Zero re-anchored zero_position to 30,
        # so the zero-ref position is 0. Under the OLD zero-ref envelope [10,60]
        # a -5 jog would land at zero-ref -5 < 10 and clamp. Absolute frame:
        # raw 30-5 = 25 ∈ [10,60] → allowed.
        self._attach(DEFAULT_MAP, (30.0, 0.0, 0.0, 0.0))
        self.ctrl.zero_position["Z"] = 30.0
        self.ctrl.move_z_relative(-5.0)
        self.assertAlmostEqual(self._z_sent(), -5.0, places=6)
        self.ctrl.move_z_relative(+5.0)
        self.assertAlmostEqual(self._z_sent(), +5.0, places=6)

    def test_clamped_at_absolute_max(self):
        self._attach(DEFAULT_MAP, (58.0, 0.0, 0.0, 0.0))
        self.ctrl.zero_position["Z"] = 30.0  # irrelevant now
        self.ctrl.move_z_relative(+5.0)      # 58→63, clamp to 60
        self.assertAlmostEqual(self._z_sent(), 2.0, places=6)

    def test_clamped_at_absolute_min(self):
        self._attach(DEFAULT_MAP, (12.0, 0.0, 0.0, 0.0))
        self.ctrl.move_z_relative(-5.0)      # 12→7, clamp to 10
        self.assertAlmostEqual(self._z_sent(), -2.0, places=6)

    def test_clamp_independent_of_zero_position(self):
        # Same raw position + delta → same sent distance regardless of zero.
        self._attach(DEFAULT_MAP, (40.0, 0.0, 0.0, 0.0))
        self.ctrl.zero_position["Z"] = 0.0
        self.ctrl.move_z_relative(+5.0)
        a = self._z_sent()
        self._attach(DEFAULT_MAP, (40.0, 0.0, 0.0, 0.0))
        self.ctrl.zero_position["Z"] = 1000.0
        self.ctrl.move_z_relative(+5.0)
        b = self._z_sent()
        self.assertAlmostEqual(a, b, places=6)
        self.assertAlmostEqual(a, 5.0, places=6)

    def test_me3b_v1_axis_map_clamps_real_z_slot(self):
        # ME3B V1: Z→Z = tuple index 2. P1→X (index 0) parked at 80 (>z_max)
        # must NOT be read as Z.
        self._attach(ME3B_V1_MAP, (80.0, 0.0, 30.0, 0.0))
        self.ctrl.move_z_relative(-5.0)      # real Z 30→25 ∈ [10,60] → allowed
        self.assertAlmostEqual(self._z_sent(), -5.0, places=6)


class TestPumpAbsoluteClamp(_Base):
    def setUp(self):
        super().setUp()
        self.ctrl.is_pump_enabled = lambda p: True  # skip enable gate

    def test_pump_small_jog_not_clamped_after_rezero(self):
        # Raw P1 = 20 (default map P1→Y, index 1). zero re-anchored to 20.
        self._attach(DEFAULT_MAP, (0.0, 20.0, 0.0, 0.0))
        self.ctrl.zero_position["P1"] = 20.0
        self.ctrl.move_pump_relative("P1", -5.0)   # raw 20→15 ∈ [0,34]
        self.assertAlmostEqual(self._sent("P1"), -5.0, places=6)

    def test_pump_clamped_at_absolute_max(self):
        self._attach(DEFAULT_MAP, (0.0, 32.0, 0.0, 0.0))
        self.ctrl.move_pump_relative("P1", +5.0)   # 32→37, clamp to 34
        self.assertAlmostEqual(self._sent("P1"), 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
