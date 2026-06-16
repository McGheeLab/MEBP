"""
v7.5.x — XY safety envelope is an ABSOLUTE stage-µm frame.

Regression coverage for the needle-zero (Needle Location) calibration bug:
after a *Set Zero* re-anchors ``zero_position`` mid-travel, small XY moves were
clamped against a stale zero-referenced envelope (negative deltas rejected even
though the absolute destination was valid).

The fix clamps the ABSOLUTE destination against the absolute envelope. These
tests call the real ``StageController`` move methods bound to a lightweight
stand-in so no serial/threads are spun up.
"""

import unittest
from types import SimpleNamespace

from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.StageController import StageController


# Real ME3B V1 envelope, now interpreted as ABSOLUTE stage µm.
ASYM = dict(xy_min_x=0.0, xy_max_x=114332.0, xy_min_y=0.0, xy_max_y=76645.0)
# Mid-travel zero, as captured during needle calibration ("Set Zero").
ZERO = {"x": 59562.0, "y": 58143.0, "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0}


class _FakeXYStage:
    """Records the last relative / absolute move sent to hardware."""

    def __init__(self):
        self.rel = None
        self.abs = None

    def move_stage_relative(self, dx, dy):
        self.rel = (dx, dy)

    def move_stage_to_position(self, x, y, fast=False):
        self.abs = (x, y)


def _ctrl(sl: SafetyLimits, zero: dict, cached_xy):
    """A stand-in exposing exactly what the move methods touch."""
    stage = _FakeXYStage()
    return SimpleNamespace(
        xy_stage=stage,
        safety_limits=sl,
        zero_position=zero,
        get_xy_position=lambda cached=True: cached_xy,
    ), stage


class TestRelativeMoveAbsoluteClamp(unittest.TestCase):
    def test_small_negative_delta_at_zero_is_not_clamped(self):
        # The exact bug: at the re-anchored zero (zero-ref ≈ 0), a small
        # negative dy must pass because the ABSOLUTE destination is mid-travel.
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO,
                         (ZERO["x"], ZERO["y"]))
        StageController.move_xy_relative_um(c, 22.5, -9.2)
        self.assertIsNotNone(stage.rel)
        self.assertAlmostEqual(stage.rel[0], 22.5)
        self.assertAlmostEqual(stage.rel[1], -9.2)  # NOT clamped to 0

    def test_move_past_absolute_max_is_clamped(self):
        # Near the upper mechanical limit, the excess is clamped away.
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, (114000.0, 50000.0))
        StageController.move_xy_relative_um(c, 1000.0, 0.0)
        self.assertAlmostEqual(stage.rel[0], 114332.0 - 114000.0)  # 332
        self.assertAlmostEqual(stage.rel[1], 0.0)

    def test_move_below_absolute_min_is_clamped(self):
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, (100.0, 50000.0))
        StageController.move_xy_relative_um(c, -500.0, 0.0)
        self.assertAlmostEqual(stage.rel[0], -100.0)  # clamped to x_min=0
        self.assertAlmostEqual(stage.rel[1], 0.0)

    def test_bypass_safety_sends_raw_delta(self):
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, (100.0, 50000.0))
        StageController.move_xy_relative_um(c, -500.0, 0.0, bypass_safety=True)
        self.assertAlmostEqual(stage.rel[0], -500.0)

    def test_move_xy_relative_mirrors_um_variant(self):
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, (ZERO["x"], ZERO["y"]))
        StageController.move_xy_relative(c, 22.5, -9.2)
        self.assertAlmostEqual(stage.rel[0], 22.5)
        self.assertAlmostEqual(stage.rel[1], -9.2)


class TestAbsoluteMoveClamp(unittest.TestCase):
    def test_from_zero_ref_origin_maps_to_zero_position(self):
        # A well at zero-ref (0,0) mm → absolute zero_position, within envelope.
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, None)
        StageController.move_xy_absolute(c, 0.0, 0.0, from_zero_ref=True)
        self.assertAlmostEqual(stage.abs[0], ZERO["x"])
        self.assertAlmostEqual(stage.abs[1], ZERO["y"])

    def test_from_zero_ref_target_past_max_is_clamped(self):
        # 60 mm in zero-ref X → 60000 + 59562 = 119562 µm > 114332 → clamp.
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, None)
        StageController.move_xy_absolute(c, 60.0, 0.0, from_zero_ref=True)
        self.assertAlmostEqual(stage.abs[0], 114332.0)

    def test_absolute_um_clamps_directly(self):
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, None)
        StageController.move_xy_absolute_um(c, -100.0, 50000.0)
        self.assertAlmostEqual(stage.abs[0], 0.0)       # clamped to x_min
        self.assertAlmostEqual(stage.abs[1], 50000.0)   # within bounds

    def test_absolute_um_within_bounds_unchanged(self):
        c, stage = _ctrl(SafetyLimits(**ASYM), ZERO, None)
        StageController.move_xy_absolute_um(c, 59584.5, 58133.8)
        self.assertAlmostEqual(stage.abs[0], 59584.5)
        self.assertAlmostEqual(stage.abs[1], 58133.8)


class TestDefaultPlateCenterAbsolute(unittest.TestCase):
    def test_center_is_absolute_midpoint_independent_of_zero(self):
        for zero in ({"x": 0.0, "y": 0.0}, dict(ZERO)):
            fake = SimpleNamespace(safety_limits=SafetyLimits(**ASYM),
                                   zero_position=zero)
            cx, cy = StageController.default_plate_center_um(fake)
            self.assertAlmostEqual(cx, 114332.0 / 2.0)
            self.assertAlmostEqual(cy, 76645.0 / 2.0)


if __name__ == "__main__":
    unittest.main()
