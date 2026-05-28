"""
test_v75_trajectory_regen.py — Verify trajectory regeneration on
PrintSession load when the saved session has parametric objects with
stripped trajectories.
"""

from __future__ import annotations

import unittest
from dataclasses import dataclass, field
from typing import Any

from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec
from SupportClasses.PrintSessionManager import (
    regenerate_parametric_trajectories,
)


@dataclass
class _StubPump:
    syringe: SyringeSpec = field(default_factory=lambda: SyringeSpec(
        volume_uL=250, stroke_length_mm=30.0, barrel_id_mm=4.65,
    ))


@dataclass
class _StubHardwareConfig:
    needle: NeedleSpec = field(default_factory=lambda: NeedleSpec(
        gauge=27, od_um=413.0, id_um=210.0, wall_um=101.5,
        length_inches=0.5,
    ))
    pumps: dict[str, _StubPump] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.pumps:
            self.pumps = {"P1": _StubPump()}


class TestRegen(unittest.TestCase):

    def test_regen_populates_parametric_trajectory(self) -> None:
        hw = _StubHardwareConfig()
        objs = [{
            "name": "ring",
            "object_type": "circle",
            "params": {"radius": 1.0, "num_points": 32},
            "position": [0, 0, 0],
            "ink_assignments": {"P1": "test_ink"},
            "source": "parametric",
        }]
        n = regenerate_parametric_trajectories(objs, hw)
        self.assertGreaterEqual(n, 1)
        traj = objs[0].get("trajectory")
        self.assertTrue(traj, "trajectory should be populated")
        self.assertGreater(len(traj), 0)

    def test_csv_objects_are_left_alone(self) -> None:
        hw = _StubHardwareConfig()
        existing_traj = [[0, 0, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 1]]
        objs = [{
            "name": "csv1",
            "object_type": "csv_import",
            "params": {},
            "source": "csv",
            "trajectory": list(existing_traj),
        }]
        n = regenerate_parametric_trajectories(objs, hw)
        self.assertEqual(n, 0)
        self.assertEqual(objs[0]["trajectory"], existing_traj)

    def test_no_hw_config_is_safe(self) -> None:
        objs = [{
            "name": "ring", "object_type": "circle",
            "params": {"radius": 1.0}, "source": "parametric",
        }]
        n = regenerate_parametric_trajectories(objs, None)
        self.assertEqual(n, 0)

    def test_no_objects_is_safe(self) -> None:
        n = regenerate_parametric_trajectories([], _StubHardwareConfig())
        self.assertEqual(n, 0)

    def test_handles_one_bad_object_without_crash(self) -> None:
        hw = _StubHardwareConfig()
        objs = [
            {"name": "broken", "object_type": "no_such_type",
             "params": {}, "source": "parametric"},
            {"name": "ring", "object_type": "circle",
             "params": {"radius": 1.0, "num_points": 16},
             "ink_assignments": {"P1": "i"}, "source": "parametric"},
        ]
        n = regenerate_parametric_trajectories(objs, hw)
        # At least the good one should regenerate
        self.assertGreaterEqual(n, 0)
        # ring should have a trajectory now
        ring = next(o for o in objs if o["name"] == "ring")
        self.assertTrue(ring.get("trajectory"))


if __name__ == "__main__":
    unittest.main()
