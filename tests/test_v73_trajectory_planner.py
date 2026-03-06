#!/usr/bin/env python3
"""
test_v73_trajectory_planner.py — Tests for PrintTrajectoryPlanner.

Run: python -m unittest tests.test_v73_trajectory_planner -v
"""

import unittest
import sys
from pathlib import Path
from unittest.mock import MagicMock
from dataclasses import dataclass, field

project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))


def _mock_settings(**overrides):
    """Create a mock PrintSettings with sensible defaults."""
    from SupportClasses.PrintManager import PrintSettings
    s = PrintSettings()
    s.xy_feedrate = overrides.get('xy_feedrate', 6000.0)    # mm/min
    s.z_feedrate = overrides.get('z_feedrate', 120.0)       # mm/min
    s.print_feedrate = overrides.get('print_feedrate', 600.0)
    s.pump_feedrate = overrides.get('pump_feedrate', 30.0)
    s.travel_z_height = overrides.get('travel_z_height', 5.0)
    s.print_z_height = overrides.get('print_z_height', 0.1)
    s.layer_height = overrides.get('layer_height', 0.1)
    s.num_layers = overrides.get('num_layers', 1)
    s.flow_rate = overrides.get('flow_rate', 0.01)
    s.dwell_after_move = overrides.get('dwell_after_move', 0.3)
    s.active_pump = overrides.get('active_pump', 'P1')
    return s


def _mock_well_model(assignments_dict):
    model = MagicMock()
    model.assignments = {}
    for name, role_val in assignments_dict.items():
        a = MagicMock()
        a.role = MagicMock()
        a.role.value = role_val
        model.assignments[name] = a
    return model


def _mock_plate(well_positions):
    plate = MagicMock()
    positions = {name: (x, y) for name, x, y in well_positions}
    plate.get_well_position = lambda name: positions[name]
    plate.format = 24
    plate.well_diameter = 15.6
    return plate


class TestWaypointBasics(unittest.TestCase):
    """Test Waypoint dataclass."""

    def test_default_waypoint(self):
        from SupportClasses.PrintTrajectoryPlanner import Waypoint
        wp = Waypoint()
        self.assertEqual(wp.t, 0.0)
        self.assertEqual(wp.x, 0.0)
        self.assertEqual(wp.z, 0.0)
        self.assertEqual(wp.segment, "")

    def test_waypoint_with_values(self):
        from SupportClasses.PrintTrajectoryPlanner import Waypoint
        wp = Waypoint(t=1.5, x=10.0, y=20.0, z=0.1, p1=0.5,
                      segment="print", well="A1")
        self.assertEqual(wp.t, 1.5)
        self.assertEqual(wp.well, "A1")


class TestVelocityProfile(unittest.TestCase):
    """Test trapezoidal velocity profile."""

    def test_endpoints(self):
        from SupportClasses.PrintTrajectoryPlanner import _trapezoidal_fraction
        self.assertAlmostEqual(_trapezoidal_fraction(0.0), 0.0)
        self.assertAlmostEqual(_trapezoidal_fraction(1.0), 1.0)

    def test_monotonic(self):
        from SupportClasses.PrintTrajectoryPlanner import _trapezoidal_fraction
        prev = 0
        for i in range(101):
            frac = i / 100
            val = _trapezoidal_fraction(frac)
            self.assertGreaterEqual(val, prev - 1e-9)
            prev = val

    def test_midpoint_near_half(self):
        from SupportClasses.PrintTrajectoryPlanner import _trapezoidal_fraction
        mid = _trapezoidal_fraction(0.5)
        self.assertAlmostEqual(mid, 0.5, delta=0.1)


class TestPlannerMotionPrimitives(unittest.TestCase):
    """Test individual motion primitives."""

    def _make_planner(self):
        from SupportClasses.PrintTrajectoryPlanner import PrintTrajectoryPlanner
        p = PrintTrajectoryPlanner()
        p._z = 5.0
        return p

    def test_move_z_generates_waypoints(self):
        p = self._make_planner()
        p._move_z(0.1, 120.0)  # 120 mm/min
        self.assertGreater(len(p._waypoints), 0)
        # Final Z should be ~0.1
        self.assertAlmostEqual(p._waypoints[-1].z, 0.1, places=2)

    def test_move_z_timing(self):
        p = self._make_planner()
        p._z = 5.0
        p._move_z(0.0, 120.0)  # 5mm at 120mm/min = 2.5s
        duration = p._t
        self.assertAlmostEqual(duration, 2.5, delta=0.2)

    def test_move_xy_generates_waypoints(self):
        p = self._make_planner()
        p._move_xy(10.0, 0.0, 6000.0)
        self.assertGreater(len(p._waypoints), 0)
        self.assertAlmostEqual(p._waypoints[-1].x, 10.0, places=2)

    def test_move_xy_timing(self):
        p = self._make_planner()
        p._move_xy(60.0, 0.0, 6000.0)  # 60mm at 6000mm/min = 0.6s
        self.assertAlmostEqual(p._t, 0.6, delta=0.1)

    def test_dwell(self):
        p = self._make_planner()
        p._dwell(2.0)
        self.assertAlmostEqual(p._t, 2.0, delta=0.01)

    def test_pump_move(self):
        p = self._make_planner()
        p._move_pump("P1", 1.0, 30.0)  # 1mm at 30mm/min = 2s
        self.assertAlmostEqual(p._pumps["P1"], 1.0, places=2)
        self.assertAlmostEqual(p._t, 2.0, delta=0.3)


class TestFullPlanGeneration(unittest.TestCase):
    """Test complete plan-to-trajectory conversion."""

    def test_simple_print(self):
        from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType)

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1"], pump_id="P1", run_number=1),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1, total_print_wells=1,
        )

        result = plan_to_trajectory(
            plan=plan,
            well_model=_mock_well_model({"A1": "print"}),
            plate=_mock_plate([("A1", 0.0, 0.0)]),
            path_points=[(0, 0), (2, 0), (2, 2), (0, 2)],
            settings=_mock_settings(),
        )

        self.assertTrue(result.valid, f"Issues: {result.issues}")
        self.assertGreater(len(result.waypoints), 10)
        self.assertGreater(result.total_duration_s, 0)
        self.assertEqual(result.well_count, 1)

        # Should end near (0,0) after RETURN_HOME
        last = result.waypoints[-1]
        self.assertAlmostEqual(last.x, 0.0, delta=0.1)
        self.assertAlmostEqual(last.y, 0.0, delta=0.1)

    def test_multi_well_print(self):
        from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType)

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.LOAD_INK, pump_id="P1",
                         volume_uL=50.0, ink_name="Test Ink"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1", "A2", "A3"],
                         pump_id="P1", run_number=1),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1, total_print_wells=3,
        )

        result = plan_to_trajectory(
            plan=plan,
            well_model=_mock_well_model({
                "A1": "print", "A2": "print", "A3": "print",
                "D1": "ink"}),
            plate=_mock_plate([
                ("A1", 0, 0), ("A2", 19.3, 0), ("A3", 38.6, 0),
                ("D1", 0, 57.9)]),
            path_points=[(0, 0), (3, 0), (3, 3)],
            settings=_mock_settings(),
        )

        self.assertTrue(result.valid, f"Issues: {result.issues}")
        self.assertEqual(result.well_count, 3)
        self.assertGreater(result.total_duration_s, 5)

    def test_service_steps(self):
        from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType)

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.LOAD_INK, pump_id="P1",
                         volume_uL=50.0, ink_name="Test"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["B2"], pump_id="P1", run_number=1),
                PlanStep(step_type=PlanStepType.WASTE, pump_id="P1"),
                PlanStep(step_type=PlanStepType.WASH, pump_id="P1"),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1, total_print_wells=1,
        )

        result = plan_to_trajectory(
            plan=plan,
            well_model=_mock_well_model({
                "A1": "ink", "B2": "print", "C1": "waste", "C2": "wash"}),
            plate=_mock_plate([
                ("A1", 0, 0), ("B2", 19.3, 19.3),
                ("C1", 0, 38.6), ("C2", 19.3, 38.6)]),
            path_points=[(0, 0), (1, 0)],
            settings=_mock_settings(),
        )

        self.assertTrue(result.valid, f"Issues: {result.issues}")
        # Should have visited ink well, print well, waste well, wash well
        wells_visited = set(wp.well for wp in result.waypoints if wp.well)
        self.assertIn("A1", wells_visited)
        self.assertIn("B2", wells_visited)
        self.assertIn("C1", wells_visited)
        self.assertIn("C2", wells_visited)

    def test_missing_service_well_reports_issue(self):
        from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType)

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.WASTE, pump_id="P1"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1"], pump_id="P1"),
            ],
            total_runs=1, total_print_wells=1,
        )

        result = plan_to_trajectory(
            plan=plan,
            well_model=_mock_well_model({"A1": "print"}),  # NO waste well
            plate=_mock_plate([("A1", 0, 0)]),
            path_points=[(0, 0), (1, 0)],
            settings=_mock_settings(),
        )

        # Should report issue about missing waste well
        self.assertTrue(any("waste" in i.lower() for i in result.issues))

    def test_feedrates_affect_timing(self):
        from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType)

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1"], pump_id="P1"),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1, total_print_wells=1,
        )

        plate = _mock_plate([("A1", 30.0, 0.0)])
        model = _mock_well_model({"A1": "print"})
        pts = [(0, 0), (5, 0), (5, 5)]

        # Fast settings
        fast = plan_to_trajectory(plan, model, plate, pts,
                                  _mock_settings(xy_feedrate=12000, z_feedrate=240,
                                                 print_feedrate=1200))
        # Slow settings
        slow = plan_to_trajectory(plan, model, plate, pts,
                                  _mock_settings(xy_feedrate=3000, z_feedrate=60,
                                                 print_feedrate=300))

        self.assertGreater(slow.total_duration_s, fast.total_duration_s * 1.5,
                           "Slower feedrates should produce longer trajectory")

    def test_waypoints_time_monotonic(self):
        from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType)

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1", "A2"], pump_id="P1"),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1, total_print_wells=2,
        )

        result = plan_to_trajectory(
            plan=plan,
            well_model=_mock_well_model({"A1": "print", "A2": "print"}),
            plate=_mock_plate([("A1", 0, 0), ("A2", 19.3, 0)]),
            path_points=[(0, 0), (2, 0)],
            settings=_mock_settings(),
        )

        # Time must be monotonically non-decreasing
        for i in range(1, len(result.waypoints)):
            self.assertGreaterEqual(
                result.waypoints[i].t, result.waypoints[i-1].t,
                f"Time went backwards at index {i}")


if __name__ == "__main__":
    unittest.main()
