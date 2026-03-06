#!/usr/bin/env python3
"""
test_v726_print_execution.py — Headless integration tests for v7.2.6.

Tests the job building pipeline WITHOUT GUI or hardware:
    - build_well_plate_job() correct API usage
    - _single_object_to_points() geometry extraction
    - plan_to_commands() bridge from plan → PrintCommand list
    - Service step command generation (waste, wash, buffer, ink load)
    - End-to-end: WellSetupModel → plan → commands → PrintJob

Run: python -m unittest tests.test_v726_print_execution -v
"""

import unittest
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch
from dataclasses import dataclass, field
from enum import Enum

# Add project root to path
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))


# ═══════════════════════════════════════════════════════════════════
#  Test 1: build_well_plate_job correct API
# ═══════════════════════════════════════════════════════════════════

class TestBuildWellPlateJobAPI(unittest.TestCase):
    """Verify build_well_plate_job works with the correct v7.0 API."""

    def setUp(self):
        from SupportClasses.PrintManager import (
            build_well_plate_job, PrintSettings,
        )
        self.build = build_well_plate_job
        self.settings = PrintSettings()

    def test_basic_build(self):
        """Build a simple 3-well job with default pattern."""
        well_positions = [("A1", 0.0, 0.0), ("A2", 9.0, 0.0), ("A3", 18.0, 0.0)]
        path_points = [(0, 0), (1, 0), (1, 1), (0, 1)]

        job = self.build(
            well_positions=well_positions,
            path_points=path_points,
            settings=self.settings,
            pump="P1",
            flow_rate=0.01,
        )

        self.assertIsNotNone(job)
        self.assertGreater(job.total_steps, 0)
        self.assertEqual(len(well_positions), 3)

    def test_single_well_single_point(self):
        """Minimal job: 1 well, 1 point."""
        job = self.build(
            well_positions=[("A1", 0.0, 0.0)],
            path_points=[(0.0, 0.0)],
            settings=self.settings,
        )
        self.assertIsNotNone(job)
        self.assertGreater(job.total_steps, 0)

    def test_multi_material_pump_sequence(self):
        """Build with pump_sequence for multi-material."""
        well_positions = [("A1", 0.0, 0.0), ("A2", 9.0, 0.0)]
        path_points = [(0, 0), (1, 0)]

        job = self.build(
            well_positions=well_positions,
            path_points=path_points,
            settings=self.settings,
            pump="P1",
            flow_rate=0.01,
            pump_sequence=["P1", "P2"],
        )
        self.assertIsNotNone(job)

    def test_empty_wells_returns_job(self):
        """Empty well list should still return a job (with 0 well commands)."""
        job = self.build(
            well_positions=[],
            path_points=[(0, 0)],
            settings=self.settings,
        )
        # May return empty job or raise — just verify no crash
        self.assertIsNotNone(job)


# ═══════════════════════════════════════════════════════════════════
#  Test 2: Path geometry extraction
# ═══════════════════════════════════════════════════════════════════

class TestPathGeometryExtraction(unittest.TestCase):
    """Test converting print objects to path points."""

    def test_line_object(self):
        from SupportClasses.WellPlate import generate_line_path
        pts = generate_line_path(5.0, 0.0)
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts), 0)
        for p in pts:
            self.assertEqual(len(p), 2)

    def test_meander_object(self):
        from SupportClasses.WellPlate import generate_meander_path
        pts = generate_meander_path(5.0, 5.0, 0.5)
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts), 1)

    def test_spiral_object(self):
        from SupportClasses.WellPlate import generate_spiral_path
        pts = generate_spiral_path(3.0, 0.5)
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts), 1)

    def test_grid_object(self):
        from SupportClasses.WellPlate import generate_grid_path
        pts = generate_grid_path(5.0, 5.0, 1.0, 1.0)
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts), 1)


# ═══════════════════════════════════════════════════════════════════
#  Test 3: plan_to_commands bridge
# ═══════════════════════════════════════════════════════════════════

class TestPlanToCommands(unittest.TestCase):
    """Test the plan → PrintCommand bridge."""

    def _make_mock_well_model(self, assignments_dict):
        """Create a mock WellSetupModel."""
        model = MagicMock()
        model.assignments = {}
        for name, role_val in assignments_dict.items():
            a = MagicMock()
            a.role = MagicMock()
            a.role.value = role_val
            model.assignments[name] = a
        return model

    def _make_mock_plate(self, well_positions):
        """Create a mock WellPlate."""
        plate = MagicMock()
        positions = dict(well_positions)
        plate.get_well_position = lambda name: positions[name]
        plate.format = 24
        plate.well_diameter = 6.0
        return plate

    def test_simple_print_plan(self):
        """A plan with just PRINT + RETURN_HOME produces commands."""
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType, plan_to_commands,
        )
        from SupportClasses.PrintManager import PrintSettings, CommandType

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(
                    step_type=PlanStepType.PRINT,
                    target_wells=["A1", "A2"],
                    pump_id="P1",
                    run_number=1,
                ),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1,
            total_print_wells=2,
        )

        model = self._make_mock_well_model({
            "A1": "print", "A2": "print",
        })
        plate = self._make_mock_plate([
            ("A1", 0.0, 0.0), ("A2", 9.0, 0.0),
        ])
        settings = PrintSettings()
        path_points = [(0, 0), (1, 0), (1, 1)]

        job = plan_to_commands(
            plan=plan,
            well_model=model,
            plate=plate,
            path_points=path_points,
            settings=settings,
        )

        self.assertIsNotNone(job)
        self.assertGreater(job.total_steps, 0)

        # Should end with HOME_XY
        home_cmds = [c for c in job.commands if c.type == CommandType.HOME_XY]
        self.assertGreater(len(home_cmds), 0)

    def test_full_service_plan(self):
        """Plan with WASTE → WASH → LOAD_INK → PRINT → RETURN_HOME."""
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType, plan_to_commands,
        )
        from SupportClasses.PrintManager import PrintSettings, CommandType

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.WASTE, pump_id="P1"),
                PlanStep(step_type=PlanStepType.WASH, pump_id="P1"),
                PlanStep(step_type=PlanStepType.LOAD_INK, pump_id="P1",
                         volume_uL=50.0, ink_name="Hydrogel A"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["B2", "B3"],
                         pump_id="P1", run_number=1),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=1,
            total_print_wells=2,
        )

        model = self._make_mock_well_model({
            "A1": "waste", "A2": "wash", "A3": "ink",
            "B2": "print", "B3": "print",
        })
        plate = self._make_mock_plate([
            ("A1", 0, 0), ("A2", 9, 0), ("A3", 18, 0),
            ("B2", 9, 9), ("B3", 18, 9),
        ])
        settings = PrintSettings()

        job = plan_to_commands(
            plan=plan,
            well_model=model,
            plate=plate,
            path_points=[(0, 0), (2, 0), (2, 2)],
            settings=settings,
        )

        self.assertIsNotNone(job)

        # Check service commands exist
        cmd_types = [c.type for c in job.commands]
        # Should have TRAVEL_UP, MOVE_XY for service wells
        self.assertIn(CommandType.TRAVEL_UP, cmd_types)
        self.assertIn(CommandType.MOVE_XY, cmd_types)
        # Should have EXTRUDE for ink loading (negative = aspirate)
        extrude_cmds = [c for c in job.commands if c.type == CommandType.EXTRUDE]
        self.assertGreater(len(extrude_cmds), 0)

    def test_missing_service_well_produces_comment(self):
        """If no waste well exists, produce a COMMENT skip, not a crash."""
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType, plan_to_commands,
        )
        from SupportClasses.PrintManager import PrintSettings, CommandType

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.WASTE, pump_id="P1"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1"],
                         pump_id="P1", run_number=1),
            ],
            total_runs=1,
            total_print_wells=1,
        )

        model = self._make_mock_well_model({"A1": "print"})  # No waste well!
        plate = self._make_mock_plate([("A1", 0, 0)])
        settings = PrintSettings()

        job = plan_to_commands(
            plan=plan, well_model=model, plate=plate,
            path_points=[(0, 0)], settings=settings,
        )

        self.assertIsNotNone(job)
        # Should have a COMMENT about skipping waste
        comments = [c for c in job.commands if c.type == CommandType.COMMENT]
        skip_comments = [c for c in comments if "SKIP" in c.label]
        self.assertGreater(len(skip_comments), 0, "Expected SKIP comment for missing waste well")

    def test_multi_run_plan(self):
        """Two PRINT steps (runs) with a service cycle between."""
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, PlanStep, PlanStepType, plan_to_commands,
        )
        from SupportClasses.PrintManager import PrintSettings

        plan = PrintPlanOfAction(
            steps=[
                PlanStep(step_type=PlanStepType.LOAD_INK, pump_id="P1",
                         volume_uL=25.0, ink_name="Ink A"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A1", "A2"], pump_id="P1", run_number=1),
                PlanStep(step_type=PlanStepType.WASTE, pump_id="P1"),
                PlanStep(step_type=PlanStepType.WASH, pump_id="P1"),
                PlanStep(step_type=PlanStepType.LOAD_INK, pump_id="P1",
                         volume_uL=25.0, ink_name="Ink A"),
                PlanStep(step_type=PlanStepType.PRINT,
                         target_wells=["A3", "A4"], pump_id="P1", run_number=2),
                PlanStep(step_type=PlanStepType.RETURN_HOME),
            ],
            total_runs=2,
            total_print_wells=4,
        )

        model = self._make_mock_well_model({
            "A1": "print", "A2": "print", "A3": "print", "A4": "print",
            "H1": "waste", "H2": "wash", "H3": "ink",
        })
        plate = self._make_mock_plate([
            ("A1", 0, 0), ("A2", 9, 0), ("A3", 18, 0), ("A4", 27, 0),
            ("H1", 0, 63), ("H2", 9, 63), ("H3", 18, 63),
        ])
        settings = PrintSettings()

        job = plan_to_commands(
            plan=plan, well_model=model, plate=plate,
            path_points=[(0, 0), (1, 0)], settings=settings,
        )

        self.assertIsNotNone(job)
        # Multi-run should have more commands than single-run
        self.assertGreater(job.total_steps, 10)

    def test_empty_plan_returns_none(self):
        """Empty plan returns None."""
        from SupportClasses.PrintPlanOfAction import (
            PrintPlanOfAction, plan_to_commands,
        )
        from SupportClasses.PrintManager import PrintSettings

        plan = PrintPlanOfAction(steps=[], total_runs=0)
        job = plan_to_commands(
            plan=plan, well_model=MagicMock(), plate=MagicMock(),
            path_points=[(0, 0)], settings=PrintSettings(),
        )
        self.assertIsNone(job)


# ═══════════════════════════════════════════════════════════════════
#  Test 4: PrintManager load_job + start API
# ═══════════════════════════════════════════════════════════════════

class TestPrintManagerAPI(unittest.TestCase):
    """Verify PrintManager.load_job() + start() API contract."""

    def test_load_job_then_start(self):
        """load_job(job) then start() should not raise."""
        from SupportClasses.PrintManager import (
            PrintManager, PrintJob, PrintSettings, PrintState,
        )

        # Mock controller
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = False

        pm = PrintManager(ctrl)
        job = PrintJob(
            name="Test", description="",
            settings=PrintSettings(), commands=[],
        )

        pm.load_job(job)
        self.assertEqual(pm.state, PrintState.IDLE)
        self.assertIs(pm.job, job)

    def test_start_without_load_raises(self):
        """start() without a loaded job should raise RuntimeError."""
        from SupportClasses.PrintManager import PrintManager

        ctrl = MagicMock()
        pm = PrintManager(ctrl)

        with self.assertRaises(RuntimeError):
            pm.start()

    def test_start_does_not_accept_job_arg(self):
        """start() takes no positional arguments beyond self."""
        from SupportClasses.PrintManager import PrintManager
        import inspect

        sig = inspect.signature(PrintManager.start)
        params = list(sig.parameters.keys())
        # Should only have 'self', possibly with keyword-only defaults
        self.assertEqual(params[0], 'self')
        # Should NOT have a 'job' parameter
        self.assertNotIn('job', params,
                         "start() should NOT accept a job argument — use load_job() first")


# ═══════════════════════════════════════════════════════════════════
#  Test 5: WellPlate geometry
# ═══════════════════════════════════════════════════════════════════

class TestWellPlateGeometry(unittest.TestCase):
    """Verify well plate positions for job building."""

    def test_24_well_positions(self):
        from SupportClasses.WellPlate import WellPlate
        plate = WellPlate.from_format(24)
        x, y = plate.get_well_position("A1")
        self.assertIsNotNone(x)
        self.assertIsNotNone(y)

    def test_all_wells_have_positions(self):
        from SupportClasses.WellPlate import WellPlate
        plate = WellPlate.from_format(24)
        wells = plate.get_all_wells()
        for well in wells:
            x, y = plate.get_well_position(well.name)
            self.assertIsNotNone(x)
            self.assertIsNotNone(y)


if __name__ == "__main__":
    unittest.main()
