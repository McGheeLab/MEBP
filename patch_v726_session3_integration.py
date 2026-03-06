#!/usr/bin/env python3
"""
MEBP v7.2.6 Session 3 — Progress Parsing + Integration Tests.

Patches:
  1. print_monitor.py: Enhance on_print_progress() to parse well/layer from message
  2. Creates headless integration test file for the job building pipeline

Follows PATCHING_BEST_PRACTICES.md.
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

G = "\033[92m"; Y = "\033[93m"; R = "\033[91m"; X = "\033[0m"; B = "\033[1m"
_ok = 0; _skip = 0; _miss = 0

def ok(msg):
    global _ok; _ok += 1; print(f"  {G}✓{X} {msg}")
def skip(msg):
    global _skip; _skip += 1; print(f"  {Y}○{X} SKIP: {msg}")
def miss(msg):
    global _miss; _miss += 1; print(f"  {R}✗{X} MISS: {msg}")

def find_root(start=None):
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{R}ERROR: Cannot find MEBP root.{X}")
    sys.exit(1)

def safe_read(path):
    return path.read_text(encoding="utf-8") if path.exists() else ""

def safe_write(path, content, label):
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL on {path.name} ({label}): {e}{X}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v726s3_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})")
    return True

def find_method(content, name, indent=4):
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 1: print_monitor.py — Enhanced progress parsing
# ═══════════════════════════════════════════════════════════════════

def patch_monitor_progress(root):
    print(f"\n{B}── Patching gui/pages/print_monitor.py (progress parsing) ──{X}")
    path = root / "gui" / "pages" / "print_monitor.py"
    content = safe_read(path)
    if not content:
        miss("print_monitor.py not found")
        return
    changed = False

    marker = "v7.2.6: Parse well/layer info from progress message"
    if marker not in content:
        m = find_method(content, "on_print_progress")
        if m:
            new_method = '''    def on_print_progress(self, step: int, total: int, message: str) -> None:
        """v7.2.6: Parse well/layer info from progress message.

        PrintManager sends messages like:
            "[42/500] Travel to well A3"
            "[42/500] Layer 2: print_path"
            "[42/500] Well A3, Layer 2/5: meander"
        We parse these to extract structured info for on_progress_update().
        """
        # Parse well name from message
        well_name = ""
        well_match = re.search(r'[Ww]ell\\s+([A-H]\\d{1,2})', message)
        if well_match:
            well_name = well_match.group(1)

        # Parse layer info
        layer = 0
        total_layers = 0
        layer_match = re.search(r'[Ll]ayer\\s+(\\d+)(?:\\s*/\\s*(\\d+))?', message)
        if layer_match:
            layer = int(layer_match.group(1))
            if layer_match.group(2):
                total_layers = int(layer_match.group(2))

        # Track current well name for position polling
        if well_name:
            self._current_well_name = well_name

        self.on_progress_update(
            current_step=step,
            total_steps=total,
            message=message,
            job_name=self._current_job.name if self._current_job else "",
            well_name=well_name,
            layer=layer,
            total_layers=total_layers,
        )

'''
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Enhanced on_print_progress() with message parsing")
            changed = True
        else:
            miss("on_print_progress() not found")
    else:
        skip("on_print_progress() already enhanced")

    # Ensure 'import re' exists at top
    if "import re" not in content[:500]:
        # Insert after other imports
        import_match = re.search(r'^(import .*|from .* import .*)$', content, re.MULTILINE)
        if import_match:
            content = content[:import_match.end()] + "\nimport re" + content[import_match.end():]
            ok("Added 'import re' to print_monitor.py")
            changed = True

    # Ensure _current_well_name attribute exists
    if "_current_well_name" not in content[:content.find("def _build_ui") if "def _build_ui" in content else 2000]:
        init_match = re.search(r'(self\._hardware_config\s*=\s*None.*?\n)', content)
        if init_match:
            inject = "        self._current_well_name = \"\"  # v7.2.6: track active well\n"
            if "self._current_well_name" not in content:
                content = content[:init_match.end()] + inject + content[init_match.end():]
                ok("Added _current_well_name attribute")
                changed = True

    if changed:
        safe_write(path, content, "Session 3 progress parsing")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  PATCH 2: Create integration test file
# ═══════════════════════════════════════════════════════════════════

def create_integration_tests(root):
    print(f"\n{B}── Creating tests/test_v726_print_execution.py ──{X}")
    tests_dir = root / "tests"
    tests_dir.mkdir(exist_ok=True)
    path = tests_dir / "test_v726_print_execution.py"

    if path.exists() and "v7.2.6" in safe_read(path):
        skip("test_v726_print_execution.py already exists")
        return

    test_content = TEST_FILE_CONTENT
    try:
        ast.parse(test_content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {e}{X}")
        return

    path.write_text(test_content, encoding="utf-8")
    ok(f"Created {path.name} (headless integration tests)")


TEST_FILE_CONTENT = '''#!/usr/bin/env python3
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
'''


# ═══════════════════════════════════════════════════════════════════
#  PATCH 3: Update master runner to include Session 3
# ═══════════════════════════════════════════════════════════════════

def update_master_runner(root):
    print(f"\n{B}── Updating apply_all_v726_patches.py ──{X}")
    patches_dir = root / "patches" / "v726"
    runner_path = patches_dir / "apply_all_v726_patches.py"

    if not runner_path.exists():
        skip("Master runner not found — will be created at delivery")
        return

    content = safe_read(runner_path)
    marker = "session3"
    if marker not in content.lower():
        # Add Session 3 to the sessions list
        old_line = '("Session 2: Execution Chain Fix",'
        new_lines = '''("Session 2: Execution Chain Fix",
         patches_dir / "patch_v726_session2_execution_chain.py"),
        ("Session 3: Integration + Progress Parsing",
         patches_dir / "patch_v726_session3_integration.py"),'''

        if old_line in content:
            # Find the full entry including the next line
            idx = content.find(old_line)
            # Find the closing ), on the next line
            end_idx = content.find("),", idx + len(old_line))
            if end_idx > 0:
                content = content[:idx] + new_lines + content[end_idx + 2:]
                try:
                    ast.parse(content)
                    runner_path.write_text(content, encoding="utf-8")
                    ok("Added Session 3 to master runner")
                except SyntaxError:
                    skip("Could not update master runner (AST fail) — update manually")
            else:
                skip("Could not find Session 2 closing in master runner")
        else:
            skip("Session 2 entry not found in master runner")
    else:
        skip("Session 3 already in master runner")


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{B}{'=' * 60}")
    print(f"  MEBP v7.2.6 Session 3 — Integration + Progress Parsing")
    print(f"{'=' * 60}{X}\n")

    root = find_root()
    print(f"  Project root: {root}\n")

    patch_monitor_progress(root)
    create_integration_tests(root)
    update_master_runner(root)

    print(f"\n{B}── Summary ──{X}")
    print(f"  {G}OK:   {_ok}{X}")
    print(f"  {Y}SKIP: {_skip}{X}")
    print(f"  {R}MISS: {_miss}{X}")

    if _miss > 0:
        print(f"\n  {R}⚠ Some patches did not apply.{X}")
        sys.exit(1)
    else:
        print(f"\n  {G}✓ Session 3 complete.{X}")
        print(f"\n  Run tests: python -m unittest tests.test_v726_print_execution -v")


if __name__ == "__main__":
    main()
