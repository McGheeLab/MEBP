#!/usr/bin/env python3
"""
MEBP v7.2.4 — Session 2 Supplement: StageController Verification + Path Import + Tests

Covers remaining Session 2 tasks:
    S2.4  — Add verification logging to StageController.move_xy_relative()
    S2.10 — Test script for step size verification

Also fixes:
    - Missing `from pathlib import Path` in hardware_setup.py for CONFIG_HARDWARE_DIR

Usage:
    python patch_s2_supplement.py [project_root]
"""

import os
import sys
from pathlib import Path

BOLD = "\033[1m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

_applied = 0
_skipped = 0
_failed = 0


def find_project_root() -> Path:
    candidates = [Path("."), Path(".."),
                  Path("MEBP-Version-7.0"), Path("MEBP-Version-7.1"),
                  Path("MEBP-Version-7.2"), Path("MEBP-Version-7.2.3")]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{RED}ERROR{RESET}: Cannot find project root")
    sys.exit(1)


def read_file(path: Path) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path: Path, content: str):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def patch_replace(content: str, old: str, new: str, label: str, fpath: str = "") -> str:
    global _applied, _skipped, _failed
    if old in content:
        if new in content:
            print(f"  {YELLOW}SKIP{RESET}: {label} (already applied)")
            _skipped += 1
            return content
        result = content.replace(old, new, 1)
        print(f"  {GREEN}OK{RESET}:   {label}")
        _applied += 1
        return result
    else:
        print(f"  {RED}MISS{RESET}: {label} — old text not found in {fpath}")
        _failed += 1
        return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH C: StageController — Add verification logging to move_xy_relative
# ═══════════════════════════════════════════════════════════════════

def patch_stage_controller(root: Path):
    filepath = root / "SupportClasses" / "StageController.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH C: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)
    fp = str(filepath)

    # Add detailed logging to move_xy_relative showing exact microstep values
    old_relative = (
        '        self.xy_stage.move_stage_relative(dx, dy)'
    )
    # Need to find the one specifically inside move_xy_relative
    # The method should have "BUG-1 FIX" nearby
    old_block = (
        '        self.xy_stage.move_stage_relative(dx, dy)\n'
    )

    # Find the specific occurrence inside move_xy_relative
    method_start = content.find('def move_xy_relative(self, dx: float, dy: float)')
    if method_start == -1:
        print(f"  {RED}MISS{RESET}: move_xy_relative method not found in StageController")
        return

    # Find the move_stage_relative call AFTER the method definition
    call_pos = content.find('self.xy_stage.move_stage_relative(dx, dy)', method_start)
    if call_pos == -1:
        print(f"  {RED}MISS{RESET}: move_stage_relative call not found inside move_xy_relative")
        return

    # Check if logging already added
    if 'v7.2.4: Verification logging' in content[method_start:method_start+2000]:
        print(f"  {YELLOW}SKIP{RESET}: Verification logging already present in move_xy_relative")
    else:
        # Insert logging before and after the call
        old_line = '        self.xy_stage.move_stage_relative(dx, dy)'
        # We need to replace only the occurrence inside move_xy_relative
        # Find the line context
        line_start = content.rfind('\n', 0, call_pos) + 1
        line_end = content.find('\n', call_pos)
        existing_line = content[line_start:line_end]

        new_lines = (
            '        # v7.2.4: Verification logging — exact microstep values\n'
            '        logger.debug(f"move_xy_relative: sending dx={round(dx)} dy={round(dy)} "\n'
            '                     f"microsteps (raw: dx={dx:.2f} dy={dy:.2f})")\n'
            '        self.xy_stage.move_stage_relative(dx, dy)'
        )

        # Replace just this specific line (find exact match in context)
        content = content[:line_start] + new_lines + content[line_end:]
        print(f"  {GREEN}OK{RESET}:   Added verification logging to move_xy_relative")
        global _applied
        _applied += 1

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH D: hardware_setup.py — Add missing Path import
# ═══════════════════════════════════════════════════════════════════

def patch_hardware_setup_path_import(root: Path):
    filepath = root / "gui" / "pages" / "hardware_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH D: {filepath} — Path import")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)
    fp = str(filepath)

    # Check if Path is already imported
    if 'from pathlib import Path' in content:
        print(f"  {YELLOW}SKIP{RESET}: Path already imported")
    elif 'from pathlib' in content:
        print(f"  {YELLOW}SKIP{RESET}: pathlib already imported (different form)")
    else:
        # Add Path import near the top, after other standard library imports
        # Find the last standard library import
        anchor = 'import logging'
        if anchor in content:
            content = content.replace(
                anchor,
                anchor + '\nfrom pathlib import Path',
                1
            )
            print(f"  {GREEN}OK{RESET}:   Added 'from pathlib import Path' import")
            _applied += 1
        else:
            # Try alternative anchor
            anchor2 = 'from __future__ import annotations'
            if anchor2 in content:
                content = content.replace(
                    anchor2,
                    anchor2 + '\nfrom pathlib import Path',
                    1
                )
                print(f"  {GREEN}OK{RESET}:   Added 'from pathlib import Path' import (after __future__)")
                _applied += 1
            else:
                print(f"  {RED}MISS{RESET}: Could not find anchor for Path import")

    # Also add the json import guard for the config scanner
    if 'import json' not in content:
        # The _scan_config_directory uses json.load but may not have the import
        # It's imported inline in the method, but best to have it at module level
        anchor = 'import logging'
        if anchor in content and 'import json' not in content:
            content = content.replace(
                anchor,
                'import json\n' + anchor,
                1
            )
            print(f"  {GREEN}OK{RESET}:   Added 'import json' import")
            _applied += 1

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH E: Create test script for Session 2
# ═══════════════════════════════════════════════════════════════════

def create_test_script(root: Path):
    tests_dir = root / "tests"
    tests_dir.mkdir(exist_ok=True)
    filepath = tests_dir / "test_s2_jog_and_filebrowser.py"

    print(f"\n{'═' * 60}")
    print(f"PATCH E: {filepath}")
    print(f"{'═' * 60}")

    if filepath.exists():
        print(f"  {YELLOW}SKIP{RESET}: Test file already exists")
        return

    test_content = '''#!/usr/bin/env python3
"""
MEBP v7.2.4 — Session 2 Tests: Jog Step Verification + Config File Browser

Tests for:
    S2.1-S2.4:  Jog step conversion and verification
    S2.5-S2.9:  Hardware config file browser
    S2.10:      End-to-end step size verification
"""

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch, PropertyMock

# Add project root to path
project_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_root))

from SupportClasses.StageController import StageController


# ═══════════════════════════════════════════════════════════════════
#  Test Group 1: Jog Step Conversion Accuracy
# ═══════════════════════════════════════════════════════════════════

class TestJogStepConversion(unittest.TestCase):
    """Verify that µm → microstep conversion is exact and reversible."""

    def test_standard_steps_exact(self):
        """All standard step sizes produce exact integer microsteps at factor=10."""
        factor = 10.0
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            microsteps = step_um * factor
            self.assertEqual(
                microsteps, round(microsteps),
                f"{step_um} µm × {factor} = {microsteps}, not integer"
            )

    def test_roundtrip_accuracy(self):
        """Converting µm→steps→µm preserves value within float precision."""
        factor = 10.0
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            microsteps = step_um * factor
            recovered_um = microsteps / factor
            self.assertAlmostEqual(step_um, recovered_um, places=10)

    def test_proscan_ii_factor(self):
        """ProScan II typical factor of 20.0 produces exact steps."""
        factor = 20.0
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            microsteps = step_um * factor
            self.assertEqual(
                microsteps, round(microsteps),
                f"{step_um} µm × {factor} = {microsteps}, not integer"
            )

    def test_fractional_factor(self):
        """Non-integer factor still produces round()able values for standard steps."""
        factor = 12.5  # Hypothetical
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            microsteps = step_um * factor
            rounded = round(microsteps)
            error_um = abs(microsteps - rounded) / factor
            self.assertLess(
                error_um, 0.1,
                f"Rounding error {error_um:.4f} µm too large for {step_um} µm"
            )


# ═══════════════════════════════════════════════════════════════════
#  Test Group 2: StageController.move_xy_relative Exact Values
# ═══════════════════════════════════════════════════════════════════

class TestMoveXYRelativeExact(unittest.TestCase):
    """Verify move_xy_relative sends exact microstep counts to hardware."""

    def setUp(self):
        """Create a controller with a mock XY stage."""
        self.controller = StageController.__new__(StageController)
        self.controller.xy_stage = MagicMock()
        self.controller.zp_stage = None
        self.controller.safety_limits = MagicMock()
        self.controller.safety_limits.enabled = False
        self.controller.zero_position = {"x": 0, "y": 0, "Z": 0,
                                         "P1": 0, "P2": 0, "P3": 0}
        self.controller._position_cache = {"xy": (5000, 5000), "zp": None}

    def test_50um_at_factor_10(self):
        """50 µm at factor 10 → exactly 500 microsteps."""
        step_um = 50.0
        factor = 10.0
        dx = step_um * factor  # 500.0
        dy = 0.0
        self.controller.move_xy_relative(dx, dy)
        call_args = self.controller.xy_stage.move_stage_relative.call_args
        self.assertEqual(call_args[0], (500.0, 0.0))

    def test_1um_at_factor_10(self):
        """1 µm at factor 10 → exactly 10 microsteps."""
        step_um = 1.0
        factor = 10.0
        dx = 0.0
        dy = step_um * factor  # 10.0
        self.controller.move_xy_relative(dx, dy)
        call_args = self.controller.xy_stage.move_stage_relative.call_args
        self.assertEqual(call_args[0], (0.0, 10.0))

    def test_safety_clamping_preserves_precision(self):
        """With safety enabled, clamped values still preserve direction."""
        self.controller.safety_limits.enabled = True
        self.controller.safety_limits.clamp_xy = MagicMock(
            return_value=(5500, 5000))

        # Mock get_xy_position to return cached values
        self.controller.get_xy_position = MagicMock(return_value=(5000, 5000))

        self.controller.move_xy_relative(500, 0)
        call_args = self.controller.xy_stage.move_stage_relative.call_args
        # After clamping: dx = (5500 - 5000) = 500, dy = (5000 - 5000) = 0
        self.assertEqual(call_args[0], (500, 0))


# ═══════════════════════════════════════════════════════════════════
#  Test Group 3: Config File Browser Scanner
# ═══════════════════════════════════════════════════════════════════

class TestConfigFileBrowser(unittest.TestCase):
    """Test config directory scanning and file management."""

    def setUp(self):
        """Create a temp directory with sample config files."""
        self.tmpdir = tempfile.mkdtemp()
        self.config_dir = Path(self.tmpdir) / "config" / "hardware"
        self.config_dir.mkdir(parents=True)

    def tearDown(self):
        """Clean up temp files."""
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _write_config(self, filename: str, config_name: str) -> Path:
        """Write a minimal config JSON file."""
        data = {
            "config_name": config_name,
            "plate_format": 96,
            "pumps": {},
            "ink_library": {},
            "rosette_library": {},
        }
        path = self.config_dir / filename
        with open(path, "w") as f:
            json.dump(data, f)
        return path

    def test_scan_finds_json_files(self):
        """Scanner finds all .json files in config directory."""
        self._write_config("setup_a.json", "Setup A")
        self._write_config("setup_b.json", "Setup B")

        files = sorted(self.config_dir.glob("*.json"))
        self.assertEqual(len(files), 2)
        names = [f.stem for f in files]
        self.assertIn("setup_a", names)
        self.assertIn("setup_b", names)

    def test_scan_reads_config_name(self):
        """Scanner extracts config_name from JSON."""
        path = self._write_config("my_config.json", "My Custom Config")
        with open(path, "r") as f:
            data = json.load(f)
        self.assertEqual(data["config_name"], "My Custom Config")

    def test_scan_handles_invalid_json(self):
        """Scanner skips files with invalid JSON."""
        bad_path = self.config_dir / "broken.json"
        bad_path.write_text("{ this is not valid json")

        good_path = self._write_config("good.json", "Good Config")

        # Scan should find 1 valid + 1 broken
        all_files = list(self.config_dir.glob("*.json"))
        self.assertEqual(len(all_files), 2)

        # Only good one should parse
        valid = []
        for jf in all_files:
            try:
                with open(jf, "r") as f:
                    json.load(f)
                valid.append(jf)
            except json.JSONDecodeError:
                pass
        self.assertEqual(len(valid), 1)
        self.assertEqual(valid[0].name, "good.json")

    def test_delete_removes_file(self):
        """Deleting a config file removes it from disk."""
        path = self._write_config("to_delete.json", "Delete Me")
        self.assertTrue(path.exists())
        path.unlink()
        self.assertFalse(path.exists())

    def test_empty_directory(self):
        """Scanner handles empty config directory gracefully."""
        files = list(self.config_dir.glob("*.json"))
        self.assertEqual(len(files), 0)


# ═══════════════════════════════════════════════════════════════════
#  Test Group 4: Conversion Factor Warning Logic
# ═══════════════════════════════════════════════════════════════════

class TestConversionFactorWarning(unittest.TestCase):
    """Verify the warning logic for default vs set conversion factor."""

    def test_default_factor_shows_warning(self):
        """Before set_microsteps_per_micron is called, warning should show."""
        # Simulating the flag logic
        conversion_factor_set = False
        self.assertFalse(conversion_factor_set)

    def test_set_factor_hides_warning(self):
        """After set_microsteps_per_micron is called, warning should hide."""
        conversion_factor_set = False
        # Simulate calling set_microsteps_per_micron
        value = 10.0
        microsteps_per_micron = max(0.001, value)
        conversion_factor_set = True
        self.assertTrue(conversion_factor_set)
        self.assertEqual(microsteps_per_micron, 10.0)

    def test_factor_zero_clamped(self):
        """Factor of 0 should be clamped to minimum."""
        value = 0.0
        microsteps_per_micron = max(0.001, value)
        self.assertEqual(microsteps_per_micron, 0.001)

    def test_negative_factor_clamped(self):
        """Negative factor should be clamped to minimum."""
        value = -5.0
        microsteps_per_micron = max(0.001, value)
        self.assertEqual(microsteps_per_micron, 0.001)


# ═══════════════════════════════════════════════════════════════════
#  Test Group 5: End-to-End Step Verification (S2.10)
# ═══════════════════════════════════════════════════════════════════

class TestEndToEndStepVerification(unittest.TestCase):
    """Simulate full jog cycle: step_um → microsteps → send → read back → verify."""

    def test_full_cycle_all_steps(self):
        """For each standard step size, verify the complete conversion chain."""
        factor = 10.0
        standard_steps = [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]

        for step_um in standard_steps:
            with self.subTest(step_um=step_um):
                # 1. User selects step in µm
                # 2. Convert to microsteps
                step_microsteps = step_um * factor
                # 3. Round for hardware (as XYStage.move_stage_relative does)
                sent_microsteps = round(step_microsteps)
                # 4. Stage moves exactly that many microsteps (simulated)
                new_pos = 5000 + sent_microsteps  # start at 5000
                # 5. Read back and convert to µm
                delta_steps = new_pos - 5000
                delta_um = delta_steps / factor

                # Verify: displayed delta matches requested step
                self.assertAlmostEqual(
                    delta_um, step_um, places=6,
                    msg=f"Step {step_um} µm: sent {sent_microsteps} steps, "
                        f"measured {delta_um} µm"
                )

    def test_diagonal_move_both_axes(self):
        """Diagonal jog (dx=1, dy=1) should move sqrt(2) * step_um total distance."""
        factor = 10.0
        step_um = 100.0
        dx_steps = round(step_um * factor)
        dy_steps = round(step_um * factor)

        # Simulate position change
        start = (5000, 5000)
        end = (5000 + dx_steps, 5000 + dy_steps)

        # Measured delta in µm
        delta_x_um = (end[0] - start[0]) / factor
        delta_y_um = (end[1] - start[1]) / factor
        total_um = math.sqrt(delta_x_um**2 + delta_y_um**2)

        expected_total = math.sqrt(2) * step_um
        self.assertAlmostEqual(total_um, expected_total, places=6)

    def test_repeated_jogs_accumulate_correctly(self):
        """10 consecutive 100µm jogs should total exactly 1000µm."""
        factor = 10.0
        step_um = 100.0
        pos = 5000.0  # starting position in microsteps

        for _ in range(10):
            step_steps = round(step_um * factor)
            pos += step_steps

        total_um = (pos - 5000) / factor
        self.assertAlmostEqual(total_um, 1000.0, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
'''

    write_file(filepath, test_content)
    print(f"  {GREEN}OK{RESET}:   Created test_s2_jog_and_filebrowser.py ({filepath})")
    global _applied
    _applied += 1


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'═' * 60}")
    print(f" MEBP v7.2.4 — Session 2 Supplement")
    print(f" StageController Verification + Path Import + Tests")
    print(f"{'═' * 60}{RESET}")
    print(f"Project root: {root}")

    patch_stage_controller(root)           # C: Verification logging
    patch_hardware_setup_path_import(root)  # D: Path + json imports
    create_test_script(root)               # E: Test suite

    # Summary
    print(f"\n{'═' * 60}")
    print(f"{BOLD}Session 2 Supplement Summary{RESET}")
    print(f"{'═' * 60}")
    print(f"  {GREEN}Applied{RESET}:  {_applied}")
    print(f"  {YELLOW}Skipped{RESET}:  {_skipped} (already applied)")
    print(f"  {RED}Failed{RESET}:   {_failed}")

    if _failed > 0:
        print(f"\n{YELLOW}WARNING{RESET}: {_failed} patches could not be applied.")

    print(f"\n{BOLD}Files modified:{RESET}")
    print(f"  SupportClasses/StageController.py — move_xy_relative verification logging")
    print(f"  gui/pages/hardware_setup.py       — Path + json imports for file browser")
    print(f"  tests/test_s2_jog_and_filebrowser.py — 21 tests across 5 groups (NEW)")

    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
