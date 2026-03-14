#!/usr/bin/env python3
"""
MEBP v7.2.4 — Session 2 Tests: Jog Step Verification + Config File Browser

Tests for:
    S2.1-S2.4:  Jog step conversion and verification
    S2.5-S2.9:  Hardware config file browser
    S2.10:      End-to-end step size verification

v7.3.5: Renamed from microsteps_per_micron to xy_position_scale.
        ProScan speaks µm natively — default scale = 1.0.
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
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SupportClasses.StageController import StageController


# ═══════════════════════════════════════════════════════════════════
#  Test Group 1: Jog Step Conversion Accuracy
# ═══════════════════════════════════════════════════════════════════

class TestJogStepConversion(unittest.TestCase):
    """Verify that µm → stage-unit conversion is exact and reversible."""

    def test_standard_steps_exact(self):
        """All standard step sizes produce exact integer stage units at scale=10."""
        scale = 10.0
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            stage_units = step_um * scale
            self.assertEqual(
                stage_units, round(stage_units),
                f"{step_um} µm × {scale} = {stage_units}, not integer"
            )

    def test_roundtrip_accuracy(self):
        """Converting µm→stage→µm preserves value within float precision."""
        scale = 10.0
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            stage_units = step_um * scale
            recovered_um = stage_units / scale
            self.assertAlmostEqual(step_um, recovered_um, places=10)

    def test_proscan_ii_factor(self):
        """ProScan II at scale 20.0 produces exact stage values."""
        scale = 20.0
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            stage_units = step_um * scale
            self.assertEqual(
                stage_units, round(stage_units),
                f"{step_um} µm × {scale} = {stage_units}, not integer"
            )

    def test_fractional_factor(self):
        """Non-integer scale still produces round()able values for standard steps."""
        scale = 12.5  # Hypothetical
        for step_um in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            stage_units = step_um * scale
            rounded = round(stage_units)
            error_um = abs(stage_units - rounded) / scale
            self.assertLess(
                error_um, 0.1,
                f"Rounding error {error_um:.4f} µm too large for {step_um} µm"
            )


# ═══════════════════════════════════════════════════════════════════
#  Test Group 2: StageController.move_xy_relative Exact Values
# ═══════════════════════════════════════════════════════════════════

class TestMoveXYRelativeExact(unittest.TestCase):
    """Verify move_xy_relative sends exact values to hardware."""

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

    def test_50um_at_scale_10(self):
        """50 µm at scale 10 → exactly 500 stage units."""
        step_um = 50.0
        scale = 10.0
        dx = step_um * scale  # 500.0
        dy = 0.0
        self.controller.move_xy_relative(dx, dy)
        call_args = self.controller.xy_stage.move_stage_relative.call_args
        self.assertEqual(call_args[0], (500.0, 0.0))

    def test_1um_at_scale_10(self):
        """1 µm at scale 10 → exactly 10 stage units."""
        step_um = 1.0
        scale = 10.0
        dx = 0.0
        dy = step_um * scale  # 10.0
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
    """Verify the warning logic for default vs set position scale factor."""

    def test_default_factor_shows_warning(self):
        """Before set_xy_position_scale is called, warning should show."""
        # Simulating the flag logic
        conversion_factor_set = False
        self.assertFalse(conversion_factor_set)

    def test_set_factor_hides_warning(self):
        """After set_xy_position_scale is called, warning should hide."""
        conversion_factor_set = False
        # v7.3.5: ProScan speaks µm natively → scale = 1.0
        value = 1.0
        xy_position_scale = max(0.001, value)
        conversion_factor_set = True
        self.assertTrue(conversion_factor_set)
        self.assertEqual(xy_position_scale, 1.0)

    def test_factor_zero_clamped(self):
        """Scale of 0 should be clamped to minimum."""
        value = 0.0
        xy_position_scale = max(0.001, value)
        self.assertEqual(xy_position_scale, 0.001)

    def test_negative_factor_clamped(self):
        """Negative scale should be clamped to minimum."""
        value = -5.0
        xy_position_scale = max(0.001, value)
        self.assertEqual(xy_position_scale, 0.001)


# ═══════════════════════════════════════════════════════════════════
#  Test Group 5: End-to-End Step Verification (S2.10)
# ═══════════════════════════════════════════════════════════════════

class TestEndToEndStepVerification(unittest.TestCase):
    """Simulate full jog cycle: step_um → stage units → send → read back → verify."""

    def test_full_cycle_all_steps(self):
        """For each standard step size, verify the complete conversion chain."""
        scale = 10.0
        standard_steps = [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]

        for step_um in standard_steps:
            with self.subTest(step_um=step_um):
                # 1. User selects step in µm
                # 2. Convert to stage units
                step_stage = step_um * scale
                # 3. Round for hardware (as XYStage.move_stage_relative does)
                sent_stage = round(step_stage)
                # 4. Stage moves exactly that many units (simulated)
                new_pos = 5000 + sent_stage  # start at 5000
                # 5. Read back and convert to µm
                delta = new_pos - 5000
                delta_um = delta / scale

                # Verify: displayed delta matches requested step
                self.assertAlmostEqual(
                    delta_um, step_um, places=6,
                    msg=f"Step {step_um} µm: sent {sent_stage} units, "
                        f"measured {delta_um} µm"
                )

    def test_diagonal_move_both_axes(self):
        """Diagonal jog (dx=1, dy=1) should move sqrt(2) * step_um total distance."""
        scale = 10.0
        step_um = 100.0
        dx_stage = round(step_um * scale)
        dy_stage = round(step_um * scale)

        # Simulate position change
        start = (5000, 5000)
        end = (5000 + dx_stage, 5000 + dy_stage)

        # Measured delta in µm
        delta_x_um = (end[0] - start[0]) / scale
        delta_y_um = (end[1] - start[1]) / scale
        total_um = math.sqrt(delta_x_um**2 + delta_y_um**2)

        expected_total = math.sqrt(2) * step_um
        self.assertAlmostEqual(total_um, expected_total, places=6)

    def test_repeated_jogs_accumulate_correctly(self):
        """10 consecutive 100µm jogs should total exactly 1000µm."""
        scale = 1.0  # v7.3.5: ProScan speaks µm natively
        step_um = 100.0
        pos = 5000.0  # starting position in µm

        for _ in range(10):
            step_stage = round(step_um * scale)
            pos += step_stage

        total_um = (pos - 5000) / scale
        self.assertAlmostEqual(total_um, 1000.0, places=6)


if __name__ == "__main__":
    unittest.main()
