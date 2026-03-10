#!/usr/bin/env python3
"""
Simulator vs. Real Hardware Conformance Tests + Metrics Report
===============================================================

Runs a comprehensive battery of tests against both simulated and (optionally)
real XY (Prior ProScan) and ZP (Marlin) stages, collecting quantitative
metrics that are written to a JSON report for tuning simulator parameters.

Output Files:
    config/sim_calibration_report.json  — full metrics + tuning recommendations

Usage:
    # Sim-only (no hardware required):
    python -m unittest tests.test_sim_vs_hardware -v

    # With real XY hardware connected:
    MEBP_TEST_REAL_XY=1 python -m unittest tests.test_sim_vs_hardware -v

    # With both real stages:
    MEBP_TEST_REAL_XY=1 MEBP_TEST_REAL_ZP=1 python -m unittest tests.test_sim_vs_hardware -v

    # Skip real hardware even if connected:
    MEBP_TEST_SKIP_REAL=1 python -m unittest tests.test_sim_vs_hardware -v

Sections:
    1. XYStageManager simulated-only tests
    2. ZPStageManager simulated-only tests
    3. XY Sim-vs-Hardware comparison (skipped if no hardware)
    4. ZP Sim-vs-Hardware comparison (skipped if no hardware)
    5. Cross-stage integration tests (sim only)
    6. Timing / performance characterisation
    7. Simulator internal consistency
    8. Edge cases & error handling
    9. Lifecycle tests

v7.2.5 — Session: Sim-vs-Hardware conformance + calibration report
"""

from __future__ import annotations

import json
import logging
import math
import os
import statistics
import sys
import time
import unittest
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

# ── Ensure project root is on sys.path ────────────────────────────
_THIS_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _THIS_DIR.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# ── Imports from MEBP ────────────────────────────────────────────
from SupportClasses.XYStage import XYStageManager
from SupportClasses.XYStageSimulator import XYStageSimulator
from SupportClasses.ZPStage import ZPStageManager, AXIS_MAP, AXIS_MAP_REVERSE
from SupportClasses.ZPStageSimulator import ZPStageSimulator

logger = logging.getLogger(__name__)

# ── Environment flags ─────────────────────────────────────────────
SKIP_REAL = os.environ.get("MEBP_TEST_SKIP_REAL", "").strip() == "1"
FORCE_REAL_XY = os.environ.get("MEBP_TEST_REAL_XY", "").strip() == "1"
FORCE_REAL_ZP = os.environ.get("MEBP_TEST_REAL_ZP", "").strip() == "1"

# Report output path
_REPORT_PATH = _PROJECT_ROOT / "config" / "sim_calibration_report.json"

# ── Marlin hardware safety constants ──────────────────────────────
# Marlin has a small serial command buffer (4-16 slots). Sending commands
# faster than the board can process them causes buffer overflow → watchdog
# reset → board needs physical power-cycle.
#
# These pacing constants prevent that. They apply ONLY to real hardware;
# simulator tests run at full speed.
MARLIN_CMD_INTERVAL_S = 0.10   # 100 ms between commands to real Marlin
MARLIN_MOVE_SETTLE_S = 0.80    # Extra settle time after real Marlin moves
PROSCAN_CMD_INTERVAL_S = 0.02  # 20 ms between commands to real ProScan


# ═══════════════════════════════════════════════════════════════════
#  METRICS COLLECTOR — Global singleton for accumulating measurements
# ═══════════════════════════════════════════════════════════════════

class MetricsCollector:
    """
    Accumulates quantitative measurements during test runs.

    After all tests complete, ``write_report()`` generates a JSON file
    mapping every measurement to the specific simulator parameter it
    informs, along with suggested values when real hardware data is
    available.
    """

    def __init__(self):
        # Raw sample buckets — each key maps to a list of float values
        self._samples: dict[str, list[float]] = {}
        # Structured result records (richer than just a float)
        self._records: dict[str, dict] = {}
        # Hardware availability flags
        self.real_xy_available: bool = False
        self.real_zp_available: bool = False

    def add_sample(self, metric: str, value: float) -> None:
        """Append a single measurement to a named bucket."""
        self._samples.setdefault(metric, []).append(value)

    def add_samples(self, metric: str, values: list[float]) -> None:
        """Append a batch of measurements."""
        self._samples.setdefault(metric, []).extend(values)

    def set_record(self, key: str, data: dict) -> None:
        """Store a structured result (e.g. move accuracy breakdown)."""
        self._records[key] = data

    def _summarise(self, values: list[float]) -> dict:
        """Compute descriptive stats for a list of samples."""
        if not values:
            return {"n": 0}
        return {
            "n": len(values),
            "mean": round(statistics.mean(values), 4),
            "median": round(statistics.median(values), 4),
            "stdev": round(statistics.stdev(values), 4) if len(values) > 1 else 0.0,
            "min": round(min(values), 4),
            "max": round(max(values), 4),
        }

    def write_report(self, path: Path | None = None) -> dict:
        """
        Generate the calibration report and write to disk.

        Returns the report dict even if file write fails.
        """
        path = path or _REPORT_PATH

        report = self._build_report()

        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w") as f:
                json.dump(report, f, indent=2)
            print(f"\n{'='*70}")
            print(f"  Calibration report written to: {path}")
            print(f"{'='*70}\n")
        except Exception as e:
            print(f"\n  WARNING: Could not write report: {e}")

        return report

    def _build_report(self) -> dict:
        """Assemble the full report structure."""
        report: dict = {
            "_description": (
                "Simulator calibration report — measured metrics from sim and "
                "real hardware, with suggested parameter adjustments."
            ),
            "_generated": datetime.now(timezone.utc).isoformat(),
            "_project_root": str(_PROJECT_ROOT),
            "hardware_detected": {
                "real_xy": self.real_xy_available,
                "real_zp": self.real_zp_available,
            },
            "metrics": {},
            "parameter_mapping": self._build_parameter_mapping(),
            "recommendations": [],
        }

        # Summarise all sample buckets
        for key, values in sorted(self._samples.items()):
            report["metrics"][key] = self._summarise(values)

        # Attach structured records
        for key, data in sorted(self._records.items()):
            report["metrics"][key] = data

        # Generate tuning recommendations
        report["recommendations"] = self._generate_recommendations(report)

        return report

    def _build_parameter_mapping(self) -> dict:
        """
        Maps each metric name to the simulator parameter(s) it informs.

        This is the key reference: when a metric shows a discrepancy
        between sim and real, this tells you exactly what to change.
        """
        return {
            "_description": (
                "Maps each metric to the simulator parameter it informs. "
                "Use this to know what to tune when sim/real diverge."
            ),
            "xy_sim_position_query_latency_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": [
                    "PROCESSING_TIMES['position']  (loaded from config/xy_diagnostic_profile.json → simulator_derived_constants.processing_time_position_s)",
                    "DEFAULT_BAUD / _PROFILE_BAUD  (baud rate → bytes_per_second → TX/RX delay)",
                ],
                "config_file": "config/xy_diagnostic_profile.json",
                "config_keys": [
                    "simulator_derived_constants.processing_time_position_s",
                    "communication.baud_rate",
                ],
            },
            "xy_real_position_query_latency_ms": {
                "file": "N/A — measured from real hardware",
                "purpose": "Reference target for xy_sim_position_query_latency_ms",
            },
            "xy_sim_send_command_latency_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": [
                    "PROCESSING_TIMES (all command types)",
                    "TX/RX baud delay",
                ],
                "config_file": "config/xy_diagnostic_profile.json",
                "config_keys": [
                    "simulator_derived_constants.*",
                ],
            },
            "xy_real_send_command_latency_ms": {
                "purpose": "Reference target for sim command timing",
            },
            "xy_sim_relative_move_accuracy_um": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": [
                    "DEFAULT_KP (proportional gain — higher = faster settle, potential overshoot)",
                    "SETTLE_THRESHOLD_UM (when physics loop declares 'arrived')",
                    "MAX_ACCEL_UM_S2 (acceleration limit)",
                    "PHYSICS_HZ (update rate — higher = smoother but more CPU)",
                ],
                "module_constants": {
                    "DEFAULT_KP": 15.0,
                    "SETTLE_THRESHOLD_UM": 0.5,
                    "MAX_ACCEL_UM_S2": 50000.0,
                    "PHYSICS_HZ": 200,
                },
            },
            "xy_real_relative_move_accuracy_um": {
                "purpose": "Reference target for sim move accuracy",
            },
            "xy_sim_absolute_move_settle_time_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": [
                    "MAX_SPEED_UM_S (top speed affects travel time)",
                    "MAX_ACCEL_UM_S2 (ramp-up/down time)",
                    "DEFAULT_KP (settling behavior)",
                    "SETTLE_THRESHOLD_UM (convergence criterion)",
                ],
                "module_constants": {
                    "MAX_SPEED_UM_S": 20000.0,
                    "MAX_ACCEL_UM_S2": 50000.0,
                },
            },
            "xy_real_absolute_move_settle_time_ms": {
                "purpose": "Reference target for sim settle time",
            },
            "xy_sim_velocity_mode_drift_after_stop_um": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": [
                    "MAX_ACCEL_UM_S2 (deceleration ramp — higher = stops faster)",
                    "DEFAULT_KP (how aggressively it brakes)",
                ],
            },
            "xy_real_velocity_mode_drift_after_stop_um": {
                "purpose": "Reference target for sim stop behavior",
            },
            "xy_sim_query_rate_hz": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": [
                    "Inverse of total round-trip time (TX delay + processing + RX delay)",
                ],
                "config_file": "config/xy_diagnostic_profile.json",
                "config_keys": ["sustained_rate_hz.position"],
            },
            "xy_real_query_rate_hz": {
                "purpose": "Reference target for sim query rate",
                "expected": "~83 Hz burst, ~62.5 Hz sustained (from diagnostic profile)",
            },
            "zp_sim_position_query_latency_ms": {
                "file": "SupportClasses/ZPStageSimulator.py",
                "parameters": [
                    "communication_delay (default 0.03 s = 30 ms)",
                    "processing_time_per_command (default 0.01 s = 10 ms)",
                ],
                "constructor_defaults": {
                    "communication_delay": 0.03,
                    "processing_time_per_command": 0.01,
                },
            },
            "zp_real_position_query_latency_ms": {
                "purpose": "Reference target for zp_sim timing",
            },
            "zp_sim_relative_move_accuracy_mm": {
                "file": "SupportClasses/ZPStageSimulator.py",
                "parameters": [
                    "kp (proportional gain, default 2.0)",
                    "acceleration_rate (default 100.0)",
                    "max_speed (default 100.0)",
                    "Physics loop rate (~100 Hz hardcoded sleep(0.01))",
                ],
                "constructor_defaults": {
                    "kp": 2.0,
                    "acceleration_rate": 100.0,
                    "max_speed": 100.0,
                },
            },
            "zp_real_relative_move_accuracy_mm": {
                "purpose": "Reference target for sim move accuracy",
            },
            "zp_sim_gcode_round_trip_ms": {
                "file": "SupportClasses/ZPStageSimulator.py",
                "parameters": [
                    "communication_delay + processing_time_per_command",
                ],
            },
            "zp_real_gcode_round_trip_ms": {
                "purpose": "Reference target for sim G-code timing",
            },
            "zp_sim_query_rate_hz": {
                "file": "SupportClasses/ZPStageSimulator.py",
                "parameters": [
                    "Inverse of (communication_delay + processing_time + read delay)",
                ],
            },
            "zp_real_query_rate_hz": {
                "purpose": "Reference target for sim query rate",
            },
            # ── Diagnostic-level metrics (Section 10/11) ──────────
            "xy_sim_position_cmd_latency_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": ["PROCESSING_TIMES['position']"],
            },
            "xy_sim_velocity_cmd_latency_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": ["PROCESSING_TIMES['velocity']"],
            },
            "xy_sim_setting_cmd_latency_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": ["PROCESSING_TIMES['setting']"],
            },
            "xy_sim_gr_cmd_latency_ms": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": ["PROCESSING_TIMES['move']"],
            },
            "xy_sim_sustained_5s_rate_hz": {
                "file": "config/xy_diagnostic_profile.json",
                "parameters": ["sustained_rate_hz.position"],
            },
            "xy_sim_sine_tracking": {
                "file": "SupportClasses/XYStageSimulator.py",
                "parameters": ["MAX_SPEED_UM_S", "MAX_ACCEL_UM_S2", "DEFAULT_KP"],
            },
            "zp_sim_m114_latency_ms": {
                "file": "SupportClasses/ZPStageSimulator.py",
                "parameters": ["communication_delay", "processing_time_per_command"],
            },
            "zp_sim_sustained_5s_rate_hz": {
                "file": "SupportClasses/ZPStageSimulator.py",
                "parameters": ["Inverse of total per-query overhead"],
            },
        }

    def _generate_recommendations(self, report: dict) -> list[dict]:
        """
        Compare sim vs real metrics and produce actionable tuning advice.
        """
        recs = []
        metrics = report["metrics"]

        # ── SANITY CHECKS — detect serial communication bugs ──────
        # If real hardware latency is wildly wrong, the tuning
        # recommendations would be nonsensical. Detect and warn.
        _LATENCY_SANITY_THRESHOLDS = {
            # metric_key: (max_reasonable_ms, probable_cause)
            "xy_real_position_query_latency_ms": (
                50.0,
                "XY get_current_position() is likely using readline() which "
                "blocks on the full serial timeout. ProScan II terminates "
                "with CR (\\r), not LF (\\n). Fix: replace readline() with "
                "read_until(b'\\r') + short timeout in XYStage.py. "
                "See patch_v725_hw_comms.py."
            ),
            "zp_real_position_query_latency_ms": (
                200.0,
                "ZP get_current_position() may have a serial read timeout "
                "issue. Expected ~10-50ms for M114 round-trip."
            ),
        }

        serial_bugs_found = False
        for metric_key, (threshold, explanation) in _LATENCY_SANITY_THRESHOLDS.items():
            if metric_key in metrics:
                val = metrics[metric_key].get("mean", 0)
                if val > threshold:
                    serial_bugs_found = True
                    recs.append({
                        "severity": "CRITICAL",
                        "parameter": "Serial communication bug detected",
                        "metric": metric_key,
                        "measured_ms": round(val, 1),
                        "expected_max_ms": threshold,
                        "action": (
                            f"⚠ {metric_key} = {val:.0f}ms — this is "
                            f"{val/threshold:.0f}× slower than expected. "
                            f"This is almost certainly a SOFTWARE BUG, not "
                            f"real hardware behavior. {explanation} "
                            f"ALL tuning recommendations below are INVALID "
                            f"until this is fixed."
                        ),
                    })

        if serial_bugs_found:
            recs.append({
                "severity": "INFO",
                "action": (
                    "Fix the serial communication bug(s) above, then re-run "
                    "the test suite. Tuning recommendations below are based "
                    "on buggy measurements and should be IGNORED."
                ),
            })

        # ── XY Position Query Latency ─────────────────────────────
        sim_key = "xy_sim_position_query_latency_ms"
        real_key = "xy_real_position_query_latency_ms"
        if sim_key in metrics and real_key in metrics:
            sim_val = metrics[sim_key].get("mean", 0)
            real_val = metrics[real_key].get("mean", 0)
            if real_val > 0:
                ratio = sim_val / real_val
                if abs(ratio - 1.0) > 0.15:
                    # Derive what processing time should be
                    # total ≈ TX_delay + processing + RX_delay
                    # At 38400 baud: TX "P\r" ≈ 0.52ms, RX "x,y,z\r" ≈ 3.6ms
                    # processing = total - TX - RX
                    est_processing_ms = max(0, real_val - 0.52 - 3.6)
                    recs.append({
                        "parameter": "config/xy_diagnostic_profile.json → simulator_derived_constants.processing_time_position_s",
                        "current_sim_latency_ms": round(sim_val, 2),
                        "measured_real_latency_ms": round(real_val, 2),
                        "ratio_sim_to_real": round(ratio, 3),
                        "suggested_processing_time_s": round(est_processing_ms / 1000, 4),
                        "action": (
                            f"Sim is {'slower' if ratio > 1 else 'faster'} than real by "
                            f"{abs(ratio-1)*100:.0f}%. Update processing_time_position_s "
                            f"to {est_processing_ms/1000:.4f} s."
                        ),
                    })

        # ── XY Query Rate ─────────────────────────────────────────
        sim_key = "xy_sim_query_rate_hz"
        real_key = "xy_real_query_rate_hz"
        if sim_key in metrics and real_key in metrics:
            sim_val = metrics[sim_key].get("mean", 0)
            real_val = metrics[real_key].get("mean", 0)
            if real_val > 0 and abs(sim_val/real_val - 1.0) > 0.15:
                recs.append({
                    "parameter": "XY simulator processing times (all command types)",
                    "measured_sim_rate_hz": round(sim_val, 1),
                    "measured_real_rate_hz": round(real_val, 1),
                    "action": (
                        f"Sim rate is {sim_val:.1f} Hz vs real {real_val:.1f} Hz. "
                        f"Adjust processing times proportionally: multiply all by "
                        f"{real_val/sim_val:.3f}."
                    ),
                })

        # ── XY Move Accuracy ─────────────────────────────────────
        sim_key = "xy_sim_relative_move_accuracy_um"
        real_key = "xy_real_relative_move_accuracy_um"
        if sim_key in metrics and real_key in metrics:
            sim_err = metrics[sim_key].get("mean", 0)
            real_err = metrics[real_key].get("mean", 0)
            # Large error (>1000µm) on real hardware indicates a unit
            # mismatch, not a sim tuning issue.
            if real_err > 1000:
                recs.append({
                    "severity": "CRITICAL",
                    "parameter": "XY position unit mismatch",
                    "sim_mean_error_um": round(sim_err, 2),
                    "real_mean_error_um": round(real_err, 2),
                    "action": (
                        f"⚠ XY real accuracy error = {real_err:.0f}µm — "
                        f"this indicates a UNIT MISMATCH between the test "
                        f"and the controller. The test sends 50 'units' but "
                        f"the controller may interpret them as microsteps "
                        f"(not µm). Check proscan_ii.json position_units "
                        f"and microsteps_per_micron. If the stage has 25 "
                        f"µsteps/µm, GR 50,0 = 2µm not 50µm."
                    ),
                })
            elif abs(sim_err) > 5.0 or abs(real_err) > 5.0:
                recs.append({
                    "parameter": "XYStageSimulator DEFAULT_KP, SETTLE_THRESHOLD_UM",
                    "sim_mean_error_um": round(sim_err, 2),
                    "real_mean_error_um": round(real_err, 2),
                    "action": (
                        f"Sim error={sim_err:.1f}µm, real error={real_err:.1f}µm. "
                        f"{'Increase DEFAULT_KP' if abs(sim_err) > abs(real_err) else 'Decrease DEFAULT_KP'} "
                        f"to better match real settling behavior."
                    ),
                })

        # ── XY Settle Time ────────────────────────────────────────
        sim_key = "xy_sim_absolute_move_settle_time_ms"
        real_key = "xy_real_absolute_move_settle_time_ms"
        if sim_key in metrics and real_key in metrics:
            sim_val = metrics[sim_key].get("mean", 0)
            real_val = metrics[real_key].get("mean", 0)
            if real_val > 0 and abs(sim_val/real_val - 1.0) > 0.2:
                recs.append({
                    "parameter": "XYStageSimulator MAX_SPEED_UM_S, MAX_ACCEL_UM_S2",
                    "sim_settle_ms": round(sim_val, 1),
                    "real_settle_ms": round(real_val, 1),
                    "action": (
                        f"Sim settles in {sim_val:.0f}ms vs real {real_val:.0f}ms. "
                        f"{'Reduce MAX_SPEED_UM_S/MAX_ACCEL_UM_S2' if sim_val < real_val else 'Increase MAX_SPEED_UM_S/MAX_ACCEL_UM_S2'} "
                        f"to match."
                    ),
                })

        # ── XY Stop Drift ─────────────────────────────────────────
        sim_key = "xy_sim_velocity_mode_drift_after_stop_um"
        real_key = "xy_real_velocity_mode_drift_after_stop_um"
        if sim_key in metrics and real_key in metrics:
            sim_val = metrics[sim_key].get("mean", 0)
            real_val = metrics[real_key].get("mean", 0)
            if abs(sim_val - real_val) > 10:
                recs.append({
                    "parameter": "XYStageSimulator MAX_ACCEL_UM_S2 (deceleration)",
                    "sim_drift_um": round(sim_val, 1),
                    "real_drift_um": round(real_val, 1),
                    "action": (
                        f"Sim drifts {sim_val:.0f}µm after stop vs real {real_val:.0f}µm. "
                        f"{'Increase' if sim_val > real_val else 'Decrease'} MAX_ACCEL_UM_S2."
                    ),
                })

        # ── ZP Position Query Latency ─────────────────────────────
        sim_key = "zp_sim_position_query_latency_ms"
        real_key = "zp_real_position_query_latency_ms"
        if sim_key in metrics and real_key in metrics:
            sim_val = metrics[sim_key].get("mean", 0)
            real_val = metrics[real_key].get("mean", 0)
            if real_val > 0 and abs(sim_val/real_val - 1.0) > 0.2:
                target_comm = max(0, real_val * 0.6) / 1000  # ~60% is comm delay
                target_proc = max(0, real_val * 0.4) / 1000  # ~40% is processing
                recs.append({
                    "parameter": "ZPStageSimulator(communication_delay, processing_time_per_command)",
                    "current_sim_latency_ms": round(sim_val, 1),
                    "measured_real_latency_ms": round(real_val, 1),
                    "suggested_communication_delay_s": round(target_comm, 4),
                    "suggested_processing_time_s": round(target_proc, 4),
                    "action": (
                        f"ZP sim latency={sim_val:.0f}ms vs real={real_val:.0f}ms. "
                        f"Set communication_delay={target_comm:.4f}s, "
                        f"processing_time_per_command={target_proc:.4f}s."
                    ),
                })

        # ── ZP Move Accuracy ─────────────────────────────────────
        sim_key = "zp_sim_relative_move_accuracy_mm"
        real_key = "zp_real_relative_move_accuracy_mm"
        if sim_key in metrics and real_key in metrics:
            sim_err = metrics[sim_key].get("mean", 0)
            real_err = metrics[real_key].get("mean", 0)
            if abs(sim_err - real_err) > 0.05:
                recs.append({
                    "parameter": "ZPStageSimulator kp, acceleration_rate",
                    "sim_mean_error_mm": round(sim_err, 4),
                    "real_mean_error_mm": round(real_err, 4),
                    "action": (
                        f"ZP sim error={sim_err:.3f}mm vs real={real_err:.3f}mm. "
                        f"Adjust kp and acceleration_rate."
                    ),
                })

        # ── Per-Command-Type Tuning (from diagnostic tests) ───────
        # Compare sim vs real for specific command types to tune
        # individual PROCESSING_TIMES entries
        cmd_type_pairs = [
            ("position", "PROCESSING_TIMES['position']"),
            ("velocity", "PROCESSING_TIMES['velocity']"),
            ("setting", "PROCESSING_TIMES['setting']"),
        ]
        for cmd_type, param_name in cmd_type_pairs:
            sim_key = f"xy_sim_{cmd_type}_cmd_latency_ms"
            real_key = f"xy_real_{cmd_type}_cmd_latency_ms"
            if sim_key in metrics and real_key in metrics:
                sim_val = metrics[sim_key].get("mean", 0)
                real_val = metrics[real_key].get("mean", 0)
                if real_val > 0 and abs(sim_val / real_val - 1.0) > 0.2:
                    recs.append({
                        "parameter": param_name,
                        "sim_ms": round(sim_val, 2),
                        "real_ms": round(real_val, 2),
                        "action": (
                            f"XY {cmd_type} cmd: sim={sim_val:.1f}ms vs "
                            f"real={real_val:.1f}ms. Ratio={sim_val/real_val:.2f}."
                        ),
                    })

        # ── Sustained Rate Comparison ─────────────────────────────
        for prefix in ["xy", "zp"]:
            sim_key = f"{prefix}_sim_sustained_5s_rate_hz"
            real_key = f"{prefix}_real_sustained_5s_rate_hz"
            if sim_key in metrics and real_key in metrics:
                sim_val = metrics[sim_key].get("mean", 0)
                real_val = metrics[real_key].get("mean", 0)
                if real_val > 0 and abs(sim_val / real_val - 1.0) > 0.2:
                    recs.append({
                        "parameter": f"{prefix.upper()} sustained polling rate",
                        "sim_hz": round(sim_val, 1),
                        "real_hz": round(real_val, 1),
                        "action": (
                            f"{prefix.upper()} sustained 5s: sim={sim_val:.1f}Hz "
                            f"vs real={real_val:.1f}Hz."
                        ),
                    })

        # ── If no real hardware, note sim-only baseline ───────────
        if not recs:
            recs.append({
                "parameter": "N/A",
                "action": (
                    "No real hardware was connected — report contains sim-only "
                    "baselines. Re-run with MEBP_TEST_REAL_XY=1 and/or "
                    "MEBP_TEST_REAL_ZP=1 to generate tuning recommendations."
                ),
            })

        return recs


# ── Global metrics instance ───────────────────────────────────────
METRICS = MetricsCollector()


# ═══════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════

def _try_connect_real_xy() -> Optional[XYStageManager]:
    """Attempt to connect a real XY stage. Returns None if unavailable."""
    if SKIP_REAL:
        return None
    try:
        mgr = XYStageManager(simulate=False, controller_json="auto")
        if mgr.spo is not None:
            return mgr
        mgr.stop()
    except Exception as e:
        logger.debug(f"Real XY not available: {e}")
    return None


def _try_connect_real_zp() -> Optional[ZPStageManager]:
    """Attempt to connect a real ZP stage. Returns None if unavailable."""
    if SKIP_REAL:
        return None
    try:
        mgr = ZPStageManager(simulate=False)
        if mgr.serial is not None and (hasattr(mgr.serial, 'is_open') and mgr.serial.is_open):
            return mgr
        mgr.stop()
    except Exception as e:
        logger.debug(f"Real ZP not available: {e}")
    return None


def _wait_for_position_stable(
    get_pos_fn, timeout: float = 5.0, tolerance: float = 0.5, axis_count: int = 3
) -> tuple:
    """Poll position until it stops changing, returns final position."""
    last_pos = get_pos_fn()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.05)
        pos = get_pos_fn()
        if pos[0] is None:
            continue
        # Build diffs only for axes where BOTH readings are valid
        diffs = []
        for i in range(min(axis_count, len(pos))):
            if last_pos[i] is not None and pos[i] is not None:
                diffs.append(abs(pos[i] - last_pos[i]))
        # Need at least one valid diff to declare stable
        if diffs and all(d < tolerance for d in diffs):
            return pos
        last_pos = pos
    return get_pos_fn()


def _zp_read_position(stage: ZPStageManager) -> tuple:
    """Read ZP position reliably, draining stale serial buffer data first.

    The ZP simulator's buffered serial path has a ~40ms round-trip.
    receive_data() only sleeps 10ms before reading, so calling
    get_current_position() can return STALE data from a previous query.

    Fix: drain the response buffer WITHOUT sending new commands, then
    do a single clean M114 query with enough processing time.
    """
    # Drain any stale responses (don't send new commands)
    stage.receive_data()
    time.sleep(0.05)
    stage.receive_data()
    time.sleep(0.05)
    # Fresh M114 query with adequate processing time
    stage.send_data("M114")
    time.sleep(0.15)  # 150ms: 30ms comm + 10ms proc + 110ms margin
    resp = stage.receive_data()
    stage._parse_position(resp)
    return (stage.x_pos, stage.y_pos, stage.z_pos, stage.e_pos)


def _xy_read_position(stage: XYStageManager) -> tuple:
    """Read XY position reliably on real hardware.

    On real ProScan, every fire-and-forget command (Z, GR, VS, I, SMS...)
    leaves an unread response (R, 0, etc.) in the serial buffer. When
    get_current_position() sends P and reads, it gets a STALE response
    from a previous command — not the P response.

    Fix: flush the serial input buffer to discard all pending responses,
    then do a fresh position query. Works for both sim and real.
    """
    if stage.simulate:
        return stage.get_current_position()

    # Flush all pending responses from the serial buffer
    if hasattr(stage.spo, 'reset_input_buffer'):
        stage.spo.reset_input_buffer()
    time.sleep(0.05)  # let any in-flight bytes arrive
    if hasattr(stage.spo, 'reset_input_buffer'):
        stage.spo.reset_input_buffer()  # clear those too
    time.sleep(0.02)

    # Fresh position query — buffer is clean, so P response is the first thing we read
    return stage.get_current_position()


# ── ZP sim timing constants ───────────────────────────────────────
# The ZP simulator is slow by design:
#   - Each command: 30ms comm_delay + 10ms processing = ~40ms
#   - move_absolute: sends 3 G-codes (G90+G0+G91) = ~120ms command overhead
#   - Physics: P-controller with kp=2.0, max_speed=100 → ~500ms per mm
# These constants ensure tests wait long enough for convergence.
ZP_SIM_CMD_OVERHEAD_S = 0.15    # Single command overhead (40ms + margin)
ZP_SIM_MOVE_ABS_OVERHEAD_S = 0.5  # move_absolute sends 3 commands
ZP_SIM_MM_PER_S = 2.0          # Approximate convergence speed (kp * typical_error / 2)


def _zp_sim_wait(distance_mm: float, n_commands: int = 1) -> float:
    """Calculate how long to wait for a ZP sim move to converge.

    Returns seconds to sleep.
    """
    cmd_time = n_commands * ZP_SIM_CMD_OVERHEAD_S
    physics_time = abs(distance_mm) / ZP_SIM_MM_PER_S
    return cmd_time + physics_time + 0.5  # + safety margin


def _measure_latency_batch(fn, n: int = 20, inter_delay: float = 0.0) -> list[float]:
    """Call fn() n times and return list of elapsed times in ms.

    Args:
        fn:          Callable to measure.
        n:           Number of iterations.
        inter_delay: Seconds to sleep between calls (use MARLIN_CMD_INTERVAL_S
                     for real ZP hardware to prevent buffer overflow).
    """
    times = []
    for _ in range(n):
        t0 = time.monotonic()
        fn()
        times.append((time.monotonic() - t0) * 1000)
        if inter_delay > 0:
            time.sleep(inter_delay)
    return times


def _measure_settle_time(stage_move_fn, stage_pos_fn, target_check_fn,
                         timeout: float = 10.0) -> float:
    """Time from issuing a move to position being within tolerance. Returns ms."""
    t0 = time.monotonic()
    stage_move_fn()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.005)
        if target_check_fn(stage_pos_fn()):
            return (time.monotonic() - t0) * 1000
    return timeout * 1000


# ═══════════════════════════════════════════════════════════════════
#  1. XY STAGE — SIMULATED-ONLY TESTS
# ═══════════════════════════════════════════════════════════════════

class TestXYStageSimulated(unittest.TestCase):
    """Tests that the XY simulator behaves correctly on its own."""

    @classmethod
    def setUpClass(cls):
        cls.stage = XYStageManager(simulate=True)

    @classmethod
    def tearDownClass(cls):
        cls.stage.stop()

    # ── 1.1 Initialisation ────────────────────────────────────────

    def test_01_simulate_flag(self):
        """XY sim mode: simulate attribute is True."""
        self.assertTrue(self.stage.simulate)

    def test_02_spo_is_simulator(self):
        """XY sim mode: spo is an XYStageSimulator instance."""
        self.assertIsInstance(self.stage.spo, XYStageSimulator)

    def test_03_spo_is_running(self):
        """XY sim mode: simulator threads are running."""
        self.assertTrue(self.stage.spo._running)

    # ── 1.2 Position Query ────────────────────────────────────────

    def test_10_position_returns_tuple(self):
        """get_current_position returns a 3-tuple."""
        pos = self.stage.get_current_position()
        self.assertIsInstance(pos, tuple)
        self.assertEqual(len(pos), 3)

    def test_11_position_values_are_numeric(self):
        """Position values are float or int, not None."""
        pos = self.stage.get_current_position()
        for v in pos:
            self.assertIsNotNone(v)
            self.assertIsInstance(v, (int, float))

    def test_12_initial_position_near_zero(self):
        """Initial position should be near (0, 0, 0)."""
        pos = self.stage.get_current_position()
        for v in pos:
            self.assertAlmostEqual(v, 0.0, delta=1.0,
                msg=f"Initial position {pos} not near zero")

    # ── 1.3 send_command ──────────────────────────────────────────

    def test_20_send_command_returns_string(self):
        """In sim mode, send_command returns a string response."""
        resp = self.stage.send_command("P")
        self.assertIsInstance(resp, str)

    def test_21_position_query_format(self):
        """P command returns comma-separated numbers."""
        resp = self.stage.send_command("P")
        parts = resp.split(",")
        self.assertGreaterEqual(len(parts), 2,
            msg=f"P response '{resp}' doesn't have enough comma-separated values")

    def test_22_firmware_version_command(self):
        """V command returns a non-empty string."""
        resp = self.stage.send_command("V")
        self.assertIsInstance(resp, str)
        self.assertTrue(len(resp) > 0, "V command returned empty string")

    def test_23_stop_command(self):
        """I (stop) command returns response without error."""
        resp = self.stage.send_command("I")
        self.assertIsInstance(resp, str)

    def test_24_set_home_command(self):
        """Z (set home) command returns response without error."""
        resp = self.stage.send_command("Z")
        self.assertIsInstance(resp, str)

    # ── 1.4 Movement — Relative (+ metrics) ──────────────────────

    def test_30_relative_move_changes_position(self):
        """move_stage_relative changes position by approximately the requested amount."""
        self.stage.set_home()
        time.sleep(0.1)
        pos_before = self.stage.get_current_position()

        dx, dy = 100.0, 200.0
        self.stage.move_stage_relative(dx, dy)
        time.sleep(0.5)

        pos_after = _wait_for_position_stable(
            self.stage.get_current_position, timeout=3.0, tolerance=1.0
        )

        actual_dx = pos_after[0] - pos_before[0]
        actual_dy = pos_after[1] - pos_before[1]
        error_x = abs(actual_dx - dx)
        error_y = abs(actual_dy - dy)

        # Record metrics
        METRICS.add_sample("xy_sim_relative_move_accuracy_um", error_x)
        METRICS.add_sample("xy_sim_relative_move_accuracy_um", error_y)

        self.assertAlmostEqual(actual_dx, dx, delta=5.0,
            msg=f"X moved {actual_dx}, expected ~{dx}")
        self.assertAlmostEqual(actual_dy, dy, delta=5.0,
            msg=f"Y moved {actual_dy}, expected ~{dy}")

    def test_31_multiple_relative_moves_accumulate(self):
        """Multiple relative moves accumulate correctly."""
        self.stage.set_home()
        time.sleep(0.1)

        total_dx, total_dy = 0.0, 0.0
        for _ in range(5):
            self.stage.move_stage_relative(50.0, 30.0)
            total_dx += 50.0
            total_dy += 30.0
            time.sleep(0.3)

        pos = _wait_for_position_stable(
            self.stage.get_current_position, timeout=5.0, tolerance=2.0
        )
        accum_err = math.hypot(pos[0] - total_dx, pos[1] - total_dy)
        METRICS.add_sample("xy_sim_accumulated_move_error_um", accum_err)

        self.assertAlmostEqual(pos[0], total_dx, delta=10.0)
        self.assertAlmostEqual(pos[1], total_dy, delta=10.0)

    # ── 1.5 Movement — Absolute (+ settle time metric) ───────────

    def test_40_absolute_move(self):
        """move_stage_to_position moves to the target coordinates."""
        target_x, target_y = 500.0, 300.0
        t0 = time.monotonic()
        self.stage.move_stage_to_position(target_x, target_y)

        pos = _wait_for_position_stable(
            self.stage.get_current_position, timeout=5.0, tolerance=2.0
        )
        settle_ms = (time.monotonic() - t0) * 1000
        METRICS.add_sample("xy_sim_absolute_move_settle_time_ms", settle_ms)

        self.assertAlmostEqual(pos[0], target_x, delta=5.0)
        self.assertAlmostEqual(pos[1], target_y, delta=5.0)

    def test_41_absolute_move_to_negative(self):
        """Absolute move to negative coordinates works."""
        target_x, target_y = -200.0, -150.0
        self.stage.move_stage_to_position(target_x, target_y)

        pos = _wait_for_position_stable(
            self.stage.get_current_position, timeout=5.0, tolerance=2.0
        )
        self.assertAlmostEqual(pos[0], target_x, delta=5.0)
        self.assertAlmostEqual(pos[1], target_y, delta=5.0)

    # ── 1.6 Set Home ──────────────────────────────────────────────

    def test_50_set_home_resets_position(self):
        """set_home resets current position to (0, 0, 0)."""
        self.stage.move_stage_to_position(100.0, 100.0)
        time.sleep(0.5)
        _wait_for_position_stable(self.stage.get_current_position, timeout=3.0)

        self.stage.set_home()
        time.sleep(0.1)

        pos = self.stage.get_current_position()
        self.assertAlmostEqual(pos[0], 0.0, delta=2.0)
        self.assertAlmostEqual(pos[1], 0.0, delta=2.0)

    # ── 1.7 Stop (+ drift metric) ────────────────────────────────

    def test_60_stop_halts_velocity_mode(self):
        """stop_stage halts ongoing velocity movement."""
        self.stage.move_stage_at_velocity(500.0, 0.0)
        time.sleep(0.2)
        self.stage.stop_stage()
        time.sleep(0.2)

        pos1 = self.stage.get_current_position()
        time.sleep(0.3)
        pos2 = self.stage.get_current_position()

        drift = abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])
        METRICS.add_sample("xy_sim_velocity_mode_drift_after_stop_um", drift)

        self.assertLess(drift, 5.0,
            msg=f"Stage still moving after stop: drift={drift}")

    # ── 1.8 Velocity Mode ────────────────────────────────────────

    def test_70_velocity_mode_causes_movement(self):
        """VS command causes continuous movement."""
        self.stage.set_home()
        time.sleep(0.1)

        self.stage.move_stage_at_velocity(1000.0, 0.0)
        time.sleep(0.5)
        pos = self.stage.get_current_position()
        self.stage.stop_stage()

        self.assertGreater(pos[0], 10.0,
            msg=f"Velocity mode didn't cause movement: X={pos[0]}")

    # ── 1.9 Settings ─────────────────────────────────────────────

    def test_80_set_velocity_no_error(self):
        """set_velocity doesn't raise."""
        try:
            self.stage.set_velocity(50)
        except Exception as e:
            self.fail(f"set_velocity raised: {e}")

    def test_81_set_acceleration_no_error(self):
        """set_acceleration doesn't raise."""
        try:
            self.stage.set_acceleration(50)
        except Exception as e:
            self.fail(f"set_acceleration raised: {e}")


# ═══════════════════════════════════════════════════════════════════
#  2. ZP STAGE — SIMULATED-ONLY TESTS
# ═══════════════════════════════════════════════════════════════════

class TestZPStageSimulated(unittest.TestCase):
    """Tests that the ZP simulator behaves correctly on its own."""

    @classmethod
    def setUpClass(cls):
        cls.stage = ZPStageManager(simulate=True)

    @classmethod
    def tearDownClass(cls):
        cls.stage.stop()

    def test_01_simulate_flag(self):
        self.assertTrue(self.stage.simulate)

    def test_02_serial_is_simulator(self):
        self.assertIsInstance(self.stage.serial, ZPStageSimulator)

    def test_03_simulator_is_running(self):
        self.assertTrue(self.stage.serial.is_running)

    def test_10_position_returns_tuple(self):
        pos = self.stage.get_current_position()
        self.assertIsInstance(pos, tuple)
        self.assertEqual(len(pos), 4)

    def test_11_position_values_are_numeric(self):
        pos = self.stage.get_current_position()
        for i, v in enumerate(pos):
            self.assertIsInstance(v, (int, float))

    def test_12_initial_position_near_zero(self):
        pos = self.stage.get_current_position()
        for i, v in enumerate(pos):
            self.assertAlmostEqual(v, 0.0, delta=1.0)

    def test_20_send_data_no_error(self):
        try:
            self.stage.send_data("M114")
        except Exception as e:
            self.fail(f"send_data raised: {e}")

    def test_21_receive_data_returns_string(self):
        self.stage.send_data("M115")
        time.sleep(0.1)
        resp = self.stage.receive_data()
        self.assertIsInstance(resp, str)

    def test_22_m115_contains_firmware(self):
        self.stage.send_data("M115")
        time.sleep(0.1)
        resp = self.stage.receive_data()
        self.assertIn("FIRMWARE_NAME", resp)

    # ── 2.4 Movement — Relative (+ metrics) ──────────────────────

    def test_30_relative_move_single_axis(self):
        """Relative move on one axis changes only that axis."""
        pos_before = _zp_read_position(self.stage)
        self.stage.move_relative({"X": 1.0})
        time.sleep(_zp_sim_wait(1.0))
        pos_after = _zp_read_position(self.stage)

        delta_x = pos_after[0] - pos_before[0]
        error = abs(delta_x - 1.0)
        METRICS.add_sample("zp_sim_relative_move_accuracy_mm", error)

        delta_y = abs(pos_after[1] - pos_before[1])
        self.assertAlmostEqual(delta_x, 1.0, delta=0.2,
            msg=f"X moved {delta_x}, expected ~1.0")
        self.assertLess(delta_y, 0.2)

    def test_31_relative_move_multiple_axes(self):
        pos_before = _zp_read_position(self.stage)
        self.stage.move_relative({"X": 0.5, "Y": -0.3, "E": 0.2})
        time.sleep(_zp_sim_wait(0.5))
        pos_after = _zp_read_position(self.stage)

        dx = pos_after[0] - pos_before[0]
        dy = pos_after[1] - pos_before[1]
        de = pos_after[3] - pos_before[3]

        METRICS.add_sample("zp_sim_relative_move_accuracy_mm", abs(dx - 0.5))
        METRICS.add_sample("zp_sim_relative_move_accuracy_mm", abs(dy - (-0.3)))
        METRICS.add_sample("zp_sim_relative_move_accuracy_mm", abs(de - 0.2))

        self.assertAlmostEqual(dx, 0.5, delta=0.2)
        self.assertAlmostEqual(dy, -0.3, delta=0.2)
        self.assertAlmostEqual(de, 0.2, delta=0.2)

    def test_32_zero_move_ignored(self):
        pos_before = _zp_read_position(self.stage)
        self.stage.move_relative({"X": 0.0, "Y": 0.0})
        time.sleep(0.2)
        pos_after = _zp_read_position(self.stage)
        for i in range(4):
            self.assertAlmostEqual(pos_before[i], pos_after[i], delta=0.1)

    def test_33_accumulated_relative_moves(self):
        self.stage.move_absolute({"X": 0, "Y": 0, "Z": 0, "E": 0})
        time.sleep(_zp_sim_wait(0, n_commands=3))  # 3 cmds for move_absolute
        total = 0.0
        for i in range(5):
            self.stage.move_relative({"X": 0.5})
            total += 0.5
            time.sleep(_zp_sim_wait(0.5))
        pos = _zp_read_position(self.stage)
        METRICS.add_sample("zp_sim_accumulated_move_error_mm",
                           abs(pos[0] - total))
        self.assertAlmostEqual(pos[0], total, delta=0.5,
            msg=f"Accumulated X={pos[0]}, expected ~{total}")

    def test_40_absolute_move(self):
        target = {"X": 5.0, "Y": 3.0}
        self.stage.move_absolute(target)
        time.sleep(_zp_sim_wait(5.0, n_commands=3))
        pos = _zp_read_position(self.stage)
        self.assertAlmostEqual(pos[0], 5.0, delta=0.5,
            msg=f"X={pos[0]}, expected 5.0")
        self.assertAlmostEqual(pos[1], 3.0, delta=0.5,
            msg=f"Y={pos[1]}, expected 3.0")

    def test_41_absolute_move_negative(self):
        self.stage.move_absolute({"X": -2.0, "Y": -1.0})
        time.sleep(_zp_sim_wait(7.0, n_commands=3))  # may travel ~7mm from prev test
        pos = _zp_read_position(self.stage)
        self.assertAlmostEqual(pos[0], -2.0, delta=0.5,
            msg=f"X={pos[0]}, expected -2.0")
        self.assertAlmostEqual(pos[1], -1.0, delta=0.5,
            msg=f"Y={pos[1]}, expected -1.0")

    def test_50_mode_switch_no_error(self):
        try:
            self.stage.set_absolute_mode()
            self.stage.set_relative_mode()
        except Exception as e:
            self.fail(f"Mode switch raised: {e}")

    def test_60_emergency_stop_no_error(self):
        try:
            self.stage.emergency_stop()
        except Exception as e:
            self.fail(f"emergency_stop raised: {e}")

    def test_61_emergency_stop_halts_movement(self):
        self.stage.move_relative({"X": 10.0})
        time.sleep(0.05)
        self.stage.emergency_stop()
        time.sleep(0.2)
        pos1 = self.stage.get_current_position()
        time.sleep(0.3)
        pos2 = self.stage.get_current_position()
        drift = sum(abs(pos2[i] - pos1[i]) for i in range(4))
        self.assertLess(drift, 0.5)

    def test_70_set_feedrate_no_error(self):
        try:
            self.stage.set_max_feedrate(300.0)
        except Exception as e:
            self.fail(f"set_max_feedrate raised: {e}")

    def test_80_axis_map_complete(self):
        for logical in ("Z", "P1", "P2", "P3"):
            self.assertIn(logical, AXIS_MAP)

    def test_81_axis_map_reverse_complete(self):
        for printer_ax in ("X", "Y", "Z", "E"):
            self.assertIn(printer_ax, AXIS_MAP_REVERSE)

    def test_82_axis_map_roundtrip(self):
        for logical, printer in AXIS_MAP.items():
            self.assertEqual(AXIS_MAP_REVERSE[printer], logical)


# ═══════════════════════════════════════════════════════════════════
#  3. XY SIM-vs-HARDWARE COMPARISON (+ rich metrics)
# ═══════════════════════════════════════════════════════════════════

class TestXYSimVsHardware(unittest.TestCase):
    """Side-by-side XY simulated vs real hardware."""

    real_xy: Optional[XYStageManager] = None
    sim_xy: Optional[XYStageManager] = None

    @classmethod
    def setUpClass(cls):
        cls.sim_xy = XYStageManager(simulate=True)
        cls.real_xy = _try_connect_real_xy()
        if cls.real_xy is not None:
            METRICS.real_xy_available = True
        elif FORCE_REAL_XY:
            raise unittest.SkipTest("MEBP_TEST_REAL_XY=1 but no real XY hardware found")

    @classmethod
    def tearDownClass(cls):
        if cls.sim_xy:
            cls.sim_xy.stop()
        if cls.real_xy:
            cls.real_xy.stop()

    def _skip_if_no_real(self):
        if self.real_xy is None:
            self.skipTest("No real XY hardware connected")

    # ── 3.1 API Parity ───────────────────────────────────────────

    def test_01_position_return_type_match(self):
        self._skip_if_no_real()
        sim_pos = self.sim_xy.get_current_position()
        real_pos = self.real_xy.get_current_position()
        self.assertEqual(len(sim_pos), len(real_pos))
        self.assertEqual(len(sim_pos), 3)

    def test_02_position_values_are_numeric(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            pos = stage.get_current_position()
            for i, v in enumerate(pos):
                self.assertIsNotNone(v)
                self.assertIsInstance(v, (int, float))

    def test_03_send_command_type_match(self):
        """send_command('P') — sim returns string, real returns None (async)."""
        self._skip_if_no_real()
        sim_resp = self.sim_xy.send_command("P")
        real_resp = self.real_xy.send_command("P")
        self.assertIsInstance(sim_resp, str)
        METRICS.set_record("xy_api_asymmetry", {
            "send_command_return": {
                "sim": "str",
                "real": "None (async write; read via get_current_position)",
            }
        })

    # ── 3.2 Movement behavior ────────────────────────────────────

    def test_10_relative_move_direction(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            is_real = (label == "real")

            # Zero position register
            stage.set_home()
            time.sleep(0.5 if is_real else 0.2)

            # Read position with flush (breaks the serial pipeline)
            pos_before = _xy_read_position(stage)
            self.assertIsNotNone(pos_before[0],
                msg=f"{label}: got None position before move")

            # Move +50, +50
            stage.move_stage_relative(50.0, 50.0)
            time.sleep(1.5 if is_real else 1.0)

            # For real hardware: poll with flush until position DIFFERS from before.
            # This avoids the serial pipeline stale-read problem where
            # _wait_for_position_stable converges on a stale value.
            if is_real:
                pos_after = pos_before  # start with before to enter loop
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    pos_after = _xy_read_position(stage)
                    if pos_after[0] is not None and pos_before[0] is not None:
                        if abs(pos_after[0] - pos_before[0]) > 1.0 or \
                           abs(pos_after[1] - pos_before[1]) > 1.0:
                            break  # position changed — move detected
                    time.sleep(0.1)
            else:
                pos_after = _wait_for_position_stable(
                    stage.get_current_position, timeout=5.0, tolerance=1.0
                )

            self.assertIsNotNone(pos_after[0],
                msg=f"{label}: got None position after move")

            dx = pos_after[0] - pos_before[0]
            dy = pos_after[1] - pos_before[1]
            error = math.hypot(dx - 50, dy - 50)
            metric_key = f"xy_{label}_relative_move_accuracy_um"
            METRICS.add_sample(metric_key, error)
            self.assertGreater(dx, 0,
                msg=f"{label}: X didn't increase (dx={dx}, before={pos_before}, after={pos_after})")
            self.assertGreater(dy, 0,
                msg=f"{label}: Y didn't increase (dy={dy})")

    def test_11_set_home_zeroes_both(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            is_real = (label == "real")
            # Move away from zero first
            stage.move_stage_relative(100.0, 100.0)
            # Wait for move to COMPLETE before set_home.
            # Real ProScan GR blocks until arrival, but the serial response
            # may not be read. Poll position until stable.
            _wait_for_position_stable(
                stage.get_current_position, timeout=5.0, tolerance=1.0
            )
            time.sleep(0.2)

            # Set home
            stage.set_home()
            # Wait for Z command to zero the position register
            time.sleep(0.5 if is_real else 0.2)

            # Poll until position is near zero
            pos = _wait_for_position_stable(
                stage.get_current_position, timeout=3.0, tolerance=2.0
            )
            self.assertAlmostEqual(pos[0], 0.0, delta=10.0,
                msg=f"{label}: X={pos[0]} after set_home, expected ~0")
            self.assertAlmostEqual(pos[1], 0.0, delta=10.0,
                msg=f"{label}: Y={pos[1]} after set_home, expected ~0")

    def test_12_stop_halts_both(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            stage.move_stage_at_velocity(500.0, 0.0)
            time.sleep(0.3)
            stage.stop_stage()
            time.sleep(0.3)
            pos1 = stage.get_current_position()
            time.sleep(0.5)
            pos2 = stage.get_current_position()
            drift = abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])
            METRICS.add_sample(f"xy_{label}_velocity_mode_drift_after_stop_um", drift)
            self.assertLess(drift, 10.0)

    # ── 3.3 Timing comparison ─────────────────────────────────────

    def test_20_position_query_latency(self):
        """Position query latency comparison — primary calibration metric."""
        self._skip_if_no_real()

        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            is_real = (label == "real")
            delay = PROSCAN_CMD_INTERVAL_S if is_real else 0.0
            times_ms = _measure_latency_batch(
                stage.get_current_position, n=30, inter_delay=delay
            )
            METRICS.add_samples(f"xy_{label}_position_query_latency_ms", times_ms)

    def test_21_send_command_latency(self):
        """send_command('P') latency comparison."""
        self._skip_if_no_real()

        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            is_real = (label == "real")
            delay = PROSCAN_CMD_INTERVAL_S if is_real else 0.0
            times_ms = _measure_latency_batch(
                lambda s=stage: s.send_command("P"), n=30, inter_delay=delay
            )
            METRICS.add_samples(f"xy_{label}_send_command_latency_ms", times_ms)

    def test_22_query_rate(self):
        """Sustained query rate (Hz) comparison."""
        self._skip_if_no_real()

        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            is_real = (label == "real")
            n = 50
            t0 = time.monotonic()
            for _ in range(n):
                stage.get_current_position()
                if is_real:
                    time.sleep(PROSCAN_CMD_INTERVAL_S)
            elapsed = time.monotonic() - t0
            rate = n / elapsed if elapsed > 0 else 0
            METRICS.add_sample(f"xy_{label}_query_rate_hz", rate)

    def test_23_absolute_move_settle_time(self):
        """Time to settle after absolute move — compares physics models."""
        self._skip_if_no_real()

        for label, stage in [("sim", self.sim_xy), ("real", self.real_xy)]:
            stage.set_home()
            time.sleep(0.3)

            targets = [(300, 0), (0, 300), (300, 300), (-300, -300)]
            for tx, ty in targets:
                stage.set_home()
                time.sleep(0.2)

                t0 = time.monotonic()
                stage.move_stage_to_position(float(tx), float(ty))
                pos = _wait_for_position_stable(
                    stage.get_current_position, timeout=10.0, tolerance=2.0
                )
                settle_ms = (time.monotonic() - t0) * 1000
                METRICS.add_sample(f"xy_{label}_absolute_move_settle_time_ms", settle_ms)


# ═══════════════════════════════════════════════════════════════════
#  4. ZP SIM-vs-HARDWARE COMPARISON (+ rich metrics)
# ═══════════════════════════════════════════════════════════════════

class TestZPSimVsHardware(unittest.TestCase):
    """Side-by-side ZP simulated vs real hardware."""

    real_zp: Optional[ZPStageManager] = None
    sim_zp: Optional[ZPStageManager] = None

    @classmethod
    def setUpClass(cls):
        cls.sim_zp = ZPStageManager(simulate=True)
        cls.real_zp = _try_connect_real_zp()
        if cls.real_zp is not None:
            METRICS.real_zp_available = True
        elif FORCE_REAL_ZP:
            raise unittest.SkipTest("MEBP_TEST_REAL_ZP=1 but no real ZP hardware found")

    @classmethod
    def tearDownClass(cls):
        if cls.sim_zp:
            cls.sim_zp.stop()
        if cls.real_zp:
            cls.real_zp.stop()

    def _skip_if_no_real(self):
        if self.real_zp is None:
            self.skipTest("No real ZP hardware connected")

    def test_01_position_return_type_match(self):
        self._skip_if_no_real()
        sim_pos = self.sim_zp.get_current_position()
        real_pos = self.real_zp.get_current_position()
        self.assertEqual(len(sim_pos), len(real_pos))
        self.assertEqual(len(sim_pos), 4)

    def test_02_position_values_numeric(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_zp), ("real", self.real_zp)]:
            pos = stage.get_current_position()
            for i, v in enumerate(pos):
                self.assertIsInstance(v, (int, float))

    def test_10_relative_move_direction(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_zp), ("real", self.real_zp)]:
            is_real = (label == "real")
            is_sim = not is_real
            stage.move_absolute({"X": 0, "Y": 0, "Z": 0, "E": 0})
            # move_absolute sends G90+G0+G91 (3 commands)
            time.sleep(2.0 if is_real else _zp_sim_wait(0, n_commands=3))
            pos_before = _zp_read_position(stage) if is_sim else stage.get_current_position()
            if is_real:
                time.sleep(MARLIN_CMD_INTERVAL_S)
            stage.move_relative({"X": 1.0})
            time.sleep(MARLIN_MOVE_SETTLE_S if is_real else _zp_sim_wait(1.0))
            pos_after = _zp_read_position(stage) if is_sim else stage.get_current_position()
            dx = pos_after[0] - pos_before[0]
            error = abs(dx - 1.0)
            METRICS.add_sample(f"zp_{label}_relative_move_accuracy_mm", error)
            self.assertGreater(dx, 0.0,
                msg=f"{label}: X didn't increase (dx={dx})")

    def test_11_m114_response_parseable(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_zp), ("real", self.real_zp)]:
            is_real = (label == "real")
            stage.send_data("M114")
            # Real Marlin needs more time to process + respond
            time.sleep(0.2 if is_real else 0.1)
            resp = stage.receive_data()
            has_axes = all(f"{ax}:" in resp for ax in ("X", "Y", "Z", "E"))
            self.assertTrue(has_axes,
                msg=f"{label} M114 response lacks axis data: '{resp[:100]}'")

    def test_20_emergency_stop_both(self):
        """Emergency stop on sim only; real Marlin M112 halts the board.

        Marlin M112 puts the board into a KILLED state that ignores all
        further commands until M999 (or physical reset). Sending it during
        tests would brick the connection for every subsequent test.
        We test M112 only on the simulator, which handles it gracefully.
        """
        self._skip_if_no_real()
        # Sim only — safe
        try:
            self.sim_zp.emergency_stop()
        except Exception as e:
            self.fail(f"sim: emergency_stop raised: {e}")

        # Real hardware: verify the method exists but DO NOT call M112.
        # Instead, verify the board is still responsive.
        self.assertTrue(hasattr(self.real_zp, 'emergency_stop'),
            "Real ZP should have emergency_stop method")
        # Confirm board is alive with a gentle M114
        time.sleep(MARLIN_CMD_INTERVAL_S)
        self.real_zp.send_data("M114")
        time.sleep(0.2)
        resp = self.real_zp.receive_data()
        self.assertIn("X:", resp,
            msg=f"Real ZP not responding after test setup: '{resp[:80]}'")

    def test_30_position_query_latency(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_zp), ("real", self.real_zp)]:
            is_real = (label == "real")
            # Real Marlin: pace queries to avoid buffer overflow.
            # We still measure per-query latency accurately — the inter_delay
            # is NOT included in the measurement, only the fn() call itself.
            delay = MARLIN_CMD_INTERVAL_S if is_real else 0.0
            n = 10 if is_real else 20  # fewer samples for real to be safe
            times_ms = _measure_latency_batch(
                stage.get_current_position, n=n, inter_delay=delay
            )
            METRICS.add_samples(f"zp_{label}_position_query_latency_ms", times_ms)

    def test_31_query_rate(self):
        """Sustained query rate — paced for real Marlin safety.

        For real Marlin, we measure the rate INCLUDING the mandatory
        inter-command delay, which gives the safe sustainable rate.
        The per-query latency (without delay) is in test_30.
        """
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_zp), ("real", self.real_zp)]:
            is_real = (label == "real")
            n = 10 if is_real else 30
            t0 = time.monotonic()
            for _ in range(n):
                stage.get_current_position()
                # Real Marlin: mandatory cooldown between commands
                if is_real:
                    time.sleep(MARLIN_CMD_INTERVAL_S)
            elapsed = time.monotonic() - t0
            rate = n / elapsed if elapsed > 0 else 0
            METRICS.add_sample(f"zp_{label}_query_rate_hz", rate)

    def test_32_gcode_round_trip(self):
        self._skip_if_no_real()
        for label, stage in [("sim", self.sim_zp), ("real", self.real_zp)]:
            is_real = (label == "real")
            n = 10 if is_real else 20
            # Real Marlin: wait long enough for command to be processed
            # and response to be fully transmitted back.
            wait_s = MARLIN_CMD_INTERVAL_S if is_real else 0.02
            times = []
            for _ in range(n):
                t0 = time.monotonic()
                stage.send_data("M114")
                time.sleep(wait_s)
                stage.receive_data()
                times.append((time.monotonic() - t0) * 1000)
                # Extra cooldown for real hardware between iterations
                if is_real:
                    time.sleep(MARLIN_CMD_INTERVAL_S)
            METRICS.add_samples(f"zp_{label}_gcode_round_trip_ms", times)


# ═══════════════════════════════════════════════════════════════════
#  5. CROSS-STAGE INTEGRATION (SIM ONLY)
# ═══════════════════════════════════════════════════════════════════

class TestCrossStageIntegration(unittest.TestCase):
    """Tests that XY and ZP simulators run concurrently without interference."""

    @classmethod
    def setUpClass(cls):
        cls.xy = XYStageManager(simulate=True)
        cls.zp = ZPStageManager(simulate=True)

    @classmethod
    def tearDownClass(cls):
        cls.xy.stop()
        cls.zp.stop()

    def test_01_concurrent_position_queries(self):
        xy_pos = self.xy.get_current_position()
        zp_pos = self.zp.get_current_position()
        self.assertEqual(len(xy_pos), 3)
        self.assertEqual(len(zp_pos), 4)

    def test_02_concurrent_movement(self):
        self.xy.set_home()
        self.zp.move_absolute({"X": 0, "Y": 0, "Z": 0, "E": 0})
        time.sleep(_zp_sim_wait(0, n_commands=3))

        self.xy.move_stage_relative(200.0, 100.0)
        self.zp.move_relative({"X": 2.0, "Y": 1.0})
        time.sleep(_zp_sim_wait(2.0))

        xy_pos = _wait_for_position_stable(
            self.xy.get_current_position, timeout=3.0, tolerance=1.0
        )
        zp_pos = _zp_read_position(self.zp)

        self.assertAlmostEqual(xy_pos[0], 200.0, delta=10.0)
        self.assertAlmostEqual(xy_pos[1], 100.0, delta=10.0)
        self.assertAlmostEqual(zp_pos[0], 2.0, delta=0.5,
            msg=f"ZP X={zp_pos[0]}, expected ~2.0")
        self.assertAlmostEqual(zp_pos[1], 1.0, delta=0.5,
            msg=f"ZP Y={zp_pos[1]}, expected ~1.0")

    def test_03_axis_map_logical_to_physical(self):
        self.zp.move_absolute({"X": 0, "Y": 0, "Z": 0, "E": 0})
        time.sleep(_zp_sim_wait(0, n_commands=3))
        printer_axis = AXIS_MAP["Z"]
        self.assertEqual(printer_axis, "X")
        self.zp.move_relative({printer_axis: 3.0})
        time.sleep(_zp_sim_wait(3.0))
        pos = _zp_read_position(self.zp)
        self.assertAlmostEqual(pos[0], 3.0, delta=0.5,
            msg=f"Z-needle (printer X) = {pos[0]}, expected 3.0")

    def test_04_all_pumps_move_independently(self):
        self.zp.move_absolute({"X": 0, "Y": 0, "Z": 0, "E": 0})
        time.sleep(_zp_sim_wait(3.0, n_commands=3))  # may travel from test_03
        self.zp.move_relative({AXIS_MAP["P1"]: 1.5})
        time.sleep(_zp_sim_wait(1.5))
        pos = _zp_read_position(self.zp)
        self.assertAlmostEqual(pos[1], 1.5, delta=0.5,
            msg=f"P1 (printer Y) = {pos[1]}, expected 1.5")
        self.assertAlmostEqual(pos[2], 0.0, delta=0.2,
            msg=f"P2 (printer Z) moved unexpectedly: {pos[2]}")
        self.assertAlmostEqual(pos[3], 0.0, delta=0.2,
            msg=f"P3 (printer E) moved unexpectedly: {pos[3]}")


# ═══════════════════════════════════════════════════════════════════
#  6. TIMING / PERFORMANCE CHARACTERISATION (+ all metrics)
# ═══════════════════════════════════════════════════════════════════

class TestTimingCharacterisation(unittest.TestCase):
    """Measures simulator timing — always-pass tests that log metrics."""

    @classmethod
    def setUpClass(cls):
        cls.xy = XYStageManager(simulate=True)
        cls.zp = ZPStageManager(simulate=True)

    @classmethod
    def tearDownClass(cls):
        cls.xy.stop()
        cls.zp.stop()

    def test_01_xy_position_query_rate(self):
        n = 100
        t0 = time.monotonic()
        for _ in range(n):
            self.xy.get_current_position()
        elapsed = time.monotonic() - t0
        rate = n / elapsed if elapsed > 0 else 0
        avg_ms = elapsed / n * 1000
        METRICS.add_sample("xy_sim_query_rate_hz", rate)
        METRICS.add_sample("xy_sim_position_query_latency_ms", avg_ms)
        print(f"\n  XY sim query: {rate:.1f} Hz ({avg_ms:.2f} ms/query)")
        self.assertGreater(rate, 1.0)

    def test_02_zp_position_query_rate(self):
        n = 50
        t0 = time.monotonic()
        for _ in range(n):
            self.zp.get_current_position()
        elapsed = time.monotonic() - t0
        rate = n / elapsed if elapsed > 0 else 0
        avg_ms = elapsed / n * 1000
        METRICS.add_sample("zp_sim_query_rate_hz", rate)
        METRICS.add_sample("zp_sim_position_query_latency_ms", avg_ms)
        print(f"\n  ZP sim query: {rate:.1f} Hz ({avg_ms:.2f} ms/query)")
        self.assertGreater(rate, 1.0)

    def test_03_xy_send_command_rate(self):
        n = 100
        times = _measure_latency_batch(lambda: self.xy.send_command("P"), n=n)
        rate = n / (sum(times) / 1000) if sum(times) > 0 else 0
        METRICS.add_samples("xy_sim_send_command_latency_ms", times)
        METRICS.add_sample("xy_sim_send_command_rate_hz", rate)
        print(f"\n  XY sim send_command('P'): {rate:.1f} Hz "
              f"({statistics.mean(times):.2f} ms avg)")

    def test_04_xy_relative_move_latency(self):
        """Measure time for a small XY relative move to settle."""
        self.xy.set_home()
        time.sleep(0.1)
        times = []
        for _ in range(10):
            t0 = time.monotonic()
            self.xy.move_stage_relative(10.0, 10.0)
            _wait_for_position_stable(
                self.xy.get_current_position, timeout=5.0, tolerance=0.5
            )
            times.append((time.monotonic() - t0) * 1000)
        METRICS.add_samples("xy_sim_small_move_settle_ms", times)
        avg = statistics.mean(times)
        print(f"\n  XY sim small move settle: {avg:.1f} ms avg")

    def test_05_zp_gcode_round_trip(self):
        times = []
        for _ in range(30):
            t0 = time.monotonic()
            self.zp.send_data("M114")
            time.sleep(0.02)
            self.zp.receive_data()
            times.append((time.monotonic() - t0) * 1000)
        METRICS.add_samples("zp_sim_gcode_round_trip_ms", times)
        avg = statistics.mean(times)
        print(f"\n  ZP sim M114 round-trip: {avg:.1f} ms avg")

    def test_06_xy_velocity_command_rate(self):
        """Measure VS command throughput — critical for jogging."""
        n = 50
        times = _measure_latency_batch(
            lambda: self.xy.move_stage_at_velocity(100.0, 0.0), n=n
        )
        self.xy.stop_stage()
        rate = n / (sum(times) / 1000) if sum(times) > 0 else 0
        METRICS.add_samples("xy_sim_velocity_command_latency_ms", times)
        METRICS.add_sample("xy_sim_velocity_command_rate_hz", rate)
        print(f"\n  XY sim VS rate: {rate:.1f} Hz ({statistics.mean(times):.2f} ms avg)")

    def test_07_xy_concurrent_query_during_move(self):
        """Position query rate while stage is in velocity mode."""
        self.xy.move_stage_at_velocity(500.0, 500.0)
        time.sleep(0.1)
        n = 50
        t0 = time.monotonic()
        for _ in range(n):
            self.xy.get_current_position()
        elapsed = time.monotonic() - t0
        self.xy.stop_stage()
        rate = n / elapsed if elapsed > 0 else 0
        METRICS.add_sample("xy_sim_query_rate_during_move_hz", rate)
        print(f"\n  XY sim query during move: {rate:.1f} Hz")

    def test_08_zp_concurrent_query_during_move(self):
        """ZP query rate while an axis is moving."""
        self.zp.move_relative({"X": 50.0})
        n = 20
        t0 = time.monotonic()
        for _ in range(n):
            self.zp.get_current_position()
        elapsed = time.monotonic() - t0
        rate = n / elapsed if elapsed > 0 else 0
        METRICS.add_sample("zp_sim_query_rate_during_move_hz", rate)
        print(f"\n  ZP sim query during move: {rate:.1f} Hz")


# ═══════════════════════════════════════════════════════════════════
#  7. SIMULATOR INTERNAL CONSISTENCY
# ═══════════════════════════════════════════════════════════════════

class TestXYSimulatorInternals(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = XYStageSimulator()
        cls.sim.start()

    @classmethod
    def tearDownClass(cls):
        cls.sim.stop()

    def test_01_direct_send_command(self):
        resp = self.sim.send_command("P")
        parts = resp.split(",")
        self.assertGreaterEqual(len(parts), 2)

    def test_02_get_current_position(self):
        pos = self.sim.get_current_position()
        self.assertEqual(len(pos), 3)

    def test_03_serial_like_interface(self):
        self.sim.write(b"P\r")
        self.sim.flush()
        time.sleep(0.1)
        data = self.sim.read_all()
        self.assertIsInstance(data, bytes)

    def test_04_is_open_property(self):
        self.assertTrue(self.sim.is_open)

    def test_05_reset_buffers(self):
        self.sim.reset_input_buffer()
        self.sim.reset_output_buffer()


class TestZPSimulatorInternals(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = ZPStageSimulator()
        cls.sim.start()

    @classmethod
    def tearDownClass(cls):
        cls.sim.stop()

    def test_01_serial_like_write_flush(self):
        self.sim.write(b"M115\n")
        self.sim.flush()
        time.sleep(0.1)
        data = self.sim.read_all()
        self.assertIsInstance(data, bytes)
        self.assertIn(b"FIRMWARE_NAME", data)

    def test_02_send_line_convenience(self):
        self.sim.send_line("M114")
        time.sleep(0.1)
        data = self.sim.read_all()
        text = data.decode("utf-8", errors="replace")
        self.assertIn("X:", text)

    def test_03_is_open_property(self):
        self.assertTrue(self.sim.is_open)

    def test_04_position_tracking(self):
        pos_before = dict(self.sim.position)
        self.sim.send_line("G0 X1.0")
        time.sleep(0.3)
        pos_after = dict(self.sim.position)
        self.assertNotAlmostEqual(pos_before["X"], pos_after["X"], delta=0.01)

    def test_05_absolute_vs_relative_mode(self):
        """G90/G91 mode switching works in simulator."""
        self.sim.send_line("G90")
        time.sleep(0.1)
        self.sim.send_line("G0 X5.0 Y5.0")
        time.sleep(_zp_sim_wait(5.0))
        self.sim.send_line("G91")
        time.sleep(0.1)
        self.sim.send_line("G0 X1.0")
        time.sleep(_zp_sim_wait(1.0))
        self.assertAlmostEqual(self.sim.position["X"], 6.0, delta=0.5,
            msg=f"X={self.sim.position['X']}, expected ~6.0")
        self.assertAlmostEqual(self.sim.position["Y"], 5.0, delta=0.5,
            msg=f"Y={self.sim.position['Y']}, expected ~5.0")

    def test_06_reset_buffers(self):
        self.sim.reset_input_buffer()
        self.sim.reset_output_buffer()


# ═══════════════════════════════════════════════════════════════════
#  8. EDGE CASES & ERROR HANDLING
# ═══════════════════════════════════════════════════════════════════

class TestEdgeCases(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xy = XYStageManager(simulate=True)
        cls.zp = ZPStageManager(simulate=True)

    @classmethod
    def tearDownClass(cls):
        cls.xy.stop()
        cls.zp.stop()

    def test_01_xy_large_move(self):
        self.xy.move_stage_to_position(50000.0, 50000.0)
        time.sleep(2.0)
        pos = self.xy.get_current_position()
        self.assertIsNotNone(pos[0])

    def test_02_xy_rapid_commands(self):
        for i in range(20):
            self.xy.send_command("P")

    def test_03_zp_rapid_commands(self):
        for i in range(20):
            self.zp.send_data("M114")
        time.sleep(0.5)

    def test_04_xy_empty_command(self):
        try:
            self.xy.send_command("")
        except Exception as e:
            self.fail(f"Empty command raised: {e}")

    def test_05_zp_unknown_gcode(self):
        try:
            self.zp.send_data("G999")
            time.sleep(0.1)
            self.zp.receive_data()
        except Exception as e:
            self.fail(f"Unknown G-code raised: {e}")

    def test_06_zp_negative_feedrate(self):
        try:
            self.zp.move_relative({"X": 0.1}, feedrate=-100)
        except Exception as e:
            self.fail(f"Negative feedrate raised: {e}")

    def test_07_xy_move_relative_zero(self):
        pos_before = self.xy.get_current_position()
        self.xy.move_stage_relative(0.0, 0.0)
        time.sleep(0.1)
        pos_after = self.xy.get_current_position()
        self.assertAlmostEqual(pos_before[0], pos_after[0], delta=1.0)
        self.assertAlmostEqual(pos_before[1], pos_after[1], delta=1.0)

    def test_08_multiple_stop_calls(self):
        xy2 = XYStageManager(simulate=True)
        xy2.stop()
        xy2.stop()

    def test_09_zp_multiple_stop_calls(self):
        zp2 = ZPStageManager(simulate=True)
        zp2.stop()
        zp2.stop()


# ═══════════════════════════════════════════════════════════════════
#  9. LIFECYCLE TESTS
# ═══════════════════════════════════════════════════════════════════

class TestLifecycle(unittest.TestCase):
    def test_01_xy_create_destroy(self):
        stage = XYStageManager(simulate=True)
        pos = stage.get_current_position()
        self.assertEqual(len(pos), 3)
        stage.stop()

    def test_02_zp_create_destroy(self):
        stage = ZPStageManager(simulate=True)
        pos = stage.get_current_position()
        self.assertEqual(len(pos), 4)
        stage.stop()

    def test_03_xy_create_multiple(self):
        s1 = XYStageManager(simulate=True)
        s2 = XYStageManager(simulate=True)
        s1.move_stage_relative(100, 0)
        s2.move_stage_relative(0, 100)
        time.sleep(0.5)
        p1 = s1.get_current_position()
        p2 = s2.get_current_position()
        self.assertGreater(abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]), 10.0)
        s1.stop()
        s2.stop()

    def test_04_zp_create_multiple(self):
        s1 = ZPStageManager(simulate=True)
        s2 = ZPStageManager(simulate=True)
        s1.move_relative({"X": 5.0})
        s2.move_relative({"Y": 5.0})
        time.sleep(_zp_sim_wait(5.0))
        p1 = _zp_read_position(s1)
        p2 = _zp_read_position(s2)
        self.assertAlmostEqual(p1[0], 5.0, delta=0.8,
            msg=f"s1 X={p1[0]}, expected ~5.0")
        self.assertAlmostEqual(p2[1], 5.0, delta=0.8,
            msg=f"s2 Y={p2[1]}, expected ~5.0")
        s1.stop()
        s2.stop()


# ═══════════════════════════════════════════════════════════════════
#  10. DIAGNOSTIC-LEVEL XY TESTS (matches proscan_diagnostic.py)
# ═══════════════════════════════════════════════════════════════════

class TestXYDiagnosticLevel(unittest.TestCase):
    """
    Per-command-type timing, VS speed levels, sustained rate, and
    sine wave tracking — matching proscan_diagnostic.py coverage.
    Sim always runs; real hardware tests auto-skip if not connected.
    """

    sim_xy: Optional[XYStageManager] = None
    real_xy: Optional[XYStageManager] = None

    @classmethod
    def setUpClass(cls):
        cls.sim_xy = XYStageManager(simulate=True)
        cls.real_xy = _try_connect_real_xy()
        if cls.real_xy is not None:
            METRICS.real_xy_available = True

    @classmethod
    def tearDownClass(cls):
        if cls.sim_xy:
            cls.sim_xy.stop()
        if cls.real_xy:
            cls.real_xy.stop()

    def _stages(self):
        """Yield (label, stage, is_real) for connected stages."""
        yield ("sim", self.sim_xy, False)
        if self.real_xy is not None:
            yield ("real", self.real_xy, True)

    def test_01_per_command_timing(self):
        """Measure round-trip for each command type separately."""
        commands = [
            ("P", "position"),
            ("VS,0,0", "velocity"),
            ("SMS", "setting"),
            ("SAS", "setting"),
            ("$", "status"),
        ]
        for label, stage, is_real in self._stages():
            delay = PROSCAN_CMD_INTERVAL_S if is_real else 0.0
            n = 10 if is_real else 20
            for cmd, cmd_type in commands:
                times = _measure_latency_batch(
                    lambda c=cmd, s=stage: s.send_command(c),
                    n=n, inter_delay=delay,
                )
                key = f"xy_{label}_{cmd_type}_cmd_latency_ms"
                METRICS.add_samples(key, times)
            # Stop any velocity from the VS test
            stage.send_command("I")
            if is_real:
                time.sleep(0.1)

    def test_02_vs_speed_levels(self):
        """VS at different velocities — check timing is speed-independent."""
        speeds = [(0, 0), (100, 0), (500, 500), (1000, 0), (5000, 0)]
        for label, stage, is_real in self._stages():
            delay = PROSCAN_CMD_INTERVAL_S if is_real else 0.0
            n = 5 if is_real else 10
            for vx, vy in speeds:
                times = _measure_latency_batch(
                    lambda s=stage, x=vx, y=vy: s.send_command(f"VS,{x},{y}"),
                    n=n, inter_delay=delay,
                )
                key = f"xy_{label}_vs_{vx}_{vy}_latency_ms"
                METRICS.add_samples(key, times)
            stage.send_command("VS,0,0")
            stage.send_command("I")
            if is_real:
                time.sleep(0.1)

    def test_03_vs_direction_alternation(self):
        """VS ±500 alternation — measures consistency under direction changes."""
        for label, stage, is_real in self._stages():
            times = []
            n = 10 if is_real else 20
            for i in range(n):
                v = 500 if i % 2 == 0 else -500
                t0 = time.monotonic()
                stage.send_command(f"VS,{v},0")
                times.append((time.monotonic() - t0) * 1000)
                if is_real:
                    time.sleep(PROSCAN_CMD_INTERVAL_S)
            stage.send_command("VS,0,0")
            stage.send_command("I")
            METRICS.add_samples(f"xy_{label}_vs_alternation_latency_ms", times)

    def test_04_gr_move_timing(self):
        """GR (relative move) command timing — separate from position query."""
        for label, stage, is_real in self._stages():
            delay = PROSCAN_CMD_INTERVAL_S if is_real else 0.0
            n = 10 if is_real else 20
            # Tiny 1-unit moves so we don't accumulate drift
            times = []
            for i in range(n):
                dx = 1 if i % 2 == 0 else -1
                t0 = time.monotonic()
                stage.send_command(f"GR,{dx},0")
                times.append((time.monotonic() - t0) * 1000)
                if is_real:
                    time.sleep(delay)
            METRICS.add_samples(f"xy_{label}_gr_cmd_latency_ms", times)

    def test_05_sustained_rate_5s(self):
        """5-second continuous position query — measures real sustained rate."""
        for label, stage, is_real in self._stages():
            duration = 5.0
            count = 0
            t0 = time.monotonic()
            while time.monotonic() - t0 < duration:
                stage.get_current_position()
                count += 1
                if is_real:
                    time.sleep(PROSCAN_CMD_INTERVAL_S)
            elapsed = time.monotonic() - t0
            rate = count / elapsed if elapsed > 0 else 0
            METRICS.add_sample(f"xy_{label}_sustained_5s_rate_hz", rate)
            print(f"\n  XY {label} sustained 5s: {rate:.1f} Hz ({count} queries)")

    def test_06_sine_tracking(self):
        """Sine wave tracking — commands velocity sine, measures position error.

        Only runs on sim (safe, no physical motion concerns).
        Measures how well the physics model tracks a continuous trajectory.
        """
        stage = self.sim_xy
        stage.set_home()
        time.sleep(0.2)

        AMPLITUDE = 300.0  # µm
        VS_RATE_HZ = 50
        dt = 1.0 / VS_RATE_HZ
        freqs = [0.1, 0.2, 0.5, 1.0]

        for freq in freqs:
            stage.set_home()
            time.sleep(0.2)

            errors = []
            n_cycles = max(2, int(2.0 * freq))  # at least 2 cycles
            total_time = n_cycles / freq
            n_steps = int(total_time * VS_RATE_HZ)

            for step in range(n_steps):
                t = step * dt
                # Commanded velocity = derivative of A*sin(2πft) = A*2πf*cos(2πft)
                vx = AMPLITUDE * 2 * math.pi * freq * math.cos(2 * math.pi * freq * t)
                stage.move_stage_at_velocity(vx, 0.0)
                time.sleep(dt)

                # Expected position
                expected_x = AMPLITUDE * math.sin(2 * math.pi * freq * t)
                actual = stage.get_current_position()
                if actual[0] is not None:
                    errors.append(abs(actual[0] - expected_x))

            stage.stop_stage()
            time.sleep(0.2)

            if errors:
                rms = math.sqrt(sum(e**2 for e in errors) / len(errors))
                max_err = max(errors)
                METRICS.add_sample(f"xy_sim_sine_{freq}hz_rms_um", rms)
                METRICS.add_sample(f"xy_sim_sine_{freq}hz_max_um", max_err)
                print(f"\n  Sine {freq}Hz: RMS={rms:.1f}µm, max={max_err:.1f}µm")

        METRICS.set_record("xy_sim_sine_tracking", {
            "amplitude_um": AMPLITUDE,
            "vs_rate_hz": VS_RATE_HZ,
            "frequencies_tested": freqs,
        })


# ═══════════════════════════════════════════════════════════════════
#  11. DIAGNOSTIC-LEVEL ZP TESTS
# ═══════════════════════════════════════════════════════════════════

class TestZPDiagnosticLevel(unittest.TestCase):
    """
    Per-G-code timing, feedrate comparison, and sustained polling
    for the Marlin-based ZP stage.
    """

    sim_zp: Optional[ZPStageManager] = None
    real_zp: Optional[ZPStageManager] = None

    @classmethod
    def setUpClass(cls):
        cls.sim_zp = ZPStageManager(simulate=True)
        cls.real_zp = _try_connect_real_zp()
        if cls.real_zp is not None:
            METRICS.real_zp_available = True

    @classmethod
    def tearDownClass(cls):
        if cls.sim_zp:
            cls.sim_zp.stop()
        if cls.real_zp:
            cls.real_zp.stop()

    def _stages(self):
        yield ("sim", self.sim_zp, False)
        if self.real_zp is not None:
            yield ("real", self.real_zp, True)

    def test_01_per_gcode_timing(self):
        """Round-trip for each G-code type."""
        gcodes = [
            ("M114", "m114"),
            ("M115", "m115"),
            ("G91", "g91"),
            ("G90", "g90"),
        ]
        for label, stage, is_real in self._stages():
            delay = MARLIN_CMD_INTERVAL_S if is_real else 0.0
            n = 5 if is_real else 10
            for cmd, cmd_key in gcodes:
                times = []
                for _ in range(n):
                    t0 = time.monotonic()
                    stage.send_data(cmd)
                    time.sleep(0.05 if is_real else 0.02)
                    stage.receive_data()
                    times.append((time.monotonic() - t0) * 1000)
                    if is_real:
                        time.sleep(delay)
                METRICS.add_samples(f"zp_{label}_{cmd_key}_latency_ms", times)
            # Ensure we're back in relative mode
            stage.send_data("G91")
            time.sleep(0.1)

    def test_02_move_feedrate_comparison(self):
        """Move timing at different feedrates — sim only (safe)."""
        stage = self.sim_zp
        feedrates = [100, 200, 500]
        for fr in feedrates:
            # Reset to zero
            stage.move_absolute({"X": 0, "Y": 0, "Z": 0, "E": 0})
            time.sleep(_zp_sim_wait(5.0, n_commands=3))

            t0 = time.monotonic()
            stage.move_relative({"X": 2.0}, feedrate=float(fr))
            time.sleep(_zp_sim_wait(2.0))
            elapsed_ms = (time.monotonic() - t0) * 1000

            pos = _zp_read_position(stage)
            METRICS.set_record(f"zp_sim_move_F{fr}", {
                "feedrate": fr,
                "target_mm": 2.0,
                "actual_mm": round(pos[0], 3),
                "elapsed_ms": round(elapsed_ms, 1),
            })

    def test_03_sustained_m114_rate_5s(self):
        """5-second continuous M114 polling."""
        for label, stage, is_real in self._stages():
            duration = 5.0
            count = 0
            t0 = time.monotonic()
            while time.monotonic() - t0 < duration:
                stage.get_current_position()
                count += 1
                if is_real:
                    time.sleep(MARLIN_CMD_INTERVAL_S)
            elapsed = time.monotonic() - t0
            rate = count / elapsed if elapsed > 0 else 0
            METRICS.add_sample(f"zp_{label}_sustained_5s_rate_hz", rate)
            print(f"\n  ZP {label} sustained 5s: {rate:.1f} Hz ({count} queries)")


# ═══════════════════════════════════════════════════════════════════
#  REPORT WRITER — hooked into test runner teardown
# ═══════════════════════════════════════════════════════════════════

class _ReportWriter(unittest.TestCase):
    """
    Runs last (z-sorted by name). Writes the calibration report.
    Not a real test — just a hook to generate the output file.
    """

    def test_zz_write_calibration_report(self):
        """Generate config/sim_calibration_report.json with all metrics."""
        report = METRICS.write_report()

        # Print summary to console
        print(f"\n{'='*70}")
        print("  CALIBRATION METRICS SUMMARY")
        print(f"{'='*70}")
        print(f"  Real XY: {'CONNECTED' if METRICS.real_xy_available else 'not found'}")
        print(f"  Real ZP: {'CONNECTED' if METRICS.real_zp_available else 'not found'}")
        print()

        metrics = report.get("metrics", {})
        for key in sorted(metrics):
            val = metrics[key]
            if isinstance(val, dict) and "mean" in val:
                print(f"  {key}: mean={val['mean']:.2f} "
                      f"(±{val.get('stdev',0):.2f}, n={val.get('n',0)})")

        recs = report.get("recommendations", [])
        if recs:
            # Separate critical warnings from normal tuning advice
            critical = [r for r in recs if r.get("severity") == "CRITICAL"]
            info = [r for r in recs if r.get("severity") == "INFO"]
            tuning = [r for r in recs if "severity" not in r]

            if critical:
                print(f"\n  {'!'*60}")
                print(f"  SERIAL COMMUNICATION BUGS DETECTED ({len(critical)}):")
                print(f"  {'!'*60}")
                for i, rec in enumerate(critical, 1):
                    print(f"    {i}. {rec.get('action', 'N/A')}")
                for rec in info:
                    print(f"    → {rec.get('action', '')}")

            if tuning:
                label = "TUNING (may be invalid — see above)" if critical else "TUNING"
                print(f"\n  {label} ({len(tuning)}):")
                for i, rec in enumerate(tuning, 1):
                    print(f"    {i}. {rec.get('action', 'N/A')}")
        print(f"{'='*70}\n")


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 70)
    print("  MEBP Simulator vs. Hardware Conformance Tests + Calibration")
    print("=" * 70)
    print(f"  SKIP_REAL:     {SKIP_REAL}")
    print(f"  FORCE_REAL_XY: {FORCE_REAL_XY}")
    print(f"  FORCE_REAL_ZP: {FORCE_REAL_ZP}")
    print(f"  Project root:  {_PROJECT_ROOT}")
    print(f"  Report path:   {_REPORT_PATH}")
    print("=" * 70)

    # Quick hardware check
    real_xy = _try_connect_real_xy()
    real_zp = _try_connect_real_zp()
    print(f"\n  Real XY: {'CONNECTED' if real_xy else 'not found'}")
    print(f"  Real ZP: {'CONNECTED' if real_zp else 'not found'}")
    if real_xy:
        real_xy.stop()
    if real_zp:
        real_zp.stop()
    print()

    unittest.main(verbosity=2)