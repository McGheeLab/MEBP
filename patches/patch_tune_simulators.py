#!/usr/bin/env python3
"""
Tune XY and ZP simulators from sim_calibration_report.json measurements.

Reads the calibration report and applies measured values to both simulators
so their timing and physics match real hardware behavior.

Changes:
  XYStageSimulator.py:
    - PROCESSING_TIMES: 0.012 → 0.0001 (real controller processing is ~0ms;
      the 4.16ms round-trip is entirely baud-rate delay, which the sim
      already models separately)
    - MAX_ACCEL_UM_S2: 50,000 → 400,000 (real stage settles 300µm in 59ms)
    - DEFAULT_KP: 15.0 → 50.0 (faster P-controller convergence)
    - Fix _DEFAULT_PROCESSING_TIMES → PROCESSING_TIMES reference bug (line 115)
    - Update docstring with real calibration data

  ZPStageSimulator.py:
    - communication_delay: 0.03 → 0.002 (real Marlin overhead is ~1ms)
    - processing_time_per_command: 0.01 → 0.002 (real processing is ~1ms)
    - kp: 2.0 → 10.0 (faster position convergence, matches real accuracy)
    - acceleration_rate: 100 → 500 (real stepper acceleration is much higher)
    - max_speed: 100 → 500 (real Marlin max feedrate is higher)

  config/xy_diagnostic_profile.json:
    - Update simulator_derived_constants with real measurements

Data source: config/sim_calibration_report.json

Run: python patches/patch_tune_simulators.py
"""

import ast
import json
import re
import sys
from pathlib import Path


def find_root() -> Path:
    here = Path(__file__).resolve().parent
    for p in [here, here.parent, here.parent.parent, Path.cwd()]:
        if (p / "SupportClasses").is_dir():
            return p
    print("ERROR: Could not find MEBP project root")
    sys.exit(1)


def load_calibration(root: Path) -> dict:
    """Load calibration report. Returns metrics dict."""
    path = root / "config" / "sim_calibration_report.json"
    if not path.exists():
        print(f"  WARNING: {path} not found — using hardcoded values from analysis")
        return {}
    with open(path) as f:
        return json.load(f).get("metrics", {})


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify then write."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ❌ AST FAIL for {label}: {e}")
        return False
    path.write_text(content, encoding="utf-8")
    print(f"  ✅ Written: {label}")
    return True


def patch_xy_simulator(root: Path, metrics: dict):
    """Tune XYStageSimulator.py from calibration data."""
    path = root / "SupportClasses" / "XYStageSimulator.py"
    if not path.exists():
        print(f"  ❌ {path} not found")
        return False

    content = path.read_text(encoding="utf-8")
    changes = 0

    # ── 1. Fix _DEFAULT_PROCESSING_TIMES bug ─────────────────────
    if "_DEFAULT_PROCESSING_TIMES" in content:
        content = content.replace(
            "return dict(_DEFAULT_PROCESSING_TIMES)",
            "return dict(PROCESSING_TIMES)"
        )
        changes += 1
        print("    Fixed: _DEFAULT_PROCESSING_TIMES → PROCESSING_TIMES")

    # ── 2. Update PROCESSING_TIMES ───────────────────────────────
    # Real hardware: 4.16ms total round-trip = 0.52ms TX + ~0ms processing + 3.6ms RX
    # Processing is essentially zero. The baud delay handles everything.
    old_proc = 'PROCESSING_TIMES = {\n    "position":  0.012,'
    if old_proc in content:
        new_proc = (
            '# v7.2.8-cal: Tuned from sim_calibration_report.json\n'
            '# Real ProScan II: 4.16ms total round-trip at 38400 baud.\n'
            '# TX delay (0.52ms) + RX delay (3.6ms) = 4.12ms.\n'
            '# Controller processing = 4.16 - 4.12 = ~0ms.\n'
            '# Set processing times to near-zero; baud model handles the rest.\n'
            'PROCESSING_TIMES = {\n'
            '    "position":  0.0001,'
        )
        content = content.replace(old_proc, new_proc)
        changes += 1
        print("    Updated: PROCESSING_TIMES header + position")

    # Replace each individual processing time
    proc_replacements = [
        ('"move":      0.012,', '"move":      0.0001,'),
        ('"velocity":  0.012,', '"velocity":  0.0001,'),
        ('"setting":   0.012,', '"setting":   0.0001,'),
        ('"stop":      0.012,', '"stop":      0.0001,'),
        ('"default":   0.012,', '"default":   0.0001,'),
    ]
    for old, new in proc_replacements:
        if old in content:
            content = content.replace(old, new)
            changes += 1

    # Update the backward-compat constant
    if 'PROCESSING_TIME_S = 0.012' in content:
        content = content.replace(
            'PROCESSING_TIME_S = 0.012',
            'PROCESSING_TIME_S = 0.0001  # v7.2.8-cal: tuned from real hardware'
        )
        changes += 1

    # ── 3. Update physics constants ──────────────────────────────
    # Real settle: 300µm in 59ms → need faster physics
    # KP=50 gives velocity=min(MAX_SPEED, 50*300)=10000 µm/s → ~30ms travel
    # MAX_ACCEL=400000 allows instant velocity changes (real stepper is very fast)
    if "MAX_ACCEL_UM_S2 = 50_000.0" in content:
        content = content.replace(
            "MAX_ACCEL_UM_S2 = 50_000.0",
            "MAX_ACCEL_UM_S2 = 400_000.0  # v7.2.8-cal: real settles 300µm in 59ms"
        )
        changes += 1
        print("    Updated: MAX_ACCEL_UM_S2 50,000 → 400,000")

    if "DEFAULT_KP = 15.0" in content:
        content = content.replace(
            "DEFAULT_KP = 15.0",
            "DEFAULT_KP = 50.0  # v7.2.8-cal: faster convergence to match real settle time"
        )
        changes += 1
        print("    Updated: DEFAULT_KP 15.0 → 50.0")

    # ── 4. Update docstring ──────────────────────────────────────
    old_doc = (
        "  ALL commands: ~12ms round-trip, ~83 Hz burst, ~62.5 Hz sustained\n"
        "  Processing (excl baud delay): ~8ms uniform across P, VS, G, GR, $"
    )
    new_doc = (
        "  ALL commands: ~4.2ms round-trip, ~240 Hz burst, ~41 Hz sustained (with pacing)\n"
        "  Processing (excl baud delay): ~0ms — baud delay accounts for entire round-trip"
    )
    if old_doc in content:
        content = content.replace(old_doc, new_doc)
        changes += 1

    # ── 5. Update comment blocks ─────────────────────────────────
    old_comment = (
        "# Per-command processing times measured on real Prior ProScan II\n"
        "# (COM4 @ 38400 baud, 2026-03-09 diagnostic).\n"
        "# All commands return in ~12ms; sustained poll rate ~62 Hz\n"
        "# including Python serial overhead. VS has NO extra motor-ramp\n"
        "# blocking in Standard (COMP,0) mode — it responds immediately."
    )
    new_comment = (
        "# v7.2.8-cal: Per-command processing measured from sim_calibration_report.json.\n"
        "# Real ProScan II @ 38400 baud: position query = 4.16ms total round-trip.\n"
        "# Controller processing is ~0ms. The 4.16ms is entirely baud-rate TX/RX delay.\n"
        "# Sustained poll rate: ~41 Hz (with 20ms inter-command pacing).\n"
        "# These near-zero values let the baud-rate model handle all timing."
    )
    if old_comment in content:
        content = content.replace(old_comment, new_comment)
        changes += 1

    if changes == 0:
        print("    ⚠️  No changes matched — file may already be tuned")
        return True

    print(f"    Applied {changes} changes")
    return safe_write(path, content, "XYStageSimulator.py")


def patch_zp_simulator(root: Path, metrics: dict):
    """Tune ZPStageSimulator.py from calibration data."""
    path = root / "SupportClasses" / "ZPStageSimulator.py"
    if not path.exists():
        print(f"  ❌ {path} not found")
        return False

    content = path.read_text(encoding="utf-8")
    changes = 0

    # ── 1. Update constructor defaults ────────────────────────────
    # Real ZP: position query = 10.6ms (includes 10ms sleep in receive_data).
    # Actual Marlin processing < 1ms. comm_delay = serial latency ≈ 2ms.
    replacements = [
        # communication_delay
        (
            "communication_delay: float = 0.03,",
            "communication_delay: float = 0.002,  # v7.2.8-cal: real Marlin serial ~2ms",
        ),
        # processing_time_per_command
        (
            "processing_time_per_command: float = 0.01,",
            "processing_time_per_command: float = 0.002,  # v7.2.8-cal: real Marlin proc ~2ms",
        ),
        # acceleration_rate
        (
            "acceleration_rate: float = 100.0,",
            "acceleration_rate: float = 500.0,  # v7.2.8-cal: real stepper accel is fast",
        ),
        # max_speed
        (
            "max_speed: float = 100.0,",
            "max_speed: float = 500.0,  # v7.2.8-cal: real Marlin max feedrate is higher",
        ),
        # kp
        (
            "kp: float = 2.0,",
            "kp: float = 10.0,  # v7.2.8-cal: faster convergence, matches real 0.0mm error",
        ),
    ]

    for old, new in replacements:
        if old in content:
            content = content.replace(old, new)
            changes += 1
            param = old.split(":")[0].strip()
            old_val = old.split("=")[1].split(",")[0].strip()
            new_val = new.split("=")[1].split(",")[0].strip()
            print(f"    {param}: {old_val} → {new_val}")

    # ── 2. Update class docstring defaults ────────────────────────
    doc_replacements = [
        ("communication_delay:          Simulated serial write latency (s).",
         "communication_delay:          Simulated serial write latency (s). Default 0.002."),
        ("processing_time_per_command:  Simulated per-command processing time (s).",
         "processing_time_per_command:  Simulated per-command processing time (s). Default 0.002."),
    ]
    for old, new in doc_replacements:
        if old in content:
            content = content.replace(old, new)

    if changes == 0:
        print("    ⚠️  No changes matched — file may already be tuned")
        return True

    print(f"    Applied {changes} changes")
    return safe_write(path, content, "ZPStageSimulator.py")


def patch_diagnostic_profile(root: Path, metrics: dict):
    """Update xy_diagnostic_profile.json with calibration data."""
    path = root / "config" / "xy_diagnostic_profile.json"
    if not path.exists():
        print(f"  ⚠️  {path} not found — skipping profile update")
        return True

    with open(path) as f:
        profile = json.load(f)

    # Update simulator_derived_constants
    profile["simulator_derived_constants"] = {
        "_note": (
            "v7.2.8-cal: Tuned from sim_calibration_report.json. "
            "Real position query = 4.16ms total. Processing = ~0ms. "
            "All timing is baud-rate TX/RX delay."
        ),
        "processing_time_position_s": 0.0001,
        "processing_time_move_s": 0.0001,
        "processing_time_velocity_s": 0.0001,
        "processing_time_setting_s": 0.0001,
        "processing_time_stop_s": 0.0001,
        "processing_time_default_s": 0.0001,
    }

    # Add calibration summary
    profile["calibration_summary"] = {
        "_source": "sim_calibration_report.json",
        "xy_real_position_query_ms": round(metrics.get(
            "xy_real_position_query_latency_ms", {}).get("mean", 4.16), 2),
        "xy_real_sustained_rate_hz": round(metrics.get(
            "xy_real_sustained_5s_rate_hz", {}).get("mean", 41.0), 1),
        "xy_real_settle_300um_ms": round(metrics.get(
            "xy_real_absolute_move_settle_time_ms", {}).get("mean", 59.0), 1),
        "zp_real_position_query_ms": round(metrics.get(
            "zp_real_position_query_latency_ms", {}).get("mean", 11.0), 2),
        "zp_real_sustained_rate_hz": round(metrics.get(
            "zp_real_sustained_5s_rate_hz", {}).get("mean", 9.0), 1),
    }

    with open(path, "w") as f:
        json.dump(profile, f, indent=4)
    print(f"  ✅ Updated: xy_diagnostic_profile.json")
    return True


def main():
    root = find_root()
    print("=" * 60)
    print("  Tune Simulators from Calibration Report")
    print("=" * 60)
    print(f"  Project root: {root}")

    metrics = load_calibration(root)

    # ── XY Simulator ──────────────────────────────────────────────
    print(f"\n[1] XYStageSimulator.py")
    xy_ok = patch_xy_simulator(root, metrics)

    # ── ZP Simulator ──────────────────────────────────────────────
    print(f"\n[2] ZPStageSimulator.py")
    zp_ok = patch_zp_simulator(root, metrics)

    # ── Diagnostic Profile ────────────────────────────────────────
    print(f"\n[3] xy_diagnostic_profile.json")
    profile_ok = patch_diagnostic_profile(root, metrics)

    # ── Summary ───────────────────────────────────────────────────
    print(f"\n{'='*60}")
    if xy_ok and zp_ok:
        print("  ✅ All patches applied successfully")
        print("\n  Expected improvements after re-running tests:")
        print("    XY position query: 13.6ms → ~4.2ms (matches real)")
        print("    XY settle (300µm): 314ms → ~60ms (matches real)")
        print("    ZP position query: 41ms → ~14ms (matches real)")
        print("    ZP move accuracy: 0.05mm → ~0.0mm (matches real)")
        print("\n  Re-run: python tests/test_sim_vs_hardware.py")
    else:
        print("  ⚠️  Some patches failed — check errors above")
    print("=" * 60)


if __name__ == "__main__":
    main()
