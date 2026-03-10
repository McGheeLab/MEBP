# Coding Plan: Simulator vs. Hardware Conformance Test Suite + Calibration Report

**File:** `tests/test_sim_vs_hardware.py`
**Output:** `config/sim_calibration_report.json`
**Status:** Complete — single session, no patches needed
**Date:** March 2026

---

## Goal

Create a test suite that compares sim vs real hardware behavior and writes a JSON calibration report mapping measured metrics to the exact simulator parameters they inform, with suggested adjustments.

---

## No Patch Script Needed

This is a **new file** — drop `tests/test_sim_vs_hardware.py` into the `tests/` directory. The report is auto-generated at `config/sim_calibration_report.json`.

---

## Architecture: MetricsCollector + Report

The test suite uses a global `MetricsCollector` singleton that accumulates measurements during all test runs. After all tests complete, a special final test (`_ReportWriter.test_zz_write_calibration_report`) writes the JSON report.

### Report Structure

```json
{
  "_description": "Simulator calibration report...",
  "_generated": "2026-03-09T...",
  "hardware_detected": { "real_xy": true, "real_zp": false },
  "metrics": {
    "xy_sim_position_query_latency_ms": { "n":130, "mean":12.1, "stdev":0.2, ... },
    "xy_real_position_query_latency_ms": { "n":30, "mean":11.9, "stdev":0.3, ... },
    "xy_sim_relative_move_accuracy_um": { ... },
    "xy_real_relative_move_accuracy_um": { ... },
    ...
  },
  "parameter_mapping": {
    "xy_sim_position_query_latency_ms": {
      "file": "SupportClasses/XYStageSimulator.py",
      "config_file": "config/xy_diagnostic_profile.json",
      "config_keys": ["simulator_derived_constants.processing_time_position_s"],
      ...
    },
    ...
  },
  "recommendations": [
    {
      "parameter": "config/xy_diagnostic_profile.json → ...",
      "current_sim_latency_ms": 12.5,
      "measured_real_latency_ms": 11.9,
      "suggested_processing_time_s": 0.0079,
      "action": "Sim is slower than real by 5%. Update processing_time_position_s..."
    }
  ]
}
```

### Parameter Mapping (key reference)

| Metric | Simulator File | Tunable Parameters | Config File |
|--------|---------------|-------------------|-------------|
| `xy_*_position_query_latency_ms` | XYStageSimulator.py | `PROCESSING_TIMES['position']`, baud rate | `config/xy_diagnostic_profile.json` → `simulator_derived_constants.processing_time_position_s` |
| `xy_*_relative_move_accuracy_um` | XYStageSimulator.py | `DEFAULT_KP`, `SETTLE_THRESHOLD_UM`, `MAX_ACCEL_UM_S2`, `PHYSICS_HZ` | Module constants |
| `xy_*_absolute_move_settle_time_ms` | XYStageSimulator.py | `MAX_SPEED_UM_S`, `MAX_ACCEL_UM_S2`, `DEFAULT_KP` | Module constants |
| `xy_*_velocity_mode_drift_after_stop_um` | XYStageSimulator.py | `MAX_ACCEL_UM_S2` (deceleration) | Module constants |
| `xy_*_query_rate_hz` | XYStageSimulator.py | All processing times | `config/xy_diagnostic_profile.json` → `sustained_rate_hz` |
| `zp_*_position_query_latency_ms` | ZPStageSimulator.py | `communication_delay`, `processing_time_per_command` | Constructor args |
| `zp_*_relative_move_accuracy_mm` | ZPStageSimulator.py | `kp`, `acceleration_rate`, `max_speed` | Constructor args |
| `zp_*_gcode_round_trip_ms` | ZPStageSimulator.py | `communication_delay` + `processing_time_per_command` | Constructor args |

### How Recommendations Are Generated

When real hardware metrics are available, the report compares sim vs real for each metric pair. If they diverge by more than a threshold (typically 15-20%), it calculates what the simulator parameter should be and emits an actionable recommendation with the exact config key to update.

For example, if the real XY latency is 11.9ms but sim is 14.2ms:
- Total = TX_baud + processing + RX_baud
- TX ≈ 0.52ms, RX ≈ 3.6ms at 38400 baud
- processing = 11.9 - 0.52 - 3.6 ≈ 7.78ms
- Recommendation: set `processing_time_position_s` to 0.0078

---

## Test Sections (9 sections, ~75 tests)

### Section 1: XY Simulated-Only (13 tests)
Initialization, position query, send_command, relative moves (+accuracy metric), absolute moves (+settle time metric), set_home, stop (+drift metric), velocity mode, settings.

### Section 2: ZP Simulated-Only (15 tests)
Initialization, position query, send_data/receive_data, relative moves (+accuracy metric), absolute moves, mode switching, emergency stop, feedrate, AXIS_MAP.

### Section 3: XY Sim-vs-Hardware (8 tests, auto-skip)
API parity (return types), movement direction, set_home, stop (+drift metric), position query latency (+metric), send_command latency (+metric), query rate (+metric), absolute move settle time (+metric).

### Section 4: ZP Sim-vs-Hardware (7 tests, auto-skip)
API parity, relative move (+accuracy metric), M114 parsing, emergency stop, position query latency (+metric), query rate (+metric), G-code round trip (+metric).

### Section 5: Cross-Stage Integration (4 tests)
Concurrent queries, concurrent movement, AXIS_MAP mapping, pump independence.

### Section 6: Timing Characterisation (8 tests)
XY query rate, ZP query rate, XY send_command rate, XY small-move settle, ZP G-code round trip, XY velocity command rate, XY/ZP query during move.

### Section 7: Simulator Internals (11 tests)
Direct interfaces, serial-like I/O, buffer management.

### Section 8: Edge Cases (9 tests)
Large moves, rapid commands, empty commands, unknown G-code, negative feedrate, zero moves, multiple stop calls.

### Section 9: Lifecycle (4 tests)
Clean create/destroy, multiple instances.

---

## How to Run

```bash
# From project root — sim-only (generates baseline report):
python -m unittest tests.test_sim_vs_hardware -v

# With real XY (generates comparison report with recommendations):
MEBP_TEST_REAL_XY=1 python -m unittest tests.test_sim_vs_hardware -v

# With both real stages:
MEBP_TEST_REAL_XY=1 MEBP_TEST_REAL_ZP=1 python -m unittest tests.test_sim_vs_hardware -v

# Skip all real hardware:
MEBP_TEST_SKIP_REAL=1 python -m unittest tests.test_sim_vs_hardware -v
```

After running, open `config/sim_calibration_report.json` to see metrics and recommendations.

---

## Files Delivered

| File | Destination | Purpose |
|------|-------------|---------|
| `tests/__init__.py` | `tests/__init__.py` | Package init |
| `tests/test_sim_vs_hardware.py` | `tests/test_sim_vs_hardware.py` | Test suite + metrics (1671 lines) |
| `CODING_PLAN_sim_vs_hardware_tests.md` | Project docs | This plan |

---

## XY Simulator Current Defaults (for reference)

| Parameter | Value | Source |
|-----------|-------|--------|
| `MAX_SPEED_UM_S` | 20,000 | Module constant |
| `MAX_ACCEL_UM_S2` | 50,000 | Module constant |
| `DEFAULT_KP` | 15.0 | Module constant |
| `SETTLE_THRESHOLD_UM` | 0.5 | Module constant |
| `PHYSICS_HZ` | 200 | Module constant |
| `DEFAULT_BAUD` | 38,400 | Module constant / diagnostic profile |
| Processing times | ~8ms all types | `config/xy_diagnostic_profile.json` |

## ZP Simulator Current Defaults (for reference)

| Parameter | Value | Source |
|-----------|-------|--------|
| `communication_delay` | 0.03 s (30 ms) | Constructor default |
| `processing_time_per_command` | 0.01 s (10 ms) | Constructor default |
| `acceleration_rate` | 100.0 | Constructor default |
| `max_speed` | 100.0 | Constructor default |
| `kp` | 2.0 | Constructor default |
| Physics loop rate | ~100 Hz | Hardcoded `sleep(0.01)` |
