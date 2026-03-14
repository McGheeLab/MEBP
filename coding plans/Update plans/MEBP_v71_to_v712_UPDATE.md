# MEBP v7.1 → v7.1.2 Update Plan

## Objective
Critical bug fixes for XY jog position accuracy, simulator performance, and protocol parameter loading.

## Bug Fixes

### BF-1 — XY jog position inconsistent with requested step (CRITICAL)
**Files:** `gui/pages/jog_control.py`, `SupportClasses/StageController.py`
**Symptom:** XY jog movements did not match the requested step size.
**Root cause:** Absolute moves were computed from cached position, which could be stale. Rounding errors accumulated over multiple jogs.
**Fix:** Replaced absolute moves from cached position with relative moves. StageController sends the exact requested delta.
**Status:** `[x]` done

### BF-2 — XYStageSimulator too slow for microstep-scale positions (CRITICAL)
**Files:** `SupportClasses/XYStageSimulator.py`, `config/controllers/proscan_iii.json`
**Symptom:** Simulator could not keep up with microstep-scale position changes, causing test timeouts and unrealistic behavior.
**Root cause:** Default simulator parameters (max_speed=100) were orders of magnitude too low for microstep-scale coordinates (positions in tens of thousands).
**Fix:** Complete rewrite of XYStageSimulator with updated defaults (max_speed 50,000 microsteps/s) and added settling threshold. Added simulator speed parameters to proscan_iii.json.
**Status:** `[x]` done

### BF-3 — Protocol parameters not loaded in simulation mode (HIGH)
**Files:** `SupportClasses/XYStage.py`
**Symptom:** Simulation mode ignored controller protocol JSON, using hardcoded defaults that didn't match the configured controller.
**Root cause:** Protocol loading was gated behind a hardware-mode check. Simulation mode skipped `_load_protocol()` entirely.
**Fix:** Changed to always load protocol when `controller_json` is provided, regardless of simulation mode. Protocol parameters (microsteps_per_micron, speed limits, etc.) now available in both modes.
**Status:** `[x]` done

### BF-4 — Position truncation bias (LOW)
**Files:** `SupportClasses/XYStage.py`
**Symptom:** Systematic negative bias in position commands over many moves.
**Root cause:** `int()` truncation instead of `round()` for position values. Positive fractional values always truncated down.
**Fix:** Replaced `int()` with `round()` for all position command values.
**Status:** `[x]` done

### BF-5 — Safety proximity check margin undocumented (LOW)
**Files:** `SupportClasses/StageController.py`
**Symptom:** Safety proximity check used a magic number (500 steps) with no explanation of physical meaning.
**Fix:** Added documentation clarifying 500 steps ≈ 50µm at 10 steps/µm default scale factor.
**Status:** `[x]` done

### BF-6 — Jog speed handler attribute mismatch
**Files:** `gui/pages/jog_control.py`
**Symptom:** Jog speed changes did not propagate to the jog handler.
**Root cause:** Code referenced `_xy_jog_handler` but the actual attribute was `xy_jog`.
**Fix:** Corrected all references to use the correct `xy_jog` attribute name.
**Status:** `[x]` done

## Implementation Steps

- [x] BF-1: Replace absolute jog moves with relative moves
- [x] BF-2: Rewrite XYStageSimulator with realistic parameters
- [x] BF-3: Load protocol parameters in simulation mode
- [x] BF-4: Fix int() truncation to round()
- [x] BF-5: Document safety proximity margin
- [x] BF-6: Fix jog handler attribute name

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/jog_control.py` | BF-1: relative moves; BF-6: attribute name fix |
| `SupportClasses/StageController.py` | BF-1: relative move API; BF-5: proximity documentation |
| `SupportClasses/XYStageSimulator.py` | BF-2: complete rewrite with realistic parameters |
| `SupportClasses/XYStage.py` | BF-3: always load protocol; BF-4: round() fix |
| `config/controllers/proscan_iii.json` | BF-2: added simulator speed parameters |
| `tests/test_workflow.py` | **New** — test suite for jog workflow verification |

## Testing Notes

1. Jog XY in all 4 directions with various step sizes — verify position delta matches requested step
2. Run simulator tests — verify XYStageSimulator reaches target positions within settling threshold
3. Launch in simulation mode with ProScan III protocol — verify protocol parameters are loaded
4. Verify no position drift over 20+ consecutive jog operations

## Issues & Decisions

- **Relative vs absolute moves**: Chose relative moves to eliminate accumulated rounding errors. Each jog is now independent — no dependency on cached position state.
- **Simulator performance**: Increased default max_speed from 100 to 50,000 to match microstep-scale coordinate ranges. Added settling threshold to prevent infinite convergence loops.
