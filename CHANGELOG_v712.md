# MEBP v7.1.2 — Bug Fix Release

## Summary

This release fixes 5 bugs identified through systematic workflow testing of the v7.1 codebase. The bugs span the jog control, simulator physics, hardware configuration propagation, and safety limit layers.

---

## Bug Fixes

### BUG-1: XY Jog Position Inconsistent with Requested Step (CRITICAL)

**Symptom**: When jogging the XY stage, the actual position change didn't match the requested step size. Multiple rapid jog commands could "lose" steps.

**Root Cause**: The jog code computed absolute targets from *cached* position data:
```python
# OLD (buggy)
pos = controller.get_xy_position(cached=True)  # Up to 300ms stale!
target = (pos - zero) + dx * step_steps
controller.move_xy_absolute(target, from_zero_ref=True)
```
If the cached position was stale (the poller runs at 300ms intervals), the computed target would be based on an outdated position, causing the actual movement to differ from the requested step.

**Fix**: Changed to relative moves that don't depend on cached position:
```python
# NEW (correct)
controller.move_xy_relative(dx * step_steps, dy * step_steps)
```

Added `StageController.move_xy_relative()` method that sends `GR dx,dy` relative move commands directly to the stage. Safety limits are still enforced by projecting the cached position + delta.

**Files Changed**:
- `gui/pages/jog_control.py` — replaced `_jog_xy()` method
- `SupportClasses/StageController.py` — added `move_xy_relative()`

---

### BUG-2: XYStageSimulator Too Slow for Microstep-Scale Positions (CRITICAL)

**Symptom**: In simulation mode, jog commands appeared to "not work" or were extremely sluggish. A 50µm jog (500 microsteps) took ~5 seconds instead of <0.1s.

**Root Cause**: The simulator was initialized with `max_speed=100` and `acceleration_rate=100`, but positions are in microsteps (1 µm = 10 microsteps for ProScan III). A 500-microstep move with max_speed=100 takes:
- Proportional controller: `desired_v = min(100, 2.0 * 500) = 100` units/s
- Time to reach target: ~500/100 = 5 seconds

Real ProScan III stages operate at up to 50,000+ microsteps/s.

**Fix**: Updated simulator defaults to microstep-scale values:
- `max_speed`: 100 → 50,000 microsteps/s
- `acceleration_rate`: 100 → 100,000 microsteps/s²
- `kp`: 2.0 → 10.0 (proportional gain for faster settling)
- Added `settling_threshold` for position snap-to-target
- Added `configure_from_protocol()` method for dynamic configuration

**Files Changed**:
- `SupportClasses/XYStageSimulator.py` — complete rewrite of defaults

---

### BUG-3: Protocol Parameters Not Loaded in Simulation Mode (HIGH)

**Symptom**: The `microsteps_per_micron` conversion factor couldn't be read from the controller protocol JSON when running in simulation mode, forcing fallback to defaults. Hardware catalog parameters (speed, acceleration) weren't available to the simulator.

**Root Cause**: Protocol loading was conditionally skipped in simulation mode:
```python
# OLD (buggy)
if not simulate:
    self._load_protocol(controller_json)
    self._apply_protocol_parameters()
```

**Fix**: Load protocol always when a controller_json path is provided:
```python
# NEW (correct)
if controller_json is not None:
    self._load_protocol(controller_json)
    self._apply_protocol_parameters()
elif not simulate:
    self._load_protocol(controller_json)
    self._apply_protocol_parameters()
```

Also added code to configure the simulator with protocol-derived speed parameters via the new `configure_from_protocol()` method.

**Files Changed**:
- `SupportClasses/XYStage.py` — conditional protocol loading
- `config/controllers/proscan_iii.json` — added `max_speed`, `acceleration`, `microsteps_per_micron` to parameters

---

### BUG-4: Position Truncation with int() Instead of round() (LOW)

**Symptom**: Position commands always truncated toward zero due to `int()` conversion. A target of 499.7 microsteps would be sent as 499 instead of 500.

**Root Cause**: `move_stage_to_position` and `move_stage_relative` used `int(x)` for position values. Python's `int()` truncates toward zero, introducing a systematic negative bias.

**Fix**: Changed `int()` to `round()` for proper nearest-integer rounding.

**Files Changed**:
- `SupportClasses/XYStage.py` — `move_stage_to_position()`, `move_stage_relative()`

---

### BUG-5: Safety Proximity Check Uses Undocumented Step Margin (LOW)

**Symptom**: The XYJogHandler slowed down jog speed near safety limits using a hardcoded `margin=500` steps. The meaning of this margin wasn't clear, and it wasn't documented in terms of physical units.

**Root Cause**: The margin was a magic number with no documentation about its physical meaning (500 steps ≈ 50µm at 10 steps/µm).

**Fix**: Added documentation and comments clarifying the margin's physical meaning. Also noted that the proximity check should ideally use zero-relative coordinates.

**Files Changed**:
- `SupportClasses/StageController.py` — added documentation comments

---

### BONUS: Jog Speed Handler Attribute Mismatch

**Symptom**: Jog speed slider changes didn't propagate to the actual jog handler.

**Root Cause**: The jog control page referenced `controller._xy_jog_handler` but the actual attribute is `controller.xy_jog`.

**Fix**: Updated attribute references in speed callbacks.

**Files Changed**:
- `gui/pages/jog_control.py` — speed handler callbacks

---

## Delivery Files

| File | Action | Bug |
|------|--------|-----|
| `SupportClasses/XYStageSimulator.py` | **Replace** entire file | BUG-2 |
| `gui/pages/jog_control.py` | **Replace** entire file | BUG-1, BONUS |
| `apply_v712_fixes.py` | **Run** to patch remaining files | BUG-1,3,4,5 |
| `tests/test_workflow.py` | **Add** new test suite | All |

## How to Apply

```bash
# 1. Copy the replacement files
cp delivery/SupportClasses/XYStageSimulator.py SupportClasses/
cp delivery/gui/pages/jog_control.py gui/pages/
cp delivery/tests/ tests/ -r

# 2. Run the patch script for remaining fixes
python apply_v712_fixes.py

# 3. Run the test suite
python -m pytest tests/test_workflow.py -v
```
