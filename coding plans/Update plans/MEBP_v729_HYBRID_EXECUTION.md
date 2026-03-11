# MEBP v7.2.9 — Hybrid Print Execution Model

## Objective

Replace the all-in-one trajectory approach with a hybrid execution model:
- **Service steps** (travel, waste, wash, ink load) → blocking absolute moves with completion confirmation
- **Print steps** (coordinated XY+pump within a well) → time-parameterized trajectory playback

## Problem

Everything was pre-compiled into one time-parameterized trajectory. The executor sleeps to each waypoint time and fires commands. This breaks because:
1. XY `G` commands are fire-and-forget — stage hasn't arrived when next command fires
2. Timing assumes constant speed — real stages have accel/decel/serial latency
3. Simulator blocks on moves, throwing off trajectory timing
4. Any delay cascades — executor rushes to catch up, shortening actual print operations

## Design

```
Old:  Plan → TrajectoryPlanner → [all waypoints] → TrajectoryExecutor → hardware

New:  Plan → HybridPlanExecutor
               ├── service steps → DirectCommandExecutor → blocking absolute moves
               └── PRINT steps  → per-well loop:
                     1. DirectCommandExecutor: travel_to_well (Z up → XY → Z down)
                     2. TrajectoryPlanner.generate_inwell_print() → in-well waypoints
                     3. TrajectoryExecutor: play back print (prime → XY+pump → retract)
                     4. DirectCommandExecutor: raise_from_well (Z up)
```

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | Add `wait_for_z_arrival()` |
| `SupportClasses/PrintManager.py` | Add `DirectCommandExecutor`, `HybridPlanExecutor`, extend `PrintJob` |
| `SupportClasses/PrintTrajectoryPlanner.py` | Add `generate_inwell_print()` — in-well only, no travel |
| `gui/app.py` | Add `"hybrid"` execution mode with `import threading` |
| `gui/pages/print_setup.py` | Attach plate/path_points/well_setup/hw_config to PrintJob |

## Implementation Steps

- [x] Create update plan (this document)
- [x] Add `wait_for_z_arrival()` to StageController
- [x] Add `DirectCommandExecutor` class (blocking move_xy, move_z, move_pump, travel_to_well, raise_from_well)
- [x] Add `generate_inwell_print()` to TrajectoryPlanner (replaces generate_print_only)
- [x] Add `HybridPlanExecutor` class
- [x] Extend `PrintJob` dataclass with plan_of_action, plate, path_points, well_setup, hw_config
- [x] Wire hybrid mode in `app.py` + `print_setup.py`
- [x] Revert previous band-aid fixes (settle dwell, wait_for_xy hacks in TrajectoryExecutor)
- [x] Fix `_execute_print_step` — travel/Z via DirectCommandExecutor, trajectory for in-well only
- [x] Fix `threading` import in app.py hybrid block
- [x] Fix fallback exec_mode reset when hybrid fails
- [x] Wire `estimate_time()` → PrintJob.estimated_duration_s → print_monitor progress
- [ ] End-to-end testing with simulator

## Step Type Dispatch

| PlanStepType | Execution Mode | Method |
|---|---|---|
| PRINT | Hybrid (direct + trajectory) | `_execute_print_step()` — per-well: direct travel → traj print → direct raise |
| WASTE | Direct | move to waste → eject → retract |
| WASH | Direct | move to wash → dwell → retract |
| REFILL_BUFFER | Direct | move to buffer → aspirate → retract |
| LOAD_INK / GATHER_INK | Direct | move to ink → aspirate → retract |
| MOVE_SAFE_Z | Direct | `move_z(safe_z)` |
| TRAVEL_XY | Direct | `move_xy(well_pos)` |
| INK_SWAP | Direct | sequence of waste+wash+buffer+ink |
| FINAL_CLEANUP | Direct | waste + wash sequence |
| RETURN_HOME | Direct | raise Z → move XY to (0,0) |

## Key Design Decisions

1. **TrajectoryExecutor unchanged** — still plays back Waypoint lists for print paths
2. **Blocking moves** — DirectCommandExecutor wraps `move + wait_for_arrival` pairs
3. **In-well trajectories only** — `generate_inwell_print()` generates waypoints for one well at a time: only prime → print path → retract. No XY travel, no Z travel. Starts at print_z_height.
4. **All travel is blocking** — Every Z raise, XY move, and Z lower between wells uses `DirectCommandExecutor` which calls `wait_for_xy_arrival()` / `wait_for_z_arrival()` before proceeding
5. **Monitor compatibility** — full trajectory still generated for preview; hybrid executor reports step-level progress during service, waypoint-level during print

## Issues & Decisions Log

- Band-aid fixes (settle dwell, XY wait + time-base correction) reverted — hybrid model makes them unnecessary
- `generate_print_only()` replaced by `generate_inwell_print()` — the old method still called `_travel_to_well()` which generated trajectory waypoints for travel, defeating the purpose
- `_execute_print_step` restructured to loop over wells itself, using DirectCommandExecutor for travel between each well
