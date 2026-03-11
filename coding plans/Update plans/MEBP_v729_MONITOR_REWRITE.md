# MEBP v7.2.9 — Print Monitor XY Detail View Rewrite

## Objective

Rewrite the XY Detail trajectory visualization in `print_monitor.py` to correctly display:
- Where the needle IS (crosshair)
- Where the needle HAS BEEN (completed waypoints + actual trail)
- Where the needle is GOING (upcoming waypoints)

## Root Causes Fixed

| Issue | Before | After |
|-------|--------|-------|
| Well circle at (0,0) | Circle drawn at origin; waypoints at plate-absolute coords = circle always off-screen | Circle drawn at computed well center from waypoint metadata |
| Filtered waypoints | Only non-travel waypoints stored; spatial gaps between wells broke matching | ALL waypoints stored with `is_travel` + `well` metadata |
| Narrow search window | Searched current_idx ±2 to +50; well jumps missed | Two-phase: local ±10/+200, then global if >2mm away |
| No actual trail | Only planned dots shown | `deque(maxlen=2000)` of polled positions drawn as purple line |
| No well-aware view | View centered on needle with fixed radius | Auto-centers on current well's waypoint bounding box |
| No well transition | View didn't update when printing moved to next well | Detected from both position matching and progress messages |

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/print_monitor.py` | Full rewrite of `XYDetailView` class, `_load_job_waypoints()`, `on_status_update()`, `on_print_progress()`, new `_find_nearest_waypoint()` |

## Implementation Steps

- [x] Rewrite `XYDetailView` class
  - New data model: `_waypoints`, `_wp_is_travel`, `_wp_well` parallel lists
  - `_needle_trail: deque` for actual position trail
  - `_well_centers: dict` for per-well center cache
  - `set_trajectory(waypoints, is_travel, wells)` replaces `set_waypoints`
  - `focus_well(name)` centers view on a well's waypoint bounds
  - `_auto_fit_view()` fits all waypoints when no well info
  - `_to_px()` uses view center + radius (not needle position)
  - `paintEvent()`: well circle at well center, travel segments dimmed, trail drawn
- [x] Rewrite `_load_job_waypoints()` — pass ALL waypoints with metadata to `set_trajectory()`
- [x] Rewrite `on_status_update()` — calls `_find_nearest_waypoint()`, detects well transitions
- [x] New `_find_nearest_waypoint()` — two-phase (local + global) search, forward-only progress
- [x] Update `on_print_progress()` — calls `focus_well()` on well change from message parsing
- [x] Update XY detail legend — added "Trail" entry

## Testing Notes

1. Run a multi-well print job (2+ print wells)
2. Verify XY detail shows:
   - Well boundary circle centered correctly on the current well
   - Green filled dots + solid green lines for completed waypoints
   - Yellow dot + dashed yellow lines for upcoming waypoints
   - Purple line showing actual needle path from polling
   - Pink crosshair at current needle position
3. Verify view re-centers when printing transitions to next well
4. Verify waypoint completion advances correctly through the trajectory
5. Test single-well prints (view should auto-fit to that well's bounds)
6. Test command-based jobs (fallback: no metadata, works like before)

## Issues & Decisions

- **Trail color**: Chose purple (`#cba6f7`) to distinguish from green completed path and pink needle
- **View padding**: 2mm padding around waypoint bounding box for comfortable viewing
- **Global search threshold**: 4.0 mm² (= 2mm distance) before triggering full waypoint scan
- **Trail maxlen**: 2000 entries at ~300ms polling = ~10 minutes of history
- **Forward-only progress**: `_find_nearest_waypoint` returns `max(best, current)` to prevent regression from noise
