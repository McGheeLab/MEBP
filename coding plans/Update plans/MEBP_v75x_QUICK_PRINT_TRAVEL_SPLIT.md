# MEBP v7.5.x — Quick Print travel-split + quick-move pressure relief

## Objective
`changes_needed.md` item 5: "when quick printing the pump isn't stopping between
quick moves and the z doesn't seem to be retracting. I think it doesn't know about
the quick moves to the new location." Operator chose **keep the path-split AND wire
the relief** (best of both).

## Root cause
`gui/pages/workflows/quick_print_workflow.py::_obj_dict_to_path_points` flattened
each object's trajectory to **XY only**, dropping the per-point Z (travel lifts) and
pump (extrude-on/off) columns; `_path_segments_for_selection` yielded one segment
per OBJECT (not per travel move); `PrintManager._execute_print_path` extrudes a
**uniform** flow at a **fixed print Z** for every segment. So a pen-up travel move
inside an object (multi-shape sketch / CSV with travel) became an extruding print
move with the needle down.

## Files Modified
- `gui/pages/workflows/quick_print_workflow.py` (ONLY file changed)
  - Refactor `_obj_dict_to_path_points` → new `_obj_dict_to_trajectory` (returns the
    full Nx7) + kept the XY-only helper for back-compat.
  - New `@staticmethod _travel_mask(arr)` — per-segment travel flag: pump columns not
    advancing (primary, layer-agnostic) OR the segment touching the top-Z band
    (fallback when no pump info); returns `None` for a path flat in BOTH (→ legacy
    single segment, preserves plain circle/meander).
  - New `_obj_dict_to_subpaths` — splits each object into contiguous print sub-paths
    at travel moves; `_path_segments_for_selection` flattens object→sub-paths.

## How the fix works
The existing seam machinery (`PrintManager.build_well_plate_job(path_segments=…)`)
already inserts a lift→hop→lower→prime between segments and only `PRINT_PATH`
extrudes; splitting each object at its travel moves makes the pump stop and the
needle retract across every travel. The "wire relief" half is satisfied by the
**existing** `PrintManager._print_pump_suckback("quick_move")` already called in the
MOVE_XY hop handler (re-primed by the next segment's prime) — it fires automatically
on every hop the split creates. **No new relief code was added** (an earlier attempt
to emit suck-back/re-prime DISPENSE commands here was reverted — it duplicated
`_print_pump_suckback` → double suck-back).

## Implementation Steps
- [x] trajectory builder + `_travel_mask` + `_obj_dict_to_subpaths`
- [x] `_path_segments_for_selection` uses sub-paths (→ existing relief fires on hops)
- [x] Tests `tests/test_v75x_quick_print_travel_split.py` (7)

## Testing
`tests/test_v75x_quick_print_travel_split.py` — `_travel_mask` (flat→1 segment,
2-shape→2, 2-shape×2-layer→4, pump-advance classification); the split's job has one
PRINT_PATH per sub-path + an inter-object hop (MOVE_XY with hop_z) between them, with
the first object on a full TRAVEL_UP. Multi-object seam + quick-print pick&place +
pump-relief suites stay green.

## Issues & Decisions
- Pump-advance is the primary travel signal (robust + layer-agnostic); top-Z band is
  a fallback so a CSV with monotonic pump still splits on lifts. Flat-in-both → legacy
  single segment (no regression for plain shapes).
- **Fix (2026-06-30) — reversed-Z-polarity sketch files wouldn't load in Quick Print.**
  Operator: "no longer able to load in any print I made from the sketch — no preview,
  Print does nothing; older non-sketch files still work." Root cause: `_travel_mask`
  OR'd the top-Z-band heuristic on TOP of the pump signal (`mask |= top` after
  `mask |= pump-flat`). Many saved sketch/CSV files lift to a numerically **LOWER** Z
  (the print plane sits ABOVE the travel/lift band — the opposite sign of what the
  current compiler emits: e.g. A_good_1 print z=12.29 / lift z=10.09; Smile_1
  −25.88/−28.08; Box_1 5.86/3.66). For those the top-Z band flagged 88–98% of points
  (the entire print plane) as travel → `_obj_dict_to_subpaths` dropped every sub-path →
  `_path_segments_for_selection` returned `[]` → blank well preview AND `_on_print`'s
  `if not path_points:` guard set a status line and returned (the "Print does nothing"
  symptom). Newer sketch files (Sketch_1/2/3, print z=0 / lift z=2.2) happened to match
  the top-band assumption so they worked, which masked the bug. **Fix:** the z-band is
  now a true fallback used ONLY when there is no pump info (`if pump_info … elif z_info`)
  — the pump column is polarity-independent and present in all affected files, so it
  wins. Restores every reversed-Z file (A_good_1 → 3 sub-paths, Smile_1 → 10, Box_1 → 1,
  Circles_1 → 8, Bold_A_1 → 10, Smile_new_1 → 10, …) with no change to the working ones.
  Only `_travel_mask` changed; test `test_reversed_z_polarity_splits_by_pump_not_top_zband`
  added (8 in file, all green).
- **Fix (2026-07-01) — connected line segments printed discontinuously (needle
  lifted at every shared node).** Operator: "if lines are connected by the same
  node the print should be continuous; instead it kept lifting off and restarting
  the print for each small line segment." Root cause: the sketch compiler already
  WELDS connected paths (shapes sharing a node within `weld_tol`) into one
  continuous bead with no pen-up (`SketchTrajectory.compile_to_trajectory`, the
  `welded` branch) — but at an **exactly** shared node it emits a zero-distance
  coincident waypoint whose pump column is flat (verified: an "L" drawn as two
  connected lines yields waypoints 10 & 11 both at `(5.0, 0.0, 0.2)`, `p1=5.0`).
  `_travel_mask` flagged **any** pump-flat segment as travel (`dp <= 1e-9`), so
  the join at each shared node was mis-read as a "quick move to a new location"
  and `_obj_dict_to_subpaths` split the run there → `build_well_plate_job`
  inserted a lift→hop→lower→prime → the needle lifted mid-stroke and restarted.
  Near-coincident welds (gap `0 < d ≤ weld_tol`) never tripped this — the compiler
  keeps `printing=True` for them so the pump advances (`dp > 0`); only the
  **exact** shared node (`d == 0`, `dp == 0`) did. **Fix:** a travel now requires
  the pump to be flat **AND the needle to actually move** — the pump branch adds
  `& (seg_dist > 1e-6)` (3D per-segment distance from cols x,y,z). Zero-distance
  coincident/weld nodes (and any duplicate CSV points) stay continuous; genuine
  travels (lift/hop/lower all have motion) still split. Only `_travel_mask`
  changed. Tests: `test_connected_shapes_print_continuously` (L + 4-line square →
  1 run), `test_disconnected_shapes_still_split` (separate lines → 2),
  `test_zero_distance_pump_flat_segment_is_not_travel` (11 in file, all green;
  seam/pick&place/relief/workflow suites 87 green). **Needs real-HW verification
  on ME3B V1** (Quick Print a connected multi-line sketch → one continuous bead,
  no lift at shared nodes).
- The quick-move suck-back was ALREADY implemented in the user's WIP
  (`_print_pump_suckback`, gated by `pump_relief_on_quick_move`, called at the hop);
  the split is what generates the hops, so the two compose with no new code.
- Deferred: making `_execute_print_path` honour the per-point pump column for variable
  in-path extrusion — the split fully resolves the reported bug with existing infra.

## Needs real-HW verification on ME3B V1
Print a multi-shape sketch via Quick Print → needle lifts + pump stops between shapes;
`logs/prints/*.jsonl` shows one PRINT_PATH per shape with hops (and suck-back/re-prime
when relief-on-quick-move is enabled).
