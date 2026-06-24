# MEBP v7.5.x — Multi-object print seam fix

## Objective

After the open-loop-pacing fixes, one Quick Print symptom remained: when
printing a saved multi-object file (e.g. `PIAR.json` / `Spirals.json` — six
spirals at different offsets), **toward the end of each spiral the needle
jumped to a new location, drove through the already-printed material, then
continued**. Confirmed from the execution log
`logs/prints/print_20260615_175132_Quick Print _ _ PIAR  _6_ _ C4.jsonl`.

Root cause = audit finding **GEO-3 / FRM-7b**: Quick Print concatenated all
objects' XY points into **one** `PRINT_PATH`. The seams between objects
(measured 1.9–4.3 mm in that log, segments #309/#618/#927/#1545) were
therefore executed as **extruded print moves at print height** — no lift, no
travel — so the needle dragged across printed material laying down a line.

## Root cause detail (from the log + PIAR.json layout)

`PIAR.json` = 6 spiral objects laid out on a **hexagonal ring** (centres at
2 mm radius, 0°/60°/…/300°), each `max_radius=2.0` / `64 pts/turn` ≈ **309
points**, generated **centre → rim**. The log confirms:

- The 4 large gaps are in the **commanded geometry** — recomputing the gap
  straight from the logged XY targets reproduces 4.288/3.629/2.100/1.898 mm
  exactly. **Not** a timing/queue artifact (if it were, the commanded path
  would be smooth and only the *actual* position would lag).
- They land at exactly the 5 object boundaries (309/618/927/1236/1545; the
  1236 seam happened to be small). Each 309-block is a clean centre→rim
  spiral (r 0.07 → ~2.0 mm, mean gap 0.1 mm); the only big gap in a block is
  its **last** point → the seam to the next spiral's centre.
- "~every 300 segments" = the deterministic 309-point spiral length, not a
  buildup. Because the 6 spirals **overlap** (2 mm centres, 2 mm radius), the
  rim→centre seam crosses already-printed material — which looks like a jump
  "mid-print" even though it is at each spiral object's end.

A **separate, real** finding (the timing intuition was not wrong, just about a
different symptom): in that pre-fix run `drift_s` grew 0.1 → 27 s and the
actual-vs-commanded `lag_um` ran a ~2.35 mm median — the open-loop pacing +
uncalibrated SMS speed (audit MOT-1/2/3). That causes smooth lag/corner-
cutting, not the discrete seams, and is tracked separately
(`PRINT_PIPELINE_AUDIT_V75x.md` P0).

## Fix

One `PRINT_PATH` per object with travel between them, **no extrusion across
the seam** (only `PRINT_PATH` extrudes; `MOVE_XY` does not):

- **First object in a well** → full **TRAVEL_UP → MOVE_XY (full retract) →
  MOVE_Z** approach from travel height (inter-well / from job start).
- **Subsequent objects in the same well** → a **small confirmed hop**: the
  seam `MOVE_XY` carries a `hop_z` param so `_retract_for_travel` raises only
  to `hop_z = print_z + z_up_sign · intra_well_hop_z_mm` (default **1 mm**,
  configurable) instead of the full travel height. Routed through
  `StageController.ensure_retracted_to` → polarity-safe (raise-only, never
  descends; verified `hop_z` is genuinely "up" on ME3B z_up_sign=-1:
  print_z −16.17 → hop_z −17.17), blocking (Z confirmed before XY), and
  floored by the plate-insert clearance. Just clears thin printed material
  without a slow ~21 mm retract + re-approach.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PrintManager.py` | `build_well_plate_job` gains optional `path_segments: list[list[(x,y)]]`. Per-well body loops `segments = [s for s in (path_segments or [path_points]) if s]`; first object = full `TRAVEL_UP → MOVE_XY → [dwell] → MOVE_Z → [prime] → PRINT_PATH → [retract]`, subsequent objects = small hop (`MOVE_XY{hop_z}` → MOVE_Z → …). New `PrintSettings.intra_well_hop_z_mm` (default 1.0). `_retract_for_travel(context, target_z=None)` accepts a custom (small) retract target; the `MOVE_XY` handler passes `hop_z` through it. `path_segments=None` (every existing caller) yields one segment == `path_points` → byte-identical plan (back-compat verified by test). |
| `gui/pages/workflows/quick_print_workflow.py` | New `_path_segments_for_selection()` (one sub-path per object); `_path_points_for_selection()` now flattens it (kept for the printable-path guard); `_on_print` passes `path_segments=` to `build_well_plate_job`. (Already stamps `settings.z_up_sign` from `controller.print_z_dir()`, so the hop direction is polarity-correct.) |
| `tests/test_v75x_multi_object_print_seam.py` | **NEW** — 11 tests (seam split, back-compat, small-hop height/direction/setting, hop execution wiring). |

## Status

- [x] `build_well_plate_job` `path_segments` + per-object travel prologue
- [x] Small intra-well hop (`hop_z` / `intra_well_hop_z_mm`, default 1 mm) for
      objects after the first; full retract only on the first object / between
      wells / at home
- [x] Quick Print page emits per-object segments
- [x] Back-compat: `path_segments=None` reproduces the legacy plan
- [x] Tests (11) + regression (print-z / hybrid / z-retract / quick-print / logging suites = 77 green)
- [x] End-to-end rebuild of the real `PIAR.json`: 6 objects → 6 PRINT_PATHs,
      worst in-PRINT_PATH segment **4.288 → 0.197 mm**; object 1 = full
      TRAVEL_UP approach, objects 2–6 = 1 mm hops (verified up-direction on
      ME3B z_up_sign=-1: print_z −16.17 → hop_z −17.17)
- [ ] Real-hardware confirmation (reprint PIAR with logging on; verify the
      `path_segment` events no longer contain >0.5 mm extruded jumps and each
      seam shows as `xy_cmd context=hop` + a ~1 mm `z_move`)

## Testing Notes

`python -m unittest tests.test_v75x_multi_object_print_seam` — 8/8.
Regression suites (`test_v75x_print_z_reference_vector`,
`test_hybrid_execution`, `test_v75x_z_retract_before_xy_travel`,
`test_v75x_quick_print_workflow`, `test_v75x_print_execution_logging`) green.

## Issues & Decisions

- **Scope**: only the seam (inter-object travel). The residual ~0.197 mm
  intra-spiral max spacing is the constant-angle sampling (audit **GEO-2**);
  it sits under the executor's 0.25 mm floor-escape so it paces at the 0.05 s
  floor — already "mostly resolved" per the user and a separate P1 item
  (arc-length resampling of `generate_spiral`).
- **Prime/retract**: emitted per segment. Quick Print defaults to 0 µL, so no
  behavioral change there; for configured prime/retract this now correctly
  brackets each object (reduces oozing during the seam travel).
- **Back-compat**: `path_segments` is keyword-only-in-practice (all callers
  use kwargs) and defaults to `None`; the standard Print Setup / plan paths
  are untouched.
