# MEBP v7.5.x — Plate-bottom Z datum + universal "don't punch through" floor

## Objective

Three user-requested changes, all centered on print Z safety/usability:

1. **Name a sketch print** — the Print Builder → Sketch tool can name the print
   it bakes (instead of the hard-coded `Sketch_1`).
2. **Print Z relative to the plate bottom (not absolute)** — at **all** print
   entry points the user-facing print height is expressed as *height above the
   plate bottom* (mm, ≥ 0), resolved at print time against the calibrated
   `plate_bottom_z`. The internal motion frame is unchanged.
3. **Universal "don't punch through the plate" guard** — during **all** printing
   activities the needle is hard-clamped so it can never go deeper than the
   calibrated plate bottom, regardless of the configured print Z, per-object
   offset, or per-layer offset. Plus an **early warning** if the configured Z
   would punch through (the user may proceed; the clamp is the backstop).

User decisions (asked up front):
- Scope of relative-Z: **all print entry points** (Sketch + Quick Print + the
  standard Print Setup).
- Floor behavior: **warn early + hard-clamp** during motion.

## Background / why this is subtle (ME3B V1 inverted Z)

On ME3B V1 the needle **descends as the raw Marlin Z counter increases**
(`steps_per_mm["Z"]=+5255`, `StageController.ZDIR=-1`). So *larger* zero-ref /
raw Z = needle *lower* (closer to / into the plate). The calibrated
`plate_bottom_z` (well floor, ≈ −25.68 zero-ref mm) is the **deepest** safe
point; "punching through" means a Z numerically **greater** than
`plate_bottom_z`.

Findings that shaped the design:
- The **only** Z safety today is `SafetyLimits.clamp_z` against the device
  envelope (`z_min=-60 … z_max=0`). It knows nothing about the plate bottom, so
  a print Z / offset deeper than the well floor is **not** prevented.
- In the standard Print Setup flow a csv_import object's trajectory **Z column
  is discarded** (only XY is used); the real print Z comes from
  `settings.print_z_height` (+ per-layer height). And
  `_compute_auto_settings` only applies `safe_z`/`top_z` when **positive**
  (`> 0` / `>= 0`) — ME3B's calibrated values are negative, so those guards
  silently fail and `print_z_height` stays at the `0.1` default. Making print Z
  plate-bottom-relative is therefore the correct, frame-agnostic fix.
- Every print path's Z ultimately flows through
  `StageController.move_z_absolute` / `move_z_relative`, which is the single
  chokepoint for the universal clamp.

## Design

### Polarity-general helpers (one home: `StageController`)

```
ZDIR = -1.0  # already defined for this build

def plate_relative_to_zref(plate_bottom_zref, height_above_bottom, zdir=ZDIR):
    return plate_bottom_zref + zdir * height_above_bottom

def zref_to_plate_relative(plate_bottom_zref, z_zref, zdir=ZDIR):
    return zdir * (z_zref - plate_bottom_zref)
```

- `height_above_bottom ≥ 0` is always "above/away from the plate" for **either**
  polarity. `< 0` = punch-through.
- Clamp rule (polarity-general, applied to the **raw** destination): if
  `ZDIR * (raw - plate_bottom_raw) < 0` → set `raw = plate_bottom_raw`. Reduces
  to "cap deeper" for ZDIR<0 and "floor" for ZDIR>0. No-op for ZDIR=+1 machines
  unless the floor is breached, so other builds are unaffected.

### Controller state + API
- `_plate_bottom_z_zref: float | None` — calibrated plate bottom (zero-ref mm),
  pushed from the calibration page (`get_z_references()["plate_bottom_z"]`).
- `_print_floor_active: bool` — armed only during print execution.
- `set_plate_bottom_z(z|None)`, `get_plate_bottom_z()`,
  `set_print_floor_active(bool)`, `print_height_to_zref(h)`,
  `zref_to_print_height(z)`, `print_floor_violation(z_zref)`,
  `_apply_print_floor_raw(raw)`.
- `move_z_absolute`: after `clamp_z`, apply `_apply_print_floor_raw`.
- `move_z_relative`: inside the existing `not bypass_safety` block, run the
  clamped destination through `_apply_print_floor_raw`.

The floor is **gated to print execution** so it never interferes with
calibration (which must reach the floor to capture it) or manual jogging.

### Arming (covers "all printing activities")
`PrintManager.start()` / `resume_from_saved()` → `set_print_floor_active(True)`;
`_execute_loop` `finally` → `False`. Every print path (Quick Print, Print Setup,
workflows) runs through `PrintManager.start → _execute_loop`, and every Z move
runs through the controller, so the guard is universal.

### Relative-Z input at each entry point
- **Quick Print**: "Print Z" → "Height above bottom" (mm, ≥0). `_build_settings`
  converts via `controller.print_height_to_zref`. Early-warn in `_on_print`.
- **Print Setup**: new "Print height" (above plate bottom) spin in the layer
  group. `_get_settings()` (covers both `_build_current_job` and the generate
  path) sets `print_z_height = plate_relative_to_zref(plate_bottom_z, height)`
  reading `plate_bottom_z` from the calibration section. If a selected object
  carries `z_above_plate_bottom_mm` metadata (a sketch), that overrides the
  spin. Early-warn in `_send_to_monitor`.
- **Sketch**: Name field (QLineEdit) + "Z start" relabeled to "Print height
  (above plate bottom)". `set_z_references` receives `plate_bottom_z`. On send:
  pass `base_name`/`object_name`=name and
  `extra_params={"z_above_plate_bottom_mm", "z_datum":"plate_bottom", ...}`; bake
  the trajectory Z into zero-ref via `plate_relative_to_zref` when the plate
  bottom is known.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | Module helpers + datum state + API + clamp in `move_z_absolute`/`move_z_relative` |
| `SupportClasses/PrintManager.py` | Arm/disarm print floor (start/resume/`_execute_loop` finally) |
| `gui/app.py` | Push `plate_bottom_z` to controller; forward `set_z_references` to Print Builder |
| `gui/pages/workflows/quick_print_workflow.py` | Height-above-bottom input + conversion + warn |
| `gui/pages/print_setup_legacy.py` | Print-height-above-bottom spin + conversion + object-metadata override + warn |
| `gui/pages/print_builder.py` | `set_z_references` forwarder to sub-pages |
| `gui/pages/print_builder_sketch.py` | Name field + relative-Z relabel + `set_z_references` + send metadata |
| `SupportClasses/SketchTrajectory.py` | Doc: `z_start_mm` now means height above plate bottom |
| `tests/test_v75x_print_z_plate_bottom.py` | New tests |

## Implementation Steps

- [x] 1. StageController: helpers + datum + clamp
- [x] 2. PrintManager: arm/disarm floor
- [x] 3. app.py: push plate bottom + z_references
- [x] 4. Quick Print: relative input + warn
- [x] 5. Print Setup: relative print-Z + metadata + warn
- [x] 6. print_builder.py: set_z_references forwarder
- [x] 7. Sketch: name + relative-Z + metadata
- [x] 8. Tests
- [x] 9. Run tests + finalize

## Testing Notes

- Unit: `plate_relative_to_zref` / `zref_to_plate_relative` round-trip for
  ZDIR=±1; `_apply_print_floor_raw` clamps deeper-than-bottom for both
  polarities and is a no-op when disarmed / uncalibrated; `print_floor_violation`.
- Integration (sim): a job whose `print_z_height` is below the plate bottom is
  clamped during `_execute_loop`; floor disarmed after the run.
- Manual (real HW): set a tiny "height above bottom", verify the needle prints
  just above the floor and a deliberately negative offset is clamped + warned.

## Status

**Complete.** All steps implemented and verified.

- New suite `tests/test_v75x_print_z_plate_bottom.py` (14) — green.
- `tests/test_v75x_quick_print_workflow.py` updated (relative-Z contract) +
  new `test_print_z_is_plate_bottom_relative`; suite green (16).
- Full `test_v75x*` discovery: **210 passed**.
- Pre-existing (not introduced here): `tests/test_v73_trajectory_planner.py`
  and `tests/test_hybrid_execution.py` have 6 `MagicMock < float` errors inside
  `PrintTrajectoryPlanner._move_z` — confirmed by a clean-checkout baseline;
  untouched by this change.

## Issues & Decisions

- **Known limitation (out of scope):** `build_well_plate_job` stacks layers with
  `print_z + layer*layer_height` (adds), which on the inverted ME3B frame drives
  *deeper* each layer. The clamp keeps this safe (extra layers cap at the floor)
  but multi-layer build-up direction is not corrected here. Flagged for a
  follow-up (polarity-aware layer stacking via `ZDIR`).
- Floor is armed only during `PrintManager` execution; PickPlace/other executors
  rely on the same controller chokepoint only when they run through a
  PrintManager. Pure pick-&-place (no PrintManager) is not armed in this pass.
