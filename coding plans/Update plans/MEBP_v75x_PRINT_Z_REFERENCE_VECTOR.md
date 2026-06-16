# MEBP v7.5.x — Print-Z up-direction from the plate-bottom→plate-top reference vector

## Objective

Stop hard-coding the print-Z up/down direction. Derive it from the two
calibrated reference points — **plate bottom** and **plate top** — so print
offsets are correct on any machine, whether a user thinks of the print height as
"above the bottom" or "below the top."

On ME3B V1 the needle **descends as the raw Marlin Z counter increases**, so
"up" is the opposite of a conventional machine. The direction was a single
hard-coded module constant `StageController.ZDIR = -1.0`, and several print-offset
sites still assumed the conventional "larger Z = higher", which inverts on this
machine:

- `PrintManager.build_well_plate_job` stacked layers with
  `print_z_height + layer*layer_height` → drove each layer **deeper** on ME3B
  (previously flagged as a known limitation in
  `MEBP_v75x_PRINT_Z_PLATE_BOTTOM_DATUM.md`).
- `PrintTrajectoryPlanner._well_print_z` used `top_z + ink_z_mm` with a comment
  that literally said "Larger Z = higher."
- The plate-bottom datum/floor helpers (`print_height_to_zref`, the print floor)
  used the hard-coded `ZDIR` rather than the machine's actual taught geometry.

**Outcome:** the print-Z up direction is now
`sign(plate_top_zref − plate_bottom_zref)` (the plate top is physically above the
bottom), falling back to `ZDIR` only until both references are taught. The same
print recipe is correct on a conventional machine and on ME3B V1, and multi-layer
build-up grows **up** instead of punching deeper.

### User decisions (asked up front)
- **Scope = print-offset math only.** The Jog/Device Z *display* numbering and
  soft-limit min/max swap stay on `ZDIR` (separate, already-correct concern).
- **Canonical input convention = "height above plate bottom" (positive = up).**
  Already the implemented convention; this change makes the *direction* derive
  from geometry instead of a constant.

## Design

```
# StageController module-level — single source of truth, pure + testable
def derive_z_up_sign(plate_top_zref, plate_bottom_zref, fallback=None):
    if plate_top_zref is not None and plate_bottom_zref is not None:
        d = plate_top_zref - plate_bottom_zref
        if abs(d) > 1e-6:
            return 1.0 if d > 0 else -1.0
    return ZDIR if fallback is None else fallback   # live module ZDIR
```

- `StageController.print_z_dir()` = `derive_z_up_sign(self._plate_top_z_zref,
  self._plate_bottom_z_zref)`. Uses `getattr(..., None)` so the lightweight
  `__new__`-built test controllers work; resolves the **live** module `ZDIR` as
  fallback (so tests that monkeypatch `SC.ZDIR` behave correctly).
- The existing polarity-general helpers `plate_relative_to_zref` /
  `zref_to_plate_relative` already take a `zdir` arg — the datum/floor methods
  now pass `self.print_z_dir()` instead of relying on the default `ZDIR`.
- The direction is carried into the discrete print path via a new
  `PrintSettings.z_up_sign` (default `+1.0` = legacy additive behaviour),
  stamped from `controller.print_z_dir()` by the GUI settings-builders. Layer
  stacking and per-well dispense-Z step by `z_up_sign * offset`.

`ink_z_mm` keeps its "relative to plate top" meaning but is now applied along the
reference-vector direction, so a negative value means "into the well" on **both**
polarities. Print height stays "above plate bottom" — the same vector resolves
both ends.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | New `derive_z_up_sign()` free fn; `_plate_top_z_zref` state + `set_plate_top_z()`/`get_plate_top_z()`; `print_z_dir()`; route `print_height_to_zref`, `zref_to_print_height`, `print_floor_violation`, `_apply_print_floor_raw` through `print_z_dir()` |
| `SupportClasses/PrintManager.py` | `PrintSettings.z_up_sign` field (default `+1`); `build_well_plate_job` layer stacking steps by `z_up_sign * layer * layer_height` |
| `SupportClasses/PrintTrajectoryPlanner.py` | `_well_print_z` → `top_z + z_up_sign * ink_z_mm`; docstring fixed (polarity-aware) |
| `gui/app.py` | `_update_print_floor_datum` also pushes `plate_top_z` via `set_plate_top_z` |
| `gui/pages/print_setup_legacy.py` | New safe `_print_z_dir()`; stamp `s.z_up_sign`; pass derived `zdir` to `_apply_plate_relative_print_z` + `_confirm_print_floor` (deepest-layer check now matches the new stacking direction) |
| `gui/pages/workflows/quick_print_workflow.py` | Stamp `settings.z_up_sign = float(controller.print_z_dir())` (guarded) |
| `tests/test_v75x_print_z_reference_vector.py` | New tests (16) |

## Implementation Steps

- [x] 1. StageController: `derive_z_up_sign`, plate-top datum + setters,
  `print_z_dir`, route datum/floor methods through it
- [x] 2. PrintManager: `PrintSettings.z_up_sign` + layer-stacking fix
- [x] 3. PrintTrajectoryPlanner: `_well_print_z` polarity + docstring
- [x] 4. app.py: push `plate_top_z`
- [x] 5. GUI builders: stamp `z_up_sign`; safe `_print_z_dir()`; derived `zdir`
- [x] 6. Tests + run unit / regression suites
- [x] 7. Docs (this plan + CLAUDE.md table row)

## Testing Notes

- **Unit (`tests/test_v75x_print_z_reference_vector.py`, 16):**
  `derive_z_up_sign` (+1/−1/fallback/coincident/default); `print_z_dir()`
  geometry vs `ZDIR` fallback; conversions + floor + violation follow the
  **derived** sign even when the module `ZDIR` is monkeypatched the opposite way
  (geometry wins); `build_well_plate_job` layers decrease (ME3B up=−1) /
  increase (conventional up=+1); fresh `PrintSettings` defaults to legacy
  additive; `_well_print_z` ink-Z sign-correct both polarities + fallbacks.
- **Regression:** `python -m unittest discover -s tests -p "test_v75x*"` →
  **305 passed**. Existing `tests/test_v75x_print_z_plate_bottom.py` stays green
  (every case omits the plate top → exercises the `ZDIR` fallback path).
- **Pre-existing (not introduced here):** `tests/test_v73_trajectory_planner.py`
  + `tests/test_hybrid_execution.py` — **6** `MagicMock < float` errors inside
  `PrintTrajectoryPlanner._move_z` (line 182), confirmed at baseline; the
  tracebacks pass *through* `_well_print_z` without erroring there.
- **Manual (real HW, ME3B V1):** teach plate bottom + plate top; set "height
  above bottom" = 0.5 mm; run a 2-layer print — layer 2 must sit *higher*
  (further from the floor) than layer 1, and a negative/over-deep value must warn
  + clamp at the plate bottom.

## Issues & Decisions

- **Frozen-default gotcha:** `derive_z_up_sign(fallback=ZDIR)` would freeze
  `ZDIR` at import time, breaking the existing `test_clamp_conventional_polarity`
  (which monkeypatches `SC.ZDIR`). Fixed with a `fallback=None` sentinel that
  resolves the **live** module `ZDIR`; `print_z_dir()` relies on this.
- **MagicMock controllers:** `hasattr(mock, "print_z_dir")` is always True and
  the mock's return isn't a float, so `z_up_sign` became a mock (broke
  `test_v75x_quick_print_workflow`'s job-build). Fixed by coercing via
  `float(controller.print_z_dir())` inside try/except → falls back to the
  default / `ZDIR`. `print_setup_legacy` centralizes this in `_print_z_dir()`.
- **Scope held to print offsets** (user choice): the trajectory planner's broader
  travel/safe-Z gates (`approach_z = top_z + 0.5`, `safe_z` raise-before-travel)
  remain `ZDIR`-based — still flagged at the top of `PrintTrajectoryPlanner.py`;
  the discrete path is the recommended one and is travel-safe via
  `ensure_retracted_to`. `GeometryEngine.compute_layer_heights` (object-internal
  3D shape, discarded by the discrete path) is also untouched.

## Status

**Complete.** All steps implemented and verified; new suite + full `test_v75x*`
green.
