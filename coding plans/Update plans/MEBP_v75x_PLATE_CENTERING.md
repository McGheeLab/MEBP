# MEBP v7.5.x — Default Plate Centering in XY Bounds

## Objective

The uncalibrated well plate must be **centered in the configured XY travel envelope
(safety limits) by default**, then refined by well-plate calibration. After the user
reset their min/max bounds to an *asymmetric* envelope, the plate rendered at the corner
/ outside the bounds, because the default seed centered the plate on `zero_position`
(which only equals the envelope center when the envelope is symmetric about zero).

## Root Cause

The default plate position is seeded by `WellPlate.get_all_positions_from_plate_center(cx, cy)`
with `cx, cy = controller.zero_position["x"/"y"]`. The original design (see the comment at
`jog_control.py:360-365`) *assumed the safety envelope is symmetric about zero*, so
zero == envelope center. With the default `±130000 / ±85000` envelope that holds, but the
user's `ME3B V1` profile uses a positive-only `0..114332 / 0..76645` envelope whose center
is `(57166, 38322.5)`, not `(0,0)` — so the plate centered on zero lands at the corner.

Bounds are **zero-referenced µm** (`SafetyLimits.xy_min_x/...`); predicted positions are
**absolute stage µm** (centered, then `_wells_in_zero_ref()` subtracts `zero_position` for
display). The fix must respect that frame split.

## Fix (centralized, reuses existing helpers)

1. **`SupportClasses/SafetyLimits.py`** — add `xy_center()` returning the zero-referenced
   envelope midpoint `((xy_min_x+xy_max_x)/2, (xy_min_y+xy_max_y)/2)`. Single source of truth.
2. **`SupportClasses/StageController.py`** — add `default_plate_center_um()` returning the
   center in **absolute stage µm**: `xy_center()` + `zero_position`. The controller owns
   both `safety_limits` and `zero_position`, so the frame conversion lives in exactly one
   place. (For the symmetric default envelope this returns `zero_position` → no behavior change.)
3. **`gui/pages/calibration.py`** — replace every default-seed site with
   `controller.default_plate_center_um()`:
   - `set_hardware_config` (startup / plate-change seed, ~877-883)
   - `_wells_in_zero_ref()` geometry-only fallback (~713-726) — also fix its frame bug: it
     subtracted the *center* `(cx,cy)` instead of `zero_position`, which pinned the plate to
     canvas-(0,0); now subtracts `zero_position` so it lands at the bounds center.
   - the **three** hardcoded `(65000.0, 42500.0)` seeds: the two scan/manual-fit prediction
     seeds (~4279, ~5426) **and** the simulated-camera plate anchor in
     `_configure_simulated_cameras()` (~3873) — all three must stay coupled or simulated
     auto-calibration drives to wells in one frame while the sim renders the plate in another.
   - add `recenter_default_plate()` — re-seeds `_predicted_positions` from the current
     envelope center and refreshes views, **gated on `_calibrated_positions is None and
     _taught_a1 is None and _three_well_calibration is None`** so it never clobbers a real
     calibration *or a solved-but-unaccepted fit* (the pending affine is anchored to the
     current predicted grid; moving it would miscalibrate on Accept).
4. **`gui/pages/jog_control.py`** — `load_startup_plate` uses `default_plate_center_um()`
   (+ docstring corrected). The Jog page has **no** recenter method: bounds-change
   re-centering is owned solely by the Calibration page (single source of truth) and reaches
   the Jog page through the existing `calibration_data_changed` → `_push_cal_to_jog` path.
5. **Live bounds-change trigger**:
   - `gui/pages/hardware_setup.py` — add `safety_limits_changed = Signal()`.
   - `gui/pages/hardware/stage_panel.py` — emit it from the existing
     `_notify_control_panel_safety_changed()` (which already walks up to the
     `HardwareSetupPage`), so it fires when the user clicks **Save Safety Limits && Zero**
     (`_apply_safety_and_zero`, the only path that pushes new bounds to the live controller).
   - `gui/app.py` — in the page-wiring method, connect `hw_page.safety_limits_changed` to a
     closure that calls **only** `cal_page.recenter_default_plate()` (gated; emits → pushes to
     Jog). It deliberately does NOT poke the Jog page directly (that would be an ungated write
     that could clobber calibrated workspace state).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SafetyLimits.py` | new `xy_center()` |
| `SupportClasses/StageController.py` | new `default_plate_center_um()` |
| `gui/pages/calibration.py` | **5** seed sites → `default_plate_center_um()` (incl. simulated-camera anchor); fallback frame fix; new gated `recenter_default_plate()` |
| `gui/pages/jog_control.py` | startup seed → `default_plate_center_um()`; docstring fixed (no recenter method — cal page owns re-centering) |
| `gui/pages/hardware_setup.py` | new `safety_limits_changed` signal |
| `gui/pages/hardware/stage_panel.py` | emit `safety_limits_changed` on save |
| `gui/app.py` | connect signal → re-center cal + jog pages |

## Implementation Steps

- [x] `SafetyLimits.xy_center()`
- [x] `StageController.default_plate_center_um()`
- [x] calibration.py: `set_hardware_config` seed
- [x] calibration.py: `_wells_in_zero_ref` fallback (seed + frame fix)
- [x] calibration.py: **3** hardcoded `(65000,42500)` seeds → scan (~4279), manual-fit (~5426), **simulated-camera anchor (~3873)**
- [x] calibration.py: `recenter_default_plate()` (gated incl. `_three_well_calibration`)
- [x] jog_control.py: `load_startup_plate` seed + docstring fix (no recenter method)
- [x] hardware_setup.py: `safety_limits_changed` signal
- [x] stage_panel.py: emit on `_apply_safety_and_zero`
- [x] app.py: wire signal → `cal_page.recenter_default_plate()` only (single source of truth)
- [x] Syntax-check all modules (all 7 parse; GUI modules import cleanly)
- [x] Unit tests: `tests/test_v75x_plate_centering.py` (6 tests pass)
- [x] Regression: full suite unchanged vs baseline (13 pre-existing failures confirmed unrelated via `git stash` baseline); `test_v730_simulated_camera` (23) passes
- [x] Adversarial review (4-dimension workflow, 7 findings) — all addressed (see below)

## Testing Notes

- **Symmetric default (`±130000 / ±85000`)**: `xy_center() == (0,0)`, so
  `default_plate_center_um() == zero_position` — behavior must be byte-identical to today.
- **ME3B V1 (`0..114332 / 0..76645`)**: plate should render centered at `(57166, 38322.5)`
  zero-ref µm — i.e. the center of the drawn bounds rectangle in `JogWorkspaceView` (Calibration
  Plate-Location tab + Jog page workspace).
- **Live trigger**: with the app running, edit bounds and click *Save Safety Limits && Zero*;
  the plate should re-center immediately (no restart).
- **Calibration wins**: teach/accept a calibration, then change bounds — the plate must stay
  at the calibrated positions (re-center is gated on uncalibrated).

## Issues & Decisions

- "Centered" = bounds **midpoint** (not clamp-to-fit). For ME3B V1 the plate footprint
  (127.76×85.48 mm) is *larger than the travel envelope in both axes*, so even centered the
  plate edges/corner wells extend beyond reachable travel — a physical/config reality, not a
  centering bug; midpoint is still the correct default.
- Re-center is hooked to **Save Safety Limits && Zero** (`_apply_safety_and_zero`), the only
  button that pushes new bounds to the live controller. The bottom *Apply Settings* (`_apply`)
  only persists and does not update `controller.safety_limits`, so re-centering there would use
  stale bounds; it takes effect on next launch via the startup seed instead.
- Frame conversion is centralized in `StageController.default_plate_center_um()` so the
  zero-ref↔absolute split can't be mis-applied per call site.
- Not hooking `zero_position` changes (zero-needle calibration) — pre-existing behavior,
  out of scope; calibration overrides shortly after a zero anyway.

### Adversarial review findings (all fixed)

A 4-dimension review workflow (frame / wiring / calibration-safety / regression) surfaced
7 confirmed-real findings, consolidating to 4 fixes:

1. **Simulated-camera seed missed (medium).** Only 2 of **3** coupled `(65000,42500)` seeds
   were converted; `_configure_simulated_cameras()` still hardcoded it, decoupling the sim
   world from the predicted grid (broke simulated auto-calibration even on the *default*
   envelope, where the new seed is `(0,0)`). Fixed: seed the sim anchor from
   `default_plate_center_um()` with the same fallback.
2. **Ungated + redundant Jog re-center (high→resolved).** The app closure called both
   `cal_page` and `jog_page` recenter; the jog one had no calibration gate and could clobber
   calibrated workspace state, plus double-wrote on every save. Fixed: app calls only the
   *gated* `cal_page.recenter_default_plate()`; the Jog `recenter_default_plate` method was
   removed (cal page pushes to Jog via the existing signal path).
3. **Gate missed solved-but-unaccepted fit (medium).** `_three_well_calibration` is set before
   `_calibrated_positions`/`_taught_a1`; a mid-scan bounds-save would shift the predicted grid
   the pending affine was fit against → silent miscalibration on Accept. Fixed: gate now also
   checks `_three_well_calibration`.
4. **Stale docstring (low).** `load_startup_plate` docstring still claimed "centred on zero /
   appears at (0,0)"; corrected to describe envelope-midpoint centering.
