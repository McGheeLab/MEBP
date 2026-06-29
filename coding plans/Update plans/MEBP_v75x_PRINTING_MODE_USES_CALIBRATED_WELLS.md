# MEBP v7.5.x — Printing-mode print path must use CALIBRATED well positions

## Objective

Fix the operator-reported failure: after calibrating the plate and assigning
inks/washes, pressing **Run** in **Printing mode** drove the stage to the stage
origin corner `(0,0)` at the wrong place, the coordinates were all wrong, and
the needle never went to the assigned wells. The log is full of:

```
PRINT: well C2 — first point (-22.5, -42.9)
XY clamped: (-22518.5,-42936.4) → (0.0,0.0)
wait_for_xy_arrival timeout: target=(-22.52, -42.94), actual=(0.234, 0.132)
```

### Root cause

The **entire Printing-mode print path** computes each well centre
*geometrically* — `WellPlate.get_well_position(name)` returns a **plate-local
A1-relative** offset (A1 at `(0,0)`) — and multiplies it by
`controller.plate_axis_sign()` (`(-1,-1)` on ME3B V1). The signed geometric
offset is then handed to `move_xy_absolute(..., from_zero_ref=True)` as if it
were a **zero-ref mm** stage coordinate. Because `zero_position` is the *needle
zero* `(0,0)`, the well grid ends up anchored at the stage origin and the
`(-1,-1)` flip makes every well **negative** → clamped to `(0,0)`.

The taught/calibrated well positions (e.g. C2 = `(86028, 28274)` µm,
A1 = `(107561, 68633)` µm) live on the **Calibration page**
(`_calibrated_positions`) and are exposed via `cal_page.get_calibration_data()`
→ `(plate, well_positions, safe_z)`. They are wired to the **Jog page** and
**Workflows mode**, but **never to the Printing-mode setup page**. So the print
ignored calibration entirely.

This affected **all** Printing-mode execution sub-paths:
- `print_setup_legacy._build_job` (discrete commands)
- `HybridPlanExecutor` (the DEFAULT mode — `_execute_print_step`, `TRAVEL_XY`,
  `_execute_service`, `estimate_time`)
- `PrintTrajectoryPlanner._well_xy` (trajectory waypoints + service wells)
- `PrintTrajectoryPlanner._find_well` returned the *raw* geometric offset (not
  even signed) — service wells went to `(0,19300)` etc.

Quick Print was **unaffected** — it receives `set_calibration_data` and resolves
the calibrated position first (`_well_center_zero_ref_mm`).

## Fix (single source of truth: calibrated positions → zero-ref mm)

A well centre used by the print path is in **zero-ref mm** (the frame
`move_xy_absolute(from_zero_ref=True)` and the trajectory waypoints expect).
Introduce one resolver that PREFERS the per-job calibrated map and falls back to
the geometric × sign behaviour (byte-identical legacy → tests/uncalibrated jobs
unaffected):

1. **`PrintSettings.well_positions_mm: dict | None = None`** — calibrated well
   centres in zero-ref mm, by name, stamped at job-build time. `None` =
   geometric fallback.
2. **`PrintTrajectoryPlanner.resolve_well_xy_mm(well_name, plate, settings)`** —
   the shared resolver: `settings.well_positions_mm[name]` if present, else
   `plate.get_well_position(name)` × `settings.plate_axis_sign`. `_well_xy` now
   delegates to it (planner stores `self._settings` in `generate()` /
   `generate_inwell_print()`), so every planner well (print + service) is fixed.
3. **`HybridPlanExecutor`** — imports the resolver; uses it in
   `_execute_print_step`, `TRAVEL_XY`, `estimate_time` (PRINT + TRAVEL_XY), and
   `_execute_service` (resolves the role-matched well's position via the resolver
   instead of `_find_well`'s raw geometric offset).
4. **`print_setup_legacy`**:
   - new `_calibrated_well_positions` attr + `set_calibration_data(plate,
     well_positions, safe_z)` (mirrors Quick Print / Jog).
   - new `_well_xy_zref_mm(name, plate)` page helper (calibrated → zero-ref mm,
     else geometric × sign).
   - `_build_job` builds `well_positions` via `_well_xy_zref_mm` (discrete path).
   - `_get_settings` stamps `s.well_positions_mm` from the calibrated dict.
5. **`WizardPrintSetupPage.set_calibration_data`** — forwards to `self._legacy`.
6. **`gui/app.py`** — wire `cal_page.calibration_data_changed` →
   `setup_page.set_calibration_data(*cal_page.get_calibration_data())` and push
   once at startup (mirrors the Jog/Workflows wiring).

Safety invariants are untouched: retract-before-XY, end-at-safe-Z, plate-bottom
floor. This only corrects *where* XY travels — onto the physically-taught well.

## Second bug — "Z was not moving up between wells" (travel/print Z defaulted)

Same architectural cause, different symptom. The legacy print page resolves its
calibrated Z heights through `_find_app_settings()` / `_app_settings`, but
`_app_settings` is **never assigned** and the page is composed **detached**
(`parent=None`) in the wizard, so the parent-walk fails. Result:

- `travel_z_height` stayed at the **5.0 mm** spinbox default (`_compute_auto_settings`
  couldn't read `safe_z`). On ME3B V1 the plate bottom is **6.06 mm** and larger
  zero-ref Z = higher, so **5.0 mm is BELOW the plate** → commanding "retract to
  safe Z" drove the needle **DOWN**, never up. This is the operator's "Z was not
  moving up and going to the next well."
- `print_z_height` stayed at the **0.1 mm** default (`_calibration_plate_bottom_z`
  also used the broken read) — below the plate bottom, so the print floor clamp
  pinned it to the bottom instead of the intended height.

Fix — source these from data already wired into the page, bypassing the broken
app-settings read:
- `set_calibration_data` receives the calibrated `safe_z` (= the Calibration
  page's Fast-Move Z) and drives `travel_z_spin` (range widened if needed) so
  both the discrete (`_get_settings` reads the spin) and hybrid/trajectory paths
  retract to the real safe height.
- `_calibration_plate_bottom_z()` now prefers `controller.get_plate_bottom_z()`
  (pushed by `app.py::_update_print_floor_datum` from the cal page), so
  `_apply_plate_relative_print_z` derives the correct print Z.
- `_get_settings` sets `top_z_height` from `controller.get_plate_top_z()` for the
  staged descent (fast → top_z+0.5 → slow entry).

(The travel spin shows 1 decimal, so e.g. 27.71 → 27.7 — a 0.01 mm rounding,
negligible for a retract height.)

## Files Modified

- `SupportClasses/PrintManager.py` — `PrintSettings.well_positions_mm`;
  `HybridPlanExecutor` well resolution (4 sites + service).
- `SupportClasses/PrintTrajectoryPlanner.py` — `resolve_well_xy_mm`,
  `_well_xy` delegation, `self._settings` capture.
- `gui/pages/print_setup_legacy.py` — `set_calibration_data` (well map +
  safe-Z → travel spin), `_well_xy_zref_mm`, `_build_job`, `_get_settings`
  (well map + top_z from controller), `_calibration_plate_bottom_z` (controller
  datum first).
- `gui/pages/print_setup/page.py` — `WizardPrintSetupPage.set_calibration_data`.
- `gui/app.py` — wire calibration → Printing-mode setup page.
- `tests/test_v75x_printing_mode_calibrated_wells.py` — NEW.

## Status

- [x] Root cause confirmed from log + calibration JSON + code trace
- [x] `PrintSettings.well_positions_mm`
- [x] `resolve_well_xy_mm` + planner delegation
- [x] HybridPlanExecutor resolver usage (print/travel/service/estimate)
- [x] print_setup_legacy set_calibration_data + resolver + stamping
- [x] WizardPrintSetupPage forward
- [x] app.py wiring
- [x] Z fix — travel/safe Z from calibrated safe_z; print/top Z from controller
- [x] Tests
- [ ] **Needs real-HW verification on ME3B V1** (re-run the Box print → retracts
      to safe Z + travels to each taught well, service wells correct, prints at
      the right height, ends at safe Z)

## Testing Notes

`tests/test_v75x_printing_mode_calibrated_wells.py`:
- resolver prefers calibrated map; falls back to geometric × sign when absent.
- a stamped `well_positions_mm` makes the discrete job's MOVE_XY land on the
  calibrated zero-ref mm (not the negative geometric value).
- planner `_well_xy` returns calibrated zero-ref mm when settings carry the map.
- regression: with no calibration map, every path is byte-identical to before.
