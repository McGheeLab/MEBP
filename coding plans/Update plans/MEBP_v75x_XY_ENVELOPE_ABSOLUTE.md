# MEBP v7.5.x — XY Safety Envelope → Absolute Stage Frame

## Objective

Fix needle-zero (Needle Location) calibration, where small XY centering moves
were wrongly clamped, plus a hard crash in the same handler.

Two bugs were reported:

1. **XY clamp rejects valid moves after re-zeroing.** During Needle Location
   centering the stage did a *Set Zero* (`zero_position` became `(59562, 58143)`
   mid-travel). The XY safety envelope is stored **zero-referenced**
   (`xy_min=0`), and `Record Min/Max` stores `xy - zero`. After re-zeroing, the
   zero-referenced current position is ≈`(0,0)` — exactly on the envelope
   minimum — so any **negative** delta (e.g. `dy=-9.2 µm`) lands below `0` and
   is clamped to `0`, even though the absolute destination is mid-travel and
   physically valid (`XY clamped: (22.5,-9.2) → (22.5,0.0)`). The envelope no
   longer corresponds to the physical box once the zero is re-anchored.

2. **Crash:** `calibration.py::_needle_loc_center_and_save` called
   `stage_to_um(value)` but the signature is `stage_to_um(readout, xy_position_scale)`
   → `TypeError: stage_to_um() missing 1 required positional argument`.

**Decision (user):** Make the XY safety envelope an **absolute stage-µm** frame —
fixed mechanical limits that do not move when the operator does *Set Zero*. The
clamp compares the **absolute final destination** against the absolute bounds.
This also prevents driving past the mechanical limits on plate moves and fixes a
latent frame mismatch in the Hardware control-panel position bars (bar *value*
was already absolute while its *range* was zero-ref).

Z and pump limits are **unchanged** (remain zero-referenced mm) — out of scope.

## Frame model (after this change)

- `get_xy_position()` → absolute Prior µm (unchanged).
- `zero_position["x"|"y"]` → absolute µm designated as plate origin (Set Zero).
- `SafetyLimits.xy_min_x/.../xy_max_y` → **absolute Prior µm** (was zero-ref).
- Clamp paths compare the **absolute** destination against the absolute bounds.
- User-facing position readouts stay **zero-ref** (`xy - zero`, plate-relative).
- Display widgets that draw the envelope alongside a zero-ref needle position
  (`JogWorkspaceView`, `XZSideView`) convert the absolute envelope → zero-ref by
  subtracting the **current** `zero_position` at draw time (single chokepoint).
  In the common case (no re-zero, or simulator zero=0) the displayed envelope is
  pixel-identical to before; after a re-zero it correctly shows the plate origin
  shifting within the fixed physical box.

### Migration note

Existing persisted envelopes are **reinterpreted as absolute** (no numeric
transform). For ME3B V1 the stored `[0, 114332] × [0, 76645]` matches the
physical span the Prior reports, so this is correct. The `Record as Min/Max`
buttons now store the absolute reading, so any stage whose origin is non-zero can
re-record each limit with one click. This is documented; no fragile auto-migration
(the original record-time zero is not persisted, so it cannot be inferred).

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | **(done)** Bug 2 — pass `self._xy_position_scale` to `stage_to_um`. Push `set_zero_offset` to the Needle-Location plate view. |
| `SupportClasses/SafetyLimits.py` | Docstrings: XY limits are now absolute stage µm; `xy_center()` returns the absolute midpoint. (`clamp_xy` body unchanged.) |
| `SupportClasses/StageController.py` | Clamp the **absolute** destination in `move_xy_absolute`, `move_xy_absolute_um`, `move_xy_relative`, `move_xy_relative_um`. `default_plate_center_um` → returns `xy_center()` (absolute) directly. |
| `gui/pages/hardware/stage_panel.py` | `_record_limit` stores absolute `xy[0]/[1]` (drop `- zero`). Label/tooltip wording (X/Y = absolute stage µm). |
| `gui/widgets/jog_workspace_view.py` | Add `_zero_off` + `set_zero_offset`; subtract it in `_envelope_bounds` (the single chokepoint feeding scale/flip/paint/click). |
| `gui/widgets/xz_side_view.py` | Add `_zero_off_x` + `set_zero_offset_x`; subtract in `_x_bounds_um`. |
| `gui/pages/jog_control.py` | Push `set_zero_offset` to workspace + xz view in `on_status_update`. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | Push `set_zero_offset` next to its workspace `set_position`. |
| `gui/unit_helpers.py` / `gui/pages/settings_page.py` | Wording only — limits display labelled absolute stage µm. |
| `tests/test_v75x_plate_centering.py` | Update to the new `default_plate_center_um == xy_center()` (absolute) contract. |
| `tests/test_v75x_xy_envelope_absolute.py` | **New** — clamp uses absolute destination; negative delta near a re-anchored zero is allowed; record-limit stores absolute. |

## Implementation Steps

- [x] Bug 2: fix `stage_to_um` call in `_needle_loc_center_and_save`.
- [x] `SafetyLimits`: docstring updates (XY absolute; `xy_center` absolute midpoint).
- [x] `StageController.move_xy_absolute`: clamp after adding zero (absolute). (Also now clamps the `from_zero_ref=False` legacy path, which previously skipped the soft limit — a safety improvement.)
- [x] `StageController.move_xy_absolute_um`: clamp `x_um/y_um` directly (absolute).
- [x] `StageController.move_xy_relative` / `move_xy_relative_um`: clamp `cached + delta` (absolute); `delta = clamped - cached`.
- [x] `StageController.default_plate_center_um`: return `xy_center()` (absolute).
- [x] `stage_panel._record_limit`: store absolute (the per-axis "now" readout was already absolute, so the spinboxes are now coherent with it).
- [x] `jog_workspace_view`: `set_zero_offset` + subtract in `_envelope_bounds`.
- [x] `xz_side_view`: `set_zero_offset_x` + subtract in `_x_bounds_um`.
- [x] `jog_control.on_status_update`: push zero offsets to both views.
- [x] `calibration` + `spheroid` workflow: push zero offset to their plate views.
- [~] `unit_helpers` / `settings_page`: no change needed — `convert_safety_xy_text` only formats the numbers (no zero-ref wording); `control_panel` bars become coherent (value was already absolute).
- [x] Update `test_v75x_plate_centering.py`; add `test_v75x_xy_envelope_absolute.py`.
- [x] Run sim test suite — new 16 + 53 related tests green. Pre-existing failures (10 print/trajectory `MagicMock < float` errors; 3 sim state-persistence flakes) verified present on a clean stash of these changes, i.e. unrelated.

## Testing Notes

- New unit tests assert: after `Set Zero` to a mid-travel absolute position, a
  small negative relative move is **not** clamped (absolute destination within
  bounds); a move that would exceed the absolute envelope **is** clamped.
- `default_plate_center_um()` equals the absolute envelope midpoint regardless of
  `zero_position`.
- Manual (HW): Needle Location centering now completes the camera-derived
  correction without the spurious `XY clamped` warning and saves `needle_origin_um`.

## Issues & Decisions

- **Plate-centering behavior change:** `default_plate_center_um` no longer adds
  `zero_position`. Under the absolute envelope the midpoint is already absolute,
  so adding zero would place the plate outside the physical box. The new value
  centers the default plate in the physical travel — more correct.
- **No auto-migration** of persisted limits (record-time zero unknown); reinterpret
  as absolute + one-click re-record. Documented above.
