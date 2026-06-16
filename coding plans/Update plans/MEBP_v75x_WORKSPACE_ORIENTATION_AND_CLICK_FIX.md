# MEBP v7.5.x — Workspace Orientation + Click-to-Move Unit Fix

## Objective

Two bugs reported after `ME3B V1` hardware calibration on the Prior ProScan II stage:

1. **Plate overlay drawn 180° rotated.** The stage origin is at the bottom-right with
   +X/+Y toward the top-left, but the top-down workspace canvas drew +X→right / +Y→down,
   so the plate rendered upside-down relative to the operator's physical view. (Motion/
   mapping were correct — purely a display orientation mismatch.)
2. **Click-to-move travels ~1000× too far.** Clicking a target in the workspace tried to
   move the stage absurdly far.

## Root Causes

1. **Orientation:** `JogWorkspaceView._um_to_px` maps `(x_um, y_um) → (x*k+ox, y*k+oy)`
   with Qt's screen-down Y. For a stage whose physical +X/+Y point up-left, both axes are
   mirrored vs. the operator's view → a 180° rotation. There is **no** existing XY
   orientation config (`_axis_flip` covers only Z/P1/P2/P3 and `axis_sign()` is unused), so
   the view never compensated.
2. **Units:** the workspace emits clicks in **zero-ref µm**, and the click handlers passed
   those µm straight into `StageController.move_xy_absolute(..., from_zero_ref=True)`, which
   is documented to take **mm** (it multiplies by 1000 internally; see `test_xy_motion.py`).
   So a 50 mm (50000 µm) click became 50000 mm → 1000× overshoot. Present in **three**
   handlers (Jog page, shared jog-context "Go to", Workflows mode). The `safe_travel_to`
   branch was already correct (it takes absolute µm).

## Changes

### Issue 1 — 180° display flip (hardcoded, per user choice)
- `gui/widgets/jog_workspace_view.py`: new class constant `_FLIP_DISPLAY_180 = True` and a
  `_apply_display_flip()` helper that reflects a zero-ref µm point about the envelope centre.
  Applied inside **both** `_um_to_px` (forward) and `_px_to_um` (inverse) — the reflection is
  its own inverse, so the whole render (plate, wells, needle, breadcrumbs, hover) rotates 180°
  while click targeting stays exact. Flip to `False` for a conventionally-mounted stage.
  (A future per-machine setting could drive this; hardcoded for now per request.)

### Issue 2 — click-to-move unit fix (µm → mm)
- `gui/pages/jog_control.py` `_on_workspace_position_clicked`: both direct-move branches now
  pass `x_um_zr/1000.0, y_um_zr/1000.0`.
- `gui/widgets/standard_jog_context.py` `_absolute_goto`: both XY branches divide by 1000
  (the goto X/Y spin-boxes are µm; the Z box is mm and `move_z_absolute` was already correct).
- `gui/pages/workflows/spheroid_pickup_workflow.py` `_on_workspace_position_clicked`: both
  direct-move branches divide by 1000.
- `safe_travel_to`/`fast_travel` paths unchanged (already pass absolute µm).

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/jog_workspace_view.py` | `_FLIP_DISPLAY_180` + `_apply_display_flip()`; applied in `_um_to_px`/`_px_to_um` |
| `gui/pages/jog_control.py` | click-to-move µm→mm (2 calls) |
| `gui/widgets/standard_jog_context.py` | `_absolute_goto` XY µm→mm (2 calls) |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | click-to-move µm→mm (2 calls) |

## Testing Notes

- Headless coordinate test (QApplication offscreen): flipped `_um_to_px`/`_px_to_um`
  round-trips to identity (clicks accurate); envelope x_min renders right / x_max left and
  y mirrored (180° confirmed); envelope centre invariant.
- `test_v731_jog_navigation` (26), `test_v75x_plate_centering` (6),
  `test_v730_simulated_camera` (23) all pass; touched modules import cleanly.
- On hardware: click a target → stage moves there (not 1000× past); plate overlay now matches
  the physical plate orientation; click still lands on the intended well.

## Issues & Decisions

- **Orientation handling:** user chose a hardcoded 180° flip over a configurable per-machine
  toggle. Implemented as a single discoverable class constant so it's a one-line change to
  revisit or promote to a setting later.
- **Bug-fix scoped to all 3 click handlers**, not just the reported Jog page — the same µm/mm
  error existed in the shared jog-context "Go to" box and the Workflows-mode picker, and would
  cause the same dangerous overshoot.

### Separate bug found — FIXED (user approved)
- `StageController.move_xy_absolute_um()` was **called** in `gui/pages/calibration.py` (lines
  ~2094 / 2166 / 2222 / 2259 — auto-calibration drive-to-well + ring scan) but was **never
  defined** on `StageController` (no `__getattr__` shim) → guaranteed `AttributeError` when
  those calibration paths ran. **Fixed:** added `move_xy_absolute_um(self, x_um, y_um,
  fast=False)` taking absolute stage µm; honors safety limits by clamping in the zero-ref
  frame (convert → `clamp_xy` → convert back) so auto-calibration can't drive past the
  envelope. Verified: in-bounds passes through, out-of-bounds clamps, nonzero `zero_position`
  handled.
