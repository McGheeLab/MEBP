# MEBP v7.5.x — Permanent Calibration Reference Markers (Plate + Microscope Views)

## Objective

Show the **taught well centres** from plate calibration as permanent reference markers on
**both** the Plate Location plate view and the live microscope camera view — during
calibration (as each well is taught) and after — so the operator can visually confirm the
plate map and the live image are registered to the physically-taught points.

## Design

- A taught reference is a well's **measured centre** in absolute stage µm (the circle-fit
  result of the rim clicks / 3-point fit / auto edge-fit), stored on
  `CalibrationPage._reference_markers: dict[name → (x_um, y_um)]`.
- **Plate view** (`JogWorkspaceView`): markers are drawn as a distinct **pink ⊕** (ring +
  crosshair + well-name label) in zero-ref µm, layered above the wells (under the live
  needle crosshair), clipped to the workspace.
- **Microscope view** (`CameraFeedView`): markers are projected into the live frame as
  `(x_um − stage_x)/µm_per_px + w/2` (camera centre = stage centre) and drawn as a pink
  ring + crosshair + label, tracking the stage as it moves. Off-frame markers are skipped.
- No new transform math — the camera projection mirrors the existing
  `TargetOverlayCameraView` / `CameraManager.pixel_to_stage_offset` convention.

## Changes

### `gui/widgets/camera_feed_view.py`
- New `set_reference_markers(markers, stage_x_um=, stage_y_um=, um_per_px=)` +
  `set_reference_stage_position(x_um, y_um)` + `_draw_reference_markers()`; hooked into the
  base `_render_frame` (re-renders the last frame when markers/stage change). Markers are
  `(name, x_um, y_um)` in absolute stage µm. (`TargetOverlayCameraView` keeps its own
  `_render_frame` override and is unaffected.)

### `gui/widgets/jog_workspace_view.py`
- New `_ref_markers` state + `set_reference_markers(dict)` (zero-ref µm) +
  `_paint_reference_markers()`, painted inside the clipped content layer after the wells.

### `gui/pages/calibration.py`
- New `self._reference_markers` (absolute stage µm) + `_reference_in_zero_ref()`.
- Populated live as each well is taught (`_ploc_finalize_click_rim`, `_ploc_confirm`
  3-point path) and in bulk from auto-fit results at `_ploc_finish_run`.
- `_refresh_ploc_view` pushes markers to the plate view (zero-ref) and the live microscope
  overlay (absolute µm + microscope-slot µm/px + current stage centre); the position-update
  loop pushes the live stage centre to `_ploc_live_view.set_reference_stage_position` so
  markers track motion. `_ploc_finalize_click_rim` also calls `_refresh_ploc_view` so a
  marker appears immediately when its well is taught.
- Persisted in `_save_calibration` (`reference_markers: {name: [x, y]}`) and restored in
  `_load_calibration`; cleared on a real plate-format change in `_on_plate_changed`.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/camera_feed_view.py` | Reference-marker overlay (stage-µm → frame projection) |
| `gui/widgets/jog_workspace_view.py` | Reference-marker layer (zero-ref µm ⊕) |
| `gui/pages/calibration.py` | `_reference_markers` state/populate/push/persist/clear |
| `tests/test_v75x_plate_location_manual_click_rim.py` | +5 tests (`TestReferenceMarkers` + finalize marker assertions) |

## Implementation Steps

- [x] Add reference-marker overlay to `CameraFeedView` (project from stage µm)
- [x] Add reference-marker layer to `JogWorkspaceView` (zero-ref µm)
- [x] `CalibrationPage` state + zero-ref helper + populate (live + bulk)
- [x] Push to both views in `_refresh_ploc_view`; track stage centre in the position loop
- [x] Immediate refresh on each well taught
- [x] Persist in save/load; clear on plate-format change
- [x] Tests + offscreen render smoke

## Testing Notes

- `tests/test_v75x_plate_location_manual_click_rim.py`:
  - `TestReferenceMarkers` — `_reference_in_zero_ref` subtracts `zero_position`; empty when
    no markers; save/load shape round-trips `{name:[x,y]}` ↔ `{name:(x,y)}`.
  - `TestFinalizeFit` — a successful fit records the marker at the taught centre; a rejected
    (<3-click) fit records **no** marker.
- Offscreen render: the plate view renders markers (incl. an off-envelope one, clipped),
  and the camera view projects a marker from stage µm without error.
- Full affected suite green (85): manual-click-rim, predict-well, plate-centering,
  freeform-warp, well-radius, jog-navigation; `CalibrationPage` builds offscreen.
- On hardware: as each well is taught its pink ⊕ appears on the plate map and (when near
  the current view) on the microscope feed; drive to a taught well and the marker should
  sit on the well's actual centre — a direct registration check. Markers persist after the
  run and across restart (until the plate format changes).

## Issues & Decisions

- **Markers = measured well centres** (the registration anchors we taught), distinct from
  the affine-corrected grid already drawn as green/yellow well dots. Color **pink** —
  unused elsewhere (wells green/yellow/blue, needle red, toggle/breadcrumbs mauve).
- **Camera overlay refresh** rides the streaming frames + stage-position pushes; if the
  camera isn't streaming the overlay simply doesn't update (acceptable for a live view).
- **Generic overlay on the base `CameraFeedView`** (not coupled to `PickPlaceTarget`) so it
  stays reusable and doesn't pull in PickAndPlace types.
