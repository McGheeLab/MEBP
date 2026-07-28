# MEBP v7.5.x — Rosette tab: no-scan live-feed sub-well centre selection

## Objective

On the **Calibration → Rosettes** tab, add the option to **skip the mosaic
scan** and instead **pick each sub-well centre directly from the live
microscope feed**. The operator jogs each sub-well under the crosshair, clicks
its centre in the live view, and confirms — recording that sub-well's
calibrated stage position, no mosaic involved.

Operator request: *"on the rosette calibration page, add the option to not do a
mosaic scan, and instead just use the live feed to select the center of each
sub well."*

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | New **"Select centers from live feed (no scan)"** button + a hidden **"Live-feed sub-well centering"** control box (hint + Confirm & Next / Skip well / Finish-Cancel) on the Rosettes tab; the Rosettes live view's `clicked` signal now routes to a new no-scan flow; flow methods added. Intro text updated. |
| `tests/test_v75x_rosette_tab_auto_reanchor.py` | New `TestRosetteLiveSelect` (3 tests). |

## Design

- **Independent flow state** (separate from the Plate-Location `_ploc_running`
  state machine, so no conflict): `_rosette_live_active`, `_rosette_live_parent`,
  `_rosette_live_subwells`, `_rosette_live_idx`, `_rosette_live_results`,
  `_rosette_live_pending`.
- **Same click → stage-µm pipeline** as the manual click-rim flow:
  `CameraManager.pixel_to_stage_offset(cam_idx, px, py, w, h)` (µm/px + rotation
  + mirror already baked in) added to `controller.get_xy_position(cached=False)`
  (absolute stage µm, the same frame as `_predict_well_xy` /
  `move_xy_absolute_um`). No new coordinate math.
- **Per-sub-well loop:** travel (retract-first via `_ploc_safe_goto`) to the
  sub-well's predicted centre so the operator starts near it → operator jogs it
  under the crosshair and clicks → the click is captured as `pending`
  (overlay ✓, re-pickable) → **Confirm & Next** records it, **Skip well**
  advances without recording. Confirming the last sub-well auto-finishes.
- **Merge = identical to the mosaic sub-well mapping**
  (`_ploc_open_single_well_mapping`): each confirmed centre updates
  `_calibrated_positions[name]`, `_ploc_well_results[name]`, and
  `_reference_markers[name]` (permanent marker on the plate + microscope views),
  then `_emit_calibration_data_changed()` (debounced autosave + view refresh).
  Finish also calls `_save_calibration()` explicitly.
- **Gates:** microscope µm/px calibrated (`is_um_per_px_calibrated`); and — when
  ZP is connected — a Safe (Fast-Move) Z must be set so the inter-sub-well
  retract-first hop has a known safe height (mirrors the Plate-Location /
  Needle-Location run gate).
- **Safety:** every inter-sub-well hop is a cross-position XY move → routed
  through `_ploc_safe_goto` → `safe_travel_to` (retract → wait → XY, never
  descends). The operator's manual jog to fine-position is exempt per the
  standard jog exemption. Travel-to-predicted is best-effort; if no prediction
  is available it logs and the operator jogs manually.

## Implementation Steps

- [x] Add the **no-scan** button + hidden control box to `_build_rosette_tab`.
- [x] Connect `_rosette_live_view.clicked → _rosette_on_live_click`.
- [x] `_rosette_start_live_select` (gates + state init + travel to first).
- [x] `_rosette_live_goto_current` (robust sub-well position resolve: calibrated
      → predicted → `_predict_well_xy`; retract-first travel).
- [x] `_rosette_on_live_click` (capture pending centre), `_rosette_live_update_hint`.
- [x] `_rosette_live_confirm_next` / `_rosette_live_skip_well` /
      `_rosette_live_finish` (merge + persist + restore buttons + summary).
- [x] Update intro text.
- [x] Tests.

## Testing Notes

- `tests/test_v75x_rosette_tab_auto_reanchor.py::TestRosetteLiveSelect`:
  full confirm/skip flow (click → pending → confirm merges into
  `_calibrated_positions`/`_reference_markers`, skip advances, last-confirm
  auto-finishes), inactive-click no-op, µm/px gate.
- Full `test_v75x_rosette_tab_auto_reanchor` (46) + reanchor + mapping suites
  green.

**Needs real-HW/GUI verification on ME3B V1** (start the no-scan flow on a
rosette well → stage retracts and hops to each sub-well's predicted centre →
jog + click centre + Confirm → the sub-well shows a reference marker and the
saved position persists after restart).

## Issues & Decisions

- **Persistence caveat (pre-existing, unchanged):** `_save_calibration` writes
  `calibrated_positions` only when no plate warp exists (`warp is None`). This
  no-scan flow persists exactly the same way the existing mosaic sub-well
  mapping does (both update `_calibrated_positions`), so behaviour is
  consistent; no new gap introduced.
