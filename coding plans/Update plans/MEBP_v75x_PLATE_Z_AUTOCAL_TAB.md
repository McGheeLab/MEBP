# MEBP v7.5.x — Move plate Z auto-calibration to its own tab (after Plate Location)

## Objective

Split the Calibration → **Needle Offset Calibration** tab so the focus-based
per-well **Z-bottom auto-calibration** lives on a new **Plate Z Auto-Cal** tab,
positioned **after Plate Location**.

Rationale: the Z auto-cal drives to each calibration well, which requires the
finished **Plate Location** XY map. The needle's vertical *reference heights*
(Replace / Max / Fast-Move / Plate-Top / Plate-Bottom Z) must still be taught
*before* Plate Location (the plate cal retracts to Fast-Move/Safe Z between
wells), so they stay on the Needle Offset tab.

## New tab order

1. Needle Location
2. Needle Offset Calibration  *(Z reference heights + estimate-from-needle-cam + Go to Replace Z)*
3. Plate Location
4. **Plate Z Auto-Cal**  *(NEW — Step 1 first-spot + Step 2 auto-cal + live microscope feed)*
5. Custom

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | Split `_build_z_offset_tab` (keeps reference heights); new `_build_plate_z_autocal_tab` (Step 1/2 + live feed); register the new tab after Plate Location; `_zauto_tab_index` + camera-start trigger moved to the new tab. |
| `tests/test_v75x_plate_z_autocal_tab.py` | New tests. |

## Design

- The moved widgets keep their **existing attribute names**
  (`_zauto_btn_goto_first`, `_zauto_btn_record_first`, `_zauto_lbl_first`,
  `_zoff_btn_run_z`, `_zoff_btn_cancel_z`, `_zoff_lbl_auto_z`,
  `_zoff_live_view`) so every auto-cal handler
  (`_zauto_goto_first_well`, `_start_auto_z_cal`, `_auto_z_set_progress`,
  `_auto_z_set_running`, …) works unchanged. `_auto_z_set_*` already iterate
  attribute names with `getattr(..., None)` and skip missing ones.
- The **live microscope feed** (`_zoff_live_view`) moves to the new tab (it's
  only used for the first-spot focus teaching / auto-cal). `_zoff_ensure_live_camera`
  is unchanged (same attribute). `_refresh_ploc_view`'s `_zoff_live_view` rebind
  is `getattr`-guarded, so it keeps working.
- `_on_workflow_tab_changed` now starts the live camera when the **Plate Z
  Auto-Cal** tab (`_zauto_tab_index`) is shown (that's where the live view now
  lives), instead of the Needle Offset tab.
- The Needle Offset tab loses its vertical splitter/live-feed (now just a
  scrollable controls column) + gains a pointer note to the new tab.

## Implementation Steps

- [x] Reword `_build_z_offset_tab` docstring/intro; drop the splitter + live feed.
- [x] Append a pointer note + scroll wrap; `return page`.
- [x] New `_build_plate_z_autocal_tab` with Step 1 + Step 2 + live feed.
- [x] Register the new tab after Plate Location; set `_zauto_tab_index`.
- [x] Point `_on_workflow_tab_changed` at `_zauto_tab_index`.
- [x] Tests — `tests/test_v75x_plate_z_autocal_tab.py` (8, real offscreen page
      build): tab order, indices, widget placement (reference heights on Needle
      Offset; auto-cal + live feed under the new tab), live-camera trigger.
- [ ] Real-HW verification on ME3B V1.

## Testing Notes

- Offscreen page build: 5 workflow tabs in the new order; tab 3 titled
  "Plate Z Auto-Cal"; the auto-cal widgets + `_zoff_live_view` exist; the
  reference-height widgets stay on tab 1.
- `_on_workflow_tab_changed(_zauto_tab_index)` starts the live camera.

## Issues & Decisions

- New tab placed **between Plate Location and Custom** (end of the guided
  workflow; Custom stays last as the advanced/legacy catch-all).
- No motion/coordinate math changed — pure UI relocation.
- **Needs real-HW verification on ME3B V1.**
