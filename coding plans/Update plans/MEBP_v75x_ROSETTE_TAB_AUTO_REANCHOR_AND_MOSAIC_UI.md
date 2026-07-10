# MEBP v7.5.x — Rosettes tab, auto re-anchor, 4-button mosaic UI, sub-well mapping bug fix, mosaic view fixes

## Objective

Operator-feedback batch after HW-testing the single-well-mosaic / guided-mapping feature:

1. Plate-level mapping uses **MAIN wells only** (rosette sub-wells get their own section).
1.1 Plate-calibration options reduced to **FOUR buttons**: *Calibrate mosaic scan* · *Mosaic scan* ·
   *Re-anchor mosaic* · *Auto re-anchor mosaic* (everything else → one **Advanced… menu**; the
   Map-wells dialog **auto-opens** after every full-plate scan — operator decisions).
1.2 **NEW hands-free "Auto re-anchor mosaic"**: the manual re-anchor's live-view click auto-saves a
   feature patch; the auto flow travels to its last position, template-matches it in the live frame,
   and shifts the whole map (operator: travel + match, one click).
2. **New "Rosettes" tab** right after Plate Location: mosaic-scan a rosette-assigned well + map its
   sub-wells (also re-mappable from the SAVED scan, no re-scan).
3. Saved **single-well mosaics overlay the full plate mosaic** (toggle on Plate Location + Jog).
4. **BUG FIX — rosette sub-wells mis-mapped**: the mapping dialog back-projected clicks with the
   **shift-including** `canvas_extent_um` (`MosaicBuilder._global_shift_um`, seeded from
   `MosaicAlignmentStore` and/or measured during stitching), whereas the detection path removes it
   (`_ploc_fit_from_mosaic_detections` via `_ploc_mosaic_world_shift`). Every mapped centre was
   offset by the registration shift — the single-well path AND the full-plate "Map wells…" path
   (pre-existing there).
5. **"Mosaic + well" view** = mosaic background + **outline-only circles** at well locations
   (including calibrated rosette sub-wells) — no plate body / fills over the image.

## Design

### Bug fix (shift-free back-projection)
- `MosaicStore.save(..., shift_um=)` records the registration shift baked into the saved extent;
  `get_shift_um(key)` (legacy → (0,0)); `update_extent(key, ext)` = metadata-only translate.
- Persist sites pass the live shift: `_ploc_apply_mosaic` + `_ploc_apply_single_well_mosaic`
  (captured from `_ploc_mosaic_world_shift()` BEFORE the builder is released in the finish path);
  `_ploc_shift_mosaic_by` preserves the stored `shift_um` on its re-save.
- The mapping dialog receives the **trusted-stage extent** (`extent − shift`) everywhere:
  `_ploc_open_well_mapping` (subtracts `store.get_shift_um`), `_ploc_open_single_well_mapping`
  (new `shift_um=` param), and the Rosettes-tab saved-scan re-open. Dialog math unchanged; zero
  shift ⇒ byte-identical (regression-locked).

### Main wells only (plate mapping)
- `mosaic_well_mapping_dialog.main_well_names(plate)`: non-sub-well names + dropped rosette PARENTS
  re-added; parents' nominal position = sub-well centroid (`_nominal_position`).
  `corner_well_names(plate, names=, positions=)` gained overrides. `_autofill` / the REFINE queue /
  `_auto_detect`'s grid names all use `_map_names` (sub-wells excluded; a rosette cell's
  letter-fallback name = the parent). Rosette mode (`rosette=True`) unchanged.

### Four buttons + Advanced menu
- Mosaic group = `Calibrate mosaic scan` (`_ploc_quick_mosaic_calibrate`) · `Mosaic scan`
  (`_ploc_start_mosaic_scan`; the FULL-plate finish now **auto-opens** `_ploc_open_well_mapping`) ·
  `Re-anchor mosaic` (`_ploc_toggle_reanchor`) · `Auto re-anchor mosaic` (`_ploc_auto_reanchor`) +
  "Plate view" combo + "Well scans" toggle + **Advanced… ▾** menu.
- Advanced menu = Settings… / Map wells… / Quick re-register / Scan well… / Re-register from
  scanned wells (N) / Re-derive from saved mosaic / Manual align (checkable → shows the hidden
  slider group). Legacy button ATTRS stay alive as hidden buttons — the menu actions `.click()`
  them (disabled = no-op) and `_ploc_sync_advanced_menu` mirrors live text + enabled state
  (`aboutToShow`), so every existing handler/refresh/test keeps working.

### Auto re-anchor
- **`SupportClasses/ReanchorFeatureStore.py`** (new; JSON + PNG per plate key): feature patch +
  `stage_um` + capture `um_per_px` + `camobj`. `set_stage_um` keeps the position in the MAP frame:
  `_ploc_apply_global_translation` shifts it by E; the 2+ scanned-well re-register transforms it
  via `warp.transform`.
- **Capture**: `_ploc_reanchor_live_click` → `_ploc_save_reanchor_feature` crops ~¼-frame-width
  around the click (raw frame), stores the PATCH-CENTRE's absolute µm (same rotation-aware
  `pixel_to_stage_offset` as the click) + live-resolved µm/px. Best-effort.
- **`VisionDetector.find_template(frame, patch)`** (new): grayscale `cv2.matchTemplate`
  TM_CCOEFF_NORMED → `(cx, cy, confidence)`.
- **`_AutoReanchorWorker(QThread)`**: poller-suspended; `safe_travel_to` the stored position
  (retract-first, never lowers) → `capture_fresh_frame` → `_match` (patch rescaled by
  stored/live µm/px when >2% apart; confidence gate 0.5) → rotation-aware pixel→stage → emits the
  feature's ACTUAL absolute µm. GUI side (`_ploc_on_auto_reanchor_done`): `E = actual − stored` →
  >30 mm sanity confirm → `_ploc_apply_global_translation(E)` (wells + markers + warp + mosaic +
  stored feature move together). Gates: XY connected, Safe-Z when ZP connected, camera running +
  µm/px calibrated, feature saved, map exists. Camera/objective mismatch warns + proceeds
  (confidence decides).

### Rosettes tab
- Inserted after Plate Location → index 3; `_rosette_tab_index = 3`, **`_zauto_tab_index` 3 → 4**
  (the only hard-coded index; Pump Compliance stays 1). `_on_workflow_tab_changed` starts the
  tab's own live feed (`_rosette_live_view`) + refreshes the picker.
- Controls: rosette-well combo (from `_ploc_rosette_parent_wells`, "(scanned)" annotated via
  `MosaicStore.list_well_keys`) · **Scan rosette well** (`_ploc_scan_single_well(name, True)` —
  factored out of `_ploc_scan_well`, which now just picks + delegates) · **Map sub-wells from saved
  scan…** (re-opens the mapping from the persisted `plate#well` mosaic + its stored shift/µm-px —
  no re-scan, no motion) · per-well scanned/calibrated status.

### Single-well overlay composite
- `MosaicStore.composite_with_wells(store, plate_key, plate_img, plate_extent)` pastes each
  `plate#well` image into the plate canvas at its extent (proportional source-crop when clipped;
  best-effort per well; input never mutated).
- **Toggles** (default ON): `_ploc_wellscan_check` (Plate Location, next to the view combo) and
  `_wellscan_check` (Jog mosaic row). Loaders composite at the VIEW push only.
- **Purity guard**: `_ploc_overlay_img/_ext` keep the PLAIN plate mosaic (what
  `_ploc_shift_mosaic_by` re-persists); the composite exists only in the pushed pixmap.
  `_ploc_shift_mosaic_by` also translates every `plate#well` extent (metadata-only
  `update_extent`) so the well scans move with the plate as one unit.
- After a single-well scan persists: plate overlay reloads + `calibration_data_changed` emits
  (Jog re-reads) + the Rosettes picker refreshes.

### Overlay view = circles only
- `JogWorkspaceView.set_plate_display_mode` stores the mode; in `"overlay"`: no plate body, and
  `_paint_well` draws **outline-only** rings (green calibrated / yellow approximate / blue current,
  no fills/halo). Sub-wells render automatically (positions dicts carry them;
  `_well_radius_um(name)` resolves their per-well diameters). `"well"` mode byte-identical.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/MosaicStore.py` | `save(shift_um=)` + `get_shift_um` + `update_extent`; `composite_with_wells`. |
| `SupportClasses/ReanchorFeatureStore.py` | New — per-plate feature patch + map-frame position. |
| `SupportClasses/VisionDetector.py` | `find_template` (normalized cross-correlation). |
| `gui/dialogs/mosaic_well_mapping_dialog.py` | `main_well_names` + `corner_well_names(names=, positions=)` + `_nominal_position`; plate flows use `_map_names` (sub-wells excluded). |
| `gui/pages/calibration.py` | Shift plumbing (persist + subtract); 4-button group + Advanced menu + hidden legacy buttons + `_ploc_sync_advanced_menu`; auto-open mapping after full scan; feature capture + `_AutoReanchorWorker` + `_ploc_auto_reanchor` + handlers + button gate; feature-position tracking in translation/warp paths; Rosettes tab (`_build_rosette_tab` + 5 `_rosette_*` methods) + `_rosette_tab_index=3` + `_zauto_tab_index=4`; `_ploc_scan_single_well` factor-out; well-scan composite at view push + toggle + purity guard + well-extent shift; scan-finish refresh/emit. |
| `gui/pages/jog_control.py` | "Well scans" toggle + composite in `_load_mosaic_overlay`. |
| `gui/widgets/jog_workspace_view.py` | `_plate_display_mode`; overlay = outline-only circles, no plate body. |
| `tests/test_v75x_rosette_tab_auto_reanchor.py` | New — 28 tests. |
| `quickstart_guide/guide_content.json` | Updated control entries (4 buttons, Advanced menu, Rosettes tab, auto re-anchor, well-scan toggle). |

## Implementation Steps

- [x] Bug fix: shift metadata + unshifted-extent handoff (all 3 mapping paths).
- [x] Plate mapping = main wells only (+ rosette-parent centroid nominals).
- [x] ReanchorFeatureStore + find_template.
- [x] Feature capture on manual re-anchor + map-frame tracking + auto re-anchor worker/handlers.
- [x] Four-button group + Advanced menu + auto-open mapping after scan.
- [x] Rosettes tab + indices + `_ploc_scan_single_well` + saved-scan re-open.
- [x] Well-scan composite + toggles + purity guard + well-extent shift.
- [x] Overlay mode = outline circles.
- [x] Tests (28 new; affected suites green).
- [x] Docs (this plan, CLAUDE.md row, quickstart guide).
- [ ] **Real-HW / GUI verification on ME3B V1.**

## Testing Notes

- `python -m unittest tests.test_v75x_rosette_tab_auto_reanchor` → **28/28 OK** (store shift/extent/
  composite; feature store; template match; main-well filtering; shift-free dialog handoff incl.
  zero-shift regression; 4-button group + menu sync; auto-open after scan; worker match math;
  feature-position tracking; tab indices; overlay render smoke).
- Affected suites (`test_v75x_plate_mosaic`, `test_v75x_single_well_mosaic_reregister`,
  `test_v75x_reanchor_mosaic_and_camera_orientation`, `test_v75x_mosaic_orientation_remap`,
  `test_v75x_plate_location_manual_click_rim`) green — the 1 standing failure is the documented
  pre-existing CV `test_real_24_well_mosaic`.
- **Previously-saved single-well/plate mosaics carry unknown baked-in shifts** (saved before
  `shift_um` was recorded → treated as 0). If a pre-fix mosaic maps wells with a visible constant
  offset, re-run the scan (fresh saves record the true shift).

**Real-HW verification (ME3B V1) — REQUIRED, pending:**
1. Mosaic scan → mapping auto-opens, MAIN wells only (no `A1.a` markers), positions land correctly
   (shift bug fixed).
2. Rosettes tab → scan a rosette well → place centre → refine → confirm → sub-wells land on the
   correct plate positions; "Map sub-wells from saved scan…" re-opens without motion.
3. Re-anchor mosaic (manual) → feature auto-saved → "Auto re-anchor mosaic" travels, matches,
   shifts mosaic+wells; repeat after nudging the plate slightly.
4. "Well scans" toggle shows the high-res single-well patches on the plate mosaic (Plate Location +
   Jog); "Mosaic + well" shows outline circles only, incl. calibrated sub-wells.

## Issues & Decisions

- **Shift-frame root cause** confirmed: `canvas_extent_um` includes `_global_shift_um` (display
  registration) while tile pixels sit at trusted raw stage positions; the dialog previously
  back-projected without removing it → every mapped centre offset by the registration shift (only
  when a shift existed — seeded or measured during the scan; the pre-existing full-plate path had
  the same defect).
- **Recording the shift in MosaicStore meta** beats subtracting a *current* live/alignment value
  for store-loaded mosaics: the extent's baked-in shift is whatever it was AT SAVE TIME.
- **Auto re-anchor reference** = the stored feature position, kept in the map frame by shifting it
  with every map translation/warp — after any correction, stored == where the map believes the
  feature to be, so `E = actual − stored` is exactly the remaining map error.
- **Advanced menu mirrors hidden buttons** rather than re-homing handlers — zero behavioural churn,
  all enable/label logic (e.g. "(N)" anchor count, template-gated Quick re-register) flows through.
- Concurrent-session note: implemented alongside another session's edits to calibration.py
  (Pump Compliance tab, quick-reregister button refresh) — reconciled; tab indices account for it.

---

## Addendum (2026-07-09) — operator follow-ups

Four further operator requests, landed together:

1. **Mosaic buttons are VERTICAL and "Map wells…" is promoted onto the list.** The Mosaic group
   layout is now a single-column `QVBoxLayout`: *Calibrate mosaic scan · Mosaic scan · Map wells… ·
   Re-anchor mosaic · Auto re-anchor mosaic*, then the Plate-view row and *Advanced… ▾*.
   `_ploc_btn_map_wells` is no longer a hidden button and no longer mirrored in the Advanced menu;
   all its wiring is unchanged.
2. **Rosette PARENTS always fit with the plate's NORMAL well diameter.** The mapping dialog gained
   `_fit_diameter_mm(name)` (used by both `_well_radius_px` and the edge-fit sanity check
   `_nominal_radius_px`): a name with a real `WellInfo` keeps its per-well diameter (sub-well names
   in the Rosettes flow are untouched), but a rosette PARENT (dropped at plate compile → no
   WellInfo) resolves to `_main_well_diameter_mm()` = the median diameter of the plate's non-sub
   wells (fallback: the uniform `well_diameter` attribute). Previously a custom rosette plate
   (`well_diameter = 0.0` "varies") sent parents to the 3 mm unknown fallback — a tiny circle and a
   skipped sanity check.
3. **"Load mosaic from another plate…" (Advanced menu).** New `MosaicStore.list_plate_keys()` +
   `CalibrationPage._ploc_copy_mosaic_key(store, src, dst)` copy the plate mosaic (image, extent,
   µm/px, scale, `shift_um`, and all its `src#well` single-well scans) under the current plate key
   — the source stays. Gated by a source picker (`QInputDialog`, entries annotated with
   date/tiles/well-scans) and an overwrite confirm; refreshes the overlay, Jog page, and rosette
   picker.
4. **The well mapping travels WITH the mosaic** (operator: "when we load in any mosaic, the last
   good well mapping should load with it"). New `MosaicStore.set_wells`/`get_wells` (meta field
   `wells_um`, name → absolute stage µm). Written on every Map-wells confirm
   (`_ploc_open_well_mapping`) and re-derive; `save()` intentionally REBUILDS the meta and drops
   it (a fresh scan needs a fresh mapping), so the two same-image re-save paths carry it across
   explicitly: `_ploc_shift_mosaic_by` re-stores it SHIFTED by the same E (mapping + mosaic move
   as one), and `_ploc_copy_mosaic_key` copies it. On "Load mosaic from another plate…" the stored
   mapping is applied automatically via `_ploc_apply_stored_mosaic_wells` (the re-derive direct-
   commit pattern: clear taught/warp state, store positions directly, filter names against
   `main_well_names(plate)`, set `_taught_a1`, save + emit); no layout match → not applied, hint
   says to run Map wells… once. Mappings confirmed BEFORE this addendum aren't stored in the
   mosaic files yet — run Map wells… once per mosaic to seed them.

Tests (all in `tests/test_v75x_rosette_tab_auto_reanchor.py`, file now 43): vertical order +
promoted Map-wells/menu membership; `_fit_diameter_mm` parent/sub-well/plain cases; wells-meta
round-trip + fresh-save drop; copy carries mapping; load handler applies mapping (bogus names
filtered) + no-match leaves calibration untouched; `_ploc_shift_mosaic_by` translates the stored
mapping with extent + preserves `shift_um`; Map-wells confirm writes `set_wells`.
**Needs GUI/HW verification on ME3B V1** (vertical buttons; rosette parent circles at the normal
well Ø; load a saved mosaic onto another plate selection → mapping appears → re-anchor lines it
up).
