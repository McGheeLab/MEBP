# MEBP v7.5.x — Single-well mosaics, rosette sub-well calibration, guided well-mapping UX, orientation-aware mosaic presentation

## Objective

Three operator requests on Calibration → Plate Location:

1. **Single-well mosaics as a first-class tool** — scan ANY single well (not just rosette parents);
   for a rosette well, **choose the rosette centre** to calibrate the sub-well locations
   (`A1.a`, `A1.b`, …); and use the picked single-well centres to **re-register the whole plate**
   (operator decision: 1 well = translation shift; 2 = translation + rotation; 3+ = affine).
2. **Guided mapping UX** (full-plate AND single-well "Map wells"): zoom + pan the mosaic; two
   **toggleable selection modes** — (a) click 3+ points on the well edge → circle fit, (b) one
   click at the centre + draggable marker; and the user must **confirm each well** before the next
   starts. Rosette flow (operator decision): place the centre once → sub-wells auto-place from the
   rosette design geometry → drag any to refine → confirm.
3. **Orientation-aware presentation** — the mapping dialog showed the mosaic RAW (stage frame)
   while every other view flips it 180° per `plate_flip_180` → the user saw the map upside-down.
   Fixed with the **plate orientation convention** (`StageController.plate_flip_180()`), NOT the
   camera's calibrated `rotation_deg` (mosaics are placed by stage motion, not camera frames — per
   the operator's explicit instruction).

## Design

### Guided, orientation-aware mapping dialog (`gui/dialogs/mosaic_well_mapping_dialog.py`, reworked)
- **180° VIEW rotation** (`_MappingView.set_flip_180` / `reset_view_transform`; new ctor param
  `plate_flip_180`): `QGraphicsView.rotate(180)` — display-only. `mapToScene` still returns raw
  scene px, so clicks, drags, the px→µm back-projection (`s = extent[:2] + scene_px/scale`) and the
  `WellTrainingStore` labels are **byte-identical** to before (locked by a test that runs the same
  click sequence flipped/unflipped and asserts identical results). `_fit()` re-applies the rotation
  (`fitInView` composes onto it). Text labels stay upright via `ItemIgnoresTransformations`;
  flip-aware label anchor. Corner prompts are orientation-INVARIANT ("«A1» — top-left in this
  view"): on a flipped machine A1 sits at stage-max → raw bottom-right → rotated top-left; on an
  unflipped machine A1 is at stage-min = raw top-left.
- **State machine**: `_Phase` (IDLE / CORNERS / REFINE / ROSETTE_CENTER / ROSETTE_REFINE) +
  `_SelMode` (CENTER / EDGE) + `_Marker` records (circle/label/state/r_px; active=yellow,
  placed=blue, confirmed=green **locked**). Per-well row: **Confirm well / Redo / Skip / Clear
  points**; current-well banner with progress; guided queue (`_advance_queue` skips confirmed).
- **CENTER mode**: click places the circle at the nominal radius, draggable; click on a marker
  passes through to drag (itemAt movable check). **EDGE mode** (`click_takes_all`): each click adds
  a rim point → live `fit_circle_to_points` (Kåsa) preview; ±50% radius sanity vs nominal
  (`_RADIUS_TOL`, skipped when nominal unknown — `_nominal_radius_px` returns 0, no 3 mm fallback);
  red-dashed rejected preview; Confirm disabled until valid.
- **Plate flow**: confirm the 3 corners (either mode) → existing `solve_affine_3` autofill
  (corners keep their confirmed markers; affine passes exactly through them) → REFINE queue over
  the remaining wells (Skip keeps the auto placement, still exported) → global
  "Confirm all + calibrate + save" (gate ≥3 markers). `_auto_detect` (unchanged detection) now
  pre-places all markers → straight to REFINE (fast path preserved).
- **Rosette / single-well flow** (`rosette=True`, `predicted_um=`, `center_label=`): queue starts
  with the pattern CENTRE (sentinel `_CENTER_KEY`, kept out of `_well_items` → never exported);
  on centre confirm, sub-wells auto-place at `centre_px + (predicted_um − centroid) × scale`
  (rotation + `plate_axis_sign` already baked into the predicted stage µm; fallback: plate-local mm
  × sign-from-flip); then drag/confirm each. Gate: centre confirmed + ≥1 marker.
- `results()` / `saved_sample_path()` contracts unchanged.

### Single-well scan generalized + persisted (`gui/pages/calibration.py`, `SupportClasses/MosaicStore.py`)
- `_ploc_scan_rosette_well` → **`_ploc_scan_well`**: picker lists all non-sub-wells + rosette
  parents (annotated "(rosette)" / "(scanned)"); rosette → existing `_ploc_subwell_scan_bounds`;
  plain well → new `_ploc_well_scan_bounds` (centre ± radius + 1.5 mm; positions already
  orientation-correct stage µm). Routes via `_ploc_scan_well_is_rosette`.
- Finish path persists the composite under **`f"{plate_key}#{well}"`** in the same `MosaicStore`
  (`_ploc_apply_single_well_mosaic` — never touches the plate mosaic/overlay; verified nothing
  enumerates store keys assuming plate keys). New store helpers `list_well_keys` / `has_well`.
- `_ploc_open_subwell_mapping` → **`_ploc_open_single_well_mapping(parent, …, is_rosette)`**:
  `_SubPlate` (rosette) or new `_SingleWell` adapter (plain); passes `plate_flip_180` +
  `rosette` + `predicted_um` + `center_label`; merges the measured centres (sub-wells AND the
  parent — centroid fallback when the dialog doesn't return it) into `_calibrated_positions` /
  `_reference_markers`; records the re-registration anchor.
- `_ploc_open_well_mapping` (full plate) passes `plate_flip_180` too (new `_ploc_plate_flip_180`
  resolver: controller → `JogWorkspaceView._FLIP_DISPLAY_180` fallback, same as the preview).

### Re-register from scanned wells
- Session-only `_ploc_well_anchors`: well → `{measured, map (at pick, BEFORE the merge), merged
  (ALL centres merged from that pick — ground-truth pins)}`. New button **"Re-register from
  scanned wells (N)"** (`_ploc_refresh_scanned_reregister_button` tracks count/enabled).
- **1 anchor** → `E = measured − map` → `_ploc_apply_global_translation(E, pin=pins)` (wells +
  markers + warp + mosaic overlay shift together, persisted). **2+** →
  `register_from_template(fit_template, measured)` (anchors' template entries reset to their
  map-at-pick values; similarity for 2, affine for 3+) → plausibility gate (`_warp_is_plausible`,
  confirm-to-override) → commit with `_plate_warp = None` + `_three_well_calibration = None` +
  explicit `_save_calibration()` (explicit ground-truth positions round-trip, same contract as the
  orientation-remap flow) → mosaic follows the fit's **translation component only**
  (`_ploc_shift_mosaic_for_warp`: `warp.transform(extent centre) − centre` →
  `_ploc_shift_mosaic_by`; noted visibly as an approximation).
- **Anti-double-correction pins** (self-review finding, HIGH): at pick time the measured centres
  are merged into the map, so a later shift/fit would move them AGAIN (anchor well → measured+E).
  The anchor's `merged` dict pins every measured centre verbatim after the shift/fit;
  `_ploc_apply_global_translation` gained an optional `pin=` param that folds the pins into the
  warp rebuild (exact interpolation → survives reload) and into the shifted reference markers.
- **Anchor lifecycle**: cleared after every successful re-register, on plate/type switch
  (`_reset_calibration_state`), and after ANY other map-moving operation (re-anchor via
  `_ploc_apply_global_translation`, quick re-register, re-derive) — their recorded map frame is
  stale the moment the map moves.

### Hardening found by self-review
- **Stale single-well flags on a gated start** (pre-existing, amplified): the scan-start gates
  (Safe-Z / camera / µm-px / frames) returned early WITHOUT clearing
  `_ploc_scan_bounds_override`/`_ploc_scan_subwell_parent` → a later full-plate scan would
  mis-route into the single-well path. `_ploc_start_mosaic_scan_impl` now snapshots + clears the
  flags at the top and re-arms them only when the worker actually launches; `_ploc_scan_well` also
  gates on `_ploc_running`.
- **Pre-existing latent bug fixed**: `_ploc_quick_reregister` called
  `not self._warp_is_plausible(warp)` but that method returns `tuple[bool, str]` → the
  plausibility guard NEVER fired. Now unpacked (`ok_fit, why = …`).

## Files Modified

| File | Change |
|------|--------|
| `gui/dialogs/mosaic_well_mapping_dialog.py` | Full rework (view rotation, `_Phase`/`_SelMode`/`_Marker`, per-well confirm, EDGE circle fit, rosette centre flow, new ctor params). Module fns (`solve_affine_3`/`apply_affine`/`corner_well_names`) unchanged. |
| `gui/pages/calibration.py` | `_ploc_scan_well` + `_ploc_well_scan_bounds` + `_SingleWell`; `_ploc_apply_single_well_mosaic` + `_ploc_open_single_well_mapping`; `_ploc_plate_flip_180`; anchors + button + `_ploc_reregister_from_scanned` + `_ploc_shift_mosaic_for_warp` + `_ploc_clear_well_anchors`; `_ploc_apply_global_translation(pin=)`; scan-start flag snapshot/re-arm; anchor clears on re-anchor/quick-reregister/re-derive/plate-switch; quick-reregister tuple fix; call-site kwargs. |
| `SupportClasses/MosaicStore.py` | `list_well_keys` / `has_well` (single-well keys `plate#well`). |
| `tests/test_v75x_single_well_mosaic_reregister.py` | New — 26 tests (store keys; dialog flip-invariance/queue/edge-fit/rosette; finish-path persistence; anchors incl. double-correction regressions; 1/2/3-anchor re-register; mosaic translation shift; implausible-fit prompt; lifecycle). |
| `tests/test_v75x_plate_mosaic.py` | Corners test updated for the per-well confirm (one `_on_confirm_well()` per corner click). |
| `quickstart_guide/guide_content.json` | Map wells / Scan well / re-register documentation. |

## Implementation Steps

- [x] Dialog: view rotation + ctor params (`plate_flip_180`, `rosette`, `predicted_um`, `center_label`).
- [x] Dialog: marker model + guided queue + Confirm/Redo/Skip + banner + mode toggle.
- [x] Dialog: EDGE mode (live Kåsa fit + radius sanity + clear).
- [x] Dialog: rosette centre → sub-well auto-placement + refine.
- [x] MosaicStore `list_well_keys`/`has_well`.
- [x] `_ploc_scan_well` (any well) + bounds + separate persistence + generalized opener + `_SingleWell`.
- [x] Anchors + "Re-register from scanned wells" (1 = translation, 2+ = similarity/affine) + pins.
- [x] Self-review fixes (double-correction pins, stale-flag snapshot/re-arm, anchor invalidation, tuple bug).
- [x] Tests (26 new + 1 updated; 6 affected suites green).
- [x] Docs (this plan, CLAUDE.md row, quickstart guide).
- [ ] **Real-HW / GUI verification on ME3B V1.**

## Testing Notes

- `python -m unittest tests.test_v75x_single_well_mosaic_reregister` → **26/26 OK**.
- Affected suites together (`test_v75x_plate_mosaic`, `test_v75x_mosaic_orientation_remap`,
  `test_v75x_reanchor_mosaic_and_camera_orientation`, `test_v75x_plate_location_manual_click_rim`,
  `test_v75x_plate_orientation_convention`) → **220/221**; the 1 failure is the **documented
  pre-existing CV** `test_real_24_well_mosaic`.
- Adversarial-review workflow was launched but most agents were cut off by the org's monthly spend
  limit — replaced by a manual self-review over the same question set, which found + fixed the
  double-correction and stale-flag issues above.

**Real-HW verification (ME3B V1) — REQUIRED, pending:**
1. Mosaic scan → **Map wells…** → the mosaic displays the SAME orientation as the plate overlay
   (A1 top-left); guided corners work in both selection modes (edge-points fit + centre-click
   drag); per-well Confirm/Redo/Skip behave; results land on the right wells.
2. **Scan well…** on a rosette well → place the pattern centre → sub-wells auto-place from the
   design → drag one → Confirm all → `A1.a…` (+ parent) navigable; the single-well mosaic persists
   (restart → `config/hardware/mosaics/<plate>_<well>.png` + it shows "(scanned)" in the picker).
3. **Scan well…** on a plain well → pick its centre → "Re-register from scanned wells (1)" shifts
   the mosaic + wells together, the scanned well stays EXACTLY at its measured centre.
4. Scan 2–3 wells → re-register upgrades to rotation/affine; the note about the translation-only
   mosaic shift appears; restart → everything persists.

## Issues & Decisions

- **Orientation = plate convention, not camera rotation** (operator explicit): mosaics are placed
  by trusted stage positions; `rotation_deg` corrects the live-view click mapping only. The
  mapping-dialog rotation is a pure VIEW transform — zero coordinate changes, locked by test.
- **Rosette centre = centroid of the sub-well pattern** (the parent well is dropped at plate
  compile, so its centre isn't in the predicted map; the centroid == bore centre for symmetric
  rosettes and per-marker drag absorbs asymmetry). The parent's measured centre IS stored into the
  calibration (centroid of measured sub-wells when not returned by the dialog).
- **Double-correction** (found in self-review): merged measured centres must be pinned through a
  later re-register — otherwise the anchor well lands at `measured + E`. Fixed via anchor `merged`
  pins + `_ploc_apply_global_translation(pin=)` (pins folded into the warp rebuild → reload-safe).
- **Mosaic under a 2+ fit** follows the translation component only (a bitmap can't cheaply
  rotate/scale to an affine fit); labelled visibly. A re-scan gives the exact image when needed.
- **Anchors are session-only** and invalidated by ANY map-moving operation — they are only
  meaningful against the map frame they were measured in.
