# MEBP v7.5.x — Fluorescence Mosaic Workflow (multi-channel single-well mosaic)

## Objective

Add a new **Fluorescence Mosaic** workflow that captures a high-resolution
stitched mosaic of a **single well**, once per fluorescence channel
(DAPI / FITC / mCherry / Cy5 …). The operator selects the objective and a well,
picks a display pseudo-colour per channel, and the workflow rasters a full
single-well mosaic for each channel — prompting the operator to physically
switch the filter/illumination between channels (there is **no filter-wheel
hardware**, confirmed by a repo-wide search). Channels are blended into a
false-colour overlay and **persisted per (plate, well)** so ANY other workflow
(Spheroid Pick & Place, Cell Targeting & Removal, Cell Labeling, the Jog plate
view) can show them as a registered background.

### Operator-confirmed scope decisions

- **Filter control = manual, one full mosaic per channel.** No filter-wheel/LED
  control exists; the workflow scans the whole well per channel, then a modal
  prompt asks the operator to switch the filter before the next channel.
- **Persistence = per (plate, well), multi-channel.** New machine-level store.
- **Overlay reuse = Spheroid, Cell Targeting, Cell Labeling, Jog plate view**
  (all share the `JogWorkspaceView` family). Quick Print was requested too but
  its preview widget (`PrintTrajectoryMonitorView`) is a separate auto-fitting
  view — deferred (see Issues & Decisions).

## Files Modified / Added

| File | Change |
|------|--------|
| `SupportClasses/FluorescenceMosaicStore.py` | **NEW** — zero-GUI store keyed by (plate, well). Per-channel PNG + metadata (colour RGB, absolute-µm extent, objective, µm/px, scale, frames). `save_channel` / `list_channels` / `list_wells` / `get_extent_um` / `channel_color` / `set_channel_color` / `load_channel_image` / `composite_overlay` (per-well false-colour blend) / `composite_plate_overlay` (all wells onto one union canvas) / `clear_*`. `CHANNELS` + `DEFAULT_CHANNEL_COLORS` + `default_color`. Atomic tmp+`os.replace`; images under `config/hardware/fluor_mosaics/`. |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | **NEW** — `FluorescenceMosaicWorkflowPage`. **Layout:** top row = objective combo (applies calibrated µm/px like the HW objective card) · **filter-cube toggle pills** (`_ChannelPill`, one per channel — click toggles inclusion, double-click sets the display colour, shows ✓ when captured) · selected-well label. Below = a 3-section resizable `QSplitter`: LEFT column (vertical split) is [top] `WellPlateNavigator` well selection + [bottom] live `CameraFeedView`; RIGHT is the **pan + scroll-to-zoom mosaic** (`_ZoomImageView`, a `QGraphicsView` with drag-pan / wheel-zoom / Fit). ⚙ Settings popout for scan knobs. Drives a per-channel sequence with a filter-switch prompt between channels (the blended/live composite updates the mosaic view, fitting on well change, preserving zoom mid-scan). `_SingleWellMosaicWorker(QThread)` mirrors `calibration._MosaicScanWorker` (raster → grab → stitch off the GUI thread; first tile `safe_travel_to`, rest pure XY; poller suspended) **minus** detection. Same raster grid reused for every channel → composites register. |
| `gui/pages/workflows/_fluorescence_overlay.py` | **NEW** — shared helper: `load_plate_fluor_overlay` / `load_well_fluor_overlay` (blend via store + push to a `JogWorkspaceView`-family view's fluor layer), `plate_key_of(hw_config)`, `has_any_fluor`. |
| `gui/widgets/jog_workspace_view.py` | Added an **independent fluorescence overlay layer** (separate from the brightfield plate mosaic): `_fluor_pixmap/_fluor_extent_abs/_fluor_visible/_fluor_opacity` + `set_fluor_overlay` / `set_fluor_visible` / `set_fluor_opacity` / `has_fluor` + `_paint_fluor_overlay` (drawn between the mosaic and the well grid, same zero-ref-µm + 180°-flip handling). Inherited by `WorkspaceTargetView`. |
| `gui/pages/workflows/workflow_picker.py` | New `fluorescence_mosaic` tile (🔬, enabled) after Cell Targeting. |
| `gui/pages/workflows_mode.py` | Import + route `fluorescence_mosaic` → `FluorescenceMosaicWorkflowPage`. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | "🔬 Fluorescence" header checkbox → `load_plate_fluor_overlay` on `self._workspace_view`; re-applied in `set_calibration_data`. |
| `gui/pages/workflows/cell_targeting_workflow.py` | Same toggle + re-apply. |
| `gui/pages/workflows/cell_labeling_workflow.py` | Same toggle + re-apply. |
| `gui/pages/jog_control.py` | "🔬 Fluorescence" checkbox beside the Plate-view combo → overlay for the active plate; re-applied in `set_calibration_data`. |
| `tests/test_v75x_fluorescence_mosaic.py` | **NEW** — 18 tests (store round-trip/blend/persistence/delete, overlay helper on a real `WorkspaceTargetView`, JogWorkspaceView fluor API, tile registration, offscreen page build). |

## Implementation Steps

- [x] Repo recon: confirmed NO filter/LED/illumination hardware; mapped MosaicBuilder / MosaicStore / `_MosaicScanWorker` / workflow registration / overlay APIs.
- [x] `FluorescenceMosaicStore` (per plate+well, per-channel images + blend).
- [x] `JogWorkspaceView` independent fluorescence overlay layer.
- [x] `_fluorescence_overlay` shared helper.
- [x] `FluorescenceMosaicWorkflowPage` + `_SingleWellMosaicWorker`.
- [x] Tile + routing.
- [x] Wire toggle into Spheroid / Cell Targeting / Cell Labeling / Jog.
- [x] Tests (18) + regression sweep.
- [ ] **Real-HW verification on ME3B V1** (manual filter switching; per-channel registration; overlay registers on the right well in the consumer workflows).
- [ ] (Follow-up) Quick Print overlay on `PrintTrajectoryMonitorView`.

## Testing Notes

- `tests/test_v75x_fluorescence_mosaic.py` — 18 green.
- Regression: `test_v731_jog_navigation`, `test_v75x_plate_mosaic` (1 **pre-existing**
  unrelated CV failure `test_real_24_well_mosaic` 23/24), `test_v75x_quick_print_workflow`,
  `test_v75x_spheroid_pick_place_z`, `test_v75x_cell_targeting_removal`,
  `test_v75x_cell_labeling`, `test_v75x_workflow_settings_popout` — all green.
- Bench checklist: pick objective → pick well → run; verify the filter-switch
  prompt appears per channel; verify the blended preview accumulates channels;
  toggle "🔬 Fluorescence" in Spheroid/Cell Targeting/Jog and confirm the overlay
  lands on the scanned well; restart and confirm persistence.

## Bug fixes & follow-ups (round 2)

Root-caused via a parallel investigate→adversarial-verify workflow (`fluor-mosaic-bug-rootcause`); both root causes verified `holds: true`.

- **Bug — mosaic never displayed (and never saved).** `_start_channel_scan`
  created a FRESH `MosaicBuilder` for the worker but `generate_raster_positions`
  (the only thing that calls `_init_composite`) had been run on a *throwaway*
  `tmpl` builder in `_on_start`. So the worker's builder kept
  `composite/_canvas_extent_um = None` → `stitch_incremental` no-ops → the live
  `tile` signal carried `None` AND `save_channel` was skipped (guarded on
  non-None composite/extent). **Fix:** call
  `builder.generate_raster_positions(self._scan_bounds, overlap=…)` on the SAME
  builder the worker uses (mirrors the Plate-Location scanner's one-builder
  pattern); stash `self._scan_bounds` in `_on_start`. Regression tests:
  `TestMosaicBuilderCanvasInit` (uninitialized→None vs initialized→composite) +
  `TestWorkerProducesComposite` (run the worker synchronously → non-None
  composite/extent emitted).
- **Bug — raster didn't cover the whole well.** `_on_start` sized the FOV from
  `effective_um_per_px`, which returns the **un-rescaled** base µm/px when the
  CameraManager has no resolution stamp (the normal startup state — the startup
  pushes in `calibration.py`/`hardware_setup.py` omit `resolution=`), so the FOV
  came out several-fold too large → far too few tiles (or a single-tile
  collapse). **Fix:** new `_microscope_um_per_px(frame_w, fallback)` resolves the
  µm/px from the **objective store** for the current objective and rescales by
  the LIVE frame width (mirrors `calibration._ploc_microscope_um_per_px`),
  independent of the manager stamp. Hardening: `_refresh_objectives` now fires
  `_on_objective_changed(current)` after un-blocking signals so the manager stamp
  is also correct app-wide. Regression: `test_microscope_um_per_px_rescales_by_live_width`,
  `test_compute_raster_plan_covers_well_many_tiles`.
- **Feature — raster grid preview.** New shared `_compute_raster_plan(well)`
  (used by BOTH the preview and `_on_start` so they always match) → bounds, tile
  centres, FOV, scale, cols×rows. `_refresh_grid_preview` draws the planned grid
  in TWO places: (1) the column-2 mosaic viewer (`_ZoomImageView.set_grid_preview`
  — well boundary circle + dashed tile footprints + centre dots, pan/zoomable),
  shown until a captured mosaic exists; and (2) **on the well selection** —
  `WellPlateNavigator.set_raster_grid(cols, rows)` overlays a cols×rows grid
  clipped to the selected well's circle (+ a yellow emphasis ring). Refreshed on
  well / objective / settings change, on `set_calibration_data` /
  `set_hardware_config`, and on `showEvent` (+ a 900 ms one-shot so it appears
  once the camera warms up). The status line reports `cols×rows = N tiles/channel
  (FOV w×h mm)`. Regression: `test_grid_preview_populates_viewer_and_navigator`,
  `TestNavigatorRasterGrid`.

## Bug fix — raster spacing ignored the selected objective (round 3, 2026-07-01)

- **Operator report:** "on the fluorescence mosaic workflow … the grid points
  are set for a 2x maybe. I just finished a raster scan with a 4x and it wasn't
  spaced out properly." **Root cause:** `_compute_raster_plan` resolved the
  effective µm/px with the SAME precedence as the fixed-objective full-plate
  scan — the shared `mosaic_scan.fov_um` override *first*, objective µm/px only
  as a fallback. On ME3B V1 that override is a calibrated **2822 µm** (≈ the 2×
  field of view). So regardless of the objective the operator picked, tiles were
  spaced for a 2× FOV: at 4× (real FOV ≈ 1426 µm) the grid stepped ≈ 2681 µm
  (`fov·(1−overlap)`) → **≈ 1255 µm gaps between tiles**. The workflow already
  resolves the correct per-objective µm/px (objective combo + `_on_objective_changed`
  + `_microscope_um_per_px`), but the `fov_um` short-circuit discarded it. The
  full-plate scan is unaffected — its objective is fixed, so its `fov_um` is
  valid there; only this objective-SELECTABLE workflow was wrong.
- **Fix (scoped to `fluorescence_mosaic_workflow.py`):** invert the precedence to
  **selected-objective µm/px > shared `fov_um` fallback > live camera µm/px**.
  New `_objective_um_per_px(frame_w)` returns the resolution-rescaled µm/px for
  the current objective (or `None` when uncalibrated); `_microscope_um_per_px`
  now delegates to it; `_compute_raster_plan` tries it first and only falls back
  to `fov_um`/camera µm/px when no objective calibration resolves (graceful
  degradation — no worse than before on an uncalibrated objective). The camera
  key + objective lookup are the exact pair the objective card WROTE the
  calibrations under, so the read is symmetric with the write. `spacing_um`
  remains an objective-independent manual step override. Because the actual scan
  drives off `self._scan_um_per_px = plan["eff_um_per_px"]`, fixing the plan
  fixes both the preview and the scan. **Needs real-HW verification on ME3B V1**
  (pick 4× → grid preview tiles overlap; raster covers the well with no gaps;
  switch to 2×/10× → spacing tracks). Regression:
  `test_objective_um_per_px_overrides_shared_fov` (4× @ 916 px = 1.547 µm/px wins
  over a 2822 fov_um; helper rescales by live width) + `test_fov_override_is_fallback_when_no_objective_cal`
  (renamed from `test_fov_override_sizes_tiles` — fov_um now the fallback).
  Suite: `tests/test_v75x_fluorescence_mosaic.py` (28) green.

## Feature — in-workflow mosaic FOV/spacing calibration (round 4, 2026-07-01)

- **Operator request:** "on the fluorescence mosaic workflow page I want the
  ability to do a mosaic calibration beforehand … in the settings to mirror the
  full mosaic scan version." **What was added:** a **"Calibrate…"** button + a
  per-objective status label in a new **"Mosaic FOV calibration"** section of the
  fluorescence Settings popout, opening the **same** `MosaicCalibrationDialog` the
  full-plate scan uses (Calibration → Plate Location → Mosaic scan → Calibrate…),
  wired to this workflow's controller / microscope camera / **per-camera+objective
  align key** (`_align_key`, identical scheme to
  `calibration._ploc_camera_objective_key`) / **selected-well centre**. Same
  Safe-Z + camera + µm/px gates as the scan. After it closes, the grid preview +
  status refresh.
- **Making the calibration take effect (resolution-safe):** the dialog stores a
  learned effective µm/px per camera+objective in `MosaicAlignmentStore`; the
  fluorescence raster planner now consumes it as the **top** precedence:
  `learned (rescaled) > objective µm/px (rescaled) > shared fov_um fallback >
  camera µm/px` (`_learned_um_per_px`). This preserves the round-3 principle
  (per-objective sources beat the shared 2× `fov_um`) — learned is also per
  camera+objective. Opened from here, the dialog is handed `fov_um=0` so the
  **calibration mosaic itself** builds at the objective scale, not the shared 2×.
- **Resolution stamp (the stale-value guard):** the on-disk store had a stale 2×
  learned value (3.094 µm/px) captured at **916 px** while the camera now runs at
  **3664 px** — consuming it naively would reintroduce the wrong-spacing bug. Fix:
  `MosaicAlignmentStore.set_um_per_px` gained an optional `resolution=` (stored)
  + `get_resolution()`; `MosaicCalibrationDialog._store_manual_align` now stamps
  `(fw, fh)`; `_learned_um_per_px` **rescales** by `cal_w / live_w` and
  **ignores** any legacy value with no resolution stamp (so the stale 2× value is
  bypassed → the resolution-safe objective value is used). All additive +
  backward-compatible: the full-plate scan still reads `get_um_per_px` un-rescaled
  and is unchanged. **Needs real-HW verification on ME3B V1** (Settings →
  Calibrate… at 4× → build a small mosaic → tune spacing → Store FOV/spacing →
  status shows "calibrated · µm/px @ 3664px"; the raster grid tiles tighten to
  match; a fresh scan covers the well with no gaps). Regression:
  `test_learned_calibration_overrides_objective_and_rescales`,
  `test_legacy_learned_without_resolution_is_ignored`,
  `test_settings_dialog_has_calibrate_button`,
  `test_open_mosaic_calibration_wires_dialog`, `TestMosaicAlignmentStoreResolution`.
  Suite: `tests/test_v75x_fluorescence_mosaic.py` (33) green; plate-mosaic /
  camera-rotation / camera-cal-liveview / spheroid-picker-scaling (146) green
  (only the documented pre-existing CV `test_real_24_well_mosaic` fails).

## Issues & Decisions

- **No filter hardware** → manual per-channel capture (one full single-well
  mosaic per channel). The same raster grid + camera scale are reused across
  channels so their composites register pixel-for-pixel (no inter-channel
  alignment needed); blending is the standard grayscale-intensity × pseudo-colour
  additive (saturating) merge.
- **Separate overlay layer** (not the existing `set_mosaic_overlay` slot) so the
  fluorescence overlay never clobbers the brightfield plate mosaic, and both can
  coexist; drawn under the well grid + targets.
- **Worker duplication**: `_SingleWellMosaicWorker` is a trimmed copy of
  `calibration._MosaicScanWorker` rather than an import, to avoid pulling the huge
  safety-critical `calibration.py` page module into the workflow import graph.
  Future cleanup could extract a shared `gui/widgets/mosaic_scan_worker.py`.
- **Quick Print deferred**: `PrintTrajectoryMonitorView` is an auto-fitting
  zero-ref-µm view distinct from the `JogWorkspaceView` family; adding a
  registered fluor overlay there needs its own coordinate pass. Left as a
  follow-up to avoid a half-correct registration.
