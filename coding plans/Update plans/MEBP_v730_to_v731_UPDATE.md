# MEBP v7.3.0 → v7.3.1 — Calibration & Jog Page Overhaul

**Objective:** Redesign the calibration workflow to use geometry-predicted well positions, full-plate mosaic stitching for plate verification/rotation detection, and per-well Z-offset teaching. Add an interactive well plate widget to the Jog page for safe fast-travel to any well.

**Branch:** `Version-7.3.1`
**Base:** `Version-7.3.0`
**Status:** Complete (110 automated tests passing)
**Date:** 2026-03-12

---

## 1. Feature Overview

### 1A — Geometry-Predicted Well Positions (Calibration Starting Points)

The system uses `WellPlate` geometry (spacing, row/col count) plus an assumed plate center (stage center = plate center) to compute rough XY positions for every well. These predicted positions serve as the **starting point** for all subsequent calibration.

**Key changes:**
- Compute all well positions via `WellPlate.get_well_position()` + plate center assumption or taught A1 offset
- Store as `_predicted_positions: dict[str, tuple[float, float]]` (well_name → (x_um, y_um))
- These are initial estimates that get refined by the 3-well auto-calibration

### 1B — 3-Well Auto-Calibration (replaces Full-Plate Mosaic Stitching)

Instead of scanning every well, the system auto-detects 3 reference wells forming a right triangle (A1, A-last-col, last-row-last-col) to determine plate position and orientation.

**Purpose:**
1. **Plate orientation** — Detect rotation and translation offset from predicted positions
2. **Affine correction** — Procrustes SVD similarity transform corrects all predicted positions

**Workflow:**
1. System raises Z to safe height
2. Navigates to each of the 3 calibration wells (approach position: center if well fits in FOV, edge offset otherwise)
3. At each position: capture frame, run well detection, record detected center in stage coords
4. Fit similarity transform (Procrustes SVD) from detected vs predicted positions
5. Apply correction to all well positions → `_calibrated_positions`
6. Show fast-travel buttons for the 3 calibration wells

**Key components:**
- `SupportClasses/MosaicBuilder.py` — `AffineCalibration` class with Procrustes fitting
- Wells shown as grey circles, calibration wells highlighted with blue rings
- Double-click any well to trigger per-well high-res scan with 50% overlap

### 1C — Auto Z-Bottom Calibration (replaces manual Z-Offset Teaching)

After auto-calibration, the system automatically finds the well bottom Z for each of the 3 calibration wells using focus-based needle detection, then fits a Z-plane.

**Algorithm — Safety-First Focus Sweep:**
1. Navigate to calibration well XY, start at safe Z
2. **Coarse sweep** (0.15mm steps down): Capture focus score at each step, detect when needle enters focus (score > 2× baseline). Hard safety floor at `top_z - well_depth_mm`
3. **Fine sweep** (0.03mm steps down): Track peak focus score. Stop on 3 consecutive declining steps — peak found
4. **Retract** to safe Z, store `best_z` for this well
5. Repeat for all 3 calibration wells
6. Fit Z-plane from the 3 detected Z values

**Safety Features:**
- Hard floor: All Z moves clamped to `estimated_bottom = top_z - well_depth_mm`. Needle NEVER goes below estimated bottom
- Baseline detection: Focus score captured at approach height (needle out of focus)
- Early stop: 3 consecutive declining focus steps triggers immediate stop in fine mode
- Cancel button available at any time, immediately retracts to safe Z

**Focus Scoring:** `NeedleDetector.compute_focus_score()` — 70% Laplacian variance + 30% Tenengrad (Sobel gradient)

**UI Elements:**
- Editable well plate depth field (pre-populated from `PLATE_DEFINITIONS`: 6/12/24/48-well: 17.4mm, 96-well: 10.67mm, 384-well: 11.56mm)
- Detect Needle + Focus Assist toggle buttons (auto-activated during Z-cal for live visual feedback)
- Auto Z-Cal button + Cancel button
- Progress display with current well and phase

### 1C-legacy — Manual Z-Offset Teaching with Jog Array (retained)

Manual Z teaching is still available via the embedded jog button array for users who prefer manual control:
- XY direction pad: ↑ ← ⌂ → ↓
- Z controls: Z ▲ / Z ▼
- Step size selectors: XY (1, 5, 10, 50, 100 µm) and Z (0.01, 0.05, 0.1, 0.5 mm)
- "Record Z" button per well

### 1D — Well Plate Widget on Jog Page (Fast Travel)

Add an interactive `WellPlateNavigator` widget to the Jog page. Clicking any well triggers safe fast-travel:

1. Raise Z to safe height (from calibration `_safe_z`)
2. Fast XY travel to well position (using calibrated or predicted positions)
3. Lower Z to approach height (safe_z or top_z + buffer)

**Requirements:**
- Well plate grid rendered as clickable circles (using existing `well_plate_view.py` pattern)
- Wells color-coded: grey (uncalibrated), green (calibrated), blue (current position)
- **Safety gate:** Fast travel is only available when safe_z has been set. If not set, show warning dialog.
- Current well highlighted based on proximity to stage position
- Tooltip showing well name + calibrated position
- Integrates with existing jog controls (step-based jog still works independently)

---

## 2. Files Modified / Created

### New Files

| File | Purpose |
|------|---------|
| `SupportClasses/MosaicBuilder.py` | Mosaic stitching engine: frame collection, composite image assembly, affine transform fitting |
| `gui/widgets/jog_well_plate.py` | Interactive well plate navigator widget for Jog page (clickable wells, fast travel) |
| `gui/widgets/jog_button_array.py` | Reusable jog button array widget (XY pad + Z + step selectors) — shared by Jog page and Calibration page |
| `tests/test_v731_mosaic.py` | Tests for MosaicBuilder (stitching, affine fitting, rotation detection) |
| `tests/test_v731_jog_navigation.py` | Tests for fast travel safety, well plate navigator signals |

### Modified Files

| File | Changes |
|------|---------|
| `gui/pages/calibration.py` | New mosaic scan step, embedded jog array for Z-offset teaching, geometry-predicted positions, revised workflow |
| `gui/pages/jog_control.py` | Add WellPlateNavigator widget, safe fast-travel logic, refactor jog buttons into shared JogButtonArray |
| `gui/widgets/well_plate_view.py` | Add click-to-navigate signals, color-coding for calibration state |
| `SupportClasses/WellPlate.py` | Add `get_all_positions_from_a1()` helper, meander traversal order generator |
| `SupportClasses/VisionDetector.py` | Batch well detection mode for mosaic scanning (optional: detect wells at each mosaic stop) |
| `SupportClasses/StageController.py` | `safe_travel_to(x_um, y_um, safe_z_mm)` convenience method (extract from calibration page) |
| `gui/app.py` | Wire calibration data → jog page (calibrated positions, safe_z) |

---

## 3. Implementation Steps

### Phase 1 — Geometry-Predicted Positions
- [x] 1.1 Add `WellPlate.get_all_positions_from_a1(a1_x_um, a1_y_um)` → returns dict of well_name → (x_um, y_um)
- [x] 1.2 Add `WellPlate.meander_order()` → returns well names in row-meander traversal order
- [x] 1.3 In `calibration.py`, after A1 teach (Step 2C accept): compute `_predicted_positions` using geometry
- [x] 1.4 Display predicted positions on the plate view (yellow dots for un-verified wells)
- [x] 1.5 Store predicted positions in calibration state for downstream use (recomputed from A1 + plate on load)

### Phase 2 — Mosaic Stitching Engine
- [x] 2.1 Create `SupportClasses/MosaicBuilder.py` with `MosaicBuilder` class
- [x] 2.2 Implement frame collection: `add_frame(well_name, frame, stage_x_um, stage_y_um, detection)`
- [x] 2.3 Implement composite stitching: `build_mosaic()` → BGR numpy array (2000px target)
- [x] 2.4 Implement well detection integration: accepts DetectionResult per frame, converts px offset → µm
- [x] 2.5 Implement affine fitting: `fit_affine(predicted)` → AffineCalibration (rotation, scale, translation, RMS residual)
- [x] 2.6 Implement position correction: `AffineCalibration.correct_positions(predicted)` → calibrated dict
- [x] 2.7 Write tests: 23 tests in `test_v731_mosaic.py` (identity, rotation, scale, translation, combined, noise, roundtrip, stitching)

### Phase 3 — Calibration Page: Mosaic Scan Step
- [x] 3.1 Add new calibration step "2D — Scan Plate" between A1 teach and corner teach (renumbered 2E/2F)
- [x] 3.2 Add "Scan Plate" button + Cancel + Accept buttons, progress and result labels
- [x] 3.3 During scan: QTimer-based meander, safe travel → capture frame → detect well → add to MosaicBuilder
- [x] 3.4 Show progress (well count, detection status per well) via `_lbl_scan_progress`
- [x] 3.5 After scan: display rotation angle, scale, RMS residual; enable Accept if ≥2 detections
- [x] 3.6 "Accept Scan" → apply affine correction → store `_calibrated_positions` + update scale/rotation
- [x] 3.7 Plate view: green dots (zValue=6) for calibrated positions, yellow (zValue=5) for predicted
- [x] 3.8 Save/load mosaic affine calibration data (rotation, scale, translation, center) in settings
- [x] 3.9 `_goto_well` and `_navigate_to_well` prefer calibrated positions over estimated

### Phase 4 — Jog Button Array Widget (Shared)
- [x] 4.1 Create `gui/widgets/jog_button_array.py` with `JogButtonArray` class
- [x] 4.2 Widget API: signals `jog_xy_requested(float, float)`, `jog_z_requested(float)`, `home_requested()`
- [x] 4.3 XY direction pad (▲ ◀ ⌂ ▶ ▼) + Z▲/Z▼ beside pad, step size combos
- [x] 4.4 Compact mode: 32px buttons, fewer step sizes (XY_STEPS_COMPACT, Z_STEPS_COMPACT)
- [~] 4.5 Jog page refactor skipped — existing page has Xbox, pump µL, step verification tightly coupled; risk of regression outweighs benefit

### Phase 5 — Calibration Page: Z-Offset Teaching
- [x] 5.1 Replaced Step 2F (Third Point) with "Step 2F — Teach Z-Offsets" section
- [x] 5.2 Embedded `JogButtonArray` (compact=True) for XY+Z fine-positioning during Z teaching
- [x] 5.3 Well selector combo (all plate wells) + "Go ▶" + "Next" for sequential navigation
- [x] 5.4 "Go to Well" uses `_navigate_to_well()` (prefers calibrated positions + safe travel)
- [x] 5.5 "Record Z" stores Z offset in `_z_teach_points` dict + updates legacy fields for compat
- [x] 5.6 Point count display with colour coding (yellow <3, green ≥3), lists taught wells
- [x] 5.7 "Fit Z-Plane" button calls `_try_fit_z_plane()` — now merges legacy + z_teach_points
- [x] 5.8 Z-teach well combo populated from `_compute_predicted_positions()` when plate/A1 set

### Phase 6 — Well Plate Navigator on Jog Page
- [x] 6.1 Create `gui/widgets/jog_well_plate.py` — `WellPlateNavigator` widget
- [x] 6.2 Render well plate grid as clickable circles (QPainter-based painted widget)
- [x] 6.3 Color coding: grey=uncalibrated, green=calibrated, blue=current, orange=hover
- [x] 6.4 Click signal: `well_clicked(str)` emitted on left-click
- [x] 6.5 Tooltip: well name + position (µm) if calibrated
- [x] 6.6 Current-well tracking: `update_current_from_position()` highlights nearest well within 0.6× spacing
- [x] 6.7 Add `WellPlateNavigator` to jog page layout (below quick actions, in collapsible group)
- [x] 6.8 Wire click → safe fast-travel via `StageController.safe_travel_to()`
- [x] 6.9 Safety gate: QMessageBox warning if safe_z not set or well has no position
- [x] 6.10 Extract `safe_travel_to(target_x_um, target_y_um, safe_z_mm)` into `StageController`
- [x] 6.11 Wire calibration state to jog page via `app.py` signal bridge (`calibration_data_changed` signal → `set_calibration_data()`)

### Phase 7 — Integration & Testing
- [x] 7.1 End-to-end test: `test_v731_integration.py` — 51 tests covering geometry prediction, meander order, affine correction, calibration→jog data flow, safe_z propagation, fast travel, safety gates, 3-well calibration, auto Z-cal
- [x] 7.2 Write `test_v731_mosaic.py` — 23 tests (affine fitting, stitching, correction roundtrip) — all passing
- [x] 7.3 Write `test_v731_jog_navigation.py` — 26 tests (navigator state, current tracking, signals, safe_travel_to, calibration data bridge, jog page integration) — all passing
- [x] 7.4 Test with SimulatedCamera: `test_v731_integration.py::TestSimulatedCameraMosaic` — 5 tests (frame generation, well visibility, mosaic build, detection pipeline, affine near-identity)
- [x] 7.5 Test rotation detection: apply known rotation, verify affine correction accuracy (covered in test_v731_mosaic.py + test_v731_integration.py)
- [~] 7.6 Verify jog button array parity: jog page refactor skipped (Phase 4.5), JogButtonArray used only in calibration page

### Phase 8 — Simplified Wizard, 3-Well Auto-Calibration & Auto Z-Cal
- [x] 8.1 Delete Step 2C (manual Teach A1) UI and logic — no longer required
- [x] 8.2 Delete Step 2E (manual Teach Corner) UI and logic — no longer required
- [x] 8.3 Renumber steps: 2D → 2C "Auto-Calibrate (3-Well)", 2F → Step 3 "Z-Bottom Calibration"
- [x] 8.4 Implement `_get_calibration_wells()` — returns A1, A(last_col), (last_row)(last_col) right triangle
- [x] 8.5 Implement `_compute_approach_position()` — center if well < 80% FOV, edge offset otherwise
- [x] 8.6 Implement `_detect_well_at_position()` — navigate, capture, detect, convert to stage coords
- [x] 8.7 Rewrite `_start_plate_scan()` for 3-well auto-calibration flow
- [x] 8.8 Rewrite `_scan_tick()` — one calibration well per tick, progress updates
- [x] 8.9 Rewrite `_finish_plate_scan()` — Procrustes SVD fit, AffineCalibration, show results
- [x] 8.10 Rewrite `_accept_plate_scan()` — apply calibration, set `_taught_a1` from detected A1, populate Z-teach combo
- [x] 8.11 Add fast-travel buttons for 3 calibration wells after auto-calibrate
- [x] 8.12 Update validation gates: `_navigate_to_well()` uses calibrated > predicted > estimated (no taught A1 required)
- [x] 8.13 Move Detect Needle + Focus Assist buttons from Step 1 area to Step 3 (Z calibration)
- [x] 8.14 Add editable well depth field (QDoubleSpinBox, pre-populated from `PLATE_DEFINITIONS`)
- [x] 8.15 Implement Auto Z-Cal state machine: `_start_auto_z_cal()`, `_auto_z_tick()`, `_auto_z_finish()`, `_cancel_auto_z()`
- [x] 8.16 Auto Z-Cal phases: navigate → coarse (0.15mm) → fine (0.03mm) → retract, per calibration well
- [x] 8.17 Safety: hard floor at `top_z - well_depth_mm`, stop on 3 declining focus steps, no overshoot
- [x] 8.18 Auto-activate Detect Needle + Focus Assist during Auto Z-Cal for live visual feedback
- [x] 8.19 Auto-deactivate vision toggles after Z-Cal completes or is cancelled
- [x] 8.20 SimulatedCamera focal plane: set to `top_z - well_depth` in `_configure_simulated_cameras()`
- [x] 8.21 Validation "Go" button lowers needle to Z-plane focus position (not just XY travel)
- [x] 8.22 Per-well scan overlap changed to 50% (from 10%) for high-res scanning
- [x] 8.23 Update UI labels: "Auto-Calibrate (3-Well)", "Auto Z-Cal", step descriptions
- [x] 8.24 Tests: `TestThreeWellCalibration`, `TestAutoZCalibration`, `TestSingleWellScan50pctOverlap` — all passing
- [x] 8.25 Total: 110 tests across 3 suites (51 integration + 36 vision + 23 camera)

---

## 4. Calibration Workflow (Revised for v7.3.1)

### Step 1: Zero Needle (unchanged)
Manual jog to contact point → "Set Zero"

### Step 2: Teach Well Plate (simplified)
| Sub-step | Description |
|----------|-------------|
| **2A — Safe Z** | Record safe travel height above plate |
| **2B — Top Z** | Record plate surface Z |
| **2C — Auto-Calibrate (3-Well)** | Auto-detect 3 reference wells (A1, A-last, last-last) → Procrustes SVD similarity transform → correct all predicted positions. Fast-travel buttons shown for calibration wells after completion |

> **Removed:** Step 2C (Teach A1) and Step 2E (Teach Corner) — manual teaching no longer required. Auto-calibrate replaces both.

### Step 3: Z-Bottom Calibration
| Sub-step | Description |
|----------|-------------|
| **Well Depth** | Editable field pre-populated from plate definition |
| **Detect Needle / Focus Assist** | Vision toggles (auto-activated during Auto Z-Cal) |
| **Auto Z-Cal** | Navigate to 3 calibration wells → coarse+fine focus sweep → fit Z-plane |
| **Manual Z-teach** | Fallback: embedded jog array, record Z per well, fit plane |

### Step 4: Validate (updated)
- Well selector + "Go" navigates to well AND lowers needle to Z-plane focus position
- Uses calibrated > predicted > estimated positions (no longer requires taught A1)
- Validation gate accepts any available position data (calibrated, predicted, or estimated)

---

## 5. Testing Notes

### Automated Tests (110 total)
- `test_v731_integration.py` (51 tests): Geometry prediction, meander order, affine correction, 3-well calibration selection, approach position (center/edge), auto Z-cal focus scoring, well depth from plate definitions, single-well 50% overlap scan, calibration→jog data flow, safe_z propagation, fast travel, safety gates
- `test_v731_mosaic.py` (23 tests): MosaicBuilder frame collection, stitching, affine fitting with known transforms
- `test_v731_jog_navigation.py` (26 tests): Safe travel precondition checks, signal emissions, well plate navigator state
- `test_v730_vision.py` (36 tests): Well/needle detection, focus scoring, detection strategies
- `test_v730_simulated_camera.py` (23 tests): SimulatedCamera rendering, DOF model, focal plane

### Manual Testing
1. **Simulated mode:** Run with `--simulate` flag, verify full calibration workflow with SimulatedCamera
2. **3-well auto-calibration:** Select plate → click Auto-Calibrate → verify stage moves to 3 wells → affine result shown → fast-travel buttons populated
3. **Auto Z-Cal:** Click Auto Z-Cal → verify vision toggles activate → coarse sweep detects needle → fine sweep finds peak → Z-plane fitted
4. **Safety verification:** Confirm needle never descends below `top_z - well_depth_mm` during Z sweep
5. **Validation with Z:** Select well in validator → click Go → verify XY travel AND Z lowering to focus plane
6. **Per-well scan:** Double-click a well → verify high-res scan with 50% overlap produces dense tile grid
7. **Jog page fast travel:** Click well on navigator, verify safe Z → fast XY → lower Z sequence
8. **Safety gate:** Remove safe_z setting, verify fast travel shows warning instead of moving

---

## 6. Issues & Decisions

| # | Issue | Decision | Date |
|---|-------|----------|------|
| 1 | Should mosaic scan be mandatory or optional? | Replaced with 3-well auto-calibration — faster, simpler, no full-plate traversal needed | 2026-03-12 |
| 2 | Where to put well plate on jog page? | Collapsible section below quick actions | 2026-03-11 |
| 3 | Should corner teach (old Step 2D) be removed entirely? | Yes — removed along with Teach A1. 3-well auto-calibration replaces both manual steps | 2026-03-12 |
| 4 | Minimum wells for Z-plane fit? | 3 points — auto Z-cal uses the same 3 calibration wells | 2026-03-11 |
| 5 | Safe travel extraction: method on StageController or standalone? | StageController method — consistent with hardware abstraction principle | 2026-03-11 |
| 6 | Z-cal safety: how to prevent needle hitting well bottom? | Hard floor at `top_z - well_depth_mm`, stop on 3 declining focus steps, no overshoot below estimated bottom | 2026-03-12 |
| 7 | Where should Detect Needle / Focus Assist buttons live? | Moved from Step 1 to Step 3 (Z calibration) — that's where they're actually used | 2026-03-12 |
| 8 | SimulatedCamera focal plane assumption? | Set to `top_z - well_depth` (bottom of well) instead of z=0.0 | 2026-03-12 |
| 9 | Should validation "Go" also lower Z? | Yes — navigate XY AND lower to Z-plane focus position for immediate visual confirmation | 2026-03-12 |
| 10 | Per-well scan overlap? | Changed from 10% to 50% for higher resolution well imaging | 2026-03-12 |
