# MEBP v7.2.5 — Comprehensive Update Plan

## Version: 7.2.5 | Date: March 2026
## Scope: Config load pump fix, XY motion unit fix, jog pump µL, print generation, print objects list, well setup redesign, wellplate zoom

---

## 1. Issue Tracker

| # | Issue Summary | Priority | Files Affected |
|---|---------------|----------|----------------|
| 1 | Loading saved HW config doesn't update pump or auto-populate needle-pump assignment | Critical | `hardware_setup.py` |
| 2 | XY jog moves wrong distance (50µm→42.2µm Y, 39.5µm X) — Prior expects microns, not microsteps | Critical | `jog_control.py`, `StageController.py`, `app.py` |
| 3 | Pump axis should be inactive if no pump designated; movements in µL not mm | High | `jog_control.py` |
| 4 | Print Settings page needs "Generate Print" button (validate + generate execution plan) | High | `print_setup.py` |
| 5 | Print Objects needs "Send to Available Prints" button; list populates well setup dropdown | High | `print_objects.py`, `print_well_setup.py`, `print_setup.py` |
| 6 | Well Setup page redesign: select wells → assign role → show role-specific options → color coding | Critical | `print_well_setup.py` |
| 7 | Well assignment summary panel to the right of well preview | High | `print_well_setup.py` |
| 8 | Ensure all inks and rosettes available in well setup from HardwareConfig | High | `print_well_setup.py` |
| 9 | Wellplate view zoom difficulty — overlay Home/Zoom+/Zoom- buttons | Medium | `well_plate_view.py`, `print_well_setup.py` |

---

## 2. Detailed Root Cause Analysis & Change Specifications

### Issue 1: Loading Saved HW Config Doesn't Update Pump / Needle-Pump Assignment

**Symptom**: When user loads a saved hardware config JSON, the pump widgets don't reflect the loaded config and the needle-channel-pump assignment doesn't auto-populate.

**Root Cause**: In `hardware_setup.py._apply_config_to_ui()`, the restore sequence has two problems:
1. The pump widget `set_config()` call may fail silently if the ink combo hasn't been refreshed yet, OR the ink name in the saved config doesn't match a currently loaded ink library entry (timing issue — ink library needs to be populated FIRST)
2. The `_rebuild_channel_map_rows()` call creates the correct number of rows but the subsequent combo restore loop may not find the pump IDs because the pump enable states haven't been fully committed to the config object yet

**Fix Strategy**:
- In `_apply_config_to_ui()`, after restoring ink library and pump configs, explicitly call `_on_config_changed()` to rebuild the internal config state BEFORE attempting to restore the channel map
- Add a `_force_pump_widget_refresh()` helper that iterates all pump widgets and re-applies their config from `self._config.pumps[pid]` AFTER the ink library is fully loaded
- In the channel map restore, add fallback: if `combo.findData(mapped_pump)` returns -1, log a warning and skip rather than leaving the combo in a stale state
- After full restore, emit `config_changed` signal to propagate to all downstream pages

**Changes Required**:

#### File: `gui/pages/hardware_setup.py`
- **`_apply_config_to_ui()` method** — restructure to enforce order:
  1. Config name + notes
  2. Plate format
  3. Ink library → `_refresh_ink_table()` + `_refresh_pump_ink_combos()`
  4. Rosette library → `_refresh_rosette_table()`
  5. Needle gauge + length + channels
  6. **Pumps**: For each pump, call `pw.set_config(pcfg, ink_names=list(self._config.ink_library.keys()))` — ensure ink_names are passed
  7. Refresh pump-ink exclusions: `_refresh_pump_ink_exclusions()` + `_update_pump_ink_summary()`
  8. Channel map: `_rebuild_channel_map_rows()` then restore combos with validation
  9. Final `_on_config_changed()` to emit signal

- Add defensive check in `PumpChannelWidget.set_config()`: if ink name not found in provided ink_names list, clear the ink selection and log warning

---

### Issue 2: XY Jog Moves Wrong Distance — Prior Controller Expects Microns Directly

**Symptom**: User sets XY step to 50µm, moves Y→42.2µm, moves X→39.5µm. Movements are consistently ~80-85% of expected.

**Root Cause Analysis**: The Prior ProScan II/III controllers accept position and movement commands **in their native encoder units**. For standard ProScan III configuration, the native unit is **0.1 µm per encoder count** (i.e., 10 encoder counts = 1 µm). 

However, the current code does:
```python
step_steps = step_um * self._microsteps_per_micron  # 50 * 10 = 500
controller.move_xy_relative(500, 0)  # sends GR 500,0
```

If the Prior controller is configured so that `GR 500,0` means "move 500 × 0.1µm = 50µm", this SHOULD work. But Alex reports it doesn't match.

**The actual issue**: The Prior controller's position values returned by `P` (position query) and consumed by `G`/`GR` (move commands) may not use "microsteps" at all — they may use **microns directly** if the controller has been configured with `ENCODER 6` or similar resolution setting. In this case:
- `GR 50,0` = move 50 µm ✓
- `GR 500,0` = move 500 µm ✗ (10x too much!)

But since the user sees ~0.8x the expected movement, NOT 10x, a different explanation is needed. The most likely cause is that the `_microsteps_per_micron` factor is **correct for sending commands** (the `round()` in `move_stage_relative` is the issue), but the READBACK conversion is wrong.

**Alternative root cause (most likely given 42.2/50 ≈ 0.844)**: The Prior ProScan controller may return positions in a different unit than it accepts commands. OR the `_microsteps_per_micron` value (10.0) doesn't match the actual hardware. If the stage actually has a resolution of ~11.86 counts/µm, then:
- Send: `GR 500,0` (meant to be 50µm × 10)
- Stage moves: 500/11.86 = 42.2 µm ← matches Y observation exactly!

**Fix Strategy — Two-part solution**:

**Part A**: The fundamental fix is to send commands **in the units the controller actually expects**. Since the Prior ProScan controllers report their resolution via the `ENCODER` command or it's documented in the protocol JSON, we should:
1. Query the stage for its actual resolution at connect time, OR
2. Use the `microsteps_per_micron` value from the protocol JSON correctly

**Part B**: If the Prior controller accepts commands in microns (as Alex believes), we should bypass the microstep conversion entirely for the move commands and only use it for display conversion from raw position readback.

**Recommended approach**: Add a flag `_controller_uses_microns: bool` to the jog control. If True, skip the microstep conversion entirely — send µm values directly to the controller. The conversion factor is only needed for position readback display.

**Changes Required**:

#### File: `gui/pages/jog_control.py`
- **`_jog_xy()`**: Change to send **micron values directly** to a new method `controller.move_xy_relative_um()` instead of converting to microsteps
- **`on_status_update()`**: Remove the `/ self._microsteps_per_micron` division for XY position display — if controller reports in microns, display directly
- Add `_xy_units` attribute: `"microns"` or `"microsteps"` — loaded from protocol
- If `_xy_units == "microns"`: send µm directly, display raw position
- If `_xy_units == "microsteps"`: keep current conversion logic

#### File: `SupportClasses/StageController.py`
- Add `move_xy_relative_um(dx_um: float, dy_um: float)` method that sends micron values converted via the controller's known factor
- OR: modify `move_xy_relative` to accept a `units` parameter

#### File: `gui/app.py`
- Propagate the `xy_units` setting to the jog page from the protocol or settings

**SIMPLEST FIX (recommended)**: Since Alex states the controller expects microns, the cleanest fix is:
1. In `_jog_xy()`: send `step_um` directly → `controller.move_xy_relative(dx * step_um, dy * step_um)` 
2. In `StageController.move_xy_relative()`: rename to clarify units, or add a new `_um` variant
3. In position readback: display raw values from controller (they're already in microns)
4. Remove the `_microsteps_per_micron` conversion from the jog page entirely

---

### Issue 3: Pump Axis Inactive If No Pump + Movements in µL

**Symptom**: Pump jog buttons are active even when no pump is configured. Pump movements display in mm instead of µL.

**Changes Required**:

#### File: `gui/pages/jog_control.py`
- **Pump button state**: In `set_hardware_config()` or `on_status_update()`, check which pumps are configured (`hw_config.configured_pump_ids`). Disable P1/P2/P3 jog buttons and labels for unconfigured pumps.
- **µL display**: When hardware config is available, convert pump position from mm to µL using `HardwareConfig.mm_to_uL(pump_id, mm_value)`. Update pump position labels to show µL.
- **µL step sizes**: Change P_STEPS to µL values: `[0.1, 0.5, 1.0, 5.0, 10.0, 50.0]` µL. Convert to mm when sending command via `HardwareConfig.uL_to_mm()`.
- **Step combo**: Change step combo to show "µL" suffix when HW config available.
- Store `_pump_buttons` dict for per-pump enable/disable access.

---

### Issue 4: Print Settings — "Generate Print" Button

**Symptom**: No button to validate and generate the execution plan from the print settings page.

**Changes Required**:

#### File: `gui/pages/print_setup.py`
- Add a "Generate Print" button in the left box / context area that:
  1. Calls `tab_wells.validate()` to check well setup
  2. Calls `tab_wells._generate_plan()` to build the execution plan
  3. Shows validation results in a status label
  4. If valid, enables the "Send to Monitor" button
- Button placement: In the print settings left panel (context widget), add a prominent "Generate Print" button above the "Send to Monitor" button
- Visual state: Button shows green when plan is valid, red/yellow when issues exist

---

### Issue 5: Print Objects → Available Prints List → Well Setup Dropdown

**Symptom**: No way to send a completed print object to a list of available prints. The well setup dropdown for print assignment doesn't populate from print objects.

**Current Flow**: `print_objects.py` has a `prints_changed` signal that emits a list of print file names. The well setup `print_combo` is populated from `self._available_prints`.

**Changes Required**:

#### File: `gui/pages/print_objects.py`
- Add a "Send to Available Prints" button in the objects toolbar (or file management section)
- When clicked, ensure the current print file is saved and the `prints_changed` signal is emitted with the updated list
- The signal already exists — the issue is likely that it's not being emitted when files are created/modified, OR the well setup isn't receiving it

#### File: `gui/pages/print_setup.py` (parent tab container)
- Verify the signal wiring: `tab_objects.prints_changed → tab_wells.set_available_prints()`
- Add `set_available_prints()` method to well setup tab if missing

#### File: `gui/pages/print_well_setup.py`
- Add/fix `set_available_prints(names: list[str])` method that:
  1. Updates `self._available_prints`
  2. Refreshes the print combo box
  3. Preserves current selection if still valid

---

### Issue 6: Well Setup Page Redesign

**Desired Flow**:
1. User selects wells (click, drag, Ctrl+click) → wells outlined in purple
2. User assigns a role (Print, Ink, Wash, Waste, Buffer, Sorted, Empty)
3. After role assignment, role-specific options appear:
   - **Print**: Print file dropdown (populated from available prints) — select which print object to assign
   - **Ink**: Ink dropdown (from ink library) — which ink this well contains
   - **Wash/Waste/Buffer**: Behavior parameters (depth, duration, etc.)
4. After configuration, well border recolored: **green** = assigned/ready, **red** = assigned but incomplete
5. Selected wells always outlined in **purple**

**Changes Required**:

#### File: `gui/pages/print_well_setup.py`
- **Layout restructure**: 
  ```
  ┌──────────────────────────┬──────────────────────┐
  │  Interactive Plate View  │  Assignment Summary   │
  │  (XY) with zoom buttons │  (scrollable table)   │
  │                          │                       │
  ├──────────────────────────┴──────────────────────┤
  │  Role Assignment Bar                             │
  │  [Print] [Ink] [Wash] [Waste] [Buffer] [Sorted]  │
  ├──────────────────────────────────────────────────┤
  │  Role-Specific Options (dynamic panel)           │
  │  - For Print: [Print File ▾] [Assign] [Clear]    │
  │  - For Ink: [Ink Name ▾] [Assign]                │
  │  - For Wash: depth, duration, etc.                │
  ├──────────────────────────────────────────────────┤
  │  Print Plan of Action (existing v7.2.4 section)  │
  └──────────────────────────────────────────────────┘
  ```

- **Role Assignment Bar**: Row of toggle buttons, one per role. Clicking a role button assigns that role to all currently selected wells. The button stays highlighted to show the "active assignment mode."

- **Dynamic Options Panel**: A stacked widget that changes based on the selected role:
  - **Print role**: Shows print file combo + assign/replace/clear buttons
  - **Ink role**: Shows ink name combo (from `_hw_config.ink_library`) + assign button  
  - **Wash/Waste/Buffer**: Shows behavior parameter spinboxes
  - **Empty**: Shows "Clear assignments" button

- **Well Color Coding**:
  - **Green border**: Well has role AND is fully configured (e.g., print well has a print file assigned)
  - **Red border**: Well has role but is incomplete (e.g., ink well with no ink selected)
  - **Gray fill**: Empty/unassigned wells
  - **Purple outline**: Currently selected wells (always on top of role color)
  - Use the existing `ROLE_COLORS` for fill, add border logic

- **Remove old layout**: Remove ZY/XZ projections (if not already removed in v7.2.4)

---

### Issue 7: Well Assignment Summary Panel

**Changes Required**:

#### File: `gui/pages/print_well_setup.py`
- Add a summary panel to the RIGHT of the well plate view using a `QSplitter`
- Summary shows a scrollable table with columns: Well, Role, Assignment, Status
- Color-coded rows matching role colors
- Clicking a row in the summary selects that well in the plate view
- Summary updates whenever assignments change
- Include counts at bottom: "4 Print | 2 Ink | 1 Wash | 1 Waste | 1 Buffer | 15 Empty"

---

### Issue 8: Ensure All Inks & Rosettes Available in Well Setup

**Changes Required**:

#### File: `gui/pages/print_well_setup.py`
- In `set_hardware_config()`:
  1. Extract `config.ink_library` → populate ink combo for Ink role assignment
  2. Extract `config.rosette_library` → populate rosette combo
  3. Extract `config.pump_ink_map` → show pump-ink mapping in ink assignment UI
  4. Store `self._hw_config` reference
- Refresh these combos whenever `set_hardware_config()` is called
- The ink combo for Ink wells should show: "Hydrogel A (P1)" format showing which pump it's assigned to

---

### Issue 9: Wellplate View Zoom Controls

**Changes Required**:

#### File: `gui/widgets/well_plate_view.py`
- Add overlay buttons for Home (fit-all), Zoom+, Zoom- in the top-right corner of the view
- Buttons should be semi-transparent, small, and always visible regardless of scroll/zoom state
- Implement as a `QWidget` overlay on top of the `QGraphicsView`
- Home button: calls `fitInView()` on the plate scene rect
- Zoom+: scale view by 1.25x
- Zoom-: scale view by 0.8x
- Ensure mouse wheel zoom still works alongside buttons

---

## 3. Session Breakdown

### Session 1: HW Config Load Fix + XY Motion Fix (4-5 hrs)
**Issues**: 1, 2

| Task | Description |
|------|-------------|
| S1.1 | Fix `_apply_config_to_ui()` restore order: ink library → pumps → channel map |
| S1.2 | Add defensive ink name validation in `PumpChannelWidget.set_config()` |
| S1.3 | After full restore, force `_on_config_changed()` + `config_changed.emit()` |
| S1.4 | Fix `_jog_xy()` — send micron values directly, remove microstep conversion |
| S1.5 | Fix position readback — display raw µm from controller, remove division |
| S1.6 | Update `StageController.move_xy_relative()` — clarify units, add logging |
| S1.7 | Update `app.py` — remove or simplify microsteps_per_micron propagation |
| S1.8 | Test: save config with pumps+inks+channels, reload, verify all restored |
| S1.9 | Test: jog 50µm, verify actual movement matches |

**Output**: `patches/v725/patch_s1_config_load_and_xy_fix.py`
**Files Modified**: `hardware_setup.py`, `jog_control.py`, `StageController.py`, `app.py`

---

### Session 2: Jog Pump µL + Print Settings Button (3-4 hrs)
**Issues**: 3, 4

| Task | Description |
|------|-------------|
| S2.1 | Add pump enable/disable state tracking in jog page from HardwareConfig |
| S2.2 | Disable P1/P2/P3 buttons when respective pump not configured |
| S2.3 | Convert pump step sizes to µL; add µL↔mm conversion using HardwareConfig |
| S2.4 | Update pump position display to show µL when HW config available |
| S2.5 | Add "Generate Print" button to print_setup.py context panel |
| S2.6 | Wire button to validate + generate plan + show results |
| S2.7 | Add visual status feedback (green/red/yellow) for plan validity |
| S2.8 | Test: disable pump P2, verify P2 buttons are grayed out |
| S2.9 | Test: click Generate Print with valid/invalid setup |

**Output**: `patches/v725/patch_s2_pump_ul_and_generate.py`
**Files Modified**: `jog_control.py`, `print_setup.py`

---

### Session 3: Print Objects List + Well Setup Wiring (3-4 hrs)
**Issues**: 5, 8

| Task | Description |
|------|-------------|
| S3.1 | Add "Send to Available Prints" button in print_objects.py |
| S3.2 | Ensure `prints_changed` signal emits on file create/save/delete |
| S3.3 | Verify signal wiring in print_setup.py: objects→wells |
| S3.4 | Add/fix `set_available_prints()` in print_well_setup.py |
| S3.5 | Refresh print combo from available prints list |
| S3.6 | Refresh ink combo from `_hw_config.ink_library` in well setup |
| S3.7 | Refresh rosette combo from `_hw_config.rosette_library` |
| S3.8 | Show pump assignment in ink combo: "Hydrogel A (P1)" |
| S3.9 | Test: create print object, send to list, verify appears in well setup |

**Output**: `patches/v725/patch_s3_prints_list_and_hw_sync.py`
**Files Modified**: `print_objects.py`, `print_well_setup.py`, `print_setup.py`

---

### Session 4: Well Setup Redesign — Part 1: Layout + Role Assignment (5-6 hrs)
**Issues**: 6, 7

| Task | Description |
|------|-------------|
| S4.1 | Restructure layout: plate view (left) + summary (right) in QSplitter |
| S4.2 | Build role assignment button bar below plate view |
| S4.3 | Build dynamic role-specific options panel (QStackedWidget) |
| S4.4 | Print role panel: print file combo + assign/replace/clear |
| S4.5 | Ink role panel: ink name combo + assign |
| S4.6 | Service role panels: wash/waste/buffer parameter spinboxes |
| S4.7 | Wire role buttons → assign role to selected wells |
| S4.8 | Wire role-specific options → update well assignments |
| S4.9 | Build assignment summary table (right panel) |
| S4.10 | Summary table: Well, Role, Assignment, Status columns |
| S4.11 | Summary click → select well in plate view |
| S4.12 | Summary counts footer: "4 Print | 2 Ink | ..." |

**Output**: `patches/v725/patch_s4_well_setup_redesign.py`
**Files Modified**: `print_well_setup.py`

---

### Session 5: Well Color Coding + Zoom Controls (3-4 hrs)
**Issues**: 6 (continued), 9

| Task | Description |
|------|-------------|
| S5.1 | Implement well border color logic: green=ready, red=incomplete, gray=empty |
| S5.2 | Purple selection outline always on top |
| S5.3 | Update `_refresh_well_colors()` with new readiness logic |
| S5.4 | Add zoom overlay buttons to WellPlateView: Home, +, − |
| S5.5 | Implement zoom button actions: fitInView, scale 1.25x, scale 0.8x |
| S5.6 | Style buttons semi-transparent, positioned top-right |
| S5.7 | Ensure mouse wheel zoom still works |
| S5.8 | Keep Plan of Action section from v7.2.4 intact |
| S5.9 | Integration test: full workflow |

**Output**: `patches/v725/patch_s5_colors_and_zoom.py`
**Files Modified**: `print_well_setup.py`, `gui/widgets/well_plate_view.py`

---

## 4. File Change Summary

| File | Issues | Action | Est. Lines Changed |
|------|--------|--------|-------------------|
| `gui/pages/hardware_setup.py` | 1 | FIX — config load restore order | +40 / -20 |
| `gui/pages/jog_control.py` | 2, 3 | MAJOR FIX — remove microstep conversion, pump µL | +120 / -60 |
| `SupportClasses/StageController.py` | 2 | FIX — clarify move units | +20 / -5 |
| `gui/app.py` | 2 | SIMPLIFY — microstep propagation | +10 / -10 |
| `gui/pages/print_setup.py` | 4, 5 | ADD — generate button, prints wiring | +80 |
| `gui/pages/print_objects.py` | 5 | ADD — send to available prints button | +30 |
| `gui/pages/print_well_setup.py` | 5, 6, 7, 8 | MAJOR REWRITE — redesigned layout + logic | +500 / -300 |
| `gui/widgets/well_plate_view.py` | 9 | ADD — zoom overlay buttons | +80 |

**Total: ~880 added, ~395 removed, ~1,275 lines touched**

---

## 5. Dependency Graph

```
Session 1 (Config Load + XY Fix)
    │
    ├──→ Session 2 (Pump µL + Generate Button)
    │
    ├──→ Session 3 (Prints List + HW Sync) ──→ Session 4 (Well Redesign P1)
    │                                                   │
    │                                          Session 5 (Colors + Zoom)
    │
    └──→ (all sessions depend on S1 for correct HW config propagation)
```

---

## 6. Schedule Estimate

| Session | Hours | Priority | Depends On |
|---------|-------|----------|------------|
| S1 | 4-5 | Critical | — |
| S2 | 3-4 | High | S1 |
| S3 | 3-4 | High | S1 |
| S4 | 5-6 | Critical | S1, S3 |
| S5 | 3-4 | High | S4 |

**Total: 18-23 hours across 5 sessions**

---

## 7. Patch File Structure

```
patches/v725/
├── apply_all_v725_patches.py
├── patch_s1_config_load_and_xy_fix.py
├── patch_s2_pump_ul_and_generate.py
├── patch_s3_prints_list_and_hw_sync.py
├── patch_s4_well_setup_redesign.py
├── patch_s5_colors_and_zoom.py
└── MEBP_v725_UPDATE_PLAN.md
```

---

## 8. Critical Implementation Notes

### XY Motion Fix (Issue 2) — IMPORTANT
The fix for issue 2 depends on confirming whether the Prior ProScan controller expects:
- (a) **Microns directly**: Commands like `GR 50,0` mean "move 50 µm"
- (b) **Encoder counts**: Commands like `GR 500,0` mean "move 500 counts × resolution"

**Alex states the controller expects microns.** The fix should:
1. Remove `* self._microsteps_per_micron` from `_jog_xy()`
2. Remove `/ self._microsteps_per_micron` from position readback
3. The `GR` command should receive the µm value directly: `GR 50,0` for 50µm

**However**, if the controller is actually in encoder-count mode (0.1µm per count), the current 10.0 factor would be correct and the 42.2µm/39.5µm discrepancy points to the factor being wrong (should be ~11.86 for Y and ~12.66 for X). In this case, the fix is to calibrate the factor per-axis.

**Recommendation**: Implement the "microns direct" approach per Alex's instruction, but add a debug mode that logs exactly what's sent vs. what's read back, so we can verify.

### Well Setup Redesign (Issue 6) — Layout
The redesign is substantial but builds on the existing `WellPlateView`, `WellSetupModel`, and `WellAssignment` infrastructure. The key change is the UI flow (select → assign role → configure) rather than the underlying data model.

### Patch Reliability
All patches will use **regex-based anchoring** (learned from v7.2.4 S2) rather than exact string matching where comment formatting may vary. Each patch function will:
1. Read the file content
2. Search for anchors using regex patterns
3. Apply replacements with SKIP/OK/MISS reporting
4. Be idempotent (safe to re-run)
