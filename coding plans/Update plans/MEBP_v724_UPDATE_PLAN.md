# MEBP v7.2.4 — Comprehensive Update Plan

## Version: 7.2.4 | Date: March 2026
## Scope: Hardware config propagation audit, jog step fix, UI polish, pump-ink-needle workflow, print plan of action, well validation

---

## 1. Issue Tracker

| # | Issue Summary | Priority | Files Affected |
|---|---------------|----------|----------------|
| 1 | HW config not propagating to calibration plate selector & well setup ink/rosette lists | Critical | `calibration.py`, `print_well_setup.py`, `print_objects.py`, `app.py` |
| 2 | XY jog step size display does not match actual motion | Critical | `jog_control.py` |
| 3 | Hardware Setup left box needs saved config file browser | Medium | `hardware_setup.py` |
| 4 | Well Setup: remove XZ and ZY projections (keep XY only) | Low | `print_well_setup.py` |
| 5 | Print Objects: well preview too small, projections misaligned, objects should be right of preview, expandable/zoomable | High | `print_objects.py` |
| 6 | Print Objects: out-of-bounds objects should flash red in objects list | Medium | `print_objects.py` |
| 7 | GUI section titles misaligned / not stylish across all pages | Medium | `gui/styles.py`, all page files |
| 8 | Inks must be assigned to pumps (not just listed) | High | `hardware_setup.py`, `HardwareConfig.py` |
| 9 | Needle selection after pump assignments; each channel/bore assigned to a pump | High | `hardware_setup.py`, `HardwareConfig.py` |
| 10 | Print Plan of Action: multi-run ink loading, wash/refill decisions, max ink per run | High | NEW `SupportClasses/PrintPlanOfAction.py`, `print_well_setup.py`, `print_setup.py` |
| 11 | Well Setup validation before sending to monitor | Critical | `print_well_setup.py`, `print_setup.py` |

---

## 2. Detailed Change Specifications

### Issue 1: Hardware Config Propagation Audit

**Problem**: The calibration page left box has its own plate format combo that is not updated when hardware config changes on Page 0. Similarly, well setup does not see current ink list or rosette list from HardwareConfig.

**Root Cause**: Several pages either:
- Have their own independent plate/ink/rosette widgets populated at init time only
- Do not implement `set_hardware_config()` properly
- Populate combos from stale `WorkspaceConfig` instead of the live `HardwareConfig`

**Changes Required**:

#### File: `gui/pages/calibration.py`
- ADD a `set_hardware_config(config: HardwareConfig)` method (or fix existing one)
- On receipt of HardwareConfig, update the plate format combo to match `config.plate_format`
- Remove the independent plate format combo if it duplicates Page 0's selection; instead show a read-only label displaying the current plate format from HardwareConfig
- If the combo must remain (for override scenarios), auto-select the matching format on config change and emit a visual indicator that it matches Page 0
- Ensure `_on_plate_changed()` still works for well list / corner well calculation using the HardwareConfig's plate_format

#### File: `gui/pages/print_well_setup.py`
- In `set_hardware_config()` (or `set_workspace()`), refresh all ink-related combos from `config.ink_library`
- Refresh all rosette-related combos/tables from `config.rosette_library`
- When populating sub-well ink combos in the rosette editor, pull from `HardwareConfig.ink_library` not a stale snapshot
- Store `self._hw_config` reference and use it as the authoritative source for ink names, rosette names, and plate format
- Add a `_refresh_from_hardware_config()` helper that updates: plate view format, ink combo options, rosette combo options, pump-related labels

#### File: `gui/pages/print_objects.py`
- In `set_hardware_config()`, update ink combo options from `config.ink_library`
- Update rosette-related references
- Update well diameter in preview from `config.plate_format`

#### File: `gui/app.py`
- Audit `_propagate_hardware_config()` to confirm ALL pages receive the config
- Ensure calibration page is not skipped in the propagation loop
- Add logging to confirm each page's `set_hardware_config()` is called

---

### Issue 2: XY Jog Step Size Display Mismatch

**Problem**: The displayed step size in the jog controls does not match the actual motion sent to the stage. User selects e.g. "50 µm" but the stage moves a different amount.

**Root Cause Analysis**: The `_jog_xy()` method converts µm → microsteps via:
```
step_steps = step_um * self._microsteps_per_micron
```
But the display readout converts back via:
```
ux = zx / self._microsteps_per_micron
```
The issue is likely that:
1. `_microsteps_per_micron` is not being set correctly from the controller protocol (defaults to 10.0 but may not match actual hardware)
2. The XY step combo labels show µm values but the conversion factor used for display doesn't match the one used for commanding
3. Position readback happens on a 300ms poll timer, so the displayed delta after a jog appears wrong

**Changes Required**:

#### File: `gui/pages/jog_control.py`
- Add a step-size verification display: after each jog command, show "Requested: X µm" next to the position readout so the user sees what was commanded vs. the position delta
- Add a `_last_jog_step_um` attribute that tracks the most recent commanded step in µm
- In `_update_position()`, compute delta from previous position and display it alongside "Step: X µm" so user can see actual vs requested
- Ensure `set_microsteps_per_micron()` is called by `app.py` when the protocol is loaded AND when HardwareConfig changes
- Add a read-only label in the context panel showing the current conversion factor: "Scale: {self._microsteps_per_micron} steps/µm"
- Add input validation: if `_microsteps_per_micron` is still at default and no protocol has been loaded, show a warning banner: "⚠ Conversion factor not calibrated — step sizes may be inaccurate"

#### File: `gui/app.py`
- Confirm that the microsteps_per_micron value from the controller protocol JSON is propagated to jog_control page on startup AND on protocol change
- If the value comes from `StageController.protocol_params`, wire a callback that updates the jog page when the protocol is loaded

#### File: `SupportClasses/StageController.py` (verify only)
- Confirm `move_xy_relative()` sends the exact microstep count computed by the jog page without any additional rounding/scaling
- Log the exact values sent to `XYStage.move_stage_relative()`

---

### Issue 3: Hardware Setup — Config File Browser in Context Panel

**Problem**: The hardware setup page has Save/Load buttons but no file browser showing available saved configs. Users must use a file dialog each time.

**Changes Required**:

#### File: `gui/pages/hardware_setup.py`
- Add a new section at the TOP of the context panel (left box): "Saved Configurations"
- Create a `QListWidget` that scans `config/hardware/` for `*.json` files on init and on refresh
- Each list item shows the config name (parsed from JSON `config_name` field) and filename
- Single-click selects; double-click or "Load" button applies the config
- Add a "Refresh" button to rescan the directory
- Add a "Delete" button (with confirmation) to remove a config file
- When saving, auto-refresh the list
- Show the currently active config highlighted in the list
- Add a `_scan_config_directory()` method that reads `config/hardware/*.json`, extracts `config_name` from each, and populates the list
- Wire the existing `_save_config()` to auto-refresh the list after save
- Wire `_load_config()` as an alternative triggered by list selection (bypass file dialog)

---

### Issue 4: Well Setup — Remove XZ and ZY Projections

**Problem**: The well setup tab shows XZ and ZY projection views that are not needed. Only the XY top-down plate view is required.

**Changes Required**:

#### File: `gui/pages/print_well_setup.py`
- Remove the `MiniProjectionView` widgets for ZY and XZ projections
- Remove the splitter layout that arranges the three projections in an L-shape
- Make the interactive plate view (`WellPlateView`) fill the entire top area
- Remove imports of `MiniProjectionView` if no longer used
- Adjust the main layout: plate view on top (takes full width), selection actions / editor / summary below
- Keep the well plate view's interactive features (multi-select, rubber-band, color-coding, right-click menus)
- Simplified layout:
  ```
  ┌────────────────────────────────────────────────────┐
  │  Interactive Plate View (XY) — full width          │
  ├────────────────────────────────────────────────────┤
  │  Selection Actions + Rosette Editor                │
  ├────────────────────────────────────────────────────┤
  │  Well Bottom Detection                             │
  ├────────────────────────────────────────────────────┤
  │  Assignment Summary Table                          │
  └────────────────────────────────────────────────────┘
  ```

---

### Issue 5: Print Objects — Well Preview Overhaul

**Problem**: The well preview in the print objects panel is too small, axes don't align with the XY view, projections don't match, and objects are in the wrong position relative to the preview. Users cannot expand or zoom the preview.

**Changes Required**:

#### File: `gui/pages/print_objects.py`
- **Layout restructure**: Move the preview to be the dominant widget using a `QSplitter`
  - LEFT: Well preview (XY only — large, expandable, zoomable)
  - RIGHT: Objects list panel
  - BOTTOM-LEFT or collapsed: Object designer (below preview or in a collapsible panel)
- **Remove XZ and ZY projections from the print objects tab** — show only the XY top-down view matching Issue 4's approach. If projections are needed, they belong only in Print Monitor during execution.
- **Make preview expandable**: Use `QSplitter` so user can drag to resize preview vs. objects list
- **Add zoom controls to the preview canvas**:
  - Mouse wheel zoom in/out
  - Zoom-to-fit button
  - Zoom percentage indicator
  - Pan by click-drag on empty space
- **Fix axis alignment**: Ensure the preview canvas X-axis is horizontal-right and Y-axis is vertical-up, matching standard Cartesian orientation and the well plate view
- **Well boundary**: Draw the well circle at the correct diameter from `HardwareConfig.plate_format`, centered in the view
- **Objects list to the RIGHT of well preview** (not below it):
  ```
  ┌──────────────────────────────┬──────────────────────────┐
  │  Well Preview (XY only)      │  Objects in This Print   │
  │  ┌────────────────────────┐  │  ┌──────────────────────┐│
  │  │     [zoom][fit][pan]   │  │  │ 1. ● Base Scaffold   ││
  │  │                        │  │  │ 2. ◯ Cell Ring       ││
  │  │    ┌────────┐          │  │  │ 3. ▦ Grid Fill       ││
  │  │    │ well Ø │          │  │  │                      ││
  │  │    │ objects│          │  │  │ [▲][▼][Edit][Dup][✕] ││
  │  │    └────────┘          │  │  └──────────────────────┘│
  │  │                        │  │                          │
  │  └────────────────────────┘  │  Summary: 3 obj | 2.3µL │
  ├──────────────────────────────┴──────────────────────────┤
  │  Object Designer (collapsible)                          │
  │  Type: [●][╱][◯][◎][▦][📄]  Params: [...]  [Add]      │
  └─────────────────────────────────────────────────────────┘
  ```
- **Add `wheelEvent` override** to the preview for zoom
- **Add `mousePressEvent`/`mouseMoveEvent`** for pan
- **Store view transform** (scale, translate) and apply in `paintEvent`

---

### Issue 6: Out-of-Bounds Object Flashing Red

**Problem**: When a print object extends beyond the well boundary, the user gets no visual warning. The object should flash red in the objects list.

**Changes Required**:

#### File: `gui/pages/print_objects.py`
- Add a `_check_bounds(entry: dict) -> bool` method that:
  - Gets the well diameter from HardwareConfig plate format
  - Computes the bounding box of the object trajectory
  - Checks if any point exceeds the well radius from center (0,0)
  - Returns `True` if in-bounds, `False` if out-of-bounds
- In `_refresh_objects_list()`, for each object entry:
  - If `_check_bounds()` returns False, set the list item background to a flashing red
  - Use a `QTimer` (500ms interval) that toggles the background color between red (`#f38ba8`) and the normal background for all out-of-bounds items
  - Show a warning icon (⚠) next to the object name
- In the preview canvas:
  - Draw out-of-bounds trajectory segments in red/dashed style
  - Show the well boundary circle prominently
- Add an `_oob_flash_timer: QTimer` attribute initialized on setup
- Add `_oob_items: list[int]` tracking which object indices are out of bounds
- The flash timer callback toggles a `_flash_state: bool` and calls `_refresh_objects_list_colors()`

---

### Issue 7: GUI Section Title Alignment & Styling

**Problem**: Across all pages, the QGroupBox titles and section headers are not consistently aligned or styled.

**Changes Required**:

#### File: `gui/styles.py`
- Add a global `SECTION_TITLE_STYLE` constant that defines consistent styling for all QGroupBox titles:
  ```python
  SECTION_TITLE_STYLE = f"""
      QGroupBox {{
          font-size: 11pt;
          font-weight: 600;
          color: {COLORS['text']};
          border: 1px solid {COLORS['surface1']};
          border-radius: 8px;
          margin-top: 12px;
          padding: 16px 10px 10px 10px;
      }}
      QGroupBox::title {{
          subcontrol-origin: margin;
          subcontrol-position: top left;
          left: 12px;
          padding: 2px 8px;
          background-color: {COLORS['base']};
          border-radius: 4px;
      }}
  """
  ```
- Add a `CONTEXT_SECTION_LABEL_STYLE` for context panel section headers
- Add a `PAGE_TITLE_STYLE` for the top bar page title display

#### Files: ALL page files
- Replace all per-page `_group_style()` static methods with `from gui.styles import SECTION_TITLE_STYLE`
- Replace all inline QGroupBox stylesheet strings with the centralized constant
- In each page, ensure QGroupBox titles use consistent capitalization (Title Case)
- Audit every `QLabel` used as a section header and apply consistent font size, weight, and color
- Specific files to update:
  - `hardware_setup.py` — replace `_group_style()` with centralized style
  - `calibration.py` — same
  - `jog_control.py` — same for context panel sections
  - `print_setup.py` — same
  - `print_workspace.py` — same
  - `print_objects.py` — same
  - `print_well_setup.py` — same
  - `print_monitor.py` — same
  - `settings_page.py` — same (card frames)
  - `dashboard.py` — same

---

### Issue 8: Inks Assigned to Pumps

**Problem**: Currently inks exist in a library and pumps have an ink selection, but the assignment relationship is not enforced or clear. Inks should be explicitly assigned to specific pumps.

**Changes Required**:

#### File: `SupportClasses/HardwareConfig.py`
- The existing `PumpChannelConfig.ink` field already holds the ink assignment — this is correct
- Add a convenience property to `HardwareConfig`:
  ```
  pump_ink_map: dict[str, str | None]  # {"P1": "Hydrogel A", "P2": "MSC Cells", "P3": None}
  ```
  Returns the pump→ink_name mapping for quick lookup
- Add a reverse lookup property:
  ```
  ink_pump_map: dict[str, str]  # {"Hydrogel A": "P1", "MSC Cells": "P2"}
  ```
  Returns ink_name→pump_id mapping
- Add validation: each ink can only be assigned to ONE pump at a time
- Add validation: warn if an ink in the library is not assigned to any pump

#### File: `gui/pages/hardware_setup.py`
- In the Pumps section, make the ink assignment combo more prominent
- Add visual feedback: when an ink is assigned to a pump, show it grayed-out in other pump combos (can't double-assign)
- Add a summary label below the pumps section: "P1→Hydrogel A, P2→MSC Cells, P3→(none)"
- When the ink library changes (add/edit/delete), refresh all pump ink combos
- If an ink is deleted that was assigned to a pump, clear that pump's ink assignment and warn

---

### Issue 9: Needle Selection After Pump Assignments + Channel-Pump Mapping

**Problem**: Currently needle is selected before pumps. But if the needle is multi-channel, each channel needs to be assigned to a specific pump. Single-bore needles also need pump assignment. Therefore needle selection should come AFTER pump setup.

**Changes Required**:

#### File: `SupportClasses/HardwareConfig.py`
- Add a new field to `HardwareConfig`:
  ```
  needle_channel_pump_map: dict[int, str]  # {0: "P1", 1: "P2", 2: "P3"}
  ```
  Maps needle channel index → pump_id
- For single-channel needles: `{0: "P1"}` (must assign exactly one pump)
- For multi-channel needles: each channel must map to a unique enabled pump
- Update `validate()`:
  - Every needle channel must be mapped to an enabled pump with a syringe
  - No two channels can map to the same pump
  - Number of channels must match `needle.channels`
- Update `to_dict()` / `from_dict()` to serialize/deserialize `needle_channel_pump_map`

#### File: `gui/pages/hardware_setup.py`
- **Reorder UI sections** to:
  1. Setup Name & Notes
  2. Well Plate Format
  3. Ink Library
  4. Pump Channels (P1/P2/P3) — with syringe + ink assignment
  5. Needle Configuration — gauge + length + channels
  6. Needle Channel → Pump Mapping (dynamic, appears after needle selection)
  7. Rosette Library
  8. Actions + Validity
- **New Section: "Needle Channel Assignment"**
  - Dynamically shows N rows based on `channels_spin.value()`
  - Each row: "Channel {i+1}: [Pump combo]" where combo lists only enabled pumps
  - For single-channel (channels=1): single row "Bore → [P1/P2/P3]"
  - For multi-channel: one row per channel
  - Validation: all channels must be assigned, no duplicate pump assignments
  - Visual: green checkmark when all channels assigned, yellow warning otherwise
- Move needle gauge/length/channels section BELOW the pump channels section
- On needle channel count change, rebuild the channel-pump mapping rows
- On pump enable/disable change, refresh the available pumps in channel mapping combos

---

### Issue 10: Print Plan of Action

**Problem**: Complex prints require multiple rounds of ink pickup. Users need to configure max ink per run, wash cycles between refills, and waste/buffer management.

**Changes Required**:

#### File: NEW `SupportClasses/PrintPlanOfAction.py`
- Create a new class `PrintPlanOfAction` that generates the execution sequence:
  ```
  PrintPlanOfAction
    ├── max_ink_volume_uL: dict[str, float]     # Max ink per pump per run (user-set)
    ├── wash_after_every_refill: bool            # Whether to wash needle between ink loads
    ├── waste_after_task: list[str]              # Which tasks trigger waste disposal
    ├── refill_buffer_after: list[str]           # When to refill buffer layer
    ├── steps: list[PlanStep]                    # Computed ordered execution steps
    └── total_runs: int                          # How many ink pickup cycles needed
  ```
- `PlanStep` types:
  - `LOAD_INK` — move to ink well, aspirate up to max_ink_volume
  - `WASH` — move to wash well, dispense/aspirate wash cycles
  - `WASTE` — move to waste well, dispense waste
  - `REFILL_BUFFER` — move to buffer well, aspirate buffer
  - `PRINT` — execute print objects in assigned wells
  - `RETURN_HOME` — move to safe position
- Add a `generate_plan()` method that:
  - Takes: HardwareConfig, WellSetupModel (well assignments), print file objects, user preferences
  - Computes total ink needed per pump across all assigned wells
  - Divides into runs based on max_ink_volume_uL per pump
  - Inserts wash/waste/buffer steps based on user preferences
  - Returns ordered list of PlanSteps with well targets and volumes
- Add a `validate_plan()` method that checks:
  - All required service wells are assigned (wash, waste, buffer)
  - Ink wells exist for incremental mode pumps
  - Volumes don't exceed syringe capacity
  - At least one print well is assigned
- Add `estimate_time()` for rough time estimate based on travel distances and pump speeds
- Add `to_dict()` / `from_dict()` for serialization

#### File: `gui/pages/print_well_setup.py`
- Add a new section: "Print Plan of Action" (collapsible group box)
- UI elements:
  - Per-pump "Max Ink Per Run": `QDoubleSpinBox` (µL), default = syringe capacity
  - "Wash After Every Refill": `QCheckBox`
  - "Waste Before Refill": `QCheckBox`
  - "Refill Buffer After Waste": `QCheckBox`
  - "Generate Plan" button → calls `PrintPlanOfAction.generate_plan()`
  - Plan display: scrollable list showing each step in order with icons
  - Summary: "Total runs: N | Estimated time: X min | Total ink: Y µL"
- Plan steps displayed as a numbered list with color-coded icons:
  - 🔵 LOAD_INK → "Load 5.0 µL Hydrogel A from A8 into P1"
  - 🟡 WASH → "Wash needle at B1 (3 cycles)"
  - 🔴 WASTE → "Dispense waste at C1"
  - 🟣 REFILL_BUFFER → "Refill buffer from D1 (2.0 µL)"
  - 🟢 PRINT → "Print wells B2, B3, B4 (run 1/3)"
- Auto-regenerate plan when well assignments or hardware config changes

#### File: `gui/pages/print_setup.py`
- The "Send to Monitor" button should include the plan of action in the `PrintJob`
- Validate plan before allowing send (see Issue 11)

---

### Issue 11: Well Setup Validation Before Sending to Monitor

**Problem**: Currently there's no validation gate between well setup and the print monitor. Invalid configurations (missing ink wells, out-of-bounds objects, missing service wells) can be sent.

**Changes Required**:

#### File: `gui/pages/print_well_setup.py`
- Add a `validate() -> tuple[bool, list[str]]` method that checks:
  1. **Ink assignments**: For each pump in incremental mode, the ink assigned to that pump must have a corresponding ink well in the plate layout. Check that at least one well has role=INK with the matching ink name.
  2. **Object bounds**: All print objects assigned to wells must fit within the well diameter. Use the same `_check_bounds()` logic from Issue 6.
  3. **Service wells present**: Based on the Plan of Action:
     - If wash is required → at least one well with role=WASH must exist
     - If waste disposal needed → at least one well with role=WASTE must exist
     - If buffer refill needed → at least one well with role=BUFFER must exist
  4. **Print wells exist**: At least one well must have role=PRINT with a print file assigned
  5. **Pump-needle mapping**: All pumps used by print objects must be mapped to needle channels (from Issue 9)
  6. **Syringe capacity**: Max ink per run must not exceed syringe volume
  7. **Plan generated**: A plan of action must have been generated and be valid
- Return `(is_valid, list_of_issues)` — same pattern as `HardwareConfig.validate()`

#### File: `gui/pages/print_setup.py`
- Before emitting `job_ready(PrintJob)`, call `tab_wells.validate()`
- If validation fails:
  - Show a `QMessageBox` listing all issues
  - Do NOT emit the signal
  - Highlight the Well Setup tab with a red badge
- If validation passes:
  - Show a confirmation dialog with the plan summary
  - On confirm, build the `PrintJob` including the plan of action and emit `job_ready`
- Add a "Validate Setup" button that runs validation without sending to monitor
- Show validation status in the context panel: "✓ Ready to print" / "⚠ 3 issues"

---

## 3. File Change Summary

| File | Issues | Action | Estimated Change |
|------|--------|--------|-----------------|
| `gui/styles.py` | 7 | MODIFY — add centralized section styles | +40 lines |
| `gui/app.py` | 1, 2 | MODIFY — propagation audit + µm factor wiring | +30 lines |
| `gui/pages/hardware_setup.py` | 1, 3, 7, 8, 9 | MAJOR REWRITE — reorder, file browser, channel mapping | +350 / -150 lines |
| `gui/pages/jog_control.py` | 2, 7 | MODIFY — step display fix, verification, styles | +60 lines |
| `gui/pages/calibration.py` | 1, 7 | MODIFY — HW config plate sync, styles | +40 / -20 lines |
| `gui/pages/print_objects.py` | 1, 5, 6, 7 | MAJOR REWRITE — preview overhaul, bounds check, styles | +400 / -200 lines |
| `gui/pages/print_well_setup.py` | 1, 4, 7, 10, 11 | MAJOR MODIFY — remove projections, plan UI, validation | +350 / -100 lines |
| `gui/pages/print_setup.py` | 10, 11 | MODIFY — validation gate, plan in PrintJob | +80 lines |
| `gui/pages/print_workspace.py` | 7 | MINOR — styles only | +10 lines |
| `gui/pages/print_monitor.py` | 7 | MINOR — styles only | +10 lines |
| `gui/pages/dashboard.py` | 7 | MINOR — styles only | +10 lines |
| `gui/pages/settings_page.py` | 7 | MINOR — styles only | +10 lines |
| `SupportClasses/HardwareConfig.py` | 8, 9 | MODIFY — pump-ink map, channel map, validation | +80 lines |
| `SupportClasses/PrintPlanOfAction.py` | 10 | NEW — plan generation + validation | +400 lines |

**Total: ~1,870 added, ~470 removed, ~2,340 lines touched**

---

## 4. Session Breakdown

### Session 1: Styles + Config Propagation Audit (3-4 hrs)

**Issues Covered**: 1, 7 (partial), 2 (partial)

| Task | Description |
|------|-------------|
| S1.1 | Add `SECTION_TITLE_STYLE`, `CONTEXT_SECTION_LABEL_STYLE`, `PAGE_TITLE_STYLE` to `gui/styles.py` |
| S1.2 | Replace all `_group_style()` methods across ALL page files with centralized import |
| S1.3 | Audit `app.py._propagate_hardware_config()` — add logging, confirm all pages receive config |
| S1.4 | Fix `calibration.py.set_hardware_config()` — sync plate format combo from HardwareConfig |
| S1.5 | Fix `print_well_setup.py.set_hardware_config()` — refresh ink combos and rosette combos from HardwareConfig |
| S1.6 | Fix `print_objects.py.set_hardware_config()` — refresh ink combo, well diameter from HardwareConfig |
| S1.7 | Verify `app.py` propagates `microsteps_per_micron` to jog page from controller protocol |
| S1.8 | Test: change plate format on Page 0, verify calibration page updates, verify well setup updates |

**Output Patch**: `patches/v724/patch_s1_styles_and_propagation.py`
**Files Modified**: `gui/styles.py`, `gui/app.py`, `gui/pages/calibration.py`, `gui/pages/print_well_setup.py`, `gui/pages/print_objects.py`, ALL page files (style replacement)

---

### Session 2: Jog Step Fix + Hardware Setup File Browser (3-4 hrs)

**Issues Covered**: 2, 3

| Task | Description |
|------|-------------|
| S2.1 | Add step verification display to jog page — show "Requested: X µm" after each jog |
| S2.2 | Add conversion factor label to jog context panel |
| S2.3 | Add warning banner if conversion factor is still at default |
| S2.4 | Verify `StageController.move_xy_relative()` sends exact microstep counts |
| S2.5 | Add config file browser `QListWidget` to hardware_setup context panel |
| S2.6 | Implement `_scan_config_directory()` to list `config/hardware/*.json` |
| S2.7 | Wire list selection → load config, double-click → load + apply |
| S2.8 | Wire save → auto-refresh list, add delete button with confirmation |
| S2.9 | Highlight currently active config in the file list |
| S2.10 | Test: select different step sizes, verify actual movement matches display |

**Output Patch**: `patches/v724/patch_s2_jog_and_filebrowser.py`
**Files Modified**: `gui/pages/jog_control.py`, `gui/pages/hardware_setup.py`

---

### Session 3: Pump-Ink Assignment + Needle-Channel Mapping (4-5 hrs)

**Issues Covered**: 8, 9

| Task | Description |
|------|-------------|
| S3.1 | Add `pump_ink_map` and `ink_pump_map` properties to `HardwareConfig` |
| S3.2 | Add `needle_channel_pump_map` field to `HardwareConfig` with serialization |
| S3.3 | Update `HardwareConfig.validate()` for unique ink-per-pump and channel-pump mapping |
| S3.4 | Reorder hardware_setup.py UI: Name → Plate → Inks → Pumps → Needle → Channel Map → Rosettes → Actions |
| S3.5 | Make pump ink combos exclusive — gray out inks already assigned to other pumps |
| S3.6 | Add pump-ink summary label below pumps section |
| S3.7 | Build dynamic "Needle Channel Assignment" section — N rows based on channel count |
| S3.8 | Channel mapping combos list only enabled pumps; validate uniqueness |
| S3.9 | On needle channel count change, rebuild channel mapping rows |
| S3.10 | On pump enable/disable, refresh channel mapping pump combos |
| S3.11 | Update `_apply_config_to_ui()` to restore channel map after pumps and needle |
| S3.12 | Update `_rebuild_config()` to capture channel map state |
| S3.13 | Test: configure multi-channel needle, assign channels to pumps, save/load round-trip |

**Output Patch**: `patches/v724/patch_s3_pump_ink_needle_channels.py`
**Files Modified**: `SupportClasses/HardwareConfig.py`, `gui/pages/hardware_setup.py`

---

### Session 4: Print Objects Preview Overhaul + Bounds Check (4-5 hrs)

**Issues Covered**: 4, 5, 6

| Task | Description |
|------|-------------|
| S4.1 | Remove XZ and ZY projections from `print_well_setup.py` — plate view fills full width |
| S4.2 | Remove XZ and ZY projections from `print_objects.py` — XY only preview |
| S4.3 | Restructure print_objects layout: QSplitter with preview LEFT, objects list RIGHT |
| S4.4 | Add zoom (mouse wheel) to preview canvas — store scale factor, apply in paint |
| S4.5 | Add pan (middle-click drag or Ctrl+drag) to preview canvas |
| S4.6 | Add zoom-to-fit button and zoom percentage indicator |
| S4.7 | Fix axis alignment — ensure X=right, Y=up in the preview |
| S4.8 | Draw well boundary circle at correct diameter from HardwareConfig |
| S4.9 | Implement `_check_bounds()` method for object-in-well validation |
| S4.10 | Add `_oob_flash_timer` (QTimer, 500ms) that toggles red background on OOB list items |
| S4.11 | In preview, draw OOB trajectory segments in red/dashed |
| S4.12 | Object designer moves to collapsible panel below preview |
| S4.13 | Test: add objects that exceed well boundary, verify flash + red rendering |

**Output Patch**: `patches/v724/patch_s4_preview_overhaul.py`
**Files Modified**: `gui/pages/print_objects.py`, `gui/pages/print_well_setup.py`

---

### Session 5: Print Plan of Action + Well Validation (5-6 hrs)

**Issues Covered**: 10, 11

| Task | Description |
|------|-------------|
| S5.1 | Create `SupportClasses/PrintPlanOfAction.py` with `PlanStep` enum and `PrintPlanOfAction` class |
| S5.2 | Implement `generate_plan()` — compute ink needs, divide into runs, insert service steps |
| S5.3 | Implement `validate_plan()` — check service wells, ink wells, syringe capacity |
| S5.4 | Implement `estimate_time()` for rough time estimation |
| S5.5 | Add serialization: `to_dict()` / `from_dict()` |
| S5.6 | Add "Print Plan of Action" collapsible section to `print_well_setup.py` |
| S5.7 | Build plan configuration UI: max ink per run spinboxes, wash/waste/buffer checkboxes |
| S5.8 | Build plan display: numbered step list with color-coded icons |
| S5.9 | Add plan summary: total runs, estimated time, total ink |
| S5.10 | Auto-regenerate plan on well assignment or hardware config change |
| S5.11 | Implement `WellSetupTab.validate()` comprehensive validation method |
| S5.12 | Wire validation into `print_setup.py` "Send to Monitor" button |
| S5.13 | Show validation failure dialog with issue list |
| S5.14 | Add "Validate Setup" standalone button |
| S5.15 | Include `PrintPlanOfAction` in `PrintJob` for monitor execution |
| S5.16 | Test: create setup with missing service wells → validation should catch |
| S5.17 | Test: create setup with incremental pump but no ink well → validation should catch |
| S5.18 | Test: verify plan step count matches expected for multi-run scenario |

**Output Patch**: `patches/v724/patch_s5_plan_and_validation.py`
**Files Modified**: `SupportClasses/PrintPlanOfAction.py` (NEW), `gui/pages/print_well_setup.py`, `gui/pages/print_setup.py`

---

### Session 6: Integration Testing + Documentation (2-3 hrs)

| Task | Description |
|------|-------------|
| S6.1 | Full end-to-end workflow test: HW setup → calibration → print objects → well setup → validate → send to monitor |
| S6.2 | Test hardware config propagation: change plate format on Page 0, verify all pages update |
| S6.3 | Test needle channel mapping with 1, 2, and 3 channels |
| S6.4 | Test jog step verification display |
| S6.5 | Test out-of-bounds flash in print objects |
| S6.6 | Test plan of action with multi-run ink loading scenario |
| S6.7 | Test validation gates — try sending invalid setup to monitor |
| S6.8 | Run full test suite |
| S6.9 | Update `README_V724.md` |
| S6.10 | Update `ARCHITECTURE_V724.md` |

**Output Patch**: `patches/v724/patch_s6_tests_and_docs.py`
**Files Created**: `README_V724.md`, tests, `ARCHITECTURE_V724.md` addendum

---

## 5. Dependency Graph

```
Session 1 (Styles + Propagation)
    │
    ├──→ Session 2 (Jog Fix + File Browser) ──→ ┐
    │                                            │
    ├──→ Session 3 (Pump-Ink + Needle Channels) ─┤
    │                                            │
    └──→ Session 4 (Preview + Bounds) ───────────┤
                                                 │
                              Session 5 (Plan + Validation)
                                                 │
                              Session 6 (Integration Tests)
```

- Session 1 must be first (all others depend on consistent styles + propagation fixes)
- Sessions 2, 3, 4 can run in parallel (different files)
- Session 5 depends on Session 3 (pump-ink mapping needed for plan validation)
- Session 6 depends on all prior sessions

---

## 6. Schedule Estimate

| Session | Hours | Priority | Depends On |
|---------|-------|----------|------------|
| S1 | 3-4 | Critical | — |
| S2 | 3-4 | Critical | S1 |
| S3 | 4-5 | High | S1 |
| S4 | 4-5 | High | S1 |
| S5 | 5-6 | High | S1, S3 |
| S6 | 2-3 | Critical | All |

**Total: 21-27 hours across 6 sessions**

---

## 7. Patch File Structure

Each session produces a self-contained Python patch script:

```
patches/v724/
├── apply_all_v724_patches.py         # Master runner (runs S1-S6 in order)
├── patch_s1_styles_and_propagation.py
├── patch_s2_jog_and_filebrowser.py
├── patch_s3_pump_ink_needle_channels.py
├── patch_s4_preview_overhaul.py
├── patch_s5_plan_and_validation.py
├── patch_s6_tests_and_docs.py
└── MEBP_v724_UPDATE_PLAN.md          # This document
```

Each patch script:
- Detects the project root automatically
- Validates prerequisite files exist
- Applies changes via string replacement (str_replace pattern)
- Reports success/failure per change
- Is idempotent (safe to run multiple times)

---

## 8. GitHub Folder Structure (Post-Update)

```
McGheeLab/MEBP/
├── main.py
├── settings.json
├── config/
│   ├── controllers/
│   ├── hardware/
│   │   ├── needles.json
│   │   ├── syringes.json
│   │   └── sample_setup.json
│   └── prints/
├── SupportClasses/
│   ├── HardwareConfig.py              # MODIFIED: pump-ink map, channel map
│   ├── PrintPlanOfAction.py           # NEW: plan generation + validation
│   ├── PrintFileManager.py
│   ├── auto_layout.py
│   ├── StageController.py
│   ├── PrintManager.py
│   ├── PhysicalModels.py
│   ├── GeometryEngine.py
│   ├── TrajectoryPlanner.py
│   ├── WellPlate.py
│   ├── WellSetup.py
│   └── ...
├── gui/
│   ├── app.py                         # MODIFIED: propagation audit
│   ├── styles.py                      # MODIFIED: centralized section styles
│   └── pages/
│       ├── hardware_setup.py          # MAJOR: reorder, file browser, channels
│       ├── dashboard.py               # MINOR: styles
│       ├── jog_control.py             # MODIFIED: step verification
│       ├── calibration.py             # MODIFIED: HW config sync
│       ├── print_setup.py             # MODIFIED: validation gate
│       ├── print_workspace.py         # MINOR: styles
│       ├── print_objects.py           # MAJOR: preview overhaul, bounds
│       ├── print_well_setup.py        # MAJOR: remove projections, plan, validation
│       ├── print_monitor.py           # MINOR: styles
│       └── settings_page.py           # MINOR: styles
├── patches/v724/
│   ├── apply_all_v724_patches.py
│   ├── patch_s1_styles_and_propagation.py
│   ├── patch_s2_jog_and_filebrowser.py
│   ├── patch_s3_pump_ink_needle_channels.py
│   ├── patch_s4_preview_overhaul.py
│   ├── patch_s5_plan_and_validation.py
│   ├── patch_s6_tests_and_docs.py
│   └── MEBP_v724_UPDATE_PLAN.md
├── tests/
│   ├── test_v724_propagation.py
│   ├── test_v724_plan_of_action.py
│   ├── test_v724_validation.py
│   └── test_v724_integration.py
└── README_V724.md
```
