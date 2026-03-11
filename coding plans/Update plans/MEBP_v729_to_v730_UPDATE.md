# MEBP v7.2.9 → v7.3.0 Update Plan

**Date:** 2026-03-10
**Branch:** Version-7.2.9
**Author:** Session work with Claude Code

---

## Overview

This update package redesigns the **Print Plan of Action** system to be the single source of truth for all print execution behavior, adds a **Finalize tab** to the print setup workflow, and addresses a **critical needle safety issue** during well-to-well travel. Previously, execution parameters were scattered across HardwareConfig (ink swap strategy), the context panel (print settings), and the Well Setup tab (plan generation). v7.2.9 consolidates everything into a comprehensive `PrintExecutionConfig` with full UI controls in a new Finalize tab, covering ink swap sequences, ink gathering, Z travel behavior, XY travel, plunge-in buffering, and final cleanup.

### Four Major Areas

1. **Print Plan of Action redesign** — New `PrintExecutionConfig` replaces `PlanPreferences`, 5 new step types, 6 new dataclasses
2. **Print Setup UI overhaul** — New 4th tab (Finalize) with two-column layout; context panel simplified to display options only; Plan of Action section removed from Well Setup tab
3. **Trajectory planner updates** — Handlers for all new step types, simplified service logic
4. **Needle safety fix** — XY arrival confirmation before Z descent prevents needle breakage during well-to-well travel

---

## 1. PrintPlanOfAction.py — Complete Rewrite

### Problem
The old `PrintPlanOfAction` only handled basic service steps (waste/wash/buffer/load_ink/print/return_home) with a simple `PlanPreferences` dataclass. There was no way to specify:
- Per-ink pickup volumes or extra percentage
- Z-axis travel behavior (safe Z, fast travel, plunge buffering)
- XY fast travel between wells
- Final cleanup sequences
- Ink swap strategy (was defined in HardwareConfig, not plan-of-action)

### Changes

**New Data Classes (6 total):**

| Class | Purpose |
|-------|---------|
| `InkSwapStrategy` | Configurable ink swap sequence — 6 boolean step toggles + 4 volume settings (moved from HardwareConfig.py) |
| `InkGatherConfig` | Per-ink-type pickup config: `max_pickup_uL`, `extra_percent`, `effective_volume()` method |
| `ZTravelConfig` | Z behavior: `safe_z_mm`, `top_z_mm`, `wait_for_z_confirm`, `fast_z_exit_well`, `fast_z_enter_well`, `use_plunge_buffer`, `plunge_buffer_um` |
| `XYTravelConfig` | XY travel: `fast_xy_travel`, `fast_xy_speed_mm_s` |
| `FinalCleanupConfig` | End-of-print cleanup: `do_waste`, `do_wash`, `do_dry_run`, `wash_cycles`, `wash_time_per_cycle_s` |
| `PrintExecutionConfig` | **Master config** — aggregates all sub-configs + service toggles, speeds, per-ink overrides. Has `from_preferences()` migration method and `get_ink_gather()` lookup. |

**New Step Types (5 total):**

| Step Type | Description |
|-----------|-------------|
| `GATHER_INK` | Ink pickup with per-ink extra % and max volume — replaces raw LOAD_INK for ink gather operations |
| `FINAL_CLEANUP` | End-of-print cleanup sequence with configurable sub-steps (waste, wash, dry run) |
| `TRAVEL_XY` | Explicit fast XY move between wells — makes inter-well travel a first-class plan step |
| `MOVE_SAFE_Z` | Raise to safe Z height for travel — explicit Z-raise step instead of implicit |
| `INK_SWAP` | Full ink swap sequence with ordered sub-steps from `InkSwapStrategy` config |

**Enriched `PlanStep`:**

New fields added to every plan step:
- `z_behavior` — `"fast"`, `"slow"`, `"plunge"`, `"default"`
- `wait_for_z_confirm` — wait for Z position confirmation before proceeding
- `fast_z_entry` / `fast_z_exit` — Z speed into/out of wells
- `plunge_buffer_um` — micron buffer distance for slow plunge-in
- `travel_mode` — `"fast"` or `"slow"` for XY movement
- `travel_speed_mm_s` — XY travel speed for this specific step
- `extra_percent` — ink extra % for gather steps
- `max_pickup_uL` — max pickup volume for gather steps
- `sub_steps` — ordered list of sub-operations for compound steps (INK_SWAP, FINAL_CLEANUP)

**Plan colors and icons** updated for all 11 step types (Catppuccin Mocha palette).

**`generate_plan()` updated** to accept optional `execution_config: PrintExecutionConfig` — takes priority over legacy `PlanPreferences` when provided.

**`_compute()` rewritten** to use `PrintExecutionConfig` for comprehensive plan generation:
- Applies per-ink gather configs with extra % calculations
- Inserts MOVE_SAFE_Z, TRAVEL_XY steps between wells
- Generates INK_SWAP compound steps using `InkSwapStrategy`
- Adds FINAL_CLEANUP as the last step when enabled
- Properly splits runs based on syringe capacity with per-ink volume limits

**Backward Compatibility:**
- `PlanPreferences` retained as thin wrapper
- `PrintExecutionConfig.from_preferences()` migration method
- Old step types (LOAD_INK, WASH, WASTE, REFILL_BUFFER, PRINT, RETURN_HOME) still work

---

## 2. HardwareConfig.py — InkSwapStrategy Moved Out

### Problem
`InkSwapStrategy` was defined in `HardwareConfig.py` but logically belongs with the plan of action — it controls execution behavior, not hardware configuration.

### Changes

- **Removed** the full `InkSwapStrategy` class definition (60+ lines)
- **Added** re-export for backwards compatibility:
  ```python
  # v7.2.9: InkSwapStrategy moved to PrintPlanOfAction.py as the single
  # source of truth. Re-exported here for backwards compatibility.
  from SupportClasses.PrintPlanOfAction import InkSwapStrategy  # noqa: F401
  ```
- Existing code that imports `InkSwapStrategy` from `HardwareConfig` continues to work unchanged
- `HardwareConfig.ink_swap_strategy` field still exists and is still serialized/deserialized

---

## 3. PrintTrajectoryPlanner.py — New Step Type Handlers

### Problem
The trajectory planner only knew how to handle 6 step types. The execution loop had overly complex conditional logic for service steps and the RETURN_HOME step duplicated waste/wash logic.

### Changes

**Flattened service step handling:**
- Old: single `elif stype in (WASTE, WASH, REFILL_BUFFER, LOAD_INK)` block with nested conditionals
- New: each service type gets its own `elif` branch for clarity

**New step type handlers:**

| Step Type | Implementation |
|-----------|---------------|
| `GATHER_INK` | Same as LOAD_INK but volume already includes extra % from plan generation |
| `MOVE_SAFE_Z` | Raises Z to `settings.travel_z_height` using fast Z feedrate if below safe height |
| `TRAVEL_XY` | Resolves target well position via `plate.get_well_position()`, moves XY at `step.travel_speed_mm_s * 60` (mm/min) |
| `FINAL_CLEANUP` | Iterates `step.sub_steps` list — executes waste/wash as configured |
| `INK_SWAP` | Iterates `step.sub_steps` list — executes waste/wash/buffer/ink_load in order |

**RETURN_HOME simplified:**
- Removed redundant waste + wash calls (now handled by FINAL_CLEANUP step in the plan)
- Only does travel Z raise + XY return to origin

**Comment cleanup:**
- Removed verbose v7.2.6 DSF (double-service fix) comments
- Simplified PRINT step comments to essential information

---

## 4. print_setup.py — New Finalize Tab + Context Panel Redesign

### Problem
Print settings were crammed into the narrow context panel. The plan of action had no UI. Validate & Generate buttons were buried at the bottom of the context panel. There was no way to configure ink swap, Z behavior, XY travel, or cleanup settings.

### Changes

**New Tab 4: Finalize (`_build_finalize_tab()`)**

The print setup page now has 4 tabs instead of 3:
1. Workspace (hardware summary)
2. Print Objects (object designer)
3. Well Setup (well plate configuration)
4. **Finalize** (print settings + plan of action + generate & send) ← NEW

The Finalize tab uses a **scrollable two-column layout**:

**Left Column — Print Parameters** (moved from context panel):
- **Extrusion** — volume fraction % (Fill) + speed scale %
- **Calculated** — read-only derived parameters (flow rate, speed, needle diameter, etc.)
- **Layers** — layer count + layer height spinboxes
- **Advanced** — collapsible group: Z feed rate, travel Z height, settle delay, per-pump retract/prime µL

**Right Column — Plan of Action** (entirely new):
- **Ink Swap Sequence** — checkable QGroupBox with 6 step checkboxes (waste, wash_pre, buffer, wash_post, ink_load, wash_final) + compact horizontal row of 4 volume spinboxes (Waste, Wash, Buff, Ink in µL). Tooltip shows full sequence.
- **Ink Gathering** — Extra % spinbox (default 10%), Max pickup µL spinbox (0 = syringe capacity). Per-ink overrides not yet in UI.
- **Z Travel** — Wait for Z position confirm checkbox, fast Z exit from well, fast Z entry into well, plunge buffer before print Z checkbox + buffer distance in µm spinbox
- **XY Travel** — Fast XY travel between wells checkbox + speed mm/s spinbox
- **Final Cleanup** — checkable QGroupBox with waste (eject remaining), wash needle, dry run (clear residual) options

**Bottom Section:**
- Horizontal button row: "Validate & Generate Print" (blue, styled) + "Send to Monitor" (green, styled)
- Status row: generation status label + workflow status label
- Export row: "Export G-code" + "Save JSON" buttons

**New `_build_execution_config()` method:**
Reads all Plan of Action UI widgets into a `PrintExecutionConfig` object. Reads:
- `_swap_checks` dict → `InkSwapStrategy`
- `_swap_*_vol` spinboxes → swap volumes
- `_ink_extra_pct_spin` / `_ink_max_pickup_spin` → `InkGatherConfig`
- `_z_*` checkboxes and spinbox → `ZTravelConfig`
- `_xy_*` checkbox and spinbox → `XYTravelConfig`
- `_cleanup_*` checkboxes → `FinalCleanupConfig`

**`_generate_print()` updated:**
- Calls `_build_execution_config()` before plan generation
- Passes `execution_config` to `tab_wells._generate_plan()`

**Context Panel simplified (`_build_context_panel()`):**
- **Removed** all print settings (moved to Finalize tab)
- **Now shows "Display Options" only:**
  - **Well Plate** group: "Show well labels" checkbox, "Show trajectory paths" checkbox, "Color by:" combo (Role/Ink/Status)
  - **Object Preview** group: "Show grid" checkbox, "Show axes" checkbox, "Show dimensions" checkbox

**Docstring updated** to reflect v7.2.9 4-tab layout.

**Import changes:**
- Added `QScrollArea` to PySide6.QtWidgets imports
- Added imports from `PrintPlanOfAction`: `PrintExecutionConfig`, `InkSwapStrategy`, `InkGatherConfig`, `ZTravelConfig`, `XYTravelConfig`, `FinalCleanupConfig`

---

## 5. hardware_setup.py — Ink Swap UI Removed

### Problem
The Ink Swap Strategy UI section lived in Hardware Setup but is now part of the Plan of Action in the Finalize tab.

### Changes

- **Removed** Section 4b "Ink Swap Strategy" QGroupBox (~45 lines of UI code)
  - 6 checkboxes (`self._swap_checks` dict)
  - 4 volume spinboxes (`_swap_waste_vol`, `_swap_wash_vol`, `_swap_buffer_vol`, `_swap_ink_load_vol`)
  - QFormLayout for volume rows
- **Removed** ink swap strategy building from `_rebuild_config()` (~12 lines) — `InkSwapStrategy(...)` construction from checkbox/spinbox values
- **Removed** ink swap UI restoration from `_apply_config_to_ui()` (~17 lines) — `blockSignals` + `setChecked`/`setValue` for all swap widgets
- **Removed** `InkSwapStrategy` from import statement (only `HardwareConfig`, `PumpChannelConfig` imported now)
- **Added** placeholder comments at each removal site: `# v7.2.9: Ink Swap Strategy UI moved to Print Setup → Plan of Action`

---

## 6. print_well_setup.py — Plan Section Removed + Config Propagation

### Problem
The "Print plan of action" section was built in the Well Setup tab but belongs in the Finalize tab. `_generate_plan()` only accepted legacy `PlanPreferences`.

### Changes

- **Removed** `self._build_plan_section(main)` call from `_build_ui()` — the Plan of Action UI no longer renders in the Well Setup tab
- **Updated** `_generate_plan()` signature: added optional `execution_config=None` parameter
- **Updated** docstring to document the new parameter
- `execution_config` is passed through to `PrintPlanOfAction.generate_plan()` where it takes priority over legacy preferences
- Backend methods (`_generate_plan`, `get_plan`, `validate`, `_get_plan_preferences`) remain intact — they're called by print_setup.py's `_generate_print()` during generation

---

## 7. StageController.py — XY Arrival Confirmation Method

### Problem
During well-to-well travel, the `TrajectoryExecutor` fires XY and Z commands sequentially through waypoints without confirming XY has actually reached the target position. If XY is still mid-travel when Z begins descending, the needle could hit the plate edge or the wrong well, causing **needle breakage**.

### Changes

**New method: `wait_for_xy_arrival()`**

```python
def wait_for_xy_arrival(
    self, target_x_mm: float, target_y_mm: float,
    tolerance_mm: float = 0.1, timeout_s: float = 10.0,
) -> bool:
```

- Polls the **actual** XY position (non-cached, live hardware query) every 50ms
- Computes Euclidean distance to target
- Returns `True` when position is within `tolerance_mm` (default 0.1mm)
- Returns `False` on timeout (default 10s) with a warning log
- Short-circuits to `True` if no XY stage is connected
- Uses `time.monotonic()` for reliable timing

---

## 8. PrintManager.py — Safety Check + Speed Bug Fix

### Problem 1: Needle safety
`TrajectoryExecutor.execute()` iterates waypoints and sends XY/Z/pump commands in sequence. When moving from one well to another, the waypoint sequence is: travel Z up → XY move → lower Z. But the executor doesn't wait for XY to actually arrive before commanding Z descent. This creates a race condition where the needle descends while still over the previous well or plate edge.

### Problem 2: `_sms_val` UnboundLocalError
The trajectory speed logger at line ~730 referenced `_sms_val`, but that variable is only defined in the `else` branch (legacy `set_velocity` path). When the `set_speed_mm_s` path is taken, `_sms_val` is undefined, causing:
```
Cannot access local variable '_sms_val' where it is not associated with a value
```

### Changes

**Safety check (v7.2.9):**
- Before any Z descent > 0.5mm (`wp.z < _prev_z - 0.5`), the executor calls `ctrl.wait_for_xy_arrival(wp.x, wp.y, tolerance_mm=0.1, timeout_s=10.0)`
- This blocks until the XY stage has confirmed arrival at the target well position
- Only triggers on significant Z descents (>0.5mm) to avoid unnecessary waits during normal printing
- Guarded with `hasattr(ctrl, 'wait_for_xy_arrival')` for backwards compatibility

**Speed logging fix:**
- Added `_speed_info = ""` before the branching logic
- `set_speed_mm_s` branch sets `_speed_info = f"{_max_spd * 1.5:.1f} mm/s"`
- `set_velocity` branch sets `_speed_info = f"SMS={_sms_val}"`
- Logger now uses `_speed_info` instead of directly referencing `_sms_val`

---

## Files Modified

| File | Lines Changed | Summary |
|------|--------------|---------|
| `SupportClasses/PrintPlanOfAction.py` | +1163 / -407 | Complete rewrite: 6 new dataclasses, 5 new step types, enriched PlanStep, comprehensive `_compute()` |
| `SupportClasses/HardwareConfig.py` | +5 / -62 | InkSwapStrategy moved out, re-exported for backwards compatibility |
| `SupportClasses/PrintTrajectoryPlanner.py` | +59 / -28 | New step type handlers, flattened service logic, simplified RETURN_HOME |
| `SupportClasses/StageController.py` | +38 / -0 | New `wait_for_xy_arrival()` method for needle safety |
| `SupportClasses/PrintManager.py` | +14 / -6 | XY arrival safety check before Z descent + `_sms_val` bug fix |
| `gui/pages/print_setup.py` | +456 / -74 | New Finalize tab, Plan of Action UI, context panel → display options only |
| `gui/pages/hardware_setup.py` | +3 / -80 | Ink Swap Strategy UI removed (moved to print_setup Finalize tab) |
| `gui/pages/print_well_setup.py` | +8 / -7 | Plan section removed from UI, `_generate_plan()` accepts `execution_config` |

**Total: ~+1746 / -664 lines across 8 source files**

---

## Architecture Decisions

### Why move InkSwapStrategy to PrintPlanOfAction?
The ink swap strategy is an execution behavior, not a hardware property. It controls what the *plan* does during ink changes. Keeping it with the plan generation code makes the data flow clearer: HardwareConfig describes what hardware exists, PrintExecutionConfig describes how to use it.

### Why a Finalize tab instead of keeping settings in the context panel?
The Plan of Action configuration has too many controls for a narrow side panel. A full-width tab gives room for the two-column layout (print parameters + plan of action) and makes the workflow explicit: configure objects → assign wells → finalize settings → generate.

### Why compound steps (INK_SWAP, FINAL_CLEANUP) with sub_steps?
These are logically single operations that expand to multiple hardware actions. Having them as single plan steps makes the plan overview cleaner while the `sub_steps` list tells the trajectory planner exactly what to execute.

### Why poll-based XY arrival confirmation?
The Prior ProScan XY stage doesn't provide a "move complete" callback. The only way to confirm arrival is polling the position. A 50ms poll interval balances responsiveness with serial port load. The 0.1mm tolerance accounts for stage positioning precision while preventing false positives during travel.

### Why 0.5mm Z descent threshold for the safety check?
Normal print movements involve very small Z changes (layer heights ~0.1-0.3mm). A 0.5mm descent indicates a significant Z movement — typically lowering into a well from travel height. This avoids unnecessary XY waits during fine Z movements within a well.

---

## Testing Checklist

### PrintPlanOfAction Backend
- [ ] `PrintExecutionConfig` serializes to/from dict correctly
- [ ] `InkGatherConfig.effective_volume()` applies extra % and clamps to max
- [ ] `ZTravelConfig.plunge_buffer_mm` converts µm → mm correctly
- [ ] Plan generation with `PrintExecutionConfig` produces correct step sequence
- [ ] Plan generation with legacy `PlanPreferences` still works (backwards compat)
- [ ] `InkSwapStrategy` importable from both `PrintPlanOfAction` and `HardwareConfig`

### New Step Types in Trajectory Planner
- [ ] GATHER_INK steps show correct volume with extra % applied
- [ ] MOVE_SAFE_Z raises Z only when below safe height
- [ ] TRAVEL_XY resolves well positions and generates fast moves
- [ ] INK_SWAP executes sub-steps in correct order
- [ ] FINAL_CLEANUP executes only enabled sub-steps
- [ ] Old step types (PRINT, WASTE, WASH, etc.) still execute correctly

### Finalize Tab UI
- [ ] Tab 4 "Finalize" appears in print setup
- [ ] Left column shows all print parameters (extrusion, calculated, layers, advanced)
- [ ] Right column shows Plan of Action sections (ink swap, gathering, Z travel, XY travel, cleanup)
- [ ] Ink swap checkboxes and volume spinboxes work
- [ ] Ink gathering extra % and max pickup spinboxes work
- [ ] Z travel checkboxes and plunge buffer spinbox work
- [ ] XY travel checkbox and speed spinbox work
- [ ] Final cleanup group box toggles enable/disable correctly
- [ ] "Validate & Generate Print" button reads config from UI and generates plan
- [ ] "Send to Monitor" button works after generation
- [ ] Scrolling works when content exceeds tab height

### Hardware Setup
- [ ] Ink Swap Strategy section no longer appears
- [ ] Hardware config still loads/saves with ink_swap_strategy field
- [ ] No import errors (InkSwapStrategy re-exported from HardwareConfig)

### Context Panel
- [ ] Context panel shows Display Options (well plate + object preview toggles)
- [ ] No print settings in context panel

### Needle Safety (CRITICAL)
- [ ] XY stage arrives at target well before Z descends — verify with simulator
- [ ] No unnecessary waits during normal Z movements < 0.5mm within a well
- [ ] Timeout (10s) logs warning but doesn't block indefinitely
- [ ] Safety check is a no-op when XY stage is not connected
- [ ] `_sms_val` logger no longer throws UnboundLocalError

### Integration
- [ ] Full workflow: Hardware Setup → Print Objects → Well Setup → Finalize → Generate → Monitor
- [ ] Old saved print files still load and generate correctly
- [ ] No circular imports between PrintPlanOfAction ↔ HardwareConfig
- [ ] App launches cleanly with no import errors

---

## Known Issues / Future Work

1. **Per-ink gather configs not yet in UI** — `PrintExecutionConfig.ink_gather_configs` supports per-ink overrides but the Finalize tab only has a single default extra % / max pickup. Per-ink controls need a dynamic list widget.
2. **Plan preview/visualization** — The Finalize tab has no plan step preview (the old `_plan_label` in Well Setup showed a text summary). A visual plan step timeline would be valuable.
3. **Z travel config not fully wired to TrajectoryPlanner** — `ZTravelConfig` fields like `fast_z_enter_well`, `wait_for_z_confirm`, and `plunge_buffer_um` are defined in `PlanStep` but the trajectory planner doesn't yet read all of them from individual step fields during execution. Currently only the global `wait_for_xy_arrival` check exists.
4. **XY travel speed per-step** — `TRAVEL_XY` steps carry `travel_speed_mm_s` but it's not yet differentiated per-step (all use the global config value).
5. **Print trajectory monitoring updates** — Monitor page needs updating to display new step types with proper colors/icons from `PLAN_STEP_COLORS` and `PLAN_STEP_ICONS`.
6. **CSV trajectory → print object workflow updates** — The import pipeline needs updating to generate `PrintExecutionConfig`-aware plans.
7. **HardwareConfig version field** still says `"7.2.4"` — should be bumped to `"7.2.9"`.
8. **Plunge buffer execution** — `ZTravelConfig.use_plunge_buffer` and `plunge_buffer_um` are configurable but the two-phase Z descent (fast to buffer, slow to print Z) is not yet implemented in TrajectoryPlanner's `_lower_to_print()`.
9. **Display Options not yet wired** — Context panel display checkboxes (show labels, trajectory paths, grid, axes) exist but are not yet connected to any rendering logic.
