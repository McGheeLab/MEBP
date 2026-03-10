# MEBP v7.2.8 → v7.2.9 Update Plan

**Date:** 2026-03-10
**Branch:** Version-7.2.8
**Author:** Session work with Claude Code

---

## Overview

This session addressed multiple interconnected feature requests spanning the Print Objects designer, hardware configuration, ink management, and geometry engine. The changes consolidate the object type system, add filled/shell geometry support, overhaul the ink-pump architecture to support multi-ink per pump, and introduce configurable ink swap strategies.

---

## 1. Object Type List Consolidation

### Problem
The object type list in the Print Objects designer showed separate entries for shell and solid variants of each 3D type (e.g., "Sphere (shell)", "Sphere (solid)", "Cube (shell)", "Cube (solid)"). This cluttered the UI and confused the relationship between the filled checkbox and 3D type selection.

### Changes

**`gui/pages/print_objects.py`**

- **Replaced `_build_object_types()` dynamic builder** with a static `OBJECT_TYPES` dict containing consolidated entries:
  - 1D: Point
  - 2D: Line, Circle, Square, Triangle, Spiral, Ellipse
  - 3D: Sphere, Cube, Cylinder, Ellipsoid (no shell/solid split)
  - Import: CSV Import

- **Added `_TYPE_CATEGORY` dict** mapping each type to its category ("1D", "2D", "3D", "Import")

- **Added `_3D_TYPE_MAP` dict** that resolves consolidated GUI types to engine types:
  ```python
  _3D_TYPE_MAP = {
      ("sphere", True): "sphere_solid",    ("sphere", False): "sphere_shell",
      ("cube", True): "cube_solid",        ("cube", False): "cube_shell",
      ("cylinder", True): "cylinder_solid", ("cylinder", False): "cylinder_shell",
      ("ellipsoid", True): "ellipsoid_solid", ("ellipsoid", False): "ellipsoid_shell",
  }
  ```

- **Added `_CONSOLIDATED_3D_PARAMS`** — default parameter dicts for each consolidated 3D type (superset of shell+solid params)

- **Rewrote `_categorize_object_types()`** to group into 1D/2D/3D/Import sections (was: 2D/3D Solid/3D Shell/Import)

- **Updated `_get_type_params()`** to check consolidated 3D params first before falling back to GeometryEngine

- **Updated `_build_print_object()`** to resolve consolidated type → engine type via `_3D_TYPE_MAP.get((obj_type, filled), obj_type)`

- **Updated `_enter_edit_mode()`** to map old saved shell/solid entries back to consolidated GUI types with inferred filled state

### Backward Compatibility
- Old saved print files with `sphere_shell`/`sphere_solid` types still work — `_3D_TYPE_MAP` fallback returns the raw type unchanged when no match found
- Edit mode detects `_solid`/`_shell` suffixes and maps to the base type + sets filled checkbox

---

## 2. Filled / Shell Checkbox + Fill Pattern Selector

### Problem
2D shapes (circle, square, triangle, ellipse) had no filled option, and there was no unified UI control for the shell/solid distinction across all object types.

### Changes

**`gui/pages/print_objects.py`**

- **Added `_filled_check` QCheckBox** ("Filled / Solid") in the common fields area
- **Added `_fill_pattern_combo` QComboBox** (Meander / Spiral) that appears next to the checkbox when filled is checked
- **Added `_fill_pattern_label` QLabel** ("Pattern:") — hidden when filled is unchecked
- **Added `_on_filled_toggled()` method** — shows/hides fill pattern combo on checkbox toggle
- **Added `_sync_filled_checkbox()` method** — hides checkbox for point/line/spiral/csv_import; shows for fillable types
- **Updated `_on_type_list_changed()`** to call `_sync_filled_checkbox()` on type selection
- **Updated `_get_current_params()`** to include `filled` flag and `fill_pattern` in params dict
- **Updated `_set_params_from_dict()`** to restore filled checkbox + fill pattern state
- **Updated `_enter_edit_mode()`** to restore filled state from params or infer from old type
- **Updated `_build_print_object()`** to pass `fill_pattern` to `generate_object_trajectory()`
- **Moved `QCheckBox` import** from inline to top-level imports

**`SupportClasses/GeometryEngine.py`**

- **Circle filled support** (already existed): Uses `generate_circular_meander_fill()` when `filled=True`
- **Square filled support**: Fixed incorrect args — was passing `(ox-side/2, oy-side/2, ox+side/2, oy+side/2)` instead of `(ox, oy, side, side, spacing)`
- **Added `generate_triangular_meander_fill()`** function — meander fill constrained to equilateral triangle region with scan lines narrowing linearly from base to apex
- **Triangle filled support**: Added dispatch using `generate_triangular_meander_fill()` when `filled=True`
- **Ellipse filled support** (already existed): Uses `generate_elliptical_meander_fill()` when `filled=True`

---

## 3. Ink System Overhaul: Multi-Ink Per Pump

### Problem
Each pump could only be assigned one ink. For single-syringe workflows requiring multiple materials, the user needed to manually reconfigure between prints. The object designer showed pump IDs alongside ink names (e.g., "P1: Hydrogel A") instead of just the ink name.

### Changes

**`SupportClasses/HardwareConfig.py`**

- **`PumpChannelConfig.ink` → `PumpChannelConfig.inks: list[InkSpec]`**
  - Backward-compat `ink` property: getter returns `inks[0]` or None; setter replaces list
  - Added `ink_names` property: returns list of ink name strings
  - Added `can_handle_ink(ink_name)` method
  - Added `add_ink(ink)` / `remove_ink(ink_name)` methods

- **`HardwareConfig.pump_ink_map`** now returns `dict[str, list[str]]` (pump → list of ink names)
- **`HardwareConfig.ink_pump_map`** now returns `dict[str, list[str]]` (ink → list of pump IDs)
- **`HardwareConfig.get_pump_for_ink()`** returns first matching pump from the list
- **Added `set_pump_inks()`, `add_pump_ink()`, `remove_pump_ink()`** convenience methods
- **Removed ink uniqueness validation** — same ink can now be assigned to multiple pumps
- **Updated `PumpChannelConfig.to_dict()`** to serialize `"inks": [...]` list
- **Updated `PumpChannelConfig.from_dict()`** to load both new `"inks"` list and old single `"ink"` key

**`gui/pages/print_objects.py`**

- **`_refresh_ink_options_from_config()`** now populates from `_hw_config.ink_library` (all defined inks) instead of filtering by pump assignment
- **Ink combo shows just ink names** (was: "P1: Hydrogel A")
- **Initial placeholder** changed from `"P1 (default)"` to `"(no inks defined)"`
- **`_resolve_ink_to_pump()`** updated to use `pcfg.can_handle_ink()` instead of `pcfg.ink.name ==`

**`gui/pages/hardware_setup.py`**

- **PumpChannelWidget ink selector**: Replaced single `QComboBox` with checkable `QListWidget` for multi-ink selection
  - `set_ink_names()` populates checklist (no exclusion logic)
  - `get_config()` returns `PumpChannelConfig` with all checked inks
  - `set_config()` restores check states from config's `inks` list
  - `get_selected_ink_names()` returns list of checked names
  - `get_selected_ink_name()` backward compat — returns first checked or None
- **Removed ink exclusion logic** — `_refresh_pump_ink_exclusions()` simplified to just refresh lists
- **Updated `_update_pump_ink_summary()`** — shows multi-ink format: `P1→[Hydrogel A, MSC Cells]`
- **Updated `_rebuild_config()`** — resolves multiple inks from library
- **Cleaned up `_apply_config_to_ui()`** — removed 3 duplicate v7.2.5 verification blocks that referenced old `ink_combo`; replaced with single multi-ink verification

---

## 4. Ink Swap Strategy

### Problem
When a single pump handles multiple inks, the system needs a configurable cleaning/loading sequence between ink changes. There was no data model or UI for this.

### Changes

**`SupportClasses/HardwareConfig.py`**

- **Added `InkSwapStrategy` dataclass** with:
  - 6 boolean step toggles: `waste`, `wash_pre`, `buffer`, `wash_post`, `ink_load`, `wash_final`
  - 4 volume settings (µL): `waste_volume_uL`, `wash_volume_uL`, `buffer_volume_uL`, `ink_load_volume_uL`
  - `get_enabled_steps()` → ordered list of active step names
  - `to_dict()` / `from_dict()` serialization

- **Added `ink_swap_strategy` field** to `HardwareConfig` (default: all steps enabled)
- **Updated `HardwareConfig.to_dict()`** to serialize swap strategy
- **Updated `HardwareConfig.from_dict()`** to deserialize swap strategy (with fallback defaults)

**`gui/pages/hardware_setup.py`**

- **Added "Ink Swap Strategy" section** (QGroupBox) between Pump Channels and Needle Configuration:
  - 6 checkboxes for each swap step
  - 4 volume spinboxes (µL) for waste, wash, buffer, ink load amounts
- **Updated `_rebuild_config()`** to capture swap strategy from checkboxes + spinboxes
- **Updated `_apply_config_to_ui()`** to restore swap strategy state
- **Added `InkSwapStrategy` to top-level imports**

---

## 5. Serialization Version Bump

- `HardwareConfig.to_dict()` version field should be updated from `"7.2.4"` to `"7.2.8"` (NOTE: not yet done — recommend for v7.2.9 cleanup)

---

## Files Modified

| File | Changes |
|------|---------|
| `gui/pages/print_objects.py` | Object type consolidation, filled checkbox + fill pattern, ink combo overhaul, multi-ink resolution |
| `gui/pages/hardware_setup.py` | Multi-ink checklist, ink swap strategy section, removed exclusion logic, cleaned up duplicate verification |
| `SupportClasses/HardwareConfig.py` | `InkSwapStrategy` dataclass, multi-ink `PumpChannelConfig`, updated properties/validation/serialization |
| `SupportClasses/GeometryEngine.py` | Fixed square fill args, added `generate_triangular_meander_fill()`, triangle filled support |

---

## Testing Checklist

### Object Type Consolidation
- [ ] Object type list shows 1D/2D/3D/Import categories (no shell/solid split)
- [ ] Selecting Sphere shows radius, layer_height, num_points params
- [ ] Filled checkbox auto-unchecked for 3D types (shell default)
- [ ] Checking filled generates solid trajectory (verify with preview)
- [ ] Old saved prints with `sphere_solid`/`sphere_shell` types load correctly in edit mode

### Filled Checkbox + Fill Pattern
- [ ] Checkbox hidden for Point, Line, Spiral, CSV Import
- [ ] Checkbox visible for Circle, Square, Triangle, Ellipse, all 3D types
- [ ] "Pattern:" combo appears when filled is checked
- [ ] Meander and Spiral fill patterns generate different trajectories
- [ ] Filled state persists in saved entries and restores on edit

### Multi-Ink Per Pump
- [ ] Hardware Setup: pump ink selector shows checklist (not single combo)
- [ ] Can check multiple inks for one pump
- [ ] Same ink can be checked on multiple pumps (no exclusion)
- [ ] Pump-ink summary shows multi-ink format
- [ ] Config saves/loads correctly with multiple inks
- [ ] Old configs with single `"ink"` field load correctly

### Ink Swap Strategy
- [ ] Ink Swap Strategy section appears in Hardware Setup
- [ ] All 6 checkboxes toggle independently
- [ ] Volume spinboxes accept values and persist
- [ ] Strategy saves/loads with hardware config

### Designer Ink Combo
- [ ] Shows just ink names (no pump ID prefix)
- [ ] Lists all inks from library (not filtered by pump)
- [ ] Color swatch updates when ink selected
- [ ] Ink resolution to pump works for multi-ink assignments

---

## Known Issues / Future Work

1. **Path planning integration**: `InkSwapStrategy` is defined but not yet integrated into `PrintPlanOfAction` or `PrintTrajectoryPlanner` — the actual ink swap sequence generation is future work
2. **HardwareConfig version field** still says `"7.2.4"` — should be bumped
3. **Hardware setup status** still may not update reactively (reported earlier, not fully resolved)
4. **Xbox controller disconnect detection** — not addressed in this session
5. **Print monitor deque import** — fix identified but not confirmed applied
6. **Print records not saving** — not addressed in this session
