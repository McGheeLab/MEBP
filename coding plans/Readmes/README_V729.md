# MEBP v7.2.9 — Release Notes

## Overview

Version 7.2.9 introduces the **PrintExecutionConfig** system, replacing the fragmented `PlanPreferences` with a comprehensive, composable execution configuration. The Plan of Action pipeline now supports 5 new granular step types, and the Print Setup page gains a dedicated **Finalize tab** (Tab 4) for configuring all execution parameters in one place.

---

## Key Changes

### 1. PrintExecutionConfig (replaces PlanPreferences)

A new unified configuration dataclass that composes all execution behavior:

| Sub-Config | Purpose |
|-----------|---------|
| `InkSwapStrategy` | 6-step cleaning sequence (waste, wash_pre, buffer, wash_post, ink_load, wash_final) with volume settings |
| `InkGatherConfig` | Per-ink pickup volume control with extra % override |
| `ZTravelConfig` | Safe Z height, plunge buffer, fast entry/exit, Z confirmation |
| `XYTravelConfig` | Fast XY travel toggle and speed setting |
| `FinalCleanupConfig` | End-of-print cleanup (waste eject, wash cycles, dry run) |

- Per-ink overrides via `ink_gather_configs` dict with `default_gather_config` fallback
- Backward-compatible migration via `PrintExecutionConfig.from_preferences()`
- Full `to_dict()` / `from_dict()` serialization

### 2. New Plan Step Types

Five new `PlanStepType` enum values for fine-grained execution control:

| Step Type | Purpose |
|-----------|---------|
| `GATHER_INK` | Ink pickup with per-ink specs and extra % |
| `FINAL_CLEANUP` | End-of-print cleanup sequence (compound step) |
| `TRAVEL_XY` | Explicit fast XY move between wells |
| `MOVE_SAFE_Z` | Raise to safe Z height for travel |
| `INK_SWAP` | Full ink swap sequence (compound: waste→wash→buffer→ink_load) |

### 3. Enriched PlanStep

Each `PlanStep` now carries execution metadata:

- `z_behavior` — Z mode: "fast", "slow", "plunge", "default"
- `travel_mode` — "fast" or "slow"
- `wait_for_z_confirm` — Wait for position confirmation
- `extra_percent` — Extra ink % for gather steps
- `max_pickup_uL` — Max pickup volume for gather steps
- `sub_steps` — Ordered list for compound operations (INK_SWAP, FINAL_CLEANUP)
- `icon` / `color` properties — Catppuccin Mocha visualization via `PLAN_STEP_COLORS` / `PLAN_STEP_ICONS`

### 4. InkSwapStrategy Relocation

- **Canonical location**: `PrintPlanOfAction.py` (execution concern, not hardware concern)
- **Backward compat**: Re-exported from `HardwareConfig.py` via import
- **Rationale**: Ink swap is a per-print execution choice, not a hardware-level setting

### 5. Finalize Tab (Tab 4)

New tab in Print Setup (Page 4) with two-column layout:

- **Left: Print Parameters** — All `PrintExecutionConfig` sub-config widgets (ink swap toggles/volumes, Z travel, XY travel, ink gather, final cleanup)
- **Right: Plan of Action** — Generated plan preview with color-coded step types, generate and send-to-monitor buttons

### 6. UI Reorganization

- Ink Swap Strategy UI removed from Hardware Setup (Page 0) → moved to Finalize tab
- Plan of Action section removed from Well Setup tab → moved to Finalize tab
- Context panel simplified to display options only

### 7. PrintTrajectoryPlanner Updates

New step type handlers in `generate()`:

- `GATHER_INK` → delegates to `_do_load_ink()` with pre-computed volume
- `MOVE_SAFE_Z` → fast Z raise to safe height
- `TRAVEL_XY` → fast XY move with speed override from step metadata
- `FINAL_CLEANUP` → executes sub-step sequence (waste/wash)
- `INK_SWAP` → compound sub-step sequence (waste→wash→buffer→ink_load)

### 8. ImagePathPlanner (New Module)

Image-stack-to-toolpath raster generator:

- Three loading modes: TIFF stack, numbered image sequence, single image × N layers
- Per-pump greyscale intensity → flow rate mapping
- Raster toolpath with alternating layer direction and closest-point reordering
- Output: Nx7 array `[x, y, z, p1, p2, p3, t]` in mm/s
- CSV export via `save_csv()`

### 9. Object Type Consolidation (Prior v7.2.9 Work)

- Unified GUI type → engine type resolution via `_3D_TYPE_MAP`
- Filled/Solid checkbox for shell vs. solid variants
- Fill pattern selector (Meander / Spiral) for filled shapes
- `generate_triangular_meander_fill()` for triangle fill patterns

---

## Files Modified

| File | Changes |
|------|---------|
| `SupportClasses/PrintPlanOfAction.py` | PrintExecutionConfig, sub-configs, 5 new step types, PLAN_STEP_COLORS/ICONS, enriched PlanStep |
| `SupportClasses/PrintTrajectoryPlanner.py` | Handlers for all new step types |
| `SupportClasses/HardwareConfig.py` | InkSwapStrategy re-export, multi-ink per pump |
| `SupportClasses/ImagePathPlanner.py` | New module: image-to-toolpath raster generator |
| `SupportClasses/GeometryEngine.py` | Triangle meander fill, object type consolidation |
| `gui/pages/print_setup.py` | Tab 4 Finalize, PrintExecutionConfig UI widgets |
| `gui/pages/print_well_setup.py` | Plan section removed (moved to Finalize) |
| `gui/pages/hardware_setup.py` | Ink swap UI removed (moved to Finalize) |
| `gui/pages/print_objects.py` | Filled/solid checkbox, fill pattern selector, type consolidation |
| `gui/pages/print_monitor.py` | New step type color/icon support |

---

## Architecture Reference

See `coding plans/Architectures/ARCHITECTURE_V729.md` for the full system architecture.
