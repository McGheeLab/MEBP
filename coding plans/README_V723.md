# MEBP v7.2.3 — GUI Modernization Upgrade

Complete GUI restructuring for the Multi-Extrusion Bioprinting Platform.
This upgrade reorganizes the Print Setup workflow into a Hardware Setup → Workspace → Objects → Wells → Monitor pipeline with proper signal propagation and execution control relocation.

## What Changed

**Page 0 — Hardware Setup** (NEW): Dedicated configuration page for needles, syringes, inks, plate format, pump loadout, and rosette library. Replaces the old Workspace tab's editable hardware controls. Features dependency-ordered config restoration and page gating.

**Page 4 — Print Setup**: Workspace tab converted to read-only summary displaying the active hardware configuration. Per-pump settings (feedrate, retract, prime) added. Execution controls (Start, Pause, Abort) removed — moved to Monitor.

**Page 5 — Print Monitor**: Now receives jobs from Print Setup via `receive_job()`. Job queue with Start button, progress tracking, and auto-advance on completion. Recording browser preserved.

**Print Files**: New persistent storage system (`config/prints/`) with JSON schema v7.2.3. Full CRUD via `PrintFileManager`, auto-save every 30s, schema migration from v7.1/v7.2.

**Auto-Layout**: Five pattern generators (Ring, Square Grid, Hex Grid, Line, Concentric Rings) with well boundary validation. Integrated into Print Objects tab with dynamic parameter panels and reusable layout presets.

## File Inventory

```
MEBP-v7.2.3/
├── gui/
│   ├── app.py                       ( 939 lines)  S3: Full rewrite with v7.2.3 wiring
│   └── pages/
│       ├── hardware_setup.py        (1043 lines)  S1: Full rewrite
│       ├── print_workspace.py       ( 580 lines)  S2: Read-only conversion
│       ├── print_setup.py           ( 626 lines)  S2: Context cleanup
│       ├── print_monitor_additions.py ( 334 lines) S3: Job queue + exec controls
│       └── print_objects_additions.py ( 951 lines) S4: File bar + auto-layout
├── SupportClasses/
│   ├── PrintFileManager.py          ( 622 lines)  S4: Print file CRUD
│   └── auto_layout.py               ( 384 lines)  S4: Layout generators
├── patches/v723/
│   ├── V723_UPGRADE_PLAN.md         ( 342 lines)  Master plan
│   ├── apply_v723_session3_patch.py ( 633 lines)  S3: Legacy patch script
│   ├── SESSION1_CHANGELOG.md                       Per-session details
│   ├── SESSION2_CHANGELOG.md
│   ├── SESSION3_CHANGELOG.md
│   ├── SESSION4_CHANGELOG.md
│   └── SESSION5_CHANGELOG.md
├── test_session1_hw_setup.py        ( 201 lines)  30 tests
├── test_session2_workspace.py       ( 293 lines)  40 tests
├── test_session3_execution.py       ( 252 lines)  42 tests
├── test_session4_print_files.py     ( 621 lines)  69 tests
├── test_session5_integration.py     ( 725 lines)  52 tests
└── README_V723.md                                  This file
```

**Totals**: ~8,200 Python lines | 233 tests | 5 sessions

## Signal Flow

```
Page 0 (Hardware Setup) → HardwareConfig
  ↓ app._on_hardware_config_changed()
Page 4 (Print Setup) → set_hardware_config()
  ↓ tab_workspace: _hardware_config_to_workspace() bridge
  ↓ workspace_changed(WorkspaceConfig)
  ↓ tab_objects / tab_wells: set_workspace()
  ↓ User clicks "Send to Monitor" → job_ready(PrintJob)
  ↓ app._send_job_to_monitor(job)
Page 5 (Print Monitor) → receive_job(job)
  ↓ User clicks "Start" → start_requested(PrintJob)
  ↓ app._on_monitor_start(job) → PrintManager.start(job)
  ↓ PrintManager callbacks → monitor.on_print_progress()
  ↓ Queue auto-advance on completion
```

## Application Order

Apply these changes to an existing MEBP v7.2 codebase:

1. **Session 1** — Replace `gui/pages/hardware_setup.py` entirely
2. **Session 2** — Replace `gui/pages/print_workspace.py` and `gui/pages/print_setup.py`
3. **Session 3** — Replace `gui/app.py` entirely. Apply additions from `print_monitor_additions.py` to existing `print_monitor.py`
4. **Session 4** — Add `SupportClasses/PrintFileManager.py` and `SupportClasses/auto_layout.py`. Apply additions from `print_objects_additions.py` to existing `print_objects.py`
5. **Session 5** — Run full test suite to verify: `python -m pytest test_session*.py -v`

Each session's changelog in `patches/v723/` has detailed step-by-step application instructions.

## Running Tests

```bash
# All 231 tests
python -m pytest test_session*.py -v

# Individual sessions
python -m pytest test_session1_hw_setup.py -v       # 30 tests
python -m pytest test_session2_workspace.py -v       # 40 tests
python -m pytest test_session3_execution.py -v       # 40 tests
python -m pytest test_session4_print_files.py -v     # 69 tests
python -m pytest test_session5_integration.py -v     # 52 tests
```

Tests require only Python 3.10+ standard library (no PySide6 needed — uses mock-based structural verification).

## Print File Schema (v7.2.3)

```json
{
    "schema_version": "7.2.3",
    "metadata": {
        "name": "...",
        "description": "...",
        "created": "ISO-8601",
        "modified": "ISO-8601",
        "author": ""
    },
    "objects": { "<name>": { "object_type": "...", "params": {...}, ... } },
    "collections": { "<name>": [ { "object_name": "...", "position": [x,y,z], "copies": N } ] },
    "layout_presets": { "<name>": { "pattern": "...", "params": {...} } }
}
```

Auto-migrates from v7.1 (adds `layout_presets`) and v7.2 (updates schema version).

## Dependencies

No new external dependencies. Uses only:
- Python 3.10+ standard library
- PySide6 (existing project dependency)
- Catppuccin color scheme (existing `gui/styles.py`)
