# MEBP v7.2.3 — GUI Modernization Upgrade

Complete GUI restructuring for the Multi-Extrusion Bioprinting Platform.
This upgrade reorganizes the Print Setup workflow into a Hardware Setup → Workspace → Objects → Wells → Monitor pipeline with proper signal propagation and execution control relocation.

## What Changed

**Page 0 — Hardware Setup** (S1): Reordered sections (Ink Library now ABOVE Pump Channels), added Rosette Library UI, fixed `_apply_config_to_ui()` to restore in dependency order so ink assignments survive auto-load and file-load.

**Page 4 — Print Setup** (S2): Workspace tab converted to read-only `HardwareSummaryWidget` displaying the active hardware configuration. Per-pump retract/prime settings added to context panel. `_hardware_config_to_workspace()` bridge method builds `WorkspaceConfig` from `HardwareConfig`.

**Page 5 — Print Monitor** (S3): Now receives jobs from Print Setup via `receive_job()`. Job queue with Start button, progress tracking, and auto-advance on completion. Recording browser preserved. PrintManager ownership moved to `app.py`.

**Print Files** (S4A): New persistent storage system (`config/prints/`) with JSON schema v7.2.3. Full CRUD via `PrintFileManager`, auto-save with dirty tracking, schema validation and migration from v7.1/v7.2.

**Auto-Layout** (S4B): Five pattern generators (Ring, Square Grid, Hex Grid, Line, Concentric Rings) with well boundary validation. Layout presets storable in print files.

## File Inventory

```
MEBP-v7.2.3/
├── SupportClasses/
│   ├── PrintFileManager.py         (622 lines)  S4: Print file CRUD + validation
│   └── auto_layout.py              (384 lines)  S4: Layout generators
├── config/prints/
│   ├── Scaffold_v1.json                         Sample: scaffold + cell ring
│   └── Dot_Array_4x4.json                      Sample: grid of dots
├── tests/
│   ├── test_session4_print_files.py (62 tests)  PrintFileManager + auto_layout
│   └── test_session5_integration.py (39 tests)  Cross-session + edge cases
├── patches/v723/
│   └── V723_UPGRADE_PLAN.md                     Master plan
└── README_V723.md                               This file
```

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

1. **Session 1** — Replace `gui/pages/hardware_setup.py` (from project knowledge)
2. **Session 2** — Replace `gui/pages/print_workspace.py` and `gui/pages/print_setup.py`
3. **Session 3** — Update `gui/app.py` with re-entrancy guard + job routing. Apply `print_monitor_additions.py` to existing `gui/pages/print_monitor.py`
4. **Session 4** — Add `SupportClasses/PrintFileManager.py` and `SupportClasses/auto_layout.py`. Apply `print_objects_additions.py` to existing `gui/pages/print_objects.py`
5. **Session 5** — Run full test suite: `python -m pytest tests/ -v`

## Running Tests

```bash
# All tests
python -m pytest tests/ -v

# Individual suites
python -m pytest tests/test_session4_print_files.py -v    # 62 tests
python -m pytest tests/test_session5_integration.py -v    # 39 tests
```

Tests require only Python 3.10+ standard library (no PySide6 needed).

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
    "objects": {
        "<name>": {
            "object_type": "dot|line|circle|spiral|grid|cylinder_solid",
            "params": {...},
            "position": [x, y, z],
            "color": "#hex",
            "ink_pump": "P1|P2|P3",
            "num_layers": 1,
            "layer_height": 0.2
        }
    },
    "collections": {
        "<name>": [{"object_name": "...", "position": [x,y,z], "copies": N}]
    },
    "layout_presets": {
        "<name>": {"pattern": "ring|square_grid|hex_grid|line|concentric_rings", "params": {...}}
    }
}
```

Auto-migrates from v7.1 (adds `layout_presets`) and v7.2 (updates schema version).

## Auto-Layout Patterns

| Pattern | Parameters | Example |
|---------|-----------|---------|
| Ring | count, radius, start_angle | 6 dots around a 2mm circle |
| Square Grid | rows, cols, spacing | 4×4 grid at 0.8mm spacing |
| Hex Grid | rows, cols, spacing | Honeycomb packing |
| Line | count, start(x,y), end(x,y) | 5 dots along a 4mm line |
| Concentric Rings | ring_count, objects_per_ring, inner/outer_radius | 3 rings of 6 dots |

All positions validated against well boundaries before application.

## Dependencies

No new external dependencies. Uses only:
- Python 3.10+ standard library
- PySide6 (existing project dependency)
- Catppuccin color scheme (existing `gui/styles.py`)
