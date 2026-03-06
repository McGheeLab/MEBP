# MEBP — Multi-Extrusion Bioprinting Platform

**Version 7.2.3 | March 2026**

A desktop application for controlling multi-material laboratory bioprinting equipment with precision trajectory planning, real-time monitoring, and persistent print file management.

---

## Features

**Hardware Control** — Orchestrates Prior ProScan II/III XY stages, Marlin-based Z-axis and pump controllers, and Hamilton syringe systems (25–1000 µL) through a unified interface. Xbox controller support for real-time jogging.

**Hardware-First Configuration** — Dedicated setup page for needles (16G–32G), syringes, inks, plate formats (6–384 well), pump loadouts, and rosette inserts. Configurations save/load as JSON and auto-restore on startup.

**µL-Native Pump Control** — All pump operations expressed in microliters throughout the entire application. Conversion to firmware units (mm) happens transparently via syringe barrel geometry.

**Parametric Print Objects** — Design dots, lines, circles, spirals, grids, and 3D objects (spheres, cubes, cylinders) with automatic trajectory generation accounting for needle diameter and ink properties.

**Auto-Layout System** — Arrange objects in ring, square grid, hex grid, line, or concentric ring patterns with well-boundary validation. Save reusable layout presets.

**Persistent Print Files** — Print designs stored as JSON with auto-save, full CRUD management, schema migration, and forward compatibility.

**Real-Time Print Monitor** — Job queue with start/pause/abort controls, full plate overview with per-well progress, current well trajectory visualization (XY/ZY/XZ projections), and syringe fill-level display.

**Print Recording** — Automatic timestamped recording of planned vs actual positions during prints. Replay visualization with tracking error statistics.

**Safety Systems** — Software endstops, flow-rate clamping, crash recovery with resume capability, and connection watchdog for serial port health monitoring.

---

## Quick Start

### Requirements

- Python 3.10+
- PySide6
- pyserial
- NumPy
- pygame (optional, for Xbox controller)

### Installation

```bash
pip install PySide6 pyserial numpy pygame
```

### Running

```bash
# GUI mode (default)
python main.py

# Simulation mode (no hardware required)
python main.py --simulate

# Specify serial ports
python main.py --xy-port COM3 --zp-port COM4
```

On first launch, the Hardware Setup page (Page 0) will open. Configure your needle, plate format, and at least one pump with a syringe before other pages become accessible.

---

## Hardware Requirements

| Component | Model | Connection | Role |
|-----------|-------|-----------|------|
| XY Stage | Prior ProScan II or III | RS-232 serial | Horizontal positioning |
| Z/Pump Board | Marlin-compatible 3D printer | RS-232 serial | Vertical needle + 3 syringe pumps |
| Syringes | Hamilton 1700 series | Mechanical (pump-driven) | 25–1000 µL dispensing |
| Needles | Blunt-tip luer-lock | Mechanical (needle holder) | 16G–32G |
| Well Plates | ANSI/SLAS standard | Mechanical (stage mount) | 6, 12, 24, 48, 96, 384 well |
| Gamepad | Xbox controller (optional) | USB | Real-time jog control |

### ZP Stage Axis Mapping

| Logical | Printer | Marlin | Purpose |
|---------|---------|--------|---------|
| Z | X | `G1 Xnn` | Vertical needle |
| P1 | Y | `G1 Ynn` | Syringe pump 1 |
| P2 | Z | `G1 Znn` | Syringe pump 2 |
| P3 | E | `G1 Enn` | Syringe pump 3 |

---

## Workflow

```
1. Hardware Setup (Page 0)
   Configure needle, plate, syringes, inks, pumps, rosettes
   → Save config → Pages 1–5 unlock

2. Dashboard (Page 1)
   Verify connections, check positions

3. Jog Control (Page 2)
   Manual positioning, zero references

4. Calibration (Page 3)
   Camera alignment, plate corners, Z-probe

5. Print Setup (Page 4)
   Tab 1: Review hardware summary (read-only)
   Tab 2: Design print objects, apply auto-layout, manage print files
   Tab 3: Assign prints to wells, set roles, calibrate plane
   → Click "Send to Monitor ▶"

6. Print Monitor (Page 5)
   Start/Pause/Abort, watch real-time progress, review recordings

7. Settings (Page 6)
   Safety limits, serial ports, simulation mode
```

---

## Project Structure

```
MEBP-Version-7.2.3/
├── main.py                          # Entry point
├── settings.json                    # Persistent app config
├── config/
│   ├── controllers/                 # XY stage protocol definitions
│   │   ├── proscan_ii.json
│   │   └── proscan_iii.json
│   ├── hardware/                    # Equipment catalogs
│   │   ├── needles.json             # 16G–32G specifications
│   │   ├── syringes.json            # Hamilton 1700 series
│   │   └── sample_setup.json        # Example hardware config
│   └── prints/                      # Persistent print file storage
│       ├── Dot_Array_4x4.json
│       └── Scaffold_v1.json
├── SupportClasses/                  # Backend (zero GUI dependencies)
│   ├── StageController.py           # Top-level hardware orchestrator
│   ├── PrintManager.py              # Print execution engine
│   ├── HardwareConfig.py            # Central µL↔mm configuration
│   ├── PrintFileManager.py          # Print file CRUD + auto-save
│   ├── auto_layout.py               # Layout pattern generators
│   ├── PhysicalModels.py            # Needle/Syringe/Ink/FluidColumn
│   ├── GeometryEngine.py            # Parametric print object generation
│   ├── TrajectoryPlanner.py         # Time-parameterized path planning
│   ├── MotionController.py          # PID + Kalman tracking controller
│   ├── FlowPhysics.py               # Pressure/flow/safety calculations
│   ├── WellPlate.py                 # ANSI/SLAS plate geometry
│   ├── PrintRecorder.py             # Print recording + replay
│   ├── SafetyLimits.py              # Software endstops + clamping
│   └── ... (13 more modules)
├── gui/                             # Frontend (PySide6)
│   ├── app.py                       # MainWindow + page routing
│   ├── styles.py                    # Catppuccin Mocha dark theme
│   └── pages/
│       ├── hardware_setup.py        # Page 0: equipment configuration
│       ├── dashboard.py             # Page 1: status overview
│       ├── jog_control.py           # Page 2: manual movement
│       ├── calibration.py           # Page 3: alignment + probing
│       ├── print_setup.py           # Page 4: design + assignment
│       ├── print_workspace.py       # Tab 1: hardware summary
│       ├── print_monitor.py         # Page 5: execution + monitoring
│       └── settings_page.py         # Page 6: system settings
└── tests/                           # 233 tests with physics simulators
    ├── test_session1_hw_setup.py
    ├── test_session2_workspace.py
    ├── test_session3_execution.py
    ├── test_session4_print_files.py
    └── test_session5_integration.py
```

---

## v7.2.3 Changes

### Session 1: Hardware Setup Fixes

- Reordered UI sections: Name → Needle → Plate → **Ink Library** (moved up) → **Pumps** (moved down) → **Rosettes** (new)
- Fixed `_apply_config_to_ui()` to restore in dependency order — ink library populated BEFORE pump combos resolve ink names
- Fixed `PumpChannelWidget.set_config()` to accept `ink_names` parameter
- Added `RosetteEditorDialog` with `create_standard()` integration

### Session 2: Read-Only Workspace + Enhanced Settings

- Complete `WorkspaceTab` rewrite — removed all editable hardware controls
- New `HardwareSummaryWidget` displaying read-only needle, plate, pumps, inks, rosettes
- `_hardware_config_to_workspace()` bridge: `HardwareConfig` → `WorkspaceConfig`
- "Edit Hardware Setup ▶" button for navigation to Page 0
- Per-pump retract/prime volumes in context panel print settings

### Session 3: Execution → Monitor

- Execution controls (Start/Pause/Abort) removed from Print Setup context panel
- "📤 Send to Monitor ▶" button emits `job_ready(PrintJob)` signal
- Print Monitor receives jobs via `receive_job()`, manages queue, owns execution UI
- `app.py` wires complete job pipeline and `PrintManager` callbacks

### Session 4: Print File Infrastructure + Auto-Layout

- `PrintFileManager`: CRUD for persistent print files in `config/prints/`
- Print file JSON schema v7.2.3 with metadata, objects, collections, layout presets
- Schema validation and forward migration (v7.1 → v7.2 → v7.2.3)
- `auto_layout.py`: five pattern generators (ring, grid, hex, line, concentric)
- Auto-save with 30-second timer and dirty-state tracking

### Session 5: Integration Testing

- 52 integration tests covering end-to-end workflows
- Hardware persistence verification across simulated restarts
- Page gating, edge cases (empty config, mid-session changes, deleted resources)
- Sample print files: `Dot_Array_4x4.json`, `Scaffold_v1.json`

---

## Print File Format

Print designs are stored as JSON in `config/prints/`:

```json
{
    "schema_version": "7.2.3",
    "metadata": {
        "name": "Scaffold_v1",
        "description": "Multi-layer hydrogel scaffold",
        "created": "2026-03-02T12:00:00+00:00",
        "modified": "2026-03-02T12:15:00+00:00"
    },
    "objects": {
        "Cylinder_1": {
            "object_type": "cylinder_solid",
            "params": {"radius": 2.0, "height": 0.6, "layer_height": 0.2},
            "position": [0.0, 0.0, 0.0],
            "ink_pump": "P1",
            "num_layers": 3
        }
    },
    "collections": { ... },
    "layout_presets": { ... }
}
```

---

## Auto-Layout Patterns

| Pattern | Parameters | Use Case |
|---------|-----------|----------|
| Ring | count, radius, start_angle | Circular arrangements around well center |
| Square Grid | rows, cols, spacing | Regular rectangular arrays |
| Hex Grid | rows, cols, spacing | Dense hexagonal packing |
| Line | count, start, end | Linear arrangements |
| Concentric Rings | rings, inner_r, outer_r, points/ring | Radial gradient patterns |

All layouts include well-boundary validation to ensure objects fit within the target plate format.

---

## Running Tests

```bash
# Full test suite (233 tests)
python -m pytest tests/ -v

# Individual session suites
python -m pytest tests/test_session1_hw_setup.py -v
python -m pytest tests/test_session4_print_files.py -v

# Quick smoke test
python -m pytest tests/test_session5_integration.py -v
```

Tests use physics-based simulators (`XYStageSimulator`, `ZPStageSimulator`) that model acceleration ramps, serial buffering, and smooth motion — no hardware required.

---

## Configuration Files

### Hardware Config (`config/hardware/sample_setup.json`)

Saved/loaded by Hardware Setup page. Contains needle, plate, pumps, inks, rosettes.

### Controller Protocols (`config/controllers/proscan_*.json`)

Define command syntax for different XY stage controllers. Auto-detection probes all available protocols on connection.

### Application Settings (`settings.json`)

Persists window state, last active print, safety limits, serial ports, and hardware config reference. Auto-saved on changes.

---

## Architecture

See `docs/ARCHITECTURE_V723.md` for the complete technical reference including module dependency graph, signal flow diagrams, threading model, and hardware interface details.

---

## License

Proprietary — McGhee Lab internal use.
