# MEBP v7.2.3 — Architecture Reference

**Multi-Extrusion Bioprinting Platform**
**Version 7.2.3 | March 2026**

---

## 1. System Overview

MEBP is a desktop application for controlling laboratory bioprinting equipment. It orchestrates Prior ProScan XY stages, Marlin-based Z-axis and pump controllers, and Hamilton syringe systems to precisely deposit biological materials into well plates. The platform supports multi-material printing with various ink types (hydrogels, cell suspensions, granular materials) using multi-channel needles and sophisticated trajectory planning.

The application is written in Python with PySide6 for the GUI, using RS-232 serial communication for hardware control and JSON for all configuration and data persistence. An Xbox controller provides real-time jogging capability. The interface follows a PyDracula-inspired dark theme (Catppuccin Mocha palette).

### Core Design Principles

1. **Hardware abstraction** — GUI pages never touch serial ports. All hardware access goes through `StageController` or `PrintManager`.
2. **JSON protocol abstraction** — XY stage commands are defined in JSON protocol files (`config/controllers/`), enabling support for different controller models (ProScan II vs III) without code changes.
3. **Hardware-first configuration** — Page 0 (Hardware Setup) must be completed before any other page is accessible. `HardwareConfig` is the single source of truth for all µL ↔ mm conversions.
4. **µL-native pump control** — All user-facing pump values are in microliters. Conversion to mm (for Marlin firmware) happens exclusively inside `StageController.move_pump_uL()` via `SyringeSpec`.
5. **Safety-layered architecture** — Software endstops, flow-rate clamping, crash recovery, and thread-safe patterns throughout.
6. **Persistent print files** — Print designs stored as JSON in `config/prints/` with auto-save, schema migration, and full CRUD management.

---

## 2. Technology Stack

| Layer | Technology |
|-------|-----------|
| Language | Python 3.10+ |
| GUI Framework | PySide6 (Qt 6) |
| Theme | Catppuccin Mocha dark theme (QSS) |
| Serial Communication | pyserial (RS-232) |
| Numerical Computation | NumPy |
| Input Devices | pygame (Xbox controller via multiprocessing) |
| Configuration | JSON (hardware, controllers, prints, settings) |
| Testing | unittest + physics-based simulators |

---

## 3. Folder Structure

```
MEBP-Version-7.2.3/
├── main.py                              # Entry point — CLI arg parsing, mode selection
├── settings.json                        # Persistent app config (auto-saved)
├── current_button_mapping.json          # Xbox controller mapping (hot-reloadable)
│
├── config/
│   ├── controllers/
│   │   ├── proscan_ii.json              # ProScan II command protocol map
│   │   └── proscan_iii.json             # ProScan III command protocol map
│   ├── hardware/
│   │   ├── needles.json                 # Needle gauge catalog (16G–32G)
│   │   ├── syringes.json                # Hamilton syringe catalog (25–1000 µL)
│   │   └── sample_setup.json            # Example HardwareConfig file
│   └── prints/                          # ★ v7.2.3 — Persistent print file storage
│       ├── Dot_Array_4x4.json           # Sample: 4×4 grid of dot prints
│       └── Scaffold_v1.json             # Sample: multi-layer scaffold
│
├── SupportClasses/                      # Backend — zero GUI dependencies
│   ├── __init__.py                      # Package exports + module listing
│   ├── Processor.py                     # Thread-safe command bus (pub/sub dispatch)
│   ├── SerialUtils.py                   # Retry decorator, safe I/O, port discovery, watchdog
│   ├── XYStageSimulator.py              # Physics-based Prior ProScan simulator
│   ├── ZPStageSimulator.py              # Physics-based Marlin board simulator
│   ├── XYStage.py                       # Prior ProScan manager + ControllerProtocol
│   ├── ZPStage.py                       # Marlin board manager (G-code, axis mapping)
│   ├── XboxController.py                # Polling worker (multiprocessing.Process)
│   ├── StageController.py               # Top-level orchestrator (jog, polling, connect)
│   ├── ControllerProtocol.py            # JSON-based command protocol loader
│   ├── PrintManager.py                  # Execution engine + trajectory + service sequences
│   ├── PrintRecorder.py                 # Auto-record prints, replay data
│   ├── WellPlate.py                     # ANSI/SLAS plate geometry (6–384 well)
│   ├── WellSetup.py                     # Well assignments, rosettes, plane fitting
│   ├── SafetyLimits.py                  # Software endstops + flow-rate clamping
│   ├── Settings.py                      # JSON-backed persistent app config
│   ├── PositionLogger.py                # Timestamped position recording + CSV/JSON export
│   ├── PrintHistory.py                  # Persistent print history log
│   ├── PhysicalModels.py                # Needle, Syringe, Ink, FluidColumn, WorkspaceConfig
│   ├── HardwareConfig.py                # ★ v7.2 — Central µL↔mm config + save/load
│   ├── PrintFileManager.py              # ★ v7.2.3 — Print file CRUD + auto-save + migration
│   ├── auto_layout.py                   # ★ v7.2.3 — Ring/grid/hex/line/concentric generators
│   ├── GeometryEngine.py                # Parametric print objects + needle path generation
│   ├── TrajectoryPlanner.py             # Time-parameterized paths, CSV import
│   ├── MotionController.py              # Kalman filter, PID, feedforward tracking
│   └── FlowPhysics.py                   # Pressure/flow calculations, safety limits
│
├── gui/                                 # Frontend — PySide6, PyDracula-style layout
│   ├── __init__.py
│   ├── app.py                           # ✏ v7.2.3 — MainWindow + page gating + job pipeline
│   ├── styles.py                        # QSS dark theme (Catppuccin Mocha) + COLORS dict
│   ├── ui_functions.py                  # Animation helpers: toggleMenu, toggleLeftBox
│   ├── pages/
│   │   ├── __init__.py
│   │   ├── hardware_setup.py            # ✏ v7.2.3 — Page 0: needle/syringe/ink/plate/rosette
│   │   ├── dashboard.py                 # Page 1: device status, positions, history
│   │   ├── jog_control.py               # Page 2: manual movement, dpad, speed control
│   │   ├── calibration.py               # Page 3: camera, plate alignment, Z probing
│   │   ├── print_setup.py               # ✏ v7.2.3 — Page 4: workspace/objects/wells tabs
│   │   ├── print_workspace.py           # ★ v7.2.3 — Read-only hardware summary + bridge
│   │   ├── print_monitor.py             # ✏ v7.2.3 — Page 5: execution + queue + recording
│   │   └── settings_page.py             # Page 6: safety, serial ports, simulation
│   └── widgets/
│       ├── console_log.py               # Collapsible log viewer
│       ├── camera_widget.py             # OpenCV camera feed with crosshair overlay
│       └── xbox_mapping_editor.py       # Xbox button remap dialog
│
├── tests/
│   ├── test_session1_hw_setup.py        # 30 tests: hardware config restore
│   ├── test_session2_workspace.py       # 40 tests: workspace bridge + settings
│   ├── test_session3_execution.py       # 42 tests: job pipeline + monitor
│   ├── test_session4_print_files.py     # 69 tests: PrintFileManager + auto-layout
│   ├── test_session5_integration.py     # 52 tests: end-to-end workflows
│   ├── test_hardware_config.py          # 15 tests: HardwareConfig µL↔mm
│   ├── test_printmanager_v72.py         # 15 tests: µL print commands
│   ├── test_v72_integration.py          # 25 tests: full v7.2 integration
│   └── test_session_a.py               # PhysicalModels + FlowPhysics + ControllerProtocol
│
└── docs/
    ├── ARCHITECTURE_V723.md             # This file
    └── README_V723.md                   # User-facing upgrade guide
```

---

## 4. Module Dependency Graph

```
main.py
  ├── SupportClasses.StageController     ← the single backend entry point
  │     ├── Processor                    (thread-safe command bus)
  │     ├── XYStage
  │     │     ├── ControllerProtocol     (JSON command maps)
  │     │     ├── XYStageSimulator       (sim mode)
  │     │     └── SerialUtils            (real mode)
  │     ├── ZPStage
  │     │     ├── ZPStageSimulator       (sim mode)
  │     │     └── SerialUtils            (real mode)
  │     ├── XboxController               (multiprocessing.Process)
  │     ├── SafetyLimits
  │     ├── PositionLogger
  │     └── SerialUtils.ConnectionWatchdog
  │
  ├── SupportClasses.PrintManager        ← execution engine
  │     ├── TrajectoryExecutor           (continuous path tracking)
  │     │     └── PrintRecorder          (auto-record actual vs planned)
  │     ├── ServiceSequenceExecutor      (waste→wash→buffer→ink cycles)
  │     └── FluidColumnTracker           (syringe fill state)
  │
  ├── SupportClasses.HardwareConfig      ← µL↔mm conversion authority
  │     ├── PumpChannelConfig × 3
  │     └── PhysicalModels (NeedleSpec, SyringeSpec, InkSpec, FluidColumn)
  │
  ├── SupportClasses.PrintFileManager    ← v7.2.3 print file persistence
  │     └── auto_layout                  (ring, grid, hex, line, concentric)
  │
  ├── SupportClasses.Settings            ← persistent JSON config
  │
  └── gui.app.MainWindow                 (GUI mode only)
        ├── gui.styles                   → QSS theme + COLORS dict
        ├── gui.ui_functions             → animation helpers
        ├── gui.pages.hardware_setup     → Page 0: HardwareConfig editor
        ├── gui.pages.dashboard          → Page 1: status + positions
        ├── gui.pages.jog_control        → Page 2: manual movement
        ├── gui.pages.calibration        → Page 3: camera + plate alignment
        ├── gui.pages.print_setup        → Page 4: workspace/objects/wells
        │     ├── print_workspace        → Tab 1: read-only summary + bridge
        │     ├── print_objects          → Tab 2: designer + auto-layout
        │     └── well_setup             → Tab 3: well assignments
        ├── gui.pages.print_monitor      → Page 5: execution + queue
        ├── gui.pages.settings_page      → Page 6: safety + serial config
        └── gui.widgets.*                → console, camera, xbox editor
```

---

## 5. GUI Layout

```
┌────────────────────────────────────────────────────────────────────┐
│  MainWindow                                                        │
│ ┌────┬──────────┬────────────────────────────────────────────────┐ │
│ │ L  │ Context  │  Top Bar (page title + connection dots + ☰)    │ │
│ │ E  │ Panel    ├────────────────────────────────────────────────┤ │
│ │ F  │ (per-    │                                                │ │
│ │ T  │  page    │   Content Pages (QStackedWidget)               │ │
│ │    │  scroll) │     🔧 Hardware Setup  (always enabled)        │ │
│ │ M  │          │     📊 Dashboard / 🕹️ Jog / 📐 Calibration     │ │
│ │ E  │          │     🖨️ Print Setup / 📈 Print Monitor          │ │
│ │ N  │          │     ⚙️ Settings  (always enabled)              │ │
│ │ U  │          │                                                │ │
│ │    │          │ ┌────────────────────────────────────────────┐ │ │
│ │ ⚙  │          │ │ Console Log (collapsible via QSplitter)    │ │ │
│ └────┴──────────┘  Bottom Bar (XY pos, ZP pos, speed, safety)    │ │
└────────────────────────────────────────────────────────────────────┘
```

### Page Index Map (v7.2.3)

| Index | Page | Icon | Gated | Context Panel |
|-------|------|------|-------|---------------|
| 0 | Hardware Setup | 🔧 | No | Save/Load config, validation status |
| 1 | Dashboard | 📊 | Yes | Connection cards, position log, history |
| 2 | Jog Control | 🕹️ | Yes | Step sizes, speed multipliers, quick actions |
| 3 | Calibration | 📐 | Yes | Camera settings, plate format, cal data |
| 4 | Print Setup | 🖨️ | Yes | Print settings (per-pump retract/prime) |
| 5 | Print Monitor | 📈 | Yes | Recording browser, job queue |
| 6 | Settings | ⚙️ | No | Safety toggle, simulation, serial ports |

**Page gating**: Pages 1–5 are disabled until `HardwareConfig.is_valid == True`. Pages 0 and 6 are always accessible.

---

## 6. Signal Flow (v7.2.3)

### 6.1 Hardware Configuration Propagation

```
User edits Hardware Setup (Page 0)
  │ config_changed(HardwareConfig)
  ▼
app.py._on_hardware_config_changed(config)
  ├── _save_hardware_config(config)          # persist to settings.json
  ├── _update_page_gating(config.is_valid)   # enable/disable pages 1–5
  └── _propagate_hardware_config(config)     # forward to all pages
        │
        ├── page.set_hardware_config(config)  # every page gets it
        │     [guard: _propagating_config flag prevents re-entrancy]
        │     [guard: skip HardwareSetupPage to avoid circular emit]
        │
        └── PrintSetupPage.set_hardware_config(config)
              │ → tab_workspace.set_hardware_config(config)
              │     → _hardware_config_to_workspace()    # bridge method
              │     → workspace_changed(WorkspaceConfig)
              │         → tab_objects.set_workspace(ws)
              │         → tab_wells.set_workspace(ws)
              ▼
            All tabs updated with converted WorkspaceConfig
```

### 6.2 HardwareConfig → WorkspaceConfig Bridge

The bridge method `_hardware_config_to_workspace()` in `print_workspace.py` converts:

| HardwareConfig Field | WorkspaceConfig Field | Conversion |
|----------------------|----------------------|------------|
| `needle` | `needle` | Direct copy |
| `plate_format` | `plate_format` | Direct copy |
| `ink_library` | `ink_library` | Direct copy |
| `rosette_library` | `rosette_library` | Direct copy |
| `PumpChannelConfig` | `PumpLoadout` | Syringe + mode copied; ink resolved from library |
| `buffer_ink_name` | `buffer_ink` | Name → InkSpec lookup from ink_library |

### 6.3 Print Job Pipeline

```
User configures wells → clicks "📤 Send to Monitor ▶"
  │ PrintSetupPage.job_ready(PrintJob)
  ▼
app.py._send_job_to_monitor(job)
  ├── monitor.receive_job(job)     # enqueue in monitor
  └── _switch_page(5)              # auto-navigate to monitor
  ▼
User clicks "▶ Start Print" on Monitor
  │ monitor.start_requested(PrintJob)
  ▼
app.py._on_monitor_start(job)
  │ → print_manager.start(job)
  │
  │ PrintManager callbacks (chained):
  │   on_progress → monitor.on_print_progress(step, total, msg)
  │   on_state_changed → monitor.on_print_state_changed(state)
  ▼
Real-time plate/trajectory/syringe visualization updates
```

### 6.4 Execution Control Signals

```
Monitor.pause_requested()  → app._on_monitor_pause()  → PrintManager.pause()
Monitor.resume_requested() → app._on_monitor_resume() → PrintManager.resume()
Monitor.abort_requested()  → app._on_monitor_abort()  → PrintManager.abort()
```

### 6.5 Print File Lifecycle

```
PrintFileManager.new_file(name)  → creates config/prints/{name}.json
PrintFileManager.save()          → writes current state + timestamp
PrintFileManager.load(name)      → read + validate + migrate → PrintFileData
  ↕ auto_save_timer (30s)        → _sync_ui_to_file() → save()
  ↓ file_changed signal          → app._on_print_file_changed(name)
```

---

## 7. Hardware Configuration Model

### 7.1 HardwareConfig (Single Source of Truth)

```
HardwareConfig
  ├── config_name: str
  ├── notes: str
  ├── needle: NeedleSpec (gauge, OD/ID/wall µm, length, channels)
  ├── plate_format: int (6/12/24/48/96/384)
  ├── ink_library: dict[str, InkSpec]
  ├── rosette_library: dict[str, RosetteInsert]
  ├── buffer_ink_name: str
  ├── pumps: dict[str, PumpChannelConfig]
  │     P1 ─┐
  │     P2 ──┤  PumpChannelConfig
  │     P3 ──┘    ├── syringe: SyringeSpec (volume, barrel_id, stroke)
  │               ├── ink: InkSpec (name, viscosity, particles)
  │               ├── printing_mode: INCREMENTAL | CONTINUOUS
  │               ├── fluid_column: FluidColumn (oil/buffer/ink layers)
  │               └── enabled: bool
  └── is_valid: bool (computed)
```

### 7.2 µL ↔ mm Conversion Chain

```
User enters: 5.0 µL at 0.25 µL/s
  ↓ PumpChannelConfig.uL_to_mm(5.0)
  ↓ SyringeSpec(100µL).mm_per_uL = 0.3  →  5.0 × 0.3 = 1.5 mm
  ↓ PumpChannelConfig.feedrate_uL_s_to_mm_min(0.25)
  ↓ 0.25 × 0.3 × 60 = 4.5 mm/min
  ↓ StageController.move_pump_relative("P1", 1.5, 4.5)
  ↓ ZPStage: Marlin G-code → G1 Y1.5 F4.5
```

### 7.3 Hardware Setup Dependency-Ordered Restore

When loading a saved config or auto-restoring on startup, `_apply_config_to_ui()` follows a strict order to avoid broken cross-references:

```
1. Config name + notes
2. Ink library → _refresh_ink_table() + _refresh_pump_ink_combos()
3. Rosette library → _refresh_rosette_table()
4. Needle gauge + length + channels
5. Plate format
6. Each pump: enable → syringe → ink (now resolvable) → mode
7. Emit signals after full restore
```

All widget signals are blocked during restore to prevent cascading `_on_config_changed` calls.

---

## 8. Page Details

### 8.1 Hardware Setup (Page 0) — 1041 lines, 4 classes

| Class | Purpose |
|-------|---------|
| `InkEditorDialog` | Modal dialog for creating/editing ink specifications |
| `RosetteEditorDialog` | Modal dialog for rosette inserts with `create_standard()` |
| `PumpChannelWidget` | Per-pump config: enable, syringe, ink, mode |
| `HardwareSetupPage` | Main page with 6 sections in dependency order |

**UI Section Order**: Name → Needle → Plate → Ink Library → Pumps → Rosettes

**Key signals**: `config_changed(HardwareConfig)`, `config_validated(bool)`

### 8.2 Dashboard (Page 1)

Device status, live XY/ZP positions, speed indicators, print history table. Connection cards in context panel for XY stage, ZP stage, and Xbox controller.

### 8.3 Jog Control (Page 2)

Manual movement with D-pad buttons, keyboard shortcuts, and Xbox controller. Step sizes in mm (XY/Z) and µL (pumps). Speed multiplier sliders. Quick actions for homing and zeroing.

### 8.4 Calibration (Page 3)

Camera feed with crosshair overlay for visual alignment. Plate corner calibration (3-point or 4-point). Z-probing for plate surface detection. Calibration data persistence.

### 8.5 Print Setup (Page 4) — 623 lines, 2 classes

Three-tab workflow:

| Tab | Widget | Function |
|-----|--------|----------|
| 1. Workspace | `WorkspaceTab` (print_workspace.py) | Read-only hardware summary + HardwareConfig→WorkspaceConfig bridge |
| 2. Print Objects | `PrintObjectsTab` | Designer + auto-layout + persistent print files |
| 3. Well Setup | `WellSetupTab` | Assign prints to wells, calibrate plane, set roles |

**Context panel**: Print settings only — XY/Z feed rates, pump rate (µL/s), layers, layer height, active pump, flow rate, travel Z, per-pump retract/prime volumes.

**Key signals**: `workspace_updated(WorkspaceConfig)`, `navigate_to_page(int)`, `job_ready(PrintJob)`

### 8.6 Print Workspace (Tab 1 of Page 4) — 580 lines, 2 classes

| Class | Purpose |
|-------|---------|
| `HardwareSummaryWidget` | Read-only display of needle, plate, pumps, inks, rosettes |
| `WorkspaceTab` | Summary + "Edit Hardware Setup ▶" button + bridge method |

**Summary sections**: Needle (gauge, dimensions), Plate (format, dimensions), Pumps (P1/P2/P3 with syringe/ink/mode), Ink Library (table), Rosette Library (table), Status indicator (✓ configured / ⚠ incomplete).

### 8.7 Print Monitor (Page 5) — v7.2.3 additions: 334 lines

Receives jobs from Print Setup. Job queue display with Start/Pause/Abort buttons. Real-time visualization: full plate overview (color-coded progress), current well trajectory view (XY/ZY/XZ projections), syringe fill-level bars. Recording browser in context panel.

**Key signals**: `start_requested(PrintJob)`, `pause_requested()`, `resume_requested()`, `abort_requested()`

### 8.8 Settings (Page 6)

Safety limits toggle, simulation mode indicators, serial port configuration, controller protocol selection. Apply/Reset buttons. Hardware config forwarding for dual mm/µL display.

---

## 9. Print File System (v7.2.3)

### 9.1 PrintFileManager

CRUD manager for persistent print files stored in `config/prints/`.

| Method | Description |
|--------|-------------|
| `new_file(name)` | Create blank print with schema v7.2.3 |
| `save()` / `save_as(name)` | Persist to JSON with timestamp |
| `load(name)` | Read + validate + migrate from disk |
| `delete(name)` | Remove file with confirmation |
| `duplicate(new_name)` | Deep copy under new name |
| `list_files()` | Enumerate saved prints with metadata |
| `auto_save()` | Timer-driven save when dirty flag set |

### 9.2 Print File Schema v7.2.3

```json
{
    "schema_version": "7.2.3",
    "metadata": {
        "name": "Scaffold_v1",
        "description": "",
        "created": "2026-03-02T12:00:00+00:00",
        "modified": "2026-03-02T12:00:00+00:00",
        "author": ""
    },
    "objects": {
        "Cylinder_1": {
            "object_type": "cylinder_solid",
            "params": {"radius": 2.0, "height": 0.6, "layer_height": 0.2},
            "position": [0.0, 0.0, 0.0],
            "color": "#a6e3a1",
            "ink_pump": "P1",
            "num_layers": 3,
            "layer_height": 0.2
        }
    },
    "collections": {
        "Group_1": [
            {"object_name": "Cylinder_1", "position": [0, 0, 0], "copies": 1}
        ]
    },
    "layout_presets": {
        "Ring_6": {
            "pattern": "ring",
            "params": {"n": 6, "radius": 2.0}
        }
    }
}
```

### 9.3 Auto-Layout Patterns

| Pattern | Parameters | Generator |
|---------|-----------|-----------|
| Ring | count, radius, start_angle, center | `auto_layout_ring()` |
| Square Grid | rows, cols, spacing, center | `auto_layout_grid()` |
| Hex Grid | rows, cols, spacing, center | `auto_layout_hex()` |
| Line | count, start(x,y), end(x,y) | `auto_layout_line()` |
| Concentric Rings | ring_count, inner_r, outer_r, points_per_ring | `auto_layout_concentric()` |

All generators return `list[tuple[float, float]]` positions and include well-boundary validation via `validate_layout()`.

---

## 10. Threading Model

```
┌─ Main Thread (GUI event loop) ─────────────────────────────────────────────┐
│  QTimer (300ms) → on_status_update() → read cached positions               │
│  Button clicks → controller.connect_stages() / print_manager.start() etc.  │
└────────────────────────────────────────────────────────────────────────────┘

┌─ Processor Thread ─────────────────────────────────────────────────────────┐
│  Blocks on queue.Queue.get(), dispatches to registered handlers            │
└────────────────────────────────────────────────────────────────────────────┘

┌─ XboxQueuePoller Thread ───────────────────────────────────────────────────┐
│  Reads multiprocessing.Queue at 50Hz, feeds Processor                      │
└────────────────────────────────────────────────────────────────────────────┘

┌─ Xbox Process (separate OS process) ───────────────────────────────────────┐
│  pygame joystick polling at 100Hz, writes to multiprocessing.Queue         │
└────────────────────────────────────────────────────────────────────────────┘

┌─ XYJogHandler Thread ──────────────────────────────────────────────────────┐
│  50ms loop: reads current stick values → sends VS command to XY stage      │
└────────────────────────────────────────────────────────────────────────────┘

┌─ ZPJogHandler Thread ──────────────────────────────────────────────────────┐
│  100ms loop: reads current stick values → sends G0 relative moves          │
└────────────────────────────────────────────────────────────────────────────┘

┌─ PositionPoller Thread ────────────────────────────────────────────────────┐
│  300ms loop: queries XY + ZP positions → updates thread-safe cache         │
└────────────────────────────────────────────────────────────────────────────┘

┌─ ConnectionWatchdog Thread ────────────────────────────────────────────────┐
│  3s loop: checks serial port health → fires disconnect callback            │
└────────────────────────────────────────────────────────────────────────────┘

┌─ PrintExec Thread ─────────────────────────────────────────────────────────┐
│  Sequential command execution with pause/abort event waits                 │
│  TrajectoryExecutor runs inside this thread for TRAJECTORY commands        │
│  ServiceSequenceExecutor runs inside for SERVICE_SEQ commands              │
│  PrintRecorder.record_sample() called on each waypoint                     │
└────────────────────────────────────────────────────────────────────────────┘

┌─ PrintQueue Thread ────────────────────────────────────────────────────────┐
│  Iterates queue, loads each job into PrintManager, waits for completion    │
└────────────────────────────────────────────────────────────────────────────┘
```

**Total active threads during printing with Xbox**: 8 threads + 1 separate process.

**Thread safety**: All cross-thread data access uses `threading.Lock`. GUI updates from worker threads go through `QTimer`-polled caches or Qt signal/slot connections. The `PrintSignalBridge` class provides thread-safe `Signal` emission from `PrintManager` callbacks.

---

## 11. Hardware Interface

### 11.1 ZP Stage Axis Mapping

The Marlin board maps logical axes to printer axes:

| Logical Axis | Printer Axis | Marlin G-code | Purpose |
|-------------|-------------|---------------|---------|
| Z | X | G1 Xnn Fnn | Vertical needle movement |
| P1 | Y | G1 Ynn Fnn | Syringe pump 1 |
| P2 | Z | G1 Znn Fnn | Syringe pump 2 |
| P3 | E | G1 Enn Fnn | Syringe pump 3 |

### 11.2 XY Stage Protocol Abstraction

Commands are defined in JSON protocol files, not hardcoded:

```json
{
    "name": "ProScan III",
    "commands": {
        "get_position": {"cmd": "P", "response_pattern": "(-?\\d+),(-?\\d+)"},
        "move_absolute": {"cmd": "G {x},{y}", "response": "R"},
        "set_velocity": {"cmd": "VS {x},{y}", "response": "0"}
    },
    "params": {
        "microsteps_per_um": 10,
        "max_velocity": 50000
    }
}
```

Auto-detection iterates all JSON files in `config/controllers/` and tests firmware queries.

### 11.3 Hamilton Syringe Reference

| Volume (µL) | PN Base | Barrel ID (mm) | µL per mm | mm per µL |
|-------------|---------|----------------|-----------|-----------|
| 25 | 1702 | 1.030 | 0.833 | 1.200 |
| 50 | 1705 | 1.457 | 1.667 | 0.600 |
| 100 | 1710 | 2.060 | 3.333 | 0.300 |
| 250 | 1725 | 3.256 | 8.333 | 0.120 |
| 500 | 1750 | 4.606 | 16.667 | 0.060 |
| 1000 | 1001 | 6.513 | 33.333 | 0.030 |

---

## 12. Print Command Types

| Command | Description | Key Parameters |
|---------|-------------|----------------|
| `MOVE_XY` | Absolute XY move (relative to zero ref) | `x`, `y`, `feedrate` |
| `MOVE_Z` | Absolute Z height | `z`, `feedrate` |
| `MOVE_Z_REL` | Relative Z movement | `dz`, `feedrate` |
| `EXTRUDE` | Pump dispense (relative) | `pump`, `amount_uL`, `rate_uL_s` |
| `PRINT_PATH` | Coordinated XY + extrusion | `path`, `flow_rate_uL_s` |
| `DWELL` | Wait | `duration_s` |
| `TRAVEL_UP` | Raise to travel height | `travel_z` |
| `TRAVEL_DOWN` | Lower to print height | `print_z` |
| `SET_PUMP_RATE` | Set pump flow rate | `pump`, `rate_uL_s` |
| `HOME_XY` | Move to zero reference | — |
| `SWITCH_PUMP` | Change active pump | `pump_id` |
| `TRAJECTORY` | Execute continuous path | `waypoints`, `dt` |
| `SERVICE_SEQ` | Waste→wash→buffer→ink cycle | `pump`, `sequence` |

---

## 13. Logging Strategy

All modules use `logging.getLogger(__name__)`. No `print()` in backend code.

| Level | Usage | Examples |
|-------|-------|---------|
| `DEBUG` | High-frequency routine | Position polls, waypoint execution, sample recording |
| `INFO` | User-visible state changes | Stage connected, print started, recording saved |
| `WARNING` | Recoverable interventions | Safety limit clamped, ink change triggered, settle timeout |
| `ERROR` | Failures requiring attention | Serial disconnect, trajectory abort, recording failure |

---

## 14. Page Interface Contract

Every page implements this interface for consistent integration with `app.py`:

```python
class PageWidget(QWidget):
    def get_page_title(self) -> str: ...
    def get_page_subtitle(self) -> str: ...          # optional
    def get_context_widget(self) -> QWidget | None: ...
    def on_status_update(self): ...                  # called by 300ms QTimer
    def set_hardware_config(self, config): ...       # v7.2: receive HardwareConfig
    def set_microsteps_per_micron(self, value): ...  # XY stage calibration
```

---

## 15. Test Architecture

Tests use **physics-based simulators** rather than simple mocks. `XYStageSimulator` and `ZPStageSimulator` model acceleration ramps, serial buffering, and smooth motion interpolation, providing realistic behavior during offline development and CI testing.

| Suite | Tests | Coverage |
|-------|-------|---------|
| Session 1 (Hardware Setup) | 30 | Config restore, dependency ordering, ink resolution |
| Session 2 (Workspace + Setup) | 40 | Bridge method, read-only summary, per-pump settings |
| Session 3 (Execution) | 42 | Job pipeline, monitor wiring, PrintManager delegation |
| Session 4 (Print Files) | 69 | CRUD lifecycle, auto-layout, schema validation |
| Session 5 (Integration) | 52 | End-to-end workflows, persistence, edge cases |
| **Total** | **233** | |

---

## 16. Version History

| Version | Date | Highlights |
|---------|------|-----------|
| 7.0 | 2025 | PyDracula GUI rewrite, 5-page structure |
| 7.1 | 2025 | PhysicalModels, GeometryEngine, TrajectoryPlanner, MotionController, FlowPhysics, PrintRecorder, Print Monitor page, JSON controller protocols |
| 7.2 | 2026-02 | Hardware Setup page (Page 0), all pump motion in µL, HardwareConfig save/load, page gating |
| **7.2.3** | **2026-03** | **Hardware restore fix, read-only workspace, execution→monitor, print file persistence, auto-layout** |
