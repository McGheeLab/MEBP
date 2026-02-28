# MEBP Version 7.1 — System Architecture

## 1. Purpose

Desktop application for controlling a multi-axis 3D bioprinting platform. Drives an XY microscope stage (Prior ProScan II/III via RS-232), a Z needle + 3 syringe pumps (Marlin 3D printer board), and an Xbox controller for manual jogging. Provides a PySide6 GUI for calibration, print job setup with well plate support, live trajectory visualization, print recording/replay, and a print queue.

### What Changed in v7.1

Version 7.1 is the **Print System Overhaul** — it adds trajectory-based execution, physical models for syringes/needles/inks, recording infrastructure, a dedicated Print Monitor page, and deep protocol abstraction for multi-controller support. The changes span 10 modified files across backend and frontend layers, adding ~2,200 lines of new code.

---

## 2. Hardware

| Device | Interface | Protocol | Notes |
|--------|-----------|----------|-------|
| **XY Stage** — Prior ProScan II or III | RS-232 serial (9600 baud) | ASCII commands via JSON protocol files | Position units: microsteps. Auto-detectable via firmware query. |
| **ZP Stage** — 3D printer board (Marlin firmware) | USB serial (115200 baud) | Standard G-code: `G0`, `G90`/`G91`, `M114`, `M302`, `M83`, `M92`, `M203`, `M500`, `M112`, `M115` | 4 logical axes mapped to printer axes (see table below). |
| **Xbox Controller** | USB HID via pygame | Polled at ~100 Hz in a separate process | Button mapping configurable via JSON file, hot-reloadable. |

### ZP Axis Mapping

The Marlin board's 4 printer axes are remapped to bioprinter-meaningful names:

| Logical Name | Printer Axis | Description |
|-------------|-------------|-------------|
| Z | X | Vertical needle (mm) |
| P1 | Y | Syringe pump 1 (mm) |
| P2 | Z | Syringe pump 2 (mm) |
| P3 | E | Syringe pump 3 (mm) |

This mapping is defined as `AXIS_MAP` in `ZPStage.py` and is the single source of truth used by all modules.

### v7.1: Controller Protocol Abstraction

XY stage commands are now defined in JSON protocol files (`config/controllers/proscan_ii.json`, `proscan_iii.json`) rather than hardcoded. The `ControllerProtocol` class loads these at runtime. `XYStage.py` accepts a `controller_json` parameter:
- `None` → default ProScan III
- `"auto"` → auto-detect by iterating all JSON files and testing firmware queries
- Explicit path → load that specific protocol

---

## 3. Folder Structure

```
MEBP-Version-7.1/
├── main.py                              # Entry point — CLI arg parsing, mode selection
├── settings.json                        # Persistent app config (auto-saved)
├── current_button_mapping.json          # Xbox controller mapping (hot-reloadable)
│
├── config/
│   ├── controllers/
│   │   ├── proscan_ii.json              # ★ v7.1 — ProScan II command map
│   │   └── proscan_iii.json             # ★ v7.1 — ProScan III command map
│   └── hardware/
│       ├── needles.json                 # ★ v7.1 — Needle gauge catalog (16G–32G+)
│       └── syringes.json                # ★ v7.1 — Hamilton syringe catalog
│
├── SupportClasses/                      # Backend — zero GUI dependencies
│   ├── __init__.py                      # Package docstring + module listing
│   ├── Processor.py                     # Thread-safe command bus (pub/sub)
│   ├── SerialUtils.py                   # Retry decorator, safe I/O, port discovery, watchdog
│   ├── XYStageSimulator.py              # Prior ProScan simulator
│   ├── ZPStageSimulator.py              # Marlin board simulator
│   ├── XYStage.py                       # ✏ v7.1 — Prior ProScan manager + ControllerProtocol
│   ├── ZPStage.py                       # Marlin board manager (G-code)
│   ├── XboxController.py                # Polling worker (multiprocessing.Process)
│   ├── StageController.py               # ✏ v7.1 — Orchestrator + velocity API + rate testing
│   ├── ControllerProtocol.py            # ★ v7.1 — JSON-based command protocol loader
│   ├── PrintManager.py                  # ✏ v7.1 — Execution engine + trajectory + service seq
│   ├── PrintRecorder.py                 # ★ v7.1 — Auto-record prints, replay data
│   ├── WellPlate.py                     # ✏ v7.1 — ANSI/SLAS plates + 384-well + z-offsets
│   ├── WellSetup.py                     # ★ v7.1 — Well assignments, rosettes, plane fitting
│   ├── SafetyLimits.py                  # ✏ v7.1 — Software endstops + flow-rate clamping
│   ├── Settings.py                      # ✏ v7.1 — JSON config + v7.1 sections
│   ├── PositionLogger.py                # Timestamped position recording + CSV/JSON export
│   ├── PrintHistory.py                  # Persistent print history log
│   ├── PhysicalModels.py                # ★ v7.1 — Syringe, Needle, Ink, FluidColumn
│   ├── GeometryEngine.py                # ★ v7.1 — Parametric print objects + needle paths
│   ├── TrajectoryPlanner.py             # ★ v7.1 — Time-parameterized paths, CSV import
│   ├── MotionController.py              # ★ v7.1 — Kalman filter, PID, feedforward tracking
│   └── FlowPhysics.py                   # ★ v7.1 — Pressure/flow calculations, safety
│
├── gui/                                 # Frontend — PySide6, PyDracula-style layout
│   ├── __init__.py
│   ├── app.py                           # ✏ v7.1 — MainWindow + Print Monitor page + recorder
│   ├── styles.py                        # QSS dark theme (Catppuccin Mocha) + COLORS dict
│   ├── ui_functions.py                  # Animation helpers: toggleMenu, toggleLeftBox
│   ├── pages/
│   │   ├── __init__.py
│   │   ├── dashboard.py                 # Device status, positions, speeds, history
│   │   ├── jog_control.py               # Manual movement, dpad, speed control
│   │   ├── calibration.py               # Guided 3-step calibration with camera
│   │   ├── print_setup.py               # Job loading, preview, execution, queue
│   │   ├── print_monitor.py             # ✏ v7.1 — Live trajectory, plate progress, recordings
│   │   ├── settings_page.py             # ✏ v7.1 — Connections, safety, controller selector
│   │   ├── print_workspace.py           # ★ v7.1 — Workspace config (syringe/needle/ink)
│   │   ├── print_well_setup.py          # ★ v7.1 — Well assignment & rosette setup
│   │   └── print_objects.py             # ★ v7.1 — Print file/object configuration
│   ├── widgets/
│   │   ├── __init__.py
│   │   ├── console_log.py               # Log viewer + QtLogHandler (thread-safe)
│   │   ├── xbox_mapping_editor.py        # Button/axis/dpad mapping editor dialog
│   │   ├── camera_widget.py             # Live microscope camera feed (OpenCV)
│   │   ├── syringe_display.py           # ★ v7.1 — Visual syringe fill-level indicator
│   │   ├── trajectory_view.py           # ★ v7.1 — Multi-projection needle path canvas
│   │   └── well_plate_view.py           # ★ v7.1 — Interactive top-down plate view
│   └── tests/
│       └── test_integration.py          # Headless integration test
│
└── sample_jobs/
    └── multi_material_test.json         # Example multi-material print job
```

### File Summary

| Layer | Files | Lines | v7.0 → v7.1 |
|-------|-------|-------|-------------|
| **SupportClasses** | 21 files | ~7,200 | +6 new files, 7 modified |
| **gui/** | ~20 files | ~8,600 | +4 new pages/widgets, 3 modified |
| **main.py** | 1 file | ~150 | Unchanged |
| **config/** | 4 files | ~400 | All new |

---

## 4. Module Dependency Graph

```
main.py
  ├── SupportClasses.StageController   ← the single backend entry point
  │     ├── Processor
  │     ├── XYStage
  │     │     ├── ControllerProtocol           ★ v7.1
  │     │     ├── Simulators.XYStageSimulator  (sim mode)
  │     │     └── SerialUtils                  (real mode)
  │     ├── ZPStage
  │     │     ├── Simulators.ZPStageSimulator  (sim mode)
  │     │     └── SerialUtils                  (real mode)
  │     ├── XboxController (multiprocessing.Process)
  │     ├── SafetyLimits
  │     ├── PositionLogger
  │     └── SerialUtils.ConnectionWatchdog
  │
  ├── SupportClasses.PrintManager      ← execution engine
  │     ├── TrajectoryExecutor           ★ v7.1
  │     │     └── PrintRecorder          ★ v7.1
  │     ├── ServiceSequenceExecutor      ★ v7.1
  │     └── FluidColumnTracker           ★ v7.1
  │
  ├── SupportClasses.Settings          ← persistent config (expanded v7.1)
  │
  └── gui.app.MainWindow               (GUI mode only)
        ├── gui.styles                 → DARK_THEME QSS, COLORS dict
        ├── gui.ui_functions           → Animation helpers
        ├── gui.pages.dashboard        → StageController (read-only), PrintHistory
        ├── gui.pages.jog_control      → StageController (jog commands)
        ├── gui.pages.calibration      → StageController, CameraWidget
        ├── gui.pages.print_setup      → PrintManager, PrintQueue, WellPlate
        ├── gui.pages.print_monitor    → PrintRecorder, TrajectoryView      ★ v7.1
        ├── gui.pages.settings_page    → StageController, SafetyLimits, ControllerProtocol
        ├── SupportClasses.PrintRecorder  ★ v7.1
        └── gui.widgets.*
```

**Key rule: GUI pages never talk to serial ports directly. Everything goes through StageController or PrintManager.**

---

## 5. v7.1 New Modules

### 5.1 PrintRecorder.py (627 lines) ★ NEW

Automatic print recording system. Creates timestamped data files per print job.

- **`PrintRecorder`**: Manages recording lifecycle. `start_recording()` → `record_sample()` × N → `stop_recording()`.
- **Sample data**: Each sample stores `(t, planned_xyz_p123, actual_xy, actual_zp, segment_id, is_travel, is_retract)`.
- **Auto-summary**: On stop, computes mean/max/RMS tracking error (XY, Z), total duration, segment count.
- **File I/O**: Saves JSON metadata + CSV sample data. `list_recordings()`, `load_recording()`, `delete_recording()`.
- **`RecordingInfo`**: Named tuple for browsing recordings (job name, date, sample count, path).
- Integration: PrintManager auto-starts recording on `start()` and stops on completion/abort/error.

### 5.2 TrajectoryExecutor (in PrintManager.py)

Executes time-parameterised trajectories using the MotionController. Replaces discrete MOVE_XY + EXTRUDE commands with smooth, continuous trajectory tracking.

- Runs waypoint list at fixed timestep, commanding stage positions at each waypoint time.
- Records actual vs planned positions via PrintRecorder.
- Supports pause/resume/abort via threading events.
- Waypoints: objects with `(t, x, y, z, p1, p2, p3)` fields.

### 5.3 ServiceSequenceExecutor (in PrintManager.py)

Automates fluid handling between prints: waste → wash → buffer → ink pickup.

- Each step finds the nearest well with the required role from WellSetup.
- Configurable per-step volumes and feedrates.
- Integrates with FluidColumnTracker for volume accounting.

### 5.4 FluidColumnTracker (in PrintManager.py)

Tracks per-pump ink state during printing.

- Monitors ink name, remaining volume, and dispensing mode per pump.
- `needs_ink_change(pump, required_ink)` → triggers service sequence automatically.
- `needs_refill(pump)` → signals when volume drops below threshold.
- Incremental mode: pick up ink before each well. Continuous mode: track remaining volume.
- Serialisable state for persistence via `get_state()` / `restore_state()`.

---

## 6. v7.1 Modified Modules

### 6.1 XYStage.py (314 → 557 lines)

- **ControllerProtocol integration**: Loads JSON protocol files for command formatting, encoding, and limits.
- **Auto-detection**: `_auto_detect_controller()` iterates all JSON files and tests firmware queries.
- **Protocol-driven commands**: `_send_protocol_command()` abstracts command formatting via protocol.
- **New methods**: `get_firmware_version()`, `stop_stage()`.
- **Backward compatible**: All existing methods preserved; simulator mode unchanged.

### 6.2 StageController.py (766 → 881 lines)

- **`controller_json` parameter**: Passed through to XYStage for protocol selection.
- **`send_velocity_xy(vx, vy)`**: Direct velocity command for trajectory tracking.
- **`get_position_with_timestamp()`**: Returns monotonic timestamp + XY/ZP positions + latency estimate.
- **`test_command_rate(num_tests)`**: Safe position-query rate test returning avg/min/max ms and max Hz.

### 6.3 PrintManager.py (1,469 → 2,014 lines)

- **New CommandTypes**: `TRAJECTORY`, `SERVICE_SEQUENCE`.
- **New classes**: `TrajectoryExecutor`, `ServiceSequenceExecutor`, `FluidColumnTracker`.
- **PrintJob additions**: `workspace`, `well_setup`, `trajectory_waypoints` fields.
- **PrintRecorder integration**: Auto-start/stop recording on print lifecycle.
- **Ink change detection**: `SWITCH_PUMP` now checks `needs_ink_change()` and auto-triggers service sequence.
- **All existing commands**: Fully backward compatible.

### 6.4 WellPlate.py (319 → 406 lines)

- **384-well plate**: Added to `PLATE_DEFINITIONS` per ANSI/SLAS-2004 (16×24 grid, 4.5mm spacing).
- **Per-well z-offsets**: `set_well_z_offset()`, `get_well_z_offset()` for bottom plane compensation.
- **Rosette insert support**: `set_rosette()`, `get_rosette()` for sub-well pattern fixtures.
- **Bulk operations**: `set_all_z_offsets()`, `clear_z_offsets()`.

### 6.5 SafetyLimits.py (211 → 274 lines)

- **Flow rate clamping**: `clamp_flow_rate(pump, rate, needle)` limits flow based on needle gauge.
- **Max flow rates per gauge**: `MAX_FLOW_RATES_UL_MIN` lookup table (25G → 500 µL/min, 30G → 50 µL/min, etc.).
- **`needle_gauge` property**: Current needle gauge for flow rate validation.

### 6.6 Settings.py (183 → 260 lines)

New v7.1 sections with deep-merge backward compatibility:
- **`controller`**: `controller_json`, `auto_detect_result`, `controllers_dir`.
- **`workspace`**: `needle_gauge`, `plate_format`, `pump_loadouts`, `buffer_ink_name`.
- **`ink_library`**: Named ink specifications.
- **`rosette_library`**: Named rosette insert definitions.
- **`well_setup`**: `last_saved_file`, `auto_save`.
- **`motion_controller`**: PID/Kalman tuning, `rate_test_results`.
- **`fluid_columns`**: Per-pump fluid state (P1/P2/P3).

### 6.7 settings_page.py (668 → 823 lines)

- **Controller selector card**: Dropdown with "Auto-Detect", "Default (ProScan III)", + all discovered JSON protocol files.
- **Auto-detect button**: Tests firmware queries and displays detected controller.
- **Rate test button**: Measures command round-trip time and displays avg/min/max ms + Hz.
- **Persistence**: Controller selection and rate test results saved to Settings.

### 6.8 print_monitor.py (665 → 884 lines)

- **Page interface**: `get_page_title()`, `get_context_widget()` for navigation integration.
- **Recording browser**: Context panel with list of past recordings, select, load, delete.
- **Replay visualization**: Load recording and overlay actual path on trajectory view with tracking error stats.
- **Recorder integration**: `set_recorder()` receives PrintRecorder reference from app.py.

### 6.9 app.py (872 → 946 lines)

- **6th navigation page**: Print Monitor (📈) added between Print Setup and Settings.
- **PrintRecorder**: Created in MainWindow, wired to both PrintManager and PrintMonitorPage.
- **WorkspaceConfig sharing**: `pass_workspace_to_monitor()` passes workspace between pages.
- **Monitor signal handlers**: Pause/Resume/Abort forwarded from monitor to PrintManager.

---

## 7. Threading Model

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
│  v7.1: TrajectoryExecutor runs inside this thread for trajectory commands  │
│  v7.1: ServiceSequenceExecutor runs inside for service_seq commands        │
│  v7.1: PrintRecorder.record_sample() called on each waypoint              │
└────────────────────────────────────────────────────────────────────────────┘

┌─ PrintQueue Thread ────────────────────────────────────────────────────────┐
│  Iterates queue, loads each job into PrintManager, waits for completion    │
└────────────────────────────────────────────────────────────────────────────┘
```

**Total active threads during printing with Xbox**: 8 threads + 1 separate process.

---

## 8. v7.1 Data Flow: Trajectory Execution

```
GUI: user clicks Start (with trajectory-based job)
  │  PrintManager.start()
  │    _start_recorder() → PrintRecorder.start_recording()
  │    TrajectoryExecutor created with recorder
  ▼
PrintExec Thread spawns
  │  for each PrintCommand:
  │    ├── TRAJECTORY → TrajectoryExecutor.execute(waypoints)
  │    │     for each waypoint at time t:
  │    │       wait until t
  │    │       controller.move_xy_absolute(wp.x, wp.y)
  │    │       controller.move_z_absolute(wp.z)
  │    │       controller.zp_stage.move_absolute(pump positions)
  │    │       recorder.record_sample(planned, actual)
  │    │
  │    ├── SERVICE_SEQUENCE → ServiceSequenceExecutor.execute_sequence()
  │    │     for each step: waste → wash → buffer → ink
  │    │       move to service well, perform aspiration/ejection
  │    │       FluidColumnTracker updates volume
  │    │
  │    ├── SWITCH_PUMP → check needs_ink_change() → auto-trigger service seq
  │    │
  │    └── (legacy commands: MOVE_XY, PRINT_PATH, etc. — unchanged)
  ▼
Print completes/aborts/errors
  │  _stop_recorder("completed") → PrintRecorder.stop_recording()
  │    → saves metadata JSON + samples CSV
  │    → computes tracking error summary
```

---

## 9. Safety Architecture

### Layers

1. **SafetyLimits clamping** (software): All `StageController.move_*` methods check limits and clamp. v7.1 adds flow rate clamping per needle gauge.
2. **Jog handler dampening** (software): `XYJogHandler` scales velocity near boundaries.
3. **Abort Z-raise** (software): `PrintManager.abort()` raises Z to travel height.
4. **FluidColumnTracker** (v7.1, software): Tracks ink volumes and triggers refill sequences.
5. **Marlin endstops** (firmware): Physical endstop switches.
6. **Emergency stop**: `ZPStage.emergency_stop()` sends `M112`.

---

## 10. Configuration Files

### 10.1 settings.json (v7.1 expanded)

```json
{
  "window": {"x": 100, "y": 100, "width": 1200, "height": 800, "active_tab": 0},
  "simulation": {"simulate_xy": true, "simulate_zp": true},
  "speeds": {"xy": 1000, "z": 5.0, "p": 3.0},
  "zero_position": {"x": 0, "y": 0, "Z": 0, "P1": 0, "P2": 0, "P3": 0},
  "safety_limits": {"enabled": true, "xy_min_x": -100000, "...": "..."},
  "polling": {"position_interval_ms": 300, "watchdog_interval_s": 3.0},
  "xbox": {"mapping_file": "current_button_mapping.json"},
  "logging": {"verbose": false},
  "controller": {"controller_json": null, "auto_detect_result": null},
  "workspace": {"needle_gauge": 25, "plate_format": 96, "pump_loadouts": {}},
  "ink_library": {"inks": {}},
  "rosette_library": {"rosettes": {}},
  "well_setup": {"last_saved_file": null, "auto_save": true},
  "motion_controller": {"controller_type": "pid", "pid_kp": 1.0, "rate_test_results": null},
  "fluid_columns": {"P1": null, "P2": null, "P3": null}
}
```

### 10.2 Controller Protocol JSON (v7.1)

```json
{
  "name": "Prior ProScan III",
  "manufacturer": "Prior Scientific",
  "commands": {
    "set_velocity": {"format": "VS {vx},{vy}", "response": "R"},
    "move_absolute": {"format": "G {x},{y}", "response": "R"},
    "get_position": {"format": "P", "response": "{x},{y},{status}"},
    "get_firmware": {"format": "V", "response": "{version}"}
  },
  "detection": {"command": "V", "tokens": ["ProScan"]},
  "encoding": "ascii",
  "tx_terminator": "\r\n",
  "rx_terminator": "\r"
}
```

### 10.3 Print Job JSON (v7.1 extended)

```json
{
  "name": "My Print",
  "settings": {"xy_feedrate": 1000, "z_feedrate": 60, "num_layers": 3},
  "commands": [
    {"type": "move_xy", "x": 10.0, "y": 20.0},
    {"type": "trajectory", "label": "Execute trajectory"},
    {"type": "service_seq", "pump": "P1", "steps": ["waste", "wash", "buffer", "ink"]},
    {"type": "switch_pump", "pump": "P2", "ink_name": "Collagen"},
    {"type": "print_path", "points": [[0,0],[10,0]], "pump": "P1", "flow_rate": 0.01}
  ]
}
```

---

## 11. GUI Layout (v7.1)

```
┌────────────────────────────────────────────────────────────────────┐
│  MainWindow                                                        │
│ ┌────┬──────────┬────────────────────────────────────────────────┐ │
│ │ L  │ Context  │  Top Bar (page title + connection dots + ☰)    │ │
│ │ E  │ Panel    ├────────────────────────────────────────────────┤ │
│ │ F  │ (per-    │                                                │ │
│ │ T  │  page    │   Content Pages (QStackedWidget)               │ │
│ │    │  scroll) │     📊 Dashboard / 🕹️ Jog / 📐 Calibration     │ │
│ │ M  │          │     🖨️ Print Setup / 📈 Print Monitor ★v7.1    │ │
│ │ E  │          │     ⚙️ Settings                                │ │
│ │ N  │          │                                                │ │
│ │ U  │          │ ┌────────────────────────────────────────────┐ │ │
│ │    │          │ │ Console Log (collapsible via QSplitter)    │ │ │
│ │ ⚙  │          │ └────────────────────────────────────────────┘ │ │
│ └────┴──────────┘  Bottom Bar (XY pos, ZP pos, speed, safety)    │ │
└────────────────────────────────────────────────────────────────────┘
```

**Page indices**: Dashboard=0, Jog=1, Calibration=2, Print Setup=3, Settings=4, Print Monitor=5.

### Print Monitor Page (v7.1)

```
┌───────────────────────┬─────────────────────────────────────────┐
│  Full Plate Overview  │  Current Well Detail (trajectory view)   │
│  (mini plate, color-  │  XY/ZY/XZ projections                   │
│   coded per-well      │  Completed path ── Upcoming path ╌╌     │
│   progress)           │  Needle position ✛                      │
├───────────────────────┼─────────────────────────────────────────┤
│  Syringe Status       │  Print Progress                        │
│  [P1] [P2] [P3]      │  Job, Well, Layer, Step, Time, ETA     │
│  Fill bars + needle   │  [Pause] [Abort]  progress bar         │
│  info                 │  Tracking error + controller status    │
└───────────────────────┴─────────────────────────────────────────┘

Context Panel: Recording browser — list, load replay, delete, overlay on trajectory view
```

---

## 12. Logging Strategy

All modules use `logging.getLogger(__name__)`. No `print()` in backend code.

| Level | Usage | Examples |
|-------|-------|---------|
| `DEBUG` | High-frequency routine | Position polls, waypoint execution, sample recording |
| `INFO` | User-visible state changes | Stage connected, print started, recording saved |
| `WARNING` | Recoverable interventions | Safety limit clamped, ink change triggered, settle timeout |
| `ERROR` | Failures requiring attention | Serial disconnect, trajectory abort, recording failure |

---

## 13. Key Design Decisions

1. **StageController as single backend entry point**: GUI never touches serial ports.
2. **JSON protocol abstraction (v7.1)**: XY stage commands defined in data files, not code. Supports multiple controller models without code changes.
3. **Proven Xbox code preserved**: `XboxController.py` structurally identical to validated version.
4. **PrintRecorder as separate concern (v7.1)**: Recording is orthogonal to execution — PrintManager delegates to recorder but doesn't depend on it.
5. **Fluid state tracking (v7.1)**: FluidColumnTracker manages ink volumes declaratively; service sequences triggered automatically when ink changes detected.
6. **Trajectory-first execution (v7.1)**: New `TRAJECTORY` command type enables continuous motion control alongside legacy discrete commands.
7. **Safety dampening, not hard stops**: Proportional velocity reduction near limits.
8. **Thread → GUI bridge pattern**: All cross-thread updates via `QObject + Signal` bridges.
