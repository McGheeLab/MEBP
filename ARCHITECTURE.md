# MEBP Version 7.0 — System Architecture

## 1. Purpose

Desktop application for controlling a multi-axis 3D bioprinting platform. Drives an XY microscope stage (Prior ProScan III), a Z needle + 3 syringe pumps (Marlin 3D printer board), and an Xbox controller for manual jogging. Provides a PySide6 GUI for calibration, print job setup with well plate support, live visualization, and a print queue.

---

## 2. Hardware

| Device | Interface | Protocol | Notes |
|--------|-----------|----------|-------|
| **XY Stage** — Prior ProScan III | RS-232 serial (9600 baud) | ASCII commands: `VS` (velocity), `G`/`PA` (goto), `GR` (relative), `P` (position), `V` (version), `Z` (home) | Position units: microsteps. Connected at ~9600 baud, CR-terminated. |
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

---

## 3. Folder Structure

```
MEBP-Version-7.0/
├── main.py                              # Entry point — CLI arg parsing, mode selection
├── settings.json                        # Persistent app config (auto-saved)
├── current_button_mapping.json          # Xbox controller mapping (hot-reloadable)
│
├── SupportClasses/                      # Backend — zero GUI dependencies
│   ├── __init__.py                      # Package docstring + module listing
│   ├── Processor.py                     # Thread-safe command bus (pub/sub)
│   ├── SerialUtils.py                   # Retry decorator, safe I/O, port discovery, watchdog
│   ├── Simulators.py                    # XYStageSimulator + ZPStageSimulator
│   ├── XYStage.py                       # Prior ProScan III manager
│   ├── ZPStage.py                       # Marlin board manager (G-code)
│   ├── XboxController.py                # Polling worker (multiprocessing.Process)
│   ├── StageController.py               # Orchestrator: jog handlers, position poller, lifecycle
│   ├── PrintManager.py                  # Print execution engine + queue + file I/O
│   ├── WellPlate.py                     # ANSI/SLAS plate geometry + pattern generators
│   ├── SafetyLimits.py                  # Software endstops with clamping
│   ├── PositionLogger.py                # Timestamped position recording + CSV/JSON export
│   ├── Settings.py                      # JSON config with dot-path access
│   └── PrintHistory.py                  # Enhancement 6: Persistent print history log
│
├── gui/                                 # Frontend — PySide6 only
│   ├── __init__.py
│   ├── app.py                           # MainWindow: tabs, connection panel, status bar, timers
│   ├── styles.py                        # QSS dark theme stylesheet (Catppuccin Mocha)
│   ├── pages/
│   │   ├── __init__.py
│   │   ├── dashboard.py                 # Device status, positions, speeds, safety, position log export
│   │   ├── jog_control.py               # Manual jog: direction pad, speed sliders, keyboard shortcuts
│   │   ├── calibration.py               # Guided calibration: zero needle, teach plate, validate
│   │   ├── print_setup.py               # Job loading, 2D preview, well plate, execution, queue
│   │   └── settings_page.py             # Connection, safety, polling, Xbox, logging controls
│   └── widgets/
│       ├── __init__.py
│       ├── console_log.py               # Log viewer widget + QtLogHandler (thread-safe)
│       ├── xbox_mapping_editor.py       # Button/axis/dpad mapping editor dialog
│       └── camera_widget.py             # Enhancement 2: Live microscope camera feed
│
└── sample_jobs/
    └── multi_material_test.json         # Example multi-material print job

```

### Completion Status

| Layer | Status | Files |
|-------|--------|-------|
| **SupportClasses** (13 files, 3,385 lines) | ✅ Complete | All 13 files delivered |
| **gui/** (12 files, 3,683 lines) | ✅ Complete | All pages, widgets, and styles delivered |
| **main.py** | ✅ Written | Supports `--headless`, `--real-xy`, `--real-zp`, `--verbose` |
| **Config files** | ✅ Written | `current_button_mapping.json`, `sample_jobs/multi_material_test.json` |

---

## 4. Module Dependency Graph

```
main.py
  ├── SupportClasses.StageController   ← the single backend entry point
  │     ├── Processor
  │     ├── XYStage
  │     │     └── Simulators.XYStageSimulator  (sim mode)
  │     │     └── SerialUtils                  (real mode)
  │     ├── ZPStage
  │     │     └── Simulators.ZPStageSimulator  (sim mode)
  │     │     └── SerialUtils                  (real mode)
  │     ├── XboxController (multiprocessing.Process)
  │     ├── SafetyLimits
  │     ├── PositionLogger
  │     └── SerialUtils.ConnectionWatchdog
  │
  ├── SupportClasses.Settings          ← persistent config
  │
  └── gui.app.MainWindow               (GUI mode only)
        ├── gui.pages.dashboard        → StageController (read-only)
        ├── gui.pages.jog_control      → StageController (jog commands)
        ├── gui.pages.calibration      → StageController (calibration)
        ├── gui.pages.print_setup      → PrintManager, PrintQueue, WellPlate
        ├── gui.pages.settings_page    → StageController, SafetyLimits, Settings
        └── gui.widgets.*              → Console log, Xbox mapping editor
```

**Key rule: GUI pages never talk to serial ports directly. Everything goes through StageController or PrintManager.**

---

## 5. SupportClasses — Module Reference

### 5.1 Processor.py (89 lines)

Thread-safe command bus using `queue.Queue`. Components register handlers for named commands; when a command is added, the dispatch loop invokes the matching handler with kwargs. Single background thread processes commands sequentially.

```
┌──────────┐    add_command()     ┌────────────┐    dispatch     ┌──────────┐
│  Xbox     │ ──────────────────► │  Processor  │ ─────────────► │ Handler  │
│  GUI      │                     │  (Queue)    │                │ functions│
│  Print    │                     └────────────┘                └──────────┘
└──────────┘
```

Commands: `move_stage_at_velocity`, `move_z_at_velocity`, `move_p1_at_velocity`, `move_p2_at_velocity`, `move_p3_at_velocity`, `zero_needle_pos`, `increment_xyspeed_up/down`, `increment_zspeed_up/down`, `increment_pspeed_up/down`.

### 5.2 SerialUtils.py (234 lines)

Resilient serial communication layer:

- **Exception hierarchy**: `SerialError` → `SerialDisconnectedError`, `SerialTimeoutError` — with user-friendly messages.
- **`retry` decorator**: Wraps serial operations with configurable max_retries (default 2) and delay. Catches transient failures.
- **Safe wrappers**: `safe_serial_write()`, `safe_serial_readline()`, `safe_serial_read_all()` — all log on failure and raise `SerialError`.
- **`list_serial_ports()`**: Returns list of `{device, description, hwid}` dicts using `serial.tools.list_ports`.
- **`check_port_health()`**: Tests if a serial port is still connected (reads `in_waiting` to trigger OS-level check).
- **`ConnectionWatchdog`**: Background thread that monitors ports every N seconds and fires callbacks on disconnect.

### 5.3 Simulators.py (369 lines)

Software-only stage simulators for development without hardware. Both expose serial-like interfaces so stage managers can use them as drop-in replacements.

**XYStageSimulator** — Prior ProScan III emulation:
- Parses ASCII commands: `VS`, `G`/`PA`, `GR`, `P`, `V`, `Z`, `S`
- Physics: 100 Hz proportional controller for absolute moves, direct velocity integration for `VS` moves
- Status tracking: reports `0` (idle) or `1..3` (moving) via `S` command
- Interface: `send_command(cmd) → response`, `is_open`, `in_waiting`

**ZPStageSimulator** — Marlin G-code emulation:
- Parses G-code: `G0`, `G90`/`G91`, `M114`, `M302`, `M83`, `M92`, `M203`, `M220`, `M500`, `M503`, `M112`, `M115`
- 4-axis physics: X(Z), Y(P1), Z(P2), E(P3) — matches the real axis mapping
- 100 Hz proportional controller with configurable feed rates
- Interface: `write(data)`, `flush()`, `read_all()`, `readline()`, `reset_input_buffer()`, `reset_output_buffer()`, `is_open`, `in_waiting`

### 5.4 XYStage.py (204 lines)

Manages connection and communication with the Prior ProScan III.

- **Hardware mode**: Scans COM ports, sends `V` (version query) to identify the correct port, opens connection.
- **Simulation mode**: Creates `XYStageSimulator` instance and delegates all commands.
- **Methods**: `connect()`, `disconnect()`, `set_velocity(vx, vy)`, `move_absolute(x, y)`, `move_relative(dx, dy)`, `get_position() → (x, y, status)`, `home()`
- **Error handling**: All serial operations wrapped with retry logic; disconnects logged at ERROR level.

### 5.5 ZPStage.py (262 lines)

Manages connection and communication with the Marlin 3D printer board.

- **Hardware mode**: Scans COM ports at 115200 baud, sends `M115` to identify Marlin firmware, initializes with `M302 S0` (cold extrude enable), `M83` (relative extrude).
- **Simulation mode**: Creates `ZPStageSimulator` instance.
- **Axis mapping**: All methods accept logical names (`Z`, `P1`, `P2`, `P3`) and translate via `AXIS_MAP`.
- **Methods**: `connect()`, `disconnect()`, `move_absolute(axes_dict, feedrate)`, `move_relative(axes_dict, feedrate)`, `get_position() → (Z, P1, P2, P3)`, `home(axes)`, `emergency_stop()`, `set_steps_per_mm(axis, value)`, `set_max_feedrate(axis, value)`, `save_settings()`
- **Position parsing**: Parses `M114` response like `X:1.00 Y:2.00 Z:3.00 E:4.00` into logical axis dict.

### 5.6 XboxController.py (182 lines)

Proven polling worker running in a **separate process** via `multiprocessing.Process`. Communicates with the main process through a `multiprocessing.Queue`.

- **`xbox_polling_worker(queue, mapping_file, stop_event)`**: The process entry point.
- Uses `pygame` for joystick input at ~100 Hz.
- Button mapping loaded from JSON file, hot-reloaded every 5 seconds.
- Axis values averaged over configurable window (default 5 samples) with deadzone (default 0.15).
- Queue messages: `{"command": str, "button": int}`, `{"command": str, "axis": str, "average": float}`, `{"command": str, "dpad": str}`, `{"debug": str}`.

**This code is proven from the XBOXCONTROLLED version and should not be structurally changed.**

### 5.7 StageController.py (605 lines)

The top-level orchestrator that GUI pages and PrintManager interact with. Contains:

**XboxQueuePoller** — Thread that reads the multiprocessing queue and dispatches to Processor.

**XYJogHandler** — Continuous velocity jogging for XY stage:
- Receives `move_stage_at_velocity` commands with axis averages from Xbox.
- Converts stick deflection to velocity command (`VS vx,vy`).
- **Safety dampening**: Near boundaries, scales velocity proportionally to remaining distance (boundary margin = 5000 steps).
- Runs in its own daemon thread, updates every 50ms.

**ZPJogHandler** — Segmented relative moves for ZP stage:
- Receives `move_z_at_velocity`, `move_p1/p2/p3_at_velocity` commands.
- Converts stick deflection to relative G-code move (`G0 Xn Fn`).
- **Safety clamping**: Clamps target positions to safety limits before sending.
- Runs in its own daemon thread, updates every 100ms.

**PositionPoller** — Background thread caching positions at 300ms intervals:
- Queries XY and ZP positions and stores in thread-safe cache.
- UI reads `get_xy_position(cached=True)` which returns instantly from cache.
- PrintManager uses `cached=False` for direct serial queries during execution.
- Automatically starts/stops with stage connections.

**StageController** — The orchestrator:
- `connect_stages()` / `disconnect_xy()` / `disconnect_zp()` — Hardware lifecycle
- `connect_xbox(mapping_file)` / `disconnect_xbox()` — Xbox process management
- `move_xy_absolute(x, y, from_zero_ref)` — Safety-limited XY movement
- `move_z_absolute(z, from_zero_ref)` — Safety-limited Z movement
- `move_z_relative(dist, feedrate)` — Relative Z
- `move_pump_relative(pump, amount, feedrate)` — Pump extrusion
- `zero_all()` — Sets current position as zero reference, logs to position logger
- `get_xy_position(cached)` / `get_zp_position(cached)` — Position reading
- `get_speed_info()` — Current speed multipliers
- `shutdown()` — Clean teardown of all threads and processes

Properties: `is_xy_connected`, `is_zp_connected`, `simulate_xy`, `simulate_zp`, `zero_position`, `safety_limits`, `position_logger`.

### 5.8 PrintManager.py (690 lines)

Print job execution engine with file I/O, job building, and queue management.

**Data structures**:
- `PrintState` enum: IDLE, RUNNING, PAUSED, COMPLETED, ABORTED, ERROR
- `CommandType` enum: MOVE_XY, MOVE_Z, MOVE_Z_REL, EXTRUDE, PRINT_PATH, DWELL, TRAVEL_UP, TRAVEL_DOWN, SET_PUMP_RATE, HOME_XY, COMMENT, SWITCH_PUMP
- `PrintCommand`: type + params dict + label
- `PrintSettings`: All print parameters including per-pump retract/prime amounts
- `PrintJob`: name, description, settings, command list; methods for path extraction and layer parsing

**File loading**:
- `load_print_file(path)`: Auto-detects .json or .gcode format
- JSON format: Custom schema with settings + command list
- G-code format: Parses G0/G1/G4/G28, ignores temperature and fan commands

**Job building**:
- `build_well_plate_job(wells, path, settings, ...)`: Generates a complete job from well plate positions + pattern.
- **Multi-material support**: `pump_sequence` (cycle pumps per-well) or `pump_per_layer` (assign pump per layer).
- Each well: travel up → move to well → settle → lower to print → prime → print path → retract.

**PrintManager class**:
- Thread-based executor with pause/resume/abort.
- `start()` spawns a daemon thread running `_execute_loop()`.
- Each `PrintCommand` dispatched to `_execute_cmd()` which calls appropriate `StageController` methods.
- **SWITCH_PUMP handling**: Retracts old pump, updates active pump, primes new pump.
- **Print path execution**: Coordinated XY movement + pump extrusion, segment by segment.
- **XY settle wait**: Polls position until within tolerance (50 microsteps) or timeout.
- Progress/state callbacks: `on_progress(step, total, message)`, `on_state_changed(state)`.
- Position logging at print start/end/error and every 10 commands.
- Abort raises Z to travel height as safety measure.

**PrintQueue class**:
- Sequential multi-job executor.
- `add_job()`, `remove_job()`, `reorder()`, `clear()`, `start_all()`, `abort()`, `abort_current()`, `pause()`, `resume()`.
- Aggregate progress via `get_total_progress() → (done, total)`.
- Stops queue on job error.
- Callbacks: `on_progress`, `on_queue_progress`, `on_state_changed`, `on_queue_completed`.

**Utility**: `save_print_job(job, filepath)` — Serializes to JSON.

### 5.9 WellPlate.py (351 lines)

ANSI/SLAS well plate geometry with pattern generators.

- **`WellPlate.from_format(n)`**: Creates plate for standard formats (6, 12, 24, 48, 96 wells).
- **`PLATE_DEFINITIONS`**: Dict of {format: {rows, cols, spacing, diameter, ...}} per ANSI/SLAS-2004.
- **Well access**: `get_well_position(name)`, `get_all_wells()`, `get_wells_in_rows(rows)`, `get_wells_in_range(start, end)`.
- **Pattern generators**: `generate_line_path(length, angle)`, `generate_meander_path(w, h, spacing)`, `generate_spiral_path(radius, spacing)`, `generate_grid_path(w, h, sx, sy)`.
- All patterns return points relative to well center (0,0).

### 5.10 SafetyLimits.py (135 lines)

Software endstops for all axes.

- **Attributes**: `enabled`, `xy_min_x/max_x`, `xy_min_y/max_y`, `z_min/max`, `p1_min/max`, `p2_min/max`, `p3_min/max`, `max_z_feedrate`, `max_pump_feedrate`, `boundary_margin` (5000 steps for XY).
- **`clamp_xy(x, y) → (x, y, was_clamped)`**: Clamps to XY bounds, logs warning if clamped.
- **`clamp_z(z) → (z, was_clamped)`**: Clamps to Z bounds.
- **`clamp_pump(pump, value) → (value, was_clamped)`**: Clamps to pump bounds.
- **`is_near_limit_xy(x, y) → bool`**: True if within `boundary_margin` of any XY limit.
- **Serialization**: `to_dict()` / `from_dict()` for settings persistence.

### 5.11 PositionLogger.py (124 lines)

Timestamped position recording for diagnostics and print replay.

- **`PositionRecord`**: dataclass with `timestamp`, `event`, `xy_pos`, `zp_pos`, `metadata`.
- **`record(event, xy_pos, zp_pos, metadata)`**: Appends a new record.
- **`save_csv(filepath)`** / **`save_json(filepath)`**: Export all records.
- **`load_json(filepath) → list[PositionRecord]`**: Import for replay.
- **`clear()`** / **`count`** / **`records`**: Management.
- **`generate_filename(prefix) → str`**: ISO-timestamped filename.

### 5.12 Settings.py (119 lines)

JSON-based persistent configuration with dot-path access.

- **Dot-path get/set**: `settings.get("window.width")`, `settings.set("speeds.xy", 500)`.
- **`get_section(key)`**: Returns entire sub-dict.
- **`set_section(key, dict)`**: Sets entire sub-dict.
- **Deep merge on load**: Missing keys fall back to `DEFAULTS` dict.
- **`save()` / `load()`**: JSON file I/O with error logging.

Default sections: `window`, `simulation`, `speeds`, `zero_position`, `safety_limits`, `polling`, `xbox`, `logging`.

---

## 6. GUI Layer

### 6.1 Architecture

```
MainWindow (app.py)
├── Connection Panel (always visible at top)
│   ├── XY Stage: [Connect] [Disconnect] + status label
│   ├── ZP Stage: [Connect] [Disconnect] + status label
│   ├── Xbox:     [Connect] [Disconnect] + status label
│   └── [Edit Mapping] button
│
├── QTabWidget
│   ├── 📊 Dashboard       → DashboardPage
│   ├── 🕹️ Jog Control     → JogControlPage (NOT YET WRITTEN)
│   ├── 📐 Calibration     → CalibrationPage (NOT YET WRITTEN)
│   ├── 🖨️ Print Setup     → PrintSetupPage
│   └── ⚙️ Settings        → SettingsPage
│
├── Console Log (collapsible, bottom splitter)
│   └── ConsoleLogWidget + QtLogHandler (NOT YET WRITTEN)
│
└── Status Bar
    ├── XY position readout
    ├── ZP position readout (Z + P1 + P2 + P3)
    ├── Speed multipliers
    ├── Safety limits indicator (🛡️ ON/OFF)
    └── Position log entry count
```

### 6.2 GUI ↔ Backend Communication

**Timer-based polling** (500ms): Main window timer calls `update_data()` on the active tab page. Pages read cached positions via `controller.get_xy_position(cached=True)` — never blocking on serial I/O.

**Thread → GUI signals**: Print execution runs in background threads. A `PrintSignalBridge(QObject)` with Qt Signals bridges thread callbacks to GUI-safe slots. Similarly, `_DisconnectBridge` bridges watchdog disconnect events.

**Settings persistence**: `MainWindow.save_settings()` stores window geometry, active tab, splitter sizes, simulation flags, speed multipliers, zero reference, and safety limits. Called on window close and app quit.

### 6.3 Page Descriptions

**DashboardPage** (`dashboard.py`, 247 lines): Read-only overview showing XY position, ZP position, speed multipliers, zero reference, safety limits status, and position log count. Includes CSV/JSON export buttons for position log.

**PrintSetupPage** (`print_setup.py`, 957 lines): The most complex page.
- **Source tabs**: Load from file / Configure well plate + pattern
- **2D canvas** (`PathPreviewCanvas`): QPainter widget with auto-scaling, grid, origin marker, well outlines, dual-color paths (completed=bright green, remaining=dim), pulsing crosshair for current position during printing, layer filtering.
- **Settings panel**: Print feedrates, Z heights, layer count, pump selection, flow rate, retract/prime amounts.
- **Multi-material controls**: Pump sequence (per-well cycling) or pump-per-layer assignment.
- **Execution controls**: Start/Pause/Resume/Abort with progress bar.
- **Queue panel**: Job list with add/remove/reorder, start queue, abort queue.
- **Live update**: During printing, reads cached position and updates canvas crosshair.

**SettingsPage** (`settings_page.py`, 422 lines): Scrollable settings editor.
- **Connection**: Simulation mode checkboxes, serial port list with refresh.
- **Safety limits**: Per-axis min/max spinboxes, "Set from Current" buttons, feedrate limits.
- **Polling**: Position poll interval (ms), watchdog interval (s).
- **Xbox**: Mapping file path with browse.
- **Logging**: Verbose checkbox (applies immediately).
- **Apply / Reset to Defaults** buttons.

**XboxMappingEditor** (`xbox_mapping_editor.py`, 352 lines): Modal dialog.
- Tabbed: Buttons (12), Axes (4 groups), D-Pad (4 directions).
- Dropdown command selectors for all available commands.
- Human-readable input labels.
- Import/Export/Reset to Defaults/Save.

### 6.4 GUI File Summary

| File | Lines | Purpose |
|------|-------|---------|
| `gui/styles.py` | 385 | QSS dark theme (Catppuccin Mocha palette) |
| `gui/app.py` | 404 | MainWindow with tabs, connection panel, status bar |
| `gui/pages/dashboard.py` | 246 | Device status, position readouts, position log export |
| `gui/pages/jog_control.py` | 363 | Direction pad, step sizes, speed sliders, keyboard shortcuts, E-stop |
| `gui/pages/calibration.py` | 383 | 3-step wizard: zero needle, teach plate (A1 + corner), validate alignment |
| `gui/pages/print_setup.py` | 956 | Job loading, 2D canvas, well plate config, print execution, queue |
| `gui/pages/settings_page.py` | 421 | Safety limits, polling, Xbox mapping, simulation mode, logging |
| `gui/widgets/console_log.py` | 171 | ConsoleLogWidget + QtLogHandler with thread-safe signal bridge |
| `gui/widgets/xbox_mapping_editor.py` | 351 | Modal editor for button/axis/dpad → command mappings |

---

## 7. Threading Model

```
┌─ Main Thread (GUI event loop) ─────────────────────────────────────────────┐
│  QTimer (500ms) → update_data() → read cached positions                    │
│  Button clicks → controller.connect_stages() / print_manager.start() etc.  │
└────────────────────────────────────────────────────────────────────────────┘

┌─ Processor Thread ─────────────────────────────────────────────────────────┐
│  Blocks on queue.Queue.get(), dispatches to registered handlers            │
│  Handlers call jog handler update methods                                  │
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

┌─ PrintExec Thread (spawned per print job) ─────────────────────────────────┐
│  Sequential command execution with pause/abort event waits                 │
│  Reports progress via callbacks bridged to GUI with Qt Signals             │
└────────────────────────────────────────────────────────────────────────────┘

┌─ PrintQueue Thread (spawned when queue starts) ────────────────────────────┐
│  Iterates queue, loads each job into PrintManager, waits for completion    │
└────────────────────────────────────────────────────────────────────────────┘
```

**Total active threads during printing with Xbox**: 8 threads + 1 separate process.

**Thread safety**:
- Processor: `queue.Queue` (inherently thread-safe).
- PositionPoller: `threading.Lock` protects the position cache.
- PrintManager: `threading.Event` for pause/abort signaling.
- Jog handlers: Atomic float writes for stick values; worst case is a stale read (harmless for jogging).
- GUI updates: All Qt widget updates happen on the main thread via timer or signal bridge.

---

## 8. Data Flow

### 8.1 Xbox Jogging Flow

```
Xbox Controller (separate process)
  │  pygame poll → button/axis event
  │  multiprocessing.Queue.put({"command": "move_stage_at_velocity", "axis": "0-1", "average": 0.8})
  ▼
XboxQueuePoller (thread)
  │  queue.get_nowait() → Processor.add_command("move_stage_at_velocity", axis="0-1", average=0.8)
  ▼
Processor (thread)
  │  dispatch → XYJogHandler.update(axis="0-1", value=0.8)
  ▼
XYJogHandler (thread, 50ms loop)
  │  stick_value → velocity = value × max_speed × safety_dampen_factor
  │  XYStage.set_velocity(vx, vy)
  ▼
XY Stage (serial)
  │  "VS vx,vy\r" → stage moves
```

### 8.2 Print Execution Flow

```
GUI: user clicks Start
  │  PrintManager.start()
  ▼
PrintExec Thread spawns
  │  for each PrintCommand:
  │    check abort_flag
  │    wait on pause_event
  │    _execute_cmd(cmd)
  │      ├── MOVE_XY → controller.move_xy_absolute() → XYStage.move_absolute()
  │      ├── MOVE_Z  → controller.move_z_absolute()  → ZPStage.move_absolute()
  │      ├── EXTRUDE → controller.move_pump_relative() → ZPStage.move_relative()
  │      ├── PRINT_PATH → segment-by-segment XY + pump extrusion
  │      ├── SWITCH_PUMP → retract old, switch, prime new
  │      ├── DWELL → time.sleep() with abort checks
  │      └── etc.
  │    report progress via callback → PrintSignalBridge → GUI update
  ▼
GUI: progress_bar.setValue(), canvas.set_progress()
```

### 8.3 Position Reading Flow

```
PositionPoller (300ms loop)
  │  XYStage.get_position() → (x, y, status)    [serial I/O]
  │  ZPStage.get_position() → (Z, P1, P2, P3)   [serial I/O]
  │  lock.acquire() → cache.update() → lock.release()
  ▼
GUI Timer (500ms)
  │  controller.get_xy_position(cached=True)
  │  lock.acquire() → read cache → lock.release()
  │  update status bar labels
```

---

## 9. Safety Architecture

### 9.1 Layers

1. **SafetyLimits clamping** (software): All `StageController.move_*` methods check limits and clamp before sending commands. Logged at WARNING level.
2. **Jog handler dampening** (software): `XYJogHandler` scales velocity near boundaries. `ZPJogHandler` clamps target positions.
3. **Abort Z-raise** (software): `PrintManager.abort()` raises Z to travel height after aborting.
4. **Marlin endstops** (firmware): Physical endstop switches on the 3D printer board provide last-resort protection.
5. **Emergency stop**: `ZPStage.emergency_stop()` sends `M112` to Marlin firmware.

### 9.2 Safety Limits Persistence

```
SettingsPage → user edits limits → Apply
  │  controller.safety_limits attributes updated
  │  settings.set_section("safety_limits", sl.to_dict())
  │  settings.save() → settings.json
  ▼
App restart
  │  Settings.load() → settings.json
  │  MainWindow.__init__() → SafetyLimits.from_dict(saved) → controller.safety_limits
```

---

## 10. Configuration Files

### 10.1 settings.json

```json
{
  "window": {"x": 100, "y": 100, "width": 1200, "height": 800, "active_tab": 0},
  "simulation": {"simulate_xy": true, "simulate_zp": true},
  "speeds": {"xy": 1000, "z": 5.0, "p": 3.0},
  "zero_position": {"x": 0, "y": 0, "Z": 0, "P1": 0, "P2": 0, "P3": 0},
  "safety_limits": {"enabled": true, "xy_min_x": -100000, ...},
  "polling": {"position_interval_ms": 300, "watchdog_interval_s": 3.0},
  "xbox": {"mapping_file": "current_button_mapping.json"},
  "logging": {"verbose": false}
}
```

### 10.2 current_button_mapping.json

```json
{
  "buttons": {"0": "zero_needle_pos", "4": "increment_zspeed_down", ...},
  "axes": {"0-1": "move_stage_at_velocity", "2-3": "move_z_at_velocity", ...},
  "dpad": {"up": "increment_zspeed_up", "down": "increment_zspeed_down", ...}
}
```

### 10.3 Print Job JSON

```json
{
  "name": "My Print",
  "settings": {"xy_feedrate": 1000, "z_feedrate": 60, "num_layers": 3, ...},
  "commands": [
    {"type": "move_xy", "x": 10.0, "y": 20.0},
    {"type": "print_path", "points": [[0,0],[10,0]], "pump": "P1", "flow_rate": 0.01},
    {"type": "switch_pump", "pump": "P2"},
    ...
  ]
}
```

---

## 11. Logging Strategy

All modules use `logging.getLogger(__name__)`. No `print()` statements in backend code.

| Level | Usage | Examples |
|-------|-------|---------|
| `DEBUG` | High-frequency routine operations | Position polls, jog stick values, command dispatch |
| `INFO` | User-visible state changes | Stage connected, calibration set, speed changed, print started |
| `WARNING` | Recoverable interventions | Safety limit clamped, near-limit detected, settle timeout |
| `ERROR` | Failures requiring attention | Serial disconnect, command parse error, print execution error |
| `EXCEPTION` | Unexpected errors with traceback | Unhandled exceptions in threads |

GUI routes all Python logging to a `ConsoleLogWidget` via `QtLogHandler` with color coding: debug=gray, info=white, warning=yellow, error=red.

---

## 12. Key Design Decisions

1. **StageController as single backend entry point**: GUI pages never touch serial ports. This allows swapping simulators/hardware transparently and ensures thread safety.

2. **Simulators in separate file**: `Simulators.py` (369 lines) keeps physics simulation code isolated from hardware communication code. Stage managers import simulators only in sim mode.

3. **Proven Xbox code preserved**: `XboxController.py` is structurally identical to the validated `XboxControl_XBOXCONTROLLED.py`. Jog handler logic in `StageController.py` matches the proven `ProcessCommand_XBOXCONTROLLED.py` behavior.

4. **Position caching via PositionPoller**: GUI reads cached positions (non-blocking), avoiding serial I/O on the main thread. Print execution uses `cached=False` for accurate real-time positioning.

5. **Per-pump retract/prime**: `PrintSettings` stores per-pump amounts (`retract_amounts`, `prime_amounts` dicts) with fallback to global values. Enables multi-material printing with different syringe properties.

6. **Safety dampening, not hard stops**: `XYJogHandler` proportionally reduces velocity near limits rather than hard-stopping, giving the operator smooth deceleration feel.

7. **Thread → GUI bridge pattern**: All cross-thread GUI updates go through `QObject + Signal` bridges (`PrintSignalBridge`, `_DisconnectBridge`). No direct widget manipulation from background threads.

8. **Print queue stops on error**: If any job in the queue fails, the entire queue stops. This prevents cascading failures in bioprinting where a failed print could damage subsequent prints.

---

## 13. Remaining Work — Task List for Next Sessions

### API Compatibility Verification

All GUI files were verified against the rewritten SupportClasses APIs. One fix was applied: `emergency_stop()` alias added to `ZPStage.py` (jog control page calls it). All other method signatures match.

Key API surface verified:
- `StageController`: `connect_stages()`, `disconnect_xy/zp()`, `connect_xbox()`, `move_xy_absolute()`, `move_z_absolute()`, `move_z_relative()`, `move_pump_relative()`, `get_xy_position()`, `get_zp_position()`, `get_speed_info()`, `_calibrate_zero()`, `shutdown()`, `is_xy_connected`, `is_zp_connected`, `zero_position`, `safety_limits`, `position_logger`, `xy_jog`, `zp_jog`, `_pos_poller`
- `XYJogHandler`: `speed` property, `xy_speed` attribute
- `ZPJogHandler`: `speeds` property, `z_speed`/`p_speed` attributes
- `SafetyLimits`: `clamp_xy()`, `clamp_z()`, `clamp_pump()`, `to_dict()`, `from_dict()`, `enabled`, all min/max attributes
- `PositionLogger`: `count`, `clear()`, `save_csv()`, `save_json()`, `generate_filename()`
- `ZPStage`: `emergency_stop()` (alias for `reset_printer()`)

### Integration Testing

- Smoke test: run `python main.py` in simulation mode — verify all tabs load
- Xbox: connect simulated stages, verify jog commands flow through Processor
- Print execution: load a sample job, start/pause/abort cycle
- Calibration: run through 3-step wizard with simulated positions
- Settings: modify safety limits, save, restart, verify persistence

### Potential Enhancements

1. **G-code export** ✅: `export_gcode()` function in `PrintManager.py` — converts `PrintJob` commands to standard G-code (G0/G1/G4/G28) with proper header, initialization, per-segment extrusion, and `SWITCH_PUMP` translation. Export button added to `PrintSetupPage` execution panel.

2. **Camera integration** ✅: `gui/widgets/camera_widget.py` — Live microscope camera feed widget using OpenCV (`cv2`). Features: auto-detect cameras, live feed with configurable FPS, crosshair overlay for needle alignment, snapshot capture. Embedded in `CalibrationPage` step 3 area. Falls back gracefully if OpenCV not installed.

3. **Print resume** ✅: `save_print_progress()` / `load_print_progress()` / `clear_print_progress()` in `PrintManager.py`. Progress saved to `print_resume.json` every 20 commands and on pause/abort/error. On app startup, `MainWindow._check_print_resume()` offers to resume. `PrintManager.resume_from_saved()` and `PrintSetupPage.resume_print()` handle the resume flow.

4. **Keyboard jogging in any tab** ✅: `MainWindow.keyPressEvent()` in `app.py` provides global keyboard shortcuts: Arrow keys (XY jog), PageUp/PageDown (Z jog), Home (go to zero), Escape (emergency stop). Text input widgets are excluded from interception.

5. **Calibration persistence** ✅: Calibration data (taught A1/corner positions, scale, rotation, offset) saved to `settings.json` under the `"calibration"` section. Save/Load buttons added to `CalibrationPage`. Auto-loaded on page initialization. `Settings.py` updated with default calibration section.

6. **Print history log** ✅: `SupportClasses/PrintHistory.py` — persistent log (`print_history.json`) with `PrintHistoryEntry` dataclass. Records job name, state, duration, commands, pumps used, errors. `PrintManager` auto-records on completion/abort/error. `DashboardPage` shows live stats (total, completed, success rate, print time). Export to CSV supported. Rolling 500-entry limit.

---

