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
│   └── PrintHistory.py                  # Persistent print history log
│
├── gui/                                 # Frontend — PySide6, PyDracula-style layout
│   ├── __init__.py
│   ├── app.py                           # MainWindow: sidebar nav, context panel, top/bottom bars
│   ├── styles.py                        # QSS dark theme (Catppuccin Mocha) + COLORS dict
│   ├── ui_functions.py                  # Animation helpers: toggleMenu, toggleLeftBox, selectMenu
│   ├── pages/
│   │   ├── __init__.py
│   │   ├── dashboard.py                 # Device status, positions, speeds, safety, history
│   │   ├── jog_control.py               # Manual jog: dpad, step sizes, speed sliders
│   │   ├── calibration.py               # Guided 3-step calibration with camera feed
│   │   ├── print_setup.py               # Job loading, 2D preview, execution, queue
│   │   └── settings_page.py             # Connection, safety, polling, Xbox, logging
│   ├── widgets/
│   │   ├── __init__.py
│   │   ├── console_log.py               # Log viewer + QtLogHandler (thread-safe)
│   │   ├── xbox_mapping_editor.py       # Button/axis/dpad mapping editor dialog
│   │   └── camera_widget.py             # Live microscope camera feed (OpenCV)
│   └── tests/
│       ├── __init__.py
│       └── test_integration.py          # Headless integration test (all pages + shell)
│
└── sample_jobs/
    └── multi_material_test.json         # Example multi-material print job

```

### Completion Status

| Layer | Status | Files |
|-------|--------|-------|
| **SupportClasses** (13 files, 3,385 lines) | ✅ Complete | All 13 files delivered |
| **gui/** (17 files, 6,453 lines) | ✅ Complete | PyDracula layout, all pages, widgets, tests |
| **main.py** (148 lines) | ✅ Written | Supports `--headless`, `--real-xy`, `--real-zp`, `--verbose` |
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
        ├── gui.styles                 → DARK_THEME QSS, COLORS dict
        ├── gui.ui_functions           → Animation helpers (toggleMenu, toggleLeftBox)
        ├── gui.pages.dashboard        → StageController (read-only), PrintHistory
        ├── gui.pages.jog_control      → StageController (jog commands)
        ├── gui.pages.calibration      → StageController (calibration), CameraWidget
        ├── gui.pages.print_setup      → PrintManager, PrintQueue, WellPlate
        ├── gui.pages.settings_page    → StageController, SafetyLimits, Settings
        └── gui.widgets.*              → Console log, Xbox mapping editor, Camera
```

**Key rule: GUI pages never talk to serial ports directly. Everything goes through StageController or PrintManager.**

**Key rule: Pages communicate with MainWindow via `self.window()` — never by storing a direct reference.**

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

### 6.1 PyDracula Layout Architecture

The GUI uses a PyDracula-inspired layout (adapted from Wanderson M. Pimenta's framework) with a Catppuccin Mocha color palette. Instead of tabs, pages are selected via an icon sidebar with animated transitions.

```
┌────────────────────────────────────────────────────────────────────┐
│  MainWindow                                                        │
│ ┌────┬──────────┬────────────────────────────────────────────────┐ │
│ │ L  │ Context  │  Top Bar (page title + connection dots + ☰)    │ │
│ │ E  │ Panel    ├────────────────────────────────────────────────┤ │
│ │ F  │ (extra   │                                                │ │
│ │ T  │  left    │   Content Pages (QStackedWidget)               │ │
│ │    │  box)    │     📊 Dashboard / 🕹️ Jog / 📐 Calibration     │ │
│ │ M  │          │     🖨️ Print Setup / ⚙️ Settings                │ │
│ │ E  │ Scrollable│                                               │ │
│ │ N  │ per-page │ ┌────────────────────────────────────────────┐ │ │
│ │ U  │ settings │ │ Console Log (collapsible via QSplitter)    │ │ │
│ │    │          │ └────────────────────────────────────────────┘ │ │
│ │ ⚙  │          ├────────────────────────────────────────────────┤ │
│ └────┴──────────┘  Bottom Bar (XY pos, ZP pos, speed, safety)    │ │
│                   ──────────────────────────────────────────────── │
└────────────────────────────────────────────────────────────────────┘
```

**Left Menu** (60px collapsed / 200px expanded):
- Icon buttons for each page (emoji + tooltip)
- Toggle button (≡) for expand/collapse with slide animation
- Active page indicator: 3px mauve left border + subtle background tint
- Settings button pinned to bottom

**Context Panel** (0px hidden / 260px open):
- Each page provides its own context widget via `get_context_widget()`
- Context widgets are stacked in a `QStackedWidget`, switched on page navigation
- Wrapped in a `QScrollArea` for overflow on small screens
- Mauve header bar with close button (✕)
- Auto-opens when navigating to a page that has context content

**Content Area**:
- Top bar: page title + 3 connection dots (XY/ZP/Xbox) + context toggle (☰)
- Main area: `QStackedWidget` with 5 page widgets
- Console: `ConsoleLogWidget` in collapsible `QSplitter`
- Bottom status bar: position readouts, speed, safety indicator, log count

### 6.2 Page Interface Contract

Every page widget must implement these methods:

```python
def get_page_title(self) -> str:
    """Return display title for the top bar."""

def get_context_widget(self) -> QWidget | None:
    """Return a QWidget for the context panel. Must cache the widget —
    this is called once during page creation, and the same object is
    returned on subsequent calls."""

def on_status_update(self):
    """Called by MainWindow's periodic timer (~300ms) when this page
    is the active page. Use for updating position readouts, connection
    statuses, and live data."""
```

Optional:
```python
def update_data(self):
    """Alias for on_status_update(), used internally by some pages."""

def resume_print(self, resume_data: dict):
    """PrintSetupPage only — restore an interrupted print."""
```

### 6.3 GUI ↔ Backend Communication

**Timer-based polling** (300ms): Main window timer calls `on_status_update()` on the active page. Pages read cached positions via `controller.get_xy_position(cached=True)` — never blocking on serial I/O.

**Thread → GUI signals**: Print execution runs in background threads. A `PrintSignalBridge(QObject)` with Qt Signals bridges thread callbacks to GUI-safe slots. Similarly, `_DisconnectBridge` bridges watchdog disconnect events.

**Page → MainWindow delegation**: Pages access MainWindow methods via `self.window()` (Qt's parent traversal). Dashboard connection buttons call `self.window().connect_xy()` etc. Pages never store a direct reference to MainWindow.

**Settings persistence**: `MainWindow.save_settings()` stores window geometry, active tab, splitter sizes, simulation flags, speed multipliers, zero reference, and safety limits. Called on window close.

### 6.4 Page Descriptions

**DashboardPage** (`dashboard.py`, 533 lines): Read-only overview.
- **Main content**: XY position card, ZP position card (Z + P1–P3), speed readouts, zero reference, safety limits display, print history stats.
- **Context panel**: Connection controls (Connect/Disconnect for XY, ZP, Xbox), Xbox mapping editor launcher, position log count.
- Position log CSV/JSON export and history export/clear buttons.

**JogControlPage** (`jog_control.py`, 402 lines): Manual movement control.
- **Main content**: XY direction pad (arrow buttons), Z up/down buttons, pump extrude/retract, step size selectors (XY: 10–10000 steps, Z/P: 0.01–5.0 mm).
- **Context panel**: Speed multiplier sliders (XY, Z, Pump), quick actions (Home All, Zero All, E-Stop).
- Keyboard shortcuts: arrow keys (XY), PgUp/PgDn (Z), Home (go to zero), Escape (E-stop).

**CalibrationPage** (`calibration.py`, 693 lines): Guided 3-step calibration wizard.
- **Main content**: Step 1 (Zero Needle), Step 2 (Teach Plate — A1 + diagonal corner), Step 3 (Validate — camera feed + computed positions). Progress indicator.
- **Context panel**: Camera source selector, brightness/gamma sliders, FPS control, crosshair toggle, start/stop camera, snapshot. Plate format selector, calibration save/load buttons, computed scale/rotation readouts.
- Camera widget is lazily created; falls back gracefully if OpenCV not installed.

**PrintSetupPage** (`print_setup.py`, 1,059 lines): The most complex page.
- **Main content**: Source tabs (File browse / Well Plate config / Pattern generator), layer selector, `PathPreviewCanvas` (auto-scaling 2D preview with dual-color paths, well overlays, current position marker, grid).
- **Context panel**: Print settings (travel/print Z, layers, speed, feedrates, pump selector, multi-material mode, retraction/prime, settle delay), execution controls (Start/Pause/Abort, G-code export, JSON save, progress bar), print queue (job list with drag-reorder, add/remove/clear, start queue).
- **Multi-material**: Per-layer or per-well pump assignment via configurable pump sequence.
- **Live update**: During printing, reads cached position and updates canvas crosshair.
- **Print resume**: `resume_print(resume_data)` restores an interrupted print from saved progress.

**SettingsPage** (`settings_page.py`, 667 lines): System configuration.
- **Main content**: Simulation mode checkboxes (XY/ZP with restart warning), serial port list, safety limits (per-axis min/max spinboxes with "Set from Current" buttons, feedrate limits), polling intervals, Xbox mapping file path, verbose logging toggle.
- **Context panel**: Quick safety enable/disable (synced bidirectionally with main content), simulation status indicators (color-coded), serial port list + refresh, Apply/Reset buttons with status feedback.
- Apply writes to controller and settings.json; simulation flag changes require restart.

### 6.5 Widgets

**ConsoleLogWidget** (`console_log.py`, 171 lines): Read-only log viewer.
- Auto-scroll toggle, clear button, color-coded by severity.
- `log(message, tag)` for direct messages; `log_record(record, formatted)` for Python logging integration.
- Thread-safe via `_LogSignalBridge(QObject)` with Qt Signal.
- `QtLogHandler(logging.Handler)` routes Python `logging` to the widget.
- Max 5,000 lines with automatic trimming.

**XboxMappingEditor** (`xbox_mapping_editor.py`, 351 lines): Modal dialog.
- Tabbed: Buttons (12), Axes (4 groups), D-Pad (4 directions).
- Dropdown command selectors for all available Processor commands.
- Import/Export/Reset to Defaults/Save.

**CameraWidget** (`camera_widget.py`, 264 lines): Live microscope feed.
- OpenCV-based capture with configurable camera index and FPS.
- Crosshair overlay for needle alignment.
- Snapshot capture to timestamped files.
- Graceful fallback if `cv2` not installed (`CV2_AVAILABLE` flag).

**UIFunctions** (`ui_functions.py`, 161 lines): PyDracula animation helpers.
- `toggleMenu(window)`: Animate sidebar expand (60px → 200px) / collapse.
- `toggleLeftBox(window)`: Animate context panel open (0 → 260px) / close.
- `setLeftBoxWidth(window, width)`: Set context panel to specific width.
- All animations use `QParallelAnimationGroup` on both `minimumWidth` and `maximumWidth`.
- `selectMenu(style)` / `deselectMenu(style)`: Append/remove active indicator stylesheet.

### 6.6 Styling & Theming

**COLORS dict** (`styles.py`): Python-accessible color constants for dynamic styling.
Used when QSS alone isn't sufficient (e.g., `QPainter` drawing, conditional `setStyleSheet()`).

| Key | Hex | Usage |
|-----|-----|-------|
| `base` | `#1e1e2e` | Main content background |
| `mantle` | `#181825` | Sidebar, top/bottom bars |
| `crust` | `#11111b` | Console background |
| `surface0` | `#313244` | Card/panel backgrounds, input fields |
| `surface1` | `#45475a` | Hover states, borders |
| `surface2` | `#585b70` | Active borders |
| `overlay0` | `#6c7086` | Disabled/dim text |
| `subtext0` | `#a6adc8` | Secondary text |
| `text` | `#cdd6f4` | Primary text |
| `green` | `#a6e3a1` | Success, connected |
| `red` | `#f38ba8` | Error, danger, disconnected |
| `yellow` | `#f9e2af` | Warning, paused |
| `blue` | `#89b4fa` | Accent, links |
| `mauve` | `#cba6f7` | Selected accent (brand purple) |
| `peach` | `#fab387` | Highlights |
| `pink` | `#f5c2e7` | Secondary accent |

**objectName conventions** — widgets use `setObjectName()` to pick up QSS rules:

| objectName | Widget Type | Purpose |
|------------|-------------|---------|
| `cardFrame` | QFrame | Card container with rounded border + hover |
| `sectionLabel` | QLabel | Bold mauve section header in main content |
| `contextSectionLabel` | QLabel | Bold mauve section header in context panel |
| `contextLabel` | QLabel | Form label in context panel |
| `valueLabel` | QLabel | Monospace data readout |
| `dimLabel` | QLabel | De-emphasized helper text |
| `successBtn` | QPushButton | Green-tinted action (Connect, Apply, Start) |
| `dangerBtn` | QPushButton | Red-tinted action (Disconnect, Abort, Delete) |
| `warningBtn` | QPushButton | Yellow-tinted action (Pause) |
| `accentBtn` | QPushButton | Mauve-tinted action (accent) |
| `flatBtn` | QPushButton | Borderless transparent button |
| `jogBtn` | QPushButton | Square jog direction pad button |

### 6.7 Responsive Behavior

- **Context panel width**: Adapts to window width via `resizeEvent()`:
  - < 1000px → 200px, < 1300px → 230px, ≥ 1300px → 260px
- **Context panel inputs**: QSS `#extraLeftBox` scoping applies smaller `min-height`, `padding`, and `font-size` to spinboxes, combos, buttons, checkboxes, progress bars, and list widgets.
- **Sidebar**: Fixed 60px collapsed. Expand/collapse animated with `QParallelAnimationGroup`.
- **Console**: Collapsible via `QSplitter` handle — user can drag to hide.
- **Tested targets**: 1080p (minimum comfortable), 1440p (primary), 4K (scales naturally).

### 6.8 GUI File Summary

| File | Lines | Purpose |
|------|-------|---------|
| `gui/app.py` | 857 | MainWindow: PyDracula shell, navigation, timers, keyboard shortcuts |
| `gui/styles.py` | 861 | QSS dark theme (Catppuccin Mocha) + COLORS dict + objectName rules |
| `gui/ui_functions.py` | 161 | Animation helpers: sidebar toggle, context panel toggle, menu selection |
| `gui/pages/dashboard.py` | 533 | Device status, positions, speeds, safety, print history |
| `gui/pages/jog_control.py` | 402 | Direction pad, step sizes, speed sliders, keyboard E-stop |
| `gui/pages/calibration.py` | 693 | 3-step wizard with camera feed, plate teaching, save/load |
| `gui/pages/print_setup.py` | 1,059 | Job loading, 2D canvas, well plate, execution, queue |
| `gui/pages/settings_page.py` | 667 | Safety limits, polling, simulation mode, Xbox, logging |
| `gui/widgets/console_log.py` | 171 | ConsoleLogWidget + QtLogHandler with thread-safe signal bridge |
| `gui/widgets/xbox_mapping_editor.py` | 351 | Modal editor for button/axis/dpad → command mappings |
| `gui/widgets/camera_widget.py` | 264 | Live microscope camera feed (OpenCV, optional) |
| `gui/tests/test_integration.py` | 278 | Headless integration test (all pages + shell + contracts) |
| **Total GUI** | **6,453** | **17 files** |

---

## 7. Threading Model

```
┌─ Main Thread (GUI event loop) ─────────────────────────────────────────────┐
│  QTimer (300ms) → on_status_update() → read cached positions               │
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
GUI Timer (300ms)
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

## 13. Migration & Integration Status

### GUI Migration: Complete

All 5 pages have been migrated from the original tab-based layout to the PyDracula sidebar + context panel architecture. The migration was completed across multiple sessions with systematic integration testing.

**Bugs found and fixed during integration:**
1. `app.py` `_update_status()` treated `get_zp_position()` tuple as dict (would crash on timer tick)
2. `ui_functions.py` only animated `minimumWidth` — collapsing animations broke (fixed with `QParallelAnimationGroup`)
3. `ui_functions.py` never toggled logo text visibility on sidebar expand/collapse
4. `settings_page.py` context panel labels started blank (context created after `_load_from_controller()`)
5. `xbox_mapping_editor.py` used `connectBtn` objectName with no QSS rule (changed to `successBtn`)
6. `settings_page.py` had redundant local `SafetyLimits` import

### Integration Testing

Integration test script: `gui/tests/test_integration.py` (run headless with `QT_QPA_PLATFORM=offscreen`).

Tests cover:
- All module imports succeed
- All 5 pages instantiate in simulation mode
- Interface contract: `get_page_title()`, `get_context_widget()`, `on_status_update()` all present and callable
- Context widgets are properly cached (idempotent)
- `PathPreviewCanvas` methods work
- Settings page safety toggle syncs bidirectionally
- Console log and `QtLogHandler` thread-safe operation
- Full `MainWindow` lifecycle: creation, page switching, status updates, keyboard events, settings save

### API Compatibility Verification

All GUI files were verified against the rewritten SupportClasses APIs. One fix was applied: `emergency_stop()` alias added to `ZPStage.py` (jog control page calls it). All other method signatures match.

Key API surface verified:
- `StageController`: `connect_stages()`, `disconnect_xy/zp()`, `connect_xbox()`, `move_xy_absolute()`, `move_z_absolute()`, `move_z_relative()`, `move_pump_relative()`, `get_xy_position()`, `get_zp_position()`, `get_speed_info()`, `_calibrate_zero()`, `shutdown()`, `is_xy_connected`, `is_zp_connected`, `zero_position`, `safety_limits`, `position_logger`, `xy_jog`, `zp_jog`, `_pos_poller`
- `XYJogHandler`: `speed` property, `xy_speed` attribute
- `ZPJogHandler`: `speeds` property, `z_speed`/`p_speed` attributes
- `SafetyLimits`: `clamp_xy()`, `clamp_z()`, `clamp_pump()`, `to_dict()`, `from_dict()`, `enabled`, all min/max attributes
- `PositionLogger`: `count`, `clear()`, `save_csv()`, `save_json()`, `generate_filename()`
- `ZPStage`: `emergency_stop()` (alias for `reset_printer()`)

### Smoke Testing Checklist

- [ ] Run `python main.py` in simulation mode — verify all tabs load
- [ ] Xbox: connect simulated stages, verify jog commands flow through Processor
- [ ] Print execution: load a sample job, start/pause/abort cycle
- [ ] Calibration: run through 3-step wizard with simulated positions
- [ ] Settings: modify safety limits, save, restart, verify persistence
- [ ] Camera: test on machine with USB microscope connected
- [ ] Print queue: add multiple jobs, test drag-reorder, start queue

### Enhancement Summary

| # | Feature | Status | Key Files |
|---|---------|--------|-----------|
| 1 | G-code export | ✅ | `PrintManager.export_gcode()`, Print Setup export button |
| 2 | Camera integration | ✅ | `gui/widgets/camera_widget.py`, Calibration page step 3 |
| 3 | Print resume | ✅ | `PrintManager.save/load/clear_print_progress()`, MainWindow popup |
| 4 | Keyboard jogging | ✅ | `MainWindow.keyPressEvent()`, arrows/PgUp/PgDn/Home/Escape |
| 5 | Calibration persistence | ✅ | `settings.json` calibration section, CalibrationPage save/load |
| 6 | Print history | ✅ | `PrintHistory.py`, Dashboard stats and export |
| 7 | PyDracula layout | ✅ | Sidebar nav, animated context panel, responsive sizing |
| 8 | Visual polish | ✅ | Hover effects, state-based conn dots, card hover, responsive context |

---

