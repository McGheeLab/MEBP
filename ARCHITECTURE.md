# Stage Controller Application - Architecture

## Overview
Application to setup and control XY stage (Prior ProScan III) and ZP stage (3D printer/Marlin) 
for 3D bioprinting. Supports Xbox controller for real-time jogging and a GUI for setup, 
calibration, and print execution.

## Hardware
- **XY Stage**: Prior ProScan III — serial RS-232, commands: VS (velocity), G (goto), P (position query)
- **ZP Stage**: 3D printer board (Marlin firmware) — serial, G-code (G0, G91, M114, etc.)
  - Z axis = vertical needle
  - P1/P2/P3 = syringe pumps (mapped to Y/Z/E on the printer board)
- **Xbox Controller**: Pygame-based, runs in separate process for responsiveness

## Folder Structure
```
project/
├── main.py                         # Entry point - launches GUI or headless mode
├── settings.json                   # App configuration
├── current_button_mapping.json     # Xbox button-to-command mapping
├── sample_jobs/                    # Example print job files
│   ├── test_line.json
│   └── wellplate_meander.json
├── SupportClasses/
│   ├── __init__.py
│   ├── Processor.py                # Thread-safe command queue & dispatch
│   ├── XYStage.py                  # XY stage manager + simulator
│   ├── ZPStage.py                  # ZP stage manager + simulator
│   ├── XboxController.py           # Xbox polling worker (multiprocessing)
│   ├── StageController.py          # Jog handlers, position poller, stage coordination
│   ├── WellPlate.py                # Well plate geometry & path pattern generators
│   ├── PrintManager.py             # Print file loading, job building, execution engine
│   ├── Settings.py                 # Persistent settings (JSON save/load)
│   └── SerialUtils.py              # Serial retry, watchdog, friendly errors
├── gui/
│   ├── __init__.py
│   ├── app.py                      # QApplication + MainWindow setup
│   ├── styles.py                   # QSS stylesheets (dark theme)
│   ├── pages/
│   │   ├── __init__.py
│   │   ├── dashboard.py            # Status overview of all devices
│   │   ├── jog_control.py          # Manual jog with speed controls
│   │   ├── calibration.py          # Needle/well plate calibration workflow
│   │   └── print_setup.py          # Print loading, 2D preview, well plate, execution
│   └── widgets/
│       ├── __init__.py
│       ├── console_log.py          # Log viewer widget + QtLogHandler
│       └── xbox_mapping_editor.py  # Xbox button mapping editor dialog
```

## Key Design Decisions

### 1. Processor (Command Bus)
- Single threaded command queue - all device commands go through here
- Pub/sub pattern: handlers register for command names
- Thread-safe via `queue.Queue`
- Same proven pattern from XBOXCONTROLLED code

### 2. Xbox Controller (Separate Process)
- Runs in `multiprocessing.Process` for latency isolation  
- Communicates via `multiprocessing.Queue`
- Button mapping loaded from JSON, hot-reloadable
- Axis averaging with configurable deadzone
- **This code is proven and should not be changed**

### 3. Stage Controllers (Jog Handlers)
- `XYJogHandler`: Continuous velocity commands to XY stage
- `ZPJogHandler`: Segmented relative moves to ZP stage
- Each runs its own daemon thread for timing
- Speed multipliers adjustable via Xbox buttons or GUI

### 4. GUI Strategy
- **Simple PySide6** - no custom title bars, no theme framework
- Tab-based navigation (QTabWidget)
- Pages: Dashboard, Jog Control, Calibration, Print Setup
- Timer-based UI updates (500ms) polling stage state
- GUI sends commands through the same Processor as Xbox

### 5. Print Workflow
- **PrintManager**: Thread-based job executor with pause/resume/abort
- **PrintJob**: List of PrintCommands (move_xy, move_z, extrude, print_path, dwell, etc.)
- **File formats**: Custom JSON or simplified G-code import
- **Job Builder**: Generate jobs from well plate + pattern combinations
- **Execution**: Commands run sequentially in dedicated thread, progress via Qt signals
- **Safety**: Abort raises Z to travel height, pause completes current command first

### 6. Well Plate Support
- Standard ANSI/SLAS formats: 6, 12, 24, 48, 96 well
- Coordinates relative to A1 well center
- Path pattern generators: line, meander, spiral, grid, concentric rings
- Well plate job builder: pattern × selected wells × layers = complete job

### 7. 2D Path Preview
- Custom QPainter canvas widget
- Color-coded: green = print moves, gray dashed = travel moves
- Shows well plate outlines with labels
- Auto-scaling with grid background
- Origin marker at zero reference

### 8. Settings Persistence
- **Settings.py**: JSON-based config with dot-path access (e.g. `settings.get("window.width")`)
- Saves/loads: window geometry, active tab, splitter sizes, simulation mode, speed multipliers, zero reference, print settings, Xbox mapping file path
- Deep merge on load — missing keys fall back to defaults
- Auto-saved on window close, auto-loaded on startup

### 9. Serial Error Handling
- **SerialUtils.py**: Retry decorator, port health checking, safe read/write wrappers
- **ConnectionWatchdog**: Background thread monitors serial port health every 3s
- Disconnect detection triggers GUI callback via Qt signal bridge
- Friendly error messages (access denied, device disconnected, timeout, etc.)
- All position queries wrapped in try/except with debug logging

### 10. Position Polling
- **PositionPoller**: Background thread polls stage positions at 300ms intervals
- UI timer reads cached positions (non-blocking) — prevents GUI freezes from serial I/O
- PrintManager uses `cached=False` for direct queries during execution
- Thread-safe via lock-protected cache
- Automatically starts/stops with stage connections

### 11. Logging Integration
- Python `logging` module routed to console widget via `QtLogHandler`
- Thread-safe via `LogSignalBridge` (QObject + Signal)
- Color-coded by level: debug=gray, info=white, warning=yellow, error=red
- All SupportClasses emit structured log messages
- Console widget in GUI splitter with clear button

### 12. Xbox Mapping Editor
- **XboxMappingEditor**: Modal dialog with table-based editing
- Tabbed: Buttons (12), Axes (4 groups), D-Pad (4 directions)
- Dropdown command selectors with all available commands
- Human-readable input labels (A/B/X/Y, Left Stick, etc.)
- Import/Export to separate files, Reset to Defaults
- Saved changes hot-reloaded by controller every 5 seconds

## Axis Mapping (ZP Stage)
| Logical | Printer Axis | Description |
|---------|-------------|-------------|
| Z       | X           | Vertical needle |
| P1      | Y           | Syringe pump 1 |
| P2      | Z           | Syringe pump 2 |
| P3      | E           | Syringe pump 3 |
