# MEBP v7.3.6 — Architecture Reference

**Multi-Extrusion Bioprinting Platform**
**Version 7.3.6 | March 2026**

---

## 1. System Overview

MEBP is a desktop application for controlling laboratory bioprinting equipment. It orchestrates Prior ProScan XY stages, Marlin-based Z-axis and pump controllers, and Hamilton syringe systems to precisely deposit biological materials into well plates. The platform supports multi-material printing with various ink types (hydrogels, cell suspensions, granular materials) using multi-channel needles and sophisticated trajectory planning.

The application is written in Python with PySide6 for the GUI, using RS-232 serial communication for hardware control and JSON for all configuration and data persistence. An Xbox controller provides real-time jogging capability. The interface follows a Catppuccin Mocha dark theme.

### Core Design Principles

1. **Hardware abstraction** — GUI pages never touch serial ports. All hardware access goes through `StageController` or `PrintManager`.
2. **JSON protocol abstraction** — XY stage commands are defined in JSON protocol files (`config/controllers/`), enabling support for different controller models (ProScan II vs III) without code changes.
3. **Hardware-first configuration** — Page 0 (Hardware Setup) must be completed before any other page is accessible. `HardwareConfig` is the single source of truth for all µL ↔ mm conversions.
4. **µL-native pump control** — All user-facing pump values are in microliters. Conversion to mm (for Marlin firmware) happens exclusively inside `StageController.move_pump_uL()` via `SyringeSpec`.
5. **Safety-layered architecture** — Software endstops, flow-rate clamping, crash recovery, dual-layer Z verification, and thread-safe patterns throughout.
6. **Persistent print files** — Print designs stored as JSON in `config/prints/` with auto-save, schema migration, and full CRUD management.
7. **Multi-ink per pump** — Each pump can be assigned multiple inks with configurable swap strategies for single-syringe multi-material workflows.
8. **Correct unit terminology** — XY stage position readout is in µm (not microsteps). The `xy_position_scale` factor converts between stage readout units and µm (identity 1.0 for ProScan). The term "microsteps" is reserved for actual physical motor microsteps.
9. **DPI-aware GUI** — All pixel and font-size values in the GUI are wrapped with scaling functions (`s()` for pixels, `sf()` for points) from `gui/scaling.py`. The theme QSS is generated dynamically via `build_theme(scale_factor)`. The UI never shrinks below its 96-DPI design size, only grows on high-DPI screens.

---

## 2. Technology Stack

| Layer | Technology |
|-------|-----------|
| Language | Python 3.10+ |
| GUI Framework | PySide6 (Qt 6) |
| Theme | Catppuccin Mocha dark theme (dynamic QSS via `build_theme(k)`) |
| Serial Communication | pyserial (RS-232) |
| Numerical Computation | NumPy, SciPy |
| Computer Vision | OpenCV (well/needle detection, focus scoring, calibration) |
| Input Devices | pygame (Xbox controller via multiprocessing) |
| Configuration | JSON (hardware, controllers, prints, settings, simulation state) |
| Testing | unittest + physics-based simulators |
| Packaging | PyInstaller (--onedir, standalone executable) |

---

## 3. Folder Structure

```
MEBP/
├── main.py                              # Entry point — CLI arg parsing, mode selection
├── MEBP.spec                          # PyInstaller spec file (--onedir bundle)         ← v7.3.6
├── build_exe.py                       # Build script: python build_exe.py [--clean] [--debug]  ← v7.3.6
├── .gitignore                         # Build artifacts, runtime files, IDE configs      ← v7.3.6
├── settings.json                        # Persistent app config (auto-saved)
├── print_resume.json                    # Crash recovery state
├── current_button_mapping.json          # Xbox controller mapping (hot-reloadable)
├── Default.json                         # Default configuration template
├── INSTRUCTIONS.md                      # User operating guide
├── README.md                            # GitHub landing page
│
├── config/
│   ├── controllers/
│   │   ├── proscan_ii.json              # ProScan II command protocol (xy_position_scale: 1.0)
│   │   └── proscan_iii.json             # ProScan III command protocol (xy_position_scale: 1.0)
│   ├── hardware/
│   │   ├── needles.json                 # Needle gauge catalog (16G–32G)
│   │   ├── syringes.json                # Hamilton syringe catalog (25–1000 µL)
│   │   ├── objectives.json              # Per-camera objective µm/px calibrations  ← v7.3.4
│   │   └── Standard Bioprinting Setup.json  # Example HardwareConfig
│   ├── prints/                          # Persistent print file storage
│   │   ├── PIAR.json                    # Saved print designs
│   │   └── ...
│   ├── sim_zp_state.json               # ZP simulator EEPROM state (runtime, not committed)  ← v7.3.5
│   ├── sim_xy_state.json               # XY simulator position state (runtime, not committed)  ← v7.3.5
│   └── xy_diagnostic_profile.json       # XY diagnostic test profile
│
├── SupportClasses/                      # Backend — zero GUI dependencies
│   ├── __init__.py                      # Package exports + module listing
│   ├── Processor.py                     # Thread-safe command bus (pub/sub dispatch)
│   ├── SerialUtils.py                   # Retry decorator, safe I/O, port discovery, watchdog + periodic callbacks  ← v7.3.5
│   ├── XYStage.py                       # Prior ProScan XY stage manager
│   ├── XYStageSimulator.py              # Physics-based XY simulator with state persistence  ← v7.3.5
│   ├── ZPStage.py                       # Marlin Z/pump board manager (G-code) — flush_moves serial drain  ← v7.3.5
│   ├── ZPStageSimulator.py              # Physics-based Z/pump simulator with EEPROM persistence  ← v7.3.5
│   ├── XboxController.py                # Polling worker (multiprocessing.Process)
│   ├── StageController.py               # Top-level hardware orchestrator — dual-layer Z verification  ← v7.3.5
│   ├── ControllerProtocol.py            # JSON-based command protocol loader
│   ├── PrintManager.py                  # Print execution engine + trajectory tracking
│   ├── PrintPlanOfAction.py             # Print plan generation + command sequencing
│   ├── PrintRecorder.py                 # Auto-record prints, replay data
│   ├── PrintHistory.py                  # Persistent print history log
│   ├── PrintFileManager.py              # Print file CRUD + schema migration
│   ├── WellPlate.py                     # ANSI/SLAS plate geometry (6–384 well)
│   ├── WellSetup.py                     # Well assignments, rosettes, plane fitting
│   ├── SafetyLimits.py                  # Software endstops + flow-rate clamping (µm labels)  ← v7.3.5
│   ├── Settings.py                      # JSON-backed persistent app config
│   ├── PositionLogger.py                # Timestamped position recording + CSV export
│   ├── HardwareConfig.py                # Central config: pumps, inks, needles, µL↔mm
│   ├── PhysicalModels.py                # Needle, Syringe, Ink, FluidColumn dataclasses
│   ├── TrajectoryPlanner.py             # Time-parameterized waypoint paths
│   ├── MotionController.py              # Kalman/PID motion tracking
│   ├── FlowPhysics.py                   # Pressure/flow validation
│   ├── GeometryEngine.py               # Parametric object generation + fill patterns
│   ├── ImagePathPlanner.py              # Image-stack-to-toolpath raster generator  ← v7.2.9
│   ├── VelocityExecutor.py              # Velocity-based motion execution
│   ├── auto_layout.py                   # Ring/grid/hex layout generators
│   ├── VisionDetector.py                # Well/needle detection, focus scoring  ← v7.3.0
│   ├── SimulatedCamera.py               # Synthetic microscope with DOF model   ← v7.3.0
│   ├── MosaicBuilder.py                 # Affine calibration (Procrustes SVD)   ← v7.3.1
│   ├── MosaicCalibrator.py              # Manual SVD Procrustes point-set reg.  ← v7.3.4
│   ├── ObjectiveCalibration.py          # Per-camera objective µm/px store      ← v7.3.4
│   ├── ImageStitcher.py                 # Stage-coordinate image stitching      ← v7.3.3
│   └── PickAndPlaceManager.py           # Pick & place execution backend        ← v7.3.3
│
├── gui/                                 # Frontend — PySide6 / Qt 6
│   ├── __init__.py
│   ├── app.py                           # MainWindow + page routing + signal bridge
│   ├── styles.py                        # Catppuccin Mocha QSS — build_theme(k) parameterized  ← v7.3.6
│   ├── ui_functions.py                  # Animation helpers
│   ├── scaling.py                        # DPI-aware scaling utility: s(), sf(), scale_factor()  ← v7.3.6
│   ├── unit_helpers.py                  # Unit conversion: stage_to_um(), um_to_stage(), xy_position_scale  ← v7.3.5
│   ├── pages/
│   │   ├── hardware_setup.py            # Page 0: Hardware configuration
│   │   ├── dashboard.py                 # Page 1: Status overview + connections
│   │   ├── jog_control.py              # Page 2: Manual jogging + Xbox control
│   │   ├── calibration.py              # Page 3: Camera alignment + Z-plane + click-to-move
│   │   ├── printing_mode.py            # Page 4: Printing mode container  ← v7.3.3
│   │   ├── print_setup.py              # Sub-page: Print setup (4 tabs)
│   │   ├── print_workspace.py          # Tab 1: Hardware summary
│   │   ├── print_objects.py            # Tab 2: Object designer
│   │   ├── print_well_setup.py         # Tab 3: Well assignments
│   │   ├── print_monitor.py            # Sub-page: Print execution monitor
│   │   ├── print_results.py            # Sub-page: Post-print analysis
│   │   ├── helper_functions.py         # Sub-page: Utility functions
│   │   ├── pick_place_mode.py          # Page 5: Pick & Place mode container  ← v7.3.3
│   │   ├── pp_target_selection.py      # Sub-page: Camera + target selection
│   │   ├── pp_operation_setup.py       # Sub-page: Operation config
│   │   ├── pp_operation_queue.py       # Sub-page: Queue management
│   │   ├── pp_execution.py            # Sub-page: Execution monitor
│   │   ├── settings_page.py            # Page 6: Safety/serial/simulation/ZP stage  ← v7.3.5 ZP card
│   │   └── mode_page.py               # ModePage base class  ← v7.3.3
│   └── widgets/
│       ├── console_log.py              # Logging viewer + Qt log handler
│       ├── camera_widget.py            # OpenCV camera feed
│       ├── camera_manager.py           # Shared camera feed manager  ← v7.3.3
│       ├── camera_feed_view.py         # Camera display with event filter  ← v7.3.5
│       ├── target_overlay_camera_view.py  # Live feed target overlays  ← v7.3.3
│       ├── well_plate_view.py          # Well plate visualization
│       ├── well_preview.py             # Individual well preview
│       ├── trajectory_view.py          # XY path visualization
│       ├── projection_canvas.py        # L-shaped triple projection (XY/ZY/XZ)
│       ├── syringe_display.py          # Pump volume indicator
│       ├── xbox_mapping_editor.py      # Controller config dialog
│       ├── toupcam_backend.py          # ToupTek camera DLL wrapper
│       ├── detection_overlay.py        # QPainter vision overlay              ← v7.3.0
│       ├── detection_worker.py         # QThread pull-based frame consumer    ← v7.3.0
│       ├── jog_button_array.py         # Reusable jog XY+Z widget            ← v7.3.1
│       └── jog_well_plate.py           # Well plate navigator for Jog page   ← v7.3.1
│
├── tests/                               # Test suite
│   ├── test_s2_jog_and_filebrowser.py   # Jog + unit conversion tests (xy_position_scale)  ← v7.3.5
│   ├── test_v726_print_execution.py
│   ├── test_v726_integration.py
│   ├── test_sim_vs_hardware.py
│   ├── test_v73_trajectory_planner.py
│   └── ...
│
├── patches/                             # Version upgrade patches (100+)
├── print_records/                       # Print execution logs (CSV/JSON)
├── PrintImages/                         # Image assets for image-based printing
├── sample_jobs/                         # Example print job files
│
└── coding plans/                        # Development documentation
    ├── Architectures/
    │   ├── ARCHITECTURE_V723.md
    │   ├── ARCHITECTURE_V729.md
    │   ├── ARCHITECTURE_V735.md
    │   └── ARCHITECTURE_V736.md         # ← This document
    ├── Readmes/
    └── Update plans/
        ├── MEBP_v734_to_v735_UPDATE.md
        └── MEBP_v735_to_v736_UPDATE.md
```

---

## 4. Architecture Layers

### 4.1 Hardware Abstraction Layer

```
StageController (top-level orchestrator)
├── Processor (thread-safe command bus — FIFO queue, pub/sub)
├── XYStageManager  ──or──  XYStageSimulator (with state persistence)
│   └── ControllerProtocol (JSON command definitions, xy_position_scale)
├── ZPStageManager  ──or──  ZPStageSimulator (with EEPROM persistence)
├── XboxQueuePoller (multiprocessing.Queue → Processor dispatch)
├── XYJogHandler (continuous velocity jogging)
├── ZPJogHandler (velocity → relative move conversion)
├── PositionPoller (background position caching, suspend/resume)
├── SafetyLimits (software endstops for all axes, µm units)
├── PositionLogger (timestamped motion recording)
└── ConnectionWatchdog (disconnect detection + periodic callbacks)  ← v7.3.5
    └── add_periodic(callback) / remove_periodic(callback)
```

**Key Properties:**
- GUI pages call `StageController` methods — never raw serial
- Simulator vs. hardware selected at startup via CLI flags
- Xbox controller runs in a separate `multiprocessing.Process` for latency isolation
- `Processor` serializes all device commands through a single FIFO queue
- `ConnectionWatchdog` supports general-purpose periodic callbacks (used for M500 EEPROM saves)
- Simulators persist state to JSON files for session continuity

### 4.2 Print Execution Layer

```
PrintManager (execution engine)
├── PrintQueue (multi-job sequencing)
├── PrintJob (single job: commands + settings + metadata)
├── PrintSettings (feedrate, Z heights, retract/prime amounts)
├── TrajectoryExecutor (smooth path tracking via MotionController)
├── ServiceSequenceExecutor (waste→wash→buffer→ink cycles)
├── FluidColumnTracker (per-syringe fill state tracking)
└── PrintRecorder (actual vs. planned motion recording)

PrintPlanOfAction (plan generation)
├── PlanStep / PlanStepType (typed plan nodes)
│   └── Step types: LOAD_INK, WASH, WASTE, REFILL_BUFFER, PRINT, RETURN_HOME,
│                    GATHER_INK, FINAL_CLEANUP, TRAVEL_XY, MOVE_SAFE_Z, INK_SWAP
├── PrintExecutionConfig (comprehensive execution config)
│   ├── InkSwapStrategy (6-step cleaning sequence)
│   ├── InkGatherConfig (per-ink pickup volume + extra %)
│   ├── ZTravelConfig (safe Z, plunge buffer, fast entry/exit)
│   ├── XYTravelConfig (fast travel speed)
│   └── FinalCleanupConfig (end-of-print cleanup sequence)
├── PLAN_STEP_COLORS / PLAN_STEP_ICONS (Catppuccin Mocha visualization maps)
└── generate_plan() → plan_to_commands() pipeline
```

**Print Job Pipeline:**
1. Hardware Setup (Page 0) → validates HardwareConfig
2. Print Objects Tab (Page 4, Tab 2) → design objects with GeometryEngine
3. Well Setup Tab (Page 4, Tab 3) → assign objects to wells
4. Finalize Tab (Page 4, Tab 4) → configure PrintExecutionConfig → generate PrintPlanOfAction → build PrintJob
5. Print Monitor (Page 4, sub-page) → execute via PrintManager
6. Print Results (Page 4, sub-page) → analyze recorded data

### 4.3 Configuration Layer

```
HardwareConfig (central authority for µL↔mm conversions)
├── NeedleSpec (gauge, OD/ID, channel count)
├── PumpChannelConfig × 3 (P1, P2, P3)
│   ├── SyringeSpec (volume, stroke_length_mm, barrel_id_mm)
│   ├── inks: list[InkSpec]  — multi-ink per pump
│   ├── FluidColumn (oil_uL, buffer_uL, ink_uL)
│   └── enabled: bool
├── InkLibrary: dict[str, InkSpec]
│   └── InkSpec (name, viscosity, granule_diameter, density, color)
├── InkSwapStrategy  — re-exported from PrintPlanOfAction for backward compat
│   ├── 6 step toggles: waste, wash_pre, buffer, wash_post, ink_load, wash_final
│   └── 4 volumes: waste_volume_uL, wash_volume_uL, buffer_volume_uL, ink_load_volume_uL
└── RosetteInsert (rosette geometry for guided deposition)
```

### 4.4 Geometry & Trajectory Layer

```
GeometryEngine
├── ObjectType (enum): POINT, LINE, CIRCLE, SQUARE, TRIANGLE, SPIRAL, ELLIPSE,
│                       SPHERE_SHELL, SPHERE_SOLID, CUBE_SHELL, CUBE_SOLID, etc.
├── PrintObject (name, object_type, params, position, well_id)
├── PrintCollection (ordered list of PrintObjects)
├── generate_object_trajectory() → Nx7 numpy array [x,y,z,p1,p2,p3,t]
│
├── 2D Generators:
│   ├── generate_circle(), generate_square(), generate_triangle(), generate_ellipse()
│   ├── generate_line(), generate_point(), generate_spiral()
│   └── Fill Patterns:
│       ├── generate_meander_fill() (rectangular)
│       ├── generate_circular_meander_fill()
│       ├── generate_elliptical_meander_fill()
│       └── generate_triangular_meander_fill()
│
├── 3D Generators:
│   ├── Sphere: shell (stacked circles) / solid (layer-by-layer fill)
│   ├── Cube: shell (perimeter layers) / solid (meander fill layers)
│   ├── Cylinder: shell / solid
│   └── Ellipsoid: shell / solid
│
└── Trajectory Format: Nx7 numpy array
    [x_mm, y_mm, z_mm, pump1_uL, pump2_uL, pump3_uL, time_s]
```

### 4.5 GUI Layer

```
MainWindow (app.py)
├── QStackedWidget (page router) — mode-based navigation  ← v7.3.3
│   ├── Page 0: HardwareSetupPage      — needle, pumps, inks, ink swap strategy
│   ├── Page 1: DashboardPage           — connection status, position readouts
│   ├── Page 2: JogControlPage          — manual jogging, Xbox mapping, well plate navigator
│   ├── Page 3: CalibrationPage         — 3-well auto-cal, manual SVD, Z-bottom, click-to-move
│   ├── Page 4: PrintingModePage        — mode container with sub-page icon nav  ← v7.3.3
│   │   ├── PrintSetupPage              — 4-tab print designer
│   │   │   ├── Tab 1: WorkspaceTab     — read-only hardware summary
│   │   │   ├── Tab 2: PrintObjectsTab  — CAD-like object editor
│   │   │   ├── Tab 3: WellSetupTab     — well assignments + roles
│   │   │   └── Tab 4: FinalizeTab      — execution config, plan of action, generate & send
│   │   ├── PrintMonitorPage            — execution visualization
│   │   ├── PrintResultsPage            — post-print analysis
│   │   └── HelperFunctionsPage         — utility functions
│   ├── Page 5: PickPlaceModePage       — mode container with sub-page icon nav  ← v7.3.3
│   │   ├── TargetSelectionPage         — camera + stitched image, click targets
│   │   ├── OperationSetupPage          — operation configuration
│   │   ├── OperationQueuePage          — queue management + auto-building
│   │   └── ExecutionPage               — real-time execution monitor
│   └── Page 6: SettingsPage            — safety limits, serial ports, ZP stage card  ← v7.3.5
├── Left Sidebar (navigation icons)
├── Top Bar (status dots: XY, ZP, Xbox + global position in µm)
├── Context Panel (right-side, page-specific controls)
├── Console Log (bottom, collapsible)
├── Scaling (gui/scaling.py)  ← v7.3.6
│   ├── scale_factor() → max(1.0, screen.logicalDPI / 96) or MEBP_UI_SCALE env override
│   ├── s(px) → scaled pixel value (int)
│   ├── sf(pt) → scaled font point (float)
│   └── scaled_font_size(pt) → rounded scaled point (int)
└── Theme Engine (gui/styles.py)  ← v7.3.6
    ├── build_theme(k) → full QSS string with all px/pt scaled by factor k
    ├── apply_scaled_styles(k) → updates module-level style constants
    └── Backward compat: DARK_THEME = build_theme(1.0)
```

---

## 5. Key Module Details

### 5.1 StageController.py — Hardware Orchestrator

The central hardware abstraction that manages all physical devices:

- **Connection management**: `connect_xy_stage()`, `connect_zp_stage()`, `connect_xbox()`
- **Position queries**: `get_xy_position()`, `get_z_position()` (cached via PositionPoller)
- **Motion**: `move_xy_absolute()`, `move_xy_relative()`, `move_z_relative()`, `move_pump_uL()`
- **Xbox integration**: `XboxQueuePoller` dispatches controller events to `XYJogHandler`/`ZPJogHandler`; `debug_mode` flag gates verbose trigger/dispatch/velocity logs
- **Xbox status**: `is_xbox_connected` property checks poller status (thread mode on macOS)
- **Properties**: `is_xy_connected`, `is_zp_connected`, `is_xbox_connected`, `current_position`
- **Safe travel**: `safe_travel_to(x_um, y_um, safe_z_mm)` — dual-layer Z verification:
  1. Suspends `PositionPoller` (prevents serial races)
  2. Sends Z move + `flush_moves` (M400 with serial buffer drain)
  3. Polls Z position via `wait_for_z_arrival()` (M114 queries) to verify physical arrival
  4. XY starts ONLY after BOTH layers confirm Z is at safe height
  5. Resumes poller after completion
- **ZP feedrate settings** (v7.3.5): `_zp_retract_feedrate`, `_zp_insert_feedrate` — configurable via Settings page ZP Stage card, applied during `safe_travel_to()`
- **Periodic M500** (v7.3.5): `_periodic_zp_position_save()` registered via `ConnectionWatchdog.add_periodic()` — saves ZP position to EEPROM at watchdog interval when enabled
- **PositionPoller**: `suspend()`/`resume()` methods pause background hardware polling to prevent serial races during programmatic moves. `_suspended` flag checked at top of `_poll_loop`.

**Critical serial protocol note (v7.3.4):** The ProScan II/III stage responds with `R\r` to *every* command — including position queries (`P`), relative moves (`GR`), absolute moves (`G`), and velocity commands (`VS`). Every command that writes to the serial port **must** consume this `R\r` ack under `_serial_lock` (write + read atomically). Failure to read the ack causes byte accumulation in the RX buffer.

**Critical serial buffer note (v7.3.5):** Marlin G-code commands (G90, G0, G91) each produce an "ok" response that `send_data()` does not consume. These stale "ok" responses accumulate in the RX buffer. `ZPStage.flush_moves()` now drains the serial input buffer (`reset_input_buffer()`) before sending M400 within a single `_serial_lock` acquisition, ensuring the next `readline()` receives M400's "ok" (which arrives only after physical motion completes).

### 5.2 HardwareConfig.py — Central Configuration

Single source of truth for hardware state. All µL↔mm conversions route through here.

- `PumpChannelConfig.inks: list[InkSpec]` — multi-ink per pump with backward-compat `ink` property
- `InkSwapStrategy` moved to `PrintPlanOfAction.py` (canonical location), re-exported here for backward compat
- `pump_ink_map` → `dict[str, list[str]]`, `ink_pump_map` → `dict[str, list[str]]`
- `can_handle_ink(ink_name)` for pump-ink resolution

#### Well Plate Calibration Data Flow (v7.3.4)

```
CalibrationPage
├── _predicted_positions  ← plate.get_all_positions_from_a1(taught_a1)
├── _calibrated_positions ← AffineCalibration.correct_positions(_predicted_positions)
│                           (set by auto-scan OR manual MosaicCalibrator training)
├── _three_well_calibration: AffineCalibration  ← persisted to settings.calibration.mosaic_affine
├── calibration_data_changed: Signal
└── get_calibration_data() → (plate, positions, safe_z)
                                    │
                      Signal connection + immediate push
                      (app.py wires after load_startup_plate)
                                    │
JogControlPage
├── _well_positions: dict (calibrated or approximate)
├── _safe_z: float
├── set_calibration_data(plate, positions, safe_z)
└── _well_nav: WellPlateNavigator
    ├── Calibrated positions  → green wells
    ├── Approximate positions → yellow wells
    ├── Current position      → blue highlight
    └── well_clicked.emit(well_name) → controller.safe_travel_to(x, y, safe_z)

Persistence:
  On training:   _save_calibration() → settings.calibration.mosaic_affine (auto-saved)
  On startup:    _load_calibration() → AffineCalibration → _calibrated_positions
                 app.py: load_startup_plate() then immediate set_calibration_data() push
```

**Coordinate systems:**
- All `_predicted_positions` and `_calibrated_positions` are in **absolute stage µm** (origin at stage mechanical limit). Zero-reference (`controller.zero_position`) is applied only when displaying values to the user. Calibrated positions remain valid even if the zero reference changes.

### 5.3 GeometryEngine.py — Parametric Object Generation

Generates time-parameterized trajectories as Nx7 numpy arrays.

### 5.4 PrintManager.py — Print Execution

Executes print jobs as sequences of typed commands:

- **Command types**: MOVE_XY, MOVE_Z, EXTRUDE, RETRACT, PRIME, SWITCH_PUMP, TRAJECTORY, SERVICE_SEQUENCE, WAIT, SET_FEEDRATE, GATHER_INK, TRAVEL_XY, MOVE_SAFE_Z, INK_SWAP, FINAL_CLEANUP
- **TrajectoryExecutor**: Smooth path tracking using MotionController (Kalman/PID)
- **ServiceSequenceExecutor**: Automated cleaning cycles (waste→wash→buffer→ink)
- **FluidColumnTracker**: Tracks per-syringe oil/buffer/ink volumes
- **States**: IDLE, RUNNING, PAUSED, ABORTING, COMPLETE, ERROR

### 5.5 SerialUtils.py — Communication Safety

Robust serial communication layer:

- `retry_serial(max_retries=3)` decorator with exponential backoff
- `safe_write()`, `safe_readline()`, `safe_read_all()` with timeout protection
- `ConnectionWatchdog` — background disconnect detection + periodic callbacks
  - `add_periodic(callback)` / `remove_periodic(callback)` — general-purpose hooks called on every watchdog cycle (v7.3.5)
  - Used for periodic ZP position save (M500 EEPROM writes)
- Exception hierarchy: `SerialError` → `SerialDisconnectedError` | `SerialTimeoutError` | `SerialAccessError`
- `list_serial_ports()` — platform-aware port discovery

### 5.6 PrintPlanOfAction.py — Plan Generation & Execution Config

Generates typed execution plans from well assignments and hardware configuration.

- `PrintExecutionConfig` composes: `InkSwapStrategy`, `InkGatherConfig`, `ZTravelConfig`, `XYTravelConfig`, `FinalCleanupConfig`
- 5 step types: `GATHER_INK`, `FINAL_CLEANUP`, `TRAVEL_XY`, `MOVE_SAFE_Z`, `INK_SWAP`
- `PlanStep` enriched with: `z_behavior`, `travel_mode`, `wait_for_z_confirm`, `extra_percent`, `max_pickup_uL`, `sub_steps`
- `PLAN_STEP_COLORS` and `PLAN_STEP_ICONS` maps for Catppuccin Mocha visualization

### 5.7 PrintTrajectoryPlanner.py — Plan-to-Trajectory

Converts `PrintPlanOfAction` into time-parameterized waypoint arrays. Handles all step types including compound steps (`INK_SWAP`, `FINAL_CLEANUP`) via `PlanStep.sub_steps`.

### 5.8 ImagePathPlanner.py — Image-to-Toolpath Generator

Converts image stacks into raster toolpaths for image-based printing:
- Three loading modes: TIFF stack, numbered image sequence, single image × N layers
- Per-pump greyscale intensity mapping to flow rates
- Output: Nx7 array `[x, y, z, p1, p2, p3, t]` (mm/s units)
- CSV export via `save_csv()`

### 5.9 Vision & Calibration System (v7.3.0 / v7.3.1 / v7.3.4)

```
VisionDetector.py
├── WellDetector (HoughCircles + contour fallback)
├── NeedleDetector (3-strategy: HoughCircles + contour + radial profile)
│   └── compute_focus_score() → FocusResult (70% Laplacian + 30% Tenengrad)
└── FocusTracker (temporal smoothing for focus assist display)

SimulatedCamera.py
├── Synthetic microscope renderer (well plate + needle)
├── DOF model: opacity = exp(-0.5*(defocus/dof_hw)²)
├── Blur model: sigma = normalized_defocus * BLUR_REFERENCE_UM / um_per_px
└── set_stage_position(x, y, z_mm)

MosaicBuilder.py — AffineCalibration (Procrustes SVD similarity transform)
MosaicCalibrator.py — Manual SVD Procrustes point-set registration  ← v7.3.4
ObjectiveCalibration.py — Per-camera objective µm/px store  ← v7.3.4

Detection Pipeline (GUI)
├── DetectionWorker (QThread, pull-based frame consumer)
└── DetectionOverlay (QPainter overlay with aspect-ratio-aware mapping)
```

### 5.10 ZPStageSimulator.py — Marlin EEPROM Simulation (v7.3.5)

Physics-based Z/pump simulator with full EEPROM state persistence:

```
ZPStageSimulator
├── Physics loop (100 Hz): motor counts tracked (delta_mm × steps_per_mm)
├── G-code parsing:
│   ├── M92  — set steps/mm (stored, not just no-op)
│   ├── M203 — set max feedrate (stored)
│   ├── M220 — set speed factor (stored)
│   ├── M400 — wait for moves (checks axis convergence)
│   ├── M500 — save state to sim_zp_state.json
│   ├── M501 — load state from sim_zp_state.json
│   └── M503 — report settings in Marlin echo format
├── State file: config/sim_zp_state.json
│   ├── position: {X, Y, Z, E}
│   ├── target_position: {X, Y, Z, E}
│   ├── steps_per_mm: {X, Y, Z, E}
│   ├── max_feedrate: {X, Y, Z, E}
│   ├── max_acceleration: {X, Y, Z, E}
│   ├── speed_factor_pct: int
│   ├── motor_counts: {X, Y, Z}
│   ├── absolute_mode: bool
│   └── cold_extrusion: bool
├── readline(timeout) — serial interface compatibility
└── Startup: auto-loads sim_zp_state.json if it exists
```

### 5.11 XYStageSimulator.py — ProScan State Persistence (v7.3.5)

Physics-based XY simulator with position persistence across sessions:

```
XYStageSimulator
├── Constructor: xy_position_scale parameter (default 1, replacing old microsteps_per_micron)
├── State file: config/sim_xy_state.json
│   ├── position: {x, y}
│   ├── settings: {speed_pct, accel_pct, scurve_pct}
│   └── stage_info: {stage_name, size_x_mm, size_y_mm, position_scale}
├── _save_state() — called on stop()
├── _load_state() — called on startup
└── Position persists across sessions (matches real ProScan behavior)
```

### 5.12 Unit Conversion System (v7.3.5)

```
gui/unit_helpers.py
├── DEFAULT_XY_POSITION_SCALE = 1.0
├── stage_to_um(readout, xy_position_scale) → µm
├── um_to_stage(microns, xy_position_scale) → stage units
├── get_position_scale_from_protocol(protocol) → float | None
├── convert_safety_xy_text(sl, xy_position_scale) → str
└── format_um(), format_um_pair(), format_um_range()

Resolution priority (in app.py._resolve_xy_position_scale):
  1. Controller protocol JSON → parameters.xy_position_scale
  2. settings.json → stage.xy_position_scale
  3. DEFAULT_XY_POSITION_SCALE (1.0)

Propagation: MainWindow.set_xy_position_scale() → all pages
```

**Terminology:** The ProScan reports positions in µm natively. The `xy_position_scale` factor is 1.0 (identity) for ProScan stages. The term "microsteps" is NOT used for position readout — only for actual physical motor microsteps.

### 5.13 GUI Scaling System (v7.3.6)

```
gui/scaling.py
├── BASELINE_DPI = 96.0
├── _compute_scale()
│   ├── 1. Check MEBP_UI_SCALE env var (manual override, min 0.5)
│   ├── 2. Query QApplication.primaryScreen().logicalDotsPerInch()
│   └── 3. Fallback: 1.0
├── scale_factor() → max(1.0, dpi / 96)  [cached after first call]
├── s(px: int) → int  (scaled pixel dimension)
├── sf(pt: float) → float  (scaled font point size)
└── scaled_font_size(pt: int) → int  (rounded scaled font point)

gui/styles.py
├── COLORS dict (unchanged Catppuccin Mocha palette)
├── build_theme(k) → parameterized QSS string
│   ├── _p(val, k) → round(val * k)  [pixel helper]
│   └── _f(val, k) → round(val * k, 1)  [font helper]
├── apply_scaled_styles(k) → updates: SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE,
│                              PAGE_HEADER_STYLE, CARD_FRAME_STYLE, etc.
└── Backward compat: DARK_THEME = build_theme(1.0)

Scaling applied across 20+ files:
  - gui/app.py: setMinimumSize, font sizes, icon sizes, sidebar widths
  - gui/ui_functions.py: menu collapsed/expanded widths, context panel
  - gui/widgets/*.py: button sizes, combo widths, overlay dimensions
  - gui/pages/*.py: all hardcoded pixel values wrapped with s() or _sc()

Note: Files using `s` as a loop variable import as `from gui.scaling import s as _sc`
```

### 5.14 PyInstaller Packaging (v7.3.6)

```
MEBP.spec (PyInstaller spec file)
├── Mode: --onedir (writable bundle, user data persists)
├── Bundled data:
│   ├── config/controllers/*.json
│   ├── config/hardware/*.json
│   ├── config/prints/*.json
│   ├── PrintImages/
│   └── Root config files (Default.json, *.json)
├── Hidden imports: PySide6, numpy, cv2, scipy, pygame, app modules
├── Excludes: tensorflow, torch, pyarrow, llvmlite, pandas, sklearn, PIL, grpc
│   (reduces bundle from 2.9GB to ~470MB)
└── console=False (GUI mode)

build_exe.py
├── Auto-installs PyInstaller if missing
├── Calls: pyinstaller MEBP.spec --noconfirm
├── --clean flag: removes build/dist before building
├── --debug flag: temporarily patches spec for console=True
└── Reports output size and run instructions

main.py changes for frozen apps:
├── multiprocessing.freeze_support() at entry point
├── os.chdir(sys._MEIPASS) for relative path resolution
└── QT_AUTO_SCREEN_SCALE_FACTOR=1 (replaces QT_FONT_DPI=96 lock)
```

---

## 6. Communication Protocols

### 6.1 Prior ProScan XY Stage (RS-232)

| Property | Value |
|----------|-------|
| Baud rate | 38400 |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Terminators | `\r\n` |
| Encoding | ASCII |
| Position unit | µm (stage reports in whole microns; `xy_position_scale` = 1.0) |

**Key commands** (from `config/controllers/proscan_iii.json`):
- `P` → query position → `x,y\r`
- `G x,y` → move absolute (in µm)
- `GR dx,dy` → move relative (in µm)
- `VS speed` → set velocity
- `SAS accel` → set acceleration
- `Z` → home (zero position)
- `I` → interrupt/stop

**Important:** Every command returns `R\r` acknowledgment. Must be consumed atomically under `_serial_lock` to prevent RX buffer accumulation.

### 6.2 Marlin Z/Pump Board (G-code, RS-232)

| Property | Value |
|----------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Terminators | `\n` |
| Encoding | ASCII |
| Axes | Z (vertical), E0/E1/E2 (pumps) |

**Key commands:**
- `G1 Z10 F60` → move Z to 10mm at 60mm/min
- `G1 E5 F10` → extrude 5mm at 10mm/min (pump)
- `M114` → report current position
- `G28 Z` → home Z axis
- `M400` → wait for all moves to complete (**used by `ZPStage.flush_moves()`** — only returns `ok` after physical completion; serial buffer is drained before sending)
- `M203 Z200` → set max feedrate for Z axis (mm/min)
- `M92 Z80` → set steps/mm for Z axis
- `M220 S100` → set speed factor percentage
- `M500` → save settings to EEPROM (used for periodic position save)
- `M501` → load settings from EEPROM
- `M503` → report current settings

**Important (v7.3.5):** Each G-code command produces an "ok" response that `send_data()` does not consume. `flush_moves()` drains the serial input buffer via `reset_input_buffer()` before sending M400, within a single `_serial_lock` acquisition.

### 6.3 Xbox Controller (pygame via multiprocessing)

- Runs in separate `multiprocessing.Process` for latency isolation
- Polls button/axis/DPAD events via `pygame.joystick`
- Events serialized as JSON dicts through `multiprocessing.Queue`
- `XboxQueuePoller` (thread in main process) dispatches events to `Processor` command bus
- Button mapping loaded from `current_button_mapping.json` (hot-reloadable)
- macOS: Uses thread mode (no subprocess) due to macOS Bluetooth restrictions
- **Trigger normalization (v7.3.4)**: Windows/Linux XInput triggers rest at `-1.0`; hardcoded offset normalizes to `0..1`. macOS triggers already `0..1`.
- **Heartbeat suppression (v7.3.4)**: `pygame.joystick.quit()/init()` every 3s causes triggers to transiently report `0.0`. `_accum_suppress_until` timestamp blocks axis accumulation after each reinit.
- **Debug mode (v7.3.4)**: `debug_mode` flag passed from Settings → Dashboard → subprocess. Gates TRIG DIAG / dispatch / velocity logs.

---

## 7. GUI Page Architecture

### Page 0: Hardware Setup (`hardware_setup.py`)

**Purpose:** Configure all hardware before other pages unlock.

**Sections:**
1. **Controller Selection** — ProScan II/III auto-detect
2. **Needle Configuration** — gauge, channel count, channel-pump mapping
3. **Pump Channels** — P1/P2/P3 each with syringe selection, multi-ink checklist, fluid column volumes
4. **Ink Library** — define ink specs (name, viscosity, density, color)
5. **Rosette Configuration** — insert geometry for guided deposition

**Output:** `HardwareConfig` instance propagated to all pages via `_propagate_hardware_config()`.

### Page 3: Calibration (`calibration.py`)

**Purpose:** Camera-based well plate alignment, Z-plane calibration, and click-to-move.

**v7.3.5 Fix:** Click-to-move now works reliably via Qt event filter on the `CameraFeedView._display` QLabel. The `eventFilter()` intercepts `MouseButtonPress` events, performs widget-to-image coordinate conversion, and emits the `clicked` signal directly — bypassing unreliable PySide6 event propagation.

### Page 4: Printing Mode (`printing_mode.py`) ← v7.3.3

**Purpose:** Container for all print-related sub-pages with right-side icon navigation.

**Sub-pages:** PrintSetup (4 tabs), PrintMonitor, PrintResults, HelperFunctions

### Page 5: Pick & Place Mode (`pick_place_mode.py`) ← v7.3.3

**Purpose:** Container for pick-and-place workflow with right-side icon navigation.

**Sub-pages:** TargetSelection, OperationSetup, OperationQueue, Execution

**3 operation modes:**
1. Spheroid pickup: V=(4/3)π(d/2)³ × safety_factor → aspirate/dispense
2. Trypsin cell: single or dual bore, trypsin dispense → dwell → extract cells
3. Fluorescent tagging: 1-3 dye bores, optional waste bore for skip-waste-well mode

### Page 6: Settings (`settings_page.py`)

**Purpose:** Safety limits, serial port configuration, simulation toggles, ZP stage settings.

**v7.3.5 Addition — ZP Stage Settings Card:**
- **Max Feedrate** (mm/min) — Hardware speed limit, sent to Marlin via M203. Default: 200.
- **Safe Z Retract Feedrate** (mm/min) — Used by `safe_travel_to()` Step 1 (raise needle). Default: 200.
- **Safe Z Insert Feedrate** (mm/min) — Used by `safe_travel_to()` Step 3 (lower needle). Default: 100 (half speed).
- **Default Jog Feedrate** (mm/min) — Used for manual jog/step moves. Default: 200.
- **Auto-Save Position** (checkbox) — Periodic M500 EEPROM save at watchdog interval. Off by default.

---

## 8. Data Flow Diagrams

### 8.1 Hardware Config Propagation

```
HardwareSetupPage
    │ (validates + builds HardwareConfig)
    ▼
MainWindow._propagate_hardware_config()
    ├──→ DashboardPage.set_hardware_config()
    ├──→ JogControlPage.set_hardware_config()
    ├──→ CalibrationPage.set_hardware_config()
    ├──→ PrintingModePage → PrintSetupPage.set_hardware_config()
    │       ├──→ WorkspaceTab.update_config()
    │       ├──→ PrintObjectsTab._refresh_ink_options_from_config()
    │       └──→ WellSetupTab.set_hardware_config()
    ├──→ PrintMonitorPage.set_hardware_config()
    └──→ Settings.save() (persisted to settings.json)
```

### 8.2 XY Position Scale Propagation (v7.3.5)

```
Startup
    │
    ▼
MainWindow._resolve_xy_position_scale()
    ├── 1. get_position_scale_from_protocol(controller_protocol)  → protocol JSON
    ├── 2. settings.get("stage.xy_position_scale")                → settings.json
    └── 3. DEFAULT_XY_POSITION_SCALE (1.0)                        → fallback
            │
            ▼
    self._xy_position_scale = resolved_value
            │
            ▼
    _propagate_xy_position_scale()
        ├──→ DashboardPage.set_xy_position_scale()
        ├──→ JogControlPage.set_xy_position_scale()
        ├──→ CalibrationPage.set_xy_position_scale()
        ├──→ PrintSetupPage.set_xy_position_scale()
        ├──→ PrintMonitorPage.set_xy_position_scale()
        ├──→ HelperFunctionsPage.set_xy_position_scale()
        └──→ SettingsPage.set_xy_position_scale()

Usage in GUI:
    stage_to_um(readout, self._xy_position_scale) → display in µm
    um_to_stage(target_um, self._xy_position_scale) → send to stage
```

### 8.3 Safe Z Travel (v7.3.5 Dual-Layer Verification)

```
StageController.safe_travel_to(target_x, target_y, safe_z)
    │
    ├── Step 0: Suspend PositionPoller
    │
    ├── Step 1: Retract Z to safe height
    │   ├── send G90 + G0 Z{safe} F{retract_feedrate} + G91
    │   ├── flush_moves():
    │   │   ├── Acquire _serial_lock
    │   │   ├── reset_input_buffer()  ← drain stale "ok" responses
    │   │   ├── Write "M400\n"
    │   │   └── readline() → wait for M400's "ok" (physical completion)
    │   └── wait_for_z_arrival():  ← Layer 2 verification
    │       ├── Poll M114 position
    │       └── Verify |current_z - safe_z| < tolerance
    │
    ├── Step 2: XY move (only if BOTH layers confirmed Z arrival)
    │   ├── move_xy_absolute(target_x, target_y)
    │   └── wait_for_xy_arrival()
    │
    ├── Step 3: Optional Z descend
    │   ├── G0 Z{target} F{insert_feedrate}
    │   └── flush_moves() + wait_for_z_arrival()
    │
    └── Step 4: Resume PositionPoller
```

### 8.4 Print Job Execution

```
PrintObjectsTab (design objects)
    ▼
WellSetupTab (assign to wells)
    ▼
FinalizeTab (configure PrintExecutionConfig)
    ├── InkSwapStrategy, ZTravelConfig, XYTravelConfig,
    │   InkGatherConfig, FinalCleanupConfig
    ▼
PrintSetupPage._generate_print()
    ├── PrintPlanOfAction.generate_plan(execution_config=...)
    └── plan_to_commands() → PrintJob
            ▼
PrintMonitorPage.receive_job()
    ▼
PrintManager.load_job() → start()
    ├── TrajectoryExecutor.execute()
    ├── ServiceSequenceExecutor.execute()
    └── PrintRecorder.record_sample()
            ▼
PrintResultsPage.load_recording()
```

---

## 9. Key Algorithms

### 9.1 Stage Position ↔ µm Conversion (v7.3.5)

```
ProScan XY: xy_position_scale = 1.0 (stage speaks µm natively)
  position_um = readout / xy_position_scale    → stage_to_um()
  stage_units = target_um * xy_position_scale  → um_to_stage()
  (Both are identity operations for ProScan; retained for future stage support)

Syringe: barrel cross-section determines µL↔mm
  area_mm2 = π × (barrel_id_mm / 2)²
  mm = µL / area_mm2 / 1000
  µL = mm × area_mm2 × 1000
```

### 9.2 Z-Plane Calibration

Three-point teach to fit plane `z = ax + by + c`:
1. User teaches 3 XY positions and their Z heights
2. Least-squares fit: `[a, b, c] = np.linalg.lstsq(A, z_values)`
3. For any well position `(wx, wy)`: `z_bottom = a*wx + b*wy + c`
4. Compensates for tilted well plates

### 9.3 Meander Fill Patterns

For 2D shapes (circle, square, triangle, ellipse):
1. Compute bounding scan lines at `spacing_mm` intervals
2. For each scan line, clip to shape boundary
3. Alternate left-to-right and right-to-left (serpentine)
4. Triangle: scan line width narrows linearly from base to apex

### 9.4 3-Well Auto-Calibration (v7.3.1)

Determines plate position and orientation from 3 reference wells:
1. Select calibration wells: A1, A(last_col), (last_row)(last_col) — right triangle
2. Navigate to approach, capture frame, run well detection via `WellDetector`
3. Convert pixel offset to stage coords using `pixel_offset_to_stage_um()`
4. Fit Procrustes SVD similarity transform from (predicted, detected) pairs
5. Apply `AffineCalibration.correct_positions()` to all predicted positions

### 9.5 Auto Z-Bottom Calibration (v7.3.1)

Focus-sweep algorithm to find well bottom Z:
1. **Coarse sweep** (0.15mm steps): Lower needle, capture focus score. Detect needle entry when score > 2× baseline.
2. **Fine sweep** (0.03mm steps): Track peak focus score. Stop on 3 consecutive declining steps.
3. Repeat for 3 calibration wells, fit Z-plane: `z = ax + by + c`

Focus scoring: 70% Laplacian variance + 30% Tenengrad (Sobel gradient)

### 9.6 Shell vs. Solid 3D Generation
- **Shell**: Generate shape perimeter at each Z layer (stacked outlines)
- **Solid**: Generate filled cross-section at each Z layer (meander fill per slice)
- Layer height configurable, number of points per layer configurable

---

## 10. Serialization & Persistence

### 10.1 Settings (settings.json)

Top-level keys: `window`, `simulation`, `speeds`, `zero_position`, `print_settings`, `xbox`, `logging`, `safety_limits`, `polling`, `calibration`, `controller`, `workspace`, `hardware_config`, `motion_controller`, `ui`, `execution`, `zp_stage` ← v7.3.5

**v7.3.5 additions under `zp_stage.*`:**
- `max_feedrate` (mm/min, default 200)
- `retract_feedrate` (mm/min, default 200)
- `insert_feedrate` (mm/min, default 100)
- `jog_feedrate` (mm/min, default 200)
- `auto_save_position` (bool, default false)

**v7.3.5 key rename:** `stage.microsteps_per_micron` → `stage.xy_position_scale`

Auto-saved on application exit via `Settings.save()`.

### 10.2 Hardware Config

```json
{
  "version": "7.2.9",
  "needle": { "gauge": "22G", "od_um": 717, "id_um": 413, "channels": 1 },
  "pumps": {
    "P1": {
      "pump_id": "P1",
      "enabled": true,
      "syringe": { "name": "Hamilton 250µL", "volume_uL": 250 },
      "inks": [
        { "name": "Hydrogel A", "viscosity_mPas": 50, "color": "#4CAF50" }
      ],
      "fluid_column": { "oil_uL": 50, "buffer_uL": 20, "ink_uL": 100 }
    }
  },
  "ink_library": { ... },
  "ink_swap_strategy": { ... }
}
```

### 10.3 Print Files (config/prints/*.json)

Standard print file format with objects, wells, and settings.

### 10.4 Simulation State Files (v7.3.5)

**ZP Simulator EEPROM** (`config/sim_zp_state.json`):
```json
{
  "_description": "Simulated Marlin EEPROM — saved by M500",
  "_last_saved": "2026-03-14T10:30:00",
  "position": {"X": 0.0, "Y": 0.0, "Z": 5.0, "E": 0.0},
  "target_position": {"X": 0.0, "Y": 0.0, "Z": 5.0, "E": 0.0},
  "steps_per_mm": {"X": 80.0, "Y": 80.0, "Z": 80.0, "E": 80.0},
  "max_feedrate": {"X": 200.0, "Y": 200.0, "Z": 200.0, "E": 200.0},
  "max_acceleration": {"X": 500.0, "Y": 500.0, "Z": 500.0, "E": 500.0},
  "speed_factor_pct": 100,
  "motor_counts": {"X": 0, "Y": 0, "Z": 400},
  "absolute_mode": true,
  "cold_extrusion": false
}
```

**XY Simulator State** (`config/sim_xy_state.json`):
```json
{
  "_description": "Simulated Prior ProScan state — saved on stop()",
  "_last_saved": "2026-03-14T10:30:00",
  "position": {"x": 1000.0, "y": 2000.0},
  "settings": {"speed_pct": 50, "accel_pct": 50, "scurve_pct": 50},
  "stage_info": {"stage_name": "H117P2IX", "size_x_mm": 114, "size_y_mm": 75, "position_scale": 1}
}
```

**Note:** These files are runtime-generated and should not be committed to git.

### 10.5 Controller Protocol Files

`config/controllers/proscan_ii.json` and `proscan_iii.json` define:
- Command templates (position query, move, velocity, etc.)
- Response parsing patterns
- `parameters.xy_position_scale` — scale factor between readout units and µm (1.0 for ProScan)

---

## 11. Thread Safety & Concurrency

| Component | Threading Model | Safety Mechanism |
|-----------|----------------|-----------------|
| StageController | Main thread + PositionPoller thread | Processor command bus (FIFO queue); PositionPoller suspended during `safe_travel_to` to prevent serial races |
| ZPStage | Main thread, shared serial | `_serial_lock` mutex; `flush_moves()` drains buffer + sends M400 atomically under lock |
| XboxController | Separate Process (multiprocessing) | Queue-based event passing |
| XboxQueuePoller | Daemon thread in main process | Dispatches to Processor; zeros jog velocities on disconnect |
| PrintManager | Daemon thread for execution | Qt Signals (QueuedConnection) for GUI updates |
| SerialUtils | Called from various threads | `retry_serial` decorator, `ConnectionWatchdog` |
| ConnectionWatchdog | Daemon thread | `_lock` mutex for periodic callback list; callbacks fire on watchdog cycle |
| GUI | Main thread only (Qt requirement) | Signal/slot for cross-thread updates |

---

## 12. Error Handling

### Serial Communication
- 3-retry with exponential backoff (`retry_serial` decorator)
- `ConnectionWatchdog` detects disconnects and emits status changes
- Graceful degradation: lost connection → red status dot → reconnect prompt
- **v7.3.5:** `flush_moves()` failure (timeout or serial error) causes `safe_travel_to()` to return `False` immediately — no "proceeding anyway"

### Print Execution
- `PrintManager` state machine: RUNNING → ERROR on unrecoverable failures
- `print_resume.json` captures crash state for recovery
- Safety limits enforced at every motion command

### GUI
- All hardware calls wrapped in try/except with console log output
- Status dots (top bar) reflect real-time connection state
- Page gating prevents access to features without valid config

---

## 13. Version History

| Version | Key Changes |
|---------|------------|
| v7.2.3 | Hardware Setup page, print file persistence, print monitor redesign |
| v7.2.4 | Pump channel mapping, ink assignment, HardwareConfig system |
| v7.2.5 | Initial multi-pump UI, ink exclusion logic |
| v7.2.6 | Print execution restoration (7 critical fixes) |
| v7.2.7 | Print speed display, execution mode selection |
| v7.2.8 | Xbox controller polling fixes, ProScan detection, stable build |
| v7.2.9 | Object type consolidation, filled/shell checkbox, multi-ink per pump, ink swap strategy, triangle fill, PrintExecutionConfig (replaces PlanPreferences), 5 new step types, Finalize tab, ImagePathPlanner |
| v7.3.0 | Autocalibration: VisionDetector (well/needle detection, 3-strategy needle, focus scoring), SimulatedCamera (Gaussian DOF model), DetectionOverlay + DetectionWorker QThread, camera integration (BUC3D-1000C ToupTek), 59 vision/camera tests |
| v7.3.1 | Calibration overhaul: geometry-predicted wells, 3-well auto-calibration (Procrustes SVD), auto Z-bottom calibration (focus-sweep), simplified wizard, per-well 50% overlap scanning, jog page well plate navigator, 110 tests |
| v7.3.2 | QoL upgrades: camera config persistence, jog page startup well plate, axis flip checkboxes, custom jog step sizes + absolute goto, calibration page restructure, print monitor camera overlay, Xbox stick zero calibration, safe travel Z-wait standardization |
| v7.3.3 | Mode-based navigation restructure (ModePage base class, right-side icon sub-nav), Printing mode container, Pick & Place mode (PickAndPlaceManager, ImageStitcher, 3 operation modes), CameraManager shared feeds, camera µm/px empirical calibration, relaxed needle detection with interactive edge refinement |
| v7.3.4 | **Bug fixes:** XY progressive lag (ProScan R-ack accumulation), XY stop (VS 0,0 on jog stop), well plate calibration persistence (MosaicCalibrator.py), CSS overhead (state-change guards). **Features:** Per-camera objective selector (ObjectiveCalibration.py + objectives.json), Xbox trigger creep fix, Xbox debug mode, calibration click-to-move, safe navigation Z-wait (flush_moves M400 + PositionPoller suspend/resume) |
| v7.3.5 | **Bug fixes:** Safe Z serial buffer drain (flush_moves drains stale "ok" before M400 — prevents premature XY start during Z retract), click-to-move event filter (QLabel event filter bypasses unreliable PySide6 propagation), XY unit system overhaul (xy_position_scale replaces microsteps_per_micron, scale 10→1, 15-file rename). **Features:** ZP Stage settings card (retract/insert/jog feedrates + EEPROM save toggle), watchdog periodic M500 (ConnectionWatchdog.add_periodic), simulation state persistence (ZP EEPROM to JSON via M500/M501/M503 + XY position to JSON on stop) |
| v7.3.6 | **DPI-aware GUI scaling:** `gui/scaling.py` with `s()`, `sf()`, `scale_factor()` functions; `build_theme(k)` parameterized QSS generation replacing static `DARK_THEME` string; all 20+ GUI files updated to wrap hardcoded pixel/font values. **PyInstaller packaging:** `MEBP.spec` + `build_exe.py` for standalone `--onedir` executable; `multiprocessing.freeze_support()`; `os.chdir(sys._MEIPASS)` for frozen path resolution; transitive dep exclusion (2.9GB → 470MB). **Misc:** `.gitignore` for build artifacts, `QT_AUTO_SCREEN_SCALE_FACTOR=1` replaces `QT_FONT_DPI=96` lock. |

---

## 14. Dependencies

### Python Packages (pip)

```
PySide6>=6.5          # Qt 6 GUI framework
pyserial>=3.5         # RS-232 serial communication
pygame>=2.5           # Xbox controller input
numpy>=1.24           # Numerical computation
scipy>=1.10           # Scientific functions (CubicSpline interpolation)
opencv-python>=4.8    # Camera image processing (optional)
```

### Packaging (optional)

```
pyinstaller>=6.0     # Standalone executable builder (optional, for distribution)
```

### System Requirements

- Python 3.10+
- macOS, Windows, or Linux
- USB-to-Serial adapter (for real hardware)
- Xbox controller (optional, for jogging)
- ToupTek camera (optional, for calibration)
