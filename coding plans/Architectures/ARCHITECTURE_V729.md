# MEBP v7.3.4 — Architecture Reference

**Multi-Extrusion Bioprinting Platform**
**Version 7.3.4 | March 2026**

---

## 1. System Overview

MEBP is a desktop application for controlling laboratory bioprinting equipment. It orchestrates Prior ProScan XY stages, Marlin-based Z-axis and pump controllers, and Hamilton syringe systems to precisely deposit biological materials into well plates. The platform supports multi-material printing with various ink types (hydrogels, cell suspensions, granular materials) using multi-channel needles and sophisticated trajectory planning.

The application is written in Python with PySide6 for the GUI, using RS-232 serial communication for hardware control and JSON for all configuration and data persistence. An Xbox controller provides real-time jogging capability. The interface follows a Catppuccin Mocha dark theme.

### Core Design Principles

1. **Hardware abstraction** — GUI pages never touch serial ports. All hardware access goes through `StageController` or `PrintManager`.
2. **JSON protocol abstraction** — XY stage commands are defined in JSON protocol files (`config/controllers/`), enabling support for different controller models (ProScan II vs III) without code changes.
3. **Hardware-first configuration** — Page 0 (Hardware Setup) must be completed before any other page is accessible. `HardwareConfig` is the single source of truth for all µL ↔ mm conversions.
4. **µL-native pump control** — All user-facing pump values are in microliters. Conversion to mm (for Marlin firmware) happens exclusively inside `StageController.move_pump_uL()` via `SyringeSpec`.
5. **Safety-layered architecture** — Software endstops, flow-rate clamping, crash recovery, and thread-safe patterns throughout.
6. **Persistent print files** — Print designs stored as JSON in `config/prints/` with auto-save, schema migration, and full CRUD management.
7. **Multi-ink per pump** — Each pump can be assigned multiple inks with configurable swap strategies for single-syringe multi-material workflows.

---

## 2. Technology Stack

| Layer | Technology |
|-------|-----------|
| Language | Python 3.10+ |
| GUI Framework | PySide6 (Qt 6) |
| Theme | Catppuccin Mocha dark theme (QSS) |
| Serial Communication | pyserial (RS-232) |
| Numerical Computation | NumPy, SciPy |
| Computer Vision | OpenCV (well/needle detection, focus scoring, calibration) |
| Input Devices | pygame (Xbox controller via multiprocessing) |
| Configuration | JSON (hardware, controllers, prints, settings) |
| Testing | unittest + physics-based simulators |

---

## 3. Folder Structure

```
MEBP/
├── main.py                              # Entry point — CLI arg parsing, mode selection
├── settings.json                        # Persistent app config (auto-saved)
├── print_resume.json                    # Crash recovery state
├── current_button_mapping.json          # Xbox controller mapping (hot-reloadable)
├── Default.json                         # Default configuration template
├── INSTRUCTIONS.md                      # User operating guide
├── README.md                            # GitHub landing page
│
├── config/
│   ├── controllers/
│   │   ├── proscan_ii.json              # ProScan II command protocol map
│   │   └── proscan_iii.json             # ProScan III command protocol map
│   ├── hardware/
│   │   ├── needles.json                 # Needle gauge catalog (16G–32G)
│   │   ├── syringes.json                # Hamilton syringe catalog (25–1000 µL)
│   │   ├── objectives.json              # Per-camera objective µm/px calibrations  ← v7.3.4
│   │   └── Standard Bioprinting Setup.json  # Example HardwareConfig
│   └── prints/                          # Persistent print file storage
│       ├── PIAR.json                    # Saved print designs
│       ├── Scaffold_v1.json
│       └── ...
│
├── SupportClasses/                      # Backend — zero GUI dependencies
│   ├── __init__.py                      # Package exports + module listing
│   ├── Processor.py                     # Thread-safe command bus (pub/sub dispatch)
│   ├── SerialUtils.py                   # Retry decorator, safe I/O, port discovery, watchdog
│   ├── XYStage.py                       # Prior ProScan XY stage manager
│   ├── XYStageSimulator.py              # Physics-based XY simulator
│   ├── ZPStage.py                       # Marlin Z/pump board manager (G-code)
│   ├── ZPStageSimulator.py              # Physics-based Z/pump simulator
│   ├── XboxController.py                # Polling worker (multiprocessing.Process)
│   ├── StageController.py               # Top-level hardware orchestrator
│   ├── ControllerProtocol.py            # JSON-based command protocol loader
│   ├── PrintManager.py                  # Print execution engine + trajectory tracking
│   ├── PrintPlanOfAction.py             # Print plan generation + command sequencing
│   ├── PrintRecorder.py                 # Auto-record prints, replay data
│   ├── PrintHistory.py                  # Persistent print history log
│   ├── PrintFileManager.py              # Print file CRUD + schema migration
│   ├── WellPlate.py                     # ANSI/SLAS plate geometry (6–384 well)
│   ├── WellSetup.py                     # Well assignments, rosettes, plane fitting
│   ├── SafetyLimits.py                  # Software endstops + flow-rate clamping
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
│   └── ObjectiveCalibration.py          # Per-camera objective µm/px store      ← v7.3.4
│
├── gui/                                 # Frontend — PySide6 / Qt 6
│   ├── __init__.py
│   ├── app.py                           # MainWindow + page routing + signal bridge
│   ├── styles.py                        # Catppuccin Mocha QSS theme
│   ├── ui_functions.py                  # Animation helpers
│   ├── unit_helpers.py                  # Unit conversion utilities
│   ├── pages/
│   │   ├── hardware_setup.py            # Page 0: Hardware configuration
│   │   ├── dashboard.py                 # Page 1: Status overview + connections
│   │   ├── jog_control.py              # Page 2: Manual jogging + Xbox control
│   │   ├── calibration.py              # Page 3: Camera alignment + Z-plane
│   │   ├── print_setup.py              # Page 4: Print setup wrapper (3 tabs)
│   │   ├── print_workspace.py          # Tab 1: Hardware summary
│   │   ├── print_objects.py            # Tab 2: Object designer
│   │   ├── print_well_setup.py         # Tab 3: Well assignments
│   │   ├── print_monitor.py            # Page 5: Print execution monitor
│   │   ├── print_results.py            # Page 6: Post-print analysis
│   │   ├── settings_page.py            # Page 7: Safety/serial/simulation
│   │   └── helper_functions.py         # Utility page
│   └── widgets/
│       ├── console_log.py              # Logging viewer + Qt log handler
│       ├── camera_widget.py            # OpenCV camera feed
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
    │   └── ARCHITECTURE_V729.md         # ← This document
    ├── Readmes/
    └── Update plans/
        └── MEBP_v728_to_v729_UPDATE.md
```

---

## 4. Architecture Layers

### 4.1 Hardware Abstraction Layer

```
StageController (top-level orchestrator)
├── Processor (thread-safe command bus — FIFO queue, pub/sub)
├── XYStageManager  ──or──  XYStageSimulator
│   └── ControllerProtocol (JSON command definitions)
├── ZPStageManager  ──or──  ZPStageSimulator
├── XboxQueuePoller (multiprocessing.Queue → Processor dispatch)
├── XYJogHandler (continuous velocity jogging)
├── ZPJogHandler (velocity → relative move conversion)
├── PositionPoller (background position caching)
├── SafetyLimits (software endstops for all axes)
└── PositionLogger (timestamped motion recording)
```

**Key Properties:**
- GUI pages call `StageController` methods — never raw serial
- Simulator vs. hardware selected at startup via CLI flags
- Xbox controller runs in a separate `multiprocessing.Process` for latency isolation
- `Processor` serializes all device commands through a single FIFO queue

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
│                    GATHER_INK, FINAL_CLEANUP, TRAVEL_XY, MOVE_SAFE_Z, INK_SWAP  ← v7.2.9
├── PrintExecutionConfig (comprehensive execution config)  ← v7.2.9 replaces PlanPreferences
│   ├── InkSwapStrategy (6-step cleaning sequence)  ← moved from HardwareConfig
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
4. Finalize Tab (Page 4, Tab 4) → configure PrintExecutionConfig → generate PrintPlanOfAction → build PrintJob  ← v7.2.9
5. Print Monitor (Page 5) → execute via PrintManager
6. Print Results (Page 6) → analyze recorded data

### 4.3 Configuration Layer

```
HardwareConfig (central authority for µL↔mm conversions)
├── NeedleSpec (gauge, OD/ID, channel count)
├── PumpChannelConfig × 3 (P1, P2, P3)
│   ├── SyringeSpec (volume, stroke_length_mm, barrel_id_mm)
│   ├── inks: list[InkSpec]  ← v7.2.9: multi-ink per pump
│   ├── FluidColumn (oil_uL, buffer_uL, ink_uL)
│   └── enabled: bool
├── InkLibrary: dict[str, InkSpec]
│   └── InkSpec (name, viscosity, granule_diameter, density, color)
├── InkSwapStrategy  ← v7.2.9: re-exported from PrintPlanOfAction for backward compat
│   ├── 6 step toggles: waste, wash_pre, buffer, wash_post, ink_load, wash_final
│   └── 4 volumes: waste_volume_uL, wash_volume_uL, buffer_volume_uL, ink_load_volume_uL
└── RosetteInsert (rosette geometry for guided deposition)
```

**Serialization:** `HardwareConfig.to_dict()` / `from_dict()` with backward compatibility:
- Old single `"ink"` key → new `"inks"` list (auto-migration in `PumpChannelConfig.from_dict()`)
- Version field tracks schema version

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
│       └── generate_triangular_meander_fill()  ← v7.2.9
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
├── QStackedWidget (page router)
│   ├── Page 0: HardwareSetupPage      — needle, pumps, inks, ink swap strategy
│   ├── Page 1: DashboardPage           — connection status, position readouts
│   ├── Page 2: JogControlPage          — manual jogging, Xbox mapping, well plate navigator
│   ├── Page 3: CalibrationPage         — 3-well auto-cal, manual SVD training, auto Z-bottom, objective µm/px, click-to-move  ← v7.3.4
│   ├── Page 4: PrintSetupPage          — 4-tab print designer  ← v7.2.9
│   │   ├── Tab 1: WorkspaceTab         — read-only hardware summary
│   │   ├── Tab 2: PrintObjectsTab      — CAD-like object editor
│   │   ├── Tab 3: WellSetupTab         — well assignments + roles
│   │   └── Tab 4: FinalizeTab          — execution config, plan of action, generate & send  ← v7.2.9
│   ├── Page 5: PrintMonitorPage        — execution visualization
│   ├── Page 6: PrintResultsPage        — post-print analysis
│   └── Page 7: SettingsPage            — safety limits, serial ports
├── Left Sidebar (navigation icons)
├── Top Bar (status dots: XY, ZP, Xbox + global position)
├── Context Panel (right-side, page-specific controls)
└── Console Log (bottom, collapsible)
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
- **Safe travel**: `safe_travel_to(x_um, y_um, safe_z_mm)` — suspends `PositionPoller`, sends Z move + `flush_moves` (M400), fast XY + `wait_for_xy_arrival`, optional Z descend + `flush_moves`, resumes poller. Guarantees physical Z completion before XY starts.
- **PositionPoller**: `suspend()`/`resume()` methods pause background hardware polling to prevent serial races during programmatic moves. `_suspended` flag checked at top of `_poll_loop`.

**Critical serial protocol note (v7.3.4):** The ProScan II/III stage responds with `R\r` to *every* command — including position queries (`P`), relative moves (`GR`), absolute moves (`G`), and velocity commands (`VS`). Every command that writes to the serial port **must** consume this `R\r` ack under `_serial_lock` (write + read atomically). Failure to read the ack causes byte accumulation in the RX buffer; the `PositionPoller` then reads stale `R` bytes instead of position responses, causing progressively worsening display lag. This was the root cause of the v7.3.4 XY lag bug fixed in `XYStage.move_stage_at_velocity()`.

### 5.2 HardwareConfig.py — Central Configuration

Single source of truth for hardware state. All µL↔mm conversions route through here.

**v7.2.9 Changes:**
- `PumpChannelConfig.inks: list[InkSpec]` — multi-ink per pump with backward-compat `ink` property
- `InkSwapStrategy` moved to `PrintPlanOfAction.py` (canonical location), re-exported here for backward compat
- `pump_ink_map` → `dict[str, list[str]]`, `ink_pump_map` → `dict[str, list[str]]`
- `can_handle_ink(ink_name)` for pump-ink resolution

### 5.6 PrintPlanOfAction.py — Plan Generation & Execution Config

Generates typed execution plans from well assignments and hardware configuration.

**v7.2.9 Changes:**
- `PrintExecutionConfig` replaces `PlanPreferences` as the comprehensive execution configuration
  - Composes: `InkSwapStrategy`, `InkGatherConfig`, `ZTravelConfig`, `XYTravelConfig`, `FinalCleanupConfig`
  - Per-ink overrides via `ink_gather_configs` dict with fallback to `default_gather_config`
  - Migration: `PrintExecutionConfig.from_preferences(old_prefs)` for backward compat
- `InkSwapStrategy` is now canonical here (moved from HardwareConfig)
- 5 new `PlanStepType` values: `GATHER_INK`, `FINAL_CLEANUP`, `TRAVEL_XY`, `MOVE_SAFE_Z`, `INK_SWAP`
- `PlanStep` enriched with: `z_behavior`, `travel_mode`, `wait_for_z_confirm`, `extra_percent`, `max_pickup_uL`, `sub_steps`
- `PLAN_STEP_COLORS` and `PLAN_STEP_ICONS` maps for Catppuccin Mocha visualization
- `PlanStep.icon` and `PlanStep.color` convenience properties

### 5.7 PrintTrajectoryPlanner.py — Plan-to-Trajectory

Converts `PrintPlanOfAction` into time-parameterized waypoint arrays.

**v7.2.9 Changes:**
- Handles all new step types: `GATHER_INK` (delegates to `_do_load_ink()`), `MOVE_SAFE_Z` (fast Z raise), `TRAVEL_XY` (fast XY move with speed override), `FINAL_CLEANUP` (sub-step sequence: waste/wash), `INK_SWAP` (compound: waste→wash→buffer→ink_load)
- Uses `PlanStep.sub_steps` list for ordered compound step execution

### 5.8 ImagePathPlanner.py — Image-to-Toolpath Generator

Converts image stacks into raster toolpaths for image-based printing.

**Features:**
- Three loading modes: TIFF stack, numbered image sequence, single image × N layers
- Per-pump greyscale intensity mapping to flow rates
- Raster toolpath generation with alternating layer direction
- Closest-point segment reordering between layers
- Output: Nx7 array `[x, y, z, p1, p2, p3, t]` (mm/s units)
- CSV export via `save_csv()`

### 5.9 Vision & Calibration System (v7.3.0 / v7.3.1 / v7.3.4)

```
VisionDetector.py
├── WellDetector (HoughCircles + contour fallback)
│   ├── detect_well() → DetectionResult (center_x, center_y, radius, confidence)
│   └── detect_well_with_fallback(frame, expected_diam_px, tolerance)
├── NeedleDetector (3-strategy: HoughCircles + contour + radial profile)
│   ├── detect_needle() → NeedleDetectionResult
│   └── compute_focus_score() → FocusResult (score: 70% Laplacian + 30% Tenengrad)
└── FocusTracker (temporal smoothing for focus assist display)

SimulatedCamera.py
├── Synthetic microscope renderer (well plate + needle)
├── DOF model: opacity = exp(-0.5*(defocus/dof_hw)²)
├── Blur model: sigma = normalized_defocus * BLUR_REFERENCE_UM / um_per_px
├── set_focal_z(z_mm) — sets focal plane (typically top_z - well_depth)
└── set_stage_position(x, y, z_mm) — updates camera viewpoint

MosaicBuilder.py
├── AffineCalibration (Procrustes SVD similarity transform)  ← auto-scan path
│   ├── correct_positions(predicted) → calibrated dict
│   ├── rotation_deg, scale, translation_um, center_um, rms_residual
│   └── is_identity — True if transform is essentially a no-op
└── Frame collection + composite stitching (legacy mosaic support)

MosaicCalibrator.py  ← v7.3.4: manual teaching path
├── add_point(pred_x, pred_y, meas_x, meas_y) — accumulate matched pairs
├── solve() — SVD Procrustes fit; requires ≥2 pairs; unique for 2-pt case
├── correct_positions(predicted_dict) → corrected dict
└── rms_error_um — RMS residual in µm after solve

ObjectiveCalibration.py  ← v7.3.4
├── ObjectiveCalibrationStore — load/save objectives.json
│   ├── get_calibration(camera_model, objective_name) → µm/px or None
│   └── set_calibration(camera_model, objective_name, um_per_px)
└── get_store() — lazy singleton accessor

Detection Pipeline (GUI)
├── DetectionWorker (QThread, pull-based frame consumer)
│   ├── DetectionMode: WELL_ONLY, NEEDLE_ONLY, BOTH, FOCUS_ASSIST
│   └── Emits: detection_result, focus_updated signals
└── DetectionOverlay (QPainter overlay with aspect-ratio-aware mapping)
```

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

**v7.2.9 Changes:**
- `generate_triangular_meander_fill()` — scan-line fill narrowing from base to apex
- Fixed square fill: corrected center+dimensions call to `generate_meander_fill()`
- Triangle filled dispatch with meander pattern
- Fill pattern parameter (`meander`/`spiral`) passed through from GUI

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
- `ConnectionWatchdog` — background disconnect detection
- Exception hierarchy: `SerialError` → `SerialDisconnectedError` | `SerialTimeoutError` | `SerialAccessError`
- `list_serial_ports()` — platform-aware port discovery

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
| Position unit | Microsteps (10 µsteps/µm) |

**Key commands** (from `config/controllers/proscan_iii.json`):
- `P` → query position → `x,y\r`
- `G x,y` → move absolute
- `GR dx,dy` → move relative
- `VS speed` → set velocity
- `SAS accel` → set acceleration
- `Z` → home (zero position)
- `I` → interrupt/stop

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
- `M400` → wait for all moves to complete (**used by `ZPStage.flush_moves()`** — the reliable Z-arrival signal in `safe_travel_to`; only returns `ok` after physical completion, not just planner acceptance)

### 6.3 Xbox Controller (pygame via multiprocessing)

- Runs in separate `multiprocessing.Process` for latency isolation
- Polls button/axis/DPAD events via `pygame.joystick`
- Events serialized as JSON dicts through `multiprocessing.Queue`
- `XboxQueuePoller` (thread in main process) dispatches events to `Processor` command bus
- Button mapping loaded from `current_button_mapping.json` (hot-reloadable)
- macOS: Uses thread mode (no subprocess) due to macOS Bluetooth restrictions
- **Trigger normalization (v7.3.4)**: Windows/Linux XInput triggers rest at `-1.0`; hardcoded offset `{4: -1.0, 5: -1.0}` normalizes to `0..1`. macOS triggers already `0..1`.
- **Heartbeat suppression (v7.3.4)**: `pygame.joystick.quit()/init()` every 3s causes triggers to transiently report `0.0`. `_accum_suppress_until` timestamp blocks axis accumulation for `avg_interval + 0.15s` after each reinit.
- **Debug mode (v7.3.4)**: `debug_mode` flag passed from Settings → Dashboard → `connect_xbox()` → subprocess kwargs. Gates: startup trigger diagnostic, periodic TRIG DIAG every 2s, per-dispatch log.

---

## 7. GUI Page Architecture

### Page 0: Hardware Setup (`hardware_setup.py`)

**Purpose:** Configure all hardware before other pages unlock.

**v7.2.9 Sections:**
1. **Controller Selection** — ProScan II/III auto-detect
2. **Needle Configuration** — gauge, channel count, channel-pump mapping
3. **Pump Channels** — P1/P2/P3 each with:
   - Syringe selection (Hamilton catalog)
   - Multi-ink checklist (`QListWidget` with checkable items) ← v7.2.9
   - Fluid column volumes (oil/buffer/ink)
4. **Ink Library** — define ink specs (name, viscosity, density, color)
5. **Rosette Configuration** — insert geometry for guided deposition

> **Note:** Ink Swap Strategy UI moved from Hardware Setup → Print Setup Finalize tab in v7.2.9 (execution config is per-print, not per-hardware).

**Output:** `HardwareConfig` instance propagated to all pages via `_propagate_hardware_config()`.

### Page 4, Tab 2: Print Objects (`print_objects.py`)

**Purpose:** CAD-like designer for print objects with interactive well preview.

**v7.2.9 Object Type System:**
```
OBJECT_TYPES (static dict):
├── 1D: Point
├── 2D: Line, Circle, Square, Triangle, Spiral, Ellipse
├── 3D: Sphere, Cube, Cylinder, Ellipsoid
└── Import: CSV Import

_3D_TYPE_MAP (gui_type, filled) → engine_type:
├── ("sphere", True) → "sphere_solid"
├── ("sphere", False) → "sphere_shell"
├── ("cube", True) → "cube_solid"
├── ... etc.
```

**v7.2.9 UI Elements:**
- Object type list (categorized: 1D/2D/3D/Import)
- Filled/Solid checkbox (hidden for point, line, spiral, csv_import)
- Fill pattern selector (Meander / Spiral) — visible when filled checked
- Ink selector (all library inks, no pump prefix)
- Per-type parameter forms (dynamic, from `_CONSOLIDATED_3D_PARAMS` or GeometryEngine)
- Interactive well preview (`DraggableObjectItem` on `InteractiveProjectionCanvas`)

**Type Resolution Flow:**
1. User selects "Sphere" from list → consolidated GUI type
2. User checks "Filled" → `filled=True`
3. `_build_print_object()` resolves: `_3D_TYPE_MAP[("sphere", True)]` → `"sphere_solid"`
4. GeometryEngine generates trajectory for `sphere_solid` type

### Page 4, Tab 3: Well Setup (`print_well_setup.py`)

**v7.2.9 Changes:**
- Plan of Action section **removed** from this tab (moved to Finalize tab)
- `_generate_plan()` still callable but invoked from Finalize tab with `execution_config` parameter

### Page 4, Tab 4: Finalize (`print_setup.py`)  ← v7.2.9 NEW

**Purpose:** Configure execution parameters, generate plan of action, and send to monitor.

**Layout:** Two-column design:
- **Left column — Print Parameters:** Ink swap strategy (6 toggles + 4 volume spinboxes), Z travel config, XY travel config, ink gather configs, final cleanup config
- **Right column — Plan of Action:** Generated plan preview with step-type colors/icons, generate button, send-to-monitor button

**Data flow:**
1. User configures `PrintExecutionConfig` sub-configs via UI widgets
2. "Generate Print" builds `PrintPlanOfAction` using execution config
3. `plan_to_commands()` → `PrintJob` with all service and motion steps
4. "Send to Monitor" emits `job_ready` signal → `PrintMonitorPage`

### Page 5: Print Monitor (`print_monitor.py`)

**Purpose:** Real-time visualization during print execution.

**Components:**
- `PlateOverviewWidget` — well plate with progress indicators
- `XYDetailView` — real-time XY path overlay (planned vs. actual)
- `YZSideView` — Z-height cross-section
- `SyringePumpWidget` — per-pump volume gauges
- Progress bar, ETA, state display
- Pause/Resume/Abort controls

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
    ├──→ PrintSetupPage.set_hardware_config()
    │       ├──→ WorkspaceTab.update_config()
    │       ├──→ PrintObjectsTab._refresh_ink_options_from_config()
    │       └──→ WellSetupTab.set_hardware_config()
    ├──→ PrintMonitorPage.set_hardware_config()
    └──→ Settings.save() (persisted to settings.json)
```

### 8.2 Print Job Execution

```
PrintObjectsTab (design objects)
    ▼
WellSetupTab (assign to wells)
    ▼
FinalizeTab (configure PrintExecutionConfig)  ← v7.2.9
    ├── InkSwapStrategy, ZTravelConfig, XYTravelConfig,
    │   InkGatherConfig, FinalCleanupConfig
    ▼
PrintSetupPage._generate_print()
    ├── PrintPlanOfAction.generate_plan(execution_config=...)
    │   └── Generates: GATHER_INK, TRAVEL_XY, MOVE_SAFE_Z,
    │                   PRINT, INK_SWAP, FINAL_CLEANUP steps
    └── plan_to_commands() → PrintJob
            ▼
PrintMonitorPage.receive_job()
    ▼
PrintManager.load_job() → start()
    ├── TrajectoryExecutor.execute()
    │   └── StageController.move_xy_absolute() / move_pump_uL()
    ├── ServiceSequenceExecutor.execute()
    │   └── waste→wash→buffer→ink swap cycles
    └── PrintRecorder.record_sample()
            ▼
PrintResultsPage.load_recording()
    └── PathComparisonWidget / ErrorTimeSeriesWidget
```

### 8.3 Ink Resolution (v7.2.9 Multi-Ink)

```
User selects "Hydrogel A" in PrintObjectsTab ink combo
    ▼
_resolve_ink_to_pump("Hydrogel A")
    ├── Iterates hw_config.pumps
    ├── Checks pcfg.can_handle_ink("Hydrogel A")
    │       └── Returns True if "Hydrogel A" in pcfg.ink_names
    └── Returns first matching pump_id (e.g., "P1")
            ▼
PrintObject.pump_id = "P1"
```

---

## 9. Key Algorithms

### 9.1 Microstep ↔ Physical Unit Conversion

```
ProScan: 10 microsteps per micron (from controller JSON)
  position_mm = microsteps / 10 / 1000
  microsteps = position_mm * 1000 * 10

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
2. For each well, compute approach position (center if well < 80% FOV, edge offset otherwise)
3. Navigate to approach, capture frame, run well detection via `WellDetector`
4. Convert pixel offset to stage coords using `pixel_offset_to_stage_um()`
5. Fit Procrustes SVD similarity transform from (predicted, detected) pairs
6. Apply `AffineCalibration.correct_positions()` to all predicted positions

### 9.5 Auto Z-Bottom Calibration (v7.3.1)

Focus-sweep algorithm to find well bottom Z:
1. **Coarse sweep** (0.15mm steps): Lower needle from safe Z, capture focus score at each step. Baseline score captured at approach. Detect needle entry when score > 2× baseline. Hard safety floor at `top_z - well_depth_mm`
2. **Fine sweep** (0.03mm steps): Track peak focus score. Stop immediately on 3 consecutive declining steps
3. Repeat for 3 calibration wells, fit Z-plane: `z = ax + by + c`

Focus scoring: `NeedleDetector.compute_focus_score()` — 70% Laplacian variance + 30% Tenengrad (Sobel gradient)

### 9.6 Shell vs. Solid 3D Generation

- **Shell**: Generate shape perimeter at each Z layer (stacked outlines)
- **Solid**: Generate filled cross-section at each Z layer (meander fill per slice)
- Layer height configurable, number of points per layer configurable

---

## 10. Serialization & Persistence

### 10.1 Settings (settings.json)

Top-level keys: `window`, `simulation`, `speeds`, `zero_position`, `print_settings`, `xbox`, `logging`, `safety_limits`, `polling`, `calibration`, `controller`, `workspace`, `hardware_config`, `motion_controller`, `ui`, `execution`

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
      "syringe": { "name": "Hamilton 250µL", "volume_uL": 250, ... },
      "inks": [
        { "name": "Hydrogel A", "viscosity_mPas": 50, "color": "#4CAF50" },
        { "name": "MSC Cells", "viscosity_mPas": 5, "color": "#2196F3" }
      ],
      "fluid_column": { "oil_uL": 50, "buffer_uL": 20, "ink_uL": 100 }
    }
  },
  "ink_library": { ... },
  "ink_swap_strategy": {
    "waste": true, "wash_pre": true, "buffer": true,
    "wash_post": true, "ink_load": true, "wash_final": true,
    "waste_volume_uL": 50, "wash_volume_uL": 100,
    "buffer_volume_uL": 100, "ink_load_volume_uL": 50
  }
}
```

### 10.3 Print Files (config/prints/*.json)

```json
{
  "name": "Scaffold_v1",
  "version": "7.2",
  "settings": { "feedrate_xy": 5.0, "feedrate_z": 1.0, "layer_height": 0.2 },
  "objects": [
    {
      "name": "Base Layer",
      "object_type": "cube_solid",
      "params": { "side": 10, "height": 0.2, "filled": true, "fill_pattern": "meander" },
      "position": { "x": 0, "y": 0, "z": 0 },
      "ink_name": "Hydrogel A"
    }
  ],
  "wells": { "A1": { "role": "print", "objects": ["Base Layer"] } }
}
```

---

## 11. Thread Safety & Concurrency

| Component | Threading Model | Safety Mechanism |
|-----------|----------------|-----------------|
| StageController | Main thread + PositionPoller thread | Processor command bus (FIFO queue); PositionPoller suspended during `safe_travel_to` to prevent serial races |
| XboxController | Separate Process (multiprocessing) | Queue-based event passing |
| XboxQueuePoller | Daemon thread in main process | Dispatches to Processor; zeros jog velocities on disconnect |
| PrintManager | Daemon thread for execution | Qt Signals (QueuedConnection) for GUI updates |
| SerialUtils | Called from various threads | `retry_serial` decorator, `ConnectionWatchdog` |
| GUI | Main thread only (Qt requirement) | Signal/slot for cross-thread updates |

---

## 12. Error Handling

### Serial Communication
- 3-retry with exponential backoff (`retry_serial` decorator)
- `ConnectionWatchdog` detects disconnects and emits status changes
- Graceful degradation: lost connection → red status dot → reconnect prompt

### Print Execution
- `PrintManager` state machine: RUNNING → ERROR on unrecoverable failures
- `print_resume.json` captures crash state for recovery
- Safety limits enforced at every motion command

### GUI
- All hardware calls wrapped in try/except with console log output
- Status dots (top bar) reflect real-time connection state
- Page gating prevents access to features without valid config

---

## 13. Version History (v7.2.x)

| Version | Key Changes |
|---------|------------|
| v7.2.3 | Hardware Setup page, print file persistence, print monitor redesign |
| v7.2.4 | Pump channel mapping, ink assignment, HardwareConfig system |
| v7.2.5 | Initial multi-pump UI, ink exclusion logic |
| v7.2.6 | Print execution restoration (7 critical fixes) |
| v7.2.7 | Print speed display, execution mode selection |
| v7.2.8 | Xbox controller polling fixes, ProScan detection, stable build |
| v7.2.9 | Object type consolidation, filled/shell checkbox, multi-ink per pump, ink swap strategy, triangle fill, PrintExecutionConfig (replaces PlanPreferences), 5 new step types (GATHER_INK, FINAL_CLEANUP, TRAVEL_XY, MOVE_SAFE_Z, INK_SWAP), Finalize tab (Tab 4), InkSwapStrategy relocated to PrintPlanOfAction, ImagePathPlanner for image-based toolpaths |
| v7.3.0 | Autocalibration: VisionDetector (well/needle detection, 3-strategy needle, focus scoring), SimulatedCamera (Gaussian DOF model), DetectionOverlay + DetectionWorker QThread, camera integration (BUC3D-1000C ToupTek), 59 vision/camera tests |
| v7.3.1 | Calibration overhaul: geometry-predicted wells, 3-well auto-calibration (Procrustes SVD), auto Z-bottom calibration (focus-sweep), simplified wizard (removed manual Teach A1/Corner), per-well 50% overlap scanning, jog page well plate navigator, MosaicBuilder affine engine, 110 tests total |
| v7.3.2 | QoL upgrades: camera config persistence, jog page startup well plate, axis flip checkboxes (Z + pumps), custom jog step sizes + absolute goto, calibration page restructure (steps to main, configurable cameras, deprecate needle zero), print monitor camera overlay, settings page manual zero calibration, Xbox stick zero calibration, safe travel Z-wait standardization |
| v7.3.3 | Mode-based navigation restructure (ModePage base class, right-side icon sub-nav), Printing mode container (wraps print setup/monitor/results/helpers), Pick & Place mode (PickAndPlaceManager backend, ImageStitcher, target selection with camera overlays, operation queue auto-building, execution monitoring), CameraManager shared across pages, TargetOverlayCameraView (live feed target overlays), PPOperationSetupPage (config-first workflow), camera µm/px empirical calibration (stage-move phase correlation dialog), relaxed needle detection with interactive edge refinement + accept/reject UI, needle-based µm/px calibration bridge |
| v7.3.4 | **Bug fixes:** XY progressive lag during Xbox jogging (ProScan R-ack accumulation in VS command), XY stage does not stop when joystick returns to deadzone (VS 0,0 not sent on stop), well plate calibration not persistent (MosaicCalibrator.py created, save/load race fixed), per-tick setStyleSheet overhead (state-change guards), MosaicCalibrator module missing. **New features:** Per-camera objective selector with theoretical + empirical µm/px (ObjectiveCalibration.py + objectives.json), Xbox trigger creep fix (platform-based rest values + `_accum_suppress_until` heartbeat suppression + velocity zeroing on disconnect), Xbox debug mode (settings toggle propagated to subprocess; gates TRIG DIAG / dispatch / velocity logs), calibration page click-to-move (Click→Move toggle + `_on_feed_clicked` handler via `CameraFeedView.clicked`), safe navigation Z-wait reliability (`ZPStage.flush_moves` M400, `PositionPoller.suspend/resume`, `safe_travel_to` uses M400 instead of M114 polling) |

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

### System Requirements

- Python 3.10+
- macOS, Windows, or Linux
- USB-to-Serial adapter (for real hardware)
- Xbox controller (optional, for jogging)
- ToupTek camera (optional, for calibration)
