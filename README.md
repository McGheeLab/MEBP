# MEBP — Multi-Extrusion Bioprinting Platform

A desktop application for controlling laboratory-scale bioprinting hardware. MEBP orchestrates Prior ProScan XY stages, Marlin-based Z-axis and syringe pump controllers, and Hamilton syringe systems to precisely deposit biological materials into standard well plates.

**Version 7.5.0** | Python 3.10+ | PySide6 (Qt 6)

> **v7.5.0** introduces the **Print Builder** mode page — a dedicated home for
> *authoring* print trajectories, inserted in the sidebar before *Printing*.
> Its flagship **Sketch** tool lets you draw a print directly from vector
> primitives (line / rect / circle / ellipse / polygon) with paint-bucket fill
> of enclosed regions, multi-layer Z-stacks, object/border snapping, and a
> per-shape printed bead width — all driven by the active needle's diameter.
> The compiled toolpath renders as the **main view** (colored by pump,
> dashed-grey travel); your shapes are a slim editable overlay on top. A
> dashed **standard well** outline is drawn at the origin for scale. Output
> bakes through the existing `csv_import` contract so it shows up in Print
> Setup's custom-prints area alongside the legacy image-stack importer
> (relocated as the *Image Import* sub-page). Bundles the staged v7.4.3–v7.4.8
> increments — see
> [`coding plans/Update plans/MEBP_v75x_PRINT_BUILDER.md`](coding%20plans/Update%20plans/MEBP_v75x_PRINT_BUILDER.md)
> and
> [`coding plans/Architectures/ARCHITECTURE_V750.md`](coding%20plans/Architectures/ARCHITECTURE_V750.md).

---

## Features

- **Multi-material printing** — Up to 3 independent syringe pumps with multi-ink per pump, configurable ink swap strategies, and per-ink gather configs
- **Parametric object designer** — 1D (point), 2D (line, circle, square, triangle, spiral, ellipse), and 3D (sphere, cube, cylinder, ellipsoid) with shell/solid and fill pattern options
- **Image-based toolpaths** — Import TIFF stacks, image sequences, or single images to generate raster toolpaths with per-pump intensity mapping
- **Well plate support** — Standard 6, 12, 24, 48, 96, and 384-well plates with per-well object assignment
- **Real-time jogging** — Xbox controller support for manual stage positioning with configurable button mapping, well plate navigator for one-click fast-travel to any well
- **Well plate calibration** — SVD Procrustes similarity-transform registration from 2+ manually-taught well positions; calibrated positions persist across restarts and sync to the jog page navigator automatically
- **Z-plane calibration** — 3-point teach + least-squares plane fit for automatic tilt compensation
- **Camera alignment** — ToupTek or USB camera integration for visual plate registration; per-camera objective selector (1x-100x) with theoretical and empirically-measured um/px display; click-to-move on live camera feed
- **Objective calibration** — Per-camera, per-objective um/px calibration stored in `config/hardware/objectives.json`; empirical stage-motion calibration via `PixelCalibrationDialog`
- **Pick & Place mode** — Config-first workflow with live camera target overlays, auto-queue building, and execution monitoring
- **DPI-aware GUI** — Scales correctly on 1080p, 1440p, 4K, Retina, and ultrawide displays; all dimensions parameterized via `gui/scaling.py`
- **Hardware simulation** — Physics-based XY and Z/pump simulators with state persistence for offline development and testing
- **Print monitoring** — Real-time XY path visualization, Z cross-section view, and per-pump volume tracking
- **Post-print analysis** — Planned vs. actual path comparison, error time series, and statistics
- **Crash recovery** — Print state persisted to disk for resumption after unexpected interruption
- **Persistent print files** — Save, load, and manage print designs with schema migration across versions
- **Standalone executable** — PyInstaller packaging for distribution without Python environment

---

## Hardware

| Component | Supported Models |
|-----------|-----------------|
| XY Stage | Prior ProScan II, ProScan III (RS-232, 38400 baud) |
| Z/Pump Board | Marlin-compatible 3D printer board (RS-232, 115200 baud, G-code) |
| Syringes | Hamilton (25-1000 uL catalog included) |
| Needles | Standard hypodermic 16G-32G |
| Controller | Xbox (USB/Bluetooth, via pygame) |
| Camera | ToupTek (DLL), OpenCV-compatible USB cameras |

All hardware is optional — the application runs in full simulation mode by default.

---

## Installation

### Requirements

- Python 3.10 or later
- pip (Python package manager)
- macOS, Windows, or Linux

### Install Dependencies

```bash
pip install PySide6 pyserial pygame numpy scipy opencv-python
```

Or install from a requirements file:

```bash
pip install -r requirements.txt
```

#### `requirements.txt`

```
PySide6>=6.5
pyserial>=3.5
pygame>=2.5
numpy>=1.24
scipy>=1.10
opencv-python>=4.8
```

> **Note:** `opencv-python` is only required for camera-based calibration. If you don't need camera features, you can omit it.

### Platform Notes

- **macOS**: Xbox controller uses thread mode (not subprocess) due to macOS Bluetooth restrictions. Install pygame with `pip install pygame`.
- **Windows**: USB-to-Serial drivers may be required for FTDI/Prolific adapters.
- **Linux**: Add your user to the `dialout` group for serial port access: `sudo usermod -aG dialout $USER`

---

## Quick Start

### Simulation Mode (No Hardware)

```bash
python main.py
```

Launches the full GUI with simulated XY and Z/pump stages.

### Real Hardware

```bash
python main.py --real-xy --real-zp
```

Connects to physical Prior ProScan XY stage and Marlin Z/pump board via auto-detected serial ports.

### Headless Mode (Xbox Only)

```bash
python main.py --headless --real-xy --real-zp
```

Xbox controller jogging without the GUI — useful for manual positioning.

### Standalone Executable

Build a standalone executable (no Python required to run):

```bash
python build_exe.py --clean
```

Run the built executable:

```bash
./dist/MEBP/MEBP                                          # macOS/Linux
dist\MEBP\MEBP.exe                                        # Windows
./dist/MEBP/MEBP --simulate-xy --simulate-zp --headless   # headless mode
```

### All Options

```
python main.py [OPTIONS]

  --real-xy          Connect to real XY stage (default: simulate)
  --real-zp          Connect to real Z/pump board (default: simulate)
  --simulate-xy      Force XY simulation even if hardware detected
  --simulate-zp      Force Z/pump simulation even if hardware detected
  --headless         Xbox controller only, no GUI
  --settings FILE    Path to settings JSON file (default: settings.json)
  --verbose, -v      Enable debug logging
```

---

## DPI Scaling

The GUI automatically detects screen DPI and scales all elements proportionally. On standard 96-DPI screens, the UI renders at its baseline size. On high-DPI screens (1440p, 4K, Retina), all buttons, fonts, panels, and spacing scale up.

To manually override the scale factor:

```bash
MEBP_UI_SCALE=1.5 python main.py    # Force 150% scaling (simulates 4K on 1080p)
MEBP_UI_SCALE=2.0 python main.py    # Force 200% scaling
```

The scale factor floor is 1.0 — the UI never shrinks below its 96-DPI design size.

---

## Usage

### 1. Hardware Setup (Page 0)

Configure your hardware before accessing other pages:

1. **Controller** — Select ProScan II or III (auto-detected from serial ports)
2. **Needle** — Choose gauge and number of channels
3. **Pumps** — For each pump (P1/P2/P3):
   - Select Hamilton syringe from catalog
   - Check one or more inks from the ink library
   - Set fluid column volumes (oil, buffer, ink)
4. **Ink Library** — Define ink specifications (name, viscosity, density, color)

### 2. Dashboard (Page 1)

Connect to XY stage, Z/pump board, and Xbox controller. Monitor real-time position and connection status.

### 3. Jog Control (Page 2)

Move stages manually using on-screen buttons or Xbox controller. Configure jog speeds and button mapping.

### 4. Calibration (Page 3)

Align your well plate and set up the camera:

**Manual XY training:**
1. Jog to any 2+ wells, teach each position
2. Press **Train** — SVD Procrustes fit corrects all predicted well positions
3. Calibrated positions are auto-saved and pushed to the Jog page navigator

**Auto-calibration (camera required):**
- 3-well auto-scan with Procrustes SVD fit
- Auto Z-bottom calibration via focus sweep

**Camera settings:**
- Per-camera objective selector (1x-100x) with live um/px readout
- Run **Calibrate um/px** to measure empirically via stage-motion correlation
- Calibrated values saved to `config/hardware/objectives.json`

**Z-plane:**
- 3-point teach + least-squares plane fit for automatic tilt compensation

### 5. Print Setup (Page 4)

Design your print across four tabs:

- **Workspace** — Review hardware configuration summary
- **Objects** — Add parametric print objects (circles, spheres, etc.) with fill options and ink assignment
- **Wells** — Assign objects to well plate positions and set well roles
- **Finalize** — Configure execution parameters (ink swap strategy, Z/XY travel, ink gather, cleanup), generate plan of action, and send to monitor

### 6. Print Monitor (Page 5)

Execute prints with real-time visualization:
- Plate overview with per-well progress
- XY path overlay (planned vs. actual)
- Z-height cross-section
- Per-pump volume gauges
- Pause, resume, and abort controls

### 7. Print Results (Page 6)

Post-print analysis:
- Path comparison (planned vs. recorded)
- Error time series
- Statistical summary
- Playback controller for reviewing print execution

---

## Project Structure

```
MEBP/
├── main.py                    # Application entry point
├── build_exe.py               # PyInstaller build script
├── MEBP.spec                  # PyInstaller spec file
├── settings.json              # Persistent configuration
├── SupportClasses/            # Backend (hardware, geometry, printing)
│   ├── StageController.py     # Hardware orchestrator
│   ├── HardwareConfig.py      # Central uL<->mm configuration
│   ├── GeometryEngine.py      # Parametric trajectory generation
│   ├── PrintManager.py        # Print execution engine
│   ├── PrintPlanOfAction.py   # Plan generation + PrintExecutionConfig
│   ├── ImagePathPlanner.py    # Image-to-toolpath raster generator
│   ├── SketchTrajectory.py    # v7.5.0 Print Builder Sketch model + compiler + paint-bucket
│   └── ...
├── gui/                       # Frontend (PySide6)
│   ├── app.py                 # Main window (7 main pages: HW · Cal · Jog · Print Builder · Printing · Workflows · Settings)
│   ├── scaling.py             # DPI-aware scaling utility
│   ├── styles.py              # Catppuccin Mocha theme (parameterized)
│   ├── pages/                 # Application pages (incl. print_builder.py + print_builder_sketch.py)
│   └── widgets/               # Custom Qt widgets (incl. sketch_canvas.py)
├── config/                    # JSON configuration files
│   ├── controllers/           # Stage protocol definitions
│   ├── hardware/              # Needle/syringe/ink catalogs + objectives.json
│   └── prints/                # Saved print designs
├── tests/                     # Test suite
└── coding plans/              # Architecture docs, update plans, archived READMEs
```

---

## Configuration

### Settings

Application settings are stored in `settings.json` and auto-saved on exit. Key sections:

- `simulation` — XY/ZP simulation flags
- `speeds` — Default jog speeds
- `safety_limits` — Software endstops and max rates
- `hardware_config` — Full hardware configuration
- `calibration` — Taught points and Z-plane coefficients
- `zp_stage` — Z/pump feedrates and EEPROM save settings

### Controller Protocols

XY stage commands are defined in JSON files under `config/controllers/`. Adding support for a new controller requires only a new JSON file — no code changes.

### Hardware Catalogs

Needle gauges (`config/hardware/needles.json`) and Hamilton syringes (`config/hardware/syringes.json`) are loaded from JSON catalogs. Add custom entries by editing these files.

---

## Development

### Running Tests

```bash
python -m pytest tests/
```

Or individual test files:

```bash
python -m unittest tests/test_v726_print_execution.py
```

### Simulation Testing

The physics-based simulators (`XYStageSimulator`, `ZPStageSimulator`) provide deterministic testing without hardware. All tests can run in simulation mode.

### Building the Executable

```bash
python build_exe.py --clean          # Clean build
python build_exe.py --debug          # Debug build (console visible)
```

The executable is output to `dist/MEBP/`. Bundle size is approximately 470 MB (large transitive dependencies like TensorFlow and PyTorch are excluded).

### Architecture Documentation

See `coding plans/Architectures/ARCHITECTURE_V737.md` for the comprehensive architecture reference, then the v7.4.0 deltas:
- `coding plans/Architectures/ARCHITECTURE_V740A.md` — foundation (components, transitions, calibration auto-save)
- `coding plans/Architectures/ARCHITECTURE_V740B.md` — workflow restructure (HW Setup sub-pages, Settings slim, migrations)
- `coding plans/Architectures/ARCHITECTURE_V740C.md` — onboarding (wizard, help mode, invalidation banner)
- `coding plans/Architectures/ARCHITECTURE_V741.md` — device profiles (per-machine reusable settings)
- `coding plans/Architectures/ARCHITECTURE_V742.md` — initial device setup workspace (connect, jog/limits, axis mapping, stepper cal)

---

## License

Proprietary — research use only.

---

## Acknowledgments

Built for laboratory bioprinting research. Uses Prior Scientific ProScan stage controllers and Hamilton precision syringes.
