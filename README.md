# MEBP — Multi-Extrusion Bioprinting Platform

A desktop application for controlling laboratory-scale bioprinting hardware. MEBP orchestrates Prior ProScan XY stages, Marlin-based Z-axis and syringe pump controllers, and Hamilton syringe systems to precisely deposit biological materials into standard well plates.

**Version 7.2.9** | Python 3.10+ | PySide6 (Qt 6)

---

## Features

- **Multi-material printing** — Up to 3 independent syringe pumps with per-pump ink assignment and configurable ink swap strategies
- **Parametric object designer** — 1D (point), 2D (line, circle, square, triangle, spiral, ellipse), and 3D (sphere, cube, cylinder, ellipsoid) with shell/solid and fill pattern options
- **Well plate support** — Standard 6, 12, 24, 48, 96, and 384-well plates with per-well object assignment
- **Real-time jogging** — Xbox controller support for manual stage positioning with configurable button mapping
- **Z-plane calibration** — 3-point teach + least-squares plane fit for automatic tilt compensation
- **Camera alignment** — ToupTek or USB camera integration for visual plate registration
- **Hardware simulation** — Physics-based XY and Z/pump simulators for offline development and testing
- **Print monitoring** — Real-time XY path visualization, Z cross-section view, and per-pump volume tracking
- **Post-print analysis** — Planned vs. actual path comparison, error time series, and statistics
- **Crash recovery** — Print state persisted to disk for resumption after unexpected interruption
- **Persistent print files** — Save, load, and manage print designs with schema migration across versions

---

## Hardware

| Component | Supported Models |
|-----------|-----------------|
| XY Stage | Prior ProScan II, ProScan III (RS-232, 38400 baud) |
| Z/Pump Board | Marlin-compatible 3D printer board (RS-232, 115200 baud, G-code) |
| Syringes | Hamilton (25–1000 µL catalog included) |
| Needles | Standard hypodermic 16G–32G |
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
5. **Ink Swap Strategy** — Configure cleaning steps for multi-ink single-syringe workflows

### 2. Dashboard (Page 1)

Connect to XY stage, Z/pump board, and Xbox controller. Monitor real-time position and connection status.

### 3. Jog Control (Page 2)

Move stages manually using on-screen buttons or Xbox controller. Configure jog speeds and button mapping.

### 4. Calibration (Page 3)

Align your well plate using 3-point teach:
1. Jog to well A1, teach position
2. Jog to opposite corner, teach position
3. Jog to third point, teach position
4. System fits a Z-plane for automatic tilt compensation

### 5. Print Setup (Page 4)

Design your print across three tabs:

- **Workspace** — Review hardware configuration summary
- **Objects** — Add parametric print objects (circles, spheres, etc.) with fill options and ink assignment
- **Wells** — Assign objects to well plate positions and set well roles

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
├── settings.json              # Persistent configuration
├── SupportClasses/            # Backend (hardware, geometry, printing)
│   ├── StageController.py     # Hardware orchestrator
│   ├── HardwareConfig.py      # Central µL↔mm configuration
│   ├── GeometryEngine.py      # Parametric trajectory generation
│   ├── PrintManager.py        # Print execution engine
│   └── ...
├── gui/                       # Frontend (PySide6)
│   ├── app.py                 # Main window
│   ├── pages/                 # Application pages (8)
│   └── widgets/               # Custom Qt widgets (9)
├── config/                    # JSON configuration files
│   ├── controllers/           # Stage protocol definitions
│   ├── hardware/              # Needle/syringe/ink catalogs
│   └── prints/                # Saved print designs
├── tests/                     # Test suite
└── patches/                   # Version upgrade patches
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

### Architecture Documentation

See `coding plans/Architectures/ARCHITECTURE_V729.md` for comprehensive architecture reference.

---

## License

Proprietary — research use only.

---

## Acknowledgments

Built for laboratory bioprinting research. Uses Prior Scientific ProScan stage controllers and Hamilton precision syringes.
