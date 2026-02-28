# MEBP — Multi-Extruder Bioprinting Platform

**Version 7.1** — Desktop application for controlling a multi-axis 3D bioprinting system.

---

## Overview

MEBP controls an XY microscope stage (Prior ProScan II/III), a Z-needle + 3 syringe pump assembly (Marlin 3D printer board), and an Xbox controller for manual jogging. It provides a full GUI for equipment setup, calibration, print job execution, and real-time monitoring.

The system is designed for laboratory bioprinting workflows: loading bio-inks into syringes, calibrating the needle position relative to a well plate, defining print patterns, and executing multi-material prints with automatic fluid handling.

### Key Features

- **6-page GUI** with PyDracula-style sidebar navigation and Catppuccin dark theme
- **Multi-controller support** — JSON-based protocol files for Prior ProScan II and III (auto-detectable)
- **Xbox controller jogging** — real-time stage/pump control via gamepad with configurable button mapping
- **Well plate support** — 6, 12, 24, 48, 96, and 384-well ANSI/SLAS plates with per-well z-offsets
- **Multi-material printing** — per-pump retract/prime, automatic ink change detection, service sequences
- **Trajectory-based execution** — smooth, time-parameterised motion with actual vs planned position recording
- **Print recording & replay** — automatic data capture with tracking error analysis and visual overlay
- **Print queue** — sequential multi-job execution with aggregate progress tracking
- **Safety system** — software endstops, flow rate clamping per needle gauge, velocity dampening near limits
- **Simulation mode** — full hardware simulation for development and testing without physical equipment

---

## Requirements

### Software

| Package | Version | Purpose |
|---------|---------|---------|
| Python | 3.10+ | Runtime |
| PySide6 | 6.5+ | GUI framework |
| pyserial | 3.5+ | Serial communication |
| pygame | 2.1+ | Xbox controller input |
| numpy | 1.24+ | Numerical computation |
| opencv-python | 4.7+ | Camera feed (optional) |

### Hardware

| Device | Connection | Notes |
|--------|-----------|-------|
| Prior ProScan II or III | RS-232 (9600 baud) | XY stage controller |
| 3D printer board (Marlin) | USB serial (115200 baud) | Z-needle + 3 syringe pumps |
| Xbox controller | USB | Optional, for manual jogging |
| USB microscope | USB | Optional, for calibration camera |

---

## Installation

```bash
# Clone the repository
git clone <repo-url>
cd MEBP-Version-7.1

# Install dependencies
pip install PySide6 pyserial pygame numpy

# Optional: camera support
pip install opencv-python
```

---

## Quick Start

### Simulation Mode (no hardware required)

```bash
python main.py
```

Starts with both XY and ZP stages in simulation mode. All features work except actual hardware movement.

### Real Hardware

```bash
# Both stages real
python main.py --real-xy --real-zp

# Only XY real (ZP simulated)
python main.py --real-xy

# Verbose logging
python main.py --real-xy --real-zp --verbose
```

### Headless Mode (no GUI)

```bash
python main.py --headless --real-xy --real-zp
```

Starts the backend only — useful for scripted operation or testing.

---

## GUI Pages

| # | Page | Icon | Purpose |
|---|------|------|---------|
| 1 | **Dashboard** | 📊 | Device status, positions, speeds, connection controls, print history |
| 2 | **Jog Control** | 🕹️ | Manual movement with direction pad, step sizes, speed sliders |
| 3 | **Calibration** | 📐 | Guided 3-step wizard: zero needle, teach plate, validate with camera |
| 4 | **Print Setup** | 🖨️ | Job loading (JSON/G-code), well plate config, 2D preview, execution |
| 5 | **Print Monitor** | 📈 | Live trajectory view, plate progress, syringe status, recording browser |
| 6 | **Settings** | ⚙️ | Connections, safety limits, controller selection, polling, Xbox mapping |

Each page has an optional **context panel** (slide-out from the left) with page-specific settings and controls.

---

## Project Structure

```
MEBP-Version-7.1/
├── main.py                     # Entry point
├── settings.json               # Persistent configuration
├── config/                     # Controller protocols + hardware catalogs
├── SupportClasses/             # Backend (no GUI dependencies)
│   ├── StageController.py      # Top-level orchestrator
│   ├── PrintManager.py         # Print execution engine
│   ├── PrintRecorder.py        # Print data recording
│   ├── XYStage.py              # Prior ProScan driver
│   ├── ZPStage.py              # Marlin G-code driver
│   ├── XboxController.py       # Gamepad polling (separate process)
│   ├── WellPlate.py            # ANSI/SLAS plate geometry
│   ├── SafetyLimits.py         # Software endstops
│   ├── Settings.py             # JSON config with dot-path access
│   └── ...                     # PhysicalModels, Geometry, Trajectory, etc.
├── gui/                        # Frontend (PySide6)
│   ├── app.py                  # MainWindow with sidebar navigation
│   ├── pages/                  # 6 page widgets
│   ├── widgets/                # Reusable UI components
│   └── styles.py               # Catppuccin Mocha dark theme
└── sample_jobs/                # Example print files
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for the complete module reference and dependency graph.

---

## Controller Support

MEBP uses JSON protocol files to define stage controller commands, enabling support for multiple controller models without code changes.

| Controller | File | Status |
|-----------|------|--------|
| Prior ProScan III | `config/controllers/proscan_iii.json` | Default |
| Prior ProScan II | `config/controllers/proscan_ii.json` | Supported |
| Custom | Create new JSON file | Extensible |

Auto-detection is available: set `controller_json = "auto"` in settings, and MEBP will probe each protocol file's detection sequence to identify the connected controller.

---

## Print Job Format

MEBP supports two print file formats:

**Custom JSON** — Full-featured format with settings and typed commands:
```json
{
  "name": "Example Print",
  "settings": {"xy_feedrate": 1000, "num_layers": 2, "travel_z_height": 5.0},
  "commands": [
    {"type": "move_xy", "x": 10.0, "y": 20.0},
    {"type": "move_z", "z": 0.1},
    {"type": "print_path", "points": [[0,0],[10,0],[10,10]], "pump": "P1", "flow_rate": 0.01}
  ]
}
```

**G-code** — Simplified G-code import (G0/G1/G4/G28):
```gcode
G0 X10 Y20 F1000
G1 X20 Y30 E0.5 F200
G4 S1.0
G28
```

---

## Safety

MEBP implements multiple safety layers:

1. **Software endstops** — Configurable per-axis min/max limits with clamping
2. **Flow rate limits** — Per-needle-gauge maximum flow rates (25G: 500 µL/min, 30G: 50 µL/min)
3. **Velocity dampening** — Proportional speed reduction near boundaries
4. **Abort Z-raise** — Automatic needle retraction on print abort
5. **Ink volume tracking** — Automatic service sequences when ink runs low
6. **Emergency stop** — Escape key sends M112 to Marlin firmware
7. **Hardware endstops** — Physical switches on printer board (last resort)

---

## Development

### Running Tests

```bash
# Headless integration test (no display required)
QT_QPA_PLATFORM=offscreen python -m pytest gui/tests/test_integration.py -v
```

### Adding a New Controller

1. Create a new JSON file in `config/controllers/` following the schema in `proscan_iii.json`
2. Define commands, detection sequence, encoding, and terminators
3. The controller will automatically appear in Settings → Controller Selector

### Extending Print Commands

1. Add new value to `CommandType` enum in `PrintManager.py`
2. Add handler in `_execute_command()` method
3. Add JSON serialization support in `_load_json_file()` / `save_print_job()`

---

## License

Proprietary — for laboratory use.

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 7.0 | 2025 | PyDracula layout, 5-page GUI, print queue, G-code export, print resume |
| 7.1 | 2026 | Trajectory execution, print recording, Print Monitor page, protocol abstraction, fluid tracking |
