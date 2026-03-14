# CLAUDE.md — MEBP Project Instructions

## Current Version

**V7.3.5** — Microscope-Enabled Bioprinting Platform

Branch: `Version-7.3.5`

---

## Required Reading

**Before making ANY code changes, read the architecture document for this version:**

`coding plans/Architectures/ARCHITECTURE_V735.md`

This document contains the full system architecture, module responsibilities, data flow diagrams, communication protocols, and key algorithms. Understanding this is mandatory before modifying any code.

---

## Project Overview

MEBP is a Python/PySide6 desktop application for controlling laboratory bioprinting equipment. It orchestrates Prior ProScan XY stages, Marlin Z/pump controllers, and Hamilton syringe systems for multi-material biological printing.

### Key Directories

| Path | Purpose |
|------|---------|
| `main.py` | Entry point — CLI args, mode selection |
| `SupportClasses/` | Backend modules — zero GUI dependencies |
| `gui/` | PySide6 frontend — pages, widgets, dialogs, styles |
| `config/` | Hardware configs, controller protocols, print files |
| `coding plans/` | Architecture docs, update plans, readmes |
| `tests/` | Test suite |
| `patches/` | Legacy version upgrade patches (deprecated — content migrated to update plans) |

### Core Design Principles

1. **Hardware abstraction** — GUI pages never touch serial ports. All hardware access through `StageController` or `PrintManager`.
2. **JSON protocol abstraction** — XY commands defined in `config/controllers/` JSON files.
3. **Hardware-first** — Page 0 (Hardware Setup) must complete before other pages unlock.
4. **µL-native pumps** — All user-facing pump values in microliters. Conversion to mm happens only inside `StageController.move_pump_uL()`.
5. **Safety-layered** — Software endstops, flow-rate clamping, crash recovery throughout.

---

## Update Plans

For each update task in a version, a plan document **must** exist in:

`coding plans/Update plans/`

### Naming Convention

`MEBP_v{from}_to_v{to}_UPDATE.md` — e.g., `MEBP_v728_to_v729_UPDATE.md`

### Plan Document Requirements

Each update plan is a **living document** that must include:

1. **Objective** — What this update accomplishes
2. **Files Modified** — List of all files touched with brief rationale
3. **Implementation Steps** — Ordered checklist of changes
4. **Status Tracking** — Mark each step as: `[ ]` pending, `[x]` done, `[~]` in-progress
5. **Testing Notes** — How to verify the changes work
6. **Issues & Decisions** — Log of problems encountered and decisions made during implementation

### Workflow

- Before starting work on a new update task, **create or update** the plan file
- As work progresses, **update the status** of each step in the plan
- When complete, ensure all steps are marked `[x]` and add a summary of what was done

---

## Existing Update Plans

| File | Scope |
|------|-------|
| `MEBP_v734_to_v735_UPDATE.md` | v7.3.5 Critical fixes: safe Z flush_moves() stale "ok" drain, click-to-move QLabel event filter, XY unit rename (microsteps_per_micron → xy_position_scale, 10→1), ZP stage settings card, watchdog M500, simulation state persistence |
| `MEBP_v733_to_v734_UPDATE.md` | v7.3.4 Objective-based µm/px calibration, MosaicCalibrator.py, XY lag fix (R-ack), XY stop fix, CSS overhead fix, Xbox trigger creep + debug mode, calibration click-to-move, safe navigation Z-wait fix |
| `MEBP_v732_to_v733_UPDATE.md` | v7.3.3 Mode-based navigation + Pick & Place, CameraManager shared feeds, µm/px empirical calibration, relaxed needle detection |
| `MEBP_v731_to_v732_UPDATE.md` | v7.3.2 QoL: camera config persistence, jog well plate, axis flip, custom steps, calibration restructure, print monitor camera overlay |
| `MEBP_v730_to_v731_UPDATE.md` | v7.3.1 Calibration overhaul: geometry-predicted wells, 3-well SVD auto-calibration, auto Z-bottom, simplified wizard, 110 tests |
| `MEBP_v730_AUTOCALIBRATION_UPDATE.md` | v7.3.0 Autocalibration: well/needle detection, focus assist, simulated camera, detection overlay, 7 phases |
| `MEBP_v729_to_v730_UPDATE.md` | v7.2.9→v7.3.0 Plan of Action redesign, PrintExecutionConfig, Finalize tab, new step types |
| `MEBP_v728_to_v729_UPDATE.md` | v7.2.8→v7.2.9 Object consolidation, multi-ink, ink swap strategy, triangle fill |
| `MEBP_v727_to_v728_UPDATE.md` | v7.2.8 Hardware comms: XY auto-detection, ProScan CR terminator, atomic position reads, Xbox status fix, simulation defaults |
| `MEBP_v726_to_v727_UPDATE.md` | v7.2.7 Calibration workflow, Xbox Bluetooth cross-platform, Helper Functions page, print speed propagation, position display fixes |
| `MEBP_v725_to_v726_UPDATE.md` | v7.2.6 Print execution restoration: 7 critical pipeline breaks, 8 secondary fixes, print monitor redesign, ToupCam integration |
| `MEBP_v724_to_v725_UPDATE.md` | v7.2.5 Config load fix, XY µm-direct, pump µL jog, well setup redesign, print file list, Print Results page |
| `MEBP_v723_to_v724_UPDATE.md` | v7.2.4 Config propagation audit, jog step verification, pump-ink-needle mapping, zoomable preview + OOB detection, plan validation |
| `MEBP_v72_to_v723_UPDATE.md` | v7.2.3 UI/workflow overhaul: HW Setup auto-load, read-only workspace, execution→monitor, file-centric print objects |
| `MEBP_v712_to_v72_UPDATE.md` | v7.2.0 Hardware Setup page, HardwareConfig model, µL-native pumps, PyDracula GUI migration, Catppuccin theme |
| `MEBP_v71_to_v712_UPDATE.md` | v7.1.2 Bug fixes: XY jog accuracy, simulator performance, protocol loading, position rounding |

---

## Code Conventions

- **Theme**: Catppuccin Mocha dark (QSS in `gui/styles.py`)
- **Config format**: JSON everywhere (hardware, controllers, prints, settings)
- **Trajectories**: Nx7 numpy arrays `[x, y, z, p1, p2, p3, t]`
- **Serial**: pyserial RS-232, retry decorator with exponential backoff
- **Threading**: `Processor` command bus for thread safety; Qt signals for GUI updates
- **Testing**: unittest + physics-based simulators (`XYStageSimulator`, `ZPStageSimulator`, `SimulatedCamera`)
- **Vision**: OpenCV-based detection in `VisionDetector.py`; `DetectionWorker` QThread for parallel processing; `DetectionOverlay` for camera feed annotations

---

## This Is a Living Document

Update this file when:
- The version number changes
- Architecture documents are added or updated
- New conventions or patterns are established
- Project structure changes significantly
