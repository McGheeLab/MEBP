# CLAUDE.md — MEBP Project Instructions

## Current Version

**V7.2.9** — Multi-Extrusion Bioprinting Platform

Branch: `Version-7.2.9`

---

## Required Reading

**Before making ANY code changes, read the architecture document for this version:**

`coding plans/Architectures/ARCHITECTURE_V729.md`

This document contains the full system architecture, module responsibilities, data flow diagrams, communication protocols, and key algorithms. Understanding this is mandatory before modifying any code.

---

## Project Overview

MEBP is a Python/PySide6 desktop application for controlling laboratory bioprinting equipment. It orchestrates Prior ProScan XY stages, Marlin Z/pump controllers, and Hamilton syringe systems for multi-material biological printing.

### Key Directories

| Path | Purpose |
|------|---------|
| `main.py` | Entry point — CLI args, mode selection |
| `SupportClasses/` | Backend modules — zero GUI dependencies |
| `gui/` | PySide6 frontend — pages, widgets, styles |
| `config/` | Hardware configs, controller protocols, print files |
| `coding plans/` | Architecture docs, update plans, readmes |
| `tests/` | Test suite |
| `patches/` | Version upgrade patches |

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
| `MEBP_v729_to_v730_UPDATE.md` | Plan of Action redesign, PrintExecutionConfig, Finalize tab, new step types |
| `MEBP_v728_to_v729_UPDATE.md` | Object consolidation, multi-ink, ink swap strategy, triangle fill |
| `V726_PRINT_RESTORATION_PLAN.md` | Print execution restoration (7 critical fixes) |
| `MEBP_v725_UPDATE_PLAN.md` | v7.2.5 multi-pump UI |
| `MEBP_v724_UPDATE_PLAN.md` | v7.2.4 pump channel mapping |
| `V723_UPGRADE_PLAN.md` | v7.2.3 hardware setup page |

---

## Code Conventions

- **Theme**: Catppuccin Mocha dark (QSS in `gui/styles.py`)
- **Config format**: JSON everywhere (hardware, controllers, prints, settings)
- **Trajectories**: Nx7 numpy arrays `[x, y, z, p1, p2, p3, t]`
- **Serial**: pyserial RS-232, retry decorator with exponential backoff
- **Threading**: `Processor` command bus for thread safety; Qt signals for GUI updates
- **Testing**: unittest + physics-based simulators (`XYStageSimulator`, `ZPStageSimulator`)

---

## This Is a Living Document

Update this file when:
- The version number changes
- Architecture documents are added or updated
- New conventions or patterns are established
- Project structure changes significantly
