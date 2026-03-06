# MEBP v7.2.6 — Print Execution Restoration

## Overview

Version 7.2.6 restores working print execution to the v7.2.5 codebase by fixing
7 breaks in the pipeline between "Send to Monitor" and "PrintManager runs commands."

## Issues Fixed

| # | Issue | Session | Severity |
|---|-------|---------|----------|
| 1 | `_build_current_job()` calls `build_well_plate_job()` with wrong API | S1 | CRITICAL |
| 2 | Tab 2 print objects never reach the job builder | S1 | CRITICAL |
| 3 | `PrintPlanOfAction` has no execution bridge to `PrintCommand` list | S1 | CRITICAL |
| 4 | `_on_monitor_start()` calls `start(job)` instead of `load_job()`+`start()` | S2 | HIGH |
| 5 | Progress callbacks fire from daemon thread → SIGSEGV on macOS | S2 | HIGH |
| 6 | PrintMonitor missing live position polling during print | S2 | MEDIUM |
| 7 | `on_print_state_changed()` doesn't advance job queue | S2 | MEDIUM |

## File Inventory

### Patch Files

| File | Lines | Targets |
|------|-------|---------|
| `patch_v726_session1_job_pipeline.py` | 931 | `print_setup.py`, `PrintPlanOfAction.py` |
| `patch_v726_session2_execution_chain.py` | 463 | `app.py`, `print_monitor.py` |
| `apply_all_v726_patches.py` | 68 | Master runner |

### Files Modified

| File | Changes |
|------|---------|
| `gui/pages/print_setup.py` | Rewritten `_build_current_job()`, new `_get_path_from_objects()` + `_single_object_to_points()`, updated `_generate_print()` + `_send_to_monitor()` |
| `SupportClasses/PrintPlanOfAction.py` | Added `plan_to_commands()` + 5 service step generators |
| `gui/app.py` | Fixed `_on_monitor_start()`, fixed `_wire_print_manager_to_monitor()` |
| `gui/pages/print_monitor.py` | Added `on_status_update()`, fixed `on_print_state_changed()`, added `_get_controller()` |

## Installation

### Prerequisites

- MEBP v7.2.5 fully deployed
- Python 3.10+
- PySide6 installed

### Apply All Patches

```bash
cd /path/to/MEBP
python patches/v726/apply_all_v726_patches.py /path/to/MEBP
```

### Apply Individual Sessions

```bash
# Session 1: Job building pipeline
python patches/v726/patch_v726_session1_job_pipeline.py /path/to/MEBP

# Session 2: Execution chain
python patches/v726/patch_v726_session2_execution_chain.py /path/to/MEBP
```

### Verify

```bash
# AST check all modified files
python3 -c "import ast; ast.parse(open('gui/pages/print_setup.py').read())"
python3 -c "import ast; ast.parse(open('gui/app.py').read())"
python3 -c "import ast; ast.parse(open('gui/pages/print_monitor.py').read())"
python3 -c "import ast; ast.parse(open('SupportClasses/PrintPlanOfAction.py').read())"

# Import check
python3 -c "from gui.pages.print_setup import PrintSetupPage; print('OK')"

# Launch
python main.py
```

## Signal Flow (Fixed)

```
Page 0 (Hardware Setup)
  │ config_changed(HardwareConfig)
  ▼
app.py._on_hardware_config_changed()
  │ _propagate_hardware_config()
  ▼
Page 4 (Print Setup)
  │ set_hardware_config(config) → all 3 tabs
  │
  │ User clicks "⚙ Generate Print"
  │   → _generate_print()
  │   → tab_wells.validate()
  │   → tab_wells._generate_plan()
  │   → plan_to_commands() → full PrintJob with service steps
  │   → self._generated_job = job
  │
  │ User clicks "📤 Send to Monitor ▶"
  │   → _send_to_monitor()
  │   → validate() gate
  │   → job_ready.emit(self._generated_job)
  ▼
app.py._send_job_to_monitor(job)
  │ → monitor.receive_job(job)
  │ → _switch_page(5)
  ▼
Page 5 (Print Monitor)
  │ User clicks "▶ Start Print"
  │ start_requested.emit(job)
  ▼
app.py._on_monitor_start(job)
  │ → pm.load_job(job)       ← v7.2.6 FIX
  │ → pm.start()             ← no args
  ▼
PrintManager daemon thread
  │ _execute_loop() → _execute_command() for each PrintCommand
  │
  │ Progress callbacks (from daemon thread):
  │   pm.on_progress → bridge.progress_signal.emit() (thread-safe)
  │     → QueuedConnection → monitor.on_print_progress() (main thread)
  │   pm.on_state_changed → bridge.state_signal.emit() (thread-safe)
  │     → QueuedConnection → monitor.on_print_state_changed() (main thread)
  ▼
monitor.on_status_update() [QTimer 300ms]  ← v7.2.6 NEW
  │ → controller.get_xy_position(cached=True)
  │ → trajectory_view.set_current_position(x, y)
  │ → position readout labels
  ▼
On COMPLETED:
  │ → monitor._advance_queue()              ← v7.2.6 FIX
  │ → next job or queue empty
```

## Dependency Order

```
Session 1 (Job Building Pipeline)
    └──→ Session 2 (Execution Chain)
```

Session 1 must be applied first. Session 2 depends on Session 1's
`plan_to_commands()` being available but handles ImportError gracefully.

## Idempotency

All patches are safe to re-run. Each change checks for its unique
`v7.2.6:` marker before applying. Re-running produces all SKIPs.
