# MEBP v7.2.6 — Print Execution Restoration Plan

## Version: 7.2.6 | Date: March 2026
## Goal: Restore working print execution in v7.2.5 by bridging the v7.0 execution engine with the v7.2.5 setup infrastructure

---

## 1. Executive Summary

**The Problem:** v7.0 runs prints perfectly but has primitive hardware/environment setup. v7.2.5 has excellent setup (HardwareConfig, WellSetupModel, PrintPlanOfAction, needle channels, ink mapping) but **cannot actually execute prints**. The print execution chain has at least 7 critical breaks between "Send to Monitor" and "PrintManager runs commands."

**The Strategy:** Keep v7.2.5's setup infrastructure intact. Fix the broken bridge between the setup layer and the execution engine. The PrintManager core (`_execute_loop`, `_execute_command`, command dispatch) is unchanged from v7.0 and known-good — the failures are in the *pipeline that feeds it*.

**Estimated Scope:** 3 coding sessions, 1 master patch file

---

## 2. Root Cause Analysis — The 7 Breaks

### Break 1: `_build_current_job()` API Mismatch ⛔ CRITICAL

**v7.0 call (WORKS):**
```python
job = build_well_plate_job(
    well_positions=[(name, x, y), ...],   # list of (str, float, float)
    path_points=[(x, y), ...],            # pattern points from generator
    settings=settings,                     # PrintSettings
    pump="P1",
    flow_rate=0.01,
)
```

**v7.2.5 call (BROKEN — two versions exist, both wrong):**
```python
# Version A: wrong keyword args — build_well_plate_job doesn't accept plate= or selected_wells=
return build_well_plate_job(
    plate=plate,
    selected_wells=print_wells,
    settings=settings,
)

# Version B: falls back but uses dummy path
path_points = [(0.0, 0.0)]  # Single center point — no actual print geometry!
return build_well_plate_job(well_positions, path_points, settings)
```

**Impact:** Either TypeError on wrong kwargs, or a job with zero meaningful print geometry.

**Fix:** Rewrite `_build_current_job()` to:
1. Extract well positions from `WellSetupModel` (correct v7.2.5 data model)
2. Extract print object geometry from Tab 2's collections/objects
3. Call `build_well_plate_job()` with the correct v7.0 API signature
4. Include per-pump settings, multi-material params from HardwareConfig

---

### Break 2: Print Object Geometry Never Reaches Job Builder ⛔ CRITICAL

**v7.0:** User selects pattern (line/meander/spiral/grid) → `_generate_current_pattern()` returns `[(x,y), ...]` → passed directly to `build_well_plate_job(path_points=...)`.

**v7.2.5:** Tab 2 (Print Objects) creates parametric objects (line, meander, spiral, grid, rosette) stored in `PrintFileData` with collections. But `_build_current_job()` **never reads from Tab 2**. It just uses `path_points = [(0.0, 0.0)]`.

**Impact:** Every generated job prints a single dot at the center of each well.

**Fix:** Create a `_get_print_geometry_from_objects()` method that:
1. Reads the current print file's objects + collections from Tab 2
2. Converts parametric objects into point lists compatible with `path_points`
3. For multi-object collections, merges all paths into a unified point list
4. Handles per-well object assignments if wells have specific prints assigned

---

### Break 3: PrintPlanOfAction Has No Execution Bridge ⛔ CRITICAL

**What it does:** `PrintPlanOfAction` generates high-level steps like:
```
Step 1: WASTE — Eject into waste well
Step 2: WASH — Clean in wash well  
Step 3: LOAD_INK — Aspirate 50µL from ink well
Step 4: PRINT — Print wells A1, A2, A3 (Run 1/2)
Step 5: WASTE → WASH → LOAD_INK → PRINT (Run 2/2)
Step 6: RETURN_HOME
```

**What it doesn't do:** Convert these high-level steps into `PrintCommand` objects that `PrintManager._execute_loop()` can run.

**Impact:** Even if a valid job is built, the plan's multi-run service sequences (ink loading, washing, buffering) are completely ignored. The job is a flat list of MOVE_XY + EXTRUDE commands without any service steps between runs.

**Fix:** Create `plan_to_commands()` bridge function in `PrintPlanOfAction.py` that:
1. Takes a `PrintPlanOfAction` + `WellSetupModel` + `PrintSettings`
2. For each WASTE step → generates MOVE_XY to waste well + EXTRUDE commands
3. For each WASH step → generates MOVE_XY to wash well + dwell/jiggle
4. For each LOAD_INK step → generates MOVE_XY to ink well + EXTRUDE (aspirate)
5. For each PRINT step → calls `build_well_plate_job()` for that subset of wells
6. Returns a complete `PrintJob` with all commands in correct order

---

### Break 4: `_on_monitor_start()` API Inconsistency ⚠️ HIGH

**Two versions exist in project knowledge:**

```python
# Version A (WRONG — start() takes no args):
def _on_monitor_start(self, job):
    setup_page.print_manager.start(job)  # TypeError: start() takes 1 positional argument

# Version B (CORRECT — load then start):
def _on_monitor_start(self, job):
    pm = setup_page.print_manager
    pm.load_job(job)   # load first
    pm.start()         # start takes no args
```

**Impact:** If version A is in the actual file, clicking "Start Print" in the Monitor crashes.

**Fix:** Verify which version is on disk. Ensure it's Version B with proper error handling.

---

### Break 5: Thread-Safety of Progress Callbacks ⚠️ HIGH

**v7.0:** Progress callbacks update the PathPreviewCanvas directly — safe because PrintSetupPage owns both the PrintManager and the canvas on the same thread through signal bridge.

**v7.2.5:** Two wiring versions exist:

```python
# Old version (DANGEROUS — direct callback from daemon thread):
def combined_progress(step, total, msg):
    monitor_page.on_print_progress(step, total, msg)  # GUI call from worker thread!

# New version (SAFE — uses QueuedConnection signal bridge):
self._print_signal_bridge.progress_signal.connect(
    monitor_page.on_print_progress,
    type=Qt.QueuedConnection,
)
```

**Impact:** If the old version is on disk, prints may SIGSEGV on macOS during execution.

**Fix:** Verify the thread-safe signal bridge version is active. The newer `_wire_print_manager_to_monitor()` with `PrintSignalBridge` + `QueuedConnection` is correct.

---

### Break 6: PrintMonitor Missing Live Visualization Updates ⚠️ MEDIUM

**v7.0:** During printing, `on_status_update()` (called by QTimer every 300ms) reads cached position and updates `PathPreviewCanvas` with current needle position + completed steps.

**v7.2.5:** `PrintMonitorPage` has plate overview, trajectory view, and syringe displays but:
- `on_print_progress()` only updates labels and progress bar
- No position polling feeds the trajectory view during printing
- The L-shaped projection views (XY/ZY/XZ) don't receive real-time updates
- `on_status_update()` on the monitor page doesn't read live positions

**Impact:** User sees progress text but no visual path rendering during print execution.

**Fix:** Add position polling to the monitor's `on_status_update()`:
1. Read cached XY + ZP positions from StageController
2. Update trajectory view with current position
3. Update plate overview with per-well progress coloring
4. Feed syringe display with pump position data

---

### Break 7: Generated Job Missing Print Objects Per-Well ⚠️ MEDIUM

**v7.0:** All wells get the same pattern (one pattern for all wells).

**v7.2.5 intent:** Different wells can have different print objects assigned (via Tab 3 Well Setup). But `_build_current_job()` never reads per-well print assignments.

**Impact:** Per-well print customization is lost.

**Fix (Phase 2):** When building the job, check each well's assignment for specific print objects. If a well has a specific print assigned, use that print's geometry instead of the global pattern.

---

## 3. Planned Changes — Session Breakdown

### Session 1: Fix the Job Building Pipeline (CRITICAL PATH)
**Files:** `gui/pages/print_setup.py`, `SupportClasses/PrintPlanOfAction.py`
**Estimated time:** 4-5 hours

| Task | Description |
|------|-------------|
| S1.1 | **Fix `_build_current_job()`** — Rewrite to use correct `build_well_plate_job()` API. Extract well positions from `WellSetupModel`, extract path points from Tab 2 objects. |
| S1.2 | **Add `_get_path_from_objects()`** — New method on PrintSetupPage that reads Tab 2's current print file objects and converts them to `path_points` list. Falls back to v7.0 default meander pattern if no objects. |
| S1.3 | **Add `_get_pump_params()`** — Extract active pump, flow rate, multi-material settings from context panel + HardwareConfig. |
| S1.4 | **Add `plan_to_commands()` in PrintPlanOfAction.py** — Bridge function that converts high-level plan steps to PrintCommand list. Handles WASTE/WASH/BUFFER/LOAD_INK/PRINT/RETURN_HOME step types. |
| S1.5 | **Update `_generate_print()` to build full job** — After plan generation + validation, call `plan_to_commands()` to produce a complete PrintJob. Store as `self._generated_job`. |
| S1.6 | **Update `_send_to_monitor()` to use generated job** — Prefer `self._generated_job` over `_build_current_job()`. Clear after sending. |
| S1.7 | **Fallback path: simple job build** — If no plan exists (user skips "Generate Print"), `_build_current_job()` should still work as a simple v7.0-style job builder. |

**Key design decisions:**
- `plan_to_commands()` produces a FLAT `PrintJob.commands` list — PrintManager doesn't need to know about the plan abstraction
- Service steps (wash, waste, buffer, ink load) become sequences of MOVE_XY + MOVE_Z + EXTRUDE + DWELL commands
- The generated job includes SWITCH_PUMP commands where ink changes between runs
- Per-well path data comes from Tab 2 objects; if empty, use a configurable default pattern

---

### Session 2: Fix the Execution Chain (Monitor → PrintManager)
**Files:** `gui/app.py`, `gui/pages/print_monitor.py`
**Estimated time:** 3-4 hours

| Task | Description |
|------|-------------|
| S2.1 | **Verify/fix `_on_monitor_start()`** — Must call `pm.load_job(job)` then `pm.start()` (not `pm.start(job)`). Add error handling. |
| S2.2 | **Verify/fix `_wire_print_manager_to_monitor()`** — Must use signal bridge with `QueuedConnection`, not direct callbacks. |
| S2.3 | **Fix Monitor `receive_job()` robustness** — Ensure it handles jobs with and without `plan_of_action` attribute. Update job queue display. |
| S2.4 | **Fix Monitor `on_print_state_changed()` queue advance** — When a print completes, properly advance to next job in queue. |
| S2.5 | **Fix Monitor pause/resume/abort signal flow** — Verify all three signals reach PrintManager correctly. |
| S2.6 | **Add `on_status_update()` to Monitor** — Wire position polling from StageController to update trajectory view during printing. |
| S2.7 | **Wire PrintSetupPage's bridge to not conflict** — PrintSetupPage still creates its own `PrintSignalBridge` and wires `on_progress`/`on_state_changed`. app.py's `_wire_print_manager_to_monitor()` creates combined callbacks. Verify no double-fire or clobber. |
| S2.8 | **Test: End-to-end signal flow** — Trace: "Start Print" click → `start_requested` signal → app.py → `load_job()` → `start()` → progress callbacks → Monitor UI updates. |

---

### Session 3: Integration Testing + Live Visualization
**Files:** `gui/pages/print_monitor.py`, `tests/test_v726_print_execution.py`
**Estimated time:** 3-4 hours

| Task | Description |
|------|-------------|
| S3.1 | **Monitor position polling** — In `on_status_update()`, read cached XY/ZP positions and update trajectory view crosshair. |
| S3.2 | **Monitor well progress coloring** — Update plate overview to show current-well-active during printing, completed wells in green. |
| S3.3 | **Monitor syringe display** — Feed syringe display with current pump position during print. |
| S3.4 | **Test: Simple print** — HW setup → assign wells → generate print → send to monitor → start → verify progress updates → completion. |
| S3.5 | **Test: Multi-run print** — Setup with more ink needed than syringe capacity → verify plan generates multiple runs → verify service sequences execute. |
| S3.6 | **Test: Pause/Resume/Abort** — Verify all three controls work during active print. |
| S3.7 | **Test: Job queue** — Multiple jobs → start → verify auto-advance after first completes. |
| S3.8 | **Fallback test: Direct file load** — Load a v7.0 .json print file directly → verify it still executes (bypass plan system). |
| S3.9 | **Write integration tests** — Headless tests that verify the job building pipeline without GUI. |

---

## 4. Detailed Code Changes

### 4.1 `_build_current_job()` Rewrite (print_setup.py)

```python
def _build_current_job(self):
    """Build a PrintJob from current well setup + print objects.
    
    v7.2.6: Fixed to use correct build_well_plate_job() API.
    Extracts geometry from Tab 2 objects, well positions from Tab 3 model.
    """
    # 1. Get well model from Tab 3
    if not (hasattr(self, 'tab_wells') and hasattr(self.tab_wells, '_model')
            and self.tab_wells._model):
        logger.warning("No well model available")
        return None
    
    model = self.tab_wells._model
    plate = model.plate
    if plate is None:
        logger.warning("No plate geometry")
        return None
    
    # 2. Get print wells
    print_wells = []
    for name, assignment in model.assignments.items():
        role = getattr(assignment, 'role', None)
        if role is not None and getattr(role, 'value', None) == 'print':
            print_wells.append(name)
    
    if not print_wells:
        logger.warning("No print wells assigned")
        return None
    
    # 3. Build well_positions: list of (name, x_mm, y_mm)
    well_positions = []
    for name in print_wells:
        try:
            x, y = plate.get_well_position(name)
        except Exception:
            x, y = 0.0, 0.0
        well_positions.append((name, x, y))
    
    # 4. Get path points from Tab 2 objects
    path_points = self._get_path_from_objects()
    if not path_points:
        # Fallback: default meander pattern
        from SupportClasses.WellPlate import generate_meander_path
        well_diameter = getattr(plate, 'well_diameter', 6.0)
        path_points = generate_meander_path(
            well_diameter * 0.7, well_diameter * 0.7, 0.5)
    
    # 5. Get settings
    settings = self._get_settings()
    
    # 6. Get pump/flow from settings or HardwareConfig
    pump = getattr(settings, 'active_pump', 'P1') or 'P1'
    flow_rate = getattr(settings, 'flow_rate', 0.01) or 0.01
    
    # 7. Build multi-material params if configured
    mm_params = {}
    # (future: extract from HardwareConfig pump assignments)
    
    # 8. Build job with correct API
    try:
        job = build_well_plate_job(
            well_positions=well_positions,
            path_points=path_points,
            settings=settings,
            pump=pump,
            flow_rate=flow_rate,
            job_name=f"Well Plate {plate.format}-well",
            **mm_params,
        )
        return job
    except Exception as exc:
        logger.error(f"build_well_plate_job failed: {exc}", exc_info=True)
        return None
```

### 4.2 `_get_path_from_objects()` New Method (print_setup.py)

```python
def _get_path_from_objects(self):
    """Extract path points from Tab 2 print objects.
    
    v7.2.6: Bridge between PrintObjects tab geometry and job builder.
    Returns list of (x, y) tuples or empty list.
    """
    if not hasattr(self, 'tab_objects'):
        return []
    
    # Try to get the current print file's objects
    tab = self.tab_objects
    
    # Check for PrintFileManager-based objects
    if hasattr(tab, '_file_manager') and hasattr(tab._file_manager, 'current_file'):
        file_data = tab._file_manager.current_file
        if file_data and hasattr(file_data, 'objects') and file_data.objects:
            return self._objects_to_path_points(file_data.objects)
    
    # Check for direct object list
    if hasattr(tab, '_objects') and tab._objects:
        return self._objects_to_path_points(tab._objects)
    
    # Check for collection-based objects  
    if hasattr(tab, '_collections') and tab._collections:
        all_points = []
        for coll in tab._collections.values():
            for entry in coll:
                obj_name = entry.get('object_name', '')
                if hasattr(tab, '_objects') and obj_name in tab._objects:
                    obj = tab._objects[obj_name]
                    pts = self._single_object_to_points(obj)
                    # Offset by collection position
                    pos = entry.get('position', [0, 0, 0])
                    all_points.extend([(x + pos[0], y + pos[1]) for x, y in pts])
        return all_points
    
    return []

def _objects_to_path_points(self, objects_dict):
    """Convert print objects dict to flat path points list."""
    all_points = []
    for obj_name, obj_data in objects_dict.items():
        pts = self._single_object_to_points(obj_data)
        pos = obj_data.get('position', [0, 0, 0])
        all_points.extend([(x + pos[0], y + pos[1]) for x, y in pts])
    return all_points

def _single_object_to_points(self, obj_data):
    """Convert a single print object to path points."""
    obj_type = obj_data.get('object_type', '') if isinstance(obj_data, dict) else getattr(obj_data, 'object_type', '')
    params = obj_data.get('params', {}) if isinstance(obj_data, dict) else getattr(obj_data, 'params', {})
    
    from SupportClasses.WellPlate import (
        generate_line_path, generate_meander_path,
        generate_spiral_path, generate_grid_path,
    )
    
    if obj_type == 'line':
        return generate_line_path(
            params.get('length', 5.0),
            params.get('angle', 0.0))
    elif obj_type == 'meander':
        return generate_meander_path(
            params.get('width', 5.0),
            params.get('height', 5.0),
            params.get('spacing', 0.5))
    elif obj_type == 'spiral':
        return generate_spiral_path(
            params.get('radius', 3.0),
            params.get('spacing', 0.5))
    elif obj_type == 'grid':
        return generate_grid_path(
            params.get('width', 5.0),
            params.get('height', 5.0),
            params.get('spacing_x', 1.0),
            params.get('spacing_y', 1.0))
    # Fallback: try to extract raw points
    elif 'points' in (params if isinstance(params, dict) else {}):
        return [(p[0], p[1]) for p in params['points']]
    return []
```

### 4.3 `plan_to_commands()` Bridge Function (PrintPlanOfAction.py)

```python
def plan_to_commands(
    plan: PrintPlanOfAction,
    well_model,      # WellSetupModel
    plate,           # WellPlate
    path_points,     # list[(float, float)] - print geometry
    settings,        # PrintSettings
    hw_config=None,  # HardwareConfig (optional, for pump/ink info)
) -> PrintJob:
    """Convert a PrintPlanOfAction into an executable PrintJob.
    
    v7.2.6: Bridge between high-level plan and low-level command executor.
    
    Each PlanStep becomes a sequence of PrintCommands:
        WASTE       → travel to waste well, lower, extrude to eject, raise
        WASH        → travel to wash well, lower, dwell/jiggle, raise
        REFILL_BUFFER → travel to buffer well, lower, aspirate, raise
        LOAD_INK    → travel to ink well, lower, aspirate, raise
        PRINT       → for each well: travel, lower, print path, raise
        RETURN_HOME → home XY
    """
    from SupportClasses.PrintManager import (
        PrintJob, PrintCommand, CommandType, build_well_plate_job,
    )
    
    all_commands = []
    
    for step in plan.steps:
        if step.step_type == PlanStepType.WASTE:
            cmds = _waste_commands(step, well_model, plate, settings)
            all_commands.extend(cmds)
            
        elif step.step_type == PlanStepType.WASH:
            cmds = _wash_commands(step, well_model, plate, settings)
            all_commands.extend(cmds)
            
        elif step.step_type == PlanStepType.REFILL_BUFFER:
            cmds = _buffer_commands(step, well_model, plate, settings)
            all_commands.extend(cmds)
            
        elif step.step_type == PlanStepType.LOAD_INK:
            cmds = _load_ink_commands(step, well_model, plate, settings)
            all_commands.extend(cmds)
            
        elif step.step_type == PlanStepType.PRINT:
            # Build sub-job for this run's wells
            well_positions = []
            for well_name in step.target_wells:
                try:
                    x, y = plate.get_well_position(well_name)
                except Exception:
                    continue
                well_positions.append((well_name, x, y))
            
            if well_positions:
                pump = step.pump_id or settings.active_pump or "P1"
                flow = getattr(settings, 'flow_rate', 0.01)
                sub_job = build_well_plate_job(
                    well_positions=well_positions,
                    path_points=path_points,
                    settings=settings,
                    pump=pump,
                    flow_rate=flow,
                )
                all_commands.extend(sub_job.commands)
            
        elif step.step_type == PlanStepType.RETURN_HOME:
            all_commands.append(PrintCommand(
                type=CommandType.HOME_XY,
                label="Return to home position",
            ))
    
    # If plan was empty or had no commands, fall back to simple build
    if not all_commands:
        return None
    
    return PrintJob(
        name=f"Plan: {plan.total_print_wells} wells, {plan.total_runs} run(s)",
        description=f"Generated from PrintPlanOfAction",
        settings=settings,
        commands=all_commands,
    )
```

### 4.4 Service Step Command Generators (PrintPlanOfAction.py)

```python
def _find_service_well(well_model, plate, role_value: str):
    """Find the first well with the given role and return its (x, y) position."""
    for name, assignment in well_model.assignments.items():
        r = getattr(assignment, 'role', None)
        if r is not None and getattr(r, 'value', None) == role_value:
            try:
                return name, *plate.get_well_position(name)
            except Exception:
                continue
    return None

def _waste_commands(step, well_model, plate, settings):
    """Generate commands for a waste ejection step."""
    well_info = _find_service_well(well_model, plate, "waste")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No waste well")]
    name, x, y = well_info
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"── Waste: {name} ──"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to waste well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower to waste depth"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": step.pump_id or "P1", "amount": 5.0, "feedrate": 60},
                     label="Eject waste"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 0.5},
                     label="Settle after waste"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from waste"),
    ]

def _wash_commands(step, well_model, plate, settings):
    """Generate commands for a wash step."""
    well_info = _find_service_well(well_model, plate, "wash")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No wash well")]
    name, x, y = well_info
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"── Wash: {name} ──"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to wash well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower into wash"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 5.0},
                     label="Wash soak"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from wash"),
    ]

def _buffer_commands(step, well_model, plate, settings):
    """Generate commands for a buffer refill step."""
    well_info = _find_service_well(well_model, plate, "buffer")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No buffer well")]
    name, x, y = well_info
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"── Buffer: {name} ──"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to buffer well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower into buffer"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": step.pump_id or "P1", "amount": -5.0, "feedrate": 30},
                     label="Aspirate buffer"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0},
                     label="Settle after buffer"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from buffer"),
    ]

def _load_ink_commands(step, well_model, plate, settings):
    """Generate commands for an ink loading step."""
    well_info = _find_service_well(well_model, plate, "ink")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No ink well")]
    name, x, y = well_info
    volume = step.volume_uL if step.volume_uL > 0 else 50.0
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"── Load Ink: {step.ink_name or '?'} ({volume:.1f}µL) ──"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to ink well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower into ink"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": step.pump_id or "P1",
                             "amount": -volume, "feedrate": 30},
                     label=f"Aspirate {volume:.1f}µL"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0},
                     label="Settle after ink load"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from ink well"),
    ]
```

---

## 5. `_on_monitor_start()` Fix (app.py)

```python
def _on_monitor_start(self, job):
    """Monitor requested start — load job then start. v7.2.6 fix."""
    setup_page = self._page_widgets[4]
    if not hasattr(setup_page, "print_manager"):
        logger.error("No print_manager on setup page")
        return
    pm = setup_page.print_manager
    try:
        pm.load_job(job)   # load first — start() takes no args
        pm.start()
    except Exception as exc:
        logger.error(f"PrintManager start failed: {exc}", exc_info=True)
```

---

## 6. Monitor Live Position Update (print_monitor.py)

```python
def on_status_update(self):
    """Called by MainWindow timer (~300ms). Update live visualization."""
    if self._print_state != PrintState.RUNNING:
        return
    
    # Read cached positions
    controller = self._get_controller()
    if controller is None:
        return
    
    try:
        xy_pos = controller.get_xy_position(cached=True)
        zp_pos = controller.get_zp_position(cached=True)
    except Exception:
        return
    
    # Update trajectory view crosshair
    if hasattr(self, 'trajectory_view') and xy_pos:
        x, y = xy_pos
        self.trajectory_view.set_current_position(x, y)
    
    # Update syringe displays
    if hasattr(self, 'syringe_displays') and zp_pos:
        for pump_id, display in self.syringe_displays.items():
            axis = {"P1": "A", "P2": "B", "P3": "C"}.get(pump_id)
            if axis and axis in zp_pos:
                display.set_position(zp_pos[axis])
```

---

## 7. Dependency Graph

```
Session 1 (Job Building Pipeline)   ← CRITICAL, must be first
    │
    ├── S1.1-S1.3: Fix _build_current_job + geometry extraction
    ├── S1.4: plan_to_commands() bridge
    ├── S1.5-S1.6: Wire into _generate_print + _send_to_monitor
    └── S1.7: Fallback path for simple prints
    │
    ▼
Session 2 (Execution Chain)         ← depends on S1
    │
    ├── S2.1-S2.2: Fix app.py wiring
    ├── S2.3-S2.5: Fix Monitor receive/control
    ├── S2.6-S2.7: Position polling + bridge conflict
    └── S2.8: End-to-end trace
    │
    ▼
Session 3 (Integration + Visualization)  ← depends on S2
    │
    ├── S3.1-S3.3: Live visualization
    ├── S3.4-S3.8: Manual test scenarios
    └── S3.9: Automated integration tests
```

---

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| `build_well_plate_job()` API changed since v7.0 | Low | High | Verify actual function signature at patch time |
| Tab 2 object storage format varies between patches | Medium | Medium | Use defensive `getattr`/`hasattr` chains |
| `_wire_print_manager_to_monitor()` clobbers PrintSetup's callbacks | Medium | High | Check for existing bridge before creating new one |
| PrintPlanOfAction `validate_well_setup` import fails | Medium | Low | Already using try/except ImportError pattern |
| Service well commands use wrong coordinates (mm vs microsteps) | Medium | High | build_well_plate_job uses mm + from_zero_ref=True; verify service commands match |
| Queue auto-advance triggers before Monitor updates | Low | Medium | Use QueuedConnection for state signals |

---

## 9. What We Are NOT Changing

- **PrintManager core** (`_execute_loop`, `_execute_command`, command dispatch) — working, don't touch
- **Hardware Setup page** — working, already validated
- **WellSetupModel / WellPlate** — data layer is sound
- **PrintPlanOfAction generation logic** — plan generation works, we only add the execution bridge
- **StageController** — hardware abstraction layer is stable
- **v7.0 file loading** — `load_print_file()` still works for .json and .gcode

---

## 10. Verification Checklist

After all sessions complete:

- [ ] **AST parse** all modified files
- [ ] **Import check** — `python3 -c "from gui.pages.print_setup import PrintSetupPage"`
- [ ] **Launch** — `python main.py` starts without crash
- [ ] **Simple print** — assign 3 wells, default pattern, generate, send to monitor, start
- [ ] **Plan print** — assign wells + service wells, generate plan with multiple runs, verify service steps in command list
- [ ] **File load** — load a v7.0 .json print file, send to monitor, start
- [ ] **Pause/Resume** — pause mid-print, resume, verify completion
- [ ] **Abort** — abort mid-print, verify Z raises to travel height
- [ ] **Progress display** — verify Monitor shows step count, well name, progress bar
- [ ] **No regression** — Hardware Setup, Jog, Calibration all still work
