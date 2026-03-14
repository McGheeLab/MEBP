# MEBP v7.2.5 → v7.2.6 Update Plan

## Objective

Restore working print execution by fixing 7 critical breaks in the pipeline from setup to PrintManager, plus 16 secondary bug fixes for serial thread safety, jog handler lifecycle, Xbox resilience, pump safety, and print monitor redesign.

---

## Background

v7.0 could execute prints perfectly but had primitive setup. v7.2.5 has excellent setup infrastructure (HardwareConfig, WellSetupModel, PrintPlanOfAction, ink mapping) but cannot actually execute prints. The PrintManager core (`_execute_loop`, `_execute_command`, command dispatch) is unchanged from v7.0 and known-good — the failures are all in the pipeline that feeds it.

---

## Critical Fixes (7 Breaks)

### BF-1 — `_build_current_job()` API mismatch (CRITICAL)

**Files:** `gui/pages/print_setup.py`

**Symptom:** TypeError or single-dot prints.

**Root cause:** v7.2.5 calls `build_well_plate_job()` with wrong kwargs (`plate=`, `selected_wells=`) that don't match the v7.0 API (`well_positions=`, `path_points=`). Fallback uses `path_points = [(0.0, 0.0)]` — a single center point.

**Fix:** Rewrote `_build_current_job()` to extract well positions from WellSetupModel and print geometry from Tab 2 objects, calling `build_well_plate_job()` with correct v7.0 signature. Added `_generated_job` attribute for caching.

**Status:** `[x]` done

---

### BF-2 — Print object geometry never reaches job builder (CRITICAL)

**Files:** `gui/pages/print_setup.py`

**Symptom:** Every job prints a single dot at the center of each well.

**Root cause:** `_build_current_job()` never reads from Tab 2 (Print Objects). Uses `path_points = [(0.0, 0.0)]` as hardcoded fallback.

**Fix:** Created `_get_path_from_objects()` to extract geometry from Tab 2 objects + `_single_object_to_points()` to convert parametric objects (line, meander, spiral, grid) into point lists.

**Status:** `[x]` done

---

### BF-3 — PrintPlanOfAction has no execution bridge (CRITICAL)

**Files:** `SupportClasses/PrintPlanOfAction.py`

**Symptom:** Multi-run service sequences (ink loading, washing, buffering) completely ignored.

**Root cause:** High-level plan steps (WASTE, WASH, LOAD_INK, PRINT) had no conversion to PrintCommand objects.

**Fix:** Created `plan_to_commands()` bridge function with helpers: `_find_service_well()`, `_waste_commands()`, `_wash_commands()`, `_buffer_commands()`, `_load_ink_commands()`. Each service step expands into MOVE_XY + MOVE_Z + EXTRUDE + DWELL sequences. Service commands use settings feedrates.

**Status:** `[x]` done

---

### BF-4 — `_on_monitor_start()` API inconsistency (HIGH)

**Files:** `gui/app.py`

**Symptom:** Start button on Monitor page fails to begin execution.

**Root cause:** `_on_monitor_start()` called `pm.start(job)` but PrintManager expects `pm.load_job(job)` then `pm.start()` (separate calls).

**Fix:** Fixed call sequence to `load_job()` then `start()`.

**Status:** `[x]` done

---

### BF-5 — Thread-safety of progress callbacks (HIGH)

**Files:** `gui/app.py`, `gui/pages/print_monitor.py`

**Symptom:** SIGSEGV crash on macOS during print execution.

**Root cause:** PrintManager callbacks fire from daemon thread. Direct GUI calls crash PySide6 on macOS.

**Fix:** Signal bridge bounces callbacks through QObject with `QueuedConnection`. All GUI updates run on main thread.

**Status:** `[x]` done

---

### BF-6 — Print monitor missing live visualization (HIGH)

**Files:** `gui/pages/print_monitor.py`

**Symptom:** Monitor page shows no live position updates during printing.

**Root cause:** `on_status_update()` not wired for live position polling. Progress messages not parsed for well/layer info.

**Fix:** Added `on_status_update()` for live position polling. Rewrote `on_print_progress()` to parse well name and layer info from messages. Added `_current_well_name` tracking. Visual fixes: print wells change status, service wells keep role color.

**Status:** `[x]` done

---

### BF-7 — Generated jobs missing per-well print objects (HIGH)

**Files:** `gui/pages/print_setup.py`

**Symptom:** All wells get the same single-point geometry regardless of per-well print file assignments.

**Fix:** `_build_current_job()` now checks per-well print file assignments from WellSetupModel and extracts the correct geometry for each well.

**Status:** `[x]` done

---

## Secondary Bug Fixes (16)

### BF-8 — XYStage serial thread safety

**Files:** `SupportClasses/XYStage.py`

**Symptom:** PositionPoller and JogHandler race on serial port, corrupting responses.

**Fix:** Added `threading.RLock` (`_serial_lock`) to XYStage. Lock initialized BEFORE `_initialise_serial()` (which calls `send_command()` during detection). Poll interval increased 0.3s → 1.0s to reduce contention.

**Status:** `[x]` done

---

### BF-9 — Jog handler lifecycle leak

**Files:** `SupportClasses/StageController.py`

**Symptom:** Processor handlers accumulate across stop/start cycles.

**Fix:** `ZPJogHandler` and `XYJogHandler` store `_registered_handlers` list. `stop()` unregisters all handlers. Added try/except around serial calls in jog loops.

**Status:** `[x]` done

---

### BF-10 — Xbox polling worker crash on disconnect

**Files:** `SupportClasses/XboxController.py`, `SupportClasses/StageController.py`

**Symptom:** Worker exits on first controller disconnect with no auto-reconnect.

**Fix:** Replaced worker with retry loop (never exits). Automatic reconnect on failure. Periodic heartbeat status messages. Graceful pygame init failure handling. Debug handler registered in StageController.

**Status:** `[x]` done

---

### BF-11 — Xbox button mapping and status tracking

**Files:** `SupportClasses/XboxController.py`, `SupportClasses/StageController.py`, `gui/pages/dashboard.py`

**Symptom:** Mapping file not found (relative vs absolute path). Xbox status not tracked.

**Fix:** Resolve mapping file to absolute path. `XboxMappingEditor` creates default if missing. `XboxQueuePoller` tracks `_xbox_status`. Dashboard uses `xbox_status` property for display.

**Status:** `[x]` done

---

### BF-12 — Pump speed attribute dead reference (CRITICAL)

**Files:** `SupportClasses/StageController.py`, `gui/pages/jog_control.py`

**Symptom:** Pump speed slider has no effect.

**Root cause:** `_on_p_speed()` used dead attribute `pump_speed` instead of `p_speed`.

**Fix:** Fixed attribute name. Added `store_as` parameter to `_make_ctx_slider()` for reference storage. Added `_sync_speed_sliders()` for readback synchronization.

**Status:** `[x]` done

---

### BF-13 — Pump velocity not clamped to safe flow rate

**Files:** `SupportClasses/StageController.py`

**Symptom:** Pump velocities could exceed hardware-safe limits.

**Fix:** Added `_clamp_pump_flow()` helper. ZPJogHandler pump velocity handlers clamp to max safe flow when HardwareConfig available. Pump speed slider interpreted as uL/s.

**Status:** `[x]` done

---

### BF-14 — XYStage init syntax errors from patching

**Files:** `SupportClasses/XYStage.py`

**Symptom:** SyntaxError on import — empty else-bodies from previous patches.

**Fix:** Replaced entire `__init__` with known-correct version. Removed leftover bare lock statements.

**Status:** `[x]` done

---

### BF-15 — Safety limits not persistent + UI bugs (6 sub-bugs)

**Files:** `main.py`, `SupportClasses/StageController.py`, `gui/pages/settings_page.py`, `gui/pages/dashboard.py`

**Symptom:** Safety limits reset on restart. Single pump spinbox instead of per-pump. Zero-offset correction was no-op.

**Fix:** Load safety_limits from settings.json on startup. Per-pump P1/P2/P3 min/max spinboxes. Per-pump zero reset buttons with `reset_pump_zero()`. "Set from Current" quick-set buttons. ZPJogHandler clamping subtracts `zero_position`. Dashboard shows per-pump limits.

**Status:** `[x]` done

---

## Features

### F-1 — Print Monitor Complete Redesign

**Files:** `gui/pages/print_monitor.py`

**Description:** Complete redesign for v7.2.6 execution chain.

**Layout:**
- **Left:** Compact plate overview + syringe status (P1/P2/P3/needle)
- **Center:** Live XY detail view (zoomable/pannable) — completed waypoints green, next target yellow, future empty; well boundary, grid, ruler, needle crosshair + tracking error ring
- **Right:** Progress display (job/well/layer/step/ETA) + controls (Start/Pause/Abort)
- **Context:** Job queue + recording browser

**Status:** `[x]` done

---

### F-2 — Auto-Derived Motion Parameters

**Files:** `gui/pages/print_setup.py`

**Description:** PrintSettings gains new fields: `top_z_height`, fast/entry Z feedrate, service XY speed. 4-phase smart Z approach for safe needle travel.

**Status:** `[x]` done

---

### F-3 — ToupCam Camera Integration

**Files:** `gui/widgets/camera_widget.py`, `gui/pages/calibration.py`

**Description:** Dual-backend camera support (OpenCV + ToupCam). WINFUNCTYPE for Windows stdcall calling convention. `CAMERA_AVAILABLE` flag in calibration page. Camera detection deferred to button click (not auto on init).

**Status:** `[x]` done

---

## Implementation Steps

- [x] Session 1: Fix job building pipeline (_build_current_job, _get_path_from_objects, plan_to_commands)
- [x] Session 2: Fix execution chain (monitor start API, signal bridge, thread safety)
- [x] Session 3: Integration + progress parsing (well/layer parsing, visual fixes)
- [x] S1-S7: Serial thread safety, jog handler lifecycle, Xbox resilience, pump safety
- [x] Print monitor complete redesign
- [x] Auto-derived motion parameters
- [x] ToupCam camera integration
- [x] Safety limits persistence + per-pump UI
- [x] XYStage init + lock syntax fixes

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/print_setup.py` | Job building rewrite, auto-settings, geometry extraction |
| `gui/pages/print_monitor.py` | Complete redesign, live updates, progress parsing |
| `gui/app.py` | Monitor start fix, signal bridge, visualization wiring |
| `gui/pages/jog_control.py` | Speed slider references, pump speed sync |
| `gui/pages/dashboard.py` | Xbox status, per-pump safety display |
| `gui/pages/settings_page.py` | Per-pump safety spinboxes |
| `gui/pages/hardware_setup.py` | `_restoring` guard on config restore |
| `gui/pages/calibration.py` | Deferred camera detect |
| `gui/widgets/camera_widget.py` | ToupCam dual-backend |
| `SupportClasses/PrintPlanOfAction.py` | plan_to_commands() bridge, service sequences |
| `SupportClasses/StageController.py` | Debug handler, pump clamping, jog lifecycle, safety |
| `SupportClasses/XboxController.py` | Retry loop, heartbeat, status tracking |
| `SupportClasses/XYStage.py` | Serial lock, init fix, bare lock removal |
| `main.py` | Safety limits load on startup |

## Testing Notes

1. Build print job → verify geometry from Tab 2 reaches PrintManager
2. Start print → verify no SIGSEGV crash (signal bridge working)
3. During print → verify live position updates on monitor
4. Multi-run plan → verify wash/ink-load/buffer steps execute between print runs
5. Xbox controller → disconnect + reconnect → verify auto-recovery
6. Pump speed slider → verify speed changes propagate and display correctly
7. Safety limits → set → restart → verify persistence

## Issues & Decisions

- **Signal bridge for thread safety**: PrintManager fires callbacks from daemon thread. Qt requires GUI updates on main thread. QueuedConnection signal bridge is the correct PySide6 pattern.
- **plan_to_commands() as bridge function**: Keeps PrintPlanOfAction (data model) separate from PrintManager (execution). The bridge converts high-level plan steps into flat PrintCommand lists.
- **Serial lock strategy**: Added RLock but removed from get_current_position() to prevent poll-starving jog commands. Increased poll interval instead.
- **Pump safety clamping**: Clamp to max safe flow rate from syringe specs in HardwareConfig. Prevents hardware damage from overspeed.
