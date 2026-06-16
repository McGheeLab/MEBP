# MEBP v7.5.x — Xbox Controller Input Pipeline Overhaul

## Objective

Fix two user-reported field failures with the Xbox controller driving the stages, and harden the
whole controller→stage input pipeline for correctness and real-time response:

- **S1 — periodic reset:** while holding a trigger to jog an axis, "everything resets every N
  seconds and goes to a default position for all axes" (visible on the Xbox setup page's live
  monitor *and* in actual motion).
- **S2 — trigger neutral ≠ 0:** the right trigger's neutral position is not 0 in software (the
  hardware is fine), and the axis mapped to the triggers keeps moving with **no physical input**.

## Root-Cause Analysis (verified by multi-agent audit)

The pipeline is: `xbox_polling_worker` (separate process, pygame) → `multiprocessing.Queue` →
`XboxQueuePoller` (thread) → `Processor` bus → `ZPJogHandler` / `XYJogHandler` (threads) → stages.

| # | Defect | Location | Explains |
|---|--------|----------|----------|
| F1 | **Destructive 3 s heartbeat**: every 3 s the worker calls `pygame.joystick.quit()` + `init()` + re-creates `Joystick(0)`. This wipes SDL's per-device axis state. With `SDL_JOYSTICK_HIDAPI=1`, a *held-still* trigger generates no new HID report after re-init, so `get_axis()` returns `0.0` indefinitely; with the Windows rest-offset −1.0 this normalizes to a **phantom 0.5 press**. The 0.25 s accumulation-suppression window (v7.3.4) cannot cover an event-driven state loss. | `SupportClasses/XboxController.py` heartbeat block | S1 (3 s period) + S2 (continuous motion) |
| F2 | **Trigger display unit mismatch**: the worker dispatches triggers normalized to [0,1] (LT negated to [−1,0]), but `TriggerBar.set_value` assumes raw [−1,1] and maps `(v+1)/2` → a rest dispatch of `0.0` renders the bar at **50 %**. | `gui/pages/hardware/xbox_panel.py` | S2 ("RT neutral is not 0" on screen) |
| F3 | **Broken live-monitor cache**: `XboxQueuePoller` caches `last_axis[int(msg["axis"])]`, but `msg["axis"]` is the *group name* (`"0-1"`, `"2-3"`) → `int()` raises → silently swallowed → stick pucks never update and **Calibrate Sticks always snapshots zeros**. | `SupportClasses/StageController.py` | monitor sticks dead; stick calibration ineffective |
| F4 | **Uninitialized trigger state at connect**: until the first physical trigger movement, pygame reports `0.0` for trigger axes → same phantom 0.5 as F1, present from startup. | `SupportClasses/XboxController.py` | S2 |
| F5 | **dpad-as-buttons `NameError`**: misindented debounce check references `_dlast` outside its guard → unmapped d-pad press on a hats==0 controller raises → caught by the outer `except` → **full reconnect cycle** (state nuked). | `SupportClasses/XboxController.py` | spurious resets |
| F6 | **Runaway-velocity hazard**: jog handlers hold the last commanded velocity forever; if the worker dies/reconnects mid-hold, no zero is ever dispatched and the stage keeps jogging. No staleness timeout existed. | `SupportClasses/StageController.py` | safety |
| F7 | **Deadzones/offsets apply only at connect**: panel changes persist to settings but the running worker never sees them (only the mapping file hot-reloads). | panel + `connect_xbox` | UX/correctness |
| F8 | **Both triggers map to one command** (`move_z_at_velocity`): the two groups alternately overwrite `vel_z` at 10 Hz when both are pulled → sign-flipping jitter. No combination semantics. | worker dispatch | jitter |
| F9 | **Calibrate Sticks reads dispatched values** (post-offset, post-deadzone, gated) instead of raw rest values; `calibrate_sticks()` would open a second pygame instance against the worker's. | panel + `StageController` | calibration wrong |
| F10 | **No zero-send when a group is unmapped mid-hold**: dispatch loop `continue`s without zeroing `last_sent`/handlers. | worker dispatch | stuck velocity |

## Fix Design

**Worker (`SupportClasses/XboxController.py`)**
1. Replace the destructive heartbeat with **non-destructive liveness**: drain `pygame.event.get()`
   each loop; `JOYDEVICEREMOVED` (matching our instance id) → zero-dispatch + reconnect;
   heartbeat keeps sending `alive` every 3 s and falls back to a `get_count()==0` check
   (hot-plug-aware in SDL2 without subsystem re-init; pygame 2.6.1 / SDL 2.28.4 confirmed).
   Remove the now-unneeded accumulation-suppression window.
2. **Trigger event gating**: a trigger axis is trusted only after its first `JOYAXISMOTION`
   event for the *current* joystick object; until then it reads as rest (normalized 0).
   Kills phantom 0.5 at startup and after any reconnect. (macOS rest-0.0 semantics unaffected.)
3. **Zero-on-loss**: on any exception/reconnect/device-removal, dispatch zero to every actively
   mapped axis group *before* attempting reconnection.
4. **Combined trigger dispatch**: when LT and RT groups map to the *same* command, dispatch one
   combined value (RT − LT, clamped to [−1,1]) under a synthetic `"4+5"` key; different commands
   keep independent dispatch. `last_sent` entries remember their command so any group that goes
   inactive (unmapped/remapped mid-hold) gets a final zero-send (also fixes F10).
5. Fix the d-pad-as-buttons debounce indentation (F5).
6. **Monitor messages**: every averaging interval send `{"monitor": {"sticks": {0..3: raw},
   "triggers": {4,5: gated normalized pull}}}` so the GUI shows physical truth, decoupled from
   control dispatch.
7. **Live tuning channel**: optional `ctrl_queue` lets the GUI push `axis_deadzones`,
   `stick_offsets`, `debug_mode` to the running worker (no reconnect needed).
8. Optional `stop_event` so `disconnect_xbox` can cleanly stop thread-mode workers (was
   documented as impossible).

**Consumer (`SupportClasses/StageController.py`)**
9. `XboxQueuePoller`: handle `monitor` messages into `last_axis` (real per-axis keys); remove the
   broken `int(group_name)` cache on dispatch messages.
10. `ZPJogHandler` / `XYJogHandler`: **staleness watchdog** — if no velocity command arrives for
    0.5 s while velocity is non-zero, zero it (worker re-sends held values every 0.1 s; these
    Processor commands have no other senders — verified).
11. `StageController.update_xbox_tuning(...)` + `connect_xbox` wiring for the ctrl/stop channels.

**GUI (`gui/pages/hardware/xbox_panel.py`, `stage_panel.py`, `control_panel.py`, `main.py`)**
12. `TriggerBar.set_value` takes the [0,1] pull directly; `_tick_monitor` defaults 0.0.
13. Deadzone/offset/debug changes push live via `update_xbox_tuning` (still persisted).
14. Calibrate Sticks now snapshots true raw rest values (fed by monitor messages) and pushes the
    new offsets live.
15. Headless `main.py` builds the same per-axis deadzone dict from settings as the GUI.

## Files Modified

- `SupportClasses/XboxController.py` — worker overhaul (items 1–8); `load_xbox_mapping` gains a
  `fallback` so a transient mapping-file read/parse failure can't silently unmap every input
- `SupportClasses/StageController.py` — poller (`monitor` handling, `on_lost`) + jog staleness
  watchdogs + `update_xbox_tuning` + ctrl/stop wiring in `connect_xbox`/`disconnect_xbox` +
  `_zero_jog_velocities` helper + `_calibrate_zero` logging reads switched to cached (the fresh
  re-reads ran on the Processor dispatch thread and stalled queued velocity commands —
  head-of-line blocking felt as "freeze then replay" when pressing the zero button mid-jog)
- `gui/pages/hardware/xbox_panel.py` — TriggerBar [0,1] contract + live tuning push + raw
  calibration snapshot (12–14)
- `gui/widgets/xbox_mapping_editor.py` — atomic mapping save (`_atomic_save_mapping`, temp file +
  `os.replace`) so the worker's 5 s hot-reload can never read a half-written file
- `main.py` — headless deadzone/debug/timeout parity with the GUI connect paths (15)
- `tests/test_xbox_bluetooth.py` — `__test__ = False` marks (manual HW diagnostic; pytest was
  collecting its `test_*` functions and probing pygame during suite runs)
- `tests/test_v75x_xbox_input_pipeline.py` — new test suite (19 tests)

(`gui/pages/hardware/stage_panel.py` / `control_panel.py` needed no changes — both connect paths
already pass settings-derived deadzones, and the new ctrl/stop channels are created inside
`connect_xbox`.)

## Implementation Steps

- [x] Update plan created
- [x] Worker: non-destructive heartbeat + event-driven device loss (JOYDEVICEREMOVED + get_count)
- [x] Worker: trigger event gating (`_armed_triggers`, cleared per joystick object)
- [x] Worker: zero-on-loss + per-key `(value, command)` tracking + combined same-command dispatch
- [x] Worker: dpad-as-buttons indentation fix
- [x] Worker: monitor messages + ctrl_queue + stop_event + mapping-load fallback
- [x] Poller: monitor handling, removed broken `int(group_name)` cache, `on_lost` callback
- [x] Jog handlers: 0.5 s staleness watchdog (ZP + XY; XY also sends the explicit (0,0) stop)
- [x] StageController: `update_xbox_tuning` + connect/disconnect wiring + `_calibrate_zero` fix
- [x] GUI: TriggerBar contract, live tuning push, calibration snapshot, atomic mapping save
- [x] main.py headless parity
- [x] Tests written and green (19/19)
- [x] Full test suite run — no regressions (pre-existing failures only: sim state-persistence
      flakes in `test_sim_vs_hardware` and MagicMock-comparison errors in
      `test_v726_print_execution` / `test_v73_trajectory_planner`, all in untouched modules)

## Testing Notes

- New `tests/test_v75x_xbox_input_pipeline.py` (19 tests) runs the **real worker loop** against a
  fake `pygame` injected via `sys.modules` (the worker imports pygame inside the function body):
  phantom-trigger gating, release zero-send, combined-trigger cancellation, non-destructive
  heartbeat (no `joystick.quit()` in steady state + `alive` still emitted), device-removal →
  zero-dispatch-before-reconnecting + gate re-closed after reconnect, dpad NameError fix, live
  deadzone update, stop_event, unmap-mid-hold zero-send, mapping fallback — plus poller monitor /
  on_lost tests, ZP/XY watchdog tests, TriggerBar contract, atomic-save test.
- The audit's empirical HW probe (this machine, controller attached) confirmed: all axes read
  0.0 at fresh open and stay 0.0 indefinitely while idle, including after a simulated heartbeat
  re-init — i.e. the 0.25 s suppression window could never work; only event gating can.
- **Needs real-HW verification**: hold a trigger > 6 s (no reset), trigger bars rest at 0 and
  stick pucks now actually move, no Z motion with hands off, deadzone slider applies live,
  controller power-off mid-jog stops motion ≤ ~0.5 s, Calibrate Sticks captures non-zero drift.

## Issues & Decisions

- The worker docstring claimed "proven code — do not change". The destructive heartbeat *was* the
  root cause of both field symptoms (empirically verified); the v7.2.8 goal it served (silent
  Bluetooth-loss detection) is preserved via SDL2 hot-plug events + a non-destructive
  `get_count()` check (hot-plug-aware in SDL2 ≥ 2.0 once events are pumped; pygame 2.6.1 /
  SDL 2.28.4 here). The `_find_controller` quit/init retry idiom (proven macOS-BT detection,
  v7.2.7) is retained for initial connect/reconnect only.
- Trigger rest-offset remains platform-hardcoded (−1.0 on Windows/Linux, none on macOS) per
  v7.3.4's finding that rest-sampling is unreliable; event gating now covers the state-unknown
  window that normalization alone could not. On macOS (rest = 0.0) gating is bypassed by design.
- Same-command group combination is per-type only (two triggers, or two sticks). A mixed
  stick+trigger sharing one command keeps independent dispatch (last-writer-wins), as before.
- The poller-side velocity zeroing on disconnect (`on_lost`) **reinstates** a v7.3.4-documented
  behavior that had been lost from `_poll_loop`; the jog-handler watchdog is defense-in-depth on
  top (worker death, queue stall).
- Deferred (candidate future hardening): the ZP jog clamp block is `except Exception: pass`, so
  a failed position read lets a segment through **unclamped**; making it skip the segment instead
  would be safer but could brick jogging on flaky position reads — needs a deliberate decision.
- Per-group `axes_invert` (uncommitted v7.5.x working-tree feature) preserved; it applies after
  LT-negation/deadzone and before combination.
