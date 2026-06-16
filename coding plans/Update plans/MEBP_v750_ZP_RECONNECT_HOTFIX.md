# MEBP v7.5.0 — ZP/Pump Reconnect Hotfix

## Objective

Fix the bug where, after the ZP (Marlin Z + pumps) controller disconnects and is
reconnected, the GUI badge reads **"Connected"** but Z/pump movement is dead, and
**the only way to recover is a full application restart**. The XY (Prior ProScan)
stage is a separate controller and is unaffected throughout.

## Root cause

Two layers of defect, asymmetric with the XY stage:

1. **Silent false-positive (certain).** `ZPStageManager._initialise_serial()`
   *returns `None`* (logs a warning, does not raise) when no Marlin is acquired,
   unlike `XYStage._initialise_serial()` which **raises** `ConnectionError`. The
   constructed `ZPStageManager` is kept with `serial = None`; `connect_stages`
   only catches exceptions so it never notices; `is_zp_connected` only checked
   `zp_stage is not None`. Result: badge "Connected", but every command no-ops in
   `send_data` (`if self.serial is None: return`). The movement path has no other
   gate (`move_z_relative`/`move_pump_relative` only guard `if not self.zp_stage`),
   so a dead serial is the *only* way jogging silently fails.

2. **In-process state not released on disconnect (the "never until restart"
   part).** Contributing defects: the watchdog's `check_port_health` is blind to a
   board powered off with USB still attached (port stays `is_open`,
   `in_waiting == 0`) so `disconnect_zp` may never run; `disconnect_zp` didn't drop
   the poller's ZP reference before closing (close raced the poll thread), didn't
   `remove_periodic` the M500 callback (leaked one per connect), and didn't clear
   the cached port; `on_disconnect` was declared but never assigned; and
   `disconnect_xy`/`disconnect_zp` were each defined twice (dead first copies).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | Layers 1–4: serial-liveness check in `connect_stages`; hardened `is_zp_connected`; robust `disconnect_zp` (poller-ref-drop first, `remove_periodic`, clear `_preferred_zp_port`); deleted duplicate `disconnect_xy`/`disconnect_zp` defs; thread-safe/idempotent `_handle_disconnect` (+ `_disconnect_lock`); `PositionPoller` poller-driven liveness (`on_zp_lost`, fail counter scaled from poll interval) |
| `SupportClasses/ZPStage.py` | Layer 3 support: `_last_position_read_ok` flag set in `_parse_position`; Layer 5: INFO-level probe diagnostics in `_try_open_marlin` (open-failure OS error + M115-no-answer) |
| `gui/app.py` | Layer 4: `stage_disconnected` Signal; `controller.on_disconnect` wired to emit it (worker thread) → `_on_stage_disconnected` slot refreshes status immediately on the GUI thread |

## Implementation Steps

- [x] **Layer 1 — truthful status.** After `ZPStageManager(...)` in
  `connect_stages`, if `not sim_zp and serial is None` → stop, null `zp_stage`,
  fire `on_disconnect("ZP")`, return (mirrors XY's caught-exception path). Harden
  `is_zp_connected` to require a live serial (sim short-circuits).
- [x] **Layer 2 — full resource release.** Rewrite surviving `disconnect_zp`:
  drop poller ZP ref first, `unwatch("ZP")` + `remove_periodic(...)`, stop jog,
  stop stage (close serial), null, clear `_preferred_zp_port`. Delete the dead
  first `disconnect_xy`/`disconnect_zp` definitions.
- [x] **Layer 3 — power-off detection.** `PositionPoller` tracks consecutive
  failed ZP position reads (via new `ZPStage._last_position_read_ok`, because
  `get_current_position()` returns stale floats not `None` on a dead board) and
  fires `on_zp_lost` → `_handle_disconnect("ZP")` after ~2.5 s. `_handle_disconnect`
  is now serialized (`_disconnect_lock`) and idempotent (guards on stage `is None`).
- [x] **Layer 4 — GUI push.** `MainWindow.stage_disconnected` Signal emitted from
  the `on_disconnect` callback (worker thread) → queued slot refreshes
  `_update_status()` immediately. (Existing ~300 ms tick already covers it via the
  now-truthful `is_*_connected`; this just removes the latency.)
- [x] **Layer 5 — diagnostics.** Promote the ZP probe open/M115 logs to INFO so a
  failed reconnect shows the reason (port held vs. board not answering M115).
- [ ] **Layer 6 (conditional).** If Layer 5 logs show the board reboots on every
  port-open (DTR reset) and misses the M115 window, open the probe with DTR
  de-asserted and/or lengthen/repeat M115. Deferred until the log confirms it.

## Testing Notes

- **Compile:** `py_compile` clean on all three files.
- **Sim smoke test:** connect→jog→`disconnect_zp` (full teardown, `_preferred_zp_port`
  cleared)→reconnect succeeds (proves the line-901 guard is freed); liveness
  threshold scales to 8 at the default 0.3 s interval; `on_zp_lost`/`on_disconnect`
  wired; `_handle_disconnect` idempotent.
- **Sim suites:** `tests/test_v75x_axis_map_jog.py` (8/8 pass). `tests/test_sim_vs_hardware.py`
  + `tests/test_hybrid_execution.py`: 2 pre-existing failures
  (`test_04_zp_create_multiple`, `test_12_initial_position_near_zero`) — verified
  identical on stashed/original code; they are simulator state-persistence flakes,
  unrelated to this change.
- **Real hardware (to run on the rig):** With Layer 5 logging, reproduce the
  unplug AND power-off cycles. Badge must flip to disconnected within ~2.5 s, and
  pressing Connect after fixing the hardware must reconnect + jog **without an app
  restart**. Hammer test: alternate unplug/power-off → Connect several times in one
  session.

## Issues & Decisions

- **Chose controller-layer handling over making `_initialise_serial` raise.** Two
  real-HW test helpers (`tests/test_sim_vs_hardware.py:669`,
  `tests/check_serial_bugs.py:247`) construct `ZPStageManager(simulate=False)` and
  inspect `.serial` for graceful degradation; raising would silently invert their
  contract. The controller-layer check keeps `ZPStageManager` tolerant.
- **`get_current_position()` returns cached floats on a dead board**, so liveness
  can't key off `pos[0] is None`; added the explicit `_last_position_read_ok` flag.
- **Liveness threshold ~2.5 s** balances catching a (permanent) power-off quickly
  against tearing down on a transient board reset.
- The exact Windows/board-level trigger for "never until restart" can't be pinned
  from source alone; Layer 5 logging will confirm it on the rig, with Layer 6 as a
  targeted follow-up if needed.
