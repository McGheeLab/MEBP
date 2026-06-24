# MEBP v7.5.x — print-path planner-buffer hardening (the print-only ZP/USB drop)

## Objective

Fix the failure where raw motions (Z, XY, concurrent, descents, pump) run
flawlessly for a long time in the Stress Test, but a real PRINT fails almost
immediately — the ZP USB-serial `WriteFile` faults (`PermissionError(13, '...
does not recognize the command', 22)`), the print "crawls" (pump commands 4–10 s
apart), and `MOVE_Z: needle not confirmed`.

## Root cause (multi-agent diff: print path vs the passing stress components)

A workflow mapped `_execute_print_path` and diffed it against the *passing*
raw-motion stress components. The print is the ONLY thing that issues a long,
continuous, **open-loop** command stream:

- Per waypoint it sends one pump G0 + one XY G0, paced only by
  `_sleep_s = max(move_time, 0.05)` where `move_time` is the **XY** transit time
  — NOT the pump's physical duration — and with **no M400 barrier** between
  segments.
- Marlin's `ok` means "admitted to the planner buffer", not "move complete". On
  a long, finely-sampled path (hundreds–thousands of points; spiral rims
  over-sampled ~17×) the host admits faster than the board executes → the
  ~16-block planner buffer fills → the board withholds `ok`/flow-control-stalls →
  `_txn` blocks (the "crawl") → the flow-control-paused CH340 endpoint faults →
  `WriteFile ERROR_BAD_COMMAND`.

**Why the raw motions never fail (the key diff):** every passing stress
component *confirms each move* (`flush_moves`/M400 + `wait_for_z_arrival`) which
drains the planner to empty before the next command, or is net-zero/tiny/short.
Only `PRINT_PATH` chains an unbarriered open-loop stream that can saturate.
(Adversarial check noted a specific low-flow log had sub-threshold volumes → no
pump G0s, so saturation bites hardest on higher-flow/denser prints; the idle USB
drop seen separately still warrants disabling Windows USB selective suspend.)

## Changes (incremental hardening — user-chosen direction)

| File | Change |
|------|--------|
| `SupportClasses/PrintManager.py` | **(1) M400 barrier:** new class const `_PATH_BARRIER_EVERY = 8`; `_execute_print_path` calls `zp_stage.flush_moves(10s)` every 8 segments (poller+watchdog already suspended) so the planner buffer depth is bounded and can never saturate. Near-instant when shallow / when ZP has no queued moves (dry/low-flow); only waits when motion is genuinely backed up. **(2) Pace by the rate-limiting axis:** `_sleep_s = max(xy_move_time, pump_move_time, 0.05)` with `pump_move_time = seg_vol_uL / flow_rate_uL_s`, so the loop never admits pump moves faster than they execute. **(3)** PRINT_PATH wrapper now also `suspend_zp_watchdog()` / `resume_zp_watchdog()` (try/finally) alongside the poller. |
| `SupportClasses/StageController.py` | New `suspend_zp_watchdog()` / `resume_zp_watchdog()` (guarded) → pause/resume the ZP watch on the `ConnectionWatchdog`. |
| `SupportClasses/SerialUtils.py` | `ConnectionWatchdog` gains per-watch `pause(name)` / `resume(name)` (+ a `paused` flag; `_run_one_cycle` skips paused watches; `resume` resets the debounce window). So during PRINT_PATH nothing but the print thread touches the ZP COM handle (its `in_waiting`/ClearCommError racing a flow-control-paused write is a fault surface). |
| `tests/test_v75x_print_path_planner_barrier.py` | **New** (8): watchdog pause/resume (skips health check, resets fail window, safe on unknown name); PRINT_PATH suspends+resumes BOTH poller and watchdog (incl. on exception); M400 barrier fires every 8 segments, skipped when ZP disconnected, short path completes without a mid-path barrier. |

## Testing Notes

- `python -m unittest tests.test_v75x_print_path_planner_barrier` → 8 OK.
- Broad regression (print-path barrier, disconnect-during-print, watchdog-debounce, serial-flow-control, close-during-read, print-always-safe-z, print-setup-routine, multi-object-seam, quick-print, stress-test) → 109 OK.
- **Real-HW (the point):** run the failing real/stress print again with `MEBP_ZP_TRACE=full`. Expect: no more "crawl" (the barrier + pacing keep the buffer shallow), no `WriteFile ERROR_BAD_COMMAND` from saturation, and `MOVE_Z` confirms. `logs/zp_serial.log` should show steady ~1 ms acks with periodic M400s and no anomaly storm. If a drop STILL occurs during a long IDLE period (not a print), that's the separate USB-selective-suspend issue (OS setting).

## Issues & Decisions

- **Incremental first (user choice).** The deeper rethink — continuous extrusion
  per object (one pump move per object instead of per waypoint) + path
  re-segmentation + fixing `generate_spiral`'s constant-angle over-sampling —
  remains the next step if needed; this change bounds the failure structurally
  without altering the extrusion model.
- `_PATH_BARRIER_EVERY = 8` ≈ half Marlin's default 16-block buffer, so depth
  stays comfortably bounded. The barrier adds a brief, regular pause (drains
  queued moves); acceptable for robustness, and a no-op when motion keeps up.
- Watchdog pause is per-watch + debounce-reset on resume, so a check right after
  resume can't false-fire. It's a defense-in-depth isolation (the workflow
  ranked the concurrent-handle access a downstream fault *surface*, not the root
  cause — the barrier+pacing are the root-cause fix).
