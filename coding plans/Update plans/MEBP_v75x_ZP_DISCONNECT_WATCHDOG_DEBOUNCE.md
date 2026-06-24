# MEBP v7.5.x — ZP disconnect-during-print, round 2: debounce the port-health watchdog + abort instead of dry-running

## Objective

The ZP board still disconnected mid-print after round 1
(`MEBP_v75x_ZP_DISCONNECT_DURING_PRINT.md`, which suspended the poller during
`PRINT_PATH`, reset the liveness fail window, and hardened the M114 read). This
round fixes the **independent** trigger that round 1 didn't touch — the
port-health watchdog — and adds the safety backstop so a print can never again
silently run against a dead board.

## Root cause (from logs/prints/*.jsonl, runs at 16:11 + 16:12 today)

- The 16:11 run's final `retract_to_safe_z:loop_end` returned in **0.011 s** —
  that only happens when `ensure_retracted_to` hits `if not is_zp_connected:
  return True`. So **ZP was already disconnected before the print ended**, even
  though the poller was suspended throughout `PRINT_PATH` (round 1). The only
  other ZP-disconnect trigger active during `PRINT_PATH` is the **port-health
  watchdog** (`SerialUtils.ConnectionWatchdog`), which ran on its own thread and
  fired `_handle_disconnect("ZP")` on the **first** `check_port_health` failure.
  `check_port_health` reads `serial.in_waiting`; on Windows that
  (ClearCommError) can raise **intermittently under the heavy write load** of a
  print — a transient, NOT a real disconnect.
- The 16:12 run's `MOVE_Z` took exactly **0.5 s** (the fixed-settle *fallback*),
  which only happens when `is_zp_connected` is False — i.e. ZP was already gone,
  and the print **silently dry-ran to a bogus `job_end: completed`** (every ZP
  move no-ops on `zp_stage=None`; XY still moves).
- The periodic M500 EEPROM save was **ruled out** — it is gated on
  `_zp_auto_save_position`, which defaults to `False`.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SerialUtils.py` | **(A)** `ConnectionWatchdog` now **debounces**: `watch(..., fail_threshold=3)` and a per-watch `fail_count`; a disconnect is declared only after `fail_threshold` **consecutive** failed health checks (was: the first `False`). A healthy check resets the window, so a single transient `in_waiting` failure under print load is ridden out while a genuine unplug (fails every check) is still caught. `_check_loop` body extracted into `_run_one_cycle()` (no behavior change) for unit-testability. |
| `SupportClasses/PrintManager.py` | **(B)** `_execute_loop` records `_zp_connected_at_start` and, if ZP was present at start but drops mid-print, **ABORTS** (state `ERROR`) and logs a `zp_disconnected` exec-log event instead of dry-running the rest of the plan. `_execute_print_path` additionally stops the path immediately on a mid-path ZP drop (don't drag the needle dry at print Z while XY keeps moving). Both gated on `_zp_connected_at_start`, so an XY-only / no-ZP job is unaffected. |
| `tests/test_v75x_zp_disconnect_watchdog_and_abort.py` | **New** — 7 tests. |

## Implementation Steps

- [x] (A) Debounce `ConnectionWatchdog` (`fail_threshold`, consecutive-fail
  counting, healthy-resets-window); extract `_run_one_cycle()`.
- [x] (B) `_execute_loop` abort + `zp_disconnected` log on mid-print ZP drop;
  `_execute_print_path` halts the path on disconnect; both gated on
  `_zp_connected_at_start`.
- [x] New test suite (7) + full `test_v75x*` regression (551 green).
- [x] Plan doc (this file) + `CLAUDE.md` row.

## Testing Notes

- `tests/test_v75x_zp_disconnect_watchdog_and_abort.py` (7, green):
  - (A) one transient failure does NOT disconnect; `fail_threshold` consecutive
    failures do (fires exactly once per edge); a healthy check resets the window.
  - (B) `_execute_loop` sets `ERROR` + logs `zp_disconnected` when ZP drops
    mid-print, and does NOT abort when ZP was never present at start; a
    `PRINT_PATH` halts (no pump moves) on a mid-path drop but runs normally when
    connected.
- Regression: full `test_v75x*` discovery = **551 tests OK** (incl. the round-1
  `test_v75x_zp_disconnect_during_print`, `test_v750_zp_reconnect_hotfix`,
  print-execution-logging, setup-routine, seam).
- **Needs real-HW verification on ME3B V1**: run the same Smile/Circles Quick
  Print and confirm the ZP no longer disconnects mid-print. If it ever does
  again, the print JSONL now carries a `zp_disconnected` event (exact step/time)
  and the console distinguishes the trigger: `[Watchdog] ZP disconnected (after
  N consecutive failed health checks)` (port-health — now means the port is
  genuinely failing for >~6 s) vs `ZP position queries failing …` (poller
  liveness).

## Issues & Decisions

- **This is the #4 backstop round 1 deliberately deferred.** Round 1's note said
  to add abort-on-disconnect only after the false positives were addressed, else
  a false positive becomes a spurious abort. With round 1 (poller) + (A)
  (watchdog debounce) removing the false positives, (B) now correctly fires only
  on a genuine drop — and even a residual false positive aborting is strictly
  better than silently dry-running to a fake "completed".
- **Debounce threshold = 3** (≈6 s at the 2 s watchdog interval). The poller
  liveness (~2.5 s) catches a genuine power-off faster anyway; the port-health
  watchdog's role is the USB-unplug case the poller can't see, where 6 s is fine.
- **Diagnostics now in place.** The disconnect was previously invisible in the
  print log (inferred from cached z / instant retract). The `zp_disconnected`
  event + the distinct console messages make the next occurrence definitively
  attributable to a specific trigger (or to a genuine hardware drop — DTR/USB/
  power — which the debounced watchdog would then correctly report).
- Builds on `MEBP_v75x_ZP_DISCONNECT_DURING_PRINT.md` (round 1) and
  `MEBP_v75x_PRINT_SETUP_ROUTINE.md`.
