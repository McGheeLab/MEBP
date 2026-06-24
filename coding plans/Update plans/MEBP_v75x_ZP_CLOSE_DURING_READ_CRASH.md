# MEBP v7.5.x — fix hard crash on the end-of-print Z-retract (close-during-read race)

## Objective

Stop the **hard Python crash** the operator hit "always … on the Z-retract-to-safe
at the end of prints." Reported context:

```
flush_moves: M400 timed out after 15.0s
safe_travel_to: Z retract M400 timed out — ABORTING, will not start XY move
… then crash (no Python traceback)
```

## Root cause — concurrent `serial.close()` during a blocked `readline()`

When the ZP board goes silent during the end-of-print retract:

- The **retract thread** (print executor / GUI) is blocked inside
  `self.serial.readline()` — `ZPStageManager.flush_moves`' 15 s M400 wait, or the
  new synchronous `ok` handshake — while **holding `_serial_lock`**.
- The **watchdog / poller thread** detects the silent board and runs
  `_handle_disconnect("ZP") → disconnect_zp() → zp_stage.stop() → serial.close()`
  — and `stop()` took **no lock**.

Calling `CloseHandle` on a USB-serial port (CH340) while another thread is mid
`ReadFile` is a hard crash on Windows (no Python traceback — hence "then crash").
The v7.5.x synchronous flow-control change widened the window (the retract now
also reads under the lock during `move_z_absolute`, and blocks ~30 s on a dead
board instead of ~15 s), so the race went from rare to "always."

`disconnect_zp` step 1 already drops the *poller's* serial ref to stop the poller
racing the close — but nothing protected the **retract thread's** in-flight read.

## Fix

`ZPStageManager.stop()` (and the XY twin `XYStageManager.stop()`) now acquire
`_serial_lock` **before** `close()`, so the close waits for any in-flight read to
finish (each `readline` self-times-out, so the lock frees) instead of closing
mid-read. After the close, the retract's next `readline` hits a closed port and
fails gracefully (already caught). Details:

- The acquire is **bounded** so shutdown can't deadlock on a wedged read; the ZP
  timeout is `DEFAULT_OK_TIMEOUT_S + 2.0` so it always exceeds the longest
  continuous lock-hold (`_read_until_ok` holds the lock across its `readline`s
  for up to `DEFAULT_OK_TIMEOUT_S`). XY uses a flat 5 s (Prior reads self-time-out
  well under that).
- **getattr-safe**: a `__new__`-built stand-in without `_serial_lock` (tests,
  partial construction) still closes rather than raising.
- Order preserved (ZP still de-asserts DTR/RTS before close — the
  `MEBP_v75x_ZP_DTR_NO_RESET` mitigation).

Plus **crash diagnostics**: `main.py` now enables `faulthandler` (all threads →
`logs/crash.log` + stderr) at startup, so any *future* hard crash leaves a full
per-thread traceback instead of nothing.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | `stop()` acquires `_serial_lock` (bounded `DEFAULT_OK_TIMEOUT_S + 2.0`, getattr-safe) around the DTR/RTS de-assert + `serial.close()`. |
| `SupportClasses/XYStage.py` | `stop()` acquires `_serial_lock` (bounded 5 s, getattr-safe) around `spo.close()` — same race, now reachable via the Stress Test's concurrent XY. |
| `main.py` | `faulthandler.enable(all_threads=True)` → `logs/crash.log` (+ stderr fallback) at startup. |
| `tests/test_v75x_zp_close_during_read_crash.py` | **New** (4): ZP + XY `stop()` blocks the close while a read holds the lock and closes once it frees; closes normally when free; getattr-safe without a lock. |

## Testing Notes

- `python -m unittest tests.test_v75x_zp_close_during_read_crash` → 4 OK.
- Broad regression: `test_v75x_zp_dtr_no_reset` (close-time DTR order), serial-flow-control, disconnect-during-print, jog-clamp, position-override, stress-test, `test_xy_motion`, quick-print, print-always-safe-z → 93 OK; full `test_v75x*` sweep clean.
- **Real-HW (the report):** run prints to completion; when the board goes silent at the end-of-print retract it must now **disconnect cleanly (no crash)** — the console/`logs/crash.log` stays empty of fatal dumps. If a crash still occurs, `logs/crash.log` will now contain the faulting thread's traceback.

## Issues & Decisions

- This is the crash fix. **Separate, still-open issue:** on a genuinely dead
  board the end-of-print retract still *hangs* ~30 s (3× `send_data` 6 s `ok`
  timeouts + `flush_moves` 15 s M400) before the disconnect resolves — slow, but
  no longer a crash. Candidate follow-up: short-circuit the retract once
  `_last_position_read_ok` is already False, and/or shorten `DEFAULT_OK_TIMEOUT_S`.
- The deeper question — *why the board goes silent at end of print* — is the
  ongoing ZP-drop investigation (`MEBP_v75x_ZP_SERIAL_FLOW_CONTROL.md` flow
  control + the hardware auto-reset disable in `MEBP_v75x_ZP_DTR_NO_RESET.md`).
  This change ensures that when it does drop, the app survives it.
