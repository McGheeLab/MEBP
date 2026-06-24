# MEBP v7.5.x — atomic M400 (flush_moves): stop the bogus "M400 timed out → ZP disconnected"

## Objective

Eliminate the **software** cause of the recurring end-of-print/retract failure
where the board "goes silent" — `flush_moves: M400 timed out after 15.0s` →
`safe_travel_to: Z retract M400 timed out — ABORTING` — even though the ZP
board (a Marlin controller that runs prints for hours) is alive and the move was
trivial. The operator confirmed it is **not hardware**.

## Root cause — `flush_moves` (M400) was not atomic; the `ok` got stolen

Decisive clue from the bench log: the print *completed* with Z already at the
safe height, then the next `safe_travel_to` retract was **zero-distance** — yet
its M400 still timed out 15 s. A no-move M400 returns `ok` in milliseconds, so
the board answered; **our read missed the `ok`.**

Every other ZP serial transaction holds `_serial_lock` for the whole
write→read-until-`ok` round trip (the synchronous `send_data`, the M114 query).
`flush_moves` did **not** — it released the lock *between every `readline()`*:

```python
with self._serial_lock:            # write M400
    ... write ...
while now < deadline:
    with self._serial_lock:        # ← lock dropped between reads
        line = self.serial.readline()
    if line == "ok": return True
```

`safe_travel_to`/`ensure_retracted_to` `suspend()` the poller first, but
`suspend()` only sets a flag — a poll **already in flight** does one more M114,
and the Xbox **jog-handler** thread is never suspended at all. Either one can
acquire the lock *between* flush_moves' reads, run its own synchronous M114
transaction, and **consume the M400 `ok`** (or leave flush_moves reading the
M114 position lines). flush_moves then never sees `ok`, waits out the full
timeout, and reports a bogus "M400 timed out", which the callers escalate to
"ZP disconnected" / abort. The synchronous-`ok` change made concurrent M114
transactions hold the lock longer, widening the steal window — so it went from
rare to "always."

## Fix

`ZPStageManager.flush_moves` now holds `_serial_lock` for the **entire**
write→read-until-`ok` (atomic), exactly like the synchronous command handshake.
Any concurrent reader (straggler poller, jog handler) must wait until M400
completes, so the `ok` can't be stolen. Also accepts `ok ...` (line-number/buffer
suffix) as success and discards busy/echo lines.

Because flush_moves can now hold the lock for up to its `timeout_s` (≤15 s on a
genuinely long move), `ZPStageManager.stop()`'s close-serialization acquire
timeout was raised (8 s → **17 s**) so the close-during-read crash fix
(`MEBP_v75x_ZP_CLOSE_DURING_READ_CRASH.md`) still reliably waits the read out
instead of force-closing mid-read. In the common disconnect case the board is
already silent, so both waits are capped to ~2 s and neither actually blocks long.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | `flush_moves`: single atomic `with _serial_lock` around drain → write M400 → read-until-`ok` (was per-read lock). `stop()`: close-serialization acquire timeout 8 s → 17 s to cover flush_moves' max hold. |
| `tests/test_v75x_zp_serial_flow_control.py` | New `test_flush_moves_holds_lock_atomically` — a concurrent thread cannot acquire `_serial_lock` while flush_moves is running, and the lock frees once it returns. |

## Testing Notes

- `python -m unittest tests.test_v75x_zp_serial_flow_control` → 15 OK.
- Broad: serial-flow-control, disconnect-during-print, close-during-read, jog-clamp, position-override, dtr-no-reset, stress-test, print-always-safe-z, print-setup-routine, z-retract → 112 OK.
- **Real-HW (the report):** run prints/stress to completion with the Xbox controller connected (the jog handler is the most likely straggler). Expect: **no more `M400 timed out`** on healthy moves, and the end-of-print retract confirms instead of aborting. The Stress Test counters should stop showing spurious disconnects.

## Issues & Decisions

- This is the strongest software explanation for "M400 times out on a live
  board, even zero-distance," and it's a real atomicity bug — but confirmation
  is on the bench. If `M400 timed out` STILL appears after this, the next signal
  to capture is whether `logs/crash.log` / console shows `ZP board RESET
  detected mid-session` (would indicate a reboot) and which thread held the port.
- Complements the rest of the ZP-drop work: synchronous `ok` flow control
  (`MEBP_v75x_ZP_SERIAL_FLOW_CONTROL.md`, incl. adaptive fail-fast) + the
  close-during-read crash fix (`MEBP_v75x_ZP_CLOSE_DURING_READ_CRASH.md`). With
  all serial transactions now atomic and acked, the protocol matches how a
  reliable Marlin host behaves.
- Latent: the Xbox jog-handler thread issuing ZP reads/writes during a
  print/retract is now harmless (it waits on the lock), but suspending it around
  prints (like the poller) would reduce contention further — possible follow-up.
