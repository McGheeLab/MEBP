# MEBP v7.5.x — ZP serial flow control (Marlin `ok` handshake) — Phase 1

## Objective

**Make the ZP (Marlin Z+pump) board stop dropping mid-operation by talking to it
the way every reliable printer host does — synchronous `ok` flow control.**

The board is a Marlin 3D-printer controller; Marlin runs print jobs for *hours*
without dropping. The difference between that reliability and our repeated
"ZP disconnected" failures is the host protocol. Marlin is a strict
request/response protocol: the host sends a G-code line, Marlin admits it to its
planner buffer and replies `ok`, and **the host must wait for that `ok`** before
sending the next line. Our `ZPStage.send_data` fired commands blind:

```python
self.serial.write(encoded); self.serial.flush()   # never read Marlin's 'ok'
```

Under any dense/sustained command stream this outruns Marlin's small AVR serial
RX buffer (~128 B) and planner block buffer; bytes are dropped, commands
corrupt, the board desyncs and goes silent — which surfaces to us as a
mid-print/mid-travel disconnect. The code already half-knew this (the M114 path
had a workaround to "drain stale acks" that `send_data` never read).

This is **Phase 1** of the user-approved staged plan ("full protocol, staged"):
synchronous `ok` flow control now; **Phase 2** (line numbering + checksum +
automatic `Resend` on corruption — the rest of the Marlin reliability protocol)
to follow after Phase 1 is validated on the real ME3B V1 board.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | **`send_data(data, wait_ok=True)`** now, on **real hardware**, BLOCKS until Marlin acknowledges with `ok` via the new synchronous primitive; returns `bool` (was `None`; callers ignore it). Simulator and `wait_ok=False` keep fire-and-forget. New **`_txn(line, *, ok_timeout, collect)`** — write + read-until-`ok`, the whole round trip serialized under `_serial_lock` (RLock) so a concurrent reader can't steal the `ok`. New **`_read_until_ok(ok_timeout, collect)`** — classifies each line: `ok`→success; `error…`→reject; `busy`/`echo:busy`→host keep-alive, board alive, extend the wait window; reset banner (`start`/`Marlin`/`FIRMWARE_NAME`)→**board reset detected** (`_board_reset_detected=True`, position counter lost); silence past deadline→fail. **`get_current_position`** real-HW path rewritten to one synchronous `M114` transaction (`_txn(..., collect=True)` → parse the position line, consume the `ok`); the old "drain stale acks before M114" is gone because the buffer is now always clean. New `DEFAULT_OK_TIMEOUT_S=6.0`, `_RESET_MARKERS`, `_board_reset_detected` flag. A command that never gets `ok` clears `_last_position_read_ok` so the existing poller-liveness watchdog escalates a silent board fast. |
| `tests/test_v75x_zp_serial_flow_control.py` | **New** — the synchronous handshake itself (see Testing Notes). |
| `tests/test_v75x_zp_disconnect_during_print.py` | `#3` section rewritten: M114 is now a synchronous `readline`-until-`ok` transaction (no pre-drain). `_FakeSerial` is line-oriented; tests cover position parse, no-pre-drain, busy-keepalive-≠-dead, dead-board-failed-read. Poller suspend/resume tests (`#1`,`#2`) unchanged. |
| `tests/test_v75x_zp_jog_clamp_freeze.py`, `tests/test_v75x_zp_position_override.py` | Fake serials gained `readline()`→`b"ok\n"` so they model the handshake (move/override commands complete synchronously). |

## Why `ok`-waiting does NOT slow motion

`ok` means "admitted to the planner buffer", **not** "move finished" — Marlin
returns it in milliseconds. The only time it's delayed is when the planner
buffer is full on a dense burst, in which case waiting is *exactly* the flow
control we want (it paces us to the board's capacity instead of overflowing it).
Motion completion is still waited on separately via `flush_moves` (M400). In
this app commands are either small/sparse (jog, setup) or paced by the print
loop's own per-segment sleeps, so the added per-command latency is negligible.

## Implementation Steps

- [x] `_read_until_ok` line classifier (ok / error / busy / reset / silence).
- [x] `_txn` write+read-until-ok under the serial lock.
- [x] `send_data` → synchronous on real HW; simulator/`wait_ok=False` unchanged; returns bool; clears `_last_position_read_ok` on failure.
- [x] `get_current_position` real-HW path → single synchronous M114 transaction; drop the stale-ack drain.
- [x] `_board_reset_detected` flag + reset-banner detection.
- [x] Update/extend tests; new dedicated suite.
- [x] Regression: full `test_v75x*` = 717/718 (the 1 failure is the pre-existing CV `test_real_24_well_mosaic`, unrelated — `MosaicBuilder`/`VisionDetector` already modified in the working tree); `test_sim_vs_hardware`, `test_xy_motion` green.

## Testing Notes

- `python -m unittest tests.test_v75x_zp_serial_flow_control` — new suite.
- `python -m unittest tests.test_v75x_zp_disconnect_during_print tests.test_v75x_zp_jog_clamp_freeze tests.test_v75x_zp_position_override tests.test_v75x_zp_dtr_no_reset tests.test_v75x_zp_envelope_absolute tests.test_v75x_axis_map_jog tests.test_v75x_zp_position_restore` → 63 OK.
- **Needs real-HW verification on ME3B V1 (the whole point):** run several prints and long jog/travel sessions. Expect: NO mid-operation "ZP disconnected (poller-driven liveness)". If a real drop still occurs, the console now distinguishes the cause — a `ZP board RESET detected mid-session` line means a brownout/auto-reset (→ hardware), whereas a silent `ok` timeout means the board stopped answering. Watch command throughput is still acceptable for jog/print.

## Issues & Decisions

- **Scope = real hardware only.** The simulator answers every command and does
  not model RX-buffer backpressure, so the handshake adds nothing there and the
  whole test suite relies on the non-blocking write semantics. `send_data`
  branches on `self.simulate`.
- **Board-reset detection** is the bonus safety win: if Marlin resets mid-session
  (the historical "drops"), we now SEE the boot banner while awaiting `ok`, flag
  it, and fail the command → the poller escalates to a disconnect → the existing
  reconnect + ZP-position-restore flow re-declares position. Previously a reset
  was invisible and we'd continue against a wrong/zeroed position.
- **Lock = RLock** (already was) so a transaction can be re-entered safely; the
  round trip holds the lock so the poller serializes behind a move's `ok`
  (correct — bounded by `ok_timeout`).
- **This is the primary fix for the recurring ZP disconnect.** It supersedes the
  symptom-patches (drain-before-M114, poller suspend during PRINT_PATH, watchdog
  debounce) as the *root-cause* fix, though those remain as defense in depth.
- **Phase 2 (next):** line numbers (`Nn`) + checksum (`*c`) + `M110` reset at
  connect + `Resend: N` handling with a small replayable line buffer. This makes
  the link robust to *corrupted* bytes (not just overflow), which is the last
  gap for "never drops". Deferred until Phase 1 is bench-confirmed so we change
  one variable at a time.
- **Addendum (2026-06-18) — adaptive fail-fast timeout.** Bench stress-testing
  showed that once a marginal board starts missing acks, paying the full 6 s
  `ok` timeout (and 15 s M400) on *every* subsequent command cascaded into
  multi-second hangs across a whole print/travel — and because the poller reads
  ZP+XY in one cycle, the stalled ZP reads even starved the XY position reads
  (an XY settle timed out at 10 s; a 2.5 s stress print took 25.9 s). Fix:
  `send_data`'s `ok` wait is now **adaptive** — full `DEFAULT_OK_TIMEOUT_S` while
  the board is acking, but drops to `SILENT_OK_TIMEOUT_S` (0.5 s) once it's gone
  silent (`_last_position_read_ok` False), and a successful ack restores both the
  full timeout and the liveness flag. `flush_moves` (M400) is likewise capped to
  ~2 s when the board is already known silent. Healthy boards are unaffected
  (`ok` returns in ms). This is the resolution of the "dead-board retract hangs
  ~30 s" follow-up noted in `MEBP_v75x_ZP_CLOSE_DURING_READ_CRASH.md`. Tests:
  `TestAdaptiveTimeoutFailFast` (3) in `tests/test_v75x_zp_serial_flow_control.py`.
- **Hardware still matters for a true "never".** If bench testing shows
  `ZP board RESET detected mid-session`, the cause is a brownout/auto-reset, not
  the protocol — pursue the hardware auto-reset disable (`MEBP_v75x_ZP_DTR_NO_RESET.md`:
  cut `RST-EN` / 10 µF RESET→GND), a clean/short USB cable + powered hub, and
  separate motor PSU from USB 5 V. Software flow control cannot prevent a power
  brownout; it can only detect it instantly and recover.
