# MEBP v7.5.x — robust ZP serial logging + stress-print component isolation

## Objective

Two things, both to nail down the intermittent ZP comms fault:

1. **Make logging much more robust** — we'd been blind to what the ZP serial
   link was actually doing (only the *outcome* — "M400 timed out" /
   "disconnected" — was logged, never the traffic). Add a dedicated serial
   trace, persist the app log to a file, and record ZP comm health in the print
   log.
2. **Break the stress print into individually-testable components** so the
   operator can isolate which command (or combination) breaks the board.

## Part 1 — robust logging

| File | Change |
|------|--------|
| `SupportClasses/ZPSerialTrace.py` (**new**) | `ZPSerialTracer` (lazy singleton via `tracer()`) → dedicated rotating file `logs/zp_serial.log`, **off the main console/app log** (`propagate=False`). Records per-command **transactions** (command, ack latency ms, outcome ∈ ok/timeout/error/reset/read_error/write_error, rx-line count, busy count) and, in `full` mode, every TX/RX line. Keeps an in-memory **ring of the last 300 transactions**; on an anomaly (timeout/error/reset/slow-ack ≥ 750 ms) or an `event()` (disconnect/reset) it **dumps the ring** so the lead-up is captured even in summary mode. Rolling `STATS` line every 250 txns. Verbosity via env `MEBP_ZP_TRACE` = `0` (off) / `1`/default (summary: anomalies + ring + stats) / `full` (every line). Never throws. |
| `SupportClasses/ZPStage.py` | `_txn` / `_read_until_ok` / `flush_moves` now feed the tracer: TX on write, RX per line, a transaction summary with measured latency + outcome. `_read_until_ok` returns `(ok, text, stats)` (only caller is `_txn`). **`flush_moves` records `rx_lines`** on timeout + a `SILENT` vs `no-ok` note — the single most useful M400 diagnostic: **rx=0 ⇒ board genuinely silent**, **rx>0 ⇒ the `ok` was stolen / move still running** (live board). A board-reset banner emits a tracer `event`. |
| `main.py` | `setup_logging` now adds a **`RotatingFileHandler` → `logs/app.log`** (always DEBUG, 8 MB × 5) alongside the console, so a session's full log survives the terminal scrolling / the app exiting. (Plus the `faulthandler` → `logs/crash.log` added earlier.) |
| `SupportClasses/PrintExecutionLogger.py` | The 5 Hz `sample` events now also carry ZP **comm health** — `zp_cmd` / `zp_ok` / `zp_ok_fail` / `zp_reset` (from `get_comm_counters`) + `zp_conn` — so a post-hoc analysis sees exactly **when** acks started failing relative to the motion (no extra serial traffic). |

## Part 2 — stress-print component isolation

`gui/pages/workflows/stress_test_workflow.py`: new individually-selectable print
sub-operations (enable ONE at a time to binary-search the culprit):

- **Z descent + confirm** (`_descend_burst` + `_z_move_confirm`) — reproduces the
  print's MOVE_Z in isolation: drive Z **down** by a configurable *descent depth*
  below Safe Z (polarity-safe via `zref_to_user_z`/`user_z_to_zref`, clamped by
  soft limits + plate floor), M400 + wait-for-arrival confirm, then retract and
  confirm — looped. This is the one motion travel/jog never do; the prime suspect.
- **Pump extrude** (`_extrude_burst`) — the print's EXTRUDE in isolation: a
  net-zero pump command stream.

New config: `do_descend` + `descend_depth_mm` (default 5 mm, safe) + `do_extrude`
+ `component_reps`. Wired into `_run` as their own phases (with the safety-stop
`check_link` between them), and into the start gate. Combined with the trace +
the print-log comm counters, the operator can run e.g. "descent-only" and see in
`logs/zp_serial.log` exactly which transaction degrades.

## Testing Notes

- `python -m unittest tests.test_v75x_stress_test_workflow` → 19 OK (new:
  descent goes down-then-up + confirms; extrude is net-zero).
- Tracer smoke: `full` mode writes TX/RX + TXN summaries + ring dump on event;
  the `SILENT` (rx=0) vs `no-ok` (rx>0) flag is captured on M400 timeout.
- Broad ZP/print/stress regression → 110 OK. `_read_until_ok` 3-tuple change has
  the single caller `_txn`.
- **Real-HW use:** reproduce the failure with **only "Z descent + confirm"**
  enabled. Then read `logs/zp_serial.log`: the anomaly line + the dumped ring
  show the exact transaction and latency where the link degraded, and the M400
  `SILENT`/`no-ok` flag says whether the board went quiet or its `ok` was eaten.

## Issues & Decisions

- The trace is on its own logger (`propagate=False`) and own file so `full` mode
  can be byte-verbose without drowning `app.log`; default summary mode is cheap
  (a deque append + a write only on anomalies).
- Component bursts issue the sub-operations **directly via the controller**
  (mirroring the discrete print handlers — e.g. `_z_move_confirm` == the MOVE_Z
  handler) rather than building partial `PrintManager` jobs, so each is a clean
  isolation of one command's serial pattern.
- Descent depth defaults to a safe 5 mm below Safe Z; the operator can raise it
  toward the real print descent cautiously (clamped by soft limits + plate floor).
