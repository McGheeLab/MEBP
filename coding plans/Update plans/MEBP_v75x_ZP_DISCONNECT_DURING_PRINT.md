# MEBP v7.5.x — Fix ZP stage disconnecting during a print

## Objective

Stop the ZP (Marlin Z + pump) board from being marked **disconnected mid-print**
(reported via Quick Print, but the mechanism is general). XY/ProScan is
unaffected — only ZP drops.

## Root cause (proven from the execution logs)

A 6-reader investigation workflow + log analysis (`logs/prints/*.jsonl`) found
**hard evidence**: the cached ZP z-position goes `None` reproducibly ~4–8 s into
the `PRINT_PATH` segment loop (B2 @≈5.4 s, B4 @≈9.2 s, PIAR @≈8.8 s) and stays
`None`, **yet every job still logs `job_end: completed`**. The cached ZP tuple
only becomes `(None,None,None,None)` when `set_stages(zp_stage=None)` runs inside
`disconnect_zp()` — so a real `_handle_disconnect("ZP")` fired mid-print **as a
false positive** while the board was still alive (the pump kept being commanded;
the print "completed").

Mechanism: during `PRINT_PATH` the discrete executor issues a dense per-segment
ZP write stream (one pump `G0` per segment, ~1 write/100 ms), while the
`PositionPoller` is left **active** and keeps sending `M114` every 0.3 s. They
share `ZPStageManager._serial_lock` but lock independently (no atomic
write-then-read), and `get_current_position()` did **not** drain the RX buffer
before `M114` while `receive_data()` only slept 10 ms then did a single
non-blocking `read_all()`. Under the write stream the `M114` reply isn't in the
RX buffer at read time → `_parse_position` finds no position line →
`_last_position_read_ok=False`. After `_zp_fail_threshold = max(5, int(2.5/0.3)) = 8`
consecutive failures (~2.4 s) the poller fires `on_zp_lost → _handle_disconnect('ZP')`.
Compounding it: `PositionPoller.resume()` never reset `_zp_fail_count`, so
failures accrued during a path carried across the next suspend/resume boundary.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | **#2** `PositionPoller.resume()` now resets `_zp_fail_count` (fresh liveness window, mirrors `set_stages`). **#1 plumbing** new guarded `StageController.suspend_position_poller()` / `resume_position_poller()`. |
| `SupportClasses/PrintManager.py` | **#1** the discrete `PRINT_PATH` branch of `_execute_command` wraps `_execute_print_path(cmd)` in `suspend_position_poller()` → … → `resume_position_poller()` (try/finally) so the poller's `M114` reads don't contend with the per-segment write stream. |
| `SupportClasses/ZPStage.py` | **#3** `get_current_position()` now `reset_input_buffer()` before `M114` (drops stale `G90/G0/G91` acks) and does a **bounded wait (≤0.25 s, early-exit)** for a complete, parseable reply — a *busy* board is no longer mistaken for a *dead* one. Lock released between reads so writes aren't starved; a truly absent reply still reports the failure. |
| `tests/test_v75x_zp_disconnect_during_print.py` | **New** — 11 tests. |

## Why each fix targets the right mode

- **Discrete (Quick Print)** — the proven failing path. Its poller cache is
  display-only during `PRINT_PATH` (the print is open-loop), so **suspending the
  poller (#1)** is safe and removes the contention entirely.
- **Trajectory / Velocity / Hybrid** — their `PrintRecorder` reads the poller's
  cached *actuals* per waypoint, so suspending the poller would corrupt the
  recording. These are protected instead by **#3 (hardened read)** + **#2
  (resume reset)**, which fix the proximate trigger (the `M114` read failing
  under write contention) without suspending.

## Implementation Steps

- [x] #2 — `PositionPoller.resume()` resets `_zp_fail_count`.
- [x] #1 plumbing — `StageController.suspend_position_poller` / `resume_position_poller` (guarded).
- [x] #1 — discrete `PRINT_PATH` wraps execution in suspend/resume (try/finally).
- [x] #3 — `ZPStage.get_current_position()` drains RX + bounded-waits for a complete reply.
- [x] New test suite (11) + full `test_v75x*` regression (510 green).
- [x] Plan doc (this file) + `CLAUDE.md` table row.
- [ ] **Deferred (recommended next, post-bench):** #4 — surface/abort a *genuine*
  ZP disconnect during a print (today it silently dry-runs to a bogus
  `completed`). Must land AFTER #1–#3 are bench-confirmed, else a residual false
  positive would become a spurious print abort (strictly worse).

## Testing Notes

- `tests/test_v75x_zp_disconnect_during_print.py` (11, green):
  - #2: `resume()` resets `_zp_fail_count` + clears `_suspended`; threshold is
    `max(5,int(2.5/0.3))=8`; a fresh window after `resume()` needs the full
    threshold again (one later failure can't immediately fire `on_zp_lost`).
  - #1: `suspend/resume_position_poller` are guarded (no poller → no raise) and
    delegate to the poller; the discrete `PRINT_PATH` suspends then resumes,
    **resumes even on exception**, and a non-`PRINT_PATH` command never touches
    the poller.
  - #3: `get_current_position()` drains the RX buffer before `M114`, parses a
    reply that arrives **after** stale `ok`s (busy board ⇒ `read_ok=True`), and
    still reports a failed read when nothing comes back (dead board ⇒ `False`).
- Regression: full `test_v75x*` discovery = **510 tests OK** (incl. the existing
  `test_v750_zp_reconnect_hotfix`, `test_v75x_zp_jog_clamp_freeze`,
  `test_v75x_zp_dtr_no_reset`, print-execution-logging, Quick Print, seam).
- **Needs real-HW verification on ME3B V1**: run the Circles/B2 Quick Print and
  confirm the exec log keeps a non-`None` ZP z through `job_end` and the ZP no
  longer disconnects mid-print; sanity-check a trajectory/hybrid print too.

## Issues & Decisions

- **Suspend only the discrete path; harden the read for the rest.** Suspending
  the poller during trajectory/velocity would freeze the recorder's actuals, so
  those modes rely on #3 + #2 instead (read survives contention). This is why #3
  is not merely "hardening" — it is the primary protection for non-discrete modes.
- **try/finally is mandatory** around the `PRINT_PATH` suspend so an
  abort/exception always resumes the poller; otherwise the live position display
  (and liveness watchdog) would stay frozen for the rest of the session.
- **Bounded `M114` read budget = 0.25 s**, under the poller's 0.3 s interval,
  with early-exit on first parse and the lock released between reads — so it
  cannot starve a concurrent pump write or make the poller fall behind. A true
  power-off still reports `_last_position_read_ok=False` within the budget, so
  the legitimate disconnect detection (the v7.5.0 reconnect hotfix) is preserved.
- **`reset_input_buffer` is added ONLY in `get_current_position`** (M114 query),
  never inside `flush_moves` (which already drains correctly for its `M400`).
- **#4 (abort-on-disconnect) deliberately deferred** per the investigation's
  explicit sequencing: layering an auto-abort on top of *unfixed* false positives
  would convert every false positive into a spurious print abort. Implement it
  only once #1–#3 are confirmed on the bench to have eliminated the false
  positives; it should also add an `exec_logger` `zp_disconnected` event (the
  disconnect is currently invisible in the print JSONL — it had to be inferred
  from the cached z going `None`).
- Latent freeze paths noted but not the cause here (B2/B4/PIAR had valid
  feedrates and the pump kept being commanded post-disconnect, inconsistent with
  a real planner freeze): `move_absolute` emits `F{feedrate:.0f}` (could round a
  sub-1 feedrate to `F0`) and the `PRINT_PATH` seg-skip floor (0.001 mm) is 10×
  looser than `move_relative`'s 1e-4 mm. Worth tightening in a follow-up.

## Investigation artifact

Root-caused via a 6-reader Understand workflow (poller liveness, serial
contention, poller-suspend balance, board-freeze stalls, app-wiring + logs,
threading) → synthesis ranking. The decisive evidence was the cached-z-goes-None
timing in `logs/prints/*.jsonl` correlated to the `PRINT_PATH` start.
