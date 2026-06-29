# MEBP v7.5.x — Prep buffer step: drain the long pump move before the next travel

## Objective

Fix the recurring "buffer prep step times out" error during printing-prep: the
needle travels to the buffer well, aspirates buffer, then the **next** move
(travel to pick up the ink) aborts with:

```
12:47:00 move_pump_uL(P2, -27.222 µL, 1.000 µL/s)        ← 4-needle buffer aspirate
12:47:14 [PREP] Prep complete
12:47:30 ZPStage: flush_moves: M400 timed out after 15.0s (rx_lines=8, busy=8)
12:47:30 StageController: safe_travel_to: Z retract M400 timed out — ABORTING, will not start XY move
```

The ink pickup step works; the buffer step (same code) does not. Operator
ask: "make the buffer step behave just like the ink pickup step."

## Root cause

The buffer prep aspirate and the ink pickup run the **identical** code path
(`PickAndPlaceManager._settled_pump_move` → `StageController.move_pump_uL(settle=True)`).
The only difference is **volume**, and that difference tripped a latent bug:

- Buffer = `buffer_needles` (4) × needle internal volume ≈ **27.2 µL** at the
  prep rate **1.0 µL/s** ⇒ a **~27 s** physical pump move.
- A pump `G0` returns on Marlin `ok` = *admitted to the planner buffer*, **not
  motion-complete**. The settle-aware completion wait in `move_pump_uL` was
  `time.sleep(min(move_s, _PUMP_MOVE_WAIT_CAP_S))` — **capped at 10 s**. So the
  call returned after 10 s while Marlin kept running the pump for ~17 s more.
- `run_prep` logged "Prep complete"; the next step (`aspirate_ink` →
  `_safe_move_to` → `safe_travel_to`) commanded a Z retract + `M400`. Marlin
  executes moves **sequentially**, so the Z retract queued *behind* the
  still-running pump move; `M400` waited for both and **timed out at 15 s** →
  the run aborted.
- **Ink pickup worked** only because its computed volume (~6.5 µL ⇒ ~6.5 s) fit
  *under* the 10 s cap, so the pump was genuinely drained before the next
  `M400`. Same code, different volume — the 10 s truncation was the bug.

The log corroborates exactly: `rx_lines=8, busy=8` means the board was **alive
and busy** (sending busy keep-alives), just still draining the long pump move
past the 15 s `M400` window.

## Fix

A discrete (settle) pump move must wait for the pump to **physically finish**
before the caller advances — confirmed via the board, not a capped guess.

`SupportClasses/StageController.py`:

- New `StageController._wait_pump_move_complete(move_s)` — drains the in-flight
  pump move via `M400` (`ZPStage.flush_moves`, the same mechanism
  `safe_travel_to`/`ensure_retracted_to` use for Z), with a timeout **scaled to
  the estimated move duration** (`move_s + margin`, capped at an absolute
  backstop). The position poller is suspended for the wait (matches the
  existing M400-confirm sites). Returns `True` (drained / sim / no real board),
  `False` (flush available but timed out — already waited ~`move_s`, so don't
  sleep again), or `None` (no `flush_moves` available — caller falls back to an
  open-loop sleep).
- `move_pump_uL(settle=True)` now calls `_wait_pump_move_complete(move_s)`
  instead of `time.sleep(min(move_s, 10 s))`. On `None` (older controller /
  fake without `flush_moves`) it falls back to an open-loop sleep of the **full
  estimated duration** (no 10 s truncation — that truncation is what bled the
  buffer move into the next M400). The pre/post settle dwell is unchanged.
- Constants: removed `_PUMP_MOVE_WAIT_CAP_S` (the 10 s truncation cap);
  added `_PUMP_MOVE_DRAIN_TIMEOUT_CAP_S = 180.0` (absolute backstop so a
  pathological near-zero rate can't hang forever) and
  `_PUMP_MOVE_DRAIN_MARGIN_S = 5.0` (accel/decel + busy keep-alive margin).

Scope: every discrete `move_pump_uL(settle=True)` caller benefits (all prep /
post-clean / cell-removal / cell-labeling / trypsin / dye sites + the discrete
print `DISPENSE`/prime). The **streamed print path and manual jog** use
`settle=False` and are a pure passthrough — untouched. With a real board the
normal case returns immediately once `M400` confirms (no redundant sleep);
sim/`flush_moves`-returns-True cases don't sleep either, so tests stay fast.

## Files Modified

- `SupportClasses/StageController.py` — constants + `_wait_pump_move_complete` +
  `move_pump_uL` settle block.
- `tests/test_v75x_pump_settle_and_prime_time.py` — updated for the renamed
  constant + new semantics; added `TestMovePumpSettleM400Confirm` (board-
  confirmed drain path) and a fallback "not truncated at 10 s" test.

## Implementation Steps

- [x] Diagnose from the log: long buffer pump move + 10 s wait cap + sequential
  Marlin execution + 15 s M400 timeout.
- [x] Add `_wait_pump_move_complete` (M400 drain, scaled timeout, poller
  suspended, exception-safe).
- [x] Rewire `move_pump_uL(settle=True)` to confirm completion; fallback sleep
  uses the full estimate (backstop-capped).
- [x] Update + extend tests.
- [ ] **Real-HW verification on ME3B V1** — run printing prep: buffer aspirate
  completes, then travel-to-ink does NOT M400-timeout, ink is picked up, print
  starts. Confirm `logs/zp_serial.log` shows the M400 after the buffer move
  returning `ok` (not `timeout`).

## Testing Notes

- `tests/test_v75x_pump_settle_and_prime_time.py` — 23 tests green
  (settle bracketing, M400-confirm drain with scaled timeout, no-double-wait on
  flush timeout, full-duration fallback when no board confirm, backstop cap,
  `settle=False` passthrough).
- Affected suites green: `test_v75x_spheroid_pick_place_z`,
  `test_v75x_cell_targeting_removal`, `test_v75x_cell_labeling`,
  `test_v75x_quick_print_pick_and_place`, `test_v75x_simple_print_manager`
  (112), plus `test_v75x_zp_feedrate_inheritance_fix`,
  `test_v75x_zp_serial_flow_control`, `test_v75x_stress_test_workflow`.
- Pre-existing (unrelated) failures in `test_v75x_print_setup_routine`
  (Quick Print prime-amount math) confirmed present with this fix stashed —
  not introduced here.

## Issues & Decisions

- **Why M400 confirm rather than just raising the sleep cap?** The board
  confirmation is correct (drains the planner) rather than a tuned magic
  number, and it guarantees the subsequent `safe_travel_to` M400 returns
  promptly. The open-loop sleep is kept only as a fallback for controllers /
  fakes that don't expose `flush_moves`.
- **No double-wait on flush timeout:** if `flush_moves` is available but times
  out, it already blocked ~`move_s`; the caller does not sleep again.
- **Poller suspended, watchdog not:** mirrors the existing `safe_travel_to` /
  `ensure_retracted_to` M400-confirm pattern (the poller's M114 reads are the
  contention source; the port-health watchdog coexists as it does today).
