# MEBP v7.5.x — Quick Print pre-position off the GUI thread + fast-fail Z-arrival on a dead ZP board

## Objective

Fix two bench-reported Quick Print problems (single run, both symptoms):

1. **"The live needle view wasn't live until halfway through the print."** The
   pre-position move (`safe_travel_to`) ran on the **GUI thread**, blocking Qt's
   event loop so the live microscope `CameraFeedView` couldn't repaint. Normally
   brief, but in the reported run the ZP board had stopped responding so the
   block lasted the **full 15 s `wait_for_z_arrival` timeout** — the feed was
   dead until the move finally returned.

2. **"The ZP stage disconnected at the end and didn't move up."** A **genuine**
   board drop (not a software false-positive). `wait_for_z_arrival` polls
   `ZPStage.get_current_position()`, which returns the last **stale** floats when
   the board stops answering M114 — and it never checked `_last_position_read_ok`.
   So the lost retract G-code + 15 s of frozen `z=68.23` reads happened **before**
   the poller-liveness watchdog noticed and declared the disconnect. The needle
   stayed parked down because the board was already dead when the retract was
   commanded.

## Root cause (from `logs/prints/print_20260617_171814_…B2.jsonl` + the app log)

- The 17:18 B2 print **completed normally** (Z descended 60.07→38.62 raw, printed,
  retracted to 60.07 = travel; pumps moved). So the mechanism is intermittent.
- The failing attempt (~17:19:56–17:20:15) was a **later** pre-position: app log
  shows `wait_for_z_arrival timeout (15.0s): target=39.80, actual=(1.0, 0.32,
  68.23, 0.0)` → `safe_travel_to … ABORTING` → 4 s later `ZP position queries
  failing — treating as disconnected (poller-driven liveness)`.
- During the 15 s wait the board returned a constant `68.23` — i.e. the **stale
  last-good read**; `get_current_position()` returns stale floats on a dead board.
  `wait_for_z_arrival` had no way to tell "slow" from "dead", so it waited the
  whole timeout (and froze the GUI-thread caller, killing the camera feed).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | `wait_for_z_arrival` now watches `zp_stage._last_position_read_ok` and **fast-fails** after a short **debounced** streak (5 consecutive failures ≈ 0.75 s) of unanswered M114 reads, instead of waiting out the full timeout on stale data. A single transient miss (busy board under a write burst) is ridden out. |
| `gui/pages/workflows/quick_print_workflow.py` | Pre-position move runs on a **worker thread** so the live microscope feed keeps painting during positioning. `_PrintBridge` gains a `prepositioned` signal; `_on_print` validates + builds settings + stashes context, then launches `safe_travel_to` on a daemon thread; the queued-signal continuation `_on_prepositioned` shows the confirm dialog and builds/starts the job on the GUI thread. It also **re-checks `is_zp_connected`** before printing (a drop now surfaces in ~1 s) so a print can't dry-run against a dead board with the needle down. `_update_button_state` treats an in-flight pre-position as busy so the 300 ms status tick can't re-enable Print mid-move. |
| `tests/test_v75x_quick_print_position_confirm.py` | Updated `TestOnPrintFlow` to drive the async flow (join the worker, pump the event loop) — the confirm-then-run and cancel paths still assert the same job shape (no `HOME_XY`, ends on `TRAVEL_UP`). |
| `tests/test_v75x_quick_print_preposition_async.py` | **New** (8): off-GUI-thread execution, Print disabled while positioning, abort-print-if-ZP-dropped, happy path; plus `wait_for_z_arrival` fast-fail on a dead board, confirm-on-alive, and transient-miss-ridden-out. |

## Implementation Steps

- [x] `wait_for_z_arrival`: track `_last_position_read_ok`; bail after 5 consecutive read failures (debounced); reset streak on a good read.
- [x] `_PrintBridge.prepositioned` signal + GUI-thread connection.
- [x] `_on_print` → launch `safe_travel_to` on a daemon worker thread; stash context in `self._pending_print`.
- [x] `_on_prepositioned` continuation: confirm dialog → re-check ZP connectivity → build + start job.
- [x] `_update_button_state` keeps Print disabled while positioning is in flight.
- [x] Update existing position-confirm tests for the async flow; add new test file.
- [x] Regression: Z-retract / ZP-disconnect / always-safe-Z / print-setup / quick-print / trajectory / multi-object suites green (95 + 16 = 111 across the touched areas).

## Testing Notes

- `python -m unittest tests.test_v75x_quick_print_preposition_async tests.test_v75x_quick_print_position_confirm` → 16 OK.
- `python -m unittest tests.test_v75x_z_retract_before_xy_travel tests.test_v75x_zp_disconnect_during_print tests.test_v75x_zp_disconnect_watchdog_and_abort tests.test_v75x_print_always_safe_z tests.test_v75x_print_setup_routine` → 55 OK.
- **Needs real-HW verification on ME3B V1:** confirm (a) the microscope feed stays live throughout pre-positioning, and (b) a ZP drop during positioning now surfaces within ~1 s with the "ZP disconnected during positioning" status (instead of a 15 s frozen feed) and does **not** start a dry-run print.

## Issues & Decisions

- **This does not stop the board from dropping.** The disconnect itself is a
  genuine ZP comms failure (the retract command was lost because the board had
  already gone silent). The durable cure remains the **hardware auto-reset
  disable** (cut `RST-EN` / 10 µF RESET→GND) flagged in
  `MEBP_v75x_ZP_DTR_NO_RESET.md`. These changes make the failure **surface fast
  and visibly** (no 15 s frozen camera; no dry-run with the needle parked down)
  rather than hiding it.
- Worker thread is a plain daemon `threading.Thread` (not a QThread) calling the
  already-thread-safe controller move primitives (serial-lock guarded; the
  Mosaic scan worker uses the same approach). The continuation runs on the GUI
  thread via a queued Qt signal, so `QMessageBox` is shown safely.
- Debounce (5 consecutive failures) chosen to match the poller-liveness spirit:
  a single missed M114 under a busy write stream is transient and must not abort
  a legitimate retract; a truly dead board fails every read and is caught in
  ~0.75 s.
