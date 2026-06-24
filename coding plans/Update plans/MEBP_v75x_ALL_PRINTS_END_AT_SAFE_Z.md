# MEBP v7.5.x — Every print always ends at a safe Z (all modes, all outcomes)

## Objective

Guarantee that **every** print leaves the needle retracted to the travel / safe
Z when it ends — on normal completion, on error, and on abort — regardless of
the execution mode (discrete / hybrid plan / trajectory) and regardless of
whether the loaded plan happened to end with a retract step. A print that errors
or is aborted mid-pattern must not leave the needle parked down in the well.

## Root cause / current behavior

The three print execution paths all relied on the *plan contents* (a trailing
`TRAVEL_UP` / `RETURN_HOME` / final retract waypoint) for the end-of-print
retract, and none retracted on the error / abort exits:

- **Discrete** (`PrintManager._execute_loop`): COMPLETED relied on the job ending
  with `TRAVEL_UP`; the `except` (ERROR) path did **not** retract; the `finally`
  only disarmed the plate-bottom floor and closed the exec log. `abort()` did its
  own (unconfirmed) `move_z_absolute` on the GUI thread, but the loop's abort
  early-return did not.
- **Hybrid** (`gui/app.py::_hybrid_thread`): relied on the plan's
  `RETURN_HOME` / `MOVE_SAFE_Z` step; the `finally` only stopped the recorder and
  closed the log — no retract on error/abort.
- **Trajectory** (`gui/app.py::_traj_thread`): relied on the trajectory's final
  waypoint (and `PrintTrajectoryPlanner`'s Z geometry still assumes `ZDIR=+1`, a
  known gap on ME3B V1); the `finally` did not retract.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PrintManager.py` | New `PrintManager._retract_to_safe_z(context, travel_z=None)` — polarity-safe, **raise-only**, best-effort (never raises) final retract via `StageController.ensure_retracted_to` (falls back to `move_z_absolute` on an older controller). Called in the `_execute_loop` `finally` (covers completion, error, and the abort early-returns) before the floor-disarm / log-close. |
| `gui/app.py` | `_hybrid_thread` and `_traj_thread` `finally` blocks now call `pm._retract_to_safe_z(..., travel_z=job.settings.travel_z_height)` before stopping the recorder / closing the log. |
| `tests/test_v75x_print_always_safe_z.py` | **New** — 9 tests. |

## Implementation Steps

- [x] Add `PrintManager._retract_to_safe_z` (raise-only via `ensure_retracted_to`;
  explicit `travel_z` override + job-settings fallback; ZP-disconnected /
  no-travel-z → no-op; older-controller `move_z_absolute` fallback; best-effort).
- [x] Call it in the discrete `_execute_loop` `finally`.
- [x] Call it in the `app.py` hybrid + trajectory thread `finally` blocks (passing
  `job.settings.travel_z_height` explicitly so it does not depend on `pm.job`).
- [x] New test suite; run print-execution + full `test_v75x*` regression.
- [x] Plan doc (this file) + `CLAUDE.md` table row.

## Testing Notes

- `tests/test_v75x_print_always_safe_z.py` (9, green):
  - `_retract_to_safe_z` calls `ensure_retracted_to(travel_z)` (explicit override
    and job fallback), no-ops when ZP disconnected or no travel-Z is available,
    falls back to `move_z_absolute` on a controller without `ensure_retracted_to`,
    and never propagates an exception.
  - The discrete `_execute_loop` `finally` retracts on **completion**, on
    **error** (`_execute_command` raises), and on the **abort early-return**
    (command never runs, but the finally still retracts).
- Regression (all green): `test_v75x_print_execution_logging`,
  `test_v75x_quick_print_position_confirm`, `test_v75x_quick_print_workflow`,
  `test_v75x_multi_object_print_seam`, `test_v75x_z_retract_before_xy_travel`,
  `test_hybrid_execution`; **full `test_v75x*` discover = 470 tests OK**.
- **Needs real-HW verification on ME3B V1**: kill / error / abort a print
  mid-pattern in each mode (discrete Quick Print, hybrid, trajectory) and confirm
  the needle lifts out of the well to the travel Z every time.

## Issues & Decisions

- **Raise-only + idempotent by design.** Because it delegates to
  `ensure_retracted_to` (which only ever raises in the polarity-safe height
  frame and returns immediately when already at/above target), the common case —
  a plan that already ended with a retract — is a confirmed **no-op** with no
  extra motion and no poller-suspend race. Real motion only happens when the
  needle is actually still down (error / abort mid-pattern).
- **State is set before the `finally`.** `COMPLETED` / `ERROR` / `ABORTED` are set
  inside the `try` / `except`, so the GUI state callback is not delayed by the
  retract (which runs afterward on the print thread).
- **`abort()` left as-is.** Its existing immediate `move_z_absolute` gives a
  prompt retract on the GUI thread; the new `finally` retract is the *guarantee*
  and runs on the print thread after the executor stops (`PRINT_PATH` checks the
  abort flag mid-path, so the loop reaches its `finally` quickly). Both are
  raise-only, so the redundancy is harmless.
- **Explicit `travel_z` in `app.py`.** The hybrid path does not set `pm.job`, so
  the app.py calls pass `job.settings.travel_z_height` directly rather than
  relying on `self.job`.
- Complements `MEBP_v75x_Z_RETRACT_BEFORE_XY_TRAVEL.md` (retract *before* every
  cross-position XY move) and `MEBP_v75x_QUICK_PRINT_POSITION_CONFIRM_AND_NO_HOME.md`
  (Quick Print pre-position confirm + no return-to-origin): this closes the
  remaining gap — the retract *after* a print, on any outcome, in any mode.
