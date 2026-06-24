# MEBP v7.5.x — ZP Stress Test workflow (flow-control bench harness)

## Objective

Give the operator a one-button, repeatable harness to **bench-validate the
synchronous-`ok` ZP flow-control fix** (`MEBP_v75x_ZP_SERIAL_FLOW_CONTROL.md`) on
the real ME3B V1 board: loop the exact command patterns that historically made
the ZP board "drop" (retract→XY travel, dense jog bursts, real prints), watch for
**any** "ZP disconnected" or mid-session board reset, and report hard flow-control
telemetry with a PASS/FAIL verdict.

## What it does

New Workflows tile **"ZP Stress Test"** (`stress_test`). Three selectable stress
modes, looped over N cycles, all on a worker thread so the GUI + live camera stay
responsive:

- **Travel stress** — repeated `StageController.safe_travel_to` hops around the
  plate (`target_z_mm=None` → retract Z → fast XY → wait; the needle is **never
  lowered**). This is the exact path that failed in the reported log, and drives
  a dense Z-move + M400 + XY + M114 stream.
- **Jog stress** — long bursts of small Z (and optional net-zero pump) relative
  jogs — the dense write stream that used to overflow Marlin's serial buffer.
  Starts each burst with `ensure_retracted_to(safe_z)` and oscillates a small
  amplitude so the needle never approaches the plate (soft limits also clamp).
- **XY stress (concurrent)** — during the jog phase (needle retracted = safe to
  move the plate), an XY (Prior) oscillator runs on its **own thread**, driving
  the XY serial channel at the same time as the ZP channel. The two links are
  independent (separate serial locks), so this is the cross-communication
  conflict case — contention on the shared position poller, threading, or host
  buffers. Net-zero, envelope-clamped relative moves. Excluded from the print
  phase (where `PrintManager` owns XY) and the travel phase (which drives XY
  itself). Works standalone too (XY-only, no ZP jog).
- **Print stress** (opt-in, default off) — runs a small synthesized circle print
  at the plate-centre/first-calibrated well through the real discrete
  `PrintManager` path; default flow 0 = dry run (no material — the *command
  stream* is what's validated). `return_home=False`.

**Monitoring & verdict:** reads `ZPStageManager.get_comm_counters()` (new) —
commands issued vs cleanly acked, `ok_fail`, board `reset` count — plus a
per-loop transition check on **both** `is_zp_connected` AND `is_xy_connected`
(a cross-comms conflict can drop either channel) and the `_board_reset_detected`
flag. Live counter grid + scrolling event log + a PASS/FAIL banner. **PASS iff
zero comms drops (XY or ZP), zero resets, zero un-acked commands.**

## Safety

- Refuses to start unless XY+ZP are connected **and a Safe Z is calibrated**
  (without it a retract target of 0 = plate-bottom datum on ME3B V1 = a crash).
- All XY travel via `safe_travel_to` (retract-first, never lowers); Z jogs are
  small oscillations around the retracted Safe Z; the `finally` block always
  `ensure_retracted_to(safe_z)` on completion / Stop / error.
- Stop is cooperative (`threading.Event`) — finishes the in-flight move (bounded
  by the new `ok` timeouts), retracts, and reports.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/workflows/stress_test_workflow.py` | **New** — `StressTestWorkflowPage` (header / config grid / live monitor + event log / run row), `_StressConfig`, `_StressBridge` (worker→GUI signals). Worker `_run` + primitives `_travel_burst` / `_jog_burst` / `_print_burst`, link/reset watcher, PASS/FAIL verdict. Reuses `StandardJogContextPanel` for the left context panel. |
| `gui/pages/workflows/workflow_picker.py` | New enabled `WorkflowTile("stress_test", "🔁", "ZP Stress Test", …)`. |
| `gui/pages/workflows_mode.py` | Import + `elif tile.workflow_id == "stress_test"` page wiring (controller/settings/camera_manager). |
| `SupportClasses/ZPStage.py` | New comm telemetry: `cmd_count`/`ok_count`/`ok_fail_count`/`reset_count` (incremented in `send_data` / `_read_until_ok`, getattr-safe for `__new__` test stand-ins) + `get_comm_counters()` / `reset_comm_counters()`. |
| `tests/test_v75x_stress_test_workflow.py` | **New** (14): registration + build, safety gate (no Safe Z / disconnected / no mode), primitives (travel never lowers + clamps to envelope; jog retracts-first + net-zero oscillation; stop halts), PASS/FAIL verdict (clean / disconnect / unacked), and the ZP comm counters. |

## Testing Notes

- `python -m unittest tests.test_v75x_stress_test_workflow` → 14 OK.
- `python -m unittest tests.test_v75x_zp_serial_flow_control tests.test_v75x_zp_disconnect_during_print tests.test_v75x_zp_jog_clamp_freeze tests.test_v75x_zp_position_override tests.test_v75x_quick_print_workflow tests.test_v75x_stress_test_workflow` → 75 OK.
- `WorkflowsModePage` smoke-builds with `stress_test` registered.
- **Real-HW use (the point):** Workflows → ZP Stress Test. With XY+ZP connected
  and Safe Z set, enable Travel + Jog (Print optional, flow 0 first), set a high
  cycle count, Start, and leave it running. Expect **PASS** with 0 unacked / 0
  resets / 0 disconnects. A FAIL with `ok_fail>0` but no reset = the board went
  silent (escalate flow-control / Phase 2); a `reset>0` = brownout/auto-reset →
  hardware fix (`MEBP_v75x_ZP_DTR_NO_RESET.md`).

## Addendum (2026-06-18) — safety stop on a degraded link

Bench finding: a *live-but-failing* board (M400/`ok` lost — see
`MEBP_v75x_ZP_M400_ATOMIC_FLUSH.md`) does NOT trip the connection watchdog, so
`is_zp_connected` stayed True and the loop kept oscillating the Z motor → **only
the Z stepper got very hot** (energized + continuously driven = the proof the
board was alive the whole time, i.e. a software comms fault, not hardware). A
continuously-driven stepper overheating is a damage/fire hazard. Fix:
`check_link()` now returns False (halts the run, FAIL) once `ok_fail >=
_ABORT_ON_OK_FAILS` (3) or any disconnect — and the cycle breaks promptly after
each burst, not just at the next cycle top — so the harness stops driving the
ZP on a failing link. Test: `test_safety_stop_halts_driving_on_sustained_ack_failures`.

## Addendum (2026-06-18 #2) — print stress no longer descends the needle

Bench finding: travel + jog cycles run clean, but **print-only** runs fail — they
move to position, start, then drop. The print phase is the only one that does a
large **Z descent** (MOVE_Z to print height; travel/jog only retract up or
oscillate near safe Z), and the failing run's descent `move_z` took 28 s (vs
6.5 s in the run before) then the board dropped 2 segments into the path. The
descent is the trigger. Worse, `_print_burst` was resolving `print_z =
print_height_to_zref(0.5)` — driving the needle ~20 mm down to 0.5 mm above the
plate bottom on **every looped print** (needle-crash risk + the load that the
real Z descent puts on the motor/board). Fix: the stress print now keeps
`print_z = travel/safe Z` — MOVE_Z becomes a safe no-op move so the discrete
command stream (and comms stress) is byte-identical, but the needle never
plunges at the plate. This is both a safety fix and a clean isolation: if
print-only now passes, the **physical Z descent** (not the comms) is the trigger,
which is the next thing to chase on the *real* print path.

## Addendum (2026-06-18 #3) — print stress descends again (realistic print)

The robust serial trace (`logs/zp_serial.log`) + the atomic-`flush_moves` /
adaptive-timeout fixes were run across many stress combinations and produced
**no failure** (e.g. `txns=3500 ok=3500 fail=0 reset=0`) — the comms fault
appears resolved. With the link healthy, the no-descent print (#2 addendum) was
keeping the print at safe Z, so it no longer moved Z into position. Reverted to a
**realistic descent**: the print stress now drives Z to the real print height via
`print_height_to_zref(print_height_mm)` (default 0.5 mm above plate bottom),
combining all components (TRAVEL_UP → MOVE_XY → descend MOVE_Z → optional
EXTRUDE → PRINT_PATH → retract) like a true print. New controls: a "Descend to
print Z" checkbox (default ON) + a "Print height" spin; unchecking keeps the
comms-only no-descent behavior. The descent is clamped by the soft limits + the
plate floor (cannot punch through the plate bottom), same as a real print. Tests:
`TestPrintDescent` (2). Safe because the board is now confirmed healthy and the
descent is plate-floor clamped; the standalone "Z descent + confirm" component
remains for isolation.

## Issues & Decisions

- **Print stress is opt-in and dry by default** (flow 0): the command stream is
  identical with or without dispensing, so validation needs no material.
- **Print Z defaults to Safe Z** (no net descent) unless a plate-bottom datum
  lets `print_height_to_zref` resolve a real safe height — keeps the harness from
  crashing the needle when the plate bottom isn't calibrated.
- **Worker thread + bridge** mirrors the established workflow pattern; the verdict
  is computed in the worker `finally` so it always reports even on Stop/error.
- The comm counters double as a general diagnostic (not stress-test-only).
