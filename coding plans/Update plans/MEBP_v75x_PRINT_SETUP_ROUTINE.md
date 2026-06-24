# MEBP v7.5.x — Quick Print "print setup routine" (confirmed descent + pump pre-flow)

## Objective

After the operator confirms the needle is over the correct location (the
pre-position + confirm dialog from `MEBP_v75x_QUICK_PRINT_POSITION_CONFIRM_AND_NO_HOME.md`),
a print must run a defined **setup routine** before extruding along the path:

1. Move Z to the safe / travel Z.
2. Move XY to the print-start location.
3. Move Z to the print Z (= plate bottom + print offset) — **and confirm the
   needle physically reached that Z before continuing**.
4. Start pump flow and wait 0.25 s (pre-flow lead-in).
5. Begin the print trajectory.

## What already existed vs. the two gaps

The discrete plan from `build_well_plate_job` already emits, per object:
`TRAVEL_UP` (1) → `MOVE_XY` (2) → `MOVE_Z` (3) → [`EXTRUDE` prime] (4) →
`PRINT_PATH` (5). So the routine's *shape* was already correct. Two gaps:

- **Step 3 had no arrival confirmation** — `MOVE_Z` did `move_z_absolute()` then
  a blind `time.sleep(0.5)`. Printing could begin before the needle actually
  reached the print Z.
- **Step 4 wasn't set for Quick Print** — Quick Print configured no prime, so
  there was no "start pump flow and wait 0.25 s" pre-flow.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PrintManager.py` | **Step 3** — the discrete `MOVE_Z` handler now CONFIRMS Z arrival before the next command: after `move_z_absolute`, it `flush_moves()` (M400) + `wait_for_z_arrival()` with the **position poller suspended** for the wait (no M114 contention), then resumes. Degrades gracefully to the old fixed settle when the controller can't confirm (no ZP / older controller / mock). A non-arrival is logged (`move_z_timeout`) but does not abort. |
| `gui/pages/workflows/quick_print_workflow.py` | **Step 4** — new class constant `_PREFLOW_S = 0.25`; `_build_settings()` sets `prime_amounts_uL[pump] = flow × _PREFLOW_S` (alongside the existing `pump_rates_uL_s[pump] = flow`). `build_well_plate_job` then emits the `EXTRUDE` prime at the flow rate right before `PRINT_PATH`, so the pump runs ~0.25 s in the `MOVE_Z → PRINT_PATH` gap. |
| `tests/test_v75x_print_setup_routine.py` | **New** — 6 tests. |

## Implementation Steps

- [x] Step 3 — `MOVE_Z` confirms Z arrival (poller suspended, graceful fallback).
- [x] Step 4 — Quick Print pre-flow prime = `flow × 0.25 s` at the flow rate.
- [x] New test suite (6) + full `test_v75x*` regression (522 green).
- [x] Plan doc (this file) + `CLAUDE.md` table row.

## How the full flow runs now (Quick Print)

1. Click **Print** → pre-position (needle retracted to safe Z, XY → print start)
   → **confirm dialog** (verify on the microscope).  *(prior update)*
2. On confirm → `PrintManager` runs the job, which is the setup routine:
   `TRAVEL_UP` (Z→safe, no-op — already there) → `MOVE_XY` (→print start, no-op)
   → `MOVE_Z` (descend to print Z, **waits for confirmed arrival**) →
   `EXTRUDE` prime (**pump flows ~0.25 s**) → `PRINT_PATH` (trajectory).
3. At the end: `TRAVEL_UP` retract, **no return to 0,0** (`return_home=False`).
   *(prior update)*

## Testing Notes

- `tests/test_v75x_print_setup_routine.py` (6, green):
  - **Step 3**: `MOVE_Z` calls `move_z_absolute`, suspends the poller,
    `flush_moves`, `wait_for_z_arrival(print_z)`, resumes — and **resumes even if
    the wait raises**; when ZP is disconnected it falls back to the fixed settle
    (no `wait_for_z_arrival` / no poller suspend).
  - **Step 4 / ordering**: the built plan is `TRAVEL_UP → MOVE_XY → MOVE_Z →
    EXTRUDE(prime) → PRINT_PATH`, the prime immediately precedes the path and
    runs at the flow rate; Quick Print's `_build_settings` sets
    `prime_amounts_uL[pump] = flow × 0.25` (scales with flow) at `pump_rates_uL_s[pump] = flow`.
- Regression: full `test_v75x*` discovery = **522 tests OK** (the generalized
  `MOVE_Z` confirmation did not regress the discrete print / execution-logging /
  seam suites).
- **Needs real-HW verification on ME3B V1**: confirm the needle descends to print
  Z and the print waits until Z is actually there, the pump pre-flows ~0.25 s,
  then the path begins — and the first bead is no longer starved at the start.

## Issues & Decisions

- **Step 3 confirmation is generalized to all discrete prints**, not just Quick
  Print — confirming the needle reached print Z before extruding is universally
  correct (the old blind `sleep(0.5)` could start printing mid-descent). It is
  best-effort: a timeout logs `move_z_timeout` and proceeds (does not abort), so
  a sim/odd setup degrades rather than stalls.
- **Poller suspended during the MOVE_Z wait** so the `M400`/`M114` verification
  doesn't race the background poller's `M114` reads (consistent with the
  `PRINT_PATH` poller-suspend fix in `MEBP_v75x_ZP_DISCONNECT_DURING_PRINT.md`).
  try/finally guarantees the poller resumes even if the wait raises.
- **Step 4 uses the existing prime mechanism** (`prime_amounts_uL` +
  `pump_rates_uL_s`) rather than a new plan step — `vol = flow × 0.25` µL at
  `flow` µL/s = the pump runs for 0.25 s, exactly "start pump flow and wait 0.25 s".
  It is **scoped to Quick Print** (Print Setup users configure their own prime).
  For multi-object Quick Prints each object re-primes (a pre-flow per object).
- `_PREFLOW_S` is a tunable class constant.
- Builds on `MEBP_v75x_QUICK_PRINT_POSITION_CONFIRM_AND_NO_HOME.md`
  (pre-position + confirm + no return-to-origin) and
  `MEBP_v75x_ALL_PRINTS_END_AT_SAFE_Z.md`.
