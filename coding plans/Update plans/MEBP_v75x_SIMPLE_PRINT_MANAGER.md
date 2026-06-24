# MEBP v7.5.x — SimplePrintManager (minimal fully-confirmed print, debug baseline)

## Objective

A deliberately MINIMAL print executor to use as a debug baseline in the
ZP-disconnect investigation, and as the bottom rung of an "add complexity back
until it breaks" process. Hypothesis: now that the serial protocol is sound
(synchronous Marlin `ok`, atomic `flush_moves`, explicit feedrate on every move
— `MEBP_v75x_ZP_Z_FEEDRATE_INHERITANCE_FIX.md`), the full `PrintManager`'s
elaborate workarounds (open-loop per-segment streaming, planner-buffer barriers,
drift pacing, poller/watchdog suspension) may no longer be necessary.

## What it is

`SupportClasses/SimplePrintManager.py` — consumes the SAME `PrintJob` command
plan from `build_well_plate_job` (trusted geometry/ordering, incl. the v7.5.x
retract-first plan shape) but executes it with a tiny, blocking,
**every-move-CONFIRMED** interpreter:

```
retract→confirm → XY→confirm → descend→confirm → [prime] →
  per segment: extrude (ZP) + XY (Prior), then M400 + XY-settle → retract→confirm
```

Because every segment is confirmed (M400 drains the ZP pump move; an XY-arrival
poll drains the Prior move) the Marlin planner buffer never holds more than one
queued move per board, so it **cannot saturate** — no barriers, pacing, or
buffer bookkeeping needed.

### Keeps ALL safety (non-negotiable per CLAUDE.md)
- Retract to safe/travel Z and CONFIRM before any cross-position XY move
  (`ensure_retracted_to`, polarity-safe, raise-only).
- Descend with an EXPLICIT insert feedrate and CONFIRM arrival (M400 + M114).
- EXPLICIT feedrate on every move (never inherit the pump's slow modal F).
- ABORT the print (→ ERROR + safe-Z retract) if a Z move can't be confirmed.
- ABORT if the ZP board drops mid-print (never dry-run / drag a dead board).
- Plate-bottom Z floor armed for the whole run.
- ALWAYS end at the safe/travel Z (completion / error / abort) via `finally`.

### Deliberately OMITS (add back incrementally as each version proves out on HW)
- Open-loop per-segment streaming + planner-buffer barriers + drift pacing.
- Poller / port-health-watchdog suspension during the path.
- Hybrid / trajectory / velocity / service-sequence executors.
- Resume-from-saved, PrintRecorder, JSONL execution log, print history.

Drop-in interface (`load_job` / `start` / `abort` / `state` / `on_progress` /
`on_state_changed`) so it can replace `PrintManager` wherever those are used.

## Files

| File | Change |
|------|--------|
| `SupportClasses/SimplePrintManager.py` | **New.** The minimal executor (above). Reuses `PrintState`/`CommandType`/`PrintJob` from `PrintManager`. Handles only the command types `build_well_plate_job` emits (COMMENT/TRAVEL_UP/MOVE_XY/MOVE_Z/EXTRUDE/PRINT_PATH/HOME_XY); logs+skips anything else. |
| `gui/pages/workflows/stress_test_workflow.py` | New **"Simple PrintManager (debug)"** checkbox (`_chk_simple_pm`) + `_StressConfig.use_simple_pm`; `_print_burst` instantiates `SimplePrintManager` vs `PrintManager` per the flag and logs which one ran (`[SimplePrintManager]` / `[PrintManager]`). Lets the operator A/B test the two executors on the same hardware. |
| `tests/test_v75x_simple_print_manager.py` | **New (10):** completes + keeps safety (retract/XY/descend/flush/pump, floor arm→disarm); every Z move has an explicit feedrate (descent = insert); retract precedes first XY; descent confirmed before extrusion; aborts on unconfirmed arrival / M400 timeout / mid-print ZP drop; dry-run skips pump; per-segment confirm count; XY-only (no ZP) completes. |

## Add-back ladder (the debug process)

If `SimplePrintManager` prints reliably on ME3B V1 but the full `PrintManager`
does not, re-introduce complexity ONE layer at a time and re-test, to find the
exact layer that reintroduces the fault:

1. **V1 (this):** every segment confirmed (M400 + XY settle). No streaming.
2. **V2:** drop the per-segment M400; add a barrier every N segments.
3. **V3:** open-loop pacing (`sleep` by the rate-limiting axis) instead of XY
   arrival polling.
4. **V4:** suspend poller + watchdog during the path.
5. **V5 == full PrintManager.**

## Testing Notes

- `python -m unittest tests.test_v75x_simple_print_manager` → 10 OK.
- Stress + simple-PM + feedrate suites → 43 OK.
- **Real-HW:** in the Stress Test, tick **Descend to print Z** + **Simple
  PrintManager (debug)** + a small flow, run it, and compare against the same
  config with the box unticked. Watch `logs/zp_serial.log` (every `G0 Z` should
  carry an explicit `F`; no multi-second `busy` M400) and whether the board
  stays connected. Report which executor(s) hold up.

## Issues & Decisions

- Consumes the existing command plan (not a new geometry path) so the only
  variable under test is *how* commands are executed, not *what* they are.
- Per-segment confirmation makes extrusion start/stop (not a continuous bead) —
  acceptable for a debug baseline; continuous extrusion is a later add-back
  rung, not a safety regression.
