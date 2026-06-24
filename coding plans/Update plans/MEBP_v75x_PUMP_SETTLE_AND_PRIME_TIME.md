# MEBP v7.5.x — Pump Settle Time + Prime Time (Hardware Setup → Pump)

## Objective

Operator request:

> For all pump moves, we need to add a delay before and after to allow the
> pump to settle. This delay time should prevent us from moving to the next
> step. We should also have a pump priming time which ports into the printing
> workflows. The pump settling time for any of these moves should be a part of
> the hardware calibration page.

Two new **global** pump timing parameters, configured on **Hardware Setup →
Pump**, persisted in `HardwareConfig`:

1. **Pump settle time (s)** — a blocking dwell applied **before AND after**
   every *discrete* pump actuation. The dwell prevents the workflow from moving
   to the next step until the pump has finished moving and the fluid/pressure
   has settled.
2. **Pump prime time (s)** — the pre-flow prime duration. Printing workflows
   compute the prime volume = `flow × prime_time` and dispense it just before
   the print path. Replaces the hardcoded `_PREFLOW_S = 0.25` constant in Quick
   Print (default kept at 0.25 s).

### Operator-confirmed scope decisions

- **Location:** Hardware Setup → Pump tab (these are fixed per-machine hardware
  traits; they live with the other pump settings — NOT the per-session
  Calibration page).
- **Settle scope:** *discrete actuations only* — prime, aspirate, dispense,
  pick/place push-pull, needle-prep oil/wash/buffer moves, and `EXTRUDE`
  commands. **Excludes** the streamed per-segment print-path extrusion
  (`_execute_print_path` — settling each segment would dwell → over-extrusion
  blobs, the very thing the print-path barrier work removed) and **manual jog**
  (`move_pump_relative` via jog handlers — user-controlled, must stay
  continuous).
- **Granularity:** one global value for all pumps (not per-P1/P2/P3).
- **Before/after:** a single settle value, applied both before and after.

## Design

The settle is centralized at the single µL chokepoint
`StageController.move_pump_uL`, behind a new keyword-only `settle` flag
(default `False`). Discrete callers opt in with `settle=True`; the streamed
print path and manual jog never pass it, so they are unaffected.

When `settle=True`, `move_pump_uL`:
1. Sleeps the configured settle time (pre-move settle).
2. Issues the move (async — the pump `G0` returns on Marlin `ok`, i.e.
   admitted-to-planner-buffer, **not** motion-complete).
3. **Blocks** for the (open-loop) move duration so the post-settle is a true
   settle and the caller's next step does not begin until the pump has
   finished. The completion estimate is `abs(vol)/rate + 0.1`, capped at 10 s —
   byte-identical to the legacy `EXTRUDE` completion wait it now subsumes.
4. Sleeps the configured settle time (post-move settle).

`settle=0` is valid (no dwell) — the completion wait still runs when
`settle=True`, preserving the old `EXTRUDE` blocking semantics.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/HardwareConfig.py` | New `HardwareConfig.pump_settle_time_s` (0.0) + `pump_prime_time_s` (0.25) fields; serialized in `to_dict`/`from_dict`. |
| `SupportClasses/StageController.py` | New `pump_settle_time_s()` reader; `move_pump_uL(..., *, settle=False)` adds pre/post settle dwell + blocking completion wait; module caps `_PUMP_MOVE_WAIT_CAP_S` / `_PUMP_SETTLE_FALLBACK_RATE_UL_S`. |
| `SupportClasses/PrintManager.py` | `EXTRUDE` handler calls `move_pump_uL(..., settle=True)` (subsumes its manual completion wait); robust `TypeError` fallback for older controllers / fakes. |
| `SupportClasses/PickAndPlaceManager.py` | New module helper `_settled_pump_move(ctrl, ...)`; all discrete pump moves (spheroid, cell-removal, prep, `aspirate_ink`) route through it (settle=True, `TypeError` fallback). |
| `gui/pages/workflows/quick_print_workflow.py` | `_resolved_print_kinematics` uses configured prime time via new `_prime_time_s()` (falls back to `_PREFLOW_S`). |
| `gui/pages/hardware_setup.py` | "Pump Timing (global)" group on the Pump sub-page (settle + prime spinboxes); wired into `_rebuild_config` + `_apply_config_to_ui`. |
| `tests/test_v75x_pump_settle_and_prime_time.py` | NEW — config round-trip, reader, settle on/off dwell + completion wait, prime-time wiring. |

## Implementation Steps

- [x] `HardwareConfig`: add fields + serialize round-trip.
- [x] `StageController`: `pump_settle_time_s()` + `move_pump_uL(settle=)`.
- [x] `PrintManager` `EXTRUDE` handler → `settle=True` (+ hybrid/service `move_pump` bracketed).
- [x] `PickAndPlaceManager` → `_settled_pump_move` at all 24 discrete sites.
- [x] Quick Print prime time (`_prime_default_s` seeds the per-run pre-flow knob; respects the operator's concurrent per-page knob).
- [x] Hardware Setup Pump-tab UI (build + rebuild + restore).
- [x] Tests + run affected suites — `tests/test_v75x_pump_settle_and_prime_time.py` (17) + Hardware Setup offscreen smoke; pick-place/cell-removal/quick-print/print-setup/print-manager/hybrid/stress/ZP suites green (199).

## Testing Notes

- `tests/test_v75x_pump_settle_and_prime_time.py` — monkeypatches `time.sleep`
  to record dwell durations against a real `StageController` + fake ZP stage.
- Regression: pump/print/spheroid/cell-removal/quick-print suites must stay
  green (the fakes lack the `settle` kwarg → the `TypeError` fallback keeps the
  recorded call sequences identical).

## Issues & Decisions

- Streamed print path (`_execute_print_path`) deliberately untouched — per-
  segment settle would re-introduce the over-extrusion dwell removed by the
  print-path barrier work.
- Manual jog untouched — settle would break continuous jogging.
- `TypeError` fallback everywhere a discrete caller passes `settle=True` so
  existing fakes / older controllers degrade to the prior behavior.

**Needs real-HW verification on ME3B V1.**
