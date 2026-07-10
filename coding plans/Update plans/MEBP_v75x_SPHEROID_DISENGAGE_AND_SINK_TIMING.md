# MEBP v7.5.x — Spheroid Pick & Place: Disengagement Flow + Sink-Timing Calibration

## Objective

Give the Spheroid Pick & Place workflow two operator-requested, **optional**
capabilities plus a **guided calibration** for them:

1. **Disengagement extra-aspirate** — an extra suction volume (at a configurable,
   usually faster flow) applied right after the normal carrier aspirate, to pop a
   spheroid that's stuck to the glass off the plate.
2. **Sink-timing model** — once aspirated, the spheroid rises into the bore then
   **sinks** back toward the tip. Calibrate a **timing function** `t_sink(lift)`
   (via a guided cumulative staircase over one test spheroid), then at run time
   choose the aspirate volume **per move** so the spheroid finishes sinking right
   as the needle arrives at the destination:
   - It can't sink out in the well (it stays lifted for the whole trip).
   - It's never lifted more than the travel needs (no getting-stuck-too-high).
   - It arrives at the tip → a small **release dispense** deposits minimal excess.

All new behavior is **off by default** — with every new field at its default the
executed sequence is byte-identical to the pre-change spheroid path.

### Decisions (operator-confirmed)
- Guided calibration helper (not settings only), **cumulative staircase**.
- **Rate/timing** model: calibrate a curve; runtime **inverts** it so
  `sink_time ≈ travel_time`; **floor at the sphere-capture volume**.
- **One curve for all** spheroids (assumed roughly uniform).
- **Travel time estimated per move** (XY distance ÷ XY max speed + Z retract/lower).
- Disengagement = extra held aspirate applied as the **fast leading portion** of
  the total aspirate.
- Placement = **small release dispense** (needle keeps the rest).

## Physics & timing model

- `height_mm = V_uL / bore_area_mm2` (1 µL = 1 mm³);
  `bore_area_mm2 = NeedleSpec.cross_section_area_mm2`.
- Calibration samples `(lift_i, t_i)`, `lift_i = ΔV_i / bore_area` → curve `t_sink(lift)`.
- Runtime per move: `target_sink = T_travel + travel_margin_s`;
  `timing_vol = curve.lift_for_time(target_sink) × bore_area`;
  `total_vol = max(carrier_vol, timing_vol)` (floor at capture);
  arrival wait `= max(0, curve.time_for_lift(total_vol/area) − T_travel)`.

## Files Modified / Added

| File | Rationale |
|------|-----------|
| `SupportClasses/SpheroidSinkCalibrationStore.py` | **NEW** — atomic-JSON singleton holding the one sink-timing curve + monotone invert helpers (`lift_for_time`/`time_for_lift`). |
| `SupportClasses/PickAndPlaceManager.py` | `SpheroidPickupConfig` new fields + `to_dict`; `PickPlaceExecutor` gains `bore_area_mm2`/`sink_curve` + `_estimate_travel_time_s` + `_planned_aspirate_uL`; `_execute_spheroid_pickup` rewritten (disengage aspirate, sink wait, release dispense). |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | New "Sink timing & disengage" settings section; `_current_config`/`_on_start` wiring + gates; `_bore_area_mm2()`; `_open_sink_calibration()`; calibration-status label; `hideEvent` hook. |
| `gui/pages/workflows/spheroid_sink_calibration.py` | **NEW** — `SinkDisengageCalibrationDialog` (staircase sink measure + disengage test, live camera, worker/bridge, safety). |
| `tests/test_v75x_spheroid_disengage_and_sink_timing.py` | **NEW** — store, runtime math, sequence, dialog, regression. |
| `CLAUDE.md` | Add this plan to the Existing Update Plans table. |

## Implementation Steps

- [ ] 1. `SpheroidSinkCalibrationStore` (curve store + invert/clamp helpers + singleton).
- [ ] 2. `SpheroidPickupConfig` fields (`disengage_enabled/volume_uL/rate_uL_s`,
      `sink_timing_enabled`, `travel_margin_s`, `release_enabled/volume_uL`) + `to_dict`.
- [ ] 3. `PickPlaceExecutor`: `bore_area_mm2`, `sink_curve`, `_estimate_travel_time_s`,
      `_planned_aspirate_uL`; rewrite `_execute_spheroid_pickup`.
- [ ] 4. GUI settings section + `_current_config` + `_on_start` (bore/curve wiring + gates)
      + `_bore_area_mm2()` + `_open_sink_calibration()` + status label + `hideEvent`.
- [ ] 5. `SinkDisengageCalibrationDialog` (staircase + disengage test).
- [ ] 6. Tests + regression.
- [ ] 7. CLAUDE.md table + memory pointer.

## Safety invariants preserved
- Every cross-position move routes through `safe_travel_to`/`ensure_retracted_to`
  (never a raw Z comparison — ZDIR=-1 on ME3B V1). Aspirate = `−`, dispense = `+`.
  Spheroid pump moves keep `relieve=False` (volume-balanced micro-captures).
  All actuation runs on a worker thread; the routine always ends at safe Z.

## Testing Notes
- Unit: store round-trip + monotone/clamped inversion; runtime volume math incl.
  capture floor + remaining wait; travel-time estimate + divide-by-zero fallback;
  sequence with a fake controller (disengage leading portion; release vs balanced
  dispense; sink dwell only when `wait_s>0`); offscreen dialog (staircase records
  `(lift, elapsed)` w/ fake clock, cumulative cap, Save curve → store; preflight).
- Regression: spheroid pick&place / prep / pump-settle suites green with defaults off.
- Real HW (ME3B V1): pending — calibrate the curve, run near+far pairs, confirm the
  spheroid survives the in-well retract, arrives at the tip, deposits minimal excess,
  ends retracted; a stuck spheroid disengages with the tuned extra aspirate.

## Issues & Decisions
- **New store (not a settings field):** the sink curve is a list of points, which
  can't live in a scalar settings widget — hence a dedicated `SpheroidSinkCalibrationStore`
  (mirrors `PrintTimingCalibrationStore`). The disengage volume/rate DO live in the
  workflow settings (scalars) and persist via the profile system.
- **Curve is needle-independent:** `t(lift)` is a property of the spheroid+fluid;
  volume↔lift always uses the *current* bore area, so a needle change only warns.
- **Volume accumulation (release mode):** small release keeps most of the aspirated
  fluid → the needle accumulates per op. Intended trade for minimal deposit; the
  existing Post-clean clears it (documented in the setting's help). Full-dispense
  mode (release off) stays net-zero.
