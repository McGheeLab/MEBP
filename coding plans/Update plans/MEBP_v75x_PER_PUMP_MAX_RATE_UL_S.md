# MEBP v7.5.x — Per-pump pump max rate with a µL/s secondary readout

## Objective

On the Hardware Setup page the pump **max rate** is edited in **mm/min** and is a
single global value (`safety_limits.max_pump_feedrate`). Two operator-requested
changes:

1. **Show µL/s as a secondary unit** beside the mm/min value (the mm/min stays the
   editable primary — the plunger speed the firmware understands; µL/s is a live
   derived readout).
2. **Make the max rate per-pump** — each pump may hold a different syringe, so the
   same mm/min plunger feedrate maps to a **different µL/s** (µL/s =
   `syringe.mm_to_uL(mm_min / 60)`). One row per configured pump, each with its own
   mm/min spin + its own µL/s readout.

## Design

- `safety_limits.max_pump_feedrate` (scalar, mm/min) is **kept as the global
  default / fallback** (back-compat: nothing that reads it breaks). Per-pump
  overrides are new fields `max_pump_feedrate_p1/p2/p3` (mm/min, `0.0` = "use the
  global"). This auto-persists through `SafetyLimits.to_dict`/`from_dict`.
- The raw-plunger feedrate clamp (`clamp_pump_feedrate`) becomes **per-pump**, so
  the actual safety clamp on `move_pump_relative` honors the pump's own ceiling.
- The µL/s readout is derived per pump from the configured syringe via the
  controller (`pump_feedrate_mm_min_to_uL_s`), so a pump with no syringe shows `—`.

## Files Modified

- `SupportClasses/SafetyLimits.py` — per-pump `max_pump_feedrate_p{1,2,3}` fields +
  `pump_feedrate_max` / `set_pump_feedrate_max` + `clamp_pump_feedrate(feedrate, pump)`.
- `SupportClasses/StageController.py` — pass `pump` to `clamp_pump_feedrate`;
  `get_pump_max_feedrate_mm_min` / `set_pump_max_feedrate_mm_min` /
  `pump_feedrate_mm_min_to_uL_s` / `pump_max_feedrate_uL_s`.
- `gui/pages/hardware/control_panel.py` — Hardware-Setup "Max speed (per axis)"
  section: replace the single **Pump** row with **one row per pump** (mm/min spin +
  live µL/s readout); seed/write/refresh per pump; hide unconfigured pumps; the
  Hardware-Setup mm pump-jog reads the per-pump max feedrate.
- `tests/test_v75x_per_pump_max_rate.py` — new.

## Implementation Steps

- [x] SafetyLimits per-pump fields + accessors + per-pump clamp.
- [x] StageController per-pump feedrate helpers + µL/s conversion + pass pump to clamp.
- [x] Control Panel per-pump rows + µL/s readout + seed/write/refresh + jog read.
- [x] Tests.
- [x] Run affected suites.

## Status Tracking

Implementation complete (pending real-HW/GUI verification on ME3B V1/V3). The
Hardware-Setup "Max speed (per axis)" section shows one **mm/min** spin + a live
**µL/s** readout per configured pump; unconfigured pumps are hidden. Editing a
pump's mm/min updates only that pump's µL/s + clamp; a syringe/pump change
re-seeds the rows via `hardware_setup._refresh_max_flow_display`.

## Test results

Green (unittest): `test_v75x_per_pump_max_rate` (11, new) +
`test_v75x_axis_max_speed_inputs` (updated for the per-pump rows);
`common_axis_speed_source`, `xbox_axis_speed_percent`, `test_v731_jog_navigation`,
`pump_plunger_setup`, `jog_direction_z_up_sign`, `zp_envelope_absolute`,
`jog_motion_interpolation`, `last_known_calibration`, `cal_z_envelope_no_clobber`,
`axis_map_jog` (197 total).

## Testing Notes

`python -m pytest tests/test_v75x_per_pump_max_rate.py
tests/test_v75x_common_axis_speed_source.py tests/test_v75x_axis_max_speed_inputs.py
tests/test_v731_jog_navigation.py -q`

Real-HW (ME3B V1/V3): on Hardware Setup, each configured pump shows its own mm/min
max spin + a live µL/s readout that reflects that pump's syringe; editing one pump's
mm/min updates only that pump's µL/s and clamps only that pump's plunger jog/move;
restart preserves per-pump values.

## Issues & Decisions

- Global `max_pump_feedrate` kept as the fallback default (safety + back-compat) —
  a per-pump value of `0.0` means "inherit the global".
- µL/s is a **derived readout** only; mm/min stays the stored/edited primary (the
  firmware plunger unit), matching the request ("µL/s as a secondary unit").
