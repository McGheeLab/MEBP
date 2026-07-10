# MEBP v7.5.x — Hardware Setup edits ABSOLUTE per-axis max speeds + pump jog % below 1%

## Objective

Two operator-requested changes to the shared jog/speed panel (`HardwareControlPanel`):

1. **Pump jog rate can go below 1%.** On the jog/move pages the pump speed is a
   *% of the max*; the spinbox was floored at 1 %. Let it go below 1 % (down to
   0.01 %, 2 decimals) so fine plunger jogs are possible.
2. **The Hardware Setup ("hardware calibration") page should NOT be in percentages.**
   Its jog-speed section showed *"% of max"* like every other page. On Hardware
   Setup those inputs now edit the **absolute maximum speed per axis** (XY µm/s,
   Z mm/min, Pump mm/min) — the single common ceiling that every other page's %
   is relative to. Editing one writes the one shared source + fans the change out.

This reuses the existing "single common per-axis speed source" model
(`MEBP_v75x_COMMON_AXIS_SPEED_SOURCE_AND_PUMP_UNITS.md`): resolvers
`get_max_xy_speed_um_s` / `get_max_z_feedrate_mm_min` / `get_max_pump_feedrate`
and the `notify_speed_limits_changed` → `on_speed_limits_changed` fan-out.

## Discriminator

Hardware Setup is the only page that builds the panel in calibration mode
(`HardwareControlPanel()` default: `pump_action_labels=False`, raw-mm pump). All
jog/workflow/calibration pages go through `StandardJogContextPanel`
(`pump_action_labels=True`, %/µL). A new explicit `speed_as_max: bool = False`
kwarg (passed `True` only from `hardware_setup.py`) selects absolute-max mode, so
existing panels and tests are unchanged by default.

## Files Modified

- `gui/pages/hardware/control_panel.py` — `speed_as_max` mode: absolute-max
  spinboxes + `_write_axis_max` (writes the shared source + persist + notify) +
  `_seed_axis_max_from_controller`; percent-mode pump spin floor 0.01 % / 2 dec;
  jog handlers use the absolute value in max mode.
- `gui/pages/hardware_setup.py` — build the panel with `speed_as_max=True`.
- `tests/test_v75x_axis_max_speed_inputs.py` — new.

## Implementation Steps

- [x] `_pct_spin(min_pct, decimals, step)` parametrized; pump spin floor 0.01 %.
- [x] `_max_spin(maximum, decimals, step, unit)` for absolute-max mode.
- [x] `_build_speed_controls` branches on `speed_as_max`; `_build_max_speed_controls`.
- [x] `_on_speed_pct_changed` / `refresh_speed_limits` / jog handlers branch on max mode.
- [x] `_write_axis_max` (XY→`safety_limits.max_xy_speed`; Z→`per_axis_max_feedrate['Z']`
  via `apply_device_settings` + mirror `max_z_feedrate`; Pump→`max_pump_feedrate`) +
  `notify_speed_limits_changed`.
- [x] `hardware_setup.py` passes `speed_as_max=True`.
- [x] Tests.

## Status Tracking

Implementation complete; pending real-HW verification on ME3B V1.

## Testing Notes

`python -m pytest tests/test_v75x_axis_max_speed_inputs.py
tests/test_v75x_common_axis_speed_source.py tests/test_v75x_xbox_axis_speed_percent.py -q`

Real-HW (ME3B V1): Hardware Setup jog panel shows "Max speed (per axis)" with
absolute units; editing XY/Z/Pump max is reflected on every % page's resolved
speed. Jog/workflow pump % goes below 1 %.

## Issues & Decisions

- Sub-1 % scoped to the **pump** per the request; XY/Z keep the 1 % floor.
- Hardware Setup pump max stays **mm/min** (matches its raw-mm plunger jog + the
  no-needle fallback anchor); the needle-derived µL/s ceiling on %/µL pages is
  still derived, not user-set.
- Jogging on Hardware Setup runs at the entered absolute speed (= the max), which
  is the intended calibration workflow (set the speed, jog to test it, it is the
  ceiling). Safe: max speeds were always reachable at 100 % on % pages.
