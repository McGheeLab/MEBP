# MEBP v7.5.x — Xbox jog speed as a percentage of the calibrated max (per axis)

## Objective

On the Xbox page, let the operator control the jog speed of each
joystick-driven axis **as a percentage of that axis's calibrated max move
speed**, cycled with the controller through a fixed ladder:

```
0.1%  0.3%  1%  3%  10%  30%  100%
```

100% = the per-axis max already calibrated on the Hardware Setup pages (the
same values the Control Panel "Speeds" seed from). The controller cycles a
group up/down one rung at a time via the existing, already-mapped
`increment_{xy,z,p}speed_up/down` commands. The Xbox page shows a **read-out**
of each group's current % and the resolved absolute speed.

### User decisions (asked before implementing)

- **Grouping:** XY / Z / Pump (3 groups) — matches the existing controller
  speed buttons and how the joystick physically moves the stage (one stick =
  XY together, triggers = Z/pumps). *Not* fully per-motor.
- **On-screen UI:** show-only read-out (no on-screen step buttons). The value
  is changed only via the controller; the page displays it. The selected %
  still **persists across restarts** (that's behaviour, not a UI control).

## Background (what already existed)

- The Xbox worker (`SupportClasses/XboxController.py`) sends normalized axis
  values; `ZPJogHandler` (Z + pumps) and `XYJogHandler` (XY) in
  `SupportClasses/StageController.py` scale them by per-group speed multipliers
  (`z_speed`, `p_speed`, `xy_speed`) and clamp at a `max_speed`.
- `increment_{xy,z,p}speed_up/down` are real, mappable commands (default-mapped
  to bumpers/D-pad in `current_button_mapping.json`) — but they stepped by
  **decades (×10 / ÷10)** with arbitrary caps, *not* a % of calibrated max.
- The calibrated maxes already live in settings (read by the Control Panel):
  - XY  → `safety_limits.max_xy_speed` (µm/s)
  - Z   → `device_profile.per_axis_max_feedrate["Z"]` (mm/min) → ÷60 → mm/s,
          falling back to `safety_limits.max_z_feedrate`
  - Pump→ per-pump flow limit `safety_limits.get_max_flow_rate(pid)` (µL/s) in
          µL mode; else `safety_limits.max_pump_feedrate` (mm/min) ÷60 → mm/s

## Design

The handler speed scalar for an axis becomes a **derived** value:

```
effective_speed = (speed_pct / 100) * speed_max_native
```

where `speed_max_native` is the calibrated max in the handler's native
velocity unit and `speed_pct` is the current ladder rung. The velocity clamp
(`max_speed`) is set to `speed_max_native` so 100% reaches — and never exceeds
— the calibrated max. The `increment_*` commands step `speed_pct` along
`JOG_SPEED_LADDER_PCT`.

- The per-axis **max** is sourced once by the controller
  (`refresh_jog_speed_limits()`) from `self.safety_limits` +
  `self._pending_per_axis_max_feedrate` + the hardware config's flow limits —
  no settings access in the jog handlers, no unit logic in the GUI.
- The per-axis **%** is owned by the controller
  (`_jog_speed_pct` dict) so it survives (re)connect: the GUI restores the
  persisted % into the controller once; the controller re-applies it whenever
  handlers are (re)created. The controller never reaches into Settings.

The pump 100% anchor is the largest configured safe flow rate (µL/s) — slower
pumps are still independently clamped to their own safe rate by the existing
`_clamp_pump_flow`, so anchoring to the fastest never lets a pump exceed its
limit.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | New `JOG_SPEED_LADDER_PCT` + `_jog_speed_ladder_step()`. `ZPJogHandler`/`XYJogHandler`: `speed_pct`/`speed_max` state, derived `*_speed`, recompute helpers, ladder-stepping `_incr_*`, `set_*` setters. Controller: `refresh_jog_speed_limits()`, `_pump_jog_max_native()`, `set_jog_speed_pct()`, `get_jog_speed_state()`, `_apply_jog_speed_pct()`; call refresh on jog-handler (re)creation + at end of `apply_device_settings` + in `set_hardware_config`. |
| `gui/pages/hardware/xbox_panel.py` | New "Jog Speed (per axis)" read-out card (XY/Z/Pump: % + resolved speed), fed at 20 Hz from `controller.get_jog_speed_state()`. Restore persisted % into the controller in `_load_from_settings`; persist on change (panel tick detects controller-side change). |
| `tests/test_v75x_xbox_axis_speed_percent.py` | New test suite. |
| `CLAUDE.md` | Add this plan to the Existing Update Plans table. |

## Implementation Steps

- [x] `JOG_SPEED_LADDER_PCT` + `_jog_speed_ladder_step(current_pct, direction)` helper
- [x] `ZPJogHandler`: `z_speed_pct`/`p_speed_pct`/`z_speed_max`/`p_speed_max`, `_recompute_zp_speeds`, derived `z_speed`/`p_speed`, ladder `_incr_z_*`/`_incr_p_*`, setters, Z clamp at `z_speed_max`, pump cap at 100%-flow
- [x] `XYJogHandler`: `speed_pct`/`speed_max`, `_recompute_xy_speed`, ladder `_incr_*`, setters
- [x] Controller `refresh_jog_speed_limits()` + `_pump_jog_max_native()` + `set_jog_speed_pct()`/`get_jog_speed_state()`/`_apply_jog_speed_pct()`
- [x] Call `refresh_jog_speed_limits()` after jog-handler creation (connect), end of `apply_device_settings`, in `set_hardware_config`
- [x] Xbox page read-out card + restore/persist of the per-axis %
- [x] Tests
- [x] CLAUDE.md table row

## Testing Notes

`python -m pytest tests/test_v75x_xbox_axis_speed_percent.py` and the existing
`tests/test_v75x_xbox_input_pipeline.py` must stay green. Covered:

- Ladder stepping clamps at both ends; lands on nearest rung from an arbitrary %.
- `increment_zspeed_up/down` (via Processor command) walks the ladder and the
  derived `z_speed` tracks `pct/100 * z_speed_max`.
- XY `increment_*` walks the ladder; `xy_speed`/`max_speed` track the max.
- Z velocity clamps at `z_speed_max` (100% reaches it, never exceeds).
- `refresh_jog_speed_limits()` converts mm/min → mm/s for Z and reads
  `max_xy_speed` for XY; pump anchor = largest configured flow rate.
- `_apply_jog_speed_pct()` re-applies a persisted % on handler (re)creation.
- `get_jog_speed_state()` reports {pct, speed, unit} per present group.

## Issues & Decisions

- **Pump units:** the pump handler scalar is µL/s (µL mode) so the % is
  anchored to flow rate (µL/s), not the mm/min feedrate. In legacy/no-config
  mode it falls back to `max_pump_feedrate` ÷ 60 (mm/s). The existing
  `_clamp_pump_flow` still enforces the per-pump safe rate, so anchoring to the
  fastest configured pump cannot let a slower pump exceed its limit.
- **Default %:** 10% on first run (before any max is pushed the handler maxes
  default to 1.0 mm/s Z / 10 µL/s pump / 10000 µm/s XY). Once calibration is
  applied the % is relative to the real hardware max.
- **Decade behaviour removed:** the old `×10/÷10` `_incr_*` semantics are
  replaced by the ladder; the default mapping (bumpers/D-pad) is unchanged.
- **Needs real-HW verification on ME3B V1.**
