# MEBP v7.5.x — Jog / Travel Motion Interpolation ("live-polling feel")

## Objective

When the operator issues a jog or travel move, the position readouts and the
live needle currently **snap to the destination** (or sit frozen) because the
move command is **fire-and-forget** and the real position **cannot be polled
while the axis is moving** (the `PositionPoller` is ~300 ms and is *suspended*
during `safe_travel_to` / prints). The hardware actually takes time to traverse.

This update makes the GUI **animate the displayed position over time** — a
display-only *dead-reckoning* estimate. At the moment a move is issued the code
already knows **both the clamped destination and the axis speed**, so the
estimate interpolates `start → target` linearly by `elapsed·speed / distance`
and the display follows it "as if it were live-polling," then **snaps to the
real polled value on arrival**. Continuous Xbox velocity jog is handled by
accumulating the commanded per-segment deltas ("advance" mode).

**Scope (operator-confirmed):** *all jog & travel moves* — point-to-point
go-to / click-to-travel, incremental button/key jogs, and the Xbox continuous
velocity jog. **Readout style (operator-confirmed):** while interpolating, the
number is **marked as estimated** (a `~` prefix + tooltip) and snaps to the real
value on arrival.

### Non-goals / design boundaries

- **Display-only.** The estimate NEVER commands hardware and NEVER affects
  motion, clamping, or safety. The real `PositionPoller` cache remains the
  single source of truth for every non-display consumer (`wait_for_*_arrival`,
  calibration, print recorder, etc. are untouched). On any error the display
  getter falls back to the raw cache.
- **Linear (constant-speed) model**, matching the operator's "aware of the speed
  … interpolate the move in time." Accel/decel ramps are not modelled (Marlin
  acceleration is not reliably known per jog); a slightly-off speed only makes
  the animation reach the target a little early/late, then the poller corrects.
- **Estimates are gated to the non-suspended poller.** Programmatic sequences
  that suspend the poller (`safe_travel_to`, `PRINT_PATH`) do not register new
  estimates — this cleanly excludes prints (no per-segment churn) and keeps the
  estimator scoped to user-initiated fire-and-forget moves. A GUI-thread-blocking
  go-to can't animate anyway (the event loop is blocked); animation materialises
  for fire-and-forget moves and the Xbox jog, which is where the symptom lives.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/MotionEstimator.py` | **NEW** — pure-Python, thread-safe, Qt-free display-only estimator. `note_target` (interpolate to a known destination at a known speed) + `advance` (accumulate continuous-jog deltas) + `estimate`/`active_channels`. Injectable clock for tests. |
| `SupportClasses/StageController.py` | Instantiate `self._motion`; new display getters `get_display_xy_position` / `get_display_zp_position`; `motion_estimate_active()` / `motion_estimating()`; guarded hook helpers `_note_move_estimate_xy` / `_note_move_estimate_axis` + advance callbacks `_note_xy_jog_segment` / `_note_zp_jog_segment` (gated on poller-not-suspended); register hooks at the bottom of `move_xy_absolute(_um)`, `move_xy_relative(_um)`, `move_z_absolute`, `move_z_relative`, `move_pump_relative`; wire `on_jog_estimate` callbacks onto the jog handlers in `connect_stages`. |
| `gui/app.py` | New ~33 ms `_motion_anim_timer` → `_motion_anim_tick` (active only while `motion_estimate_active()`); factor `_update_status_bar_positions()` out of `_update_status` and source it from the display getters + `~`/"moving" cue; forward a `on_motion_tick()` to the active page. |
| `gui/pages/jog_control.py` | `on_status_update` + new `on_motion_tick` refresh the workspace/XZ needles from the display getters (factored `_refresh_live_needles`); forward motion ticks to the context panel. |
| `gui/pages/hardware/control_panel.py` | `on_status_update` + new `on_motion_tick` source positions from the display getters; `_update_position_displays` adds a `~` prefix + tooltip for estimated axes. |
| `gui/pages/hardware/stage_panel.py` | `_refresh_jog_positions` sources positions from the display getters; new `on_motion_tick`. |
| `tests/test_v75x_jog_motion_interpolation.py` | **NEW** — unit tests for the estimator + controller display getters with a fake stage and injected clock. |

## Frames (match the existing caches exactly)

- `"XY"` channel: 2-tuple `(x, y)` **absolute stage µm** (same as `PositionPoller.xy_position`).
- `"Z" / "P1" / "P2" / "P3"` channels: scalar **raw Marlin mm** (the value at that
  logical axis's physical slot in `PositionPoller.zp_position`, resolved via the
  live `axis_map` with `_axis_index`).

## Implementation Steps

- [x] 1. `MotionEstimator` class (new module).
- [x] 2. StageController: instantiate + display getters + `motion_*` accessors.
- [x] 3. StageController: guarded hook helpers + register at the bottom of every
      `move_*` primitive (after the hardware send), gated on poller-not-suspended.
      (Relative moves call `_note_move_estimate_*_rel`, which read the cache
      *inside* the gate so a partially-constructed controller is never touched.)
- [x] 4. StageController: jog-loop advance callbacks (`on_jog_estimate`) + wired
      onto handlers in `connect_stages`; XY/ZP `_jog_loop` call them per segment.
- [x] 5. app.py: 33 ms `_motion_anim_timer` + factored `_update_status_bar_positions`
      + `~` cue; timer stopped in `closeEvent`.
- [x] 6. Pages: jog (`_refresh_live_needles` + `on_motion_tick`), control panel
      (`_display_xy/_zp` + `on_motion_tick` + `~`), stage panel (display getters +
      `on_motion_tick` + `~`), Hardware Setup page forwards `on_motion_tick`.
- [x] 7. Tests (`tests/test_v75x_jog_motion_interpolation.py`, 29) + regressions.

### Status: implemented; tests green (pending real-HW verification)

`tests/test_v75x_jog_motion_interpolation.py` — **29 pass** (estimator math,
display getters, registration hooks + suspended-poller gate, jog-segment
accumulation, offscreen page cue smoke). Regressions green: axis-map-jog /
jog-direction / jog-navigation / pump-settle / z-retract (139); quick-print /
simple-PM / print-setup-routine (74); z-display / pump-plunger / xbox×2 (76);
common-axis-speed / always-safe-z / jog-pump-fill / jog-nav (67).

## Testing Notes

- Unit (offline, injected clock): `note_target` interpolates linearly and lands
  on target at `t = duration`, then goes idle (defers to cache); zero-distance →
  instant; `advance` accumulates commanded deltas from a cache-locked base and
  expires after `settle_s`; `estimate` returns `None`/raw on any malformed input;
  display getters override only the estimated channel/slot and fall back to the
  raw cache otherwise; `motion_estimating()` maps `"XY"` → `{X, Y}`.
- Regression: jog-direction / axis-map / jog-navigation / pump-settle / quick-print
  suites stay green (display getters == cache when no estimate is active, so
  every existing consumer is byte-identical).
- **Needs real-HW verification on ME3B V1:** jog XY / Z / a pump and watch the
  readout + needle glide toward the destination (marked `~`) and snap to the real
  value on arrival; hold the Xbox stick and confirm the readout tracks smoothly
  instead of stepping every 300 ms.

## Issues & Decisions

- **Display-only, separate getters** (not mutating `get_*_position(cached=True)`):
  the cached reads are consumed by clamping in the move methods and by the Xbox
  jog loops; feeding them a predicted value could perturb relative-move deltas.
  New `get_display_*` getters are wired into display sites ONLY; everything else
  keeps the real cache.
- **`advance` mode does not re-base to the cache mid-jog** (base is locked from
  the cache at jog start; the estimate = base + Σ commanded deltas): a velocity
  jog tracks the commanded velocity closely, so this shows "where we've commanded
  it" with no sawtooth, and reconciles to the real value on stop (estimate expires
  → cache).
- **Estimates gated to poller-not-suspended** — excludes prints / `safe_travel_to`
  (programmatic, often GUI-blocking). Known boundary: a worker-thread go-to that
  suspends the poller won't animate; revisit if needed.
