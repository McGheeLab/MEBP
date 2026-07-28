# MEBP v7.5.x — Pump Jog Off the GUI Thread (camera freeze fix)

## Objective

Fix operator report: **"when I use the 3D printer board to move the pump, the
cameras stop displaying frames."** Root cause is the same class of bug as
`MEBP_v75x_JOG_TRAVEL_OFF_GUI_THREAD.md` (Z-retract-before-XY-travel freezing
the app) — a blocking, multi-second hardware call was being run directly on
the Qt GUI thread, so the event loop (and every `QTimer`-driven camera feed,
via `CameraWidget._grab_frame`) stalls for the duration.

## Root Cause

The shared pump-jog UI (`HardwareControlPanel._on_jog_pump` in
`gui/pages/hardware/control_panel.py`, embedded everywhere via
`StandardJogContextPanel` — the "Jog" pill panel on the Jog page, Calibration,
and every jog-capable Workflow tile — as well as directly on Hardware Setup)
called `StageController.move_pump_uL(...)` **synchronously in the button
click handler**.

On ME3B V3, `device_profile.backlash_comp_enabled` is `true`. With backlash
compensation on, `move_pump_uL(compensate=True)` brackets the click with a
TAKE-UP move → MAIN move → UNLOAD move, and (per its own docstring) each
sub-move **blocks** via `StageController._wait_pump_move_complete` — an M400
drain wait — plus a `pump_settle_time_s()` dwell after the take-up and after
the unload. That is up to three blocking serial round-trips and two sleeps,
all executed on the GUI thread, for a single pump-jog click. While that runs,
Qt's event loop cannot process events, so the `QTimer.timeout → _grab_frame`
tick every camera feed depends on (`gui/widgets/camera_widget.py`) never
fires — the live feed appears to freeze / stop updating for the duration of
the pump move, exactly matching the report.

(Even with backlash compensation off, the same handler still calls
`move_pump_relative`/`move_pump_uL` directly on the GUI thread; those are
normally fast single-command sends, but were left going through the same
off-thread path below for consistency and to avoid depending on Marlin's `ok`
ack always being immediate.)

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/hardware/control_panel.py` | `HardwareControlPanel._on_jog_pump` now runs the actual `move_pump_uL`/`move_pump_relative` call on a daemon thread (mirrors the existing `SafeTravelWorker` / `calibration.py::_compcal_step_move` pattern), with a busy-guard so overlapping clicks are ignored rather than queued on the shared serial channel. Completion is marshalled back to the GUI thread via a new `_pump_jog_done` Signal, whose slot (`_on_pump_jog_done`) clears the busy flag and calls the existing `_force_refresh_positions()`. |
| `tests/test_v75x_axis_max_speed_inputs.py` | `test_max_mode_pump_jog_absolute_feed` now waits (via a small `_wait_for` Qt-event-pump helper) for the backgrounded mock call before asserting, since the move no longer completes synchronously within `_on_jog_pump`. |
| `tests/test_v75x_common_axis_speed_source.py` | Same `_wait_for` helper added; `test_hardware_setup_pump_jog_is_mm` / `test_other_pages_pump_jog_is_percent_uL` updated to wait for the backgrounded mock call. |

## Implementation Steps

- [x] Identify the blocking call chain: `_on_jog_pump` → `move_pump_uL(compensate=True)` → `_wait_pump_move_complete` (M400 drain) × up to 3, on the GUI thread.
- [x] Confirm `backlash_comp_enabled: true` is a real, persisted setting (`config/hardware/devices/ME3B V3.json`), not a hypothetical.
- [x] Confirm the camera feed is driven by a GUI-thread `QTimer` (`camera_widget.py`), so a blocked event loop directly explains "frames stop displaying."
- [x] Move the pump-jog call (`move_pump_uL` / `move_pump_relative`, both branches — %/µL pages and the Hardware Setup raw-mm path) onto a daemon thread, guarded by a busy flag, with a queued Signal back to the GUI thread for the post-move position refresh.
- [x] Update the 3 existing tests that called `_on_jog_pump` and asserted on the mock synchronously.
- [x] Run affected test suites.

## Testing Notes

`python -m unittest tests.test_v75x_axis_max_speed_inputs tests.test_v75x_common_axis_speed_source tests.test_v75x_per_pump_max_rate tests.test_v75x_jog_motion_interpolation` — 62 tests, all green.

Broader regression spot-check: `tests.test_v75x_pump_compliance_and_backlash`,
`tests.test_v75x_pump_plunger_setup`, `tests.test_v75x_jog_direction_z_up_sign`
— green except one **pre-existing, unrelated** failure
(`test_tabs_restructured` expects a `"Rosettes"` calibration tab absent from
its own expected-titles list; reproduced identically with this change
stashed — a stale test predating the Rosettes tab, not touched here).

**Needs real-HW verification on ME3B V3**: with backlash compensation enabled,
jog a pump from the standard Jog panel while watching a live camera feed —
the feed should keep updating (no freeze) for the duration of the
take-up/main/unload sequence.

## Issues & Decisions

- Scoped to `HardwareControlPanel._on_jog_pump`, the one confirmed
  synchronous, GUI-thread call site for pump jogging. `stage_panel.py`'s
  separate `_on_jog_array_pump` (Hardware Setup → Device jog card) calls
  `move_pump_relative` directly with no `compensate`/`settle` bracketing, so
  it isn't subject to the multi-second blocking chain and was left as-is.
- Followed the repo's existing off-GUI-thread idiom exactly
  (`gui/widgets/safe_travel_worker.py`, `calibration.py::_compcal_step_move`)
  rather than introducing a new abstraction — a plain `threading.Thread` +
  Qt `Signal` bridge, with a busy-guard so rapid re-clicks are ignored, not
  queued.
