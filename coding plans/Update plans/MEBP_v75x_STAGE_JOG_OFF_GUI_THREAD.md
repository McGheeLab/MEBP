# MEBP v7.5.x — XY / Z jog off the GUI thread (camera freeze)

## Objective

Fix the operator report: **"the camera freezes whenever I use any other stages
in jog mode."** The pump jog was already moved off the GUI thread
(`MEBP_v75x_PUMP_JOG_OFF_GUI_THREAD.md`), but the **XY and Z** incremental-jog
handlers still ran synchronously on the GUI thread, so jogging those axes froze
the live camera feed.

## Root cause

`HardwareControlPanel._on_jog_xy` / `_on_jog_z` (the jog-button-array handlers,
used everywhere the "Jog" pill panel appears — the Jog page, Calibration, jog-
capable workflows) did **two** blocking things on the Qt GUI thread per jog:

1. the move itself — `move_xy_relative_um` / `move_z_user_relative` (a serial
   round-trip; for Z, the Marlin `ok`-handshaked `send_data`), and
2. `_force_refresh_positions()` — which does **two** `cached=False` serial reads
   (`get_xy_position` + `get_zp_position`, i.e. a Prior `P` query and a Marlin
   `M114`), each a blocking round-trip, contended with the position poller.

While the GUI thread is blocked in those round-trips, no `QTimer`-driven camera
feed (`camera_widget.py`) can tick, so the live view appears frozen — the same
class of bug as `MEBP_v75x_PUMP_JOG_OFF_GUI_THREAD.md` /
`MEBP_v75x_JOG_TRAVEL_OFF_GUI_THREAD.md`. (The button jog was the worst case:
move **plus** two fresh reads. The pump path had already been fixed; XY/Z were
the remaining synchronous jog handlers.)

## Fix

`gui/pages/hardware/control_panel.py` — mirror the pump-jog pattern for XY/Z:

- New `_run_stage_jog(busy_attr, move_fn, label)` runs `move_fn()` **and** the
  fresh `get_xy_position(cached=False)` / `get_zp_position(cached=False)` reads on
  a daemon thread (serial access is `_serial_lock`-protected, so reading off the
  GUI thread is safe), then hands the results back to the GUI thread via a new
  queued `_stage_jog_done(object, object)` signal.
- `_on_stage_jog_done(xy, zp)` (GUI thread) updates the readout from the
  worker-read positions via `_update_position_displays` — **no serial I/O on the
  GUI thread at all** (falls back to the cached display values if a read failed).
- `_on_jog_xy` / `_on_jog_z` now resolve the jog speed from the GUI spin (cheap),
  then dispatch the speed-set + move via `_run_stage_jog`. Per-axis busy guards
  (`_xy_jog_busy` / `_z_jog_busy`) **drop overlapping clicks** rather than queuing
  blocking moves behind the shared serial channel (independent guards, so an XY
  jog isn't blocked by an in-flight Z jog). The `_apply_xy_speed` serial write
  now happens inside the worker too.

Jog behaviour and the emitted signal contract are otherwise unchanged.

**Not changed (deliberately):** the Jog page's **keyboard** jog
(`jog_control._key_jog_xy` / `_key_jog_z`) still runs on the GUI thread, but those
are the move only (no fresh reads) — a much smaller cost than the button jog's
move + two `cached=False` reads, and key auto-repeat wants tight coupling. Left
as a documented follow-up if a freeze is observed on held-key jogging.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/hardware/control_panel.py` | `_stage_jog_done` signal + `_xy_jog_busy`/`_z_jog_busy` guards; `_on_jog_xy`/`_on_jog_z` dispatch through new `_run_stage_jog` (move + fresh reads on a daemon thread); `_on_stage_jog_done` updates the readout on the GUI thread with no serial I/O. |
| `tests/test_v75x_axis_max_speed_inputs.py` | Test 5a now `_wait_for`s the backgrounded `set_velocity` (the XY speed-set is now off-thread). |
| `tests/test_v75x_stage_jog_off_gui_thread.py` | **New** — 3 tests: XY jog returns promptly with the blocking move on the worker + overlapping click dropped + fresh reads off-thread; Z jog off-thread; XY/Z guards independent. |

## Implementation Steps

- [x] `_stage_jog_done` signal + per-axis busy guards + `_run_stage_jog` / `_on_stage_jog_done`
- [x] `_on_jog_xy` / `_on_jog_z` dispatch off-thread (speed resolved on GUI, applied in worker)
- [x] Update test 5a to wait for the backgrounded call
- [x] New test module (3 tests)
- [x] Compile + jog/speed/context regression green (136)

## Testing Notes

- Automated (offscreen, `MEBP_UI_SCALE=1.0`): `tests/test_v75x_stage_jog_off_gui_thread.py`
  (3) + `test_v75x_axis_max_speed_inputs` / `_common_axis_speed_source` /
  `_jog_direction_z_up_sign` / `_per_pump_max_rate` / `_jog_pump_fill_readout` /
  `_jog_motion_interpolation` / `_xbox_axis_speed_percent` / `_responsive_context_panel`
  — 136 green.
- **Needs real-HW verification on ME3B V1/V3:** jog XY and Z (arrow / Z buttons)
  while watching a live camera feed — the feed keeps updating (no freeze); the
  position readout still updates after each jog.

## Issues & Decisions

- **Busy guard drops overlapping clicks** (per axis) rather than queuing — the
  same choice as the pump jog; prevents blocking moves piling up on the shared
  serial channel and the stage lagging behind the clicks.
- **Reads moved into the worker** (unlike the pump path, whose `_on_pump_jog_done`
  still refreshes on the GUI thread): for XY/Z the fresh read is the *dominant*
  blocking cost, so it must be off-thread; the worker reads and the GUI slot only
  paints. The pump path's post-move refresh is comparatively cheap next to a
  multi-second backlash-compensated move and was left as-is.
