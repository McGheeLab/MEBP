# MEBP v7.5.x — Jog direction unified on `z_up_sign` + soft-limit clamp can only shorten a jog

## Objective

Two operator-reported hardware bugs:

1. **"The jog axis is reversed."** The Z jog should move in the direction
   established by the **Set Bottom / Set Top** setup (the per-machine
   `z_up_sign`). Right now jog direction is hardcoded per-widget and the
   on-screen buttons, the keyboard, and the Xbox stick **disagree** with each
   other and with the position readout.

2. **"Z went negative, then the Xbox controller clamped and kept moving the
   axis constantly."** When the cached jog position is *outside* the soft-limit
   envelope, the velocity jog loop's clamp synthesizes a large delta back toward
   the limit and re-sends it every segment → constant motion.

Make **`z_up_sign` (derived by `apply_z_setup`) the single source of truth for
jog direction** across every jog input, and make the soft-limit clamp on a
*relative* jog only ever **shorten** the requested move (never reverse or
amplify it).

## Root cause

The machine has **two Z-direction sign systems that no longer agree**:

| Source | Value | Where |
|--------|------|-------|
| Per-machine `z_up_sign` | **+1** (from Set Bottom/Top, loaded at startup) | `raw_to_user_z`, readouts, `z_height_of`, `ensure_retracted_to` |
| Module `ZDIR` constant | **−1** (hardcoded) | `z_raw_to_display`, fallbacks |
| Hardcoded jog-button signs | "Z▲ = −step" | `JogButtonArray` |

`z_up_sign=+1` is correct for the **current** hardware because
`steps_per_mm.Z = −5255` (negative) inverts the raw Marlin Z frame (M92 sends
the sign to Marlin) — the needle now rises as raw Z increases. But the module
`ZDIR=−1` and the hardcoded jog signs still encode the **old** `+5255` polarity
(the `ARCHITECTURE`/`Z_AXIS_CONVENTION_RETHINK` docs still list `+5255`), so:

- on-screen **Z▲** ([jog_button_array.py:305](../../gui/widgets/jog_button_array.py#L305)) emitted `−step` → needle **down**;
- keyboard **PageUp** ([jog_control.py:576](../../gui/pages/jog_control.py#L576)) emitted `+step` → needle **up** (opposite the button);
- **Xbox** ([StageController.py](../../SupportClasses/StageController.py)) `dz = −vz·seg·_flip_sign("Z")` → reversed in the new frame;
- the position **display** uses `z_up_sign` → numbers disagree with the buttons.

**Clamp runaway:** the ZP jog loop clamps with `clamp(cur+delta) − cur` using
the **cached** poller position. When `cur` is outside the envelope, that is a
large delta *opposite* the requested direction; the loop re-sends it every
0.12 s before the cache catches up. The `1e-4` freeze-gate doesn't stop it
(the delta is large, not a residual).

## Design (user-confirmed: `z_up_sign` authoritative; all jog inputs)

- **Height-frame jog contract.** `JogButtonArray.jog_z_requested(dz_mm)` now
  carries a **height-frame** delta (`+` = up, toward the taught Top). Z▲ emits
  `+step`, Z▼ emits `−step`.
- **One conversion point.** New `StageController.move_z_user_relative(user_dz,
  feedrate, bypass_safety)` converts height→raw with `Δraw = Δuser · z_up_sign`
  and calls the raw primitive `move_z_relative`. Every jog input routes through
  it, so "up" always moves toward the taught Top regardless of raw-Z polarity.
- **Retire the Z `axis_flip`.** `move_z_relative` no longer multiplies by
  `_flip_sign("Z")` (the axis-flip UI was removed in v7.4.2 and Apply
  force-zeros `axis_flip.z`, so it was already a dead `×1.0`). Z direction is
  owned solely by `z_up_sign`; pumps keep `_flip_sign`.
- **Xbox** uses `dz = vz·seg·z_up_sign` (a `z_up_sign_provider` is passed into
  `ZPJogHandler`). This is algebraically identical to the old `−vz·seg` when
  `z_up_sign=−1`, and correctly flips when `z_up_sign=+1`.
- **Shorten-only clamp** (`_shorten_only_delta`): a relative jog's clamp may
  never reverse the sign of the request or exceed it in magnitude. A jog that
  would push further out of bounds becomes a no-op; a jog back toward the range
  moves by at most the requested step. Applied in `move_z_relative`,
  `move_pump_relative`, and the ZP jog loop.

## Files Modified

- `SupportClasses/StageController.py` — `_shorten_only_delta` (module fn);
  `move_z_user_relative`; `move_z_relative` (drop Z flip, shorten-only clamp);
  `move_pump_relative` (shorten-only clamp); `ZPJogHandler` (`z_up_sign_provider`
  + `_z_up_sign()` + `_jog_loop` dz formula + `_clamp_delta`); pass provider at
  the ZP-jog construction site.
- `gui/widgets/jog_button_array.py` — Z▲ = `+1`, Z▼ = `−1`; contract docstring.
- `gui/pages/hardware/control_panel.py` — `_on_jog_z` → `move_z_user_relative`.
- `gui/pages/hardware/stage_panel.py` — `_on_jog_array_z` → `move_z_user_relative`.
- `gui/pages/calibration.py` — `_zteach_jog_z` → `move_z_user_relative`.
- `gui/pages/settings_page.py` — `_ctx_jog_z` → `move_z_user_relative`.
- `gui/pages/jog_control.py` — `_key_jog_z` → `move_z_user_relative`.
- `tests/test_v75x_jog_direction_z_up_sign.py` — new.

## Implementation Steps

- [x] `_shorten_only_delta(cur, requested, clamped_dest)` module helper
- [x] `move_z_user_relative` (+ retire Z flip in `move_z_relative`)
- [x] shorten-only clamp in `move_z_relative` / `move_pump_relative` / ZP loop
- [x] `ZPJogHandler` z_up_sign provider + dz formula
- [x] flip `JogButtonArray` Z + update contract docs
- [x] route all 4 array consumers + keyboard through `move_z_user_relative`
  (`control_panel`, `stage_panel`, `calibration._zteach_jog_z`,
  `settings_page._ctx_jog_z`, `jog_control._key_jog_z`)
- [x] tests — `tests/test_v75x_jog_direction_z_up_sign.py` (20)
- [x] run affected suites

## Testing Notes

- Unit: `_shorten_only_delta` invariant (in-bounds unchanged; out-of-bounds
  push-out → 0; push-in ≤ requested; at-wall push-out → 0).
- `move_z_user_relative` sends `user_dz · z_up_sign` for both `±1`.
- `JogButtonArray`: Z▲ emits `+step`, Z▼ emits `−step` (offscreen Qt).
- `ZPJogHandler`: stick-up → +raw dz for `z_up_sign=+1`, −raw dz for `−1`; an
  out-of-bounds cached position does not produce a large opposite-direction
  delta.
- **Verified on ME3B V1 (bench, 2026-06-16):** Z▲/PageUp/stick-up retract the
  needle toward the taught Top, numbers increase, and jogging into a soft limit
  no longer runs away. Direction follows the `z_up_sign` from Set Bottom/Top.

Result: `test_v75x_jog_direction_z_up_sign` = 20 green. Affected suites green:
`xbox_input_pipeline`, `zp_jog_clamp_freeze`, `z_display_numbering`,
`z_axis_unified_setup`, `axis_map_jog`, `cal_z_envelope_no_clobber`,
`z_retract_before_xy_travel`, `zp_envelope_absolute`. Full `test_v75x_*` =
449/450 (the 1 failure, `test_v75x_needle_center_direction_z::
test_result_is_normalized`, is a PRE-EXISTING needle-centering angle-edge bug in
`pixel_calibration_dialog.plus_column_direction_deg` — unrelated to this change
and untouched here).

## Issues & Decisions

- Chose `z_up_sign` authoritative over reverting `steps_per_mm.Z` to `+5255`
  (would re-invert the motor and contradict the just-derived setup).
- `move_z_relative` stays the raw/zero-ref delta primitive used by
  print/calibration; only the **jog UI** routes through the new height-frame
  method. Print/calibration Z math is untouched.
- Module `ZDIR=−1` remains as a latent inconsistency for non-jog fallbacks;
  out of scope here (separate convention-cleanup), noted for follow-up.
