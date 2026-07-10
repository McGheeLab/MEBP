# MEBP v7.5.x — Pump soft-limit envelope restored from the plunger calibration on startup

## Objective

Fix the operator report: **"when I restart the software the pump axis does not
load in correctly — the location is correct but the min and max are not."**

After a restart, a calibrated pump's **datum (location)** restored correctly but
its **soft-limit envelope (min/max)** came up wrong (on ME3B V3, P2 came up as
`[0, 30]` instead of the calibrated `[-30, 0]`).

## Root Cause

A calibrated pump's envelope had **two persisted sources of truth that drifted
apart**:

1. `device_profile.pump_setup` — the captured raw extremes (authoritative).
   For P2: `raw_dispensed=0, raw_aspirated=-30` ⇒ envelope **`[-30, 0]`**.
2. `safety_limits.p2_min/max` — a *separate mirror*. On disk it was the stale,
   sign-flipped **`[0, 30]`** (from an earlier `+30` calibration / a
   device-profile apply that overwrote the corrected value).

The startup chain (`main.py`):

1. `safety_limits` restored from `settings.json` → P2 = `[0, 30]` (stale).
2. `apply_pump_convention(device_profile.pump_setup)` restored the **datum**
   (`zero_position["P2"]=0`) and **direction sign** (`-1`) from `pump_setup`,
   but **never restored the soft-limit envelope** — its docstring explicitly
   deferred that to "the `safety_limits` section separately".
3. `set_hardware_config` ran `update_from_hardware_config(skip_pumps=["P2"])` —
   the clobber-guard, which **preserved** the stale `[0, 30]` (it only stops the
   syringe-stroke estimate from overwriting; it never re-derives the calibrated
   envelope).

Net: the calibration owned the datum + direction on reboot but **not the
envelope it captured**, so location was right while min/max were stale — exactly
the symptom.

A secondary re-clobber path: `gui/app.py::_apply_and_save_after_restore` mirrors
the persisted `safety_limits` back into the live envelope (and saves) when a
restore prompt is accepted — which would re-apply the stale `[0, 30]`
mid-session even after a boot-time correction. So the on-disk mirror had to be
corrected too.

## Fix

**Code (durable, self-healing each boot):** `StageController.apply_pump_convention`
now **re-derives the soft-limit envelope from the captured extremes** (exact
extremes, no margin — never command past the mechanical hard stops), matching
`apply_pump_setup`. All three — datum, direction, envelope — are restored from
the one authoritative source (`pump_setup`) on every boot, instead of trusting
the separately-persisted `safety_limits.p*` mirror.

```python
if getattr(self, "safety_limits", None) is not None:
    lo, hi = min(rd, ra), max(rd, ra)
    setattr(self.safety_limits, f"{pump.lower()}_min", lo)
    setattr(self.safety_limits, f"{pump.lower()}_max", hi)
```

**Data (immediate, closes the `_apply_and_save_after_restore` re-clobber hole):**
corrected the stale `safety_limits.p2` mirror from `[0, 30]` → `[-30, 0]` in both
`settings.json` and `config/hardware/devices/ME3B V3.json` so every source agrees
with `pump_setup`.

Uncalibrated pumps (no `pump_setup` entry, e.g. P1/P3) are unaffected — they keep
being recomputed from the syringe-stroke estimate by `update_from_hardware_config`
each boot.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | `apply_pump_convention` re-derives the soft-limit envelope from the captured extremes (+ docstring). |
| `settings.json` | `safety_limits.p2` `[0,30]` → `[-30,0]` (match authoritative `pump_setup`). |
| `config/hardware/devices/ME3B V3.json` | `safety_limits.p2` `[0,30]` → `[-30,0]` (same). |
| `tests/test_v75x_pump_plunger_setup.py` | `TestApplyPumpConvention`: assert envelope restored in `test_restore_sign_and_datum`; new `test_envelope_restored_overrides_stale_mirror`. |

## Implementation Steps

- [x] Re-derive the envelope from extremes in `apply_pump_convention`.
- [x] Correct the stale on-disk `safety_limits.p2` in `settings.json` + `ME3B V3.json`.
- [x] Strengthen `test_restore_sign_and_datum` + add `test_envelope_restored_overrides_stale_mirror`.
- [x] Regression suites green.

## Testing Notes

- `tests/test_v75x_pump_plunger_setup.py` (25) green, incl. the new envelope assertions.
- pump-plunger / common-axis-speed-source / zp-envelope-absolute / jog-pump-fill
  suites green (62).
- **Needs real-HW verification on ME3B V3:** calibrate a pump, restart, confirm
  the Hardware Setup → Stage pump min/max rows AND the jog soft-limit clamp come
  up at the calibrated envelope (not a stale/sign-flipped value); confirm the
  plunger still stops at both mechanical extremes.

## Issues & Decisions

- **Why re-derive in `apply_pump_convention` rather than fix the save path?** The
  calibration already owns the datum + direction on reboot; the envelope it
  captured belongs to the same authoritative record. Deriving all three from
  `pump_setup` removes the stale-mirror failure mode entirely (self-heals any
  future drift), instead of chasing every writer that could desync the mirror.
- **Why also edit the JSON?** The code fix corrects the *live* value at boot, but
  `_apply_and_save_after_restore` can re-apply the persisted mirror mid-session;
  correcting the on-disk value makes every source agree so that path is safe.

## Addendum (2026-06-30) — the "safe because the data agrees" fix REGRESSED; fix the code path directly

**Operator report:** *"during the quick print it goes to pick up ink and nothing
happens."* Same root cause, resurfaced on ME3B V3.

**What was actually on disk on `Version-7.5.1`:** both `settings.json` **and**
`config/hardware/devices/ME3B V3.json` still had the stale `safety_limits.p2 =
[0, 30]` (the earlier data correction never landed on this branch), and the
`_apply_and_save_after_restore` re-clobber hole was still open in code.

**Decisive trace (`logs/app.log`, 16:24 boot → 17:20 run):**

```
16:24:41  Pump plunger convention restored: ['P2']          # apply_pump_convention → live p2 = [-30, 0] (correct)
16:24:55  calibration restore: applied limits + saved …      # _apply_and_save_after_restore → mirrors stale [0,30] back
16:25:01  ZP position restore: applied limits + saved …      # again
16:30:06  P2 clamped: -18.510 → 0.000 mm                     # live p2 now [0, …] → MIN=0
17:20:40  move_pump_uL(P2, -17.618 µL) → -2.11417 mm         # INK PICKUP aspirate
17:20:42  P2 clamped: -21.234 → 0.000 mm                     # dest clamped to MIN=0 …
                                                             # … then _shorten_only_delta zeroes the reversed delta → NO MOVE
```

The needle travelled to the ink well (the visible "goes to pick up ink"), then
the aspirate was clamped to the envelope MIN (`0`) and the shorten-only guard
(which forbids a clamp from reversing the move's sign) collapsed it to a
zero-length move → **the pump never actuated** ("nothing happens"). The prep
buffer aspirate was silently broken the same way one step earlier.

**Durable fix — calibration owns the pump envelope at the clobber site, not just
at boot.** `gui/app.py::_apply_and_save_after_restore` now, after mirroring the
persisted `safety_limits`, **re-derives each calibrated pump's envelope from
`controller.get_pump_setup()`** (`min/max` of the captured raw extremes — same
derivation as `apply_pump_convention`/`apply_pump_setup`) so the calibration
wins over the (possibly stale) mirror. Uncalibrated pumps keep the mirror. This
closes the re-clobber hole in *code* rather than relying on the on-disk data
staying in sync — the data-only approach demonstrably regressed. Also corrected
the stale `[0,30]` → `[-30,0]` in `settings.json` + `ME3B V3.json` again.

| File | Change |
|------|--------|
| `gui/app.py` | `_apply_and_save_after_restore` re-derives calibrated pump envelopes from `pump_setup` after mirroring persisted limits. |
| `settings.json`, `config/hardware/devices/ME3B V3.json` | `safety_limits.p2` `[0,30]` → `[-30,0]` (again — never landed on this branch). |
| `tests/test_v75x_last_known_calibration.py` | New `test_calibrated_pump_envelope_survives_stale_mirror` (+ `_fake` gains `get_pump_setup`/`saved_pump_limits`). |

- [x] Re-derive calibrated pump envelope in `_apply_and_save_after_restore`.
- [x] Re-correct the stale on-disk `safety_limits.p2`.
- [x] Regression test + suites green (last-known-calibration + pump-plunger 46; quick-print-pick&place + pump-settle + zp-envelope 88).
- **Needs real-HW verification on ME3B V3:** Quick Print with prep + ink pickup →
  buffer aspirate and ink aspirate both actuate the pump (no `P2 clamped → 0.000`
  in `logs/app.log`), print flows.

## Addendum 2 (2026-06-30) — DISPLAY the pump limits in the positive fill frame (like Z)

**Operator:** *"it sets the location correctly but flips the min and max with
negative max for the min. P2 should be min = 0, max = 30 (fill ~20). Last time I
loaded it set min = -30, max = 0. … we calibrate the pump by setting the zero and
full location; that sets raw + display + direction; the display should always be
positive; any conversion is done behind the scenes — I believe this is how the Z
axis works."*

**Cause:** the two clamp-side fixes above correctly made the stored/clamp
envelope **raw** (`[-30, 0]` on ME3B V3, since the motor counts DOWN toward full),
but the Hardware Setup → Stage limit **spinboxes displayed that raw value
directly** — so a calibrated pump showed `min = -30, max = 0`. Z already shows a
user "height" frame over a raw store (`_z_raw_to_user` / `_z_user_to_raw` with a
min/max-by-value swap); the pump spinboxes were never migrated (the position
readout above them already used `_pump_raw_to_user`, so only the limit rows were
inconsistent).

**Fix — pump limit spinboxes are now the user FILL frame (0 = empty → +capacity =
full, always positive), converting to/from the raw store behind the scenes,
exactly like Z.** The clamp, persistence, and `safety_limits.p*` stay **raw**
(untouched — no clamp code changed). New `StageHardwarePanel._pump_user_to_raw`
(inverse of `_pump_raw_to_user`; `raw = zero + aspirate_sign·user`). Applied at
every spinbox boundary, always **min/max by VALUE** (a negative `aspirate_sign`
swaps raw-min↔user-max, so the swap is essential to show `[0, +cap]` not
`[-cap, 0]`): `_load_from_settings` (raw→user), `_apply_safety_and_zero` +
`_apply` + the live-controller mirror (user→raw), `_record_limit` (⤓/⤒ Set
Min/Max stamp the fill value), `_pump_setup_persist` (post-calibration spinbox
update), `_reset_defaults`. `_pump_raw_to_user` normalises `-0.0 → 0.0` so the
empty datum shows a clean `0.000`. Uncalibrated pumps (no fill frame) pass through
identity → unchanged raw display + round-trip.

| File | Change |
|------|--------|
| `gui/pages/hardware/stage_panel.py` | Pump limit spinboxes display/edit the fill frame; new `_pump_user_to_raw`; raw↔user conversion (min/max by value) in load/save/mirror/`_record_limit`/`_pump_setup_persist`/`_reset_defaults`; `-0.0→0.0` normalise. Clamp + persistence stay raw. |
| `tests/test_v75x_pump_plunger_setup.py` | `test_limit_spinboxes_use_fill_frame_but_persist_raw` (calibrated: load raw [-30,0]→spinboxes [0,30]; save→raw [-30,0] in settings + live clamp), `test_uncalibrated_pump_limits_shown_and_stored_raw` (identity), updated `test_capture_flow_calibrates_and_persists` spinbox assertions [−35,0]→[0,35], `_FakeZP` gains `steps_per_mm`. |

- [x] `_pump_user_to_raw` + fill-frame conversion at all spinbox boundaries.
- [x] Tests (pump-plunger 27) + regression (last-known-cal / zp-envelope / z-display / cal-z-envelope / jog-pump-fill = 84) green.
- [x] End-to-end sim: settings raw [-30,0] → UI [0,30]; aspirate to raw −21 not clamped.
- **Needs real-HW verification on ME3B V3:** restart → Hardware Setup → Stage shows
  P2 `min 0 / max 30` (positive), the plunger still stops at both mechanical
  extremes, and ink pickup aspirates (no `P2 clamped → 0.000`).
