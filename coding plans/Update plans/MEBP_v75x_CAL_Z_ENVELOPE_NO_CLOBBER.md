# MEBP v7.5.x — Calibration must not clobber the Z safety envelope

## Objective

Fix the bug where Xbox triggers mapped to `move_z_at_velocity` (and every
other Z jog) were clamped to a single point (~48.4 mm) regardless of the
device-setup Z range, logged as:

```
[W] SafetyLimits: Z clamped: 48.280 → 48.400 mm
[W] SafetyLimits: Z clamped: 48.509 → 48.400 mm
```

Both directions clamp to the *same* value (48.400) — the diagnostic for a
**collapsed / inverted** envelope (`z_min == z_max`, or `z_min > z_max`).

## Root Cause

The calibration page wrote its captured Z reference heights into the live
`SafetyLimits` envelope:

| Calibration value | Setting (this machine) | Pushed to | via |
|---|---|---|---|
| `max_z` ("Max Z") | **16.4** | `z_max` (`as_max=True`) | `_zoff_set_max_z`, `_load_calibration` |
| `plate_bottom_z` ("Plate Bottom Z") | **48.4** | `z_min` (`as_max=False`) | `_zoff_set_plate_bottom_z`, `_load_calibration` |

The author assumed `Max Z` (ceiling) is numerically **above** `Plate Bottom
Z` (floor). But on this machine the needle **descends as Z increases**
(`safe_z 26.4 → plate top 28.6 → plate bottom 48.4`), so "Max Z" (highest
*physical* position = max retraction) is the *smallest* coordinate (16.4)
and "Plate Bottom Z" is the *largest* (48.4). The push therefore produced
`z_min = 48.4 > z_max = 16.4`, and `clamp_z(z) = max(48.4, min(16.4, z))`
collapses to **48.4 for every input**.

Secondary defect: `_load_calibration` re-applied this on **every startup**,
silently overriding the user's Hardware Setup → Device Z range
(`safety_limits.z_min/z_max`, e.g. 10..60 in `settings.json`) — so even a
correctly-ordered calibration would have clobbered the device-setup range.

## Decision

Per user (Device tab is where soft limits belong): **the Z soft-limit
envelope is owned exclusively by Hardware Setup → Device.** Calibration's
`Max Z` / `Plate Bottom Z` remain captured as **print/Z references**
(`_max_z` / `_plate_bottom_z`) but no longer touch `safety_limits`.

(Considered & rejected: re-ordering the push to `[min, max]` so calibration
keeps governing — would still override the user's device-setup range.)

## Files Modified

| File | Change |
|---|---|
| `gui/pages/calibration.py` | `_zoff_set_max_z` / `_zoff_set_plate_bottom_z`: drop the `safety_limits.set_z_from_current(...)` push (still capture the reference height + update label + emit). `_load_calibration`: remove the v7.4.4 envelope-restore block (lines ~6311-6322) that pushed `_max_z`/`_plate_bottom_z` into the live envelope on startup. Docstrings/comments updated to explain the inversion bug. |
| `tests/test_v75x_cal_z_envelope_no_clobber.py` | New — 4 tests (unbound-method + fake-self; no QApplication). |

## Implementation Steps

- [x] Identify clamp source — `SafetyLimits.clamp_z`, fed by `ZPJogHandler._jog_loop` `_clamp_delta`
- [x] Trace 48.4 to `settings.json::calibration.plate_bottom_z`; `max_z = 16.4`
- [x] Confirm `_load_calibration` + the two `_zoff_set_*` setters are the only sites pushing into the envelope (`set_z_from_current` grep)
- [x] Confirm device-setup Apply (`stage_panel`) + startup (`main.py:236`) write the same live `sl` — so removing the override makes Device authoritative
- [x] Remove the push from `_zoff_set_max_z`
- [x] Remove the push from `_zoff_set_plate_bottom_z`
- [x] Remove the startup restore from `_load_calibration` (drop the now-unused local `sl`)
- [x] Add regression test
- [x] Verify existing calibration / envelope tests still pass

## Testing Notes

- `python -m unittest tests.test_v75x_cal_z_envelope_no_clobber` — 4 pass.
- Regression suite: `test_v744_calibration_revision`,
  `test_v74x_objective_calibration`, `test_v75x_zp_envelope_absolute`,
  `test_v75x_axis_map_jog` — 61 pass.
- **In-session recovery** (the live `sl` is still inverted until reloaded):
  open Hardware Setup → Device and click **Apply Settings** (re-pushes
  `z_min/z_max` from the spinboxes to the live envelope), or restart (startup
  now loads `settings.json` 10..60 and calibration no longer overrides it).
- **Real-HW check**: with triggers on `move_z_at_velocity`, Z should now jog
  freely across the device-setup range; no "Z clamped → 48.400" spam.

## Issues & Decisions

- The captured `max_z`/`plate_bottom_z` are *physical-height* labels, not
  *coordinate* extremes; conflating them with `z_max`/`z_min` is only correct
  on Z-up-positive machines. Keeping them out of the envelope avoids the
  polarity assumption entirely.
- `_load_calibration` is too heavyweight to instantiate headless, so its
  regression is guarded by a source-level assertion that `calibration.py`
  contains no `set_z_from_current(` call (comments stripped before matching).
