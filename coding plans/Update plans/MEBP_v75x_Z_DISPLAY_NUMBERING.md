# MEBP v7.5.x — Z display numbering (height, up = +)

## Objective

On ME3B V1 the Z motor is calibrated so the **needle physically descends as the
raw Marlin Z counter increases** (`steps_per_mm["Z"] = +5255`). The app numbered
Z by the raw value, so:

- "Z UP" already raised the needle correctly (raw Z *decreases*) — **motion was
  never wrong**.
- The displayed value *was* raw Z, so going up the number went **negative** →
  read −60 at the top.
- Recording the top into "Max" stored `z_max = -60` with `z_min = 0` → an
  **inverted envelope** (`z_min > z_max`); `SafetyLimits.clamp_z` then collapsed
  every Z move to a single point → **Z froze**.

v7.4.2 had removed the separate axis-flip UI, tying direction solely to the
`steps_per_mm` sign — so inverting the motor also reversed the hard-wired jog
buttons, leaving no way to decouple "which way the motor turns" from "how Z is
numbered."

**This update numbers Z as a *height* (up = +) at the human boundary only.** A
fixed sign `ZDIR = -1` so **user-facing Z = ZDIR × raw**. Motor, motion, print,
and calibration frames are unchanged (no re-calibration). Scope = the whole
**Jog + Device pages** (readouts, Min/Max limits, side-view, Absolute Go-To Z).
Calibration / Print / workflow pages keep their existing plate-relative frame.

Result: Z UP raises the needle **and** the value (0 at bottom → +60 at top);
Min = 0 / Max = 60 record correctly and the envelope can never invert/freeze.

## Approach

Multiply by `ZDIR` wherever a Z value is **shown** on Jog/Device pages; divide
by `ZDIR` wherever a Z is **typed/clicked** and sent to motion. Internal frames
(raw Marlin, zero-ref, `SafetyLimits.z_min/z_max`, print, calibration) are
unchanged. Because `ZDIR = -1` reverses ordering, converting between the height
limit spinboxes and the raw-stored limits uses a **min/max SWAP** — this is what
permanently prevents the inverted/frozen envelope.

The dormant `_axis_flip["Z"]` / `_flip_sign` (applied *inside* motion) stays
**unused** — repurposing it would double-apply against the motor.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | New module-level `ZDIR = -1.0` + helpers `z_raw_to_display` / `z_display_to_raw` (the single source of truth). |
| `gui/pages/hardware/control_panel.py` | Z position readout → `z_raw_to_display`; Z PositionBar range converted + swapped. (X/Y/pumps untouched.) |
| `gui/pages/hardware/stage_panel.py` | Z limit-row + live-position readouts → height; `_record_limit` Z → height; `_load_from_settings` / both apply paths / reset-defaults convert + **swap** between height spinboxes and raw-stored limits. Override-card readout left zero-ref (out of scope). |
| `gui/widgets/xz_side_view.py` | New `set_z_display_sign(sign)` + `_disp()`; sign applied in `_z_safety_bounds` (swap), the needle/reference `_z_to_px` feeds, badge text, and the bottom readout (Z / Δplate / Δwell / safe-Z). `go_to_z_requested` emit stays raw zero-ref (hit rects keep the raw value). Default `+1` = byte-identical (spheroid's view unaffected). |
| `gui/pages/jog_control.py` | `self._xz_view.set_z_display_sign(ZDIR)` once at construction. |
| `gui/widgets/standard_jog_context.py` | Absolute Go-To Z input converted (`z_display_to_raw`) before every Z motion call + tooltip; Hardware-Info reference labels shown via `z_raw_to_display`. |
| `config/hardware/devices/ME3B V1.json`, `settings.json` | Existing inverted/frozen envelope `z_min:0 / z_max:-60` corrected to the non-inverted equivalent `z_min:-60 / z_max:0` (same physical range; what the new Apply logic now writes). |
| `tests/test_v75x_z_display_numbering.py` | New (13 tests). |

## Implementation Steps

- [x] `ZDIR` + helpers in `StageController.py`.
- [x] Numeric Z readouts (control_panel display + Z bar range; stage_panel labels).
- [x] Device-setup Z Min/Max: record / load / apply ×2 / reset, all with swap.
- [x] `XZSideView.set_z_display_sign` + display chokepoints; emit stays raw.
- [x] Wire `set_z_display_sign(ZDIR)` + sign reference labels (jog_control / context).
- [x] Convert Absolute Go-To Z input + relabel.
- [x] Correct the inverted stored envelope in `ME3B V1.json` + `settings.json`.
- [x] Tests + suite green.

## Testing Notes

`python -m unittest tests.test_v75x_z_display_numbering -v` → 13 pass. Covers:
helper signs + round-trip; limit apply/load swap + round-trip; the anti-freeze
regression (record top→Max / bottom→Min → non-inverted, `clamp_z` passes
mid-range); the **real** `StageHardwarePanel._record_limit` Z branch via a
stand-in ("Set Max" at the top records +60); and motion-frame-untouched
(`move_z_absolute`/`move_z_relative` send raw, no ZDIR; `_flip_sign("Z") == 1.0`).

Regression: `tests.test_v75x_zp_envelope_absolute`, `..._cal_z_envelope_no_clobber`,
`..._axis_map_jog`, `test_v731_jog_navigation`, `test_v731_integration`,
`test_v75x_plate_centering`, `test_v75x_quick_print_workflow` all green.

Manual on hardware: Device → Jog & Setup Limits — jog to bottom, Set Min (≈0);
jog to top, Set Max (≈+60); Apply → `ME3B V1.json` saves `z_min ≈ -60`,
`z_max ≈ 0` (non-inverted). Jog Z UP → needle rises **and** value rises toward
+60; Z DOWN → drops toward 0; no clamp-to-0 freeze. Jog-page readout, side-view
picture/text, and a positive Go-To Z all agree (positive = up).

## Issues & Decisions

- **Numbering vs motor (user choice):** flip numbering only — keep motor/motion;
  no re-calibration. The conflict was that v7.4.2 conflated motor direction with
  value numbering; this update separates them via a display-only sign.
- **ZDIR fixed, not a toggle (user choice):** applied unconditionally for this
  machine rather than a per-profile checkbox.
- **Scope = whole Jog + Device pages (user choice):** includes the Absolute
  Go-To Z input (converted) so a typed positive value can never drive the needle
  down. Calibration/Print/workflow Z left in their plate-relative frame.
- **Side view stays self-contained:** the sign is applied inside the widget and
  the `go_to_z_requested` emit keeps the raw zero-ref value, so the move contract
  and other consumers (spheroid, default `+1`) are unchanged.
- **Corrected existing config:** the stored envelope was the inverted/frozen
  state from the bug; rewritten to the equivalent non-inverted range so Z works
  on next launch without a forced re-record.
