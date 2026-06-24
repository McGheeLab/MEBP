# MEBP v7.5.x — Z-axis convention rethink

## DECISIONS (locked with user)

- **D1 — Datum = mechanical hard-bottom.** Z = 0 at the lowest the Z carriage can physically travel (below the plate). Always reachable for a repeatable zero. The plate bottom is then a small positive value.
- **D2 — Full unification.** One canonical converter for EVERY Z readout / spinbox / limit / calibration capture. Eliminates the 3-inconsistent-frames root cause.
- **D3 — Soft limits come from the SAME setup.** The Z setup procedure that sets the bottom datum also sets `z_min`/`z_max`, and **verifies the direction is correct** as part of that procedure.
- **D4 — Migration = the same setup.** Re-running the one setup re-establishes datum + direction + limits coherently. No separate auto-migration of the (already-incoherent) stored values.

### Resulting model — ONE setup procedure defines everything

The Z setup (Hardware Setup → Device) captures two physical extremes by jogging the real needle:
1. **Bottom** (needle all the way down): record `raw_bottom` → this becomes user **Z = 0** (`zero_position["Z"] = raw_bottom`).
2. **Top** (needle all the way up): record `raw_top`.

From those two readings the setup DERIVES, with no hardcoded polarity:
- **Up-direction** `z_up_sign = sign(raw_top − raw_bottom)` — guarantees user-Z increases as the needle rises, whatever the motor wiring. (Replaces the hardcoded module `ZDIR`; persisted per-machine in the device profile, module `ZDIR` becomes only the default.)
- **Canonical conversion**: `user_Z = z_up_sign · (raw − zero_position["Z"])`; inverse `raw = zero_position["Z"] + user_Z / z_up_sign`. At the bottom user_Z = 0; needle up ⇒ positive; always ≥ 0 in range.
- **Soft limits** (stored absolute raw, displayed in user frame): `z_min_raw = min(raw_bottom, raw_top)`, `z_max_raw = max(...)`; user-frame envelope = `[0, |raw_top − raw_bottom|]`.
- **Direction check**: after both captures, the procedure confirms jogging up increases user_Z (and that `|raw_top − raw_bottom|` is non-trivial); warns/blocks otherwise.

This satisfies D1–D4 simultaneously: datum, direction, and limits all come out of the one procedure, and re-running it is the migration.

### Implementation phases + status

- [x] **Phase 1 — StageController foundation.** Per-machine `_z_up_sign` (default = module `ZDIR`); canonical converters `raw_to_user_z` / `user_z_to_raw` / `zref_to_user_z` / `user_z_to_zref` + `z_up_sign()` getter (all via `getattr` fallback so `__new__` stubs + module-`ZDIR`-patching tests still pass); `z_height_of` / `default_travel_z` / `print_z_dir` fallback generalized to `z_up_sign()`; `apply_z_setup(raw_bottom, raw_top)` = single source of truth that sets the datum (`zero["Z"]=raw_bottom`), derives the up-sign, sets the raw soft-limit envelope, and reports `direction_ok`; `capture_current_z_raw()` for the UI. Tests: `tests/test_v75x_z_axis_unified_setup.py` (10) — all green; existing Z polarity/print tests still green (72 total in the Z group).
- [x] **Phase 2 — Persistence.** `DeviceProfile` + `settings.json` gained `z_up_sign`, `needle_cam_z`, `plate_z_offsets` (to_dict/from_dict/from_settings/apply_to_settings). `StageController.apply_z_convention()` restores them; wired at startup in `main.py` (after safety-limits load). Backend: `set_needle_cam_z_from_raw` / `get_needle_cam_z_user` / `set_plate_z_offsets` / `get_plate_z_offsets` / `estimate_plate_z_refs`. Tests in `test_v75x_z_axis_unified_setup.py` (12).
- [x] **Phase 3 — Z setup UI.** New "Z Axis Setup" group on Hardware Setup → Device: **Set Bottom (Z=0)** + **Set Top** capture raw extremes → `apply_z_setup`, refresh the Z limit spinboxes (user frame), show derived direction/travel/envelope, warn on `direction_ok=False`, persist (zero_position + safety_limits + `device_profile.z_up_sign` + profile JSON). Plus three editable **standard plate offset** spinboxes (top/bottom/safe) with Save.
- [x] **Phase 4 — GUI unification sweep.** Controller-aware converters added to `stage_panel` (`_z_raw_to_user`/`_z_user_to_raw`) + `control_panel` (`_z_raw_to_user`); routed: stage_panel readout / record-Z / limit load / limit save (both `_apply` paths) / reset-defaults; control_panel readout + Z bar range; `standard_jog_context` go-to-Z (`user_z_to_zref`) + reference labels (`zref_to_user_z`); `jog_control` side-view sign = `controller.z_up_sign()`; calibration `_zoff_user_z` display helper applied to all five reference labels + the top Z readout + the `_load_calibration` label restore. All keep a module-helper fallback when no controller.
- [x] **Phase 5 — Calibration + needle-cam + print.** `_needle_loc_center_and_save` captures the needle-cam Z fiducial (`set_needle_cam_z_from_raw`) + persists `device_profile.needle_cam_z`. New "Estimate plate Z from needle-cam" group on the Needle Offset tab pre-fills Plate Top/Bottom/Fast-Move Z guesses (`estimate_plate_z_refs`, editable, labelled "(guess)") + pushes the plate datum to the controller. Print path: `print_z_dir()` now falls back to `z_up_sign()` (not the module constant); motion stays raw/zero-ref, plate datum unchanged → no executor change needed.
- [x] **Phase 6 — Tests + full suite.** Updated `test_v75x_z_display_numbering` (record-limit Z now via the unified converter) + `test_v75x_cal_z_envelope_no_clobber` (stub gained `_zoff_user_z`). 213 Z/calibration/jog/envelope tests green; offscreen build smoke of the Device + control panels passes. Full suite: 819 tests, the only failures are the **13 pre-existing** ones in `test_v726_print_execution` / `test_v73_trajectory_planner` (MagicMock `<` float in `PrintTrajectoryPlanner._move_z`) + `test_sim_vs_hardware` flakes — confirmed unrelated (fail identically with this change reverted).

### Real-hardware follow-up (must verify on the bench)
1. Hardware Setup → Device → **Z Axis Setup**: jog needle fully DOWN → Set Bottom; jog fully UP → Set Top. Confirm the status shows the correct direction + travel, the Z readout reads **0 at the bottom and positive going up everywhere** (control panel, jog, calibration), and jogging never clamps.
2. Set the three plate offsets for the real machine.
3. Run XY needle calibration → confirm the needle-cam Z is captured → **Estimate plate Z** pre-fills sensible Plate Top/Bottom/Fast-Move guesses → refine with the manual first-spot / auto-Z.
4. Confirm a print descends toward the plate and the print-floor guard still holds.

---

# (original analysis follows)


## The ask

Redefine the user-facing Z convention so that:
- **Z = 0 when the needle is all the way down** (at the plate/well bottom).
- **Z increases as the needle moves up** (away from the plate).
- Working values are therefore **always positive**; closer to the plate bottom = smaller.
- **Setup must establish this** (a deliberate "this is the bottom → 0" step).

Then go through the whole system, find everywhere this breaks, and remedy it.

---

## 1. Current Z frame model (as built)

There are **five** Z representations in play today:

| Frame | Definition | Used by |
|-------|-----------|---------|
| **RAW Marlin mm** | physical Marlin counter. ME3B V1: needle **descends as raw ↑** (`steps_per_mm.Z = +5255`) | `ZPStage`, position poller (`get_zp_position` returns RAW) |
| **Zero-ref mm** | `raw − zero_position["Z"]` | `move_z_absolute(from_zero_ref=True)`, `move_z_relative`, all calibration refs, print planner inputs |
| **Height mm** | `ZDIR · (zero-ref)` (ZDIR = −1 ⇒ up = +) | jog reference labels, `XZSideView`, the recently-added `z_height_of` / `needle_at_or_above` / `ensure_retracted_to` |
| **Plate-relative mm** | height above the plate bottom (`plate_relative_to_zref` / `zref_to_plate_relative`, sign = `print_z_dir()`) | print planner Z, print floor, `print_height_to_zref` |
| **Microsteps** | steps_per_mm conversion | inside `ZPStage` only |

Safety envelope: `SafetyLimits.z_min/z_max` are **absolute RAW mm** (deliberately Set-Zero-independent since the v7.5.x "envelope absolute" fixes).

Helper inventory (all in `StageController`): `ZDIR`, `z_raw_to_display`, `z_display_to_raw`, `z_height_of`, `default_travel_z`, `needle_at_or_above`, `ensure_retracted_to`, `plate_relative_to_zref`, `zref_to_plate_relative`, `print_z_dir`, `print_height_to_zref`, `zref_to_print_height`, `print_floor_violation`, `_apply_print_floor_raw`, `set_plate_bottom_z`/`set_plate_top_z`, `set_min_travel_z`.

## 2. Evidence the current state is broken (the "real issue")

**(a) The display frame is applied three inconsistent ways to the same physical Z:**
- `control_panel._update_position_displays` → `z_raw_to_display(raw)` = **−raw** (ignores `zero_position`). [control_panel.py:896](gui/pages/hardware/control_panel.py#L896)
- `standard_jog_context` reference labels → `z_raw_to_display(zero_ref)` = **−(raw−zero)** (height above software zero). [standard_jog_context.py:383](gui/widgets/standard_jog_context.py#L383)
- `calibration._zoff_capture_current_z` → **raw − zero** (zero-ref, *no* sign flip → opposite sign from jog). [calibration.py:1808](gui/pages/calibration.py#L1808)

When `zero_position["Z"] ≠ 0` all three diverge; even at `zero=0`, calibration captures are sign-flipped vs the jog/readout.

**(b) The recorded Z references are physically incoherent** (live `settings.json`, `zero_position.Z = 0`, so zero-ref == raw; height = −raw):

| Reference | stored (raw/zero-ref) | height (−raw) | should be |
|-----------|----:|----:|-----------|
| `replace_z` | −16.91 | +16.91 | highest (needle swap) |
| `max_z` | −17.71 | +17.71 | high |
| `safe_z` (fast move) | −18.35 | +18.35 | travel height |
| `top_z` (plate top) | −23.20 | +23.20 | above bottom |
| `plate_bottom_z` | −43.25 | **+43.25** | **lowest** |

The plate **bottom** reads as the **highest** point (+43.25 > travel +18.35). Either the captures were taken in a confused frame, or the envelope datum is wrong. Under any single coherent convention these must be monotonic. This is what makes Quick-Print/auto-Z/jog behave unpredictably.

**(c) `z_raw_to_display` is overloaded** — fed RAW in one place and zero-ref in another, so its single definition cannot be right for both.

## 3. Target convention (recommended)

**One canonical user-facing frame: "needle height above the down-datum."**

```
user_Z = ZDIR · (raw − zero_position["Z"])         # display / spinboxes / limits
raw    = zero_position["Z"] + ZDIR · user_Z         # inverse (parse user input)
```

with the **down-datum (`zero_position["Z"]`) established at the plate bottom during setup.** Then:
- `user_Z == height above the plate bottom` (the existing **print** frame) → display, print, and calibration all collapse into ONE positive-up frame.
- needle at the bottom ⇒ `user_Z = 0`; needle up ⇒ positive; always ≥ 0 in the working range.
- The **manual first-spot Z teach we just added** (`_zauto_record_first_spot`) *is* the datum-setting action.

Internals unchanged: motion stays raw/zero-ref; the safety envelope stays absolute raw (Set-Zero-independent). Only the **boundary conversion** is unified and the **datum is anchored at the bottom**.

## 4. Gap analysis — everywhere that must change / be verified

**A. Single conversion boundary (new).** Add one pair of canonical converters (`user_z_from_raw` / `raw_from_user_z` and a zero-ref variant) and route EVERY readout/input through them. Today the conversion is duplicated and divergent.

**B. GUI readouts (make consistent):**
- `control_panel._update_position_displays` — currently `−raw`; must subtract zero first. Position bars (`bar_pos["Z"]`, ranges at control_panel.py:612-615) likewise.
- `standard_jog_context` Z reference labels + Absolute Go-To Z parse (standard_jog_context.py:219, 383).
- `XZSideView` (`set_z_display_sign`, `set_zero_offset_z`, Z scale/scrollbar) — already height-aware; confirm datum.
- `stage_panel` Z readout + soft-limit spinboxes (stage_panel.py:2007, 2138, 2169-2174, 2462-2466, 2610-2614, 2683-2684) — the min/max SWAP logic.
- `jog_control` Z readout + `set_z_display_sign` wiring (jog_control.py:172).

**C. Calibration page:**
- `_zoff_capture_current_z` (returns zero-ref, no sign) → return the canonical user_Z.
- The five reference setters (replace/max/fast-move/plate-top/plate-bottom) + their labels — confirm they store one frame and display the user frame.
- Auto-Z + manual first-spot (`_auto_z_*`, `_zauto_record_first_spot`) — already height/seed based; re-point at the datum.
- `_save_calibration` / `_load_calibration` Z fields, and `CalibrationSnapshotStore`.

**D. Setup workflow (new/!):** a deliberate "**Set Z = 0 at the plate bottom**" step (tie to the manual first-spot teach). This sets `zero_position["Z"] = raw_at_bottom` and makes plate_bottom_z = 0.

**E. Safety envelope:** keep `z_min/z_max` absolute raw internally; display/edit in the user frame (positive-up). The current swap logic stays but the displayed numbers become "height above datum."

**F. Print path:** already polarity-general (`print_z_dir`, plate-bottom datum). If the datum == plate bottom, `print_height_to_zref(h)` and `user_Z` coincide → simplification, low risk. **Verify** discrete/hybrid executors read the right frame.

**G. Persistence + migration (HIGH RISK):**
- `settings.json` `calibration.*` Z refs + `zero_position.Z` + `safety_limits.z_min/z_max`.
- `config/hardware/devices/*.json` envelope (`z_min/z_max`), `steps_per_mm.Z` (sign), `axis_flip.z`.
- `config/prints/*.json` — **must confirm** the trajectory Z column / print heights frame (height-above-bottom vs zero-ref). If datum-relative, prints are unaffected by moving the datum; if absolute zero-ref, they need migration.
- Moving the datum shifts every stored zero-ref value → either re-cal at setup or a one-time reinterpretation.

**H. Tests to update:** `test_v75x_z_display_numbering`, `test_v75x_cal_z_envelope_no_clobber`, `test_v75x_zp_envelope_absolute`, `test_v75x_print_z_plate_bottom`, `test_v75x_print_z_reference_vector`, `test_v75x_z_autocal_manual_seed`, `test_v75x_quick_print_workflow`, `test_v731_*`.

## 5. Risks
- **Safety-critical**: a sign error in the boundary conversion drives the needle into the plate. Every change must keep motion in raw and only convert at the edges; the `ensure_retracted_to` / print-floor guards must keep working.
- **Datum reachability**: if 0 = plate bottom, the needle physically stops at the plate (can't go below 0). If 0 = mechanical hard-bottom, 0 is reachable but plate bottom is a small positive.
- **Migration**: stale stored values are already incoherent (§2b); a clean re-cal at setup is probably safer than reinterpreting them.
- **steps_per_mm sign flip is OFF the table** — history (`MEBP_v75x_Z_DISPLAY_NUMBERING`) shows flipping the motor sign also reversed the hard-wired jog buttons; that's why ZDIR is display-only. Keep it that way.

## 6. Recommended approach (phased) — pending decisions
1. Add the single canonical converter pair + a `set_z_zero_at_bottom()` setup action; make the datum = plate bottom.
2. Route every GUI readout/input/limit through the converter (consistency sweep, §B).
3. Re-point calibration captures + references + auto-Z at the datum; persist in one frame.
4. Confirm print path coincides; verify executors.
5. Migration/setup: re-establish the datum + re-teach references during a one-time setup; verify print-file frame.
6. Update tests.

## 7. Open decisions (need user input)
- **D1 — Datum**: Z=0 at the **plate/well bottom** (recommended; unifies with print frame, reuses the manual first-spot cal) vs the **mechanical hard-bottom** of travel vs a **fixed firmware home**.
- **D2 — Scope**: full unification (one frame everywhere, fixes the inconsistencies) vs display-only minimal.
- **D3 — Safety envelope**: keep stored absolute-raw + display in user frame (recommended) vs store the envelope in the user frame.
- **D4 — Migration**: re-cal at setup (recommended) vs one-time reinterpretation of existing values.
