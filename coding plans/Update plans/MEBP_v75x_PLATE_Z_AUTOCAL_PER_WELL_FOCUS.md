# MEBP v7.5.x — Plate Z Auto-Cal: per-well guided refocus + needle best-focus

## Objective

Rework the **Plate Z Auto-Cal** tab so the per-well Z-bottom workflow matches how
the operator actually works on ME3B V1:

For **each** calibration well:
1. Retract Z and travel XY to the well (safe travel).
2. The operator **refocuses the microscope on the glass** (manual focus knob) and
   clicks **Confirm focus & lower needle**.
3. The needle **lowers** while the app tracks focus; as the needle becomes a
   sharper circle the **best-focus Z is auto-recorded** (the needle tip is then at
   the glass plane). The operator can override via **Record now**.
4. The operator **Accepts** (keeps the recorded Z and advances) or **Redo**es.

After all wells are accepted, the **Z-plane is fit** from the recorded points.

This replaces the previous "teach the first spot by hand, then auto-cal the
remaining wells from that single seed" design (`MEBP_v75x_Z_AUTOCAL_MANUAL_SEED.md`
/ `MEBP_v75x_PLATE_Z_AUTOCAL_TAB.md`), which did not let the operator refocus on
the glass per well and silently failed to find a focus peak.

## Design decisions (confirmed with the operator)

- **Refocus = manual microscope knob.** The Confirm step is a pure user gate; no
  axis moves during refocus.
- **Needle descent = auto-detect peak, with manual override** (`Record now`).
- **All calibration wells use the same flow** — no separate first-spot teach.

## Files Modified

- `gui/pages/calibration.py`
  - `_build_plate_z_autocal_tab` — replace Step 1 (first-spot teach) + Step 2
    (auto-remaining) with one guided per-well card: Start/Cancel + Confirm /
    Record now / Accept / Redo action buttons + status line + live feed.
  - `_start_auto_z_cal` — drop the seed/skip-first logic; initialize the per-well
    loop (`_auto_z_results={}`, `_auto_z_well_idx=0`, `_auto_z_last_z=None`).
  - `_auto_z_tick` — `navigate` now travels then **pauses** in `await_focus`
    (user-gated); `coarse`/`fine` descent record via `_auto_z_enter_recorded`.
  - New: `_auto_z_phase_buttons`, `_auto_z_enter_await_focus`,
    `_zauto_confirm_focus`, `_auto_z_enter_recorded`, `_zauto_record_now`,
    `_zauto_accept_well`, `_zauto_redo_well`.
  - Reference bottom for the safe descent window: previous accepted Z → Plate
    Bottom Z → `top_z − well_depth` (polarity-safe via `_auto_z_move_to_h`).
  - Removed: `_zauto_goto_first_well`, `_zauto_record_first_spot`, seed attrs
    (`_zauto_seed_z`, `_zauto_first_well`).
  - `_cancel_auto_z` / `_auto_z_finish` — reset the action buttons.

## Implementation Steps

- [x] Rebuild the Plate Z Auto-Cal tab UI (per-well action buttons).
- [x] Rewrite `_start_auto_z_cal` for the per-well loop.
- [x] Rewrite `_auto_z_tick` navigate→await_focus; record via helper.
- [x] Add confirm / record / accept / redo handlers + phase-button helper.
- [x] Remove obsolete first-spot seed methods/attrs.
- [x] Update `_cancel_auto_z` / `_auto_z_finish` button resets.
- [x] Tests.

## Testing Notes

- `tests/test_v75x_plate_z_autocal_per_well_focus.py` — drives the per-well state
  machine with duck-typed stubs (no Qt/camera/event loop): navigate→await_focus,
  confirm→descend window from prev/plate-bottom/estimate, auto-record on focus
  peak, manual `Record now` override, accept advances + fits plane on the last
  well, redo returns to await_focus.
- Updated `tests/test_v75x_plate_z_autocal_tab.py` widget-existence assertions to
  the new button names.
- Removed `tests/test_v75x_z_autocal_manual_seed.py` (the seed design is gone);
  the polarity helper `_auto_z_move_to_h` is re-covered in the new file.
- **Needs real-HW verification on ME3B V1.**

## Issues & Decisions

- The microscope focused on the glass means the whole-frame focus score includes
  the (sharp) glass texture; the needle adds edges that peak when its tip reaches
  the glass plane. Auto-detect tracks score above the captured baseline; the
  **Record now** override is the safety net when the peak is ambiguous.
- The descent window is bounded by `_auto_z_floor_h` (polarity-safe) so a poor
  reference estimate degrades to "no peak found → Redo / Record now", never a
  crash-down.
