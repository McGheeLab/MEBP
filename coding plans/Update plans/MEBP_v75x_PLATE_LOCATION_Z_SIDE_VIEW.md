# MEBP v7.5.x — Z side view on the Plate Location tab

## Objective

On **Calibration → Plate Location**, add a **Z position side view** next to the
well-layout map so the operator can **move the needle up to safe travel
manually** while teaching wells (e.g. after jogging Z down to focus on a rim,
retract before continuing). This reuses the existing `XZSideView` widget (the
same one on the Jog page): its green **"Safe"** badge drives Z to the Fast Move
height, and the other captured-Z badges are clickable too.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | Build `_ploc_xz_view` (an `XZSideView`) in `_build_plate_location_tab`; place it in a horizontal splitter beside `_ploc_plate_view` (top of the left column, above the live microscope feed). Wire `go_to_z_requested → _on_ploc_go_to_z` (→ `move_z_absolute(z, from_zero_ref=True)`). Feed live Z + zero offsets in `on_status_update`; push safety limits / needle / Z references / display sign in `_refresh_ploc_view`. |
| `tests/test_v75x_plate_location_z_side_view.py` | New — 6 tests (presence, display-sign, go-to-Z routing, disconnected no-op, reference propagation, status tick). |

## Implementation Steps

- [x] Construct `_ploc_xz_view` in `_build_plate_location_tab`, configured from
  the controller (`z_up_sign` → display sign), safety limits, needle OD, and the
  current `get_z_references()`.
- [x] Lay out the well map + side view side-by-side in a horizontal `QSplitter`
  (stretch 3:1), nested as the top pane of the existing vertical left split
  (map+side-view over the live microscope feed). Added a one-line hint.
- [x] `_on_ploc_go_to_z(z_mm)` — manual, operator-initiated Z move via
  `move_z_absolute(z_mm, from_zero_ref=True)` (soft limits respected; no-op when
  ZP disconnected). Mirrors `jog_control._on_go_to_z_requested`.
- [x] `on_status_update` pushes live needle Z (logical, zero-ref) + the X/Z zero
  offsets so the view renders in the zero-ref frame (mirrors the Jog page).
- [x] `_refresh_ploc_view` keeps the view's envelope / needle / display-sign /
  Z references in sync (so the Safe badge updates the moment Fast Move Z is set
  on the Needle Offset tab — `_set_safe_z` already triggers
  `_emit_calibration_data_changed → _refresh_ploc_view`).

## Testing Notes

- `python -m unittest tests.test_v75x_plate_location_z_side_view` → 6 green.
- Regression: `tests.test_v731_jog_navigation`, `tests.test_v75x_plate_centering`,
  `tests.test_v75x_z_retract_before_xy_travel`,
  `tests.test_v75x_last_known_calibration` → 74 green (real-page builds).
- `tests.test_v75x_plate_location_manual_click_rim` → 27 green.

## Issues & Decisions

- **Reused `XZSideView` rather than a new widget** — it already renders the
  needle/plate/Z-references to scale and emits `go_to_z_requested` in the raw
  zero-ref move frame. The badge move uses the standard `move_z_absolute`, so
  the Z soft-limit envelope and unified Z convention apply unchanged.
- **Safety:** clicking **Safe** is a pure *up*-Z move (retract) — exactly the
  requested action. Other badges (Plate ↓ etc.) can lower the needle, identical
  to the Jog page; this is the manual-jog exemption (operator-controlled Z), so
  no auto-retract gate is involved. The move is display-sign-agnostic (the emit
  carries the raw zero-ref value; only the picture is flipped via
  `set_z_display_sign`).
- **Layout:** kept the live microscope feed below the map+side-view row so the
  manual click-rim workflow is unaffected; the side view is the narrower pane.

**Status: complete (needs a quick real-HW glance on ME3B V1 to confirm the
badge retract feels right; logic mirrors the verified Jog-page path).**
