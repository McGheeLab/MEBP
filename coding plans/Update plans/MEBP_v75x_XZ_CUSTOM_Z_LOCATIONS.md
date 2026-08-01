# MEBP v7.5.x — Custom Z Locations on the XZ Side View (Jog)

## Objective

New jog feature (operator request): on the Z-axis overview (the `XZSideView`
"Side View (XZ)" card on the Jog page), let the operator tag **custom Z
locations**:

1. **Click + drag** anywhere on the Z axis plot → a floating readout tag pops
   up showing the location in mm (live, in the user display frame). On
   **release**, that Z is set as a custom location (a new badge on the view).
2. **Click the current-location line** (the red "current Z" line) → the
   needle's current Z is set as a custom location.
3. Each custom location renders like the calibration Z references: a dashed
   line + a clickable badge (`★ +12.34 mm`). Clicking the badge drives Z there
   (same `go_to_z_requested` → `move_z_absolute(from_zero_ref=True)` contract
   as the existing Replace/Max/Safe/Plate badges). A small `✕` button beside
   the badge (or right-click on the badge) removes the location.
4. Custom locations persist per machine in `settings.json`
   (`jog.custom_z_mm`, raw zero-ref mm) and are restored on startup.

## Design decisions

- **Opt-in at the widget level** — `XZSideView.set_custom_z_enabled(True)` is
  called ONLY by the Jog page. The other four hosts (Calibration Needle-Offset
  + Plate-Location tabs, spheroid/cell workflows) are byte-identical: with the
  flag off, mouse press/drag behaviour is unchanged.
- **Frames:** custom locations are stored in the **raw zero-ref mm** frame
  (the same frame as `_z_refs` and the `go_to_z_requested` emit, so the move
  contract and soft limits hold on both Z polarities). All *display* (drag
  readout, badge text) goes through the existing `_disp()` sign mapping, so on
  ME3B V1 (`z_up_sign = -1`) the operator reads heights (up = +). Conversion
  back: `raw = disp × sign` (sign ∈ {±1}).
- **Gesture disambiguation:** a press inside the plot starts a *pending* drag;
  it becomes a live drag only after moving > ~4 px. A plain click (no drag)
  adds the current Z **only when the press was within ~6 px of the current-Z
  line** — so stray clicks in the plot do nothing.
- **Dedup:** adds within 0.005 mm of an existing custom location are ignored.
- **No new motion math** — go-to reuses the existing badge → `go_to_z_requested`
  path; drags never move the stage.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/xz_side_view.py` | Custom-Z state + API (`set_custom_z_enabled` / `set_custom_z_locations` / `custom_z_locations` / `add_custom_z` / `remove_custom_z`), `custom_z_changed(list)` signal, drag-to-tag mouse handling (`mousePressEvent`/`mouseMoveEvent`/`mouseReleaseEvent`), `_px_to_z_disp` inverse mapping, custom badges + ✕ painted in `_paint_z_references` (shared collision-nudge), floating drag readout `_paint_drag_ghost` |
| `gui/pages/jog_control.py` | Enable the feature on the Jog XZ view; persist via `custom_z_changed` → `settings.set("jog.custom_z_mm", …)` + `save()`; restore in `set_settings` |
| `tests/test_v75x_xz_custom_z_locations.py` | New suite (widget gestures, frames/sign, badges, persistence, off-by-default) |

## Implementation Steps

- [x] `XZSideView`: state, signal, public API (programmatic set does NOT emit
      `custom_z_changed` — no save loop)
- [x] Inverse pixel→Z mapping `_px_to_z_disp` (uses the zoomed `_z_bounds_mm`,
      so dragging while zoomed is exact)
- [x] Paint: custom dashed lines + `★` badges + `✕` remove buttons, sharing the
      Z-ref badge collision/nudge/leader machinery; drag ghost line + mm tag
- [x] Mouse: badge/✕ hits take precedence; drag-to-tag; click-current-line;
      cursor feedback (cross in plot when enabled, pointing hand on badges)
- [x] Jog page: enable + persist/restore (`jog.custom_z_mm`)
- [x] Tests
- [x] CLAUDE.md table entry

## Testing Notes

`tests/test_v75x_xz_custom_z_locations.py` (offscreen QApplication, synthesized
`QMouseEvent`s; badge hit rects populated via `widget.grab()`):

- drag on the plot → readout state live during drag → release adds the custom
  location at the dragged Z (round-trip through the widget's own `_z_to_px`)
- display sign −1: dragging at display height −20 stores raw 20 (ME3B V1)
- plain click ON the current-Z line adds the current Z; a click elsewhere in
  the plot adds nothing
- clicking a custom badge emits `go_to_z_requested` with the raw zero-ref
  value; `✕` (and right-click) removes + emits `custom_z_changed`
- dedup within 0.005 mm; programmatic `set_custom_z_locations` does not emit
- **off by default** — a drag on a plain `XZSideView` adds nothing (the other
  four host pages are unchanged)
- Jog page integration: feature enabled, `set_settings` restores
  `jog.custom_z_mm`, an add persists the new list + calls `save()`
- zoom interplay: drag while zoomed maps through the zoomed bounds

Manual (needs GUI verification on ME3B V1): drag on the Jog side view → tag
follows cursor with mm readout → release leaves a ★ badge; click the red
current-Z line → badge at current height; badge click drives Z there (soft
limits respected); ✕ removes; restart restores badges.

## Issues & Decisions

- The ✕ affordance is drawn to the LEFT of each ★ badge so the badge itself
  stays right-anchored like the Z-ref badges; right-click on the badge is an
  equivalent remove path.
- The drag readout shows 3 decimals (µm-ish resolution) while badges show 2,
  matching the Z-ref badge format.
- Custom badges are painted AFTER the Z-ref badges in the same pass so the
  existing vertical collision-nudge (`placed`) keeps all labels readable.
- Persistence uses the raw zero-ref frame; a re-zero of Z shifts the meaning
  of stored locations exactly like it shifts the calibration Z references —
  consistent with every other Z the app stores.
