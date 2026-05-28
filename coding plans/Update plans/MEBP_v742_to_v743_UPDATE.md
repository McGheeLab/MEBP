# MEBP v7.4.2 → v7.4.3 Update Plan

## Objective

Two coordinated changes:

1. **Jog Control page becomes a visualization-first cockpit.** The main
   content area shows only live state — top-down XY workspace + XZ side
   view + pump rack — and the left context panel is the project's new
   reusable `StandardJogContextPanel`. Click-to-travel on the workspace
   canvas (Free / Snap-to-wells toggle) is the only "control" the main
   area exposes.
2. **`StandardJogContextPanel` becomes the canonical jog context** for
   any jog-aware page. It wraps the existing `HardwareControlPanel`
   (jog UI proven out on Calibration, safety limits engaged) and adds
   an **Absolute Go To** section plus a read-only **Hardware Info**
   section at the bottom (needle gauge / OD / length, syringe sizes per
   configured pump, well-plate format, calibrated-well count, safe Z).
   Both the Calibration page and the Jog page now use it, so users
   learn one jog UI.

Side effects:

- **Calibration moves above Jog Control** in the sidebar. Both pages
  now have the same left panel, so the earlier-in-list ordering reads
  as the natural workflow (calibrate first, then drive).
- **Pre-existing index bug fixed.** `app.py` was reading
  `pages[2]` / `pages[3]` for Jog / Calibration which were actually
  Calibration / Printing — calibration data and the startup plate never
  reached the Jog page. Now realigned to `cal_page = pages[1]`,
  `jog_page = pages[2]`.

Branch: `Version-7.4.3` (branched from `Version-7.4.2`).

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/widgets/standard_jog_context.py` | NEW. Reusable jog context panel: wraps `HardwareControlPanel(show_connect=False, bypass_safety=False)` + adds Absolute Go To + Hardware Info quick-reference. Forwards `set_controller`, `set_settings`, `set_hardware_config`, `set_calibration_data`, `on_status_update`. |
| `gui/widgets/jog_workspace_view.py` | NEW. Top-down XY canvas with Free/Snap toggle: safety envelope (red edges go solid when within 500 µm), plate outline, wells colored by status, fading breadcrumb trail (last ~30 positions), needle at scale with crosshair, hover-ghost target with live µm readout. |
| `gui/widgets/xz_side_view.py` | NEW. Side-view canvas: needle drawn to real length + OD, plate cross-section, safe-Z dashed line, well cavity under needle when over a calibrated well, right-edge annotations (Current Z, Δ to plate, Safe Z, Δ to well bottom). |
| `gui/pages/jog_control.py` | Rewritten. Main area = pure visualization (workspace + XZ + pump rack). Left context panel = `StandardJogContextPanel`. Click-to-travel routed through `safe_travel_to` when current Z < safe Z, direct `move_xy_absolute` otherwise. Esc → STOP shortcut, arrow keys / PgUp / PgDn preserved with sensible defaults. |
| `gui/pages/calibration.py` | Swapped bare `HardwareControlPanel` usage for `StandardJogContextPanel` in `get_context_widget()`. Forwards `set_hardware_config` and calibration-changed updates into the panel so the Hardware Info / Safe Z line stays in sync. |
| `gui/app.py` | Sidebar `menu_items` reordered (Calibration before Jog). `pages` list reordered to match. `cal_page = pages[1]` / `jog_page = pages[2]` references updated. Module-level + `_create_pages` docstrings updated to v7.4.3 page indices. |

## Implementation Steps

- [x] Branch from `Version-7.4.2` → `Version-7.4.3`
- [x] Build `gui/widgets/jog_workspace_view.py` — top-down XY canvas
- [x] Build `gui/widgets/xz_side_view.py` — side-view canvas
- [x] Build `gui/widgets/standard_jog_context.py` — reusable left panel
- [x] Rewrite `gui/pages/jog_control.py` — visualization-first cockpit
- [x] Switch Calibration page to `StandardJogContextPanel`
- [x] Reorder Calibration above Jog in `gui/app.py`
- [x] Fix pre-existing `pages[2]` / `pages[3]` index bug in app.py
- [x] Smoke-test in simulator (`python main.py --simulate-xy --simulate-zp`)
- [ ] Bump version to v7.4.3 (on completion)
- [ ] Run test suite (on completion)
- [ ] Architecture doc + README archive per CLAUDE.md checklist (on completion)

## Testing Notes

Manual test plan (run via `python main.py --simulate-xy --simulate-zp`):

1. **Sidebar order** — Hardware Setup → Calibration → Jog Control → Printing → Pick & Place → Settings (bottom).
2. **Main area is visualization only** — open Jog → no buttons / spinboxes / sliders in the main content; only XY workspace, XZ view, and pump rack.
3. **Same jog UI on Jog and Calibration** — open Calibration, then Jog. Both left panels look and behave identically (jog buttons, speeds, position bars).
4. **Hardware Info populated** — bottom of the left panel lists current needle, configured syringes, plate format, calibrated wells, safe Z. Confirmed on both pages.
5. **Absolute Go To works** — enter (0, 0, 0) with Safe Travel on → retract → XY → lower. Toggle off → direct moves, no retract.
6. **Soft limits engaged** — jog +X repeatedly until hitting `xy_max_x` — stage stops at the limit (not the mechanical end). Workspace view right-edge tints red.
7. **Free / Snap toggle** — default Snap → clicking outside any well does nothing. Toggle to Free → click anywhere in envelope → stage travels.
8. **Click-to-travel safe path** — Z = 0, safe_Z = 5 mm → click a distant well → Z lifts → XY travels → Z lowers (verify in XZ side view).
9. **Click-to-travel direct path** — Z = 8 mm → click a well → only XY moves.
10. **XZ side view** — jog Z toward plate; "Δ to plate" annotation counts down. Move past a calibrated well → well cavity drawn beneath needle.
11. **Breadcrumb trail** — jog in a zigzag → ~30 fading dots in the workspace; oldest fades within a few seconds.
12. **Calibration → Jog wiring fix** — after the index-bug fix, calibration data (plate + well positions + safe Z) reaches the Jog page on startup and on each `calibration_data_changed` emission.
13. **STOP / Esc** — Esc on the Jog page halts motion.
14. **No regressions in Print Monitor** — open Print Monitor after the rewrite — page still renders during a sim print.
15. **Tests** — `python -m pytest tests/` passes with no new failures.

## Issues & Decisions

- **`StandardJogContextPanel` is a wrapper, not a re-implementation.**
  The user explicitly asked us to use the calibration page's jog UI as
  the standard. `HardwareControlPanel` already implements that UI
  (compact JogButtonArray + speed controls + live position bars), so
  the new file wraps it rather than duplicating its logic. The wrapper
  adds two sections (Absolute Go To, Hardware Info) and forwards a few
  setters into the inner panel.
- **Hardware Info is read-only.** It's a quick-reference card —
  syringe sizes, well-plate format, safe Z. To change any of these the
  user goes to Hardware Setup or Calibration. This keeps the jog page
  focused on driving, not configuring.
- **`Quick Actions` (Set Zero / Go to Zero / Raise to Safe Z) dropped.**
  The user asked us to mirror the calibration panel's jog UI exactly +
  add Absolute Go To + Hardware Info. Calibration's panel has none of
  these buttons. The Absolute Go To with (0, 0, 0) covers Go-to-Zero;
  Set Zero lives on Hardware Setup; the canvas's Snap-to-wells covers
  most safe-travel cases.
- **Sidebar reorder reflects workflow.** With Calibration and Jog
  sharing the same left panel, ordering them earlier-in-list (Cal then
  Jog) reads as "calibrate first, then drive." This matches the
  natural session flow.
- **Pre-existing index bug.** The `pages[2]` / `pages[3]` mis-indexing
  in `app.py` was silent (hasattr-guarded), so calibration data never
  reached the Jog page in v7.4.2. The index realignment is required
  for the new Jog page to function correctly.
- **`PerAxisJogRow` widget removed.** Briefly written during the early
  iteration of this update, never used after the pivot to
  `StandardJogContextPanel`. Deleted to avoid dead code.
