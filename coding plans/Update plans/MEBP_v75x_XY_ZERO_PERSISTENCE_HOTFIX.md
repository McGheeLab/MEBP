# MEBP v7.5.x — XY Zero-Reference Persistence Hotfix

## Objective

Fix the reported bug: **the XY position readout is wrong after a software
restart**, even though the Prior ProScan controller stays powered and keeps
its absolute position.

## Diagnosis

This is the opposite situation to the ZP power-cycle problem:

- The ProScan reports **absolute controller µm** (`P` command). When the
  controller box stays powered across a software restart, it keeps the same
  absolute frame, and the saved hardware calibration (`_taught_a1`, well
  positions — all stored in absolute µm) is still valid.
- Everywhere in the app the XY readout is `displayed = raw − zero_position`
  (e.g. [app.py](../../gui/app.py) status bar, [calibration.py](../../gui/pages/calibration.py),
  [jog_control.py](../../gui/pages/jog_control.py)). `zero_position` is the
  software reference restored on startup (`main.py:207-209`).
- **The gap:** `zero_position` was only persisted by the explicit "Set Zero"
  buttons ([stage_panel.py](../../gui/pages/hardware/stage_panel.py),
  [settings_page.py](../../gui/pages/settings_page.py)). Paths that mutate it
  in RAM **without** saving — the Xbox **"zero_needle_pos"** button
  (`xbox_mapping_editor.py:307` → `StageController._calibrate_zero`) and
  auto-calibration — left the saved value untouched. And the clean-shutdown
  `MainWindow.save_settings()` persisted window geometry + hardware config but
  **not** `zero_position`.

So: the user zeroes XY (often via the Xbox controller), the readout looks
right, they close the app, and on restart `zero_position` reverts to the last
*button-saved* value (commonly `0`). The ProScan still reports the same
(correct) absolute position, but it's now decoded against the wrong reference
→ wrong numbers. (With `zero_position = 0` the readout is the raw absolute µm,
which looks especially wrong.)

## Fix

Persist `zero_position` on clean shutdown as a catch-all, so whatever
reference was active at close — regardless of how it was set — survives
restart. With the powered ProScan retaining its absolute position, the
restored reference reproduces the exact readout the user had before closing.

This complements `MEBP_v75x_ZP_POSITION_OVERRIDE.md`: ZP loses its *firmware*
position on power cycle and needs a manual re-declare; XY keeps its *hardware*
position but lost its *software reference* — restoring the reference is all
that's needed.

## Files Modified

| File | Change |
|------|--------|
| `gui/app.py` | `MainWindow.save_settings()` now also writes `settings.set_section("zero_position", dict(self.controller.zero_position))` (guarded). Runs on clean shutdown via `closeEvent`. |
| `tests/test_v75x_xy_zero_persistence.py` | New round-trip tests (3). |

## Implementation Steps

- [x] Persist `zero_position` in `save_settings()` (clean-shutdown catch-all)
- [x] Round-trip tests (save → restart restore → correlate)
- [ ] Architecture doc / README (defer to version-completion checklist)

## Testing Notes

`python -m unittest tests.test_v75x_xy_zero_persistence`

1. The zero reference round-trips through `Settings` using the exact calls
   `save_settings()` now makes and the `main.py` startup restore.
2. With the reference restored, a fixed raw read decodes to the same zero-ref
   position as before closing.
3. Guard test: without the restore (pre-fix behavior), the same raw read
   decodes to the wrong (absolute) value — documents the bug mechanism.

Manual (hardware): zero XY via the Xbox button or calibration (do **not** click
"Set Zero"), note the readout, close the app cleanly, reopen, reconnect XY —
the readout should match what it showed before closing.

## Issues & Decisions

- **Scope:** Covers the reported *clean-restart* symptom. The explicit "Set
  Zero" buttons already persist immediately; this adds the close-path
  catch-all that captures non-button zeroing. A crash/kill between zeroing and
  a clean close would still lose a non-button reference — making
  `StageController` notify the GUI on every `zero_position` change (immediate
  persist) is a possible follow-up but was kept out to avoid crossing the
  hardware-abstraction boundary for a clean-restart bug.
- **Why not re-anchor from calibration:** since the ProScan retains its
  absolute frame while powered, no re-anchoring is needed — only the software
  reference was missing. (If a deployment power-cycles the ProScan box, that
  would be the lost-frame case and would need an XY re-anchor tool, which the
  ProScan III protocol here can't do directly — it has only `Z` (zero-at-spot),
  no set-position-to-arbitrary command.)
