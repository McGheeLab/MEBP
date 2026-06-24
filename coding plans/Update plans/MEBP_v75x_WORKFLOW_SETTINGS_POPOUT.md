# MEBP v7.5.x — Per-Workflow Settings Popout (saveable/loadable)

## Objective

Give **every** workflow a comprehensive, **scrollable** "⚙ Settings" popout that
exposes a robust set of options for how that workflow operates (offsets, Z
heights, pick/place flow rates, dwell/pause times, prep/clean sub-parameters,
speeds, timeouts) **plus** a read-only "Locations & Hardware" panel that *shows
where all inks/reagents/service wells are*, the needle/syringe geometry, pump↔ink
assignments, and the calibrated Z reference heights.

The operator can tune the settings, then **save them to a named settings file
for that workflow and reload it later** (Save / Save As… / Load / Delete /
Import file… / Export file… / Reset to defaults). The last-used values
auto-restore on the next visit.

Decisions (operator-confirmed):
- **Comprehensive, no duplication** — the numeric config moves *into* the popout
  so it is THE settings surface. The page keeps only: header (+ ⚙ Settings
  button + a one-line settings summary), the live views, object/well selection
  needed for live preview, a status line, and Start/Abort.
- **Saveable per-workflow settings files** the user can reload (not just a
  single settings.json blob), so a user can keep the settings they like and
  reload them for that workflow.

## Files Modified / Added

| File | Change |
|------|--------|
| `SupportClasses/WorkflowSettingsStore.py` | **NEW** — per-workflow JSON profile store under `config/workflows/<workflow_id>/`. `list_profiles` / `load_profile` / `save_profile` / `delete_profile` / `load_last` / `save_last` / `read_file` / `write_file`. Atomic writes (tmp + `os.replace`), filename sanitising. `__last__.json` = auto-saved current values. |
| `gui/dialogs/workflow_settings_dialog.py` | **NEW** — scrollable `WorkflowSettingsDialog` (QScrollArea, `setWidgetResizable(True)`): profile bar (combo + Save/Save As/Load file/Export/Delete/Reset), section helpers (`add_section` → `SettingsSection.add(key,label,widget,default,help)` which lays out a `FormRow` AND registers the field for persistence/reset), generic widget get/set for QSpinBox/QDoubleSpinBox/QCheckBox/QComboBox, last-used auto-restore + pending-combo resolution (combos populated by hw-config after init), and a shared **Locations & Hardware** read-only builder (`build_locations_widget`) re-run each time the dialog is shown. |
| `SupportClasses/PickAndPlaceManager.py` | `SpheroidPickupConfig` gains `pick_dwell_s` / `place_dwell_s` (default 0.0 → no-op via `_dwell`); `_execute_spheroid_pickup` dwells after the aspirate and after the dispense. (`pickup_speed_uL_s`/`release_speed_uL_s` already existed but had no UI.) |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | Config/prep rows → settings popout; new advanced knobs (pick/place flow µL/s, pick/place dwell, prep rate, oil/buffer needles, wash Z/XY amplitude + dwell, post-clean + expel needles, intra-well retract, Z/XY timeouts). |
| `gui/pages/workflows/cell_targeting_workflow.py` | Config/reagent/prep rows → settings popout; advanced prep/clean + timeout knobs. |
| `gui/pages/workflows/quick_print_workflow.py` | Pump/flow/speed/print-Z/ink/prep → settings popout (object + size + well selection stay on the page for live preview); advanced (travel speed, pre-flow s, intra-well hop Z). |
| `gui/pages/workflows/stress_test_workflow.py` | Config grid → settings popout (saveable stress presets). Monitor + run row stay on the page. |
| `gui/pages/workflows/timing_calibration_workflow.py` | Config grid + start-location → settings popout. Camera/plots/monitor/run row stay on the page. |
| `CLAUDE.md` | Add this plan to the Existing Update Plans table. |

## Implementation Steps

- [x] Read architecture (workflows mode + page hooks) + store/save-load conventions
- [x] `WorkflowSettingsStore`
- [x] `WorkflowSettingsDialog` + `build_locations_widget`
- [x] Backend: spheroid pick/place dwell (+ flow rates already present)
- [x] Spheroid page integration (reference implementation)
- [x] Cell Targeting page integration
- [x] Quick Print page integration
- [x] Stress Test page integration
- [x] Timing Calibration page integration
- [x] Tests
- [x] CLAUDE.md table

## Status: complete (pending real-HW verification)

## Testing Notes

- `tests/test_v75x_workflow_settings_popout.py` — store round-trip (save/load/list/
  delete/last/import/export), dialog field registry get/set/reset for each widget
  type, pending-combo resolution, locations-panel builds from a fake hw_config,
  and each workflow page builds offscreen + exposes its config widgets + opens the
  popout without error.
- Backend: spheroid dwell defaults 0 (no behavior change); existing spheroid suite
  stays green.
- Regression: quick-print / cell-targeting / spheroid / stress / timing suites.

## Issues & Decisions

- Widgets are created **eagerly** in each page `__init__` (inside the dialog) so
  the existing tests that read `page._flow_spin` / `page._push_depth` etc. and the
  `_current_config()` / `_build_settings()` builders keep working. The dialog is a
  persistent, modeless child of the page (shown on demand) — never destroyed while
  the page lives, so the hosted widgets stay valid.
- Quick Print keeps `_object_combo` + `_size_spin` + the well navigator inline
  (object/well selection drives the live trajectory preview, and a test asserts
  `_size_spin.isVisibleTo(page)` toggles with object kind).
- `pick_dwell_s` / `place_dwell_s` default 0.0; `_dwell(0)` returns immediately so
  the executor stream is byte-identical when dwell is unused.

## Adversarial review fixes (post-implementation)

A 4-dimension adversarial review (21 agents) confirmed 13 findings; the
substantive ones were fixed:

- **Empty "(none)" combo not restored on Reset / lost next session** (ink/reagent)
  — `set_widget_value` now selects the `data==""` placeholder item for an empty
  selection; `load_last(reapply_combos=True)` + `resolve_pending` re-assert a
  restored "(none)" once *after* the page's combo repopulate (which auto-defaults
  to the pump's assigned ink) so it isn't clobbered.
- **Silent save failure** — `WorkflowSettingsStore._write` now RAISES on failure;
  Save / Save As / Export surface it via a warning dialog (`save_last` stays
  best-effort).
- **Reserved `__last__` profile name** — `profile_path` never lets a user profile
  collide with the auto-saved last-used file.
- **Spheroid post-clean ignored wash/buffer settings** — wash + buffer-needle
  knobs are applied whenever prep OR clean runs (not prep-only), and the prep
  widgets are enabled when prep OR clean is on (`_post_clean_check` now drives
  `_on_prep_toggled`).
- **Cross-thread widget read** — Quick Print captures `wash_cycles` on the GUI
  thread before launching the preflight worker.
- **Stress / Timing locations panel** now stores + shows the plate Z heights.
- Dead `_loading` flag removed.

Tests extended to lock these in (reserved name, write-failure raises, empty-combo
reset/reapply, spheroid clean-only enable).

## Real-HW

Needs real-HW verification on ME3B V1 (the new flow-rate / dwell / prep knobs feed
the same executor primitives that are already HW-validated; this change is GUI +
config plumbing).
