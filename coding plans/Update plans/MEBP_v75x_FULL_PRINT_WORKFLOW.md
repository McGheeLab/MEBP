# MEBP v7.5.x — Full Print workflow (relocate Printing into Workflows + modernize)

## Objective

Treat **Quick Print** as the place to *test printing conditions* on one object in one
well, then *graduate to a full plate* via a new **"Full Print"** Workflows tile. Concretely:

1. **Re-home, don't rewrite** — move the existing rich Printing-mode stack (Setup → Monitor →
   Results) into the Workflows mode as a "Full Print" tile, preserving every setup option
   (multi-object collections, per-well pump/role assignment, the full Plan-of-Action, validation,
   monitor, results).
2. **Remove** the old "Printing" entry from the left sidebar — Workflows is now the sole entry to
   full-plate printing.
3. **Modernize** the full-print build path with the v7.5.x ecosystem fixes it had drifted behind.
4. **Share settings** — Full Print uses the same modern settings substrate (CommonPrintSettings)
   and gains a one-click **"Load from Quick Print profile…"** import so conditions tuned in Quick
   Print transfer to Full Print.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/workflows/full_print_workflow.py` | **NEW** — `FullPrintWorkflowPage(QWidget)` thin wrapper hosting a `PrintingModePage`; "← Back to Workflows" header; exposes `setup_page`/`monitor_page`/`results_page` + `switch_to_*`; delegates the workflow-page contract; re-emits inner `sub_page_changed`. |
| `gui/pages/workflows/workflow_picker.py` | Added the `full_print` `WorkflowTile` (🖨️, enabled) after `quick_print`. |
| `gui/pages/workflows_mode.py` | Import + `elif "full_print"` dispatch branch; `full_print_page` property + `open_workflow(id)`; loop hook re-emitting each page's `sub_page_changed`. |
| `gui/app.py` | Dropped `PrintingModePage` import + `self._printing_mode`; added `self._full_print_page = self._workflows_mode.full_print_page`; repointed all `_printing_mode.*` uses (recorder, `_wire_job_pipeline`, `_wire_print_manager_to_monitor`, `_send_job_to_monitor`, `_setup_monitor_visualization` callers, `_on_monitor_start/pause/resume/abort`, `_on_print_completed_v726`, `_on_print_created`); rewrote the two nav tails to route via Workflows → Full Print; removed the `btn_printing` menu item + `pages` entry (Workflows 5→4, Settings 6→5); updated `btn_map` + fallback `titles`/`context_titles`; added `_workflows_index()`. None-guarded every repointed site. |
| `gui/pages/print_setup/page.py` | `WizardPrintSetupPage.set_common_print_settings` forwards to the legacy page. |
| `gui/pages/print_setup_legacy.py` | µL units fix (`_get_settings` populates `retract_amounts_uL`/`prime_amounts_uL`); consume stashed hop / line-speed / travel-speed; new `_get_path_segments_from_objects()` + `_build_current_job` passes `path_segments=`/`return_home=True`; `set_common_print_settings` (stores model); "Load from Quick Print profile…" button + `_on_load_quick_print_profile` + `_apply_quick_print_profile` + `_line_move_speed_maxes`. |
| `tests/test_v75x_full_print_workflow.py` | **NEW** — 19 tests. |

## Implementation Steps

- [x] **1a** `FullPrintWorkflowPage` wrapper hosting `PrintingModePage`.
- [x] **1b** Register the `full_print` tile.
- [x] **1c** Dispatch branch + `full_print_page`/`open_workflow` + `sub_page_changed` forward.
- [x] **1d** Repoint `app.py` `_printing_mode` → `_full_print_page` (all sites, None-guarded).
- [x] **1e** Remove the Printing sidebar entry + reindex `pages`/`btn_map`/`titles`/`context_titles`; add `_workflows_index()`.
- [x] **2** Modernize build path: µL units fix; per-object `path_segments`; `return_home=True`; consume hop/line-speed stash.
- [x] **3a** `set_common_print_settings` plumbed wrapper → wizard → legacy (reaches the page via the existing `_fanout_common_print_settings` → workflows-mode fanout).
- [x] **3b** "Load from Quick Print profile…" import (button + chooser + mapping).
- [x] Tests + suites + MainWindow smoke.
- [ ] Real-HW verification on ME3B V1.

## Testing Notes

- `tests/test_v75x_full_print_workflow.py` (19, all green): tile registered+enabled; wrapper
  exposes setup/monitor/results + switch_to_* + print_manager (via setup_page); contract methods
  present; `get_context_widget()` is None (left box stays hidden); `set_common_print_settings`
  forwards to legacy; inner `sub_page_changed` re-emits; `_get_settings` populates the µL dicts;
  hop stash consumed; `_get_path_segments_from_objects` splits per object with offsets + empty
  fallback; `_apply_quick_print_profile` maps speed/printz exact, extrusion_mod via ×100 clamp,
  stashes hop, decodes combo tokens, and never imports globals/prep/ink.
- Regression suites green: `print_setup_print_manager_forward`, `printing_mode_calibrated_wells`,
  `multi_object_print_seam`, `print_setup_routine`, `common_print_settings`,
  `workflow_settings_popout` (94), plus the quick-print suites (130; the 6
  `TestLineMoveSpeedPercent` failures are **pre-existing** — reproduced with this change stashed,
  unrelated to it).
- **Full `MainWindow` smoke** (simulated XY+ZP): builds cleanly; `_full_print_page` is a
  `FullPrintWorkflowPage`; pages = [HardwareSetup, Calibration, Jog, PrintBuilder, Workflows,
  Settings] (no `PrintingModePage`); `_workflows_index()` = 4; monitor pause/resume/abort resolve.
- **Manual (real app, pending):** sidebar has no 🖨️ Printing; Workflows shows the Full Print tile →
  opens Setup/Monitor/Results with the vertical icon nav + embedded Tools; Back returns to the
  picker; sending a job from Print Setup auto-navigates to Workflows → Full Print → Monitor (not
  Print Builder); Print Builder "Send to Print Setup" → Workflows → Full Print → Setup;
  start/pause/abort/results work; calibration reaches the page (taught wells); "Load from Quick
  Print profile…" fills the Finalize controls.

## Issues & Decisions

- **Wrapper-hosts-`PrintingModePage`** chosen over moving the 3 sub-pages directly — re-homes the
  mature page with zero changes to it (preserves vertical icon nav, embedded Tools context, wizard
  step strip), honoring "re-home, don't rewrite."
- **Pre-existing nav bug fixed:** `_send_job_to_monitor` previously did `_switch_page(3)` which
  navigated to **Print Builder** (index 3), not Printing. Routing through
  `open_workflow("full_print")` corrects it.
- **µL units bug:** the Finalize retract/prime spins are µL-labelled but `_get_settings` only wrote
  the legacy *mm* dicts; `build_well_plate_job` reads `*_uL` first, so the µL value drove a mm pump
  move. Now both dicts are populated (µL path wins). Aligns with CLAUDE.md §4.
- **`path_segments` scope (honest limitation):** the seam fix lands on the **discrete**
  `build_well_plate_job` path (`_build_current_job`), which is the FALLBACK. The full print's
  *default* executed path is **hybrid** (`_generate_print` → `PrintTrajectoryPlanner` +
  `PrintPlanOfAction`), whose in-well multi-object trajectory is generated separately and is NOT
  changed here. A hybrid/trajectory-path multi-object seam pass is a **deferred follow-up** (and
  the trajectory planner has a known `ZDIR=+1` polarity gap on ME3B V1 — keep `execution.mode` on
  the default "hybrid"; do not switch to pure trajectory).
- **Quick-Print-profile import scope (safety):** maps ONLY speed / fill / print-height / prime /
  inter-line motion knobs. `extrusion_mod → volume_fraction` is a documented semantic adapter
  (×100, clamp 1..100); `preflow → prime µL` uses the derived flow × seconds. It SKIPS
  ink/prep/postclean (no clean Full-Print analog) and NEVER writes back the GLOBAL HardwareConfig
  `g_settle`/`g_relief`, well positions, or calibration.
- **Safety invariants preserved:** retract-before-XY (only existing primitives; the inter-object
  hop routes through `ensure_retracted_to`); end-at-safe-Z (hybrid `finally` + real
  `travel_z_height` from calibrated safe Z); `z_up_sign`/`plate_axis_sign` stamped; calibrated
  positions never re-signed.

**Needs real-HW verification on ME3B V1.**
