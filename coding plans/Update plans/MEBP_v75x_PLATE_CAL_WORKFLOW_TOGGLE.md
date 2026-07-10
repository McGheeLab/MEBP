# MEBP v7.5.x — Plate-Calibration Workflow Toggle (Mosaic default, Legacy deprecated)

**Status: implementation complete — needs GUI verification on ME3B V1.**

## Objective

Operator request (follow-on to `MEBP_v75x_ROSETTE_TAB_AUTO_REANCHOR_AND_MOSAIC_UI.md`):

> "on the plate location calibration, we no longer really need the well fit or the target queue,
> this is now better handled by the mosaic tools. I don't want to take these features away just
> yet, but we should make a toggle for version of plate calibrations that defaults to the mosaic
> workflows, and then these well fit target queue can be another version that is slightly
> deprecated."

The Plate Location tab now shows exactly ONE of two workflow versions, chosen by a
**"Plate calibration:"** combo at the top of the tab:

- **"Mosaic workflow (recommended)"** — the DEFAULT. Shows the Mosaic group (Calibrate mosaic
  scan · Mosaic scan · Re-anchor mosaic · Auto re-anchor mosaic · Plate view combo · Well scans ·
  Advanced… menu) and the live mosaic preview.
- **"Legacy well-fit / target queue (deprecated)"** — the old per-well flow: the "Well fit"
  Manual/Auto combo, the target queue list + Clear/Run, and the queue status line, with an inline
  deprecation note. Fully functional; nothing removed.

Only widget **visibility** switches — every widget attribute stays alive and every handler stays
wired, so the legacy flow (and its tests) keep working until it is actually removed. The choice
persists across restarts.

## Design

- **Independence verified before design**: the two flows already share nothing but mutual
  exclusion (`_ploc_running` / `_ploc_mosaic_running`) and the shared canvas/live-view widgets.
  `_ploc_fit_mode` (the "Well fit" combo) is read ONLY inside the legacy queue run
  (`_ploc_run_queue` / `_ploc_advance`) → legacy-only. Confirm/Skip/**Cancel** row stays untouched
  in both modes (state-driven-hidden; Cancel is shared with the mosaic-scan cancel).
- **Shared widgets stay visible in both modes**: banner (text swapped per mode), plate view +
  Free/Snap toggle, XZ side view, live microscope feed + hint, confirm row.
- **Mosaic mode hides**: `_ploc_mode_row_w` (Well-fit combo row), `_ploc_queue_group`
  (new QGroupBox "Well-fit queue (legacy)" wrapping the Target-queue label + `_ploc_queue_list` +
  Clear/Run row + `_ploc_legacy_note`), `_ploc_status` (queue count line).
- **Legacy mode hides**: `_ploc_mosaic_box` (whole Mosaic group incl. the Advanced menu),
  `_ploc_mosaic_prev_col` (the "Mosaic (live)" preview column — the microscope feed gets full
  width for click-rim work), and force-exits/hides the Manual-align slider group
  (`_ploc_align_group.setChecked(False)` for a clean handler exit + `setVisible(False)` +
  uncheck `_ploc_act_manual_align` with blocked signals) since its only entry point (Advanced…)
  hides with the mosaic box.
- **Click gating**: with the queue hidden, plate-view clicks must not mutate invisible state —
  `_ploc_on_well_clicked` / `_ploc_on_position_clicked` now return early unless
  `_ploc_workflow_mode == "legacy"`. The gate sits AFTER the manual re-anchor stage routing, so
  re-anchor overview clicks keep working in mosaic mode (regression-locked).
- **Run guard**: `_ploc_on_workflow_changed` refuses to switch while a queue run or mosaic scan is
  active (QMessageBox + combo revert with blocked signals) — the UI is never yanked out from
  under a run.
- **Persistence**: settings.json section `"plate_location_prefs"` → `{"workflow": "mosaic" |
  "legacy"}` (merge-preserving write + `settings.save()`, mirroring the `mosaic_scan` section
  pattern). Restored at the end of `_build_plate_location_tab` via
  `_ploc_set_workflow_mode(mode, persist=False)` (combo synced with blocked signals). Missing or
  garbage value → mosaic.
- **Banner text per mode**: new class constants `_PLOC_BANNER_MOSAIC` (short mosaic-flow
  instructions) and `_PLOC_BANNER_LEGACY` (the previous banner text prefixed with a
  "[Deprecated]" sentence).

## Files Modified

| File | Change |
|---|---|
| `gui/pages/calibration.py` | Workflow combo row (`_ploc_workflow_combo`) above the banner; legacy widgets wrapped in `_ploc_mode_row_w` + `_ploc_queue_group` (with `_ploc_legacy_note`); `_ploc_mosaic_box` / `_ploc_mosaic_prev_col` attribute refs; `_ploc_workflow_mode` state + `_PLOC_BANNER_MOSAIC/_LEGACY` constants; new `_ploc_on_workflow_changed` + `_ploc_set_workflow_mode(mode, persist=True)`; restore-at-build; legacy-mode gate in the two plate-view click handlers. |
| `tests/test_v75x_plate_location_workflow_toggle.py` (new) | 21 tests (see below). |
| `quickstart_guide/guide_content.json` | Plate Location entry documents the workflow toggle. |
| `CLAUDE.md` | Row in Existing Update Plans. |

## Implementation Steps

- [x] Wrap legacy widgets in containers (`_ploc_mode_row_w`, `_ploc_queue_group` + deprecation
      note); keep `_ploc_mosaic_box` / `_ploc_mosaic_prev_col` refs.
- [x] Workflow combo row + `_ploc_on_workflow_changed` (run guard + revert) +
      `_ploc_set_workflow_mode` (visibility + align-group cleanup + banner swap + persist).
- [x] Restore persisted mode at the end of `_build_plate_location_tab` (default mosaic).
- [x] Gate queue mutation in `_ploc_on_well_clicked` / `_ploc_on_position_clicked` on legacy mode
      (after the re-anchor routing).
- [x] Tests + regression suites.
- [x] Docs (this plan, CLAUDE.md, quickstart guide).

## Testing Notes

- New `tests/test_v75x_plate_location_workflow_toggle.py` — **21 tests, all green**:
  default-mosaic visibility + attrs alive; switch flips visibility/banner (idempotent);
  align-group cleanup on legacy entry; combo drives mode; bogus mode coerced; persistence
  (write + merge + restore-at-build + garbage/missing → mosaic + restore doesn't rewrite);
  run guard (queue run + mosaic scan refuse, idle allows); click gating (well + freeform,
  re-anchor overview still routes in mosaic mode); legacy Run button enables on enqueue.
- Regression: `test_v75x_plate_mosaic`, `test_v75x_rosette_tab_auto_reanchor`,
  `test_v75x_single_well_mosaic_reregister`, `test_v75x_plate_location_manual_click_rim`,
  `test_v75x_mosaic_orientation_remap` — green except the documented pre-existing CV failure
  `test_real_24_well_mosaic`.
- **Real-HW verification on ME3B V1 (pending)**: tab opens in Mosaic mode showing only the
  4-button group + view controls; switching to Legacy restores the old queue flow end-to-end
  (enqueue → Run → click-rim → Confirm); preference survives a restart; switching is refused
  mid-run/mid-scan.

## Issues & Decisions

- **Toggle placement**: a combo at the very top of the tab (outside both groups) — an
  Advanced-menu action wouldn't work because the menu itself hides with the mosaic box in legacy
  mode.
- **Well-fit combo classified legacy-only** after verifying `_ploc_fit_mode` is read solely by
  the queue-run state machine (calibration.py `_ploc_run_queue` / `_ploc_advance`).
- **Confirm/Skip/Cancel row untouched**: already state-driven-hidden, and Cancel is shared with
  the mosaic-scan cancel path.
- **Click-to-enqueue disabled in mosaic mode** (silent no-op): with the queue hidden, enqueueing
  would mutate invisible state. Re-anchor clicks are routed before the gate.
- **Manual-align group force-exited on legacy entry**: `setChecked(False)` first (fires
  `_ploc_on_align_mode_toggled(False)` for a clean align-mode exit), then hidden; the menu action
  unchecked with blocked signals (its `toggled → setVisible` link is bypassed deliberately).
