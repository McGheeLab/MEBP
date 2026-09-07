# MEBP v7.19 — Quick Print: a plate-wide queue of independent prints

## Objective

Turn the Quick Print **Setup** zone into a plate-layout surface: assign an
independent print to each of many wells, see at a glance what each well will get,
click any well to pull that print back for editing, then run the whole plate
unattended — while keeping the ability to run just the one being edited.

Operator's request, verbatim:

> "on the quickprint workflow, in the setup page I want the section with the well
> plate selector to have the print preview above the well. Also, I want to be able
> to select in multiple prints in queue. An overlay of the print should be drawn
> into the well selected in the well plate selector. clicking on any well
> populates that print into the active view for editing. users can fill the plate
> with prints, a button for run all prints or run active print should be there.
> each print is still treated as an independent activity."

Three gaps it names:

1. **The print preview was in the wrong zone** — `PrintTrajectoryMonitorView`
   (`_traj_view`) lives in the *Run* zone, so during setup there was no picture of
   what the chosen object actually draws.
2. **The plate selector showed no content** — `WellPlateNavigator` colours wells by
   calibration state only; nothing said "this well gets a spiral".
3. **There was no queue** — one well, one print, one run. Every parameter was read
   live off the widgets the moment Print was pressed, and the page held exactly one
   of everything: one `_selected_well`, one `_pending_print`, one `_pm`.

### Decisions taken with the operator (AskUserQuestion ×7)

| Question | Answer |
|---|---|
| Where the preview goes | **Both** — a large ACTIVE-print preview above the plate, *and* small toolpath overlays drawn into every queued well |
| What a queued print remembers | **Full snapshot** — its own object, size, pump, ink, speed, resolution, print Z, extrusion × |
| Run-all needle handling | **Prep once** at the start, ink pickup per print, cleanup once at the end; ink swap only between prints that change ink |
| Filling the plate | Select wells (**drag-click** supported), then **"Apply print to wells"** in the **bottom-right of the print preview** |
| Run order | **Plate order** by default, with an optional **"Group by ink"** toggle |
| An entry that can't run | **Skip it, run the rest** — named in the confirm dialog and flagged on the plate |
| Pausing | **Hold between wells**, so the operator can go run a fluorescence mosaic and come back |

**With an empty queue nothing about the existing single-print path changes**, and a
navigator with no overlays and multi-select off is **pixel-identical** to before.

---

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/PrintQueue.py` | **NEW** — `QueuedPrint`, `order_queue`, `count_ink_swaps`, tolerant (de)serialization. Qt-free and controller-free, modelled on `PrintReadiness.py`, so the expensive logic is testable in milliseconds |
| `gui/widgets/jog_well_plate.py` | per-well toolpath glyphs, opt-in multi-select + brush drag, selection rings; `set_plate` clears the new well-keyed state |
| `gui/pages/workflows/quick_print_workflow.py` | Setup-zone preview + vertical splitter, queue card, apply-to-wells, the resolve pass, the run-all worker + hold, the debounced write-back, persistence, and two pre-existing bug fixes |
| `gui/widgets/components.py` | `Card.set_title` — the setter the existing `title()` getter had no counterpart for (the queue card shows a live count) |
| `SupportClasses/StageController.py` | `is_position_poller_suspended()` — one guarded accessor, the cheap "is another workflow driving the stage" proxy a held queue needs |
| `tests/test_v719_print_queue_model.py` | **NEW** — 35 pure-model tests, no Qt |
| `tests/test_v719_quick_print_queue.py` | **NEW** — 91 widget + page + worker tests |

---

## Implementation Steps

- [x] `SupportClasses/PrintQueue.py` — `QueuedPrint` + queue helpers + ordering
- [x] `StageController.is_position_poller_suspended()`
- [x] Navigator: `set_well_paths`, multi-select + brush drag, selection rings
- [x] Setup zone: vertical splitter, second `PrintTrajectoryMonitorView`, plan fan-out
- [x] `Apply print to wells` footer + queue card + `Run all queued` / `Hold after well`
- [x] Snapshot capture / apply / the `_snapshot_applied` context manager
- [x] The debounced single write-back sink + the flush on well switch
- [x] `_resolve_queue` + per-unit gating + the cumulative syringe budget
- [x] The run-all worker + hold/resume + abort routing
- [x] Persistence via the existing `set_extra_state` hook
- [x] Two pre-existing bug fixes (below)
- [x] Tests: 126 new, 25/25 mutations caught
- [x] Regression sweep

---

## Design notes — the decisions that carry weight

### 🔴 Store INPUTS, never derived outputs

`segments`, `PrintSettings`, `center`, `ink_pos` and above all any **zero-ref** Z
stay OUT of the snapshot. Between queueing a print and running it the operator can
re-run Plate Location, swap the needle, or re-calibrate the plate bottom; a
snapshot holding a derived zero-ref Z would then print at a **stale absolute height
with nothing on screen saying so**. `print_z_mm` and `ink_dip_z_mm` are heights
*above plate bottom*, so every run re-resolves them against live calibration. It
also keeps a snapshot JSON-serializable and makes "the print file was deleted since
you queued it" a detectable condition rather than a crash.

### 🔴 `ink_map_by_name`, not `ink_map: dict[int, str]`

Two independent reasons, both verified in the code. (a) JSON round-trips integer
keys as **strings**, so an int-keyed map silently breaks on reload. (b)
`_rebuild_ink_mapping_ui` **hard-resets `self._ink_map = {}`** (`:3171`) and
re-derives it from the name-keyed `_ink_map_last` (`:3200`) — so a snapshot that
assigned `_ink_map` directly would have its mapping **wiped by the very refresh
that follows applying it**. The snapshot seeds `_ink_map_last` and lets the existing
owner derive `_ink_map`.

⚠ Known limitation, accepted and documented rather than papered over:
`_ink_map_last` is page-wide, so two queued prints using the same abstract-ink
*name* with different mappings will fight. A second per-well ink-map store would be
the two-homes trap.

### 🔴 No modal dialog while a transient snapshot is applied

`_path_segments_for_selection` and `_build_settings` read widgets, so resolving a
snapshot that is not on screen means transiently applying it. That is only safe
while **nothing re-enters the Qt event loop**: `_confirm_print_floor` calls
`QMessageBox.warning`, and the app's ~300 ms status tick would then run
`_refresh_setup_status` / `_refresh_planned_path` against the **transient** widget
values and repaint the preview from the wrong snapshot. So the floor is evaluated
**numerically** per unit via `controller.print_floor_violation(z)`, the offenders
are collected, and **one** dialog is shown after everything has been restored —
better UX than N modals *and* it removes the hazard.

Two more traps in the same area:

* **With signals blocked the derived caches do not refresh.** The risk is the side
  effect *not* happening: `_loaded_sketch` is normally set only inside
  `_on_object_changed`, so `_is_multi_ink()` / `_ink_groups()` would read the
  *previous* object's sketch. `_apply_snapshot` sets it explicitly.
* **`setValue` clamps silently.** `_update_top_speed_cap` puts a dynamic maximum on
  the speed spin, so a print queued at 6 mm/s on a machine since measured at
  4.2 mm/s would quietly run slower. Every numeric set is compared back and any
  difference is **disclosed** in the confirm dialog.

Restoration IS an apply of the snapshot captured on the way in, so the save-list and
the restore-list are the same list by construction and cannot drift as fields are
added.

### 🔴 The sortable atom is the queued print, never the print unit

`_ink_groups()` returns groups in *sketch shape order* — the author's deposition
sequence (support before structure). Globally regrouping **units** by ink to save
swaps would silently reorder within a multi-ink print and print it wrong. A
multi-ink entry's unit run is **atomic**; `order_queue` permutes `QueuedPrint`
objects only.

### 🔴 One shared abort predicate

`_run_group_job_blocking` reads the sticky abort flag **directly** to close the
documented `pm.start()` race (`abort()` is gated on RUNNING/PAUSED and `start()`
clears the flag). A queue with its own flag would have left that guard **dead for
the queue** — a unit would print after the operator had already aborted. Both
engines now go through `_sequence_abort_requested()`.

### Hold between wells — and the honest limit of its guard

At a well boundary the worker retracts to safe Z, signals `queue_held(True)` and
parks on an `Event`. **A held batch holds no hardware** — verified:
`safe_travel_to` resumes the position poller in its own `finally`, the
poller/watchdog suspends are scoped to `PRINT_PATH`, and no `PrintManager` is live
between units. So the operator really can go run a fluorescence mosaic.

The hazard is the reverse: **Resume while another workflow drives the stage**.
`_stage_busy()` is a per-**page** guard, and there is no cross-page registry, so
Quick Print cannot see a running mosaic scan. Resume is therefore gated on the one
cheap observable — `is_position_poller_suspended()`.

⚠ **That is a PROXY, not a lease.** It catches a running scan or print (both suspend
the poller for their duration); it does **not** catch a manual jog, and it cannot
say *which* workflow is driving. A process-wide stage lease is the right long-term
fix and is deliberately out of scope here. Also disclosed: ink sits in the needle
across a long hold and may settle.

### Refusals that are stated rather than guessed

* **`ink_name == ""` ("needle already loaded") is refused by Run all** — after the
  single prep, and after every swap, the needle has been washed out, so "print with
  whatever is loaded" is a continuity claim the engine has already invalidated.
  Allowed only when the *whole* queue is `""` **and** prep and post-clean are both
  off (then it is purely a job loop).
* **Same-ink adjacency correctly skips the swap**, but the plunger then drifts
  monotonically across the batch. The single-print `_check_syringe_budget` models
  ONE print and the multi-ink path skips it with no mention anywhere in the UI —
  **not repeated here**. `_check_queue_syringe_budget` sums the whole queue through
  the existing `simulate_pump_budget` and always states one of three verdicts:
  **fits** (naming the span, and that wash/prep volumes are *not* modelled) ·
  **won't fit** (→ blocks, remedies named) · **not checked** (no plunger
  calibration / unreadable fill → proceeds, operator informed).
* **An uncalibrated well is refused, not run at a guess** —
  `_well_center_zero_ref_mm` has a geometric fallback, and a batch that dips into
  ink wells and prints at a computed height must not run against it. Two
  enforcement points, deliberately (the skip pre-check owns the plate FLAG; expand's
  guard owns the geometry), each independently pinned so neither becomes dead code.

### The performance trap

384 wells × `generate_object_trajectory` on a 120 ms debounce is not viable, so the
glyph geometry is cached by **config identity** (object + size + resolution), not by
well: a real queue is "the same print in forty wells", so the cache normally holds
**one** entry (measured: 1 miss + 7 hits for 8 wells). Polylines are decimated to
≤64 points for **display only** — the executor always uses the full geometry.

### UI choices worth recording

* **A second `PrintTrajectoryMonitorView`**, not `WellPreviewWidget`: the monitor
  already consumes exactly what `_refresh_planned_path` computes, so **no second
  copy of the well-centre → zero-ref conversion has to exist**. That conversion
  handles calibrated-vs-geometric and `plate_axis_sign()`; a copy would get the sign
  wrong exactly once, on hardware. `WellPreviewWidget` would also add a second
  coordinate convention (well-relative mm, **Y up**) to one page.
* **The live trace stays on `_traj_view` only.** During a queue run the Setup
  preview shows the ACTIVE print, which may not be the well printing — a needle dot
  there would be **false**, not merely noisy.
* **The Apply button is a footer ROW, not a floating overlay.** The monitor paints
  its prediction caption along the bottom — the very text needed before stamping a
  print into forty wells — and an overlay child would occlude it, need a subclass or
  event filter to hook `resizeEvent`, and contribute nothing to `sizeHint` so it
  would clip on a narrow pane instead of the layout yielding.
* **Apply is NOT gated on readiness** — laying out a plate with the stage
  disconnected is the whole point of a queue. **Run all IS**, but on
  `readiness.blocking()` filtered to ids outside `{"object", "well"}`: those describe
  the *active on-screen* print, which a batch supplies per entry, so gating on them
  would refuse a valid queue because the object combo was left empty.
* **No Y flip on the glyphs** — plate-local is Y-down (`PlateTransform.to_px`),
  matching `PlateThumbnail` and *not* `PrintThumbnail` (a standalone "Y up" chart).
  The clip is load-bearing: an oversized print is visibly **clipped, not scaled**,
  because it is a real design problem the operator should see. Below
  `_GLYPH_MIN_R_PX = 6.0` the glyph becomes a dot — on a 384 plate a few-pixel
  scribble carries no shape and costs thousands of `lineTo` calls.
  ⚠ The red spill ring is a **rendering** fact only: `_well_radius` is floored at
  `_MIN_WELL_RADIUS_PX`, so a floored radius can exceed the true one and a genuinely
  spilling print could read as fitting. The authoritative fit check is in mm.
* **Multi-select is opt-in, default OFF.** It changes what a click *means*, and the
  only other consumer (`fluorescence_mosaic_workflow`) selects exactly one well to
  scan. Brush-drag rather than a rubber band: it matches "drag click multiple
  wells", and a rubber band's 10 px overshoot on a 384 plate grabs a whole extra row.
* **The plate is the single selection surface.** The queue list is a read-only
  readout (`NoSelection`) and Remove acts on the plate's selection — two selection
  models for one fact is what this project keeps having to unwind. Rows are keyed by
  well NAME, never row index.
* **Write-back is ONE debounced sink**, connected once to all eleven snapshot
  widgets. Hooking `_on_object_changed` + `_on_settings_changed` + the four inline
  per-widget lambdas would be four seams and a fifth would be forgotten. It is
  **write-only** (never calls a widget setter, never calls `_apply_snapshot`), which
  is what makes the edit → write-back → refresh → edit cycle impossible, and
  `_on_well_clicked` **flushes** it before the active well changes so a pending
  120 ms debounce cannot land in the wrong entry.

---

## 🐞 Two pre-existing bugs fixed on the way in (both verified)

1. **`_run_group_job_blocking` discarded `pm.exec_logger.path`.** It nulled `_pm`
   without reading it, so after a **multi-ink** run `_on_multi_done` never called
   `_load_report` and `_last_log_path` still pointed at the *previous single print*
   — **"Open log" opened the wrong file.** It now returns the log path.
2. **`_ideal_path_mm` reads the live widgets and `_selected_well`.** After a queue
   run the report would have scored the last unit's executed trace against
   **whatever object happened to be on screen, in whatever well was selected** — a
   silently wrong accuracy number in a report whose entire purpose is credible
   accuracy. `_load_report` gained optional `ideal_pts=` / `context=` (defaults
   preserve the single-print behaviour) and each unit carries its own `ideal_pts`.

Also closed while in there: **`_on_print` had no `_cleanup_thread` guard**, so a
post-print cleanup could be running while Print was re-clicked, with only the
300 ms status tick standing between them.

**Report after a Run all:** the last *completed* unit's log is loaded (so an abort
mid-unit shows the previous unit's real report, not a truncated one), with
`context={"object", "well", "queue": "print 4 of 7"}` flowing into the existing
`QuickPrintReportPanel.load(..., context=)` — no panel change. `_queue_results`
retains per-unit `log_path` + `ideal_pts`, so a per-unit report selector is a
follow-up that needs **no model change**.

---

## Testing Notes

**126 new tests, all green** — `test_v719_print_queue_model` (35, no Qt, 1 ms) +
`test_v719_quick_print_queue` (91, offscreen).

**25/25 mutations CAUGHT**, sources verified free of residue afterwards:
glyph Y-flipped · glyphs keyed off `_current_well` · multi-select defaulting True ·
the glyph clip removed · the `selection_changed` equality gate removed · the ink map
assigned to `_ink_map` · the restore-the-active-snapshot `finally` removed · the
print floor checked once instead of per unit · the ink swap on every boundary
instead of on a CHANGE · the sticky-abort check before `pm.start()` removed ·
`_sequence_abort_requested` reverted to multi-ink only · `_hold_event.set()` missing
from abort · the print height resolved once outside the snapshot · an uncalibrated
well accepted (both guards, separately) · one shared snapshot stamped into every
well · the write-back timer unwired · the live trace fanned out to the Setup
preview · Apply gated on readiness · the switch-wells flush removed ·
`exec_logger.path` discarded again · `order_queue`'s ink mode collapsed to plate
order · `count_ink_swaps` charging the first unit · `from_dict` splatting unknown
keys · the glyph cache keyed by well.

### ⚠ Five of my own tests were too weak and the mutation run caught them first

Recorded because each is a trap this project keeps re-recording:

1. **The clip test used a DIAGONAL line**, which misses the neighbouring well's
   sampled row by a whole well pitch — so removing the clip SURVIVED. Rewritten with
   a horizontal line plus a guard-the-guard assertion that something was drawn in
   the source well at all.
2. **`_render` returned a `QImage` taken from a local `QPixmap`**, whose buffer is
   freed when it goes out of scope — so two "renders" compared freed memory. Now
   `.copy()`.
3. **Every write-back test called `_flush_write_back()` directly**, so they all
   passed with the debounce timer left unconnected — the exact weakness the v7.18
   setpoint keeper had. Fixed with an AST pin on the `timeout.connect` **plus** a
   behavioural test that lets only the timer fire.
4. **The `selection_changed` gate was unpinned**: the drag path has its own
   "already touched" guard, so a drag test alone cannot see the gate inside
   `_set_selection`. Added a direct same-set no-emit test.
5. **The hold tests were passing vacuously** — the fakes complete instantly, so the
   whole batch finished before the hold request landed. Replaced with a `_GatedPM`
   that blocks inside the first unit, and the tests now assert the worker parks
   **before** unit 2, retracts, is still alive, and that Resume finishes all three.

Two of my **mutations** were also wrong and proved nothing until fixed — a
`_build_settings(pump=…)` change that is a semantic no-op (the default already
resolves to the same pump inside the snapshot), and one that hit only the second of
two deliberate enforcement points. A mutation that does not change behaviour is not
evidence.

### Regression sweep

| Batch | Result |
|---|---|
| new suites + `test_v77_quick_print_zones` + `test_v731_jog_navigation` + `test_v712_custom_plate_rendering` + `test_v75x_fluorescence_mosaic` | **315 green** |
| all 12 `test_v75x_quick_print_*` | 182, **3 pre-existing failures** |
| `test_v79_cell_targeting_setup_page` + `test_test_suite_hygiene` + context-panel/responsive + `test_v75x_plate_location_workflow_toggle` + `test_v712_plate_builder_ui` | 355, **3 pre-existing failures** |
| `gui.app` import smoke | OK |

**All 6 failures PROVED not ours** by running them in a `git worktree` at HEAD
(never `git stash` in this repo — CLAUDE.md records why), where they fail
identically: `quick_print_workflow::test_settings_use_safe_z_and_flow`
(`_speed_pct_spin`, retired by v7.6) · `test_enabled_when_ready` (stale
`_readiness`) · `quick_print_pick_and_place::test_all_set_is_clean` ("Stage motion
not characterised") · `cell_targeting_setup_page::test_the_real_saved_profile…`
(a golden file the operator re-saved) · `plate_builder_ui::TestLearnLoopSavesToADesign`
×2 (the plate-type `max` z-offset from other working-tree WIP).

⚠ The working tree also carries substantial **concurrent v7.19 fluorescence work
from another session** (extensive edits to `fluorescence_mosaic_workflow.py`,
`hardware_setup.py`, `objective_calibration_card.py`, several new modules). Nothing
in this change touches those files; the mutation harness's byte-identity check
reported a mismatch purely because that session edited the same files mid-run —
verified by grepping for every mutation string afterwards (zero residue).

---

## Needs GUI / HW verification on ME3B V1, IN ORDER

The first two are the go/no-go — nothing else is trusted until they pass.

1. With an **empty queue**, run a single print exactly as today — unchanged.
2. Queue two wells with **different objects**; the plate glyphs match their
   objects; clicking each well reloads that snapshot into the editors.
3. Drag-select several wells → **Apply print to wells** → the count and the glyphs
   are right, and the confirm dialog names any wells it would replace.
4. **Run all**, one ink: one prep first, **no swap between wells**, one cleanup
   last; needle at safe Z between wells.
5. Two inks: the swap count matches the dialog, and **Group by ink** reduces it
   (the dialog states the counterfactual either way).
6. **Hold after well** → the needle retracts and parks → go run a small
   fluorescence mosaic → return → **Resume** finishes the batch.
7. Press **Resume while a scan is running** → refused, with a reason.
8. **Abort while held** → ends at safe Z, no hung thread, and the status says the
   needle still holds ink.
9. Delete a queued print's file on disk → its well is flagged red, Run all skips it
   **by name**, and the rest still run.
10. Queue a print wider than its well → the ring turns red.
11. Restart → the queue and both toggles came back.
12. **Open log** after a multi-print run opens *that* run's log (bug fix 1), and the
    report's accuracy numbers match the last unit's own geometry (bug fix 2).

## Issues & Decisions

* **`motion_mode` kept per-print** for faithfulness to "independent activity",
  against one reviewer's advice to make it global. The cost is real and disclosed:
  two units in different modes produce reports whose deviation numbers are not
  directly comparable, so the confirm dialog names the mode per unit.
* **`ink_padding_uL` added to the per-print set** (it was in neither of my first
  two lists): it feeds `_compute_pickup_volume_uL` and is the only lever against the
  fine-tip reserve warning, so a granular ink and an easy hydrogel genuinely need
  different values.
* **Deliberately NOT done:** a per-unit report selector (the data model already
  supports it — `_queue_results` retains everything needed, so it is pure UI); a
  process-wide stage lease to replace the poller-suspended proxy; modelling the
  wash/prep volumes in the cumulative syringe budget (the under-claim is stated in
  the dialog rather than hidden); and per-well **step size** or layer stacking
  (two prints in one well is a different, unbuilt feature — the queue enforces one
  print per well now precisely so that stays designable).
