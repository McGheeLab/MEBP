# MEBP v7.20 — Print Calibrator workflow (prime-time calibration)

## Objective

Add a **Print Calibrator** workflow page that helps the operator *tune print
settings by measuring them on the machine* instead of typing a guess. The first
(and, for now, only) calibration is the **prime time**.

Operator request:

> "i want to add a print calibrator workflow. this is a workflow page that help
> the user tune the print settings. One important calibration is the prime time.
> for this we should setup a print as normal, the user selects a well to
> calibrate in and offsets, everything that quickprint currently does. for the
> prime time calibration, the print should be a single line, and the printer
> should move at a set velocity from point A to B on the live view, the user
> selects point C which is the location where the print actually started. the
> distance from A to C divided by the velocity is the prime time needed"

### Why it matters

`build_well_plate_job` already dispenses a prime of
`flow_uL_s × HardwareConfig.pump_prime_time_s` as an `EXTRUDE`/`DISPENSE`
command in the `MOVE_Z → PRINT_PATH` gap, so material is already forming a bead
when the path starts. But `pump_prime_time_s` was a **hand-typed global**
(default 0.25 s) that nothing measured, while the quantity it stands for depends
on syringe compliance, needle bore/length and ink viscosity. Too small and the
first millimetres of every print are missing; too large and there is a blob at
the start.

Follow-up request, same session:

> "as a part of this calibration we should add a second line just below to
> calibrate the settings for mid print move up and reposition settings"

### The measurement

The pump command and the XY command start together, but ink does not emerge until
the compliance is taken up. During that delay τ the stage has already moved from
the commanded start **A** toward **B**, so the printed line begins at some point
**C** along it:

```
τ  =  prime_used_in_run  +  (distance A→C along the line) ÷ velocity
```

`prime_used_in_run` is in the sum on purpose — it makes one formula serve both
run kinds with no special case, and it is what makes the calibration *converge*:

* **Baseline** run forces the prime to 0, so τ is the absolute answer.
* **Verify** run uses the applied prime, and any residual A→C is what is still
  *missing*, so the corrected prime is `used + residual`. Re-running walks in.
* **Over-primed**: ink appears *before* A, the operator clicks behind it,
  `along_mm` is negative and the sum correctly *reduces* the prime (clamped at 0
  with a warning). A negative `along_mm` on an unprimed run is physically
  impossible, so *there* it is a refusal — the click is wrong.

### The second line — two different primes, one knob

**Operator's framing, which is the authoritative one:** *"the first line for prime
time is measuring the time needed to start the flow after the wash and clean
steps, the second line is measuring the time for the restart time after a pause
in the pump."*

A **second line is printed just below the first**, same direction, offset
perpendicular by a gap. They are two SEPARATE objects, so `build_well_plate_job`
inserts its lift → reposition → lower → re-prime between them — and while that
happens **the pump is stopped**, with
`PrintManager._print_pump_suckback("quick_move")` relieving the column on the way
out. So the two lines measure two genuinely different quantities:

| | |
|---|---|
| **τ₁ — the COLD START** (line 1) | flow starting after the wash and clean steps. The column is relaxed and may hold buffer or air ⇒ the **long** prime. |
| **τ₂ — the RESTART** (line 2) | flow restarting after the pump merely *paused*. The column is still largely pressurised ⇒ the **short** prime. |
| **τ₁ − τ₂** | how far apart they are. **Expect this to be positive.** |
| **max(τ₁, τ₂)** | what `pump_prime_time_s` must be, because it primes **every** segment including the first. |

`build_well_plate_job` gives one value to both, so there is a real trade and the
panel states it rather than leaving it to be discovered:

* at τ₁ (normally the max) every mid-print **restart over-primes** by τ₁ − τ₂ — a
  small blob where each line resumes;
* at τ₂ the **first line under-primes** — a missing start.

Taking the max is the safe error (nothing missing), and the cost is named.

⚠ **τ₂ > τ₁ is the UNEXPECTED case** and is flagged as such: a restart needing
more than a clean start means the *pause* is losing more pressure than the wash
and clean left behind, which points at the quick-move pressure relief or at how
long the pump is paused (the hop height and speeds) — not at the prime. The Apply
button says *(the RESTART binds)* and the dialog says where to look.

Because line 1 is **defined** as "after wash & clean", whether the prep sequence
actually ran is part of what the number means — so the provenance line states it
either way, and warns when prep was off.

---

## Decisions (with the operator, via AskUserQuestion)

1. **The calibrator HOSTS A LIVE `QuickPrintWorkflowPage` instance** — the repo's
   existing embedded-instance pattern (`full_print_workflow.py` thin-wraps
   `PrintingModePage`; the Spheroid page hosts a live Fluorescence Mosaic page).
   So needle prep, ink pickup, syringe-budget preflight, readiness checks, the
   print-floor confirm, the preposition→`PrintManager` launch, abort and
   post-print cleanup are **the same code** and cannot diverge. There is no
   second print-launch implementation.
2. **A and B are computed** from well centre + XY offsets + line length + angle,
   with a *click-to-place-the-line-start* that rewrites the offset spins (so the
   geometry stays computed, never hand-held).
3. **Apply writes straight to the global** `pump_prime_time_s` via
   `CommonPrintSettings`. No new calibration store; the provenance (velocity and
   its source, flow, prime used, pump, needle, ink) is *displayed* beside the
   result and named in the Apply confirmation.
4. **Multi-calibration shell**, prime time first — a calibration selector plus a
   per-calibration Measure panel is the extension seam.

---

## Files Modified

### New

| File | Why |
|---|---|
| `SupportClasses/PrimeTimeCalibration.py` | The arithmetic, pure (stdlib + a duck-typed `PrintLog`; no Qt/numpy/cv2): projection, the τ formula, the five refusals, the per-segment run reader, the hop cost, and the velocity/endpoint/prime-used/hop-settings readers for a finished run's exec log. |
| `gui/pages/workflows/print_calibrator_workflow.py` | The page: header + calibration selector, the calibration-line card (two lines + gap), a `1 · Print` / `2 · Measure` pill strip over a stack whose Print step **is** a live `QuickPrintWorkflowPage`. |
| `tests/test_v720_print_calibrator.py` | 145 tests (128 + 17 for the round-2 live-view line overlay). |
| `coding plans/Update plans/MEBP_v720_PRINT_CALIBRATOR_PRIME_TIME.md` | This document. |

### Modified

| File | Change |
|---|---|
| `gui/pages/workflows/quick_print_workflow.py` | Six additive embedding hooks (below) + the `preflow` field **linked** to the global instead of merely seeded from it. Nothing in the launch/print path changed. |
| `gui/widgets/camera_feed_view.py` | **Round 2:** new `set_stage_paths` + `_draw_stage_paths` — polylines in the bore-marker frame, drawn under the dots. Additive and change-gated; every other feed in the app stays at `[]`. |
| `gui/pages/workflows/workflow_picker.py` | `WorkflowTile(workflow_id="print_calibrator", icon="🎚️", …, enabled=True)` after Quick Print. |
| `gui/pages/workflows_mode.py` | Import + the `elif tile.workflow_id == "print_calibrator":` dispatch branch. |
| `CLAUDE.md` | Row in the *Existing Update Plans* table. |

---

## Implementation Steps

- [x] **1. `SupportClasses/PrimeTimeCalibration.py`** — `project_onto_line` →
  `LineFit(length, along, perp, frac)`; `evaluate(...)` → `PrimeTimeResult`;
  refusals `DEGENERATE_LINE` / `NO_VELOCITY` / `OFF_LINE` / `BEFORE_START` /
  `BEYOND_END`; a near-the-end **warning**; `resolve_velocity` (whole-log) and
  **`segments_from_log` → `SegmentRun` (per-segment, windowed)**;
  **`hop_cost` → `HopCost(first, second, extra, required, hop_binds)`**;
  `line_endpoints_from_log`, `prime_used_s_from_log`, `flow_uL_s_from_log`,
  **`hop_settings_from_log`** + `describe_hop_settings`.
- [x] **2. Quick Print embedding hooks** (all defaults = today's behaviour):
  `embedded` / `owns_camera` / `settings_id` / `settings_title` ctor kwargs;
  **`set_external_objects`** (a LIST → one segment per line, so the hop is
  inserted between them) + `set_external_object` wrapper + `external_object(s)`
  (+ the `_EXTERNAL_DATA` combo token and the one
  `_path_segments_for_selection` branch); `set_prime_override_s` /
  `prime_override_s` / `set_preflow_s`; `print_state_changed` +
  `cleanup_finished` signals; the read accessors
  (`resolved_print_kinematics`, `selected_well`, `well_center_zero_ref_mm`,
  `well_radius_um`, `pump`, `prime_default_s`, `last_log_path`, `is_printing`,
  `cleanup_running`); `_hide_queue_surface()`.
- [x] **3. `preflow` → `add_common(common_key="pump_prime_time_s")`** — see
  *Issues & Decisions* #1. Without this the whole feature is a no-op for Quick
  Print.
- [x] **4. The Print Calibrator page** — line card (length / angle / offset X,Y /
  **second line + gap** / run mode / step-after-run), pill strip, embedded Quick
  Print, Measure step (live feed + A1/B1/A2/B2/C markers + go-to per line +
  per-line mark buttons + both results + the hop cost + Apply).
- [x] **5. Registration** — tile + dispatch.
- [x] **6. Tests** — 100, plus a mutation matrix.
- [ ] **7. Bench verification** — see *Testing Notes*; needs real hardware.

---

## Design notes worth keeping

### A and B are read back out of the run's execution log

`PrintLog.ideal_points()` returns `path_start.points` — *the path the executor
was handed* — so A and B come from there, not from the calibration row's own
length/angle/offset arithmetic. That makes a desync between "what printed" and
"what is being measured" structurally impossible. A pre-v7.7 log recorded only
`n_points`; then the page falls back to its computed endpoints **and says so** in
the provenance line.

### Velocity is resolved PER SEGMENT, and the source travels with the number

1. `vel_sample` rows (`s_mm` = the **actual** position projected onto the path) →
   `measured`.
2. `openvel_sample` rows (`s_mm` = the time-paced **commanded** target) →
   `commanded-schedule`.
3. `settings["print_speed_mm_s"]` → `commanded`.

⚠ **Per segment, not per log** — this is a hard requirement of the two-line
feature, not a refinement. Each `PRINT_PATH` restarts `s_mm` at 0, so a whole-log
first-to-last span crosses the reset *and* swallows the inter-segment hop. On the
suite's two-line fixture the whole-log answer is **0.5 mm/s against a true
3.0 mm/s** — a 6× error that would multiply straight into every prime time.
`segments_from_log` therefore windows each segment's samples to its own
`[path_start.t, path_end.t]`, and a test pins both the correct per-segment answer
*and* that the whole-log helper is the one that gets it wrong.

⚠ `path_end.t − path_start.t` is deliberately **not** used: on the discrete path
`path_start` is logged *before* the move-to-`points[0]` and its 5 s settle, so
that window over-states the traverse, under-states the velocity and over-states
the prime — a blob at A, i.e. exactly the fault this calibration exists to
remove. A test pins that the resolver does not pick it (the synthetic log is
built so the contaminated answer would be 0.4 mm/s against a true 3.0).

**Deliberately not done, recorded not overlooked:** interpolating the sample time
at `s = along_mm` rather than dividing by an average velocity. That is strictly
more accurate across the initial accel ramp — which is exactly where C lands —
but it is a second, differently-derived number for the same quantity, and it is
not the formula the operator specified. If the ramp turns out to matter on the
bench, this is the refinement to make.

### What the pump was paused FOR is read from the run, not the widgets

`hop_settings_from_log` reads `intra_well_hop_z_mm` / `line_move_z_speed_mm_s` /
`line_move_xy_speed_mm_s` out of the log's stamped `PrintSettings`. Those three
set how **long the pump is paused** between the lines, and therefore how much
pressure is lost before the restart — so they are exactly what to reach for when
τ₂ comes out too high. They are read from the log rather than the live widgets
because the operator may well have nudged a spin since the run, and a provenance
line reporting the *current* values would attribute the measurement to settings
that never ran.

⚠ This replaced a `hop_settings()` accessor on `QuickPrintWorkflowPage`. Reading
the log is the better source regardless, but the immediate reason for the switch
is recorded in *Issues & Decisions* #12: something on the machine was
concurrently saving over that file and reverting the addition.

### The line is clamped to the well, and the clamp is always stated

`r_safe = well_radius − needle_radius − 0.25 mm`. The length is clamped so both
endpoints stay inside it. Clamping can only *shorten* the line — and a shorter
line may fail to contain the ink start — so a silent clamp would be a silently
different measurement; the card says `⚠ Line clamped to 3.20 mm (from 20.00 mm)
to stay inside C4`. An offset that puts the line outside the well is **refused,
not clamped**, and then no object is pushed at all, so the embedded page's own
readiness blocks Print and names the reason.

### The stage does not stay where the print ended

On a clean completion Quick Print launches the post-print cleanup immediately,
which travels waste → wash → oil. So the calibrator never assumes the live view
still shows the line: `cleanup_running()` / `cleanup_finished` gate the go-to
buttons, and the operator drives back with an explicit **⤵ Go to line start (A)**
(`SafeTravelWorker` with `target_z_mm=None` → retract, travel, **never lower**).

### ⭐ Round 2 — the line is DRAWN on the live view, never placed from it

Operator: *"we should not be setting the locations of the line start and end,
those should just be overlays on the live view that live at some xy location, and
the print path line between the calibration line should overlay on the live view
as well, as we move the live view around the overlay matches to what should be
there."*

The first cut had a **⌖ Set line start from view** button: arm it, click the live
view, and `_place_line_start` back-solved the clicked point into the Offset X/Y
spins. It is gone. There are now **two directions a location can flow, and only
one of them is allowed**:

* **spins → view (allowed).** The line's position is computed from Length / Angle
  / Offset and *drawn* where it sits on the plate.
* **view → spins (removed).** A click could rewrite the geometry, which makes the
  printed line and the on-screen line two different objects with no single
  authority. A click can now do exactly one thing: record **where the ink
  started** (C). Mutation-pinned, including the retired arm string, so re-adding
  the affordance fails a test.

**And the line itself is now drawn, not just its endpoints.** New
`CameraFeedView.set_stage_paths(paths)` strokes polylines in the **same
camera-centre-relative µm frame as `set_bore_markers`** — deliberately one frame,
because the path, the A/B dots at its ends and a click on it must be registered
with each other; a second convention here is a sign error waiting to happen. Two
overlays, answering different questions:

* **DASHED** (`overlay1`) — the planned pair, i.e. where the **next** run will
  print. This is what *replaces* clicking to place the line: nudge Offset X/Y and
  watch it move over the real glass. It is also how the operator confirms
  `_step_offset_for_next_run` actually stepped onto clean glass.
* **SOLID** (`teal`) — the pair the last run **commanded**, read back out of its
  own exec log, with A/B/C dots at the ends.

Both are pushed relative to the **live** stage position and re-pushed on the
~300 ms tick, so jogging slides them across the frame and they stay on the glass.
That is not cosmetic: **the drawn stroke landing on the real bead is itself the
check that the camera↔stage frame is right** — the same claim the bore-offset
wizard makes, now free on every measurement.

⚠ **No bounding-box cull, deliberately.** The bore dots skip a marker whose
projected point is off-frame, which is right for a *point* and wrong for a *line*:
the case that matters most is a line whose endpoints are both outside the view
and whose middle crosses it. A clip rect does the work instead (it also bounds a
very long line at a small µm/px). Mutation-pinned by a render test with both
endpoints 5000 px outside a 640×480 frame, plus a guard-the-guard test that a
fully off-frame path paints nothing.

⚠ Paths are drawn **under** the dots, since the dots label their ends —
mutation-pinned, because "draw order" is exactly the kind of thing a later edit
tidies.

⚠ **One of my own mutations was a no-op and proved nothing until I fixed the
production code:** dropping `pen.setStyle(DashLine)` left the overlay dashed
anyway, because `setDashPattern` *implicitly* sets `CustomDashLine`. Two
enforcement points for one fact is what made the mutation harmless, so the
redundant `setStyle` was deleted and the mutation re-aimed at `setDashPattern`.
Same lesson this file already records about partial mutations.

⚠ Render tests go through `_render_frame` (the real paint chain), **not**
`_draw_stage_paths` directly — calling the helper by hand would leave "the
overlay is never wired into the paint chain" invisible, and a mutation proved
exactly that.

`COLORS` carries neither `overlay1` nor `teal`; both are Catppuccin Mocha values
and two existing sites already reach for `teal` with this same literal fallback,
so the palette was left untouched rather than widened for one page.

### Markers use `set_bore_markers`, not `set_reference_markers`

`set_bore_markers` takes camera-centre-relative µm and routes through
`stage_offset_to_pixel` — the exact inverse of the click path — so A/B/C land
correctly on a rotated or mirrored camera. `set_reference_markers` uses a naive
`(x − stage)/µm_per_px` divide (a flaw its own siblings document) and would
mis-place every marker on this rig's camera. Mutation-pinned.

### Clicking C

`CameraFeedView.clicked` (RAW frame px) → `CameraManager.pixel_to_stage_offset`
(fed the **live** `image_size`, because µm/px resolves against the frame width) →
`+ controller.get_xy_position(cached=True)`, which **already returns µm**
(`get_xy_position_mm` is the one that divides by 1000 — the ×1000 here was a
documented 1000× bug that drove the stage into the envelope corner). The stage is
read on every click, so the operator may jog freely between clicks and A may be
off-screen.

---

## Issues & Decisions

1. **🔴 Applying the measurement would have changed nothing — found by design
   review, confirmed in source.** Quick Print's pre-flow spin was
   `sec.add("preflow", …)`, seeded from `_prime_default_s()` **once**, at
   dialog-build time — when `_hw_config` is still `None`, so it took the 0.25 s
   fallback — and then had the saved profile applied on top. So
   `CommonPrintSettings.set("pump_prime_time_s", τ)` reached every workflow that
   *links* that key (`cell_labeling`, `cell_targeting`, `spheroid_pickup`) and
   silently did **not** reach Quick Print, the primary consumer: the calibration
   would have appeared to succeed and changed nothing. Fixed by making it
   `add_common(..., common_key="pump_prime_time_s", overridable=True)` — the same
   convention the sibling `g_settle` row already uses. `overridable=True`
   preserves the per-run knob the code comment describes (unchecked *inherits*
   the global live, and because an inheriting widget is force-set, `_preflow_s()`
   already returns the effective value), and the dialog's `_migrate_common_links`
   promotes an existing customised profile value to an explicit override, so
   saved profiles keep the prime they were saved with.
2. **The camera lifecycle must be *declared*, not won.** A `QTabWidget` hides the
   inactive tab, so an embedded page receives a `hideEvent` whenever the operator
   looks elsewhere — and its `hideEvent` calls `_stop_camera()`. Worse, Qt
   delivers `showEvent` to a **child before its parent**, so an embedded instance
   would always win the start race and thereby claim the stop. Hence a separate
   `owns_camera` flag (the shape `FluorescenceMosaicWorkflowPage` already uses),
   with the host owning start/stop for the whole shell. ⚠ **My first cut of
   `_start_camera` early-returned *before* `set_camera(cam_idx)`**, so the
   embedded view would never have re-bound to the right microscope slot — the
   bind is now unconditional and only start/stop is ceded (mutation-pinned).
3. **Two live instances would clobber one settings store.**
   `WorkflowSettingsDialog.save_last()` fires on the popout's `hideEvent`/
   `closeEvent`, Quick Print hides its popout on *its* `hideEvent`, and all
   workflow pages are constructed eagerly at startup — so a shared
   `"quick_print"` id would let the calibrator silently overwrite the operator's
   real last-used values (including, post-v7.19, the whole plate queue). Hence
   `settings_id="print_calibrator"` → `config/workflows/print_calibrator/`.
   Rejected alternative: a `persist_settings=False` flag — it would skip
   `load_last()` too, resetting the calibrator to factory defaults every session.
4. **The v7.19 plate queue is hidden when embedded, not merely disabled.**
   `QueuedPrint.object_data` stores the object combo's userData as a
   *reference*, so a host-supplied object would persist as a dangling one; the
   per-well snapshot write-back re-applies stored widget values on every plate
   click, which would overwrite the calibration line; and "Apply print to wells"
   / "Run all queued" would stamp a one-off calibration print across the plate.
   Hiding removes all three without touching queue logic (every widget stays
   alive, so the queue's own methods are untouched), and the helper is
   getattr-guarded so it survives the queue being renamed or moved.
5. **`print_state_changed` is emitted LAST in `_on_state`**, so a host slot sees
   `_pm` cleared, `_last_log_path` set, live sampling stopped and any cleanup
   worker already started. It is documented as **re-entrant**
   (`PrintManager._set_state` runs on the calling thread and `pm.start()` is
   called from the GUI thread), so the calibrator's slot only reads state and
   repaints.
6. **Two `PrintManager`s / one `StageController`.** `_is_running()` is per-page
   and `suspend_position_poller` is not refcounted, so the calibrator gates its
   own motion on `controller.is_position_poller_suspended()` — the v7.19
   accessor documented as a *proxy* for "another sequence is driving the stage".
   It is a proxy, not a lease; residual risk stands and is disclosed.
7. **One jog panel.** `get_context_widget()` delegates to the embedded page's
   `StandardJogContextPanel` rather than building a second one — the operator
   needs to jog while hunting for the ink start, and the embedded page already
   builds *and ticks* exactly that widget.
8. **🐞 Found by my own tests:** `prime_used_s_from_log` / `flow_uL_s_from_log`
   assumed `PrintLog.settings` is a dict, but it is `manifest.get("settings") or
   {}` — which passes a corrupt log's non-dict straight through and raised
   `AttributeError`. One `_settings_of(log)` guard now serves all three readers.
9. **Prime is not the only pump take-up — disclosed, not solved.** Even at
   prime 0, where ink first emerges also depends on the ink pickup's
   backlash/compliance compensation, `_print_pump_suckback`'s per-pump
   `pump_relief_uL`, and the needle dead-volume reserve. So a prime time measured
   with one preamble configuration is not transferable to another. The
   provenance line and the Apply dialog name the run's pump, needle, ink,
   velocity, flow and prime-used, and the dialog says to re-measure after
   changing any of them. Deliberately **not** auto-refused on a mismatch: with no
   calibration store there is nothing to compare against, and inventing a
   fingerprint here would be a second home for a fact the operator can see.
10. **⚠ The design-review agent reported the file "cannot construct as checked
    out"** because of the in-flight v7.19 queue work. **Checked, and it is
    wrong** — the page constructs, and its object combo populates normally. The
    review had read the file mid-save (it changed under me during this session
    too). The genuine residual from that finding is #4, which is fixed.
11. **⚠ This branch carries substantial uncommitted work in
    `gui/pages/workflows/quick_print_workflow.py`** (the v7.19 plate queue,
    +~1500 lines) and in eight other files. Every edit here is additive and was
    applied by exact-match; the mutation harness backs up and restores by SHA-256
    rather than going anywhere near `git`. **Do not `git stash` / `git restore`
    in this repo** — the documented v7.17 incident.
12. **⚠ SOMETHING WAS CONCURRENTLY SAVING OVER `quick_print_workflow.py` DURING
    THIS WORK.** The file changed under me repeatedly (its line count moved and
    `_build_header` shifted by ~45 lines mid-session), and a `hop_settings()`
    accessor added to it was silently reverted **three times** — an editor buffer
    saving over the file. Every other edit to that file survived and is verified
    present; the accessor was dropped entirely in favour of reading the hop
    settings from the exec log, which needs no change to the contested file and is
    the better source anyway. **Anyone continuing this work should re-verify the
    v7.20 markers in that file before editing it** (`grep -c 'v7.20'` — expect 18
    or more) and be aware that an edit may need re-applying.
13. **The mutation harness needed two fixes before it proved anything**, both
    worth recording: (a) hardcoded anchors had drifted, so it now VERIFIES every
    anchor against the live source and refuses to run otherwise; (b)
    `quick_print_workflow.py` is **CRLF** while the two new files are LF, so
    multi-line anchors written with `\n` matched nothing when compared against
    raw decoded bytes — the harness now translates each anchor to the file's own
    newline. A harness that silently skips is worse than one that fails loudly.
14. **⚠ MY FIRST CUT OF THE SECOND LINE HAD THE SEMANTICS BACKWARDS.** I framed
    it as "the mid-print hop costs an EXTRA τ₂ − τ₁", i.e. I assumed τ₂ > τ₁ and
    attributed the whole difference to the Z lift and XY move. The operator
    corrected it: line 1 measures **starting flow after the wash and clean
    steps**, line 2 measures **restarting after a pause in the pump**. That
    reverses the expected direction (a paused column is still pressurised, so
    τ₂ < τ₁), which matters because the old wording would have reported the
    ORDINARY case as a surprise ("ink restarted SOONER… the residual pressure
    survived") and the case genuinely worth investigating as ordinary. Renamed
    `HopCost` → `PrimeComparison` and `hop_cost` → `compare_primes` with
    `initial_s` / `restart_s` / `difference_s` / `restart_binds`, re-worded every
    message, relabelled the checkbox / mark buttons / result rows, and added the
    prep-state provenance (line 1 is *defined* as "after wash & clean", so
    whether prep ran is part of the meaning). The hop settings are still
    reported — they set how LONG the pump is paused, so they are what to reach
    for when τ₂ comes out too high — but they are no longer presented as the
    cause of the whole difference.
15. **🐞 One of my own tests HUNG instead of failing, and a mutation found it.**
    `test_apply_does_nothing_on_a_refusal` called `_on_apply()` without patching
    `QMessageBox`, relying on the refusal guard to return early — so when a
    mutation removed that guard, the test opened a REAL modal and blocked
    forever offscreen (this repo's documented "modals block forever offscreen"
    trap) instead of reporting a failure. It now patches the modal and asserts it
    was **never called**, which is the stronger claim anyway.

---

## Testing Notes

`tests/test_v720_print_calibrator.py` — **128 tests, all green.**

* **The second line** — two objects become two print segments (so the pump pause
  happens between them), parallel and offset by the gap; one line when the
  checkbox is off; a second line that would leave the well is **dropped with a
  named warning** (only one line prints, so the restart cannot be measured);
  `compare_primes` gives the difference, takes the **max** as required, states the
  over-prime cost in the expected direction, **flags the unexpected direction**
  (restart > cold start) and returns `None` unless BOTH lines measured; each line
  uses its **own** velocity; one refused line does not throw away the other, but
  the comparison stays absent; the result rows **name which line is which**;
  marking targets the armed line; the second Go-to button targets line 2 and is
  disabled with only one line; Apply uses the larger value, the dialog states what
  the choice costs, and the button says *(the RESTART binds)* only in the
  unexpected case; the step **clears the whole pair** (a bare 1 mm step at a 1 mm
  gap would print the next run's line 2 on this run's line 1); the provenance
  **states whether prep ran**, because line 1 is defined as the cold start after
  it.

* **Pure geometry** — along/perp at 0/37/90/143/180/−90°, a translated frame,
  perpendicular vs Euclidean, degenerate and malformed inputs, the off-line limit
  scaling with length.
* **The formula** — the baseline case; `prime_used` **added** (0.30 + 0.12 =
  0.42); an over-primed verify run reducing the prime; the clamp at 0 with a
  warning; every refusal, including that `BEFORE_START` fires only when no prime
  was used; the near-the-end warning; and that a refusal carries **no** number
  (so a caller that forgets to check cannot apply one).
* **Log reading** — `vel_sample` beats `openvel_sample` beats `settings`; the
  settle-contaminated window is not used; a single sample / zero dt / zero ds are
  rejected; endpoints from `path_start.points` and `None` without them;
  `prime_used = volume ÷ rate`; `None`/garbage never raise.
* **Quick Print hooks** — the external object is selected, locked, yields exactly
  one 41-point segment, and **survives a `_refresh_objects()`** (which runs on
  every `showEvent`); the stored dict is copied not aliased; a page with no
  external object is byte-identical; the prime override zeroes
  `prime_amounts_uL` and disables the spin; `embedded` drops the Back button but
  keeps ⚙ Settings; `owns_camera=False` still **binds** the feed and never stops
  it; the settings store is isolated; the queue is hidden when embedded and kept
  when not; the terminal signals fire with `last_log_path()` already readable.
* **The prime link** — the field is registered as a common link on
  `pump_prime_time_s`, and setting the global reaches `_preflow_s()` and the
  stamped `prime_amounts_uL` (issue #1).
* **The page** — the line is pushed to the embedded page; geometry centred on the
  offsets; the well clamp fires **and says so**; an impossible offset is refused
  and pushes no object; unknown well size applies no clamp; run mode drives the
  override; the offset steps perpendicular.
* **The measurement flow** — a synthetic exec log → `_capture_run()` → a clicked
  C → τ; A/B taken from the **log** even when the page's own arithmetic would
  disagree; a verify run adding its prime; off-line and past-B refusals leaving
  Apply disabled; the geometry fallback naming itself; COMPLETED advancing to
  Measure and ABORTED not.
* **Click → stage** — `pixel_to_stage_offset` called with the **live** frame size
  and no ×1000; arming is one-shot; set-line-start rewrites the offsets.
* **Markers, travel, Apply, contract, registration** — as described above.

### Mutation matrix

32 mutations, each breaking exactly one fix; every anchor is verified unique
against the live source before the run starts, and sources are restored and
verified byte-identical by SHA-256 (never `git` — this tree carries uncommitted
work).

⚠ **The first full run had to be discarded and re-run.** I edited one of the three
target files *while the harness held a byte snapshot of it*, so the restore
reverted my edit and every mutation after that point ran against a suite with one
unrelated failing test — inflating the "caught" counts and turning the
post-restore check RED. Four mutations had shown only *1 failing* and were
therefore unproven. The re-run is clean. **Do not edit the target files while the
harness runs.**

What each mutation proves:

| | |
|---|---|
| M1 | the prime override ignored ⇒ the run measures the residual, not the prime |
| M2 | `preflow` back to a plain field ⇒ Apply reaches nothing (issue #1) |
| M3–M4 | the external entry not re-selected after a rebuild / the geometry branch removed |
| M5–M6 | the camera stop no longer ceded (host feed dark) / the bind skipped (wrong slot) |
| M7 | the settings store id shared again ⇒ Quick Print's profile clobbered |
| M8 | the terminal signal not emitted ⇒ the host never learns a run finished |
| M9 | the plate queue left visible when embedded |
| M10 | external objects collapsed to one ⇒ **no hop between the lines** |
| M11 | `prime_used` dropped from the sum ⇒ a verify run never converges |
| M12–M14 | each refusal removed in turn |
| M15 | velocity no longer prefers the measurement |
| M16 | **the per-segment window removed ⇒ the velocity crosses the `s_mm` reset** |
| M17–M18 | `required_s` takes the cold start instead of the max / the comparison returns a number on a refusal |
| M19 | the unexpected direction (restart > cold start) not flagged |
| M20 | the restart over-prime cost not stated |
| M21 | A/B recomputed instead of read from the log |
| M22 / M29 | the line clamp / the dropped-second-line warning made silent |
| M23 | the go-to lowers the needle |
| M24 | markers use the naive reference-marker projection |
| M25 | the busy-needle guard removed from the go-to |
| M26 | an impossible line clamped instead of refused |
| M27 | the second line not offset ⇒ it prints on top of the first |
| M28 | Apply uses one line instead of the required max |
| M30 | the prep state not reported ⇒ line 1's meaning is undefined |
| M31 | the step no longer clears the pair ⇒ the next run prints over this one |
| M32 | the result rows stop naming which line is which |

**Result: 33/33 caught, 0 survived, 0 hung**, sources restored byte-identical.

⚠ **TWO SURVIVED on the first clean pass, and both were real weaknesses in my
own work — the run earned its keep:**

* **M26 exposed genuinely unreachable code.** `_line_geometry_mm` had three
  guards, and the third (`h_max <= 0`) *cannot fire*: `off² = proj² + perp²`, and
  the first guard already established `off < r_safe`, so
  `sqrt(r² − perp²) > |proj|` strictly. No test could reach it, which is exactly
  why the mutation survived. Rather than write a test for an impossible branch,
  the branch was **deleted** (one enforcement point, not three — the v7.18.1
  lesson) and replaced with the guard it was groping at, which IS reachable: an
  offset just inside `r_safe` leaves room for only a sliver, and clamping down to
  a 0.15 mm "line" would print something too short to read an ink start in, to be
  rejected later as degenerate with nothing pointing at the offset. Now refused
  at `MIN_USABLE_LINE_MM`, with a companion test on the *other* side of the
  threshold so the guard cannot pass by refusing everything, and a second
  mutation (M26b) pinning the threshold itself.
* **M32 was a vacuous assertion.** `test_the_result_names_which_line_is_which`
  asserted that `"wash & clean"` and `"pump pause"` appear in the result text —
  but `compare_primes`' own message contains both phrases, so stripping the
  per-line row labels left the test green. Rewritten to assert the full row
  labels (`"Line 1 — flow start after wash & clean"`).

#### Round-2 matrix — the live-view line overlay

A separate 13-mutation run, same discipline (anchors verified unique against the
live source first — a stale anchor *skips* and looks like a pass — sources
restored and re-read in a `finally`). **13/13 CAUGHT.**

| # | Defect re-introduced | Caught by |
|---|---|---|
| M1 | the calibrator never pushes any path | the printed stroke / planned line tests |
| M2 | the **planned** (dashed) line is not drawn | `test_the_planned_line_is_drawn_dashed_before_anything_is_printed` |
| M3 | stage subtraction sign flipped → the overlay **rides the camera** instead of staying on the glass | `test_the_overlay_tracks_the_stage` |
| M4 | the planned line drawn solid → planned and printed indistinguishable | the dashed/solid partition tests |
| M5 | only the first line of a pair drawn | `test_both_lines_of_a_pair_are_drawn` |
| M6 | **a click can move the line again** (the retired affordance) | `test_a_click_can_NEVER_move_the_line` |
| M7 | `set_stage_paths` change-gate removed → a repaint per tick | `test_an_identical_push_does_not_rerender` |
| M8 | a degenerate 1-point "path" kept | `test_garbage_and_degenerate_paths_are_dropped_not_fatal` |
| M9 | **per-point off-frame cull** (the bore-dot rule, wrong for a line) | the both-ends-outside render test |
| M10 | paths unwired from the paint chain | the render tests (they go through `_render_frame`) |
| M11 | paths drawn with the naive divide, not the calibrated inverse | `test_the_drawn_line_lands_where_the_CALIBRATED_map_puts_it` |
| M12 | the dash pattern dropped | `test_dashed_paints_less_than_solid` |
| M13 | paths drawn **over** the dots, painting out their labels | the draw-order render test |

⚠ **M12 initially SURVIVED as a no-op and proved nothing** — see the round-2
design note: `setDashPattern` implicitly sets `CustomDashLine`, so removing the
redundant `setStyle(DashLine)` left the line dashed. The production redundancy was
deleted and the mutation re-aimed at the line that actually controls dashing.

⚠ **M11 needs a rotated + mirrored fake manager** (the `_OrientedMgr` shape
borrowed from `test_v713_bore_dot_overlay`): on an unrotated camera the naive
divide and the calibrated inverse agree exactly, so an axis-aligned stub cannot
tell them apart. The test samples a **quarter** point, not the midpoint — these
endpoints are symmetric about the frame centre, so both maps agree there — and
asserts up front that the two predictions differ by >30 px, so it cannot pass
because nothing is being distinguished.

### Regression

Run per-suite (this repo's convention): the Quick Print suites, the
common-print-settings and pump-settle/prime suites, the workflow-tile contract
suite, plus a `gui.app` import smoke and an offscreen build of the real page and
of `WorkflowsModePage`.

**Pre-existing failures — PROVED not ours in a clean `git worktree` at committed
HEAD** (never `git stash` in this repo). All three reproduce identically there,
with neither this change nor the in-flight v7.19 queue work present:

| Test | Why it fails |
|---|---|
| `test_v75x_quick_print_workflow::TestJobBuilding::test_settings_use_safe_z_and_flow` | Drives `page._speed_pct_spin`, a knob **v7.6 retired** — already recorded in CLAUDE.md as a stale test. |
| `test_v75x_quick_print_workflow::TestButtonGating::test_enabled_when_ready` | `_update_button_state` reads the readiness model, and the test sets `_plate`/`_selected_well` directly without recomputing it, so the *stale* model blocks with `Plate: no plate; Well: none chosen`. |
| `test_v76_two_param_and_overlays::TestLimitWarnings::test_no_warning_when_within_limits` | Its `_Page` **overrides `__init__` without calling super**, so the ctor (and therefore `add_common`) never runs; it exercises `_append_limit_warnings`, untouched here. |
| `test_v75x_quick_print_pick_and_place::TestSetupStatus::test_all_set_is_clean` | Fails on `⚠ Stage motion not characterised` — this machine's `print_timing_calibration.json` has no measured stage characteristics. **Named with this exact cause in CLAUDE.md's v7.17 entry** as already proved pre-existing. |

Green: `test_v75x_common_print_settings` + `test_v75x_pump_settle_and_prime_time`
+ `test_v718_incubator_gui` (89), and the Quick Print job-building batch
(`test_v77_quick_print_zones`, `_pick_and_place`, `_multi_ink`, `_travel_split`,
`test_v75x_multi_object_print_seam` — 137, the one failure above excepted).

**Round 2** added a sweep over the other `CameraFeedView` consumers, since
`set_stage_paths` touches the shared paint chain — **616 green** in three batches:
calibrator + bore-dot overlay + live-view-vs-measured orientation + square crop
(265) · mount square-up + camera-rotation-cal + bore wizard + gui-watchdog (194) ·
the v7.19 Quick Print queue + zones + suite hygiene (157), plus a `gui.app` import
smoke.

And an **offscreen end-to-end smoke through the REAL page and the REAL
`CameraFeedView`** (not a `MagicMock` feed — a stand-in that accepts the push
proves nothing about the widget): both planned lines arrive dashed in
`feed._stage_paths`, a simulated finished run adds both printed strokes solid with
`A1/B1/C1/A2/B2` dots, a +1 mm jog in X moves **every** overlay point exactly
−1 mm in the frame (so it stays on the glass), and 320 teal pixels reach the
rendered pixmap through the full paint chain.

---

## Bench verification — needs real hardware, IN THIS ORDER

1. Workflows → **Print Calibrator** opens; the embedded Quick Print surface is
   fully usable (well click, pump, ink, prep, height, speed, readiness) with **no
   duplicate Back button**, no plate-queue card, and the object locked to
   **⟂ Calibration lines ×2 — N.NN mm + hop**.
2. The trajectory monitor draws **two parallel lines** inside the chosen well;
   change length / angle / offset / gap and watch them move together. Pick a small
   well and confirm the **clamp message** appears; raise the gap until the second
   line no longer fits and confirm it is dropped **with the warning**.
3. **Baseline run** (prime forced to 0) — confirm in `logs/prints/<run>.jsonl`
   that the prime `DISPENSE` is absent or zero, and that there are **two
   `path_start` events** with a `MOVE_XY {hop_z: …}` between them. Both lines
   print; the page auto-advances to **Measure** and the offset steps for the next
   run.
4. Under the microscope: line 1 (the cold start after wash & clean) should
   visibly **start late**, and line 2 (the restart after the pump pause) should
   start **sooner** — the column was still pressurised. Press *◎ Mark line 1
   (cold start)*, click it; then *◎ Mark line 2 (restart)*, click that. The panel
   reports τ₁, τ₂ and the difference. Sanity-check both reported velocities and
   their source against the speed you set — **they should be nearly equal**; a
   wildly different line-2 velocity means the per-segment windowing is wrong.
   Also check the provenance line says prep **RAN** (if it says it did not, τ₁ is
   not a cold start).
5. Click deliberately **off** a line → that line is refused while the other keeps
   its result. Click **past B** → refused with "print a longer line".
6. **Apply** → the button names `max(τ₁, τ₂)` — normally τ₁ — and the dialog
   states that every restart will over-prime by τ₁ − τ₂. Hardware Setup → Pump's
   *Prime time* shows the new value, Common Print Settings agrees, and the
   calibrator's own Pre-flow lead-in shows it too (issue #1 — if it does not, the
   link is broken and the calibration is inert). Restart and confirm it persisted.
7. **If τ₂ came out LARGER than τ₁** (the button says *the RESTART binds*), that
   is the wrong way round: shorten the pause — halve the hop height or raise the
   hop speeds in the ⚙ settings — or look at the quick-move pressure relief, then
   re-run and confirm τ₂ drops below τ₁.
8. **Verify run** at the stepped offset — both lines should now start **at A**.
   Any residual clicks to a small Δ that **adds** to the prime; re-apply and
   confirm it converges.
9. Over-prime deliberately (double the value), run, and confirm a blob at A and
   that clicking behind A reports *over-primed; reduce* rather than refusing.
10. **The round-2 overlay, and it doubles as the rotated/mirrored-camera check.**
    Before printing anything, look at the live view: a **dashed** line shows where
    the next run will print. Nudge **Offset X/Y** and confirm it slides over the
    glass in the direction you expect; confirm there is **no way to click the view
    and move it** (that affordance was removed on purpose). After a run, the
    **solid** stroke should lie **on the actual bead**, with A1/B1/A2/B2/C dots at
    its ends. Then **jog the stage well away and back**: the overlay must stay
    stuck to the glass, not ride along with the camera.
    ⚠ **A solid stroke that is parallel to the bead but offset, or rotated away
    from it, is a camera↔stage calibration fault, not an overlay bug** — re-run
    the objective µm/px + orientation calibration before trusting any prime time
    measured through it, because the same projection converts the C click.
11. With post-print cleanup ON, confirm the go-to buttons stay disabled until the
    cleanup finishes, then *Go to line 1 start* retracts and travels **without
    lowering the needle** (watch the needle, not the screen). Check *Go to line 2
    start* lands on the second bead.

---

## Status

- [x] Pure calibration module (projection, τ, refusals, per-segment runs,
      `compare_primes`)
- [x] Quick Print embedding hooks (+ the prime-link fix that makes Apply real)
- [x] Print Calibrator page, two lines (cold start + restart) with the comparison
- [x] Line-2 semantics corrected to the operator's framing (issue #14)
- [x] Registration (tile + dispatch)
- [x] 128 tests green
- [x] 4 pre-existing failures proved at committed HEAD in a worktree
- [x] Regression **364 green** in one run (this suite + quick-print-zones +
      common-print-settings + pump-settle/prime + incubator-gui + the operator's
      in-flight `test_v719_quick_print_queue`), plus the Quick Print job-building
      batch (137) and a `gui.app` import + offscreen page walkthrough
- [x] Mutation matrix — **33/33 caught, 0 survived** (clean re-run after the
      contaminated first attempt; the 2 first-pass survivors were real weaknesses
      in my own work and are recorded above)
- [ ] Bench verification on ME3B V1 — the checklist above, in order
