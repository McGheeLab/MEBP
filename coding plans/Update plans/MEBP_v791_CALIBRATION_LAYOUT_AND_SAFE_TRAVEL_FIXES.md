# MEBP v7.9.1 — five operator-reported defects

**Branch:** `Version-7.9.1`

Reported together on 2026-08-10, four of them on the same rig (ME3B V1). One is
a hardware-safety defect: a "retract" that drove the needle **down onto the
plate**, arrested only by the print floor.

---

## Objective

| # | Operator's words | Verdict |
|---|---|---|
| P1 | *"in the layout tab, i cant add a rosette to a plate because the plate i have selected doesnt load"* | Real. Bundled plates have no document. |
| P2 | *"on here i want to be able to select which references get applied to the quick z move pannel"* | Feature. |
| P3 | *"i try to 'go to survey height' but it doesnt actually move"* + a clamp/timeout log | Real, **and worse than reported** — it was a crash-down. |
| P4 | *"the map wells does a good job finding the wells now, but the map is flipped"* | Real. Orientation rule bypassed. |
| P5 | *"the preflight … says the taught plate bottom has no recorded xy. I did the touch-off based method because the optical method didnt work"* | Real. Only the optical writer recorded the anchor. |

---

## P3 — the survey-height park drove the needle onto the glass 🔴

### What the log actually says

```
Z clamped: -51.560 → -76.350 mm
Print floor: Z -76.350 would punch through the plate bottom (-101.980 mm raw) — clamped to plate bottom
wait_for_z_arrival timeout (16.503s): target=24.79, actual=(-42.0, -16.55, -101.98, -13.66)
safe_travel_to: Z retract not confirmed at safe height — ABORTING, will not start XY move
```

Reproduced **exactly** from this machine's own `settings.json`:

```
zero_position.Z = -76.35     safety_limits z_min/z_max = -136.35 / -76.35
z_up_sign       = -1         ⇒ height = −zref
envelope raw [-136.35, -76.35]  →  zref [-60, 0]  →  height [60, 0]
                z_max = -76.35 = the datum = the mechanical BOTTOM (height 0)
```

| reference | stored zref | height | in envelope |
|---|---|---|---|
| `plate_top_z` | −43.75 | **+43.75** | ✅ |
| `plate_bottom_z` | −25.63 | **+25.63** | ✅ |
| `fast_move_z` (Safe) | **+24.79** | **−24.79** | ❌ |
| `max_z` | +25.79 | −25.79 | ❌ |
| `replace_z` | +48.70 | −48.70 | ❌ |

Three of the five references are **24–49 mm below the mechanical hard bottom**.
They were captured against a *previous* Z datum: `apply_z_setup` (Set Bottom /
Set Top) rewrites `zero_position["Z"]`, `z_up_sign` **and** the envelope, and
nothing invalidated the references captured against the old one. `plate_top_z`
and `plate_bottom_z` were re-taught afterwards; the other three were not. That
is the exact fingerprint of a re-datum with no migration.

So `safe_travel_to`'s **step 1 retract** asked for height −24.79, the soft-limit
clamp pulled it to the hard bottom, and the print floor pulled it back up to the
plate bottom — i.e. the *retract* was a **full-depth plunge**, and the only thing
that stopped the needle short of the glass was a clamp that exists for a
different purpose. Then `wait_for_z_arrival` polled for `24.79` while the stage
sat at `−25.63`, so it burned 16.5 s and aborted the travel. The operator saw
"it doesn't move".

### Five defects, all fixed

1. **`safe_travel_to` step 1 had no never-descend guard.** `ensure_retracted_to`
   early-returns via `needle_at_or_above` so *"a misconfigured/too-low target can
   never cause a crash-down"* — but `safe_travel_to` called the shared helper
   `_retract_z_slow_then_fast` **directly**, bypassing it. The guard now lives in
   that helper, which is where both callers meet, and both callers are retracts.

   ⚠ **The at-or-above check IS the whole raise-only guard.** My first cut added
   a separate "refuse a descent" branch as well — wrong, and I caught it before
   it shipped: every descent case is already an at-or-above case, so that branch
   could only ever fire on ordinary travel that starts above the safe height, and
   would have refused it. There is a comment saying so, because it reads like a
   missing check.

2. **A clamped move was structurally unconfirmable.** `move_z_absolute` mutates
   its destination twice (soft limit, print floor) and told no one; the helper
   then waited on the *requested* value. New pure `effective_z_target_zref()`
   predicts the landing point; the retract and the step-3 descent both command
   **and confirm** against it. `move_z_absolute` now returns its effective
   destination (it used to return `None` unconditionally, so no caller changes).

3. **Stale references survived a re-datum.** New
   `StageController.z_reference_reachable()` — a reference is unreachable when it
   maps to a raw position the machine's *own* envelope rejects, which is provable,
   not a guess. `CalibrationPage._zoff_drop_unreachable_z_references()` drops such
   references on load, marks the label `⚠ stale — re-teach`, and logs why.
   **Dropped, never clamped**: absent forces a re-teach and is recoverable; a
   silently corrected travel height is a wrong move that looks right.

4. **`_safe_navigate_to` fell back to `_safe_z or 0.0`.** Zero-ref 0 is not a
   neutral default — on this machine it **is** the hard bottom, so the fallback
   was itself a full-depth descent. It now derives a real height via
   `default_travel_z(ref, 10 mm)` from the nearest taught plate reference, and
   **refuses** when there is nothing to derive from. It also returns a bool and
   no longer logs "Safe navigate to …" after an abort.

5. **The park swallowed the failure.** `_on_park` ignored the result and armed a
   measurement session whose validity rests on the stage being where it thinks it
   is. It now refuses with the actionable cause named.

**The operator must re-teach Fast Move Z, Max Z and Replace Z.** They will come
up flagged `⚠ stale — re-teach` on the next launch.

---

## P5 — the contact touch-off recorded no anchor XY

`PlateLevelWizard.gate()` refuses with *"The taught Plate Bottom Z has no recorded
XY … a plane anchored at an unknown position is not a plane."* That gate is
correct and must not be relaxed: the bed-level survey measures **tilt only**, so
the taught scalar is the plane's single absolute point and `PlateLeveling.solve`
would raise on a `None` anchor.

`set_plate_bottom_z(at_xy_um=…)` had **one** production caller — the *optical*
path. The contact touch-off routes through `_zoff_set_plate_bottom_z`, which did
not push to the controller at all; `app.py` re-pushed it untagged, and
`at_xy_um=None` is a deliberate **no-op, not a clear**. So the anchor was never
established on the very path documented as the fallback — the one this operator
used *because the optical method didn't work*.

Fixed:

* One page-level writer `_zoff_push_plate_bottom_to_controller(z, source)` now
  fronts every plate-bottom push, always carrying `at_xy_um` + `source`, so a new
  capture path cannot forget the anchor again.
* New `_zoff_read_stage_xy_um()`. ⚠ A first cut called `_read_xy_um`, which
  belongs to `_MosaicScanWorker`, **not** the page — the existing v7.13 suite
  caught it immediately.
* **Persisted** (`plate_bottom_anchor_xy_um`, `plate_bottom_z_source`) and
  re-pushed on load. Without this the anchor lived only in memory, so even an
  operator who *did* record one lost it at the next restart.
* The needle-cam **estimate** now *clears* the anchor via new
  `clear_plate_bottom_anchor()`. It is measured nowhere on the plate; leaving a
  previous touch-off's XY would pair that position with a different Z, anchoring
  the plane at the right place and the **wrong height**.

---

## P4 — Map-wells labelled the plate 180° out

`PlateWellDetector.fit_lattice` canonicalises to the near-0° branch, so its
`row`/`col` are anchored at the **min-pixel** corner, and a mosaic canvas puts
pixel (0,0) at min stage X / min stage Y. Its own comment says *which corner is
really A1 stays the caller's decision, made from the plate-orientation
convention, not from pixels* — a 180°-rotated lattice fits the image equally well
and nothing in the picture breaks the tie.

`_auto_detect` made no such decision: it equated detector indices to plate
indices. On this rig (`plate_flip_180 = True` ⇒ `plate_axis_sign == (-1,-1)`) A1
is the **max**-stage corner, so every name landed on the diagonally opposite
well. The pre-v7.5.x path was safe only because it matched detections against
already-signed predicted positions.

Fixed with **one shared rule**, placed next to the existing authority:
`MosaicWellRemap.orient_lattice_index()`. Per-axis (each axis flips on its own
sign; only both-negative reduces to a rotation), and pinned by a test that walks
a real 24-well plate and asserts it agrees with `label_positions` on all 24 —
two answers to one question is what caused this.

---

## P4b — 🔴 the P4 fix corrupted the stored mosaic (found on the bench, repaired)

Operator: *"I am not able to set the mosaic scan in the right position now. the
bottom right part of the mosaic scan picture should be at the xy-stage 0,0 as a
default."* Right on both counts — and the second half explains the first.

**The convention was never broken.** The default whole-plate scan spans the
entire reachable XY envelope, so the mosaic's world-min corner IS stage (0,0)
less half a FOV, and the 180° display flip renders that corner bottom-right.
Exactly what the operator described.

**What broke was the stored extent.** The P4 rename moved A1 from the min-stage
corner to the max-stage corner (correct), and one line leaks a NAME into a
POSITION:

```python
# calibration.py, _ploc_feed_affine
if name == "A1":
    self._taught_a1 = (sx, sy)
```

`_taught_a1` is the mosaic's plate-frame anchor, and `MosaicStore.reanchor`
rigid-translates every stored extent by any change in it — on the assumption
that a changed anchor always means the plate physically moved. Here it meant the
labelling convention had been corrected, while `plate_frame.extent_mm` was still
recorded against the OLD corner. Two incompatible definitions of "A1" straddled
a saved artifact.

Verified on disk against the last committed copy of `plate_mosaics.json`:

| | value |
|---|---|
| correct extent (= envelope + half FOV) | `[-1652.3, -1652.3, 117979.3, 75887.3]` |
| after the re-anchor | `[93291.3, 55369.5, 212922.9, 132909.0]` |
| translation | **(+94 943.6, +57 021.8) µm** — the plate diagonal |

X max 212 923 µm on a 116 340 µm machine: clean off the stage. The manual nudge
sliders are ±10 mm, so the operator genuinely **could not** put it back — which
is precisely what they reported.

### Fixed two ways

**Code** — `MosaicStore.reanchor` now refuses a shift larger than
`MAX_REANCHOR_SHIFT_UM` (25 mm). A re-seat is millimetres; at ~95 mm this is not
a re-seat but a frame mismatch, so the honest response is to leave the stored
extent alone and say so. Re-scanning is recoverable; a silently relocated mosaic
is not — every well centre mapped off it inherits the error.

⚠ **My first cut made the limit a FRACTION of the mosaic's own size, and the
v7.13 suite caught it immediately.** A single-well scan is ~2 mm across, so any
fraction of it is smaller than a real remount, and a legitimate 3.5 mm re-teach
was refused. A re-seat is a physical quantity bounded by the plate holder, not by
how much of the plate was imaged — so the limit is absolute. Pinned by a
single-well test.

**Data** — the operator's `nest-plastic-24` scan was restored to the provable
extent (it round-trips exactly to envelope + half FOV, and matches the last
committed copy to 0.1 µm) and its `plate_frame.extent_mm` re-derived against the
NEW anchor, so the record is self-consistent and the next re-anchor is a no-op
rather than a repeat. Backed up as `plate_mosaics.json.bak-v791anchor`.

### ⚠ Separately: three single-well scans are also off-envelope, and are NOT ours

`nest-plastic-24#A2/#A3/#A4` sit at X 134 000–191 900 µm, also past the 116 340
envelope — but they were translated by **(+95 487.0, +55 767.7) µm**, a
*different* vector, and they carry no `plate_frame`, so `reanchor` never touched
them. That is pre-existing damage from an earlier global translation
(`_ploc_apply_global_translation` shifts well entries too). Left alone and
reported rather than reverted on a guess: the last committed values
(`#A2 [78032.4, 57331.6, 96412.0, 74579.0]`, `#A3 [61400.0, 60728.1, 80053.7,
74761.9]`, `#A4 [38800.9, 56896.7, 57295.2, 75224.8]`) are available if the
operator wants them restored.

---

## P1 — the Layout tab could not load a bundled plate

`RosettePlacementPage` renders only a plate carried by
`HardwareConfig.plate_doc_id`. But the library legitimately offers **bundled**
cards (bare standard formats and `PlateType` products), and
`_on_active_plate_changed` *clears* `plate_doc_id` for them by design — they have
no document. Every saved hardware config in this repo is in that state, and a
fresh install's library contains nothing else, so the tab was dead.

Fixed with the convention the builder already uses — *standards auto-fork on
first edit*: `_active_plate_document(materialize=True)` stands up an unsaved
`PlateDocument.from_standard_format(...)` (carrying `plate_type_id` so the
product's Z offsets and per-plate stores still resolve), cached per
`(format, type_id)`; `_on_placements_changed` saves it on the first placement and
re-points `plate_doc_id` at the new id.

### P1b — and then it still would not place (found on the bench, same day)

With the tab reachable, the operator hit the *next* defect: stamping a rosette
did nothing, with a clean terminal. Their saved fork
(`plt_9d67ddb19bd8.json`) is the evidence — the grid carries an override under
member key `""`:

```json
"overrides": { "": { "rosette": { "rosette_id": "ros_27b05d7a5f09", … } } }
```

**A member key identifies one generated well (`g0_0`, `r3`); `""` is the
pattern itself.** Three compounding defects, all pre-existing v7.12 behaviour
that only became reachable now:

1. `plate_document_canvas._press_select`: *"A pattern member selects its FEATURE
   unless Alt is held."* Correct in the **builder**, where you edit pattern
   parameters — but a standard plate is **one `GridPattern`**, so on the Layout
   tab every click yielded the pattern. New `set_member_pick_default(True)`
   inverts it there (Alt still reaches the pattern); the builder is unchanged.
2. `PlateDocument.place_rosette` **accepted** a `PatternFeature` with an empty
   member and wrote `override("")`. It now refuses — `_stamp` already handles
   `KeyError`.
3. `_stamp` swallowed every `KeyError` and returned silently, which is why the
   terminal was clean. It now logs and tells the operator what to click.

Plus a load-time filter dropping empty-member overrides, so the files the buggy
version already wrote heal themselves.

⚠ **Two of my own tests for this were too weak and mutations caught them**, both
the exact traps CLAUDE.md records: the override test asserted on
`placements()`, which walks `evaluate()` and therefore never sees a `""` member
(it read 0 either way) — rewritten against the raw `overrides` dict; and the
refusal test asserted `"well" in hint`, which the **idle** hint already
satisfies ("select wells…") — rewritten against the pre-stamp text plus a
distinctive phrase.

### P1c — a seated rosette now looks like the rosette you chose

Operator: *"when i place a rosette into the layout tab, it should look like the
rosette chosen."* It didn't: `_draw_rosette_badge` painted a **hard-coded ring
of six dots** for every placement, so a 3-channel insert and a 6-channel one
were indistinguishable, the bore sizes were fictional, and the rotation the
operator had set was invisible.

It now draws the rosette's own sub-wells — real count, real diameters, real
positions, rotated by the placement's `rotation_deg`. The geometry comes from
new `PlateDocument.rosette_subwell_offsets()`, which **`compile()` now also
uses**, so the picture on the plate cannot drift from what gets printed. Two
copies of that arithmetic is exactly what produced the flipped well map (P4).

Seating is rotation-only at the rosette's authored size — never scaled to fit —
so a rosette drawn larger than its well is a real design problem the operator
can now see rather than something the renderer quietly normalises away. The old
six-dot mark survives for the one case with no geometry to show: a placement
whose rosette can't be resolved, which `compile()` deliberately keeps and
`validate()` reports by name.

⚠ **A third weak test, caught by mutation.** `test_the_drawing_matches_what_compiles`
compared the drawing against the shared *helper* — which says nothing about
whether `compile()` still uses that helper. Reverting `compile()` to its own
arithmetic **survived**. Rewritten to compare against a real `compile()` result
(centroid-relative, since canvas mm and A1-relative mm differ by a translation),
plus an AST test naming the single authority.

⚠ **`materialize` defaults to `False`, and that is load-bearing.** My first cut
made it unconditional and the v7.12 suite caught a real regression:
`_rebuild_config` copies `doc.meta.name` into `plate_name`, which sits in
`active_plate_key`'s precedence chain — so a bundled plate would have been rekeyed
to `"Copy of 24-well"`, repointing the mosaic, taught calibration and
well-training stores for a plate the operator never forked. Pinned by an AST test
that `_rebuild_config` passes no keyword.

---

## P2 — choose which Z references get a quick-move badge

Five badges crowd a short XZ strip into the collision-nudge, and most rigs only
drive to two. A "Quick move" tick column now sits beside the capture buttons in
**Advanced Z references**; the selection persists (`calibration.z_ref_visible`)
and fans out to every `XZSideView` host (Jog, both calibration views, the three
pick-and-place workflow pages).

**Presentation only, deliberately.** Hiding a badge by writing `None` into
`get_z_references()` would have been simpler and would have disarmed the
controller's print-floor datum and blanked the Hardware Info card — so visibility
is a **separate** dict (`get_z_reference_visibility()`) and the reference values
are untouched. Pinned by a test.

---

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/StageController.py` | P3: `effective_z_target_zref`, `z_reference_reachable`, raise-only `_retract_z_slow_then_fast`, confirm-against-effective in `safe_travel_to` step 3, `move_z_absolute` returns its destination. P5: `clear_plate_bottom_anchor` + the `at_xy_um` no-op contract documented. |
| `SupportClasses/MosaicWellRemap.py` | P4: `orient_lattice_index` — the shared corner rule. |
| `gui/dialogs/mosaic_well_mapping_dialog.py` | P4: route auto-detect labels through it. |
| `gui/pages/calibration.py` | P3: drop unreachable refs, `_safe_navigate_to` fallback + bool. P5: anchor capture, shared writer, persistence. P2: tick column, accessors, persistence. |
| `gui/widgets/needle_bore_wizard.py` | P3: the park reports a failed travel and does not arm a session. |
| `gui/pages/hardware_setup.py` | P1: materialise + auto-fork bundled plates. |
| `gui/widgets/xz_side_view.py` | P2: `set_visible_z_references` + paint filter. |
| `gui/app.py`, `gui/pages/jog_control.py`, `gui/pages/workflows_mode.py`, 3 × `gui/pages/workflows/*.py` | P2: fan-out. |
| `tests/test_v791_calibration_and_layout_fixes.py` | NEW — 48 tests. |
| `tests/test_v711_plate_bottom_wizard.py`, `tests/test_v713_step2_plate_bottom_from_top.py` | Updated for the stronger P5 contract (see below). |

### The two updated tests

Both were asserting the *old, weaker* call shape and both now assert **more**:

* `test_apply_sets_the_reference_and_pushes_tagged` — now requires
  `at_xy_um` **and** `source`, so dropping either fails.
* The v7.11 AST provenance guard — extended to follow the new shared writer
  (`_zoff_push_plate_bottom_to_controller` takes provenance positionally) rather
  than being defeated by the delegation, **plus** a new test that the writer
  itself forwards both `source=` and `at_xy_um=`.

---

## Implementation Steps

- [x] P3 — raise-only retract, effective-target confirm, reachability gate, `_safe_navigate_to` fallback, park refusal
- [x] P5 — anchor capture on the contact path, shared writer, persistence, estimate clears the anchor
- [x] P4 — shared `orient_lattice_index`, wired into the mapping dialog
- [x] P1 — materialise + auto-fork bundled plates, `materialize=False` default
- [x] P4b — re-anchor refuses a corner-jump translation; operator's store repaired
- [x] P1c — a seated rosette draws as itself, sharing `compile()`'s transform
- [x] P1b — member-pick on the Layout canvas, `place_rosette` refuses a pattern, stamp reports, empty-member overrides healed on load
- [x] P2 — quick-move tick column, persistence, fan-out
- [x] Tests + mutation verification
- [ ] **Bench verification on ME3B V1 — see below**

---

## Testing Notes

### Mutation verification — **22/22 CAUGHT**

Each mutation is a real source edit, reverted in a `finally`; all sources
verified byte-restored afterwards.

| # | Mutation (the fix reverted) | Result |
|---|---|---|
| P3a | raise-only guard removed from `_retract_z_slow_then_fast` | CAUGHT (3 failures) |
| P3b | confirm against the REQUESTED target, not the effective one | CAUGHT (1) |
| P3c | `z_reference_reachable` always True | CAUGHT (2) |
| P3d | `_safe_z or 0.0` fallback restored | CAUGHT (3) |
| P4a | mapping dialog uses raw detector indices | CAUGHT (1) |
| P4b | remap ignores per-axis sign (always 180°) | CAUGHT (2) |
| P1a | `materialize` defaults True | CAUGHT (4) |
| P1b | fork saved but config not re-pointed | CAUGHT (1) |
| P2a | visibility filter ignored | CAUGHT (3) |
| P5a | `at_xy_um` dropped from the shared writer | CAUGHT (1) |
| P1c | `place_rosette` accepts the pattern again | CAUGHT (2) |
| P1d | Layout canvas back to the builder pick rule | CAUGHT (1) |
| P1e | empty-member override no longer filtered | CAUGHT (1) — **survived the first version of that test** |
| P1f | silent stamp failure restored | CAUGHT (1) — **survived the first version of that test** |
| P1g | rosette badge back to the hard-coded 6 dots | CAUGHT (5) |
| P1h | rotation ignored when drawing | CAUGHT (2) |
| P1i | sub-well diameter ignored | CAUGHT (1) |
| P1j | Layout page stops supplying a rosette loader | CAUGHT (1) |
| P1k | `compile()` stops sharing the offset helper | CAUGHT (2) — **survived the first version of that test** |
| M1 | re-anchor guard removed | CAUGHT (2) |
| M2 | re-anchor limit widened to 500 mm | CAUGHT (2) |
| M3 | re-anchor limit 0.1 mm (too strict) | CAUGHT (6) |

### Suite results

`tests/test_v791_calibration_and_layout_fixes.py` — **63 green**;
`tests/test_v791_mosaic_reanchor_guard.py` — **11 green** (incl. a check
against the operator's real repaired store).

Regression, run per-batch, all green:

| Batch | Suites | Tests |
|---|---|---|
| Z safety + plate bottom | z-retract · gentle-descent · print-z-reference-vector · needle-bore-wizard · plate-level-wizard · plate-bottom-wizard · step2-plate-bottom · plate-bottom-plane-controller · print-z-plate-bottom | **298** |
| Plate documents + rosette layout (re-run after P1b) | plate-doc-geometry · plate-builder-ui · plate-identity-and-stores · phase0-hygiene · rosette-flatten · plate-types · custom-plate-rendering · suite-hygiene | **455** |
| Orientation + plate documents | mosaic-orientation-remap · plate-orientation-convention · plate-well-detection · startup-well-map · mosaic-plate-frame · plate-builder-ui · plate-identity-and-stores · plate-doc-geometry · phase0-hygiene · rosette-flatten · plate-types | **447** (3 skipped) |
| XZ view / jog / context | xz-custom-z · needle-offset-z-side-view · plate-location-z-side-view · jog-navigation · context-panel · responsive-context-panel · suite-hygiene | **110** |
| Bore safety + travel | bore-safety-remediation · other-workflows-bore-safety · hard-abort · jog-travel-off-gui-thread · per-bore-cell-targeting · bore-gate-live-refresh · bore-offset-calibration | **271** |
| Bed-level math | plate-level-math · plate-level-worker · holdout-focal · plate-bottom-optical · plate-bottom-worker · plate-z-plane-math | **154** |

Plus a `gui.app` import smoke, and offscreen builds of the real `CalibrationPage`
and `HardwareSetupPage` driving the new paths end to end.

Also green individually: plate-location-manual-click-rim (27) · calibration-revision
(20) · last-known-calibration (21) · plate-centering (6) · freeform-warp (12), and
the map-wells classes of `test_v75x_plate_mosaic` (23).

**Two pre-existing conditions, both documented in CLAUDE.md and both confirmed not
ours:**

* `test_v75x_plate_mosaic::TestFilledWellDetector::test_real_24_well_mosaic` fails
  (23 vs 24). It exercises the legacy blob detector in
  `SupportClasses/VisionDetector.py`, which this change does not touch — verified
  against the diff. (The new `PlateWellDetector` resolves all 24 on that image;
  this is the old detector's standing failure.)
* `test_v75x_plate_mosaic::TestManualAlignPage` hangs, which makes a whole-suite
  run of that file never terminate. Excluded per the precedent already recorded
  for it; every other class in the file was run.

### Needs real-HW verification on ME3B V1, IN ORDER

**P3 first — nothing else is trusted until it passes.**

1. Launch. Fast Move Z / Max Z / Replace Z must read **`⚠ stale — re-teach`**
   (they are 24–49 mm below the hard bottom). If they read as ordinary values,
   the reachability gate did not fire.
2. **Re-teach Fast Move Z** with the needle at a genuine travel height, then Max
   and Replace. Confirm each reads a sensible positive height.
3. Bore offsets → set survey height 0.5 mm → **"Go to needle at survey height"**.
   The needle must **retract first**, travel XY, then descend to 0.5 mm above the
   glass. Watch the needle, not the screen.
4. Before step 3, with the stale values still in place, confirm the park now
   **refuses in the panel** instead of moving and timing out for 16 s.
5. P5: run the **contact touch-off**, then open Plate Bed Level — the preflight
   must no longer say "no recorded XY". Restart and confirm it still passes.
6. P5: run the needle-cam **estimate** and confirm the preflight goes back to
   refusing (an estimate has no anchor — that is correct, not a regression).
7. P4: Plate Location → Mosaic scan → Map wells. **A1 must be top-left.** Confirm
   all, then drive to two named wells and check the needle lands on them.
8. P1: with a bundled plate starred, open Hardware Setup → Layout. The plate must
   render. Stamp a rosette; confirm the library gains "Copy of …" and the ★
   moves to it.
9. P2: untick Replace and Max on Advanced Z references; confirm the badges vanish
   on the Jog page too, restart, and confirm the choice stuck.

---

## Issues & Decisions

* **Three of my own first cuts were wrong and were caught before shipping** — the
  redundant descent-refusal branch (would have refused ordinary travel), the
  unconditional `materialize=True` (would have rekeyed every per-plate store),
  and `_read_xy_um` (belongs to the scan worker, not the page). The second and
  third were caught by the *existing* v7.12 / v7.13 suites, which is the argument
  for running them before writing new ones.
* **The gate at `plate_level_wizard.py` was deliberately NOT relaxed.** The
  anchor XY is load-bearing arithmetic, not a formality.
* **Not done:** the stale references are dropped but not *migrated*. There is no
  way to recover what datum they were captured against, so a re-teach is the only
  honest answer.
