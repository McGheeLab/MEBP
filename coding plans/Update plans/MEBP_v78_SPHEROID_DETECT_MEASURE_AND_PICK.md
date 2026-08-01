# MEBP v7.8 — Spheroid detection, measurement and curated picking

## Objective

Let the operator survey a whole well once, have every spheroid detected and
**measured**, curate that list, and pick from it — instead of eyeballing each
target and running the entire queue on one typed diameter.

Operator's request (verbatim, 2026-07-30):

> On the spheroid pick and place, I want a toggle mode where each spheroid is
> detected and a circle is fit around the spheroid to measure its true radius.
> The user can scan the entire well with a fluorescent mosaic, and we detect the
> location and size of all spheroids within a user defined range… Spheroids that
> have an autodetected diameter can be modified by the user to redraw it for
> actual diameter. Any user selected spheroids should have an image taken and be
> cropped to the bounds of the spheroid then saved in a folder, we will use this
> as training data for future spheroid detection (do not build this training
> system) we just want to save the images.
>
> I want to be able to click on a list of spheroids to select one, this list
> should be filtered by size ascending or descending. When we click on one it
> moves to that spheroid location and the user can click on it to do a pick
> location operation to store that spheroid in the list of picks.
>
> For any spheroid in the pick list or place list, the user can select that
> location, and a button for goto, edit, or delete should be on the screen
> somewhere.
>
> Since we know the needle diameter, we should also give the user a warning when
> the spheroid is larger than the needle ID — we should have a needle ID that is
> at least 1.5x the spheroid diameter.

### Decisions taken with the operator (AskUserQuestion)

| Question | Choice |
|---|---|
| Layout | **A second tab** for the survey, with an explicit transfer of a curated list into the pick list |
| Scan source | The tab hosts a **live instance of the Fluorescence Mosaic page** — *"if we make changes to the fluorescent scan workflow it should automatically update this page like an instance"* |
| Per-spheroid volume | **Opt-in checkbox, default ON**; always multiplied by the safety factor; safety-factor range widened to **0.01 – 10** |
| Redraw a circle | **Both gestures** — drag the ring, or click 3+ rim points — on both the mosaic and the live feed |
| Training crops | A **fresh live capture** at the spheroid, cropped to the circle bounds, with metadata |
| Needle rule | Advisory only, **never blocks**; the 1.5× factor is a setting |

## The dangerous part, fixed first

**Mosaic pixel → stage µm was not trustworthy.**
`MosaicBuilder.canvas_extent_um` (`SupportClasses/MosaicBuilder.py:600-615`) *adds*
`_global_shift_um` to the extent it returns while the tile pixels stay at raw
stage positions, so back-projection must use `extent[:2] − shift` — which is why
`MosaicStore.save` records `shift_um` and `calibration.py:7511` has a
`_ploc_mosaic_world_shift` helper. The fluorescence worker called
`finalize_global_shift()` (`fluorescence_mosaic_workflow.py:191`), read the now
shifted extent, and saved it through a `FluorescenceMosaicStore.save_channel`
that had **no `shift_um` parameter** — so the shift was discarded.

`_max_shift_px()` (`MosaicBuilder.py:1023-1028`) with the default
`max_shift_um = 0` (`MosaicCalibration.py:107`) bounds that shift to **20 % of
the FOV width** — roughly ±337 µm at 10×, i.e. larger than a spheroid. Driving
the needle to such a coordinate with `pick_z_offset_mm = 0.10` would put a pulled
glass tip on the glass a quarter of a millimetre from the target.

So Stage 1 lands before anything else, and two independent guards follow it:

1. `has_shift()` distinguishes "recorded as zero" from "never recorded"; **Go to
   and Transfer are disabled** on a legacy mosaic, with the error bound stated in
   µm. Detection and measurement still work on it — a diameter is
   translation-invariant.
2. A transferred position is stamped `PROV_MOSAIC`, drawn as a **dashed** ring and
   marked `~mosaic` in the list until a live click on that spheroid re-derives it
   through `pixel_to_stage_offset` and upgrades it to `PROV_CONFIRMED`.

## Files

### Created
| File | Purpose |
|---|---|
| `SupportClasses/SpheroidDetector.py` | Pure detector: absolute-µm size window, two named shape gates, and the ONE px→stage-µm back-projection |
| `SupportClasses/SpheroidTrainingStore.py` | Banked crops + metadata, plus the pure crop geometry and its refusals |
| `gui/widgets/spheroid_survey_panel.py` | Detect → curate → transfer, with a threaded detector and a numerically-sorted table |
| `gui/widgets/spheroid_mosaic_items.py` | Editable circles + radius handle + rim-fit layer on the mosaic view |
| `gui/widgets/spheroid_crop_worker.py` | Off-GUI-thread fresh-frame capture |
| `tests/test_v78_spheroid_detection.py` | 39 — detector + projection |
| `tests/test_v78_fluor_mosaic_shift.py` | 15 — the shift round-trip |
| `tests/test_v78_spheroid_training_store.py` | 37 — crop store + geometry |
| `tests/test_v78_spheroid_per_target_volume.py` | 43 — per-spheroid volume + the 1.5× rule |
| `tests/test_v78_target_actions.py` | 48 — picker selection, actions, live circle editing |
| `tests/test_v78_spheroid_survey_tab.py` | 49 — survey panel + mosaic overlay |
| `tests/test_v78_spheroid_page_integration.py` | 50 — the page built for real, safety gates |

### Modified
| File | Change |
|---|---|
| `SupportClasses/FluorescenceMosaicStore.py` | `save_channel(shift_um=)`, `get_shift_um`, `has_shift`, `get_mosaic_scale`, `get_um_per_px`, `get_objective`, `MEBP_FLUOR_MOSAIC_PATH`, `get_store(path=)` |
| `SupportClasses/VisionDetector.py` | Public `fit_circle_robust` delegate (so the new module touches no private name) |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | `embedded=True`; `mosaic_ready` signal; `mosaic_context()` + accessors; worker captures the shift and `finished_ok` gains it; `_ZoomImageView.set_interactive_items` |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | Two tabs; embeds the scan page; per-pair configs; the clearance setting; `_stage_busy`; `_travel_to_absolute`; crop path; extra-state persistence |
| `gui/widgets/live_target_picker.py` | Selection survival by id; Goto/Edit/Delete; measure mode with both gestures; provenance; `pick_added`; size-scaled removal radius; `to_px` orientation fix |
| `gui/widgets/camera_manager.py` | `stage_offset_to_pixel` — the documented exact inverse of `pixel_to_stage_offset` |
| `gui/widgets/workspace_target_view.py` | Optional 4th tuple element so a measured spheroid draws at its true relative size |
| `tests/test_v75x_fluorescence_mosaic.py` | `finished_ok` arity + a shift assertion |

## Implementation steps

- [x] **Stage 1 — registration-shift integrity.** `shift_um` persisted and read
      back; `has_shift` gates motion; the worker captures it via
      `getattr(builder, "_global_shift_um", (0, 0))` (the same defensive read
      `calibration.py` uses); env override + `get_store(path=)` for test isolation.
- [x] **Stage 2 — pure detector.** A **sibling** of `detect_filled_wells`, not an
      edit of it: its gate is *relative* (`min_area_frac` of the largest blob, no
      maximum) and it feeds the plate-location calibration path whose well centres
      command motion. Two separately named gates, because that function's
      `circularity_min` is actually a circle-fit inlier fraction and its 0.55
      default is tuned to accept a partial well arc. `THRESH_TRIANGLE` by default
      (Otsu assumes bimodality and floods a mostly-dark mosaic), a morphology
      kernel scaled to the requested minimum size, canvas-margin masking, an
      optional well-boundary mask, and per-reason rejection counters.
- [x] **Stage 3 — crop store + geometry.** Raw-frame crops only; clamped rather
      than black-padded; refusals for a clipped circle and for the stage not being
      on the target.
- [x] **Stage 4 — per-spheroid volume + the 1.5× rule.** One config per pair via
      `dataclasses.replace`; an unmeasured pick keeps the SAME config object, so
      the queue is byte-identical to pre-v7.8. `clearance` threaded into the
      existing `spheroid_pickup_detail`. Safety factor 0.01–10.
- [x] **Stage 5 — picker selection survival + row actions.** Ids in
      `Qt.UserRole`; selection captured and restored around the rebuild; multi-
      select enabled so "Remove selected" finally matches its own plural loop.
- [x] **Stage 6 — live-feed circle editing.** Ring-before-centre hit test in
      widget px; projection frozen during a drag; rim-point fit; right-click
      unchanged; every gesture gated on measure mode.
- [x] **Stage 7 — the survey tab.** Embedded scan page + survey panel + editable
      mosaic circles + curated transfer.
- [x] **Stage 8 — training-crop capture.** Threaded, with the stage position
      sampled alongside the frame.
- [x] **Stage 9 — settings + docs.** Crop knobs in the popout; the detection band
      and mode flags via `set_extra_state`; this document.

## Testing notes

**281 new tests, all green** (`python -m unittest discover -s tests -p "test_v78_*.py"`).

Regression, each suite run on its own: fluorescence-mosaic (35),
spheroid-picker-scaling (12), spheroid-pick-place-z (16),
spheroid-disengage-sink-timing (21), cell-labeling (23), cell-targeting (21),
workflow-settings-popout (35), camera-calibration-store (30),
camera-rotation-monitor (43), mosaic-orientation-remap (14) — all green.
`plate_mosaic` runs 110 with the one **pre-existing** `test_real_24_well_mosaic`
CV failure CLAUDE.md already documents, plus the documented `TestManualAlignPage`
hang (excluded to get a terminating run).

⚠ **Pre-existing environmental flake, confirmed NOT from this change.** Running
the camera-probing suites (`camera_calibration_store`,
`camera_rotation_cal_and_monitor`) together with several others in ONE process
intermittently hangs on this machine — real DirectShow enumeration against
attached cameras. Verified by running the identical 9-suite command in a pristine
`git worktree` at HEAD: it timed out there too (exit 124), while a separate
attempt completed with exit 0 at both HEAD and on this branch. Every suite passes
individually. Run the camera suites on their own.

The tests worth knowing about:

* **The projection closure**, parameterised over `(rotation, flip_x, flip_y)` ∈
  `{0, 90, 180, 270, 37.5} × {F, T} × {F, T}`: click → stage → overlay → click
  must return the original pixel. At θ=0 with no flips the correct and the naive
  inverse agree, which is exactly why the marker-placement bug survived; one test
  asserts the naive inverse is >50 px wrong on a 90° camera.
* **Non-zero shift and non-zero origin** in every back-projection test. A
  `shift=(0,0)`, `origin=(0,0)` fixture passes for both the correct formula and
  the buggy one. `back_project_px` is also pinned against
  `MosaicWellRemap.detect_raw_positions` so the two cannot drift.
* **A giant well-wall blob does not suppress small in-band discs** — the failure
  the relative gate in `detect_filled_wells` would have had, with a companion test
  demonstrating that gate finds fewer.
* **Queue byte-identity**: with per-spheroid sizing off, or every `size_um` still
  0, every operation shares the same config OBJECT.
* **Safety gates**: `_travel_to_absolute(12345, 6789)` reaches `SafeTravelWorker`
  with exactly those numbers and `target_z_mm=None`, and no `move_xy_absolute*`
  is ever called; feeding the same numbers to the zero-ref handler must give a
  *different* destination (so the zero can never be added twice); travel is
  refused while the executor thread or a mosaic scan is running.
* **`pick_only` non-regression** (Cell Labeling shares the picker) and
  **measure-mode-off** non-regression (every left press still adds a target).

## Needs real-hardware verification on ME3B V1, in this order

1. **Registration ground truth — the go/no-go for the whole feature.** Scan a
   well from the survey tab, note the logged shift, Go to a detected spheroid and
   read the residual. **Do not attempt a pick until the residual is within
   µm/px-scale noise for at least three spheroids spread near the mosaic
   corners**, not just the centre: a *scale* error produces a position error that
   grows with distance from the origin and is invisible at one central point.
2. **Axis/sign sanity, needle at safe Z.** Go to five spheroids; each must land
   centred. A mirrored axis reads as "right distance, wrong way" — obvious at
   five points, invisible at one.
3. **Diameter truth.** Cross-check three auto diameters against the existing
   `MeasurementCameraView` µm/px tool on the same spheroids.
4. **Redraw.** Drag a ring and rim-click a circle on both surfaces; the Ø must
   agree between them and in the pick list.
5. **Needle advisory.** With an orifice under 1.5× a real spheroid: ⚠ in the row,
   in the aggregate line and at Start — **and Start still runs**.
6. **Per-spheroid volume.** Pick two visibly different spheroids in one run; the
   log must show two different aspirate volumes matching (4/3)πr³ × safety.
7. **Crops.** PNGs land under `config/hardware/spheroid_training/`, are
   **un-annotated** (no overlay ring baked in), and `diameter_um` matches the
   on-screen circle.
8. **Regression sweep.** One ordinary run with measure mode off; one Cell
   Labeling selection; one Fluorescence Mosaic scan from its own page.

## Issues & decisions

* **Existing installs must RE-SCAN a well before picking from it.** Mosaics saved
  before v7.8 have no `shift_um`, and the shift baked into their extent is
  unrecoverable. They stay fully usable for viewing and measuring; only motion is
  gated.
* **A sibling detector, not an edit of `detect_filled_wells`.** Its relative size
  gate is right for "the N biggest discs on a plate mosaic" and wrong for an
  absolute µm band, and it is on the plate-location calibration path.
* **Detection may legitimately return zero.** On a nuclear stain a spheroid is a
  cluster of puncta, not a filled disc, so the realistic outcomes are "0 found"
  and "hundreds found". Hence: manual add is first-class (click the mosaic, or 3
  rim points), and the status line reports per-reason counters —
  `saw 14 blobs → 3 in band · 5 too small · 4 too large · 2 not round` — rather
  than a bare count that cannot tell the operator what to change.
* **Touching spheroids are NOT split** in v1. A doublet exceeds
  `max_diameter_um` (or fails circularity) and is dropped, which is literally the
  operator's stated rule; the counters make that visible. Distance-transform +
  watershed over-segments fluorescent spheroids and would ship confidently-wrong
  diameters.
* **Detection runs on ONE raw channel**, never `composite_overlay`'s blend: the
  blend goes through grayscale and a saturating `cv2.add`, which clips
  overlapping channels and pushes a saturated blob's edge outward — it over-reads
  diameters. `composite_plate_overlay` is never used for back-projection at all;
  it derives its own scale.
* **The stored scale is cross-checked against the image width** and a >1 %
  disagreement is reported. `composite_overlay` resizes later channels to channel
  0's shape and returns channel 0's extent without saying which channel that was,
  so a `target_px` change between channels would otherwise corrupt every
  coordinate.
* **`size_um` becomes load-bearing** after years of being serialized but never
  written, so it is clamped to 1–5000 µm on read: it now changes an aspirate
  volume, which scales as diameter cubed.
* **Release-volume interaction warned, not blocked.** With `release_enabled` the
  dispense is a fixed volume while a per-spheroid aspirate varies with diameter
  cubed, so the retained residual differs per pick and accumulates; the run warns
  with the projected total against the needle's internal volume.
* **`_travel_blocked_by_run` became `_stage_busy`** and now also covers the mosaic
  scan worker, which drives `safe_travel_to` and toggles the **non-refcounted**
  `suspend_position_poller` — whichever finishes first would otherwise re-enable
  the poller underneath the other.
* **Two frames, two handlers, deliberately.** `_travel_to_absolute` takes absolute
  stage µm; `_on_workspace_position_clicked` takes zero-ref µm and adds
  `zero_position`. Sharing one handler would add the zero twice.
* **Multi-select enabled on the picker lists.** "Remove selected" always intended
  plural (its loop walks rows in reverse) but the lists were left on the default
  single-selection mode, so it could only ever remove one. Row actions stay gated
  on exactly one selection.
* **Bug found and fixed en route: `_PickerCameraView.to_px`** was the plain
  identity inverse while the click path applies mirror → flip_y → R(θ). On any
  rotated or mirrored camera the marker did not land under the click — cosmetic
  for a point, a correctness bug the moment the operator resizes a circle or
  clicks a spheroid a Go to just travelled to. Fixed by inverting the same
  transform through a new `CameraManager.stage_offset_to_pixel`, which lives
  beside the forward map so the two cannot drift.
* **Label and handle are CHILDREN of the mosaic circle**, so a drag carries them.
  `mosaic_well_mapping_dialog` makes them siblings and its labels lag during a
  drag; that is an existing bug there, deliberately not retrofitted here.
* **No in-scene numeric widgets** — `plate_designer_canvas.py:718-719` records
  that in-scene `QGraphicsProxyWidget` spin boxes crashed.
* **The detected list is deliberately NOT persisted.** A restored coordinate from
  another plate would be a crash, and nothing can prove the mosaic it came from is
  still the one loaded. Only the detection parameters and mode flags round-trip.

### Deferred by choice

* Watershed splitting of touching spheroids (v1 drops them and says so).
* An "adopt the measured residual as this well's shift" button — the residual
  check itself is worth adding once the hardware numbers are known, but nothing
  should write a shift automatically.
* Sorting the survey list by distance from the current position (would minimise
  travel across a queue; nearly free now the table exists).
