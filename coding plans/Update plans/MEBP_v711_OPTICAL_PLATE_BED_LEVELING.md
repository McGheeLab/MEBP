# MEBP v7.11 — Automated optical plate-bed leveling

Implements **Stage 7 ("Mode B")** of `MEBP_v75x_PLATE_Z_PLANE_AUTOCAL.md`, upgraded
from "the operator types a focal readout" to a trained, multi-objective, automatic
through-focus survey.

---

## Objective

Measure the plate bottom's **tilt** optically and install it as a `PlateZPlane`,
with the needle retracted throughout.

The plate bottom is flat but tilted. This machine's own saved calibration records
`0.0199 mm/mm` — about **1.15 mm of Z across a 24-well plate's row span**, i.e.
2–10× a typical 0.1–0.5 mm print height. That tilt was being measured by the
calibration page and then discarded: prints resolved plate bottom from a single
scalar.

**The consumption path was already built and had zero production callers:**
`PlateZPlane.from_focal_readings`, `StageController.set_plate_z_plane`, and
`PrintManager.well_print_z_zref_mm`. This work is the missing producer.

### Operator decisions

Sweep range is **a fraction of working distance** · plate-wide tolerance target
**better than 50 µm** · feature **auto-proposed with operator override** ·
**operator-configured objective ladder** · **site 1 taught, sites 2..N automatic**,
tolerating plate-to-plate differences · this tab becomes about the plate **bed** ·
wizard-style UI · **one landing**.

---

## Blockers found and fixed first

| # | Defect | Consequence | Status |
|---|---|---|---|
| **B1** | `gui/widgets/needle_bore_wizard.py:1041` called `get_microscope().state` without `()`. `state` is a *method*, so `getattr(bound_method, "has_focus", False)` was always False — inside a broad `except`. | `_microscope_focus_um()` returned `None` on **every** call, so every `NeedleFocusTemplateStore` capture stored `microscope_focus_um=None`: **there was no focus↔needle datum anywhere on disk.** | [x] |
| **B2** | `holdout_error_mm` filters `z_zref_mm is not None`, which focal points never have ⇒ `from_focal_readings` passed `holdout=None` ⇒ `_validate_plate_z_plane` skipped its hold-out branch. | **The module's own stated acceptance gate was not enforced on the mode that measures the plate optically.** Measured: a 140 µm bad site gives a 46.7 µm residual, which PASSES the 50 µm residual gate — so that plane was **accepted**. | [x] |
| **B3** | `MountedOptic` read NA / working distance / magnification and formatted them into a `detail` **string**, discarding the numbers. | Sweep step sizing and the collision bound both need floats; re-parsing `detail` fails silently into a wrong step or a wrong bound. | [x] |
| **B4** | `MicroscopeController` had no exclusivity; `STALE_OP_S = 20 s` silently drops a queued op. | A dropped `set_objective` means every later focus sample is taken through the **wrong objective**, and the resulting plane looks well-formed. | [x] |
| **B5** | `set_plate_bottom_z` recorded anchor XY and provenance but **not** the zero-Z epoch. | Set Z Zero rejected the *plane* loudly and left the *scalar* stale **silently** — and the scalar is what every non-tilt-aware consumer reads. | [x] |
| **B6** | CLAUDE.md states "µm-NATIVE … 1.0 units/µm" and "travel 0–400000 µm" ~2000 words **upstream** of its own correction to 40 units/µm and 10 mm. | A reader sizing a sweep who stops at the first claim is 40× wrong and believes 40× the travel. *(Correction to the plan's framing: the doc is not stale — the correction is present but buried, so it is annotated in place rather than rewritten.)* | [x] |

---

## Files

### New — pure, GUI-free

| File | Contents |
|---|---|
| `SupportClasses/ObjectiveOptics.py` | Berek depth of field, FOV, **`wd_bounded_half_range_um`** (the collision bound), FWHM bands, field-number sanity, `from_mounted_optic` |
| `SupportClasses/FocusCurve.py` | `FocusSample` / `FocusPeak`, `peak_focus_um`, `centroid_peak`, and **12 refusal codes** |
| `SupportClasses/FocusSweepPlanner.py` | `SweepRung` / `SweepPlan`, `plan_sweep`, `plan_handoff`, `next_rung`, `choose_ladder`, `sweep_targets_um` |
| `SupportClasses/ObjectiveLadder.py` | `LadderRung`, `resolve_ladder`, `ladder_gate`, `um_per_px_at_resolution`, `parfocal_offsets_um` |
| `SupportClasses/PlateLeveling.py` | `sites_gate` / `sites_geometry` / `propose_sites`, `PlatePrior` / `update_prior`, `SiteMeasurement`, `solve` |
| `SupportClasses/PlateFocusDatumStore.py` | focus↔needle datum + **`check_scale`** |
| `SupportClasses/PlateLevelSiteStore.py` | multi-defocus feature bank + the plate-to-plate prior |

### New — GUI

| File | Contents |
|---|---|
| `gui/widgets/plate_level_wizard.py` | `PlateLevelWizard` (5 steps, gate table) + `PlateLevelSurveyWorker` (QThread) + Qt-free `SiteSpec` / `SurveyState` |

### Modified

`SupportClasses/PlateZPlane.py` (B2 + surface-mix + `focus_sigma_um`) ·
`SupportClasses/StageController.py` (B5 + `would_accept_plate_z_plane`) ·
`SupportClasses/MicroscopeControl.py` (B3 numerics + B4 lease + simulator optics) ·
`SupportClasses/MicroscopeConfigStore.py` (parfocal/centration block) ·
`gui/widgets/needle_bore_wizard.py` (B1) ·
`gui/pages/calibration.py` (tab → "Plate Bed Level", index from `addTab`) ·
`CLAUDE.md` (B6).

---

## Algorithm

**Depth of field — Berek**, `DOF = λn/NA² + n·(µm/px)/NA`, λ=0.55 µm, n=1.0.
*Inoué & Spring 2e p.31; Nikon MicroscopyU (Berek 1927).* The geometric term's
`e/M` **is** the measured sample-side µm/px already in `ObjectiveCalibration`, so
the formula is driven by a measurement rather than a nameplate. `e` = 1 px (not
Nyquist 2), which halves the geometric term — conservative on purpose.

Verified: 4x/NA0.13 → **57.2 µm**, 10x/NA0.30 → **10.4 µm**, 20x/NA0.45 → **4.1 µm**.

**Sweep range = a fraction of working distance** (`WD_SWEEP_FRACTION = 0.25`),
under the SDK travel and the operator soft limits, and the lead-in position must
be legal too. Verified: 4x → ±4100 µm, 20x → **±250 µm** (a 20x physically cannot
do ±1 mm). Unknown WD falls back to ±50 µm **and says so**. An empty plan
**raises** — returning `()` would surface as "no peak found", an optics diagnosis
for a limits problem.

**Coarse-to-fine**: `s_r = R_r/K`, `R_{r+1} = 2·s_r`, each rung re-centred on the
**measured** peak. Caps: `s_0 ≤ 8·DOF`, `s_r ≥ 4×` drive resolution. Verified 4x
plan: 3 rungs × 13 frames, steps 166.7 / 55.6 / 18.5 µm (2.92 / 0.97 / 0.32 DOF).
**Backlash lead-in on every position** — each rung boundary reverses direction.

**Peak — weighted Gaussian fit to log(F−B)** over the half-max set, seeded and
cross-checked by the closed-form 3-point log-parabola vertex; closed form, so it
cannot fail to converge on the pathological curves it is meant to refuse.
*Naidu & Fisher BMVC 1991; Yeo et al. IVC 11(10) 1993; Groen et al. Cytometry 6:81
1985; Santos et al. J.Microsc. 188:264 1997; Guo IEEE SPM 28(5) 2011.*
Measured accuracy: **0.3 % of a step** on a synthetic Gaussian.

**FWHM is the "point spread" quality metric.** Expected 1–3×DOF; hard-refuse
outside 0.5–6×DOF — below 0.5×DOF is *physically impossible*.

**Refusals:** `PEAK_AT_EDGE` · `MONOTONIC` (checked FIRST, else unreachable) ·
`LOW_PROMINENCE` · `MULTIMODAL` · `SATURATED` · `NON_CONCAVE` ·
`VERTEX_OUTSIDE_BRACKET` · `FWHM_MISMATCH` · `DRIFT` · `MIXED_EXPOSURE` ·
`INSUFFICIENT_SAMPLES` · `BAD_BASELINE`.

---

## Teach once, then automatic

**The prior only ever narrows a SEARCH; it never contributes to a fitted plane.**
`solve()` has no `prior` parameter — pinned by a test that inspects its signature.
Every run re-measures the tilt, so a plate that seats differently is measured
rather than assumed.

| Site | Window | Source |
|---|---|---|
| 1 | `±(3σ + margin)` from the stored prior, else full WD-bounded range | operator confirms the auto-proposed patch |
| 2–3 | `stored_tilt + Δ_run` | this plate's own offset, from site 1 |
| 4–N | the live fit over this plate's own sites | very narrow |

Verified self-tightening: 4 runs of ±20 µm spread take site 1 from **1000 µm →
110 µm**; a noisier history keeps a wider window; no history keeps the full range.

---

## Safety

- **The needle never descends.** Every site via `safe_travel_to(target_z_mm=None,
  apply_insert_floor=False)`; a refused travel **stops the run**.
- **`set_print_floor_active` is never called** — pinned by `floor_calls == []`.
- **Objective-into-plate** guarded by four layers, including reading
  `state().focus_um` **back** after every move (it clamps silently).
- **Exit guarantee**, asserted on success / failure / cancel: poller resumed
  exactly once, objective and focus restored, lease released, nothing installed.
- **Drift closure**: site 1 is re-measured at the end; >10 µm blocks Accept.

---

## Status

- [x] B1–B6
- [x] Seven pure modules + three stores
- [x] Wizard + survey worker
- [x] `calibration.py` — tab renamed "Plate Bed Level", wizard mounted
- [x] Tests (180 new)
- [ ] **Relocate the legacy needle-descent auto-cal into the needle-centring
      wizard.** Deliberately deferred: it owns `_zoff_live_view` (the microscope
      feed the descent watches) and the Custom tab's button drives it, so moving
      it means moving a live camera pane between tabs. It is kept as the second
      sub-tab of "Plate Bed Level" and works unchanged.
- [ ] Real-hardware verification (below)

---

## Testing notes

**180 new tests green**, 9 files:

| File | n | Covers |
|---|---|---|
| `test_v711_focus_curve.py` | 21 | peak accuracy, every refusal, two-surface detection at 168 µm |
| `test_v711_objective_optics.py` | 21 | Berek by hand, the WD bound, field-number contradiction |
| `test_v711_focus_sweep_planner.py` | 18 | **the guard as a property over 400 randomised plans**, empty-plan raise, re-centring |
| `test_v711_plate_level_math.py` | 27 | gate⟺fitter coupling, propose/anchor, prior, sign mutation |
| `test_v711_plate_level_holdout_focal.py` | 12 | **B2 before/after**, surface mixing |
| `test_v711_objective_ladder.py` | 20 | no-fallback refusal, rescale, the two-records disagreement |
| `test_v711_focus_needle_datum.py` | 19 | scale gate incl. the 40× case, staleness kept-not-deleted |
| `test_v711_plate_level_worker.py` | 17 | stale-op abort, clamp, cancel-during-pause, exit guarantee |
| `test_v711_plate_level_wizard.py` | 22 | the gate table as data, accept blockers, nothing-installed-on-failure |
| `test_v711_microscope_focus_state_bug.py` | 3 | **B1 (fails on pre-fix code)** + repo-wide guard |

**Regression, run per suite, all green:** plate-z-plane-math (47) ·
plate-bottom-plane-controller (40) · print-z-plate-plane (18) ·
print-z-plate-bottom (14) · print-z-reference-vector (15) ·
needle-focus-template-store (31) · nikon-ti-microscope (101) ·
needle-bore-wizard (55) · microscope-bore-sign (16) · bore-focus-roi (24) ·
print-floor-refcount (12) · bore-gate-live-refresh (13) · optical-needle-datum
(16) · objective-calibration (26) · suite-hygiene (8) · z-retract (22) ·
calibration-revision (20) · last-known-calibration (21) · plate-types (38) ·
plate-z-autocal-per-well-focus (24) · plate-z-autocal-tab (11, updated for the
rename + 2 new) · plus a `gui.app` import smoke and a real offscreen
`CalibrationPage` build.

**Three mutation checks confirmed:** restoring `holdout=None` re-accepts the
140 µm plane · flipping `focal_sign` inverts the recovered tilt · the pre-fix
`.state` returns `None` and fails `test_v711_microscope_focus_state_bug`.

---

## Needs real-HW verification on ME3B V1, IN ORDER

1. Bore-wizard touch-off stores a **non-null** `microscope_focus_um` (B1 —
   nothing else works until this is true).
2. Focus↔needle datum. **It is captured at ONE moment and no other: the plate
   touch-off, with the microscope focused on the plate bottom and the needle tip
   touching it** — the two planes coincide there and nowhere else. Tick "The
   microscope is focused on the plate bottom" and confirm; the status line
   states the focus-axis value it stored. If you did not focus on the glass,
   untick it — the datum is then recorded as ABSENT, which is recoverable,
   rather than as a number that means nothing, which is not.
   Then, for the scale and sign: from that touch-off, step the needle **UPWARD**
   through ≥5 heights over ≥2 mm — the same upward moves that assign the bore
   offsets — refocusing on the tip at each, and fit the slope. `|scale|` must
   land within 1 ± 0.01, closure < 10 µm, sign agreeing with `z_up_sign`.
   **A scale outside the band is a bug, not a calibration — stop and find it.**
3. Single site, 4x only: the curve is unimodal with FWHM in 1–3 × DOF.
   **Look at the curve shape before trusting any number.**
4. Add 10x: parfocal offset repeatable across sites; feature re-acquired after
   the turret switch.
5. Full 5-site survey → residual and hold-out **better than 50 µm**; site-1 drift
   closure < 10 µm.
6. Force `focal_sign` inverted; confirm the hold-out **rejects** it.
7. Accept → restart → the plane reloads and re-validates; a Set Z Zero
   invalidates it loudly.
8. Print a flat pattern at the far corner from the anchor vs a run with tilt
   disabled. **The only end-to-end proof the plane does what it claims.**
9. Second plate of the same type: site 1 acquires inside the learned window.

**Do not skip step 3.** Two peaks ~170 µm apart means the sweep crossed both
coverslip faces — a systematic common to every site, invisible to every numeric
gate, and 3.4× the tolerance.

---

## Where the optical Z datum may be captured — operator correction

> *"when we set needle center using the needle cameras, these cameras are
> located at a distance far above the focal plane, so this should not collect
> the focal distance data. the focal distance data should be collected later in
> the workflow when we focus on the bottom of the plate and then move up to
> assign bore offsets."*

The datum pairs a **microscope focus reading** with a **needle Z**. It is only
meaningful when both are at the same physical plane.

**The needle-centring step cannot supply it.** The side cameras are bolted to the
frame far above the focal plane, so when the needle is centred in their
crosshairs its tip is nowhere the objective can focus — the focus axis is reading
an unrelated position entirely. Verified: `_needle_loc_record_origin_here` and
`_needle_loc_center_and_save` record only the needle-cam Z **fiducial** (a needle
height, used to pre-fill the plate Z guesses) and never touch the focus axis. A
repo-wide test now pins that exactly one production site passes
`microscope_focus_um=`, and that it is the touch-off.

**🐞 What WAS wrong: the touch-off captured the focus axis unconditionally.**
`_write_touchoff_capture` read `_microscope_focus_um()` whatever the drive
happened to be at. Step 4's text asks the operator to focus on the plate bottom
first, but nothing verified it — so a touch-off performed with the focus left
wherever it was stored a `microscope_focus_um` describing nothing, paired with a
needle Z genuinely at the glass. Every consumer treats the datum as ground truth
for converting focus to needle Z, so that error would propagate into the
plate-bottom height everywhere. Now gated on an explicit confirmation
(`_optical_datum_focus_um`), which records **absence** rather than a wrong
number, and the step reports which happened.

**⚠ This also corrects the scale/sign protocol in the approved plan**, which said
to park the needle "nowhere near glass" and autofocus on the tip. The tip is only
focusable near the focal plane, so that park position is unfocusable. The sweep
instead **starts at the plate bottom and moves UP** through the bore-survey
heights — away from the glass, which is the safe direction, and the moves the
bore step is making anyway.

---

## Issues & decisions

- **The turret ↔ calibration link, narrowed.** CLAUDE.md records it as
  deliberately deferred. `ObjectiveLadder` resolves µm/px **from the turret
  position, read-only, for one operation**, and never writes
  `current_objective_name` — writing it would change the µm/px every other
  surface reads, mid-run, from a background thread. The turret is restored on
  exit. A new gate compares `current_objective_name` against the live turret
  position, the first time those two records have ever been checked against each
  other.
- **No fallback for an uncalibrated objective.** Falling back to the live
  manager's µm/px is how "I literally just calibrated it" happened. It refuses.
- **`check_scale` band tightened 2 % → 1 %** after computing what it costs: the
  applied scale is exactly ±1, so error `e` leaks `e × Δfocus`, and across a
  ~1.15 mm tilt 2 % would spend most of the 25 µm budget while 1 % costs ~12 µm.
- **`MONOTONIC` is checked before `PEAK_AT_EDGE`**, otherwise it is unreachable
  (a monotone curve's argmax is always at an edge) and the less useful diagnosis
  wins.
- **The fake camera in the worker tests modulates texture, not brightness.**
  Brightness modulation makes the score bimodal — the feature vanishes as it
  crosses the background level and reappears inverted — and the estimator
  correctly refused it. Defocus reduces contrast; it does not shift the mean.

---

# Part 2 — Non-contact plate bottom (v7.11, second landing)

**Objective.** Replace the glass-CONTACT touch-off with an optical measurement of
the same number. Operator: *"first guess the plate bottom based on the needle zero
position and the known plate properties. then … the user focuses on a point on the
plate they think is the plate bottom. they set the plate bottom focus position.
then the focus moves up X (default 100) microns. then the needle moves down to be
autofocused at the focal plane … the user should be able to manually do the
motions, and the autofocus should tell the user what the best plane of focus has
been so far. The user can override this, and if they do, then a picture of the
current focus and needle type should be taken and used to train the focus model …
the human should confirm that this focus should be used as ground truth."*

Operator decisions (AskUserQuestion, 4): lives in the **needle-centring (bore)
wizard** · automatic mode **sweeps the focus with the needle parked** ·
**operator-selected ladder of margins, largest first** · **optical default, contact
touch-off retained as fallback**.

## The protocol

| Step | Action | Recorded |
|---|---|---|
| A | Operator focuses on the glass | `f0` |
| B | Focus rises by margin X | `f_X = f0 + X*focus_up_sign` |
| C | Needle descends **Z only, at the current XY**, once | parks near that plane |
| D | The **focus** sweeps through the stationary tip | `f_tip` from `peak_focus_um` |

Tip height above the glass is `h = (f_tip - f0)*focus_up_sign` — **measured, not
commanded** — so `B_k = z_needle - zdir*h/1000` does **not** inherit the error in
the guess that positioned the needle. Verified across all four
`(zdir, focus_up_sign)` combinations and placement errors up to 400 µm.

**Why the focus sweeps and not the needle.** The literal gesture is fine at 10x
(DOF 10.4 µm) and 20x (4.1 µm). At 4x the DOF is 57 µm, so a curve worth fitting
spans ~±150 µm — which at a 100 µm margin puts the tip **50 µm through the glass**.
Sweeping the focus moves only the objective, bounded by 25 % of a 16 mm working
distance. It also means the needle makes exactly ONE bounded descent per margin
and then holds still, which is the property the worker tests pin.

**Why a ladder.** Each margin is an independent estimate of one number; their
agreement is the only available verification. A single margin reproduces exactly
the weakness of `NeedleFocusTemplateStore.focus_to_needle_z_mm` — one point fits an
offset and must assume the scale. Default `[1000, 500, 200, 100] µm`; **largest
first is a safety ordering**, since only the first descent trusts the guess and
every later one is bounded by the measurement before it.

**⚠ The gate that is not optional.** The ROI scores **bore 0, the datum**, but a
bore protruding further reaches the glass first. Every margin must satisfy
`X >= max_bore_z_offset_mm*1000 + 50 µm`; a 200 µm bore at a 100 µm margin would be
100 µm *through* the plate while the screen reads a comfortable clearance. Refuses,
never clamps — a clamped margin measures a different height than the one reported.

**⚠ The scale gate is only meaningful if the data can support it.** Precision is
`sigma*sqrt(12)/(span*sqrt(n))`: with the default ladder ~0.33 % at 10x but
**1.83 % at 4x**, where sigma = DOF/6 ≈ 9.5 µm. Below the 1 % band the result is
reported **UNVERIFIED** rather than passed — a gate that could not have failed
reads on screen as a check that succeeded.

**⚠ A wrong `f0` is COMMON-MODE and the ladder cannot see it.** It shifts every
margin equally, so the agreement check passes and the answer is wrong by exactly
that shift. Pinned by a test asserting spread ≈ 0 while the bottom is 50 µm out.
This is why step A carries its own on-glass verification, and it is where a
coverslip's two faces would appear.

## Four defects fixed

| # | Defect | Consequence |
|---|---|---|
| **D1** | `PlateLevelWizard._cam()` read `host.camera_manager` (the page stores `_camera_manager`) and called `mgr.cameras()` when `cameras` is a **property** — `TypeError` into a bare `except` | `_cam()` returned `None` on hardware ⇒ the survey worker was built with `cam=None` ⇒ **part 1's optical survey could not grab a single frame.** The stub host in my own tests defined `camera_manager`, so nothing could see it. Fixed, plus `_mgr()`, plus a guard test resolving both wizards' host accessors against the **real `CalibrationPage`**. |
| **D2** | `_zoff_estimate_from_needle_cam` pushed `set_plate_bottom_z` with **no `source=`** | A guess was indistinguishable from a taught value, so the plate-level wizard's "plate bottom is only estimated" warning could never fire from the only path producing an estimate. |
| **D3** | The sole `NeedleFocusTemplateStore` writer never passed `needle_type=` / `needle_bore_um=` / `focus_score=` | `reference_focus_score()` returned `None` for every capture on disk and the needle type survived only as a substring of the key. The bank this work trains against was **write-only in production** — `load_patches` / `focus_to_needle_z_mm` have zero readers in `gui/`. |
| **D4** | `plate_level_wizard.focus_sample` was declared, emitted per frame and disconnected in teardown — never connected | No feedback during a multi-minute survey. Now drives one status line, not an append: a rung is ~39 frames and appending each would bury the per-site verdicts. |

## "Training the focus model" — what it honestly is

Not ML. Three testable things, all unlocked by D3: a **multi-defocus template bank**
per needle type (`TM_CCOEFF_NORMED` is invariant to intensity but **not to blur**),
a usable **reference focus score**, and a **one-parameter learned bias** — the
median `operator_focus - auto_focus` over confirmed captures. If the gradient
metric peaks reproducibly 8 µm off a 30 G tip, that is measurable and correctable;
storing only the adopted value would leave nothing to measure it from.

Captured on **every** adoption, not only overrides (a confirmed-correct automatic
pick is an equally valuable positive example), each carrying `adopted_by`,
`auto_focus_um`, `margin_um` and `ground_truth` — the last set **only** by an
explicit operator confirmation, mirroring `_s4_focus_ok`: absence is recoverable,
wrong is not. Only ground-truth captures feed the bias, and it needs two before it
reports anything (one disagreement is an anecdote).

The UI keeps **best sample so far** (a running max over jogged frames, ~half a jog
step) separate from the **fitted peak** (sub-step, carries a sigma). An overridden
rung is stored with `focus_sigma_um = 0.0` so it can never be weighted as a fit.

## Files

**New:** `SupportClasses/PlateBottomOptical.py` — pure, GUI-free; imports
`SCALE_TOLERANCE` and `check_scale` from `PlateFocusDatumStore` rather than copying
them (the `MIN_TRIANGLE_AREA_MM2` precedent).

**Modified:** `gui/widgets/needle_bore_wizard.py` (step-4 mode selector, ladder
editor, live readout, override, `PlateBottomRungWorker`, D3) ·
`SupportClasses/NeedleFocusTemplateStore.py` (training fields, conditional-emit, so
a capture supplying none of them round-trips byte-identically; plus
`ground_truth_captures`, `focus_bias_um`) · `gui/widgets/plate_level_wizard.py`
(D1, D4) · `gui/pages/calibration.py` (D2).

## Tests

`test_v711_plate_bottom_optical.py` (34) · `test_v711_plate_bottom_worker.py` (18)
· `test_v711_plate_bottom_wizard.py` (34) · `test_v711_host_accessor_contract.py`
(7) · +8 in `test_v75x_needle_focus_template_store.py`. **v711 total 282 green.**

**4 mutations confirmed CAUGHT:** drop the longest-bore term from the ladder gate ·
move the needle at each swept position (i.e. sweep the needle instead of the focus)
· revert D1's accessor · strip D3's kwargs.

⚠ **One of my own tests was too weak and a mutation caught it.** The D2 guard
asserted `'source="estimated"' in inspect.getsource(...)` — which matched **my own
explanatory comment**, so removing the keyword from the CODE left it green.
Rewritten to walk the AST for a `source=` keyword on the `set_plate_bottom_z` call.
Same trap CLAUDE.md already records for a v7.10 guard that passed on an import line
alone.

Two bugs the tests found in my own new code: the override path wrote **two**
training captures for one decision (`_adopt_rung` now takes `ground_truth=` rather
than the caller writing a second), and the worker used only `plan.rungs[0]` while
leaving the focus at the sweep's last position — so a tip that landed off-target
was never found (the search was ~3·DOF wide, far narrower than the guess error it
must absorb) and the operator was shown a blurred tip when asked to confirm it.
Now it walks the coarse→fine ladder via `next_rung`, sizes the search from
`SEARCH_FRACTION` of the margin, and parks ON the fitted peak.

Regression green per-suite: needle-bore-wizard (55) · optical-needle-datum (16) ·
bore-focus-roi (24) · microscope-bore-sign (16) · bore-gate-live-refresh (13) ·
needle-focus-template (39) · plate-z-plane-math (47) · plate-bottom-plane-controller
(40) · print-z-plate-plane (18) · nikon-ti (101) · plate-z-autocal-tab (11) ·
per-well-focus (24) · bore-offset (93) · multibore (36) · camera-orientation-audit
(50) · calibration-revision (20) · z-retract (22) · bore-safety (65) ·
suite-hygiene (8), plus a `gui.app` import smoke and an offscreen build of the real
`CalibrationPage` confirming D1 end-to-end (`_cam()` now resolves to a real camera
instead of `None`).

## Needs real-HW verification on ME3B V1, in order

1. Step 2's guess displays and is tagged **estimated** (D2).
2. Needle retracted, focus on glass, **Set plate-bottom focus**.
3. **Look at the curve shape at the largest margin before trusting any number.**
   Two peaks ~170 µm apart = the sweep crossed both coverslip faces — common to
   every margin and therefore invisible to the spread check.
4. Largest margin alone: measured `h` vs commanded X. A disagreement here is the
   guess being wrong, which is what this measurement exists to absorb.
5. Full ladder → spread < 25 µm; scale within 1 % of ±1 at 10x (expect
   *unverified* at 4x, and that is the correct report, not a failure).
6. Override one margin deliberately → the capture lands with `needle_type` **and**
   `focus_score` populated (both dropped by the old writer) and
   `adopted_by="operator"`.
7. Restart → plate bottom and datum reload; a Set Z Zero invalidates loudly.
8. Contact fallback behaves exactly as before.
9. **The proof:** run the contact touch-off on the same well and compare it to the
   optical bottom. They must agree within tolerance — the only end-to-end
   confirmation that the optical number is the same physical plane.
10. Re-run part 1's bed-level survey: with D1 fixed it should acquire frames at
    all, which it could not do before.

## Deliberately not done

- **The legacy `_auto_z_*` needle-descent hill-climb is now superseded** — this
  measures the plate bottom without contact, and part 1 measures the tilt without
  the needle. It is left in place and working: the Custom-tab button still opens it
  and it owns `_zoff_live_view`. Retiring it is its own separately-verified change.
- `_auto_z_capture_baseline` still does a blocking `time.sleep(0.2)` on the GUI
  thread — in code being superseded, so left alone.
