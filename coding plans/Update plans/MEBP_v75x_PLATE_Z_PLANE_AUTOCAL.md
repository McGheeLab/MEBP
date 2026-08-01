# MEBP v7.5.x — Plate Z Auto-Cal rework: two guided modes + plate-bottom Z anywhere

Supersedes `MEBP_v75x_PLATE_Z_AUTOCAL_PER_WELL_FOCUS.md` and
`MEBP_v75x_PLATE_Z_AUTOCAL_TAB.md`.

---

## Objective

Rework the **Plate Z Auto-Cal** tab around the fact that the plate bottom is a
**flat but tilted plane**, and make that plane the authority for plate-bottom Z
everywhere — so the software always knows the plate bottom Z wherever it is on
the plate.

Operator's two modes:

- **Mode A — manual needle touch per well.** Travel to a selected well centre →
  operator focuses the scope on the well bottom → the needle auto-lowers to the
  initial guess → operator fine-jogs Z and presses **Set** → operator clicks the
  needle centre (records the needle's offset from camera centre AND captures an
  in-focus needle image) → retract, next well, where the software gives live
  feedback on whether the needle looks like the in-focus reference → fit the
  plane on the last well.
- **Mode B — motorized objective focus stage.** Focus at 3 operator-chosen
  locations, entering the focal-plane readout (real units) at each → then ONE
  needle touch in a well centre (with the needle-centre click and the
  fine-control "at bottom" mark) **anchors** the focal plane to needle Z.

Both replace the previous automated focus-peak descent.

### Why this is worth doing — the tilt was measured and then discarded

`WellSetup.PlaneResult.z_at()` is called in exactly two places, both
navigation-only inside `calibration.py` (`_navigate_to_well:13178`,
`_goto_well:13467`). Prints resolve plate bottom from a **single scalar** and
`StageController.print_height_to_zref()` **took no (x, y)**, so per-well tilt
compensation was structurally impossible.

The machine's own `settings.json` shows the two datums were never reconciled:
every plate block stores the same plate-local plane
`{a: -0.000725, b: -0.019862, c: 5.600, r_squared: 1.0}` while `plate_bottom_z`
per plate is `0.09 / 14.54 / 1.83 / 6.29 / -14.42 / -10.42` — the intercept
matches none of them. `b = -0.0199 mm/mm` over a 24-well plate's ~58 mm row span
is **~1.15 mm of un-compensated Z**, i.e. 2–10× a typical 0.1–0.5 mm print
height.

### Operator decisions (AskUserQuestion)

Wells = user-selectable set ≥3 · focal readout in real units (mm/µm) ·
needle-centre click used for template centring **and** XY refinement **and**
persisted for reuse by workflows (spheroid pick & place target selection) · the
two modes **replace** the focus-peak flow · **the plane drives print Z** ·
template captured at **every well, keep all** · F-stage sign/units **asked in the
UI, never assumed** · Mode A's initial guess comes from the existing **Plate
Bottom Z reference**.

---

## Key design decisions

**D-1 — the plane is an ANCHORED GRADIENT, not `a·x + b·y + c`:**

```
plate_bottom_z(x, y) = z0_zref_mm + sx·(x - x0_um)/1000 + sy·(y - y0_um)/1000
```

where `(x0_um, y0_um, z0_zref_mm)` **is the taught touch-off point**. This is the
plan's central safety property, not a style choice:

- degrade-to-scalar is exact and free (`sx = sy = 0` ⇒ the scalar, bit-for-bit);
- `c` is the value at the frame ORIGIN and is meaningless once the origin moves
  (a Set Zero, or plate-local vs stage); `z0` is the value at a physical point and
  is translation-invariant;
- **the 5.5 mm disaster above becomes structurally impossible** — a fit can only
  contribute a *gradient*, never move Z at the anchor;
- "re-anchor after a re-mount" is one touch-off: replace `z0`, keep `sx`/`sy`.

**D-2 — frame: XY in absolute stage µm, Z in zero-ref mm, slopes mm/mm.**
Absolute stage XY is the frame taught data already lives in (`SafetyLimits`
documents the envelope as absolute and "unaffected by Set Zero"; CLAUDE.md:
taught/warped positions are already absolute stage µm and must never be
re-signed), so `plate_axis_sign` / `plate_flip_180` never enter the consumption
path. Plate-local mm is disqualified: evaluating at an arbitrary stage XY would
need the inverse of the `affine_tps` plate warp, and Mode B's locations are not
wells at all.

**D-3 — the print-floor clamp uses the plane at the commanded XY** (explicit hint
→ cached XY → scalar), NOT a worst-case bound. Verified facts:
`_apply_print_floor_raw` is called only from `move_z_absolute:4123` /
`move_z_relative:4190` — once per **Z move**, never per XY segment (no Z is
commanded during `PRINT_PATH`); `PositionPoller.note_xy:1255` back-fills the cache
from the print path's own 25–31 Hz direct reads and explicitly "keeps the cache
live even while polling is SUSPENDED"; and every `MOVE_Z` is preceded by
`MOVE_XY` + `_wait_for_xy_settle`. Meanwhile a worst-case floor would hold the
needle ≥1.15 mm above glass over half the plate — turning a rare crash into a
**guaranteed** silent mid-air print. Worst-case is kept for the pre-flight
advisory and the tilt readout only.

**D-4 — `use_plate_tilt` defaults OFF for any install that already has a
`calibration` section**, ON only for a freshly fitted validated plane. Existing
installs see bit-for-bit unchanged Z on upgrade.

**D-5 — hold-out is the acceptance gate, not R².** At exactly 3 points the
anchor-constrained fit has 2 unknowns and 2 equations, so it is exact and R² is
identically 1.0 — which is precisely the `r_squared: 1.0` on disk. It cannot
detect a bad touch-off; predicting a held-out point can.

**D-6..D-10** — new pure module (not an extension of the plate-local
`WellBottomDetector`) · needle offset on the controller + patches in a new store ·
needle offset captured/persisted/exposed now but **applied to workflows later**
(click-to-centre must NOT apply it while click-to-pick must) · live needle-match
feedback is **advisory only**, ROI-scoped · one active plane per plate key, never
merging needle-touch and focal points into one fit.

Full rationale in the session plan file.

---

## Files Modified

### New
| File | Purpose | Status |
|---|---|---|
| `SupportClasses/PlateZPlane.py` | Pure, GUI-free plane model: `ZPlanePoint`, `PlateZPlane` (anchored gradient), `plate_plane_z_zref_mm` / `job_plane_z_zref_mm` evaluators, `fit_gradient_through_anchor`, `holdout_error_mm`, `from_needle_touches`, `from_focal_readings`, `tilt_is_plausible`, `triangle_max_area_mm2`, frame-tagged serialisation. numpy imported lazily. | ✅ done |
| `tests/test_v75x_plate_z_plane_math.py` | 48 tests: evaluator + polarity, anchor exactness, fit/degeneracy, hold-out, focal sign, plausibility, serialisation, re-anchor. | ✅ done |
| `tests/test_v75x_plate_bottom_plane_controller.py` | 40 tests: degrade-to-scalar identity, every trust rule, reporting, job stamping. | ✅ done |
| `SupportClasses/NeedleFocusTemplateStore.py` | Per (camera+objective+needle) in-focus needle patches (**list** per key — the operator asked for one per well, all kept) + the needle↔camera-centre offset (mean + spread) + median reference focus score. Mirrors `ReanchorFeatureStore` (atomic write, `get_store()`, guarded cv2, `MEBP_NEEDLE_TEMPLATE_DIR`). Bounded history prunes its own PNGs. | ✅ done |
| `tests/test_v75x_print_z_plate_plane.py` | 18 tests: job-builder **byte-identity** without a plane, per-well Z + constant standoff on both polarities, floor-XY hints, malformed-stamp fallback. | ✅ done |
| `tests/test_v75x_needle_focus_template_store.py` | 31 tests: round-trip, multi-capture, bounded history, atomicity, corrupt-index recovery, path-escape refusal, derived offset/spread/median, env isolation, **offset sign as an inverse property**. | ✅ done |

### Modified
| File | Change | Status |
|---|---|---|
| `SupportClasses/StageController.py` | Plane datum + trust validation + evaluation API (see below). `set_plate_bottom_z` gains `at_xy_um=` / `source=`. `print_height_to_zref` / `zref_to_print_height` / `print_floor_violation` gain optional `x_zref_mm`/`y_zref_mm`. | ✅ done |
| `SupportClasses/PrintManager.py` | `PrintSettings.plate_z_plane_zref_mm` + `print_height_above_bottom_mm` (both default `None` = legacy); new `well_print_z_zref_mm()`; per-well Z + `hop_z` in `build_well_plate_job`; `MOVE_Z` `floor_x_mm`/`floor_y_mm` hints emitted only when a plane is stamped. Start/resume coherence check still pending (Stage 3). | ✅ mostly (coherence check → Stage 3) |
| `SupportClasses/StageController.py` (needle offset) | `set/get_needle_camera_offset_um` + `needle_target_xy_for_feature_um` — ONE function owns the sign so no consumer can re-derive it wrongly. Passthrough when unmeasured. | ✅ done |
| `SupportClasses/StageController.py` (clamp) | Plane-aware `_apply_print_floor_raw(raw_z, xy_hint_mm=None)` + rate-limited warning. | ⬜ pending (Stage 3, needs rig) |
| `gui/pages/calibration.py` | Rebuild the tab; Mode A/B state machines; well selection; needle click + template; persistence of plane **and points**; orientation guard; anchor↔scalar reconciliation; the defect list below. | ⬜ pending |
| `gui/app.py` | `_update_print_floor_datum` pushes the plane + anchor XY + footprint bbox. | ⬜ pending |
| `gui/widgets/detection_worker.py` | `DetectionMode.NEEDLE_TEMPLATE_VERIFY` + combined `needle_verify_updated`. | ⬜ pending |

### Controller API added (Stage 1)

```python
set_plate_bottom_z(z_zero_ref_mm, at_xy_um=None, source=None)   # kwargs are new
get_plate_bottom_z()                    # UNCHANGED — the ANCHOR scalar
get_plate_bottom_anchor_xy_um() / get_plate_bottom_z_source()
set_plate_footprint_bbox_um(bbox) / get_plate_footprint_bbox_um()
set_plate_tilt_enabled(enabled) / plate_tilt_enabled()
set_plate_z_plane(plane_or_dict_or_None) -> (accepted, reason)
get_plate_z_plane() / active_plate_z_plane()
plate_bottom_z_at_um(x_um, y_um) / plate_bottom_z_at_zref_mm(x_mm, y_mm)
plate_bottom_z_extremes_zref(footprint_um=None) / plate_z_tilt_span_mm()
plate_z_plane_for_job()                 # the ONE job-stamp source, zero-ref mm
print_height_to_zref(h, x_zref_mm=None, y_zref_mm=None)
zref_to_print_height(z, x_zref_mm=None, y_zref_mm=None)
print_floor_violation(z, x_zref_mm=None, y_zref_mm=None)
```

`get_plate_bottom_z()` deliberately stays the **anchor scalar** — never the plane
at plate centre or the last XY — because it is the number the operator taught and
reads back on the Calibration page, the jog context panel, the XZ side view and
the readiness report. Position-aware callers use `plate_bottom_z_at_um`; the tilt
story comes from `plate_bottom_z_extremes_zref`.

**Trust rules** (all must hold for `status="active"`): stage frame · not
degenerate · ≥3 points · a taught scalar exists · needle-zero epoch matches
(±0.01 mm) · plate orientation matches · anchor agrees with the scalar (±0.05 mm) ·
tilt plausible · at ≥4 points `residual_max ≤ 50 µm` · `holdout ≤ 50 µm` when
present. A rejected plane is **retained for the UI but never used**, and is
exactly equivalent to no plane.

---

## Implementation Steps

- [x] **Stage 1 — pure plane model + controller datum.** No caller sets a plane,
      so this is a provable no-op in the app. 87 new tests; existing print-Z
      suites green untouched.
- [x] **Stage 2 — print plumbing (still a no-op by default).** `PrintSettings`
      fields, `well_print_z_zref_mm()`, per-well Z + `hop_z` in
      `build_well_plate_job`, per-well floor-XY hints. Nothing stamps a plane yet
      (that is Stage 8), so every existing print is byte-identical — pinned by
      `TestByteIdentityWithoutAPlane`. `PrintTrajectoryPlanner._well_print_z` and
      the Print Setup / Quick Print stamping remain in Stage 8.
- [ ] **Stage 3 — plane-aware print floor** (`xy_hint_mm` into
      `_apply_print_floor_raw`, start/resume coherence check, rate-limited log).
      The per-well `floor_x_mm`/`floor_y_mm` hints the clamp will consume are
      already emitted by Stage 2. **Needs rig.**
- [ ] **Stage 4 — persistence.** New `plate_bottom_plane` calibration key
      carrying the plane **and the touch points**; provenance-based invalidation;
      orientation-guard extension; `num_points` de-hardcode; anchor↔scalar
      reconciliation.
- [x] **Stage 5a — `NeedleFocusTemplateStore`** + the controller's needle-offset
      accessor and sign contract.
- [ ] **Stage 5b — `DetectionMode.NEEDLE_TEMPLATE_VERIFY`** in
      `gui/widgets/detection_worker.py` (off-GUI-thread template match + ROI focus
      in one combined signal).
- [ ] **Stage 6 — Mode A** UI + state machine (replaces coarse/fine). **Needs rig.**
- [ ] **Stage 7 — Mode B** + focal-direction measurement + 4th-point verify +
      read-only `get_focus_axis_um` + "Re-anchor plane (1 well)". **Needs rig.**
- [ ] **Stage 8 — enable per seam, one commit each** (`PrintTrajectoryPlanner`
      LAST; it is already flagged `ZDIR=+1`-unsafe). **Needs rig.**
- [ ] **Stage 9 — needle-offset application** to consumer workflows, behind
      `device_profile.apply_needle_cam_offset`. Own update plan.

---

## Defects to fix inside this work

| Location | Problem | Status |
|---|---|---|
| `calibration.py:14010` `_get_primary_camera()` | Returns the first **running** camera, not the microscope — with a needle/Monitor cam up, the focus score, template and **the click→offset** are computed against the wrong camera, silently poisoning the persisted offset. | ⬜ |
| `calibration.py:12411`, `12165` | `_auto_z_enter_recorded` / `_cancel_auto_z` call raw `move_z_absolute(self._safe_z)`, which **descends** if `_safe_z` is mis-set. Use `ensure_retracted_to` (raise-only). Live latent bug. | ⬜ |
| The tab never retracts on cancel/finish | Leaves the needle inside a well next to glass. | ⬜ |
| `calibration.py:12218-12228` | `navigate` inlines its own lookup and **silently skips** wells with no position — a 3-well run can finish with 2 points and never fit. | ⬜ |
| `calibration.py:13116` `_try_fit_z_plane` | Sets `_z_plane_result` but never emits/saves, so coefficients survive only if some other save happens to fire. | ⬜ |
| `WellSetup.py:613` | `lstsq`'s `rank` discarded and R² ≡ 1.0 at n=3 ⇒ collinear points give a silently wrong tilt. | ✅ addressed in the new module (keeps rank + singular values, reports max residual and hold-out) |
| `calibration.py:13664-13666` | Orientation guard explicitly **keeps** the Z plane, whose plate-local slopes flip with `plate_axis_sign()` on a 180° remount. | ⬜ |
| `calibration.py:12186` `_auto_z_move_to_h` | Uses module `ZDIR` while the print path uses `print_z_dir()`. | ⬜ |
| `calibration.py` `_zauto_tab_index = 4` | Hardcoded; assign from `addTab`. | ⬜ |
| `StageController.py:2564` | Floor warning fires on **every** call — floods the log during a print. | ⬜ (Stage 3) |
| `plate_bottom_z` had no provenance | Could be a needle-cam / plate-type **estimate**; anchoring a plane to an estimate propagates the error plate-wide. | ✅ `set_plate_bottom_z(..., source=)` + `get_plate_bottom_z_source()` |
| `PositionPoller.note_xy:1255` has no timestamp | Cache staleness undetectable; mitigated by preferring the explicit floor XY hint. | ⬜ (Stage 3) |

---

## Testing Notes

**Stage 1, all headless, no Qt / camera / event loop.** Controllers are built via
`__new__` with only the attributes under test — the same partial-stub pattern
`test_v75x_print_z_plate_bottom.py` uses — so every new attribute access is
`getattr`-guarded (pinned by
`test_partial_stub_without_new_attributes_survives`).

```
python -m unittest tests.test_v75x_plate_z_plane_math \
                   tests.test_v75x_plate_bottom_plane_controller \
                   tests.test_v75x_print_z_plate_plane \
                   tests.test_v75x_needle_focus_template_store      # 136 OK
```

Pinned invariants worth calling out:

- **Degrade-to-scalar identity** over a 3×3 XY grid for: no plane · plane present
  but tilt disabled · rejected plane · no XY supplied · outside the taught region ·
  a measured-but-level plane. Plus
  `print_height_to_zref(h) == print_height_to_zref(h, None, None)` and the same
  for `zref_to_print_height` / `print_floor_violation`.
- **`test_legacy_plate_local_plane_does_not_move_z_by_5mm`** — feeds the literal
  on-disk `{a: -0.000725, b: -0.019862, c: 5.600}` against the taught `0.090`,
  asserts rejection and that every position still reports `0.090`.
- **Anchor exactness** — the taught Z survives the fit bit-for-bit, so a fit can
  never move Z at the touch-off point.
- **Polarity** — `extremes_zref` picks the correct corner under `z_up_sign = ±1`
  (on ME3B V1 a larger zero-ref Z is *deeper*, so a raw numeric comparison would
  be backwards).
- **Frame separation** — the µm-keyed live plane and the mm-keyed job plane are
  NOT interchangeable: handing one to the other's evaluator raises `KeyError`
  rather than silently computing a 1000×-wrong offset.
- **Hold-out catches a bad touch-off** (400 µm error on one of four points) that
  R² cannot see.

Additional invariants pinned by the Stage 2 / 5 suites:

- **Job-builder byte-identity** — with no stamped plane the emitted `MOVE_Z` /
  `hop_z` sequence is unchanged on both polarities, across layers and
  multi-segment wells, and no `floor_*_mm` params appear at all. Also asserted as
  a full plan-signature equality between "fields absent" and "fields explicitly
  `None`".
- **Both stamps required** — a plane without `print_height_above_bottom_mm` (or
  vice versa) is ignored, because a plane alone could only *shift* the plate-wide
  Z rather than recompute it.
- **Constant standoff** — the actual point of the feature: `z − local_bottom`
  equals the intended height at every well, on `z_up_sign = ±1`.
- **Malformed / wrong-form stamp degrades, never raises** — including handing the
  live µm-keyed plane where the mm-keyed job form belongs, which falls back to the
  plate-wide Z instead of computing a 1000×-wrong offset.
- **Needle-offset sign as an inverse property** —
  `feature == needle_target_xy_for_feature_um(feature) + offset`, plus a
  passthrough when unmeasured so consumers can adopt it unconditionally.

**Regression, green untouched:** `test_v75x_print_z_plate_bottom` ·
`test_v75x_print_z_reference_vector` · `test_v75x_cal_z_envelope_no_clobber` ·
`test_v75x_z_axis_unified_setup` · `test_v75x_last_known_calibration` ·
`test_v75x_plate_types` · `test_v75x_z_retract_before_xy_travel` ·
`test_v75x_jog_direction_z_up_sign` · `test_v75x_zp_envelope_absolute` ·
`test_v75x_gentle_descent_slow_final` (176) · all `test_v75x_print*` (145) ·
`test_v77_print_readiness` + `test_v76_feed_plan_print` +
`test_v75x_quick_print_pick_and_place` + `test_v75x_full_print_workflow` +
`test_v75x_printing_mode_calibrated_wells` (148) ·
`test_v75x_multi_object_print_seam` + `test_v75x_simple_print_manager` +
`test_v75x_quick_print_travel_split` (61) · `test_v75x_plate_z_autocal_tab` +
`test_v75x_plate_z_autocal_per_well_focus` (the flows Stage 6 will replace).

**Two pre-existing failures, confirmed NOT from this change** (reproduced with
`SupportClasses/PrintManager.py` + `StageController.py` stashed):
`test_v75x_quick_print_workflow::TestJobBuilding::test_settings_use_safe_z_and_flow`
and `::TestButtonGating::test_enabled_when_ready` — both driving
`_speed_pct_spin`, the knob the v7.6 two-param rework retired (already noted in
CLAUDE.md), plus one spheroid `simulated move failure` case in the same family.

**Real-hardware verification (ME3B V1)** — Stages 1–2 and 4–5 need no rig:

1. Touch off 5 wells (A1, A6, D1, D6, centre) in ONE needle-zero epoch.
2. **Acceptance gate:** fit on A1/A6/D1, then *predict* D6 and centre — residual
   must be < 0.05 mm. R² is worthless at 3 points; prediction is the only
   evidence.
3. **Magnitude check:** the fitted `sy` should be ≈ −0.02 mm/mm if the on-disk
   plane is real. A much smaller fresh slope means the old plane had a bad point —
   discard it rather than carry it forward.
4. **Equal-gap test:** 0.10 mm print height at A1 and at D6 must give the *same*
   needle–glass gap (before this change D6 is off by ~1.1 mm).
5. **Punch-through refusal:** at the shallowest well command −0.5 mm height; the
   clamp must hold at the *plane* value and log once.
6. **No-move-on-upgrade:** launch against the existing `settings.json` — readiness
   must report plate bottom 0.09 (not 5.60) plus "legacy plate-local plane —
   re-teach"; jog Z and confirm zero change.
7. **Epoch guard:** do a Set Z Zero → the plane must flip to rejected with
   "needle zero changed since the plate plane was measured".
8. Needle-match verdict green at the taught Z, red 0.3 mm off (tune the
   0.65/0.50/0.7–1.4 thresholds against real frames); needle-centre offset
   measured independently matches the clicked value.
9. Retract discipline on every inter-well hop; cancel mid-run ends at Safe Z.
10. Mode B: if the Prior third axis is live, confirm `get_focus_axis_um` tracks
    the knob; run the focal-direction measurement and check the derived sign
    against the physical knob; confirm the 4th-point verify catches a
    deliberately inverted sign.

---

## Issues & Decisions

**The slope plausibility gate was initially set too tight — caught by its own
test.** The first cut used `MAX_PLAUSIBLE_SLOPE_MM_PER_MM = 0.005` (≈0.3°), which
**rejects the tilt this machine actually measured** (0.0199 mm/mm ≈ 1.14°). That
would have made the feature unusable on the very rig it was written for. The
slope gate's job is to catch a **unit slip** (mm vs µm = 1000×), not to adjudicate
whether a real measurement is believable, so it is now 0.05 mm/mm (≈2.9°) — still
catching a slip by a factor of hundreds — and the **span** gate
(`MAX_PLAUSIBLE_SPAN_MM = 2.0`) is the physical arbiter, because it bounds the
thing that actually matters (how far the datum moves) and is what catches a single
bad touch-off skewing the fit. Both directions are pinned:
`test_slope_limit_catches_a_unit_slip` and
`test_slope_limit_accepts_this_machines_real_tilt`.

**Two plane forms, deliberately non-interchangeable.** The live plane is keyed
`x0_um`/`y0_um` (absolute stage µm); the job-stamped copy is keyed `x0_mm`/`y0_mm`
(zero-ref mm, the frame the print path works in). Because the key *names* differ,
passing one to the other's evaluator raises `KeyError` instead of silently
producing a 1000×-wrong offset. Pinned by
`test_job_form_and_um_form_are_not_interchangeable`.

**Trust is never restored from disk.** `PlateZPlane.from_dict` always returns
`status="unvalidated"`, so a plane is re-adjudicated against the *current* needle
zero, orientation and taught scalar on every load. Pinned by
`test_status_is_never_restored_from_disk`.

**Rejected planes are retained, not discarded.** `set_plate_z_plane` stores the
plane with `status="rejected"` and its reason so the UI can explain the refusal
and offer a re-teach, while `active_plate_z_plane()` returns `None` so no consumer
can use it. Target and floor both gate on the same accessor, which is what stops
them from ever disagreeing about whether the tilt is real.

**Fitting the gradient of the differences, not a free plane.** The anchor is exact
*by construction* because the fit solves for `(sx, sy)` minimising
`dz_i - sx·dx_i - sy·dy_i` about the anchor, rather than fitting `a·x + b·y + c`
and reading its value at the anchor. That exactness is the safety property, not an
optimisation detail — it is what makes "no fit can move Z at the taught point"
true rather than approximately true.

**`WellSetup.WellBottomDetector` left untouched.** It is keyed by well *name* in
plate-local mm and is still used by the Custom → Full Wizard; the new module is
additive. Its `WellSetupModel.z_offset` / `effective_z_offset` remain a dead end
(zero external call sites) and should get a docstring pointer to the live
mechanism so nobody wires new code into it.
