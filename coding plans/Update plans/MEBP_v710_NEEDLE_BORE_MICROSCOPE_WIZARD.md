# MEBP v7.10 — Needle/bore calibration: microscope wizard, live gate, per-bore Z

## Objective

Three operator-reported problems with **Calibration → Needle Location → Bore mount
offsets**, plus the two already-built-but-unwired calibrations they unlock.

> 1. all bores are located in the same z plane.
> 2. it has a warning to connect the xy stage, but i can confirm the xy stage is connected.
> 3. bore offsets should be calculated in the xy frame which means that we should have a
>    needle cam view, and a microscope cam view toggle, so we can go through the workflow of
>    step 1 needle cam x,y,z zero position relative to the stages current xy location then
>    step 2 we get our reference z heights … then step 3 the needle is lowered into the plate
>    z bottom so we can create the offsets for the needle bore from the center of the camera
>    frame or multiple bores centers from the cameras center.

**Operator decisions (AskUserQuestion, 4):** the Needle Location tab becomes a **wizard every
needle goes through, single-bore included** (a single-bore needle still needs its XY offset
from the microscope centre), and the final step **confirms the plate Z bottom** and captures
the **distance between the microscope's optical focus and the needle tip** · bores are measured
at a **safe height above the glass** · per-bore Z is **manual jog + a live autofocus score**,
not an automatic sweep · **all bores are visible in one microscope frame**.

---

## Root causes (verified, not inferred)

### Bug A — the stale "Connect the XY stage" warning

Neither plausible culprit was guilty: `StageController.is_xy_connected` is a proper
`@property` used bare (no bound-method truthiness bug), and `CalibrationPage.controller` is
assigned once from the same long-lived controller the main window polls (not stale).

The cause is *when* the gate runs. `_bore_cal_refresh()` had exactly four callers — page build,
`set_hardware_config`, after a capture, after a clear — and **none fire when the XY stage
connects**. The page is constructed at startup *before* the operator connects hardware, so the
gate evaluated once against a disconnected stage, painted the yellow refusal, called
`setEnabled(False)` on every row button, and nothing re-ran it. The tab stayed dead for the
whole session; the only accidental cure was re-saving a hardware config, which happens to call
`set_hardware_config`.

The freeze was **wholesale, not just the XY clause** — the camera clause reads a live frame and
is equally unset at build time, so any fix that only re-checked `is_xy_connected` would have
left the tab dead behind a camera refusal.

### Bug B — every bore records the same Z

`_bore_cal_capture` computes `dz` as the difference of the two stage-Z readings taken when each
"Bore N is on the crosshairs" button is pressed. The entire flow — intro text, both tooltips —
tells the operator to jog **XY** onto the side-camera crosshairs, so Z never changes between
captures and `dz == 0.0` **exactly**. The one path that could have produced a real per-bore Z
(the side-camera row refinement) needs four fresh edge picks *and* a checkbox, and
`_needle_loc_reset()` wipes the picks after every capture — so it never applies past bore 1.

Consequence: `PickAndPlaceManager._descend_z_mm` is a dormant identity function and the
assembly's coplanarity is **assumed, never verified**.

---

## ⚠ The sign convention — the hard part, and a correction

The first draft of this work derived `offset_k = pto(P_0) − pto(P_k)`. An adversarial review
derived the exact opposite. Re-checking showed **the two existing authorities in the repo
contradict each other**:

* `NeedleBoreCalibrationStore`'s worked example: a bore protruding +320 µm in world +X is
  reached with the stage at `S_1 = S_0 − 320` — stage −X moves the needle −X *relative to the
  side camera*.
* `TwoCameraNeedleAligner.offset_from_edge_clicks` returns the needle's displacement `n` from
  the optical axis, and its caller commands `move_xy_relative_um(+n)` to **recentre** — stage
  +δ moves the needle **−δ** relative to that camera.

Opposite senses. An attempt to settle it by **measurement** (probe with a known stage move) was
designed, implemented, and then **abandoned as unmeasurable**: the needle and the microscope sit
on the same body in *every* rig configuration, so the tip never moves in that view and the probe
would always read zero. That dead end is what exposed the correct derivation.

**The resolution is that the lab frame never enters.** The one code-verified fact is the
contract of `CameraManager.pixel_to_stage_offset` (`pto`), used identically at every live-view
click site: *a plate feature at pixel P has stage-coordinate label `c = current_xy + pto(P)`*.
Bore k's tip appears at pixel `P_k`, so at stage `S` bore k sits over the plate point labelled
`S + pto(P_k)`. To put it on a target labelled `T`: `S = T − pto(P_k)`; the store's convention
is `S = T − offset_k`; therefore

```
offset_um(k) = pto(P_k) − pto(P_0)          ← bore MINUS datum
```

Because the pick workflow builds its target the *same* way (`T = current_xy + pto(click)`),
measurement and consumption are consistent **by construction** whichever body the stage carries.
Reasoning via "the bore protrudes +320 µm in world +X" introduces an unobservable intermediate
and is precisely where the confusion came from.

**Note the argument order is the OPPOSITE of `offset_from_centred_positions` (`datum − bore`),
and that is correct** — one subtracts stage positions the stage was *driven* to, the other
subtracts pixel-derived labels of where the bores *already are*. The asymmetry is documented at
length in the module and must not be "tidied".

The same clicks also yield `needle_camera_offset_um = pto(P_0)` — the raw click, **no**
negation — matching `StageController.needle_target_xy_for_feature_um`'s `target = feature −
offset`. Invariant, pinned by test: `offset_um(k) == centre_offset(k) − centre_offset(0)`.

---

## Already-built infrastructure this WIRES UP (nothing rebuilt)

| Component | Was |
|---|---|
| `SupportClasses/NeedleFocusTemplateStore.py` | Complete + tested, **zero production writers or readers**. Its module docstring describes the exact capture gesture step 4 performs. |
| `StageController.set_needle_camera_offset_um` / `needle_target_xy_for_feature_um` | Complete + tested, **zero production writers**. Returns the feature unchanged when unmeasured, so adopting it is safe. |
| `NeedleDetector.compute_focus_score(frame, roi_rect)` | Already ROI-capable — which is why one frame can score every bore at once. |

---

## Files modified

| File | Change |
|---|---|
| `SupportClasses/StageController.py` | **Refcount** `_print_floor_active` (was a plain bool with 3 independent callers; this work adds a 4th). Public API unchanged. |
| `SupportClasses/NeedleBoreCalibrationStore.py` | NEW pure `offset_from_frame_clicks` + `needle_camera_offset_from_click`, with the full derivation and the argument-order warning. |
| `SupportClasses/NeedleFocusTemplateStore.py` | `add_capture(microscope_focus_um=, needle_z_user_mm=)` (additive) + `focus_to_needle_z_mm` / `focus_to_needle_z_spread_mm`. |
| **NEW** `SupportClasses/BoreFocusROI.py` | Pure per-bore focus-ROI math with the non-overlap guarantee. |
| **NEW** `gui/widgets/needle_bore_wizard.py` | The wizard: step strip, own microscope feed, `BoreMeasurement` (Qt-free arithmetic), steps 1-4. |
| `gui/pages/calibration.py` | `_bore_cal_gate_key` / `_bore_cal_tick` (Bug A) + `_bore_cal_refresh` re-syncs the key; tick hook in `on_status_update`; Needle Location branch in `_on_workflow_tab_changed` (it had none); `_build_needle_bore_wizard` mount; wizard hooks in `set_hardware_config`. |

---

## Implementation steps

- [x] Refcount the plate-bottom floor + tests (mutation-verified against the old bool)
- [x] Bug A: change-key tick, tab hook, refresh re-sync (mutation-verified)
- [x] Pure sign helpers + the composed end-to-end sign test (mutation-verified)
- [x] Optical↔needle Z datum in `NeedleFocusTemplateStore` + tests
- [x] Pure ROI math + tests (caught a real Euclidean/Chebyshev bug — see below)
- [x] The wizard widget + tests
- [x] Mount in `calibration.py`, keeping the side-camera path as the cross-validator
- [x] Per-suite regression + `gui.app` import smoke
- [ ] **Real-hardware verification on ME3B V1** (checklist below)

---

## Safety

| Motion | Guard |
|---|---|
| Step 3 park | `_safe_navigate_to` → `safe_travel_to` (retract → wait → XY → wait → lower). Never a bare Z move then an XY move. Refused without Plate Bottom Z **and** Fast-Move Z. |
| Step 3 survey height | `max(typed, MIN_SURVEY_CLEARANCE_MM)` through `print_height_to_zref`; **never a raw literal** and never a raw `z >= safe_z` comparison (polarity-wrong on `ZDIR=-1`). |
| Step 3 Z jog | Manual-jog exemption; plate-bottom floor armed. |
| Step 4 approach | **Z only, at the current XY**, so there is no XY move to retract for. Target planned against `max_bore_z_offset_mm` — the chicken-and-egg resolver: `dz` is already measured, so the *longest* bore stops at the clearance instead of being buried. |
| Floor lifecycle | Armed on entering steps 3/4, disarmed on leaving and unconditionally in `hideEvent`. Moving **between** the two armed steps does **not** disarm-and-rearm — that would open a window with the clamp off (pinned by test). |
| Clicks | Zero stage motion. A stage drift > 5 µm since the park **refuses** the click: the one-frame method's validity rests on the stage being stationary. |
| Commit | Plausibility (`MAX_BORE_OFFSET_UM`, `MAX_BORE_Z_OFFSET_MM`) **refuses before writing** — a clamped offset is a wrong move that looks right. |

---

## Issues & decisions

* **🐞 A real bug the tests caught: Euclidean vs Chebyshev.** The focus ROIs were sized against
  the *Euclidean* distance to the nearest neighbouring bore, but the ROIs are **axis-aligned
  squares** — two squares are disjoint only when `max(|dx|, |dy|) >= side`. A triple at
  (900,500)/(1100,500)/(1000,660) has a 189 px Euclidean gap between bores 0 and 2, admitting a
  170 px box whose x and y separations are only 100 and 160 px. Overlapping boxes make both
  bores peak at whichever tip is sharper, so **every bore would report the same focus Z** —
  reproducing Bug B by another route while looking like a genuine measurement. Fixed to
  Chebyshev; pinned by `test_a_triple_never_overlaps` and `test_it_is_CHEBYSHEV_not_euclidean`.
* **🐞 The wizard wrote the store but never applied to the live needle**, relying on a host
  listener that may not exist. Now calls `apply_to_needle` itself (idempotent, so the host
  re-applying is harmless) — caught by `test_commit_reaches_the_live_needle`.
* **The probe that was designed and then deleted** is recorded above rather than silently
  dropped: it was unmeasurable in principle, and finding out why is what produced the correct
  frame-independent derivation.
* **The side-camera path is KEPT**, not replaced — it works when the objective's FOV cannot hold
  every bore, and it is the in-app **cross-validator**: measuring one bore both ways must agree
  including sign, and a disagreement that is an exact negation is a sign bug made visible.
* **Applying `needle_camera_offset_um` to the pick/place workflows is explicitly OUT OF SCOPE.**
  That store's own docstring is clear that click-to-CENTRE must **not** apply it while
  click-to-PICK must; auditing those call sites is its own change. Until then
  `needle_target_xy_for_feature_um` returns the feature unchanged when unmeasured, so nothing
  changes behaviour.
* **A pre-existing failure, confirmed NOT from this change:**
  `test_v79_cell_targeting_setup_page::test_the_real_saved_profile_reproduces_its_exact_volume`
  (0.003142 vs 0.012568 — exactly 4.0×). It is a golden-file test reading the operator's own
  `config/workflows/cell_targeting/__last__.json`, which they re-saved on 2026-08-05 changing
  `push_depth` 0.1 → 0.400052 and adding `push_volume_nL: 3.142`. **Proved** by restoring the
  HEAD copy of that one file: the test passes; with their current copy it fails. Their file was
  restored untouched. The golden expectation is stale relative to their re-save.

---

## Testing notes

**New: 136 tests, all green.**

| Suite | n | Covers |
|---|---|---|
| `test_v710_print_floor_refcount` | 12 | nested arm/disarm, clamp-at-zero, `__new__` partials, direct assignment |
| `test_v710_bore_gate_live_refresh` | 13 | Bug A end to end, throttling (200 ticks → 0 rebuilds), gate-key cheapness |
| `test_v710_microscope_bore_sign` | 16 | the composed sign test, all rotation×mirror×flip, per-bore Z on both polarities |
| `test_v710_optical_needle_datum` | 16 | round-trip, degradation without a focus axis, inverted-axis spread |
| `test_v710_bore_focus_roi` | 24 | sizing, clamping, the non-overlap guarantee, Chebyshev |
| `test_v710_needle_bore_wizard` | 55 | measurement state, gates, safety, clicking, commit |

**Three mutations confirmed CAUGHT:** the tick removed (Bug A returns) · the sign flipped
(bore misses by 640 µm = 2× the spacing) · the floor reverted to a plain bool (nested disarm
drops it).

**Regression, per-suite:** bore-offset (93) · bore-safety (65) · multibore (36) ·
other-workflows-bore-safety (14) · focus-template (31) · needle-location (25) ·
plate-z-autocal ×2 (33) · calibration-revision (20) · needle-offset-z-view (5) ·
z-retract (22) · per-bore-cell-targeting (50) · suite-hygiene (8) ·
quick-print-pick-place (46) · spheroid (16) · cell-labeling (23) · hard-abort (28) ·
gentle-descent (23) — **all green**, plus a `gui.app` import smoke.

---

## ⚠ Needs real-HW verification on ME3B V1 — IN THIS ORDER

A mis-signed offset is a *right-distance-wrong-way* error, so check **direction**, not just
magnitude.

1. Connect XY → the warning clears with **no** config re-save. *(Bug A)*
2. Step 2 → teach or auto-fill Plate Bottom Z + Fast-Move Z.
3. Step 3 → park; confirm the needle stops ~0.5 mm above the glass, never nearer.
4. Click all three bore tips; confirm the printed offsets match the physical spacing
   (100-500 µm) and are plausible per axis.
5. **Drive each bore in turn to the SAME target and confirm each tip lands on it.** The go/no-go
   for the sign. A flip lands 2× the spacing away on the wrong side.
6. **Cross-validate one bore against the side-camera method.** They must agree, sign included.
   An exact negation is the sign bug.
7. Per-bore Z: jog and confirm the focus score peaks at a **visibly different Z per bore**, and
   the recorded spread is plausible (~50 µm per D7). *(Bug B — the number that was always 0.0)*
8. Step 4: confirm the displayed longest-bore offset, approach, and verify the **longest** bore
   reaches the glass first with the stated residual clearance.
9. With the Ti connected, confirm `microscope_focus_um` is captured and
   `focus_to_needle_z_mm` predicts a needle height that matches a re-touch.
10. Restart → offsets, camera offset and plate-bottom Z all restore.
