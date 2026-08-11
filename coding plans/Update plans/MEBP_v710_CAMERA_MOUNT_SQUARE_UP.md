# MEBP v7.10 — Square up a camera mount (live rotation alignment aid)

## Objective

> *"In the camera calibration, we have a great way to detect camera rotation, but what I want to
> do is fix the camera rotation to as close to a 0, 90, 180, 270 as possible by physically
> rotating the actual camera at the mount. The software will first capture the rotation via
> calibration, then we can monitor the features as I rotate the frame, it tells me to keep
> turning the camera until I rotate it into where it needs to be. One way to do this is to rotate
> the calibrated view to the exact rotation we want, then set its alpha such that when I
> physically rotate the hardware it will align with the frame. Then we can recalculate the
> alignment."* — operator, 2026-08-05

The app already **measures** camera rotation vs the stage and corrects for it in software
(`R(θ)` in `pixel_to_stage_offset`, `MosaicBuilder._orient_tile`, `CameraFeedView._orient_qimage`).
It could not help **remove** that rotation physically. The only feedback was a static line on the
slot card — `Rotation vs stage: 12.4° (Δ +12.4° from nominal 0°)` — pull-refreshed, never live.
Squaring a mount meant turning the camera blind, re-running a stage-motion calibration, reading
the new number, and repeating.

Squaring the mount matters beyond tidiness: at an exact multiple of 90° both
`_orient_qimage` and `_orient_tile` hit their **no-op fast paths**, so the live feed and every
mosaic tile stop being resampled.

**Delivered:** a per-slot **⊾ Square up mount…** tool that freezes a reference frame, shows it
pre-rotated to the target as a translucent ghost over the live feed, reports a **live
"degrees still to turn"** readout while the operator turns the camera — with **no stage motion at
all** — and then re-runs the existing rotation calibration to confirm and commit.

### Operator decisions (AskUserQuestion, 3)

| Question | Answer |
|---|---|
| Which slots get the tool | **All four** |
| Ghost style | **Both** — photo blend by default, edge-outline toggle |
| Final stored value | **Always the measured residual** (no snapping to the exact cardinal) |

---

## Files Modified

| File | Change |
|---|---|
| `gui/dialogs/pixel_calibration_dialog.py` | **Stage 0** — new `column_direction_to_camera_rotation_deg`; `_compute` converts the raw column direction into the θ that belongs beside the stored flips. `fold_parallel_deg` moved out and re-exported. |
| `SupportClasses/CameraRotationTracker.py` | **NEW** — pure, GUI-free. Sign, nominals, frame prep, ghost/edge builders, the live estimator, `RotationTracker`. Owns the canonical `wrap_deg` / `fold_parallel_deg` / `nearest_nominal`. |
| `gui/widgets/camera_feed_view.py` | `set_alignment_ghost(...)` + cached display-space composite + cache invalidation on orientation / edge-pick / size / image change. |
| `gui/dialogs/camera_rotation_align_dialog.py` | **NEW** — the dialog, `_ResidualDial`, the sampling worker and its bridge, plus `suggest_reverification`. |
| `gui/pages/hardware_setup.py` | Per-slot **⊾ Square up mount…** button, `square_up_refusal` gate, `_refresh_square_up_gate`, `_on_square_up_mount`, `_set_slot_preview_throttled`; `nominal_rotation_delta` delegates to the pure module; `_on_calibrate_slot_rotation` now returns whether it committed. |
| `gui/dialogs/mosaic_calibration_confirm_dialog.py` | `_axis_phrase` points at the tool when the measurement is >2° off square. |
| `tests/test_v710_camera_mount_square_up.py` | **NEW** — 64 tests. |

---

## Implementation Steps

- [x] **Stage 0** — fix the mirrored-camera rotation measurement (prerequisite).
- [x] **Stage 1** — `SupportClasses/CameraRotationTracker.py`, sign helpers first, tests before UI.
- [x] **Stage 2** — `CameraFeedView.set_alignment_ghost` with the cached display-space composite.
- [x] **Stage 3** — the dialog: live feed, ghost, residual dial, worker + bridge, re-measure step.
- [x] **Stage 4** — slot button, gating, confirm-dialog pointer, helper de-duplication.
- [x] Tests (64) + 8 mutation checks, all confirmed CAUGHT.
- [x] Per-suite regression — 985 green across 38 suites.
- [ ] **Real-hardware verification on ME3B V1** (order below).

---

## Issues & Decisions

### 🐞 Stage 0 — the rotation measurement is wrong for a mirrored camera, and that is this rig's microscope

`plus_column_direction_deg` ([pixel_calibration_dialog.py:74](../../gui/dialogs/pixel_calibration_dialog.py))
models the stage→image map as `scale·R(α)`, and its own docstring says *"assumes a non-mirrored
image"*. Nothing acted on that. The repo's actual convention is `s = R(θ)·F·u·p` with
`F = diag(mx, my)`, `mx = −1` iff mirrored, `my = −1` iff flip_y.

`pto`'s contract — a plate feature's stage **label** `xy + pto(P)` is invariant under stage motion
— gives the forward model `d = −(1/u)·F⁻¹·R(−θ)·m` for a commanded move `m` at angle `c`. Two
separate errors fall out, and **only finding both makes the fix correct**:

1. `diag(−1,−1) == R(180)`, so `R(θ)·F` always reduces to `R(θ+180)·diag(1,−my)` when `mx = −1`.
   The helper can only ever report that *effective* angle — it is **180° out for any mirrored
   camera**.
2. Conjugating a rotation by a reflection reverses it, so for `det F = −1` the helper returns
   `2c − θ_eff` — a reflection **about the commanded angle**, not a sign flip. It therefore
   changes with whichever direction preset the operator clicked, which is exactly why no single
   reading ever looked wrong.

⚠ **My first cut fixed only (2).** A numeric forward-simulation of all four flip combinations
scored **56/112** and showed the residual 180° on every `mx = −1` case. With both terms:
**112/112 recover θ exactly.** The lesson is the same one this repo keeps re-learning — the
derivation was plausible, and only simulating the consequence exposed the missing term.

**Live impact.** The assigned microscope is `andor:VSC-07863`, `rotation_deg: 180.0,
mirrored: true` ([camera_calibrations.json](../../config/hardware/camera_calibrations.json)) ⇒
`det F = −1`. Pressing the existing **⟳ Rotation…** on that slot writes `90°` at the default 45°
preset instead of `180°` — into `CameraManager`, `CameraCalibrationStore` **and every
per-objective entry** via `adopt_camera_rotation`. Pre-existing and independent of this feature,
but step 4 of the new tool routes the operator straight into it, so it is fixed first.

*Benign today:* `nearest_square(180) = (180, 0)`, so that camera is already square and the tool
says so rather than commanding a turn.

### The sign, derived once and pinned by a test that never names it

A scene point at raw centred pixel `p` now appears at `R(φ)p`; its stage offset is a property of
the plate, so `R(θ)·F = R(θ')·F·R(φ)` ⇒ `R(θ') = R(θ)·[F·R(−φ)·F⁻¹]`. Conjugation by `F` reverses
a rotation exactly when `det F = −1`:

```
θ' = θ − φ   (det F = +1)          φ_target = +Δ   unmirrored
θ' = θ + φ   (det F = −1)          φ_target = −Δ   net-mirrored
```

Only `det F` is read, so it does not matter that `derive_camera_stage_orientation` always parks
handedness on `flip_y` — `(T,F)` and `(F,T)` give identical answers (pinned).

**⭐ Why a round-trip test is not enough.** The ghost and the numeric readout both come from
`target_image_rotation_deg`, so they are consistent **by construction** and would lie together if
the mirror condition were backwards — `estimate(ref, make_ghost(ref, φ)) ≈ φ` passes either way.
`TestClosedLoopSign` closes the loop through the REAL `pixel_to_stage_offset` instead: build
`θ' = θ ∓ φ`, then assert 20 random pixels report the same stage offset before and after. It
never mentions φ's sign. Mutation-confirmed: swapping the branches, `XOR`→`OR`, or dropping the
mirror branch all fail it.

The `make_ghost` warp sign has its own second anchor: the round-trip pins it against the
estimator, and `test_absolute_anchor_independent_of_make_ghost` rotates with a hand-written
`getRotationMatrix2D` call. **Mutation-confirmed that this matters**: flipping the sign in
`make_ghost` *and* the estimator together leaves the round-trip perfectly green and fails only the
absolute anchor.

### What is deliberately NOT derived: which way to turn

Physical-turn direction maps to image-rotation direction through the optical path — an odd number
of mirrors reverses it — which software cannot know. So the tool says *"start turning either
way"* and **latches the direction empirically** after 2° of measured motion, saying **"wrong way —
turn back"** when the residual grows. This is strictly better than a guess: it is right on any
rig, and a sign error costs two seconds rather than a wrong result. A **Flip ghost direction**
button makes the ghost recoverable in one click if it ever points the wrong way.

### The 180° unwrap was designed, then deleted — it cannot fire

The estimator works on the FFT magnitude, which is centro-symmetric, so its output is only defined
mod 180. The plan originally carried an unwrapper. Measured on synthetic scenes:

| applied | measured | conf |
|---|---|---|
| 80° | −79.97° | **0.83** |
| 100° | +80.01° | **0.010** |
| 135° | +45.00° | 0.028 |
| 180° | +0.00° | 0.015 |

FM's `conf` is the **translation** response measured *after* de-rotating by the estimated angle
([MosaicBuilder.py:306-311](../../SupportClasses/MosaicBuilder.py)), so a wrong branch de-rotates
wrongly and the response collapses. A wrapped sample can never pass the gate — the unwrapper would
have been dead code with a way to be wrong. `|Δ| ≤ 45°` by construction anyway (nearest of four
nominals 90° apart). Replaced by: fold to (−90, 90], gate on `MIN_CONF = 0.15`, report *"lost
lock"* instead of a number. Pinned by `test_a_wrapped_branch_is_rejected_not_unwrapped`.

### Measured numbers that set the constants

**`TRACK_SIZE = 256`** — cost/accuracy over ±45° on a synthetic scene:

| size | worst error | mean | cost |
|---|---|---|---|
| 192² | 0.417° | 0.167° | 5.0 ms |
| **256²** | **0.255°** | **0.126°** | **9.6 ms** |
| 384² | 0.247° | 0.072° | 22.9 ms |
| 512² | 0.221° | 0.057° | 43.7 ms |

512 costs 4.5× for no useful gain against a 1° tolerance.

**Accuracy, pinned by test:** the estimate is **exactly 0.000° when the frames coincide** — the
reading the operator actually stops on — and reads slightly *short* (toward zero) at small
non-zero angles, worst ≈ 0.58° near 2°, returning to ~0 by 5°. That is the benign direction: the
residual reads a touch large, so the operator turns slightly further rather than stopping early.
Step 4's stage-motion measurement is the authority regardless.

**Threading.** Sampling runs on a daemon worker. FM genuinely releases the GIL (4 calls: 164 ms
serial vs 70 ms threaded, **2.36×**), and the camera views already spend 6–33 ms per frame in
`QImage.transformed` — adding 10 ms of estimator to the GUI thread is the freeze class this repo
has fixed four times (`PUMP_JOG_OFF_GUI_THREAD`, `STAGE_JOG_OFF_GUI_THREAD`,
`JOG_TRAVEL_OFF_GUI_THREAD`, the PowerShell-in-camera-detection freeze).

### The ghost: display space, cached, and a corner-square bug avoided

Composited **after** the `KeepAspectRatio` downscale onto a ~600×450 pixmap (sub-millisecond),
not onto the full-res frame. Two reasons:

* **The cache is load-bearing, not an optimisation.** `QImage.transformed(SmoothTransformation)`
  costs **33 ms at 3664×2748** — measured. Re-orienting the ghost per frame would halve the GUI
  thread's budget. Cache key covers the resolved transform, the ghost identity and the display
  size; invalidated by `set_view_orientation`, `set_edge_pick_mode` (which also changes
  `_orient_qimage`'s output and is easy to miss), resize and a new ghost image. Mutation-confirmed.
* **A naive `drawImage(0, 0)` of the 256² tracking crop would land a small square in the corner.**
  The ghost is built from the *full* reference frame (downscaled to ≤720 px), so it covers the
  whole view — pinned by sampling the rendered centre AND both corners.

The dialog also throttles the slot card's own preview while it is open: two live views on a 10 Mpx
camera would otherwise stack ~82 ms/frame of orientation work.

**The feed shows flips but NOT rotation** — `auto_orient=False`,
`set_view_orientation(mirrored=flip_x, rotation_deg=0, flip_y=flip_y)`. The rule:
**correct what is not changing** (the flips are a fixed property of the optical path)
**and show what is** (the rotation, the thing being adjusted).

### 🔴 The needle-camera gate — three legacy store entries make this real

`role_nominal_rotations` returns the **±45° diagonals** for a needle role, but a needle cam's
`rotation_deg` is the sensor **roll** (nominal 0°) with the mount in `column_dir_deg`. So
`nominal_rotation_delta` is the wrong target function here; the tool always squares against
`(0, 90, 180, −90)` for every role.

Worse, three entries in this machine's store still hold the **pre-v7.5.x conflated value** —
`rotation_deg: 45.0` and two at `−45.0` on Teslong identities. The v1.0→1.1 migration only moves
`rotation_deg`→`column_dir_deg` for identities that were *assigned at the time*, and these were
not. Reading a 45° mount value as a roll would command a 45° physical turn, rolling the camera out
of level and scaling its column µm/px by `cos 45° = 0.707` — a 41 % error straight into
`TwoCameraNeedleAligner`, i.e. the needle driven to the wrong XY.

**Gate:** a needle-role slot must have `column_dir_deg` present, else refuse and point at
⟳ Rotation… (which stores mount and roll separately). Mutation-confirmed.

Rolling a *properly calibrated* needle cam is safe and useful — the column's XY-plane projection
direction is unchanged by roll (only its magnitude scales by `cos ψ`, negligible for a few
degrees), so levelling the view does not disturb `column_dir_deg`.

### Staleness — the v7.10 bore-gate lesson applied

That bug was a gate evaluated at page build that never re-ran. Every live input here is re-read:

| Input | Why it matters | Handling |
|---|---|---|
| `mirrored` / `flip_y` | On the card **behind** the dialog; flipping either inverts `det F` and therefore φ_target, with no visual cue | re-read `full_orientation` every 300 ms tick, recompute, re-render the ghost (mutation-confirmed) |
| `rotation_deg` | Writable from the spin box, a store restore, or an objective push | same tick |
| Feed death | `_grab_frame` calls `stop()` on read failure; `get_current_frame()` then returns the same array forever | watchdog on `frame_count_value()`; >1.5 s ⇒ *"feed stopped"* |
| Resolution change | `prepare_frame` normalises every resolution to the same square, hiding it | `RotationTracker.update` refuses on a shape change |
| Button gate | θ, `column_dir_deg`, running state | refreshed from the **same two paths** as `rot_btn` |
| Re-measure cancelled | `_on_calibrate_slot_rotation` returns early on **five** paths | it now returns a bool; the dialog reports *"Not re-measured"* rather than success |

### Helper de-duplication

`wrap_deg`, `fold_parallel_deg` and the nominal search now have exactly one implementation each,
in the Qt-free module; `hardware_setup` and `pixel_calibration_dialog` re-export them (asserted by
`assertIs`, so a future copy cannot creep back). `nominal_rotation_delta`'s behaviour is
byte-identical, including its ±45° knife-edge and its odd seed-at-nominal-0.

`MosaicBuilder` is **lazy-imported** inside the estimator — importing it eagerly costs ~483 ms and
drags skimage/scipy into the camera path, and this repo already has a `STARTUP_CAMERA_FREEZE`
plan. Pinned: importing the tracker pulls in no skimage, scipy or PySide6. Refactoring the
log-polar code down into the pure module was considered and rejected — lazy import already gives
the one-implementation property without touching a hardware-verified function.

### The tool writes nothing

Tracking commands no motion and persists nothing. The committed value always comes from step 4's
stage-motion measurement, through the host's existing commit path (manager + store +
`adopt_camera_rotation`), so there is one commit path rather than two. The worker is stopped
**before** that dialog launches — it moves the stage, and the tool's whole safety story is that
nothing moves while it runs.

### Advisories after a physical rotation (documented, not automated)

All fail **safely** — they refuse or look stale; none drives the hardware anywhere. Auto-clearing
them would risk destroying operator work for no safety gain.

| Artefact | Effect |
|---|---|
| `ReanchorFeatureStore`, `NeedleFocusTemplateStore` | Raw patches; `matchTemplate` is not rotation-invariant, so they stop matching (their confidence gates refuse). **Re-save them.** |
| Saved mosaic composites | Safe — `_orient_tile` bakes tiles into **stage** axes, not sensor axes. |
| `um_per_px` | Unchanged by a pure rotation. |
| A **C-mount thread** | 45° of rotation also translates the sensor ~0.10 mm axially — a large defocus at 20× and a µm/px change. Squaring a threaded mount is not free; check before loosening. |
| Non-square sensor landing on ±90° | ⚠ `_orient_tile` keeps the tile W×H while the scan steps by the unrotated FOV ⇒ silent mosaic coverage gaps. |

---

## Testing Notes

`tests/test_v710_camera_mount_square_up.py` — **64 tests, green.**

Pure: wrap/fold/nominals incl. the ±45° knife-edge · `target_image_rotation_deg` over 4 flip
combos × 8 θ × all nominals · handedness-parking invariance · **`TestClosedLoopSign`** (the one
that matters) · Stage 0 over 4 combos × 8 θ × 4 presets, its no-op property, its preset-dependence
tell, and the real Andor entry.

Estimator: round-trip ±45° · **absolute anchor** independent of `make_ghost` · linearity
(`|slope − 1| < 0.01`) · accuracy pinned (exact at 0, < 0.7° elsewhere) · flat field rejected ·
wrapped branch rejected · 41 px motion blur · shape mismatch · confidence gate at the boundary ·
`prepare_frame` across 3664×2748 / 640×480 / 2600×2048, centred crop, `INTER_AREA`, 16-bit input.

Qt: ghost covers the whole frame (not a corner square) · opacity · offset · cache holds across 10
frames · every cache input invalidates · RGBA ghost blends only where opaque · dialog target
selection, mirrored inversion + warning, **mid-session mirror and θ changes**, flip button,
uncalibrated camera, flips-shown-rotation-not, feed watchdog, re-measure cancelled vs committed,
worker stopped before the stage moves, worker lifecycle, and a **live turn driving the residual
from >6° to <1.5°** through the real worker and bridge.

Gate: all five refusals as pure predicates.

**8 mutations confirmed CAUGHT:** swap ±Δ · `XOR`→`OR` · drop the mirror branch · `make_ghost` and
the estimator both sign-flipped (round-trip blind, anchor catches it) · ghost cache disabled ·
Stage 0 correction removed · needle-cam legacy gate removed · φ_target snapshotted at construction.

**Regression: 985 green across 38 suites, run per-suite** (this repo's camera suites hang when
combined) — needle-cam-mount-and-roll, camera-rotation-cal-and-monitor, camera-rotation,
needle-center-direction, scale-fov-and-registration, unified-mosaic-calibration,
calibration-revision, cal-liveview, camera-calibration-store, image-correction, hardware-controls,
async-open, picker-scaling, reanchor, mosaic-orientation-adjust/remap, memory-perf,
mapping-camera-orient, fluor-mosaic-shift, fluorescence-mosaic, mosaic-unreachable-travel,
simulated-camera, tucsen-libra, andor-zyla, suite-hygiene, objective-calibration,
needle-location-quick-move, the five v7.10 bore suites, bore-offset-calibration,
plate-z-autocal ×2, hardware-config-bores. Plus a `gui.app` import smoke and a real
`HardwareSetupPage()` build confirming four correctly-gated buttons that survive a rotation
refresh.

---

## Needs real-hardware verification on ME3B V1, IN ORDER

1. **Stage 0 first, and nothing else is trusted until it passes.** On the Andor (mirrored),
   ⟳ Rotation… must now return ≈180° — **from two different direction presets**, since
   preset-dependence is the signature of the bug.
2. Open the tool on the microscope. Expect *"already square"* (Δ = 0) and no instruction to turn.
3. Deliberately loosen the mount and turn a few degrees: the readout must track, and the direction
   word must lock on after ~2° and say "wrong way" if you reverse.
4. Turn back to ≤1°, lock the mount, re-measure. **Confirm the committed θ matches the readout's
   prediction and not its negation** — if it lands about twice the original tilt away on the other
   side, the ghost was inverted for this optical path: press **Flip ghost direction** and repeat.
5. Cancel the re-measure once and confirm the dialog reports *"Not re-measured"*, not success.
6. Check the live feed is visibly sharper afterwards (an exact cardinal skips resampling).
7. Re-save the mosaic re-anchor feature, then confirm auto re-anchor still finds it.
