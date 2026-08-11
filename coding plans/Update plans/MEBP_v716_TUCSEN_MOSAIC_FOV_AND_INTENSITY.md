# MEBP v7.16 — the Tucsen mosaic: camera keying, rectangular FOV, intensity

**Branch:** `Version-7.9.1`

Operator, after calibrating a Tucsen with the 4x objective:

> *"I just calibrated the tucsen camera with the 4x onjective. and i tried to do
> a full plate mosaic, and it said it would take 400 minutes for the scan with
> way too many tiles. I think its because the microscope camera setup section is
> not working for the tucsen camera. Also the mosaic tiles are using the
> microscope view as a square. it is not a square it is rectangular and we should
> take advantage of those pixels. After we have done the full mosaic, lets
> reguralize the intensity across the mosaic"*

All three are real. Their hunch about the camera-setup section was right, and
the mechanism turned out to be worse than "not working".

Plus, mid-session:

> *"when i save a setup in the identity tab, it needs to default to the folder
> that it is reading from"*

---

## The 400-minute scan, root-caused end to end

Every number below is from the operator's own `logs/app.log` and
`config/hardware/objectives.json`.

```
19:09:14  resolved: 1.2710 um/px at 2600x2048 (measured 3.2271 @ 1024x1024)
          | FOV 3305x2603 um        <- the ANDOR's 4x, stretched onto the Tucsen
19:09:18  resolved: 0.3891 um/px at 2600x2048 | FOV 1012x797 um
          Raster grid: 153x124 = 18972 positions          -> ~474 min
```

**Link 1 — the objective store was keyed by camera MODEL NAME.**
`ObjectiveCalibrationCard._camera_key()` returned
`camera_config.camera_spec.name`, a *configured catalogue label*. The Tucsen was
fitted while the spec still read `"Bestscope BUC3D-1000C (ToupTek
C3CMOS10000KPA)"` — there is **no Tucsen entry in `config/hardware/cameras.json`
at all**, so the operator could not select one. Both cameras therefore shared one
calibration block. The log shows it plainly:

```
19:06:00  Objective calibration saved: Bestscope BUC3D-1000C (ToupTek …)/4x
          = 0.3891 um/px @ (2600, 2048)
19:06:00  Camera orientation saved: Tucsen Camera = -89.97 deg [tucam:0…]
```

…the Tucsen's calibration filed under the ToupTek's name, **destroying the
ToupTek's genuine 4x value**.

**Link 2 — the identity store never got a resolution stamp.** The card reaches
the store through `um_per_px_committed = Signal(int, float)`, which carries only
`(cam_idx, value)`. Every camera start logged
`µm/px restored WITHOUT a measurement resolution`. Unstamped means
`effective_um_per_px` degrades to a **passthrough**.

**Link 3 — the Scale/FOV dialog's Verify step overwrote the measurement.**
`_verify_mosaic` re-synced `result_um_per_px` from the live manager
*unconditionally*, "so Accept reflects the correction". But the manager holds
whatever was last pushed into the slot — here the ToupTek's 0.389135, pushed by
link 1 and unrescalable by link 2. Opening Verify and changing **nothing**
replaced the fresh reading with a stale one, silently.

That is why the saved value is **bit-identical to the ToupTek's stored 4x**, a
coincidence a fresh stage-motion measurement cannot produce. The dialog had in
fact measured `1.8683 µm/px` 90 seconds earlier (19:04:03).

Result: FOV **1012 x 797 µm** instead of ~3305 µm → **18,972 tiles instead of
~1,800**. Tile count scales with the *square* of the scale error.

---

## The "square" sensor

`_orient_tile` rotated each tile about its centre and rendered it **back into the
input W x H**. This camera sits at **-89.97°**. On a 2600x2048 frame:

* the 2600-px axis becomes vertical and is **clipped to 2048** — **23.1 % of
  every frame discarded**;
* the other axis gets black bars blended in;
* `generate_raster_positions` stepped by the *unrotated* FOV, so the real
  coverage per tile collapsed to `min(w, h)²` — **literally a square** — leaving
  near-zero overlap on one axis while over-scanning the other.

CLAUDE.md already carried this as a known hazard ("a non-square sensor landing on
±90° leaves silent mosaic coverage gaps"). It is now fixed rather than recorded.

---

## What changed

| File | Why |
|---|---|
| `SupportClasses/MosaicCalibration.py` | NEW `objective_camera_key()` — the ONE key every caller uses (device identity; spec name only when no identity exists). `_resolve_scale` uses it. |
| `SupportClasses/MosaicBuilder.py` | NEW `_fov_um` / `_oriented_fov_um` / `_oriented_size_px`; `_orient_tile` outputs the rotated bounding box; placement, raster, canvas, tile rects and `_max_shift_px` all use the oriented footprint; NEW `_place_from_box`; NEW `estimate_flat_field` / `regularize_intensity` / `_intensity_correct`. |
| `SupportClasses/ObjectiveCalibration.py` | NEW `implausible_reason()` — the spec-free plausibility invariant. |
| `gui/pages/hardware/objective_calibration_card.py` | `_camera_key` → identity; NEW `_legacy_camera_key` / `adoptable_legacy_calibrations` / `_persist_um_per_px_stamp`; plausibility gate before anything is written. |
| `gui/dialogs/scale_fov_calibration_dialog.py` | Verify adopts the manager's value only when it actually CHANGED. |
| `gui/dialogs/mosaic_settings_dialog.py` | NEW `regularize_intensity` setting (default on). |
| `gui/pages/calibration.py` | Identity key at 4 sites; true captured resolution on write; scan runs `regularize_intensity`. |
| `gui/widgets/live_target_picker.py` | Identity key (was `spec.model` — a field `CameraSpec` does not define, so the lookup always missed). |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | Identity key. |
| `gui/pages/hardware_setup.py` | NEW `_setup_dir()`; Save/Load default to `config/hardware`. |

### The key rule, and what is deliberately NOT done

The device identity is used whenever one can be read, and there is **no fallback
to the spec name once an identity exists**. Falling back is exactly how one
camera reads another's calibration. An absent calibration is recoverable — a
30-second re-measure — where a wrong one silently scans at another sensor's
scale. `adoptable_legacy_calibrations()` exists so a legacy block can be adopted
as a *deliberate* act; it is never consulted automatically.

⚠ Two v7.11 wizards (`plate_level_wizard`, `needle_bore_wizard`) were **already**
identity-keyed, so they had been reading values the card never wrote. That is
independent corroboration that identity is the right key and the card was the
outlier.

### The plausibility guard

For ONE camera, `µm/px × magnification × capture_width` is a property of the
sensor and must agree across objectives — no datasheet needed. On the operator's
ToupTek: 2x → 5707, 4x → 5669, 10x → 5724 (**1.0 % spread**). It compares only
against the *same camera's* other objectives, so it cannot be fooled by another
camera's numbers, and returns None when there is nothing to compare against —
unverifiable is reported as unverified, never as passed.

### Intensity regularization

Measured from the scan itself, no reference slide and no extra frames. Specimen
detail moves between tiles so a per-pixel **median** of self-normalised tiles
averages it out, while the illumination pattern is fixed in the frame and
survives. The estimate is heavily blurred (real vignetting is smooth; anything
sharp is residual specimen structure that would otherwise be *burned* into every
tile) and refuses below `min_tiles=6`. Per-tile level matching is bounded to
0.5–2.0× so a tile legitimately dominated by a bright object is not dragged to
the plate average.

Measured on a synthetic 49-tile scan with a cos⁴-like falloff and ±18 counts of
level drift: **low-frequency (seam/vignette) structure reduced 78.8 %**, and the
recovered bright:dark ratio **4.02 vs the true 4.00**.

---

## Round 2 — three follow-ups

> *"on the microscope camera setup i want the tucsen camera to show up on the
> list."*
> *"when calibrating the rotation of the microscope camera, it says that it
> needs to move less than 199 microns. this is fully not true. for it to know
> how far it can move it needs to know the objective its on and the measured
> magnification of the objective only this way can we get then native microns
> per pixel for the camera at a 1x frame."*
> *"on the identity tab i want the save setup to open the dialogue box to the
> same folder that the identy selection reads from"*

### The Tucsen in the list

Added `TUCSEN-LIBRA-25` (hardware-verified resolutions from the v7.9 bring-up:
5200x4096 / 2600x2048, the latter 2x2 binned so it does **not** change the FOV)
and a `TUCSEN-GENERIC` fallback for other TUCam bodies.

`sensor_pixel_size_um` is **`null` on purpose**, and `CameraSpec` now types it
`Optional[float]`. I could not find a verifiable pitch for this model, and a
guessed one shows up as a plausible "theoretical µm/px" that a real calibration
is then judged against — the exact class of plausible-but-wrong number this
whole update is about. All four consumers now report *unknown* instead
(`"µm/px unknown for this model — run Mosaic & Camera Calibration to measure
it"`). A known pitch (the Zyla's 6.5 µm) computes byte-identically to before.

### The bogus 199 µm bound — the operator was exactly right

The bound is `0.25 × min(w, h) × µm/px`. It had been sized from
`CameraManager.effective_um_per_px`, which held **another camera's** 0.389135
µm/px — unstamped, so it passed straight through un-rescaled:

```
0.25 x 2048 px x 0.389135 = 199.2 um     <- reproduced exactly in test
```

At this camera's real scale the same bound is ~651 µm.

The operator's proposed derivation is the correct one and is now implemented.
NEW on `ObjectiveCalibrationStore`:

* `sensor_width_um(camera)` — `µm/px × magnification × capture_width` is the
  imaged sensor width, a property of the camera, identical for every objective;
* `native_um_per_px(camera, frame_width)` — literally "the native µm/px for the
  camera at a 1x frame";
* `predicted_um_per_px(camera, objective, frame_width)` — native ÷ this
  objective's magnification.

`PixelCalibrationDialog` gains `cam_key` / `objective` and resolves the bound
**measurement-first**: this objective's stored calibration → *predicted from the
camera's other objectives* → the manager **only when calibrated AND stamped** →
silent. The refusal now names its source, because a bound the operator can see
is wrong is one they can act on.

**Why the prediction leg matters:** the move happens *before* the measurement
exists, so a never-calibrated objective has no scale of its own. Derived from
its siblings, a 20x that has never been touched still gets a correct bound.

⚠ **`sensor_width_um` REFUSES when the objectives disagree** (>25 %). With two
entries a plain median just picks one, so a single bad stamp would silently
become the camera's native scale. This machine's Andor is exactly that case
(13218 vs 27036) — it now logs *"its objectives disagree … re-measure the odd
one out"* and returns None rather than a confident wrong answer.

### The identity-tab save folder

Already fixed earlier in this same update (`_setup_dir()`), verified again here:
the browser and the Setup-Name list both scan `CONFIG_HARDWARE_DIR`, and both
the Save and Load dialogs now open there. **Needs an app restart to take
effect** — the running instance still has the old code.

---

## Testing

`tests/test_v716_tucsen_mosaic_fov_and_intensity.py` — **49 green** (34 from round 1, 15 from round 2).

### Mutation verification — **14/14 CAUGHT**

| # | Mutation | Result |
|---|---|---|
| M1b | Key falls back to the spec name when the identity block misses | CAUGHT |
| M2 | Ignore the identity entirely (pre-v7.16 keying) | CAUGHT |
| M3 | `_orient_tile` renders back into W x H (the square bug) | CAUGHT |
| M4 | Raster steps by the unrotated camera-axis FOV | CAUGHT |
| M5 | Skip the flat-field division | CAUGHT |
| M6b | Defeat BOTH min-tiles guards | CAUGHT |
| M7 | Plausibility never warns | CAUGHT |
| M8 | Verify adopts the manager's value unconditionally (the 400-min bug) | CAUGHT |
| M9 | Setup save reverts to the CWD | CAUGHT |
| N1 | Dialog back to the raw manager passthrough (the 199 um bound) | CAUGHT |
| N2 | Drop the objective-prediction leg | CAUGHT |
| N3 | Average inconsistent objectives instead of refusing | CAUGHT |
| N4 | Fabricate a sensor pitch when it is unknown | CAUGHT |
| N5 | Remove the Tucsen catalogue entries | CAUGHT |

⚠ My first attempts at M1 and M6 were **partial mutations** (a no-op rewrite, and
a second guard left intact) and appeared to survive. Re-run properly, both are
caught. A mutation that does not actually change behaviour proves nothing.

### Regression

Green: unified-mosaic-calibration (37) · mosaic-plate-frame · objective-calibration ·
camera-orientation-audit (167 combined) · mosaic-orientation-adjust ·
memory-perf · orientation-remap · fluor-shift · camera-rotation (60) ·
camera-store · hw-controls · cal-liveview · image-correction · tucsen-libra ·
objective-ladder (177) · nd3-export · picker-scaling · spheroid-survey ·
suite-hygiene · plate-well-detection (144) · plate-mosaic (110, class-by-class).

Two legitimate test updates: the unified-calibration fixture now keys the
objective store by identity (the contract that changed), and the mosaic-settings
round-trip covers the new `regularize_intensity` key.

**One pre-existing failure, confirmed NOT from this change:**
`test_v75x_plate_mosaic::test_real_24_well_mosaic` (23 vs 24) — the legacy blob
detector in `VisionDetector.py`, a file this diff does not touch (verified with
`git diff --name-only`). Documented in CLAUDE.md.
`TestManualAlignPage` still hangs; excluded per existing precedent.

---

## Operator's data — migrated and repaired

`config/hardware/objectives.json`, backed up as `.bak-v716identity`:

* **Andor** block → `andor:VSC-07863`. Values untouched. Still resolves
  3.2271 µm/px, FOV 3305x3305 — unchanged behaviour.
* **ToupTek** → its `toupcam:…` identity, taking the internally-consistent
  legacy block plus the newer 2x (0.778843 @ 3664, which matches the identity
  store exactly and satisfies the invariant). Result: **1.0 % spread**, i.e.
  self-consistent.
* **Tucsen (`tucam:0`) → deliberately EMPTY.** Its only stored value was the
  ToupTek's, so the mosaic now **refuses** with
  *"The microscope camera has no µm/px calibration…"* instead of scanning 19,000
  tiles at a stolen scale.

⚠ **A separate pre-existing problem this exposed, NOT repaired:** the Andor's
**10x** entry has a 2.045× invariant spread against its own 4x (13218 vs 27036).
The 4x matches the Zyla datasheet (6.5 µm × 2048 = 13312 µm) to 0.7 %, so the
**10x resolution stamp is wrong** — almost certainly measured at 1024 and stamped
2048. Left alone and reported rather than inferred: it is a 30-second re-measure,
and silently rewriting a calibration stamp is the class of thing this whole
update is about.

---

## Needs real-HW verification on ME3B V1, IN ORDER

1. **Re-run Mosaic & Camera Calibration on the Tucsen at 4x.** It has no stored
   calibration by design. Watch the reported **FOV**: on the same port the Andor
   sees 3305 µm at 4x, so a Tucsen FOV near ~1000 µm means the measurement is
   still wrong — re-measure with a larger baseline on a well-textured region.
2. Confirm the log no longer says *"µm/px restored WITHOUT a measurement
   resolution"* on camera start.
3. Calibrate a **second** Tucsen objective (2x or 10x) and confirm the
   plausibility guard stays quiet — or fires, which would settle question 1.
4. Full-plate mosaic: the tile count should be in the low thousands, not ~19,000.
5. **Look at the stitched result**: tiles must butt up with no gaps and no dark
   seams, and the whole rectangular frame must be used (no black wedges).
6. Toggle **Even out illumination** off and re-scan a small region — the tile
   grid should visibly return, which proves the correction is doing the work.
7. Map wells on the new mosaic and drive to two of them — the needle must land
   on the centres (proves the oriented placement did not disturb
   back-projection).
8. Identity tab: **Save Setup** must open in `config/hardware` and the saved file
   must appear in the browser list immediately.
