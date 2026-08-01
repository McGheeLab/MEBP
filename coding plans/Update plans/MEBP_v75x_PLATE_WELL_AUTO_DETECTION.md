# MEBP v7.5.x — Model-driven well auto-detection on a plate mosaic

**Operator (2026-07-30):** *"I just ran a full mosaic on a clear plastic 24 well
plate — the goal for you is to detect 24 wells on the mosaic and do the best
effort to ensure that they are where they should be and they are the size they
should be. Figure out an algorithm for this and lets use that for the auto
detection of wells button. We may have different parameters for a black bottom
glass plate, or other well plate types, so as I add these into the library we
will figure out the best way to auto detect those wells, then we will make a
nice classifier after we have many plates in the system."*

---

## Objective

Detect all 24 wells on the freshly-scanned `nest-plastic-24` mosaic, at the
right positions **and** the right sizes; make that the "Auto-detect wells"
button; and put every appearance-dependent parameter on the plate type so new
plate products are a data problem, not a code problem.

---

## Why the old button never worked on this plate

`MosaicWellMappingDialog._auto_detect` called **`WellDetector.detect_filled_wells`
and nothing else**. That is a generic blob finder — threshold the bright pixels,
keep the large round blobs. On a clear plastic plate in brightfield the well is
**a hole, not a bright blob**: the interior and the plate body read the same
grey and only the moulded rim is bright. Measured on the operator's own mosaic:

```
detect_filled_wells(nest-plastic-24.png)  ->  0 detections
```

So the button reported *"Auto-detect couldn't find the grid"* every single time,
on exactly the plate being scanned. (The mosaic-scan path fared better only
because it falls through to a Hough pass when the blob finder returns < 3; the
dialog had no such fallback.)

Neither path used the fact that **we already know the answer's shape**: the
plate definition gives 4 × 6 wells, 19.3 mm apart, 15.6 mm across, and the
mosaic carries its own px/µm. The well radius and lattice spacing in pixels are
therefore known to ~1 % *before looking at a single pixel*.

---

## Algorithm (NEW `SupportClasses/PlateWellDetector.py`, GUI-free)

Four stages, each turning an open-ended search into a well-posed one:

**1 — Matched filter at the known radius.** `ring_response()` correlates the
mosaic with an annulus kernel: −1 just inside the expected radius, +1 just
outside. Each lobe is normalised to unit weight, so the response is a
*difference of local means* — invariant to the tile-to-tile brightness steps a
stitched mosaic always has. On the operator's plate the 24 true wells score
**22.8–45.0 and the next-best spurious peak scores 8.8** — a 2.6× gap, so
thresholding is trivial. Cost: 0.02 s at 1/4 resolution.

**2 — Fit the known lattice.** `fit_lattice()` fits rows × cols as
**rotation + per-axis scale + translation — deliberately no shear** (a moulded
plate cannot shear, and allowing it lets a wrong assignment hide in a skew).
Rotation seeds come from a histogram of adjacent-pair angles folded into a 90°
wedge, plus a coarse sweep; translation is found by *voting* — every
(candidate, node) pairing implies one offset and the true one is voted for by
every well at once. This both labels the detections and **manufactures the
missing ones**, so the result always carries exactly rows × cols wells.

**3 — Measure each well.** `refine_ring()` walks a radial intensity profile
outward along 180 rays, takes the strongest correctly-signed step on each
(sub-pixel, parabolic), drops sectors that disagree with the rest (label text,
bubbles, a neighbour's rim), and fits a circle to the survivors. This *measures*
the well rather than assuming the catalogue size.

**4 — Regularise against the plate.** A well whose refined radius is a
statistical outlier (MAD) or whose centre wandered off the lattice falls back to
the plate median / the lattice prediction. A moulded plate's wells are identical
to well under a percent, so the median of 24 measurements beats any lone fit
that disagreed with all the others. This is the *"best effort to ensure they are
where they should be and the size they should be"* half of the request.

### Two findings that shaped the code

* **Inlier COUNT cannot score the lattice fit.** It saturates at "all wells" for
  a whole family of wrong fits — a 5° tilt still lands every well inside a
  0.25-pitch gate. A first cut scored by count and settled on a **−4.99° tilt
  with a 6 % anisotropic pitch**, dragging the refinement with it. Rotation is
  now chosen by *minimising the inlier residual*; the count only ranks the
  discrete hypotheses.
* **A wide radius search band is actively harmful.** At ±30 % the radial walk on
  a low-contrast plate latched onto a shoulder well outside the well and
  reported **18.38 mm for a 15.6 mm well**; at ±10 % the same image measures
  **15.58 mm**. We know the catalogue diameter to ~1 %, so a tight band is both
  honest and better conditioned. `diameter_tol` 0.30 → **0.10**, and
  `ring_band_frac` 0.10 → **0.06**, both chosen by a sweep over the three real
  mosaics (best worst-case).

### Refusing beats guessing

Placing a full grid of wrong markers is worse than saying "I couldn't find it" —
the caller's fallback (teach 3 corners by hand) is a fine outcome. Two gates:

* **lattice-inlier fraction ≥ 0.60** — the "wrong plate type" discriminator.
  Fitting a 96-well grid to the 24-well mosaic still "measures" **79 %** of its
  nodes (the radial walk finds *some* edge inside those big wells) but only
  **11 %** are real rings. 0.60 rather than 0.50 because a square grid also
  admits a **45° lattice at √2× the pitch**, which lands on exactly half the
  wells and so scores 50 %.
* **measured fraction ≥ 0.25** — the "wrong appearance profile" discriminator:
  forcing the wrong edge polarity still snaps half the lattice but measures
  nothing.

A **180°-flipped lattice fits equally well** (the plate is symmetric; nothing in
the image says which corner is A1), and the two solutions put row/col labels on
opposite corners. The fit is canonicalised to the near-zero-rotation solution so
the answer is deterministic; which corner is *really* A1 stays the caller's
decision, made from the plate-orientation convention rather than from pixels.

---

## Per-plate-type parameters — the path to a classifier

Everything appearance-dependent is in `WellAppearance`, which a `PlateType`
carries in its JSON as `well_detection` (emitted only when set, so every plate
type written before this round-trips byte-identically). A black-bottom glass
plate whose wells read as dark discs needs **a different edge polarity, not
different code**.

`edge_polarity="auto"` — the default — runs the pipeline under both polarities
and keeps whichever locks the lattice better, so a brand-new plate product works
with no tuning at all, and the **resolved polarity is reported** so it can be
written back onto the plate type. Named presets: `clear_plastic_rim` (validated
here), `bright_disc`, `dark_disc`.

As the library grows, those stored profiles are exactly the labelled training
set a classifier would need: (plate product, mosaic, winning polarity, tuned
band) tuples produced as a by-product of ordinary use.

`config/hardware/plate_types/user/nest-plastic-24.json` is stamped with
`{"key": "clear_plastic_rim"}` now that the operator's own scan settled it —
deterministic, and 0.38 s instead of 0.51 s.

---

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PlateWellDetector.py` | **NEW** — the whole algorithm + `WellAppearance` profile + presets. GUI-free. |
| `SupportClasses/PlateTypeStore.py` | `PlateType.well_detection` (conditional-serialised, dict-coerced). |
| `gui/dialogs/mosaic_well_mapping_dialog.py` | `_auto_detect` rewritten onto the new detector; `_grid_pitch_mm`, `_well_appearance`; markers drawn at the MEASURED radius; refusal reason surfaced. |
| `config/hardware/plate_types/user/nest-plastic-24.json` | stamped with the validated profile. |
| `tests/test_v75x_plate_well_detection.py` | **NEW** — 36 tests. |
| `tests/test_v75x_plate_mosaic.py` | the auto-detect-on-open test's synthetic image made geometrically consistent with the plate (see Issues). |

---

## Implementation Steps

- [x] Matched-filter ring response at the known radius, polarity aware
- [x] Candidate peaks with non-maximum suppression at 0.6 × pitch
- [x] Lattice fit: rotation/scale/translation, residual-scored, canonicalised
- [x] Per-well radial-edge measurement with robust sector rejection
- [x] Plate-level regularisation of radii and centres
- [x] Acceptance gates + actionable refusal messages
- [x] `WellAppearance` profile + presets + `PlateType.well_detection`
- [x] Wire the "Auto-detect wells" button; draw measured sizes
- [x] Validate on all three real mosaics + synthetic robustness suite

---

## Results — the operator's plate

`nest-plastic-24.png` (3000 × 1944, 1410 tiles, 0.0251 px/µm), **0.4 s**:

| | |
|---|---|
| wells found | **24 / 24**, all measured (none filled in from the grid) |
| measured Ø | **15.63 mm** vs 15.60 nominal (+0.2 %) |
| well-to-well radius spread | **σ = 0.95 px = 38 µm** (0.5 %) |
| rotation | **−0.30°** |
| centre vs lattice | RMS **3.5 px = 140 µm**, max 7.4 px |
| measured pitch | 19.14 × 19.11 mm vs 19.30 nominal |

Rendered overlay: every ring sits on the inner edge of the bright moulded rim.

**A real cross-check falls out of this.** The plate is a manufactured ruler, so a
systematic pitch error is evidence about the **mosaic's µm/px**, not the plate:
the measured 19.14 mm against a nominal 19.30 mm says this mosaic's scale is
about **0.9 % small**, and the detector says so in a warning. (The older `24`
mosaic measures 19.32 × 19.40 mm — that one's scale is right.)

### The other two mosaics on disk

| mosaic | result |
|---|---|
| `24` (per-well composite, blue) | 23/24 measured + 1 from the grid, Ø 15.22 mm, RMS 3.3 px — including wells clipped at the mosaic edge |
| `plate-24_Rossette A1` (low-contrast fluorescence) | 23/24 measured, Ø **15.59 mm**, RMS 9.6 px. The one fallback is the well holding the rosette insert. **This is the case the ±30 % band got wrong (18.38 mm) — it is what motivated the tighter default.** |

### Robustness (synthetic + real)

| perturbation | outcome |
|---|---|
| 4 wells blacked out | 20 measured + 4 placed from the grid; the rest move ≤ 8.8 px |
| illumination gradient 0.55×→1.45× across the plate | centres move ≤ 0.2 px |
| Gaussian noise σ = 25 | centres move ≤ 0.9 px |
| plate rotated 3° / 12° | 24/24 |
| 96-well plate type selected | **refused**, naming the plate type |
| wrong edge polarity forced | **refused**, naming the appearance profile |
| mosaic µm/px off by ±5 % | degrades gracefully + the pitch warning fires |

---

## Testing Notes

`tests/test_v75x_plate_well_detection.py` — **36 tests, all green**. Most run on
a synthetic plate (deterministic, CI-safe) whose rim runs 1.00 R → 1.19 R,
matching the radial profile measured on the real mosaic; the last class runs
against the operator's real mosaics when present and **skips** when they are
not, so the suite stays portable while still pinning the accuracy numbers
quoted above.

Regression, all green: plate-well-detection, plate-types, well-type-presets,
mosaic orientation-remap / orientation-adjust / memory-perf, unified-mosaic-
calibration, rosette-tab-auto-reanchor, single-well-reregister, startup-well-map,
fluor-mosaic-shift, plate-location click-rim / workflow-toggle, fluorescence-
mosaic, v731-integration, last-known-calibration, plate-centering, freeform-warp
— **440**.

One pre-existing failure, **confirmed not from this change**:
`test_v75x_plate_mosaic.TestFilledWellDetector.test_real_24_well_mosaic`
(documented in CLAUDE.md). Verified by running the **committed HEAD** copy of
`VisionDetector.detect_filled_wells` against `24.png` — it returns 23 where the
test wants ≥ 24. That is the legacy blob detector, which this work does not
touch. (Worth noting: the new detector resolves all 24 wells on that same
image.)

**Needs GUI verification on the rig:** open Plate Location → Map wells on the
fresh mosaic; auto-detect should place 24 rings sitting on the wells with the
measured Ø in the status line; confirm all and check that a couple of named
wells drive to the right place.

---

## Issues & Decisions

- **Legacy fallback removed from this button, deliberately.** Keeping
  `detect_filled_wells` as a secondary attempt would let a wrong grid be placed
  silently, defeating the acceptance gates — and on the plate that prompted this
  work it returns nothing anyway. The operator's guaranteed manual path (teach
  3 corners) is the fallback, and it is left armed on every refusal.
- **The mosaic-scan path still uses the old detection** (`detect_filled_wells` →
  Hough → warp fit) for its own auto-fit. It is not broken, and since a finished
  scan now auto-opens the Map-wells dialog the new detector is the surface the
  operator actually meets. Adopting it there too is a clean follow-up.
- **A test had to be corrected, not the code.**
  `test_auto_detect_on_open_places_all_wells` built a synthetic image with 40 px
  wells 120 px apart while telling the dialog it was a 24-well plate at a scale
  that made those wells 390 px across and 965 px apart. The old detector never
  looked at the plate definition, so the contradiction did not matter; a
  model-driven detector correctly refuses it. The image is now geometrically
  consistent with the plate.
- **Brightness = max over channels**, not `BGR2GRAY`: a fluorescence mosaic can
  be almost pure blue, which the luma weights (B = 0.114) all but discard — on
  the real blue mosaic that dropped working contrast from std 47.8 to 5.6.
- **Not addressed:** whether the 0.9 % pitch discrepancy on the new mosaic is
  the objective µm/px or the plate. The detector now measures and reports it
  every run, so that call can be made from evidence — the same pattern used for
  `PREDICTION_HEADROOM` in v7.7.
