# Well-detection R&D

Scratch area for developing/tuning automatic well detection against real, labeled
plate mosaics (the `config/hardware/well_training/` corpus + `config/hardware/mosaics/`).

## Current algorithm (shipping)

`SupportClasses/VisionDetector.py`:

- **`WellDetector.detect_filled_wells(frame)`** — robust detection of FILLED-disc
  wells on a stitched mosaic. Colour/illumination-agnostic (brightness = max over
  BGR → Otsu → morphology → connected components → area/roundness gate), and it
  fits a circle to each blob with image-**border points dropped** so wells clipped
  at the mosaic edge still recover their true centre. Replaces HoughCircles (which
  is finicky/misses on high-contrast filled circles — the mosaic case).
- **`WellDetector.fit_well_grid(centers, n_rows, n_cols)`** — fits the known plate
  lattice (iterative snap + least-squares affine, percentile seed + inlier refit).
  Assigns each blob a `(row, col)`, **rejects off-grid spurious blobs**, **fills
  missing wells** from the grid prediction, and handles rotation/skew.

Wired into: the Plate Location **Mosaic scan** auto-detect (`calibration.py`
`_ploc_detect_and_fit_from_mosaic` + the off-thread worker) and the **Map wells…**
dialog's "Auto-detect" (runs on open).

Validated on `config/hardware/mosaics/24.png` (24-well, 4×6): 24/24, grid residual
≈ 5 px (≪ the 513 px pitch). Stress-tested: missing wells filled, spurious blobs
rejected, 4° rotation + noise + dimming all handled (see
`tests/test_v75x_plate_mosaic.py::TestFilledWellDetector`).

## Tools

- `well_finder_viz.py` — runs the SHIPPING detector on a mosaic and writes an
  annotated overlay (no duplicated algorithm). Usage:
  `python "coding plans/well_detection_rnd/well_finder_viz.py" [IMAGE] [ROWS] [COLS]`

## Next ideas (if detection needs to get better on messier plates)

- Per-objective expected radius/pitch prior from `MosaicAlignmentStore` µm/px +
  `MosaicStore` scale to gate blob size precisely.
- Template/normalized-cross-correlation fallback when wells are rings (not filled).
- Use the accumulated `well_training/` labels to tune thresholds / train a model.
