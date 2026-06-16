# MEBP v7.5.x — Freeform-Point Plate Calibration: Interpolating Warp

## Objective

Make the **freeform-point** plate calibration warp the plate so the software
XY coordinate space *matches the measured control points exactly*, replacing
the single global **similarity** fit (rotation + **uniform** scale +
translation, least-squares — never passing through the control points) with an
**interpolating warp**:

> select freeform points → drive the stage to where software *says* each point
> should be → user centers the real feature → software records the **delta**
> (how far off) per point → after all points, **interpolate** those deltas
> across the whole plate so every control point lands exactly and the
> correction blends smoothly between them.

Two concrete deliverables (user-confirmed):

1. **Warp model** — *affine base + thin-plate-spline (TPS) residual*, with
   graceful degradation by control-point count.
2. **Per-point delta display** — show Δx/Δy (and |Δ|) per freeform point as
   it's taught, plus an overall RMS.

## Design

New `SupportClasses/PlateWarpCalibrator.py` mirrors the `MosaicCalibrator`
API (`add_point` / `solve` / `correct_positions` / `n_points`) but fits:

```
warp(p) = A·[p,1]  (linear base: rotation, NON-uniform X/Y scale, shear, shift)
        + tps(p)   (TPS of the leftover residual → interpolates to 0 at every
                    control point, so the total is EXACT at all of them)
```

Degradation by control-point count:

| #pts | model        | property                          |
|------|--------------|-----------------------------------|
| 0    | identity     | no-op                             |
| 1    | translation  | exact                             |
| 2    | similarity   | rotation + uniform scale, exact   |
| 3    | affine       | exact (non-collinear)             |
| ≥4   | affine + TPS | **exact at all control points** + smooth between |

- Pure numpy (no scipy dep, matches `MosaicCalibrator` style). TPS control
  points are centered + RMS-normalized before the kernel solve for
  conditioning (stage µm span ~10⁵). Degenerate/collinear layouts fall back to
  affine-only (and `lstsq`) instead of raising.
- All coordinates are **absolute stage µm** (the frame the calibration page
  already pairs against), so the warp is frame-agnostic.
- Persistence is by **raw control-point pairs** (`to_dict`/`from_dict`),
  re-solved deterministically on load → the exact-at-control-points correction
  is reproduced on restart (not silently degraded to a similarity).
- `similarity_approx()` (polar/SVD of the linear part) feeds the legacy
  `AffineCalibration` (`_three_well_calibration`) used only for the on-screen
  scale/rotation label + the backward-compatible `mosaic_affine` save.

**Scope guard:** only `_manual_fit_xy` (the freeform / manual-teach path) was
switched to the warp. The separate **3-well SVD auto-cal** (`_accept_plate_scan`)
is untouched and still uses the similarity `AffineCalibration`; it now clears
`_plate_warp` so the two paths don't clobber each other's persistence.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PlateWarpCalibrator.py` | **NEW** — `PlateWarpCalibrator` (affine + TPS residual), `_TPS`, helpers; `to_dict`/`from_dict`; `similarity_approx`; raw per-point deltas + mean/max correction. |
| `gui/pages/calibration.py` | `_manual_fit_xy` now fits `PlateWarpCalibrator` instead of `MosaicCalibrator`; stores `self._plate_warp`; status shows mode + avg/max Δ. `_ploc_confirm` records per-point `dx`/`dy`. `_ploc_refresh_queue_view` renders `Δ(dx, dy) µm \|mag\|` for taught freeform points. `_ploc_finish_run` appends overall freeform RMS Δ. `_accept_plate_scan` + plate-change reset clear `_plate_warp`. `_save_calibration` writes `plate_warp` (raw pairs). `_load_calibration` restores the warp **preferentially** (re-solve), guarding the legacy `mosaic_affine` recompute behind `_plate_warp is None`. `__init__` adds `self._plate_warp = None`. |
| `tests/test_v75x_freeform_warp.py` | **NEW** — 12 tests: degradation modes, exact-at-control-points, raw-delta reporting, `correct_positions` key preservation, dict round-trip, collinear robustness, `similarity_approx`. |

## Implementation Steps

- [x] Map current freeform path + transform + frames + conventions (workflow exploration)
- [x] New `PlateWarpCalibrator` (affine + TPS residual, degradation, persistence, deltas)
- [x] `__init__`: add `self._plate_warp = None`
- [x] `_manual_fit_xy`: fit the warp; set `_plate_warp`; approx `AffineCalibration`; status text
- [x] `_ploc_confirm`: record per-point `dx`/`dy`
- [x] `_ploc_refresh_queue_view`: render per-point Δ for taught points
- [x] `_ploc_finish_run`: overall freeform RMS Δ in status
- [x] `_accept_plate_scan` + plate-change reset: clear `_plate_warp` (no cross-path clobber)
- [x] `_save_calibration`: persist `plate_warp` (raw pairs)
- [x] `_load_calibration`: restore warp preferentially; guard legacy `mosaic_affine`
- [x] Tests: `tests/test_v75x_freeform_warp.py`
- [x] Byte-compile + regression run (jog-nav, plate-centering, calibration-revision, objective-cal)

## Testing Notes

- `python -m unittest tests.test_v75x_freeform_warp -v` → **12/12 OK**. Asserts
  the warp passes through every control point within 0.5 µm (the key new
  property), correct degradation modes, raw-delta math, dict round-trip,
  collinear no-crash.
- Regression: `test_v731_jog_navigation`, `test_v75x_plate_centering`
  (32 OK), `test_v744_calibration_revision`, `test_v74x_objective_calibration`
  (46 OK). `gui/pages/calibration.py` byte-compiles.
- **Pending real-HW verification:** run the Plate Location tab in Free mode —
  add ≥4 freeform points, Run, center + Confirm each (watch the per-point Δ
  appear in the queue list), confirm `_calibrated_positions` track the taught
  points and survive an app restart (re-solved from `plate_warp`).

## Issues & Decisions

- **3-well auto-cal left on similarity.** A rigid plate's well-fit cal is
  well-served by the noise-robust similarity; the user's interpolation request
  was specifically about freeform points, so the change is scoped to
  `_manual_fit_xy` to avoid regressing the tested 3-well SVD path. The two
  paths are mutually exclusive in persistence (each clears the other's state).
- **TPS via raw-pairs persistence**, not stored matrices — re-solving on load
  is deterministic, compact, and avoids a schema for the kernel weights. The
  approximate `mosaic_affine` is still written so a downgraded build keeps a
  similarity calibration; load prefers `plate_warp`.
- **Per-point delta = raw (actual − intended)**, computed at Confirm time —
  this is the user-meaningful "how far off it was," distinct from the post-fit
  residual (≈0 for an exact interpolation).
- **Pure numpy TPS** with coordinate normalization for conditioning; collinear
  layouts degrade to affine-only rather than raising.
