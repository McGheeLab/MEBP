# MEBP v7.5.x — Camera orientation in plate/rosette mapping + rosette-map Z-clamp fix

## Objective

Two operator reports (2026-07-24, follow-up to the per-camera rotation/mirror
work):

1. **"The camera size, mirrored status, and rotation all need to be considered
   when mapping the plate, or mapping the rosette."** The mosaic-based plate /
   rosette mapping did not apply the microscope camera's calibrated rotation or
   mirror, so a rotated/mirrored camera mapped wells to the wrong stage XY.
2. **"Odd issue when mapping the rosette where it is clamping Z. It should just
   go to the safe Z travel that I assign."** Rosette (but not plate) mapping
   retracts were overridden by the insert-clearance floor.

Both were root-caused with read-only investigation agents before any edit.

## Part 1 — Camera orientation at mosaic build time

### Finding (investigation)

- **Live-feed click paths already correct.** Manual click-rim well fit,
  re-anchor live click, and click-to-move all route through
  `CameraManager.pixel_to_stage_offset`, which already applies µm/px
  (resolution-aware) + rotation + mirror. No change needed.
- **Mosaic build placed RAW frames.** `MosaicBuilder._blend_tile_to_composite`
  placed each frame with only a `cv2.resize` — no per-frame rotation/mirror
  (only a coarse manual none/rot180/flip dropdown that ignored the calibration
  store). So tile CENTERS were correct (stage-driven) but within-tile pixel
  offsets ran along camera axes, not stage axes.
- **Every mosaic back-projection** (`MosaicWellMappingDialog._on_confirm`,
  `_ploc_fit_from_mosaic_detections`, `MosaicWellRemap.detect_raw_positions`,
  rosette sub-well offsets) is a raw `stage = extent + px/scale` map — µm/px
  only, no rotation/mirror. Consequence: for a rotated/mirrored camera, a
  feature off-centre within a tile back-projects along the wrong stage
  direction (a mirror flips handedness entirely); error grows with distance
  from tile centre (up to ~½ FOV).
- **Correct fix is at BUILD time:** orient every tile into the stage frame
  before placement. Then the composite's axes equal the stage axes and the
  existing raw back-projection is geometrically correct for free — all four
  consumers inherit it with no change. (A global post-stitch canvas transform
  can't work: per-tile orientation about each tile's own centre is lost once
  tiles are blended into a shared canvas.)

### Change

- `MosaicBuilder.__init__` gains `frame_rotation_deg: float = 0.0`,
  `frame_mirrored: bool = False`.
- New `MosaicBuilder._orient_tile(tile)` — mirror (horizontal flip) then
  rotation θ about the tile centre, matching `pixel_to_stage_offset`
  (mirror `dx→−dx` then `R(θ)`). Tile centre invariant → tile stays pinned at
  its stage position. **No-op fast path** when unmirrored and |θ| < 0.05°
  (byte-identical legacy placement). Applied right after the `cv2.resize` in
  `_blend_tile_to_composite`, so both the incremental composite and any rebuild
  (`build_mosaic`) get it, and the plate/rosette/well scan paths all inherit it.
- `CalibrationPage._ploc_microscope_frame_orientation()` reads the microscope
  slot's `CameraManager.get_rotation_deg` / `get_mirrored` (→ `(0.0, False)`
  when unknown) and is passed into the two calibration scan builders
  (`_ploc_start_mosaic_scan_impl` full/rosette scan + `_well_scan_builder`).
- Large non-axis rotations clip tile corners (content rotated within the same
  W×H box); the mapping stays geometrically correct — only peripheral coverage
  is lost, which the scan overlap covers. Documented in `_orient_tile`.
- **Rotation SIGN / mirror axis need real-HW verification** (a mis-signed θ
  rotates the mosaic the wrong way — same caveat as the rotation calibration
  itself); the transform is built to match `pixel_to_stage_offset` by
  construction.

## Part 2 — Rosette-map Z-clamp

### Finding (investigation)

- The rosette scan/map retracts pass the operator's `_safe_z` to
  `safe_travel_to` / `ensure_retracted_to`, but those apply the
  **insert-clearance floor** `_min_travel_z_mm` (raise the retract to clear the
  tallest tube). That floor is armed **only when `plate.max_rim_height_mm > 0`**
  — i.e. only for rosette tube inserts — which is exactly why plate mapping
  never clamped and rosette mapping did.
- Two facets: (a) the floor overrides the assigned safe Z during imaging-height
  mapping travel where it isn't needed (the needle never descends there); and
  (b) `app.py::_update_insert_clearance` computed `top_z + max_rim + margin`
  **without the `z_up_sign` factor** — the identical polarity bug already fixed
  in `PrintTrajectoryPlanner._well_print_z`. On a `z_up_sign=-1` machine that
  lands on the wrong side of the plate top; a too-high floor can also hit the
  raw Z soft-limit clamp.

### Change

- `StageController.safe_travel_to` / `ensure_retracted_to` gain
  `apply_insert_floor: bool = True`. When False the tube-clearance floor is
  skipped and the retract goes to exactly the requested safe Z.
- The calibration **mosaic scan / mapping** retracts pass
  `apply_insert_floor=False` (pre-scan `ensure_retracted_to`, `_MosaicScanWorker`
  first-tile `safe_travel_to`, `_AutoReanchorWorker` `safe_travel_to`). These
  stay at the operator-assigned imaging safe Z the whole time (`target_z_mm=None`,
  never descend), so the tube-clearance floor is unnecessary and was surprising
  the operator. **Print / pick-place travel keep the floor (default True).**
- `app.py::_update_insert_clearance` now adds the clearance with `z_up_sign`:
  `top_z + z_up_sign * (max_rim + margin)` — polarity-correct on both machines
  (no-op on the current `z_up_sign=+1` ME3B V1 profile; fixes `-1`).

### Safety note

The insert-clearance floor is a needle-crash safeguard (clear tall tubes before
travel). It is bypassed **only** for imaging-height mapping/scan travel, where
the needle stays at the assigned safe Z and never descends into a well — so the
bypass cannot drag the needle through a tube (the safe Z is the imaging height,
inherently above the tubes). Printing and pick-place travel, which descend into
wells, keep the floor unchanged.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/MosaicBuilder.py` | `frame_rotation_deg` ctor param; `_orient_tile` (rotation-only); applied after resize in `_blend_tile_to_composite` (mirror moved to the frame source — see addendum) |
| `gui/pages/calibration.py` | `_ploc_microscope_frame_orientation()`; passed into the full/rosette scan + `_well_scan_builder`; `apply_insert_floor=False` on the 3 mapping/scan retracts |
| `SupportClasses/StageController.py` | `apply_insert_floor` param on `safe_travel_to` + `ensure_retracted_to` |
| `gui/app.py` | `_update_insert_clearance` polarity fix (`z_up_sign` factor) |
| `tests/test_v75x_mapping_camera_orient_and_rosette_z.py` | new suite (13) |

## Testing

- New `tests/test_v75x_mapping_camera_orient_and_rosette_z.py` (13):
  floor bypass (default floors / bypass uses assigned safe Z / bypass still
  never descends / `safe_travel_to` accepts the flag), clearance polarity math
  (±z_up_sign), `_orient_tile` (no-op default + near-zero / mirror flips /
  180° / centre invariant), builder carries params.
- Regression green: `test_v731_mosaic`, `test_v75x_mosaic_memory_and_overlay_perf`,
  `test_v75x_mosaic_orientation_remap` (44); `test_v75x_z_retract_before_xy_travel`,
  `test_v75x_jog_travel_off_gui_thread`, `test_v75x_plate_location_z_side_view`
  (36). `test_v75x_plate_mosaic` — [pending background run]. No-op default
  (0°, unmirrored, floor applied) preserves all existing behavior.
- **Needs real-HW verification on ME3B V1:** (1) with the microscope camera's
  rotation/mirror calibrated, scan a plate + rosette → mapped well/sub-well
  centres land correctly (if a mapped well is rotated off, flip the
  `_orient_tile` sign — same as the rotation-calibration caveat); (2) map a
  rosette → the retract goes to the assigned safe Z (no Z clamp), and printing
  over a rosette still clears the tubes.

## Addendum (2026-07-24) — mirror moved to the frame source

Operator follow-up ("mirror the output of the camera feed so what i see is not
mirrored") reworked the mirror handling (see
`MEBP_v75x_CAMERA_ROTATION_CAL_AND_MONITOR_CAMERA.md` addendum): a mirrored
camera is now un-mirrored at the **frame source** (`CameraWidget`), so the
mosaic builder already receives an un-mirrored image. Consequently the
`frame_mirrored` param + the mirror in `_orient_tile` were **removed** — the
builder applies **rotation only** — and `_ploc_microscope_frame_orientation()`
returns rotation only. The `_orient_tile` test now covers rotation only
(mirror-flip test dropped); the rosette-Z half is unchanged. Suites re-run
green (49 across the reworked mirror/orient suites + 80 camera + 79 mosaic).

## Status

- [x] Investigate both (read-only agents)
- [x] MosaicBuilder frame orientation + wire from calibration scan builders
- [x] `apply_insert_floor` bypass on the mapping/scan retracts
- [x] `_update_insert_clearance` polarity fix
- [x] Tests + regression
- [x] Plan doc + CLAUDE.md row
