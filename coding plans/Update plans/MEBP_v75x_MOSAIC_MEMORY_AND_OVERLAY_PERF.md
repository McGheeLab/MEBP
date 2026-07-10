# MEBP v7.5.x — mosaic memory + overlay-paint performance

## Objective
`changes_needed.md` items 1 & 2:
- (1) "when building a mosaic, the captures are very fast at first, then by the 300th
  tile everything slows down… RAM 80-90%… a data flow problem."
- (2) "after I build the mosaic the software is very laggy."

## Root cause
- (1) `MosaicBuilder._records` retained **every raw frame** (~2 MB each → ~½ GB at
  300 tiles); the float64 `_composite` + `_weight_sum` (~190 MB) lived for the whole
  scan → RAM pressure → swap thrash → seconds/tile.
- (2) `JogWorkspaceView._paint_mosaic_overlay` re-scaled the full mosaic pixmap (~18
  MB) with a 180° painter transform on **every** paintEvent (needle moves, hovers).

## Files Modified
- `SupportClasses/MosaicBuilder.py`
  - New ctor flag `retain_frames=True`. When False, `stitch_incremental` frees each
    tile's `rec.frame` right after blending (the only place it's needed; this path
    never re-blends). `build_mosaic`/`tile_images_px`/`_ensure_composite` skip freed
    frames (defensive).
  - New `free_accumulators()` — releases `_composite` + `_weight_sum`, keeps the uint8
    `_display_cache` (the finished mosaic; the global shift is applied via
    `canvas_extent_um`, not by re-normalizing, so the result is unaffected).
- `gui/pages/calibration.py`
  - Full-plate scan builder constructed with `retain_frames=False`.
  - `_MosaicScanWorker.run` calls `free_accumulators()` after detection + the final
    composite copy, before `finished_ok`.
- `gui/widgets/jog_workspace_view.py`
  - `_paint_mosaic_overlay` scales (+180° rotates) the overlay ONCE into
    `_mosaic_scaled_cache`, keyed by `(on-screen w, h, flip, source id)`. The manual
    nudge only moves the rect's top-left (size-invariant), so pan/needle-motion
    repaints reuse the cache; zoom / new overlay / flip rebuild it. `set_mosaic_overlay`
    invalidates the cache.

## Scope notes
- Frame-light + `free_accumulators` are applied ONLY to the full-plate Plate Location
  scan (the reported problem). The per-well scan (`build_mosaic`) and the mosaic
  calibration dialog (`tile_images_px`) keep the default `retain_frames=True`.
- A per-tile preview-emit throttle was prototyped but reverted — it broke the
  "one preview per tile" worker contract, and the memory fixes already resolve the
  slowdown (per-tile copies are cheap once RAM isn't thrashing).

## Implementation Steps
- [x] `retain_frames` mode + free-frame-after-blend + defensive skips
- [x] `free_accumulators()` + worker call after the scan
- [x] Full-plate builder uses `retain_frames=False`
- [x] Cached scaled+rotated overlay pixmap in `JogWorkspaceView`
- [x] Tests `tests/test_v75x_mosaic_memory_and_overlay_perf.py` (8)

## Testing
`tests/test_v75x_mosaic_memory_and_overlay_perf.py` — frame-light frees frames while
keeping the composite; retain default keeps them; `free_accumulators` releases the
float64 buffers but keeps the display cache; `tile_images_px` skips freed frames;
overlay cache builds, reuses on repaint + pan, invalidates on a new overlay. Full
`test_v75x_plate_mosaic` + orientation-remap suites green (only the documented
pre-existing CV detection failure `test_real_24_well_mosaic` remains).

## Needs real-HW verification on ME3B V1
Full-plate scan: RAM stays bounded, tiles stay fast past 300, app responsive after
build (overlay pan/zoom smooth).
