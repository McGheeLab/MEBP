# MEBP v7.5.x — Full-plate mosaic stalls after one row (unreachable raster points)

**Operator report (2026-07-30):** *"the full plate mosaic is not able to get past
a single row of images. I think there is an issue with the data flow and
management."*

---

## Objective

Make the full-plate mosaic raster survive a scan region that the stage cannot
actually traverse: detect non-arrival from the **measured** stage position, stop
re-commanding the unreachable region, stop stitching tiles captured at the wrong
place, and tell the operator what happened — instead of grinding through a flat
10 s arrival timeout per dead grid point until the scan is cancelled.

---

## Root cause (from the operator's own `logs/app.log`, 18:08–18:12)

```
18:08:32  PlateLocation mosaic scan: 912 tiles over (0, 0, 116327, 74235) µm
18:08:36  XY absolute move: (4792, 1652)          <- row 0 advancing, ~1 s/tile
   …      … 27 tiles OK, X climbing 1652 → 80134 …
18:09:00  Failed to parse XY position: Expected 3 values, got 1: R
18:09:10  wait_for_xy_arrival timeout (10.0s): target=(83.27, 1.65), actual=(80.593, 1.652)
18:09:21  wait_for_xy_arrival timeout (10.0s): target=(86.41, 1.65), actual=(80.593, 1.652)
   …      … every remaining tile, 10 s each, X frozen at 80.593 mm …
18:11:05  XY absolute move: (114675, 4792)        <- row 1 starts at the far end
18:11:15  wait_for_xy_arrival timeout (10.0s): target=(114.67, 4.79), actual=(80.593, 4.792)
   …      … row 1 grinds back through the same dead band …
18:12:57  PlateLocation mosaic scan cancelled
```

**The scan region is the configured XY safety envelope** — the whole-plate
default is `_ploc_whole_plate_scan_bounds() → env`, and `ME3B V3.json` records
`xy_max_x = 116327 µm`. The stage physically stopped at **X = 80593 µm** and
never moved in X again (Y kept tracking its commands exactly, which is how we
know this was real travel exhaustion and *not* the known stale-`R`-ack read
desync — a desynced read would have mis-reported both axes).

From that column on the worker:

1. ignored `wait_for_xy_arrival`'s return value entirely,
2. burned the **flat 10 s** timeout on every remaining point,
3. grabbed a frame anyway and **stitched a duplicate tile** at the same canvas
   spot (feeding the overlap registration a bogus measurement), and
4. hit the serpentine reversal, which starts row 1 at the *far, unreachable*
   end — so the next row re-ground the same dead band.

11 dead points × 10 s ended row 0; row 1 immediately started 9 more. At ~2
minutes of dead waiting per row against ~30 s of real scanning, the scan
appeared never to leave the first row. Nothing warned; the mosaic would have
been silently partial.

**Why the travel ran out is a separate, machine-side question** and is *not*
changed here: the taught plate map in `settings.json` spans X 10162 → 106427 µm
with `taught_a1 = (105618, 65890)`, i.e. the stage *could* reach ~106 mm when the
plate was taught. A Prior ProScan sets position 0 at power-on wherever the stage
happens to be sitting, so a saved *absolute* envelope (and the taught well map)
only line up across a power cycle if the stage starts from the same place. Both
the "envelope recorded too large" and the "origin has shifted" explanations are
named in the operator-facing message rather than guessed at.

---

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/calibration.py` (`_MosaicScanWorker`) | Measured-position arrival check; distance-sized arrival timeout; runtime reachable-travel box; skip (don't re-command) unreachable points; don't stitch a tile captured off-target; scan summary. |
| `gui/pages/calibration.py` (`_ploc_on_mosaic_finished`) | Capture the worker's reachability report before releasing it and warn the operator that the mosaic is partial. |
| `main.py` (`setup_logging`) | Silence the `comtypes` DEBUG firehose (see Issues). |
| `tests/test_v75x_mosaic_unreachable_travel.py` | NEW — 9 tests. |

---

## Implementation Steps

- [x] **Judge arrival from the measured position, not the boolean.** After each
  move the worker reads `get_xy_position(cached=False)` once and compares to the
  requested grid point (`_ARRIVE_TOL_UM` = 150 µm). This is deliberately *not*
  gated on `wait_for_xy_arrival`'s return value — that call is documented to time
  out spuriously when a stale Prior `R` ack desyncs the read, so trusting it
  would drop tiles the stage *did* reach. A failed/garbled read (`(None, None,
  None)`) degrades to the legacy behaviour (place at the commanded target).
- [x] **Don't stitch an off-target tile.** Its frame shows somewhere else;
  blending it stacks a duplicate on the canvas and injects a false overlap
  measurement into registration. The same single read now also supplies the
  placement position, halving the per-tile XY serial traffic (it used to read
  twice: once inside the arrival wait, once after the frame grab).
- [x] **Distance-sized arrival timeout** (`_arrival_timeout_s`):
  `2 × dist/max_xy_speed + 2 s`, clamped to 3–20 s. A dead 3.1 mm raster step now
  costs ~3 s instead of a flat 10 s, and a genuinely long move still gets room.
  Falls back safely when the controller exposes no `safety_limits`.
- [x] **Runtime reachable-travel box** (`_note_unreachable` / `_reachable`).
  After `_MAX_CONSEC_STALL` = 3 consecutive non-arrivals — three, so a one-off
  comms hiccup cannot permanently truncate a scan — the box is clipped on the
  axis *and direction* that fell short only. A stage short in +X keeps its whole
  Y range.
- [x] **Skip, don't re-command.** Points outside the discovered box are skipped
  before any move, still emitting `progress` so the operator's `n/total` counter
  reaches the end. The serpentine's reversal now costs nothing.
- [x] **Report it.** Worker exposes `unreachable_skipped` + `reach_note`; the
  scan logs a summary with the discovered box; `_ploc_on_mosaic_finished` shows a
  warning naming both plausible causes and telling the operator to verify a known
  well before trusting the mosaic.

---

## Testing Notes

`tests/test_v75x_mosaic_unreachable_travel.py` (9, all green) drives the real
`_MosaicScanWorker.run()` synchronously against a stage stand-in whose X travel
ends mid-grid (39-point raster, 12 reachable):

- scan **completes** (does not hang or fail), stitches exactly the 12 reachable
  tiles, skips 27, and emits progress for all 39;
- **only 15 moves are commanded** instead of 39 — the dead region costs one
  3-stall burst per discovery, not one timeout per point;
- only the short axis is clipped (full Y range still scanned);
- a **single transient miss does not clip the grid** (box untouched, all points
  still commanded, only that one tile dropped);
- a tile that *did* arrive is still stitched **even when `wait_for_xy_arrival`
  always returns False** (stale-`R`-ack regression guard);
- a failed position read falls back to the commanded target;
- timeout scaling/clamping, and survival of a controller with no `safety_limits`.

Regression (all green): `test_v75x_plate_mosaic` worker/UI/bounds/settings/
detect-fit/reanchor/overlay/reentrancy (27), `test_v75x_mosaic_orientation_remap`,
`test_v75x_mosaic_orientation_adjust`, `test_v75x_mosaic_memory_and_overlay_perf`,
`test_v75x_unified_mosaic_calibration`, `test_v75x_rosette_tab_auto_reanchor`,
`test_v75x_single_well_mosaic_reregister`, `test_v75x_startup_well_map_persistence`,
`test_v78_fluor_mosaic_shift`, `test_v75x_plate_location_manual_click_rim`,
`test_v75x_plate_location_workflow_toggle`, `test_v75x_fluorescence_mosaic`,
`test_v75x_nikon_ti_microscope`, `test_v75x_z_retract_before_xy_travel` — **309**.
`test_v75x_plate_mosaic.TestManualAlignPage` was excluded as usual (documented
pre-existing hang).

**Needs real-HW verification on the rig**: re-run the full-plate scan and confirm
it now advances past row 1 at ~1 s/tile; if travel is still short, the warning
dialog should name the stopping point. Then settle the underlying question —
whether `xy_max_x = 116327` is simply too large, or the stage origin moved (drive
to a known taught well and see whether the needle lands on it).

---

## Issues & Decisions

- **Skip-and-continue, not abort.** An out-of-travel region makes the mosaic
  partial, not worthless — the reachable area still maps correctly and the
  canvas simply stays black elsewhere. Aborting would discard good tiles; the
  warning dialog carries the honesty instead.
- **3 consecutive stalls before clipping.** Clipping on the first miss would let
  one comms hiccup permanently truncate a 900-tile scan. Three costs ~9 s once.
- **The envelope/origin question is left to the operator.** The recorded XY
  envelope is *their* measurement, and the evidence (a taught map reaching 106 mm
  vs. a stage stopping at 80.6 mm) is consistent with either a bad envelope or a
  shifted power-on origin. Silently rewriting `safety_limits` would be guessing
  at a value that governs every clamped move on the machine.
- **Also fixed en route: the `comtypes` DEBUG firehose.** The Nikon Ti panel
  polls the scope's COM object model about once a second and `comtypes` logs
  ~10 DEBUG lines per COM call — ~35 000 lines (several MB) during this
  4-minute scan alone, rotating `logs/app.log` every few minutes and burying the
  app's own output (the mosaic diagnosis above required filtering it out).
  `setup_logging` now pins `comtypes` (+ `PIL`, `matplotlib`) to WARNING, so real
  COM errors still surface while the app's own modules stay at DEBUG. Unrelated
  to the stall, but it actively obstructs diagnosing it.
- **Not changed:** the scan's own settings. This run used **5 % overlap** and a
  **2 ms** post-move settle; the code already warns about both at scan start
  (`25 %` and `300 ms` are the recommended values) and they are operator knobs,
  not defects. Worth revisiting if the stitch looks gappy or blurred.
