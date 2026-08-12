# MEBP v7.17.0 — Jog-view zoom crashes Python

## Objective

Operator report: *"on any jog views if we zoom in too far or too aggressively,
python will crash"* — a hard process crash (no Python traceback, no dialog).

Root-cause it, fix it, and pin it so it cannot come back.

---

## Root cause (REPRODUCED — segfault, exit 139)

`JogWorkspaceView._paint_mosaic_overlay` pre-scaled the plate mosaic to its
**full on-screen size** and cached that pixmap. `rect` is the mosaic's on-screen
box, so it grows with the zoom and the cached pixmap's **area grows as zoom²**.

Measured on this machine's real `nest-plastic-24` mosaic (3000×2016 source
spanning the whole XY envelope) on a 1000×780 canvas:

| zoom | `scaled()` request | ARGB32 | result |
|---|---|---|---|
| 1 | 999 × 672 | 0.003 GB | fine |
| 10 | 9991 × 6717 | 0.27 GB | fine |
| 20 | 19981 × 13433 | 1.07 GB | fine, but heavy |
| 30 | 29972 × 20150 | 2.42 GB | past the 2 GiB single-buffer line |
| 40 | 39962 × 26866 | 4.29 GB | allocated, ~pathological |
| **60** (`_ZOOM_MAX`) | **59943 × 40299** | **9.66 GB** | **SEGFAULT** |

Reproduced directly: `QPixmap.scaled(59943, 40299, …)` on the real mosaic →
`Segmentation fault`, exit **139**, no Python exception. That is the operator's
crash. `.transformed()` for the 180° plate flip would double it again, and the
previous cache was still alive during the rebuild, so peak was ~3× the figure.

**"Too aggressively" is the same bug**, not a second one: a fast wheel flick
jumps several zoom steps in one gesture, and each intermediate repaint rebuilt a
fresh giant pixmap.

Two things this was NOT (both checked): the crash is not in `XZSideView` (it
paints no pixmaps; its Z zoom only remaps the axis and survives its 30× max),
and not in the fluorescence overlay (that one scales painter-side, which is
memory-bounded by the clip).

⚠ **Why the guard was missing:** the cache was added in
`MEBP_v75x_MOSAIC_MEMORY_AND_OVERLAY_PERF.md` to stop a full re-scale on every
repaint — correct, and it fixed a real lag — but zoom + pan
(`MEBP_v75x_MOSAIC_ORIENTATION_ADJUST_TOOL.md` era) later made the on-screen
size operator-controlled and unbounded, and nothing tied the two together.

---

## The fix

**Never scale the cache beyond the SOURCE pixmap's own resolution.** Upscaling a
3000 px source to 60000 px invents no detail; the painter magnifies the capped
cache into the **same destination rect** under the `SmoothPixmapTransform` hint
`paintEvent` already sets. Peak memory becomes O(source) instead of O(zoom²).

Result: the cache is **24 MB at every zoom** (was 9.66 GB at `_ZOOM_MAX`), while
the destination rect still grows exactly as before.

Two properties make this safe rather than merely smaller:

1. **Below the cap nothing changes at all.** The `scaled()` request is still the
   exact on-screen size and is drawn 1:1 — rendering is **byte-identical**
   (sha256-compared before/after at zooms 1/2/3, the zooms the old code
   survived). The fix only alters the regime that used to crash.
2. **Zooming above the cap now allocates nothing.** Once the request pins to the
   source size the cache key stops changing, so further zooming reuses it and
   only the destination rect moves — the aggressive-wheel case is allocation-free
   exactly where it used to die.

Also: the previous cache is released *before* the new scale, since at high zoom
those are the two largest allocations in the process.

⚠ **THE DANGEROUS WRONG FIX, avoided deliberately:** clamping the **destination**
rect instead. That bounds memory too, but silently shrinks the drawn mosaic so it
no longer registers with the well grid / needle — the operator would be shown a
feature at the wrong place, on a view whose whole job is registration. All
destination geometry is untouched here, and `TestDestinationRect` fails if anyone
"fixes" it that way (mutation-verified).

Registration is display-only regardless: mosaic tiles are placed at trusted raw
stage positions and every consumer back-projects
`stage = (extent − shift) + px/scale` for MOTION, so nothing in this diff can
move the stage.

---

## Files Modified

| File | Rationale |
|---|---|
| `gui/widgets/jog_workspace_view.py` | Cap the cached mosaic scale at the source resolution; draw into an explicit destination rect (unchanged geometry); release the old cache before rebuilding. |
| `tests/test_v717_jog_view_zoom_memory.py` | NEW — 16 tests. |
| `coding plans/Update plans/MEBP_v717_JOG_VIEW_ZOOM_CRASH.md` | NEW — this document. |

Inherited for free: `WorkspaceTargetView` subclasses `JogWorkspaceView`, so the
spheroid / cell-targeting / cell-labeling workflow views are fixed too.
`PrintTrajectoryMonitorView` has no wheel zoom and is unaffected.

---

## Implementation Steps

- [x] Reproduce the crash and identify the exact failing allocation
- [x] Confirm `XZSideView` and the fluorescence overlay are NOT the cause
- [x] Cap the cached scale at the source resolution
- [x] Keep the destination rect (and the 90/270 transpose rule) unchanged
- [x] Release the previous cache before rebuilding
- [x] Verify byte-identical rendering below the cap
- [x] NEW test file (16 tests)
- [x] Mutation-verify the tests
- [x] Regression suites
- [ ] **GUI verification on ME3B V1 (see below)**

---

## Testing Notes

`tests/test_v717_jog_view_zoom_memory.py` — **16 tests, all green.** Drives the
PRODUCTION `JogWorkspaceView` through real `paintEvent`s (offscreen), recording
every `QPixmap.scaled` request and every `drawPixmap` destination rect. The
fixture is built in code (synthetic 3000×2016 pixmap on the real ME3B V1
envelope) so the suite does not depend on one machine's files.

Includes `test_the_naive_answer_would_have_been_catastrophic` — a guard on the
guard, asserting the un-capped request would still be multi-GB and >100× the
capped one, so the bounding tests cannot pass merely because nothing scales.

**3/3 mutations CAUGHT** (source hash-verified restored after each):

| # | Mutation | Result |
|---|---|---|
| M1 | restore the unbounded `scaled(tw, th)` (the original bug) | CAUGHT — *"zoom 5: asked for a 4960px-wide pixmap from a 3000px source"* |
| M2 | clamp the destination rect to the cache size (the dangerous wrong fix) | CAUGHT — *"dest rect stopped tracking the zoom — the overlay would mis-register"* |
| M3 | drop the release-before-rebuild | CAUGHT — previous cache still alive during the new allocation |

⚠ **My first M2 was an INVALID mutation and is recorded as such:** it moved a
variable use above its definition, so it "failed" with `UnboundLocalError`
instead of exercising the clamped-destination hypothesis. A mutation that merely
breaks the code proves nothing; it was rewritten to mutate the `drawPixmap`
destination and then genuinely failed `TestDestinationRect`.

⚠ **Two of my own tests were wrong and the run corrected me:** I asserted that a
zoom change always rebuilds the cache. Above the cap it does not — and *that is
the point*, because the capped key stops changing. Rewritten to test rebuild
below the cap, plus a new test pinning the no-allocation property above it.

Regression, run per-suite (this repo's documented cross-suite hang):
mosaic-memory-perf 7 · mosaic-orientation-adjust 19 · orientation-remap 14 ·
jog-navigation 28 · plate-orientation-convention 17 · custom-plate-rendering 70 ·
fluorescence-mosaic 35 · rosette-flatten 32 · unified-mosaic-cal 37 ·
quick-print-trajectory-view 13 · mosaic-plate-frame 54 · camera-square-crop 75 —
**401 green**, plus a `gui.app` import smoke.

One pre-existing failure PROVED not ours: `test_v75x_rosette_tab_auto_reanchor::
test_tab_order_and_indices` (*"Plate Bed Level" != "Plate Z Auto-Cal"*) — the
v7.11 tab rename already recorded twice in CLAUDE.md; fails identically with this
change stashed.

### Needs GUI verification on ME3B V1

1. Open Jog with a full-plate mosaic shown and **wheel-zoom hard, all the way in
   — it must not crash** (this is the whole fix).
2. Zoom in on a feature, note where it sits relative to its well ring, and
   confirm the mosaic **still lines up with the well grid and the needle** at
   every zoom (a shrunken or offset overlay means the destination geometry
   regressed).
3. Click a feature at high zoom and drive to it — the needle must land on it.
4. At extreme zoom the mosaic is magnified past its own resolution, so it will
   look soft/blocky. **That is expected and was always true** — the pixels do not
   exist; the old code faked them at 9.7 GB and then died.
5. Repeat with the fluorescence overlay on, and on a workflow page (spheroid /
   cell targeting) which uses the same view.

---

## Issues & Decisions

- **Cap at the source resolution, not an arbitrary pixel ceiling.** The source
  is the information limit, so this is principled rather than tuned, and it
  automatically scales with whatever `MosaicBuilder.target_mosaic_px` produces.
- **Did NOT lower `_ZOOM_MAX`.** That would take away magnification the operator
  explicitly asked for ("magnify small features seen in the mosaic") and would
  only move the crash threshold rather than remove it — a bigger monitor or a
  larger mosaic would find it again.
- **Did NOT crop to the visible region instead.** Cropping would preserve full
  smooth-scaled sharpness at any zoom and is the better end state, but the crop
  math has to compose with the 180° plate flip and the operator's 0/90/180/270
  output rotation — i.e. it puts registration at risk to buy sharpness. Capping
  touches no destination geometry at all. Recorded as a possible follow-up.
- **Left the fluorescence overlay alone.** It scales painter-side, which is
  bounded by the clip; a test now pins that so a future "cache it like the
  mosaic" optimisation cannot reintroduce this crash there.
- **Left `XZSideView` alone** — verified it survives its 30× max; it was named in
  the report only because it shares the Jog page.
