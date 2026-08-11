# MEBP v7.13 — Mosaics referenced to the plate, several scans per plate, and a visual picker

## Objective

Operator: *"in the plate selection tool, we have the ability to select the
mosaic that will be used for that plate if there are multiple. make sure that
the mosaic is referenced to the plate coordinate system and the xy stage
coordinate system. this allows a recalibration later to offset the mosaic to
the live view which we already have. in the plate selection tool, we should be
able to see the mosaic scan that we are selecting."*

Plus the operator's framing of the plate coordinate system: a plate should have
its own frame, offset from stage 0,0, defaulting to centred in the stage's
travel range, with calibration measuring the true offset.

---

## What already existed (assessed before building)

The plate-frame half was largely in place and is unchanged by this work:

| Requirement | Where it already lives |
|---|---|
| plate-local coordinate system | `WellPlate.get_well_position` — A1-relative mm |
| default offset centres the plate in the stage travel range | `StageController.default_plate_center_um()` → `safety_limits.xy_center()` → `get_all_positions_from_plate_center` (v7.5.x `PLATE_CENTERING`) |
| calibration measures the true offset | `taught_a1` + `plate_warp`, archived per plate in `calibration_by_plate` |

The genuinely missing pieces were: a **bug** in that centring, and everything
about the mosaic.

---

## The centring bug

`WellPlate.get_a1_from_plate_center` centred every plate using **hardcoded ANSI
constants**:

```python
a1_x_um = center_x_um + sx * (self.a1_offset_x - PLATE_FOOTPRINT_X_MM / 2.0) * 1000.0
```

Any carrier that is not 127.76 × 85.48 mm therefore defaulted to a position off
by half the footprint difference — **33.9 mm × 22.7 mm for a 60 × 40 carrier**,
far outside any well.

Fixed with a new `WellPlate.footprint_mm` property, resolved from the authoring
`PlateDocument`'s boundary (where the footprint is actually declared) and cached,
falling back to the ANSI constants when the plate cannot be resolved — which is
the value it used to hardcode, so a plate that *is* standard is unaffected. Every
standard format now centres its outline **exactly** on the requested point.

---

## MosaicStore v1.0 → v2.0

### Several scans per plate

A plate key now maps to a container:

```json
"<plate_key>": { "active": "s2", "scans": { "s1": {…}, "s2": {…} } }
```

Single-well entries (`"<plate_key>#<well>"`) stay bare leaves — a single-well
scan is inherently single.

**The public API did not change.** `get_meta` resolves the container to the
**active** scan and returns exactly the pre-v7.13 shape; `image_path`,
`load_image`, `get_extent_um`, `get_shift_um`, `get_wells`, `has` are all built
on it and so resolve too. That is what let this land without touching the nine
consumer modules and eleven other test files that read this store — the same
invariant that made the v7.12 plate work shippable.

v1.0 files are normalised **in memory** on load, so merely opening the app does
not churn the operator's data; the file is rewritten in v2.0 form only when
something else causes a save.

New API: `list_scans`, `active_scan_id`, `set_active_scan`, `rename_scan`,
`delete_scan`, `plate_frame`, `needs_rescan`, `reanchor`.

### Referenced to BOTH coordinate systems

Each scan now stores, beside its stage extent:

```json
"plate_frame": {
  "extent_mm": [x0, y0, x1, y1],   # the same rectangle, in the plate frame
  "anchor_um": [a1_x, a1_y],       # the taught A1 it was tied to
  "axis_sign": [sx, sy]
}
```

Module functions `stage_to_plate_mm` / `plate_mm_to_stage_um` convert, exactly
inverse for every axis sign and re-normalising the rectangle (a negative sign
swaps corners, and an un-normalised extent silently breaks every `min_x`-style
consumer downstream).

### The correction is applied once, at re-teach

`MosaicStore.reanchor(plate_key, anchor_um, axis_sign)` rewrites each scan's
**stage** extent from its unchanged **plate** extent. It is called from
`CalibrationPage._save_calibration` — the one commit funnel every XY teach path
reaches — guarded so it fires only when the taught A1 actually moves.

Doing it there rather than inside `get_extent_um` is deliberate: the **eight
reader sites** across the app keep calling the store exactly as they always have
and still see a corrected extent. `get_extent_um(key, anchor_um=…)` exists too
for a preview, and defaults to the stored value so every existing caller is
byte-identical.

`_ploc_shift_mosaic_by` (the manual nudge) now re-derives the plate frame from
the new extent — nudging is the operator saying "the image sits HERE relative to
the plate", so leaving the old reference would make the next re-teach undo it.

---

## Migration: existing scans require a re-scan (operator decision)

Scans on disk carry no plate frame. They are reported by `needs_rescan()` and
labelled in the UI, and they keep working **exactly** as before: stage frame,
manual re-anchor, full display. What they cannot do is follow a re-teach.

A plate-frame extent is deliberately **not** back-filled for them. Doing so
would have to assume the calibration that was live when the scan was taken, and
a wrong assumption puts the mosaic — and every well centre mapped off it —
silently in the wrong place. This follows the v7.8 precedent where `has_shift`
distinguished "recorded as zero" from "never recorded".

Verified against the operator's real store: all three existing scans (`24`,
`nest-plastic-24`, `plate-24_Rossette A1`) report `needs_rescan=True` and still
load.

---

## The plate selection tool

- **`PlateThumbnail.set_mosaic(pixmap, extent_mm)`** draws the scan behind the
  wells at its own extent, clipped to the card — so a mosaic that does not cover
  the whole plate visibly does not, instead of being stretched to fit. A scan
  with no known extent is **not drawn**: showing a registration that does not
  exist is worse than showing none.
- **`mosaic_extent_in_doc_frame`** converts out of the A1-**well** frame (where
  a scan's plate frame lives) into the document's **storage** frame (where the
  thumbnail draws). New `PlateDocumentStore.a1_well_offset_mm` is the one place
  that offset is computed. The two frames coincide on a standard plate, which is
  exactly why mixing them goes unnoticed until a pattern is authored away from
  the datum.
- **NEW `MosaicPickerDialog`** replaces the `QInputDialog` list of plate keys:
  scan list on the left, the actual image on the right, tiles/date/reference
  status underneath, and Use / Rename / Import-from-another-plate / Delete.
  Previewing a scan restores the active slot afterwards, so merely *looking* at
  a scan never re-points the plate at it.
- The card's mosaic line now names the active scan, says how many there are, and
  flags `⚠ re-scan to track the plate`.

---

## Files modified

| File | Change |
|---|---|
| `SupportClasses/MosaicStore.py` | v2.0 schema, container/leaf normalisation, multi-scan API, plate-frame reference, `reanchor`, frame-conversion functions |
| `SupportClasses/WellPlate.py` | NEW `footprint_mm`; `get_a1_from_plate_center` uses it |
| `SupportClasses/PlateDocumentStore.py` | NEW `a1_well_offset_mm` (factored out of the footprint resolver) |
| `gui/pages/calibration.py` | NEW `_ploc_plate_anchor`; three mosaic saves record it; `_save_calibration` re-anchors |
| `gui/pages/hardware/plate_library.py` | `mosaic_pixmap`, `mosaic_extent_in_doc_frame`, `PlateThumbnail.set_mosaic`, NEW `MosaicPickerDialog`, richer card label |

---

## Testing notes

`tests/test_v713_mosaic_plate_frame.py` — **54 tests, green.** The first class,
`TestPreV713FilesReadUnchanged`, writes a real v1.0 file and drives every legacy
reader through it; if that class fails, the whole "the API did not change" claim
is void.

**10 / 10 mutations confirmed CAUGHT:**

| Mutation | Caught by |
|---|---|
| drop the plate-frame record on save | frame-conversion + reanchor tests |
| re-anchor also rewrites the PLATE frame | `test_the_plate_frame_itself_is_unchanged_by_a_reanchor` |
| re-anchor also moves legacy scans | `test_legacy_scans_are_left_alone` |
| a second scan overwrites the first PNG | `test_a_second_scan_does_not_overwrite_the_first` |
| `get_meta` returns the container | multiple resolver tests |
| `get_extent_um` ignores a supplied anchor | `test_get_extent_um_replaces_against_a_supplied_anchor` |
| centre by the hardcoded ANSI footprint again | `test_a_non_ansi_carrier_is_centred_by_its_own_footprint` |
| preview leaves the active scan switched | `test_previewing_a_scan_does_not_change_which_one_the_plate_uses` |
| mosaic extent skips the A1-well offset | `test_extent_is_converted_out_of_the_A1_WELL_frame` |
| stop recording the anchor on the full-plate scan | `test_every_mosaic_save_records_the_plate_anchor` |

⚠ **Two of my own tests were too weak and mutations caught them first.** The
picker-preview test swept every row and ended on the row that was already
active, so it asserted the right answer for the wrong reason; it now lands on a
non-active row and self-checks that it did. The calibration-wiring AST matcher
keyed on the method name `save`, which also matches `PlateDocumentStore.save`
and the objective store's; it now keys on `mosaic_scale` (unique to this
signature) and asserts it finds exactly the three known sites, so a matcher that
found nothing could not pass vacuously.

**Regression, run per-suite (~570 green):** fluor-mosaic-shift (11),
single-well-mosaic-reregister (26), fluorescence-mosaic (35), target-types (81),
target-type-editor (27), plate-builder-UI (103), plate-identity-and-stores (42),
custom-plate-rendering (70), plate-centering (6), plate-types (38),
jog-navigation (28), click-rim (27), startup-well-map (6),
last-known-calibration (21), calibration-revision (20),
mosaic-orientation-remap (14), workflow-toggle (21), suite-hygiene (10), plus
`test_v75x_plate_mosaic` (110, excluding the documented `TestManualAlignPage`
hang). Plus a `gui.app` import smoke and a real `PlateLibraryPage` built,
refreshed and rendered offscreen (9 cards).

**Pre-existing failures, proved not from this change:**
`test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices` (the
operator's uncommitted v7.11 "Plate Bed Level" tab rename) and
`test_v75x_plate_mosaic::test_real_24_well_mosaic` (the legacy blob detector;
`VisionDetector.py` is untouched here). Both already documented in CLAUDE.md.

---

## Needs GUI / hardware verification on ME3B V1

In order:

1. Open **Hardware Setup → Plate**. Each card's mosaic line should name its
   scan and show `⚠ re-scan to track the plate` for the three existing scans.
2. Click the mosaic button on a card — the picker should show the scan image,
   not just a key.
3. **Re-scan a plate** (Calibration → Plate Location → Mosaic scan) with a
   taught calibration live. The card should stop saying `re-scan`, and the
   picker should report *plate-referenced*.
4. Scan the same plate again — the plate should now list **two** scans; switch
   between them and confirm the overlay on Plate Location and Jog changes.
5. **The payoff:** re-teach the plate (move it slightly, re-run Map wells). The
   mosaic should follow the wells with no manual "Re-anchor mosaic".
6. Confirm a **legacy** scan does *not* move on a re-teach (it still needs the
   manual nudge) — that is the documented, deliberate behaviour.
7. Author a **non-ANSI carrier** (say 60 × 40 mm) and confirm the uncalibrated
   default plate lands centred in the travel envelope rather than tens of mm off.

## Deliberately not done

The operator's fourth option — making the plate→stage relationship **one
explicit stored offset** rather than implicit in the taught well dict — was
scoped out of this change. It touches calibration persistence for every taught
plate and deserves its own separately-verified pass.
