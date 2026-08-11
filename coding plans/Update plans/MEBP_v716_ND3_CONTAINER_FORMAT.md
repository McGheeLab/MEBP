# MEBP v7.16 — `.nd3` Container Format (HDF5-based)

## Objective

An ND2-like multidimensional image container **we control**, named `.nd3`, holding
everything needed to reconstruct images elsewhere — for use across the operator's
ecosystem (MEBP, Blender scenes, LabLink, future "nd3 studio" tools). Operator:
*"lets make the best file system that stores all that we need to reconstruct images
elsewhere. we can call the file system .nd3 we will use this in other software like
nd2 studios blender, and lab link."*

Writing real ND2 was investigated and rejected: the format is proprietary, every
open library (`nd2`, `nd2reader`, Bio-Formats) is read-only, and the only writer is
NIS-Elements itself — a byte-level mimic would be validated against a reverse-
engineered understanding with NIS-Elements as the only oracle. Instead `.nd3` keeps
ND2's *semantics* (multi-channel / multi-dimensional pixel data + physical
georeferencing) and adds what ND2 never carried for this platform:

- **Dual-frame georeferencing** — absolute stage µm AND A1-relative plate mm
  (the v7.13 `plate_frame` model), per image.
- **Explicit pixel→world affine matrices** (3×3, stored, authoritative) — this
  repo's history of frame/sign bugs is exactly why derived transforms are written
  down instead of re-derived by each consumer.
- Channel pseudo-colors, frozen display levels, acquisition + provenance records
  in the `CaptureMeta` vocabulary.

## Decisions (operator, via AskUserQuestion)

1. **Container = HDF5 (h5py)** — chosen over ZIP+npy+JSON and over an ND2-style
   bespoke chunk binary. Consequences accepted: consumers need `h5py`
   (pip-installable into Blender's bundled Python; the spec documents the
   one-liner). What HDF5 buys: chunked datasets with partial reads (a 12k-px
   mosaic can be read tile-wise), per-dataset gzip + Fletcher32 checksums,
   extendable datasets (append frames to a time-lapse without rewriting), and a
   self-indexing tree (no separate manifest that could desync from the data).
2. **Exporters v1 = all four sources**: fluorescence well scans, plate mosaics,
   camera captures/stills, video/time-lapse.
3. **Backend + spec only** — no GUI this pass. Hook points recorded below.

## Files

| File | Status | Purpose |
|---|---|---|
| `SupportClasses/ND3.py` | new | Core format: writer/reader/verify/sniff. Imports **stdlib + numpy + h5py ONLY** — no repo imports, no cv2, no Qt. Vendorable as a single file into Blender/LabLink. |
| `SupportClasses/ND3Export.py` | new | Store-facing exporters (imports cv2, the stores, CaptureMetadata, ND3). |
| `docs/ND3_SPEC.md` | new | The format spec — a first-class deliverable; third parties implement readers from it. |
| `tests/test_v716_nd3_container.py` | new | Core format tests (GUI-free). |
| `tests/test_v716_nd3_export.py` | new | Exporter tests against real store instances in tmp dirs. |
| `requirements.txt` | edit | `h5py>=3.12` added (numpy pin untouched — verified installed beside numpy 2.4.6). |
| `CLAUDE.md` | edit | Existing Update Plans table row. |

## Format summary (normative text lives in `docs/ND3_SPEC.md`)

`.nd3` = an HDF5 file with a defined tree. Sniff = HDF5 magic
`\x89HDF\r\n\x1a\n` + root attr `format == "nd3"`.

```
/                        attrs: format="nd3", schema_version="1.0",
                                created_iso, generator_json
/dataset_json            bytes dataset — free-form dataset metadata (profile,
                         plate, well, wells_um, operator, machine, notes)
/images/<id>/data        HDF5 dataset (chunked, gzip-4, Fletcher32)
                         attrs: axes, pixel_format
/images/<id>/meta_json   bytes dataset — per-image metadata document
/images/<id>/preview_png optional bytes dataset (pre-encoded PNG)
/attachments/<name>      bytes datasets, attr media_type
```

- Axes = subsequence of `"TCZYXS"`; Y and X required; `S` (interleaved samples)
  trailing only, size 2–4. `pixel_format` **required** (`gray`, `gray16`, `RGB`,
  `RGBA`, `BGR`, `BGRA`, …) — the field is the contract; MEBP's export policy is
  convert-to-RGB.
- Image ids `[A-Za-z0-9_.-]+`, no `.`/`..`, case-insensitively unique —
  **refused if invalid, never sanitized** (the v7.12 many-to-one-sanitizer
  lesson: two names differing only in punctuation must never collide silently).
- Versioning: readers accept same-major/any-minor and ignore unknown keys;
  refuse higher major (`ND3VersionError`).
- Atomic writes: `path + ".tmp"` then `os.replace`; an exception inside the
  writer context manager aborts — the final path is never touched.

### Load-bearing frame rules (each pinned by a test)

1. **Back-projection rule:** trusted pixel origin = `extent_um[0:2] − shift_um`.
   `shift_known: false` = legacy/unknown (NOT zero) — mirrors
   `FluorescenceMosaicStore.has_shift()`.
2. **⚠ Scale trap:** a mosaic composite's pixel pitch is **`1/mosaic_scale`**
   (px/µm), NOT `um_per_px` (the camera pitch at capture). Verified in
   `MosaicBuilder.tile_rects_px` (canvas row 0 = min_y, +row = +stage-Y, no Y
   flip). Tests seed the two values differently so a mix-up fails.
3. **Transforms** computed in ND3Export, stored explicitly, authoritative over
   scalar fields; `pixel_to_plate_mm` must agree with
   `MosaicStore.stage_to_plate_mm` (tied by test).
4. **Never fabricate:** legacy scan (plate_frame None) → stage-frame-only file +
   `needs_plate_frame: true`; capture with rotation/flips →
   `pixels_stage_aligned: false` and **no transforms**.

## Implementation Steps

- [x] Install h5py; add `h5py>=3.12` to `requirements.txt`
- [x] This plan document
- [x] `docs/ND3_SPEC.md` (schema-first; tests written against the spec)
- [x] `SupportClasses/ND3.py` — errors, validation, `ND3Writer` (+`begin_stack`
      appender for time-lapse), `sniff`/`open_nd3`/`ND3Reader`/`ND3Image`
      (+`section()` partial reads), `verify(deep=)`
- [x] `tests/test_v716_nd3_container.py` green (53)
- [x] `SupportClasses/ND3Export.py` — `export_fluorescence_well`,
      `export_plate_mosaic`, `export_capture`, `export_time_lapse`
- [x] **`export_lablink_job`** — mid-flight operator request: bridge one .nd3
      image to LabLink's `lablink.imagejob/1` TIFF + `.job.json` pair
- [x] `tests/test_v716_nd3_export.py` green (37, incl. 9 LabLink-bridge)
- [x] Mutation-verify key guards (6/6 caught, sources restored byte-identically)
- [x] Regression: fluorescence-mosaic / plate-mosaic / capture suites +
      `gui.app` import smoke
- [x] End-to-end: real fluorescence well + real plate mosaic from this
      machine's stores; re-opened with raw h5py (consumer simulation)
- [x] Spec §14 LabLink annex + CLAUDE.md table row

## Testing Notes

- `python -m unittest tests.test_v716_nd3_container` — **53 green**: round-trips
  for every whitelisted dtype (incl. uint16 with values > 255 — the exact
  reason CaptureImageWriter refuses 16-bit PNG), CYX/TCZYX/YXS, the stack
  appender (append + auto-finish on close + shape/dtype refusals), the refusal
  matrix (each guard = one named test), version gate ("1.9" opens / "2.0"
  refused / garbage = format error), corrupted-payload deep-verify naming the
  entry while shallow verify proves deep does the reading, laziness (metadata
  reads never touch pixels; a closed reader gives a clear error), atomicity
  (exception in `with` leaves NO file and no tmp), purity (AST walk: stdlib +
  numpy + h5py only, no repo imports) + a vendoring subprocess test (ND3.py
  copied ALONE to a tmp dir, imported, round-tripped), and a third-party
  consumer simulation reading pixels + metadata with raw h5py + json only.
- `python -m unittest tests.test_v716_nd3_export` — **37 green**: BGR→RGB
  asserted at the PIXEL level, gray collapse, mismatched channel shapes
  exported verbatim (no resize), the pitch trap (mosaic_scale and um_per_px
  seeded DIFFERENT so a mix-up fails), origin = extent − shift, legacy
  no-shift → `shift_known: false` with the matrix on the display extent,
  channel entry fields incl. display_lo/hi absent-stays-absent, Bright Field
  id mapping keeping the true name, invalid channel name refused, plate-frame
  matrices agreeing with `MosaicStore.stage_to_plate_mm` through a mapped-well
  round-trip, legacy plate honesty, capture gray16 + geometry-in-plane +
  rotated-view-gets-no-transforms, time-lapse directory → TYX stack with
  per-frame `t_iso`/`t_s`, and the LabLink bridge (sidecar authority fields,
  sensor-vs-container bit-depth refusal, wavelength honesty, upload-name rule,
  null-knob preservation, fluor pixel_size = canvas pitch).
- **6/6 mutations confirmed CAUGHT** (each a real source edit, restored
  byte-identically — hashes compared against a pre-mutation backup): BGR→RGB
  swap removed → 2 pixel tests fail · pitch switched to camera `um_per_px` →
  matrix test fails · `shift_known` forced true → legacy test fails ·
  `needs_plate_frame` dropped → 2 honesty tests fail · id sanitized instead of
  refused → 2 refusal tests fail · version gate loosened → 2.0-refusal fails.
- Regression, run per-suite: `test_v75x_fluorescence_mosaic` **35 green** ·
  `test_v714_capture_core` **47 green** · `test_v75x_plate_mosaic` **110 ran,
  1 failure = the documented pre-existing `test_real_24_well_mosaic`** (23 vs
  24, legacy blob detector), with the documented `TestManualAlignPage` hang
  class excluded per this file's own precedent · `import gui.app` smoke green
  (h5py addition does not disturb startup).
- **End-to-end on this machine's real stores** (artifacts in `logs/nd3/`):
  `nest-plastic-24|C5` (FITC + Cy5) → 13.3 MB .nd3, deep verify OK, stage
  extents EXACT vs the store, `shift_known=False` correctly reported (real
  pre-v7.8 records) and `needs_plate_frame=True` (fluor store carries no plate
  frame); plate `24` → 2.8 MB .nd3, deep verify OK, extent EXACT,
  `plate_frame` honestly ABSENT (`store.plate_frame()` is None — pre-v7.13
  scan), and a mapped-well matrix round-trip closed. Both files re-read with
  RAW h5py + json (no ND3.py) — the Blender/LabLink read path.

## Issues & Decisions

- **HDF5 over ZIP+npy**: operator's call. The "stdlib-only reader" property is
  traded for chunked partial reads, native checksums, and extendable time-lapse
  datasets. `ND3.py` stays a single vendorable file; its only third-party deps
  are numpy + h5py.
- **No separate manifest**: HDF5 is self-indexing; shape/dtype live natively in
  the dataset and CANNOT diverge from the pixels. One authoritative home per
  fact: structural facts = HDF5 native (dataset shape/dtype, `axes`/
  `pixel_format` attrs); rich metadata = the JSON documents.
- **Fluorescence wells export one image entry per channel, NOT a CYX stack** —
  each channel carries its own extent/shift/scale/display-levels and grids can
  legitimately mismatch (`composite_overlay` resizes mismatches; the exporter
  ships originals verbatim). A stack would force a lossy resize and falsely
  imply the uint8 planes are radiometrically comparable across different frozen
  display levels.
- **`"Bright Field"` fails the id charset** → fixed bijective table
  `{"Bright Field": "Bright_Field"}` for the known channel set; the TRUE name is
  kept in the channel entry's `name`. Unknown channel names that fail the
  charset are REFUSED (never sanitized).
- **BGR dies at the export boundary**: `.nd3` color images are RGB; the
  `pixel_format` field stays mandatory so a future non-converting writer can
  honestly write `"BGR"` and readers must check rather than assume.
- **Time-lapse source layout**: `RawFrameSequenceWriter` writes
  `frame_000001.tif/png` + per-frame embedded meta + `.json` sidecars + a
  `sequence.json` summary; `export_time_lapse` consumes either such a directory
  or an in-memory iterable of `(frame, plane_dict)`.
- **Deferred (recorded hook points)**: GUI capture format option at
  `gui/widgets/capture_controller.py::_do_still` (the single choke point through
  `CaptureImageWriter.write_image`); "Export .nd3" button on the Fluorescence
  Mosaic page; per-`scan_id` plate mosaic export (v1 = ACTIVE scan);
  live recording straight to .nd3 (the `begin_stack` appender is the ready hook).
- **Timestamps in HDF5 attrs**: `created_iso` is written by the writer;
  reproducible-output mode was NOT pursued (HDF5 embeds internal heap state;
  byte-reproducibility is not a goal of v1 — checksums + verify are).
- **⭐ LabLink bridge (operator request, mid-flight)**: LabLink's contract
  (`lablink/docs/IMAGE-JOB-FORMAT.md`, read from the private repo via cached
  git credentials) is TIFF/ND2 + a `.job.json` sidecar
  (`"lablink.imagejob/1"`) where **the sidecar OVERRIDES the image file** — a
  TIFF *invents* the container bit depth and a placeholder channel name.
  `export_lablink_job(nd3, image_id, out_dir)` extracts one .nd3 image into
  that pair. Three honesty rules carried over: the sensor bit depth is parsed
  from `acquisition.bit_depth` or passed explicitly, and the array dtype is
  **never** used as a fallback (it IS the invented container value the sidecar
  exists to override); emission/excitation wavelengths are NOT recorded
  anywhere in MEBP, so they are caller-supplied or absent — letting LabLink's
  `missing_metadata` refusal name the fields (its designed remediation loop)
  beats fabricating nominal fluorophore values; a stem violating LabLink's
  upload-name rule (`^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$`) is refused with a
  `stem=` escape hatch, the true name surviving in `source.original_name`.
  T/C/Z stacks are refused (one image per LabLink job). Spec §14 records the
  full field mapping; ND3 channel entries gained optional
  `emission_nm`/`excitation_nm` so a future acquisition path that knows its
  filter wavelengths has a home for them.
- 🐞 **Found while testing: an `ND3Image` from a CLOSED reader** hit h5py's
  cryptic `KeyError: 'Unable to synchronously open object (invalid identifier
  type to function)'` — the lazy handle is tied to the open file. `array()`/
  `section()`/`preview_png()` now check handle liveness first and raise a
  clear "the ND3Reader that produced this handle is closed" error (pinned by
  test).
