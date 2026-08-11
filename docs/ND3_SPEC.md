# ND3 Container Format — Specification

**Version 1.0 (schema_version "1.0")** · Status: current · Owner: MEBP

`.nd3` is a multidimensional microscopy image container built on HDF5. It stores
pixel data together with everything needed to reconstruct the images *in physical
space* somewhere else: pixel scale, absolute stage coordinates (µm), plate-local
coordinates (mm), orientation, channel identities and pseudo-colors, per-plane
acquisition records, and provenance. It is the interchange format for the MEBP
ecosystem (MEBP itself, Blender scene builders, LabLink, nd3 tooling).

This document is the normative spec. A third party should be able to implement a
complete reader (or a conforming writer) from this file alone.

- **RFC 2119 language**: MUST / SHOULD / MAY are used in their usual sense.
- Reference implementation: `SupportClasses/ND3.py` in the MEBP repository — a
  single self-contained module whose only dependencies are the Python standard
  library, `numpy`, and `h5py`. It may be vendored verbatim.

---

## 1. Purpose and design goals

- **Self-describing**: a `.nd3` file carries its own georeferencing and channel
  semantics; no side-channel files, databases, or MEBP installation required.
- **Random access**: metadata is readable without touching pixel data; pixel
  data supports partial (tiled / per-plane) reads.
- **Integrity**: per-dataset checksums; a verification procedure that names the
  corrupt entry.
- **Honesty over convenience**: unknown values are *absent*, never fabricated
  or defaulted (see §8 — several fields exist specifically to distinguish
  "recorded as zero" from "never recorded").
- **Explicit over derived**: pixel→world transforms are stored as matrices, not
  left for each consumer to re-derive from scalars (a historical source of
  sign/frame bugs).

Non-goals (v1): byte-reproducible output, sparse/pyramidal storage, encryption.

## 2. Container

An `.nd3` file **is a valid HDF5 file** (any HDF5 ≥ 1.10 library reads it).

- Byte signature: the standard HDF5 magic `\x89HDF\r\n\x1a\n` at offset 0.
- **Identification**: the HDF5 root group MUST carry the attribute
  `format = "nd3"` (string). A file without it is not ND3.
- Recommended extension: `.nd3`.

**Sniffing recipe** (no h5py needed): first 8 bytes equal the HDF5 magic. Full
confirmation requires opening the file and checking the root `format` attr.

**Reading in any Python** (this is the whole reader for one image):

```python
import h5py, json, numpy as np
with h5py.File("scan.nd3", "r") as f:
    assert f.attrs["format"] == "nd3"
    pixels = f["images/DAPI/data"][()]                   # numpy array
    meta   = json.loads(bytes(f["images/DAPI/meta_json"][()]))
    M      = np.array(meta["transforms"]["pixel_to_stage_um"])
```

To install the dependency inside Blender's bundled Python:
`<blender>/python/bin/python -m pip install h5py`.

## 3. Tree layout

```
/                          root group
  @format                  "nd3"                       (MUST)
  @schema_version          "1.0"                       (MUST; "MAJOR.MINOR")
  @created_iso             ISO-8601 timestamp          (MUST)
  @generator_json          JSON string                 (SHOULD; see §3.1)
  dataset_json             bytes dataset               (MAY;  JSON, see §6)
  images/                  group                       (MUST, may be empty)
    <id>/                  one group per image
      data                 N-D dataset                 (MUST)
        @axes              e.g. "YX", "CYX", "TYXS"    (MUST)
        @pixel_format      e.g. "gray16", "RGB"        (MUST)
      meta_json            bytes dataset               (MUST; JSON, see §7)
      preview_png          bytes dataset               (MAY;  PNG-encoded)
  attachments/             group                       (MAY)
    <name>                 bytes dataset
      @media_type          e.g. "application/json"     (SHOULD)
```

"bytes dataset" = an HDF5 scalar dataset of an opaque byte string, or a 1-D
`uint8` dataset; readers MUST accept both and treat the content as raw bytes.
Writers SHOULD use a scalar opaque/bytes dataset.

### 3.1 `generator_json`

JSON object identifying the writer, e.g.
`{"software": "MEBP", "software_version": "7.16", "writer": "ND3.py/1.0"}`.
Informative only.

## 4. Identifiers

Image ids and attachment names MUST match `[A-Za-z0-9_.-]+`, MUST NOT be `"."`
or `".."`, and MUST be unique **case-insensitively** within their group (files
get extracted onto case-insensitive filesystems). Writers MUST refuse invalid
ids — never silently rename (two names that sanitize to the same string must
never collide silently).

## 5. Pixel data

### 5.1 Dataset

`images/<id>/data` is an HDF5 dataset holding the pixel array.

- **dtype whitelist**: `bool`, `uint8/16/32/64`, `int8/16/32/64`, `float16/32/64`.
  Object, structured, and complex dtypes are forbidden.
- Writers SHOULD store little-endian; readers MUST honor the dtype HDF5 reports.
- Writers SHOULD chunk (leading non-spatial dims at 1 per plane; Y/X tiled, e.g.
  ≤1024 px) and SHOULD apply gzip and the Fletcher32 checksum filter. Readers
  MUST accept contiguous/unfiltered data too.
- A time-lapse writer MAY create the dataset extendable (`maxshape=None` on the
  leading axis) and append frames.

### 5.2 `axes` (attribute, string)

The dimension semantics of `data`, one character per dimension, in order:

- Grammar: a **subsequence of `"TCZYXS"`** (that exact relative order), with:
  - `Y` and `X` MUST both be present (row, column).
  - `S` — interleaved samples (e.g. RGB) — MAY appear only as the **final**
    axis, with size 2–4.
  - `T` = time points, `C` = channels, `Z` = focal planes.
- `len(axes) == data.ndim`; every dimension size ≥ 1.

Examples: `"YX"` (mono image) · `"YXS"` (interleaved RGB) · `"CYX"` (channel
stack) · `"TYX"` (time-lapse) · `"TCZYX"` (full 5-D).

### 5.3 `pixel_format` (attribute, string; REQUIRED)

Declares how to interpret samples. Registry (v1):

| value | meaning | constraint |
|---|---|---|
| `gray` | single-sample intensity, 8-bit typical | no `S` axis |
| `gray16` | single-sample intensity, 16-bit | no `S` axis; dtype uint16 |
| `RGB` / `BGR` | interleaved 3-sample color | `S` axis size 3 |
| `RGBA` / `BGRA` | interleaved 4-sample color | `S` axis size 4 |

Other values MAY be used (matching `[A-Za-z0-9_]+`); readers MUST NOT guess an
unknown format's sample order. **The field is the contract** — MEBP's own
exporters always convert to `RGB`/`gray`/`gray16`, but a reader MUST check
rather than assume.

## 6. `dataset_json` — dataset-level metadata

A JSON object with free-form keys. Reserved keys used by MEBP profiles:

| key | type | meaning |
|---|---|---|
| `profile` | string | e.g. `"mebp.fluor_well/1"`, `"mebp.plate_mosaic/1"`, `"mebp.capture/1"`, `"mebp.time_lapse/1"` |
| `plate_id` | string | plate identity / store key |
| `well` | string | well name (e.g. `"B3"`) |
| `objective` | string | objective description |
| `wells_um` | object | `{well_name: [x_um, y_um]}` mapped well centres, absolute stage µm |
| `operator`, `machine`, `notes` | string | provenance |

Readers MUST ignore unknown keys.

## 7. Per-image `meta_json`

A JSON object. All top-level blocks are OPTIONAL unless marked; readers MUST
ignore unknown keys everywhere. `null` is not used — an unknown value is an
**absent key**.

```jsonc
{
  // structural echo (informative; HDF5 dataset is authoritative for shape/dtype)
  "id": "DAPI", "axes": "YX", "shape": [4096, 4096],
  "dtype": "uint8", "pixel_format": "gray",

  "scale": {
    "um_per_px": 0.17321,             // pitch of THIS image's pixels (µm/px)
    "captured_um_per_px": 0.65,       // camera pitch at capture, if the image
                                      // was downscaled (e.g. a mosaic canvas)
    "mosaic_scale_px_per_um": 5.7736  // mosaic canvas scale, if applicable
  },

  "stage_frame": {
    "extent_um": [minx, miny, maxx, maxy], // ABSOLUTE stage µm, display-registered
    "shift_um": [dx, dy],                  // registration shift baked into extent
    "shift_known": true                    // false = LEGACY record: shift UNKNOWN,
                                           // not zero (see §8.2)
  },

  "plate_frame": {                    // absent on legacy scans — see needs_plate_frame
    "plate_id": "plate-24",
    "well": "B3",
    "extent_mm": [x0, y0, x1, y1],    // A1-relative plate mm, normalised x0<=x1, y0<=y1
    "anchor_um": [ax, ay],            // taught A1 in absolute stage µm
    "axis_sign": [sx, sy]             // ±1 per axis (180°-mounted plates)
  },
  "needs_plate_frame": true,          // present+true ONLY when plate_frame is absent

  "orientation": {
    "rotation_deg": 0.0, "flip_x": false, "flip_y": false,
    "pixels_stage_aligned": true      // false ⇒ transforms are absent (see §8.4)
  },

  "channels": [                       // one entry per C plane; exactly one when no C axis
    { "name": "DAPI",                 // TRUE channel name (may contain spaces)
      "color_rgb": [0, 80, 255],      // display pseudo-color
      "channel_number": 1,            // microscope filter channel, if known
      "exposure_us": 20000.0,
      "avg_frames": 4,
      "display_lo": 210.0,            // frozen mono16→8 display levels;
      "display_hi": 1900.0,           // BOTH absent = unknown, not 0/65535
      "processing": {} }              // record of processing applied, if any
  ],

  "planes": [                         // one record per (T,C,Z) plane, C-order over
                                      // the non-YXS axes; length == product of
                                      // those dims (1 for a plain YX image)
    { "t": 0, "c": 0, "z": 0,
      "stage_x_um": 0.0, "stage_y_um": 0.0, "focus_um": 0.0,
      "exposure_us": 0.0, "gain_pct": 0.0, "t_iso": "2026-08-08T10:00:00" }
  ],

  "transforms": {                     // stored, AUTHORITATIVE (§8.4)
    "pixel_to_stage_um": [[p,0,ox],[0,p,oy],[0,0,1]],
    "pixel_to_plate_mm": [[...],[...],[0,0,1]]   // only when plate_frame present
  },

  "acquisition": {                    // camera/optics record (MEBP CaptureMeta
                                      // vocabulary; empty fields omitted):
                                      // camera_identity, camera_name, camera_slot,
                                      // exposure_us, gain_pct, bit_depth, gain_mode,
                                      // readout_rate, source_mode, objective,
                                      // objective_label, magnification,
                                      // numerical_aperture, working_distance_mm,
                                      // objective_position, channel, filter_position
  },

  "provenance": {
    "software": "MEBP", "software_version": "7.16",
    "source": "FluorescenceMosaicStore",   // or MosaicStore | capture | time_lapse
    "machine": "", "operator": "",
    "created_iso": "...", "source_date": "2026-08-08"
  }
}
```

## 8. Coordinate frames (load-bearing)

### 8.1 Pixel frame

Continuous pixel coordinates `(px_x, px_y)` where `px_x` = column, `px_y` =
row. `(0.0, 0.0)` is the **outer corner** of pixel `[0, 0]`; the center of
array element `[i, j]` is `(j + 0.5, i + 0.5)`. Row index increases with
+stage-Y (no Y flip in ND3 files written by MEBP — the mosaic canvas is built
that way).

### 8.2 Stage frame and the back-projection rule

`stage_frame.extent_um` is the image's bounding box in **absolute stage µm**,
*display-registered*: a registration shift `shift_um` may be baked into it
while the pixels themselves sit at raw stage positions. Therefore:

> **The trusted pixel origin is `extent_um[0:2] − shift_um`.**
> Any consumer converting pixels to stage coordinates MUST subtract the shift
> (or, better, use the stored `pixel_to_stage_um` matrix, which already has).

`shift_known: false` means the source record predates shift tracking — the
shift is **unknown**, not zero. Such an image is fine for display and for
measurements that are translation-invariant (diameters, areas); it MUST NOT be
trusted for absolute positioning (the error bound is up to ~½ field of view).

### 8.3 Plate frame

`plate_frame` locates the image on the *plate* independent of how the plate
sits on the stage: `extent_mm` is A1-relative plate-local mm, related to stage
µm by the taught A1 anchor and per-axis sign:

```
plate_mm = axis_sign * (stage_um − anchor_um) / 1000        (per axis)
```

`axis_sign` is ±1 per axis (−1,−1 on plates mounted 180° to the stage).
Extents are normalised so `x0 ≤ x1`, `y0 ≤ y1` after sign application.

A file whose source predates plate-frame tracking carries
`needs_plate_frame: true` and NO `plate_frame` — the writer does not guess.

### 8.4 Explicit transforms (authoritative)

When present, `transforms` matrices are **authoritative**; scalar fields
(`scale`, extents) are informative. Both are 3×3 row-major homogeneous affines
over `[px_x, px_y, 1]ᵀ`:

```
pixel_to_stage_um = [[p, 0, ox],
                     [0, p, oy],
                     [0, 0, 1 ]]
  p        = this image's pixel pitch in µm/px
  (ox, oy) = extent_um[0:2] − shift_um       (the trusted origin, §8.2)

pixel_to_plate_mm = [[sx·p/1000, 0,          sx·(ox−ax)/1000],
                     [0,         sy·p/1000,  sy·(oy−ay)/1000],
                     [0,         0,          1              ]]
  (ax, ay) = plate_frame.anchor_um,  (sx, sy) = plate_frame.axis_sign
```

Transforms are emitted **only when honest**: no `pixel_to_plate_mm` without a
plate frame; no transforms at all when `pixels_stage_aligned` is false (the
pixels carry an uncorrected rotation/flip relative to stage axes).

### 8.5 Scale — the mosaic pitfall

`scale.um_per_px` is always the pitch of **this image's** pixels. For a mosaic
composite that is `1 / mosaic_scale_px_per_um`, NOT the camera's µm/px — the
canvas is downscaled from camera resolution. `captured_um_per_px` preserves
the camera pitch for reference. Consumers MUST use `um_per_px` (or better, the
matrices) for geometry.

### 8.6 Worked example

From a real exported plate mosaic (values rounded for reading):
`mosaic_scale_px_per_um = 0.02` (canvas px per µm) ⇒ image pitch
`p = 1/0.02 = 50 µm/px`. `extent_um = [10000, 20000, 110000, 76000]`,
`shift_um = [150, −40]` ⇒ trusted origin `(ox, oy) = (9850, 20040)`.

Pixel `(px_x=2000, px_y=1000)` (pixel corners; centre of array element
`[1000, 2000]` would use `2000.5, 1000.5`):

```
stage = p·px + origin = (50·2000 + 9850, 50·1000 + 20040)
      = (109850, 70040) µm
```

With `anchor_um = (105618, 65890)` (taught A1) and `axis_sign = (−1, −1)`:

```
plate_mm = (−1·(109850−105618)/1000, −1·(70040−65890)/1000)
         = (−4.232, −4.150) mm            (A1-relative)
```

## 9. Previews and attachments

`preview_png` is a small, **non-quantitative** PNG for thumbnails: it may be
downscaled, autoscaled (min–max) and tone-mapped. Never measure from it.

Attachments are opaque byte payloads (e.g. a focus survey JSON, a log). Their
`media_type` attribute is advisory.

## 10. Versioning and forward compatibility

`schema_version` is `"MAJOR.MINOR"`.

- Readers MUST refuse a file whose MAJOR is greater than they support.
- Readers MUST accept any MINOR within a supported MAJOR and MUST ignore
  unknown keys/attributes/datasets.
- Writers MUST NOT change the meaning of an existing field within a MAJOR;
  additive changes bump MINOR.

## 11. Integrity and verification

- Writers SHOULD enable the Fletcher32 filter on pixel datasets (checksum
  validated by HDF5 on every read).
- The verification procedure (reference: `ND3.verify(path)`):
  1. root attrs present and well-formed; version acceptable;
  2. every `images/<id>` group has `data` (+ `axes`, `pixel_format` attrs
     satisfying §5) and a parseable `meta_json`;
  3. deep mode: fully read every dataset (drives checksum validation) and
     parse every JSON document.
  Result is a list of problem strings naming the offending entry; empty = OK.
- Writers SHOULD write to a temporary sibling and atomically replace the final
  path, so a `.nd3` never exists half-written.

## 12. Conformance checklists

**Minimal conforming reader** — MUST: check `format`/`schema_version`; honor
`axes`/`pixel_format` rather than assuming; use `extent − shift` (or the stored
matrices) for positioning; treat absent keys as unknown; ignore unknown keys.

**Minimal conforming writer** — MUST: valid ids (§4); `axes` grammar (§5.2);
`pixel_format` always set; whitelisted dtypes; `meta_json` present per image;
version attrs; never fabricate frames it does not know (§8.2–8.4 honesty
rules).

## 13. MEBP profiles (informative annex)

| profile | contents |
|---|---|
| `mebp.fluor_well/1` | One image per fluorescence channel of one well (ids = channel names, `"Bright Field"` → id `Bright_Field`, true name in the channel entry). Per-channel stage extents/scale/display levels; optional `focus_survey.json` attachment. |
| `mebp.plate_mosaic/1` | Single image `mosaic` = the full-plate brightfield composite; `plate_frame` when the scan is plate-referenced; `wells_um` mapping in `dataset_json`. |
| `mebp.capture/1` | Single still `capture` with the full CaptureMeta acquisition record and a single `planes[0]` geometry record. |
| `mebp.time_lapse/1` | One `TYX`/`TYXS` stack; one `planes` record per frame with `t_iso` timestamps. |

## 14. LabLink interchange (informative annex)

LabLink's analysis hub consumes **TIFF/ND2 + a `.job.json` sidecar**
(`lablink.imagejob/1` — see `lablink/docs/IMAGE-JOB-FORMAT.md`, the source of
truth for that side). `SupportClasses/ND3Export.export_lablink_job()` extracts
one `.nd3` image into that pair. The field mapping:

| lablink `image` field | from ND3 | notes |
|---|---|---|
| `pixel_size_um` | `meta.scale.um_per_px` | **This image's** pitch — for a mosaic that is `1/mosaic_scale`, already resolved by the exporters (§8.5). Refused if absent. |
| `bit_depth` | `meta.acquisition.bit_depth` (leading integer) | The SENSOR's depth. The array dtype is the container depth — LabLink's named trap — so it is never used as a fallback; unknown ⇒ refused unless the caller supplies it. |
| `objective_magnification` | parsed from `acquisition.magnification` / `acquisition.objective` (`"10x…"`) | omitted when unparseable |
| `objective_na` | `acquisition.numerical_aperture` | omitted when absent |
| `channels[].name` | ND3 channel entry `name` (true name, e.g. `"Bright Field"`) | required by LabLink; refused when ND3 has none |
| `channels[].emission_nm` / `excitation_nm` | **not recorded by MEBP** | caller-supplied (`wavelengths=`); absent otherwise — LabLink's `missing_metadata` refusal names the fields, which is the designed remediation loop. Never fabricated from nominal fluorophore tables. |
| `source.original_name` | the `.nd3` filename | LabLink upload names are restricted (`^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$`); a non-compliant stem is refused with a `stem=` escape hatch |
| `source.acquired` | `planes[0].t_iso`, else `provenance.created_iso` | |

Stacks (`T`/`C`/`Z` axes) are refused — LabLink jobs carry one image per pair;
export `.nd3` per-channel images individually. ND3 channel entries MAY carry
`emission_nm` / `excitation_nm` (numbers, nm) so a future acquisition path
that knows its filter wavelengths has a home for them; the LabLink bridge
passes them through when present.
