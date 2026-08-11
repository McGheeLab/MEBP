"""ND3Export.py — build .nd3 containers from the MEBP stores.

v7.16. The store-facing half of the .nd3 work: `SupportClasses/ND3.py` is the
pure format module (stdlib + numpy + h5py, vendorable into Blender/LabLink);
THIS module is where MEBP's own vocabularies are translated into the spec's
(docs/ND3_SPEC.md) — and where cv2's BGR convention dies. Exporters:

* :func:`export_fluorescence_well` — one image per stored channel of one well
  (profile ``mebp.fluor_well/1``).
* :func:`export_plate_mosaic` — the ACTIVE plate scan + plate frame + mapped
  well centres (profile ``mebp.plate_mosaic/1``).
* :func:`export_capture` — a single still + its CaptureMeta record (profile
  ``mebp.capture/1``).
* :func:`export_time_lapse` — a RawFrameSequenceWriter directory (or an
  in-memory frame iterable) as one T-stack (profile ``mebp.time_lapse/1``).

Frame rules carried over verbatim from the stores (each pinned by a test):

* **Back-projection**: the trusted pixel origin is ``extent_um[:2] − shift_um``
  and ``shift_known=False`` means the shift is UNKNOWN (legacy record), not
  zero — mirrors ``FluorescenceMosaicStore.has_shift``.
* **⚠ The pitch trap**: a mosaic composite's pixel pitch is ``1/mosaic_scale``
  (the canvas is downscaled), NOT ``um_per_px`` (the camera pitch at capture).
  Both are stored; the transforms use the image's own pitch.
* **Never fabricate**: no plate frame ⇒ ``needs_plate_frame: true`` and no
  plate matrix; a capture whose pixels carry an uncorrected rotation/flip gets
  ``pixels_stage_aligned: false`` and NO transforms.

Zero GUI dependencies (numpy + cv2 + json only).
"""

from __future__ import annotations

import json
import logging
import platform
import re
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import cv2
    _CV2 = True
except ImportError:  # pragma: no cover
    cv2 = None
    _CV2 = False

from SupportClasses import ND3
from SupportClasses.ND3 import ND3Error, ND3Writer

logger = logging.getLogger(__name__)

SOFTWARE = "MEBP"
SOFTWARE_VERSION = "7.16"

# The known channel set whose display names are not valid ND3 image ids.
# Bijective on purpose; anything NOT in this table must already satisfy the id
# charset or the export is REFUSED (never sanitized — the v7.12 lesson).
CHANNEL_IMAGE_IDS = {"Bright Field": "Bright_Field"}

_PREVIEW_MAX_PX = 1024


class ND3ExportError(ND3Error):
    """An export refused — the message names exactly what was missing."""


# --------------------------------------------------------------------------
# shared helpers
# --------------------------------------------------------------------------

def _channel_image_id(channel_name: str) -> str:
    image_id = CHANNEL_IMAGE_IDS.get(channel_name, channel_name)
    try:
        ND3._validate_id(image_id)
    except ND3Error:
        raise ND3ExportError(
            f"channel name {channel_name!r} is not a valid .nd3 image id and "
            f"has no entry in CHANNEL_IMAGE_IDS — refused (never sanitized)")
    return image_id


def _bgr_to_stored(img, *, collapse_gray: bool):
    """(array, axes, pixel_format) — BGR dies here; .nd3 color is RGB."""
    arr = np.asarray(img)
    if arr.ndim == 2:
        fmt = "gray16" if arr.dtype == np.uint16 else "gray"
        return arr, "YX", fmt
    if arr.ndim == 3 and arr.shape[2] == 3:
        if collapse_gray and np.array_equal(arr[..., 0], arr[..., 1]) \
                and np.array_equal(arr[..., 1], arr[..., 2]):
            plane = arr[..., 0]
            fmt = "gray16" if plane.dtype == np.uint16 else "gray"
            return np.ascontiguousarray(plane), "YX", fmt
        return np.ascontiguousarray(arr[..., ::-1]), "YXS", "RGB"
    raise ND3ExportError(
        f"unsupported image shape {arr.shape} (expected HxW or HxWx3)")


def _encode_preview_png(img_bgr_or_gray, *, max_px: int = _PREVIEW_MAX_PX
                        ) -> Optional[bytes]:
    """Small NON-QUANTITATIVE preview (spec §9): may downscale and min-max
    autoscale 16-bit input.  Takes cv2-native (BGR/gray) input."""
    if not _CV2:
        return None
    a = np.asarray(img_bgr_or_gray)
    if a.dtype == np.uint16:
        lo, hi = int(a.min()), int(a.max())
        a = ((a.astype(np.float32) - lo) * (255.0 / max(1, hi - lo))
             ).astype(np.uint8)
    elif a.dtype != np.uint8:
        return None
    h, w = a.shape[:2]
    longest = max(h, w)
    if longest > max_px:
        f = max_px / float(longest)
        a = cv2.resize(a, (max(1, int(w * f)), max(1, int(h * f))),
                       interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".png", a)
    return bytes(buf) if ok else None


def _pixel_to_stage_um(pitch: float, origin) -> list:
    ox, oy = float(origin[0]), float(origin[1])
    p = float(pitch)
    return [[p, 0.0, ox], [0.0, p, oy], [0.0, 0.0, 1.0]]


def _pixel_to_plate_mm(pitch: float, origin, anchor_um, axis_sign) -> list:
    p = float(pitch)
    ox, oy = float(origin[0]), float(origin[1])
    ax, ay = float(anchor_um[0]), float(anchor_um[1])
    sx, sy = float(axis_sign[0]), float(axis_sign[1])
    return [[sx * p / 1000.0, 0.0, sx * (ox - ax) / 1000.0],
            [0.0, sy * p / 1000.0, sy * (oy - ay) / 1000.0],
            [0.0, 0.0, 1.0]]


def _provenance(source: str, *, operator: str = "", source_date: str = ""
                ) -> dict:
    prov = {
        "software": SOFTWARE,
        "software_version": SOFTWARE_VERSION,
        "source": source,
        "created_iso": datetime.now().astimezone().isoformat(),
    }
    machine = platform.node()
    if machine:
        prov["machine"] = machine
    if operator:
        prov["operator"] = operator
    if source_date:
        prov["source_date"] = source_date
    return prov


def _drop_empty(d: dict) -> dict:
    return {k: v for k, v in d.items() if v not in (None, "", [], {})}


def _stage_block(extent_um, shift_um, shift_known: bool) -> dict:
    return {
        "extent_um": [float(v) for v in extent_um],
        "shift_um": [float(shift_um[0]), float(shift_um[1])],
        "shift_known": bool(shift_known),
    }


def _frames_meta(extent_um, shift_um, shift_known, pitch,
                 plate_frame: Optional[dict], plate_id: str = "",
                 well: str = "") -> dict:
    """stage_frame + plate_frame + transforms, honestly (rules 1/3/4)."""
    meta: dict = {}
    transforms: dict = {}
    if extent_um is not None:
        meta["stage_frame"] = _stage_block(extent_um, shift_um, shift_known)
        if pitch:
            # Trusted origin per the back-projection rule.  When the shift is
            # UNKNOWN (legacy) the matrix uses the display extent verbatim and
            # shift_known carries the accuracy caveat (spec §8.2).
            ox = float(extent_um[0]) - float(shift_um[0])
            oy = float(extent_um[1]) - float(shift_um[1])
            transforms["pixel_to_stage_um"] = _pixel_to_stage_um(pitch, (ox, oy))
            if plate_frame:
                transforms["pixel_to_plate_mm"] = _pixel_to_plate_mm(
                    pitch, (ox, oy), plate_frame["anchor_um"],
                    plate_frame.get("axis_sign", (1.0, 1.0)))
    if plate_frame:
        pf = {"extent_mm": [float(v) for v in plate_frame["extent_mm"]],
              "anchor_um": [float(v) for v in plate_frame["anchor_um"]],
              "axis_sign": [float(v) for v in
                            plate_frame.get("axis_sign", (1.0, 1.0))]}
        if plate_id:
            pf["plate_id"] = str(plate_id)
        if well:
            pf["well"] = str(well)
        meta["plate_frame"] = pf
    else:
        meta["needs_plate_frame"] = True
    if transforms:
        meta["transforms"] = transforms
    return meta


# --------------------------------------------------------------------------
# fluorescence well
# --------------------------------------------------------------------------

def export_fluorescence_well(store, plate_key, well_name, path, *,
                             channels: Optional[list] = None,
                             include_processed: bool = False,
                             collapse_gray: bool = True,
                             plate_frame: Optional[dict] = None,
                             operator: str = "", notes: str = "") -> Path:
    """One .nd3 with one image entry PER CHANNEL (not a CYX stack — each
    channel carries its own extent/shift/scale/display-levels, and grids can
    legitimately mismatch; originals ship verbatim, no resize).

    ``plate_frame`` may be passed (e.g. ``MosaicStore().plate_frame(key)``) to
    georeference the well on the plate; without it the file is stage-frame
    only and says so (``needs_plate_frame``).
    """
    from SupportClasses.FluorescenceMosaicStore import (
        channel_number, default_color)

    if not store.has(plate_key, well_name):
        raise ND3ExportError(
            f"no fluorescence scan stored for {plate_key!r} / {well_name!r}")
    requested = channels is not None
    channel_names = list(channels) if requested \
        else store.list_channels(plate_key, well_name)
    if not channel_names:
        raise ND3ExportError(
            f"{plate_key!r} / {well_name!r} has no stored channels")

    well_rec = store.get_well(plate_key, well_name) or {}
    warnings: list = []

    # Load pass first so the dataset metadata (incl. any skip warnings) is
    # final before the writer opens — no post-hoc mutation of writer state.
    to_write: list = []
    for name in channel_names:
        image_id = _channel_image_id(name)
        img_bgr = store.load_channel_image(plate_key, well_name, name)
        if img_bgr is None:
            if requested:
                raise ND3ExportError(
                    f"channel {name!r} of {plate_key!r}/{well_name!r} "
                    "has no readable image")
            warnings.append(f"channel {name!r} skipped: image unreadable")
            continue
        to_write.append((name, image_id, img_bgr, False))
        if include_processed:
            proc = store.load_channel_image(plate_key, well_name, name,
                                            prefer_processed=True)
            if proc is not None and not np.array_equal(proc, img_bgr):
                to_write.append((name, image_id + ".proc", proc, True))
    if not to_write:
        raise ND3ExportError(
            f"{plate_key!r} / {well_name!r}: no channel image was readable")

    dataset_meta = _drop_empty({
        "profile": "mebp.fluor_well/1",
        "plate_id": str(plate_key),
        "well": str(well_name),
        "objective": store.get_objective(plate_key, well_name),
        "operator": operator,
        "notes": notes,
        "warnings": warnings,
    })

    path = Path(path)
    with ND3Writer(path, dataset_meta=dataset_meta,
                   generator={"software": SOFTWARE,
                              "software_version": SOFTWARE_VERSION}) as w:
        for name, image_id, img_bgr, processed in to_write:
            _write_fluor_channel(
                w, store, plate_key, well_name, name, image_id, img_bgr,
                collapse_gray=collapse_gray, plate_frame=plate_frame,
                processed=processed, channel_number_fn=channel_number,
                default_color_fn=default_color,
                source_date=well_rec.get("date", ""), operator=operator)
        survey = store.get_focus_survey(plate_key, well_name)
        if survey:
            w.add_attachment("focus_survey.json",
                             json.dumps(survey, sort_keys=True).encode("utf-8"),
                             media_type="application/json")
    return path


def _write_fluor_channel(w, store, plate_key, well_name, name, image_id,
                         img_bgr, *, collapse_gray, plate_frame, processed,
                         channel_number_fn, default_color_fn, source_date,
                         operator):
    arr, axes, fmt = _bgr_to_stored(img_bgr, collapse_gray=collapse_gray)

    extent = store.get_extent_um(plate_key, well_name, name)
    shift = store.get_shift_um(plate_key, well_name, name)
    shift_known = bool(store.has_shift(plate_key, well_name, name))
    mosaic_scale = store.get_mosaic_scale(plate_key, well_name, name)
    captured = store.get_um_per_px(plate_key, well_name, name)
    # ⚠ The pitch trap: the composite's pixels are 1/mosaic_scale µm apart —
    # um_per_px is the CAMERA pitch and would be wrong here (spec §8.5).
    pitch = (1.0 / float(mosaic_scale)) if mosaic_scale else None

    scale = _drop_empty({
        "um_per_px": pitch,
        "captured_um_per_px": float(captured) if captured else None,
        "mosaic_scale_px_per_um": float(mosaic_scale) if mosaic_scale else None,
    })

    channel_entry: dict = {"name": name}
    color = store.channel_color(plate_key, well_name, name) \
        or default_color_fn(name)
    if color:
        channel_entry["color_rgb"] = [int(c) for c in color]
    number = channel_number_fn(name)
    if number is not None:
        channel_entry["channel_number"] = int(number)
    exposure = store.get_exposure_us(plate_key, well_name, name)
    if exposure:
        channel_entry["exposure_us"] = float(exposure)
    avg = store.get_avg_frames(plate_key, well_name, name)
    if avg and avg > 1:
        channel_entry["avg_frames"] = int(avg)
    levels = store.get_display_levels(plate_key, well_name, name)
    if levels is not None:
        channel_entry["display_lo"] = float(levels[0])
        channel_entry["display_hi"] = float(levels[1])
    if processed:
        processing = store.get_processing(plate_key, well_name, name)
        if processing:
            channel_entry["processing"] = processing

    meta = {"scale": scale} if scale else {}
    meta.update(_frames_meta(extent, shift, shift_known, pitch, plate_frame,
                             plate_id=str(plate_key), well=str(well_name)))
    meta["orientation"] = {"pixels_stage_aligned": True}
    meta["provenance"] = _provenance("FluorescenceMosaicStore",
                                     operator=operator,
                                     source_date=str(source_date or ""))
    w.add_image(image_id, arr, axes=axes, pixel_format=fmt, meta=meta,
                channels=[channel_entry],
                preview_png=_encode_preview_png(img_bgr))


# --------------------------------------------------------------------------
# plate mosaic
# --------------------------------------------------------------------------

def export_plate_mosaic(store, plate_key, path, *,
                        operator: str = "", notes: str = "") -> Path:
    """The ACTIVE scan of ``plate_key`` as a single image ``mosaic`` with the
    plate frame (when the scan is plate-referenced) and the mapped well-centre
    table in ``dataset_json``."""
    meta_rec = store.get_meta(plate_key)
    if not meta_rec:
        raise ND3ExportError(f"no mosaic stored for plate {plate_key!r}")
    img_bgr = store.load_image(plate_key)
    if img_bgr is None:
        raise ND3ExportError(
            f"mosaic image for plate {plate_key!r} is missing/unreadable")

    extent = store.get_extent_um(plate_key)
    shift = store.get_shift_um(plate_key)
    shift_known = meta_rec.get("shift_um") is not None
    mosaic_scale = meta_rec.get("mosaic_scale") or 0.0
    pitch = (1.0 / float(mosaic_scale)) if mosaic_scale else None
    captured = meta_rec.get("um_per_px") or 0.0
    plate_frame = store.plate_frame(plate_key)

    dataset_meta = _drop_empty({
        "profile": "mebp.plate_mosaic/1",
        "plate_id": str(plate_key),
        "operator": operator,
        "notes": notes,
        "wells_um": {str(k): [float(v[0]), float(v[1])]
                     for k, v in (store.get_wells(plate_key) or {}).items()},
    })

    arr, axes, fmt = _bgr_to_stored(img_bgr, collapse_gray=True)
    scale = _drop_empty({
        "um_per_px": pitch,
        "captured_um_per_px": float(captured) if captured else None,
        "mosaic_scale_px_per_um": float(mosaic_scale) if mosaic_scale else None,
    })
    meta = {"scale": scale} if scale else {}
    meta.update(_frames_meta(extent, shift, shift_known, pitch, plate_frame,
                             plate_id=str(plate_key)))
    meta["orientation"] = {"pixels_stage_aligned": True}
    meta["provenance"] = _provenance("MosaicStore", operator=operator,
                                     source_date=str(meta_rec.get("date", "")))

    path = Path(path)
    with ND3Writer(path, dataset_meta=dataset_meta,
                   generator={"software": SOFTWARE,
                              "software_version": SOFTWARE_VERSION}) as w:
        w.add_image("mosaic", arr, axes=axes, pixel_format=fmt, meta=meta,
                    preview_png=_encode_preview_png(img_bgr))
    return path


# --------------------------------------------------------------------------
# single capture
# --------------------------------------------------------------------------

_GEOMETRY_KEYS = ("stage_x_um", "stage_y_um", "focus_um", "needle_z_mm")
_ORIENTATION_KEYS = ("view_rotation_deg", "view_flip_x", "view_flip_y")
_DATASET_KEYS = ("plate", "well", "operator", "notes")
_PROVENANCE_KEYS = ("software", "timestamp_iso", "warnings", "filename", "kind")


def export_capture(array, meta, path, *, image_id: str = "capture",
                   preview: bool = True) -> Path:
    """A single still + its CaptureMeta record (or an equivalent dict).

    Transforms are emitted ONLY when the pixels are stage-aligned (no view
    rotation/flip) and both µm/px and the stage position are known — a rotated
    view gets ``pixels_stage_aligned: false`` and no transforms (rule 4)."""
    d = meta.as_dict(drop_empty=True) if hasattr(meta, "as_dict") \
        else dict(meta or {})

    src = np.asarray(array)
    arr, axes, fmt = _bgr_to_stored(src, collapse_gray=False)

    rotation = d.get("view_rotation_deg")
    flips = bool(d.get("view_flip_x")) or bool(d.get("view_flip_y"))
    aligned = (rotation in (None, 0, 0.0)) and not flips
    orientation = _drop_empty({
        "rotation_deg": rotation,
        "flip_x": d.get("view_flip_x"),
        "flip_y": d.get("view_flip_y"),
    })
    orientation["pixels_stage_aligned"] = aligned

    um_per_px = d.get("um_per_px")
    sx_um, sy_um = d.get("stage_x_um"), d.get("stage_y_um")

    img_meta: dict = {}
    if um_per_px:
        img_meta["scale"] = {"um_per_px": float(um_per_px)}
    if aligned and um_per_px and sx_um is not None and sy_um is not None:
        h, w_px = arr.shape[0], arr.shape[1]
        half_w = float(w_px) * float(um_per_px) / 2.0
        half_h = float(h) * float(um_per_px) / 2.0
        extent = [float(sx_um) - half_w, float(sy_um) - half_h,
                  float(sx_um) + half_w, float(sy_um) + half_h]
        # A capture has no registration shift — genuinely zero, hence known.
        img_meta.update(_frames_meta(extent, (0.0, 0.0), True,
                                     float(um_per_px), None,
                                     well=str(d.get("well", ""))))
    else:
        img_meta["needs_plate_frame"] = True
    img_meta["orientation"] = orientation

    plane = _drop_empty({k: d.get(k) for k in _GEOMETRY_KEYS})
    if d.get("timestamp_iso"):
        plane["t_iso"] = d["timestamp_iso"]
    if d.get("exposure_us"):
        plane["exposure_us"] = d["exposure_us"]
    if d.get("gain_pct") is not None:
        plane["gain_pct"] = d["gain_pct"]

    acquisition = {k: v for k, v in d.items()
                   if k not in _GEOMETRY_KEYS + _ORIENTATION_KEYS
                   + _DATASET_KEYS + _PROVENANCE_KEYS}
    if acquisition:
        img_meta["acquisition"] = acquisition
    prov = _provenance("capture", operator=str(d.get("operator", "")))
    if d.get("warnings"):
        prov["warnings"] = list(d["warnings"])
    img_meta["provenance"] = prov

    channels = None
    if d.get("channel"):
        channels = [{"name": str(d["channel"])}]

    dataset_meta = _drop_empty({
        "profile": "mebp.capture/1",
        "plate_id": d.get("plate", ""),
        "well": d.get("well", ""),
        "objective": d.get("objective", ""),
        "operator": d.get("operator", ""),
        "notes": d.get("notes", ""),
    })

    path = Path(path)
    with ND3Writer(path, dataset_meta=dataset_meta,
                   generator={"software": SOFTWARE,
                              "software_version": SOFTWARE_VERSION}) as w:
        w.add_image(image_id, arr, axes=axes, pixel_format=fmt,
                    meta=img_meta, channels=channels,
                    planes=[plane] if plane else None,
                    preview_png=_encode_preview_png(src) if preview else None)
    return path


# --------------------------------------------------------------------------
# LabLink image-job bridge
# --------------------------------------------------------------------------

# LabLink's contract (lablink/docs/IMAGE-JOB-FORMAT.md): one TIFF (or ND2) +
# a `.job.json` sidecar with format "lablink.imagejob/1". The sidecar
# OVERRIDES whatever the image file claims — a TIFF *invents* the container's
# bit depth and a placeholder channel name, so the sidecar is the authority.
LABLINK_SIDECAR_FORMAT = "lablink.imagejob/1"
# Upload-name rule, verbatim from that doc (verified against validate_name).
_LABLINK_NAME_RE = "^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$"


def export_lablink_job(nd3_path, image_id, out_dir, *,
                       stem: str = "",
                       recipe: str = "", knobs: Optional[dict] = None,
                       bit_depth: Optional[int] = None,
                       wavelengths: Optional[dict] = None,
                       note: str = "") -> tuple:
    """Extract one image from an .nd3 into a LabLink job pair
    (``<stem>.tif`` + ``<stem>.job.json``), ready to push per
    lablink/docs/IMAGE-JOB-FORMAT.md §4.

    Refuses rather than guesses (LabLink's own philosophy):

    * no ``scale.um_per_px`` on the image → refused (``pixel_size_um`` is
      required and everything spatial derives from it);
    * no sensor bit depth → refused unless ``bit_depth=`` is passed — the
      array's dtype is the CONTAINER depth, exactly the invented value the
      sidecar exists to override, so it is never used as a fallback;
    * a stem outside LabLink's upload-name rule → refused (pass ``stem=``);
    * a T/C/Z stack → refused (LabLink jobs are one image per pair; export
      per-channel .nd3 images individually).

    ``wavelengths`` maps channel name → ``{"emission_nm": .., "excitation_nm":
    ..}``. MEBP does not currently record filter wavelengths, so without this
    the sidecar omits them and a deconvolution recipe will refuse with
    ``missing_metadata`` naming the fields — the designed remediation loop,
    preferred over fabricating nominal values.
    """
    if not _CV2:
        raise ND3ExportError("cv2 is required to write the LabLink TIFF")
    nd3_path = Path(nd3_path)
    out_dir = Path(out_dir)

    with ND3.open_nd3(nd3_path) as reader:
        img = reader.image(image_id)
        if any(ax in img.axes for ax in "TCZ"):
            raise ND3ExportError(
                f"image {image_id!r} is a {img.axes} stack — LabLink jobs "
                "carry one image per pair; export each plane/channel "
                "separately")
        meta = img.meta
        pixels = img.array()
        dataset = reader.dataset_meta

    um_per_px = (meta.get("scale") or {}).get("um_per_px")
    if not um_per_px:
        raise ND3ExportError(
            f"image {image_id!r} has no scale.um_per_px — LabLink requires "
            "pixel_size_um (everything spatial derives from it)")

    acquisition = meta.get("acquisition") or {}
    sensor_bits = bit_depth
    if sensor_bits is None:
        m = re.match(r"^\s*(\d+)", str(acquisition.get("bit_depth", "")))
        if m:
            sensor_bits = int(m.group(1))
    if sensor_bits is None:
        raise ND3ExportError(
            "sensor bit depth unknown — pass bit_depth= explicitly. The "
            "array dtype is the CONTAINER depth, the invented value the "
            "sidecar exists to override, so it is deliberately not used")

    stem = stem or f"{nd3_path.stem}_{image_id}"
    if not re.match(_LABLINK_NAME_RE, stem + ".job.json"):
        raise ND3ExportError(
            f"stem {stem!r} violates LabLink's upload-name rule "
            f"({_LABLINK_NAME_RE}) — pass a compliant stem= (the true name "
            "survives in source.original_name)")

    # --- image block: the sidecar is the authority over the TIFF ---------
    image_block: dict = {
        "pixel_size_um": float(um_per_px),
        "bit_depth": int(sensor_bits),
    }
    mag = acquisition.get("magnification") or acquisition.get("objective", "")
    m = re.match(r"^\s*(\d+(?:\.\d+)?)\s*[xX×]", str(mag))
    if m:
        image_block["objective_magnification"] = float(m.group(1))
    na = acquisition.get("numerical_aperture")
    if na:
        image_block["objective_na"] = float(na)

    channel_names = [c.get("name") for c in (img.channels or []) if c.get("name")]
    if not channel_names:
        acq_channel = acquisition.get("channel") or dataset.get("channel")
        channel_names = [str(acq_channel)] if acq_channel else []
    if not channel_names:
        raise ND3ExportError(
            f"image {image_id!r} declares no channel name — LabLink requires "
            "the REAL name (a placeholder like Ch0 is the trap the sidecar "
            "exists to prevent); add a channels entry or acquisition.channel")
    channels = []
    for name in channel_names:
        entry: dict = {"name": str(name)}
        wl = (wavelengths or {}).get(name) or {}
        if wl.get("emission_nm"):
            entry["emission_nm"] = float(wl["emission_nm"])
        if wl.get("excitation_nm"):
            entry["excitation_nm"] = float(wl["excitation_nm"])
        channels.append(entry)
    image_block["channels"] = channels

    sidecar: dict = {"format": LABLINK_SIDECAR_FORMAT, "image": image_block}
    if recipe or knobs:
        job: dict = {}
        if recipe:
            job["recipe"] = str(recipe)
        if knobs is not None:
            job["knobs"] = knobs
        sidecar["job"] = job
    source = _drop_empty({
        "original_name": nd3_path.name,
        "note": note or str(dataset.get("notes", "")),
        "acquired": (img.planes[0].get("t_iso") if img.planes else None)
        or (meta.get("provenance") or {}).get("created_iso"),
    })
    if source:
        sidecar["source"] = source

    # --- files ------------------------------------------------------------
    out_dir.mkdir(parents=True, exist_ok=True)
    tif_path = out_dir / f"{stem}.tif"
    to_write = pixels
    if pixels.ndim == 3:  # .nd3 color is RGB; cv2 writes BGR
        to_write = np.ascontiguousarray(pixels[..., ::-1])
    if not cv2.imwrite(str(tif_path), to_write):
        raise ND3ExportError(f"cv2.imwrite failed for {tif_path}")
    sidecar_path = out_dir / f"{stem}.job.json"
    sidecar_path.write_text(json.dumps(sidecar, indent=2, sort_keys=True),
                            encoding="utf-8")
    return tif_path, sidecar_path


# --------------------------------------------------------------------------
# time-lapse
# --------------------------------------------------------------------------

def export_time_lapse(source, path, *, image_id: str = "lapse",
                      operator: str = "", notes: str = "") -> Path:
    """A raw time-lapse as ONE extendable T-stack.

    ``source`` is either a ``RawFrameSequenceWriter`` output directory
    (``frame_%06d.tif`` + ``manifest.json``) or an iterable of
    ``(frame, plane_dict)`` pairs.  Every frame must share shape and dtype —
    a mismatch is REFUSED rather than resampled."""
    frames_iter, acquisition, interval_s, dataset_extra = \
        _time_lapse_source(source)

    dataset_meta = _drop_empty({
        "profile": "mebp.time_lapse/1",
        "operator": operator,
        "notes": notes,
        **dataset_extra,
    })

    path = Path(path)
    appender = None
    first_bgr = None
    with ND3Writer(path, dataset_meta=dataset_meta,
                   generator={"software": SOFTWARE,
                              "software_version": SOFTWARE_VERSION}) as w:
        for frame, plane in frames_iter:
            src = np.asarray(frame)
            arr, axes2d, fmt = _bgr_to_stored(src, collapse_gray=False)
            if appender is None:
                first_bgr = src
                axes = "T" + axes2d
                img_meta: dict = {"orientation":
                                  {"pixels_stage_aligned": True}}
                if interval_s is not None:
                    img_meta["time_lapse"] = {"interval_s": float(interval_s)}
                if acquisition:
                    img_meta["acquisition"] = acquisition
                img_meta["provenance"] = _provenance("time_lapse",
                                                     operator=operator)
                appender = w.begin_stack(
                    image_id, axes=axes, dtype=arr.dtype,
                    frame_shape=arr.shape, pixel_format=fmt, meta=img_meta,
                    preview_png=_encode_preview_png(first_bgr))
            appender.append(arr, plane=plane or None)
        if appender is None:
            raise ND3ExportError("time-lapse source produced no frames")
        appender.finish()
    return path


def _time_lapse_source(source):
    """Normalise the two accepted sources to (frames_iter, acquisition,
    interval_s, dataset_extra)."""
    if isinstance(source, (str, Path)):
        seq_dir = Path(source)
        manifest_path = seq_dir / "manifest.json"
        if not manifest_path.exists():
            raise ND3ExportError(
                f"{seq_dir} has no manifest.json (not a raw time-lapse "
                "directory)")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        records = manifest.get("frames") or []
        if not records:
            raise ND3ExportError(f"{seq_dir} manifest lists no frames")
        if not _CV2:
            raise ND3ExportError("cv2 is required to read a time-lapse "
                                 "directory")

        meta_doc = manifest.get("meta") or {}
        acquisition = {k: v for k, v in meta_doc.items()
                       if k not in _GEOMETRY_KEYS + _ORIENTATION_KEYS
                       + _DATASET_KEYS + _PROVENANCE_KEYS}
        dataset_extra = _drop_empty(
            {"plate_id": meta_doc.get("plate", ""),
             "well": meta_doc.get("well", ""),
             "objective": meta_doc.get("objective", "")})

        t0 = records[0].get("t_mono")

        def _iter():
            for rec in records:
                fp = seq_dir / rec["file"]
                frame = cv2.imread(str(fp), cv2.IMREAD_UNCHANGED)
                if frame is None:
                    raise ND3ExportError(f"frame {fp} is unreadable")
                plane = {}
                if rec.get("t_wall") is not None:
                    plane["t_iso"] = datetime.fromtimestamp(
                        rec["t_wall"]).astimezone().isoformat()
                if rec.get("t_mono") is not None and t0 is not None:
                    plane["t_s"] = float(rec["t_mono"]) - float(t0)
                yield frame, plane

        return _iter(), acquisition, manifest.get("interval_s"), dataset_extra

    # In-memory iterable of (frame, plane) pairs.
    def _iter_pairs():
        for item in source:
            if isinstance(item, tuple) and len(item) == 2:
                yield item
            else:
                yield item, None

    return _iter_pairs(), {}, None, {}
