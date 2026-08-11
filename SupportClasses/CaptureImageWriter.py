"""
CaptureImageWriter.py — write a captured frame with its metadata inside it.

v7.14, from the operator's ask: "if we can save this information in the image
meta data that would be great."

* **PNG** — one ``tEXt`` chunk per field, plus the whole record as ``iTXt``.
  ⚠ tEXt is **latin-1 only**: a value containing ``µ`` (which every µm/px field
  has) raises or mangles. Anything non-latin-1 goes through ``iTXt``, which is
  UTF-8. This repo already has a cp1252-encoding scar; this is the same trap.
* **TIFF** — ``ImageDescription`` carrying an ImageJ header block followed by
  the JSON, plus the resolution tags so **ImageJ's Set Scale is automatic**
  from our µm/px. That is the operator's real downstream workflow.
* **Raw 16-bit REQUIRES TIFF** — Pillow's 16-bit PNG byte order is surprising
  and ImageJ reads 16-bit TIFF natively, so a raw PNG is refused rather than
  silently written as something that looks right and reads wrong.
* **A JSON sidecar is always written** (unless disabled): it is greppable, it
  survives the operator re-saving the image in ImageJ (which discards our text
  chunks), and for video it is the ONLY metadata path.

The sidecar is ``foo.png.json``, not ``foo.json`` — a PNG and a TIFF captured
in the same second would otherwise fight over one sidecar.

Pure: numpy + Pillow + (optionally) cv2. No Qt.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP = True
except ImportError:      # pragma: no cover
    _NP = False

try:
    from PIL import Image, PngImagePlugin
    _PIL = True
except ImportError:      # pragma: no cover
    Image = PngImagePlugin = None
    _PIL = False


class CaptureWriteError(RuntimeError):
    """A capture could not be written — always with an operator-readable why."""


def _is_latin1(text: str) -> bool:
    try:
        text.encode("latin-1")
        return True
    except (UnicodeEncodeError, AttributeError):
        return False


def _to_pil(arr):
    """ndarray → PIL Image. BGR (cv2 order) becomes RGB; 16-bit stays I;16."""
    a = np.asarray(arr)
    if a.ndim == 3:
        if a.shape[2] == 3:
            a = a[:, :, ::-1]           # BGR → RGB
        elif a.shape[2] == 1:
            a = a[:, :, 0]
    if a.dtype == np.uint16:
        if a.ndim != 2:
            raise CaptureWriteError(
                "16-bit colour images are not supported — capture raw as mono.")
        # No mode= : Pillow infers "I;16" from uint16, and passing mode
        # explicitly is deprecated (removed in Pillow 13).
        return Image.fromarray(a)
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return Image.fromarray(a)


def write_image(path: Path, array, meta=None, *, embed: bool = True,
                sidecar: bool = True) -> Path:
    """Write ``array`` to ``path``, embedding ``meta`` where the format allows.

    ``path`` must already exist as a 0-byte file created by
    ``CaptureSpec.open_unique`` — that is what guarantees no capture ever
    overwrites another. Returns the path written.
    """
    if not _NP:
        raise CaptureWriteError("numpy is unavailable — cannot save an image.")
    path = Path(path)
    a = np.asarray(array)
    is_16 = (a.dtype == np.uint16)
    fmt = "TIFF" if path.suffix.lower() in (".tif", ".tiff") else "PNG"

    if is_16 and fmt != "TIFF":
        raise CaptureWriteError(
            "A raw 16-bit image must be saved as TIFF (16-bit PNG is not "
            "reliably readable). Change the still format to TIFF.")

    if not _PIL:
        _write_without_pillow(path, a)
    elif fmt == "TIFF":
        _write_tiff(path, a, meta if embed else None)
    else:
        _write_png(path, a, meta if embed else None)

    if sidecar and meta is not None:
        try:
            from SupportClasses.CaptureMetadata import to_json
            side = path.with_suffix(path.suffix + ".json")
            side.write_text(to_json(meta), encoding="utf-8")
        except Exception as exc:
            logger.warning(f"capture sidecar not written: {exc}")
    return path


def _write_png(path: Path, a, meta):
    info = None
    if meta is not None:
        from SupportClasses.CaptureMetadata import to_json, to_text_pairs
        info = PngImagePlugin.PngInfo()
        for key, value in to_text_pairs(meta).items():
            # latin-1 → tEXt (compact, universally read); anything else → iTXt
            # (UTF-8). "µm/px" in a tEXt chunk is the exact encoding trap.
            if _is_latin1(value):
                info.add_text(key, value)
            else:
                info.add_itxt(key, value, lang="", tkey=key)
        info.add_itxt("MEBP:json", to_json(meta), lang="", tkey="MEBP:json")
        info.add_text("Software", "MEBP")
    _to_pil(a).save(str(path), format="PNG", pnginfo=info)


def _write_tiff(path: Path, a, meta):
    kwargs = {}
    if meta is not None:
        from SupportClasses.CaptureMetadata import to_imagej_description
        kwargs["description"] = to_imagej_description(meta)
        um = getattr(meta, "um_per_px", None)
        if um:
            try:
                # ImageJ reads resolution in pixels per unit; unit 3 = cm, so
                # 1 cm / (µm/px) / 10000 µm-per-cm gives pixels per cm.
                # A SINGLE float — passing a pair makes Pillow write two
                # entries into tags 282/283, which is a malformed TIFF that
                # warns on read (and is exactly what ImageJ must parse).
                kwargs["resolution"] = 10000.0 / float(um)
                kwargs["resolution_unit"] = 3
            except (TypeError, ValueError, ZeroDivisionError):
                pass
    _to_pil(a).save(str(path), format="TIFF", **kwargs)


def _write_without_pillow(path: Path, a):      # pragma: no cover
    """Last-resort pixel write; the sidecar carries the metadata."""
    try:
        import cv2
    except ImportError:
        raise CaptureWriteError(
            "Neither Pillow nor OpenCV is available — cannot save an image.")
    if not cv2.imwrite(str(path), a):
        raise CaptureWriteError(f"OpenCV could not write {path.name}.")
    logger.warning("Pillow unavailable — image saved without embedded "
                   "metadata; see the .json sidecar.")


def read_embedded_metadata(path: Path) -> Optional[dict]:
    """Read back what :func:`write_image` embedded (tests, and diagnostics)."""
    path = Path(path)
    side = path.with_suffix(path.suffix + ".json")
    if not _PIL:
        return json.loads(side.read_text(encoding="utf-8")) if side.exists() else None
    try:
        with Image.open(str(path)) as im:
            if path.suffix.lower() in (".tif", ".tiff"):
                desc = im.tag_v2.get(270) if hasattr(im, "tag_v2") else None
                if desc:
                    _hdr, _sep, body = str(desc).partition("\n\n")
                    return json.loads(body)
            blob = (im.info or {}).get("MEBP:json")
            if blob:
                return json.loads(blob)
    except Exception as exc:
        logger.debug(f"embedded metadata unreadable in {path.name}: {exc}")
    if side.exists():
        return json.loads(side.read_text(encoding="utf-8"))
    return None
