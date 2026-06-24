"""
FluorescenceMosaicStore.py — Per-(plate, well) multi-channel fluorescence mosaics.

v7.5.x: The new **Fluorescence Mosaic** workflow rasters a single well at high
resolution once per fluorescence channel (DAPI / FITC / mCherry / Cy5 / …). The
operator switches the physical filter/illumination between channels (there is no
filter-wheel hardware), so each channel is captured as its own full single-well
mosaic. Every channel of a well shares the SAME raster grid + camera scale, so
their composites register pixel-for-pixel and can be overlaid in any pseudo-colour.

The captured mosaics are a property of the *physical plate on the stage* (their
absolute stage-µm extent matters) — so, like ``MosaicStore`` / objectives.json,
they are persisted at the machine level (not inside the swappable
``HardwareConfig``). Any other workflow (Spheroid Pick & Place, Cell Targeting,
Cell Labeling, Quick Print, the Jog plate view, …) can then look up a well and
draw the blended fluorescence image as a registered background overlay.

Files:
    config/hardware/fluorescence_mosaics.json          — metadata
    config/hardware/fluor_mosaics/<plate>_<well>_<channel>.png  — per-channel BGR

Metadata layout::

    {
      "version": "1.0",
      "wells": {
        "<plate_key>|<well_name>": {
          "plate_key": "24",
          "well_name": "A1",
          "objective": "10x",
          "date": "2026-06-22",
          "channels": {
            "DAPI": {
              "image": "fluor_mosaics/24_A1_DAPI.png",  # relative to config dir
              "color": [0, 0, 255],          # display pseudo-colour, RGB 0-255
              "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
              "um_per_px": 0.92, "mosaic_scale": 0.31,
              "frames": 36, "exposure_us": 0, "date": "2026-06-22"
            },
            ...
          }
        }
      }
    }

Colours are stored RGB (matching the channel swatches in the UI / QColor) and
converted to BGR only when blending with OpenCV. This module has ZERO GUI
dependencies (numpy + OpenCV + json only).
"""

from __future__ import annotations

import json
import logging
import os
import re
from datetime import date
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import cv2
    import numpy as np
    _CV2 = True
except ImportError:   # pragma: no cover - cv2/numpy always present in this project
    cv2 = None
    np = None
    _CV2 = False

_DEFAULT_PATH = Path("config/hardware/fluorescence_mosaics.json")

# The standard fluorescence channels surfaced in the workflow UI, with their
# default display pseudo-colours (RGB 0-255). The operator can override any
# colour per capture; this is just the seed.
CHANNELS: tuple[str, ...] = ("DAPI", "FITC", "mCherry", "Cy5")

DEFAULT_CHANNEL_COLORS: dict[str, tuple[int, int, int]] = {
    "DAPI": (60, 120, 255),     # blue
    "FITC": (0, 230, 0),        # green
    "mCherry": (255, 40, 40),   # red
    "Cy5": (255, 0, 230),       # magenta / far-red
}


def default_color(channel: str) -> tuple[int, int, int]:
    """Default display pseudo-colour (RGB) for a channel name."""
    return DEFAULT_CHANNEL_COLORS.get(channel, (220, 220, 220))


def _safe_token(value) -> str:
    """Filesystem-safe token (used in the PNG filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "x"


def well_key(plate_key, well_name) -> str:
    """Composite metadata key for one (plate, well)."""
    return f"{plate_key}|{well_name}"


class FluorescenceMosaicStore:
    """Load/save per-(plate, well) multi-channel fluorescence mosaics."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._img_dir = self._path.parent / "fluor_mosaics"
        self._data: dict = {"version": "1.0", "wells": {}}
        self._load()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
            if not isinstance(self._data.get("wells"), dict):
                self._data["wells"] = {}
        except Exception as exc:
            logger.warning(
                f"FluorescenceMosaicStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "wells": {}}

    def _save_meta(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(
                f"FluorescenceMosaicStore: failed to save metadata: {exc}")

    # ── Write ─────────────────────────────────────────────────────

    def save_channel(
        self,
        plate_key,
        well_name,
        channel: str,
        image_bgr,
        extent_um: tuple[float, float, float, float],
        color_rgb: tuple[int, int, int] | None = None,
        objective: str = "",
        um_per_px: float = 0.0,
        mosaic_scale: float = 0.0,
        frames: int = 0,
        exposure_us: float = 0.0,
    ) -> bool:
        """Persist one channel's stitched single-well mosaic.

        ``image_bgr`` is the composite (numpy BGR uint8). ``extent_um`` is its
        world extent ``(min_x, min_y, max_x, max_y)`` in **absolute stage µm**
        (``MosaicBuilder.canvas_extent_um``). ``color_rgb`` is the display
        pseudo-colour (defaults to the channel's standard colour). Returns True
        on a successful image + metadata write.
        """
        if not _CV2 or image_bgr is None:
            logger.warning("FluorescenceMosaicStore.save_channel: no cv2 / empty image")
            return False
        wkey = well_key(plate_key, well_name)
        fname = f"{_safe_token(plate_key)}_{_safe_token(well_name)}_{_safe_token(channel)}.png"
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            ok = cv2.imwrite(str(self._img_dir / fname), image_bgr)
            if not ok:
                logger.error("FluorescenceMosaicStore.save_channel: imwrite failed")
                return False
        except Exception as exc:
            logger.error(
                f"FluorescenceMosaicStore.save_channel: image write failed: {exc}")
            return False

        if color_rgb is None:
            color_rgb = default_color(channel)
        ex = [float(v) for v in extent_um]
        wells = self._data.setdefault("wells", {})
        entry = wells.setdefault(wkey, {
            "plate_key": str(plate_key),
            "well_name": str(well_name),
            "channels": {},
        })
        entry["plate_key"] = str(plate_key)
        entry["well_name"] = str(well_name)
        if objective:
            entry["objective"] = str(objective)
        entry["date"] = date.today().isoformat()
        entry.setdefault("channels", {})[str(channel)] = {
            "image": f"fluor_mosaics/{fname}",
            "color": [int(c) for c in color_rgb],
            "extent_um": ex,
            "um_per_px": float(um_per_px),
            "mosaic_scale": float(mosaic_scale),
            "frames": int(frames),
            "exposure_us": float(exposure_us),
            "date": date.today().isoformat(),
        }
        self._save_meta()
        logger.info(
            f"FluorescenceMosaicStore: saved {plate_key}/{well_name}/{channel} "
            f"({fname}, extent={ex})")
        return True

    def set_channel_color(self, plate_key, well_name, channel: str,
                          color_rgb: tuple[int, int, int]) -> bool:
        """Update the stored display pseudo-colour for one channel."""
        ch = self._channel_meta(plate_key, well_name, channel)
        if ch is None:
            return False
        ch["color"] = [int(c) for c in color_rgb]
        self._save_meta()
        return True

    # ── Read (metadata) ───────────────────────────────────────────

    def get_well(self, plate_key, well_name) -> Optional[dict]:
        """Return the metadata dict for one (plate, well) (or None)."""
        return self._data.get("wells", {}).get(well_key(plate_key, well_name))

    def _channel_meta(self, plate_key, well_name, channel) -> Optional[dict]:
        well = self.get_well(plate_key, well_name)
        if not well:
            return None
        return well.get("channels", {}).get(str(channel))

    def list_channels(self, plate_key, well_name) -> list[str]:
        well = self.get_well(plate_key, well_name)
        if not well:
            return []
        return list(well.get("channels", {}).keys())

    def list_wells(self, plate_key) -> list[str]:
        """Well names that have at least one stored channel for ``plate_key``."""
        out: list[str] = []
        for entry in self._data.get("wells", {}).values():
            if str(entry.get("plate_key")) == str(plate_key) and entry.get("channels"):
                out.append(str(entry.get("well_name")))
        return out

    def channel_color(self, plate_key, well_name, channel) -> Optional[tuple[int, int, int]]:
        ch = self._channel_meta(plate_key, well_name, channel)
        if not ch or "color" not in ch:
            return None
        c = ch["color"]
        if not (isinstance(c, (list, tuple)) and len(c) == 3):
            return None
        return tuple(int(v) for v in c)

    def get_extent_um(self, plate_key, well_name, channel=None) -> Optional[tuple]:
        """Extent of one channel, or (channel=None) of the first stored channel
        — all channels of a well share the same raster grid + extent."""
        well = self.get_well(plate_key, well_name)
        if not well:
            return None
        channels = well.get("channels", {})
        if channel is not None:
            ch = channels.get(str(channel))
            target = [ch] if ch else []
        else:
            target = list(channels.values())
        for ch in target:
            ex = ch.get("extent_um")
            if isinstance(ex, (list, tuple)) and len(ex) == 4:
                return tuple(float(v) for v in ex)
        return None

    def has(self, plate_key, well_name) -> bool:
        return bool(self.list_channels(plate_key, well_name))

    # ── Read (images) ─────────────────────────────────────────────

    def _abs_image_path(self, rel: str) -> Optional[str]:
        if not rel:
            return None
        p = self._path.parent / rel.replace("fluor_mosaics/", "fluor_mosaics" + os.sep)
        return str(p) if p.exists() else None

    def load_channel_image(self, plate_key, well_name, channel):
        """Return one channel's stored composite as a numpy BGR array (or None)."""
        if not _CV2:
            return None
        ch = self._channel_meta(plate_key, well_name, channel)
        if not ch:
            return None
        path = self._abs_image_path(ch.get("image", ""))
        if path is None:
            return None
        try:
            return cv2.imread(path)
        except Exception as exc:
            logger.warning(f"FluorescenceMosaicStore.load_channel_image failed: {exc}")
            return None

    # ── Blending ──────────────────────────────────────────────────

    def composite_overlay(self, plate_key, well_name, channels=None):
        """Blend the stored channels of one well into a single BGR overlay.

        Each channel image is converted to a grayscale intensity, tinted by its
        display colour, and additively combined (saturating) — the standard
        fluorescence false-colour merge. ``channels`` optionally restricts to a
        subset (defaults to all stored). Returns ``(image_bgr, extent_um)`` or
        ``(None, None)`` if nothing is available.
        """
        if not _CV2:
            return None, None
        well = self.get_well(plate_key, well_name)
        if not well:
            return None, None
        names = list(channels) if channels else list(well.get("channels", {}).keys())
        acc = None
        extent = None
        shape = None
        for name in names:
            img = self.load_channel_image(plate_key, well_name, name)
            if img is None:
                continue
            color = self.channel_color(plate_key, well_name, name) or default_color(name)
            if shape is None:
                shape = img.shape[:2]
                extent = self.get_extent_um(plate_key, well_name, name) or extent
            elif img.shape[:2] != shape:
                img = cv2.resize(img, (shape[1], shape[0]),
                                 interpolation=cv2.INTER_AREA)
            tinted = _tint_gray(img, color)
            acc = tinted if acc is None else cv2.add(acc, tinted)
        if acc is None:
            return None, None
        return acc, extent

    def composite_plate_overlay(self, plate_key, channels=None, target_px: int = 3000):
        """Blend EVERY stored well of a plate onto one canvas spanning their
        union extent (absolute stage µm), so a single registered overlay shows
        all captured wells. Returns ``(image_bgr, extent_um)`` or ``(None, None)``.
        """
        if not _CV2:
            return None, None
        items = []  # (well_bgr, extent)
        union = None
        for well_name in self.list_wells(plate_key):
            img, ex = self.composite_overlay(plate_key, well_name, channels)
            if img is None or ex is None:
                continue
            items.append((img, ex))
            if union is None:
                union = list(ex)
            else:
                union[0] = min(union[0], ex[0])
                union[1] = min(union[1], ex[1])
                union[2] = max(union[2], ex[2])
                union[3] = max(union[3], ex[3])
        if not items or union is None:
            return None, None
        w_um = max(1.0, union[2] - union[0])
        h_um = max(1.0, union[3] - union[1])
        scale = float(target_px) / max(w_um, h_um)   # px per µm
        cw = max(1, int(round(w_um * scale)))
        ch_ = max(1, int(round(h_um * scale)))
        canvas = np.zeros((ch_, cw, 3), dtype=np.uint8)
        for img, ex in items:
            tw = max(1, int(round((ex[2] - ex[0]) * scale)))
            th = max(1, int(round((ex[3] - ex[1]) * scale)))
            tile = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA)
            left = int(round((ex[0] - union[0]) * scale))
            top = int(round((ex[1] - union[1]) * scale))
            x0, y0 = max(0, left), max(0, top)
            x1, y1 = min(cw, left + tw), min(ch_, top + th)
            if x1 <= x0 or y1 <= y0:
                continue
            sub = tile[(y0 - top):(y1 - top), (x0 - left):(x1 - left)]
            canvas[y0:y1, x0:x1] = cv2.add(canvas[y0:y1, x0:x1], sub)
        return canvas, tuple(float(v) for v in union)

    # ── Delete ────────────────────────────────────────────────────

    def clear_channel(self, plate_key, well_name, channel) -> None:
        well = self.get_well(plate_key, well_name)
        if not well:
            return
        ch = well.get("channels", {}).pop(str(channel), None)
        if ch and ch.get("image"):
            self._unlink(ch["image"])
        if not well.get("channels"):
            self._data.get("wells", {}).pop(well_key(plate_key, well_name), None)
        self._save_meta()

    def clear_well(self, plate_key, well_name) -> None:
        well = self._data.get("wells", {}).pop(well_key(plate_key, well_name), None)
        if not well:
            return
        for ch in well.get("channels", {}).values():
            if ch.get("image"):
                self._unlink(ch["image"])
        self._save_meta()

    def _unlink(self, rel: str) -> None:
        try:
            p = self._path.parent / rel.replace(
                "fluor_mosaics/", "fluor_mosaics" + os.sep)
            if p.exists():
                p.unlink()
        except Exception as exc:
            logger.debug(f"FluorescenceMosaicStore: image unlink failed: {exc}")


def _tint_gray(image_bgr, color_rgb):
    """Grayscale-intensity → BGR tinted by ``color_rgb`` (RGB 0-255)."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    r, g, b = (float(c) for c in color_rgb)
    # OpenCV is BGR.
    out = np.empty((*gray.shape, 3), dtype=np.uint8)
    out[..., 0] = np.clip(gray * b, 0, 255).astype(np.uint8)
    out[..., 1] = np.clip(gray * g, 0, 255).astype(np.uint8)
    out[..., 2] = np.clip(gray * r, 0, 255).astype(np.uint8)
    return out


_store_singleton: Optional[FluorescenceMosaicStore] = None


def get_store() -> FluorescenceMosaicStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = FluorescenceMosaicStore()
    return _store_singleton
