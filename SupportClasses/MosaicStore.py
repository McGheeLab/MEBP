"""
MosaicStore.py — Per-machine persistence of stitched full-plate mosaics.

v7.5.x: The Plate Location tab can raster the whole plate, stitch the camera
snapshots into one composite image (see ``MosaicBuilder``), and detect every
well at once on that field. The stitched mosaic is also useful as a background
**overlay** on other pages (Jog, Plate Location) so the operator can see the
real plate under the rendered wells/needle.

Because that mosaic is a property of the *physical plate on the stage* (its
absolute stage-µm extent matters), it is persisted at the machine level —
not inside the swappable ``HardwareConfig`` — so it survives restarts and any
page can show it. This mirrors ``CameraCalibrationStore`` / objectives.json.

Files:
    config/hardware/plate_mosaics.json          — metadata, keyed by plate key
    config/hardware/mosaics/<plate_key>.png      — the composite image (BGR)

Metadata per plate key::

    {
      "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
      "um_per_px": 3.34,                            # camera scale used
      "mosaic_scale": 0.0185,                       # mosaic px per µm
      "image": "mosaics/96.png",                    # path relative to config dir
      "frames": 42,
      "date": "2026-06-16"
    }

This module has ZERO GUI dependencies (numpy + OpenCV + json only).
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
    _CV2 = True
except ImportError:   # pragma: no cover - cv2 always present in this project
    cv2 = None
    _CV2 = False

_DEFAULT_PATH = Path("config/hardware/plate_mosaics.json")


def _safe_key(plate_key) -> str:
    """Filesystem-safe token for a plate key (used in the PNG filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(plate_key)) or "plate"


class MosaicStore:
    """Load/save per-plate stitched mosaics (image + absolute-µm extent)."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._img_dir = self._path.parent / "mosaics"
        self._data: dict = {"version": "1.0", "mosaics": {}}
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
            if not isinstance(self._data.get("mosaics"), dict):
                self._data["mosaics"] = {}
        except Exception as exc:
            logger.warning(f"MosaicStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "mosaics": {}}

    def _save_meta(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"MosaicStore: failed to save metadata: {exc}")

    # ── Write ─────────────────────────────────────────────────────

    def save(
        self,
        plate_key,
        image_bgr,
        extent_um: tuple[float, float, float, float],
        um_per_px: float = 0.0,
        mosaic_scale: float = 0.0,
        frames: int = 0,
    ) -> bool:
        """Persist a stitched mosaic for ``plate_key``.

        ``image_bgr`` is the composite (numpy BGR uint8). ``extent_um`` is the
        composite's world extent ``(min_x, min_y, max_x, max_y)`` in **absolute
        stage µm** (i.e. ``MosaicBuilder.canvas_extent_um``). Returns True on a
        successful image+metadata write.
        """
        if not _CV2 or image_bgr is None:
            logger.warning("MosaicStore.save: no cv2 or empty image")
            return False
        key = str(plate_key)
        fname = f"{_safe_key(key)}.png"
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            ok = cv2.imwrite(str(self._img_dir / fname), image_bgr)
            if not ok:
                logger.error("MosaicStore.save: cv2.imwrite failed")
                return False
        except Exception as exc:
            logger.error(f"MosaicStore.save: image write failed: {exc}")
            return False

        ex = [float(v) for v in extent_um]
        self._data.setdefault("mosaics", {})[key] = {
            "extent_um": ex,
            "um_per_px": float(um_per_px),
            "mosaic_scale": float(mosaic_scale),
            "image": f"mosaics/{fname}",
            "frames": int(frames),
            "date": date.today().isoformat(),
        }
        self._save_meta()
        logger.info(
            f"MosaicStore: saved mosaic for '{key}' "
            f"({fname}, extent={ex})")
        return True

    # ── Read ──────────────────────────────────────────────────────

    def get_meta(self, plate_key) -> Optional[dict]:
        """Return the metadata dict for ``plate_key`` (or None)."""
        if plate_key is None:
            return None
        return self._data.get("mosaics", {}).get(str(plate_key))

    def image_path(self, plate_key) -> Optional[str]:
        """Absolute path to the stored PNG for ``plate_key`` (or None)."""
        meta = self.get_meta(plate_key)
        if not meta:
            return None
        rel = meta.get("image")
        if not rel:
            return None
        p = self._path.parent / rel.replace("mosaics/", "mosaics" + os.sep)
        return str(p) if p.exists() else None

    def load_image(self, plate_key):
        """Return the stored composite as a numpy BGR array (or None)."""
        if not _CV2:
            return None
        p = self.image_path(plate_key)
        if p is None:
            return None
        try:
            return cv2.imread(p)
        except Exception as exc:
            logger.warning(f"MosaicStore.load_image failed: {exc}")
            return None

    def get_extent_um(self, plate_key) -> Optional[tuple]:
        """Return ``(min_x, min_y, max_x, max_y)`` absolute µm (or None)."""
        meta = self.get_meta(plate_key)
        if not meta or "extent_um" not in meta:
            return None
        ex = meta["extent_um"]
        if not (isinstance(ex, (list, tuple)) and len(ex) == 4):
            return None
        return tuple(float(v) for v in ex)

    def has(self, plate_key) -> bool:
        return self.image_path(plate_key) is not None

    def clear(self, plate_key) -> None:
        """Remove the stored mosaic (image + metadata) for ``plate_key``."""
        key = str(plate_key)
        meta = self._data.get("mosaics", {}).pop(key, None)
        if meta and meta.get("image"):
            try:
                p = self._path.parent / meta["image"].replace(
                    "mosaics/", "mosaics" + os.sep)
                if p.exists():
                    p.unlink()
            except Exception as exc:
                logger.debug(f"MosaicStore.clear: image unlink failed: {exc}")
        self._save_meta()


_store_singleton: Optional[MosaicStore] = None


def get_store() -> MosaicStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = MosaicStore()
    return _store_singleton
