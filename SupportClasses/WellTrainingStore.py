"""
WellTrainingStore.py — Labeled mosaic + well-location dataset for detection R&D.

v7.5.x: When the operator maps wells on a stitched full-plate mosaic by hand (place
3 corner wells → auto-fill the grid → nudge), we save the **mosaic image + the
well-centre labels** as a training sample. The accumulated corpus lets us (offline,
via the Claude interface — NOT in the running software yet) develop and tune better
automatic well-detection algorithms against real, labeled plate mosaics.

Each sample is a timestamped folder under ``config/hardware/well_training/``::

    <plate_key>_<YYYYmmdd_HHMMSS>/
        mosaic.png        — the stitched composite (BGR)
        labels.json       — metadata + per-well centres (mosaic px AND stage µm)

``labels.json``::

    {
      "plate_key": "96",
      "date": "2026-06-17T13:22:05",
      "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
      "mosaic_scale": 0.0185,                       # mosaic px per µm
      "um_per_px": 3.34,                            # camera µm/px used
      "image": "mosaic.png",
      "image_size_px": [w, h],
      "wells": {
        "A1": {"px": [cx, cy], "um": [sx, sy], "r_px": 18.0},
        ...
      }
    }

Zero GUI dependencies (numpy + OpenCV + json only).
"""

from __future__ import annotations

import json
import logging
import os
import re
from datetime import datetime
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

try:
    import cv2
    _CV2 = True
except ImportError:   # pragma: no cover - cv2 present in this project
    cv2 = None
    _CV2 = False

_DEFAULT_DIR = resolve_machine_path("well_training")


def _safe(token) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(token)) or "plate"


class WellTrainingStore:
    """Append-only store of labeled mosaic samples for well-detection R&D."""

    def __init__(self, root: Path = _DEFAULT_DIR):
        self._root = Path(root)

    @property
    def root(self) -> Path:
        return self._root

    def save_sample(
        self,
        plate_key,
        image_bgr,
        wells: dict,
        extent_um=None,
        mosaic_scale: float = 0.0,
        um_per_px: float = 0.0,
        timestamp: Optional[str] = None,
    ) -> Optional[str]:
        """Persist one labeled sample. Returns the sample folder path (or None).

        ``wells`` maps well name → a dict that may contain ``px`` (mosaic-pixel
        centre [x, y]), ``um`` (absolute stage µm [x, y]) and ``r_px`` (radius).
        Whatever is provided is recorded verbatim.
        """
        if not _CV2 or image_bgr is None or not wells:
            logger.warning("WellTrainingStore.save_sample: no cv2 / image / wells")
            return None
        ts = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = self._root / f"{_safe(plate_key)}_{ts}"
        try:
            folder.mkdir(parents=True, exist_ok=True)
            ok = cv2.imwrite(str(folder / "mosaic.png"), image_bgr)
            if not ok:
                logger.error("WellTrainingStore: cv2.imwrite failed")
                return None
        except Exception as exc:
            logger.error(f"WellTrainingStore: image write failed: {exc}")
            return None

        try:
            h, w = image_bgr.shape[:2]
        except Exception:
            h, w = 0, 0
        clean_wells: dict = {}
        for name, rec in wells.items():
            entry: dict = {}
            if isinstance(rec, dict):
                if rec.get("px") is not None:
                    entry["px"] = [float(rec["px"][0]), float(rec["px"][1])]
                if rec.get("um") is not None:
                    entry["um"] = [float(rec["um"][0]), float(rec["um"][1])]
                if rec.get("r_px") is not None:
                    entry["r_px"] = float(rec["r_px"])
            else:                       # bare (x, y) tuple → treat as px
                entry["px"] = [float(rec[0]), float(rec[1])]
            clean_wells[str(name)] = entry

        meta = {
            "plate_key": str(plate_key),
            "date": datetime.now().isoformat(timespec="seconds"),
            "extent_um": ([float(v) for v in extent_um]
                          if extent_um is not None and len(extent_um) == 4
                          else None),
            "mosaic_scale": float(mosaic_scale),
            "um_per_px": float(um_per_px),
            "image": "mosaic.png",
            "image_size_px": [int(w), int(h)],
            "wells": clean_wells,
        }
        try:
            tmp = folder / "labels.json.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
            os.replace(tmp, folder / "labels.json")
        except Exception as exc:
            logger.error(f"WellTrainingStore: labels write failed: {exc}")
            return None
        logger.info(
            f"WellTrainingStore: saved sample {folder.name} "
            f"({len(clean_wells)} wells)")
        return str(folder)

    def list_samples(self) -> list:
        """Sample folder paths that contain both a mosaic and labels."""
        if not self._root.exists():
            return []
        out = []
        for d in sorted(self._root.iterdir()):
            if (d.is_dir() and (d / "mosaic.png").exists()
                    and (d / "labels.json").exists()):
                out.append(str(d))
        return out


_store_singleton: Optional[WellTrainingStore] = None


def get_store() -> WellTrainingStore:
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = WellTrainingStore()
    return _store_singleton
