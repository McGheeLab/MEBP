"""
MosaicAlignmentStore.py — Learned mosaic correction per microscope camera + objective.

v7.5.x: A full-plate mosaic's accuracy is dominated by the camera's effective
µm/px (FOV) for the *current objective*, plus a small residual systematic offset
the global registration measures. Both are a property of the **microscope camera
+ objective in use** — so once measured (by a Quick FOV calibration, or learned
from a completed mosaic's global registration) they are persisted here and
re-applied to the NEXT mosaic, which then needs very little correction.

This is distinct from:
  • ObjectiveCalibration (objectives.json) — the click-mapping µm/px per objective.
  • MosaicStore (plate_mosaics.json) — the stitched image, per plate.

Data file: config/hardware/mosaic_alignment.json

Per key ("<camera-identity>|<objective>", falling back to just the objective)::

    {
      "um_per_px": 3.21,             # effective FOV µm/px for the mosaic
      "shift_um": [12.4, -3.1],      # residual global registration offset (µm)
      "frames": 42,                  # tiles in the mosaic that learned the shift
      "source": "quick_fov" | "mosaic_scan",
      "date": "2026-06-17"
    }

Zero GUI dependencies (json + Path only).
"""

from __future__ import annotations

import json
import logging
import os
from datetime import date
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_PATH = Path("config/hardware/mosaic_alignment.json")


class MosaicAlignmentStore:
    """Load/save the per-camera+objective mosaic correction."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._data: dict = {"version": "1.0", "alignments": {}}
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
            if not isinstance(self._data.get("alignments"), dict):
                self._data["alignments"] = {}
        except Exception as exc:
            logger.warning(f"MosaicAlignmentStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "alignments": {}}

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"MosaicAlignmentStore: failed to save: {exc}")

    # ── Read ──────────────────────────────────────────────────────

    def get(self, key) -> Optional[dict]:
        if not key:
            return None
        return self._data.get("alignments", {}).get(str(key))

    def get_um_per_px(self, key) -> Optional[float]:
        rec = self.get(key)
        if not rec:
            return None
        v = rec.get("um_per_px")
        try:
            v = float(v)
        except (TypeError, ValueError):
            return None
        return v if v > 0 else None

    def is_manual(self, key) -> bool:
        """True if the stored shift for ``key`` came from a deliberate by-eye
        manual alignment (``source == "manual_align"``). Auto registration uses
        this to avoid silently clobbering an operator's manual calibration."""
        rec = self.get(key)
        return bool(rec) and rec.get("source") == "manual_align"

    def get_shift_um(self, key) -> Optional[tuple]:
        rec = self.get(key)
        if not rec or "shift_um" not in rec:
            return None
        s = rec["shift_um"]
        if not (isinstance(s, (list, tuple)) and len(s) == 2):
            return None
        try:
            return (float(s[0]), float(s[1]))
        except (TypeError, ValueError):
            return None

    # ── Write ─────────────────────────────────────────────────────

    def set_um_per_px(self, key, um_per_px: float, source: str = "quick_fov") -> None:
        if not key or not um_per_px or um_per_px <= 0:
            return
        rec = self._data.setdefault("alignments", {}).setdefault(str(key), {})
        rec["um_per_px"] = round(float(um_per_px), 6)
        rec["source"] = source
        rec["date"] = date.today().isoformat()
        self._save()
        logger.info(f"MosaicAlignment: '{key}' µm/px = {um_per_px:.4f} ({source})")

    def set_shift_um(self, key, dx_um: float, dy_um: float,
                     frames: int = 0, source: str = "mosaic_scan") -> None:
        if not key:
            return
        rec = self._data.setdefault("alignments", {}).setdefault(str(key), {})
        rec["shift_um"] = [round(float(dx_um), 3), round(float(dy_um), 3)]
        rec["frames"] = int(frames)
        rec["source"] = source
        rec["date"] = date.today().isoformat()
        self._save()
        logger.info(
            f"MosaicAlignment: '{key}' shift = "
            f"({dx_um:.1f}, {dy_um:.1f}) µm ({frames} tiles)")

    def clear(self, key) -> None:
        self._data.get("alignments", {}).pop(str(key), None)
        self._save()


_store_singleton: Optional[MosaicAlignmentStore] = None


def get_store() -> MosaicAlignmentStore:
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = MosaicAlignmentStore()
    return _store_singleton
