"""
ReanchorFeatureStore.py — Per-plate "auto re-anchor" feature patch.

v7.5.x: During a MANUAL re-anchor the operator clicks a recognizable plate
feature in the live microscope view. That click's surrounding image patch is
saved here (per plate key) together with the feature's absolute stage position
and the µm/px it was captured at. "Auto re-anchor mosaic" then re-finds the
feature hands-free: safe-travel to the stored position, grab a live frame,
template-match the patch, and shift the whole map by (found − stored).

The stored ``stage_um`` lives in the MAP frame: after the manual re-anchor the
map equals the actual plate at that feature, and every later map translation /
warp updates it via :meth:`set_stage_um` — so it always reflects where the map
currently believes the feature to be.

Files:
    config/hardware/reanchor_features.json           — metadata per plate key
    config/hardware/reanchor_features/<key>.png       — the feature patch (BGR)

Zero GUI dependencies (numpy + OpenCV + json only). Mirrors ``MosaicStore``.
"""

from __future__ import annotations

import json
import logging
import os
import re
from datetime import date
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

try:
    import cv2
    _CV2 = True
except ImportError:   # pragma: no cover - cv2 always present in this project
    cv2 = None
    _CV2 = False

_DEFAULT_PATH = resolve_machine_path("reanchor_features.json")
# Migrated at import time (see MosaicStore.py's _DEFAULT_IMG_DIR comment).
_DEFAULT_IMG_DIR = resolve_machine_path("reanchor_features")


def _safe_key(plate_key) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(plate_key)) or "plate"


class ReanchorFeatureStore:
    """Load/save the per-plate auto-re-anchor feature patch + position."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._img_dir = (_DEFAULT_IMG_DIR if self._path == _DEFAULT_PATH
                          else self._path.parent / "reanchor_features")
        self._data: dict = {"version": "1.0", "features": {}}
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
            if not isinstance(self._data.get("features"), dict):
                self._data["features"] = {}
        except Exception as exc:
            logger.warning(
                f"ReanchorFeatureStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "features": {}}

    def _save_meta(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"ReanchorFeatureStore: failed to save: {exc}")

    # ── Write ─────────────────────────────────────────────────────

    def save(self, plate_key, patch_bgr, stage_um, um_per_px: float,
             camobj: str = "") -> bool:
        """Persist the feature patch + its map-frame position for a plate."""
        if not _CV2 or patch_bgr is None or patch_bgr.size == 0:
            return False
        if not (isinstance(stage_um, (list, tuple)) and len(stage_um) >= 2):
            return False
        key = str(plate_key)
        fname = f"{_safe_key(key)}.png"
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(self._img_dir / fname), patch_bgr):
                return False
        except Exception as exc:
            logger.error(f"ReanchorFeatureStore.save: image write: {exc}")
            return False
        self._data.setdefault("features", {})[key] = {
            "image": f"reanchor_features/{fname}",
            "stage_um": [float(stage_um[0]), float(stage_um[1])],
            "um_per_px": float(um_per_px or 0.0),
            "camobj": str(camobj or ""),
            "date": date.today().isoformat(),
        }
        self._save_meta()
        logger.info(
            f"ReanchorFeature: saved for '{key}' at "
            f"({stage_um[0]:.0f}, {stage_um[1]:.0f}) µm")
        return True

    def set_stage_um(self, plate_key, x_um: float, y_um: float) -> bool:
        """Update ONLY the stored feature position (map moved under it)."""
        rec = self._data.get("features", {}).get(str(plate_key))
        if not isinstance(rec, dict):
            return False
        rec["stage_um"] = [float(x_um), float(y_um)]
        self._save_meta()
        return True

    def clear(self, plate_key) -> None:
        rec = self._data.get("features", {}).pop(str(plate_key), None)
        if rec and rec.get("image"):
            try:
                p = self._path.parent / rec["image"].replace(
                    "reanchor_features/", "reanchor_features" + os.sep)
                if p.exists():
                    p.unlink()
            except Exception:
                pass
        self._save_meta()

    # ── Read ──────────────────────────────────────────────────────

    def get(self, plate_key) -> Optional[dict]:
        """Metadata dict {image, stage_um, um_per_px, camobj, date} or None."""
        rec = self._data.get("features", {}).get(str(plate_key))
        return dict(rec) if isinstance(rec, dict) else None

    def has(self, plate_key) -> bool:
        rec = self.get(plate_key)
        if not rec:
            return False
        p = self._path.parent / str(rec.get("image", "")).replace(
            "reanchor_features/", "reanchor_features" + os.sep)
        return p.exists()

    def load_patch(self, plate_key):
        """The stored patch as a numpy BGR array (or None)."""
        if not _CV2:
            return None
        rec = self.get(plate_key)
        if not rec or not rec.get("image"):
            return None
        p = self._path.parent / rec["image"].replace(
            "reanchor_features/", "reanchor_features" + os.sep)
        if not p.exists():
            return None
        try:
            return cv2.imread(str(p))
        except Exception as exc:
            logger.warning(f"ReanchorFeatureStore.load_patch: {exc}")
            return None


_store_singleton: Optional[ReanchorFeatureStore] = None


def get_store() -> ReanchorFeatureStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = ReanchorFeatureStore()
    return _store_singleton
