"""
PlateTemplateStore.py — Reusable "as-built" plate well map for fast re-registration.

v7.5.x: A full-plate mosaic scan is slow (thousands of tiles). Once it's done and
the wells are mapped (auto-detect or the Map-wells dialog), the resulting well
centres are an accurate **as-built template** of THIS plate as seen by THIS camera
+ objective. We persist that template so the operator only scans once: on a later
session they re-measure just **3 reference wells** and the whole plate re-registers
by fitting the template → the 3 new measurements (see
``PlateWarpCalibrator.register_from_template``).

The template is keyed by ``<plate_key>|<camera-identity>|<objective>`` because the
well map's pixel/µm geometry is only consistent for the same camera + objective
(µm/px, FOV). Re-registration must use the same settings.

Data file: config/hardware/plate_templates.json

Per key::

    {
      "plate_key": "24",
      "camera": "C3CMOS10000KPA",
      "objective": "4x",
      "wells": {"A1": [x_um, y_um], ...},   # absolute stage µm at scan time
      "ref_wells": ["A1", "A6", "D1"],      # 3 wells to re-measure
      "um_per_px": 3.12,
      "mosaic_scale": 0.0266,
      "n_wells": 24,
      "date": "2026-06-17T..."
    }

Only relative geometry is used at re-register time, so the absolute (scan-time)
frame is fine — the plate can sit anywhere on the stage later. Zero GUI deps.
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

_DEFAULT_PATH = resolve_machine_path("plate_templates.json")


def make_key(plate_key, camera_identity, objective) -> str:
    cam = camera_identity if camera_identity else "cam"
    obj = objective if objective else "default"
    return f"{plate_key}|{cam}|{obj}"


def corner_ref_wells(wells: dict) -> list:
    """Pick 3 non-collinear reference wells (origin / +X end / +Y end) from a
    well-centre map by position extremes — robust to naming/custom plates."""
    names = list(wells.keys())
    if len(names) < 3:
        return names

    def p(n):
        return (float(wells[n][0]), float(wells[n][1]))
    origin = min(names, key=lambda n: p(n)[0] + p(n)[1])
    xend = max(names, key=lambda n: p(n)[0] - p(n)[1])
    yend = max(names, key=lambda n: p(n)[1] - p(n)[0])
    chosen = [origin]
    for c in (xend, yend):
        if c not in chosen:
            chosen.append(c)
    for n in names:
        if len(chosen) >= 3:
            break
        if n not in chosen:
            chosen.append(n)
    return chosen[:3]


class PlateTemplateStore:
    """Load/save reusable per-(plate, camera, objective) well-map templates."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._data: dict = {"version": "1.0", "templates": {}}
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
            if not isinstance(self._data.get("templates"), dict):
                self._data["templates"] = {}
        except Exception as exc:
            logger.warning(f"PlateTemplateStore: load failed {self._path}: {exc}")
            self._data = {"version": "1.0", "templates": {}}

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"PlateTemplateStore: save failed: {exc}")

    # ── Write ──────────────────────────────────────────────────────

    def save_template(self, plate_key, camera_identity, objective, wells: dict,
                      ref_wells=None, um_per_px: float = 0.0,
                      mosaic_scale: float = 0.0) -> Optional[str]:
        """Persist a well-map template. ``wells`` = {name: (x_um, y_um)}.
        Returns the key, or None if there aren't enough wells."""
        clean = {}
        for name, xy in (wells or {}).items():
            try:
                clean[str(name)] = [float(xy[0]), float(xy[1])]
            except (TypeError, ValueError, IndexError):
                continue
        if len(clean) < 3:
            logger.warning("PlateTemplateStore.save_template: <3 wells, skipped")
            return None
        key = make_key(plate_key, camera_identity, objective)
        refs = list(ref_wells) if ref_wells else corner_ref_wells(clean)
        refs = [r for r in refs if r in clean][:3]
        if len(refs) < 3:
            refs = corner_ref_wells(clean)
        self._data.setdefault("templates", {})[key] = {
            "plate_key": str(plate_key),
            "camera": str(camera_identity) if camera_identity else None,
            "objective": str(objective) if objective else None,
            "wells": clean,
            "ref_wells": refs,
            "um_per_px": float(um_per_px),
            "mosaic_scale": float(mosaic_scale),
            "n_wells": len(clean),
            "date": datetime.now().isoformat(timespec="seconds"),
        }
        self._save()
        logger.info(f"PlateTemplateStore: saved '{key}' ({len(clean)} wells, "
                    f"refs={refs})")
        return key

    def clear(self, plate_key, camera_identity, objective) -> None:
        self._data.get("templates", {}).pop(
            make_key(plate_key, camera_identity, objective), None)
        self._save()

    # ── Read ───────────────────────────────────────────────────────

    def get(self, plate_key, camera_identity, objective) -> Optional[dict]:
        return self._data.get("templates", {}).get(
            make_key(plate_key, camera_identity, objective))

    def has(self, plate_key, camera_identity, objective) -> bool:
        return self.get(plate_key, camera_identity, objective) is not None

    def get_wells(self, plate_key, camera_identity, objective) -> dict:
        """{name: (x_um, y_um)} for the template (or empty)."""
        rec = self.get(plate_key, camera_identity, objective)
        if not rec:
            return {}
        return {n: (float(v[0]), float(v[1]))
                for n, v in rec.get("wells", {}).items()
                if isinstance(v, (list, tuple)) and len(v) == 2}

    def get_ref_wells(self, plate_key, camera_identity, objective) -> list:
        rec = self.get(plate_key, camera_identity, objective)
        return list(rec.get("ref_wells", [])) if rec else []


_store_singleton: Optional[PlateTemplateStore] = None


def get_store() -> PlateTemplateStore:
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = PlateTemplateStore()
    return _store_singleton
