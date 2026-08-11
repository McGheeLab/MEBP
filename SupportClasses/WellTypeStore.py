"""
WellTypeStore.py — Selectable well "types" (physical tube/well geometry presets).

v7.5.x: When building a rosette (a multi-tube insert dropped into a plate well) each
sub-well is a physical vessel — e.g. a **0.1 mL PCR tube** (~20 mm tall, 6 mm top Ø,
3 mm bottom Ø). Standard vessels recur, so this module ships a library of named
``WellType`` presets whose geometry can be stamped onto a sub-well in one click, plus
user-savable custom types.

A ``WellType`` carries only DESIGN-TIME geometry:

  * ``diameter_mm``   — the opening / top diameter (single-diameter approximation; the
    tapered bottom Ø is a later follow-up).
  * ``well_depth_mm`` — cavity depth (informational).
  * ``rim_height_mm`` — how far the vessel TOP sits ABOVE the plate surface. It maps
    onto ``PlateDesign.Well.rim_height_mm`` and is carried through ``compile()`` onto
    ``WellInfo.rim_height_mm`` and aggregated by ``WellPlate.max_rim_height_mm``.

    ⚠ It is NOT wired to travel-Z clearance today, despite what this docstring used
    to claim. ``app.py::_update_insert_clearance`` unconditionally calls
    ``StageController.set_min_travel_z(None)`` (2026-07-27 operator decision — Fast
    Move Z is authoritative), so ``max_rim_height_mm`` currently has **zero**
    production consumers and nothing floors ``safe_travel_to`` from this value.
    Re-arming it is a one-line change in ``_update_insert_clearance``; until then,
    do not rely on a rim height to keep the needle clear of a tall insert.
  * ``ink_z_mm``      — optional prescribed dispense Z (rel. plate top); ``None`` = the
    well uses the global print Z.
  * ``volume_uL``     — optional, purely informational (e.g. 100 µL for a 0.1 mL tube).

Persistence (this module has ZERO GUI dependencies — json + dataclasses only):

    config/hardware/well_types/builtin/<id>.json   — bundled presets (read-only)
    config/hardware/well_types/user/<id>.json      — user presets + overrides

A user entry with the **same id SHADOWS** the built-in (user wins) — the same pattern
as ``PlateTypeStore``. Built-ins stay pristine; ``delete_user`` only removes user files.
"""

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_HW_DIR = Path(__file__).resolve().parent.parent / "config" / "hardware"
_DEFAULT_BUILTIN_DIR = _HW_DIR / "well_types" / "builtin"
_DEFAULT_USER_DIR = _HW_DIR / "well_types" / "user"


def safe_id(value) -> str:
    """Filesystem-safe token for a well-type id (used as the JSON filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "well_type"


@dataclass
class WellType:
    """A selectable physical well/tube geometry preset."""

    id: str
    display_name: str = ""
    manufacturer: str = ""
    model: str = ""
    diameter_mm: float = 6.35          # opening / top Ø (single-diameter approx)
    well_depth_mm: float = 10.67       # cavity depth (informational)
    rim_height_mm: float = 0.0         # vessel top above the plate surface (clearance)
    ink_z_mm: Optional[float] = None   # prescribed dispense Z rel. plate top; None=default
    volume_uL: Optional[float] = None  # informational nominal volume
    builtin: bool = False

    def __post_init__(self) -> None:
        # Coerce types defensively (JSON may carry strings / ints).
        self.id = str(self.id)
        for attr in ("diameter_mm", "well_depth_mm", "rim_height_mm"):
            try:
                setattr(self, attr, float(getattr(self, attr)))
            except (TypeError, ValueError):
                setattr(self, attr, 0.0)
        for attr in ("ink_z_mm", "volume_uL"):
            val = getattr(self, attr)
            if val is not None:
                try:
                    setattr(self, attr, float(val))
                except (TypeError, ValueError):
                    setattr(self, attr, None)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "display_name": self.display_name,
            "manufacturer": self.manufacturer,
            "model": self.model,
            "diameter_mm": self.diameter_mm,
            "well_depth_mm": self.well_depth_mm,
            "rim_height_mm": self.rim_height_mm,
            "ink_z_mm": self.ink_z_mm,
            "volume_uL": self.volume_uL,
            "builtin": self.builtin,
        }

    @classmethod
    def from_dict(cls, data: dict, *, builtin: bool = False) -> "WellType":
        return cls(
            id=data.get("id", ""),
            display_name=data.get("display_name", ""),
            manufacturer=data.get("manufacturer", ""),
            model=data.get("model", ""),
            diameter_mm=data.get("diameter_mm", 6.35),
            well_depth_mm=data.get("well_depth_mm", 10.67),
            rim_height_mm=data.get("rim_height_mm", 0.0),
            ink_z_mm=data.get("ink_z_mm"),
            volume_uL=data.get("volume_uL"),
            builtin=bool(data.get("builtin", builtin)),
        )

    @property
    def label(self) -> str:
        """Human-readable combo label (falls back to a composed name)."""
        if self.display_name:
            return self.display_name
        parts = [self.manufacturer, self.model]
        head = " ".join(p for p in parts if p) or self.id
        if self.volume_uL:
            return f"{head} · {self.volume_uL:g} µL"
        return head


class WellTypeStore:
    """Load/save well types: bundled built-ins + user overrides (user wins)."""

    def __init__(self, builtin_dir: Path = _DEFAULT_BUILTIN_DIR,
                 user_dir: Path = _DEFAULT_USER_DIR):
        self._builtin_dir = Path(builtin_dir)
        self._user_dir = Path(user_dir)
        self._types: dict[str, WellType] = {}
        self.reload()

    # ── Load ──────────────────────────────────────────────────────

    def _load_dir(self, directory: Path, builtin: bool) -> None:
        if not directory.exists():
            return
        for path in sorted(directory.glob("*.json")):
            try:
                with open(path, encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as exc:  # pragma: no cover - corrupt file
                logger.warning(f"WellTypeStore: failed to load {path}: {exc}")
                continue
            wt = WellType.from_dict(data, builtin=builtin)
            if not wt.id:
                logger.warning(f"WellTypeStore: {path} has no id — skipped")
                continue
            # User entries (loaded second) shadow built-ins of the same id.
            self._types[wt.id] = wt

    def reload(self) -> None:
        """Re-scan both directories (built-ins first, then user overrides)."""
        self._types = {}
        self._load_dir(self._builtin_dir, builtin=True)
        self._load_dir(self._user_dir, builtin=False)

    # ── Read ──────────────────────────────────────────────────────

    def get(self, well_type_id) -> Optional[WellType]:
        if well_type_id is None:
            return None
        return self._types.get(str(well_type_id))

    def all(self) -> list[WellType]:
        """Every known well type, sorted by label."""
        return sorted(self._types.values(), key=lambda w: w.label.lower())

    # ── Write (user overrides only) ───────────────────────────────

    def save_user(self, well_type: WellType) -> bool:
        """Persist a USER well type / override (built-ins stay pristine).

        Always written under the user directory with ``builtin=False``; the same id
        shadows a bundled built-in. Updates the in-memory cache so the live list
        reflects the new type immediately.
        """
        if not well_type.id:
            logger.warning("WellTypeStore.save_user: missing id — skipped")
            return False
        well_type.builtin = False
        fname = f"{safe_id(well_type.id)}.json"
        try:
            self._user_dir.mkdir(parents=True, exist_ok=True)
            tmp = self._user_dir / f"{fname}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(well_type.to_dict(), f, indent=2)
            os.replace(tmp, self._user_dir / fname)
        except Exception as exc:
            logger.error(f"WellTypeStore.save_user: write failed: {exc}")
            return False
        self._types[well_type.id] = well_type
        logger.info(f"WellTypeStore: saved user well type '{well_type.id}'")
        return True

    def delete_user(self, well_type_id) -> bool:
        """Delete a USER well-type file (built-ins cannot be deleted).

        Removes ``user/<id>.json`` if present, then reloads so a shadowed built-in
        (if any) re-surfaces. Returns True if a user file was removed.
        """
        if not well_type_id:
            return False
        fname = f"{safe_id(well_type_id)}.json"
        path = self._user_dir / fname
        if not path.exists():
            return False
        try:
            os.remove(path)
        except Exception as exc:
            logger.error(f"WellTypeStore.delete_user: remove failed: {exc}")
            return False
        self.reload()
        logger.info(f"WellTypeStore: deleted user well type '{well_type_id}'")
        return True


_store_singleton: Optional[WellTypeStore] = None


def get_store() -> WellTypeStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = WellTypeStore()
    return _store_singleton
