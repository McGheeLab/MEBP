"""
PlateTypeStore.py — Selectable plate "products" (a thin overlay on a base format).

v7.5.x: A well-count alone (24-well) under-specifies a plate — a 24-well ships as
Corning glass-bottom, NEST plastic-bottom, a generic plate, etc. These share the
XY grid of their base standard format but differ in **bottom material**, which
changes the needle Z references (plate bottom / safe travel / max / plate top),
and each product has its **own full-plate mosaic scan**.

A ``PlateType`` is therefore a thin OVERLAY on a base standard format:

  * geometry  — inherited from the base format (``WellPlate.from_format``); the
    runtime ``WellPlate.format`` stays the base ``int`` so every same-format
    guard / ``PLATE_DEFINITIONS`` lookup keeps working.
  * identity  — the ``PlateType.id`` becomes the ``HardwareConfig.active_plate_key``,
    so the per-plate mosaic / template / well-training stores (all keyed by that
    string) auto-segregate and auto-load when the type is selected.
  * Z offsets — ``{top, bottom, safe, max}`` mm BELOW the needle-cam fiducial,
    the guess source consumed by ``StageController.estimate_plate_z_refs``.

Persistence (this module has ZERO GUI dependencies — json + dataclasses only):

    config/hardware/plate_types/builtin/<id>.json   — bundled products (read-only)
    config/hardware/plate_types/user/<id>.json      — user products + learned overrides

A user entry with the **same id SHADOWS** the built-in (user wins). The
calibration "learn loop" writes a user override after the operator teaches the
real Z, so the built-in stays pristine and the selection key never changes.

The "Generic <N>-well" option is NOT a stored PlateType — the UI maps it to an
empty ``plate_type_id`` so ``active_plate_key`` falls back to the bare ``int``
format. Never persist a ``generic-NN`` id.
"""

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_HW_DIR = Path(__file__).resolve().parent.parent / "config" / "hardware"
_DEFAULT_BUILTIN_DIR = _HW_DIR / "plate_types" / "builtin"
_DEFAULT_USER_DIR = _HW_DIR / "plate_types" / "user"

# Canonical Z-offset keys (mm BELOW the needle-cam fiducial).
Z_OFFSET_KEYS = ("top", "bottom", "safe", "max")


def safe_id(value) -> str:
    """Filesystem-safe token for a plate-type id (used as the JSON filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "plate_type"


def is_generic(plate_type_id) -> bool:
    """True for the 'no specific product' sentinel (empty / None / 'generic*').

    A generic selection maps to ``plate_type_id=""`` so ``active_plate_key``
    falls back to the bare integer format — it is never a stored PlateType.
    """
    if not plate_type_id:
        return True
    return str(plate_type_id).lower().startswith("generic")


@dataclass
class PlateType:
    """A selectable plate product overlaying a base standard format."""

    id: str
    base_format: int
    display_name: str = ""
    manufacturer: str = ""
    model: str = ""
    bottom_material: str = ""          # e.g. "glass", "plastic" / "PS"
    well_depth_mm: Optional[float] = None   # optional geometry override
    # Standard mm-BELOW-the-needle-cam-fiducial offsets to the plate features.
    z_offsets: dict = field(
        default_factory=lambda: {k: 0.0 for k in Z_OFFSET_KEYS})
    # v7.5.x: how this product's wells LOOK to the microscope, for the mosaic
    # well auto-detection (SupportClasses/PlateWellDetector.WellAppearance).
    # A clear plastic plate shows a bright moulded rim; a black-bottom glass
    # plate can read as a dark disc — same geometry, different edge polarity.
    # Empty = auto-sense (the detector tries both and reports which won, which
    # is what you then store here). Kept as a plain dict so this module stays
    # free of any OpenCV/numpy import.
    well_detection: dict = field(default_factory=dict)
    builtin: bool = False

    def __post_init__(self) -> None:
        # Coerce types defensively (JSON may carry strings / ints).
        self.id = str(self.id)
        try:
            self.base_format = int(self.base_format)
        except (TypeError, ValueError):
            self.base_format = 24
        if self.well_depth_mm is not None:
            try:
                self.well_depth_mm = float(self.well_depth_mm)
            except (TypeError, ValueError):
                self.well_depth_mm = None
        # Normalise the offsets dict to the canonical keys (float, default 0).
        src = self.z_offsets if isinstance(self.z_offsets, dict) else {}
        self.z_offsets = {
            k: float(src.get(k, 0.0) or 0.0) for k in Z_OFFSET_KEYS
        }
        if not isinstance(self.well_detection, dict):
            self.well_detection = {}

    def to_dict(self) -> dict:
        out = {
            "id": self.id,
            "base_format": self.base_format,
            "display_name": self.display_name,
            "manufacturer": self.manufacturer,
            "model": self.model,
            "bottom_material": self.bottom_material,
            "well_depth_mm": self.well_depth_mm,
            "z_offsets": dict(self.z_offsets),
            "builtin": self.builtin,
        }
        # Emit only when set, so every plate type written before v7.5.x
        # round-trips byte-identically.
        if self.well_detection:
            out["well_detection"] = dict(self.well_detection)
        return out

    @classmethod
    def from_dict(cls, data: dict, *, builtin: bool = False) -> "PlateType":
        return cls(
            id=data.get("id", ""),
            base_format=data.get("base_format", 24),
            display_name=data.get("display_name", ""),
            manufacturer=data.get("manufacturer", ""),
            model=data.get("model", ""),
            bottom_material=data.get("bottom_material", ""),
            well_depth_mm=data.get("well_depth_mm"),
            z_offsets=data.get("z_offsets") or {},
            well_detection=data.get("well_detection") or {},
            builtin=bool(data.get("builtin", builtin)),
        )

    @property
    def label(self) -> str:
        """Human-readable combo label (falls back to a composed name)."""
        if self.display_name:
            return self.display_name
        parts = [self.manufacturer, self.model]
        head = " ".join(p for p in parts if p) or self.id
        bits = [f"{self.base_format}-well"]
        if self.bottom_material:
            bits.append(self.bottom_material)
        return f"{head} · " + " · ".join(bits)


class PlateTypeStore:
    """Load/save plate types: bundled built-ins + user overrides (user wins)."""

    def __init__(self, builtin_dir: Path = _DEFAULT_BUILTIN_DIR,
                 user_dir: Path = _DEFAULT_USER_DIR):
        self._builtin_dir = Path(builtin_dir)
        self._user_dir = Path(user_dir)
        self._types: dict[str, PlateType] = {}
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
                logger.warning(f"PlateTypeStore: failed to load {path}: {exc}")
                continue
            pt = PlateType.from_dict(data, builtin=builtin)
            if not pt.id:
                logger.warning(f"PlateTypeStore: {path} has no id — skipped")
                continue
            # User entries (loaded second) shadow built-ins of the same id.
            self._types[pt.id] = pt

    def reload(self) -> None:
        """Re-scan both directories (built-ins first, then user overrides)."""
        self._types = {}
        self._load_dir(self._builtin_dir, builtin=True)
        self._load_dir(self._user_dir, builtin=False)

    # ── Read ──────────────────────────────────────────────────────

    def get(self, plate_type_id) -> Optional[PlateType]:
        if plate_type_id is None:
            return None
        return self._types.get(str(plate_type_id))

    def all(self) -> list[PlateType]:
        """Every known plate type, sorted by (base_format, label)."""
        return sorted(self._types.values(), key=lambda p: (p.base_format, p.label.lower()))

    def list_for_format(self, base_format: int) -> list[PlateType]:
        """Plate types whose base format matches, sorted by label."""
        try:
            fmt = int(base_format)
        except (TypeError, ValueError):
            return []
        return sorted(
            (p for p in self._types.values() if p.base_format == fmt),
            key=lambda p: p.label.lower())

    @staticmethod
    def is_generic(plate_type_id) -> bool:
        return is_generic(plate_type_id)

    # ── Write (user overrides only) ───────────────────────────────

    def save_user(self, plate_type: PlateType) -> bool:
        """Persist a USER override for ``plate_type`` (built-ins stay pristine).

        Always written under the user directory with ``builtin=False``; the
        same id shadows the bundled built-in. Updates the in-memory cache so
        the live readout reflects the new offsets immediately.
        """
        if not plate_type.id:
            logger.warning("PlateTypeStore.save_user: missing id — skipped")
            return False
        plate_type.builtin = False
        fname = f"{safe_id(plate_type.id)}.json"
        try:
            self._user_dir.mkdir(parents=True, exist_ok=True)
            tmp = self._user_dir / f"{fname}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(plate_type.to_dict(), f, indent=2)
            os.replace(tmp, self._user_dir / fname)
        except Exception as exc:
            logger.error(f"PlateTypeStore.save_user: write failed: {exc}")
            return False
        self._types[plate_type.id] = plate_type
        logger.info(
            f"PlateTypeStore: saved user plate type '{plate_type.id}' "
            f"(z_offsets={plate_type.z_offsets})")
        return True


_store_singleton: Optional[PlateTypeStore] = None


def get_store() -> PlateTypeStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = PlateTypeStore()
    return _store_singleton
