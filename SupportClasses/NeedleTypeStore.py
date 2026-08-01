"""
NeedleTypeStore.py — Selectable pulled-glass-capillary needle presets.

v7.6: A hypodermic cannula is fully described by its ASTM gauge, so it comes from
``config/hardware/needles.json``. A **pulled glass capillary** is not — it is a
glass blank (barrel ID/OD/length) drawn out on a puller into a fine tip (tip
ID/OD/length). Those six numbers come from the pull *recipe*, and a lab pulls the
same recipe over and over, so this module ships a library of named
``NeedleType`` presets whose geometry can be stamped onto the needle in one
click, plus user-savable custom types.

A ``NeedleType`` carries only DESIGN-TIME geometry plus provenance:

  * ``barrel_id_um`` / ``barrel_od_um`` / ``barrel_length_mm`` — the bulk section.
    This is the volume-holding stage; it also contributes (a little) to the flow
    resistance.
  * ``tip_id_um`` / ``tip_length_mm`` — the pulled tip. The tip sets the deposited
    feature size and DOMINATES the flow resistance (Q ∝ d⁴), so it is what the
    max-flow ceiling and the print bead are computed from.
  * ``tip_od_um``   — optional; when unknown the needle falls back to the barrel's
    OD/ID ratio applied to the tip ID.
  * ``tip_profile`` — "cylinder" (straight tip of the tip Ø) or "cone" (linear
    taper from the barrel Ø down to the tip Ø).
  * ``puller_program`` / ``notes`` — free text. The recipe is the metadata worth
    keeping; without it a stored geometry cannot be reproduced at the bench.

STAMP, DON'T REFERENCE. Selecting a preset COPIES its geometry into the
``NeedleSpec``; ``NeedleSpec.needle_type_id`` records which preset it came from
and is never resolved at load time. Editing or deleting a preset therefore can
never silently mutate a saved hardware config or invalidate a calibration —
the same contract as ``WellTypeStore`` / ``PlateTypeStore``.

Persistence (this module has ZERO GUI dependencies — json + dataclasses only):

    config/hardware/needle_types/builtin/<id>.json   — bundled presets (read-only)
    config/hardware/needle_types/user/<id>.json      — user presets + overrides

A user entry with the **same id SHADOWS** the built-in (user wins). Built-ins stay
pristine; ``delete_user`` only removes user files and then reloads so a shadowed
built-in re-surfaces.
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
_DEFAULT_BUILTIN_DIR = _HW_DIR / "needle_types" / "builtin"
_DEFAULT_USER_DIR = _HW_DIR / "needle_types" / "user"


def safe_id(value) -> str:
    """Filesystem-safe token for a needle-type id (used as the JSON filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "needle_type"


@dataclass
class NeedleType:
    """A selectable pulled-glass-capillary geometry preset."""

    id: str
    display_name: str = ""
    manufacturer: str = ""
    model: str = ""
    # Bulk barrel
    barrel_id_um: float = 1000.0
    barrel_od_um: float = 1500.0
    barrel_length_mm: float = 100.0
    # Pulled tip
    tip_id_um: float = 30.0
    tip_length_mm: float = 5.0
    tip_od_um: Optional[float] = None     # None = unknown (derive from the barrel ratio)
    tip_profile: str = "cylinder"         # "cylinder" | "cone"
    # Provenance
    puller_program: str = ""
    notes: str = ""
    builtin: bool = False

    def __post_init__(self) -> None:
        # Coerce types defensively (JSON may carry strings / ints).
        self.id = str(self.id)
        for attr in ("barrel_id_um", "barrel_od_um", "barrel_length_mm",
                     "tip_id_um", "tip_length_mm"):
            try:
                setattr(self, attr, float(getattr(self, attr)))
            except (TypeError, ValueError):
                setattr(self, attr, 0.0)
        if self.tip_od_um is not None:
            try:
                self.tip_od_um = float(self.tip_od_um)
            except (TypeError, ValueError):
                self.tip_od_um = None
            if self.tip_od_um is not None and self.tip_od_um <= 0:
                self.tip_od_um = None
        profile = str(self.tip_profile or "cylinder").strip().lower()
        self.tip_profile = profile if profile in ("cylinder", "cone") else "cylinder"

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "display_name": self.display_name,
            "manufacturer": self.manufacturer,
            "model": self.model,
            "barrel_id_um": self.barrel_id_um,
            "barrel_od_um": self.barrel_od_um,
            "barrel_length_mm": self.barrel_length_mm,
            "tip_id_um": self.tip_id_um,
            "tip_length_mm": self.tip_length_mm,
            "tip_od_um": self.tip_od_um,
            "tip_profile": self.tip_profile,
            "puller_program": self.puller_program,
            "notes": self.notes,
            "builtin": self.builtin,
        }

    @classmethod
    def from_dict(cls, data: dict, *, builtin: bool = False) -> "NeedleType":
        return cls(
            id=data.get("id", ""),
            display_name=data.get("display_name", ""),
            manufacturer=data.get("manufacturer", ""),
            model=data.get("model", ""),
            barrel_id_um=data.get("barrel_id_um", 1000.0),
            barrel_od_um=data.get("barrel_od_um", 1500.0),
            barrel_length_mm=data.get("barrel_length_mm", 100.0),
            tip_id_um=data.get("tip_id_um", 30.0),
            tip_length_mm=data.get("tip_length_mm", 5.0),
            tip_od_um=data.get("tip_od_um"),
            tip_profile=data.get("tip_profile", "cylinder"),
            puller_program=data.get("puller_program", ""),
            notes=data.get("notes", ""),
            builtin=bool(data.get("builtin", builtin)),
        )

    @property
    def label(self) -> str:
        """Human-readable combo label (falls back to a composed name)."""
        if self.display_name:
            return self.display_name
        parts = [self.manufacturer, self.model]
        head = " ".join(p for p in parts if p) or self.id
        if self.tip_id_um:
            return f"{head} · {self.tip_id_um:g} µm tip"
        return head

    def matches(self, *, barrel_id_um: float, barrel_od_um: float,
                barrel_length_mm: float, tip_id_um: float,
                tip_length_mm: float, tip_od_um: Optional[float],
                tip_profile: str, tol: float = 1e-6) -> bool:
        """True when live geometry still equals this preset.

        The Needle page uses this to flip its picker to "(custom)" the moment the
        operator edits a value away from the stored recipe — so the combo never
        claims a preset the needle no longer matches.
        """
        def close(a, b) -> bool:
            return abs(float(a or 0.0) - float(b or 0.0)) <= tol

        return (close(self.barrel_id_um, barrel_id_um)
                and close(self.barrel_od_um, barrel_od_um)
                and close(self.barrel_length_mm, barrel_length_mm)
                and close(self.tip_id_um, tip_id_um)
                and close(self.tip_length_mm, tip_length_mm)
                and close(self.tip_od_um or 0.0, tip_od_um or 0.0)
                and str(self.tip_profile) == str(tip_profile))


class NeedleTypeStore:
    """Load/save needle types: bundled built-ins + user overrides (user wins)."""

    def __init__(self, builtin_dir: Path = _DEFAULT_BUILTIN_DIR,
                 user_dir: Path = _DEFAULT_USER_DIR):
        self._builtin_dir = Path(builtin_dir)
        self._user_dir = Path(user_dir)
        self._types: dict[str, NeedleType] = {}
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
                logger.warning(f"NeedleTypeStore: failed to load {path}: {exc}")
                continue
            nt = NeedleType.from_dict(data, builtin=builtin)
            if not nt.id:
                logger.warning(f"NeedleTypeStore: {path} has no id — skipped")
                continue
            # User entries (loaded second) shadow built-ins of the same id.
            self._types[nt.id] = nt

    def reload(self) -> None:
        """Re-scan both directories (built-ins first, then user overrides)."""
        self._types = {}
        self._load_dir(self._builtin_dir, builtin=True)
        self._load_dir(self._user_dir, builtin=False)

    # ── Read ──────────────────────────────────────────────────────

    def get(self, needle_type_id) -> Optional[NeedleType]:
        if needle_type_id is None:
            return None
        return self._types.get(str(needle_type_id))

    def all(self) -> list[NeedleType]:
        """Every known needle type, sorted by label."""
        return sorted(self._types.values(), key=lambda n: n.label.lower())

    # ── Write (user overrides only) ───────────────────────────────

    def save_user(self, needle_type: NeedleType) -> bool:
        """Persist a USER needle type / override (built-ins stay pristine).

        Always written under the user directory with ``builtin=False``; the same
        id shadows a bundled built-in. Updates the in-memory cache so the live
        list reflects the new type immediately.
        """
        if not needle_type.id:
            logger.warning("NeedleTypeStore.save_user: missing id — skipped")
            return False
        needle_type.builtin = False
        fname = f"{safe_id(needle_type.id)}.json"
        try:
            self._user_dir.mkdir(parents=True, exist_ok=True)
            tmp = self._user_dir / f"{fname}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(needle_type.to_dict(), f, indent=2)
            os.replace(tmp, self._user_dir / fname)
        except Exception as exc:
            logger.error(f"NeedleTypeStore.save_user: write failed: {exc}")
            return False
        self._types[needle_type.id] = needle_type
        logger.info(f"NeedleTypeStore: saved user needle type '{needle_type.id}'")
        return True

    def delete_user(self, needle_type_id) -> bool:
        """Delete a USER needle-type file (built-ins cannot be deleted).

        Removes ``user/<id>.json`` if present, then reloads so a shadowed
        built-in (if any) re-surfaces. Returns True if a user file was removed.
        """
        if not needle_type_id:
            return False
        fname = f"{safe_id(needle_type_id)}.json"
        path = self._user_dir / fname
        if not path.exists():
            return False
        try:
            os.remove(path)
        except Exception as exc:
            logger.error(f"NeedleTypeStore.delete_user: remove failed: {exc}")
            return False
        self.reload()
        logger.info(f"NeedleTypeStore: deleted user needle type '{needle_type_id}'")
        return True


_store_singleton: Optional[NeedleTypeStore] = None


def get_store() -> NeedleTypeStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = NeedleTypeStore()
    return _store_singleton
