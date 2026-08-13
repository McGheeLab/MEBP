"""
FilterCubeStore.py — selectable fluorescence filter-cube presets.

v7.18. A filter cube's identity is four or five numbers (excitation band,
emission band, dichroic edge) that recur across every rig, so typing them per
slot is both tedious and a place to make a silent mistake. This module ships a
catalogue of standard cubes that can be stamped onto a cassette slot in one
pick, plus user-savable custom entries.

**Bands are stored as CENTER + BANDWIDTH, because that is how a filter is
labelled.** A part engraved ``470/40`` is entered as center 470, width 40; the
edges (450–490) are derived. Storing edges instead would make the operator do
that arithmetic off the part every time, which is exactly the kind of
transcription step that produces a wrong number nobody can spot later. Bands are
symmetric about the center by definition of FWHM, so nothing is lost.

⚠ **PROVENANCE IS PART OF THE VALUE, not decoration.** A catalogue entry's
wavelengths are *nominal for that cube type* — real parts vary by vendor and
part number, and these numbers travel through ``MicroscopeConfigStore`` into the
LabLink sidecar, where they select a deconvolution PSF. That store's own rule is
*"leave blank if you do not know it — LabLink names a missing field, which is
recoverable; a wrong one is not."* Auto-filling nominal numbers would quietly
break that rule, so every value carries :data:`PROV_NOMINAL` /
:data:`PROV_DATASHEET` / :data:`PROV_MEASURED` and the UI shows it. Editing a
value is what promotes it off "nominal".

Persistence (ZERO GUI dependencies — json + dataclasses only):

    config/hardware/filter_cubes/builtin/*.json   — bundled catalogue (read-only)
    config/hardware/filter_cubes/user/<id>.json   — user cubes + overrides

A user entry with the **same id SHADOWS** the built-in (user wins) — the same
pattern as ``WellTypeStore`` / ``PlateTypeStore``. Built-ins stay pristine;
``delete_user`` only removes user files. A builtin file may hold a single cube
object OR a list of them, so the shipped catalogue is one readable file while a
saved custom cube is its own ``user/<id>.json``.
"""

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_shared_path

logger = logging.getLogger(__name__)

# Shared/portable catalogue — a filter cube is the same part on every rig, so
# this syncs across machines rather than living under config/hardware/<id>/.
_FILTER_CUBES_DIR = resolve_shared_path("filter_cubes")
_DEFAULT_BUILTIN_DIR = _FILTER_CUBES_DIR / "builtin"
_DEFAULT_USER_DIR = _FILTER_CUBES_DIR / "user"

#: Where a wavelength came from. Ordered weakest → strongest.
PROV_NOMINAL = "nominal"
PROV_DATASHEET = "datasheet"
PROV_MEASURED = "measured"
PROVENANCES = (PROV_NOMINAL, PROV_DATASHEET, PROV_MEASURED)

#: Human text for each provenance, for a UI that must not imply more than it has.
PROVENANCE_TEXT = {
    PROV_NOMINAL: "nominal for this cube type — confirm against your filter's "
                  "datasheet before relying on it",
    PROV_DATASHEET: "from the filter's datasheet",
    PROV_MEASURED: "measured on this instrument",
}

#: Plausible optical band, matching MicroscopeConfigStore.WAVELENGTH_BAND_NM.
MIN_WAVELENGTH_NM = 200.0
MAX_WAVELENGTH_NM = 1200.0
#: A bandpass FWHM. 0 is allowed and means "width not known" (a center alone is
#: still useful — it is what the LabLink sidecar consumes).
MAX_BANDWIDTH_NM = 600.0


def safe_id(value) -> str:
    """Filesystem-safe token for a cube id (used as the JSON filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "filter_cube"


def clean_nm(value):
    """A plausible wavelength in nm, or ``None``. Never raises."""
    try:
        nm = float(value)
    except (TypeError, ValueError):
        return None
    if not (MIN_WAVELENGTH_NM <= nm <= MAX_WAVELENGTH_NM):
        return None
    return nm


def clean_width_nm(value):
    """A plausible bandwidth in nm, or ``None``. 0 means "unknown width"."""
    try:
        nm = float(value)
    except (TypeError, ValueError):
        return None
    if not (0.0 <= nm <= MAX_BANDWIDTH_NM):
        return None
    return nm


def clean_provenance(value) -> str:
    """Coerce to a known provenance, defaulting to the WEAKEST claim.

    An unrecognised value must not be promoted — reading junk and calling it
    ``measured`` is precisely the overclaim this field exists to prevent.
    """
    v = str(value or "").strip().lower()
    return v if v in PROVENANCES else PROV_NOMINAL


def band_edges(center, width):
    """``(min_nm, max_nm)`` for a center/FWHM pair, or ``(None, None)``.

    A center with no width has no edges — reporting the center twice would
    invent a zero-width band.
    """
    c = clean_nm(center)
    w = clean_width_nm(width)
    if c is None or not w:
        return (None, None)
    return (c - w / 2.0, c + w / 2.0)


def format_band(center, width) -> str:
    """``"470/40 nm (450–490)"``, or ``"—"`` when there is nothing to show."""
    c = clean_nm(center)
    if c is None:
        return "—"
    w = clean_width_nm(width)
    if not w:
        return f"{c:g} nm"
    lo, hi = band_edges(c, w)
    return f"{c:g}/{w:g} nm ({lo:g}–{hi:g})"


@dataclass
class FilterCube:
    """One selectable fluorescence filter cube / block."""

    id: str
    display_name: str = ""
    #: Nikon block designation when there is one ("B-2A"), else "".
    nikon_block: str = ""
    vendor: str = ""
    #: Dyes/fluorophores this cube is intended for, for searching.
    dyes: tuple = ()
    excitation_nm: Optional[float] = None
    excitation_width_nm: Optional[float] = None
    emission_nm: Optional[float] = None
    emission_width_nm: Optional[float] = None
    #: Dichroic/beamsplitter EDGE wavelength (a cut-on, not a band).
    dichroic_nm: Optional[float] = None
    provenance: str = PROV_NOMINAL
    notes: str = ""
    builtin: bool = False

    def __post_init__(self) -> None:
        self.id = str(self.id)
        self.provenance = clean_provenance(self.provenance)
        for attr in ("excitation_nm", "emission_nm", "dichroic_nm"):
            setattr(self, attr, clean_nm(getattr(self, attr)))
        for attr in ("excitation_width_nm", "emission_width_nm"):
            setattr(self, attr, clean_width_nm(getattr(self, attr)))
        if isinstance(self.dyes, str):
            self.dyes = tuple(d.strip() for d in self.dyes.split(",") if d.strip())
        else:
            self.dyes = tuple(str(d).strip() for d in (self.dyes or ())
                              if str(d).strip())

    # ── Derived edges ─────────────────────────────────────────────

    @property
    def excitation_range_nm(self) -> tuple:
        return band_edges(self.excitation_nm, self.excitation_width_nm)

    @property
    def emission_range_nm(self) -> tuple:
        return band_edges(self.emission_nm, self.emission_width_nm)

    @property
    def label(self) -> str:
        """Combo label: the name, with the Nikon block when they differ."""
        head = self.display_name or self.id
        if self.nikon_block and self.nikon_block.lower() != head.lower():
            return f"{head}  ({self.nikon_block})"
        return head

    def describe(self) -> str:
        """One line for a readout: the two bands."""
        return (f"Ex {format_band(self.excitation_nm, self.excitation_width_nm)}"
                f"   ·   Em {format_band(self.emission_nm, self.emission_width_nm)}")

    @property
    def has_wavelengths(self) -> bool:
        return self.excitation_nm is not None or self.emission_nm is not None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "display_name": self.display_name,
            "nikon_block": self.nikon_block,
            "vendor": self.vendor,
            "dyes": list(self.dyes),
            "excitation_nm": self.excitation_nm,
            "excitation_width_nm": self.excitation_width_nm,
            "emission_nm": self.emission_nm,
            "emission_width_nm": self.emission_width_nm,
            "dichroic_nm": self.dichroic_nm,
            "provenance": self.provenance,
            "notes": self.notes,
            "builtin": self.builtin,
        }

    @classmethod
    def from_dict(cls, data: dict, *, builtin: bool = False) -> "FilterCube":
        return cls(
            id=data.get("id", ""),
            display_name=data.get("display_name", ""),
            nikon_block=data.get("nikon_block", ""),
            vendor=data.get("vendor", ""),
            dyes=data.get("dyes", ()),
            excitation_nm=data.get("excitation_nm"),
            excitation_width_nm=data.get("excitation_width_nm"),
            emission_nm=data.get("emission_nm"),
            emission_width_nm=data.get("emission_width_nm"),
            dichroic_nm=data.get("dichroic_nm"),
            provenance=data.get("provenance", PROV_NOMINAL),
            notes=data.get("notes", ""),
            builtin=bool(data.get("builtin", builtin)),
        )

    def optics_entry(self) -> dict:
        """The dict shape ``MicroscopeConfigStore.filter_optics`` stores.

        ⚠ ``excitation_nm`` / ``emission_nm`` stay the plain CENTERS under their
        original key names, because that is what every existing consumer reads —
        ``OpticsRegistry.resolve_slots`` and the LabLink sidecar (whose
        ``lablink.imagejob/1`` contract carries one number per channel). The
        widths, dichroic, provenance and cube id are ADDITIVE, so a cube picked
        here is byte-compatible with everything written before v7.18.
        """
        entry: dict = {}
        for key, value in (
                ("excitation_nm", self.excitation_nm),
                ("emission_nm", self.emission_nm),
                ("excitation_width_nm", self.excitation_width_nm),
                ("emission_width_nm", self.emission_width_nm),
                ("dichroic_nm", self.dichroic_nm)):
            if value is not None and value != 0:
                entry[key] = value
        if entry:
            entry["provenance"] = self.provenance
        if self.id:
            entry["cube_id"] = self.id
        return entry


class FilterCubeStore:
    """Load/save filter cubes: bundled built-ins + user overrides (user wins)."""

    def __init__(self, builtin_dir: Path = _DEFAULT_BUILTIN_DIR,
                 user_dir: Path = _DEFAULT_USER_DIR):
        self._builtin_dir = Path(builtin_dir)
        self._user_dir = Path(user_dir)
        self._cubes: dict[str, FilterCube] = {}
        self.reload()

    # ── Load ──────────────────────────────────────────────────────

    def _load_dir(self, directory: Path, builtin: bool) -> None:
        if not directory.exists():
            return
        for path in sorted(directory.glob("*.json")):
            try:
                with open(path, encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as exc:
                logger.warning(f"FilterCubeStore: failed to load {path}: {exc}")
                continue
            # A file may hold one cube or a list of them (the shipped catalogue
            # is one readable file; a saved custom cube is its own file).
            records = data if isinstance(data, list) else [data]
            for record in records:
                if not isinstance(record, dict):
                    continue
                cube = FilterCube.from_dict(record, builtin=builtin)
                if not cube.id:
                    logger.warning(f"FilterCubeStore: entry in {path} has no id "
                                   f"— skipped")
                    continue
                # User entries (loaded second) shadow built-ins of the same id.
                self._cubes[cube.id] = cube

    def reload(self) -> None:
        """Re-scan both directories (built-ins first, then user overrides)."""
        self._cubes = {}
        self._load_dir(self._builtin_dir, builtin=True)
        self._load_dir(self._user_dir, builtin=False)

    # ── Read ──────────────────────────────────────────────────────

    def get(self, cube_id) -> Optional[FilterCube]:
        if cube_id is None:
            return None
        return self._cubes.get(str(cube_id))

    def all(self) -> list[FilterCube]:
        """Every known cube, sorted by label."""
        return sorted(self._cubes.values(), key=lambda c: c.label.lower())

    def find_by_name(self, name) -> Optional[FilterCube]:
        """Match a slot label to a cube, case-insensitively.

        Tries the id, the display name, then the Nikon block designation — the
        three things an operator's typed label could plausibly be. Exact
        (folded) matches only: no substring or fuzzy tier, for the reason
        ``OpticsRegistry`` documents at length — "TxRed" and "mCherry" are not
        the same cube, and a wrong match silently mislabels a channel.
        """
        want = str(name or "").strip().lower()
        if not want:
            return None
        for attr in ("id", "display_name", "nikon_block"):
            for cube in self.all():
                if str(getattr(cube, attr) or "").strip().lower() == want:
                    return cube
        return None

    # ── Write (user overrides only) ───────────────────────────────

    def save_user(self, cube: FilterCube) -> bool:
        """Persist a USER cube / override (built-ins stay pristine)."""
        if not cube.id:
            logger.warning("FilterCubeStore.save_user: missing id — skipped")
            return False
        cube.builtin = False
        fname = f"{safe_id(cube.id)}.json"
        try:
            self._user_dir.mkdir(parents=True, exist_ok=True)
            tmp = self._user_dir / f"{fname}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(cube.to_dict(), f, indent=2)
            os.replace(tmp, self._user_dir / fname)
        except Exception as exc:
            logger.error(f"FilterCubeStore.save_user: write failed: {exc}")
            return False
        self._cubes[cube.id] = cube
        logger.info(f"FilterCubeStore: saved user filter cube '{cube.id}'")
        return True

    def delete_user(self, cube_id) -> bool:
        """Delete a USER cube file (built-ins cannot be deleted)."""
        if not cube_id:
            return False
        path = self._user_dir / f"{safe_id(cube_id)}.json"
        if not path.exists():
            return False
        try:
            os.remove(path)
        except Exception as exc:
            logger.error(f"FilterCubeStore.delete_user: remove failed: {exc}")
            return False
        self.reload()
        logger.info(f"FilterCubeStore: deleted user filter cube '{cube_id}'")
        return True


_store_singleton: Optional[FilterCubeStore] = None


def get_store() -> FilterCubeStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = FilterCubeStore()
    return _store_singleton
