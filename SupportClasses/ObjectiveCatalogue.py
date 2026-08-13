"""
ObjectiveCatalogue.py — selectable objective presets (Nikon CFI and friends).

v7.18. An objective's identity is three or four numbers — magnification, NA,
working distance, immersion — and **two of them are load-bearing for safety and
for measurement**, not decoration:

* **NA sets the depth of field**, via Berek's two-term formula in
  ``ObjectiveOptics.depth_of_field_um``. DOF sizes every focus step, so a wrong NA
  produces a sweep that either under-samples (a real peak hides between samples)
  or wastes minutes. The difference is not subtle: a 20x/0.45 has ~4 µm of DOF at
  this rig's scale while a 20x/0.75 has ~1.5 µm — nearly 3x shallower.
* **Working distance is THE COLLISION BOUND.** ``WD_SWEEP_FRACTION`` of it is how
  far the focus may travel, and ``FocusSweepPlanner.plan_turret_change`` uses it to
  decide whether a nosepiece rotation is safe at all. A CFI Plan Fluor 10x/0.30 has
  **16.0 mm**; a CFI Plan Achromat 20x/0.40 has **1.2 mm**. Confusing them is a
  13x error in the direction that puts glass through a front lens.

Typing those per slot is both tedious and a place to make a silent mistake, so
this ships a catalogue that can be stamped onto a nosepiece position in one pick,
plus user-savable custom entries. Deliberately mirrors ``FilterCubeStore`` —
same shared-path layout, same builtin/user shadowing, same provenance rule — so
there is one pattern for "pick a standard part", not two.

⚠ **PROVENANCE IS PART OF THE VALUE.** Every bundled figure is the NOMINAL
published spec for that series, and objectives of the same nominal name genuinely
differ between vintages and variants (a "Plan Fluor 10x" exists as DIC L/N1,
Ph1 DL, and plain, and the ELWD variants share names with the standard ones while
having wildly different WD). The number engraved on YOUR objective wins. So every
entry carries :data:`PROV_NOMINAL` / :data:`PROV_DATASHEET` / :data:`PROV_MEASURED`,
the UI shows it, and editing a value promotes it off "nominal". Entries confirmed
against the parts physically on a rig are shipped as ``datasheet``.

Persistence (ZERO GUI dependencies — json + dataclasses only):

    config/hardware/ME3B_general/objective_types/builtin/*.json   (read-only)
    config/hardware/ME3B_general/objective_types/user/<id>.json    (user entries)

Shared, not per-machine: a CFI Plan Apo VC 20x/0.75 is the same part on every rig.
Which objective sits in which nosepiece position is per-machine and lives in
``MicroscopeConfigStore`` — this is the parts list, not the assignment.
"""

from __future__ import annotations

import json
import logging
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_shared_path
from SupportClasses.OpticsRegistry import (
    DEFAULT_IMMERSION as _DEFAULT_IMMERSION,
    IMMERSION_N as _IMMERSION_N,
    clean_immersion,
    immersion_n,
)

logger = logging.getLogger(__name__)

_OBJECTIVE_DIR = resolve_shared_path("objective_types")
_DEFAULT_BUILTIN_DIR = _OBJECTIVE_DIR / "builtin"
_DEFAULT_USER_DIR = _OBJECTIVE_DIR / "user"

#: Where a figure came from. Ordered weakest → strongest. Same vocabulary as
#: ``FilterCubeStore`` so a UI can render both with one helper.
PROV_NOMINAL = "nominal"
PROV_DATASHEET = "datasheet"
PROV_MEASURED = "measured"
PROVENANCES = (PROV_NOMINAL, PROV_DATASHEET, PROV_MEASURED)

PROVENANCE_TEXT = {
    PROV_NOMINAL: "nominal for this objective series — confirm against the "
                  "numbers engraved on your objective before relying on it",
    PROV_DATASHEET: "from the objective's engraving or datasheet",
    PROV_MEASURED: "measured on this instrument",
}

#: Immersion medium → refractive index, for the DOF diffraction term.
#:
#: RE-EXPORTED, not redefined: the table lives in ``OpticsRegistry`` because that
#: module must import nothing from the repo (``ObjectiveCalibration`` and
#: ``MicroscopeConfigStore`` both import it), and a second copy here would be the
#: "two homes for one fact" trap this codebase keeps paying for. A test pins that
#: both names refer to the same object.
IMMERSION_N = _IMMERSION_N
DEFAULT_IMMERSION = _DEFAULT_IMMERSION

#: Plausibility bands. Generous on purpose — these exist to catch a unit slip
#: (WD typed in µm, or NA as a percentage), not to police exotic optics.
MIN_NA, MAX_NA = 0.01, 1.65
MIN_MAG, MAX_MAG = 0.5, 200.0
#: A dry objective can reach ~60 mm (macro); an oil 100x is ~0.13 mm.
MIN_WD_MM, MAX_WD_MM = 0.05, 60.0
#: 0 means "no coverslip correction" (most 4x, and dipping objectives).
MAX_COVERSLIP_MM = 2.0
MAX_FIELD_NUMBER_MM = 30.0


def safe_id(value) -> str:
    """Filesystem-safe token for an objective id (used as the JSON filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "objective"


def clean_na(value):
    """A plausible numerical aperture, or ``None``. Never raises."""
    try:
        na = float(value)
    except (TypeError, ValueError):
        return None
    return na if MIN_NA <= na <= MAX_NA else None


def clean_magnification(value):
    """A plausible magnification, or ``None``. Never raises."""
    try:
        m = float(value)
    except (TypeError, ValueError):
        return None
    return m if MIN_MAG <= m <= MAX_MAG else None


def clean_wd_mm(value):
    """A plausible working distance in mm, or ``None``. Never raises.

    ⚠ Returning None for an out-of-band value is the safe direction: an unknown
    WD makes ``plan_turret_change`` fall back to its tiny conservative budget and
    SAY so, whereas a value that survived a unit slip (17100 for 17.1 mm) would
    authorise a 17-metre focus excursion.
    """
    try:
        wd = float(value)
    except (TypeError, ValueError):
        return None
    return wd if MIN_WD_MM <= wd <= MAX_WD_MM else None


def clean_optional_mm(value, ceiling: float):
    """A non-negative length in mm within ``ceiling``, or ``None``."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if 0.0 <= v <= ceiling else None


def clean_provenance(value) -> str:
    """Coerce to a known provenance, defaulting to the WEAKEST claim."""
    key = str(value or "").strip().lower()
    return key if key in PROVENANCES else PROV_NOMINAL


@dataclass
class Objective:
    """One catalogue objective. Every optical figure may be ``None`` = unknown."""

    id: str = ""
    name: str = ""
    #: Vendor + optical series, e.g. "Nikon CFI Plan Fluor". Display only.
    series: str = ""
    magnification: Optional[float] = None
    numerical_aperture: Optional[float] = None
    working_distance_mm: Optional[float] = None
    immersion: str = DEFAULT_IMMERSION
    #: Design coverslip thickness in mm (0.17 for most; 0 = none/uncorrected).
    coverslip_mm: Optional[float] = None
    #: Field number in mm, for ``ObjectiveOptics``' geometry cross-check.
    field_number_mm: Optional[float] = None
    #: Vendor part number, when known. NEVER guessed — a wrong code would
    #: silently invalidate a parfocal offset via the product-code check in
    #: ``MicroscopeConfigStore.parfocal_offset_um``.
    product_code: str = ""
    provenance: str = PROV_NOMINAL
    notes: str = ""
    builtin: bool = False

    def __post_init__(self) -> None:
        self.id = safe_id(self.id or self.name)
        self.name = str(self.name or "").strip()
        self.series = str(self.series or "").strip()
        self.magnification = clean_magnification(self.magnification)
        self.numerical_aperture = clean_na(self.numerical_aperture)
        self.working_distance_mm = clean_wd_mm(self.working_distance_mm)
        self.immersion = clean_immersion(self.immersion)
        self.coverslip_mm = clean_optional_mm(self.coverslip_mm, MAX_COVERSLIP_MM)
        self.field_number_mm = clean_optional_mm(
            self.field_number_mm, MAX_FIELD_NUMBER_MM)
        self.product_code = str(self.product_code or "").strip()
        self.provenance = clean_provenance(self.provenance)
        self.notes = str(self.notes or "").strip()

    # ── derived ───────────────────────────────────────────────────────

    @property
    def immersion_n(self) -> float:
        return immersion_n(self.immersion)

    @property
    def label(self) -> str:
        """What the operator sees in a drop-down — the engraving, essentially."""
        bits = []
        if self.magnification:
            mag = (f"{self.magnification:g}x"
                   if self.magnification == int(self.magnification)
                   else f"{self.magnification:g}x")
            bits.append(f"{mag}/{self.numerical_aperture:g}"
                        if self.numerical_aperture else mag)
        head = " ".join(x for x in (self.series, " ".join(bits)) if x).strip()
        return head or self.name or self.id

    @property
    def has_optics(self) -> bool:
        """Enough to size a sweep AND bound a rotation."""
        return bool(self.numerical_aperture and self.working_distance_mm)

    def describe(self) -> str:
        bits = []
        if self.numerical_aperture:
            bits.append(f"NA {self.numerical_aperture:g}")
        if self.working_distance_mm:
            bits.append(f"WD {self.working_distance_mm:g} mm")
        if self.immersion != DEFAULT_IMMERSION:
            bits.append(self.immersion)
        if self.coverslip_mm:
            bits.append(f"coverslip {self.coverslip_mm:g} mm")
        if self.product_code:
            bits.append(self.product_code)
        bits.append(PROVENANCE_TEXT.get(self.provenance, self.provenance))
        return " · ".join(bits)

    # ── serialisation ─────────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Only what is known — an absent key means unknown, never zero."""
        out: dict = {"id": self.id, "name": self.name,
                     "provenance": self.provenance}
        for key in ("series", "product_code", "notes"):
            if getattr(self, key):
                out[key] = getattr(self, key)
        for key in ("magnification", "numerical_aperture",
                    "working_distance_mm", "coverslip_mm", "field_number_mm"):
            v = getattr(self, key)
            if v is not None:
                out[key] = v
        if self.immersion != DEFAULT_IMMERSION:
            out["immersion"] = self.immersion
        return out

    @classmethod
    def from_dict(cls, data: dict, *, builtin: bool = False) -> "Objective":
        d = dict(data or {})
        return cls(
            id=d.get("id") or d.get("name") or "",
            name=d.get("name") or "",
            series=d.get("series") or "",
            magnification=d.get("magnification"),
            numerical_aperture=d.get("numerical_aperture"),
            working_distance_mm=d.get("working_distance_mm"),
            immersion=d.get("immersion") or DEFAULT_IMMERSION,
            coverslip_mm=d.get("coverslip_mm"),
            field_number_mm=d.get("field_number_mm"),
            product_code=d.get("product_code") or "",
            provenance=d.get("provenance"),
            notes=d.get("notes") or "",
            builtin=bool(builtin),
        )

    def spec_entry(self) -> dict:
        """The per-slot record ``MicroscopeConfigStore`` stores for this pick.

        Mirrors ``FilterCube.optics_entry``: only the optical facts a consumer
        needs, plus enough provenance that a UI can never imply more than it has.
        """
        out: dict = {"provenance": self.provenance,
                     "immersion": self.immersion}
        for key in ("magnification", "numerical_aperture",
                    "working_distance_mm", "coverslip_mm", "field_number_mm"):
            v = getattr(self, key)
            if v is not None:
                out[key] = v
        if self.id:
            out["objective_id"] = self.id
        if self.product_code:
            out["product_code"] = self.product_code
        return out


class ObjectiveCatalogue:
    """Load/serve the objective catalogue. User entries shadow builtins by id."""

    def __init__(self, builtin_dir: Path = _DEFAULT_BUILTIN_DIR,
                 user_dir: Path = _DEFAULT_USER_DIR):
        self._builtin_dir = Path(builtin_dir)
        self._user_dir = Path(user_dir)
        self._items: dict[str, Objective] = {}
        self.reload()

    def _load_dir(self, directory: Path, builtin: bool) -> None:
        if not directory.is_dir():
            return
        for path in sorted(directory.glob("*.json")):
            try:
                raw = json.loads(path.read_text(encoding="utf-8"))
            except Exception as exc:
                logger.warning(f"ObjectiveCatalogue: skipping {path.name}: {exc}")
                continue
            # A builtin file may hold ONE objective or a list, so the shipped
            # catalogue is one readable file while a saved custom is its own.
            entries = raw if isinstance(raw, list) else [raw]
            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                try:
                    obj = Objective.from_dict(entry, builtin=builtin)
                except Exception as exc:
                    logger.warning(
                        f"ObjectiveCatalogue: bad entry in {path.name}: {exc}")
                    continue
                if not obj.id:
                    continue
                self._items[obj.id] = obj

    def reload(self) -> None:
        self._items = {}
        self._load_dir(self._builtin_dir, builtin=True)
        # User second: a same-id user entry SHADOWS the builtin.
        self._load_dir(self._user_dir, builtin=False)

    def get(self, objective_id) -> Optional[Objective]:
        return self._items.get(safe_id(objective_id))

    def all(self) -> list:
        """Every objective, ordered by magnification then label — the order an
        operator scans a nosepiece in."""
        return sorted(self._items.values(),
                      key=lambda o: (o.magnification or 0.0, o.label.lower()))

    def by_magnification(self, magnification) -> list:
        mag = clean_magnification(magnification)
        if mag is None:
            return []
        return [o for o in self.all() if o.magnification == mag]

    def find_by_name(self, name) -> Optional[Objective]:
        """Match a free-text slot label against the catalogue, or None.

        Exact id, then exact label/name, then a case/space-folded compare — the
        same tiering ``OpticsRegistry.find_slot`` uses, and for the same reason:
        typography folds, meaning does not. There is deliberately no fuzzy tier,
        so "20x" alone matches nothing (it names a magnification, not a part, and
        this rig's 20x/0.75 and a 20x/0.40 differ by 13x in working distance).
        """
        wanted = str(name or "").strip()
        if not wanted:
            return None
        hit = self.get(wanted)
        if hit is not None:
            return hit
        for obj in self._items.values():
            if wanted in (obj.name, obj.label):
                return obj
        folded = " ".join(wanted.casefold().split())
        matches = [o for o in self._items.values()
                   if " ".join((o.name or "").casefold().split()) == folded
                   or " ".join(o.label.casefold().split()) == folded]
        return matches[0] if len(matches) == 1 else None

    def save_user(self, objective: Objective) -> bool:
        """Persist a user objective (shadowing a builtin of the same id)."""
        if not objective.id:
            return False
        try:
            self._user_dir.mkdir(parents=True, exist_ok=True)
            path = self._user_dir / f"{safe_id(objective.id)}.json"
            tmp = path.with_name(path.name + ".tmp")
            tmp.write_text(json.dumps(objective.to_dict(), indent=2),
                           encoding="utf-8")
            import os
            os.replace(tmp, path)
        except Exception as exc:
            logger.error(f"ObjectiveCatalogue: could not save "
                         f"{objective.id!r}: {exc}")
            return False
        self.reload()
        logger.info(f"Objective saved to the catalogue: {objective.id}")
        return True

    def delete_user(self, objective_id) -> bool:
        """Remove a USER entry. Built-ins are never touched."""
        path = self._user_dir / f"{safe_id(objective_id)}.json"
        if not path.exists():
            return False
        try:
            path.unlink()
        except Exception as exc:
            logger.error(f"ObjectiveCatalogue: could not delete "
                         f"{objective_id!r}: {exc}")
            return False
        self.reload()
        return True


_catalogue: Optional[ObjectiveCatalogue] = None


def get_store() -> ObjectiveCatalogue:
    """Module-level singleton, matching ``FilterCubeStore.get_store``."""
    global _catalogue
    if _catalogue is None:
        _catalogue = ObjectiveCatalogue()
    return _catalogue


def reset_store() -> None:
    """Forget the singleton (tests)."""
    global _catalogue
    _catalogue = None
