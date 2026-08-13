"""On-disk library for v7.12 plate and rosette documents.

Mirrors ``PrintFileManager`` for CRUD shape and ``PlateTypeStore`` for
builtin/user shadowing and atomic writes.

**The filename is the document id, never its display name.** That single
decision is what makes renaming safe: the id is what reaches
``active_plate_key`` and therefore the eight per-plate stores (mosaics,
fluorescence mosaics, plate templates, well training, re-anchor features, plate
level sites, focus datum, and the taught-calibration archive). Under the old
name-is-the-key scheme a rename orphaned every one of them, and two such
orphans are still sitting in this machine's ``settings.json``.

``PlateDesign.save`` wrote non-atomically, so a crash mid-write corrupted the
plate. Every write here is tmp + ``os.replace``.

Directories, in precedence order:

* ``$MEBP_PLATES_DIR`` / ``$MEBP_ROSETTES_DIR`` — test and tooling override
* ``config/hardware/plates/v2``  ·  ``config/hardware/rosettes/{builtin,user}``

v2 plates live in their own directory rather than beside the v1 ``PlateDesign``
files, so the old designer keeps working untouched through the transition.
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from SupportClasses.PlateDocument import (
    FutureSchemaError, LegacySchemaError, PlateDocument, new_doc_id,
)
from SupportClasses.MachineConfig import resolve_shared_path

logger = logging.getLogger(__name__)

# Shared/portable catalogs — user-authored plate/rosette designs are meant to
# sync across every rig, unlike the per-machine stores that live under
# config/hardware/<machine-id>/.
_PLATES_ROOT = resolve_shared_path("plates")
_ROSETTES_ROOT = resolve_shared_path("rosettes")

DEFAULT_PLATES_DIR = _PLATES_ROOT / "v2"
DEFAULT_ROSETTES_USER_DIR = _ROSETTES_ROOT / "user"
DEFAULT_ROSETTES_BUILTIN_DIR = _ROSETTES_ROOT / "builtin"


@dataclass(frozen=True)
class DocSummary:
    """Enough to draw a library card without parsing the whole document."""
    id: str
    name: str
    kind: str
    path: Path
    modified: str
    well_count: int
    rosette_count: int
    boundary: tuple[float, float]
    builtin: bool = False
    plate_type_id: str = ""

    @property
    def label(self) -> str:
        return self.name or self.id


class PlateDocumentStore:
    """A collection of documents of one *kind* (``"plate"`` or ``"rosette"``)."""

    def __init__(self, kind: str = "plate",
                 user_dir: Optional[Path] = None,
                 builtin_dir: Optional[Path] = None):
        self.kind = kind
        if user_dir is not None:
            self._user_dir = Path(user_dir)
        elif kind == "rosette":
            self._user_dir = Path(os.environ.get("MEBP_ROSETTES_DIR")
                                  or DEFAULT_ROSETTES_USER_DIR)
        else:
            self._user_dir = Path(os.environ.get("MEBP_PLATES_DIR")
                                  or DEFAULT_PLATES_DIR)
        if builtin_dir is not None:
            self._builtin_dir = Path(builtin_dir)
        elif kind == "rosette" and os.environ.get("MEBP_ROSETTES_DIR") is None:
            self._builtin_dir = DEFAULT_ROSETTES_BUILTIN_DIR
        else:
            self._builtin_dir = None
        self._cache: dict[str, PlateDocument] = {}

    # ── Paths ─────────────────────────────────────────────────────

    @property
    def user_dir(self) -> Path:
        return self._user_dir

    def _path_for(self, doc_id: str, builtin: bool = False) -> Path:
        base = self._builtin_dir if builtin else self._user_dir
        return Path(base) / f"{doc_id}.json"

    def _dirs(self) -> list[tuple[Path, bool]]:
        out: list[tuple[Path, bool]] = []
        if self._builtin_dir is not None:
            out.append((self._builtin_dir, True))
        out.append((self._user_dir, False))
        return out

    # ── Read ──────────────────────────────────────────────────────

    def list(self) -> list[DocSummary]:
        """Every document, user entries shadowing built-ins of the same id.

        Best-effort per file: one malformed or legacy document is logged and
        skipped rather than emptying the library.
        """
        found: dict[str, DocSummary] = {}
        for directory, builtin in self._dirs():
            if not directory.exists():
                continue
            for path in sorted(directory.glob("*.json")):
                try:
                    doc = PlateDocument.load(path)
                except LegacySchemaError:
                    logger.debug("Skipping pre-v7.12 plate file %s", path.name)
                    continue
                except FutureSchemaError as exc:
                    logger.warning("Skipping %s: %s", path.name, exc)
                    continue
                except Exception as exc:
                    logger.warning("Skipping unreadable plate %s: %s",
                                   path.name, exc)
                    continue
                found[doc.meta.id] = self._summarize(doc, path, builtin)
        return sorted(found.values(), key=lambda s: s.label.lower())

    @staticmethod
    def _summarize(doc: PlateDocument, path: Path,
                   builtin: bool) -> DocSummary:
        wells = doc.evaluate()
        b = doc.boundary
        size = ((2 * b.radius_mm, 2 * b.radius_mm) if b.is_circle()
                else (b.width_mm, b.height_mm))
        return DocSummary(
            id=doc.meta.id, name=doc.meta.name, kind=doc.meta.kind, path=path,
            modified=doc.meta.modified, well_count=len(wells),
            rosette_count=sum(1 for w in wells if w.rosette is not None),
            boundary=size, builtin=builtin,
            plate_type_id=doc.meta.plate_type_id)

    def get(self, doc_id: str) -> Optional[PlateDocument]:
        if not doc_id:
            return None
        cached = self._cache.get(doc_id)
        if cached is not None:
            return cached
        for directory, _builtin in reversed(self._dirs()):   # user wins
            path = Path(directory) / f"{doc_id}.json"
            if not path.exists():
                continue
            try:
                doc = PlateDocument.load(path)
            except Exception as exc:
                logger.warning("Could not load plate %s: %s", doc_id, exc)
                return None
            self._cache[doc_id] = doc
            return doc
        return None

    def exists(self, doc_id: str) -> bool:
        return any(Path(d) / f"{doc_id}.json"
                   for d, _ in self._dirs()
                   if (Path(d) / f"{doc_id}.json").exists())

    def is_builtin(self, doc_id: str) -> bool:
        if self._builtin_dir is None:
            return False
        return (self._path_for(doc_id, builtin=True).exists()
                and not self._path_for(doc_id).exists())

    def find_by_name(self, name: str) -> Optional[DocSummary]:
        """First document whose display name matches, case-insensitively.

        Names are NOT unique — only ids are — so this is a convenience for
        resolving a legacy config, never an identity lookup.
        """
        want = (name or "").strip().lower()
        return next((s for s in self.list() if s.name.strip().lower() == want),
                    None)

    def thumbnail_wells(self, doc_id: str, max_n: int = 4096
                        ) -> list[tuple[float, float, float]]:
        """``(x, y, diameter)`` per well, for a library card. No compile."""
        doc = self.get(doc_id)
        if doc is None:
            return []
        return [(w.x, w.y, w.diameter_mm) for w in doc.evaluate()[:max_n]]

    # ── Write ─────────────────────────────────────────────────────

    def create(self, name: str,
               template: Optional[int | str] = None) -> PlateDocument:
        """A new document. *template* may be a standard well count, or a
        document id to fork (which is how "Duplicate to my plates" works)."""
        if self.kind == "rosette":
            doc = PlateDocument.new_rosette(name=name)
        elif isinstance(template, int) or (isinstance(template, str)
                                           and template.isdigit()):
            doc = PlateDocument.from_standard_format(int(template), name=name)
        elif isinstance(template, str) and template:
            src = self.get(template)
            if src is None:
                raise KeyError(f"No template document {template!r}")
            doc = self._fork(src, name)
        else:
            doc = PlateDocument.new_plate(name=name)
        self.save(doc)
        return doc

    def save(self, doc: PlateDocument) -> Path:
        if not doc.meta.id:
            doc.meta.id = new_doc_id(doc.meta.kind)
        doc.meta.rev = int(doc.meta.rev) + 1
        path = self._path_for(doc.meta.id)
        doc.save(path)
        self._cache[doc.meta.id] = doc
        logger.info("Saved %s '%s' (%s) → %s", doc.meta.kind, doc.meta.name,
                    doc.meta.id, path.name)
        return path

    def duplicate(self, doc_id: str, new_name: str = "") -> PlateDocument:
        """A standalone copy under a NEW id, so editing it cannot touch the
        original (and vice versa)."""
        src = self.get(doc_id)
        if src is None:
            raise KeyError(f"No document {doc_id!r}")
        copy = self._fork(src, new_name or f"{src.meta.name} copy")
        self.save(copy)
        return copy

    def _fork(self, src: PlateDocument, name: str) -> PlateDocument:
        copy = PlateDocument.from_dict(src.to_dict())
        copy.meta.id = new_doc_id(copy.meta.kind)
        copy.meta.name = name
        copy.meta.rev = 0                # save() bumps to 1
        copy.meta.legacy_keys = []       # a fork inherits no store history
        copy.meta.created = copy.meta.modified = ""
        return copy

    def rename(self, doc_id: str, new_name: str) -> bool:
        """Change the DISPLAY name only.

        No file moves and no store key changes — which is the entire reason the
        id is decoupled from the name. A duplicate display name is legal.
        """
        doc = self.get(doc_id)
        if doc is None or not (new_name or "").strip():
            return False
        if self.is_builtin(doc_id):
            logger.warning("Refusing to rename built-in document %s", doc_id)
            return False
        doc.meta.name = new_name.strip()
        self.save(doc)
        return True

    def delete(self, doc_id: str) -> bool:
        """Delete a USER document. Built-ins are never removed."""
        path = self._path_for(doc_id)
        if not path.exists():
            return False
        try:
            os.remove(path)
        except OSError as exc:
            logger.error("Could not delete plate %s: %s", doc_id, exc)
            return False
        self._cache.pop(doc_id, None)
        logger.info("Deleted %s %s", self.kind, doc_id)
        return True

    def adopt_legacy_key(self, doc_id: str, legacy_key: str) -> bool:
        """Record a store key this document used to be known by.

        Read paths consult these in order (see :func:`plate_store_keys`), so
        adopting a v1 plate's name preserves its taught calibration, mosaics and
        well training without migrating any of that data.
        """
        doc = self.get(doc_id)
        if doc is None or not legacy_key or legacy_key == doc_id:
            return False
        if legacy_key in doc.meta.legacy_keys:
            return False
        doc.meta.legacy_keys.append(legacy_key)
        self.save(doc)
        return True

    def invalidate(self, doc_id: str = "") -> None:
        """Drop cached documents so the next read hits disk."""
        if doc_id:
            self._cache.pop(doc_id, None)
        else:
            self._cache.clear()


# ── Per-plate store keys ──────────────────────────────────────────

def plate_store_keys(config) -> list[str]:
    """Keys to try, newest first, when reading a per-plate store.

    Always WRITE ``keys[0]``; READ through the list in order. That is what lets
    a plate adopt a stable id without orphaning the calibration, mosaics and
    training data taught under its old name — the first read after adoption
    migrates naturally.
    """
    keys: list[str] = []
    active = getattr(config, "active_plate_key", None)
    if active is not None:
        keys.append(str(active))

    doc_id = getattr(config, "plate_doc_id", "") or ""
    if doc_id:
        try:
            doc = get_plate_store().get(doc_id)
        except Exception:                                  # pragma: no cover
            doc = None
        if doc is not None:
            keys.extend(str(k) for k in doc.meta.legacy_keys)

    for extra in (getattr(config, "plate_name", ""),
                  getattr(config, "plate_format", None)):
        if extra:
            keys.append(str(extra))

    seen: set[str] = set()
    return [k for k in keys if k and not (k in seen or seen.add(k))]


# ── Plate footprint ───────────────────────────────────────────────

def a1_well_offset_mm(doc) -> tuple[float, float]:
    """Where the A1 WELL sits in *doc*'s own storage frame, in mm.

    Two plate frames are in play and they are NOT the same:

    * the document's **storage frame**, whose origin is the boundary's A1
      datum — what ``evaluate()`` and ``boundary.extent_a1()`` report;
    * the compiled plate's **A1-well frame**, whose origin is the A1 well —
      what ``WellPlate.get_well_position`` and a mosaic's stored plate frame
      report.

    On a standard plate the A1 well sits exactly on the datum and the two
    coincide, which is why mixing them goes unnoticed until someone authors a
    pattern away from the datum. This is the offset between them; add it to an
    A1-well-frame coordinate to land in the storage frame.

    Returns ``(0.0, 0.0)`` when it cannot be determined, which is the
    coincident case and therefore the safe default.
    """
    try:
        plate = doc.compile(loader=get_rosette_store().get)
        for well in doc.evaluate():
            try:
                cx, cy = plate.get_well_position(well.name)
            except KeyError:                               # pragma: no cover
                continue
            return (float(well.x - cx), float(well.y - cy))
    except Exception as exc:                               # pragma: no cover
        logger.debug("A1-well offset unavailable: %s", exc)
    return (0.0, 0.0)


def _doc_footprint_extent_mm(doc) -> Optional[tuple[float, float, float, float]]:
    """Footprint of *doc*, expressed in the COMPILED plate's A1-well frame.

    ``PlateBoundary.extent_a1`` is measured from the document's **A1 reference
    point** (the ``a1_x_mm`` / ``a1_y_mm`` datum on the boundary), while every
    ``WellPlate`` coordinate is measured from the **A1 well**. On a standard
    plate those coincide, but on a parametric plate the author places the first
    pattern wherever they like — on the operator's 6-insert plate the grid
    anchor sits 7.39 / 9.89 mm from the A1 datum — so handing a renderer the
    raw ``extent_a1`` shifts the outline by exactly that much relative to the
    wells drawn inside it.

    The offset is recovered empirically, by asking the compiled plate where one
    evaluated well ended up, rather than re-deriving whatever anchor
    ``compile()`` chose. That keeps the two in step no matter which well
    becomes A1 (a rosette-only document has no grid to name A1 from).
    """
    extent = doc.boundary.extent_a1()
    try:
        plate = doc.compile(loader=get_rosette_store().get)
        for well in doc.evaluate():
            try:
                cx, cy = plate.get_well_position(well.name)
            except KeyError:                               # pragma: no cover
                continue
            dx, dy = well.x - cx, well.y - cy
            return (extent[0] - dx, extent[1] - dy,
                    extent[2] - dx, extent[3] - dy)
    except Exception as exc:                               # pragma: no cover
        logger.debug("Footprint frame shift unavailable: %s", exc)
    return extent


def plate_footprint_extent_mm(key) -> Optional[tuple[float, float, float, float]]:
    """v7.12: the plate's physical outline as ``(x0, y0, x1, y1)`` mm, or None.

    Coordinates are in the **A1-well-relative frame** (+X right, +Y down) — the
    frame ``WellPlate.get_well_position`` reports — so a renderer can draw the
    outline and the wells together without converting. A standard 24-well plate
    reports ``(-17.05, -13.67, 110.71, 71.81)``.

    The footprint is authored on the ``PlateDocument`` (``boundary``) and is
    dropped at ``compile()`` time — ``WellPlate`` has nowhere to put it — so
    every view fell back to a box hugging the wells. Rather than duplicate the
    footprint onto the compiled plate (two homes for one fact), callers resolve
    it here from the document, by the same key
    ``WellPlate._load_plate_document`` uses.

    *key* may be a plate-document id, a display name, a ``"custom:<name>"``
    tag, a plate-type id, a standard format int, or a stringified int. Returns
    None when nothing resolves, so callers keep whatever fallback they already
    had — this must never be the reason a plate fails to draw.
    """
    if key is None or isinstance(key, bool):
        return None

    # Standard formats: one home for the ANSI/SLAS footprint.
    fmt: Optional[int] = None
    if isinstance(key, int):
        fmt = int(key)
    elif isinstance(key, str) and key.isdigit():
        fmt = int(key)

    name = str(key)
    if fmt is None:
        if name.startswith("custom:"):
            name = name[7:]
        # A plate TYPE (product overlay) is a thin layer on a base format.
        try:
            from SupportClasses.PlateTypeStore import get_store as _pt_store
            ptype = _pt_store().get(name)
        except Exception:                                  # pragma: no cover
            ptype = None
        base = int(getattr(ptype, "base_format", 0) or 0) \
            if ptype is not None else 0
        if base:
            fmt = base

    if fmt is not None:
        try:
            return _doc_footprint_extent_mm(
                PlateDocument.from_standard_format(fmt))
        except Exception as exc:                           # pragma: no cover
            logger.debug("No standard footprint for %s: %s", fmt, exc)
            return None

    try:
        store = get_plate_store()
        doc = store.get(name)
        if doc is None:
            summary = store.find_by_name(name)
            doc = store.get(summary.id) if summary else None
        if doc is not None:
            return _doc_footprint_extent_mm(doc)
    except Exception as exc:                               # pragma: no cover
        logger.debug("No plate-document footprint for %r: %s", key, exc)
    return None


# ── Singletons ────────────────────────────────────────────────────

_plate_store: Optional[PlateDocumentStore] = None
_rosette_store: Optional[PlateDocumentStore] = None


def get_plate_store() -> PlateDocumentStore:
    global _plate_store
    if _plate_store is None:
        _plate_store = PlateDocumentStore(kind="plate")
    return _plate_store


def get_rosette_store() -> PlateDocumentStore:
    global _rosette_store
    if _rosette_store is None:
        _rosette_store = PlateDocumentStore(kind="rosette")
    return _rosette_store


def reset_stores() -> None:
    """Drop the singletons — for tests that redirect the directories."""
    global _plate_store, _rosette_store
    _plate_store = _rosette_store = None
