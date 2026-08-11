"""v7.12 parametric plate / rosette document.

The successor to :mod:`SupportClasses.PlateDesign`. Both exist during the
transition; nothing here imports that module and nothing there imports this one.

WHAT IS DIFFERENT, AND WHY
==========================

**1. Pattern wells are DERIVED, not stored.**
``PlateDesign`` materialised every grid/ring well as a real ``Well`` + ``Point``
entity owned by a ``Group``. Changing the row count meant ``rebuild_group()``,
which deleted every member and every constraint touching them — silently taking
each well's rosette, lock, custom name and entity id with it.

Here a pattern owns exactly one real, constrainable anchor ``Point``, a
parameter block, and a sparse ``overrides`` dict keyed by a STABLE MEMBER KEY
(``"r0"``, ``"g3_5"``, …). Growing a ring from 6 to 8 leaves ``r0``–``r5``
untouched; shrinking retains their overrides dormant so growing back restores
them. There is nothing to rebuild, so there is nothing to lose.

A 384-well plate is therefore ONE ``GridPattern`` (~2 KB on disk, 2 solver
variables) rather than 768 entities.

**2. Member positions are not solver variables.**
A member's position is a pure function of ``(anchor, params, index)``. Patterns
cannot shear, and a dimension on a member is still satisfiable — the solver
moves the whole pattern by its anchor. That is exactly "pick a seed point for
the first well and dimension that".

**3. Origin is presentation, never geometry.**
``origin_ref`` selects which datum the UI measures from and reports in. Stored
coordinates NEVER change, so switching it leaves ``to_dict()`` byte-identical.
This is deliberate: if the authoring origin could leak into the compiled frame,
every well would land ~0.9 well-pitch off while Go-To-A1 still looked correct
(A1 being the anchor) — the hardest possible bug to notice on a live rig.

**4. Rosettes are library documents, referenced live.**
``PlateDesign`` nested a whole rosette design inside each well. Here a well
carries a ``RosettePlacement`` naming a rosette document by STABLE ID, resolved
at compile time. Editing a rosette updates every plate using it (the operator's
explicit choice), and renaming one breaks nothing because the link is by id.

    ⚠ Because a rosette edit MOVES sub-well positions on plates that may already
    be taught, :meth:`PlateDocument.rosette_revision_fingerprint` exists to be
    folded into the calibration fingerprint so a changed rosette forces a
    re-teach rather than driving to stale coordinates.

THE COMPILE CONTRACT — DO NOT CHANGE
====================================
``compile()`` must keep emitting ``WellInfo.x/y`` in millimetres **relative to
the A1 well centre**, +X toward increasing column, +Y toward increasing row
(Y-down). ``WellPlate.get_all_positions_from_a1`` and every calibration, mosaic
and jog consumer depend on it, as do the taught positions already persisted in
``settings.calibration_by_plate``.

Sub-wells keep the ``"<parent>.<sub>"`` naming protocol — ``HardwareConfig.
_drop_redundant_parents`` and ``mosaic_well_mapping_dialog.main_well_names``
both parse that dot.

Pure Python + numpy. No Qt.
"""
from __future__ import annotations

import json
import logging
import math
import os
import re
import uuid
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Callable, ClassVar, Iterable, Optional

from SupportClasses.WellPlate import ROW_LABELS, WellInfo, WellPlate

logger = logging.getLogger(__name__)

SCHEMA_NAME = "mebp.plate-document"
SCHEMA_VERSION = 2          # int, not a string: "1.10" < "1.9" lexicographically

EntityId = int
MemberKey = str
#: ``(entity_id, member_key)``. ``member_key == ""`` means the entity itself.
Ref = tuple[EntityId, MemberKey]

# ANSI / SLAS footprint.
DEFAULT_FOOTPRINT_X_MM = 127.76
DEFAULT_FOOTPRINT_Y_MM = 85.48

_ID_SAFE_RE = re.compile(r"[^A-Za-z0-9_.-]")


class LegacySchemaError(ValueError):
    """Raised for a pre-v7.12 (``PlateDesign``) file."""


class FutureSchemaError(ValueError):
    """Raised for a file written by a NEWER build.

    Loading it optimistically would silently drop whatever it added.
    """


def new_doc_id(kind: str) -> str:
    """Opaque, filesystem-safe, collision-free document id.

    Generated ONCE at creation and never re-derived from the display name — a
    rename must not move the file or change any store key.

    The character set matters: every per-plate store sanitises its key with
    ``re.sub(r"[^A-Za-z0-9_.-]", "_", key)`` before using it as a filename, and
    that map is many-to-one. Two ids differing only in punctuation would collide
    on disk while their metadata stayed distinct — one plate's mosaic silently
    overwriting another's. A hex id makes the sanitiser the identity function.
    """
    prefix = "ros" if kind == "rosette" else "plt"
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


def is_store_key_safe(doc_id: str) -> bool:
    """True when *doc_id* survives every store's key sanitiser unchanged."""
    return bool(doc_id) and _ID_SAFE_RE.sub("_", doc_id) == doc_id


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _r6(v: float) -> float:
    """Round for serialization; a solve returning 14.379999999999999 should not
    produce a diff."""
    return round(float(v), 6)


# ═══════════════════════════════════════════════════════════════════
# Boundary + origin
# ═══════════════════════════════════════════════════════════════════

class OriginRef(str, Enum):
    """Which datum the UI dimensions from and reports in.

    Presentation only — see the module docstring. ``A1`` reproduces the legacy
    designer's readout.
    """
    A1 = "a1"
    BOTTOM_LEFT = "bottom_left"
    BOTTOM_RIGHT = "bottom_right"
    TOP_LEFT = "top_left"
    TOP_RIGHT = "top_right"
    CENTER = "center"


class BoundaryKind(str, Enum):
    RECT = "rect"       # a plate footprint
    CIRCLE = "circle"   # a rosette bore


#: Datum names a dimension may reference.
DATUMS: tuple[str, ...] = (
    "edge_left", "edge_right", "edge_top", "edge_bottom",
    "axis_v", "axis_h", "origin", "bore_wall",
)


@dataclass
class PlateBoundary:
    """The plate footprint (or the rosette bore) and where A1 sits inside it.

    ``a1_x_mm`` / ``a1_y_mm`` place A1 relative to the footprint's TOP-LEFT
    corner, +X right, +Y down — matching the runtime frame.

    v7.12 splits a field that used to do two jobs. ``PlateDesign.a1_offset_x``
    was simultaneously the stage-facing ANSI-corner→A1 offset consumed by
    ``WellPlate.get_a1_from_plate_center`` AND the design-frame location of the
    left/top edges used by the dimension residuals. Two features, one variable,
    no test covering both. Here the footprint placement is authored and the
    stage-facing offset is DERIVED from it.
    """
    kind: str = BoundaryKind.RECT
    width_mm: float = DEFAULT_FOOTPRINT_X_MM
    height_mm: float = DEFAULT_FOOTPRINT_Y_MM
    radius_mm: float = 0.0
    a1_x_mm: float = 14.38
    a1_y_mm: float = 11.24
    origin_ref: str = OriginRef.BOTTOM_LEFT

    def is_circle(self) -> bool:
        return self.kind == BoundaryKind.CIRCLE

    def extent_a1(self) -> tuple[float, float, float, float]:
        """Footprint as ``(x_min, y_min, x_max, y_max)`` in the A1-relative
        storage frame (+X right, +Y down)."""
        if self.is_circle():
            r = float(self.radius_mm)
            return (-r, -r, r, r)
        return (-self.a1_x_mm,
                -self.a1_y_mm,
                self.width_mm - self.a1_x_mm,
                self.height_mm - self.a1_y_mm)

    def datum_value(self, datum: str) -> Optional[float]:
        """Scalar for *datum* in the storage frame, or None if not applicable.

        Edges return a coordinate; ``bore_wall`` returns a radius.
        """
        x0, y0, x1, y1 = self.extent_a1()
        if datum == "edge_left":
            return x0
        if datum == "edge_right":
            return x1
        if datum == "edge_top":
            return y0
        if datum == "edge_bottom":
            return y1
        if datum == "axis_v":
            return (x0 + x1) / 2.0
        if datum == "axis_h":
            return (y0 + y1) / 2.0
        if datum == "origin":
            return 0.0
        if datum == "bore_wall":
            return float(self.radius_mm) if self.is_circle() else None
        return None

    def origin_offset(self) -> tuple[float, float]:
        """Storage-frame coordinate of the chosen display origin.

        ``display = storage - origin_offset`` (see
        :meth:`PlateDocument.to_display`).
        """
        ref = self.origin_ref
        if ref == OriginRef.A1 or self.is_circle():
            return (0.0, 0.0)
        x0, y0, x1, y1 = self.extent_a1()
        if ref == OriginRef.BOTTOM_LEFT:
            return (x0, y1)
        if ref == OriginRef.BOTTOM_RIGHT:
            return (x1, y1)
        if ref == OriginRef.TOP_LEFT:
            return (x0, y0)
        if ref == OriginRef.TOP_RIGHT:
            return (x1, y0)
        if ref == OriginRef.CENTER:
            return ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        return (0.0, 0.0)

    def display_y_is_up(self) -> bool:
        """True when the display frame reports +Y upward.

        A bottom-referenced origin means "how far ABOVE the bottom edge", which
        an operator expects to be positive going up — while storage is Y-down.
        """
        return self.origin_ref in (OriginRef.BOTTOM_LEFT,
                                   OriginRef.BOTTOM_RIGHT)

    def contains(self, x: float, y: float, r: float = 0.0) -> bool:
        if self.is_circle():
            return math.hypot(x, y) + r <= self.radius_mm + 1e-9
        x0, y0, x1, y1 = self.extent_a1()
        return (x0 - 1e-9 <= x - r and x + r <= x1 + 1e-9
                and y0 - 1e-9 <= y - r and y + r <= y1 + 1e-9)

    def to_dict(self) -> dict:
        d: dict[str, Any] = {"kind": _enum_str(self.kind)}
        if self.is_circle():
            d["radius_mm"] = _r6(self.radius_mm)
        else:
            d["width_mm"] = _r6(self.width_mm)
            d["height_mm"] = _r6(self.height_mm)
            d["a1_x_mm"] = _r6(self.a1_x_mm)
            d["a1_y_mm"] = _r6(self.a1_y_mm)
        d["origin_ref"] = _enum_str(self.origin_ref)
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "PlateBoundary":
        kind = data.get("kind", BoundaryKind.RECT)
        if kind == BoundaryKind.CIRCLE:
            return cls(kind=BoundaryKind.CIRCLE,
                       radius_mm=float(data.get("radius_mm", 7.8)),
                       origin_ref=OriginRef.CENTER)
        return cls(
            kind=BoundaryKind.RECT,
            width_mm=float(data.get("width_mm", DEFAULT_FOOTPRINT_X_MM)),
            height_mm=float(data.get("height_mm", DEFAULT_FOOTPRINT_Y_MM)),
            a1_x_mm=float(data.get("a1_x_mm", 14.38)),
            a1_y_mm=float(data.get("a1_y_mm", 11.24)),
            origin_ref=data.get("origin_ref", OriginRef.BOTTOM_LEFT),
        )


# ═══════════════════════════════════════════════════════════════════
# Well styling / naming / overrides
# ═══════════════════════════════════════════════════════════════════

@dataclass
class WellStyle:
    """Geometry shared by every member of a pattern (or one standalone well)."""
    diameter_mm: float = 6.35
    well_depth_mm: float = 10.67
    bottom_z_offset: float = 0.0
    rim_height_mm: float = 0.0
    ink_z_mm: Optional[float] = None
    #: Provenance only — the stamped values above are authoritative.
    well_type_id: Optional[str] = None

    def to_dict(self) -> dict:
        d = {"diameter_mm": _r6(self.diameter_mm),
             "well_depth_mm": _r6(self.well_depth_mm)}
        if self.bottom_z_offset:
            d["bottom_z_offset"] = _r6(self.bottom_z_offset)
        if self.rim_height_mm:
            d["rim_height_mm"] = _r6(self.rim_height_mm)
        if self.ink_z_mm is not None:
            d["ink_z_mm"] = _r6(self.ink_z_mm)
        if self.well_type_id:
            d["well_type_id"] = self.well_type_id
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "WellStyle":
        return cls(
            diameter_mm=float(data.get("diameter_mm", 6.35)),
            well_depth_mm=float(data.get("well_depth_mm", 10.67)),
            bottom_z_offset=float(data.get("bottom_z_offset", 0.0)),
            rim_height_mm=float(data.get("rim_height_mm", 0.0)),
            ink_z_mm=(None if data.get("ink_z_mm") is None
                      else float(data["ink_z_mm"])),
            well_type_id=data.get("well_type_id") or None,
        )


class NamingScheme(str, Enum):
    ANSI = "ansi"          # A1, A2, … — plates
    LETTERS = "letters"    # a, b, c, … aa — rosette sub-wells
    NUMBERS = "numbers"    # <prefix>1, <prefix>2, …
    MANUAL = "manual"      # overrides only, with an auto fallback


@dataclass
class NamingSpec:
    scheme: str = NamingScheme.ANSI
    prefix: str = ""
    start_row: int = 0
    start_col: int = 1

    def to_dict(self) -> dict:
        d: dict[str, Any] = {}
        if self.scheme != NamingScheme.ANSI:
            d["scheme"] = _enum_str(self.scheme)
        if self.prefix:
            d["prefix"] = self.prefix
        if self.start_row:
            d["start_row"] = self.start_row
        if self.start_col != 1:
            d["start_col"] = self.start_col
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "NamingSpec":
        return cls(scheme=data.get("scheme", NamingScheme.ANSI),
                   prefix=data.get("prefix", ""),
                   start_row=int(data.get("start_row", 0)),
                   start_col=int(data.get("start_col", 1)))


def letter_label(index: int) -> str:
    """0 → 'a', 25 → 'z', 26 → 'aa' (spreadsheet-style, lowercase)."""
    out = ""
    n = index + 1
    while n > 0:
        n, rem = divmod(n - 1, 26)
        out = chr(ord("a") + rem) + out
    return out


def ansi_label(row: int, col: int) -> str:
    """(0, 0) → 'A1'. Past row P the row falls back to ``R<n>``."""
    if 0 <= row < len(ROW_LABELS):
        return f"{ROW_LABELS[row]}{col + 1}"
    return f"R{row + 1}C{col + 1}"


@dataclass
class RosettePlacement:
    """A live reference to a rosette document, seated in one well.

    Resolved at compile time via a loader, so editing the rosette updates every
    plate that uses it. The link is by ID, so renaming the rosette is safe.
    """
    rosette_id: str = ""
    rotation_deg: float = 0.0
    #: Display-only cache so the UI can label a placement without a store hit.
    rosette_name: str = ""

    def to_dict(self) -> dict:
        d: dict[str, Any] = {"rosette_id": self.rosette_id}
        if self.rotation_deg:
            d["rotation_deg"] = _r6(self.rotation_deg)
        if self.rosette_name:
            d["rosette_name"] = self.rosette_name
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "RosettePlacement":
        return cls(rosette_id=data.get("rosette_id", ""),
                   rotation_deg=float(data.get("rotation_deg", 0.0)),
                   rosette_name=data.get("rosette_name", ""))


@dataclass
class MemberOverride:
    """Per-member deviations from a pattern's defaults.

    Keyed by a stable member key, so a parameter change preserves them. This is
    the whole reason ``rebuild_group``'s data loss cannot recur.
    """
    name: Optional[str] = None
    diameter_mm: Optional[float] = None
    well_depth_mm: Optional[float] = None
    bottom_z_offset: Optional[float] = None
    rim_height_mm: Optional[float] = None
    ink_z_mm: Optional[float] = None
    well_type_id: Optional[str] = None
    rosette: Optional[RosettePlacement] = None
    #: Nudge in the pattern's local frame — one member off-lattice, still a member.
    offset_mm: Optional[tuple[float, float]] = None
    suppressed: bool = False

    def is_empty(self) -> bool:
        return (self.name is None and self.diameter_mm is None
                and self.well_depth_mm is None
                and self.bottom_z_offset is None
                and self.rim_height_mm is None and self.ink_z_mm is None
                and self.well_type_id is None and self.rosette is None
                and self.offset_mm is None and not self.suppressed)

    def to_dict(self) -> dict:
        d: dict[str, Any] = {}
        for f_name in ("name", "well_type_id"):
            v = getattr(self, f_name)
            if v is not None:
                d[f_name] = v
        for f_name in ("diameter_mm", "well_depth_mm", "bottom_z_offset",
                       "rim_height_mm", "ink_z_mm"):
            v = getattr(self, f_name)
            if v is not None:
                d[f_name] = _r6(v)
        if self.rosette is not None:
            d["rosette"] = self.rosette.to_dict()
        if self.offset_mm is not None:
            d["offset_mm"] = [_r6(self.offset_mm[0]), _r6(self.offset_mm[1])]
        if self.suppressed:
            d["suppressed"] = True
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "MemberOverride":
        off = data.get("offset_mm")
        ros = data.get("rosette")
        return cls(
            name=data.get("name"),
            diameter_mm=_opt_float(data.get("diameter_mm")),
            well_depth_mm=_opt_float(data.get("well_depth_mm")),
            bottom_z_offset=_opt_float(data.get("bottom_z_offset")),
            rim_height_mm=_opt_float(data.get("rim_height_mm")),
            ink_z_mm=_opt_float(data.get("ink_z_mm")),
            well_type_id=data.get("well_type_id"),
            rosette=RosettePlacement.from_dict(ros) if ros else None,
            offset_mm=(float(off[0]), float(off[1])) if off else None,
            suppressed=bool(data.get("suppressed", False)),
        )


def _opt_float(v) -> Optional[float]:
    return None if v is None else float(v)


def _enum_str(v) -> str:
    """Serialize an enum member (or a plain string) as its wire value.

    NOT ``str(v)``: since Python 3.11 a ``(str, Enum)`` member stringifies as
    ``'OriginRef.BOTTOM_LEFT'``, not ``'bottom_left'``. These fields are typed
    ``str`` and may hold either a member (set in code) or a plain string (read
    from JSON), so both have to land on the same wire value.
    """
    return v.value if isinstance(v, Enum) else str(v)


# ═══════════════════════════════════════════════════════════════════
# Entities
# ═══════════════════════════════════════════════════════════════════

@dataclass
class Entity:
    id: EntityId = 0
    construction: bool = False
    fixed: bool = False
    TYPE: ClassVar[str] = "Entity"

    def to_dict(self) -> dict:
        d: dict[str, Any] = {"type": self.TYPE, "id": self.id}
        if self.construction:
            d["construction"] = True
        if self.fixed:
            d["fixed"] = True
        return d


@dataclass
class Point(Entity):
    x: float = 0.0
    y: float = 0.0
    TYPE: ClassVar[str] = "Point"

    def to_dict(self) -> dict:
        d = super().to_dict()
        d.update(x=_r6(self.x), y=_r6(self.y))
        return d


@dataclass
class Line(Entity):
    p1: EntityId = 0
    p2: EntityId = 0
    TYPE: ClassVar[str] = "Line"

    def to_dict(self) -> dict:
        d = super().to_dict()
        d.update(p1=self.p1, p2=self.p2)
        return d


@dataclass
class Well(Entity):
    """A standalone well: a named wrapper around a centre Point."""
    name: str = "A1"
    center: EntityId = 0
    style: WellStyle = field(default_factory=WellStyle)
    rosette: Optional[RosettePlacement] = None
    TYPE: ClassVar[str] = "Well"

    def to_dict(self) -> dict:
        d = super().to_dict()
        d.update(name=self.name, center=self.center, style=self.style.to_dict())
        if self.rosette is not None:
            d["rosette"] = self.rosette.to_dict()
        return d


@dataclass
class PatternFeature(Entity):
    """Base for generative features. Owns one anchor Point."""
    name: str = "pattern"
    anchor: EntityId = 0
    style: WellStyle = field(default_factory=WellStyle)
    naming: NamingSpec = field(default_factory=NamingSpec)
    overrides: dict[MemberKey, MemberOverride] = field(default_factory=dict)
    TYPE: ClassVar[str] = "PatternFeature"

    def member_keys(self) -> list[MemberKey]:
        raise NotImplementedError

    def local_offsets(self) -> list[tuple[MemberKey, float, float]]:
        """``(key, dx, dy)`` per member, relative to the anchor, storage frame."""
        raise NotImplementedError

    def to_dict(self) -> dict:
        d = super().to_dict()
        d.update(name=self.name, anchor=self.anchor,
                 style=self.style.to_dict())
        nm = self.naming.to_dict()
        if nm:
            d["naming"] = nm
        ov = {k: o.to_dict() for k, o in sorted(self.overrides.items())
              if not o.is_empty()}
        if ov:
            d["overrides"] = ov
        return d


@dataclass
class RingPattern(PatternFeature):
    """N wells on a circle about the anchor, plus an optional centre well.

    ``start_angle_deg`` is measured with **0° = +Y** in the storage frame and
    increases in the ``direction`` sense. The +Y convention matches
    ``RosetteInsert``'s polar model, which the legacy rosette code already used.
    """
    count: int = 6
    ring_diameter_mm: float = 8.0
    start_angle_deg: float = 0.0
    sweep_deg: float = 360.0
    direction: int = 1          # +1 / -1
    center_well: bool = False
    TYPE: ClassVar[str] = "RingPattern"

    def member_keys(self) -> list[MemberKey]:
        keys = ["c"] if self.center_well else []
        keys += [f"r{i}" for i in range(max(0, int(self.count)))]
        return keys

    def local_offsets(self) -> list[tuple[MemberKey, float, float]]:
        out: list[tuple[MemberKey, float, float]] = []
        if self.center_well:
            out.append(("c", 0.0, 0.0))
        n = max(0, int(self.count))
        if n == 0:
            return out
        full = abs(self.sweep_deg) >= 360.0 - 1e-9
        steps = n if full else max(n - 1, 1)
        radius = self.ring_diameter_mm / 2.0
        sign = 1 if self.direction >= 0 else -1
        for i in range(n):
            theta = math.radians(
                self.start_angle_deg + sign * self.sweep_deg * i / steps)
            out.append((f"r{i}",
                        radius * math.sin(theta),
                        radius * math.cos(theta)))
        return out

    def to_dict(self) -> dict:
        d = super().to_dict()
        d.update(count=int(self.count),
                 ring_diameter_mm=_r6(self.ring_diameter_mm))
        if self.start_angle_deg:
            d["start_angle_deg"] = _r6(self.start_angle_deg)
        if abs(self.sweep_deg - 360.0) > 1e-9:
            d["sweep_deg"] = _r6(self.sweep_deg)
        if self.direction != 1:
            d["direction"] = int(self.direction)
        if self.center_well:
            d["center_well"] = True
        return d


@dataclass
class GridPattern(PatternFeature):
    """rows x cols lattice seeded at the anchor (which IS member ``g0_0``).

    ``row_dir = +1`` marches rows toward +Y, i.e. DOWN the plate in the storage
    frame — the ANSI default (row B below row A).
    """
    rows: int = 8
    cols: int = 12
    pitch_x_mm: float = 9.0
    pitch_y_mm: float = 9.0
    rotation_deg: float = 0.0
    col_dir: int = 1
    row_dir: int = 1
    stagger_x_mm: float = 0.0
    TYPE: ClassVar[str] = "GridPattern"

    def member_keys(self) -> list[MemberKey]:
        return [f"g{r}_{c}"
                for r in range(max(0, int(self.rows)))
                for c in range(max(0, int(self.cols)))]

    def local_offsets(self) -> list[tuple[MemberKey, float, float]]:
        out: list[tuple[MemberKey, float, float]] = []
        rot = math.radians(self.rotation_deg)
        cos_t, sin_t = math.cos(rot), math.sin(rot)
        cdir = 1 if self.col_dir >= 0 else -1
        rdir = 1 if self.row_dir >= 0 else -1
        for r in range(max(0, int(self.rows))):
            for c in range(max(0, int(self.cols))):
                lx = cdir * c * self.pitch_x_mm
                if r % 2:
                    lx += self.stagger_x_mm
                ly = rdir * r * self.pitch_y_mm
                out.append((f"g{r}_{c}",
                            lx * cos_t - ly * sin_t,
                            lx * sin_t + ly * cos_t))
        return out

    def to_dict(self) -> dict:
        d = super().to_dict()
        d.update(rows=int(self.rows), cols=int(self.cols),
                 pitch_x_mm=_r6(self.pitch_x_mm),
                 pitch_y_mm=_r6(self.pitch_y_mm))
        if self.rotation_deg:
            d["rotation_deg"] = _r6(self.rotation_deg)
        if self.col_dir != 1:
            d["col_dir"] = int(self.col_dir)
        if self.row_dir != 1:
            d["row_dir"] = int(self.row_dir)
        if self.stagger_x_mm:
            d["stagger_x_mm"] = _r6(self.stagger_x_mm)
        return d


_ENTITY_TYPES: dict[str, type] = {
    "Point": Point, "Line": Line, "Well": Well,
    "RingPattern": RingPattern, "GridPattern": GridPattern,
}


def _entity_from_dict(data: dict) -> Entity:
    cls = _ENTITY_TYPES.get(data.get("type", ""))
    if cls is None:
        raise ValueError(f"Unknown entity type {data.get('type')!r}")
    common = dict(id=int(data.get("id", 0)),
                  construction=bool(data.get("construction", False)),
                  fixed=bool(data.get("fixed", False)))
    if cls is Point:
        return Point(**common, x=float(data.get("x", 0.0)),
                     y=float(data.get("y", 0.0)))
    if cls is Line:
        return Line(**common, p1=int(data.get("p1", 0)),
                    p2=int(data.get("p2", 0)))
    if cls is Well:
        ros = data.get("rosette")
        return Well(**common, name=data.get("name", "A1"),
                    center=int(data.get("center", 0)),
                    style=WellStyle.from_dict(data.get("style", {})),
                    rosette=RosettePlacement.from_dict(ros) if ros else None)

    pat_common = dict(
        **common,
        name=data.get("name", "pattern"),
        anchor=int(data.get("anchor", 0)),
        style=WellStyle.from_dict(data.get("style", {})),
        naming=NamingSpec.from_dict(data.get("naming", {})),
        # v7.9.1: drop an override under an EMPTY member key. A member key
        # identifies one generated well ("g0_0", "r3"); "" is the pattern
        # itself and can never match a member, so such an entry is invisible
        # dead weight — a phantom rosette that renders nowhere and inflates
        # the placement count. `place_rosette` now refuses to create one
        # (that was the "it won't let me place it" bug), and this heals the
        # files already written by the version that did.
        overrides={k: MemberOverride.from_dict(v)
                   for k, v in (data.get("overrides") or {}).items() if k},
    )
    if cls is RingPattern:
        return RingPattern(
            **pat_common,
            count=int(data.get("count", 6)),
            ring_diameter_mm=float(data.get("ring_diameter_mm", 8.0)),
            start_angle_deg=float(data.get("start_angle_deg", 0.0)),
            sweep_deg=float(data.get("sweep_deg", 360.0)),
            direction=int(data.get("direction", 1)),
            center_well=bool(data.get("center_well", False)))
    return GridPattern(
        **pat_common,
        rows=int(data.get("rows", 8)), cols=int(data.get("cols", 12)),
        pitch_x_mm=float(data.get("pitch_x_mm", 9.0)),
        pitch_y_mm=float(data.get("pitch_y_mm", 9.0)),
        rotation_deg=float(data.get("rotation_deg", 0.0)),
        col_dir=int(data.get("col_dir", 1)),
        row_dir=int(data.get("row_dir", 1)),
        stagger_x_mm=float(data.get("stagger_x_mm", 0.0)))


# ═══════════════════════════════════════════════════════════════════
# Constraints
# ═══════════════════════════════════════════════════════════════════

CONSTRAINT_KINDS: tuple[str, ...] = (
    "fix",
    "coincident",
    "distance",             # mode "center" | "edge" (edge-to-edge)
    "distance_to_datum",    # + datum=...; mode "center" | "edge"
    "distance_to_line",
    "horizontal",
    "vertical",
    "concentric",
    "equal_radius",
    "point_on_line",
    "parallel",
    "perpendicular",
    "equal_length",
    "tangent",
    "symmetric",
    "angle",
    "radial",
)
#: ``drag_ghost`` is absent BY DESIGN. In v7.4.x the solver appended its
#: transient drag pin into the model's constraint list and it reached disk seven
#: times as an invisible weight-1000 pin. Here the ghost lives solver-side only
#: and is not a document kind at all, so it cannot be serialized even by a hand
#: edit.
PERSISTABLE_CONSTRAINT_KINDS = frozenset(CONSTRAINT_KINDS)


@dataclass
class Constraint:
    id: int = 0
    kind: str = ""
    refs: list[Ref] = field(default_factory=list)
    value: Optional[float] = None
    datum: str = ""
    mode: str = "center"        # center|edge, or external|internal for tangent
    weight: float = 1.0
    #: A reference dimension: displays a measurement, contributes no residual.
    driven: bool = False
    label: str = ""

    def to_dict(self) -> dict:
        d: dict[str, Any] = {
            "id": self.id, "kind": self.kind,
            "refs": [[int(e), str(k)] for e, k in self.refs],
        }
        if self.value is not None:
            d["value"] = _r6(self.value)
        if self.datum:
            d["datum"] = self.datum
        if self.mode != "center":
            d["mode"] = self.mode
        if self.weight != 1.0:
            d["weight"] = _r6(self.weight)
        if self.driven:
            d["driven"] = True
        if self.label:
            d["label"] = self.label
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "Constraint":
        refs: list[Ref] = []
        for r in data.get("refs", []):
            if isinstance(r, (list, tuple)):
                refs.append((int(r[0]), str(r[1]) if len(r) > 1 else ""))
            else:
                refs.append((int(r), ""))
        return cls(id=int(data.get("id", 0)), kind=data.get("kind", ""),
                   refs=refs, value=_opt_float(data.get("value")),
                   datum=data.get("datum", ""),
                   mode=data.get("mode", "center"),
                   weight=float(data.get("weight", 1.0)),
                   driven=bool(data.get("driven", False)),
                   label=data.get("label", ""))


def _is_persistable(c: Constraint) -> bool:
    return c.id > 0 and c.kind in PERSISTABLE_CONSTRAINT_KINDS


# ═══════════════════════════════════════════════════════════════════
# Evaluated wells
# ═══════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class EvaluatedWell:
    """One concrete well, resolved from a standalone Well or a pattern member.

    ``x`` / ``y`` are in the A1-relative storage frame.
    """
    key: Ref
    name: str
    x: float
    y: float
    diameter_mm: float
    well_depth_mm: float
    bottom_z_offset: float
    rim_height_mm: float
    ink_z_mm: Optional[float]
    well_type_id: Optional[str]
    rosette: Optional[RosettePlacement]
    source: str                                   # "well" | "ring" | "grid"
    declared_rc: Optional[tuple[int, int]] = None


# ═══════════════════════════════════════════════════════════════════
# Document metadata
# ═══════════════════════════════════════════════════════════════════

@dataclass
class DocMeta:
    id: str = ""
    name: str = "Untitled"
    description: str = ""
    kind: str = "plate"          # "plate" | "rosette"
    created: str = ""
    modified: str = ""
    rev: int = 1
    #: Store keys this document used to be known by, newest last. Per-plate
    #: stores read through these in order and always write the current id, so
    #: adopting a stable id does not orphan taught calibration.
    legacy_keys: list[str] = field(default_factory=list)
    #: Product overlay (PlateTypeStore id) this design represents, if any.
    plate_type_id: str = ""
    #: Learned needle-Z references, mm BELOW the needle-camera fiducial, keyed
    #: ``top`` / ``bottom`` / ``safe`` / ``max`` — the same shape and meaning as
    #: ``PlateType.z_offsets``.
    #:
    #: A design is a plate identity in its own right (it is what
    #: ``active_plate_key`` returns), so it has to be able to own these. Before
    #: this existed the learn loop could only write to a ``PlateType``, and an
    #: operator on a custom plate was told to "select a specific plate TYPE"
    #: — for a plate that already WAS specific.
    z_offsets: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict:
        d: dict[str, Any] = {"id": self.id, "name": self.name,
                             "kind": self.kind, "rev": int(self.rev)}
        if self.description:
            d["description"] = self.description
        if self.created:
            d["created"] = self.created
        if self.modified:
            d["modified"] = self.modified
        if self.legacy_keys:
            d["legacy_keys"] = list(self.legacy_keys)
        if self.plate_type_id:
            d["plate_type_id"] = self.plate_type_id
        if self.z_offsets:
            d["z_offsets"] = {k: round(float(v), 6)
                              for k, v in sorted(self.z_offsets.items())}
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "DocMeta":
        return cls(id=data.get("id", ""), name=data.get("name", "Untitled"),
                   description=data.get("description", ""),
                   kind=data.get("kind", "plate"),
                   created=data.get("created", ""),
                   modified=data.get("modified", ""),
                   rev=int(data.get("rev", 1)),
                   legacy_keys=list(data.get("legacy_keys", [])),
                   plate_type_id=data.get("plate_type_id", ""),
                   z_offsets={str(k): float(v) for k, v
                              in (data.get("z_offsets") or {}).items()})


# ═══════════════════════════════════════════════════════════════════
# The document
# ═══════════════════════════════════════════════════════════════════

#: Resolves a rosette id → its document. Injected so ``compile()`` stays pure
#: and testable without a store on disk.
RosetteLoader = Callable[[str], Optional["PlateDocument"]]


def rosette_subwell_offsets(subs, rotation_deg: float) -> list:
    """Sub-well offsets from the PARENT well centre, in plate mm.

    v7.9.1 — THE one place the seated-rosette transform lives. ``compile()``
    uses it, and so does the Layout canvas's preview, so what the operator sees
    on the plate is what will be compiled and printed. Two copies of this
    arithmetic is precisely the failure mode that produced the flipped well map
    (one corner rule in `label_positions`, another in the mapping dialog).

    A rosette is seated at its AUTHORED size and offset — rotation only, never
    scaled to fit the parent well. So a rosette drawn larger than its well is a
    real design problem the operator should be able to see, not something a
    renderer should quietly normalise away.
    """
    theta = math.radians(float(rotation_deg or 0.0))
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return [(s.x * cos_t - s.y * sin_t, s.x * sin_t + s.y * cos_t)
            for s in subs]


@dataclass
class PlateDocument:
    meta: DocMeta = field(default_factory=DocMeta)
    boundary: PlateBoundary = field(default_factory=PlateBoundary)
    defaults: WellStyle = field(default_factory=WellStyle)
    entities: dict[EntityId, Entity] = field(default_factory=dict)
    constraints: list[Constraint] = field(default_factory=list)
    units: str = "mm"
    _next_entity_id: int = 1
    _next_constraint_id: int = 1

    # ── Construction ──────────────────────────────────────────────

    @classmethod
    def new_plate(cls, name: str = "Untitled plate") -> "PlateDocument":
        return cls(meta=DocMeta(id=new_doc_id("plate"), name=name,
                                kind="plate", created=_utc_now(),
                                modified=_utc_now()),
                   boundary=PlateBoundary(),
                   defaults=WellStyle(diameter_mm=6.35, well_depth_mm=10.67))

    @classmethod
    def new_rosette(cls, bore_diameter_mm: float = 15.6,
                    name: str = "Untitled rosette") -> "PlateDocument":
        return cls(
            meta=DocMeta(id=new_doc_id("rosette"), name=name, kind="rosette",
                         created=_utc_now(), modified=_utc_now()),
            boundary=PlateBoundary(kind=BoundaryKind.CIRCLE,
                                   radius_mm=bore_diameter_mm / 2.0,
                                   origin_ref=OriginRef.CENTER),
            defaults=WellStyle(diameter_mm=max(bore_diameter_mm * 0.2, 0.4),
                               well_depth_mm=10.0))

    @classmethod
    def from_standard_format(cls, well_count: int,
                             name: str = "") -> "PlateDocument":
        """An editable document reproducing a bundled ``PLATE_DEFINITIONS`` entry.

        Verified by test to compile byte-identically to ``WellPlate.from_format``.
        """
        from SupportClasses.WellPlate import PLATE_DEFINITIONS
        if well_count not in PLATE_DEFINITIONS:
            raise ValueError(f"Unknown plate format: {well_count}")
        pdef = PLATE_DEFINITIONS[well_count]
        doc = cls.new_plate(name or f"{well_count}-well")
        doc.boundary = PlateBoundary(
            kind=BoundaryKind.RECT,
            width_mm=DEFAULT_FOOTPRINT_X_MM, height_mm=DEFAULT_FOOTPRINT_Y_MM,
            a1_x_mm=pdef["a1_offset_x"], a1_y_mm=pdef["a1_offset_y"],
            origin_ref=OriginRef.BOTTOM_LEFT)
        doc.defaults = WellStyle(diameter_mm=pdef["well_diameter"],
                                 well_depth_mm=pdef["well_depth_mm"])
        doc.meta.description = pdef.get("description", "")
        doc.add_grid(0.0, 0.0, rows=pdef["rows"], cols=pdef["cols"],
                     pitch_x_mm=pdef["well_spacing_x"],
                     pitch_y_mm=pdef["well_spacing_y"],
                     style=WellStyle(diameter_mm=pdef["well_diameter"],
                                     well_depth_mm=pdef["well_depth_mm"]),
                     name=f"grid-{well_count}")
        return doc

    # ── Entity plumbing ───────────────────────────────────────────

    def _alloc_entity_id(self) -> EntityId:
        eid = self._next_entity_id
        self._next_entity_id += 1
        return eid

    def _alloc_constraint_id(self) -> int:
        cid = self._next_constraint_id
        self._next_constraint_id += 1
        return cid

    def add_entity(self, ent: Entity) -> Entity:
        if not ent.id:
            ent.id = self._alloc_entity_id()
        else:
            self._next_entity_id = max(self._next_entity_id, ent.id + 1)
        self.entities[ent.id] = ent
        return ent

    def add_point(self, x: float, y: float, *, fixed: bool = False,
                  construction: bool = False) -> Point:
        return self.add_entity(Point(x=x, y=y, fixed=fixed,
                                     construction=construction))

    def add_well(self, x: float, y: float, name: str,
                 style: Optional[WellStyle] = None) -> Well:
        centre = self.add_point(x, y)
        return self.add_entity(Well(
            name=name, center=centre.id,
            style=replace(style or self.defaults)))

    def add_ring(self, cx: float, cy: float, *, count: int = 6,
                 ring_diameter_mm: float = 8.0, start_angle_deg: float = 0.0,
                 center_well: bool = False,
                 style: Optional[WellStyle] = None,
                 naming: Optional[NamingSpec] = None,
                 name: str = "") -> RingPattern:
        anchor = self.add_point(cx, cy)
        feat = self.add_entity(RingPattern(
            name=name or self._next_feature_name("ring"), anchor=anchor.id,
            style=replace(style or self.defaults),
            naming=naming or NamingSpec(scheme=NamingScheme.LETTERS),
            count=count, ring_diameter_mm=ring_diameter_mm,
            start_angle_deg=start_angle_deg, center_well=center_well))
        if naming is None:
            self.autoname_feature(feat)
        return feat

    def add_grid(self, seed_x: float, seed_y: float, *, rows: int = 8,
                 cols: int = 12, pitch_x_mm: float = 9.0,
                 pitch_y_mm: float = 9.0, rotation_deg: float = 0.0,
                 style: Optional[WellStyle] = None,
                 naming: Optional[NamingSpec] = None,
                 name: str = "") -> GridPattern:
        anchor = self.add_point(seed_x, seed_y)
        feat = self.add_entity(GridPattern(
            name=name or self._next_feature_name("grid"), anchor=anchor.id,
            style=replace(style or self.defaults),
            naming=naming or NamingSpec(scheme=NamingScheme.ANSI),
            rows=rows, cols=cols, pitch_x_mm=pitch_x_mm,
            pitch_y_mm=pitch_y_mm, rotation_deg=rotation_deg))
        if naming is None:
            self.autoname_feature(feat)
        return feat

    def _next_feature_name(self, stem: str) -> str:
        taken = {getattr(e, "name", "") for e in self.entities.values()}
        i = 1
        while f"{stem}-{i}" in taken:
            i += 1
        return f"{stem}-{i}"

    def remove_entity(self, eid: EntityId) -> None:
        """Remove *eid*, its owned anchor/centre Point, and any constraint
        referencing either."""
        ent = self.entities.pop(eid, None)
        if ent is None:
            return
        owned = None
        if isinstance(ent, Well):
            owned = ent.center
        elif isinstance(ent, PatternFeature):
            owned = ent.anchor
        if owned is not None:
            self.entities.pop(owned, None)
        dead = {eid} | ({owned} if owned is not None else set())
        self.constraints = [
            c for c in self.constraints
            if not any(r[0] in dead for r in c.refs)]

    def patterns(self) -> list[PatternFeature]:
        return [e for e in self.entities.values()
                if isinstance(e, PatternFeature)]

    def standalone_wells(self) -> list[Well]:
        return [e for e in self.entities.values() if isinstance(e, Well)]

    def anchor_point(self, feature: PatternFeature) -> Optional[Point]:
        p = self.entities.get(feature.anchor)
        return p if isinstance(p, Point) else None

    # ── Constraints ───────────────────────────────────────────────

    def add_constraint(self, kind: str, refs: Iterable[Ref], *,
                       value: Optional[float] = None, datum: str = "",
                       mode: str = "center", driven: bool = False,
                       label: str = "") -> Constraint:
        if kind not in CONSTRAINT_KINDS:
            raise ValueError(f"Unknown constraint kind {kind!r}")
        c = Constraint(id=self._alloc_constraint_id(), kind=kind,
                       refs=[(int(e), str(k)) for e, k in refs], value=value,
                       datum=datum, mode=mode, driven=driven, label=label)
        self.constraints.append(c)
        return c

    def remove_constraint(self, cid: int) -> bool:
        n = len(self.constraints)
        self.constraints = [c for c in self.constraints if c.id != cid]
        return len(self.constraints) != n

    def constraint_by_id(self, cid: int) -> Optional[Constraint]:
        return next((c for c in self.constraints if c.id == cid), None)

    def prune_constraints(self) -> int:
        """Drop constraints whose refs no longer resolve. Returns the count."""
        live = set(self.entities)
        keep: list[Constraint] = []
        for c in self.constraints:
            ok = True
            for eid, key in c.refs:
                if eid not in live:
                    ok = False
                    break
                ent = self.entities[eid]
                if key and isinstance(ent, PatternFeature) \
                        and key not in set(ent.member_keys()):
                    ok = False
                    break
            if ok:
                keep.append(c)
        removed = len(self.constraints) - len(keep)
        self.constraints = keep
        return removed

    # ── Member overrides ──────────────────────────────────────────

    def override(self, feature: PatternFeature,
                 key: MemberKey) -> MemberOverride:
        """Get-or-create the override record for one member."""
        ov = feature.overrides.get(key)
        if ov is None:
            ov = MemberOverride()
            feature.overrides[key] = ov
        return ov

    def prune_orphan_overrides(self, feature: PatternFeature) -> int:
        """Drop overrides whose member no longer exists.

        Deliberately NOT automatic: shrinking a pattern and growing it back
        should restore what was there, so orphans are kept dormant until the
        operator asks.
        """
        live = set(feature.member_keys())
        dead = [k for k in feature.overrides if k not in live]
        for k in dead:
            del feature.overrides[k]
        return len(dead)

    # ── Evaluation ────────────────────────────────────────────────

    def evaluate(self, *, include_suppressed: bool = False
                 ) -> list[EvaluatedWell]:
        """Resolve every well this document describes. Pure; no side effects.

        The single source of truth for "what wells exist" — used by
        :meth:`compile`, :meth:`validate` and the canvas, so the drawing and the
        compiled plate can never disagree.
        """
        out: list[EvaluatedWell] = []

        for ent in sorted(self.entities.values(), key=lambda e: e.id):
            if isinstance(ent, Well):
                pt = self.entities.get(ent.center)
                if not isinstance(pt, Point):
                    logger.warning(
                        "Well %s (id=%s) has no centre Point", ent.name, ent.id)
                    continue
                st = ent.style
                out.append(EvaluatedWell(
                    key=(ent.id, ""), name=ent.name, x=pt.x, y=pt.y,
                    diameter_mm=st.diameter_mm,
                    well_depth_mm=st.well_depth_mm,
                    bottom_z_offset=st.bottom_z_offset,
                    rim_height_mm=st.rim_height_mm, ink_z_mm=st.ink_z_mm,
                    well_type_id=st.well_type_id, rosette=ent.rosette,
                    source="well"))
                continue

            if not isinstance(ent, PatternFeature):
                continue
            anchor = self.anchor_point(ent)
            if anchor is None:
                logger.warning("Pattern %s (id=%s) has no anchor Point",
                               ent.name, ent.id)
                continue
            src = "ring" if isinstance(ent, RingPattern) else "grid"
            for idx, (key, dx, dy) in enumerate(ent.local_offsets()):
                ov = ent.overrides.get(key) or MemberOverride()
                if ov.suppressed and not include_suppressed:
                    continue
                if ov.offset_mm:
                    dx += ov.offset_mm[0]
                    dy += ov.offset_mm[1]
                st = ent.style
                rc = None
                if isinstance(ent, GridPattern) and key.startswith("g"):
                    r_s, _, c_s = key[1:].partition("_")
                    try:
                        rc = (int(r_s), int(c_s))
                    except ValueError:
                        rc = None
                out.append(EvaluatedWell(
                    key=(ent.id, key),
                    name=ov.name or self._member_name(ent, key, idx),
                    x=anchor.x + dx, y=anchor.y + dy,
                    diameter_mm=_pick(ov.diameter_mm, st.diameter_mm),
                    well_depth_mm=_pick(ov.well_depth_mm, st.well_depth_mm),
                    bottom_z_offset=_pick(ov.bottom_z_offset,
                                          st.bottom_z_offset),
                    rim_height_mm=_pick(ov.rim_height_mm, st.rim_height_mm),
                    ink_z_mm=(ov.ink_z_mm if ov.ink_z_mm is not None
                              else st.ink_z_mm),
                    well_type_id=ov.well_type_id or st.well_type_id,
                    rosette=ov.rosette, source=src, declared_rc=rc))
        return out

    def _member_name(self, feature: PatternFeature, key: MemberKey,
                     index: int) -> str:
        spec = feature.naming
        if isinstance(feature, GridPattern) and key.startswith("g"):
            r_s, _, c_s = key[1:].partition("_")
            try:
                r, c = int(r_s), int(c_s)
            except ValueError:
                r = c = 0
            if spec.scheme == NamingScheme.ANSI:
                return (f"{spec.prefix}"
                        f"{ansi_label(spec.start_row + r, spec.start_col - 1 + c)}")
            if spec.scheme == NamingScheme.LETTERS:
                return f"{spec.prefix}{letter_label(spec.start_col - 1 + index)}"
            if spec.scheme == NamingScheme.NUMBERS:
                return f"{spec.prefix}{spec.start_col + index}"
            return f"{spec.prefix}{key}"
        if spec.scheme == NamingScheme.ANSI:
            return (f"{spec.prefix}"
                    f"{ansi_label(spec.start_row, spec.start_col - 1 + index)}")
        if spec.scheme == NamingScheme.NUMBERS:
            return f"{spec.prefix}{spec.start_col + index}"
        if spec.scheme == NamingScheme.MANUAL:
            return f"{spec.prefix}{key}"
        return f"{spec.prefix}{letter_label(spec.start_col - 1 + index)}"

    # ── Automatic name de-confliction ─────────────────────────────
    #
    # Every pattern used to be born with the SAME default spec — every grid
    # ``A1…``, every ring ``a…`` — so the second one you placed collided with
    # the first on every single well. That is not cosmetic: ``WellPlate.
    # from_wells`` keys its dict by ``name.upper()``, so the duplicates are
    # silently DROPPED and most of the second pattern simply vanishes from the
    # compiled plate. ``validate()`` reported it, but there was no control
    # anywhere to act on the report.

    def feature_member_names(self, feature: PatternFeature) -> list[str]:
        """The names *feature* currently produces, overrides included."""
        out: list[str] = []
        for idx, (key, _dx, _dy) in enumerate(feature.local_offsets()):
            ov = feature.overrides.get(key)
            if ov is not None and ov.suppressed:
                continue
            out.append((ov.name if ov is not None and ov.name else None)
                       or self._member_name(feature, key, idx))
        return out

    def names_in_use(self, exclude: Optional[EntityId] = None) -> set[str]:
        """Upper-cased names taken by every well EXCEPT those of *exclude*."""
        return {w.name.upper()
                for w in self.evaluate(include_suppressed=True)
                if w.key[0] != exclude}

    def autoname_feature(self, feature: PatternFeature,
                         *, max_tries: int = 200) -> bool:
        """Shift *feature*'s naming until none of its wells collide.

        Advances the natural axis for the scheme — the row letter for an ANSI
        grid (so a second grid continues ``E1…`` after the first ends at D),
        the letter/number offset otherwise — and falls back to a guaranteed
        unique prefix if the document is genuinely crowded. Returns True when
        the result is collision-free.
        """
        spec = feature.naming
        taken = self.names_in_use(exclude=feature.id)
        for _ in range(max_tries):
            mine = self.feature_member_names(feature)
            upper = [n.upper() for n in mine]
            if len(set(upper)) == len(upper) and not (taken & set(upper)):
                return True
            if (spec.scheme == NamingScheme.ANSI
                    and isinstance(feature, GridPattern)):
                spec.start_row += 1
            else:
                spec.start_col += 1
        # Crowded past the point of shifting: a prefix keyed to the entity id
        # cannot collide, because ids are unique within a document.
        spec.start_row, spec.start_col = 0, 1
        spec.prefix = f"{feature.name or 'p'}-".replace(" ", "")
        mine = [n.upper() for n in self.feature_member_names(feature)]
        return len(set(mine)) == len(mine) and not (taken & set(mine))

    # ── Display transform (origin_ref) ────────────────────────────

    def to_display(self, x: float, y: float) -> tuple[float, float]:
        """Storage → the operator's chosen origin frame."""
        ox, oy = self.boundary.origin_offset()
        dx, dy = x - ox, y - oy
        return (dx, -dy) if self.boundary.display_y_is_up() else (dx, dy)

    def from_display(self, x: float, y: float) -> tuple[float, float]:
        ox, oy = self.boundary.origin_offset()
        if self.boundary.display_y_is_up():
            y = -y
        return (x + ox, y + oy)

    # ── Rosettes ──────────────────────────────────────────────────

    def placements(self) -> list[tuple[Ref, RosettePlacement]]:
        return [(w.key, w.rosette) for w in self.evaluate()
                if w.rosette is not None and w.rosette.rosette_id]

    def place_rosette(self, key: Ref, rosette_id: str, *,
                      rotation_deg: float = 0.0,
                      rosette_name: str = "") -> RosettePlacement:
        """Seat a rosette in the well identified by *key*.

        One call serves both placement gestures — stamp-click passes one key,
        multi-select passes many.
        """
        placement = RosettePlacement(rosette_id=rosette_id,
                                     rotation_deg=rotation_deg,
                                     rosette_name=rosette_name)
        eid, member = key
        ent = self.entities.get(eid)
        if isinstance(ent, Well) and not member:
            ent.rosette = placement
        elif isinstance(ent, PatternFeature) and member:
            self.override(ent, member).rosette = placement
        else:
            # v7.9.1: a PatternFeature with an EMPTY member is the pattern
            # itself, not a well — and it used to be accepted, writing an
            # override under the key "" that matches no member. The rosette was
            # then saved to disk, rendered nowhere, and reported no error: the
            # operator saw "it won't let me place it". A 24-well plate is ONE
            # GridPattern and the canvas selects the pattern on a plain click,
            # so this was the ordinary path, not an edge case. Refuse instead —
            # `_stamp` already handles KeyError, and the caller now reports it.
            raise KeyError(f"No well at {key!r}")
        return placement

    def clear_rosette(self, key: Ref) -> bool:
        eid, member = key
        ent = self.entities.get(eid)
        if isinstance(ent, Well) and not member:
            had = ent.rosette is not None
            ent.rosette = None
            return had
        if isinstance(ent, PatternFeature):
            ov = ent.overrides.get(member)
            if ov is None or ov.rosette is None:
                return False
            ov.rosette = None
            if ov.is_empty():
                del ent.overrides[member]
            return True
        return False

    def rosette_ids_used(self) -> set[str]:
        return {p.rosette_id for _, p in self.placements()}

    def rosette_revision_fingerprint(
            self, loader: Optional[RosetteLoader]) -> dict[str, int]:
        """``{rosette_id: rev}`` for every rosette this plate references.

        Belongs in the plate's calibration fingerprint. Rosettes are linked
        LIVE, so editing one moves sub-well positions on plates that may already
        be taught; without this the run would drive to stale coordinates with no
        warning. With it, a changed rosette invalidates the taught calibration
        and prompts a re-teach — which is what makes live linking safe.
        """
        out: dict[str, int] = {}
        for rid in sorted(self.rosette_ids_used()):
            doc = loader(rid) if loader else None
            out[rid] = int(doc.meta.rev) if doc is not None else -1
        return out

    # ── Compile ───────────────────────────────────────────────────

    def compile(self, loader: Optional[RosetteLoader] = None) -> WellPlate:
        """Resolve to a runtime :class:`WellPlate`.

        See the module docstring: ``WellInfo.x/y`` stay A1-relative and
        sub-wells keep the ``"<parent>.<sub>"`` protocol.
        """
        wells = [w for w in self.evaluate()
                 if not (w.rosette and not w.rosette.rosette_id)
                 or True]
        if not wells:
            raise ValueError(
                f"Cannot compile '{self.meta.name}' — it has no wells")

        rows_cols = _assign_rows_cols(wells)
        a1_idx = _a1_index(wells, rows_cols)
        ax, ay = wells[a1_idx].x, wells[a1_idx].y

        infos: list[WellInfo] = []
        for w, (r, c) in zip(wells, rows_cols):
            px, py = w.x - ax, w.y - ay
            subs = self._resolve_subwells(w, loader)
            if not subs:
                infos.append(WellInfo(
                    name=w.name, row=r, col=c, x=px, y=py,
                    diameter=w.diameter_mm,
                    bottom_z_offset=w.bottom_z_offset,
                    rim_height_mm=w.rim_height_mm, ink_z_mm=w.ink_z_mm))
                continue
            rot = w.rosette.rotation_deg if w.rosette else 0.0
            for sub, (rx, ry) in zip(subs, rosette_subwell_offsets(subs, rot)):
                infos.append(WellInfo(
                    name=f"{w.name}.{sub.name}", row=r, col=c,
                    x=px + rx, y=py + ry, diameter=sub.diameter_mm,
                    bottom_z_offset=sub.bottom_z_offset,
                    rim_height_mm=sub.rim_height_mm, ink_z_mm=sub.ink_z_mm,
                    is_subwell=True, parent_well=w.name))

        if not infos:
            raise ValueError(
                f"Cannot compile '{self.meta.name}' — every well resolved empty")

        x0, y0, x1, y1 = self.boundary.extent_a1()
        return WellPlate.from_wells(
            name=self.meta.name, wells=infos,
            well_depth_mm=self.defaults.well_depth_mm,
            # DERIVED from the authored footprint, never a stored second copy.
            a1_offset_x=ax - x0, a1_offset_y=ay - y0,
            description=self.meta.description)

    @staticmethod
    def rosette_subwells(rosette_doc) -> list:
        """The sub-wells a rosette document contributes, or ``[]``.

        v7.9.1 — the read-only half of :meth:`_resolve_subwells`, exposed so a
        RENDERER can show a placed rosette without a plate compile.
        """
        if rosette_doc is None:
            return []
        try:
            return list(rosette_doc.evaluate())
        except Exception:                              # pragma: no cover
            return []

    def _resolve_subwells(self, w: EvaluatedWell,
                          loader: Optional[RosetteLoader]
                          ) -> list[EvaluatedWell]:
        if w.rosette is None or not w.rosette.rosette_id:
            return []
        doc = loader(w.rosette.rosette_id) if loader else None
        if doc is None:
            # Compile as an ordinary well rather than dropping it — losing a
            # well silently would shift row/col for everything after it.
            # validate() reports this by name.
            logger.warning(
                "Well %s references rosette %r, which could not be loaded — "
                "compiling it as an ordinary well",
                w.name, w.rosette.rosette_id)
            return []
        subs = doc.evaluate()
        if not subs:
            return []
        bad = [s.name for s in subs if "." in s.name]
        if bad:
            raise ValueError(
                f"Rosette '{doc.meta.name}' has sub-well name(s) containing "
                f"'.': {bad}. The dot separates a parent from its sub-well.")
        return subs

    # ── Validation ────────────────────────────────────────────────

    def validate(self, loader: Optional[RosetteLoader] = None) -> list[str]:
        """Human-readable problems. Empty list = fine."""
        problems: list[str] = []
        wells = self.evaluate()
        if not wells:
            problems.append("The design has no wells.")

        seen: dict[str, int] = {}
        for w in wells:
            seen[w.name.upper()] = seen.get(w.name.upper(), 0) + 1
        for nm, n in sorted(seen.items()):
            if n > 1:
                problems.append(
                    f"{n} wells are named '{nm}' — well names must be unique "
                    f"(a duplicate is silently dropped at runtime).")

        for w in wells:
            if "." in w.name:
                problems.append(
                    f"Well '{w.name}' contains '.', which is reserved for "
                    f"rosette sub-wells.")
            if not self.boundary.contains(w.x, w.y, w.diameter_mm / 2.0):
                problems.append(
                    f"Well '{w.name}' extends outside the plate boundary.")

        for w in wells:
            if w.rosette is None or not w.rosette.rosette_id:
                continue
            doc = loader(w.rosette.rosette_id) if loader else None
            if doc is None:
                problems.append(
                    f"Well '{w.name}' references rosette "
                    f"'{w.rosette.rosette_name or w.rosette.rosette_id}', "
                    f"which is missing from the rosette library.")
                continue
            for s in doc.evaluate():
                if math.hypot(s.x, s.y) + s.diameter_mm / 2.0 > \
                        w.diameter_mm / 2.0 + 1e-6:
                    problems.append(
                        f"Rosette '{doc.meta.name}' does not fit inside well "
                        f"'{w.name}'.")
                    break

        if not is_store_key_safe(self.meta.id):
            problems.append(
                f"Document id '{self.meta.id}' is not filesystem-safe.")
        return problems

    # ── Serialization ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "schema": SCHEMA_NAME,
            "schema_version": SCHEMA_VERSION,
            "meta": self.meta.to_dict(),
            "units": self.units,
            "boundary": self.boundary.to_dict(),
            "defaults": self.defaults.to_dict(),
            "entities": [self.entities[eid].to_dict()
                         for eid in sorted(self.entities)],
            "constraints": [c.to_dict() for c in self.constraints
                            if _is_persistable(c)],
            "next_entity_id": self._next_entity_id,
            "next_constraint_id": self._next_constraint_id,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "PlateDocument":
        if data.get("schema") != SCHEMA_NAME:
            raise LegacySchemaError(
                "Not a v7.12 plate document (no 'schema' key) — this is a "
                "pre-v7.12 PlateDesign file.")
        ver = int(data.get("schema_version", 0))
        if ver > SCHEMA_VERSION:
            raise FutureSchemaError(
                f"Plate document schema v{ver} is newer than this build "
                f"(v{SCHEMA_VERSION}). Opening it would discard data.")

        doc = cls(meta=DocMeta.from_dict(data.get("meta", {})),
                  boundary=PlateBoundary.from_dict(data.get("boundary", {})),
                  defaults=WellStyle.from_dict(data.get("defaults", {})),
                  units=data.get("units", "mm"))
        for edata in data.get("entities", []):
            ent = _entity_from_dict(edata)
            doc.entities[ent.id] = ent
        for cdata in data.get("constraints", []):
            con = Constraint.from_dict(cdata)
            if not _is_persistable(con):
                logger.warning("Dropping non-persistable constraint on load: "
                               "kind=%s id=%s", con.kind, con.id)
                continue
            doc.constraints.append(con)

        doc._next_entity_id = max(int(data.get("next_entity_id", 1)),
                                  max(doc.entities, default=0) + 1)
        doc._next_constraint_id = max(
            int(data.get("next_constraint_id", 1)),
            max((c.id for c in doc.constraints), default=0) + 1)
        return doc

    def save(self, path: Path) -> Path:
        """Atomic write — a crash mid-save must not corrupt the plate."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.meta.modified = _utc_now()
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)
        return path

    @classmethod
    def load(cls, path: Path) -> "PlateDocument":
        with open(Path(path), encoding="utf-8") as f:
            return cls.from_dict(json.load(f))


def _pick(override, default):
    return default if override is None else override


# ═══════════════════════════════════════════════════════════════════
# Row / column assignment
# ═══════════════════════════════════════════════════════════════════

def _assign_rows_cols(wells: list[EvaluatedWell]) -> list[tuple[int, int]]:
    """(row, col) per well.

    Fast path: when a single unrotated grid declares indices for every well, use
    them — O(n) and exact. Otherwise bin by coordinate at O(n log n), replacing
    the legacy O(n^2) min-pairwise-distance scan (73,536 hypot calls per compile
    at 384 wells).
    """
    if wells and all(w.declared_rc is not None for w in wells):
        return [w.declared_rc for w in wells]   # type: ignore[misc]
    rows = _bin_axis([w.y for w in wells])
    cols = _bin_axis([w.x for w in wells])
    return list(zip(rows, cols))


#: Two wells nearer than this on an axis are the same row/column. Below any
#: real well pitch (a 384-well plate is 4.5 mm) and above any plausible jitter.
_MIN_CLUSTER_GAP_MM = 0.5


def _bin_axis(values: list[float]) -> list[int]:
    """Cluster *values* into ordered bins; returns each value's bin index.

    A gap separates two bins when it exceeds BOTH half the largest gap and
    :data:`_MIN_CLUSTER_GAP_MM`.

    Scaling by the largest gap rather than the median is what makes hand-placed
    wells work: with two rows of two, the gaps are ``[jitter, pitch, jitter]``
    and the median IS the jitter, so a median-scaled tolerance splits every
    well into its own row. The absolute floor then covers the opposite case —
    a single row whose only gaps are jitter, where half the largest gap is
    itself tiny.
    """
    if not values:
        return []
    uniq = sorted(set(round(v, 6) for v in values))
    if len(uniq) == 1:
        return [0] * len(values)
    max_gap = max(b - a for a, b in zip(uniq, uniq[1:]))
    tol = max(max_gap * 0.5, _MIN_CLUSTER_GAP_MM)
    edges: list[float] = [uniq[0]]
    for v in uniq[1:]:
        if v - edges[-1] > tol:
            edges.append(v)
    return [min(range(len(edges)), key=lambda i: abs(edges[i] - v))
            for v in values]


def _a1_index(wells: list[EvaluatedWell],
              rows_cols: list[tuple[int, int]]) -> int:
    """Index of the A1 well: lowest (row, col), ties by min x then min y."""
    best = 0
    best_key = None
    for i, (w, (r, c)) in enumerate(zip(wells, rows_cols)):
        key = (r, c, w.x, w.y)
        if best_key is None or key < best_key:
            best_key, best = key, i
    return best
