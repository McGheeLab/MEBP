"""
PlateDesign.py — Editable parametric well-plate sketch (v7.4.5).

This module is the *document* the user edits in the Plate Designer.
The runtime artifact every other page consumes is `WellPlate`; the
designer compiles a `PlateDesign` into a `WellPlate` via `compile()`.

Conceptual model
================

Every entity has a unique integer id. Wells *own* a center `Point` rather
than embedding x/y directly — that way every geometric constraint operates
on points, lines, or circles, and a "Well" is just a labeled wrapper
around a center + diameter. Distance between two wells is therefore
`distance_pp(well_a.center, well_b.center)` — no special-cased
well-to-well constraint kind is needed.

Constraints are a tagged union (`Constraint` with `kind: str`). Each kind
contributes one or more scalar residuals; see `PlateSketchSolver` for the
math.

JSON round-trip
===============

`to_dict()` / `from_dict()` serialize the full document. The entities
dict serializes each entity with a "type" discriminator. See
`tests/test_v745_plate_design.py` for round-trip coverage.

Compile path
============

`compile()` walks all non-construction wells, resolves each center
`Point` to absolute coordinates, bins wells into rows/cols by Y-then-X
clustering (tolerance = half the minimum pairwise distance), and emits
a `WellPlate` via `WellPlate.from_wells`. Construction geometry is
dropped.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar, Optional

from SupportClasses.WellPlate import (
    WellPlate, WellInfo, ROW_LABELS, PLATE_DEFINITIONS, USER_PLATES_DIR,
)
from SupportClasses.MachineConfig import resolve_shared_path

logger = logging.getLogger(__name__)


# v7.4.8: standard in-house insert designs (nested rosette PlateDesigns)
# that can be dropped into any well. Shared/portable — meant to sync across
# every rig, unlike a per-machine store.
INSERTS_DIR = resolve_shared_path("inserts")


def list_standard_inserts() -> list[str]:
    """Names of saved standard inserts under INSERTS_DIR."""
    if not INSERTS_DIR.exists():
        return []
    return sorted(p.stem for p in INSERTS_DIR.glob("*.json"))


def load_standard_insert(name: str) -> "PlateDesign":
    """Load a saved standard insert as a nested rosette PlateDesign."""
    path = INSERTS_DIR / f"{name}.json"
    with open(path) as f:
        data = json.load(f)
    return PlateDesign.from_dict(data)


def save_standard_insert(design: "PlateDesign", name: str) -> Path:
    """Save a nested rosette design as a reusable standard insert."""
    INSERTS_DIR.mkdir(parents=True, exist_ok=True)
    path = INSERTS_DIR / f"{name}.json"
    design.name = name
    with open(path, "w") as f:
        json.dump(design.to_dict(), f, indent=2)
    logger.info(f"Standard insert saved to {path}")
    return path


# ═══════════════════════════════════════════════════════════════════
# Entities — points, lines, circles, wells, groups
# ═══════════════════════════════════════════════════════════════════

EntityId = int


@dataclass
class Entity:
    """Base class for all sketch entities.

    Every entity has a unique integer id, a construction flag (sketch
    helpers that are excluded from compile()), and a fixed flag (locks
    the entity's free parameters out of the solver).
    """
    id: EntityId = 0
    construction: bool = False
    fixed: bool = False

    # Subclasses set this to drive JSON discriminator serialization.
    TYPE: ClassVar[str] = "Entity"


@dataclass
class Point(Entity):
    x: float = 0.0
    y: float = 0.0
    TYPE: ClassVar[str] = "Point"


@dataclass
class Line(Entity):
    p1: EntityId = 0
    p2: EntityId = 0
    TYPE: ClassVar[str] = "Line"


@dataclass
class Circle(Entity):
    center: EntityId = 0  # Point id
    radius: float = 1.0
    TYPE: ClassVar[str] = "Circle"


@dataclass
class Well(Entity):
    """A real well — compiles to a `WellInfo` in `compile()`."""
    name: str = "A1"
    center: EntityId = 0  # Point id
    diameter: float = 6.35
    well_depth_mm: float = 10.67
    bottom_z_offset: float = 0.0
    group: Optional[EntityId] = None
    naming_scheme: str = "ANSI"  # "ANSI" | "MANUAL" — manual names skip auto re-letter
    # v7.4.7: optional nested rosette design (sub-wells inside this well).
    # v7.4.8: compile() FLATTENS this into named sub-wells (A1.a, A1.b, …);
    # the parent well is not itself a runtime well.
    rosette_design: Optional["PlateDesign"] = None
    # v7.4.8: clockwise rotation (deg) applied to the rosette sub-well
    # layout when flattening — aligns a dropped standard insert.
    rosette_rotation_deg: float = 0.0
    # v7.4.8: insert/tube geometry (set on rosette sub-wells).
    rim_height_mm: float = 0.0       # top above plate (clearance)
    ink_z_mm: Optional[float] = None  # dispense Z rel. plate top; None=default
    # v7.5.x: id of the WellTypeStore preset the geometry was stamped from
    # (design-time only — lets the picker show the current selection and
    # round-trip; the stamped diameter/depth/rim/ink_z are the source of
    # truth, so compile()/WellInfo do NOT read this). None = custom.
    well_type_id: Optional[str] = None
    TYPE: ClassVar[str] = "Well"


@dataclass
class Group(Entity):
    """A named selection of entities (typically wells placed together)."""
    name: str = "group"
    members: list[EntityId] = field(default_factory=list)
    # "free" | "grid" | "circle" — drives the properties panel options.
    pattern_kind: str = "free"
    # Pattern parameters (free-form blob, depends on pattern_kind).
    params: dict[str, Any] = field(default_factory=dict)
    TYPE: ClassVar[str] = "Group"


_ENTITY_TYPES: dict[str, type[Entity]] = {
    "Point": Point,
    "Line": Line,
    "Circle": Circle,
    "Well": Well,
    "Group": Group,
}


# ═══════════════════════════════════════════════════════════════════
# Plate outline (rectangular footprint or polygon)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PlateOutline:
    kind: str = "rect"  # "rect" | "polygon" | "circle" (v7.4.7, rosette bore)
    width: float = 127.76   # ANSI/SLAS footprint width (mm)
    height: float = 85.48   # ANSI/SLAS footprint height (mm)
    radius: float = 0.0     # v7.4.7: bore radius (mm) when kind == "circle"
    polygon: list[tuple[float, float]] = field(default_factory=list)
    origin_point: EntityId = 0  # Always fixed at (0, 0) via `ground`

    def to_dict(self) -> dict:
        return {
            "kind": self.kind,
            "width": self.width,
            "height": self.height,
            "radius": self.radius,
            "polygon": [list(p) for p in self.polygon],
            "origin_point": self.origin_point,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "PlateOutline":
        return cls(
            kind=data.get("kind", "rect"),
            width=data.get("width", 127.76),
            height=data.get("height", 85.48),
            radius=data.get("radius", 0.0),
            polygon=[tuple(p) for p in data.get("polygon", [])],
            origin_point=data.get("origin_point", 0),
        )


# ═══════════════════════════════════════════════════════════════════
# Constraint — tagged union with scalar residuals
# ═══════════════════════════════════════════════════════════════════

# Constraint kinds shipped in v7.4.5+. Solver maps each to a residual.
CONSTRAINT_KINDS: tuple[str, ...] = (
    "ground",
    "fix",
    "coincident_pp",
    "distance_pp",
    "horizontal",
    "vertical",
    "concentric",
    "equal_radius",
    "point_on_line",   # v7.4.6
    "parallel",        # v7.4.6
    "perpendicular",   # v7.4.6
    "equal_length",    # v7.4.6
    "tangent_cc",      # v7.4.6 — well-to-well tangent (external)
    "symmetric_pp",    # v7.4.6 — two points symmetric about a line
    "dist_left_edge",  # v7.4.7 — well distance from plate left edge
    "dist_top_edge",   # v7.4.7 — well distance from plate top edge
    "drag_ghost",      # transient, solver-internal — NEVER persisted (see below)
)

# Kinds that may reach disk. `drag_ghost` is the solver's transient drag pin
# (weight 1000, id -1) and is deliberately excluded.
#
# v7.12: this filter exists because the exclusion was previously only a comment.
# `PlateSketchSolver.begin_drag` appends the ghost into `design.constraints`, and
# any save or undo-snapshot taken between begin_drag and end_drag serialized it.
# Reloading then restored it as an INVISIBLE weight-1000 pin that silently fought
# every later edit and solve. Seven of them had already reached disk across three
# shipped plate files. Filtered on BOTH write and read, so a hand-edited or
# already-contaminated file cannot reintroduce one either.
PERSISTABLE_CONSTRAINT_KINDS: frozenset[str] = frozenset(
    k for k in CONSTRAINT_KINDS if k != "drag_ghost")


def _is_persistable(c: "Constraint") -> bool:
    """True when *c* belongs on disk. Transients carry id <= 0."""
    return c.id > 0 and c.kind in PERSISTABLE_CONSTRAINT_KINDS


@dataclass
class Constraint:
    id: int = 0
    kind: str = ""
    refs: list[EntityId] = field(default_factory=list)
    value: Optional[float] = None
    weight: float = 1.0
    # `fix` snapshots the entity's position when the lock is applied so
    # subsequent solves don't drift the locked point.
    snapshot: Optional[tuple[float, float]] = None
    # v7.4.7: reference mode for edge-distance dimensions —
    # "center" (default) measures to the well center, "edge" to the
    # well's near edge (subtracts the radius).
    mode: str = "center"

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "kind": self.kind,
            "refs": list(self.refs),
            "value": self.value,
            "weight": self.weight,
            "snapshot": list(self.snapshot) if self.snapshot else None,
            "mode": self.mode,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Constraint":
        snap = data.get("snapshot")
        return cls(
            id=data.get("id", 0),
            kind=data.get("kind", ""),
            refs=list(data.get("refs", [])),
            value=data.get("value"),
            weight=data.get("weight", 1.0),
            snapshot=tuple(snap) if snap else None,
            mode=data.get("mode", "center"),
        )


# ═══════════════════════════════════════════════════════════════════
# PlateDesign — the editable document
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PlateDesign:
    schema_version: str = "1.0"
    name: str = "Custom Plate"
    description: str = ""
    units: str = "mm"
    outline: PlateOutline = field(default_factory=PlateOutline)
    entities: dict[EntityId, Entity] = field(default_factory=dict)
    constraints: list[Constraint] = field(default_factory=list)
    a1_offset_x: float = 14.38
    a1_offset_y: float = 11.24
    well_depth_default_mm: float = 10.67
    _next_entity_id: int = 1
    _next_constraint_id: int = 1

    # ── ID helpers ────────────────────────────────────────────────

    def new_entity_id(self) -> EntityId:
        eid = self._next_entity_id
        self._next_entity_id += 1
        return eid

    def new_constraint_id(self) -> int:
        cid = self._next_constraint_id
        self._next_constraint_id += 1
        return cid

    # ── Add / remove ──────────────────────────────────────────────

    def add_entity(self, entity: Entity) -> Entity:
        """Add an entity (auto-assigning an id if needed) and return it."""
        if entity.id == 0:
            entity.id = self.new_entity_id()
        else:
            self._next_entity_id = max(self._next_entity_id, entity.id + 1)
        self.entities[entity.id] = entity
        return entity

    def add_constraint(self, constraint: Constraint) -> Constraint:
        if constraint.id == 0:
            constraint.id = self.new_constraint_id()
        else:
            self._next_constraint_id = max(
                self._next_constraint_id, constraint.id + 1)
        self.constraints.append(constraint)
        return constraint

    def remove_entity(self, entity_id: EntityId) -> None:
        """Remove an entity and all constraints touching it.

        For `Well` entities the underlying `center` Point is also removed
        unless another entity still references it. Constraints touching
        either id are dropped.
        """
        ent = self.entities.pop(entity_id, None)
        if ent is None:
            return

        removed_ids: set[EntityId] = {entity_id}

        # Drop dependent center points for wells.
        if isinstance(ent, Well):
            center_id = ent.center
            still_referenced = any(
                getattr(e, "center", None) == center_id
                or getattr(e, "p1", None) == center_id
                or getattr(e, "p2", None) == center_id
                for e in self.entities.values()
            )
            if not still_referenced:
                self.entities.pop(center_id, None)
                removed_ids.add(center_id)

        # Drop constraints touching any removed id.
        self.constraints = [
            c for c in self.constraints
            if not (removed_ids & set(c.refs))
        ]

        # Remove from group memberships.
        for grp in self.entities.values():
            if isinstance(grp, Group):
                grp.members = [m for m in grp.members if m not in removed_ids]

    def remove_constraint(self, constraint_id: int) -> None:
        self.constraints = [c for c in self.constraints if c.id != constraint_id]

    # ── High-level construction (designer tool helpers) ───────────

    def ensure_outline_origin(self) -> Point:
        """Ensure the outline has a ground-anchored origin Point at (0, 0)."""
        origin = self.entities.get(self.outline.origin_point)
        if not isinstance(origin, Point):
            origin = Point(id=self.new_entity_id(), x=0.0, y=0.0, fixed=True)
            self.entities[origin.id] = origin
            self.outline.origin_point = origin.id
            self.add_constraint(Constraint(
                kind="ground", refs=[origin.id], weight=1.0))
        return origin

    def add_well(
        self,
        x: float,
        y: float,
        diameter: float,
        name: str,
        well_depth_mm: Optional[float] = None,
        group: Optional[EntityId] = None,
        naming_scheme: str = "ANSI",
    ) -> Well:
        """Create a Well + its center Point. Returns the Well."""
        center = self.add_entity(Point(x=x, y=y))
        well = self.add_entity(Well(
            name=name,
            center=center.id,
            diameter=diameter,
            well_depth_mm=(
                well_depth_mm if well_depth_mm is not None
                else self.well_depth_default_mm),
            group=group,
            naming_scheme=naming_scheme,
        ))
        return well  # type: ignore[return-value]

    def rebuild_group(self, group_id: EntityId, new_params: dict) -> Group:
        """Replace a group's wells with a fresh batch using `new_params`.

        Drops the group's existing member wells (and orphan center points,
        plus any constraints touching them) and re-creates wells per the
        group's `pattern_kind`. Constraints external to the group are
        preserved when their refs are still valid.

        Returns the same `Group` (id unchanged) with updated `members` /
        `params`.

        v7.4.5: powers the "Edit group" properties panel.
        """
        group = self.entities.get(group_id)
        if not isinstance(group, Group):
            raise ValueError(f"Entity {group_id} is not a Group")

        # Snapshot what we want to drop, then drop.
        old_members = list(group.members)
        for mid in old_members:
            self.remove_entity(mid)
        group.members = []
        group.params = dict(new_params)

        kind = group.pattern_kind
        depth = self.well_depth_default_mm
        if kind == "grid":
            rows = int(new_params["rows"])
            cols = int(new_params["cols"])
            sx = float(new_params["spacing_x"])
            sy = float(new_params["spacing_y"])
            ox = float(new_params.get("origin_x", 0.0))
            oy = float(new_params.get("origin_y", 0.0))
            diam = float(new_params["diameter"])
            for r in range(rows):
                for c in range(cols):
                    row_letter = (
                        ROW_LABELS[r] if r < len(ROW_LABELS) else f"R{r}")
                    name = f"{row_letter}{c + 1}"
                    well = self.add_well(
                        x=ox + c * sx, y=oy + r * sy,
                        diameter=diam, name=name,
                        well_depth_mm=depth, group=group.id,
                        naming_scheme="ANSI",
                    )
                    group.members.append(well.id)
        elif kind == "circle":
            count = int(new_params["count"])
            cx = float(new_params["center_x"])
            cy = float(new_params["center_y"])
            r = float(new_params["radius"])
            diam = float(new_params["diameter"])
            start_deg = float(new_params.get("start_angle_deg", 0.0))
            center_well = bool(new_params.get("center_well", False))
            start_rad = math.radians(start_deg)
            # v7.4.8: optional center well first (so it letters as 'a').
            if center_well:
                well = self.add_well(
                    x=cx, y=cy, diameter=diam, name=f"{group.name}-c",
                    well_depth_mm=depth, group=group.id,
                    naming_scheme="MANUAL")
                group.members.append(well.id)
            for i in range(count):
                angle = start_rad + (2 * math.pi * i) / max(count, 1)
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                well = self.add_well(
                    x=x, y=y, diameter=diam,
                    name=f"{group.name}-{i + 1}",
                    well_depth_mm=depth, group=group.id,
                    naming_scheme="MANUAL",
                )
                group.members.append(well.id)
        else:
            raise ValueError(
                f"rebuild_group: unsupported pattern_kind '{kind}'")
        return group

    def update_circle_layout(
        self, group_id: EntityId, radius: float, start_angle_deg: float,
    ) -> None:
        """Reposition an existing circle group's ring wells in place — no
        delete/recreate, so ids + names survive (smooth live dragging of
        the radius handle). v7.4.8. Center well (if any) stays put.
        """
        grp = self.entities.get(group_id)
        if not isinstance(grp, Group) or grp.pattern_kind != "circle":
            return
        p = grp.params
        p["radius"] = radius
        p["start_angle_deg"] = start_angle_deg
        cx = float(p.get("center_x", 0.0))
        cy = float(p.get("center_y", 0.0))
        count = max(int(p.get("count", 1)), 1)
        has_center = bool(p.get("center_well", False))
        ring_ids = grp.members[1:] if has_center else list(grp.members)
        start = math.radians(start_angle_deg)
        for i, mid in enumerate(ring_ids):
            well = self.entities.get(mid)
            if not isinstance(well, Well):
                continue
            pt = self.entities.get(well.center)
            if not isinstance(pt, Point):
                continue
            ang = start + (2 * math.pi * i) / count
            pt.x = cx + radius * math.cos(ang)
            pt.y = cy + radius * math.sin(ang)

    def add_circle_pattern(
        self,
        count: int,
        center_x: float,
        center_y: float,
        radius: float,
        diameter: float,
        start_angle_deg: float = 0.0,
        group_name: str = "ring",
        well_depth_mm: Optional[float] = None,
        center_well: bool = False,
    ) -> Group:
        """N equally-spaced wells on a circle, optionally plus a center well.

        v7.4.8: `center_well` adds a well at the ring centre (listed first
        so it letters as 'a' after `relabel_wells_as_letters`).
        """
        if count < 1:
            raise ValueError(f"count must be ≥ 1; got {count}")
        members: list[EntityId] = []
        group = self.add_entity(Group(
            name=group_name,
            members=[],
            pattern_kind="circle",
            params={
                "count": count,
                "center_x": center_x, "center_y": center_y,
                "radius": radius, "diameter": diameter,
                "start_angle_deg": start_angle_deg,
                "center_well": center_well,
            },
        ))
        if center_well:
            cw = self.add_well(
                x=center_x, y=center_y, diameter=diameter,
                name=f"{group_name}-c", well_depth_mm=well_depth_mm,
                group=group.id, naming_scheme="MANUAL")
            members.append(cw.id)
        start_rad = math.radians(start_angle_deg)
        for i in range(count):
            angle = start_rad + (2 * math.pi * i) / count
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            well = self.add_well(
                x=x, y=y, diameter=diameter,
                name=f"{group_name}-{i + 1}",
                well_depth_mm=well_depth_mm,
                group=group.id,
                naming_scheme="MANUAL",
            )
            members.append(well.id)
        group.members = members  # type: ignore[union-attr]
        return group  # type: ignore[return-value]

    def add_grid(
        self,
        rows: int,
        cols: int,
        spacing_x: float,
        spacing_y: float,
        origin_x: float,
        origin_y: float,
        diameter: float,
        group_name: str = "grid",
        well_depth_mm: Optional[float] = None,
    ) -> Group:
        """Create a grid of wells with ANSI-style auto names (A1, A2, ...)."""
        members: list[EntityId] = []
        group = self.add_entity(Group(
            name=group_name,
            members=[],
            pattern_kind="grid",
            params={
                "rows": rows, "cols": cols,
                "spacing_x": spacing_x, "spacing_y": spacing_y,
                "origin_x": origin_x, "origin_y": origin_y,
                "diameter": diameter,
            },
        ))
        for r in range(rows):
            for c in range(cols):
                row_letter = ROW_LABELS[r] if r < len(ROW_LABELS) else f"R{r}"
                name = f"{row_letter}{c + 1}"
                well = self.add_well(
                    x=origin_x + c * spacing_x,
                    y=origin_y + r * spacing_y,
                    diameter=diameter,
                    name=name,
                    well_depth_mm=well_depth_mm,
                    group=group.id,
                    naming_scheme="ANSI",
                )
                members.append(well.id)
        group.members = members  # type: ignore[union-attr]
        return group  # type: ignore[return-value]

    def get_wells(self) -> list[Well]:
        """All non-construction wells, sorted by id."""
        return sorted(
            (e for e in self.entities.values()
             if isinstance(e, Well) and not e.construction),
            key=lambda w: w.id,
        )

    def relabel_wells_as_letters(self) -> None:
        """Rename wells a, b, c, … aa, ab … in id (creation) order.

        v7.4.8: used for rosette sub-wells so flattened names read
        ``A1.a``, ``A1.b``. Append-only id order keeps existing letters
        stable when new sub-wells are added.
        """
        for idx, w in enumerate(self.get_wells()):
            n = idx + 1
            s = ""
            while n > 0:
                n, rem = divmod(n - 1, 26)
                s = chr(ord("a") + rem) + s
            w.name = s
            w.naming_scheme = "MANUAL"

    def get_well_position(self, well: Well) -> tuple[float, float]:
        """Resolve a Well's center coordinates from its Point."""
        center = self.entities.get(well.center)
        if not isinstance(center, Point):
            raise ValueError(
                f"Well '{well.name}' (id={well.id}) has invalid center "
                f"id={well.center}")
        return (center.x, center.y)

    # ── Compile → WellPlate ───────────────────────────────────────

    def compile(self) -> WellPlate:
        """Resolve all wells to absolute positions and emit a WellPlate.

        v7.4.8: a well carrying a `rosette_design` is **flattened** — its
        parent is NOT emitted; instead each sub-well becomes a first-class
        `WellInfo` named ``<parent>.<sub>`` (e.g. ``A1.a``), positioned at
        the parent centre plus the sub-well offset rotated by
        ``rosette_rotation_deg``, inheriting the parent's row/col. Insert
        geometry (rim height, ink Z) rides along. Downstream printing /
        ink-assignment / workflows then treat sub-wells as ordinary wells.
        """
        parents = self.get_wells()
        if not parents:
            raise ValueError(
                f"Cannot compile empty design '{self.name}' — add at "
                f"least one well")

        # Resolve parent positions + assign row/col by Y-then-X clustering.
        parent_pos: list[tuple[Well, float, float]] = [
            (w, *self.get_well_position(w)) for w in parents
        ]
        rows_assigned = _cluster_to_rows_cols(parent_pos)

        well_infos: list[WellInfo] = []
        for (w, px, py), (r_idx, c_idx) in zip(parent_pos, rows_assigned):
            ros = w.rosette_design
            if ros is None or not ros.get_wells():
                # Ordinary well (or an empty rosette → treat as ordinary).
                well_infos.append(WellInfo(
                    name=w.name, row=r_idx, col=c_idx, x=px, y=py,
                    diameter=w.diameter,
                    bottom_z_offset=w.bottom_z_offset,
                    rim_height_mm=w.rim_height_mm,
                    ink_z_mm=w.ink_z_mm,
                ))
                continue

            # Flatten the rosette into sub-wells (parent is dropped).
            theta = math.radians(w.rosette_rotation_deg)
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            for sub in ros.get_wells():
                sx, sy = ros.get_well_position(sub)
                # Rotate the offset (CCW-positive) about the well centre.
                rx = sx * cos_t - sy * sin_t
                ry = sx * sin_t + sy * cos_t
                well_infos.append(WellInfo(
                    name=f"{w.name}.{sub.name}",
                    row=r_idx, col=c_idx,
                    x=px + rx, y=py + ry,
                    diameter=sub.diameter,
                    bottom_z_offset=sub.bottom_z_offset,
                    rim_height_mm=sub.rim_height_mm,
                    ink_z_mm=sub.ink_z_mm,
                    is_subwell=True,
                    parent_well=w.name,
                ))

        if not well_infos:
            raise ValueError(
                f"Cannot compile '{self.name}' — every well is an empty "
                f"rosette")

        return WellPlate.from_wells(
            name=self.name,
            wells=well_infos,
            well_depth_mm=self.well_depth_default_mm,
            a1_offset_x=self.a1_offset_x,
            a1_offset_y=self.a1_offset_y,
            description=self.description,
        )

    # ── Rosette helpers (v7.4.7) ──────────────────────────────────

    @classmethod
    def blank_rosette(cls, bore_radius_mm: float,
                      name: str = "rosette") -> "PlateDesign":
        """A fresh nested design with a circular outline (a well bore).

        Origin (0,0) = the parent well's center; sub-wells are placed
        inside the bore. No sub-wells initially.
        """
        design = cls(name=name)
        design.outline.kind = "circle"
        design.outline.radius = bore_radius_mm
        design.outline.width = bore_radius_mm * 2
        design.outline.height = bore_radius_mm * 2
        design.ensure_outline_origin()
        return design

    def to_rosette_insert(
        self, name: str = "rosette", well_format: int = 96,
    ) -> "Any":
        """Compile this nested design into a `RosetteInsert`.

        Sub-well positions (relative to well center at origin) are
        converted to the RosetteInsert polar convention: 0° = +Y
        (12 o'clock), via `(r·sinθ, r·cosθ)` → `θ = atan2(x, y)`.
        A sub-well at the origin (r ≈ 0) is treated as the center well.
        """
        from SupportClasses.PhysicalModels import (
            RosetteInsert, RosetteSubWell,
        )

        subwells: list = []
        ring_radii: list[float] = []
        has_center = False
        sub_diam = 0.0
        for i, w in enumerate(self.get_wells()):
            x, y = self.get_well_position(w)
            r = math.hypot(x, y)
            angle = math.degrees(math.atan2(x, y))  # 0° = +Y
            if r < 1e-6:
                has_center = True
            else:
                ring_radii.append(r)
            sub_diam = max(sub_diam, w.diameter)
            subwells.append(RosetteSubWell(
                index=i,
                angle_deg=angle,
                radial_offset_mm=r,
                diameter_mm=w.diameter,
                depth_mm=w.well_depth_mm,
                z_offset_mm=w.bottom_z_offset,
            ))

        ring_radius = (sorted(ring_radii)[len(ring_radii) // 2]
                       if ring_radii else 0.0)
        ring_count = len(ring_radii)
        return RosetteInsert(
            name=name,
            well_format=well_format,
            num_subwells=ring_count,
            has_center_well=has_center,
            ring_radius_mm=ring_radius,
            subwell_diameter_mm=sub_diam,
            subwell_depth_mm=(subwells[0].depth_mm if subwells else 1.0),
            subwells=subwells,
        )

    # ── Standard-format factory ───────────────────────────────────

    @classmethod
    def blank(
        cls,
        name: str = "Untitled",
        width: float = 127.76,
        height: float = 85.48,
        a1_offset_x: float = 14.38,
        a1_offset_y: float = 11.24,
    ) -> "PlateDesign":
        """A fresh plate with the ANSI footprint outline + ground origin
        but no wells. v7.4.7 — starting point for designing from scratch.
        """
        design = cls(
            name=name,
            a1_offset_x=a1_offset_x,
            a1_offset_y=a1_offset_y,
        )
        design.outline.kind = "rect"
        design.outline.width = width
        design.outline.height = height
        design.ensure_outline_origin()
        return design

    @classmethod
    def from_standard_format(cls, well_count: int) -> "PlateDesign":
        """Clone a standard ANSI/SLAS format into an editable design.

        Geometry mirrors `WellPlate.from_format(well_count)` exactly, so
        compiling the returned design yields an equivalent plate.
        """
        if well_count not in PLATE_DEFINITIONS:
            raise ValueError(
                f"Unsupported standard format: {well_count}")
        spec = PLATE_DEFINITIONS[well_count]

        design = cls(
            name=f"{well_count}-well",
            description=spec.get("description", ""),
            a1_offset_x=spec["a1_offset_x"],
            a1_offset_y=spec["a1_offset_y"],
            well_depth_default_mm=spec.get("well_depth_mm", 10.67),
        )
        design.ensure_outline_origin()

        design.add_grid(
            rows=spec["rows"],
            cols=spec["cols"],
            spacing_x=spec["well_spacing_x"],
            spacing_y=spec["well_spacing_y"],
            origin_x=0.0,
            origin_y=0.0,
            diameter=spec["well_diameter"],
            group_name=f"grid-{well_count}",
            well_depth_mm=spec.get("well_depth_mm", 10.67),
        )
        return design

    # ── Serialization ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "description": self.description,
            "units": self.units,
            "outline": self.outline.to_dict(),
            "entities": {
                str(eid): _entity_to_dict(e)
                for eid, e in self.entities.items()
            },
            "constraints": [
                c.to_dict() for c in self.constraints if _is_persistable(c)
            ],
            "a1_offset_x": self.a1_offset_x,
            "a1_offset_y": self.a1_offset_y,
            "well_depth_default_mm": self.well_depth_default_mm,
            "_next_entity_id": self._next_entity_id,
            "_next_constraint_id": self._next_constraint_id,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "PlateDesign":
        design = cls(
            schema_version=data.get("schema_version", "1.0"),
            name=data.get("name", "Custom Plate"),
            description=data.get("description", ""),
            units=data.get("units", "mm"),
            outline=PlateOutline.from_dict(data.get("outline", {})),
            a1_offset_x=data.get("a1_offset_x", 14.38),
            a1_offset_y=data.get("a1_offset_y", 11.24),
            well_depth_default_mm=data.get("well_depth_default_mm", 10.67),
        )
        for eid_str, edata in data.get("entities", {}).items():
            ent = _entity_from_dict(edata)
            ent.id = int(eid_str)
            design.entities[ent.id] = ent

        for cdata in data.get("constraints", []):
            con = Constraint.from_dict(cdata)
            if not _is_persistable(con):
                # An already-contaminated file, or a hand edit. Dropping on read
                # is what heals the seven ghosts that reached disk before v7.12.
                logger.warning(
                    "Dropping non-persistable constraint on load: "
                    "kind=%s id=%s weight=%s", con.kind, con.id, con.weight)
                continue
            design.constraints.append(con)

        # Restore id counters; fall back to max(id) + 1.
        max_eid = max(design.entities.keys(), default=0)
        design._next_entity_id = max(
            data.get("_next_entity_id", 1), max_eid + 1)
        max_cid = max((c.id for c in design.constraints), default=0)
        design._next_constraint_id = max(
            data.get("_next_constraint_id", 1), max_cid + 1)
        return design

    def save(self, path: Optional[Path] = None) -> Path:
        """Save to JSON. Default location: USER_PLATES_DIR/<name>.json."""
        if path is None:
            USER_PLATES_DIR.mkdir(parents=True, exist_ok=True)
            path = USER_PLATES_DIR / f"{self.name}.json"
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        logger.info(f"PlateDesign saved to {path}")
        return path

    @classmethod
    def load(cls, name_or_path: str | Path) -> "PlateDesign":
        """Load a saved design from `USER_PLATES_DIR/<name>.json` or a path."""
        path = Path(name_or_path)
        if not path.suffix:
            path = USER_PLATES_DIR / f"{name_or_path}.json"
        with open(path) as f:
            data = json.load(f)
        return cls.from_dict(data)


def _entity_to_dict(entity: Entity) -> dict:
    """Serialize one entity, embedding its discriminator."""
    base = {
        "type": entity.TYPE,
        "id": entity.id,
        "construction": entity.construction,
        "fixed": entity.fixed,
    }
    if isinstance(entity, Point):
        base.update({"x": entity.x, "y": entity.y})
    elif isinstance(entity, Line):
        base.update({"p1": entity.p1, "p2": entity.p2})
    elif isinstance(entity, Circle):
        base.update({"center": entity.center, "radius": entity.radius})
    elif isinstance(entity, Well):
        base.update({
            "name": entity.name, "center": entity.center,
            "diameter": entity.diameter,
            "well_depth_mm": entity.well_depth_mm,
            "bottom_z_offset": entity.bottom_z_offset,
            "group": entity.group,
            "naming_scheme": entity.naming_scheme,
            # v7.4.7: nested rosette design (recursive) or None.
            "rosette_design": (
                entity.rosette_design.to_dict()
                if entity.rosette_design is not None else None),
            # v7.4.8: rosette rotation + insert geometry.
            "rosette_rotation_deg": entity.rosette_rotation_deg,
            "rim_height_mm": entity.rim_height_mm,
            "ink_z_mm": entity.ink_z_mm,
        })
        # v7.5.x: only emit the well-type id when set → byte-identical
        # legacy output for wells with no stamped preset.
        if entity.well_type_id is not None:
            base["well_type_id"] = entity.well_type_id
    elif isinstance(entity, Group):
        base.update({
            "name": entity.name,
            "members": list(entity.members),
            "pattern_kind": entity.pattern_kind,
            "params": dict(entity.params),
        })
    return base


def _entity_from_dict(data: dict) -> Entity:
    """Inverse of `_entity_to_dict`."""
    t = data.get("type", "Point")
    cls = _ENTITY_TYPES.get(t, Point)
    common = {
        "id": data.get("id", 0),
        "construction": data.get("construction", False),
        "fixed": data.get("fixed", False),
    }
    if cls is Point:
        return Point(**common, x=data.get("x", 0.0), y=data.get("y", 0.0))
    if cls is Line:
        return Line(**common, p1=data.get("p1", 0), p2=data.get("p2", 0))
    if cls is Circle:
        return Circle(
            **common, center=data.get("center", 0),
            radius=data.get("radius", 1.0))
    if cls is Well:
        rosette_data = data.get("rosette_design")
        return Well(
            **common,
            name=data.get("name", ""),
            center=data.get("center", 0),
            diameter=data.get("diameter", 6.35),
            well_depth_mm=data.get("well_depth_mm", 10.67),
            bottom_z_offset=data.get("bottom_z_offset", 0.0),
            group=data.get("group"),
            naming_scheme=data.get("naming_scheme", "ANSI"),
            rosette_design=(
                PlateDesign.from_dict(rosette_data)
                if rosette_data else None),
            rosette_rotation_deg=data.get("rosette_rotation_deg", 0.0),
            rim_height_mm=data.get("rim_height_mm", 0.0),
            ink_z_mm=data.get("ink_z_mm"),
            well_type_id=data.get("well_type_id"),
        )
    if cls is Group:
        return Group(
            **common,
            name=data.get("name", "group"),
            members=list(data.get("members", [])),
            pattern_kind=data.get("pattern_kind", "free"),
            params=dict(data.get("params", {})),
        )
    raise ValueError(f"Unknown entity type: {t}")


# ═══════════════════════════════════════════════════════════════════
# Row/col clustering for compile()
# ═══════════════════════════════════════════════════════════════════

def _cluster_to_rows_cols(
    well_pos: list[tuple[Well, float, float]],
) -> list[tuple[int, int]]:
    """Assign each well a (row, col) index by Y-then-X binning.

    Tolerance is half the minimum pairwise distance between wells (so
    adjacent rows in a regular grid never merge). Returns row/col indices
    in the same order as `well_pos`.
    """
    if not well_pos:
        return []

    if len(well_pos) == 1:
        return [(0, 0)]

    # Minimum pairwise distance → tolerance for Y/X binning.
    min_d = math.inf
    for i in range(len(well_pos)):
        _, xi, yi = well_pos[i]
        for j in range(i + 1, len(well_pos)):
            _, xj, yj = well_pos[j]
            d = math.hypot(xi - xj, yi - yj)
            if d < min_d:
                min_d = d
    tol = max(min_d / 2.0, 1e-6)

    # Bin Y values → row indices (sorted top-to-bottom: smaller Y = earlier row).
    ys_sorted = sorted({round(y, 6) for _, _, y in well_pos})
    row_centers: list[float] = []
    for y in ys_sorted:
        if not row_centers or abs(y - row_centers[-1]) > tol:
            row_centers.append(y)

    def row_index(y: float) -> int:
        # Pick the nearest center index.
        return min(range(len(row_centers)),
                   key=lambda i: abs(y - row_centers[i]))

    # For each well, compute its row index, then col index within row by X.
    # First pass: row indices for everyone.
    row_indices = [row_index(y) for _, _, y in well_pos]

    # Per-row, sort by X to derive col indices.
    by_row: dict[int, list[int]] = {}
    for i, ri in enumerate(row_indices):
        by_row.setdefault(ri, []).append(i)
    col_indices = [0] * len(well_pos)
    for ri, idx_list in by_row.items():
        idx_list.sort(key=lambda i: well_pos[i][1])  # sort by X
        for ci, i in enumerate(idx_list):
            col_indices[i] = ci

    return list(zip(row_indices, col_indices))
