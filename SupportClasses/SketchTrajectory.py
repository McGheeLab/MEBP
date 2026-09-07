"""
SketchTrajectory.py — draw-to-print backend for the Print Builder (v7.5.x).

The Print Builder's Sketch tool lets users *draw* a 2D print pattern from
vector primitives (line / rect / circle / ellipse / polygon), optionally
filled, and stacked across N Z-layers. This module is the headless model +
compiler: it turns a :class:`Sketch` into an Nx7 trajectory
``[x, y, z, p1, p2, p3, t]`` (mm + seconds) — the same column contract every
other MEBP trajectory uses — so the result can be baked into a ``csv_import``
print object and consumed by Print Setup unchanged.

Geometry is built on the existing low-level generators in
:mod:`SupportClasses.GeometryEngine` (``generate_circle``, ``generate_line``,
``generate_*_meander_fill``, …) so shapes match the rest of the pipeline.

The compiler also returns a parallel ``pump_states`` list (per-waypoint flow
fractions) used purely for the 2D preview colouring — it mirrors the
travel-vs-print convention of :class:`ImagePathPlanner`.
"""

from __future__ import annotations

import copy
import logging
import math
from dataclasses import dataclass, field

import numpy as np

from SupportClasses.GeometryEngine import (
    generate_line,
    generate_circle,
    generate_ellipse,
    generate_meander_fill,
    generate_circular_meander_fill,
    generate_elliptical_meander_fill,
)
from SupportClasses.PhysicalModels import (
    needle_orifice_area_mm2,
    needle_orifice_id_um,
    needle_orifice_od_mm,
)

logger = logging.getLogger(__name__)

# Shape kinds understood by the canvas + compiler. ``region`` is a baked
# paint-bucket fill (its meander toolpath is stored in ``points``). ``travel``
# is a user-placed retract-&-move point (no geometry — the needle lifts and
# travels to its ``cx,cy``); it prints nothing and splits the toolpath into
# separate runs (used to bound back-trace passes).
SHAPE_KINDS = ("line", "rect", "circle", "ellipse", "polygon", "region",
               "travel")

# Palette for abstract inks. The first three mirror the legacy pump colours
# (P1/P2/P3) so migrated sketches keep their look; the rest give >3 inks
# visually distinct hues in the editor.
_INK_PALETTE = ["#89b4fa", "#a6e3a1", "#f9b387", "#f38ba8", "#cba6f7",
                "#f9e2af", "#94e2d5", "#eba0ac", "#74c7ec", "#fab387"]


# ═══════════════════════════════════════════════════════════════════
# Data model
# ═══════════════════════════════════════════════════════════════════

@dataclass
class SketchInk:
    """One abstract ink the sketch uses. The sketch is pump-AGNOSTIC — it only
    declares the inks it needs (an ordered list, unlimited count) and colours;
    a physical pump is chosen later at print time (Quick Print maps each
    abstract ink → a configured ink → a pump). ``id`` is a stable int assigned
    by :class:`Sketch` (never recycled) so shapes reference an ink robustly
    across reordering / deletion."""

    id: int = 1
    name: str = "Ink 1"
    color: str = "#89b4fa"

    def to_dict(self) -> dict:
        return {"id": int(self.id), "name": str(self.name),
                "color": str(self.color)}

    @classmethod
    def from_dict(cls, d: dict) -> "SketchInk":
        return cls(id=int(d.get("id", 1)), name=str(d.get("name", "")),
                   color=str(d.get("color", "#89b4fa")))


@dataclass
class SketchShape:
    """One drawn primitive. All coordinates are in millimetres."""

    kind: str = "circle"

    # v7.5.x: stable per-shape identity for PARAMETRIC CONSTRAINTS. 0 means
    # "unassigned" (the legacy state); ids are handed out lazily by
    # ``Sketch.ensure_shape_ids()`` the first time a constraint is created, so
    # a sketch that never uses constraints serializes byte-identically to
    # legacy. Constraints MUST reference shapes by id, never list index — the
    # print-order optimizer reorders the shape list.
    id: int = 0

    # Center-based shapes (circle / ellipse / rect)
    cx: float = 0.0
    cy: float = 0.0
    radius: float = 5.0          # circle
    rx: float = 5.0              # ellipse semi-axis x
    ry: float = 3.0              # ellipse semi-axis y
    width: float = 10.0          # rect
    height: float = 10.0         # rect

    # Vertex-based shapes (line / polygon): list of (x, y). For ``region``
    # (paint-bucket fill) this holds the baked meander toolpath.
    points: list[tuple[float, float]] = field(default_factory=list)

    filled: bool = False
    # v7.21.4: the TARGET printed bead width (mm). It is a DECLARATION, not a
    # toolpath instruction: the compiler lays exactly ONE pass per outline and
    # never synthesises adjacent parallel lines to fill this width — the
    # operator reaches it by raising ``Sketch.extrusion_multiplier`` (see
    # ``_pass_offsets``). Used for reporting / the width-vs-extrusion match
    # hint on the Sketch page.
    line_width_mm: float = 0.4
    # Abstract ink this shape prints with — a STABLE id into ``Sketch.inks``,
    # NOT a physical pump (the sketch is pump-agnostic; see SketchInk). Legacy
    # sketches migrate the old ``pump_index`` → ``ink_id`` (P1→1,P2→2,P3→3) in
    # ``from_dict``.
    ink_id: int = 1
    color: str = "#89b4fa"       # per-shape cache of the ink's colour (fast paint)

    # v7.5.x: per-shape height offset (mm) added to this shape's layer Z when
    # it prints. 0 = at the layer height (default). Non-zero is produced by a
    # back-trace return pass so it can retrace the run at a different height.
    z_offset_mm: float = 0.0
    # v7.5.x: when True the needle follows this shape's path WITHOUT extruding
    # (a move-only pass). Used by a move-only back-trace return.
    no_print: bool = False

    # v7.5.x: optional world-mm anchor for WHERE this shape's toolpath begins
    # (print continuity control). None = the geometric default (circle → angle
    # 0, rect → first corner, line → points[0]). When set, the compiler starts a
    # closed shape at the nearest ring vertex (rolling the seam) and an open
    # shape at the nearest point along the path (the trim-START; see
    # ``end_point``). Set manually (drag the green start marker) or by the path
    # optimizer. For a closed shape this only moves the seam (deposited geometry
    # unchanged); for an OPEN shape it can trim where printing begins.
    start_point: tuple[float, float] | None = None

    # v7.5.x: (OPEN shapes — line / open polygon only) world-mm anchor for where
    # printing ENDS. With ``start_point`` it selects a SUB-SEGMENT of the drawn
    # path (trims both ends; direction start→end). None = the far end (so a
    # start-only shape reduces to the legacy "choose the leading endpoint"). For
    # CLOSED shapes the end is driven by the overlap fields below, not this.
    end_point: tuple[float, float] | None = None

    # v7.5.x: (CLOSED-loop outlines only) closure OVERLAP — after the loop
    # returns to its seam the compiler continues along the path PAST the seam so
    # the deposited ink fully closes (on re-entry the needle pushes ink aside;
    # the overshoot makes it close). ``overlap_mode``:
    #   "none"     → no overlap (default; legacy-identical serialization).
    #   "needle"   → overshoot one needle OUTER Ø (fallback: one bead width).
    #   "distance" → overshoot ``overlap_distance_mm``.
    # (Replaces the retired boolean ``overlap_closure``, which overshot the
    # needle RADIUS; a legacy ``overlap_closure: true`` migrates → "needle" in
    # from_dict.) No effect on open paths / filled shapes.
    overlap_mode: str = "none"
    overlap_distance_mm: float = 0.0

    # v7.5.x: (set by the OPTIMIZER only) when not None, the compiler reaches
    # this shape's start by RETRACING along the previously-printed bead from
    # this world-mm anchor (= the previous shape's exit) instead of lifting.
    # See Sketch.overlap_travel_*. None = normal (pen-up if it doesn't weld).
    retrace_from: tuple[float, float] | None = None

    def to_dict(self) -> dict:
        d = {
            "kind": self.kind, "cx": self.cx, "cy": self.cy,
            "radius": self.radius, "rx": self.rx, "ry": self.ry,
            "width": self.width, "height": self.height,
            "points": [list(p) for p in self.points],
            "filled": self.filled, "line_width_mm": self.line_width_mm,
            "ink_id": self.ink_id, "color": self.color,
            "z_offset_mm": self.z_offset_mm, "no_print": self.no_print,
        }
        # Emit start_point only when set so a shape with the default start
        # serializes byte-identically to legacy sketches.
        if self.start_point is not None:
            d["start_point"] = [float(self.start_point[0]),
                                float(self.start_point[1])]
        if self.end_point is not None:         # OPEN-shape trim end
            d["end_point"] = [float(self.end_point[0]),
                              float(self.end_point[1])]
        if self.overlap_mode and self.overlap_mode != "none":
            d["overlap_mode"] = str(self.overlap_mode)
            if self.overlap_mode == "distance":
                d["overlap_distance_mm"] = float(self.overlap_distance_mm)
        if self.retrace_from is not None:      # only when the optimizer set it
            d["retrace_from"] = [float(self.retrace_from[0]),
                                 float(self.retrace_from[1])]
        if self.id:                            # only once constraints assigned ids
            d["id"] = int(self.id)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "SketchShape":
        sp = d.get("start_point")
        start_point = (float(sp[0]), float(sp[1])) if sp else None
        ep = d.get("end_point")
        end_point = (float(ep[0]), float(ep[1])) if ep else None
        rf = d.get("retrace_from")
        retrace_from = (float(rf[0]), float(rf[1])) if rf else None
        # Overlap: prefer the new mode; migrate a legacy overlap_closure bool.
        overlap_mode = d.get("overlap_mode")
        if overlap_mode is None:
            overlap_mode = "needle" if bool(d.get("overlap_closure", False)) \
                else "none"
        ink_id = d.get("ink_id")
        if ink_id is None:                      # legacy: pump_index → ink_id
            ink_id = int(d.get("pump_index", 0)) + 1
        else:
            ink_id = int(ink_id)
        return cls(
            id=int(d.get("id", 0)),
            kind=d.get("kind", "circle"),
            cx=float(d.get("cx", 0.0)), cy=float(d.get("cy", 0.0)),
            radius=float(d.get("radius", 5.0)),
            rx=float(d.get("rx", 5.0)), ry=float(d.get("ry", 3.0)),
            width=float(d.get("width", 10.0)),
            height=float(d.get("height", 10.0)),
            points=[tuple(p) for p in d.get("points", [])],
            filled=bool(d.get("filled", False)),
            line_width_mm=float(d.get("line_width_mm", 0.4)),
            ink_id=ink_id,
            color=d.get("color", "#89b4fa"),
            z_offset_mm=float(d.get("z_offset_mm", 0.0)),
            no_print=bool(d.get("no_print", False)),
            start_point=start_point,
            end_point=end_point,
            overlap_mode=str(overlap_mode),
            overlap_distance_mm=float(d.get("overlap_distance_mm", 0.0)),
            retrace_from=retrace_from,
        )

    def overlap_amount_mm(self, needle_od: float = 0.0,
                          bead: float = 0.4) -> float:
        """Closure overshoot (mm) for a CLOSED loop, resolved from the mode.

        ``needle`` mode = one needle OUTER Ø (fallback: one bead width when the
        Ø is unknown); ``distance`` = the typed value; ``none`` = 0. Callers
        gate on ``_is_closed_outline`` + ``not filled`` — an open / filled
        shape has nothing to over-close."""
        mode = getattr(self, "overlap_mode", "none")
        if mode == "distance":
            return max(0.0, float(self.overlap_distance_mm))
        if mode == "needle":
            od = float(needle_od or 0.0)
            return od if od > 0 else max(float(bead), 0.0)
        return 0.0


# v7.5.x: parametric-constraint kinds understood by the sketch solver
# (SupportClasses.SketchConstraintSolver). ``drag_ghost`` is transient (held
# by the solver during an interactive drag, never stored on the sketch).
SKETCH_CONSTRAINT_KINDS = (
    "coincident",       # two anchor points share a position
    "concentric",       # two centers share a position
    "horizontal",       # two points share a y (a line: its endpoints)
    "vertical",         # two points share an x
    "parallel",         # two lines' directions are parallel
    "perpendicular",    # two lines' directions are perpendicular
    "equal_length",     # two lines have equal length
    "equal_radius",     # two circles have equal radius
    "tangent",          # line↔circle or circle↔circle tangency
    "distance",         # two points at a driven distance (dimension)
    "radius",           # a circle at a driven radius (dimension)
    "point_on",         # an anchor point lies on a curve (line / circle)
    "fix",              # lock a shape's DOFs in place
)


@dataclass
class SketchConstraint:
    """One parametric constraint between shape anchors (v7.5.x).

    ``refs`` is a list of ``[shape_id, anchor]`` pairs. ``shape_id`` is the
    stable :attr:`SketchShape.id` (NEVER a list index — the optimizer reorders
    the list). ``anchor`` names a point or the entity itself:

    - ``"center"``      — circle / ellipse / rect / travel center
    - ``"p{k}"``        — line / polygon vertex ``k`` (``p0``, ``p1``, …)
    - ``"c0".."c3"``    — rect corners (TL, TR, BR, BL)
    - ``"shape"``       — the whole entity (parallel / tangent / fix / curves)

    ``value`` drives dimensions (``distance`` mm, ``radius`` mm). ``mode``
    disambiguates ``tangent`` between two circles (``"external"`` /
    ``"internal"``, chosen from the geometry when the constraint is created).
    Constraints are design-time only — they position geometry in the editor
    and never affect the compiled trajectory of the shapes as placed.
    """

    id: int = 0
    kind: str = ""
    refs: list = field(default_factory=list)     # [[shape_id, anchor], ...]
    value: float | None = None
    mode: str = ""
    weight: float = 1.0

    def to_dict(self) -> dict:
        d = {
            "id": int(self.id),
            "kind": str(self.kind),
            "refs": [[int(sid), str(anchor)] for sid, anchor in self.refs],
        }
        if self.value is not None:
            d["value"] = float(self.value)
        if self.mode:
            d["mode"] = str(self.mode)
        if self.weight != 1.0:
            d["weight"] = float(self.weight)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "SketchConstraint":
        v = d.get("value")
        return cls(
            id=int(d.get("id", 0)),
            kind=str(d.get("kind", "")),
            refs=[[int(r[0]), str(r[1])] for r in d.get("refs", [])
                  if isinstance(r, (list, tuple)) and len(r) >= 2],
            value=None if v is None else float(v),
            mode=str(d.get("mode", "")),
            weight=float(d.get("weight", 1.0)),
        )

    def shape_ids(self) -> set[int]:
        return {int(sid) for sid, _ in self.refs}


@dataclass
class Sketch:
    """A full drawing plus the parameters needed to compile a trajectory."""

    shapes: list[SketchShape] = field(default_factory=list)

    # v7.5.x: parametric constraints between shapes (see SketchConstraint).
    # Empty for every legacy sketch → serialized only when non-empty, keeping
    # legacy dicts byte-identical. ``_next_shape_id``/``_next_constraint_id``
    # are monotonic allocators (never recycled), derived on load.
    constraints: list[SketchConstraint] = field(default_factory=list)
    _next_shape_id: int = 1
    _next_constraint_id: int = 1

    # v7.5.x: abstract inks the sketch uses (ordered; unlimited count). The
    # sketch is pump-AGNOSTIC — each shape references one of these by
    # ``ink_id``; a physical pump is chosen later at print time. ``_next_ink_id``
    # is a monotonic id allocator so an ink id is never recycled after deletion.
    inks: list[SketchInk] = field(default_factory=list)
    _next_ink_id: int = 1

    # v7.5.x: ``z_start_mm`` is the first-layer print height measured UP from
    # the calibrated plate bottom (mm, ≥ 0), not an absolute Z. The page bakes
    # it into the internal zero-ref frame when the plate bottom is known.
    z_start_mm: float = 0.2
    layer_height_mm: float = 0.2
    num_layers: int = 1

    print_speed_mm_s: float = 5.0
    travel_speed_mm_s: float = 20.0
    travel_clearance_mm: float = 2.0   # lift above the top layer for travel

    # When retracting the needle out of a print, run the first
    # ``lift_slow_dist_mm`` of the lift at ``lift_slow_speed_mm_s`` so surface
    # tension / back-pressure can't peel the deposited bead up with the needle
    # at high speed; the remainder of the lift is at ``travel_speed_mm_s``.
    lift_slow_dist_mm: float = 1.0
    lift_slow_speed_mm_s: float = 1.0

    # v7.5.x: OPTIMIZER path flexibility — "retrace along an existing bead".
    # When enabled, if the next shape's start can be reached by retracing along
    # the already-printed path for ≤ ``overlap_travel_max_mm`` (same ink, same
    # print Z), the optimizer marks that connection (stamps ``retrace_from``) and
    # the compiler continues along the bead at print Z INSTEAD of lifting
    # (pen-up). During the retrace the needle can move faster
    # (``overlap_travel_speed_factor``) and/or the pump can be PAUSED
    # (``overlap_travel_pause_pump`` — deposit ~nothing but never retract, so the
    # quick-move pressure relief is not triggered). All OFF by default → the
    # compiled trajectory is byte-identical to legacy.
    overlap_travel_enabled: bool = False
    overlap_travel_max_mm: float = 5.0
    overlap_travel_pause_pump: bool = True
    overlap_travel_speed_factor: float = 1.0

    line_spacing_mm: float = 0.4       # fill raster pitch (≈ tool diameter)
    outline_points_per_mm: float = 2.0  # outline sampling resolution
    flow_factor: float = 1.0           # fallback pump growth when no syringe

    # v7.5.x: extrusion multiplier — scales the deposited volume-per-mm (and,
    # in the preview, the shaded bead-width band). 1.0× ≈ a bead the width of
    # the needle inner Ø; 0.5× thinner, 2× thicker. It changes how MUCH is
    # laid, not WHERE the needle goes (the toolpath geometry is unchanged).
    extrusion_multiplier: float = 1.0

    # v7.21.5: when True each shape's declared ``line_width_mm`` scales its OWN
    # deposition — modifier(shape) = extrusion_multiplier x line_width_mm /
    # reference_bead_width — so a sketch with several line widths prints each at
    # its own width and the per-segment profile that ships to Quick Print has
    # real content. ``extrusion_multiplier`` stays a global trim on top.
    #
    # Defaults True for a NEW sketch and **False for any sketch loaded from a
    # dict without the key** (see ``from_dict``): before v7.21.5 a shape's
    # width had no effect on volume, and new shapes were seeded from the
    # orifice OUTER diameter while 1x is the INNER diameter, so switching a
    # saved sketch on silently multiplies its deposition by od/id (~1.7x on a
    # hypodermic needle). Opting an old sketch in is the operator's call, and
    # the page states the consequence.
    width_drives_extrusion: bool = True

    # v7.5.x: single-needle vs multi-needle/channel mode.
    #   None  → AUTO: derive from the needle (num_channels ≤ 1 ⇒ single).
    #   True  → single needle: only one material at the tip at a time, so a
    #           pump/channel change between connected shapes forces a pen-up
    #           (an "ink replacement"); the compiler will NOT weld across it.
    #   False → multi (coaxial/multi-pump): channels are simultaneous, so the
    #           compiler welds across a channel change (legacy behaviour).
    # Resolve the effective bool with :meth:`is_single_needle`.
    single_needle: bool | None = None

    def is_single_needle(self, needle=None) -> bool:
        """Effective single-needle mode: the explicit flag if set, else derived
        from the needle (``num_channels`` ≤ 1 ⇒ single; unknown ⇒ multi)."""
        if self.single_needle is not None:
            return bool(self.single_needle)
        if needle is not None:
            try:
                return int(getattr(needle, "num_channels", 1) or 1) <= 1
            except (TypeError, ValueError):
                return False
        return False

    # ── Abstract-ink management ───────────────────────────────────
    def __post_init__(self):
        # Keep the ink list consistent for ANY construction path: an explicit
        # ink list is normalized; a bare ``Sketch(shapes=[...])`` (tests /
        # canvas) synthesizes inks from the shapes' ids; an empty sketch gets a
        # single default ink. (from_dict overwrites this afterwards.)
        if self.inks:
            self.ensure_default_ink()
        elif any(getattr(s, "kind", None) != "travel" for s in self.shapes):
            self.rebuild_inks_from_shapes()
        else:
            self.ensure_default_ink()

    def ensure_default_ink(self) -> None:
        """Guarantee at least one abstract ink exists (id 1). Called after
        construction / load so no code path ever sees an empty ink list."""
        if not self.inks:
            self.inks = [SketchInk(id=1, name="Ink 1", color=_INK_PALETTE[0])]
            self._next_ink_id = 2
        else:
            self._next_ink_id = max(self._next_ink_id,
                                    max(i.id for i in self.inks) + 1)

    def ink_by_id(self, ink_id: int) -> "SketchInk | None":
        for ink in self.inks:
            if ink.id == int(ink_id):
                return ink
        return None

    def ink_order_index(self, ink_id: int) -> int:
        """Position of an ink id in the ordered list — used ONLY to pick a
        preview/volume trajectory column (index % 3), never for the weld gate."""
        for i, ink in enumerate(self.inks):
            if ink.id == int(ink_id):
                return i
        return 0

    def add_ink(self, name: str | None = None,
                color: str | None = None) -> "SketchInk":
        iid = self._next_ink_id
        self._next_ink_id += 1
        idx = len(self.inks)
        ink = SketchInk(
            id=iid, name=name or f"Ink {iid}",
            color=color or _INK_PALETTE[idx % len(_INK_PALETTE)])
        self.inks.append(ink)
        return ink

    def rebuild_inks_from_shapes(self) -> None:
        """(Best-effort import path) synthesize the ink list from the ink ids
        the shapes already carry, preferring each id's first-seen shape colour,
        then a palette fallback. Ensures every referenced ink exists."""
        first_color: dict[int, str] = {}
        for s in self.shapes:
            if getattr(s, "kind", None) == "travel":
                continue
            first_color.setdefault(int(s.ink_id), s.color)
        ids = sorted(first_color) or [1]
        self.inks = [
            SketchInk(id=i, name=f"Ink {i}",
                      color=first_color.get(i)
                      or _INK_PALETTE[(i - 1) % len(_INK_PALETTE)])
            for i in ids]
        self._next_ink_id = max(ids) + 1

    # ── Parametric constraints (v7.5.x) ───────────────────────────

    def ensure_shape_ids(self) -> None:
        """Assign a stable id to every shape that lacks one (id 0). Called
        lazily the first time a constraint is created — sketches that never
        use constraints keep all ids 0 and serialize byte-identically to
        legacy. Collision-safe against explicitly loaded ids."""
        used = {int(s.id) for s in self.shapes if getattr(s, "id", 0)}
        nxt = max(self._next_shape_id, max(used) + 1 if used else 1)
        for s in self.shapes:
            if not getattr(s, "id", 0):
                s.id = nxt
                nxt += 1
        self._next_shape_id = nxt

    def shape_by_id(self, sid: int) -> "SketchShape | None":
        for s in self.shapes:
            if getattr(s, "id", 0) == int(sid):
                return s
        return None

    def shape_index_by_id(self, sid: int) -> int:
        for i, s in enumerate(self.shapes):
            if getattr(s, "id", 0) == int(sid):
                return i
        return -1

    def add_constraint(self, kind: str, refs: list, value: float | None = None,
                       mode: str = "") -> "SketchConstraint":
        """Append a constraint. ``refs`` = [[shape_id, anchor], ...] — shape
        ids must already be assigned (call :meth:`ensure_shape_ids` first)."""
        c = SketchConstraint(id=self._next_constraint_id, kind=str(kind),
                             refs=[[int(sid), str(a)] for sid, a in refs],
                             value=value, mode=str(mode or ""))
        self._next_constraint_id += 1
        self.constraints.append(c)
        return c

    def remove_constraint(self, cid: int) -> bool:
        for i, c in enumerate(self.constraints):
            if c.id == int(cid):
                self.constraints.pop(i)
                return True
        return False

    def constraint_by_id(self, cid: int) -> "SketchConstraint | None":
        for c in self.constraints:
            if c.id == int(cid):
                return c
        return None

    def constraints_referencing(self, sid: int) -> list["SketchConstraint"]:
        sid = int(sid)
        return [c for c in self.constraints if sid in c.shape_ids()]

    def prune_constraints(self) -> int:
        """Drop constraints whose referenced shapes no longer exist (or whose
        vertex anchor exceeds the shape's current vertex count). Returns the
        number removed — call after deleting shapes."""
        live = {int(s.id) for s in self.shapes if getattr(s, "id", 0)}

        def ok(c: SketchConstraint) -> bool:
            for sid, anchor in c.refs:
                sh = self.shape_by_id(sid) if int(sid) in live else None
                if sh is None:
                    return False
                a = str(anchor)
                if a.startswith("p") and a[1:].isdigit():
                    if int(a[1:]) >= len(sh.points):
                        return False
            return True

        before = len(self.constraints)
        self.constraints = [c for c in self.constraints if ok(c)]
        return before - len(self.constraints)

    def to_dict(self) -> dict:
        d = {
            "shapes": [s.to_dict() for s in self.shapes],
            "inks": [ink.to_dict() for ink in self.inks],
            "_next_ink_id": int(self._next_ink_id),
            "z_start_mm": self.z_start_mm,
            "layer_height_mm": self.layer_height_mm,
            "num_layers": self.num_layers,
            "print_speed_mm_s": self.print_speed_mm_s,
            "travel_speed_mm_s": self.travel_speed_mm_s,
            "travel_clearance_mm": self.travel_clearance_mm,
            "lift_slow_dist_mm": self.lift_slow_dist_mm,
            "lift_slow_speed_mm_s": self.lift_slow_speed_mm_s,
            "line_spacing_mm": self.line_spacing_mm,
            "outline_points_per_mm": self.outline_points_per_mm,
            "flow_factor": self.flow_factor,
            "extrusion_multiplier": self.extrusion_multiplier,
            "width_drives_extrusion": bool(self.width_drives_extrusion),
        }
        # Emit single_needle only when explicitly set (not AUTO) → legacy dicts
        # stay byte-identical.
        if self.single_needle is not None:
            d["single_needle"] = bool(self.single_needle)
        # Emit the overlap-travel settings only when the feature is ON → OFF
        # (the default) keeps legacy dicts byte-identical.
        if self.overlap_travel_enabled:
            d["overlap_travel_enabled"] = True
            d["overlap_travel_max_mm"] = float(self.overlap_travel_max_mm)
            d["overlap_travel_pause_pump"] = bool(self.overlap_travel_pause_pump)
            d["overlap_travel_speed_factor"] = \
                float(self.overlap_travel_speed_factor)
        # Emit constraints only when any exist → legacy dicts byte-identical.
        if self.constraints:
            d["constraints"] = [c.to_dict() for c in self.constraints]
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Sketch":
        sk = cls()
        sk.shapes = [SketchShape.from_dict(s) for s in d.get("shapes", [])]
        raw_inks = d.get("inks")
        if raw_inks:
            sk.inks = [SketchInk.from_dict(x) for x in raw_inks]
            sk._next_ink_id = int(d.get(
                "_next_ink_id", max((i.id for i in sk.inks), default=0) + 1))
            sk.ensure_default_ink()
        else:
            # Legacy sketch (no ink list): synthesize inks from the migrated
            # per-shape ink ids (ink_id = old pump_index + 1).
            sk.rebuild_inks_from_shapes()
        sn = d.get("single_needle")
        sk.single_needle = None if sn is None else bool(sn)
        sk.z_start_mm = float(d.get("z_start_mm", 0.2))
        sk.layer_height_mm = float(d.get("layer_height_mm", 0.2))
        sk.num_layers = int(d.get("num_layers", 1))
        sk.print_speed_mm_s = float(d.get("print_speed_mm_s", 5.0))
        sk.travel_speed_mm_s = float(d.get("travel_speed_mm_s", 20.0))
        sk.travel_clearance_mm = float(d.get("travel_clearance_mm", 2.0))
        sk.lift_slow_dist_mm = float(d.get("lift_slow_dist_mm", 1.0))
        sk.lift_slow_speed_mm_s = float(d.get("lift_slow_speed_mm_s", 1.0))
        sk.line_spacing_mm = float(d.get("line_spacing_mm", 0.4))
        sk.outline_points_per_mm = float(d.get("outline_points_per_mm", 2.0))
        sk.flow_factor = float(d.get("flow_factor", 1.0))
        sk.extrusion_multiplier = float(d.get("extrusion_multiplier", 1.0))
        # ABSENT means a pre-v7.21.5 sketch, whose stored line widths were never
        # an extrusion instruction — default OFF so re-baking it deposits
        # exactly what it always did.
        sk.width_drives_extrusion = bool(d.get("width_drives_extrusion", False))
        sk.overlap_travel_enabled = bool(d.get("overlap_travel_enabled", False))
        sk.overlap_travel_max_mm = float(d.get("overlap_travel_max_mm", 5.0))
        sk.overlap_travel_pause_pump = bool(
            d.get("overlap_travel_pause_pump", True))
        sk.overlap_travel_speed_factor = float(
            d.get("overlap_travel_speed_factor", 1.0))
        sk.constraints = [SketchConstraint.from_dict(c)
                          for c in d.get("constraints", [])]
        # Re-derive the monotonic allocators from what was loaded.
        sk._next_shape_id = max(
            (int(getattr(s, "id", 0)) for s in sk.shapes), default=0) + 1
        sk._next_constraint_id = max(
            (c.id for c in sk.constraints), default=0) + 1
        return sk

    def copy(self) -> "Sketch":
        return copy.deepcopy(self)


@dataclass
class CompiledSketch:
    """Result of :func:`compile_to_trajectory`."""

    trajectory: np.ndarray                    # Nx7 [x,y,z,p1,p2,p3,t]
    pump_states: list[list[float]]            # per-waypoint [f1,f2,f3] (preview)
    # v7.21.5 — THE EXTRUSION ALONG THE PATH, one entry per SEGMENT (waypoint
    # i -> i+1), so len == len(trajectory) - 1:
    #   ``extrusion_profile``  the extrusion MODIFIER actually applied
    #                          (bore-relative; 0.0 on a travel / no-extrude
    #                          segment), and
    #   ``vol_per_mm_profile`` the same thing in absolute microlitres per mm.
    # Both are baked into the print file so Quick Print prints what the sketch
    # computed instead of re-deriving one flow for the whole path.
    extrusion_profile: list[float] = field(default_factory=list)
    vol_per_mm_profile: list[float] = field(default_factory=list)
    #: the width that a 1x bead is (mm) — the reference the modifier is against
    extrusion_ref_width_mm: float = 0.0
    total_length_mm: float = 0.0
    total_volume_uL: float = 0.0
    total_time_s: float = 0.0
    num_layers: int = 0
    num_waypoints: int = 0
    bounds_mm: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    @property
    def is_empty(self) -> bool:
        return self.trajectory is None or len(self.trajectory) < 2


# Per-pump display colours (mirror gui.widgets.sketch_canvas.PUMP_HEX so an
# imported region reads with the right pump colour without importing the GUI).
_PUMP_COLORS = ["#89b4fa", "#a6e3a1", "#f9b387"]


def regions_from_trajectory(trajectory) -> list["SketchShape"]:
    """Best-effort: rebuild editable Sketch shapes from a baked Nx7 trajectory.

    Splits the trajectory into contiguous **printing** sub-paths (runs where a
    pump advances between consecutive points; if the pump columns are flat the
    whole path is treated as one print) and returns one ``region`` shape per
    sub-path (its ``points`` = the printed XY). This is NOT a vector
    decomposition into primitives — it lets the operator reposition / rescale /
    delete parts of an existing print that carries no stored ``Sketch`` and
    re-save it (after which the print carries a real Sketch).

    Travel segments are intentionally dropped so a re-bake doesn't extrude
    across them; pump assignment is inferred from the dominant advancing pump.
    """
    arr = np.asarray(trajectory, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2 or len(arr) < 2:
        return []

    n_seg = len(arr) - 1
    if arr.shape[1] >= 6:
        dp = np.diff(arr[:, 3:6], axis=0)          # per-segment pump delta
        printing = (dp > 1e-9).any(axis=1)
        seg_pump = np.argmax(dp, axis=1)
    else:
        printing = np.ones(n_seg, dtype=bool)
        seg_pump = np.zeros(n_seg, dtype=int)
    if not printing.any():                         # flat pumps → treat all print
        printing = np.ones(n_seg, dtype=bool)

    shapes: list[SketchShape] = []
    i = 0
    while i < n_seg:
        if not printing[i]:
            i += 1
            continue
        j = i
        while j < n_seg and printing[j]:
            j += 1
        pts = [(float(arr[k, 0]), float(arr[k, 1])) for k in range(i, j + 1)]
        if len(pts) >= 2:
            run = seg_pump[i:j]
            pump = int(np.bincount(run, minlength=3).argmax()) if len(run) else 0
            pump = max(0, min(2, pump))
            shapes.append(SketchShape(kind="region", points=pts,
                                      ink_id=pump + 1, color=_PUMP_COLORS[pump]))
        i = j
    return shapes


# ═══════════════════════════════════════════════════════════════════
# Back-trace: retrace a run reversed, offset in height + in-plane
# ═══════════════════════════════════════════════════════════════════

def offset_polyline(pts, distance: float, closed: bool = False
                    ) -> list[tuple[float, float]]:
    """Offset a polyline perpendicular to itself by ``distance`` mm (a parallel
    curve). Each vertex is pushed along its (averaged) left normal; a clamped
    miter factor keeps the spacing ≈ ``distance`` at corners. Positive shifts to
    the left of the path's travel direction, negative to the right. ``closed``
    treats the points as a loop. Degenerate/zero cases return a copy unchanged.
    """
    src = [(float(x), float(y)) for x, y in pts]
    if len(src) < 2 or abs(distance) < 1e-9:
        return src
    # Drop consecutive duplicate vertices — a zero-length segment has no normal
    # and would make the averaged normal diverge into a self-intersecting spike.
    P = [src[0]]
    for q in src[1:]:
        if math.hypot(q[0] - P[-1][0], q[1] - P[-1][1]) > 1e-9:
            P.append(q)
    n = len(P)
    if n < 2:
        return src

    def seg_normal(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < 1e-12:
            return None
        return (-dy / L, dx / L)          # left-hand normal

    out: list[tuple[float, float]] = []
    for i in range(n):
        if closed:
            n1 = seg_normal(P[(i - 1) % n], P[i])
            n2 = seg_normal(P[i], P[(i + 1) % n])
        elif i == 0:
            n1 = n2 = seg_normal(P[0], P[1])
        elif i == n - 1:
            n1 = n2 = seg_normal(P[n - 2], P[n - 1])
        else:
            n1 = seg_normal(P[i - 1], P[i])
            n2 = seg_normal(P[i], P[i + 1])
        if n1 is None and n2 is None:
            out.append(P[i])
            continue
        if n1 is None:
            n1 = n2
        if n2 is None:
            n2 = n1
        mx, my = n1[0] + n2[0], n1[1] + n2[1]
        L = math.hypot(mx, my)
        if L < 1e-9:                       # ~180° reversal → use one normal
            mx, my = n1
            L = 1.0
        mx, my = mx / L, my / L
        cos_half = mx * n1[0] + my * n1[1]     # miter half-angle cosine
        scale = min(1.0 / cos_half, 4.0) if cos_half > 0.25 else 1.0
        out.append((P[i][0] + mx * distance * scale,
                    P[i][1] + my * distance * scale))
    return out


def backtrace_shape(shape: "SketchShape", *, z_offset: float = 0.0,
                    xy_offset: float = 0.0, print_on_return: bool = True
                    ) -> "SketchShape":
    """Return a **reversed** copy of ``shape`` for a back-trace return pass.

    The copy retraces the shape in the opposite direction, raised by
    ``z_offset`` mm (added to the shape's own height offset) and shifted
    ``xy_offset`` mm perpendicular to the path (a parallel bead / return route).
    ``print_on_return=False`` makes it a move-only (non-extruding) pass.
    """
    c = copy.deepcopy(shape)
    # A back-trace copy is a NEW shape: it must not inherit the source's
    # stable constraint id (two shapes sharing an id would corrupt constraint
    # references). It starts unconstrained; ensure_shape_ids() assigns a fresh
    # id if a constraint later references it.
    c.id = 0
    # Accumulate onto the source's own offset (so back-tracing a back-trace
    # stacks), but clamp to the same ±40 mm range the UI spin allows so
    # repeatedly back-tracing a result can't run the offset (and the resulting
    # travel-Z lift) away to absurd heights.
    total = float(getattr(shape, "z_offset_mm", 0.0) or 0.0) + float(z_offset)
    c.z_offset_mm = max(-40.0, min(40.0, total))
    c.no_print = not bool(print_on_return)
    k = c.kind
    if k in ("line", "polygon", "region"):
        pts = list(shape.points)
        closed = (k == "polygon" and len(pts) >= 3)
        if abs(xy_offset) > 1e-9:
            pts = offset_polyline(pts, float(xy_offset), closed=closed)
        c.points = list(reversed(pts))          # retrace in the opposite order
    elif k == "circle":
        c.radius = max(0.05, shape.radius + float(xy_offset))
    elif k == "ellipse":
        c.rx = max(0.05, shape.rx + float(xy_offset))
        c.ry = max(0.05, shape.ry + float(xy_offset))
    elif k == "rect":
        c.width = max(0.05, shape.width + 2.0 * float(xy_offset))
        c.height = max(0.05, shape.height + 2.0 * float(xy_offset))
    # travel points aren't back-traceable — callers exclude them.
    return c


# ═══════════════════════════════════════════════════════════════════
# Start-point control: reorder a generated path to begin at a chosen point
# ═══════════════════════════════════════════════════════════════════

def reorder_path_to_start(path, start_xy, closed: bool) -> np.ndarray:
    """Reorder a generated Nx2 ``path`` so it begins at (or nearest to)
    ``start_xy`` — a print-continuity control. The deposited geometry is
    UNCHANGED; only the entry (and, for closed loops, exit) point moves.

    - ``closed`` loop (circle / ellipse / rect / closed polygon): the ring is
      rolled so its vertex nearest ``start_xy`` leads, then re-closed (the new
      first vertex is appended so the bead returns to it).
    - open path (line / open polygon): reversed iff its LAST point is closer to
      ``start_xy`` than its first — i.e. the nearer endpoint leads.

    Degenerate / tiny paths are returned unchanged.
    """
    arr = np.asarray(path, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2 or len(arr) < 2 or start_xy is None:
        return arr
    sx, sy = float(start_xy[0]), float(start_xy[1])

    if not closed:
        d0 = math.hypot(arr[0, 0] - sx, arr[0, 1] - sy)
        d1 = math.hypot(arr[-1, 0] - sx, arr[-1, 1] - sy)
        return arr[::-1].copy() if d1 < d0 else arr

    # Closed: drop a duplicate closing vertex (generators append it), roll to
    # the nearest vertex, re-close.
    core = arr
    if len(arr) > 2 and math.hypot(arr[0, 0] - arr[-1, 0],
                                   arr[0, 1] - arr[-1, 1]) < 1e-9:
        core = arr[:-1]
    if len(core) < 2:
        return arr
    d = np.hypot(core[:, 0] - sx, core[:, 1] - sy)
    i = int(np.argmin(d))
    rolled = np.roll(core, -i, axis=0)
    return np.vstack([rolled, rolled[0]])


def _cumlen(arr: np.ndarray) -> np.ndarray:
    """Cumulative arc length at each vertex of an Nx2 polyline (``[0]==0``)."""
    if len(arr) < 2:
        return np.zeros(len(arr), dtype=np.float64)
    seg = np.hypot(np.diff(arr[:, 0]), np.diff(arr[:, 1]))
    return np.concatenate([[0.0], np.cumsum(seg)])


def _project_arclen(arr: np.ndarray, cum: np.ndarray, xy) -> float:
    """Arc-length position of the point on the polyline nearest ``xy``."""
    x, y = float(xy[0]), float(xy[1])
    best_s, best_d = 0.0, None
    for i in range(len(arr) - 1):
        ax, ay = arr[i]
        bx, by = arr[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(
            1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(px - x, py - y)
        if best_d is None or d < best_d:
            best_d = d
            best_s = float(cum[i] + t * math.hypot(dx, dy))
    return best_s


def _point_at_arclen(arr: np.ndarray, cum: np.ndarray, s: float):
    """Interpolate the polyline point at arc length ``s`` (clamped to ends)."""
    L = float(cum[-1])
    s = max(0.0, min(L, float(s)))
    i = int(np.searchsorted(cum, s, side="right")) - 1
    i = max(0, min(i, len(arr) - 2))
    seg = float(cum[i + 1] - cum[i])
    t = 0.0 if seg <= 1e-12 else (s - cum[i]) / seg
    ax, ay = arr[i]
    bx, by = arr[i + 1]
    return (float(ax + t * (bx - ax)), float(ay + t * (by - ay)))


def _subpath_arclen(arr: np.ndarray, cum: np.ndarray, s0: float,
                    s1: float) -> np.ndarray:
    """The polyline from arc length ``s0`` to ``s1`` (reversed when s1<s0).

    Interpolated endpoints at exactly ``s0``/``s1`` plus every interior vertex
    strictly between them. ``s0=0``→``s1=L`` returns the original path
    unchanged (and its reverse for ``L``→``0``) so a full-range trim is a
    byte-identical no-op.
    """
    L = float(cum[-1])
    a = max(0.0, min(L, float(s0)))
    b = max(0.0, min(L, float(s1)))
    lo, hi = (a, b) if a <= b else (b, a)
    pts = [_point_at_arclen(arr, cum, lo)]
    for i in range(len(arr)):
        if lo + 1e-9 < float(cum[i]) < hi - 1e-9:
            pts.append((float(arr[i, 0]), float(arr[i, 1])))
    pts.append(_point_at_arclen(arr, cum, hi))
    out = np.asarray(pts, dtype=np.float64)
    return out[::-1].copy() if a > b else out


def trim_open_path(path, start_xy, end_xy) -> np.ndarray:
    """Trim an OPEN polyline to the sub-segment between two anchors.

    ``start_xy`` / ``end_xy`` (world mm, either may be None) are projected onto
    the path; the returned polyline runs from the start projection to the end
    projection (in that direction). ``end_xy=None`` → the FAR extreme relative
    to the start (so a start-only shape reproduces the legacy "lead with the
    nearer endpoint" flip exactly — byte-identical). Both None or degenerate →
    the path unchanged.
    """
    arr = np.asarray(path, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2 or len(arr) < 2:
        return arr
    if start_xy is None and end_xy is None:
        return arr
    cum = _cumlen(arr)
    L = float(cum[-1])
    if L <= 1e-9:
        return arr
    s_start = _project_arclen(arr, cum, start_xy) if start_xy is not None else 0.0
    if end_xy is not None:
        s_end = _project_arclen(arr, cum, end_xy)
    else:                                       # far extreme → legacy flip
        s_end = L if s_start <= L / 2.0 else 0.0
    if abs(s_end - s_start) < 1e-9:             # degenerate → don't collapse
        return arr
    return _subpath_arclen(arr, cum, s_start, s_end)


def extend_closed_path(path, overshoot: float) -> np.ndarray:
    """Continue a CLOSED loop past its closure point by ``overshoot`` mm of
    arc-length (an over-closure). The needle re-enters the start and keeps
    printing along the beginning of the loop for ``overshoot`` mm, so the seam
    fully closes (otherwise the needle pushes ink aside on re-entry and the loop
    doesn't close). ``path`` must be a closed ring (``path[-1] == path[0]``);
    returns it unchanged if degenerate / not closed / overshoot ≤ 0. Never walks
    more than one full lap.
    """
    arr = np.asarray(path, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2 or len(arr) < 3 or overshoot <= 0:
        return arr
    if math.hypot(arr[0, 0] - arr[-1, 0], arr[0, 1] - arr[-1, 1]) > 1e-6:
        return arr                             # not a closed loop → nothing to do
    extra: list[list[float]] = []
    acc = 0.0
    prev = arr[-1]                             # == arr[0], the closure point
    for i in range(1, len(arr)):               # walk forward, at most one lap
        cur = arr[i]
        seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        if seg <= 1e-12:
            continue
        if acc + seg >= overshoot:
            t = (overshoot - acc) / seg
            extra.append([prev[0] + t * (cur[0] - prev[0]),
                          prev[1] + t * (cur[1] - prev[1])])
            break
        extra.append([float(cur[0]), float(cur[1])])
        acc += seg
        prev = cur
    return np.vstack([arr, extra]) if extra else arr


def _retrace_polyline(path, from_xy, to_xy, max_mm: float):
    """Vertices ALONG ``path`` from the vertex nearest ``from_xy`` to the vertex
    nearest ``to_xy`` (a single traversal), arc-length-capped at ``max_mm``.

    Used to "retrace along an existing bead": the needle is at ``from_xy`` (the
    previous shape's exit, on ``path``) and the next shape starts at ``to_xy``
    (also on ``path``, a brief distance back). Returns the intermediate vertices
    to walk (excluding the start, since the needle is already there), or ``None``
    when the two anchors aren't distinct, lie in the wrong order for one
    traversal, or the run exceeds ``max_mm``.
    """
    arr = np.asarray(path, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2 or len(arr) < 2 or max_mm <= 0:
        return None
    i0 = int(np.hypot(arr[:, 0] - from_xy[0], arr[:, 1] - from_xy[1]).argmin())
    i1 = int(np.hypot(arr[:, 0] - to_xy[0], arr[:, 1] - to_xy[1]).argmin())
    if i0 == i1:
        return None
    step = 1 if i1 > i0 else -1
    pts: list[tuple[float, float]] = []
    acc = 0.0
    prev = arr[i0]
    k = i0
    while k != i1:
        k += step
        cur = arr[k]
        acc += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        if acc > max_mm:
            return None                          # too far to retrace ≤ max_mm
        pts.append((float(cur[0]), float(cur[1])))
        prev = cur
    return pts if pts else None


# ═══════════════════════════════════════════════════════════════════
# Per-shape footprint (one layer)  →  Nx2 path
# ═══════════════════════════════════════════════════════════════════

def _rect_outline(cx, cy, w, h, ppm) -> np.ndarray:
    hw, hh = w / 2.0, h / 2.0
    corners = [(cx - hw, cy - hh), (cx + hw, cy - hh),
               (cx + hw, cy + hh), (cx - hw, cy + hh), (cx - hw, cy - hh)]
    return _polyline(corners, ppm, closed=False)


def _polyline(pts, ppm, closed: bool) -> np.ndarray:
    """Sample a polyline (optionally closed) at ``ppm`` points per mm."""
    if len(pts) < 2:
        return np.asarray(pts, dtype=float).reshape(-1, 2)
    seq = list(pts)
    if closed and seq[0] != seq[-1]:
        seq = seq + [seq[0]]
    out = []
    for i in range(len(seq) - 1):
        (x1, y1), (x2, y2) = seq[i], seq[i + 1]
        length = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(length * ppm) + 1)
        last = (i == len(seq) - 2)
        xs = np.linspace(x1, x2, n, endpoint=last)
        ys = np.linspace(y1, y2, n, endpoint=last)
        out.append(np.column_stack([xs, ys]))
    return np.vstack(out) if out else np.empty((0, 2))


def _polygon_fill(pts, spacing) -> np.ndarray:
    """Even-odd scanline meander fill for a simple polygon."""
    if len(pts) < 3 or spacing <= 0:
        return np.empty((0, 2))
    ys = [p[1] for p in pts]
    y0, y1 = min(ys), max(ys)
    n_lines = max(1, int((y1 - y0) / spacing))
    m = len(pts)
    rows = []
    for i in range(n_lines + 1):
        y = y0 + i * spacing
        xs = []
        for j in range(m):
            ax, ay = pts[j]
            bx, by = pts[(j + 1) % m]
            if (ay <= y < by) or (by <= y < ay):
                t = (y - ay) / (by - ay)
                xs.append(ax + t * (bx - ax))
        xs.sort()
        spans = [(xs[k], xs[k + 1]) for k in range(0, len(xs) - 1, 2)]
        if i % 2 == 1:                       # serpentine
            spans = [(b, a) for (a, b) in reversed(spans)]
        for xa, xb in spans:
            n = max(2, int(abs(xb - xa) / spacing) + 1)
            xline = np.linspace(xa, xb, n)
            rows.append(np.column_stack([xline, np.full(n, y)]))
    return np.vstack(rows) if rows else np.empty((0, 2))


def _pass_offsets(line_width: float, bead: float) -> list[float]:
    """Perpendicular pass offsets for one outline — ALWAYS a single pass.

    v7.21.4 (operator): *"do not auto calculate the need for multiple lines to
    fill in the width of any line. We will set that line width and just
    increase the extrusion modifier to match."*

    Before this, an outline whose ``line_width_mm`` exceeded the fill pitch was
    silently expanded into ``round(line_width / bead)`` adjacent parallel
    passes (a 2 mm width at a 0.4 mm pitch → 5 concentric passes). That made
    ONE drawn line print as five, each offset from the geometry the operator
    drew — so the outer pass sat half the width outside the sketched shape,
    the path length (and therefore volume and time) jumped 5×, and the width
    was reached by geometry rather than by flow.

    Now the width is reached by FLOW: exactly one bead is laid on the drawn
    geometry and ``Sketch.extrusion_multiplier`` scales how much is deposited
    (the Sketch page reports the × needed to match a shape's declared
    ``line_width_mm``, and its shaded band shows the width that × predicts).

    Kept as a function — rather than deleting it — so every shape branch below
    keeps one uniform "for each pass" shape, and so re-introducing multi-pass
    (if it is ever wanted as an explicit, opt-in mode) is a change in ONE
    place instead of five.

    ``line_width``/``bead`` are accepted and ignored.
    """
    return [0.0]


# Outline kinds whose toolpath start point can be repositioned (Feature: print
# start/stop control). Fills / regions (rasters) and travel markers cannot.
_START_REORDER_KINDS = ("line", "circle", "ellipse", "rect", "polygon")


def _is_closed_outline(shape: SketchShape) -> bool:
    """Whether ``shape``'s (unfilled) outline is a closed loop — so a start
    point rolls the ring rather than choosing an endpoint."""
    k = shape.kind
    if k in ("circle", "ellipse", "rect"):
        return True
    if k == "polygon":
        return len(shape.points) >= 3
    return False                              # line / open polygon


def _shape_paths(shape: SketchShape, sketch: Sketch) -> list[np.ndarray]:
    """Build the print path(s) for one shape (single layer), honouring an
    optional per-shape ``start_point``.

    Thin wrapper over :func:`_shape_paths_raw`; when the shape carries a
    ``start_point`` and is an unfilled outline, each generated pass is reordered
    to begin at that point (see :func:`reorder_path_to_start`). Filled shapes,
    ``region`` rasters and ``travel`` markers are never reordered.
    """
    paths = _shape_paths_raw(shape, sketch)
    if shape.kind not in _START_REORDER_KINDS or shape.filled:
        return paths
    sp = getattr(shape, "start_point", None)
    ep = getattr(shape, "end_point", None)
    if _is_closed_outline(shape):
        # Closed loop: the start rolls the seam; the end is handled by the
        # closure overlap (extend_closed_path), NOT here.
        if sp is not None:
            paths = [reorder_path_to_start(p, sp, True)
                     if p is not None and len(p) >= 2 else p for p in paths]
    elif sp is not None or ep is not None:
        # Open path: start + end anchors trim it to a sub-segment (end unset →
        # legacy "lead with the nearer endpoint" flip).
        paths = [trim_open_path(p, sp, ep)
                 if p is not None and len(p) >= 2 else p for p in paths]
    return paths


def _shape_paths_raw(shape: SketchShape, sketch: Sketch) -> list[np.ndarray]:
    """Build the print path(s) for one shape (single layer).

    Returns a list of Nx2 paths — one per side-by-side pass for thick
    outlines, a single path for fills / regions / thin outlines.
    """
    k = shape.kind
    sp = max(sketch.line_spacing_mm, 1e-3)
    ppm = max(sketch.outline_points_per_mm, 0.1)
    offsets = _pass_offsets(shape.line_width_mm, sp)

    if k == "travel":
        return []                                # no printed geometry

    if k == "region":
        pts = np.asarray(shape.points, dtype=float)
        return [pts] if len(pts) >= 1 else []

    if k == "line":
        if len(shape.points) < 2:
            return []
        (x1, y1), (x2, y2) = shape.points[0], shape.points[1]
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) * ppm) + 1)
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / length, dx / length     # unit normal
        return [generate_line(x1 + o * nx, y1 + o * ny,
                              x2 + o * nx, y2 + o * ny, n) for o in offsets]

    if k == "circle":
        if shape.radius <= 0:
            return []
        if shape.filled:
            return [generate_circular_meander_fill(
                shape.cx, shape.cy, shape.radius, sp)]
        out = []
        for o in offsets:
            r = shape.radius + o
            if r <= 0.05:
                continue
            n = max(16, int(2 * math.pi * r * ppm))
            out.append(generate_circle(shape.cx, shape.cy, r, n))
        return out

    if k == "ellipse":
        if shape.rx <= 0 or shape.ry <= 0:
            return []
        if shape.filled:
            return [generate_elliptical_meander_fill(
                shape.cx, shape.cy, shape.rx, shape.ry, sp)]
        out = []
        for o in offsets:
            rx, ry = shape.rx + o, shape.ry + o
            if rx <= 0.05 or ry <= 0.05:
                continue
            n = max(16, int(math.pi * (rx + ry) * ppm))
            out.append(generate_ellipse(shape.cx, shape.cy, rx, ry, n))
        return out

    if k == "rect":
        if shape.width <= 0 or shape.height <= 0:
            return []
        if shape.filled:
            return [generate_meander_fill(
                shape.cx, shape.cy, shape.width, shape.height, sp)]
        out = []
        for o in offsets:
            w, h = shape.width + 2 * o, shape.height + 2 * o
            if w <= 0.05 or h <= 0.05:
                continue
            out.append(_rect_outline(shape.cx, shape.cy, w, h, ppm))
        return out

    if k == "polygon":
        pts = list(shape.points)
        if len(pts) < 2:
            return []
        if shape.filled and len(pts) >= 3:
            return [_polygon_fill(pts, sp)]
        # Arbitrary-polygon offsetting is non-trivial — single pass.
        return [_polyline(pts, ppm, closed=len(pts) >= 3)]

    return []


# ═══════════════════════════════════════════════════════════════════
# Compiler
# ═══════════════════════════════════════════════════════════════════

def compile_to_trajectory(sketch: Sketch, needle=None, syringe=None
                          ) -> CompiledSketch:
    """Compile a :class:`Sketch` into an Nx7 trajectory.

    Each shape's 2D footprint is generated per layer (reversed on odd layers
    for a serpentine Z-stack), stitched together with lift→move→lower travel
    moves, and time-parameterized at ``print_speed_mm_s`` /
    ``travel_speed_mm_s``. Pump columns hold cumulative plunger displacement
    (mm); when ``needle`` + ``syringe`` are supplied the displacement is
    volume-accurate, otherwise it grows with ``flow_factor`` so the column is
    monotonic and previewable.
    """
    num_layers = max(1, int(sketch.num_layers))

    # Each printed pass lays a bead of width ``bead`` (the fill gap), so the
    # deposited cross-section per mm of travel is bead × layer_height.
    # v7.21.4: an outline is ALWAYS one pass (see ``_pass_offsets``) — a wider
    # declared ``line_width_mm`` is reached by raising the extrusion
    # multiplier, never by emitting adjacent parallel lines.
    bead = max(sketch.line_spacing_mm, 1e-3)      # fill raster pitch (geometry)
    # v7.5.x (Finding A) — UNIFY on the canonical F-1 bead model: deposited
    # volume per mm of travel = the needle's INNER-BORE cross-section × the
    # extrusion multiplier, exactly like FlowPhysics / GeometryEngine / Quick
    # Print. So a sketch-baked print and a Quick Print of the SAME needle+path
    # now extrude identically, and the baked volume no longer depends on
    # layer_height (which is only the Z step between stacked layers). The
    # multiplier scales HOW MUCH is deposited, never the toolpath geometry.
    # NOTE: the Sketch page's shaded thickness band is a display WIDTH
    # (orifice Ø × mult); the baked VOLUME here is orifice AREA × mult — width
    # vs area differ by design (mirrors Quick Print, which shows no band). When
    # no needle is supplied (bore unknown) fall back to bead × layer_height ×
    # mult so the no-syringe preview stays monotonic and previewable.
    # v7.6: "bore" is the ORIFICE — the pulled tip when present, else the barrel.
    # v7.21.5: split into a BASE (per unit modifier) and a per-shape modifier.
    # base_vol_per_mm is the deposition at modifier 1.0 — the orifice
    # cross-section — and ``_shape_modifier`` below turns each shape's declared
    # ``line_width_mm`` into its own multiple of it. ``ref_width_mm`` is the
    # width a 1.0x bead is, i.e. the reference the modifier is measured
    # against: the orifice INNER diameter, matching the Sketch page's shaded
    # band and its "this width needs Nx" readout, so what the page reports is
    # what the compiler applies.
    mult = max(float(getattr(sketch, "extrusion_multiplier", 1.0)), 0.0)
    base_vol_per_mm = bead * sketch.layer_height_mm
    ref_width_mm = bead
    if needle is not None:
        try:
            _area = float(needle_orifice_area_mm2(needle) or 0.0)
            if _area > 0:
                base_vol_per_mm = _area
        except (TypeError, ValueError):
            pass
        try:
            _id = float(needle_orifice_id_um(needle) or 0.0) / 1000.0
            if _id > 0:
                ref_width_mm = _id
        except (TypeError, ValueError):
            pass
    width_scaled = bool(getattr(sketch, "width_drives_extrusion", False))

    def _shape_modifier(shape) -> float:
        """The extrusion modifier for one shape.

        ``extrusion_multiplier`` is the sketch-wide trim; when
        ``width_drives_extrusion`` is on, a shape's declared ``line_width_mm``
        scales it by how many reference beads wide that line is — which is what
        makes the per-segment profile shipped to Quick Print meaningful, and
        what makes a declared width actually print at that width now that an
        outline is a single pass (v7.21.4).

        ``no_print`` is deliberately NOT checked here: ``move_to`` already
        zeroes a non-printing segment via its ``printing`` flag, which is the
        single enforcement point (it also covers travel, lifts and the
        hold-pressure retrace). A second check here proved to be dead code — a
        mutation removing it changed nothing.
        """
        m = mult
        if width_scaled and ref_width_mm > 0:
            try:
                lw = float(getattr(shape, "line_width_mm", 0.0) or 0.0)
            except (TypeError, ValueError):
                lw = 0.0
            if lw > 0:
                m *= lw / ref_width_mm
        return max(0.0, m)

    # The modifier of the shape currently being emitted; ``move_to`` reads it so
    # every segment is stamped with the extrusion actually applied to it.
    cur_mod = mult
    # Two paths "connect" — and print as one continuous bead with no pen-up —
    # when an endpoint of the next sits within half a bead of where the last
    # left off (their deposited beads already overlap). Half a bead keeps the
    # weld tight: a line JOINED to another shape's print start/stop on the
    # Sketch canvas lands exactly on it, so it welds; a line merely passing
    # nearby does not.
    weld_tol = max(bead * 0.5, 1e-3)
    # Plunger mm per mm of travel, at modifier 1.0 (scaled per shape below).
    if syringe is not None:
        base_pump_per_mm = base_vol_per_mm * syringe.mm_per_uL
    else:
        base_pump_per_mm = max(sketch.flow_factor, 1e-6)

    # Single-needle mode (one material at the tip at a time): a pump/channel
    # change between connected shapes cannot weld — it needs an ink replacement
    # (pen-up). In multi mode the compiler welds across channels (legacy).
    single = sketch.is_single_needle(needle)
    # Closure overlap for closed loops = the per-shape amount
    # (shape.overlap_amount_mm): needle mode → one ORIFICE OUTER Ø (the pulled
    # tip when present — the seam has to close over the bead actually laid
    # down), distance mode → the typed value. Resolved once here.
    _od = 0.0
    if needle is not None:
        try:
            _od = float(needle_orifice_od_mm(needle) or 0.0)
        except (TypeError, ValueError):
            _od = 0.0

    # Travel Z clears the tallest printed pass — including any positive
    # per-shape height offset (a back-trace return pass sits at layer Z +
    # offset), so the lift always rises above everything it might drag through.
    max_z_off = 0.0
    for sh in sketch.shapes:
        if sh.kind != "travel":
            max_z_off = max(max_z_off,
                            float(getattr(sh, "z_offset_mm", 0.0) or 0.0))
    z_travel = (sketch.z_start_mm
                + num_layers * sketch.layer_height_mm
                + max(max_z_off, 0.0)
                + max(sketch.travel_clearance_mm, 0.0))

    waypoints: list[list[float]] = []
    pump_states: list[list[float]] = []
    pump_disp = [0.0, 0.0, 0.0]
    t = 0.0
    last: tuple[float, float, float] | None = None
    prev_print_ink: int | None = None          # abstract ink of the last print
    prev_print_path: np.ndarray | None = None  # world Nx2 of the last printed pass
    total_len = 0.0

    # Retrace-along-bead settings (opt-in; OFF → the block below never fires).
    retrace_on = bool(getattr(sketch, "overlap_travel_enabled", False))
    retrace_max = float(getattr(sketch, "overlap_travel_max_mm", 5.0) or 0.0)
    retrace_pause = bool(getattr(sketch, "overlap_travel_pause_pump", True))
    retrace_speed = (sketch.print_speed_mm_s
                     * max(float(getattr(sketch, "overlap_travel_speed_factor",
                                         1.0) or 1.0), 1e-3))

    # v7.21.5 — per-SEGMENT extrusion, filled in lockstep with ``waypoints`` by
    # ``move_to`` (so it can never drift out of alignment with the toolpath).
    ext_profile: list[float] = []
    vpm_profile: list[float] = []
    total_vol = 0.0

    def move_to(x: float, y: float, z: float, printing: bool, pump: int,
                speed: float | None = None, pump_creep: float = 0.0):
        """Append a waypoint. ``speed`` (when given) overrides the segment speed
        in BOTH the print and travel cases (so a retrace can print/move faster).
        ``pump_creep`` advances the pump by a tiny FRACTION of normal flow on a
        NON-printing segment (a "paused" retrace) — floored just above the
        downstream travel-detection threshold (1e-9) so Quick Print's
        ``_travel_mask`` does NOT split it into a pen-up/pressure-relief, while
        depositing ~nothing (the pump holds pressure, not stops)."""
        nonlocal t, last, total_len, total_vol
        if last is None:
            waypoints.append([x, y, z, pump_disp[0], pump_disp[1],
                              pump_disp[2], t])
            pump_states.append([0.0, 0.0, 0.0])
            last = (x, y, z)
            return
        dist = math.dist(last, (x, y, z))
        # v7.21.5: the modifier in force for THIS segment (``cur_mod`` is set by
        # the shape loop). A travel / hold-pressure segment deposits nothing, so
        # its profile entry is 0.0 — which is also what tells a downstream
        # consumer where the bead stops.
        seg_mod = 0.0
        seg_vpm = 0.0
        if printing and dist > 0:
            seg_mod = cur_mod
            seg_vpm = base_vol_per_mm * cur_mod
            pump_disp[pump] += dist * base_pump_per_mm * cur_mod
            total_len += dist
            total_vol += dist * seg_vpm
            # Colour the segment LEAVING the previous waypoint (preview).
            pump_states[-1][pump] = 1.0
            seg_speed = sketch.print_speed_mm_s if speed is None else speed
        else:
            if pump_creep > 0 and dist > 0:
                # Hold-pressure creep: enough to clear the 1e-9 travel threshold
                # (so it isn't split as a pen-up) but volumetrically negligible.
                pump_disp[pump] += max(
                    dist * base_pump_per_mm * cur_mod * pump_creep, 1e-6)
            seg_speed = sketch.travel_speed_mm_s if speed is None else speed
        t += dist / max(seg_speed, 1e-9)
        waypoints.append([x, y, z, pump_disp[0], pump_disp[1],
                          pump_disp[2], t])
        pump_states.append([0.0, 0.0, 0.0])
        ext_profile.append(seg_mod)
        vpm_profile.append(seg_vpm)
        last = (x, y, z)

    def lift_out(z_target: float):
        """Retract the needle in place to ``z_target``. The first
        ``lift_slow_dist_mm`` is run slowly so the deposited bead doesn't peel
        off with the needle; the rest is at travel speed. ``z_target`` is the
        (higher) travel Z — a no-up target degrades to a plain move."""
        if last is None:
            return
        x0, y0, z0 = last
        if z_target <= z0:
            move_to(x0, y0, z_target, printing=False, pump=0)
            return
        z_slow = min(z_target, z0 + max(sketch.lift_slow_dist_mm, 0.0))
        if z_slow > z0:
            move_to(x0, y0, z_slow, printing=False, pump=0,
                    speed=sketch.lift_slow_speed_mm_s)
        if z_target > z_slow:
            move_to(x0, y0, z_target, printing=False, pump=0)

    for li in range(num_layers):
        z_layer = sketch.z_start_mm + li * sketch.layer_height_mm
        for shape in sketch.shapes:
            # Retract-&-move point: lift the needle out and travel (pen-up) to
            # the marker; it deposits nothing and stays retracted (the next
            # printing shape lowers). This forces a break between runs.
            if shape.kind == "travel":
                tx, ty = float(shape.cx), float(shape.cy)
                if last is not None:
                    lift_out(z_travel)
                move_to(tx, ty, z_travel, printing=False, pump=0)
                prev_print_ink = None              # travel breaks the chain
                continue

            ink = int(shape.ink_id)                # abstract ink (weld gate)
            col = sketch.ink_order_index(ink) % 3  # preview/volume column only
            # This shape's print Z = layer Z + its own height offset (0 for
            # normal shapes; non-zero for a back-trace return pass). Z is a
            # height above the plate bottom, so floor at 0 — a negative
            # back-trace offset must never drive a pass through the plate.
            z = max(0.0, z_layer + float(getattr(shape, "z_offset_mm", 0.0)
                                         or 0.0))
            do_print = not bool(getattr(shape, "no_print", False))
            # v7.21.5: this shape's own extrusion modifier — read by ``move_to``
            # for every segment it emits, so the per-segment profile records the
            # extrusion actually applied and a wider line really is wider.
            cur_mod = _shape_modifier(shape)
            # Closure overlap: continue a closed loop past its seam by the
            # per-shape amount (needle Ø / typed distance; closed unfilled only).
            overlap_mm = (shape.overlap_amount_mm(_od, bead)
                          if (_is_closed_outline(shape) and not shape.filled)
                          else 0.0)
            overlap = overlap_mm > 0.0
            paths = _shape_paths(shape, sketch)
            first_shape_pass = True
            for path in paths:
                if path is None or len(path) < 1:
                    continue
                if li % 2 == 1:
                    path = path[::-1]
                # Weld onto the previous bead when this path connects to it at a
                # shared node: print straight through instead of pen-up/travel.
                # Only when the needle is already down at this layer's print Z;
                # flip the path so the matching endpoint leads if needed. In
                # single-needle mode a channel change cannot weld (ink swap).
                welded = False
                # An ink change blocks a weld only in single-needle mode AND
                # only for a pass that actually deposits — a move-only pass
                # (no_print) extrudes nothing, so it never needs an ink swap.
                ink_ok = ((not single) or (not do_print)
                          or (ink == prev_print_ink))
                if last is not None and abs(last[2] - z) < 1e-6 and ink_ok:
                    d_start = math.hypot(last[0] - float(path[0][0]),
                                         last[1] - float(path[0][1]))
                    d_end = math.hypot(last[0] - float(path[-1][0]),
                                       last[1] - float(path[-1][1]))
                    if d_end <= weld_tol and d_end < d_start:
                        path = path[::-1]
                        d_start = d_end
                    welded = d_start <= weld_tol
                # Over-closure overshoot is appended AFTER any flip so it always
                # continues past the final closure point in the print direction.
                if overlap:
                    path = extend_closed_path(path, overlap_mm)
                sx, sy = float(path[0][0]), float(path[0][1])
                # Retrace along the previous bead instead of lifting: when the
                # optimizer marked this shape (retrace_from) and its start is
                # reachable along the last printed pass within the cap, stay
                # DOWN and walk the bead there (faster and/or pump paused). Only
                # on the FIRST pass of a shape — a shape-to-shape connection
                # (an outline is one pass; a fill can still emit several).
                if (retrace_on and first_shape_pass and not welded
                        and last is not None and prev_print_path is not None
                        and ink_ok and abs(last[2] - z) < 1e-6
                        and getattr(shape, "retrace_from", None) is not None):
                    rpts = _retrace_polyline(prev_print_path,
                                             (last[0], last[1]), (sx, sy),
                                             retrace_max)
                    if rpts:
                        creep = 1e-3 if retrace_pause else 0.0
                        for rx, ry in rpts:
                            move_to(rx, ry, z, printing=(not retrace_pause),
                                    pump=col, speed=retrace_speed,
                                    pump_creep=creep)
                        welded = True              # reached the start; no lift
                # Travel to the start (lift, move over, lower) unless welding.
                if last is not None and not welded:
                    lift_out(z_travel)              # slow first mm, then fast
                    move_to(sx, sy, z_travel, printing=False, pump=col)
                    move_to(sx, sy, z, printing=False, pump=col)
                for px, py in path:
                    move_to(float(px), float(py), z, printing=do_print,
                            pump=col)
                # Remember which ink just PRINTED so the next pass/shape can
                # (in single mode) refuse to weld across an ink change, and
                # keep the last printed pass as the retrace source. A move-only
                # pass (no_print back-trace) deposits nothing → must NOT change
                # the last-printed ink.
                if do_print:
                    prev_print_ink = ink
                    prev_print_path = np.asarray(path, dtype=np.float64)[:, :2]
                first_shape_pass = False

    # Final retract so the needle ends clear of the print (slow first mm).
    if last is not None:
        lift_out(z_travel)

    if len(waypoints) < 2:
        return CompiledSketch(trajectory=np.zeros((0, 7)), pump_states=[],
                              num_layers=num_layers,
                              extrusion_ref_width_mm=ref_width_mm)

    traj = np.asarray(waypoints, dtype=np.float64)
    minx, miny = float(traj[:, 0].min()), float(traj[:, 1].min())
    maxx, maxy = float(traj[:, 0].max()), float(traj[:, 1].max())

    return CompiledSketch(
        trajectory=traj,
        pump_states=pump_states,
        # One entry per segment, by construction (``move_to`` appends exactly
        # one per emitted waypoint after the first).
        extrusion_profile=ext_profile,
        vol_per_mm_profile=vpm_profile,
        extrusion_ref_width_mm=ref_width_mm,
        total_length_mm=total_len,
        # v7.21.5: SUMMED per segment (was total_len x one scalar) — with a
        # per-shape modifier the two are no longer the same number.
        total_volume_uL=total_vol,
        total_time_s=float(traj[-1, 6]),
        num_layers=num_layers,
        num_waypoints=len(traj),
        bounds_mm=(minx, miny, maxx, maxy),
    )


# ═══════════════════════════════════════════════════════════════════
# Print-path optimizer: reorder shapes to minimize pen-up discontinuities
# ═══════════════════════════════════════════════════════════════════

def _weld_tol_for(sketch: Sketch) -> float:
    """The same connection tolerance the compiler welds within (≈ ½ bead)."""
    return max(float(getattr(sketch, "line_spacing_mm", 0.4) or 0.4) * 0.5,
               1e-3)


def _shape_conn(shape: SketchShape, sketch: Sketch):
    """Connectivity descriptor for the optimizer, derived from the shape's
    FIRST generated pass (so it matches what the compiler snaps to).

    Returns ``(kind, points)`` where ``kind`` is:
      - ``"closed"``  → ``points`` is the loop's candidate entry vertices
                        (entry == exit; the seam can sit at any of them).
      - ``"open"``    → ``points`` is ``[first, last]`` (reversible; the nearer
                        endpoint leads).
      - ``"fixed"``   → ``points`` is ``[first, last]`` but NOT reorientable
                        (fills / regions: raster start is fixed in v1).
    or ``None`` when the shape produces no printable path.
    """
    paths = _shape_paths_raw(shape, sketch)
    paths = [p for p in paths if p is not None and len(p) >= 1]
    if not paths:
        return None
    p0 = np.asarray(paths[0], dtype=np.float64)
    if len(p0) < 1:
        return None
    if shape.kind in _START_REORDER_KINDS and not shape.filled:
        if _is_closed_outline(shape):
            core = p0
            if len(p0) > 2 and math.hypot(p0[0, 0] - p0[-1, 0],
                                          p0[0, 1] - p0[-1, 1]) < 1e-9:
                core = p0[:-1]
            return ("closed", core[:, :2])
        return ("open", np.asarray([p0[0, :2], p0[-1, :2]]))
    # Fill / region: entered at its fixed first point, exits at its last.
    return ("fixed", np.asarray([p0[0, :2], p0[-1, :2]]))


def _conn_entry_exit(conn, frm):
    """Given a connectivity descriptor and the previous exit point ``frm``
    (or None for the first shape), return ``(entry, exit, cost, start_point)``.

    ``cost`` is the travel distance from ``frm`` to ``entry``; ``start_point``
    is the value to stamp on the shape so the compiler reproduces this entry
    (None keeps the geometric default)."""
    kind, pts = conn
    if kind == "closed":
        if frm is None:
            entry = pts[0]
            return entry, entry, 0.0, None          # keep default seam
        d = np.hypot(pts[:, 0] - frm[0], pts[:, 1] - frm[1])
        i = int(np.argmin(d))
        entry = pts[i]
        return entry, entry, float(d[i]), (float(entry[0]), float(entry[1]))
    # open / fixed: two endpoints
    first, last = pts[0], pts[1]
    if kind == "fixed" or frm is None:
        # first shape or non-reversible fill → default orientation
        cost = 0.0 if frm is None else math.hypot(first[0] - frm[0],
                                                   first[1] - frm[1])
        return first, last, cost, None
    d_first = math.hypot(first[0] - frm[0], first[1] - frm[1])
    d_last = math.hypot(last[0] - frm[0], last[1] - frm[1])
    if d_last < d_first:                            # reverse: last endpoint leads
        return last, first, d_last, (float(last[0]), float(last[1]))
    return first, last, d_first, (float(first[0]), float(first[1]))


def _greedy_order(conns: list, weld_tol: float, seed: int,
                  inks: list | None = None, single: bool = False):
    """Greedy nearest-neighbour tour over the shape indices, starting at
    ``seed``. Returns ``(order, starts, discontinuities, total_travel)`` where
    ``order`` is the visiting sequence, ``starts[k]`` the start_point to stamp
    on ``order[k]`` (None = default).

    In single-needle mode (``single``) an abstract-ink change cannot weld (it is
    an ink replacement), so the next shape is chosen to PREFER the same ink
    (fewer ink swaps) and an ink change always counts as a discontinuity."""
    n = len(conns)

    def ink_of(k):
        return int(inks[k]) if inks is not None else 0

    remaining = set(range(n))
    order: list[int] = []
    starts: list = []
    disc = 0
    travel = 0.0
    frm = None
    frm_ink = None
    cur = seed
    while True:
        conn = conns[cur]
        entry, exit_pt, cost, sp = _conn_entry_exit(conn, frm)
        this_ink = ink_of(cur)
        if frm is not None:
            travel += cost
            welds = (cost <= weld_tol) and (not single or this_ink == frm_ink)
            if not welds:
                disc += 1
        order.append(cur)
        starts.append(sp)
        remaining.discard(cur)
        frm = (float(exit_pt[0]), float(exit_pt[1]))
        frm_ink = this_ink
        if not remaining:
            break
        # Pick the unplaced shape: in single mode prefer the same ink (to avoid
        # ink swaps), then nearest entry; in multi mode, nearest only.
        best_i, best_key = None, None
        for j in remaining:
            _, _, c, _ = _conn_entry_exit(conns[j], frm)
            switch = 1 if (single and ink_of(j) != frm_ink) else 0
            key = (switch, c)
            if best_key is None or key < best_key:
                best_key, best_i = key, j
        cur = best_i
    return order, starts, disc, travel


def _optimize_segment(shapes: list, sketch: Sketch, weld_tol: float,
                      single: bool = False) -> list:
    """Reorder one segment (a run of non-travel shapes) to minimize pen-up
    discontinuities and stamp each shape's start_point. Returns a new list."""
    if len(shapes) < 2:
        return [copy.deepcopy(s) for s in shapes]
    conns = [_shape_conn(s, sketch) for s in shapes]
    # Shapes with no printable path (e.g. a degenerate primitive) keep their
    # place and are excluded from the tour.
    movable = [i for i, c in enumerate(conns) if c is not None]
    if len(movable) < 2:
        return [copy.deepcopy(s) for s in shapes]

    sub_conns = [conns[i] for i in movable]
    sub_inks = [int(shapes[i].ink_id) for i in movable]
    # Try every start when small; else just the first movable shape.
    seeds = range(len(sub_conns)) if len(sub_conns) <= 14 else [0]
    best = None
    for seed in seeds:
        order, starts, disc, travel = _greedy_order(
            sub_conns, weld_tol, seed, inks=sub_inks, single=single)
        key = (disc, travel)
        if best is None or key < best[0]:
            best = (key, order, starts)
    _, order, starts = best

    # Map the tour (indices into ``movable``) back to the original shapes.
    reordered_movable = [movable[k] for k in order]
    out: list = []
    move_iter = iter(zip(reordered_movable, starts))
    for i in range(len(shapes)):
        if i in movable:
            src_i, sp = next(move_iter)
            sh = copy.deepcopy(shapes[src_i])
            sh.start_point = sp
            out.append(sh)
        else:
            out.append(copy.deepcopy(shapes[i]))
    return out


def optimize_print_order(sketch: Sketch, *, weld_tol: float | None = None,
                         needle=None, single_needle: bool | None = None
                         ) -> Sketch:
    """Return a copy of ``sketch`` whose shapes are reordered (and given
    ``start_point``s) to minimize the number of pen-up discontinuities.

    User-placed ``travel`` (retract-&-move) points are FIXED breaks: the shape
    list is partitioned at them and each segment is optimized independently, so
    the operator's intentional pen-up breaks are preserved. All print
    parameters are carried over unchanged. Filled shapes / ``region`` rasters
    keep their fixed raster orientation (they are reordered but not flipped).

    In single-needle mode (resolved from ``single_needle`` else the sketch/needle
    via :meth:`Sketch.is_single_needle`) the optimizer also groups same-channel
    shapes to minimize ink replacements (a channel change can't weld there)."""
    out = sketch.copy()                        # preserves all print params
    if len(out.shapes) < 2:
        return out
    tol = _weld_tol_for(sketch) if weld_tol is None else float(weld_tol)
    single = (sketch.is_single_needle(needle) if single_needle is None
              else bool(single_needle))

    new_shapes: list = []
    seg: list = []
    for sh in sketch.shapes:
        if sh.kind == "travel":
            if seg:
                new_shapes.extend(_optimize_segment(seg, sketch, tol, single))
                seg = []
            new_shapes.append(copy.deepcopy(sh))   # fixed break, kept in place
        else:
            seg.append(sh)
    if seg:
        new_shapes.extend(_optimize_segment(seg, sketch, tol, single))
    out.shapes = new_shapes
    # Mark connections that can retrace along an existing bead instead of
    # lifting (clears any stale marks; a no-op when the feature is off).
    _stamp_segment_retraces(out.shapes, out, tol)
    return out


def _stamp_segment_retraces(shapes: list, sketch: Sketch,
                            weld_tol: float) -> None:
    """After ordering + start-point stamping, mark each consecutive SAME-INK
    pair that does NOT weld but whose next start is reachable by retracing ≤
    ``overlap_travel_max_mm`` along the previous printed bead — set
    ``retrace_from`` on the following shape so the compiler retraces there
    instead of lifting. Always clears stale marks first; only stamps when the
    feature is enabled. A ``travel`` marker resets the chain (its pen-up is
    intentional)."""
    for sh in shapes:
        sh.retrace_from = None
    if not bool(getattr(sketch, "overlap_travel_enabled", False)):
        return
    max_mm = float(getattr(sketch, "overlap_travel_max_mm", 5.0) or 0.0)
    prev_exit = prev_ink = prev_path = None
    for sh in shapes:
        if sh.kind == "travel":
            prev_exit = prev_ink = prev_path = None
            continue
        paths = [p for p in _shape_paths(sh, sketch)
                 if p is not None and len(p) >= 2]
        if not paths:
            continue
        p0 = np.asarray(paths[0], dtype=np.float64)
        entry = (float(p0[0, 0]), float(p0[0, 1]))
        last_pass = np.asarray(paths[-1], dtype=np.float64)
        exit_pt = (float(last_pass[-1, 0]), float(last_pass[-1, 1]))
        ink = int(sh.ink_id)
        if (prev_exit is not None and prev_path is not None
                and ink == prev_ink
                and math.dist(prev_exit, entry) > weld_tol
                and _retrace_polyline(prev_path, prev_exit, entry, max_mm)):
            sh.retrace_from = (float(prev_exit[0]), float(prev_exit[1]))
        prev_exit, prev_ink, prev_path = exit_pt, ink, last_pass[:, :2]


def count_discontinuities(sketch: Sketch, needle=None, syringe=None) -> int:
    """Number of pen-up travels between printing sub-paths in the compiled
    trajectory — the metric the optimizer minimizes.

    Counted as (number of separate lift-to-travel-Z events − 1): the needle only
    reaches the travel (maximum) Z when it lifts to move between disconnected
    sub-paths, plus once for the final retract. A single fully-welded print
    therefore lifts once (the final retract) → 0 discontinuities. This is the
    same ground truth the sketch tests assert via ``n_lifts`` — and, unlike
    counting pump-flat segments, it is not fooled by the zero-length coincident
    waypoint the compiler emits at an exact weld node.
    """
    try:
        res = compile_to_trajectory(sketch, needle, syringe)
    except Exception:
        return 0
    traj = res.trajectory
    if traj is None or len(traj) < 2:
        return 0
    z = np.asarray(traj, dtype=np.float64)[:, 2]
    zmax = float(z.max())
    zmin = float(z.min())
    if zmax - zmin < 1e-6:                      # never lifts (degenerate)
        return 0
    at_travel = np.isclose(z, zmax, atol=1e-6)
    groups = 0
    prev = False
    for flag in at_travel:
        if flag and not prev:
            groups += 1
        prev = bool(flag)
    return max(0, groups - 1)


# ═══════════════════════════════════════════════════════════════════
# Print sequence plan: connected shapes → color-coded sections + breaks
# ═══════════════════════════════════════════════════════════════════

def _paths_length(paths) -> float:
    total = 0.0
    for p in paths:
        arr = np.asarray(p, dtype=np.float64)
        if arr.ndim == 2 and len(arr) >= 2:
            total += float(np.hypot(np.diff(arr[:, 0]),
                                    np.diff(arr[:, 1])).sum())
    return total


def plan_print_sections(sketch: Sketch, needle=None,
                        single_needle: bool | None = None) -> list[dict]:
    """Break the sketch into the ordered list the sequence panel renders:
    color-coded PRINT sections (maximal runs of shapes printed as one continuous
    bead) and the MOVES that separate them, mirroring the compiler's grouping.

    A section continues while the next shape connects (same print Z, its start
    within ½-bead of the last end) AND — in single-needle mode — uses the same
    abstract ink. Returns a flat list of dicts:
      - ``{"type":"section", "shape_indices":[...], "ink_id":int,
           "pump_index":int, "length_mm":float,
           "break_before": None|"move"|"ink_change"|"layer"}``  where ``ink_id``
        is the abstract ink and ``pump_index`` is the preview column (0..2,
        GUI back-compat only).
      - ``{"type":"move", "reason":"travel", "shape_index":int, "x","y"}``
    ``break_before`` explains the pen-up preceding a section when it is NOT a
    user retract point (``move`` = quick move / reposition, ``ink_change`` =
    single-needle ink change, ``layer`` = different print height).

    Sections group by OBJECT (each drawn shape is one section, one pass since
    v7.21.4). Computed for layer 0; odd layers reverse each pass (serpentine),
    so their inter-shape welds can differ slightly — the panel shows the
    layer-0 sequence."""
    single = (sketch.is_single_needle(needle) if single_needle is None
              else bool(single_needle))
    tol = _weld_tol_for(sketch)
    # Over-closure overshoot (mirror the compiler) so a shape following an
    # overlap-closed loop groups against the loop's real (extended) exit.
    bead = max(float(getattr(sketch, "line_spacing_mm", 0.4) or 0.4), 1e-3)
    _od = 0.0
    if needle is not None:
        try:
            _od = float(needle_orifice_od_mm(needle) or 0.0)
        except (TypeError, ValueError):
            _od = 0.0
    items: list[dict] = []
    cur: dict | None = None
    last_end = None
    last_ink = None
    last_z = None
    had_print = False
    prev_travel = False

    for i, sh in enumerate(sketch.shapes):
        if sh.kind == "travel":
            if cur is not None:
                items.append(cur)
                cur = None
            items.append({"type": "move", "reason": "travel",
                          "shape_index": i,
                          "x": float(sh.cx), "y": float(sh.cy)})
            last_end = last_ink = last_z = None
            prev_travel = True
            continue

        paths = [p for p in _shape_paths(sh, sketch)
                 if p is not None and len(p) >= 1]
        if not paths:
            continue
        p_first = np.asarray(paths[0], dtype=np.float64)
        p_last = np.asarray(paths[-1], dtype=np.float64)
        # The FIRST pass's endpoints drive the weld to the previous shape; the
        # shape's EXIT (what the next shape connects to) is the LAST pass's end
        # — mirroring the compiler's running ``last``. Outlines are one pass
        # (v7.21.4) so first == last there; fills may still emit several.
        fs = (float(p_first[0, 0]), float(p_first[0, 1]))
        fe = (float(p_first[-1, 0]), float(p_first[-1, 1]))
        ink_id = int(sh.ink_id)                    # abstract ink (weld gate)
        col = sketch.ink_order_index(ink_id) % 3   # preview column (GUI compat)
        do_print = not bool(getattr(sh, "no_print", False))
        z = max(0.0, sketch.z_start_mm
                + float(getattr(sh, "z_offset_mm", 0.0) or 0.0))
        length = _paths_length(paths)
        overlap_mm = (sh.overlap_amount_mm(_od, bead)
                      if (_is_closed_outline(sh) and not sh.filled) else 0.0)
        overlap = overlap_mm > 0.0

        # Mirror the compiler's weld decision (incl. the far-end flip on pass 0).
        # An ink change blocks a weld only in single mode AND for a printing
        # pass (a move-only pass extrudes nothing → no ink swap needed).
        ink_ok = (not single) or (not do_print) or (ink_id == last_ink)
        exit_path = p_last
        connects = False
        if (cur is not None and last_z is not None
                and abs(last_z - z) < 1e-6 and ink_ok):
            d_start = math.dist(last_end, fs)
            d_end = math.dist(last_end, fe)
            if d_end <= tol and d_end < d_start:     # compiler flips pass 0
                d_start = d_end
                if len(paths) == 1:                  # single pass → exit flips
                    exit_path = p_last[::-1]
            connects = d_start <= tol
        # Over-closure extends the printed exit past the seam (closed loops).
        if overlap:
            exit_path = extend_closed_path(exit_path, overlap_mm)
        exit_pt = (float(exit_path[-1, 0]), float(exit_path[-1, 1]))

        if connects:
            cur["shape_indices"].append(i)
            cur["length_mm"] += length
        else:
            if cur is not None:
                items.append(cur)
            reason = None
            if not prev_travel and had_print and last_end is not None:
                geo_d = min(math.dist(last_end, fs), math.dist(last_end, fe))
                if last_z is not None and abs(last_z - z) >= 1e-6:
                    reason = "layer"
                elif geo_d > tol:
                    reason = "move"
                    # The optimizer may have planned this pen-up as a retrace
                    # along the existing bead (no lift) instead of a quick move.
                    if (getattr(sh, "retrace_from", None) is not None
                            and bool(getattr(sketch, "overlap_travel_enabled",
                                             False))):
                        reason = "retrace"
                else:                          # connected+same-Z but blocked →
                    reason = "ink_change"      # single-needle ink change
            cur = {"type": "section", "shape_indices": [i],
                   "ink_id": ink_id, "pump_index": col,
                   "length_mm": length, "break_before": reason}

        # Track the last PRINTED ink (a move-only pass doesn't change it),
        # mirroring the compiler's prev_print_ink.
        last_end, last_z = exit_pt, z
        if do_print:
            last_ink = ink_id
        had_print = True
        prev_travel = False

    if cur is not None:
        items.append(cur)
    return items


# ═══════════════════════════════════════════════════════════════════
# Paint-bucket fill of an enclosed region
# ═══════════════════════════════════════════════════════════════════

def _shape_extent_pts(sh: SketchShape):
    if sh.kind == "travel":
        return []                            # markers aren't fill borders
    if sh.kind == "circle":
        return [(sh.cx - sh.radius, sh.cy - sh.radius),
                (sh.cx + sh.radius, sh.cy + sh.radius)]
    if sh.kind == "ellipse":
        return [(sh.cx - sh.rx, sh.cy - sh.ry),
                (sh.cx + sh.rx, sh.cy + sh.ry)]
    if sh.kind == "rect":
        return [(sh.cx - sh.width / 2, sh.cy - sh.height / 2),
                (sh.cx + sh.width / 2, sh.cy + sh.height / 2)]
    return list(sh.points)


def _outline_paths_for_mask(sh: SketchShape, ppm: float) -> list[np.ndarray]:
    """Dense outline polyline(s) of a shape for rasterizing borders."""
    k = sh.kind
    if k == "circle" and sh.radius > 0:
        n = max(24, int(2 * math.pi * sh.radius * ppm))
        return [generate_circle(sh.cx, sh.cy, sh.radius, n)]
    if k == "ellipse" and sh.rx > 0 and sh.ry > 0:
        n = max(24, int(math.pi * (sh.rx + sh.ry) * ppm))
        return [generate_ellipse(sh.cx, sh.cy, sh.rx, sh.ry, n)]
    if k == "rect" and sh.width > 0 and sh.height > 0:
        return [_rect_outline(sh.cx, sh.cy, sh.width, sh.height, ppm)]
    if k == "line" and len(sh.points) >= 2:
        (x1, y1), (x2, y2) = sh.points[0], sh.points[1]
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) * ppm) + 1)
        return [generate_line(x1, y1, x2, y2, n)]
    if k == "polygon" and len(sh.points) >= 2:
        return [_polyline(sh.points, ppm, closed=len(sh.points) >= 3)]
    return []  # regions don't act as borders


def _mark_line(grid, c0, r0, c1, r1, W, H):
    """Bresenham — mark grid pixels along a segment (sealing borders)."""
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc - dr
    while True:
        if 0 <= r0 < H and 0 <= c0 < W:
            grid[r0, c0] = True
        if c0 == c1 and r0 == r1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c0 += sc
        if e2 < dc:
            err += dc
            r0 += sr


def compute_fill_region(shapes, click_xy, spacing_mm,
                        resolution_px_per_mm: float = 6.0):
    """Paint-bucket: meander toolpath (``list[(x,y)]`` mm) filling the region
    enclosed by the outlines of ``shapes`` that contains ``click_xy``.

    Returns ``None`` if scipy is unavailable, the click isn't inside a closed
    region, or the region reaches the canvas bounds (i.e. it's open / the
    outside). Best for simple, convex-ish sections.
    """
    try:
        from scipy import ndimage
    except Exception:
        logger.warning("compute_fill_region: scipy unavailable")
        return None
    borders = [sh for sh in shapes if sh.kind not in ("region", "travel")]
    if not borders:
        return None

    xs, ys = [], []
    for sh in borders:
        for (x, y) in _shape_extent_pts(sh):
            xs.append(x)
            ys.append(y)
    if not xs:
        return None
    xs.append(click_xy[0])
    ys.append(click_xy[1])
    margin = max(spacing_mm * 4, 2.0)
    minx, maxx = min(xs) - margin, max(xs) + margin
    miny, maxy = min(ys) - margin, max(ys) + margin

    res = max(2.0, float(resolution_px_per_mm))
    W = max(8, int((maxx - minx) * res) + 1)
    H = max(8, int((maxy - miny) * res) + 1)
    if W * H > 4_000_000:                       # cap grid size
        res *= (4_000_000 / (W * H)) ** 0.5
        W = max(8, int((maxx - minx) * res) + 1)
        H = max(8, int((maxy - miny) * res) + 1)

    def to_px(x, y):
        return int((x - minx) * res), int((y - miny) * res)  # (col, row)

    border = np.zeros((H, W), dtype=bool)
    sample_ppm = res * 2.0
    for sh in borders:
        for path in _outline_paths_for_mask(sh, sample_ppm):
            if len(path) < 2:
                continue
            for i in range(len(path) - 1):
                c0, r0 = to_px(path[i][0], path[i][1])
                c1, r1 = to_px(path[i + 1][0], path[i + 1][1])
                _mark_line(border, c0, r0, c1, r1, W, H)
    border = ndimage.binary_dilation(border, iterations=1)

    labels, _ = ndimage.label(~border)          # 4-connected free space
    cx, ry = to_px(*click_xy)
    if not (0 <= ry < H and 0 <= cx < W):
        return None
    lab = labels[ry, cx]
    if lab == 0:                                # clicked on a border pixel
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                r2, c2 = ry + dr, cx + dc
                if 0 <= r2 < H and 0 <= c2 < W and labels[r2, c2]:
                    lab = labels[r2, c2]
                    break
            if lab:
                break
        if not lab:
            return None
    region = labels == lab
    if (region[0, :].any() or region[-1, :].any()
            or region[:, 0].any() or region[:, -1].any()):
        return None                             # open region / the outside

    # Serpentine meander over the region.
    step_px = max(1, int(round(spacing_mm * res)))
    pts: list[tuple[float, float]] = []
    flip = False
    for r in range(0, H, step_px):
        idx = np.where(region[r])[0]
        if idx.size == 0:
            continue
        runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        spans = [(minx + run[0] / res, minx + run[-1] / res) for run in runs]
        y = miny + r / res
        if flip:
            spans = [(b, a) for (a, b) in reversed(spans)]
        for xa, xb in spans:
            nseg = max(2, int(abs(xb - xa) * res / step_px) + 1)
            for q in range(nseg):
                t = q / (nseg - 1)
                pts.append((xa + t * (xb - xa), y))
        flip = not flip
    return pts if len(pts) >= 2 else None
