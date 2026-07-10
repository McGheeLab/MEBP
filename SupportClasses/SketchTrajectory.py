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
    line_width_mm: float = 0.4   # printed bead width — render + multi-pass + volume
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
    # shape at the nearest endpoint (choosing which end leads) — so a shape can
    # be made to start exactly where the previous one ended and weld into one
    # continuous bead. Set manually (drag the start marker) or by the path
    # optimizer. Does NOT change the deposited geometry, only the entry/exit.
    start_point: tuple[float, float] | None = None

    # v7.5.x: (closed-loop outlines only) when True the compiler continues the
    # printed path PAST the closure point by ~the needle OUTER radius, so the
    # seam over-closes. As the needle re-enters the start it pushes deposited
    # ink aside; without the overshoot the loop doesn't fully close. No effect
    # on open paths (nothing to close) or filled shapes (raster).
    overlap_closure: bool = False

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
        if self.overlap_closure:               # only when on → legacy-identical
            d["overlap_closure"] = True
        if self.retrace_from is not None:      # only when the optimizer set it
            d["retrace_from"] = [float(self.retrace_from[0]),
                                 float(self.retrace_from[1])]
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "SketchShape":
        sp = d.get("start_point")
        start_point = (float(sp[0]), float(sp[1])) if sp else None
        rf = d.get("retrace_from")
        retrace_from = (float(rf[0]), float(rf[1])) if rf else None
        ink_id = d.get("ink_id")
        if ink_id is None:                      # legacy: pump_index → ink_id
            ink_id = int(d.get("pump_index", 0)) + 1
        else:
            ink_id = int(ink_id)
        return cls(
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
            overlap_closure=bool(d.get("overlap_closure", False)),
            retrace_from=retrace_from,
        )


@dataclass
class Sketch:
    """A full drawing plus the parameters needed to compile a trajectory."""

    shapes: list[SketchShape] = field(default_factory=list)

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
        sk.overlap_travel_enabled = bool(d.get("overlap_travel_enabled", False))
        sk.overlap_travel_max_mm = float(d.get("overlap_travel_max_mm", 5.0))
        sk.overlap_travel_pause_pump = bool(
            d.get("overlap_travel_pause_pump", True))
        sk.overlap_travel_speed_factor = float(
            d.get("overlap_travel_speed_factor", 1.0))
        return sk

    def copy(self) -> "Sketch":
        return copy.deepcopy(self)


@dataclass
class CompiledSketch:
    """Result of :func:`compile_to_trajectory`."""

    trajectory: np.ndarray                    # Nx7 [x,y,z,p1,p2,p3,t]
    pump_states: list[list[float]]            # per-waypoint [f1,f2,f3] (preview)
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
    """Centered perpendicular offsets for a multi-pass thick outline.

    A bead of width ``bead`` is laid per pass; ``n`` passes reach
    ``line_width``. Returns offsets symmetric about 0 (e.g. n=3, bead=0.4 →
    [-0.4, 0, 0.4]).
    """
    n = max(1, int(round(line_width / bead))) if line_width > bead else 1
    if n == 1:
        return [0.0]
    return [(k - (n - 1) / 2.0) * bead for k in range(n)]


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
    sp = getattr(shape, "start_point", None)
    if (sp is not None and shape.kind in _START_REORDER_KINDS
            and not shape.filled):
        closed = _is_closed_outline(shape)
        paths = [reorder_path_to_start(p, sp, closed)
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
    # Thick outlines emit several adjacent passes, which sums to the bead
    # width the user asked for — keeping volume coherent across fills and
    # multi-pass outlines.
    bead = max(sketch.line_spacing_mm, 1e-3)      # fill raster pitch (geometry)
    # v7.5.x (Finding A) — UNIFY on the canonical F-1 bead model: deposited
    # volume per mm of travel = the needle's INNER-BORE cross-section × the
    # extrusion multiplier, exactly like FlowPhysics / GeometryEngine / Quick
    # Print. So a sketch-baked print and a Quick Print of the SAME needle+path
    # now extrude identically, and the baked volume no longer depends on
    # layer_height (which is only the Z step between stacked layers). The
    # multiplier scales HOW MUCH is deposited, never the toolpath geometry.
    # NOTE: the Sketch page's shaded thickness band is a display WIDTH
    # (inner Ø × mult); the baked VOLUME here is bore AREA × mult — width vs
    # area differ by design (mirrors Quick Print, which shows no band). When no
    # needle is supplied (bore unknown) fall back to bead × layer_height × mult
    # so the no-syringe preview stays monotonic and previewable.
    mult = max(float(getattr(sketch, "extrusion_multiplier", 1.0)), 0.0)
    vol_per_mm = bead * sketch.layer_height_mm * mult
    if needle is not None:
        try:
            _area = float(getattr(needle, "cross_section_area_mm2", 0.0) or 0.0)
            if _area > 0:
                vol_per_mm = _area * mult
        except (TypeError, ValueError):
            pass
    # Two paths "connect" — and print as one continuous bead with no pen-up —
    # when an endpoint of the next sits within half a bead of where the last
    # left off (their deposited beads already overlap). Half a bead is tight
    # enough never to fuse the adjacent parallel passes of a thick outline,
    # which are a full bead apart.
    weld_tol = max(bead * 0.5, 1e-3)
    if syringe is not None:
        pump_per_mm = vol_per_mm * syringe.mm_per_uL
    else:
        pump_per_mm = max(sketch.flow_factor, 1e-6) * mult

    # Single-needle mode (one material at the tip at a time): a pump/channel
    # change between connected shapes cannot weld — it needs an ink replacement
    # (pen-up). In multi mode the compiler welds across channels (legacy).
    single = sketch.is_single_needle(needle)
    # Over-closure overshoot for closed loops with overlap_closure on = the
    # needle OUTER radius (it pushes ink aside on re-entry); fall back to ½ bead
    # when the needle Ø is unknown.
    _od = 0.0
    if needle is not None:
        try:
            _od = float(getattr(needle, "od_mm", 0.0) or 0.0)
        except (TypeError, ValueError):
            _od = 0.0
    overlap_overshoot = (_od / 2.0) if _od > 0 else (bead / 2.0)

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

    def move_to(x: float, y: float, z: float, printing: bool, pump: int,
                speed: float | None = None, pump_creep: float = 0.0):
        """Append a waypoint. ``speed`` (when given) overrides the segment speed
        in BOTH the print and travel cases (so a retrace can print/move faster).
        ``pump_creep`` advances the pump by a tiny FRACTION of normal flow on a
        NON-printing segment (a "paused" retrace) — floored just above the
        downstream travel-detection threshold (1e-9) so Quick Print's
        ``_travel_mask`` does NOT split it into a pen-up/pressure-relief, while
        depositing ~nothing (the pump holds pressure, not stops)."""
        nonlocal t, last, total_len
        if last is None:
            waypoints.append([x, y, z, pump_disp[0], pump_disp[1],
                              pump_disp[2], t])
            pump_states.append([0.0, 0.0, 0.0])
            last = (x, y, z)
            return
        dist = math.dist(last, (x, y, z))
        if printing and dist > 0:
            pump_disp[pump] += dist * pump_per_mm
            total_len += dist
            # Colour the segment LEAVING the previous waypoint (preview).
            pump_states[-1][pump] = 1.0
            seg_speed = sketch.print_speed_mm_s if speed is None else speed
        else:
            if pump_creep > 0 and dist > 0:
                # Hold-pressure creep: enough to clear the 1e-9 travel threshold
                # (so it isn't split as a pen-up) but volumetrically negligible.
                pump_disp[pump] += max(dist * pump_per_mm * pump_creep, 1e-6)
            seg_speed = sketch.travel_speed_mm_s if speed is None else speed
        t += dist / max(seg_speed, 1e-9)
        waypoints.append([x, y, z, pump_disp[0], pump_disp[1],
                          pump_disp[2], t])
        pump_states.append([0.0, 0.0, 0.0])
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
            # Over-closure: continue a closed loop past its seam by the needle
            # radius (opt-in, closed outlines only).
            overlap = (bool(getattr(shape, "overlap_closure", False))
                       and _is_closed_outline(shape) and not shape.filled)
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
                    path = extend_closed_path(path, overlap_overshoot)
                sx, sy = float(path[0][0]), float(path[0][1])
                # Retrace along the previous bead instead of lifting: when the
                # optimizer marked this shape (retrace_from) and its start is
                # reachable along the last printed pass within the cap, stay
                # DOWN and walk the bead there (faster and/or pump paused). Only
                # on the FIRST pass of a shape — a shape-to-shape connection, not
                # between a thick outline's own concentric passes.
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
                              num_layers=num_layers)

    traj = np.asarray(waypoints, dtype=np.float64)
    minx, miny = float(traj[:, 0].min()), float(traj[:, 1].min())
    maxx, maxy = float(traj[:, 0].max()), float(traj[:, 1].max())

    return CompiledSketch(
        trajectory=traj,
        pump_states=pump_states,
        total_length_mm=total_len,
        total_volume_uL=total_len * vol_per_mm,
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

    Sections group by OBJECT (each drawn shape is one section): a thick multi-
    pass outline lifts internally between its concentric passes, but that is
    intra-object and NOT shown as a break here. Computed for layer 0; odd layers
    reverse each pass (serpentine), so their inter-shape welds can differ
    slightly — the panel shows the layer-0 sequence."""
    single = (sketch.is_single_needle(needle) if single_needle is None
              else bool(single_needle))
    tol = _weld_tol_for(sketch)
    # Over-closure overshoot (mirror the compiler) so a shape following an
    # overlap-closed loop groups against the loop's real (extended) exit.
    bead = max(float(getattr(sketch, "line_spacing_mm", 0.4) or 0.4), 1e-3)
    _od = 0.0
    if needle is not None:
        try:
            _od = float(getattr(needle, "od_mm", 0.0) or 0.0)
        except (TypeError, ValueError):
            _od = 0.0
    overshoot = (_od / 2.0) if _od > 0 else (bead / 2.0)
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
        # — mirroring the compiler's running ``last`` for a multi-pass outline.
        fs = (float(p_first[0, 0]), float(p_first[0, 1]))
        fe = (float(p_first[-1, 0]), float(p_first[-1, 1]))
        ink_id = int(sh.ink_id)                    # abstract ink (weld gate)
        col = sketch.ink_order_index(ink_id) % 3   # preview column (GUI compat)
        do_print = not bool(getattr(sh, "no_print", False))
        z = max(0.0, sketch.z_start_mm
                + float(getattr(sh, "z_offset_mm", 0.0) or 0.0))
        length = _paths_length(paths)
        overlap = (bool(getattr(sh, "overlap_closure", False))
                   and _is_closed_outline(sh) and not sh.filled)

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
            exit_path = extend_closed_path(exit_path, overshoot)
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
