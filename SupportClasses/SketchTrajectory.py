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
# paint-bucket fill (its meander toolpath is stored in ``points``).
SHAPE_KINDS = ("line", "rect", "circle", "ellipse", "polygon", "region")


# ═══════════════════════════════════════════════════════════════════
# Data model
# ═══════════════════════════════════════════════════════════════════

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
    pump_index: int = 0          # 0→P1, 1→P2, 2→P3
    color: str = "#89b4fa"

    def to_dict(self) -> dict:
        return {
            "kind": self.kind, "cx": self.cx, "cy": self.cy,
            "radius": self.radius, "rx": self.rx, "ry": self.ry,
            "width": self.width, "height": self.height,
            "points": [list(p) for p in self.points],
            "filled": self.filled, "line_width_mm": self.line_width_mm,
            "pump_index": self.pump_index, "color": self.color,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SketchShape":
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
            pump_index=int(d.get("pump_index", 0)),
            color=d.get("color", "#89b4fa"),
        )


@dataclass
class Sketch:
    """A full drawing plus the parameters needed to compile a trajectory."""

    shapes: list[SketchShape] = field(default_factory=list)

    # v7.5.x: ``z_start_mm`` is the first-layer print height measured UP from
    # the calibrated plate bottom (mm, ≥ 0), not an absolute Z. The page bakes
    # it into the internal zero-ref frame when the plate bottom is known.
    z_start_mm: float = 0.2
    layer_height_mm: float = 0.2
    num_layers: int = 1

    print_speed_mm_s: float = 5.0
    travel_speed_mm_s: float = 20.0
    travel_clearance_mm: float = 2.0   # lift above the top layer for travel

    line_spacing_mm: float = 0.4       # fill raster pitch (≈ tool diameter)
    outline_points_per_mm: float = 2.0  # outline sampling resolution
    flow_factor: float = 1.0           # fallback pump growth when no syringe

    def to_dict(self) -> dict:
        return {
            "shapes": [s.to_dict() for s in self.shapes],
            "z_start_mm": self.z_start_mm,
            "layer_height_mm": self.layer_height_mm,
            "num_layers": self.num_layers,
            "print_speed_mm_s": self.print_speed_mm_s,
            "travel_speed_mm_s": self.travel_speed_mm_s,
            "travel_clearance_mm": self.travel_clearance_mm,
            "line_spacing_mm": self.line_spacing_mm,
            "outline_points_per_mm": self.outline_points_per_mm,
            "flow_factor": self.flow_factor,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Sketch":
        sk = cls()
        sk.shapes = [SketchShape.from_dict(s) for s in d.get("shapes", [])]
        sk.z_start_mm = float(d.get("z_start_mm", 0.2))
        sk.layer_height_mm = float(d.get("layer_height_mm", 0.2))
        sk.num_layers = int(d.get("num_layers", 1))
        sk.print_speed_mm_s = float(d.get("print_speed_mm_s", 5.0))
        sk.travel_speed_mm_s = float(d.get("travel_speed_mm_s", 20.0))
        sk.travel_clearance_mm = float(d.get("travel_clearance_mm", 2.0))
        sk.line_spacing_mm = float(d.get("line_spacing_mm", 0.4))
        sk.outline_points_per_mm = float(d.get("outline_points_per_mm", 2.0))
        sk.flow_factor = float(d.get("flow_factor", 1.0))
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


def _shape_paths(shape: SketchShape, sketch: Sketch) -> list[np.ndarray]:
    """Build the print path(s) for one shape (single layer).

    Returns a list of Nx2 paths — one per side-by-side pass for thick
    outlines, a single path for fills / regions / thin outlines.
    """
    k = shape.kind
    sp = max(sketch.line_spacing_mm, 1e-3)
    ppm = max(sketch.outline_points_per_mm, 0.1)
    offsets = _pass_offsets(shape.line_width_mm, sp)

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
    bead = max(sketch.line_spacing_mm, 1e-3)
    vol_per_mm = bead * sketch.layer_height_mm
    if syringe is not None:
        pump_per_mm = vol_per_mm * syringe.mm_per_uL
    else:
        pump_per_mm = max(sketch.flow_factor, 1e-6)

    z_travel = (sketch.z_start_mm
                + num_layers * sketch.layer_height_mm
                + max(sketch.travel_clearance_mm, 0.0))

    waypoints: list[list[float]] = []
    pump_states: list[list[float]] = []
    pump_disp = [0.0, 0.0, 0.0]
    t = 0.0
    last: tuple[float, float, float] | None = None
    total_len = 0.0

    def move_to(x: float, y: float, z: float, printing: bool, pump: int):
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
            speed = sketch.print_speed_mm_s
        else:
            speed = sketch.travel_speed_mm_s
        t += dist / max(speed, 1e-9)
        waypoints.append([x, y, z, pump_disp[0], pump_disp[1],
                          pump_disp[2], t])
        pump_states.append([0.0, 0.0, 0.0])
        last = (x, y, z)

    for li in range(num_layers):
        z = sketch.z_start_mm + li * sketch.layer_height_mm
        for shape in sketch.shapes:
            pump = max(0, min(2, int(shape.pump_index)))
            paths = _shape_paths(shape, sketch)
            for path in paths:
                if path is None or len(path) < 1:
                    continue
                if li % 2 == 1:
                    path = path[::-1]
                sx, sy = float(path[0][0]), float(path[0][1])
                # Travel to the start: lift, move over, lower.
                if last is not None:
                    move_to(last[0], last[1], z_travel, printing=False, pump=pump)
                    move_to(sx, sy, z_travel, printing=False, pump=pump)
                    move_to(sx, sy, z, printing=False, pump=pump)
                for px, py in path:
                    move_to(float(px), float(py), z, printing=True, pump=pump)

    # Final retract so the needle ends clear of the print.
    if last is not None:
        move_to(last[0], last[1], z_travel, printing=False, pump=0)

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
# Paint-bucket fill of an enclosed region
# ═══════════════════════════════════════════════════════════════════

def _shape_extent_pts(sh: SketchShape):
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
    borders = [sh for sh in shapes if sh.kind != "region"]
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
