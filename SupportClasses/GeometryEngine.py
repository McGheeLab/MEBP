"""
GeometryEngine.py — Parametric geometry engine for MEBP v7.1.

Generates print objects as time-parameterized trajectories in (x,y,z,p1,p2,p3,t)
space, accounting for needle diameter and ink properties.

Print Object Types:
  2D (single Z): Point, Line, Circle, Square, Triangle, Spiral, Ellipse
  3D (multi-layer): Sphere, Cube, Cylinder, Ellipsoid — each in shell/solid variants

Key calculations:
  - Line spacing = needle_OD × (1 - overlap_fraction)
  - Volume conservation: flow_rate = travel_speed × needle_OD × layer_height
  - Pump rate = flow_rate / syringe.cross_section_area
  - Time parameterization based on target print speed

All coordinates are in mm relative to well center. The trajectory planner
(TrajectoryPlanner.py) later transforms these into absolute stage coordinates.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np

from .PhysicalModels import NeedleSpec, SyringeSpec, InkSpec
from .FlowPhysics import extrusion_flow_rate, flow_rate_to_pump_speed

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Trajectory array column indices
COL_X = 0
COL_Y = 1
COL_Z = 2
COL_P1 = 3
COL_P2 = 4
COL_P3 = 5
COL_T = 6
NUM_COLS = 7


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class ObjectType(Enum):
    """Available print object types."""
    # 2D objects
    POINT = "point"
    LINE = "line"
    CIRCLE = "circle"
    SQUARE = "square"
    TRIANGLE = "triangle"
    SPIRAL = "spiral"
    ELLIPSE = "ellipse"
    # 3D objects
    SPHERE_SHELL = "sphere_shell"
    SPHERE_SOLID = "sphere_solid"
    CUBE_SHELL = "cube_shell"
    CUBE_SOLID = "cube_solid"
    CYLINDER_SHELL = "cylinder_shell"
    CYLINDER_SOLID = "cylinder_solid"
    ELLIPSOID_SHELL = "ellipsoid_shell"
    ELLIPSOID_SOLID = "ellipsoid_solid"
    # Custom
    CSV_IMPORT = "csv_import"


class FillPattern(Enum):
    """Fill pattern for solid 3D objects."""
    MEANDER = "meander"     # Back-and-forth raster
    SPIRAL = "spiral"       # Inside-out spiral fill


# ---------------------------------------------------------------------------
# PrintObject Dataclass
# ---------------------------------------------------------------------------

@dataclass
class PrintObject:
    """
    A parametric print object defined in (x,y,z,p1,p2,p3,t) space.

    The object stores its type-specific parameters and, after generation,
    the complete trajectory as an Nx7 numpy array.
    """
    name: str
    object_type: str                        # ObjectType value string
    params: dict = field(default_factory=dict)  # Type-specific parameters
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)  # (x,y,z) offset in well
    ink_assignments: dict[str, str] = field(default_factory=dict)  # {"P1": "ink_name"}
    color: str = "#a6e3a1"                  # Display color (hex)

    # Generated trajectory — set by generate_object_trajectory()
    trajectory: np.ndarray | None = None    # Nx7: [x, y, z, p1, p2, p3, t]

    # Metadata computed during generation
    total_length_mm: float = 0.0            # Total path length
    total_volume_uL: float = 0.0            # Total ink volume required
    total_time_s: float = 0.0               # Total print time
    num_layers: int = 1                     # Number of Z layers

    @property
    def has_trajectory(self) -> bool:
        return self.trajectory is not None and len(self.trajectory) > 0

    @property
    def num_waypoints(self) -> int:
        return len(self.trajectory) if self.has_trajectory else 0

    @property
    def bounds(self) -> dict[str, tuple[float, float]] | None:
        """Get bounding box {axis: (min, max)} of the trajectory."""
        if not self.has_trajectory:
            return None
        t = self.trajectory
        return {
            "x": (float(t[:, COL_X].min()), float(t[:, COL_X].max())),
            "y": (float(t[:, COL_Y].min()), float(t[:, COL_Y].max())),
            "z": (float(t[:, COL_Z].min()), float(t[:, COL_Z].max())),
        }

    def to_dict(self) -> dict:
        d = {
            "name": self.name,
            "object_type": self.object_type,
            "params": dict(self.params),
            "position": list(self.position),
            "ink_assignments": dict(self.ink_assignments),
            "color": self.color,
            "total_length_mm": self.total_length_mm,
            "total_volume_uL": self.total_volume_uL,
            "total_time_s": self.total_time_s,
            "num_layers": self.num_layers,
        }
        if self.has_trajectory:
            d["trajectory"] = self.trajectory.tolist()
        return d

    @classmethod
    def from_dict(cls, data: dict) -> PrintObject:
        traj_data = data.pop("trajectory", None)
        obj = cls(
            name=data["name"],
            object_type=data["object_type"],
            params=data.get("params", {}),
            position=tuple(data.get("position", (0, 0, 0))),
            ink_assignments=data.get("ink_assignments", {}),
            color=data.get("color", "#a6e3a1"),
            total_length_mm=data.get("total_length_mm", 0.0),
            total_volume_uL=data.get("total_volume_uL", 0.0),
            total_time_s=data.get("total_time_s", 0.0),
            num_layers=data.get("num_layers", 1),
        )
        if traj_data is not None:
            obj.trajectory = np.array(traj_data, dtype=np.float64)
        return obj


# ---------------------------------------------------------------------------
# PrintCollection
# ---------------------------------------------------------------------------

@dataclass
class PrintCollection:
    """
    A set of positioned/colored print objects for one well.

    Represents the complete print job for a single well — multiple objects
    printed in sequence with their individual positions and ink assignments.
    """
    name: str = "Untitled"
    objects: list[PrintObject] = field(default_factory=list)

    def add_object(self, obj: PrintObject) -> None:
        self.objects.append(obj)

    def remove_object(self, index: int) -> PrintObject | None:
        if 0 <= index < len(self.objects):
            return self.objects.pop(index)
        return None

    def move_object(self, from_idx: int, to_idx: int) -> None:
        if 0 <= from_idx < len(self.objects) and 0 <= to_idx < len(self.objects):
            obj = self.objects.pop(from_idx)
            self.objects.insert(to_idx, obj)

    @property
    def total_time_s(self) -> float:
        return sum(o.total_time_s for o in self.objects)

    @property
    def total_volume_uL(self) -> float:
        return sum(o.total_volume_uL for o in self.objects)

    @property
    def total_length_mm(self) -> float:
        return sum(o.total_length_mm for o in self.objects)

    @property
    def num_objects(self) -> int:
        return len(self.objects)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "objects": [o.to_dict() for o in self.objects],
        }

    @classmethod
    def from_dict(cls, data: dict) -> PrintCollection:
        coll = cls(name=data.get("name", "Untitled"))
        for od in data.get("objects", []):
            coll.objects.append(PrintObject.from_dict(od))
        return coll


# ---------------------------------------------------------------------------
# Geometry Helpers
# ---------------------------------------------------------------------------

def line_spacing(needle: NeedleSpec, overlap_fraction: float = 0.0) -> float:
    """
    Calculate line spacing for multi-pass fills.

    spacing = needle_OD × (1 - overlap_fraction)

    Args:
        needle: Needle specification
        overlap_fraction: 0.0 = no overlap, 0.2 = 20% overlap

    Returns:
        Line spacing in mm
    """
    return needle.od_mm * (1.0 - overlap_fraction)


def _path_length(points: np.ndarray) -> float:
    """Calculate total path length from Nx2 or Nx3 array of points."""
    if len(points) < 2:
        return 0.0
    diffs = np.diff(points[:, :min(3, points.shape[1])], axis=0)
    return float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))


def _time_parameterize(
    xy_points: np.ndarray,
    z: float,
    speed_mm_s: float,
    t_start: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Assign timestamps to an Nx2 XY path at constant speed.

    Returns:
        (times, cumulative_distances) both as 1D arrays of length N
    """
    n = len(xy_points)
    if n == 0:
        return np.array([]), np.array([])

    dists = np.zeros(n)
    if n > 1:
        diffs = np.diff(xy_points, axis=0)
        seg_lengths = np.sqrt(np.sum(diffs ** 2, axis=1))
        dists[1:] = np.cumsum(seg_lengths)

    if speed_mm_s <= 0:
        speed_mm_s = 1.0

    times = t_start + dists / speed_mm_s
    return times, dists


def _build_trajectory(
    xy_points: np.ndarray,
    z: float,
    times: np.ndarray,
    pump_column: int,
    pump_positions: np.ndarray,
) -> np.ndarray:
    """
    Build Nx7 trajectory array from XY path + Z + times + pump positions.

    Args:
        xy_points: Nx2 array of (x, y)
        z: Z height for this layer
        times: N-length array of timestamps
        pump_column: Which pump column (COL_P1=3, COL_P2=4, COL_P3=5)
        pump_positions: N-length array of cumulative pump positions (mm)

    Returns:
        Nx7 array [x, y, z, p1, p2, p3, t]
    """
    n = len(xy_points)
    traj = np.zeros((n, NUM_COLS))
    traj[:, COL_X] = xy_points[:, 0]
    traj[:, COL_Y] = xy_points[:, 1]
    traj[:, COL_Z] = z
    traj[:, pump_column] = pump_positions
    traj[:, COL_T] = times
    return traj


# ---------------------------------------------------------------------------
# Extrusion Calculator
# ---------------------------------------------------------------------------

def compute_pump_positions(
    distances: np.ndarray,
    needle: NeedleSpec,
    syringe: SyringeSpec,
    layer_height_mm: float,
) -> np.ndarray:
    """
    Compute cumulative pump plunger positions along a path.

    Volume conservation: deposited volume = needle_OD × layer_height × distance
    Pump travel = volume / syringe.uL_per_mm  (since 1 mm³ = 1 µL)

    Args:
        distances: Cumulative path distances (mm) — N-length array
        needle: Needle specification
        syringe: Syringe specification
        layer_height_mm: Layer height (mm)

    Returns:
        N-length array of cumulative pump positions (mm)
    """
    # Volume per mm of travel = needle_OD × layer_height (µL/mm since mm² ≈ µL/mm)
    volume_per_mm = needle.od_mm * layer_height_mm
    # Total volume along path
    volumes_uL = distances * volume_per_mm
    # Convert to pump plunger travel
    return volumes_uL * syringe.mm_per_uL


def compute_total_volume(
    path_length_mm: float,
    needle: NeedleSpec,
    layer_height_mm: float,
) -> float:
    """
    Compute total ink volume for a path.

    Returns:
        Volume in µL
    """
    return path_length_mm * needle.od_mm * layer_height_mm


# ---------------------------------------------------------------------------
# 2D Generators
# ---------------------------------------------------------------------------

def generate_point(
    cx: float = 0.0,
    cy: float = 0.0,
    dwell_time_s: float = 1.0,
    dispense_volume_uL: float = 0.1,
) -> np.ndarray:
    """
    Generate a single deposition point.

    Returns Nx2 XY path (just one point) plus metadata dict.
    Dwell time is handled via time parameterization.
    """
    return np.array([[cx, cy]]), {"dwell_time_s": dwell_time_s,
                                   "dispense_volume_uL": dispense_volume_uL}


def generate_line(
    x1: float, y1: float,
    x2: float, y2: float,
    num_points: int = 50,
) -> np.ndarray:
    """Generate points along a line segment."""
    xs = np.linspace(x1, x2, num_points)
    ys = np.linspace(y1, y2, num_points)
    return np.column_stack([xs, ys])


def generate_circle(
    cx: float = 0.0,
    cy: float = 0.0,
    radius: float = 1.0,
    num_points: int = 64,
) -> np.ndarray:
    """Generate points along a circle (closed path)."""
    angles = np.linspace(0, 2 * np.pi, num_points + 1)  # +1 to close
    xs = cx + radius * np.cos(angles)
    ys = cy + radius * np.sin(angles)
    return np.column_stack([xs, ys])


def generate_square(
    cx: float = 0.0,
    cy: float = 0.0,
    side: float = 2.0,
    points_per_side: int = 20,
) -> np.ndarray:
    """Generate points along a square perimeter (closed path)."""
    half = side / 2.0
    corners = [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),  # close
    ]
    points = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[i + 1]
        xs = np.linspace(x1, x2, points_per_side, endpoint=(i == 3))
        ys = np.linspace(y1, y2, points_per_side, endpoint=(i == 3))
        points.append(np.column_stack([xs, ys]))
    return np.vstack(points)


def generate_triangle(
    cx: float = 0.0,
    cy: float = 0.0,
    side: float = 2.0,
    points_per_side: int = 20,
) -> np.ndarray:
    """Generate points along an equilateral triangle perimeter (closed)."""
    h = side * math.sqrt(3) / 2
    # Vertices centered at (cx, cy)
    v0 = (cx, cy + 2 * h / 3)
    v1 = (cx - side / 2, cy - h / 3)
    v2 = (cx + side / 2, cy - h / 3)
    vertices = [v0, v1, v2, v0]  # closed

    points = []
    for i in range(3):
        x1, y1 = vertices[i]
        x2, y2 = vertices[i + 1]
        xs = np.linspace(x1, x2, points_per_side, endpoint=(i == 2))
        ys = np.linspace(y1, y2, points_per_side, endpoint=(i == 2))
        points.append(np.column_stack([xs, ys]))
    return np.vstack(points)


def generate_spiral(
    cx: float = 0.0,
    cy: float = 0.0,
    max_radius: float = 2.0,
    spacing_mm: float = 0.5,
    num_points_per_turn: int = 64,
) -> np.ndarray:
    """
    Generate an Archimedes spiral (inside-out).

    pitch = spacing_mm per revolution
    """
    if spacing_mm <= 0:
        spacing_mm = 0.5
    num_turns = max_radius / spacing_mm
    total_points = max(10, int(num_turns * num_points_per_turn))
    angles = np.linspace(0, num_turns * 2 * np.pi, total_points)
    radii = spacing_mm * angles / (2 * np.pi)
    xs = cx + radii * np.cos(angles)
    ys = cy + radii * np.sin(angles)
    return np.column_stack([xs, ys])


def generate_ellipse(
    cx: float = 0.0,
    cy: float = 0.0,
    a: float = 2.0,
    b: float = 1.0,
    num_points: int = 64,
) -> np.ndarray:
    """Generate points along an ellipse (closed path)."""
    angles = np.linspace(0, 2 * np.pi, num_points + 1)
    xs = cx + a * np.cos(angles)
    ys = cy + b * np.sin(angles)
    return np.column_stack([xs, ys])


# ---------------------------------------------------------------------------
# Fill Generators (for solid 3D objects)
# ---------------------------------------------------------------------------

def generate_meander_fill(
    cx: float, cy: float,
    width: float, height: float,
    spacing_mm: float,
    points_per_line: int = 20,
) -> np.ndarray:
    """
    Generate a meander (back-and-forth raster) fill pattern.

    Used for filling solid cross-sections of 3D objects.
    """
    half_w = width / 2.0
    half_h = height / 2.0
    num_lines = max(1, int(height / spacing_mm) + 1)
    y_positions = np.linspace(cy - half_h, cy + half_h, num_lines)

    points = []
    for i, y in enumerate(y_positions):
        if i % 2 == 0:
            xs = np.linspace(cx - half_w, cx + half_w, points_per_line)
        else:
            xs = np.linspace(cx + half_w, cx - half_w, points_per_line)
        ys = np.full(points_per_line, y)
        points.append(np.column_stack([xs, ys]))
    return np.vstack(points) if points else np.empty((0, 2))


def generate_circular_meander_fill(
    cx: float, cy: float,
    radius: float,
    spacing_mm: float,
    points_per_line: int = 20,
) -> np.ndarray:
    """
    Generate a meander fill constrained to a circular region.

    Used for filling circular cross-sections (cylinder solid, sphere solid).
    """
    num_lines = max(1, int(2 * radius / spacing_mm) + 1)
    y_positions = np.linspace(cy - radius, cy + radius, num_lines)

    points = []
    for i, y in enumerate(y_positions):
        dy = y - cy
        if abs(dy) > radius:
            continue
        half_chord = math.sqrt(radius ** 2 - dy ** 2)
        if half_chord < spacing_mm * 0.1:
            continue
        if i % 2 == 0:
            xs = np.linspace(cx - half_chord, cx + half_chord, points_per_line)
        else:
            xs = np.linspace(cx + half_chord, cx - half_chord, points_per_line)
        ys = np.full(points_per_line, y)
        points.append(np.column_stack([xs, ys]))
    return np.vstack(points) if points else np.empty((0, 2))


def generate_elliptical_meander_fill(
    cx: float, cy: float,
    a: float, b: float,
    spacing_mm: float,
    points_per_line: int = 20,
) -> np.ndarray:
    """
    Generate a meander fill constrained to an elliptical region.

    Ellipse equation: ((x-cx)/a)² + ((y-cy)/b)² ≤ 1
    """
    num_lines = max(1, int(2 * b / spacing_mm) + 1)
    y_positions = np.linspace(cy - b, cy + b, num_lines)

    points = []
    for i, y in enumerate(y_positions):
        dy = y - cy
        ratio = 1.0 - (dy / b) ** 2 if b > 0 else 0
        if ratio <= 0:
            continue
        half_chord = a * math.sqrt(ratio)
        if half_chord < spacing_mm * 0.1:
            continue
        if i % 2 == 0:
            xs = np.linspace(cx - half_chord, cx + half_chord, points_per_line)
        else:
            xs = np.linspace(cx + half_chord, cx - half_chord, points_per_line)
        ys = np.full(points_per_line, y)
        points.append(np.column_stack([xs, ys]))
    return np.vstack(points) if points else np.empty((0, 2))


def generate_triangular_meander_fill(
    cx: float, cy: float,
    side: float,
    spacing_mm: float,
    points_per_line: int = 20,
) -> np.ndarray:
    """
    Generate a meander fill constrained to an equilateral triangle region.

    Triangle vertices centered at (cx, cy).
    """
    h = side * math.sqrt(3) / 2
    # Vertices: top, bottom-left, bottom-right (centered at cx, cy)
    y_top = cy + 2 * h / 3
    y_bot = cy - h / 3
    # Scan lines from bottom to top
    num_lines = max(1, int(h / spacing_mm) + 1)
    y_positions = np.linspace(y_bot, y_top, num_lines)

    points = []
    for i, y in enumerate(y_positions):
        # At height y, triangle width narrows linearly from base to apex
        frac = (y - y_bot) / h if h > 0 else 0
        half_width = (side / 2.0) * (1.0 - frac)
        if half_width < spacing_mm * 0.1:
            continue
        if i % 2 == 0:
            xs = np.linspace(cx - half_width, cx + half_width, points_per_line)
        else:
            xs = np.linspace(cx + half_width, cx - half_width, points_per_line)
        ys = np.full(points_per_line, y)
        points.append(np.column_stack([xs, ys]))
    return np.vstack(points) if points else np.empty((0, 2))


def generate_spiral_fill(
    cx: float, cy: float,
    radius: float,
    spacing_mm: float,
    num_points_per_turn: int = 48,
) -> np.ndarray:
    """Generate inside-out spiral fill for a circular region."""
    return generate_spiral(cx, cy, radius, spacing_mm, num_points_per_turn)


# ---------------------------------------------------------------------------
# 3D Layer Slicing
# ---------------------------------------------------------------------------

def _sphere_radius_at_z(z: float, center_z: float, radius: float) -> float:
    """Cross-sectional radius of a sphere at height z."""
    dz = z - center_z
    r_sq = radius ** 2 - dz ** 2
    return math.sqrt(r_sq) if r_sq > 0 else 0.0


def _ellipsoid_radii_at_z(
    z: float, center_z: float, a: float, b: float, c: float,
) -> tuple[float, float]:
    """Cross-sectional semi-axes of an ellipsoid at height z."""
    dz = z - center_z
    ratio = 1.0 - (dz / c) ** 2 if c > 0 else 0
    if ratio <= 0:
        return 0.0, 0.0
    return a * math.sqrt(ratio), b * math.sqrt(ratio)


def compute_layer_heights(
    base_z: float,
    total_height: float,
    layer_height: float,
) -> list[float]:
    """
    Compute Z values for each layer.

    Returns list of Z positions from base upward.
    """
    n_layers = max(1, int(math.ceil(total_height / layer_height)))
    return [base_z + i * layer_height for i in range(n_layers)]


# ---------------------------------------------------------------------------
# Main Generation Dispatch
# ---------------------------------------------------------------------------

def generate_object_trajectory(
    obj: PrintObject,
    needle: NeedleSpec,
    syringe_map: dict[str, SyringeSpec],
    overlap_fraction: float = 0.0,
    print_speed_mm_s: float = 5.0,
    layer_height_mm: float = 0.2,
    fill_pattern: str = "meander",
    pump_id: str = "P1",
) -> np.ndarray:
    """
    Generate the (x,y,z,p1,p2,p3,t) trajectory for a print object.

    This is the main entry point. Dispatches to type-specific generators,
    handles time parameterization and extrusion calculation.

    Args:
        obj: PrintObject with type and params
        needle: Needle specification
        syringe_map: {"P1": SyringeSpec, ...}
        overlap_fraction: Line overlap (0.0 = none, 0.2 = 20%)
        print_speed_mm_s: Target print speed (mm/s)
        layer_height_mm: Layer height for extrusion calc (mm)
        fill_pattern: "meander" or "spiral" for solid objects
        pump_id: Which pump to use ("P1", "P2", "P3")

    Returns:
        Nx7 numpy array [x, y, z, p1, p2, p3, t]
    """
    syringe = syringe_map.get(pump_id)
    if syringe is None:
        logger.warning(f"No syringe for pump {pump_id}, using zero extrusion")

    # Determine pump column index
    pump_col = {"P1": COL_P1, "P2": COL_P2, "P3": COL_P3}.get(pump_id, COL_P1)
    spacing = line_spacing(needle, overlap_fraction)

    otype = obj.object_type
    p = obj.params
    ox, oy, oz = obj.position

    # ----- 2D Objects -----
    if otype == ObjectType.POINT.value:
        traj = _gen_point_trajectory(p, ox, oy, oz, pump_col, syringe)
        obj.num_layers = 1

    elif otype == ObjectType.LINE.value:
        traj = _gen_2d_trajectory(
            generate_line(
                p.get("x1", -1) + ox, p.get("y1", 0) + oy,
                p.get("x2", 1) + ox, p.get("y2", 0) + oy,
                p.get("num_points", 50),
            ),
            oz, print_speed_mm_s, layer_height_mm, needle, syringe, pump_col,
        )
        obj.num_layers = 1

    elif otype == ObjectType.CIRCLE.value:
        if p.get("filled", False):
            pts = generate_circular_meander_fill(
                ox, oy, p.get("radius", 1.0), spacing)
        else:
            pts = generate_circle(ox, oy, p.get("radius", 1.0), p.get("num_points", 64))
        traj = _gen_2d_trajectory(
            pts, oz, print_speed_mm_s, layer_height_mm, needle, syringe, pump_col,
        )
        obj.num_layers = 1

    elif otype == ObjectType.SQUARE.value:
        side = p.get("side", 2.0)
        if p.get("filled", False):
            pts = generate_meander_fill(ox, oy, side, side, spacing)
        else:
            pts = generate_square(ox, oy, side, p.get("points_per_side", 20))
        traj = _gen_2d_trajectory(
            pts, oz, print_speed_mm_s, layer_height_mm, needle, syringe, pump_col,
        )
        obj.num_layers = 1

    elif otype == ObjectType.TRIANGLE.value:
        if p.get("filled", False):
            pts = generate_triangular_meander_fill(
                ox, oy, p.get("side", 2.0), spacing)
        else:
            pts = generate_triangle(ox, oy, p.get("side", 2.0), p.get("points_per_side", 20))
        traj = _gen_2d_trajectory(
            pts, oz, print_speed_mm_s, layer_height_mm, needle, syringe, pump_col,
        )
        obj.num_layers = 1

    elif otype == ObjectType.SPIRAL.value:
        traj = _gen_2d_trajectory(
            generate_spiral(ox, oy, p.get("max_radius", 2.0), spacing,
                           p.get("num_points_per_turn", 64)),
            oz, print_speed_mm_s, layer_height_mm, needle, syringe, pump_col,
        )
        obj.num_layers = 1

    elif otype == ObjectType.ELLIPSE.value:
        if p.get("filled", False):
            pts = generate_elliptical_meander_fill(
                ox, oy, p.get("a", 2.0), p.get("b", 1.0), spacing)
        else:
            pts = generate_ellipse(ox, oy, p.get("a", 2.0), p.get("b", 1.0),
                                  p.get("num_points", 64))
        traj = _gen_2d_trajectory(
            pts, oz, print_speed_mm_s, layer_height_mm, needle, syringe, pump_col,
        )
        obj.num_layers = 1

    # ----- 3D Shell Objects -----
    elif otype == ObjectType.SPHERE_SHELL.value:
        traj = _gen_sphere_shell(p, ox, oy, oz, spacing, print_speed_mm_s,
                                 layer_height_mm, needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    elif otype == ObjectType.CUBE_SHELL.value:
        traj = _gen_cube_shell(p, ox, oy, oz, print_speed_mm_s,
                               layer_height_mm, needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    elif otype == ObjectType.CYLINDER_SHELL.value:
        traj = _gen_cylinder_shell(p, ox, oy, oz, print_speed_mm_s,
                                   layer_height_mm, needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    elif otype == ObjectType.ELLIPSOID_SHELL.value:
        traj = _gen_ellipsoid_shell(p, ox, oy, oz, spacing, print_speed_mm_s,
                                    layer_height_mm, needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    # ----- 3D Solid Objects -----
    elif otype == ObjectType.SPHERE_SOLID.value:
        traj = _gen_sphere_solid(p, ox, oy, oz, spacing, fill_pattern,
                                 print_speed_mm_s, layer_height_mm,
                                 needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    elif otype == ObjectType.CUBE_SOLID.value:
        traj = _gen_cube_solid(p, ox, oy, oz, spacing, fill_pattern,
                               print_speed_mm_s, layer_height_mm,
                               needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    elif otype == ObjectType.CYLINDER_SOLID.value:
        traj = _gen_cylinder_solid(p, ox, oy, oz, spacing, fill_pattern,
                                   print_speed_mm_s, layer_height_mm,
                                   needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    elif otype == ObjectType.ELLIPSOID_SOLID.value:
        traj = _gen_ellipsoid_solid(p, ox, oy, oz, spacing, fill_pattern,
                                    print_speed_mm_s, layer_height_mm,
                                    needle, syringe, pump_col)
        obj.num_layers = _count_layers(traj)

    else:
        logger.warning(f"Unknown object type '{otype}', returning empty trajectory")
        traj = np.empty((0, NUM_COLS))

    # Store trajectory and compute metadata
    obj.trajectory = traj
    if len(traj) > 0:
        obj.total_time_s = float(traj[-1, COL_T] - traj[0, COL_T])
        obj.total_length_mm = _path_length(traj[:, :3])
        obj.total_volume_uL = compute_total_volume(
            obj.total_length_mm, needle, layer_height_mm
        )
    return traj


# ---------------------------------------------------------------------------
# Internal Trajectory Builders
# ---------------------------------------------------------------------------

def _gen_point_trajectory(
    params: dict, ox: float, oy: float, oz: float,
    pump_col: int, syringe: SyringeSpec | None,
) -> np.ndarray:
    """Build trajectory for a point deposit."""
    dwell = params.get("dwell_time_s", 1.0)
    volume = params.get("dispense_volume_uL", 0.1)

    traj = np.zeros((2, NUM_COLS))
    traj[0, COL_X] = ox + params.get("cx", 0.0)
    traj[0, COL_Y] = oy + params.get("cy", 0.0)
    traj[0, COL_Z] = oz
    traj[0, COL_T] = 0.0

    traj[1, :] = traj[0, :]
    traj[1, COL_T] = dwell
    if syringe is not None:
        traj[1, pump_col] = volume * syringe.mm_per_uL
    return traj


def _gen_2d_trajectory(
    xy_points: np.ndarray,
    z: float,
    speed: float,
    layer_h: float,
    needle: NeedleSpec,
    syringe: SyringeSpec | None,
    pump_col: int,
) -> np.ndarray:
    """Build trajectory for any 2D path at a fixed Z."""
    if len(xy_points) == 0:
        return np.empty((0, NUM_COLS))

    times, dists = _time_parameterize(xy_points, z, speed)
    pump_pos = np.zeros(len(xy_points))
    if syringe is not None:
        pump_pos = compute_pump_positions(dists, needle, syringe, layer_h)
    return _build_trajectory(xy_points, z, times, pump_col, pump_pos)


def _gen_multilayer(
    layer_gen_func,
    z_values: list[float],
    speed: float,
    layer_h: float,
    needle: NeedleSpec,
    syringe: SyringeSpec | None,
    pump_col: int,
) -> np.ndarray:
    """
    Build a multi-layer trajectory by stacking single-layer paths.

    layer_gen_func(z) -> Nx2 XY points for that Z layer
    """
    all_layers = []
    t_offset = 0.0
    p_offset = 0.0

    for z in z_values:
        xy = layer_gen_func(z)
        if len(xy) == 0:
            continue

        times, dists = _time_parameterize(xy, z, speed, t_start=t_offset)
        pump_pos = np.zeros(len(xy))
        if syringe is not None:
            pump_pos = compute_pump_positions(dists, needle, syringe, layer_h) + p_offset

        layer_traj = _build_trajectory(xy, z, times, pump_col, pump_pos)
        all_layers.append(layer_traj)

        if len(times) > 0:
            t_offset = float(times[-1]) + 0.1  # Small gap between layers
        if len(pump_pos) > 0:
            p_offset = float(pump_pos[-1])

    if not all_layers:
        return np.empty((0, NUM_COLS))
    return np.vstack(all_layers)


def _count_layers(traj: np.ndarray) -> int:
    """Count unique Z values in a trajectory."""
    if len(traj) == 0:
        return 0
    return len(np.unique(np.round(traj[:, COL_Z], decimals=4)))


# ---------------------------------------------------------------------------
# 3D Shell Generators
# ---------------------------------------------------------------------------

def _gen_sphere_shell(
    p: dict, ox: float, oy: float, oz: float,
    spacing: float, speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Sphere shell: circular perimeters varying by Z."""
    radius = p.get("radius", 1.0)
    lh = p.get("layer_height", layer_h)
    n_pts = p.get("num_points", 64)

    z_values = compute_layer_heights(oz, 2 * radius, lh)
    center_z = oz + radius

    def layer_gen(z):
        r = _sphere_radius_at_z(z, center_z, radius)
        if r < spacing * 0.5:
            return np.empty((0, 2))
        return generate_circle(ox, oy, r, n_pts)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


def _gen_cube_shell(
    p: dict, ox: float, oy: float, oz: float,
    speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Cube shell: square perimeters stacked."""
    side = p.get("side", 2.0)
    height = p.get("height", side)
    lh = p.get("layer_height", layer_h)
    pps = p.get("points_per_side", 20)

    z_values = compute_layer_heights(oz, height, lh)

    def layer_gen(z):
        return generate_square(ox, oy, side, pps)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


def _gen_cylinder_shell(
    p: dict, ox: float, oy: float, oz: float,
    speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Cylinder shell: circular perimeters stacked."""
    radius = p.get("radius", 1.0)
    height = p.get("height", 2.0)
    lh = p.get("layer_height", layer_h)
    n_pts = p.get("num_points", 64)

    z_values = compute_layer_heights(oz, height, lh)

    def layer_gen(z):
        return generate_circle(ox, oy, radius, n_pts)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


def _gen_ellipsoid_shell(
    p: dict, ox: float, oy: float, oz: float,
    spacing: float, speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Ellipsoid shell: elliptical perimeters varying by Z."""
    a = p.get("a", 2.0)
    b = p.get("b", 1.5)
    c = p.get("c", 1.0)
    lh = p.get("layer_height", layer_h)
    n_pts = p.get("num_points", 64)

    z_values = compute_layer_heights(oz, 2 * c, lh)
    center_z = oz + c

    def layer_gen(z):
        ra, rb = _ellipsoid_radii_at_z(z, center_z, a, b, c)
        if ra < spacing * 0.5 or rb < spacing * 0.5:
            return np.empty((0, 2))
        return generate_ellipse(ox, oy, ra, rb, n_pts)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


# ---------------------------------------------------------------------------
# 3D Solid Generators
# ---------------------------------------------------------------------------

def _gen_sphere_solid(
    p: dict, ox: float, oy: float, oz: float,
    spacing: float, fill: str, speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Sphere solid: filled circles per layer."""
    radius = p.get("radius", 1.0)
    lh = p.get("layer_height", layer_h)

    z_values = compute_layer_heights(oz, 2 * radius, lh)
    center_z = oz + radius

    def layer_gen(z):
        r = _sphere_radius_at_z(z, center_z, radius)
        if r < spacing * 0.5:
            return np.empty((0, 2))
        if fill == "spiral":
            return generate_spiral_fill(ox, oy, r, spacing)
        return generate_circular_meander_fill(ox, oy, r, spacing)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


def _gen_cube_solid(
    p: dict, ox: float, oy: float, oz: float,
    spacing: float, fill: str, speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Cube solid: filled squares per layer."""
    side = p.get("side", 2.0)
    height = p.get("height", side)
    lh = p.get("layer_height", layer_h)

    z_values = compute_layer_heights(oz, height, lh)

    def layer_gen(z):
        return generate_meander_fill(ox, oy, side, side, spacing)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


def _gen_cylinder_solid(
    p: dict, ox: float, oy: float, oz: float,
    spacing: float, fill: str, speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Cylinder solid: filled circles stacked."""
    radius = p.get("radius", 1.0)
    height = p.get("height", 2.0)
    lh = p.get("layer_height", layer_h)

    z_values = compute_layer_heights(oz, height, lh)

    def layer_gen(z):
        if fill == "spiral":
            return generate_spiral_fill(ox, oy, radius, spacing)
        return generate_circular_meander_fill(ox, oy, radius, spacing)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


def _gen_ellipsoid_solid(
    p: dict, ox: float, oy: float, oz: float,
    spacing: float, fill: str, speed: float, layer_h: float,
    needle: NeedleSpec, syringe: SyringeSpec | None, pump_col: int,
) -> np.ndarray:
    """Ellipsoid solid: filled elliptical cross-sections per layer."""
    a = p.get("a", 2.0)
    b = p.get("b", 1.5)
    c = p.get("c", 1.0)
    lh = p.get("layer_height", layer_h)

    z_values = compute_layer_heights(oz, 2 * c, lh)
    center_z = oz + c

    def layer_gen(z):
        ra, rb = _ellipsoid_radii_at_z(z, center_z, a, b, c)
        if ra < spacing * 0.5 or rb < spacing * 0.5:
            return np.empty((0, 2))
        return generate_elliptical_meander_fill(ox, oy, ra, rb, spacing)

    return _gen_multilayer(layer_gen, z_values, speed, lh, needle, syringe, pump_col)


# ---------------------------------------------------------------------------
# Utility: Available object types for GUI
# ---------------------------------------------------------------------------

OBJECT_TYPE_INFO: dict[str, dict] = {
    ObjectType.POINT.value: {
        "label": "Point",
        "category": "2D",
        "params": {"cx": 0.0, "cy": 0.0, "dwell_time_s": 1.0, "dispense_volume_uL": 0.1},
        "description": "Single deposition point with dwell time",
    },
    ObjectType.LINE.value: {
        "label": "Line",
        "category": "2D",
        "params": {"x1": -1.0, "y1": 0.0, "x2": 1.0, "y2": 0.0, "num_points": 50},
        "description": "Straight line segment",
    },
    ObjectType.CIRCLE.value: {
        "label": "Circle",
        "category": "2D",
        "params": {"radius": 1.0, "num_points": 64},
        "description": "Circular perimeter",
    },
    ObjectType.SQUARE.value: {
        "label": "Square",
        "category": "2D",
        "params": {"side": 2.0, "points_per_side": 20},
        "description": "Square perimeter",
    },
    ObjectType.TRIANGLE.value: {
        "label": "Triangle",
        "category": "2D",
        "params": {"side": 2.0, "points_per_side": 20},
        "description": "Equilateral triangle perimeter",
    },
    ObjectType.SPIRAL.value: {
        "label": "Spiral",
        "category": "2D",
        "params": {"max_radius": 2.0, "num_points_per_turn": 64},
        "description": "Archimedes spiral (spacing auto from needle OD)",
    },
    ObjectType.ELLIPSE.value: {
        "label": "Ellipse",
        "category": "2D",
        "params": {"a": 2.0, "b": 1.0, "num_points": 64},
        "description": "Elliptical perimeter",
    },
    ObjectType.SPHERE_SHELL.value: {
        "label": "Sphere (shell)",
        "category": "3D",
        "params": {"radius": 1.0, "layer_height": 0.2, "num_points": 64},
        "description": "Hollow sphere — circular perimeters varying by Z",
    },
    ObjectType.SPHERE_SOLID.value: {
        "label": "Sphere (solid)",
        "category": "3D",
        "params": {"radius": 1.0, "layer_height": 0.2},
        "description": "Filled sphere — meander/spiral fill per layer",
    },
    ObjectType.CUBE_SHELL.value: {
        "label": "Cube (shell)",
        "category": "3D",
        "params": {"side": 2.0, "height": 2.0, "layer_height": 0.2, "points_per_side": 20},
        "description": "Hollow cube — square perimeters stacked",
    },
    ObjectType.CUBE_SOLID.value: {
        "label": "Cube (solid)",
        "category": "3D",
        "params": {"side": 2.0, "height": 2.0, "layer_height": 0.2},
        "description": "Filled cube — meander fill per layer",
    },
    ObjectType.CYLINDER_SHELL.value: {
        "label": "Cylinder (shell)",
        "category": "3D",
        "params": {"radius": 1.0, "height": 2.0, "layer_height": 0.2, "num_points": 64},
        "description": "Hollow cylinder — circular perimeters stacked",
    },
    ObjectType.CYLINDER_SOLID.value: {
        "label": "Cylinder (solid)",
        "category": "3D",
        "params": {"radius": 1.0, "height": 2.0, "layer_height": 0.2},
        "description": "Filled cylinder — meander/spiral fill stacked",
    },
    ObjectType.ELLIPSOID_SHELL.value: {
        "label": "Ellipsoid (shell)",
        "category": "3D",
        "params": {"a": 2.0, "b": 1.5, "c": 1.0, "layer_height": 0.2, "num_points": 64},
        "description": "Hollow ellipsoid — elliptical perimeters varying by Z",
    },
    ObjectType.ELLIPSOID_SOLID.value: {
        "label": "Ellipsoid (solid)",
        "category": "3D",
        "params": {"a": 2.0, "b": 1.5, "c": 1.0, "layer_height": 0.2},
        "description": "Filled ellipsoid — elliptical meander fill per layer",
    },
}


def get_available_object_types() -> dict[str, dict]:
    """Return the full object type catalog for GUI dropdowns."""
    return dict(OBJECT_TYPE_INFO)


def get_default_params(object_type: str) -> dict:
    """Get default parameters for an object type."""
    info = OBJECT_TYPE_INFO.get(object_type, {})
    return dict(info.get("params", {}))
