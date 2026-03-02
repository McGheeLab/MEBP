"""
auto_layout.py — Auto-layout generators for MEBP v7.2.3 Print Objects.

Generates (x, y) position lists for arranging N objects within a well
in geometric patterns. Used by the Print Objects tab to quickly populate
a print file with regularly-spaced objects.

All coordinates are in mm, relative to well center (0, 0).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class LayoutPattern(Enum):
    """Available auto-layout patterns."""
    RING = "ring"
    SQUARE_GRID = "square_grid"
    HEX_GRID = "hex_grid"
    LINE = "line"
    CONCENTRIC_RINGS = "concentric_rings"


# Pattern display info for UI
LAYOUT_INFO = {
    LayoutPattern.RING: {
        "label": "Ring",
        "icon": "◯",
        "description": "Objects evenly spaced on a circle",
        "params": {
            "count": {"label": "Count", "type": "int", "min": 1, "max": 100, "default": 6},
            "radius_mm": {"label": "Radius", "type": "float", "min": 0.1, "max": 20.0,
                          "default": 2.0, "suffix": " mm"},
            "start_angle_deg": {"label": "Start Angle", "type": "float", "min": 0.0,
                                "max": 359.0, "default": 0.0, "suffix": "°"},
        },
    },
    LayoutPattern.SQUARE_GRID: {
        "label": "Square Grid",
        "icon": "▦",
        "description": "Rectangular grid of objects",
        "params": {
            "rows": {"label": "Rows", "type": "int", "min": 1, "max": 20, "default": 3},
            "cols": {"label": "Columns", "type": "int", "min": 1, "max": 20, "default": 3},
            "spacing_mm": {"label": "Spacing", "type": "float", "min": 0.1, "max": 10.0,
                           "default": 1.0, "suffix": " mm"},
        },
    },
    LayoutPattern.HEX_GRID: {
        "label": "Hex Grid",
        "icon": "⬡",
        "description": "Hexagonal close-packed grid",
        "params": {
            "rows": {"label": "Rows", "type": "int", "min": 1, "max": 20, "default": 3},
            "cols": {"label": "Columns", "type": "int", "min": 1, "max": 20, "default": 3},
            "spacing_mm": {"label": "Spacing", "type": "float", "min": 0.1, "max": 10.0,
                           "default": 1.0, "suffix": " mm"},
        },
    },
    LayoutPattern.LINE: {
        "label": "Line",
        "icon": "╱",
        "description": "Objects in a straight line",
        "params": {
            "count": {"label": "Count", "type": "int", "min": 2, "max": 50, "default": 5},
            "length_mm": {"label": "Length", "type": "float", "min": 0.5, "max": 20.0,
                          "default": 4.0, "suffix": " mm"},
            "angle_deg": {"label": "Angle", "type": "float", "min": 0.0, "max": 359.0,
                          "default": 0.0, "suffix": "°"},
        },
    },
    LayoutPattern.CONCENTRIC_RINGS: {
        "label": "Concentric Rings",
        "icon": "◎",
        "description": "Multiple rings at different radii",
        "params": {
            "num_rings": {"label": "Rings", "type": "int", "min": 1, "max": 10, "default": 3},
            "objects_per_ring": {"label": "Per Ring", "type": "int", "min": 2, "max": 30,
                                 "default": 6},
            "inner_radius_mm": {"label": "Inner R", "type": "float", "min": 0.3, "max": 10.0,
                                "default": 1.0, "suffix": " mm"},
            "outer_radius_mm": {"label": "Outer R", "type": "float", "min": 0.5, "max": 15.0,
                                "default": 3.0, "suffix": " mm"},
        },
    },
}


# ═══════════════════════════════════════════════════════════════════
#  Layout Generators
# ═══════════════════════════════════════════════════════════════════

def auto_layout_ring(
    count: int,
    radius_mm: float,
    start_angle_deg: float = 0.0,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate (x, y) positions for objects evenly spaced on a circle.

    Args:
        count: Number of objects
        radius_mm: Circle radius in mm
        start_angle_deg: Angle of first object (0 = right, 90 = top)
        center: (x, y) center of the ring in mm

    Returns:
        List of (x, y) tuples in mm
    """
    if count <= 0:
        return []
    if count == 1:
        angle = math.radians(start_angle_deg)
        return [(center[0] + radius_mm * math.cos(angle),
                 center[1] + radius_mm * math.sin(angle))]

    positions = []
    for i in range(count):
        angle = math.radians(start_angle_deg + i * 360.0 / count)
        x = center[0] + radius_mm * math.cos(angle)
        y = center[1] + radius_mm * math.sin(angle)
        positions.append((round(x, 4), round(y, 4)))
    return positions


def auto_layout_grid(
    rows: int,
    cols: int,
    spacing_mm: float,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate (x, y) positions for a rectangular grid, centered on center.

    Args:
        rows: Number of rows
        cols: Number of columns
        spacing_mm: Distance between adjacent objects in mm
        center: (x, y) center of the grid

    Returns:
        List of (x, y) tuples in mm, row-major order
    """
    positions = []
    x_start = center[0] - (cols - 1) * spacing_mm / 2
    y_start = center[1] - (rows - 1) * spacing_mm / 2
    for r in range(rows):
        for c in range(cols):
            x = x_start + c * spacing_mm
            y = y_start + r * spacing_mm
            positions.append((round(x, 4), round(y, 4)))
    return positions


def auto_layout_hex(
    rows: int,
    cols: int,
    spacing_mm: float,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate (x, y) positions for a hexagonal close-packed grid.

    Odd rows are offset by half a spacing to create hex packing.

    Args:
        rows: Number of rows
        cols: Number of columns
        spacing_mm: Center-to-center distance between adjacent objects
        center: (x, y) center of the grid

    Returns:
        List of (x, y) tuples in mm
    """
    positions = []
    row_height = spacing_mm * math.sqrt(3) / 2
    x_start = center[0] - (cols - 1) * spacing_mm / 2
    y_start = center[1] - (rows - 1) * row_height / 2

    for r in range(rows):
        x_offset = spacing_mm / 2 if r % 2 == 1 else 0
        for c in range(cols):
            x = x_start + c * spacing_mm + x_offset
            y = y_start + r * row_height
            positions.append((round(x, 4), round(y, 4)))
    return positions


def auto_layout_line(
    count: int,
    length_mm: float,
    angle_deg: float = 0.0,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate (x, y) positions along a straight line, centered on center.

    Args:
        count: Number of objects (minimum 2)
        length_mm: Total length of the line
        angle_deg: Angle of the line (0 = horizontal right)
        center: (x, y) center of the line

    Returns:
        List of (x, y) tuples in mm
    """
    if count <= 0:
        return []
    if count == 1:
        return [center]

    angle = math.radians(angle_deg)
    dx = math.cos(angle) * length_mm / 2
    dy = math.sin(angle) * length_mm / 2

    x_start = center[0] - dx
    y_start = center[1] - dy

    positions = []
    for i in range(count):
        t = i / (count - 1)  # 0.0 to 1.0
        x = x_start + t * 2 * dx
        y = y_start + t * 2 * dy
        positions.append((round(x, 4), round(y, 4)))
    return positions


def auto_layout_concentric_rings(
    num_rings: int,
    objects_per_ring: int,
    inner_radius_mm: float,
    outer_radius_mm: float,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate (x, y) positions on multiple concentric rings.

    Each ring has the same number of objects, evenly spaced.
    Rings are evenly spaced between inner and outer radius.
    Adjacent rings are offset by half a step for hex-like packing.

    Args:
        num_rings: Number of concentric rings
        objects_per_ring: Number of objects per ring
        inner_radius_mm: Radius of innermost ring
        outer_radius_mm: Radius of outermost ring
        center: (x, y) center

    Returns:
        List of (x, y) tuples in mm, from inner to outer ring
    """
    if num_rings <= 0 or objects_per_ring <= 0:
        return []

    positions = []
    if num_rings == 1:
        radii = [inner_radius_mm]
    else:
        radii = [
            inner_radius_mm + i * (outer_radius_mm - inner_radius_mm) / (num_rings - 1)
            for i in range(num_rings)
        ]

    for ring_idx, radius in enumerate(radii):
        # Offset alternate rings for better packing
        angle_offset = (180.0 / objects_per_ring) if ring_idx % 2 == 1 else 0.0
        ring_positions = auto_layout_ring(
            count=objects_per_ring,
            radius_mm=radius,
            start_angle_deg=angle_offset,
            center=center,
        )
        positions.extend(ring_positions)

    return positions


# ═══════════════════════════════════════════════════════════════════
#  Dispatch
# ═══════════════════════════════════════════════════════════════════

def generate_layout(
    pattern: LayoutPattern | str,
    params: dict[str, Any],
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate object positions for a given layout pattern.

    Args:
        pattern: LayoutPattern enum or string value
        params: Pattern-specific parameters (from LAYOUT_INFO)
        center: (x, y) center of the layout

    Returns:
        List of (x, y) positions in mm
    """
    if isinstance(pattern, str):
        pattern = LayoutPattern(pattern)

    if pattern == LayoutPattern.RING:
        return auto_layout_ring(
            count=params.get("count", 6),
            radius_mm=params.get("radius_mm", 2.0),
            start_angle_deg=params.get("start_angle_deg", 0.0),
            center=center,
        )
    elif pattern == LayoutPattern.SQUARE_GRID:
        return auto_layout_grid(
            rows=params.get("rows", 3),
            cols=params.get("cols", 3),
            spacing_mm=params.get("spacing_mm", 1.0),
            center=center,
        )
    elif pattern == LayoutPattern.HEX_GRID:
        return auto_layout_hex(
            rows=params.get("rows", 3),
            cols=params.get("cols", 3),
            spacing_mm=params.get("spacing_mm", 1.0),
            center=center,
        )
    elif pattern == LayoutPattern.LINE:
        return auto_layout_line(
            count=params.get("count", 5),
            length_mm=params.get("length_mm", 4.0),
            angle_deg=params.get("angle_deg", 0.0),
            center=center,
        )
    elif pattern == LayoutPattern.CONCENTRIC_RINGS:
        return auto_layout_concentric_rings(
            num_rings=params.get("num_rings", 3),
            objects_per_ring=params.get("objects_per_ring", 6),
            inner_radius_mm=params.get("inner_radius_mm", 1.0),
            outer_radius_mm=params.get("outer_radius_mm", 3.0),
            center=center,
        )
    else:
        raise ValueError(f"Unknown layout pattern: {pattern}")


def get_layout_count(pattern: LayoutPattern | str, params: dict) -> int:
    """Get the total number of objects a layout will generate."""
    if isinstance(pattern, str):
        pattern = LayoutPattern(pattern)

    if pattern == LayoutPattern.RING:
        return params.get("count", 6)
    elif pattern in (LayoutPattern.SQUARE_GRID, LayoutPattern.HEX_GRID):
        return params.get("rows", 3) * params.get("cols", 3)
    elif pattern == LayoutPattern.LINE:
        return params.get("count", 5)
    elif pattern == LayoutPattern.CONCENTRIC_RINGS:
        return params.get("num_rings", 3) * params.get("objects_per_ring", 6)
    return 0


def validate_layout_in_well(
    positions: list[tuple[float, float]],
    well_diameter_mm: float,
) -> tuple[bool, list[str]]:
    """
    Check if all layout positions fit within a circular well.

    Returns (all_fit, list_of_warnings).
    """
    warnings = []
    well_radius = well_diameter_mm / 2
    all_fit = True

    for i, (x, y) in enumerate(positions):
        dist = math.sqrt(x ** 2 + y ** 2)
        if dist > well_radius:
            all_fit = False
            warnings.append(
                f"Object {i+1} at ({x:.1f}, {y:.1f}) is {dist - well_radius:.2f} mm "
                f"outside well boundary (r={well_radius:.1f} mm)"
            )

    return all_fit, warnings
