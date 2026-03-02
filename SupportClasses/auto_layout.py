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


# ═══════════════════════════════════════════════════════════════════
# Layout Parameter Definitions (for dynamic UI generation)
# ═══════════════════════════════════════════════════════════════════

LAYOUT_INFO: dict[str, dict[str, Any]] = {
    "ring": {
        "label": "Ring",
        "icon": "◎",
        "description": "Objects equally spaced around a circle",
        "params": {
            "n": {"label": "Count", "type": "int", "min": 1, "max": 100, "default": 6},
            "radius": {"label": "Radius (mm)", "type": "float", "min": 0.1, "max": 20.0, "default": 2.0, "step": 0.1},
            "start_angle": {"label": "Start Angle (°)", "type": "float", "min": 0, "max": 360, "default": 0.0, "step": 15},
        },
    },
    "square_grid": {
        "label": "Square Grid",
        "icon": "▦",
        "description": "Objects in a rectangular rows × columns grid",
        "params": {
            "rows": {"label": "Rows", "type": "int", "min": 1, "max": 20, "default": 3},
            "cols": {"label": "Columns", "type": "int", "min": 1, "max": 20, "default": 3},
            "spacing": {"label": "Spacing (mm)", "type": "float", "min": 0.05, "max": 10.0, "default": 1.0, "step": 0.05},
        },
    },
    "hex_grid": {
        "label": "Hex Grid",
        "icon": "⬡",
        "description": "Hexagonal (honeycomb) packing — efficient space use",
        "params": {
            "rows": {"label": "Rows", "type": "int", "min": 1, "max": 20, "default": 3},
            "cols": {"label": "Columns", "type": "int", "min": 1, "max": 20, "default": 3},
            "spacing": {"label": "Spacing (mm)", "type": "float", "min": 0.05, "max": 10.0, "default": 1.0, "step": 0.05},
        },
    },
    "line": {
        "label": "Line",
        "icon": "╱",
        "description": "Objects spaced along a straight line",
        "params": {
            "n": {"label": "Count", "type": "int", "min": 2, "max": 100, "default": 5},
            "start_x": {"label": "Start X (mm)", "type": "float", "min": -20, "max": 20, "default": -2.0, "step": 0.1},
            "start_y": {"label": "Start Y (mm)", "type": "float", "min": -20, "max": 20, "default": 0.0, "step": 0.1},
            "end_x": {"label": "End X (mm)", "type": "float", "min": -20, "max": 20, "default": 2.0, "step": 0.1},
            "end_y": {"label": "End Y (mm)", "type": "float", "min": -20, "max": 20, "default": 0.0, "step": 0.1},
        },
    },
    "concentric_rings": {
        "label": "Concentric Rings",
        "icon": "◉",
        "description": "Multiple rings with increasing radii",
        "params": {
            "ring_count": {"label": "Ring Count", "type": "int", "min": 1, "max": 10, "default": 3},
            "objects_per_ring": {"label": "Objects per Ring", "type": "int", "min": 2, "max": 50, "default": 6},
            "inner_radius": {"label": "Inner Radius (mm)", "type": "float", "min": 0.1, "max": 10.0, "default": 0.5, "step": 0.1},
            "outer_radius": {"label": "Outer Radius (mm)", "type": "float", "min": 0.2, "max": 20.0, "default": 3.0, "step": 0.1},
        },
    },
}


# ═══════════════════════════════════════════════════════════════════
# Layout Generators
# ═══════════════════════════════════════════════════════════════════

def generate_ring(
    n: int,
    radius: float = 2.0,
    start_angle: float = 0.0,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate positions equally spaced around a circle.

    Args:
        n: Number of objects
        radius: Circle radius in mm
        start_angle: Starting angle in degrees (0 = right / +X)
        center: Center point (x, y) in mm

    Returns:
        List of (x, y) positions
    """
    if n < 1:
        return []
    if n == 1:
        return [center]

    positions = []
    for i in range(n):
        angle_rad = math.radians(start_angle + i * 360.0 / n)
        x = center[0] + radius * math.cos(angle_rad)
        y = center[1] + radius * math.sin(angle_rad)
        positions.append((round(x, 4), round(y, 4)))
    return positions


def generate_square_grid(
    rows: int = 3,
    cols: int = 3,
    spacing: float = 1.0,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate positions in a rectangular grid centered on (0, 0).

    Args:
        rows: Number of rows
        cols: Number of columns
        spacing: Distance between adjacent objects in mm
        center: Center point (x, y) in mm
    """
    x0 = center[0] - (cols - 1) * spacing / 2
    y0 = center[1] - (rows - 1) * spacing / 2
    positions = []
    for r in range(rows):
        for c in range(cols):
            x = x0 + c * spacing
            y = y0 + r * spacing
            positions.append((round(x, 4), round(y, 4)))
    return positions


def generate_hex_grid(
    rows: int = 3,
    cols: int = 3,
    spacing: float = 1.0,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate positions in a hexagonal (honeycomb) grid.

    Odd rows are shifted right by half the spacing.
    """
    row_height = spacing * math.sqrt(3) / 2
    x0 = center[0] - (cols - 1) * spacing / 2
    y0 = center[1] - (rows - 1) * row_height / 2
    positions = []
    for r in range(rows):
        offset = spacing / 2 if r % 2 else 0
        for c in range(cols):
            x = x0 + c * spacing + offset
            y = y0 + r * row_height
            positions.append((round(x, 4), round(y, 4)))
    return positions


def generate_line(
    n: int = 5,
    start_x: float = -2.0,
    start_y: float = 0.0,
    end_x: float = 2.0,
    end_y: float = 0.0,
) -> list[tuple[float, float]]:
    """
    Generate positions equally spaced along a straight line.

    Args:
        n: Number of objects (minimum 2)
        start_x, start_y: Line start point
        end_x, end_y: Line end point
    """
    if n < 1:
        return []
    if n == 1:
        mx = (start_x + end_x) / 2
        my = (start_y + end_y) / 2
        return [(round(mx, 4), round(my, 4))]

    positions = []
    for i in range(n):
        t = i / (n - 1)
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        positions.append((round(x, 4), round(y, 4)))
    return positions


def generate_concentric_rings(
    ring_count: int = 3,
    objects_per_ring: int = 6,
    inner_radius: float = 0.5,
    outer_radius: float = 3.0,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    """
    Generate positions on concentric rings.

    Objects per ring is uniform. Rings are equally spaced between
    inner_radius and outer_radius.
    """
    if ring_count < 1 or objects_per_ring < 1:
        return []

    positions = []
    if ring_count == 1:
        radii = [inner_radius]
    else:
        radii = [
            inner_radius + i * (outer_radius - inner_radius) / (ring_count - 1)
            for i in range(ring_count)
        ]

    for ring_idx, radius in enumerate(radii):
        # Offset each ring slightly for visual distinction
        start_angle = ring_idx * 15.0  # 15° offset per ring
        ring_positions = generate_ring(
            n=objects_per_ring,
            radius=radius,
            start_angle=start_angle,
            center=center,
        )
        positions.extend(ring_positions)

    return positions


# ═══════════════════════════════════════════════════════════════════
# Dispatch + Count
# ═══════════════════════════════════════════════════════════════════

GENERATORS = {
    "ring": generate_ring,
    "square_grid": generate_square_grid,
    "hex_grid": generate_hex_grid,
    "line": generate_line,
    "concentric_rings": generate_concentric_rings,
}


def generate_layout(pattern: str, **kwargs) -> list[tuple[float, float]]:
    """
    Dispatch to the appropriate layout generator.

    Args:
        pattern: One of 'ring', 'square_grid', 'hex_grid', 'line', 'concentric_rings'
        **kwargs: Parameters specific to the chosen pattern

    Returns:
        List of (x, y) positions in mm
    """
    gen = GENERATORS.get(pattern)
    if gen is None:
        raise ValueError(f"Unknown layout pattern: {pattern}. "
                         f"Available: {list(GENERATORS.keys())}")
    return gen(**kwargs)


def count_layout_objects(pattern: str, **kwargs) -> int:
    """
    Compute how many objects a layout will produce without generating positions.
    """
    if pattern == "ring":
        return kwargs.get("n", 6)
    elif pattern in ("square_grid", "hex_grid"):
        return kwargs.get("rows", 3) * kwargs.get("cols", 3)
    elif pattern == "line":
        return kwargs.get("n", 5)
    elif pattern == "concentric_rings":
        return kwargs.get("ring_count", 3) * kwargs.get("objects_per_ring", 6)
    else:
        raise ValueError(f"Unknown pattern: {pattern}")


# ═══════════════════════════════════════════════════════════════════
# Validation Against Well Boundary
# ═══════════════════════════════════════════════════════════════════

def validate_layout_in_well(
    positions: list[tuple[float, float]],
    well_diameter_mm: float,
) -> tuple[bool, list[str]]:
    """
    Check whether all positions fit within a circular well.

    Args:
        positions: List of (x, y) from a layout generator
        well_diameter_mm: Well diameter in mm

    Returns:
        (all_fit, list_of_warnings)
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
