"""
Well Plate Geometry — Standard ANSI/SLAS well plate definitions and path generators.

Provides coordinate geometry for 6, 12, 24, 48, 96, and 384-well plates.
All coordinates are relative to the A1 well centre, which aligns
with the zero reference position set during calibration.

Path generators produce lists of (x, y) waypoints for common fill
patterns: line, meander, spiral, grid, and concentric rings.

v7.1 additions:
- well_depth_mm per plate format (P8.10)
- per-well bottom_z_offset storage from plane fitting (P8.11)
- rosette_insert field on WellInfo for attached geometry (P8.12)
- 384-well plate definition (P8.13)

Usage::

    plate = WellPlate.from_format(96)
    x, y = plate.get_well_position("B3")
    wells = plate.get_all_wells()
    path = generate_meander_path(5.0, 5.0, 0.5)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# ANSI/SLAS Standard Plate Definitions (all measurements in mm)
# ═══════════════════════════════════════════════════════════════════

PLATE_DEFINITIONS: dict[int, dict] = {
    6: {
        "rows": 2, "cols": 3,
        "well_spacing_x": 39.12, "well_spacing_y": 39.12,
        "well_diameter": 34.8,
        "well_depth_mm": 17.4,
        "a1_offset_x": 24.76, "a1_offset_y": 23.16,
        "description": "6-well plate",
    },
    12: {
        "rows": 3, "cols": 4,
        "well_spacing_x": 26.01, "well_spacing_y": 26.01,
        "well_diameter": 22.1,
        "well_depth_mm": 17.4,
        "a1_offset_x": 24.94, "a1_offset_y": 16.79,
        "description": "12-well plate",
    },
    24: {
        "rows": 4, "cols": 6,
        "well_spacing_x": 19.30, "well_spacing_y": 19.30,
        "well_diameter": 15.6,
        "well_depth_mm": 17.4,
        "a1_offset_x": 17.05, "a1_offset_y": 13.67,
        "description": "24-well plate",
    },
    48: {
        "rows": 6, "cols": 8,
        "well_spacing_x": 13.00, "well_spacing_y": 13.00,
        "well_diameter": 11.0,
        "well_depth_mm": 17.4,
        "a1_offset_x": 18.16, "a1_offset_y": 10.08,
        "description": "48-well plate",
    },
    96: {
        "rows": 8, "cols": 12,
        "well_spacing_x": 9.00, "well_spacing_y": 9.00,
        "well_diameter": 6.35,
        "well_depth_mm": 10.67,
        "a1_offset_x": 14.38, "a1_offset_y": 11.24,
        "description": "96-well plate",
    },
    384: {
        "rows": 16, "cols": 24,
        "well_spacing_x": 4.50, "well_spacing_y": 4.50,
        "well_diameter": 3.63,
        "well_depth_mm": 11.56,
        "a1_offset_x": 12.13, "a1_offset_y": 8.99,
        "description": "384-well plate",
    },
}

# Extended row labels for 384-well plates (rows A–P)
ROW_LABELS = "ABCDEFGHIJKLMNOP"


# ═══════════════════════════════════════════════════════════════════
# Data Structures
# ═══════════════════════════════════════════════════════════════════

@dataclass
class WellInfo:
    """Geometry for a single well."""
    name: str           # e.g. "A1", "B3"
    row: int            # 0-indexed
    col: int            # 0-indexed
    x: float            # X centre (mm, relative to A1)
    y: float            # Y centre (mm, relative to A1)
    diameter: float     # Well diameter (mm)

    # v7.1 additions (P8.11, P8.12)
    bottom_z_offset: float = 0.0    # Z offset from plane fit (mm), 0 = on reference plane
    rosette_insert: Any = None      # Attached RosetteInsert geometry (or None)


@dataclass
class WellPlate:
    """
    A well plate with coordinate geometry for each well.

    All coordinates are relative to A1 centre (0, 0).
    The stage controller's zero_position maps A1 to the physical origin.
    """
    format: int
    rows: int
    cols: int
    well_spacing_x: float
    well_spacing_y: float
    well_diameter: float
    well_depth_mm: float            # v7.1 (P8.10): Typical well depth for this format
    a1_offset_x: float
    a1_offset_y: float
    description: str
    _wells: dict[str, WellInfo] = field(default_factory=dict, repr=False)

    def __post_init__(self):
        """Compute well coordinates."""
        self._wells = {}
        for r in range(self.rows):
            for c in range(self.cols):
                name = f"{ROW_LABELS[r]}{c + 1}"
                self._wells[name] = WellInfo(
                    name=name,
                    row=r,
                    col=c,
                    x=c * self.well_spacing_x,
                    y=r * self.well_spacing_y,
                    diameter=self.well_diameter,
                )

    # ── Construction ──────────────────────────────────────────────

    @classmethod
    def from_format(cls, well_count: int) -> WellPlate:
        """
        Create a plate from a standard format (6, 12, 24, 48, 96, 384).

        Raises ValueError for unsupported formats.
        """
        if well_count not in PLATE_DEFINITIONS:
            raise ValueError(
                f"Unsupported format: {well_count}. "
                f"Choose from {sorted(PLATE_DEFINITIONS)}"
            )
        return cls(format=well_count, **PLATE_DEFINITIONS[well_count])

    # ── Lookups ───────────────────────────────────────────────────

    def get_well_position(self, well_name: str) -> tuple[float, float]:
        """Return (x, y) centre of *well_name* (e.g. 'A1'), relative to A1."""
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not in {self.format}-well plate")
        return (well.x, well.y)

    def get_well_info(self, well_name: str) -> WellInfo:
        """Return full :class:`WellInfo` for a well."""
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not found")
        return well

    def get_all_wells(self) -> list[WellInfo]:
        """All wells in row-major order (A1, A2, …, B1, B2, …)."""
        return sorted(self._wells.values(), key=lambda w: (w.row, w.col))

    def get_wells(self, names: list[str]) -> list[WellInfo]:
        """Subset of wells by name (unknown names logged and skipped)."""
        result = []
        for name in names:
            well = self._wells.get(name.upper())
            if well:
                result.append(well)
            else:
                logger.warning(f"Well '{name}' not found — skipping")
        return result

    def get_row(self, row_letter: str) -> list[WellInfo]:
        """All wells in a row, e.g. 'A' → [A1, A2, …]."""
        idx = ROW_LABELS.index(row_letter.upper())
        return sorted(
            (w for w in self._wells.values() if w.row == idx),
            key=lambda w: w.col,
        )

    def get_column(self, col_number: int) -> list[WellInfo]:
        """All wells in a column (1-indexed)."""
        return sorted(
            (w for w in self._wells.values() if w.col == col_number - 1),
            key=lambda w: w.row,
        )

    # ── v7.1 Well Z Offset Methods (P8.11) ───────────────────────

    def set_well_z_offset(self, well_name: str, z_offset: float) -> None:
        """Set the bottom Z offset for a well (from plane fitting)."""
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not found")
        well.bottom_z_offset = z_offset

    def get_well_z_offset(self, well_name: str) -> float:
        """Get the bottom Z offset for a well."""
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not found")
        return well.bottom_z_offset

    def set_z_offsets_from_plane(self, z_offsets: dict[str, float]) -> None:
        """
        Bulk-set Z offsets from a plane fit result.

        Args:
            z_offsets: Dict mapping well names to Z offsets (mm)
        """
        for name, offset in z_offsets.items():
            well = self._wells.get(name.upper())
            if well is not None:
                well.bottom_z_offset = offset

    def get_all_z_offsets(self) -> dict[str, float]:
        """Get Z offsets for all wells as a dict."""
        return {
            name: well.bottom_z_offset
            for name, well in self._wells.items()
        }

    def clear_z_offsets(self) -> None:
        """Reset all well Z offsets to zero."""
        for well in self._wells.values():
            well.bottom_z_offset = 0.0

    # ── v7.1 Rosette Methods (P8.12) ─────────────────────────────

    def attach_rosette(self, well_name: str, rosette_insert: Any) -> None:
        """
        Attach a RosetteInsert to a well.

        Args:
            well_name: Target well (e.g. "A1")
            rosette_insert: RosetteInsert instance (from PhysicalModels)
        """
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not found")
        well.rosette_insert = rosette_insert
        logger.debug(f"Rosette attached to well {well_name}")

    def detach_rosette(self, well_name: str) -> None:
        """Remove rosette insert from a well."""
        well = self._wells.get(well_name.upper())
        if well is not None:
            well.rosette_insert = None

    def get_wells_with_rosettes(self) -> list[WellInfo]:
        """Return all wells that have rosette inserts attached."""
        return [w for w in self._wells.values() if w.rosette_insert is not None]

    # ── Properties ────────────────────────────────────────────────

    @property
    def well_names(self) -> list[str]:
        return [w.name for w in self.get_all_wells()]

    @property
    def plate_width(self) -> float:
        """Total width in mm (X direction)."""
        return (self.cols - 1) * self.well_spacing_x

    @property
    def plate_height(self) -> float:
        """Total height in mm (Y direction)."""
        return (self.rows - 1) * self.well_spacing_y

    def get_bounding_box(self) -> tuple[float, float, float, float]:
        """Plate bounding box relative to A1: (min_x, min_y, max_x, max_y)."""
        r = self.well_diameter / 2
        return (-r, -r, self.plate_width + r, self.plate_height + r)


# ═══════════════════════════════════════════════════════════════════
# Path Generators
# ═══════════════════════════════════════════════════════════════════

def generate_line_path(
    length: float,
    angle_deg: float = 0.0,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float]]:
    """Straight line centred at (center_x, center_y)."""
    rad = math.radians(angle_deg)
    dx = math.cos(rad) * length / 2
    dy = math.sin(rad) * length / 2
    return [(center_x - dx, center_y - dy), (center_x + dx, center_y + dy)]


def generate_meander_path(
    width: float,
    height: float,
    line_spacing: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float]]:
    """Serpentine/raster fill pattern."""
    points: list[tuple[float, float]] = []
    x0 = center_x - width / 2
    x1 = center_x + width / 2
    y_start = center_y - height / 2

    num_lines = max(1, int(height / line_spacing) + 1)
    actual_spacing = height / max(num_lines - 1, 1)

    for i in range(num_lines):
        y = y_start + i * actual_spacing
        if i % 2 == 0:
            points.extend([(x0, y), (x1, y)])
        else:
            points.extend([(x1, y), (x0, y)])
    return points


def generate_spiral_path(
    diameter: float,
    line_spacing: float,
    turns: int = 0,
    center_x: float = 0.0,
    center_y: float = 0.0,
    points_per_turn: int = 36,
) -> list[tuple[float, float]]:
    """Archimedean spiral path."""
    radius = diameter / 2
    if turns <= 0:
        turns = max(1, int(radius / line_spacing))

    total_points = turns * points_per_turn
    points: list[tuple[float, float]] = []
    for i in range(total_points + 1):
        frac = i / total_points
        angle = frac * turns * 2 * math.pi
        r = frac * radius
        points.append((center_x + r * math.cos(angle), center_y + r * math.sin(angle)))
    return points


def generate_grid_path(
    width: float,
    height: float,
    spacing_x: float,
    spacing_y: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float]]:
    """Grid of points (for dot/droplet printing)."""
    points: list[tuple[float, float]] = []
    x0 = center_x - width / 2
    y0 = center_y - height / 2
    cols = max(1, int(width / spacing_x) + 1)
    rows = max(1, int(height / spacing_y) + 1)

    for r in range(rows):
        for c in range(cols):
            points.append((x0 + c * spacing_x, y0 + r * spacing_y))
    return points


def generate_concentric_rings(
    diameter: float,
    ring_spacing: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    points_per_ring: int = 36,
) -> list[list[tuple[float, float]]]:
    """Concentric circular rings (each ring is a separate path)."""
    radius = diameter / 2
    num_rings = max(1, int(radius / ring_spacing))
    rings: list[list[tuple[float, float]]] = []

    for ring_idx in range(1, num_rings + 1):
        r = ring_idx * ring_spacing
        ring_points = [
            (
                center_x + r * math.cos(i / points_per_ring * 2 * math.pi),
                center_y + r * math.sin(i / points_per_ring * 2 * math.pi),
            )
            for i in range(points_per_ring + 1)
        ]
        rings.append(ring_points)
    return rings
