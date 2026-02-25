"""
Well Plate Geometry - Standard laboratory well plate definitions.

Provides coordinate geometry for common well plate formats (6, 12, 24, 48, 96 well).
All dimensions follow ANSI/SLAS microplate standards (127.76 x 85.48 mm footprint).

Coordinates are generated relative to the A1 well center, which should be aligned
with the zero reference position set during calibration.

Usage:
    plate = WellPlate.from_format(96)
    wells = plate.get_all_wells()         # [('A1', x, y), ('A2', x, y), ...]
    x, y = plate.get_well_position('B3')  # Single well lookup
    subset = plate.get_wells(['A1', 'A2', 'B1', 'B2'])  # Subset
"""

from dataclasses import dataclass, field
import logging

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# ANSI/SLAS Standard Well Plate Definitions
# ═══════════════════════════════════════════════════════════════════
# All measurements in mm
# A1 offset = distance from plate corner to center of A1 well

PLATE_DEFINITIONS = {
    6: {
        "rows": 2, "cols": 3,
        "well_spacing_x": 39.12, "well_spacing_y": 39.12,
        "well_diameter": 34.8,
        "a1_offset_x": 24.76, "a1_offset_y": 23.16,
        "description": "6-well plate",
    },
    12: {
        "rows": 3, "cols": 4,
        "well_spacing_x": 26.01, "well_spacing_y": 26.01,
        "well_diameter": 22.1,
        "a1_offset_x": 24.94, "a1_offset_y": 16.79,
        "description": "12-well plate",
    },
    24: {
        "rows": 4, "cols": 6,
        "well_spacing_x": 19.30, "well_spacing_y": 19.30,
        "well_diameter": 15.6,
        "a1_offset_x": 17.05, "a1_offset_y": 13.67,
        "description": "24-well plate",
    },
    48: {
        "rows": 6, "cols": 8,
        "well_spacing_x": 13.00, "well_spacing_y": 13.00,
        "well_diameter": 11.0,
        "a1_offset_x": 18.16, "a1_offset_y": 10.08,
        "description": "48-well plate",
    },
    96: {
        "rows": 8, "cols": 12,
        "well_spacing_x": 9.00, "well_spacing_y": 9.00,
        "well_diameter": 6.35,
        "a1_offset_x": 14.38, "a1_offset_y": 11.24,
        "description": "96-well plate",
    },
}

ROW_LABELS = "ABCDEFGH"


@dataclass
class WellInfo:
    """Information about a single well."""
    name: str           # e.g. "A1", "B3"
    row: int            # 0-indexed row
    col: int            # 0-indexed column
    x: float            # X center in mm (relative to A1)
    y: float            # Y center in mm (relative to A1)
    diameter: float     # Well diameter in mm


@dataclass
class WellPlate:
    """
    Represents a well plate with geometry and coordinate lookup.
    
    Coordinates are relative to A1 well center (0, 0).
    During printing, the StageController's zero_position maps A1 to the
    physical calibrated origin.
    """
    format: int
    rows: int
    cols: int
    well_spacing_x: float
    well_spacing_y: float
    well_diameter: float
    a1_offset_x: float
    a1_offset_y: float
    description: str
    _wells: dict = field(default_factory=dict, repr=False)

    def __post_init__(self):
        """Generate well coordinates."""
        self._wells = {}
        for r in range(self.rows):
            for c in range(self.cols):
                name = f"{ROW_LABELS[r]}{c + 1}"
                x = c * self.well_spacing_x
                y = r * self.well_spacing_y
                self._wells[name] = WellInfo(
                    name=name, row=r, col=c,
                    x=x, y=y,
                    diameter=self.well_diameter,
                )

    @classmethod
    def from_format(cls, well_count: int) -> "WellPlate":
        """
        Create a WellPlate from a standard format.
        
        Args:
            well_count: 6, 12, 24, 48, or 96
            
        Returns:
            WellPlate instance
            
        Raises:
            ValueError: If well_count is not a supported format
        """
        if well_count not in PLATE_DEFINITIONS:
            raise ValueError(
                f"Unsupported plate format: {well_count}. "
                f"Choose from {list(PLATE_DEFINITIONS.keys())}"
            )
        defn = PLATE_DEFINITIONS[well_count]
        return cls(format=well_count, **defn)

    def get_well_position(self, well_name: str) -> tuple[float, float]:
        """
        Get the (x, y) center coordinate for a well.
        
        Args:
            well_name: Well identifier like "A1", "B3", "H12"
            
        Returns:
            (x, y) tuple in mm relative to A1
        """
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not found in {self.format}-well plate")
        return (well.x, well.y)

    def get_well_info(self, well_name: str) -> WellInfo:
        """Get full WellInfo for a well."""
        well = self._wells.get(well_name.upper())
        if well is None:
            raise KeyError(f"Well '{well_name}' not found")
        return well

    def get_all_wells(self) -> list[WellInfo]:
        """Get all wells in order (row-major: A1, A2, ..., B1, B2, ...)."""
        return sorted(self._wells.values(), key=lambda w: (w.row, w.col))

    def get_wells(self, names: list[str]) -> list[WellInfo]:
        """Get a subset of wells by name."""
        result = []
        for name in names:
            well = self._wells.get(name.upper())
            if well:
                result.append(well)
            else:
                logger.warning(f"Well '{name}' not found, skipping")
        return result

    def get_row(self, row_letter: str) -> list[WellInfo]:
        """Get all wells in a row (e.g., 'A' returns A1..A12)."""
        row_idx = ROW_LABELS.index(row_letter.upper())
        return [w for w in self._wells.values() if w.row == row_idx]

    def get_column(self, col_number: int) -> list[WellInfo]:
        """Get all wells in a column (1-indexed)."""
        return [w for w in self._wells.values() if w.col == col_number - 1]

    @property
    def well_names(self) -> list[str]:
        """All well names in order."""
        return [w.name for w in self.get_all_wells()]

    @property
    def plate_width(self) -> float:
        """Total plate width in mm (X direction)."""
        return (self.cols - 1) * self.well_spacing_x

    @property
    def plate_height(self) -> float:
        """Total plate height in mm (Y direction)."""
        return (self.rows - 1) * self.well_spacing_y

    def get_bounding_box(self) -> tuple[float, float, float, float]:
        """
        Get plate bounding box relative to A1.
        Returns: (min_x, min_y, max_x, max_y)
        """
        return (
            -self.well_diameter / 2,
            -self.well_diameter / 2,
            self.plate_width + self.well_diameter / 2,
            self.plate_height + self.well_diameter / 2,
        )


# ═══════════════════════════════════════════════════════════════════
# Path Generators - Common printing patterns within a well
# ═══════════════════════════════════════════════════════════════════

def generate_line_path(length: float, angle_deg: float = 0,
                       center_x: float = 0, center_y: float = 0) -> list[tuple[float, float]]:
    """
    Generate a straight line path centered at (center_x, center_y).
    
    Returns list of (x, y) points.
    """
    import math
    rad = math.radians(angle_deg)
    dx = math.cos(rad) * length / 2
    dy = math.sin(rad) * length / 2
    return [
        (center_x - dx, center_y - dy),
        (center_x + dx, center_y + dy),
    ]


def generate_meander_path(width: float, height: float, line_spacing: float,
                          center_x: float = 0, center_y: float = 0) -> list[tuple[float, float]]:
    """
    Generate a meander (serpentine/raster) fill pattern.
    
    Args:
        width: Total width of the meander
        height: Total height of the meander
        line_spacing: Distance between parallel lines
        center_x, center_y: Center offset
        
    Returns:
        List of (x, y) waypoints
    """
    points = []
    x_start = center_x - width / 2
    x_end = center_x + width / 2
    y_start = center_y - height / 2

    num_lines = max(1, int(height / line_spacing) + 1)
    actual_spacing = height / max(num_lines - 1, 1)

    for i in range(num_lines):
        y = y_start + i * actual_spacing
        if i % 2 == 0:
            points.append((x_start, y))
            points.append((x_end, y))
        else:
            points.append((x_end, y))
            points.append((x_start, y))

    return points


def generate_spiral_path(diameter: float, line_spacing: float, turns: int = 0,
                         center_x: float = 0, center_y: float = 0,
                         points_per_turn: int = 36) -> list[tuple[float, float]]:
    """
    Generate an Archimedean spiral path.
    
    Args:
        diameter: Maximum diameter of the spiral
        line_spacing: Radial spacing between turns
        turns: Number of turns (auto-calculated from diameter/spacing if 0)
        center_x, center_y: Center offset
        points_per_turn: Resolution of the spiral
        
    Returns:
        List of (x, y) waypoints
    """
    import math

    radius = diameter / 2
    if turns == 0:
        turns = max(1, int(radius / line_spacing))

    total_points = turns * points_per_turn
    points = []

    for i in range(total_points + 1):
        fraction = i / total_points
        angle = fraction * turns * 2 * math.pi
        r = fraction * radius
        x = center_x + r * math.cos(angle)
        y = center_y + r * math.sin(angle)
        points.append((x, y))

    return points


def generate_grid_path(width: float, height: float, spacing_x: float, spacing_y: float,
                       center_x: float = 0, center_y: float = 0) -> list[tuple[float, float]]:
    """
    Generate a grid of points (for dot/droplet printing).
    
    Returns:
        List of (x, y) points at grid intersections
    """
    points = []
    x_start = center_x - width / 2
    y_start = center_y - height / 2

    cols = max(1, int(width / spacing_x) + 1)
    rows = max(1, int(height / spacing_y) + 1)

    for r in range(rows):
        for c in range(cols):
            x = x_start + c * spacing_x
            y = y_start + r * spacing_y
            points.append((x, y))

    return points


def generate_concentric_rings(diameter: float, ring_spacing: float,
                              center_x: float = 0, center_y: float = 0,
                              points_per_ring: int = 36) -> list[list[tuple[float, float]]]:
    """
    Generate concentric circular rings.
    
    Returns:
        List of rings, each ring is a list of (x, y) points.
    """
    import math

    radius = diameter / 2
    num_rings = max(1, int(radius / ring_spacing))
    rings = []

    for ring in range(1, num_rings + 1):
        r = ring * ring_spacing
        ring_points = []
        for i in range(points_per_ring + 1):
            angle = i / points_per_ring * 2 * math.pi
            x = center_x + r * math.cos(angle)
            y = center_y + r * math.sin(angle)
            ring_points.append((x, y))
        rings.append(ring_points)

    return rings
