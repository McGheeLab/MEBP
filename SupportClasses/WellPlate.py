"""
Well Plate Geometry — Standard ANSI/SLAS well plate definitions and path generators.

Provides coordinate geometry for 6, 12, 24, 48, 96, and 384-well plates,
plus user-designed custom plates with arbitrary per-well position and diameter.

All standard-format coordinates are relative to the A1 well centre, which aligns
with the zero reference position set during calibration. Custom plates
likewise place their first/anchor well at the workspace origin.

Orientation convention (v7.5.x — ONE convention everywhere):
  * This data model is PLATE-LOCAL: A1 is at relative (0, 0); column index
    increases +X (drawn to screen-right), row index increases +Y (drawn
    screen-down). The model itself is orientation-agnostic and is NEVER
    flipped — ``get_well_position("H12")`` is always ``(99.0, 63.0)`` on a
    96-well plate.
  * The canonical machine convention is "well A1 displays TOP-LEFT; stage
    physical 0,0 is BOTTOM-RIGHT". Whether the plate-local axes align with the
    stage axes is a PER-MACHINE property (``StageController.plate_flip_180``).
    On ME3B V1 (Prior origin bottom-right, +X/+Y toward top-left) they are
    ANTI-aligned, so the plate-local→stage mapping multiplies the A1-relative
    offset by ``plate_axis_sign`` = ``(-1, -1)``.
  * The geometric helpers below (``get_all_positions_from_a1`` /
    ``get_a1_from_plate_center`` / ``get_well_area_bounds_*``) take an optional
    ``plate_axis_sign`` (default ``(1, 1)`` = aligned/legacy) so callers that
    turn a well into a STAGE coordinate pass ``controller.plate_axis_sign()``.
    Positions that are explicitly TAUGHT/warped are already absolute stage µm
    and must NOT be re-signed.

Path generators produce lists of (x, y) waypoints for common fill
patterns: line, meander, spiral, grid, and concentric rings.

v7.1 additions:
- well_depth_mm per plate format (P8.10)
- per-well bottom_z_offset storage from plane fitting (P8.11)
- rosette_insert field on WellInfo for attached geometry (P8.12)
- 384-well plate definition (P8.13)

v7.4.5 additions:
- `format: int | str` — custom plates use `"custom:<name>"`
- `from_wells(name, wells, **meta)` factory for arbitrary per-well plates
- `load(name_or_format)` polymorphic factory: int → standard, str → user JSON
- `__post_init__` guarded so explicit `_wells` skip grid auto-fill

Usage::

    plate = WellPlate.from_format(96)              # legacy, unchanged
    plate = WellPlate.load(96)                     # equivalent
    plate = WellPlate.load("my-coverslip-array")   # user-saved custom plate
    x, y = plate.get_well_position("B3")
    wells = plate.get_all_wells()
    path = generate_meander_path(5.0, 5.0, 0.5)
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


# v7.4.5: User-saved custom plate designs live here as JSON.
# Standards are still computed in-code from `PLATE_DEFINITIONS`.
USER_PLATES_DIR = (
    Path(__file__).resolve().parent.parent / "config" / "hardware" / "plates" / "user"
)


# ═══════════════════════════════════════════════════════════════════
# ANSI/SLAS Standard Plate Footprint (all microplates share this)
# ═══════════════════════════════════════════════════════════════════

PLATE_FOOTPRINT_X_MM = 127.76   # Length (mm) — long axis
PLATE_FOOTPRINT_Y_MM = 85.48    # Width (mm) — short axis

# Stage travel limits (mm) — raw stage coordinate range (0 to max)
STAGE_TRAVEL_X_MM = 130.0
STAGE_TRAVEL_Y_MM = 85.0


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
    rosette_insert: Any = None      # Attached RosetteInsert geometry (or None) — legacy

    # v7.4.8 — insert/tube geometry (rosette sub-wells flatten into these).
    # Larger Z = higher / further from the plate (matches calibration safe_z).
    rim_height_mm: float = 0.0      # height the well/tube top sits ABOVE the
                                    # plate top surface (clearance); 0 = flush
    ink_z_mm: float | None = None   # prescribed ink-dispense Z relative to the
                                    # plate top; None = use the global print Z
    is_subwell: bool = False        # True = flattened rosette sub-well
    parent_well: str | None = None  # parent well name (e.g. "A1") for subwells


@dataclass
class WellPlate:
    """
    A well plate with coordinate geometry for each well.

    All coordinates are relative to A1 centre (0, 0).
    The stage controller's zero_position maps A1 to the physical origin.

    Standard formats (6/12/24/48/96/384) have uniform per-well diameters and
    a regular grid. v7.4.5 generalizes the dataclass to also hold custom
    plates where each well has its own position and diameter; in that case
    `format` is a string `"custom:<name>"`, `well_spacing_x/y` and
    `well_diameter` are 0.0 ("varies — read per-well from WellInfo"), and
    `rows` / `cols` are advisory (max row/col + 1).
    """
    format: int | str
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
        """Compute well coordinates for standard-format grids.

        v7.4.5: skip auto-fill if `_wells` was already supplied (custom plates
        constructed via `from_wells()`).
        """
        if self._wells:
            return
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

    @classmethod
    def from_wells(
        cls,
        name: str,
        wells: list[WellInfo],
        well_depth_mm: float = 10.67,
        a1_offset_x: float = 0.0,
        a1_offset_y: float = 0.0,
        description: str = "",
    ) -> WellPlate:
        """
        Construct a custom plate from an explicit list of `WellInfo`.

        Skips the grid auto-fill in `__post_init__`. `rows` / `cols` are
        derived as `max(row)+1` / `max(col)+1` for backward compat with
        callers that index by row/col; spacings and the plate-level
        diameter are 0.0 ("varies — see WellInfo per well").

        Args:
            name: Human-readable plate name; stored in `format` as
                  `"custom:<name>"` to distinguish from standard formats.
            wells: All wells on the plate. Each must have its name, row,
                   col, x, y, and diameter set.
            well_depth_mm: Plate-level default well depth (mm).
            a1_offset_x / a1_offset_y: Offset (mm) from the plate's
                top-left corner to the A1 (or first/anchor) well centre,
                used by `get_a1_from_plate_center()` for ANSI/SLAS-style
                positioning.
            description: Optional human-readable description.
        """
        if not wells:
            raise ValueError("from_wells: at least one WellInfo required")

        max_row = max(w.row for w in wells)
        max_col = max(w.col for w in wells)

        plate = cls(
            format=f"custom:{name}",
            rows=max_row + 1,
            cols=max_col + 1,
            well_spacing_x=0.0,
            well_spacing_y=0.0,
            well_diameter=0.0,
            well_depth_mm=well_depth_mm,
            a1_offset_x=a1_offset_x,
            a1_offset_y=a1_offset_y,
            description=description,
            # Key by UPPER-cased name so case-insensitive lookups
            # (get_well_info/get_well_position uppercase the query) resolve
            # sub-wells like "A1.a" (display name preserved on WellInfo).
            _wells={w.name.upper(): w for w in wells},
        )
        return plate

    @classmethod
    def load(cls, name_or_format: int | str) -> WellPlate:
        """
        Polymorphic loader. v7.4.5.

        - `int` → standard format (delegates to `from_format`).
        - `str` matching a standard format (e.g. "96") → `from_format(int)`.
        - `str` like ``"custom:<name>"`` (the WellPlate.format encoding) →
          strips the prefix and resolves to the user file.
        - Any other `str` → looks for `config/hardware/plates/user/<name>.json`
          and deserializes it as a `PlateDesign`, then `compile()`s to a
          `WellPlate`.
        """
        # Accept stringified ints transparently
        if isinstance(name_or_format, str) and name_or_format.isdigit():
            return cls.from_format(int(name_or_format))
        if isinstance(name_or_format, int):
            return cls.from_format(name_or_format)

        name = name_or_format

        # v7.5.x: a selectable plate TYPE (Corning glass-bottom, NEST plastic,
        # …) is a thin overlay on a base standard format — resolve it to that
        # format's geometry. The type's identity lives in `active_plate_key`
        # (so per-plate mosaic/template/well-training stores segregate), NOT
        # here: the runtime `format` stays the base int so every same-format
        # guard / PLATE_DEFINITIONS lookup keeps working. Lazy import + guarded
        # so an unknown id never crashes a caller.
        try:
            from SupportClasses.PlateTypeStore import get_store as _pt_store
            plate_type = _pt_store().get(name)
        except Exception:   # pragma: no cover - defensive
            plate_type = None
        if plate_type is not None and plate_type.base_format in PLATE_DEFINITIONS:
            plate = cls.from_format(plate_type.base_format)
            if plate_type.well_depth_mm is not None:
                plate.well_depth_mm = float(plate_type.well_depth_mm)
            return plate

        # Strip the "custom:" tag the dataclass uses internally so users
        # can round-trip `WellPlate.load(plate.format)`.
        if name.startswith("custom:"):
            name = name[7:]

        # v7.12: a parametric PlateDocument, resolved by stable id. Probed
        # BEFORE the v1 path so the two schemes coexist during the transition —
        # rolling back is deleting this block.
        plate = cls._load_plate_document(name)
        if plate is not None:
            return plate

        path = USER_PLATES_DIR / f"{name}.json"
        if not path.exists():
            raise FileNotFoundError(
                f"No plate named '{name}' under {USER_PLATES_DIR}"
            )
        # Import locally to avoid a circular import: PlateDesign imports WellPlate.
        from SupportClasses.PlateDesign import PlateDesign
        with open(path) as f:
            data = json.load(f)
        design = PlateDesign.from_dict(data)
        return design.compile()

    @classmethod
    def _load_plate_document(cls, key: str) -> "WellPlate | None":
        """Resolve *key* as a v7.12 ``PlateDocument``, or None.

        Tries the stable id first, then falls back to a display-name lookup.
        The fallback matters: without it, a saved ``HardwareConfig`` still
        naming a plate the old way would silently fall through to the 96-well
        default instead of loading the operator's plate.

        Every failure degrades to None so the v1 path still gets its turn, and
        the import is local because PlateDocument imports this module.
        """
        try:
            from SupportClasses.PlateDocumentStore import (
                get_plate_store, get_rosette_store,
            )
            store = get_plate_store()
            doc = store.get(key)
            if doc is None:
                summary = store.find_by_name(key)
                doc = store.get(summary.id) if summary else None
            if doc is None:
                return None
            return doc.compile(loader=get_rosette_store().get)
        except Exception as exc:   # pragma: no cover - defensive
            logger.debug("Not a v7.12 plate document (%s): %s", key, exc)
            return None

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
    def footprint_mm(self) -> tuple[float, float]:
        """This plate's physical outline ``(width, height)`` in mm.

        v7.12: resolved from the authoring ``PlateDocument``'s boundary, which
        is where the footprint is actually declared — ``WellPlate`` itself is a
        compiled well list and deliberately does not carry a second copy.
        Falls back to the ANSI/SLAS footprint when the plate cannot be resolved
        (an unsaved plate, a synthetic test plate), which is the value this used
        to hardcode, so a plate that IS standard is unaffected.

        Cached: it is read on every default-position seed, and the resolution
        walks the plate stores.
        """
        cached = getattr(self, "_footprint_cache", None)
        if cached is not None:
            return cached
        footprint = (PLATE_FOOTPRINT_X_MM, PLATE_FOOTPRINT_Y_MM)
        try:
            # Local import: PlateDocumentStore -> PlateDocument -> WellPlate.
            from SupportClasses.PlateDocumentStore import (
                plate_footprint_extent_mm,
            )
            extent = plate_footprint_extent_mm(self.format)
            if extent is not None:
                w, h = extent[2] - extent[0], extent[3] - extent[1]
                if w > 0 and h > 0:
                    footprint = (float(w), float(h))
        except Exception as exc:                           # pragma: no cover
            logger.debug("Footprint lookup failed for %s: %s", self.format, exc)
        object.__setattr__(self, "_footprint_cache", footprint)
        return footprint

    @property
    def is_custom(self) -> bool:
        """v7.4.5: True iff this plate was built from a custom design."""
        return isinstance(self.format, str) and self.format.startswith("custom:")

    @property
    def plate_width(self) -> float:
        """Total width in mm (X direction)."""
        if self.well_spacing_x > 0:
            return (self.cols - 1) * self.well_spacing_x
        # Custom plate: derive from actual well extents.
        wells = self._wells.values()
        if not wells:
            return 0.0
        return max(w.x for w in wells) - min(w.x for w in wells)

    @property
    def plate_height(self) -> float:
        """Total height in mm (Y direction)."""
        if self.well_spacing_y > 0:
            return (self.rows - 1) * self.well_spacing_y
        wells = self._wells.values()
        if not wells:
            return 0.0
        return max(w.y for w in wells) - min(w.y for w in wells)

    @property
    def max_rim_height_mm(self) -> float:
        """v7.4.8: tallest insert rim above the plate top across all wells.

        Drives the plate-wide travel-Z clearance floor so the needle
        clears the highest tube/insert when moving across the plate.
        Returns 0.0 for plates with no tall inserts.
        """
        return max((w.rim_height_mm for w in self._wells.values()), default=0.0)

    def _max_well_radius_mm(self) -> float:
        """v7.4.5: largest well radius on the plate (for custom plates)."""
        return self.representative_well_diameter / 2.0

    @property
    def representative_well_diameter(self) -> float:
        """v7.12: ONE well diameter (mm) to size things that aren't per-well.

        A parametric plate has no single diameter — ``from_wells`` stores 0.0
        on purpose ("varies — see WellInfo per well") — so every caller that
        reaches for the plate-level ``well_diameter`` gets 0.0 and silently
        sizes a circle, a search radius or a detection window to nothing.

        Prefer :meth:`well_diameter_of` whenever a well NAME is in hand; this
        is the fallback for the genuinely plate-wide cases (a snap radius, a
        default FOV). Returns the largest well so a radius derived from it
        still *reaches* every well; returns 0.0 only for an empty plate.
        """
        if self.well_diameter > 0:
            return float(self.well_diameter)
        wells = list(self._wells.values())
        if not wells:
            return 0.0
        return float(max(w.diameter for w in wells))

    def well_diameter_of(self, well_name: str | None) -> float:
        """v7.12: diameter (mm) of *well_name*, or a representative one.

        The resolution chain — per-well → plate-level → largest well — used to
        be written out separately in four places (the jog workspace view, the
        mosaic mapping dialog, the sketch page and the plate view). Three
        copies agreeing the day they are written says nothing about the next
        edit, so it lives here once.

        Never raises: an unknown/None name falls through to
        :attr:`representative_well_diameter`.
        """
        if well_name:
            well = self._wells.get(str(well_name).upper())
            if well is not None and well.diameter > 0:
                return float(well.diameter)
        return self.representative_well_diameter

    def nearest_neighbour_pitch_mm(self) -> float:
        """v7.12: typical centre-to-centre spacing (mm) between wells.

        ``well_spacing_x``/``_y`` are 0.0 on a parametric plate, which turns
        every "is the needle within ~one well pitch" gate into "within 0" —
        i.e. never. This measures the pitch from the wells themselves as the
        MEDIAN nearest-neighbour distance, so a plate mixing a 40 mm grid with
        an 8 mm ring reports a sane middle value instead of being dominated by
        either extreme. Falls back to the declared spacing on a regular plate
        (byte-identical there) and to 0.0 for a plate with fewer than 2 wells.
        """
        if self.well_spacing_x > 0 and self.well_spacing_y > 0:
            return float(min(self.well_spacing_x, self.well_spacing_y))
        wells = list(self._wells.values())
        if len(wells) < 2:
            return 0.0
        nearest: list[float] = []
        for i, a in enumerate(wells):
            best = float("inf")
            for j, b in enumerate(wells):
                if i == j:
                    continue
                d = math.hypot(a.x - b.x, a.y - b.y)
                if d < best:
                    best = d
            if math.isfinite(best):
                nearest.append(best)
        if not nearest:
            return 0.0
        nearest.sort()
        return float(nearest[len(nearest) // 2])

    def extreme_well(self, dx: float, dy: float) -> str | None:
        """v7.12: name of the well farthest along direction *(dx, dy)*.

        A support function over the real well centres, so it works on any
        layout. On a regular plate ``(-1,-1)`` is A1, ``(+1,-1)`` the
        top-right, ``(+1,+1)`` the bottom-right and ``(-1,+1)`` the
        bottom-left — the same wells the old ``f"A{cols}"`` name arithmetic
        produced, but derived from wells that actually exist. Returns None for
        an empty plate.
        """
        wells = list(self._wells.values())
        if not wells:
            return None
        # Tie-break on (row, col) so the choice is deterministic when several
        # wells sit on the same extreme (e.g. a single-row plate).
        best = max(wells, key=lambda w: (dx * w.x + dy * w.y, -w.row, -w.col))
        return best.name

    def calibration_triangle(self) -> list[str]:
        """v7.12: three wells spanning the plate, for 3-point teaching.

        Replaces the ``["A1", f"A{cols}", f"{ROW_LABELS[rows-1]}{cols}"]`` name
        arithmetic, which asks for wells that do not exist on a parametric
        plate (``rows``/``cols`` there are a pseudo-grid where a whole ring of
        wells shares one cell) and raises ``KeyError`` from
        :meth:`get_well_position`.

        Returns the three corner wells of the layout's bounding box — on a
        standard plate exactly ``A1``, the top-right and the bottom-right, i.e.
        the same right triangle as before. Falls back to the fourth corner, and
        then to any remaining wells, if corners coincide on a degenerate
        layout; may return fewer than 3 names on a plate with fewer wells.
        """
        picks: list[str] = []
        for dx, dy in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
            name = self.extreme_well(dx, dy)
            if name is not None and name not in picks:
                picks.append(name)
            if len(picks) == 3:
                return picks
        for well in self.get_all_wells():
            if well.name not in picks:
                picks.append(well.name)
            if len(picks) == 3:
                break
        return picks

    def get_bounding_box(self) -> tuple[float, float, float, float]:
        """Plate bounding box relative to A1: (min_x, min_y, max_x, max_y)."""
        r = self._max_well_radius_mm()
        if self.well_spacing_x > 0 and self.well_spacing_y > 0:
            return (-r, -r, self.plate_width + r, self.plate_height + r)
        # Custom plate: walk wells.
        wells = list(self._wells.values())
        if not wells:
            return (-r, -r, r, r)
        xs = [w.x for w in wells]
        ys = [w.y for w in wells]
        return (min(xs) - r, min(ys) - r, max(xs) + r, max(ys) + r)

    # ── v7.3.1: Geometry-predicted positions ───────────────────────

    def get_all_positions_from_a1(
        self, a1_x_um: float, a1_y_um: float,
        plate_axis_sign: tuple[float, float] = (1.0, 1.0),
    ) -> dict[str, tuple[float, float]]:
        """Compute absolute stage positions (µm) for every well given A1's position.

        Uses plate geometry (well spacing) to predict all well centres.
        Coordinates are in the same frame as *a1_x_um / a1_y_um* (absolute stage µm).

        ``plate_axis_sign`` maps the plate-local axes (+col→+X, +row→+Y) onto the
        stage axes — pass ``StageController.plate_axis_sign()``. On a 180°-mounted
        stage (ME3B V1) it is ``(-1, -1)`` so prediction lands on the physically
        correct side of A1; the default ``(1, 1)`` is byte-identical to the
        legacy behaviour for callers that don't supply it.

        Returns:
            Dict mapping well name → (x_um, y_um).
        """
        sx, sy = plate_axis_sign
        positions: dict[str, tuple[float, float]] = {}
        for well in self.get_all_wells():
            # well.x / well.y are mm relative to A1 (plate-local frame)
            positions[well.name] = (
                a1_x_um + sx * well.x * 1000.0,
                a1_y_um + sy * well.y * 1000.0,
            )
        return positions

    def get_a1_from_plate_center(
        self, center_x_um: float, center_y_um: float,
        plate_axis_sign: tuple[float, float] = (1.0, 1.0),
    ) -> tuple[float, float]:
        """Compute A1 absolute position (µm) given the plate center position.

        Uses ANSI/SLAS footprint (127.76 × 85.48 mm) and the per-format
        A1 offset from the top-left corner of the plate.

        ``plate_axis_sign`` maps plate-local axes onto stage axes (see
        :meth:`get_all_positions_from_a1`); on a 180°-mounted stage the plate's
        "top-left" corner is on the opposite stage side, so the A1-offset term
        reflects with the sign.

        Args:
            center_x_um: Plate center X in µm (absolute stage coords).
            center_y_um: Plate center Y in µm (absolute stage coords).

        Returns:
            (a1_x_um, a1_y_um) — absolute stage position of well A1.
        """
        sx, sy = plate_axis_sign
        # A1 is offset from the plate's top-left corner by (a1_offset_x, a1_offset_y).
        # Plate center is at (footprint/2) from that corner.
        #
        # v7.12: use THIS plate's authored footprint, not the ANSI constants.
        # A parametric carrier need not be 127.76 x 85.48, and centring it by
        # the standard footprint puts it off by half the difference — visible
        # as "the default plate sits off-centre in the travel envelope".
        fw, fh = self.footprint_mm
        a1_x_um = center_x_um + sx * (self.a1_offset_x - fw / 2.0) * 1000.0
        a1_y_um = center_y_um + sy * (self.a1_offset_y - fh / 2.0) * 1000.0
        return (a1_x_um, a1_y_um)

    def get_all_positions_from_plate_center(
        self, center_x_um: float, center_y_um: float,
        plate_axis_sign: tuple[float, float] = (1.0, 1.0),
    ) -> dict[str, tuple[float, float]]:
        """Compute all well positions (µm) from the plate center position.

        Convenience method combining get_a1_from_plate_center + get_all_positions_from_a1.
        ``plate_axis_sign`` is threaded into both (see :meth:`get_all_positions_from_a1`).

        Args:
            center_x_um: Plate center X in µm (absolute stage coords).
            center_y_um: Plate center Y in µm (absolute stage coords).

        Returns:
            Dict mapping well name → (x_um, y_um).
        """
        a1_x, a1_y = self.get_a1_from_plate_center(
            center_x_um, center_y_um, plate_axis_sign)
        return self.get_all_positions_from_a1(a1_x, a1_y, plate_axis_sign)

    def get_well_area_bounds_um(
        self, center_x_um: float, center_y_um: float,
        plate_axis_sign: tuple[float, float] = (1.0, 1.0),
    ) -> tuple[float, float, float, float]:
        """Bounding box (µm) of the well area from plate center, for raster scanning.

        Returns the min/max stage coordinates that enclose all wells with
        a half-well-diameter margin, clamped to stage travel limits.
        ``plate_axis_sign`` maps plate-local axes onto stage axes (see
        :meth:`get_all_positions_from_a1`).

        Args:
            center_x_um: Plate center X in µm.
            center_y_um: Plate center Y in µm.

        Returns:
            (min_x_um, min_y_um, max_x_um, max_y_um)
        """
        a1_x, a1_y = self.get_a1_from_plate_center(
            center_x_um, center_y_um, plate_axis_sign)
        return self.get_well_area_bounds_from_a1_um(a1_x, a1_y, plate_axis_sign)

    def get_well_area_bounds_from_a1_um(
        self, a1_x_um: float, a1_y_um: float,
        plate_axis_sign: tuple[float, float] = (1.0, 1.0),
    ) -> tuple[float, float, float, float]:
        """Bounding box (µm) of the well area from A1 position.

        Returns the min/max stage coordinates that enclose all wells with
        a half-well-diameter margin, clamped to stage travel limits.
        ``plate_axis_sign`` maps plate-local axes onto stage axes; a negative
        sign mirrors the box, so min/max are re-normalised before clamping.

        Args:
            a1_x_um: Well A1 X in µm (absolute stage coords).
            a1_y_um: Well A1 Y in µm (absolute stage coords).

        Returns:
            (min_x_um, min_y_um, max_x_um, max_y_um)
        """
        sx, sy = plate_axis_sign

        # Use plate bounding box (handles both standard and custom plates).
        bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y = self.get_bounding_box()
        x_a = a1_x_um + sx * bbox_min_x * 1000.0
        x_b = a1_x_um + sx * bbox_max_x * 1000.0
        y_a = a1_y_um + sy * bbox_min_y * 1000.0
        y_b = a1_y_um + sy * bbox_max_y * 1000.0
        # A negative sign swaps which corner is min/max — re-normalise.
        min_x, max_x = (x_a, x_b) if x_a <= x_b else (x_b, x_a)
        min_y, max_y = (y_a, y_b) if y_a <= y_b else (y_b, y_a)

        # Clamp to stage travel limits (stage range: 0 to travel_mm * 1000)
        max_travel_x = STAGE_TRAVEL_X_MM * 1000.0
        max_travel_y = STAGE_TRAVEL_Y_MM * 1000.0
        min_x = max(min_x, 0.0)
        min_y = max(min_y, 0.0)
        max_x = min(max_x, max_travel_x)
        max_y = min(max_y, max_travel_y)

        return (min_x, min_y, max_x, max_y)

    def get_single_well_scan_bounds_um(
        self,
        well_x_um: float,
        well_y_um: float,
        margin_factor: float = 1.25,
    ) -> tuple[float, float, float, float]:
        """Bounding box (µm) for scanning a single well.

        Args:
            well_x_um: Well center X in µm (absolute stage coords).
            well_y_um: Well center Y in µm (absolute stage coords).
            margin_factor: Multiplier on well diameter for scan area
                          (1.25 = 25% margin around the well).

        Returns:
            (min_x_um, min_y_um, max_x_um, max_y_um)
        """
        half_span = self._max_well_radius_mm() * 1000.0 * margin_factor
        min_x = max(well_x_um - half_span, 0.0)
        min_y = max(well_y_um - half_span, 0.0)
        max_x = min(well_x_um + half_span, STAGE_TRAVEL_X_MM * 1000.0)
        max_y = min(well_y_um + half_span, STAGE_TRAVEL_Y_MM * 1000.0)
        return (min_x, min_y, max_x, max_y)

    def meander_order(self) -> list[str]:
        """Return well names in row-meander (serpentine) traversal order.

        Even rows (A, C, …) go left-to-right (col 1 → N),
        odd rows (B, D, …) go right-to-left (col N → 1).
        """
        order: list[str] = []
        for r in range(self.rows):
            cols = range(self.cols) if r % 2 == 0 else range(self.cols - 1, -1, -1)
            for c in cols:
                order.append(f"{ROW_LABELS[r]}{c + 1}")
        return order


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
