"""
WellSetup.py — Well plate assignment model for MEBP v7.1.

Backend data layer for Tab 3 (Well Setup). Manages:
- WellAssignment: per-well role, print collections, rosette, calibration
- Role behaviors: WashBehavior, WasteBehavior, BufferBehavior,
  InkPickupBehavior, SortedCellBehavior
- ServiceSequence: configurable waste→wash→buffer→ink workflow
- WellBottomDetector: 3-point teach + least-squares plane fitting
- WellSetupModel: full plate assignment state with save/load JSON

Dependencies:
- numpy (for plane fitting via lstsq)
- PhysicalModels (WellRole, ROLE_COLORS, InkSpec, RosetteInsert, WorkspaceConfig)
- WellPlate (plate geometry)

Session G — Tasks P5.25, P5.28, P5.29, P5.30, P5.33, P5.35.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any

import numpy as np

from SupportClasses.PhysicalModels import (
    WellRole, ROLE_COLORS, InkSpec, RosetteInsert, WorkspaceConfig,
)
from SupportClasses.WellPlate import WellPlate, WellInfo, ROW_LABELS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Well Role Behaviors (Automated Workflows)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class WashBehavior:
    """
    Wash well: needle moves randomly to scrub off debris.

    Sequence: travel to well → lower to wash_depth → random jiggle → raise.
    """
    wash_depth_mm: float = 2.0
    wash_duration_s: float = 5.0
    jiggle_radius_mm: float = 2.0
    jiggle_speed_mm_s: float = 5.0
    num_jiggle_points: int = 20

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> WashBehavior:
        return cls(**data)


@dataclass
class WasteBehavior:
    """
    Waste well: eject material to reset needle conditions.

    Sequence: travel to well → lower → push pump to eject → raise.
    """
    waste_depth_mm: float = 2.0
    eject_rate_uL_s: float = 2.0
    extra_push_uL: float = 5.0
    settle_time_s: float = 0.5

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> WasteBehavior:
        return cls(**data)


@dataclass
class BufferBehavior:
    """
    Buffer well: aspirate fresh buffer to separate oil from ink.

    Sequence: travel to well → lower → aspirate buffer → raise.
    """
    buffer_depth_mm: float = 2.0
    aspirate_volume_uL: float = 5.0
    aspirate_rate_uL_s: float = 0.5
    settle_time_s: float = 1.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> BufferBehavior:
        return cls(**data)


@dataclass
class InkPickupBehavior:
    """
    Ink well: aspirate ink for printing.

    Supports both incremental (per-well pickup) and continuous (fill syringe).
    """
    ink_depth_mm: float = 3.0
    pickup_volume_uL: float = 2.0       # Per-pickup for incremental mode
    fill_volume_uL: float = 0.0         # For continuous mode (0 = fill syringe)
    refill_threshold_uL: float = 5.0    # Refill when ink drops below this
    aspirate_rate_uL_s: float = 0.5
    settle_time_s: float = 1.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> InkPickupBehavior:
        return cls(**data)


@dataclass
class SortedCellBehavior:
    """
    Sorted cell well: deposit picked/sorted cells.

    Sequence: travel to well → lower → eject deposit volume → settle → raise.
    """
    deposit_depth_mm: float = 1.0
    deposit_volume_uL: float = 1.0
    eject_rate_uL_s: float = 0.5
    settle_time_s: float = 1.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> SortedCellBehavior:
        return cls(**data)


# Behavior factory: creates default behavior for a given role
_BEHAVIOR_FACTORIES: dict[WellRole, type] = {
    WellRole.WASH: WashBehavior,
    WellRole.WASTE: WasteBehavior,
    WellRole.BUFFER: BufferBehavior,
    WellRole.INK: InkPickupBehavior,
    WellRole.SORTED_CELLS: SortedCellBehavior,
}

_BEHAVIOR_DESERIALIZERS: dict[str, type] = {
    "wash_behavior": WashBehavior,
    "waste_behavior": WasteBehavior,
    "buffer_behavior": BufferBehavior,
    "ink_pickup": InkPickupBehavior,
    "sorted_deposit": SortedCellBehavior,
}

# Maps WellRole to the single behavior attribute that is meaningful
# for that role. Roles not listed (PRINT, EMPTY) have no behavior.
_ROLE_TO_BEHAVIOR_KEY: dict[WellRole, str] = {
    WellRole.WASH: "wash_behavior",
    WellRole.WASTE: "waste_behavior",
    WellRole.BUFFER: "buffer_behavior",
    WellRole.INK: "ink_pickup",
    WellRole.SORTED_CELLS: "sorted_deposit",
}


def default_behavior_for_role(role: WellRole):
    """Create default behavior object for a role, or None if role has none."""
    factory = _BEHAVIOR_FACTORIES.get(role)
    return factory() if factory else None


# ═══════════════════════════════════════════════════════════════════
# Service Sequence
# ═══════════════════════════════════════════════════════════════════

@dataclass
class ServiceSequence:
    """
    Configurable sequence of service operations between prints.

    Each step references a well role; the planner finds the nearest
    well (or rosette sub-well) with that role.

    Default full ink-change: waste → wash → buffer → ink
    Continuous same-ink top-up: just ["ink"]
    Full with prime: ["waste", "wash", "buffer", "ink", "waste"]
    """
    steps: list[str] = field(default_factory=lambda: [
        "waste", "wash", "buffer", "ink"
    ])

    # Named presets
    PRESETS: dict[str, list[str]] = field(default=None, repr=False, init=False)

    def __post_init__(self):
        self.PRESETS = {
            "Full Ink Change": ["waste", "wash", "buffer", "ink"],
            "Quick Ink Change": ["waste", "ink"],
            "Same Ink Top-up": ["ink"],
            "Full with Prime": ["waste", "wash", "buffer", "ink", "waste"],
            "Wash Only": ["wash"],
            "No Service": [],
        }

    @classmethod
    def from_preset(cls, preset_name: str) -> ServiceSequence:
        """Create a service sequence from a named preset."""
        seq = cls()
        if preset_name in seq.PRESETS:
            seq.steps = list(seq.PRESETS[preset_name])
        else:
            logger.warning(f"Unknown preset '{preset_name}', using default")
        return seq

    def to_dict(self) -> dict:
        return {"steps": list(self.steps)}

    @classmethod
    def from_dict(cls, data: dict) -> ServiceSequence:
        return cls(steps=data.get("steps", ["waste", "wash", "buffer", "ink"]))


# ═══════════════════════════════════════════════════════════════════
# Well Assignment
# ═══════════════════════════════════════════════════════════════════

@dataclass
class WellAssignment:
    """
    Complete assignment for a single well on the plate.

    Tracks role, ink, print collections, rosette insert, sub-well roles,
    calibration data, and role-specific behavior parameters.
    """
    well_name: str
    role: WellRole = WellRole.EMPTY
    role_index: int = 0                 # For numbered roles: Ink 1, Ink 2, Sorted 1…

    # Ink assignment (for INK wells)
    ink_name: str | None = None         # Key into workspace ink_library

    # Print assignment (for PRINT wells)
    print_collections: list[str] = field(default_factory=list)
    print_offsets: list[tuple[float, float, float]] = field(default_factory=list)

    # Rosette insert (optional — for any role)
    rosette_name: str | None = None     # Key into workspace rosette_library
    subwell_roles: list[str] = field(default_factory=list)
    subwell_inks: list[str | None] = field(default_factory=list)
    subwell_labels: list[str] = field(default_factory=list)

    # Behaviors (populated based on role)
    wash_behavior: WashBehavior | None = None
    waste_behavior: WasteBehavior | None = None
    buffer_behavior: BufferBehavior | None = None
    ink_pickup: InkPickupBehavior | None = None
    sorted_deposit: SortedCellBehavior | None = None

    # Calibration
    z_offset: float = 0.0              # From plane fit (mm)
    manually_taught_z: float | None = None  # Override from manual teach

    # Display
    color: str = "#585b70"

    def set_role(self, role: WellRole, role_index: int = 0) -> None:
        """
        Change well role, resetting role-specific fields and creating
        default behavior for the new role.
        """
        self.role = role
        self.role_index = role_index
        self.color = ROLE_COLORS.get(role, "#585b70")

        # Clear all behaviors
        self.wash_behavior = None
        self.waste_behavior = None
        self.buffer_behavior = None
        self.ink_pickup = None
        self.sorted_deposit = None

        # Create default behavior for new role
        if role == WellRole.WASH:
            self.wash_behavior = WashBehavior()
        elif role == WellRole.WASTE:
            self.waste_behavior = WasteBehavior()
        elif role == WellRole.BUFFER:
            self.buffer_behavior = BufferBehavior()
        elif role == WellRole.INK:
            self.ink_pickup = InkPickupBehavior()
        elif role == WellRole.SORTED_CELLS:
            self.sorted_deposit = SortedCellBehavior()

        # Clear print list if not a print well
        if role != WellRole.PRINT:
            self.print_collections.clear()
            self.print_offsets.clear()

        # Clear ink if not an ink well
        if role != WellRole.INK:
            self.ink_name = None

    def get_effective_z(self) -> float:
        """Z offset: manual teach overrides plane fit."""
        if self.manually_taught_z is not None:
            return self.manually_taught_z
        return self.z_offset

    def get_display_label(self) -> str:
        """Short display label for the well assignment summary."""
        if self.role == WellRole.EMPTY:
            return ""
        elif self.role == WellRole.PRINT:
            if self.print_collections:
                return ", ".join(self.print_collections[:2])
            return "Print"
        elif self.role == WellRole.INK:
            idx = f" {self.role_index}" if self.role_index > 0 else ""
            name = f": {self.ink_name}" if self.ink_name else ""
            return f"Ink{idx}{name}"
        elif self.role in (WellRole.WASH, WellRole.WASTE, WellRole.BUFFER):
            return self.role.value.capitalize()
        elif self.role == WellRole.SORTED_CELLS:
            idx = f" {self.role_index}" if self.role_index > 0 else ""
            return f"Sorted{idx}"
        return self.role.value

    def add_print(self, collection_name: str,
                  offset: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """Append a print collection to this well's print list."""
        self.print_collections.append(collection_name)
        self.print_offsets.append(offset)

    def remove_print(self, index: int) -> None:
        """Remove a print collection by index."""
        if 0 <= index < len(self.print_collections):
            self.print_collections.pop(index)
            self.print_offsets.pop(index)

    def clear_prints(self) -> None:
        """Remove all print assignments."""
        self.print_collections.clear()
        self.print_offsets.clear()

    def reorder_print(self, from_idx: int, to_idx: int) -> None:
        """Move a print collection from one position to another."""
        if (0 <= from_idx < len(self.print_collections)
                and 0 <= to_idx < len(self.print_collections)):
            item = self.print_collections.pop(from_idx)
            offset = self.print_offsets.pop(from_idx)
            self.print_collections.insert(to_idx, item)
            self.print_offsets.insert(to_idx, offset)

    def to_dict(self) -> dict:
        """Serialize to dict for JSON persistence."""
        d: dict[str, Any] = {
            "well_name": self.well_name,
            "role": self.role.value,
            "role_index": self.role_index,
            "ink_name": self.ink_name,
            "print_collections": list(self.print_collections),
            "print_offsets": [list(o) for o in self.print_offsets],
            "rosette_name": self.rosette_name,
            "subwell_roles": list(self.subwell_roles),
            "subwell_inks": list(self.subwell_inks),
            "subwell_labels": list(self.subwell_labels),
            "z_offset": self.z_offset,
            "manually_taught_z": self.manually_taught_z,
            "color": self.color,
        }
        # Serialize only the behavior relevant to the assigned role.
        # Older versions persisted all five behavior slots even when most
        # were nonsensical for the role (e.g. wash_behavior on a PRINT
        # well). Reading those old saves still works — from_dict ignores
        # behaviors that don't match the active role.
        role_key = _ROLE_TO_BEHAVIOR_KEY.get(self.role)
        if role_key is not None:
            beh = getattr(self, role_key)
            d[role_key] = beh.to_dict() if beh else None
        return d

    @classmethod
    def from_dict(cls, data: dict) -> WellAssignment:
        """Deserialize from dict."""
        wa = cls(well_name=data["well_name"])
        wa.role = WellRole(data.get("role", "empty"))
        wa.role_index = data.get("role_index", 0)
        wa.ink_name = data.get("ink_name")
        wa.print_collections = data.get("print_collections", [])
        wa.print_offsets = [tuple(o) for o in data.get("print_offsets", [])]
        wa.rosette_name = data.get("rosette_name")
        wa.subwell_roles = data.get("subwell_roles", [])
        wa.subwell_inks = data.get("subwell_inks", [])
        wa.subwell_labels = data.get("subwell_labels", [])
        wa.z_offset = data.get("z_offset", 0.0)
        wa.manually_taught_z = data.get("manually_taught_z")
        wa.color = data.get("color", ROLE_COLORS.get(wa.role, "#585b70"))

        # Deserialize behaviors
        for key, klass in _BEHAVIOR_DESERIALIZERS.items():
            beh_data = data.get(key)
            if beh_data is not None:
                setattr(wa, key, klass.from_dict(beh_data))
        return wa


# ═══════════════════════════════════════════════════════════════════
# Well Bottom Plane Detection
# ═══════════════════════════════════════════════════════════════════

@dataclass
class TeachPoint:
    """One calibration point: well name + measured Z height."""
    well_name: str
    x_mm: float
    y_mm: float
    z_mm: float


@dataclass
class PlaneResult:
    """Result of least-squares plane fit: z = a*x + b*y + c."""
    a: float          # dz/dx slope
    b: float          # dz/dy slope
    c: float          # z-intercept
    r_squared: float  # Goodness of fit (1.0 = perfect plane)
    num_points: int

    def z_at(self, x: float, y: float) -> float:
        """Evaluate fitted plane at (x, y)."""
        return self.a * x + self.b * y + self.c

    def describe(self) -> str:
        """Human-readable plane equation."""
        return (
            f"z = {self.c:+.4f} {self.a:+.6f}·x {self.b:+.6f}·y  "
            f"(R²={self.r_squared:.4f}, {self.num_points} pts)"
        )


class WellBottomDetector:
    """
    Detects the well-bottom plane via 3+ taught points.

    User jogs needle to glass surface in each well, records the Z position.
    With 3+ points, fits z = a*x + b*y + c via numpy least squares.
    The fitted plane provides per-well Z offsets for the entire plate.

    Usage:
        detector = WellBottomDetector(plate)
        detector.add_point("A1", z=-0.12)
        detector.add_point("D6", z=-0.08)
        detector.add_point("H12", z=-0.15)
        result = detector.fit_plane()
        z_offset = result.z_at(well_x, well_y)
    """

    def __init__(self, plate: WellPlate):
        self._plate = plate
        self._points: list[TeachPoint] = []
        self._result: PlaneResult | None = None

    @property
    def points(self) -> list[TeachPoint]:
        return list(self._points)

    @property
    def num_points(self) -> int:
        return len(self._points)

    @property
    def result(self) -> PlaneResult | None:
        return self._result

    @property
    def can_fit(self) -> bool:
        """Need at least 3 non-collinear points to define a plane."""
        return len(self._points) >= 3

    def add_point(self, well_name: str, z: float) -> TeachPoint:
        """
        Record a teach point at the given well.

        If a point already exists for this well, it is replaced.
        """
        x, y = self._plate.get_well_position(well_name)

        # Replace existing point for same well
        self._points = [p for p in self._points if p.well_name != well_name]

        pt = TeachPoint(well_name=well_name, x_mm=x, y_mm=y, z_mm=z)
        self._points.append(pt)
        self._result = None  # Invalidate cached fit
        logger.info(f"Teach point: {well_name} at ({x:.2f}, {y:.2f}) z={z:.4f}")
        return pt

    def remove_point(self, well_name: str) -> bool:
        """Remove a teach point. Returns True if found."""
        before = len(self._points)
        self._points = [p for p in self._points if p.well_name != well_name]
        self._result = None
        return len(self._points) < before

    def clear(self) -> None:
        """Remove all teach points."""
        self._points.clear()
        self._result = None

    def fit_plane(self) -> PlaneResult | None:
        """
        Fit z = a*x + b*y + c using numpy least squares.

        Returns PlaneResult or None if fewer than 3 points.
        """
        if not self.can_fit:
            logger.warning(
                f"Need ≥3 points for plane fit, have {len(self._points)}")
            return None

        xs = np.array([p.x_mm for p in self._points])
        ys = np.array([p.y_mm for p in self._points])
        zs = np.array([p.z_mm for p in self._points])

        # Design matrix: [x, y, 1]
        A = np.column_stack([xs, ys, np.ones_like(xs)])

        # Least squares: solve A @ [a, b, c]^T = z
        result, residuals, rank, sv = np.linalg.lstsq(A, zs, rcond=None)
        a, b, c = result

        # R-squared
        z_pred = A @ result
        ss_res = np.sum((zs - z_pred) ** 2)
        ss_tot = np.sum((zs - np.mean(zs)) ** 2)
        r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 1.0

        self._result = PlaneResult(
            a=float(a),
            b=float(b),
            c=float(c),
            r_squared=float(r_squared),
            num_points=len(self._points),
        )
        logger.info(f"Plane fit: {self._result.describe()}")
        return self._result

    def get_z_offset(self, well_name: str) -> float:
        """
        Get the Z offset for a specific well from the fitted plane.

        Falls back to 0.0 if plane not yet fitted.
        """
        if self._result is None:
            return 0.0
        x, y = self._plate.get_well_position(well_name)
        return self._result.z_at(x, y)

    def get_all_offsets(self) -> dict[str, float]:
        """Get Z offsets for every well on the plate."""
        offsets = {}
        for well in self._plate.get_all_wells():
            offsets[well.name] = self.get_z_offset(well.name)
        return offsets

    def to_dict(self) -> dict:
        return {
            "points": [
                {"well_name": p.well_name, "z_mm": p.z_mm}
                for p in self._points
            ],
            "result": {
                "a": self._result.a,
                "b": self._result.b,
                "c": self._result.c,
                "r_squared": self._result.r_squared,
                "num_points": self._result.num_points,
            } if self._result else None,
        }

    def load_dict(self, data: dict) -> None:
        """Restore teach points from dict (re-fits plane if enough points)."""
        self.clear()
        for pt in data.get("points", []):
            self.add_point(pt["well_name"], pt["z_mm"])
        if self.can_fit:
            self.fit_plane()


# ═══════════════════════════════════════════════════════════════════
# Auto-Assign Patterns
# ═══════════════════════════════════════════════════════════════════

def auto_assign_block(
    plate: WellPlate,
    start_well: str,
    end_well: str,
    role: WellRole,
    role_index: int = 0,
) -> list[str]:
    """
    Select all wells within a rectangular block (inclusive).

    Returns list of well names in the block.
    Example: auto_assign_block(plate, "B2", "D5", WellRole.PRINT)
    """
    start = plate.get_well_info(start_well)
    end = plate.get_well_info(end_well)
    r_min, r_max = sorted([start.row, end.row])
    c_min, c_max = sorted([start.col, end.col])

    names = []
    for well in plate.get_all_wells():
        if r_min <= well.row <= r_max and c_min <= well.col <= c_max:
            names.append(well.name)
    return names


def auto_assign_checkerboard(
    plate: WellPlate,
    role_a: WellRole,
    role_b: WellRole,
    start_with_a: bool = True,
) -> dict[str, WellRole]:
    """
    Assign alternating roles in a checkerboard pattern.

    Returns {well_name: WellRole} mapping.
    """
    assignments: dict[str, WellRole] = {}
    for well in plate.get_all_wells():
        is_even = (well.row + well.col) % 2 == 0
        if start_with_a:
            assignments[well.name] = role_a if is_even else role_b
        else:
            assignments[well.name] = role_b if is_even else role_a
    return assignments


def auto_assign_border(
    plate: WellPlate,
    border_role: WellRole,
    inner_role: WellRole,
) -> dict[str, WellRole]:
    """
    Assign border wells one role, inner wells another.

    Useful for: border = service wells, inner = print wells.
    """
    assignments: dict[str, WellRole] = {}
    max_row = plate.rows - 1
    max_col = plate.cols - 1
    for well in plate.get_all_wells():
        is_border = (
            well.row == 0 or well.row == max_row
            or well.col == 0 or well.col == max_col
        )
        assignments[well.name] = border_role if is_border else inner_role
    return assignments


# ═══════════════════════════════════════════════════════════════════
# Well Setup Model — Full plate assignment state
# ═══════════════════════════════════════════════════════════════════

class WellSetupModel:
    """
    Complete well plate assignment state.

    Owns the WellPlate geometry, per-well assignments dict,
    WellBottomDetector, and ServiceSequence configuration.

    This is the single source of truth for Tab 3 state and is passed
    to the print planner when executing a job.
    """

    def __init__(self, plate_key: int | str = 24):
        """v7.4.5: accepts an int (standard format) or a str (custom plate name)."""
        self._plate = WellPlate.load(plate_key)
        self._assignments: dict[str, WellAssignment] = {}
        self._detector = WellBottomDetector(self._plate)
        self._service_sequence = ServiceSequence()

        # Initialize empty assignment for every well
        for well in self._plate.get_all_wells():
            self._assignments[well.name] = WellAssignment(
                well_name=well.name,
                color=ROLE_COLORS[WellRole.EMPTY],
            )

    # ── Properties ────────────────────────────────────────────────

    @property
    def plate(self) -> WellPlate:
        return self._plate

    @property
    def plate_format(self) -> int | str:
        """v7.4.5: returns the WellPlate.format (int for standards, "custom:<name>" for custom)."""
        return self._plate.format

    @property
    def assignments(self) -> dict[str, WellAssignment]:
        return self._assignments

    @property
    def detector(self) -> WellBottomDetector:
        return self._detector

    @property
    def service_sequence(self) -> ServiceSequence:
        return self._service_sequence

    @service_sequence.setter
    def service_sequence(self, seq: ServiceSequence) -> None:
        self._service_sequence = seq

    # ── Plate Format ──────────────────────────────────────────────

    def set_plate_format(self, key: int | str) -> None:
        """
        Change plate. Resets all assignments and teach points.

        v7.4.5: `key` may be a standard format int (6/12/24/48/96/384)
        or a custom plate name (resolves via `WellPlate.load`).
        """
        self._plate = WellPlate.load(key)
        self._assignments.clear()
        for well in self._plate.get_all_wells():
            self._assignments[well.name] = WellAssignment(
                well_name=well.name,
                color=ROLE_COLORS[WellRole.EMPTY],
            )
        self._detector = WellBottomDetector(self._plate)
        logger.info(f"Plate changed to {key}, all assignments reset")

    # ── Assignment Operations ─────────────────────────────────────

    def get_assignment(self, well_name: str) -> WellAssignment:
        """Get assignment for a specific well."""
        return self._assignments[well_name]

    def set_role(
        self,
        well_names: list[str],
        role: WellRole,
        role_index: int = 0,
    ) -> None:
        """Set role for one or more wells."""
        for name in well_names:
            if name in self._assignments:
                self._assignments[name].set_role(role, role_index)

    def set_ink(
        self,
        well_names: list[str],
        ink_name: str | None,
    ) -> None:
        """Set ink assignment for INK wells."""
        for name in well_names:
            wa = self._assignments.get(name)
            if wa and wa.role == WellRole.INK:
                wa.ink_name = ink_name

    def assign_print(
        self,
        well_names: list[str],
        collection_name: str,
        offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
        replace: bool = False,
    ) -> None:
        """
        Add a print collection to wells (append or replace).
        Automatically sets role to PRINT if not already.
        """
        for name in well_names:
            wa = self._assignments.get(name)
            if wa is None:
                continue
            if wa.role != WellRole.PRINT:
                wa.set_role(WellRole.PRINT)
            if replace:
                wa.clear_prints()
            wa.add_print(collection_name, offset)

    def clear_assignments(self, well_names: list[str]) -> None:
        """Reset wells to EMPTY."""
        self.set_role(well_names, WellRole.EMPTY)

    def attach_rosette(
        self,
        well_names: list[str],
        rosette_name: str | None,
        workspace: WorkspaceConfig | None = None,
    ) -> None:
        """Attach or detach a rosette insert to wells."""
        for name in well_names:
            wa = self._assignments.get(name)
            if wa is None:
                continue
            wa.rosette_name = rosette_name
            if rosette_name and workspace:
                rosette = workspace.get_rosette(rosette_name)
                if rosette:
                    # Initialize sub-well role/ink lists
                    n = rosette.num_subwells
                    wa.subwell_roles = ["empty"] * n
                    wa.subwell_inks = [None] * n
                    wa.subwell_labels = [f"SW{i}" for i in range(n)]
            elif rosette_name is None:
                wa.subwell_roles.clear()
                wa.subwell_inks.clear()
                wa.subwell_labels.clear()

    # ── Queries ───────────────────────────────────────────────────

    def get_wells_by_role(self, role: WellRole) -> list[str]:
        """Get all well names with a given role."""
        return [
            name for name, wa in self._assignments.items()
            if wa.role == role
        ]

    def get_print_wells(self) -> list[str]:
        """All wells assigned for printing."""
        return self.get_wells_by_role(WellRole.PRINT)

    def get_ink_wells(self, ink_name: str | None = None) -> list[str]:
        """Ink wells, optionally filtered by ink name."""
        wells = self.get_wells_by_role(WellRole.INK)
        if ink_name:
            wells = [
                w for w in wells
                if self._assignments[w].ink_name == ink_name
            ]
        return wells

    def get_nearest_service_well(
        self,
        role: WellRole,
        from_xy: tuple[float, float],
    ) -> str | None:
        """
        Find the nearest well of a given role to a position.

        Used by the planner to find wash/waste/buffer/ink wells.
        """
        wells = self.get_wells_by_role(role)
        if not wells:
            return None

        best_name = None
        best_dist = float("inf")
        for name in wells:
            info = self._plate.get_well_info(name)
            dx = info.x - from_xy[0]
            dy = info.y - from_xy[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < best_dist:
                best_dist = dist
                best_name = name
        return best_name

    def get_assignment_summary(self) -> list[dict]:
        """
        Generate summary table data for all non-empty wells.

        Returns list of dicts suitable for QTableWidget population.
        """
        summary = []
        for well in self._plate.get_all_wells():
            wa = self._assignments[well.name]
            if wa.role == WellRole.EMPTY:
                continue
            summary.append({
                "well": well.name,
                "role": wa.role.value.capitalize(),
                "insert": wa.rosette_name or "—",
                "prints": ", ".join(wa.print_collections) if wa.print_collections else "—",
                "z_offset": f"{wa.get_effective_z():+.3f}",
                "color": wa.color,
            })
        return summary

    def apply_auto_pattern(
        self,
        pattern_result: dict[str, WellRole],
    ) -> None:
        """Apply result from auto_assign_* functions."""
        for name, role in pattern_result.items():
            if name in self._assignments:
                self._assignments[name].set_role(role)

    # ── Plane Fitting ─────────────────────────────────────────────

    def teach_well_z(self, well_name: str, z_mm: float) -> None:
        """Record a teach point for plane fitting."""
        self._detector.add_point(well_name, z_mm)

    def fit_plane(self) -> PlaneResult | None:
        """Fit well-bottom plane and apply offsets to all wells."""
        result = self._detector.fit_plane()
        if result:
            offsets = self._detector.get_all_offsets()
            for name, z in offsets.items():
                self._assignments[name].z_offset = z
        return result

    # ── Serialization ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Full state to dict for JSON persistence."""
        return {
            "plate_format": self._plate.format,
            "assignments": {
                name: wa.to_dict()
                for name, wa in self._assignments.items()
                if wa.role != WellRole.EMPTY  # Skip empties to save space
            },
            "detector": self._detector.to_dict(),
            "service_sequence": self._service_sequence.to_dict(),
        }

    def load_dict(self, data: dict) -> None:
        """Restore state from dict (plate format must match)."""
        fmt = data.get("plate_format", self._plate.format)
        if fmt != self._plate.format:
            self.set_plate_format(fmt)

        for name, wa_data in data.get("assignments", {}).items():
            if name in self._assignments:
                self._assignments[name] = WellAssignment.from_dict(wa_data)

        if data.get("detector"):
            self._detector.load_dict(data["detector"])
            # Re-apply plane offsets
            if self._detector.result:
                offsets = self._detector.get_all_offsets()
                for name, z in offsets.items():
                    self._assignments[name].z_offset = z

        if data.get("service_sequence"):
            self._service_sequence = ServiceSequence.from_dict(
                data["service_sequence"])

    def save_json(self, filepath: str | Path) -> None:
        """Save full well setup to JSON file."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        logger.info(f"Well setup saved to {filepath}")

    @classmethod
    def load_json(cls, filepath: str | Path) -> WellSetupModel:
        """Load well setup from JSON file."""
        with open(filepath) as f:
            data = json.load(f)
        model = cls(plate_format=data.get("plate_format", 24))
        model.load_dict(data)
        logger.info(f"Well setup loaded from {filepath}")
        return model

    def validate(self, workspace: WorkspaceConfig | None = None) -> list[str]:
        """
        Validate the well setup for completeness.

        Returns list of warning/error strings. Empty = valid.
        """
        issues: list[str] = []
        print_wells = self.get_print_wells()

        if not print_wells:
            issues.append("No wells assigned for printing")

        # Check print wells have collections
        for name in print_wells:
            wa = self._assignments[name]
            if not wa.print_collections:
                issues.append(f"{name}: Print well has no print collections assigned")

        # Check ink wells if any pump uses incremental mode
        ink_wells = self.get_wells_by_role(WellRole.INK)
        if workspace:
            for pump in workspace.get_active_pumps():
                if pump.printing_mode.value == "incremental" and not ink_wells:
                    issues.append(
                        f"{pump.pump_id}: Incremental mode but no ink wells assigned")

        # Check service wells
        needed_roles = set()
        for step in self._service_sequence.steps:
            needed_roles.add(step)
        for role_str in needed_roles:
            try:
                role = WellRole(role_str)
                if not self.get_wells_by_role(role):
                    issues.append(
                        f"Service sequence requires '{role_str}' well but none assigned")
            except ValueError:
                issues.append(f"Unknown role in service sequence: '{role_str}'")

        # Check plane calibration
        if not self._detector.result:
            issues.append("Well bottom plane not calibrated (teach ≥3 wells)")

        return issues
