"""
PhysicalModels.py — Physical hardware models and data layer for MEBP v7.3.

Dataclasses representing:
- NeedleSpec: Needle gauge specifications (loaded from JSON)
- SyringeSpec: Hamilton syringe specifications (loaded from JSON)
- InkSpec: Printing material properties
- FluidColumn: Oil/buffer/ink layer tracking per syringe
- PumpLoadout: Complete pump channel configuration
- PrintingMode: Incremental vs continuous dispensing
- RosetteSubWell / RosetteInsert: Well insert geometry
- CameraSpec: Microscope camera sensor specifications (v7.3.0)
- WorkspaceConfig: Complete session configuration

All needle/syringe catalogs are loaded from user-editable JSON files
in config/hardware/ so users can add new sizes without modifying Python.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default config paths (relative to application root)
DEFAULT_NEEDLES_JSON = "config/hardware/needles.json"
DEFAULT_SYRINGES_JSON = "config/hardware/syringes.json"
DEFAULT_CAMERAS_JSON = "config/hardware/cameras.json"


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class PrintingMode(Enum):
    """How a pump delivers ink during a print run."""
    INCREMENTAL = "incremental"
    # Pick up a small volume of ink before each well/object.
    # Workflow: aspirate ink → print → (optionally waste/wash) → repeat
    # Pros: Fresh ink each time, less waste
    # Cons: Slower, more needle travel

    CONTINUOUS = "continuous"
    # Fill the entire syringe (or a large portion) with ink upfront.
    # Workflow: fill syringe → print many wells → refill when empty
    # Pros: Faster, fewer interruptions
    # Cons: Ink sits in syringe longer, may settle


class WellRole(Enum):
    """Purpose assigned to a well on the plate."""
    EMPTY = "empty"                 # No assignment
    PRINT = "print"                 # Active printing target
    INK = "ink"                     # Ink reservoir — aspirate ink from here
    WASH = "wash"                   # Wash station — jiggle needle to clean
    WASTE = "waste"                 # Waste deposit — dispense buffer+old ink
    BUFFER = "buffer"               # Buffer pickup — aspirate buffer material
    SORTED_CELLS = "sorted"         # Deposit sorted/picked cells


# Well role display colors (Catppuccin Mocha palette)
ROLE_COLORS: dict[WellRole, str] = {
    WellRole.EMPTY:        "#585b70",   # Gray (surface2)
    WellRole.PRINT:        "#a6e3a1",   # Green
    WellRole.INK:          "#89b4fa",   # Blue
    WellRole.WASH:         "#f9e2af",   # Yellow
    WellRole.WASTE:        "#f38ba8",   # Red
    WellRole.BUFFER:       "#cba6f7",   # Mauve/Purple
    WellRole.SORTED_CELLS: "#fab387",   # Peach/Orange
}


# ---------------------------------------------------------------------------
# Reagent taxonomy — well type vs ink subtype (v7.5.x)
# ---------------------------------------------------------------------------
#
# A reagent's PRIMARY classification is its WELL TYPE, stored in
# ``InkSpec.ink_type``. This single value drives ALL behavior: service-well
# resolution (``_reagent_prep.service_well_names`` and the ``__waste__`` /
# ``__oil__`` / ``__wash__`` / ``__buffer__`` keys in ``PickAndPlaceManager``),
# the role mapping (:func:`well_role_for_ink_type`), and the reagent-location
# colors (:func:`ink_type_border_color`). The four SERVICE well types are never
# printable inks (a pump never aspirates from them).
#
# A printable ink may ALSO carry an INK SUBTYPE (``InkSpec.ink_subtype``) — an
# INFORMATIONAL material category, meaningful only when the well type is
# ``ink``. It changes no motion/pump behavior; it just tells the operator what
# the ink is. The list is editable (operators may add their own).

WELL_TYPES: tuple[str, ...] = ("ink", "wash", "buffer", "waste", "oil")

# The non-printable service well types. The pump ink list and the printable-ink
# filters exclude these (by well type OR by an ink literally named one of them).
SERVICE_WELL_TYPES: tuple[str, ...] = ("waste", "wash", "buffer", "oil")

# Standard informational subtypes offered for the ``ink`` well type.
INK_SUBTYPES: tuple[str, ...] = (
    "granular material",
    "fluorescent stains",
    "cells",
    "hydrogel monomer",
    "media",
    "cell removal reagents",
    "ELISA Beads",
    "Growth Factor Beads",
)

# Pre-v7.5.x stored the material category in ``ink_type`` itself. The
# type/subtype split moves it to ``ink_subtype`` and sets the well type to the
# printable ``ink``. Maps legacy ``ink_type`` → the new subtype.
_LEGACY_MATERIAL_SUBTYPE: dict[str, str] = {
    "granular": "granular material",
    "cells": "cells",
    "hydrogel": "hydrogel monomer",
    "media": "media",
    "custom": "",
}


def is_service_reagent(name: str | None, ink_type: str | None) -> bool:
    """True when this reagent is a SERVICE reagent (wash/waste/buffer/oil).

    Matched by the well type OR by an ink literally named one of the service
    roles (mirrors the workflow-page printable-ink filters). Case-insensitive.
    """
    t = (ink_type or "").strip().lower()
    n = (name or "").strip().lower()
    return t in SERVICE_WELL_TYPES or n in SERVICE_WELL_TYPES


def is_printable_ink_type(ink_type: str | None) -> bool:
    """True when this WELL TYPE is a printable ink (not a service reagent)."""
    return (ink_type or "").strip().lower() not in SERVICE_WELL_TYPES


def split_legacy_ink_type(
    ink_type: str | None, ink_subtype: str | None = "",
) -> tuple[str, str]:
    """Normalize a possibly-legacy ``(ink_type, ink_subtype)`` pair to v7.5.x.

    Legacy material well types (``granular``/``cells``/``hydrogel``/``media``/
    ``custom``) become ``("ink", <subtype>)`` — keeping an explicit subtype if
    one is already present, else the mapped default. Valid well types
    (``ink``/``wash``/``buffer``/``waste``/``oil``) pass through unchanged; an
    empty/unknown type defaults to ``ink``.
    """
    t = (ink_type or "").strip().lower()
    sub = (ink_subtype or "").strip()
    if t in _LEGACY_MATERIAL_SUBTYPE:
        return "ink", (sub or _LEGACY_MATERIAL_SUBTYPE[t])
    return (t or "ink"), sub


def well_role_for_ink_type(ink_type: str | None) -> WellRole:
    """Map an :class:`InkSpec` ``ink_type`` to a :class:`WellRole`.

    v7.5.x: reagent locations reuse the ink's material/purpose ``ink_type``
    (there is no separate role enum on the ink). Service types map to their
    dedicated roles; every other type — including ``oil`` and the material
    types (``hydrogel``/``cells``/``media``/``granular``/``custom``/``ink``) —
    is treated as an INK reservoir (a place the needle aspirates from).
    """
    t = (ink_type or "").strip().lower()
    if t == "wash":
        return WellRole.WASH
    if t == "waste":
        return WellRole.WASTE
    if t == "buffer":
        return WellRole.BUFFER
    return WellRole.INK


# Outline colors for reagent locations, keyed by InkSpec.ink_type.
# The well *fill* is the reagent's own InkSpec.color; this map gives the
# *border*, so the operator reads the functional type (ink/wash/waste/buffer/
# oil) at a glance while the fill reads the specific reagent. Oil has no
# WellRole of its own (well_role_for_ink_type maps it to INK), so it gets a
# dedicated border color here to stay visually distinct.
INK_TYPE_COLORS: dict[str, str] = {
    "ink":    "#89b4fa",   # Blue   (matches WellRole.INK)
    "wash":   "#f9e2af",   # Yellow (matches WellRole.WASH)
    "waste":  "#f38ba8",   # Red    (matches WellRole.WASTE)
    "buffer": "#cba6f7",   # Mauve  (matches WellRole.BUFFER)
    "oil":    "#fab387",   # Peach  (oil-specific)
}


def ink_type_border_color(ink_type: str | None) -> str:
    """Outline color for a reagent location, keyed by its ``ink_type``.

    Service + oil types get distinct colors (see :data:`INK_TYPE_COLORS`);
    every other (material) type falls back to the INK role color via
    :func:`well_role_for_ink_type`.
    """
    t = (ink_type or "").strip().lower()
    if t in INK_TYPE_COLORS:
        return INK_TYPE_COLORS[t]
    return ROLE_COLORS.get(well_role_for_ink_type(t), "#585b70")


# ---------------------------------------------------------------------------
# Needle Specification
# ---------------------------------------------------------------------------

@dataclass
class NeedleSpec:
    """
    Physical needle specification — loaded from needles.json.

    All dimensional values stored in micrometers (µm) internally.
    Convenience properties provide mm conversions for calculations.
    """
    gauge: int                          # 16–32+
    od_um: float                        # Outer diameter (µm)
    id_um: float                        # Inner diameter (µm)
    wall_um: float                      # Wall thickness (µm)
    length_inches: float = 1.0          # 1.0 or 2.0 (user-selected in GUI)
    num_channels: int = 1               # 1 for single, up to 3 for multi-channel
    channel_pump_map: dict | None = None  # e.g. {1: "P1", 2: "P2"}

    def __post_init__(self):
        if self.channel_pump_map is None:
            self.channel_pump_map = {1: "P1"}

    @property
    def length_mm(self) -> float:
        """Needle length in millimeters."""
        return self.length_inches * 25.4

    @property
    def id_mm(self) -> float:
        """Inner diameter in millimeters."""
        return self.id_um / 1000.0

    @property
    def od_mm(self) -> float:
        """Outer diameter in millimeters."""
        return self.od_um / 1000.0

    @property
    def wall_mm(self) -> float:
        """Wall thickness in millimeters."""
        return self.wall_um / 1000.0

    @property
    def id_m(self) -> float:
        """Inner diameter in meters (for flow physics)."""
        return self.id_um * 1e-6

    @property
    def cross_section_area_mm2(self) -> float:
        """Inner cross-sectional area in mm²."""
        return math.pi * (self.id_mm / 2) ** 2

    @property
    def internal_volume_uL(self) -> float:
        """Internal bore volume (µL) — the needle modelled as a cylinder of its
        inner diameter and length. ``1 mm³ == 1 µL``, so this is just
        ``cross_section_area_mm2 × length_mm``. This is "1 needle's worth" of
        fluid used by the pick & place prep (waste/oil/buffer volumes)."""
        return self.cross_section_area_mm2 * self.length_mm

    def to_dict(self) -> dict:
        """Serialize to dictionary."""
        return {
            "gauge": self.gauge,
            "od_um": self.od_um,
            "id_um": self.id_um,
            "wall_um": self.wall_um,
            "length_inches": self.length_inches,
            "num_channels": self.num_channels,
            "channel_pump_map": self.channel_pump_map,
        }

    @classmethod
    def from_dict(cls, data: dict) -> NeedleSpec:
        """Deserialize from dictionary."""
        return cls(**data)


# ---------------------------------------------------------------------------
# Syringe Specification
# ---------------------------------------------------------------------------

@dataclass
class SyringeSpec:
    """
    Hamilton 1700-series syringe — loaded from syringes.json.

    All volumes in µL, all lengths in mm.
    """
    volume_uL: int                      # 25, 50, 100, 250, 500, 1000
    stroke_length_mm: float = 30.0
    barrel_id_mm: float = 0.0
    part_number: str = ""

    @property
    def uL_per_mm(self) -> float:
        """Microliters dispensed per mm of plunger travel."""
        return self.volume_uL / self.stroke_length_mm

    @property
    def mm_per_uL(self) -> float:
        """Millimeters of plunger travel per microliter."""
        return self.stroke_length_mm / self.volume_uL

    @property
    def cross_section_area_mm2(self) -> float:
        """Barrel cross-sectional area for flow calculations (mm²)."""
        return math.pi * (self.barrel_id_mm / 2) ** 2

    def uL_to_mm(self, volume_uL: float) -> float:
        """Convert volume in µL to plunger travel in mm."""
        return volume_uL * self.mm_per_uL

    def mm_to_uL(self, travel_mm: float) -> float:
        """Convert plunger travel in mm to volume in µL."""
        return travel_mm * self.uL_per_mm

    def to_dict(self) -> dict:
        return {
            "volume_uL": self.volume_uL,
            "stroke_length_mm": self.stroke_length_mm,
            "barrel_id_mm": self.barrel_id_mm,
            "part_number": self.part_number,
        }

    @classmethod
    def from_dict(cls, data: dict) -> SyringeSpec:
        return cls(**data)


# ---------------------------------------------------------------------------
# JSON Catalog Loaders
# ---------------------------------------------------------------------------

def load_needle_catalog(path: str | Path = DEFAULT_NEEDLES_JSON) -> dict[int, NeedleSpec]:
    """
    Load needle gauge specs from JSON.

    Returns: {gauge_int: NeedleSpec} e.g. {16: NeedleSpec(...), 22: NeedleSpec(...)}
    """
    path = Path(path)
    if not path.exists():
        logger.warning(f"Needle catalog not found at {path}, returning empty catalog")
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
        catalog = {}
        for gauge_str, dims in data["needles"].items():
            gauge = int(gauge_str)
            catalog[gauge] = NeedleSpec(gauge=gauge, **dims)
        logger.info(f"Loaded {len(catalog)} needle specs from {path}")
        return catalog
    except (json.JSONDecodeError, KeyError) as e:
        logger.error(f"Error loading needle catalog from {path}: {e}")
        return {}


def load_syringe_catalog(path: str | Path = DEFAULT_SYRINGES_JSON) -> dict[int, SyringeSpec]:
    """
    Load syringe specs from JSON.

    Returns: {volume_uL: SyringeSpec} e.g. {25: SyringeSpec(...), 100: SyringeSpec(...)}
    """
    path = Path(path)
    if not path.exists():
        logger.warning(f"Syringe catalog not found at {path}, returning empty catalog")
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
        stroke = data.get("stroke_length_mm", 30.0)
        catalog = {}
        for vol_str, spec in data["syringes"].items():
            vol = int(vol_str)
            catalog[vol] = SyringeSpec(
                volume_uL=vol,
                stroke_length_mm=stroke,
                barrel_id_mm=spec["barrel_id_mm"],
                part_number=spec.get("part_number", ""),
            )
        logger.info(f"Loaded {len(catalog)} syringe specs from {path}")
        return catalog
    except (json.JSONDecodeError, KeyError) as e:
        logger.error(f"Error loading syringe catalog from {path}: {e}")
        return {}


# ---------------------------------------------------------------------------
# Camera Specification (v7.3.0)
# ---------------------------------------------------------------------------

@dataclass
class CameraSpec:
    """
    Microscope camera sensor specification — loaded from cameras.json.

    Stores physical sensor properties needed for pixel-to-micron conversion
    in autocalibration. The effective pixel size depends on the active
    resolution (binning/downscaling from max resolution).
    """
    name: str                                       # e.g. "BUC3D-1000C"
    sensor_pixel_size_um: float                     # Physical pixel pitch on sensor (µm)
    max_resolution: tuple[int, int] = (3664, 2748)  # (width, height) at full res
    preview_resolutions: list[tuple[int, int]] = field(default_factory=list)
    interface: str = "USB 3.0"
    notes: str = ""

    def effective_pixel_size_um(self, active_resolution: tuple[int, int]) -> float:
        """
        Compute effective pixel size at a given active resolution.

        When the camera is binning or downscaling from max resolution,
        each output pixel covers more physical area.
        """
        if active_resolution[0] <= 0:
            return self.sensor_pixel_size_um
        bin_factor = self.max_resolution[0] / active_resolution[0]
        return self.sensor_pixel_size_um * bin_factor

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "sensor_pixel_size_um": self.sensor_pixel_size_um,
            "max_resolution": list(self.max_resolution),
            "preview_resolutions": [list(r) for r in self.preview_resolutions],
            "interface": self.interface,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: dict) -> CameraSpec:
        return cls(
            name=data["name"],
            sensor_pixel_size_um=data["sensor_pixel_size_um"],
            max_resolution=tuple(data.get("max_resolution", [3664, 2748])),
            preview_resolutions=[
                tuple(r) for r in data.get("preview_resolutions", [])
            ],
            interface=data.get("interface", "USB 3.0"),
            notes=data.get("notes", ""),
        )


def load_camera_catalog(path: str | Path = DEFAULT_CAMERAS_JSON) -> dict[str, CameraSpec]:
    """
    Load camera specs from JSON.

    Returns: {camera_name: CameraSpec} e.g. {"BUC3D-1000C": CameraSpec(...)}
    """
    path = Path(path)
    if not path.exists():
        logger.warning(f"Camera catalog not found at {path}, returning empty catalog")
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
        catalog = {}
        for name, spec in data.get("cameras", {}).items():
            spec_with_name = dict(spec)
            spec_with_name.setdefault("name", name)
            catalog[name] = CameraSpec.from_dict(spec_with_name)
        logger.info(f"Loaded {len(catalog)} camera specs from {path}")
        return catalog
    except (json.JSONDecodeError, KeyError) as e:
        logger.error(f"Error loading camera catalog from {path}: {e}")
        return {}


# ---------------------------------------------------------------------------
# Ink Specification
# ---------------------------------------------------------------------------

@dataclass
class InkSpec:
    """
    Printing material specification.

    Covers bioinks, cell suspensions, hydrogels, media, buffers, and custom inks.
    The granule/cell diameter is used for needle compatibility checks and
    shear stress safety calculations — NOT for extrusion rate corrections.
    """
    name: str
    # v7.5.x: ``ink_type`` is the WELL TYPE — one of WELL_TYPES
    # (ink/wash/buffer/waste/oil). It drives all behavior (service-well
    # resolution, roles, colors). The legacy material categories
    # (granular/cells/media/hydrogel/custom) now live in ``ink_subtype`` and a
    # loaded legacy ``ink_type`` is migrated to ``ink`` + the matching subtype
    # (see split_legacy_ink_type, applied in from_dict).
    ink_type: str = "ink"
    # Informational material subtype, meaningful only when ink_type == "ink"
    # (e.g. "cells", "hydrogel monomer", "ELISA Beads"). Editable; see INK_SUBTYPES.
    ink_subtype: str = ""
    viscosity_cP: float = 1.0           # Dynamic viscosity (centipoise)
    granule_diameter_um: float = 0.0    # For granular inks
    cell_diameter_um: float = 0.0       # For cell inks
    density_g_mL: float = 1.0           # Density
    color: str = "#a6e3a1"              # Display color (hex)

    @property
    def viscosity_Pa_s(self) -> float:
        """Dynamic viscosity in Pa·s (SI units)."""
        return self.viscosity_cP * 0.001

    @property
    def max_particle_diameter_um(self) -> float:
        """Largest particle dimension (granule or cell)."""
        return max(self.granule_diameter_um, self.cell_diameter_um)

    def can_flow_through(self, needle: NeedleSpec) -> str:
        """
        Check if ink can pass through needle.

        Returns status string:
        - "free_flow": needle ID > 4× max particle diameter
        - "risk_clogging": 1× < needle ID < 4× particle diameter
        - "pick_and_place": needle ID < 1× particle diameter (single pickup only)
        - "compatible": no particles (pure liquid)
        """
        particle_d = self.max_particle_diameter_um
        if particle_d <= 0:
            return "compatible"

        ratio = needle.id_um / particle_d
        if ratio >= 4.0:
            return "free_flow"
        elif ratio >= 1.0:
            return "risk_clogging"
        else:
            return "pick_and_place"

    def flow_compatibility_detail(self, needle: NeedleSpec) -> dict:
        """
        Detailed compatibility info for GUI display.

        Returns dict with status, ratio, message, and severity.
        """
        particle_d = self.max_particle_diameter_um
        if particle_d <= 0:
            return {
                "status": "compatible",
                "ratio": float("inf"),
                "message": f"{self.name} is a pure liquid — flows freely through {needle.gauge}G",
                "severity": "ok",
            }

        ratio = needle.id_um / particle_d
        if ratio >= 4.0:
            return {
                "status": "free_flow",
                "ratio": round(ratio, 1),
                "message": (f"{self.name} flows freely through {needle.gauge}G "
                           f"(ID/particle = {ratio:.1f}×)"),
                "severity": "ok",
            }
        elif ratio >= 1.0:
            return {
                "status": "risk_clogging",
                "ratio": round(ratio, 1),
                "message": (f"⚠️ {self.name} near jamming limit for {needle.gauge}G "
                           f"(ID/particle = {ratio:.1f}×, need >4×)"),
                "severity": "warning",
            }
        else:
            return {
                "status": "pick_and_place",
                "ratio": round(ratio, 1),
                "message": (f"{self.name} too large for {needle.gauge}G "
                           f"(ID/particle = {ratio:.1f}×) — pick-and-place only"),
                "severity": "info",
            }

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "ink_type": self.ink_type,
            "ink_subtype": self.ink_subtype,
            "viscosity_cP": self.viscosity_cP,
            "granule_diameter_um": self.granule_diameter_um,
            "cell_diameter_um": self.cell_diameter_um,
            "density_g_mL": self.density_g_mL,
            "color": self.color,
        }

    @classmethod
    def from_dict(cls, data: dict) -> InkSpec:
        # v7.5.x: migrate a legacy material ink_type (granular/cells/...) to the
        # well-type + subtype scheme. Service/ink well types pass through.
        well_type, subtype = split_legacy_ink_type(
            data.get("ink_type"), data.get("ink_subtype", ""))
        return cls(
            name=data["name"],
            ink_type=well_type,
            ink_subtype=subtype,
            viscosity_cP=data.get("viscosity_cP", 1.0),
            granule_diameter_um=data.get("granule_diameter_um", 0.0),
            cell_diameter_um=data.get("cell_diameter_um", 0.0),
            density_g_mL=data.get("density_g_mL", 1.0),
            color=data.get("color", "#a6e3a1"),
        )


# ---------------------------------------------------------------------------
# Fluid Column Model
# ---------------------------------------------------------------------------

@dataclass
class FluidColumn:
    """
    Tracks the layered fluid state inside one syringe + tubing + needle.

    Physical layout (from plunger tip to needle tip):
        [=== mineral oil ===][== buffer ==][=== ink ===] → needle tip

    All volumes in µL. Plunger positions are converted via SyringeSpec.
    The oil stays in the syringe as a hydraulic medium. A buffer layer
    separates oil from ink to prevent mixing. The ink is drawn up into
    the needle/tubing from a well.
    """
    oil_volume_uL: float = 0.0         # Mineral oil (hydraulic medium, always present)
    buffer_volume_uL: float = 0.0      # Buffer separating oil from ink
    ink_volume_uL: float = 0.0         # Current ink loaded
    ink_spec: InkSpec | None = None    # What ink is currently loaded
    dead_volume_uL: float = 2.0        # Tubing + needle dead volume (measured)

    @property
    def total_volume_uL(self) -> float:
        """Total fluid volume in the system."""
        return self.oil_volume_uL + self.buffer_volume_uL + self.ink_volume_uL

    @property
    def is_empty(self) -> bool:
        """Whether any ink is loaded."""
        return self.ink_volume_uL <= 0

    @property
    def has_buffer(self) -> bool:
        """Whether buffer layer is present."""
        return self.buffer_volume_uL > 0

    def can_dispense(self, amount_uL: float) -> bool:
        """Check if enough ink remains to dispense the requested amount."""
        return self.ink_volume_uL >= amount_uL

    def dispense(self, amount_uL: float) -> float:
        """
        Record dispensing ink (plunger pushes oil→buffer→ink out).

        Returns actual amount dispensed (may be less than requested if low).
        """
        actual = min(amount_uL, self.ink_volume_uL)
        self.ink_volume_uL = max(0.0, self.ink_volume_uL - actual)
        return actual

    def aspirate_ink(self, amount_uL: float, ink: InkSpec) -> None:
        """Record aspirating ink into needle (plunger pulls back)."""
        self.ink_volume_uL += amount_uL
        self.ink_spec = ink

    def waste_ink(self) -> float:
        """
        Dispense all ink to waste (push until only buffer remains).

        Returns the volume dispensed.
        """
        dispensed = self.ink_volume_uL
        self.ink_volume_uL = 0.0
        self.ink_spec = None
        return dispensed

    def refresh_buffer(self, buffer_uL: float, buffer_ink: InkSpec | None = None) -> None:
        """Reset buffer layer (waste old buffer+ink, aspirate fresh buffer)."""
        self.ink_volume_uL = 0.0
        self.ink_spec = None
        self.buffer_volume_uL = buffer_uL

    def volume_fractions(self, syringe: SyringeSpec | None = None) -> dict[str, float]:
        """
        Get volume fractions for visualization (0.0–1.0).

        If syringe is provided, fractions are relative to syringe capacity.
        Otherwise, fractions are relative to total fluid volume.
        """
        total = syringe.volume_uL if syringe else self.total_volume_uL
        if total <= 0:
            return {"oil": 0.0, "buffer": 0.0, "ink": 0.0, "empty": 1.0}
        return {
            "oil": self.oil_volume_uL / total,
            "buffer": self.buffer_volume_uL / total,
            "ink": self.ink_volume_uL / total,
            "empty": max(0.0, 1.0 - self.total_volume_uL / total),
        }

    def to_dict(self) -> dict:
        return {
            "oil_volume_uL": self.oil_volume_uL,
            "buffer_volume_uL": self.buffer_volume_uL,
            "ink_volume_uL": self.ink_volume_uL,
            "ink_spec": self.ink_spec.to_dict() if self.ink_spec else None,
            "dead_volume_uL": self.dead_volume_uL,
        }

    @classmethod
    def from_dict(cls, data: dict) -> FluidColumn:
        ink_data = data.pop("ink_spec", None)
        fc = cls(**data)
        if ink_data:
            fc.ink_spec = InkSpec.from_dict(ink_data)
        return fc


# ---------------------------------------------------------------------------
# Pump Loadout
# ---------------------------------------------------------------------------

@dataclass
class PumpLoadout:
    """Configuration and state for one pump channel."""
    pump_id: str                            # "P1", "P2", "P3"
    syringe: SyringeSpec | None = None
    fluid_column: FluidColumn = field(default_factory=FluidColumn)
    printing_mode: PrintingMode = PrintingMode.INCREMENTAL
    current_position_mm: float = 0.0        # Tracked plunger position (mm)

    @property
    def current_position_uL(self) -> float:
        """Current plunger position expressed as volume (µL)."""
        if self.syringe:
            return self.current_position_mm * self.syringe.uL_per_mm
        return 0.0

    @property
    def remaining_ink_uL(self) -> float:
        """Remaining ink volume in µL."""
        return self.fluid_column.ink_volume_uL

    @property
    def current_ink(self) -> InkSpec | None:
        """Currently loaded ink spec, or None."""
        return self.fluid_column.ink_spec

    @property
    def capacity_uL(self) -> int:
        """Syringe total capacity in µL."""
        return self.syringe.volume_uL if self.syringe else 0

    def needs_refill(self, threshold_uL: float = 5.0) -> bool:
        """Check if ink level is below refill threshold."""
        return self.fluid_column.ink_volume_uL < threshold_uL

    def to_dict(self) -> dict:
        return {
            "pump_id": self.pump_id,
            "syringe": self.syringe.to_dict() if self.syringe else None,
            "fluid_column": self.fluid_column.to_dict(),
            "printing_mode": self.printing_mode.value,
            "current_position_mm": self.current_position_mm,
        }

    @classmethod
    def from_dict(cls, data: dict) -> PumpLoadout:
        syringe_data = data.pop("syringe", None)
        fc_data = data.pop("fluid_column", None)
        mode_str = data.pop("printing_mode", "incremental")

        pump = cls(
            pump_id=data["pump_id"],
            printing_mode=PrintingMode(mode_str),
            current_position_mm=data.get("current_position_mm", 0.0),
        )
        if syringe_data:
            pump.syringe = SyringeSpec.from_dict(syringe_data)
        if fc_data:
            pump.fluid_column = FluidColumn.from_dict(fc_data)
        return pump


# ---------------------------------------------------------------------------
# Rosette Well Insert
# ---------------------------------------------------------------------------

@dataclass
class RosetteSubWell:
    """One sub-well within a rosette insert."""
    index: int                          # 0-based position in rosette
    angle_deg: float                    # Angular position (0° = 12 o'clock)
    radial_offset_mm: float             # Distance from well center to sub-well center
    diameter_mm: float                  # Sub-well opening diameter
    depth_mm: float                     # Sub-well depth
    z_offset_mm: float = 0.0           # Z offset of sub-well bottom vs well bottom

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> RosetteSubWell:
        return cls(**data)


@dataclass
class RosetteInsert:
    """
    A rosette insert that fits into a standard well.

    Physical object: a cylindrical plug with N sub-wells arranged in a
    ring pattern, optionally with a center sub-well.

    The same physical insert can serve different roles:
    - Ink rosette: each sub-well holds a different ink
    - Sort rosette: each sub-well collects a different cell type
    - Service rosette: wash + waste + buffer in one well

    The role/purpose is defined by the well assignment (Tab 3), not here.
    """
    name: str
    well_format: int                    # Which plate format this fits (6, 12, 24...)
    num_subwells: int                   # Total sub-wells including center
    has_center_well: bool = True        # Whether there's a center sub-well
    ring_radius_mm: float = 0.0         # Radius of the sub-well ring
    subwell_diameter_mm: float = 0.0
    subwell_depth_mm: float = 0.0
    insert_z_offset_mm: float = 0.0     # How much the insert raises the bottom

    subwells: list[RosetteSubWell] = field(default_factory=list)

    @classmethod
    def create_standard(
        cls,
        name: str,
        well_format: int,
        num_ring: int,
        has_center: bool = True,
        ring_radius_mm: float = 0.0,
        subwell_diameter_mm: float = 1.5,
        subwell_depth_mm: float = 6.0,
        insert_z_offset_mm: float = 0.0,
        well_diameter_mm: float | None = None,
    ) -> RosetteInsert:
        """
        Create a rosette with evenly-spaced sub-wells.

        Automatically calculates sub-well angular positions. If ring_radius_mm
        is 0, it's auto-calculated to fit within the well.

        Args:
            name: Rosette identifier
            well_format: Plate format (6, 12, 24, etc.)
            num_ring: Number of sub-wells in the ring (excluding center)
            has_center: Include a center sub-well
            ring_radius_mm: Ring radius (0 = auto-calculate)
            subwell_diameter_mm: Diameter of each sub-well
            subwell_depth_mm: Depth of each sub-well
            insert_z_offset_mm: Z-offset from well bottom
            well_diameter_mm: Override well diameter for auto-radius calc
        """
        # Standard well diameters by format (approximate, mm)
        WELL_DIAMETERS = {
            6: 34.8, 12: 22.1, 24: 15.6, 48: 11.0, 96: 6.4, 384: 3.3,
        }

        if well_diameter_mm is None:
            well_diameter_mm = WELL_DIAMETERS.get(well_format, 15.0)

        # Auto-calculate ring radius to fit sub-wells inside well
        if ring_radius_mm <= 0:
            # Leave clearance: ring radius = (well_radius - subwell_radius) * 0.7
            ring_radius_mm = (well_diameter_mm / 2 - subwell_diameter_mm / 2) * 0.7

        total = num_ring + (1 if has_center else 0)
        subwells = []
        idx = 0

        # Center sub-well (if present)
        if has_center:
            subwells.append(RosetteSubWell(
                index=idx,
                angle_deg=0.0,
                radial_offset_mm=0.0,
                diameter_mm=subwell_diameter_mm,
                depth_mm=subwell_depth_mm,
                z_offset_mm=0.0,
            ))
            idx += 1

        # Ring sub-wells — evenly spaced starting from 12 o'clock
        angle_step = 360.0 / num_ring if num_ring > 0 else 0
        for i in range(num_ring):
            subwells.append(RosetteSubWell(
                index=idx,
                angle_deg=i * angle_step,
                radial_offset_mm=ring_radius_mm,
                diameter_mm=subwell_diameter_mm,
                depth_mm=subwell_depth_mm,
                z_offset_mm=0.0,
            ))
            idx += 1

        return cls(
            name=name,
            well_format=well_format,
            num_subwells=total,
            has_center_well=has_center,
            ring_radius_mm=ring_radius_mm,
            subwell_diameter_mm=subwell_diameter_mm,
            subwell_depth_mm=subwell_depth_mm,
            insert_z_offset_mm=insert_z_offset_mm,
            subwells=subwells,
        )

    def get_subwell_xy(self, index: int) -> tuple[float, float]:
        """
        Get (x, y) offset of sub-well relative to parent well center.

        Convention: 0° = positive Y (12 o'clock), 90° = positive X (3 o'clock).
        """
        sw = self.subwells[index]
        if sw.radial_offset_mm == 0:
            return (0.0, 0.0)
        rad = math.radians(sw.angle_deg)
        return (
            sw.radial_offset_mm * math.sin(rad),
            sw.radial_offset_mm * math.cos(rad),
        )

    def get_subwell_z_bottom(self, index: int) -> float:
        """Get Z position of sub-well bottom (for needle depth targeting)."""
        return self.insert_z_offset_mm + self.subwells[index].z_offset_mm

    def get_all_subwell_positions(self) -> list[tuple[float, float, float]]:
        """Get (x, y, z_bottom) for all sub-wells."""
        positions = []
        for i, sw in enumerate(self.subwells):
            x, y = self.get_subwell_xy(i)
            z = self.get_subwell_z_bottom(i)
            positions.append((x, y, z))
        return positions

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "well_format": self.well_format,
            "num_subwells": self.num_subwells,
            "has_center_well": self.has_center_well,
            "ring_radius_mm": self.ring_radius_mm,
            "subwell_diameter_mm": self.subwell_diameter_mm,
            "subwell_depth_mm": self.subwell_depth_mm,
            "insert_z_offset_mm": self.insert_z_offset_mm,
            "subwells": [sw.to_dict() for sw in self.subwells],
        }

    @classmethod
    def from_dict(cls, data: dict) -> RosetteInsert:
        subwells_data = data.pop("subwells", [])
        insert = cls(**data)
        insert.subwells = [RosetteSubWell.from_dict(sw) for sw in subwells_data]
        return insert


# ---------------------------------------------------------------------------
# Workspace Config
# ---------------------------------------------------------------------------

@dataclass
class WorkspaceConfig:
    """
    Complete physical configuration for a print session.

    Aggregates needle, pump loadouts, plate format, rosette library,
    ink library, and print settings into a single shareable object
    passed between GUI tabs.
    """
    needle: NeedleSpec | None = None
    pumps: dict[str, PumpLoadout] = field(default_factory=lambda: {
        "P1": PumpLoadout(pump_id="P1"),
        "P2": PumpLoadout(pump_id="P2"),
        "P3": PumpLoadout(pump_id="P3"),
    })
    plate_format: int = 24                  # 6, 12, 24, 48, 96 (legacy)
    # v7.4.5: custom-plate name when set; takes precedence over plate_format
    # via `active_plate_key`.
    plate_name: str = ""
    rosette_library: dict[str, RosetteInsert] = field(default_factory=dict)
    ink_library: dict[str, InkSpec] = field(default_factory=dict)
    buffer_ink: InkSpec | None = None       # The buffer material (e.g. DPBS)

    # Print defaults
    print_settings: dict[str, Any] = field(default_factory=lambda: {
        "travel_z_mm": 5.0,
        "layer_height_mm": 0.2,
        "print_speed_mm_s": 5.0,
        "z_feed_rate_mm_s": 2.0,
        "pump_feed_rate_uL_s": 1.0,
        "retract_distance_uL": 0.5,
        "prime_distance_uL": 0.5,
        "retract_speed_uL_s": 2.0,
        "prime_speed_uL_s": 1.0,
    })

    @property
    def active_plate_key(self) -> int | str:
        """v7.4.5: returns plate_name (str) when set, else plate_format (int)."""
        return self.plate_name if self.plate_name else self.plate_format

    def get_pump(self, pump_id: str) -> PumpLoadout | None:
        """Get pump loadout by ID."""
        return self.pumps.get(pump_id)

    def get_active_pumps(self) -> list[PumpLoadout]:
        """Get all pumps with a syringe attached."""
        return [p for p in self.pumps.values() if p.syringe is not None]

    def get_ink(self, name: str) -> InkSpec | None:
        """Get ink from library by name."""
        return self.ink_library.get(name)

    def get_rosette(self, name: str) -> RosetteInsert | None:
        """Get rosette from library by name."""
        return self.rosette_library.get(name)

    def validate(self) -> list[str]:
        """
        Validate workspace configuration.

        Returns list of warning/error strings. Empty list = valid.
        """
        issues = []
        if self.needle is None:
            issues.append("No needle selected")
        if not any(p.syringe for p in self.pumps.values()):
            issues.append("No syringes configured — attach at least one syringe")
        # v7.4.5: a non-empty plate_name overrides plate_format; both
        # branches are valid (load() resolves the right plate).
        if not self.plate_name and self.plate_format not in (6, 12, 24, 48, 96, 384):
            issues.append(f"Unknown plate format: {self.plate_format}")
        return issues

    def to_dict(self) -> dict:
        return {
            "needle": self.needle.to_dict() if self.needle else None,
            "pumps": {k: v.to_dict() for k, v in self.pumps.items()},
            "plate_format": self.plate_format,
            "plate_name": self.plate_name,                          # v7.4.5
            "rosette_library": {k: v.to_dict() for k, v in self.rosette_library.items()},
            "ink_library": {k: v.to_dict() for k, v in self.ink_library.items()},
            "buffer_ink": self.buffer_ink.to_dict() if self.buffer_ink else None,
            "print_settings": dict(self.print_settings),
        }

    @classmethod
    def from_dict(cls, data: dict) -> WorkspaceConfig:
        ws = cls()
        # Needle
        if data.get("needle"):
            ws.needle = NeedleSpec.from_dict(data["needle"])
        # Pumps
        if data.get("pumps"):
            ws.pumps = {k: PumpLoadout.from_dict(v) for k, v in data["pumps"].items()}
        # Plate format
        ws.plate_format = data.get("plate_format", 24)
        ws.plate_name = data.get("plate_name", "") or ""              # v7.4.5
        # Rosette library
        if data.get("rosette_library"):
            ws.rosette_library = {
                k: RosetteInsert.from_dict(v)
                for k, v in data["rosette_library"].items()
            }
        # Ink library
        if data.get("ink_library"):
            ws.ink_library = {
                k: InkSpec.from_dict(v)
                for k, v in data["ink_library"].items()
            }
        # Buffer ink
        if data.get("buffer_ink"):
            ws.buffer_ink = InkSpec.from_dict(data["buffer_ink"])
        # Print settings
        if data.get("print_settings"):
            ws.print_settings.update(data["print_settings"])
        return ws

    def save_json(self, filepath: str | Path) -> None:
        """Save workspace config to JSON file."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        logger.info(f"Workspace saved to {filepath}")

    @classmethod
    def load_json(cls, filepath: str | Path) -> WorkspaceConfig:
        """Load workspace config from JSON file."""
        with open(filepath) as f:
            data = json.load(f)
        ws = cls.from_dict(data)
        logger.info(f"Workspace loaded from {filepath}")
        return ws
