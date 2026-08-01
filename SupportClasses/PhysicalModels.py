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
from dataclasses import (
    dataclass, field, asdict, fields as _dc_fields, replace as _dc_replace,
)
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

# --- Needle types (v7.6) ---------------------------------------------------
# A HYPODERMIC needle is one straight bore: geometry comes from the ASTM gauge
# catalog and the whole needle is a single cylinder.
# A PULLED_CAPILLARY is TWO flow stages in series — a wide bulk barrel feeding a
# narrow pulled tip. The tip sets the deposited feature size and dominates the
# flow resistance; the barrel sets the held volume.
NEEDLE_TYPE_HYPODERMIC = "hypodermic"
NEEDLE_TYPE_CAPILLARY = "pulled_capillary"
NEEDLE_TYPES = (NEEDLE_TYPE_HYPODERMIC, NEEDLE_TYPE_CAPILLARY)

# How the pulled section is modelled between the barrel bore and the orifice.
TIP_PROFILE_CYLINDER = "cylinder"   # straight capillary of the tip Ø over its length
TIP_PROFILE_CONE = "cone"           # linear taper from the barrel Ø down to the tip Ø
TIP_PROFILES = (TIP_PROFILE_CYLINDER, TIP_PROFILE_CONE)

# --- Needle FORM (v7.9) ----------------------------------------------------
# How many bores the *assembly* has, and how they are arranged. This axis is
# ORTHOGONAL to ``needle_type`` (which describes one bore's taper) — a backpack
# of two pulled capillaries needs both. Do NOT smuggle a form into
# ``needle_type``: ``NeedleSpec.__post_init__`` silently rewrites an unknown
# needle_type to "hypodermic", so the whole assembly would vanish with no error.
NEEDLE_FORM_SINGLE = "single"        # one bore — every needle before v7.9
NEEDLE_FORM_BACKPACK = "backpack"    # two needles of DIFFERENT sizes bound together
NEEDLE_FORM_TRIPLE = "triple"        # three needles fused together
NEEDLE_FORMS = (NEEDLE_FORM_SINGLE, NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE)

# Nominal bore count per form. The authority is always ``len(bores_resolved())``;
# this only seeds the GUI and validates what the operator selected.
NEEDLE_FORM_BORE_COUNT = {
    NEEDLE_FORM_SINGLE: 1,
    NEEDLE_FORM_BACKPACK: 2,
    NEEDLE_FORM_TRIPLE: 3,
}


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
# Needle bore geometry (v7.6)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class FlowSegment:
    """One axial stage of a needle bore, ordered barrel → tip.

    ``d_in_um == d_out_um`` for a straight cylinder; they differ for a taper.
    """
    name: str
    length_mm: float
    d_in_um: float
    d_out_um: float

    @property
    def is_taper(self) -> bool:
        return abs(self.d_out_um - self.d_in_um) > 1e-9

    @property
    def length_m(self) -> float:
        return self.length_mm / 1000.0

    @property
    def volume_uL(self) -> float:
        """Bore volume of this stage (µL; 1 mm³ == 1 µL). A taper is a cone
        frustum: V = (π·L/12)·(d1² + d1·d2 + d2²)."""
        d1, d2 = self.d_in_um / 1000.0, self.d_out_um / 1000.0
        if self.is_taper:
            return (math.pi * self.length_mm / 12.0) * (d1 * d1 + d1 * d2 + d2 * d2)
        return math.pi * (d1 / 2) ** 2 * self.length_mm

    @property
    def equivalent_area_mm2(self) -> float:
        """Equal-VOLUME cylinder area (mm²) — lets a taper be used in the
        closed-form lift↔volume conversion of :class:`BoreProfile`."""
        return self.volume_uL / self.length_mm if self.length_mm > 0 else 0.0

    @property
    def resistance_factor(self) -> float:
        """Hydraulic resistance factor K (m⁻³) with R = 128·µ/π · K.

        cylinder: K = L / d⁴
        cone:     K = (L/3)·(d1² + d1·d2 + d2²) / (d1³·d2³)

        The cone form is the exact conical-Poiseuille result and reduces to the
        cylinder form at d1 == d2 (3d²/d⁶ == 3/d⁴, the 1/3 cancelling).
        """
        d1, d2 = self.d_in_um * 1e-6, self.d_out_um * 1e-6
        if d1 <= 0 or d2 <= 0:
            return float("inf")
        if self.is_taper:
            return (self.length_m / 3.0) * (d1 * d1 + d1 * d2 + d2 * d2) / (d1 ** 3 * d2 ** 3)
        return self.length_m / (d1 ** 4)


@dataclass(frozen=True)
class BoreProfile:
    """Volume ↔ axial-lift conversion for a needle bore, measured UP from the tip.

    The bore is a narrow tip stage (area A2, length L2) stacked under a wide
    barrel (area A1). Something aspirated by V µL rises V/A2 while it is still
    inside the tip, then far more slowly once it enters the barrel::

        lift(V) = V/A2                      for V <= A2·L2
                = L2 + (V - A2·L2)/A1       otherwise
        V(h)    = h·A2                      for h <= L2
                = A2·L2 + (h - L2)·A1       otherwise

    With no tip stage (L2 == 0) both collapse to the single-area forms ``V/A1``
    and ``h·A1`` — byte-identical to the pre-v7.6 scalar ``bore_area_mm2``
    arithmetic.

    A TAPERED tip is represented by its equal-VOLUME cylinder area
    (:attr:`FlowSegment.equivalent_area_mm2`): exact in volume, monotone, and
    closed-form invertible. The exact frustum inverse is a cubic, which is not
    worth the numerics for a timing estimate — this is the single approximation
    in the sink model and it is documented here, in one place.
    """
    barrel_area_mm2: float
    tip_area_mm2: float = 0.0
    tip_length_mm: float = 0.0

    @property
    def has_tip(self) -> bool:
        return self.tip_area_mm2 > 0.0 and self.tip_length_mm > 0.0

    @property
    def tip_capacity_uL(self) -> float:
        """Volume that fits inside the tip stage before reaching the barrel."""
        return self.tip_area_mm2 * self.tip_length_mm if self.has_tip else 0.0

    @property
    def near_tip_area_mm2(self) -> float:
        """The area governing the FIRST microlitre of lift — what a legacy
        scalar ``bore_area_mm2`` should be replaced by."""
        return self.tip_area_mm2 if self.has_tip else self.barrel_area_mm2

    def is_usable(self) -> bool:
        return self.near_tip_area_mm2 > 0.0

    def lift_for_volume(self, volume_uL: float) -> float:
        """Axial rise (mm) produced by aspirating ``volume_uL``."""
        a_near = self.near_tip_area_mm2
        if a_near <= 0.0:
            return 0.0
        v = max(0.0, float(volume_uL))
        if not self.has_tip:
            return v / a_near
        cap = self.tip_capacity_uL
        if v <= cap:
            return v / self.tip_area_mm2
        if self.barrel_area_mm2 <= 0.0:
            return self.tip_length_mm
        return self.tip_length_mm + (v - cap) / self.barrel_area_mm2

    def volume_for_lift(self, lift_mm: float) -> float:
        """Volume (µL) needed to produce an axial rise of ``lift_mm``."""
        h = max(0.0, float(lift_mm))
        if not self.has_tip:
            return h * self.barrel_area_mm2
        if h <= self.tip_length_mm:
            return h * self.tip_area_mm2
        return self.tip_capacity_uL + (h - self.tip_length_mm) * self.barrel_area_mm2

    @classmethod
    def from_area(cls, area_mm2: float) -> BoreProfile:
        """Legacy single-area profile (backward-compat shim)."""
        return cls(barrel_area_mm2=float(area_mm2 or 0.0))


# ---------------------------------------------------------------------------
# One bore of a needle assembly (v7.9)
# ---------------------------------------------------------------------------

@dataclass
class NeedleBore:
    """ONE lumen of a needle assembly, with its own geometry, pump and offsets.

    Before v7.9 a needle was one bore and ``NeedleSpec``'s flat fields described
    it. A **backpack** ("two needles of *different sizes* bound together") is
    literally unrepresentable that way — there is nowhere to put the second
    diameter — so geometry moves here, per bore, and ``NeedleSpec`` keeps its
    flat fields as a mirror of **bore 0** for byte-identical legacy behaviour.

    Fields mirror ``NeedleSpec``'s naming rule exactly:
      * ``id_*`` / ``od_*`` / ``wall_um`` / ``length_mm`` are the **barrel**.
      * ``orifice_*`` / :attr:`orifice_area_mm2` are the **exit hole** — the
        pulled tip when present, else the barrel.

    MOUNT OFFSETS (``offset_um``, ``z_offset_mm``) are a **per-mount
    calibration, not a manufacturing constant**: a fused assembly's rotation
    about Z in the holder is arbitrary, so they must be re-measured on every
    needle change or re-seat. They are stored here so the motion path can
    consume them, but the authority is ``NeedleBoreCalibrationStore``
    (per-machine) — never a preset library, which is the
    CAMERA_CAL_PERSIST_STORE lesson.

    SIGN CONVENTION — fixed once, pinned by a round-trip test::

        to place bore k on target T:  stage_xy = T - offset_um(k)
        bore k currently sits at:     stage_xy + offset_um(k)
        bore 0 is the datum:          offset_um(0) == (0.0, 0.0)

    A mis-signed offset is a *right-distance-wrong-way* error — the same class
    as the plate-orientation bugs recorded in CLAUDE.md.
    """
    # --- barrel geometry (same meaning as the NeedleSpec fields) ---
    id_um: float = 0.0
    od_um: float = 0.0
    wall_um: float = 0.0
    length_mm: float = 25.4
    gauge: int | None = None

    # --- optional pulled tip stage (same model as NeedleSpec) ---
    needle_type: str = NEEDLE_TYPE_HYPODERMIC
    tip_id_um: float | None = None
    tip_length_mm: float | None = None
    tip_od_um: float | None = None
    tip_profile: str = TIP_PROFILE_CYLINDER

    # --- assembly wiring ---
    pump_id: str | None = None          # "P1"/"P2"/"P3"; None = unassigned
    label: str = ""                     # operator display name for this bore

    # --- per-MOUNT calibration (see the class docstring) ---
    offset_um: tuple[float, float] = (0.0, 0.0)   # lateral offset from bore 0
    z_offset_mm: float = 0.0            # + = this tip reaches LOWER than bore 0

    def __post_init__(self):
        nt = (self.needle_type or "").strip().lower() or NEEDLE_TYPE_HYPODERMIC
        self.needle_type = nt if nt in NEEDLE_TYPES else NEEDLE_TYPE_HYPODERMIC
        tp = (self.tip_profile or "").strip().lower() or TIP_PROFILE_CYLINDER
        self.tip_profile = tp if tp in TIP_PROFILES else TIP_PROFILE_CYLINDER
        for name in ("tip_id_um", "tip_length_mm", "tip_od_um"):
            v = getattr(self, name)
            if v is None:
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                v = None
            setattr(self, name, v if (v is not None and v > 0.0) else None)
        # Normalize the offset to a 2-tuple of floats so consumers can unpack it
        # unconditionally (it round-trips through JSON as a list).
        try:
            ox, oy = self.offset_um
            self.offset_um = (float(ox), float(oy))
        except (TypeError, ValueError):
            self.offset_um = (0.0, 0.0)
        try:
            self.z_offset_mm = float(self.z_offset_mm)
        except (TypeError, ValueError):
            self.z_offset_mm = 0.0

    # --- predicates -------------------------------------------------------
    @property
    def has_tip(self) -> bool:
        return self.tip_id_um is not None and self.tip_length_mm is not None

    @property
    def is_capillary(self) -> bool:
        return self.needle_type == NEEDLE_TYPE_CAPILLARY

    # --- barrel -----------------------------------------------------------
    @property
    def id_mm(self) -> float:
        return self.id_um / 1000.0

    @property
    def od_mm(self) -> float:
        return self.od_um / 1000.0

    @property
    def id_m(self) -> float:
        return self.id_um * 1e-6

    @property
    def barrel_area_mm2(self) -> float:
        return math.pi * (self.id_mm / 2) ** 2

    # --- orifice ----------------------------------------------------------
    @property
    def orifice_id_um(self) -> float:
        return self.tip_id_um if self.has_tip else self.id_um

    @property
    def orifice_id_mm(self) -> float:
        return self.orifice_id_um / 1000.0

    @property
    def orifice_area_mm2(self) -> float:
        """The hole material passes through — this bore's own bead/column area."""
        return math.pi * (self.orifice_id_mm / 2) ** 2

    # Alias so a NeedleBore is duck-compatible with the ~30 call sites that read
    # ``cross_section_area_mm2`` off a needle-like object.
    @property
    def cross_section_area_mm2(self) -> float:
        return self.orifice_area_mm2

    @property
    def orifice_od_um(self) -> float:
        if self.tip_od_um is not None:
            return self.tip_od_um
        if self.has_tip and self.id_um > 0:
            return self.tip_id_um * (self.od_um / self.id_um)
        return self.od_um

    @property
    def orifice_od_mm(self) -> float:
        return self.orifice_od_um / 1000.0

    # --- lengths + volumes ------------------------------------------------
    @property
    def tip_length_mm_or_zero(self) -> float:
        return float(self.tip_length_mm) if self.has_tip else 0.0

    @property
    def total_length_mm(self) -> float:
        return float(self.length_mm) + self.tip_length_mm_or_zero

    @property
    def internal_volume_uL(self) -> float:
        """"One bore's worth" of fluid (µL) — barrel + tip.

        This is what the prep multiples (waste/oil/buffer) are counted in, and
        it is **per bore**: a backpack's two bores hold different volumes, so a
        single scalar cannot size both.
        """
        return sum(seg.volume_uL for seg in self.flow_segments())

    @property
    def ink_reserve_volume_uL(self) -> float:
        return (self.flow_segments()[-1].volume_uL if self.has_tip
                else self.internal_volume_uL)

    # --- segment geometry -------------------------------------------------
    def flow_segments(self) -> list[FlowSegment]:
        """This bore's axial stages, barrel first — same series model as
        :meth:`NeedleSpec.flow_segments`."""
        segs = [FlowSegment("barrel", float(self.length_mm), self.id_um, self.id_um)]
        if self.has_tip:
            d_in = self.id_um if self.tip_profile == TIP_PROFILE_CONE else self.tip_id_um
            segs.append(FlowSegment("tip", float(self.tip_length_mm), d_in, self.tip_id_um))
        return segs

    def bore_profile(self) -> BoreProfile:
        tip = next((s for s in self.flow_segments() if s.name == "tip"), None)
        return BoreProfile(
            barrel_area_mm2=self.barrel_area_mm2,
            tip_area_mm2=tip.equivalent_area_mm2 if tip else 0.0,
            tip_length_mm=tip.length_mm if tip else 0.0,
        )

    def resistance_factor(self) -> float:
        """ΣK over this bore's stages, in SERIES (R = 128·µ/π · ΣK)."""
        return sum(seg.resistance_factor for seg in self.flow_segments())

    def summary_line(self) -> str:
        parts = []
        if self.label:
            parts.append(self.label)
        if self.is_capillary:
            parts.append(f"barrel ID {self.id_um:.0f} µm")
            if self.has_tip:
                parts.append(f"tip ID {self.tip_id_um:.1f} µm")
        else:
            parts.append(f"{self.gauge}G" if self.gauge else "needle")
            if self.id_um:
                parts.append(f"ID {self.id_um:.0f} µm")
        if self.pump_id:
            parts.append(self.pump_id)
        return " · ".join(parts)

    def to_dict(self) -> dict:
        """Serialize. Only non-default optional keys are emitted so a bore list
        stays small and diffable."""
        d = {
            "id_um": self.id_um,
            "od_um": self.od_um,
            "wall_um": self.wall_um,
            "length_mm": self.length_mm,
        }
        if self.gauge is not None:
            d["gauge"] = self.gauge
        if self.needle_type != NEEDLE_TYPE_HYPODERMIC:
            d["needle_type"] = self.needle_type
        for key in ("tip_id_um", "tip_length_mm", "tip_od_um"):
            value = getattr(self, key)
            if value is not None:
                d[key] = value
        if self.tip_profile != TIP_PROFILE_CYLINDER:
            d["tip_profile"] = self.tip_profile
        if self.pump_id:
            d["pump_id"] = self.pump_id
        if self.label:
            d["label"] = self.label
        if tuple(self.offset_um) != (0.0, 0.0):
            d["offset_um"] = list(self.offset_um)
        if self.z_offset_mm:
            d["z_offset_mm"] = self.z_offset_mm
        return d

    @classmethod
    def from_dict(cls, data: dict) -> NeedleBore:
        """Deserialize, ignoring unknown keys (forward-compat, same rule as
        :meth:`NeedleSpec.from_dict`)."""
        data = dict(data or {})
        known = {f.name for f in _dc_fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            logger.debug("NeedleBore.from_dict: ignoring unknown key(s) %s", unknown)
        kwargs = {k: v for k, v in data.items() if k in known}
        if "offset_um" in kwargs:
            try:
                ox, oy = kwargs["offset_um"]
                kwargs["offset_um"] = (float(ox), float(oy))
            except (TypeError, ValueError):
                kwargs.pop("offset_um")
        return cls(**kwargs)


# ---------------------------------------------------------------------------
# Needle Specification
# ---------------------------------------------------------------------------

@dataclass
class NeedleSpec:
    """
    Physical needle specification — hypodermic gauges load from needles.json;
    pulled glass capillaries are entered on Hardware Setup → Needle.

    All dimensional values stored in micrometers (µm) internally.
    Convenience properties provide mm conversions for calculations.

    TWO STAGES (v7.6). The base fields describe the **bulk barrel**; for a
    straight hypodermic needle the barrel *is* the whole needle, so every
    legacy number and every serialized key is unchanged. A pulled capillary
    adds a narrow tip stage on the end of it.

    NAMING RULE — read this before adding a call site:
      * ``id_*`` / ``od_*`` / ``wall_*`` / ``length_mm`` are the **barrel** —
        the dimensions the operator types and every legacy readout prints.
      * ``cross_section_area_mm2`` is the **ORIFICE** area (the pulled tip when
        present, else the barrel). Every consumer of it means "the bore the
        material passes through", so this is the fail-safe default; anything
        that genuinely wants the barrel says ``barrel_area_mm2``.
      * ``orifice_*`` are explicit aliases for readable new code.
    """
    gauge: int | None = None            # 16–32+ for a hypodermic; None for a capillary
    od_um: float = 0.0                  # Barrel outer diameter (µm)
    id_um: float = 0.0                  # Barrel inner diameter (µm)
    wall_um: float = 0.0                # Barrel wall thickness (µm)
    length_inches: float = 1.0          # Barrel length (user-selected in GUI)
    num_channels: int = 1               # bore COUNT; == len(bores_resolved())
    # ⚠ DEPRECATED, WRITE-ONLY, 1-BASED, NEVER READ (v7.9).
    # Zero readers repo-wide. It is still emitted verbatim because four
    # byte-identity tests assert the exact legacy key set, and because it exists
    # in every saved setup on disk. Its index base CONTRADICTS the live 0-based
    # HardwareConfig.needle_channel_pump_map. Use NeedleBore.pump_id instead —
    # bores are a LIST, so position IS the index and there is no base to get
    # wrong. Do not add a reader.
    channel_pump_map: dict | None = None  # e.g. {1: "P1", 2: "P2"}

    # --- Pulled glass capillary (v7.6; all absent for a hypodermic) ---
    needle_type: str = NEEDLE_TYPE_HYPODERMIC
    tip_id_um: float | None = None      # D2 — orifice inner Ø (µm)
    tip_length_mm: float | None = None  # L2 — pulled length (mm)
    tip_od_um: float | None = None      # optional; falls back to the barrel OD/ID ratio
    tip_profile: str = TIP_PROFILE_CYLINDER
    needle_type_id: str | None = None   # preset provenance only; never resolved at load
    label: str = ""                     # operator display name

    # --- Multi-bore assembly (v7.9; both absent for every pre-v7.9 needle) ---
    # ``needle_form`` is the assembly's form factor and is ORTHOGONAL to
    # ``needle_type`` (one bore's taper). ``bores`` is the authority when
    # present: bore 0 MIRRORS the flat fields above, so every legacy reader of
    # ``id_um``/``cross_section_area_mm2``/… keeps getting bore 0's real number.
    # ``None`` ⇒ synthesized from the flat fields, so ``bores_resolved()`` always
    # returns at least one real bore and no consumer needs a None branch.
    needle_form: str = NEEDLE_FORM_SINGLE
    bores: list[NeedleBore] | None = None

    def __post_init__(self):
        if self.channel_pump_map is None:
            self.channel_pump_map = {1: "P1"}

        nf = (self.needle_form or "").strip().lower() or NEEDLE_FORM_SINGLE
        self.needle_form = nf if nf in NEEDLE_FORMS else NEEDLE_FORM_SINGLE

        # Tolerate a list of plain dicts (a config loaded before from_dict ran,
        # or a hand-edited file) so callers never have to pre-convert.
        if self.bores is not None:
            coerced = []
            for b in self.bores:
                if isinstance(b, NeedleBore):
                    coerced.append(b)
                elif isinstance(b, dict):
                    coerced.append(NeedleBore.from_dict(b))
            self.bores = coerced or None

        if self.bores:
            # bores is authoritative → mirror bore 0 back onto the flat fields so
            # the ~30 legacy readers stay truthful, and reconcile the count.
            b0 = self.bores[0]
            self.gauge = b0.gauge if b0.gauge is not None else self.gauge
            self.od_um = b0.od_um
            self.id_um = b0.id_um
            self.wall_um = b0.wall_um
            self.length_inches = (float(b0.length_mm) / 25.4) if b0.length_mm else 0.0
            self.needle_type = b0.needle_type
            self.tip_id_um = b0.tip_id_um
            self.tip_length_mm = b0.tip_length_mm
            self.tip_od_um = b0.tip_od_um
            self.tip_profile = b0.tip_profile
            self.num_channels = len(self.bores)
            # Bore 0 IS the needle_origin_um datum, by definition.
            self.bores[0].offset_um = (0.0, 0.0)
            self.bores[0].z_offset_mm = 0.0

        # Normalize the enums to known values so downstream branches are total.
        nt = (self.needle_type or "").strip().lower() or NEEDLE_TYPE_HYPODERMIC
        self.needle_type = nt if nt in NEEDLE_TYPES else NEEDLE_TYPE_HYPODERMIC
        tp = (self.tip_profile or "").strip().lower() or TIP_PROFILE_CYLINDER
        self.tip_profile = tp if tp in TIP_PROFILES else TIP_PROFILE_CYLINDER

        # Collapse unusable tip dimensions to None so `has_tip` is one clean
        # predicate rather than a >0 check repeated at every call site.
        for name in ("tip_id_um", "tip_length_mm", "tip_od_um"):
            v = getattr(self, name)
            if v is None:
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                v = None
            setattr(self, name, v if (v is not None and v > 0.0) else None)

        # A tip wider than the barrel is physically odd but the maths is
        # well-defined either way, so warn rather than silently discarding
        # what the operator entered.
        if self.has_tip and self.id_um > 0 and self.tip_id_um > self.id_um:
            logger.warning(
                "NeedleSpec: tip ID %.1f µm exceeds barrel ID %.1f µm — a pulled "
                "tip should be narrower. Keeping the values as entered.",
                self.tip_id_um, self.id_um)

    # --- predicates ------------------------------------------------------
    @property
    def has_tip(self) -> bool:
        """True when a second (pulled tip) flow stage is defined."""
        return self.tip_id_um is not None and self.tip_length_mm is not None

    @property
    def is_capillary(self) -> bool:
        return self.needle_type == NEEDLE_TYPE_CAPILLARY

    # --- bores (v7.9) -----------------------------------------------------
    @property
    def is_multi_bore(self) -> bool:
        return len(self.bores_resolved()) > 1

    def bores_resolved(self) -> list[NeedleBore]:
        """This assembly's bores — ALWAYS at least one, never None.

        With ``bores`` set, that list is returned verbatim (bore 0 already
        mirrors the flat fields). With ``bores`` absent — every pre-v7.9 needle —
        ``num_channels`` identical bores are synthesized from the flat fields,
        which is faithful to the legacy meaning of ``num_channels`` (a bare count
        with all bores implicitly identical, since the GUI built every one of
        them from ONE catalog gauge).

        Callers never need a None branch, and a single-bore needle gets exactly
        one bore whose geometry IS the needle's.
        """
        if self.bores:
            return list(self.bores)
        n = max(1, int(self.num_channels or 1))
        proto = NeedleBore(
            id_um=self.id_um,
            od_um=self.od_um,
            wall_um=self.wall_um,
            length_mm=self.length_mm,
            gauge=self.gauge,
            needle_type=self.needle_type,
            tip_id_um=self.tip_id_um,
            tip_length_mm=self.tip_length_mm,
            tip_od_um=self.tip_od_um,
            tip_profile=self.tip_profile,
            label=self.label,
        )
        if n == 1:
            return [proto]
        return [_dc_replace(proto) for _ in range(n)]

    @property
    def bore_count(self) -> int:
        return len(self.bores_resolved())

    def bore(self, bore_index: int = 0) -> NeedleBore:
        """Bore ``bore_index`` (0-based, displayed as "Bore N+1").

        Out-of-range clamps to a real bore rather than raising — a stale index
        from a saved workflow profile must not abort a run, and clamping to bore
        0 is the fail-safe direction (bore 0 is the calibrated datum).
        """
        bores = self.bores_resolved()
        try:
            k = int(bore_index)
        except (TypeError, ValueError):
            k = 0
        if k < 0 or k >= len(bores):
            logger.debug("NeedleSpec.bore: index %r out of range (have %d) — using bore 0",
                         bore_index, len(bores))
            k = 0
        return bores[k]

    def bore_for_pump(self, pump_id: str) -> NeedleBore | None:
        """The bore fed by ``pump_id``, or None when no bore claims it.

        This is the lookup the per-PUMP flow ceiling needs: each bore has its own
        pump, so each pump gets its own ceiling from its own bore's geometry.
        """
        if not pump_id:
            return None
        want = str(pump_id).strip().upper()
        for b in self.bores_resolved():
            if b.pump_id and str(b.pump_id).strip().upper() == want:
                return b
        return None

    def bore_index_for_pump(self, pump_id: str) -> int | None:
        if not pump_id:
            return None
        want = str(pump_id).strip().upper()
        for k, b in enumerate(self.bores_resolved()):
            if b.pump_id and str(b.pump_id).strip().upper() == want:
                return k
        return None

    def bore_offset_um(self, bore_index: int = 0) -> tuple[float, float]:
        """Lateral offset of bore ``bore_index`` from bore 0 (the datum).

        To place this bore on a target: ``stage_xy = target_xy - offset``.
        """
        return tuple(self.bore(bore_index).offset_um)

    @property
    def assembly_internal_volume_uL(self) -> float:
        """TOTAL fluid held by EVERY bore (µL).

        Deliberately separate from :attr:`internal_volume_uL`, which resolves
        bore 0 — "1 needle's worth" for prep is per bore, and summing it would
        silently inflate every prep multiple on a multi-bore assembly.
        """
        return sum(b.internal_volume_uL for b in self.bores_resolved())

    @property
    def max_bore_z_offset_mm(self) -> float:
        """The largest ``z_offset_mm`` across the assembly (mm, + = reaches lower).

        A descend must be planned against the LONGEST bore, or a bore that
        protrudes further is driven into the glass while the datum bore sits at
        its nominal clearance.
        """
        return max((b.z_offset_mm for b in self.bores_resolved()), default=0.0)

    # --- barrel dimensions (meaning unchanged since v7.2) ----------------
    @property
    def length_mm(self) -> float:
        """Barrel length in millimeters (the whole needle when there is no tip)."""
        return self.length_inches * 25.4

    @property
    def id_mm(self) -> float:
        """Barrel inner diameter in millimeters."""
        return self.id_um / 1000.0

    @property
    def od_mm(self) -> float:
        """Barrel outer diameter in millimeters."""
        return self.od_um / 1000.0

    @property
    def wall_mm(self) -> float:
        """Barrel wall thickness in millimeters."""
        return self.wall_um / 1000.0

    @property
    def id_m(self) -> float:
        """Barrel inner diameter in meters (for flow physics)."""
        return self.id_um * 1e-6

    @property
    def barrel_area_mm2(self) -> float:
        """Barrel inner cross-sectional area in mm²."""
        return math.pi * (self.id_mm / 2) ** 2

    @property
    def barrel_length_mm(self) -> float:
        return self.length_mm

    @property
    def barrel_od_um(self) -> float:
        return self.od_um

    # --- orifice: the hole material actually leaves through ---------------
    @property
    def orifice_id_um(self) -> float:
        """Inner Ø at the exit (µm). == ``id_um`` for a straight needle."""
        return self.tip_id_um if self.has_tip else self.id_um

    @property
    def orifice_id_mm(self) -> float:
        return self.orifice_id_um / 1000.0

    @property
    def orifice_id_m(self) -> float:
        return self.orifice_id_um * 1e-6

    @property
    def cross_section_area_mm2(self) -> float:
        """ORIFICE cross-sectional area in mm² — the deposited-bead area of the
        F-1 one-bead model, and the bore area for pick & place volume columns.

        For a straight needle this is the barrel bore, exactly as before v7.6.
        Use :attr:`barrel_area_mm2` when you specifically mean the barrel.
        """
        return math.pi * (self.orifice_id_mm / 2) ** 2

    @property
    def orifice_area_mm2(self) -> float:
        """Explicit alias of :attr:`cross_section_area_mm2`."""
        return self.cross_section_area_mm2

    @property
    def orifice_od_um(self) -> float:
        """Outer Ø at the exit (µm). ``tip_od_um`` when measured; else the
        barrel's OD/ID ratio applied to the tip ID (a pull preserves wall
        proportion reasonably well); else the barrel OD."""
        if self.tip_od_um is not None:
            return self.tip_od_um
        if self.has_tip and self.id_um > 0:
            return self.tip_id_um * (self.od_um / self.id_um)
        return self.od_um

    @property
    def orifice_od_mm(self) -> float:
        return self.orifice_od_um / 1000.0

    # --- lengths + volumes ------------------------------------------------
    @property
    def tip_length_mm_or_zero(self) -> float:
        return float(self.tip_length_mm) if self.has_tip else 0.0

    @property
    def total_length_mm(self) -> float:
        """Barrel + tip (mm). A pulled needle is physically LONGER than its
        barrel, so this — not ``length_mm`` — is what the Z touch-off depends on."""
        return self.length_mm + self.tip_length_mm_or_zero

    @property
    def barrel_volume_uL(self) -> float:
        """Barrel bore column (µL). Identical to the pre-v7.6
        ``internal_volume_uL``."""
        return self.barrel_area_mm2 * self.length_mm

    @property
    def tip_volume_uL(self) -> float:
        """Pulled-tip bore volume (µL); 0.0 with no tip stage."""
        if not self.has_tip:
            return 0.0
        L = float(self.tip_length_mm)
        d2 = self.orifice_id_mm
        if self.tip_profile == TIP_PROFILE_CONE:
            d1 = self.id_mm
            return (math.pi * L / 12.0) * (d1 * d1 + d1 * d2 + d2 * d2)
        return math.pi * (d2 / 2) ** 2 * L

    @property
    def internal_volume_uL(self) -> float:
        """TOTAL bore volume (µL) = barrel + tip. ``1 mm³ == 1 µL``.

        This is "1 needle's worth" of fluid that the pick & place prep multiples
        (waste/oil/buffer/wash) are counted in. Unchanged for a straight needle
        because the tip term is then 0."""
        return self.barrel_volume_uL + self.tip_volume_uL

    @property
    def ink_reserve_volume_uL(self) -> float:
        """Ink kept BEHIND the deposit so a print never reaches the buffer/oil
        plug (the Quick Print pickup reserve).

        For a pulled capillary this is the TIP volume — the ink that actually
        sits in the working section — rather than the whole barrel, which on a
        1 mm blank would exceed a 25 µL syringe. For a straight needle there is
        no tip, so it stays the full bore volume: exactly today's number.

        ⚠ On a fine tip this is a very small number (a 30 µm × 3 mm tip holds
        ~2 pL). Quick Print warns when the resolved reserve falls below one pump
        step so the operator can raise the ink padding instead.
        """
        return self.tip_volume_uL if self.has_tip else self.internal_volume_uL

    # --- segment geometry -------------------------------------------------
    def flow_segments(self) -> list[FlowSegment]:
        """Bore geometry as an ordered list of axial stages, barrel first.

        straight needle  -> [barrel]                one cylinder
        pulled/cylinder  -> [barrel, tip(D2, D2)]
        pulled/cone      -> [barrel, tip(D1, D2)]   linear taper

        THIS IS THE ONLY PLACE THE TAPER MODEL IS EXPRESSED — switching
        ``tip_profile`` changes the physics with no downstream edits.
        """
        segs = [FlowSegment("barrel", self.length_mm, self.id_um, self.id_um)]
        if self.has_tip:
            d_in = self.id_um if self.tip_profile == TIP_PROFILE_CONE else self.tip_id_um
            segs.append(FlowSegment("tip", float(self.tip_length_mm), d_in, self.tip_id_um))
        return segs

    def bore_profile(self) -> BoreProfile:
        """Volume ↔ lift geometry for aspiration (spheroid sink timing)."""
        tip = next((s for s in self.flow_segments() if s.name == "tip"), None)
        return BoreProfile(
            barrel_area_mm2=self.barrel_area_mm2,
            tip_area_mm2=tip.equivalent_area_mm2 if tip else 0.0,
            tip_length_mm=tip.length_mm if tip else 0.0,
        )

    def spheroid_pickup_detail(
        self,
        spheroid_diameter_um: float,
        volume_uL: float | None = None,
        clearance: float = 1.2,
    ) -> dict:
        """Can this needle aspirate a spheroid of this diameter, and will it
        stay inside the calibrated tip?

        Returns the same ``{status, ratio, message, severity}`` shape as
        :meth:`InkSpec.flow_compatibility_detail`. Every severity is ADVISORY —
        a deformable spheroid can squeeze through a slightly smaller orifice, so
        the workflow warns and proceeds rather than blocking.
        """
        d_orifice = self.orifice_id_um
        d_sph = float(spheroid_diameter_um or 0.0)
        if d_sph <= 0 or d_orifice <= 0:
            return {"status": "unknown", "ratio": 0.0, "severity": "info",
                    "message": "Needle bore or spheroid diameter not set."}

        ratio = d_orifice / d_sph
        where = "pulled tip" if self.has_tip else "needle bore"
        if ratio < 1.0:
            return {
                "status": "too_large", "ratio": ratio, "severity": "warning",
                "message": (f"Spheroid {d_sph:.0f} µm is wider than the {where} "
                            f"({d_orifice:.1f} µm) — it may not be aspirated at all."),
            }
        if ratio < clearance:
            return {
                "status": "tight", "ratio": ratio, "severity": "warning",
                "message": (f"Spheroid {d_sph:.0f} µm barely clears the {where} "
                            f"({d_orifice:.1f} µm) — expect wall contact, and the "
                            f"sink curve will not match its calibration."),
            }

        # Does the planned aspirate lift it clean out of the tip?
        if volume_uL is not None and self.has_tip:
            profile = self.bore_profile()
            lift = profile.lift_for_volume(volume_uL)
            if lift > profile.tip_length_mm:
                return {
                    "status": "past_tip", "ratio": ratio, "severity": "warning",
                    "message": (f"The planned {float(volume_uL):.4f} µL lifts the "
                                f"spheroid {lift:.2f} mm — past the {profile.tip_length_mm:.2f} mm "
                                f"tip and into the wide barrel, where the "
                                f"tip-calibrated sink curve no longer applies."),
                }

        return {"status": "ok", "ratio": ratio, "severity": "info",
                "message": f"Spheroid {d_sph:.0f} µm clears the {where} ({ratio:.1f}×)."}

    # --- display ----------------------------------------------------------
    @property
    def display_label(self) -> str:
        """Short identity for logs, readouts and repr — never prints "NoneG"."""
        if self.label:
            return self.label
        if self.is_capillary or self.has_tip:
            return f"Capillary {self.id_um:.0f} µm bore → {self.orifice_id_um:.1f} µm tip"
        return f"{self.gauge}G" if self.gauge else "needle"

    def summary_line(self) -> str:
        """One-line geometry summary shared by every needle readout."""
        if self.is_capillary or self.has_tip:
            parts = [
                "Capillary",
                f"barrel OD {self.od_um:.0f} / ID {self.id_um:.0f} µm",
                f"L {self.length_mm:.1f} mm",
            ]
            if self.has_tip:
                tip_od = f"OD {self.orifice_od_um:.0f} / " if self.tip_od_um else ""
                parts.append(f"tip {tip_od}ID {self.tip_id_um:.1f} µm")
                parts.append(f"L {float(self.tip_length_mm):.2f} mm")
        else:
            parts = [f"{self.gauge}G" if self.gauge else "needle"]
            if self.od_um:
                parts.append(f"OD {self.od_um:.0f} µm")
            if self.id_um:
                parts.append(f"ID {self.id_um:.0f} µm")
            parts.append(f"L {self.length_mm:.1f} mm")
        # v7.9: an assembly reports its FORM and bore count. "channels" is the
        # banned overloaded word (see the vocabulary table in the v7.9 plan) —
        # nothing asserts this string, so it adopts the new vocabulary.
        bores = self.bores_resolved()
        if len(bores) > 1:
            parts.append(f"{self.needle_form} · {len(bores)} bores")
            distinct = {round(b.orifice_id_um, 3) for b in bores}
            if len(distinct) > 1:
                parts.append("ID " + "/".join(
                    f"{b.orifice_id_um:.0f}" for b in bores) + " µm")
        return " · ".join(parts)

    def assembly_summary_lines(self) -> list[str]:
        """One line per bore — for a readout that must show which pump feeds
        which bore, and how far each is offset from the datum."""
        out = []
        for k, b in enumerate(self.bores_resolved()):
            bits = [f"Bore {k + 1}", b.summary_line() or "—"]
            ox, oy = b.offset_um
            if k == 0:
                bits.append("datum")
            elif ox or oy:
                bits.append(f"offset ({ox:+.0f}, {oy:+.0f}) µm")
            else:
                bits.append("offset not measured")
            if b.z_offset_mm:
                bits.append(f"Z {b.z_offset_mm:+.3f} mm")
            out.append(" · ".join(bits))
        return out

    def to_dict(self) -> dict:
        """Serialize to dictionary.

        The seven legacy keys are ALWAYS emitted, in their original order and
        with nothing else, for a needle with no capillary fields — so every
        existing saved setup and workspace round-trips byte-identically.

        v7.9: ``needle_form`` and ``bores`` are conditional-emit — a single-bore
        needle emits NEITHER, so all six real on-disk setups and the four
        byte-identity assertions are untouched. ``bores`` is emitted only when
        the list was explicitly supplied AND holds more than one bore; a
        synthesized single bore carries no information the flat fields lack.
        """
        d = {
            "gauge": self.gauge,
            "od_um": self.od_um,
            "id_um": self.id_um,
            "wall_um": self.wall_um,
            "length_inches": self.length_inches,
            "num_channels": self.num_channels,
            "channel_pump_map": self.channel_pump_map,
        }
        if self.needle_type != NEEDLE_TYPE_HYPODERMIC:
            d["needle_type"] = self.needle_type
        for key in ("tip_id_um", "tip_length_mm", "tip_od_um", "needle_type_id"):
            value = getattr(self, key)
            if value is not None:
                d[key] = value
        if self.tip_profile != TIP_PROFILE_CYLINDER:
            d["tip_profile"] = self.tip_profile
        if self.label:
            d["label"] = self.label
        if self.needle_form != NEEDLE_FORM_SINGLE:
            d["needle_form"] = self.needle_form
        if self.bores and len(self.bores) > 1:
            d["bores"] = [b.to_dict() for b in self.bores]
        return d

    @classmethod
    def from_dict(cls, data: dict) -> NeedleSpec:
        """Deserialize from dictionary, ignoring unknown keys.

        The pre-v7.6 ``cls(**data)`` raised TypeError on any unrecognised key,
        which meant a config written by a newer build silently destroyed the
        whole HardwareConfig load (it is caught by a broad ``except`` in
        ``gui/app.py``). Filtering keeps old builds forward-compatible.
        """
        data = dict(data or {})
        known = {f.name for f in _dc_fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            logger.debug("NeedleSpec.from_dict: ignoring unknown key(s) %s", unknown)
        kwargs = {k: v for k, v in data.items() if k in known}
        raw_bores = kwargs.get("bores")
        if raw_bores:
            # __post_init__ also tolerates dicts, but converting here keeps the
            # constructed object fully typed even for a direct cls(**data) path.
            kwargs["bores"] = [
                b if isinstance(b, NeedleBore) else NeedleBore.from_dict(b)
                for b in raw_bores if isinstance(b, (NeedleBore, dict))
            ] or None
        return cls(**kwargs)


# ---------------------------------------------------------------------------
# Needle geometry accessors (duck-typing tolerant)
# ---------------------------------------------------------------------------
# Several call sites (and a number of tests) pass needle-LIKE objects that
# expose only a couple of attributes — e.g. a stub with just gauge/id_m/
# length_mm, or a MagicMock. Physics and geometry consumers must therefore go
# through these free functions rather than touching properties directly.

def _needle_num(needle, names: tuple[str, ...]) -> float | None:
    """First attribute in ``names`` that reads back as a real positive number."""
    for name in names:
        try:
            value = getattr(needle, name, None)
        except Exception:
            continue
        if value is None or isinstance(value, bool):
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0:
            return value
    return None


def needle_orifice_id_um(needle) -> float:
    """Inner Ø at the exit (µm) for any needle-like object."""
    value = _needle_num(needle, ("orifice_id_um", "tip_id_um", "id_um"))
    if value is not None:
        return value
    id_m = _needle_num(needle, ("id_m",))
    return id_m * 1e6 if id_m else 0.0


def needle_orifice_id_m(needle) -> float:
    """Inner Ø at the exit (m) for any needle-like object."""
    return needle_orifice_id_um(needle) * 1e-6


def needle_orifice_area_mm2(needle) -> float:
    """Deposited-bead / bore-column cross-section (mm²) for any needle-like
    object — the pulled tip when present, else the barrel bore."""
    value = _needle_num(needle, ("orifice_area_mm2", "cross_section_area_mm2"))
    if value is not None:
        return value
    d_mm = needle_orifice_id_um(needle) / 1000.0
    return math.pi * (d_mm / 2) ** 2 if d_mm > 0 else 0.0


def needle_orifice_od_mm(needle) -> float:
    """Outer Ø at the exit (mm) — the line-spacing / bead-width reference."""
    value = _needle_num(needle, ("orifice_od_mm", "od_mm"))
    if value is not None:
        return value
    od_um = _needle_num(needle, ("orifice_od_um", "tip_od_um", "od_um"))
    return od_um / 1000.0 if od_um else 0.0


def needle_flow_segments(needle) -> list[FlowSegment]:
    """Bore geometry for any needle-like object. Prefers ``flow_segments()``;
    falls back to a single barrel cylinder built from whatever diameter and
    length attributes exist."""
    fn = getattr(needle, "flow_segments", None)
    if callable(fn):
        try:
            segs = list(fn())
            if segs:
                return segs
        except Exception:
            logger.debug("needle_flow_segments: flow_segments() failed", exc_info=True)
    id_um = _needle_num(needle, ("id_um",))
    if id_um is None:
        id_m = _needle_num(needle, ("id_m",))
        id_um = id_m * 1e6 if id_m else 0.0
    length_mm = _needle_num(needle, ("length_mm",)) or 0.0
    return [FlowSegment("barrel", length_mm, id_um or 0.0, id_um or 0.0)]


def needle_bore_profile(needle) -> BoreProfile:
    """Volume ↔ lift geometry for any needle-like object."""
    fn = getattr(needle, "bore_profile", None)
    if callable(fn):
        try:
            profile = fn()
            if isinstance(profile, BoreProfile):
                return profile
        except Exception:
            logger.debug("needle_bore_profile: bore_profile() failed", exc_info=True)
    return BoreProfile.from_area(needle_orifice_area_mm2(needle))


def needle_particle_ratio(needle, particle_d_um: float) -> float:
    """Orifice-to-particle ratio — the single clogging metric.

    Uses the needle's ORIFICE Ø (the pulled tip when present) because that is
    the constriction a cell or granule actually jams in.
    """
    if particle_d_um is None or particle_d_um <= 0:
        return float("inf")
    return needle_orifice_id_um(needle) / particle_d_um


# ---------------------------------------------------------------------------
# Per-bore accessors (v7.9, duck-typing tolerant)
# ---------------------------------------------------------------------------
# The single-valued functions above stay single-valued ON PURPOSE — ~30 call
# sites pass MagicMocks and minimal stubs, and every one of them means "the bore
# the material passes through", which resolves to BORE 0 (see the plan's
# discussion of the v7.6 fail-safe redefinition). These indexed siblings are how
# genuinely multi-bore code becomes explicit.
#
# A NeedleBore is itself duck-compatible with every function above (it exposes
# orifice_area_mm2 / cross_section_area_mm2 / flow_segments() / bore_profile() /
# id_um / od_mm / …), so `needle_orifice_area_mm2(needle_bore_at(n, k))` is the
# general pattern and no function needed duplicating.

def needle_bore_count(needle) -> int:
    """How many bores this needle-like object has. 1 for anything that cannot
    say — a stub or MagicMock is treated as a single bore, never as unknown."""
    fn = getattr(needle, "bore_count", None)
    if isinstance(fn, int):
        return max(1, fn)
    try:
        bores = needle.bores_resolved()
    except Exception:
        bores = None
    if bores:
        return max(1, len(bores))
    n = _needle_num(needle, ("num_channels",))
    return max(1, int(n)) if n else 1


def needle_bore_at(needle, bore_index: int = 0):
    """The bore-like object for ``bore_index``, or ``needle`` itself.

    Returning the needle for a single-bore/stub object is what lets every
    existing single-valued accessor be reused unchanged: for a single-bore
    needle the assembly IS bore 0.
    """
    fn = getattr(needle, "bore", None)
    if callable(fn):
        try:
            bore = fn(bore_index)
            if bore is not None:
                return bore
        except Exception:
            logger.debug("needle_bore_at: bore(%r) failed", bore_index, exc_info=True)
    return needle


def needle_bore_for_pump(needle, pump_id: str):
    """The bore fed by ``pump_id``, or None. Used by the per-PUMP flow ceiling."""
    fn = getattr(needle, "bore_for_pump", None)
    if callable(fn):
        try:
            return fn(pump_id)
        except Exception:
            logger.debug("needle_bore_for_pump: failed for %r", pump_id, exc_info=True)
    return None


def needle_bore_offset_um(needle, bore_index: int = 0) -> tuple[float, float]:
    """Lateral offset (µm) of a bore from bore 0.

    ``stage_xy = target_xy - offset`` places that bore on the target. Returns
    (0, 0) for anything that cannot say, which is exactly the single-bore
    behaviour — so the offset-aware motion path is a no-op on every existing
    needle and can be enabled unconditionally.
    """
    fn = getattr(needle, "bore_offset_um", None)
    if callable(fn):
        try:
            ox, oy = fn(bore_index)
            return (float(ox), float(oy))
        except Exception:
            logger.debug("needle_bore_offset_um: failed for %r", bore_index, exc_info=True)
    bore = needle_bore_at(needle, bore_index)
    try:
        ox, oy = getattr(bore, "offset_um", (0.0, 0.0))
        return (float(ox), float(oy))
    except (TypeError, ValueError):
        return (0.0, 0.0)


def needle_bore_z_offset_mm(needle, bore_index: int = 0) -> float:
    """Axial offset (mm, + = reaches LOWER than bore 0) of one bore.

    ⚠ Requires a REAL number, not merely something ``float()`` accepts. A
    ``MagicMock`` implements ``__float__`` and returns **1.0**, so a duck-typed
    coercion here would make every mock/stub needle claim its bore reaches 1 mm
    lower than the datum — which ``_bore_z_mm`` would then apply as a 1 mm shift
    to a descend planned 0.1 mm off the glass. A real ``NeedleBore`` always
    stores a float, so the strict check costs nothing and the unknown case
    correctly degrades to "no offset".
    """
    bore = needle_bore_at(needle, bore_index)
    v = getattr(bore, "z_offset_mm", 0.0)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return 0.0
    v = float(v)
    return v if math.isfinite(v) else 0.0


def needle_max_bore_z_offset_mm(needle) -> float:
    """Largest z_offset across the assembly — what a DESCEND must be planned
    against so a longer bore is not driven into the glass."""
    v = getattr(needle, "max_bore_z_offset_mm", None)
    if isinstance(v, (int, float)):
        return float(v)
    return max((needle_bore_z_offset_mm(needle, k)
                for k in range(needle_bore_count(needle))), default=0.0)


def needle_bore_internal_volume_uL(needle, bore_index: int = 0) -> float:
    """"One bore's worth" of fluid (µL) for a specific bore.

    Prep multiples are counted in this, and it is PER BORE — a backpack's two
    bores hold different volumes, so the legacy scalar cannot size both.
    """
    bore = needle_bore_at(needle, bore_index)
    v = _needle_num(bore, ("internal_volume_uL",))
    if v is not None:
        return v
    segs = needle_flow_segments(bore)
    return sum(s.volume_uL for s in segs)


def default_fallback_needle() -> NeedleSpec:
    """The 22G stand-in used when no needle is configured.

    Deliberately a hypodermic: a silent capillary default would under-extrude by
    ~190× and read as a hardware fault rather than a configuration gap.
    """
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)


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
            # via from_dict so the catalog tolerates keys a newer build added
            catalog[gauge] = NeedleSpec.from_dict({**dims, "gauge": gauge})
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

        Uses the needle's ORIFICE Ø (the pulled tip when present) — a particle
        jams at the constriction, not in the bulk barrel.
        """
        particle_d = self.max_particle_diameter_um
        if particle_d <= 0:
            return "compatible"

        ratio = needle_particle_ratio(needle, particle_d)
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

        Uses the needle's ORIFICE Ø (the pulled tip when present).
        """
        label = getattr(needle, "display_label", None) or f"{getattr(needle, 'gauge', None)}G"
        particle_d = self.max_particle_diameter_um
        if particle_d <= 0:
            return {
                "status": "compatible",
                "ratio": float("inf"),
                "message": f"{self.name} is a pure liquid — flows freely through {label}",
                "severity": "ok",
            }

        ratio = needle_particle_ratio(needle, particle_d)
        if ratio >= 4.0:
            return {
                "status": "free_flow",
                "ratio": round(ratio, 1),
                "message": (f"{self.name} flows freely through {label} "
                           f"(ID/particle = {ratio:.1f}×)"),
                "severity": "ok",
            }
        elif ratio >= 1.0:
            return {
                "status": "risk_clogging",
                "ratio": round(ratio, 1),
                "message": (f"⚠️ {self.name} near jamming limit for {label} "
                           f"(ID/particle = {ratio:.1f}×, need >4×)"),
                "severity": "warning",
            }
        else:
            return {
                "status": "pick_and_place",
                "ratio": round(ratio, 1),
                "message": (f"{self.name} too large for {label} "
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
