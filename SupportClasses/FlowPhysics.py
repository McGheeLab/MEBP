"""
FlowPhysics.py — Flow physics calculations and safety thresholds for MEBP v7.1.

Provides:
- Hagen-Poiseuille pressure drop calculation for laminar needle flow
- Reynolds number computation and flow regime classification
- Granular/cell flow regime classification (free-flow, clogging, pick-and-place)
- Cell shear stress calculation for viability assessment
- Integrated flow safety result with warnings and limits
- Maximum safe flow rate computation for any needle/syringe/ink combination

All calculations use SI units internally and convert to user-friendly units
(µL/s, kPa, etc.) at the interface boundaries.

References:
- Hagen-Poiseuille: ΔP = (128 × µ × L × Q) / (π × d⁴)
- Reynolds: Re = (ρ × v × d) / µ
- Wall shear stress: τ = (32 × µ × Q) / (π × d³)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum

# Import physical models — these are the foundation layer
from .PhysicalModels import NeedleSpec, SyringeSpec, InkSpec

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default safety thresholds
DEFAULT_PRESSURE_LIMIT_PA = 200_000     # 200 kPa — typical syringe pressure limit
DEFAULT_SHEAR_STRESS_LIMIT_PA = 5.0     # 5 Pa — typical max for cell viability
MIN_FLOW_RATE_UL_S = 0.001             # Minimum meaningful flow rate


# ---------------------------------------------------------------------------
# Flow Regime Enum
# ---------------------------------------------------------------------------

class FlowRegime(Enum):
    """Flow regime classification based on Reynolds number."""
    LAMINAR = "laminar"           # Re < 2100 — Hagen-Poiseuille valid
    TRANSITIONAL = "transitional"  # 2100 ≤ Re < 4000
    TURBULENT = "turbulent"       # Re ≥ 4000 — need different model


class GranularRegime(Enum):
    """Flow regime for granular/cell suspensions through needle."""
    NO_PARTICLES = "no_particles"     # Pure liquid, no particle concerns
    FREE_FLOW = "free_flow"           # needle_ID > 4× particle diameter
    INTERMITTENT = "intermittent"     # 1× < needle_ID < 4× particle diameter
    JAMMING = "jamming"               # needle_ID < 1× particle diameter (pick-and-place)


# ---------------------------------------------------------------------------
# Result Dataclass
# ---------------------------------------------------------------------------

@dataclass
class FlowSafetyResult:
    """
    Complete result of flow safety calculation.

    Aggregates pressure, Reynolds, granular, and shear stress checks
    into a single result with clear is_safe determination and warnings.
    """
    # Flow rate being evaluated
    requested_flow_rate_uL_s: float = 0.0

    # Pressure analysis
    pressure_at_requested_rate_Pa: float = 0.0
    pressure_limit_Pa: float = DEFAULT_PRESSURE_LIMIT_PA
    max_safe_flow_rate_uL_s: float = 0.0

    # Reynolds number
    reynolds_number: float = 0.0
    flow_regime: FlowRegime = FlowRegime.LAMINAR

    # Granular/cell flow
    granular_regime: GranularRegime = GranularRegime.NO_PARTICLES
    particle_to_needle_ratio: float = 0.0

    # Cell shear stress
    wall_shear_stress_Pa: float = 0.0
    shear_stress_limit_Pa: float = DEFAULT_SHEAR_STRESS_LIMIT_PA
    shear_safe: bool = True

    # Overall
    is_safe: bool = True
    warnings: list[str] = field(default_factory=list)

    @property
    def pressure_kPa(self) -> float:
        """Pressure at requested rate in kPa."""
        return self.pressure_at_requested_rate_Pa / 1000.0

    @property
    def pressure_limit_kPa(self) -> float:
        """Pressure limit in kPa."""
        return self.pressure_limit_Pa / 1000.0

    @property
    def max_safe_flow_rate_uL_min(self) -> float:
        """Max safe flow rate in µL/min."""
        return self.max_safe_flow_rate_uL_s * 60.0

    @property
    def pressure_utilization(self) -> float:
        """Fraction of pressure limit used (0.0–1.0+)."""
        if self.pressure_limit_Pa <= 0:
            return 0.0
        return self.pressure_at_requested_rate_Pa / self.pressure_limit_Pa

    def summary(self) -> str:
        """Human-readable summary for GUI display."""
        lines = []
        status = "✅ SAFE" if self.is_safe else "❌ UNSAFE"
        lines.append(f"Flow Safety: {status}")
        lines.append(f"  Requested rate: {self.requested_flow_rate_uL_s:.3f} µL/s")
        lines.append(f"  Max safe rate: {self.max_safe_flow_rate_uL_s:.3f} µL/s")
        lines.append(f"  Pressure: {self.pressure_kPa:.1f} / {self.pressure_limit_kPa:.1f} kPa "
                     f"({self.pressure_utilization:.0%})")
        lines.append(f"  Reynolds: {self.reynolds_number:.1f} ({self.flow_regime.value})")
        if self.granular_regime != GranularRegime.NO_PARTICLES:
            lines.append(f"  Particle regime: {self.granular_regime.value} "
                        f"(ratio: {self.particle_to_needle_ratio:.1f}×)")
        if self.wall_shear_stress_Pa > 0:
            lines.append(f"  Wall shear: {self.wall_shear_stress_Pa:.2f} Pa "
                        f"({'✅' if self.shear_safe else '⚠️'} limit: {self.shear_stress_limit_Pa} Pa)")
        for w in self.warnings:
            lines.append(f"  ⚠️ {w}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Core Calculations
# ---------------------------------------------------------------------------

def hagen_poiseuille_pressure(
    viscosity_Pa_s: float,
    length_m: float,
    flow_rate_m3_s: float,
    diameter_m: float,
) -> float:
    """
    Calculate pressure drop through a cylindrical tube (Hagen-Poiseuille).

    ΔP = (128 × µ × L × Q) / (π × d⁴)

    Valid for laminar flow (Re < 2100) in a straight tube.

    Args:
        viscosity_Pa_s: Dynamic viscosity (Pa·s)
        length_m: Tube length (m)
        flow_rate_m3_s: Volumetric flow rate (m³/s)
        diameter_m: Inner diameter (m)

    Returns:
        Pressure drop in Pa
    """
    if diameter_m <= 0:
        return float("inf")
    return (128 * viscosity_Pa_s * length_m * flow_rate_m3_s) / (math.pi * diameter_m ** 4)


def reynolds_number(
    density_kg_m3: float,
    velocity_m_s: float,
    diameter_m: float,
    viscosity_Pa_s: float,
) -> float:
    """
    Calculate Reynolds number.

    Re = (ρ × v × d) / µ

    Args:
        density_kg_m3: Fluid density (kg/m³)
        velocity_m_s: Mean flow velocity (m/s)
        diameter_m: Inner diameter (m)
        viscosity_Pa_s: Dynamic viscosity (Pa·s)

    Returns:
        Dimensionless Reynolds number
    """
    if viscosity_Pa_s <= 0:
        return float("inf")
    return (density_kg_m3 * velocity_m_s * diameter_m) / viscosity_Pa_s


def classify_flow_regime(re: float) -> FlowRegime:
    """Classify flow regime from Reynolds number."""
    if re < 2100:
        return FlowRegime.LAMINAR
    elif re < 4000:
        return FlowRegime.TRANSITIONAL
    else:
        return FlowRegime.TURBULENT


def wall_shear_stress(
    viscosity_Pa_s: float,
    flow_rate_m3_s: float,
    diameter_m: float,
) -> float:
    """
    Calculate wall shear stress in a cylindrical tube.

    τ_wall = (32 × µ × Q) / (π × d³)

    Critical for cell viability assessment — high shear damages cells.

    Args:
        viscosity_Pa_s: Dynamic viscosity (Pa·s)
        flow_rate_m3_s: Volumetric flow rate (m³/s)
        diameter_m: Inner diameter (m)

    Returns:
        Wall shear stress in Pa
    """
    if diameter_m <= 0:
        return float("inf")
    return (32 * viscosity_Pa_s * flow_rate_m3_s) / (math.pi * diameter_m ** 3)


def classify_granular_regime(needle: NeedleSpec, ink: InkSpec) -> tuple[GranularRegime, float]:
    """
    Classify granular/cell flow regime through needle.

    Uses the ratio of needle inner diameter to maximum particle diameter:
    - No particles: pure liquid
    - Free flow: needle_ID > 4× particle diameter
    - Intermittent: 1× < needle_ID < 4× particle diameter (risk of clogging)
    - Jamming: needle_ID < 1× particle diameter (pick-and-place only)

    Returns:
        (regime, ratio) where ratio = needle_ID / particle_diameter
    """
    particle_d = ink.max_particle_diameter_um
    if particle_d <= 0:
        return GranularRegime.NO_PARTICLES, 0.0

    ratio = needle.id_um / particle_d

    if ratio >= 4.0:
        return GranularRegime.FREE_FLOW, ratio
    elif ratio >= 1.0:
        return GranularRegime.INTERMITTENT, ratio
    else:
        return GranularRegime.JAMMING, ratio


def max_safe_flow_rate_uL_s(
    needle: NeedleSpec,
    ink: InkSpec,
    pressure_limit_Pa: float = DEFAULT_PRESSURE_LIMIT_PA,
) -> float:
    """
    Calculate maximum flow rate that stays below pressure limit.

    Rearranging Hagen-Poiseuille:
    Q_max = (ΔP_max × π × d⁴) / (128 × µ × L)

    Returns:
        Maximum safe flow rate in µL/s
    """
    d = needle.id_m
    if d <= 0 or ink.viscosity_Pa_s <= 0:
        return 0.0

    length_m = needle.length_mm / 1000.0
    q_max_m3_s = (pressure_limit_Pa * math.pi * d ** 4) / (128 * ink.viscosity_Pa_s * length_m)

    # Convert m³/s → µL/s (1 m³ = 1e9 µL)
    return q_max_m3_s * 1e9


# ---------------------------------------------------------------------------
# Pump Speed Calculations
# ---------------------------------------------------------------------------

def flow_rate_to_pump_speed(
    flow_rate_uL_s: float,
    syringe: SyringeSpec,
) -> float:
    """
    Convert volumetric flow rate to pump plunger speed.

    Args:
        flow_rate_uL_s: Desired flow rate (µL/s)
        syringe: Syringe specification

    Returns:
        Plunger speed in mm/s
    """
    return flow_rate_uL_s * syringe.mm_per_uL


def pump_speed_to_flow_rate(
    pump_speed_mm_s: float,
    syringe: SyringeSpec,
) -> float:
    """
    Convert pump plunger speed to volumetric flow rate.

    Args:
        pump_speed_mm_s: Plunger speed (mm/s)
        syringe: Syringe specification

    Returns:
        Flow rate in µL/s
    """
    return pump_speed_mm_s * syringe.uL_per_mm


# ---------------------------------------------------------------------------
# Extrusion Rate for Print Objects
# ---------------------------------------------------------------------------

def extrusion_flow_rate(
    print_speed_mm_s: float,
    needle: NeedleSpec,
    layer_height_mm: float = 0.0,   # v7.5.x F-1: retained for signature compat; UNUSED
    extrusion_modifier: float = 1.0,
) -> float:
    """
    Calculate required extrusion flow rate from the deposited bead geometry.

    v7.5.x (F-1) — ONE bead model everywhere: the deposited bead is a stream the
    size of the needle's **inner bore**, optionally scaled by an extrusion
    ``modifier`` (the operator's "line thickness" lever). This matches Quick
    Print's auto-flow (``bore_area × modifier``); it replaces the legacy
    ``outer-Ø × layer_height`` rectangular approximation. ``layer_height`` no
    longer appears in the volume formula — it is now only the Z step between
    stacked layers (see ``compute_layer_heights``).

    Volume conservation: volume in (from needle) = volume deposited.
      Deposited cross-section = π·(id/2)² × modifier  (mm²)
      Volume per mm of travel  = cross-section            (µL/mm, since mm² ≡ µL/mm)
      Required flow rate       = travel_speed × cross-section × modifier  (µL/s)

    Args:
        print_speed_mm_s: Linear travel speed (mm/s)
        needle: Needle specification (uses the INNER bore)
        layer_height_mm: Deprecated — ignored (kept so older positional callers
            don't break); pass the modifier instead.
        extrusion_modifier: Bead thickness multiplier (1.0 = a pure bore-sized
            stream).

    Returns:
        Required flow rate in µL/s (1 mm³ = 1 µL)
    """
    return print_speed_mm_s * needle.cross_section_area_mm2 * extrusion_modifier


def extrusion_pump_speed(
    print_speed_mm_s: float,
    needle: NeedleSpec,
    syringe: SyringeSpec,
    layer_height_mm: float,
) -> float:
    """
    Calculate pump plunger speed for a given print speed.

    Combines extrusion_flow_rate with flow_rate_to_pump_speed.

    Returns:
        Pump plunger speed in mm/s
    """
    flow = extrusion_flow_rate(print_speed_mm_s, needle, layer_height_mm)
    return flow_rate_to_pump_speed(flow, syringe)


# ---------------------------------------------------------------------------
# Main Safety Calculator
# ---------------------------------------------------------------------------

def calculate_flow_safety(
    needle: NeedleSpec,
    syringe: SyringeSpec,
    ink: InkSpec,
    requested_flow_rate_uL_s: float,
    pressure_limit_Pa: float = DEFAULT_PRESSURE_LIMIT_PA,
    shear_stress_limit_Pa: float = DEFAULT_SHEAR_STRESS_LIMIT_PA,
) -> FlowSafetyResult:
    """
    Calculate whether a flow rate is safe for the given hardware combination.

    Performs all safety checks:
    1. Hagen-Poiseuille pressure drop vs. pressure limit
    2. Reynolds number and flow regime
    3. Granular/cell clogging regime
    4. Wall shear stress for cell viability

    Args:
        needle: Needle specification
        syringe: Syringe specification
        ink: Ink specification
        requested_flow_rate_uL_s: Desired flow rate (µL/s)
        pressure_limit_Pa: Maximum allowable pressure (Pa)
        shear_stress_limit_Pa: Maximum wall shear stress for cells (Pa)

    Returns:
        FlowSafetyResult with all computed values and safety determination
    """
    result = FlowSafetyResult(
        requested_flow_rate_uL_s=requested_flow_rate_uL_s,
        pressure_limit_Pa=pressure_limit_Pa,
        shear_stress_limit_Pa=shear_stress_limit_Pa,
    )
    warnings = []

    # --- Unit conversions to SI ---
    d_m = needle.id_m                                 # diameter (m)
    L_m = needle.length_mm / 1000.0                    # length (m)
    mu = ink.viscosity_Pa_s                            # viscosity (Pa·s)
    rho = ink.density_g_mL * 1000.0                    # density (kg/m³)
    Q_m3_s = requested_flow_rate_uL_s * 1e-9          # flow rate (m³/s)

    # Cross-sectional area of needle bore
    A_m2 = math.pi * (d_m / 2) ** 2                    # m²

    # --- 1. Pressure drop (Hagen-Poiseuille) ---
    if d_m > 0 and mu > 0 and L_m > 0:
        pressure = hagen_poiseuille_pressure(mu, L_m, Q_m3_s, d_m)
        result.pressure_at_requested_rate_Pa = pressure

        if pressure > pressure_limit_Pa:
            result.is_safe = False
            warnings.append(
                f"Pressure {pressure / 1000:.1f} kPa exceeds limit "
                f"{pressure_limit_Pa / 1000:.1f} kPa"
            )
    else:
        result.pressure_at_requested_rate_Pa = 0.0

    # --- Max safe flow rate ---
    result.max_safe_flow_rate_uL_s = max_safe_flow_rate_uL_s(
        needle, ink, pressure_limit_Pa
    )

    # --- 2. Reynolds number ---
    if A_m2 > 0 and mu > 0:
        v_m_s = Q_m3_s / A_m2  # mean velocity (m/s)
        re = reynolds_number(rho, v_m_s, d_m, mu)
        result.reynolds_number = re
        result.flow_regime = classify_flow_regime(re)

        if result.flow_regime == FlowRegime.TRANSITIONAL:
            warnings.append(
                f"Transitional flow (Re={re:.0f}) — Hagen-Poiseuille approximation "
                f"may underestimate pressure"
            )
        elif result.flow_regime == FlowRegime.TURBULENT:
            warnings.append(
                f"Turbulent flow (Re={re:.0f}) — actual pressure significantly "
                f"higher than laminar estimate"
            )
            result.is_safe = False

    # --- 3. Granular/cell flow regime ---
    gran_regime, ratio = classify_granular_regime(needle, ink)
    result.granular_regime = gran_regime
    result.particle_to_needle_ratio = ratio

    if gran_regime == GranularRegime.INTERMITTENT:
        warnings.append(
            f"Particle/needle ratio {ratio:.1f}× (need >4× for free flow) "
            f"— intermittent clogging risk"
        )
    elif gran_regime == GranularRegime.JAMMING:
        warnings.append(
            f"Particle/needle ratio {ratio:.1f}× — particles too large for "
            f"continuous flow, pick-and-place only"
        )

    # --- 4. Cell shear stress ---
    # v7.5.x: cell inks are now ink_type=="ink" with ink_subtype=="cells";
    # the legacy ink_type=="cells" is still honored for directly-constructed
    # (un-migrated) specs.
    _subtype = (getattr(ink, "ink_subtype", "") or "").strip().lower()
    has_cells = (ink.cell_diameter_um > 0
                 or ink.ink_type == "cells"
                 or _subtype == "cells")
    if has_cells and d_m > 0 and mu > 0:
        tau = wall_shear_stress(mu, Q_m3_s, d_m)
        result.wall_shear_stress_Pa = tau
        result.shear_safe = tau <= shear_stress_limit_Pa

        if not result.shear_safe:
            warnings.append(
                f"Wall shear stress {tau:.2f} Pa exceeds cell viability "
                f"limit {shear_stress_limit_Pa} Pa"
            )
            # Shear stress alone doesn't make it "unsafe" (user may accept)
            # but we flag it prominently

    result.warnings = warnings
    return result


def calculate_print_safety(
    needle: NeedleSpec,
    syringe: SyringeSpec,
    ink: InkSpec,
    print_speed_mm_s: float,
    layer_height_mm: float,
    pressure_limit_Pa: float = DEFAULT_PRESSURE_LIMIT_PA,
    shear_stress_limit_Pa: float = DEFAULT_SHEAR_STRESS_LIMIT_PA,
) -> FlowSafetyResult:
    """
    Convenience function: compute flow safety for a given print speed + layer height.

    Calculates the required extrusion flow rate from the print parameters,
    then runs full safety analysis.

    Args:
        needle: Needle specification
        syringe: Syringe specification
        ink: Ink to be deposited
        print_speed_mm_s: Linear travel speed (mm/s)
        layer_height_mm: Layer height (mm)
        pressure_limit_Pa: Pressure limit (Pa)
        shear_stress_limit_Pa: Shear stress limit for cells (Pa)

    Returns:
        FlowSafetyResult
    """
    flow = extrusion_flow_rate(print_speed_mm_s, needle, layer_height_mm)
    return calculate_flow_safety(
        needle, syringe, ink, flow,
        pressure_limit_Pa, shear_stress_limit_Pa,
    )


# ---------------------------------------------------------------------------
# Compatibility Report Generator
# ---------------------------------------------------------------------------

def generate_compatibility_report(
    needle: NeedleSpec,
    pumps: dict,  # str -> PumpLoadout
    pressure_limit_Pa: float = DEFAULT_PRESSURE_LIMIT_PA,
    default_flow_rate_uL_s: float = 1.0,
) -> list[dict]:
    """
    Generate a compatibility report for all pump/ink combinations.

    Used by the Workspace tab to show the auto-computed compatibility panel.

    Args:
        needle: Current needle spec
        pumps: Dict of pump_id -> PumpLoadout
        pressure_limit_Pa: Pressure limit
        default_flow_rate_uL_s: Default flow rate to test

    Returns:
        List of dicts with pump_id, status, message, max_rate, safety_result
    """
    reports = []
    for pump_id, pump in pumps.items():
        if pump.syringe is None or pump.current_ink is None:
            reports.append({
                "pump_id": pump_id,
                "status": "no_config",
                "message": f"{pump_id}: No syringe or ink configured",
                "max_rate_uL_s": 0.0,
                "safety_result": None,
            })
            continue

        ink = pump.current_ink
        safety = calculate_flow_safety(
            needle, pump.syringe, ink,
            default_flow_rate_uL_s, pressure_limit_Pa,
        )

        # Determine ink compatibility
        compat = ink.flow_compatibility_detail(needle)

        reports.append({
            "pump_id": pump_id,
            "status": compat["severity"],
            "message": compat["message"],
            "max_rate_uL_s": safety.max_safe_flow_rate_uL_s,
            "safety_result": safety,
        })

    return reports
