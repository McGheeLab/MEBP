"""
Unit conversion helpers for MEBP GUI — stage readout ↔ microns (µm).

The XY stage (Prior ProScan) reports positions in µm natively
(confirmed via diagnostic in v7.2.5). The ``xy_position_scale``
factor exists for future support of stages that report in different
units, but defaults to 1.0 since the ProScan speaks µm directly.

The ``xy_position_scale`` factor is loaded from:
    1. Controller protocol JSON  →  parameters.xy_position_scale
    2. settings.json             →  stage.xy_position_scale
    3. Default                   →  1.0

Default: 1.0 (ProScan speaks µm natively — no conversion needed).

Usage::

    from gui.unit_helpers import stage_to_um, um_to_stage, format_um

    um_x = stage_to_um(readout_x, scale)
    readout_x = um_to_stage(um_x, scale)
    label.setText(format_um(um_x))           # "1,234.5"
    label.setText(format_um_pair(ux, uy))    # "1,234.5, 567.8"
"""

from __future__ import annotations


# ── Default conversion factor ────────────────────────────────────
# v7.3.5: The Prior ProScan stage speaks µm natively (confirmed via
# diagnostic). Scale factor = 1.0 (identity — no conversion needed).
DEFAULT_XY_POSITION_SCALE: float = 1.0


# ══════════════════════════════════════════════════════════════════
#  Core conversions
# ══════════════════════════════════════════════════════════════════

def stage_to_um(readout: float, xy_position_scale: float) -> float:
    """Convert stage position readout → microns (µm)."""
    if xy_position_scale <= 0:
        return readout
    return readout / xy_position_scale


def um_to_stage(microns: float, xy_position_scale: float) -> float:
    """Convert microns (µm) → stage position units."""
    if xy_position_scale <= 0:
        return microns
    return microns * xy_position_scale


# ══════════════════════════════════════════════════════════════════
#  Display formatters
# ══════════════════════════════════════════════════════════════════

def format_um(value: float, decimals: int = 1) -> str:
    """Format a single micron value with comma separators."""
    return f"{value:,.{decimals}f}"


def format_um_pair(x: float, y: float, decimals: int = 1) -> str:
    """Format an XY pair in microns: '1,234.5, 567.8'."""
    return f"{x:,.{decimals}f}, {y:,.{decimals}f}"


def format_um_range(min_val: float, max_val: float, decimals: int = 0) -> str:
    """Format a min..max range in microns."""
    return f"{min_val:,.{decimals}f} .. {max_val:,.{decimals}f}"


def convert_safety_xy_text(sl, xy_position_scale: float) -> str:
    """
    Return a one-line summary of safety limits with XY in µm.

    Args:
        sl: SafetyLimits instance (has xy_min_x, xy_max_x, etc.)
        xy_position_scale: conversion factor (stage readout units per µm)
    """
    x_min = stage_to_um(sl.xy_min_x, xy_position_scale)
    x_max = stage_to_um(sl.xy_max_x, xy_position_scale)
    y_min = stage_to_um(sl.xy_min_y, xy_position_scale)
    y_max = stage_to_um(sl.xy_max_y, xy_position_scale)
    return (
        f"X [{x_min:,.0f} .. {x_max:,.0f}] µm  ×  "
        f"Y [{y_min:,.0f} .. {y_max:,.0f}] µm  |  "
        f"Z [{sl.z_min:.1f} .. {sl.z_max:.1f}] mm  |  "
        f"P [{sl.p1_min:.1f} .. {sl.p1_max:.1f}] mm"
    )


# ══════════════════════════════════════════════════════════════════
#  Protocol JSON helper
# ══════════════════════════════════════════════════════════════════

def get_position_scale_from_protocol(protocol) -> float | None:
    """
    Extract xy_position_scale from a ControllerProtocol instance.

    Returns None if the value is not found, so the caller can fall back
    to settings or the default.
    """
    if protocol is None:
        return None
    try:
        params = protocol._config.get("parameters", {})
        value = params.get("xy_position_scale")
        if value is not None and float(value) > 0:
            return float(value)
    except (AttributeError, TypeError, ValueError):
        pass
    return None
