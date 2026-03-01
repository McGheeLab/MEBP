"""
Unit conversion helpers for MEBP GUI — microsteps ↔ microns (µm).

The XY stage (Prior ProScan) reports positions in microsteps.
This module provides a single source of truth for converting
to display-friendly microns (µm).

The ``microsteps_per_micron`` factor is loaded from:
    1. Controller protocol JSON  →  parameters.microsteps_per_micron
    2. settings.json             →  stage.microsteps_per_micron
    3. Default                   →  10.0

Default: 10.0 microsteps/µm  (Prior ProScan III typical: 0.1 µm resolution).

Usage::

    from gui.unit_helpers import steps_to_um, um_to_steps, format_um

    um_x = steps_to_um(step_x, factor)
    step_x = um_to_steps(um_x, factor)
    label.setText(format_um(um_x))           # "1,234.5"
    label.setText(format_um_pair(ux, uy))    # "1,234.5, 567.8"
"""

from __future__ import annotations


# ── Default conversion factor ────────────────────────────────────
DEFAULT_MICROSTEPS_PER_MICRON: float = 10.0


# ══════════════════════════════════════════════════════════════════
#  Core conversions
# ══════════════════════════════════════════════════════════════════

def steps_to_um(steps: float, microsteps_per_micron: float) -> float:
    """Convert stage microsteps → microns (µm)."""
    if microsteps_per_micron <= 0:
        return steps
    return steps / microsteps_per_micron


def um_to_steps(microns: float, microsteps_per_micron: float) -> float:
    """Convert microns (µm) → stage microsteps."""
    if microsteps_per_micron <= 0:
        return microns
    return microns * microsteps_per_micron


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


def convert_safety_xy_text(sl, microsteps_per_micron: float) -> str:
    """
    Return a one-line summary of safety limits with XY in µm.

    Args:
        sl: SafetyLimits instance (has xy_min_x, xy_max_x, etc.)
        microsteps_per_micron: conversion factor
    """
    x_min = steps_to_um(sl.xy_min_x, microsteps_per_micron)
    x_max = steps_to_um(sl.xy_max_x, microsteps_per_micron)
    y_min = steps_to_um(sl.xy_min_y, microsteps_per_micron)
    y_max = steps_to_um(sl.xy_max_y, microsteps_per_micron)
    return (
        f"X [{x_min:,.0f} .. {x_max:,.0f}] µm  ×  "
        f"Y [{y_min:,.0f} .. {y_max:,.0f}] µm  |  "
        f"Z [{sl.z_min:.1f} .. {sl.z_max:.1f}] mm  |  "
        f"P [{sl.p1_min:.1f} .. {sl.p1_max:.1f}] mm"
    )


# ══════════════════════════════════════════════════════════════════
#  Protocol JSON helper
# ══════════════════════════════════════════════════════════════════

def get_microsteps_per_micron_from_protocol(protocol) -> float | None:
    """
    Extract microsteps_per_micron from a ControllerProtocol instance.

    Returns None if the value is not found, so the caller can fall back
    to settings or the default.
    """
    if protocol is None:
        return None
    try:
        params = protocol._config.get("parameters", {})
        value = params.get("microsteps_per_micron")
        if value is not None and float(value) > 0:
            return float(value)
    except (AttributeError, TypeError, ValueError):
        pass
    return None
