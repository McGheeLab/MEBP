"""
help_texts.py — Markdown help strings for FormRow widgets (v7.4.0-c).

When the user toggles Help mode on (HelpToggle in the top bar), every
registered FormRow reveals its help text inline. This file is the single
source of truth for those strings; the goal is to keep them tightly
co-located with where they're referenced.

Naming convention: ``page.section.field`` so future grep makes it easy
to find which UI surface a help string belongs to.
"""

from __future__ import annotations

HELP_TEXTS: dict[str, str] = {
    # ── Hardware Setup ──────────────────────────────────────────
    "hardware.identity.name": (
        "A short, memorable label for this hardware configuration. "
        "Use the lab member name, project, or experiment series — "
        "you can save and reload by name."
    ),
    "hardware.plate.format": (
        "Standard ANSI/SLAS well plate format. Determines well count, "
        "row/column count, and well spacing for calibration and the "
        "well-setup grid."
    ),
    "hardware.needle.gauge": (
        "Hypodermic needle gauge in standard sizing (16G = thickest, "
        "32G = thinnest). Determines bore diameter and maximum flow rate."
    ),
    "hardware.needle.channels": (
        "Number of independent fluid channels in this needle. "
        "Single-channel needles use one pump; multi-channel needles "
        "(co-axial) use multiple pumps simultaneously."
    ),
    "hardware.pump.syringe": (
        "Total syringe volume in µL. Used to convert pump positions "
        "between µL and stage millimetres. Hamilton catalog includes "
        "25, 100, 250, 500, and 1000 µL."
    ),
    "hardware.pump.ink": (
        "Which ink(s) in the library this pump is loaded with. A pump "
        "with multiple inks uses the configured ink-swap strategy "
        "(see Print Setup → Finalize) to transition between them."
    ),

    # ── Stage sub-page ──────────────────────────────────────────
    "hardware.stage.safety_enabled": (
        "Master safety toggle. When off, software endstops do not prevent "
        "moves outside the configured XY/Z/Pump min-max range. Turn off "
        "only when you have a specific reason; you'll be working without "
        "a net."
    ),
    "hardware.stage.xy_min_max": (
        "Software endstop range for the XY stage in microns, relative "
        "to the zero reference. The stage will refuse moves outside "
        "this range while safety limits are enabled."
    ),
    "hardware.stage.zp_jog_feedrate": (
        "Z and pump jog speed in mm/min. Higher values move faster but "
        "may overshoot on slow controllers; 900 mm/min is a safe default."
    ),
    "hardware.stage.axis_flip": (
        "Invert the positive direction of an axis. Use when your "
        "mechanical setup moves opposite to expected — common for "
        "pumps installed in reverse orientation."
    ),

    # ── Settings ────────────────────────────────────────────────
    "settings.simulation.xy": (
        "When enabled, the XY stage is simulated by an in-memory "
        "physics model. Useful for testing print files without "
        "moving real hardware. Requires app restart."
    ),
    "settings.simulation.zp": (
        "When enabled, the Z and pump axes are simulated. The "
        "simulator persists position to disk so you don't lose "
        "your reference between sessions. Requires app restart."
    ),
    "settings.polling.position_interval": (
        "How often the app reads back stage positions, in milliseconds. "
        "Lower values give smoother live position display but more "
        "serial traffic. 300 ms is a sensible default."
    ),
    "settings.xbox.deadzone_sticks": (
        "Stick deadzone as a percentage. Input below this threshold "
        "is ignored — prevents drift from worn analog sticks."
    ),

    # ── Calibration ─────────────────────────────────────────────
    "calibration.umpx": (
        "Microns per pixel for the active camera. Empirical "
        "calibration via stage motion (Calibrate µm/px button) is "
        "more accurate than the theoretical value from objective "
        "magnification + pixel pitch."
    ),
    "calibration.safe_z": (
        "Travel height in mm — Z position used for inter-well moves. "
        "Must be high enough to clear the tallest well rim without "
        "hitting the needle. Teach by jogging Z to a safe height "
        "and clicking 'Set Safe Z'."
    ),
}


def get(key: str) -> str:
    """Look up help text by key. Returns empty string if not found."""
    return HELP_TEXTS.get(key, "")
