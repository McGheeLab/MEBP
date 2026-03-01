"""
HardwareConfig.py — Central hardware configuration for MEBP v7.2.

Bundles all physical setup into one saveable/loadable unit:
    - Per-pump syringe selection and ink assignment
    - Needle configuration
    - Well plate format
    - Ink library
    - Fluid column state per pump

This is the single source of truth for µL ↔ mm conversion.
All pump motion in the entire application is described in µL;
this module handles the conversion to mm for the Marlin firmware.

Key design principle: The GUI and print commands work exclusively in µL.
Only StageController.move_pump_uL() calls SyringeSpec.uL_to_mm() internally
when sending commands to the Marlin board.

Usage::

    config = HardwareConfig()
    config.set_pump_syringe("P1", syringe_catalog[100])
    config.set_pump_ink("P1", my_ink)
    config.validate()  # → True if all required fields set
    config.save("my_setup.json")
    config = HardwareConfig.load("my_setup.json")

    # Convert µL to mm for firmware
    mm = config.uL_to_mm("P1", 5.0)   # 5 µL → mm of plunger travel
    uL = config.mm_to_uL("P1", 1.5)   # 1.5 mm → µL dispensed
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger(__name__)

# Import physical models — these are the existing v7.1 dataclasses
from SupportClasses.PhysicalModels import (
    NeedleSpec,
    SyringeSpec,
    InkSpec,
    FluidColumn,
    PumpLoadout,
    PrintingMode,
    RosetteInsert,
    load_needle_catalog,
    load_syringe_catalog,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS


# ═══════════════════════════════════════════════════════════════════
# Pump Channel Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PumpChannelConfig:
    """
    Configuration for a single pump channel (P1, P2, or P3).

    Tracks what syringe is installed, what ink is loaded, printing mode,
    and the current fluid column state. All user-facing values are in µL.
    """
    pump_id: str = "P1"
    syringe: SyringeSpec | None = None
    ink: InkSpec | None = None
    printing_mode: PrintingMode = PrintingMode.INCREMENTAL
    fluid_column: FluidColumn = field(default_factory=FluidColumn)
    enabled: bool = False  # True once syringe is assigned

    @property
    def is_configured(self) -> bool:
        """True if this pump has a syringe assigned."""
        return self.syringe is not None and self.enabled

    @property
    def has_ink(self) -> bool:
        """True if ink is assigned to this channel."""
        return self.ink is not None and self.is_configured

    def uL_to_mm(self, volume_uL: float) -> float:
        """Convert µL to mm of plunger travel. Raises if no syringe."""
        if self.syringe is None:
            raise ValueError(f"{self.pump_id}: No syringe configured — cannot convert µL to mm")
        return self.syringe.uL_to_mm(volume_uL)

    def mm_to_uL(self, travel_mm: float) -> float:
        """Convert mm of plunger travel to µL. Raises if no syringe."""
        if self.syringe is None:
            raise ValueError(f"{self.pump_id}: No syringe configured — cannot convert mm to µL")
        return self.syringe.mm_to_uL(travel_mm)

    def feedrate_uL_s_to_mm_min(self, rate_uL_s: float) -> float:
        """Convert flow rate in µL/s to Marlin feedrate in mm/min."""
        if self.syringe is None:
            raise ValueError(f"{self.pump_id}: No syringe configured")
        mm_per_s = self.syringe.uL_to_mm(rate_uL_s)
        return mm_per_s * 60.0  # mm/s → mm/min

    def feedrate_mm_min_to_uL_s(self, feedrate_mm_min: float) -> float:
        """Convert Marlin feedrate in mm/min to flow rate in µL/s."""
        if self.syringe is None:
            raise ValueError(f"{self.pump_id}: No syringe configured")
        mm_per_s = feedrate_mm_min / 60.0
        return self.syringe.mm_to_uL(mm_per_s)

    def to_dict(self) -> dict:
        return {
            "pump_id": self.pump_id,
            "syringe_volume_uL": self.syringe.volume_uL if self.syringe else None,
            "syringe": self.syringe.to_dict() if self.syringe else None,
            "ink": self.ink.to_dict() if self.ink else None,
            "printing_mode": self.printing_mode.value,
            "fluid_column": {
                "oil_volume_uL": self.fluid_column.oil_volume_uL,
                "buffer_volume_uL": self.fluid_column.buffer_volume_uL,
                "ink_volume_uL": self.fluid_column.ink_volume_uL,
                "dead_volume_uL": self.fluid_column.dead_volume_uL,
            },
            "enabled": self.enabled,
        }

    @classmethod
    def from_dict(cls, data: dict) -> PumpChannelConfig:
        syringe = SyringeSpec.from_dict(data["syringe"]) if data.get("syringe") else None
        ink = InkSpec.from_dict(data["ink"]) if data.get("ink") else None
        mode = PrintingMode(data.get("printing_mode", "incremental"))
        fc_data = data.get("fluid_column", {})
        fluid_column = FluidColumn(
            oil_volume_uL=fc_data.get("oil_volume_uL", 0.0),
            buffer_volume_uL=fc_data.get("buffer_volume_uL", 0.0),
            ink_volume_uL=fc_data.get("ink_volume_uL", 0.0),
            dead_volume_uL=fc_data.get("dead_volume_uL", 2.0),
            ink_spec=ink,
        )
        return cls(
            pump_id=data.get("pump_id", "P1"),
            syringe=syringe,
            ink=ink,
            printing_mode=mode,
            fluid_column=fluid_column,
            enabled=data.get("enabled", syringe is not None),
        )


# ═══════════════════════════════════════════════════════════════════
# Hardware Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class HardwareConfig:
    """
    Complete hardware setup — the gating configuration for the entire app.

    Must be valid before any other page (Jog, Calibration, Print, etc.)
    can operate. Saved/loaded as JSON for quick experiment swaps.
    """

    # ── Needle ────────────────────────────────────────────────────
    needle: NeedleSpec | None = None

    # ── Pump channels ─────────────────────────────────────────────
    pumps: dict[str, PumpChannelConfig] = field(default_factory=lambda: {
        "P1": PumpChannelConfig(pump_id="P1"),
        "P2": PumpChannelConfig(pump_id="P2"),
        "P3": PumpChannelConfig(pump_id="P3"),
    })

    # ── Well plate ────────────────────────────────────────────────
    plate_format: int = 24  # 6, 12, 24, 48, 96, 384

    # ── Ink library (persisted across sessions) ───────────────────
    ink_library: dict[str, InkSpec] = field(default_factory=dict)

    # ── Rosette library ───────────────────────────────────────────
    rosette_library: dict[str, RosetteInsert] = field(default_factory=dict)

    # ── Buffer ink ────────────────────────────────────────────────
    buffer_ink_name: str | None = None

    # ── Metadata ──────────────────────────────────────────────────
    config_name: str = "Untitled Setup"
    notes: str = ""

    # ══════════════════════════════════════════════════════════════
    #  VALIDATION
    # ══════════════════════════════════════════════════════════════

    def validate(self) -> tuple[bool, list[str]]:
        """
        Check if the hardware setup is complete enough to proceed.

        Returns (is_valid, list_of_issues).
        Minimum requirement: at least one pump configured with a syringe.
        """
        issues = []

        # Must have a needle
        if self.needle is None:
            issues.append("No needle gauge selected")

        # Must have at least one pump with a syringe
        configured_pumps = [p for p in self.pumps.values() if p.is_configured]
        if not configured_pumps:
            issues.append("At least one pump must have a syringe assigned")

        # Plate format must be valid
        if self.plate_format not in PLATE_DEFINITIONS:
            issues.append(f"Invalid plate format: {self.plate_format}")

        return (len(issues) == 0, issues)

    @property
    def is_valid(self) -> bool:
        """Quick check: is this config ready for use?"""
        valid, _ = self.validate()
        return valid

    @property
    def configured_pump_ids(self) -> list[str]:
        """List of pump IDs that have syringes assigned."""
        return [pid for pid, p in self.pumps.items() if p.is_configured]

    @property
    def active_pump_count(self) -> int:
        return len(self.configured_pump_ids)

    # ══════════════════════════════════════════════════════════════
    #  µL ↔ mm CONVERSION (central access point)
    # ══════════════════════════════════════════════════════════════

    def uL_to_mm(self, pump: str, volume_uL: float) -> float:
        """Convert µL to mm of plunger travel for a specific pump."""
        return self.pumps[pump].uL_to_mm(volume_uL)

    def mm_to_uL(self, pump: str, travel_mm: float) -> float:
        """Convert mm plunger travel to µL for a specific pump."""
        return self.pumps[pump].mm_to_uL(travel_mm)

    def feedrate_uL_s_to_mm_min(self, pump: str, rate_uL_s: float) -> float:
        """Convert flow rate (µL/s) to Marlin feedrate (mm/min) for a pump."""
        return self.pumps[pump].feedrate_uL_s_to_mm_min(rate_uL_s)

    def feedrate_mm_min_to_uL_s(self, pump: str, feedrate_mm_min: float) -> float:
        """Convert Marlin feedrate (mm/min) to flow rate (µL/s) for a pump."""
        return self.pumps[pump].feedrate_mm_min_to_uL_s(feedrate_mm_min)

    def get_syringe(self, pump: str) -> SyringeSpec | None:
        """Get the syringe spec for a pump, or None."""
        return self.pumps[pump].syringe if pump in self.pumps else None

    # ══════════════════════════════════════════════════════════════
    #  PUMP CONFIGURATION HELPERS
    # ══════════════════════════════════════════════════════════════

    def set_pump_syringe(self, pump: str, syringe: SyringeSpec | None):
        """Assign a syringe to a pump channel."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].syringe = syringe
        self.pumps[pump].enabled = syringe is not None
        logger.info(f"{pump}: syringe set to {syringe.volume_uL}µL" if syringe else f"{pump}: syringe cleared")

    def set_pump_ink(self, pump: str, ink: InkSpec | None):
        """Assign an ink to a pump channel."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].ink = ink
        if ink:
            self.pumps[pump].fluid_column.ink_spec = ink
        logger.info(f"{pump}: ink set to '{ink.name}'" if ink else f"{pump}: ink cleared")

    def set_pump_mode(self, pump: str, mode: PrintingMode):
        """Set printing mode for a pump."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].printing_mode = mode

    # ══════════════════════════════════════════════════════════════
    #  INK LIBRARY
    # ══════════════════════════════════════════════════════════════

    def add_ink(self, ink: InkSpec):
        """Add or update an ink in the library."""
        self.ink_library[ink.name] = ink

    def remove_ink(self, name: str):
        """Remove an ink from the library."""
        self.ink_library.pop(name, None)
        # Also clear from any pump that was using it
        for pump in self.pumps.values():
            if pump.ink and pump.ink.name == name:
                pump.ink = None

    def get_ink(self, name: str) -> InkSpec | None:
        return self.ink_library.get(name)

    # ══════════════════════════════════════════════════════════════
    #  SAVE / LOAD
    # ══════════════════════════════════════════════════════════════

    def to_dict(self) -> dict:
        """Serialize complete hardware config to a dictionary."""
        return {
            "version": "7.2",
            "config_name": self.config_name,
            "notes": self.notes,
            "needle": self.needle.to_dict() if self.needle else None,
            "pumps": {pid: p.to_dict() for pid, p in self.pumps.items()},
            "plate_format": self.plate_format,
            "ink_library": {name: ink.to_dict() for name, ink in self.ink_library.items()},
            "rosette_library": {
                name: r.to_dict() for name, r in self.rosette_library.items()
            },
            "buffer_ink_name": self.buffer_ink_name,
        }

    @classmethod
    def from_dict(cls, data: dict) -> HardwareConfig:
        """Deserialize from dictionary."""
        config = cls()
        config.config_name = data.get("config_name", "Untitled Setup")
        config.notes = data.get("notes", "")

        # Needle
        if data.get("needle"):
            config.needle = NeedleSpec.from_dict(data["needle"])

        # Pumps
        for pid, pdata in data.get("pumps", {}).items():
            if pid in config.pumps:
                config.pumps[pid] = PumpChannelConfig.from_dict(pdata)

        # Plate format
        config.plate_format = data.get("plate_format", 24)

        # Ink library
        for name, ink_data in data.get("ink_library", {}).items():
            config.ink_library[name] = InkSpec.from_dict(ink_data)

        # Rosette library
        for name, r_data in data.get("rosette_library", {}).items():
            try:
                config.rosette_library[name] = RosetteInsert.from_dict(r_data)
            except Exception as e:
                logger.warning(f"Failed to load rosette '{name}': {e}")

        config.buffer_ink_name = data.get("buffer_ink_name")
        return config

    def save(self, filepath: str | Path) -> None:
        """Save hardware configuration to JSON file."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        logger.info(f"Hardware config saved to {filepath}")

    @classmethod
    def load(cls, filepath: str | Path) -> HardwareConfig:
        """Load hardware configuration from JSON file."""
        filepath = Path(filepath)
        with open(filepath) as f:
            data = json.load(f)
        config = cls.from_dict(data)
        logger.info(f"Hardware config loaded from {filepath}")
        return config

    def __repr__(self) -> str:
        pumps = ", ".join(
            f"{pid}={'✓' if p.is_configured else '—'}"
            for pid, p in self.pumps.items()
        )
        return (f"HardwareConfig('{self.config_name}', needle={self.needle.gauge if self.needle else '?'}G, "
                f"plate={self.plate_format}, pumps=[{pumps}])")
