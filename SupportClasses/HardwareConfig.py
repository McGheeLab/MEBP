"""
HardwareConfig.py — Central hardware configuration for MEBP v7.2.4.

Bundles all physical setup into one saveable/loadable unit:
    - Per-pump syringe selection and ink assignment
    - Needle configuration with channel-to-pump mapping
    - Well plate format
    - Ink library
    - Fluid column state per pump

This is the single source of truth for µL ↔ mm conversion.
All pump motion in the entire application is described in µL;
this module handles the conversion to mm for the Marlin firmware.

v7.2.4 Changes:
    - Added pump_ink_map / ink_pump_map convenience properties (S3.1)
    - Added needle_channel_pump_map field with serialization (S3.2)
    - Enhanced validate() with ink uniqueness + channel mapping checks (S3.3)
    - Validation returns categorized issues (error vs warning)

Usage::

    config = HardwareConfig()
    config.set_pump_syringe("P1", syringe_catalog[100])
    config.set_pump_ink("P1", my_ink)
    config.validate()  # → (True, []) if all required fields set
    config.save("my_setup.json")
    config = HardwareConfig.load("my_setup.json")

    # Convenience lookups
    config.pump_ink_map    # {"P1": "Hydrogel A", "P2": None, ...}
    config.ink_pump_map    # {"Hydrogel A": "P1"}

    # Channel mapping for multi-channel needles
    config.needle_channel_pump_map  # {0: "P1", 1: "P2"}
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
            raise ValueError(f"{self.pump_id}: No syringe configured")
        return self.syringe.uL_to_mm(volume_uL)

    def mm_to_uL(self, distance_mm: float) -> float:
        """Convert mm of plunger travel to µL. Raises if no syringe."""
        if self.syringe is None:
            raise ValueError(f"{self.pump_id}: No syringe configured")
        return self.syringe.mm_to_uL(distance_mm)

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

    # ── v7.2.4: Needle channel → pump mapping ────────────────────
    needle_channel_pump_map: dict[int, str] = field(default_factory=dict)
    # Maps channel index (0-based) → pump_id
    # Single-channel needle: {0: "P1"}
    # Multi-channel: {0: "P1", 1: "P2", 2: "P3"}

    # ── Metadata ──────────────────────────────────────────────────
    config_name: str = "Untitled Setup"
    notes: str = ""

    # ══════════════════════════════════════════════════════════════
    #  v7.2.4: PUMP-INK MAPPING PROPERTIES (S3.1)
    # ══════════════════════════════════════════════════════════════

    @property
    def pump_ink_map(self) -> dict[str, str | None]:
        """
        Get pump → ink_name mapping for quick lookup.

        Returns dict like {"P1": "Hydrogel A", "P2": "MSC Cells", "P3": None}
        Only includes enabled pumps.
        """
        result = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled:
                result[pid] = pcfg.ink.name if pcfg.ink else None
            else:
                result[pid] = None
        return result

    @property
    def ink_pump_map(self) -> dict[str, str]:
        """
        Get ink_name → pump_id reverse lookup.

        Returns dict like {"Hydrogel A": "P1", "MSC Cells": "P2"}
        Only includes pumps that have an ink assigned.
        """
        result = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.ink:
                result[pcfg.ink.name] = pid
        return result

    @property
    def unassigned_inks(self) -> list[str]:
        """Get list of ink names in the library not assigned to any pump."""
        assigned = set(self.ink_pump_map.keys())
        return [name for name in self.ink_library if name not in assigned]

    @property
    def enabled_pump_ids(self) -> list[str]:
        """List of pump IDs that are enabled (have syringes)."""
        return [pid for pid, p in self.pumps.items() if p.enabled and p.is_configured]

    # ══════════════════════════════════════════════════════════════
    #  VALIDATION (v7.2.4 enhanced — S3.3)
    # ══════════════════════════════════════════════════════════════

    def validate(self) -> tuple[bool, list[str]]:
        """
        Check if the hardware setup is complete enough to proceed.

        Returns (is_valid, list_of_issues).
        """
        issues = []

        # Needle
        if self.needle is None:
            issues.append("No needle gauge selected")

        # Pumps — at least one enabled with syringe
        configured_pumps = [p for p in self.pumps.values() if p.is_configured]
        if not configured_pumps:
            issues.append("At least one pump must be enabled with a syringe")

        # Enabled pumps must have inks assigned
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.syringe and not pcfg.ink:
                issues.append(f"{pid} is enabled but has no ink assigned")

        # Plate format
        if self.plate_format not in PLATE_DEFINITIONS:
            issues.append(f"Invalid plate format: {self.plate_format}")

        # Ink uniqueness — each ink assigned to at most one pump
        ink_assignments: dict[str, list[str]] = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.ink:
                ink_assignments.setdefault(pcfg.ink.name, []).append(pid)
        for ink_name, pump_ids in ink_assignments.items():
            if len(pump_ids) > 1:
                issues.append(
                    f"Ink '{ink_name}' assigned to multiple pumps: "
                    f"{', '.join(pump_ids)}")

        # Needle channel-pump mapping
        if self.needle is not None:
            num_channels = self.needle.num_channels
            enabled_ids = self.enabled_pump_ids

            if num_channels > 0:
                if not enabled_ids:
                    issues.append(
                        f"Needle has {num_channels} channel(s) but "
                        f"no pumps are enabled")
                elif len(self.needle_channel_pump_map) != num_channels:
                    issues.append(
                        f"Needle has {num_channels} channel(s) but "
                        f"{len(self.needle_channel_pump_map)} mapped")
                else:
                    # Check mapped pumps are enabled
                    for ch_idx, pump_id in self.needle_channel_pump_map.items():
                        if pump_id not in enabled_ids:
                            issues.append(
                                f"Channel {ch_idx + 1} → {pump_id} "
                                f"but {pump_id} is not enabled")

                    # Check uniqueness
                    mapped_pumps: dict[str, list[int]] = {}
                    for ch_idx, pump_id in self.needle_channel_pump_map.items():
                        mapped_pumps.setdefault(pump_id, []).append(ch_idx)
                    for pump_id, channels in mapped_pumps.items():
                        if len(channels) > 1:
                            ch_strs = [str(c + 1) for c in channels]
                            issues.append(
                                f"{pump_id} mapped to multiple channels: "
                                f"{', '.join(ch_strs)}")

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
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        return self.pumps[pump].uL_to_mm(volume_uL)

    def mm_to_uL(self, pump: str, distance_mm: float) -> float:
        """Convert mm of plunger travel to µL for a specific pump."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        return self.pumps[pump].mm_to_uL(distance_mm)

    # ══════════════════════════════════════════════════════════════
    #  PUMP CONFIGURATION HELPERS
    # ══════════════════════════════════════════════════════════════

    def set_pump_syringe(self, pump: str, syringe: SyringeSpec | None):
        """Assign a syringe to a pump. None to remove."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].syringe = syringe
        self.pumps[pump].enabled = syringe is not None
        # v7.2.4: Clear channel mappings if pump changes
        self._clear_invalid_channel_mappings()
        # Clear channel mapping if pump changes configuration
        self._clear_invalid_channel_mappings()

    def set_pump_ink(self, pump: str, ink: InkSpec | None):
        """Assign ink to a pump. None to remove."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].ink = ink

    def set_pump_mode(self, pump: str, mode: PrintingMode):
        """Set printing mode for a pump."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].printing_mode = mode

    # ══════════════════════════════════════════════════════════════
    #  v7.2.4: NEEDLE CHANNEL MAP HELPERS (S3.2)
    # ══════════════════════════════════════════════════════════════

    def set_channel_pump(self, channel_index: int, pump_id: str | None):
        """
        Assign a pump to a needle channel.

        Args:
            channel_index: 0-based channel index
            pump_id: Pump ID ("P1", "P2", "P3") or None to clear
        """
        if pump_id is None:
            self.needle_channel_pump_map.pop(channel_index, None)
        else:
            if pump_id not in self.pumps:
                raise ValueError(f"Unknown pump: {pump_id}")
            self.needle_channel_pump_map[channel_index] = pump_id

    def get_channel_pump(self, channel_index: int) -> str | None:
        """Get pump ID assigned to a channel, or None."""
        return self.needle_channel_pump_map.get(channel_index)

    def clear_channel_map(self):
        """Clear all channel-pump assignments."""
        self.needle_channel_pump_map.clear()

    def auto_assign_channels(self):
        """
        Auto-assign channels to enabled pumps in order.

        Channel 0 → first enabled pump, channel 1 → second, etc.
        Only assigns up to min(num_channels, num_enabled_pumps).
        """
        if self.needle is None:
            return
        self.needle_channel_pump_map.clear()
        enabled = self.enabled_pump_ids
        for ch_idx in range(self.needle.num_channels):
            if ch_idx < len(enabled):
                self.needle_channel_pump_map[ch_idx] = enabled[ch_idx]

    def _clear_invalid_channel_mappings(self):
        """Remove channel mappings that reference disabled/unconfigured pumps."""
        enabled = set(self.enabled_pump_ids)
        invalid_channels = [
            ch for ch, pid in self.needle_channel_pump_map.items()
            if pid not in enabled
        ]
        for ch in invalid_channels:
            del self.needle_channel_pump_map[ch]

    def get_pump_for_ink(self, ink_name: str) -> str | None:
        """Get the pump ID that has the given ink assigned, or None."""
        return self.ink_pump_map.get(ink_name)

    def get_channel_for_ink(self, ink_name: str) -> int | None:
        """Get the needle channel index that carries the given ink, or None."""
        pump_id = self.get_pump_for_ink(ink_name)
        if pump_id is None:
            return None
        for ch_idx, pid in self.needle_channel_pump_map.items():
            if pid == pump_id:
                return ch_idx
        return None

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
    #  SAVE / LOAD (v7.2.4: adds needle_channel_pump_map)
    # ══════════════════════════════════════════════════════════════

    def to_dict(self) -> dict:
        """Serialize complete hardware config to a dictionary."""
        return {
            "version": "7.2.4",
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
            # v7.2.4: Channel mapping (serialize int keys as strings for JSON)
            "needle_channel_pump_map": {
                str(ch): pid for ch, pid in self.needle_channel_pump_map.items()
            },
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

        # v7.2.4: Needle channel-pump map (JSON keys are strings → int)
        raw_map = data.get("needle_channel_pump_map", {})
        config.needle_channel_pump_map = {
            int(ch): pid for ch, pid in raw_map.items()
        }

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
        ch_map = ""
        if self.needle_channel_pump_map:
            ch_map = f", channels={self.needle_channel_pump_map}"
        return (
            f"HardwareConfig('{self.config_name}', "
            f"needle={self.needle.gauge if self.needle else '?'}G, "
            f"plate={self.plate_format}, pumps=[{pumps}]{ch_map})"
        )
