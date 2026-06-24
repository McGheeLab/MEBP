"""
HardwareConfig.py — Central hardware configuration for MEBP v7.3.0.

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
    CameraSpec,
    load_needle_catalog,
    load_syringe_catalog,
    load_camera_catalog,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS


# ═══════════════════════════════════════════════════════════════════
# Ink Swap Strategy (v7.2.9: canonical definition in PrintPlanOfAction.py)
# ═══════════════════════════════════════════════════════════════════

# v7.2.9: InkSwapStrategy moved to PrintPlanOfAction.py as the single
# source of truth. Re-exported here for backwards compatibility.
from SupportClasses.PrintPlanOfAction import InkSwapStrategy  # noqa: F401


# ═══════════════════════════════════════════════════════════════════
# Camera Roles (v7.4.4)
# ═══════════════════════════════════════════════════════════════════

class CameraRole(str, Enum):
    """Workflow role assigned to a live camera.

    The Calibration page resolves which `CameraManager.cameras[i]` to
    mount on each workflow tab by looking up the index that has the
    expected role. Two side-mounted needle cameras (X-view and Y-view)
    plus the MICROSCOPE camera cover the workflows; MICROSCOPE tags the
    camera that sits behind an objective lens and looks down at the well
    plate, and supports per-objective µm/px calibration.

    v7.5.x: the separate ``PLATE`` ("overhead cam over the well plate")
    role was removed — the microscope camera *is* the camera that views
    the plate, so MICROSCOPE serves both purposes. Legacy ``"plate"``
    role strings migrate to MICROSCOPE in ``from_dict``.

    All non-UNASSIGNED roles are enforced as singletons by
    `HardwareConfig.set_camera_role()` — assigning a role to a new slot
    clears it from any other slot.
    """
    UNASSIGNED = "unassigned"
    NEEDLE_X = "needle_x"     # Side cam looking down the X axis (sees Y/Z)
    NEEDLE_Y = "needle_y"     # Side cam looking down the Y axis (sees X/Z)
    MICROSCOPE = "microscope" # Behind an objective lens, viewing the plate


# v7.4.x: roles that may only be held by a single camera slot at a time.
SINGLETON_CAMERA_ROLES: frozenset[CameraRole] = frozenset({
    CameraRole.NEEDLE_X,
    CameraRole.NEEDLE_Y,
    CameraRole.MICROSCOPE,
})


# Maximum live cameras tracked simultaneously by CameraManager (v7.3.3).
# Kept in sync with the `max_cams = 3` constant used by the Hardware
# Setup mini-cards. Bumping this means bumping the default
# `camera_roles` list length below.
MAX_LIVE_CAMERAS = 3


# ═══════════════════════════════════════════════════════════════════
# Camera Configuration (v7.3.0)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class CameraConfig:
    """
    Camera calibration configuration for autocalibration.

    Stores the camera spec, objective magnification, active resolution,
    and computed or user-overridden micron-per-pixel scale. Also stores
    the XY offset from camera center to needle tip (calibrated externally).
    """
    camera_spec: CameraSpec | None = None
    objective_magnification: float = 2.0   # Nikon Ti2-U: 2x, 4x, 10x, 20x
    active_resolution: tuple[int, int] = (916, 686)
    micron_per_pixel_override: float | None = None
    camera_to_needle_offset_um: tuple[float, float] = (0.0, 0.0)
    # v7.4.x: which objective is currently mounted on the microscope camera.
    # Used by `ObjectiveCalibrationCard` to look up the per-objective µm/px
    # stored in objectives.json. None = no objective selected yet.
    current_objective_name: str | None = None

    @property
    def computed_micron_per_pixel(self) -> float | None:
        """µm/px from sensor spec, resolution, and magnification."""
        if self.camera_spec is None:
            return None
        effective = self.camera_spec.effective_pixel_size_um(self.active_resolution)
        return effective / self.objective_magnification

    @property
    def micron_per_pixel(self) -> float | None:
        """Active µm/px scale (override takes priority over computed)."""
        if self.micron_per_pixel_override is not None:
            return self.micron_per_pixel_override
        return self.computed_micron_per_pixel

    @property
    def fov_um(self) -> tuple[float, float] | None:
        """Field of view in micrometers (width, height), or None."""
        scale = self.micron_per_pixel
        if scale is None:
            return None
        return (
            self.active_resolution[0] * scale,
            self.active_resolution[1] * scale,
        )

    def pixel_to_um(self, px: float) -> float:
        """Convert pixel distance to micrometers."""
        scale = self.micron_per_pixel
        if scale is None:
            raise ValueError("No micron/pixel calibration available")
        return px * scale

    def um_to_pixel(self, um: float) -> float:
        """Convert micrometers to pixel distance."""
        scale = self.micron_per_pixel
        if scale is None:
            raise ValueError("No micron/pixel calibration available")
        return um / scale

    def to_dict(self) -> dict:
        return {
            "camera_spec": self.camera_spec.to_dict() if self.camera_spec else None,
            "objective_magnification": self.objective_magnification,
            "active_resolution": list(self.active_resolution),
            "micron_per_pixel_override": self.micron_per_pixel_override,
            "camera_to_needle_offset_um": list(self.camera_to_needle_offset_um),
            "current_objective_name": self.current_objective_name,
        }

    @classmethod
    def from_dict(cls, data: dict) -> CameraConfig:
        spec_data = data.get("camera_spec")
        return cls(
            camera_spec=CameraSpec.from_dict(spec_data) if spec_data else None,
            objective_magnification=data.get("objective_magnification", 2.0),
            active_resolution=tuple(data.get("active_resolution", [916, 686])),
            micron_per_pixel_override=data.get("micron_per_pixel_override"),
            camera_to_needle_offset_um=tuple(
                data.get("camera_to_needle_offset_um", [0.0, 0.0])
            ),
            current_objective_name=data.get("current_objective_name"),
        )


# ═══════════════════════════════════════════════════════════════════
# Pump Channel Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PumpChannelConfig:
    """
    Configuration for a single pump channel (P1, P2, or P3).

    Tracks what syringe is installed, which inks it can handle (multi-ink),
    printing mode, and the current fluid column state.
    All user-facing values are in µL.

    A pump with multiple inks will require ink swap sequences between
    objects that use different inks (configured via InkSwapStrategy).
    """
    pump_id: str = "P1"
    syringe: SyringeSpec | None = None
    inks: list[InkSpec] = field(default_factory=list)  # All inks this pump can handle
    printing_mode: PrintingMode = PrintingMode.INCREMENTAL
    fluid_column: FluidColumn = field(default_factory=FluidColumn)
    enabled: bool = False  # True once syringe is assigned

    @property
    def ink(self) -> InkSpec | None:
        """Backward compat: return first ink (or None)."""
        return self.inks[0] if self.inks else None

    @ink.setter
    def ink(self, value: InkSpec | None):
        """Backward compat: set single ink (replaces list with one item)."""
        if value is None:
            self.inks = []
        else:
            self.inks = [value]

    @property
    def ink_names(self) -> list[str]:
        """List of ink names this pump can handle."""
        return [ink.name for ink in self.inks]

    @property
    def is_configured(self) -> bool:
        """True if this pump has a syringe assigned."""
        return self.syringe is not None and self.enabled

    @property
    def has_ink(self) -> bool:
        """True if at least one ink is assigned to this channel."""
        return len(self.inks) > 0 and self.is_configured

    def can_handle_ink(self, ink_name: str) -> bool:
        """Check if this pump is configured to handle a specific ink."""
        return ink_name in self.ink_names

    def add_ink(self, ink: InkSpec) -> None:
        """Add an ink to this pump's capability list (no duplicates)."""
        if ink.name not in self.ink_names:
            self.inks.append(ink)

    def remove_ink(self, ink_name: str) -> None:
        """Remove an ink from this pump's capability list."""
        self.inks = [i for i in self.inks if i.name != ink_name]

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
            "inks": [ink.to_dict() for ink in self.inks],
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

        # Load inks — support both new "inks" list and old single "ink"
        inks = []
        if "inks" in data:
            for ink_data in data["inks"]:
                inks.append(InkSpec.from_dict(ink_data))
        elif data.get("ink"):
            inks.append(InkSpec.from_dict(data["ink"]))

        mode = PrintingMode(data.get("printing_mode", "incremental"))
        fc_data = data.get("fluid_column", {})
        first_ink = inks[0] if inks else None
        fluid_column = FluidColumn(
            oil_volume_uL=fc_data.get("oil_volume_uL", 0.0),
            buffer_volume_uL=fc_data.get("buffer_volume_uL", 0.0),
            ink_volume_uL=fc_data.get("ink_volume_uL", 0.0),
            dead_volume_uL=fc_data.get("dead_volume_uL", 2.0),
            ink_spec=first_ink,
        )
        return cls(
            pump_id=data.get("pump_id", "P1"),
            syringe=syringe,
            inks=inks,
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

    # ── v7.5.x: Global pump timing (configured on Hardware Setup → Pump) ──
    # Settle dwell (s) applied BEFORE and AFTER every *discrete* pump
    # actuation (prime, aspirate, dispense, pick/place push-pull, needle-prep
    # oil/wash/buffer, DISPENSE commands) so the fluid/pressure settles and the
    # workflow does not advance to the next step until the pump has finished.
    # NOT applied to the streamed per-segment print path or manual jog.
    pump_settle_time_s: float = 0.0
    # Pre-flow prime duration (s). Printing workflows dispense a prime of
    # volume = flow × this time just before the print path. (Default matches
    # the legacy Quick Print _PREFLOW_S constant.)
    pump_prime_time_s: float = 0.25

    # ── Well plate ────────────────────────────────────────────────
    plate_format: int = 24  # 6, 12, 24, 48, 96, 384 — legacy field, still
                            # honored when `plate_name` is empty.

    # ── v7.4.5: Custom parametric plates ──────────────────────────
    # When non-empty, takes precedence over `plate_format`. Resolved via
    # `WellPlate.load(cfg.active_plate_key)` to pick up either a standard
    # (int) or a user-saved design (file under config/hardware/plates/user/).
    plate_name: str = ""

    # ── Ink library (persisted across sessions) ───────────────────
    ink_library: dict[str, InkSpec] = field(default_factory=dict)

    # ── v7.5.x: Reagent locations (ink name → well/sub-well names) ─
    # The canonical "go-to" locations for each reagent, picked in
    # Hardware Setup → Ink. Wells may be rosette sub-wells (e.g. "A1.a").
    # Invariant: one reagent per well (a well appears in at most one
    # ink's list). Used to auto-fill the per-print Well Setup.
    ink_locations: dict[str, list[str]] = field(default_factory=dict)

    # ── Rosette library ───────────────────────────────────────────
    rosette_library: dict[str, RosetteInsert] = field(default_factory=dict)

    # ── Buffer ink ────────────────────────────────────────────────
    buffer_ink_name: str | None = None

    # ── v7.2.4: Needle channel → pump mapping ────────────────────
    needle_channel_pump_map: dict[int, str] = field(default_factory=dict)
    # Maps channel index (0-based) → pump_id
    # Single-channel needle: {0: "P1"}
    # Multi-channel: {0: "P1", 1: "P2", 2: "P3"}

    # ── v7.2.8: Ink swap strategy for single-pump multi-ink ──────
    ink_swap_strategy: InkSwapStrategy = field(default_factory=InkSwapStrategy)

    # ── v7.3.0: Camera configuration for autocalibration ──────────
    camera_config: CameraConfig = field(default_factory=CameraConfig)

    # ── v7.4.4: Per-camera workflow roles ─────────────────────────
    # One entry per CameraManager slot (MAX_LIVE_CAMERAS). The
    # Calibration page workflow tabs resolve which physical camera to
    # mount by looking up the index with the matching role.
    camera_roles: list[CameraRole] = field(
        default_factory=lambda: [CameraRole.UNASSIGNED] * MAX_LIVE_CAMERAS
    )

    # ── Metadata ──────────────────────────────────────────────────
    config_name: str = "Untitled Setup"
    notes: str = ""

    # ══════════════════════════════════════════════════════════════
    #  v7.2.4: PUMP-INK MAPPING PROPERTIES (S3.1)
    # ══════════════════════════════════════════════════════════════

    @property
    def pump_ink_map(self) -> dict[str, list[str]]:
        """
        Get pump → ink_names mapping for quick lookup.

        Returns dict like {"P1": ["Hydrogel A", "MSC Cells"], "P2": ["Buffer"], "P3": []}
        Only includes enabled pumps.
        """
        result = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled:
                result[pid] = pcfg.ink_names
            else:
                result[pid] = []
        return result

    @property
    def ink_pump_map(self) -> dict[str, list[str]]:
        """
        Get ink_name → [pump_ids] reverse lookup.

        Returns dict like {"Hydrogel A": ["P1"], "MSC Cells": ["P1", "P2"]}
        An ink may be handled by multiple pumps.
        """
        result: dict[str, list[str]] = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled:
                for ink_name in pcfg.ink_names:
                    result.setdefault(ink_name, []).append(pid)
        return result

    @property
    def unassigned_inks(self) -> list[str]:
        """Get list of ink names in the library not assigned to any pump."""
        assigned = set(self.ink_pump_map.keys())
        return [name for name in self.ink_library if name not in assigned]

    # ══════════════════════════════════════════════════════════════
    #  v7.5.x: REAGENT LOCATIONS (ink name → wells)
    # ══════════════════════════════════════════════════════════════

    @property
    def well_reagent_map(self) -> dict[str, str]:
        """Reverse lookup: ``well_name -> ink_name``.

        One reagent per well (the invariant maintained by
        ``assign_wells_to_ink``). If stale data ever holds a well under
        two inks, the last one encountered wins.
        """
        result: dict[str, str] = {}
        for ink_name, wells in self.ink_locations.items():
            for w in wells:
                result[w] = ink_name
        return result

    def assign_wells_to_ink(
        self,
        ink_name: str,
        wells: list[str],
        *,
        replace: bool = False,
    ) -> None:
        """Assign ``wells`` to ``ink_name`` as reagent locations.

        Enforces one-reagent-per-well: each well in ``wells`` is first
        removed from any *other* ink's list. When ``replace`` is True the
        ink's existing list is cleared first (so it becomes exactly
        ``wells``); otherwise ``wells`` are appended (de-duped).
        """
        wells = [w for w in dict.fromkeys(wells) if w]  # de-dup, drop blanks
        # Drop these wells from every other ink (one-reagent-per-well).
        for other, lst in list(self.ink_locations.items()):
            if other == ink_name:
                continue
            kept = [w for w in lst if w not in wells]
            if kept:
                self.ink_locations[other] = kept
            else:
                self.ink_locations.pop(other, None)

        existing = [] if replace else list(self.ink_locations.get(ink_name, []))
        merged = list(dict.fromkeys(existing + wells))
        if merged:
            self.ink_locations[ink_name] = merged
        else:
            self.ink_locations.pop(ink_name, None)

    def clear_ink_location(self, ink_name: str) -> None:
        """Remove all reagent-location wells for an ink."""
        self.ink_locations.pop(ink_name, None)

    def unassign_well(self, well_name: str) -> None:
        """Remove ``well_name`` from whichever ink currently owns it."""
        for ink_name, lst in list(self.ink_locations.items()):
            if well_name in lst:
                kept = [w for w in lst if w != well_name]
                if kept:
                    self.ink_locations[ink_name] = kept
                else:
                    self.ink_locations.pop(ink_name, None)

    def rename_ink_location(self, old_name: str, new_name: str) -> None:
        """Move an ink's location list to a new ink name (on rename)."""
        if old_name == new_name or old_name not in self.ink_locations:
            return
        self.ink_locations[new_name] = self.ink_locations.pop(old_name)

    def _prune_ink_locations(self) -> None:
        """Drop locations for inks not in the library; de-dupe lists.

        Wells are NOT validated against the active plate here — the plate
        can change between sessions, and a reagent should reappear when
        its plate is reloaded. The UI only displays wells present in the
        current plate.
        """
        pruned: dict[str, list[str]] = {}
        for ink_name, wells in self.ink_locations.items():
            if ink_name not in self.ink_library:
                continue
            clean = [w for w in dict.fromkeys(wells) if w]
            if clean:
                pruned[ink_name] = clean
        self.ink_locations = pruned

    @property
    def enabled_pump_ids(self) -> list[str]:
        """List of pump IDs that are enabled (have syringes)."""
        return [pid for pid, p in self.pumps.items() if p.enabled and p.is_configured]

    # ── v7.4.5: Active plate key (custom name OR standard int) ───
    @property
    def active_plate_key(self) -> int | str:
        """The key to pass to `WellPlate.load()` for this config.

        Returns `plate_name` (str) when a custom plate is active, otherwise
        falls back to the legacy `plate_format` (int). Centralizes the
        precedence rule so downstream callers stop reaching into both
        fields.
        """
        return self.plate_name if self.plate_name else self.plate_format

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

        # Enabled pumps must have at least one ink assigned
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.syringe and not pcfg.has_ink:
                issues.append(f"{pid} is enabled but has no ink assigned")

        # Plate format / custom plate name (v7.4.5)
        if self.plate_name:
            # Custom plate — file must exist under user plates dir.
            from SupportClasses.WellPlate import USER_PLATES_DIR
            if not (USER_PLATES_DIR / f"{self.plate_name}.json").exists():
                issues.append(
                    f"Custom plate '{self.plate_name}' not found in "
                    f"{USER_PLATES_DIR}")
        elif self.plate_format not in PLATE_DEFINITIONS:
            issues.append(f"Invalid plate format: {self.plate_format}")

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
        """Backward compat: assign single ink (replaces all inks on pump)."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].ink = ink  # Uses the property setter

    def set_pump_inks(self, pump: str, inks: list[InkSpec]):
        """Assign multiple inks to a pump."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].inks = list(inks)

    def add_pump_ink(self, pump: str, ink: InkSpec):
        """Add an ink to a pump's capability list."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].add_ink(ink)

    def remove_pump_ink(self, pump: str, ink_name: str):
        """Remove an ink from a pump by name."""
        if pump not in self.pumps:
            raise ValueError(f"Unknown pump: {pump}")
        self.pumps[pump].remove_ink(ink_name)

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

    # ── v7.4.4: Camera role lookup ────────────────────────────────

    def camera_for_role(self, role: CameraRole) -> int | None:
        """First camera index whose `camera_roles[i]` matches `role`.

        Returns None if no camera has that role assigned. Used by the
        Calibration page workflow tabs to resolve which physical
        `CameraManager.cameras[i]` to mount.
        """
        for idx, r in enumerate(self.camera_roles):
            if r == role:
                return idx
        return None

    def set_camera_role(self, cam_idx: int, role: CameraRole) -> None:
        """Assign `role` to the camera at `cam_idx`.

        Out-of-range indices are ignored. The list is fixed-length
        (`MAX_LIVE_CAMERAS`); migrate/pad through `from_dict`.

        v7.4.x: roles in `SINGLETON_CAMERA_ROLES` are enforced as
        unique — assigning one to a new slot clears it from any other
        slot that holds it. UNASSIGNED is always allowed on multiple
        slots simultaneously. Legacy duplicates already in a loaded
        config are not retroactively normalized; cleanup only happens
        on user-driven role mutations.
        """
        if not (0 <= cam_idx < len(self.camera_roles)):
            return
        if role in SINGLETON_CAMERA_ROLES:
            for i, existing in enumerate(self.camera_roles):
                if i != cam_idx and existing == role:
                    self.camera_roles[i] = CameraRole.UNASSIGNED
        self.camera_roles[cam_idx] = role

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
        """Get the first pump ID that can handle the given ink, or None."""
        pumps = self.ink_pump_map.get(ink_name, [])
        return pumps[0] if pumps else None

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
            "version": "7.3.0",
            "config_name": self.config_name,
            "notes": self.notes,
            "needle": self.needle.to_dict() if self.needle else None,
            "pumps": {pid: p.to_dict() for pid, p in self.pumps.items()},
            # v7.5.x: global pump timing
            "pump_settle_time_s": self.pump_settle_time_s,
            "pump_prime_time_s": self.pump_prime_time_s,
            "plate_format": self.plate_format,
            "plate_name": self.plate_name,  # v7.4.5
            "ink_library": {name: ink.to_dict() for name, ink in self.ink_library.items()},
            # v7.5.x: reagent locations (ink name → wells); skip empty lists
            "ink_locations": {
                name: list(wells)
                for name, wells in self.ink_locations.items() if wells
            },
            "rosette_library": {
                name: r.to_dict() for name, r in self.rosette_library.items()
            },
            "buffer_ink_name": self.buffer_ink_name,
            # v7.2.4: Channel mapping (serialize int keys as strings for JSON)
            "needle_channel_pump_map": {
                str(ch): pid for ch, pid in self.needle_channel_pump_map.items()
            },
            # v7.2.8: Ink swap strategy
            "ink_swap_strategy": self.ink_swap_strategy.to_dict(),
            # v7.3.0: Camera config for autocalibration
            "camera_config": self.camera_config.to_dict(),
            # v7.4.4: Per-camera workflow roles (length = MAX_LIVE_CAMERAS).
            # Tolerate raw strings sneaking into the list via legacy
            # in-memory state — coerce each entry to its string value.
            "camera_roles": [
                (r.value if isinstance(r, CameraRole) else str(r))
                for r in self.camera_roles
            ],
            # v7.5.x: per-camera µm/px + rotation are NOT stored here — they
            # live in the per-machine CameraCalibrationStore
            # (config/hardware/camera_calibrations.json) so loading a saved
            # hardware-setup file can't wipe them.
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

        # v7.5.x: global pump timing (tolerate missing / bad values)
        def _nonneg_float(key, default):
            try:
                v = float(data.get(key, default))
                return v if v >= 0 else default
            except (TypeError, ValueError):
                return default
        config.pump_settle_time_s = _nonneg_float("pump_settle_time_s", 0.0)
        config.pump_prime_time_s = _nonneg_float("pump_prime_time_s", 0.25)

        # Plate format
        config.plate_format = data.get("plate_format", 24)
        # v7.4.5: custom plate name (takes precedence when non-empty)
        config.plate_name = data.get("plate_name", "") or ""

        # Ink library
        for name, ink_data in data.get("ink_library", {}).items():
            config.ink_library[name] = InkSpec.from_dict(ink_data)

        # v7.5.x: reagent locations (ink name → wells). Tolerate a single
        # string value by wrapping it in a list. Pruned against the library
        # below so a stale entry for a deleted ink doesn't linger.
        raw_locs = data.get("ink_locations", {}) or {}
        for name, wells in raw_locs.items():
            if isinstance(wells, str):
                wells = [wells]
            config.ink_locations[name] = [str(w) for w in (wells or []) if w]
        config._prune_ink_locations()

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

        # v7.2.8: Ink swap strategy
        if "ink_swap_strategy" in data:
            config.ink_swap_strategy = InkSwapStrategy.from_dict(data["ink_swap_strategy"])

        # v7.3.0: Camera config
        if "camera_config" in data:
            config.camera_config = CameraConfig.from_dict(data["camera_config"])

        # v7.4.4: Per-camera workflow roles. Missing field migrates to
        # all-UNASSIGNED; shorter lists are zero-padded; unknown values
        # fall back to UNASSIGNED rather than erroring out.
        # v7.5.x: the PLATE role was removed — migrate any legacy "plate"
        # string to MICROSCOPE (the microscope camera views the plate).
        raw_roles = data.get("camera_roles") or []
        roles: list[CameraRole] = []
        for entry in raw_roles[:MAX_LIVE_CAMERAS]:
            if entry == "plate":
                roles.append(CameraRole.MICROSCOPE)
                continue
            try:
                roles.append(CameraRole(entry))
            except ValueError:
                roles.append(CameraRole.UNASSIGNED)
        while len(roles) < MAX_LIVE_CAMERAS:
            roles.append(CameraRole.UNASSIGNED)
        config.camera_roles = roles

        # v7.5.x: per-camera µm/px + rotation are no longer stored in the
        # hardware config — they live in the per-machine CameraCalibrationStore
        # so a setup-file load can't wipe them. Any legacy "camera_calibrations"
        # key in older saved configs is simply ignored.

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
