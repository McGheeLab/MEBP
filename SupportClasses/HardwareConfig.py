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
    needle_bore_count,
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

    v7.5.x: MONITOR added — a camera resting on the stage that overviews
    the entire operation (needle, plate, and stage motion at once).
    Informational/live-view only; no workflow gates on it.

    v7.5.x (rotated rig): the two needle side cameras are now mounted
    **symmetric about the stage +X axis at +45° and −45°** and are
    interchangeable — which camera looks along which direction is
    determined by the stage-motion µm/px calibration (measured
    ``column_dir_deg``), not by the role. The roles are therefore
    presented to the operator as "Needle cam 1" / "Needle cam 2"
    (see ``needle_role_label``); the enum VALUES stay ``needle_x`` /
    ``needle_y`` because they are serialized in saved configs and the
    camera-calibration store's role→identity assignments.

    All non-UNASSIGNED roles are enforced as singletons by
    `HardwareConfig.set_camera_role()` — assigning a role to a new slot
    clears it from any other slot.
    """
    UNASSIGNED = "unassigned"
    NEEDLE_X = "needle_x"     # Needle cam 1 — side cam, ±45° about +X
    NEEDLE_Y = "needle_y"     # Needle cam 2 — side cam, ±45° about +X
    MICROSCOPE = "microscope" # Behind an objective lens, viewing the plate
    MONITOR = "monitor"       # Rests on the stage, overviews the operation


def needle_role_label(role: "CameraRole") -> str:
    """Operator-facing display name for a camera role.

    The two needle side cameras are physically interchangeable (symmetric
    ±45° about stage +X), so they are shown as numbered cams rather than
    the historical X-view/Y-view framing. Enum values are unchanged.
    """
    if role == CameraRole.NEEDLE_X:
        return "Needle cam 1"
    if role == CameraRole.NEEDLE_Y:
        return "Needle cam 2"
    try:
        return str(role.value).replace("_", " ").title()
    except Exception:
        return str(role)


# v7.4.x: roles that may only be held by a single camera slot at a time.
SINGLETON_CAMERA_ROLES: frozenset[CameraRole] = frozenset({
    CameraRole.NEEDLE_X,
    CameraRole.NEEDLE_Y,
    CameraRole.MICROSCOPE,
    CameraRole.MONITOR,
})


# Maximum live cameras tracked simultaneously by CameraManager (v7.3.3).
# The Hardware Setup mini-cards, CameraManager's default slot count, and
# the default `camera_roles` list length below all derive from this.
# v7.5.x: 3 → 4 to host the MONITOR overview camera. `from_dict` pads
# shorter legacy `camera_roles` lists with UNASSIGNED, so old configs
# migrate without touching settings.json.
MAX_LIVE_CAMERAS = 4


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
    # ── v7.5.x: Gentle-Z near the plate (configured on Common Print Settings) ──
    # Every needle motion near a print/work position eases in and out: the first
    # ``gentle_z_slow_dist_mm`` of a LIFT out of a print and the last
    # ``gentle_z_slow_dist_mm`` of a DESCENT back into position both run at
    # ``gentle_z_slow_speed_mm_s`` (mm/s), with the rest of the travel at the
    # fast retract/insert feedrate. The slow lift keeps a deposited bead from
    # peeling up with the needle; the slow descent is a controlled touch-down
    # instead of a crash-down. Applied to EVERY workflow via
    # ``ensure_retracted_to`` / ``safe_travel_to`` / the discrete ``MOVE_Z``
    # handler. One distance + one speed drives BOTH directions (their defaults
    # are identical). ``gentle_z_slow_dist_mm`` = 0 disables the eased motion
    # (single-speed — legacy). Pushed to StageController.set_retract_slow_lift /
    # set_descend_slow_final in set_hardware_config.
    gentle_z_slow_dist_mm: float = 1.0
    gentle_z_slow_speed_mm_s: float = 1.0
    # ── v7.5.x: Pump compliance / "pressure relief" (µL, PER PUMP) ──
    # The pressure-relief / backlash value is now an ABSOLUTE µL per pump,
    # measured by the Needle Location compliance calibration (= ½ the
    # aspirate-back volume) and stored per-machine in
    # ``device_profile.pump_compliance_uL`` (read via
    # ``StageController.pump_relief_uL(pump)``). It drives backlash compensation
    # (take-up on reversal + unload on stop), gated by the pump-jog-panel toggle
    # (``device_profile.backlash_comp_enabled``). It is NOT a HardwareConfig
    # field — the legacy ``pump_relief_percent`` + ``pump_relief_on_*`` toggles
    # were retired here (ignored on load; see ``from_dict``).

    # ── Well plate ────────────────────────────────────────────────
    plate_format: int = 24  # 6, 12, 24, 48, 96, 384 — legacy field, still
                            # honored when `plate_name` is empty.

    # ── v7.4.5: Custom parametric plates ──────────────────────────
    # When non-empty, takes precedence over `plate_format`. Resolved via
    # `WellPlate.load(cfg.active_plate_key)` to pick up either a standard
    # (int) or a user-saved design (file under config/hardware/plates/user/).
    plate_name: str = ""

    # ── v7.5.x: Selectable plate TYPE (product) ───────────────────
    # A plate type is a thin overlay on a base standard format (Corning glass
    # bottom, NEST plastic, …) — same XY grid, but its own Z offsets + its own
    # mosaic. The id (resolved via `PlateTypeStore`) becomes the
    # `active_plate_key` so per-plate stores auto-segregate. Empty = "generic"
    # (active key falls back to the bare int format). Takes precedence over
    # both `plate_name` and `plate_format`.
    plate_type_id: str = ""

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

    # ── v7.2.4: Needle bore → pump mapping ───────────────────────
    needle_channel_pump_map: dict[int, str] = field(default_factory=dict)
    # Maps BORE index (0-based) → pump_id
    # Single-bore needle: {0: "P1"}
    # Multi-bore: {0: "P1", 1: "P2", 2: "P3"}
    #
    # ⚠ The key name is FROZEN (it is in every saved setup on disk) but the word
    # "channel" in it is the banned overloaded term — this is a BORE index. The
    # base is 0, unlike the deprecated 1-based NeedleSpec.channel_pump_map.
    #
    # v7.9: for a genuinely multi-bore assembly the authority is
    # NeedleBore.pump_id (position in the list IS the index, so there is no base
    # to get wrong) and this dict is DERIVED from it — see
    # `resolved_bore_pump_map`. A single-bore config's hand-set map is never
    # touched, because there is nothing on the bore to derive it from.

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
        merged = self._drop_redundant_parents(
            list(dict.fromkeys(existing + wells)))
        if merged:
            self.ink_locations[ink_name] = merged
        else:
            self.ink_locations.pop(ink_name, None)

    @staticmethod
    def _drop_redundant_parents(wells: list[str]) -> list[str]:
        """Drop a flattened-rosette PARENT (e.g. ``"A2"``) from a reagent's
        well list when a sub-well of it (``"A2.a"``) is also present.

        Once a plain well becomes a rosette its bare parent name is no longer a
        real pickup well (``compile()`` replaces it with ``A2.a/b/c``), but
        ``ink_locations`` is append-only so the stale parent lingers — usually
        at index 0 — and pickup then resolves to the sub-well *centroid* instead
        of the intended sub-well. This is a pure string rule (a sub-well is any
        name containing ``"."``), so it is plate-independent and safe across
        plate switches: it only removes a parent that is redundant *within the
        same list*, never a well merely absent from the current plate.
        """
        sub_parents = {w.split(".", 1)[0] for w in wells if "." in w}
        return [w for w in wells if w not in sub_parents]

    def clear_ink_location(self, ink_name: str) -> None:
        """Remove all reagent-location wells for an ink."""
        self.ink_locations.pop(ink_name, None)

    def clear_all_ink_locations(self) -> None:
        """Remove EVERY reagent → well assignment (fresh ink landscape).

        A hard reset that also wipes any stale/hidden entries (e.g. a
        flattened-rosette parent no longer shown in the picker) so a new ink
        landscape starts with no residual members.
        """
        self.ink_locations = {}

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
            # De-dupe + drop a redundant flattened-rosette parent (e.g. "A2"
            # left over when a sub-well "A2.a" is also assigned). One-time
            # migration on load: legacy ["A2", "A2.a"] → ["A2.a"].
            clean = self._drop_redundant_parents(
                [w for w in dict.fromkeys(wells) if w])
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

        Precedence (v7.5.x): `plate_type_id` (a selectable plate product)
        → `plate_name` (a custom parametric design) → `plate_format` (the
        legacy standard int). Centralizes the rule so downstream callers and
        the per-plate stores (mosaic / template / well-training) stop reaching
        into the individual fields. A plate-type id resolves to its base
        format geometry inside `WellPlate.load`, so the returned key may be a
        string even though the plate is geometrically a standard format.
        """
        if self.plate_type_id:
            return self.plate_type_id
        return self.plate_name if self.plate_name else self.plate_format

    @property
    def geometry_plate_key(self) -> int | str:
        """The key to pass to `WellPlate.load()` for this config's GEOMETRY.

        Split from `active_plate_key` (v7.5.x): a plate TYPE (`plate_type_id`)
        carries identity (per-plate mosaic/calibration segregation) and Z
        offsets, but resolves to a PLAIN base-format geometry — so a layered
        custom design (e.g. a rosette added in the plate designer, saved under
        `plate_name`) would be dropped if geometry loaded from the type id.

        When both a plate type AND a custom design are set, the custom design
        wins for GEOMETRY (so its rosette sub-wells are honored everywhere)
        while `active_plate_key` keeps the type's identity (the type's Z
        offsets still apply — they key on `plate_type_id` directly). The custom
        file must exist; otherwise fall back to `active_plate_key` so a stale
        `plate_name` can't break the load.
        """
        if self.plate_type_id and self.plate_name:
            try:
                from SupportClasses.WellPlate import USER_PLATES_DIR
                if (USER_PLATES_DIR / f"{self.plate_name}.json").exists():
                    return self.plate_name
            except Exception:   # pragma: no cover - defensive
                pass
        return self.active_plate_key

    # ══════════════════════════════════════════════════════════════
    #  VALIDATION (v7.2.4 enhanced — S3.3)
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _needle_bore_issues(needle) -> list[str]:
        """Geometry issues for every bore of ``needle`` (v7.9).

        On a SINGLE-bore assembly the messages are byte-identical to the v7.6
        strings — no "Bore 1:" prefix, same wording, same order — because those
        exact strings are asserted and shown to the operator. The prefix appears
        only once there is more than one bore to disambiguate.

        Two bores claiming the same pump is flagged here rather than in the
        bore→pump map section: on a multi-bore assembly the bores' ``pump_id``
        IS the authority (the map is derived from it), so a collision there can
        never be repaired by editing the map.
        """
        issues: list[str] = []

        # A duck-typed stub (or a MagicMock) may not implement bores_resolved —
        # treat it as its own single bore, which is exactly the legacy shape.
        try:
            bores = list(needle.bores_resolved())
        except Exception:
            bores = []
        if not bores:
            bores = [needle]
        multi = len(bores) > 1

        for k, b in enumerate(bores):
            p = f"Bore {k + 1}: " if multi else ""
            b_id = getattr(b, "id_um", 0.0) or 0.0
            b_od = getattr(b, "od_um", 0.0) or 0.0
            # A NeedleBore stores mm; NeedleSpec stores inches. Both expose
            # `length_mm`, so read that and keep the message in mm either way.
            b_len = getattr(b, "length_mm", 0.0) or 0.0

            if getattr(b, "needle_type", "hypodermic") == "pulled_capillary":
                t_id = getattr(b, "tip_id_um", None)
                t_len = getattr(b, "tip_length_mm", None)
                t_od = getattr(b, "tip_od_um", None)
                if b_id <= 0:
                    issues.append(
                        f"{p}Capillary barrel inner Ø must be greater than 0 µm")
                if not t_id or t_id <= 0:
                    issues.append(
                        f"{p}Capillary tip inner Ø must be greater than 0 µm")
                elif b_id and t_id > b_id:
                    issues.append(
                        f"{p}Capillary tip inner Ø ({t_id:.1f} µm) cannot exceed "
                        f"the barrel inner Ø ({b_id:.0f} µm)")
                if not t_len or t_len <= 0:
                    issues.append(
                        f"{p}Capillary pulled-tip length must be greater than 0 mm")
                if b_len <= 0:
                    issues.append(
                        f"{p}Capillary barrel length must be greater than 0 mm")
                if b_od and b_id and b_od <= b_id:
                    issues.append(
                        f"{p}Capillary barrel outer Ø must exceed its inner Ø")
                if t_od and t_id and t_od <= t_id:
                    issues.append(
                        f"{p}Capillary tip outer Ø must exceed its inner Ø")
            elif not getattr(b, "gauge", None):
                # Unchanged legacy wording — the operator's next action is to
                # pick a gauge, so piling dimension errors on top is noise.
                issues.append(f"{p}No needle gauge selected")
            else:
                # v7.9: a gauge alone no longer implies sane dimensions, because
                # a backpack's bores are entered per bore rather than pulled
                # wholesale from one catalog entry.
                if b_id <= 0:
                    issues.append(f"{p}Needle inner Ø must be greater than 0 µm")
                if b_od and b_id and b_od <= b_id:
                    issues.append(f"{p}Needle outer Ø must exceed its inner Ø")
                if b_len <= 0:
                    issues.append(f"{p}Needle length must be greater than 0 mm")

        # Two bores fed by one pump is unbuildable — the pump can only push one
        # volume, so whichever bore was addressed second would be driven blind.
        claimed: dict[str, list[int]] = {}
        for k, b in enumerate(bores):
            pid = getattr(b, "pump_id", None)
            if pid:
                claimed.setdefault(str(pid).strip().upper(), []).append(k + 1)
        for pid, which in claimed.items():
            if len(which) > 1:
                issues.append(
                    f"{pid} feeds more than one bore: "
                    f"{', '.join(str(i) for i in which)}")

        return issues

    def validate(self) -> tuple[bool, list[str]]:
        """
        Check if the hardware setup is complete enough to proceed.

        Returns (is_valid, list_of_issues).
        """
        issues = []

        # Needle (v7.6: hypodermic gauge OR pulled glass capillary;
        # v7.9: validated PER BORE — a backpack may fuse bores of different
        # sizes, and a pulled capillary is no longer restricted to one bore
        # because the FORM and the taper are orthogonal axes.)
        n = self.needle
        if n is None:
            issues.append("No needle gauge selected")
        else:
            issues.extend(self._needle_bore_issues(n))

        # Pumps — at least one enabled with syringe
        configured_pumps = [p for p in self.pumps.values() if p.is_configured]
        if not configured_pumps:
            issues.append("At least one pump must be enabled with a syringe")

        # Enabled pumps must have at least one ink assigned
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.syringe and not pcfg.has_ink:
                issues.append(f"{pid} is enabled but has no ink assigned")

        # Plate type / custom plate name / format (v7.5.x precedence)
        if self.plate_type_id:
            # Selectable plate product — must resolve to a known base format.
            from SupportClasses.PlateTypeStore import get_store as _pt_store
            pt = _pt_store().get(self.plate_type_id)
            if pt is None:
                issues.append(
                    f"Plate type '{self.plate_type_id}' not found in the "
                    f"plate-type library")
            elif pt.base_format not in PLATE_DEFINITIONS:
                issues.append(
                    f"Plate type '{self.plate_type_id}' has an invalid base "
                    f"format: {pt.base_format}")
        elif self.plate_name:
            # Custom plate — file must exist under user plates dir.
            from SupportClasses.WellPlate import USER_PLATES_DIR
            if not (USER_PLATES_DIR / f"{self.plate_name}.json").exists():
                issues.append(
                    f"Custom plate '{self.plate_name}' not found in "
                    f"{USER_PLATES_DIR}")
        elif self.plate_format not in PLATE_DEFINITIONS:
            issues.append(f"Invalid plate format: {self.plate_format}")

        # Needle bore→pump mapping. v7.9: read the RESOLVED map, so a multi-bore
        # assembly whose bores declare their own pumps is not reported as
        # "0 mapped" (the stored mirror may not have been written yet), and
        # `bore_count` via the duck-safe accessor so a needle-like stub with no
        # `num_channels` cannot raise here.
        if self.needle is not None:
            n_bores = needle_bore_count(self.needle)
            bore_map = self.resolved_bore_pump_map()
            enabled_ids = self.enabled_pump_ids

            if n_bores > 0:
                if not enabled_ids:
                    issues.append(
                        f"Needle has {n_bores} bore(s) but "
                        f"no pumps are enabled")
                elif len(bore_map) != n_bores:
                    issues.append(
                        f"Needle has {n_bores} bore(s) but "
                        f"{len(bore_map)} mapped")
                else:
                    # Check mapped pumps are enabled
                    for bore_idx, pump_id in bore_map.items():
                        if pump_id not in enabled_ids:
                            issues.append(
                                f"Bore {bore_idx + 1} → {pump_id} "
                                f"but {pump_id} is not enabled")

                    # Check uniqueness
                    mapped_pumps: dict[str, list[int]] = {}
                    for bore_idx, pump_id in bore_map.items():
                        mapped_pumps.setdefault(pump_id, []).append(bore_idx)
                    for pump_id, bores in mapped_pumps.items():
                        if len(bores) > 1:
                            which = [str(b + 1) for b in bores]
                            issues.append(
                                f"{pump_id} mapped to multiple bores: "
                                f"{', '.join(which)}")

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
    #  v7.2.4: NEEDLE BORE → PUMP MAP HELPERS (S3.2)
    # ══════════════════════════════════════════════════════════════

    def resolved_bore_pump_map(self) -> dict[int, str]:
        """The truthful bore-index → pump_id map (0-based).

        For a genuinely multi-bore assembly (``needle.bores`` explicitly set)
        each bore carries its OWN ``pump_id``, so the map is DERIVED from the
        bore list — position in the list is the index. Otherwise the stored
        ``needle_channel_pump_map`` is returned unchanged.

        Deliberately conservative in two ways, both to protect operator data:

        * Only ``needle.bores`` being *explicitly set* counts as multi-bore.
          A pre-v7.9 needle synthesizes its bores from the flat fields and they
          carry no ``pump_id``, so deriving there would empty a hand-set map.
        * Even for a multi-bore assembly, a bore list where NO bore claims a
          pump falls back to the stored map — mid-migration the bores may not
          have been wired up yet, and emptying the map would break the very
          assignment the operator made in the UI.

        Derived ids are NORMALIZED (stripped + upper-cased) exactly as
        ``NeedleSpec.bore_for_pump`` normalizes its lookup — otherwise a bore
        storing ``"p1"`` resolves fine for the flow ceiling yet lands in this map
        verbatim, where every consumer compares against ``"P1"``: ``validate``
        reports the enabled pump as "not enabled", the un-normalized id is
        persisted by ``to_dict``, and ``PrintPlanOfAction``'s lookup misses.
        The STORED map is still returned byte-for-byte as saved (normalizing it
        would rewrite legacy files on load).
        """
        needle = self.needle
        bores = getattr(needle, "bores", None) if needle is not None else None
        if not bores or len(bores) < 2:
            return dict(self.needle_channel_pump_map)
        derived = {
            k: str(b.pump_id).strip().upper()
            for k, b in enumerate(bores)
            if getattr(b, "pump_id", None)
        }
        return derived or dict(self.needle_channel_pump_map)

    def sync_bore_pump_map_from_needle(self) -> bool:
        """Replace the stored map with :meth:`resolved_bore_pump_map`.

        Returns True when the stored map actually changed. Called on load so the
        live field agrees with the bores that own it; a single-bore config is a
        no-op.
        """
        derived = self.resolved_bore_pump_map()
        if derived == self.needle_channel_pump_map:
            return False
        self.needle_channel_pump_map = derived
        return True

    def set_channel_pump(self, channel_index: int, pump_id: str | None):
        """
        Assign a pump to a needle bore.

        .. deprecated:: 7.9
            Zero call sites repo-wide. On a multi-bore assembly the authority is
            ``NeedleBore.pump_id`` (see :meth:`resolved_bore_pump_map`), so
            writing the map directly can be silently overridden. Set the bore's
            ``pump_id`` instead. Kept only because a deletion is a separate
            decision from this change.

        Args:
            channel_index: 0-based BORE index
            pump_id: Pump ID ("P1", "P2", "P3") or None to clear
        """
        if pump_id is None:
            self.needle_channel_pump_map.pop(channel_index, None)
        else:
            if pump_id not in self.pumps:
                raise ValueError(f"Unknown pump: {pump_id}")
            self.needle_channel_pump_map[channel_index] = pump_id

    def get_channel_pump(self, channel_index: int) -> str | None:
        """Get pump ID assigned to a bore, or None.

        .. deprecated:: 7.9
            Zero call sites repo-wide. Reads the STORED map, which a multi-bore
            assembly overrides — use ``needle.bore(k).pump_id`` (or
            :meth:`resolved_bore_pump_map`) so a derived assignment is honored.
        """
        return self.needle_channel_pump_map.get(channel_index)

    def clear_channel_map(self):
        """Clear all bore→pump assignments.

        .. deprecated:: 7.9
            Zero call sites repo-wide. On a multi-bore assembly this clears only
            the derived MIRROR — the bores keep their ``pump_id`` and the map
            re-derives — so it does not do what its name promises.
        """
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
        Auto-assign bores to enabled pumps in order.

        Bore 0 → first enabled pump, bore 1 → second, etc.
        Only assigns up to min(bore_count, num_enabled_pumps).

        .. deprecated:: 7.9
            Zero call sites repo-wide, and it writes only the map — on a
            multi-bore assembly the bores' own ``pump_id`` wins, so the result
            can be silently discarded. Assign ``NeedleBore.pump_id`` instead.
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
        """Get the needle BORE index that carries the given ink, or None.

        .. deprecated:: 7.9
            Zero call sites repo-wide. Reads the stored map rather than
            :meth:`resolved_bore_pump_map`, so it can miss a derived multi-bore
            assignment.
        """
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
            # v7.5.x: gentle-Z near the plate (slow lift + slow descent)
            "gentle_z_slow_dist_mm": self.gentle_z_slow_dist_mm,
            "gentle_z_slow_speed_mm_s": self.gentle_z_slow_speed_mm_s,
            # v7.5.x: pressure relief / compliance is now per-pump µL, persisted
            # in device_profile.pump_compliance_uL (NOT here). The legacy
            # pump_relief_percent + pump_relief_on_* fields were retired.
            "plate_format": self.plate_format,
            "plate_name": self.plate_name,  # v7.4.5
            "plate_type_id": self.plate_type_id,  # v7.5.x
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
            # v7.2.4: bore→pump map (serialize int keys as strings for JSON).
            # v7.9: emitted from `resolved_bore_pump_map` so a multi-bore
            # assembly's saved map agrees with the bores that own it. Pure — a
            # single-bore config emits its stored map verbatim, byte-identically.
            "needle_channel_pump_map": {
                str(ch): pid
                for ch, pid in self.resolved_bore_pump_map().items()
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
        # v7.5.x: gentle-Z near the plate (dist 0 disables; speed kept positive)
        config.gentle_z_slow_dist_mm = _nonneg_float("gentle_z_slow_dist_mm", 1.0)
        config.gentle_z_slow_speed_mm_s = _nonneg_float(
            "gentle_z_slow_speed_mm_s", 1.0)
        # v7.5.x: pressure relief / compliance moved to per-pump µL
        # (device_profile.pump_compliance_uL) + a backlash-comp toggle. The
        # legacy `pump_relief_percent` / `pump_relief_on_*` / the older absolute
        # `pump_relief_volume_uL` keys are IGNORED here (discarded on load) — the
        # per-pump value now comes from the Needle Location compliance
        # calibration, not from this config.

        # Plate format
        config.plate_format = data.get("plate_format", 24)
        # v7.4.5: custom plate name (takes precedence when non-empty)
        config.plate_name = data.get("plate_name", "") or ""
        # v7.5.x: selectable plate type/product id (takes precedence over both)
        config.plate_type_id = data.get("plate_type_id", "") or ""

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

        # v7.2.4: Needle bore→pump map (JSON keys are strings → int)
        raw_map = data.get("needle_channel_pump_map", {})
        config.needle_channel_pump_map = {
            int(ch): pid for ch, pid in raw_map.items()
        }
        # v7.9: a multi-bore assembly's bores own the mapping, so reconcile the
        # live field once here — otherwise a stale saved map (e.g. written by an
        # older build, or by the GUI before the bores were wired) would keep
        # disagreeing with the bore that actually feeds each pump. A single-bore
        # config is untouched.
        config.sync_bore_pump_map_from_needle()

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
        # v7.6: a pulled capillary has no gauge — never render "NoneG".
        needle_txt = "?"
        if self.needle is not None:
            needle_txt = (getattr(self.needle, "display_label", None)
                          or (f"{self.needle.gauge}G" if self.needle.gauge
                              else "capillary"))
        return (
            f"HardwareConfig('{self.config_name}', "
            f"needle={needle_txt}, "
            f"plate={self.plate_format}, pumps=[{pumps}]{ch_map})"
        )
